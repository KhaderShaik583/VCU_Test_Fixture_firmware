 /*
 * 
 * ULTRAVIOLETTE AUTOMOTIVE CONFIDENTIAL
 * ______________________________________
 * 
 * [2017] - [2018] Ultraviolette Automotive Pvt. Ltd.
 * All Rights Reserved.
 * 
 * NOTICE: All information contained herein is, and remains the 
 * property of Ultraviolette Automotive Pvt. Ltd. and its suppliers, if 
 * any. The intellectual and technical concepts contained herein are 
 * proprietary to Ultraviolette Automotive  and its suppliers and may 
 * be covered by U.S. and Foreign Patents, patents in process,  and 
 * are protected by trade secret or copyright law. Dissemination of 
 * this information or reproduction of this material is strictly 
 * forbidden unless prior written permission is obtained from 
 * Ultraviolette Automotive Pvt. Ltd.
 * 
 *
 * Author : Rishi F. [010]
 *


 */

#include <stdio.h>

#include "mqtt_task.h"
#include "lte_atc.h"

#include "osif.h"
#include "lpuart_driver.h" 
#include "bcu_task.h"
#include "lte_task.h"
#include "gps_task.h"
#include "csq_task.h"

#include "rtx_os.h"

#ifdef USE_NATIVE_LTE
__attribute__((section(".bss.os.thread.cb")))
osRtxThread_t mqtt_thread_tcb;

osThreadId_t thread_mqtt;

__attribute__((section("ARM_LIB_STACK")))
uint64_t mqtt_thread_stk[MQTT_TASK_STACK_SIZE];

static const osThreadAttr_t mqtt_task_attr = {
    /* MQTT task attributes */
    MQTT_TASK_NAME,
    osThreadDetached,
    &mqtt_thread_tcb,
    sizeof(mqtt_thread_tcb),
    &mqtt_thread_stk[0],
    sizeof(mqtt_thread_stk),
    osPriorityNormal,
    0U
};

static uint8_t rxbuffer[MAX_IOT_BUF_LEN];
static uint32_t rx_len = 0U;

static char iot_buf[MAX_IOT_BUF_LEN];
static char gps_data[512];

static uint32_t session = 0U;
static uint32_t dev_lockup = 0U;
static osMessageQueueId_t lte_task_observer;
static osMessageQueueId_t gps_task_observer;
static lte_data_tx_t *data = NULL;

static uint32_t pdp_deactivation_count = 0U;

static uint32_t mqtt_task_state = MQTT_WAIT_AT_INIT_COMPLETE_STATE;

static lte_ret_codes mqtt_pub(uint32_t msg_len)
{
    lte_ret_codes rc = LTE_AT_WAIT;
    
    static uint32_t millis = 0U;

    uint32_t i = 0U;
    uint32_t err = 0U;
    uint8_t cme_err = 0U;
    uint8_t rsp;
    uint8_t qmt_stat_urc;

    
    rc = lte_send_data_blocking((uint8_t *)iot_buf, msg_len, rxbuffer, &rx_len);
    if(rc == LTE_AT_OK)
    {
        /* Check Response */
        rsp = lte_at_parse_response(LTE_AT_CMD_QMTPUB, rxbuffer, rx_len);
        qmt_stat_urc = is_qmtstat_urc(rxbuffer, rx_len);
        err = is_atc_error(rxbuffer, rx_len);
        
        if(qmt_stat_urc == 5U)
        {
            /* Received QMTRECV */
            /* TBD */
        }
        
        /*
            If rsp is Packet sent successfully, and there was a QMTSTAT URC 
            which is usually Peer closed connection, then, close the connection and 
            re-open.
        */
        if((rsp == 0U) && (qmt_stat_urc == 1U))
        {
            session++;
            rc = LTE_AT_ERR;
           
        }else if((rsp == 0U) && (qmt_stat_urc == 0U))
        {
            /*
                If rsp is Packet sent successfully, and there was no QMTSTAT URC 
                then send next publish request.
            */
            
            rc = LTE_AT_OK;
        }
        else if((rsp == 2U) && (qmt_stat_urc == 0U))
        {
            /*
                If rsp is Packet failed to send, and there was no QMTSTAT URC 
                then, ignore
            */
            
            rc = LTE_AT_OK;
        }
        else if((rsp == 2U) && (qmt_stat_urc == 1U))
        {
            /*
                If rsp is Packet failed to send, and there was QMTSTAT URC peer closed
                then, close the connection and re-open.
            */
            
            rc = LTE_AT_ERR;
        }
        else if(err == 1U)
        {
            rc = LTE_AT_ERR;
        }
        else
        {
            
            rc = LTE_AT_OK;
        }
    }
    else if(rc == LTE_AT_ERR)
    {
        err = is_atc_error(rxbuffer, rx_len);
        qmt_stat_urc = is_qmtstat_urc(rxbuffer, rx_len);
        is_atc_cme_error(&cme_err, rxbuffer, rx_len);

        /*
            AT Command error. Reset EC25 if QMSTAT is peer closed connection
        */
        if((qmt_stat_urc == 1U) || (err == 1U))
        {
            rc = LTE_AT_ERR;
        }
    }
    else if(rc == LTE_AT_HW_ERR)
    {
        /* Framing Error on UART */
        /* Reset UART */
        LPUART_DRV_Deinit(SYS_LTE_UART_IF);
        OSIF_TimeDelay(100);
        lte_uart_reconfig();
    }

    return rc;
}

static void mqtt_task(void *arg)
{
    lte_ret_codes rc = LTE_AT_OK;
    uint8_t rsp = 0U;
    osStatus_t status;
    msgqueue_obj_t msg;
    uint8_t qmt_stat_urc;
    uint8_t err;
    
    static uint32_t bus = 0U;
    static uint32_t msg_len = 0U;
    static uint32_t msg_count = 0U;
    int32_t rssi = 0U;
    uint32_t receive_idx = 0U;
    volatile osThreadId_t tid_gps;
    
    tid_gps = gps_task_get_id();
    while(true)
    {
        switch(mqtt_task_state)
        {
            case MQTT_WAIT_AT_INIT_COMPLETE_STATE:
                osThreadFlagsWait(0x0001, osFlagsWaitAll, osWaitForever);
                osThreadFlagsClear(0x0001);

                mqtt_task_state = MQTT_OPEN_NW_CONNECTION_STATE;            
                break;
                
            case MQTT_OPEN_NW_CONNECTION_STATE:
                /* Initialize & open MQTT network connection */
                lte_atc_sem_acquire(OSIF_WAIT_FOREVER);
            
                rc = lte_atc_at_start(LTE_AT_CMD_QMTOPEN, rxbuffer, &rx_len);
                if(rc == LTE_AT_OK)
                {
                    /* Check Response */
                    rsp = lte_at_parse_response(LTE_AT_CMD_QMTOPEN, rxbuffer, rx_len);
                    if(rsp == 0U)
                    {
                        mqtt_task_state = MQTT_CONNECT_BROKER_STATE;
                    }
                    else if(rsp == LTE_AT_MQTT_ID_OCCUPIED)
                    {
                        mqtt_task_state = MQTT_DISCONN_BROKER_STATE;
                    }
                    else if(rsp == LTE_AT_FAILED_PDP_ACTIVATION)
                    {
                        mqtt_task_state = MQTT_DEACT_PDP_STATE;
                    }
                    else if(rsp == LTE_AT_NW_CONN_ERROR)
                    {
                        mqtt_task_state = MQTT_DEACT_PDP_STATE;
                    }
                    else
                    {
                        mqtt_task_state = MQTT_OPEN_NW_CONNECTION_STATE;   
                    }    
                    
                }
                else if(rc == LTE_AT_HW_ERR)
                {
                    __NOP();
                }
                
                lte_atc_sem_release();
                break;
            
            case MQTT_CONNECT_BROKER_STATE:
                /* Connect to MQTT broker */
                lte_atc_sem_acquire(OSIF_WAIT_FOREVER);
            
                rc = lte_atc_at_start(LTE_AT_CMD_QMTCONN, rxbuffer, &rx_len);
                if(rc == LTE_AT_OK)
                {
                    /* Check Response */
                    rsp = lte_at_parse_response(LTE_AT_CMD_QMTCONN, rxbuffer, rx_len);
                    if(rsp == 0U)
                    {
                        mqtt_task_state = MQTT_SETUP_SUBSCRIPTION_STATE;
                    }
                    else
                    {
                        OSIF_TimeDelay(MQTT_TASK_DELAY);
                        __NOP();
                    }    
                    
                }
                else if(rc == LTE_AT_ERR)
                {
                    mqtt_task_state = MQTT_OPEN_NW_CONNECTION_STATE;
                }
                else if(rc == LTE_AT_HW_ERR)
                {
                    mqtt_task_state = MQTT_OPEN_NW_CONNECTION_STATE;
                }
                
                lte_atc_sem_release();
                break;
            
            case MQTT_SETUP_SUBSCRIPTION_STATE:
                /* Subscribe to topic */
                
                lte_atc_sem_acquire(OSIF_WAIT_FOREVER); 
                rc = lte_atc_at_start(LTE_AT_CMD_QMTSUB, rxbuffer, &rx_len);
                if(rc == LTE_AT_OK)
                {
                    /* Check Response */
                    rsp = lte_at_parse_response(LTE_AT_CMD_QMTSUB, rxbuffer, rx_len);
                    err = is_atc_error(rxbuffer, rx_len);
                    if(rsp == 0U)
                    {
                        mqtt_task_state = MQTT_WAIT_BCU_DATA_DONE_STATE;
                    }
                    else if(rsp == 4U)
                    {
                        
                    }
                    else if(err == 1U)
                    {
                        mqtt_task_state = MQTT_OPEN_NW_CONNECTION_STATE;
                    }
                    else
                    {
                        __NOP();
                    }    
                    
                }
                else if(rc == LTE_AT_ERR)
                {
                    mqtt_task_state = MQTT_OPEN_NW_CONNECTION_STATE;
                }
                else if(rc == LTE_AT_HW_ERR)
                {
                    mqtt_task_state = MQTT_OPEN_NW_CONNECTION_STATE;
                }
                
                lte_atc_sem_release();
                break;
                
            case MQTT_WAIT_BCU_DATA_DONE_STATE:

#ifndef USE_FEATURE_OVRD_BCU
                osThreadFlagsWait(0x0002, osFlagsWaitAll, osWaitForever);
#else
                osThreadFlagsWait(0x0002, osFlagsWaitAll, 250U);
#endif /* USE_FEATURE_OVRD_BCU */ 
            
                osThreadFlagsClear(0x0002);

                lte_atc_sem_acquire(OSIF_WAIT_FOREVER);
            
#ifndef USE_FEATURE_OVRD_BCU
                
                data = get_lte_data(bus);
                csq_read_rssi(&rssi);
                gps_read_data(&gps_data[0]);
            

                msg_len = sprintf(&iot_buf[0], "%d,%d,%d,%d,%3.3f,%3.3f,%3.3f,%3.3f,%3.3f,%3.3f,%3.3f,%d,%d,%3.3f,%3.3f,%3.3f,%d,%s", \
                                  dev_lockup, pdp_deactivation_count, bus, msg_count, data->vdsg, data->vchg, data->idsg, data->ichg, data->dsgq, data->chgq,  \
                                  data->timestmp, data->statusl, data->statush, data->soc, data->fettmp, data->max_cell_temp, rssi, gps_data);
#else
                msg_len = sprintf(&iot_buf[0], "%d,%d,%d,%d,%3.3f,%3.3f,%3.3f,%3.3f,%3.3f,%3.3f,%3.3f,%d,%d,%3.3f,%3.3f,%3.3f,%d,%s", \
                      dev_lockup, pdp_deactivation_count, bus, msg_count, 50.3, 50.3, 0.0f, 0.0f, 0.0f, 0.0f,  \
                      data->timestmp, 0, 0, 50.0f, 25.0f, 25.0f, rssi, gps_data);
#endif /* USE_FEATURE_OVRD_BCU */        
            
                msg_count++;
                bus = bus + 1U;
                if(bus >= 3U)
                {
                    bus = 0U;
                }
                
                if(msg_len > 0U)
                {
                    mqtt_task_state = MQTT_PUBLISH_CMD_STATE;
                }
                lte_atc_sem_release();
                break;
            
            case MQTT_PUBLISH_CMD_STATE:
                /* Send QMTPUB command */
                lte_atc_sem_acquire(OSIF_WAIT_FOREVER);
                rc = lte_atc_at_start(LTE_AT_CMD_QMTPUB, rxbuffer, &rx_len);
                if(rc == LTE_AT_OK)
                {
                    mqtt_task_state = MQTT_PUBLISH_DATA_STATE;
                }
                else if(rc == LTE_AT_ERR)
                {
                    session++;
                    mqtt_task_state = MQTT_OPEN_NW_CONNECTION_STATE;
                    lte_atc_sem_release();
                }
                else if(rc == LTE_AT_HW_ERR)
                {
                    mqtt_task_state = MQTT_OPEN_NW_CONNECTION_STATE;
                    lte_atc_sem_release();
                }

                break;
                
             case MQTT_PUBLISH_DATA_STATE:
                /* Publish data */
                rc = mqtt_pub(msg_len);
             
                /* Check Response */
                rsp = lte_at_parse_response(LTE_AT_CMD_QMTPUB, rxbuffer, rx_len);
                qmt_stat_urc = is_qmtstat_urc(rxbuffer, rx_len);
                err = is_atc_error(rxbuffer, rx_len);
                
                if((rsp == 0U) && (qmt_stat_urc == 1U))
                {
                    session++;
                    mqtt_task_state = MQTT_OPEN_NW_CONNECTION_STATE;
                   
                }else if((rsp == 0U) && (qmt_stat_urc == 0U))
                {
                    /*
                        If rsp is Packet sent successfully, and there was no QMTSTAT URC 
                        then send next publish request.
                    */
                    
                    mqtt_task_state = MQTT_CHK_RX_BUFFER_STATE;
                }
                else if(err == 1U)
                {
                    mqtt_task_state = MQTT_OPEN_NW_CONNECTION_STATE;
                }
                if(rc == LTE_AT_ERR)
                {
                    mqtt_task_state = MQTT_OPEN_NW_CONNECTION_STATE;
                }
                else if(rc == LTE_AT_OK)
                {                   
                    mqtt_task_state = MQTT_CHK_RX_BUFFER_STATE;
                }
                else if(rc == LTE_AT_ROP)
                {
                    /* reset state variables */
                    mqtt_task_state = MQTT_WAIT_AT_INIT_COMPLETE_STATE;
                    
                    /* Notify LTE task to reboot EC25 */
                    msg.msg_buf[0] = 0xA9;
                    msg.idx = 0U;
                    status = osMessageQueuePut(lte_task_observer, &msg, 0U, 0U);
                    dev_lockup++;
                    
                }
                else if(rc == LTE_AT_HW_ERR)
                {
                    mqtt_task_state = MQTT_OPEN_NW_CONNECTION_STATE;
                }
                else
                {
                    __NOP();
                    OSIF_TimeDelay(MQTT_TASK_DELAY);
                }
                
                lte_atc_sem_release();
                break;
             
             case MQTT_CHK_RX_BUFFER_STATE:
                 lte_atc_sem_acquire(OSIF_WAIT_FOREVER);
             
                 rc = lte_atc_at_start(LTE_AT_CMD_QMTRECV, rxbuffer, &rx_len);
                 if(rc == LTE_AT_OK)
                 {
                     rsp = lte_at_parse_response(LTE_AT_CMD_QMTRECV, rxbuffer, rx_len);
                     if(rsp == 5U)
                     {
                         mqtt_task_state = MQTT_WAIT_BCU_DATA_DONE_STATE;
                     }
                     else
                     {
                        if(rsp == 14U)
                        {
                            receive_idx = 0U;
                            mqtt_task_state = MQTT_READ_RX_BUFFER_STATE;
                        }
                        else if(rsp == 16U)
                        {
                            receive_idx = 1U;
                            mqtt_task_state = MQTT_READ_RX_BUFFER_STATE;
                        }
                        else if(rsp == 18U)
                        {
                            receive_idx = 2U;
                            mqtt_task_state = MQTT_READ_RX_BUFFER_STATE;
                        }
                        else if(rsp == 20U)
                        {
                            receive_idx = 3U;
                            mqtt_task_state = MQTT_READ_RX_BUFFER_STATE;
                        }
                        else if(rsp == 22U)
                        {
                            receive_idx = 4U;
                            mqtt_task_state = MQTT_READ_RX_BUFFER_STATE;
                        }
                        else
                        {
                            mqtt_task_state = MQTT_WAIT_BCU_DATA_DONE_STATE;
                        }
                     }
                 }
                 else
                 {
                     mqtt_task_state = MQTT_WAIT_BCU_DATA_DONE_STATE;
                 }
                 
                 lte_atc_sem_release();
                 break;
                 
             case MQTT_READ_RX_BUFFER_STATE:
                 lte_atc_sem_acquire(OSIF_WAIT_FOREVER);
                 lte_atc_form_qmt_read_cmd(receive_idx);
                 rc = lte_atc_at_start(LTE_AT_CMD_QMTRECV_READ, rxbuffer, &rx_len);
                 if(rc == LTE_AT_OK)
                 {
                     /* Index 25, 26 Msg Len */
                     /* Msg start index 29 */
                     rsp = lte_at_parse_response(LTE_AT_CMD_QMTRECV_READ, rxbuffer, rx_len);
                     receive_idx = 0U;
                 }
                 mqtt_task_state = MQTT_WAIT_BCU_DATA_DONE_STATE;
                 lte_atc_sem_release();
                 break;
             
             case MQTT_DISCONN_BROKER_STATE:
                /* Disconnect from MQTT broker */
                lte_atc_sem_acquire(OSIF_WAIT_FOREVER);
            
                rc = lte_atc_at_start(LTE_AT_CMD_QMTDISC, rxbuffer, &rx_len);
                if(rc == LTE_AT_OK)
                {
                    /* Check Response */
                    rsp = lte_at_parse_response(LTE_AT_CMD_QMTDISC, rxbuffer, rx_len);
                    if(rsp == 0U)
                    {
                        mqtt_task_state = MQTT_OPEN_NW_CONNECTION_STATE;
                    }
                    else
                    {
                        OSIF_TimeDelay(MQTT_TASK_DELAY);
                        __NOP();
                    }    
                }
                else if(rc == LTE_AT_ERR)
                {
                    mqtt_task_state = MQTT_OPEN_NW_CONNECTION_STATE;
                }
                else if(rc == LTE_AT_HW_ERR)
                {
                    mqtt_task_state = MQTT_OPEN_NW_CONNECTION_STATE;
                }
                lte_atc_sem_release();
                break;
                
             case MQTT_DEACT_PDP_STATE:
                /* De-activate PDP */
                lte_atc_sem_acquire(OSIF_WAIT_FOREVER);
                pdp_deactivation_count++;
                rc = lte_atc_at_start(LTE_AT_CMD_PDP_DEACT, rxbuffer, &rx_len);
                if(rc == LTE_AT_OK)
                {
                    mqtt_task_state = MQTT_ACT_PDP_STATE;
                }
                lte_atc_sem_release();
                break;
                
             case MQTT_ACT_PDP_STATE:
                /* De-activate PDP */
                lte_atc_sem_acquire(OSIF_WAIT_FOREVER);
                rc = lte_atc_at_start(LTE_AT_CMD_ACT_PDP, rxbuffer, &rx_len);
                if(rc == LTE_AT_OK)
                {
                    mqtt_task_state = MQTT_OPEN_NW_CONNECTION_STATE;
                }
                lte_atc_sem_release();
                break;
                
             case MQTT_QUERY_PDP_STATE:
                /* Query PDP */
                lte_atc_sem_acquire(OSIF_WAIT_FOREVER);
                rc = lte_atc_at_start(LTE_AT_CMD_QUERY_PDP_STATE, rxbuffer, &rx_len);
                if(rc == LTE_AT_OK)
                {
                    rsp = lte_at_parse_response(LTE_AT_CMD_QUERY_PDP_STATE, rxbuffer, rx_len);
                    if(rsp == 1U)
                    {
                        mqtt_task_state = MQTT_PUBLISH_CMD_STATE;
                    }
                    else
                    {
                        mqtt_task_state = MQTT_DEACT_PDP_STATE;
                    }
                }
                lte_atc_sem_release();
                break;
        }
    }
}

static void register_observers(void)
{
    /* Get the LTE Task message Q */
    lte_task_observer = lte_task_get_queue();
}

osThreadId_t mqtt_task_get_id(void)
{
    return thread_mqtt;
}

void mqtt_task_create(void)
{
    uint32_t param = NULL;
    
    register_observers();

    thread_mqtt = osThreadNew(mqtt_task, &param, &mqtt_task_attr);
    DEV_ASSERT(thread_mqtt != NULL);

}
#endif /* USE_NATIVE_LTE */
