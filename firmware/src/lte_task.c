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

#include "lte_task.h"
#include "lte_atc.h"
#include "osif.h"
#include "mqtt_task.h" 
#include "gps_task.h"
#include "csq_task.h"
#include "pins_driver.h"

#include "rtx_os.h"

#ifdef USE_NATIVE_LTE
__attribute__((section(".bss.os.thread.cb")))
osRtxThread_t lte_thread_tcb;

osThreadId_t thread_lte;

__attribute__((section("ARM_LIB_STACK")))
uint64_t lte_thread_stk[LTE_TASK_STACK_SIZE];

static const osThreadAttr_t lte_task_attr = {
    /* LTE task attributes */
    LTE_TASK_NAME,
    osThreadDetached,
    &lte_thread_tcb,
    sizeof(lte_thread_tcb),
    &lte_thread_stk[0],
    sizeof(lte_thread_stk),
    osPriorityNormal,
    0U
};

static uint8_t rxbuffer[MAX_AT_CMD_LEN];
static uint32_t rx_len = 0U;

static osMessageQueueId_t lte_msgqueue_id;
static msgqueue_obj_t msgqueue_obj;

static uint32_t lte_task_state = LTE_WAIT_EC25_RDY_URC_STATE;

osMessageQueueId_t lte_task_get_queue(void)
{
    return lte_msgqueue_id;
}

static uint32_t wait_rdy_sig(uint32_t reinit_uart)
{
    uint32_t lc = 0U;
    uint32_t rx_len = 7U;
    uint8_t rdy_buffer[7];
    uint32_t num_rx_bytes = 0U;
    uint32_t ret = 0U;
    
    if(reinit_uart == 1U)
    {
        LPUART_DRV_Deinit(SYS_LTE_UART_IF);
        lte_uart_reconfig();
    }
    
    LPUART_DRV_ReceiveData(SYS_LTE_UART_IF, (uint8_t *)rdy_buffer, rx_len); 
    
    rdy_buffer[2] = 'Y';
    rdy_buffer[3] = 'D';
    rdy_buffer[4] = 'R';
    
    lc = osKernelLock();
    /* Wait for [CR][LF]RDY[CR][LF] */
    while(LPUART_DRV_GetReceiveStatus(SYS_LTE_UART_IF, &num_rx_bytes) == STATUS_BUSY)
    {
        OSIF_TimeDelay(50);
    }
    osKernelRestoreLock(lc);
    
    if((rdy_buffer[2] == 'R') &&
       (rdy_buffer[3] == 'D') &&
       (rdy_buffer[4] == 'Y'))
    {
            OSIF_TimeDelay(1000);
            ret = 1U;
    }
    
    return ret;
}

static void lte_task(void *arg)
{   
    lte_ret_codes rc = LTE_AT_OK;
    uint8_t rsp = 0U;
    osStatus_t status;
    uint32_t uart_reset = 0U;
    volatile osThreadId_t tid_mqtt;
    volatile osThreadId_t tid_gps;
    volatile osThreadId_t tid_csq;
    static uint32_t cmd_idx = 0U;

    /* Pre-process AT command list */
    init_at_cmd_list();
    
    
    osThreadFlagsWait(0x0001, osFlagsWaitAny, osWaitForever);
    tid_mqtt = mqtt_task_get_id();
    tid_gps = gps_task_get_id();
    tid_csq = csq_task_get_id();
    
    
    while(true)
    {
        switch(lte_task_state)
        {
            case LTE_WAIT_EC25_RDY_URC_STATE:
                /* Wait for RDY URC from EC25 when it boots up */
                /* Until then stay in the same state */
                rc = wait_rdy_sig(uart_reset);
                if(rc == 1U)
                {
                    uart_reset = 0U;
                    lte_atc_init();
                    
                    lte_task_state = LTE_CMD_INIT_STATE;
                }
                break;
                
            case LTE_CMD_INIT_STATE:
                /* Send initialization command set */
                rc = lte_atc_at_start(cmd_idx, rxbuffer, &rx_len);  
                if(rc == LTE_AT_OK)
                {
                    /* Check Response */
                    rsp = lte_at_parse_response(cmd_idx, rxbuffer, rx_len);
                    if(rsp == lte_atc_get_rsp(cmd_idx))
                    {
                        cmd_idx++;
                        
                        if(cmd_idx == LTE_AT_CMD_CONFIG_AUDIO_MODE)
                        {
                            OSIF_TimeDelay(2000);
                        }
                        
                        if(cmd_idx == LTE_AT_CMD_QMTOPEN)
                        {
                            /* Break out on QMTOPEN as it will be handled in MQTT task */
                            lte_task_state = LTE_WAIT_MSG_STATE; 
                            
                            /* Notify MQTT task to continue */
                            osThreadFlagsSet(tid_mqtt,  0x0001);
                            
                            /* Notify GPS task to continue */
                            osThreadFlagsSet(tid_gps,  0x0001);
                            
                            /* Notify CSQ task to continue */
                            osThreadFlagsSet(tid_csq,  0x0001);
                            
                            cmd_idx = 0U;
                        }
                    }
                }
                break;
                
            case LTE_WAIT_MSG_STATE:
                /* Wait for messages in queue */
                status = osMessageQueueGet(lte_msgqueue_id, &msgqueue_obj, NULL, OSIF_WAIT_FOREVER);
                if(status == osOK)
                {
                    /* Process message */
                    switch(msgqueue_obj.msg_buf[0])
                    {
                        case 0xA9:
                            /* Unrecoverable error Reset EC25 */
                            PINS_DRV_SetPins(RESET_MCU_GPIO, (1U << RESET_MCU_PIN));
                            OSIF_TimeDelay(500);
                            PINS_DRV_ClearPins(RESET_MCU_GPIO, (1U << RESET_MCU_PIN));
                            OSIF_TimeDelay(500);
                        
                            lte_task_state = LTE_WAIT_EC25_RDY_URC_STATE;
                            uart_reset = 1U;
                            break;
                        
                        case 0xA7:
                            break;
                    }
                    
                }
                
                break;
        }
    }
}

osThreadId_t lte_task_get_id(void)
{
    return thread_lte;
}

void lte_task_create(void)
{
    uint32_t param = NULL;
    
    lte_msgqueue_id = osMessageQueueNew(LTE_MSGQUEUE_OBJECTS, sizeof(msgqueue_obj_t), NULL);
    DEV_ASSERT(lte_msgqueue_id != NULL);
  
    thread_lte = osThreadNew(lte_task, &param, &lte_task_attr);
    DEV_ASSERT(thread_lte != NULL);
    
    lte_atc_sem_create();

}

void led_task(void)
{
    /* TODO */
}
#endif /* USE_NATIVE_LTE */
