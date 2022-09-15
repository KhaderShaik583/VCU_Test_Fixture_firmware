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

#include "gps_task.h"
#include "lte_atc.h"
#include "pins_driver.h"
#include "osif.h"
#include "lpuart_driver.h" 
#include "bcu_task.h"
#include "lte_task.h"
#include "lte_atc.h"

#include "rtx_os.h"

#ifdef USE_NATIVE_LTE
__attribute__((section(".bss.os.thread.cb")))
osRtxThread_t gps_thread_tcb;

osThreadId_t thread_gps;

__attribute__((section("ARM_LIB_STACK")))
uint64_t gps_thread_stk[GPS_TASK_STACK_SIZE];

static const osThreadAttr_t gps_task_attr = {
    /* MQTT task attributes */
    GPS_TASK_NAME,
    osThreadDetached,
    &gps_thread_tcb,
    sizeof(gps_thread_tcb),
    &gps_thread_stk[0],
    sizeof(gps_thread_stk),
    osPriorityNormal,
    0U
};

static char gps_buf[MAX_GPS_BUF_LEN];
static uint32_t rx_len = 0U;

static uint32_t gps_cmd = 0U;
static uint32_t gps_state = GPS_STATE_WAIT_MSG;

void gps_read_data(char *gps_data)
{
    uint32_t len = strlen(gps_buf);
    
    if((gps_buf[2] == '+') && (gps_buf[3] == 'Q'))
    {
        for(uint32_t i = 14U, j = 0U; i < len; i++, j++)
        {
            gps_data[j] = gps_buf[i];
            if(gps_buf[i] == 0x0AU)
            {
                gps_data[j] = 0x0U;
                gps_data[j - 1] = 0x0U;
                break;
            }    
            
        }
        
        /*gps_cmd = (gps_cmd + 1U) % 2U;*/
    }

}

static void gps_nmea_rmc_fix(void)
{
    uint8_t rsp = 0U;
    lte_ret_codes rc;
    
    rc = lte_atc_at_start(LTE_AT_CMD_QGPSNMEA_RMC, (uint8_t *)gps_buf, &rx_len);

    if(rc == LTE_AT_OK)
    {
        /* Check Response */
        rsp = lte_at_parse_response(LTE_AT_CMD_QGPSNMEA_RMC, (uint8_t *)gps_buf, rx_len); 
    }
    else if(rc == LTE_AT_ERR)
    {
        lte_atc_sem_release();
    }  
}

static void gps_nmea_vtg_fix(void)
{
    uint8_t rsp = 0U;
    lte_ret_codes rc;
    
    rc = lte_atc_at_start(LTE_AT_CMD_QGPSNMEA_VTG, (uint8_t *)gps_buf, &rx_len);

    if(rc == LTE_AT_OK)
    {
        /* Check Response */
        rsp = lte_at_parse_response(LTE_AT_CMD_QGPSNMEA_VTG, (uint8_t *)gps_buf, rx_len); 
    }
    else if(rc == LTE_AT_ERR)
    {
        lte_atc_sem_release();
    }  
}

static void gps_task(void *arg)
{
    osStatus_t status;   
    uint32_t lc = 0U;

    osThreadFlagsWait(0x0001, osFlagsWaitAll, osWaitForever);
    osThreadFlagsClear(0x0001);
    
    while(true)
    {
        OSIF_TimeDelay(GPS_TASK_DELAY);
        
        lte_atc_sem_acquire(OSIF_WAIT_FOREVER);

        switch(gps_cmd)
        {
            case 0:
                gps_nmea_rmc_fix();
                break;
            case 1:
                gps_nmea_vtg_fix();
                break;
        }
        
        lte_atc_sem_release();
             
    } 
}

osThreadId_t gps_task_get_id(void)
{
    return thread_gps;
}


void gps_task_create(void)
{
    uint32_t param = NULL;
    
    thread_gps = osThreadNew(gps_task, &param, &gps_task_attr);
    DEV_ASSERT(thread_gps != NULL);

}
#endif /* USE_NATIVE_LTE */

