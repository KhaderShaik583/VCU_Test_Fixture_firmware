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

#include "csq_task.h"
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
osRtxThread_t csq_thread_tcb;

osThreadId_t thread_csq;

__attribute__((section("ARM_LIB_STACK")))
uint64_t csq_thread_stk[CSQ_TASK_STACK_SIZE];

static const osThreadAttr_t csq_task_attr = {
    /* MQTT task attributes */
    CSQ_TASK_NAME,
    osThreadDetached,
    &csq_thread_tcb,
    sizeof(csq_thread_tcb),
    &csq_thread_stk[0],
    sizeof(csq_thread_stk),
    osPriorityNormal,
    0U
};

static uint8_t rx_buf[MAX_CSQ_BUF_LEN];
static uint8_t rssi_char[4];
static uint32_t rssi = 0U;
static int32_t rssi_dbm = 0;
static uint32_t rx_len = 0U;

int rssi_dbm_lut[32] = {
    -113,                                                                                                                                                                                          
    -111,                                                                                                                                                                                          
    -109,                                                                                                                                                                                          
    -107,                                                                                                                                                                                          
    -105,                                                                                                                                                                                          
    -103,                                                                                                                                                                                          
    -101,                                                                                                                                                                                          
    -99,                                                                                                                                                                                           
    -97,
    -95,                                                                                                                                                                                           
    -93,                                                                                                                                                                                           
    -91,                                                                                                                                                                                         
    -89,                                                                                                                                                                                           
    -87,                                                                                                                                                                                           
    -85,                                                                                                                                                                                           
    -83,                                                                                                                                                                                           
    -81,                                                                                                                                                                                           
    -79,                                                                                                                                                                                           
    -77,                                                                                                                                                                                           
    -75,                                                                                                                                                                                           
    -73,                                                                                                                                                                                           
    -71,                                                                                                                                                                                           
    -69,                                                                                                                                                                                           
    -67,                                                                                                                                                                                           
    -65,                                                                                                                                                                                           
    -63,                                                                                                                                                                                           
    -61,                                                                                                                                                                                           
    -59,                                                                                                                                                                                           
    -57,                                                                                                                                                                                           
    -55,                                                                                                                                                                                           
    -53,                                                                                                                                                                                           
    -51
};

void csq_read_rssi(int32_t *rssi_value)
{
    *rssi_value = rssi_dbm;
}

int csq_atoi(char* str) 
{ 
    int res = 0; 

    for (int i = 0; str[i] != '\0'; ++i) 
    {
        res = res * 10 + str[i] - '0'; 
    }
  
    return res; 
} 

static void csq_at_rssi(void)
{
    uint8_t rsp = 0U;
    lte_ret_codes rc;
    uint32_t rssi_char_len = 0U;
    
    rc = lte_atc_at_start(LTE_AT_CMD_CSQ, rx_buf, &rx_len);

    if(rc == LTE_AT_OK)
    {
        /* Check Response */
        rsp = lte_at_parse_response(LTE_AT_CMD_CSQ, rx_buf, rx_len);
        if(rsp == 1U)
        {
            /* parse the RSSI value */
            if(rx_buf[8] != ',')
            {
                rssi_char[0] = rx_buf[8];
                rssi_char_len++;
            }
            else
            {
                rssi_char[0] = '\0';
            }
            
            if(rx_buf[9] != ',')
            {
                rssi_char[1] = rx_buf[9];
                rssi_char_len++;
            }
            else
            {
                rssi_char[1] = '\0';
            }
            
            if(rx_buf[10] != ',')
            {
                rssi_char[2] = rx_buf[10];
                rssi_char_len++;
            }
            else
            {
                rssi_char[2] = '\0';
            }
            
            rssi_char[3] = '\0';
            
            rssi = csq_atoi((char *)rssi_char);
            
            if(rssi < 32U)
            {
                rssi_dbm = rssi_dbm_lut[rssi];
            }
            else if(rssi == 99U)
            {
                rssi_dbm = 0U;
            }
            
        }
    }
    else
    {
        lte_atc_sem_release();
    }  
}

static void csq_task(void *arg)
{
    osStatus_t status;   
    uint32_t lc = 0U;

    osThreadFlagsWait(0x0001, osFlagsWaitAll, osWaitForever);
    osThreadFlagsClear(0x0001);
    
    while(true)
    {
        OSIF_TimeDelay(CSQ_TASK_DELAY);
        
        lte_atc_sem_acquire(OSIF_WAIT_FOREVER);

        csq_at_rssi();
        
        lte_atc_sem_release();
             
    } 
}

osThreadId_t csq_task_get_id(void)
{
    return thread_csq;
}

void csq_task_create(void)
{
    uint32_t param = NULL;
    
    thread_csq = osThreadNew(csq_task, &param, &csq_task_attr);
    DEV_ASSERT(thread_csq != NULL);

}
#endif /* USE_NATIVE_LTE */

