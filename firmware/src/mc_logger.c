 /*
 * 
 * ULTRAVIOLETTE AUTOMOTIVE CONFIDENTIAL
 * ______________________________________
 * 
 * [2019] - [2020] Ultraviolette Automotive Pvt. Ltd.
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
 * Author : Rishi F. [011]
 *
 */
 
#include "mc_logger.h"
#include "osif.h"
#include "drv_MOT_CO.h"
#include "lpuart_driver.h" 
#include "mc_task.h"

#ifdef USE_MC_FIFO

#ifdef USE_FEATURE_RTX_VIEWER
__attribute__((section(".bss.os.thread.cb")))
thread_tcb_t mc_logger_thread_tcb;
#else
static thread_tcb_t mc_logger_thread_tcb;
#endif /* USE_FEATURE_RTX_VIEWER */

static thread_id_t thread_mc_logger;
osif_msg_queue_id_t mc_msg_log_queue; 

__attribute__((section("ARM_LIB_STACK")))
static uint64_t mc_logger_thread_stk[MC_STACK_SIZE];

static int32_t fault_count_lg = 0U;

typedef struct 
{
    uint32_t msg_id;
    store_msgs data;
}mc_log_queue_obj_t;

const thread_attrs_t mc_logger_controller_attr = {
    /* Motor controller task attributes */
    MC_LOGGER_TASK_NAME,
    osThreadDetached,
    &mc_logger_thread_tcb,
    sizeof(mc_logger_thread_tcb),
    &mc_logger_thread_stk[0],
    sizeof(mc_logger_thread_stk),
    osPriorityNormal3,
    0U,
    0U    
};

static void mc_logger_task(void *arg)
{
    osif_thread_wait_on_flag(MC_LOGGER_TASK_START_FLAG, OSIF_WAIT_ANY_FLAG, OSIF_WAIT_FOREVER);
    status_t status = STATUS_ERROR;
    
    for(;; )
    {
        status_t qstatus = STATUS_SUCCESS;
        mc_log_queue_obj_t tpdo_msg;
        
        qstatus = osif_msg_queue_recv(mc_msg_log_queue, &tpdo_msg, NULL, 5U);        
        if(qstatus == osOK)   
        {
            /* Write message to UART */
            switch(tpdo_msg.msg_id)
            {
                case MC_LOG_SAVE_CMD:
                    if(tpdo_msg.data.msgID == 0x81)
                    {
                        fault_count_lg = fault_count_lg + 1U;
                    }
                    
                    status = LPUART_DRV_SendDataBlocking(SYS_LTE_UART_IF, (uint8_t *)&tpdo_msg.data, sizeof(store_msgs), 30U);
                    DEV_ASSERT(status == STATUS_SUCCESS);
                    break;
            }
        }
    }
}

thread_id_t mc_logger_task_get_id(void)
{
    return thread_mc_logger;
}

void mc_logger_task_create(void)
{
    uint32_t param = NULL;
    
    mc_msg_log_queue = osif_msg_queue_create(MC_MSGQUEUE_LOG_OBJECTS, sizeof(mc_log_queue_obj_t));
    DEV_ASSERT(mc_msg_log_queue != NULL);

    thread_mc_logger = osif_thread_create(mc_logger_task, &param, &mc_logger_controller_attr);
    DEV_ASSERT(thread_mc_logger != NULL);

}
#endif /* USE_MC_FIFO */
