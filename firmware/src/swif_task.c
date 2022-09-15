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
 
#include "cd1030.h"
#include "swif_task.h"
#include "udp_task.h"
#include "osif.h"
#include "shared_mem.h"
#include "wdt_task.h"
#include "dba_task.h"
#include "charger_task.h"

#ifdef USE_FEATURE_RTX_VIEWER
__attribute__((section(".bss.os.thread.cb")))
thread_tcb_t swif_thread_tcb;
#else
static thread_tcb_t swif_thread_tcb;
#endif /* USE_FEATURE_RTX_VIEWER */

__attribute__((section("ARM_LIB_STACK")))
static uint64_t swif_thread_stk[SWIF_TASK_STACK_SIZE];

static thread_id_t thread_swif;

static const thread_attrs_t swif_attr = {
    SWIF_TASK_NAME,
    osThreadDetached,
    &swif_thread_tcb,
    sizeof(swif_thread_tcb),
    &swif_thread_stk[0],
    SWIF_TASK_STACK_SIZE * sizeof(uint64_t),
    osPriorityNormal3,
    0U,
    0U
};

static uint32_t swif_sm_state = SWIF_STATE_ONPRESS;

__NO_RETURN static void swif_task(void *arg)
{
    status_t status = STATUS_ERROR;
    volatile uint32_t switch_index_s1 = 0U;
    volatile uint32_t switch_if_s1 = 0U;

    uint32_t switch_func_mode = 0U;
    shmem_block_swif_t *msg = NULL;
    udp_msg_queue_obj_t qmsg;
    dba_msg_queue_obj_t dmq;
    
    uint32_t ignore_irq2 = 0U;
    uint8_t sw_info = 0U;
    int32_t lc = 0;
    
    UNUSED_PARAM(arg);
    
    (void)osif_thread_wait_on_flag(SWIF_TASK_START_FLAG, OSIF_WAIT_ANY_FLAG, OSIF_WAIT_FOREVER);

	for(;; )
    {
        switch(swif_sm_state)
        {
            case SWIF_STATE_ONPRESS:

                cd1030_read_error();
                
                ignore_irq2 = 0U;
                NVIC_ClearPendingIRQ(PORTC_IRQn);
                INT_SYS_EnableIRQ(PORTC_IRQn);
            
                status = drv_swif_sem_acquire(5);
                if(status == STATUS_TIMEOUT)
                {
                    swif_sm_state = SWIF_STATE_ONPRESS;
                }
                else
                {
                    if(status == STATUS_SUCCESS)
                    {
                        clear_status_bit(STAT_VCU_SWIF_ERROR);
                        cd1030_initiate_swif_read();
                    
                        lc = osif_enter_critical();
                        switch_func_mode = cd1030_eval_switch((uint32_t *)&switch_index_s1, (uint32_t *)&switch_if_s1);
                        (void)osif_exit_critical(lc);
                    
                        if((switch_func_mode == SW_FUNC_MODE_LP_ONLY) || (switch_func_mode == SW_FUNC_MODE_SP_LP))
                        {
                            cd1030_start_lp_timer();
                        }

                        cd1030_set_irq_ctrl(switch_index_s1, switch_if_s1);
                        set_status_bit(STAT_VCU_KEY_EVENT);
                        swif_sm_state = SWIF_STATE_ONRELEASE;
                    }
                    else
                    {
                        swif_sm_state = SWIF_STATE_ONPRESS;
                        set_status_bit(STAT_VCU_SWIF_ERROR);
                    }
                }
                break;
                
            case SWIF_STATE_ONRELEASE:
                
                status = drv_swif_sem_acquire(LONG_PRESS_TIMER_TIMEOUT + 10U);
                clear_status_bit(STAT_VCU_KEY_EVENT);    
                if((status == STATUS_SUCCESS) || (status == STATUS_TIMEOUT))
                {
                    INT_SYS_DisableIRQ(PORTC_IRQn);

                    if(switch_func_mode == SW_FUNC_MODE_SP_ONLY)
                    {
                        sw_info = (uint8_t)cd1030_exec_cb(switch_index_s1, switch_if_s1, SWIF_CB_TYPE_SP);
                    }
                    else if(switch_func_mode == SW_FUNC_MODE_LP_ONLY)
                    {
                        if(cd1030_is_lp_timer_running() != 1U)
                        {
                            sw_info = (uint8_t)cd1030_exec_cb(switch_index_s1, switch_if_s1, SWIF_CB_TYPE_LP);
                        }
                        
                        ignore_irq2 = 1U;
                    }
                    else if(switch_func_mode == SW_FUNC_MODE_SP_LP)
                    {
                        if(cd1030_is_lp_timer_running() == 1U)
                        {
                            cd1030_stop_lp_timer();
                            sw_info = (uint8_t)cd1030_exec_cb(switch_index_s1, switch_if_s1, SWIF_CB_TYPE_SP);
                        }
                        else
                        {
                            sw_info = (uint8_t)cd1030_exec_cb(switch_index_s1, switch_if_s1, SWIF_CB_TYPE_LP);
                            ignore_irq2 = 1U;
                        }
                    }

                    if(sw_info > 0U)
                    {
                        lc = osif_enter_critical();
                        msg = (shmem_block_swif_t *)shmem_alloc_block(SHMEM_POOL_TYPE_SWIF);
                        (void)osif_exit_critical(lc);
                        if(msg != NULL)
                        {
                            msg->udp_msg.cmd = MSG_ID_SWIF_S32_IMX;
                            msg->udp_msg.len = 1U;
                            msg->sw_info = sw_info;
                            
                            qmsg.msg = (void *)msg;
                            qmsg.msg_id = msg->udp_msg.cmd;
                            qmsg.msg_len = msg->udp_msg.len;
                            
                            (void)osif_msg_queue_send(udp_msg_queue, &qmsg, 1U, 5U);
                            
                            if(chg_get_connection_state() == 1U)
                            {
                                dmq.msg_id = DBA_WAKE_DISPLAY_MSG;                                
                                (void)osif_msg_queue_send(dba_msg_queue, &dmq, 0U, 0U);
                            }
                        }
                        sw_info = 0U;
                                               
#ifdef USE_FEATURE_SWIF_CHEAT_CODES
                        if(cd1030_open_cheatbox(switch_index_s1) > 0U)
                        {
                            lc = osif_enter_critical();
                            msg = (shmem_block_swif_t *)shmem_alloc_block(SHMEM_POOL_TYPE_SWIF);
                            if(msg != NULL)
                            {
                                msg->udp_msg.cmd = MSG_ID_SWIF_S32_IMX;
                                msg->udp_msg.len = 1U;
                                msg->sw_info = sw_info;
                                
                                qmsg.msg = (void *)msg;
                                qmsg.msg_id = msg->udp_msg.cmd;
                                qmsg.msg_len = msg->udp_msg.len;
                                
                                (void)osif_msg_queue_send(udp_msg_queue, &qmsg, 1U, 0U);
                            }
                            
                            sw_info = 0U; 
                            (void)osif_exit_critical(lc);   
                        }
#endif /* USE_FEATURE_SWIF_CHEAT_CODES */
                    }

                    if(ignore_irq2 == 1U)
                    {
                        /* Ignore second IRQ */
                        INT_SYS_EnableIRQ(PORTC_IRQn);
                        status = drv_swif_sem_acquire(SW_IRQ_2_TIMEOUT);
                        while(status == STATUS_TIMEOUT)
                        {
                            cd1030_poll_switches();
                            status = drv_swif_sem_acquire(SW_IRQ_2_TIMEOUT);
                        }
                        DEV_ASSERT(status == STATUS_SUCCESS);
                    }
                }
               
                swif_sm_state = SWIF_STATE_ONPRESS;
                cd1030_enable_all_irqs();

                break;
                    
            default:
                __NOP();
                break;
        }
        
        cd1030_poll_switches();
	}
}

thread_id_t swif_task_get_id(void)
{
    return thread_swif;
}

void swif_task_create(void)
{
    uint32_t param = NULL;
    
    (void)shmem_create_pool(SHMEM_POOL_TYPE_SWIF);
    
    thread_swif = osif_thread_create(swif_task, &param, &swif_attr);
    DEV_ASSERT(thread_swif != NULL);
}
