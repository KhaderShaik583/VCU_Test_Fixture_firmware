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
 * Author : Rishi F. [011]
 *
 */
 
#include "bms_task.h"
#include "bms.h"
#include "osif.h"
#include "pins_driver.h"
#include "csec_driver.h"
#include "aes.h"
#include "bms.h"
#include "canfd_queue.h"
#include "init_msg.h"
#include "can_messenger_tx.h"
#include "can_messenger_rx.h"
#include "bms_can_if.h"
#include "wdt_task.h"

#define BMS_MSGQUEUE_OBJECTS        8 
#define ETH_HDR_AUX_INFO        (16U)

#ifdef USE_FEATURE_RTX_VIEWER
__attribute__((section(".bss.os.thread.cb")))
thread_tcb_t bms_thread_tcb;
#else
static thread_tcb_t bms_thread_tcb;
#endif /* USE_FEATURE_RTX_VIEWER */

static thread_id_t thread_bms;

osif_msg_queue_id_t bms_msg_queue; 
static uint32_t esr_1 = 0U;

static uint32_t charger_state = 0U;

__attribute__((section("ARM_LIB_STACK")))
static uint64_t bms_thread_stk[BMS_STACK_SIZE];

static canfd_queue_desc_t canfd_message[MAX_CANFD_LOGICAL_INTERFACES];

static const thread_attrs_t bms_attr = {
    BMS_TASK_NAME,
    osThreadDetached,
    &bms_thread_tcb,
    sizeof(bms_thread_tcb),
    &bms_thread_stk[0],
    BMS_STACK_SIZE * sizeof(uint64_t),
    osPriorityNormal3,
    0U,
    0U    
};

static void bms_send_can_fd_msg(uint32_t bus)
{
    if(QUEUE_SUCCESS == canfd_dequeue(&canfd_message[bus], (uint8_t)bus))
    {
        (void)can_msg_send_process(canfd_message[bus], bus);
    }
    else
    {
        /*
        If PC message is sent it results in Q empty as there
        is a possibility of the VCU message being missed and not being
        placed back into the queue.
        */
        
        __NOP();
    }
}

static void process_BMS_MSG_ASYNC_FET_OFF(void)
{
    canfd_queue_desc_t can_message;

    can_message.is_cyclic = 0;
    can_message.cycle_count = 0;
    can_message.msg_id = CAN_IF_MSG_DSG_CHG_FET_OFF;
    
    (void)canfd_enqueue(can_message, CANFD_LOGICAL_BUS0);
    
    bms_data_tx_timer_stop();
    
#ifdef USE_FEATURE_VCU_ON_DESK
    dbg_printf("I,0,BMS_MSG_ASYNC_FET_OFF\n\r");
#endif
}

static void process_BMS_MSG_ASYNC_FET_ON(void)
{
    canfd_queue_desc_t can_message;
    
    can_message.is_cyclic = 0;
    can_message.cycle_count = 0;
    can_message.msg_id = CAN_IF_MSG_DSG_CHG_FET_ON;
    
    (void)canfd_enqueue(can_message, CANFD_LOGICAL_BUS0); 
    
#ifdef USE_FEATURE_VCU_ON_DESK
    dbg_printf("I,0,BMS_MSG_ASYNC_FET_ON\n\r");
#endif
}

static void process_BMS_MSG_POST_FW_UPD_EVENT(void)
{
    canfd_queue_desc_t can_message;
    
    process_BMS_MSG_ASYNC_FET_OFF();
    canfd_queue_flush(0U);
    
    can_message.is_cyclic = 0;
    can_message.cycle_count = 0;
    can_message.msg_id = CAN_IF_MSG_BMS_FW_UPD_MSG;
    
    (void)canfd_enqueue(can_message, CANFD_LOGICAL_BUS0);     
}

static void process_BMS_MSG_PWR_MODE(uint8_t event)
{
    UNUSED_PARAM(event);
    
    canfd_queue_desc_t can_message;
    
    can_message.is_cyclic = 0;
    can_message.cycle_count = 0;
    can_message.msg_id = CAN_IF_MSG_BMS_SET_PWR_MODE;
    
    (void)canfd_enqueue(can_message, CANFD_LOGICAL_BUS0);     
}

static void process_BMS_MSG_FLUSH_CAN_MSG_QUEUE(void)
{
    canfd_queue_flush(0U);
}

static void process_BMS_MSG_KEEP_ALIVE_NO_DATA(void)
{
    canfd_queue_desc_t can_message;
    
    canfd_queue_flush(0U);
    
    can_message.is_cyclic = 1;
    can_message.cycle_count = 0;
    can_message.msg_id = CAN_IF_MSG_ID_BMS_STATUS;
    
    (void)canfd_enqueue(can_message, CANFD_LOGICAL_BUS0);  
}

static void process_BMS_MSG_CLEAR_TAMPER_STATE(void)
{
    canfd_queue_desc_t can_message;
    
    canfd_queue_flush(0U);
    
    can_message.is_cyclic = 0;
    can_message.cycle_count = 0;
    can_message.msg_id = CAN_IF_MSG_RST_TAMPER_SENTINEL;
    
    (void)canfd_enqueue(can_message, CANFD_LOGICAL_BUS0);      
}

static void bms_can_err_cb(uint8_t instance, flexcan_event_type_t eventType, flexcan_state_t *flexcanState)
{
    UNUSED_PARAM(flexcanState);
    
    if(eventType == FLEXCAN_EVENT_ERROR)
    {
        esr_1 = FLEXCAN_DRV_GetErrorStatus(instance);
    }
}

static void bms_pre_sm(void)
{
    volatile uint32_t tf = 0U;
    static uint32_t count = 0U;
    static uint32_t retry_attempts = 0U;
    
    while(tf != BMS_TASK_WAKE_NTF_FLAG)
    {
        tf = osif_thread_wait_on_flag(BMS_TASK_WAKE_NTF_FLAG, OSIF_WAIT_ANY_FLAG, 1U);
        can_fd_bms_receive(CANFD_LOGICAL_BUS0);
        
        count = count + 1U;
        if(count > 100U)
        {
            /* resend wake notification */
            (void)can_wake_msg_send(CANFD_LOGICAL_BUS0);
            count = 0U;
            retry_attempts = retry_attempts + 1U;
            if(retry_attempts > 5U)
            {
                /* Failure to communicate with pack */
                /* Either genuine failure or BMS under balancing which, if the */
                /* case, the pack is already awake */
                break;
            }
        }
    }
}

static void bms_sm(void)
{
    if(CANFD_LOGICAL_BUS_ENABLED == canfd_get_bus_en_state(CANFD_LOGICAL_BUS0))
    {
        /* Start transfers only if pack is detected during boot and s/w has enabled access */
        if((can_fd_get_rx_state(CANFD_LOGICAL_BUS0) == CANFD_STATE_INIT_RX) && 
           (FLEXCAN_DRV_GetTransferStatus(CAN_IF_BMS, CAN_BMS_TX_MAILBOX) == STATUS_SUCCESS))
        {
            /* If the SM is in CANFD_STATE_INIT_RX state, initiate a transfer */
            /* Also check if the TX mailbox is IDLE */
            if(canfd_message[CANFD_LOGICAL_BUS0].msg_id == CAN_IF_MSG_BMS_HEART_BEAT)
            {
                /* Log to UART */
                __NOP();
                PINS_DRV_TogglePins(LED_RED_GPIO, (1U << LED_RED_PIN));
            }
            
            /* Trigger a RX mailbox before sending */
            can_fd_bms_receive(CANFD_LOGICAL_BUS0);
            bms_send_can_fd_msg(CANFD_LOGICAL_BUS0);
            osif_time_delay(40);
        }

        can_fd_bms_receive(CANFD_LOGICAL_BUS0);
    }    
    else
    {
        osif_time_delay(40);
    }
}

static void bms_queue_proc(void)
{
    status_t qstatus = STATUS_SUCCESS;
    bms_msg_queue_obj_t bms_mq;
    
    qstatus = osif_msg_queue_recv(bms_msg_queue, &bms_mq, NULL, 5U); 
    if(qstatus == STATUS_SUCCESS)  
    {
        switch(bms_mq.msg_id)
        {
            case BMS_MSG_ASYNC_FET_OFF:
                process_BMS_MSG_ASYNC_FET_OFF();
                break;

            case BMS_MSG_ASYNC_FET_ON:
                process_BMS_MSG_ASYNC_FET_ON();
                break;
            
            case BMS_MSG_POST_FW_UPD_EVENT:
                process_BMS_MSG_POST_FW_UPD_EVENT();
                break;
            
            case BMS_MSG_PWR_MODE:
                charger_state = bms_mq.data;
                process_BMS_MSG_PWR_MODE(charger_state);                
                break;
            
            case BMS_MSG_FLUSH_CAN_MSG_QUEUE:
                process_BMS_MSG_FLUSH_CAN_MSG_QUEUE();
                break;
            
            case BMS_MSG_KEEP_ALIVE_NO_DATA: 
                process_BMS_MSG_KEEP_ALIVE_NO_DATA();
                break;
            
            case BMS_MSG_CLEAR_TAMPER_STATE:
                process_BMS_MSG_CLEAR_TAMPER_STATE();
                break;
            
            default:
                __NOP();
                break;
        }
    }    
}

thread_id_t bms_task_get_id(void)
{
    return thread_bms;
}

uint32_t bms_get_charger_state(void)
{
    return charger_state;
}

__NO_RETURN static void bms_task(void *arg)
{
    UNUSED_PARAM(arg);

    canfd_queue_init();
    init_fast_boot_mesg();
    fet_sw_init();
    bms_state_init();
    
    /* Install callback for CAN bus error events */
    FLEXCAN_DRV_InstallErrorCallback(CAN_IF_BMS, bms_can_err_cb, NULL);
    
    (void)osif_thread_wait_on_flag(BMS_TASK_START_FLAG, OSIF_WAIT_ANY_FLAG, OSIF_WAIT_FOREVER);
    
    (void)can_wake_msg_send(CANFD_LOGICAL_BUS0);
    
#ifdef USE_FEATURE_VCU_ON_DESK
    osif_time_delay(500U);
#endif
    
    bms_pre_sm();
    can_fd_bms_receive(CANFD_LOGICAL_BUS0);
    
	for(;; )
	{
        bms_sm();
        bms_queue_proc();
        
        can_fd_if_bms_err_process(&esr_1);
	}
}

void bms_task_create(void)
{
    uint32_t param = NULL;
    
    (void)shmem_create_pool(SHMEM_POOL_TYPE_BMS);
    
    bms_msg_queue = osif_msg_queue_create(BMS_MSGQUEUE_OBJECTS, sizeof(bms_msg_queue_obj_t));
    DEV_ASSERT(bms_msg_queue != NULL);
    
    thread_bms = osif_thread_create(bms_task, &param, &bms_attr);
    DEV_ASSERT(thread_bms != NULL);
}
