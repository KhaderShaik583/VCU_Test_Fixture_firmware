#include "abs_task.h"
#include "osif.h"
#include "flexcan_driver.h"

#include "rtx_os.h"

__attribute__((section(".bss.os.thread.cb")))
thread_tcb_t abs_thread_tcb;

thread_id_t thread_abs;

__attribute__((section("ARM_LIB_STACK")))
uint64_t abs_thread_stk[ABS_STACK_SIZE];

/* global variables declaration */
static flexcan_msgbuff_t recvBuff;

/* Driver semaphore */
static semaphore_t drv_abs_can_sem_t;

static status_t drv_abs_sem_acquire(uint32_t timeout)
{
    status_t s;
    s = osif_sem_acquire(&drv_abs_can_sem_t, timeout);
    return s;
}

static status_t drv_abs_sem_release(void)
{
    status_t s;
    s = osif_sem_release(&drv_abs_can_sem_t);
    return s;
}

const thread_attrs_t abs_attr = {
    ABS_TASK_NAME,
    osThreadDetached,
    &abs_thread_tcb,
    sizeof(abs_thread_tcb),
    &abs_thread_stk[0],
    sizeof(abs_thread_stk),
    osPriorityNormal,
    0U,
    0U    
};

/*FUNCTION**********************************************************************
 *
 * Function Name : read_motor_controller
 * Description   : read messages from Motor controller
 *
 * Implements    : read_motor_controller_Activity
 *END**************************************************************************/

static status_t config_abs_can(void)
{
	status_t read_status = STATUS_SUCCESS;
	flexcan_data_info_t dataInfo =
    {
       .data_length = 8U,
       .msg_id_type = FLEXCAN_MSG_ID_STD,
       .enable_brs  = false,
       .fd_enable   = false,
       .fd_padding  = 0U
    };

    FLEXCAN_DRV_ConfigRxMb(CAN_IF_ABS, RX_ABS_MAILBOX, &dataInfo, RX_ABS_MSG_ID);
	FLEXCAN_DRV_SetRxMbGlobalMask(CAN_IF_ABS, FLEXCAN_MSG_ID_STD, ABS_CAN_IF_MBX_FILTER);
	FLEXCAN_DRV_SetRxIndividualMask(CAN_IF_ABS, FLEXCAN_MSG_ID_STD, RX_ABS_MAILBOX, ABS_CAN_IF_GBL_MBX_FILTER);
    
	return read_status;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : abs_rx_msgs
 * Description   : storing the received messages bytewise
 *
 * Implements    : abs_rx_msgs_Activity
 *END**************************************************************************/

static void abs_rx_msgs(void)
{
	status_t status = STATUS_SUCCESS;
    uint16_t speed = 0U;
    uint8_t fwheel_validity = 0U;
    uint8_t rwheel_validity = 0U;
    
    uint8_t warning_lamp = 0U;   

	FLEXCAN_DRV_Receive(CAN_IF_ABS, RX_ABS_MAILBOX, &recvBuff);
	
	while(1)
    {
		status = FLEXCAN_DRV_GetTransferStatus(CAN_IF_ABS, RX_ABS_MAILBOX);
		if(status == STATUS_SUCCESS)
		{	
            if(recvBuff.msgId == ABS_CAN_SPEED_INFO_MSG_ID)
            {
                fwheel_validity = (recvBuff.data[4] & (1U << 4U));
                rwheel_validity = (recvBuff.data[4] & (1U << 5U));
                
                if(fwheel_validity == 0U)
                {
                    speed = (uint16_t)recvBuff.data[1] | ((uint16_t)recvBuff.data[0] << 8U);
                }
                
                if(rwheel_validity == 0U)
                {
                    speed = (uint16_t)recvBuff.data[3] | ((uint16_t)recvBuff.data[2] << 8U);
                }                
            }
			else if(recvBuff.msgId == ABS_CAN_WARNING_LAMP_MSG_ID)
            {
                warning_lamp = (recvBuff.data[0] & (1U << 0U));
            }
            else
            {
                __nop();
            }

            FLEXCAN_DRV_Receive(CAN_IF_ABS, RX_ABS_MAILBOX, &recvBuff);
		}
        
		__nop();
    }
}	

static void abs_task(void *arg)
{
    config_abs_can();
    
    osif_thread_wait_on_flag(ABS_TASK_START_FLAG, OSIF_WAIT_ANY_FLAG, OSIF_WAIT_FOREVER);
    
    abs_rx_msgs();
    
	osif_time_delay(1000U);
		
}

thread_id_t abs_task_get_id(void)
{
    return thread_abs;
}

void abs_task_create(void)
{
    uint32_t param = NULL;
    
    thread_abs = osif_thread_create(abs_task, &param, &abs_attr);
    DEV_ASSERT(thread_abs != NULL);
}

