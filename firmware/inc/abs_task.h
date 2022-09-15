#ifndef ABS_TASK_H
#define ABS_TASK_H

#include "fw_common.h"
#include "board.h"
#include "flexcan_driver.h"

#define ABS_TASK_NAME        "thread_absif"
#define ABS_STACK_SIZE       STACK_SIZE(512U)

#define ABS_TASK_START_FLAG     (0x0001U)

/* Definition of the TX and RX message buffers depending on the bus role */

#define RX_ABS_MAILBOX       (0UL)
#define RX_ABS_MSG_ID        (0x12BU)

#define ABS_CAN_IF_MBX_FILTER         (0x800U)  
#define ABS_CAN_IF_GBL_MBX_FILTER     (0x400U)
#define ABS_TO_VCU_2_0_RX_MBX    	  (4U)

#define ABS_CAN_SPEED_INFO_MSG_ID       (0x12bU)
#define ABS_CAN_WARNING_LAMP_MSG_ID     (0x13cU)

#define ABS_CAN_SPEED_MUL_FACTOR        (0.01f)

void abs_task_create(void);
osThreadId_t abs_task_get_id(void);

#endif /* ABS_TASK_H */
