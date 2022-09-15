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

#ifndef LTE_TASK_H
#define LTE_TASK_H

#include "board.h"
#include "fw_common.h"

#define LTE_TASK_NAME           "thread_lte"
#define LTE_TASK_STACK_SIZE     STACK_SIZE(512U)
#define MAX_IOT_BUF_LEN         (512U)
#define LTE_MSGQUEUE_OBJECTS    (32U)

#define LTE_WAIT_EC25_RDY_URC_STATE (0U)
#define LTE_CMD_INIT_STATE          (1U)
#define LTE_WAIT_MSG_STATE          (2U)


typedef struct {                                // object data type
  uint8_t msg_buf[32];
  uint8_t idx;
} msgqueue_obj_t;


void lte_task_create(void);
osThreadId_t lte_task_get_id(void);
osMessageQueueId_t lte_task_get_queue(void);

#endif /* LTE_TASK_H */
