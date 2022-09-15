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
 
#ifndef WDT_TASK_H
#define WDT_TASK_H

#include "board.h"
#include "fw_common.h"

#define WDT_TASK_NAME           "thread_wdt"
#define WDT_TASK_STACK_SIZE     STACK_SIZE(128U)
#define WDT_MAX_REG_TASKS       (32U)

#define WDT_ENABLE_MSG      (0x7AU)
#define WDT_DISABLE_MSG     (0x75U)
#define WDT_KICK_MSG        (0x7CU)
#define WDT_SUSPEND_MSG     (0x7FU)

typedef struct 
{
    uint32_t msg_id;
    uint32_t data[2];
}wdt_msg_queue_obj_t;

typedef struct
{
    uint32_t reg_task_id;
    uint32_t is_alive;
}wdt_states_t;

extern osif_msg_queue_id_t wdt_msg_queue;

/* Thread Flags */
#define WDT_TASK_START_FLAG     (0x0001U)

void wdt_task_create(void);
thread_id_t wdt_task_get_id(void);
void wdt_register_task(uint32_t task_id);
void wdt_task_vote(uint32_t tid);

#endif /* WDT_TASK_H */
