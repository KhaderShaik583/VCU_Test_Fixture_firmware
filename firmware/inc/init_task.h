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
 
#ifndef INIT_TASK_H
#define INIT_TASK_H

#include "fw_common.h"

#define INIT_TASK_NAME           "thread_init"
#define INIT_TASK_STACK_SIZE     STACK_SIZE(128U)

#define MSG_ID_REBOOT_REQ           (0x5100FFFFU)
#define MSG_ID_SLEEP_REQ            (0x5101FFFFU)
#define MSG_ID_CHARGER_CONNECTED    (0x5102FFFFU)
#define MSG_ID_CHARGER_DISCONNECTED (0x5103FFFFU)

extern osif_msg_queue_id_t sys_msg_queue;

typedef struct 
{
    uint32_t msg_id;
    uint32_t msg_len;
    uint8_t msg[4];
}sys_msg_queue_obj_t;

void init_task_create(void);
osThreadId_t init_task_get_id(void);
uint32_t sys_init_get_os_state(void);

#endif /* INIT_TASK_H */ 
