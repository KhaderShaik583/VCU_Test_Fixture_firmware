#ifndef INIT_TASK_DESK_H
#define INIT_TASK_DESK_H

#include "fw_common.h"

#define INIT_TASK_NAME           "thread_init_desk"
#define INIT_TASK_STACK_SIZE     STACK_SIZE(128U)

#define MSG_ID_REBOOT_REQ           (0x5100FFFFU)
#define MSG_ID_SLEEP_REQ            (0x5101FFFFU)

extern osif_msg_queue_id_t sys_msg_queue;

typedef struct 
{
    uint32_t msg_id;
    uint32_t msg_len;
    uint8_t msg[4];
}sys_msg_queue_obj_t;

void init_task_desk_create(void);
osThreadId_t init_task_desk_get_id(void);
uint32_t sys_init_get_os_state(void);

#endif /* INIT_TASK_DESK_H */
