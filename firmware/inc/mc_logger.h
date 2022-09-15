#ifndef MC_LOGGER_H
#define MC_LOGGER_H

#include "board.h"
#include "fw_common.h"

#define MC_LOGGER_TASK_NAME      "thread_mc_logger"
#define MC_LOGGER_STACK_SIZE     STACK_SIZE(128U)

/* Thread Flags */
#define MC_LOGGER_TASK_START_FLAG   (0x0001U)

#define MC_MSGQUEUE_LOG_OBJECTS     (MC_MAX_TPDOS * 16) 

void mc_logger_task_create(void);
thread_id_t mc_logger_task_get_id(void);

#define MC_LOG_SAVE_CMD     (0xC01U)

#endif /* MC_LOGGER_H */
