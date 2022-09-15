/*
* 
* ULTRAVIOLETTE AUTOMOTIVE CONFIDENTIAL
* ______________________________________
* 
* [2020] - [2021] Ultraviolette Automotive Pvt. Ltd.
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


#ifndef OSIF_H
#define OSIF_H

#include <stdint.h>
#include "fw_common.h"
#include "fw_features.h"
#include "status.h"

#ifdef USE_FEATURE_RTX_OS
#include "rtx_os.h"
#include "rtx_lib.h"


typedef osMutexId_t 	    mutex_t;
typedef osSemaphoreId_t     semaphore_t;
typedef osThreadFunc_t	    thread_t;
typedef osThreadAttr_t      thread_attrs_t;
typedef osThreadId_t	    thread_id_t;
typedef osRtxThread_t	    thread_tcb_t;

typedef osTimerId_t 	    osif_timer_id_t;
typedef osTimerFunc_t	    osif_timer_cb_t;
typedef osTimerType_t	    osif_timer_type_t;
typedef osTimerAttr_t	    osif_timer_attrs_t;
typedef osMessageQueueId_t  osif_msg_queue_id_t;

/* VCU has 1 tick = 1 ms */
#define MSEC_TO_TICK(msec) (msec)
#define OSIF_WAIT_FOREVER 0xFFFFFFFFu

#define OSIF_WAIT_ANY_FLAG	osFlagsWaitAny
#define OSIF_WAIT_ALL_FLAG	osFlagsWaitAll
#define OSIF_FLAGS_NO_CLEAR osFlagsNoClear

void osif_time_delay(const uint32_t delay);
uint32_t osif_millis(void);

status_t osif_kernel_init(void);
status_t osif_kernel_start(void);
thread_id_t osif_thread_create(thread_t func, void *argument, const thread_attrs_t *attr);
uint32_t osif_thread_wait_on_flag(uint32_t flags, uint32_t options, uint32_t timeout);
uint32_t osif_thread_clear_flag(uint32_t flag);
uint32_t osif_thread_set_flag(thread_id_t thread, uint32_t flag);
status_t osif_thread_suspend(thread_id_t thread);
status_t osif_thread_resume(thread_id_t thread);

osif_timer_id_t osif_timer_create(osif_timer_cb_t func, osif_timer_type_t type, void *argument, const osif_timer_attrs_t *attr);
status_t osif_timer_start(osif_timer_id_t timer_id, uint32_t ticks);
status_t osif_timer_stop(osif_timer_id_t timer_id);

int32_t osif_enter_critical(void);
int32_t osif_exit_critical(int32_t lock);
int32_t osif_kernel_suspend(void);

status_t osif_mutex_lock(const mutex_t *const pMutex, const uint32_t timeout);
status_t osif_mutex_unlock(const mutex_t *const pMutex);
status_t osif_mutex_create(osMutexId_t *mid);
status_t osif_mutex_destroy(const mutex_t *const pMutex);
status_t osif_sem_acquire(semaphore_t *const pSem, const uint32_t timeout);
status_t osif_sem_release(semaphore_t *const pSem);
status_t osif_sem_create(semaphore_t *const sid, const uint32_t initValue);
status_t osif_sem_destroy(const semaphore_t *const pSem);

osif_msg_queue_id_t osif_msg_queue_create(uint32_t msg_count, uint32_t msg_size);
status_t osif_msg_queue_send(osif_msg_queue_id_t queue_id, const void *message, uint8_t msg_prio, uint32_t timeout);
status_t osif_msg_queue_recv(osif_msg_queue_id_t queue_id, void *message, uint8_t *msg_prio, uint32_t timeout);
void osif_msg_queue_num_msgs(osif_msg_queue_id_t queue_id, uint32_t *num_messages);


/* required for compatibility with NXP RTM SDK */
#define OSIF_SemaPost 			osif_sem_release
#define OSIF_SemaWait 			osif_sem_acquire
#define OSIF_SemaCreate 		osif_sem_create
#define OSIF_SemaDestroy 		osif_sem_destroy

#define OSIF_MutexLock 			osif_mutex_lock
#define OSIF_MutexUnlock 		osif_mutex_unlock
#define OSIF_MutexCreate 		osif_mutex_create
#define OSIF_MutexDestroy 		osif_mutex_destroy
#define OSIF_TimeDelay			osif_time_delay
#define OSIF_GetMilliseconds	osif_millis

/*
    Start a task.
    Default start flag is alway bit 0
*/

status_t osif_task_start(osThreadId_t tid);

/*
    Terminate a task.
*/
status_t osif_task_end(osThreadId_t tid);

#endif /* USE_FEATURE_RTX_OS */
#endif /* OSIF_H */

