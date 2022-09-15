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



#include "osif.h"
#include <stddef.h>

#include "device_registers.h"
#include "fw_common.h"
#include "fw_features.h"
#include "devassert.h"
#include "os_tick.h"

static uint32_t sys_num_tasks = 0U;

static inline uint32_t osif_get_current_tick_count(void)
{
    return osKernelGetTickCount();
}

static status_t translate_os_to_platform_status(osStatus_t oss)
{
    volatile status_t s;
    if(oss == osOK)
    {
        s = STATUS_SUCCESS;
    }
    else if((oss == osError) || (oss == osErrorISR) || (oss == osErrorParameter))
    {
        s = STATUS_ERROR;
    }
    else if((oss == osErrorTimeout))
    {
        s = STATUS_TIMEOUT;
    }
    else
    {
        s = STATUS_UNSUPPORTED;
    }
    
    return s;
    
}

static inline bool osif_is_isr_ctxt(void)
{
    bool is_isr = false;
    volatile uint32_t ipsr_code = (uint32_t)( (S32_SCB->ICSR & S32_SCB_ICSR_VECTACTIVE_MASK) >> S32_SCB_ICSR_VECTACTIVE_SHIFT );
    if (ipsr_code != 0u)
    {
        is_isr = true;
    }

    return is_isr;
}

void osif_time_delay(const uint32_t delay)
{
    uint32_t ticks = 0U;
    ticks = MSEC_TO_TICK(delay);
    osDelay(ticks);
}

uint32_t osif_millis(void)
{
    /*
     * Please make sure the timer is initialized before calling this function.
     * For example, calling osif_time_delay(0) ensures that the timer is initialized
     * without any other side-effects. If osif_time_delay or osif_sem_acquire functions
     * have been called, the timer is already initialized.
     */
    return osif_get_current_tick_count(); /* 1 tick = 1 millisecond */
}

status_t osif_kernel_init(void)
{
    status_t s = STATUS_SUCCESS;
    osStatus_t oss;
    
    oss = osKernelInitialize();
    s = translate_os_to_platform_status(oss);
    
    return s;
}

status_t osif_kernel_start(void)
{
    status_t s = STATUS_SUCCESS;
    osStatus_t oss;
    
    oss = osKernelStart();
    s = translate_os_to_platform_status(oss);
    
    return s;
}

status_t osif_mutex_lock(const mutex_t *const pMutex, const uint32_t timeout)
{
    status_t s = STATUS_SUCCESS;
    uint32_t os_timeout;
    osStatus_t oss;
    mutex_t m;

    if(timeout == OSIF_WAIT_FOREVER)
    {
        os_timeout = osWaitForever;
    }
    else 
    {
        os_timeout = MSEC_TO_TICK(timeout);
    }
    m = *pMutex;
    oss = osMutexAcquire(m, timeout);
    s = translate_os_to_platform_status(oss);
    
    return s;
}

status_t osif_mutex_unlock(const mutex_t *const pMutex)
{
    status_t s = STATUS_SUCCESS;

    osStatus_t oss;
    mutex_t m;

    m = *pMutex;
    oss = osMutexRelease(m);
    s = translate_os_to_platform_status(oss);

    return s;
}


status_t osif_mutex_create(osMutexId_t *mid)
{
    status_t s;
    
    *mid = osMutexNew(NULL);
    if(*mid == NULL)
    {
        s = STATUS_ERROR;
    }
    else
    {
        s = STATUS_SUCCESS;
    }
    
    return s;
}


status_t osif_mutex_destroy(const mutex_t *const pMutex)
{
    status_t s = STATUS_SUCCESS;

    osStatus_t oss;
    mutex_t m;
    
    m = *pMutex;
    oss = osMutexDelete(m);
    s = translate_os_to_platform_status(oss);

    return s;
}


status_t osif_sem_acquire(semaphore_t *const pSem, const uint32_t timeout)
{
   
    DEV_ASSERT(pSem != NULL);
    
    status_t osif_ret_code = STATUS_SUCCESS;

    uint32_t os_timeout;
    osStatus_t oss;
    semaphore_t sem;

    if(timeout == OSIF_WAIT_FOREVER)
    {
        os_timeout = osWaitForever;
    }
    else 
    {
        os_timeout = MSEC_TO_TICK(timeout);
    }
    
    sem = *pSem;
    oss = osSemaphoreAcquire(sem, os_timeout);
    osif_ret_code = translate_os_to_platform_status(oss);

    return osif_ret_code;
}


status_t osif_sem_release(semaphore_t *const pSem)
{
    DEV_ASSERT(pSem != NULL);

    status_t osif_ret_code = STATUS_SUCCESS;
    
    osStatus_t oss;
    semaphore_t sem;

    sem = *pSem;
    bool is_isr = osif_is_isr_ctxt();
    if (is_isr)
    {
        __nop();
    }
    else
    {
        /* Execution from task */
        __nop();
        __nop();
    }
    
    oss = osSemaphoreRelease(sem);
    osif_ret_code = translate_os_to_platform_status(oss);


    return osif_ret_code;
}


status_t osif_sem_create(semaphore_t *const sid, const uint32_t initValue)
{
    status_t s;
    uint32_t new_tokens = initValue;// + 1U;
    
    /* Create binary semaphore */
    *sid = osSemaphoreNew(1U, new_tokens, NULL);
    if(*sid == NULL)
    {
        s = STATUS_ERROR;
    }
    else
    {
        s = STATUS_SUCCESS;
    }
    
    return s;
    
}

status_t osif_sem_destroy(const semaphore_t *const pSem)
{
    DEV_ASSERT(pSem != NULL);

    status_t osif_ret_code = STATUS_SUCCESS;

    osStatus_t oss;
    semaphore_t sem;

    sem = *pSem;
    oss = osSemaphoreDelete(sem);
    osif_ret_code = translate_os_to_platform_status(oss);

    return osif_ret_code;
}

status_t osif_task_start(osThreadId_t tid)
{
    uint32_t tflag = 0U;
    
    tflag = osThreadFlagsSet(tid, 0x0001);
    return tflag;
}

status_t osif_task_end(osThreadId_t tid)
{
    osStatus_t oss; 
    status_t osif_ret_code = STATUS_SUCCESS;
    
    oss = osThreadTerminate(tid);
    
    osif_ret_code = translate_os_to_platform_status(oss);

    return osif_ret_code; 
}

thread_id_t osif_thread_create(thread_t func, void *argument, const thread_attrs_t *attr)
{
	DEV_ASSERT(func != NULL);
	DEV_ASSERT(attr != NULL);
	
	thread_id_t tid;

	tid = osThreadNew(func, argument, attr);
    if(tid != NULL)
    {
        sys_num_tasks++;
    }
    
	return tid; 

}

uint32_t osif_thread_wait_on_flag(uint32_t flags, uint32_t options, uint32_t timeout)
{
	uint32_t wflags = 0U;
	
	wflags = osThreadFlagsWait(flags, options, MSEC_TO_TICK(timeout));

	return wflags;
}

uint32_t osif_thread_set_flag(thread_id_t thread, uint32_t flag)
{
	return osThreadFlagsSet(thread, flag);
}


uint32_t osif_thread_clear_flag(uint32_t flag)
{
	return osThreadFlagsClear(flag);
}

status_t osif_thread_suspend(thread_id_t thread)
{
    osStatus_t oss; 
    status_t osif_ret_code = STATUS_SUCCESS;
    
    oss = osThreadSuspend(thread);
    
    osif_ret_code = translate_os_to_platform_status(oss);

    return osif_ret_code;
}

status_t osif_thread_resume(thread_id_t thread)
{
    osStatus_t oss; 
    status_t osif_ret_code = STATUS_SUCCESS;
    
    oss = osThreadResume(thread);
    
    osif_ret_code = translate_os_to_platform_status(oss);

    return osif_ret_code;
}


int32_t osif_enter_critical(void)
{
	return osKernelLock();
}

int32_t osif_exit_critical(int32_t lock)
{
	return osKernelRestoreLock(lock);
}

int32_t osif_kernel_suspend(void)
{
    return (int32_t)osKernelSuspend();
}

osif_timer_id_t osif_timer_create(osif_timer_cb_t func, osif_timer_type_t type, void *argument, const osif_timer_attrs_t *attr)
{
	return osTimerNew(func, type, argument, attr);
}

status_t osif_timer_start(osif_timer_id_t timer_id, uint32_t ticks)
{
    status_t osif_ret_code = STATUS_SUCCESS;
    osStatus_t oss;

	oss = osTimerStart(timer_id, ticks);

	osif_ret_code = translate_os_to_platform_status(oss);

	return osif_ret_code;
}

status_t osif_timer_stop(osif_timer_id_t timer_id)
{
    status_t osif_ret_code = STATUS_SUCCESS;
    osStatus_t oss;

	oss = osTimerStop(timer_id);

	osif_ret_code = translate_os_to_platform_status(oss);

	return osif_ret_code;
}

osif_msg_queue_id_t osif_msg_queue_create(uint32_t msg_count, uint32_t msg_size)
{
    return osMessageQueueNew(msg_count, msg_size, NULL);
}

status_t osif_msg_queue_send(osif_msg_queue_id_t queue_id, const void *message, uint8_t msg_prio, uint32_t timeout)
{
    status_t osif_ret_code = STATUS_SUCCESS;
    osStatus_t oss; 

	oss = osMessageQueuePut(queue_id, message, msg_prio, MSEC_TO_TICK(timeout));

	osif_ret_code = translate_os_to_platform_status(oss);

	return osif_ret_code;
}

status_t osif_msg_queue_recv(osif_msg_queue_id_t queue_id, void *message, uint8_t *msg_prio, uint32_t timeout)
{
    status_t osif_ret_code = STATUS_SUCCESS;
    osStatus_t oss; 
    
    UNUSED_PARAM(msg_prio);
	oss = osMessageQueueGet(queue_id, message, NULL, MSEC_TO_TICK(timeout)); 

	osif_ret_code = translate_os_to_platform_status(oss);

	return osif_ret_code;    
}

void osif_msg_queue_num_msgs(osif_msg_queue_id_t queue_id, uint32_t *num_messages)
{
    *num_messages = osMessageQueueGetCount(queue_id);
}

