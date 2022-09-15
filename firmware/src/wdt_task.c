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
 
#include "wdt_task.h"
#include "pins_driver.h"
#include "wdog_driver.h" 

#ifdef USE_FEATURE_WDT

#ifdef USE_FEATURE_RTX_VIEWER
__attribute__((section(".bss.os.thread.cb")))
thread_tcb_t wdt_thread_tcb;
#else
static thread_tcb_t led_thread_tcb;
#endif /* USE_FEATURE_RTX_VIEWER */

__attribute__((section("ARM_LIB_STACK")))
static uint64_t wdt_thread_stk[WDT_TASK_STACK_SIZE];

#define WDT_MSGQUEUE_OBJECTS    (32U) 
static thread_id_t thread_wdt;
osif_msg_queue_id_t wdt_msg_queue; 

static wdt_states_t wdt_registered_tasks[WDT_MAX_REG_TASKS];
static uint32_t wdt_reg_task_count = 0U;
static volatile uint32_t wdt_sw_suspend = 0U;

static const thread_attrs_t wdt_task_attr = {
    WDT_TASK_NAME,
    osThreadDetached,
    &wdt_thread_tcb,
    sizeof(wdt_thread_tcb),
    &wdt_thread_stk[0],
    WDT_TASK_STACK_SIZE * sizeof(uint64_t),
    osPriorityNormal4,
    0U,
    0U    
};

static void ext_wdt_enable(void)
{
    PINS_DRV_SetPins(EXT_WDT_EN_GPIO, (1U << EXT_WDT_EN_PIN));
}

static void ext_wdt_disable(void)
{
    PINS_DRV_ClearPins(EXT_WDT_EN_GPIO, (1U << EXT_WDT_EN_PIN));
}

static void disable_wdt_internal(void)
{
    /* Write of the WDOG unlock key to CNT register, must be done in order to allow any modifications*/
    WDOG->CNT = (uint32_t ) FEATURE_WDOG_UNLOCK_VALUE;
    /* The dummy read is used in order to make sure that the WDOG registers will be configured only
    * after the write of the unlock value was completed. */
    (void)WDOG->CNT;

    /* Initial write of WDOG configuration register:
     * enables support for 32-bit refresh/unlock command write words,
     * clock select from LPO, update enable, watchdog disabled */
    WDOG->CS  = (uint32_t ) ( (1UL << WDOG_CS_CMD32EN_SHIFT)                       |
                            (FEATURE_WDOG_CLK_FROM_LPO << WDOG_CS_CLK_SHIFT)     |
                            (0U << WDOG_CS_EN_SHIFT)                             |
                            (1U << WDOG_CS_UPDATE_SHIFT)                         );

    /* Configure timeout */
    WDOG->TOVAL = (uint32_t )0xFFFF;
}

static void ext_wdt_kick(void)
{
    /* Merge core and external wdt apis */
    
    /* Kick internal WDT */
    WDOG_DRV_Trigger(0U);
    
    /* Kick external WDT */
    PINS_DRV_ClearPins(EXT_WDT_WDI_GPIO, (1U << EXT_WDT_WDI_PIN));
    sw_asm_delay_us(50);
    PINS_DRV_SetPins(EXT_WDT_WDI_GPIO, (1U << EXT_WDT_WDI_PIN));    
}

static void process_WDT_KICK_MSG(uint32_t data[])
{
    uint32_t i = 0U;
    int32_t lc = 0;
    
    lc = osif_enter_critical();
    for(i = 0U; i < wdt_reg_task_count; i++)
    {
        if(data[0] == wdt_registered_tasks[i].reg_task_id)
        {
            wdt_registered_tasks[i].is_alive = 1U;
            break;
        }
    }
    (void)osif_exit_critical(lc);
}

static void wdt_suspend_internal(void)
{
    wdt_sw_suspend = 1U;
}

static void wdt_resume_internal(void)
{
    wdt_sw_suspend = 0U;
}

thread_id_t wdt_task_get_id(void)
{
    return thread_wdt;
}

void wdt_task_vote(uint32_t tid)
{
    uint32_t i = 0U;

    for(i = 0U; i < wdt_reg_task_count; i++)
    {
        if(tid == wdt_registered_tasks[i].reg_task_id)
        {
            wdt_registered_tasks[i].is_alive = 1U;
            break;
        }
    }
}

__NO_RETURN static void wdt_task(void *arg)
{
    status_t qstatus = STATUS_SUCCESS;
    wdt_msg_queue_obj_t wdt_mq;
    uint32_t i = 0U;
    int32_t lc = 0;
    
    UNUSED_PARAM(arg);
    osif_thread_wait_on_flag(WDT_TASK_START_FLAG, OSIF_WAIT_ANY_FLAG, OSIF_WAIT_FOREVER);
    
    for(;; )
    {
        qstatus = osif_msg_queue_recv(wdt_msg_queue, &wdt_mq, NULL, 10U); 
        if(qstatus == osOK) 
        {
            switch(wdt_mq.msg_id)
            {
                case WDT_ENABLE_MSG:
                    ext_wdt_enable();
                    break;
                case WDT_DISABLE_MSG:
                    disable_wdt_internal();
                    ext_wdt_disable();
                    break;
                case WDT_SUSPEND_MSG:
                    wdt_suspend_internal();
                    break;
                case WDT_KICK_MSG:
                    while(qstatus == osOK)
                    {
                        process_WDT_KICK_MSG(wdt_mq.data);
                        qstatus = osif_msg_queue_recv(wdt_msg_queue, &wdt_mq, NULL, 0U); 
                    }
                    break;
            }
        }
        
        lc = osif_enter_critical();
        for(i = 0U; i < wdt_reg_task_count; i++)
        {
            if(wdt_registered_tasks[i].is_alive == 1U)
            {
                continue;
            }
            else
            {
                break;
            }
        }
        
        
        if(i == wdt_reg_task_count)
        {
            /* At this point all tasks have checked-in */
            for(i = 0U; i < wdt_reg_task_count; i++)
            {
                wdt_registered_tasks[i].is_alive = 0U;
            }
            ext_wdt_kick();
        }
        (void)osif_exit_critical(lc);
    }
}

void wdt_register_task(uint32_t task_id)
{
    wdt_registered_tasks[wdt_reg_task_count].reg_task_id = task_id;
    wdt_registered_tasks[wdt_reg_task_count].is_alive = 0U;
    wdt_reg_task_count++;
}

void wdt_task_create(void)
{
    uint32_t param = NULL;
    
    thread_wdt = osif_thread_create(wdt_task, &param, &wdt_task_attr);
    DEV_ASSERT(thread_wdt != NULL);
    
    wdt_msg_queue = osif_msg_queue_create(WDT_MSGQUEUE_OBJECTS, sizeof(wdt_msg_queue_obj_t));
    DEV_ASSERT(wdt_msg_queue != NULL);
}
#endif /* USE_FEATURE_WDT */
