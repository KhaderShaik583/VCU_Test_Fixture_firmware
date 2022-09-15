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
 
#include "led_task.h"
#include "pins_driver.h"
#include "rtc_task.h"
#include "pac1921_task.h"
#include "wdt_task.h"

#ifdef USE_FEATURE_RTX_VIEWER
__attribute__((section(".bss.os.thread.cb")))
thread_tcb_t led_thread_tcb;
#else
static thread_tcb_t led_thread_tcb;
#endif /* USE_FEATURE_RTX_VIEWER */

__attribute__((section("ARM_LIB_STACK")))
static uint64_t led_thread_stk[LED_TASK_STACK_SIZE];

static thread_id_t thread_led;

static const thread_attrs_t led_task_attr = {
    LED_TASK_NAME,
    osThreadDetached,
    &led_thread_tcb,
    sizeof(led_thread_tcb),
    &led_thread_stk[0],
    LED_TASK_STACK_SIZE * sizeof(uint64_t),
    osPriorityNormal,
    0U,
    0U    
};

__NO_RETURN static void led_task(void *arg)
{  
    UNUSED_PARAM(arg);
    
    (void)osif_thread_wait_on_flag(LED_TASK_START_FLAG, OSIF_WAIT_ANY_FLAG, OSIF_WAIT_FOREVER);

    while(1)
    {
#ifndef USE_FEATURE_VCU_ON_DESK
        /* update RTC context every second */
        rtc_trig_measurement();
        
        /* Read the aux battery voltage & current */
        pac1921_ops();
        
        /* Toggle hearbeat LED */
        PINS_DRV_TogglePins(LED_GREEN_GPIO, 1U << LED_GREEN_PIN);
        osif_time_delay(1000);
#else
        __NOP();
#endif
    }
}

thread_id_t led_task_get_id(void)
{
    return thread_led;
}

void led_task_create(void)
{
    uint32_t param = NULL;
    
    thread_led = osif_thread_create(led_task, &param, &led_task_attr);
    DEV_ASSERT(thread_led != NULL);
}

