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
 * Author : Ashwini V. [056]
 *
 */
 
#include "lsm_task.h"
#include "osif.h"
#include "drv_loadswitches.h"
#include "mc_task.h"
#include "wdt_task.h"

#define THROTTLE_MAX_VOLTAGE_LIMIT      (2.35f)
#define THROTTLE_MIN_VOLTAGE_LIMIT      (1.65f)
#define THROTTLE_ZERO_VOLTAGE_LIMIT     (0.50f)

#ifdef USE_FEATURE_RTX_VIEWER
__attribute__((section(".bss.os.thread.cb")))
thread_tcb_t lsm_thread_tcb;
#else
static thread_tcb_t lsm_thread_tcb;
#endif /* USE_FEATURE_RTX_VIEWER */

static thread_id_t thread_lsm;

__attribute__((section("ARM_LIB_STACK")))
static uint64_t lsm_thread_stk[LSM_TASK_STACK_SIZE];

static const thread_attrs_t lsm_attr = {
    LSM_TASK_NAME,
    osThreadDetached,
    &lsm_thread_tcb,
    sizeof(lsm_thread_tcb),
    &lsm_thread_stk[0],
    LSM_TASK_STACK_SIZE * sizeof(uint64_t),
    osPriorityNormal,
    0U,
    0U    
};

static float_t throttle_1_voltage = 0.0f;
static float_t throttle_2_voltage = 0.0f;
static float_t throttle_adc_vref = 0.0f;
static volatile float_t throttle_diff = 0.0f;

#if 0
static uint16_t switch_current[2] = {0U};
#endif

static void detect_throttle_fault(void)
{
    static uint32_t counter = 0U;
	
	if(throttle_1_voltage > 0.0f)
	{
		throttle_diff = throttle_2_voltage / throttle_1_voltage;
		if((throttle_diff > THROTTLE_MAX_VOLTAGE_LIMIT) || 
		   (throttle_diff < THROTTLE_MIN_VOLTAGE_LIMIT))
		{
			counter += 1U;
			if(counter == 5U)
			{
				/* 20ms task period * 5 gives an approx. 100ms window */
				mc_set_gear(MC_GEAR_POS_NEUTRAL);
				set_status_bit(STAT_VCU_THROTTLE_ERROR);
				counter = 0U;
			}
		}
		else
		{
			counter = 0U;
		}
	}
}    


static void throttle_exec(void)
{
    int32_t lc = 0;
    
    lc = osif_enter_critical();

    get_throttle_voltage(&throttle_1_voltage, &throttle_2_voltage, &throttle_adc_vref);
    detect_throttle_fault(); 
    
    (void)osif_exit_critical(lc);
}

uint32_t lsm_is_throttle_voltage_zero(void)
{
    uint32_t ret = 0U;
    
    if(throttle_1_voltage < THROTTLE_ZERO_VOLTAGE_LIMIT)
    {
        ret = 1U;
    }
    else
    {
        ret = 0U;
    }   
    
    return ret;
}

void lsm_throttle_2_voltage(float_t *tv)
{
    *tv = throttle_2_voltage;
}

void lsm_throttle_1_voltage(float_t *tv)
{
    *tv = throttle_1_voltage;
}

void lsm_throttle_ratio_voltage(float_t *tr)
{
    *tr = throttle_diff;
}

void lsm_throttle_adc_vref_voltage(float_t *vref)
{
    *vref = throttle_adc_vref;
}

__NO_RETURN static void lsm_task(void *arg)
{
    UNUSED_PARAM(arg);

	(void)osif_thread_wait_on_flag(LSM_TASK_START_FLAG, OSIF_WAIT_ANY_FLAG, OSIF_WAIT_FOREVER);
    
    /* delay for stabilizing throttle voltage at ADC inputs */
    osif_time_delay(1000);
    
	for( ;; )
    {
        throttle_exec();
        osif_time_delay(20);
	}
}
 
thread_id_t lsm_task_get_id(void)
{
    return thread_lsm;
}

void lsm_task_create(void)
{
    uint32_t param = NULL;
    
    thread_lsm = osif_thread_create(lsm_task, &param, &lsm_attr);
    DEV_ASSERT(thread_lsm != NULL);
}

