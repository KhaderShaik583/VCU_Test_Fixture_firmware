#ifndef LOADSWITCHES_H
#define LOADSWITCHES_H

#include <stdint.h>

#include "adc_driver.h"
#include "fw_common.h"

#define AUX_ADC_INSTANCE 	    0U
#define HORN_ADC_INSTANCE 	    0U
#define DRL_TL_ADC_INSTANCE     0U
#define HL_ADC_INSTANCE 	    0U
#define BR_NP_ADC_INSTANCE 	    0U  
#define ABS_ADC_INSTANCE 	    1U
#define BOOT_ADC_INSTANCE 	    1U
#define THROTTLE_ADC_INSTANCE 	1U


/* structure declaration used for storing the current_sense raw values read from ADC for the horn and the Aux pins */
typedef struct
{
	uint16_t horn_adc_raw_value;
	uint16_t aux_adc_raw_value;
	
}loadswitches_adc_raw_values_t;


/* structure declaration used for storing the current_sense calibrated values read from ADC for the horn and the Aux pins */
typedef struct
{
	float_t horn_calibr_value;
	float_t aux_calibr_value;
	
}loadswitches_adc_calibr_values_t;


void loadswitch_adc_init(void);
void drv_loadswitches_init(void);
status_t drv_loadswitches_sem_acquire(uint32_t );
status_t drv_loadswitches_sem_release(void);
void get_loadswitches_status_list (uint16_t *);
void throttle_adc_init(void);
void get_throttle_1_voltage(float_t *voltage, uint16_t *raw_value);
void get_throttle_2_voltage(float_t *voltage, uint16_t *raw_value);
void get_throttle_voltage(float_t *t1_voltage, float_t *t2_voltage, float_t *adc_vref);
#endif /* LOADSWITCHES_H */

