#include<stdint.h>
#include<status.h>

#include "board.h" 
#include "drv_loadswitches.h"
#include "pins_driver.h"
#include "pins_port_hw_access.h" 
#include "board.h"
#include "osif.h"
#include "bcm.h"
#include "ftm_common.h"
#include "ftm_ic_driver.h"

static loadswitches_adc_raw_values_t current_adc_raw_values;
static loadswitches_adc_calibr_values_t current_adc_calibr_values;
static volatile float_t conv_adc_value = 0.0f;

static semaphore_t drv_loadswitches_sem_t;

void drv_loadswitches_init(void)
{
    status_t s;
    s = OSIF_SemaCreate(&drv_loadswitches_sem_t, 0U);
    DEV_ASSERT(s == STATUS_SUCCESS);
    
}

status_t drv_loadswitches_sem_acquire(uint32_t timeout)
{
    status_t s;
    s = OSIF_SemaWait(&drv_loadswitches_sem_t, timeout);
    return s;
}

status_t drv_loadswitches_sem_release(void)
{
    status_t s;
    s = OSIF_SemaPost(&drv_loadswitches_sem_t);
    return s;
}

/*common configuration for ADC channels */
static const adc_converter_config_t adConv_ConvConfig0 = 
{
    .clockDivide              = ADC_CLK_DIVIDE_4,
    .sampleTime               = 15U,
    .resolution               = ADC_RESOLUTION_10BIT,
    .inputClock               = ADC_CLK_ALT_1,
    .trigger                  = ADC_TRIGGER_SOFTWARE,
    .pretriggerSel            = ADC_PRETRIGGER_SEL_PDB,
    .triggerSel               = ADC_TRIGGER_SEL_PDB,
    .dmaEnable                = false,
    .voltageRef               = ADC_VOLTAGEREF_VREF,
    .continuousConvEnable     = false,
    .supplyMonitoringEnable   = false,
};

/* ADC channel configuration for Horn */
static adc_chan_config_t Horn_adConv_ChnConfig0 = 
{
  .interruptEnable = false,
  .channel = ADC_INPUTCHAN_EXT19,
};

static adc_chan_config_t throttle_1_adConv_ChnConfig0 = 
{
  .interruptEnable = false,
  .channel = ADC_INPUTCHAN_EXT2,
};

static adc_chan_config_t throttle_2_adConv_ChnConfig0 = 
{
  .interruptEnable = false,
  .channel = ADC_INPUTCHAN_EXT26,
};

static adc_chan_config_t adc_band_gap_config = 
{
  .interruptEnable = false,
  .channel = ADC_INPUTCHAN_BANDGAP,
};

/*
	For ADC calibration, 32 samples are averaged.
*/
static const adc_average_config_t adc_common_avg_en_config = 
{
    .hwAvgEnable              = true,
    .hwAverage                = ADC_AVERAGE_32,
};

/*
	For all other conversion after calibration, 4 samples are averaged
*/
static const adc_average_config_t adc_common_avg_config = 
{
    .hwAvgEnable              = true,
    .hwAverage                = ADC_AVERAGE_32,
};

void throttle_adc_init(void)
{
	/*  ADC conversion configuration   */
	ADC_DRV_ConfigConverter(THROTTLE_ADC_INSTANCE, &adConv_ConvConfig0);
	
    /* Enable HW Averaging of 32 samples */
    ADC_DRV_ConfigHwAverage(THROTTLE_ADC_INSTANCE, &adc_common_avg_en_config);
    ADC_DRV_AutoCalibration(THROTTLE_ADC_INSTANCE);

    /* HW Averaging for subsequent conversions */
    ADC_DRV_ConfigHwAverage(THROTTLE_ADC_INSTANCE, &adc_common_avg_config);

}	

void loadswitch_adc_init(void)
{
	/*  ADC conversion configuration   */
	ADC_DRV_ConfigConverter(AUX_ADC_INSTANCE, &adConv_ConvConfig0);

    /* Enable HW Averaging of 32 samples */
    ADC_DRV_ConfigHwAverage(AUX_ADC_INSTANCE, &adc_common_avg_en_config);

    ADC_DRV_AutoCalibration(AUX_ADC_INSTANCE);

    /* HW Averaging for subsequent conversions */
    ADC_DRV_ConfigHwAverage(AUX_ADC_INSTANCE, &adc_common_avg_config);
}	


/*FUNCTION**********************************************************************
 *
 * Function Name : horn_monitor
 * Description   : detecting monitoring the current_sense at load switches pins
 *
 * Implements    : horn_monitor_Activity
 *END**************************************************************************/

/* function definitions for monitoring the current_sense at load switches pins*/

static void horn_monitor(void)
{
	const int IL = 21U;
	const int KILIS = 22700U;
	
	ADC_DRV_ConfigChan(HORN_ADC_INSTANCE, 0UL, &Horn_adConv_ChnConfig0);
	ADC_DRV_WaitConvDone(HORN_ADC_INSTANCE);
	
	ADC_DRV_GetChanResult(HORN_ADC_INSTANCE, 0U, (uint16_t *)&current_adc_raw_values.horn_adc_raw_value);
	conv_adc_value = ((float_t)current_adc_raw_values.horn_adc_raw_value/4095.0f)*(5.0f);
	
	/* 2. calibrated ADC value in normal operation*/
	current_adc_calibr_values.horn_calibr_value  = (float_t)(IL/KILIS) * conv_adc_value;       //  IS = IL / KILIS	
	
}

/* funtion definig the monitoring of Aux cuurent_sense pin */

static void aux_monitor(void)
{
	const uint16_t IL = 40U;
	const uint16_t dKILIS = 52100U;
	
	/* 1.get read the ADC converted value */ 
	ADC_DRV_GetChanResult(AUX_ADC_INSTANCE, 0U, (uint16_t *)&current_adc_raw_values.aux_adc_raw_value);
	conv_adc_value = ((float_t)current_adc_raw_values.aux_adc_raw_value/4095.0f)*(5.0f);
	
	/* 2.calibrated ADC value in normal operation*/
	current_adc_calibr_values.aux_calibr_value  = (float_t)(IL/dKILIS);     		  //  IS = IL / KILIS	
}	

/*FUNCTION**********************************************************************
 *
 * Function Name : fault_ind_monitor
 * Description   : detecting faulty indicators   
 *
 * Implements    : fault_ind_monitor_Activity
 *END**************************************************************************/


void fault_ind_monitor(void)
{
	/* 1. FTM initialisation	*/
	
	uint16_t inputCaptureMeas_right_ind = 0;
	uint16_t inputCaptureMeas_left_ind = 0;
	uint32_t frequency_right_ind_pin;
	uint32_t frequency_left_ind_pin;
	
	
	/*	frequency in normal operation  */
    uint32_t normal_operation_frequency = 3;					

   	
    /* Get the FTM1 frequency to calculate
     * the frequency of the measured signal.
    */
    frequency_right_ind_pin = FTM_DRV_GetFrequency(SYS_FTM_INST);
	frequency_left_ind_pin = FTM_DRV_GetFrequency(SYS_FTM_INST);
	
	
	 /* Get values */
    inputCaptureMeas_right_ind = FTM_DRV_GetInputCaptureMeasurement(SYS_FTM_INST, 0U);
    /* Calculate the signal frequency using recorded data*/
	inputCaptureMeas_right_ind = (uint16_t)(frequency_right_ind_pin / inputCaptureMeas_right_ind);
		
	/* comparing the measured/captured frequency at pin with frequency in normal operation */		
	if(inputCaptureMeas_right_ind >= 2*normal_operation_frequency)
	{
		PINS_DRV_ClearPins(LED_GREEN_GPIO, 1U << LED_GREEN_PIN);
	}
	else
	{
        PINS_DRV_SetPins(LED_GREEN_GPIO, 1U << LED_GREEN_PIN);		
	}
	
	/* Get values */
    inputCaptureMeas_left_ind = FTM_DRV_GetInputCaptureMeasurement(SYS_FTM_INST, 0U);
    /* Calculate the signal frequency using recorded data*/
	inputCaptureMeas_left_ind = (uint16_t)(frequency_left_ind_pin / inputCaptureMeas_left_ind);		
	
	/* comparing the measured/captured frequency at pin with frequency in normal operation */	
	if(inputCaptureMeas_left_ind >= 2*normal_operation_frequency)
	{
		PINS_DRV_ClearPins(LED_GREEN_GPIO, 1U << LED_GREEN_PIN);
	}
	else
	{
        PINS_DRV_SetPins(LED_GREEN_GPIO, 1U << LED_GREEN_PIN);
	}	
}


/*FUNCTION**********************************************************************
 *
 * Function Name : get_loadswitches_status_list
 * Description   : defining to get the current_sense values at loadswitches  
 *
 * Implements    : get_loadswitches_status_list_Activity
 *END**************************************************************************/

void get_loadswitches_status_list (uint16_t *buffer)
{
	horn_monitor();
	aux_monitor();
	buffer[0] = current_adc_raw_values.horn_adc_raw_value;
	buffer[1] = current_adc_raw_values.aux_adc_raw_value;
	fault_ind_monitor();
}

/*FUNCTION**********************************************************************
 *
 * Function Name : get_throttle_1_voltage
 * Description   : getting the throttle_1 voltage values at ADc channel
 *
 * Implements    : get_throttle_1_voltage_Activity
 *END**************************************************************************/
void get_throttle_1_voltage(float_t *voltage, uint16_t *raw_value)
{   
    volatile float_t vref = 0.0f;
    volatile uint16_t vref_raw = 0U;
    float_t conv_volt = 0.0f;
    uint16_t adc_throttle_1_voltage = 0U;
    
    ADC_DRV_ConfigChan(THROTTLE_ADC_INSTANCE, 0UL, &adc_band_gap_config);
	ADC_DRV_WaitConvDone(THROTTLE_ADC_INSTANCE);
    ADC_DRV_GetChanResult(THROTTLE_ADC_INSTANCE, 0U, (uint16_t *)&vref_raw);
    vref = (float_t)((1000.0f * (1U << 10U)) / vref_raw) / 1000.0f;
    
    ADC_DRV_ConfigChan(THROTTLE_ADC_INSTANCE, 0UL, &throttle_1_adConv_ChnConfig0);
	ADC_DRV_WaitConvDone(THROTTLE_ADC_INSTANCE);
    ADC_DRV_GetChanResult(THROTTLE_ADC_INSTANCE, 0U, (uint16_t *)&adc_throttle_1_voltage);
    
    conv_volt =  (1.0f / 0.0909f) * ((float_t)adc_throttle_1_voltage / ((1U << 10U) - 1U)) * vref;

    *voltage = conv_volt;
    *raw_value = adc_throttle_1_voltage;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : get_throttle_2_voltage
 * Description   : getting the throttle_2 voltage values at ADC channel 
 *
 * Implements    : get_throttle_2_voltage_Activity
 *END**************************************************************************/
void get_throttle_2_voltage(float_t *voltage, uint16_t *raw_value)
{   
    volatile float_t vref = 0.0f;
    volatile uint16_t vref_raw = 0U;
    float_t conv_volt = 0.0f;
    uint16_t adc_throttle_2_voltage = 0U;
    
    ADC_DRV_ConfigChan(THROTTLE_ADC_INSTANCE, 0UL, &adc_band_gap_config);
	ADC_DRV_WaitConvDone(THROTTLE_ADC_INSTANCE);
    ADC_DRV_GetChanResult(THROTTLE_ADC_INSTANCE, 0U, (uint16_t *)&vref_raw);
    vref = (float_t)((1000.0f * (1U << 10U)) / vref_raw) / 1000.0f;
    
    ADC_DRV_ConfigChan(THROTTLE_ADC_INSTANCE, 0UL, &throttle_2_adConv_ChnConfig0);
	ADC_DRV_WaitConvDone(THROTTLE_ADC_INSTANCE);
    ADC_DRV_GetChanResult(THROTTLE_ADC_INSTANCE, 0U, (uint16_t *)&adc_throttle_2_voltage);
    
    conv_volt =  (1.0f / 0.0909f) * ((float_t)adc_throttle_2_voltage / ((1U << 10U) - 1U)) * vref;

    *voltage = conv_volt;
    *raw_value = adc_throttle_2_voltage;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : get_throttle_voltages
 * Description   : getting the throttle_2 & throttle_1 voltage values at ADC channel 
 *
 * Implements    : get_throttle_voltages
 *END**************************************************************************/
void get_throttle_voltage(float_t *t1_voltage, float_t *t2_voltage, float_t *adc_vref)
{
    volatile float_t vref = 0.0f;
    volatile uint16_t vref_raw = 0U;
    float_t conv_volt = 0.0f;
    uint16_t adc_throttle_1_voltage = 0U;
    uint16_t adc_throttle_2_voltage = 0U;
    
    ADC_DRV_ConfigChan(THROTTLE_ADC_INSTANCE, 0UL, &adc_band_gap_config);
	ADC_DRV_WaitConvDone(THROTTLE_ADC_INSTANCE);
    ADC_DRV_GetChanResult(THROTTLE_ADC_INSTANCE, 0U, (uint16_t *)&vref_raw);
    vref = (float_t)((1000.0f * (1U << 10U)) / vref_raw) / 1000.0f;
    *adc_vref = vref;
    
    ADC_DRV_ConfigChan(THROTTLE_ADC_INSTANCE, 0UL, &throttle_1_adConv_ChnConfig0);
	ADC_DRV_WaitConvDone(THROTTLE_ADC_INSTANCE);
    ADC_DRV_GetChanResult(THROTTLE_ADC_INSTANCE, 0U, (uint16_t *)&adc_throttle_1_voltage);
    
    conv_volt =  (1.0f / 0.0909f) * ((float_t)adc_throttle_1_voltage / ((1U << 10U) - 1U)) * vref;
    *t1_voltage = conv_volt;
    
    ADC_DRV_ConfigChan(THROTTLE_ADC_INSTANCE, 0UL, &throttle_2_adConv_ChnConfig0);
	ADC_DRV_WaitConvDone(THROTTLE_ADC_INSTANCE);
    ADC_DRV_GetChanResult(THROTTLE_ADC_INSTANCE, 0U, (uint16_t *)&adc_throttle_2_voltage);
    
    conv_volt =  (1.0f / 0.0909f) * ((float_t)adc_throttle_2_voltage / ((1U << 10U) - 1U)) * vref;
    *t2_voltage = conv_volt;
    
}


static void highlow_beam_monitor(void)
{
	if (PINS_DRV_GetPortIntFlag(HL_IS_PORT) & (1 << HL_IS_PIN))
    {    
        /* Clear interrupt flag */
        PINS_ClearPortIntFlag(HL_IS_PORT, (1U << (HL_IS_PIN)));
    }
}

static void DRL_TL_monitor(void)
{
	if(PINS_DRV_GetPortIntFlag(DRL_TL_IS_PORT) & (1 << DRL_LED_EN_PIN))
    {    
       /* Clear interrupt flag */
        PINS_ClearPortIntFlag(DRL_TL_IS_PORT, (1U << (DRL_LED_EN_PIN)));
    }
}


static void rearbreak_numplate_monitor(void)	
{
	if(PINS_DRV_GetPortIntFlag(BR_NP_IS_PORT) & (1 << BR_NP_IS_PIN))
    {    
       /* Clear interrupt flag */
        PINS_ClearPortIntFlag(BR_NP_IS_PORT, (1U << (BR_NP_IS_PIN)));
    }
}


static void ABS_VCU_monitor(void)
{
	if (PINS_DRV_GetPortIntFlag(ABS_LED_IS_PORT) & (1 << ABS_LED_IS_PIN))
    {    
        /* Clear interrupt flag */
        PINS_ClearPortIntFlag(ABS_LED_IS_PORT, (1U << (ABS_LED_IS_PIN)));
    }
}


