#ifndef DRV_PAC1921_H
#define DRV_PAC1921_H

#include <stdint.h>
#include "board.h"

/*	PAC Slave address    */

#define PAC1921_SLAVE_ADDRESS	(0x98 >> 1U)

/* Current Monitor PAC1921 Registers Addresses */
#define PAC1921_GAIN_CONFG_REG						0x00U
#define PAC1921_INTG_CONFG_REG						0x01U
#define PAC1921_CONTROL_REG			 				0x02U
#define PAC1921_VBUS_RESULT_REG						0x10U
//#define PAC1921_VBUS_HIGH_BYTE_REG				0x10U
//#define PAC1921_VBUS_LOW_BYTE_REG					0x11U
		
#define PAC1921_VSENSE_RESULT_REG					0x12U
//#define PAC1921_VSENSE_HIGH_BYTE_REG				0x12U
//#define PAC1921_VSENSE_LOW_BYTE_REG				0x13U

#define PAC1921_VSUM_ACC_REG						0x14U
//#define PAC1921_VSUM_HIGH_BYTE_REG				0x14U
//#define PAC1921_VSUM_MIDDLE_HIGH_BYTE_REG			0x15U
//#define PAC1921_VSUM_MIDDLE_LOW_BYTE_REG			0x16U
//#define PAC1921_VSUM_LOW_BYTE_REG					0x17U

#define PAC1921_ISUM_ACC_REG						0x18U
//#define PAC1921_ISUM_HIGH_BYTE_REG				0x18U
//#define PAC1921_ISUM_MIDDLE_HIGH_REG				0x19U
//#define PAC1921_ISUM_MIDDLE_LOW_REG				0x1AU
//#define PAC1921_ISUM_LOW_BYTE_REG					0x1BU

#define PAC1921_OVERFLOW_STATUS_REG					0x1CU
#define PAC1921_VPOWER_RESULT_REG					0x1DU
//#define PAC1921_VPOWER_HIGH_BYTE_REG				0x1DU
//#define PAC1921_VPOWER_LOW_BYTE_REG				0x1EU

#define PAC1921_SAMPLES_REG							0x21U
//#define PAC1921_SAMPLES_HIGH_BYTE_REG				0x21U
//#define PAC1921_SAMPLES_LOW_BYTE_REG				0x22U

#define PAC1921_PSUM_ACC_REG						0x23U
//#define PAC1921_PSUM_HIGH_BYTE_REG				0x23U
//#define PAC1921_PSUM_MIDDLE_HIGH_REG				0x24U
//#define PAC1921_PSUM_MIDDLE_REG					0x25U
//#define PAC1921_PSUM_MIDDLE_LOW_REG				0x26U
//#define PAC1921_PSUM_LOW_BYTE_REG					0x27U

#define PAC1921_PID_REG								0xFDU
#define PAC1921_MID_REG								0xFEU
#define PAC1921_REVID_REG							0xFFU


/*	Control bits for PAC1921_GAIN_CONFIG_REG  Register */

#define PAC1921_ADC_RESOLUTION_14_BIT_GAIN_1X  		0x00U
#define PAC1921_ADC_RESOLUTION_VSENSE_11_BIT 		0x80U
#define PAC1921_ADC_RESOLUTION_VBUS_11_BIT 			0x40U
#define PAC1921_DIG_CURRENT_GAIN_2X 				0x08U
#define PAC1921_DIG_CURRENT_GAIN_4X 				0x10U
#define PAC1921_DIG_CURRENT_GAIN_8X					0x18U
#define PAC1921_DIG_CURRENT_GAIN_16X				0x20U
#define PAC1921_DIG_CURRENT_GAIN_32X 				0x28U
#define PAC1921_DIG_CURRENT_GAIN_64X 				0x30U
#define PAC1921_DIG_CURRENT_GAIN_128X 				0x38U
#define PAC1921_DIG_BUS_VOLTAGE_GAIN_2X				0x01U
#define PAC1921_DIG_BUS_VOLTAGE_GAIN_4X 			0x02U
#define PAC1921_DIG_BUS_VOLTAGE_GAIN_8X 			0x03U
#define PAC1921_DIG_BUS_VOLTAGE_GAIN_16X 			0x04U
#define PAC1921_DIG_BUS_VOLTAGE_GAIN_32X 			0x05U


/*	Control bits for PAC1921_INTERGRATION_CONFIG_REG  Register */

#define PAC1921_1_SAMPLE_ADC_FILTER_DISABLE_OVERRIDE_DISABLE_READ_STATE  0x00U
#define PAC1921_2_SAMPLES							0x10U
#define PAC1921_4_SAMPLES 							0x20U
#define PAC1921_8_SAMPLES 							0x30U
#define PAC1921_16_SAMPLES 							0x40U
#define PAC1921_32_SAMPLES							0x50U
#define PAC1921_64_SAMPLES 							0x60U
#define PAC1921_128_SAMPLES 						0x70U
#define PAC1921_256_SAMPLES 						0x80U
#define PAC1921_512_SAMPLES 						0x90U
#define PAC1921_1024_SAMPLES 						0xA0U
#define PAC1921_2048_SAMPLES 						0xB0U
#define PAC1921_VSENSE_ADC_FILTER_ENABLE 			0x08U
#define PAC1921_VBUS_ADC_FILTER_ENABLE 				0x04U
#define PAC1921_OVERRIDE_ENABLE 					0x02U
#define PAC1921_INTEGRATE_STATE 					0x01U

/*	Control bits for PAC1921_CONTROL_REG  Register */

#define PAC1921_VPOWER_PIN_CONTROLLED_3000MV_FULL_SCALE_TIMEOUT_DIS_NORMAL_OP  0x00U
#define PAC1921_VSENSE_FREE_RUN 					0x40U
#define PAC1921_VBUS_FREE_RUN 						0x80U
#define PAC1921_VPOWER_FREE_RUN 					0xC0U
#define PAC1921_2000MV_FULL_SCALE_RANGE 			0x10U
#define PAC1921_1500MV_FULL_SCALE_RANGE				0x20U
#define PAC1921_1000MV_FULL_SCALE_RANGE				0x30U
#define PAC1921_TIMEOUT_ENABLE  					0x08U
#define PAC1921_SLEEP_STATE							0x04U
#define PAC1921_FORCED_READ_MODE 					0x02U
#define PAC1921_RECALCULATE_DAC_UPDATE_MODE 		0x01U

#define PAC1921_SEL_VBUS_MUX    (0U)
#define PAC1921_SEL_VSNS_MUX    (1U)

/* Function to Intialize different configuration registers and control register for Current Monitor */

status_t drv_PAC1921_init(void);
status_t drv_PAC1921_sem_acquire(uint32_t timeout);
status_t drv_PAC1921_sem_release(void);
/*
 *	Function for reading the desired register data
 *	parameters to send 1. desired register address 
 *										 2. data (2bytes) read from register
 */

//static status_t send_byte(uint8_t , uint8_t);

status_t drv_PAC1921_read_8bits(uint8_t , uint8_t *);
status_t drv_PAC1921_read_16bits(uint8_t, uint8_t *);
status_t drv_PAC1921_read_32bits(uint8_t, uint8_t *);
//uint8_t drv_ctm_read_64bits(uint8_t, uint8_t **);

//uint8_t write_byte(void);
//uint8_t send_byte(void);
//uint8_t receive_byte(void);
//uint8_t write_block(void);
status_t drv_PAC1921_mux_sel(uint32_t mux);

#endif /* DRV_PAC1921_H */
