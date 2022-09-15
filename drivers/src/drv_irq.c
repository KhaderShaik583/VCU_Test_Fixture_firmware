 /*
 * 
 * ULTRAVIOLETTE AUTOMOTIVE CONFIDENTIAL
 * ______________________________________
 * 
 * [2017] - [2018] Ultraviolette Automotive Pvt. Ltd.
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

#include "fw_common.h"
#include "pins_driver.h"
#include "pins_port_hw_access.h" 
#include "board.h" 

#include "drv_led.h"
#include "cd1030.h"
#include "imu.h"

#include "sleep.h"

void PORTA_IRQHandler(void);
void PORTB_IRQHandler(void);
void PORTC_IRQHandler(void);
void PORTD_IRQHandler(void);
void PORTE_IRQHandler(void);

void PORTA_IRQHandler(void)
{
    volatile uint32_t pins_gpioa = 0U;  
    
    pins_gpioa = PINS_GetPortIntFlag(KEY_WAKE_SIG_PORT);
    
    if(pins_gpioa > 0)
    {
        if(PINS_DRV_GetPortIntFlag(KEY_WAKE_SIG_PORT) & (1U << KEY_WAKE_SIG_PIN))
        {
            sw_asm_delay_us(SW_DEBOUNCE_DELAY_US);
            key_sw_irq();
        }
    }
}

void PORTB_IRQHandler(void)
{
    volatile uint32_t pins_value = 0U;
    
#ifdef USE_FEATURE_LP_SLEEP_MODE
    pins_value = PINS_GetPortIntFlag(DBA_CAN_RX_PORT);
    sleep_set_pin(pins_value);
    PINS_ClearPortIntFlag(DBA_CAN_RX_PORT, (1U << (DBA_CAN_RX_PIN)));
	NVIC_ClearPendingIRQ(PORTB_IRQn);
#else
    __NOP();
#endif
}

void PORTC_IRQHandler(void)
{
    volatile uint32_t pins_gpioc = 0U;  

    pins_gpioc = PINS_GetPortIntFlag(SWIF_IRQ_PORT);
    
    if(pins_gpioc > 0U)
    {
        pins_gpioc = PINS_GetPortIntFlag(SWIF_IRQ_PORT);
        
        if(pins_gpioc & (1U << SWIF_IRQ_PIN))
        {
            cd1030_irq();
        }
    }
    
    NVIC_ClearPendingIRQ(PORTC_IRQn);
}

void PORTD_IRQHandler(void)
{
    if(PINS_DRV_GetPortIntFlag(IMU_IRQ_PORT) & (1U << IMU_IRQ_PIN))
    {
        imu_read_data_irq();
        PINS_ClearPortIntFlag(IMU_IRQ_PORT, (1U << (IMU_IRQ_PIN)));
    }
}

void PORTE_IRQHandler(void)
{

}


