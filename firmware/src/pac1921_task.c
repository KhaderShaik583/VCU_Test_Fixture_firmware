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
 
#include "pac1921_task.h"
#include "lpi2c_driver.h"
#include "sem_common.h"


#define SHUNT_RES   (0.005f)

#define PAC_SM_STATE_ENTER_READ         (0U)
#define PAC_SM_STATE_READ_VBUS          (1U)
#define PAC_SM_STATE_READ_VSNS          (2U)
#define PAC_SM_STATE_VBUS_SET           (3U)
#define PAC_SM_STATE_VSNS_SET           (4U)
#define PAC_SM_STATE_INTEGRATE          (5U)

static volatile float_t vbus_voltage = 0.0f;
static volatile float_t vbus_current = 0.0f;
static const float_t vbus_min_threshold = 11.0f;


static void pac1921_get_vbus_voltage(float_t *v)
{
    uint8_t PAC1921_reg16_data[2] = {0U, 0U};
    volatile uint16_t vbus_digital_val = 0U;
    
    PINS_DRV_ClearPins(PAC9129_IRQ_GPIO, (1U << PAC9129_IRQ_PIN));
    sw_asm_delay_us(10);
    PINS_DRV_SetPins(PAC9129_IRQ_GPIO, (1U << PAC9129_IRQ_PIN));
    
    (void)drv_PAC1921_sem_acquire(10U);
    (void)drv_PAC1921_read_16bits(PAC1921_VBUS_RESULT_REG, PAC1921_reg16_data);
    (void)drv_PAC1921_sem_release();
    
    vbus_digital_val = (PAC1921_reg16_data[1] >> 6U);
    vbus_digital_val |=  ((uint16_t)PAC1921_reg16_data[0] << 2U);
    
    vbus_voltage = (float_t)(vbus_digital_val * 32.0f) / 1023.0f;
    
    sw_asm_delay_us(100);
    PINS_DRV_SetPins(PAC9129_IRQ_GPIO, (1U << PAC9129_IRQ_PIN));
    
    if(vbus_voltage < vbus_min_threshold)
    {
        set_status_bit(STAT_VCU_LAC_BUS_LOW_VOLTAGE_WARNING);
    }
    else
    {
        clear_status_bit(STAT_VCU_LAC_BUS_LOW_VOLTAGE_WARNING);
    }
    
    if(v != NULL)
    {
        *v = vbus_voltage;
    }
}

static void pac1921_get_vbus_current(float_t *i)
{
    uint8_t PAC1921_reg16_data[2] = {0U, 0U};
    volatile uint16_t vbus_digital_val = 0U;
    
    PINS_DRV_ClearPins(PAC9129_IRQ_GPIO, (1U << PAC9129_IRQ_PIN));
    sw_asm_delay_us(10);
    PINS_DRV_SetPins(PAC9129_IRQ_GPIO, (1U << PAC9129_IRQ_PIN));
    
    (void)drv_PAC1921_sem_acquire(10U);
    (void)drv_PAC1921_read_16bits(PAC1921_VSENSE_RESULT_REG, PAC1921_reg16_data);
    (void)drv_PAC1921_sem_release();
    
    vbus_digital_val = (PAC1921_reg16_data[1] >> 6U);
    vbus_digital_val |=  ((uint16_t)PAC1921_reg16_data[0] << 2U);
    
    vbus_current = (float_t)(vbus_digital_val * (0.1f / SHUNT_RES)) / 1023.0f;
    
    sw_asm_delay_us(100);
    PINS_DRV_SetPins(PAC9129_IRQ_GPIO, (1U << PAC9129_IRQ_PIN));
    
    if(i != NULL)
    {
        *i = vbus_current;
    }
}

void pac1921_read_bus(float_t *v, float_t *i)
{
    if((v != NULL) && (i != NULL))
    {
        *v = vbus_voltage;
        *i = vbus_current;
    }
}

void pac1921_ops(void)
{
    static uint32_t op = 0U;
    /*
        16 measurement slots.
        4 for voltage measurement.
        12 for current measurement.
    */
    static const uint32_t vbus_read_count = 4U;
    static const uint32_t vsns_read_count = 12U;
    
    if(op < vbus_read_count)
    {
        pac1921_get_vbus_voltage(NULL);

        if(op == (vbus_read_count - 1U))
        {
            PINS_DRV_ClearPins(PAC9129_IRQ_GPIO, (1U << PAC9129_IRQ_PIN));
            sw_asm_delay_us(10);
            PINS_DRV_SetPins(PAC9129_IRQ_GPIO, (1U << PAC9129_IRQ_PIN));
            
            (void)drv_PAC1921_sem_acquire(10U);
            (void)drv_PAC1921_mux_sel(PAC1921_SEL_VSNS_MUX);
            (void)drv_PAC1921_sem_release();
            
            sw_asm_delay_us(100);
            PINS_DRV_SetPins(PAC9129_IRQ_GPIO, (1U << PAC9129_IRQ_PIN));
        }
        
        op = op + 1U;
    }
    else if(op < (vbus_read_count + vsns_read_count))
    {
        pac1921_get_vbus_current(NULL);

        if(op == ((vbus_read_count + vsns_read_count) - 1U))
        {
            PINS_DRV_ClearPins(PAC9129_IRQ_GPIO, (1U << PAC9129_IRQ_PIN));
            sw_asm_delay_us(10);
            PINS_DRV_SetPins(PAC9129_IRQ_GPIO, (1U << PAC9129_IRQ_PIN));
            
            (void)drv_PAC1921_sem_acquire(10U);
            (void)drv_PAC1921_mux_sel(PAC1921_SEL_VBUS_MUX);
            (void)drv_PAC1921_sem_release();
            
            sw_asm_delay_us(100);
            PINS_DRV_SetPins(PAC9129_IRQ_GPIO, (1U << PAC9129_IRQ_PIN));
        }
        
        op = op + 1U;
    }
    else
    {
        op = 0U;
    }
}

