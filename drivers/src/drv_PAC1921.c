#include "drv_PAC1921.h"
#include "drv_PCA8565.h"

#include "lpi2c_driver.h"
#include "status.h"

/* Current Monitor PAC1921 */
#define PAC1921_TRANSFER_SIZE		(1U)

/* Driver semaphore */
static semaphore_t drv_PAC1921_sem_t;

static status_t send_byte(uint8_t register_addr, const uint8_t *transfer_data)
{
	status_t ctm_status = STATUS_SUCCESS;
	uint8_t buffer[2];
	buffer[0] = register_addr;
	buffer[1] = *transfer_data;
	
	/* setting the slave address for PAC1921 device*/
	LPI2C_DRV_MasterSetSlaveAddr(PAC1921_I2C_IF, PAC1921_SLAVE_ADDRESS, false);
	ctm_status |= LPI2C_DRV_MasterSendDataBlocking(PAC1921_I2C_IF, buffer, 2U, true, OSIF_WAIT_FOREVER);   
	
	return ctm_status;
}

status_t drv_PAC1921_sem_acquire(uint32_t timeout)
{
    status_t s;
    s = osif_sem_acquire(&drv_PAC1921_sem_t, timeout);
    return s;
}


status_t drv_PAC1921_sem_release(void)
{
    status_t s;
    s = osif_sem_release(&drv_PAC1921_sem_t);
    return s;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : send_byte
 * Description   : Intialize different configuration registers and control register for Current Monitor 
 * Implements    : send_to_motor_controller_Activity
 *END****************************************************************/
status_t drv_PAC1921_init(void)
{
	uint8_t buffer = 0U;  
	status_t ctm_status = STATUS_SUCCESS;
	status_t s;
    
    s = osif_sem_create(&drv_PAC1921_sem_t, 0U);
    DEV_ASSERT(s == STATUS_SUCCESS);
    
	
	/* sending data to GAIN_Configuration register  */
	
	/* 	sending GAIN_CONFG reg Address */
	/*	buffer data = 0xC0U i.e 0x11000000;	1 VSENS ADC Measurement Resolution 10-bit
	 * 									 	1 VBUS ADC Measurement Resolution 10-bit 
	 *										000 Digital Current gain
	 * 										000 Digital Voltage Gain
	*/
	buffer = (PAC1921_ADC_RESOLUTION_VSENSE_11_BIT | PAC1921_ADC_RESOLUTION_VBUS_11_BIT);								
	send_byte(PAC1921_GAIN_CONFG_REG, &buffer);
	
	/* 	sending data to Integration_configuration register	*/
	/*	buffer data = 0x3DU i.e 0x00111101;	0011 considering 8 as No.of samples for selected measurement type */
	buffer = (PAC1921_64_SAMPLES | PAC1921_VSENSE_ADC_FILTER_ENABLE | PAC1921_VBUS_ADC_FILTER_ENABLE | PAC1921_INTEGRATE_STATE);		
	send_byte(PAC1921_INTG_CONFG_REG, &buffer);
	
	/*	Sending data to CONTROL register	*/
	/*	No recalculation of measurement, TOUT diabled, SLEEP disabled, Sleep Override Disabled	*/ 
	buffer = (PAC1921_VBUS_FREE_RUN | PAC1921_FORCED_READ_MODE | PAC1921_RECALCULATE_DAC_UPDATE_MODE);
	send_byte(PAC1921_CONTROL_REG, &buffer);
	
	return ctm_status;
	
}

status_t drv_PAC1921_mux_sel(uint32_t mux)
{
    uint8_t buffer = 0U;  
    status_t s = STATUS_SUCCESS;
    
    if(mux == PAC1921_SEL_VBUS_MUX)
    {
        buffer = (PAC1921_VBUS_FREE_RUN | PAC1921_FORCED_READ_MODE | PAC1921_RECALCULATE_DAC_UPDATE_MODE);
        send_byte(PAC1921_CONTROL_REG, &buffer);
    }
    else if(mux == PAC1921_SEL_VSNS_MUX)
    {
        buffer = (PAC1921_VSENSE_FREE_RUN | PAC1921_FORCED_READ_MODE | PAC1921_RECALCULATE_DAC_UPDATE_MODE);
        send_byte(PAC1921_CONTROL_REG, &buffer);
    }
    else
    {
        __NOP();
    }
    
    return s;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : drv_PAC1921_read_8bits
 * Description   : reading Vbus register data / any other register of PAC1921 
 * Implements    : drv_PAC1921_read_8bits_Activity
 *END****************************************************************/


status_t drv_PAC1921_read_8bits(uint8_t PAC1921_reg8_addr, uint8_t *reg_data)
{
	status_t ctm_status = STATUS_SUCCESS;
	uint8_t ctm_reg = PAC1921_reg8_addr;
	uint8_t buffer;
	
	/* setting the slave address for PAC1921 device*/
	LPI2C_DRV_MasterSetSlaveAddr(PAC1921_I2C_IF, PAC1921_SLAVE_ADDRESS, false);
    
	/* Writing the Word Address(VBUS_RESULT's)from which data need to be written into RTC	*/
	ctm_status |= LPI2C_DRV_MasterSendDataBlocking(PAC1921_I2C_IF, &ctm_reg, PAC1921_TRANSFER_SIZE, true, OSIF_WAIT_FOREVER);	
	/* Reading Time and Date from respective CTM register	*/
	ctm_status |= LPI2C_DRV_MasterReceiveDataBlocking(PAC1921_I2C_IF, &buffer, PAC1921_TRANSFER_SIZE, true, OSIF_WAIT_FOREVER);	
	*reg_data = buffer;	
	return ctm_status;
}

/* Function foe reading Vbus register data / any other register of PAC1921	
 * para :	read_reg -> PAC1921 register address 
 *			reg_data -> the array to store the 2 bytes read from given CTM register address
 */



/* function to read 2 byte registers such as VBUS result, VSENSE result Samples register , product ID ,Manufacture ID, Revision*/
status_t drv_PAC1921_read_16bits(uint8_t PAC1921_reg16_addr, uint8_t *reg_data)
{
	status_t ctm_status = STATUS_SUCCESS;
	uint8_t ctm_reg = PAC1921_reg16_addr;
	uint8_t buffer[2];
	
	/* setting the slave address for PAC1921 device*/
	LPI2C_DRV_MasterSetSlaveAddr(PAC1921_I2C_IF, PAC1921_SLAVE_ADDRESS, false);
	
	ctm_status |= LPI2C_DRV_MasterSendDataBlocking(PAC1921_I2C_IF, &ctm_reg, PAC1921_TRANSFER_SIZE, true, OSIF_WAIT_FOREVER);	
    
	/* Reading 2 bytes from respective CTM register	*/
	ctm_status |= LPI2C_DRV_MasterReceiveDataBlocking(PAC1921_I2C_IF, buffer, 2U, true, OSIF_WAIT_FOREVER);	
	reg_data[0] = buffer[0];
	reg_data[1] = buffer[1];
	
	return ctm_status;
}

/* Function for reading Vbus register data / any other register of PAC1921	
 * para :	read_reg -> PAC1921  register address 
 *			reg_data -> the array to store the 4 bytes read from given CTM register address
 */

/* function to read 2 byte registers such as VSSUM Accumulator , ISUM Accumulator, PSUM Accuulator ,PSUM Accumulator */
status_t drv_PAC1921_read_32bits(uint8_t PAC1921_reg32_addr, uint8_t *reg_data)
{
	status_t ctm_status = STATUS_SUCCESS;
	uint8_t ctm_reg = PAC1921_reg32_addr;
	uint8_t buffer[4];
	
	/* setting the slave address for PAC1921 device*/
	LPI2C_DRV_MasterSetSlaveAddr(PAC1921_I2C_IF, PAC1921_SLAVE_ADDRESS, false);
	
	ctm_status |= LPI2C_DRV_MasterSendDataBlocking(PAC1921_I2C_IF, &ctm_reg, PAC1921_TRANSFER_SIZE, true, OSIF_WAIT_FOREVER);	
	/* Reading 4 bytes from respective CTM register	*/
	ctm_status |= LPI2C_DRV_MasterReceiveDataBlocking(PAC1921_I2C_IF, buffer, 4U, true, OSIF_WAIT_FOREVER);	
	reg_data[0] = buffer[0];
	reg_data[1] = buffer[1];
	reg_data[2] = buffer[2];
	reg_data[3] = buffer[3];
	
	return ctm_status;
}
