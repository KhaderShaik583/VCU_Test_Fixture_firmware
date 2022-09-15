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
 
#include "lpi2c_driver.h"
#include "drv_PCA8565.h"
#include "status.h"
#include "board.h" 

/* Definition of the data transfer size */
#define RTC_TRANSFER_SIZE 	        (1U)
#define RTC_STARTUP_DELAY_MS        (100U)
#define RTC_API_TIMEOUT             (300U)
#define RTC_API_INF_TIMEOUT         (OSIF_WAIT_FOREVER)

/* Driver semaphore */
static semaphore_t drv_rtc_sem_t;

/*FUNCTION**********************************************************************
 *
 * Function Name : bcd_to_dec
 * Description   : converts given BCD value to Decimal,value in BCD to be converted into Decimal
 * and returns the converted Decimal value
 * 
 * Implements    : bcd_to_dec_Activity
 *END**************************************************************************/

static uint8_t bcd_to_dec(uint8_t  value)
{
  return (((value / 16U) * 10U) + (value % 16U));
}

/*FUNCTION**********************************************************************
 *
 * Function Name : dec_to_bcd
 * Description   : converts given  Decimal value to BCD,value in Decimal to be converted into  BCD
 * and returns the converted BCD value
 * 
 * Implements    : dec_to_bcd_Activity
 *END**************************************************************************/

static uint8_t dec_to_bcd(uint8_t value)
{
  return (((value / 10U) * 16U) + (value % 10U));
}

/*FUNCTION**********************************************************************
 *
 * Function Name : convert_datetime_to_dec
 * Description   : given date and time (BCD to Decimal), param timedate  structure pointer holds the date and time that read from RTC
 * and returns the structure pointer containing he converted(BCD to Decimal) date and time that read from RTC
 * Implements    : convert_datetime_to_dec_Activity
 *END**************************************************************************/

static void convert_datetime_to_dec(datetime_t timedate_bcd, datetime_t *timedate_dec)
{
    DEV_ASSERT(timedate_dec != NULL);
    
	timedate_dec->second    = bcd_to_dec(timedate_bcd.second);	
	timedate_dec->minute    = bcd_to_dec(timedate_bcd.minute);
	timedate_dec->hour      = bcd_to_dec(timedate_bcd.hour);
	timedate_dec->day       = bcd_to_dec(timedate_bcd.day);
	timedate_dec->month     = bcd_to_dec(timedate_bcd.month);
	timedate_dec->weekday   = bcd_to_dec(timedate_bcd.weekday);
	timedate_dec->year      = bcd_to_dec(timedate_bcd.year);
}

/*
 * @brief Perform conversion of given date and time
 * This function converts given date and time Decimal to BCD,
 * @param timedate  structure pointer holds the date and time of user_Datetime
 * @ returns the structure pointer containing converted(decimal to BCD) date and time of user_Datetime
 */
static void convert_datetime_to_bcd(datetime_t timedate_dec, datetime_t *timedate_bcd)
{
    DEV_ASSERT(timedate_bcd != NULL);
    
	timedate_bcd->second    = dec_to_bcd(timedate_dec.second);		
	timedate_bcd->minute    = dec_to_bcd(timedate_dec.minute);		
	timedate_bcd->hour      = dec_to_bcd(timedate_dec.hour);		
	timedate_bcd->day       = dec_to_bcd(timedate_dec.day);			
	timedate_bcd->month     = dec_to_bcd(timedate_dec.month);		
	timedate_bcd->weekday   = dec_to_bcd(timedate_dec.weekday);		
	timedate_bcd->year      = dec_to_bcd(timedate_dec.year);   		
}

/*
 * @brief Perform conversion of given alarm time in Decimal
 * This function converts given alarm time Decimal to BCD,
 * @param timedate  structure pointer holds the Alarmtime  given by user
 * @ returns the structure pointer containing converted(decimal to BCD) Alarmtime igven by user
 */
static void convert_alarmtime_to_bcd(alarmtime_t alarmtime_dec, alarmtime_t *alarmtime_bcd)
{
    DEV_ASSERT(alarmtime_bcd != NULL);
   
	alarmtime_bcd->alarm_minute    = dec_to_bcd(alarmtime_dec.alarm_minute);		/* Minutes, 0 - 59 */ 
	alarmtime_bcd->alarm_hour      = dec_to_bcd(alarmtime_dec.alarm_hour);			/* Hours, 0 - 12 */
	alarmtime_bcd->alarm_day       = dec_to_bcd(alarmtime_dec.alarm_day);			/* Days, 1 - 31 */
    alarmtime_bcd->alarm_weekday   = dec_to_bcd(alarmtime_dec.alarm_weekday);		
					
}

/*FUNCTION**********************************************************************
 *
 * Function Name : convert_alarmtime_to_dec
 * Description   : converts given alarm time (BCD to Decimal),
 * @param timedate  structure pointer holds the alarm time that read from RTC
 * @ returns the structure pointer containing he converted(BCD to Decimal) alarm time that read from RTC
 *
 * Implements    : convert_alarmtime_to_dec_Activity
 *END**************************************************************************/


/*
 * @brief Perform conversion of given alarm time in BCD
 * 
 */
static void convert_alarmtime_to_dec(alarmtime_t alarmtime_bcd, alarmtime_t *alarmtime_dec)
{
    DEV_ASSERT(alarmtime_dec != NULL);
   
	alarmtime_dec->alarm_minute    = bcd_to_dec(alarmtime_bcd.alarm_minute);		/* Minutes, 0 - 59 */ 
	alarmtime_dec->alarm_hour      = bcd_to_dec(alarmtime_bcd.alarm_hour);			/* Hours, 0 - 12 */
	alarmtime_dec->alarm_day       = bcd_to_dec(alarmtime_bcd.alarm_day);			/* Days, 1 - 31 */
	alarmtime_dec->alarm_weekday   = bcd_to_dec(alarmtime_bcd.alarm_weekday);		/* Weekdays 0 - 6 */
					
}

/*FUNCTION**********************************************************************
 *
 * Function Name : drv_rtc_validate_input
 * Description   : validating the Datetime input for setting RTC datetime
 *
 * Implements    : drv_rtc_validate_input_Activity
 *END**************************************************************************/

static status_t drv_rtc_validate_input(datetime_t settime_dec)
{
    volatile status_t s = STATUS_ERROR;
    
    if( ((settime_dec.second <= 59u)) &&		                            /* Seconds, 0 - 59 */
        ((settime_dec.minute <= 59U)) &&		                            /* Minutes, 0 - 59 */ 
        ((settime_dec.hour <= 23U)) &&			                            /* Hours, 0 - 12 */
        ((settime_dec.day >= 1U) && (settime_dec.day <= 31U)) &&			/* Days, 1 - 31 */
        ((settime_dec.weekday <= 6U)) &&		                            /* Weekdays 0 - 6 */
        ((settime_dec.month >= 1U) && (settime_dec.month <= 12U)) &&		/* Months, 1 - 12 */
		((settime_dec.year <= 99U)))			                            /* Years 0 - 99 */
    {
        s = STATUS_SUCCESS;
    }
    return s;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : drv_rtc_validate_alarm_input
 * Description   : validating the Datetime input for setting RTC Alarmtime
 *
 * Implements    : drv_rtc_validate_alarm_input_Activity
 *END**************************************************************************/

static status_t drv_rtc_validate_alarm_input(alarmtime_t alarmtime_dec)
{
	volatile status_t s = STATUS_ERROR;
	
    if(((alarmtime_dec.alarm_minute <= 59U))&&
       ((alarmtime_dec.alarm_hour <= 23U)) &&
       ((alarmtime_dec.alarm_day <= 31U)) &&
	   ((alarmtime_dec.alarm_weekday <= 6U)) )
	{
        s = STATUS_SUCCESS;
	}
        
     return s;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : drv_rtc_trancieve_blocking
 * Description   : RTC_ I2C transmit and receive combined blocking function 
 *
 * Implements    : drv_rtc_trancieve_blocking_Activity
 *END**************************************************************************/
static status_t drv_rtc_trancieve_blocking(uint8_t *tx_buffer, uint8_t tx_len, uint8_t *rx_buffer, uint8_t rx_len, uint32_t timeout)
{
    status_t s = STATUS_ERROR;
    
    /* setting the slave address for RTC device*/
	LPI2C_DRV_MasterSetSlaveAddr(RTC_I2C_IF, RTC_I2C_ADDR, false);
    if((tx_buffer != NULL) && (tx_len > 0U))
    {
        s = LPI2C_DRV_MasterSendDataBlocking(RTC_I2C_IF, tx_buffer, tx_len, true, timeout);
    }
    
    if((rx_buffer != NULL) && (rx_len > 0U))
    {
        s = LPI2C_DRV_MasterReceiveDataBlocking(RTC_I2C_IF, rx_buffer, rx_len, true, timeout);
    }
    
    return s;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : drv_rtc_clk_ctrl
 * Description   : disabling the RTC clock with given seegings
 *
* Implements     : drv_rtc_clk_ctrl_Activity
 *END**************************************************************************/

static status_t drv_rtc_clk_ctrl(uint8_t disable_clock)
{
	status_t status = STATUS_ERROR;
	uint8_t buffer[2];
	
	buffer[0] = RTC_CONTROL_1;

	if(disable_clock == 1U)
	{
		buffer[1] = RTC_CTRL1_STOP_SHIFT;
	}else 
	{
		buffer[1] = 0U;
	}

    status = drv_rtc_trancieve_blocking(buffer, 3U, NULL, 0U, RTC_API_TIMEOUT);    
    if(status != STATUS_SUCCESS)
    {
        set_status_bit(STAT_VCU_RTC_READ_FAILURE);
    }
    return status;
	
}

/*FUNCTION**********************************************************************
 *
 * Function Name : drv_rtc_enable_irq
 * Description   : enabling the alarm interrupt flag and setting respective indivitual enable bit
 *
 * Implements     : drv_rtc_enable_irq_Activity
 *END**************************************************************************/

static status_t drv_rtc_enable_irq(void)
{
	/* Enabling alarm interrupt */
	status_t status = STATUS_ERROR;
	uint8_t buffer[5];
    
	buffer[0] = RTC_CONTROL_2;
	buffer[1] = RTC_ALARM_INT_ENABLE;
    
    status = drv_rtc_trancieve_blocking(buffer, 2U, NULL, 0U, RTC_API_TIMEOUT);	
    if(status != STATUS_SUCCESS)
    {
        set_status_bit(STAT_VCU_RTC_READ_FAILURE);
    }
    
	return status;
}

status_t drv_rtc_sem_acquire(uint32_t timeout)
{
    status_t s;
    s = OSIF_SemaWait(&drv_rtc_sem_t, timeout);
    return s;
}


status_t drv_rtc_sem_release(void)
{
    status_t s;
    s = OSIF_SemaPost(&drv_rtc_sem_t);
    return s;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : drv_rtc_init
 * Description   : Initializes RTC control registers and Timer control registers
 *
 * Implements    : drv_rtc_init_Activity
 *END**************************************************************************/

status_t drv_rtc_init(void)
{
	uint8_t buffer[3];
    status_t rtc_status = STATUS_ERROR;
	
    rtc_status = osif_sem_create(&drv_rtc_sem_t, 0U);
    DEV_ASSERT(rtc_status == STATUS_SUCCESS);
	
    osif_time_delay(RTC_STARTUP_DELAY_MS);

	/*  writing into the RTC_CONTROL_1 and RTC_CONTROL_2 registers of RTC */
	buffer[0] = RTC_CONTROL_1;
	buffer[1] = 0x00U;
	buffer[2] = 0x00U; 
	
    /* configuring RTC_CONTROL_1 & RTC_CONTROL_2 registers */
    rtc_status = drv_rtc_trancieve_blocking(buffer, 3U, NULL, 0U, RTC_API_TIMEOUT);
    if(rtc_status != STATUS_SUCCESS)
    {
        set_status_bit(STAT_VCU_RTC_INIT_FAILURE);
    }

	/* configuring the CLOCKOUT and TIMER_CONTROL registers of RTC   */
	buffer[0] = RTC_CLKOUT_CONTROL;
	buffer[1] = RTC_CLKOUT_ENABLE | RTC_CLKOUT_1_HZ;
    
	/* sending RTC_CLKOUT_CONTROL reg Address */
    rtc_status = drv_rtc_trancieve_blocking(buffer, 2U, NULL, 0U, RTC_API_TIMEOUT);	
    if(rtc_status != STATUS_SUCCESS)
    {
        set_status_bit(STAT_VCU_RTC_INIT_FAILURE);
    }
    
    return rtc_status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : drv_rtc_set
 * Description   : Writing the date time to the RTC
 *
 * Implements    : drv_rtc_set_Activity
 *END**************************************************************************/

status_t drv_rtc_set(datetime_t settime_dec)
{
    datetime_t time_bcd;
    uint8_t buffer[8];
	status_t status = STATUS_SUCCESS;
    
    if(drv_rtc_validate_input(settime_dec) == STATUS_SUCCESS)
    {
        convert_datetime_to_bcd(settime_dec, &time_bcd);

        buffer[0] = RTC_SEC;
        
        buffer[1] = time_bcd.second;
        buffer[2] = time_bcd.minute;
        buffer[3] = time_bcd.hour;
        buffer[4] = time_bcd.day;
        buffer[5] = time_bcd.weekday;
        buffer[6] = time_bcd.month;
        buffer[7] = time_bcd.year;
        
        status |= drv_rtc_clk_ctrl(1);

        /* Writing the Word Address(seconds_reg's)from which data need to be written into RTC */
        status |= drv_rtc_trancieve_blocking(buffer, 8U, NULL, 0U, RTC_API_TIMEOUT);	
        status |= drv_rtc_clk_ctrl(0);
        if(status != STATUS_SUCCESS)
        {
            set_status_bit(STAT_VCU_RTC_READ_FAILURE);
        }

    }

	return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : drv_rtc_read
 * Description   : Reads the current date and time from RTC,
 *
 * Implements     : drv_rtc_read_Activity
 *END**************************************************************************/

status_t drv_rtc_read(datetime_t *curr_time_dec)
{
    uint8_t buffer[7];
	uint8_t rtc_sec_addr = RTC_SEC;
    datetime_t current_date_time;
	status_t status = STATUS_ERROR;
    
    DEV_ASSERT(curr_time_dec != NULL);
    
    status = drv_rtc_trancieve_blocking(&rtc_sec_addr, RTC_TRANSFER_SIZE, buffer, 7U, RTC_API_TIMEOUT); 
    
    if(status == STATUS_SUCCESS)
    {
        /* Check the V bit to verify clock integrity */
        if((buffer[0] & 0x80U) == 0x00U)
        {
            current_date_time.valid     =   1U;
            current_date_time.second    =   (buffer[0] & RTC_SECONDS_MASK);
            current_date_time.minute    =   (buffer[1] & RTC_MINUTES_MASK);
            current_date_time.hour      =   (buffer[2] & RTC_HOUR_MASK);
            current_date_time.day       =   (buffer[3] & RTC_DAY_MASK);
            current_date_time.weekday   =   (buffer[4] & RTC_WEEKDAY_MASK);
            current_date_time.month     =   (buffer[5] & RTC_MONTH_MASK);
            current_date_time.year      =   (buffer[6] & RTC_YEAR_MASK);	
            
            convert_datetime_to_dec(current_date_time, curr_time_dec);
            clear_status_bit(STAT_VCU_RTC_READ_FAILURE);
        }
    }
    else
    {
        set_status_bit(STAT_VCU_RTC_READ_FAILURE);
    }
    
	return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : drv_rtc_clear_alarm_flag
 * Description   : clearing the alarm interrupt flag
 *
 * Implements     : drv_rtc_clear_alarm_flag_Activity
 *END**************************************************************************/

void drv_rtc_clear_alarm_flag(void)
{
	uint8_t buffer[2];
    status_t status = STATUS_ERROR;
    
	buffer[0] = RTC_CONTROL_2;
	buffer[1] = RTC_CLEAR_ALARM_FLAG;

    status = drv_rtc_trancieve_blocking(buffer, 2U, NULL, 0U, RTC_API_TIMEOUT);
    if(status != STATUS_SUCCESS)
    {
        set_status_bit(STAT_VCU_RTC_READ_FAILURE);
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : drv_set_rtc_alarm
 * Description   : Perform Setting the alarm minute register
 *
 * Implements     : drv_set_rtc_alarm_Activity
 *END**************************************************************************/

status_t drv_rtc_set_alarm(alarmtime_t set_alarm_dec, uint8_t which_alarm)
{
	status_t status = STATUS_ERROR;
	uint8_t buffer[5];
	alarmtime_t set_alarm_bcd;
	
	/*   checking the given minutes input of alarm is valid */
	if(drv_rtc_validate_alarm_input(set_alarm_dec) == STATUS_SUCCESS)
    {
		convert_alarmtime_to_bcd(set_alarm_dec, &set_alarm_bcd);       

		buffer[0] = RTC_ALARM_MIN;

        /* setting the alarm for given minutes */
        if((which_alarm & RTC_MINUTE_ALM_IRQ_N) == RTC_MINUTE_ALM_IRQ_N)
        {
            buffer[1] = (set_alarm_bcd.alarm_minute & 0x7FU) | 0x00U;
        }
        else
        {
            buffer[1] = (set_alarm_bcd.alarm_minute & 0x7FU) | 0x80U;
        }
		
        /* setting the alarm for given hours */
        if((which_alarm & RTC_HOUR_ALM_IRQ_N) == RTC_HOUR_ALM_IRQ_N)
        {
            buffer[2] = (set_alarm_bcd.alarm_hour & 0x7FU) | 0x00U;		
        }
        else
        {
            buffer[2] = (set_alarm_bcd.alarm_hour & 0x7FU) | 0x80U;		
        }
        
        /* setting the alarm for given days */
        if((which_alarm & RTC_DAY_ALM_IRQ_N) == RTC_DAY_ALM_IRQ_N)
        {
            buffer[3] = (set_alarm_bcd.alarm_day & 0x7FU) | 0x00U;		
        }
        else
        {
            buffer[3] = (set_alarm_bcd.alarm_day & 0x7FU) | 0x80U;		
        }
        
        /* setting the alarm for given weekday */
        if((which_alarm & RTC_WEEKDAY_ALM_IRQ_N) == RTC_WEEKDAY_ALM_IRQ_N)
        {
            buffer[4] = (set_alarm_bcd.alarm_weekday & 0x7FU) | 0x00U;		
        }
        else
        {
            buffer[4] = (set_alarm_bcd.alarm_weekday & 0x7FU) | 0x80U;		
        }

        status = drv_rtc_trancieve_blocking(buffer, 5U, NULL, 0U, RTC_API_TIMEOUT);
        if(status != STATUS_SUCCESS)
        {
            set_status_bit(STAT_VCU_RTC_READ_FAILURE);
        }
        
        /* enabling the alarm_minute IRQ  */
        drv_rtc_enable_irq();					    
	}
	return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : drv_get_rtc_alarm
 * Description   : Read the RTC current alarm settings
 *
 * Implements    : drv_get_rtc_alarm_Activity
 *END**************************************************************************/

status_t drv_rtc_get_alarm(alarmtime_t *get_alarm_dec)
{
	uint8_t rtc_reg_addr = RTC_ALARM_MIN;
	status_t status = STATUS_SUCCESS;
	uint8_t buffer[4];
	alarmtime_t get_alarm_bcd;

    status = drv_rtc_trancieve_blocking(&rtc_reg_addr, RTC_TRANSFER_SIZE, buffer, 4U, RTC_API_TIMEOUT); 	
		
    get_alarm_bcd.alarm_minute	 = buffer[0] & RTC_MIN_ALARM_MASK;							/* getting the alarm for given minutes */
	get_alarm_bcd.alarm_hour	 = buffer[1] & RTC_HOUR_ALARM_MASK;							/* getting the alarm for given hours */
	get_alarm_bcd.alarm_day 	 = buffer[2] & RTC_DAY_ALARM_MASK;							/* getting the alarm for given days */
	get_alarm_bcd.alarm_weekday	 = buffer[3] & RTC_WEEKDAY_ALARM_MASK;						/* getting the alarm for given weekdays */
		
	convert_alarmtime_to_dec(get_alarm_bcd, (alarmtime_t*)&get_alarm_dec);
    
	return status;
}
