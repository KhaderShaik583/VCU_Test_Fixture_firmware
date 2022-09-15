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
 
#include "rtc_task.h"
#include "sem_common.h"
#include "nvm.h"

static datetime_t local_dt;
static datetime_t local_nw_dt;
static uint32_t rtc_update_nw_time = 0U;

/**
 * @brief Reads the date & time from the RTC.
 * 
 *
 * @param date_time - Pointer to datetime_t struct 
 */
static void rtc_get_date_time(datetime_t *date_time)
{
    (void)lpi2c_sem_acquire(10U);
    (void)drv_rtc_read(date_time);
    (void)lpi2c_sem_release();
}

static status_t rtc_set_date_time(datetime_t date_time)
{
    status_t s = STATUS_SUCCESS;

    (void)lpi2c_sem_acquire(10U);
    s = drv_rtc_set(date_time);
    (void)lpi2c_sem_release();
    
    return s;
}

/**
 * @brief Sets RTC time as received from the network.
 * @param date_time - Pointer to datetime_t struct 
 */
void rtc_set_date_time_nw(datetime_t *date_time)
{
    if(date_time != NULL)
    {
        local_nw_dt.day     = date_time->day;
        local_nw_dt.hour    = date_time->hour;
        local_nw_dt.minute  = date_time->minute;
        local_nw_dt.month   = date_time->month;
        local_nw_dt.second  = date_time->second;
        local_nw_dt.valid   = date_time->valid;
        local_nw_dt.weekday = date_time->weekday;
        local_nw_dt.year    = date_time->year;
        
        rtc_update_nw_time = 1U;
    }
}

/**
 * @brief Initiates a read from RTC into local datetime struct
 * 
 */
void rtc_trig_measurement(void)
{
    if(rtc_update_nw_time == 1U)
    {
        (void)rtc_set_date_time(local_nw_dt);
        rtc_update_nw_time = 0U;
    }
    
    rtc_get_date_time(&local_dt);
}

/**
 * @brief Returns the last value read into local_dt
 * Function is re-entrant.
 * 
 * @param dt - Pointer to datetime_t struct 
 */
void rtc_read_local_time(datetime_t *dt)
{
    if(dt != NULL)
    {
        dt->day = local_dt.day;
        dt->hour = local_dt.hour;
        dt->minute = local_dt.minute;
        dt->second = local_dt.second;
        dt->weekday = local_dt.weekday;
        dt->year = local_dt.year;
    }
}

void rtc_setup_alarm(void)
{
    datetime_t dt_dec;
    alarmtime_t alrm_dec;
    
    (void)drv_rtc_read(&dt_dec);

    /* set alarm two minutes from now */
    alrm_dec.alarm_minute   = dt_dec.minute + 1U;
    alrm_dec.alarm_hour     = dt_dec.hour;
    alrm_dec.alarm_day      = dt_dec.day;
    alrm_dec.alarm_weekday  = dt_dec.weekday;
    
    (void)lpi2c_sem_acquire(10U);
    (void)drv_rtc_set_alarm(alrm_dec, RTC_MINUTE_ALM_IRQ_N);
    (void)lpi2c_sem_release();
}

/**
 * @brief Sets up the RTC.
 * 
 */
void rtc_init(void)
{
    status_t s = STATUS_SUCCESS;
    datetime_t set_rtc_time_dec;
    volatile uint32_t rtc_sentinel = 0U;
    
    nvm_read(FILE_SECT_RTC_TIME_SENTINEL, (uint8_t *)&rtc_sentinel, 4U);
    
    if(rtc_sentinel == 0xFFFFFFFFU)
    {
        /* Confinguring the RTC datetime */
        set_rtc_time_dec.second  = 0U;
        set_rtc_time_dec.minute  = 28U;
        set_rtc_time_dec.hour    = 10U;
        set_rtc_time_dec.day     = 6U;
        set_rtc_time_dec.weekday = RTC_DAY_SAT;
        set_rtc_time_dec.month   = RTC_MONTH_MAR;
        set_rtc_time_dec.year    = 21U;
        
        s = rtc_set_date_time(set_rtc_time_dec);
        if(s == STATUS_SUCCESS)
        {
            rtc_sentinel = 0xDEADCACAU;
            (void)nvm_write(FILE_SECT_RTC_TIME_SENTINEL, (uint8_t *)&rtc_sentinel, 4U);   
        }
        else
        {
            set_status_bit(STAT_VCU_RTC_INIT_FAILURE);
        }
    }    
}

