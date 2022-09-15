#ifndef PCA8565_H
#define PCA8565_H

#include <stdint.h>
#include "board.h"
#include "lpi2c_driver.h"

/* RTC Registers Addresses */
#define RTC_CONTROL_1 				0x00U
#define RTC_CONTROL_2  				0x01U
#define RTC_SEC   	       			0x02U
#define RTC_MINUTE         			0x03U
#define RTC_HOUR         			0x04U
#define RTC_DOM         			0x05U
#define RTC_DOW          			0x06U
#define RTC_MONTH	     			0x07U
#define RTC_YEAR	        		0x08U
#define RTC_ALARM_MIN  				0x09U
#define RTC_ALARM_HOUR		 		0x0AU
#define RTC_ALARM_DOM		 		0x0BU
#define RTC_ALARM_DOW  				0x0CU
#define RTC_CLKOUT_CONTROL  		0x0DU
#define RTC_TIMER_CONTROL			0x0EU

/* Register bit defines */
/* Control registers */
#define RTC_CTRL1_TEST_SHIFT        (1U << 3U)
#define RTC_CTRL1_STOP_SHIFT        (1U << 5U)
#define RTC_CTRL1_TEST1_SHIFT       (1U << 7U)

/* Time and Date registers */
#define RTC_SECONDS_MASK            (0x7FU)
#define RTC_MINUTES_MASK            (0x7FU)
#define RTC_HOUR_MASK               (0x3FU)
#define RTC_DAY_MASK                (0x3FU)
#define RTC_WEEKDAY_MASK            (0x07U)
#define RTC_MONTH_MASK              (0x1FU)
#define RTC_YEAR_MASK               (0xFFU)

/* Alarm Registers */
#define RTC_MIN_ALARM_MASK          (0x7FU)
#define RTC_HOUR_ALARM_MASK         (0x3FU)
#define RTC_DAY_ALARM_MASK          (0x3FU)
#define RTC_WEEKDAY_ALARM_MASK      (0x07U)

#define RTC_ALARM_INT_ENABLE		(1U << 1U)
#define RTC_CLEAR_ALARM_FLAG	    (0x07U)
#define RTC_MINUTE_ALM_IRQ_N		(1U)
#define RTC_HOUR_ALM_IRQ_N			(2U)
#define RTC_DAY_ALM_IRQ_N			(4U)
#define RTC_WEEKDAY_ALM_IRQ_N		(8U)

/* CLKOUT control Register */
#define RTC_CLKOUT_ENABLE           (1U << 7U)
#define RTC_CLKOUT_32768_HZ         (0x00U)
#define RTC_CLKOUT_1024_HZ          (0x01U)
#define RTC_CLKOUT_32_HZ            (0x02U)
#define RTC_CLKOUT_1_HZ             (0x03U)

/* Timer register */
#define RTC_TIMER_ENABLE           (1U << 7U)
#define RTC_TIMER_MASK             (0x83U)
#define RTC_TIMER_4096_HZ          (0x00U)
#define RTC_TIMER_64_HZ            (0x01U)
#define RTC_TIMER_1_HZ             (0x02U)
#define RTC_TIMER_0166_HZ          (0x03U)

/* Weekday Mappings */
#define RTC_DAY_SUN                 (0U)
#define RTC_DAY_MON                 (1U)
#define RTC_DAY_TUE                 (2U)
#define RTC_DAY_WED                 (3U)
#define RTC_DAY_THU                 (4U)
#define RTC_DAY_FRI                 (5U)
#define RTC_DAY_SAT                 (6U)

#define RTC_MONTH_JAN               (1U)
#define RTC_MONTH_FEB               (2U)
#define RTC_MONTH_MAR               (3U)
#define RTC_MONTH_APR               (4U)
#define RTC_MONTH_MAY               (5U)
#define RTC_MONTH_JUN               (6U)
#define RTC_MONTH_JUL               (7U)
#define RTC_MONTH_AUG               (8U)
#define RTC_MONTH_SEP               (9U)
#define RTC_MONTH_OCT               (10U)
#define RTC_MONTH_NOV               (11U)
#define RTC_MONTH_DEC               (12U)

/* structure declaration used for storing the date and time */
typedef struct
{
    uint8_t valid;
	uint8_t second;
	uint8_t minute;
	uint8_t hour;
	uint8_t day;
	uint8_t weekday;
	uint8_t month;
	uint8_t year;
}datetime_t;

/* structure declaration used for storing the alarm time */
typedef struct
{
	uint8_t alarm_minute;
	uint8_t alarm_hour;
	uint8_t alarm_day;
	uint8_t alarm_weekday;
}alarmtime_t;


/* Function prototypes for RTC */

/*
 * @brief Perform Initializes the RTC
 *
 * This function 
 * -initializes the RTC instance with the settings
 * 	provided by the user via the rtcUserCfg parameter such as
 * baud rate,Set slave address ,transfer type etc.
 * 
 */
status_t drv_rtc_init(void);

/*
 * @brief Perform Writing the date time to the RTC
 * This function writes date and time to RTC,
 * @param datetime  structure variable that stores the date and time to be written into RTC
 *
 */
status_t drv_rtc_set(datetime_t settime_dec);


/*
 * @brief Perform Reading the date time from the RTC
 * This function Reads the current date and time from RTC,
 * @ returns the structure variable stores the current date and time that read from RTC
 */
status_t drv_rtc_read(datetime_t *curr_time_dec);
 

/*
 * @brief Perform writng the date time to alarm registers of RTC
 * This function sets alarm registers as given alarmitme variable from user,
 * 
 */

status_t drv_rtc_set_alarm(alarmtime_t set_alarm_dec, uint8_t which_alarm);
 
/*
 * @brief Perform reading the date time to alarm registers of RTC
 * This function gets alarm registers read from RTC
 * @ returns the structure variable stores the current date and time that read from RTC
 */

status_t drv_rtc_get_alarm(alarmtime_t *get_alarm_dec);

/* Function to clear the alarm interrupt flag bit in RTC_CONTROL_2*/
status_t drv_rtc_init(void);
status_t drv_rtc_read(datetime_t *curr_time_dec);
void drv_rtc_clear_alarm_flag(void);
status_t drv_rtc_sem_acquire(uint32_t timeout);
status_t drv_rtc_sem_release(void);

#endif /* PCA8565_H */
