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
 
#ifndef RTC_TASK_H
#define RTC_TASK_H

#include "fw_common.h"
#include "drv_PCA8565.h" 

#define RTC_TASK_STACK_SIZE     (512U)

void rtc_read_local_time(datetime_t *dt);
void rtc_trig_measurement(void);
void rtc_setup_alarm(void);
void rtc_init(void);
void rtc_set_date_time_nw(datetime_t *date_time);

#endif /* RTC_TASK_H */
