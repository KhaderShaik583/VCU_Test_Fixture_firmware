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
 
#ifndef LED_TASK_H
#define LED_TASK_H

#include "board.h"
#include "fw_common.h"
#include "drv_led.h"

#define LED_TASK_NAME           "thread_led"
#define LED_TASK_STACK_SIZE     STACK_SIZE(128U)


/* Thread Flags */
#define LED_TASK_START_FLAG     (0x0001U)

void led_task_create(void);
thread_id_t led_task_get_id(void);


#endif /* LED_TASK_H */
