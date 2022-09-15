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
 
#ifndef LSM_TASK_H
#define LSM_TASK_H

#include "fw_common.h"

#define LSM_TASK_NAME           "thread_lsm"
#define LSM_TASK_STACK_SIZE     STACK_SIZE(128U)

#define LSM_TASK_START_FLAG      (0x0001U)

void lsm_task_create(void);
osThreadId_t lsm_task_get_id(void);
uint32_t lsm_is_throttle_voltage_zero(void);
void lsm_throttle_2_voltage(float_t *tv);
void lsm_throttle_1_voltage(float_t *tv);
void lsm_throttle_ratio_voltage(float_t *tr);
void lsm_throttle_adc_vref_voltage(float_t *vref);

#endif /* LSM_TASK_H */
