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
 
#ifndef IMU_DYNAMICS_H
#define IMU_DYNAMICS_H

#include <stdint.h>
#include <math.h>

#include "imu.h"

#define ACCELEROMETER_SENSITIVITY   8192.0f
#define GYROSCOPE_SENSITIVITY       16.4f
#define PI                          3.14159265359f   

#ifdef USE_IMU_FILTER
void complimentary_filter(imu_data_t imu_data, float_t *pitch, float_t *roll, float_t dt);
void kalman_filter(imu_data_t imu_data, float_t *pitch, float_t *roll, float_t dt);
void kalman_filter_v2(imu_data_t imu_data, float_t *pitch, float_t *roll, float_t dt);
void imu_filter(imu_data_t imu_data, float_t *pitch, float_t *roll, float_t dt);
#endif /* USE_IMU_FILTER */

void imu_quat_frame(imu_data_t imu_data, float_t *pitch, float_t *roll);

#endif /* IMU_DYNAMICS_H */
