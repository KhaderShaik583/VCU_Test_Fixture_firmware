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
 
#include "imu_dynamics.h"

/*
 Accelerometer values in Gs
 Gyro values in rad/sec
 dt approx. 10 ms
*/

#define M_PI    (3.14159f)

static float_t av_pitch = 0.0f;
static float_t av_roll = 0.0f;

/* Process Noise co-variance */
static float_t q[2] = {0.0125f, 0.0125f};

/* Measurement noise co-variance */
static float_t r[2] = {32.0f, 32.0f};

/* Estimation error co-variance */
static float_t p[2] = {1023.0f, 1023.0f};

/* Kalman gain */
static float_t k[2] = {0.0f, 0.0f};
    
/* Iterated filter value */
static float_t x[2] = {0.0f, 0.0f};

/*
    index 0 -> Pitch measurement
          1 -> Roll measurement
*/
static void kalman_filt(float_t meas, float_t *angle, uint32_t index)
{   
    if((index == 0U) || (index == 1U))
    {
        x[index] = *angle;
        
        p[index] = p[index] + q[index];
        k[index] = p[index] / (p[index] + r[index]);
        x[index] = x[index] + (k[index] * (meas - x[index]));
        p[index] = (1.0f - k[index]) * p[index];
        
        *angle = x[index];
    }
    else
    {
        set_status_bit(STAT_VCU_IMU_FAULT);
    }
}

void imu_quat_frame(imu_data_t imu_data, float_t *pitch, float_t *roll)
{
    float_t sinr_cosp = 0.0f;
    float_t cosr_cosp = 0.0f;
    float_t sinp = 0.0f;
    float_t pitch_acc = 0.0f;
    float_t roll_acc = 0.0f;    
    
    /* Quaternions to Euler angle conversion 
       https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    */
    sinr_cosp = 2.0f * ((imu_data.quat_w * imu_data.quat_x) + (imu_data.quat_y * imu_data.quat_z));
    cosr_cosp = 1.0f - 2.0f * ((imu_data.quat_x * imu_data.quat_x) + (imu_data.quat_y * imu_data.quat_y));
    
    sinp = 2.0f * ((imu_data.quat_w * imu_data.quat_y) - (imu_data.quat_z * imu_data.quat_x));
    if(fabs(sinp) > 1.0f)
    {
        roll_acc = copysign(M_PI/2, sinp) * 57.29577;
    }
    else
    {
        roll_acc = asinf(sinp) * 57.29577f;
    }
    
    /* Pitch = atan2f(sinr_cosp, cosr_cosp) * (180/PI) */
    pitch_acc = atan2f(sinr_cosp, cosr_cosp) * 57.29577f; 
    
    
#if 0
    kalman_filt(pitchAcc, pitch, 0U);
    kalman_filt(rollAcc, roll, 1U);
#endif
    
    *pitch = pitch_acc;
    *roll = roll_acc;
}

