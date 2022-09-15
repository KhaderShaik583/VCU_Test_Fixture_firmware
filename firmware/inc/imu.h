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
 
#ifndef IMU_H
#define IMU_H

#include "fw_common.h"
#include "pins_driver.h"
#include "lpspi_master_driver.h"

#define IMU_TASK_NAME           "thread_imu"
#define IMU_TASK_STACK_SIZE     STACK_SIZE(256U)

/* Thread Flags */
#define IMU_TASK_START_FLAG     (0x0001U)
#define IMU_TASK_IRQ_FLAG       (0x0004U)

#define IMU_NUM_MEASUREMENTS    (256U)
#define IMU_RING_MSG_THRESHOLD  (48U)

#define IMU_CAL_SENTINEL            (0xFFC00CFEU)

#define IMU_MSG_CAL_START           (0xCFU)
#define IMU_MSG_CAL_RESET           (0xCAU)
#define AK0991x_DEFAULT_I2C_ADDR	(0x0C)

void imu_task_create(void);
thread_id_t imu_task_get_id(void);

typedef struct 
{
    int32_t mode;
    bool enable_gyroscope;
    bool enable_accelerometer;
    bool enable_magnetometer;
    bool enable_quaternion;
    int32_t gyroscope_frequency;
    int32_t accelerometer_frequency;
    int32_t magnetometer_frequency;
    int32_t quaternion_frequency;
    int32_t orientation_frequency;
}imu_settings_t;

typedef struct 
{
    uint32_t millis;
    float_t accel_x;
    float_t accel_y;
    float_t accel_z;
    
    float_t gyro_x;
    float_t gyro_y;
    float_t gyro_z;
    
    float_t mag_x;
    float_t mag_y;
    float_t mag_z;
    
    float_t quat_w;
    float_t quat_x;
    float_t quat_y;
    float_t quat_z;
    
    float_t orientation_x;
    float_t orientation_y;
    float_t orientation_z;
    
}imu_data_t;

typedef struct 
{
    uint32_t millis;
    uint16_t throttle_speed;
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    
    int16_t mag_x;
    int16_t mag_y;
    int16_t mag_z;
    
    int16_t quat_w;
    int16_t quat_x;
    int16_t quat_y;
    int16_t quat_z;

}imu_data_16_t; 

typedef struct 
{
    uint32_t msg_id;
    uint8_t data;
}imu_msg_queue_obj_t;

typedef struct 
{
    uint32_t imu_cal_sentinel;
    float_t ofs_accel_x;
    float_t ofs_accel_y;
    float_t ofs_accel_z;
    
    float_t ofs_gyro_x;
    float_t ofs_gyro_y;
    float_t ofs_gyro_z;
    
    float_t ofs_orientation_x;
    float_t ofs_orientation_y;
    float_t ofs_orientation_z;

}imu_cal_state_t;

extern osif_msg_queue_id_t imu_msg_queue;

int32_t icm20948_sensor_setup(void);
void imu_init(void);
void imu_poll_sensors(void);
bool imu_is_accel_data_ready(void);
void imu_read_gyro_data(float_t *x, float_t *y, float_t *z);
void imu_read_accesl_data(float_t *x, float_t *y, float_t *z);

void imu_task_create(void);
thread_id_t imu_task_get_id(void);
void imu_read_data_irq(void);
void imu_get_msg_params(void *msg);
float_t imu_get_pitch(void);
float_t imu_get_roll(void);
float_t imu_get_yaw(void);
void imu_get_data(imu_data_t *imu_data);

#endif /* IMU_H */
