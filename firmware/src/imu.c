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
 
#include "imu.h"
#include "lpspi_hw_access.h"
#include "drv_spi_legacy.h" 
#include "Icm20948.h"
#include "SensorTypes.h"
#include "Icm20948MPUFifoControl.h"
#include "Icm20948DataBaseDriver.h"
#include "Icm20948Setup.h"

#include "imu_dynamics.h" 
#include "rtx_os.h"
#include "udp_task.h" 
#include "dba_task.h"
#include "drv_loadswitches.h"
#include "nvm.h"
#include "wdt_task.h"

#define IMU_MSGQUEUE_OBJECTS 4 

#ifdef USE_FEATURE_IMU_CIRCULAR_BUFFER
#include "ringbuff.h" 
#endif /* USE_FEATURE_IMU_CIRCULAR_BUFFER */

#ifdef USE_FEATURE_RTX_VIEWER
__attribute__((section(".bss.os.thread.cb")))
thread_tcb_t imu_thread_tcb;
#else
static thread_tcb_t imu_thread_tcb;
#endif /* USE_FEATURE_RTX_VIEWER */

__attribute__((section("ARM_LIB_STACK")))
static uint64_t imu_thread_stk[IMU_TASK_STACK_SIZE];

static thread_id_t thread_imu;

static const thread_attrs_t imu_task_attr = {
    IMU_TASK_NAME,
    osThreadDetached,
    &imu_thread_tcb,
    sizeof(imu_thread_tcb),
    &imu_thread_stk[0],
    IMU_TASK_STACK_SIZE * sizeof(uint64_t),
    osPriorityNormal1,
    0U,
    0U 
};

static const imu_settings_t settings = {
    .mode                       = 1,        /* 0 = low power mode, 1 = high performance mode */
    .enable_gyroscope           = true,    /* Enables gyroscope output */
    .enable_accelerometer       = true,    /* Enables accelerometer output */
    .enable_magnetometer        = true,    /* Enables magnetometer output */
    .enable_quaternion          = true,     /* Enables quaternion output */
    .gyroscope_frequency        = 100,      /* Max frequency = 225, min frequency = 1 */
    .accelerometer_frequency    = 100,      /* Max frequency = 225, min frequency = 1 */
    .magnetometer_frequency     = 100,      /* Max frequency = 70, min frequency = 1 */
    .quaternion_frequency       = 100,      /* Max frequency = 225, min frequency = 50 */
    .orientation_frequency      = 100
};

static imu_data_t imu_measurements;
static imu_data_t imu_measurements_offs;

#ifdef USE_FEATURE_PACK_IMU_MEASUREMENT_DATA
static imu_data_16_t imu_measurements_compressed;
#endif

static imu_cal_state_t imu_cal_info;

static float_t pitch = 0.0f;
static float_t roll = 0.0f;
static float_t yaw = 0.0f;

#ifdef USE_NATIVE_IMU_COMPUTATION
static float_t pitch_native = 0.0f;
static float_t roll_native = 0.0f;
static float_t yaw_native = 0.0f;
#endif

static bool gyro_data_ready = false;
static bool accel_data_ready = false;
static bool mag_data_ready = false;
static bool quat_data_ready = false;
static bool orientation_data_ready = false;

static uint32_t imu_fault = 0U;
static uint32_t imu_ofs_calibration = 0U;
static uint32_t imu_ofs_calibration_apply = 0U;

static inv_icm20948_t icm_device;
static int32_t unscaled_bias[THREE_AXES * 2];

static uint32_t time_step = 0U;
static uint32_t prev_time_step = 0U;

/* Default = +/- 4g. Valid ranges: 2, 4, 8, 16 */
static int32_t cfg_acc_fsr = 2;

/* Default = +/- 2000dps. Valid ranges: 250, 500, 1000, 2000 */
static int32_t cfg_gyr_fsr = 250;

/* Default = +/- 4g. Valid ranges: 2, 4, 8, 16 */
static int32_t cal_cfg_acc_fsr = 2;

/* Default = +/- 2000dps. Valid ranges: 250, 500, 1000, 2000 */
static int32_t cal_cfg_gyr_fsr = 250;

static const uint8_t dmp3_image[] = {
    #include "icm20948_img.dmp3a.h"
};

#ifdef USE_FEATURE_IMU_CIRCULAR_BUFFER
static ringbuff_t imu_ring0;
static uint8_t imu_buffer0[1U + (IMU_NUM_MEASUREMENTS * sizeof(imu_data_16_t))];
#endif

osif_msg_queue_id_t imu_msg_queue; 

static int32_t idd_io_hal_read_reg(void *context, uint8_t reg, uint8_t *rbuffer, uint32_t rlen)
{
    volatile uint8_t icm_reg = 0x80U | reg;
    
    UNUSED_PARAM(context);

    (void)lpspi_read8_icm20948(icm_reg, rbuffer, (uint8_t)rlen);
    
    return 0;
}

static int32_t idd_io_hal_write_reg(void *context, uint8_t reg, const uint8_t *wbuffer, uint32_t wlen)
{
    volatile uint8_t icm_reg = 0x7FU & reg;
    int32_t lc = 0;
    
    UNUSED_PARAM(context);
    
    lc = osif_enter_critical();
    (void)lpspi_write8_icm20948(icm_reg, (uint8_t *)wbuffer, (uint8_t)wlen);
    (void)osif_exit_critical(lc);

    return 0;

}

/*************************************************************************
  Invensense Variables
*************************************************************************/

static uint8_t convert_to_generic_ids[INV_ICM20948_SENSOR_MAX] = {
	INV_SENSOR_TYPE_ACCELEROMETER,
	INV_SENSOR_TYPE_GYROSCOPE,
	INV_SENSOR_TYPE_RAW_ACCELEROMETER,
	INV_SENSOR_TYPE_RAW_GYROSCOPE,
	INV_SENSOR_TYPE_UNCAL_MAGNETOMETER,
	INV_SENSOR_TYPE_UNCAL_GYROSCOPE,
	INV_SENSOR_TYPE_BAC,
	INV_SENSOR_TYPE_STEP_DETECTOR,
	INV_SENSOR_TYPE_STEP_COUNTER,
	INV_SENSOR_TYPE_GAME_ROTATION_VECTOR,
	INV_SENSOR_TYPE_ROTATION_VECTOR,
	INV_SENSOR_TYPE_GEOMAG_ROTATION_VECTOR,
	INV_SENSOR_TYPE_MAGNETOMETER,
	INV_SENSOR_TYPE_SMD,
	INV_SENSOR_TYPE_PICK_UP_GESTURE,
	INV_SENSOR_TYPE_TILT_DETECTOR,
	INV_SENSOR_TYPE_GRAVITY,
	INV_SENSOR_TYPE_LINEAR_ACCELERATION,
	INV_SENSOR_TYPE_ORIENTATION,
	INV_SENSOR_TYPE_B2S
};

static inv_bool_t interface_is_SPI(void)
{
  return true;
}

#ifdef USE_NATIVE_IMU_COMPUTATION
static int32_t imu_get_rotation_matrix(float_t rotation_matrix[], float_t inclination_matrix[], imu_data_t imu_data)
{
    const float_t g = 9.81f;
    
    int32_t ret = -1;
    float_t free_fall_gravity_squared = 0.0f;
    
    float_t Ax = 0.0f;
    float_t Ay = 0.0f;
    float_t Az = 0.0f;
    
    float_t Ex = 0.0f;
    float_t Ey = 0.0f;
    float_t Ez = 0.0f;
    
    float_t Hx = 0.0f;
    float_t Hy = 0.0f;
    float_t Hz = 0.0f;
    
    float_t Mx = 0.0f;
    float_t My = 0.0f;
    float_t Mz = 0.0f;
    
    float_t normsqA = 0.0f;
    float_t normH = 0.0f;
    
    float_t invH = 0.0f;
    float_t invA = 0.0f;
    
    float_t invE = 0.0f;
    float_t c = 0.0f;
    float_t s = 0.0f;
    
    Ax = imu_data.accel_x;
    Ay = imu_data.accel_y;
    Az = imu_data.accel_z;
    
    normsqA = ((Ax * Ax) + (Ay * Ay) + (Az * Az));
    free_fall_gravity_squared = 0.0f;
    free_fall_gravity_squared = 0.01f * g * g;
    
    if(normsqA < free_fall_gravity_squared) 
    {
        /* gravity less than 10% of normal value */
        ret = -1;
    }
    else
    {
        Ex = imu_data.mag_x;
        Ey = imu_data.mag_y;
        Ez = imu_data.mag_z;
        
        Hx = (Ey * Az) - (Ez * Ay);
        Hy = (Ez * Ax) - (Ex * Az);
        Hz = (Ex * Ay) - (Ey * Ax);
        
        normH = (float_t)sqrtf((Hx * Hx) + (Hy * Hy) + (Hz * Hz));
        if(normH < 0.1f) 
        {
            /* device is close to free fall (or in space?), or close to 
               magnetic north pole. Typical values are  > 100. */
            ret = -1;
        }
        else
        {
            invH = 1.0f / normH;
            Hx *= invH;
            Hy *= invH;
            Hz *= invH;
            
            invA = 1.0f / (float_t)sqrtf((Ax * Ax) + (Ay * Ay) + (Az * Az));
            Ax *= invA;
            Ay *= invA;
            Az *= invA;
            
            Mx = (Ay * Hz) - (Az * Hy);
            My = (Az * Hx) - (Ax * Hz);
            Mz = (Ax * Hy) - (Ay * Hx);

            if(rotation_matrix != NULL) 
            {
                /* Accesses are in row-major form */
                rotation_matrix[0] = Hx;    rotation_matrix[1] = Hy;    rotation_matrix[2] = Hz;
                rotation_matrix[3] = Mx;    rotation_matrix[4] = My;    rotation_matrix[5] = Mz;
                rotation_matrix[6] = Ax;    rotation_matrix[7] = Ay;    rotation_matrix[8] = Az;
            }
        }
        
        if(inclination_matrix != NULL) 
        {
            /* compute the inclination matrix by projecting the geomagnetic
               vector onto the Z (gravity) and X (horizontal component
               of geomagnetic vector) axes.
            */
            
            invE = 1.0f / (float_t)sqrtf((Ex * Ex) + (Ey * Ey) + (Ez * Ez));
            c = ((Ex * Mx) + (Ey * My) + (Ez * Mz)) * invE;
            s = ((Ex * Ax) + (Ey * Ay) + (Ez * Az)) * invE;
            
            /* Accesses are in row-major form */
            inclination_matrix[0] = 1;  inclination_matrix[1] = 0;  inclination_matrix[2] = 0;
            inclination_matrix[3] = 0;  inclination_matrix[4] = c;  inclination_matrix[5] = s;
            inclination_matrix[6] = 0;  inclination_matrix[7] = -s; inclination_matrix[8] = c;
        }
    }
    
    return ret;
}

static void imu_get_orientation_native(float_t rot_mat[], imu_data_t *imu_data)
{
    imu_data->orientation_x = (float_t)asinf(-rot_mat[7]);
    imu_data->orientation_y = (float_t)atan2f(-rot_mat[6], rot_mat[8]);
    imu_data->orientation_z = (float_t)atan2f(rot_mat[1], rot_mat[4]);
}
#endif /* USE_NATIVE_IMU_COMPUTATION */

static void icm20948_apply_mounting_matrix(void)
{
    int32_t i = 0;

    float_t cfg_mounting_matrix[9];
	
    cfg_mounting_matrix[0] = 1.0f;  cfg_mounting_matrix[1] = 0.0f;  cfg_mounting_matrix[2] = 0.0f;
    cfg_mounting_matrix[3] = 0.0f;  cfg_mounting_matrix[4] = 1.0f;  cfg_mounting_matrix[5] = 0.0f;
    cfg_mounting_matrix[6] = 0.0f;  cfg_mounting_matrix[7] = 0.0f;  cfg_mounting_matrix[8] = 1.0f;

    for(i = 0; i < INV_ICM20948_SENSOR_MAX; i++)
    {
        (void)inv_icm20948_set_matrix(&icm_device, cfg_mounting_matrix, i);
    }
    
#if 0		
    inv_icm20948_set_matrix(&icm_device, cfg_compass_mounting_matrix, INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED);
#endif
    
}

static void icm20948_set_fsr(void)
{
	(void)inv_icm20948_set_fsr(&icm_device, INV_ICM20948_SENSOR_RAW_ACCELEROMETER, (const void *)&cfg_acc_fsr);
	(void)inv_icm20948_set_fsr(&icm_device, INV_ICM20948_SENSOR_ACCELEROMETER, (const void *)&cfg_acc_fsr);
	(void)inv_icm20948_set_fsr(&icm_device, INV_ICM20948_SENSOR_RAW_GYROSCOPE, (const void *)&cfg_gyr_fsr);
	(void)inv_icm20948_set_fsr(&icm_device, INV_ICM20948_SENSOR_GYROSCOPE, (const void *)&cfg_gyr_fsr);
	(void)inv_icm20948_set_fsr(&icm_device, INV_ICM20948_SENSOR_GYROSCOPE_UNCALIBRATED, (const void *)&cfg_gyr_fsr);
}

static void icm20948_set_fsr_calib(void)
{
	(void)inv_icm20948_set_fsr(&icm_device, INV_ICM20948_SENSOR_RAW_ACCELEROMETER, (const void *)&cal_cfg_acc_fsr);
	(void)inv_icm20948_set_fsr(&icm_device, INV_ICM20948_SENSOR_ACCELEROMETER, (const void *)&cal_cfg_acc_fsr);
	(void)inv_icm20948_set_fsr(&icm_device, INV_ICM20948_SENSOR_RAW_GYROSCOPE, (const void *)&cal_cfg_gyr_fsr);
	(void)inv_icm20948_set_fsr(&icm_device, INV_ICM20948_SENSOR_GYROSCOPE, (const void *)&cal_cfg_gyr_fsr);
	(void)inv_icm20948_set_fsr(&icm_device, INV_ICM20948_SENSOR_GYROSCOPE_UNCALIBRATED, (const void *)&cal_cfg_gyr_fsr);
}

static uint8_t icm20948_get_grv_accuracy(void)
{
	uint8_t accel_accuracy;
	uint8_t gyro_accuracy;

	accel_accuracy = (uint8_t)inv_icm20948_get_accel_accuracy();
	gyro_accuracy = (uint8_t)inv_icm20948_get_gyro_accuracy();
	return (min(accel_accuracy, gyro_accuracy));
}

static void build_sensor_event_data(void *context, enum inv_icm20948_sensor sensortype, uint64_t timestamp, const void *data, const void *arg)
{
	float_t raw_bias_data[6];
	inv_sensor_event_t event;
	(void)context;
	uint8_t sensor_id = convert_to_generic_ids[sensortype];

	memset((void *)&event, 0, sizeof(event));
	event.sensor = sensor_id;
	event.timestamp = timestamp;
    
	switch(sensor_id) 
    {
        case INV_SENSOR_TYPE_UNCAL_GYROSCOPE:
            (void)memcpy(raw_bias_data, data, sizeof(raw_bias_data));
            (void)memcpy(event.data.gyr.vect, &raw_bias_data[0], sizeof(event.data.gyr.vect));
            (void)memcpy(event.data.gyr.bias, &raw_bias_data[3], sizeof(event.data.gyr.bias));
            (void)memcpy(&(event.data.gyr.accuracy_flag), arg, sizeof(event.data.gyr.accuracy_flag));
            break;
        
        case INV_SENSOR_TYPE_UNCAL_MAGNETOMETER:
            (void)memcpy(raw_bias_data, data, sizeof(raw_bias_data));
            (void)memcpy(event.data.mag.vect, &raw_bias_data[0], sizeof(event.data.mag.vect));
            (void)memcpy(event.data.mag.bias, &raw_bias_data[3], sizeof(event.data.mag.bias));
            (void)memcpy(&(event.data.gyr.accuracy_flag), arg, sizeof(event.data.gyr.accuracy_flag));
            break;
        
        case INV_SENSOR_TYPE_GYROSCOPE:
            (void)memcpy(event.data.gyr.vect, data, sizeof(event.data.gyr.vect));
            (void)memcpy(&(event.data.gyr.accuracy_flag), arg, sizeof(event.data.gyr.accuracy_flag));
            
            /* Required data */
            imu_measurements.gyro_x = event.data.gyr.vect[0];
            imu_measurements.gyro_y = event.data.gyr.vect[1];
            imu_measurements.gyro_z = event.data.gyr.vect[2];
            gyro_data_ready = true;
        
            break;
        
        case INV_SENSOR_TYPE_GRAVITY:
            (void)memcpy(event.data.acc.vect, data, sizeof(event.data.acc.vect));
            event.data.acc.accuracy_flag = inv_icm20948_get_accel_accuracy();
            break;
        
        case INV_SENSOR_TYPE_LINEAR_ACCELERATION:
        case INV_SENSOR_TYPE_ACCELEROMETER:
            (void)memcpy(event.data.acc.vect, data, sizeof(event.data.acc.vect));
            (void)memcpy(&(event.data.acc.accuracy_flag), arg, sizeof(event.data.acc.accuracy_flag));
        
            /* Required data */
            imu_measurements.accel_x = event.data.acc.vect[0];
            imu_measurements.accel_y = event.data.acc.vect[1];
            imu_measurements.accel_z = event.data.acc.vect[2];
            accel_data_ready = true;
            break;

        case INV_SENSOR_TYPE_MAGNETOMETER:
            (void)memcpy(event.data.mag.vect, data, sizeof(event.data.mag.vect));
            (void)memcpy(&(event.data.mag.accuracy_flag), arg, sizeof(event.data.mag.accuracy_flag));
        
            /* Required data */
            imu_measurements.mag_x = event.data.mag.vect[0];
            imu_measurements.mag_y = event.data.mag.vect[1];
            imu_measurements.mag_z = event.data.mag.vect[2];
            mag_data_ready = true;
            break;

        case INV_SENSOR_TYPE_GEOMAG_ROTATION_VECTOR:
        case INV_SENSOR_TYPE_ROTATION_VECTOR:
            (void)memcpy(&(event.data.quaternion.accuracy), arg, sizeof(event.data.quaternion.accuracy));
            (void)memcpy(event.data.quaternion.quat, data, sizeof(event.data.quaternion.quat));

                    /* Required data */
            imu_measurements.quat_w = event.data.quaternion.quat[0];
            imu_measurements.quat_x = event.data.quaternion.quat[1];
            imu_measurements.quat_y = event.data.quaternion.quat[2];
            imu_measurements.quat_z = event.data.quaternion.quat[3];
            quat_data_ready = true;
            break;
        
        case INV_SENSOR_TYPE_GAME_ROTATION_VECTOR:
            (void)memcpy(event.data.quaternion.quat, data, sizeof(event.data.quaternion.quat));
            event.data.quaternion.accuracy_flag = icm20948_get_grv_accuracy();
        
                    /* Required data */
            imu_measurements.quat_w = event.data.quaternion.quat[0];
            imu_measurements.quat_x = event.data.quaternion.quat[1];
            imu_measurements.quat_y = event.data.quaternion.quat[2];
            imu_measurements.quat_z = event.data.quaternion.quat[3];
            quat_data_ready = true;
            break;

        case INV_SENSOR_TYPE_BAC:
            (void)memcpy(&(event.data.bac.event), data, sizeof(event.data.bac.event));
            break;
        
        case INV_SENSOR_TYPE_PICK_UP_GESTURE:
        case INV_SENSOR_TYPE_TILT_DETECTOR:
        case INV_SENSOR_TYPE_STEP_DETECTOR:
        case INV_SENSOR_TYPE_SMD:
            event.data.event = true;
            break;
        
        case INV_SENSOR_TYPE_B2S:
            event.data.event = true;
            (void)memcpy(&(event.data.b2s.direction), data, sizeof(event.data.b2s.direction));
            break;
        
        case INV_SENSOR_TYPE_STEP_COUNTER:
            (void)memcpy(&(event.data.step.count), data, sizeof(event.data.step.count));
            break;
        
        case INV_SENSOR_TYPE_ORIENTATION:
            /* copy x,y,z from orientation data */
            (void)memcpy(&(event.data.orientation), data, 3*sizeof(float_t));
        
            /* Android convention Yaw, Pitch, Roll */
            imu_measurements.orientation_z = event.data.orientation.x;
            imu_measurements.orientation_x = event.data.orientation.y;
            imu_measurements.orientation_y = event.data.orientation.z;
        
            orientation_data_ready = true;
            break;
        
        case INV_SENSOR_TYPE_RAW_ACCELEROMETER:
        case INV_SENSOR_TYPE_RAW_GYROSCOPE:
            (void)memcpy(event.data.raw3d.vect, data, sizeof(event.data.raw3d.vect));
            break;
        
        default:
            __NOP();
            break;
	}
}

static enum inv_icm20948_sensor idd_sensortype_conversion(int32_t sensor)
{
    enum inv_icm20948_sensor sns;
    
    switch (sensor)
    {
        case INV_SENSOR_TYPE_RAW_ACCELEROMETER:
            sns = INV_ICM20948_SENSOR_RAW_ACCELEROMETER;
            break;
        
        case INV_SENSOR_TYPE_RAW_GYROSCOPE:
            sns = INV_ICM20948_SENSOR_RAW_GYROSCOPE;
            break;
        
        case INV_SENSOR_TYPE_ACCELEROMETER:
            sns = INV_ICM20948_SENSOR_ACCELEROMETER;
            break;
        
        case INV_SENSOR_TYPE_GYROSCOPE:
            sns = INV_ICM20948_SENSOR_GYROSCOPE;
            break;
        
        case INV_SENSOR_TYPE_UNCAL_MAGNETOMETER:
            sns = INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED;
            break;
        
        case INV_SENSOR_TYPE_UNCAL_GYROSCOPE:
            sns = INV_ICM20948_SENSOR_GYROSCOPE_UNCALIBRATED;
            break;
        
        case INV_SENSOR_TYPE_BAC:
            sns = INV_ICM20948_SENSOR_ACTIVITY_CLASSIFICATON;
            break;
        
        case INV_SENSOR_TYPE_STEP_DETECTOR:
            sns = INV_ICM20948_SENSOR_STEP_DETECTOR;
            break;
        
        case INV_SENSOR_TYPE_STEP_COUNTER:
            sns = INV_ICM20948_SENSOR_STEP_COUNTER;
            break;
        
        case INV_SENSOR_TYPE_GAME_ROTATION_VECTOR:
            sns = INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR;
            break;
        
        case INV_SENSOR_TYPE_ROTATION_VECTOR:
            sns = INV_ICM20948_SENSOR_ROTATION_VECTOR;
            break;
        
        case INV_SENSOR_TYPE_GEOMAG_ROTATION_VECTOR:
            sns = INV_ICM20948_SENSOR_GEOMAGNETIC_ROTATION_VECTOR;
            break;
        
        case INV_SENSOR_TYPE_MAGNETOMETER:
            sns = INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD;
            break;
        
        case INV_SENSOR_TYPE_SMD:
            sns = INV_ICM20948_SENSOR_WAKEUP_SIGNIFICANT_MOTION;
            break;
        
        case INV_SENSOR_TYPE_PICK_UP_GESTURE:
            sns = INV_ICM20948_SENSOR_FLIP_PICKUP;
            break;
        
        case INV_SENSOR_TYPE_TILT_DETECTOR:
            sns = INV_ICM20948_SENSOR_WAKEUP_TILT_DETECTOR;
            break;
        
        case INV_SENSOR_TYPE_GRAVITY:
            sns = INV_ICM20948_SENSOR_GRAVITY;
            break;
        
        case INV_SENSOR_TYPE_LINEAR_ACCELERATION:
            sns = INV_ICM20948_SENSOR_LINEAR_ACCELERATION;
            break;
        
        case INV_SENSOR_TYPE_ORIENTATION:
            sns = INV_ICM20948_SENSOR_ORIENTATION;
            break;
        
        case INV_SENSOR_TYPE_B2S:
            sns = INV_ICM20948_SENSOR_B2S;
            break;
        
        default:
            sns = INV_ICM20948_SENSOR_MAX;
            break;
    }
    
    return sns;
}

static void imu_power_cycle(void)
{
    PINS_DRV_ClearPins(IMU_VDD_EN_GPIO, 1U << IMU_VDD_EN_PIN); 
    osif_time_delay(100);
    PINS_DRV_SetPins(IMU_VDD_EN_GPIO, 1U << IMU_VDD_EN_PIN); 
    osif_time_delay(100);    
}

#ifdef USE_FEATURE_PACK_IMU_MEASUREMENT_DATA
static void imu_compress_imu_measurements(void)
{
    uint32_t speed = 0U;
    float_t throttle_voltage = 0.0f;
    
    imu_measurements_compressed.millis = imu_measurements.millis;
    
    speed = mc_get_vehicle_speed();
    
    imu_measurements_compressed.throttle_speed = 0U;
    imu_measurements_compressed.throttle_speed |= (uint8_t)(speed & 0x000000FFU);
    imu_measurements_compressed.throttle_speed |= (uint16_t)((uint8_t)(throttle_voltage * 10.0f)) << 8U;
    
    imu_measurements_compressed.accel_x = (int16_t)(imu_measurements.accel_x * 1000.0f);
    imu_measurements_compressed.accel_y = (int16_t)(imu_measurements.accel_y * 1000.0f);
    imu_measurements_compressed.accel_z = (int16_t)(imu_measurements.accel_z * 1000.0f);
    
    imu_measurements_compressed.gyro_x = (int16_t)(imu_measurements.gyro_x * 1000.0f);
    imu_measurements_compressed.gyro_y = (int16_t)(imu_measurements.gyro_y * 1000.0f);
    imu_measurements_compressed.gyro_z = (int16_t)(imu_measurements.gyro_z * 1000.0f);
    
    imu_measurements_compressed.mag_x = (int16_t)(imu_measurements.mag_x * 1000.0f);
    imu_measurements_compressed.mag_y = (int16_t)(imu_measurements.mag_y * 1000.0f);
    imu_measurements_compressed.mag_z = (int16_t)(imu_measurements.mag_z * 1000.0f);
    
    imu_measurements_compressed.quat_w = (int16_t)(imu_measurements.quat_w * 1000.0f);
    imu_measurements_compressed.quat_x = (int16_t)(imu_measurements.quat_x * 1000.0f);
    imu_measurements_compressed.quat_y = (int16_t)(imu_measurements.quat_y * 1000.0f);
    imu_measurements_compressed.quat_z = (int16_t)(imu_measurements.quat_z * 1000.0f);    
}
#endif /* USE_FEATURE_PACK_IMU_MEASUREMENT_DATA */

static void imu_offset_calibration(void)
{
    static const int32_t N = 128;
    static int32_t sample_count = N;
    static uint32_t cal_state = 0U;
    
    switch(cal_state)
    {
        case 0U:
            /* Read NVM and check if calibration is done */
            nvm_read(FILE_SECT_IMU_CAL_INFO, (uint8_t *)&imu_cal_info, sizeof(imu_cal_state_t));
            if(imu_cal_info.imu_cal_sentinel != IMU_CAL_SENTINEL)
            {
                icm20948_set_fsr_calib();
                cal_state = 1U;
                set_status_bit(STAT_VCU_IMU_OFS_CALIBRATION);
            }
            else
            {
                cal_state = 0U;
                imu_ofs_calibration = 0U;
            }
            break;
        
        case 1U:
            if(sample_count >= 0)
            {
                imu_measurements_offs.accel_x += imu_measurements.accel_x;
                imu_measurements_offs.accel_y += imu_measurements.accel_y;
                imu_measurements_offs.accel_z += imu_measurements.accel_z;
                
                imu_measurements_offs.gyro_x += imu_measurements.gyro_x;
                imu_measurements_offs.gyro_y += imu_measurements.gyro_y;
                imu_measurements_offs.gyro_z += imu_measurements.gyro_z;
                
                sample_count--;
            }
            else
            {
                imu_measurements_offs.accel_x = imu_measurements_offs.accel_x / N;
                imu_measurements_offs.accel_y = imu_measurements_offs.accel_y / N;
                imu_measurements_offs.accel_z = imu_measurements_offs.accel_z / N;
                
                imu_measurements_offs.gyro_x = imu_measurements_offs.gyro_x / N;
                imu_measurements_offs.gyro_y = imu_measurements_offs.gyro_y / N;
                imu_measurements_offs.gyro_z = imu_measurements_offs.gyro_z / N;

                imu_ofs_calibration = 0U;
                imu_ofs_calibration_apply = 1U;
                
                clear_status_bit(STAT_VCU_IMU_OFS_CALIBRATION);
                
                imu_cal_info.ofs_accel_x = imu_measurements_offs.accel_x;
                imu_cal_info.ofs_accel_y = imu_measurements_offs.accel_y;
                imu_cal_info.ofs_accel_z = imu_measurements_offs.accel_z;
                
                imu_cal_info.ofs_gyro_x = imu_measurements_offs.gyro_x;
                imu_cal_info.ofs_gyro_y = imu_measurements_offs.gyro_y;
                imu_cal_info.ofs_gyro_z = imu_measurements_offs.gyro_z;
                
                imu_cal_info.imu_cal_sentinel = IMU_CAL_SENTINEL;
                
                (void)nvm_write(FILE_SECT_IMU_CAL_INFO, (uint8_t *)&imu_cal_info, sizeof(imu_cal_state_t));
                
                icm20948_set_fsr();
            }
            break;
            
        default:
            __NOP();
            break;
    }
}

static void imu_orientation_offset_calibration(void)
{
    static const int32_t N = 128;
    static int32_t sample_count = N;
    static uint32_t cal_state = 0U;
    
    switch(cal_state)
    {
        case 0U:
            /* Read NVM and check if calibration is done */
            nvm_read(FILE_SECT_IMU_CAL_INFO, (uint8_t *)&imu_cal_info, sizeof(imu_cal_state_t));
            if(imu_cal_info.imu_cal_sentinel != IMU_CAL_SENTINEL)
            {
                icm20948_set_fsr_calib();
                cal_state = 1U;
                set_status_bit(STAT_VCU_IMU_OFS_CALIBRATION);
            }
            else
            {
                cal_state = 0U;
                imu_ofs_calibration = 0U;
            }
            break;
        
        case 1U:
            if(sample_count >= 0)
            {
                imu_measurements_offs.orientation_x += imu_measurements.orientation_x;
                imu_measurements_offs.orientation_y += imu_measurements.orientation_y;
                imu_measurements_offs.orientation_z += imu_measurements.orientation_z;
                
                sample_count--;
            }
            else
            {
                imu_measurements_offs.orientation_x = imu_measurements_offs.orientation_x / N;
                imu_measurements_offs.orientation_y = imu_measurements_offs.orientation_y / N;
                imu_measurements_offs.orientation_z = imu_measurements_offs.orientation_z / N;

                imu_ofs_calibration = 0U;
                imu_ofs_calibration_apply = 1U;
                
                clear_status_bit(STAT_VCU_IMU_OFS_CALIBRATION);
                
                imu_cal_info.ofs_orientation_x = imu_measurements_offs.orientation_x;
                imu_cal_info.ofs_orientation_y = imu_measurements_offs.orientation_y;
                imu_cal_info.ofs_orientation_z = imu_measurements_offs.orientation_z;
                
                imu_cal_info.imu_cal_sentinel = IMU_CAL_SENTINEL;
                
                (void)nvm_write(FILE_SECT_IMU_CAL_INFO, (uint8_t *)&imu_cal_info, sizeof(imu_cal_state_t));
                
                icm20948_set_fsr();
            }
            break;
            
        default:
            __NOP();
            break;
    }    
}

static void imu_adj_ofs(void)
{
    if(imu_ofs_calibration_apply == 1U)
    {
        imu_measurements.accel_x -= imu_cal_info.ofs_accel_x;
        imu_measurements.accel_y -= imu_cal_info.ofs_accel_y;
        imu_measurements.accel_z += 1.0f - imu_cal_info.ofs_accel_z;
        
        imu_measurements.gyro_x -= imu_cal_info.ofs_gyro_x;
        imu_measurements.gyro_y -= imu_cal_info.ofs_gyro_y;
        imu_measurements.gyro_z -= imu_cal_info.ofs_gyro_z;
    }
}

static void imu_adj_orientation_ofs(void)
{
    if(imu_ofs_calibration_apply == 1U)
    {
        if(imu_measurements.orientation_x < 0.0f)
        {
            imu_measurements.orientation_x -= imu_cal_info.ofs_orientation_x;
        }
        else
        {
            imu_measurements.orientation_x += imu_cal_info.ofs_orientation_x;
        }
        
        if(imu_measurements.orientation_y < 0.0f)
        {
            imu_measurements.orientation_y += imu_cal_info.ofs_orientation_y;
        }
        else
        {
            imu_measurements.orientation_y -= imu_cal_info.ofs_orientation_y;
        }
    }
}

static void imu_calibration_reset(void)
{
    nvm_read(FILE_SECT_IMU_CAL_INFO, (uint8_t *)&imu_cal_info, sizeof(imu_cal_state_t));
    imu_cal_info.imu_cal_sentinel = 0xFFFFFFFFU;
    (void)nvm_write(FILE_SECT_IMU_CAL_INFO, (uint8_t *)&imu_cal_info, sizeof(imu_cal_state_t));
    
    __DMB();
    __DSB();
}

uint64_t inv_icm20948_get_time_us(void)
{
    return 1000U * osif_millis();
}

void inv_icm20948_sleep_us(uint32_t us)
{
    sw_asm_delay_us(us);
}

int32_t icm20948_sensor_setup(void)
{
    int32_t rc = 0;
    uint8_t whoami = 0xff;
    uint8_t d[1] = {0x10U};

    /* Disable I2C */    
    rc += inv_icm20948_write_mems_reg(&icm_device, REG_USER_CTRL, 1, (unsigned char *)&d[0]);
    
    rc += inv_icm20948_get_whoami(&icm_device, &whoami);

    if(whoami != 0xEAU)
    {
        set_status_bit(STAT_VCU_IMU_FAULT);
        imu_fault = STAT_VCU_IMU_FAULT;
    }
        
    rc += inv_icm20948_read_mems_reg(&icm_device, REG_USER_CTRL, 1, (unsigned char *)&d[0]);
        
    /* Setup accel and gyro mounting matrix and associated angle for current board */
    inv_icm20948_init_matrix(&icm_device);

    /* set default power mode */
    rc += inv_icm20948_initialize(&icm_device, dmp3_image, sizeof(dmp3_image));  
    if(rc != 0)
    {
        set_status_bit(STAT_VCU_IMU_FAULT);
        imu_fault = STAT_VCU_IMU_FAULT;    
    }
    
    if(icm_device.base_state.firmware_loaded != 1U)
    {
        set_status_bit(STAT_VCU_IMU_DMP_FAULT);
    }
    
	return 0;
}

void imu_init(void)
{
    int32_t rc = 0;
    struct inv_icm20948_serif icm20948_serif;
    
    icm20948_serif.context   = (void *)0; /* no need */
    icm20948_serif.read_reg  = idd_io_hal_read_reg;
    icm20948_serif.write_reg = idd_io_hal_write_reg;
    icm20948_serif.max_read  = 1024 * 16; /* maximum number of bytes allowed per serial read */
    icm20948_serif.max_write = 1024 * 16; /* maximum number of bytes allowed per serial write */
    icm20948_serif.is_spi = interface_is_SPI();

    imu_power_cycle();
 
    /* Reset icm20948 driver states */
    inv_icm20948_reset_states(&icm_device, &icm20948_serif);

    /* Setup the icm20948 device */
    rc = icm20948_sensor_setup();

    inv_icm20948_register_aux_compass(&icm_device, INV_ICM20948_COMPASS_ID_AK09916, AK0991x_DEFAULT_I2C_ADDR);
	
    if((icm_device.selftest_done) && (!icm_device.offset_done))
    {
        /* If we've run selftes and not already set the offset. */
        inv_icm20948_set_offset(&icm_device, unscaled_bias);
        icm_device.offset_done = 1;
    }
    
    /* initialize on-chip compass */
    inv_icm20948_set_slave_compass_id(&icm_device, 0U);
    inv_icm20948_set_offset(&icm_device, unscaled_bias);
	icm20948_apply_mounting_matrix();
	icm20948_set_fsr();

	/* re-initialize base state structure */
	(void)inv_icm20948_init_structure(&icm_device);

    /* Set mode */
    (void)inv_icm20948_set_lowpower_or_highperformance(&icm_device, (uint8_t)settings.mode);

    /* Set frequency */
    rc = inv_icm20948_set_sensor_period(&icm_device, idd_sensortype_conversion(INV_SENSOR_TYPE_GAME_ROTATION_VECTOR), (uint32_t)(1000 / settings.quaternion_frequency));
    rc = inv_icm20948_set_sensor_period(&icm_device, idd_sensortype_conversion(INV_SENSOR_TYPE_GYROSCOPE), (uint32_t)(1000 / settings.gyroscope_frequency));
    rc = inv_icm20948_set_sensor_period(&icm_device, idd_sensortype_conversion(INV_SENSOR_TYPE_ACCELEROMETER), (uint32_t)(1000 / settings.accelerometer_frequency));
    rc = inv_icm20948_set_sensor_period(&icm_device, idd_sensortype_conversion(INV_SENSOR_TYPE_MAGNETOMETER), (uint32_t)(1000 / settings.magnetometer_frequency));
    rc = inv_icm20948_set_sensor_period(&icm_device, idd_sensortype_conversion(INV_SENSOR_TYPE_ORIENTATION), (uint32_t)(1000 / settings.orientation_frequency));
    
    /* Enable / disable */
    rc = inv_icm20948_enable_sensor(&icm_device, idd_sensortype_conversion(INV_SENSOR_TYPE_GYROSCOPE), settings.enable_gyroscope);
    rc = inv_icm20948_enable_sensor(&icm_device, idd_sensortype_conversion(INV_SENSOR_TYPE_ACCELEROMETER), settings.enable_accelerometer);
    rc = inv_icm20948_enable_sensor(&icm_device, idd_sensortype_conversion(INV_SENSOR_TYPE_GRAVITY), settings.enable_quaternion);
    rc = inv_icm20948_enable_sensor(&icm_device, idd_sensortype_conversion(INV_SENSOR_TYPE_GAME_ROTATION_VECTOR), settings.enable_quaternion);
    rc = inv_icm20948_enable_sensor(&icm_device, idd_sensortype_conversion(INV_SENSOR_TYPE_MAGNETOMETER), settings.enable_magnetometer);
    rc = inv_icm20948_enable_sensor(&icm_device, idd_sensortype_conversion(INV_SENSOR_TYPE_ORIENTATION), true);
    
    if(rc != 0)
    {
        set_status_bit(STAT_VCU_IMU_FAULT);
    }
    
}

void imu_read_data_irq(void)
{
    (void)osif_thread_set_flag(thread_imu, IMU_TASK_IRQ_FLAG);
}

void imu_poll_sensors(void)
{
    
#ifdef USE_NATIVE_IMU_COMPUTATION
    imu_data_t imu_measurements_native;
    float_t rot_mat[9];
#endif
    
    int16_t int_read_back = 0;

#ifdef USE_FEATURE_IMU_CIRCULAR_BUFFER
    volatile uint32_t rb_ret = 0U;
#endif

    (void)osif_thread_wait_on_flag(IMU_TASK_IRQ_FLAG, OSIF_WAIT_ANY_FLAG, OSIF_WAIT_FOREVER);
    INT_SYS_DisableIRQ(PORTD_IRQn);
    
	(void)inv_icm20948_identify_interrupt(&icm_device, &int_read_back);

	if(int_read_back & (BIT_MSG_DMP_INT | BIT_MSG_DMP_INT_0))
    {
        (void)inv_icm20948_poll_sensor(&icm_device, (void*)0, build_sensor_event_data);
        
#ifndef USE_FEATURE_IMU_QUATERNIONS
        if(gyro_data_ready && accel_data_ready && quat_data_ready)
#else
        if(orientation_data_ready || accel_data_ready || mag_data_ready || gyro_data_ready)
#endif
        {
            time_step = osif_millis() - prev_time_step;
            prev_time_step += time_step;
            imu_measurements.millis = time_step;

            if(imu_ofs_calibration == 1U)
            {
                imu_offset_calibration();
                imu_orientation_offset_calibration();
            }
            
            if(imu_ofs_calibration_apply == 1U)
            {
                imu_adj_ofs();
                imu_adj_orientation_ofs();
            }

#if 0
            imu_quat_frame(imu_measurements, &pitch, &roll);
#endif
            pitch = imu_measurements.orientation_x;
            roll = imu_measurements.orientation_y;
            yaw = imu_measurements.orientation_z;

#ifdef USE_IMU_FILTER
            imu_filter(imu_measurements, &pitch, &roll, (float_t)(time_step/1000));
#endif
            gyro_data_ready = false;
            quat_data_ready = false;
            mag_data_ready = false;
            orientation_data_ready = false;
            
#ifdef USE_FEATURE_IMU_CIRCULAR_BUFFER
            imu_compress_imu_measurements();
            rb_ret = ringbuff_write(&imu_ring0, (uint8_t *)&imu_measurements_compressed, sizeof(imu_data_16_t));
#endif /* USE_FEATURE_IMU_CIRCULAR_BUFFER */
        }

#ifdef USE_NATIVE_IMU_COMPUTATION
        if((accel_data_ready == true) && (mag_data_ready == true))
        {
            imu_get_rotation_matrix(rot_mat, NULL, imu_measurements);
            imu_get_orientation_native(rot_mat, &imu_measurements_native);
            
            accel_data_ready = false;
            mag_data_ready = false;
            
            pitch_native = imu_measurements_native.orientation_x  * 57.29577;
            roll_native = imu_measurements_native.orientation_y  * 57.29577;
            yaw_native = imu_measurements_native.orientation_z  * 57.29577;
        }
#endif /* USE_NATIVE_IMU_COMPUTATION */
    }
    
    INT_SYS_EnableIRQ(PORTD_IRQn);
}

#ifdef USE_FEATURE_IMU_CIRCULAR_BUFFER
void imu_get_msg_params(void *msg)
{
    ((udp_msg_queue_obj_t *)msg)->msg = (void *)&imu_ring0;
    ((udp_msg_queue_obj_t *)msg)->msg_id = MSG_ID_IMU_DATA_S32_LTE;
    ((udp_msg_queue_obj_t *)msg)->msg_len = sizeof(imu_data_16_t);
}
#endif /* USE_FEATURE_IMU_CIRCULAR_BUFFER */

float_t imu_get_pitch(void)
{
    return pitch;
}

float_t imu_get_roll(void)
{
    return roll;
}

float_t imu_get_yaw(void)
{
    return yaw;
}

void imu_get_data(imu_data_t *imu_data)
{
    if(imu_data != NULL)
    {
        imu_data->accel_x = imu_measurements.accel_x;
        imu_data->accel_y = imu_measurements.accel_y;
        imu_data->accel_z = imu_measurements.accel_z;
        
        imu_data->gyro_x = imu_measurements.gyro_x;
        imu_data->gyro_y = imu_measurements.gyro_y;
        imu_data->gyro_z = imu_measurements.gyro_z;
        
        imu_data->mag_x = imu_measurements.mag_x;
        imu_data->mag_y = imu_measurements.mag_y;
        imu_data->mag_z = imu_measurements.mag_z;
        
        imu_data->orientation_x = imu_measurements.orientation_x;
        imu_data->orientation_y = imu_measurements.orientation_y;
        imu_data->orientation_z = imu_measurements.orientation_z;
        
        imu_data->quat_w = imu_measurements.quat_w;
        imu_data->quat_x = imu_measurements.quat_x;
        imu_data->quat_y = imu_measurements.quat_y;
        imu_data->quat_z = imu_measurements.quat_z;
    }
}

void imu_read_gyro_data(float_t *x, float_t *y, float_t *z)
{
    *x = imu_measurements.gyro_x;
    *y = imu_measurements.gyro_y;
    *z = imu_measurements.gyro_z;
    gyro_data_ready = false;
}

void imu_read_accesl_data(float_t *x, float_t *y, float_t *z)
{
    *x = imu_measurements.accel_x;
    *y = imu_measurements.accel_y;
    *z = imu_measurements.accel_z;
    accel_data_ready = false;
}

thread_id_t imu_task_get_id(void)
{
    return thread_imu;
}

__NO_RETURN static void imu_task(void *arg)
{
    status_t qstatus = STATUS_SUCCESS;
    imu_msg_queue_obj_t imu_mq;
    
    UNUSED_PARAM(arg);
    
    (void)osif_thread_wait_on_flag(IMU_TASK_START_FLAG, OSIF_WAIT_ANY_FLAG, OSIF_WAIT_FOREVER);
    NVIC_EnableIRQ(PORTD_IRQn);

    while(1)
    {
        if(imu_fault == 0U)
        {
            imu_poll_sensors();
        }        

        qstatus = osif_msg_queue_recv(imu_msg_queue, &imu_mq, NULL, 10U); 
        if(qstatus == osOK)   
        {
            switch(imu_mq.msg_id)
            {
                case IMU_MSG_CAL_START:
                    imu_ofs_calibration = 1U;
                    break;
                
                case IMU_MSG_CAL_RESET:
                    imu_calibration_reset();
                    break;
                
                default:
                    __NOP();
                    break;
            }
        }
    }
}

void imu_task_create(void)
{
    uint32_t param = NULL;
    
    nvm_read(FILE_SECT_IMU_CAL_INFO, (uint8_t *)&imu_cal_info, sizeof(imu_cal_state_t));
    imu_ofs_calibration_apply = (imu_cal_info.imu_cal_sentinel == IMU_CAL_SENTINEL) ? 1U : 0U;
    
    imu_msg_queue = osif_msg_queue_create(IMU_MSGQUEUE_OBJECTS, sizeof(imu_msg_queue_obj_t));
    DEV_ASSERT(imu_msg_queue != NULL);
    
#ifdef USE_FEATURE_IMU_CIRCULAR_BUFFER
    rb_status = ringbuff_init(&imu_ring0, imu_buffer0, 1U + (IMU_NUM_MEASUREMENTS * sizeof(imu_data_16_t)));
    DEV_ASSERT(rb_status == 1U);
#endif
    
    thread_imu = osif_thread_create(imu_task, &param, &imu_task_attr);
    DEV_ASSERT(thread_imu != NULL);
}
