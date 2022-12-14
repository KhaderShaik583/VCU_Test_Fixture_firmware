#ifndef FW_FEATURES_H
#define FW_FEATURES_H

#define USE_FEATURE_DEV_ERROR_DETECT
#define USE_FEATURE_RTX_OS

#define USE_FEATURE_MPU
#define USE_FEATURE_CORE_DUMP
#define USE_FEATURE_FLEXCAN_IF
#define USE_FEATURE_CAN_BUS_ENCRYPTION
#define USE_FEATURE_CORE_DUMP
#define DEV_ERROR_DETECT
#define USE_FEATURE_ALL_FET_CMD
#define USE_FEATURE_FLEXCAN_IF
#define USE_FEATURE_CAN_BUS_ENCRYPTION
#define USE_FEATURE_SWIF_IRQ
#define USE_FEATURE_CAN_BUS_TIMEOUT
#define USE_CALC_CELL_DELTA_VOLTAGE
#define USE_FEATURE_RTX_VIEWER
#define USE_FEATURE_BOSCH_ABS
#define USE_RPDO_FOR_REGEN

#ifdef USE_FEATURE_VCU_ON_DESK
#define USE_DEBUG_PRINTS
#endif

#define USE_DEBUG_PRINTS

#define USE_RANGE_PRED_ALG0
#define USE_FEATURE_LP_SLEEP_MODE
#define USE_FEATURE_IMU_QUATERNIONS
#define USE_FEATURE_S32_PA_MODE
#define USE_FEATURE_FAST_TURN_ON
#define USE_FEATURE_THROTTLE_ERR_DBG

/* #define USE_FEATURE_SWIF_CHEAT_CODES */
/* #define USE_NW_SW_DEBUG */ 
/* #define USE_FEATURE_IMU_CIRCULAR_BUFFER */
/* #define USE_NATIVE_IMU_COMPUTATION */

/* Software AES-128 is only used for testing & validation and not for production */
//#define USE_SW_AES_MOD

#endif /* FW_FEATURES_H */

