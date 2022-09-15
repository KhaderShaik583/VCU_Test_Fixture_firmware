 /*
 * 
 * ULTRAVIOLETTE AUTOMOTIVE CONFIDENTIAL
 * ______________________________________
 * 
 * [2017] - [2018] Ultraviolette Automotive Pvt. Ltd.
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
 * Author : Rishi F. [010]
 *


 */

#ifndef STAT_H
#define STAT_H

#include <stdint.h>
#include "fw_features.h"
typedef enum
{
    /* Status Low Bits */
    STAT_DSG_FET_STATUS_FLAG = 0U,
    STAT_CHG_FET_STATUS_FLAG,

    STAT_BAL_TIMER_STATUS_FLAG,
    STAT_BAL_ACT_STATUS_FLAG,
    
    STAT_LTC2946_DSG_ALERT_FLAG,
    STAT_LTC2946_CHG_ALERT_FLAG,
    STAT_BMS_UNECOVERABLE_FAILURE,
    
#ifdef USE_FEATURE_CHARGE_OVERVALUE_CUTOFF
    STAT_LTC2946_OVERCAPACITY_FLAG,
#endif
    STAT_UV_THR_FLAG,
    STAT_OV_THR_FLAG,
    
    STAT_ICP_CMD_FAULT_FLAG, 
    STAT_PACKET_CORRUPTION_FAULT_FLAG,
    STAT_CMAC_FAIL_FLAG,
    STAT_LOW_VOLTAGE_DETECT_FLAG,
    STAT_LTC6812_WDT_SET_FLAG,
    
#ifdef USE_FEATURE_LTC6812_REGISTER_SYNC_CHECK
    STAT_LTC6812_REG_SYNC_LOSS_FLAG,
#endif

#ifdef USE_FEATURE_LTC2946_REGISTER_SYNC_CHECK
    STAT_LTC2946_REG_SYNC_LOSS_FLAG,
#endif

    STAT_BATTERY_TEMP_OVER_MIN_THRESHOLD,
    STAT_BATTERY_TEMP_OVER_MAX_THRESHOLD,
    STAT_BATTERY_TEMP_TOO_LOW,
    STAT_LTC6812_SAFETY_TIMER_FLAG,
 
#ifdef USE_BALANCER_DEBUG_FLAG
    STAT_BALANCER_ABORT_FLAG,
    STAT_BALANCER_RESET_FLAG,
    STAT_BALANCING_COMPLETE_FLAG,
#endif
    
    STAT_LTC6812_PEC_ERROR,

    STAT_UV_OV_THR_FOR_TURN_ON,
    STAT_ECC_ERM_ERR_FLAG,
 
    STAT_DSG_INA302_ALERT1,
    STAT_DSG_INA302_ALERT2,

    STAT_MOSFET_OVER_TMP_ALERT,
    STAT_PCON_OVER_TMP_ALERT,
    STAT_NCON_OVER_TMP_ALERT,
    STAT_TOP_MCPCB_OVER_TMP_ALERT,
    STAT_BOT_MCPCB_OVER_TMP_ALERT,
    STAT_REL_HUMIDITY_OVERVALUE_ALERT,
    
    /* Status High Bits */
    STAT_DSG_FUSE_BLOWN_ALERT,
    STAT_CHG_FUSE_BLOWN_ALERT,
    STAT_FET_TURN_ON_FAILURE,
    STAT_FET_TURN_OFF_FAILURE,
    
    STAT_BAL_RES_OVER_TEMPERATURE,
    STAT_LTC2946_COMM_FAILURE,
    STAT_HW_UV_SHUTDOWN,
    STAT_HW_OV_SHUTDOWN,
    STAT_HW_OVER_TMP_SHUTDOWN,
    STAT_LTC7103_PGOOD,
    
    STAT_SYS_BOOT_FAILURE,
    STAT_MEMS_SINGLE_TAP_DETECTED,

#ifdef USE_SD_CARD_DEV
    STAT_DISK_NOT_FOUND,
#endif 

    STAT_CAN_MSG_SIG_ERR,
    STAT_FG_I2C_BUS_RECOVERY_EXEC,
    STAT_FG_MEAS_ABORT,

    MAX_STATUS_FLAGS
    
}pack_status_e;


uint64_t get_pack_status(void);
void set_pack_status(pack_status_e b);
void clear_status_bit(pack_status_e b);
void clear_all_flags(void);
void clear_select_bits(void);



#endif /* STAT_H */

