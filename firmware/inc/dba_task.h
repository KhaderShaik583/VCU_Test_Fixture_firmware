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
 
#ifndef DBA_TASK_H
#define DBA_TASK_H

#include "fw_common.h"
#include "flexcan_driver.h"

#define DBA_MAX_MESSAGE_STORAGE  (20U)
#define DBA_TASK_NAME            "thread_dba"
#define DBA_STACK_SIZE           STACK_SIZE(256U)

#define DBA_TASK_START_FLAG         (0x0001U)
#define DBA_ABS_DMA_COMPLETE_FLAG   (0x0004U)

/* Definition of the TX and RX message buffers depending on the bus role */

#define CAN_DBA_TX_MAILBOX          (11UL)
#define CAN_DBA_RX_MAILBOX          (6UL)
#define CAN_DBA_RX_SLEEP_MAILBOX    (1UL)

#define DBA_CAN_IF_MBX_FILTER         (0x1FFU)  
#define DBA_CAN_IF_GBL_MBX_FILTER     (0x1FFU)
#define DBA_TO_VCU_2_0_RX_MBX         (4U)

/* ABS unit */
#define RX_ABS_MSG_ID                   (0x12BU)
#define TX_ABS_MSG_ID                   (0x450U)
#define ABS_CAN_SPEED_INFO_MSG_ID       (0x12bU)

#define ABS_LGC_MODE_SINGLE_CHANNEL     (0x4dU)
#define ABS_LGC_MODE_DUAL_CHANNEL       (0x44U)

#define ABS_MENU_DISABLE                (0xD0U)
#define ABS_MENU_ENABLE                 (0xD1U)

#define ABS_CAN_SPEED_MUL_FACTOR        (0.01f)

/* Display unit */
#define DISP_RX_MSG_ID                      (0x116U)
#define DISP_CAN_ALS_MSG_ID                 (0x117U)
#define DISP_CAN_MAG_MSG_ID                 (0x118U)

#define DBA_SEND_DISPLAY_MSG                (1U)
#define DBA_SEND_ABS_MSG                    (2U)
#define DBA_SEND_CHG_MSG                    (3U)
#define DBA_RESTART_TIMERS_MSG              (4U)
#define DBA_WAKE_DISPLAY_MSG                (5U)
#define DBA_START_DISPLAY_SLEEP_TIMER_MSG   (6U)
#define DBA_STOP_DISPLAY_SLEEP_TIMER_MSG    (7U)
#define DBA_SEND_UI_CHG_ON_NTF              (8U)
#define DBA_SEND_UI_CHG_OFF_NTF             (9U)
#define DBA_SEND_UI_CHG_PAUSE_ERR_NTF       (10U)
#define DBA_SEND_CHG_MSG_HBT_TIMER_OFF      (11U)

#define DBA_DISPLAY_TELL_TALE_CTRL_MSG          (0x4EU)
#define DBA_DISPLAY_BRIGHTNESS_CTRL_MSG         (0x6EU)
#define DBA_DISPLAY_TELL_TALE_ON                (0x1U)
#define DBA_DISPLAY_TELL_TALE_OFF               (0x0U)
#define DBA_STOP_SYS_FAIL_TIMER_MSG             (0xA0U)
#define DBA_DISP_PWR_OFF                        (0xD0U)
#define DBA_DISP_PWR_ON                         (0xD1U)
#define DBA_DISP_BRIGHTNESS_CTRL                (0xBBU)
#define DBA_DISPLAY_TELL_TALE_BLINK_CTRL_MSG    (0xBCU)

typedef enum
{
    TT_TURN_INDICATOR               = 0U,
    TT_REGEN_INDICATOR,
    TT_OVER_TEMPERATURE_INDICATOR,
    TT_HIGH_BEAM_INDICATOR,
    TT_ABS_LAMP_INDICATOR,
    TT_CHARGING_INDICATOR,
    TT_ERROR_INDICATOR,
    TT_MOTOR_ARMED_INDICATOR,
#ifdef USE_FEATURE_TRACTION_CTRL
    TT_TRACTION_CTRL_INDICATOR,
#endif
    TT_SERVICE_INDICATOR,
    
    TT_MAX_INDICATORS
}tell_tale_e;

typedef struct 
{
    uint32_t msg_id;
    uint8_t data[8];
}dba_msg_queue_obj_t;

typedef struct 
{
    uint32_t abs_state;
    uint32_t speed;
    float_t avg_speed;
    float_t distance;
    uint32_t abs_mode;
    uint32_t abs_last_mode;
    uint32_t abs_mode_perm;
    uint8_t abs_menu_state;
    osif_timer_id_t abs_comm_timer;
    osif_timer_id_t abs_info_timer;
}abs_ctx_t;

extern osif_msg_queue_id_t dba_msg_queue;

void dba_task_create(void);
osThreadId_t dba_task_get_id(void);
uint32_t abs_is_vehicle_stationary(void);
uint32_t abs_get_vehicle_speed(void);
uint32_t abs_get_vehicle_average_speed(void);
uint32_t abs_is_active(void);
void abs_get_last_mode(uint8_t *last_mode);
void abs_update_menu_state(void);
void dba_get_als_value(uint32_t *als_val);
void abs_info_timer_stop(void);
void chg_get_ui_init_state(void);
void dba_get_chg_fw_ver(uint8_t *fw_max, uint8_t *fw_min);

#endif /* DBA_TASK_H */
