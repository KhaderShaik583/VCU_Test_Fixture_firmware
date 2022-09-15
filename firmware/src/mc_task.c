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
 *          Rishi F.   [011]
 *
 */
 
#include "mc_task.h"
#include "osif.h"
#include "udp_task.h" 
#include "lpuart_driver.h" 
#include "bcm.h"
#include "dba_task.h"
#include "lsm_task.h"
#include "odometer_task.h"
#include "can_messenger_rx.h"
#include "wdt_task.h"
#include "charger_task.h"
#include "rm_governor.h"
#include "shared_mem.h" 

#define MC_LOG_LEVEL_FULL       (1U)
#define MC_LOG_LEVEL_NONE       (0U)
#define MC_LOG_LEVEL_IDLE       (2U)

#define MC_REGEN_STATE0         (0U)
#define MC_REGEN_STATE1         (1U)
#define MC_REGEN_STATE2         (2U)
#define MC_REGEN_STATE3         (3U)
#define MC_REGEN_STATE4         (4U)

#ifdef MC_GEAR_RATIO_OLD
#define MC_GEAR_RATIO           (5.351f)
#else
#define MC_GEAR_RATIO           (5.521126761f)
#endif

#define MC_WHEEL_DIAMETER       (0.5888f)
#define MC_MSGQUEUE_OBJECTS     (8U) 

#define MC_MAX_SEG_SDO      (28U)
#define MC_SEG_SDO_LEN      (8U)

#define MOT_CONT_CAN_IF_MBX_FILTER         (0x500U)  
#define MOT_CONT_CAN_IF_GBL_MBX_FILTER     (0x500U)

#define MOT_CONT_CAN_IF_MBX_FILTER_2         (0x100U)  
#define MOT_CONT_CAN_IF_GBL_MBX_FILTER_2     (0x100U)

#define MC_NUM_PA_LUTS      (2U)
#define MC_NUM_MODES		(3U)

#define MC_MODE_BALLISTIC	(1U)
#define MC_MODE_COMBAT		(2U)
#define MC_MODE_GLIDE		(4U)

#define MC_MOTOR_MODE_SET_01    (0U)
#define MC_MOTOR_MODE_SET_02    (1U)
#define MC_MOTOR_MODE_SET_03    (2U)

#define MC_MOTOR_MAX_FWD_RPM    (7000)
#define MC_MOTOR_MAX_REV_RPM    (7000U)

#define MC_MSG_UPDATE_TORQUE        (0x3700A200U)

#define MC_CAN_MAX_TORQUE_UPDATE_ID (0x150U)
#define MC_CAN_REGEN_CONTROL_ID     (0x160U)
#define MC_CAN_TRQ_INFO_MSG         (0x770U)
#define MC_RPM_TIMER_TIMEOUT        (500U)

#define MC_MAX_MOTOR_TEMPERATURE_THRESHOLD      (120.0f)
#define MC_MIN_MOTOR_TEMPERATURE_THRESHOLD      (105.0f)

#define MC_MAX_MOTOR_HS_TEMPERATURE_THRESHOLD   (85.0f)
#define MC_MIN_MOTOR_HS_TEMPERATURE_THRESHOLD   (75.0f)

/* Target Id, Target Iq, Id, Iq */
#define MC_CAN_TPDO1_TID_TIQ_ID                 (0x101U)

/* Cutback Gain, throttle input voltage, velocity */
#define MC_CAN_TPDO2_CB_THRV_VEL                (0x102U)

/* Footbrake pot, torque demand, max. power limit torque, Uq */
#define MC_CAN_TPDO3_FB_TD_MPLT_UQ              (0x103U)

/* Capacitor voltage, battery current, LUT voltage index, voltage modulation */
#define MC_CAN_TPDO4_CV_BC_LUT_ID               (0x104U)

/* Motor Temperature, Ud, heatsink temperature, traction drive state, fwd s/w, rev sw */
#define MC_CAN_TPDO5_MT_HS_TDS_FWD_REV          (0x105U)

#define MC_CAN_FAULT_ID                         (0x81U)

#define MC_CAN_SDO_REQ_ID                       (0x601U)
#define MC_CAN_SDO_RESP_ID                      (0x581U)


#ifdef USE_FEATURE_THROTTLE_ERR_DBG
#define MC_CAN_THROTTLE_ERR_DBG_ID              (0x812U)
#endif

#define MC_MAX_TPDOS                            (11U)

typedef enum
{
    MC_REGEL_LEVEL_LOW  = 0U,
    MC_REGEN_LEVEL_MID,
    MC_REGEN_LEVEL_HIGH,
    
    MC_MAX_REGEN_LEVELS
}mc_regen_levels_e;

/* structure declaration used for storing the messages from motor controller */
typedef struct
{
    uint32_t millis;
	uint16_t msg_id;
	uint8_t raw_data[8];
}mc_store_msgs_t;


typedef struct  
{
    uint32_t millis;
    uint16_t err_code;
}mc_err_ctx_t;

typedef struct 
{
    mc_gear_e mc_current_gear;
    int32_t shaft_rpm;
    uint32_t mc_speed;
    uint32_t mc_speed_f;
    float_t mc_distance;
    float_t mc_avg_speed;
    uint32_t motor_pwr_active;
    uint32_t reserved;
    uint32_t regen_setting;
    uint32_t mc_last_err_idx;
    uint32_t motor_log_control;
    uint32_t mc_sm_state;
    mc_err_ctx_t mc_last_errs[MC_ERROR_STORE];
    float_t motor_temperature;
    float_t motor_heatsink_temperature;
    uint32_t mc_mode_selector;
    uint32_t available_ride_modes;
    uint32_t ride_mode;
}mc_ctx_t;

typedef struct
{
    mc_err_ctx_t mc_last_errs_nvm[MC_ERROR_STORE];
    uint32_t mc_last_err_idx;
    uint32_t last_ride_mode;
    uint32_t regen_setting;
    uint32_t mc_last_mode_selector;
}mc_nvm_ctxt_t;

static const uint8_t mc_fw_sdo[MC_MAX_SEG_SDO][MC_SEG_SDO_LEN] = {
    {0x40,0x00,0x10,0x00,0x00,0x00,0x00,0x00},
    {0x40,0x00,0x50,0x01,0x00,0x00,0x00,0x00},
    {0x40,0x18,0x10,0x02,0x00,0x00,0x00,0x00},
    {0x40,0x18,0x10,0x03,0x00,0x00,0x00,0x00},
    {0x40,0x00,0x29,0x00,0x00,0x00,0x00,0x00},
    {0x40,0x0A,0x10,0x00,0x00,0x00,0x00,0x00},
    {0x60,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x70,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x40,0x01,0x55,0x00,0x00,0x00,0x00,0x00},
    {0x40,0x20,0x56,0x00,0x00,0x00,0x00,0x00},
    {0x40,0x09,0x10,0x00,0x00,0x00,0x00,0x00},
    {0x60,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x70,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x40,0x18,0x10,0x02,0x00,0x00,0x00,0x00},
    {0x40,0x18,0x10,0x04,0x00,0x00,0x00,0x00},
    {0x40,0x00,0x51,0x01,0x00,0x00,0x00,0x00},
    {0x40,0x00,0x51,0x04,0x00,0x00,0x00,0x00},
    {0x40,0x08,0x10,0x00,0x00,0x00,0x00,0x00},
    {0x60,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x70,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x60,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x70,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x60,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x70,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x40,0x09,0x10,0x00,0x00,0x00,0x00,0x00},
    {0x60,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x70,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x40,0x18,0x10,0x04,0x00,0x00,0x00,0x00}
};

static lut_info_t *torque_maps_current = NULL;
static lut_info_t torque_maps_ride_nv;
static lut_info_t torque_pa_maps_nv;
static lut_info_t torque_maps_udp;

static volatile uint32_t tm_sig = 0U;
static const float_t speed_mul_factor = (MC_WHEEL_DIAMETER / MC_GEAR_RATIO);

#ifdef USE_RPDO_FOR_REGEN
/* Regen levels with a scaling factor of 0.00390625 
    Low Range Vehicles:
    Low  : 1 V   : 1 / 0.00390625   = 256
    Mid  : 1.5 V : 1.5 / 0.00390625 = 384
    High : 2 V   : 2 / 0.00390625   = 512

    High Range Vehicles:
    Low  : 1 V   : 1 / 0.00390625   = 256
    Mid  : 1.5 V : 1.5 / 0.00390625 = 384
    High : 2 V   : 2 / 0.00390625   = 512

*/
static const uint16_t mc_lr_regen_levels[MC_MAX_REGEN_LEVELS] = {256U, 384U, 512U};
static const uint16_t mc_hr_regen_levels[MC_MAX_REGEN_LEVELS] = {256U, 384U, 512U};
static uint16_t mc_regen_levels[MC_MAX_REGEN_LEVELS] = {256U, 256U, 256U};
#endif /* USE_RPDO_FOR_REGEN */

static uint32_t mc_regen_state = 0U;
static shmem_block_bms_t mc_udp_resp;
static const uint8_t mc_mode_map[MC_NUM_MODES] = {MC_MODE_GLIDE, MC_MODE_COMBAT, MC_MODE_BALLISTIC};

static volatile mc_fw_info_t mc_firmware_info;
static volatile uint8_t mc_fw_resp[MC_MAX_SEG_SDO][MC_SEG_SDO_LEN];
static uint32_t sdo_rsp_cnt = 0U;

#ifdef USE_FEATURE_S32_PA_MODE
static const int32_t mc_pa_mode_rpm_limits[4] = {0, 300, -220, 0};
#endif

/* Low Range & High Range */
static const lut_info_t torque_lr_maps = {
    MC_NUM_LUTS, 
    {
        /* GLIDE */  
        {0, 0,
            {0 ,500, 1000, 1500, 2000, 2500, 3000, 5000, 7000},
            {41, 41, 36, 32, 27, 22, 17, 2, 1}
        },  

        /* COMBAT */      
        {1, 0,
            {0, 979, 1957, 2936, 3915, 4893, 5627, 6950, 7000}, 
            {74, 74, 54, 42, 34, 26, 21, 12, 1}
        },

        /* BALLISTIC */  
        {2, 0,
            {0, 1000, 2000, 2500, 3000, 4000, 5000, 6950, 7000}, 
            {87, 87, 87, 87, 87, 87, 87, 25, 1}
        }     
    }
};

static const lut_info_t torque_hr_maps = {
    MC_NUM_LUTS, 
    {
        /* GLIDE */  
        {0, 0, 
            {0, 500, 1000, 1500, 2000, 2500, 3000, 5000, 7000}, 
            {49, 49, 43, 38, 32, 26, 20, 2, 1}
        },
        
        /* COMBAT */      
        {1, 0, 
            {0, 979, 1957, 2936, 3915, 4893, 5627, 6950, 7000}, 
            {79, 79, 59, 46, 37, 30, 25, 15, 1}
        },

        /* BALLISTIC */  
        {2, 0, 
            {0, 1000, 2000, 2500, 3000, 4000, 5000, 6950, 7000}, 
            {100, 100, 100, 100, 100, 100, 100, 29, 1}
        }
    }
};

static const lut_info_t torque_pa_maps = {
    MC_NUM_PA_LUTS,
    {
        /* PARK ASSIST FORWARD */  
        {0, 0, 
            {0,  50,  100,  125,  150,  200,  250,  290,  300},
            {49, 49,   37,   31,   26,   16,    7,    2,    1}
        },
        
        /* PARK ASSIST REVERSE */  
        {1, 0, 
            {-0,  -25, -50, -75, -110,  -160,  -185,  -210,  -220},
            {-30, -30, -30, -20,  -12,    -3,    -1,    -0,    -0}
        },
        
        /* RESERVED */
        {2, 0, 
            {0,  25, 50, 75, 100,  125,  150,  165,  180},
            {0,   0,  0,  0,   0,    0,    0,    0,    0}
        }
    }
};

#ifdef USE_FEATURE_S32_PA_MODE
static volatile uint32_t mc_pa_mode = MC_PA_MODE_OFF;
#endif

static mc_nvm_ctxt_t mc_current_nvm_ctx;
static void mc_init_nvm_ctx(void);

static const mc_nvm_ctxt_t mc_nvm_ctx = {
    .mc_last_errs_nvm = { 
        {.millis = 0U, .err_code = 0U}, 
        {.millis = 0U, .err_code = 0U}, 
        {.millis = 0U, .err_code = 0U}, 
        {.millis = 0U, .err_code = 0U}, 
        {.millis = 0U, .err_code = 0U}, 
        {.millis = 0U, .err_code = 0U}, 
        {.millis = 0U, .err_code = 0U}, 
        {.millis = 0U, .err_code = 0U}, 
    },
    
    .mc_last_err_idx = 0U,
    .last_ride_mode = MC_MODE_COMBAT,
    .regen_setting = MC_REGEN_LEVEL_MID,
    .mc_last_mode_selector = 1U
};
 
static osif_timer_id_t mc_rpm_timer;
static mc_ctx_t mc_ctxt;

/* global variables declaration */
static flexcan_msgbuff_t recvBuff;
static mc_gear_e mc_current_gear = MC_GEAR_POS_NEUTRAL;

static volatile uint32_t motor_log_control = MC_LOG_LEVEL_IDLE;
static volatile float_t new_torque = 0.0f;
static void mc_exec(void);
static void mc_regen_enable(void);
static float_t mc_avg_speed_prev = 0.0f;

static status_t mc_config_can_mbx_fifo(uint8_t instance, uint8_t mb_idx, flexcan_msgbuff_t *buff);

/* 
    New TPDOS need to be added here.
    Update MC_MAX_TPDOS in mc_task.h when adding/removing.
*/
static const uint16_t mc_tpdo_ids[MC_MAX_TPDOS] = {
    MC_CAN_TPDO1_TID_TIQ_ID,
    MC_CAN_TPDO2_CB_THRV_VEL,
    MC_CAN_TPDO3_FB_TD_MPLT_UQ,
    MC_CAN_TPDO4_CV_BC_LUT_ID,
    MC_CAN_TPDO5_MT_HS_TDS_FWD_REV,
    MC_CAN_SDO_RESP_ID,
    MC_CAN_FAULT_ID,
    MC_CAN_FAULT_ID,
    MC_CAN_FAULT_ID,
    MC_CAN_FAULT_ID,
    MC_CAN_FAULT_ID
};

#ifdef USE_FEATURE_RTX_VIEWER
__attribute__((section(".bss.os.thread.cb")))
thread_tcb_t mc_thread_tcb;
#else
static thread_tcb_t mc_thread_tcb;
#endif /* USE_FEATURE_RTX_VIEWER */

static thread_id_t thread_mc;

__attribute__((section("ARM_LIB_STACK")))
static uint64_t mc_thread_stk[MC_STACK_SIZE];

static const thread_attrs_t motor_controller_attr = {
    MC_TASK_NAME,
    osThreadDetached,
    &mc_thread_tcb,
    sizeof(mc_thread_tcb),
    &mc_thread_stk[0],
    MC_STACK_SIZE * sizeof(uint64_t),
    osPriorityNormal2,
    0U,
    0U    
};

osif_msg_queue_id_t mc_msg_queue;
void mc_rpm_timer_handler(void *arg);
void mc_can_callback(uint8_t instance, flexcan_event_type_t eventType, uint32_t idx, flexcan_state_t *flexcan_state);

static void mc_mode_status_ntf(void)
{
     switch(mc_ctxt.ride_mode)
     {
         case MC_MODE_GLIDE:
             clear_status_bit(STAT_VCU_MC_MODE_BALLISTIC);
             clear_status_bit(STAT_VCU_MC_MODE_COMBAT);
             set_status_bit(STAT_VCU_MC_MODE_GLIDE);
             break;
             
         case MC_MODE_COMBAT:
             clear_status_bit(STAT_VCU_MC_MODE_GLIDE);
             clear_status_bit(STAT_VCU_MC_MODE_BALLISTIC);
             set_status_bit(STAT_VCU_MC_MODE_COMBAT);
             break;
             
         case MC_MODE_BALLISTIC:
             clear_status_bit(STAT_VCU_MC_MODE_GLIDE);
             clear_status_bit(STAT_VCU_MC_MODE_COMBAT);
             set_status_bit(STAT_VCU_MC_MODE_BALLISTIC);
             break;
             
         default:
             __NOP();
             break;
    }
}

static float_t mc_lerp_lin(int32_t x_new_rpm, uint32_t lut)
{
    volatile float_t y_new_torque = 0.0f;
	float_t p = 0.0f;
	float_t q = 0.0f;
	float_t r = 0.0f;
    uint32_t i = 0U;
    int32_t lc = 0;
    
    DEV_ASSERT(lut < MC_NUM_LUTS);
    DEV_ASSERT(x_new_rpm >= 0);
    DEV_ASSERT(torque_maps_current != NULL);
    
    lc = osif_enter_critical();
    /* Clip at min-max values */
    if(x_new_rpm < torque_maps_current->entries[lut].rpms[0])
    {
        x_new_rpm = torque_maps_current->entries[lut].rpms[0];
    }
    
    if(x_new_rpm > torque_maps_current->entries[lut].rpms[MC_LERP_LUT_SIZE - 1])
    {
        x_new_rpm = torque_maps_current->entries[lut].rpms[MC_LERP_LUT_SIZE - 1];
    }
    
	/* 
       y = y1 + (x-x1)((y2-y1)/(x2-x1))
	   y = y1 + p * (q / r) 
    */

	for(i = 0; i < MC_LERP_LUT_SIZE; i++)
	{
        if ((x_new_rpm >= torque_maps_current->entries[lut].rpms[i]) &&
            (x_new_rpm <= torque_maps_current->entries[lut].rpms[i + 1]))
        {
            break;
        }
	}
    
    p = (float_t)(x_new_rpm - torque_maps_current->entries[lut].rpms[i]);
    q = torque_maps_current->entries[lut].torque[i + 1] - torque_maps_current->entries[lut].torque[i];
    r = torque_maps_current->entries[lut].rpms[i + 1] - torque_maps_current->entries[lut].rpms[i];

    if(r > 0.0f)
    {
        y_new_torque = torque_maps_current->entries[lut].torque[i] + (p * (q / r));
    }

    if(y_new_torque < 0.0f)
    {
        y_new_torque = 0.0f;
    }

    (void)osif_exit_critical(lc);

    return y_new_torque;   
}

#ifdef USE_FEATURE_S32_PA_MODE
static float_t mc_lerp_lin_pa_rev(int32_t x_new_rpm, uint32_t lut)
{
    volatile float_t y_new_torque = 0.0f;
	float_t p = 0.0f;
	float_t q = 0.0f;
	float_t r = 0.0f;
    uint32_t i = 0U;
    int32_t lc = 0;
    
    DEV_ASSERT(lut < MC_NUM_LUTS);
    DEV_ASSERT(x_new_rpm <= 0);
    DEV_ASSERT(torque_maps_current != NULL);
    
    lc = osif_enter_critical();
    /* Clip at min-max values */
    if(x_new_rpm > torque_maps_current->entries[lut].rpms[0])
    {
        x_new_rpm = torque_maps_current->entries[lut].rpms[0];
    }
    
    if(x_new_rpm < torque_maps_current->entries[lut].rpms[MC_LERP_LUT_SIZE - 1])
    {
        x_new_rpm = torque_maps_current->entries[lut].rpms[MC_LERP_LUT_SIZE - 1];
    }
    
	/* 
       y = y1 + (x-x1)((y2-y1)/(x2-x1))
	   y = y1 + p * (q / r) 
    */

	for(i = 0; i < MC_LERP_LUT_SIZE; i++)
	{
        if ((x_new_rpm <= torque_maps_current->entries[lut].rpms[i]) &&
            (x_new_rpm >= torque_maps_current->entries[lut].rpms[i + 1]))
        {
            break;
        }
	}
    
    p = (float_t)(x_new_rpm - torque_maps_current->entries[lut].rpms[i]);
    q = torque_maps_current->entries[lut].torque[i + 1] - torque_maps_current->entries[lut].torque[i];
    r = torque_maps_current->entries[lut].rpms[i + 1] - torque_maps_current->entries[lut].rpms[i];

    if(r < 0.0f)
    {
        y_new_torque = torque_maps_current->entries[lut].torque[i] + (p * (q / r));
    }

    if(y_new_torque > 0.0f)
    {
        y_new_torque = 0.0f;
    }

    (void)osif_exit_critical(lc);

    return y_new_torque;   
}
#endif /* USE_FEATURE_S32_PA_MODE */

static uint32_t mc_xlate_mode_to_selector(void)
{
    uint32_t sel = 0U;
    
    switch(mc_ctxt.ride_mode)
    {
        case MC_MODE_GLIDE:
            sel = 0U;
            break;

        case MC_MODE_COMBAT:
            sel = 1U;
            break;

        case MC_MODE_BALLISTIC:
            sel = 2U;
            break;

        default:
            __NOP();
            break;
    }

    return sel;
}

static void mc_task(void *arg)
{
    flexcan_msgbuff_t data_buff;
    
    UNUSED_PARAM(arg);
    (void)mc_config_can_mbx_fifo(CAN_IF_MOTOR, RX_MAILBOX, &data_buff);
    
    (void)osif_thread_wait_on_flag(MC_TASK_START_FLAG, OSIF_WAIT_ANY_FLAG, OSIF_WAIT_FOREVER);
    
    mc_rpm_timer = osif_timer_create(mc_rpm_timer_handler, osTimerOnce, NULL, NULL);
    DEV_ASSERT(mc_rpm_timer != NULL);
    
	mc_exec();

}

static void mc_init_ctxt(void)
{
    (void)memset(&mc_ctxt, 0U, sizeof(mc_ctx_t));
    
    mc_ctxt.regen_setting = mc_current_nvm_ctx.regen_setting;

    /* Ride mode will be one of MC_MODE_GLIDE, MC_MODE_COMBAT, MC_MODE_BALLISTIC */
    mc_ctxt.ride_mode = mc_current_nvm_ctx.last_ride_mode;
    mc_ctxt.mc_mode_selector = mc_current_nvm_ctx.mc_last_mode_selector;
    
    mc_mode_status_ntf();
}

static status_t mc_config_can_mbx_fifo(uint8_t instance, uint8_t mb_idx, flexcan_msgbuff_t *buff)
{
	status_t read_status = STATUS_SUCCESS;
    uint16_t id_counter = 0U;
    flexcan_id_table_t filterTable[8];
    uint16_t mc_tpdo = 0U;
    
    UNUSED_PARAM(buff);
    
	flexcan_data_info_t data_info =
    {
       .data_length = 8U,
       .msg_id_type = FLEXCAN_MSG_ID_STD,
       .enable_brs  = false,
       .fd_enable   = false,
       .fd_padding  = 0U
    };

	/* Configure RX message buffer with index RX_MSG_ID and RX_MAILBOX */
    for(mc_tpdo = 0U; mc_tpdo < MC_MAX_TPDOS; mc_tpdo++)
    {
        (void)FLEXCAN_DRV_ConfigRxMb(instance, (uint8_t)(mb_idx + mc_tpdo), &data_info, mc_tpdo_ids[mc_tpdo]);
        (void)FLEXCAN_DRV_SetRxIndividualMask(instance, FLEXCAN_MSG_ID_STD, (uint8_t)(mb_idx + mc_tpdo), mc_tpdo_ids[mc_tpdo]);
    }

	FLEXCAN_DRV_SetRxMbGlobalMask(instance, FLEXCAN_MSG_ID_STD, MOT_CONT_CAN_IF_MBX_FILTER);

	/* Fill id filter table */
	for(id_counter = 0U; id_counter < 8U; id_counter++)
	{
		filterTable[id_counter].isRemoteFrame = false;
		filterTable[id_counter].isExtendedFrame = 0U;
		filterTable[id_counter].id = 0;
	}
    
    /* Fill id filter table */
	for(mc_tpdo = 0; mc_tpdo < MC_MAX_TPDOS; mc_tpdo++)
	{
		filterTable[mc_tpdo].isRemoteFrame = false;
		filterTable[mc_tpdo].isExtendedFrame = 0U;
		filterTable[mc_tpdo].id = mc_tpdo_ids[mc_tpdo];
	}
    
	/* Configure RX FIFO ID filter table elements based on filter table defined above*/
	FLEXCAN_DRV_ConfigRxFifo(instance, FLEXCAN_RX_FIFO_ID_FORMAT_A, filterTable);
    
	/* set individual masking type */
	FLEXCAN_DRV_SetRxMaskType(instance, FLEXCAN_RX_MASK_INDIVIDUAL); 

	/* rest of filter items are masked with RXFGMASK */
	FLEXCAN_DRV_SetRxFifoGlobalMask(instance, FLEXCAN_MSG_ID_STD, MOT_CONT_CAN_IF_GBL_MBX_FILTER);

    FLEXCAN_DRV_InstallEventCallback(instance, mc_can_callback, NULL);
    
	return read_status;
}

static status_t mc_send_torque_rpdo_msg(uint16_t max_torque)
{
	status_t write_status = STATUS_SUCCESS;
	uint8_t buffer[2];
    
    buffer[0] = (uint8_t)max_torque;
    buffer[1] = (uint8_t)(max_torque >> 8U);
    
	flexcan_data_info_t tx_info =
    {
      .data_length = 2U,
      .msg_id_type = FLEXCAN_MSG_ID_STD,
      .enable_brs  = false,
      .fd_enable   = false,
      .fd_padding  = 0U
    };
    
    (void)FLEXCAN_DRV_Send(CAN_IF_MOTOR, CAN_MC_TX_MAILBOX, &tx_info, MC_CAN_MAX_TORQUE_UPDATE_ID, buffer);
	
	return write_status;   
}

static void mc_send_disp_tt_msg(uint32_t tt, uint32_t msg)
{
    dba_msg_queue_obj_t dmq;

    dmq.msg_id = DBA_SEND_DISPLAY_MSG;
    dmq.data[0] = DBA_DISPLAY_TELL_TALE_CTRL_MSG;
    dmq.data[1] = (uint8_t)tt;
    dmq.data[2] = (uint8_t)msg;
    (void)osif_msg_queue_send(dba_msg_queue, &dmq, 0U, 0U);
}

static uint32_t mc_ride_mode_to_lut_index(void)
{
    uint32_t lut_idx = 0U;
    
    if(mc_ctxt.ride_mode == MC_MODE_GLIDE)
    {
        lut_idx = 0U;
    }
    else if(mc_ctxt.ride_mode == MC_MODE_COMBAT)
    {
        lut_idx = 1U;
    }
    else if(mc_ctxt.ride_mode == MC_MODE_BALLISTIC)
    {
        lut_idx = 2U;
    }
    else
    {
        DEV_ASSERT(false);
        __NOP();
    }    

    return lut_idx;
}

#ifdef USE_FEATURE_S32_PA_MODE
static uint32_t mc_pa_mode_to_lut_index(void)
{
    uint32_t lut_idx = 0U;
    
    if(mc_pa_mode == MC_PA_MODE_REV)
    {
        lut_idx = 1U;
    }
    else if(mc_pa_mode == MC_PA_MODE_FWD)
    {
        lut_idx = 0U;
    }
    else
    {
        __NOP();
    } 
    
    return lut_idx;
}
#endif /* USE_FEATURE_S32_PA_MODE */

static uint32_t mc_eval_regen_authority(void)
{
    volatile float_t bms_soc = 0.0f;
    volatile float_t bms_temp = 0.0f;
    volatile uint32_t allow_regen = 0U;
    static volatile uint32_t prev_allow_regen = 0U;
    dba_msg_queue_obj_t dmq;
    
    bms_temp = bms_get_max_temp(0U);
    bms_soc = bms_get_soc(0U);

    if((bms_temp > 45.0f)   || 
       (bms_temp <= 0.5f)   || 
       (bms_soc > 90.0f))
    {
        allow_regen = 0U;
    }
    else
    {
        allow_regen = 1U;
    }
    
    if(prev_allow_regen != allow_regen)
    {
        /* Tell tale */
        dmq.msg_id = DBA_SEND_DISPLAY_MSG;
        dmq.data[0] = DBA_DISPLAY_TELL_TALE_CTRL_MSG;
        dmq.data[1] = TT_REGEN_INDICATOR;
        
        if(allow_regen == 1U)
        {
            dmq.data[2] = DBA_DISPLAY_TELL_TALE_OFF;
        }
        else
        {
            dmq.data[2] = DBA_DISPLAY_TELL_TALE_ON;
        }
        
        (void)osif_msg_queue_send(dba_msg_queue, &dmq, 0U, 0U);
    }
    
    prev_allow_regen = allow_regen;
    
    return allow_regen;   
}

static void mc_calc_torque_factor(float_t throttle_value, float_t *tf, uint32_t ride_mode)
{
    volatile float_t ex = 0.0f;
    volatile float_t exd = 0.0f;
    volatile float_t k = 0.0f;
    
    DEV_ASSERT(torque_maps_current != NULL);
    DEV_ASSERT(tf != NULL);
    
    if(torque_maps_current->entries[ride_mode].k > 0)
    {
        k = (float_t)(torque_maps_current->entries[ride_mode].k / 100.0f);
        
        ex = k * throttle_value;
        exd = exp(k) - 1.0f;

        if(exd > 0.0f)
        {
            *tf = (exp(ex) - 1.0f) / exd;
        }
    }
}

static void mc_calc_voltage_derating_factor(float_t *vf)
{
    static const float_t upper_threshold = 3.20f;
    static const float_t lower_threshold = 3.01f;
    
    volatile float_t min_cell_voltage = 0.0f;
    volatile float_t voltage_factor = 1.0f;
    float_t denominator = 0.0f;
    float_t numerator = 0.0f;
    
    DEV_ASSERT(vf != NULL);
    
    min_cell_voltage = bms_get_min_cell_voltage(0U);
    if(min_cell_voltage <= upper_threshold)
    {
        set_status_bit(STAT_VCU_MC_MODE_HOVER);
        denominator = upper_threshold - lower_threshold;
        if (denominator < 0.0001f)
        {
            denominator = 0.0001f;
        }
        
        numerator = min_cell_voltage - lower_threshold;
        
        if(denominator > 0.0f)
        {
            voltage_factor = numerator / denominator;
            if(voltage_factor > 1.0f)
            {
                voltage_factor = 1.0f;
            }
            
            if(voltage_factor < 0.0f)
            {
                voltage_factor = 0.0f;
            }
        }
    }
    else
    {
        clear_status_bit(STAT_VCU_MC_MODE_HOVER);
        *vf = 1.0f;
    }
    
    *vf = voltage_factor;
}

#ifdef USE_FEATURE_S32_PA_MODE
static void mc_rpm_cond_eval(int32_t *rpm)
{
    int32_t lc = 0;
    
    DEV_ASSERT(rpm != NULL);
    
    lc = osif_enter_critical();
    if(mc_pa_mode == MC_PA_MODE_REV)
    {
        if(*rpm < mc_pa_mode_rpm_limits[MC_PA_MODE_REV])
        {
            *rpm = mc_pa_mode_rpm_limits[MC_PA_MODE_REV];
        }
        else if(*rpm > 0)
        {
            *rpm = 0;
        }
        else
        {
            __NOP();
        }
    }
    else if(mc_pa_mode == MC_PA_MODE_FWD)
    {
        if(*rpm > mc_pa_mode_rpm_limits[MC_PA_MODE_FWD])
        {
            *rpm = mc_pa_mode_rpm_limits[MC_PA_MODE_FWD];
        }
        else if(*rpm < 0)
        {
            *rpm = 0;
        }
        else
        {
            __NOP();
        }
    }
    else if(mc_pa_mode == MC_PA_MODE_OFF)
    {
        if(*rpm > MC_MOTOR_MAX_FWD_RPM)
        {
            *rpm = MC_MOTOR_MAX_FWD_RPM;
        }
        else if(*rpm < 0)
        {
            /* In case of negative RPM, saturate at 0 -> Motor control team */
            *rpm = 0;
        }
        else
        {
            __NOP();
        }
    }
    else
    {
        /* It would be a fatal error if control reaches here.
           As a safety measure set rpm to 0 and signal an error.
        */
        *rpm = 0;
        set_status_bit(STAT_VCU_MC_PA_ERROR);
    }    
    
    (void)osif_exit_critical(lc);
}
#endif /* USE_FEATURE_S32_PA_MODE */

static void mc_update_torque(mc_store_msgs_t *msg)
{
    volatile int32_t rpm = 0; 
    volatile uint16_t scaled_torque = 0U;
    
    /* Refer to sevcon CANOpen dict XL index 6072H */
    static const uint16_t max_torque_scaling_factor = 10U; 
    
    uint32_t lut = 0U;
    volatile float_t torque_cutback_gain = 0.0f;
    volatile uint16_t torque_cb_gain = 0;
    
    volatile float_t mc_throttle_rx_val = 0.0f;
    volatile int16_t mc_throttle_rx = 0;

    volatile float_t torque_factor = 0.0f;
    volatile float_t voltage_factor = 0.0f;

    volatile uint32_t regen_state = 0U;
    
#ifdef USE_FEATURE_S32_PA_MODE
    int32_t lc = 0;
#endif

#ifdef USE_FEATURE_THROTTLE_ERR_DBG
    float_t throttle_voltage_ratio = 0.0f;
    uint16_t tr_s = 0U;

    mc_store_msgs_t msg_throttle;
#endif /* USE_FEATURE_THROTTLE_ERR_DBG */
	
    DEV_ASSERT(msg != NULL);
    
    rpm = (int32_t)msg->raw_data[4] | ((int32_t)msg->raw_data[5] << 8U) | ((int32_t)msg->raw_data[6] << 16U) | ((int32_t)msg->raw_data[7] << 24U);

    torque_cb_gain = (uint16_t)msg->raw_data[0] | (uint16_t)((uint16_t)msg->raw_data[1] << 8U);
    torque_cutback_gain = torque_cb_gain * 0.00390625f;
    
    mc_throttle_rx = (int16_t)msg->raw_data[2] | (int16_t)((int16_t)msg->raw_data[3] << 8U);
    mc_throttle_rx_val = mc_throttle_rx * 0.0000305185f;
    
#ifdef USE_FEATURE_S32_PA_MODE
    /* If any park assist mode is active, switch the LUT */
    if((mc_pa_mode == MC_PA_MODE_FWD) || (mc_pa_mode == MC_PA_MODE_REV))
    {
        lc = osif_enter_critical();
        lut = mc_pa_mode_to_lut_index();
        torque_maps_current = &torque_pa_maps_nv;
        (void)osif_exit_critical(lc);
    }
    else
    {
        lc = osif_enter_critical();
        lut = mc_ride_mode_to_lut_index();
        torque_maps_current = &torque_maps_ride_nv;
        osif_exit_critical(lc);
    }
    
    mc_rpm_cond_eval((int32_t *)&rpm);

#else
    if(rpm > MC_MOTOR_MAX_FWD_RPM)
    {
        rpm = MC_MOTOR_MAX_FWD_RPM;
    }
    else if(rpm < 0)
    {
        /* In case of negative RPM, saturate at 0 -> Motor control team */
        rpm = 0;
    }
    else
    {
        __NOP();
    }
    
    lut = mc_ride_mode_to_lut_index();
    torque_maps_current = &torque_maps_ride_nv;
#endif /* USE_FEATURE_S32_PA_MODE */

    torque_factor = mc_throttle_rx_val;
    mc_calc_torque_factor(mc_throttle_rx_val, (float_t *)&torque_factor, lut);
    mc_calc_voltage_derating_factor((float_t *)&voltage_factor);
    
    /* 
        If mc_regen_state = 1 & regen is allowed set torque factor to 1 
        If mc_regen_state = 0 & regen is allowed do not change torque factor
    
        If mc_regen_state = 1 & regen is not allowed do not change torque factor
    */
    regen_state = mc_regen_state & mc_eval_regen_authority();
    if(regen_state > 0U)
    {
        torque_factor = 1.0f;
    }
    
#ifdef USE_FEATURE_S32_PA_MODE
    if(mc_pa_mode == MC_PA_MODE_REV)
    {
        new_torque = mc_lerp_lin_pa_rev(rpm, lut);
    }
    else
    {
        /* As a safety measure, ensure reverse GPIO is not set */
        bcm_control(BCM_REV_CTRL, BCM_CTRL_STATE_OFF);
        new_torque = mc_lerp_lin(rpm, lut);
    }
#else
    new_torque = mc_lerp_lin(rpm, lut);
#endif
    
    new_torque = new_torque * torque_factor * torque_cutback_gain * voltage_factor;
    
    scaled_torque = (uint16_t)(new_torque * max_torque_scaling_factor);
    (void)mc_send_torque_rpdo_msg(scaled_torque);

    /* Piggyback on incoming message for logging scaled response */
    msg->msg_id = MC_CAN_TRQ_INFO_MSG;
    msg->raw_data[0] = (uint8_t)scaled_torque;
    msg->raw_data[1] = (uint8_t)(scaled_torque >> 8U);

    msg->raw_data[2] = (uint8_t)(torque_factor * 10);
    msg->raw_data[3] = (uint8_t)(voltage_factor * 10);
    msg->raw_data[4] = (uint8_t)lut;

    msg->raw_data[7] = 0U;

    (void)LPUART_DRV_SendDataBlocking(SYS_LTE_UART_IF, (uint8_t *)msg, sizeof(mc_store_msgs_t), 8U);
    
#ifdef USE_FEATURE_THROTTLE_ERR_DBG
    lsm_throttle_ratio_voltage(&throttle_voltage_ratio);

    tr_s = (uint16_t)(throttle_voltage_ratio * 10000);
    
    lc = osif_enter_critical();
    
    msg_throttle.raw_data[0] = 0U;
    msg_throttle.raw_data[1] = 0U;
    
    msg_throttle.raw_data[2] = 0U;
    msg_throttle.raw_data[3] = 0U;
    
    msg_throttle.raw_data[4] = (uint8_t)tr_s;
    msg_throttle.raw_data[5] = (uint8_t)(tr_s >> 8U);
    
    msg_throttle.raw_data[6] = 0U;
    msg_throttle.raw_data[7] = 0U;
    
    msg_throttle.millis = msg->millis;
    msg_throttle.msg_id = MC_CAN_THROTTLE_ERR_DBG_ID;
    
    osif_exit_critical(lc);
    
    (void)LPUART_DRV_SendDataBlocking(SYS_LTE_UART_IF, (uint8_t *)&msg_throttle, sizeof(mc_store_msgs_t), 8U);
    
#endif /* USE_FEATURE_THROTTLE_ERR_DBG */

}

static void mc_motor_temperature(uint8_t low_byte, uint8_t high_byte)
{
    static uint32_t mc_tmp_sm_state = 0U;
    
    mc_ctxt.motor_temperature = (float_t)((uint16_t)(low_byte)|((uint16_t)(high_byte) << 8U));
    
    switch(mc_tmp_sm_state)
    {
        case 0U:
            if(mc_ctxt.motor_temperature > MC_MAX_MOTOR_TEMPERATURE_THRESHOLD)
            {
                set_status_bit(STAT_VCU_MOTOR_OVER_TEMPERATURE);

                mc_send_disp_tt_msg(TT_OVER_TEMPERATURE_INDICATOR, DBA_DISPLAY_TELL_TALE_ON);
                mc_tmp_sm_state = 1U;
            }
            break;
            
        case 1U:
            if(mc_ctxt.motor_temperature < MC_MIN_MOTOR_TEMPERATURE_THRESHOLD)
            {
                clear_status_bit(STAT_VCU_MOTOR_OVER_TEMPERATURE);

                mc_send_disp_tt_msg(TT_OVER_TEMPERATURE_INDICATOR, DBA_DISPLAY_TELL_TALE_OFF);
                mc_tmp_sm_state = 0U;
            }
            break;
            
        default:
            __NOP();
            break;
    }
}

static void mc_heatsink_temperature(uint8_t low_byte)
{
    static uint32_t mc_hs_tmp_sm_state = 0U;
    
    mc_ctxt.motor_heatsink_temperature = (float_t)((uint16_t)(low_byte));
    
    switch(mc_hs_tmp_sm_state)
    {
        case 0U:
            if(mc_ctxt.motor_heatsink_temperature > MC_MAX_MOTOR_HS_TEMPERATURE_THRESHOLD)
            {
                set_status_bit(STAT_VCU_MOTOR_HS_OVER_TEMPERATURE);

                mc_send_disp_tt_msg(TT_OVER_TEMPERATURE_INDICATOR, DBA_DISPLAY_TELL_TALE_ON);
                mc_hs_tmp_sm_state = 1U;
            }
            break;
            
        case 1U:
            if(mc_ctxt.motor_heatsink_temperature < MC_MIN_MOTOR_HS_TEMPERATURE_THRESHOLD)
            {
                clear_status_bit(STAT_VCU_MOTOR_HS_OVER_TEMPERATURE);

                mc_send_disp_tt_msg(TT_OVER_TEMPERATURE_INDICATOR, DBA_DISPLAY_TELL_TALE_OFF);
                mc_hs_tmp_sm_state = 0U;
            }
            break;
    }
}

static void mc_gear_op_fwd(void)
{
    if(mc_ctxt.motor_pwr_active == MC_MOTOR_PWR_ON)
    {
        bcm_control(BCM_REV_CTRL, BCM_CTRL_STATE_OFF);
        clear_status_bit(STAT_VCU_MOTOR_CON_DIR_REV);
        
        bcm_control(BCM_FWD_CTRL, BCM_CTRL_STATE_ON);
        set_status_bit(STAT_VCU_MOTOR_CON_DIR_FWD);
        clear_status_bit(STAT_VCU_THROTTLE_ERROR);

        motor_log_control = MC_LOG_LEVEL_FULL;

        mc_send_disp_tt_msg(TT_MOTOR_ARMED_INDICATOR, DBA_DISPLAY_TELL_TALE_ON);
    }        
}

static void mc_gear_op_neutral(void)
{
    if(mc_ctxt.motor_pwr_active == MC_MOTOR_PWR_ON)
    {
        bcm_control(BCM_FWD_CTRL, BCM_CTRL_STATE_OFF);
        clear_status_bit(STAT_VCU_MOTOR_CON_DIR_FWD);
        
        bcm_control(BCM_REV_CTRL, BCM_CTRL_STATE_OFF);
        clear_status_bit(STAT_VCU_MOTOR_CON_DIR_REV);
        
        bcm_control(BCM_DRL_LED_CTRL, BCM_CTRL_STATE_OFF);
        bcm_control(BCM_LOW_BEAM_CTRL, BCM_CTRL_STATE_OFF);
        
        motor_log_control = MC_LOG_LEVEL_NONE;
        
        mc_send_disp_tt_msg(TT_MOTOR_ARMED_INDICATOR, DBA_DISPLAY_TELL_TALE_OFF);  
    }        
}

static void mc_gear_op_neutral_and_off(void)
{
    if(mc_ctxt.motor_pwr_active == MC_MOTOR_PWR_ON)
    {
        bcm_control(BCM_FWD_CTRL, BCM_CTRL_STATE_OFF);
        clear_status_bit(STAT_VCU_MOTOR_CON_DIR_FWD);
        
        bcm_control(BCM_REV_CTRL, BCM_CTRL_STATE_OFF);
        clear_status_bit(STAT_VCU_MOTOR_CON_DIR_REV);
        
        motor_log_control = MC_LOG_LEVEL_NONE;
        mc_set_motor_pwr_state(MC_MOTOR_PWR_OFF);
        
        bcm_control(BCM_DRL_LED_CTRL, BCM_CTRL_STATE_OFF);
        bcm_control(BCM_LOW_BEAM_CTRL, BCM_CTRL_STATE_OFF);
        
        mc_send_disp_tt_msg(TT_MOTOR_ARMED_INDICATOR, DBA_DISPLAY_TELL_TALE_OFF);  
    }     
}

static uint32_t mc_eval_gear_aux_cond(void)
{
    volatile uint32_t cond = 0U;
    volatile uint32_t side_stand_state = 0U;
    
    side_stand_state = (get_status() & ((uint64_t)1U << STAT_VCU_SIDE_STAND_DEPLOYED));
    
    if((mc_ctxt.motor_pwr_active == MC_MOTOR_PWR_ON)    &&
       (side_stand_state == 0U)                         &&
       (get_dsg_fet_state(0U) == MOSFET_ON_SW_STATE)    &&
       (chg_get_connection_state() == 0U))
    {
        cond = 1U;
    }
    else
    {
        cond = 0U;
    }
    
    return cond;
}

static void mc_cm_average(void)
{
    /* Cumulative Moving Average */
    /* Ref - https://en.wikipedia.org/wiki/Moving_average */
    static uint64_t n = 0U;
    
    mc_ctxt.mc_avg_speed = (((float_t)(n * mc_avg_speed_prev) + mc_ctxt.mc_speed) / (n + 1U));
    mc_avg_speed_prev = mc_ctxt.mc_avg_speed;
    
    n++; 
}

static void mc_regen_disable(void)
{
    /* Disable Regen */
#ifndef USE_RPDO_FOR_REGEN
    PINS_DRV_ClearPins(IO_EN_GPIO, (1U << IO_EN_PIN));
#else
    /* Scaling Factor = 0.00390625 */
	uint8_t buffer[2];
    
    buffer[0] = (uint8_t)0U;
    buffer[1] = (uint8_t)1U;
    
	flexcan_data_info_t tx_info =
    {
        .data_length = 2U,
        .msg_id_type = FLEXCAN_MSG_ID_STD,
        .enable_brs  = false,
        .fd_enable   = false,
        .fd_padding  = 0U
    };
    
    (void)FLEXCAN_DRV_Send(CAN_IF_MOTOR, CAN_MC_TX_MAILBOX, &tx_info, MC_CAN_REGEN_CONTROL_ID, buffer);  
#endif /* USE_RPDO_FOR_REGEN */
}

static void mc_regen_enable(void)
{
    /* Enable Regen */
#ifndef USE_RPDO_FOR_REGEN
    PINS_DRV_SetPins(IO_EN_GPIO, (1U << IO_EN_PIN));
#else
    /* Scaling Factor = 0.00390625 */
	uint8_t buffer[2] = {0U, 0U};
    
    if(mc_eval_regen_authority() == 1U)
    {
        if(mc_ctxt.regen_setting < MC_MAX_REGEN_LEVELS)
        {
            buffer[0] = (uint8_t)(mc_regen_levels[mc_ctxt.regen_setting] & 0x00FFU);
            buffer[1] = (uint8_t)(mc_regen_levels[mc_ctxt.regen_setting] >> 8U);
        }
    }
    else
    {
        buffer[0] = (uint8_t)0U;
        buffer[1] = (uint8_t)1U;
    }
    
	flexcan_data_info_t tx_info =
    {
      .data_length = 2U,
      .msg_id_type = FLEXCAN_MSG_ID_STD,
      .enable_brs  = false,
      .fd_enable   = false,
      .fd_padding  = 0U
    };
    
    (void)FLEXCAN_DRV_Send(CAN_IF_MOTOR, CAN_MC_TX_MAILBOX, &tx_info, MC_CAN_REGEN_CONTROL_ID, buffer);    
#endif /* USE_RPDO_FOR_REGEN */
}

static void mc_eval_ride_mode(void)
{
    float_t soc = 0.0f;
    float_t max_temp = 0.0f;
    uint32_t variant = 0U;
    int32_t lc = 0;

    lc = osif_enter_critical();
    variant = bms_get_variant_info();
    soc = bms_get_soc(0U);
    max_temp = bms_get_max_temp(0U);

    rm_gov_eval_ride_mode((uint32_t)soc, variant, max_temp, &mc_ctxt.available_ride_modes);
    (void)osif_exit_critical(lc);

    if((mc_ctxt.ride_mode & mc_ctxt.available_ride_modes) != mc_ctxt.ride_mode)
    {
        for(int32_t i = MC_NUM_MODES - 1; i >=0; i--)
        {
            if((mc_mode_map[i] & mc_ctxt.available_ride_modes) == mc_mode_map[i])
            {
                mc_ctxt.ride_mode = mc_mode_map[i];

                /* Notify automatic change of mode to cluster */
                
                mc_mode_status_ntf();
                break;
            }
        }
    }
}

static void mc_regen_control(void)
{
    volatile uint32_t is_throttle_zero = 0U;
    volatile uint32_t is_abs_active = 0U;
    static uint32_t regen_state = MC_REGEN_STATE0;
    
    is_throttle_zero = lsm_is_throttle_voltage_zero();
    is_abs_active = abs_is_active();

    switch(regen_state)
    {
        case MC_REGEN_STATE0:
            if((is_throttle_zero == 1U) &&
               (is_abs_active == 0U))
            {
                mc_regen_enable();
                set_status_bit(STAT_VCU_MC_REGEN);
                mc_regen_state = 1U;
                regen_state = MC_REGEN_STATE1;
            }
            else
            {
                mc_regen_disable();
                mc_regen_state = 0U;
            }
            break;
            
        case MC_REGEN_STATE1:
            if((is_throttle_zero == 0U) ||
               (is_abs_active == 1U))
            {
                regen_state = MC_REGEN_STATE2;
            }
            mc_regen_enable();
            mc_regen_state = 1U;
            break;
            
        case MC_REGEN_STATE2:
            mc_regen_disable();
            mc_regen_state = 0U;
            clear_status_bit(STAT_VCU_MC_REGEN);
            regen_state = MC_REGEN_STATE0;
            break;
        
        default:
            __NOP();
            break;
    }
}

static void sys_signal_ok(void)
{
    dba_msg_queue_obj_t dmq;
    
    /* Stop the system failure notification timer on display */
    dmq.msg_id = DBA_SEND_DISPLAY_MSG;
    dmq.data[0] = DBA_STOP_SYS_FAIL_TIMER_MSG;
    dmq.data[1] = 0U;
    dmq.data[2] = 0U;
    (void)osif_msg_queue_send(dba_msg_queue, &dmq, 0U, 5U);
}

static uint32_t mc_range_to_nvm_sector(void)
{   
    uint8_t range_type = 0U;
    uint32_t sector = 0U;
    
    range_type = bms_get_range_type();
    /* For redundancy factor in motor controller type Size 4 / 6 */
    if(range_type == 0U)
    {
        sector = FILE_SECT_MC_TORQUE_LR_MAPS;
    }
    else
    {
        sector = FILE_SECT_MC_TORQUE_HR_MAPS;
    }

    return sector;
}

static void init_lr_regen_levels(void)
{
    mc_regen_levels[0] = mc_lr_regen_levels[0];
    mc_regen_levels[1] = mc_lr_regen_levels[1];
    mc_regen_levels[2] = mc_lr_regen_levels[2];
}

static void init_hr_regen_levels(void)
{
    mc_regen_levels[0] = mc_hr_regen_levels[0];
    mc_regen_levels[1] = mc_hr_regen_levels[1];
    mc_regen_levels[2] = mc_hr_regen_levels[2];
}


static void init_lr_torque_maps(void)
{
    uint32_t sentinel_addr = 0U;
    uint8_t persistent_hash[CMAC_HASH_SIZE];
    uint8_t calc_persistent_hash[CMAC_HASH_SIZE];
    const uint8_t tm_sentinel[4] = {0xFEU, 0xCAU, 0x0DU, 0xD0U};
    int32_t macval = 1;
    
    volatile uint32_t persistent_nvm_sz = sizeof(lut_info_t);
    DEV_ASSERT(persistent_nvm_sz < (NVM_FILE_SECTOR_BYTES - 20U));
    
    nvm_read(FILE_SECT_MC_TORQUE_LR_MAPS, (uint8_t *)&sentinel_addr, sizeof(uint32_t));
    if(sentinel_addr != 0xD00DCAFEU)
    {
        (void)memset((uint8_t *)&torque_maps_ride_nv, 0U, sizeof(lut_info_t));
        
        (void)nvm_write(FILE_SECT_MC_TORQUE_LR_MAPS, (uint8_t *)&tm_sentinel[0], sizeof(uint32_t));
        
        /* First time configuration */
        (void)nvm_write(FILE_SECT_MC_TORQUE_LR_MAPS + 4U, (uint8_t *)&torque_lr_maps, sizeof(lut_info_t));
        
        /* Compute MAC */
        aes_cmacnvm((uint8_t *)&torque_lr_maps, sizeof(lut_info_t), persistent_hash);
        
        /* Place the MAC at the end of the sector. Leaving 4 Byte guard */
        (void)nvm_write((FILE_SECT_MC_TORQUE_LR_MAPS + (NVM_FILE_SECTOR_BYTES - 20U)), persistent_hash, CMAC_KEY_SIZE);
        
        nvm_read(FILE_SECT_MC_TORQUE_LR_MAPS + 4U, (uint8_t *)&torque_maps_ride_nv, sizeof(lut_info_t));
    }
    else
    {
        /* Load NVM */
        nvm_read(FILE_SECT_MC_TORQUE_LR_MAPS + 4U, (uint8_t *)&torque_maps_ride_nv, sizeof(lut_info_t));
        
        /* Load hash */
        nvm_read((FILE_SECT_MC_TORQUE_LR_MAPS + (NVM_FILE_SECTOR_BYTES - 20U)), persistent_hash, CMAC_KEY_SIZE);
        
        /* Compute MAC */
        aes_cmacnvm((uint8_t *)&torque_maps_ride_nv, sizeof(lut_info_t), calc_persistent_hash);
        
        macval = memcmp(calc_persistent_hash, persistent_hash, CMAC_KEY_SIZE); 
        
        if(macval != 0)
        {
            /* If integrity check fail, load default map */
            memset((uint8_t *)&torque_maps_ride_nv, 0U, sizeof(lut_info_t));
            
            (void)nvm_write(FILE_SECT_MC_TORQUE_LR_MAPS, (uint8_t *)&tm_sentinel[0], sizeof(uint32_t));
            
            /* First time configuration */
            (void)nvm_write(FILE_SECT_MC_TORQUE_LR_MAPS + 4U, (uint8_t *)&torque_lr_maps, sizeof(lut_info_t));
            
            /* Compute MAC */
            aes_cmacnvm((uint8_t *)&torque_lr_maps, sizeof(lut_info_t), persistent_hash);
            
            /* Place the MAC at the end of the sector. Leaving 4 Byte guard */
            (void)nvm_write((FILE_SECT_MC_TORQUE_LR_MAPS + (NVM_FILE_SECTOR_BYTES - 20U)), persistent_hash, CMAC_KEY_SIZE);
            
            nvm_read(FILE_SECT_MC_TORQUE_LR_MAPS + 4U, (uint8_t *)&torque_maps_ride_nv, sizeof(lut_info_t));
            
            set_status_bit(STAT_VCU_MC_TMAP_LOAD_FAIL);
        }      
    }
}

static void init_hr_torque_maps(void)
{
    uint32_t sentinel_addr = 0U;
    uint8_t persistent_hash[CMAC_HASH_SIZE];
    uint8_t calc_persistent_hash[CMAC_HASH_SIZE];
    const uint8_t tm_sentinel[4] = {0xFEU, 0xCAU, 0x0DU, 0xD0U};
    int32_t macval = 1;
    
    volatile uint32_t persistent_nvm_sz = sizeof(lut_info_t);
    DEV_ASSERT(persistent_nvm_sz < (NVM_FILE_SECTOR_BYTES - 20U));
    
    nvm_read(FILE_SECT_MC_TORQUE_HR_MAPS, (uint8_t *)&sentinel_addr, sizeof(uint32_t));
    if(sentinel_addr != 0xD00DCAFEU)
    {
        (void)memset((uint8_t *)&torque_maps_ride_nv, 0U, sizeof(lut_info_t));
        
        (void)nvm_write(FILE_SECT_MC_TORQUE_HR_MAPS, (uint8_t *)&tm_sentinel[0], sizeof(uint32_t));
        
        /* First time configuration */
        (void)nvm_write(FILE_SECT_MC_TORQUE_HR_MAPS + 4U, (uint8_t *)&torque_hr_maps, sizeof(lut_info_t));
        
        /* Compute MAC */
        aes_cmacnvm((uint8_t *)&torque_hr_maps, sizeof(lut_info_t), persistent_hash);
        
        /* Place the MAC at the end of the sector. Leaving 4 Byte guard */
        (void)nvm_write((FILE_SECT_MC_TORQUE_HR_MAPS + (NVM_FILE_SECTOR_BYTES - 20U)), persistent_hash, CMAC_KEY_SIZE);
        
        nvm_read(FILE_SECT_MC_TORQUE_HR_MAPS + 4U, (uint8_t *)&torque_maps_ride_nv, sizeof(lut_info_t));
    }
    else
    {
        /* Load NVM */
        nvm_read(FILE_SECT_MC_TORQUE_HR_MAPS + 4U, (uint8_t *)&torque_maps_ride_nv, sizeof(lut_info_t));
        
        /* Load hash */
        nvm_read((FILE_SECT_MC_TORQUE_HR_MAPS + (NVM_FILE_SECTOR_BYTES - 20U)), persistent_hash, CMAC_KEY_SIZE);
        
        /* Compute MAC */
        aes_cmacnvm((uint8_t *)&torque_maps_ride_nv, sizeof(lut_info_t), calc_persistent_hash);
        
        macval = memcmp(calc_persistent_hash, persistent_hash, CMAC_KEY_SIZE); 
        
        if(macval != 0)
        {
            /* If integrity check fail, load default map */
            memset((uint8_t *)&torque_maps_ride_nv, 0U, sizeof(lut_info_t));
            
            (void)nvm_write(FILE_SECT_MC_TORQUE_HR_MAPS, (uint8_t *)&tm_sentinel[0], sizeof(uint32_t));
            
            /* First time configuration */
            (void)nvm_write(FILE_SECT_MC_TORQUE_HR_MAPS + 4U, (uint8_t *)&torque_hr_maps, sizeof(lut_info_t));
            
            /* Compute MAC */
            aes_cmacnvm((uint8_t *)&torque_hr_maps, sizeof(lut_info_t), persistent_hash);
            
            /* Place the MAC at the end of the sector. Leaving 4 Byte guard */
            (void)nvm_write((FILE_SECT_MC_TORQUE_HR_MAPS + (NVM_FILE_SECTOR_BYTES - 20U)), persistent_hash, CMAC_KEY_SIZE);
            
            nvm_read(FILE_SECT_MC_TORQUE_HR_MAPS + 4U, (uint8_t *)&torque_maps_ride_nv, sizeof(lut_info_t));
            
            set_status_bit(STAT_VCU_MC_TMAP_LOAD_FAIL);
        }      
    }
}

static void init_pa_maps(void)
{
    uint32_t sentinel_addr = 0U;
    uint8_t persistent_hash[CMAC_HASH_SIZE];
    uint8_t calc_persistent_hash[CMAC_HASH_SIZE];
    const uint8_t tm_sentinel[4] = {0x77U, 0xFEU, 0xBAU, 0xB0U};
    int32_t macval = 1;
    
    volatile uint32_t persistent_nvm_sz = sizeof(lut_info_t);
    DEV_ASSERT(persistent_nvm_sz < (NVM_FILE_SECTOR_BYTES - 20U));
    
    nvm_read(FILE_SECT_MC_TORQUE_PA_MAPS, (uint8_t *)&sentinel_addr, sizeof(uint32_t));
    if(sentinel_addr != 0xB0BAFE77U)
    {
        (void)memset((uint8_t *)&torque_pa_maps_nv, 0U, sizeof(lut_info_t));
        
        (void)nvm_write(FILE_SECT_MC_TORQUE_PA_MAPS, (uint8_t *)&tm_sentinel[0], sizeof(uint32_t));
        
        /* First time configuration */
        (void)nvm_write(FILE_SECT_MC_TORQUE_PA_MAPS + 4U, (uint8_t *)&torque_pa_maps, sizeof(lut_info_t));
        
        /* Compute MAC */
        aes_cmacnvm((uint8_t *)&torque_pa_maps, sizeof(lut_info_t), persistent_hash);
        
        /* Place the MAC at the end of the sector. Leaving 4 Byte guard */
        (void)nvm_write((FILE_SECT_MC_TORQUE_PA_MAPS + (NVM_FILE_SECTOR_BYTES - 20U)), persistent_hash, CMAC_KEY_SIZE);
        
        nvm_read(FILE_SECT_MC_TORQUE_PA_MAPS + 4U, (uint8_t *)&torque_pa_maps_nv, sizeof(lut_info_t));
    }
    else
    {
        /* Load NVM */
        nvm_read(FILE_SECT_MC_TORQUE_PA_MAPS + 4U, (uint8_t *)&torque_pa_maps_nv, sizeof(lut_info_t));
        
        /* Load hash */
        nvm_read((FILE_SECT_MC_TORQUE_PA_MAPS + (NVM_FILE_SECTOR_BYTES - 20U)), persistent_hash, CMAC_KEY_SIZE);
        
        /* Compute MAC */
        aes_cmacnvm((uint8_t *)&torque_pa_maps_nv, sizeof(lut_info_t), calc_persistent_hash);
        
        macval = memcmp(calc_persistent_hash, persistent_hash, CMAC_KEY_SIZE); 
        
        if(macval != 0)
        {
            /* If integrity check fail, load default map */
            (void)memset((uint8_t *)&torque_pa_maps_nv, 0U, sizeof(lut_info_t));
            
            (void)nvm_write(FILE_SECT_MC_TORQUE_PA_MAPS, (uint8_t *)&tm_sentinel[0], sizeof(uint32_t));
            
            /* First time configuration */
            (void)nvm_write(FILE_SECT_MC_TORQUE_PA_MAPS + 4U, (uint8_t *)&torque_pa_maps, sizeof(lut_info_t));
            
            /* Compute MAC */
            aes_cmacnvm((uint8_t *)&torque_pa_maps, sizeof(lut_info_t), persistent_hash);
            
            /* Place the MAC at the end of the sector. Leaving 4 Byte guard */
            (void)nvm_write((FILE_SECT_MC_TORQUE_PA_MAPS + (NVM_FILE_SECTOR_BYTES - 20U)), persistent_hash, CMAC_KEY_SIZE);
            
            nvm_read(FILE_SECT_MC_TORQUE_PA_MAPS + 4U, (uint8_t *)&torque_pa_maps_nv, sizeof(lut_info_t));
            
            set_status_bit(STAT_VCU_MC_TMAP_LOAD_FAIL);
        }      
    }    
}

static void mc_init_nvm_ctx(void)
{
    uint32_t sentinel_addr = 0U;
    uint8_t persistent_hash[CMAC_HASH_SIZE];
    uint8_t calc_persistent_hash[CMAC_HASH_SIZE];
    const uint8_t tm_sentinel[4] = {0x77U, 0xFEU, 0xCAU, 0xC0U};
    int32_t macval = 1;
    
    volatile uint32_t persistent_nvm_sz = sizeof(mc_nvm_ctxt_t);
    DEV_ASSERT(persistent_nvm_sz < (NVM_FILE_SECTOR_BYTES - 20U));
    
    nvm_read(FILE_SECT_MC_CTXT, (uint8_t *)&sentinel_addr, sizeof(uint32_t));
    if(sentinel_addr != 0xC0CAFE77U)
    {
        (void)memset((uint8_t *)&mc_current_nvm_ctx, 0U, sizeof(mc_nvm_ctxt_t));
        
        (void)nvm_write(FILE_SECT_MC_CTXT, (uint8_t *)&tm_sentinel[0], sizeof(uint32_t));
        
        /* First time configuration */
        (void)nvm_write(FILE_SECT_MC_CTXT + 4U, (uint8_t *)&mc_nvm_ctx, sizeof(mc_nvm_ctxt_t));
        
        /* Compute MAC */
        aes_cmacnvm((uint8_t *)&mc_nvm_ctx, sizeof(mc_nvm_ctxt_t), persistent_hash);
        
        /* Place the MAC at the end of the sector. Leaving 4 Byte guard */
        (void)nvm_write((FILE_SECT_MC_CTXT + (NVM_FILE_SECTOR_BYTES - 20U)), persistent_hash, CMAC_KEY_SIZE);
        
        nvm_read(FILE_SECT_MC_CTXT + 4U, (uint8_t *)&mc_current_nvm_ctx, sizeof(mc_nvm_ctxt_t));
    }
    else
    {
        /* Load NVM */
        nvm_read(FILE_SECT_MC_CTXT + 4U, (uint8_t *)&mc_current_nvm_ctx, sizeof(mc_nvm_ctxt_t));
        
        /* Load hash */
        nvm_read((FILE_SECT_MC_CTXT + (NVM_FILE_SECTOR_BYTES - 20U)), persistent_hash, CMAC_KEY_SIZE);
        
        /* Compute MAC */
        aes_cmacnvm((uint8_t *)&mc_current_nvm_ctx, sizeof(mc_nvm_ctxt_t), calc_persistent_hash);
        
        macval = memcmp(calc_persistent_hash, persistent_hash, CMAC_KEY_SIZE); 
        
        if(macval != 0)
        {
            /* If integrity check fail, load default map */
            (void)memset((uint8_t *)&mc_current_nvm_ctx, 0U, sizeof(mc_nvm_ctxt_t));
            
            nvm_write(FILE_SECT_MC_CTXT, (uint8_t *)&tm_sentinel[0], sizeof(uint32_t));
            
            /* First time configuration */
            (void)nvm_write(FILE_SECT_MC_CTXT + 4U, (uint8_t *)&mc_nvm_ctx, sizeof(mc_nvm_ctxt_t));
            
            /* Compute MAC */
            aes_cmacnvm((uint8_t *)&mc_nvm_ctx, sizeof(mc_nvm_ctxt_t), persistent_hash);
            
            /* Place the MAC at the end of the sector. Leaving 4 Byte guard */
            (void)nvm_write((FILE_SECT_MC_CTXT + (NVM_FILE_SECTOR_BYTES - 20U)), persistent_hash, CMAC_KEY_SIZE);
            
            nvm_read(FILE_SECT_MC_CTXT + 4U, (uint8_t *)&mc_current_nvm_ctx, sizeof(mc_nvm_ctxt_t));
        }      
    }        
}

static void init_regen_levels(void)
{
    volatile uint8_t range_type = 0U;
    
    range_type = bms_get_range_type();

    if(range_type == 0U)
    {
        /* Load low range settings */
        init_lr_regen_levels();
    }
    else
    {
        /* Load high range settings */
        init_hr_regen_levels();
    }

    __DMB();
}

static void init_torque_maps(void)
{
    volatile uint8_t range_type = 0U;
    
    range_type = bms_get_range_type();
    
    if(range_type == 0U)
    {
        /* Load low range settings */
        init_lr_torque_maps();
    }
    else
    {
        /* Load high range settings */
        init_hr_torque_maps();
    }
    
    init_pa_maps();
    __DMB();
    
}

static void mc_process_fw_info(void)
{
    /* Motor controller s/w version is placed in sdo index 6 and 7 in ASCII */
    mc_firmware_info.sw_version[0] = mc_fw_resp[6][1];
    mc_firmware_info.sw_version[1] = mc_fw_resp[6][2];
    mc_firmware_info.sw_version[2] = mc_fw_resp[6][3];
    mc_firmware_info.sw_version[3] = mc_fw_resp[6][4];
    mc_firmware_info.sw_version[4] = mc_fw_resp[6][5];
    mc_firmware_info.sw_version[5] = mc_fw_resp[6][6];
    mc_firmware_info.sw_version[6] = mc_fw_resp[6][7];
    mc_firmware_info.sw_version[7] = mc_fw_resp[7][1];
    mc_firmware_info.sw_version[8] = mc_fw_resp[7][2];
    mc_firmware_info.sw_version[9] = '\0';
    
    /* Motor controller h/w version is placed in sdo index 11 and 12 in ASCII */
    mc_firmware_info.hw_version[0] = '0';
    mc_firmware_info.hw_version[1] = 'x';
    mc_firmware_info.hw_version[2] = mc_fw_resp[11][3];
    mc_firmware_info.hw_version[3] = mc_fw_resp[11][4];
    mc_firmware_info.hw_version[4] = mc_fw_resp[11][5];
    mc_firmware_info.hw_version[5] = mc_fw_resp[11][6];
    mc_firmware_info.hw_version[6] = mc_fw_resp[11][7];
    mc_firmware_info.hw_version[7] = mc_fw_resp[12][1];
    mc_firmware_info.hw_version[8] = mc_fw_resp[12][2];
    mc_firmware_info.hw_version[9] = mc_fw_resp[12][3];
    mc_firmware_info.hw_version[10] = '\0';
    
    /* Motor controller serial no. is placed in sdo index 27 as an integer value */
    mc_firmware_info.serial_no = 0U;
    
    mc_firmware_info.serial_no |= (uint32_t)mc_fw_resp[27][4];
    mc_firmware_info.serial_no |= ((uint32_t)mc_fw_resp[27][5] << 8U);
    mc_firmware_info.serial_no |= ((uint32_t)mc_fw_resp[27][6] << 16U);
    mc_firmware_info.serial_no |= ((uint32_t)mc_fw_resp[27][7] << 24U);
}

static void mc_sw_version_purge(void)
{
    static uint32_t sdo_num = 0U;
    status_t s = STATUS_SUCCESS;
    uint8_t sdo_buffer[MC_SEG_SDO_LEN];
    int32_t lc = 0;
    
    flexcan_data_info_t tx_info =
    {
      .data_length = 8U,
      .msg_id_type = FLEXCAN_MSG_ID_STD,
      .enable_brs  = false,
      .fd_enable   = false,
      .fd_padding  = 0U
    };
    
    if(sdo_num < MC_MAX_SEG_SDO)
    {
        lc = osif_enter_critical();
        
        sdo_buffer[0] = mc_fw_sdo[sdo_num][0];
        sdo_buffer[1] = mc_fw_sdo[sdo_num][1];
        sdo_buffer[2] = mc_fw_sdo[sdo_num][2];
        sdo_buffer[3] = mc_fw_sdo[sdo_num][3];
        sdo_buffer[4] = mc_fw_sdo[sdo_num][4];
        sdo_buffer[5] = mc_fw_sdo[sdo_num][5];
        sdo_buffer[6] = mc_fw_sdo[sdo_num][6];
        sdo_buffer[7] = mc_fw_sdo[sdo_num][7];
        
        (void)osif_exit_critical(lc);
        
        s = FLEXCAN_DRV_Send(CAN_IF_MOTOR, CAN_MC_TX_MAILBOX, &tx_info, MC_CAN_SDO_REQ_ID, sdo_buffer);  
        if(s == STATUS_SUCCESS)
        {
            sdo_num++;
        }        
    }    
}

static void mc_process_sdo_rsp(mc_store_msgs_t msg)
{
    if(sdo_rsp_cnt < MC_MAX_SEG_SDO)
    {
        for(uint32_t i = 0U; i < MC_SEG_SDO_LEN; i++)
        {
            mc_fw_resp[sdo_rsp_cnt][i] = msg.raw_data[i];
        }
        
        sdo_rsp_cnt++;
        if(sdo_rsp_cnt >= MC_MAX_SEG_SDO)
        {
            /* Need to block all SDO requests after this so that DVT tool does not interefere */
            FLEXCAN_DRV_SetRxMbGlobalMask(CAN_IF_MOTOR, FLEXCAN_MSG_ID_STD, MOT_CONT_CAN_IF_MBX_FILTER_2);
            FLEXCAN_DRV_SetRxFifoGlobalMask(CAN_IF_MOTOR, FLEXCAN_MSG_ID_STD, MOT_CONT_CAN_IF_GBL_MBX_FILTER_2);
            
            mc_process_fw_info();
        }
    }    
}

static void update_torque_maps(void)
{
    int16_t num_entries = torque_maps_udp.num_of_entries;
    
    DEV_ASSERT(num_entries <= (int16_t)MC_NUM_LUTS);
    
    if((mc_current_gear == MC_GEAR_POS_NEUTRAL) && 
       (lsm_is_throttle_voltage_zero() == 1U)   &&
       (abs_is_vehicle_stationary() == 1U)      &&
       (mc_ctxt.motor_pwr_active == MC_MOTOR_PWR_ON))
    {
        for(int32_t i = 0; i < num_entries; i++)
        {
            torque_maps_ride_nv.entries[torque_maps_udp.entries[i].lut_idx].k = torque_maps_udp.entries[torque_maps_udp.entries[i].lut_idx].k;
            for(int32_t j = 0; j < (int32_t)MC_LERP_LUT_SIZE; j++)
            {
                torque_maps_ride_nv.entries[torque_maps_udp.entries[i].lut_idx].rpms[j] =  torque_maps_udp.entries[i].rpms[j];
                torque_maps_ride_nv.entries[torque_maps_udp.entries[i].lut_idx].torque[j] =  torque_maps_udp.entries[i].torque[j];
            }
        }
    }
}

uint32_t mc_get_available_ride_modes(void)
{
    return mc_ctxt.available_ride_modes;
}

uint32_t mc_get_motor_pwr_state(void)
{
    return mc_ctxt.motor_pwr_active;
}

uint32_t mc_get_ride_mode(void)
{
    return mc_ctxt.ride_mode;
}

void mc_get_regen_level(void)
{
    int32_t lc = 0;
    udp_msg_queue_obj_t udp_msg;

    mc_udp_resp.udp_msg.cmd = MSG_ID_GET_REGEN_LEVEL_IMX_S32;
    mc_udp_resp.udp_msg.len = 1U;
    
    lc = osif_enter_critical();
    mc_udp_resp.buffer[0] = (uint8_t)mc_ctxt.regen_setting;
    osif_exit_critical(lc);

    udp_msg.msg = (void *)&mc_udp_resp;
    udp_msg.msg_id = mc_udp_resp.udp_msg.cmd;
    udp_msg.msg_len = mc_udp_resp.udp_msg.len;
    (void)osif_msg_queue_send(udp_msg_queue, &udp_msg, 0U, 2U);
}

void mc_set_regen_level(uint32_t mc_regen_setting)
{
    int32_t lc = 0;
    
    if(mc_regen_setting < (uint32_t)MC_MAX_REGEN_LEVELS)
    {
        lc = osif_enter_critical();
        mc_ctxt.regen_setting = (uint32_t)mc_regen_setting;
        osif_exit_critical(lc);
    }
}

void mc_ctxt_update_persitence(void)
{
    uint8_t persistent_hash[CMAC_HASH_SIZE];
    
    mc_current_nvm_ctx.last_ride_mode = mc_ctxt.ride_mode;
    mc_current_nvm_ctx.mc_last_err_idx = mc_ctxt.mc_last_err_idx;
    mc_current_nvm_ctx.regen_setting = mc_ctxt.regen_setting;
    
    mc_current_nvm_ctx.mc_last_mode_selector = mc_xlate_mode_to_selector();
    
    for(uint32_t i = 0U; i < MC_ERROR_STORE; i++)
    {
        mc_current_nvm_ctx.mc_last_errs_nvm[i] = mc_ctxt.mc_last_errs[i];
    }

    /* Write Current NVM */
    (void)nvm_write(FILE_SECT_MC_CTXT + 4U, (uint8_t *)&mc_current_nvm_ctx, sizeof(mc_nvm_ctxt_t));
        
    /* Compute MAC */
    aes_cmacnvm((uint8_t *)&mc_current_nvm_ctx, sizeof(mc_nvm_ctxt_t), persistent_hash);
        
    /* Place the MAC at the end of the sector. Leaving 4 Byte guard */
    (void)nvm_write((FILE_SECT_MC_CTXT + (NVM_FILE_SECTOR_BYTES - 20U)), persistent_hash, CMAC_KEY_SIZE);   
}

void mc_set_motor_pwr_state(uint32_t state)
{
    switch(state)
    {
        case MC_MOTOR_PWR_OFF:
            mc_set_pa_mode(MC_PA_MODE_OFF);
            bcm_control(BCM_MOTOR_PWR_CTRL, BCM_CTRL_STATE_OFF);
            mc_ctxt.motor_pwr_active = state;
            clear_status_bit(STAT_VCU_MOTOR_CON_KEY_SW_ON);
            clear_status_bit(STAT_VCU_MC_MODE_HOVER);
            break;
            
        case MC_MOTOR_PWR_ON:
            bcm_control(BCM_MOTOR_PWR_CTRL, BCM_CTRL_STATE_ON);
            set_status_bit(STAT_VCU_MOTOR_CON_KEY_SW_ON);
            mc_ctxt.motor_pwr_active = state;
            (void)osif_timer_start(mc_rpm_timer, MSEC_TO_TICK(MC_RPM_TIMER_TIMEOUT)); 
            break;
            
        default:
            __NOP();
            break;
    }
}

void mc_rpm_timer_handler(void *arg)
{
    UNUSED_PARAM(arg);
    
    mc_ctxt.mc_speed = 0U;    
    (void)osif_timer_stop(mc_rpm_timer);
}

thread_id_t mc_task_get_id(void)
{
    return thread_mc;
}

void mc_task_create(void)
{
    uint32_t param = NULL;
    
    mc_msg_queue = osif_msg_queue_create(MC_MSGQUEUE_OBJECTS, sizeof(mc_msg_queue_obj_t));
    DEV_ASSERT(mc_msg_queue != NULL);

    thread_mc = osif_thread_create(mc_task, &param, &motor_controller_attr);
    DEV_ASSERT(thread_mc != NULL);

    mc_set_gear(MC_GEAR_POS_NEUTRAL);
	mc_init_nvm_ctx();
    mc_init_ctxt();
}

void mc_can_callback(uint8_t instance, flexcan_event_type_t eventType, uint32_t idx, flexcan_state_t *flexcan_state)
{
    UNUSED_VAR(flexcan_state);
    UNUSED_VAR(instance);
    UNUSED_VAR(idx);
    
    if(eventType == FLEXCAN_EVENT_DMA_COMPLETE)
    {
        (void)osif_thread_set_flag(thread_mc, MC_TASK_DMA_COMPLETE_FLAG);    
    }
}

void mc_get_motor_temperature(float_t *mc_temp)
{
    *mc_temp = mc_ctxt.motor_temperature;
}

void mc_get_heatsink_temperature(float_t *mc_hs_temp)
{
    *mc_hs_temp = mc_ctxt.motor_heatsink_temperature;
}

void mc_get_shaft_rpm(int32_t *sr)
{
    *sr = mc_ctxt.shaft_rpm;
}

#ifdef USE_FEATURE_S32_PA_MODE
void mc_set_gear_irq(mc_gear_e new_gear)
{
    /* Special case for PA mode */
    
    if((mc_current_gear == MC_GEAR_POS_NEUTRAL) && 
       (new_gear == MC_GEAR_POS_FWD)            &&
       (mc_eval_gear_aux_cond() == 1U))
    {
        mc_current_gear = MC_GEAR_POS_FWD;
        mc_gear_op_fwd();
        
        clear_status_bit(STAT_VCU_MC_TMAP_UPDATED);
        clear_status_bit(STAT_VCU_MOTOR_CON_FAULT);
    }
    else if((mc_current_gear == MC_GEAR_POS_FWD) && 
            (new_gear == MC_GEAR_POS_FWD))
    {
        mc_current_gear = MC_GEAR_POS_FWD;
    }
    else if(new_gear == MC_GEAR_POS_NEUT_OFF)
    {
        mc_gear_op_neutral_and_off();
    }
    else
    {
        mc_current_gear = MC_GEAR_POS_NEUTRAL;
        mc_gear_op_neutral();
    }   
}
#endif /* USE_FEATURE_S32_PA_MODE */

void mc_set_gear(mc_gear_e new_gear)
{
    int32_t lc = 0;

    lc = osif_enter_critical();
    
    if((mc_current_gear == MC_GEAR_POS_NEUTRAL) && 
       (new_gear == MC_GEAR_POS_FWD)            &&
       (mc_eval_gear_aux_cond() == 1U))
    {
        mc_current_gear = MC_GEAR_POS_FWD;
        mc_gear_op_fwd();
        
        clear_status_bit(STAT_VCU_MC_TMAP_UPDATED);
        clear_status_bit(STAT_VCU_MOTOR_CON_FAULT);
    }
    else if((mc_current_gear == MC_GEAR_POS_FWD) && 
            (new_gear == MC_GEAR_POS_FWD))
    {
        mc_current_gear = MC_GEAR_POS_FWD;
    }
    else if(new_gear == MC_GEAR_POS_NEUT_OFF)
    {
        mc_gear_op_neutral_and_off();
    }
    else
    {
        mc_current_gear = MC_GEAR_POS_NEUTRAL;
        mc_gear_op_neutral();
    }
    
    (void)osif_exit_critical(lc);
}

void mc_get_gear(mc_gear_e *g, uint32_t *pa_mode)
{
    if((g != NULL) && (pa_mode != NULL))
    {
        *g = mc_current_gear;
        *pa_mode = mc_pa_mode;
    }
}

void mc_mode_cycle(void)
{
    int32_t lc = 0;
	volatile uint32_t mode = 0U;
    volatile uint32_t temp = mc_ctxt.ride_mode;

    if((lsm_is_throttle_voltage_zero() == 1U) &&
       (mc_ctxt.motor_pwr_active == MC_MOTOR_PWR_ON))
    {
        lc = osif_enter_critical();

		mc_ctxt.mc_mode_selector++;
		if(mc_ctxt.mc_mode_selector >= MC_NUM_MODES)
		{
			mc_ctxt.mc_mode_selector = 0U;
		}
		
		if((mc_mode_map[mc_ctxt.mc_mode_selector] & mc_ctxt.available_ride_modes) == mc_mode_map[mc_ctxt.mc_mode_selector])
		{
			mode = mc_mode_map[mc_ctxt.mc_mode_selector];
		}
		else
		{
		    /* Notify mode not available */
            
		    /* Stay in the current mode */
			mode = temp;
		}

		mc_ctxt.ride_mode = mode;
        (void)osif_exit_critical(lc);

        mc_mode_status_ntf();

    }
}

#ifdef USE_FEATURE_S32_PA_MODE
uint32_t mc_eval_pa_mode_entry(void)
{
    int32_t lc = 0;
    float_t soc = 0.0f;
    volatile uint32_t side_stand_state = 0U;
    uint32_t p = MC_PA_MODE_NOT_AVAILABLE;
    
    /* Ensure no context switches occur during this evaluation */
    lc = osif_enter_critical();
    
    /* Do not allow PA mode when SOC is less than 5 % */
    soc = bms_get_soc(0U);
    
    side_stand_state = (get_status() & ((uint64_t)1U << STAT_VCU_SIDE_STAND_DEPLOYED));
    
    /* Do not allow PA mode entry when SOC is less than 5 % */
    if((lsm_is_throttle_voltage_zero() == 1U)           &&
       (mc_ctxt.motor_pwr_active == MC_MOTOR_PWR_ON)    &&
       (mc_is_vehicle_stationary() == 1U)               &&
       (get_dsg_fet_state(0U) == MOSFET_ON_SW_STATE)    &&
       (side_stand_state == 0U)                         &&
       (soc > 5.0f))
    {
        p = MC_PA_MODE_AVAILABLE;
    }
    
    (void)osif_exit_critical(lc);
    
    return p;
}

void mc_set_pa_mode_off(mc_gear_e gear)
{
    int32_t lc = 0;
    
    /* Ensure no context switches occur during this evaluation */
    lc = osif_enter_critical();
    mc_pa_mode = MC_PA_MODE_OFF;
    (void)osif_exit_critical(lc);
    
    clear_status_bit(STAT_VCU_PA_MODE_ENTRY);
    clear_status_bit(STAT_VCU_PA_MODE_REV);
    clear_status_bit(STAT_VCU_PA_MODE_FWD);
    
    clear_status_bit(STAT_VCU_PA_MODE_ERROR);
    clear_status_bit(STAT_VCU_MC_PA_ERROR);
    
    /*
        On exiting PA mode transition to the specified
        gear setting in the arguments 
    */
    mc_set_gear(gear);
}

void mc_set_pa_mode_off_irq(mc_gear_e gear)
{
    /* Ensure no context switches occur during this evaluation */
    mc_pa_mode = MC_PA_MODE_OFF;
    
    clear_status_bit(STAT_VCU_PA_MODE_ENTRY);
    clear_status_bit(STAT_VCU_PA_MODE_REV);
    clear_status_bit(STAT_VCU_PA_MODE_FWD);
    
    clear_status_bit(STAT_VCU_PA_MODE_ERROR);
    clear_status_bit(STAT_VCU_MC_PA_ERROR);
    
    /*
        On exiting PA mode transition to the specified
        gear setting in the arguments 
    */
    mc_set_gear_irq(gear);
}

static void mc_set_pa_rev(void)
{
    set_status_bit(STAT_VCU_PA_MODE_REV);
    clear_status_bit(STAT_VCU_PA_MODE_FWD);
    
    bcm_control(BCM_FWD_CTRL, BCM_CTRL_STATE_OFF);
    bcm_control(BCM_REV_CTRL, BCM_CTRL_STATE_ON); 
}

static void mc_set_pa_fwd(void)
{
    clear_status_bit(STAT_VCU_PA_MODE_REV);
    set_status_bit(STAT_VCU_PA_MODE_FWD);
    
    bcm_control(BCM_FWD_CTRL, BCM_CTRL_STATE_ON);
    bcm_control(BCM_REV_CTRL, BCM_CTRL_STATE_OFF);  
}

static void mc_set_pa_off(void)
{
    clear_status_bit(STAT_VCU_PA_MODE_ENTRY);
    clear_status_bit(STAT_VCU_PA_MODE_REV);
    clear_status_bit(STAT_VCU_PA_MODE_FWD);
    
    mc_set_gear(MC_GEAR_POS_NEUTRAL); 
    mc_send_disp_tt_msg(TT_MOTOR_ARMED_INDICATOR, DBA_DISPLAY_TELL_TALE_OFF);
}

int32_t mc_set_pa_mode(uint32_t pa_mode)
{
    int32_t ret = 0;
    int32_t lc = 0;
    volatile uint32_t side_stand_state = 0U;
    
    side_stand_state = (get_status() & ((uint64_t)1U << STAT_VCU_SIDE_STAND_DEPLOYED));

    /* Double check here as a safety measure */
    if((lsm_is_throttle_voltage_zero() == 1U)           &&
       (mc_ctxt.motor_pwr_active == MC_MOTOR_PWR_ON)    &&
       (mc_is_vehicle_stationary() == 1U)               &&
       (get_dsg_fet_state(0U) == MOSFET_ON_SW_STATE)    &&
       (side_stand_state == 0U))
    {
        /* Ensure no context switches occur while updating the PA mode */
        lc = osif_enter_critical();
        mc_pa_mode = pa_mode;
        (void)osif_exit_critical(lc);
        
        mc_set_gear(MC_GEAR_POS_NEUTRAL);
        if(mc_pa_mode == MC_PA_MODE_REV)
        {
            mc_set_pa_rev();
        }
        else if(mc_pa_mode == MC_PA_MODE_FWD)
        {
            mc_set_pa_fwd();
        }
        else if(mc_pa_mode == MC_PA_MODE_OFF)
        {
            mc_set_pa_off();
        }
        else 
        {
            /*  Control must not reach here. As a safety measure put the 
                vehicle in neutral 
            */
            mc_set_pa_off();
            mc_set_gear(MC_GEAR_POS_NEUTRAL);
            set_status_bit(STAT_VCU_MC_PA_ERROR);
        }

        if((mc_pa_mode == MC_PA_MODE_REV) ||
           (mc_pa_mode == MC_PA_MODE_FWD))
        {
            mc_send_disp_tt_msg(TT_MOTOR_ARMED_INDICATOR, DBA_DISPLAY_TELL_TALE_ON);
        }
    }    
    else
    {
        ret = -1;
    }

    return ret;
}
#endif /* USE_FEATURE_S32_PA_MODE */

uint32_t mc_is_vehicle_stationary(void)
{
    uint32_t ret = 0U;
    
    if(mc_ctxt.mc_speed == 0U)
    {
        ret = 1U;
    }
    
    return ret;
}

uint32_t mc_get_vehicle_speed(void)
{
    return mc_ctxt.mc_speed;
}

uint32_t mc_get_vehicle_average_speed(void)
{
    return (uint32_t)mc_ctxt.mc_avg_speed;
}

void mc_commit_torque_map_signal(void)
{
    uint8_t persistent_hash[CMAC_HASH_SIZE];
    uint32_t sector = 0U;

    sector = mc_range_to_nvm_sector();

    (void)nvm_write(sector + 4U, (uint8_t *)&torque_maps_ride_nv, sizeof(lut_info_t));
    
    /* Compute MAC */
    aes_cmacnvm((uint8_t *)&torque_maps_ride_nv, sizeof(lut_info_t), persistent_hash);
    
    /* Place the MAC at the end of the sector. Leaving 4 Byte guard */
    (void)nvm_write((sector + (NVM_FILE_SECTOR_BYTES - 20U)), persistent_hash, CMAC_KEY_SIZE);
    
    set_status_bit(STAT_VCU_MC_TMAP_COMITTED);
}

void mc_update_torque_map_signal(lut_info_t *maps)
{
    DEV_ASSERT(maps != NULL);
    
    int16_t num_entries = maps->num_of_entries;
    
    DEV_ASSERT(num_entries <= (int16_t)MC_NUM_LUTS);
    
    if(tm_sig == 0U)
    {
        if((mc_current_gear == MC_GEAR_POS_NEUTRAL) && 
           (lsm_is_throttle_voltage_zero() == 1U)   &&
           (abs_is_vehicle_stationary() == 1U)      &&
           (mc_ctxt.motor_pwr_active == MC_MOTOR_PWR_ON))
        {
            torque_maps_udp.num_of_entries = num_entries;
            
            for(int32_t i = 0; i < num_entries; i++)
            {
                torque_maps_udp.entries[i].lut_idx = maps->entries[i].lut_idx;
                torque_maps_udp.entries[i].k = maps->entries[i].k;
                for(int32_t j = 0; j < (int32_t)MC_LERP_LUT_SIZE; j++)
                {
                    torque_maps_udp.entries[i].rpms[j] = maps->entries[i].rpms[j];
                    torque_maps_udp.entries[i].torque[j] = maps->entries[i].torque[j];
                }
            }
            
            tm_sig = 0xFFDDAB2CU;
            set_status_bit(STAT_VCU_MC_TMAP_UPDATED);
        }
    }
}

uint32_t mc_read_tmap_info(uint8_t *buffer)
{   
	if(buffer != NULL)
	{
    	(void)memcpy(buffer, (uint8_t *)&torque_maps_ride_nv, sizeof(lut_info_t));
    	(void)memcpy(buffer + sizeof(lut_info_t), (uint8_t *)&torque_pa_maps_nv, sizeof(lut_info_t));
	} 
	
    return (2U * sizeof(lut_info_t));
}

void mc_factory_reset_ride_tmaps(void)
{
    uint32_t sector = 0U;
    uint8_t sentinel[4] = {0xFFU, 0xFFU, 0xFFU, 0xFFU};
    
    sector = mc_range_to_nvm_sector();
    
    (void)nvm_write(sector, sentinel, 4U);
    
    /* Reset the PA mode maps */
    sector = FILE_SECT_MC_TORQUE_PA_MAPS;
    (void)nvm_write(sector, sentinel, 4U);
    
    set_status_bit(STAT_VCU_MC_TMAP_FACT_RESET);
}

void mc_factory_reset(void)
{
    uint32_t sector = 0U;
    uint8_t sentinel[4] = {0xFFU, 0xFFU, 0xFFU, 0xFFU};
    
    /* Reset the PA mode maps */
    sector = FILE_SECT_MC_CTXT;
    (void)nvm_write(sector, sentinel, 4U);
    
    set_status_bit(STAT_VCU_MC_FACT_RESET);
}

void mc_get_fw_info(mc_fw_info_t *msg)
{
    msg = (mc_fw_info_t *)&mc_firmware_info;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : mc_exec
 * Description   : storing the received messages bytewise
 *
 * Implements    : store_rcvmsgs_Activity
 *END**************************************************************************/
__NO_RETURN static void mc_exec(void)
{
	int32_t lc = 0;
	status_t status = STATUS_SUCCESS;
    mc_store_msgs_t msg;
    uint32_t mc_task_start_flag = 0U;
    volatile float_t time_diff = 0.0f;
    uint32_t current_millis = 0U;
    uint32_t prev_millis = 0U;
    float_t mc_speed_f = 0.0f;
    dba_msg_queue_obj_t dmq;
    
    /* Wait for bms task to signal on a successful FET turn ON */
    mc_task_start_flag = osif_thread_wait_on_flag(MC_TASK_FET_ON_FLAG, OSIF_WAIT_ANY_FLAG, OSIF_WAIT_FOREVER);
    (void)osif_thread_clear_flag(MC_TASK_FET_ON_FLAG);
    
    lc = osif_enter_critical();
    init_torque_maps();
    init_regen_levels();
    (void)osif_exit_critical(lc);
    
    if((mc_task_start_flag == MC_TASK_FET_ON_FLAG) && 
       (chg_get_connection_state() == 0U))
    {
        mc_set_motor_pwr_state(MC_MOTOR_PWR_ON);
        sys_signal_ok();
    }
    else if(chg_get_connection_state() == 1U)
    {
        sys_signal_ok();
    }
    else
    {
        __NOP();
    }

	(void)FLEXCAN_DRV_RxFifo(CAN_IF_MOTOR, &recvBuff);
    (void)osif_timer_start(mc_rpm_timer, MSEC_TO_TICK(MC_RPM_TIMER_TIMEOUT)); 
    
    /* Disable Regen */
    mc_regen_disable();
    
    if(mc_eval_regen_authority() == 0U)
    {
        dmq.msg_id = DBA_SEND_DISPLAY_MSG;
        dmq.data[0] = DBA_DISPLAY_TELL_TALE_CTRL_MSG;
        dmq.data[1] = TT_REGEN_INDICATOR;
        dmq.data[2] = DBA_DISPLAY_TELL_TALE_ON;
        
        (void)osif_msg_queue_send(dba_msg_queue, &dmq, 0U, 0U);   
    }
       
	while(1)
    {
        (void)osif_thread_wait_on_flag(MC_TASK_DMA_COMPLETE_FLAG, OSIF_WAIT_ANY_FLAG, OSIF_WAIT_FOREVER);
        (void)osif_thread_clear_flag(MC_TASK_DMA_COMPLETE_FLAG);

        lc = osif_enter_critical();
        
        msg.millis = osif_millis();
        msg.msg_id = (uint16_t)recvBuff.msgId;
        
        msg.raw_data[0] = recvBuff.data[0];
        msg.raw_data[1] = recvBuff.data[1];
        msg.raw_data[2] = recvBuff.data[2];
        msg.raw_data[3] = recvBuff.data[3];
        msg.raw_data[4] = recvBuff.data[4];
        msg.raw_data[5] = recvBuff.data[5];
        msg.raw_data[6] = recvBuff.data[6];
        msg.raw_data[7] = recvBuff.data[7];

        if(tm_sig == 0xFFDDAB2CU)
        {
            update_torque_maps();
            tm_sig = 0U;
        }

        (void)FLEXCAN_DRV_RxFifo(CAN_IF_MOTOR, &recvBuff);

        (void)osif_exit_critical(lc);

        if(motor_log_control == MC_LOG_LEVEL_FULL)
        {
            status = LPUART_DRV_SendDataBlocking(SYS_LTE_UART_IF, (uint8_t *)&msg, sizeof(mc_store_msgs_t), 30U);
            DEV_ASSERT(status == STATUS_SUCCESS);
        }
        else if(motor_log_control == MC_LOG_LEVEL_NONE)
        {
            status = LPUART_DRV_SendDataBlocking(SYS_LTE_UART_IF, (uint8_t *)&msg, sizeof(mc_store_msgs_t), 30U);
            DEV_ASSERT(status == STATUS_SUCCESS);
            
            msg.millis = osif_millis();
            msg.msg_id = (uint16_t)666U;
            
            msg.raw_data[0] = 0xFFU;
            msg.raw_data[1] = 0xAFU;
            msg.raw_data[2] = 0xACU;
            msg.raw_data[3] = 0xA5U;
            msg.raw_data[4] = 0x5AU;
            msg.raw_data[5] = 0xCAU;
            msg.raw_data[6] = 0xFAU;
            msg.raw_data[7] = 0xFFU; 
            
            status = LPUART_DRV_SendDataBlocking(SYS_LTE_UART_IF, (uint8_t *)&msg, sizeof(mc_store_msgs_t), 30U);
            DEV_ASSERT(status == STATUS_SUCCESS);
            
            motor_log_control = MC_LOG_LEVEL_IDLE;
        }
        else
        {
            __NOP();
        }
        
        switch(msg.msg_id)
        {
            case MC_CAN_TPDO2_CB_THRV_VEL:
                current_millis = osif_millis();
                if(current_millis > prev_millis)
                {
                    /* Time difference in sec */
                    time_diff = (float_t)(current_millis - prev_millis) / 1000.0f;
                }
                else
                {
                    time_diff = (float_t)(((0xFFFFFFFFU - prev_millis) + 1U) + current_millis) / 1000.0f;
                }
                
                mc_ctxt.shaft_rpm = (int32_t)msg.raw_data[4] | ((int32_t)msg.raw_data[5] << 8U) | ((int32_t)msg.raw_data[6] << 16U) | ((int32_t)msg.raw_data[7] << 24U);
                /* 
                    Shaft RPM is observed to be negative under ceratin circumstances, e.g. regen
                    If it is below zero the value will be set to zero.
                */
                if(mc_ctxt.shaft_rpm < 0)
                {
                    if(mc_pa_mode == MC_PA_MODE_REV)
                    {
                        mc_ctxt.shaft_rpm = mc_ctxt.shaft_rpm * (-1);
                    }
                    else
                    {
                        mc_ctxt.shaft_rpm = 0;
                    }
                }
                
                mc_ctxt.mc_speed =  (uint32_t)(0.1885f * (float_t)mc_ctxt.shaft_rpm * speed_mul_factor);
                mc_speed_f = 0.1885f * (float_t)mc_ctxt.shaft_rpm * speed_mul_factor;
                mc_ctxt.mc_distance = mc_speed_f * time_diff * 0.00027777777f;
                update_counters(mc_ctxt.mc_distance);

                mc_cm_average();
                prev_millis = current_millis;

                /* All processing for this TPDO must be before calculating motor torque */
                mc_update_torque(&msg);

                (void)osif_timer_start(mc_rpm_timer, MSEC_TO_TICK(MC_RPM_TIMER_TIMEOUT)); 

                break;
                
            case MC_CAN_FAULT_ID:
                set_status_bit(STAT_VCU_MOTOR_CON_FAULT);
                mc_ctxt.mc_last_errs[mc_ctxt.mc_last_err_idx].err_code = msg.raw_data[0];
                mc_ctxt.mc_last_errs[mc_ctxt.mc_last_err_idx].millis = osif_millis();
                mc_ctxt.mc_last_err_idx++;
            
                if(mc_ctxt.mc_last_err_idx >= MC_ERROR_STORE)
                {
                    mc_ctxt.mc_last_err_idx = 0U;
                }
                break;
                
            case MC_CAN_TPDO5_MT_HS_TDS_FWD_REV:
                mc_motor_temperature(msg.raw_data[0], msg.raw_data[1]);
                mc_heatsink_temperature(msg.raw_data[4]);
                mc_sw_version_purge();
                break;
                    
            case MC_CAN_SDO_RESP_ID:
                mc_process_sdo_rsp(msg);
                break;
            
            default:
                __NOP();
                break;
        }
        
        mc_regen_control();
        mc_eval_ride_mode();
	}
}	

