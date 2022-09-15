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
 
#ifndef MC_TASK_H
#define MC_TASK_H

#include "fw_common.h"
#include "flexcan_driver.h"

/* Definition of the TX and RX message buffers depending on the bus role */

#define CAN_MC_TX_MAILBOX   (11UL)
#define RX_MAILBOX          (6UL)

#define MC_TASK_START_FLAG          (0x0001U)
#define MC_TASK_DMA_COMPLETE_FLAG   (0x0004U)
#define MC_TASK_FET_ON_FLAG         (0x0008U)
#define MC_TASK_MAP_UPDATE_FLAG     (0x0010U)

#define MC_MAX_MESSAGE_STORAGE  20U
#define MC_TASK_NAME            "thread_mcif"
#define MC_STACK_SIZE           STACK_SIZE(128U)

#define MC_MOTOR_PWR_ON         (1U)
#define MC_MOTOR_PWR_OFF        (0U)

#define MC_LERP_LUT_SIZE    (9U)
#define MC_NUM_LUTS         (3U)
#define MC_ERROR_STORE      (16U)

#define MC_PA_MODE_AVAILABLE        (1U)
#define MC_PA_MODE_NOT_AVAILABLE    (0U)

#define MC_PA_MODE_FWD      (1U)
#define MC_PA_MODE_REV      (2U)
#define MC_PA_MODE_OFF      (4U)

typedef struct 
{
    char sw_version[9 + 1];
    char hw_version[10 + 1];
    uint32_t serial_no;
}mc_fw_info_t;

typedef struct 
{
    uint32_t msg_id;
    uint8_t data;
}mc_msg_queue_obj_t;

typedef enum
{
    MC_GEAR_POS_NEUTRAL = 0U,
    MC_GEAR_POS_FWD,
    MC_GEAR_POS_PA,
    MC_GEAR_POS_NEUT_OFF,
    
    MC_GEAR_POS_MAX,
}mc_gear_e;

typedef struct 
{
    int16_t lut_idx;
    int16_t k;
    int16_t rpms[MC_LERP_LUT_SIZE];
    int16_t torque[MC_LERP_LUT_SIZE];
}lut_entry_t;

typedef struct
{
    int16_t num_of_entries;
    lut_entry_t entries[MC_NUM_LUTS];
}lut_info_t;

void mc_task_create(void);
osThreadId_t mc_task_get_id(void);
void mc_set_gear(mc_gear_e new_gear);
void mc_set_gear_v0(mc_gear_e new_gear);
void mc_get_gear(mc_gear_e *g, uint32_t *pa_mode);

void mc_set_motor_pwr_state(uint32_t state);
uint32_t mc_get_motor_pwr_state(void);
void mc_get_motor_temperature(float_t *mc_temp);
void mc_get_heatsink_temperature(float_t *mc_hs_temp);
void mc_get_shaft_rpm(int32_t *sr);
void mc_mode_cycle(void);
uint32_t mc_get_speed(void);

void mc_update_torque_map_signal(lut_info_t *maps);
void mc_commit_torque_map_signal(void);

uint32_t mc_is_vehicle_stationary(void);
uint32_t mc_get_vehicle_speed(void);
uint32_t mc_get_vehicle_average_speed(void);
void mc_get_fw_info(mc_fw_info_t *msg);
uint32_t mc_get_available_ride_modes(void);
uint32_t mc_get_ride_mode(void);
void mc_ctxt_update_persitence(void);
uint32_t mc_read_tmap_info(uint8_t *buffer);
void mc_factory_reset_ride_tmaps(void);

#ifdef USE_FEATURE_S32_PA_MODE
uint32_t mc_eval_pa_mode_entry(void);
int32_t mc_set_pa_mode(uint32_t pa_mode);
void mc_set_pa_mode_off(mc_gear_e gear);
void mc_set_pa_mode_off_irq(mc_gear_e gear);
void mc_set_gear_irq(mc_gear_e new_gear);
#endif

void mc_get_regen_level(void);
void mc_set_regen_level(uint32_t mc_regen_setting);
void  mc_factory_reset(void);

#endif /* MC_TASK_H */
