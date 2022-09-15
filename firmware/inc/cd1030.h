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
 
#ifndef CD1030_H
#define CD1030_H

#include "status.h"
#include "msdi.h"

#ifdef USE_DEBUG_PRINTS
#include "debug_console.h"
#endif

#define MAX_SGX_SW      MSDI_SG_CNT_CD1030
#define MAX_SPX_SW      MSDI_SP_CNT_CD1030

#define SW_SGX_TIMEOUT  (575U)
#define SW_SPX_TIMEOUT  (575U)

#define LONG_PRESS_TIMER_TIMEOUT    (500U)
#define SHORT_PRESS_TIMER_TIMEOUT   (200U)
#define SW_IRQ_2_TIMEOUT            (50U)

#define SWIF_SW_INTERFACE_SG    (0x00005347U)
#define SWIF_SW_INTERFACE_SP    (0x00005350U)

#define SWIF_CB_TYPE_LP         (0x00004C50U)
#define SWIF_CB_TYPE_SP         (0x0000405CU)

#define SW_NO_CODE	                    (0x00U)	
#define SW_MC_ARM_DISABLED              (0xFCU)

#define LEFT_SP_CODE                    (0x31U)
#define LEFT_LP_CODE                    (0x13U)

#define RIGHT_SP_CODE                   (0xC1U)
#define RIGHT_LP_CODE                   (0x1CU)

#define ENTER_SP_CODE                   (0xC5U)
#define ENTER_LP_CODE                   (0xE1U)

#define BACK_SP_CODE                    (0xB7U)
#define BACK_LP_CODE                    (0x7BU)

#define SET_LP_PARK_ASSIST_ENTRY_CODE   (0x5DU)
#define SET_LP_PARK_ASSIST_FWD_CODE     (0x5AU)
#define SET_LP_PARK_ASSIST_REV_CODE     (0x57U)
#define SET_LP_PARK_ASSIST_EXIT_CODE    (0xD5U)

#define SG3_BACK_LP_REV_MODE_ON_CODE    (0xE9U)
#define SG3_BACK_LP_REV_MODE_OFF_CODE   (0xE6U)

#define SG3_BACK_SP_CHEAT_MODE_OFF_CODE (0xE7U)

#define SS_SP_CODE                      (0x5CU)
#define SS_LP_CODE                      (0xA9U)

#define SS_CC_DBG_SCREEN                (0xDBU)

typedef enum
{
    SWIF_STATE_ONPRESS = 0,
    SWIF_STATE_ONRELEASE,
    
    SWIF_STATE_MAX
}swif_sm_states_e;

typedef enum
{
    SW_FUNC_MODE_SP_ONLY = 1,
    SW_FUNC_MODE_LP_ONLY,
    SW_FUNC_MODE_SP_LP,
    
    SW_FUNC_MODES_MAX
}sw_func_modes_e;

void cd1030_read_swif_status(uint8_t *sw_inf);
void cd1030_irq(void);
status_t drv_swif_sem_acquire(uint32_t timeout);
status_t drv_swif_sem_release(void);
void cd1030_sem_init(void);
void cd1030_start_sp_timer(void);
void cd1030_timer_init(void);
void cd1030_initiate_swif_read(void);
uint32_t cd1030_eval_switch(uint32_t *sw_idx, uint32_t *sw_if);
uint32_t cd1030_is_lp_timer_running(void);
uint32_t cd1030_exec_cb(uint32_t sw_id, uint32_t interface, uint32_t cb_type);
void cd1030_enable_all_irqs(void);
void cd1030_set_irq_ctrl(uint32_t sw_id, uint32_t sw_if);
void cd1030_clear_port_irq(void);
void cd1030_irq_late_init(void);
void cd1030_stop_sp_timer(void);
void cd1030_start_lp_timer(void);
void cd1030_stop_lp_timer(void);
void cd1030_read_error(void);
void cd1030_clear_sw_info(void);
void check_key_insert(void);
void cd1030_inhibit_second_irq(uint32_t state);
uint32_t cd1030_is_any_sw_pressed(void);
#ifdef USE_FEATURE_SWIF_CHEAT_CODES
uint32_t cd1030_open_cheatbox(uint32_t cheats);
#endif
void key_sw_irq(void);
void cd1030_enter_LPM(void);
void cd1030_poll_switches(void);
void cd1030_disable_pa_mode(void);
uint32_t cd1030_is_high_beam_active(void);
void cd1030_get_internal_faults(uint16_t *faults);
uint8_t get_side_stand_action(void);
void set_side_stand_action(uint8_t val);
#endif /* CD1030_H */
