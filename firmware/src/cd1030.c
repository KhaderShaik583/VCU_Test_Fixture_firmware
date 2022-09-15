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
 
#include <stdint.h>
#include <status.h>

#include "fw_common.h" 
#include "cd1030.h"
#include "pins_port_hw_access.h" 
#include "pins_driver.h"
#include "osif.h"
#include "bcm.h"

#include "can_messenger_rx.h"
#include "mc_task.h"
#include "dba_task.h"
#include "imu.h" 

#include "charger_task.h"

#ifdef USE_FEATURE_VCU_ON_DESK
#include "bms_task.h"
#endif

#define SWIF_SG_ID              (0x3EU)
#define SWIF_SP_ID              (0x4AU)

#define PA_MODE_ENTRY_SENTINEL  (0x5D74FCFFU)
#define PA_MODE_EXIT_SENTINEL   (0x3774FDFEU)

/* Pins used in design */

/* SG4 */
#define SW_LEFT_PIN		        (4U)
#define SW_LEFT_PRESS(x)	    (((x) & (1U << SW_LEFT_PIN)) == (1U << SW_LEFT_PIN)) 

/* SG5 */
#define SW_RIGHT_PIN		    (5U)
#define SW_RIGHT_PRESS(x)	    (((x) & (1U << SW_RIGHT_PIN)) == (1U << SW_RIGHT_PIN))

/* SG3 */
#define SW_ENTER_PIN		    (3U)
#define SW_ENTER_PRESS(x)	    (((x) & (1U << SW_ENTER_PIN)) == (1U << SW_ENTER_PIN))

/* SP1 */
#define SW_HIGH_BEAM_PIN	    (1U)
#define SW_HIGH_BEAM_PRESS(x)	(((x) & (1U << SW_HIGH_BEAM_PIN)) == (1U << SW_HIGH_BEAM_PIN))

#define SW_NEUTRAL_PIN		    (20U)
#define SW_NEUTRAL_PRESS(x)	    (((x) & (1U << SW_NEUTRAL_PIN)) == (1U << SW_NEUTRAL_PIN))

/* SG0 */
#define SW_BACK_PIN		        (0U)

#define SW_START_PIN		    (2U)
#define SW_START_PRESS(x)	    (((x) & (1U << SW_START_PIN)) == (1U << SW_START_PIN))

/* SP4 */
#define SW_REAR_BREAK_PIN	    (4U)
#define SW_REAR_BREAK_PRESS(x)	(((x) & (1U << SW_REAR_BREAK_PIN)) == (1U << SW_REAR_BREAK_PIN))

/* SP5 */
#define SW_FRONT_BREAK_PIN	    (5U)
#define SW_FRONT_BREAK_PRESS(x)	(((x) & (1U << SW_FRONT_BREAK_PIN)) == (1U << SW_FRONT_BREAK_PIN))

/* SG8 */
#define SW_SIDE_STAND_PIN	    (8U)
#define SW_SIDE_STAND_PRESS(x)	(((x) & (1U << SW_SIDE_STAND_PIN)) == (1U << SW_SIDE_STAND_PIN))

/* Side stand state machine states */
#define S0  (0U)
#define S1  (1U)
#define S2  (2U)
#define S3  (3U)

typedef void (*sw_callback)(void);

typedef struct 
{
    uint32_t group;         /* SP = 1 or SG = 0*/
    uint32_t sw_state;      /* ON / OFF */
}sw_state_machine_t;

typedef struct
{
    uint32_t sw_idx;
    uint32_t function;
    int32_t buddy;
}sw_ctx_descriptor_t;

#ifdef USE_FEATURE_S32_PA_MODE
/* 
    cd1030_pa_mode_ctxt variable keeps track of the park assist mode context from the 
    cd1030 perspective.
*/
static volatile uint32_t cd1030_pa_mode_ctx = PA_MODE_EXIT_SENTINEL;
static void cd1030_set_pa_mode_ctx(uint32_t pa_mode);
static uint32_t mc_pa_mode_idx = 1U;

#endif /* USE_FEATURE_S32_PA_MODE */

static void sw_sg0_cb(void);
static void sw_sg1_cb(void);
static void sw_sg2_cb(void);
static void sw_sg3_cb(void);
static void sw_sg4_cb(void);
static void sw_sg5_cb(void);
static void sw_sg6_cb(void);
static void sw_sg7_cb(void);
static void sw_sg8_cb(void);
static void sw_sg9_cb(void);
static void sw_sg10_cb(void);
static void sw_sg11_cb(void);
static void sw_sg12_cb(void);
static void sw_sg13_cb(void);
static void sw_sg20_cb(void);

static void sw_sg0_lp_cb(void);
static void sw_sg2_lp_cb(void);
static void sw_sg3_lp_cb(void);
static void sw_sg4_lp_cb(void);
static void sw_sg5_lp_cb(void);
static void sw_sg12_lp_cb(void);

static void sw_sp0_cb(void);
static void sw_sp1_cb(void);
static void sw_sp2_cb(void);
static void sw_sp3_cb(void);
static void sw_sp4_cb(void);
static void sw_sp5_cb(void);
static void sw_sp6_cb(void);
static void sw_sp7_cb(void);

static void sw_sp10_lp_cb(void);

static msdi_drv_config_t drv_config = {.drvInstance = 0U, .deviceType = MSDI_DEVICE_CD1030};

static uint32_t sp_old   = 0U;
static uint32_t sg_old   = 0U; 
static uint32_t sp      = 0U;
static uint32_t sg      = 0U;

static uint32_t lp_timer_args;
static uint32_t high_beam_active = 0U;
static uint16_t msdi_fault_state = 0U;

/* Driver semaphore */
static semaphore_t drv_swif_sem_t;
static bool key_sw_on                   = false;
static uint32_t is_hazard_sw_activated  = 0U;

/* argument for the timer call back function */     
static volatile uint8_t sw_info = 0U;
static osif_timer_id_t long_press_timer;
static volatile uint32_t is_long_press_timer_running = 0U;
static volatile uint32_t inhibit_second_irq = 0U;

static sw_state_machine_t sgx_sm[MAX_SGX_SW];
static sw_state_machine_t spx_sm[MAX_SPX_SW];

void cd1030_lp_timer_handler(void *arg);
static uint8_t cd1030_eval_pa_mode_entry(void);

#ifdef USE_FEATURE_SWIF_CHEAT_CODES
static uint32_t cheat_state = 0U;
#endif

static const sw_ctx_descriptor_t sw_sg_ctxt[MAX_SGX_SW] = 
{
    {0U, SW_FUNC_MODE_SP_LP,    4},     /* ENTER/SET */
    {1U, 3U, -1},   
    {2U, SW_FUNC_MODE_SP_LP,    -1},    /* MODE */
    {3U, SW_FUNC_MODE_SP_LP,    -1},    /* BACK */
    {4U, SW_FUNC_MODE_SP_LP,    -1},    /* LEFT */
    {5U, SW_FUNC_MODE_SP_LP,    -1},    /* RIGHT */
    {6U, 3U, -1},
    {7U, 3U, -1},
    {8U, 3U, -1},
    {9U, 3U, -1},
    {10U, 3U, -1},
    {11U, SW_FUNC_MODE_SP_ONLY, -1},    /* HAZARD */
    {12U, 3U, -1},
    {13U, 3U, -1},
    
    {14U, 0U, -1},
    {15U, 0U, -1},
    {16U, 0U, -1},
    {17U, 0U, -1},
    {18U, 0U, -1},
    {19U, 0U, -1},
    {20U, SW_FUNC_MODE_SP_ONLY, -1},   /* MODE Inhibit/Neutral */
};

static const sw_ctx_descriptor_t sw_sp_ctxt[MAX_SPX_SW] = 
{
    {0U, 3U, -1},
    {1U, 3U, -1},
    {2U, 3U, -1},
    {3U, 3U, -1},
    {4U, 3U, -1},
    {5U, 3U, -1},
    {6U, 3U, -1},
    {7U, 3U, -1},   
    
    {8U, 0U, -1},
    {9U, 0U, -1},
    
    {10U, 0U, -1},
    {11U, 0U, -1},
};

static const sw_callback sg_cb_lp_funcs[MAX_SGX_SW] =
{
    sw_sg0_lp_cb,
    NULL,
    sw_sg2_lp_cb,
    sw_sg3_lp_cb,
    sw_sg4_lp_cb,
    sw_sg5_lp_cb,
    NULL,
	NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
};

static const sw_callback sg_cb_funcs[MAX_SGX_SW] =
{
    sw_sg0_cb,
    NULL,
    sw_sg2_cb,
    sw_sg3_cb,
    sw_sg4_cb,
    sw_sg5_cb,
    NULL,
	NULL,
    NULL,
    NULL,
    NULL,
    sw_sg11_cb,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    sw_sg20_cb,
};

static const sw_callback sp_cb_funcs[MAX_SPX_SW] =
{
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
	NULL,
    NULL,
    NULL,
    NULL,
    NULL,

};

static const sw_callback sp_cb_lp_funcs[MAX_SPX_SW] =
{
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
	NULL,
    NULL,
    NULL,
    sw_sp10_lp_cb,
    NULL,

};

#ifdef USE_FEATURE_S32_PA_MODE

static void cd1030_set_pa_mode_ctx_irq(uint32_t pa_mode)
{
    /* Special case for handling from inside a IRQ */
    cd1030_pa_mode_ctx = pa_mode;
    
    if(pa_mode == PA_MODE_ENTRY_SENTINEL)
    {
        mc_pa_mode_idx = 0U;
        bcm_control(BCM_LEFT_IND_CTRL, BCM_CTRL_STATE_ON);
        bcm_control(BCM_RIGHT_IND_CTRL, BCM_CTRL_STATE_ON);
    }
    else
    {
        mc_pa_mode_idx = 1U;
        bcm_control(BCM_LEFT_IND_CTRL, BCM_CTRL_STATE_OFF);
        bcm_control(BCM_RIGHT_IND_CTRL, BCM_CTRL_STATE_OFF);
    }
}

static void cd1030_set_pa_mode_ctx(uint32_t pa_mode)
{
    int32_t lc = 0;
    
    /* Ensure no context switches occur when updating this variable */
    lc = osif_enter_critical();
    cd1030_pa_mode_ctx = pa_mode;
    (void)osif_exit_critical(lc);
    
    if(pa_mode == PA_MODE_ENTRY_SENTINEL)
    {
        lc = osif_enter_critical();
        mc_pa_mode_idx = 0U;
        (void)osif_exit_critical(lc);
        
        bcm_control(BCM_LEFT_IND_CTRL, BCM_CTRL_STATE_ON);
        bcm_control(BCM_RIGHT_IND_CTRL, BCM_CTRL_STATE_ON);
    }
    else
    {
        lc = osif_enter_critical();
        mc_pa_mode_idx = 1U;
        (void)osif_exit_critical(lc);
        
        bcm_control(BCM_LEFT_IND_CTRL, BCM_CTRL_STATE_OFF);
        bcm_control(BCM_RIGHT_IND_CTRL, BCM_CTRL_STATE_OFF);
    }
}
#endif /* USE_FEATURE_S32_PA_MODE */

void cd1030_clear_sw_info(void)
{
    sw_info = SW_NO_CODE;
}

/**
 * @brief Handler for the back switch short press event
 * 
 * @return None 
 */
static void sw_sg0_cb(void)
{
    /* BACK SWITCH */
    sw_info = BACK_SP_CODE;
}

/**
 * @brief Handler for the back switch long press event
 * 
 * @return None 
 */
static void sw_sg0_lp_cb(void)
{
    /* BACK SWITCH Long Press */
    sw_info = BACK_LP_CODE;  
    
}

static void sw_sg1_cb(void)
{
    if(sgx_sm[1].sw_state == 0)
    {   
        sgx_sm[1].sw_state = 1;
    }
    else 
    {
        sgx_sm[1].sw_state = 0;
    }
}

/**
 * @brief Handler for start button short press event for cycling
 * ride modes.
 *
 * @return None 
 */
static void sw_sg2_cb(void)
{
#ifdef USE_FEATURE_S32_PA_MODE
    /* When in PA mode, cycling of ride modes is not allowed */
    if(cd1030_pa_mode_ctx == PA_MODE_ENTRY_SENTINEL)
    {
        sw_info = SW_NO_CODE;
    }
    else
    {
        mc_mode_cycle();
        sw_info = SS_SP_CODE;
    }
#else
    mc_mode_cycle();
    sw_info = SS_SP_CODE;
#endif /* USE_FEATURE_S32_PA_MODE */
}

/**
 * @brief Handler for start button long press event for putting the
 * motor in forward mode if front break is pressed simultaeneously.
 *
 * @return None 
 */
static void sw_sg2_lp_cb(void)
{
    msdi_status_t msdi_status = MSDI_STATUS_SUCCESS;
    
    sw_info = SS_LP_CODE;
    
    msdi_status = MSDI_ReadSwitchStatus(&drv_config, &sp, &sg); 
    if(msdi_status == MSDI_STATUS_SUCCESS)
    {
        if(sp > 0U)
        {
            msdi_status = MSDI_ReadSwitchStatus(&drv_config, &sp, &sg); 
            if(msdi_status == MSDI_STATUS_SUCCESS)
            {
                /* checking FET state to prevent sequence faults */
                if((SW_FRONT_BREAK_PRESS(sp) || SW_REAR_BREAK_PRESS(sp)) &&
                   (get_dsg_fet_state(0U) == MOSFET_ON_SW_STATE))
                {
                    /* Delay to prevent sequence fault right after motor power on */
#ifdef USE_FEATURE_S32_PA_MODE
                    /* 
                        If we have entered PA mode then ignore this event.
                        Ride mode behaviours are not allowed inside PA mode.
                     */
                    if(cd1030_pa_mode_ctx == PA_MODE_ENTRY_SENTINEL)
                    {
                        sw_info = SW_NO_CODE;
                    }
                    else
                    {
                        osif_time_delay(510U);
                        mc_set_gear(MC_GEAR_POS_FWD);
                    }
#else
                    if(SW_SIDE_STAND_PRESS(sg))
                    {
                        osif_time_delay(510U);
                        mc_set_gear(MC_GEAR_POS_FWD);
                    }
                    else
                    {
                        sw_info = SW_MC_ARM_DISABLED;
                    }
#endif /* USE_FEATURE_S32_PA_MODE */
                }
                else
                {
                    /* Long press action */
                    sw_info = SW_NO_CODE;
                }
            }
            else
            {
                set_status_bit(STAT_VCU_SWIF_ERROR);
            }
        }
        else
        {
            if(SW_START_PRESS(sg))
            {
                sw_info = SS_LP_CODE;
            }
            else
            {
                sw_info = SW_NO_CODE;
            }
        } 
    }
    else
    {
        set_status_bit(STAT_VCU_SWIF_ERROR);
    }    
}

/**
 * @brief Handler for the Enter switch short press event.
 *
 * @return None 
 */
static void sw_sg3_cb(void)
{
#ifdef USE_FEATURE_S32_PA_MODE
    static const uint32_t pa_modes[2] = {MC_PA_MODE_FWD, MC_PA_MODE_REV};
    static const uint32_t pa_swif_codes[2] = {SET_LP_PARK_ASSIST_FWD_CODE, SET_LP_PARK_ASSIST_REV_CODE};

    int32_t mc_ret = 0;
#endif /* USE_FEATURE_S32_PA_MODE */
    
    /* If Left or Right is pressed */
    if(SW_LEFT_PRESS(sg) || SW_RIGHT_PRESS(sg))
    {
        sw_info = SW_NO_CODE;
    }
    else
    {
#ifdef USE_FEATURE_S32_PA_MODE
        if(cd1030_pa_mode_ctx == PA_MODE_ENTRY_SENTINEL)
        {
            /* If PA mode is active */
            mc_ret = mc_set_pa_mode(pa_modes[mc_pa_mode_idx]);
            if(mc_ret == 0)
            {
                sw_info = pa_swif_codes[mc_pa_mode_idx];
                mc_pa_mode_idx = (mc_pa_mode_idx + 1U) % 2U;
            }
            else
            {
                /* Set error bit incase PA mode conditions are not met */
                set_status_bit(STAT_VCU_PA_MODE_ERROR);
                sw_info = SW_NO_CODE;
            } 
        }
        else   
        {
            sw_info = ENTER_SP_CODE;
        }
#else
        sw_info = ENTER_SP_CODE;
#endif /* USE_FEATURE_S32_PA_MODE */
    }
}

/**
 * @brief Handler for the Enter switch long press event.
 * For entry into park assist mode if front break is pressed simultaeneously.
 * @return None 
 */
static void sw_sg3_lp_cb(void)
{
    msdi_status_t msdi_status = MSDI_STATUS_SUCCESS;
    volatile uint32_t mc_pa_evaluation = MC_PA_MODE_NOT_AVAILABLE;
    
    /* Enter / Set  Long Press */
    sw_info = ENTER_LP_CODE;
    
    msdi_status = MSDI_ReadSwitchStatus(&drv_config, &sp, &sg); 
    if(msdi_status == MSDI_STATUS_SUCCESS)
    {
        if(sp > 0U)
        {
            msdi_status = MSDI_ReadSwitchStatus(&drv_config, &sp, &sg); 
            if(msdi_status == MSDI_STATUS_SUCCESS)
            {
                if(SW_FRONT_BREAK_PRESS(sp) || SW_REAR_BREAK_PRESS(sp))
                {
#ifdef USE_FEATURE_S32_PA_MODE
                    /* Enter Park Assist Mode (PA) */
                    if(cd1030_pa_mode_ctx == PA_MODE_EXIT_SENTINEL)
                    {
                        /* Evaluate if all conditions are met to enter PA mode */
                        sw_info = cd1030_eval_pa_mode_entry();
                    }
                    else
                    {
                        /* 
                            Exit PA mode only if conditions defined in mc_eval_pa_mode_entry
                            are satisfied.
                        */
                         mc_pa_evaluation = mc_eval_pa_mode_entry();
                         if(mc_pa_evaluation == MC_PA_MODE_AVAILABLE)
                         {
                             mc_set_pa_mode_off(MC_GEAR_POS_FWD);
                             cd1030_set_pa_mode_ctx(PA_MODE_EXIT_SENTINEL);
                             sw_info = SET_LP_PARK_ASSIST_EXIT_CODE;
                         }
                    }
#endif /* USE_FEATURE_S32_PA_MODE */
                }
            }
            else
            {
                set_status_bit(STAT_VCU_SWIF_ERROR);
            }
        }
    }
    else
    {
        set_status_bit(STAT_VCU_SWIF_ERROR);
    }
}

/**
 * @brief Handler for the Left switch short press event.
 *
 * @return None 
 */
static void sw_sg4_cb(void)
{
    sw_info = LEFT_SP_CODE;
}

/**
 * @brief Handler for the Left switch long press event.
 *
 * @return None 
 */
static void sw_sg4_lp_cb(void)
{
    sw_info = LEFT_LP_CODE;
}

/**
 * @brief Handler for the Right switch short press event.
 *
 * @return None 
 */
static void sw_sg5_cb(void)
{
    sw_info = RIGHT_SP_CODE;
}

/**
 * @brief Handler for the Right switch long press event.
 *
 * @return None 
 */
static void sw_sg5_lp_cb(void)
{
    sw_info = RIGHT_LP_CODE;
}

static void sw_sg6_cb(void)
{
    if(sgx_sm[6].sw_state == 0)
    {
        sgx_sm[6].sw_state = 1;
    }
    else 
    {
        sgx_sm[6].sw_state = 0;
    }
}

static void sw_sg7_cb(void)
{
    if(sgx_sm[7].sw_state == 0)
    {
        sgx_sm[7].sw_state = 1;
    }
    else 
    {
        sgx_sm[7].sw_state = 0;
    }
}

static void sw_sg8_cb(void)
{
    if(sgx_sm[8].sw_state == 0)
    {
        sgx_sm[8].sw_state = 1;
    }
    else 
    {
        sgx_sm[8].sw_state = 0;
    }
}

static void sw_sg9_cb(void)
{
    if(sgx_sm[9].sw_state == 0)
    {
        sgx_sm[9].sw_state = 1;
    }
    else 
    {
        sgx_sm[9].sw_state = 0;
    }
}

static void sw_sg10_cb(void)
{
    if(sgx_sm[10].sw_state == 0)
    {
        sgx_sm[10].sw_state = 1;
    }
    else 
    {
        sgx_sm[10].sw_state = 0;
    }
}

/**
 * @brief Handler for the Hazard switch short press event.
 *
 * @return None 
 */
static void sw_sg11_cb(void)
{
    if(chg_get_connection_state() == 0U)
    {
#ifdef USE_FEATURE_S32_PA_MODE
        if(cd1030_pa_mode_ctx == PA_MODE_EXIT_SENTINEL)
        {
#endif
            if(sgx_sm[11].sw_state == 0)
            {
                sgx_sm[11].sw_state = 1;
                is_hazard_sw_activated = 1U;
                bcm_control(BCM_LEFT_IND_CTRL, BCM_CTRL_STATE_ON);
                bcm_control(BCM_RIGHT_IND_CTRL, BCM_CTRL_STATE_ON);
            }
            else 
            {
                sgx_sm[11].sw_state = 0;
                is_hazard_sw_activated = 0U;
                bcm_control(BCM_LEFT_IND_CTRL, BCM_CTRL_STATE_OFF);
                bcm_control(BCM_RIGHT_IND_CTRL, BCM_CTRL_STATE_OFF);
            }
#ifdef USE_FEATURE_S32_PA_MODE
        }
#endif
    }
    
    sw_info = SW_NO_CODE;
}

static void sw_sg12_cb(void)
{
    if(sgx_sm[12].sw_state == 0)
    {
        sgx_sm[12].sw_state = 1;
    }
    else 
	{
        sgx_sm[12].sw_state = 0;
    }
}

static void sw_sg12_lp_cb(void)
{

    if(sgx_sm[12].sw_state == 0)
    {
        sgx_sm[12].sw_state = 1;
    }
    else 
    {
        sgx_sm[12].sw_state = 0;
    }
}

static void sw_sg13_cb(void)
{
    if(sgx_sm[13].sw_state == 0)
    {
        sgx_sm[13].sw_state = 1;
    }
    else 
    {
        sgx_sm[13].sw_state = 0;
    }
}

static void sw_sg20_cb(void)
{
    __NOP();   
}

static void sw_sp0_cb(void)
{
    if(spx_sm[0].sw_state == 0)
    {
        spx_sm[0].sw_state = 1;
    }
    else 
    {
        spx_sm[0].sw_state = 0;
	}
}

static void sw_sp1_cb(void)
{
    if(spx_sm[1].sw_state == 0)
    {
        spx_sm[1].sw_state = 1;
    }
    else 
    {
        spx_sm[1].sw_state = 0;
	}
}

static void sw_sp2_cb(void)
{

}

static void sw_sp3_cb(void)
{
           
}

static void sw_sp4_cb(void)
{
    
}

static void sw_sp5_cb(void)
{
    if(spx_sm[5].sw_state == 0)
    {
        spx_sm[5].sw_state = 1;
    }
    else 
    {
        spx_sm[5].sw_state = 0;
    }    
}

static void sw_sp6_cb(void)
{
    if(spx_sm[6].sw_state == 0)
    {
        spx_sm[6].sw_state = 1;
    }
    else 
    {
        spx_sm[6].sw_state = 0;
    }    
}

static void sw_sp7_cb(void)
{
    if(spx_sm[7].sw_state == 0)
    {
        spx_sm[7].sw_state = 1;
    }
    else 
    {
        spx_sm[7].sw_state = 0;
    }    
}

static void sw_sp10_lp_cb(void)
{
    if(spx_sm[10].sw_state == 0)
    {
        spx_sm[10].sw_state = 1;
    }
    else 
    {
        spx_sm[10].sw_state = 0;
    } 
}

#ifdef USE_FEATURE_SWIF_CHEAT_CODES
uint32_t cd1030_open_cheatbox(uint32_t cheats)
{
    volatile uint32_t send_to_task = 0U;
    
    msdi_status_t msdi_status = MSDI_STATUS_SUCCESS;
    
    msdi_status = MSDI_ReadSwitchStatus(&drvConfig, &sp, &sg); 
    DEV_ASSERT(msdi_status == STATUS_SUCCESS);
    
    /* Only if front brake is pressed */
    /*                    
                          01234
        Long Press Back - LRLLR - VCU Hard Reset/Debug Screen
        Long Press Back - LRLRR - i.Mx Hard Reset
        Long Press Back - LRLLL - EC25 Hard Reset
    */
    if(sp > 0)
    {
        if(sp & (1U << 5U))
        {
            switch(cheat_state)
            {
                case 0U:
                    cheat_state = (cheats == 4U)?1U:0U; break; /* LEFT */
                case 1U:
                    cheat_state = (cheats == 5U)?2U:0U; break; /* RIGHT */
                case 2U:
                    cheat_state = (cheats == 4U)?3U:0U; break; /* LEFT */
                
                case 3U:
                    if(cheats == 4U)
                    {
                        /* LEFT */
                        cheat_state = 4U;
                    }else if(cheats == 5U)
                    {
                        /* RIGHT */
                        cheat_state = 6U;
                    }
                    else
                    {
                        cheat_state = 0U;
                    }
                    break;
                case 4U:
                    if(cheats == 4U)
                    {
                        /* LEFT */
                        cheat_state = 0U;
                        /* Minimum spec. according to Quectel is 100ms or more */
                        PINS_DRV_SetPins(RESET_MCU_GPIO, (1U << RESET_MCU_PIN));
                        OSIF_TimeDelay(500);
                        PINS_DRV_ClearPins(RESET_MCU_GPIO, (1U << RESET_MCU_PIN));
                    }else if(cheats == 5U)
                    {
                        /* RIGHT */
                        cheat_state = 0U;
#if 0
                        /* Power Cycle VCU */
                        bcm_control(BCM_VCU_CTRL, BCM_CTRL_STATE_OFF);
                        OSIF_TimeDelay(100);
                        bcm_control(BCM_VCU_CTRL, BCM_CTRL_STATE_ON);
#endif
                        /* Send debug screen open message */
                        sw_info = SS_CC_DBG_SCREEN;
                        send_to_task = 1U;
                    }
                    else
                    {
                        cheat_state = 0U;
                    }
                    break;
                case 5U:
                    break;
                case 6U:
                    if(cheats == 5U)
                    {
                        cheat_state = 0U;
                        /* Power Cycle i.Mx */
                        PINS_DRV_SetPins(IMX6_RESET_GPIO, (1U << IMX6_RESET_PIN));
                        OSIF_TimeDelay(100);
                        PINS_DRV_ClearPins(IMX6_RESET_GPIO, (1U << IMX6_RESET_PIN));
                    }
                    else if(cheats == 4U)
                    {
                        
                    }
                    else
                    {
                        cheat_state = 0U;
                    }
                    break;
                case 7U:
                    break;
            }
        }
    }
    
    return send_to_task;
}

#endif /* USE_FEATURE_SWIF_CHEAT_CODES */

#ifdef USE_FEATURE_S32_PA_MODE
static uint8_t cd1030_eval_pa_mode_entry(void)
{
    int32_t mc_ret = 0;
    uint8_t ret = SW_NO_CODE;
    volatile uint32_t mc_pa_evaluation = MC_PA_MODE_NOT_AVAILABLE;

    mc_pa_evaluation = mc_eval_pa_mode_entry();

    if((mc_pa_evaluation == MC_PA_MODE_AVAILABLE) &&
       (!SW_NEUTRAL_PRESS(sg)))
    {
        set_status_bit(STAT_VCU_PA_MODE_ENTRY);
        cd1030_set_pa_mode_ctx(PA_MODE_ENTRY_SENTINEL);
        mc_ret = mc_set_pa_mode(MC_PA_MODE_REV);
        
        if(mc_ret == 0)
        {
            /* The vehicle will be configured for PA mode reverse */
            ret = SET_LP_PARK_ASSIST_ENTRY_CODE;
        }
        else
        {
            /* In case PA entry conditions are not satisfied the cluster will
               show eror notification instead of switching to PA mode screens 
            */
            ret = SW_NO_CODE;
            cd1030_set_pa_mode_ctx(PA_MODE_EXIT_SENTINEL);
            set_status_bit(STAT_VCU_PA_MODE_ERROR);
        }
    }
    else
    {
        ret = SW_NO_CODE;
        cd1030_set_pa_mode_ctx(PA_MODE_EXIT_SENTINEL);
        set_status_bit(STAT_VCU_PA_MODE_ERROR);
    }

    return ret;
}
#endif /* USE_FEATURE_S32_PA_MODE */

/**
 * @brief Handler for the Rear break switch short press event.
 *
 * @return None 
 */
static void cd1030_rear_brake_handler(void)
{
    if(SW_REAR_BREAK_PRESS(sp))
    {
        set_status_bit(STAT_VCU_REAR_BRAKE_PRESS);
    }
    else
    {
        clear_status_bit(STAT_VCU_REAR_BRAKE_PRESS);
    }   
}

/**
 * @brief Handler for the Front break switch short press event.
 *
 * @return None 
 */
static void cd1030_front_brake_handler(void)
{    
    if(SW_FRONT_BREAK_PRESS(sp))
    {
        set_status_bit(STAT_VCU_FRONT_BRAKE_PRESS);
    }
    else
    {
        clear_status_bit(STAT_VCU_FRONT_BRAKE_PRESS);
    }
}

/**
 * @brief Handler for the Side stand switch short press event.
 *
 * @return None 
 */
static void cd1030_check_side_stand(void)
{
    static uint32_t ss_sm = S0;
    volatile bool ss_state = false;
    
    /*
        ss_state = true - side stand retracted/up
        ss_state = false - side stand deployed/down
    */
    ss_state = SW_SIDE_STAND_PRESS(sg);
    
    switch(ss_sm)
    {
        case S0:
            /* Initial state of the side stand after vehicle power on.
               Based on this information transition to S1 or S2.
            */
            if(ss_state)
            {
                /* Side stand retracted */
                clear_status_bit(STAT_VCU_SIDE_STAND_DEPLOYED);
                ss_sm = S1;
            }
            else
            {
                /* Side stand deployed */
                ss_sm = S2;
            }
            break;
            
        case S1:
            /* Do nothing but wait for side stand deployment */
            if(ss_state)
            {
                ss_sm = S1;
            }
            else
            {
                ss_sm = S2;
            }
            break;
            
        case S2:
            /* Side stand deployed. Execute actions. This state must only
               be entered once.
            */
            set_status_bit(STAT_VCU_SIDE_STAND_DEPLOYED);
        
#ifdef USE_FEATURE_S32_PA_MODE
            /* Vehicle to exit PA mode when side stand gets deployed */
            if(cd1030_pa_mode_ctx == PA_MODE_ENTRY_SENTINEL)
            {
                mc_set_pa_mode_off(MC_GEAR_POS_NEUTRAL);
                cd1030_set_pa_mode_ctx(PA_MODE_EXIT_SENTINEL);
            }
            else
            {
                mc_set_gear(MC_GEAR_POS_NEUTRAL);
            }
#else   
            mc_set_gear(MC_GEAR_POS_NEUTRAL);
#endif /* USE_FEATURE_S32_PA_MODE */
            
            ss_sm = S3;
            break;
            
        case S3:
            /* Wait for side stand to be retracted */
            if(!ss_state)
            {
                ss_sm = S3;
            }
            else
            {
                clear_status_bit(STAT_VCU_SIDE_STAND_DEPLOYED);
                ss_sm = S1;
            }
            break;

        default:
            __NOP();
            break;
    }
}

/**
 * @brief Handler for the High beam switch short press event.
 *
 * @return None 
 */
static void cd1030_sp01_high_beam(void)
{
    dba_msg_queue_obj_t dmq;
    static uint32_t high_beam_state = 0U;
    
    switch(high_beam_state)
    {
        case 0U:
            if(SW_HIGH_BEAM_PRESS(sp))
            {
                dmq.msg_id = DBA_SEND_DISPLAY_MSG;
                dmq.data[0] = DBA_DISPLAY_TELL_TALE_CTRL_MSG;
                dmq.data[1] = TT_HIGH_BEAM_INDICATOR;
                dmq.data[2] = DBA_DISPLAY_TELL_TALE_ON;
                (void)osif_msg_queue_send(dba_msg_queue, &dmq, 0U, 1U);
                high_beam_active = 1U;
                high_beam_state = 1U;
                
                /* On high beam activation DRL is expected to turn OFF */
                bcm_control(BCM_DRL_LED_CTRL, BCM_CTRL_STATE_OFF);
            }
            else
            {
                high_beam_state = 0U;
            }

            break;
            
        case 1U:
            if(!SW_HIGH_BEAM_PRESS(sp))
            {
                dmq.msg_id = DBA_SEND_DISPLAY_MSG;
                dmq.data[0] = DBA_DISPLAY_TELL_TALE_CTRL_MSG;
                dmq.data[1] = TT_HIGH_BEAM_INDICATOR;
                dmq.data[2] = DBA_DISPLAY_TELL_TALE_OFF;
                (void)osif_msg_queue_send(dba_msg_queue, &dmq, 0U, 1U); 
                high_beam_active = 0U;
                high_beam_state = 0U;
            }
            else
            {
                high_beam_state = 1U;
            }
            break;
            
        default:
            __NOP();
            break;
    }
}

static void cd1030_sg20_neutral(void)
{
    static uint32_t state = 0U;
    
    switch(state)
    {
        case 0U:
            if(SW_NEUTRAL_PRESS(sg))
            {
#ifdef USE_FEATURE_S32_PA_MODE
                /* When start is toggled to the kill position PA mode must exit */
                if(cd1030_pa_mode_ctx == PA_MODE_ENTRY_SENTINEL)
                {
                    mc_set_pa_mode_off(MC_GEAR_POS_NEUTRAL);
                    cd1030_set_pa_mode_ctx(PA_MODE_EXIT_SENTINEL);
                }
                else
                {
                    mc_set_gear(MC_GEAR_POS_NEUTRAL);
                }
#else
                mc_set_gear(MC_GEAR_POS_NEUTRAL);
#endif
                state = 1U;
            }
            break;
            
        case 1U:
            if(!SW_NEUTRAL_PRESS(sg))
            {
                state = 0U;
            }
            break;
            
        default:
            __NOP();
            break;
    }
}

void cd1030_sem_init(void)
{
    status_t s = STATUS_SUCCESS;
    
    s = osif_sem_create(&drv_swif_sem_t, 0U);
    DEV_ASSERT(s == STATUS_SUCCESS);
	
	INT_SYS_EnableIRQ(PORTC_IRQn);
    INT_SYS_EnableIRQ(PORTA_IRQn); 
}

void cd1030_timer_init(void)
{
    long_press_timer = osif_timer_create(cd1030_lp_timer_handler, osTimerOnce, &lp_timer_args, NULL);
    DEV_ASSERT(long_press_timer != NULL);
}

status_t drv_swif_sem_acquire(uint32_t timeout)
{
    status_t s;
    s = osif_sem_acquire(&drv_swif_sem_t, timeout);
    return s;
}

status_t drv_swif_sem_release(void)
{
    status_t s;
    s = osif_sem_release(&drv_swif_sem_t);
    return s;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : key_sw_irq
 * Description   : checks port A pin status for detecting key insertion and 
 *                 updates key_sw_inserted flag
 *
 * Implements    : key_sw_irq_Activity
 *END**************************************************************************/

#ifdef USE_FEATURE_VCU_ON_DESK
void key_sw_irq(void)
{
    volatile uint32_t pins_porta = 0U;
    bms_msg_queue_obj_t bmsg;
    
    if(PINS_DRV_GetPortIntFlag(KEY_WAKE_SIG_PORT) & (1 << KEY_WAKE_SIG_PIN))
    {   
        pins_porta = PINS_DRV_ReadPins(KEY_WAKE_SIG_GPIO);
        pins_porta |= PINS_DRV_ReadPins(KEY_WAKE_SIG_GPIO);
        pins_porta |= PINS_DRV_ReadPins(KEY_WAKE_SIG_GPIO);
        pins_porta |= PINS_DRV_ReadPins(KEY_WAKE_SIG_GPIO);
        
        /* 
            A logic level of 0 indicates that the KEY is in the ON position 
            A logic level of 1 indicates that the KEY is in the OFF position 
        */
        
        if((pins_porta & (1U << KEY_WAKE_SIG_PIN)) == 0U)
        {
            /* Send bms command to turn FETs OFF */
            bmsg.msg_id = BMS_MSG_ASYNC_FET_ON;
            bmsg.data = 0U;
            (void)osif_msg_queue_send(bms_msg_queue, &bmsg, 0U, 0U);
        }
        else
        {
            /* Send bms command to turn FETs ON */
            bmsg.msg_id = BMS_MSG_ASYNC_FET_OFF;
            bmsg.data = 0U;
            (void)osif_msg_queue_send(bms_msg_queue, &bmsg, 0U, 0U); 
        }
     }

     PINS_ClearPortIntFlag(KEY_WAKE_SIG_PORT, (1U << (KEY_WAKE_SIG_PIN)));
}
#else
void key_sw_irq(void)
{
    volatile uint32_t pins_porta = 0U;
    
    if(PINS_DRV_GetPortIntFlag(KEY_WAKE_SIG_PORT) & (1U << KEY_WAKE_SIG_PIN))
    {   
        pins_porta = PINS_DRV_ReadPins(KEY_WAKE_SIG_GPIO);
        pins_porta |= PINS_DRV_ReadPins(KEY_WAKE_SIG_GPIO);
        pins_porta |= PINS_DRV_ReadPins(KEY_WAKE_SIG_GPIO);
        pins_porta |= PINS_DRV_ReadPins(KEY_WAKE_SIG_GPIO);
        
        /* 
            A logic level of 0 indicates that the KEY is in the ON position 
            A logic level of 1 indicates that the KEY is in the OFF position 
        */
        
        if((pins_porta & (1U << KEY_WAKE_SIG_PIN)) == 0U)
        {
#ifdef USE_FEATURE_S32_PA_MODE
            /* Initialization failsafe */
            cd1030_set_pa_mode_ctx_irq(PA_MODE_EXIT_SENTINEL);
#endif
            key_sw_on = true;
        }
        else
        {
#ifdef USE_FEATURE_S32_PA_MODE
            /* When key is turned off the PA mode must exit */
            mc_set_pa_mode_off_irq(MC_GEAR_POS_NEUTRAL);
            cd1030_set_pa_mode_ctx_irq(PA_MODE_EXIT_SENTINEL);
#endif
            key_sw_on = false;
        }
     }

     PINS_ClearPortIntFlag(KEY_WAKE_SIG_PORT, (1U << (KEY_WAKE_SIG_PIN)));
}
#endif /* USE_FEATURE_VCU_ON_DESK */

void cd1030_enter_LPM(void)
{
    msdi_lpm_cfg_t lpm_cfg;
    
    lpm_cfg.pollRate = MSDI_POLL_RATE_128;
    lpm_cfg.intTmrVal = MSDI_INT_TMR_VAL_394;
    
    (void)MSDI_SetLPMConfig(&drv_config, &lpm_cfg);
}

uint32_t cd1030_is_lp_timer_running(void)
{
    return is_long_press_timer_running;
}

void cd1030_start_lp_timer(void)
{
    (void)osif_timer_start(long_press_timer, MSEC_TO_TICK(LONG_PRESS_TIMER_TIMEOUT)); 
    is_long_press_timer_running = 1U;
}

void cd1030_stop_lp_timer(void)
{
    (void)osif_timer_stop(long_press_timer); 
    is_long_press_timer_running = 0U;    
}

void cd1030_clear_port_irq(void)
{
    PINS_ClearPortIntFlag(SWIF_IRQ_PORT, (1U << (SWIF_IRQ_PIN)));
}

void cd1030_irq_late_init(void)
{
    PINS_DRV_SetPinIntSel(SWIF_IRQ_PORT, SWIF_IRQ_PIN, PORT_INT_FALLING_EDGE);
}

void cd1030_read_error(void)
{
    msdi_faults_t f;
    msdi_status_t ret = MSDI_STATUS_SUCCESS;
    static uint32_t ignore_hash_once_flag = 0U;
    static uint32_t ignore_por_once_flag = 0U;

    ret = MSDI_GetFaultStatus(&drv_config, &f);
    if(ret == MSDI_STATUS_SUCCESS)
    {
        if(f.hashFault == true)
        {
            if(ignore_hash_once_flag == 0)
            {
                ignore_hash_once_flag = 1U;
            }
            else
            {
                msdi_fault_state |= MSDI_FLT_STAT_HASH_FLT_MASK;   
            }
        }
        else if(f.intBWake == true)
        {
            msdi_fault_state |= MSDI_FLT_STAT_WAKE_WAKE_MASK;
        }
        else if(f.ot == true)
        {
            msdi_fault_state |= MSDI_FLT_STAT_OV_MASK;
        }
        else if(f.ov == true)
        {
            msdi_fault_state |= MSDI_FLT_STAT_OV_MASK;
        }
        else if(f.por == true)
        {
            if(ignore_por_once_flag == 0U)
            {
                ignore_por_once_flag = 1U;
            }
            else
            {
                msdi_fault_state |= MSDI_FLT_STAT_POR_MASK;
            }
        }
        else if(f.spiError == true)
        {
            msdi_fault_state |= MSDI_FLT_STAT_SPI_ERROR_MASK;
        }
        else if(f.tempFlag == true)
        {
            msdi_fault_state |= MSDI_FLT_STAT_TEMP_FLG_MASK;
        }
        else if(f.uv == true)
        {
            msdi_fault_state |= MSDI_FLT_STAT_UV_MASK;
        }
        else if(f.wakeBWake == true)
        {
            msdi_fault_state |= MSDI_FLT_STAT_WAKE_WAKE_MASK;
        }
        else
        {
            __NOP();
        }
        
        if(msdi_fault_state > 0U)
        {
            set_status_bit(STAT_VCU_SWIF_INTERNAL_ERROR);
        }
    }
    else
    {
        set_status_bit(STAT_VCU_SWIF_ERROR);
    }
}

void cd1030_get_internal_faults(uint16_t *faults)
{
    if(faults != NULL)
    {
        *faults = msdi_fault_state;
    }
}

void cd1030_inhibit_second_irq(uint32_t state)
{
    inhibit_second_irq = state;
}

uint32_t cd1030_is_high_beam_active(void)
{
    return high_beam_active;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : cd1030_sp_timer_handler
 * Description   : it's timer callback function which gets called after osTimerStart 
 * which checks if switch is still pressed even after 300 msec, if it is, it detects as short press
 *
 * Implements    : cd1030_sp_timer_handler_Activity
 *END**************************************************************************/

void cd1030_lp_timer_handler(void *arg)
{
    UNUSED_PARAM(arg);
    
    is_long_press_timer_running = 0U; 
    
#ifdef USE_FEATURE_SWIF_IRQ
    (void)drv_swif_sem_release();
#endif    
}

/*FUNCTION**********************************************************************
 *
 * Function Name : cd1030_irq
 * Description   : cheks port B pin status for detecting switch/es press detection 
 *                 and updates switchStatusChanged flag
 *
 * Implements    : cd1030_irq_Activity
 *END**************************************************************************/

void cd1030_irq(void)
{
    (void)drv_swif_sem_release();  
    
    PINS_ClearPortIntFlag(SWIF_IRQ_PORT, (1U << (SWIF_IRQ_PIN)));
}

void cd1030_initiate_swif_read(void)
{
    msdi_status_t msdi_status = MSDI_STATUS_SUCCESS;
    msdi_status = MSDI_ReadSwitchStatus(&drv_config, &sp, &sg);
    
    if(msdi_status != MSDI_STATUS_SUCCESS)
    {
        set_status_bit(STAT_VCU_SWIF_ERROR);
    }
}

void cd1030_set_irq_ctrl(uint32_t sw_id, uint32_t sw_if)
{
    msdi_status_t msdi_status = MSDI_STATUS_SUCCESS;
    uint32_t sgirq = 0U;
    uint32_t spirq = 0U;
    uint32_t temp = 0U;

    if(sw_if == SWIF_SW_INTERFACE_SG)
    {
        DEV_ASSERT(sw_id < MSDI_SG_CNT_CD1030);
        
        /* SGX */
        temp = (1U << sw_id);
        sgirq = sgirq | temp;
        msdi_status = MSDI_SetIntEnable(&drv_config, spirq, sgirq);
    } 
    
    if(sw_if == SWIF_SW_INTERFACE_SP)
    {
        DEV_ASSERT(sw_id < MSDI_SP_CNT_CD1030);
        
        /* SPX */
        temp = (1U << sw_id);
        spirq = spirq | temp;
        msdi_status = MSDI_SetIntEnable(&drv_config, spirq, sgirq);
    } 
}

void cd1030_enable_all_irqs(void)
{
    msdi_status_t msdi_status = MSDI_STATUS_SUCCESS;
    uint32_t sgirq = 0x000FFFFFU;
    uint32_t spirq = 0x00000000U;
    
    msdi_status = MSDI_SetIntEnable(&drv_config, spirq, sgirq);
}

uint32_t cd1030_eval_switch(uint32_t *sw_idx, uint32_t *sw_if)
{
    uint32_t lpsp_ret = 0U;
    
    DEV_ASSERT(sw_if != NULL);
    DEV_ASSERT(sw_idx != NULL);
    
    if(sg > 0U)
    {
        for(uint32_t i = 0U; i < MSDI_SG_CNT_CD1030; i++)
        {
            /* Ignore SG8 - Side stand */
            if(i == 8U)
            {
                continue;                
            }
            
            if((sg & READ_SW_STAT_MASK(i)) == READ_SW_STAT_CLOSED(i)) 
            {
                lpsp_ret = sw_sg_ctxt[i].function;
                *sw_idx = i;
                *sw_if = SWIF_SW_INTERFACE_SG;
                break;
            }
        }
    }   
    else if(sp > 0U)
    {
        for(uint32_t i = 0U; i < MSDI_SP_CNT_CD1030; i++)
        {
            /* Ignore Low-beam status, high beam status, number plate LED status, DRL status */
            /* SP0, SP7, SP11 */
            if((i == 0U) || (i == 1U) || (i == 7U) || (i == 11U))
            {
                continue;
            }
            
            if((sp & READ_SW_STAT_MASK(i)) == READ_SW_STAT_CLOSED(i)) 
            {
                lpsp_ret = sw_sp_ctxt[i].function;
                *sw_idx = i;
                *sw_if = SWIF_SW_INTERFACE_SP;
                break;
            }
        }   
    }
    else
    {
        __NOP();
    }
    
    return lpsp_ret;    
}

uint32_t cd1030_exec_cb(uint32_t sw_id, uint32_t interface, uint32_t cb_type)
{
    if(interface == SWIF_SW_INTERFACE_SG)
    {
        DEV_ASSERT(sw_id < MSDI_SG_CNT_CD1030);
        
        /* SGX */
        if(cb_type == SWIF_CB_TYPE_SP)
        {
            if(sg_cb_funcs[sw_id] != NULL)
            {
                sg_cb_funcs[sw_id]();
            }
        }
        else if(cb_type == SWIF_CB_TYPE_LP)
        {
            if(sg_cb_lp_funcs[sw_id] != NULL)
            {
                sg_cb_lp_funcs[sw_id]();
            }
        }
        else
        {
            __NOP();
        }
    }
    else if(interface == SWIF_SW_INTERFACE_SP)
    {
        DEV_ASSERT(sw_id < MSDI_SP_CNT_CD1030);
        
        /* SPX */
        if(cb_type == SWIF_CB_TYPE_SP)
        {
            if(sp_cb_funcs[sw_id] != NULL)
            {
                sp_cb_funcs[sw_id]();
            }
        }
        else if(cb_type == SWIF_CB_TYPE_LP)
        {
            if(sp_cb_lp_funcs[sw_id] != NULL)
            {
                sp_cb_lp_funcs[sw_id]();
            }
        }
        else
        {
            __NOP();
        }
    }
    else
    {
        __NOP();
    }
    
    return sw_info;
}

void cd1030_poll_switches(void)
{
    msdi_status_t s = MSDI_STATUS_SUCCESS;
    uint32_t ind_r = 0U;
    uint32_t ind_l = 0U;
    
    INT_SYS_DisableIRQ(PORTC_IRQn);
    s = MSDI_ReadSwitchStatus(&drv_config, &sp, &sg);
    INT_SYS_EnableIRQ(PORTC_IRQn);
    
    if(s == MSDI_STATUS_SUCCESS)
    {
        /* Check if rear brake is pressed */
        cd1030_rear_brake_handler();
        cd1030_front_brake_handler();
        cd1030_check_side_stand();
        
        /* SG20, put vehicle in neutral */
        cd1030_sg20_neutral();
        
        /* High Beam control */
        if(chg_get_connection_state() == 0U)
        {
            cd1030_sp01_high_beam();
        }
    }
    else
    {
        set_status_bit(STAT_VCU_SWIF_ERROR);
    }
    
    /* Indicator workaround */
    if(chg_get_connection_state() == 0U)
    {
        if((is_hazard_sw_activated == 0U) && (cd1030_pa_mode_ctx == PA_MODE_EXIT_SENTINEL))
        {
            ind_r = PINS_DRV_ReadPins(RIGHT_IND_IO_GPIO);
            if((ind_r & (1U << RIGHT_IND_IO_PIN)) > 0U)
            {
                bcm_control(BCM_RIGHT_IND_CTRL, BCM_CTRL_STATE_ON);
            }
            else
            {
                bcm_control(BCM_RIGHT_IND_CTRL, BCM_CTRL_STATE_OFF);
            }
            
            ind_l = PINS_DRV_ReadPins(LEFT_IND_IO_GPIO);
            if((ind_l & (1U << LEFT_IND_IO_PIN)) > 0U)
            {
                bcm_control(BCM_LEFT_IND_CTRL, BCM_CTRL_STATE_ON);
            }
            else
            {
                bcm_control(BCM_LEFT_IND_CTRL, BCM_CTRL_STATE_OFF);
            }
        }
    }
}

void cd1030_read_swif_status(uint8_t *sw_inf)
{
    uint32_t i = 0U;

    if(MSDI_ReadSwitchStatus(&drv_config, &sp, &sg) == MSDI_STATUS_SUCCESS)
    {
        /* Check which switch is open/closed */
        if((sp != sp_old) || (sg != sg_old))
        {
            for(i = 0U; i < MSDI_SP_CNT_CD1030; i++) 
            {
                if((sp ^ sp_old) & READ_SW_STAT_MASK(i)) 
                {
                    /* SPI state changed. */
                    if((sp & READ_SW_STAT_MASK(i)) == READ_SW_STAT_CLOSED(i))
                    {
                        /* Press */
                        if((sw_sp_ctxt[i].function == SW_FUNC_MODE_LP_ONLY) || 
                           (sw_sp_ctxt[i].function == SW_FUNC_MODE_SP_LP))
                        {
                            cd1030_start_lp_timer();
                        }
                    }
                    else
                    {
                        /* Release */
                        if(sp_cb_funcs[i] != NULL)
                        {
                            sp_cb_funcs[i]();
                            break;
                        }
                    }
                }
            }
            
            for(i = 0U; i < MSDI_SG_CNT_CD1030; i++) 
            {
                if((sg ^ sg_old) & READ_SW_STAT_MASK(i)) 
                {
                    /* SGi state changed. */
                    if((sg & READ_SW_STAT_MASK(i)) == READ_SW_STAT_CLOSED(i))
                    {
                        /* Press */
                        if((sw_sg_ctxt[i].function == SW_FUNC_MODE_LP_ONLY) || 
                           (sw_sg_ctxt[i].function == SW_FUNC_MODE_SP_LP))
                        {
                            cd1030_start_lp_timer();
                        }
                    } 
                    else 
                    {
                        /* Release */
                        if(sw_sg_ctxt[i].function == SW_FUNC_MODE_SP_ONLY)
                        {
                            *sw_inf = (uint8_t)cd1030_exec_cb(i, SWIF_SW_INTERFACE_SG, SWIF_CB_TYPE_SP);
                        }
                        else if(sw_sg_ctxt[i].function == SW_FUNC_MODE_LP_ONLY)
                        {
                            if(cd1030_is_lp_timer_running() == 1U)
                            {
                                __NOP();
                            }
                            else
                            {
                                *sw_inf = (uint8_t)cd1030_exec_cb(i, SWIF_SW_INTERFACE_SG, SWIF_CB_TYPE_LP);
                            }
                        }
                        else if(sw_sg_ctxt[i].function == SW_FUNC_MODE_SP_LP)
                        {
                            if(cd1030_is_lp_timer_running() == 1U)
                            {
                                cd1030_stop_lp_timer();
                                *sw_inf = (uint8_t)cd1030_exec_cb(i, SWIF_SW_INTERFACE_SG, SWIF_CB_TYPE_SP);
                            }
                            else
                            {
                                *sw_inf = (uint8_t)cd1030_exec_cb(i, SWIF_SW_INTERFACE_SG, SWIF_CB_TYPE_LP);
                            }
                        }
                    }
                }
            }
            
            sp_old = sp;
            sg_old = sg;
        }
    }
    else
    {
        set_status_bit(STAT_VCU_SWIF_ERROR);
    }
}
