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
 
#include "fw_common.h"
#include "bcm.h"

/* The sequence of I/Os below must be in the same sequence as in bcm_controls_e */
static const bcm_io_t bcm_io_map[BCM_MAX_CTRLS] = {
    {HIGH_BEAM_GPIO,        HIGH_BEAM_PIN,          BCM_CTRL_ACTIVE_HIGH},
    {LOW_BEAM_GPIO,         LOW_BEAM_PIN,           BCM_CTRL_ACTIVE_HIGH},
    {FWD_EN_GPIO,           FWD_EN_PIN,             BCM_CTRL_ACTIVE_HIGH},
    {REV_EN_GPIO,           REV_EN_PIN,             BCM_CTRL_ACTIVE_HIGH},
    {HORN_VCC_EN_GPIO,      HORN_VCC_EN_PIN,        BCM_CTRL_ACTIVE_HIGH},
    {AUX_EN_GPIO,           AUX_EN_PIN,             BCM_CTRL_ACTIVE_HIGH},
    {DRL_LED_EN_GPIO,       DRL_LED_EN_PIN,         BCM_CTRL_ACTIVE_HIGH},
    {NBR_PLT_LED_EN_GPIO,   NBR_PLT_LED_EN_PIN,     BCM_CTRL_ACTIVE_HIGH},
    {TAIL_LAMP_EN_GPIO,     TAIL_LAMP_EN_PIN,       BCM_CTRL_ACTIVE_HIGH},
    {REAR_BRAKE_LED_EN_GPIO,REAR_BRAKE_LED_EN_PIN,  BCM_CTRL_ACTIVE_HIGH},
    {HORN_EN_GPIO,          HORN_EN_PIN,            BCM_CTRL_ACTIVE_HIGH},
    {HEAD_LAMP_12V_EN_GPIO, HEAD_LAMP_12V_EN_PIN,   BCM_CTRL_ACTIVE_HIGH},
    {RIGHT_IND_GPIO,        RIGHT_IND_PIN,          BCM_CTRL_ACTIVE_HIGH},
    {LEFT_IND_GPIO,         LEFT_IND_PIN,           BCM_CTRL_ACTIVE_HIGH},
    
    /* ABS Power */
    {ABS_EN_GPIO,           ABS_EN_PIN,             BCM_CTRL_ACTIVE_HIGH}, 
    
    /* ABS Function */
    {ABS_OFF_MC_GPIO,       ABS_OFF_MC_PIN,         BCM_CTRL_ACTIVE_HIGH}, 
    {ON_OFF_MCU_GPIO,       ON_OFF_MCU_PIN,         BCM_CTRL_ACTIVE_HIGH},
    {VDD_MCU_EN_GPIO,       VDD_MCU_EN_PIN,         BCM_CTRL_ACTIVE_HIGH},    
    {MOTOR_CON_ON_OFF_GPIO, MOTOR_CON_ON_OFF_PIN,   BCM_CTRL_ACTIVE_HIGH},  
    {IO_EN_GPIO,            IO_EN_PIN,              BCM_CTRL_ACTIVE_HIGH} 
};

void bcm_control(bcm_controls_e bcm_module, uint32_t state)
{
    DEV_ASSERT(bcm_module < BCM_MAX_CTRLS);
    
    if(state == BCM_CTRL_STATE_ON)
    {
        /* Some signals need to be Low (Active Low) to turn ON. Hence the logic member */
        if(bcm_io_map[bcm_module].logic == BCM_CTRL_ACTIVE_HIGH)
        {
            PINS_DRV_SetPins(bcm_io_map[bcm_module].base, 1U << bcm_io_map[bcm_module].pin);
        }
        else
        {
            PINS_DRV_ClearPins(bcm_io_map[bcm_module].base, 1U << bcm_io_map[bcm_module].pin);
        }
    }
    else if(state == BCM_CTRL_STATE_OFF)
    {
        if(bcm_io_map[bcm_module].logic == BCM_CTRL_ACTIVE_HIGH)
        {
            PINS_DRV_ClearPins(bcm_io_map[bcm_module].base, 1U << bcm_io_map[bcm_module].pin);
        }
        else
        {
            PINS_DRV_SetPins(bcm_io_map[bcm_module].base, 1U << bcm_io_map[bcm_module].pin);
        }
    }
    else
    {
        DEV_ASSERT(false);
        __NOP();
    }
}
