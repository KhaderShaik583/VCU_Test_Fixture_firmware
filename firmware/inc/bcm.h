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
 
#ifndef BCM_H
#define BCM_H

#include <stdint.h>

#include "pins_driver.h"

#define BCM_CTRL_STATE_ON       (1U)
#define BCM_CTRL_STATE_OFF      (0U)

#define BCM_CTRL_ACTIVE_HIGH    (1U)
#define BCM_CTRL_ACTIVE_LOW     (0U)

typedef enum
{
    BCM_HIGH_BEAM_CTRL = 0U,
    BCM_LOW_BEAM_CTRL,
    BCM_FWD_CTRL,
    BCM_REV_CTRL,
    BCM_HORN_VCC_CTRL,
    BCM_AUX_CTRL,
    BCM_DRL_LED_CTRL,
    BCM_NBR_PLATE_CTRL,
    BCM_TAIL_LAMP_CTRL,
    BCM_REAR_BRAKE_LED_CTRL,
    BCM_HORN_EN_CTRL,
    BCM_HDL_12V_EN_CTRL,
    BCM_RIGHT_IND_CTRL,
    BCM_LEFT_IND_CTRL,
    BCM_ABS_EN_CTRL,
    BCM_ABS_OFF_MC_CTRL,
    BCM_MCU_PWR_CTRL,
    BCM_MCU_EN_CTRL,
    BCM_MOTOR_PWR_CTRL,
    BCM_CHARGER_LAMP_CTRL,
    
    BCM_MAX_CTRLS
}bcm_controls_e;

typedef struct
{
    GPIO_Type *base;
    uint32_t pin;
    uint32_t logic;
}bcm_io_t;

void bcm_control(bcm_controls_e bcm_module, uint32_t state);

#endif /* BCM_H	*/
