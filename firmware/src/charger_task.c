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
 
#include "charger_task.h"
#include "osif.h"
#include "dba_task.h"
#include "can_messenger_rx.h"
#include "mc_task.h"
#include "pins_driver.h"
#include "bcm.h"
#include "bms_task.h"
#include "odometer_task.h"
#include "cd1030.h"

#ifdef USE_FEATURE_RTX_VIEWER
__attribute__((section(".bss.os.thread.cb")))
thread_tcb_t chg_thread_tcb;
#else
static thread_tcb_t chg_logger_thread_tcb;
#endif /* USE_FEATURE_RTX_VIEWER */

static thread_id_t thread_chg;
osif_msg_queue_id_t chg_msg_queue; 

__attribute__((section("ARM_LIB_STACK")))
static uint64_t chg_thread_stk[CHG_TASK_STACK_SIZE];

static charger_ctx_t charger_context;
static volatile uint32_t charger_next_state = CHARGER_SM_WAIT_TASK_START_NTF;
static volatile uint32_t charger_prev_state = CHARGER_SM_WAIT_TASK_START_NTF;

static const thread_attrs_t chg_controller_attr = {
    CHG_TASK_NAME,
    osThreadDetached,
    &chg_thread_tcb,
    sizeof(chg_thread_tcb),
    &chg_thread_stk[0],
    CHG_TASK_STACK_SIZE * sizeof(uint64_t),
    osPriorityNormal3,
    0U,
    0U    
};

static uint16_t battery_voltage = 0U;
static uint16_t battery_current = 0U;
static uint16_t battery_internal_resistance = 0U;
static volatile float_t battery_voltage_f = 0.0f;
static volatile float_t battery_current_f = 0.0f;
static volatile float_t battery_max_cell_t = 0.0f;

static uint32_t c20_sentinel = 0U;
static uint32_t charger_status_ored = 0U;

static void process_charger_status(void)
{
    __NOP();
}

static uint32_t chg_get_key_state(void)
{
    volatile uint32_t key_sw = 0U;
    volatile uint32_t key_sw_st = 0U;

    key_sw = PINS_DRV_ReadPins(KEY_WAKE_SIG_GPIO);
    key_sw |= PINS_DRV_ReadPins(KEY_WAKE_SIG_GPIO);
    key_sw |= PINS_DRV_ReadPins(KEY_WAKE_SIG_GPIO);
    key_sw |= PINS_DRV_ReadPins(KEY_WAKE_SIG_GPIO);
    
    if((key_sw & (1U << KEY_WAKE_SIG_PIN)) == 0U)
    {
        /* Key ON */
        key_sw_st = 1U;
    }
    else
    {
        /* Key OFF */
        key_sw_st = 0U;
    }

    return key_sw_st;    
}

static void chg_send_bms_mode_ntf(uint8_t mode)
{
    bms_msg_queue_obj_t bmsg;
    
    bmsg.msg_id = BMS_MSG_PWR_MODE;
    bmsg.data = mode;
    
    (void)osif_msg_queue_send(bms_msg_queue, &bmsg, 0U, 10U);
}

static void process_CHG_MQ_MSG_ANALOG_MEAS(uint8_t buffer[])
{
    volatile uint8_t sub_id = 0U;
    
    sub_id = buffer[0];
    
    if(sub_id == 0xD7U)
    {
        charger_context.chg_bandgap_volts   = (float_t)((int16_t)(buffer[1] | (buffer[2] << 8U)) / 100.0f);
        charger_context.psfb_fet_temp       = (float_t)((int16_t)(buffer[3] | (buffer[4] << 8U)) / 100.0f);
        charger_context.transformer_temp    = (float_t)((int16_t)(buffer[5] | (buffer[6] << 8U)) / 100.0f);
    }
    else if(sub_id == 0xE7U)
    {
        charger_context.pfc_fet_temp    = (float_t)((int16_t)(buffer[1] | (buffer[2] << 8U)) / 100.0f);
        charger_context.vofb_volts      = (float_t)((int16_t)(buffer[3] | (buffer[4] << 8U)) / 100.0f);
        charger_context.iofb_volts      = (float_t)((int16_t)(buffer[5] | (buffer[6] << 8U)) / 100.0f);
    }
    else
    {
        __NOP();
    }
}

static void charger_queue_process(void)
{
    dba_msg_queue_obj_t dmq;
    chg_msg_queue_obj_t cmq;
    status_t qstatus = STATUS_SUCCESS;
    
    qstatus = osif_msg_queue_recv(chg_msg_queue, &cmq, NULL, 500U); 
    if(qstatus == osOK)   
    {
        switch(cmq.msg_id)
        {
            case CHG_MQ_MSG_FAULT_INFO:
                charger_context.charger_status = cmq.data[0] | ((uint32_t)cmq.data[1] << 8U) | ((uint32_t)cmq.data[2] << 16U) | ((uint32_t)cmq.data[3] << 24U);
                charger_status_ored |= charger_context.charger_status;
            
                process_charger_status();
            
                if((charger_context.charger_status > 1U))
                {                    
                    charger_next_state = CHARGER_SM_CHECK_FAULTS;
                }
                
                (void)osif_timer_stop(charger_context.charger_comm_timer);
                (void)osif_timer_start(charger_context.charger_comm_timer, MSEC_TO_TICK(CHARGER_TIMEOUT_PERIOD_MS));
                break;

            case CHG_MQ_MSG_CHARGER_CTRL:
                dmq.msg_id = DBA_SEND_CHG_MSG;
                dmq.data[0] = CHG_MSG_CHARGER_ON_OFF_CTRL & 0x00FFU;
                dmq.data[1] = (CHG_MSG_CHARGER_ON_OFF_CTRL >> 8U) & 0x00FFU;
                dmq.data[2] = cmq.data[0];
            
                (void)osif_msg_queue_send(dba_msg_queue, &dmq, 0U, 10U);
                break;
            
            case CHG_MQ_MSG_ANALOG_MEAS:
                process_CHG_MQ_MSG_ANALOG_MEAS(cmq.data);
                break;
            
            default:
                __NOP();
                break;
        }
    }    
}

static void charger_sm(void)
{
    dba_msg_queue_obj_t dmq;
    dba_msg_queue_obj_t dmq_msg_1;
    dba_msg_queue_obj_t dmq_msg_2;
    dba_msg_queue_obj_t dmq_msg_3;
    dba_msg_queue_obj_t dmq_msg_4;
    dba_msg_queue_obj_t dmq_msg_5;
    
    volatile uint8_t design_capacity = 0;
    uint32_t charger_task_start_flag = 0U;
    int32_t lc = 0;
    
    switch(charger_next_state)
    {
        case CHARGER_SM_WAIT_TASK_START_NTF:
            charger_task_start_flag = osif_thread_wait_on_flag(CHG_TASK_START_FLAG, OSIF_WAIT_ANY_FLAG | OSIF_FLAGS_NO_CLEAR, OSIF_WAIT_FOREVER);
            if((charger_task_start_flag & CHG_TASK_START_FLAG) == CHG_TASK_START_FLAG)
            {
                (void)osif_thread_clear_flag(CHG_TASK_START_FLAG);
                charger_next_state = CHARGER_SM_WAIT_CHARGER_BOOT_NTF;
            }
            else
            {
                charger_next_state = CHARGER_SM_WAIT_TASK_START_NTF;
            }
            
            charger_prev_state = CHARGER_SM_WAIT_TASK_START_NTF;
            break;
        
        case CHARGER_SM_WAIT_CHARGER_BOOT_NTF:
            
            (void)osif_timer_start(charger_context.charger_comm_timer, MSEC_TO_TICK(CHARGER_TIMEOUT_PERIOD_MS));
            bcm_control(BCM_ABS_EN_CTRL, BCM_CTRL_STATE_OFF);
            
            lc = osif_enter_critical();
            charger_context.charger_connection_state = 1U;
            (void)osif_exit_critical(lc);
        
            /* Double tap, turn the motor off */
            mc_set_gear(MC_GEAR_POS_NEUT_OFF);
        
            /* Double tap, lights out */
            bcm_control(BCM_ABS_EN_CTRL, BCM_CTRL_STATE_OFF);
            bcm_control(BCM_DRL_LED_CTRL, BCM_CTRL_STATE_OFF);
            bcm_control(BCM_LOW_BEAM_CTRL, BCM_CTRL_STATE_OFF);
            bcm_control(BCM_HDL_12V_EN_CTRL, BCM_CTRL_STATE_OFF);
            bcm_control(BCM_HORN_VCC_CTRL, BCM_CTRL_STATE_OFF);
            bcm_control(BCM_CHARGER_LAMP_CTRL, BCM_CTRL_STATE_ON);
            bcm_control(BCM_TAIL_LAMP_CTRL, BCM_CTRL_STATE_OFF);
            bcm_control(BCM_RIGHT_IND_CTRL, BCM_CTRL_STATE_OFF);
            bcm_control(BCM_LEFT_IND_CTRL, BCM_CTRL_STATE_OFF);
        
            charger_next_state = CHARGER_SM_SEND_NTF;
            charger_prev_state = CHARGER_SM_WAIT_CHARGER_BOOT_NTF;
            break;
        
        case CHARGER_SM_SEND_NTF:
            mc_set_gear(MC_GEAR_POS_NEUT_OFF);
        
            /* Send wake notification confirmation */
            dmq.msg_id = DBA_SEND_CHG_MSG;
            dmq.data[0] = CHG_MSG_BOOT_NTF_RSP_ID & 0x00FFU;
            dmq.data[1] = (CHG_MSG_BOOT_NTF_RSP_ID >> 8U) & 0x00FFU;
        
            dmq.data[2] = 0x2A;
            dmq.data[3] = 0xf4;
            dmq.data[4] = 0x42;
            dmq.data[5] = 0xF8;
            dmq.data[6] = 0U;
            dmq.data[7] = 0U;
        
            (void)osif_msg_queue_send(dba_msg_queue, &dmq, 0U, 10U);
                       
            charger_next_state = CHARGER_SM_SEND_CHARGE_CURRENT;
            charger_prev_state = CHARGER_SM_SEND_NTF;
            break;
        
        case CHARGER_SM_SEND_CHARGE_CURRENT:
            /* Set charge current */
            dmq.msg_id = DBA_SEND_CHG_MSG;
            dmq.data[0] = CHG_MSG_CHG_SET_IOUT_ID & 0x00FFU;
            dmq.data[1] = (CHG_MSG_CHG_SET_IOUT_ID >> 8U) & 0x00FFU;
        
            /* 17 A charging - Scaling factor 10 */
            dmq.data[2] = 0xAAU;
            dmq.data[3] = 0U;
            dmq.data[4] = 0U;
            dmq.data[5] = 0U;
            dmq.data[6] = 0U;
            dmq.data[7] = 0U;
        
            (void)osif_msg_queue_send(dba_msg_queue, &dmq, 0U, 10U);
            charger_next_state = CHARGER_SM_CHECK_KEY_STATE;
        
            charger_prev_state = CHARGER_SM_SEND_CHARGE_CURRENT;
            break;
        
        case CHARGER_SM_CHECK_KEY_STATE:
            if(chg_get_key_state() == 1U)
            {
                charger_next_state = CHARGER_SM_WAIT_KEY_OFF;
            }
            else
            {
                charger_next_state = CHARGER_SM_WAIT_BATT_VOLTAGE;
            }
            
            charger_prev_state = CHARGER_SM_CHECK_KEY_STATE;
            break;
            
        case CHARGER_SM_WAIT_KEY_OFF:
            if(chg_get_key_state() == 0U)
            {
                charger_next_state = CHARGER_SM_WAIT_BATT_VOLTAGE;
            }
            else
            {
                charger_next_state = CHARGER_SM_WAIT_KEY_OFF;
            }
            
            charger_prev_state = CHARGER_SM_WAIT_KEY_OFF;
            break;
        
        case CHARGER_SM_START_CHARGING:
            odo_trip_reset(TRIP_NUM_4);
        
            /* Turn ON the charger output */
            dmq_msg_1.msg_id = DBA_SEND_CHG_MSG;
            dmq_msg_1.data[0] = CHG_MSG_CHARGER_ON_OFF_CTRL & 0x00FFU;
            dmq_msg_1.data[1] = (CHG_MSG_CHARGER_ON_OFF_CTRL >> 8U) & 0x00FFU;
            dmq_msg_1.data[2] = CHARGER_CTRL_ON;
        
            (void)osif_msg_queue_send(dba_msg_queue, &dmq_msg_1, 0U, 10U);
        
            /* Notify DBA to start display timeout timer */
            dmq_msg_2.msg_id = DBA_START_DISPLAY_SLEEP_TIMER_MSG;
            (void)osif_msg_queue_send(dba_msg_queue, &dmq_msg_2, 0U, 10U);
        
            /* Notify UI to transition to charging screen */
            dmq_msg_3.msg_id = DBA_SEND_UI_CHG_ON_NTF;
            (void)osif_msg_queue_send(dba_msg_queue, &dmq_msg_3, 0U, 10U);
        
            /* Charging tell tale blink while charging */
            dmq.msg_id = DBA_SEND_DISPLAY_MSG;
            dmq.data[0] = DBA_DISPLAY_TELL_TALE_BLINK_CTRL_MSG;
            dmq.data[1] = TT_CHARGING_INDICATOR;
            dmq.data[2] = 250U;
        
            (void)osif_msg_queue_send(dba_msg_queue, &dmq, 0U, 10U);
            
            /* Turn OFF ABS warning lamp telltale */
            dmq_msg_4.msg_id = DBA_SEND_DISPLAY_MSG;
            dmq_msg_4.data[0] = DBA_DISPLAY_TELL_TALE_CTRL_MSG;
            dmq_msg_4.data[1] = (uint8_t)TT_ABS_LAMP_INDICATOR;
            dmq_msg_4.data[2] = DBA_DISPLAY_TELL_TALE_OFF;
        
            (void)osif_msg_queue_send(dba_msg_queue, &dmq_msg_4, 0U, 10U);
            
            /* Turn OFF high beam tell tale */
            dmq_msg_5.msg_id = DBA_SEND_DISPLAY_MSG;
            dmq_msg_5.data[0] = DBA_DISPLAY_TELL_TALE_CTRL_MSG;
            dmq_msg_5.data[1] = (uint8_t)TT_HIGH_BEAM_INDICATOR;
            dmq_msg_5.data[2] = DBA_DISPLAY_TELL_TALE_OFF;
        
            (void)osif_msg_queue_send(dba_msg_queue, &dmq_msg_5, 0U, 10U);
        
            set_status_bit(STAT_VCU_CHARGING_IN_PROGRESS);
            clear_status_bit(STAT_VCU_CHARGING_ERROR);
            charger_next_state = CHARGER_SM_SEND_BATT_VI;
            
            bms_get_chg_ah(0U, &charger_context.coulombs_before_charge);
            
            charger_prev_state = CHARGER_SM_START_CHARGING;
            break;
        
        case CHARGER_SM_WAIT_BATT_VOLTAGE:
            bms_get_voltage_cm((float_t *)&battery_voltage);

            if(battery_voltage > 0.0f)
            {
                chg_send_bms_mode_ntf(1U);
                charger_next_state = CHARGER_SM_START_CHARGING;
            }
            else
            {
                charger_next_state = CHARGER_SM_WAIT_BATT_VOLTAGE;
            }
            
            charger_prev_state = CHARGER_SM_WAIT_BATT_VOLTAGE;
            break;
        
        case CHARGER_SM_SEND_BATT_VI:
            /* Send battery voltage */
            bms_get_current((float_t *)&battery_current_f, LTC2946_CHG_CHANNEL);
            bms_get_voltage_cm((float_t *)&battery_voltage_f);
            
            battery_voltage = (uint16_t)(battery_voltage_f * 100U);
            battery_current = (uint16_t)(battery_current_f * 100U);
        
            /* Send battery v, i parameters to charger */
            dmq.msg_id = DBA_SEND_CHG_MSG;
            dmq.data[0] = CHG_MSG_BATT_VI_ID & 0x00FFU;
            dmq.data[1] = (CHG_MSG_BATT_VI_ID >> 8U) & 0x00FFU;
        
            dmq.data[2] = battery_current & 0xFFU;
            dmq.data[3] = (battery_current >> 8U) & 0x00FFU;
            dmq.data[4] = battery_voltage & 0x00FFU;
            dmq.data[5] = (battery_voltage >> 8U) & 0x00FFU;

            dmq.data[6] = 0U;
            dmq.data[7] = 0U;
        
            (void)osif_msg_queue_send(dba_msg_queue, &dmq, 0U, 10U);
            charger_next_state = CHARGER_SM_CHECK_BATT_TEMPERATURES;
            
            charger_prev_state = CHARGER_SM_SEND_BATT_VI;
            break;
            
        case CHARGER_SM_CHECK_BATT_TEMPERATURES:
            bms_get_max_cell_temperature((float_t *)&battery_max_cell_t);
            if(battery_max_cell_t > 45.0f)
            {
                /* Stop Charging */
                dmq.msg_id = DBA_SEND_CHG_MSG;
                dmq.data[0] = CHG_MSG_CHARGER_ON_OFF_CTRL & 0x00FFU;
                dmq.data[1] = (CHG_MSG_CHARGER_ON_OFF_CTRL >> 8U) & 0x00FFU;
                dmq.data[2] = CHARGER_CTRL_OFF;
                
                clear_status_bit(STAT_VCU_CHARGING_IN_PROGRESS);
                
                /* Notify UI to transition to pause screen */
                dmq.msg_id = DBA_SEND_UI_CHG_PAUSE_ERR_NTF;
                (void)osif_msg_queue_send(dba_msg_queue, &dmq, 0U, 10U);
                
                charger_next_state = CHARGER_SM_WAIT_BATT_COOLDOWN;
            }
            else
            {
                charger_next_state = CHARGER_SM_CHECK_FAULTS;
            }
            
            charger_prev_state = CHARGER_SM_CHECK_BATT_TEMPERATURES;
            break;
        
        case CHARGER_SM_WAIT_BATT_COOLDOWN:
            bms_get_max_cell_temperature((float_t *)&battery_max_cell_t);
            if(battery_max_cell_t <= 40.0f)
            {
                if(charger_status_ored > 1U)
                {
                    charger_status_ored = 0U;
                    charger_next_state = CHARGER_SM_SEND_NTF;
                }
                else
                {
                    /* Start Charging */
                    dmq.msg_id = DBA_SEND_CHG_MSG;
                    dmq.data[0] = CHG_MSG_CHARGER_ON_OFF_CTRL & 0x00FFU;
                    dmq.data[1] = (CHG_MSG_CHARGER_ON_OFF_CTRL >> 8U) & 0x00FFU;
                    dmq.data[2] = CHARGER_CTRL_ON;
                    
                    set_status_bit(STAT_VCU_CHARGING_IN_PROGRESS);
                    charger_next_state = CHARGER_SM_CHECK_FAULTS;
                }
            }
            else
            {
                charger_next_state = CHARGER_SM_WAIT_BATT_COOLDOWN;
            }
            
            if(chg_get_key_state() != 0U)
            {
                charger_next_state = CHARGER_SM_CHECK_KEY_STATE2;
            }

            charger_prev_state = CHARGER_SM_WAIT_BATT_COOLDOWN;
            break;
            
        case CHARGER_SM_CHECK_FAULTS:
            if((charger_context.charger_status == 0U)                   ||
               ((charger_context.charger_status & 0x01U) == 0x01U)      ||
               ((charger_context.charger_status & 0x800U) == 0x800U)    ||
               (((charger_context.charger_status & 0x200U) == 0x200U)))
            {
                charger_next_state = CHARGER_SM_CHECK_C20_CURRENT;
            }
            else
            {
                set_status_bit(STAT_VCU_CHARGING_ERROR);

                dmq.msg_id = DBA_SEND_CHG_MSG;
                dmq.data[0] = CHG_MSG_CHARGER_ON_OFF_CTRL & 0x00FFU;
                dmq.data[1] = (CHG_MSG_CHARGER_ON_OFF_CTRL >> 8U) & 0x00FFU;
                dmq.data[2] = CHARGER_CTRL_OFF;
                (void)osif_msg_queue_send(dba_msg_queue, &dmq, 0U, 10U);
                
                dmq_msg_1.msg_id = DBA_SEND_DISPLAY_MSG;
                dmq_msg_1.data[0] = DBA_DISPLAY_TELL_TALE_CTRL_MSG;
                dmq_msg_1.data[1] = TT_CHARGING_INDICATOR;
                dmq_msg_1.data[2] = DBA_DISPLAY_TELL_TALE_ON;
                (void)osif_msg_queue_send(dba_msg_queue, &dmq_msg_1, 0U, 10U);
                
                osif_thread_clear_flag(CHG_TASK_START_FLAG);

                charger_next_state = CHARGER_WAIT_DISCONNECT;
            }

            charger_prev_state = CHARGER_SM_CHECK_FAULTS;
            break;
        
        case CHARGER_SM_CHECK_C20_CURRENT:
            bms_get_design_capacity((uint8_t *)&design_capacity);
        
            if(battery_current_f > 5.0f)
            {
                c20_sentinel = 0U;
                charger_next_state = CHARGER_SM_CHECK_KEY_STATE2;
            }
            else
            {
                c20_sentinel++;
                if(c20_sentinel > 50U)
                {
                    dmq.msg_id = DBA_SEND_CHG_MSG_HBT_TIMER_OFF;
                    dmq.data[0] = 0U;
                    (void)osif_msg_queue_send(dba_msg_queue, &dmq, 0U, 10U);
                    
                    dmq_msg_1.msg_id = DBA_SEND_CHG_MSG;
                    dmq_msg_1.data[0] = CHG_MSG_CHARGER_ON_OFF_CTRL & 0x00FFU;
                    dmq_msg_1.data[1] = (CHG_MSG_CHARGER_ON_OFF_CTRL >> 8U) & 0x00FFU;
                    dmq_msg_1.data[2] = CHARGER_CTRL_OFF;
                    dmq_msg_1.data[3] = 0xACU;
                    (void)osif_msg_queue_send(dba_msg_queue, &dmq_msg_1, 0U, 10U);

                    dmq_msg_2.msg_id = DBA_SEND_DISPLAY_MSG;
                    dmq_msg_2.data[0] = DBA_DISPLAY_TELL_TALE_CTRL_MSG;
                    dmq_msg_2.data[1] = TT_CHARGING_INDICATOR;
                    dmq_msg_2.data[2] = DBA_DISPLAY_TELL_TALE_ON;
                    (void)osif_msg_queue_send(dba_msg_queue, &dmq_msg_2, 0U, 10U);
                    
                    set_status_bit(STAT_VCU_CHARGING_COMPLETE);
                    clear_status_bit(STAT_VCU_CHARGING_IN_PROGRESS);
                    osif_thread_clear_flag(CHG_TASK_START_FLAG);
                    
                    bms_get_chg_ah(0U, &charger_context.coulombs_after_charge);
                    
                    charger_next_state = CHARGER_WAIT_DISCONNECT;
                    c20_sentinel = 0U;
                }
                else
                {
                    charger_next_state = CHARGER_SM_CHECK_KEY_STATE2;
                }
            }
            
            charger_prev_state = CHARGER_SM_CHECK_C20_CURRENT;
            break;
            
        case CHARGER_SM_CHECK_KEY_STATE2:
            if(chg_get_key_state() == 0U)
            {
                charger_next_state = CHARGER_SM_SEND_BATT_VI;
            }
            else
            {
                dmq.msg_id = DBA_SEND_CHG_MSG;
                dmq.data[0] = CHG_MSG_CHARGER_ON_OFF_CTRL & 0x00FFU;
                dmq.data[1] = (CHG_MSG_CHARGER_ON_OFF_CTRL >> 8U) & 0x00FFU;
                dmq.data[2] = CHARGER_CTRL_OFF;
            
                (void)osif_msg_queue_send(dba_msg_queue, &dmq, 0U, 10U);
                
                dmq_msg_1.msg_id = DBA_SEND_DISPLAY_MSG;
                dmq_msg_1.data[0] = DBA_DISPLAY_TELL_TALE_CTRL_MSG;
                dmq_msg_1.data[1] = TT_CHARGING_INDICATOR;
                dmq_msg_1.data[2] = DBA_DISPLAY_TELL_TALE_ON;
            
                (void)osif_msg_queue_send(dba_msg_queue, &dmq_msg_1, 0U, 10U);
                
                dmq_msg_2.msg_id = DBA_STOP_DISPLAY_SLEEP_TIMER_MSG;
                (void)osif_msg_queue_send(dba_msg_queue, &dmq_msg_2, 0U, 10U);
                
                dmq_msg_3.msg_id = DBA_SEND_UI_CHG_OFF_NTF;
                (void)osif_msg_queue_send(dba_msg_queue, &dmq_msg_3, 0U, 10U);
                osif_thread_clear_flag(CHG_TASK_START_FLAG);
                
                osif_time_delay(500U);
                charger_next_state = CHARGER_SM_SEND_NTF;
            }
            
            charger_prev_state = CHARGER_SM_CHECK_KEY_STATE2;
            break;
            
        case CHARGER_WAIT_DISCONNECT:
            if(charger_prev_state == CHARGER_SM_CHECK_C20_CURRENT)
            {
                charger_task_start_flag = osif_thread_wait_on_flag(CHG_TASK_WAIT_DISCONNECT_FLAG, OSIF_WAIT_ANY_FLAG | OSIF_FLAGS_NO_CLEAR, OSIF_WAIT_FOREVER);
                if((charger_task_start_flag & CHG_TASK_WAIT_DISCONNECT_FLAG) == CHG_TASK_WAIT_DISCONNECT_FLAG)
                {
                    osif_thread_clear_flag(CHG_TASK_WAIT_DISCONNECT_FLAG);
                    charger_next_state = CHARGER_SM_CLEANUP;
                }
            }
            else if(charger_prev_state == CHARGER_SM_CHECK_FAULTS)
            {
                if(charger_context.charger_status == 0U)
                {
                    osif_time_delay(500U);
                    charger_next_state = CHARGER_SM_SEND_NTF;
                }
                else
                {
                    charger_next_state = CHARGER_WAIT_DISCONNECT;
                }
            }
            
            break;
        
        case CHARGER_SM_CLEANUP:
            chg_send_bms_mode_ntf(0U);

            dmq_msg_1.msg_id = DBA_RESTART_TIMERS_MSG;
            dmq_msg_1.data[0] = 0U;
            (void)osif_msg_queue_send(dba_msg_queue, &dmq_msg_1, 0U, 10U);
        
            osif_time_delay(1000);

            dmq_msg_2.msg_id = DBA_SEND_DISPLAY_MSG;
            dmq_msg_2.data[0] = DBA_DISPLAY_TELL_TALE_CTRL_MSG;
            dmq_msg_2.data[1] = TT_CHARGING_INDICATOR;
            dmq_msg_2.data[2] = DBA_DISPLAY_TELL_TALE_OFF;
            (void)osif_msg_queue_send(dba_msg_queue, &dmq_msg_2, 1U, 100U);
        
            dmq_msg_3.msg_id = DBA_SEND_DISPLAY_MSG;
            dmq_msg_3.data[0] = DBA_DISP_PWR_ON;
            dmq_msg_3.data[1] = 0U;
            dmq_msg_3.data[2] = 0U;
            (void)osif_msg_queue_send(dba_msg_queue, &dmq_msg_3, 0U, 10U);
        
            if(cd1030_is_high_beam_active() == 1U)
            {
                /* Turn ON high beam tell tale if required */
                dmq_msg_5.msg_id = DBA_SEND_DISPLAY_MSG;
                dmq_msg_5.data[0] = DBA_DISPLAY_TELL_TALE_CTRL_MSG;
                dmq_msg_5.data[1] = (uint8_t)TT_HIGH_BEAM_INDICATOR;
                dmq_msg_5.data[2] = DBA_DISPLAY_TELL_TALE_ON;
            
                (void)osif_msg_queue_send(dba_msg_queue, &dmq_msg_5, 0U, 10U);
            }

            mc_set_motor_pwr_state(MC_MOTOR_PWR_ON);
            bcm_control(BCM_ABS_EN_CTRL, BCM_CTRL_STATE_ON);
            
            set_status_bit(STAT_VCU_CHARGING_COMPLETE);
            clear_status_bit(STAT_VCU_CHARGING_IN_PROGRESS);
        
            osif_thread_clear_flag(CHG_TASK_START_FLAG);
            charger_next_state = CHARGER_SM_WAIT_TASK_START_NTF;
            
            charger_prev_state = CHARGER_SM_CLEANUP;
            break;
            
        default:
            __NOP();
            break;
    }    
}

__NO_RETURN static void chg_task(void *arg)
{   
    UNUSED_PARAM(arg);
    
    while(1)
    {
        charger_queue_process();
        charger_sm();
    }
}

uint32_t charger_get_status(void)
{
    return charger_context.charger_status;
}

uint32_t chg_get_connection_state(void)
{
    return charger_context.charger_connection_state;
}

void chg_get_charger_context(charger_ctx_t *charger_ctxt)
{
    volatile uint8_t obc_fw_max = 0U;
    volatile uint8_t obc_fw_min = 0U;

    if(charger_ctxt != NULL)
    {
        charger_ctxt->charger_log_boundary = 0xC5A710BFU;
        charger_ctxt->charger_comm_timer = charger_context.charger_comm_timer;
        charger_ctxt->charger_connection_state = charger_context.charger_connection_state;
        charger_ctxt->coulombs_after_charge = charger_context.coulombs_after_charge;
        charger_ctxt->coulombs_before_charge = charger_context.coulombs_before_charge;
        charger_ctxt->charger_fw_major_num = charger_context.charger_fw_major_num;
        charger_ctxt->charger_fw_minor_num = charger_context.charger_fw_minor_num;
        charger_ctxt->charger_type = charger_context.charger_type;
        charger_ctxt->charger_status = charger_context.charger_status;
        
        charger_ctxt->chg_bandgap_volts = charger_context.chg_bandgap_volts;
        charger_ctxt->psfb_fet_temp = charger_context.psfb_fet_temp;
        charger_ctxt->pfc_fet_temp = charger_context.pfc_fet_temp;
        charger_ctxt->iofb_volts = charger_context.iofb_volts;
        charger_ctxt->vofb_volts = charger_context.vofb_volts;
        charger_ctxt->transformer_temp = charger_context.transformer_temp;
        
        if(charger_ctxt->charger_fw_major_num == 0U)
        {
            dba_get_chg_fw_ver((uint8_t *)&obc_fw_max, (uint8_t *)&obc_fw_min);
        }
        
        charger_ctxt->charger_fw_major_num = obc_fw_max;
        charger_ctxt->charger_fw_minor_num = obc_fw_min;        
    }
}

thread_id_t chg_task_get_id(void)
{
    return thread_chg;
}

static void charger_comm_timeout_handler(void *arg)
{
    UNUSED_PARAM(arg);
    
    (void)osif_timer_stop(charger_context.charger_comm_timer);
    
    charger_context.charger_connection_state = 0U;

    charger_next_state = CHARGER_SM_CLEANUP;
    (void)osif_thread_set_flag(thread_chg, CHG_TASK_WAIT_DISCONNECT_FLAG);
}

void chg_task_create(void)
{
    uint32_t param = NULL;

    chg_msg_queue = osif_msg_queue_create(CHG_MSGQUEUE_OBJECTS, sizeof(chg_msg_queue_obj_t));
    DEV_ASSERT(chg_msg_queue != NULL);

    thread_chg = osif_thread_create(chg_task, &param, &chg_controller_attr);
    DEV_ASSERT(thread_chg != NULL);
    
    charger_context.charger_comm_timer = osif_timer_create(charger_comm_timeout_handler, osTimerPeriodic, NULL, NULL);
    DEV_ASSERT(charger_context.charger_comm_timer != NULL);

}
