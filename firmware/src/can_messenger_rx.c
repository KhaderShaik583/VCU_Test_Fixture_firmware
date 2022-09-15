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
 
#include "can_messenger_rx.h"
#include "stat.h"
#include "version.h"
#include "bms.h"
#include "bms_task.h"
#include "debug_console.h"
#include "fota.h"
#include "lsm_task.h"
#include "init_msg.h"
#include "charger_task.h"
#include "cd1030.h"
#include "pac1921_task.h"

#define TS_HOUR_MASK    (0x0000001fU)
#define TS_MINUTE_MASK  (0x000007e0U)
#define TS_SECOND_MASK  (0x0001f800U)

#define CHARGER_STATE_IDLE              (0U)
#define CHARGER_STATE_WAIT              (1U)
#define CHARGER_STATE_CONNECTED         (2U)
#define CHARGER_STATE_WAIT_DISCONNECT   (3U)

static bms_meas_t *meas = NULL;
static auxiliary_info_t aux_info;
static charger_ctx_t charger_ctxt;
static trip_meter_disp_t trip_meters;
static vcu_info_msg_t hdr;
static imx_dbg_msg_t imx_dbg_msg;
static osif_timer_id_t *data_tx_timer = NULL;
static uint32_t initial_sentinel = 0U;

#ifdef USE_FEATURE_FLEXCAN_IF

static bms_ctx_t batt_ctx[MAX_CANFD_LOGICAL_INTERFACES];
static volatile uint32_t canfd_logical_bus_id = 0U;
static canfd_queue_desc_t canfd_expected_rx[MAX_CANFD_LOGICAL_INTERFACES];
static float_t cell_deltav = 0.0f;
static volatile uint8_t alert_flag = 0U;

static float_t initial_dsg_charge = 0.0f;
static float_t initial_chg_charge = 0.0f;
static float_t initial_dsg_energy = 0.0f;
static float_t initial_chg_energy = 0.0f;

static void bms_send_imx_dbg_msg(uint32_t msg_prio);
void bms_data_tx_timer_handler(void *arg);
static float_t get_max_cell_temp(uint32_t bus);
static bms_info_t battery_info;
static ltc2946_boot_chg_eng_t boot_chg_eng;

static void bus_process_status(uint64_t status, uint32_t bus)
{
    uint32_t active_pack = 0U;
    uint32_t balancing_timer = 0U;
    
    canfd_queue_desc_t can_message;
    
    active_pack = bus;
    
    if(status & ((uint64_t)1U << STAT_LTC2946_DSG_ALERT_FLAG)) 
    {
        dbg_printf("W,%d,Alert signal from LTC2946-D\n\r", active_pack);
        
        /* Retrieve over-current value */
        can_message.is_cyclic = 0;
        can_message.cycle_count = 0;
        can_message.msg_id = CAN_IF_MSG_BMS_DSG_OC_INFO; 
        (void)canfd_enqueue(can_message, (uint8_t)bus); 
        
        can_message.msg_id = CAN_IF_MSG_BMS_PEAK_DSG_I_INFO; 
        (void)canfd_enqueue(can_message, (uint8_t)bus);
        
        can_message.msg_id = CAN_IF_MSG_DSG_CHG_FET_OFF; 
        (void)canfd_enqueue(can_message, (uint8_t)bus);
    }

    if(status & ((uint64_t)1U << STAT_LTC2946_CHG_ALERT_FLAG)) 
    {
        alert_flag = 1U;
        dbg_printf("W,%d,Alert signal from LTC2946-C\n\r", active_pack);
        
        /* Retrieve over-current value */
        can_message.is_cyclic = 0;
        can_message.cycle_count = 0;
        can_message.msg_id = CAN_IF_MSG_BMS_CHG_OC_INFO; 
        (void)canfd_enqueue(can_message, (uint8_t)bus); 
        
        can_message.msg_id = CAN_IF_MSG_DSG_CHG_FET_OFF; 
        (void)canfd_enqueue(can_message, (uint8_t)bus);
    }
    
    if((status & ((uint64_t)1U << STAT_DSG_FET_STATUS_FLAG)) &&
       (batt_ctx[active_pack].battery_dfet_state != MOSFET_ON_SW_STATE))
    {
        batt_ctx[active_pack].battery_dfet_state = MOSFET_ON_SW_STATE;
        (void)osif_thread_set_flag(mc_task_get_id(), MC_TASK_FET_ON_FLAG);
    }
    else if((status & ((uint64_t)1U << STAT_DSG_FET_STATUS_FLAG)) == 0U)
    {
        batt_ctx[active_pack].battery_dfet_state = MOSFET_OFF_SW_STATE;
    }
    else
    {
        __NOP();
    }
        
    if((status & ((uint64_t)1U << STAT_CHG_FET_STATUS_FLAG) &&
       (batt_ctx[active_pack].battery_cfet_state != MOSFET_ON_SW_STATE)))
    {
        batt_ctx[active_pack].battery_cfet_state = MOSFET_ON_SW_STATE;
    }
    else if((status & ((uint64_t)1U << STAT_CHG_FET_STATUS_FLAG)) == 0U)
    {
        batt_ctx[active_pack].battery_cfet_state = MOSFET_OFF_SW_STATE;
    }
    else
    {
        __NOP();
    }
    
    dbg_printf("I,%d,FET: %s | %s\n\r", bus, (batt_ctx[active_pack].battery_dfet_state == MOSFET_ON_SW_STATE)?"ON":"OFF", (batt_ctx[active_pack].battery_cfet_state == MOSFET_ON_SW_STATE)?"ON":"OFF");
    
    if(status & ((uint64_t)1U << STAT_BAL_ACT_STATUS_FLAG))   
    {
        batt_ctx[active_pack].is_balancer_running = 1U;
    }
    else 
    {
        batt_ctx[active_pack].is_balancer_running = 0U;
    }
    
    if(status & ((uint64_t)1U << STAT_BAL_TIMER_STATUS_FLAG))   
    {
        balancing_timer = 1U;   
    }
    else 
    {
        balancing_timer = 0U;
    }
    
    dbg_printf("I,%d,BAL: %s | %s\n\r", bus, (balancing_timer == 1U)?"ON":"OFF", (batt_ctx[active_pack].is_balancer_running)?"ON":"OFF");

    if(status & ((uint64_t)1U << STAT_UV_THR_FLAG)) 
    {
        dbg_printf("W,%d,Undervoltage threshold triggered\n\r", active_pack);
        
        /* Retrieve overvoltage cell and value value */
        can_message.is_cyclic = 0;
        can_message.cycle_count = 0;
        can_message.msg_id = CAN_IF_MSG_BMS_DSG_UV_INFO; 
        (void)canfd_enqueue(can_message, (uint8_t)bus);
        
        can_message.msg_id = CAN_IF_MSG_BMS_PEAK_DSG_I_INFO; 
        (void)canfd_enqueue(can_message, (uint8_t)bus);
        
        can_message.msg_id = CAN_IF_MSG_DSG_CHG_FET_OFF; 
        (void)canfd_enqueue(can_message, (uint8_t)bus);
    }

    if(status & ((uint64_t)1U << STAT_OV_THR_FLAG)) 
    {
        dbg_printf("W,%d,Overvoltage threshold triggered\n\r", active_pack);

        /* Retrieve overvoltage cell and value value */
        can_message.is_cyclic = 0;
        can_message.cycle_count = 0;
        can_message.msg_id = CAN_IF_MSG_BMS_CHG_OV_INFO; 
        (void)canfd_enqueue(can_message, (uint8_t)bus);
        
        can_message.msg_id = CAN_IF_MSG_DSG_CHG_FET_OFF; 
        (void)canfd_enqueue(can_message, (uint8_t)bus);
    }
    
    if(status & ((uint64_t)1U << STAT_LTC6812_WDT_SET_FLAG))
    {
        dbg_printf("W,%d,LTC6812 WDT Triggered\n\r", active_pack);
    }

    if(status & ((uint64_t)1U << STAT_BATTERY_TEMP_OVER_MIN_THRESHOLD))
    {
        dbg_printf("W,%d,Warning Battery crossing min. temperature threshold.\n\r", active_pack);
    }    
    
    if(status & ((uint64_t)1U << STAT_BATTERY_TEMP_OVER_MAX_THRESHOLD))
    {
        dbg_printf("W,%d,Warning Battery crossing max. temperature threshold.\n\r", active_pack);
    }    
    
    if(status & ((uint64_t)1U << STAT_LTC6812_SAFETY_TIMER_FLAG))
    {
        dbg_printf("W,%d,Warning UV/OV Safety Timer triggered.\n\r", active_pack);
    } 
    
    if(status & ((uint64_t)1U << STAT_LTC6812_PEC_ERROR))
    {
        dbg_printf("E,%d,Error LTC6812 SPI data corruption.\n\r", active_pack);
    }
    
    if(status & ((uint64_t)1U << STAT_UV_OV_THR_FOR_TURN_ON))
    {
        dbg_printf("W,%d,Cells not in a state to turn ON.\n\r", active_pack);
    }

    if(status & ((uint64_t)1U << STAT_BALANCER_ABORT_FLAG))
    {
        dbg_printf("W,%d,Warning Balancing Aborted.\n\r", active_pack);
    }    
    
    if(status & ((uint64_t)1U << STAT_BALANCER_RESET_FLAG))
    {
        dbg_printf("W,%d,Warning Balancer Reset.\n\r", active_pack);
    } 

    if(status & ((uint64_t)1U << STAT_BALANCING_COMPLETE_FLAG))
    {
        dbg_printf("W,%d,Balancing Complete.\n\r", active_pack);
    } 
    
    if(status & ((uint64_t)1U << STAT_BMS_UNECOVERABLE_FAILURE))
    {
        dbg_printf("E,%d,BMS Unrecoverable Error.\n\r", active_pack);
    }
    
    if(status & ((uint64_t)1U << STAT_DSG_INA302_ALERT1))
    {
        dbg_printf("E,%d,DSG INA302 ALERT1 Triggered.\n\r", active_pack);
    }
    
    if(status & ((uint64_t)1U << STAT_DSG_INA302_ALERT2))
    {
        dbg_printf("E,%d,DSG INA302 ALERT2 Triggered.\n\r", active_pack);
    }     
    
    if(status & ((uint64_t)1U << STAT_MOSFET_OVER_TMP_ALERT))
    {
        dbg_printf("E,%d,MOSFET Over-temperature Alert.\n\r", active_pack);
    }
    
    if(status & ((uint64_t)1U << STAT_FET_FRONT_OVER_TMP_ALERT))
    {
        dbg_printf("E,%d,MOD1 Over-temperature Alert.\n\r", active_pack);
    }

    if(status & ((uint64_t)1U << STAT_BAT_PLUS_OVER_TMP_ALERT))
    {
        dbg_printf("E,%d,MOD2 Over-temperature Alert.\n\r", active_pack);
    }
    
    if(status & ((uint64_t)1U << STAT_BAT_MINUS_OVER_TMP_ALERT))
    {
        dbg_printf("E,%d,MOD3 over 65 Degrees Celcius.\n\r", active_pack);
    }
    
    if(status & ((uint64_t)1U << STAT_PACK_PLUS_MCPCB_OVER_TMP_ALERT))
    {
        dbg_printf("E,%d,MOD4 over 65 Degrees Celcius.\n\r", active_pack);
    }

    if(status & ((uint64_t)1U << STAT_REL_HUMIDITY_OVERVALUE_ALERT))
    {
        dbg_printf("E,%d,Relative Humidity over 90%.\n\r", active_pack);
    }
    
    if(status & ((uint64_t)1U << STAT_DSG_FUSE_BLOWN_ALERT))
    {
        dbg_printf("E,%d,Fuse Blown DSG.\n\r", active_pack);
    }
    
    
    if(status & ((uint64_t)1U << STAT_CHG_FUSE_BLOWN_ALERT))
    {
        dbg_printf("E,%d,Fuse Blown CHG.\n\r", active_pack);
    }
    
    if(status & ((uint64_t)1U << STAT_FET_TURN_ON_FAILURE))
    {
        dbg_printf("E,%d,FET Turn ON Fail. MOSFET FB\n\r", active_pack);
    }
    
    if(status & ((uint64_t)1U << STAT_FET_TURN_OFF_FAILURE))
    {
        dbg_printf("E,%d,FET Turn OFF Fail. MOSFET FB\n\r", active_pack);
    } 

    if(status & ((uint64_t)1U << STAT_BAL_RES_OVER_TEMPERATURE))
    {
        dbg_printf("E,%d,Balancing resistor over temperture\n\r", active_pack);
    }     
    
    if(status & ((uint64_t)1U << STAT_LTC2946_COMM_FAILURE))
    {
        dbg_printf("E,%d,LTC2946 I2C bus failure\n\r", active_pack);
    }  

    if(status & ((uint64_t)1U << STAT_HW_UV_SHUTDOWN))
    {
        dbg_printf("E,%d,Hardware Undervoltage detected.\n\r", active_pack);
    }       
    
    if(status & ((uint64_t)1U << STAT_HW_OV_SHUTDOWN))
    {
        dbg_printf("E,%d,Hardware Overvoltage detected.\n\r", active_pack);
    }
    
    if(status & ((uint64_t)1U << STAT_HW_OVER_TMP_SHUTDOWN))
    {
        dbg_printf("E,%d,Hardware Overtemperature detected.\n\r", active_pack);
    }

    if(status & ((uint64_t)1U << STAT_LTC7103_PGOOD))
    {
        dbg_printf("E,%d,LTC7103 Power not good.\n\r", active_pack);
    }
    
    if(status & ((uint64_t)1U << STAT_BATTERY_TEMP_TOO_LOW))
    {
        dbg_printf("E,%d,Cell temperature too low.\n\r", active_pack);
    }
    
    if(status & ((uint64_t)1U << STAT_ECC_ERM_ERR_FLAG))
    {
        dbg_printf("E,%d,BMS ECC ERM Correction.\n\r", active_pack);
    }
    
    if(status & ((uint64_t)1U << STAT_FG_MEAS_ABORT))
    {
        dbg_printf("E,%d,BMS FG Unrecoverable\n\r", active_pack);
    }
    
    if(status & ((uint64_t)1U << STAT_FG_I2C_BUS_RECOVERY_EXEC))
    {
        dbg_printf("W,%d,BMS FG I2C recovery start\n\r", active_pack);
    }
    
    if(status & ((uint64_t)1U << STAT_BAT_TAMPER_DETECTED))
    {
        dbg_printf("W,%d,BMS tamper alert\n\r", active_pack);
    }
    
    if(status & ((uint64_t)1U << STAT_FET_FRONT_OVER_TMP_WARN))
    {
        dbg_printf("W,%d,FET front min. threshold warning\n\r", active_pack);
    }
    
    if(status & ((uint64_t)1U << STAT_BAT_PLUS_OVER_TMP_WARN))
    {
        dbg_printf("W,%d,BAT+ min. threshold warning\n\r", active_pack);
    }
    
    if(status & ((uint64_t)1U << STAT_BAT_MINUS_OVER_TMP_WARN))
    {
        dbg_printf("W,%d,BAT- min. threshold warning\n\r", active_pack);
    }
    
    if(status & ((uint64_t)1U << STAT_PACK_PLUS_OVER_TMP_WARN))
    {
        dbg_printf("W,%d,Pack+ min. threshold warning\n\r", active_pack);
    }
    
    if(status & ((uint64_t)1U << STAT_TMP_THR_FOR_TURN_ON))
    {
        dbg_printf("W,%d,Unsafe temperature for turn on\n\r", active_pack);
    }
    
    if(status & ((uint64_t)1U << STAT_FG_MEAS_ERROR))
    {
        dbg_printf("W,%d,Fuelgauge i2c bus error\n\r", active_pack);
    }
    
    if(status & ((uint64_t)1U << STAT_PWR_MODE_CHARGE))
    {
        dbg_printf("W,%d,Power Mode->Charge\n\r", active_pack);
    }
}

static void process_CAN_IF_MSG_ID_GET_CELLS_INFO(const uint8_t *buffer)
{
    uint16_t *cv = NULL;
    
    DEV_ASSERT(buffer != NULL);
    
    cv = (uint16_t *)buffer;
    
    for(uint8_t i = 0U; i < CELL_CHANNELS; i++)
    {
        batt_ctx[canfd_logical_bus_id].bms_data.LTC6812.cv[i] = cv[i];
    }
}

static void process_CAN_IF_MSG_ID_GET_TEMPERATURE_INFO(const uint8_t *buffer)
{
    uint16_t *t = NULL;
    
    DEV_ASSERT(buffer != NULL);
    
    t = (uint16_t *)buffer;

    for(uint8_t i = 0U; i < MAX_TEMP_AUX_SENSORS; i++)
    {
        batt_ctx[canfd_logical_bus_id].bms_data.LTC6812.t[i] = t[i];
    }
}

static void process_CAN_IF_MSG_ID_GET_CM_DIAG_INFO(const uint8_t *buffer)
{
    ltc6812_diag_t *d = NULL;
    
    DEV_ASSERT(buffer != NULL);
    
    d = (ltc6812_diag_t *)buffer;
    
    batt_ctx[canfd_logical_bus_id].bms_diag = *d;
    
    dbg_printf("I,%d,Diag Info:", canfd_logical_bus_id);
    dbg_printf("I,Internal Temperature = %3.4f deg C\n\r", ((batt_ctx[canfd_logical_bus_id].bms_diag.itmp * 100e-6) / 7.5e-3) - (double)273.0);
    dbg_printf("I,Analog Power Supply = %3.4f V\n\r", batt_ctx[canfd_logical_bus_id].bms_diag.va * 100e-6);
    dbg_printf("I,Digital Power Supply = %3.4f V\n\r", batt_ctx[canfd_logical_bus_id].bms_diag.vd * 100e-6);
    dbg_printf("I,Mux Self Test = %s\n\r", (batt_ctx[canfd_logical_bus_id].bms_diag.mux_fail == 0U ? "Pass" : "Fail"));
    dbg_printf("I,ADC Self Test = %s\n\r", (batt_ctx[canfd_logical_bus_id].bms_diag.adc_selftest_fail == 0U ? "Pass" : "Fail"));
    dbg_printf("I,OV/UV Fault on Cell [%d]\n\r", batt_ctx[canfd_logical_bus_id].bms_diag.cell_index);
    
    for(uint8_t i = 0U; i < CELL_CHANNELS; i++)
    {
        if(batt_ctx[canfd_logical_bus_id].bms_diag.open_cell == 0U)
        {
            dbg_printf("I,No open wires detected\n\r");
            break;
        }
        else if(batt_ctx[canfd_logical_bus_id].bms_diag.open_cell & (1U << i))
        {
            dbg_printf("W,Open Wire detected on cell %d\n\r", i);
        }
    }
    
    dbg_printf("\n\r");
}

static void process_CAN_IF_MSG_ID_GET_FG_INFO(const uint8_t *buffer)
{
    max17205_ctx_t *m = NULL;
    
    DEV_ASSERT(buffer != NULL);
    
    m = (max17205_ctx_t *)buffer;
    
    batt_ctx[canfd_logical_bus_id].bms_data.m = *m;
}

static void bms_charger_workaround(void)
{
    static uint32_t chg_state = CHARGER_STATE_IDLE;
    static const float_t charger_wait_time = 3.0f;
    static const float_t charge_start_det_threshold = 7.0f;
    static float_t start_time = 0.0f;
    float_t end_time = 0.0f;
    sys_msg_queue_obj_t smq;
    float_t end_current = 0.0f;
    
#ifdef USE_FEATURE_VCU_ON_DESK
    canfd_queue_desc_t can_message;
#endif
    
    switch(chg_state)
    {
        case CHARGER_STATE_IDLE:
            if((batt_ctx[canfd_logical_bus_id].bms_data.LTC2946[LTC2946_CHG_CHANNEL].current > charge_start_det_threshold) &&
               (mc_get_vehicle_speed() == 0U) &&
               (chg_get_connection_state() == 0U))
            {
                start_time = batt_ctx[canfd_logical_bus_id].bms_data.LTC2946[LTC2946_CHG_CHANNEL].time;
                chg_state = CHARGER_STATE_WAIT;
            }
            else
            {
                chg_state = CHARGER_STATE_IDLE;
            }
            break;
            
        case CHARGER_STATE_WAIT:
            end_time = batt_ctx[canfd_logical_bus_id].bms_data.LTC2946[LTC2946_CHG_CHANNEL].time;
            if((end_time - start_time) > charger_wait_time)
            {
                if((batt_ctx[canfd_logical_bus_id].bms_data.LTC2946[LTC2946_CHG_CHANNEL].current > charge_start_det_threshold) &&
                   (mc_get_vehicle_speed() == 0U))
                {
                    mc_set_gear(MC_GEAR_POS_NEUT_OFF);
                    
                    bcm_control(BCM_ABS_EN_CTRL, BCM_CTRL_STATE_OFF);
                    bcm_control(BCM_DRL_LED_CTRL, BCM_CTRL_STATE_OFF);
                    bcm_control(BCM_LOW_BEAM_CTRL, BCM_CTRL_STATE_OFF);
                    
                    chg_state = CHARGER_STATE_CONNECTED;
                    set_status_bit(STAT_VCU_CHARGING_IN_PROGRESS);
                }
                else
                {
                    chg_state = CHARGER_STATE_IDLE;
                }
            }
            break;
            
        case CHARGER_STATE_CONNECTED:
            end_current = (float_t)(batt_ctx[canfd_logical_bus_id].bms_data.m.designCapacity / 20.0f);
            if((batt_ctx[canfd_logical_bus_id].battery_dfet_state == MOSFET_OFF_SW_STATE) ||
               (batt_ctx[canfd_logical_bus_id].bms_data.LTC2946[LTC2946_CHG_CHANNEL].current < end_current))
            {
                start_time = batt_ctx[canfd_logical_bus_id].bms_data.LTC2946[LTC2946_CHG_CHANNEL].time;
                chg_state = CHARGER_STATE_WAIT_DISCONNECT;
            }
            break;   
            
        case CHARGER_STATE_WAIT_DISCONNECT:
            end_time = batt_ctx[canfd_logical_bus_id].bms_data.LTC2946[LTC2946_CHG_CHANNEL].time;
            end_current = (float_t)(batt_ctx[canfd_logical_bus_id].bms_data.m.designCapacity / 20.0f);
        
            if((end_time - start_time) > 180.0f)
            {
                if(((batt_ctx[canfd_logical_bus_id].battery_dfet_state == MOSFET_OFF_SW_STATE) ||
                   (batt_ctx[canfd_logical_bus_id].bms_data.LTC2946[LTC2946_CHG_CHANNEL].current < end_current)))
                {
#if 1
                    set_status_bit(STAT_VCU_CHARGING_COMPLETE);
                    
/* Do not go to sleep after charging as long as charger is connected */
#ifndef USE_FEATURE_VCU_ON_DESK
                    smq.msg_id = MSG_ID_SLEEP_REQ;
                    smq.msg_len = 0U;
                    (void)osif_msg_queue_send(sys_msg_queue, &smq, 0U, 10U); 
#else
                    /* FETS OFF */
                    can_message.is_cyclic = 0;
                    can_message.cycle_count = 0;
                    can_message.msg_id = CAN_IF_MSG_DSG_CHG_FET_OFF; 
                    canfd_enqueue(can_message, 0U);
                
#endif /* USE_FEATURE_VCU_ON_DESK */
#endif
                    
                    chg_state = CHARGER_STATE_IDLE; 
                }
                else
                {
                    chg_state = CHARGER_STATE_CONNECTED;
                }
            }
            else
            {
                chg_state = CHARGER_STATE_WAIT_DISCONNECT; 
            }
            break;
            
        default:
            __NOP();
            break;
    }
}

static void process_CAN_IF_MSG_ID_GET_EM_INFO(const uint8_t *buffer)
{
    typedef struct {
        ltc2946_meas_min_t meas[2];
    }ltc_min_info_t;
    
    ltc_min_info_t *e = NULL;
    
    DEV_ASSERT(buffer != NULL);
    
    e = (ltc_min_info_t *)buffer;

    batt_ctx[canfd_logical_bus_id].bms_data.LTC2946[LTC2946_DSG_CHANNEL].adin     = e->meas[LTC2946_DSG_CHANNEL].adin;
    batt_ctx[canfd_logical_bus_id].bms_data.LTC2946[LTC2946_DSG_CHANNEL].alert    = e->meas[LTC2946_DSG_CHANNEL].alert;
    batt_ctx[canfd_logical_bus_id].bms_data.LTC2946[LTC2946_DSG_CHANNEL].current  = e->meas[LTC2946_DSG_CHANNEL].current;
    batt_ctx[canfd_logical_bus_id].bms_data.LTC2946[LTC2946_DSG_CHANNEL].fault    = e->meas[LTC2946_DSG_CHANNEL].fault;
    batt_ctx[canfd_logical_bus_id].bms_data.LTC2946[LTC2946_DSG_CHANNEL].status   = e->meas[LTC2946_DSG_CHANNEL].status;
    batt_ctx[canfd_logical_bus_id].bms_data.LTC2946[LTC2946_DSG_CHANNEL].swstatus = e->meas[LTC2946_DSG_CHANNEL].swstatus;
    batt_ctx[canfd_logical_bus_id].bms_data.LTC2946[LTC2946_DSG_CHANNEL].time     = e->meas[LTC2946_DSG_CHANNEL].time;
    batt_ctx[canfd_logical_bus_id].bms_data.LTC2946[LTC2946_DSG_CHANNEL].charge   = e->meas[LTC2946_DSG_CHANNEL].charge;
    
    batt_ctx[canfd_logical_bus_id].bms_data.LTC2946[LTC2946_CHG_CHANNEL].adin     = e->meas[LTC2946_CHG_CHANNEL].adin;
    batt_ctx[canfd_logical_bus_id].bms_data.LTC2946[LTC2946_CHG_CHANNEL].alert    = e->meas[LTC2946_CHG_CHANNEL].alert;
    batt_ctx[canfd_logical_bus_id].bms_data.LTC2946[LTC2946_CHG_CHANNEL].current  = e->meas[LTC2946_CHG_CHANNEL].current;
    batt_ctx[canfd_logical_bus_id].bms_data.LTC2946[LTC2946_CHG_CHANNEL].fault    = e->meas[LTC2946_CHG_CHANNEL].fault;
    batt_ctx[canfd_logical_bus_id].bms_data.LTC2946[LTC2946_CHG_CHANNEL].status   = e->meas[LTC2946_CHG_CHANNEL].status;
    batt_ctx[canfd_logical_bus_id].bms_data.LTC2946[LTC2946_CHG_CHANNEL].swstatus = e->meas[LTC2946_CHG_CHANNEL].swstatus;
    batt_ctx[canfd_logical_bus_id].bms_data.LTC2946[LTC2946_CHG_CHANNEL].time     = e->meas[LTC2946_CHG_CHANNEL].time;    
    batt_ctx[canfd_logical_bus_id].bms_data.LTC2946[LTC2946_CHG_CHANNEL].charge   = e->meas[LTC2946_CHG_CHANNEL].charge; 

#ifndef USE_FEATURE_VCU_ON_DESK
    bms_charger_workaround();
#endif
}

static void process_CAN_IF_MSG_ID_GET_SENSOR_INFO(const uint8_t *buffer)
{
    /* 
        the size of the meas array below must equal the MAX_SELECTED_SENSORS value
        defined in sensors.h in BMS source.
    */
    
    typedef struct {
        float_t meas[6];
    }sensors_info_t;
    
    sensors_info_t *s = NULL;
    
    DEV_ASSERT(buffer != NULL);
    
    s = (sensors_info_t *)buffer;
    
    for(uint8_t i = 0U; i < 6U; i++)
    {
        batt_ctx[canfd_logical_bus_id].bms_data.sensor_values[i] = s->meas[i];
    }
}

static void process_CAN_IF_MSG_ID_BMS_STATUS(const uint8_t *buffer)
{
    uint64_t *stat = NULL;
    
    DEV_ASSERT(buffer != NULL);
    
    stat = (uint64_t *)buffer;
    
    batt_ctx[canfd_logical_bus_id].status = *stat;
    
    bus_process_status(batt_ctx[canfd_logical_bus_id].status, canfd_logical_bus_id);
}

static void process_CAN_IF_MSG_ID_BAL_INFO(const uint8_t *buffer)
{
    uint32_t hrs = 0U;
    uint32_t min = 0U;
    uint32_t sec = 0U;

    typedef struct {
        uint32_t bal_run_count;
        uint32_t bal_runtime;
        uint32_t bal_abort_cause;
    }bal_info_t;
    
    DEV_ASSERT(buffer != NULL);
    
    bal_info_t *b = NULL;
    
    DEV_ASSERT(buffer != NULL);
    
    b = (bal_info_t *)buffer;
    
    batt_ctx[canfd_logical_bus_id].bms_balancer_ctx.balancer_run_count = b->bal_run_count;
    batt_ctx[canfd_logical_bus_id].bms_balancer_ctx.balancer_time_ticks = b->bal_runtime;
    batt_ctx[canfd_logical_bus_id].bms_balancer_ctx.balancer_abort_cause = b->bal_abort_cause;
    
    hrs = b->bal_runtime / 3600U;
    min = (b->bal_runtime - (3600U * hrs)) / 60U;
    sec = (b->bal_runtime - (3600 * hrs) - (min * 60U));
    
    dbg_printf("I,%d,BRC: %d | BAC: 0x%x | BRT: %d Days %d Hr %d Min %d Sec\n\r", 
                    canfd_logical_bus_id,
                    batt_ctx[canfd_logical_bus_id].bms_balancer_ctx.balancer_run_count,
                    batt_ctx[canfd_logical_bus_id].bms_balancer_ctx.balancer_abort_cause,
                    0U, hrs, min, sec
              );
              
     dbg_printf("\n\r");
}

static void process_CAN_IF_MSG_GET_FW_INFO(const uint8_t *buffer)
{
    DEV_ASSERT(buffer != NULL);

    dbg_printf("I,%d,BMS Firmware Info: %s\n\r", canfd_logical_bus_id, buffer);
    dbg_printf("\n\r");
    
#ifdef USE_FEATURE_VCU_ON_DESK
    dbg_printf("\n\r");
    dbg_printf("Slot,Millis,DFET ON/OFF,CFET ON/OFF,FET TEMP,DSG ADIN,SC Current,DSG Current,CHG VIN,CHG Current,DSG Time,CHG Time,DSG Charge,CHG Charge,Cell1,Cell2,Cell3,Cell4,Cell5,Cell6,Cell7,Cell8,Cell9,Cell10,Cell11,Cell12,Cell13,Cell14,Cell DV,Sum-OF-Cells,DSG Power,DSG Energy,CHG Power,CHG Energy,Min CV,Balancing Bitmap\n\r");
    dbg_printf("Slot,Millis,TS1,TS2,TS3,TS4,TS5,TS6,TS7,TS8,TS9,TS10,TS11,TS12,TSMOD1,TSMOD2,TSMOD3,TSMOD4,TS0_FLT,TS13_FLT\n\r");
    dbg_printf("Slot,Millis,FET TEMP,DSG INA,BAL TEMP,HUM,IMON,SLOT_VTG,MAX_VTG,MAX_VBATT,MAX_AVG_CURN,MAX_SOC,MAX_TTE,MAX_TTF,REP_CAP,TS0_FLT,IR,Cycles,DS_CAP,FSTAT\n\r");
    dbg_printf("\n\r");
#endif 

    fet_sw_init();
    
    (void)snprintf(batt_ctx[canfd_logical_bus_id].fw_version, 64U, "%s", (char *)buffer);
    
}

static void process_CAN_IF_MSG_GET_LAST_RST_STATE(const uint8_t *buffer)
{
    volatile uint32_t can_esr_reg = 0U;
    
    DEV_ASSERT(buffer != NULL);
    
    volatile uint8_t rcm_srs0 = buffer[0];
    volatile uint8_t rcm_srs1 = buffer[1];
    
    (void)memcpy((uint8_t *)&can_esr_reg, &buffer[2], sizeof(uint32_t));
    
    dbg_printf("I,%d,Last Reset State:", canfd_logical_bus_id);
    
    if((rcm_srs0 & (1U << 1U)) > 0U)
    {
        dbg_printf("LVD Reset\n\r");
    }

    if((rcm_srs0 & (1U << 2U)) > 0U)
    {
        dbg_printf("Loss of Clock Reset\n\r");
    }

    if((rcm_srs0 & (1U << 3U)) > 0U)
    {
        dbg_printf("Loss of Lock Reset\n\r");
    }
    
    if((rcm_srs0 & (1U << 4U)) > 0U)
    {
        dbg_printf("CMU Loss of Clock Reset\n\r");
    }
    
    if((rcm_srs0 & (1U << 5U)) > 0U)
    {
        dbg_printf("Internal WDT Reset\n\r");
    }
    
    if((rcm_srs0 & (1U << 6U)) > 0U)
    {
        dbg_printf("External PIN Reset\n\r");
    }
    
    if((rcm_srs0 & (1U << 7U)) > 0U)
    {
        dbg_printf("Power on Reset\n\r");
    }
    
    if((rcm_srs1 & (1U << 0U)) > 0U)
    {
        dbg_printf("JTAG Reset\n\r");
    }
    
    if((rcm_srs1 & (1U << 1U)) > 0U)
    {
        dbg_printf("LOCKUP Reset\n\r");
    }
    
    if((rcm_srs1 & (1U << 2U)) > 0U)
    {
        dbg_printf("Software Reset\n\r");
    }
    
    if((rcm_srs1 & (1U << 3U)) > 0U)
    {
        dbg_printf("MDM AP Reset\n\r");
    }
    
    if((rcm_srs1 & (1U << 5U)) > 0U)
    {
        dbg_printf("SACKERR Reset\n\r");
    }
    
    dbg_printf("BMS CAN ESR: 0x%x\n\r", can_esr_reg);
    
    batt_ctx[canfd_logical_bus_id].rcm_srs0 = rcm_srs0;
    batt_ctx[canfd_logical_bus_id].rcm_srs1 = rcm_srs1;
    batt_ctx[canfd_logical_bus_id].bms_data.can_esr = can_esr_reg;
    
    dbg_printf("\n\r");
}

static void process_CAN_IF_MSG_GET_UID(const uint8_t *buffer)
{
    uint32_t uidl = 0U;
    uint32_t uidml = 0U;
    uint32_t uidmh = 0U;
    uint32_t uidh = 0U;
    
    DEV_ASSERT(buffer != NULL);
    
    uidl  = buffer[0] | ((uint32_t)buffer[1] << 8U) | ((uint32_t)buffer[2] << 16U) | ((uint32_t)buffer[3] << 24U);
    uidml = buffer[4] | ((uint32_t)buffer[5] << 8U) | ((uint32_t)buffer[6] << 16U) | ((uint32_t)buffer[7] << 24U);
    uidmh = buffer[8] | ((uint32_t)buffer[9] << 8U) | ((uint32_t)buffer[10] << 16U) | ((uint32_t)buffer[11] << 24U);
    uidh  = buffer[12] | ((uint32_t)buffer[13] << 8U) | ((uint32_t)buffer[14] << 16U) | ((uint32_t)buffer[15] << 24U);  

    batt_ctx[canfd_logical_bus_id].uid.uidl  = uidl;
    batt_ctx[canfd_logical_bus_id].uid.uidml = uidml;
    batt_ctx[canfd_logical_bus_id].uid.uidmh = uidmh;
    batt_ctx[canfd_logical_bus_id].uid.uidh  = uidh;

    dbg_printf("I,%d,BMS UID: UIDL,UIDML,UIDMH,UIDH\n\r", canfd_logical_bus_id);
    dbg_printf("|%d,0x%x,0x%x,0x%x,0x%x\n\r", canfd_logical_bus_id, batt_ctx[canfd_logical_bus_id].uid.uidl, batt_ctx[canfd_logical_bus_id].uid.uidml, batt_ctx[canfd_logical_bus_id].uid.uidmh, batt_ctx[canfd_logical_bus_id].uid.uidh);
    dbg_printf("\n\r");
}

static void process_CAN_IF_MSG_CM_PART2(const uint8_t *buffer)
{
	DEV_ASSERT(buffer != NULL);
    
    typedef struct {
            float_t mosfet_temp;
            float_t energyd;
            float_t energyc;
            float_t charged;
            float_t chargec;
            uint16_t aux_vref;
            uint16_t min_cv;
            uint16_t soc;
            uint16_t BalanceTime;
            uint16_t cd_bitmap;
    }ltc6812_daq_2_t;
    
    ltc6812_daq_2_t *dl2 = (ltc6812_daq_2_t *)buffer;
    
    batt_ctx[canfd_logical_bus_id].bms_data.LTC6812.mosfet_temp = dl2->mosfet_temp;
    batt_ctx[canfd_logical_bus_id].bms_data.LTC6812.aux_vref = dl2->aux_vref;
    batt_ctx[canfd_logical_bus_id].bms_data.LTC6812.min_cv = dl2->min_cv;
    batt_ctx[canfd_logical_bus_id].bms_data.LTC6812.soc = dl2->soc;
    batt_ctx[canfd_logical_bus_id].bms_data.LTC6812.cd_bitmap = dl2->cd_bitmap;
    
    if(initial_sentinel == 0U)
    {
        initial_dsg_charge = dl2->charged;
        initial_chg_charge = dl2->chargec;
        
        initial_dsg_energy = dl2->energyd;
        initial_chg_energy = dl2->energyc;
        
        initial_sentinel = 1U;
    }
    else
    {
        batt_ctx[canfd_logical_bus_id].bms_data.LTC2946[0].energy = dl2->energyd;
        batt_ctx[canfd_logical_bus_id].bms_data.LTC2946[1].energy = dl2->energyc;
    }
}

static void process_CAN_IF_MSG_BMS_READ_TIME(const uint8_t *buffer)
{
    uint32_t t = 0U;
    
    t = ((uint32_t)buffer[0]) | (((uint32_t)buffer[1]) << 8U) | 
        (((uint32_t)buffer[2]) << 16U) | (((uint32_t)buffer[3]) << 24U);
    
    batt_ctx[canfd_logical_bus_id].time = t;
}

static void get_cell_min_voltage(uint16_t *mcv, uint32_t bus)
{
    uint8_t i = 0U;
    uint8_t idx = 0U;
    float_t c = batt_ctx[bus].bms_data.LTC6812.cv[0];

	DEV_ASSERT(mcv != NULL);

    for(i = 1U; i < CELL_CHANNELS; i++)
    {
        if(batt_ctx[bus].bms_data.LTC6812.cv[i] < c)
        {
            c = batt_ctx[bus].bms_data.LTC6812.cv[i];
            idx = i;
        }
    }

    *mcv = batt_ctx[bus].bms_data.LTC6812.cv[idx];   
}

static void get_cell_max_voltage(uint16_t *mcv, uint32_t bus)
{
    uint8_t i = 0U;
    uint8_t idx = 0U;
    float_t c = batt_ctx[bus].bms_data.LTC6812.cv[0];

	DEV_ASSERT(mcv != NULL);

    for(i = 1U; i < CELL_CHANNELS; i++)
    {
        if(batt_ctx[bus].bms_data.LTC6812.cv[i] > c)
        {
            c = batt_ctx[bus].bms_data.LTC6812.cv[i];
            idx = i;
        }
    }

    *mcv = batt_ctx[bus].bms_data.LTC6812.cv[idx];
}

static void process_cell_delta(uint32_t bus)
{
    uint16_t mincv = 0U;
    uint16_t maxcv = 0U;
    
    get_cell_min_voltage(&mincv, bus);
    get_cell_max_voltage(&maxcv, bus);
    
    cell_deltav = (float_t)(maxcv * 0.0001f) - (mincv * 0.0001f);
    
    batt_ctx[bus].bms_data.delta_voltage = cell_deltav;
}

static float_t get_max_cell_temp(uint32_t bus)
{
    uint16_t max_temp = 0U;
    uint16_t cell_temperatures[CELL_CHANNELS];
    
    for(uint32_t i = 0; i < 12U; i++)
    {
        cell_temperatures[i] = batt_ctx[bus].bms_data.LTC6812.t[i];
    }
    
    cell_temperatures[12] = batt_ctx[bus].bms_data.LTC6812.t[16];
    cell_temperatures[13] = batt_ctx[bus].bms_data.LTC6812.t[17];
    
    for(uint32_t i = 0U; i < 14U; i++)
    {
        if(cell_temperatures[i] > max_temp)
        {
            max_temp = cell_temperatures[i];
        }
    }
    
    return (max_temp * 0.01f);
}

static void process_CAN_IF_MSG_BMS_EXECEPTION_INFO(const uint8_t *buffer, uint32_t bus)
{
    uint32_t lr = 0U;

    lr |= buffer[0] | ((uint32_t)buffer[1] << 8U) | ((uint32_t)buffer[2] << 16U) | ((uint32_t)buffer[3] << 24U);
    
    if(lr != 0xCEFCADDEU)
    {
        dbg_printf("\n\r############## BMS[%d] CRASH INFO ##############\n\r", bus);
        dbg_printf("LR: 0x%x\n\r", lr);
        dbg_printf("\n\r############## BMS[%d] CRASH INFO ##############\n\r", bus); 
        dbg_printf("\n\r");
        
        batt_ctx[bus].exeption_lr = lr;
    }
    else
    {
        batt_ctx[bus].exeption_lr = 0U;
        dbg_printf("I,%d,No Crash Info\n\r", bus);
    }
}

static void process_CAN_IF_MSG_BMS_DSG_OC_INFO_ID(const uint8_t *buffer, uint32_t bus)
{   
    volatile float_t sc_current = 0.0f;
    volatile float_t adin = 0.0f;
    
    (void)memcpy((uint8_t *)&sc_current, buffer, sizeof(float_t));
    (void)memcpy((uint8_t *)&adin, buffer + sizeof(float_t), sizeof(float_t));
    
    dbg_printf("\n\r############## BMS[%d] OC INFO ##############\n\r", bus);
    dbg_printf("I,%d,DSG OC Info: %3.3f A\n\r", bus, sc_current);
    dbg_printf("I,%d,ADIN Info: %3.3f V\n\r", bus, adin);
    dbg_printf("\n\r############## BMS[%d] OC INFO ##############\n\r", bus); 
    dbg_printf("\n\r");
    
    batt_ctx[bus].bms_data.dsg_oc_value = sc_current;
    aux_info.meas_marker = 0xD755A75DU;
    aux_info.dsg_ov_value = adin;
}

static void process_CAN_IF_MSG_BMS_CHG_OC_INFO_ID(const uint8_t *buffer, uint32_t bus)
{
    volatile float_t rg_current = 0.0f;
    volatile float_t adin = 0.0f;
    
    (void)memcpy((uint8_t *)&rg_current, buffer, sizeof(float_t));
    (void)memcpy((uint8_t *)&adin, buffer + sizeof(float_t), sizeof(float_t));
    
    dbg_printf("\n\r############## BMS[%d] OC INFO ##############\n\r", bus);
    dbg_printf("I,%d,CHG OC Info: %3.3f A\n\r", bus, rg_current);
    dbg_printf("I,%d,ADIN Info: %3.3f V\n\r", bus, adin);
    dbg_printf("\n\r############## BMS[%d] OC INFO ##############\n\r", bus); 
    dbg_printf("\n\r");
    
    batt_ctx[bus].bms_data.chg_oc_value = rg_current;
    aux_info.meas_marker = 0xC755A75CU;
    aux_info.chg_ov_value = adin;
}

static void process_CAN_IF_MSG_BMS_DSG_UV_INFO_ID(const uint8_t *buffer, uint32_t bus)
{
    volatile ov_uv_cell_info_t cell_fault_info;
    
    (void)memcpy((uint8_t *)&cell_fault_info, buffer, sizeof(ov_uv_cell_info_t));
    
    dbg_printf("\n\r############## BMS[%d] UV INFO ##############\n\r", bus);
    dbg_printf("I,%d,DSG UV Info: %3.3f V | Cell %d\n\r", bus, cell_fault_info.cell_voltage * 0.0001f, cell_fault_info.cell_idx);
    dbg_printf("\n\r############## BMS[%d] UV INFO ##############\n\r", bus); 
    dbg_printf("\n\r");
    
    batt_ctx[bus].bms_data.dsg_uv_cell_index = cell_fault_info.cell_idx;
    batt_ctx[bus].bms_data.dsg_uv_cell_value = cell_fault_info.cell_voltage * 0.0001f;
}

static void process_CAN_IF_MSG_BMS_CHG_OV_INFO_ID(const uint8_t *buffer, uint32_t bus)
{
    volatile ov_uv_cell_info_t cell_fault_info;
    
    (void)memcpy((uint8_t *)&cell_fault_info, buffer, sizeof(ov_uv_cell_info_t));
    
    dbg_printf("\n\r############## BMS[%d] OV INFO ##############\n\r", bus);
    dbg_printf("I,%d,CHG OV Info: %3.3f V | Cell %d\n\r", bus, cell_fault_info.cell_voltage * 0.0001f, cell_fault_info.cell_idx);
    dbg_printf("\n\r############## BMS[%d] OV INFO ##############\n\r", bus); 
    dbg_printf("\n\r");
    
    batt_ctx[bus].bms_data.chg_uv_cell_index = cell_fault_info.cell_idx;
    batt_ctx[bus].bms_data.chg_uv_cell_value = cell_fault_info.cell_voltage * 0.0001f;
}

static void process_CAN_IF_MSG_BMS_PEAK_DSG_I_INFO_ID(const uint8_t *buffer, uint32_t bus)
{
    struct peak_currents_t {
        float_t ltc_peak;
        float_t ltc_hw_peak;
        float_t ina_peak;
    }peak_currents;
    
    (void)memcpy((uint8_t *)&peak_currents, buffer, sizeof(struct peak_currents_t));
    
    dbg_printf("\n\r############## BMS[%d] PEAK CURRENT INFO ##############\n\r", bus);
    dbg_printf("I,%d,Peak Currents,%3.3f,%3.3f,%3.3f\n\r", bus, peak_currents.ltc_peak, peak_currents.ltc_hw_peak, peak_currents.ina_peak);
    dbg_printf("\n\r############## BMS[%d] PEAK CURRENT INFO ##############\n\r", bus); 
    dbg_printf("\n\r");
    
    batt_ctx[bus].bms_data.peak_currents[0] = peak_currents.ltc_peak;
    batt_ctx[bus].bms_data.peak_currents[1] = peak_currents.ltc_hw_peak;
    batt_ctx[bus].bms_data.peak_currents[2] = peak_currents.ina_peak;
}

static void process_CAN_IF_MSG_BF_BMS_BAL_CELL_INFO_ID(const uint8_t *buffer, uint32_t bus)
{
    (void)memcpy(batt_ctx[bus].bms_balancer_ctx.cv_before_balancing, buffer, (CELL_CHANNELS * sizeof(uint16_t)));
    
    dbg_printf("I,%d,CV Before Balancing: ", bus);
    for(uint32_t i = 0U; i < CELL_CHANNELS; i++)
    {
        dbg_printf("%3.3f ", batt_ctx[bus].bms_balancer_ctx.cv_before_balancing[i] * 0.0001f);
    }        
    dbg_printf("\n\r");

}

static void process_CAN_IF_MSG_AF_BMS_BAL_CELL_INFO_ID(const uint8_t *buffer, uint32_t bus)
{
    (void)memcpy(batt_ctx[bus].bms_balancer_ctx.cv_after_balancing, buffer, (CELL_CHANNELS * sizeof(uint16_t)));

    dbg_printf("I,%d,CV After Balancing:  ", bus);
    for(uint32_t i = 0U; i < CELL_CHANNELS; i++)
    {
        dbg_printf("%3.3f ", batt_ctx[bus].bms_balancer_ctx.cv_after_balancing[i] * 0.0001f);
    }        
    dbg_printf("\n\r");
    dbg_printf("\n\r");
}

static void process_CAN_IF_MSG_NTF_INIT_COMPLETE_ID(const uint8_t *buffer, uint32_t bus)
{
    UNUSED_PARAM(buffer);
    
    canfd_queue_desc_t can_message;
#ifndef USE_FEATURE_VCU_ON_DESK
    if(batt_ctx[bus].status == 0U)
    {
        /* Turn FETs ON */
        can_message.is_cyclic = 0;
        can_message.cycle_count = 0;
        can_message.msg_id = CAN_IF_MSG_DSG_CHG_FET_ON; 
        
        canfd_enqueue(can_message, (uint8_t)bus);
    }
    
    init_boot_mesg();
#endif /* USE_FEATURE_VCU_ON_DESK */
    
}

static void process_CAN_IF_MSG_BMS_PROC_EXEC_ERR_ID(const uint8_t *buffer, uint32_t bus)
{
    UNUSED_PARAM(bus);
    
    uint32_t message_id = 0U;
    
    (void)memcpy((uint8_t *)&message_id, buffer, 4U);
    set_status_bit(STAT_VCU_CAN_MSG_EXEC_ERR);
    
    dbg_printf("E,%d,Message exec fail: ID -> 0x%x\n\r", bus, message_id);
}

static void process_CAN_IF_MSG_BMS_FW_UPD_MSG_ID(const uint8_t *buffer, uint32_t bus)
{
    static uint32_t num_block_transfers = 0U;
    dba_msg_queue_obj_t dmq;
    canfd_queue_desc_t can_message;
    
    UNUSED_PARAM(buffer);
    UNUSED_PARAM(bus);

    num_block_transfers++;
    if(num_block_transfers >= (S32_BMS_FOTA_DL_SIZE / BMS_FOTA_BLOCK_SIZE))
    {
        dmq.msg_id = DBA_SEND_DISPLAY_MSG;
        dmq.data[0] = DBA_DISPLAY_TELL_TALE_CTRL_MSG;
        dmq.data[1] = TT_SERVICE_INDICATOR;
        dmq.data[2] = DBA_DISPLAY_TELL_TALE_OFF;
        (void)osif_msg_queue_send(dba_msg_queue, &dmq, 0U, 0U);
        
        can_message.is_cyclic = 0;
        can_message.cycle_count = 0;
        can_message.msg_id = CAN_IF_MSG_BMS_FW_UPD_DONE_MSG;
        
        (void)canfd_enqueue(can_message, CANFD_LOGICAL_BUS0); 
    }
    else
    {
        can_message.is_cyclic = 0;
        can_message.cycle_count = 0;
        can_message.msg_id = CAN_IF_MSG_BMS_FW_UPD_MSG;
        
        (void)canfd_enqueue(can_message, CANFD_LOGICAL_BUS0);   
    }
}

static void process_CAN_IF_MSG_BMS_FW_UPD_DONE_MSG_ID(const uint8_t *buffer, uint32_t bus)
{
    /* TBD */
    UNUSED_PARAM(buffer);
    UNUSED_PARAM(bus);
}

static void process_CAN_IF_MSG_BATTERY_PACK_INFO_MSG_ID(const uint8_t *buffer, uint32_t bus)
{
    UNUSED_PARAM(bus);
    
    (void)memcpy((uint8_t *)&battery_info, buffer, sizeof(bms_info_t));
    
    dbg_printf("I,%d,%d Ah, %dS%dP, %d, %d-%c, %d mAh\n\r", 
                    bus,
                    battery_info.theoretical_capacity, 
                    battery_info.num_cells_in_series, 
                    battery_info.num_cells_in_parallel,  
                    battery_info.battery_chemistry,
                    battery_info.cell_manufacturer & 0x0FU,
                    battery_info.range_type == 0U ? 'L': 'H',
                    battery_info.nominal_ah * 100 
    );
}

static void process_CAN_IF_MSG_EXCP_NTF_MSG_ID(const uint8_t *buffer, uint32_t bus)
{
    UNUSED_PARAM(bus);
    
    typedef struct {
        uint32_t lr;
        uint32_t pc;
        uint32_t acc_type;
        uint32_t addr;
        uint32_t dfsr;
        uint32_t afsr;
    }can_ex_msg_t;
    
    volatile can_ex_msg_t c;
    
    mc_set_gear(MC_GEAR_POS_NEUTRAL);
    (void)memcpy((uint8_t *)&c, &buffer[4], sizeof(can_ex_msg_t));
    
    dbg_printf("E,%d,Exception Info: LR=0x%x, PC=0x%x, MPU access=0x%x\n\r",
                    bus, c.lr, c.pc, c.acc_type);
    
    set_status_bit(STAT_VCU_BMS_SW_EXCEPTION);
}

static void process_CAN_IF_MSG_BMS_GET_EC_AT_BOOT(const uint8_t *buffer, uint32_t bus)
{
    UNUSED_PARAM(bus);
    
    ltc2946_boot_chg_eng_t *msg = NULL;
    
    msg = (ltc2946_boot_chg_eng_t *)buffer;
    
    boot_chg_eng.charge[0] = msg->charge[0];
    boot_chg_eng.charge[1] = msg->charge[1];
    
    boot_chg_eng.energy[0] = msg->energy[0];
    boot_chg_eng.energy[1] = msg->energy[1];

#if 0
    update_trip_watthrs(boot_chg_eng.energy[0]);
#endif
    
}

void can_fd_if_bms_err_process(uint32_t *esr)
{
    static uint32_t fault_confinement_count = 0U;
    
    batt_ctx[CANFD_LOGICAL_BUS0].bms_data.can_esr = *esr;
    
    if((*esr & CAN_ESR1_BOFFINT_MASK))
    {
        dbg_printf("E,CAN I/F BUS OFF ERROR\n\r");
    }
    else if((*esr & CAN_ESR1_FLTCONF_MASK))
    {
        fault_confinement_count++;
        if(fault_confinement_count < 10U)
        {
            dbg_printf("E,CAN I/F Fault confinement state\n\r");
        }
        else if(fault_confinement_count == 10U)
        {
            dbg_printf("E,CAN I/F Fault confinement state overflow\n\r");
        }
        else
        {
            __NOP();
        }
    }
    else if((*esr & CAN_ESR1_RXWRN_MASK))
    {
        dbg_printf("E,CAN I/F Repetitive RX errors\n\r");
    }
    else if((*esr & CAN_ESR1_TXWRN_MASK))
    {
        dbg_printf("E,CAN I/F Repetitive TX errors\n\r");
    }
    else if((*esr & CAN_ESR1_STFERR_MASK))
    {
        dbg_printf("E,CAN I/F Stuff error\n\r");
    }
    else if((*esr & CAN_ESR1_FRMERR_MASK))
    {
        dbg_printf("E,CAN I/F Form error\n\r");
    }
    else if((*esr & CAN_ESR1_CRCERR_MASK))
    {
        dbg_printf("E,CAN I/F CRC error\n\r");
    }
    else if((*esr & CAN_ESR1_ACKERR_MASK))
    {
        dbg_printf("E,CAN I/F ACK error\n\r");
    }
    else if((*esr & CAN_ESR1_BIT0ERR_MASK))
    {
        dbg_printf("E,CAN I/F BIT0 error\n\r");
    }
    else if((*esr & CAN_ESR1_BIT1ERR_MASK))
    {
        dbg_printf("E,CAN I/F BIT1 error\n\r");
    }
    else if((*esr & CAN_ESR1_SYNCH_MASK))
    {
        dbg_printf("E,CAN I/F SYNCH errors\n\r");
    }
    else
    {
        __NOP();
    }
    
    *esr = 0U;
}

static void bms_calc_min_max_cv(uint32_t bus, float_t *min_cv, float_t *max_cv)
{
    uint16_t cv[CELL_CHANNELS];
    
	for(int32_t i = 0; i < CELL_CHANNELS; i++)
	{
		cv[i] = batt_ctx[bus].bms_data.LTC6812.cv[i];
	}
	
    insertion_sort(cv, CELL_CHANNELS);
    
    *min_cv = cv[0] * 0.0001f;
    *max_cv = cv[CELL_CHANNELS - 1U] * 0.0001f;
}

static void send_bms_data(uint32_t bus)
{
    uint32_t statush = 0U;
    uint32_t statusl = 0U;
    uint64_t vcu_stat = 0U;
    
    int32_t lc = 0;
    datetime_t dt;
    shmem_block_bms_t *msg = NULL;
    udp_msg_queue_obj_t qmsg;
    
    volatile uint32_t actual_speed = 0U;
    volatile uint32_t indicated_speed = 0U;
    volatile float_t distance = 0.0f;
    volatile float_t tv = 0.0f;
    volatile uint16_t range = 0U;
    volatile float_t wh_per_km = 0.0f;
    volatile float_t wh_per_km_rg = 0.0f;

    statusl = (uint32_t)(batt_ctx[bus].status & (uint64_t)0x00000000ffffffff);
    statush = (uint32_t)((uint64_t)(batt_ctx[bus].status & (uint64_t)0xffffffff00000000) >> 32U);

    vcu_stat = get_status();
    
    hdr.bms_id = batt_ctx[bus].uid;
    hdr.statush = statush;
    hdr.statusl = statusl;
    hdr.millis = osif_millis();
    hdr.bus = (uint8_t)bus;
    hdr.vcu_statush = (uint32_t)((uint64_t)(vcu_stat & (uint64_t)0xffffffff00000000) >> 32U);
    hdr.vcu_statusl = (uint32_t)(vcu_stat & (uint64_t)0x00000000ffffffff);
    
    actual_speed = mc_get_vehicle_speed();
    
    /* 
        IS 11827 requirement
        Indicated speed = Actual Speed + (0.1 * Actual Speed) + 8 Km/h
    */
    if(actual_speed > 8U)
    {
        indicated_speed = actual_speed + (uint32_t)((0.1f * actual_speed));
    }
    else
    {
        indicated_speed = actual_speed;
    }
    
    distance = odo_get_trip_distance();
    lsm_throttle_2_voltage((float_t *)&tv);
    hdr.odometer = odo_get_distance();
    
    hdr.roll = (int32_t)imu_get_roll();
    hdr.pitch = (int32_t)imu_get_pitch();
    
    (void)memcpy(&hdr.throttle_voltage[0], (uint8_t *)&tv, sizeof(tv));
    (void)memcpy(&hdr.speed[0], (uint8_t *)&indicated_speed, sizeof(indicated_speed));
    (void)memcpy(&hdr.distance[0], (uint8_t *)&distance, sizeof(distance));
    
    update_trip_watthrs(batt_ctx[bus].bms_data.LTC2946[0].energy, batt_ctx[bus].bms_data.LTC2946[1].energy);
    
    odo_get_wh_quantized((float_t *)&wh_per_km);
    odo_get_wh_rg_quantized((float_t *)&wh_per_km_rg);
    
    hdr.wh_per_km = wh_per_km;
    hdr.wh_per_km_regen = wh_per_km_rg;
    odo_get_range(&hdr.range);
    (void)memcpy(&hdr.actual_speed[0], (uint8_t *)&actual_speed, sizeof(actual_speed));
    
    hdr.available_modes = mc_get_available_ride_modes();
    hdr.current_ride_mode = mc_get_ride_mode();
    hdr.vehicle_range_type = (uint32_t)bms_get_range_type();
    
    rtc_read_local_time(&dt);
    (void)memcpy(&hdr.rtc[0], (uint8_t *)&dt, sizeof(datetime_t));
    
    chg_get_charger_context(&charger_ctxt);

    cd1030_get_internal_faults(&aux_info.swif_internal_faults);
    dba_get_als_value(&aux_info.als_lux);
        
    /* Send To EC25 */
    msg = (shmem_block_bms_t *)shmem_alloc_block(SHMEM_POOL_TYPE_BMS);
    if(msg != NULL)
    {
        DEV_ASSERT((sizeof(vcu_info_msg_t) + sizeof(bms_meas_t) + sizeof(charger_ctx_t) + sizeof(auxiliary_info_t)) < SHMEM_BMS_BLK_SIZE);
        
        msg->udp_msg.len = sizeof(vcu_info_msg_t) + sizeof(bms_meas_t) + sizeof(charger_ctx_t) + sizeof(auxiliary_info_t);
        
        (void)memcpy(msg->buffer, (uint8_t *)&hdr, sizeof(vcu_info_msg_t));
        (void)memcpy(&msg->buffer[0U + sizeof(vcu_info_msg_t)], (uint8_t *)&batt_ctx[bus].bms_data, sizeof(bms_meas_t));
        (void)memcpy(&msg->buffer[0U + sizeof(vcu_info_msg_t) + sizeof(bms_meas_t)], (uint8_t *)&charger_ctxt, sizeof(charger_ctx_t));
        (void)memcpy(&msg->buffer[0U + sizeof(vcu_info_msg_t) + sizeof(bms_meas_t) + sizeof(charger_ctx_t)], (uint8_t *)&aux_info, sizeof(auxiliary_info_t));
        
        msg->udp_msg.cmd = MSG_ID_DBG_S32_LTE;
        
        qmsg.msg = (void *)msg;
        qmsg.msg_id = msg->udp_msg.cmd;
        qmsg.msg_len = msg->udp_msg.len;       
            
        (void)osif_msg_queue_send(udp_msg_queue, &qmsg, 0U, 0U);

        imx_dbg_msg.pack_voltage = batt_ctx[bus].bms_data.LTC6812.soc * 30 * 100e-6f;
        imx_dbg_msg.max_cell_temperature = get_max_cell_temp(bus);
        imx_dbg_msg.fet_temperature = batt_ctx[bus].bms_data.sensor_values[0];
        mc_get_motor_temperature(&imx_dbg_msg.motor_temperature);
        mc_get_heatsink_temperature(&imx_dbg_msg.motor_heatsink_temperature);
        mc_get_shaft_rpm(&imx_dbg_msg.shaft_rpm);
        bms_calc_min_max_cv(bus, &imx_dbg_msg.min_cell_voltage, &imx_dbg_msg.max_cell_voltage);
        
        if(batt_ctx[bus].bms_data.LTC2946[0].current > batt_ctx[bus].bms_data.LTC2946[LTC2946_CHG_CHANNEL].current)
        {
            imx_dbg_msg.pack_current = batt_ctx[bus].bms_data.LTC2946[LTC2946_DSG_CHANNEL].current;
        }
        else
        {
            imx_dbg_msg.pack_current = (-1) * batt_ctx[bus].bms_data.LTC2946[LTC2946_CHG_CHANNEL].current;
        }
        
        imx_dbg_msg.soc = (uint32_t)batt_ctx[bus].bms_data.m.soc;  
        
#ifdef USE_RANGE_PRED_ALG0
        odo_get_range((uint16_t *)&range);
        imx_dbg_msg.soc = imx_dbg_msg.soc & 0x000000FFU;
        imx_dbg_msg.soc = imx_dbg_msg.soc | (uint32_t)(range << 8U);
#endif /* USE_RANGE_PRED_ALG0 */
        
        lc = osif_enter_critical();
        imx_dbg_msg.available_modes = mc_get_available_ride_modes();
        (void)osif_exit_critical(lc);
        
        bms_send_imx_dbg_msg(2);

        hdr.msg_sequence++;
    }
}

static void bms_send_imx_dbg_msg(uint32_t msg_prio)
{
    shmem_block_bms_t *msg = NULL;
    udp_msg_queue_obj_t qmsg;
    imu_data_t imu_data;

    for(uint32_t i = 0U; i < 3U; i++)
    {
        odo_get_trip_info(&trip_meters.trip[i], i);
    }

    imu_get_data(&imu_data);
    
    /* Send To i.MX */
    msg = (shmem_block_bms_t *)shmem_alloc_block(SHMEM_POOL_TYPE_BMS);
    
    if(msg != NULL)
    {
        msg->udp_msg.len = sizeof(vcu_info_msg_t) + sizeof(imx_dbg_msg_t) + sizeof(trip_meter_disp_t) + sizeof(imu_data_t);
        (void)memcpy(msg->buffer, (uint8_t *)&hdr, sizeof(vcu_info_msg_t));
        (void)memcpy(&msg->buffer[0U + sizeof(vcu_info_msg_t)], (uint8_t *)&imx_dbg_msg, sizeof(imx_dbg_msg_t));
        (void)memcpy(&msg->buffer[0U + sizeof(vcu_info_msg_t) + sizeof(imx_dbg_msg_t)], (uint8_t *)&trip_meters, sizeof(trip_meter_disp_t));
        (void)memcpy(&msg->buffer[0U + sizeof(vcu_info_msg_t) + sizeof(imx_dbg_msg_t) + sizeof(trip_meter_disp_t)], (uint8_t *)&imu_data, sizeof(imu_data_t));
        msg->udp_msg.cmd = MSG_ID_DBG_AUX_S32_IMX;
        
        qmsg.msg = (void *)msg;
        qmsg.msg_id = msg->udp_msg.cmd;
        qmsg.msg_len = msg->udp_msg.len;
            
        (void)osif_msg_queue_send(udp_msg_queue, &qmsg, (uint8_t)msg_prio, 0U);    
    }
}

static void vcu_update_id(void)
{
    uint8_t uids[16];
    
    uids[0] = (uint8_t)(SIM->UIDL & 0x000000FFU);
    uids[1] = (uint8_t)((SIM->UIDL & 0x0000FF00U) >> 8U);
    uids[2] = (uint8_t)((SIM->UIDL & 0x00FF0000U) >> 16U);
    uids[3] = (uint8_t)((SIM->UIDL & 0xFF000000U) >> 24U);

    uids[4] = (uint8_t)(SIM->UIDML & 0x000000FFU);
    uids[5] = (uint8_t)((SIM->UIDML & 0x0000FF00U) >> 8U);
    uids[6] = (uint8_t)((SIM->UIDML & 0x00FF0000U) >> 16U);
    uids[7] = (uint8_t)((SIM->UIDML & 0xFF000000U) >> 24U);
    
    uids[8] = (uint8_t)(SIM->UIDMH & 0x000000FFU);
    uids[9] = (uint8_t)((SIM->UIDMH & 0x0000FF00U) >> 8U);
    uids[10] = (uint8_t)((SIM->UIDMH & 0x00FF0000U) >> 16U);
    uids[11] = (uint8_t)((SIM->UIDMH & 0xFF000000U) >> 24U);
    
    uids[12] = (uint8_t)(SIM->UIDH & 0x000000FFU);
    uids[13] = (uint8_t)((SIM->UIDH & 0x0000FF00U) >> 8U);
    uids[14] = (uint8_t)((SIM->UIDH & 0x00FF0000U) >> 16U);
    uids[15] = (uint8_t)((SIM->UIDH & 0xFF000000U) >> 24U);
    
    aes_cmacl_can_bms_vcu(uids, 16U, &hdr.vcu_id[0]); 
}

static void vcu_update_api_version(void)
{
    volatile uint32_t ota_sentinel = 0U;

    nvm_read(FILE_SECT_BOOT_CONFIGURATION + FILE_SECT_BOOT_FW_UPD_OFFSET, (uint8_t *)&ota_sentinel, sizeof(uint32_t));
    
    hdr.api_version[0] = LOG_API_VERSION;
    hdr.api_version[1] =  get_fw_max_ver();
    hdr.api_version[2] =  get_fw_min_ver();
    
    if(ota_sentinel == 0U)
    {
        hdr.api_version[3] = (uint8_t)'P';
    }
    else
    {
        hdr.api_version[3] = (uint8_t)'F';
    }
}

static void process_wake(const uint8_t *buffer)
{
    static uint32_t state = 0U;
    canfd_queue_desc_t can_message;
    
    UNUSED_PARAM(buffer);
    
    initial_sentinel = 0U;
    
    switch(state)
    {
        case 0U:
            (void)osif_thread_set_flag(bms_task_get_id(), BMS_TASK_WAKE_NTF_FLAG);
            state = 1U;
            break;
        
        case 1U:
            /* Unsolicited wakeup. Probable BMS reset */
            /* Retrieve reset cause */
        
            can_message.is_cyclic = 0;
            can_message.cycle_count = 0;
            can_message.msg_id = CAN_IF_MSG_GET_LAST_RST_STATE; 
            canfd_enqueue(can_message, (uint8_t)0U);
            break;
        
        default:
            __NOP();
            break;
    }    
}

void can_enqueue_msg(uint32_t bus)
{
    (void)canfd_enqueue(canfd_expected_rx[bus], (uint8_t)bus);
}

void fet_sw_init(void)
{
    batt_ctx[0U].battery_cfet_state = MOSFET_OFF_SW_STATE;
    batt_ctx[0U].battery_dfet_state = MOSFET_OFF_SW_STATE;
}

float_t bms_get_min_cell_voltage(uint32_t bus)
{
    return batt_ctx[bus].bms_data.LTC6812.min_cv * 0.0001f;
}

float_t bms_get_max_temp(uint32_t bus)
{
    return  get_max_cell_temp(bus);
}

float_t bms_get_soc(uint32_t bus)
{
    return batt_ctx[bus].bms_data.m.soc;
}

void bms_get_tte(uint32_t bus, uint32_t *tte)
{
    static uint32_t tte_hold = 0U;
    
    if(batt_ctx[bus].bms_data.m.tte < 368634.38)
    {
        *tte = (uint32_t)batt_ctx[bus].bms_data.m.tte;
        tte_hold = *tte;
    }
    else
    {
        *tte = tte_hold;
    }
}

void bms_get_avcap(uint32_t bus, float_t *avcap)
{
    *avcap = (uint32_t)batt_ctx[bus].bms_data.m.repcap;
}

void bms_get_pack_ocv(uint32_t bus, float_t *pack_estimated_ocv)
{
    *pack_estimated_ocv = batt_ctx[bus].bms_data.m.voltage * CELL_CHANNELS;
}

void bms_get_dsg_energy_wh(uint32_t bus, float_t *dsg_energy)
{
    if(dsg_energy != NULL)
    {
        *dsg_energy = batt_ctx[bus].bms_data.LTC2946[0].energy / 3600U;
    }
}

void bms_get_chg_energy_wh(uint32_t bus, float_t *chg_energy)
{
    if(chg_energy != NULL)
    {
        *chg_energy = batt_ctx[bus].bms_data.LTC2946[1].energy / 3600U;
    }
}

void bms_get_chg_ah(uint32_t bus, uint32_t *chg_ah)
{
    if(chg_ah != NULL)
    {
        *chg_ah = batt_ctx[bus].bms_data.LTC2946[1].charge / 3600U;
    }  
}

uint32_t get_dsg_fet_state(uint32_t bus)
{
    return batt_ctx[bus].battery_dfet_state;
}

uint32_t get_chg_fet_state(uint32_t bus)
{
    return batt_ctx[bus].battery_cfet_state;
}

void bms_get_vi(float_t *v, float_t *i, ltc2946_channels_e ch)
{
    DEV_ASSERT(ch < MAX_LTC_CHANNELS);
    
    if(v != NULL)
    {
        *v = batt_ctx[canfd_logical_bus_id].bms_data.LTC2946[ch].adin;
    }
    
    if(i != NULL)
    {
        *i = batt_ctx[canfd_logical_bus_id].bms_data.LTC2946[ch].current;
    }
}

void bms_get_voltage_cm(float_t *v)
{
    if(v != NULL)
    {
        *v = batt_ctx[canfd_logical_bus_id].bms_data.LTC6812.soc * 30 * 100e-6;
    }
}

void bms_get_current(float_t *i, ltc2946_channels_e ch)
{
    DEV_ASSERT(ch < MAX_LTC_CHANNELS);
    
    if(i != NULL)
    {
        *i = batt_ctx[canfd_logical_bus_id].bms_data.LTC2946[ch].current;
    }
}

void bms_get_max_cell_temperature(float_t *cell_t)
{
    *cell_t = get_max_cell_temp(0U);
}

void bms_get_design_capacity(uint8_t *design_cap)
{
    *design_cap = battery_info.theoretical_capacity;
}

char *bms_get_fw_version(void)
{
    return batt_ctx[canfd_logical_bus_id].fw_version;
}

void bms_get_balancer_info(balancer_run_info_t *bal_info)
{
    (void)memcpy(bal_info, &batt_ctx[canfd_logical_bus_id].bms_balancer_ctx, sizeof(balancer_run_info_t));
}

void bms_create_tx_timer(void)
{
    data_tx_timer = osif_timer_create(bms_data_tx_timer_handler, osTimerPeriodic, NULL, NULL);
}

void bms_data_tx_timer_start(uint32_t duration)
{
    if(osTimerIsRunning(data_tx_timer) == 0U)
    {
        (void)osif_timer_start(data_tx_timer, duration);
    }
}

uint32_t bms_get_variant_info(void)
{
    return battery_info.cell_manufacturer;
}

uint8_t bms_get_range_type(void)
{
    return battery_info.range_type;
}


void bms_data_tx_timer_stop(void)
{
    if(osTimerIsRunning(data_tx_timer) == 1U)
    {
        (void)osif_timer_stop(data_tx_timer);
    }    
}

void bms_fg_reset(void)
{
    canfd_queue_desc_t can_message;
    
    /* Hard reset the fuelgauge */
    can_message.is_cyclic = 0;
    can_message.cycle_count = 0;
    can_message.msg_id = CAN_IF_MSG_HW_RST_FG; 
    
    (void)canfd_enqueue(can_message, 0U);     
}

/*  
    pragma to suppress warning due to float to double promotion
    This is observed to happen only where dbg_printf is being used and
    not during any computation. Hence suppressed in this module.
*/
#pragma clang diagnostic ignored "-Wdouble-promotion"

void debug_bms_data(uint32_t bus)
{
    uint32_t millis = 0U;
    uint8_t alert = 0U;
    
    millis = OSIF_GetMilliseconds();
    
    meas = &batt_ctx[bus].bms_data;
    
    dbg_printf("I,%d,SL | SH: 0x%x | 0x%x\n\r", bus, (uint32_t)(batt_ctx[bus].status & 0x00000000FFFFFFFFU), (uint32_t)(batt_ctx[bus].status >> 32U));
    
    dbg_printf("I,%d,FET TMP = %3.2f\n\r", bus, batt_ctx[bus].bms_data.sensor_values[0]);

#ifdef USE_CALC_CELL_DELTA_VOLTAGE
    process_cell_delta(bus);
    dbg_printf("I,%d,Delta-V = %3.4f V\n\r", bus, cell_deltav);
#endif
#if 1
    dbg_printf("[%d,%d,%d,%d,%3.2f,"
    
                "%3.3f,%3.4f,%3.3f,%3.3f,%3.3f,%3.3f,%3.3f,"
                "%f,%f,"
                "%3.3f,%3.3f,%3.3f,%3.3f,%3.3f,%3.3f,%3.3f,%3.3f,%3.3f,%3.3f,%3.3f,%3.3f,%3.3f,%3.3f,"
                "%3.4f,"
                
                "%3.4f,"
                "%3.2f,"
                "%3.2f,"
                "%3.2f,"
                "%3.2f,"
                
                "%3.3f,%d]\n\r",
                
                (uint8_t)bus,
                millis,
                (batt_ctx[bus].battery_dfet_state == MOSFET_ON_SW_STATE) ? 1 : 0,
                batt_ctx[bus].is_balancer_running, 
                batt_ctx[bus].bms_data.LTC6812.mosfet_temp,
                
                batt_ctx[bus].bms_data.LTC2946[LTC2946_DSG_CHANNEL].adin,
                batt_ctx[bus].bms_data.LTC2946[LTC2946_DSG_CHANNEL].sc_current,
                batt_ctx[bus].bms_data.LTC2946[LTC2946_DSG_CHANNEL].current,
                batt_ctx[bus].bms_data.LTC2946[LTC2946_CHG_CHANNEL].adin,
                batt_ctx[bus].bms_data.LTC2946[LTC2946_CHG_CHANNEL].current,
                batt_ctx[bus].bms_data.LTC2946[LTC2946_DSG_CHANNEL].time,
                batt_ctx[bus].bms_data.LTC2946[LTC2946_CHG_CHANNEL].time,
                
                batt_ctx[bus].bms_data.LTC2946[LTC2946_DSG_CHANNEL].charge,
                batt_ctx[bus].bms_data.LTC2946[LTC2946_CHG_CHANNEL].charge,

                batt_ctx[bus].bms_data.LTC6812.cv[0] * 0.0001f,
                batt_ctx[bus].bms_data.LTC6812.cv[1] * 0.0001f,
                batt_ctx[bus].bms_data.LTC6812.cv[2] * 0.0001f,
                batt_ctx[bus].bms_data.LTC6812.cv[3] * 0.0001f,
                batt_ctx[bus].bms_data.LTC6812.cv[4] * 0.0001f,
                batt_ctx[bus].bms_data.LTC6812.cv[5] * 0.0001f,
                batt_ctx[bus].bms_data.LTC6812.cv[6] * 0.0001f,
                batt_ctx[bus].bms_data.LTC6812.cv[7] * 0.0001f,
                batt_ctx[bus].bms_data.LTC6812.cv[8] * 0.0001f,
                batt_ctx[bus].bms_data.LTC6812.cv[9] * 0.0001f,
                batt_ctx[bus].bms_data.LTC6812.cv[10] * 0.0001f,
                batt_ctx[bus].bms_data.LTC6812.cv[11] * 0.0001f,
                batt_ctx[bus].bms_data.LTC6812.cv[12] * 0.0001f,
                batt_ctx[bus].bms_data.LTC6812.cv[13] * 0.0001f,
                
                cell_deltav,
                
                batt_ctx[bus].bms_data.LTC6812.soc * 30 * 100e-6,
                batt_ctx[bus].bms_data.LTC2946[0].power, 
                batt_ctx[bus].bms_data.LTC2946[0].energy,
                batt_ctx[bus].bms_data.LTC2946[1].power, 
                batt_ctx[bus].bms_data.LTC2946[1].energy,
                
                batt_ctx[bus].bms_data.LTC6812.min_cv * 0.0001f,
                batt_ctx[bus].bms_data.LTC6812.cd_bitmap
              ); 
              
     dbg_printf("{%d,%d,%3.2f,%3.2f,%3.2f,%3.2f,%3.2f,%3.2f,%3.2f,%3.2f,%3.2f,%3.2f,%3.2f,%3.2f,%3.2f,%3.2f,%3.2f,%3.2f,%3.2f,%3.2f}\n\r",
                    (uint8_t)bus,
                    millis,
                    batt_ctx[bus].bms_data.LTC6812.t[0] * 0.01f,      /* TS1 */
                    batt_ctx[bus].bms_data.LTC6812.t[1] * 0.01f,      /* TS2 */
                    batt_ctx[bus].bms_data.LTC6812.t[2] * 0.01f,      /* TS3 */
                    batt_ctx[bus].bms_data.LTC6812.t[3] * 0.01f,      /* TS4 */
                    batt_ctx[bus].bms_data.LTC6812.t[4] * 0.01f,      /* TS5 */
                    batt_ctx[bus].bms_data.LTC6812.t[5] * 0.01f,      /* TS6 */
                    batt_ctx[bus].bms_data.LTC6812.t[6] * 0.01f,      /* TS7 */
                    batt_ctx[bus].bms_data.LTC6812.t[7] * 0.01f,      /* TS8 */
                    batt_ctx[bus].bms_data.LTC6812.t[8] * 0.01f,      /* TS9 */
                    batt_ctx[bus].bms_data.LTC6812.t[9] * 0.01f,      /* TS10 */
                
                    batt_ctx[bus].bms_data.LTC6812.t[10] * 0.01f,     /* TS11 */
                    batt_ctx[bus].bms_data.LTC6812.t[11] * 0.01f,     /* TS12 */
                    batt_ctx[bus].bms_data.LTC6812.t[12] * 0.01f,     /* FET FRONT */
                    batt_ctx[bus].bms_data.LTC6812.t[13] * 0.01f,     /* BAT+ */
                    batt_ctx[bus].bms_data.LTC6812.t[14] * 0.01f,     /* BAT1 */
                    batt_ctx[bus].bms_data.LTC6812.t[15] * 0.01f,     /* PACK+ */
                    
                    batt_ctx[bus].bms_data.LTC6812.t[16] * 0.01f,     /* TS0_FLT */
                    batt_ctx[bus].bms_data.LTC6812.t[17] * 0.01f      /* TS13_FLT */
               );
               
    dbg_printf("(%d,%d,%3.2f,%3.2f,%3.2f,%3.2f,%3.2f,%3.4f,%3.2f,%3.2f,%3.2f,%3.2f,%3.2f,%3.2f,%3.2f,%3.2f,%3.2f,%d,%d,%d)\n\r",
                    (uint8_t)bus,
                    millis,
                    batt_ctx[bus].bms_data.sensor_values[0],  /*SYS_SENSOR_TMP_MOSFET_TMP*/
                    batt_ctx[bus].bms_data.sensor_values[1],  /*SYS_SENSOR_VTG_DSG_INA302_VOUT*/
                    batt_ctx[bus].bms_data.sensor_values[2],  /*SYS_SENSOR_TMP_BAL_RES_TMP*/
                    batt_ctx[bus].bms_data.sensor_values[3],  /*SYS_SENSOR_HMD_SNS_MCU_HUM*/
                    batt_ctx[bus].bms_data.sensor_values[4],  /*SYS_SENSOR_VTG_LTC7103_IMON*/
                    batt_ctx[bus].bms_data.sensor_values[5],  /*SYS_SENSOR_VTG_BP_SLOT_DETECT*/

                    batt_ctx[bus].bms_data.m.voltage,
                    batt_ctx[bus].bms_data.m.vbatt,
                    batt_ctx[bus].bms_data.m.avg_current,
                    batt_ctx[bus].bms_data.m.soc,
                    batt_ctx[bus].bms_data.m.tte,
                    batt_ctx[bus].bms_data.m.ttf,
                    batt_ctx[bus].bms_data.m.repcap,
                    batt_ctx[bus].bms_data.m.ts,
                    batt_ctx[bus].bms_data.m.internal_resistance,
                    batt_ctx[bus].bms_data.m.num_cycles,
                    batt_ctx[bus].bms_data.m.designCapacity,
                    batt_ctx[bus].bms_data.m.fstat
               );
#endif /* 0 */

    if(alert_flag == 1U)
    {
        alert_flag = 0U;
        alert = batt_ctx[bus].bms_data.LTC2946[LTC2946_DSG_CHANNEL].alert;
        
        dbg_printf("I,Fault Information of pack %d\n\r", bus);       
        if(alert & (1 << LTC2946_ALERT_MIN_ADIN))
        {
            dbg_printf("W,Alert LTC2946_DSG_CHANNEL: LTC2946_ALERT_MIN_ADIN\n\r");
        }else if(alert & (1 << LTC2946_ALERT_MIN_ISENSE))
        {
            dbg_printf("W,Alert LTC2946_DSG_CHANNEL: LTC2946_ALERT_MIN_ISENSE\n\r");
        }else if(alert & (1 << LTC2946_ALERT_MAX_ADIN))
        {
            dbg_printf("W,Alert LTC2946_DSG_CHANNEL: LTC2946_ALERT_MAX_ADIN\n\r");
        }else if(alert & (1 << LTC2946_ALERT_MIN_VIN))
        {
            dbg_printf("W,Alert LTC2946_DSG_CHANNEL: LTC2946_ALERT_MIN_VIN\n\r");
        }else if(alert & (1 << LTC2946_ALERT_MAX_VIN))
        {
            dbg_printf("W,Alert LTC2946_DSG_CHANNEL: LTC2946_ALERT_MAX_VIN\n\r");
        }else if(alert & (1 << LTC2946_ALERT_MAX_ISENSE))
        {
            dbg_printf("W,Alert LTC2946_DSG_CHANNEL: LTC2946_ALERT_MAX_ISENSE\n\r");
        }else if(alert & (1 << LTC2946_ALERT_MIN_PWR))
        {
            dbg_printf("W,Alert LTC2946_DSG_CHANNEL: LTC2946_ALERT_MIN_PWR\n\r");
        }else if(alert & (1 << LTC2946_ALERT_MAX_PWR))
        {
            dbg_printf("W,Alert LTC2946_DSG_CHANNEL: LTC2946_ALERT_MAX_PWR\n\r");
        }

        alert = batt_ctx[bus].bms_data.LTC2946[LTC2946_CHG_CHANNEL].alert;
        
        if(alert & (1U << LTC2946_ALERT_MIN_ADIN))
        {
            dbg_printf("W,Alert LTC2946_CHG_CHANNEL: LTC2946_ALERT_MIN_ADIN\n\r");
        }else if(alert & (1U << LTC2946_ALERT_MIN_ISENSE))
        {
            dbg_printf("W,Alert LTC2946_CHG_CHANNEL: LTC2946_ALERT_MIN_ISENSE\n\r");
        }else if(alert & (1U << LTC2946_ALERT_MAX_ADIN))
        {
            dbg_printf("W,Alert LTC2946_CHG_CHANNEL: LTC2946_ALERT_MAX_ADIN\n\r");
        }else if(alert & (1U << LTC2946_ALERT_MIN_VIN))
        {
            dbg_printf("W,Alert LTC2946_CHG_CHANNEL: LTC2946_ALERT_MIN_VIN\n\r");
        }else if(alert & (1U << LTC2946_ALERT_MAX_VIN))
        {
            dbg_printf("W,Alert LTC2946_CHG_CHANNEL: LTC2946_ALERT_MAX_VIN\n\r");
        }else if(alert & (1U << LTC2946_ALERT_MAX_ISENSE))
        {
            dbg_printf("W,Alert LTC2946_CHG_CHANNEL: LTC2946_ALERT_MAX_ISENSE\n\r");
        }else if(alert & (1U << LTC2946_ALERT_MIN_PWR))
        {
            dbg_printf("W,Alert LTC2946_CHG_CHANNEL: LTC2946_ALERT_MIN_PWR\n\r");
        }else if(alert & (1U << LTC2946_ALERT_MAX_PWR))
        {
            dbg_printf("W,Alert LTC2946_CHG_CHANNEL: LTC2946_ALERT_MAX_PWR\n\r");
        }        
    }
    
    dbg_printf("\n\r");
    
}

void bms_data_tx_timer_handler(void *arg)
{
    UNUSED_PARAM(arg);
    
    send_bms_data(0U);
}

void bms_state_init(void)
{
    hdr.msg_sequence = 0U;
    
    vcu_update_id();
    vcu_update_api_version();
    
    aux_info.meas_marker = 0xCAFEBABEU;
}

status_t process_bms_can_data(const uint8_t *buffer, flexcan_msgbuff_t bcu_can_msg, uint16_t *tx_len, uint32_t bus)
{
    status_t s = STATUS_SUCCESS;
    volatile uint32_t msg_id = 0U;
    canfd_logical_bus_id = bus;
    
    UNUSED_PARAM(tx_len);
    
    volatile uint32_t msg_id_can = bcu_can_msg.msgId;
    msg_id = (msg_id_can >> CAN_MSG_MSG_ID_SHIFT) & 0x0000FFFU;

    clear_status_bit(STAT_VCU_CAN_MSG_EXEC_ERR);
    switch(msg_id)
    {
        case CAN_IF_MSG_GET_EM_INFO_ID:
            process_CAN_IF_MSG_ID_GET_EM_INFO(buffer);
            break;

        case CAN_IF_MSG_GET_TEMPERATURE_INFO_ID:
            process_CAN_IF_MSG_ID_GET_TEMPERATURE_INFO(buffer);
            break;
        
        case CAN_IF_MSG_GET_CM_DIAG_INFO_ID:
            process_CAN_IF_MSG_ID_GET_CM_DIAG_INFO(buffer);
            break;
        
        case CAN_IF_MSG_GET_FG_INFO_ID:
            process_CAN_IF_MSG_ID_GET_FG_INFO(buffer);
            break;
        
        case CAN_IF_MSG_GET_CELLS_ID:
            process_CAN_IF_MSG_ID_GET_CELLS_INFO(buffer);
            break;
        
        case CAN_IF_MSG_GET_SENSOR_INFO_ID:
            process_CAN_IF_MSG_ID_GET_SENSOR_INFO(buffer);
            break;
        
        case CAN_IF_MSG_BMS_STATUS_ID:
            process_CAN_IF_MSG_ID_BMS_STATUS(buffer);
            break;
        
        case CAN_IF_MSG_BAL_INFO_ID:
            process_CAN_IF_MSG_ID_BAL_INFO(buffer);
            break;
        
        case CAN_IF_MSG_GET_FW_INFO_ID:
            process_CAN_IF_MSG_GET_FW_INFO(buffer);
            break;
        
        case CAN_IF_MSG_GET_LAST_RST_STATE_ID:
            process_CAN_IF_MSG_GET_LAST_RST_STATE(buffer);
            break;
        
        case CAN_IF_MSG_GET_UID_ID:
            process_CAN_IF_MSG_GET_UID(buffer);
            break;
        
        case CAN_IF_MSG_CM_PART2_ID:
            process_CAN_IF_MSG_CM_PART2(buffer);
            break;
        
        case CAN_IF_MSG_BMS_READ_TIME_ID:
            process_CAN_IF_MSG_BMS_READ_TIME(buffer);
            break;
        
        case CAN_IF_MSG_ID_WAKEUP_BMS_ID:
            dbg_printf("Bus %d: Wakeup response ACK\n\r", bus);
            break;
        
        case CAN_IF_MSG_BMS_WAKE_NTF_ID:
            process_wake(buffer);
            break;
        
        case CAN_IF_MSG_ERR_RESPONSE_ID:
            dbg_printf("Bus %d: Protocol Stack Error Response\n\r", bus);
            break;
        
        case CAN_IF_MSG_DSG_CHG_FET_ON_ID:
            dbg_printf("VCU Firmware Info: %d.%d, FW Date = %s, FW Time = %s\n\r", get_fw_max_ver(), get_fw_min_ver(), get_fw_date(), get_fw_timestamp());
#ifdef USE_FEATURE_VCU_ON_DESK
            PINS_DRV_ClearPins(LED_GREEN_GPIO, 1U << LED_GREEN_PIN);
#endif
            break;
        
        case CAN_IF_MSG_DSG_CHG_FET_OFF_ID:
#ifdef USE_FEATURE_VCU_ON_DESK
            PINS_DRV_SetPins(LED_GREEN_GPIO, 1U << LED_GREEN_PIN);
#endif
            mc_set_motor_pwr_state(MC_MOTOR_PWR_OFF);
            mc_set_gear(MC_GEAR_POS_NEUTRAL);
            break;
        
        case CAN_IF_MSG_BMS_HEART_BEAT_ID:
#ifdef USE_FEATURE_VCU_ON_DESK
            debug_bms_data(bus);
#else
            send_bms_data(bus);
#endif
            break;
        
        case CAN_IF_MSG_BMS_EXECEPTION_INFO_ID:
            process_CAN_IF_MSG_BMS_EXECEPTION_INFO(buffer, bus);
            break;
        
        case CAN_IF_MSG_BMS_DSG_OC_INFO_ID:
            process_CAN_IF_MSG_BMS_DSG_OC_INFO_ID(buffer, bus);
            break;
        
        case CAN_IF_MSG_BMS_CHG_OC_INFO_ID:
            process_CAN_IF_MSG_BMS_CHG_OC_INFO_ID(buffer, bus);
            break;
        
        case CAN_IF_MSG_BMS_DSG_UV_INFO_ID:
            process_CAN_IF_MSG_BMS_DSG_UV_INFO_ID(buffer, bus);
            break;
        
        case CAN_IF_MSG_BMS_CHG_OV_INFO_ID:
            process_CAN_IF_MSG_BMS_CHG_OV_INFO_ID(buffer, bus);
            break;  
        
        case CAN_IF_MSG_BMS_PEAK_DSG_I_INFO_ID:
            process_CAN_IF_MSG_BMS_PEAK_DSG_I_INFO_ID(buffer, bus);
            break; 
        
        case CAN_IF_MSG_BMS_BF_BAL_CELL_INFO_ID:
            process_CAN_IF_MSG_BF_BMS_BAL_CELL_INFO_ID(buffer, bus);
            break;
        
        case CAN_IF_MSG_BMS_AF_BAL_CELL_INFO_ID:
            process_CAN_IF_MSG_AF_BMS_BAL_CELL_INFO_ID(buffer, bus);
            break;
        
        case CAN_IF_MSG_NTF_INIT_COMPLETE_ID:
            process_CAN_IF_MSG_NTF_INIT_COMPLETE_ID(buffer, bus);
            break;
        
        case CAN_IF_MSG_BMS_PROC_EXEC_ERR_ID:
            process_CAN_IF_MSG_BMS_PROC_EXEC_ERR_ID(buffer, bus);
            break;
        
        case CAN_IF_MSG_BMS_FW_UPD_MSG_ID:
            process_CAN_IF_MSG_BMS_FW_UPD_MSG_ID(buffer, bus);
            break;
        
        case CAN_IF_MSG_BMS_FW_UPD_DONE_MSG_ID:
            process_CAN_IF_MSG_BMS_FW_UPD_DONE_MSG_ID(buffer, bus);
            break;
        
        case CAN_IF_MSG_BATTERY_PACK_INFO_MSG_ID:
            process_CAN_IF_MSG_BATTERY_PACK_INFO_MSG_ID(buffer, bus);
            break;
        
        case CAN_IF_MSG_EXCP_NTF_MSG_ID:
            process_CAN_IF_MSG_EXCP_NTF_MSG_ID(buffer, bus);
            break;
        
        case CAN_IF_MSG_BMS_GET_EC_AT_BOOT_ID:
            process_CAN_IF_MSG_BMS_GET_EC_AT_BOOT(buffer, bus);
            break;
        
        default:
            __NOP();
            break;
    }

    return s;
}
#endif /* USE_FEATURE_FLEXCAN_IF */
