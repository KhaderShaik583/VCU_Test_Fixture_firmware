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
 
#include "dba_task.h"
#include "osif.h"
#include "aes.h"
#include "rtx_os.h"
#include "udp_task.h" 
#include "rtc_task.h"
#include "odometer_task.h"
#include "wdt_task.h"
#include "shared_mem.h" 
#include "charger_task.h"
#include "init_task.h"
#include "bcm.h"
#include "mc_task.h"
#include "cd1030.h"

#define DBA_MSGQUEUE_OBJECTS    (16)
#define ABS_MODE_SINGLE_CHANNEL (1U)
#define ABS_MODE_DUAL_CHANNEL   (0U)
#define ABS_COMM_TIMER_TIMEOUT  (1000U)
#define ABS_SPEED_FACTOR        (0.05625f)

#define DBA_STATE_RX_MSGS           (0U)
#define DBA_STATE_QUEUE_PROCESS     (1U)
#define DBA_STATE_START_RX_FIFO     (2U)

#define ABS_MSG_INFO_SEND           (0xA4U)
#define ABS_MSG_INFO_PERIOD_MS      (80U)
#define DISPLAY_TIMEOUT_MS          (30000U)
#define CHG_HBT_TIMEOUT_MSG         (3000U)

typedef struct
{
    uint16_t mag_x_raw;
    uint16_t mag_y_raw;
    uint16_t mag_z_raw;
}mag_raw_measurement_t;

#ifdef USE_FEATURE_RTX_VIEWER
__attribute__((section(".bss.os.thread.cb")))
thread_tcb_t dba_thread_tcb;
#else
static thread_tcb_t dba_thread_tcb;
#endif /* USE_FEATURE_RTX_VIEWER */

static thread_id_t thread_dba;

__attribute__((section("ARM_LIB_STACK")))
static uint64_t dba_thread_stk[DBA_STACK_SIZE];

static const osThreadAttr_t dba_attr = {
    DBA_TASK_NAME,
    osThreadDetached,
    &dba_thread_tcb,
    sizeof(dba_thread_tcb),
    &dba_thread_stk[0],
    DBA_STACK_SIZE * sizeof(uint64_t),
    osPriorityNormal4,
    0U,
    0U    
};

osif_msg_queue_id_t dba_msg_queue; 
void dba_can_callback(uint8_t instance, flexcan_event_type_t eventType, uint32_t idx, flexcan_state_t *flexcanState);

static uint8_t obc_fw_min = 0U;
static uint8_t obc_fw_max = 0U;

static uint32_t als_value = 0U;

static shmem_block_bms_t shmem_abs_msg;
static shmem_block_bms_t chg_evt_msg;

static osif_timer_id_t display_timeout_timer;
static osif_timer_id_t chg_hbt_timer;

static abs_ctx_t abs_ctx;
static uint32_t chg_ui_event_val = 0U;
static uint32_t charger_task_started = 0U;

/* 
    Messages incoming on this bus.
*/
#define ABS_MAX_MSGS    (6U)
static const uint16_t abs_msg_ids[ABS_MAX_MSGS] = {
    DISP_CAN_ALS_MSG_ID,
    DISP_CAN_MAG_MSG_ID,
    ABS_CAN_SPEED_INFO_MSG_ID,
    CHG_MSG_CHG_BOOT_NTF_ID,
    CHG_MSG_CHRG_FAULTS_ID,			
    CAN_MSG_ANALOG_MEAS_ID
};

static mag_raw_measurement_t mag_values;

static void abs_cm_average(float_t *prev_avg, uint32_t spd)
{
    /* Cumulative Moving Average */
    static uint32_t n = 0U;
    
    if(prev_avg != NULL)
    {
        *prev_avg = ((float_t)spd + ((float_t)n * *prev_avg)) / (n + 1U);

        n++;
    }
}

static void dba_config_can(void)
{
    uint16_t id_counter = 0U;
    flexcan_id_table_t filterTable[8];    
    uint16_t abs_msg = 0U;
    
    flexcan_data_info_t data_info =
    {
       .data_length = 8U,
       .msg_id_type = FLEXCAN_MSG_ID_STD,
       .enable_brs  = false,
       .fd_enable   = false,
       .fd_padding  = 0U
    };
    
    (void)FLEXCAN_DRV_ConfigRxMb(CAN_IF_ABS, CAN_DBA_RX_MAILBOX, &data_info, RX_ABS_MSG_ID);
    for(abs_msg = 0; abs_msg < ABS_MAX_MSGS; abs_msg++)
    {
        (void)FLEXCAN_DRV_ConfigRxMb(CAN_IF_MOTOR, CAN_DBA_RX_MAILBOX + abs_msg, &data_info, abs_msg_ids[abs_msg]);
        (void)FLEXCAN_DRV_SetRxIndividualMask(CAN_IF_MOTOR, FLEXCAN_MSG_ID_STD, CAN_DBA_RX_MAILBOX + abs_msg, abs_msg_ids[abs_msg]);
    }
    
	FLEXCAN_DRV_SetRxMbGlobalMask(CAN_IF_ABS, FLEXCAN_MSG_ID_STD, DBA_CAN_IF_MBX_FILTER);
    
	/* Fill id filter table */
	/* Fill id filter table */
	for(id_counter = 0U; id_counter < 8U; id_counter++)
	{
		filterTable[id_counter].isRemoteFrame = false;
		filterTable[id_counter].isExtendedFrame = 0U;
		filterTable[id_counter].id = 0;
	}
    
    /* Fill id filter table */
	for(abs_msg = 0; abs_msg < ABS_MAX_MSGS; abs_msg++)
	{
		filterTable[abs_msg].isRemoteFrame = false;
		filterTable[abs_msg].isExtendedFrame = 0U;
		filterTable[abs_msg].id = abs_msg_ids[abs_msg];
	}
    
	/* Configure RX FIFO ID filter table elements based on filter table defined above*/
	FLEXCAN_DRV_ConfigRxFifo(CAN_IF_ABS, FLEXCAN_RX_FIFO_ID_FORMAT_A, filterTable);
    
	/* set individual masking type */
	FLEXCAN_DRV_SetRxMaskType(CAN_IF_ABS, FLEXCAN_RX_MASK_INDIVIDUAL);
    
	/* rest of filter items are masked with RXFGMASK */
	FLEXCAN_DRV_SetRxFifoGlobalMask(CAN_IF_ABS, FLEXCAN_MSG_ID_STD, DBA_CAN_IF_GBL_MBX_FILTER);
    
	/* set mask affecting MB10 */
    (void)FLEXCAN_DRV_SetRxIndividualMask(CAN_IF_ABS, FLEXCAN_MSG_ID_STD, CAN_DBA_RX_MAILBOX, DBA_CAN_IF_MBX_FILTER);
    
    FLEXCAN_DRV_InstallEventCallback(CAN_IF_ABS, dba_can_callback, NULL);
}


static void process_DISP_CAN_MAG_MSG_ID(uint8_t buffer[])
{
    (void)memcpy((uint8_t *)&mag_values, buffer, sizeof(mag_raw_measurement_t));
    
    __NOP();
}

/**
 * @brief Process ambient light sensor data.
 * During Day   - DRL is ON and Low Beam is OFF
 *        Night - DRL is OFF and Low Beam is ON
 * 
 * @return None 
 */
static void process_DISP_CAN_ALS_MSG_ID(uint8_t buffer[])
{
    mc_gear_e gear;
    uint32_t pa_mode = 0U;
    
    mc_get_gear(&gear, &pa_mode);
    
    als_value = (uint32_t)buffer[0] | ((uint32_t)buffer[1] << 8U) | ((uint32_t)buffer[2] << 16U) | ((uint32_t)buffer[3] << 24U);
    
    /* Low beam and DRL are primarily controlled by the ALS sensor value 
       During day time - the DRL is ON and Low Beam OFF
       During night time - the DRL is OFF and Low Beam ON.
    
       In either case if the pass beam is activated the DRL must turn OFF.
    */
    
    if(chg_get_connection_state() == 0U)
    {
        if((gear == MC_GEAR_POS_FWD) || (pa_mode != MC_PA_MODE_OFF))
        {
            if(als_value == 0)
            {
                /* Night time */
                bcm_control(BCM_DRL_LED_CTRL, BCM_CTRL_STATE_OFF);
                bcm_control(BCM_LOW_BEAM_CTRL, BCM_CTRL_STATE_ON);
            }
            else
            {
                /* Day time */
                if(cd1030_is_high_beam_active() == 1U)
                {
                    bcm_control(BCM_DRL_LED_CTRL, BCM_CTRL_STATE_OFF);
                }
                else
                {
                    bcm_control(BCM_DRL_LED_CTRL, BCM_CTRL_STATE_ON);
                }
                
                bcm_control(BCM_LOW_BEAM_CTRL, BCM_CTRL_STATE_OFF);
            }
        }
    }
}

static void dba_abs_menu_ctrl(uint8_t state)
{
    udp_msg_queue_obj_t qmsg;

    shmem_abs_msg.udp_msg.cmd = MSG_ID_ABS_MENU_CTRL_S32_IMX;
    shmem_abs_msg.udp_msg.len = 1U;
    shmem_abs_msg.buffer[0] = state;

    qmsg.msg = (void *)&shmem_abs_msg;
    qmsg.msg_id = shmem_abs_msg.udp_msg.cmd;
    qmsg.msg_len = shmem_abs_msg.udp_msg.len;
    (void)osif_msg_queue_send(udp_msg_queue, &qmsg, 0U, 2U);   

    abs_ctx.abs_menu_state = state;    
}

static void dba_charger_evt_imx(uint32_t event)
{
    udp_msg_queue_obj_t udp_msg;

    chg_evt_msg.udp_msg.cmd = MSG_ID_CHARGER_EVT_S32_IMX;
    chg_evt_msg.udp_msg.len = 1U;
    chg_evt_msg.buffer[0] = event;

    udp_msg.msg = (void *)&chg_evt_msg;
    udp_msg.msg_id = chg_evt_msg.udp_msg.cmd;
    udp_msg.msg_len = chg_evt_msg.udp_msg.len;
    (void)osif_msg_queue_send(udp_msg_queue, &udp_msg, 0U, 2U);      
}

static void abs_mode_err_sm(uint32_t mode)
{
    static uint32_t sm_state = 0U;
    
    switch(sm_state)
    {
        case 0U:
            if(mode != abs_ctx.abs_last_mode)
            {
                set_status_bit(STAT_VCU_ABS_MODE_ERR);
                sm_state = 1U;
            }   
            break;
        case 1U:
            if(mode == abs_ctx.abs_last_mode)
            {
                sm_state = 0U;
                clear_status_bit(STAT_VCU_ABS_MODE_ERR);
            }
            break;
        default:
            __NOP();
            break;           
    }
}

static void abs_menu_sm(uint32_t speed)
{
    static uint32_t sm_state = 0U;
    
    switch(sm_state)
    {
        case 0U:
            if((speed > 5U) && 
               (abs_ctx.abs_menu_state == ABS_MENU_ENABLE))
            {
                dba_abs_menu_ctrl(ABS_MENU_DISABLE);
                sm_state = 1U;
            }  
            break;
            
        case 1U:
            if(speed < 5U)
            {
                dba_abs_menu_ctrl(ABS_MENU_ENABLE);
                sm_state = 0U;
            }  
            break;
            
        default:
            __NOP();
            break;
    }
}

static void process_ABS_CAN_SPEED_INFO_MSG(flexcan_msgbuff_t recv_buff)
{
    volatile uint8_t fwheel_validity = 0U;
    volatile uint8_t rwheel_validity = 0U;

    uint32_t current_millis = 0U;
    uint32_t prev_millis = 0U;
    uint16_t speed_raw = 0U;
    float_t time_diff = 0.0f;
    
    fwheel_validity = (recv_buff.data[4] & (1U << 4U));
    rwheel_validity = (recv_buff.data[4] & (1U << 5U));
    
    (void)osif_timer_start(abs_ctx.abs_comm_timer, MSEC_TO_TICK(ABS_COMM_TIMER_TIMEOUT));
    
    if((recv_buff.data[4] & (1U << 7U)) == 0x80U)
    {
        abs_ctx.abs_state = 1U;
        set_status_bit(STAT_VCU_ABS_FCN_ACTIVE);
    }
    else
    {
        abs_ctx.abs_state = 0U;
        clear_status_bit(STAT_VCU_ABS_FCN_ACTIVE);
    }
    
#if 0
    current_millis = osif_millis();

    /* Calculate Distance. Consider tick rollover */
    if(current_millis > prev_millis)
    {
        /* Time difference in sec */
        time_diff = (float_t)(current_millis - prev_millis) / 1000.0f;
    }
    else
    {
        time_diff = (float_t)(((0xFFFFFFFFU - prev_millis) + 1.0f) + current_millis) / 1000.0f;
    }
#endif
    
    if(fwheel_validity == 0U)
    {
        speed_raw = recv_buff.data[3] | ((uint16_t)recv_buff.data[2] << 8U);
        abs_ctx.speed = (uint32_t)(speed_raw * ABS_SPEED_FACTOR);
        clear_status_bit(STAT_VCU_ABS_FRONT_WHEEL_SPEED_SENSOR_FAILURE);
    }
    else
    {
        abs_ctx.speed = 0;
        set_status_bit(STAT_VCU_ABS_FRONT_WHEEL_SPEED_SENSOR_FAILURE);
    }
    
    if(rwheel_validity == 0U)                     
    {
        speed_raw = recv_buff.data[3] | ((uint16_t)recv_buff.data[2] << 8U);
        abs_ctx.speed = (uint32_t)(speed_raw * ABS_SPEED_FACTOR);
        clear_status_bit(STAT_VCU_ABS_REAR_WHEEL_SPEED_SENSOR_FAILURE);
    }
    else
    {
        abs_ctx.speed = 0;
        set_status_bit(STAT_VCU_ABS_REAR_WHEEL_SPEED_SENSOR_FAILURE);
    }
    
    /* Get ABS mode */
    abs_ctx.abs_mode = recv_buff.data[5];
    abs_mode_err_sm(abs_ctx.abs_mode);
    
    /* Get ABS mode permission */
    abs_ctx.abs_mode_perm = recv_buff.data[4] & 0x01U;
    
#if 0
    distance = speed * (time_diff / 3600.0f);
    update_counters(distance);
    abs_cm_average(&avg_speed, speed);

    prev_millis = current_millis;
#endif
}

static void process_CHG_MSG_CHG_BOOT_NTF_ID(uint8_t buffer[])
{
    dba_msg_queue_obj_t dba_mq;
    
    (void)osif_timer_stop(abs_ctx.abs_info_timer);
    (void)osif_timer_stop(abs_ctx.abs_comm_timer);

    dba_mq.msg_id = DBA_SEND_DISPLAY_MSG;
    dba_mq.data[0] = DBA_DISPLAY_TELL_TALE_CTRL_MSG;
    dba_mq.data[1] = TT_CHARGING_INDICATOR;
    dba_mq.data[2] = DBA_DISPLAY_TELL_TALE_ON;
    (void)osif_msg_queue_send(dba_msg_queue, &dba_mq, 0U, 10U);

    bcm_control(BCM_ABS_EN_CTRL, BCM_CTRL_STATE_OFF);
    bcm_control(BCM_DRL_LED_CTRL, BCM_CTRL_STATE_OFF);
    bcm_control(BCM_LOW_BEAM_CTRL, BCM_CTRL_STATE_OFF);
    bcm_control(BCM_HDL_12V_EN_CTRL, BCM_CTRL_STATE_OFF);
    bcm_control(BCM_HORN_VCC_CTRL, BCM_CTRL_STATE_OFF);
    bcm_control(BCM_CHARGER_LAMP_CTRL, BCM_CTRL_STATE_ON);
    bcm_control(BCM_TAIL_LAMP_CTRL, BCM_CTRL_STATE_OFF);
    bcm_control(BCM_RIGHT_IND_CTRL, BCM_CTRL_STATE_OFF);
    bcm_control(BCM_LEFT_IND_CTRL, BCM_CTRL_STATE_OFF);

    /* Charger Connected and powered on */
    dba_charger_evt_imx(0xC1U);
    
    if(chg_get_connection_state() == 0U)
    {
        /* will happen if charger powers off and restarts */
        (void)osif_task_start(chg_task_get_id());
        charger_task_started = 1U;
    }
    
    obc_fw_max = buffer[4];
    obc_fw_min = buffer[5];
}

static void display_timeout_handler(void *arg)
{
    UNUSED_PARAM(arg);
    
    dba_msg_queue_obj_t dmq;
    
    (void)osif_timer_stop(display_timeout_timer);
    
    dmq.msg_id = DBA_SEND_DISPLAY_MSG;
    dmq.data[0] = DBA_DISP_PWR_OFF;
    dmq.data[1] = 0U;
    dmq.data[2] = 0U;
    
    (void)osif_msg_queue_send(dba_msg_queue, &dmq, 0U, 10U);
}

static void chg_hbt_timeout_handler(void *arg)
{
    UNUSED_PARAM(arg);
    
    dba_msg_queue_obj_t dmq;
    
    dmq.msg_id = DBA_SEND_CHG_MSG;
    dmq.data[0] = CAN_MSG_CHG_HBT_ID & 0x00FFU;
    dmq.data[1] = (CAN_MSG_CHG_HBT_ID >> 8U) & 0x00FFU;
    dmq.data[2] = 0xFCU;

    (void)osif_msg_queue_send(dba_msg_queue, &dmq, 0U, 10U);
}

void dba_can_callback(uint8_t instance, flexcan_event_type_t eventType, uint32_t idx, flexcan_state_t *flexcanState)
{
    UNUSED_VAR(flexcanState);
    UNUSED_VAR(instance);
    UNUSED_VAR(idx);
    
    if(eventType == FLEXCAN_EVENT_DMA_COMPLETE)
    {
        (void)osif_thread_set_flag(thread_dba, DBA_ABS_DMA_COMPLETE_FLAG);
    }
}

void abs_get_last_mode(uint8_t *last_mode)
{
    *last_mode = abs_ctx.abs_last_mode;
}

void dba_get_als_value(uint32_t *als_val)
{
    *als_val = als_value;
}

uint32_t abs_is_vehicle_stationary(void)
{
    uint32_t ret = 0U;
    
    if(abs_ctx.speed == 0U)
    {
        ret = 1U;
    }
    
    return ret;
}

uint32_t abs_get_vehicle_speed(void)
{
    return abs_ctx.speed;
}

uint32_t abs_is_active(void)
{
    return abs_ctx.abs_state;
}

uint32_t abs_get_vehicle_average_speed(void)
{
    return (uint32_t)abs_ctx.avg_speed;
}

void chg_get_ui_init_state(void)
{
    dba_charger_evt_imx(chg_ui_event_val);
}

void abs_update_menu_state(void)
{
    if(abs_ctx.abs_mode_perm == 0x1U)
    {
        dba_abs_menu_ctrl(ABS_MENU_ENABLE);
        abs_ctx.abs_menu_state = ABS_MENU_ENABLE;
    }
    else
    {
        dba_abs_menu_ctrl(ABS_MENU_DISABLE);
        abs_ctx.abs_menu_state = ABS_MENU_DISABLE;
    }    
}

void abs_info_timer_stop(void)
{
    (void)osif_timer_stop(abs_ctx.abs_info_timer);
}

void dba_get_chg_fw_ver(uint8_t *fw_max, uint8_t *fw_min)
{
    if((fw_max != NULL) && (fw_min != NULL))
    {
        *fw_max = obc_fw_max;
        *fw_min = obc_fw_min;
    }
}

__NO_RETURN static void dba_task(void *arg)
{
    volatile status_t s = STATUS_SUCCESS;
    status_t qstatus = STATUS_SUCCESS;
    flexcan_msgbuff_t dba_recv_buff;
    uint32_t dba_chg_boot_ntf = 0U;
    
    dba_msg_queue_obj_t dba_mq;
    chg_msg_queue_obj_t chg_mq;
    
    static uint32_t dba_state = 0U;
    volatile uint32_t flag_err = 0U;
    
    uint8_t data[8] = {0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U};
    int32_t lc = 0;
    uint32_t chg_msg_id = 0U;
    
    UNUSED_PARAM(arg); 
    
    flexcan_data_info_t data_info =
    {
       .data_length = 8U,
       .msg_id_type = FLEXCAN_MSG_ID_STD,
       .enable_brs  = false,
       .fd_enable   = false,
       .fd_padding  = 0U
    };
   
    (void)osif_thread_wait_on_flag(DBA_TASK_START_FLAG, OSIF_WAIT_ANY_FLAG, OSIF_WAIT_FOREVER);
    
    (void)FLEXCAN_DRV_RxFifo(CAN_IF_ABS, &dba_recv_buff);
    
    while(1)
    {
        switch(dba_state)
        {
            case DBA_STATE_RX_MSGS:                
                flag_err = osif_thread_wait_on_flag(DBA_ABS_DMA_COMPLETE_FLAG, OSIF_WAIT_ANY_FLAG, 5U);
                if((flag_err == osFlagsErrorTimeout) ||
                   (flag_err == osFlagsErrorUnknown))
                {
                    dba_state = 1U;
                    (void)osif_thread_clear_flag(DBA_ABS_DMA_COMPLETE_FLAG);
                    break;
                }
                
                (void)osif_thread_clear_flag(DBA_ABS_DMA_COMPLETE_FLAG);

                lc = osif_enter_critical();

                if(dba_recv_buff.dataLen > 0U)
                {
                    switch(dba_recv_buff.msgId)
                    {
                        case ABS_CAN_SPEED_INFO_MSG_ID:
                            process_ABS_CAN_SPEED_INFO_MSG(dba_recv_buff);
                            break;
                        
                        case DISP_CAN_ALS_MSG_ID:
                            process_DISP_CAN_ALS_MSG_ID(dba_recv_buff.data);
                            break;
                        
                        case DISP_CAN_MAG_MSG_ID:
                            process_DISP_CAN_MAG_MSG_ID(dba_recv_buff.data);
                            break;
                        
                        case CHG_MSG_CHG_BOOT_NTF_ID:
                            dba_chg_boot_ntf = 1U;
                            (void)osif_timer_start(chg_hbt_timer, MSEC_TO_TICK(CHG_HBT_TIMEOUT_MSG));
                            process_CHG_MSG_CHG_BOOT_NTF_ID(dba_recv_buff.data);
                            break;
                        
                        case CHG_MSG_CHRG_FAULTS_ID:
                            if(dba_chg_boot_ntf == 1U)
                            {
                                chg_mq.msg_id = CHG_MQ_MSG_FAULT_INFO;
                                chg_mq.data[0] = dba_recv_buff.data[0];
                                chg_mq.data[1] = dba_recv_buff.data[1];
                                chg_mq.data[2] = dba_recv_buff.data[2];
                                chg_mq.data[3] = dba_recv_buff.data[3];

                                (void)osif_msg_queue_send(chg_msg_queue, &chg_mq, 0U, 2U);
                            }
                            else
                            {
                                __NOP();
                            }

                            break;
                            
                        case CAN_MSG_ANALOG_MEAS_ID:
                            chg_mq.msg_id = CHG_MQ_MSG_ANALOG_MEAS;
                        
                            chg_mq.data[0] = dba_recv_buff.data[0];
                            chg_mq.data[1] = dba_recv_buff.data[1];
                            chg_mq.data[2] = dba_recv_buff.data[2];
                            chg_mq.data[3] = dba_recv_buff.data[3];
                            chg_mq.data[4] = dba_recv_buff.data[4];
                            chg_mq.data[5] = dba_recv_buff.data[5];
                            chg_mq.data[6] = dba_recv_buff.data[6];
                            chg_mq.data[7] = dba_recv_buff.data[7];
                        
                            (void)osif_msg_queue_send(chg_msg_queue, &chg_mq, 0U, 2U);
                            break;
                        
                        case CHG_MSG_BATT_INT_RESIST_ID:
                            /* Unused */
                            __NOP();
                            break;
                        
                        default:
                            __NOP();
                            break;
                    }
                }
                
                (void)osif_exit_critical(lc);
                
                dba_state = DBA_STATE_QUEUE_PROCESS;
                break;
                
            case DBA_STATE_QUEUE_PROCESS:
                qstatus = osif_msg_queue_recv(dba_msg_queue, &dba_mq, NULL, 10U); 
                if(qstatus == osOK)   
                {
                    switch(dba_mq.msg_id)
                    {
                        case DBA_SEND_DISPLAY_MSG:
                            data[0] = dba_mq.data[0];
                            data[1] = dba_mq.data[1];
                            data[2] = dba_mq.data[2];
                            data[3] = dba_mq.data[3];
                            data[4] = dba_mq.data[4];
                            data[5] = dba_mq.data[5];
                            data[6] = dba_mq.data[6];
                            data[7] = dba_mq.data[7];
        
                            if(FLEXCAN_DRV_GetTransferStatus(CAN_IF_ABS, CAN_DBA_TX_MAILBOX) == STATUS_SUCCESS)
                            {
                                /* Execute send non-blocking */
                                s = FLEXCAN_DRV_SendBlocking(CAN_IF_ABS, CAN_DBA_TX_MAILBOX, &data_info, DISP_RX_MSG_ID, data, 10U);
                                __NOP();
                            }
                            break;
                            
                        case DBA_SEND_ABS_MSG:
                            switch(dba_mq.data[0])
                            {
                                case ABS_LGC_MODE_SINGLE_CHANNEL:
                                    abs_ctx.abs_last_mode = ABS_MODE_SINGLE_CHANNEL;
                                    set_status_bit(STAT_VCU_ABS_MODE);
                                    break;
                                
                                case ABS_LGC_MODE_DUAL_CHANNEL:
                                    abs_ctx.abs_last_mode = ABS_MODE_DUAL_CHANNEL;
                                    clear_status_bit(STAT_VCU_ABS_MODE);
                                    break;
                                
                                case ABS_MSG_INFO_SEND:
                                    data[1] = abs_ctx.abs_last_mode;  
                                    data[0] = 0U;
                                    data[2] = 0U;
                                    data[3] = 0U;
                                    data[4] = 0U;
                                    data[5] = 0U;
                                    data[6] = 0U;
                                    data[7] = 0U;
                                    
                                    /* Send every 100 ms */
                                    if(FLEXCAN_DRV_GetTransferStatus(CAN_IF_ABS, CAN_DBA_TX_MAILBOX) == STATUS_SUCCESS)
                                    {
                                        /* Execute send non-blocking */
                                        s = FLEXCAN_DRV_SendBlocking(CAN_IF_ABS, CAN_DBA_TX_MAILBOX, &data_info, TX_ABS_MSG_ID, data, 10U);
                                        __NOP();
                                    }
                                    break;
                                
                                default:
                                    set_status_bit(STAT_VCU_ABS_MODE_ERR);
                                    break;
                            }
                            break;

                        case DBA_SEND_CHG_MSG:
                            /* data[0] & data[1] contains the message id */
                            chg_msg_id = dba_mq.data[0] | ((uint32_t)dba_mq.data[1] << 8U);
                            data[0] = dba_mq.data[2];
                            data[1] = dba_mq.data[3];
                            data[2] = dba_mq.data[4];
                            data[3] = dba_mq.data[5];
                            data[4] = dba_mq.data[6];
                            data[5] = dba_mq.data[7];
                            data[6] = 0U;
                            data[7] = 0U;
                        
                            if(FLEXCAN_DRV_GetTransferStatus(CAN_IF_ABS, CAN_DBA_TX_MAILBOX) == STATUS_SUCCESS)
                            {
                                /* Execute send non-blocking */
                                s = FLEXCAN_DRV_SendBlocking(CAN_IF_ABS, CAN_DBA_TX_MAILBOX, &data_info, chg_msg_id, data, 10U);
                                __NOP();
                            }
                            break;
                            
                        case DBA_SEND_CHG_MSG_HBT_TIMER_OFF:
                            (void)osif_timer_stop(chg_hbt_timer);
                            break;
                            
                        case DBA_RESTART_TIMERS_MSG:
                            (void)osif_timer_stop(display_timeout_timer);
                        
                            dba_chg_boot_ntf = 0U;
                            dba_mq.msg_id = DBA_SEND_DISPLAY_MSG;
                            dba_mq.data[0] = DBA_DISP_PWR_ON;
                            dba_mq.data[1] = 0U;
                            dba_mq.data[2] = 0U;
                            (void)osif_msg_queue_send(dba_msg_queue, &dba_mq, 0U, 10U);
                        
                            bcm_control(BCM_ABS_EN_CTRL, BCM_CTRL_STATE_ON);
                            bcm_control(BCM_HDL_12V_EN_CTRL, BCM_CTRL_STATE_ON);
                            bcm_control(BCM_HORN_VCC_CTRL, BCM_CTRL_STATE_ON);
                            bcm_control(BCM_CHARGER_LAMP_CTRL, BCM_CTRL_STATE_OFF);
                            bcm_control(BCM_TAIL_LAMP_CTRL, BCM_CTRL_STATE_ON);
                            
                            /* Charger disconnected / powered off */
                            dba_charger_evt_imx(0xC0U);
                            chg_ui_event_val = 0U;
                        
                            (void)osif_timer_stop(chg_hbt_timer);
                            
                            (void)osif_timer_start(abs_ctx.abs_info_timer, MSEC_TO_TICK(ABS_MSG_INFO_PERIOD_MS));
                            (void)osif_timer_start(abs_ctx.abs_comm_timer, MSEC_TO_TICK(ABS_COMM_TIMER_TIMEOUT));
                            break;
                        
                        case DBA_WAKE_DISPLAY_MSG:
                            dba_mq.msg_id = DBA_SEND_DISPLAY_MSG;
                            dba_mq.data[0] = DBA_DISP_PWR_ON;
                            dba_mq.data[1] = 0U;
                            dba_mq.data[2] = 0U;
                            
                            (void)osif_msg_queue_send(dba_msg_queue, &dba_mq, 0U, 10U);
                            (void)osif_timer_start(display_timeout_timer, MSEC_TO_TICK(DISPLAY_TIMEOUT_MS));
                            break;
                        
                        case DBA_START_DISPLAY_SLEEP_TIMER_MSG:
                            (void)osif_timer_start(display_timeout_timer, MSEC_TO_TICK(DISPLAY_TIMEOUT_MS));
                            break;
                        
                        case DBA_STOP_DISPLAY_SLEEP_TIMER_MSG:
                            (void)osif_timer_stop(display_timeout_timer);  
                        
                            dba_mq.msg_id = DBA_SEND_DISPLAY_MSG;
                            dba_mq.data[0] = DBA_DISP_PWR_ON;
                            dba_mq.data[1] = 0U;
                            dba_mq.data[2] = 0U;
                            
                            (void)osif_msg_queue_send(dba_msg_queue, &dba_mq, 0U, 10U);
                            break;
                        
                        case DBA_SEND_UI_CHG_ON_NTF:
                            /* Charging in progress */
                            chg_ui_event_val = 0xC2U;
                            dba_charger_evt_imx(0xC2U);
                            break;
                        
                        case DBA_SEND_UI_CHG_OFF_NTF:
                            /* Charging stopped */
                            chg_ui_event_val = 0xC3U;
                            dba_charger_evt_imx(0xC3U);
                            break;
                        
                        case DBA_SEND_UI_CHG_PAUSE_ERR_NTF:
                            /* Charging paused */
                            chg_ui_event_val = 0xC5U;
                            dba_charger_evt_imx(0xC5U);
                            break;
                        
                        default:
                            __NOP();
                            break;
                    }
                }
                
                dba_state = DBA_STATE_START_RX_FIFO;
                break;
                
            case DBA_STATE_START_RX_FIFO:
                (void)FLEXCAN_DRV_RxFifo(CAN_IF_ABS, &dba_recv_buff);
                dba_state = DBA_STATE_RX_MSGS;
                break;
            
            default:
                dba_state = DBA_STATE_RX_MSGS;
                break; 
        }
    }
}

thread_id_t dba_task_get_id(void)
{
    return thread_dba;
}

void abs_comm_timeout_handler(void *arg)
{
    (void)osif_timer_stop(abs_ctx.abs_comm_timer);
    dba_abs_menu_ctrl(ABS_MENU_DISABLE);
}

void abs_info_timeout_handler(void *arg)
{
    dba_msg_queue_obj_t dmq;
    
    dmq.msg_id = DBA_SEND_ABS_MSG;
    dmq.data[0] = ABS_MSG_INFO_SEND;
    
    (void)osif_msg_queue_send(dba_msg_queue, (void *)&dmq, 0U, 2U);  
}

void dba_task_create(void)
{
    uint32_t param = NULL;
    
    dba_config_can();
    
    dba_msg_queue = osif_msg_queue_create(DBA_MSGQUEUE_OBJECTS, sizeof(dba_msg_queue_obj_t));
    DEV_ASSERT(dba_msg_queue != NULL);
    
    thread_dba = osif_thread_create(dba_task, &param, &dba_attr);
    DEV_ASSERT(thread_dba != NULL);

    abs_ctx.abs_comm_timer = osif_timer_create(abs_comm_timeout_handler, osTimerOnce, NULL, NULL);
    DEV_ASSERT(abs_ctx.abs_comm_timer != NULL);
    
    abs_ctx.abs_info_timer = osif_timer_create(abs_info_timeout_handler, osTimerPeriodic, NULL, NULL);
    DEV_ASSERT(abs_ctx.abs_info_timer != NULL);
    
    display_timeout_timer = osif_timer_create(display_timeout_handler, osTimerOnce, NULL, NULL);
    DEV_ASSERT(abs_ctx.abs_info_timer != NULL);
    
    chg_hbt_timer = osif_timer_create(chg_hbt_timeout_handler, osTimerPeriodic, NULL, NULL);
    DEV_ASSERT(chg_hbt_timer!= NULL);
    
    (void)osif_timer_start(abs_ctx.abs_info_timer, MSEC_TO_TICK(ABS_MSG_INFO_PERIOD_MS));
    
    clear_status_bit(STAT_VCU_ABS_MODE);
}
