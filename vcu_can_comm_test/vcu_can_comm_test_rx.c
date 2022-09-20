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
#include "vcu_can_comm_test_tx.h"
#include "lpuart_driver.h"

#define TS_HOUR_MASK    (0x0000001fU)
#define TS_MINUTE_MASK  (0x000007e0U)
#define TS_SECOND_MASK  (0x0001f800U)

#define CHARGER_STATE_IDLE              (0U)
#define CHARGER_STATE_WAIT              (1U)
#define CHARGER_STATE_CONNECTED         (2U)
#define CHARGER_STATE_WAIT_DISCONNECT   (3U)

#define CAN_SM_STATE_START_RX   (0U)
#define CAN_SM_STATE_WAIT_RX    (1U)

static uint8_t dec_fd_rx_buffer[CAN_FD_MAX_LEN];
static flexcan_msgbuff_t bms_rx_buff[MAX_CANFD_LOGICAL_INTERFACES];

static uint32_t canfd_rx_state[MAX_CANFD_LOGICAL_INTERFACES] = {
                        CANFD_STATE_INIT_RX
};

static uint32_t bus_transaction_state[MAX_CANFD_LOGICAL_INTERFACES] = {
                        CANFD_BUS_TRANSACTION_PENDING
};
  
static const uint32_t rx_mbx[MAX_CANFD_LOGICAL_INTERFACES] = {
                        CAN_BMS_RX_MAILBOX
};

/* CANFD_LOGICAL_BUS_ENABLED, CANFD_LOGICAL_BUS_DISABLED */
static uint32_t canfd_bus_en_state[MAX_CANFD_LOGICAL_INTERFACES] = {
                        CANFD_LOGICAL_BUS_ENABLED
};

static bms_meas_t *meas = NULL;
static auxiliary_info_t aux_info;
static charger_ctx_t charger_ctxt;
static trip_meter_disp_t trip_meters;
static vcu_info_msg_t hdr;
static imx_dbg_msg_t imx_dbg_msg;
static osif_timer_id_t *data_tx_timer = NULL;
static uint32_t initial_sentinel = 0U;



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

static flexcan_msgbuff_t dba_recv_buff;
static flexcan_msgbuff_t mc_recv_buff;
static flexcan_msgbuff_t bms_recv_buff;

static uint32_t dba_rx_state = CAN_SM_STATE_START_RX;
static uint32_t mc_rx_state = CAN_SM_STATE_START_RX;
static uint32_t bms_rx_state = CAN_SM_STATE_START_RX;

static volatile uint32_t dba_dma_complete = 0U;
static volatile uint32_t mc_dma_complete = 0U;
static volatile uint32_t bms_dma_complete = 0U;

static uint8_t dba_tx_buffer[64] = "DBA CAN TEST PASS \n\r";
static uint8_t mc_tx_buffer[64] = "MC CAN TEST PASS \n\r";
static uint8_t mc_tx_buffer1[64] = "MC CAN TEST FAIL \n\r";

void bms_can_callback(uint8_t instance, flexcan_event_type_t eventType, uint32_t idx, flexcan_state_t *flexcanState)
{
    UNUSED_VAR(flexcanState);
    UNUSED_VAR(instance);
    UNUSED_VAR(idx);
   
    if(eventType == FLEXCAN_EVENT_DMA_COMPLETE)
    {
        bms_dma_complete = 1U;
    }
    else if(eventType == FLEXCAN_EVENT_DMA_ERROR)
    {
        FLEXCAN_DRV_AbortTransfer(CAN_IF_BMS, CAN_BMS_RX_MAILBOX);
        bms_rx_state = CAN_SM_STATE_START_RX;
    }
    else
    {
        __NOP();
    }
}

void dba1_can_callback(uint8_t instance, flexcan_event_type_t eventType, uint32_t idx, flexcan_state_t *flexcanState)
{
    UNUSED_VAR(flexcanState);
    UNUSED_VAR(instance);
    UNUSED_VAR(idx);
   
    if(eventType == FLEXCAN_EVENT_DMA_COMPLETE)
    {
        dba_dma_complete = 1U;
    }
    else if(eventType == FLEXCAN_EVENT_DMA_ERROR)
    {
        FLEXCAN_DRV_AbortTransfer(CAN_IF_ABS, CAN_DBA_RX_MAILBOX);
        dba_rx_state = CAN_SM_STATE_START_RX;
    }
    else
    {
        __NOP();
    }
}

void mc1_can_callback(uint8_t instance, flexcan_event_type_t eventType, uint32_t idx, flexcan_state_t *flexcanState)
{
    UNUSED_VAR(flexcanState);
    UNUSED_VAR(instance);
    UNUSED_VAR(idx);
   
    if(eventType == FLEXCAN_EVENT_DMA_COMPLETE)
    {
        mc_dma_complete = 1U;
    }
    else if(eventType == FLEXCAN_EVENT_DMA_ERROR)
    {
        FLEXCAN_DRV_AbortTransfer(CAN_IF_MOTOR, RX_MAILBOX);
        mc_rx_state = CAN_SM_STATE_START_RX;
    }
    else
    {
        __NOP();
    }
}

static void can_fd_bms_receive_test_nb()
{
    	switch(bms_rx_state)
    {
        case CAN_SM_STATE_START_RX:
            FLEXCAN_DRV_RxFifo(CAN_IF_BMS, &bms_recv_buff);
            bms_rx_state = CAN_SM_STATE_WAIT_RX;
            break;
        
        case CAN_SM_STATE_WAIT_RX:
            if(bms_dma_complete == 1U)
            {
                bms_dma_complete = 0U;
                
                /* Process Data */
               // (void)process_can_data(vcu_rx_buff.data, &vcu_rx_buff.dataLen, vcu_rx_buff.msgId);
				vcu_2_bms_can_test_msg_2(bms_recv_buff.msgId);
                bms_rx_state = CAN_SM_STATE_START_RX;
            }
            else
            {
                bms_rx_state = CAN_SM_STATE_WAIT_RX;
            }
            break;
            
		 default:
            __NOP();
            break;
    } 
}

/**
 * @brief CAN receive state machine.
 * CAN_SM_STATE_START_RX :Start a RX operation on the RX mailbox.
 * CAN_FD_STATE_WAIT_RX - Wait for data to arrive on the mailbox without blocking.
 */
static void can_if_dba_receive_nb(void)
{
	switch(dba_rx_state)
    {
        case CAN_SM_STATE_START_RX:
            FLEXCAN_DRV_RxFifo(CAN_IF_ABS, &dba_recv_buff);
            dba_rx_state = CAN_SM_STATE_WAIT_RX;
            break;
        
        case CAN_SM_STATE_WAIT_RX:
            if(dba_dma_complete == 1U)
            {
                dba_dma_complete = 0U;
                
                /* Process Data */
               // (void)process_can_data(vcu_rx_buff.data, &vcu_rx_buff.dataLen, vcu_rx_buff.msgId);
				vcu_2_dba_send_test_msg(dba_recv_buff.msgId);
			
				if(dba_recv_buff.msgId == 0x150U)
				{
					LPUART_DRV_SendDataPolling(SYS_DEBUG_LPUART_INTERFACE, dba_tx_buffer, 64);
				}
                dba_rx_state = CAN_SM_STATE_START_RX;
            }
            else
            {
                dba_rx_state = CAN_SM_STATE_WAIT_RX;
            }
            break;
            
		 default:
            __NOP();
            break;
    } 
}

/**
 * @brief CAN receive state machine.
 * CAN_SM_STATE_START_RX :Start a RX operation on the RX mailbox.
 * CAN_FD_STATE_WAIT_RX - Wait for data to arrive on the mailbox without blocking.
 */
static void can_if_mc_receive_nb(void)
{
	switch(mc_rx_state)
    {
        case CAN_SM_STATE_START_RX:
            FLEXCAN_DRV_RxFifo(CAN_IF_MOTOR, &mc_recv_buff);
            mc_rx_state = CAN_SM_STATE_WAIT_RX;
            break;
        
        case CAN_SM_STATE_WAIT_RX:
            if(mc_dma_complete == 1U)
            {
                mc_dma_complete = 0U;
                
                /* Process Data */
               // (void)process_can_data(vcu_rx_buff.data, &vcu_rx_buff.dataLen, vcu_rx_buff.msgId);
				vcu_2_mc_send_rpdo_msg(mc_recv_buff.msgId);
				if(mc_recv_buff.msgId == 0x200U)
				{
					LPUART_DRV_SendDataPolling(SYS_DEBUG_LPUART_INTERFACE, mc_tx_buffer, 64);
				}
                mc_rx_state = CAN_SM_STATE_START_RX;
            }
            else
            {
                mc_rx_state = CAN_SM_STATE_WAIT_RX;
            }
            break;
            
		 default:
            __NOP();
            break;
    } 
}
void can_dba_receive_test()
{ 
    (void)can_if_dba_receive_nb();   
}

void can_mc_receive_test()
{ 
    (void)can_if_mc_receive_nb();   
}

void can_fd_bms_receive_test()
{ 
    (void)can_fd_bms_receive_test_nb();   
}

