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
 * Author : Khader S. [088]
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

#ifdef USE_SW_AES_MOD
#include "aes_sw.h"
#endif

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
static uint8_t mc_tx_buffer[64] =  "MC CAN TEST PASS \n\r";
static uint8_t bms_tx_buffer[64] = "BMS CAN TEST PASS \n\r";

static uint8_t mc_tx_buffer1[64] = "MC CAN TEST FAIL \n\r";
static uint32_t esrfx_1 = 0U;
static uint32_t esrfx_2 = 0U;
static uint32_t esrfx_3 = 0U;

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

void bmsfx_can_err_cb(uint8_t instance, flexcan_event_type_t eventType, flexcan_state_t *flexcanState)
{
    UNUSED_PARAM(flexcanState);
    
    if(eventType == FLEXCAN_EVENT_ERROR)
    {
        esrfx_1 = FLEXCAN_DRV_GetErrorStatus(instance);
    }
}

void chrgfx_can_err_cb(uint8_t instance, flexcan_event_type_t eventType, flexcan_state_t *flexcanState)
{
    UNUSED_PARAM(flexcanState);
    
    if(eventType == FLEXCAN_EVENT_ERROR)
    {
        esrfx_2 = FLEXCAN_DRV_GetErrorStatus(instance);
    }
}

void mcfx_can_err_cb(uint8_t instance, flexcan_event_type_t eventType, flexcan_state_t *flexcanState)
{
    UNUSED_PARAM(flexcanState);
    
    if(eventType == FLEXCAN_EVENT_ERROR)
    {
        esrfx_3 = FLEXCAN_DRV_GetErrorStatus(instance);
    }
}
/**
 * @brief Error processing for the bms CAN interface
 * 
 * @param esr - Error & status data register 
 */
void can_fd_if_bmsfx_err_process(uint32_t *esr)
{
    static uint32_t fault_confinement_count = 0U;
    
    
    if((*esr & CAN_ESR1_BOFFINT_MASK))
    {
        dbg_printf("E,BMS CAN I/F BUS OFF ERROR\n\r");
    }
    else if((*esr & CAN_ESR1_FLTCONF_MASK))
    {
		fault_confinement_count++;
        if(fault_confinement_count < 10U)
        {
            dbg_printf("E,BMS CAN I/F Fault confinement state\n\r");
        }
        else if(fault_confinement_count == 10U)
        {
            dbg_printf("E,BMS CAN I/F Fault confinement state overflow\n\r");
        }
        else
        {
            __NOP();
        }
    }
    else if((*esr & CAN_ESR1_RXWRN_MASK))
    {
        dbg_printf("E,BMS CAN I/F Repetitive RX errors\n\r");
    }
    else if((*esr & CAN_ESR1_TXWRN_MASK))
    {
        dbg_printf("E,BMS CAN I/F Repetitive TX errors\n\r");
    }
    else if((*esr & CAN_ESR1_STFERR_MASK))
    {
        dbg_printf("E,BMS CAN I/F Stuff error\n\r");
    }
    else if((*esr & CAN_ESR1_FRMERR_MASK))
    {
        dbg_printf("E,BMS CAN I/F Form error\n\r");
    }
    else if((*esr & CAN_ESR1_CRCERR_MASK))
    {
        dbg_printf("E,BMS CAN I/F CRC error\n\r");
    }
    else if((*esr & CAN_ESR1_ACKERR_MASK))
    {
        dbg_printf("E,BMS CAN I/F ACK error\n\r");
    }
    else if((*esr & CAN_ESR1_BIT0ERR_MASK))
    {
        dbg_printf("E,BMS CAN I/F BIT0 error\n\r");
    }
    else if((*esr & CAN_ESR1_BIT1ERR_MASK))
    {
        dbg_printf("E,BMS CAN I/F BIT1 error\n\r");
    }
    else if((*esr & CAN_ESR1_SYNCH_MASK))
    {
        dbg_printf("E,BMS CAN I/F SYNCH errors\n\r");
    }
    else
    {
        __NOP();
    }
    
    *esr = 0U;
}

/**
 * @brief Error processing for the charger CAN interface
 * 
 * @param esr - Error & status data register 
 */
void can_if_chg_err_process(uint32_t *esr)
{
	static uint32_t fault_confinement_count = 0U;
    
    
    if((*esr & CAN_ESR1_BOFFINT_MASK))
    {
        dbg_printf("E,DBA CAN I/F BUS OFF ERROR\n\r");
    }
    else if((*esr & CAN_ESR1_FLTCONF_MASK))
    {
		fault_confinement_count++;
        if(fault_confinement_count < 10U)
        {
            dbg_printf("E,DBA CAN I/F Fault confinement state\n\r");
        }
        else if(fault_confinement_count == 10U)
        {
            dbg_printf("E,DBA CAN I/F Fault confinement state overflow\n\r");
        }
        else
        {
            __NOP();
        }
    }
    else if((*esr & CAN_ESR1_RXWRN_MASK))
    {
        dbg_printf("E,DBA CAN I/F Repetitive RX errors\n\r");
    }
    else if((*esr & CAN_ESR1_TXWRN_MASK))
    {
        dbg_printf("E,DBA CAN I/F Repetitive TX errors\n\r");
    }
    else if((*esr & CAN_ESR1_STFERR_MASK))
    {
        dbg_printf("E,DBA CAN I/F Stuff error\n\r");
    }
    else if((*esr & CAN_ESR1_FRMERR_MASK))
    {
        dbg_printf("E,DBA CAN I/F Form error\n\r");
    }
    else if((*esr & CAN_ESR1_CRCERR_MASK))
    {
        dbg_printf("E,DBA CAN I/F CRC error\n\r");
    }
    else if((*esr & CAN_ESR1_ACKERR_MASK))
    {
        dbg_printf("E,DBA CAN I/F ACK error\n\r");
    }
    else if((*esr & CAN_ESR1_BIT0ERR_MASK))
    {
        dbg_printf("E,DBA CAN I/F BIT0 error\n\r");
    }
    else if((*esr & CAN_ESR1_BIT1ERR_MASK))
    {
        dbg_printf("E,DBA CAN I/F BIT1 error\n\r");
    }
    else if((*esr & CAN_ESR1_SYNCH_MASK))
    {
        dbg_printf("E,DBA CAN I/F SYNCH errors\n\r");
    }
    else
    {
        __NOP();
    }
    
    *esr = 0U;
}

/**
 * @brief Error processing for the mc CAN interface
 * 
 * @param esr - Error & status data register 
 */
void can_if_mcfx_err_process(uint32_t *esr)
{
    static uint32_t fault_confinement_count = 0U;
    
    
    if((*esr & CAN_ESR1_BOFFINT_MASK))
    {
        dbg_printf("E,MC CAN I/F BUS OFF ERROR\n\r");
    }
    else if((*esr & CAN_ESR1_FLTCONF_MASK))
    {
        fault_confinement_count++;
        if(fault_confinement_count < 10U)
        {
            dbg_printf("E,MC CAN I/F Fault confinement state\n\r");
        }
        else if(fault_confinement_count == 10U)
        {
            dbg_printf("E,MC CAN I/F Fault confinement state overflow\n\r");
        }
        else
        {
            __NOP();
        }
    }
    else if((*esr & CAN_ESR1_RXWRN_MASK))
    {
        dbg_printf("E,MC CAN I/F Repetitive RX errors\n\r");
    }
    else if((*esr & CAN_ESR1_TXWRN_MASK))
    {
        dbg_printf("E,MC CAN I/F Repetitive TX errors\n\r");
    }
    else if((*esr & CAN_ESR1_STFERR_MASK))
    {
        dbg_printf("E,MC CAN I/F Stuff error\n\r");
    }
    else if((*esr & CAN_ESR1_FRMERR_MASK))
    {
        dbg_printf("E,MC CAN I/F Form error\n\r");
    }
    else if((*esr & CAN_ESR1_CRCERR_MASK))
    {
        dbg_printf("E,MC CAN I/F CRC error\n\r");
    }
    else if((*esr & CAN_ESR1_ACKERR_MASK))
    {
        dbg_printf("E,MC CAN I/F ACK error\n\r");
    }
    else if((*esr & CAN_ESR1_BIT0ERR_MASK))
    {
        dbg_printf("E,MC CAN I/F BIT0 error\n\r");
    }
    else if((*esr & CAN_ESR1_BIT1ERR_MASK))
    {
        dbg_printf("E,MC CAN I/F BIT1 error\n\r");
    }
    else if((*esr & CAN_ESR1_SYNCH_MASK))
    {
        dbg_printf("E,MC CAN I/F SYNCH errors\n\r");
    }
    else
    {
        __NOP();
    }
    
    *esr = 0U;
}

int32_t can_fd_bms_receive_test_nb(uint32_t bus)
{
    int32_t ret = 0;
    status_t s;
    
#ifdef USE_FEATURE_CAN_BUS_ENCRYPTION
    uint8_t canfd_rx_hash[CMAC_KEY_SIZE];
    int32_t canfd_macval = 1U;
    uint16_t canfd_cipher_len = 0U;
    uint32_t volatile msgid = 0U;
#endif
    
    switch(canfd_rx_state[bus])
    {
        case CANFD_STATE_INIT_RX:
            /* Start a RX on Mailbox specified by bus parameter */
            (void)FLEXCAN_DRV_Receive(CAN_IF_BMS, (uint8_t)rx_mbx[bus], &bms_rx_buff[bus]);
        
            /* Go to next state after starting Rx */
            canfd_rx_state[bus] = CANFD_STATE_WAIT_RX_NB;
        
            /* Mark the bus transaction as pending for the current bus*/
            bus_transaction_state[bus] = CANFD_BUS_TRANSACTION_PENDING;

            break;
        
        case CANFD_STATE_WAIT_RX_NB:
            
            s = FLEXCAN_DRV_GetTransferStatus(CAN_IF_BMS, (uint8_t)rx_mbx[bus]);
            if(s == STATUS_SUCCESS)
            {
                
                if(bms_rx_buff[bus].dataLen > 0U)
                {
#ifdef USE_FEATURE_CAN_BUS_ENCRYPTION
                                        
                    /* If its a PC message ID ignore and discard */
                    msgid = (bms_rx_buff[bus].msgId >> CAN_MSG_MSG_ID_SHIFT) & 0x0000FFFU;

                    /* WATCH OUT - The below if statement can break stuff if the msg id range it
                       excludes is not in sync with PC CAN message ids. 
                    */
                    if((msgid < 0x808U) || (msgid > 0x8FFU))
                    {
                        canfd_cipher_len = bms_rx_buff[bus].dataLen - CMAC_HASH_SIZE;
#ifndef USE_SW_AES_MOD
                        aes_cmacl_can_bms_vcu(bms_rx_buff[bus].data, canfd_cipher_len, canfd_rx_hash);
#else
                        aes_sw_cmac(bms_rx_buff[bus].data, canfd_cipher_len, canfd_rx_hash);
#endif /* USE_SW_AES_MOD */                  
                        canfd_macval = memcmp((bms_rx_buff[bus].data + canfd_cipher_len), &canfd_rx_hash[0], CMAC_KEY_SIZE); 
                        
                        if(canfd_macval == 0)
                        {
                            /* Decrypt Message */
#ifndef USE_SW_AES_MOD
                            aes_decrypt_buffer_can_bms_vcu(bms_rx_buff[bus].data, dec_fd_rx_buffer, canfd_cipher_len);
#else
                            aes_sw_dec(bms_rx_buff[bus].data, dec_fd_rx_buffer, canfd_cipher_len);
#endif /* USE_SW_AES_MOD */
								if((msgid == 0x908U) || (msgid == 0x906U))
								{
									LPUART_DRV_SendDataBlocking(SYS_DEBUG_LPUART_INTERFACE, bms_tx_buffer, 20, 200);
								}
								else if(msgid == 0x927)
								{
									bms_ntf_boot_done(0x919U);
								}
									
//								vcu_2_bms_can_test_msg_reply(bms_rx_buff[bus].msgId, dec_fd_rx_buffer);

                            /* Process Data */
                           // (void)process_bms_can_data(dec_fd_rx_buffer, bms_rx_buff[bus], &canfd_cipher_len, bus);
                        }    
                    }
#else
                     /* Process Data */
//                     extern void can_fd_validate(uint8_t *buffer, uint16_t len);
//                     can_fd_validate(bms_rx_buff.data, bms_rx_buff.dataLen);
					 vcu_2_bms_can_test_msg(bms_rx_buff[bus].msgId);

#endif /* USE_FEATURE_CAN_BUS_ENCRYPTION */

                    /* RX Success on current bus. Go to CANFD_STATE_INIT_RX to start new RX */
                    canfd_rx_state[bus] = CANFD_STATE_INIT_RX;
                }
                else
                {
                    canfd_rx_state[bus] = CANFD_STATE_INIT_RX;
                }
                
                bus_transaction_state[bus] = CANFD_BUS_TRANSACTION_DONE;
            }
            else
            {
                canfd_rx_state[bus] = CANFD_STATE_WAIT_RX_NB;
            }
            break;
            
        default:
            canfd_rx_state[bus] = CANFD_STATE_INIT_RX;
            break;
    }

    return ret;  
}

/**
 * @brief CAN receive state machine.
 * CAN_SM_STATE_START_RX :Start a RX operation on the RX mailbox.
 * CAN_FD_STATE_WAIT_RX - Wait for data to arrive on the mailbox without blocking.
 */
void can_if_dba_receive_nb(void)
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
				//vcu_2_dba_send_test_msg(dba_recv_buff.msgId);
			
				if(dba_recv_buff.msgId == 0x124U)
				{
					LPUART_DRV_SendDataBlocking(SYS_DEBUG_LPUART_INTERFACE, dba_tx_buffer, 20, 200);
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
void can_if_mc_receive_nb(void)
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
				//vcu_2_mc_send_rpdo_msg(mc_recv_buff.msgId);
				if(mc_recv_buff.msgId == 0x108U)
				{
					LPUART_DRV_SendDataBlocking(SYS_DEBUG_LPUART_INTERFACE, mc_tx_buffer, 19, 200);
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
	can_if_chg_err_process(&esrfx_2);
}

void can_mc_receive_test()
{ 
    (void)can_if_mc_receive_nb();
	can_if_mcfx_err_process(&esrfx_3);
}

void can_fd_bms_receive_test()
{ 
    (void)can_fd_bms_receive_test_nb(0); 
	can_fd_if_bmsfx_err_process(&esrfx_1);
}

