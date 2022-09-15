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
 
#include "bms.h"
#include "osif.h"
#include "pins_driver.h"
#include "aes.h"
#include "bms_can_if.h"
#include "can_messenger_rx.h"

#ifdef USE_SW_AES_MOD
#include "aes_sw.h"
#endif

#ifdef USE_FEATURE_FLEXCAN_IF

#ifdef USE_FEATURE_CAN_BUS_TIMEOUT

#define BMS_DATA_TX_TIMER_PERIOD    (470U)
#define CAN_MSG_MAX_TIMEOUTS        (10U)

static osif_timer_id_t *can_bus_if_timeout_timers[MAX_CANFD_LOGICAL_INTERFACES];
static uint32_t can_msg_timeout_count = 0U;
void canfd_bus0_timeout_handler(void *arg);

#endif

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

#ifdef USE_FEATURE_CAN_BUS_TIMEOUT
static void can_fd_reset_bus_rx_state(uint32_t bus)
{
    canfd_rx_state[bus] = CANFD_STATE_INIT_RX;
}

static status_t can_fd_encrypt(uint8_t *buffer, uint16_t len, uint8_t *enc_buffer, uint16_t *enc_len)
{
    status_t ret = STATUS_SUCCESS;
    uint16_t enc_sz = 0U;
    
    uint8_t can_msg_hash[CMAC_HASH_SIZE];
        
    if((len % (uint8_t)AES_BLOCK_LENGTH) > 0U)
    {
        enc_sz = len + ((uint8_t)AES_BLOCK_LENGTH - ((len % (uint8_t)AES_BLOCK_LENGTH)));
    }   
    else
    {
        enc_sz = len;
    }
    
    DEV_ASSERT(enc_sz <= (CAN_FD_MAX_LEN - CMAC_HASH_SIZE));
    
#ifndef USE_SW_AES_MOD
    aes_encrypt_buffer_can_bms_vcu(buffer, enc_buffer, enc_sz);
    aes_cmacl_can_bms_vcu(enc_buffer, enc_sz, can_msg_hash); 
#else
   aes_sw_enc(buffer, enc_buffer, enc_sz);
   aes_sw_cmac(enc_buffer, enc_sz, can_msg_hash); 
#endif
    
    (void)memcpy(&enc_buffer[enc_sz], can_msg_hash, CMAC_HASH_SIZE);

    *enc_len = enc_sz + CMAC_HASH_SIZE;   
    
    return ret;    
}

static int32_t can_fd_bms_receive_nb(uint32_t bus)
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

#ifdef USE_FEATURE_CAN_BUS_TIMEOUT
            (void)osif_timer_start(can_bus_if_timeout_timers[bus], CAN_FD_BUS_TIMEOUT);
#endif
            break;
        
        case CANFD_STATE_WAIT_RX_NB:
            
            s = FLEXCAN_DRV_GetTransferStatus(CAN_IF_BMS, (uint8_t)rx_mbx[bus]);
            if(s == STATUS_SUCCESS)
            {
                
#ifdef USE_FEATURE_CAN_BUS_TIMEOUT
                (void)osif_timer_stop(can_bus_if_timeout_timers[bus]);
                bms_data_tx_timer_stop();
                clear_status_bit(STAT_VCU_BMS_CAN_MSG_TIMEOUT);
                clear_status_bit(STAT_VCU_BMS_CAN_LINK_FAIL);
                can_msg_timeout_count = 0U;
#endif
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
                            /* Process Data */
                            (void)process_bms_can_data(dec_fd_rx_buffer, bms_rx_buff[bus], &canfd_cipher_len, bus);
                        }    
                    }
#else
                     /* Process Data */
                     extern void can_fd_validate(uint8_t *buffer, uint16_t len);
                     can_fd_validate(bms_rx_buff.data, bms_rx_buff.dataLen);
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

void canfd_bus0_timeout_handler(void *arg)
{
    UNUSED_PARAM(arg);
    
    (void)osif_timer_stop(can_bus_if_timeout_timers[CANFD_LOGICAL_BUS0]);
    
    (void)FLEXCAN_DRV_AbortTransfer(CAN_IF_BMS, (uint8_t)rx_mbx[CANFD_LOGICAL_BUS0]);
    can_fd_reset_bus_rx_state(CANFD_LOGICAL_BUS0);
    
    set_status_bit(STAT_VCU_BMS_CAN_MSG_TIMEOUT);
    can_msg_timeout_count++;
    
    if(can_msg_timeout_count > CAN_MSG_MAX_TIMEOUTS)
    {
        bms_data_tx_timer_start(BMS_DATA_TX_TIMER_PERIOD);
        set_status_bit(STAT_VCU_BMS_CAN_LINK_FAIL);
    }
}

#endif /* USE_FEATURE_CAN_BUS_TIMEOUT */

uint32_t canfd_get_bus_en_state(uint32_t bus)
{
    return canfd_bus_en_state[bus];
}

void canfd_disable_bus(uint32_t bus)
{
    canfd_bus_en_state[bus] = CANFD_LOGICAL_BUS_DISABLED;
}

void canfd_enable_bus(uint32_t bus)
{
    canfd_bus_en_state[bus] = CANFD_LOGICAL_BUS_ENABLED;
}

void can_fd_reset_rx_state(void)
{
    canfd_rx_state[CANFD_LOGICAL_BUS0] = CANFD_STATE_INIT_RX;
}

uint32_t can_fd_get_rx_state(uint32_t bus)
{
    UNUSED_PARAM(bus);
    
    return canfd_rx_state[CANFD_LOGICAL_BUS0];
}

void can_fd_if_bms_init(void)
{
    flexcan_data_info_t data_info =
    {
       .data_length = CAN_FD_MAX_LEN,
       .msg_id_type = FLEXCAN_MSG_ID_EXT,
       .enable_brs  = false,
       .fd_enable   = true,
       .fd_padding  = 0U
    };
    
    if(canfd_bus_en_state[CANFD_LOGICAL_BUS0] == CANFD_LOGICAL_BUS_ENABLED)
    {
        (void)FLEXCAN_DRV_ConfigRxMb(CAN_IF_BMS, CAN_BMS_RX_MAILBOX, &data_info, BMS_VCU_SLOT0_COMM_ID);
        FLEXCAN_DRV_SetRxMbGlobalMask(CAN_IF_BMS, FLEXCAN_MSG_ID_EXT, BMS_VCU_MBX_GBL_MASK_FILTER);
        (void)FLEXCAN_DRV_SetRxIndividualMask(CAN_IF_BMS, FLEXCAN_MSG_ID_EXT, CAN_BMS_RX_MAILBOX, BMS_VCU_SLOT0_COMM_ID);
        
#ifdef USE_FEATURE_CAN_BUS_TIMEOUT
        can_bus_if_timeout_timers[CANFD_LOGICAL_BUS0] = osif_timer_create(canfd_bus0_timeout_handler, osTimerOnce, NULL, NULL);
#endif
        bms_create_tx_timer();
    }
}

status_t can_fd_if_bms_send(uint8_t *buffer, uint16_t len, uint32_t msg_id, uint32_t bus_id)
{
    status_t ret;
    flexcan_data_info_t data_info;
    
#ifdef USE_FEATURE_CAN_BUS_ENCRYPTION
    uint8_t can_encrypted_buffer[CAN_FD_MAX_LEN];
    uint16_t enc_len = 0U;
    
    UNUSED_PARAM(bus_id);

    (void)can_fd_encrypt(buffer, len, can_encrypted_buffer, &enc_len);
    
    data_info.data_length = enc_len;
    data_info.msg_id_type = FLEXCAN_MSG_ID_EXT;
    data_info.enable_brs  = false;
    data_info.fd_enable   = true;
    data_info.fd_padding  = 0U;
    data_info.is_remote   = false;
    
    /* Configure TX message buffer with index TX_MSG_ID and TX_MAILBOX*/
    (void)FLEXCAN_DRV_ConfigTxMb(CAN_IF_BMS, CAN_BMS_TX_MAILBOX, &data_info, msg_id);

    /* Execute send non-blocking */
    ret = FLEXCAN_DRV_Send(CAN_IF_BMS, CAN_BMS_TX_MAILBOX, &data_info, msg_id, can_encrypted_buffer);
#else
    data_info.data_length = len,
    data_info.msg_id_type = FLEXCAN_MSG_ID_EXT,
    data_info.enable_brs  = false,
    data_info.fd_enable   = true,
    data_info.fd_padding  = 0U;
    data_info.is_remote   = false;
    
    /* Configure TX message buffer with index TX_MSG_ID and TX_MAILBOX*/
    FLEXCAN_DRV_ConfigTxMb(SYS_CAN_IF, BMS_TO_VCU_FD_TX_MBX, &data_info, msg_id);

    /* Execute send non-blocking */
    ret = FLEXCAN_DRV_Send(SYS_CAN_IF, BMS_TO_VCU_FD_TX_MBX, &data_info, msg_id, buffer);
#endif /* USE_FEATURE_CAN_BUS_ENCRYPTION */

    return ret;
}

void can_fd_bms_receive(uint32_t bus_id)
{ 
    (void)can_fd_bms_receive_nb(bus_id);   
}

#endif /* USE_FEATURE_FLEXCAN_IF */
