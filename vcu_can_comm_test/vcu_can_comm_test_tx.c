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
 
#include "can_messenger_tx.h"
#include "can_messenger_rx.h"
#include "batterypack.h"
#include "stat.h"
#include "bms_can_if.h"
#include "bms_task.h"
#include "init_msg.h"
#include "fota.h" 
#include "vcu_can_comm_test_tx.h"
#include "aes.h"

#ifdef USE_SW_AES_MOD
#include "aes_sw.h"
#endif

#define CANFD_MSG_SIG_LEN   (8U) 
#define CAN_IF_VCU_2_BMS_TEST_MSG_ID 	0x123U
#define CAN_IF_VCU_2_MC_TEST_MSG_ID 	0x456U
#define CAN_IF_VCU_2_DBA_TEST_MSG_ID 	0x789U

/* For compatibility with earlier designs that had 3 ports for 3 packs */
static const uint32_t slot_to_msg_id_map[MAX_CANFD_LOGICAL_INTERFACES] = {
        /* PORT 0 */ 0x200000U
};

static const uint32_t outgoing_slot_to_msg_id_map[3] = {
        /* SLOT 0 */ 0x400000U,
        /* SLOT 1 */ 0x200000U,
        /* SLOT 2 */ 0x800000U,
};

static uint8_t fw_msg_sig[CANFD_MSG_SIG_LEN + 56U];
static volatile uint32_t mscm_ocmdr0_save = 0U;
static volatile uint32_t mscm_ocmdr1_save = 0U;

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

static status_t can_fd_if_bms_testmsg_send(uint8_t *buffer, uint16_t len, uint32_t msg_id, uint32_t bus_id)
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

static status_t can_fd_if_bms_testmsg_send_afterdecryption(uint8_t *buffer, uint16_t len, uint32_t msg_id, uint32_t bus_id)
{
    status_t ret;
    flexcan_data_info_t data_info;
    
#ifndef USE_FEATURE_CAN_BUS_ENCRYPTION
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
    FLEXCAN_DRV_ConfigTxMb(CAN_IF_BMS, CAN_BMS_TX_MAILBOX, &data_info, msg_id);

    /* Execute send non-blocking */
    ret = FLEXCAN_DRV_Send(CAN_IF_BMS, CAN_BMS_TX_MAILBOX, &data_info, msg_id, buffer);
#endif /* USE_FEATURE_CAN_BUS_ENCRYPTION */

    return ret;
}
//status_t vcu_2_bms_can_test_msg(uint32_t msgid)
//{
//    status_t s = STATUS_SUCCESS;
//    
//    uint8_t dummy_buffer[64] = {"BMSCANTESTMESSAGE"};
//    
//    (void)can_fd_if_bms_testmsg_send(dummy_buffer, 13U, msgid, 0);
//    
//    return s;
//}

status_t vcu_2_bms_can_test_msg(uint32_t msgid)
{
    status_t s = STATUS_SUCCESS;
     volatile uint32_t emsg_id = 0U;
    uint8_t msg_sig[CANFD_MSG_SIG_LEN] = {0x25U, 0x9cU, 0x1fU, 0x57U, 0x93U, 0xacU, 0x8bU, 0x92U};
	
	    msgid = outgoing_slot_to_msg_id_map[0] | ((uint32_t)msgid << CAN_MSG_MSG_ID_SHIFT);   
	    /* Add message specific data other than signature */
    /* Increment data length by CANFD_MSG_SIG_LEN + data length */
    
    (void)can_fd_if_bms_testmsg_send(msg_sig, CANFD_MSG_SIG_LEN, msgid, 0);
    
    return s;
}

status_t vcu_2_bms_can_test_msg_reply(uint32_t msgid, const uint8_t *msg_sig)
{
    status_t s = STATUS_SUCCESS;
     volatile uint32_t emsg_id = 0U;
	uint8_t smsg_sig[8];	
//	    emsg_id |= slot_to_msg_id_map[0] | ((uint32_t)msgid << CAN_MSG_MSG_ID_SHIFT) |   \
//              (0 << CAN_MSG_PRIO_SHIFT);
	    /* Add message specific data other than signature */
    /* Increment data length by CANFD_MSG_SIG_LEN + data length */
	
    memcpy(&smsg_sig,msg_sig,sizeof(smsg_sig));
    (void)can_fd_if_bms_testmsg_send(smsg_sig, CANFD_MSG_SIG_LEN, msgid, 0);
    
    return s;
}

status_t vcu_2_mc_send_rpdo_msg(uint32_t msg_Id)
{
	status_t write_status = STATUS_SUCCESS;
	uint8_t buffer[8] = {"MCCANTES"};
    
    
	flexcan_data_info_t tx_info =
    {
      .data_length = 8U,
      .msg_id_type = FLEXCAN_MSG_ID_STD,
      .enable_brs  = false,
      .fd_enable   = false,
      .fd_padding  = 0U
    };
    
    (void)FLEXCAN_DRV_Send(CAN_IF_MOTOR, CAN_MC_TX_MAILBOX, &tx_info, msg_Id, buffer);
	
	return write_status;   
}

status_t vcu_2_dba_send_test_msg(uint32_t msg_Id)
{
	status_t write_status = STATUS_SUCCESS;
	uint8_t buffer[8] = {"DBACAN"};
    
    
	flexcan_data_info_t tx_info =
    {
      .data_length = 8U,
      .msg_id_type = FLEXCAN_MSG_ID_STD,
      .enable_brs  = false,
      .fd_enable   = false,
      .fd_padding  = 0U
    };
    
    (void)FLEXCAN_DRV_Send(CAN_IF_ABS, CAN_DBA_TX_MAILBOX, &tx_info, msg_Id, buffer);
	
	return write_status;   
}


