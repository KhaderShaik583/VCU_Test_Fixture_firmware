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
 
#include "can_messenger_tx.h"
#include "can_messenger_rx.h"
#include "batterypack.h"
#include "stat.h"
#include "bms_can_if.h"
#include "bms_task.h"
#include "init_msg.h"
#include "fota.h" 
#include "vcu_can_comm_test_tx.h"

#define CANFD_MSG_SIG_LEN   (8U) 
#define CAN_IF_VCU_2_BMS_TEST_MSG_ID 	0x123U
#define CAN_IF_VCU_2_MC_TEST_MSG_ID 	0x456U
#define CAN_IF_VCU_2_DBA_TEST_MSG_ID 	0x789U

/* For compatibility with earlier designs that had 3 ports for 3 packs */
static const uint32_t slot_to_msg_id_map[MAX_CANFD_LOGICAL_INTERFACES] = {
        /* PORT 0 */ 0x200000U
};

static uint8_t fw_msg_sig[CANFD_MSG_SIG_LEN + 56U];
static volatile uint32_t mscm_ocmdr0_save = 0U;
static volatile uint32_t mscm_ocmdr1_save = 0U;


static status_t can_fd_if_bms_testmsg_send(uint8_t *buffer, uint16_t len, uint32_t msg_id, uint32_t bus_id)
{
    status_t ret;
    flexcan_data_info_t data_info;
    
#ifdef USE_FEATURE_CAN_BUS_ENCRYPTION
    uint8_t can_encrypted_buffer[CAN_FD_MAX_LEN];
    uint16_t enc_len = 0U;
    
    UNUSED_PARAM(bus_id);

    
    data_info.data_length = len;
    data_info.msg_id_type = FLEXCAN_MSG_ID_EXT;
    data_info.enable_brs  = false;
    data_info.fd_enable   = true;
    data_info.fd_padding  = 0U;
    data_info.is_remote   = false;
    
    /* Configure TX message buffer with index TX_MSG_ID and TX_MAILBOX*/
    (void)FLEXCAN_DRV_ConfigTxMb(CAN_IF_BMS, CAN_BMS_TX_MAILBOX, &data_info, msg_id);

    /* Execute send non-blocking */
    ret = FLEXCAN_DRV_Send(CAN_IF_BMS, CAN_BMS_TX_MAILBOX, &data_info, msg_id, buffer);
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

status_t vcu_2_bms_can_test_msg(uint64_t msgid)
{
    status_t s = STATUS_SUCCESS;
    
    uint8_t dummy_buffer[64] = {"BMSCANTESTMESSAGE"};
    
    (void)can_fd_if_bms_testmsg_send(dummy_buffer, 13U, msgid, 0);
    
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


