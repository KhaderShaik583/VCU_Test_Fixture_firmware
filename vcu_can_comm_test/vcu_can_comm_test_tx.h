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
 
#ifndef VCU_CAN_COMM_TEST_TX
#define VCU_CAN_COMM_TEST_TX

#include <stdint.h>
#include "fw_common.h"
#include "bms.h"
#include "canfd_queue.h"


status_t vcu_2_bms_can_test_msg(uint32_t msgid);
status_t vcu_2_mc_send_rpdo_msg(uint32_t msg_Id);
status_t vcu_2_dba_send_test_msg(uint32_t msg_Id);
status_t vcu_2_bms_can_test_msg_2(uint32_t msgid);
status_t vcu_2_bms_status_msg(uint32_t msgid);
status_t bms_ntf_boot_done(uint32_t msgid);
status_t vcu_2_bms_can_test_msg_reply(uint32_t msgid, const uint8_t *msg_sig);
#endif /* VCU_CAN_COMM_TEST_TX */
