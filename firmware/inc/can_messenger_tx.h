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
 
#ifndef CAN_MESSENGER_TX_H
#define CAN_MESSENGER_TX_H

#include <stdint.h>
#include "fw_common.h"
#include "bms.h"
#include "canfd_queue.h"

typedef status_t (*proc_func)(can_messages_e msg, uint32_t bus);

typedef struct 
{
    uint16_t can_msg_id;
    uint16_t can_msg_tx_len;
    uint16_t can_msg_rx_len;
    uint32_t can_msg_prio;
    uint32_t can_msg_type;
    proc_func pf;
}can_message_t;

status_t can_msg_send_process(canfd_queue_desc_t message, uint32_t bus);
status_t can_wake_msg_send(uint32_t bus);

#endif /* CAN_MESSENGER_TX_H */
