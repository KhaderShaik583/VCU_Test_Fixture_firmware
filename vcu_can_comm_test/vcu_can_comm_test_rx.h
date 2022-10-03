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
 
#ifndef CAN_MESSENGER_RX_H
#define CAN_MESSENGER_RX_H

#include "fw_common.h"

#include "flexcan_driver.h"
#include "canfd_queue.h"
#include "batterypack.h"
#include "odometer_task.h"

extern void can_dba_receive_test();
extern void can_mc_receive_test();
extern void can_fd_bms_receive_test();
status_t can_charger_receive();
void dba1_can_callback(uint8_t instance, flexcan_event_type_t eventType, uint32_t idx, flexcan_state_t *flexcanState);
void mc1_can_callback(uint8_t instance, flexcan_event_type_t eventType, uint32_t idx, flexcan_state_t *flexcanState);
void bms_can_callback(uint8_t instance, flexcan_event_type_t eventType, uint32_t idx, flexcan_state_t *flexcanState);
#endif /* CAN_MESSENGER_RX_H */
