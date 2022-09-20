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
 
#ifndef BMS_CAN_IF_H
#define BMS_CAN_IF_H

#include "fw_common.h"

void can_fd_bms_receive(uint32_t bus_id);
uint32_t can_fd_get_rx_state(uint32_t bus);
void can_fd_reset_rx_state(void);
uint32_t canfd_get_bus_en_state(uint32_t bus);
void can_fd_if_bms_err_process(uint32_t *esr);
//void can_fd_if_bms_init(void);
status_t can_fd_if_bms_send(uint8_t *buffer, uint16_t len, uint32_t msg_id, uint32_t bus_id);
void canfd_disable_bus(uint32_t bus);
void canfd_enable_bus(uint32_t bus);

#endif /* BMS_CAN_IF_H */
