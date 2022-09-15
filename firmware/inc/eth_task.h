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
 
#ifndef ETH_TASK_H
#define ETH_TASK_H

#include <stdint.h>

#include "fw_common.h"
#include "net_if_cfg.h"

#include "enet_driver.h"
#include "NXP_SJA1105P_diagnostics.h"
#include "NXP_SJA1105P_switchCore.h" 
#include "NXP_SJA1105P_sgmii.h"

#define ENET_INSTANCE           (0U)
#define ENET_QUEUE              (0U)
#define ETHERNET0_NUM_RINGS0    (1U)

#define ENET_RXBD_NUM0_0        (1U)
#define ENET_TXBD_NUM0_0        (1U)

#define ENET_RXBUFF_SIZE        ENET_BUFF_ALIGN(ENET_FRAME_MAX_FRAMELEN)

#define ETH_TASK_NAME           "thread_eth"
#define ETH_TASK_STACK_SIZE     STACK_SIZE(1024U)
#define ETH_TASK_PRIO           osPriorityNormal

#define ETH_MAX_FRAMELEN        1500U

/* Thread Flags */
#define ETH_TASK_START_FLAG     (0x0001U)
#define ETH_TASK_RX_EVENT       (0x0010U)

#define FORWARD_FRAME   (0U)

typedef uint32_t (*rx_buff_process_condition_handler_t)(uint8_t eth_instance, enet_buffer_t *buff);
extern mutex_t enetif_tx_lock;

/* API's exposed to the Application */
#define ETHIF_INIT              enet_ethernetif_init
#define ETHIF_SHUTDOWN          enet_ethernetif_shutdown
#define ETHIF_BUFFER_t          enet_buffer_t
#define ETHIF_POLL_INTERFACE    enet_poll_interface

#define ETHIF_REGISTER_RX_BUFF_PROCESS_CONDITION_HANDLER \
	enetif_register_rx_buff_process_condition_handler

err_t enet_ethernetif_init(struct netif *netif);
void enet_ethernetif_shutdown(struct netif *netif);
void enetif_register_rx_buff_process_condition_handler(rx_buff_process_condition_handler_t handler);

void eth_task_create(void);
osThreadId_t eth_task_get_id(void);
void net_init(void);
void net_tasks_init(void);
void net_tasks_deinit(void);

#endif /* ETH_TASK_H */
