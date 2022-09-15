 /*
 * 
 * ULTRAVIOLETTE AUTOMOTIVE CONFIDENTIAL
 * ______________________________________
 * 
 * [2021] - [2022] Ultraviolette Automotive Pvt. Ltd.
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
 

#ifndef BMS_TASK_H
#define BMS_TASK_H

#include "fw_common.h"
#include "flexcan_driver.h"

#define BMS_TASK_NAME        "thread_bms"
#define BMS_STACK_SIZE       STACK_SIZE(256U)

#define BMS_TASK_START_FLAG     (0x0001U)
#define BMS_TASK_WAKE_NTF_FLAG  (0x0002U)

#define BMS_MSG_ASYNC_FET_OFF       (0xB00U)
#define BMS_MSG_ASYNC_FET_ON        (0xB01U)
#define BMS_MSG_POST_FW_UPD_EVENT   (0xB02U)
#define BMS_MSG_PWR_MODE            (0xB03U)
#define BMS_MSG_FLUSH_CAN_MSG_QUEUE (0xB04U)
#define BMS_MSG_KEEP_ALIVE_NO_DATA  (0xB05U)
#define BMS_MSG_CLEAR_TAMPER_STATE  (0xB06U)

typedef struct 
{
    uint32_t msg_id;
    uint8_t data;
}bms_msg_queue_obj_t;

extern osif_msg_queue_id_t bms_msg_queue;

void can_if_vcu_receive_nb(void);
void bms_task_create(void);
osThreadId_t bms_task_get_id(void);
status_t can_if_init_bms(void);
uint32_t bms_is_motor_active(void);
status_t can_fd_if_bms_send(uint8_t *buffer, uint16_t len, uint32_t msg_id, uint32_t bus);
uint32_t bms_get_charger_state(void);

#endif /* BMS_TASK_H */
