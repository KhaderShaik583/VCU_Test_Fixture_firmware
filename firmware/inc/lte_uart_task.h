 /*
 * 
 * ULTRAVIOLETTE AUTOMOTIVE CONFIDENTIAL
 * ______________________________________
 * 
 * [2017] - [2018] Ultraviolette Automotive Pvt. Ltd.
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
 
#ifndef LTE_UART_TASK_H
#define LTE_UART_TASK_H

#include "fw_common.h"

#define LTE_UART_TASK_NAME          "thread_lte_uart"
#define LTE_UART_TASK_STACK_SIZE    STACK_SIZE(64U)
#define MAX_LTE_UART_BUF_LEN        (512U)

#define LTE_UART_TASK_START_FLAG    (0x0001U)

#define LTE_UART_TASK_DELAY         (1000U)
#define MAX_UART_PAYLOAD            (64U)

void lte_uart_task_create(void);
osThreadId_t lte_uart_task_get_id(void);

#endif /* LTE_UART_TASK_H */
