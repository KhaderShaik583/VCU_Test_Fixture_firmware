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

#include "lte_uart_task.h"
#include "pins_driver.h"
#include "osif.h"
#include "lpuart_driver.h" 
#include "rtx_os.h"
#include "wdt_task.h"

#ifdef USE_FEATURE_RTX_VIEWER
__attribute__((section(".bss.os.thread.cb")))
thread_tcb_t lte_uart_thread_tcb;
#else
static thread_tcb_t lte_uart_thread_tcb;
#endif /* USE_FEATURE_RTX_VIEWER */

static thread_id_t thread_lte_uart;

__attribute__((section("ARM_LIB_STACK")))
static uint64_t lte_uart_thread_stk[LTE_UART_TASK_STACK_SIZE];

static uint8_t tx_buffer[MAX_UART_PAYLOAD] = "UV UART TEST MESG\n\r";
static uint8_t rx_buffer[MAX_UART_PAYLOAD];

static const thread_attrs_t lte_uart_task_attr = {
    LTE_UART_TASK_NAME,
    osThreadDetached,
    &lte_uart_thread_tcb,
    sizeof(lte_uart_thread_tcb),
    &lte_uart_thread_stk[0],
    LTE_UART_TASK_STACK_SIZE * sizeof(uint64_t),
    osPriorityNormal,
    0U,
    0U    
};

__NO_RETURN static void lte_uart_task(void *arg)
{
    status_t ret = STATUS_SUCCESS;
    
    UNUSED_PARAM(arg);
    
    (void)osif_thread_wait_on_flag(LTE_UART_TASK_START_FLAG, OSIF_WAIT_ANY_FLAG, OSIF_WAIT_FOREVER);
    (void)osif_thread_clear_flag(LTE_UART_TASK_START_FLAG);

    while(true)
    {
        ret = LPUART_DRV_SendDataBlocking(SYS_LTE_UART_IF, tx_buffer, MAX_UART_PAYLOAD, 10U);
        ret = LPUART_DRV_ReceiveDataBlocking(SYS_LTE_UART_IF, rx_buffer, MAX_UART_PAYLOAD, LTE_UART_TASK_DELAY);   

        __NOP();
    } 
}

thread_id_t lte_uart_task_get_id(void)
{
    return thread_lte_uart;
}

void lte_uart_task_create(void)
{
    uint32_t param = NULL;
    
    thread_lte_uart = osif_thread_create(lte_uart_task, &param, &lte_uart_task_attr);
    DEV_ASSERT(thread_lte_uart != NULL);
}
