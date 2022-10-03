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
 
#include "lpuart_driver.h"
#include "vcu_can_comm_test_tx.h"
#include "fw_common.h"
#define LTE_UART_TASK_DELAY         (1000U)
#define MAX_UART_PAYLOAD            (64U)

static int32_t uart_if_receive_nb(void);
	
static uint8_t tx_buffer[MAX_UART_PAYLOAD] = "UV UART TEST MESG\n\r";
static uint8_t rx_buffer[MAX_UART_PAYLOAD];
typedef enum
{
    UART_STATE_START_RX = 0U,
    UART_STATE_WAIT_RX,
    
    
}uart_rx_states_e;	
static uart_rx_states_e rx_state = UART_STATE_START_RX;

void uart_check()
{	
	status_t ret = STATUS_ERROR;
	uint32_t bytesRemaining;
//	uart_if_receive_nb();
        
//		LPUART_DRV_SendDataBlocking(SYS_DEBUG_LPUART_INTERFACE, tx_buffer, MAX_UART_PAYLOAD, 200);
		ret = LPUART_DRV_ReceiveDataBlocking(SYS_DEBUG_LPUART_INTERFACE, rx_buffer, 1, 3000);
		
		if(ret == STATUS_SUCCESS)
		{
			ret = LPUART_DRV_SendDataBlocking(SYS_DEBUG_LPUART_INTERFACE, tx_buffer, MAX_UART_PAYLOAD, 200);
		}

}

//void ThisIsMyRXCallback()

//{

//    LPUART_DRV_ReceiveData(SYS_DEBUG_LPUART_INTERFACE, rx_buffer, 1UL);

//   dbg_printf("We got: %s\n",rx_buffer);

//}

//static int32_t uart_if_receive_nb(void)
//{
//	status_t s = STATUS_ERROR;
//	int32_t ret = 0;
//	uint32_t bytesRemaining = 2;
//		switch(rx_state)
//		{
//			case UART_STATE_START_RX:
//				LPUART_DRV_ReceiveData(SYS_DEBUG_LPUART_INTERFACE, rx_buffer, 2);
//				rx_state = UART_STATE_WAIT_RX;
//            break;
//			
//			case UART_STATE_WAIT_RX:
//				s = LPUART_DRV_GetReceiveStatus(SYS_DEBUG_LPUART_INTERFACE, &bytesRemaining);
//					if(s == STATUS_SUCCESS)
//					{
//						dbg_printf(" Status = %d ", s);
//					}
//					else
//					{
//						rx_state = UART_STATE_WAIT_RX;
//					}
//				
//		}
//		
//	return ret;
//}

