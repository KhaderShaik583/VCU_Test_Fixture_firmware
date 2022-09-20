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

void uart_check()
{	
	uint8_t rxdata[2] = {0x00, 0x00};
	uint8_t txdata[3] = {"ACK"};
	status_t s = STATUS_SUCCESS;
	
	s = LPUART_DRV_ReceiveDataBlocking(SYS_DEBUG_LPUART_INTERFACE,rxdata,1,1000);
	
	if (s == STATUS_SUCCESS)
	{
		LPUART_DRV_SendDataBlocking(SYS_DEBUG_LPUART_INTERFACE,rxdata,sizeof(rxdata),3);
//		vcu_2_bms_can_test_msg(1);
//		vcu_2_mc_send_rpdo_msg();
//		vcu_2_dba_send_test_msg();
	}
}

