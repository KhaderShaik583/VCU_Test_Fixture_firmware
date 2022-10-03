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
 
#include <stdint.h>

#include "fw_features.h"
#include "vcu_can_comm_test_tx.h"
#include "vcu_can_comm_test_rx.h"
#include "lpuart_driver.h"
#include "Uart.h"
#include "wdt_task.h"

#ifndef USE_FEATURE_VCU_ON_DESK
#include "init_task.h"
#else
#include "init_task_desk.h"
#endif

int32_t main(void)
{
    (void)board_init();
    
    __DMB();
    
    (void)osif_kernel_init();
    
    __DSB();
    __ISB();

#ifndef USE_FEATURE_VCU_ON_DESK
    init_task_create();
#else
    init_task_desk_create();
#endif
    
    (void)osif_kernel_start();
	
    for(;; )
	{
//		can_dba_receive_test();

	}
    
    return 0;
}

