 /*
 * 
 * ULTRAVIOLETTE AUTOMOTIVE CONFIDENTIAL
 * ______________________________________
 * 
 * [2020] - [2021] Ultraviolette Automotive Pvt. Ltd.
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

#include "fw_common.h"
#include "stat.h"

static volatile uint64_t vcu_status_bits;

void clear_select_bits(void)
{
    INT_SYS_DisableIRQGlobal();
    
    vcu_status_bits = vcu_status_bits & 0x000000000000007fU;
    
    INT_SYS_EnableIRQGlobal();
}

uint64_t get_status(void)
{
    return vcu_status_bits;
}

void set_status_bit(vcu_status_e b)
{
    INT_SYS_DisableIRQGlobal();
    
    vcu_status_bits = vcu_status_bits | ((uint64_t)1U << (uint64_t)b);
    
    INT_SYS_EnableIRQGlobal();

}

void clear_status_bit(vcu_status_e b)
{
    INT_SYS_DisableIRQGlobal();
    
    vcu_status_bits = vcu_status_bits & ~((uint64_t)1U << (uint64_t)b);
    
    INT_SYS_EnableIRQGlobal();
}

void clear_all_flags(void)
{
    INT_SYS_DisableIRQGlobal();
    
    vcu_status_bits = (uint64_t)0U;
    
    INT_SYS_EnableIRQGlobal();
}


