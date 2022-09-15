#ifndef FW_COMMON_H
#define FW_COMMON_H

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <arm_compat.h>
#include <arm_acle.h>

#include "S32K148.h" 
#include "system_S32K148.h"
#include "core_cm4.h" 
#include "interrupt_manager.h" 
#include "RTE_Components.h"
#include "cmsis_os2.h"
#include "status.h"
#include "devassert.h" 

#ifdef USE_DEBUG_PRINTS
#include "debug_console.h"
#endif

#include "fw_features.h"
#include "board.h" 
#include "osif.h"
#include "stat.h" 
#include "err_mgr.h"
#include "common_utils.h" 

#define SYSTICK_TICK_DURATION_DIV_100      (100U)   /* Gives a 10ms tick duration */
#define SYSTICK_TICK_DURATION_DIV_1000     (1000U)  /* Gives a 1ms tick duration */

#define UNUSED_PARAM(x) (void)(x)
#define UNUSED_VAR(x)   (void)(x)
    
#define LOG_API_VERSION (2U)
    
#define STACK_ALIGNMENT (8U)    /* Stack alignment boundary - 8 Bytes */
#define STACK_SIZE(x)   (((x) + (STACK_ALIGNMENT - ((x) % STACK_ALIGNMENT))))

extern void sw_asm_delay_us(uint32_t);

typedef struct {
    uint32_t os_err_code;
    uint32_t os_obj_id;
    char thread[32];
    uint32_t rsvd0;
    uint32_t rsvd1;
    uint32_t rsvd2;
    uint32_t rsvd3;
    uint32_t rsvd4;
}os_nvm_err_t;
    

#endif /* FW_COMMON_H */
