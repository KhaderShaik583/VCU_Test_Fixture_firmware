#ifndef CORE_DUMP_H
#define CORE_DUMP_H

#include <stdint.h>
#include "mpu_driver.h"
#include "mpu_hw_access.h" 

/* 
    This structure contains the registers that are automatically 
    stacked by Cortex-M processor when it enters an exception handler. 
*/
typedef struct
{
    uint32_t r0;
    uint32_t r1;
    uint32_t r2;
    uint32_t r3;
    uint32_t r12;
    uint32_t lr;
    uint32_t pc;
    uint32_t psr;
}stacked_registers;

/* 
    This structure is filled in by the Hard Fault exception handler 
*/
typedef struct
{
    uint32_t msp;
    uint32_t psp;
    uint32_t exceptionPSR;
    uint32_t r4;
    uint32_t r5;
    uint32_t r6;
    uint32_t r7;
    uint32_t r8;
    uint32_t r9;
    uint32_t r10;
    uint32_t r11;
    uint32_t exceptionLR;
}exception_registers;


typedef struct
{
    uint32_t mfar;
    uint32_t bfar;
    uint32_t cfsr;
    uint32_t hfsr;
    uint32_t dfsr;
    uint32_t afsr;    
}fault_reports;

typedef struct
{
    uint32_t sentinel;
    const exception_registers *pExceptionRegisters;
    const stacked_registers *pSP;
    mpu_access_err_info_t mmu_exception;
    fault_reports fr;
    uint32_t sp;
    uint32_t flags;
    uint32_t dump_size;
}core_dump;


#define CORE_DUMP_STACK_WORD_COUNT  (128)
#define CORE_DUMP_STACK_SENTINEL    (0xDEADBABE)

uint32_t get_cd_stk_base(void);

#endif /* CORE_DUMP_H */
