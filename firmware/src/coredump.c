#include "fw_common.h"
#include "fw_features.h"
#include "wdog_driver.h"
#include "coredump.h"
#include "board.h"
#include "drv_led.h"
#include "nvm.h"
#include "mc_task.h" 

#ifdef USE_FEATURE_CORE_DUMP

extern void coredump_entry(const exception_registers *pExceptionRegisters);

/* Bit in LR to indicate whether PSP was used for automatic stacking of registers during exception entry. */
#define LR_PSP (1 << 2)

/* Bit in LR set to 0 when automatic stacking of floating point registers occurs during exception handling. */
#define LR_FLOAT (1 << 4)

/* Bit in auto stacked xPSR which indicates whether stack was force 8-byte aligned. */
#define PSR_STACK_ALIGN (1 << 9)

uint32_t core_dump_stack[CORE_DUMP_STACK_WORD_COUNT];

#ifdef USE_DEBUG_PRINTS
static const char *mpu_err_attrib_str[4] = {
    "MPU_INSTRUCTION_ACCESS_IN_USER_MODE",
    "MPU_DATA_ACCESS_IN_USER_MODE",
    "MPU_INSTRUCTION_ACCESS_IN_SUPERVISOR_MODE",
    "MPU_DATA_ACCESS_IN_SUPERVISOR_MODE"
};

static const char *mpu_access_type_str[2] = {
    "MPU_ERR_TYPE_READ",
    "MPU_ERR_TYPE_WRITE"
};

#endif /* USE_DEBUG_PRINTS */

/*FUNCTION**********************************************************************
 *
 * Function Name : get_exception_stack_addr
 * Description   : Gets the exception stack address.
 *
 * Implements    : get_exception_stack_addr_Activity
 *END**************************************************************************/

static uint32_t get_exception_stack_addr(const exception_registers *pExceptionRegisters)
{
    if (pExceptionRegisters->exceptionLR & LR_PSP)
    {
        return pExceptionRegisters->psp;
    }
    else
    {
        return pExceptionRegisters->msp;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SP_value_before_exception
 * Description   : Gets the SP value before exception.
 *
 * Implements    : SP_value_before_exception_Activity
 *END**************************************************************************/

static void SP_value_before_exception(core_dump *pDump)
{
    /* Cortex-M processor always push 8 integer registers on the stack. */
    pDump->sp += 8 * sizeof(uint32_t);
    
    /* Cortex-M processor may also have had to force 8-byte alignment before auto stacking registers. */
    if((pDump->pSP->psr & PSR_STACK_ALIGN) == PSR_STACK_ALIGN)
    {
        pDump->sp = pDump->sp | (uint32_t)4U;
    }
    else
    {
        pDump->sp = pDump->sp | (uint32_t)0U;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : init_stack_pointer
 * Description   : Initialize the stack pointer.
 *
 * Implements    : init_stack_pointer_Activity
 *END**************************************************************************/

static core_dump init_stack_pointer(const exception_registers *pExceptionRegisters)
{
    core_dump cd;
    uint8_t slave_num = 0U;

    cd.pExceptionRegisters = pExceptionRegisters;
    cd.sp = get_exception_stack_addr(pExceptionRegisters);
    cd.pSP = (const void*)(unsigned long)(cd.sp);
    
    /* Check the 4 slaves for errors */
    for(slave_num = 0U; slave_num < FEATURE_MPU_SLAVE_WIDTH; slave_num++)
    {
        if(true == MPU_GetErrorStatus(MPU, slave_num))
        {
            break;
        }
    }
    
    if(slave_num < FEATURE_MPU_SLAVE_WIDTH)
    {
        (void)MPU_DRV_GetDetailErrorAccessInfo(0U, slave_num, &cd.mmu_exception);
    }
    
    cd.flags = 0;
   
    return cd;
}

#ifdef USE_DEBUG_PRINTS
/*FUNCTION**********************************************************************
 *
 * Function Name : lpuart_print_cb
 * Description   : Minimal blocking API for UART serial prints. Overrides the default callback
 * in the debug console structure. This is a polling implementation that runs in exception context
 * IRQ based transfers will not execute in exception context.
 * Implements    : lpuart_print_cb
 *END**************************************************************************/
status_t lpuart_print_cb(uint32_t instance, const uint8_t *buffer, uint32_t length, uint32_t timeout)
{
    (void)instance;
    (void)timeout;

    LPUART1->CTRL |= (1U << 19U);
    while (length--)
    {
        while (!(LPUART1->STAT & LPUART_STAT_TDRE_MASK))
        {
            
        }
        LPUART1->DATA = *(buffer++);
        /*ext_wdt_kick();*/
    }    
    
    return STATUS_SUCCESS;
}
#endif /* USE_DEBUG_PRINTS */

static void coredump_save_nv(core_dump cd)
{
    uint32_t cd_nv_base_addr = FILE_SECT_COREDUMP_INFO;
    uint32_t base_save = FILE_SECT_COREDUMP_INFO;
    uint8_t cd_size = 0U;

    /* Base address + offset 0 has indication if core dump is present or not */
    /* 0xB5 -> Core dump present, 0xBA -> Core dump not present */
    /* 
        Update core dump sentinel to indicate core dump present. When the dump will be 
        read by the ICP command this location must be set to 0xBA 
    */
    (void)nvm_write_byte(cd_nv_base_addr, 0xB5U);
    cd_nv_base_addr++;
    
    /* Number of bytes of coredump data. Updated later */
    cd_nv_base_addr++;
    
    (void)nvm_write(cd_nv_base_addr, (uint8_t *)cd.pExceptionRegisters, sizeof(exception_registers));
    cd_nv_base_addr = cd_nv_base_addr + sizeof(exception_registers);
    
    (void)nvm_write(cd_nv_base_addr, (uint8_t *)cd.pSP, sizeof(stacked_registers));
    cd_nv_base_addr = cd_nv_base_addr + sizeof(stacked_registers); 

    (void)nvm_write(cd_nv_base_addr, (uint8_t *)&cd.mmu_exception, sizeof(mpu_access_err_info_t));
    cd_nv_base_addr = cd_nv_base_addr + sizeof(mpu_access_err_info_t); 
    
    /* Fault report registers*/
    cd.fr.mfar = S32_SCB->MMFAR;
    cd.fr.bfar = S32_SCB->BFAR;
    cd.fr.cfsr = S32_SCB->CFSR;
    cd.fr.hfsr = S32_SCB->HFSR;
    cd.fr.dfsr = S32_SCB->DFSR;
    cd.fr.afsr = S32_SCB->AFSR;
    
    (void)nvm_write(cd_nv_base_addr, (uint8_t *)&cd.fr, sizeof(fault_reports));
    cd_nv_base_addr = cd_nv_base_addr + sizeof(fault_reports);        
    
    cd_size = (uint8_t)(cd_nv_base_addr - base_save) - 2U;
        
    (void)nvm_write_byte(base_save + 1U, cd_size);
    
    __NOP();
    
}

/*FUNCTION**********************************************************************
 *
 * Function Name : coredump_entry
 * Description   : Entry point into the coredump functionality. Called from 
 * the hardfault handler.
 *
 * Implements    : coredump_entry_Activity
 *END**************************************************************************/

void coredump_entry(const exception_registers *pExceptionRegisters)
{
    /* Disable WDT ? */
    S32_SCB->CCR = 0U;
    
    (void)WDOG_DRV_Deinit(0U);
    
    mc_set_gear(MC_GEAR_POS_NEUTRAL);
    set_status_bit(STAT_VCU_SW_EXCEPTION);
    
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    
    core_dump cd = init_stack_pointer(pExceptionRegisters);
    SP_value_before_exception(&cd);
    
    coredump_save_nv(cd);
    
    SystemSoftwareReset();
}

#endif /* USE_FEATURE_CORE_DUMP */
