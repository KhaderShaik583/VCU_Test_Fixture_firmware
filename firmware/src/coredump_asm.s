        PRESERVE8
        
        AREA THUMB, CODE, READONLY
            
        IF   :DEF:USE_FEATURE_CORE_DUMP
            
        EXPORT HardFault_Handler
        IMPORT core_dump_stack
        IMPORT coredump_entry
        IMPORT core_dump_stack

CORE_DUMP_STACK_WORD_COUNT EQU   128
    
HardFault_Handler   PROC
        cpsid   i
        mrs     r12, msp
        ldr     r0, =(core_dump_stack + (4 * CORE_DUMP_STACK_WORD_COUNT))
        mov     sp, r0
        mov     r0, r8
        mov     r1, r9
        mov     r2, r10
        mov     r3, r11
        push    {r0-r3, lr}
        mrs     r3, xpsr
        mrs     r2, psp
        mov     r1, r12
        push    {r1-r7}
        mov     r0, sp
        cpsie   i
        
        bl      coredump_entry    
        
        ENDP
        
            
UsageFault_Handler   PROC
        dsb
        dmb
        mrs     r12, msp
        ldr     r0, =(core_dump_stack + (4 * CORE_DUMP_STACK_WORD_COUNT))
        mov     sp, r0
        mov     r0, r8
        mov     r1, r9
        mov     r2, r10
        mov     r3, r11
        push    {r0-r3, lr}
        mrs     r3, xpsr
        mrs     r2, psp
        mov     r1, r12
        push    {r1-r7}
        mov     r0, sp
        
        bl      coredump_entry    
        
        
        ENDP


BusFault_Handler   PROC
        dsb
        dmb
        mrs     r12, msp
        ldr     r0, =(core_dump_stack + (4 * CORE_DUMP_STACK_WORD_COUNT))
        mov     sp, r0
        mov     r0, r8
        mov     r1, r9
        mov     r2, r10
        mov     r3, r11
        push    {r0-r3, lr}
        mrs     r3, xpsr
        mrs     r2, psp
        mov     r1, r12
        push    {r1-r7}
        mov     r0, sp
        
        bl      coredump_entry    
        
        
        ENDP

            
MemManage_Handler   PROC
        dsb
        dmb
        mrs     r12, msp
        ldr     r0, =(core_dump_stack + (4 * CORE_DUMP_STACK_WORD_COUNT))
        mov     sp, r0
        mov     r0, r8
        mov     r1, r9
        mov     r2, r10
        mov     r3, r11
        push    {r0-r3, lr}
        mrs     r3, xpsr
        mrs     r2, psp
        mov     r1, r12
        push    {r1-r7}
        mov     r0, sp
        
        bl      coredump_entry    
        
        
        ENDP
        ENDIF
            
WDOG_EWM_IRQHandler PROC
    
        ENDP
            
        END
            
        END
            