    PRESERVE8
        
    AREA THUMB, CODE, READONLY
    
    IMPORT SystemCoreClock
    EXPORT sw_asm_delay_us

    ; void sw_asm_delay_us(uint32_t delay_us)
    ; Input parameter delay_us is in micro-second units [R0] 

    ALIGN
sw_asm_delay_us  FUNCTION
    
    push {r4-r7, lr}
    
    ldr r4, =SystemCoreClock
    ldr r5, [r4]                    ; r5 = SystemCoreClock
    
    ldr r4, =1000000                ; r4 = 1000000
    udiv r4, r5, r4                 ; r4 = r5 / r4 -> r4 = 80
    mul r4, r0, r4                  ; r4 = r0 * r4 -> delay_us * 80
    mov r5, #3                      ; account for cycles in loop for subs and bne
    udiv r4, r4, r5
    subs r4, #20                    ; account for initialization cycles before loop

loop
    subs r4, r4, #1
    bne loop

    nop
    nop
    pop {r4-r7, pc}
    
    ENDP

    END
        
        