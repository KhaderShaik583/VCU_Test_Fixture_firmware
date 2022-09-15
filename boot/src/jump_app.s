     PRESERVE8
     
     
     AREA THUMB, CODE, READONLY
     
     EXPORT jump_to_app
          
jump_to_app
     ldr r0,=0xE000ED08           ; Set R0 to VTOR address
     ldr r1,=0x00020000           ; User’s flash memory based address
     str r1, [r0]                ; Define beginning of user’s flash memory as vector table
     ldr r0, [r1]                ; Load initial MSP value
     mov sp, r0                     ; Set SP value (assume MSP is selected)
     ldr r0,     [r1, #4]           ; Load reset vector
     nop
     nop
     bx r0                          ; Branch to reset handler in user’s flash
     
     END
          