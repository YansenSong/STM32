.syntax unified
.cpu cortex-m4
.thumb

.section .isr_vector,"a",%progbits
.word 0x20020000
.word Boot_Reset
.rept 14
.word Boot_Fault
.endr

.text
.global Boot_Reset
.thumb_func
Boot_Reset:
    b boot_main

.thumb_func
Boot_Fault:
    b .

.global boot_jump
.thumb_func
boot_jump:
    cpsid i
    ldr r1, =0xe000ed08
    str r0, [r1]
    dsb
    isb
    ldr r1, [r0]
    ldr r2, [r0, #4]
    msr msp, r1
    cpsie i
    bx r2
