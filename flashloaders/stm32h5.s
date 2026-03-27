    .syntax unified
    .text

    /*
     * Arguments:
     *   r0 - source SRAM ptr
     *   r1 - target flash ptr
     *   r2 - exact count of bytes to program
     *   r3 - flash bank selector (0 = bank 1, 1 = bank 2)
     *
     * Return:
     *   r0 - last observed NSSR value
     *   r2 - remaining bytes (0 on success)
     */

    .global copy
copy:
    cmp r3, #0
    beq bank1

bank2:
    ldr r12, flash_bank2_nssr
    ldr r11, flash_bank2_nscr
    ldr r10, flash_bank2_nsccr
    b prepare

bank1:
    ldr r12, flash_bank1_nssr
    ldr r11, flash_bank1_nscr
    ldr r10, flash_bank1_nsccr

prepare:
    mrs r3, primask
    cpsid i
    sub sp, sp, #16
    ldr r9, flash_clear_mask
    ldr r8, flash_error_mask
    str r9, [r10]

wait_ready:
    ldr r4, [r12]

    # wait until BSY, WBNE and DBNE are all clear
    tst r4, #0xb
    bne wait_ready

    # keep PG set for the full chunk write
    ldr r4, [r11]
    orr r4, r4, #0x2
    str r4, [r11]

loop:
    cmp r2, #0
    ble exit

    cmp r2, #16
    blo tail

    # copy one quadword as four consecutive 32-bit writes
    ldmia r0!, {r4-r7}
    b program_quad

tail:
    mvn r4, #0
    str r4, [sp, #0]
    str r4, [sp, #4]
    str r4, [sp, #8]
    str r4, [sp, #12]
    mov r5, sp
    mov r6, r2

tail_copy:
    cmp r6, #0
    beq tail_load
    ldrb r4, [r0], #1
    strb r4, [r5], #1
    subs r6, r6, #1
    b tail_copy

tail_load:
    ldmia sp, {r4-r7}

program_quad:
    stmia r1!, {r4-r7}
    dsb sy

wait:
    ldr r4, [r12]

    # wait until BSY, WBNE and DBNE are all clear
    tst r4, #0xb
    bne wait

    # stop early and leave r2 pointing at the first unwritten quadword
    tst r4, r8
    bne exit

    cmp r2, #16
    blo tail_done
    subs r2, r2, #16
    b loop

tail_done:
    movs r2, #0
    b loop

exit:
    mov r0, r4
    ldr r4, [r11]
    bic r4, r4, #0x2
    str r4, [r11]
    str r9, [r10]
    add sp, sp, #16
    msr primask, r3
    bkpt

    .align 2
flash_bank1_nssr:
    .word 0x40022020
flash_bank1_nscr:
    .word 0x40022028
flash_bank1_nsccr:
    .word 0x40022030
flash_bank2_nssr:
    .word 0x50022020
flash_bank2_nscr:
    .word 0x50022028
flash_bank2_nsccr:
    .word 0x50022030
flash_clear_mask:
    .word 0x009f0000
flash_error_mask:
    .word 0x009e0000
