/*
    ChibiOS/RT - Copyright (C) 2006,2007,2008,2009,2010,
                 2011,2012 Giovanni Di Sirio.

    This file is part of ChibiOS/RT.

    ChibiOS/RT is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    (at your option) any later version.

    ChibiOS/RT is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/**
 * @file    MIPS/MIPS32r2/vectors.s
 * @brief   Exception vectors for the MIPS32r2 family.
 *
 * @defgroup MIPS_MIPS32R2_VECTORS MIPS32r2 Exception Vectors
 * @ingroup MIPS_SPECIFIC
 * @details Exception vectors for the MIPS32r2 family.
 * @{
 */

#include "mipsasm.h"
#include "mipsarch.h"
#include "mipsparams.h"

#include "halconf.h"

#if !defined(__DOXYGEN__)

    .text

    /* A very simple exception handler. Does not support preemption since does not save all regs.
     * MIPS context is pretty heavyweight, so try to make it as simple as possible here. Exceptions are disabled.
     * ISR should be very lightweight and return quickly. All the other job should be done out of exception context.
     * That's the point of RTOS, isn't it =)? Also it's possible to run out of stack ...
     */
    .section .vectors
MIPS_FUNC_START(e_vector)
    /* Here we have only k0 and k1 regs, be carefull.
       Fetch exception number(into k0) and save CPU state. */

    /* FIXME: do via offsets of real structure */
    /* FIXME: use shadow regs */

    /* Do not save s[0-7] as they will be saved by callee(exception handler) if needed or during context switch.
       s[0-7] usage is not permitted here. */
    /* gp is not saved as no PIC/GOT is currently supported
       CPU status and config registers are not preserved as well as no interrupt preemption supported */

    subu    $sp, $sp, 88         /* sizeof(struct extctx) */

    .set noat
    sw      $at, 0  ($sp)
    sw      $v0, 4  ($sp)
    sw      $v1, 8  ($sp)
    sw      $a0, 12 ($sp)
    sw      $a1, 16 ($sp)
    sw      $a2, 20 ($sp)
    sw      $a3, 24 ($sp)
    sw      $t0, 28 ($sp)
    sw      $t1, 32 ($sp)
    sw      $t2, 36 ($sp)
    sw      $t3, 40 ($sp)
    sw      $t4, 44 ($sp)
    sw      $t5, 48 ($sp)
    sw      $t6, 52 ($sp)
    sw      $t7, 56 ($sp)
    sw      $t8, 60 ($sp)
    sw      $t9, 64 ($sp)
    sw      $fp, 68 ($sp)
    sw      $ra, 72 ($sp)
    .set at

    mfc0    $a0, cause          /* Passed to port_handle_exception */
    mfc0    $a1, status         /* Passed to port_handle_exception */
    
    mfhi    $t0
    mflo    $t1
    sw      $t0, 76 ($sp)
    sw      $t1, 80 ($sp)

    mfc0    $a2, epc            /* Passed to port_handle_exception */
    ehb
    sw      $a2, 84 ($sp)

    move    $k1, $sp            /* Save original SP in k1 */

    /* Switch to exception stack. Recall, nested exceptions are not supported here */
    .extern port_handle_exception
    la      $sp, exception_stack_bottom
    jal     port_handle_exception
    subu    $sp, $sp, MIPS_STACK_FRAME_SIZE

    /* Restore CPU state or maybe reschedule */

    .extern chSchIsPreemptionRequired
    jal     chSchIsPreemptionRequired
    nop
    
    move    $sp, $k1            /* Switch back to preempted task's SP */

    beqz    $v0, restore
    nop

    /* Interrupts are still disabled during the switch and restored when new task status reg is set */

    .extern chSchDoReschedule
    jal     chSchDoReschedule
    subu    $sp, $sp, MIPS_STACK_FRAME_SIZE

    addi    $sp, $sp, MIPS_STACK_FRAME_SIZE

restore:
    lw      $a0, 76 ($sp)
    lw      $a1, 80 ($sp)
    mthi    $a0
    mtlo    $a1

    .set noat
    lw      $at, 0  ($sp)
    lw      $v0, 4  ($sp)
    lw      $v1, 8  ($sp)
    lw      $a0, 12 ($sp)
    lw      $a1, 16 ($sp)
    lw      $a2, 20 ($sp)
    lw      $a3, 24 ($sp)
    lw      $t0, 28 ($sp)
    lw      $t1, 32 ($sp)
    lw      $t2, 36 ($sp)
    lw      $t3, 40 ($sp)
    lw      $t4, 44 ($sp)
    lw      $t5, 48 ($sp)
    lw      $t6, 52 ($sp)
    lw      $t7, 56 ($sp)
    lw      $t8, 60 ($sp)
    lw      $t9, 64 ($sp)
    lw      $fp, 68 ($sp)
    lw      $ra, 72 ($sp)
    .set at

    /* clear EXL, ERL and IE bits */
    mfc0  $k0, status
    li    $k1, 0xFFFFFFF0
    and   $k0, $k0, $k1
    mtc0  $k0, status
    ehb

    /* restore original PC */
    lw      $k0, 84 ($sp)
    /* ... and SP */
    addi    $sp, $sp, 88       /* sizeof(struct extctx) */

    /* return to the interrupted task with interrupts enabled */
    jr      $k0                 
    ei
    .set at
MIPS_FUNC_END(e_vector)

    /* System entry point */
    .section .entry
MIPS_FUNC_START(_start)
    /* Start in uncached mode */
    la      $k0, 0x2
    mtc0    $k0, config0

    /* Clear CPU status */
    mtc0    $zero, cause
    mtc0    $zero, status
    mtc0    $zero, status, 1

    /* Set exception base */
    la      $t0, e_vector
    lui     $t1, 0xFFFF
    and     $t0, $t0, $t1
    mtc0    $t0, $15, 1

    /* Setup initial context, run C setup code and finally call main() */

    /* <clear:bss> */
    /* Strongly assume that .bss START and END are word aligned */
    la      $t0, __bss_start__
    la      $t1, __bss_end__
    addiu   $t1, $t1, -4

1:  sw      $zero, 0($t0)
    bne     $t0, $t1, 1b
    addiu   $t0, $t0, 4
    /* </clear:bss> */

    /* <copy:data> */
    /* Copy .data from ROM to RAM if needed */
    /* Strongly assume that .data START and END are word aligned */
    la      $t0, __rom_data_start__
    la      $t1, __ram_data_start__
    beq     $t0, $t1, _early_init
    nop

    la      $t2, __ram_data_end__
    addiu   $t2, $t2, -4

2:  lw      $t3, 0($t0)
    sw      $t3, 0($t1)
    addiu   $t0, $t0, 4
    bne     $t1, $t2, 2b
    addiu   $t1, $t1, 4
    /* </copy:data> */

_early_init:
    /* FIXME: execute ctors */

    .extern port_early_init
    la      $sp, main_stack_bottom
    jal     port_early_init
    subu    $sp, $sp, MIPS_STACK_FRAME_SIZE

    .extern main
    la      $t0, main
    jalr    $t0
    nop
_life_after_main:
    b _start
    nop
MIPS_FUNC_END(_start)

    /* ============================================================ */
    
    .bss

    /* Separate stack for exception routines for simplicity */
    .balign 32
    .global exception_stack_top
exception_stack_top:
    .rept MIPS_EXC_STACK_SIZE
    .byte 0
    .endr
    .balign 32
    .global exception_stack_bottom
exception_stack_bottom:

    /* Separate stack for main() */
    .balign 32
    .global main_stack_top
main_stack_top:
    .global __main_thread_stack_base__
__main_thread_stack_base__:
    .rept MIPS_MAIN_STACK_SIZE
    .byte 0
    .endr
    .balign 32
    .global main_stack_bottom
main_stack_bottom:

#endif

/** @} */
