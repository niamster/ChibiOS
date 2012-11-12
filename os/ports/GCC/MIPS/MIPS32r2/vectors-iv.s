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

#define MIPS_IVECTOR_X(x)           \
MIPS_FUNC_START(i_vector ## x)      \
    j       i_vector;               \
    li      $a1, x;                 \
    .rept 6;                        \
    nop;                            \
    .endr;                          \
MIPS_FUNC_END(i_vector ## x)


    .text

    .section .evectors

MIPS_FUNC_START(gen_vector)
    j       e_vector
    nop
MIPS_FUNC_END(gen_vector)


    .section .ivectors

  MIPS_IVECTOR_X(0)
  MIPS_IVECTOR_X(1)
  MIPS_IVECTOR_X(2)
  MIPS_IVECTOR_X(3)
  MIPS_IVECTOR_X(4)
  MIPS_IVECTOR_X(5)
  MIPS_IVECTOR_X(6)
  MIPS_IVECTOR_X(7)
#if MIPS_EXC_TABLE_SIZE > 8
  MIPS_IVECTOR_X(8)
  MIPS_IVECTOR_X(9)
  MIPS_IVECTOR_X(10)
  MIPS_IVECTOR_X(11)
  MIPS_IVECTOR_X(12)
  MIPS_IVECTOR_X(13)
  MIPS_IVECTOR_X(14)
  MIPS_IVECTOR_X(15)
  MIPS_IVECTOR_X(16)
  MIPS_IVECTOR_X(17)
  MIPS_IVECTOR_X(18)
  MIPS_IVECTOR_X(19)
  MIPS_IVECTOR_X(20)
  MIPS_IVECTOR_X(21)
  MIPS_IVECTOR_X(22)
  MIPS_IVECTOR_X(23)
  MIPS_IVECTOR_X(24)
  MIPS_IVECTOR_X(25)
  MIPS_IVECTOR_X(26)
  MIPS_IVECTOR_X(27)
  MIPS_IVECTOR_X(28)
  MIPS_IVECTOR_X(29)
  MIPS_IVECTOR_X(30)
  MIPS_IVECTOR_X(31)
  MIPS_IVECTOR_X(32)
  MIPS_IVECTOR_X(33)
  MIPS_IVECTOR_X(34)
  MIPS_IVECTOR_X(35)
  MIPS_IVECTOR_X(36)
  MIPS_IVECTOR_X(37)
  MIPS_IVECTOR_X(38)
  MIPS_IVECTOR_X(39)
  MIPS_IVECTOR_X(40)
  MIPS_IVECTOR_X(41)
  MIPS_IVECTOR_X(42)
  MIPS_IVECTOR_X(43)
  MIPS_IVECTOR_X(44)
  MIPS_IVECTOR_X(45)
  MIPS_IVECTOR_X(46)
  MIPS_IVECTOR_X(47)
  MIPS_IVECTOR_X(48)
  MIPS_IVECTOR_X(49)
  MIPS_IVECTOR_X(50)
  MIPS_IVECTOR_X(51)
  MIPS_IVECTOR_X(52)
  MIPS_IVECTOR_X(53)
  MIPS_IVECTOR_X(54)
  MIPS_IVECTOR_X(55)
  MIPS_IVECTOR_X(56)
  MIPS_IVECTOR_X(57)
  MIPS_IVECTOR_X(58)
  MIPS_IVECTOR_X(59)
  MIPS_IVECTOR_X(60)
  MIPS_IVECTOR_X(61)
  MIPS_IVECTOR_X(62)
  MIPS_IVECTOR_X(63)
#endif


#if defined(MIPS_USE_SHADOW_GPR)
MIPS_FUNC_START(i_vector)
    /* FIXME: do via offsets of real structure */

    /* A very simple exception handler that uses shadow registers for ISRs.
     * Does not support preemption. All other exceptions are disabled.
     * ISR should be very lightweight and return quickly. All the other job should be done out of exception context.
     * That's the point of RTOS, isn't it =)? Also it's possible to run out of stack ...
     */

    mfc0    $a0, cause          /* Passed to port_handle_irq */
    ehb

    /* FIXME: do we have to save mfhi/mflo here? */

    /* Switch to exception stack. Recall, nested exceptions are not supported here */
    .extern port_handle_exception
    la      $sp, exception_stack_bottom
    jal     port_handle_irq
    subu    $sp, $sp, MIPS_STACK_FRAME_SIZE

    /* Restore CPU state or maybe reschedule */

    .extern chSchIsPreemptionRequired
    jal     chSchIsPreemptionRequired
    nop

    bnez    $v0, vi_resched
    nop

    /* Do nothing but return from exception */
    eret                        /* PC <- EPC, EXL = ERL = 0, SRSCtlCSS <- SRSCtlPSS */


  /* Interrupts are still disabled(EXL=1 or ERL=1) during the switch and restored when new task status reg is set */
vi_resched:
    /* Switch to previous SRS */
    di
    mfc0    $k0, epc
    la      $k1, savedEPC
    sw      $k0, 0($k1)
    la      $k0, oldSRS
    mtc0    $k0, epc
    eret

oldSRS:
    subu    $sp, $sp, 88        /* sizeof(struct extctx) */

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

    mfhi    $t0
    mflo    $t1
    sw      $t0, 76 ($sp)
    sw      $t1, 80 ($sp)

    la      $t0, savedEPC
    lw      $t0, 0($t0)
    sw      $t0, 84 ($sp)

    .extern chSchDoReschedule
    jal     chSchDoReschedule
    subu    $sp, $sp, MIPS_STACK_FRAME_SIZE

    addi    $sp, $sp, MIPS_STACK_FRAME_SIZE

iv_restore:
    lw      $t0, 76 ($sp)
    lw      $t1, 80 ($sp)
    mthi    $t0
    mtlo    $t1

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

    /* restore original PC */
    lw      $k0, 84 ($sp)
    /* ... and SP */
    addi    $sp, $sp, 88        /* sizeof(struct extctx) */

    /* return to the interrupted task with interrupts enabled */
    jr      $k0                 
    ei
    .set at
MIPS_FUNC_END(i_vector)
#else
#error IV mode without SRS is not suppoted
#endif

#include "vectors-single.s"
  
    .bss

  /* old EPC when switching SRS */
savedEPC:
  .word 0
  

#endif

/** @} */
