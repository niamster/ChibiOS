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

#include "vectors.inc"

#if !defined(__DOXYGEN__)


  MIPS_SECTION_CODE(.vectors)

MIPS_FUNC_START(e_vector)
  /* FIXME: do via offsets of real structure */

  /* A very simple exception handler. Does not support preemption since does not save all regs.
   * MIPS context is pretty heavyweight, so try to make it as simple as possible here. Exceptions are disabled.
   * ISR should be very lightweight and return quickly. All the other job should be done out of exception context.
   * That's the point of RTOS, isn't it =)? Also it's possible to run out of stack ...
   */

  /* Here we have only k0 and k1 regs, be carefull. */

  /* Do not save s[0-7] as they will be saved by callee(exception handler) if needed or during context switch.
     s[0-7] usage is not permitted here. */
  /* gp is not saved as no PIC/GOT is currently supported
     CPU status and config registers are not preerved as well as no interrupt preemption supported */

  isr_save_ctx

  mfc0    $a0, cause          /* Passed to port_handle_exception */
  mfc0    $a1, status         /* Passed to port_handle_exception */
    
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
    
  move    $sp, $k1            /* Switch back to preempted task's SP */

  beqz    $v0, resume
  nop

  /* Interrupts are still disabled(EXL=1 or ERL=1) during the switch and restored when new task status reg is set */

  .extern chSchDoReschedule
#if defined(MIPS_USE_MIPS16_ISA)
  jalx    chSchDoReschedule
#else
  jal     chSchDoReschedule
#endif
  subu    $sp, $sp, MIPS_STACK_FRAME_SIZE

  addi    $sp, $sp, MIPS_STACK_FRAME_SIZE

resume:
  /* clear EXL, ERL and IE(enabled later with ei) bits */
  mfc0    $k0, status
  li      $k1, 0xFFFFFFF0
  and     $k0, $k0, $k1
  mtc0    $k0, status

restore:
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
MIPS_FUNC_END(e_vector)

#endif

/** @} */
