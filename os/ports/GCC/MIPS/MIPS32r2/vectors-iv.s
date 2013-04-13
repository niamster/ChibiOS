/*
    ChibiOS/RT - Copyright (C) 2006-2013 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/
/*
   Concepts and parts of this file have been contributed by Dmytro Milinevskyy <milinevskyy@gmail.com>
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

#if defined(MIPS_USE_SHADOW_GPR)
  /* FIXME: do we have to save mfhi/mflo here? */
  .extern port_handle_irq
  /* A very simple exception handler that uses shadow registers for ISRs.
   * Does not support preemption.
   * ISR should be very lightweight and return quickly. All the other job should be done out of interrupt context.
   * That's the point of RTOS, isn't it =)?
   */
#define MIPS_IVECTOR_X(x)         \
  MIPS_FUNC_START(i_vector ## x)  \
  li      $a0, x;                 \
                                  \
  la      $sp, irq_stack_ready;   \
  jal     port_handle_irq;        \
  mfc0    $a1, cause;             \
                                  \
  bnez    $v0, srs_resched;       \
  nop;                            \
                                  \
  eret;                           \
MIPS_FUNC_END(i_vector ## x)
#else
#define MIPS_IVECTOR_X(x)         \
MIPS_FUNC_START(i_vector ## x)    \
  j       i_vector;               \
  li      $k0, x;                 \
  .rept 6;                        \
  nop;                            \
  .endr;                          \
MIPS_FUNC_END(i_vector ## x)
#endif


  MIPS_SECTION_CODE(.evectors)

MIPS_FUNC_START(gen_vector)
  j       e_vector
  nop
MIPS_FUNC_END(gen_vector)


  MIPS_SECTION_CODE(.ivectors)

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

MIPS_FUNC_START(i_vector)
#if defined(MIPS_USE_SHADOW_GPR)
  /* Interrupts are still disabled(EXL=1 or ERL=1) during the switch and restored when new task status reg is set */
srs_resched:
  /* Switch to previous SRS */
  di
  mfc0    $k0, epc
  wrpgpr  $k0, $k0
  la      $k0, prev_srs
  mtc0    $k0, epc
  eret

prev_srs:
  isr_save_ctx

  /* EPC was stored in k0 before switching to this SRS */
  sw      $k0, 84 ($sp)

  .extern chSchDoReschedule
#if defined(MIPS_USE_MIPS16_ISA)
  jalx    chSchDoReschedule
#else
  jal     chSchDoReschedule
#endif
  subu    $sp, $sp, MIPS_STACK_FRAME_SIZE

  addi    $sp, $sp, MIPS_STACK_FRAME_SIZE

  b       restore
  nop
#else
  isr_save_ctx


  mfc0    $a1, cause          /* Passed to port_handle_irq */

  mfc0    $t0, epc
  ehb
  sw      $t0, 84 ($sp)

  move    $k1, $sp            /* Save original SP in k1 */

  /* Switch to exception stack. Recall, nested exceptions are not supported here */
  .extern port_handle_irq
  la      $sp, irq_stack_ready
  jal     port_handle_irq
  move    $a0, $k0            /* Passed to port_handle_irq */

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

  b       resume
  nop
#endif
MIPS_FUNC_END(i_vector)

#include "vectors-single.s"

#endif

/** @} */
