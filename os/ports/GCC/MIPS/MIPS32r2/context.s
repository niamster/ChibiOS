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
 * @file    MIPS/MIPS32r2/chcoreasm.s
 * @brief   MIPS32r2 architecture port low level code.
 *
 * @addtogroup MIPS_CORE
 * @{
 */

#include "chconf.h"
#include "mipsasm.h"
#include "mipsarch.h"

#if !defined(__DOXYGEN__)

MIPS_FUNC_START(_port_switch_mips)
  subu    $sp, $sp, 44        /* sizeof(struct intctx) */

  .set noat
  sw      $s0, 0  ($sp)
  sw      $s1, 4  ($sp)
  sw      $s2, 8  ($sp)
  sw      $s3, 12 ($sp)
  sw      $s4, 16 ($sp)
  sw      $s5, 20 ($sp)
  sw      $s6, 24 ($sp)
  sw      $s7, 28 ($sp)
  sw      $fp, 32 ($sp)
  sw      $ra, 36 ($sp)
  .set at

  mfc0    $t0, status
  ehb
  sw      $t0, 40 ($sp)

  sw      $sp, 12 ($a1)

  /* ---- */

  lw      $sp, 12 ($a0)

  .set noat
  lw      $s0, 0  ($sp)
  lw      $s1, 4  ($sp)
  lw      $s2, 8  ($sp)
  lw      $s3, 12 ($sp)
  lw      $s4, 16 ($sp)
  lw      $s5, 20 ($sp)
  lw      $s6, 24 ($sp)
  lw      $s7, 28 ($sp)
  lw      $fp, 32 ($sp)
  lw      $ra, 36 ($sp)
  .set at

  lw      $t0, 40 ($sp)

  addi    $sp, $sp, 44        /* sizeof(struct intctx) */

  jr      $ra
  mtc0    $t0, status
MIPS_FUNC_END(_port_switch_mips)

MIPS_FUNC_START(_port_thread_start)
  jalr    $s1
  move    $a0, $s0
#if defined(MIPS_USE_MIPS16_ISA)
  jalx    chThdExit
#else
  jal     chThdExit
#endif
  nop
MIPS_FUNC_END(_port_thread_start)

#endif /* !defined(__DOXYGEN__) */

/** @} */
