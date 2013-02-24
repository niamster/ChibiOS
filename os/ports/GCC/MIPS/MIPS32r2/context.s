/*
    ChibiOS/RT - Copyright (C) 2006,2007,2008,2009,2010,
                 2011,2012,2013 Giovanni Di Sirio.

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
