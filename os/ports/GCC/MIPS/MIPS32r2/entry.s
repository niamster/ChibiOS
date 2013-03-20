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


  MIPS_SECTION_CODE(.entry)

MIPS_FUNC_START(_start)
  /* Start in uncached mode */
  li      $k0, 0x2
  mtc0    $k0, config0

  /* Clear CPU status */
  mtc0    $zero, cause
  mtc0    $zero, status
  mtc0    $zero, intctl

  /* Set exception base */
  la      $t0, e_vector
  lui     $t1, 0xFFFF
  and     $t0, $t0, $t1
  mtc0    $t0, ebase

  /* Setup initial context, run C setup code and finally call main() */

  /* <clear:bss> */
#ifdef __XC32
  /* .bss is initialized via dinit table */
#else
  /* Strongly assume that .bss START and END are word aligned */
  la      $t0, __bss_start__
  la      $t1, __bss_end__
  addiu   $t1, $t1, -4

1:
  sw      $zero, 0($t0)
  bne     $t0, $t1, 1b
  addiu   $t0, $t0, 4
#endif
  /* </clear:bss> */

  /* <copy:data> */
  /* Copy .data from ROM to RAM if needed */
#ifdef __XC32
  /* xc32-ish way to copy data */

  /* Table format:
     struct {
       uint32_t dst;
       uint32_t len;
       uint32_t fmt;
       uint8_t  data[];
     };
    If fmt is 0 then clear the memory at dst, otherwise copy from data[] to dst
    If fmt is clear then size of data[] is zero
    Next entry is at 4-bytes aligned address after data.
    If dst is 0 then it's the end of the table, e.g. table is NULL-terminated
  */

  la      $t0, _dinit_addr

_dinit_next:
  lw      $t1, 0($t0)           /* dst */
  beqz    $t1, _early_init
  lw      $t2, 4($t0)           /* len */
  lw      $t3, 8($t0)           /* fmt */
  beq     $t3, $zero, _dinit_clear
  addu    $t0, $t0, 0xC

_dinit_copy:
  lbu     $t3, 0($t0)
  subu    $t2, $t2, 1
  addu    $t0, $t0, 1
  sb      $t3, 0($t1)
  bnez    $t2, _dinit_copy
  addu    $t1, $t1, 1

  b       _dinit_end
  nop

_dinit_clear:
  sb      $zero, 0($t1)
  subu    $t2, $t2, 1
  bne     $t2, $zero, _dinit_clear
  addu    $t1, $t1, 1

_dinit_end:
  li      $t3, 0xFFFFFFFC
  addu    $t0, $t0, 3
  b       _dinit_next
  and     $t0, $t0, $t3
#else
  /* Strongly assume that .data START and END are word aligned */
  la      $t0, __rom_data_start__
  la      $t1, __ram_data_start__
  beq     $t0, $t1, _early_init
  nop

  la      $t2, __ram_data_end__
  addiu   $t2, $t2, -4

2:
  lw      $t3, 0($t0)
  sw      $t3, 0($t1)
  addiu   $t0, $t0, 4
  bne     $t1, $t2, 2b
  addiu   $t1, $t1, 4
#endif
  /* </copy:data> */

_early_init:
  la      $sp, main_stack_bottom
  subu    $sp, $sp, MIPS_STACK_FRAME_SIZE

  .extern port_early_init
  la      $t0, port_early_init
  jalr    $t0
  nop

  /* <exec:ctors> */
  la      $s0, __ctors_start__
  la      $s1, __ctors_end__
  beq     $s0, $s1, start_app
  nop

3:
  lw      $s2, 0($s0)
  jalr    $s2
  addiu   $s0, $s0, 4
  bne     $s0, $s1, 3b
  nop
  /* </exec:ctors> */

start_app:
  .extern main
  la      $t0, main
  jalr    $t0
  nop
_life_after_main:
  b _start
  nop
MIPS_FUNC_END(_start)

    
  .bss

  /* Separate stack for main() */
  .balign 4
  .global main_stack_top
main_stack_top:
  .global __main_thread_stack_base__
__main_thread_stack_base__:
  .rept MIPS_MAIN_STACK_SIZE
  .byte 0
  .endr
  .balign 4
  .global main_stack_bottom
main_stack_bottom:

#endif

/** @} */
