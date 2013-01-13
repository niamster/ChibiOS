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
 * @file    MIPS/MIPS32r2/mipsasm.h
 * @brief   MIPS32r2 Specific ASM Macros.
 *
 * @defgroup MIPS_MIPS32R2 MIPS32r2 Specific ASM Macros
 * @ingroup MIPS_SPECIFIC
 * @details This file contains the MIPS specific ASM for the
 *          MIPS32r2 platform.
 * @{
 */

#ifndef _MIPSASM_H_
#define _MIPSASM_H_

/**
 * @brief   Simple Single Instruction MIPS Inline ASM Wrapper.
 */
#define MIPS_SIMPLE_ASM(inst) asm volatile (".set push; .set noreorder; .set noat; " #inst "; .set pop" : :)

/**
 * @brief   Disable MIPS interrupts and return status.
 */
#define MIPS_DISABLE_IRQ()                                              \
  ({                                                                    \
    uint32_t __flags;                                                   \
    asm volatile (".set push; .set noreorder; .set noat; di %0; .set pop" : "=r"(__flags) :); \
    __flags;                                                            \
  })

/**
 * @brief   Enable MIPS interrupts depending on status.
 */
#define MIPS_RESTORE_IRQ(flags) do {                                    \
    if ((flags) & 0x1)                                                  \
      asm volatile (".set push; .set noreorder; .set noat; ei; .set pop" : :); \
  } while (0)

/**
 * @brief   MIPS ASM Function Prologue.
 */
#define MIPS_FUNC_START(name)                   \
  .type name,@function;                         \
  .set push;                                    \
  .set noreorder;                               \
  .globl name;                                  \
name:

/**
 * @brief   MIPS ASM Function Epilogue.
 */
#define MIPS_FUNC_END(name)                     \
  name##_end:                                   \
  .set pop

/**
 * @brief   MIPS ASM Section Header.
 */
#ifdef __XC32
#define MIPS_SECTION_CODE(name) .section name, code
#else
#define MIPS_SECTION_CODE(name) .section name
#endif

#endif /* _MIPSASM_H_ */

/** @} */
