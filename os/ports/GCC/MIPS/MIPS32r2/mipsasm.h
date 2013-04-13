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
