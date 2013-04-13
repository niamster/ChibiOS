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

#if !defined(__DOXYGEN__)

#if defined(MIPS_USE_SHADOW_GPR) || defined(MIPS_USE_VECTORED_IRQ)
#include "vectors-iv.s"
#else
#include "vectors-single.s"
#endif


  .bss

  /* Separate stack for exception routines for simplicity */
  .balign 4
  .global exception_stack_top
exception_stack_top:
  .rept MIPS_EXC_STACK_SIZE - MIPS_STACK_FRAME_SIZE
  .byte 0
  .endr
  .balign 4
  .global exception_stack_ready
exception_stack_ready:
  .rept MIPS_STACK_FRAME_SIZE
  .byte 0
  .endr
  .balign 4
  .global exception_stack_bottom
exception_stack_bottom:

#if defined(MIPS_USE_SHADOW_GPR) || defined(MIPS_USE_VECTORED_IRQ)
  /* Separate stack for IRQ routines for simplicity */
  .balign 4
  .global irq_stack_top
irq_stack_top:
  .rept MIPS_IRQ_STACK_SIZE - MIPS_STACK_FRAME_SIZE
  .byte 0
  .endr
  .balign 4
  .global irq_stack_ready
irq_stack_ready:
  .rept MIPS_STACK_FRAME_SIZE
  .byte 0
  .endr
  .balign 4
  .global irq_stack_bottom
irq_stack_bottom:
#endif

#endif

/** @} */
