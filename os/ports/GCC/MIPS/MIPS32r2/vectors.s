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
  .rept MIPS_EXC_STACK_SIZE
  .byte 0
  .endr
  .balign 4
  .global exception_stack_bottom
exception_stack_bottom:

#endif

/** @} */
