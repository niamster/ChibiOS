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
 * @file    MIPS/MIPS32r2/mipsparams.h
 * @brief   MIPS32r2 Specific Parameters.
 *
 * @defgroup MIPS_MIPS32R2 MIPS32r2 Specific Parameters
 * @ingroup MIPS_SPECIFIC
 * @details This file contains the MIPS specific parameters for the
 *          MIPS32r2 platform.
 * @{
 */

#ifndef _MIPSPARAMS_H_
#define _MIPSPARAMS_H_

/**
 * @brief   MIPS core model.
 */
#define MIPS_CORE                MIPS_CORE_MIPS32R2

/**
 * @brief   MIPS stack frame size when called from ASM (four 64-bit args)
 */
#define MIPS_STACK_FRAME_SIZE   32

/**
 * @brief   Size of MIPS exception table(6 wired to HW interrupt sources, 2 to SW interrupts)
 * @note    Set this value to 64 if EIC mode is supported
 */
#define MIPS_EXC_TABLE_SIZE     8

/**
 * @brief   MIPS exception IDs(wired to HW interrupt sources)
 */
#define MIPS_HW_IRQ0            0
#define MIPS_HW_IRQ1            1
#define MIPS_HW_IRQ2            2
#define MIPS_HW_IRQ3            3
#define MIPS_HW_IRQ4            4
#define MIPS_HW_IRQ5            5

#endif /* _MIPSPARAMS_H_ */

/** @} */
