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
