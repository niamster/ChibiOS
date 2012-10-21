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
 * @file    pic.c
 * @brief   PIC Driver code.
 *
 * @addtogroup PIC
 * @{
 */

#include "ch.h"
#include "hal.h"

#if HAL_USE_PIC || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   PIC Driver initialization.
 * @note    This function is implicitly invoked by @p halInit(), there is
 *          no need to explicitly initialize the driver.
 *
 * @init
 */
void picInit(void) {

  pic_lld_init();
}

/**
 * @brief   PIC IRQ handler registration.
 *
 * @param[in] irq       irq number.
 * @param[in] handler   ponter to irq handler.
 * @param[in] data      opaque data passed to irq handler.
 */
void picRegisterIrq(int irq, picIrqHandler handler, void *data) {

  pic_lld_register_irq(irq, handler, data);
}

/**
 * @brief   Enable PIC IRQ.
 *
 * @param[in] irq       irq number.
 */
void picEnableIrq(int irq) {

  pic_lld_enable_irq(irq);
}

/**
 * @brief   Disable PIC IRQ.
 *
 * @param[in] irq       irq number.
 */
void picDisableIrq(int irq) {

  pic_lld_disable_irq(irq);
}

#endif /* HAL_USE_PIC */

/** @} */
