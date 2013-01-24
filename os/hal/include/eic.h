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
 * @file    os/hal/include/eic.h
 * @brief   External Interrupt Controller common header.
 *
 * @addtogroup EIC
 * @{
 */

#ifndef _EIC_H_
#define _EIC_H_

#if HAL_USE_EIC || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Type of a callback called to handle IRQ.
 */
typedef void (*eicIrqHandler)(void *);

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#include "eic_lld.h"

#ifdef __cplusplus
extern "C" {
#endif
  void eicInit(void);
  void eicRegisterIrq(int irq, eicIrqHandler handler, void *data);
  void eicUnregisterIrq(int irq);
  void eicEnableIrq(int irq);
  void eicDisableIrq(int irq);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_EIC */

#endif  /* _EIC_H_ */

/** @} */
