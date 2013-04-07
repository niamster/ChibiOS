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
 * @file    MIPS-PIC32MX/hal_lld.c
 * @brief   MIPS-PIC32MX HAL subsystem low level driver source.
 *
 * @addtogroup HAL
 * @{
 */

#include "ch.h"
#include "hal.h"

#include "mcu/pic32mxxx.h"

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#define FLASH_MAX_CLK          30000000

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
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level HAL driver initialization.
 *
 * @notapi
 */
void hal_lld_init(void) {
#if defined(_PCACHE)
  /* Do not enable D-cache because it's inefficient for PIC32 */

  CHECON = ((MIPS_CPU_FREQ/FLASH_MAX_CLK))      /* PFM Access Time Defined in terms of SYSLK Wait states bits */
    | (3 << _CHECON_PREFEN_POSITION)            /* Enable predictive prefetch cache for all regions */
    | (1 << _CHECON_CHECOH_POSITION);           /* Invalidate all data and instruction lines */

  BMXCONSET = _BMXCON_BMXCHEDMA_MASK; /* Enable program Flash memory (data) cacheability for DMA accesses */

  // cached KSEG0
  c0_set_config0((c0_get_config0() & ~7) | 3);
#endif

  BMXCONCLR = _BMXCON_BMXWSDRM_MASK; /* Data RAM accesses from CPU have zero wait states for address setup */
}

void hal_system_unlock(void) {
  SYSKEY = 0;
  SYSKEY = 0xAA996655;
  SYSKEY = 0x556699AA;
}

void hal_system_lock(void) {
  SYSKEY = 0x33333333;
}

/**
 * @brief   Device software reset.
 *
 * @notapi
 */
void hal_lld_reset(void) {
  volatile uint32_t dummy;

  port_disable();

  /* Suspend DMA */
  DMACONSET = _DMACON_SUSPEND_MASK;
  while ((DMACON >> _DMACON_DMABUSY_POSITION)&1);

  hal_system_unlock();

  /* Toggle SW reset */
  RSWRSTSET = 1 << _RSWRST_SWRST_POSITION;
  dummy = RSWRST;
  (void)dummy;

  for (;;);
}

/** @} */
