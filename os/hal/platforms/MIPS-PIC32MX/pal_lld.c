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
 * @file    MIPS-PIC32MX/pal_lld.c
 * @brief   MIPS-PIC32MX PAL subsystem low level driver.
 *
 * @addtogroup PAL
 * @{
 */

#include "ch.h"
#include "hal.h"

#if HAL_USE_PAL || defined(__DOXYGEN__)

#include "mcu/pic32mxxx.h"

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

static void _pal_lld_init_port(ioportid_t port) {
  /* Configure the I/Os as inputs w/o pullups */

  /* Nothing to do as this is default configuration after reset */

  (void)port;
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   PIC32 I/O ports configuration.
 *
 * @param[in] config    the PIC32 ports configuration
 *
 * @notapi
 */
void _pal_lld_init(void) {
  AD1PCFG = 0xFFFFFFFF & _AD1PCFG_PCFG_MASK; /* Configure all pins as digital I/Os */

#if defined(IOPORTA)
  _pal_lld_init_port(IOPORTA);
#endif
#if defined(IOPORTB)
  _pal_lld_init_port(IOPORTB);
#endif
#if defined(IOPORTC)
  _pal_lld_init_port(IOPORTC);
#endif
#if defined(IOPORTD)
  _pal_lld_init_port(IOPORTD);
#endif
#if defined(IOPORTE)
  _pal_lld_init_port(IOPORTE);
#endif
#if defined(IOPORTF)
  _pal_lld_init_port(IOPORTF);
#endif
#if defined(IOPORTG)
  _pal_lld_init_port(IOPORTG);
#endif
}

/**
 * @brief   Pads mode setup.
 * @details This function programs a pads group belonging to the same port
 *          with the specified mode.
 *
 * @param[in] port      the port identifier
 * @param[in] mask      the group mask
 * @param[in] mode      the mode
 *
 * @notapi
 */
void _pal_lld_setgroupmode(ioportid_t port, ioportmask_t mask, iomode_t mode) {
  switch (mode) {
    default:
      chDbgPanic("Unsupported PAD mode\n");
    case PAL_MODE_OUTPUT:
      port->tris.clear = mask;
      break;
    case PAL_MODE_INPUT:
      port->tris.set = mask;
      break;
    case PAL_MODE_OUTPUT_OPENDRAIN:
      port->odc.set = mask;
      break;
  }
}

#endif /* HAL_USE_PAL */

/** @} */
