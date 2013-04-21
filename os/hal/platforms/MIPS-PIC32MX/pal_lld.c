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
 * @file    MIPS-PIC32MX/pal_lld.c
 * @brief   MIPS-PIC32MX PAL subsystem low level driver.
 *
 * @addtogroup PAL
 * @{
 */

#include "ch.h"
#include "hal.h"

#if HAL_USE_PAL || defined(__DOXYGEN__)

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

static void
pal_enable_analog_pin(ioportid_t port, ioportmask_t mask, bool_t enable) {
#if defined(_AD1PCFG_PCFG_POSITION)
#if defined(PIC32MX3XX) || defined(PIC32MX4XX) \
 || defined(PIC32MX5XX) || defined(PIC32MX6XX) || defined(PIC32MX7XX)
  if (port != IOPORTB)
    return;
#else
#error PAL LLD can not detect analog port
#endif

  if (enable)
    AD1PCFGCLR = mask;
  else
    AD1PCFGSET = mask;
#else
  if (enable)
    port->ansel.set = mask;
  else
    port->ansel.clear = mask;
#endif
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

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
      break;
    case PAL_MODE_OUTPUT_OPENDRAIN:
      port->odc.set = mask;
    case PAL_MODE_OUTPUT:
      pal_enable_analog_pin(port, mask, false);
      port->tris.clear = mask;
      break;
    case PAL_MODE_INPUT:
      pal_enable_analog_pin(port, mask, false);
      port->tris.set = mask;
      break;
    case PAL_MODE_INPUT_ANALOG:
      pal_enable_analog_pin(port, mask, true);
      port->tris.set = mask;
      break;
  }
}

#endif /* HAL_USE_PAL */

/** @} */
