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
 * @file    MIPS-PIC32MX/ext_lld.c
 * @brief   MIPS-PIC32MX EXT subsystem low level driver source.
 *
 * @addtogroup EXT
 * @{
 */

#include "ch.h"
#include "hal.h"

#if HAL_USE_EXT || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver constants and error checks.                                        */
/*===========================================================================*/

/**
 * @brief   Configuration bits of cncon register.
 */
enum extConBits {
  EXT_CON_ON      = 15, /* Change Notice Module On */
};

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

typedef volatile struct {
  PicReg   cncon;
  PicReg   cnen;
  PicReg   cnpue;
} ExtPort;

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

/**
 * @brief   EXT IRQ handler.
 *
 * @param[in] extd       Driver associated to the EXT
 */
static void lld_serve_interrupt(void *data) {
  EXTDriver *extd = data;
  int i;

  /* FIXME: call the callbacks after probing the channels? */
  for (i = 0; i < EXT_MAX_CHANNELS; ++i) {
    const EXTChannelConfig *ch = &extd->config->channels[i];
    if (ch->mode != EXT_CH_MODE_DISABLED) {
      const uint8_t v = palReadPad(ch->port, ch->pad);
      if (v != extd->values[i]) {
        uint32_t mode = ch->mode&EXT_CH_MODE_EDGES_MASK;

        if (EXT_CH_MODE_BOTH_EDGES == mode
            || (EXT_CH_MODE_RISING_EDGE == mode && v)
            || (EXT_CH_MODE_FALLING_EDGE == mode && !v))
          ch->cb(extd, i);
        extd->values[i] = palReadPad(ch->port, ch->pad);
      }
    }
  }
}

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level EXT driver initialization.
 *
 * @notapi
 */
void ext_lld_init(void) {
}

/**
 * @brief   Configures and activates the EXT peripheral.
 *
 * @param[in] extd      pointer to the @p EXTDriver object
 *
 * @notapi
 */
void ext_lld_start(EXTDriver *extd) {
  unsigned i;
  ExtPort *port = (ExtPort *)extd->config->base;

  if (extd->state == EXT_STOP) {
#if HAL_USE_EIC
    eicRegisterIrq(extd->config->irq, lld_serve_interrupt, extd);
    eicEnableIrq(extd->config->irq);
#endif
  }

  /* Configuration of automatic channels.*/
  for (i = 0; i < EXT_MAX_CHANNELS; ++i)
    if (extd->config->channels[i].mode & EXT_CH_MODE_AUTOSTART)
      ext_lld_channel_enable(extd, i);
    else
      ext_lld_channel_disable(extd, i);

  port->cncon.set = 1 << EXT_CON_ON;
}

/**
 * @brief   Deactivates the EXT peripheral.
 *
 * @param[in] extd      pointer to the @p EXTDriver object
 *
 * @notapi
 */
void ext_lld_stop(EXTDriver *extd) {
  ExtPort *port = (ExtPort *)extd->config->base;

  if (extd->state == EXT_ACTIVE) {
#if HAL_USE_EIC
    eicDisableIrq(extd->config->irq);
    eicUnregisterIrq(extd->config->irq);
#endif
  }

  port->cncon.clear = 1 << EXT_CON_ON;
}

/**
 * @brief   Enables an EXT channel.
 *
 * @param[in] extd      pointer to the @p EXTDriver object
 * @param[in] channel   channel to be enabled
 *
 * @notapi
 */
void ext_lld_channel_enable(EXTDriver *extd, expchannel_t channel) {
  const EXTChannelConfig *ch = &extd->config->channels[channel];
  ExtPort *port = (ExtPort *)extd->config->base;

  extd->values[channel] = palReadPad(ch->port, ch->pad);
  port->cnen.set = 1 << channel;
}

/**
 * @brief   Disables an EXT channel.
 *
 * @param[in] extd      pointer to the @p EXTDriver object
 * @param[in] channel   channel to be disabled
 *
 * @notapi
 */
void ext_lld_channel_disable(EXTDriver *extd, expchannel_t channel) {
  ExtPort *port = (ExtPort *)extd->config->base;

  port->cnen.clear = 1 << channel;
}

#endif /* HAL_USE_EXT */

/** @} */
