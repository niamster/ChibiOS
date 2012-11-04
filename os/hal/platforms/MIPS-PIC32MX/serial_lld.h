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
 * @file    MIPS-PIC32MX/serial_lld.h
 * @brief   MIPS-PIC32MX low level serial driver header.
 *
 * @addtogroup SERIAL
 * @{
 */

#ifndef _SERIAL_LLD_H_
#define _SERIAL_LLD_H_

#if HAL_USE_SERIAL || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @brief   UART driver enable switch.
 * @details If set to @p TRUE the support for UART is included.
 * @note    The default is @p TRUE.
 */
#if !defined(USE_MIPS_PIC32MX_UART) || defined(__DOXYGEN__)
#define USE_MIPS_PIC32MX_UART             TRUE
#endif

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Serial Driver configuration structure.
 * @details An instance of this structure must be passed to @p sdStart()
 *          in order to configure and start a serial driver operations.
 */
typedef struct {
  /**
   * @brief   Baud rate.
   */
  uint32_t sc_baud;
} SerialConfig;

/**
 * @brief   @p SerialDriver specific data.
 */
#define _serial_driver_data                           \
  _base_asynchronous_channel_data                     \
  /* Driver state.*/                                  \
  sdstate_t                 state;                    \
  /* Input queue.*/                                   \
  InputQueue                iqueue;                   \
  /* Output queue.*/                                  \
  OutputQueue               oqueue;                   \
  /* Input circular buffer.*/                         \
  uint8_t                   ib[SERIAL_BUFFERS_SIZE];  \
  /* Output circular buffer.*/                        \
  uint8_t                   ob[SERIAL_BUFFERS_SIZE];  \
  /* End of the mandatory fields.*/                   \
  /* MCU UART port.*/                                 \
  uint8_t                   port;                     \
  /* MCU UART RX IRQ.*/                               \
  uint8_t                   rxIrq;                    \

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if USE_MIPS_PIC32MX_UART && !defined(__DOXYGEN__)
extern SerialDriver SD1;
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void sd_lld_init(void);
  void sd_lld_start(SerialDriver *sdp, const SerialConfig *config);
  void sd_lld_stop(SerialDriver *sdp);

  void sd_lld_putc(unsigned char c);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_SERIAL */

#endif /* _SERIAL_LLD_H_ */

/** @} */
