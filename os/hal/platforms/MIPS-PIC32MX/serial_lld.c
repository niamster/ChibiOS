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
 * @file    MIPS-PIC32MX/serial_lld.c
 * @brief   MIPS-PIC32MX low level serial driver code.
 *
 * @addtogroup SERIAL
 * @{
 */

#include "ch.h"
#include "hal.h"

#undef TRUE
#undef FALSE

#undef _UART_H_
#include "plib.h"

#define TRUE 1
#define FALSE 0

#if HAL_USE_SERIAL || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

#if USE_MIPS_PIC32MX_UART || defined(__DOXYGEN__)
/** @brief UART serial driver identifier.*/
SerialDriver SD1;
#endif

/*===========================================================================*/
/* Driver local variables.                                                   */
/*===========================================================================*/

/* @brief Driver default configuration. */
static const SerialConfig sc = {
  SERIAL_DEFAULT_BITRATE,
};

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

#if USE_MIPS_PIC32MX_UART
/**
 * @brief   UART initialization.
 *
 * @param[in] sdp       communication channel associated to the UART
 * @param[in] config    the architecture-dependent serial driver configuration
 */
static void uartInit(SerialDriver *sdp, const SerialConfig *config) {
  uint8_t port = sdp->port;

  UARTConfigure(port, UART_ENABLE_PINS_TX_RX_ONLY);
  UARTSetFifoMode(port, /* UART_INTERRUPT_ON_TX_NOT_FULL |  */UART_INTERRUPT_ON_RX_NOT_EMPTY);
  UARTSetLineControl(port, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
  UARTSetDataRate(port, MIPS_CPU_FREQ / (1 << OSCCONbits.PBDIV), config->sc_baud);
  UARTEnable(port, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));
}

void sd_lld_putc(uint8_t c) {
  uint8_t port = SD1.port;

  while (!UARTTransmitterIsReady(port));

  UARTSendDataByte(port, c);
}

static void oNotify(GenericQueue *qp) {
  uint8_t port = SD1.port;

  (void)qp;

  if (UARTTransmitterIsReady(port)) {
    msg_t b = sdRequestDataI(&SD1);
    if (b != Q_EMPTY) {
      UARTSendDataByte(port, b);
      while (!UARTTransmissionHasCompleted(port));
    }
  }
}
#endif

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if USE_MIPS_PIC32MX_UART || defined(__DOXYGEN__)
/**
 * @brief   Common IRQ handler.
 *
 * @param[in] sdp       communication channel associated to the USART
 */
void sd_lld_serve_interrupt(void *data) {
  SerialDriver *sdp = data;
  uint8_t port = sdp->port;
  uint8_t c;

  chSysLockFromIsr();

  if (UARTReceivedDataIsAvailable(port)) {
    c = UARTGetDataByte(port);
    sdIncomingDataI(sdp, c);
  }

  chSysUnlockFromIsr();
}
#endif

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level serial driver initialization.
 *
 * @notapi
 */
void sd_lld_init(void) {
#if USE_MIPS_PIC32MX_UART
  SD1.port = UART1;
  SD1.rxIrq = EIC_IRQ_UART1_RX;
  sdObjectInit(&SD1, NULL, oNotify);
#if HAL_USE_EIC
  eicRegisterIrq(SD1.rxIrq, sd_lld_serve_interrupt, &SD1);
#endif
#endif
}

/**
 * @brief   Low level serial driver configuration and (re)start.
 *
 * @param[in] sdp       pointer to a @p SerialDriver object
 * @param[in] config    the architecture-dependent serial driver configuration.
 *                      If this parameter is set to @p NULL then a default
 *                      configuration is used.
 *
 * @notapi
 */
void sd_lld_start(SerialDriver *sdp, const SerialConfig *config) {
  if (config == NULL)
    config = &sc;

#if USE_MIPS_PIC32MX_UART
  uartInit(sdp, config);
#if HAL_USE_EIC
  eicEnableIrq(sdp->rxIrq);
#endif
#endif
}

/**
 * @brief   Low level serial driver stop.
 * @details De-initializes the UART, stops the associated clock, resets the
 *          interrupt vector.
 *
 * @param[in] sdp       pointer to a @p SerialDriver object
 *
 * @notapi
 */
void sd_lld_stop(SerialDriver *sdp) {
  (void)sdp;
}

#endif /* HAL_USE_SERIAL */

/** @} */
