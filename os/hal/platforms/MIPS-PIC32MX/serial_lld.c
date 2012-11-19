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

#if HAL_USE_SERIAL || defined(__DOXYGEN__)

#include "mcu/pic32mxxx.h"

/*===========================================================================*/
/* Driver constants and error checks.                                       */
/*===========================================================================*/

#define UART_INTERRUPT_TX_EMPTY          (2 << 14)
#define UART_INTERRUPT_TX_COMPLETED      (1 << 14)
#define UART_INTERRUPT_TX_NOT_FULL       (0 << 14)
#define UART_INTERRUPT_TX_MASK           (3 << 14)

#define UART_INTERRUPT_RX_FULL           (3 <<  6)
#define UART_INTERRUPT_RX_3_QUARTER_FULL (2 <<  6)
#define UART_INTERRUPT_RX_NOT_EMPTY      (0 <<  6)
#define UART_INTERRUPT_RX_MASK           (3 <<  6)

#define UART_CONTROL_9_BITS              (3 <<  1)
#define UART_CONTROL_8_BITS_PARITY_ODD   (2 <<  1)
#define UART_CONTROL_8_BITS_PARITY_EVEN  (1 <<  1)
#define UART_CONTROL_8_BITS_PARITY_NONE  (0 <<  1)
#define UART_CONTROL_STOP_BITS_2         (1 <<  0)
#define UART_CONTROL_STOP_BITS_1         (0 <<  0)
#define UART_CONTROL_MASK                (7 <<  0)

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

typedef struct {
	volatile uint32_t reg;
	volatile uint32_t clear;
	volatile uint32_t set;
	volatile uint32_t invert;
} UartReg;

typedef struct {
	UartReg mode;
	UartReg status;
	UartReg tx;
	UartReg rx;
	UartReg brg;
} UartPort;

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

#if USE_MIPS_PIC32MX_UART1 || defined(__DOXYGEN__)
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

/**
 * @brief   UART initialization.
 *
 * @param[in] sdp       communication channel associated to the UART
 * @param[in] config    the architecture-dependent serial driver configuration
 */
static void uartInit(SerialDriver *sdp, const SerialConfig *config) {
  UartPort *port = (UartPort *)sdp->base;
  uint32_t fpb = MIPS_CPU_FREQ >> OSCCONbits.PBDIV;
  uint32_t brg0, brg1;
  int32_t brgErr0, brgErr1;

  port->status.clear = UART_INTERRUPT_TX_MASK | UART_INTERRUPT_RX_MASK;
  port->status.set = /* UART_INTERRUPT_TX_NOT_FULL |  */UART_INTERRUPT_RX_NOT_EMPTY;

  port->mode.clear = UART_CONTROL_MASK;
  port->mode.set = UART_CONTROL_8_BITS_PARITY_NONE | UART_CONTROL_STOP_BITS_1;

  /* Try to set brg with smallest possible error */
  brg0 = ((fpb / config->sc_baud) >> 4) - 1;
  brgErr0 = (fpb >> 4) / (brg0 + 1) - config->sc_baud;
  if (brgErr0 < 0) brgErr0 = -brgErr0;

  brg1 = ((fpb / config->sc_baud) >> 2) - 1;
  brgErr1 = (fpb >> 2) / (brg0 + 1) - config->sc_baud;
  if (brgErr1 < 0) brgErr1 = -brgErr1;

  if (brgErr0 < brgErr1) {
    port->mode.reg &= ~_U1MODE_BRGH_MASK;
    port->brg.reg = brg0 & 0xFFFF;
  } else {
    port->mode.reg |= _U1MODE_BRGH_MASK;
    port->brg.reg = brg1 & 0xFFFF;
  }

  port->status.set = _U1STA_URXEN_MASK | _U1STA_UTXEN_MASK;
  port->mode.set = _U1MODE_ON_MASK;
}

/**
 * @brief Transmit buffer is full
 */
static inline bool_t
uartTxBufferFull(UartPort *port)
{
    return port->status.reg & _U1STA_UTXBF_MASK;
}

/**
 * @brief Transmit shift register is empty and transmit buffer is empty(the last transmission has completed)
 */
static inline bool_t
uartTxComplete(UartPort *port)
{
    return !!(port->status.reg & _U1STA_TRMT_MASK);
}

static inline void
uartTxByte(UartPort *port, uint8_t b)
{
    port->tx.reg = b;
}

static inline bool_t
uartRxReady(UartPort *port)
{
    return !!(port->status.reg & _U1STA_URXDA_MASK);
}

static inline uint8_t
uartRxByte(UartPort *port)
{
    return port->rx.reg;
}

void sd_lld_putc(uint8_t c) {
  UartPort *port = (UartPort *)SD1.base;

  while (uartTxBufferFull(port));

  uartTxByte(port, c);

  /* while (!uartTxComplete(port)); */
}

static void oNotify(GenericQueue *qp) {
  msg_t b;

  (void)qp;

  b = sdRequestDataI(&SD1);
  if (b != Q_EMPTY)
    sd_lld_putc(b);
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/**
 * @brief   Common IRQ handler.
 *
 * @param[in] sdp       communication channel associated to the USART
 */
void sd_lld_serve_interrupt(void *data) {
  SerialDriver *sdp = data;
  UartPort *port = (UartPort *)sdp->base;

  chSysLockFromIsr();

  if (uartRxReady(port))
    sdIncomingDataI(sdp, uartRxByte(port));

  chSysUnlockFromIsr();
}

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level serial driver initialization.
 *
 * @notapi
 */
void sd_lld_init(void) {
#if USE_MIPS_PIC32MX_UART1
  SD1.base = (void *)/* MIPS_UNCACHED */(_UART1_BASE_ADDRESS);
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

  uartInit(sdp, config);
#if HAL_USE_EIC
  eicEnableIrq(sdp->rxIrq);
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
