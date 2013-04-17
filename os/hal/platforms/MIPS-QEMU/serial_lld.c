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
 * @file    MIPS-QEMU/serial_lld.c
 * @brief   MIPS_QEMU low level serial driver code.
 *
 * @addtogroup SERIAL
 * @{
 */

#include "ch.h"
#include "hal.h"

#if HAL_USE_SERIAL || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

#if USE_MIPS_QEMU_UART || defined(__DOXYGEN__)
/** @brief UART serial driver identifier.*/
SerialDriver SD1;
#define HAL_MIPS_QEMU_UART_CLK         115200
#endif

#if !defined(HAL_MIPS_QEMU_UART_BASE)
#error HAL_MIPS_QEMU_UART_BASE not defined
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

#if USE_MIPS_QEMU_UART
/**
 * @brief   UART initialization.
 *
 * @param[in] sdp       communication channel associated to the UART
 * @param[in] config    the architecture-dependent serial driver configuration
 */
static void uartInit(SerialDriver *sdp, const SerialConfig *config) {
  volatile SerialRegs *port = sdp->base;

  const uint16_t x_div =  16;
  const uint32_t clk = HAL_MIPS_QEMU_UART_CLK;
  const uint32_t baud = config->sc_baud;
  const uint16_t div = (clk + (baud * (x_div / 2))) / (x_div * baud);

  port->ier = 0;

  port->lcr = UART_LCR_DLAB;
  port->dll = div;
  port->dlh = div >> 8;

  port->lcr = UART_LCR_WLS_8;
  port->mcr = UART_MCR_DTR | UART_MCR_RTS;
  port->fcr = UART_FCR_CLR_RCVR | UART_FCR_CLR_XMIT;
  port->fcr = UART_FCR_FIFO_EN | UART_FCR_CLR_RCVR | UART_FCR_CLR_XMIT;

  port->ier = UART_IER_RDI /* | UART_IER_THRI */;
}

void sd_lld_putc(uint8_t c) {
  volatile SerialRegs *port = SD1.base;

  port->thr = c;
}

static void oNotify(GenericQueue *qp) {
  volatile SerialRegs *port = SD1.base;

  (void)qp;

  if (port->lsr & UART_LSR_THRE) {
    msg_t b = sdRequestDataI(&SD1);
    if (b != Q_EMPTY)
      port->thr = b;
  }
}
#endif

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if USE_MIPS_QEMU_UART || defined(__DOXYGEN__)
/**
 * @brief   Common IRQ handler.
 *
 * @param[in] sdp       communication channel associated to the USART
 */
void sd_lld_serve_interrupt(uint32_t irq, void *data) {
  SerialDriver *sdp = data;
  volatile SerialRegs *port = sdp->base;
  uint8_t lsr = port->lsr;

  (void)irq;

  chSysLockFromIsr();

  /* if (lsr & UART_LSR_THRE) { */
  /*   msg_t b = sdRequestDataI(sdp); */
  /*   if (b != Q_EMPTY) */
  /*     port->thr = b; */
  /* } */

  if (lsr & UART_LSR_DR) {
    uint8_t c = port->thr;
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
#if USE_MIPS_QEMU_UART
  SD1.base = (void *)MIPS_UNCACHED(HAL_MIPS_QEMU_UART_BASE);
  sdObjectInit(&SD1, NULL, oNotify);
#if HAL_USE_EIC
  eicRegisterIrq(EIC_IRQ_UART, sd_lld_serve_interrupt, &SD1);
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

#if USE_MIPS_QEMU_UART
  uartInit(sdp, config);
#if HAL_USE_EIC
  eicEnableIrq(EIC_IRQ_UART);
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
