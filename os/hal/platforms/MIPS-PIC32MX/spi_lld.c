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
 * @file    MIPS-PIC32MX/spi_lld.c
 * @brief   MIPS-PIC32MX low level SPI driver code.
 *
 * @addtogroup SPI
 * @{
 */

#include "ch.h"
#include "hal.h"

#if HAL_USE_SPI || defined(__DOXYGEN__)

#include "mcu/pic32mxxx.h"

/*===========================================================================*/
/* Driver constants and error checks.                                        */
/*===========================================================================*/

enum spiConBits {
  SPI_CON_ON      = 15,
  SPI_CON_WIDTH   = 10,
  SPI_CON_CPHA    = 8,
  SPI_CON_CPOL    = 6,
  SPI_CON_MASTER  = 5,
};

enum spiStatusBits {
  SPI_STATUS_RBF  = 0,
};

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

typedef struct {
  PicReg   con;
  PicReg   status;
  volatile uint32_t buf;
  uint32_t pad0[4];
  PicReg   brg;
} SpiPort;

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

static void __spi_config(SPIDriver *spid) {
  volatile SpiPort *port = (SpiPort *)spid->base;
  const SPIConfig *cfg = spid->config;
  uint32_t fpb = MIPS_CPU_FREQ >> OSCCONbits.PBDIV;
  uint32_t con = 0;

  chDbgAssert(cfg->master,
      "SPI master mode is only supported", "");
  chDbgAssert(cfg->width == SPI_DATA_MODE_8_BIT,
      "SPI 8-bit mode is only supported", ""); // for the simplicity of buffer handling

  (void)port->buf;

  port->con.reg = 0;
  port->status.reg = 0;

  con |= cfg->width << SPI_CON_WIDTH;
  switch (cfg->clk_mode) {
    case SPI_CLK_MODE0:
      break;
    case SPI_CLK_MODE1:
      con |= 1 << SPI_CON_CPHA;
      break;
    case SPI_CLK_MODE2:
      con |= 1 << SPI_CON_CPOL;
      break;
    case SPI_CLK_MODE3:
      con |= 1 << SPI_CON_CPOL;
      con |= 1 << SPI_CON_CPHA;
      break;
  }
  con |= 1 << SPI_CON_MASTER;

  port->con.reg = con;

  port->brg.reg = ((fpb / cfg->clk) / 2) - 1;
}

static void __spi_start(SPIDriver *spid) {
  volatile SpiPort *port = (SpiPort *)spid->base;

  port->con.set = 1 << SPI_CON_ON;
}

static void __spi_stop(SPIDriver *spid) {
  volatile SpiPort *port = (SpiPort *)spid->base;

  port->con.clear = 1 << SPI_CON_ON;
}

static void __spi_start_transaction(SPIDriver *spid) {
  volatile SpiPort *port = (SpiPort *)spid->base;

  if (spid->txptr)
    port->buf = *spid->txptr;
  else
    port->buf = 0xFFFFFFFF;
}

static void __spi_finish_transaction(SPIDriver *spid) {
  volatile SpiPort *port = (SpiPort *)spid->base;
  uint32_t reg;

  reg = port->buf;
  if (spid->rxptr) {
    *spid->rxptr = reg;
    ++spid->rxptr;
  }

  --spid->cnt;

  if (spid->txptr)
    ++spid->txptr;
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/**
 * @brief   SPI RX IRQ handler.
 *
 * @param[in] data       Driver associated with the SPI channel
 */
static void lld_serve_rx_interrupt(void *data) {
  SPIDriver *spid = data;
  volatile SpiPort *port = (SpiPort *)spid->base;

  chSysLockFromIsr();

  while (port->status.reg & (1 << SPI_STATUS_RBF)) {
    __spi_finish_transaction(spid);

    if (!spid->cnt) {
      _spi_isr_code(spid);
      break;
    } else
      __spi_start_transaction(spid);
  }

  chSysUnlockFromIsr();
}

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level SPI driver initialization.
 *
 * @notapi
 */
void spi_lld_init(void) {
}

/**
 * @brief   Configures and activates the SPI peripheral.
 *
 * @param[in] spid      pointer to the @p SPIDriver object
 *
 * @notapi
 */
void spi_lld_start(SPIDriver *spid) {
  const SPIConfig *cfg = spid->config;

  spid->base = cfg->base;
  __spi_config(spid);

#if HAL_USE_EIC
  if (spid->rx_irq != cfg->rx_irq) { /* Assuming this is done only once */
    eicRegisterIrq(cfg->rx_irq, lld_serve_rx_interrupt, spid);
    spid->rx_irq = cfg->rx_irq;
  }
  eicEnableIrq(cfg->rx_irq);
#endif

  __spi_start(spid);
}

/**
 * @brief   Deactivates the SPI peripheral.
 *
 * @param[in] spid      pointer to the @p SPIDriver object
 *
 * @notapi
 */
void spi_lld_stop(SPIDriver *spid) {
  const SPIConfig *cfg = spid->config;

#if HAL_USE_EIC
  eicDisableIrq(cfg->rx_irq);
#endif

  __spi_stop(spid);
}

/**
 * @brief   Asserts the slave select signal and prepares for transfers.
 *
 * @param[in] spid      pointer to the @p SPIDriver object
 *
 * @notapi
 */
void spi_lld_select(SPIDriver *spid) {
  palClearPad(spid->config->cs.port, spid->config->cs.pad);
}

/**
 * @brief   Deasserts the slave select signal.
 * @details The previously selected peripheral is unselected.
 *
 * @param[in] spid      pointer to the @p SPIDriver object
 *
 * @notapi
 */
void spi_lld_unselect(SPIDriver *spid) {
  palSetPad(spid->config->cs.port, spid->config->cs.pad);
}

/**
 * @brief   Ignores data on the SPI bus.
 * @details This function transmits a series of idle words on the SPI bus and
 *          ignores the received data. This function can be invoked even
 *          when a slave select signal has not been yet asserted.
 *
 * @param[in] spid      pointer to the @p SPIDriver object
 * @param[in] n         number of words to be ignored
 *
 * @notapi
 */
void spi_lld_ignore(SPIDriver *spid, size_t n) {
  spid->rxptr = NULL;
  spid->txptr = NULL;
  spid->cnt = n;

  __spi_start_transaction(spid);
}

/**
 * @brief   Exchanges data on the SPI bus.
 * @details This function performs a simultaneous transmit/receive operation.
 * @note    The buffers are organized as uint8_t arrays.
 *
 * @param[in] spid      pointer to the @p SPIDriver object
 * @param[in] n         number of words to be exchanged
 * @param[in] txbuf     the pointer to the transmit buffer
 * @param[out] rxbuf    the pointer to the receive buffer
 *
 * @notapi
 */
void spi_lld_exchange(SPIDriver *spid, size_t n,
                      const void *txbuf, void *rxbuf) {
  spid->rxptr = rxbuf;
  spid->txptr = txbuf;
  spid->cnt = n;

  __spi_start_transaction(spid);
}

/**
 * @brief   Sends data over the SPI bus.
 * @note    The buffers are organized as uint8_t arrays.
 *
 * @param[in] spid      pointer to the @p SPIDriver object
 * @param[in] n         number of words to send
 * @param[in] txbuf     the pointer to the transmit buffer
 *
 * @notapi
 */
void spi_lld_send(SPIDriver *spid, size_t n, const void *txbuf) {
  spid->rxptr = NULL;
  spid->txptr = txbuf;
  spid->cnt = n;

  __spi_start_transaction(spid);
}

/**
 * @brief   Receives data from the SPI bus.
 * @note    The buffers are organized as uint8_t arrays.
 *
 * @param[in] spid      pointer to the @p SPIDriver object
 * @param[in] n         number of words to receive
 * @param[out] rxbuf    the pointer to the receive buffer
 *
 * @notapi
 */
void spi_lld_receive(SPIDriver *spid, size_t n, void *rxbuf) {
  spid->rxptr = rxbuf;
  spid->txptr = NULL;
  spid->cnt = n;

  __spi_start_transaction(spid);
}

/**
 * @brief   Exchanges one frame using a polled wait.
 * @details This synchronous function exchanges one frame using a polled
 *          synchronization method. This function is useful when exchanging
 *          small amount of data on high speed channels, usually in this
 *          situation is much more efficient just wait for completion using
 *          polling than suspending the thread waiting for an interrupt.
 *
 * @param[in] spid      pointer to the @p SPIDriver object
 * @param[in] frame     the data frame to send over the SPI bus
 * @return              The received data frame from the SPI bus.
 */
uint32_t spi_lld_polled_exchange(SPIDriver *spid, uint32_t frame) {
  const SPIConfig *cfg = spid->config;
  volatile SpiPort *port = (SpiPort *)spid->base;

#if HAL_USE_EIC
  eicDisableIrq(cfg->rx_irq);
#endif

  port->buf = frame;

  while (!(port->status.reg & (1 << SPI_STATUS_RBF)));

  frame = port->buf;

#if HAL_USE_EIC
  eicEnableIrq(cfg->rx_irq);
#endif

  return frame;
}

#endif /* HAL_USE_SPI */

/** @} */
