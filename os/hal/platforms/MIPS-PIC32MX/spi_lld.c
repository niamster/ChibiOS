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
 * @file    MIPS-PIC32MX/spi_lld.c
 * @brief   MIPS-PIC32MX low level SPI driver code.
 *
 * @addtogroup SPI
 * @{
 */

#include "ch.h"
#include "hal.h"

#if HAL_USE_SPI || defined(__DOXYGEN__)

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
/* Driver macros.                                                            */
/*===========================================================================*/

#define container_of(ptr, type, member) ({                  \
      const typeof( ((type *)0)->member ) *__mptr = (ptr);	\
      (type *)( (char *)__mptr - __builtin_offsetof(type, member) );   \
    })

#define min(x, y) ((x)<(y)?(x):(y))

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

typedef volatile struct {
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

static uint8_t dummyDmaTxBuffer[SPI_DUMMY_DMA_BUFFER_SIZE];
static uint8_t dummyDmaRxBuffer[SPI_DUMMY_DMA_BUFFER_SIZE];

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

static void __spi_config(SPIDriver *spid) {
  SpiPort *port = (SpiPort *)spid->base;
  const SPIConfig *cfg = spid->config;
  uint32_t fpb = hal_pb_frequency();
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
  SpiPort *port = (SpiPort *)spid->base;

  port->con.set = 1 << SPI_CON_ON;
}

static void __spi_stop(SPIDriver *spid) {
  SpiPort *port = (SpiPort *)spid->base;

  port->con.clear = 1 << SPI_CON_ON;
}

static void __spi_dma_tx_complete(struct dmaTransaction *tr);
static void __spi_dma_rx_complete(struct dmaTransaction *tr);

static inline void __spi_dma_set_completion(SPIDriver *spid, dmaTransaction *rx, dmaTransaction *tx) {
    /* The trick around EIC's way to handle interrupts:
     * the higher number interrupts are handled first, so
     * we need to finish SPI transaction after the last DMA irq in the chain,
     * that is we chose the lowest IRQ number
     */
    if (spid->rxDmaChan.irq < spid->txDmaChan.irq) {
      rx->cb = __spi_dma_rx_complete;
      tx->cb = NULL;
    } else {
      rx->cb = NULL;
      tx->cb = __spi_dma_tx_complete;
    }
}

static void __spi_dma_complete(SPIDriver *spid, dmaTransaction *rx, dmaTransaction *tx, size_t n) {
  SpiPort *port = (SpiPort *)spid->base;

  dmaUnmap(tx->src);
  dmaUnmap(rx->dst);

  spid->cnt -= n;

  if (!spid->cnt
      || tx->status != DMA_TRANSACTION_SUCCESSFULL
      || rx->status != DMA_TRANSACTION_SUCCESSFULL) {
#if HAL_USE_EIC
    eicAckIrq(spid->config->rx_irq);
    eicEnableIrq(spid->config->rx_irq);
#endif

    _spi_isr_code(spid);
  } else {
    uint32_t cnt = spid->cnt;

    if (!spid->txptr || !spid->rxptr)
      cnt = min(spid->cnt, SPI_DUMMY_DMA_BUFFER_SIZE);

    if (spid->txptr)
      spid->txptr += n;
    if (spid->rxptr)
      spid->rxptr += n;

    tx->src = dmaMap(spid->txptr?(uint8_t *)spid->txptr+1:dummyDmaTxBuffer);
    tx->n = cnt-1;
    tx->status = DMA_TRANSACTION_FAILED;

    rx->dst = dmaMap(spid->rxptr?:dummyDmaRxBuffer);
    rx->n = cnt;
    rx->status = DMA_TRANSACTION_FAILED;

    __spi_dma_set_completion(spid, rx, tx);

    dmaStartTransactionI(&spid->txDmaChan, tx);
    dmaStartTransactionI(&spid->rxDmaChan, rx);

    if (spid->txptr)
      port->buf = *spid->txptr;
    else
      port->buf = 0xFFFFFFFF;
  }
}

static void __spi_dma_rx_complete(struct dmaTransaction *tr) {
  SPIDriver *spid = container_of(tr, SPIDriver, rxDmaTransaction);

  /* chDbgAssert(DMA_CHANNEL_ACTIVE == spid->txDmaChan.state, */
  /*     "DMA TX transaction was not complete", ""); */

  if (spid->txDmaChan.state != DMA_CHANNEL_ACTIVE)
    spid->txDmaTransaction.cb = __spi_dma_tx_complete;
  else
    __spi_dma_complete(spid, tr, &spid->txDmaTransaction, tr->n);
}

static void __spi_dma_tx_complete(struct dmaTransaction *tr) {
  SPIDriver *spid = container_of(tr, SPIDriver, txDmaTransaction);

  /* chDbgAssert(DMA_CHANNEL_ACTIVE == spid->rxDmaChan.state, */
  /*     "DMA RX transaction was not complete", ""); */

  if (spid->rxDmaChan.state != DMA_CHANNEL_ACTIVE)
    spid->rxDmaTransaction.cb = __spi_dma_rx_complete;
  else
    __spi_dma_complete(spid, &spid->rxDmaTransaction, tr, tr->n+1);
}

static void __spi_start_transaction(SPIDriver *spid) {
  SpiPort *port = (SpiPort *)spid->base;

  if (spid->config->dmad && spid->cnt > 1) {
    dmaTransaction *tx = &spid->txDmaTransaction;
    dmaTransaction *rx = &spid->rxDmaTransaction;
    uint32_t cnt = spid->cnt;

    if (!spid->txptr || !spid->txptr)
      cnt = min(spid->cnt, SPI_DUMMY_DMA_BUFFER_SIZE);

    tx->src = dmaMap(spid->txptr?(uint8_t *)spid->txptr+1:dummyDmaTxBuffer);
    tx->n = cnt-1;
    tx->status = DMA_TRANSACTION_FAILED;

    rx->dst = dmaMap(spid->rxptr?:dummyDmaRxBuffer);
    rx->n = cnt;
    rx->status = DMA_TRANSACTION_FAILED;

#if HAL_USE_EIC
    eicDisableIrq(spid->config->rx_irq);
#endif

    __spi_dma_set_completion(spid, rx, tx);

    dmaStartTransactionI(&spid->txDmaChan, tx);
    dmaStartTransactionI(&spid->rxDmaChan, rx);

    if (spid->txptr)
      port->buf = *spid->txptr;
    else
      port->buf = 0xFFFFFFFF;
  } else {
    if (spid->txptr)
      port->buf = *spid->txptr;
    else
      port->buf = 0xFFFFFFFF;
  }
}

static void __spi_finish_transaction(SPIDriver *spid) {
  SpiPort *port = (SpiPort *)spid->base;
  uint32_t reg;

  reg = port->buf;
#if HAL_USE_EIC
  eicAckIrq(spid->config->rx_irq);
#endif
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
static void lld_serve_rx_interrupt(uint32_t irq, void *data) {
  SPIDriver *spid = data;
  SpiPort *port = (SpiPort *)spid->base;

  (void)irq;

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
  uint32_t i;

  for (i=0;i<SPI_DUMMY_DMA_BUFFER_SIZE;++i)
    dummyDmaTxBuffer[i] = 0xFF;
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

  spid->base = (void *)cfg->base;
  __spi_config(spid);

  if (cfg->dmad && SPI_STOP == spid->state) {
    SpiPort *port = (SpiPort *)spid->base;
    dmaTransaction *tx = &spid->txDmaTransaction;
    dmaTransaction *rx = &spid->rxDmaTransaction;
    dmaChannelCfg ccfg = {
      .prio = DMA_CHANNEL_PRIO_LOWEST,
      .fifownd = 1,
      .evt = TRUE,
      .eirq = cfg->rx_irq,
    };

    dmaChannelObjectInit(cfg->dmad, &spid->txDmaChan);
    dmaChannelObjectInit(cfg->dmad, &spid->rxDmaChan);

    ccfg.mode = DMA_CHANNEL_MEM_TO_FIFO;
    chDbgAssert(dmaChannelAcquire(&spid->txDmaChan, TIME_IMMEDIATE),
        "Unable to acquire DMA channel for TX SPI", "");
    dmaChannelConfig(&spid->txDmaChan, &ccfg);
    dmaChannelStart(&spid->txDmaChan);

    ccfg.mode = DMA_CHANNEL_FIFO_TO_MEM;
    chDbgAssert(dmaChannelAcquire(&spid->rxDmaChan, TIME_IMMEDIATE),
        "Unable to acquire DMA channel for RX SPI", "");
    dmaChannelConfig(&spid->rxDmaChan, &ccfg);
    dmaChannelStart(&spid->rxDmaChan);

    spid->dmaPort = dmaMap((uint8_t *)&port->buf);
    rx->src = spid->dmaPort;
    tx->dst = spid->dmaPort;
  }

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

  if (cfg->dmad) {
    dmaChannelStop(&spid->rxDmaChan);
    dmaChannelRelease(&spid->rxDmaChan);

    dmaChannelStop(&spid->txDmaChan);
    dmaChannelRelease(&spid->txDmaChan);

    dmaUnmap(spid->dmaPort);
  }
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
  SpiPort *port = (SpiPort *)spid->base;

#if HAL_USE_EIC
  eicDisableIrq(cfg->rx_irq);
#endif

  port->buf = frame;

  while (!(port->status.reg & (1 << SPI_STATUS_RBF)));

  frame = port->buf;

#if HAL_USE_EIC
  eicAckIrq(cfg->rx_irq);
  eicEnableIrq(cfg->rx_irq);
#endif

  return frame;
}

#endif /* HAL_USE_SPI */

/** @} */
