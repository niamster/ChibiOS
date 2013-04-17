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
 * @file    MIPS-PIC32MX/dma_lld.c
 * @brief   MIPS-PIC32MX low level DMA driver code.
 *
 * @addtogroup DMA
 * @{
 */

#include "ch.h"
#include "hal.h"

#if HAL_USE_DMA || defined(__DOXYGEN__)

#include "mcu/pic32mxxx.h"

/*===========================================================================*/
/* Driver constants and error checks.                                        */
/*===========================================================================*/

#if defined(_DCH7CON_w_POSITION)
#define DMA_MAX_CHANNELS    8
#elif defined(_DCH3CON_w_POSITION)
#define DMA_MAX_CHANNELS    4
#else
#error DMA LLD can not detect max number of DMA channels
#endif

#if defined(__32MX320F032H__)
  || defined(__32MX320F064H__)
  || defined(__32MX320F128H__)
  || defined(__32MX320F128L__)
  || defined(__32MX330F064H__)
  || defined(__32MX330F064L__)
  || defined(__32MX340F128H__)
  || defined(__32MX340F128L__)
  || defined(__32MX340F256H__)
  || defined(__32MX340F512H__)
  || defined(__32MX350F128H__)
  || defined(__32MX350F128L__)
  || defined(__32MX350F256H__)
  || defined(__32MX350F256L__)
  || defined(__32MX360F256L__)
  || defined(__32MX360F512L__)
  || defined(__32MX420F032H__)
  || defined(__32MX430F064H__)
  || defined(__32MX430F064L__)
  || defined(__32MX440F128H__)
  || defined(__32MX440F128L__)
  || defined(__32MX440F256H__)
  || defined(__32MX440F512H__)
  || defined(__32MX450F128H__)
  || defined(__32MX450F128L__)
  || defined(__32MX450F256H__)
  || defined(__32MX450F256L__)
  || defined(__32MX460F256L__)
  || defined(__32MX460F512L__)
#define DMA_TRANSACTION_MAX_SIZE  255
#else
#warning Using 65535 DMA max block size, check your MCU manual if this value is applicable
#define DMA_TRANSACTION_MAX_SIZE  65535
#endif

/**
 * @brief   Configuration bits of dmacon register.
 */
enum dmaConBits {
  DMA_CON_ON      = 15, /* DMA Module On */
};

/**
 * @brief   Configuration bits of dchxcon register.
 */
enum dmaChanConBits {
  DMA_CHAN_CON_EN      = 7, /* DMA Channel Enabled */
  DMA_CHAN_CON_CHN     = 5, /* DMA Channel Chain Enable */
  DMA_CHAN_CON_AEN     = 4, /* DMA Channel Automatic Enable */
  DMA_CHAN_CON_DET     = 2, /* DMA Channel Event Detected */

  /* Channel Priority */
  DMA_CHAN_CON_CHPRI_MASK  = 0x3,
  DMA_CHAN_CON_CHPRI_SHIFT = 0,
};

/**
 * @brief   Configuration bits of dchxecon register.
 */
enum dmaChanEconBits {
  DMA_CHAN_ECON_CFORCE = 7, /* Force transfer */
  DMA_CHAN_ECON_CABORT = 6, /* Abort transfer */
  DMA_CHAN_ECON_PATEN  = 5, /* Channel Pattern Match Abort Enable */
  DMA_CHAN_ECON_SIRQEN = 4, /* Channel Start IRQ Enable */
  DMA_CHAN_ECON_AIRQEN = 3, /* Channel Abort IRQ Enable */

  /* IRQ that will Start Channel Transfer */
  DMA_CHAN_ECON_CHSIRQ_MASK  = 0xFF,
  DMA_CHAN_ECON_CHSIRQ_SHIFT = 8,

  /* IRQ that will Abort Channel Transfer */
  DMA_CHAN_ECON_CHAIRQ_MASK  = 0xFF,
  DMA_CHAN_ECON_CHAIRQ_SHIFT = 16,
};

/**
 * @brief   Configuration bits of dchxintr register.
 */
enum dmaChanIntrBits {
  DMA_CHAN_INTR_CHSDIE = 23, /* Channel Source Done Interrupt Enable */
  DMA_CHAN_INTR_CHSHIE = 22, /* Channel Source Half Empty Interrupt Enable */
  DMA_CHAN_INTR_CHDDIE = 21, /* Channel Destination Done Interrupt Enable */
  DMA_CHAN_INTR_CHDHIE = 20, /* Channel Destination Half Full Interrupt Enable */
  DMA_CHAN_INTR_CHBCIE = 19, /* Channel Block Transfer Complete Interrupt Enable */
  DMA_CHAN_INTR_CHCCIE = 18, /* Channel Cell Transfer Complete Interrupt Enable */
  DMA_CHAN_INTR_CHTAIE = 17, /* Channel Transfer Abort Interrupt Enable */
  DMA_CHAN_INTR_CHERIE = 16, /* Channel Address Error Interrupt Enable */

  DMA_CHAN_INTR_CHSDIF = 7, /* Channel Source Done Interrupt Flag */
  DMA_CHAN_INTR_CHSHIF = 6, /* Channel Source Half Empty Interrupt Flag */
  DMA_CHAN_INTR_CHDDIF = 5, /* Channel Destination Done Interrupt Flag */
  DMA_CHAN_INTR_CHDHIF = 4, /* Channel Destination Half Full Interrupt Flag */
  DMA_CHAN_INTR_CHBCIF = 3, /* Channel Block Transfer Complete Interrupt Flag */
  DMA_CHAN_INTR_CHCCIF = 2, /* Channel Cell Transfer Complete Interrupt Flag */
  DMA_CHAN_INTR_CHTAIF = 1, /* Channel Transfer Abort Interrupt Flag */
  DMA_CHAN_INTR_CHERIF = 0, /* Channel Address Error Interrupt Flag */
};

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

typedef volatile struct {
  PicReg   dmacon;
  PicReg   dmastat;
  PicReg   dmaaddr;
  PicReg   dcrccon;
  PicReg   dcrcdata;
  PicReg   dcrcxor;
  struct dchx {
    PicReg   con;
    PicReg   econ;
    PicReg   intr;
    PicReg   ssa;
    PicReg   dsa;
    PicReg   ssiz;
    PicReg   dsiz;
    PicReg   sptr;
    PicReg   dptr;
    PicReg   csiz;
    PicReg   cptr;
    PicReg   dat;
  } dchx[DMA_MAX_CHANNELS];
} DmaPort;

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables.                                                   */
/*===========================================================================*/

static const uint8_t idToIrq[] = {
  EIC_IRQ_DMA0,
  EIC_IRQ_DMA1,
  EIC_IRQ_DMA2,
  EIC_IRQ_DMA3,
#if DMA_MAX_CHANNELS > 4
  EIC_IRQ_DMA4,
  EIC_IRQ_DMA5,
  EIC_IRQ_DMA6,
  EIC_IRQ_DMA7,
#endif
#if DMA_MAX_CHANNELS > 8
#error DMA channel ID to IRQ mapping is incomplete
#endif
};

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

static uint8_t dma_lld_get_channel(dmaDriver *dmad) {
  uint8_t ch;

  for (ch=0;ch<DMA_MAX_CHANNELS;++ch)
    if (dmad->channels & (1 << ch)) {
      dmad->channels &= ~(1 << ch);
      break;
    }

  return ch;
}

static void dma_lld_put_channel(dmaDriver *dmad, uint8_t ch) {
  dmad->channels |= 1 << ch;
}

static void dma_lld_process_transaction(dmaChannel *chan, bool_t poll) {
  struct dmaTransaction *tr = chan->current;
  dmaDriver *dmad = chan->dmad;
  DmaPort *port = dmad->port;
  volatile struct dchx *dchx = port->dchx + chan->id;
  dmaChannelCfg *cfg = &chan->cfg;
  size_t n = chan->n;

  dchx->ssa.reg = chan->src;
  dchx->dsa.reg = chan->dst;

  if (n > DMA_TRANSACTION_MAX_SIZE)
    n = DMA_TRANSACTION_MAX_SIZE;
  chan->n -= n;

  dchx->econ.clear = 1 << DMA_CHAN_ECON_SIRQEN;

  if (DMA_CHANNEL_MEM_TO_FIFO == cfg->mode
      || DMA_CHANNEL_FIFO_TO_MEM == cfg->mode) {
    if (DMA_CHANNEL_FIFO_TO_MEM == cfg->mode) {
      if (cfg->evt) {
        dchx->csiz.reg = dchx->ssiz.reg = cfg->fifownd;
        dchx->econ.clear = DMA_CHAN_ECON_CHSIRQ_MASK << DMA_CHAN_ECON_CHSIRQ_SHIFT;
        dchx->econ.set = (cfg->eirq << DMA_CHAN_ECON_CHSIRQ_SHIFT) | (1 << DMA_CHAN_ECON_SIRQEN);
      } else {
        dchx->csiz.reg = n;
        dchx->ssiz.reg = cfg->fifownd;
      }
      dchx->dsiz.reg = n;
      if (chan->n)
        chan->dst += n;
    } else {
      if (cfg->evt) {
        dchx->csiz.reg = dchx->dsiz.reg = cfg->fifownd;
        dchx->econ.clear = DMA_CHAN_ECON_CHSIRQ_MASK << DMA_CHAN_ECON_CHSIRQ_SHIFT;
        dchx->econ.set = (cfg->eirq << DMA_CHAN_ECON_CHSIRQ_SHIFT) | (1 << DMA_CHAN_ECON_SIRQEN);
      } else {
        dchx->csiz.reg = n;
        dchx->dsiz.reg = cfg->fifownd;
      }
      dchx->ssiz.reg = n;
      if (chan->n)
        chan->src += n;
    }
  } else {
    dchx->csiz.reg = dchx->dsiz.reg = dchx->ssiz.reg = n;
    if (chan->n) {
      chan->src += n;
      chan->dst += n;
    }
  }

  dchx->intr.reg = (1 << DMA_CHAN_INTR_CHBCIE)
    | (1 << DMA_CHAN_INTR_CHTAIE)
    | (1 << DMA_CHAN_INTR_CHERIE);
  dchx->con.set = 1 << DMA_CHAN_CON_EN;
  if (!cfg->evt)
    dchx->econ.set = 1 << DMA_CHAN_ECON_CFORCE;

  if (poll) {
    const uint32_t iflags = (1 << DMA_CHAN_INTR_CHBCIF)
      | (1 << DMA_CHAN_INTR_CHTAIF)
      | (1 << DMA_CHAN_INTR_CHERIF);
    uint32_t intr;

    while (!((intr = dchx->intr.reg) & iflags));
    dchx->intr.clear = 0xFF;

    if (intr & (1 << DMA_CHAN_INTR_CHERIF))
      tr->status = DMA_TRANSACTION_FAILED;
    else if (intr & (1 << DMA_CHAN_INTR_CHTAIF))
      tr->status = DMA_TRANSACTION_ABORTED;
    else /* if (intr & (1 << DMA_CHAN_INTR_CHBCIE)) */
      tr->status = DMA_TRANSACTION_SUCCESSFULL;
  }
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/**
 * @brief   DMA channel IRQ handler.
 *
 * @param[in] data       Driver associated with the DMA channel
 */
static void lld_serve_interrupt(uint32_t irq, void *data) {
  dmaChannel *chan = data;
  struct dmaTransaction *tr = chan->current;
  dmaDriver *dmad = chan->dmad;
  DmaPort *port = dmad->port;
  volatile struct dchx *dchx = port->dchx + chan->id;
  uint32_t intr;

  (void)irq;

  chSysLockFromIsr();

  intr = dchx->intr.reg;
  dchx->intr.clear = 0xFF;

  if (intr & (1 << DMA_CHAN_INTR_CHERIF))
    tr->status = DMA_TRANSACTION_FAILED;
  else if (intr & (1 << DMA_CHAN_INTR_CHTAIF))
    tr->status = DMA_TRANSACTION_ABORTED;
  else /* if (intr & (1 << DMA_CHAN_INTR_CHBCIE)) */
    tr->status = DMA_TRANSACTION_SUCCESSFULL;

  if (chan->n && DMA_TRANSACTION_SUCCESSFULL == tr->status)
    dma_lld_process_transaction(chan, FALSE);
  else
    _dma_isr_code(chan, tr);

  chSysUnlockFromIsr();
}

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level DMA driver initialization.
 *
 * @notapi
 */
void dma_lld_init(void) {
}

/**
 * @brief   Configures the DMA peripheral.
 *
 * @param[in] dmad      pointer to the @p dmaDriver object
 * @param[in] cfg       pointer to the @p dmaCfg object
 *
 * @notapi
 */
void dma_lld_config(dmaDriver *dmad, const dmaCfg *cfg) {
  uint8_t i;

  chDbgAssert(cfg->port, "dma_lld_config(), #1", "invalid configuration");

  dmad->port = (void *)cfg->port;

  dmad->channels = 0;
  for (i=0;i<DMA_MAX_CHANNELS;++i)
    dmad->channels |= 1 << i;

  ((DmaPort *)dmad->port)->dmacon.reg = 0;
}

/**
 * @brief   Activates the DMA peripheral.
 *
 * @param[in] dmad      pointer to the @p dmaDriver object
 *
 * @notapi
 */
void dma_lld_start(dmaDriver *dmad) {
  DmaPort *port = dmad->port;

  port->dmacon.set = 1 << DMA_CON_ON;
}

/**
 * @brief Deactivates the DMA peripheral.
 *
 * @param[in] dmad      pointer to the @p dmaDriver object
 *
 * @notapi
 */
void dma_lld_stop(dmaDriver *dmad) {
  DmaPort *port = dmad->port;

  port->dmacon.clear = 1 << DMA_CON_ON;
}

/**
 * @brief   Configures the DMA channel.
 *
 * @param[in] chan      pointer to the @p dmaChannel object
 * @param[in] cfg       pointer to the @p dmaChannelCfg object
 *
 * @notapi
 */
void dma_lld_channel_config(dmaChannel *chan, const dmaChannelCfg *cfg) {
  if (cfg) {
    chDbgAssert(/* cfg->prio >= DMA_CHANNEL_PRIO_LOWEST &&  */cfg->prio <= DMA_CHANNEL_PRIO_HIGHEST,
        "dma_lld_channel_config(), #1", "wrong channel priority");

    chan->cfg = *cfg;
  }
}

/**
 * @brief   Activates the DMA channel.
 *
 * @param[in] chan      pointer to the @p dmaChannel object
 *
 * @notapi
 */
void dma_lld_channel_start(dmaChannel *chan) {
  dmaDriver *dmad = chan->dmad;
  uint8_t id = dma_lld_get_channel(dmad);
  DmaPort *port = dmad->port;
  volatile struct dchx *dchx = port->dchx + id;
  dmaChannelCfg *cfg = &chan->cfg;

  chDbgAssert(id < DMA_MAX_CHANNELS,
      "dma_lld_channel_start(), #1", "no available channels");

  chan->id = id;
  chan->irq = idToIrq[id];
  dchx->con.set = (cfg->prio & DMA_CHAN_CON_CHPRI_MASK) << DMA_CHAN_CON_CHPRI_SHIFT;

#if HAL_USE_EIC
  eicRegisterIrq(chan->irq, lld_serve_interrupt, chan);
  eicEnableIrq(chan->irq);
#endif
}

/**
 * @brief Deactivates the DMA channel.
 *
 * @param[in] chan      pointer to the @p dmaChannel object
 *
 * @notapi
 */
void dma_lld_channel_stop(dmaChannel *chan) {
  dmaDriver *dmad = chan->dmad;

#if HAL_USE_EIC
  eicDisableIrq(chan->irq);
  eicUnregisterIrq(chan->irq);
#endif

  dma_lld_put_channel(dmad, chan->id);
}

/**
 * @brief   Starts DMA transaction.
 * @details May block the caller in in polled mode.
 * @post    If in non-polled mode, at the end of the operation the configured callback is invoked(by ISR).
 * @note    The caller may lock the system before calling this function.
 *
 * @param[in] chan      pointer to the @p dmaChannel object
 * @param[in] tr        pointer to the @p dmaTransaction object
 * @param[in] poll      flag to poll for the end of the transaction
 *
 * @notapi
 */
void dma_lld_start_transaction(dmaChannel *chan, struct dmaTransaction *tr, bool_t poll) {
  chDbgAssert(tr->n, "dma_lld_start_transaction(), #1", "invalid transaction size");

  chan->current = tr;
  chan->src = tr->src;
  chan->dst = tr->dst;
  chan->n = tr->n;

  if (!poll)
    dma_lld_process_transaction(chan, FALSE);
  else {
#if HAL_USE_EIC
    eicDisableIrq(chan->irq);
#endif

    do {
      dma_lld_process_transaction(chan, TRUE);

      if (tr->status != DMA_TRANSACTION_SUCCESSFULL)
        break;
    } while (chan->n);

#if HAL_USE_EIC
    eicAckIrq(chan->irq);
    eicEnableIrq(chan->irq);
#endif
  }
}

/**
 * @brief   Aborts current DMA transaction.
 * @details The transaction may be completed successfully
 *          which will be reflected in transaction status.
 *
 * @param[in] chan      pointer to the @p dmaChannel object
 *
 * @api
 */
void dma_lld_abort_transaction(dmaChannel *chan) {
  dmaDriver *dmad = chan->dmad;
  DmaPort *port = dmad->port;
  volatile struct dchx *dchx = port->dchx + chan->id;

  dchx->econ.set = 1 << DMA_CHAN_ECON_CABORT;
}

#endif /* HAL_USE_DMA */

/** @} */
