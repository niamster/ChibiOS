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
 * @file    dma.c
 * @brief   DMA Driver code.
 *
 * @addtogroup DMA
 * @{
 */

#include "ch.h"
#include "hal.h"

#if HAL_USE_DMA || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/**
 * @brief   Structure representing a private view of DMA transaction.
 */
typedef struct {
  /**
   * @brief   DMA transaction.
   */
  dmaTransaction tr;
  /**
   * @brief   Thread waiting for the completion of the transaction.
   */
  Thread *th;
} dmaPriv;

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/**
 * @brief   Wakes up the thread waiting for the completion of the DMA transaction.
 *
 * @notapi
 */
static void dma_complete(struct dmaTransaction *tr) {
  dmaPriv *priv = (dmaPriv *)tr;

  chSysLockFromIsr();
  chSchReadyI(priv->th);
  chSysUnlockFromIsr();
}

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   DMA Driver initialization.
 * @note    This function is implicitly invoked by @p halInit(), there is
 *          no need to explicitly initialize the driver.
 *
 * @init
 */
void dmaInit(void) {
  dma_lld_init();
}

/**
 * @brief   Initializes the standard part of a @p dmaDriver structure.
 *
 * @param[out] dmad     pointer to the @p dmaDriver object
 *
 * @init
 */
void dmaObjectInit(dmaDriver *dmad) {
  chDbgCheck(dmad, "dmaObjectInit");

  dmad->state = DMA_DRIVER_UNINIT;
}

/**
 * @brief   Configures the DMA peripheral.
 *
 * @param[in] dmad      pointer to the @p dmaDriver object
 * @param[in] cfg       pointer to the @p dmaCfg object
 *
 * @api
 */
void dmaConfig(dmaDriver *dmad, const dmaCfg *cfg) {
  chDbgCheck(dmad, "dmaCfg");

  chSysLock();
  chDbgAssert(dmad->state == DMA_DRIVER_INACTIVE
      || dmad->state == DMA_DRIVER_UNINIT,
              "dmaCfg(), #1", "invalid state");
  dma_lld_config(dmad, cfg);
  dmad->state = DMA_DRIVER_INACTIVE;
  chSysUnlock();
}

/**
 * @brief   Activates the DMA peripheral.
 *
 * @param[in] dmad      pointer to the @p dmaDriver object
 *
 * @api
 */
void dmaStart(dmaDriver *dmad) {
  chDbgCheck(dmad, "dmaStart");

  chSysLock();
  chDbgAssert(dmad->state == DMA_DRIVER_INACTIVE,
              "dmaStart(), #1", "invalid state");
  dma_lld_start(dmad);
  dmad->state = DMA_DRIVER_ACTIVE;
  chSysUnlock();
}

/**
 * @brief Deactivates the DMA peripheral.
 *
 * @param[in] dmad      pointer to the @p dmaDriver object
 *
 * @api
 */
void dmaStop(dmaDriver *dmad) {
  chDbgCheck(dmad, "dmaStop");

  chSysLock();
  chDbgAssert(dmad->state == DMA_DRIVER_ACTIVE,
              "dmaStop(), #1", "invalid state");
  dma_lld_stop(dmad);
  dmad->state = DMA_DRIVER_INACTIVE;
  chSysUnlock();
}

/**
 * @brief   Initializes the standard part of a @p dmaChannel structure.
 *
 * @param[in]  dmad     pointer to the @p dmaDriver object
 * @param[out] chan     pointer to the @p dmaChannel object
 *
 * @init
 */
void dmaChannelObjectInit(dmaDriver *dmad, dmaChannel *chan) {
  chDbgCheck(dmad && chan, "dmaChannelObjectInit");

  chan->dmad = dmad;
  chan->state = DMA_CHANNEL_UNINIT;
  chan->acquired = FALSE;
}

/**
 * @brief   Configures the DMA channel.
 *
 * @param[in] chan      pointer to the @p dmaChannel object
 * @param[in] cfg       pointer to the @p dmaChannelCfg object
 *
 * @api
 */
void dmaChannelConfig(dmaChannel *chan, const dmaChannelCfg *cfg) {
  chDbgCheck(chan, "dmaCfg");

  chSysLock();
  chDbgAssert(chan->state == DMA_CHANNEL_INACTIVE
      || chan->state == DMA_CHANNEL_UNINIT,
              "dmaChannelCfg(), #1", "invalid state");
  chDbgAssert(chan->acquired, "dmaChannelCfg(), #2", "invalid state");
  dma_lld_channel_config(chan, cfg);
  chan->state = DMA_CHANNEL_INACTIVE;
  chSysUnlock();
}

/**
 * @brief   Activates the DMA channel.
 *
 * @param[in] chan      pointer to the @p dmaChannel object
 *
 * @api
 */
void dmaChannelStart(dmaChannel *chan) {
  chDbgCheck(chan, "dmaChannelStart");

  chSysLock();
  chDbgAssert(chan->state == DMA_CHANNEL_INACTIVE,
              "dmaChannelStart(), #1", "invalid state");
  chDbgAssert(chan->acquired, "dmaChannelStart(), #2", "invalid state");
  dma_lld_channel_start(chan);
  chan->state = DMA_CHANNEL_ACTIVE;
  chSysUnlock();
}

/**
 * @brief Deactivates the DMA channel.
 *
 * @param[in] chan      pointer to the @p dmaChannel object
 *
 * @api
 */
void dmaChannelStop(dmaChannel *chan) {
  chDbgCheck(chan, "dmaChannelStop");

  chSysLock();
  chDbgAssert(chan->state == DMA_CHANNEL_ACTIVE,
              "dmaStop(), #1", "invalid state");
  chDbgAssert(chan->acquired, "dmaChannelStop(), #2", "invalid state");
  dma_lld_channel_stop(chan);
  chan->state = DMA_CHANNEL_INACTIVE;
  chSysUnlock();
}

/**
 * @brief   Acquires DMA channel.
 *
 * @param[out] chan     pointer to the @p dmaChannel object
 * @param[in] time      the number of ticks before the operation timeouts, the
 *                      special values are handled as follow:
 *                      - @a TIME_INFINITE the thread enters an infinite sleep
 *                        state, this is equivalent to invoking
 *                        @p chSchGoSleepS() but, of course, less efficient.
 *                      - @a TIME_IMMEDIATE the routine will return immediatelly
 *                        if operation may block the context.
 *                      .
 * @return              TRUE on success.
 *
 * @api
 */
bool_t dmaChannelAcquire(dmaChannel *chan, systime_t time) {
  bool_t acquired = FALSE;

  chSysLock();

  if (!chan->acquired) {
    acquired = chan->acquired = TRUE;
    goto out;
  }

  if (TIME_IMMEDIATE == time)
    goto out;

  if (TIME_INFINITE == time) {
    while (chan->acquired)
      chThdSleepS(1);

    acquired = chan->acquired = TRUE;
  } else {
    while (time--) {
      chThdSleepS(1);

      if (!chan->acquired) {
        acquired = chan->acquired = TRUE;
        break;
      }
    }
  }

 out:
  chSysUnlock();

  return acquired;
}

/**
 * @brief   Releases DMA channel.
 *
 * @param[in] chan      pointer to the @p dmaChannel object
 *
 * @notapi
 */
void dmaChannelRelease(dmaChannel *chan) {
  chan->acquired = FALSE;
}

/**
 * @brief   Starts DMA transaction.
 * @details Does not block the caller.
 * @post    At the end of the operation the configured callback is invoked.
 * @note    Can be called only in ISR context.
 *
 * @param[in] chan      pointer to the @p dmaChannel object
 * @param[in] tr        pointer to the @p dmaTransaction object
 *
 * @api
 */
void dmaStartTransactionI(dmaChannel *chan, dmaTransaction *tr) {
  chDbgAssert(chan->acquired, "dmaStartTransaction(), #1", "invalid state");
  chDbgAssert(chan->state == DMA_CHANNEL_ACTIVE || chan->state == DMA_CHANNEL_COMPLETE,
              "dmaStartTransaction(), #2", "invalid state");
  chDbgAssert(chan->dmad->state == DMA_DRIVER_ACTIVE,
              "dmaStartTransaction(), #3", "invalid state");

  chan->state = DMA_CHANNEL_RUNNNING;
  dma_lld_start_transaction(chan, tr, FALSE);
}

/**
 * @brief   Starts DMA transaction.
 * @details Does not block the caller.
 * @post    At the end of the operation the configured callback is invoked.
 *
 * @param[in] chan      pointer to the @p dmaChannel object
 * @param[in] tr        pointer to the @p dmaTransaction object
 *
 * @api
 */
void dmaStartTransaction(dmaChannel *chan, dmaTransaction *tr) {
  chSysLock();
  dmaStartTransactionI(chan, tr);
  chSysUnlock();
}

/**
 * @brief   Starts DMA transaction.
 * @details Block the caller.
 * @post    At the end of the operation the configured callback is invoked.
 *
 * @param[in] chan      pointer to the @p dmaChannel object
 * @param[in] tr        pointer to the @p dmaTransaction object
 *
 * @api
 */
void dmaStartPolledTransaction(dmaChannel *chan, dmaTransaction *tr) {
  chSysLock();
  chDbgAssert(chan->acquired, "dmaStartPolledTransaction(), #1", "invalid state");
  chDbgAssert(chan->state == DMA_CHANNEL_ACTIVE || chan->state == DMA_CHANNEL_COMPLETE,
              "dmaStartPolledTransaction(), #2", "invalid state");
  chDbgAssert(chan->dmad->state == DMA_DRIVER_ACTIVE,
              "dmaStartPolledTransaction(), #3", "invalid state");

  chan->state = DMA_CHANNEL_RUNNNING;
  chSysUnlock();

  dma_lld_start_transaction(chan, tr, TRUE);

  chSysLock();
  _dma_isr_code(chan, tr);
  chSysUnlock();
}

/**
 * @brief   Starts DMA transaction.
 * @details Block the caller.
 * @post    At the end of the operation the configured callback is invoked.
 * @note    Can be called only in ISR context.
 *
 * @param[in] chan      pointer to the @p dmaChannel object
 * @param[in] tr        pointer to the @p dmaTransaction object
 *
 * @api
 */
void dmaStartPolledTransactionI(dmaChannel *chan, dmaTransaction *tr) {
  chDbgAssert(chan->acquired, "dmaStartPolledTransactionI(), #1", "invalid state");
  chDbgAssert(chan->state == DMA_CHANNEL_ACTIVE || chan->state == DMA_CHANNEL_COMPLETE,
              "dmaStartPolledTransactionI(), #2", "invalid state");
  chDbgAssert(chan->dmad->state == DMA_DRIVER_ACTIVE,
              "dmaStartPolledTransactionI(), #3", "invalid state");

  chan->state = DMA_CHANNEL_RUNNNING;

  dma_lld_start_transaction(chan, tr, TRUE);

  _dma_isr_code(chan, tr);
}

/**
 * @brief   Starts DMA transaction.
 * @details Blocks the caller till the end of the transaction.
 * @note    May sleep
 *
 * @param[in] chan      pointer to the @p dmaChannel object
 * @param[in] dst       destination address
 * @param[in] src       source address
 * @param[in] n         length
 * @return              transaction completion status
 *
 * @api
 */
dmatrstate_t dmaStartSimpleTransaction(dmaChannel *chan,
    dmaptr_t dst, dmaptr_t src, size_t n) {
  dmaPriv tr = {
    .tr = {
      .src = src,
      .dst = dst,
      .n = n,
      .status = DMA_TRANSACTION_FAILED,
      .cb = dma_complete,
    },
    .th = chThdSelf(),
  };

  chSysLock();
  chDbgAssert(chan->acquired, "dmaStartSimpleTransaction(), #1", "invalid state");
  chDbgAssert(chan->state == DMA_CHANNEL_ACTIVE,
              "dmaStartSimpleTransaction(), #2", "invalid state");
  chDbgAssert(chan->dmad->state == DMA_DRIVER_ACTIVE,
              "dmaStartSimpleTransaction(), #3", "invalid state");

  chan->state = DMA_CHANNEL_RUNNNING;
  dma_lld_start_transaction(chan, (dmaTransaction *)&tr, FALSE);

  chSchGoSleepS(THD_STATE_SUSPENDED);
  chSysUnlock();

  return tr.tr.status;
}

/**
 * @brief   Starts DMA transaction.
 * @details Blocks the caller till the end of the transaction.
 * @note    Never sleeps
 *
 * @param[in] chan      pointer to the @p dmaChannel object
 * @param[in] dst       destination address
 * @param[in] src       source address
 * @param[in] n         length
 * @return              transaction completion status
 *
 * @api
 */
dmatrstate_t dmaStartSimplePolledTransaction(dmaChannel *chan,
    dmaptr_t dst, dmaptr_t src, size_t n) {
  dmaTransaction tr = {
    .src = src,
    .dst = dst,
    .n = n,
    .status = DMA_TRANSACTION_FAILED,
  };

  chSysLock();
  chDbgAssert(chan->acquired, "dmaStartTransaction(), #1", "invalid state");
  chDbgAssert(chan->state == DMA_CHANNEL_ACTIVE,
              "dmaStartTransaction(), #2", "invalid state");
  chDbgAssert(chan->dmad->state == DMA_DRIVER_ACTIVE,
              "dmaStartTransaction(), #3", "invalid state");

  chan->state = DMA_CHANNEL_RUNNNING;
  chSysUnlock();

  dma_lld_start_transaction(chan, &tr, TRUE);

  chSysLock();
  _dma_isr_code(chan, &tr);
  chSysUnlock();

  return tr.status;
}

/**
 * @brief   Aborts current DMA transactions.
 * @details The transaction may be completed successfully
 *          which will be reflected in transaction status.
 *
 * @param[in] chan      pointer to the @p dmaChannel object
 *
 * @api
 */
void dmaAbortTransaction(dmaChannel *chan) {
  chSysLock();
  chDbgAssert(chan->acquired, "dmaAbortTransaction(), #1", "invalid state");
  dma_lld_abort_transaction(chan);
  chSysUnlock();
}

#endif /* HAL_USE_DMA */

/** @} */
