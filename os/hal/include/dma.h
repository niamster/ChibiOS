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
 * @file    dma.h
 * @brief   DMA Driver macros and structures.
 *
 * @addtogroup DMA
 * @{
 */

#ifndef _DMA_H_
#define _DMA_H_

#if HAL_USE_DMA || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Driver FSM possible states.
 */
typedef enum {
  DMA_DRIVER_UNINIT,                    /**< Not initialized.                   */
  DMA_DRIVER_INACTIVE,                  /**< Stopped.                           */
  DMA_DRIVER_ACTIVE,                    /**< Started.                           */
} dmadstate_t;

/**
 * @brief   Channel FSM possible states.
 */
typedef enum {
  DMA_CHANNEL_UNINIT,                   /**< Not initialized.                   */
  DMA_CHANNEL_INACTIVE,                 /**< Stopped.                           */
  DMA_CHANNEL_ACTIVE,                   /**< Started.                           */
  DMA_CHANNEL_RUNNNING,                 /**< Operation in progress.             */
  DMA_CHANNEL_COMPLETE,                 /**< Operation complete.                */
} dmachstate_t;

/**
 * @brief   Transaction completion status.
 */
typedef enum {
  DMA_TRANSACTION_FAILED,               /**< Failed.                            */
  DMA_TRANSACTION_SUCCESSFULL,          /**< Successfully completed.            */
  DMA_TRANSACTION_ABORTED,              /**< Aborted.                           */
} dmatrstate_t;

/**
 * @brief   Forward declaration of a structure representing a DMA driver.
 */
struct dmaDriver;

/**
 * @brief   Forward declaration of a structure representing a DMA channel.
 */
struct dmaChannel;

/**
 * @brief   Forward declaration of a structure representing a DMA transaction.
 */
struct dmaTransaction;

/**
 * @brief   DMA notification callback type.
 *
 * @param[in] tr      pointer to the @p dmaTransaction object triggering the
 *                    callback
 */
typedef void (*dmatrcallback_t)(struct dmaTransaction *tr);

#include "dma_lld.h"

/**
 * @brief   Structure representing a DMA transaction.
 */
typedef struct dmaTransaction {
  /**
   * @brief Destination address.
   */
  dmaptr_t        dst;
  /**
   * @brief Source address.
   */
  dmaptr_t        src;
  /**
   * @brief Length.
   */
   size_t         n;
  /**
   * @brief Operation completion status.
   */
  dmatrstate_t    status;
  /**
   * @brief Operation complete callback or @p NULL.
   */
  dmatrcallback_t cb;
} dmaTransaction;

/**
 * @brief Structure representing a DMA driver.
 */
typedef struct dmaDriver {
  /**
   * @brief Driver state.
   */
  dmadstate_t            state;
#if defined(DMA_DRIVER_EXT_FIELDS)
  DMA_DRIVER_EXT_FIELDS
#endif
} dmaDriver;

/**
 * @brief Structure representing a DMA channel.
 */
typedef struct dmaChannel {
  /**
   * @brief DMA driver to which this channel belongs
   */
  dmaDriver               *dmad;
  /**
   * @brief Channel state.
   */
  dmachstate_t            state;
  /**
   * @brief Channel ownership flag.
   */
  bool_t                  acquired;
#if defined(DMA_CHANNEL_EXT_FIELDS)
  DMA_CHANNEL_EXT_FIELDS
#endif
} dmaChannel;

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @brief   Common ISR code.
 * @details This code handles the portable part of the ISR code:
 *          - Callback invocation.
 *          - Channel state transitions.
 *          .
 * @note    This macro is meant to be used in the low level drivers
 *          implementation only.
 *
 * @param[in] ch        pointer to the @p dmaChannel object
 * @param[in] tr        pointer to the @p dmaTransaction object
 *
 * @notapi
 */
#define _dma_isr_code(ch, tr) do {                                    \
    if ((tr)->cb) {                                                   \
      (ch)->state = DMA_CHANNEL_COMPLETE;                             \
      (tr)->cb(tr);                                                   \
      if (DMA_CHANNEL_COMPLETE == (ch)->state)                        \
        (ch)->state = DMA_CHANNEL_ACTIVE;                             \
    } else                                                            \
      (ch)->state = DMA_CHANNEL_ACTIVE;                               \
  } while (0)

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  void dmaInit(void);

  void dmaObjectInit(dmaDriver *dmad);
  void dmaConfig(dmaDriver *dmad, const dmaCfg *cfg);
  void dmaStart(dmaDriver *dmad);
  void dmaStop(dmaDriver *dmad);

  void dmaChannelObjectInit(dmaDriver *dmad, dmaChannel *chan);
  void dmaChannelConfig(dmaChannel *chan, const dmaChannelCfg *cfg);
  void dmaChannelStart(dmaChannel *chan);
  void dmaChannelStop(dmaChannel *chan);

  bool_t dmaChannelAcquire(dmaChannel *chan, systime_t time);
  void dmaChannelRelease(dmaChannel *chan);

  void dmaStartTransaction(dmaChannel *chan, dmaTransaction *tr);
  void dmaStartTransactionI(dmaChannel *chan, dmaTransaction *tr);
  void dmaStartPolledTransaction(dmaChannel *chan, dmaTransaction *tr);
  void dmaStartPolledTransactionI(dmaChannel *chan, dmaTransaction *tr);

  dmatrstate_t dmaStartSimpleTransaction(dmaChannel *chan, dmaptr_t dst, dmaptr_t src, size_t n);
  dmatrstate_t dmaStartSimplePolledTransaction(dmaChannel *chan, dmaptr_t dst, dmaptr_t src, size_t n);
  void dmaAbortTransaction(dmaChannel *chan);

  static inline dmaptr_t dmaMap(void *ptr) {
    return dma_lld_map(ptr);
  }
  static inline void dmaUnmap(dmaptr_t ptr) {
    dma_lld_unmap(ptr);
  }
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_DMA */

#endif /* _DMA_H_ */

/** @} */
