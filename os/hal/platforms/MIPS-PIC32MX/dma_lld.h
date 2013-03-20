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
 * @file    MIPS-PIC32MX/dma_lld.h
 * @brief   MIPS-PIC32MX low level DMA driver header.
 *
 * @addtogroup DMA
 * @{
 */

#ifndef _DMA_LLD_H_
#define _DMA_LLD_H_

#if HAL_USE_DMA || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

enum dmaChannelPriority {
  DMA_CHANNEL_PRIORITY0,        /* The lowest priority */
  DMA_CHANNEL_PRIORITY1,
  DMA_CHANNEL_PRIORITY2,
  DMA_CHANNEL_PRIORITY3,        /* The highest priority */

  DMA_CHANNEL_PRIO_LOWEST  = DMA_CHANNEL_PRIORITY0,
  DMA_CHANNEL_PRIO_HIGHEST = DMA_CHANNEL_PRIORITY3,
};

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
 * @brief   A pointer type suitable for DMA transaction.
 */
typedef uint32_t dmaptr_t;

/**
 * @brief   Structure representing a DMA driver configuration.
 */
typedef struct dmaCfg {
  /**
   * @brief DMA port.
   */
  uint32_t               port;
} dmaCfg;

/**
 * @brief   DMA channel operation mode.
 */
enum dmaChannelMode {
  DMA_CHANNEL_MEM_TO_MEM,
  DMA_CHANNEL_MEM_TO_FIFO,
  DMA_CHANNEL_FIFO_TO_MEM,
};

/**
 * @brief   Structure representing a DMA channel configuration.
 */
typedef struct dmaChannelCfg {
  /**
   * @brief Channel priority.
   */
  enum dmaChannelPriority prio;

  /**
   * @brief Destination address is a FIFO and address
   *        is not autoincremented.
   */
  enum dmaChannelMode     mode;
  /**
   * @brief The width of destination or source FIFO window in bytes
   *        if applicable.
   */
  uint8_t                 fifownd;
  /**
   * @brief External event is used to start DMA transaction.
   */
  bool_t                  evt;
  /**
   * @brief IRQ number as event to start DMA transaction.
   */
  uint8_t                 eirq;
} dmaChannelCfg;

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @brief LLD-specific DMA driver attributes.
 */
#define DMA_DRIVER_EXT_FIELDS                     \
  void                    *port;                  \
  uint8_t                 channels;

/**
 * @brief LLD-specific DMA channel attributes.
 */
#define DMA_CHANNEL_EXT_FIELDS                         \
  uint8_t                 irq;                         \
  struct dmaChannelCfg    cfg;                         \
  uint8_t                 id;                          \
  dmaTransaction          *current;                    \
  uint32_t                dst;                         \
  uint32_t                src;                         \
  size_t                  n;

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  void dma_lld_init(void);

  void dma_lld_config(struct dmaDriver *dmad, const dmaCfg *cfg);
  void dma_lld_start(struct dmaDriver *dmad);
  void dma_lld_stop(struct dmaDriver *dmad);

  void dma_lld_channel_config(struct dmaChannel *chan, const dmaChannelCfg *cfg);
  void dma_lld_channel_start(struct dmaChannel *chan);
  void dma_lld_channel_stop(struct dmaChannel *chan);

  void dma_lld_start_transaction(struct dmaChannel *chan, struct dmaTransaction *tr, bool_t poll);
  void dma_lld_abort_transaction(struct dmaChannel *chan);

  static inline dmaptr_t dma_lld_map(void *ptr) {
    return (dmaptr_t)MIPS_PHYSICAL((uint32_t)ptr);
  }
  static inline void dma_lld_unmap(dmaptr_t ptr) {
    (void)ptr;
  }
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_DMA */

#endif /* _DMA_LLD_H_ */

/** @} */
