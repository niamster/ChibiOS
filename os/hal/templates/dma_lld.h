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
 * @file    templates/dma_lld.h
 * @brief   Template for low level DMA driver header.
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
} dmaCfg;

/**
 * @brief   Structure representing a DMA channel configuration.
 */
typedef struct dmaChannelCfg {
  /**
   * @brief DMA destination properties.
   */
  struct {
    /**
     * @brief Destination address is a FIFO and address
     *        is not autoincremented.
     */
    bool_t          fifo;
    /**
     * @brief The width of destination FIFO window in bytes.
     */
    uint8_t         wnd;
  } dst;
  /**
   * @brief DMA source properties.
   */
  struct {
    /**
     * @brief Source address is a FIFO and address
     *        is not autoincremented.
     */
    bool_t          fifo;
    /**
     * @brief The width of source FIFO window in bytes.
     */
    uint8_t         wnd;
  } src;
} dmaChannelCfg;

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @brief LLD-specific DMA driver attributes.
 */
#define DMA_DRIVER_EXT_FIELDS                   \
  size_t                  channels;

/**
 * @brief LLD-specific DMA channel attributes.
 */
#define DMA_CHANNEL_EXT_FIELDS                  \
  size_t                  id;


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
    return (dmaptr_t)ptr;
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
