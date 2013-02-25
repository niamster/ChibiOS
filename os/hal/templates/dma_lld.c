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
 * @file    templates/dma_lld.c
 * @brief   Templeate for low level DMA driver code.
 *
 * @addtogroup DMA
 * @{
 */

#include "ch.h"
#include "hal.h"

#if HAL_USE_DMA || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants and error checks.                                        */
/*===========================================================================*/

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

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
}

/**
 * @brief   Activates the DMA peripheral.
 *
 * @param[in] dmad      pointer to the @p dmaDriver object
 *
 * @notapi
 */
void dma_lld_start(dmaDriver *dmad) {
}

/**
 * @brief Deactivates the DMA peripheral.
 *
 * @param[in] dmad      pointer to the @p dmaDriver object
 *
 * @notapi
 */
void dma_lld_stop(dmaDriver *dmad) {
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
}

/**
 * @brief   Activates the DMA channel.
 *
 * @param[in] chan      pointer to the @p dmaChannel object
 *
 * @notapi
 */
void dma_lld_channel_start(dmaChannel *chan) {
}

/**
 * @brief Deactivates the DMA channel.
 *
 * @param[in] chan      pointer to the @p dmaChannel object
 *
 * @notapi
 */
void dma_lld_channel_stop(dmaChannel *chan) {
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
void dma_lld_start_transaction(dmaChannel *chan, dmaTransaction *tr, bool_t poll) {
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
}

#endif /* HAL_USE_DMA */

/** @} */
