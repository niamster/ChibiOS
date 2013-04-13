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
 * @file    MIPS-PIC32MX/spi_lld.h
 * @brief   MIPS-PIC32MX low level SPI driver header.
 *
 * @addtogroup SPI
 * @{
 */

#ifndef _SPI_LLD_H_
#define _SPI_LLD_H_

#include "dma.h"

#if HAL_USE_SPI || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @brief   Width of single data transfer.
 */
enum spiDataWidth {
  SPI_DATA_MODE_8_BIT,
  SPI_DATA_MODE_16_BIT,
  SPI_DATA_MODE_32_BIT,
};

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @brief   Defines size of the buffer TX DMA channel
 *          for RX-only SPI transactions and
 *          RX buffer for TX-only transactions
 */
#if !defined(SPI_DUMMY_DMA_BUFFER_SIZE)
#define SPI_DUMMY_DMA_BUFFER_SIZE     512
#endif

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Type of a structure representing an SPI driver.
 */
typedef struct SPIDriver SPIDriver;

/**
 * @brief   SPI notification callback type.
 *
 * @param[in] spid      pointer to the @p SPIDriver object triggering the
 *                      callback
 */
typedef void (*spicallback_t)(SPIDriver *spid);

/**
 * @brief   SPI chip select toggling callback type.
 *
 */
typedef void (*cscallback_t)(bool_t select);

/**
 * @brief   SPI clock polarity and phase configuration.
 *
 */
enum spiClkMode {
  SPI_CLK_MODE0, /* base(idle) value of clock is low, data is sampled on a leading edge */
  SPI_CLK_MODE1, /* base(idle) value of clock is low, data is sampled on a trailing edge */
  SPI_CLK_MODE2, /* base(idle) value of clock is high, data is sampled on a leading edge */
  SPI_CLK_MODE3, /* base(idle) value of clock is high, data is sampled on a trailing edge */
};

/**
 * @brief   Driver configuration structure.
 */
typedef struct {
  /**
   * @brief Operation complete callback or @p NULL.
   */
  spicallback_t         end_cb;
  /* End of the mandatory fields.*/

  struct {
    /**
     * @brief The chip select line port.
     */
    ioportid_t            port;
    /**
     * @brief The chip select line pad number.
     */
    uint16_t              pad;
  } cs;
  /**
   * @brief The width of single transaction.
   */
  enum spiDataWidth     width;
  /**
   * @brief Indicates master-slave relationship.
   */
  bool_t                master;
  /**
   * @brief Clock speed in Hz units.
   */
  uint32_t              clk;
  /**
   * @brief Clock polarity and phase.
   */
  enum spiClkMode       clk_mode;
  /**
   * @brief DMA driver for the transactions.
   */
  dmaDriver             *dmad;
  /**
   * @brief SPI RX interrupt.
   */
  uint8_t               rx_irq;
  /**
   * @brief SPIx port.
   */
  uint32_t              base;
} SPIConfig;

/**
 * @brief Structure representing a SPI driver.
 */
struct SPIDriver {
  /**
   * @brief Driver state.
   */
  spistate_t            state;
  /**
   * @brief Current configuration data.
   */
  const SPIConfig       *config;
#if SPI_USE_WAIT || defined(__DOXYGEN__)
  /**
   * @brief Waiting thread.
   */
  Thread                *thread;
#endif /* SPI_USE_WAIT */
#if SPI_USE_MUTUAL_EXCLUSION || defined(__DOXYGEN__)
#if CH_USE_MUTEXES || defined(__DOXYGEN__)
  /**
   * @brief Mutex protecting the bus.
   */
  Mutex                 mutex;
#elif CH_USE_SEMAPHORES
  Semaphore             semaphore;
#endif
#endif /* SPI_USE_MUTUAL_EXCLUSION */
#if defined(SPI_DRIVER_EXT_FIELDS)
  SPI_DRIVER_EXT_FIELDS
#endif
  /* End of the mandatory fields.*/

  /**
   * @brief SPIx port.
   */
  void                  *base;
  /**
   * @brief SPI RX interrupt.
   */
  uint8_t               rx_irq;
  /**
   * @brief Number of bytes of the transaction.
   */
  uint16_t              cnt;
  /**
   * @brief Receive pointer or @p NULL.
   */
  uint8_t               *rxptr;
  /**
   * @brief Transmit pointer or @p NULL.
   */
  const uint8_t         *txptr;
  /**
   * @brief DMA channel for the TX transactions.
   */
  dmaChannel            txDmaChan;
  /**
   * @brief DMA TX transaction descriptor.
   */
  dmaTransaction        txDmaTransaction;
  /**
   * @brief DMA channel for the RX transactions.
   */
  dmaChannel            rxDmaChan;
  /**
   * @brief DMA RX transaction descriptor.
   */
  dmaTransaction        rxDmaTransaction;
  /**
   * @brief DMA mapped SPI RX/TX port.
   */
  dmaptr_t              dmaPort;
};

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  void spi_lld_init(void);
  void spi_lld_start(SPIDriver *spid);
  void spi_lld_stop(SPIDriver *spid);
  void spi_lld_select(SPIDriver *spid);
  void spi_lld_unselect(SPIDriver *spid);
  void spi_lld_ignore(SPIDriver *spid, size_t n);
  void spi_lld_exchange(SPIDriver *spid, size_t n,
                        const void *txbuf, void *rxbuf);
  void spi_lld_send(SPIDriver *spid, size_t n, const void *txbuf);
  void spi_lld_receive(SPIDriver *spid, size_t n, void *rxbuf);
  uint32_t spi_lld_polled_exchange(SPIDriver *spid, uint32_t frame);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_SPI */

#endif /* _SPI_LLD_H_ */

/** @} */
