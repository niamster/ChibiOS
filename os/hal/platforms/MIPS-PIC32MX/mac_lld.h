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
   Concepts and parts of this file have been contributed by Gera Kazakov <gkazakov@msn.com>
*/

/**
 * @file    templates/mac_lld.h
 * @brief   MAC Driver subsystem low level driver header template.
 *
 * @addtogroup MAC
 * @{
 */

#ifndef _MAC_LLD_H_
#define _MAC_LLD_H_

#if HAL_USE_MAC || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @brief   This implementation supports the zero-copy mode API.
 */
#define MAC_SUPPORTS_ZERO_COPY      FALSE

#define MAC_FRAME_SIZE              1522

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    Configuration options
 * @{
 */
/**
 * @brief   MAC driver enable switch.
 * @details If set to @p TRUE the support for MAC1 is included.
 */
#if !defined(PLATFORM_MAC_USE_MAC1) || defined(__DOXYGEN__)
#define PLATFORM_MAC_USE_MAC1             FALSE
#endif
/** @} */

/**
 * @brief   Size of MAC buffers.
 */
#if !defined(MAC_BUFFER_SIZE) || defined(__DOXYGEN__)
#define MAC_BUFFER_SIZE             256
#endif

/**
 * @brief   Number of available transmit descriptors.
 */
#if !defined(MAC_TRANSMIT_DESCRIPTORS) || defined(__DOXYGEN__)
#define MAC_TRANSMIT_DESCRIPTORS    2
#endif

/**
 * @brief   Number of available receive descriptors.
 */
#if !defined(MAC_RECEIVE_DESCRIPTORS) || defined(__DOXYGEN__)
#define MAC_RECEIVE_DESCRIPTORS     4
#endif


/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#define PIC32_MAC_BUF_PER_FRAME        \
  ((MAC_FRAME_SIZE/MAC_BUFFER_SIZE) + \
  ((MAC_FRAME_SIZE%MAC_BUFFER_SIZE) ? 1 : 0))

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Driver configuration structure.
 */
typedef struct {
  /**
   * @brief MAC address.
   */
  uint8_t               *mac_address;
  /* End of the mandatory fields.*/
} MACConfig;

/**
 * @brief   Structure representing a MAC driver.
 */
struct MACDriver {
  /**
   * @brief Driver state.
   */
  macstate_t            state;
  /**
   * @brief Current configuration data.
   */
  const MACConfig       *config;
  /**
   * @brief Transmit semaphore.
   */
  Semaphore             tdsem;
  /**
   * @brief Receive semaphore.
   */
  Semaphore             rdsem;
#if MAC_USE_EVENTS || defined(__DOXYGEN__)
  /**
   * @brief Receive event.
   */
  EventSource           rdevent;
#endif
  /* End of the mandatory fields.*/

  /**
   * @brief ETHx interrupt.
   */
  uint8_t               irq;
  /**
   * @brief ETHx port.
   */
  uint32_t              base;

  uint32_t phyID;           // PHY ID
  uint8_t phy;              // PHY address
  uint8_t mii : 1;          // PHY interface: 1 - MII; 0 - RMII
  uint8_t io : 1;           // IO pins: 1 - default; 0 - alternate
  uint8_t link : 1;         // link status

  // RX descriptors ring pointers
  uint8_t rxStart;          // points to current start of Rx ring
  uint8_t rxEnd;            // points to current end of Rx ring
  void* rxLock;

  // TX descriptors
  uint8_t txNext;           // points to next available TX descriptor
  uint8_t txAvail;          // number of available tx descriptors
  uint8_t txActive;         // number of descriptors in current transmit
  void* txLock;
};

/**
 * @brief   Structure representing a transmit descriptor.
 */
typedef struct {
  /**
   * @brief Current write offset.
   */
  size_t                offset;
  /**
   * @brief Available space size.
   */
  size_t                size;
  /* End of the mandatory fields.*/

  MACDriver* macp;
  uint8_t dma_start;    // first dma descriptor associated with this MAC descriptor
  uint8_t dma_count;    // number of dma descriptors
} MACTransmitDescriptor;

/**
 * @brief   Structure representing a receive descriptor.
 */
typedef struct {
  /**
   * @brief Current read offset.
   */
  size_t                offset;
  /**
   * @brief Available data size.
   */
  size_t                size;
  /* End of the mandatory fields.*/

  MACDriver* macp;
  uint8_t dma_start;  // first dma descriptor associated with this MAC descriptor
  uint8_t dma_count;  // number of dma descriptors
} MACReceiveDescriptor;

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if PLATFORM_MAC_USE_MAC1 && !defined(__DOXYGEN__)
extern MACDriver ETHD1;
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void mac_lld_init(void);
  void mac_lld_start(MACDriver *macp);
  void mac_lld_stop(MACDriver *macp);
  msg_t mac_lld_get_transmit_descriptor(MACDriver *macp,
                                        MACTransmitDescriptor *tdp);
  void mac_lld_release_transmit_descriptor(MACTransmitDescriptor *tdp);
  msg_t mac_lld_get_receive_descriptor(MACDriver *macp,
                                       MACReceiveDescriptor *rdp);
  void mac_lld_release_receive_descriptor(MACReceiveDescriptor *rdp);
  bool_t mac_lld_poll_link_status(MACDriver *macp);
  size_t mac_lld_write_transmit_descriptor(MACTransmitDescriptor *tdp,
                                           uint8_t *buf,
                                           size_t size);
  size_t mac_lld_read_receive_descriptor(MACReceiveDescriptor *rdp,
                                         uint8_t *buf,
                                         size_t size);
#if MAC_USE_ZERO_COPY
  uint8_t *mac_lld_get_next_transmit_buffer(MACTransmitDescriptor *tdp,
                                            size_t size,
                                            size_t *sizep);
  const uint8_t *mac_lld_get_next_receive_buffer(MACReceiveDescriptor *rdp,
                                                 size_t *sizep);
#endif /* MAC_USE_ZERO_COPY */
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_MAC */

#endif /* _MAC_LLD_H_ */

/** @} */
