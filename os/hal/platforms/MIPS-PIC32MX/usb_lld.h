/*
    ChibiOS/RT - Copyright (C) 2006,2007,2008,2009,2010,
                 2011,2012 Giovanni Di Sirio.

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
 * @file    PIC32MX/usb_lld.h
 * @brief   PIC32MX USB subsystem low level driver header.
 *
 * @addtogroup USB
 * @{
 */

#ifndef _USB_LLD_H_
#define _USB_LLD_H_

#if HAL_USE_USB || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @brief   Maximum endpoint address.
 */
#ifndef USB_MAX_ENDPOINTS
#define USB_MAX_ENDPOINTS                   15
#endif

/**
 * @brief   Maximum packet size per endpoint.
 * @note    PIC32MX currently supports only USB 1.0 and USB 1.1 for device mode
 */
#define USB_MAX_PACKET_SIZE                 64

/**
 * @brief   The device address must be changed after the status packet.
 */
#define USB_SET_ADDRESS_MODE                USB_LATE_SET_ADDRESS

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
 * @brief   Type of an IN endpoint state structure.
 */
typedef struct {
  /**
   * @brief   Buffer mode, queue or linear.
   */
  bool_t                        txqueued;
  /**
   * @brief   Requested transmit transfer size.
   */
  size_t                        txsize;
  /**
   * @brief   Transmitted bytes so far.
   */
  size_t                        txcnt;
  union {
    struct {
      /**
       * @brief   Pointer to the transmission linear buffer.
       */
      const uint8_t             *txbuf;
    } linear;
    struct {
      /**
       * @brief   Pointer to the output queue.
       */
      OutputQueue               *txqueue;
    } queue;
  } mode;
  /* End of the mandatory fields.*/

  /**
   * @brief   DATA0/1 indicator.
   */
  uint8_t                        data01;
  /**
   * @brief   Ping-Pong buffer indicator.
   */
  uint8_t                        pp;
  /**
   * @brief   Continuous buffer in system RAM suitable for DMA transactions.
   */
  uint8_t                        *dma;
} USBInEndpointState;

/**
 * @brief   Type of an OUT endpoint state structure.
 */
typedef struct {
  /**
   * @brief   Buffer mode, queue or linear.
   */
  bool_t                        rxqueued;
  /**
   * @brief   Requested receive transfer size.
   */
  size_t                        rxsize;
  /**
   * @brief   Received bytes so far.
   */
  size_t                        rxcnt;
  union {
    struct {
      /**
       * @brief   Pointer to the receive linear buffer.
       */
      uint8_t                   *rxbuf;
    } linear;
    struct {
      /**
       * @brief   Pointer to the input queue.
       */
      InputQueue               *rxqueue;
    } queue;
  } mode;
  /* End of the mandatory fields.*/

  /**
   * @brief   DATA0/1 indicator.
   */
  uint8_t                        data01;
  /**
   * @brief   Ping-Pong buffer indicator.
   */
  uint8_t                        pp;
  /**
   * @brief   Continuous buffer in system RAM suitable for DMA transactions.
   */
  uint8_t                        *dma;
} USBOutEndpointState;

/**
 * @brief   Type of an USB endpoint configuration structure.
 * @note    Platform specific restrictions may apply to endpoints.
 */
typedef struct {
  /**
   * @brief   Type and mode of the endpoint.
   */
  uint32_t                      ep_mode;
  /**
   * @brief   Setup packet notification callback.
   * @details This callback is invoked when a setup packet has been
   *          received.
   * @post    The application must immediately call @p usbReadPacket() in
   *          order to access the received packet.
   * @note    This field is only valid for @p USB_EP_MODE_TYPE_CTRL
   *          endpoints, it should be set to @p NULL for other endpoint
   *          types.
   */
  usbepcallback_t               setup_cb;
  /**
   * @brief   IN endpoint notification callback.
   * @details This field must be set to @p NULL if the IN endpoint is not
   *          used.
   */
  usbepcallback_t               in_cb;
  /**
   * @brief   OUT endpoint notification callback.
   * @details This field must be set to @p NULL if the OUT endpoint is not
   *          used.
   */
  usbepcallback_t               out_cb;
  /**
   * @brief   IN endpoint maximum packet size.
   * @details This field must be set to zero if the IN endpoint is not
   *          used.
   */
  uint16_t                      in_maxsize;
  /**
   * @brief   OUT endpoint maximum packet size.
   * @details This field must be set to zero if the OUT endpoint is not
   *          used.
   */
  uint16_t                      out_maxsize;
  /**
   * @brief   @p USBEndpointState associated to the IN endpoint.
   * @details This structure maintains the state of the IN endpoint.
   */
  USBInEndpointState            *in_state;
  /**
   * @brief   @p USBEndpointState associated to the OUT endpoint.
   * @details This structure maintains the state of the OUT endpoint.
   */
  USBOutEndpointState           *out_state;
  /* End of the mandatory fields.*/
} USBEndpointConfig;

/**
 * @brief   Type of an USB driver configuration structure.
 */
typedef struct {
  /**
   * @brief   USB events callback.
   * @details This callback is invoked when an USB driver event is registered.
   */
  usbeventcb_t                  event_cb;
  /**
   * @brief   Device GET_DESCRIPTOR request callback.
   * @note    This callback is mandatory and cannot be set to @p NULL.
   */
  usbgetdescriptor_t            get_descriptor_cb;
  /**
   * @brief   Requests hook callback.
   * @details This hook allows to be notified of standard requests or to
   *          handle non standard requests.
   */
  usbreqhandler_t               requests_hook_cb;
  /**
   * @brief   Start Of Frame callback.
   */
  usbcallback_t                 sof_cb;
  /* End of the mandatory fields.*/

  /**
   * @brief Memory provider for the pool.
  */
  memgetfunc_t                  mp_provider;
} USBConfig;

/**
 * @brief   Structure representing an USB driver.
 */
struct USBDriver {
  /**
   * @brief   Driver state.
   */
  usbstate_t                    state;
  /**
   * @brief   Current configuration data.
   */
  const USBConfig               *config;
  /**
   * @brief   Field available to user, it can be used to associate an
   *          application-defined handler to the USB driver.
   */
  void                          *param;
  /**
   * @brief   Bit map of the transmitting IN endpoints.
   */
  uint16_t                      transmitting;
  /**
   * @brief   Bit map of the receiving OUT endpoints.
   */
  uint16_t                      receiving;
  /**
   * @brief   Active endpoints configurations.
   */
  const USBEndpointConfig       *epc[USB_MAX_ENDPOINTS + 1];
  /**
   * @brief   Endpoint 0 state.
   */
  usbep0state_t                 ep0state;
  /**
   * @brief   Next position in the buffer to be transferred through endpoint 0.
   */
  uint8_t                       *ep0next;
  /**
   * @brief   Number of bytes yet to be transferred through endpoint 0.
   */
  size_t                        ep0n;
  /**
   * @brief   Endpoint 0 end transaction callback.
   */
  usbcallback_t                 ep0endcb;
  /**
   * @brief   Setup packet buffer.
   */
  uint8_t                       setup[8];
  /**
   * @brief   Current USB device status.
   */
  uint16_t                      status;
  /**
   * @brief   Assigned USB address.
   */
  uint8_t                       address;
  /**
   * @brief   Current USB device configuration.
   */
  uint8_t                       configuration;
#if defined(USB_DRIVER_EXT_FIELDS)
  USB_DRIVER_EXT_FIELDS
#endif
  /* End of the mandatory fields.*/
  
  /**
   * @brief USBx interrupt.
   */
  uint8_t                       irq;
  /**
   * @brief USBx port.
   */
  volatile void                 *base;
  /**
   * @brief USBx buffer descriptor table.
   */
  volatile void                 *bdt;
#if CH_USE_MEMPOOLS
  /**
   * @brief Memory pool for transactions which need .
   */
  MemoryPool                    mp;
#else
#error USB module requires memory pools
#endif
};

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @brief   Returns the exact size of a receive transaction.
 * @details The received size can be different from the size specified in
 *          @p usbStartReceiveI() because the last packet could have a size
 *          different from the expected one.
 * @pre     The OUT endpoint must have been configured in transaction mode
 *          in order to use this function.
 *
 * @param[in] usbd      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 * @return              Received data size.
 *
 * @notapi
 */
#define usb_lld_get_transaction_size(usbd, ep)  \
  ((usbd)->epc[ep]->out_state->rxcnt)

/**
 * @brief   Declears buffer descriptor table.
 * @details Table entry has following layout:
 *            EPx OUT even descriptor
 *            EPx OUT odd descriptor
 *            EPx IN even descriptor
 *            EPx IN odd descriptor
 *          Each descriptor is 64-bits long
 *          BDT address has to be 512-byte aligned
 *
 * @notapi
 */
#define DECLEAR_USB_BDT(name) uint8_t name[((USB_MAX_ENDPOINTS + 1) * 4) * 8] __attribute__ ((aligned (512)))

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  void usb_lld_init(void);
  void usb_lld_start(USBDriver *usbd);
  void usb_lld_stop(USBDriver *usbd);
  void usb_lld_reset(USBDriver *usbd);
  void usb_lld_set_address(USBDriver *usbd);
  void usb_lld_init_endpoint(USBDriver *usbd, usbep_t ep);
  void usb_lld_disable_endpoints(USBDriver *usbd);
  usbepstatus_t usb_lld_get_status_in(USBDriver *usbd, usbep_t ep);
  usbepstatus_t usb_lld_get_status_out(USBDriver *usbd, usbep_t ep);
  void usb_lld_read_setup(USBDriver *usbd, usbep_t ep, uint8_t *buf);
  void usb_lld_prepare_receive(USBDriver *usbd, usbep_t ep);
  void usb_lld_prepare_transmit(USBDriver *usbd, usbep_t ep);
  void usb_lld_start_out(USBDriver *usbd, usbep_t ep);
  void usb_lld_start_in(USBDriver *usbd, usbep_t ep);
  void usb_lld_stall_out(USBDriver *usbd, usbep_t ep);
  void usb_lld_stall_in(USBDriver *usbd, usbep_t ep);
  void usb_lld_clear_out(USBDriver *usbd, usbep_t ep);
  void usb_lld_clear_in(USBDriver *usbd, usbep_t ep);
  void usb_lld_connect_bus(USBDriver *usbd);
  void usb_lld_disconnect_bus(USBDriver *usbd);
  uint16_t usb_lld_get_frame_number(USBDriver *usbd);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_USB */

#endif /* _USB_LLD_H_ */

/** @} */
