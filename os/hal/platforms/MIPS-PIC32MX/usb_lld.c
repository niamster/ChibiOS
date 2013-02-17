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
 * @file    PIC32MX/usb_lld.c
 * @brief   PIC32MX USB subsystem low level driver source.
 *
 * @addtogroup USB
 * @{
 */

#include <string.h>

#include "ch.h"
#include "hal.h"

#if HAL_USE_USB || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Type of a buffer descriptor entry.
 */
typedef volatile struct PACK_STRUCT_STRUCT {
  uint8_t   ctrl;
  uint8_t   pad;
  /* The number of bytes to be transmitted or
     the maximum number of bytes to be received during a transfer. */
  uint16_t  cnt;
  /* Address of the endpoint packet data buffer. */
  uint32_t  addr;
} bdtEntry;

/**
 * @brief   Configuration bits of ctrl field in BDT entry.
 */
enum bdeCtrlBits {
  BDE_CTRL_STALL = 2, // Buffer stall enable
  BDE_CTRL_DTS   = 3, // Data toggle synchronisation enable
  BDE_CTRL_DTP   = 6, // Data(0/1) toggle packet
  BDE_CTRL_UOW   = 7, // USB ownership
};

/**
 * @brief   Type of buffer descriptor entries per EP.
 */
typedef volatile struct {
  bdtEntry out[2]; // Even[0] and odd[1] OUT descriptors
  bdtEntry in[2];  // Even[0] and odd[1] IN descriptors
} bdtEpEntry;

/**
 * @brief   Type of USB registers layout.
 */
typedef volatile struct {
  PicReg otgir;
  PicReg otgie;
  PicReg otgstat;
  PicReg otgcon;

  PicReg pwrc;

  uint8_t pad0[0x170];

  PicReg ir;
  PicReg ie;
  PicReg eir;
  PicReg eie;

  PicReg stat;
  PicReg con;

  PicReg addr;

  PicReg bdtp1;

  PicReg frml;
  PicReg frmh;

  PicReg tok;
  PicReg sof;

  PicReg bdtp2;
  PicReg bdtp3;

  PicReg cnfg1;

  uint8_t pad1[0x10];

  PicReg ep[USB_MAX_ENDPOINTS + 1];
} usbRegs;

/**
 * @brief   Configuration bits of otgcon register.
 */
enum regOtgconBits {
  REG_OTGCON_OTGEN   = 2, // OTG functionality enable
                          // If set, DPPULUP, DMPULUP, DPPULDWN, and DMPULDWN bits are under software control,
                          // otherwise DPPULUP, DMPULUP, DPPULDWN, and DMPULDWN bits are under USB hardware control
};

/**
 * @brief   Configuration bits of ie and ir registers.
 */
enum usbInterruptBits {
  USB_INTERRUPT_RST    = 0, // Reset interrupt enable (Device Mode)
  USB_INTERRUPT_ERR    = 1, // Error interrupt enable
  USB_INTERRUPT_SOF    = 2, // SOF interrupt enable
  USB_INTERRUPT_TRN    = 3, // Token processing complete interrupt enable
  USB_INTERRUPT_IDLE   = 4, // Idle interrupt enable
  USB_INTERRUPT_RESUME = 5, // Resume interrupt enable
  USB_INTERRUPT_ATTACH = 6, // Attach interrupt enable
  USB_INTERRUPT_STALL  = 7, // Stall interrupt enable
};

/**
 * @brief   Configuration bits of pwrc register.
 */
enum regPwrcBits {
  REG_PWRC_PWR      = 0, // USB operation enable
  REG_PWRC_SUSPEND  = 1, // USB suspend mode
};

/**
 * @brief   Configuration bits of con register.
 */
enum regConBits {
  REG_CON_USBEN   = 0, // USB module and supporting circuitry enable (Device Mode)
  REG_CON_PPBRST  = 1, // Ping-Pong buffers reset
  REG_CON_PKTDIS  = 5, // Packet transfer disable (Device Mode)
};

/**
 * @brief   Configuration bits of ep register.
 */
enum regEpBits {
  REG_EP_HSHK   = 0, // EP handshake enable
  REG_EP_STALL  = 1, // EP stall status
  REG_EP_TXEN   = 2, // Transmit enable
  REG_EP_RXEN   = 3, // Receive enable
  REG_EP_CONDIS = 4, // If (TXEN=1 and RXEN=1) and CONDIS=0, enable EP for Control (SETUP) transfersm TX and RX transfers are also allowed
};

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
 * @brief   Sets BDT address to the HW.
 *
 * @param[in] regs      pointer to USB module registers
 * @param[in] bdt       address of BDT
 */
static void usb_set_bdt(usbRegs *regs, uint32_t bdt) {
  uint32_t p = MIPS_PHYSICAL(bdt);

  regs->bdtp3.reg = (p >> 24) & 0xFF;
  regs->bdtp2.reg = (p >> 16) & 0xFF;
  regs->bdtp1.reg = (p >>  8) & 0xFE;
}

/**
 * @brief   Powers on or off the module.
 *
 * @param[in] regs      pointer to USB module registers
 * @param[in] on        enable the module if TRUE, disable otherwise
 */
static void usb_power_on(usbRegs *regs, bool_t on) {
  if (on)
    regs->pwrc.set = (1 << REG_PWRC_PWR) | (1 << REG_PWRC_SUSPEND);
  else
    regs->pwrc.clear = (1 << REG_PWRC_PWR) | (1 << REG_PWRC_SUSPEND);
}

/**
 * @brief   Enables or disables the module.
 *
 * @param[in] regs      pointer to USB module registers
 * @param[in] enable    enable the module if TRUE, disable otherwise
 */
static void usb_enable(usbRegs *regs, bool_t enable) {
  if (enable)
    regs->con.set = 1 << REG_CON_USBEN;
  else
    regs->con.clear = 1 << REG_CON_USBEN;
}

/**
 * @brief   Enables or disables the interrupt sources.
 *
 * @param[in] regs      pointer to USB module registers
 * @param[in] mask      mask of interrupt sources to enable or disable
 * @param[in] enable    enable the interrupt sources if TRUE, disable otherwise
 */
static void usb_enable_interrupts(usbRegs *regs, uint8_t mask, bool_t enable) {
  if (enable)
    regs->ie.set = mask;
  else
    regs->ie.clear = mask;
}

/**
 * @brief   Enables or disables the interrupt sources.
 *
 * @param[in] regs      pointer to USB module registers
 * @param[in] mask      mask of interrupt sources to enable or disable
 * @param[in] enable    enable the interrupt sources if TRUE, disable otherwise
 */
static void usb_disable_all_interrupts(usbRegs *regs) {
  usb_enable_interrupts(regs, 0xFF, FALSE);
}

/**
 * @brief   Acknowledges the interrupt source.
 *
 * @param[in] regs      pointer to USB module registers
 * @param[in] irq       IRQ number to acknowledge
 */
static void usb_ack_interrupt(usbRegs *regs, uint8_t irq) {
  regs->ir.reg = 1 << irq;
}

/**
 * @brief   Returns mask of enabled interrupt sources.
 *
 * @param[in] regs      pointer to USB module registers
 * @return              mask of enabled interrupt sources
 */
static uint8_t usb_enabled_interrupts_mask(usbRegs *regs) {
  return regs->ie.reg;
}

/**
 * @brief   Returns mask of active interrupt sources.
 *
 * @param[in] regs      pointer to USB module registers
 * @return              mask of active interrupt sources
 */
static uint8_t usb_pending_interrupts(usbRegs *regs) {
  return regs->ir.reg;
}

/**
 * @brief   Retrieves details of processed token.
 *
 * @param[in] regs      pointer to USB module registers
 * @param[out] ep       EP number
 * @param[out] in       EP direction, TRUE if IN
 * @param[out] pp       Ping-Pong BD pointer
 */
static void usb_processed_token(usbRegs *regs,
    usbep_t *ep, bool_t *in, uint8_t *pp) {
  uint8_t stat = regs->stat.reg;

  stat >>= 2;
  *pp = stat&1;

  stat >>= 1;
  *in = stat&1;

  stat >>= 1;
  *ep = stat&0xF;
}

/**
 * @brief   Prepares CTRL EP for STATUS phase.
 *
 * @param[in] usbd      pointer to the @p USBDriver object
 */
static void usb_prepare_for_status(USBDriver *usbd) {
  const usbep_t ep = 0;
  bdtEpEntry *bdt = usbd->bdt;
  const USBEndpointConfig *epc = usbd->epc[ep];
  USBOutEndpointState *oes = epc->out_state;
  uint8_t pp = oes->pp;
  bdtEntry *bde = &bdt[0].out[pp];

  bde->addr = MIPS_PHYSICAL((uint32_t)&usbd->setup[0]);
  bde->cnt = sizeof(usbd->setup);
  bde->ctrl = (1 << BDE_CTRL_DTS) | (1 << BDE_CTRL_STALL) | (1 << BDE_CTRL_UOW);

  epc->out_state->data01 = 1;
  epc->in_state->data01 = 1;

  oes->pp ^= 1;
}

/**
 * @brief   Prepares EP for the transaction.
 *
 * @param[in] usbd      pointer to the @p USBDriver object
 * @param[in] bde       buffer descriptor
 * @param[in] data      buffer for the transaction
 * @param[in] n         transaction size
 * @param[in] data01    DATA0/1 indicator
 */
static void usb_prepare_transaction(USBDriver *usbd, bdtEntry *bde,
    const uint8_t *data, size_t n, uint8_t data01) {
  (void)usbd;

  bde->addr = MIPS_PHYSICAL((uint32_t)data);
  bde->cnt = n;
  bde->ctrl = (1 << BDE_CTRL_DTS) | (data01 << BDE_CTRL_DTP);
}

/**
 * @brief   Starts transaction on given EP.
 *
 * @param[in] usbd      pointer to the @p USBDriver object
 * @param[in] bde       buffer descriptor
 */
static void usb_start_transaction(USBDriver *usbd, bdtEntry *bde) {
  (void)usbd;

  bde->ctrl |= (1 << BDE_CTRL_UOW);
}

/**
 * @brief   Checks if the given address is suitable for USB DMA.
 *
 * @param[in] addr      address to check
 * @return              TRUE if given address is suitable for USB DMA,
 *                      FALSE otherwise
 */
static inline bool_t usb_is_suitable_for_dma(uint32_t addr) {
  return MIPS_PHYSICAL(addr) < 32*1024*1024;
}

/**
 * @brief   Prepares for a transmit operation.
 *
 * @param[in] usbd      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
static void __usb_lld_prepare_transmit(USBDriver *usbd, usbep_t ep) {
  bdtEpEntry *bdt = usbd->bdt;
  const USBEndpointConfig *epc = usbd->epc[ep];
  USBInEndpointState *ies = epc->in_state;
  uint8_t pp = ies->pp;
  bdtEntry *bde = &bdt[ep].in[pp];
  size_t n;

  n = ies->txsize;
  if (n > (size_t)epc->in_maxsize)
    n = (size_t)epc->in_maxsize;

  if (ies->txqueued) {
    uint8_t *b;
    size_t i;

    chDbgAssert(!ies->dma, "DMA IN transaction for given EP is not finished", "");

    b = chPoolAlloc(&usbd->mp);
    chDbgAssert(b, "Not enough memory for USB IN transaction", "");

    for (i=0;i<n;++i)
      b[i] = chOQGetI(ies->mode.queue.txqueue);

    ies->dma = b;
    usb_prepare_transaction(usbd, bde, b, n, ies->data01);
  } else {
    if (usb_is_suitable_for_dma((uint32_t)ies->mode.linear.txbuf))
      usb_prepare_transaction(usbd, bde, ies->mode.linear.txbuf, n, ies->data01);
    else {
      uint8_t *b;
      size_t i;

      chDbgAssert(!ies->dma, "DMA IN transaction for given EP is not finished", "");

      b = chPoolAlloc(&usbd->mp);
      chDbgAssert(b, "Not enough memory for USB IN transaction", "");

      for (i=0;i<n;++i)
        b[i] = ies->mode.linear.txbuf[i];

      ies->dma = b;
      usb_prepare_transaction(usbd, bde, b, n, ies->data01);
    }
  }
}

/**
 * @brief   Prepares for a receive operation.
 *
 * @param[in] usbd      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
static void __usb_lld_prepare_receive(USBDriver *usbd, usbep_t ep) {
  bdtEpEntry *bdt = usbd->bdt;
  const USBEndpointConfig *epc = usbd->epc[ep];
  USBOutEndpointState *oes = epc->out_state;
  uint8_t pp = oes->pp;
  bdtEntry *bde = &bdt[ep].out[pp];
  size_t n;

  n = oes->rxsize;
  if (n > (size_t)epc->out_maxsize)
    n = (size_t)epc->out_maxsize;

  if (oes->rxqueued) {
    uint8_t *b;

    chDbgAssert(!oes->dma, "DMA OUT transaction for given EP is not finished", "");

    b = chPoolAlloc(&usbd->mp);
    chDbgAssert(b, "Not enough memory for USB OUT transaction", "");

    oes->dma = b;
    usb_prepare_transaction(usbd, bde, b, n, oes->data01);
  } else {
    if (!usb_is_suitable_for_dma((uint32_t)oes->mode.linear.rxbuf))
      chDbgPanic("Wrong linear buffer address for OUT transaction");
    usb_prepare_transaction(usbd, bde, oes->mode.linear.rxbuf, n, oes->data01);
  }
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/**
 * @brief   USB IRQ handler.
 *
 * @param[in] usbd       Driver associated to the USB
 */
static void lld_serve_interrupt(void *data) {
  USBDriver *usbd = data;
  usbRegs *regs = usbd->base;
  uint8_t msk = usb_enabled_interrupts_mask(regs);
  uint8_t ir;
  bdtEpEntry *bdt = usbd->bdt;

  chSysLockFromIsr();

  while ((ir = (usb_pending_interrupts(regs)&msk))) {
    if (ir&(1 << USB_INTERRUPT_SOF)) {
      usb_ack_interrupt(regs, USB_INTERRUPT_SOF);
      _usb_isr_invoke_sof_cb(usbd);
    }

    if (ir&(1 << USB_INTERRUPT_STALL)) {
      usb_ack_interrupt(regs, USB_INTERRUPT_STALL);
      regs->ep[0].clear = 1 << REG_EP_STALL;
      usb_prepare_for_status(usbd);
    }

    if (ir&(1 << USB_INTERRUPT_TRN)) {
      const USBEndpointConfig *epc;
      bdtEntry *bde;
      usbep_t ep;
      bool_t in;
      uint8_t pp;
      
      usb_processed_token(regs, &ep, &in, &pp);
      usb_ack_interrupt(regs, USB_INTERRUPT_TRN);

      if (in)
        bde = &bdt[ep].in[pp];
      else
        bde = &bdt[ep].out[pp];
      epc = usbd->epc[ep];

      if (0 == ep
          && bde->addr == MIPS_PHYSICAL((uint32_t)&usbd->setup[0])) {
          regs->con.clear = 1 << REG_CON_PKTDIS; // Enable packet processing
        _usb_isr_invoke_setup_cb(usbd, ep);
      } else {
        const USBEndpointConfig *epc = usbd->epc[ep];
        size_t cnt = bde->cnt;

        if (in) {
          USBInEndpointState *ies = epc->in_state;

          ies->txcnt  += cnt;
          ies->txsize -= cnt;

          if (ies->dma) {
            chPoolFree(&usbd->mp, ies->dma);
            ies->dma = NULL;
          }

          if (ies->txsize > 0) {
            if (!ies->txqueued)
              ies->mode.linear.txbuf += cnt;
            __usb_lld_prepare_transmit(usbd, ep);
            usb_lld_start_in(usbd, ep);
          } else {
            _usb_isr_invoke_in_cb(usbd, ep);
            if (0 == ep && 0 == cnt
                    && USB_EP0_WAITING_SETUP == usbd->ep0state)
              usb_prepare_for_status(usbd);
          }
        } else {
          USBOutEndpointState *oes = epc->out_state;

          oes->rxcnt  += cnt;
          oes->rxsize -= cnt;

          if (oes->dma) {
            if (oes->rxqueued) {
              size_t i;

              for (i=0;i<cnt;++i)
                chIQPutI(oes->mode.queue.rxqueue, oes->dma[i]);
            }

            chPoolFree(&usbd->mp, oes->dma);
            oes->dma = NULL;
          }

          if (oes->rxsize > 0) {
            if (!oes->rxqueued)
              oes->mode.linear.rxbuf += cnt;
            __usb_lld_prepare_receive(usbd, ep);
            usb_lld_start_out(usbd, ep);
          } else {
            _usb_isr_invoke_out_cb(usbd, ep);
            if (0 == ep && 0 == cnt
                    && USB_EP0_WAITING_SETUP == usbd->ep0state)
              usb_prepare_for_status(usbd);
          }
        }
      }
    }

    if (ir&(1 << USB_INTERRUPT_RST)) {
      usb_ack_interrupt(regs, USB_INTERRUPT_RST);
      _usb_reset(usbd);
      _usb_isr_invoke_event_cb(usbd, USB_EVENT_RESET);
    }

    if (ir&(1 << USB_INTERRUPT_ERR)) {
      usb_ack_interrupt(regs, USB_INTERRUPT_ERR);
      chDbgPanic("USB error");
    }
  }

  chSysUnlockFromIsr();
}

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level USB driver initialization.
 *
 * @notapi
 */
void usb_lld_init(void) {
}

/**
 * @brief   Configures and activates the USB peripheral.
 *
 * @param[in] usbd      pointer to the @p USBDriver object
 *
 * @notapi
 */
void usb_lld_start(USBDriver *usbd) {
  if (usbd->state == USB_STOP) {
    usbRegs *regs = usbd->base;
    bdtEntry *bdt = usbd->bdt;

    chDbgAssert(regs, "USB port address is not defined", "");
    chDbgAssert(bdt, "USB BDT address is not defined", "");

    chDbgAssert(usbd->config->mp_provider, "Memory provider for the pool is not defined", "");

    chPoolInit(&usbd->mp, USB_MAX_PACKET_SIZE, usbd->config->mp_provider);

    usb_lld_disable_endpoints(usbd);

    /* While OTGCON[REG_OTGCON_OTGEN] == 0,
       the HW controls pull-up resistors
     */

    usb_power_on(regs, TRUE);

    usb_set_bdt(regs, (uint32_t)bdt);

#if HAL_USE_EIC
    eicRegisterIrq(usbd->irq, lld_serve_interrupt, usbd);
    eicEnableIrq(usbd->irq);
#endif

    regs->eie.reg = 0xFF; // Enable all error interrupts

    usb_enable_interrupts(regs,
        (1 << USB_INTERRUPT_RST) | (1 << USB_INTERRUPT_TRN)
        | (1 << USB_INTERRUPT_ERR) | (1 << USB_INTERRUPT_STALL),
        TRUE);
  }
}

/**
 * @brief   Deactivates the USB peripheral.
 *
 * @param[in] usbd      pointer to the @p USBDriver object
 *
 * @notapi
 */
void usb_lld_stop(USBDriver *usbd) {
  /* If in ready state then disables the USB module.*/
  if (usbd->state == USB_STOP) {
    usbRegs *regs = usbd->base;

    usb_disable_all_interrupts(regs);

#if HAL_USE_EIC
    eicDisableIrq(usbd->irq);
    eicUnregisterIrq(usbd->irq);
#endif

    usb_power_on(regs, FALSE);
  }
}

/**
 * @brief   USB low level reset routine.
 *
 * @param[in] usbd      pointer to the @p USBDriver object
 *
 * @notapi
 */
void usb_lld_reset(USBDriver *usbd) {
  usbRegs *regs = usbd->base;

  usb_lld_disable_endpoints(usbd);

  regs->addr.reg = 0x00; // Reset to default address
  regs->con.clear = 1 << REG_CON_PKTDIS; // Enable packet processing

  /* The SOF interrupt is only enabled if a callback is defined for
     this service because it is an high rate source.*/
  if (usbd->config->sof_cb != NULL)
    usb_enable_interrupts(regs, (1 << USB_INTERRUPT_SOF), TRUE);
}

/**
 * @brief   Connects the USB device.
 *
 * @api
 */
void usb_lld_connect_bus(USBDriver *usbd) {
  usbRegs *regs = usbd->base;

  usb_enable(regs, TRUE);
}

/**
 * @brief   Disconnect the USB device.
 *
 * @api
 */

void usb_lld_disconnect_bus(USBDriver *usbd) {
  usbRegs *regs = usbd->base;

  usb_enable(regs, FALSE);
}

/**
 * @brief   Returns the current frame number.
 *
 * @param[in] usbd      pointer to the @p USBDriver object
 * @return              The current frame number.
 *
 * @notapi
 */
uint16_t usb_lld_get_frame_number(USBDriver *usbd) {
  usbRegs *regs = usbd->base;

  return (regs->frml.reg) | (regs->frmh.reg << 8);
}

/**
 * @brief   Sets the USB address.
 *
 * @param[in] usbd      pointer to the @p USBDriver object
 *
 * @notapi
 */
void usb_lld_set_address(USBDriver *usbd) {
  usbRegs *regs = usbd->base;

  regs->addr.reg = usbd->address & 0x7F;
}

/**
 * @brief   Enables an endpoint.
 *
 * @param[in] usbd      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
void usb_lld_init_endpoint(USBDriver *usbd, usbep_t ep) {
  usbRegs *regs = usbd->base;
  const USBEndpointConfig *epc = usbd->epc[ep];
  uint32_t reg = 1 << REG_EP_HSHK;

  /* Setting the endpoint type.*/
  switch (epc->ep_mode & USB_EP_MODE_TYPE) {
    case USB_EP_MODE_TYPE_ISOC:
    case USB_EP_MODE_TYPE_BULK:
    case USB_EP_MODE_TYPE_INTR:
      reg |= 1 << REG_EP_CONDIS;
      break;
    case USB_EP_MODE_TYPE_CTRL:
      chDbgAssert(0 == ep, "Wrong CTRL EP number", "");
      break;
  }

  chDbgAssert(epc->in_maxsize <= USB_MAX_PACKET_SIZE
      && epc->out_maxsize <= USB_MAX_PACKET_SIZE, "Wrong max packet size", "");

  if (epc->in_cb)
    reg |= 1 << REG_EP_TXEN;
  if (epc->out_cb)
    reg |= 1 << REG_EP_RXEN;

  regs->ep[ep].reg = reg;

  if (0 == ep && USB_EP0_WAITING_SETUP == usbd->ep0state)
    usb_prepare_for_status(usbd);
}

/**
 * @brief   Disables all the active endpoints except the endpoint zero.
 *
 * @param[in] usbd      pointer to the @p USBDriver object
 *
 * @notapi
 */
void usb_lld_disable_endpoints(USBDriver *usbd) {
  usbRegs *regs = usbd->base;
  unsigned i;

  regs->con.set = 1 << REG_CON_PPBRST; // Put Ping-Pong buffers into reset

  for (i = 1; i < USB_MAX_ENDPOINTS+1; i++)
    regs->ep[i].reg = 0;

  regs->con.clear = 1 << REG_CON_PPBRST; // Pull Ping-Pong buffers from the reset
}

/**
 * @brief   Returns the status of an OUT endpoint.
 *
 * @param[in] usbd      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 * @return              The endpoint status.
 * @retval EP_STATUS_DISABLED The endpoint is not active.
 * @retval EP_STATUS_STALLED  The endpoint is stalled.
 * @retval EP_STATUS_ACTIVE   The endpoint is active.
 *
 * @notapi
 */
usbepstatus_t usb_lld_get_status_out(USBDriver *usbd, usbep_t ep) {
  usbRegs *regs = usbd->base;
  uint32_t reg = regs->ep[ep].reg;

  if (!reg)
    return EP_STATUS_DISABLED;

  if (reg&(1 << REG_EP_STALL))
    return EP_STATUS_STALLED;

  return EP_STATUS_ACTIVE;
}

/**
 * @brief   Returns the status of an IN endpoint.
 *
 * @param[in] usbd      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 * @return              The endpoint status.
 * @retval EP_STATUS_DISABLED The endpoint is not active.
 * @retval EP_STATUS_STALLED  The endpoint is stalled.
 * @retval EP_STATUS_ACTIVE   The endpoint is active.
 *
 * @notapi
 */
usbepstatus_t usb_lld_get_status_in(USBDriver *usbd, usbep_t ep) {
  usbRegs *regs = usbd->base;
  uint32_t reg = regs->ep[ep].reg;

  if (!reg)
    return EP_STATUS_DISABLED;

  if (reg&(1 << REG_EP_STALL))
    return EP_STATUS_STALLED;

  return EP_STATUS_ACTIVE;
}

/**
 * @brief   Reads a setup packet from the dedicated packet buffer.
 * @details This function must be invoked in the context of the @p setup_cb
 *          callback in order to read the received setup packet.
 * @pre     In order to use this function the endpoint must have been
 *          initialized as a control endpoint.
 * @post    The endpoint is ready to accept another packet.
 *
 * @param[in] usbd      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 * @param[out] buf      buffer where to copy the packet data
 *
 * @notapi
 */
void usb_lld_read_setup(USBDriver *usbd, usbep_t ep, uint8_t *buf) {
  chDbgAssert(0 == ep,
      "Wrong EP number for SETUP", "");
  chDbgAssert(buf == usbd->setup,
      "Not a 0-copy SETUP transaction", "");
}

/**
 * @brief   Prepares for a receive operation.
 *
 * @param[in] usbd      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
void usb_lld_prepare_receive(USBDriver *usbd, usbep_t ep) {
  bdtEpEntry *bdt = usbd->bdt;
  const USBEndpointConfig *epc = usbd->epc[ep];
  USBOutEndpointState *oes = epc->out_state;
  uint8_t pp = oes->pp;
  bdtEntry *bde = &bdt[ep].out[pp];


  if (0 == ep &&
      0 == oes->rxsize)
    usb_prepare_transaction(usbd, bde, 0, 0, 1);
  else
    __usb_lld_prepare_receive(usbd, ep);
}

/**
 * @brief   Starts a receive operation on an OUT endpoint.
 *
 * @param[in] usbd      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
void usb_lld_start_out(USBDriver *usbd, usbep_t ep) {
  bdtEpEntry *bdt = usbd->bdt;
  const USBEndpointConfig *epc = usbd->epc[ep];
  USBOutEndpointState *oes = epc->out_state;
  uint8_t pp = oes->pp;
  bdtEntry *bde = &bdt[ep].out[pp];

  usb_start_transaction(usbd, bde);

  oes->pp ^= 1;
  oes->data01 ^= 1;
}

/**
 * @brief   Prepares for a transmit operation.
 *
 * @param[in] usbd      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
void usb_lld_prepare_transmit(USBDriver *usbd, usbep_t ep) {
  bdtEpEntry *bdt = usbd->bdt;
  const USBEndpointConfig *epc = usbd->epc[ep];
  USBInEndpointState *ies = epc->in_state;
  uint8_t pp = ies->pp;
  bdtEntry *bde = &bdt[ep].in[pp];

  if (0 == ep && 0 == ies->txsize)
    usb_prepare_transaction(usbd, bde, 0, 0, 1);
  else
    __usb_lld_prepare_transmit(usbd, ep);
}

/**
 * @brief   Starts a transmit operation on an IN endpoint.
 *
 * @param[in] usbd      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
void usb_lld_start_in(USBDriver *usbd, usbep_t ep) {
  bdtEpEntry *bdt = usbd->bdt;
  const USBEndpointConfig *epc = usbd->epc[ep];
  USBInEndpointState *ies = epc->in_state;
  uint8_t pp = ies->pp;
  bdtEntry *bde = &bdt[ep].in[pp];

  usb_start_transaction(usbd, bde);

  ies->pp ^= 1;
  ies->data01 ^= 1;
}

/**
 * @brief   Brings an OUT endpoint in the stalled state.
 *
 * @param[in] usbd      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
void usb_lld_stall_out(USBDriver *usbd, usbep_t ep) {
  (void)usbd;
  (void)ep;
}

/**
 * @brief   Brings an IN endpoint in the stalled state.
 *
 * @param[in] usbd      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
void usb_lld_stall_in(USBDriver *usbd, usbep_t ep) {
  if (0 == ep) {
    usbRegs *regs = usbd->base;
    regs->ep[0].set = 1 << REG_EP_STALL;
  }
}

/**
 * @brief   Brings an OUT endpoint in the active state.
 *
 * @param[in] usbd      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
void usb_lld_clear_out(USBDriver *usbd, usbep_t ep) {
  (void)usbd;
  (void)ep;
}

/**
 * @brief   Brings an IN endpoint in the active state.
 *
 * @param[in] usbd      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
void usb_lld_clear_in(USBDriver *usbd, usbep_t ep) {
  if (0 == ep) {
    usbRegs *regs = usbd->base;
    regs->ep[0].clear = 1 << REG_EP_STALL;
  }
}

#endif /* HAL_USE_USB */

/** @} */
