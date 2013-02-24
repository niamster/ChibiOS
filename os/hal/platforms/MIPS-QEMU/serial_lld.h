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
 * @file    MIPS-QEMU/serial_lld.h
 * @brief   MIPS_QEMU low level serial driver header.
 *
 * @addtogroup SERIAL
 * @{
 */

#ifndef _SERIAL_LLD_H_
#define _SERIAL_LLD_H_

#if HAL_USE_SERIAL || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/*
 * @brief FIFO Control Register
 */
#define UART_FCR_FIFO_EN    0x01 /* Fifo enable */
#define UART_FCR_CLR_RCVR   0x02 /* Clear the RCVR FIFO */
#define UART_FCR_CLR_XMIT   0x04 /* Clear the XMIT FIFO */

/*
 * @brief the Modem Control Register
 */
#define UART_MCR_DTR        0x01
#define UART_MCR_RTS        0x02
#define UART_MCR_OUT1       0x04
#define UART_MCR_OUT2       0x08

/*
 * @brief Line Control Register
 */
#define UART_LCR_WLS_MSK    0x03 /* Character length select mask */
#define UART_LCR_WLS_5      0x00 /* 5 bit character length */
#define UART_LCR_WLS_6      0x01 /* 6 bit character length */
#define UART_LCR_WLS_7      0x02 /* 7 bit character length */
#define UART_LCR_WLS_8      0x03 /* 8 bit character length */
#define UART_LCR_STB        0x04 /* # stop Bits, off=1, on=1.5 or 2) */
#define UART_LCR_PEN        0x08 /* Parity enable */
#define UART_LCR_EPS        0x10 /* Even Parity Select */
#define UART_LCR_STKP       0x20 /* Stick Parity */
#define UART_LCR_SBRK       0x40 /* Set Break */
#define UART_LCR_BKSE       0x80 /* Bank select enable */
#define UART_LCR_DLAB       0x80 /* Divisor latch access bit */

/*
 * @brief Line Status Register
 */
#define UART_LSR_DR         0x01 /* Data ready */
#define UART_LSR_OE         0x02 /* Overrun */
#define UART_LSR_PE         0x04 /* Parity error */
#define UART_LSR_FE         0x08 /* Framing error */
#define UART_LSR_BI         0x10 /* Break */
#define UART_LSR_THRE       0x20 /* Xmit holding register empty */
#define UART_LSR_TEMT       0x40 /* Xmitter empty */
#define UART_LSR_ERR        0x80 /* Error */

/*
 * @brief Modem Status Register
 */
#define UART_MSR_DCD        0x80 /* Data Carrier Detect */
#define UART_MSR_RI         0x40 /* Ring Indicator */
#define UART_MSR_DSR        0x20 /* Data Set Ready */
#define UART_MSR_CTS        0x10 /* Clear to Send */
#define UART_MSR_DDCD       0x08 /* Delta DCD */
#define UART_MSR_TERI       0x04 /* Trailing edge ring indicator */
#define UART_MSR_DDSR       0x02 /* Delta DSR */
#define UART_MSR_DCTS       0x01 /* Delta CTS */

/*
 * @brief Interrupt Identification Register
 */
#define UART_IIR_NO_INT     0x01 /* No interrupts pending */
#define UART_IIR_ID         0x06 /* Mask for the interrupt ID */
#define UART_IIR_MSI        0x00 /* Modem status interrupt */
#define UART_IIR_THRI       0x02 /* Transmitter holding register empty */
#define UART_IIR_RDI        0x04 /* Receiver data interrupt */
#define UART_IIR_RLSI       0x06 /* Receiver line status interrupt */
#define UART_IIR_CTI        0x0C /* Character Timeout Indication */

/*
 * @brief Interrupt Enable Register
 */
#define UART_IER_MSI        0x08 /* Enable Modem status interrupt */
#define UART_IER_RLSI       0x04 /* Enable receiver line status interrupt */
#define UART_IER_THRI       0x02 /* Enable transmitter holding register int. */
#define UART_IER_RDI        0x01 /* Enable receiver data interrupt */

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @brief   UART driver enable switch.
 * @details If set to @p TRUE the support for UART is included.
 * @note    The default is @p TRUE.
 */
#if !defined(USE_MIPS_QEMU_UART) || defined(__DOXYGEN__)
#define USE_MIPS_QEMU_UART             TRUE
#endif

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Serial Driver register mapping.
 */
typedef struct SerialRegs {
  uint8_t thr;
#define dll thr
  uint8_t ier;
#define dlh ier
  uint8_t fcr;
#define iir fcr
  uint8_t lcr;
  uint8_t mcr;
  uint8_t lsr;
  uint8_t msr;
  uint8_t scr;
} SerialRegs;

/**
 * @brief   Serial Driver configuration structure.
 * @details An instance of this structure must be passed to @p sdStart()
 *          in order to configure and start a serial driver operations.
 */
typedef struct {
  /**
   * @brief   Baud rate.
   */
  uint32_t sc_baud;
} SerialConfig;

/**
 * @brief   @p SerialDriver specific data.
 */
#define _serial_driver_data                           \
  _base_asynchronous_channel_data                     \
  /* Driver state.*/                                  \
  sdstate_t                 state;                    \
  /* Input queue.*/                                   \
  InputQueue                iqueue;                   \
  /* Output queue.*/                                  \
  OutputQueue               oqueue;                   \
  /* Input circular buffer.*/                         \
  uint8_t                   ib[SERIAL_BUFFERS_SIZE];  \
  /* Output circular buffer.*/                        \
  uint8_t                   ob[SERIAL_BUFFERS_SIZE];  \
  /* End of the mandatory fields.*/                   \
                                                      \
  /* Base address of memory mapped registers.*/       \
  void *base;                                         \

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if USE_MIPS_QEMU_UART && !defined(__DOXYGEN__)
extern SerialDriver SD1;
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void sd_lld_init(void);
  void sd_lld_start(SerialDriver *sdp, const SerialConfig *config);
  void sd_lld_stop(SerialDriver *sdp);

  void sd_lld_putc(unsigned char c);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_SERIAL */

#endif /* _SERIAL_LLD_H_ */

/** @} */
