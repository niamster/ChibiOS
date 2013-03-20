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
 * @file    MIPS-PIC32MX/eic_lld.h
 * @brief   MIPS-PIC32MX low level eic driver header.
 *
 * @addtogroup EIC
 * @{
 */

#ifndef _EIC_LLD_H_
#define _EIC_LLD_H_

#if HAL_USE_EIC || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/* Well known IRQ numbers */
#define EIC_IRQ_CT          _CORE_TIMER_IRQ
#define EIC_IRQ_UART1_RX    _UART1_RX_IRQ
#define EIC_IRQ_SPI1_RX     _SPI1_RX_IRQ
#define EIC_IRQ_SPI2_RX     _SPI2_RX_IRQ
#define EIC_IRQ_SPI3_RX     _SPI3_RX_IRQ
#define EIC_IRQ_SPI4_RX     _SPI4_RX_IRQ
#define EIC_IRQ_USB         _USB_IRQ
#define EIC_IRQ_EXT         _CHANGE_NOTICE_IRQ
#define EIC_IRQ_DMA0        _DMA0_IRQ
#define EIC_IRQ_DMA1        _DMA1_IRQ
#define EIC_IRQ_DMA2        _DMA2_IRQ
#define EIC_IRQ_DMA3        _DMA3_IRQ
#define EIC_IRQ_DMA4        _DMA4_IRQ
#define EIC_IRQ_DMA5        _DMA5_IRQ
#define EIC_IRQ_DMA6        _DMA6_IRQ
#define EIC_IRQ_DMA7        _DMA7_IRQ

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  void eic_lld_init(void);
  void eic_lld_register_irq(int irq, eicIrqHandler handler, void *data);
  void eic_lld_unregister_irq(int irq);
  void eic_lld_enable_irq(int irq);
  void eic_lld_disable_irq(int irq);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_EIC */

#endif  /* _EIC_LLD_H_ */

/** @} */
