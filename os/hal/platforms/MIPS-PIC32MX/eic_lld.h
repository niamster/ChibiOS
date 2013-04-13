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
#define EIC_IRQ_RTC         _RTCC_IRQ
#define EIC_IRQ_ADC         _ADC_IRQ
#define EIC_IRQ_TMR1        _TIMER_1_IRQ
#define EIC_IRQ_TMR2        _TIMER_2_IRQ
#define EIC_IRQ_TMR3        _TIMER_3_IRQ
#define EIC_IRQ_TMR4        _TIMER_4_IRQ
#define EIC_IRQ_TMR5        _TIMER_5_IRQ

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
  void eic_lld_ack_irq(int irq);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_EIC */

#endif  /* _EIC_LLD_H_ */

/** @} */
