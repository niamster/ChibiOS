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
 * @file    MIPS-QEMU/eic_lld.h
 * @brief   MIPS_QEMU low level eic driver header.
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

/* Max number of IRQs */
#define EIC_NUM_IRQS    8

/* Well known IRQ numbers */
#define EIC_IRQ_UART    4

/* i8259A EIC registers */
#define EIC_MASTER_CMD    0x20
#define EIC_MASTER_IMR    0x21
#define EIC_MASTER_ISR    EIC_MASTER_CMD

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * Let the port hadle MIPS core timer
 */
#define MIPS_PORT_HANDLE_CORE_TIMER

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   EIC driver data.
 */
typedef struct EicDriver {
  /** @brief Current cached IRQ mask.*/
  uint8_t mask;
  /** @brief EIC io-memory base.*/
  void *base;
} EicDriver;

/**
 * @brief   EIC IRQ handler data.
 */
typedef struct EicIrqInfo {
  /** @brief IRQ handle.*/
  eicIrqHandler handler;
  /** @brief IRQ handler data.*/
  void *data;
} EicIrqInfo;

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
