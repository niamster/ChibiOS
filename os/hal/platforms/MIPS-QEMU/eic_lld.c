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
 * @file    MIPS-QEMU/eic_lld.c
 * @brief   MIPS_QEMU low level EIC driver code.
 *
 * @addtogroup EIC
 * @{
 */

#include "ch.h"
#include "hal.h"

#if HAL_USE_EIC || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables.                                                   */
/*===========================================================================*/

static EicDriver drv = {
  .mask = 0xff,
  .base = (void *)MIPS_UNCACHED(HAL_MIPS_QEMU_REGS_BASE),
};

static EicIrqInfo isr[EIC_NUM_IRQS];

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

static inline void eic_set_imr(uint8_t value) {
  volatile uint8_t *imr = (uint8_t *)drv.base + EIC_MASTER_IMR;

  *imr = value;
}

static inline uint8_t eic_get_imr(void) {
  volatile uint8_t *imr = (uint8_t *)drv.base + EIC_MASTER_IMR;

  return *imr;
}

static inline void eic_set_cmd(uint8_t value) {
  volatile uint8_t *imr = (uint8_t *)drv.base + EIC_MASTER_CMD;

  *imr = value;
}

static inline uint8_t eic_get_isr(void) {
  volatile uint8_t *isr = (uint8_t *)drv.base + EIC_MASTER_ISR;

  return *isr;
}

static inline void eic_set_mask(void) {
  eic_set_imr(drv.mask);
}

static inline uint8_t eic_pending(void) {
  return eic_get_isr() & ~drv.mask;
}

static void eic_ack(uint8_t irq) {
  volatile uint8_t dummy;
  eic_set_cmd(0x0C);        /* Switch to poll mode to ack the irq by reading ISR */
  dummy = eic_get_isr();
  (void)dummy;
  eic_set_cmd(0x60 + irq);  /* EOI to master */
}

/* Main EIC ISR */
CH_IRQ_HANDLER(MIPS_HW_IRQ2) {
  CH_IRQ_PROLOGUE();

  uint32_t pending = eic_pending();

  while (pending) {
    uint32_t i = 31 - __builtin_clz(pending);
    EicIrqInfo *info = &isr[i];

    eic_ack(i);

    if (!info->handler)
      chDbgPanic("unhandled EIC irq");

    info->handler(i, info->data);

    pending &= ~(1 << i);
  }

  CH_IRQ_EPILOGUE();
}

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   EIC LLD initialization.
 * @note    This function is implicitly invoked by @p eicInit(), there is
 *          no need to explicitly initialize the driver.
 *
 * @init
 */
void eic_lld_init(void) {
  eic_set_cmd(0x11); /* ICW1: select 8259A-1 init */
  eic_set_imr(0x00); /* ICW2: 8259A-1 IR0 mapped to 0x00 */
  eic_set_imr(0x00); /* ICW2: 8259A-1 (the master) does not have a slave */
  eic_set_imr(0x00); /* ICW4: master does not expect auto EOI */
  eic_set_imr(0xff); /* ICW5: mask all IRQs and finish init */

  eic_set_mask();
}

/**
 * @brief   EIC IRQ handler registration.
 *
 * @param[in] irq       irq number.
 * @param[in] handler   ponter to irq handler.
 * @param[in] data      opaque data passed to irq handler.
 */
void eic_lld_register_irq(int irq, eicIrqHandler handler, void *data) {
  if (irq >= EIC_NUM_IRQS)
    return;

  isr[irq].handler = handler;
  isr[irq].data = data;
}

/**
 * @brief   EIC IRQ handler unregistration.
 *
 * @param[in] irq       irq number.
 */
void eic_lld_unregister_irq(int irq) {
  isr[irq].handler = NULL;
}

/**
 * @brief   Enable EIC IRQ.
 *
 * @param[in] irq       irq number.
 */
void eic_lld_enable_irq(int irq) {
  if (irq >= EIC_NUM_IRQS)
    return;

  chSysLock();
  drv.mask &= ~(1<<irq);
  eic_set_mask();
  chSysUnlock();
}

/**
 * @brief   Disable EIC IRQ.
 *
 * @param[in] irq       irq number.
 */
void eic_lld_disable_irq(int irq) {
  if (irq >= EIC_NUM_IRQS)
    return;

  chSysLock();
  drv.mask |= 1<<irq;
  eic_set_mask();
  chSysUnlock();
}

/**
 * @brief   Ack EIC IRQ.
 *
 * @param[in] irq       irq number.
 */
void eic_lld_ack_irq(int irq) {
  if (irq >= EIC_NUM_IRQS)
    return;

  eic_ack(irq);
}

#endif /* HAL_USE_EIC */

/** @} */
