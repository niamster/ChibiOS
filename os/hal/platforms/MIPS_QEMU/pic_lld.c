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
 * @file    MIPS_QEMU/pic_lld.c
 * @brief   MIPS_QEMU low level PIC driver code.
 *
 * @addtogroup PIC
 * @{
 */

#include "ch.h"
#include "hal.h"

#if HAL_USE_PIC || defined(__DOXYGEN__)

#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables.                                                   */
/*===========================================================================*/

static PicDriver drv = {
  .mask = 0xff,
  .base = (void *)MIPS_UNCACHED(HAL_MIPS_QEMU_REGS_BASE),
};

static PicIrqInfo isr[PIC_NUM_IRQS];

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

static inline void pic_set_imr(uint8_t value) {
  volatile uint8_t *imr = (uint8_t *)drv.base + PIC_MASTER_IMR;

  *imr = value;
}

static inline uint8_t pic_get_imr(void) {
  volatile uint8_t *imr = (uint8_t *)drv.base + PIC_MASTER_IMR;

  return *imr;
}

static inline void pic_set_cmd(uint8_t value) {
  volatile uint8_t *imr = (uint8_t *)drv.base + PIC_MASTER_CMD;

  *imr = value;
}

static inline uint8_t pic_get_isr(void) {
  volatile uint8_t *isr = (uint8_t *)drv.base + PIC_MASTER_ISR;

  return *isr;
}

static inline void pic_set_mask(void) {
  pic_set_imr(drv.mask);
}

static inline uint8_t pic_pending(void) {
  return pic_get_isr() & ~drv.mask;
}

static void pic_ack(uint8_t irq) {
  volatile uint8_t dummy;
  pic_set_cmd(0x0C);        /* Switch to poll mode to ack the irq by reading ISR */
  dummy = pic_get_isr();
  (void)dummy;
  pic_set_cmd(0x60 + irq);  /* EOI to master */
}

/* Main PIC ISR */
CH_IRQ_HANDLER(MIPS_HW_IRQ0) {
  CH_IRQ_PROLOGUE();

  uint32_t pending = pic_pending();

  while (pending) {
    uint32_t i = 31 - __builtin_clz(pending);
    PicIrqInfo *info = &isr[i];

    pic_ack(i);

    if (!info->handler)
      chDbgPanic("unhandled PIC irq");

    info->handler(info->data);

    pending &= ~(1 << i);
  }

  CH_IRQ_EPILOGUE();
}

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   PIC LLD initialization.
 * @note    This function is implicitly invoked by @p picInit(), there is
 *          no need to explicitly initialize the driver.
 *
 * @init
 */
void pic_lld_init(void) {
  pic_set_cmd(0x11); /* ICW1: select 8259A-1 init */
  pic_set_imr(0x00); /* ICW2: 8259A-1 IR0 mapped to 0x00 */
  pic_set_imr(0x00); /* ICW2: 8259A-1 (the master) does not have a slave */
  pic_set_imr(0x00); /* ICW4: master does not expect auto EOI */
  pic_set_imr(0xff); /* ICW5: mask all IRQs and finish init */

  pic_set_mask();
}

/**
 * @brief   PIC IRQ handler registration.
 *
 * @param[in] irq       irq number.
 * @param[in] handler   ponter to irq handler.
 * @param[in] data      opaque data passed to irq handler.
 */
void pic_lld_register_irq(int irq, picIrqHandler handler, void *data) {
  if (irq >= PIC_NUM_IRQS)
    return;

  isr[irq].handler = handler;
  isr[irq].data = data;
}

/**
 * @brief   Enable PIC IRQ.
 *
 * @param[in] irq       irq number.
 */
void pic_lld_enable_irq(int irq) {
  if (irq >= PIC_NUM_IRQS)
    return;

  chSysLock();
  drv.mask &= ~(1<<irq);
  pic_set_mask();
  chSysUnlock();
}

/**
 * @brief   Disable PIC IRQ.
 *
 * @param[in] irq       irq number.
 */
void pic_lld_disable_irq(int irq) {
  if (irq >= PIC_NUM_IRQS)
    return;

  chSysLock();
  drv.mask |= 1<<irq;
  pic_set_mask();
  chSysUnlock();
}

#endif /* HAL_USE_PIC */

/** @} */
