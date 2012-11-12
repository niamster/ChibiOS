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
 * @file    MIPS-PIC32MX/eic_lld.c
 * @brief   MIPS-PIC32MX low level EIC driver code.
 *
 * @addtogroup EIC
 * @{
 */

#include "ch.h"
#include "hal.h"

#undef TRUE
#undef FALSE

#include "plib.h"

#define TRUE 1
#define FALSE 0

#if HAL_USE_EIC || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables.                                                   */
/*===========================================================================*/

static EicIrqInfo iInfo[EIC_NUM_IRQS];
static EicIrqBank iBank[EIC_IRQ_BANK_QTY];

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/* Main EIC ISR */
CH_IRQ_HANDLER(MIPS_HW_IRQ0) { // In PIC32 single-vectored mode all interrupts are wired to IRQ0
  CH_IRQ_PROLOGUE();

  int bank;

  for (bank=0;bank<EIC_IRQ_BANK_QTY;++bank) {
    uint32_t pending = *iBank[bank].status & iBank[bank].mask;

    while (pending) {
      uint32_t i = 31 - __builtin_clz(pending);

      if (EIC_IRQ_CT != i) {    /* Core timer is handled depending on mtc0.cause status */
        EicIrqInfo *info = &iInfo[i];

        if (!info->handler)
          chDbgPanic("unhandled EIC irq");

        info->handler(info->data);
      }

      pending &= ~(1 << i);

      *iBank[bank].clear = 1 << i;
    }
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
  INTCONCLR = _INTCON_MVEC_MASK; // Single-vectored mode
#if defined(MIPS_USE_SHADOW_GPR)
  /* Since we now in IV mode and EIC=1 need to clear IPL(IM) bits */
  {
    uint32_t sr = c0_get_status();
    sr &= ~(0x7f << 10);
    c0_set_status(sr);
  }
  INTCONSET = _INTCON_SS0_MASK; // Use second shadow set on any vectored interrupt
#endif

  iBank[0].status = &IFS0;
  iBank[0].clear = &IFS0CLR;
  iBank[0].enable = &IEC0SET;
  iBank[0].disable = &IEC0CLR;

  iBank[1].status = &IFS1;
  iBank[1].clear = &IFS1CLR;
  iBank[1].enable = &IEC1SET;
  iBank[1].disable = &IEC1CLR;

  iBank[2].status = &IFS2;
  iBank[2].clear = &IFS2CLR;
  iBank[2].enable = &IEC2SET;
  iBank[2].disable = &IEC2CLR;

  /* *iBank[0].disable = 0xFFFFFFFF; */
  /* *iBank[1].disable = 0xFFFFFFFF; */
  /* *iBank[2].disable = 0xFFFFFFFF; */

  /* All interrupts have equal priority ... */
  IPC0CLR  = 0x1F1F1F1F;
  IPC1CLR  = 0x1F1F1F1F;
  IPC2CLR  = 0x1F1F1F1F;
  IPC3CLR  = 0x1F1F1F1F;
  IPC4CLR  = 0x1F1F1F1F;
  IPC5CLR  = 0x1F1F1F1F;
  IPC6CLR  = 0x1F1F1F1F;
  IPC7CLR  = 0x1F1F1F1F;
  IPC8CLR  = 0x1F1F1F1F;
  IPC9CLR  = 0x1F1F1F1F;
  IPC10CLR = 0x1F1F1F1F;
  IPC11CLR = 0x1F1F1F1F;
  IPC12CLR = 0x1F1F1F1F;

  IPC0SET  = 0x04040404;
  IPC1SET  = 0x04040404;
  IPC2SET  = 0x04040404;
  IPC3SET  = 0x04040404;
  IPC4SET  = 0x04040404;
  IPC5SET  = 0x04040404;
  IPC6SET  = 0x04040404;
  IPC7SET  = 0x04040404;
  IPC8SET  = 0x04040404;
  IPC9SET  = 0x04040404;
  IPC10SET = 0x04040404;
  IPC11SET = 0x04040404;
  IPC12SET = 0x04040404;

  eic_lld_enable_irq(EIC_IRQ_CT);
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

  if (iInfo[irq].handler)
    chDbgPanic("ISR is already registered for this irq");

  iInfo[irq].handler = handler;
  iInfo[irq].data = data;
}

/**
 * @brief   Enable EIC IRQ.
 *
 * @param[in] irq       irq number.
 */
void eic_lld_enable_irq(int irq) {
  int bank;

  if (irq >= EIC_NUM_IRQS)
    return;

  if (irq < 32)
    bank = 0;
  else if (irq < 64)
    bank = 1;
  else
    bank = 2;

  iBank[bank].mask |= 1 << (irq - bank*32);
  *iBank[bank].enable = 1 << (irq - bank*32);
}

/**
 * @brief   Disable EIC IRQ.
 *
 * @param[in] irq       irq number.
 */
void eic_lld_disable_irq(int irq) {
  int bank;

  if (irq >= EIC_NUM_IRQS)
    return;

  if (irq < 32)
    bank = 0;
  else if (irq < 64)
    bank = 1;
  else
    bank = 2;

  iBank[bank].mask &= ~(1 << (irq - bank*32));
  *iBank[bank].disable = 1 << (irq - bank*32);
}

#endif /* HAL_USE_EIC */

/** @} */
