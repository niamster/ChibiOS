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
 * @file    MIPS_QEMU/pic_lld.h
 * @brief   MIPS_QEMU low level pic driver header.
 *
 * @addtogroup PIC
 * @{
 */

#ifndef _PIC_LLD_H_
#define _PIC_LLD_H_

#if HAL_USE_PIC || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/* Max number of IRQs */
#define PIC_NUM_IRQS    8

/* Well known IRQ numbers */
#define PIC_IRQ_UART    4

/* i8259A PIC registers */
#define PIC_MASTER_CMD    0x20
#define PIC_MASTER_IMR    0x21
#define PIC_MASTER_ISR    PIC_MASTER_CMD

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
 * @brief   PIC driver data.
 */
typedef struct PicDriver {
  /** @brief Current cached IRQ mask.*/
  uint8_t mask;
  /** @brief PIC io-memory base.*/
  void *base;
} PicDriver;

/**
 * @brief   PIC IRQ handler data.
 */
typedef struct PicIrqInfo {
  /** @brief IRQ handle.*/
  picIrqHandler handler;
  /** @brief IRQ handler data.*/
  void *data;
} PicIrqInfo;

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  void pic_lld_init(void);
  void pic_lld_register_irq(int irq, picIrqHandler handler, void *data);
  void pic_lld_enable_irq(int irq);
  void pic_lld_disable_irq(int irq);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_PIC */

#endif  /* _PIC_LLD_H_ */

/** @} */
