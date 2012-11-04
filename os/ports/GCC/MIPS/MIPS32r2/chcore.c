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
 * @file    MIPS/MIPS32r2/chcore.c
 * @brief   MIPS32r2 architecture port code.
 *
 * @addtogroup MIPS_CORE
 * @{
 */

#include "ch.h"
#include "hal.h"
#include "halconf.h"
#include "chdebug.h"

#include "mipsarch.h"

#define __entry \
  __attribute__ ((__section__(".core.entry")))       \
  __attribute__ ((__optimize__("-Os")))              \
  __attribute__ ((__aligned__(4)))

/**
 * Halts the system.
 */
#if !defined(__DOXYGEN__)
__attribute__((weak))
#endif
void port_halt(void) {
  port_disable();
#if CH_DBG_ENABLED && USE_MIPS_QEMU_UART
  {
    const char *m = dbg_panic_msg;
    while (*m)
      sd_lld_putc(*m++);
  }
#endif
  for (;;);
}

static void port_reset_timer(void) {
  c0_set_compare(c0_get_count() +
      (100000000 + CH_FREQUENCY/2) / CH_FREQUENCY);
}

static void port_timer_isr(void) {
  CH_IRQ_PROLOGUE();

  port_reset_timer();

  chSysLockFromIsr();
  chSysTimerHandlerI();
  chSysUnlockFromIsr();

  CH_IRQ_EPILOGUE();
}

static void __entry port_init_timer(void) {
  port_reset_timer();

  /* c0_set_cause(c0_get_cause()&~(1<<27)); */
}

static void __entry port_init_cpu(void) {
}

static void __entry port_init_cache(void) {
  volatile uint8_t *addr;

  /* Clear tagLo/tagHi */
  MTC0(0, $28, 0);
  MTC0(0, $29, 0);

#if MIPS_CPU_DCACHE_SIZE == MIPS_CPU_ICACHE_SIZE
  for (addr = (uint8_t *)MIPS_KSEG0_CACHED_BASE;
       addr < (uint8_t *)(MIPS_KSEG0_CACHED_BASE + MIPS_CPU_DCACHE_SIZE);
       addr += MIPS_CPU_DCACHE_LINE_SIZE) {
    asm volatile ("cache 9, 0(%0)" : : "r"(addr)); /* D-cache */
    asm volatile ("cache 8, 0(%0)" : : "r"(addr)); /* I-cache */
  }
#else
  for (addr = (uint8_t *)MIPS_KSEG0_CACHED_BASE;
       addr < (uint8_t *)(MIPS_KSEG0_CACHED_BASE + MIPS_CPU_DCACHE_SIZE);
       addr += MIPS_CPU_DCACHE_LINE_SIZE)
    asm volatile ("cache 9, 0(%0)" : : "r"(addr)); /* D-cache */

  for (addr = (uint8_t *)MIPS_KSEG0_CACHED_BASE;
       addr < (uint8_t *)(MIPS_KSEG0_CACHED_BASE + MIPS_CPU_ICACHE_SIZE);
       addr += MIPS_CPU_ICACHE_LINE_SIZE)
    asm volatile ("cache 8, 0(%0)" : : "r"(addr)); /* I-cache */
#endif

  // cached KSEG0
  c0_set_config0(c0_get_config0() & ~3);
}

/**
 * Exception dispatcher.
 */

void MIPS_HW_IRQ0_handler(void) __attribute__ ((weak));
void MIPS_HW_IRQ1_handler(void) __attribute__ ((weak));
void MIPS_HW_IRQ2_handler(void) __attribute__ ((weak));
void MIPS_HW_IRQ3_handler(void) __attribute__ ((weak));
void MIPS_HW_IRQ4_handler(void) __attribute__ ((weak));
void MIPS_HW_IRQ5_handler(void) __attribute__ ((weak));

typedef void (*mips_hw_irq_t)(void);

static mips_hw_irq_t hw_irq_table[] = {
  MIPS_HW_IRQ0_handler,
  MIPS_HW_IRQ1_handler,
  MIPS_HW_IRQ2_handler,
  MIPS_HW_IRQ3_handler,
  MIPS_HW_IRQ4_handler,
  MIPS_HW_IRQ5_handler,
};

void port_handle_exception(uint32_t cause, uint32_t status/* , uint32_t epc */) {
  uint32_t ex = (cause >> 2) & 0x1f;

  if (0 == ex) { /* IRQ */
    uint32_t ip = ((cause & status) >> 10) & 0x3F; /* only masked IRQs */

    if (cause&(1<<30))
      port_timer_isr();

    while (ip) {
      uint32_t i = 31 - __builtin_clz(ip);

      if (hw_irq_table[i])
        hw_irq_table[i]();

      ip &= ~(1 << i);
    }
  } else
    chDbgPanic("unhandled exception");
}

static void __entry port_init_irq(void) {
  uint32_t sr = c0_get_status();
  uint32_t ipti = c0_get_intctl() >> 29;
  unsigned int i;

  if (ipti)
    sr |= 1 << (ipti + 8); // IM[x] timer

  for (i=0; i<ARRAY_SIZE(hw_irq_table); ++i) {
    if (hw_irq_table[i])
      sr |= 1 << (i + 10); // IM[x]
  }

  c0_set_status(sr);
}

void __entry port_early_init(void) {
  port_init_cpu();
  port_init_cache();
  port_init_timer();
  port_init_irq();
}

/** @} */
