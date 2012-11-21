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
      (MIPS_CPU_FREQ + CH_FREQUENCY/2) / CH_FREQUENCY);
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

  c0_set_cause(c0_get_cause()&~(1<<27));
}

static void __entry port_init_cpu(void) {
}

static void __entry port_init_cache(void) {
#if MIPS_CPU_DCACHE_SIZE || MIPS_CPU_ICACHE_SIZE
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
  c0_set_config0((c0_get_config0() & ~7) | 3);
#endif
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
void MIPS_HW_IRQ6_handler(void) __attribute__ ((weak));
void MIPS_HW_IRQ7_handler(void) __attribute__ ((weak));
#if MIPS_EXC_TABLE_SIZE > 8
void MIPS_HW_IRQ8_handler(void) __attribute__ ((weak));
void MIPS_HW_IRQ9_handler(void) __attribute__ ((weak));
void MIPS_HW_IRQ10_handler(void) __attribute__ ((weak));
void MIPS_HW_IRQ11_handler(void) __attribute__ ((weak));
void MIPS_HW_IRQ12_handler(void) __attribute__ ((weak));
void MIPS_HW_IRQ13_handler(void) __attribute__ ((weak));
void MIPS_HW_IRQ14_handler(void) __attribute__ ((weak));
void MIPS_HW_IRQ15_handler(void) __attribute__ ((weak));
void MIPS_HW_IRQ16_handler(void) __attribute__ ((weak));
void MIPS_HW_IRQ17_handler(void) __attribute__ ((weak));
void MIPS_HW_IRQ18_handler(void) __attribute__ ((weak));
void MIPS_HW_IRQ19_handler(void) __attribute__ ((weak));
void MIPS_HW_IRQ20_handler(void) __attribute__ ((weak));
void MIPS_HW_IRQ21_handler(void) __attribute__ ((weak));
void MIPS_HW_IRQ22_handler(void) __attribute__ ((weak));
void MIPS_HW_IRQ23_handler(void) __attribute__ ((weak));
void MIPS_HW_IRQ24_handler(void) __attribute__ ((weak));
void MIPS_HW_IRQ25_handler(void) __attribute__ ((weak));
void MIPS_HW_IRQ26_handler(void) __attribute__ ((weak));
void MIPS_HW_IRQ27_handler(void) __attribute__ ((weak));
void MIPS_HW_IRQ28_handler(void) __attribute__ ((weak));
void MIPS_HW_IRQ29_handler(void) __attribute__ ((weak));
void MIPS_HW_IRQ30_handler(void) __attribute__ ((weak));
void MIPS_HW_IRQ31_handler(void) __attribute__ ((weak));
void MIPS_HW_IRQ32_handler(void) __attribute__ ((weak));
void MIPS_HW_IRQ33_handler(void) __attribute__ ((weak));
void MIPS_HW_IRQ34_handler(void) __attribute__ ((weak));
void MIPS_HW_IRQ35_handler(void) __attribute__ ((weak));
void MIPS_HW_IRQ36_handler(void) __attribute__ ((weak));
void MIPS_HW_IRQ37_handler(void) __attribute__ ((weak));
void MIPS_HW_IRQ38_handler(void) __attribute__ ((weak));
void MIPS_HW_IRQ39_handler(void) __attribute__ ((weak));
void MIPS_HW_IRQ40_handler(void) __attribute__ ((weak));
void MIPS_HW_IRQ41_handler(void) __attribute__ ((weak));
void MIPS_HW_IRQ42_handler(void) __attribute__ ((weak));
void MIPS_HW_IRQ43_handler(void) __attribute__ ((weak));
void MIPS_HW_IRQ44_handler(void) __attribute__ ((weak));
void MIPS_HW_IRQ45_handler(void) __attribute__ ((weak));
void MIPS_HW_IRQ46_handler(void) __attribute__ ((weak));
void MIPS_HW_IRQ47_handler(void) __attribute__ ((weak));
void MIPS_HW_IRQ48_handler(void) __attribute__ ((weak));
void MIPS_HW_IRQ49_handler(void) __attribute__ ((weak));
void MIPS_HW_IRQ50_handler(void) __attribute__ ((weak));
void MIPS_HW_IRQ51_handler(void) __attribute__ ((weak));
void MIPS_HW_IRQ52_handler(void) __attribute__ ((weak));
void MIPS_HW_IRQ53_handler(void) __attribute__ ((weak));
void MIPS_HW_IRQ54_handler(void) __attribute__ ((weak));
void MIPS_HW_IRQ55_handler(void) __attribute__ ((weak));
void MIPS_HW_IRQ56_handler(void) __attribute__ ((weak));
void MIPS_HW_IRQ57_handler(void) __attribute__ ((weak));
void MIPS_HW_IRQ58_handler(void) __attribute__ ((weak));
void MIPS_HW_IRQ59_handler(void) __attribute__ ((weak));
void MIPS_HW_IRQ60_handler(void) __attribute__ ((weak));
void MIPS_HW_IRQ61_handler(void) __attribute__ ((weak));
void MIPS_HW_IRQ62_handler(void) __attribute__ ((weak));
void MIPS_HW_IRQ63_handler(void) __attribute__ ((weak));
#endif

typedef void (*mips_hw_irq_t)(void);

static mips_hw_irq_t hw_irq_table[] = {
  MIPS_HW_IRQ0_handler,
  MIPS_HW_IRQ1_handler,
  MIPS_HW_IRQ2_handler,
  MIPS_HW_IRQ3_handler,
  MIPS_HW_IRQ4_handler,
  MIPS_HW_IRQ5_handler,
  MIPS_HW_IRQ6_handler,
  MIPS_HW_IRQ7_handler,
#if MIPS_EXC_TABLE_SIZE > 8
  MIPS_HW_IRQ8_handler,
  MIPS_HW_IRQ9_handler,
  MIPS_HW_IRQ10_handler,
  MIPS_HW_IRQ11_handler,
  MIPS_HW_IRQ12_handler,
  MIPS_HW_IRQ13_handler,
  MIPS_HW_IRQ14_handler,
  MIPS_HW_IRQ15_handler,
  MIPS_HW_IRQ16_handler,
  MIPS_HW_IRQ17_handler,
  MIPS_HW_IRQ18_handler,
  MIPS_HW_IRQ19_handler,
  MIPS_HW_IRQ20_handler,
  MIPS_HW_IRQ21_handler,
  MIPS_HW_IRQ22_handler,
  MIPS_HW_IRQ23_handler,
  MIPS_HW_IRQ24_handler,
  MIPS_HW_IRQ25_handler,
  MIPS_HW_IRQ26_handler,
  MIPS_HW_IRQ27_handler,
  MIPS_HW_IRQ28_handler,
  MIPS_HW_IRQ29_handler,
  MIPS_HW_IRQ30_handler,
  MIPS_HW_IRQ31_handler,
  MIPS_HW_IRQ32_handler,
  MIPS_HW_IRQ33_handler,
  MIPS_HW_IRQ34_handler,
  MIPS_HW_IRQ35_handler,
  MIPS_HW_IRQ36_handler,
  MIPS_HW_IRQ37_handler,
  MIPS_HW_IRQ38_handler,
  MIPS_HW_IRQ39_handler,
  MIPS_HW_IRQ40_handler,
  MIPS_HW_IRQ41_handler,
  MIPS_HW_IRQ42_handler,
  MIPS_HW_IRQ43_handler,
  MIPS_HW_IRQ44_handler,
  MIPS_HW_IRQ45_handler,
  MIPS_HW_IRQ46_handler,
  MIPS_HW_IRQ47_handler,
  MIPS_HW_IRQ48_handler,
  MIPS_HW_IRQ49_handler,
  MIPS_HW_IRQ50_handler,
  MIPS_HW_IRQ51_handler,
  MIPS_HW_IRQ52_handler,
  MIPS_HW_IRQ53_handler,
  MIPS_HW_IRQ54_handler,
  MIPS_HW_IRQ55_handler,
  MIPS_HW_IRQ56_handler,
  MIPS_HW_IRQ57_handler,
  MIPS_HW_IRQ58_handler,
  MIPS_HW_IRQ59_handler,
  MIPS_HW_IRQ60_handler,
  MIPS_HW_IRQ61_handler,
  MIPS_HW_IRQ62_handler,
  MIPS_HW_IRQ63_handler,
#endif
};

bool_t port_handle_exception(uint32_t cause, uint32_t status, uint32_t epc) {
  uint32_t ex = (cause >> 2) & 0x1f;

  (void)epc;

  if (0 == ex) { /* IRQ */
    uint32_t ip = ((cause & status) >> 8) & 0xFF; /* only masked IRQs */

    if (cause&(1<<30))
      port_timer_isr();

    while (ip) {
      uint32_t i = 31 - __builtin_clz(ip);

      if (!hw_irq_table[i])
        chDbgPanic("spurious IRQ");

      hw_irq_table[i]();

      ip &= ~(1 << i);
    }
  } else
    chDbgPanic("unhandled exception");

  return chSchIsPreemptionRequired();
}

bool_t port_handle_irq(uint32_t irq, uint32_t cause) {
  if (cause&(1<<30))
    port_timer_isr();

  if (!hw_irq_table[irq])
    chDbgPanic("spurious IRQ");

  hw_irq_table[irq]();

  return chSchIsPreemptionRequired();
}

static void __entry port_init_irq(void) {
  uint32_t sr = c0_get_status();
  uint32_t ictl = c0_get_intctl();
  uint32_t ipti = ictl >> 29;
  unsigned int i;

  /* NOTE: This will not work in EIC mode(IM bits are treated as interrupt priority level)
   * EIC hal driver should then take care
   */
  if (ipti)
    sr |= 1 << (ipti + 8); // IM[x] timer

  for (i=0; i<8; ++i) {
    if (hw_irq_table[i])
      sr |= 1 << (i + 8); // IM[x]
  }

  c0_set_status(sr);

#if defined(MIPS_USE_SHADOW_GPR) || defined(MIPS_USE_VECTORED_IRQ)
  {
    uint32_t c = c0_get_cause();
    uint32_t sctl = c0_get_srsctl();
    uint32_t smap = c0_get_srsmap();

#if defined(MIPS_USE_SHADOW_GPR)
    /* Set same shadow registers set for all interrupts and exceptions */
    sctl |= 1 << 12; // Second shadow set on any exception other than a vectored interrupt
    smap = 0x11111111; // Second shadow set on any vectored interrupt
#endif

    c |= 1 << 23; // Enable IV mode

    ictl |= 1 << 5; // Spacing between the vectors(32)

    c0_set_srsctl(sctl);
    c0_set_srsmap(smap);
    c0_set_intctl(ictl);
    c0_set_cause(c);
  }
#endif
}

void __entry port_early_init(void) {
  port_init_cpu();
  port_init_cache();
  port_init_timer();
  port_init_irq();
}

/** @} */
