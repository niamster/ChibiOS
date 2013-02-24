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

#define __entry                                      \
  __attribute__ ((__section__(".core.entry")))       \
  __attribute__ ((__optimize__("-O1")))              \
  __attribute__ ((__aligned__(4)))                   \
  __nomips16

void dbgPanic(const char *) __attribute__ ((weak));

/**
 * Halts the system.
 */
#if !defined(__DOXYGEN__)
__attribute__((weak))
#endif
void __nomips16 port_halt(void) {
  port_disable();
#if CH_DBG_ENABLED
  if (dbgPanic)
    dbgPanic(dbg_panic_msg);
#endif
  for (;;);
}

#if defined(MIPS_PORT_HANDLE_CORE_TIMER)
static uint8_t mt_ip_msk = ~0;

static void __nomips16
port_mips_timer_isr(void) {
  CH_IRQ_PROLOGUE();

  port_reset_mips_timer();

  chSysLockFromIsr();
  chSysTimerHandlerI();
  chSysUnlockFromIsr();

  CH_IRQ_EPILOGUE();
}
#endif

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

bool_t __nomips16
port_handle_exception(uint32_t cause, uint32_t status, uint32_t epc) {
  uint32_t ex = (cause >> 2) & 0x1f;

  (void)epc;

  if (0 == ex) { /* IRQ */
    uint32_t ip = ((cause & status) >> 8) & 0xFF; /* only masked IRQs */

#if defined(MIPS_PORT_HANDLE_CORE_TIMER)
    if (cause&(1<<30))
      port_mips_timer_isr();
    ip &= mt_ip_msk;
#endif

#if defined(MIPS_USE_MIPS16_ISA)
    {
      uint32_t i;

      for (i=0;i<8;++i) {
        if (ip&1) {
          if (!hw_irq_table[i])
            chDbgPanic("spurious IRQ");

          hw_irq_table[i]();
        }

        ip >>= 1;
      }
    }
#else
    while (ip) {
      uint32_t i = 31 - __builtin_clz(ip);

      if (!hw_irq_table[i])
        chDbgPanic("spurious IRQ");

      hw_irq_table[i]();

      ip &= ~(1 << i);
    }
#endif
  } else
    chDbgPanic("unhandled exception");

  return chSchIsPreemptionRequired();
}

bool_t port_handle_irq(uint32_t irq, uint32_t cause) {
#if defined(MIPS_PORT_HANDLE_CORE_TIMER)
  if (cause&(1<<30))
    port_mips_timer_isr();
#else
  (void)cause;
#endif

  chDbgAssert(hw_irq_table[irq], "spurious IRQ", "");

  hw_irq_table[irq]();

  return chSchIsPreemptionRequired();
}

static void __entry port_init_irq(void) {
  uint32_t mask = 0, i;

  /* NOTE: This will not work in EIC mode(IM bits are treated as interrupt priority level)
   * EIC hal driver should then take care
   */

#if defined(MIPS_PORT_HANDLE_CORE_TIMER)
  {
    uint32_t ipti = c0_get_intctl() >> 29;
    if (ipti) {
      mask |= 1 << (ipti + 8); // IM[x] timer
      mt_ip_msk = ~(mask >> 8);
    }
  }
#endif

  for (i=0; i<8; ++i) {
    if (hw_irq_table[i])
      mask |= 1 << (i + 8); // IM[x]
  }

  c0_set_status(mask | c0_get_status());

#if defined(MIPS_USE_VECTORED_IRQ) || defined(MIPS_USE_SHADOW_GPR)
  {
    c0_set_intctl(c0_get_intctl() | (1 << 5)); // Spacing between the vectors(32)
    c0_set_cause(c0_get_cause() | (1 << 23)); // Enable IV mode
  }
#endif
#if defined(MIPS_USE_SHADOW_GPR)
    {
      /* Set same shadow registers set for all interrupts and exceptions */

      // Second shadow set on any exception other than a vectored interrupt
      c0_set_srsctl(c0_get_srsctl() | (1 << 12));
      // Second shadow set on any vectored interrupt
      c0_set_srsmap(0x11111111);
    }
#endif
}

void __entry port_early_init(void) {
  port_init_cpu();
  port_init_cache();
#if defined(MIPS_PORT_HANDLE_CORE_TIMER)
  port_init_mips_timer();
#endif
  port_init_irq();
}

#if defined(MIPS_USE_MIPS16_ISA)
void __nomips16
port_lock(void) {
  MIPS_DISABLE_IRQ();
}

void __nomips16
port_unlock(void) {
  MIPS_RESTORE_IRQ(1);
}

void __nomips16
port_enable(void) {
  MIPS_RESTORE_IRQ(1);
}

void __nomips16
port_disable(void) {
  MIPS_DISABLE_IRQ();
}

void __nomips16
port_wait_for_interrupt(void) {
  MIPS_SIMPLE_ASM(wait);
}

uint32_t __nomips16
c0_get_status(void) {
  return __c0_get_status();
}

void __nomips16
c0_set_status(uint32_t r) {
  __c0_set_status(r);
}

uint32_t __nomips16
c0_get_config0(void) {
  return __c0_get_config0();
}

void __nomips16
c0_set_config0(uint32_t r) {
  __c0_set_config0(r);
}

uint32_t __nomips16
c0_get_intctl(void) {
  return __c0_get_intctl();
}

void __nomips16
c0_set_intctl(uint32_t r) {
  __c0_set_intctl(r);
}

uint32_t __nomips16
c0_get_srsctl(void) {
  return __c0_get_srsctl();
}

void __nomips16
c0_set_srsctl(uint32_t r) {
  __c0_set_srsctl(r);
}

uint32_t __nomips16
c0_get_srsmap(void) {
  return __c0_get_srsmap();
}

void __nomips16
c0_set_srsmap(uint32_t r) {
  __c0_set_srsmap(r);
}

uint32_t __nomips16
c0_get_cause(void) {
  return __c0_get_cause();
}

void __nomips16
c0_set_cause(uint32_t r) {
  __c0_set_cause(r);
}

uint32_t __nomips16
c0_get_compare(void) {
  return __c0_get_compare();
}

void __nomips16
c0_set_compare(uint32_t r) {
  __c0_set_compare(r);
}

uint32_t __nomips16
c0_get_count(void) {
  return __c0_get_count();
}

void __nomips16
c0_set_count(uint32_t r) {
  __c0_set_count(r);
}
#endif
/** @} */
