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

#include "ch.h"
#include "hal.h"
#include "shell.h"
#include "chprintf.h"
#include "chheap.h"
#include "test.h"

void dbgprintf(const char *fmt, ...) {
  va_list ap;

  va_start(ap, fmt);
  chvprintf((BaseSequentialStream *)&SD1, fmt, ap);
  va_end(ap);
}

void dbgPanic(const char *m) {
  while (*m)
    sd_lld_putc(*m++);
}

uint32_t hardJob(uint32_t arg) {
  return chTimeNow() + US2ST(arg);
}

static WORKING_AREA(waThread0, 128);
static msg_t Thread0(void *p) {
  uint32_t c = 0;

  (void)p;

  chRegSetThreadName("T0");

  dbgprintf("T0\n");

  while (TRUE) {
    c = hardJob(++c);
    chThdSleepMicroseconds(0);
  }

  return 0;
}

static WORKING_AREA(waThread1, 128);
static msg_t Thread1(void *p) {
  uint32_t c = 0;

  (void)p;

  chRegSetThreadName("T1");

  dbgprintf("T1\n");

  while (TRUE) {
    c = hardJob(c += 2);
    chThdSleepMicroseconds(0);
  }

  return 0;
}

void boardInfo(void) {
  dbgprintf("*** Kernel:       " CH_KERNEL_VERSION "\n");
#ifdef CH_COMPILER_NAME
  dbgprintf("*** Compiler:     " CH_COMPILER_NAME "\n");
#endif
  dbgprintf("*** Architecture: " CH_ARCHITECTURE_NAME "\n");
#ifdef CH_CORE_VARIANT_NAME
  dbgprintf("*** Core Variant: " CH_CORE_VARIANT_NAME "\n");
#endif
#ifdef CH_PORT_INFO
  dbgprintf("*** Port Info:    " CH_PORT_INFO "\n");
#endif
#ifdef PLATFORM_NAME
  dbgprintf("*** Platform:     " PLATFORM_NAME "\n");
#endif
#ifdef BOARD_NAME
  dbgprintf("*** Test Board:   " BOARD_NAME "\n");
#endif
}

#define SHELL_WA_SIZE   THD_WA_SIZE(1024)

static void cmd_mem(BaseSequentialStream *chp, int argc, char *argv[]) {
  extern uint8_t __heap_base__[];
  extern uint8_t __heap_end__[];
  extern uint8_t __init_start__[];
  extern uint8_t __init_end__[];
  extern uint8_t __vectors_start__[];
  extern uint8_t __vectors_end__[];
  extern uint8_t __text_start__[];
  extern uint8_t __text_end__[];
  extern uint8_t __rom_data_start__[];
  extern uint8_t __ram_data_start__[];
  extern uint8_t __ram_data_end__[];
  extern uint8_t __rodata_start__[];
  extern uint8_t __rodata_end__[];
  extern uint8_t __bss_start__[];
  extern uint8_t __bss_end__[];

#if CH_USE_HEAP
  size_t n, hSize;

  n = chHeapStatus(NULL, &hSize);
  chprintf(chp, "heap fragments   : %u\n", n);
  chprintf(chp, "heap total fragmented free space : %u bytes\n", hSize);
#else
  chprintf(chp, "Heap was not built in\n");
#endif
#if CH_USE_MEMCORE
  chprintf(chp, "core free memory : %u bytes\n", chCoreStatus());
#endif
  chprintf(chp, "init:    %.8x:%.8x(%d bytes)\n",
      (uint32_t)(uint8_t *)__init_start__, (uint32_t)(uint8_t *)__init_end__, (uint32_t)(uint8_t *)__init_end__ - (uint32_t)(uint8_t *)__init_start__);
  chprintf(chp, "vectors: %.8x:%.8x(%d bytes)\n",
      (uint32_t)(uint8_t *)__vectors_start__, (uint32_t)(uint8_t *)__vectors_end__, (uint32_t)(uint8_t *)__vectors_end__ - (uint32_t)(uint8_t *)__vectors_start__);
  chprintf(chp, "text:    %.8x:%.8x(%d bytes)\n",
      (uint32_t)(uint8_t *)__text_start__, (uint32_t)(uint8_t *)__text_end__, (uint32_t)(uint8_t *)__text_end__ - (uint32_t)(uint8_t *)__text_start__);
  chprintf(chp, "ro-data: %.8x:%.8x(%d bytes)\n",
      (uint32_t)(uint8_t *)__rodata_start__, (uint32_t)(uint8_t *)__rodata_end__, (uint32_t)(uint8_t *)__rodata_end__ - (uint32_t)(uint8_t *)__rodata_start__);
  chprintf(chp, "data:    %.8x:%.8x(%d bytes)\n",
      (uint32_t)(uint8_t *)__ram_data_start__, (uint32_t)(uint8_t *)__ram_data_end__, (uint32_t)(uint8_t *)__ram_data_end__ - (uint32_t)(uint8_t *)__ram_data_start__);
  chprintf(chp, "bss :    %.8x:%.8x(%d bytes)\n",
      (uint32_t)(uint8_t *)__bss_start__, (uint32_t)(uint8_t *)__bss_end__, (uint32_t)(uint8_t *)__bss_end__ - (uint32_t)(uint8_t *)__bss_start__);
  chprintf(chp, "heap:    %.8x:%.8x(%d bytes)\n",
      (uint32_t)(uint8_t *)__heap_base__, (uint32_t)(uint8_t *)__heap_end__, (uint32_t)(uint8_t *)__heap_end__ - (uint32_t)(uint8_t *)__heap_base__);

  if ((uint8_t *)__rom_data_start__ != (uint8_t *)__ram_data_start__)
      chprintf(chp, " ROM .data was relocated to RAM at %.8x\n", (uint32_t)(uint8_t *)__ram_data_start__);
  
  (void)argc;
  (void)argv;
}

static void cmd_threads(BaseSequentialStream *chp, int argc, char *argv[]) {
#if CH_USE_REGISTRY
  static const char *states[] = {
    "READY",
    "CURRENT",
    "SUSPENDED",
    "WTSEM",
    "WTMTX",
    "WTCOND",
    "SLEEPING",
    "WTEXIT",
    "WTOREVT",
    "WTANDEVT",
    "SNDMSGQ",
    "SNDMSG",
    "WTMSG",
    "WTQUEUE",
    "FINAL"
  };
  Thread *tp;

  chprintf(chp, "    addr       name    stack prio refs     state time\n");
  tp = chRegFirstThread();
  do {
#if CH_DBG_THREADS_PROFILING
    systime_t p_time = tp->p_time;
#else
    systime_t p_time = 0;
#endif
    chprintf(chp, "%.8x %10s %.8x %4lu %4lu %9s %lu\n",
        (uint32_t)tp, tp->p_name, (uint32_t)tp->p_ctx.sp,
        (uint32_t)tp->p_prio, (uint32_t)(tp->p_refs - 1),
        states[tp->p_state], (uint32_t)p_time);
    tp = chRegNextThread(tp);
  } while (tp != NULL);
#else
  chprintf(chp, "Registry was not built in\n");
#endif

  (void)argc;
  (void)argv;
}

static void cmd_test(BaseSequentialStream *chp, int argc, char *argv[]) {
  TestThread(chp);

  (void)argc;
  (void)argv;
}

static const ShellCommand shCmds[] = {
  {"mem",       cmd_mem},
  {"threads",   cmd_threads},
  {"test",      cmd_test},
  {NULL, NULL}
};

static const ShellConfig shCfg = {
  (BaseSequentialStream *)&SD1,
  shCmds
};

void __attribute__((constructor)) ll_init(void) {
  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();

  /*
   * Activates the serial driver 1 using the driver default configuration.
   */
  sdStart(&SD1, NULL);

  boardInfo();

  shellInit();

  /*
   * Creates the threads.
   */
  chThdCreateStatic(waThread0, sizeof(waThread0), NORMALPRIO-2, Thread0, NULL);
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO-1, Thread1, NULL);
}

/*
 * Application entry point.
 */
int main(void) {
  Thread *sh = NULL;

  /*
   * Normal main() thread activity ;).
   */

  for (;;) {
    if (!sh)
      sh = shellCreate(&shCfg, SHELL_WA_SIZE, NORMALPRIO);
    else if (chThdTerminated(sh)) {
      chThdRelease(sh);    /* Recovers memory of the previous shell. */
      sh = NULL;           /* Triggers spawning of a new shell.      */
    }

    chThdSleepMilliseconds(1000/* TIME_INFINITE */);
  }

  return 0;
}
