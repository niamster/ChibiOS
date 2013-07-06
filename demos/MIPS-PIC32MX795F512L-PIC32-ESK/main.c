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

#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#include "ch.h"
#include "hal.h"
#include "shell.h"
#include "chprintf.h"
#include "chheap.h"
#include "test.h"
#include "usbcfg.h"
#include "mcu/pic32mxxx.h"
#include "lwipthread.h"
#include "web/web.h"
#include "../various/memstreams.h"

static SerialDriver SD1;
static const SerialConfig sd1_config = {
  .sc_baud    = SERIAL_DEFAULT_BITRATE,
  .sc_rxirq   = EIC_IRQ_UART1_RX,
  .sc_port    = _UART1_BASE_ADDRESS
};

static void oNotifySD1(GenericQueue *qp) {
  msg_t b;

  (void)qp;

  b = sdRequestDataI(&SD1);
  if (b != Q_EMPTY)
    sd_lld_putc(&SD1, b);
}

static MemoryStream dbg_stream;
static uint8_t dbg_stream_buffer[80];

void dbgInit(void) {
  msObjectInit(&dbg_stream, dbg_stream_buffer, sizeof(dbg_stream_buffer), 0);
  sd_lld_start_dbg(&SD1, &sd1_config);
}

void dbgPutc(char c) {
  if (SD1.base) {
    sd_lld_putc(&SD1,c);
    if (c=='\n')
      sd_lld_putc(&SD1, '\r');
  }
}

void dbgprintf(const char *fmt, ...) {
  va_list ap;
  uint8_t c;

  va_start(ap, fmt);
  chvprintf((BaseSequentialStream *)&dbg_stream, fmt, ap);
  va_end(ap);
  while (chSequentialStreamRead(&dbg_stream, &c, 1))
      dbgPutc(c);
  msObjectInit(&dbg_stream, dbg_stream_buffer, sizeof(dbg_stream_buffer), 0);
}

void dbgPuts(const char *s) {
  if (s) while (*s)
    dbgPutc(*s++);
}

void dbgPanic(const char *m) {
  port_enable();
  dbgPuts("\nPANIC: ");
  dbgPuts(m);
  dbgPutc('\n');
  port_disable();
  hal_lld_reset();
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

#define SHELL_WA_SIZE   THD_WA_SIZE(1024*2)

static void cmd_mem(BaseSequentialStream *chp, int argc, char *argv[]) {
  extern uint8_t __heap_base__[];
  extern uint8_t __heap_end__[];
  extern uint8_t __init_start__[];
  extern uint8_t __init_end__[];
  extern uint8_t __vectors_start__[];
  extern uint8_t __vectors_end__[];
  extern uint8_t __text_start__[];
  extern uint8_t __text_end__[];
#ifndef __XC32
  extern uint8_t __rom_data_start__[];
  extern uint8_t __ram_data_start__[];
  extern uint8_t __ram_data_end__[];
#endif
  extern uint8_t __rodata_start__[];
  extern uint8_t __rodata_end__[];
  extern uint8_t __bss_start__[];
  extern uint8_t __bss_end__[];
#ifdef __XC32
  struct dinit {
    uint32_t dst;
    uint32_t len;
    uint32_t fmt;
    uint8_t  data[];
  };
  extern struct dinit _dinit_addr;
  struct dinit *dinit = &_dinit_addr;
#endif

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
#ifndef __XC32
  chprintf(chp, "data:    %.8x:%.8x(%d bytes)\n",
      (uint32_t)(uint8_t *)__ram_data_start__, (uint32_t)(uint8_t *)__ram_data_end__, (uint32_t)(uint8_t *)__ram_data_end__ - (uint32_t)(uint8_t *)__ram_data_start__);
#endif
  chprintf(chp, "bss :    %.8x:%.8x(%d bytes)\n",
      (uint32_t)(uint8_t *)__bss_start__, (uint32_t)(uint8_t *)__bss_end__, (uint32_t)(uint8_t *)__bss_end__ - (uint32_t)(uint8_t *)__bss_start__);
  chprintf(chp, "heap:    %.8x:%.8x(%d bytes)\n",
      (uint32_t)(uint8_t *)__heap_base__, (uint32_t)(uint8_t *)__heap_end__, (uint32_t)(uint8_t *)__heap_end__ - (uint32_t)(uint8_t *)__heap_base__);

#ifdef __XC32
  if (dinit->dst)
    chprintf(chp, " dinit entries:\n");
  while (dinit->dst) {
    chprintf(chp, "   dst %.8x len %.8x fmt %s\n", dinit->dst, dinit->len, dinit->fmt?"copy":"clear");
    dinit = (struct dinit *)(((uint32_t)((uint8_t *)dinit + sizeof(struct dinit) + (dinit->fmt?dinit->len:0)) + 3) & ~3);
  }
#else
  if ((uint8_t *)__ram_data_start__ != (uint8_t *)__ram_data_end__
      && (uint8_t *)__rom_data_start__ != (uint8_t *)__ram_data_start__)
    chprintf(chp, " ROM .data was relocated from %.8x to %.8x\n",
        (uint32_t)(uint8_t *)__rom_data_start__,
        (uint32_t)(uint8_t *)__ram_data_start__);
#endif

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
static const ShellConfig ushCfg = {
  (BaseSequentialStream *)&SDU1,
  shCmds
};

static VirtualTimer ledTmr;

static void ledTmrFunc(void *p) {
  static uint32_t led = 0;

  (void)p;

  switch (led++%2) {
    case 0:
      palTogglePad(IOPORTB, 10);
      break;
    case 1:
      palTogglePad(IOPORTD, 1);
      break;
  }

  chVTSet(&ledTmr, MS2ST(500), ledTmrFunc, NULL);
}

void __attribute__((constructor)) ll_init(void) {
  dbgInit();
  dbgprintf("ll_init\n");

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
   * Activates the serial driver 1.
   */
  sdObjectInit(&SD1, NULL, oNotifySD1);
  sdStart(&SD1, &sd1_config);

  boardInfo();

  shellInit();

  /*
   * LED fun stuff
   */
  {
    palSetPadMode(IOPORTB, 10, PAL_MODE_OUTPUT);
    palClearPad(IOPORTB, 10);
    palSetPadMode(IOPORTD, 1, PAL_MODE_OUTPUT);
    palClearPad(IOPORTD, 1);

    chVTSet(&ledTmr, MS2ST(500), ledTmrFunc, NULL);
  }

  /*
   * Activates the USB driver
   */
  usbObjectInit(serUsbCfg.usbp);
  usbStart(serUsbCfg.usbp, &usbCfg);

  /*
   * Initializes a serial-over-USB CDC driver.
   */
  sduObjectInit(&SDU1);
  sduStart(&SDU1, &serUsbCfg);

  usbConnectBus(serUsbCfg.usbp);
}

/*
 * Application entry point.
 */
int main(void) {
  Thread *sh = NULL;
  Thread *ush = NULL;

  /*
   * Creates the LWIP threads (it changes priority internally).
   */
  chThdCreateStatic(wa_lwip_thread, LWIP_THREAD_STACK_SIZE, NORMALPRIO + 1,
                    lwip_thread, NULL);

  /*
   * Creates the HTTP thread (it changes priority internally).
   */
  chThdCreateStatic(wa_http_server, sizeof(wa_http_server), NORMALPRIO + 1,
                    http_server, NULL);

  /*
   * Normal main() thread activity ;).
   */

  for (;;) {
    if (!ush) {
      if (SDU1.config->usbp->state == USB_ACTIVE)
        ush = shellCreate(&ushCfg, SHELL_WA_SIZE, NORMALPRIO);
    } else if (chThdTerminated(ush)) {
      chThdRelease(ush);    /* Recovers memory of the previous shell. */
      ush = NULL;           /* Triggers spawning of a new shell.      */
    }
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
