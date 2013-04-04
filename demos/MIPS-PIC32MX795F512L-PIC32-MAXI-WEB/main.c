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

#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#include "ch.h"
#include "hal.h"
#include "shell.h"
#include "chprintf.h"
#include "chheap.h"

#include "ff.h"

#include "test.h"
#include "testdma.h"

#if defined(GFX_DEMO)
#include "gfx.h"
#include "fruits.h"
#endif

#include "usbcfg.h"

#include "mcu/pic32mxxx.h"

static dmaDriver DMA1;
static dmaChannel DMACH1;

static SerialDriver SD1;
static SerialUSBDriver SDU1;

void dbgprintf(const char *fmt, ...) {
  va_list ap;

  va_start(ap, fmt);
  chvprintf((BaseSequentialStream *)&SD1, fmt, ap);
  va_end(ap);
}

static void oNotifySD1(GenericQueue *qp) {
  msg_t b;

  (void)qp;

  b = sdRequestDataI(&SD1);
  if (b != Q_EMPTY)
    sd_lld_putc(&SD1, b);
}

void dbgPanic(const char *m) {
  if (SD1.base && m) {
    port_enable();
    while (*m)
      sd_lld_putc(&SD1, *m++);
    port_disable();
  }

  hal_lld_reset();
}

void pNotifyEXT1(EXTDriver *extd, expchannel_t channel) {
  const EXTChannelConfig *ch = &extd->config->channels[channel];

  dbgprintf("Channel %d, value %d\n",
      channel, palReadPad(ch->port, ch->pad));
}

static const EXTConfig EXTC1 = {
  .channels = {
    [15] = {EXT_CH_MODE_AUTOSTART|EXT_CH_MODE_FALLING_EDGE, pNotifyEXT1, IOPORTD, 6},
    [16] = {EXT_CH_MODE_AUTOSTART|EXT_CH_MODE_RISING_EDGE, pNotifyEXT1, IOPORTD, 7},
    [19] = {EXT_CH_MODE_AUTOSTART|EXT_CH_MODE_BOTH_EDGES, pNotifyEXT1, IOPORTD, 13},
  },
  .irq = EIC_IRQ_EXT,
  .base = _EXT_BASE_ADDRESS,
};
static EXTDriver EXTD1;

static SPIDriver SPID4;

bool_t mmc_lld_is_card_inserted(MMCDriver *mmcp) {
  (void)mmcp;

  return TRUE;
}

bool_t mmc_lld_is_write_protected(MMCDriver *mmcp) {
  (void)mmcp;

  return TRUE;
}

/* Maximum speed SPI configuration (20MHz, CPHA=0, CPOL=0).*/
static SPIConfig HS_SPIC = {
  .end_cb   = NULL,
  .cs       = {IOPORTC, 1},
  .width    = SPI_DATA_MODE_8_BIT,
  .master   = TRUE,
  .clk      = 20000000,
  .clk_mode = SPI_CLK_MODE0,
  .dmad     = &DMA1,
  .rx_irq   = EIC_IRQ_SPI4_RX,
  .base     = _SPI4_BASE_ADDRESS,
};

/* Low speed SPI configuration (200KHz, CPHA=0, CPOL=0).*/
static SPIConfig LS_SPIC = {
  .end_cb   = NULL,
  .cs       = {IOPORTC, 1},
  .width    = SPI_DATA_MODE_8_BIT,
  .master   = TRUE,
  .clk      = 200000,
  .clk_mode = SPI_CLK_MODE0,
  .dmad     = &DMA1,
  .rx_irq   = EIC_IRQ_SPI4_RX,
  .base     = _SPI4_BASE_ADDRESS,
};

MMCDriver MMCD1;
MMCConfig MMCC = {&SPID4, &LS_SPIC, &HS_SPIC};

FATFS MMC_FS;

uint32_t hardJob(uint32_t arg) {
  return chTimeNow() + US2ST(arg);
}

static WORKING_AREA(waThread0, 256);
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

static WORKING_AREA(waThread1, 256);
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

static FRESULT scan_files(BaseSequentialStream *chp, const char *path) {
  FRESULT res;
  FILINFO fno;
  DIR dir;
  char *fn;

#if _USE_LFN
  fno.lfname = 0;
  fno.lfsize = 0;
#endif
  res = f_opendir(&dir, path);
  if (FR_OK == res) {
    for (;;) {
      res = f_readdir(&dir, &fno);
      if (res != FR_OK || fno.fname[0] == 0)
        break;
      if (fno.fname[0] == '.')
        continue;
      fn = fno.fname;
      if (fno.fattrib & AM_DIR)
        chprintf(chp, "[D] /%s\n", fn);
      else
        chprintf(chp, "[F] /%s\n", fn);
    }
  }

  return res;
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

static void cmd_fs(BaseSequentialStream *chp, int argc, char *argv[]) {
  FRESULT err;
  uint32_t clusters;
  FATFS *fsp;

  if (mmcConnect(&MMCD1) != CH_SUCCESS) {
    chprintf(chp, "MMC not connected\n");
    return;
  }

  err = f_mount(0, &MMC_FS);
  if (err != FR_OK) {
    chprintf(chp, "FS: f_mount() failed\n");
    goto out;
  }

  err = f_getfree("/", &clusters, &fsp);
  if (FR_OK != err) {
    chprintf(chp, "FS: f_getfree() failed\n");
    goto out;
  }

  chprintf(chp,
           "FS: %lu free clusters, %lu sectors per cluster, %lu bytes free\n",
           clusters, (uint32_t)MMC_FS.csize,
           clusters * (uint32_t)MMC_FS.csize * 512);

  scan_files(chp, argc>0?argv[0]:"");

 out:
  mmcDisconnect(&MMCD1);
}

static void cmd_dmatest(BaseSequentialStream *chp, int argc, char *argv[]) {
  dmaChannelCfg ccfg = {.prio = DMA_CHANNEL_PRIO_LOWEST, .evt = FALSE};
  (void)argc;
  (void)argv;

  chDbgAssert(dmaChannelAcquire(&DMACH1, TIME_INFINITE),
      "Unable to acquire DMA channel", "");

  ccfg.mode = DMA_CHANNEL_MEM_TO_MEM;
  dmaChannelConfig(&DMACH1, &ccfg);
  dmaChannelStart(&DMACH1);
  TestDma(&DMACH1, DMA_TEST_MEM_TO_MEM, chp);
  dmaChannelStop(&DMACH1);

  ccfg.fifownd = 1;

  ccfg.mode = DMA_CHANNEL_FIFO_TO_MEM;
  dmaChannelConfig(&DMACH1, &ccfg);
  dmaChannelStart(&DMACH1);
  TestDma(&DMACH1, DMA_TEST_FIFO_TO_MEM, chp);
  dmaChannelStop(&DMACH1);

  ccfg.mode = DMA_CHANNEL_MEM_TO_FIFO;
  dmaChannelConfig(&DMACH1, &ccfg);
  dmaChannelStart(&DMACH1);
  TestDma(&DMACH1, DMA_TEST_MEM_TO_FIFO, chp);
  dmaChannelStop(&DMACH1);

  dmaChannelRelease(&DMACH1);
}

static const ShellCommand shCmds[] = {
  {"mem",       cmd_mem},
  {"threads",   cmd_threads},
  {"test",      cmd_test},
  {"fs",        cmd_fs},
  {"dmatest",   cmd_dmatest},
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

  switch (led++%3) {
    case 0:
      palTogglePad(IOPORTB, 10);
      break;
    case 1:
      palTogglePad(IOPORTD, 1);
      break;
    case 2:
      palTogglePad(IOPORTD, 2);
      break;
  }

  chVTSet(&ledTmr, MS2ST(500), ledTmrFunc, NULL);
}

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
   * Activates the serial driver 1.
   */
  {
    const SerialConfig sc = {
      .sc_baud    = SERIAL_DEFAULT_BITRATE,
      .sc_rxirq   = EIC_IRQ_UART1_RX,
      .sc_port    = _UART1_BASE_ADDRESS
    };

    sdObjectInit(&SD1, NULL, oNotifySD1);
    sdStart(&SD1, &sc);
  }

  /* Initialize and start the MMC driver to work with SPI4. */
  spiObjectInit(&SPID4);
  mmcObjectInit(&MMCD1);
  mmcStart(&MMCD1, &MMCC);
  palSetPadMode(IOPORTC, 1, PAL_MODE_OUTPUT);
  palSetPad(IOPORTC, 1);

  boardInfo();

  shellInit();

  /*
   * Creates the threads.
   */
  chThdCreateStatic(waThread0, sizeof(waThread0), NORMALPRIO-2, Thread0, NULL);
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO-1, Thread1, NULL);

  /*
   * Buttons notifications
   */
  extObjectInit(&EXTD1);
  extStart(&EXTD1, &EXTC1);

  /*
   * LED fun stuff
   */
  {
    palSetPadMode(IOPORTB, 10, PAL_MODE_OUTPUT);
    palClearPad(IOPORTB, 10);
    palSetPadMode(IOPORTD, 1, PAL_MODE_OUTPUT);
    palClearPad(IOPORTD, 1);
    palSetPadMode(IOPORTD, 2, PAL_MODE_OUTPUT);
    palClearPad(IOPORTD, 2);
    chVTSet(&ledTmr, MS2ST(500), ledTmrFunc, NULL);
  }

  /*
   * DMA configuration
   */
  {
    const dmaCfg cfg = {.port = _DMAC_BASE_ADDRESS};
    dmaObjectInit(&DMA1);
    dmaConfig(&DMA1, &cfg);
    dmaStart(&DMA1);

    dmaChannelObjectInit(&DMA1, &DMACH1);
  }

  /*
   * Activates the USB driver and then the USB bus pull-up on D+.
   * Note, a delay is inserted in order to not have to disconnect the cable
   * after a reset.
   */
  usbObjectInit(serUsbCfg.usbp);
  usbStart(serUsbCfg.usbp, &usbCfg);

  /*
   * Initializes a serial-over-USB CDC driver.
   */
  sduObjectInit(&SDU1);
  sduStart(&SDU1, &serUsbCfg);

  usbConnectBus(serUsbCfg.usbp);

#if defined(GFX_DEMO)
  {
    extern uint32_t DISPLAY_CODE;
    coord_t	width, height;
    coord_t	i, j;

    gdispInit();

    dbgprintf("LCD %x\n", DISPLAY_CODE);

    width = gdispGetWidth();
    height = gdispGetHeight();

    gdispClear(Fuchsia);

    gdispSetOrientation(GDISP_ROTATE_90);
    gdispBlitArea(0, 0, fruits.width, fruits.height, (const pixel_t *)fruits.pixel_data);

    gdispDrawBox(10, 10, width/2, height/2, Yellow);
    gdispFillArea(width/2, height/2, width/2-10, height/2-10, Blue);
    gdispDrawLine(5, 30, width-50, height-40, Red);
    
    for(i = 5, j = 0; i < width && j < height; i += 7, j += i/20)
    	gdispDrawPixel (i, j, White);
  }
#endif
}

/*
 * Application entry point.
 */
int main(void) {
  Thread *sh = NULL;
  Thread *ush = NULL;

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
