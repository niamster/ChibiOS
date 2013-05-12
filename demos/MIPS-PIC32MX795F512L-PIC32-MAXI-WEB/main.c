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

#if defined(FATFS_DEMO)
#include "ff.h"
#endif

#include "test.h"
#include "testdma.h"

#if defined(GFX_DEMO)
#include "gfx.h"
#include "nyan.h"
#endif

#include "usbcfg.h"

#include "mcu/pic32mxxx.h"

static I2CConfig I2CC = {
  .frequency = 10000,
  .bus_irq = EIC_IRQ_I2C2_BUS,
  .slave_irq = EIC_IRQ_I2C2_SLAVE,
  .master_irq = EIC_IRQ_I2C2_MASTER,
  .base = _I2C2_BASE_ADDRESS,
};
static I2CDriver I2CD;

static RTCDriver RTC;

static dmaDriver DMA1;
static dmaChannel DMACH1;

static SerialDriver SD1;

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

static void pNotifyEXT1(EXTDriver *extd, expchannel_t channel) {
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

// timer prescaler is set to 64; peripheral clk prescaler is set to 1
// thereby timer frequency is 1.25MHz(assuming core clk is 80MHz), setting period to 50 milliseconds
#define GPT_FREQ_HZ    (MIPS_CPU_FREQ/64)
#define GPT_TICK_NS    (1000000000ULL/GPT_FREQ_HZ)
#define GPT_PERIOD_MS  50ULL
#define GPT_COUNT      ((GPT_PERIOD_MS*1000000ULL)/GPT_TICK_NS)

static void gptCb(GPTDriver *gptd) {
  static uint32_t cnt[2];

  int tmr = -1;

  switch (gptd->config->base) {
    case _TMR1_BASE_ADDRESS:
      tmr = 0;
      break;
    case _TMR2_BASE_ADDRESS:
      tmr = 1;
      break;
  }

  if (-1 == tmr)
    dbgprintf("unsupported timer");

  ++cnt[tmr];

  if (!(cnt[tmr]%100)) {
    RTCTime ts;


    rtcGetTime(&RTC, &ts);
    /* dbgprintf("TMR%d: %02d:%02d:%02d -> %u\n", tmr+1, */
    /*     ts.hours, ts.min, ts.sec, cnt[tmr]); */
  }
}

static GPTDriver GPT1;
static const GPTConfig GPTC1 = {
  .prescaler = GPT_PRESCALER_64,
  .ext = FALSE,
  .callback = gptCb,
  .irq = EIC_IRQ_TMR1,
  .base = _TMR1_BASE_ADDRESS,
};
      
static GPTDriver GPT2;
static const GPTConfig GPTC2 = {
  .prescaler = GPT_PRESCALER_64,
  .ext = FALSE,
  .callback = gptCb,
  .irq = EIC_IRQ_TMR2,
  .base = _TMR2_BASE_ADDRESS,
};

#define PWM_FREQ_HZ        14000ULL
#define PB_FREQ_HZ         MIPS_CPU_FREQ
// PWM Frequency = 1/[({PWM Period} + 1) * Tpb * Tp]
// Tp  := timer prescaler
// Tpb := 1/Fpb
// Fpb := peripheral bus clock frequency
// PWM Period = [Fpb/({PWM Frequency} * Tp)] - 1
// PWM resolution = log2(PWM Period)
// ---
// max(map(lambda Tp: (log2((PB_FREQ_HZ/(PWM_FREQ_HZ*Tp))-1), Tp), (1, 2, 4, 8, 16, 32, 64, 256))) -> (12.48, 1)
// to achieve max resolution for given PWM frequency 1:1 timer prescaler fits best
#define GPT_PR             1ULL
#define PWM_PERIOD         ((PB_FREQ_HZ/(PWM_FREQ_HZ*GPT_PR)) - 1)

#define container_of(ptr, type, member) ({                  \
      const typeof( ((type *)0)->member ) *__mptr = (ptr);	\
      (type *)( (char *)__mptr - __builtin_offsetof(type, member) );   \
    })

void pwmCb(GPTDriver *gptd) {
  PWMDriver *pwmd = container_of(gptd, PWMDriver, gptd);
  static uint32_t cnt;
  static uint16_t width;
  static int8_t dir;

  ++cnt;
  if (!(cnt%1000)) { // approximately 70mS at 14kHz PWM frequency
    if (width == 10000)
      dir = -100;
    else if (width == 0)
      dir = 100;

    width += dir;
    pwmChannelChangeWidthI(pwmd, 0, PWM_PERCENTAGE_TO_WIDTH(pwmd, width));
  }
}

static PWMDriver PWMD3;
static const PWMConfig PWMC3 = {
  .period = PWM_PERIOD,
  .channels = {
    {
      .mode = PWM_OUTPUT_ACTIVE_HIGH,
      .irq = EIC_IRQ_OC3,
      .base = _OCMP3_BASE_ADDRESS,
    },
  },
  .gptc = {
    .prescaler = GPT_PRESCALER_1,
    .callback = pwmCb,
    .ext = FALSE,
    .irq = EIC_IRQ_TMR3,
    .base = _TMR3_BASE_ADDRESS,
  },
};

#if defined(FATFS_DEMO)
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
#endif

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

#if defined(FATFS_DEMO)
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
#endif

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

static void cmd_rtc(BaseSequentialStream *chp, int argc, char *argv[]) {
  RTCTime timespec;

  if (!argc) {
   usage:
    chprintf(chp, "Usage:\n");
    chprintf(chp, "rtc get\n");
    chprintf(chp, "rtc set year month day wday hour minutes seconds\n");
    return;
  }

  if (!strcasecmp(argv[0], "get")) {
    rtcGetTime(&RTC, &timespec);
    chprintf(chp, "year 20%d month %d day %d wday %d hour %d minute %d second %d\n",
        timespec.year, timespec.month, timespec.day, timespec.wday,
        timespec.hours, timespec.min, timespec.sec);
  } else if (!strcasecmp(argv[0], "set")) {
    if (argc != 9)
      goto usage;

    /* rtcSetTime(&RTC, &timespec); */
    chprintf(chp, "unimplemented\n");
  } else
    goto usage;
}

static void cmd_test(BaseSequentialStream *chp, int argc, char *argv[]) {
  TestThread(chp);

  (void)argc;
  (void)argv;
}

#if defined(FATFS_DEMO)
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
#endif

static void cmd_accel(BaseSequentialStream *chp, int argc, char *argv[]) {
  const i2caddr_t addr = 0x38;
  uint8_t data[7] = {0, }, id = 0;
  uint8_t reg;
  msg_t ret;

  (void)argc;
  (void)argv;

  i2cAcquireBus(&I2CD);

  reg = 0x00;
  ret = i2cMasterTransmit(&I2CD, addr,
      &reg, 1, &id, 1);
  if (ret != RDY_OK)
    goto out;
  chprintf(chp, "accel id: %d\n", id);

  reg = 0x02;
  ret = i2cMasterTransmit(&I2CD, addr,
      &reg, 1, data, 7);
  if (ret != RDY_OK)
    goto out;
  chprintf(chp, "x: %d, y %d, z %d\n",
      (((uint16_t)data[1]) << 2) | (data[0] >> 6),
      (((uint16_t)data[3]) << 2) | (data[2] >> 6),
      (((uint16_t)data[5]) << 2) | (data[4] >> 6));
  chprintf(chp, "temp %f\n", ((float)data[6] * 0.5f) - 30.0f);

 out:
  if (ret)
    chprintf(chp, "ret: %d, err: %d\n", ret, i2cGetErrors(&I2CD));
  i2cReleaseBus(&I2CD);
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
  {"rtc",       cmd_rtc},
  {"test",      cmd_test},
#if defined(FATFS_DEMO)
  {"fs",        cmd_fs},
#endif
  {"accel",     cmd_accel},
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

#define RTC_THREAD_WORKAREA_SIZE 512

static BSEMAPHORE_DECL(rtcSem, TRUE);
static WORKING_AREA(waRtcThread, RTC_THREAD_WORKAREA_SIZE);

static void
ubtod(char *p, uint8_t n) {
  unsigned long rem;

  rem = n%10;
  n /= 10;
  p[1] = '0' + rem;
  rem = n%10;
  p[0] = '0' + rem;
}

static msg_t rtcThread(void *p) {
  RTCTime ts;
  static char time[] = "HH:MM:SS";
#if defined(GFX_DEMO)
  font_t font;
  coord_t	width, height;
#endif

  (void)p;

#if defined(GFX_DEMO)
  font = gdispOpenFont("LargeNumbers");
  width = gdispGetWidth();
  height = gdispGetHeight();
#endif

  for (;;) {
    chBSemWait(&rtcSem);

    rtcGetTime(&RTC, &ts);

    ubtod(time, ts.hours);
    ubtod(time+3, ts.min);
    ubtod(time+3+3, ts.sec);
    //dbgprintf("%s\n", time);
    
#if defined(GFX_DEMO)
    gdispFillString(width-10-gdispGetStringWidth(time, font), 10, time, font, Green, Orange);
#endif
  }

  return 0;
}

#if defined(GFX_DEMO) && GINPUT_NEED_MOUSE
#define MOUSE_THREAD_WORKAREA_SIZE 1024
static WORKING_AREA(waMouseThread, MOUSE_THREAD_WORKAREA_SIZE);

static msg_t mouseThread(void *p) {
  GEventMouse ev;
  coord_t	width, height;

  (void)p;

  width = gdispGetWidth() - 1;
  height = gdispGetHeight() - 1;

  for (;;) {
    ginputGetMouseStatus(0, &ev);

    if (!(ev.current_buttons & GINPUT_TOUCH_PRESSED))
      continue;

    if (ev.x >= width || ev.x < 2)
      continue;
    if (ev.y >= height || ev.y < 2)
      continue;
    if (ev.z < 100)
      continue;

    // dbgprintf("x %d y %d z %d\n", (uint32_t)ev.x, (uint32_t)ev.y, (uint32_t)ev.z);

    gdispFillCircle(ev.x, ev.y, 10, Black);
  }

  return 0;
}
#endif
 
static void rtcEvt(RTCDriver *rtcd, rtcevent_t event) {
  (void)rtcd;
  (void)event;

  chBSemSignalI(&rtcSem);
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

#if defined(FATFS_DEMO)
  /* Initialize and start the MMC driver to work with SPI4. */
  spiObjectInit(&SPID4);
  mmcObjectInit(&MMCD1);
  mmcStart(&MMCD1, &MMCC);
  palSetPadMode(IOPORTC, 1, PAL_MODE_OUTPUT);
  palSetPad(IOPORTC, 1);
#endif

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

    chVTSet(&ledTmr, MS2ST(500), ledTmrFunc, NULL);
  }

  /*
   * PWM fun
   * OC3 is bound to D.2
   */
  {
    palSetPadMode(IOPORTD, 2, PAL_MODE_OUTPUT);
    palClearPad(IOPORTD, 2);

    pwmObjectInit(&PWMD3);
    pwmStart(&PWMD3, &PWMC3);
    pwmEnableChannel(&PWMD3, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD3, 0));
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
    /* coord_t	i, j; */

    gdispInit();

    dbgprintf("LCD %x\n", DISPLAY_CODE);

    width = gdispGetWidth();
    height = gdispGetHeight();

#if GINPUT_NEED_MOUSE
    ginputGetMouse(0);
    chThdCreateStatic(waMouseThread, sizeof(waMouseThread), LOWPRIO, mouseThread, NULL);
#endif

    gdispClear(Fuchsia);

    gdispBlitArea(0, 0, nyan.width, nyan.height, (const pixel_t *)nyan.pixel_data);

    /* gdispDrawBox(10, 10, width/2, height/2, Yellow); */
    /* gdispFillArea(width/2, height/2, width/2-10, height/2-10, Blue); */
    /* gdispDrawLine(5, 30, width-50, height-40, Red); */
    
    /* for(i = 5, j = 0; i < width && j < height; i += 7, j += i/20) */
    /* 	gdispDrawPixel (i, j, White); */
  }
#endif

  /*
   * RTC configuration
   */
  {
    const RTCConfig cfg = {.base = _RTCC_BASE_ADDRESS, .irq = EIC_IRQ_RTC};
    const RTCTime timespec = {
      .year = 13,
      .month = 5,
      .day = 13,
      .wday = 0,
      .hours = 21,
      .min = 01,
      .sec = 20,
    };
    const RTCAlarm alarmspec = {
      .ts = {
        .month = 5,
        .day = 13,
        .wday = 0,
        .hours = 21,
        .min = 01,
        .sec = 21,
      },
      .repeat = TRUE,
      .period = ALARM_PERIOD_SECOND,
    };

    chThdCreateStatic(waRtcThread, sizeof(waRtcThread), LOWPRIO, rtcThread, NULL);

    rtcConfigure(&RTC, &cfg);
    rtcSetTime(&RTC, &timespec);
    rtcSetAlarm(&RTC, 0, &alarmspec);
    rtcSetCallback(&RTC, rtcEvt);
  }

  gptObjectInit(&GPT2);
  gptStart(&GPT2, &GPTC2);
  gptStartContinuous(&GPT2, GPT_COUNT);

  gptObjectInit(&GPT1);
  gptStart(&GPT1, &GPTC1);
  gptStartContinuous(&GPT1, GPT_COUNT);

  i2cObjectInit(&I2CD);
  i2cStart(&I2CD, &I2CC);
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
