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

/**
 * @file    MIPS-PIC32MX/gpt_lld.c
 * @brief   MIPS-PIC32MX GPT Driver subsystem low level driver source.
 *
 * @addtogroup GPT
 * @{
 */

#include "ch.h"
#include "hal.h"

#if HAL_USE_GPT || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants and error checks.                                        */
/*===========================================================================*/

/**
 * @brief   Configuration bits of con register.
 */
enum conBits {
  CON_ON       = 15, /* TMR Module On */
  CON_T32      = 3,  /* 32-Bit(TimerB)/16-Bit(TimerA) Timer Mode Select */
  CON_TCS      = 1,  /* Timer Clock Source Select (1: External clock from TxCKI pin, 0: Internal peripheral clock) */
  
  /* TimerA Input Clock Prescale */
  CON_TA_PS_1      = 0,   /* 1:1 */
  CON_TA_PS_8      = 1,   /* 1:8 */
  CON_TA_PS_64     = 2,   /* 1:64 */
  CON_TA_PS_256    = 3,   /* 1:256 */
  CON_TA_PS_MASK   = 0x3,
  CON_TA_PS_SHIFT  = 4,

  /* TimerB Input Clock Prescale */
  CON_TB_PS_1      = 0,   /* 1:1 */
  CON_TB_PS_2      = 1,   /* 1:2 */
  CON_TB_PS_4      = 2,   /* 1:4 */
  CON_TB_PS_8      = 3,   /* 1:8 */
  CON_TB_PS_16     = 4,   /* 1:16 */
  CON_TB_PS_32     = 5,   /* 1:32 */
  CON_TB_PS_64     = 6,   /* 1:64 */
  CON_TB_PS_256    = 7,   /* 1:256 */
  CON_TB_PS_MASK   = 0x7,
  CON_TB_PS_SHIFT  = 4,
};

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

typedef volatile struct {
  PicReg   con;
  PicReg   tmr;
  PicReg   pr;
} TmrPort;

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/**
 * @brief   Timer IRQ handler.
 *
 * @param[in] data        Driver associated with GPT
 */
static void lld_serve_interrupt(void *data) {
  GPTDriver *gptd = data;
  const GPTConfig *cfg = gptd->config;
  TmrPort *port = (TmrPort *)cfg->base;

  chSysLockFromIsr();

  if (GPT_ONESHOT == gptd->state)
    port->con.clear = 1 << CON_ON;

  cfg->callback(gptd);

  chSysUnlockFromIsr();
}

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level GPT driver initialization.
 *
 * @notapi
 */
void gpt_lld_init(void) {
}

/**
 * @brief   Configures and activates the GPT peripheral.
 *
 * @param[in] gptd      pointer to the @p GPTDriver object
 *
 * @notapi
 */
void gpt_lld_start(GPTDriver *gptd) {
  const GPTConfig *cfg = gptd->config;
  TmrPort *port = (TmrPort *)cfg->base;
  chDbgAssert(port, "gpt_lld_start(), #1", "wrong configuration");

  if (gptd->state == GPT_STOP) {
    uint8_t prescaler = prescaler; // suppress GCC warning

#if HAL_USE_EIC
    eicRegisterIrq(cfg->irq, lld_serve_interrupt, gptd);
    if (cfg->callback)
      eicEnableIrq(cfg->irq);
#endif

    if (_TMR1_BASE_ADDRESS == cfg->base) {
      switch (cfg->prescaler) {
        default:
          chDbgPanic("unsupported prescaler value");
          break;
        case GPT_PRESCALER_1:
          prescaler = CON_TA_PS_1;
          break;
        case GPT_PRESCALER_8:
          prescaler = CON_TA_PS_8;
          break;
        case GPT_PRESCALER_64:
          prescaler = CON_TA_PS_64;
          break;
        case GPT_PRESCALER_256:
          prescaler = CON_TA_PS_256;
          break;
      }

      prescaler <<= CON_TA_PS_SHIFT;
  } else {
      switch (cfg->prescaler) {
        default:
          chDbgPanic("unsupported prescaler value");
          break;
        case GPT_PRESCALER_1:
          prescaler = CON_TB_PS_1;
          break;
        case GPT_PRESCALER_2:
          prescaler = CON_TB_PS_2;
          break;
        case GPT_PRESCALER_4:
          prescaler = CON_TB_PS_4;
          break;
        case GPT_PRESCALER_8:
          prescaler = CON_TB_PS_8;
          break;
        case GPT_PRESCALER_16:
          prescaler = CON_TB_PS_16;
          break;
        case GPT_PRESCALER_32:
          prescaler = CON_TB_PS_32;
          break;
        case GPT_PRESCALER_64:
          prescaler = CON_TB_PS_64;
          break;
        case GPT_PRESCALER_256:
          prescaler = CON_TB_PS_256;
          break;
      }

      prescaler <<= CON_TB_PS_SHIFT;
    }

    port->con.reg = prescaler | (cfg->ext << CON_TCS);
  }
}

/**
 * @brief   Deactivates the GPT peripheral.
 *
 * @param[in] gptd      pointer to the @p GPTDriver object
 *
 * @notapi
 */
void gpt_lld_stop(GPTDriver *gptd) {
  const GPTConfig *cfg = gptd->config;
  TmrPort *port = (TmrPort *)cfg->base;

  if (gptd->state == GPT_READY) {
#if HAL_USE_EIC
    if (cfg->callback)
      eicDisableIrq(cfg->irq);
    eicUnregisterIrq(cfg->irq);
#endif

    port->con.clear = 1 << CON_ON;
  }
}

/**
 * @brief   Starts the timer in continuous mode.
 *
 * @param[in] gptd      pointer to the @p GPTDriver object
 * @param[in] interval  period in ticks
 *
 * @notapi
 */
void gpt_lld_start_timer(GPTDriver *gptd, gptcnt_t interval) {
  const GPTConfig *cfg = gptd->config;
  TmrPort *port = (TmrPort *)cfg->base;

  port->pr.reg = interval;
  port->con.set = 1 << CON_ON;
}

/**
 * @brief   Stops the timer.
 *
 * @param[in] gptd      pointer to the @p GPTDriver object
 *
 * @notapi
 */
void gpt_lld_stop_timer(GPTDriver *gptd) {
  const GPTConfig *cfg = gptd->config;
  TmrPort *port = (TmrPort *)cfg->base;

  port->con.clear = 1 << CON_ON;
}

/**
 * @brief   Starts the timer in one shot mode and waits for completion.
 * @details This function specifically polls the timer waiting for completion
 *          in order to not have extra delays caused by interrupt servicing,
 *          this function is only recommended for short delays.
 *
 * @param[in] gptd      pointer to the @p GPTDriver object
 * @param[in] interval  time interval in ticks
 *
 * @notapi
 */
void gpt_lld_polled_delay(GPTDriver *gptd, gptcnt_t interval) {
  const GPTConfig *cfg = gptd->config;
  TmrPort *port = (TmrPort *)cfg->base;

  chDbgAssert(interval < 0xFFF0, "gpt_lld_polled_delay()", "interval value too big");

#if HAL_USE_EIC
  if (cfg->callback)
    eicDisableIrq(cfg->irq);
#endif

  port->pr.reg = 0xFFFF;
  port->con.set = 1 << CON_ON;

  while (port->tmr.reg < interval);

  port->con.clear = 1 << CON_ON;

#if HAL_USE_EIC
  if (cfg->callback) {
    eicAckIrq(cfg->irq);
    eicEnableIrq(cfg->irq);
  }
#endif
}

/**
 * @brief   Changes the interval of GPT peripheral.
 * @details This function changes the interval of a running GPT unit.
 * @pre     The GPT unit must have been activated using @p gptStart().
 * @pre     The GPT unit must have been running in continuous mode using
 *          @p gptStartContinuous().
 * @post    The GPT unit interval is changed to the new value.
 *
 * @param[in] gptd      pointer to a @p GPTDriver object
 * @param[in] interval  new cycle time in timer ticks
 *
 * @notapi
 */
void gpt_lld_change_interval(GPTDriver *gptd, gptcnt_t interval) {
  const GPTConfig *cfg = gptd->config;
  TmrPort *port = (TmrPort *)cfg->base;

  port->pr.reg = interval;
}

#endif /* HAL_USE_GPT */

/** @} */
