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
 * @file    MIPS-PIC32MX/pwm_lld.c
 * @brief   MIPS-PIC32MX PWM Driver subsystem low level driver source.
 *
 * @addtogroup PWM
 * @{
 */

#include "ch.h"
#include "hal.h"

#if HAL_USE_PWM || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants and error checks.                                        */
/*===========================================================================*/

/**
 * @brief   Configuration bits of con register.
 */
enum conBits {
  CON_ON       = 15, /* TMR Module On */
  CON_OCTSEL   = 3,  /* Output Compare Timer Select(1: Timer3, 0: Timer2) */
  
  /* Output Compare Mode Select */
  CON_OCM_COP     = 5,   /* Initialize OCx pin low, generate continuous output pulses on OCx pin */
  CON_OCM_PWM_FD  = 6,   /* PWM mode on OCx; Fault pin disabled */
  CON_OCM_MASK    = 0x3,
  CON_OCM_SHIFT   = 0,
};

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

typedef volatile struct {
  PicReg   con;
  PicReg   r;
  PicReg   rs;
} OcPort;

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
 * @brief   Output compare IRQ handler.
 *
 * @param[in] data        Driver associated with PWM
 */
static void lld_serve_interrupt(uint32_t irq, void *data) {
  PWMDriver *pwmd = data;
  const PWMConfig *cfg = pwmd->config;
  pwmchannel_t c;

  chSysLockFromIsr();

  for (c=0;c<PWM_CHANNELS;++c)
    if (cfg->channels[c].irq == irq) {
      cfg->channels[c].callback(pwmd);
      break;
    }

  chSysUnlockFromIsr();
}

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level PWM driver initialization.
 *
 * @notapi
 */
void pwm_lld_init(void) {
}

/**
 * @brief   Configures and activates the PWM peripheral.
 *
 * @param[in] pwmd      pointer to the @p PWMDriver object
 *
 * @notapi
 */
void pwm_lld_start(PWMDriver *pwmd) {
  if (pwmd->state == PWM_STOP) {
    const PWMConfig *cfg = pwmd->config;
    pwmchannel_t c;
    bool_t tmr3 = tmr3; // suppress GCC warning

    switch (cfg->gptc.irq) {
      default:
        chDbgPanic("Wrong timer source");
        break;
      case EIC_IRQ_TMR2:
        tmr3 = false;
        break;
      case EIC_IRQ_TMR3:
        tmr3 = true;
        break;
    }

    for (c=0;c<PWM_CHANNELS;++c) {
      const PWMChannelConfig *ccfg = &cfg->channels[c];

      if (ccfg->mode != PWM_OUTPUT_DISABLED) {
        OcPort *port = (OcPort *)ccfg->base;

        port->con.reg = (tmr3 << CON_OCTSEL)
          | (CON_OCM_PWM_FD << CON_OCM_SHIFT);

        chDbgAssert(!ccfg->callback, "pwm_lld_start", "PIC hw does not generate OC irq in PWM mode");

#if HAL_USE_EIC
        eicRegisterIrq(ccfg->irq, lld_serve_interrupt, pwmd);
#endif
      }
    }

    gptObjectInit(&pwmd->gptd);
    gptStart(&pwmd->gptd, &cfg->gptc);
    gptStartContinuous(&pwmd->gptd, pwmd->period);
  }
}

/**
 * @brief   Deactivates the PWM peripheral.
 *
 * @param[in] pwmd      pointer to the @p PWMDriver object
 *
 * @notapi
 */
void pwm_lld_stop(PWMDriver *pwmd) {
  if (pwmd->state == PWM_READY) {
    const PWMConfig *cfg = pwmd->config;
    pwmchannel_t c;

    for (c=0;c<PWM_CHANNELS;++c) {
      const PWMChannelConfig *ccfg = &cfg->channels[c];

      if (ccfg->mode != PWM_OUTPUT_DISABLED) {
        OcPort *port = (OcPort *)ccfg->base;

        port->con.clear = 1 << CON_ON;

#if HAL_USE_EIC
        if (ccfg->callback)
          eicDisableIrq(ccfg->irq);
        eicUnregisterIrq(ccfg->irq);
#endif
      }
    }

    gptStop(&pwmd->gptd);
  }
}

/**
 * @brief   Changes the period the PWM peripheral.
 * @details This function changes the period of a PWM unit that has already
 *          been activated using @p pwmStart().
 * @pre     The PWM unit must have been activated using @p pwmStart().
 * @post    The PWM unit period is changed to the new value.
 * @note    The function has effect at the next cycle start.
 * @note    If a period is specified that is shorter than the pulse width
 *          programmed in one of the channels then the behavior is not
 *          guaranteed.
 *
 * @param[in] pwmd      pointer to a @p PWMDriver object
 * @param[in] period    new cycle time in ticks
 *
 * @notapi
 */
void pwm_lld_change_period(PWMDriver *pwmd, pwmcnt_t period) {
  gptChangeInterval(&pwmd->gptd, period);
}

/**
 * @brief   Enables a PWM channel.
 * @pre     The PWM unit must have been activated using @p pwmStart().
 * @post    The channel is active using the specified configuration.
 *
 * @param[in] pwmd      pointer to a @p PWMDriver object
 * @param[in] channel   PWM channel identifier (0...PWM_CHANNELS-1)
 * @param[in] width     PWM pulse width as clock pulses number
 *
 * @notapi
 */
void pwm_lld_enable_channel(PWMDriver *pwmd,
                            pwmchannel_t channel,
                            pwmcnt_t width) {
  const PWMConfig *cfg = pwmd->config;
  const PWMChannelConfig *ccfg = &cfg->channels[channel];
  OcPort *port = (OcPort *)ccfg->base;

  if (pwmd->period == width)
    ++width;

  port->r.reg = width;
  port->rs.reg = width;
  port->con.set = 1 << CON_ON;

#if HAL_USE_EIC
  if (ccfg->callback)
    eicEnableIrq(ccfg->irq);
#endif
}

/**
 * @brief   Disables a PWM channel.
 * @pre     The PWM unit must have been activated using @p pwmStart().
 * @post    The channel is disabled and its output line returned to the
 *          idle state.
 *
 * @param[in] pwmd      pointer to a @p PWMDriver object
 * @param[in] channel   PWM channel identifier (0...PWM_CHANNELS-1)
 *
 * @notapi
 */
void pwm_lld_disable_channel(PWMDriver *pwmd, pwmchannel_t channel) {
  const PWMConfig *cfg = pwmd->config;
  const PWMChannelConfig *ccfg = &cfg->channels[channel];
  OcPort *port = (OcPort *)ccfg->base;

  port->con.clear = 1 << CON_ON;

#if HAL_USE_EIC
  if (ccfg->callback)
    eicDisableIrq(ccfg->irq);
#endif
}

/**
 * @brief   Returns a PWM channel status.
 * @pre     The PWM unit must have been activated using @p pwmStart().
 *
 * @param[in] pwmd      pointer to a @p PWMDriver object
 * @param[in] channel   PWM channel identifier (0...PWM_CHANNELS-1)
 *
 * @notapi
 */
bool_t pwm_lld_is_channel_enabled(PWMDriver *pwmd, pwmchannel_t channel) {
  const PWMConfig *cfg = pwmd->config;
  const PWMChannelConfig *ccfg = &cfg->channels[channel];
  OcPort *port = (OcPort *)ccfg->base;

  return (port->con.reg & (1 << CON_ON));
}

/**
 * @brief   Changes the width of the pulse of the PWM channel.
 * @details This function changes the width of the pulse of a PWM channel
 *          that has already been activated using @p pwmEnableChannel().
 * @pre     The PWM channel must have been activated using @p pwmEnableChannel().
 * @post    The width of the pulse of PWM channel is changed to the new value.
 * @note    If a pulse width is specified that is longer than the period
 *          programmed in the PWM unit then the behavior is undefined.
 *
 * @param[in] pwmd      pointer to a @p PWMDriver object
 * @param[in] channel   PWM channel identifier (0...PWM_CHANNELS-1)
 * @param[in] width     new width of the pulse in ticks
 *
 * @notapi
 */
void pwm_lld_channel_change_width(PWMDriver *pwmd,
                                  pwmchannel_t channel,
                                  pwmcnt_t width) {
  const PWMConfig *cfg = pwmd->config;
  const PWMChannelConfig *ccfg = &cfg->channels[channel];
  OcPort *port = (OcPort *)ccfg->base;

  if (pwmd->period == width)
    ++width;

  port->rs.reg = width;
}

#endif /* HAL_USE_PWM */

/** @} */
