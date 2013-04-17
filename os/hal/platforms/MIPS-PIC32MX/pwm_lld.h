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

/**
 * @file    MIPS-PIC32MX/pwm_lld.h
 * @brief   MIPS-PIC32MX PWM Driver subsystem low level driver header.
 *
 * @addtogroup PWM
 * @{
 */

#ifndef _PWM_LLD_H_
#define _PWM_LLD_H_

#if HAL_USE_PWM || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @brief   Number of PWM channels per PWM driver.
 */
#define PWM_CHANNELS                        5

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

#undef PWM_OUTPUT_ACTIVE_LOW

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief PWM mode type.
 */
typedef uint32_t pwmmode_t;

/**
 * @brief   PWM channel type.
 */
typedef uint8_t pwmchannel_t;

/**
 * @brief   PWM counter type.
 */
typedef uint16_t pwmcnt_t;

/**
 * @brief   PWM driver channel configuration structure.
 */
typedef struct {
  /**
   * @brief Channel active logic level.
   */
  pwmmode_t                 mode;
  /**
   * @brief Channel callback pointer.
   * @note  This callback is invoked on the channel compare event. If set to
   *        @p NULL then the callback is disabled.
   */
  pwmcallback_t             callback;
  /* End of the mandatory fields.*/

  /**
   * @brief Output compare interrupt.
   */
  uint8_t                   irq;
  /**
   * @brief Output compare port.
   */
  uint32_t                  base;
} PWMChannelConfig;

/**
 * @brief   Driver configuration structure.
 */
typedef struct {
  /**
   * @brief   PWM period in ticks.
   */
  pwmcnt_t                  period;
  /**
   * @brief Channels configuration.
   */
  PWMChannelConfig          channels[PWM_CHANNELS];
  /* End of the mandatory fields.*/

  /**
   * @brief   Configuration data of associated GPT driver.
   */
  GPTConfig                 gptc;
} PWMConfig;

/**
 * @brief   Structure representing an PWM driver.
 */
struct PWMDriver {
  /**
   * @brief Driver state.
   */
  pwmstate_t                state;
  /**
   * @brief Current configuration data.
   */
  const PWMConfig           *config;
  /**
   * @brief   Current PWM period in ticks.
   */
  pwmcnt_t                  period;
  /* End of the mandatory fields.*/

  /**
   * @brief   GPT driver associated with this PWM.
   */
  GPTDriver                 gptd;
};

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  void pwm_lld_init(void);
  void pwm_lld_start(PWMDriver *pwmd);
  void pwm_lld_stop(PWMDriver *pwmd);
  void pwm_lld_change_period(PWMDriver *pwmd, pwmcnt_t period);
  void pwm_lld_enable_channel(PWMDriver *pwmd,
                              pwmchannel_t channel,
                              pwmcnt_t width);
  void pwm_lld_disable_channel(PWMDriver *pwmd, pwmchannel_t channel);
  bool_t pwm_lld_is_channel_enabled(PWMDriver *pwmd, pwmchannel_t channel);
  void pwm_lld_channel_change_width(PWMDriver *pwmd,
                                    pwmchannel_t channel,
                                    pwmcnt_t width);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_PWM */

#endif /* _PWM_LLD_H_ */

/** @} */
