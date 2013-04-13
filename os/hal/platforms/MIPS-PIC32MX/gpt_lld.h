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
 * @file    MIPS-PIC32MX/gpt_lld.h
 * @brief   MIPS-PIC32MX GPT Driver subsystem low level driver header.
 *
 * @addtogroup GPT
 * @{
 */

#ifndef _GPT_LLD_H_
#define _GPT_LLD_H_

#if HAL_USE_GPT || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   GPT counter type.
 */
typedef uint16_t gptcnt_t;

/**
 * @brief   GPT input clock prescaler.
 */
enum gptPrescaler {
  GPT_PRESCALER_1,    /* 1:1 */
  GPT_PRESCALER_2,    /* 1:2 */
  GPT_PRESCALER_4,    /* 1:4 */
  GPT_PRESCALER_8,    /* 1:8 */
  GPT_PRESCALER_16,   /* 1:16 */
  GPT_PRESCALER_32,   /* 1:32 */
  GPT_PRESCALER_64,   /* 1:64 */
  GPT_PRESCALER_256,  /* 1:256 */
};

/**
 * @brief   Driver configuration structure.
 * @note    It could be empty on some architectures.
 */
typedef struct {
  /**
   * @brief   Timer input clock in Hz.
   */
  enum gptPrescaler         prescaler;
  /**
   * @brief   Timer callback pointer.
   * @note    This callback is invoked on GPT counter events.
   */
  gptcallback_t             callback;
  /* End of the mandatory fields.*/

  /**
   * @brief External clock from TxCKI pin.
   */
  bool_t                    ext;
  /**
   * @brief GPT interrupt.
   */
  uint8_t                   irq;
  /**
   * @brief GPT port.
   */
  uint32_t                  base;
} GPTConfig;

/**
 * @brief   Structure representing a GPT driver.
 */
struct GPTDriver {
  /**
   * @brief Driver state.
   */
  gptstate_t                state;
  /**
   * @brief Current configuration data.
   */
  const GPTConfig           *config;
#if defined(GPT_DRIVER_EXT_FIELDS)
  GPT_DRIVER_EXT_FIELDS
#endif
  /* End of the mandatory fields.*/
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
  void gpt_lld_init(void);
  void gpt_lld_start(GPTDriver *gptd);
  void gpt_lld_stop(GPTDriver *gptd);
  void gpt_lld_start_timer(GPTDriver *gptd, gptcnt_t period);
  void gpt_lld_stop_timer(GPTDriver *gptd);
  void gpt_lld_polled_delay(GPTDriver *gptd, gptcnt_t interval);
  void gpt_lld_change_interval(GPTDriver *gptd, gptcnt_t interval);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_GPT */

#endif /* _GPT_LLD_H_ */

/** @} */
