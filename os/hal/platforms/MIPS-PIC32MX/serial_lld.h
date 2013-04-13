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
 * @file    MIPS-PIC32MX/serial_lld.h
 * @brief   MIPS-PIC32MX low level serial driver header.
 *
 * @addtogroup SERIAL
 * @{
 */

#ifndef _SERIAL_LLD_H_
#define _SERIAL_LLD_H_

#if HAL_USE_SERIAL || defined(__DOXYGEN__)

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
 * @brief   Serial Driver configuration structure.
 * @details An instance of this structure must be passed to @p sdStart()
 *          in order to configure and start a serial driver operations.
 */
typedef struct {
  /**
   * @brief   Baud rate.
   */
  uint32_t  sc_baud;
  /**
   * @brief   RX IRQ number.
   */
  uint8_t   sc_rxirq;
  /**
   * @brief   Port address.
   */
  uint32_t  sc_port;
} SerialConfig;

/**
 * @brief   @p SerialDriver specific data.
 */
#define _serial_driver_data                           \
  _base_asynchronous_channel_data                     \
  /* Driver state.*/                                  \
  sdstate_t                 state;                    \
  /* Input queue.*/                                   \
  InputQueue                iqueue;                   \
  /* Output queue.*/                                  \
  OutputQueue               oqueue;                   \
  /* Input circular buffer.*/                         \
  uint8_t                   ib[SERIAL_BUFFERS_SIZE];  \
  /* Output circular buffer.*/                        \
  uint8_t                   ob[SERIAL_BUFFERS_SIZE];  \
  /* End of the mandatory fields.*/                   \
                                                      \
  /* UART port.*/                                     \
  void                      *base;

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  void sd_lld_init(void);
  void sd_lld_start(SerialDriver *sd, const SerialConfig *config);
  void sd_lld_stop(SerialDriver *sd);

  void sd_lld_putc(SerialDriver *sd, unsigned char c);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_SERIAL */

#endif /* _SERIAL_LLD_H_ */

/** @} */
