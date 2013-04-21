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
 * @file    MIPS-PIC32MX/pal_lld.h
 * @brief   MIPS-PIC32MX PAL subsystem low level driver header.
 *
 * @addtogroup PAL
 * @{
 */

#ifndef _PAL_LLD_H_
#define _PAL_LLD_H_

#if HAL_USE_PAL || defined(__DOXYGEN__)

#include "mcu/pic32mxxx.h"

/*===========================================================================*/
/* Unsupported modes and specific modes                                      */
/*===========================================================================*/

#undef PAL_MODE_RESET
#undef PAL_MODE_UNCONNECTED
#undef PAL_MODE_INPUT_PULLUP
#undef PAL_MODE_INPUT_PULLDOWN
#undef PAL_MODE_OUTPUT_PUSHPULL

/**
 * @brief   Regular output pad.
 */
#define PAL_MODE_OUTPUT                  255

/*===========================================================================*/
/* I/O Ports Types and constants.                                            */
/*===========================================================================*/

/**
 * @brief   GPIO static initializer.
 * @details An instance of this structure must be passed to @p palInit() at
 *          system startup time in order to initialized the digital I/O
 *          subsystem. This represents only the initial setup, specific pads
 *          or whole ports can be reprogrammed at later time.
 * @note    This structure is not used in the current implementation. See notes
 *          to @p palInit().
 */
typedef struct {
} PALConfig;

/**
 * @brief   Width, in bits, of an I/O port.
 */
#define PAL_IOPORTS_WIDTH 32

/**
 * @brief   Whole port mask.
 * @brief   This macro specifies all the valid bits into a port.
 */
#define PAL_WHOLE_PORT ((ioportmask_t)0xFFFFFFFF)

/**
 * @brief   Digital I/O port sized unsigned type.
 */
typedef uint32_t ioportmask_t;

/**
 * @brief   Digital I/O modes.
 */
typedef uint32_t iomode_t;

/**
 * @brief   Port Identifier.
 */
typedef volatile struct PicReg {
#if !defined(_AD1PCFG_PCFG_POSITION)
  PicReg ansel;
#endif
  PicReg tris;
  PicReg port;
  PicReg latch;
  PicReg odc;
} *ioportid_t;

/*===========================================================================*/
/* I/O Ports Identifiers.                                                    */
/*===========================================================================*/

/**
 * @brief   I/O port A identifier.
 */
#if defined(_PORTA_BASE_ADDRESS)
#define IOPORTA         ((ioportid_t)_PORTA_BASE_ADDRESS)
#endif

/**
 * @brief   I/O port B identifier.
 */
#if defined(_PORTB_BASE_ADDRESS)
#define IOPORTB         ((ioportid_t)_PORTB_BASE_ADDRESS)
#endif

/**
 * @brief   I/O port C identifier.
 */
#if defined(_PORTC_BASE_ADDRESS)
#define IOPORTC         ((ioportid_t)_PORTC_BASE_ADDRESS)
#endif

/**
 * @brief   I/O port D identifier.
 */
#if defined(_PORTD_BASE_ADDRESS)
#define IOPORTD         ((ioportid_t)_PORTD_BASE_ADDRESS)
#endif

/**
 * @brief   I/O port E identifier.
 */
#if defined(_PORTE_BASE_ADDRESS)
#define IOPORTE         ((ioportid_t)_PORTE_BASE_ADDRESS)
#endif

/**
 * @brief   I/O port F identifier.
 */
#if defined(_PORTF_BASE_ADDRESS)
#define IOPORTF         ((ioportid_t)_PORTF_BASE_ADDRESS)
#endif

/**
 * @brief   I/O port G identifier.
 */
#if defined(_PORTG_BASE_ADDRESS)
#define IOPORTG         ((ioportid_t)_PORTG_BASE_ADDRESS)
#endif

/**
 * @brief   PIC32 I/O ports configuration.
 *
 * @param[in] config    the PIC32 ports configuration
 *
 * @notapi
 */
#define pal_lld_init(config) do { } while (0)

/**
 * @brief   Reads the physical I/O port states.
 *
 * @param[in] port      port identifier
 * @return              The port bits.
 *
 * @notapi
 */
#define pal_lld_readport(p) ((p)->port.reg)

/**
 * @brief   Reads the output latch.
 * @details The purpose of this function is to read back the latched output
 *          value.
 *
 * @param[in] port      port identifier
 * @return              The latched logical states.
 *
 * @notapi
 */
#define pal_lld_readlatch(p) ((p)->latch.reg)

/**
 * @brief   Writes a bits mask on a I/O port.
 *
 * @param[in] port      port identifier
 * @param[in] bits      bits to be written on the specified port
 *
 * @notapi
 */
#define pal_lld_writeport(p, bits) do { (p)->port.reg = bits; } while (0)

/**
 * @brief   Sets a bits mask on a I/O port.
 * @note    The @ref PAL provides a default software implementation of this
 *          functionality, implement this function if can optimize it by using
 *          special hardware functionalities or special coding.
 *
 * @param[in] port      port identifier
 * @param[in] bits      bits to be ORed on the specified port
 *
 * @notapi
 */
#define pal_lld_setport(p, bits) do { (p)->port.set = bits; } while (0)

/**
 * @brief   Clears a bits mask on a I/O port.
 * @note    The @ref PAL provides a default software implementation of this
 *          functionality, implement this function if can optimize it by using
 *          special hardware functionalities or special coding.
 *
 * @param[in] port      port identifier
 * @param[in] bits      bits to be cleared on the specified port
 *
 * @notapi
 */
#define pal_lld_clearport(p, bits) do { (p)->port.clear = bits; } while (0)

/**
 * @brief   Toggles a bits mask on a I/O port.
 * @note    The @ref PAL provides a default software implementation of this
 *          functionality, implement this function if can optimize it by using
 *          special hardware functionalities or special coding.
 *
 * @param[in] port      port identifier
 * @param[in] bits      bits to be XORed on the specified port
 *
 * @notapi
 */
#define pal_lld_toggleport(p, bits) do { (p)->port.invert = bits; } while (0)

/**
 * @brief   Pads group mode setup.
 * @details This function programs a pads group belonging to the same port
 *          with the specified mode.
 *
 * @param[in] port      port identifier
 * @param[in] mask      group mask
 * @param[in] offset    group bit offset within the port
 * @param[in] mode      group mode
 *
 * @notapi
 */
#define pal_lld_setgroupmode(p, mask, offset, mode) _pal_lld_setgroupmode(p, mask << offset, mode)

/**
 * @note Following accessors are very well simulated by PAL API
 */
#undef pal_lld_readgroup
#undef pal_lld_writegroup
#undef pal_lld_readpad
#undef pal_lld_writepad
#undef pal_lld_setpad
#undef pal_lld_clearpad
#undef pal_lld_togglepad
#undef pal_lld_setpadmode

#ifdef __cplusplus
extern "C" {
#endif
  void _pal_lld_setgroupmode(ioportid_t port, ioportmask_t mask, iomode_t mode);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_PAL */

#endif /* _PAL_LLD_H_ */

/** @} */
