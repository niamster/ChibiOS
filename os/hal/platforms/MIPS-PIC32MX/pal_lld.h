/*
    ChibiOS/RT - Copyright (C) 2006,2007,2008,2009,2010,
                 2011,2012 Giovanni Di Sirio.

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
#undef PAL_MODE_INPUT_ANALOG
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
typedef struct PicReg {
  volatile PicReg tris;
  volatile PicReg port;
  volatile PicReg latch;
  volatile PicReg odc;
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
 * @note      On PIC32 SoC some I/Os are configured as analog inputs.
 *            To make a uniform configuration all I/Os are configured
 *            as digital inputs w/o the pullups after the call to @p palInit.
 *            In the order of initialization @p palInit() is called before @p adcInit,
 *            which may reconfigure corresponding I/Os as analog if needed.
 *
 * @notapi
 */
#define pal_lld_init(config) do { _pal_lld_init(); } while (0)

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
  void _pal_lld_init(void);
  void _pal_lld_setgroupmode(ioportid_t port, ioportmask_t mask, iomode_t mode);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_PAL */

#endif /* _PAL_LLD_H_ */

/** @} */
