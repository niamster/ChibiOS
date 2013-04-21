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
 * @file    MIPS-PIC32MX/hal_lld.c
 * @brief   MIPS-PIC32MX HAL subsystem low level driver source.
 *
 * @addtogroup HAL
 * @{
 */

#include "ch.h"
#include "hal.h"

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#define FLASH_MAX_CLK          30000000

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level HAL driver initialization.
 *
 * @notapi
 */
void hal_lld_init(void) {
#if defined(_PCACHE)
  /* Do not enable D-cache because it's inefficient for PIC32 */

  CHECON = ((MIPS_CPU_FREQ/FLASH_MAX_CLK))      /* PFM Access Time Defined in terms of SYSLK Wait states bits */
    | (3 << _CHECON_PREFEN_POSITION)            /* Enable predictive prefetch cache for all regions */
    | (1 << _CHECON_CHECOH_POSITION);           /* Invalidate all data and instruction lines */

  BMXCONSET = _BMXCON_BMXCHEDMA_MASK; /* Enable program Flash memory (data) cacheability for DMA accesses */

  // cached KSEG0
  c0_set_config0((c0_get_config0() & ~7) | 3);
#endif

  BMXCONCLR = _BMXCON_BMXWSDRM_MASK; /* Data RAM accesses from CPU have zero wait states for address setup */
}

void hal_system_unlock(void) {
  SYSKEY = 0;
  SYSKEY = 0xAA996655;
  SYSKEY = 0x556699AA;
}

void hal_system_lock(void) {
  SYSKEY = 0x33333333;
}

/**
 * @brief   Device software reset.
 *
 * @notapi
 */
void hal_lld_reset(void) {
  volatile uint32_t dummy;

  port_disable();

  /* Suspend DMA */
  DMACONSET = _DMACON_SUSPEND_MASK;
  while ((DMACON >> _DMACON_DMABUSY_POSITION)&1);

  hal_system_unlock();

  /* Toggle SW reset */
  RSWRSTSET = 1 << _RSWRST_SWRST_POSITION;
  dummy = RSWRST;
  (void)dummy;

  for (;;);
}

/**
 * @brief   Returns the frequency of peripheral bus clock.
 *
 * @return  Peripheral bus clock frequency
 *
 * @notapi
 */
uint32_t hal_pb_frequency(void)
{
  return MIPS_CPU_FREQ >> OSCCONbits.PBDIV;
}

/** @} */
