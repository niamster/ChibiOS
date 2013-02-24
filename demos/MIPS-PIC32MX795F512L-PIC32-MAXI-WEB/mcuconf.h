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

#ifndef _MCUCONF_H_
#define _MCUCONF_H_

/*
 * Use MIPS32r2 shadow registers in exception
 */
#define MIPS_USE_SHADOW_GPR

/*
 * Use MIPS32r2 vectored interrupt mode(enabled if MIPS_USE_SHADOW_GPR is defined)
 */
/* #define MIPS_USE_VECTORED_IRQ */

/*
 * MIPS CPU frequency
 */
#define MIPS_CPU_FREQ                   80000000UL

/*
 * MIPS CPU cache configuration
 * @note define both cashes as 0 to avoid early cache configuration
 */
#define MIPS_CPU_ICACHE_LINE_SIZE       16
#define MIPS_CPU_ICACHE_SIZE            0

#define MIPS_CPU_DCACHE_LINE_SIZE       16
#define MIPS_CPU_DCACHE_SIZE            0

/*
 * MIPS Timer frequency
 */
#define MIPS_TIMER_FREQ                 (MIPS_CPU_FREQ/2)

#endif  /* _MCUCONF_H_ */
