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
 * MIPS CPU frequency
 */
#define MIPS_CPU_FREQ                   132000000

/*
 * MIPS CPU cache configuration
 */
#define MIPS_CPU_ICACHE_LINE_SIZE       32
#define MIPS_CPU_ICACHE_SIZE            16384

#define MIPS_CPU_DCACHE_LINE_SIZE       32
#define MIPS_CPU_DCACHE_SIZE            16384

/*
 * MIPS Timer frequency
 */
#define MIPS_TIMER_FREQ                 100000000

/* MIPS Memory map */
#define HAL_MIPS_QEMU_REGS_BASE         0x14000000
#define HAL_MIPS_QEMU_UART_BASE         (HAL_MIPS_QEMU_REGS_BASE + 0x3F8)

#endif  /* _MCUCONF_H_ */
