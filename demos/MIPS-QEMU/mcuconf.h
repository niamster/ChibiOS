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
