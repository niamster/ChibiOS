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
