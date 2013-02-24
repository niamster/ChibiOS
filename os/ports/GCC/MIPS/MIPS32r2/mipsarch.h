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

/**
 * @file    MIPS/MIPS32r2/mipsarch.h
 * @brief   MIPS32r2 Specific ARCH Macros.
 *
 * @defgroup MIPS_MIPS32R2 MIPS32r2 Specific ARCH Macros
 * @ingroup MIPS_SPECIFIC
 * @details This file contains the MIPS specific ARCH for the
 *          MIPS32r2 platform.
 * @{
 */

#ifndef _MIPSARCH_H_
#define _MIPSARCH_H_

/**
 * @brief   MIPS FM Base Addresses
 */
#define MIPS_KSEG0_MASK                               (0xE0000000)
#define MIPS_KSEG0_CACHED_BASE                        (0x80000000)
#define MIPS_KSEG0_UNCACHED_BASE                      (0xA0000000)

/**
 * @brief   C and ASM macros to manipulate base addresses
 */
#define MIPS_CACHED(x)                       ((((x)) & ~MIPS_KSEG0_MASK) | MIPS_KSEG0_CACHED_BASE)
#define MIPS_UNCACHED(x)                     ((((x)) & ~MIPS_KSEG0_MASK) | MIPS_KSEG0_UNCACHED_BASE)
#define MIPS_PHYSICAL(x)                     (((x)) & ~MIPS_KSEG0_MASK)
#ifdef __ASSEMBLER__
#define MIPS_REG_CACHED(reg)                    \
  and     reg, reg, ~MIPS_KSEG0_MASK;           \
  or      reg, reg, MIPS_KSEG0_CACHED_BASE
#define MIPS_REG_UNCACHED(reg)                  \
  and     reg, reg, ~MIPS_KSEG0_MASK;           \
  or      reg, reg, MIPS_KSEG0_UNCACHED_BASE
#define MIPS_REG_PHYSICAL(reg)                  \
  and     reg, reg, ~MIPS_KSEG0_MASK
#endif /* __ASSEMBLER__ */

/**
 * @brief   Coprocessor 0 registers aliases.
 */
#define index       $0
#define random      $1
#define tlblo0      $2
#define tlblo1      $3
#define config      $3
#define context     $4
#define pagemask    $5
#define wired       $6
#define cachectrl   $7
#define badvr       $8
#define count       $9
#define tlbhi       $10
#define compare     $11
#define status      $12
#define intctl      $12, 1
#define srsctl      $12, 2
#define srsmap      $12, 3
#define cause       $13
#define epc         $14
#define prid        $15
#define ebase       $15, 1
#define config0     $16
#define lladdr      $17
#define xcontext    $20
#define ecc         $26
#define cache_err   $27
#define taglo       $28
#define taghi       $29
#define error_epc   $30

#ifndef __ASSEMBLER__
/**
 * @brief   Coprocessor 0 accessors.
 */

#define MFC0(reg, sel)                                                  \
  ({                                                                    \
    uint32_t __v;                                                       \
    asm volatile (".set push; .set noreorder; .set noat; mfc0 %0, " #reg ", " #sel "; ehb; .set pop" : "=r"(__v) :); \
    __v;                                                                \
  })

#define MTC0(v, reg, sel) do {                                          \
    uint32_t __v = v;                                                   \
    asm volatile (".set push; .set noreorder; .set noat; mtc0 %0, " #reg ", " #sel "; ehb; .set pop" : : "r"(__v)); \
  } while (0)

#if defined(MIPS_USE_MIPS16_ISA)
uint32_t __attribute__((nomips16)) c0_get_status(void);
void __attribute__((nomips16)) c0_set_status(uint32_t r);

uint32_t __attribute__((nomips16)) c0_get_config0(void);
void __attribute__((nomips16)) c0_set_config0(uint32_t r);

uint32_t __attribute__((nomips16)) c0_get_intctl(void);
void __attribute__((nomips16)) c0_set_intctl(uint32_t r);

uint32_t __attribute__((nomips16)) c0_get_srsctl(void);
void __attribute__((nomips16)) c0_set_srsctl(uint32_t r);

uint32_t __attribute__((nomips16)) c0_get_srsmap(void);
void __attribute__((nomips16)) c0_set_srsmap(uint32_t r);

uint32_t __attribute__((nomips16)) c0_get_cause(void);
void __attribute__((nomips16)) c0_set_cause(uint32_t r);

uint32_t __attribute__((nomips16)) c0_get_compare(void);
void __attribute__((nomips16)) c0_set_compare(uint32_t r);

uint32_t __attribute__((nomips16)) c0_get_count(void);
void __attribute__((nomips16)) c0_set_count(uint32_t r);
#else
#define c0_get_status       __c0_get_status
#define c0_set_status       __c0_set_status

#define c0_get_config0      __c0_get_config0
#define c0_set_config0      __c0_set_config0

#define c0_get_intctl       __c0_get_intctl
#define c0_set_intctl       __c0_set_intctl

#define c0_get_srsctl       __c0_get_srsctl
#define c0_set_srsctl       __c0_set_srsctl

#define c0_get_srsmap       __c0_get_srsmap
#define c0_set_srsmap       __c0_set_srsmap

#define c0_get_cause        __c0_get_cause
#define c0_set_cause        __c0_set_cause

#define c0_get_compare      __c0_get_compare
#define c0_set_compare      __c0_set_compare

#define c0_get_count        __c0_get_count
#define c0_set_count        __c0_set_count
#endif

#define __c0_get_status()     MFC0($12, 0)
#define __c0_set_status(r)    MTC0(r, $12, 0)

#define __c0_get_config0()    MFC0($16, 0)
#define __c0_set_config0(r)   MTC0(r, $16, 0)

#define __c0_get_intctl()     MFC0($12, 1)
#define __c0_set_intctl(r)    MTC0(r, $12, 1)

#define __c0_get_srsctl()     MFC0($12, 2)
#define __c0_set_srsctl(r)    MTC0(r, $12, 2)

#define __c0_get_srsmap()     MFC0($12, 3)
#define __c0_set_srsmap(r)    MTC0(r, $12, 3)

#define __c0_get_cause()      MFC0($13, 0)
#define __c0_set_cause(r)     MTC0(r, $13, 0)

#define __c0_get_compare()    MFC0($11, 0)
#define __c0_set_compare(r)   MTC0(r, $11, 0)

#define __c0_get_count()      MFC0($9, 0)
#define __c0_set_count(r)     MTC0(r, $9, 0)

#endif /* __ASSEMBLER__ */

#endif /* _MIPSARCH_H_ */

/** @} */
