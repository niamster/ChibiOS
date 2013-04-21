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
 * @file    MIPS-PIC32MX/hal_lld.h
 * @brief   MIPS-PIC32MX HAL subsystem low level driver header.
 *
 * @addtogroup HAL
 * @{
 */

#ifndef _HAL_LLD_H_
#define _HAL_LLD_H_

#include "mcu/pic32mxxx.h"

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @brief   Defines the support for realtime counters in the HAL.
 */
#define HAL_IMPLEMENTS_COUNTERS FALSE

/**
 * @brief   Platform name.
 */
#define PLATFORM_NAME   "MIPS-PIC32MX"

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*
 * DEVCFG0 register.
 */
/*===========================================================================*/
#if defined(PIC32MX1XX) || defined(PIC32MX2XX)
#define DEVCFG0_UNIMPLEMENTED   0x6EFE03E0
#define DEVCFG0_DEFAULT         0x00000000
#define DEVCFG0_INVERTED        0x1101FC00

#define DEVCFG0_DEBUG_DISABLED  0x00000002 /* Debugger disabled */
#define DEVCFG0_ICESEL_CH1      0x00000018 /* Use PGC1/PGD1 */
#define DEVCFG0_ICESEL_CH2      0x00000010 /* Use PGC2/PGD2 */
#define DEVCFG0_ICESEL_CH3      0x00000008 /* Use PGC3/PGD3 */
#define DEVCFG0_ICESEL_CH4      0x00000000 /* Use PGC4/PGD4 */
#define DEVCFG0_BWP             0x01000000 /* Boot flash write protect */
#define DEVCFG0_CP              0x10000000 /* Code protect */
#elif defined(PIC32MX3XX) || defined(PIC32MX4XX) \
   || defined(PIC32MX5XX) || defined(PIC32MX6XX) || defined(PIC32MX7XX)
#define DEVCFG0_UNIMPLEMENTED   0x6EF00FF4
#define DEVCFG0_DEFAULT         0x00000002
#define DEVCFG0_INVERTED        0x110FF000

#define DEVCFG0_DEBUG_DISABLED  0x00000003 /* Debugger disabled */
#define DEVCFG0_ICESEL_CH1      0x00000000 /* Use PGC1/PGD1 */
#define DEVCFG0_ICESEL_CH2      0x00000008 /* Use PGC2/PGD2 */
#define DEVCFG0_BWP             0x01000000 /* Boot flash write protect */
#define DEVCFG0_CP              0x10000000 /* Code protect */
#else
#error Unknown PIC32MX MCU
#endif

/*===========================================================================*/

/*
 * DEVCFG1 register.
 */
/*===========================================================================*/
#if defined(PIC32MX1XX) || defined(PIC32MX2XX) \
 || defined(PIC32MX3XX) || defined(PIC32MX4XX) \
 || defined(PIC32MX5XX) || defined(PIC32MX6XX) || defined(PIC32MX7XX)
#define DEVCFG1_UNIMPLEMENTED   0xFF600858
#define DEVCFG1_DEFAULT         0x00000000
#if defined(PIC32MX1XX) || defined(PIC32MX2XX) \
 || defined(PIC32MX5XX) || defined(PIC32MX6XX) || defined(PIC32MX7XX)
#define DEVCFG1_INVERTED        0x0000C700
#else
#define DEVCFG1_INVERTED        0x0000C300
#endif

/* Oscillator selection */
#define DEVCFG1_FNOSC_FRC       0x00000000 /* Fast RC */
#define DEVCFG1_FNOSC_FRCDIVPLL 0x00000001 /* Fast RC with divide-by-N and PLL */
#define DEVCFG1_FNOSC_PRI       0x00000002 /* Primary oscillator XT, HS, EC */
#define DEVCFG1_FNOSC_PRIPLL    0x00000003 /* Primary oscillator with PLL */
#define DEVCFG1_FNOSC_SEC       0x00000004 /* Secondary oscillator */
#define DEVCFG1_FNOSC_LPRC      0x00000005 /* Low-power RC */
#define DEVCFG1_FNOSC_FRCDIV    0x00000007 /* Fast RC with divide-by-N */

/* Primary oscillator */
#define DEVCFG1_POSCMOD_EXT     0x00000300 /* External mode */
#define DEVCFG1_POSCMOD_XT      0x00000200 /* XT oscillator */
#define DEVCFG1_POSCMOD_HS      0x00000100 /* HS oscillator */

#define DEVCFG1_FSOSCEN         0x00000020 /* Enable secondary oscillator */

#define DEVCFG1_IESO            0x00000080 /* Internal-external switch over */

#define DEVCFG1_OSCIOFNC        0x00000400 /* CLKO output active */

/* Peripheral clock divider */
#define DEVCFG1_FPBDIV_1        0x00000000 /* SYSCLK / 1 */
#define DEVCFG1_FPBDIV_2        0x00001000 /* SYSCLK / 2 */
#define DEVCFG1_FPBDIV_4        0x00002000 /* SYSCLK / 4 */
#define DEVCFG1_FPBDIV_8        0x00003000 /* SYSCLK / 8 */

#define DEVCFG1_FCKM            0x00004000 /* Fail-safe clock monitor */
#define DEVCFG1_FCKS            0x00008000 /* Clock switching */

#define DEVCFG1_FWDTEN          0x00800000 /* Watchdog */

/* Watchdog postscaler */
#define DEVCFG1_WDT_PS_1        0x00000000 /* 1:1 */
#define DEVCFG1_WDT_PS_2        0x00010000 /* 1:2 */
#define DEVCFG1_WDT_PS_4        0x00020000 /* 1:4 */
#define DEVCFG1_WDT_PS_8        0x00030000 /* 1:8 */
#define DEVCFG1_WDT_PS_16       0x00040000 /* 1:16 */
#define DEVCFG1_WDT_PS_32       0x00050000 /* 1:32 */
#define DEVCFG1_WDT_PS_64       0x00060000 /* 1:64 */
#define DEVCFG1_WDT_PS_128      0x00070000 /* 1:128 */
#define DEVCFG1_WDT_PS_256      0x00080000 /* 1:256 */
#define DEVCFG1_WDT_PS_512      0x00090000 /* 1:512 */
#define DEVCFG1_WDT_PS_1024     0x000A0000 /* 1:1024 */
#define DEVCFG1_WDT_PS_2048     0x000B0000 /* 1:2048 */
#define DEVCFG1_WDT_PS_4096     0x000C0000 /* 1:4096 */
#define DEVCFG1_WDT_PS_8192     0x000D0000 /* 1:8192 */
#define DEVCFG1_WDT_PS_16384    0x000E0000 /* 1:16384 */
#define DEVCFG1_WDT_PS_32768    0x000F0000 /* 1:32768 */
#define DEVCFG1_WDT_PS_65536    0x00100000 /* 1:65536 */
#define DEVCFG1_WDT_PS_131072   0x00110000 /* 1:131072 */
#define DEVCFG1_WDT_PS_262144   0x00120000 /* 1:262144 */
#define DEVCFG1_WDT_PS_524288   0x00130000 /* 1:524288 */
#define DEVCFG1_WDT_PS_1048576  0x00140000 /* 1:1048576 */
#else
#error Unknown PIC32MX MCU
#endif

/*===========================================================================*/

/*
 * DEVCFG2 register.
 */
/*===========================================================================*/
#if defined(PIC32MX1XX) || defined(PIC32MX2XX) \
    || defined(PIC32MX3XX) || defined(PIC32MX4XX) \
    || defined(PIC32MX5XX) || defined(PIC32MX6XX) || defined(PIC32MX7XX)
#define DEVCFG2_UNIMPLEMENTED   0xFFF87888
#define DEVCFG2_DEFAULT         0x00000000
#define DEVCFG2_INVERTED        0x00008000

/* PLL input divider */
#define DEVCFG2_FPLLIDIV_1      0x00000000 /* 1x */
#define DEVCFG2_FPLLIDIV_2      0x00000001 /* 2x */
#define DEVCFG2_FPLLIDIV_3      0x00000002 /* 3x */
#define DEVCFG2_FPLLIDIV_4      0x00000003 /* 4x */
#define DEVCFG2_FPLLIDIV_5      0x00000004 /* 5x */
#define DEVCFG2_FPLLIDIV_6      0x00000005 /* 6x */
#define DEVCFG2_FPLLIDIV_10     0x00000006 /* 10x */
#define DEVCFG2_FPLLIDIV_12     0x00000007 /* 12x */

/* PLL multiplier */
#define DEVCFG2_FPLLMUL_15      0x00000000 /* 15x */
#define DEVCFG2_FPLLMUL_16      0x00000010 /* 16x */
#define DEVCFG2_FPLLMUL_17      0x00000020 /* 17x */
#define DEVCFG2_FPLLMUL_18      0x00000030 /* 18x */
#define DEVCFG2_FPLLMUL_19      0x00000040 /* 19x */
#define DEVCFG2_FPLLMUL_20      0x00000050 /* 20x */
#define DEVCFG2_FPLLMUL_21      0x00000060 /* 21x */
#define DEVCFG2_FPLLMUL_24      0x00000070 /* 24x */

/* USB PLL input divider */
#define DEVCFG2_UPLLIDIV_1      0x00000000 /* 1x */
#define DEVCFG2_UPLLIDIV_2      0x00000100 /* 2x */
#define DEVCFG2_UPLLIDIV_3      0x00000200 /* 3x */
#define DEVCFG2_UPLLIDIV_4      0x00000300 /* 4x */
#define DEVCFG2_UPLLIDIV_5      0x00000400 /* 5x */
#define DEVCFG2_UPLLIDIV_6      0x00000500 /* 6x */
#define DEVCFG2_UPLLIDIV_10     0x00000600 /* 10x */
#define DEVCFG2_UPLLIDIV_12     0x00000700 /* 12x */

#define DEVCFG2_FUPLLEN         0x00008000 /* USB PLL */

/* PLL postscaler */
#define DEVCFG2_FPLLODIV_1      0x00000000 /* 1x */
#define DEVCFG2_FPLLODIV_2      0x00010000 /* 2x */
#define DEVCFG2_FPLLODIV_4      0x00020000 /* 4x */
#define DEVCFG2_FPLLODIV_8      0x00030000 /* 8x */
#define DEVCFG2_FPLLODIV_16     0x00040000 /* 16x */
#define DEVCFG2_FPLLODIV_32     0x00050000 /* 32x */
#define DEVCFG2_FPLLODIV_64     0x00060000 /* 64x */
#define DEVCFG2_FPLLODIV_256    0x00070000 /* 256x */
#else
#error Unknown PIC32MX MCU
#endif

/*===========================================================================*/

/*
 * DEVCFG3 register.
 */
#if defined(PIC32MX1XX) || defined(PIC32MX2XX)
#define DEVCFG3_UNIMPLEMENTED   0x0FFF0000
#define DEVCFG3_DEFAULT         0x00000000
#define DEVCFG3_INVERTED        0x00000000

#define DEVCFG3_PMDl1WAY        0x10000000 /* Peripheral Module Disable Configuration bit */
#define DEVCFG3_IOLWAY          0x20000000 /* Peripheral Pin Select Configuration bit */
#define DEVCFG3_FUSBIDIO        0x40000000 /* USBID controlled by port function */
#define DEVCFG3_FVBUSONIO       0x80000000 /* VBUS_ON controlled by port function */
#elif defined(PIC32MX3XX) || defined(PIC32MX4XX)
#define DEVCFG3_UNIMPLEMENTED   0xFFFF0000
#define DEVCFG3_DEFAULT         0x00000000
#define DEVCFG3_INVERTED        0x00000000
#elif defined(PIC32MX5XX) || defined(PIC32MX6XX) || defined(PIC32MX7XX)
#define DEVCFG3_UNIMPLEMENTED   0x38F80000
#define DEVCFG3_DEFAULT         0x00000000
#define DEVCFG3_INVERTED        0x07000000

#define DEVCFG3_FSRSSEL_ALL     0x00000000 /* All irqs assigned to shadow set */
#define DEVCFG3_FSRSSEL_1       0x00010000 /* Assign irq priority 1 to shadow set */
#define DEVCFG3_FSRSSEL_2       0x00020000 /* Assign irq priority 2 to shadow set */
#define DEVCFG3_FSRSSEL_3       0x00030000 /* Assign irq priority 3 to shadow set */
#define DEVCFG3_FSRSSEL_4       0x00040000 /* Assign irq priority 4 to shadow set */
#define DEVCFG3_FSRSSEL_5       0x00050000 /* Assign irq priority 5 to shadow set */
#define DEVCFG3_FSRSSEL_6       0x00060000 /* Assign irq priority 6 to shadow set */
#define DEVCFG3_FSRSSEL_7       0x00070000 /* Assign irq priority 7 to shadow set */

#define DEVCFG3_FMIIEN          0x01000000 /* Enable RMII */
#define DEVCFG3_FETHIO          0x02000000 /* Alternate ethernet pins */

#define DEVCFG3_FCANIO          0x04000000 /* Alternate CAN pins */

#define DEVCFG3_FUSBIDIO        0x40000000 /* USBID controlled by port function */
#define DEVCFG3_FVBUSONIO       0x80000000 /* VBUS_ON controlled by port function */
#else
#error Unknown PIC32MX MCU
#endif

/*===========================================================================*/

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Defines generic PIC32 register block.
 */
typedef volatile struct {
  volatile uint32_t reg;        /* Full access to the register */
  volatile uint32_t clear;      /* Clear corresponding bits in the register by writing bits at certain positions  */
  volatile uint32_t set;        /* Set corresponding bits in the register by writing bits at certain positions */
  volatile uint32_t invert;     /* Invert corresponding bits in the register by writing bits at certain positions */
} PicReg;

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

#define __PIC32MX_DEVCFG(c, v)                                          \
  const uint32_t __DEVCFG ## c ## __                                    \
  __attribute__ ((section (".devcfg" #c))) =                            \
    ((v) | DEVCFG ## c ## _UNIMPLEMENTED | DEVCFG ## c ## _DEFAULT) ^ DEVCFG ## c ## _INVERTED;

#define PIC32MX_DEVCFG0(c) __PIC32MX_DEVCFG(0, c)
#define PIC32MX_DEVCFG1(c) __PIC32MX_DEVCFG(1, c)
#define PIC32MX_DEVCFG2(c) __PIC32MX_DEVCFG(2, c)
#define PIC32MX_DEVCFG3(c) __PIC32MX_DEVCFG(3, c)

#define PIC32MX_DEVCFG3_UID(x) (x&0xFFFF)

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  void hal_lld_init(void);
  void hal_system_unlock(void);
  void hal_system_lock(void);
  void hal_lld_reset(void);
  uint32_t hal_pb_frequency(void);
#ifdef __cplusplus
}
#endif

#endif /* _HAL_LLD_H_ */

/** @} */
