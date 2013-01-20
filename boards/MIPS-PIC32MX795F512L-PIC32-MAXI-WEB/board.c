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

#include "ch.h"
#include "hal.h"

PIC32MX_DEVCFG0(
                DEVCFG0_ICESEL_CH2    // Use PGC2/PGD2
                | DEVCFG0_DEBUG_DISABLED // Disable DEBUG
    );
PIC32MX_DEVCFG1(
                DEVCFG1_FNOSC_PRIPLL  // Primary oscillator with PLL
                | DEVCFG1_IESO        // Internal-external switch over
                | DEVCFG1_POSCMOD_HS  // HS oscillator
                | DEVCFG1_OSCIOFNC    // CLKO output active
                | DEVCFG1_FPBDIV_1    // SYSCLK / 1
    );
PIC32MX_DEVCFG2(
                DEVCFG2_FPLLIDIV_2    // PLL Input Divider
                | DEVCFG2_FPLLODIV_1  // PLL Output Divider
                | DEVCFG2_FPLLMUL_20  // PLL Multiplier
                | DEVCFG2_FUPLLEN     // USB PLL Enabled
                | DEVCFG2_UPLLIDIV_2  // USB PLL Input Divider
    );
PIC32MX_DEVCFG3(
                PIC32MX_DEVCFG3_UID(0xC0C0) // User ID ;)
                | DEVCFG3_FMIIEN            // Enable RMII
                | DEVCFG3_FETHIO            // Alternate ethernet pins
    );

/*
 * Board-specific initialization code.
 */
void boardInit(void) {
}
