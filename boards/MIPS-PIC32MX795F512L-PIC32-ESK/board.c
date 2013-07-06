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
                | DEVCFG1_FPBDIV_1    // SYSCLK / 1
    );
PIC32MX_DEVCFG2(
                DEVCFG2_FPLLIDIV_2    // PLL Input Divider
                | DEVCFG2_FPLLODIV_1  // PLL Output Divider
                | DEVCFG2_FPLLMUL_20  // PLL Multiplier
                | DEVCFG2_FUPLLEN     // USB PLL Enabled
                | DEVCFG2_UPLLIDIV_2  // USB PLL Input Divider
    );
PIC32MX_DEVCFG3( PIC32MX_DEVCFG3_UID(0xBEBE) // User ID ;)
//                | DEVCFG3_FUSBIDIO
//                | DEVCFG3_FVBUSONIO
//                | DEVCFG3_FCANIO
                | DEVCFG3_FMIIEN            // Enable RMII
                | DEVCFG3_FETHIO            // Alternate ethernet pins
//                | DEVCFG3_FSRSSEL_7
    );

/*
 * Board-specific initialization code.
 */
void boardInit(void) {
}
