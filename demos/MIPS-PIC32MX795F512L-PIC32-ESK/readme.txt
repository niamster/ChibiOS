*****************************************************************************
** ChibiOS/RT port for PIC32 Ethernet Startup Kit                          **
*****************************************************************************

1. The demo includes: 
- latest ChibiOS/RT (2.7.0unstable)
- PIC32 port by Dmytro Milinevskyy (http://forum.chibios.org/phpbb/viewtopic.php?f=17&t=715)
- mac_lld driver
- lwIP and rudimentary web server

3. The demo was built using custom-built GCC compiler gcc-4.7.2.
See http://retrobsd.org/wiki/doku.php/doc/toolchain-mips for instructions.
Any toolchain based on GCC and GNU userspace programs should work.

2. The demo runs on a PIC32 Ethernet Startup Kit.
It currently includes a small shell available on both UART1 and USB ports and a test 
web server. The IP address is currently hardcoded to 192.168.10.40 (see halconf.h).

4. Use MPLABX to program the binary (ch.elf) to th ESK board. The on-boad
debugger/programmer works just fine. Just create a new Prebuilt Project,
pint it to ch.elf and set CPU type to PIC32MX795F512L.

5. mac_lld debuggng
There are trace macros in the mac_lld.c (Msg, Tr1, Tr2, Tr3) which are disabled by default.
To enable them, uncomment the DBL_LEVEL define and set it to desired verbosity level (0..3).
It is also necessary to use low-level version of the dbgprintf routine (located in main.c), 
because some trace macros are called from inside interrupt handler or critical section.
In order to switch to low-level dbgprintf, comment out the DBGPRINTF_SD define and uncomment
DBGPRINTF_LLD.
Note that some messages from early driver initialization routines may not be visible 
because serial port may be not yet initialized.