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

