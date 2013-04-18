*****************************************************************************
** ChibiOS/RT port for QEMU MIPS 24kc.                                     **
*****************************************************************************

** TARGET **

The demo runs on a OLIMEX PIC32MX795F512L(MIPS 4kc) board.

** The Demo **

The demo currently just executes shell with commands to show threads activity,
memory usage and run Chibios test suite.

** Build Procedure **

The demo was built using the official MIPS toolchain(http://developer.mips.com/tools/compilers/bare-metal-toolchain/).
Any toolchain based on GCC and GNU userspace programs will work.

** Notes **

All files used by the demo are part of ChibiOS/RT.

Once Chibios ELF is ready(build/ch.elf) you can flash it with MPLABX studio.

First steps to create MPLABX project for prebuilt image (as for v1.41):

File->New Project
  1. Chose Project:
     Categories: Microchip Embedded
     Projects: Prebuilt (Hex, Loadable Image) Project
  2. Create Prebuilt Project
     Prebuilt Filename: /full/path/to/ch.elf
     Device: PIC32MX795F512L
     ....

The flash it using your favorite PIC debugger/programmer(works fine with PICKIT3).

Enjoy!
