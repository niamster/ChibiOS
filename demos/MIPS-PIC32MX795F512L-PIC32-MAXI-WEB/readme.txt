*****************************************************************************
** ChibiOS/RT port for QEMU MIPS 24kc.                                     **
*****************************************************************************

** TARGET **

The demo runs on a OLIMEX PIC32MX795F512L(MIPS 4kc) board.

** The Demo **

The demo currently just executes shell with commands to show threads activity,
memory usage and run Chibios test suite.

** Build Procedure **

The demo was built using the official MIPS toolchain(http://developer.mips.com/tools/compilers/bare-metal-toolchain/)
and xc32 PIC32 toolchain.
Any toolchain based on GCC and GNU userspace programs will work for the whole system however xc32 is still required to
provide plib(pic peripherals headers and sources).
This dependeny is nasty so it should be removed in the future.

** Notes **

All files used by the demo are part of ChibiOS/RT.

To build the demo you have to provide a full path to XC32 toolchain root by defining XCPATH variable:
export PATH=$PATH:/opt/mips-toolchain/bin/
export XCPATH=/opt/microchip/xc32/v1.11

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

Please note that it's not possible to do this trick with prebuilt image with old MPLAB(not X).

/QA/
Q1: Why not to use xc32 to build the world?
A1: It apppears that free version of xc32 is very slow(seems like they put smth like {if (free) sleep(60);})
    and it has optimization level limitations(max -O1) 
    as well as a limitation on the size of output image(64KB if I'm not mistaken).
    Drawbacks of current dependency on xc32: 
      lots of CC warning for the files where plib is included;
      extra dependency on external headers and sources.
      
