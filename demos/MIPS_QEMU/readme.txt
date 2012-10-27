*****************************************************************************
** ChibiOS/RT port for QEMU MIPS 24kc.                                     **
*****************************************************************************

** TARGET **

The demo runs on a QEMU MIPS 24kc virtual board.

** The Demo **

The demo currently just executes shell with commands to show threads activity,
memory usage and run Chibios test suite.

** Build Procedure **

The demo was built using the official MIPS toolchain(http://developer.mips.com/tools/compilers/bare-metal-toolchain/)
but any toolchain based on GCC and GNU userspace programs will work.

** Notes **

All files used by the demo are part of ChibiOS/RT.

How to run demo with QEMU:
1. You need u-boot as current implementation only works when the code is located in memory
2. Build u-boot for MIPS QEMU and put it on "flash":
   make qemu_mips_config CROSS_COMPILE=mips-sde-elf-
   make all CROSS_COMPILE=mips-sde-elf-
   dd of=flash bs=1k count=4k if=/dev/zero
   dd of=flash bs=1k conv=notrunc if=u-boot.bin
3. Put chibi.elf.strip at second MiB of flash supplied to qemu:
   dd bs=1k seek=2k conv=notrunc of=flash if=build/ch.elf.strip
4. Run QEMU and boot from flash:
   qemu-system-mips -cpu 24Kc -M mips -pflash flash -monitor null -nographic
5. Once you have u-boot prompt load and run Chibi ELF from flash:
   bootelf -s 0xBFE00000


If you have any issues running u-boot from qemu(it didn't work for me out of box),
you can check out a working branch qemu-mips at github: https://github.com/niamster/u-boot.git

