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

There are several ways to boot ChibiOS in QEMU: from ROM or from RAM.

* To boot from ROM you build ch.elf using mips_rom.lds linker script,
  put ch.bin in the begining of "flash" and launch QEMU to boot from it:
   > dd of=flash bs=1k count=4k if=/dev/zero
   > dd bs=1k  conv=notrunc of=flash if=build/ch.bin
   > qemu-system-mips -cpu 24Kc -M mips -pflash flash -monitor null -nographic

* To boot from RAM you need some external loader.
  u-boot can run in QEMU and it can load ELF. Following steps
  describe the whole process.

   1. Build u-boot for MIPS QEMU and put it on "flash":
    > make QEMU_mips_config CROSS_COMPILE=mips-sde-elf-
    > make all CROSS_COMPILE=mips-sde-elf-
    > dd of=flash bs=1k count=4k if=/dev/zero
    > dd of=flash bs=1k conv=notrunc if=u-boot.bin
   2. Build ch.elf using mips_ram.lds linker script
      and put chibi.elf.strip at second MiB of "flash" supplied to QEMU:
    > dd bs=1k seek=2k conv=notrunc of=flash if=build/ch.elf.strip
   3. Run QEMU and boot from "flash":
    > qemu-system-mips -cpu 24Kc -M mips -pflash flash -monitor null -nographic
   4. Once you have u-boot prompt load and run Chibi ELF:
    # bootelf -s 0xBFE00000


If you have any issues running u-boot from QEMU(it didn't work for me out of box),
you can check out a working branch QEMU-mips at github: https://github.com/niamster/u-boot.git

