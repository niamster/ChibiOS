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

/* New toolchain */
/* OUTPUT_FORMAT("elf32-bigmips", "elf32-bigmips", "elf32-bigmips") */

/* Old toolchain */
OUTPUT_FORMAT("elf32-tradbigmips", "elf32-tradbigmips", "elf32-tradlittlemips")

OUTPUT_ARCH(mips)
ENTRY(_start)
PROVIDE(__heap_end__ = 0x88000000);
SECTIONS
{
  . = 0x80000000;

  .text : {
    __init_start__ = .;
    KEEP (*(.entry*))
    __init_end__ = .;

    ASSERT(. < 0x180, "entry point too long");
    . = 0x180;
    ASSERT(. == 0x180, "wrong location of vectors");
    __vectors_start__ = .;
    KEEP (*(.vectors*))
    __vectors_end__ = .;

    . = ALIGN(32);
    __text_start__ = .;
    KEEP (*(.core.entry*))
    KEEP (*(.context*))
    *(.text*)
    *(.gcc*)
    *(.ctors)
    __text_end__ = .;
  }

  .rodata ALIGN(32) : {
    __rodata_start__ = .;
    *(.rodata*)
    __rodata_end__ = .;
  }

  .data ALIGN(32) : {
    __rom_data_start__ = .;
    __ram_data_start__ = .;
    *(.data*)
    *(.sdata*)
    . = ALIGN(4);
    __ram_data_end__ = .;
  }

  .bss ALIGN(32) : {
    __bss_start__ = .;
    *(.bss*)
    *(.sbss*)
    *(COMMON)
    . = ALIGN(4);
    __bss_end__ = .;
  }

  . = ALIGN(32);
  __heap_base__ = .;

  /DISCARD/ : {
    *(.rel.dyn);
  }
}
