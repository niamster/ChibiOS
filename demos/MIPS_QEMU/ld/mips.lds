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

/* New toolchain */
/* OUTPUT_FORMAT("elf32-bigmips", "elf32-bigmips", "elf32-bigmips") */

/* Old toolchain */
OUTPUT_FORMAT("elf32-tradbigmips", "elf32-tradbigmips", "elf32-tradlittlemips")

OUTPUT_ARCH(mips)
ENTRY(_start)
SECTIONS
{
  . = 0x80000000;

  . = ALIGN(32);
  .text : {
    KEEP (*(.entry*))
    ASSERT(. < 0x180, "entry point too long");
    . = . + 0x100; . = ALIGN(0x80); KEEP (*(.vectors*))
    KEEP (*(.context*))
    *(.text*)
    *(.gcc*)
    *(.ctors)
    *(.dtors)
  }

  . = ALIGN(32);
  .data : {
    __data_start__ = .;
    . = .;
    *(.rodata*)
    *(.data*)
    *(.sdata*)
    __data_end__ = .;
  }

  . = ALIGN(32);
  .bss : { 
    __bss_start__ = .;
    *(.sbss*)
    *(.bss*)
    __bss_end__ = .;
  }

  . = ALIGN(32);
  __heap_base__ = .;
}
