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

OUTPUT_FORMAT("elf32-tradlittlemips")

OUTPUT_ARCH(mips)
ENTRY(_start)
PROVIDE(__heap_end__ = 0xA001FFFF);
MEMORY
{
  kseg0_boot_mem        (rx) : ORIGIN = 0x9FC00000, LENGTH = 0x2FF0
  kseg1_boot_mem        (rx) : ORIGIN = 0xBFC00000, LENGTH = 0x2FF0

  kseg0_program_mem     (rx) : ORIGIN = 0x9D000000, LENGTH = 512K
  kseg1_data_mem             : ORIGIN = 0xA0000000, LENGTH = 128K

  devcfg3                    : ORIGIN = 0xBFC02FF0, LENGTH = 0x4
  devcfg2                    : ORIGIN = 0xBFC02FF4, LENGTH = 0x4
  devcfg1                    : ORIGIN = 0xBFC02FF8, LENGTH = 0x4
  devcfg0                    : ORIGIN = 0xBFC02FFC, LENGTH = 0x4

  sfrs                       : ORIGIN = 0xBF800000, LENGTH = 0x100000

  exception_mem              : ORIGIN = 0x9D000000, LENGTH = 0x400
}
SECTIONS
{
  .devcfg3 : {
    KEEP (*(.devcfg3))
  } > devcfg3
  .devcfg2 : {
    KEEP (*(.devcfg2))
  } > devcfg2
  .devcfg1 : {
    KEEP (*(.devcfg1))
  } > devcfg1
  .devcfg0 : {
    KEEP (*(.devcfg0))
  } > devcfg0

  .boot : {
    __init_start__ = .;
    KEEP (*(.entry*))
    KEEP (*(.core.entry*))
    __init_end__ = .;
  } > kseg1_boot_mem

  .rom_data ALIGN(32) : {
    __rom_data_start__ = .;
  } > kseg1_boot_mem

  .text : {
    . = 0x180;
    ASSERT(. == 0x180, "wrong location of vectors");
    __vectors_start__ = .;
    KEEP (*(.evectors*))
    . = 0x200;
    ASSERT(. == 0x200, "wrong location of vectors");
    KEEP (*(.ivectors*))
    KEEP (*(.vectors*))
    __vectors_end__ = .;

    . = ALIGN(32);
    __text_start__ = .;
    *(.text*)
    *(.ctors)
    . = ALIGN(32);
    __text_end__ = .;
  } > kseg0_program_mem

  .rodata ALIGN(32) : {
    __rodata_start__ = .;
    *(.rodata*)
    . = ALIGN(32);
    __rodata_end__ = .;
  } > kseg0_program_mem

  .data : AT((LOADADDR(.rom_data) + SIZEOF(.rom_data) + 31) & ~(31)) /* LMA */ {
    __ram_data_start__ = .;
    *(.data*)
    *(.sdata*)
    . = ALIGN(32);
    __ram_data_end__ = .;
  } > kseg1_data_mem

  .bss ALIGN(32) : {
    __bss_start__ = .;
    *(.bss*)
    *(.sbss*)
    *(COMMON)
    . = ALIGN(32);
    __bss_end__ = .;
  } > kseg1_data_mem

  .heap ALIGN(32) : {
    __heap_base__ = .;
  } > kseg1_data_mem

  /DISCARD/ : {
    *(.rel.dyn);
    *(.reginfo)
    *(.discard);
  }
}
