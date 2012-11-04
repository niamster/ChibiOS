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
PROVIDE(__heap_end__ = 0x8001FFFF);
MEMORY
{
  kseg0_program_mem     (rx) : ORIGIN = 0x9D000000, LENGTH = 0x7FFFF
  kseg0_ram_mem              : ORIGIN = 0x80000000, LENGTH = 0x1FFFF
  kseg1_boot_mem        (rx) : ORIGIN = 0xBFC00000, LENGTH = 0x2FEF
  config3                    : ORIGIN = 0xBFC02FF0, LENGTH = 0x4
  config2                    : ORIGIN = 0xBFC02FF4, LENGTH = 0x4
  config1                    : ORIGIN = 0xBFC02FF8, LENGTH = 0x4
  config0                    : ORIGIN = 0xBFC02FFC, LENGTH = 0x4
  sfrs                       : ORIGIN = 0xBF800000, LENGTH = 0x100000
}
SECTIONS
{
  .config_BFC02FF0 : {
    KEEP (*(.config_BFC02FF0))
  } > config3
  .config_BFC02FF4 : {
    KEEP (*(.config_BFC02FF4))
  } > config2
  .config_BFC02FF8 : {
    KEEP (*(.config_BFC02FF8))
  } > config1
  .config_BFC02FFC : {
    KEEP (*(.config_BFC02FFC))
  } > config0

  .boot : {
    __init_start__ = .;
    KEEP (*(.entry*))
    KEEP (*(.core.entry*))
    __init_end__ = .;
  } > kseg1_boot_mem

  . = 0x9D000000;

  .text : {
    . = 0x180;
    ASSERT(. == 0x180, "wrong location of vectors");
    __vectors_start__ = .;
    KEEP (*(.vectors*))
    __vectors_end__ = .;

    . = ALIGN(32);
    __text_start__ = .;
    KEEP (*(.context*))
    *(.text*)
    *(.gcc*)
    *(.ctors)
    __text_end__ = .;
  } > kseg0_program_mem

  .rodata ALIGN(32) : {
    __rodata_start__ = .;
    *(.rodata*)
    __rodata_end__ = .;
  } > kseg0_program_mem

  .rom_data ALIGN(32) : {
    __rom_data_start__ = .;
  } > kseg0_program_mem

  .data : AT((LOADADDR(.rom_data) + SIZEOF(.rom_data) + 31) & ~(31)) /* LMA */ {
    __ram_data_start__ = .;
    *(.data*)
    *(.sdata*)
    . = ALIGN(4);
    __ram_data_end__ = .;
  } > kseg0_ram_mem

  .bss ALIGN(32) : {
    __bss_start__ = .;
    *(.bss*)
    *(.sbss*)
    *(COMMON)
    . = ALIGN(4);
    __bss_end__ = .;
  } > kseg0_ram_mem

  .heap ALIGN(32) : {
    __heap_base__ = .;
  } > kseg0_ram_mem

  /DISCARD/ : {
    *(.rel.dyn);
  }
}
