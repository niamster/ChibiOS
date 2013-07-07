/*
    ChibiOS/RT - Copyright (C) 2006-2013 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/
/*
   Concepts and parts of this file have been contributed by Dmytro Milinevskyy <milinevskyy@gmail.com>
 */

OUTPUT_FORMAT("elf32-littlemips")

OUTPUT_ARCH(mips)
ENTRY(_start)
PROVIDE(__heap_end__ = 0xA001FFFF);
MEMORY
{
  kseg0_boot_mem        (rx) : ORIGIN = 0x9FC00000, LENGTH = 0x2FF0
  kseg1_boot_mem        (rx) : ORIGIN = 0xBFC00000, LENGTH = 0x2FF0

  kseg0_program_mem     (rx) : ORIGIN = 0x9D000000, LENGTH = 512K
  kseg1_data_mem       (w!x) : ORIGIN = 0xA0000000, LENGTH = 128K

  devcfg                 (w) : ORIGIN = 0xBFC02FF0, LENGTH = 0x10

  sfrs                   (w) : ORIGIN = 0xBF800000, LENGTH = 0x100000
}
SECTIONS
{
  .devcfg : {
    KEEP (*(.devcfg3*))
    KEEP (*(.devcfg2*))
    KEEP (*(.devcfg1*))
    KEEP (*(.devcfg0*))
  } > devcfg

  .boot : {
    __init_start__ = .;
    KEEP (*(.entry*))
    KEEP (*(.core.entry*))
    __init_end__ = .;
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

    . = ALIGN(16);
    __text_start__ = .;
    *(.text*)
    __ctors_start__ = .;
    KEEP (*(.ctors))
    __ctors_end__ = .;
    __text_end__ = .;
  } > kseg0_program_mem

  .rodata ALIGN(16) : {
    __rodata_start__ = .;
    *(.rodata*)
    . = ALIGN(4);
    __rodata_end__ = .;
  } > kseg0_program_mem

  .rom_data ALIGN(16) : {
    __rom_data_start__ = .;
  } > kseg0_program_mem

  .data : AT((LOADADDR(.rom_data) + SIZEOF(.rom_data) + 15) & ~(15)) /* LMA */ {
    __ram_data_start__ = .;
    *(.data*)
    *(.sdata*)
    __ram_data_end__ = .;
  } > kseg1_data_mem

  .bss ALIGN(4) : {
    __bss_start__ = .;
    *(.bss*)
    *(.sbss*)
    *(COMMON)
    . = ALIGN(4);
    __bss_end__ = .;
  } > kseg1_data_mem

  .heap : {
    __heap_base__ = .;
  } > kseg1_data_mem

  .gnu.attributes 0 : { KEEP (*(.gnu.attributes)) }
  
  /* Stabs debugging sections.  */
  .stab          0 : { *(.stab) }
  .stabstr       0 : { *(.stabstr) }
  .stab.excl     0 : { *(.stab.excl) }
  .stab.exclstr  0 : { *(.stab.exclstr) }
  .stab.index    0 : { *(.stab.index) }
  .stab.indexstr 0 : { *(.stab.indexstr) }
  .comment       0 : { *(.comment) }

  /* DWARF debug sections.
     Symbols in the DWARF debugging sections are relative to the beginning
     of the section so we begin them at 0.  */
  /* DWARF 1 */
  .debug          0 : { *(.debug) }
  .line           0 : { *(.line) }
  /* GNU DWARF 1 extensions */
  .debug_srcinfo  0 : { *(.debug_srcinfo) }
  .debug_sfnames  0 : { *(.debug_sfnames) }
  /* DWARF 1.1 and DWARF 2 */
  .debug_aranges  0 : { *(.debug_aranges) }
  .debug_pubnames 0 : { *(.debug_pubnames) }
  /* DWARF 2 */
  .debug_info     0 : { *(.debug_info .gnu.linkonce.wi.*) }
  .debug_abbrev   0 : { *(.debug_abbrev) }
  .debug_line     0 : { *(.debug_line) }
  .debug_frame    0 : { *(.debug_frame) }
  .debug_str      0 : { *(.debug_str) }
  .debug_loc      0 : { *(.debug_loc) }
  .debug_macinfo  0 : { *(.debug_macinfo) }
  /* SGI/MIPS DWARF 2 extensions */
  .debug_weaknames 0 : { *(.debug_weaknames) }
  .debug_funcnames 0 : { *(.debug_funcnames) }
  .debug_typenames 0 : { *(.debug_typenames) }
  .debug_varnames  0 : { *(.debug_varnames) }
  .debug_pubtypes 0 : { *(.debug_pubtypes) }
  .debug_ranges   0 : { *(.debug_ranges) }

  /DISCARD/ : {
    *(.rel.dyn)
    *(.reginfo)
    *(.discard)
  }
}
