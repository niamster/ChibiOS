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
    __ctors_start__ = .;
    KEEP (*(.ctors))
    __ctors_end__ = .;
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
