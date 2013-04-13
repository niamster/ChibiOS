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

#ifndef _VECTORS_INC_
#define _VECTORS_INC_

  .macro	isr_save_ctx
  /* FIXME: do via offsets of real structure */
  subu    $sp, $sp, 88        /* sizeof(struct extctx) */

  .set noat
  sw      $at, 0  ($sp)
  sw      $v0, 4  ($sp)
  sw      $v1, 8  ($sp)
  sw      $a0, 12 ($sp)
  sw      $a1, 16 ($sp)
  sw      $a2, 20 ($sp)
  sw      $a3, 24 ($sp)
  sw      $t0, 28 ($sp)
  sw      $t1, 32 ($sp)
  sw      $t2, 36 ($sp)
  sw      $t3, 40 ($sp)
  sw      $t4, 44 ($sp)
  sw      $t5, 48 ($sp)
  sw      $t6, 52 ($sp)
  sw      $t7, 56 ($sp)
  sw      $t8, 60 ($sp)
  sw      $t9, 64 ($sp)
  sw      $fp, 68 ($sp)
  sw      $ra, 72 ($sp)
  .set at

  mfhi    $t0
  mflo    $t1
  sw      $t0, 76 ($sp)
  sw      $t1, 80 ($sp)
  .endm

#endif
