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
