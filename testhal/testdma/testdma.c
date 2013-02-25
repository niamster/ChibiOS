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

/**
 * @file    testdma.c
 * @brief   DMA tests support code.
 *
 * @addtogroup test
 * @{
 */

#include "ch.h"
#include "hal.h"
#include "chprintf.h"

#include "testdma.h"

typedef struct {
  dmaTransaction tr;
  Thread *th;
  BaseSequentialStream *chp;
} dmaPriv;

typedef struct {
  dmaTransaction tr;
  Thread *th;
  dmaChannel *chan;
  size_t n;
} dmaSGPriv;

static uint8_t src[TEST_DMA_BUF_SIZE];
static uint8_t dst[TEST_DMA_BUF_SIZE];

static bool_t testDmaDo0(dmaChannel *chan, BaseSequentialStream *chp, dmatrstate_t (*doDmaTrasaction)(dmaChannel *, dmaptr_t, dmaptr_t, size_t)) {
  const int n = TEST_DMA_BUF_SIZE;
  dmatrstate_t status;
  int i;
  dmaptr_t d = dmaMap(dst + 3), s = dmaMap(src);

  for (i=0;i<n;++i) {
    src[i] = 'a';
    dst[i] = 'z';
  }

  status = doDmaTrasaction(chan, d, s, n-6);

  dmaUnmap(s);
  dmaUnmap(d);

  if (status != DMA_TRANSACTION_SUCCESSFULL) {
    chprintf(chp, "transaction not successfull(%d)", status);
    return FALSE;
  }
  if (!(dst[0] == 'z' && dst[1] == 'z' && dst[2] == 'z')) {
    chprintf(chp, "corrupted beginning of the buffer");
    return FALSE;
  }
  if (!(dst[n-1] == 'z' && dst[n-2] == 'z' && dst[n-3] == 'z')) {
    chprintf(chp, "corrupted tail of the buffer");
    return FALSE;
  }

  for (i=3;i<n-6;++i)
    if (dst[i] != 'a') {
      chprintf(chp, "corrupted buffer");
      return FALSE;
    }

  return TRUE;
}

static bool_t testDmaSimple(dmaChannel *chan, BaseSequentialStream *chp) {
  return testDmaDo0(chan, chp, dmaStartSimpleTransaction);
}

static bool_t testDmaSimplePolled(dmaChannel *chan, BaseSequentialStream *chp) {
  return testDmaDo0(chan, chp, dmaStartSimplePolledTransaction);
}

static void testDmaSGComplete(struct dmaTransaction *tr) {
  dmaSGPriv *priv = (dmaSGPriv *)tr;

  chSysLockFromIsr();
  priv->n -= tr->n;
  if (DMA_TRANSACTION_SUCCESSFULL == tr->status
      && priv->n) {
    tr->dst += tr->n;
    dmaStartTransactionI(priv->chan, tr);
  } else
    chSchReadyI(priv->th);
  chSysUnlockFromIsr();
}

static bool_t testDmaSG(dmaChannel *chan, BaseSequentialStream *chp) {
  const int n = TEST_DMA_BUF_SIZE;
  int i;
  dmaSGPriv tr = {
    .tr = {
      .src = dmaMap(src),
      .dst = dmaMap(dst + 32),
      .n = 64,
      .status = DMA_TRANSACTION_FAILED,
      .cb = testDmaSGComplete,
    },
    .th = chThdSelf(),
    .chan = chan,
    .n = n - 64,
  };

  for (i=0;i<n;++i) {
    src[i] = 'a';
    dst[i] = 'z';
  }
  for (i=0;i<=64;++i)
    src[i] = 'X';

  chSysLock();
  dmaStartTransaction(chan, (dmaTransaction *)&tr);
  chSchGoSleepS(THD_STATE_SUSPENDED);
  chSysUnlock();

  dmaUnmap(tr.tr.src);
  dmaUnmap(tr.tr.dst);

  if (tr.tr.status != DMA_TRANSACTION_SUCCESSFULL) {
    chprintf(chp, "transaction not successfull(%d)", tr.tr.status);
    return FALSE;
  }
  for (i=0;i<32;++i)
    if (dst[i] != 'z') {
      chprintf(chp, "corrupted beginning of the buffer");
      return FALSE;
    }
  for (;i<n-32;++i)
    if (dst[i] != 'X') {
      chprintf(chp, "corrupted buffer");
      return FALSE;
    }
  for (;i<n;++i)
    if (dst[i] != 'z') {
      chprintf(chp, "corrupted tail of the buffer");
      return FALSE;
    }

  return TRUE;
}

static bool_t testDmaFifoStatus;

static void testDmaDstFifoComplete(struct dmaTransaction *tr) {
  dmaPriv *priv = (dmaPriv *)tr;
  const int n = TEST_DMA_BUF_SIZE;
  BaseSequentialStream *chp = priv->chp;
  int i;

  chSysLockFromIsr();
  testDmaFifoStatus = TRUE;

  if (tr->status != DMA_TRANSACTION_SUCCESSFULL) {
    chprintf(chp, "transaction not successfull(%d)", tr->status);
    testDmaFifoStatus = FALSE;
  }
  if (testDmaFifoStatus)
    if (dst[0] != 'z') {
      chprintf(chp, "wrong symbol at destination(%c)", dst[0]);
      testDmaFifoStatus = FALSE;
    }
  if (testDmaFifoStatus)
    for (i=1;i<n;++i)
      if (dst[i] != 'A') {
        chprintf(chp, "corrupted buffer");
        testDmaFifoStatus = FALSE;
        break;
      }

  chSchReadyI(priv->th);
  chSysUnlockFromIsr();
}

static bool_t testDmaDstFifo(dmaChannel *chan, BaseSequentialStream *chp) {
  const int n = TEST_DMA_BUF_SIZE;
  int i, c;
  dmaPriv tr = {
    .tr = {
      .src = dmaMap(src + 3),
      .dst = dmaMap(dst),
      .n = 'z'-'a'+1,
      .status = DMA_TRANSACTION_FAILED,
      .cb = testDmaDstFifoComplete,
    },
    .th = chThdSelf(),
    .chp = chp,
  };

  for (i=0;i<n;++i)
    dst[i] = 'A';
  for (i=3,c='a';c<='z';++i,++c)
    src[i] = c;

  chSysLock();
  testDmaFifoStatus = FALSE;
  dmaStartTransaction(chan, (dmaTransaction *)&tr);
  chSchGoSleepS(THD_STATE_SUSPENDED);
  chSysUnlock();

  dmaUnmap(tr.tr.src);
  dmaUnmap(tr.tr.dst);

  return testDmaFifoStatus;
}

static void testDmaSrcFifoComplete(struct dmaTransaction *tr) {
  dmaPriv *priv = (dmaPriv *)tr;
  const int n = TEST_DMA_BUF_SIZE;
  BaseSequentialStream *chp = priv->chp;
  int i, c;

  chSysLockFromIsr();

  testDmaFifoStatus = TRUE;
  if (tr->status != DMA_TRANSACTION_SUCCESSFULL) {
    chprintf(chp, "transaction not successfull(%d)", tr->status);
    testDmaFifoStatus = FALSE;
  }
  if (testDmaFifoStatus)
    if (!(dst[0] == 'A' && dst[1] == 'A' && dst[2] == 'A')) {
      chprintf(chp, "corrupted beginning of the buffer");
      testDmaFifoStatus = FALSE;
    }
  if (testDmaFifoStatus)
    for (i=3,c='a';c<='z';++i,++c)
      if (dst[i] != 'a') {
        chprintf(chp, "corrupted buffer");
        testDmaFifoStatus = FALSE;
        break;
    }
  if (testDmaFifoStatus)
    for (;i<n;++i)
      if (dst[i] != 'A') {
        chprintf(chp, "corrupted tail of the buffer");
        testDmaFifoStatus = FALSE;
        break;
      }

  chSchReadyI(priv->th);
  chSysUnlockFromIsr();
}

static bool_t testDmaSrcFifo(dmaChannel *chan, BaseSequentialStream *chp) {
  const int n = TEST_DMA_BUF_SIZE;
  int i, c;
  dmaPriv tr = {
    .tr = {
      .src = dmaMap(src),
      .dst = dmaMap(dst + 3),
      .n = 'z'-'a'+1,
      .status = DMA_TRANSACTION_FAILED,
      .cb = testDmaSrcFifoComplete,
    },
    .th = chThdSelf(),
    .chp = chp,
  };

  for (i=0;i<n;++i)
    dst[i] = 'A';
  for (i=0,c='a';c<='z';++i,++c)
    src[i] = c;

  chSysLock();
  testDmaFifoStatus = FALSE;
  dmaStartTransaction(chan, (dmaTransaction *)&tr);
  chSchGoSleepS(THD_STATE_SUSPENDED);
  chSysUnlock();

  dmaUnmap(tr.tr.src);
  dmaUnmap(tr.tr.dst);

  return testDmaFifoStatus;
}

struct {
  char *name;
  enum dmaTestType type;
  bool_t (*f)(dmaChannel *, BaseSequentialStream *);
} dmaTests[] = {
  {"Simple", DMA_TEST_MEM_TO_MEM, testDmaSimple},
  {"Simple polled", DMA_TEST_MEM_TO_MEM, testDmaSimplePolled},
  {"S/G", DMA_TEST_MEM_TO_MEM, testDmaSG},

  {"Simple", DMA_TEST_FIFO_TO_MEM, testDmaSrcFifo},

  {"Simple", DMA_TEST_MEM_TO_FIFO, testDmaDstFifo},

  {NULL, 0, NULL}
};

void TestDma(dmaChannel *chan, enum dmaTestType type, BaseSequentialStream *chp) {
  int i, n;
  const char *mode[] = {
    [DMA_TEST_MEM_TO_MEM]   = "mem-to-mem",
    [DMA_TEST_FIFO_TO_MEM]  = "fifo-to-mem",
    [DMA_TEST_MEM_TO_FIFO]  = "mem-to-fifo",
  };

  chprintf(chp, "DMA test suite: %s\r\n", mode[type]);

  for (i=0,n=1;;++i) {
    if (!dmaTests[i].name)
      break;
    if (dmaTests[i].type != type)
      continue;

    chprintf(chp, "Test #%d(%s):", n, dmaTests[i].name);
    if (dmaTests[i].f(chan, chp))
      chprintf(chp, "SUCCESS\r\n");
    else
      chprintf(chp, ":FAILURE\r\n");

    ++n;
  }
}


/** @} */
