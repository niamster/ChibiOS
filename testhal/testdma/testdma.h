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
 * @file    testdma.h
 * @brief   DMA tests support header.
 *
 * @addtogroup test
 * @{
 */

#ifndef _TESTDMA_H_
#define _TESTDMA_H_

#ifndef TEST_DMA_BUF_SIZE
#define TEST_DMA_BUF_SIZE 1024
#endif

/**
 * @brief Defines DMA test type.
 */
enum dmaTestType {
  DMA_TEST_MEM_TO_MEM,          /* Test memory-to-memory transactions */
  DMA_TEST_FIFO_TO_MEM,         /* Test fifo-to-memory transactions. Fifo size accessor must be 1 byte wide. */
  DMA_TEST_MEM_TO_FIFO,         /* Test memory-to-fifo transactions. Fifo size accessor must be 1 byte wide. */
};

#ifdef __cplusplus
extern "C" {
#endif
  void TestDma(dmaChannel *chan, enum dmaTestType type, BaseSequentialStream *chp);
#ifdef __cplusplus
}
#endif

#endif /* _TESTDMA_H_ */

/** @} */
