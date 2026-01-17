/******************************************************************************
 * Copyright (c) 2025 Texas Instruments Incorporated - http://www.ti.com
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *****************************************************************************/
#ifndef IO_TEST_H
#define IO_TEST_H



/* ========================================================================== */
/*                              Include Files                                 */
/* ========================================================================== */

#include "platform.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

void io_test(void *args);

void test_neg_io_ioCrcDisable_nullHandle(void);
void test_neg_io_ioCrcEnable_nullHandle(void);
void test_neg_io_ioGetCrcEnableState_nullHandle(void);
void test_neg_io_ioGetCrcEnableState_nullIsEnabled(void);
void test_neg_io_ioRxByte_CS_nullHandle(void);
void test_neg_io_ioRxByte_CS_nullRxBuffer(void);
void test_neg_io_ioRxByte_nullHandle(void);
void test_neg_io_ioRxByte_nullRxBuffer(void);
void test_neg_io_ioRxByte_zeroRetryCntImmediateFail(void);
void test_neg_io_ioSetCrcEnableState_nullHandle(void);
void test_neg_io_ioTxByte_CS_nullHandle(void);
void test_neg_io_ioTxByte_nullHandle(void);
void test_neg_io_ioTxByte_nullIoWrite(void);
void test_neg_io_ioUpdateByte_nullHandle(void);
void test_neg_io_crcErrorExhaustsRetries(void);
void test_neg_io_readWithCrcError(void);
void test_pos_io_crcEnableDisableTransitions(void);
void test_pos_io_enableDisableCrc(void);
void test_pos_io_ioRxByte_withRetryOnCrcError(void);
void test_pos_io_ioTxByte_CS_ioRxByte_CS_writeReadScratchpadReg1To4(void);
void test_pos_io_ioTxByte_ioRxByte_writeReadScratchpadReg1To4(void);
void test_pos_io_ioTxByte_multipleRetryAttempts(void);
void test_pos_io_ioTxByte_retrySucceedsOnLastAttempt(void);
void test_pos_io_ioTxByte_withRetryOnFailure(void);
void test_pos_io_operationsAllPages(void);
void test_pos_io_readWithCrcValidation(void);
void test_pos_io_setCrcStateErrorHandling(void);
void test_pos_io_setGetCrcEnableState(void);
void test_pos_io_updateByte_basic(void);
void test_pos_io_updateByte_booleanBit(void);
void test_pos_io_updateByte_booleanWithCS(void);
void test_pos_io_updateByte_withCriticalSection(void);
void test_pos_io_writeWithCrcCalculation(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /*__IO_TEST_H__*/
