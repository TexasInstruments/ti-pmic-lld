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

#include "../platform.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

void io_test(void *args);

/* Negative Tests - NULL Handle */
void test_negative_ioTxByte_nullHandle(void);
void test_negative_ioRxByte_nullHandle(void);
void test_negative_ioTxByte_CS_nullHandle(void);
void test_negative_ioRxByte_CS_nullHandle(void);

/* Negative Tests - NULL Parameters */
void test_negative_ioRxByte_nullRxBuffer(void);
void test_negative_ioRxByte_CS_nullRxBuffer(void);
void test_negative_ioRxWordSeq_nullHandle(void);
void test_negative_ioRxWordSeq_nullRxData(void);
void test_negative_ioTxWordSeq_nullHandle(void);

/* Negative Tests - Invalid Parameters */
void test_negative_ioRxWordSeq_invalidCount(void);
void test_negative_ioTxWordSeq_invalidCount(void);

/* Positive Tests - Single Byte Operations */
void test_positive_ioTxRxByte_scratchpad1(void);
void test_positive_ioTxRxByte_scratchpad2(void);
void test_positive_ioTxRxByte_CS_scratchpad1(void);
void test_positive_ioTxRxByte_CS_scratchpad2(void);

/* Positive Tests - Multi-Byte Sequential Operations */
void test_positive_ioTxRxWordSeq_1byte(void);
void test_positive_ioTxRxWordSeq_2bytes(void);

/* Positive Tests - Read-Modify-Write Operations */
void test_positive_ioUpdateByte_singleBitField(void);
void test_positive_ioUpdateByte_multiBitField(void);
void test_positive_ioUpdateByte_b_setBit(void);
void test_positive_ioUpdateByte_b_clearBit(void);
void test_positive_ioUpdateByte_CS_singleBitField(void);
void test_positive_ioUpdateByte_bCS_setBit(void);

/* Positive Tests - Register Boundaries */
void test_positive_ioTxRxByte_registerBoundaries(void);
void test_positive_ioTxRxByte_allScratchpadRegs(void);

/* Positive Tests - CRC Validation */
void test_positive_io_read_with_crc_validation(void);
void test_negative_io_read_with_crc_error(void);
void test_positive_io_write_with_crc_calculation(void);
void test_positive_io_crc_enable_disable_transitions(void);

/* Retry loop and CRC error tests */
void test_positive_ioRxByte_withRetryOnCrcError(void);
void test_positive_ioTxByte_withRetryOnFailure(void);
void test_negative_io_crcErrorExhaustsRetries(void);
void test_positive_ioTxByte_retrySucceedsOnLastAttempt(void);
void test_negative_ioRxByte_zeroRetryCntImmediateFail(void);
void test_positive_ioTxByte_multipleRetryAttempts(void);

/* Coverage tests for pmic_io.c validation paths */
void test_negative_io_nullCommHandle(void);
void test_negative_io_nullIoFptrs(void);
void test_negative_io_nullTimerWithRetry(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /*__IO_TEST_H__*/
