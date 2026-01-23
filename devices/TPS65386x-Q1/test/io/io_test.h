/******************************************************************************
 * Copyright (c) 2026 Texas Instruments Incorporated - http://www.ti.com
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
/*                    API-Specific Test Macros - ioTxByte                     */
/* ========================================================================== */

#define IO_TEST_POS_IOTXBYTE() \
    PLATFORM_RUN_TEST(test_pos_io_ioTxByte_scratchpad1); \
    PLATFORM_RUN_TEST(test_pos_io_ioTxByte_scratchpad2); \
    PLATFORM_RUN_TEST(test_pos_io_ioTxByte_retrySucceedsOnLastAttempt); \
    PLATFORM_RUN_TEST(test_pos_io_ioTxByte_multipleRetryAttempts)

#define IO_TEST_NEG_IOTXBYTE() \
    PLATFORM_RUN_TEST(test_neg_io_ioTxByte_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_io_ioTxByte_withRetryOnFailure)

#define IO_TEST_IOTXBYTE() \
    IO_TEST_POS_IOTXBYTE(); \
    IO_TEST_NEG_IOTXBYTE()

/* ========================================================================== */
/*                    API-Specific Test Macros - ioRxByte                     */
/* ========================================================================== */

#define IO_TEST_POS_IORXBYTE() \
    PLATFORM_RUN_TEST(test_pos_io_ioRxByte_scratchpad1); \
    PLATFORM_RUN_TEST(test_pos_io_ioRxByte_scratchpad2); \
    PLATFORM_RUN_TEST(test_pos_io_ioRxByte_withRetryOnCrcError)

#define IO_TEST_NEG_IORXBYTE() \
    PLATFORM_RUN_TEST(test_neg_io_ioRxByte_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_io_ioRxByte_nullRxBuffer); \
    PLATFORM_RUN_TEST(test_neg_io_ioRxByte_zeroRetryCntImmediateFail); \
    PLATFORM_RUN_TEST(test_neg_io_ioRxByte_crcError)

#define IO_TEST_IORXBYTE() \
    IO_TEST_POS_IORXBYTE(); \
    IO_TEST_NEG_IORXBYTE()

/* ========================================================================== */
/*                   API-Specific Test Macros - ioTxByte_CS                   */
/* ========================================================================== */

#define IO_TEST_POS_IOTXBYTE_CS() \
    PLATFORM_RUN_TEST(test_pos_io_ioTxByte_CS_scratchpad1); \
    PLATFORM_RUN_TEST(test_pos_io_ioTxByte_CS_scratchpad2)

#define IO_TEST_NEG_IOTXBYTE_CS() \
    PLATFORM_RUN_TEST(test_neg_io_ioTxByte_CS_nullHandle)

#define IO_TEST_IOTXBYTE_CS() \
    IO_TEST_POS_IOTXBYTE_CS(); \
    IO_TEST_NEG_IOTXBYTE_CS()

/* ========================================================================== */
/*                   API-Specific Test Macros - ioRxByte_CS                   */
/* ========================================================================== */

#define IO_TEST_POS_IORXBYTE_CS() \
    PLATFORM_RUN_TEST(test_pos_io_ioRxByte_CS_scratchpad1); \
    PLATFORM_RUN_TEST(test_pos_io_ioRxByte_CS_scratchpad2)

#define IO_TEST_NEG_IORXBYTE_CS() \
    PLATFORM_RUN_TEST(test_neg_io_ioRxByte_CS_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_io_ioRxByte_CS_nullRxBuffer)

#define IO_TEST_IORXBYTE_CS() \
    IO_TEST_POS_IORXBYTE_CS(); \
    IO_TEST_NEG_IORXBYTE_CS()

/* ========================================================================== */
/*                 API-Specific Test Macros - ioTxWordSeq                     */
/* ========================================================================== */

#define IO_TEST_POS_IOTXWORDSEQ() \
    PLATFORM_RUN_TEST(test_pos_io_ioTxRxWordSeq_1byte); \
    PLATFORM_RUN_TEST(test_pos_io_ioTxRxWordSeq_2bytes)

#define IO_TEST_NEG_IOTXWORDSEQ() \
    PLATFORM_RUN_TEST(test_neg_io_ioTxWordSeq_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_io_ioTxWordSeq_invalidCount)

#define IO_TEST_IOTXWORDSEQ() \
    IO_TEST_POS_IOTXWORDSEQ(); \
    IO_TEST_NEG_IOTXWORDSEQ()

/* ========================================================================== */
/*                 API-Specific Test Macros - ioRxWordSeq                     */
/* ========================================================================== */

#define IO_TEST_POS_IORXWORDSEQ() \
    /* Positive tests combined with ioTxWordSeq tests */

#define IO_TEST_NEG_IORXWORDSEQ() \
    PLATFORM_RUN_TEST(test_neg_io_ioRxWordSeq_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_io_ioRxWordSeq_nullRxData); \
    PLATFORM_RUN_TEST(test_neg_io_ioRxWordSeq_invalidCount)

#define IO_TEST_IORXWORDSEQ() \
    IO_TEST_POS_IORXWORDSEQ(); \
    IO_TEST_NEG_IORXWORDSEQ()

/* ========================================================================== */
/*                  API-Specific Test Macros - ioUpdateByte                   */
/* ========================================================================== */

#define IO_TEST_POS_IOUPDATEBYTE() \
    PLATFORM_RUN_TEST(test_pos_io_ioUpdateByte_singleBitField); \
    PLATFORM_RUN_TEST(test_pos_io_ioUpdateByte_multiBitField)

#define IO_TEST_NEG_IOUPDATEBYTE() \
    /* No negative tests for ioUpdateByte */

#define IO_TEST_IOUPDATEBYTE() \
    IO_TEST_POS_IOUPDATEBYTE(); \
    IO_TEST_NEG_IOUPDATEBYTE()

/* ========================================================================== */
/*                         Aggregate Test Runners                             */
/* ========================================================================== */

#define IO_TEST_RUN_POSITIVE() \
    IO_TEST_POS_IOTXBYTE(); \
    IO_TEST_POS_IORXBYTE(); \
    IO_TEST_POS_IOTXBYTE_CS(); \
    IO_TEST_POS_IORXBYTE_CS(); \
    IO_TEST_POS_IOTXWORDSEQ(); \
    IO_TEST_POS_IORXWORDSEQ()

#define IO_TEST_RUN_NEGATIVE() \
    IO_TEST_NEG_IOTXBYTE(); \
    IO_TEST_NEG_IORXBYTE(); \
    IO_TEST_NEG_IOTXBYTE_CS(); \
    IO_TEST_NEG_IORXBYTE_CS(); \
    IO_TEST_NEG_IOTXWORDSEQ(); \
    IO_TEST_NEG_IORXWORDSEQ()

#define IO_TEST_RUN_ALL() \
    IO_TEST_RUN_POSITIVE(); \
    IO_TEST_RUN_NEGATIVE()

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

void io_test(void *args);

/* Negative Tests - ioTxByte */
void test_neg_io_ioTxByte_nullHandle(void);
void test_neg_io_ioTxByte_withRetryOnFailure(void);

/* Negative Tests - ioRxByte */
void test_neg_io_ioRxByte_nullHandle(void);
void test_neg_io_ioRxByte_nullRxBuffer(void);
void test_neg_io_ioRxByte_zeroRetryCntImmediateFail(void);

/* Negative Tests - ioTxByte_CS */
void test_neg_io_ioTxByte_CS_nullHandle(void);

/* Negative Tests - ioRxByte_CS */
void test_neg_io_ioRxByte_CS_nullHandle(void);
void test_neg_io_ioRxByte_CS_nullRxBuffer(void);

/* Negative Tests - ioTxWordSeq */
void test_neg_io_ioTxWordSeq_nullHandle(void);
void test_neg_io_ioTxWordSeq_invalidCount(void);

/* Negative Tests - ioRxWordSeq */
void test_neg_io_ioRxWordSeq_nullHandle(void);
void test_neg_io_ioRxWordSeq_nullRxData(void);
void test_neg_io_ioRxWordSeq_invalidCount(void);

/* Negative Tests - CRC and Retry */
void test_neg_io_ioRxByte_crcError(void);
void test_neg_io_crcErrorExhaustsRetries(void);

/* Negative Tests - Validation Paths */
void test_neg_io_nullCommHandle(void);
void test_neg_io_nullIoFptrs(void);
void test_neg_io_nullTimerWithRetry(void);

/* Positive Tests - ioTxByte */
void test_pos_io_ioTxByte_scratchpad1(void);
void test_pos_io_ioTxByte_scratchpad2(void);
void test_pos_io_ioTxByte_retrySucceedsOnLastAttempt(void);
void test_pos_io_ioTxByte_multipleRetryAttempts(void);

/* Positive Tests - ioRxByte */
void test_pos_io_ioRxByte_scratchpad1(void);
void test_pos_io_ioRxByte_scratchpad2(void);
void test_pos_io_ioRxByte_withRetryOnCrcError(void);

/* Positive Tests - ioTxByte_CS */
void test_pos_io_ioTxByte_CS_scratchpad1(void);
void test_pos_io_ioTxByte_CS_scratchpad2(void);

/* Positive Tests - ioRxByte_CS */
void test_pos_io_ioRxByte_CS_scratchpad1(void);
void test_pos_io_ioRxByte_CS_scratchpad2(void);

/* Positive Tests - ioTxRxWordSeq */
void test_pos_io_ioTxRxWordSeq_1byte(void);
void test_pos_io_ioTxRxWordSeq_2bytes(void);

/* Positive Tests - ioUpdateByte */
void test_pos_io_ioUpdateByte_singleBitField(void);
void test_pos_io_ioUpdateByte_multiBitField(void);

/* Positive Tests - ioUpdateByte_b */
void test_pos_io_ioUpdateByte_b_setBit(void);
void test_pos_io_ioUpdateByte_b_clearBit(void);

/* Positive Tests - ioUpdateByte_CS */
void test_pos_io_ioUpdateByte_CS_singleBitField(void);

/* Positive Tests - ioUpdateByte_bCS */
void test_pos_io_ioUpdateByte_bCS_setBit(void);

/* Positive Tests - Register Boundaries */
void test_pos_io_ioTxRxByte_registerBoundaries(void);
void test_pos_io_ioTxRxByte_allScratchpadRegs(void);

/* Positive Tests - CRC Validation */
void test_pos_io_read_with_crc_validation(void);
void test_pos_io_write_with_crc_calculation(void);
void test_pos_io_crc_enable_disable_transitions(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /*__IO_TEST_H__*/
