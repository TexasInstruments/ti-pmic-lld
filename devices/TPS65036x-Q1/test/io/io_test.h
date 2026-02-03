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

/**
 * @file io_test.h
 * @brief Contains macros/defines and test declarations specific to testing the
 * IO module.
 */

/* ========================================================================== */
/*                              Include Files                                 */
/* ========================================================================== */

#include "platform.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/* ======================================================================== */
/*         Test APIs: ioTxByte, ioTxByte_CS, ioRxByte, ioRxByte_CS          */
/* ======================================================================== */
#define IO_TEST_POS_IOTXRXBYTE() \
    PLATFORM_RUN_TEST(test_pos_io_ioTxByte_ioRxByte_writeReadScratchpadReg1To4); \
    PLATFORM_RUN_TEST(test_pos_io_ioTxByte_CS_ioRxByte_CS_writeReadScratchpadReg1To4)

#define IO_TEST_NEG_IOTXRXBYTE() \
    PLATFORM_RUN_TEST(test_neg_io_ioTxByte_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_io_ioRxByte_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_io_ioRxByte_nullRxBuffer); \
    PLATFORM_RUN_TEST(test_neg_io_ioTxByte_CS_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_io_ioRxByte_CS_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_io_ioRxByte_CS_nullRxBuffer); \
    PLATFORM_RUN_TEST(test_neg_io_ioTxByte_nullCommHandle); \
    PLATFORM_RUN_TEST(test_neg_io_ioRxByte_nullCommHandle)

/* Test: TC-IO-0029 */
#define IO_TEST_IOTXRXBYTE() \
    IO_TEST_NEG_IOTXRXBYTE(); \
    IO_TEST_POS_IOTXRXBYTE()

/* ======================================================================== */
/*      Test APIs: Pmic_ioUpdateByte, ioUpdateByte_CS, ioUpdateByte_b,      */
/*                 ioUpdateByte_bCS                                         */
/* ======================================================================== */
#define IO_TEST_POS_IOUPDATEBYTE() \
    PLATFORM_RUN_TEST(test_pos_io_ioUpdateByte_modifyBitFields); \
    PLATFORM_RUN_TEST(test_pos_io_ioUpdateByte_CS_modifyBitFields); \
    PLATFORM_RUN_TEST(test_pos_io_ioUpdateByte_b_modifySingleBit); \
    PLATFORM_RUN_TEST(test_pos_io_ioUpdateByte_bCS_modifySingleBit)

#define IO_TEST_NEG_IOUPDATEBYTE() \
    PLATFORM_RUN_TEST(test_neg_io_ioUpdateByte_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_io_ioUpdateByte_CS_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_io_ioUpdateByte_b_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_io_ioUpdateByte_bCS_nullHandle)

/* Test: TC-IO-0030 */
#define IO_TEST_IOUPDATEBYTE() \
    IO_TEST_NEG_IOUPDATEBYTE(); \
    IO_TEST_POS_IOUPDATEBYTE()

/* ======================================================================== */
/*      Test APIs: ioSetCrcEnableState, ioGetCrcEnableState, ioTxByte,      */
/*                 ioTxByte_CS, ioRxByte, ioRxByte_CS, ioUpdateByte,        */
/*                 ioCrcEnable, ioCrcDisable                                */
/* ======================================================================== */
#define IO_TEST_POS_CRC() \
    PLATFORM_RUN_TEST(test_pos_io_setGetCrcEnableState); \
    PLATFORM_RUN_TEST(test_pos_io_enableDisableCrc); \
    PLATFORM_RUN_TEST(test_pos_io_ioTxRxByte_withCrcEnabled); \
    PLATFORM_RUN_TEST(test_pos_io_ioTxRxByte_CS_withCrcEnabled); \
    PLATFORM_RUN_TEST(test_pos_io_ioUpdateByte_withCrcEnabled)

#define IO_TEST_NEG_CRC() \
    PLATFORM_RUN_TEST(test_neg_io_ioGetCrcEnableState_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_io_ioGetCrcEnableState_nullIsEnabled); \
    PLATFORM_RUN_TEST(test_neg_io_ioSetCrcEnableState_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_io_ioCrcEnable_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_io_ioCrcDisable_nullHandle)

/* Test: TC-IO-0031 */
#define IO_TEST_CRC() \
    IO_TEST_NEG_CRC(); \
    IO_TEST_POS_CRC()

/* ======================================================================== */
/*                      Test APIs: ioTxByte, ioRxByte                       */
/* ======================================================================== */
#define IO_TEST_POS_REVISION() \
    PLATFORM_RUN_TEST(test_pos_io_ioTxRxByte_A0_revisionMapping); \
    PLATFORM_RUN_TEST(test_pos_io_ioTxRxByte_B0_revisionMapping); \
    PLATFORM_RUN_TEST(test_pos_io_a0RevisionMapping)

#define IO_TEST_NEG_REVISION()

/* Test: TC-IO-0032 */
#define IO_TEST_REVISION() \
    IO_TEST_NEG_REVISION(); \
    IO_TEST_POS_REVISION()

/* ======================================================================== */
/*                      Test APIs: ioTxByte, ioRxByte                       */
/* ======================================================================== */
#define IO_TEST_POS_RETRY() \
    PLATFORM_RUN_TEST(test_pos_io_ioRxByte_withRetryOnCrcError); \
    PLATFORM_RUN_TEST(test_pos_io_ioTxByte_withRetryOnFailure); \
    PLATFORM_RUN_TEST(test_pos_io_ioTxByte_retrySucceedsOnLastAttempt); \
    PLATFORM_RUN_TEST(test_pos_io_ioTxByte_multipleRetryAttempts)

#define IO_TEST_NEG_RETRY() \
    PLATFORM_RUN_TEST(test_neg_io_crcErrorExhaustsRetries); \
    PLATFORM_RUN_TEST(test_neg_io_ioRxByte_zeroRetryCntImmediateFail); \
    PLATFORM_RUN_TEST(test_neg_io_nullTimerWithRetry)

/* Test: TC-IO-0033 */
#define IO_TEST_RETRY() \
    IO_TEST_NEG_RETRY(); \
    IO_TEST_POS_RETRY()

/* ======================================================================== */
/*                      Test APIs: ioTxByte, ioRxByte                       */
/* ======================================================================== */
#define IO_TEST_POS_COVERAGE()

#define IO_TEST_NEG_COVERAGE() \
    PLATFORM_RUN_TEST(test_neg_io_nullIoWriteFunc); \
    PLATFORM_RUN_TEST(test_neg_io_nullIoRead)

/* Test: TC-IO-0034 */
#define IO_TEST_COVERAGE() \
    IO_TEST_NEG_COVERAGE()

/* ========================================================================== */
/*                     Aggregate Test Macros                                  */
/* ========================================================================== */

/* Run all IO positive tests */
#define IO_TEST_RUN_POSITIVE() \
    IO_TEST_POS_IOTXRXBYTE(); \
    IO_TEST_POS_IOUPDATEBYTE(); \
    IO_TEST_POS_CRC(); \
    IO_TEST_POS_REVISION(); \
    IO_TEST_POS_RETRY(); \
    IO_TEST_POS_COVERAGE()

/* Run all IO negative tests */
#define IO_TEST_RUN_NEGATIVE() \
    IO_TEST_NEG_IOTXRXBYTE(); \
    IO_TEST_NEG_IOUPDATEBYTE(); \
    IO_TEST_NEG_CRC(); \
    IO_TEST_NEG_REVISION(); \
    IO_TEST_NEG_RETRY(); \
    IO_TEST_NEG_COVERAGE()

/* Run all IO tests */
#define IO_TEST_RUN_ALL() \
    IO_TEST_IOTXRXBYTE(); \
    IO_TEST_IOUPDATEBYTE(); \
    IO_TEST_CRC(); \
    IO_TEST_REVISION(); \
    IO_TEST_RETRY(); \
    IO_TEST_COVERAGE()

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

void io_test(void *args);

/* Negative tests - Pmic_ioTxByte / Pmic_ioRxByte */
void test_neg_io_ioTxByte_nullHandle(void);
void test_neg_io_ioRxByte_nullHandle(void);
void test_neg_io_ioRxByte_nullRxBuffer(void);
void test_neg_io_ioTxByte_CS_nullHandle(void);
void test_neg_io_ioRxByte_CS_nullHandle(void);
void test_neg_io_ioRxByte_CS_nullRxBuffer(void);
void test_neg_io_ioTxByte_nullCommHandle(void);
void test_neg_io_ioRxByte_nullCommHandle(void);

/* Negative tests - Pmic_ioUpdateByte */
void test_neg_io_ioUpdateByte_nullHandle(void);
void test_neg_io_ioUpdateByte_CS_nullHandle(void);
void test_neg_io_ioUpdateByte_b_nullHandle(void);
void test_neg_io_ioUpdateByte_bCS_nullHandle(void);

/* Negative tests - CRC */
void test_neg_io_ioGetCrcEnableState_nullHandle(void);
void test_neg_io_ioGetCrcEnableState_nullIsEnabled(void);
void test_neg_io_ioSetCrcEnableState_nullHandle(void);
void test_neg_io_ioCrcEnable_nullHandle(void);
void test_neg_io_ioCrcDisable_nullHandle(void);

/* Positive tests - Basic read/write operations */
void test_pos_io_ioTxByte_ioRxByte_writeReadScratchpadReg1To4(void);
void test_pos_io_ioTxByte_CS_ioRxByte_CS_writeReadScratchpadReg1To4(void);
void test_pos_io_ioUpdateByte_modifyBitFields(void);
void test_pos_io_ioUpdateByte_CS_modifyBitFields(void);
void test_pos_io_ioUpdateByte_b_modifySingleBit(void);
void test_pos_io_ioUpdateByte_bCS_modifySingleBit(void);

/* Positive tests - CRC operations */
void test_pos_io_setGetCrcEnableState(void);
void test_pos_io_enableDisableCrc(void);
void test_pos_io_ioTxRxByte_withCrcEnabled(void);
void test_pos_io_ioTxRxByte_CS_withCrcEnabled(void);
void test_pos_io_ioUpdateByte_withCrcEnabled(void);

/* Positive tests - Silicon revision handling */
void test_pos_io_ioTxRxByte_A0_revisionMapping(void);
void test_pos_io_ioTxRxByte_B0_revisionMapping(void);

/* Retry loop and CRC error tests */
void test_pos_io_ioRxByte_withRetryOnCrcError(void);
void test_pos_io_ioTxByte_withRetryOnFailure(void);
void test_neg_io_crcErrorExhaustsRetries(void);

/* Additional coverage tests */
void test_neg_io_nullTimerWithRetry(void);
void test_pos_io_a0RevisionMapping(void);
void test_neg_io_nullIoWriteFunc(void);
void test_neg_io_nullIoRead(void);
void test_pos_io_ioTxByte_retrySucceedsOnLastAttempt(void);
void test_neg_io_ioRxByte_zeroRetryCntImmediateFail(void);
void test_pos_io_ioTxByte_multipleRetryAttempts(void);


#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /*__IO_TEST_H__*/
