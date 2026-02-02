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

#include "platform.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* ========================================================================== */
/*                 API-Specific Test Macros - ioCrcDisable                    */
/* ========================================================================== */

#define IO_TEST_POS_IOCRCDISABLE() \
    PLATFORM_RUN_TEST(test_pos_io_enableDisableCrc)

#define IO_TEST_NEG_IOCRCDISABLE() \
    PLATFORM_RUN_TEST(test_neg_io_ioCrcDisable_nullHandle)

#define IO_TEST_IOCRCDISABLE() \
    IO_TEST_POS_IOCRCDISABLE(); \
    IO_TEST_NEG_IOCRCDISABLE()

/* ========================================================================== */
/*                 API-Specific Test Macros - ioCrcEnable                     */
/* ========================================================================== */

#define IO_TEST_POS_IOCRCENABLE() \
    PLATFORM_RUN_TEST(test_pos_io_enableDisableCrc)

#define IO_TEST_NEG_IOCRCENABLE() \
    PLATFORM_RUN_TEST(test_neg_io_ioCrcEnable_nullHandle)

#define IO_TEST_IOCRCENABLE() \
    IO_TEST_POS_IOCRCENABLE(); \
    IO_TEST_NEG_IOCRCENABLE()

/* ========================================================================== */
/*              API-Specific Test Macros - ioGetCrcEnableState                */
/* ========================================================================== */

#define IO_TEST_POS_IOGETCRCENABLESTATE() \
    PLATFORM_RUN_TEST(test_pos_io_setGetCrcEnableState)

#define IO_TEST_NEG_IOGETCRCENABLESTATE() \
    PLATFORM_RUN_TEST(test_neg_io_ioGetCrcEnableState_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_io_ioGetCrcEnableState_nullIsEnabled)

#define IO_TEST_IOGETCRCENABLESTATE() \
    IO_TEST_POS_IOGETCRCENABLESTATE(); \
    IO_TEST_NEG_IOGETCRCENABLESTATE()

/* ========================================================================== */
/*                 API-Specific Test Macros - ioRxByte                        */
/* ========================================================================== */

#define IO_TEST_POS_IORXBYTE() \
    PLATFORM_RUN_TEST(test_pos_io_ioRxByte_withRetryOnCrcError); \
    PLATFORM_RUN_TEST(test_pos_io_ioTxByte_ioRxByte_writeReadScratchpadReg1To4); \
    PLATFORM_RUN_TEST(test_pos_io_operationsAllPages); \
    PLATFORM_RUN_TEST(test_pos_io_readWithCrcValidation)

#define IO_TEST_NEG_IORXBYTE() \
    PLATFORM_RUN_TEST(test_neg_io_crcErrorExhaustsRetries); \
    PLATFORM_RUN_TEST(test_neg_io_ioRxByte_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_io_ioRxByte_nullRxBuffer); \
    PLATFORM_RUN_TEST(test_neg_io_ioRxByte_zeroRetryCntImmediateFail); \
    PLATFORM_RUN_TEST(test_neg_io_readWithCrcError)

#define IO_TEST_IORXBYTE() \
    IO_TEST_POS_IORXBYTE(); \
    IO_TEST_NEG_IORXBYTE()

/* ========================================================================== */
/*                API-Specific Test Macros - ioRxByte_CS                      */
/* ========================================================================== */

#define IO_TEST_POS_IORXBYTE_CS() \
    PLATFORM_RUN_TEST(test_pos_io_ioTxByte_CS_ioRxByte_CS_writeReadScratchpadReg1To4)

#define IO_TEST_NEG_IORXBYTE_CS() \
    PLATFORM_RUN_TEST(test_neg_io_ioRxByte_CS_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_io_ioRxByte_CS_nullRxBuffer)

#define IO_TEST_IORXBYTE_CS() \
    IO_TEST_POS_IORXBYTE_CS(); \
    IO_TEST_NEG_IORXBYTE_CS()

/* ========================================================================== */
/*              API-Specific Test Macros - ioSetCrcEnableState                */
/* ========================================================================== */

#define IO_TEST_POS_IOSETCRCENABLESTATE() \
    PLATFORM_RUN_TEST(test_pos_io_setCrcStateErrorHandling); \
    PLATFORM_RUN_TEST(test_pos_io_setGetCrcEnableState)

#define IO_TEST_NEG_IOSETCRCENABLESTATE() \
    PLATFORM_RUN_TEST(test_neg_io_ioSetCrcEnableState_nullHandle)

#define IO_TEST_IOSETCRCENABLESTATE() \
    IO_TEST_POS_IOSETCRCENABLESTATE(); \
    IO_TEST_NEG_IOSETCRCENABLESTATE()

/* ========================================================================== */
/*                 API-Specific Test Macros - ioTxByte                        */
/* ========================================================================== */

#define IO_TEST_POS_IOTXBYTE() \
    PLATFORM_RUN_TEST(test_pos_io_crcEnableDisableTransitions); \
    PLATFORM_RUN_TEST(test_pos_io_ioTxByte_ioRxByte_writeReadScratchpadReg1To4); \
    PLATFORM_RUN_TEST(test_pos_io_ioTxByte_multipleRetryAttempts); \
    PLATFORM_RUN_TEST(test_pos_io_ioTxByte_retrySucceedsOnLastAttempt); \
    PLATFORM_RUN_TEST(test_pos_io_ioTxByte_withRetryOnFailure); \
    PLATFORM_RUN_TEST(test_pos_io_operationsAllPages); \
    PLATFORM_RUN_TEST(test_pos_io_writeWithCrcCalculation)

#define IO_TEST_NEG_IOTXBYTE() \
    PLATFORM_RUN_TEST(test_neg_io_ioTxByte_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_io_ioTxByte_nullIoWrite)

#define IO_TEST_IOTXBYTE() \
    IO_TEST_POS_IOTXBYTE(); \
    IO_TEST_NEG_IOTXBYTE()

/* ========================================================================== */
/*                API-Specific Test Macros - ioTxByte_CS                      */
/* ========================================================================== */

#define IO_TEST_POS_IOTXBYTE_CS() \
    PLATFORM_RUN_TEST(test_pos_io_ioTxByte_CS_ioRxByte_CS_writeReadScratchpadReg1To4)

#define IO_TEST_NEG_IOTXBYTE_CS() \
    PLATFORM_RUN_TEST(test_neg_io_ioTxByte_CS_nullHandle)

#define IO_TEST_IOTXBYTE_CS() \
    IO_TEST_POS_IOTXBYTE_CS(); \
    IO_TEST_NEG_IOTXBYTE_CS()

/* ================================================================================== */
/* API-Specific Test Macros - ioRxByte, ioUpdateByte, ioUpdateByte_b, ioUpdateByte_CS */
/* ================================================================================== */

#define IO_TEST_POS_IOUPDATEBYTE() \
    PLATFORM_RUN_TEST(test_pos_io_updateByte_basic); \
    PLATFORM_RUN_TEST(test_pos_io_updateByte_booleanBit); \
    PLATFORM_RUN_TEST(test_pos_io_updateByte_booleanWithCS); \
    PLATFORM_RUN_TEST(test_pos_io_updateByte_withCriticalSection)

#define IO_TEST_NEG_IOUPDATEBYTE() \
    PLATFORM_RUN_TEST(test_neg_io_ioUpdateByte_nullHandle)

#define IO_TEST_IOUPDATEBYTE() \
    IO_TEST_POS_IOUPDATEBYTE(); \
    IO_TEST_NEG_IOUPDATEBYTE()

/* ========================================================================== */
/*                         Aggregate Test Runners                             */
/* ========================================================================== */

#define IO_TEST_RUN_POSITIVE() \
    IO_TEST_POS_IOCRCDISABLE(); \
    IO_TEST_POS_IOCRCENABLE(); \
    IO_TEST_POS_IOGETCRCENABLESTATE(); \
    IO_TEST_POS_IORXBYTE(); \
    IO_TEST_POS_IORXBYTE_CS(); \
    IO_TEST_POS_IOSETCRCENABLESTATE(); \
    IO_TEST_POS_IOTXBYTE(); \
    IO_TEST_POS_IOTXBYTE_CS(); \
    IO_TEST_POS_IOUPDATEBYTE()

#define IO_TEST_RUN_NEGATIVE() \
    IO_TEST_NEG_IOCRCDISABLE(); \
    IO_TEST_NEG_IOCRCENABLE(); \
    IO_TEST_NEG_IOGETCRCENABLESTATE(); \
    IO_TEST_NEG_IORXBYTE(); \
    IO_TEST_NEG_IORXBYTE_CS(); \
    IO_TEST_NEG_IOSETCRCENABLESTATE(); \
    IO_TEST_NEG_IOTXBYTE(); \
    IO_TEST_NEG_IOTXBYTE_CS(); \
    IO_TEST_NEG_IOUPDATEBYTE()

#define IO_TEST_RUN_ALL() \
    IO_TEST_RUN_POSITIVE(); \
    IO_TEST_RUN_NEGATIVE()

/* PMIC scratchpad register addresses */
#define IO_TEST_SCRATCH_PAD_REG_1_REG (0x0AU)
#define IO_TEST_SCRATCH_PAD_REG_2_REG (0x0BU)
#define IO_TEST_SCRATCH_PAD_REG_3_REG (0x0CU)
#define IO_TEST_SCRATCH_PAD_REG_4_REG (0x0DU)

/* Max PMIC user-space register address */
#define IO_TEST_MAX_REG (0x62U)

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
