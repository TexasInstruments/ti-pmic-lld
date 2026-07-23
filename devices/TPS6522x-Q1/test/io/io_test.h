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
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/* ======================================================================== */
/*                      Test APIs: ioTxByte, ioRxByte                       */
/* ======================================================================== */
#define IO_TEST_POS_IOTXBYTE() \
    PLATFORM_RUN_TEST(test_pos_io_ioTxByte_singleRegisterWrite); \
    PLATFORM_RUN_TEST(test_pos_io_ioTxByte_multipleRegisterAccess); \
    PLATFORM_RUN_TEST(test_pos_io_ioTxByte_writeWithCrcCalculation); \
    PLATFORM_RUN_TEST(test_pos_io_ioTxByte_i2cWriteWithCrc); \
    PLATFORM_RUN_TEST(test_pos_io_ioTxByte_wdgWriteWithCrc); \
    PLATFORM_RUN_TEST(test_pos_io_ioTxByte_asyncWriteSpi); \
    PLATFORM_RUN_TEST(test_pos_io_ioTxByte_asyncWriteI2c); \
    PLATFORM_RUN_TEST(test_pos_io_ioTxByte_withRetryOnFailure); \
    PLATFORM_RUN_TEST(test_pos_io_ioTxByte_retrySucceedsOnLastAttempt); \
    PLATFORM_RUN_TEST(test_pos_io_ioTxByte_multipleRetryAttempts); \
    PLATFORM_RUN_TEST(test_pos_io_ioTxByte_i2cTxRetry)

#define IO_TEST_NEG_IOTXBYTE() \
    PLATFORM_RUN_TEST(test_neg_io_ioTxByte_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_io_ioTxByte_nullIoWrite); \
    PLATFORM_RUN_TEST(test_neg_io_ioTxByte_nullCommHandle); \
    PLATFORM_RUN_TEST(test_neg_io_ioTxByte_nullTimerWithRetry); \
    PLATFORM_RUN_TEST(test_neg_io_ioTxByte_nullAsyncHooks)

/* Test: TC-IO-0008 */
#define IO_TEST_IOTXBYTE() \
    IO_TEST_POS_IOTXBYTE(); \
    IO_TEST_NEG_IOTXBYTE()

/* ======================================================================== */
/*                          Test APIs: ioTxByte_CS                          */
/* ======================================================================== */
#define IO_TEST_POS_IOTXBYTE_CS() \
    PLATFORM_RUN_TEST(test_pos_io_ioTxByte_CS_singleRegisterWrite)

#define IO_TEST_NEG_IOTXBYTE_CS() \
    PLATFORM_RUN_TEST(test_neg_io_ioTxByte_CS_nullHandle)

/* Test: TC-IO-0009 */
#define IO_TEST_IOTXBYTE_CS() \
    IO_TEST_POS_IOTXBYTE_CS(); \
    IO_TEST_NEG_IOTXBYTE_CS()

/* ======================================================================== */
/*                           Test APIs: ioRxByte                            */
/* ======================================================================== */
#define IO_TEST_POS_IORXBYTE() \
    PLATFORM_RUN_TEST(test_pos_io_ioRxByte_singleRegisterRead); \
    PLATFORM_RUN_TEST(test_pos_io_ioRxByte_registerReadVerification); \
    PLATFORM_RUN_TEST(test_pos_io_ioRxByte_readWithCrcValidation); \
    PLATFORM_RUN_TEST(test_pos_io_ioRxByte_wdgReadWithCrc); \
    PLATFORM_RUN_TEST(test_pos_io_ioRxByte_asyncReadSpi); \
    PLATFORM_RUN_TEST(test_pos_io_ioRxByte_asyncReadI2c); \
    PLATFORM_RUN_TEST(test_pos_io_ioRxByte_withRetryOnCrcError); \
    PLATFORM_RUN_TEST(test_pos_io_ioRxByte_i2cRxRetry)

#define IO_TEST_NEG_IORXBYTE() \
    PLATFORM_RUN_TEST(test_neg_io_ioRxByte_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_io_ioRxByte_nullRxData); \
    PLATFORM_RUN_TEST(test_neg_io_ioRxByte_nullIoRead); \
    PLATFORM_RUN_TEST(test_neg_io_ioRxByte_nullCommHandle); \
    PLATFORM_RUN_TEST(test_neg_io_ioRxByte_crcErrorExhaustsRetries); \
    PLATFORM_RUN_TEST(test_neg_io_ioRxByte_zeroRetryCntImmediateFail); \
    PLATFORM_RUN_TEST(test_neg_io_ioRxByte_spiRxCrcMismatch); \
    PLATFORM_RUN_TEST(test_neg_io_ioRxByte_i2cRxCrcMismatch)

/* Test: TC-IO-0010 */
#define IO_TEST_IORXBYTE() \
    IO_TEST_POS_IORXBYTE(); \
    IO_TEST_NEG_IORXBYTE()

/* ======================================================================== */
/*                          Test APIs: ioRxByte_CS                          */
/* ======================================================================== */
#define IO_TEST_POS_IORXBYTE_CS() \
    PLATFORM_RUN_TEST(test_pos_io_ioRxByte_CS_singleRegisterRead)

#define IO_TEST_NEG_IORXBYTE_CS() \
    PLATFORM_RUN_TEST(test_neg_io_ioRxByte_CS_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_io_ioRxByte_CS_nullRxData)

/* Test: TC-IO-0011 */
#define IO_TEST_IORXBYTE_CS() \
    IO_TEST_POS_IORXBYTE_CS(); \
    IO_TEST_NEG_IORXBYTE_CS()

/* ======================================================================== */
/*                         Test APIs: ioUpdateByte                          */
/* ======================================================================== */
#define IO_TEST_POS_IOUPDATEBYTE() \
    PLATFORM_RUN_TEST(test_pos_io_ioUpdateByte_readModifyWrite)

#define IO_TEST_NEG_IOUPDATEBYTE() \
    PLATFORM_RUN_TEST(test_neg_io_ioUpdateByte_nullHandle)

/* Test: TC-IO-0012 */
#define IO_TEST_IOUPDATEBYTE() \
    IO_TEST_POS_IOUPDATEBYTE(); \
    IO_TEST_NEG_IOUPDATEBYTE()

/* ======================================================================== */
/*                        Test APIs: ioUpdateByte_CS                        */
/* ======================================================================== */
#define IO_TEST_POS_IOUPDATEBYTE_CS() \
    PLATFORM_RUN_TEST(test_pos_io_ioUpdateByte_CS_readModifyWrite)

#define IO_TEST_NEG_IOUPDATEBYTE_CS() \
    PLATFORM_RUN_TEST(test_neg_io_ioUpdateByte_CS_nullHandle)

/* Test: TC-IO-0013 */
#define IO_TEST_IOUPDATEBYTE_CS() \
    IO_TEST_POS_IOUPDATEBYTE_CS(); \
    IO_TEST_NEG_IOUPDATEBYTE_CS()

/* ======================================================================== */
/*                        Test APIs: ioUpdateByte_b                         */
/* ======================================================================== */
#define IO_TEST_POS_IOUPDATEBYTE_B() \
    PLATFORM_RUN_TEST(test_pos_io_ioUpdateByte_b_readModifyWriteBit)

#define IO_TEST_NEG_IOUPDATEBYTE_B() \
    PLATFORM_RUN_TEST(test_neg_io_ioUpdateByte_b_nullHandle)

/* Test: TC-IO-0014 */
#define IO_TEST_IOUPDATEBYTE_B() \
    IO_TEST_POS_IOUPDATEBYTE_B(); \
    IO_TEST_NEG_IOUPDATEBYTE_B()

/* ======================================================================== */
/*                       Test APIs: ioUpdateByte_bCS                        */
/* ======================================================================== */
#define IO_TEST_POS_IOUPDATEBYTE_BCS() \
    PLATFORM_RUN_TEST(test_pos_io_ioUpdateByte_bCS_readModifyWriteBit)

#define IO_TEST_NEG_IOUPDATEBYTE_BCS() \
    PLATFORM_RUN_TEST(test_neg_io_ioUpdateByte_bCS_nullHandle)

/* Test: TC-IO-0015 */
#define IO_TEST_IOUPDATEBYTE_BCS() \
    IO_TEST_POS_IOUPDATEBYTE_BCS(); \
    IO_TEST_NEG_IOUPDATEBYTE_BCS()

/* ======================================================================== */
/*                          Test APIs: ioCrcEnable                          */
/* ======================================================================== */
#define IO_TEST_POS_IOCRCENABLE() \
    PLATFORM_RUN_TEST(test_pos_io_ioCrcEnable_crcEnableDisable); \
    PLATFORM_RUN_TEST(test_pos_io_ioCrcEnable_crcWithRegisterAccess); \
    PLATFORM_RUN_TEST(test_pos_io_ioCrcEnable_crcEnableDisableTransitions); \
    PLATFORM_RUN_TEST(test_pos_io_ioCrcEnable_crcStateTransitionsWithOperations)

#define IO_TEST_NEG_IOCRCENABLE() \
    PLATFORM_RUN_TEST(test_neg_io_ioCrcEnable_nullHandle)

/* Test: TC-IO-0016 */
#define IO_TEST_IOCRCENABLE() \
    IO_TEST_POS_IOCRCENABLE(); \
    IO_TEST_NEG_IOCRCENABLE()

/* ======================================================================== */
/*                         Test APIs: ioCrcDisable                          */
/* ======================================================================== */
#define IO_TEST_POS_IOCRCDISABLE() \
    /* Covered in ioCrcEnable tests */

#define IO_TEST_NEG_IOCRCDISABLE() \
    PLATFORM_RUN_TEST(test_neg_io_ioCrcDisable_nullHandle)

/* Test: TC-IO-0017 */
#define IO_TEST_IOCRCDISABLE() \
    IO_TEST_POS_IOCRCDISABLE(); \
    IO_TEST_NEG_IOCRCDISABLE()

/* ======================================================================== */
/*                      Test APIs: ioSetCrcEnableState                      */
/* ======================================================================== */
#define IO_TEST_POS_IOSETCRCENABLESTATE() \
    PLATFORM_RUN_TEST(test_pos_io_ioSetCrcEnableState_crcSetEnableState); \
    PLATFORM_RUN_TEST(test_pos_io_ioSetCrcEnableState_crcEnable1); \
    PLATFORM_RUN_TEST(test_pos_io_ioSetCrcEnableState_crcEnableBoth)

#define IO_TEST_NEG_IOSETCRCENABLESTATE() \
    PLATFORM_RUN_TEST(test_neg_io_ioSetCrcEnableState_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_io_ioSetCrcEnableState_nullCfg)

/* Test: TC-IO-0018 */
#define IO_TEST_IOSETCRCENABLESTATE() \
    IO_TEST_POS_IOSETCRCENABLESTATE(); \
    IO_TEST_NEG_IOSETCRCENABLESTATE()

/* ======================================================================== */
/*                      Test APIs: ioGetCrcEnableState                      */
/* ======================================================================== */
#define IO_TEST_POS_IOGETCRCENABLESTATE() \
    PLATFORM_RUN_TEST(test_pos_io_ioGetCrcEnableState_crcEnable0); \
    PLATFORM_RUN_TEST(test_pos_io_ioGetCrcEnableState_crcEnable1); \
    PLATFORM_RUN_TEST(test_pos_io_ioGetCrcEnableState_crcEnableBoth)

#define IO_TEST_NEG_IOGETCRCENABLESTATE() \
    PLATFORM_RUN_TEST(test_neg_io_ioGetCrcEnableState_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_io_ioGetCrcEnableState_nullEnabled)

/* Test: TC-IO-0019 */
#define IO_TEST_IOGETCRCENABLESTATE() \
    IO_TEST_POS_IOGETCRCENABLESTATE(); \
    IO_TEST_NEG_IOGETCRCENABLESTATE()

/* ========================================================================== */
/*                        Aggregate Test Macros                               */
/* ========================================================================== */

#define IO_TEST_RUN_POSITIVE() \
    IO_TEST_POS_IOTXBYTE(); \
    IO_TEST_POS_IOTXBYTE_CS(); \
    IO_TEST_POS_IORXBYTE(); \
    IO_TEST_POS_IORXBYTE_CS(); \
    IO_TEST_POS_IOUPDATEBYTE(); \
    IO_TEST_POS_IOUPDATEBYTE_CS(); \
    IO_TEST_POS_IOUPDATEBYTE_B(); \
    IO_TEST_POS_IOUPDATEBYTE_BCS(); \
    IO_TEST_POS_IOCRCENABLE(); \
    IO_TEST_POS_IOCRCDISABLE(); \
    IO_TEST_POS_IOSETCRCENABLESTATE(); \
    IO_TEST_POS_IOGETCRCENABLESTATE()

#define IO_TEST_RUN_NEGATIVE() \
    IO_TEST_NEG_IOTXBYTE(); \
    IO_TEST_NEG_IOTXBYTE_CS(); \
    IO_TEST_NEG_IORXBYTE(); \
    IO_TEST_NEG_IORXBYTE_CS(); \
    IO_TEST_NEG_IOUPDATEBYTE(); \
    IO_TEST_NEG_IOUPDATEBYTE_CS(); \
    IO_TEST_NEG_IOUPDATEBYTE_B(); \
    IO_TEST_NEG_IOUPDATEBYTE_BCS(); \
    IO_TEST_NEG_IOCRCENABLE(); \
    IO_TEST_NEG_IOCRCDISABLE(); \
    IO_TEST_NEG_IOSETCRCENABLESTATE(); \
    IO_TEST_NEG_IOGETCRCENABLESTATE()

#define IO_TEST_RUN_ALL() \
    IO_TEST_RUN_POSITIVE(); \
    IO_TEST_RUN_NEGATIVE()

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

void io_test(void *args);

/* Negative test functions */
void test_neg_io_ioTxByte_nullHandle(void);
void test_neg_io_ioTxByte_CS_nullHandle(void);
void test_neg_io_ioRxByte_nullHandle(void);
void test_neg_io_ioRxByte_nullRxData(void);
void test_neg_io_ioRxByte_CS_nullHandle(void);
void test_neg_io_ioRxByte_CS_nullRxData(void);
void test_neg_io_ioTxByte_nullIoWrite(void);
void test_neg_io_ioTxByte_nullCommHandle(void);
void test_neg_io_ioRxByte_nullIoRead(void);
void test_neg_io_ioRxByte_nullCommHandle(void);
void test_neg_io_ioUpdateByte_nullHandle(void);
void test_neg_io_ioUpdateByte_CS_nullHandle(void);
void test_neg_io_ioUpdateByte_b_nullHandle(void);
void test_neg_io_ioUpdateByte_bCS_nullHandle(void);
void test_neg_io_ioSetCrcEnableState_nullHandle(void);
void test_neg_io_ioSetCrcEnableState_nullCfg(void);
void test_neg_io_ioCrcEnable_nullHandle(void);
void test_neg_io_ioCrcDisable_nullHandle(void);
void test_pos_io_ioGetCrcEnableState_crcEnable0(void);
void test_pos_io_ioGetCrcEnableState_crcEnable1(void);
void test_pos_io_ioGetCrcEnableState_crcEnableBoth(void);
void test_neg_io_ioGetCrcEnableState_nullHandle(void);
void test_neg_io_ioGetCrcEnableState_nullEnabled(void);
void test_neg_io_ioRxByte_crcErrorExhaustsRetries(void);
void test_neg_io_ioRxByte_zeroRetryCntImmediateFail(void);
void test_neg_io_ioTxByte_nullTimerWithRetry(void);
void test_neg_io_ioTxByte_nullAsyncHooks(void);
void test_neg_io_ioRxByte_spiRxCrcMismatch(void);
void test_neg_io_ioRxByte_i2cRxCrcMismatch(void);

/* Positive test functions */
void test_pos_io_ioRxByte_singleRegisterRead(void);
void test_pos_io_ioTxByte_singleRegisterWrite(void);
void test_pos_io_ioRxByte_CS_singleRegisterRead(void);
void test_pos_io_ioTxByte_CS_singleRegisterWrite(void);
void test_pos_io_ioUpdateByte_readModifyWrite(void);
void test_pos_io_ioUpdateByte_CS_readModifyWrite(void);
void test_pos_io_ioUpdateByte_b_readModifyWriteBit(void);
void test_pos_io_ioUpdateByte_bCS_readModifyWriteBit(void);
void test_pos_io_ioCrcEnable_crcEnableDisable(void);
void test_pos_io_ioSetCrcEnableState_crcSetEnableState(void);
void test_pos_io_ioSetCrcEnableState_crcEnable1(void);
void test_pos_io_ioSetCrcEnableState_crcEnableBoth(void);
void test_pos_io_ioTxByte_wdgWriteWithCrc(void);
void test_pos_io_ioRxByte_wdgReadWithCrc(void);
void test_pos_io_ioTxByte_multipleRegisterAccess(void);
void test_pos_io_ioRxByte_registerReadVerification(void);
void test_pos_io_ioCrcEnable_crcWithRegisterAccess(void);
void test_pos_io_ioRxByte_readWithCrcValidation(void);
void test_pos_io_ioTxByte_writeWithCrcCalculation(void);
void test_pos_io_ioCrcEnable_crcEnableDisableTransitions(void);
void test_pos_io_ioTxByte_i2cWriteWithCrc(void);
void test_pos_io_ioTxByte_asyncWriteSpi(void);
void test_pos_io_ioTxByte_asyncWriteI2c(void);
void test_pos_io_ioRxByte_asyncReadSpi(void);
void test_pos_io_ioRxByte_asyncReadI2c(void);
void test_pos_io_ioCrcEnable_crcStateTransitionsWithOperations(void);
void test_pos_io_ioRxByte_withRetryOnCrcError(void);
void test_pos_io_ioTxByte_withRetryOnFailure(void);
void test_pos_io_ioTxByte_retrySucceedsOnLastAttempt(void);
void test_pos_io_ioTxByte_multipleRetryAttempts(void);
void test_pos_io_ioTxByte_i2cTxRetry(void);
void test_pos_io_ioRxByte_i2cRxRetry(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* IO_TEST_H */
