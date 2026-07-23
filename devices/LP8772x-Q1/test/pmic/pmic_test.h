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
#ifndef PMIC_TEST_H
#define PMIC_TEST_H

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
/*         Test APIs: init, deinit, checkHandle, setScratchPadValue         */
/* ======================================================================== */

#define PMIC_TEST_POS_CHECKHANDLE() \
    PLATFORM_RUN_TEST(test_pos_pmic_checkHandle_validations); \
    PLATFORM_RUN_TEST(test_pos_pmic_checkPmicCoreHandle)

#define PMIC_TEST_NEG_CHECKHANDLE() \
    PLATFORM_RUN_TEST(test_neg_pmic_checkHandle_invalidCommMode); \
    PLATFORM_RUN_TEST(test_neg_pmic_checkHandle_nullTimerWithRetry); \
    PLATFORM_RUN_TEST(test_neg_pmic_checkHandle_nullIoWrite); \
    PLATFORM_RUN_TEST(test_neg_pmic_checkHandle_nullCriticalSectionStop); \
    PLATFORM_RUN_TEST(test_neg_pmic_checkPmicCoreHandle_incorrectDrvInitStatus); \
    PLATFORM_RUN_TEST(test_neg_pmic_checkPmicCoreHandle_nullCommHandle0); \
    PLATFORM_RUN_TEST(test_neg_pmic_checkPmicCoreHandle_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_pmic_checkPmicCoreHandle_nullIoRead)

/* Test: TC-PMIC-0007 */
#define PMIC_TEST_CHECKHANDLE() \
    PMIC_TEST_POS_CHECKHANDLE(); \
    PMIC_TEST_NEG_CHECKHANDLE()

/* ======================================================================== */
/*                            Test APIs: deinit                             */
/* ======================================================================== */

#define PMIC_TEST_POS_DEINIT() \
    PLATFORM_RUN_TEST(test_pos_pmic_deinit); \
    PLATFORM_RUN_TEST(test_pos_pmic_deinit_completeFlow)

#define PMIC_TEST_NEG_DEINIT() \
    PLATFORM_RUN_TEST(test_neg_pmic_deinit_nullHandle)

/* Test: TC-PMIC-0008 */
#define PMIC_TEST_DEINIT() \
    PMIC_TEST_POS_DEINIT(); \
    PMIC_TEST_NEG_DEINIT()

/* ======================================================================== */
/*                             Test APIs: init                              */
/* ======================================================================== */

#define PMIC_TEST_POS_INIT() \
    PLATFORM_RUN_TEST(test_pos_pmic_init); \
    PLATFORM_RUN_TEST(test_pos_pmic_init_communicationValidation); \
    PLATFORM_RUN_TEST(test_pos_pmic_init_completeFlow); \
    PLATFORM_RUN_TEST(test_pos_pmic_init_crcErrorRecovery); \
    PLATFORM_RUN_TEST(test_pos_pmic_init_deviceInfoRetrieval); \
    PLATFORM_RUN_TEST(test_pos_pmic_init_withBothCrcEnabled); \
    PLATFORM_RUN_TEST(test_pos_pmic_init_withConfigCrcEnabled); \
    PLATFORM_RUN_TEST(test_pos_pmic_init_withCrcEnabled); \
    PLATFORM_RUN_TEST(test_pos_pmic_init_withRetryCnt); \
    PLATFORM_RUN_TEST(test_pos_pmic_init_withRetryInterval); \
    PLATFORM_RUN_TEST(test_pos_pmic_init_withTimerWaitMs); \
    PLATFORM_RUN_TEST(test_pos_pmic_init_noCommModeValidBit); \
    PLATFORM_RUN_TEST(test_neg_pmic_init_noIoReadValidBit); \
    PLATFORM_RUN_TEST(test_neg_pmic_init_noCritSecStartValidBit)

#define PMIC_TEST_NEG_INIT() \
    PLATFORM_RUN_TEST(test_neg_pmic_init_incorrectCoreCfgCommMode); \
    PLATFORM_RUN_TEST(test_neg_pmic_init_nullCoreCfg); \
    PLATFORM_RUN_TEST(test_neg_pmic_init_nullCoreCfgCommHandle0); \
    PLATFORM_RUN_TEST(test_neg_pmic_init_nullCoreCfgCriticalSectionStart); \
    PLATFORM_RUN_TEST(test_neg_pmic_init_nullCoreCfgCriticalSectionStop); \
    PLATFORM_RUN_TEST(test_neg_pmic_init_nullCoreCfgIoRead); \
    PLATFORM_RUN_TEST(test_neg_pmic_init_nullCoreCfgIoWrite); \
    PLATFORM_RUN_TEST(test_neg_pmic_init_nullCoreCfgIrqResponseCallback); \
    PLATFORM_RUN_TEST(test_neg_pmic_init_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_pmic_init_timerWaitNull); \
    PLATFORM_RUN_TEST(test_neg_pmic_init_getPmicInfo_secondReadFail); \
    PLATFORM_RUN_TEST(test_neg_pmic_init_configureDeviceCrc_ioFail); \
    PLATFORM_RUN_TEST(test_neg_pmic_init_validateComms_readFail); \
    PLATFORM_RUN_TEST(test_neg_pmic_init_configureDeviceCrc_lockDisableFail); \
    PLATFORM_RUN_TEST(test_neg_pmic_init_configureDeviceCrc_lockEnableFail)

/* Test: TC-PMIC-0009 */
#define PMIC_TEST_INIT() \
    PMIC_TEST_POS_INIT(); \
    PMIC_TEST_NEG_INIT()

/* ========================================================================== */
/*                         Aggregate Test Runners                             */
/* ========================================================================== */

#define PMIC_TEST_RUN_POSITIVE() \
    PMIC_TEST_POS_INIT(); \
    PMIC_TEST_POS_CHECKHANDLE(); \
    PMIC_TEST_POS_DEINIT()

#define PMIC_TEST_RUN_NEGATIVE() \
    PMIC_TEST_NEG_CHECKHANDLE(); \
    PMIC_TEST_NEG_DEINIT(); \
    PMIC_TEST_NEG_INIT()

#define PMIC_TEST_RUN_ALL() \
    PMIC_TEST_RUN_POSITIVE(); \
    PMIC_TEST_RUN_NEGATIVE()

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

void pmic_test(void *args);

void test_neg_pmic_checkHandle_invalidCommMode(void);
void test_neg_pmic_checkHandle_nullTimerWithRetry(void);
void test_neg_pmic_checkPmicCoreHandle_incorrectDrvInitStatus(void);
void test_neg_pmic_checkPmicCoreHandle_nullCommHandle0(void);
void test_neg_pmic_checkPmicCoreHandle_nullHandle(void);
void test_neg_pmic_checkPmicCoreHandle_nullIoRead(void);
void test_neg_pmic_deinit_nullHandle(void);
void test_neg_pmic_init_incorrectCoreCfgCommMode(void);
void test_neg_pmic_init_nullCoreCfg(void);
void test_neg_pmic_init_nullCoreCfgCommHandle0(void);
void test_neg_pmic_init_nullCoreCfgCriticalSectionStart(void);
void test_neg_pmic_init_nullCoreCfgCriticalSectionStop(void);
void test_neg_pmic_init_nullCoreCfgIoRead(void);
void test_neg_pmic_init_nullCoreCfgIoWrite(void);
void test_neg_pmic_init_nullCoreCfgIrqResponseCallback(void);
void test_neg_pmic_init_nullHandle(void);
void test_neg_pmic_init_timerWaitNull(void);
void test_pos_pmic_checkHandle_validations(void);
void test_pos_pmic_checkPmicCoreHandle(void);
void test_pos_pmic_deinit(void);
void test_pos_pmic_deinit_completeFlow(void);
void test_pos_pmic_init(void);
void test_pos_pmic_init_communicationValidation(void);
void test_pos_pmic_init_completeFlow(void);
void test_pos_pmic_init_crcErrorRecovery(void);
void test_pos_pmic_init_deviceInfoRetrieval(void);
void test_pos_pmic_init_withBothCrcEnabled(void);
void test_pos_pmic_init_withConfigCrcEnabled(void);
void test_pos_pmic_init_withCrcEnabled(void);
void test_pos_pmic_init_withRetryCnt(void);
void test_pos_pmic_init_withRetryInterval(void);
void test_pos_pmic_init_withTimerWaitMs(void);
void test_pos_pmic_init_noCommModeValidBit(void);
void test_neg_pmic_init_noIoReadValidBit(void);
void test_neg_pmic_init_noCritSecStartValidBit(void);
void test_neg_pmic_init_getPmicInfo_secondReadFail(void);
void test_neg_pmic_init_configureDeviceCrc_ioFail(void);
void test_neg_pmic_init_validateComms_readFail(void);
void test_neg_pmic_init_configureDeviceCrc_lockDisableFail(void);
void test_neg_pmic_init_configureDeviceCrc_lockEnableFail(void);
void test_neg_pmic_checkHandle_nullIoWrite(void);
void test_neg_pmic_checkHandle_nullCriticalSectionStop(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /*__PMIC_TEST_H__*/
