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
/*                          Function Declarations                             */
/* ========================================================================== */

void pmic_test(void *args);

/* ========================================================================== */
/*                        Test Organization Macros                            */
/* ========================================================================== */

/* Pmic_init Tests */
#define PMIC_TEST_POS_PMICINIT() \
    PLATFORM_RUN_TEST(test_pos_pmic_pmicInit_validConfig); \
    PLATFORM_RUN_TEST(test_pos_pmic_pmicInit_reinit); \
    PLATFORM_RUN_TEST(test_pos_pmic_pmicInit_with_crc_enabled); \
    PLATFORM_RUN_TEST(test_pos_pmic_pmicInit_with_both_crc_flags); \
    PLATFORM_RUN_TEST(test_pos_pmic_pmicInit_crc_disabled); \
    PLATFORM_RUN_TEST(test_pos_pmic_pmicInit_verify_crc_state); \
    PLATFORM_RUN_TEST(test_pos_pmic_pmicInit_complete_flow); \
    PLATFORM_RUN_TEST(test_pos_pmic_pmicInit_i2c_single_mode); \
    PLATFORM_RUN_TEST(test_pos_pmic_pmicInit_i2c_dual_mode); \
    PLATFORM_RUN_TEST(test_pos_pmic_pmicInit_device_info_retrieval); \
    PLATFORM_RUN_TEST(test_pos_pmic_pmicInit_async_mode); \
    PLATFORM_RUN_TEST(test_pos_pmic_pmicInit_with_i2c_addresses); \
    PLATFORM_RUN_TEST(test_pos_pmic_pmicInit_with_task_handle); \
    PLATFORM_RUN_TEST(test_pos_pmic_pmicInit_withRetryCnt); \
    PLATFORM_RUN_TEST(test_pos_pmic_pmicInit_withRetryInterval); \
    PLATFORM_RUN_TEST(test_pos_pmic_pmicInit_withTimerWaitMs)

#define PMIC_TEST_NEG_PMICINIT() \
    PLATFORM_RUN_TEST(test_neg_pmic_pmicInit_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_pmic_pmicInit_nullConfig); \
    PLATFORM_RUN_TEST(test_neg_pmic_pmicInit_invalidCommMode); \
    PLATFORM_RUN_TEST(test_neg_pmic_pmicInit_nullCommHandle); \
    PLATFORM_RUN_TEST(test_neg_pmic_pmicInit_nullTaskHandle); \
    PLATFORM_RUN_TEST(test_neg_pmic_pmicInit_nullIoRead); \
    PLATFORM_RUN_TEST(test_neg_pmic_pmicInit_nullIoWrite); \
    PLATFORM_RUN_TEST(test_neg_pmic_pmicInit_nullAsyncRxStart); \
    PLATFORM_RUN_TEST(test_neg_pmic_pmicInit_nullAsyncTxStart); \
    PLATFORM_RUN_TEST(test_neg_pmic_pmicInit_nullAsyncRxAwait); \
    PLATFORM_RUN_TEST(test_neg_pmic_pmicInit_nullAsyncTxAwait); \
    PLATFORM_RUN_TEST(test_neg_pmic_pmicInit_nullCritSecStart); \
    PLATFORM_RUN_TEST(test_neg_pmic_pmicInit_nullCritSecStop); \
    PLATFORM_RUN_TEST(test_neg_pmic_pmicInit_nullIrqCallback); \
    PLATFORM_RUN_TEST(test_neg_pmic_pmicInit_timerWaitNull); \
    PLATFORM_RUN_TEST(test_neg_pmic_pmicInit_timerWaitMsCallbackNull)

#define PMIC_TEST_PMICINIT() \
    PMIC_TEST_POS_PMICINIT(); \
    PMIC_TEST_NEG_PMICINIT()

/* Pmic_deinit Tests */
#define PMIC_TEST_POS_PMICDEINIT() \
    PLATFORM_RUN_TEST(test_pos_pmic_pmicDeinit_afterInit); \
    PLATFORM_RUN_TEST(test_pos_pmic_pmicDeinit_success_path)

#define PMIC_TEST_NEG_PMICDEINIT() \
    PLATFORM_RUN_TEST(test_neg_pmic_pmicDeinit_nullHandle)

#define PMIC_TEST_PMICDEINIT() \
    PMIC_TEST_POS_PMICDEINIT(); \
    PMIC_TEST_NEG_PMICDEINIT()

/* Pmic_checkHandle Tests */
#define PMIC_TEST_POS_PMICCHECKHANDLE() \
    PLATFORM_RUN_TEST(test_pos_pmic_pmicCheckHandle_validHandle); \
    PLATFORM_RUN_TEST(test_pos_pmic_pmicCheckHandle_invalidHandle); \
    PLATFORM_RUN_TEST(test_pos_pmic_pmicCheckHandle_all_validations)

#define PMIC_TEST_NEG_PMICCHECKHANDLE() \
    PLATFORM_RUN_TEST(test_neg_pmic_pmicCheckHandle_nullHandle)

#define PMIC_TEST_PMICCHECKHANDLE() \
    PMIC_TEST_POS_PMICCHECKHANDLE(); \
    PMIC_TEST_NEG_PMICCHECKHANDLE()

/* ========================================================================== */
/*                        Aggregate Test Macros                               */
/* ========================================================================== */

#define PMIC_TEST_RUN_POSITIVE() \
    PMIC_TEST_POS_PMICINIT(); \
    PMIC_TEST_POS_PMICDEINIT(); \
    PMIC_TEST_POS_PMICCHECKHANDLE()

#define PMIC_TEST_RUN_NEGATIVE() \
    PMIC_TEST_NEG_PMICINIT(); \
    PMIC_TEST_NEG_PMICDEINIT(); \
    PMIC_TEST_NEG_PMICCHECKHANDLE()

#define PMIC_TEST_RUN_ALL() \
    PMIC_TEST_RUN_POSITIVE(); \
    PMIC_TEST_RUN_NEGATIVE()

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* Negative test functions */
void test_neg_pmic_pmicInit_nullHandle(void);
void test_neg_pmic_pmicInit_nullConfig(void);
void test_neg_pmic_pmicDeinit_nullHandle(void);
void test_neg_pmic_pmicCheckHandle_nullHandle(void);
void test_neg_pmic_pmicInit_invalidCommMode(void);
void test_neg_pmic_pmicInit_nullCommHandle(void);
void test_neg_pmic_pmicInit_nullTaskHandle(void);
void test_neg_pmic_pmicInit_nullIoRead(void);
void test_neg_pmic_pmicInit_nullIoWrite(void);
void test_neg_pmic_pmicInit_nullAsyncRxStart(void);
void test_neg_pmic_pmicInit_nullAsyncTxStart(void);
void test_neg_pmic_pmicInit_nullAsyncRxAwait(void);
void test_neg_pmic_pmicInit_nullAsyncTxAwait(void);
void test_neg_pmic_pmicInit_nullCritSecStart(void);
void test_neg_pmic_pmicInit_nullCritSecStop(void);
void test_neg_pmic_pmicInit_nullIrqCallback(void);
void test_neg_pmic_pmicInit_timerWaitNull(void);
void test_neg_pmic_pmicInit_timerWaitMsCallbackNull(void);

/* Positive test functions */
void test_pos_pmic_pmicInit_validConfig(void);
void test_pos_pmic_pmicDeinit_afterInit(void);
void test_pos_pmic_pmicCheckHandle_validHandle(void);
void test_pos_pmic_pmicCheckHandle_invalidHandle(void);
void test_pos_pmic_pmicInit_reinit(void);
void test_pos_pmic_pmicInit_with_crc_enabled(void);
void test_pos_pmic_pmicInit_with_both_crc_flags(void);
void test_pos_pmic_pmicInit_crc_disabled(void);
void test_pos_pmic_pmicInit_verify_crc_state(void);
void test_pos_pmic_pmicInit_complete_flow(void);
void test_pos_pmic_pmicInit_i2c_single_mode(void);
void test_pos_pmic_pmicInit_i2c_dual_mode(void);
void test_pos_pmic_pmicInit_device_info_retrieval(void);
void test_pos_pmic_pmicDeinit_success_path(void);
void test_pos_pmic_pmicCheckHandle_all_validations(void);
void test_pos_pmic_pmicInit_async_mode(void);
void test_pos_pmic_pmicInit_with_i2c_addresses(void);
void test_pos_pmic_pmicInit_with_task_handle(void);
void test_pos_pmic_pmicInit_withRetryCnt(void);
void test_pos_pmic_pmicInit_withRetryInterval(void);
void test_pos_pmic_pmicInit_withTimerWaitMs(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /*__PMIC_TEST_H__*/
