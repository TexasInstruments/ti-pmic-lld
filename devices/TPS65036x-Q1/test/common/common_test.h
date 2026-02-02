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
 *    distribution and/or other materials provided with the
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

#ifndef COMMON_TEST_H
#define COMMON_TEST_H

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include "pmic.h"
#include "pmic_common.h"
#include "unity.h"
#include "platform.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* ======================================================================== */
/*           Test APIs: criticalSectionStart, criticalSectionStop           */
/* ======================================================================== */
/* Test: TC-COMMON-0036 */
#define COMMON_TEST_RUN_CRITICAL_SECTION() \
    PLATFORM_RUN_TEST(test_pos_common_criticalSection_nullHandle); \
    PLATFORM_RUN_TEST(test_pos_common_criticalSection_nullCallback); \
    PLATFORM_RUN_TEST(test_pos_common_criticalSection_communication); \
    PLATFORM_RUN_TEST(test_pos_common_criticalSection_diagnostic)

/* ======================================================================== */
/*                          Test APIs: timerWaitMs                          */
/* ======================================================================== */
/* Test: TC-COMMON-0037 */
#define COMMON_TEST_RUN_TIMER() \
    PLATFORM_RUN_TEST(test_pos_common_timerWait_nullHandle); \
    PLATFORM_RUN_TEST(test_pos_common_timerWait_nullCallback); \
    PLATFORM_RUN_TEST(test_pos_common_timerWait_validCall)

/* ======================================================================== */
/*                           Test APIs: logStatus                           */
/* ======================================================================== */
/* Test: TC-COMMON-0038 */
#define COMMON_TEST_RUN_LOG_STATUS() \
    PLATFORM_RUN_TEST(test_pos_common_logStatus_success); \
    PLATFORM_RUN_TEST(test_pos_common_logStatus_nullHandle); \
    PLATFORM_RUN_TEST(test_pos_common_logStatus_nullCritSec); \
    PLATFORM_RUN_TEST(test_pos_common_logStatus_validError); \
    PLATFORM_RUN_TEST(test_pos_common_logStatus_validWarning); \
    PLATFORM_RUN_TEST(test_pos_common_logStatus_invalidStatusType); \
    PLATFORM_RUN_TEST(test_pos_common_logStatus_invalidStatusId); \
    PLATFORM_RUN_TEST(test_pos_common_logStatus_allErrorCodes); \
    PLATFORM_RUN_TEST(test_pos_common_logStatus_invalidStatusNullHandle); \
    PLATFORM_RUN_TEST(test_pos_common_logStatus_successTypeInvalidId); \
    PLATFORM_RUN_TEST(test_pos_common_logStatus_warningTypeInvalidId); \
    PLATFORM_RUN_TEST(test_pos_common_logStatus_maxErrorId); \
    PLATFORM_RUN_TEST(test_pos_common_logStatus_maxWarningId); \
    PLATFORM_RUN_TEST(test_neg_common_logStatus_exceedsMaxErrorId)

/* ======================================================================== */
/* Test APIs: getDiagnostic, getDiagnostics, clrDiagnostic, clrDiagnostics, */
/*            clrDiagnosticsAll                                             */
/* ======================================================================== */
/* Test: TC-COMMON-0039 */
#define COMMON_TEST_RUN_DIAGNOSTIC() \
    PLATFORM_RUN_TEST(test_pos_common_getDiagnostic_nullHandle); \
    PLATFORM_RUN_TEST(test_pos_common_getDiagnostic_nullDiagnostic); \
    PLATFORM_RUN_TEST(test_pos_common_getDiagnostic_invalidValidParams); \
    PLATFORM_RUN_TEST(test_pos_common_getDiagnostic_invalidStatusCode); \
    PLATFORM_RUN_TEST(test_pos_common_getDiagnostic_errorCnt); \
    PLATFORM_RUN_TEST(test_pos_common_getDiagnostic_errorFlag); \
    PLATFORM_RUN_TEST(test_pos_common_getDiagnostic_warningCnt); \
    PLATFORM_RUN_TEST(test_pos_common_getDiagnostic_warningFlag); \
    PLATFORM_RUN_TEST(test_pos_common_getDiagnostic_successType); \
    PLATFORM_RUN_TEST(test_pos_common_getDiagnostic_nullCritSecStart); \
    PLATFORM_RUN_TEST(test_pos_common_getDiagnostic_nullCritSecStop); \
    PLATFORM_RUN_TEST(test_pos_common_getDiagnostic_maxErrorId); \
    PLATFORM_RUN_TEST(test_neg_common_getDiagnostic_exceedsMaxErrorId); \
    PLATFORM_RUN_TEST(test_neg_common_getDiagnostic_exceedsMaxWarningId); \
    PLATFORM_RUN_TEST(test_pos_common_getDiagnostics_multiple); \
    PLATFORM_RUN_TEST(test_pos_common_getDiagnostics_zeroCount); \
    PLATFORM_RUN_TEST(test_pos_common_getDiagnostics_exceedsMax); \
    PLATFORM_RUN_TEST(test_pos_common_getDiagnostics_nullCritSecStart); \
    PLATFORM_RUN_TEST(test_pos_common_getDiagnostics_nullCritSecStop); \
    PLATFORM_RUN_TEST(test_pos_common_getDiagnostics_nullHandle); \
    PLATFORM_RUN_TEST(test_pos_common_getDiagnostics_nullDiagnosticArray); \
    PLATFORM_RUN_TEST(test_pos_common_getDiagnostics_zeroValidParamsInArray); \
    PLATFORM_RUN_TEST(test_pos_common_getDiagnostics_invalidStatusCodeInArray); \
    PLATFORM_RUN_TEST(test_pos_common_getDiagnostics_successTypeInArray); \
    PLATFORM_RUN_TEST(test_pos_common_clrDiagnostic_nullHandle); \
    PLATFORM_RUN_TEST(test_pos_common_clrDiagnostic_errorCnt); \
    PLATFORM_RUN_TEST(test_pos_common_clrDiagnostic_warningCnt); \
    PLATFORM_RUN_TEST(test_pos_common_clrDiagnostic_nullCritSecStart); \
    PLATFORM_RUN_TEST(test_pos_common_clrDiagnostic_nullCritSecStop); \
    PLATFORM_RUN_TEST(test_pos_common_clrDiagnostic_nullDiagnostic); \
    PLATFORM_RUN_TEST(test_pos_common_clrDiagnostic_invalidValidParams); \
    PLATFORM_RUN_TEST(test_pos_common_clrDiagnostic_invalidStatusCode); \
    PLATFORM_RUN_TEST(test_pos_common_clrDiagnostic_successType); \
    PLATFORM_RUN_TEST(test_pos_common_clrDiagnostic_errorFlagOnly); \
    PLATFORM_RUN_TEST(test_pos_common_clrDiagnostic_warningFlagOnly); \
    PLATFORM_RUN_TEST(test_pos_common_clrDiagnostic_maxErrorId); \
    PLATFORM_RUN_TEST(test_neg_common_clrDiagnostic_exceedsMaxErrorId); \
    PLATFORM_RUN_TEST(test_neg_common_clrDiagnostic_exceedsMaxWarningId); \
    PLATFORM_RUN_TEST(test_pos_common_clrDiagnostics_multiple); \
    PLATFORM_RUN_TEST(test_pos_common_clrDiagnostics_nullCritSecStart); \
    PLATFORM_RUN_TEST(test_pos_common_clrDiagnostics_nullCritSecStop); \
    PLATFORM_RUN_TEST(test_pos_common_clrDiagnostics_nullHandle); \
    PLATFORM_RUN_TEST(test_pos_common_clrDiagnostics_nullDiagnosticArray); \
    PLATFORM_RUN_TEST(test_pos_common_clrDiagnostics_zeroCount); \
    PLATFORM_RUN_TEST(test_pos_common_clrDiagnostics_exceedsMax); \
    PLATFORM_RUN_TEST(test_pos_common_clrDiagnostics_zeroValidParamsInArray); \
    PLATFORM_RUN_TEST(test_pos_common_clrDiagnostics_invalidStatusCodeInArray); \
    PLATFORM_RUN_TEST(test_pos_common_clrDiagnostics_successTypeInArray); \
    PLATFORM_RUN_TEST(test_pos_common_clrDiagnosticsAll_clearAll); \
    PLATFORM_RUN_TEST(test_pos_common_clrDiagnosticsAll_nullHandle); \
    PLATFORM_RUN_TEST(test_pos_common_clrDiagnosticsAll_nullCritSecStart); \
    PLATFORM_RUN_TEST(test_pos_common_clrDiagnosticsAll_nullCritSecStop); \
    PLATFORM_RUN_TEST(test_pos_common_overflow_errorCnt); \
    PLATFORM_RUN_TEST(test_pos_common_overflow_warningCnt); \
    PLATFORM_RUN_TEST(test_pos_common_getDiagnostics_errorFlagOnly); \
    PLATFORM_RUN_TEST(test_pos_common_getDiagnostics_warningFlagOnly); \
    PLATFORM_RUN_TEST(test_pos_common_clrDiagnostics_errorFlagOnly); \
    PLATFORM_RUN_TEST(test_pos_common_clrDiagnostics_warningCntOnly); \
    PLATFORM_RUN_TEST(test_pos_common_clrDiagnostics_warningFlagOnly)

/* ======================================================================== */
/*        Test APIs: getRetryCnt, incrementRetryCnt, clearRetryCnt,         */
/*                   getRetryCntOverflow, clrRetryCntOverflow               */
/* ======================================================================== */
/* Test: TC-COMMON-0040 */
#define COMMON_TEST_RUN_RETRY_CNT() \
    PLATFORM_RUN_TEST(test_pos_common_getRetryCnt_nullHandle); \
    PLATFORM_RUN_TEST(test_pos_common_getRetryCnt_nullOutput); \
    PLATFORM_RUN_TEST(test_pos_common_getRetryCnt_initialZero); \
    PLATFORM_RUN_TEST(test_pos_common_getRetryCnt_nullCritSecStart); \
    PLATFORM_RUN_TEST(test_pos_common_getRetryCnt_nullCritSecStop); \
    PLATFORM_RUN_TEST(test_pos_common_incrementRetryCnt_nullHandle); \
    PLATFORM_RUN_TEST(test_pos_common_incrementRetryCnt_once); \
    PLATFORM_RUN_TEST(test_pos_common_incrementRetryCnt_multiple); \
    PLATFORM_RUN_TEST(test_pos_common_incrementRetryCnt_nullCritSecStart); \
    PLATFORM_RUN_TEST(test_pos_common_incrementRetryCnt_nullCritSecStop); \
    PLATFORM_RUN_TEST(test_pos_common_clrRetryCnt_nullHandle); \
    PLATFORM_RUN_TEST(test_pos_common_clrRetryCnt_afterIncrement); \
    PLATFORM_RUN_TEST(test_pos_common_clrRetryCnt_nullCritSecStart); \
    PLATFORM_RUN_TEST(test_pos_common_clrRetryCnt_nullCritSecStop); \
    PLATFORM_RUN_TEST(test_pos_common_getRetryCntOverflow_nullHandle); \
    PLATFORM_RUN_TEST(test_pos_common_getRetryCntOverflow_nullOutput); \
    PLATFORM_RUN_TEST(test_pos_common_getRetryCntOverflow_nullCritSecStart); \
    PLATFORM_RUN_TEST(test_pos_common_getRetryCntOverflow_nullCritSecStop); \
    PLATFORM_RUN_TEST(test_pos_common_clrRetryCntOverflow_nullHandle); \
    PLATFORM_RUN_TEST(test_pos_common_clrRetryCntOverflow_nullCritSecStart); \
    PLATFORM_RUN_TEST(test_pos_common_clrRetryCntOverflow_nullCritSecStop); \
    PLATFORM_RUN_TEST(test_pos_common_overflow_retryCnt)

/* ========================================================================== */
/*                           Aggregate Test Runners                           */
/* ========================================================================== */
#define COMMON_TEST_RUN_ALL() \
    COMMON_TEST_RUN_CRITICAL_SECTION(); \
    COMMON_TEST_RUN_TIMER(); \
    COMMON_TEST_RUN_LOG_STATUS(); \
    COMMON_TEST_RUN_DIAGNOSTIC(); \
    COMMON_TEST_RUN_RETRY_CNT()

/* ========================================================================== */
/*                       Function Declarations                                */
/* ========================================================================== */

/* Test entry point */
void common_test(void *args);

/* Critical Section Tests */
void test_pos_common_criticalSection_nullHandle(void);
void test_pos_common_criticalSection_nullCallback(void);
void test_pos_common_criticalSection_communication(void);
void test_pos_common_criticalSection_diagnostic(void);

/* Timer Tests */
void test_pos_common_timerWait_nullHandle(void);
void test_pos_common_timerWait_nullCallback(void);
void test_pos_common_timerWait_validCall(void);

/* Pmic_logStatus Tests */
void test_pos_common_logStatus_success(void);
void test_pos_common_logStatus_nullHandle(void);
void test_pos_common_logStatus_nullCritSec(void);
void test_pos_common_logStatus_validError(void);
void test_pos_common_logStatus_validWarning(void);
void test_pos_common_logStatus_invalidStatusType(void);
void test_pos_common_logStatus_invalidStatusId(void);
void test_pos_common_logStatus_allErrorCodes(void);
void test_pos_common_logStatus_invalidStatusNullHandle(void);
void test_pos_common_logStatus_successTypeInvalidId(void);
void test_pos_common_logStatus_warningTypeInvalidId(void);

/* Diagnostic Get/Clear Tests */
void test_pos_common_getDiagnostic_nullHandle(void);
void test_pos_common_getDiagnostic_nullDiagnostic(void);
void test_pos_common_getDiagnostic_invalidValidParams(void);
void test_pos_common_getDiagnostic_invalidStatusCode(void);
void test_pos_common_getDiagnostic_errorCnt(void);
void test_pos_common_getDiagnostic_errorFlag(void);
void test_pos_common_getDiagnostic_warningCnt(void);
void test_pos_common_getDiagnostic_warningFlag(void);
void test_pos_common_getDiagnostic_successType(void);
void test_pos_common_getDiagnostic_nullCritSecStart(void);
void test_pos_common_getDiagnostic_nullCritSecStop(void);
void test_pos_common_getDiagnostics_multiple(void);
void test_pos_common_getDiagnostics_zeroCount(void);
void test_pos_common_getDiagnostics_exceedsMax(void);
void test_pos_common_getDiagnostics_nullCritSecStart(void);
void test_pos_common_getDiagnostics_nullCritSecStop(void);
void test_pos_common_getDiagnostics_nullHandle(void);
void test_pos_common_getDiagnostics_nullDiagnosticArray(void);
void test_pos_common_getDiagnostics_zeroValidParamsInArray(void);
void test_pos_common_getDiagnostics_invalidStatusCodeInArray(void);
void test_pos_common_getDiagnostics_successTypeInArray(void);
void test_pos_common_clrDiagnostic_nullHandle(void);
void test_pos_common_clrDiagnostic_errorCnt(void);
void test_pos_common_clrDiagnostic_warningCnt(void);
void test_pos_common_clrDiagnostic_nullCritSecStart(void);
void test_pos_common_clrDiagnostic_nullCritSecStop(void);
void test_pos_common_clrDiagnostic_nullDiagnostic(void);
void test_pos_common_clrDiagnostic_invalidValidParams(void);
void test_pos_common_clrDiagnostic_invalidStatusCode(void);
void test_pos_common_clrDiagnostic_successType(void);
void test_pos_common_clrDiagnostic_errorFlagOnly(void);
void test_pos_common_clrDiagnostic_warningFlagOnly(void);
void test_pos_common_clrDiagnostics_multiple(void);
void test_pos_common_clrDiagnostics_nullCritSecStart(void);
void test_pos_common_clrDiagnostics_nullCritSecStop(void);
void test_pos_common_clrDiagnostics_nullHandle(void);
void test_pos_common_clrDiagnostics_nullDiagnosticArray(void);
void test_pos_common_clrDiagnostics_zeroCount(void);
void test_pos_common_clrDiagnostics_exceedsMax(void);
void test_pos_common_clrDiagnostics_zeroValidParamsInArray(void);
void test_pos_common_clrDiagnostics_invalidStatusCodeInArray(void);
void test_pos_common_clrDiagnostics_successTypeInArray(void);
void test_pos_common_clrDiagnosticsAll_clearAll(void);
void test_pos_common_clrDiagnosticsAll_nullHandle(void);
void test_pos_common_clrDiagnosticsAll_nullCritSecStart(void);
void test_pos_common_clrDiagnosticsAll_nullCritSecStop(void);
void test_pos_common_overflow_errorCnt(void);
void test_pos_common_overflow_warningCnt(void);
void test_pos_common_getDiagnostics_errorFlagOnly(void);
void test_pos_common_getDiagnostics_warningFlagOnly(void);
void test_pos_common_clrDiagnostics_errorFlagOnly(void);
void test_pos_common_clrDiagnostics_warningCntOnly(void);
void test_pos_common_clrDiagnostics_warningFlagOnly(void);

/* Retry Counter Tests */
void test_pos_common_getRetryCnt_nullHandle(void);
void test_pos_common_getRetryCnt_nullOutput(void);
void test_pos_common_getRetryCnt_initialZero(void);
void test_pos_common_getRetryCnt_nullCritSecStart(void);
void test_pos_common_getRetryCnt_nullCritSecStop(void);
void test_pos_common_incrementRetryCnt_nullHandle(void);
void test_pos_common_incrementRetryCnt_once(void);
void test_pos_common_incrementRetryCnt_multiple(void);
void test_pos_common_incrementRetryCnt_nullCritSecStart(void);
void test_pos_common_incrementRetryCnt_nullCritSecStop(void);
void test_pos_common_clrRetryCnt_nullHandle(void);
void test_pos_common_clrRetryCnt_afterIncrement(void);
void test_pos_common_clrRetryCnt_nullCritSecStart(void);
void test_pos_common_clrRetryCnt_nullCritSecStop(void);
void test_pos_common_getRetryCntOverflow_nullHandle(void);
void test_pos_common_getRetryCntOverflow_nullOutput(void);
void test_pos_common_getRetryCntOverflow_nullCritSecStart(void);
void test_pos_common_getRetryCntOverflow_nullCritSecStop(void);
void test_pos_common_clrRetryCntOverflow_nullHandle(void);
void test_pos_common_clrRetryCntOverflow_nullCritSecStart(void);
void test_pos_common_clrRetryCntOverflow_nullCritSecStop(void);
void test_pos_common_overflow_retryCnt(void);

/* Boundary Condition Tests */
void test_pos_common_logStatus_maxErrorId(void);
void test_pos_common_logStatus_maxWarningId(void);
void test_pos_common_getDiagnostic_maxErrorId(void);
void test_pos_common_clrDiagnostic_maxErrorId(void);
void test_neg_common_logStatus_exceedsMaxErrorId(void);
void test_neg_common_getDiagnostic_exceedsMaxErrorId(void);
void test_neg_common_getDiagnostic_exceedsMaxWarningId(void);
void test_neg_common_clrDiagnostic_exceedsMaxErrorId(void);
void test_neg_common_clrDiagnostic_exceedsMaxWarningId(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* COMMON_TEST_H */
