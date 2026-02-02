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

/* ============================================================================== */
/* API-Specific Test Macros - Pmic_criticalSectionStart, Pmic_criticalSectionStop */
/* ============================================================================== */

#define COMMON_TEST_POS_CRITICALSECTION() \
    PLATFORM_RUN_TEST(test_pos_common_criticalSection_communication); \
    PLATFORM_RUN_TEST(test_pos_common_criticalSection_diagnostic)

#define COMMON_TEST_NEG_CRITICALSECTION() \
    PLATFORM_RUN_TEST(test_neg_common_criticalSectionStart_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_common_criticalSectionStart_nullCallback); \
    PLATFORM_RUN_TEST(test_neg_common_criticalSectionStop_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_common_criticalSectionStop_nullCallback)

/* Test: TC-COMMON-0021 */
#define COMMON_TEST_CRITICALSECTION() \
    COMMON_TEST_POS_CRITICALSECTION(); \
    COMMON_TEST_NEG_CRITICALSECTION()

/* ========================================================================== */
/*                 API-Specific Test Macros - timerWaitMs                     */
/* ========================================================================== */

#define COMMON_TEST_POS_TIMERWAITMS() \
    PLATFORM_RUN_TEST(test_pos_common_timerWaitMs_validCall)

#define COMMON_TEST_NEG_TIMERWAITMS() \
    PLATFORM_RUN_TEST(test_neg_common_timerWaitMs_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_common_timerWaitMs_nullCallback)

/* Test: TC-COMMON-0022 */
#define COMMON_TEST_TIMERWAITMS() \
    COMMON_TEST_POS_TIMERWAITMS(); \
    COMMON_TEST_NEG_TIMERWAITMS()

/* ========================================================================== */
/*                  API-Specific Test Macros - logStatus                      */
/* ========================================================================== */

#define COMMON_TEST_POS_LOGSTATUS() \
    PLATFORM_RUN_TEST(test_pos_common_logStatus_success); \
    PLATFORM_RUN_TEST(test_pos_common_logStatus_validError); \
    PLATFORM_RUN_TEST(test_pos_common_logStatus_validWarning); \
    PLATFORM_RUN_TEST(test_pos_common_logStatus_allErrorCodes); \
    PLATFORM_RUN_TEST(test_pos_common_logStatus_maxErrorId); \
    PLATFORM_RUN_TEST(test_pos_common_logStatus_maxWarningId)

#define COMMON_TEST_NEG_LOGSTATUS() \
    PLATFORM_RUN_TEST(test_neg_common_logStatus_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_common_logStatus_nullCritSec); \
    PLATFORM_RUN_TEST(test_neg_common_logStatus_invalidStatusType); \
    PLATFORM_RUN_TEST(test_neg_common_logStatus_invalidStatusId); \
    PLATFORM_RUN_TEST(test_neg_common_logStatus_invalidStatusNullHandle); \
    PLATFORM_RUN_TEST(test_neg_common_logStatus_warningTypeInvalidId); \
    PLATFORM_RUN_TEST(test_neg_common_logStatus_exceedsMaxErrorId)

/* Test: TC-COMMON-0023 */
#define COMMON_TEST_LOGSTATUS() \
    COMMON_TEST_POS_LOGSTATUS(); \
    COMMON_TEST_NEG_LOGSTATUS()

/* ========================================================================== */
/*               API-Specific Test Macros - getDiagnostic                     */
/* ========================================================================== */

#define COMMON_TEST_POS_GETDIAGNOSTIC() \
    PLATFORM_RUN_TEST(test_pos_common_getDiagnostic_errorCnt); \
    PLATFORM_RUN_TEST(test_pos_common_getDiagnostic_errorFlag); \
    PLATFORM_RUN_TEST(test_pos_common_getDiagnostic_warningCnt); \
    PLATFORM_RUN_TEST(test_pos_common_getDiagnostic_warningFlag); \
    PLATFORM_RUN_TEST(test_pos_common_getDiagnostic_maxErrorId)

#define COMMON_TEST_NEG_GETDIAGNOSTIC() \
    PLATFORM_RUN_TEST(test_neg_common_getDiagnostic_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_common_getDiagnostic_nullDiagnostic); \
    PLATFORM_RUN_TEST(test_neg_common_getDiagnostic_invalidValidParams); \
    PLATFORM_RUN_TEST(test_neg_common_getDiagnostic_invalidStatusCode); \
    PLATFORM_RUN_TEST(test_neg_common_getDiagnostic_successType); \
    PLATFORM_RUN_TEST(test_neg_common_getDiagnostic_exceedsMaxErrorId); \
    PLATFORM_RUN_TEST(test_neg_common_getDiagnostic_exceedsMaxWarningId)

/* Test: TC-COMMON-0024 */
#define COMMON_TEST_GETDIAGNOSTIC() \
    COMMON_TEST_POS_GETDIAGNOSTIC(); \
    COMMON_TEST_NEG_GETDIAGNOSTIC()

/* ========================================================================== */
/*               API-Specific Test Macros - getDiagnostics                    */
/* ========================================================================== */

#define COMMON_TEST_POS_GETDIAGNOSTICS() \
    PLATFORM_RUN_TEST(test_pos_common_getDiagnostics_multiple); \
    PLATFORM_RUN_TEST(test_pos_common_getDiagnostics_errorFlagOnly); \
    PLATFORM_RUN_TEST(test_pos_common_getDiagnostics_warningFlagOnly)

#define COMMON_TEST_NEG_GETDIAGNOSTICS() \
    PLATFORM_RUN_TEST(test_neg_common_getDiagnostics_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_common_getDiagnostics_nullDiagnosticArray); \
    PLATFORM_RUN_TEST(test_neg_common_getDiagnostics_zeroCount); \
    PLATFORM_RUN_TEST(test_neg_common_getDiagnostics_exceedsMax); \
    PLATFORM_RUN_TEST(test_neg_common_getDiagnostics_zeroValidParamsInArray); \
    PLATFORM_RUN_TEST(test_neg_common_getDiagnostics_invalidStatusCodeInArray); \
    PLATFORM_RUN_TEST(test_neg_common_getDiagnostics_successTypeInArray)

/* Test: TC-COMMON-0025 */
#define COMMON_TEST_GETDIAGNOSTICS() \
    COMMON_TEST_POS_GETDIAGNOSTICS(); \
    COMMON_TEST_NEG_GETDIAGNOSTICS()

/* ========================================================================== */
/*               API-Specific Test Macros - clrDiagnostic                     */
/* ========================================================================== */

#define COMMON_TEST_POS_CLRDIAGNOSTIC() \
    PLATFORM_RUN_TEST(test_pos_common_clrDiagnostic_errorCnt); \
    PLATFORM_RUN_TEST(test_pos_common_clrDiagnostic_warningCnt); \
    PLATFORM_RUN_TEST(test_pos_common_clrDiagnostic_errorFlagOnly); \
    PLATFORM_RUN_TEST(test_pos_common_clrDiagnostic_warningFlagOnly); \
    PLATFORM_RUN_TEST(test_pos_common_clrDiagnostic_maxErrorId)

#define COMMON_TEST_NEG_CLRDIAGNOSTIC() \
    PLATFORM_RUN_TEST(test_neg_common_clrDiagnostic_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_common_clrDiagnostic_nullDiagnostic); \
    PLATFORM_RUN_TEST(test_neg_common_clrDiagnostic_invalidValidParams); \
    PLATFORM_RUN_TEST(test_neg_common_clrDiagnostic_invalidStatusCode); \
    PLATFORM_RUN_TEST(test_neg_common_clrDiagnostic_successType); \
    PLATFORM_RUN_TEST(test_neg_common_clrDiagnostic_exceedsMaxErrorId); \
    PLATFORM_RUN_TEST(test_neg_common_clrDiagnostic_exceedsMaxWarningId)

/* Test: TC-COMMON-0026 */
#define COMMON_TEST_CLRDIAGNOSTIC() \
    COMMON_TEST_POS_CLRDIAGNOSTIC(); \
    COMMON_TEST_NEG_CLRDIAGNOSTIC()

/* ========================================================================== */
/*               API-Specific Test Macros - clrDiagnostics                    */
/* ========================================================================== */

#define COMMON_TEST_POS_CLRDIAGNOSTICS() \
    PLATFORM_RUN_TEST(test_pos_common_clrDiagnostics_multiple); \
    PLATFORM_RUN_TEST(test_pos_common_clrDiagnostics_errorFlagOnly); \
    PLATFORM_RUN_TEST(test_pos_common_clrDiagnostics_warningCntOnly); \
    PLATFORM_RUN_TEST(test_pos_common_clrDiagnostics_warningFlagOnly)

#define COMMON_TEST_NEG_CLRDIAGNOSTICS() \
    PLATFORM_RUN_TEST(test_neg_common_clrDiagnostics_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_common_clrDiagnostics_nullDiagnosticArray); \
    PLATFORM_RUN_TEST(test_neg_common_clrDiagnostics_zeroCount); \
    PLATFORM_RUN_TEST(test_neg_common_clrDiagnostics_exceedsMax); \
    PLATFORM_RUN_TEST(test_neg_common_clrDiagnostics_zeroValidParamsInArray); \
    PLATFORM_RUN_TEST(test_neg_common_clrDiagnostics_invalidStatusCodeInArray); \
    PLATFORM_RUN_TEST(test_neg_common_clrDiagnostics_successTypeInArray)

/* Test: TC-COMMON-0027 */
#define COMMON_TEST_CLRDIAGNOSTICS() \
    COMMON_TEST_POS_CLRDIAGNOSTICS(); \
    COMMON_TEST_NEG_CLRDIAGNOSTICS()

/* ========================================================================== */
/*               API-Specific Test Macros - clrDiagnosticsAll                 */
/* ========================================================================== */

#define COMMON_TEST_POS_CLRDIAGNOSTICSALL() \
    PLATFORM_RUN_TEST(test_pos_common_clrDiagnosticsAll_clearAll)

#define COMMON_TEST_NEG_CLRDIAGNOSTICSALL() \
    PLATFORM_RUN_TEST(test_neg_common_clrDiagnosticsAll_nullHandle)

/* Test: TC-COMMON-0028 */
#define COMMON_TEST_CLRDIAGNOSTICSALL() \
    COMMON_TEST_POS_CLRDIAGNOSTICSALL(); \
    COMMON_TEST_NEG_CLRDIAGNOSTICSALL()

/* ========================================================================================= */
/* API-Specific Test Macros - Pmic_getDiagnostic, Pmic_getRetryCnt, Pmic_getRetryCntOverflow */
/* ========================================================================================= */

#define COMMON_TEST_POS_OVERFLOW() \
    PLATFORM_RUN_TEST(test_pos_common_overflow_errorCnt); \
    PLATFORM_RUN_TEST(test_pos_common_overflow_warningCnt); \
    PLATFORM_RUN_TEST(test_pos_common_overflow_retryCnt)

/* Test: TC-COMMON-0029 */
#define COMMON_TEST_OVERFLOW() \
    COMMON_TEST_POS_OVERFLOW()

/* ========================================================================== */
/*                 API-Specific Test Macros - getRetryCnt                     */
/* ========================================================================== */

#define COMMON_TEST_POS_GETRETRYCNT() \
    PLATFORM_RUN_TEST(test_pos_common_getRetryCnt_initialZero)

#define COMMON_TEST_NEG_GETRETRYCNT() \
    PLATFORM_RUN_TEST(test_neg_common_getRetryCnt_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_common_getRetryCnt_nullOutput)

/* Test: TC-COMMON-0030 */
#define COMMON_TEST_GETRETRYCNT() \
    COMMON_TEST_POS_GETRETRYCNT(); \
    COMMON_TEST_NEG_GETRETRYCNT()

/* ========================================================================== */
/*               API-Specific Test Macros - incrementRetryCnt                 */
/* ========================================================================== */

#define COMMON_TEST_POS_INCREMENTRETRYCNT() \
    PLATFORM_RUN_TEST(test_pos_common_incrementRetryCnt_once); \
    PLATFORM_RUN_TEST(test_pos_common_incrementRetryCnt_multiple)

#define COMMON_TEST_NEG_INCREMENTRETRYCNT() \
    PLATFORM_RUN_TEST(test_neg_common_incrementRetryCnt_nullHandle)

/* Test: TC-COMMON-0031 */
#define COMMON_TEST_INCREMENTRETRYCNT() \
    COMMON_TEST_POS_INCREMENTRETRYCNT(); \
    COMMON_TEST_NEG_INCREMENTRETRYCNT()

/* ========================================================================== */
/*                 API-Specific Test Macros - clrRetryCnt                     */
/* ========================================================================== */

#define COMMON_TEST_POS_CLRRETRYCNT() \
    PLATFORM_RUN_TEST(test_pos_common_clrRetryCnt_afterIncrement)

#define COMMON_TEST_NEG_CLRRETRYCNT() \
    PLATFORM_RUN_TEST(test_neg_common_clrRetryCnt_nullHandle)

/* Test: TC-COMMON-0032 */
#define COMMON_TEST_CLRRETRYCNT() \
    COMMON_TEST_POS_CLRRETRYCNT(); \
    COMMON_TEST_NEG_CLRRETRYCNT()

/* ========================================================================== */
/*              API-Specific Test Macros - getRetryCntOverflow                */
/* ========================================================================== */

#define COMMON_TEST_NEG_GETRETRYCNTOVERFLOW() \
    PLATFORM_RUN_TEST(test_neg_common_getRetryCntOverflow_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_common_getRetryCntOverflow_nullOutput)

/* Test: TC-COMMON-0033 */
#define COMMON_TEST_GETRETRYCNTOVERFLOW() \
    COMMON_TEST_NEG_GETRETRYCNTOVERFLOW()

/* ========================================================================== */
/*              API-Specific Test Macros - clrRetryCntOverflow                */
/* ========================================================================== */

#define COMMON_TEST_NEG_CLRRETRYCNTOVERFLOW() \
    PLATFORM_RUN_TEST(test_neg_common_clrRetryCntOverflow_nullHandle)

/* Test: TC-COMMON-0034 */
#define COMMON_TEST_CLRRETRYCNTOVERFLOW() \
    COMMON_TEST_NEG_CLRRETRYCNTOVERFLOW()

/* ========================================================================== */
/*             API-Specific Test Macros - irqResponseCallback                 */
/* ========================================================================== */

#define COMMON_TEST_POS_IRQRESPONSECALLBACK() \
    PLATFORM_RUN_TEST(test_pos_common_irqResponseCallback_validCall)

#define COMMON_TEST_NEG_IRQRESPONSECALLBACK() \
    PLATFORM_RUN_TEST(test_neg_common_irqResponseCallback_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_common_irqResponseCallback_nullCallback)

/* Test: TC-COMMON-0035 */
#define COMMON_TEST_IRQRESPONSECALLBACK() \
    COMMON_TEST_POS_IRQRESPONSECALLBACK(); \
    COMMON_TEST_NEG_IRQRESPONSECALLBACK()

/* ========================================================================== */
/*                         Aggregate Test Runners                             */
/* ========================================================================== */

#define COMMON_TEST_RUN_POSITIVE() \
    COMMON_TEST_POS_CRITICALSECTION(); \
    COMMON_TEST_POS_TIMERWAITMS(); \
    COMMON_TEST_POS_LOGSTATUS(); \
    COMMON_TEST_POS_GETDIAGNOSTIC(); \
    COMMON_TEST_POS_GETDIAGNOSTICS(); \
    COMMON_TEST_POS_CLRDIAGNOSTIC(); \
    COMMON_TEST_POS_CLRDIAGNOSTICS(); \
    COMMON_TEST_POS_CLRDIAGNOSTICSALL(); \
    COMMON_TEST_POS_OVERFLOW(); \
    COMMON_TEST_POS_GETRETRYCNT(); \
    COMMON_TEST_POS_INCREMENTRETRYCNT(); \
    COMMON_TEST_POS_CLRRETRYCNT(); \
    COMMON_TEST_POS_IRQRESPONSECALLBACK()

#define COMMON_TEST_RUN_NEGATIVE() \
    COMMON_TEST_NEG_CRITICALSECTION(); \
    COMMON_TEST_NEG_TIMERWAITMS(); \
    COMMON_TEST_NEG_LOGSTATUS(); \
    COMMON_TEST_NEG_GETDIAGNOSTIC(); \
    COMMON_TEST_NEG_GETDIAGNOSTICS(); \
    COMMON_TEST_NEG_CLRDIAGNOSTIC(); \
    COMMON_TEST_NEG_CLRDIAGNOSTICS(); \
    COMMON_TEST_NEG_CLRDIAGNOSTICSALL(); \
    COMMON_TEST_NEG_GETRETRYCNT(); \
    COMMON_TEST_NEG_INCREMENTRETRYCNT(); \
    COMMON_TEST_NEG_CLRRETRYCNT(); \
    COMMON_TEST_NEG_GETRETRYCNTOVERFLOW(); \
    COMMON_TEST_NEG_CLRRETRYCNTOVERFLOW(); \
    COMMON_TEST_NEG_IRQRESPONSECALLBACK()

#define COMMON_TEST_RUN_ALL() \
    COMMON_TEST_RUN_POSITIVE(); \
    COMMON_TEST_RUN_NEGATIVE()

/* ========================================================================== */
/*                       Function Declarations                                */
/* ========================================================================== */

/* Test entry point */
void common_test(void *args);

/* Critical Section Tests */
void test_neg_common_criticalSectionStart_nullHandle(void);
void test_neg_common_criticalSectionStop_nullHandle(void);
void test_neg_common_criticalSectionStart_nullCallback(void);
void test_neg_common_criticalSectionStop_nullCallback(void);
void test_pos_common_criticalSection_communication(void);
void test_pos_common_criticalSection_diagnostic(void);

/* Timer Tests */
void test_neg_common_timerWaitMs_nullHandle(void);
void test_neg_common_timerWaitMs_nullCallback(void);
void test_pos_common_timerWaitMs_validCall(void);

/* Pmic_logStatus Tests */
void test_pos_common_logStatus_success(void);
void test_neg_common_logStatus_nullHandle(void);
void test_neg_common_logStatus_nullCritSec(void);
void test_pos_common_logStatus_validError(void);
void test_pos_common_logStatus_validWarning(void);
void test_neg_common_logStatus_invalidStatusType(void);
void test_neg_common_logStatus_invalidStatusId(void);
void test_pos_common_logStatus_allErrorCodes(void);
void test_neg_common_logStatus_invalidStatusNullHandle(void);
void test_neg_common_logStatus_warningTypeInvalidId(void);

/* Diagnostic Get/Clear Tests */
void test_neg_common_getDiagnostic_nullHandle(void);
void test_neg_common_getDiagnostic_nullDiagnostic(void);
void test_neg_common_getDiagnostic_invalidValidParams(void);
void test_neg_common_getDiagnostic_invalidStatusCode(void);
void test_pos_common_getDiagnostic_errorCnt(void);
void test_pos_common_getDiagnostic_errorFlag(void);
void test_pos_common_getDiagnostic_warningCnt(void);
void test_pos_common_getDiagnostic_warningFlag(void);
void test_neg_common_getDiagnostic_successType(void);
void test_pos_common_getDiagnostics_multiple(void);
void test_neg_common_getDiagnostics_zeroCount(void);
void test_neg_common_getDiagnostics_exceedsMax(void);
void test_neg_common_getDiagnostics_nullHandle(void);
void test_neg_common_getDiagnostics_nullDiagnosticArray(void);
void test_neg_common_getDiagnostics_zeroValidParamsInArray(void);
void test_neg_common_getDiagnostics_invalidStatusCodeInArray(void);
void test_neg_common_getDiagnostics_successTypeInArray(void);
void test_neg_common_clrDiagnostic_nullHandle(void);
void test_pos_common_clrDiagnostic_errorCnt(void);
void test_pos_common_clrDiagnostic_warningCnt(void);
void test_neg_common_clrDiagnostic_nullDiagnostic(void);
void test_neg_common_clrDiagnostic_invalidValidParams(void);
void test_neg_common_clrDiagnostic_invalidStatusCode(void);
void test_neg_common_clrDiagnostic_successType(void);
void test_pos_common_clrDiagnostic_errorFlagOnly(void);
void test_pos_common_clrDiagnostic_warningFlagOnly(void);
void test_pos_common_clrDiagnostics_multiple(void);
void test_neg_common_clrDiagnostics_nullHandle(void);
void test_neg_common_clrDiagnostics_nullDiagnosticArray(void);
void test_neg_common_clrDiagnostics_zeroCount(void);
void test_neg_common_clrDiagnostics_exceedsMax(void);
void test_neg_common_clrDiagnostics_zeroValidParamsInArray(void);
void test_neg_common_clrDiagnostics_invalidStatusCodeInArray(void);
void test_neg_common_clrDiagnostics_successTypeInArray(void);
void test_pos_common_clrDiagnosticsAll_clearAll(void);
void test_neg_common_clrDiagnosticsAll_nullHandle(void);
void test_pos_common_overflow_errorCnt(void);
void test_pos_common_overflow_warningCnt(void);
void test_pos_common_getDiagnostics_errorFlagOnly(void);
void test_pos_common_getDiagnostics_warningFlagOnly(void);
void test_pos_common_clrDiagnostics_errorFlagOnly(void);
void test_pos_common_clrDiagnostics_warningCntOnly(void);
void test_pos_common_clrDiagnostics_warningFlagOnly(void);

/* Retry Counter Tests */
void test_neg_common_getRetryCnt_nullHandle(void);
void test_neg_common_getRetryCnt_nullOutput(void);
void test_pos_common_getRetryCnt_initialZero(void);
void test_neg_common_incrementRetryCnt_nullHandle(void);
void test_pos_common_incrementRetryCnt_once(void);
void test_pos_common_incrementRetryCnt_multiple(void);
void test_neg_common_clrRetryCnt_nullHandle(void);
void test_pos_common_clrRetryCnt_afterIncrement(void);
void test_neg_common_getRetryCntOverflow_nullHandle(void);
void test_neg_common_getRetryCntOverflow_nullOutput(void);
void test_neg_common_clrRetryCntOverflow_nullHandle(void);
void test_pos_common_overflow_retryCnt(void);

/* IRQ Response Callback Tests (LP8772x-Q1 only) */
void test_pos_common_irqResponseCallback_validCall(void);
void test_neg_common_irqResponseCallback_nullHandle(void);
void test_neg_common_irqResponseCallback_nullCallback(void);

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
