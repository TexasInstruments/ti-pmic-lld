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
/*                          Function Declarations                             */
/* ========================================================================== */

/* Test entry point */
void common_test(void *args);

/* ========================================================================== */
/*                       Positive Test Declarations                           */
/* ========================================================================== */

/* criticalSection Tests */
void test_pos_common_criticalSection_communication(void);
void test_pos_common_criticalSection_diagnostic(void);

/* timerWait Tests */
void test_pos_common_timerWait_validCall(void);

/* logStatus Tests */
void test_pos_common_logStatus_success(void);
void test_pos_common_logStatus_validError(void);
void test_pos_common_logStatus_validWarning(void);
void test_pos_common_logStatus_allErrorCodes(void);

/* getDiagnostic Tests */
void test_pos_common_getDiagnostic_errorCnt(void);
void test_pos_common_getDiagnostic_errorFlag(void);
void test_pos_common_getDiagnostic_warningCnt(void);
void test_pos_common_getDiagnostic_warningFlag(void);
void test_pos_common_getDiagnostics_multiple(void);
void test_pos_common_getDiagnostics_errorFlagOnly(void);
void test_pos_common_getDiagnostics_warningFlagOnly(void);

/* clrDiagnostic Tests */
void test_pos_common_clrDiagnostic_errorCnt(void);
void test_pos_common_clrDiagnostic_warningCnt(void);
void test_pos_common_clrDiagnostic_errorFlagOnly(void);
void test_pos_common_clrDiagnostic_warningFlagOnly(void);
void test_pos_common_clrDiagnostics_multiple(void);
void test_pos_common_clrDiagnostics_errorFlagOnly(void);
void test_pos_common_clrDiagnostics_warningCntOnly(void);
void test_pos_common_clrDiagnostics_warningFlagOnly(void);
void test_pos_common_clrDiagnosticsAll_clearAll(void);
void test_pos_common_overflow_errorCnt(void);
void test_pos_common_overflow_warningCnt(void);

/* getRetryCnt Tests */
void test_pos_common_getRetryCnt_initialZero(void);

/* incrementRetryCnt Tests */
void test_pos_common_incrementRetryCnt_once(void);
void test_pos_common_incrementRetryCnt_multiple(void);

/* clrRetryCnt Tests */
void test_pos_common_clrRetryCnt_afterIncrement(void);

/* getRetryCntOverflow and clrRetryCntOverflow Tests */
void test_pos_common_overflow_retryCnt(void);

/* ========================================================================== */
/*                       Negative Test Declarations                           */
/* ========================================================================== */

/* criticalSection Tests */
void test_neg_common_criticalSection_nullHandle(void);
void test_neg_common_criticalSection_nullCallback(void);

/* timerWait Tests */
void test_neg_common_timerWait_nullHandle(void);
void test_neg_common_timerWait_nullCallback(void);

/* logStatus Tests */
void test_neg_common_logStatus_nullHandle(void);
void test_neg_common_logStatus_nullCritSec(void);
void test_neg_common_logStatus_invalidStatusType(void);
void test_neg_common_logStatus_invalidStatusId(void);
void test_neg_common_logStatus_invalidStatusNullHandle(void);
void test_neg_common_logStatus_successTypeInvalidId(void);
void test_neg_common_logStatus_warningTypeInvalidId(void);

/* getDiagnostic Tests */
void test_neg_common_getDiagnostic_nullHandle(void);
void test_neg_common_getDiagnostic_nullDiagnostic(void);
void test_neg_common_getDiagnostic_invalidValidParams(void);
void test_neg_common_getDiagnostic_invalidStatusCode(void);
void test_neg_common_getDiagnostic_successType(void);
void test_neg_common_getDiagnostics_nullHandle(void);
void test_neg_common_getDiagnostics_nullDiagnosticArray(void);
void test_neg_common_getDiagnostics_zeroCount(void);
void test_neg_common_getDiagnostics_exceedsMax(void);
void test_neg_common_getDiagnostics_zeroValidParamsInArray(void);
void test_neg_common_getDiagnostics_invalidStatusCodeInArray(void);
void test_neg_common_getDiagnostics_successTypeInArray(void);

/* clrDiagnostic Tests */
void test_neg_common_clrDiagnostic_nullHandle(void);
void test_neg_common_clrDiagnostic_nullDiagnostic(void);
void test_neg_common_clrDiagnostic_invalidValidParams(void);
void test_neg_common_clrDiagnostic_invalidStatusCode(void);
void test_neg_common_clrDiagnostic_successType(void);
void test_neg_common_clrDiagnostics_nullHandle(void);
void test_neg_common_clrDiagnostics_nullDiagnosticArray(void);
void test_neg_common_clrDiagnostics_zeroCount(void);
void test_neg_common_clrDiagnostics_exceedsMax(void);
void test_neg_common_clrDiagnostics_zeroValidParamsInArray(void);
void test_neg_common_clrDiagnostics_invalidStatusCodeInArray(void);
void test_neg_common_clrDiagnostics_successTypeInArray(void);
void test_neg_common_clrDiagnosticsAll_nullHandle(void);

/* getRetryCnt Tests */
void test_neg_common_getRetryCnt_nullHandle(void);
void test_neg_common_getRetryCnt_nullOutput(void);

/* incrementRetryCnt Tests */
void test_neg_common_incrementRetryCnt_nullHandle(void);

/* clrRetryCnt Tests */
void test_neg_common_clrRetryCnt_nullHandle(void);

/* getRetryCntOverflow Tests */
void test_neg_common_getRetryCntOverflow_nullHandle(void);
void test_neg_common_getRetryCntOverflow_nullOutput(void);

/* clrRetryCntOverflow Tests */
void test_neg_common_clrRetryCntOverflow_nullHandle(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* COMMON_TEST_H */
