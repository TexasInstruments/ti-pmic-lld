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

/* ======================================================================== */
/*      Test APIs: Pmic_criticalSectionStart, Pmic_criticalSectionStop      */
/* ======================================================================== */

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

/* ======================================================================== */
/*                          Test APIs: timerWaitMs                          */
/* ======================================================================== */

#define COMMON_TEST_POS_TIMERWAITMS() \
    PLATFORM_RUN_TEST(test_pos_common_timerWaitMs_validCall)

#define COMMON_TEST_NEG_TIMERWAITMS() \
    PLATFORM_RUN_TEST(test_neg_common_timerWaitMs_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_common_timerWaitMs_nullCallback)

/* Test: TC-COMMON-0022 */
#define COMMON_TEST_TIMERWAITMS() \
    COMMON_TEST_POS_TIMERWAITMS(); \
    COMMON_TEST_NEG_TIMERWAITMS()

/* ======================================================================== */
/*                           Test APIs: logStatus                           */
/* ======================================================================== */

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

/* ======================================================================== */
/*                         Test APIs: getDiagnostic                         */
/* ======================================================================== */

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
    PLATFORM_RUN_TEST(test_neg_common_getDiagnostic_exceedsMaxWarningId); \
    PLATFORM_RUN_TEST(test_neg_common_getDiagnostic_nullCritSecStart); \
    PLATFORM_RUN_TEST(test_neg_common_getDiagnostic_nullCritSecStop); \
    PLATFORM_RUN_TEST(test_neg_common_statusCodeCheck_successTypeNonZeroId)

/* Test: TC-COMMON-0024 */
#define COMMON_TEST_GETDIAGNOSTIC() \
    COMMON_TEST_POS_GETDIAGNOSTIC(); \
    COMMON_TEST_NEG_GETDIAGNOSTIC()

/* ======================================================================== */
/*                        Test APIs: getDiagnostics                         */
/* ======================================================================== */

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
    PLATFORM_RUN_TEST(test_neg_common_getDiagnostics_successTypeInArray); \
    PLATFORM_RUN_TEST(test_neg_common_getDiagnostics_nullCritSecStart); \
    PLATFORM_RUN_TEST(test_neg_common_getDiagnostics_nullCritSecStop)

/* Test: TC-COMMON-0025 */
#define COMMON_TEST_GETDIAGNOSTICS() \
    COMMON_TEST_POS_GETDIAGNOSTICS(); \
    COMMON_TEST_NEG_GETDIAGNOSTICS()

/* ======================================================================== */
/*                         Test APIs: clrDiagnostic                         */
/* ======================================================================== */

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
    PLATFORM_RUN_TEST(test_neg_common_clrDiagnostic_exceedsMaxWarningId); \
    PLATFORM_RUN_TEST(test_neg_common_clrDiagnostic_nullCritSecStart); \
    PLATFORM_RUN_TEST(test_neg_common_clrDiagnostic_nullCritSecStop)

/* Test: TC-COMMON-0026 */
#define COMMON_TEST_CLRDIAGNOSTIC() \
    COMMON_TEST_POS_CLRDIAGNOSTIC(); \
    COMMON_TEST_NEG_CLRDIAGNOSTIC()

/* ======================================================================== */
/*                        Test APIs: clrDiagnostics                         */
/* ======================================================================== */

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
    PLATFORM_RUN_TEST(test_neg_common_clrDiagnostics_successTypeInArray); \
    PLATFORM_RUN_TEST(test_neg_common_clrDiagnostics_nullCritSecStart); \
    PLATFORM_RUN_TEST(test_neg_common_clrDiagnostics_nullCritSecStop)

/* Test: TC-COMMON-0027 */
#define COMMON_TEST_CLRDIAGNOSTICS() \
    COMMON_TEST_POS_CLRDIAGNOSTICS(); \
    COMMON_TEST_NEG_CLRDIAGNOSTICS()

/* ======================================================================== */
/*                       Test APIs: clrDiagnosticsAll                       */
/* ======================================================================== */

#define COMMON_TEST_POS_CLRDIAGNOSTICSALL() \
    PLATFORM_RUN_TEST(test_pos_common_clrDiagnosticsAll_clearAll)

#define COMMON_TEST_NEG_CLRDIAGNOSTICSALL() \
    PLATFORM_RUN_TEST(test_neg_common_clrDiagnosticsAll_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_common_clrDiagnosticsAll_nullCritSecStart); \
    PLATFORM_RUN_TEST(test_neg_common_clrDiagnosticsAll_nullCritSecStop)

/* Test: TC-COMMON-0028 */
#define COMMON_TEST_CLRDIAGNOSTICSALL() \
    COMMON_TEST_POS_CLRDIAGNOSTICSALL(); \
    COMMON_TEST_NEG_CLRDIAGNOSTICSALL()

/* ======================================================================== */
/*Test APIs: Pmic_getDiagnostic, Pmic_getRetryCnt, Pmic_getRetryCntOverflow */
/* ======================================================================== */

#define COMMON_TEST_POS_OVERFLOW() \
    PLATFORM_RUN_TEST(test_pos_common_overflow_errorCnt); \
    PLATFORM_RUN_TEST(test_pos_common_overflow_warningCnt); \
    PLATFORM_RUN_TEST(test_pos_common_overflow_retryCnt)

/* Test: TC-COMMON-0029 */
#define COMMON_TEST_OVERFLOW() \
    COMMON_TEST_POS_OVERFLOW()

/* ======================================================================== */
/*                          Test APIs: getRetryCnt                          */
/* ======================================================================== */

#define COMMON_TEST_POS_GETRETRYCNT() \
    PLATFORM_RUN_TEST(test_pos_common_getRetryCnt_initialZero)

#define COMMON_TEST_NEG_GETRETRYCNT() \
    PLATFORM_RUN_TEST(test_neg_common_getRetryCnt_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_common_getRetryCnt_nullOutput); \
    PLATFORM_RUN_TEST(test_neg_common_getRetryCnt_nullCritSecStart); \
    PLATFORM_RUN_TEST(test_neg_common_getRetryCnt_nullCritSecStop)

/* Test: TC-COMMON-0030 */
#define COMMON_TEST_GETRETRYCNT() \
    COMMON_TEST_POS_GETRETRYCNT(); \
    COMMON_TEST_NEG_GETRETRYCNT()

/* ======================================================================== */
/*                       Test APIs: incrementRetryCnt                       */
/* ======================================================================== */

#define COMMON_TEST_POS_INCREMENTRETRYCNT() \
    PLATFORM_RUN_TEST(test_pos_common_incrementRetryCnt_once); \
    PLATFORM_RUN_TEST(test_pos_common_incrementRetryCnt_multiple)

#define COMMON_TEST_NEG_INCREMENTRETRYCNT() \
    PLATFORM_RUN_TEST(test_neg_common_incrementRetryCnt_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_common_incrementRetryCnt_nullCritSecStart); \
    PLATFORM_RUN_TEST(test_neg_common_incrementRetryCnt_nullCritSecStop)

/* Test: TC-COMMON-0031 */
#define COMMON_TEST_INCREMENTRETRYCNT() \
    COMMON_TEST_POS_INCREMENTRETRYCNT(); \
    COMMON_TEST_NEG_INCREMENTRETRYCNT()

/* ======================================================================== */
/*                          Test APIs: clrRetryCnt                          */
/* ======================================================================== */

#define COMMON_TEST_POS_CLRRETRYCNT() \
    PLATFORM_RUN_TEST(test_pos_common_clrRetryCnt_afterIncrement)

#define COMMON_TEST_NEG_CLRRETRYCNT() \
    PLATFORM_RUN_TEST(test_neg_common_clrRetryCnt_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_common_clrRetryCnt_nullCritSecStart); \
    PLATFORM_RUN_TEST(test_neg_common_clrRetryCnt_nullCritSecStop)

/* Test: TC-COMMON-0032 */
#define COMMON_TEST_CLRRETRYCNT() \
    COMMON_TEST_POS_CLRRETRYCNT(); \
    COMMON_TEST_NEG_CLRRETRYCNT()

/* ======================================================================== */
/*                      Test APIs: getRetryCntOverflow                      */
/* ======================================================================== */

#define COMMON_TEST_NEG_GETRETRYCNTOVERFLOW() \
    PLATFORM_RUN_TEST(test_neg_common_getRetryCntOverflow_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_common_getRetryCntOverflow_nullOutput); \
    PLATFORM_RUN_TEST(test_neg_common_getRetryCntOverflow_nullCritSecStart); \
    PLATFORM_RUN_TEST(test_neg_common_getRetryCntOverflow_nullCritSecStop)

/* Test: TC-COMMON-0033 */
#define COMMON_TEST_GETRETRYCNTOVERFLOW() \
    COMMON_TEST_NEG_GETRETRYCNTOVERFLOW()

/* ======================================================================== */
/*                      Test APIs: clrRetryCntOverflow                      */
/* ======================================================================== */

#define COMMON_TEST_NEG_CLRRETRYCNTOVERFLOW() \
    PLATFORM_RUN_TEST(test_neg_common_clrRetryCntOverflow_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_common_clrRetryCntOverflow_nullCritSecStart); \
    PLATFORM_RUN_TEST(test_neg_common_clrRetryCntOverflow_nullCritSecStop)

/* Test: TC-COMMON-0034 */
#define COMMON_TEST_CLRRETRYCNTOVERFLOW() \
    COMMON_TEST_NEG_CLRRETRYCNTOVERFLOW()

/* ======================================================================== */
/*                      Test APIs: irqResponseCallback                      */
/* ======================================================================== */

#define COMMON_TEST_POS_IRQRESPONSECALLBACK() \
    PLATFORM_RUN_TEST(test_pos_common_irqResponseCallback_validCall)

#define COMMON_TEST_NEG_IRQRESPONSECALLBACK() \
    PLATFORM_RUN_TEST(test_neg_common_irqResponseCallback_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_common_irqResponseCallback_nullCallback)

/* Test: TC-COMMON-0035 */
#define COMMON_TEST_IRQRESPONSECALLBACK() \
    COMMON_TEST_POS_IRQRESPONSECALLBACK(); \
    COMMON_TEST_NEG_IRQRESPONSECALLBACK()

/* ======================================================================== */
/*                    Test APIs: Pmic_validParamCheck                       */
/* ======================================================================== */

#define COMMON_TEST_POS_VALIDPARAMCHECK() \
    PLATFORM_RUN_TEST(test_pos_common_validParamCheck_bitSet); \
    PLATFORM_RUN_TEST(test_pos_common_validParamCheck_bitClear); \
    PLATFORM_RUN_TEST(test_pos_common_validParamCheck_noParamsSet); \
    PLATFORM_RUN_TEST(test_pos_common_validParamCheck_allParamsSet); \
    PLATFORM_RUN_TEST(test_pos_common_validParamCheck_multipleParams)

/* Test: TC-COMMON-0059 */
#define COMMON_TEST_VALIDPARAMCHECK() \
    COMMON_TEST_POS_VALIDPARAMCHECK()

/* ======================================================================== */
/*                  Test APIs: Pmic_validParamStatusCheck                   */
/* ======================================================================== */

#define COMMON_TEST_POS_VALIDPARAMSTATUSCHECK() \
    PLATFORM_RUN_TEST(test_pos_common_validParamStatusCheck_successAndBitSet); \
    PLATFORM_RUN_TEST(test_pos_common_validParamStatusCheck_errorAndBitSet); \
    PLATFORM_RUN_TEST(test_pos_common_validParamStatusCheck_successAndBitClear); \
    PLATFORM_RUN_TEST(test_pos_common_validParamStatusCheck_errorAndBitClear)

/* Test: TC-COMMON-0060 */
#define COMMON_TEST_VALIDPARAMSTATUSCHECK() \
    COMMON_TEST_POS_VALIDPARAMSTATUSCHECK()

/* ======================================================================== */
/*                      Test APIs: Pmic_getBitField                         */
/* ======================================================================== */

#define COMMON_TEST_POS_GETBITFIELD() \
    PLATFORM_RUN_TEST(test_pos_common_getBitField_lowNibble); \
    PLATFORM_RUN_TEST(test_pos_common_getBitField_highNibble); \
    PLATFORM_RUN_TEST(test_pos_common_getBitField_singleBitSet); \
    PLATFORM_RUN_TEST(test_pos_common_getBitField_singleBitClear); \
    PLATFORM_RUN_TEST(test_pos_common_getBitField_fullByteMask); \
    PLATFORM_RUN_TEST(test_pos_common_getBitField_twoBitField)

/* Test: TC-COMMON-0061 */
#define COMMON_TEST_GETBITFIELD() \
    COMMON_TEST_POS_GETBITFIELD()

/* ======================================================================== */
/*                     Test APIs: Pmic_getBitField_b                        */
/* ======================================================================== */

#define COMMON_TEST_POS_GETBITFIELD_B() \
    PLATFORM_RUN_TEST(test_pos_common_getBitField_b_bitSet); \
    PLATFORM_RUN_TEST(test_pos_common_getBitField_b_bitClear); \
    PLATFORM_RUN_TEST(test_pos_common_getBitField_b_lsb); \
    PLATFORM_RUN_TEST(test_pos_common_getBitField_b_msb); \
    PLATFORM_RUN_TEST(test_pos_common_getBitField_b_alternatingPattern)

/* Test: TC-COMMON-0062 */
#define COMMON_TEST_GETBITFIELD_B() \
    COMMON_TEST_POS_GETBITFIELD_B()

/* ======================================================================== */
/*                      Test APIs: Pmic_setBitField                         */
/* ======================================================================== */

#define COMMON_TEST_POS_SETBITFIELD() \
    PLATFORM_RUN_TEST(test_pos_common_setBitField_setLowNibble); \
    PLATFORM_RUN_TEST(test_pos_common_setBitField_setHighNibble); \
    PLATFORM_RUN_TEST(test_pos_common_setBitField_clearBits); \
    PLATFORM_RUN_TEST(test_pos_common_setBitField_preservesOtherBits); \
    PLATFORM_RUN_TEST(test_pos_common_setBitField_fullByteMask)

/* Test: TC-COMMON-0063 */
#define COMMON_TEST_SETBITFIELD() \
    COMMON_TEST_POS_SETBITFIELD()

/* ======================================================================== */
/*                     Test APIs: Pmic_setBitField_b                        */
/* ======================================================================== */

#define COMMON_TEST_POS_SETBITFIELD_B() \
    PLATFORM_RUN_TEST(test_pos_common_setBitField_b_setTrue); \
    PLATFORM_RUN_TEST(test_pos_common_setBitField_b_setFalse); \
    PLATFORM_RUN_TEST(test_pos_common_setBitField_b_preservesOtherBitsWhenSet); \
    PLATFORM_RUN_TEST(test_pos_common_setBitField_b_preservesOtherBitsWhenCleared); \
    PLATFORM_RUN_TEST(test_pos_common_setBitField_b_lsb); \
    PLATFORM_RUN_TEST(test_pos_common_setBitField_b_msb)

/* Test: TC-COMMON-0064 */
#define COMMON_TEST_SETBITFIELD_B() \
    COMMON_TEST_POS_SETBITFIELD_B()

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
    COMMON_TEST_POS_IRQRESPONSECALLBACK(); \
    COMMON_TEST_POS_VALIDPARAMCHECK(); \
    COMMON_TEST_POS_VALIDPARAMSTATUSCHECK(); \
    COMMON_TEST_POS_GETBITFIELD(); \
    COMMON_TEST_POS_GETBITFIELD_B(); \
    COMMON_TEST_POS_SETBITFIELD(); \
    COMMON_TEST_POS_SETBITFIELD_B()

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
void test_neg_common_getDiagnostic_nullCritSecStart(void);
void test_neg_common_getDiagnostic_nullCritSecStop(void);
void test_pos_common_getDiagnostics_multiple(void);
void test_neg_common_getDiagnostics_zeroCount(void);
void test_neg_common_getDiagnostics_exceedsMax(void);
void test_neg_common_getDiagnostics_nullHandle(void);
void test_neg_common_getDiagnostics_nullDiagnosticArray(void);
void test_neg_common_getDiagnostics_zeroValidParamsInArray(void);
void test_neg_common_getDiagnostics_invalidStatusCodeInArray(void);
void test_neg_common_getDiagnostics_successTypeInArray(void);
void test_neg_common_getDiagnostics_nullCritSecStart(void);
void test_neg_common_getDiagnostics_nullCritSecStop(void);
void test_neg_common_clrDiagnostic_nullHandle(void);
void test_pos_common_clrDiagnostic_errorCnt(void);
void test_pos_common_clrDiagnostic_warningCnt(void);
void test_neg_common_clrDiagnostic_nullDiagnostic(void);
void test_neg_common_clrDiagnostic_invalidValidParams(void);
void test_neg_common_clrDiagnostic_invalidStatusCode(void);
void test_neg_common_clrDiagnostic_successType(void);
void test_pos_common_clrDiagnostic_errorFlagOnly(void);
void test_pos_common_clrDiagnostic_warningFlagOnly(void);
void test_neg_common_clrDiagnostic_nullCritSecStart(void);
void test_neg_common_clrDiagnostic_nullCritSecStop(void);
void test_pos_common_clrDiagnostics_multiple(void);
void test_neg_common_clrDiagnostics_nullHandle(void);
void test_neg_common_clrDiagnostics_nullDiagnosticArray(void);
void test_neg_common_clrDiagnostics_zeroCount(void);
void test_neg_common_clrDiagnostics_exceedsMax(void);
void test_neg_common_clrDiagnostics_zeroValidParamsInArray(void);
void test_neg_common_clrDiagnostics_invalidStatusCodeInArray(void);
void test_neg_common_clrDiagnostics_successTypeInArray(void);
void test_neg_common_clrDiagnostics_nullCritSecStart(void);
void test_neg_common_clrDiagnostics_nullCritSecStop(void);
void test_pos_common_clrDiagnosticsAll_clearAll(void);
void test_neg_common_clrDiagnosticsAll_nullHandle(void);
void test_neg_common_clrDiagnosticsAll_nullCritSecStart(void);
void test_neg_common_clrDiagnosticsAll_nullCritSecStop(void);
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
void test_neg_common_getRetryCnt_nullCritSecStart(void);
void test_neg_common_getRetryCnt_nullCritSecStop(void);
void test_neg_common_incrementRetryCnt_nullHandle(void);
void test_pos_common_incrementRetryCnt_once(void);
void test_pos_common_incrementRetryCnt_multiple(void);
void test_neg_common_incrementRetryCnt_nullCritSecStart(void);
void test_neg_common_incrementRetryCnt_nullCritSecStop(void);
void test_neg_common_clrRetryCnt_nullHandle(void);
void test_pos_common_clrRetryCnt_afterIncrement(void);
void test_neg_common_clrRetryCnt_nullCritSecStart(void);
void test_neg_common_clrRetryCnt_nullCritSecStop(void);
void test_neg_common_getRetryCntOverflow_nullHandle(void);
void test_neg_common_getRetryCntOverflow_nullOutput(void);
void test_neg_common_getRetryCntOverflow_nullCritSecStart(void);
void test_neg_common_getRetryCntOverflow_nullCritSecStop(void);
void test_neg_common_clrRetryCntOverflow_nullHandle(void);
void test_neg_common_clrRetryCntOverflow_nullCritSecStart(void);
void test_neg_common_clrRetryCntOverflow_nullCritSecStop(void);
void test_pos_common_overflow_retryCnt(void);

/* IRQ Response Callback Tests (LP8772x-Q1 only) */
void test_pos_common_irqResponseCallback_validCall(void);
void test_neg_common_irqResponseCallback_nullHandle(void);
void test_neg_common_irqResponseCallback_nullCallback(void);

/* Bit Field and validParam Utility Tests */
void test_pos_common_validParamCheck_bitSet(void);
void test_pos_common_validParamCheck_bitClear(void);
void test_pos_common_validParamCheck_noParamsSet(void);
void test_pos_common_validParamCheck_allParamsSet(void);
void test_pos_common_validParamCheck_multipleParams(void);
void test_pos_common_validParamStatusCheck_successAndBitSet(void);
void test_pos_common_validParamStatusCheck_errorAndBitSet(void);
void test_pos_common_validParamStatusCheck_successAndBitClear(void);
void test_pos_common_validParamStatusCheck_errorAndBitClear(void);
void test_pos_common_getBitField_lowNibble(void);
void test_pos_common_getBitField_highNibble(void);
void test_pos_common_getBitField_singleBitSet(void);
void test_pos_common_getBitField_singleBitClear(void);
void test_pos_common_getBitField_fullByteMask(void);
void test_pos_common_getBitField_twoBitField(void);
void test_pos_common_getBitField_b_bitSet(void);
void test_pos_common_getBitField_b_bitClear(void);
void test_pos_common_getBitField_b_lsb(void);
void test_pos_common_getBitField_b_msb(void);
void test_pos_common_getBitField_b_alternatingPattern(void);
void test_pos_common_setBitField_setLowNibble(void);
void test_pos_common_setBitField_setHighNibble(void);
void test_pos_common_setBitField_clearBits(void);
void test_pos_common_setBitField_preservesOtherBits(void);
void test_pos_common_setBitField_fullByteMask(void);
void test_pos_common_setBitField_b_setTrue(void);
void test_pos_common_setBitField_b_setFalse(void);
void test_pos_common_setBitField_b_preservesOtherBitsWhenSet(void);
void test_pos_common_setBitField_b_preservesOtherBitsWhenCleared(void);
void test_pos_common_setBitField_b_lsb(void);
void test_pos_common_setBitField_b_msb(void);

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

/* statusCodeCheck Coverage Tests */
void test_neg_common_statusCodeCheck_successTypeNonZeroId(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* COMMON_TEST_H */
