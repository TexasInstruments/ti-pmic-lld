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
#ifndef WDG_TEST_H
#define WDG_TEST_H



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

void wdg_test(void *args);

void test_negative_Pmic_wdgEnable_nullParam_handle(void);
void test_negative_Pmic_wdgDisable_nullParam_handle(void);
void test_negative_Pmic_wdgSetEnableState_nullParam_handle(void);
void test_negative_Pmic_wdgGetEnableState_nullParam_handle(void);
void test_negative_Pmic_wdgGetEnableState_nullParam_isEnabled(void);
void test_negative_Pmic_wdgSetCfg_nullParam_handle(void);
void test_negative_Pmic_wdgSetCfg_nullParam_wdgCfg(void);
void test_negative_Pmic_wdgSetCfg_outOfBounds_thresholdReset(void);
void test_negative_Pmic_wdgSetCfg_outOfBounds_thresholdFail(void);
void test_negative_Pmic_wdgSetCfg_outOfBounds_win1Code(void);
void test_negative_Pmic_wdgSetCfg_outOfBounds_win2Code(void);
void test_negative_Pmic_wdgSetCfg_outOfBounds_qaFdbk(void);
void test_negative_Pmic_wdgSetCfg_outOfBounds_qaLfsr(void);
void test_negative_Pmic_wdgSetCfg_outOfBounds_qaQuesSeed(void);
void test_negative_Pmic_wdgGetCfg_nullParam_handle(void);
void test_negative_Pmic_wdgGetCfg_nullParam_wdgCfg(void);
void test_negative_Pmic_wdgSetPowerHold_nullParam_handle(void);
void test_negative_Pmic_wdgGetPowerHold_nullParam_handle(void);
void test_negative_Pmic_wdgGetPowerHold_nullParam_isEnabled(void);
void test_negative_Pmic_wdgSetReturnToLongWindow_nullParam_handle(void);
void test_negative_Pmic_wdgGetReturnToLongWindow_nullParam_handle(void);
void test_negative_Pmic_wdgGetReturnToLongWindow_nullParam_isEnabled(void);
void test_negative_Pmic_wdgGetErrStatus_nullParam_handle(void);
void test_negative_Pmic_wdgGetErrStatus_nullParam_errors(void);
void test_negative_Pmic_wdgClrErrStatus_nullParam_handle(void);
void test_negative_Pmic_wdgClrErrStatus_nullParam_errors(void);
void test_negative_Pmic_wdgClrErrStatusAll_nullParam_handle(void);
void test_negative_Pmic_wdgGetFailCntStatus_nullParam_handle(void);
void test_negative_Pmic_wdgGetFailCntStatus_nullParam_failCount(void);
void test_negative_Pmic_wdgQaWriteAnswer_nullParam_handle(void);
void test_negative_Pmic_wdgGetFdbkRegData_nullParam_handle(void);
void test_negative_Pmic_wdgGetFdbkRegData_nullParam_regData(void);
void test_negative_Pmic_wdgExtractFdbk_nullParam_wdgAnsInfo(void);
void test_negative_Pmic_wdgGetAnsCntAndQuesRegData_nullParam_handle(void);
void test_negative_Pmic_wdgGetAnsCntAndQuesRegData_nullParam_regData(void);
void test_negative_Pmic_wdgExtractAnsCntAndQues_nullParam_handle(void);
void test_negative_Pmic_wdgExtractAnsCntAndQues_nullParam_wdgAnsInfo(void);
void test_negative_Pmic_wdgWriteAnswer_nullParam_handle(void);
void test_negative_Pmic_wdgWriteAnswer_nullParam_wdgAnsInfo(void);
void test_positive_wdgEnableDisable(void);
void test_positive_wdgEnableDisablePowerHold(void);
void test_positive_wdgEnableDisableReturnToLongWindow(void);
void test_positive_wdgSetGetCfg_rstEn(void);
void test_positive_wdgSetGetCfg_thresholdReset(void);
void test_positive_wdgSetGetCfg_thresholdFail(void);
void test_positive_wdgSetGetCfg_longWinCode(void);
void test_positive_wdgSetGetCfg_win1Code(void);
void test_positive_wdgSetGetCfg_win2Code(void);
void test_positive_wdgSetGetCfg_qaFdbk(void);
void test_positive_wdgSetGetCfg_qaLfsr(void);
void test_positive_wdgSetGetCfg_qaQuesSeed(void);
void test_positive_wdgQaSequence_detectNoErrors(void);
void test_positive_wdgQaSequence_detectTimeout(void);
void test_positive_wdgQaSequence_detectLongWindowTimeout(void);
void test_positive_wdgQaSequence_detectAnswerEarlyError(void);
void test_positive_wdgQaSequence_detectSequenceError(void);
void test_positive_wdgQaSequence_detectAnswerError(void);
void test_positive_wdgQaSequence_detectFailInt(void);
void test_positive_wdgQaSequence_detectResetInt(void);
void test_positive_wdg_qa_with_irq_callback(void);
void test_positive_wdg_clear_error_optimization(void);
void test_positive_wdgQaWriteAnswer_qaFdbk0(void);
void test_positive_wdgQaWriteAnswer_qaFdbk1(void);
void test_positive_wdgQaWriteAnswer_qaFdbk2(void);
void test_positive_wdgQaWriteAnswer_qaFdbk3(void);
void test_positive_wdgClrErrStatus_th1ErrorOnly(void);
void test_positive_wdgClrErrStatus_th2ErrorOnly(void);
void test_positive_wdgClrErrStatus_seqErrorOnly(void);
void test_positive_wdgGetFailCntStatus_failCntOnly(void);
void test_positive_wdgGetFailCntStatus_badCntOnly(void);

/* LP8772x-Q1 tests for uncovered lines in pmic_wdg.c */
void test_positive_wdg_qaWithIrqCallback(void);
void test_positive_wdg_getErrStatus_longWindowTimeout(void);
void test_positive_wdg_getErrStatus_answerEarly(void);
void test_positive_wdg_getErrStatus_answerError(void);
void test_positive_wdg_getFailCnt_goodEvent(void);
void test_positive_wdg_writeAnswer_success(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /*__WDG_TEST_H__*/
