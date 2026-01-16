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
#ifndef PMIC_TEST_WDG_H
#define PMIC_TEST_WDG_H



/* ========================================================================== */
/*                              Include Files                                 */
/* ========================================================================== */

#include "test_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

void wdg_test(void *args);

void test_negative_Pmic_wdgSetEnableState_nullParam_pmicHandle(void);
void test_negative_Pmic_wdgEnable_nullParam_pmicHandle(void);
void test_negative_Pmic_wdgDisable_nullParam_pmicHandle(void);
void test_negative_Pmic_wdgGetEnableState_nullParam_pmicHandle(void);
void test_negative_Pmic_wdgGetEnableState_nullParam_wdgEnabled(void);
void test_negative_Pmic_wdgSetCfg_nullParam_pmicHandle(void);
void test_negative_Pmic_wdgSetCfg_nullParam_wdgCfg(void);
void test_negative_Pmic_wdgSetCfg_invalidParam_validParams(void);
void test_negative_Pmic_wdgSetCfg_outOfBounds_mode(void);
void test_negative_Pmic_wdgSetCfg_outOfBounds_trigSel(void);
void test_negative_Pmic_wdgSetCfg_outOfBounds_failThr(void);
void test_negative_Pmic_wdgSetCfg_outOfBounds_rstThr(void);
void test_negative_Pmic_wdgSetCfg_outOfBounds_win1Duration(void);
void test_negative_Pmic_wdgSetCfg_outOfBounds_win2Duration(void);
void test_negative_Pmic_wdgSetCfg_outOfBounds_qaFdbk(void);
void test_negative_Pmic_wdgSetCfg_outOfBounds_qaLfsr(void);
void test_negative_Pmic_wdgSetCfg_outOfBounds_qaSeed(void);
void test_negative_Pmic_wdgGetCfg_nullParam_pmicHandle(void);
void test_negative_Pmic_wdgGetCfg_nullParam_wdgCfg(void);
void test_negative_Pmic_wdgGetCfg_invalidParam_validParams(void);
void test_negative_Pmic_wdgSetPowerHold_nullParam_pmicHandle(void);
void test_negative_Pmic_wdgGetPowerHold_nullParam_pmicHandle(void);
void test_negative_Pmic_wdgGetPowerHold_nullParam_pwrHoldStat(void);
void test_negative_Pmic_wdgSetReturnToLongWindow_nullParam_pmicHandle(void);
void test_negative_Pmic_wdgGetReturnToLongWindow_nullParam_pmicHandle(void);
void test_negative_Pmic_wdgGetReturnToLongWindow_nullParam_retLongWinStat(void);
void test_negative_Pmic_wdgSendSwTrigger_nullParam_pmicHandle(void);
void test_negative_Pmic_wdgQaWriteAnswer_nullParam_pmicHandle(void);
void test_negative_Pmic_wdgClrErrStatus_nullParam_pmicHandle(void);
void test_negative_Pmic_wdgClrErrStatus_nullParam_wdgErrStat(void);
void test_negative_Pmic_wdgClrErrStatus_invalidParam_validParams_zero(void);
void test_negative_Pmic_wdgClrErrStatus_invalidParam_validParams_outOfBounds(void);
void test_negative_Pmic_wdgClrErrStatusAll_nullParam_pmicHandle(void);
void test_negative_Pmic_wdgGetErrStatus_nullParam_pmicHandle(void);
void test_negative_Pmic_wdgGetErrStatus_nullParam_wdgErrStat(void);
void test_negative_Pmic_wdgGetErrStatus_invalidParam_validParams_zero(void);
void test_negative_Pmic_wdgGetErrStatus_invalidParam_validParams_outOfBounds(void);
void test_negative_Pmic_wdgGetFailCntStatus_nullParam_pmicHandle(void);
void test_negative_Pmic_wdgGetFailCntStatus_nullParam_wdgFailCntStat(void);
void test_negative_Pmic_wdgGetFailCntStatus_invalidParam_validParams(void);
void test_positive_wdgEnableDisable(void);
void test_positive_wdgEnableDisablePowerHold(void);
void test_positive_wdgEnableDisableReturnToLongWindow(void);
void test_positive_wdgSetGetCfg_rstEn(void);
void test_positive_wdgSetGetCfg_mode(void);
void test_positive_wdgSetGetCfg_trigSel(void);
void test_positive_wdgSetGetCfg_failThr(void);
void test_positive_wdgSetGetCfg_rstThr(void);
void test_positive_wdgSetGetCfg_longWinDuration(void);
void test_positive_wdgSetGetCfg_win1Duration(void);
void test_positive_wdgSetGetCfg_win2Duration(void);
void test_positive_wdgSetGetCfg_qaFdbk(void);
void test_positive_wdgSetGetCfg_qaLfsr(void);
void test_positive_wdgSetGetCfg_qaSeed(void);
void test_positive_wdgSwTrigger_detectNoErrors(void);
void test_positive_wdgSwTrigger_detectTrigEarlyErr(void);
void test_positive_wdgQaSequence_detectNoErrors(void);
void test_positive_wdgQaSequence_detect_answErr(void);
void test_positive_wdgQaSequence_detect_seqErr(void);
void test_positive_wdgQaSequence_detect_AnswEarlyErr(void);
void test_positive_wdgQaSequence_detect_timeoutErr(void);
void test_positive_wdgQaSequence_detect_longWinTimeoutErr(void);
void test_positive_wdgQaSequence_detect_failInt(void);
void test_positive_wdgQaSequence_detect_RstInt(void);
void test_positive_wdgClrErrStatus_rstInt(void);
void test_positive_wdgClrErrStatus_failInt(void);
void test_positive_wdgClrErrStatus_answErr(void);
void test_positive_wdgClrErrStatus_seqErr(void);
void test_positive_wdgClrErrStatus_answEarlyErr(void);
void test_positive_wdgClrErrStatus_trigEarlyErr(void);
void test_positive_wdgClrErrStatus_timeout(void);
void test_positive_wdgClrErrStatus_longwinTimeout(void);
void test_positive_wdgClrErrStatus_multipleFlags(void);
void test_positive_wdgGetFailCntStatus_copyFunction(void);
void test_positive_wdgClrErrStatus_th1ErrorOnly(void);
void test_positive_wdgClrErrStatus_th2ErrorOnly(void);
void test_positive_wdgClrErrStatus_seqErrorOnly(void);
void test_positive_wdgGetFailCntStatus_failCntOnly(void);
void test_positive_wdgGetFailCntStatus_badCntOnly(void);
void test_positive_wdgSetCfg_qaFdbk1(void);
void test_positive_wdgSetCfg_qaFdbk2(void);
void test_positive_wdgSetCfg_qaFdbk3(void);
void test_positive_wdgQaWithIrqCallback(void);
void test_positive_wdgQaWriteAnswer_qaFdbk0(void);
void test_positive_wdgQaWriteAnswer_qaFdbk1(void);
void test_positive_wdgQaWriteAnswer_qaFdbk2(void);
void test_positive_wdgQaWriteAnswer_qaFdbk3(void);
void test_positive_wdgGetErrStatus_allFields(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* PMIC_TEST_WDG_H */
