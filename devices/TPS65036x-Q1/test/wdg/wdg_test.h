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
#ifndef __WDG_TEST_H__
#define __WDG_TEST_H__

/**
 * @file wdg_test.h
 * @brief Contains macros/defines and test declarations specific to testing the
 * WDG module.
 */

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
void test_negative_Pmic_wdgGetEnable_nullParam_pmicHandle(void);
void test_negative_Pmic_wdgGetEnable_nullParam_wdgEnabled(void);
void test_negative_Pmic_wdgSetCfg_nullParam_pmicHandle(void);
void test_negative_Pmic_wdgSetCfg_nullParam_wdgCfg(void);
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
void test_negative_Pmic_wdgSetPwrHold_nullParam_pmicHandle(void);
void test_negative_Pmic_wdgGetPwrHold_nullParam_pmicHandle(void);
void test_negative_Pmic_wdgGetPwrHold_nullParam_pwrHoldStat(void);
void test_negative_Pmic_wdgSetRetLongWin_nullParam_pmicHandle(void);
void test_negative_Pmic_wdgGetRetLongWin_nullParam_pmicHandle(void);
void test_negative_Pmic_wdgGetRetLongWin_nullParam_retLongWinStat(void);
void test_negative_Pmic_wdgSendSwTrigger_nullParam_pmicHandle(void);
void test_negative_Pmic_wdgWriteAnswer_nullParam_pmicHandle(void);
void test_negative_Pmic_wdgClrErrStat_nullParam_pmicHandle(void);
void test_negative_Pmic_wdgClrErrStat_nullParam_wdgErrStat(void);
void test_negative_Pmic_wdgClrErrStatAll_nullParam_pmicHandle(void);
void test_negative_Pmic_wdgGetErrStat_nullParam_pmicHandle(void);
void test_negative_Pmic_wdgGetErrStat_nullParam_wdgErrStat(void);
void test_negative_Pmic_wdgGetFailCntStat_nullParam_pmicHandle(void);
void test_negative_Pmic_wdgGetFailCntStat_nullParam_wdgFailCntStat(void);
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

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /*__WDG_TEST_H__*/
