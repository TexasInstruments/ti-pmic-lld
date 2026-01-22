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
#ifndef PMIC_TEST_WDG_H
#define PMIC_TEST_WDG_H



/* ========================================================================== */
/*                              Include Files                                 */
/* ========================================================================== */

#include "test_utils.h"
#include "wdg_test_macros.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

void wdg_test(void *args);

void test_neg_wdg_wdgSetEnableState_nullHandle(void);
void test_neg_wdg_wdgEnable_nullHandle(void);
void test_neg_wdg_wdgDisable_nullHandle(void);
void test_neg_wdg_wdgGetEnableState_nullHandle(void);
void test_neg_wdg_wdgGetEnableState_nullParam(void);
void test_neg_wdg_wdgSetCfg_nullHandle(void);
void test_neg_wdg_wdgSetCfg_nullConfig(void);
void test_neg_wdg_wdgSetCfg_invalidParam(void);
void test_neg_wdg_wdgSetCfg_invalidMode(void);
void test_neg_wdg_wdgSetCfg_invalidTrigSel(void);
void test_neg_wdg_wdgSetCfg_invalidFailThr(void);
void test_neg_wdg_wdgSetCfg_invalidRstThr(void);
void test_neg_wdg_wdgSetCfg_invalidWin1Duration(void);
void test_neg_wdg_wdgSetCfg_invalidWin2Duration(void);
void test_neg_wdg_wdgSetCfg_invalidQaFdbk(void);
void test_neg_wdg_wdgSetCfg_invalidQaLfsr(void);
void test_neg_wdg_wdgSetCfg_invalidQaSeed(void);
void test_neg_wdg_wdgGetCfg_nullHandle(void);
void test_neg_wdg_wdgGetCfg_nullConfig(void);
void test_neg_wdg_wdgGetCfg_invalidParam(void);
void test_neg_wdg_wdgSetPowerHold_nullHandle(void);
void test_neg_wdg_wdgGetPowerHold_nullHandle(void);
void test_neg_wdg_wdgGetPowerHold_nullParam(void);
void test_neg_wdg_wdgSetReturnToLongWindow_nullHandle(void);
void test_neg_wdg_wdgGetReturnToLongWindow_nullHandle(void);
void test_neg_wdg_wdgGetReturnToLongWindow_nullParam(void);
void test_neg_wdg_wdgSendSwTrigger_nullHandle(void);
void test_neg_wdg_wdgQaWriteAnswer_nullHandle(void);
void test_neg_wdg_wdgClrErrStatus_nullHandle(void);
void test_neg_wdg_wdgClrErrStatus_nullParam(void);
void test_neg_wdg_wdgClrErrStatus_invalidParamZero(void);
void test_neg_wdg_wdgClrErrStatus_invalidParamOutOfBounds(void);
void test_neg_wdg_wdgClrErrStatusAll_nullHandle(void);
void test_neg_wdg_wdgGetErrStatus_nullHandle(void);
void test_neg_wdg_wdgGetErrStatus_nullParam(void);
void test_neg_wdg_wdgGetErrStatus_invalidParamZero(void);
void test_neg_wdg_wdgGetErrStatus_invalidParamOutOfBounds(void);
void test_neg_wdg_wdgGetFailCntStatus_nullHandle(void);
void test_neg_wdg_wdgGetFailCntStatus_nullParam(void);
void test_neg_wdg_wdgGetFailCntStatus_invalidParam(void);
void test_pos_wdg_wdgEnable_enableDisable(void);
void test_pos_wdg_wdgSetPowerHold_enableDisable(void);
void test_pos_wdg_wdgSetReturnToLongWindow_enableDisable(void);
void test_pos_wdg_wdgSetCfg_rstEn(void);
void test_pos_wdg_wdgSetCfg_mode(void);
void test_pos_wdg_wdgSetCfg_trigSel(void);
void test_pos_wdg_wdgSetCfg_failThr(void);
void test_pos_wdg_wdgSetCfg_rstThr(void);
void test_pos_wdg_wdgSetCfg_longWinDuration(void);
void test_pos_wdg_wdgSetCfg_win1Duration(void);
void test_pos_wdg_wdgSetCfg_win2Duration(void);
void test_pos_wdg_wdgSetCfg_qaFdbk(void);
void test_pos_wdg_wdgSetCfg_qaLfsr(void);
void test_pos_wdg_wdgSetCfg_qaSeed(void);
void test_pos_wdg_wdgSendSwTrigger_detectNoErrors(void);
void test_pos_wdg_wdgSendSwTrigger_detectTrigEarlyErr(void);
void test_pos_wdg_wdgQaSequence_detectNoErrors(void);
void test_pos_wdg_wdgQaSequence_detectAnswErr(void);
void test_pos_wdg_wdgQaSequence_detectSeqErr(void);
void test_pos_wdg_wdgQaSequence_detectAnswEarlyErr(void);
void test_pos_wdg_wdgQaSequence_detectTimeoutErr(void);
void test_pos_wdg_wdgQaSequence_detectLongWinTimeoutErr(void);
void test_pos_wdg_wdgQaSequence_detectFailInt(void);
void test_pos_wdg_wdgQaSequence_detectRstInt(void);
void test_pos_wdg_wdgClrErrStatus_rstInt(void);
void test_pos_wdg_wdgClrErrStatus_failInt(void);
void test_pos_wdg_wdgClrErrStatus_answErr(void);
void test_pos_wdg_wdgClrErrStatus_seqErr(void);
void test_pos_wdg_wdgClrErrStatus_answEarlyErr(void);
void test_pos_wdg_wdgClrErrStatus_trigEarlyErr(void);
void test_pos_wdg_wdgClrErrStatus_timeout(void);
void test_pos_wdg_wdgClrErrStatus_longwinTimeout(void);
void test_pos_wdg_wdgClrErrStatus_multipleFlags(void);
void test_pos_wdg_wdgGetFailCntStatus_copyFunction(void);
void test_pos_wdg_wdgClrErrStatus_th1ErrorOnly(void);
void test_pos_wdg_wdgClrErrStatus_th2ErrorOnly(void);
void test_pos_wdg_wdgClrErrStatus_seqErrorOnly(void);
void test_pos_wdg_wdgGetFailCntStatus_failCntOnly(void);
void test_pos_wdg_wdgGetFailCntStatus_badCntOnly(void);
void test_pos_wdg_wdgSetCfg_qaFdbk1(void);
void test_pos_wdg_wdgSetCfg_qaFdbk2(void);
void test_pos_wdg_wdgSetCfg_qaFdbk3(void);
void test_pos_wdg_wdgQaSequence_qaWithIrqCallback(void);
void test_pos_wdg_wdgQaWriteAnswer_qaFdbk0(void);
void test_pos_wdg_wdgQaWriteAnswer_qaFdbk1(void);
void test_pos_wdg_wdgQaWriteAnswer_qaFdbk2(void);
void test_pos_wdg_wdgQaWriteAnswer_qaFdbk3(void);
void test_pos_wdg_wdgGetErrStatus_allFields(void);
void test_pos_wdg_wdgQaWriteAnswer_fullSequence(void);
void test_pos_wdg_wdgGetErrStatus_answerError(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* PMIC_TEST_WDG_H */
