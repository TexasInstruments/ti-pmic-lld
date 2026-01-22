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

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#include "wdg_test_macros.h"

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

void wdg_test(void *args);

/* Negative Test Functions */
void test_neg_wdg_wdgClrErrStatusAll_nullHandle(void);
void test_neg_wdg_wdgClrErrStatus_nullHandle(void);
void test_neg_wdg_wdgClrErrStatus_nullParam(void);
void test_neg_wdg_wdgDisable_nullHandle(void);
void test_neg_wdg_wdgEnable_nullHandle(void);
void test_neg_wdg_wdgExtractAnsCntAndQues_nullHandle(void);
void test_neg_wdg_wdgExtractAnsCntAndQues_nullParam(void);
void test_neg_wdg_wdgExtractFdbk_nullParam(void);
void test_neg_wdg_wdgGetAnsCntAndQuesRegData_nullHandle(void);
void test_neg_wdg_wdgGetAnsCntAndQuesRegData_nullParam(void);
void test_neg_wdg_wdgGetCfg_nullConfig(void);
void test_neg_wdg_wdgGetCfg_nullHandle(void);
void test_neg_wdg_wdgGetEnableState_nullHandle(void);
void test_neg_wdg_wdgGetEnableState_nullParam(void);
void test_neg_wdg_wdgGetErrorStatus_nullHandle(void);
void test_neg_wdg_wdgGetErrorStatus_nullParam(void);
void test_neg_wdg_wdgGetFailCntStatus_nullHandle(void);
void test_neg_wdg_wdgGetFailCntStatus_nullParam(void);
void test_neg_wdg_wdgGetFdbkRegData_nullHandle(void);
void test_neg_wdg_wdgGetFdbkRegData_nullParam(void);
void test_neg_wdg_wdgGetPowerHold_nullHandle(void);
void test_neg_wdg_wdgGetPowerHold_nullParam(void);
void test_neg_wdg_wdgGetReturnToLongWindow_nullHandle(void);
void test_neg_wdg_wdgGetReturnToLongWindow_nullParam(void);
void test_neg_wdg_wdgQaWriteAnswer_nullHandle(void);
void test_neg_wdg_wdgSetCfg_invalidQaFdbk(void);
void test_neg_wdg_wdgSetCfg_invalidQaLfsr(void);
void test_neg_wdg_wdgSetCfg_invalidQaQuesSeed(void);
void test_neg_wdg_wdgSetCfg_invalidThreshold1(void);
void test_neg_wdg_wdgSetCfg_invalidThreshold2(void);
void test_neg_wdg_wdgSetCfg_invalidWin1Code(void);
void test_neg_wdg_wdgSetCfg_invalidWin2Code(void);
void test_neg_wdg_wdgSetCfg_nullConfig(void);
void test_neg_wdg_wdgSetCfg_nullHandle(void);
void test_neg_wdg_wdgSetEnableState_nullHandle(void);
void test_neg_wdg_wdgSetPowerHold_nullHandle(void);
void test_neg_wdg_wdgSetReturnToLongWindow_nullHandle(void);
void test_neg_wdg_wdgWriteAnswer_nullHandle(void);
void test_neg_wdg_wdgWriteAnswer_nullParam(void);
void test_pos_wdg_wdgClrErrStatusAll_optimization(void);
void test_pos_wdg_wdgClrErrStatus_seqErrorOnly(void);
void test_pos_wdg_wdgClrErrStatus_th1ErrorOnly(void);
void test_pos_wdg_wdgClrErrStatus_th2ErrorOnly(void);
void test_pos_wdg_wdgEnable_enableDisable(void);
void test_pos_wdg_wdgGetErrorStatus_answerEarly(void);
void test_pos_wdg_wdgGetErrorStatus_answerError(void);
void test_pos_wdg_wdgGetErrorStatus_longWindowTimeout(void);
void test_pos_wdg_wdgGetErrorStatus_timeout(void);
void test_pos_wdg_wdgGetFailCntStatus_badCntOnly(void);
void test_pos_wdg_wdgGetFailCntStatus_failCntOnly(void);
void test_pos_wdg_wdgGetFailCntStatus_goodEvent(void);
void test_pos_wdg_wdgQaSequence_answerEarly(void);
void test_pos_wdg_wdgQaSequence_answerError(void);
void test_pos_wdg_wdgQaSequence_failInt(void);
void test_pos_wdg_wdgQaSequence_longWindowTimeout(void);
void test_pos_wdg_wdgQaSequence_noErrors(void);
void test_pos_wdg_wdgQaSequence_qaWithIrqCallback(void);
void test_pos_wdg_wdgQaSequence_resetInt(void);
void test_pos_wdg_wdgQaSequence_sequenceError(void);
void test_pos_wdg_wdgQaSequence_withIrqCallback(void);
void test_pos_wdg_wdgQaWriteAnswer_qaFdbk0(void);
void test_pos_wdg_wdgQaWriteAnswer_qaFdbk1(void);
void test_pos_wdg_wdgQaWriteAnswer_qaFdbk2(void);
void test_pos_wdg_wdgQaWriteAnswer_qaFdbk3(void);
void test_pos_wdg_wdgSetCfg_longWindowCode(void);
void test_pos_wdg_wdgSetCfg_qaFdbk(void);
void test_pos_wdg_wdgSetCfg_qaLfsr(void);
void test_pos_wdg_wdgSetCfg_qaQuesSeed(void);
void test_pos_wdg_wdgSetCfg_resetEnable(void);
void test_pos_wdg_wdgSetCfg_threshold1(void);
void test_pos_wdg_wdgSetCfg_threshold2(void);
void test_pos_wdg_wdgSetCfg_win1Code(void);
void test_pos_wdg_wdgSetCfg_win2Code(void);
void test_pos_wdg_wdgSetPowerHold_enableDisable(void);
void test_pos_wdg_wdgSetReturnToLongWindow_enableDisable(void);
void test_pos_wdg_wdgWriteAnswer_success(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* WDG_TEST_H */
