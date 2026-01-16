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
/*                             Include Files                                  */
/* ========================================================================== */

#include "unity.h"
#include "platform.h"
#include "pmic.h"
#include "pmic_wdg.h"
#include "test_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Function Declarations                            */
/* ========================================================================== */

/**
 * @brief Run all WDG tests
 * @param args Test arguments (unused)
 */
void wdg_test(void *args);

/* ========================================================================== */
/* Positive Tests                                                             */
/* ========================================================================== */
extern void test_positive_wdgEnableDisable(void);
extern void test_positive_wdgSetGetCfg_longWindowDuration(void);
extern void test_positive_wdgSetGetCfg_window1Duration(void);
extern void test_positive_wdgSetGetCfg_window2Duration(void);
extern void test_positive_wdgSetGetCfg_failThreshold(void);
extern void test_positive_wdgSetGetCfg_resetThreshold(void);
extern void test_positive_wdgSetGetCfg_threshold1IntBehavior(void);
extern void test_positive_wdgSetGetCfg_wdgMode(void);
extern void test_positive_wdgSetGetCfg_threshold2IntBehavior(void);
extern void test_positive_wdgSetGetCfg_returnLongWindow(void);
extern void test_positive_wdgSetGetCfg_QA_feedback(void);
extern void test_positive_wdgSetGetCfg_QA_LFSR(void);
extern void test_positive_wdgSetGetCfg_QA_questionSeed(void);

/* Test Injection Debug */
extern void test_positive_wdgTestInjectDebug(void);

/* Q&A Sequence Tests */
extern void test_positive_wdgQaWriteAnswer_fullSequence(void);
extern void test_positive_wdgQaWriteAnswer_qaFdbk0(void);
extern void test_positive_wdgQaWriteAnswer_qaFdbk1(void);
extern void test_positive_wdgQaWriteAnswer_qaFdbk2(void);
extern void test_positive_wdgQaWriteAnswer_qaFdbk3(void);
extern void test_positive_wdgQaWriteAnswer_differentSeeds(void);
extern void test_positive_wdgQaWriteAnswer_differentLfsr(void);
extern void test_positive_wdgGetErrorStatus_afterAnswerError(void);

/* Error Status Get Tests */
extern void test_positive_wdgGetErrorStatus_timeout(void);
extern void test_positive_wdgGetErrorStatus_longWindowTimeout(void);
extern void test_positive_wdgGetErrorStatus_answerEarlyError(void);
extern void test_positive_wdgGetErrorStatus_sequenceErr(void);
extern void test_positive_wdgGetErrorStatus_answerErr(void);
extern void test_positive_wdgGetErrorStatus_triggerEarly(void);
extern void test_positive_wdgGetErrorStatus_th1Int(void);
extern void test_positive_wdgGetErrorStatus_th2Int(void);
extern void test_positive_wdgGetErrorStatus_allFlags(void);

/* Clear Error Status Tests */
extern void test_positive_wdgClrErrStatus_timeout(void);
extern void test_positive_wdgClrErrStatus_longWindowTimeout(void);
extern void test_positive_wdgClrErrStatus_answerEarlyError(void);
extern void test_positive_wdgClrErrStatus_sequenceErr(void);
extern void test_positive_wdgClrErrStatus_answerErr(void);
extern void test_positive_wdgClrErrStatus_triggerEarly(void);
extern void test_positive_wdgClrErrStatusAll_whenNoErrors(void);

/* Fail Count Status Tests */
extern void test_positive_wdgGetFailCntStatus_badEvent(void);
extern void test_positive_wdgGetFailCntStatus_goodEvent(void);
extern void test_positive_wdgGetFailCntStatus_wdFailCnt(void);
extern void test_positive_wdgGetFailCntStatus_allFields(void);

/* Configuration Tests */
extern void test_positive_wdgSetGetCfg_timeBase(void);
extern void test_negative_wdgSetCfg_zeroValidParams(void);

/* Mode Tests */
extern void test_positive_wdgSetGetMode_triggerMode(void);
extern void test_positive_wdgSetGetMode_qAndAMode(void);

/* Power Hold Tests */
extern void test_positive_wdgSetGetPowerHold_enable(void);
extern void test_positive_wdgSetGetPowerHold_disable(void);

/* Return To Long Window Tests */
extern void test_positive_wdgSetGetReturnToLongWindow_enable(void);
extern void test_positive_wdgSetGetReturnToLongWindow_disable(void);

/* ========================================================================== */
/* Negative Tests - Enable/Disable APIs                                      */
/* ========================================================================== */
extern void test_negative_Pmic_wdgEnable_nullHandle(void);
extern void test_negative_Pmic_wdgDisable_nullHandle(void);
extern void test_negative_Pmic_wdgSetEnableState_nullHandle(void);
extern void test_negative_Pmic_wdgGetEnableState_nullHandle(void);
extern void test_negative_Pmic_wdgGetEnableState_nullParam(void);

/* ========================================================================== */
/* Negative Tests - Configuration APIs                                       */
/* ========================================================================== */
extern void test_negative_Pmic_wdgSetCfg_nullHandle(void);
extern void test_negative_Pmic_wdgSetCfg_nullConfig(void);
extern void test_negative_Pmic_wdgSetCfg_invalidMode(void);
extern void test_negative_Pmic_wdgSetCfg_invalidTimeBase(void);
extern void test_negative_Pmic_wdgSetCfg_invalidThreshold1(void);
extern void test_negative_Pmic_wdgSetCfg_invalidThreshold2(void);
extern void test_negative_Pmic_wdgSetCfg_invalidQaFdbk(void);
extern void test_negative_Pmic_wdgSetCfg_invalidQaLfsr(void);
extern void test_negative_Pmic_wdgSetCfg_invalidQaQuesSeed(void);
extern void test_negative_Pmic_wdgSetCfg_invalidThreshold1IntBehavior(void);
extern void test_negative_Pmic_wdgSetCfg_invalidThreshold2IntBehavior(void);
extern void test_negative_Pmic_wdgGetCfg_nullHandle(void);
extern void test_negative_Pmic_wdgGetCfg_nullConfig(void);

/* ========================================================================== */
/* Negative Tests - Mode APIs                                                */
/* ========================================================================== */
extern void test_negative_Pmic_wdgSetMode_nullHandle(void);
extern void test_negative_Pmic_wdgSetMode_invalidMode(void);
extern void test_negative_Pmic_wdgGetMode_nullHandle(void);
extern void test_negative_Pmic_wdgGetMode_nullParam(void);

/* ========================================================================== */
/* Negative Tests - Power Hold APIs                                          */
/* ========================================================================== */
extern void test_negative_Pmic_wdgSetPowerHold_nullHandle(void);
extern void test_negative_Pmic_wdgGetPowerHold_nullHandle(void);
extern void test_negative_Pmic_wdgGetPowerHold_nullParam(void);

/* ========================================================================== */
/* Negative Tests - Return to Long Window APIs                               */
/* ========================================================================== */
extern void test_negative_Pmic_wdgSetReturnToLongWindow_nullHandle(void);
extern void test_negative_Pmic_wdgGetReturnToLongWindow_nullHandle(void);
extern void test_negative_Pmic_wdgGetReturnToLongWindow_nullParam(void);

/* ========================================================================== */
/* Negative Tests - Error Status APIs                                        */
/* ========================================================================== */
extern void test_negative_Pmic_wdgGetErrorStatus_nullHandle(void);
extern void test_negative_Pmic_wdgGetErrorStatus_nullParam(void);
extern void test_negative_Pmic_wdgClrErrStatus_nullHandle(void);
extern void test_negative_Pmic_wdgClrErrStatus_nullParam(void);
extern void test_negative_Pmic_wdgClrErrStatusAll_nullHandle(void);

/* ========================================================================== */
/* Negative Tests - Fail Count APIs                                          */
/* ========================================================================== */
extern void test_negative_Pmic_wdgGetFailCntStatus_nullHandle(void);
extern void test_negative_Pmic_wdgGetFailCntStatus_nullParam(void);

/* ========================================================================== */
/* Negative Tests - Q&A APIs                                                 */
/* ========================================================================== */
extern void test_negative_Pmic_wdgQaWriteAnswer_nullHandle(void);

#ifdef __cplusplus
}
#endif

#endif /* WDG_TEST_H */
