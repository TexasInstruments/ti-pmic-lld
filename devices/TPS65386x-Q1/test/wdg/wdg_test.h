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
#include "test_utils.h"

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
/*                         Test Injection Debug                               */
/* ========================================================================== */
void test_pos_wdg_testInject_debug(void);

/* ========================================================================== */
/*                         wdgEnable API Tests                                */
/* ========================================================================== */
void test_pos_wdg_wdgEnable_enableDisable(void);
void test_neg_wdg_wdgEnable_nullHandle(void);

/* ========================================================================== */
/*                         wdgDisable API Tests                               */
/* ========================================================================== */
void test_neg_wdg_wdgDisable_nullHandle(void);

/* ========================================================================== */
/*                       wdgSetEnableState API Tests                          */
/* ========================================================================== */
void test_neg_wdg_wdgSetEnableState_nullHandle(void);

/* ========================================================================== */
/*                       wdgGetEnableState API Tests                          */
/* ========================================================================== */
void test_neg_wdg_wdgGetEnableState_nullHandle(void);
void test_neg_wdg_wdgGetEnableState_nullParam(void);

/* ========================================================================== */
/*                         wdgSetCfg API Tests                                */
/* ========================================================================== */
void test_pos_wdg_wdgSetCfg_longWindowDuration(void);
void test_pos_wdg_wdgSetCfg_window1Duration(void);
void test_pos_wdg_wdgSetCfg_window2Duration(void);
void test_pos_wdg_wdgSetCfg_failThreshold(void);
void test_pos_wdg_wdgSetCfg_resetThreshold(void);
void test_pos_wdg_wdgSetCfg_threshold1IntBehavior(void);
void test_pos_wdg_wdgSetCfg_wdgMode(void);
void test_pos_wdg_wdgSetCfg_threshold2IntBehavior(void);
void test_pos_wdg_wdgSetCfg_returnLongWindow(void);
void test_pos_wdg_wdgSetCfg_QA_feedback(void);
void test_pos_wdg_wdgSetCfg_QA_LFSR(void);
void test_pos_wdg_wdgSetCfg_QA_questionSeed(void);
void test_pos_wdg_wdgSetCfg_timeBase(void);
void test_neg_wdg_wdgSetCfg_nullHandle(void);
void test_neg_wdg_wdgSetCfg_nullConfig(void);
void test_neg_wdg_wdgSetCfg_invalidMode(void);
void test_neg_wdg_wdgSetCfg_invalidTimeBase(void);
void test_neg_wdg_wdgSetCfg_invalidThreshold1(void);
void test_neg_wdg_wdgSetCfg_invalidThreshold2(void);
void test_neg_wdg_wdgSetCfg_invalidQaFdbk(void);
void test_neg_wdg_wdgSetCfg_invalidQaLfsr(void);
void test_neg_wdg_wdgSetCfg_invalidQaQuesSeed(void);
void test_neg_wdg_wdgSetCfg_invalidThreshold1IntBehavior(void);
void test_neg_wdg_wdgSetCfg_invalidThreshold2IntBehavior(void);
void test_neg_wdg_wdgSetCfg_zeroValidParams(void);

/* ========================================================================== */
/*                         wdgGetCfg API Tests                                */
/* ========================================================================== */
void test_neg_wdg_wdgGetCfg_nullHandle(void);
void test_neg_wdg_wdgGetCfg_nullConfig(void);

/* ========================================================================== */
/*                         wdgSetMode API Tests                               */
/* ========================================================================== */
void test_pos_wdg_wdgSetMode_triggerMode(void);
void test_pos_wdg_wdgSetMode_qAndAMode(void);
void test_neg_wdg_wdgSetMode_nullHandle(void);
void test_neg_wdg_wdgSetMode_invalidMode(void);

/* ========================================================================== */
/*                         wdgGetMode API Tests                               */
/* ========================================================================== */
void test_neg_wdg_wdgGetMode_nullHandle(void);
void test_neg_wdg_wdgGetMode_nullParam(void);

/* ========================================================================== */
/*                       wdgSetPowerHold API Tests                            */
/* ========================================================================== */
void test_pos_wdg_wdgSetPowerHold_enable(void);
void test_pos_wdg_wdgSetPowerHold_disable(void);
void test_neg_wdg_wdgSetPowerHold_nullHandle(void);

/* ========================================================================== */
/*                       wdgGetPowerHold API Tests                            */
/* ========================================================================== */
void test_neg_wdg_wdgGetPowerHold_nullHandle(void);
void test_neg_wdg_wdgGetPowerHold_nullParam(void);

/* ========================================================================== */
/*                  wdgSetReturnToLongWindow API Tests                        */
/* ========================================================================== */
void test_pos_wdg_wdgSetReturnToLongWindow_enable(void);
void test_pos_wdg_wdgSetReturnToLongWindow_disable(void);
void test_neg_wdg_wdgSetReturnToLongWindow_nullHandle(void);

/* ========================================================================== */
/*                  wdgGetReturnToLongWindow API Tests                        */
/* ========================================================================== */
void test_neg_wdg_wdgGetReturnToLongWindow_nullHandle(void);
void test_neg_wdg_wdgGetReturnToLongWindow_nullParam(void);

/* ========================================================================== */
/*                      wdgGetErrorStatus API Tests                           */
/* ========================================================================== */
void test_pos_wdg_wdgGetErrorStatus_afterAnswerError(void);
void test_pos_wdg_wdgGetErrorStatus_timeout(void);
void test_pos_wdg_wdgGetErrorStatus_longWindowTimeout(void);
void test_pos_wdg_wdgGetErrorStatus_answerEarlyError(void);
void test_pos_wdg_wdgGetErrorStatus_sequenceErr(void);
void test_pos_wdg_wdgGetErrorStatus_answerErr(void);
void test_pos_wdg_wdgGetErrorStatus_triggerEarly(void);
void test_pos_wdg_wdgGetErrorStatus_th1Int(void);
void test_pos_wdg_wdgGetErrorStatus_th2Int(void);
void test_pos_wdg_wdgGetErrorStatus_allFlags(void);
void test_neg_wdg_wdgGetErrorStatus_nullHandle(void);
void test_neg_wdg_wdgGetErrorStatus_nullParam(void);

/* ========================================================================== */
/*                      wdgClrErrStatus API Tests                             */
/* ========================================================================== */
void test_pos_wdg_wdgClrErrStatus_timeout(void);
void test_pos_wdg_wdgClrErrStatus_longWindowTimeout(void);
void test_pos_wdg_wdgClrErrStatus_answerEarlyError(void);
void test_pos_wdg_wdgClrErrStatus_sequenceErr(void);
void test_pos_wdg_wdgClrErrStatus_answerErr(void);
void test_pos_wdg_wdgClrErrStatus_triggerEarly(void);
void test_pos_wdg_wdgClrErrStatus_th1ErrorOnly(void);
void test_pos_wdg_wdgClrErrStatus_th2ErrorOnly(void);
void test_pos_wdg_wdgClrErrStatus_seqErrorOnly(void);
void test_neg_wdg_wdgClrErrStatus_nullHandle(void);
void test_neg_wdg_wdgClrErrStatus_nullParam(void);

/* ========================================================================== */
/*                     wdgClrErrStatusAll API Tests                           */
/* ========================================================================== */
void test_pos_wdg_wdgClrErrStatusAll_whenNoErrors(void);
void test_neg_wdg_wdgClrErrStatusAll_nullHandle(void);

/* ========================================================================== */
/*                    wdgGetFailCntStatus API Tests                           */
/* ========================================================================== */
void test_pos_wdg_wdgGetFailCntStatus_badEvent(void);
void test_pos_wdg_wdgGetFailCntStatus_goodEvent(void);
void test_pos_wdg_wdgGetFailCntStatus_wdFailCnt(void);
void test_pos_wdg_wdgGetFailCntStatus_allFields(void);
void test_pos_wdg_wdgGetFailCntStatus_failCntOnly(void);
void test_pos_wdg_wdgGetFailCntStatus_badCntOnly(void);
void test_neg_wdg_wdgGetFailCntStatus_nullHandle(void);
void test_neg_wdg_wdgGetFailCntStatus_nullParam(void);

/* ========================================================================== */
/*                      wdgQaWriteAnswer API Tests                            */
/* ========================================================================== */
void test_pos_wdg_wdgQaWriteAnswer_fullSequence(void);
void test_pos_wdg_wdgQaWriteAnswer_qaFdbk0(void);
void test_pos_wdg_wdgQaWriteAnswer_qaFdbk1(void);
void test_pos_wdg_wdgQaWriteAnswer_qaFdbk2(void);
void test_pos_wdg_wdgQaWriteAnswer_qaFdbk3(void);
void test_pos_wdg_wdgQaWriteAnswer_differentSeeds(void);
void test_pos_wdg_wdgQaWriteAnswer_differentLfsr(void);
void test_neg_wdg_wdgQaWriteAnswer_nullHandle(void);

#ifdef __cplusplus
}
#endif

#endif /* WDG_TEST_H */
