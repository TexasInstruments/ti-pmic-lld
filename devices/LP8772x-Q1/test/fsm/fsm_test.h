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
#ifndef __FSM_TEST_H__
#define __FSM_TEST_H__

/**
 * @file fsm_test.h
 * @brief Contains macros/defines and test declarations specific to testing the
 * FSM module.
 */

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

void fsm_test(void *args);

void test_negative_Pmic_fsmMcuCommand_nullParam_handle(void);
void test_negative_Pmic_fsmMcuCommand_invalidParam_cmd(void);
void test_negative_Pmic_fsmSetResetCntThr_nullParam_handle(void);
void test_negative_Pmic_fsmSetResetCntThr_outOfBounds_resetCntThr(void);
void test_negative_Pmic_fsmGetResetCntThr_nullParam_handle(void);
void test_negative_Pmic_fsmGetResetCntThr_nullParam_resetCntThr(void);
void test_negative_Pmic_fsmGetResetCnt_nullParam_handle(void);
void test_negative_Pmic_fsmGetResetCnt_nullParam_resetCnt(void);
void test_negative_Pmic_fsmClrResetCnt_nullParam_handle(void);
void test_negative_Pmic_fsmSetRecovCntThr_nullParam_handle(void);
void test_negative_Pmic_fsmSetRecovCntThr_outOfBounds_recovCntThr(void);
void test_negative_Pmic_fsmGetRecovCntThr_nullParam_handle(void);
void test_negative_Pmic_fsmGetRecovCntThr_nullParam_recovCntThr(void);
void test_negative_Pmic_fsmGetRecovCnt_nullParam_handle(void);
void test_negative_Pmic_fsmGetRecovCnt_nullParam_recovCnt(void);
void test_negative_Pmic_fsmClrRecovCnt_nullParam_handle(void);
void test_positive_setGetResetCntThr(void);
void test_positive_setGetRecovCntThr(void);
void test_positive_Pmic_fsmMcuCommand_coldBootReq(void);
void test_positive_Pmic_fsmMcuCommand_warmResetReq(void);
void test_positive_Pmic_fsmMcuCommand_safeRecovReq(void);
void test_positive_Pmic_fsmMcuCommand_offReq(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /*__FSM_TEST_H__*/
