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
#ifndef FSM_TEST_H
#define FSM_TEST_H



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

void test_neg_fsm_fsmClrRecovCnt_nullHandle(void);
void test_neg_fsm_fsmClrResetCnt_nullHandle(void);
void test_neg_fsm_fsmGetRecovCnt_nullHandle(void);
void test_neg_fsm_fsmGetRecovCnt_nullRecovCnt(void);
void test_neg_fsm_fsmGetRecovCntThr_nullHandle(void);
void test_neg_fsm_fsmGetRecovCntThr_nullRecovCntThr(void);
void test_neg_fsm_fsmGetResetCnt_nullHandle(void);
void test_neg_fsm_fsmGetResetCnt_nullResetCnt(void);
void test_neg_fsm_fsmGetResetCntThr_nullHandle(void);
void test_neg_fsm_fsmGetResetCntThr_nullResetCntThr(void);
void test_neg_fsm_fsmSetDevState_invalidCmd(void);
void test_neg_fsm_fsmSetDevState_nullHandle(void);
void test_neg_fsm_fsmSetRecovCntThr_nullHandle(void);
void test_neg_fsm_fsmSetRecovCntThr_outOfBoundsRecovCntThr(void);
void test_neg_fsm_fsmSetResetCntThr_nullHandle(void);
void test_neg_fsm_fsmSetResetCntThr_outOfBoundsResetCntThr(void);
void test_pos_fsm_fsmSetDevState_coldBootReq(void);
void test_pos_fsm_fsmSetDevState_offReq(void);
void test_pos_fsm_fsmSetDevState_safeRecovReq(void);
void test_pos_fsm_fsmSetDevState_warmResetReq(void);
void test_pos_fsm_setGetRecovCntThr(void);
void test_pos_fsm_setGetResetCntThr(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /*__FSM_TEST_H__*/
