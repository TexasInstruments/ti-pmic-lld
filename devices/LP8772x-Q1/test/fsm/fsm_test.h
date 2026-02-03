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
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* ======================================================================== */
/*                        Test APIs: fsmClrRecovCnt                         */
/* ======================================================================== */

#define FSM_TEST_NEG_FSMCLRRECOVCNT() \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmClrRecovCnt_nullHandle)

/* Test: TC-FSM-0026 */
#define FSM_TEST_FSMCLRRECOVCNT() \
    FSM_TEST_NEG_FSMCLRRECOVCNT()

/* ======================================================================== */
/*                        Test APIs: fsmClrResetCnt                         */
/* ======================================================================== */

#define FSM_TEST_NEG_FSMCLRRESETCNT() \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmClrResetCnt_nullHandle)

/* Test: TC-FSM-0027 */
#define FSM_TEST_FSMCLRRESETCNT() \
    FSM_TEST_NEG_FSMCLRRESETCNT()

/* ======================================================================== */
/*                        Test APIs: fsmGetRecovCnt                         */
/* ======================================================================== */

#define FSM_TEST_NEG_FSMGETRECOVCNT() \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmGetRecovCnt_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmGetRecovCnt_nullRecovCnt)

/* Test: TC-FSM-0028 */
#define FSM_TEST_FSMGETRECOVCNT() \
    FSM_TEST_NEG_FSMGETRECOVCNT()

/* ======================================================================== */
/*                       Test APIs: fsmGetRecovCntThr                       */
/* ======================================================================== */

#define FSM_TEST_POS_FSMGETRECOVCNTTHR() \
    PLATFORM_RUN_TEST(test_pos_fsm_setGetRecovCntThr)

#define FSM_TEST_NEG_FSMGETRECOVCNTTHR() \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmGetRecovCntThr_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmGetRecovCntThr_nullRecovCntThr)

/* Test: TC-FSM-0029 */
#define FSM_TEST_FSMGETRECOVCNTTHR() \
    FSM_TEST_POS_FSMGETRECOVCNTTHR(); \
    FSM_TEST_NEG_FSMGETRECOVCNTTHR()

/* ======================================================================== */
/*                        Test APIs: fsmGetResetCnt                         */
/* ======================================================================== */

#define FSM_TEST_NEG_FSMGETRESETCNT() \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmGetResetCnt_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmGetResetCnt_nullResetCnt)

/* Test: TC-FSM-0030 */
#define FSM_TEST_FSMGETRESETCNT() \
    FSM_TEST_NEG_FSMGETRESETCNT()

/* ======================================================================== */
/*                       Test APIs: fsmGetResetCntThr                       */
/* ======================================================================== */

#define FSM_TEST_POS_FSMGETRESETCNTTHR() \
    PLATFORM_RUN_TEST(test_pos_fsm_setGetResetCntThr)

#define FSM_TEST_NEG_FSMGETRESETCNTTHR() \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmGetResetCntThr_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmGetResetCntThr_nullResetCntThr)

/* Test: TC-FSM-0031 */
#define FSM_TEST_FSMGETRESETCNTTHR() \
    FSM_TEST_POS_FSMGETRESETCNTTHR(); \
    FSM_TEST_NEG_FSMGETRESETCNTTHR()

/* ======================================================================== */
/*                        Test APIs: fsmSetDevState                         */
/* ======================================================================== */

#define FSM_TEST_POS_FSMSETDEVSTATE() \
    PLATFORM_RUN_TEST(test_pos_fsm_fsmSetDevState_coldBootReq); \
    PLATFORM_RUN_TEST(test_pos_fsm_fsmSetDevState_offReq); \
    PLATFORM_RUN_TEST(test_pos_fsm_fsmSetDevState_safeRecovReq); \
    PLATFORM_RUN_TEST(test_pos_fsm_fsmSetDevState_warmResetReq)

#define FSM_TEST_NEG_FSMSETDEVSTATE() \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmSetDevState_invalidCmd); \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmSetDevState_nullHandle)

/* Test: TC-FSM-0032 */
#define FSM_TEST_FSMSETDEVSTATE() \
    FSM_TEST_POS_FSMSETDEVSTATE(); \
    FSM_TEST_NEG_FSMSETDEVSTATE()

/* ======================================================================== */
/*                       Test APIs: fsmSetRecovCntThr                       */
/* ======================================================================== */

#define FSM_TEST_POS_FSMSETRECOVCNTTHR() \
    PLATFORM_RUN_TEST(test_pos_fsm_setGetRecovCntThr)

#define FSM_TEST_NEG_FSMSETRECOVCNTTHR() \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmSetRecovCntThr_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmSetRecovCntThr_outOfBoundsRecovCntThr)

/* Test: TC-FSM-0033 */
#define FSM_TEST_FSMSETRECOVCNTTHR() \
    FSM_TEST_POS_FSMSETRECOVCNTTHR(); \
    FSM_TEST_NEG_FSMSETRECOVCNTTHR()

/* ======================================================================== */
/*                       Test APIs: fsmSetResetCntThr                       */
/* ======================================================================== */

#define FSM_TEST_POS_FSMSETRESETCNTTHR() \
    PLATFORM_RUN_TEST(test_pos_fsm_setGetResetCntThr)

#define FSM_TEST_NEG_FSMSETRESETCNTTHR() \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmSetResetCntThr_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmSetResetCntThr_outOfBoundsResetCntThr)

/* Test: TC-FSM-0034 */
#define FSM_TEST_FSMSETRESETCNTTHR() \
    FSM_TEST_POS_FSMSETRESETCNTTHR(); \
    FSM_TEST_NEG_FSMSETRESETCNTTHR()

/* ========================================================================== */
/*                         Aggregate Test Runners                             */
/* ========================================================================== */

#define FSM_TEST_RUN_POSITIVE() \
    FSM_TEST_POS_FSMGETRECOVCNTTHR(); \
    FSM_TEST_POS_FSMGETRESETCNTTHR(); \
    FSM_TEST_POS_FSMSETDEVSTATE(); \
    FSM_TEST_POS_FSMSETRECOVCNTTHR(); \
    FSM_TEST_POS_FSMSETRESETCNTTHR()

#define FSM_TEST_RUN_NEGATIVE() \
    FSM_TEST_NEG_FSMCLRRECOVCNT(); \
    FSM_TEST_NEG_FSMCLRRESETCNT(); \
    FSM_TEST_NEG_FSMGETRECOVCNT(); \
    FSM_TEST_NEG_FSMGETRECOVCNTTHR(); \
    FSM_TEST_NEG_FSMGETRESETCNT(); \
    FSM_TEST_NEG_FSMGETRESETCNTTHR(); \
    FSM_TEST_NEG_FSMSETDEVSTATE(); \
    FSM_TEST_NEG_FSMSETRECOVCNTTHR(); \
    FSM_TEST_NEG_FSMSETRESETCNTTHR()

#define FSM_TEST_RUN_ALL() \
    FSM_TEST_RUN_POSITIVE(); \
    FSM_TEST_RUN_NEGATIVE()

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
