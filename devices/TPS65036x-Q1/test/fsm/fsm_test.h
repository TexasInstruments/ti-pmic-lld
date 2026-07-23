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
#ifndef PMIC_TEST_FSM_H
#define PMIC_TEST_FSM_H

/* ========================================================================== */
/*                              Include Files                                 */
/* ========================================================================== */

#include "test_utils.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* ======================================================================== */
/*                        Test APIs: fsmSetDevState                         */
/* ======================================================================== */
#define FSM_TEST_POS_FSMSETDEVSTATE() \
    PLATFORM_RUN_TEST(test_pos_fsm_fsmSetDevState_safeRecovery); \
    PLATFORM_RUN_TEST(test_pos_fsm_fsmSetDevState_warmReset); \
    PLATFORM_RUN_TEST(test_pos_fsm_fsmSetDevState_lowPowerEntryExit); \
    PLATFORM_RUN_TEST(test_pos_fsm_fsmSetDevState_coldBootRequest); \
    PLATFORM_RUN_TEST(test_pos_fsm_fsmSetDevState_offRequest)

#define FSM_TEST_NEG_FSMSETDEVSTATE() \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmSetDevState_nullParam_pmicHandle); \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmSetDevState_invalid_fsmCmd)

/* Test: TC-FSM-0035 */
#define FSM_TEST_FSMSETDEVSTATE() \
    FSM_TEST_POS_FSMSETDEVSTATE(); \
    FSM_TEST_NEG_FSMSETDEVSTATE()

/* ======================================================================== */
/*                      Test APIs: fsmSetRecovCntThr                        */
/* ======================================================================== */
#define FSM_TEST_POS_FSMSETRECOVCNTTHR() \
    PLATFORM_RUN_TEST(test_pos_fsm_setGetRecovCntThr)

#define FSM_TEST_NEG_FSMSETRECOVCNTTHR() \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmSetRecovCntThr_nullParam_pmicHandle); \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmSetRecovCntThr_outOfBounds_threshold)

/* Test: TC-FSM-0036 */
#define FSM_TEST_FSMSETRECOVCNTTHR() \
    FSM_TEST_POS_FSMSETRECOVCNTTHR(); \
    FSM_TEST_NEG_FSMSETRECOVCNTTHR()

/* ======================================================================== */
/*                      Test APIs: fsmGetRecovCntThr                        */
/* ======================================================================== */
#define FSM_TEST_NEG_FSMGETRECOVCNTTHR() \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmGetRecovCntThr_nullParam_pmicHandle); \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmGetRecovCntThr_nullParam_threshold)

/* Test: TC-FSM-0037 */
#define FSM_TEST_FSMGETRECOVCNTTHR() \
    FSM_TEST_NEG_FSMGETRECOVCNTTHR()

/* ======================================================================== */
/*                       Test APIs: fsmGetRecovCnt                          */
/* ======================================================================== */
#define FSM_TEST_POS_FSMGETRECOVCNT() \
    PLATFORM_RUN_TEST(test_pos_fsm_getClrRecovCnt)

#define FSM_TEST_NEG_FSMGETRECOVCNT() \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmGetRecovCnt_nullParam_pmicHandle); \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmGetRecovCnt_nullParam_recovCnt)

/* Test: TC-FSM-0038 */
#define FSM_TEST_FSMGETRECOVCNT() \
    FSM_TEST_POS_FSMGETRECOVCNT(); \
    FSM_TEST_NEG_FSMGETRECOVCNT()

/* ======================================================================== */
/*                       Test APIs: fsmClrRecovCnt                          */
/* ======================================================================== */
#define FSM_TEST_NEG_FSMCLRRECOVCNT() \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmClrRecovCnt_nullParam_pmicHandle)

/* Test: TC-FSM-0039 */
#define FSM_TEST_FSMCLRRECOVCNT() \
    FSM_TEST_NEG_FSMCLRRECOVCNT()

/* ======================================================================== */
/*                      Test APIs: fsmSetResetCntThr                        */
/* ======================================================================== */
#define FSM_TEST_POS_FSMSETRESETCNTTHR() \
    PLATFORM_RUN_TEST(test_pos_fsm_setGetResetCntThr)

#define FSM_TEST_NEG_FSMSETRESETCNTTHR() \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmSetResetCntThr_nullParam_pmicHandle); \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmSetResetCntThr_outOfBounds_threshold)

/* Test: TC-FSM-0040 */
#define FSM_TEST_FSMSETRESETCNTTHR() \
    FSM_TEST_POS_FSMSETRESETCNTTHR(); \
    FSM_TEST_NEG_FSMSETRESETCNTTHR()

/* ======================================================================== */
/*                      Test APIs: fsmGetResetCntThr                        */
/* ======================================================================== */
#define FSM_TEST_NEG_FSMGETRESETCNTTHR() \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmGetResetCntThr_nullParam_pmicHandle); \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmGetResetCntThr_nullParam_threshold)

/* Test: TC-FSM-0041 */
#define FSM_TEST_FSMGETRESETCNTTHR() \
    FSM_TEST_NEG_FSMGETRESETCNTTHR()

/* ======================================================================== */
/*                       Test APIs: fsmGetResetCnt                          */
/* ======================================================================== */
#define FSM_TEST_POS_FSMGETRESETCNT() \
    PLATFORM_RUN_TEST(test_pos_fsm_getClrResetCnt)

#define FSM_TEST_NEG_FSMGETRESETCNT() \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmGetResetCnt_nullParam_pmicHandle); \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmGetResetCnt_nullParam_resetCnt)

/* Test: TC-FSM-0042 */
#define FSM_TEST_FSMGETRESETCNT() \
    FSM_TEST_POS_FSMGETRESETCNT(); \
    FSM_TEST_NEG_FSMGETRESETCNT()

/* ======================================================================== */
/*                       Test APIs: fsmClrResetCnt                          */
/* ======================================================================== */
#define FSM_TEST_NEG_FSMCLRRESETCNT() \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmClrResetCnt_nullParam_pmicHandle)

/* Test: TC-FSM-0043 */
#define FSM_TEST_FSMCLRRESETCNT() \
    FSM_TEST_NEG_FSMCLRRESETCNT()

/* ========================================================================== */
/*                        Aggregate Test Macros                               */
/* ========================================================================== */

#define FSM_TEST_RUN_POSITIVE() \
    FSM_TEST_POS_FSMSETDEVSTATE(); \
    FSM_TEST_POS_FSMSETRECOVCNTTHR(); \
    FSM_TEST_POS_FSMGETRECOVCNT(); \
    FSM_TEST_POS_FSMSETRESETCNTTHR(); \
    FSM_TEST_POS_FSMGETRESETCNT()

#define FSM_TEST_RUN_NEGATIVE() \
    FSM_TEST_NEG_FSMSETDEVSTATE(); \
    FSM_TEST_NEG_FSMSETRECOVCNTTHR(); \
    FSM_TEST_NEG_FSMGETRECOVCNTTHR(); \
    FSM_TEST_NEG_FSMGETRECOVCNT(); \
    FSM_TEST_NEG_FSMCLRRECOVCNT(); \
    FSM_TEST_NEG_FSMSETRESETCNTTHR(); \
    FSM_TEST_NEG_FSMGETRESETCNTTHR(); \
    FSM_TEST_NEG_FSMGETRESETCNT(); \
    FSM_TEST_NEG_FSMCLRRESETCNT()

#define FSM_TEST_RUN_ALL() \
    FSM_TEST_RUN_POSITIVE(); \
    FSM_TEST_RUN_NEGATIVE()

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

void fsm_test(void *args);

/* fsmSetDevState API tests */
void test_pos_fsm_fsmSetDevState_safeRecovery(void);
void test_pos_fsm_fsmSetDevState_warmReset(void);
void test_pos_fsm_fsmSetDevState_lowPowerEntryExit(void);
void test_pos_fsm_fsmSetDevState_coldBootRequest(void);
void test_pos_fsm_fsmSetDevState_offRequest(void);
void test_neg_fsm_fsmSetDevState_nullParam_pmicHandle(void);
void test_neg_fsm_fsmSetDevState_invalid_fsmCmd(void);

/* fsmSetRecovCntThr API tests */
void test_neg_fsm_fsmSetRecovCntThr_nullParam_pmicHandle(void);
void test_neg_fsm_fsmSetRecovCntThr_outOfBounds_threshold(void);
void test_pos_fsm_setGetRecovCntThr(void);

/* fsmGetRecovCntThr API tests */
void test_neg_fsm_fsmGetRecovCntThr_nullParam_pmicHandle(void);
void test_neg_fsm_fsmGetRecovCntThr_nullParam_threshold(void);

/* fsmGetRecovCnt API tests */
void test_neg_fsm_fsmGetRecovCnt_nullParam_pmicHandle(void);
void test_neg_fsm_fsmGetRecovCnt_nullParam_recovCnt(void);
void test_pos_fsm_getClrRecovCnt(void);

/* fsmClrRecovCnt API tests */
void test_neg_fsm_fsmClrRecovCnt_nullParam_pmicHandle(void);

/* fsmSetResetCntThr API tests */
void test_neg_fsm_fsmSetResetCntThr_nullParam_pmicHandle(void);
void test_neg_fsm_fsmSetResetCntThr_outOfBounds_threshold(void);
void test_pos_fsm_setGetResetCntThr(void);

/* fsmGetResetCntThr API tests */
void test_neg_fsm_fsmGetResetCntThr_nullParam_pmicHandle(void);
void test_neg_fsm_fsmGetResetCntThr_nullParam_threshold(void);

/* fsmGetResetCnt API tests */
void test_neg_fsm_fsmGetResetCnt_nullParam_pmicHandle(void);
void test_neg_fsm_fsmGetResetCnt_nullParam_resetCnt(void);
void test_pos_fsm_getClrResetCnt(void);

/* fsmClrResetCnt API tests */
void test_neg_fsm_fsmClrResetCnt_nullParam_pmicHandle(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* PMIC_TEST_FSM_H */
