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


#ifndef TIMER_TEST_H
#define TIMER_TEST_H

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "unity.h"
#include "platform.h"
#include "pmic.h"
#include "pmic_timer.h"
#include "test_utils.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/* ========================================================================== */
/*                    API-Specific Test Macros - timerSetCfg                  */
/* ========================================================================== */
#define TIMER_TEST_POS_TIMERSETCFG() \
    PLATFORM_RUN_TEST(test_pos_timer_timerSetCfg_prescale64us); \
    PLATFORM_RUN_TEST(test_pos_timer_timerSetCfg_prescale16ms); \
    PLATFORM_RUN_TEST(test_pos_timer_timerSetCfg_prescale131ms); \
    PLATFORM_RUN_TEST(test_pos_timer_timerSetCfg_prescale1049ms); \
    PLATFORM_RUN_TEST(test_pos_timer_timerSetCfg_modeStopped); \
    PLATFORM_RUN_TEST(test_pos_timer_timerSetCfg_modeOperSeq); \
    PLATFORM_RUN_TEST(test_pos_timer_timerSetCfg_modeStdby); \
    PLATFORM_RUN_TEST(test_pos_timer_timerSetCfg_modeStdbyWu); \
    PLATFORM_RUN_TEST(test_pos_timer_timerSetCfg_modeOperSeqStdby); \
    PLATFORM_RUN_TEST(test_pos_timer_timerSetCfg_modeOperSeqStdbyWu); \
    PLATFORM_RUN_TEST(test_pos_timer_timerSetCfg_prescaleAndModeVerify)

#define TIMER_TEST_NEG_TIMERSETCFG() \
    PLATFORM_RUN_TEST(test_neg_timer_timerSetCfg_invalidPrescale); \
    PLATFORM_RUN_TEST(test_neg_timer_timerSetCfg_nullCfg); \
    PLATFORM_RUN_TEST(test_neg_timer_timerSetCfg_invalidMode); \
    PLATFORM_RUN_TEST(test_neg_timer_timerSetCfg_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_timer_timerSetCfg_validParamsZero)

#define TIMER_TEST_TIMERSETCFG() \
    TIMER_TEST_POS_TIMERSETCFG(); \
    TIMER_TEST_NEG_TIMERSETCFG()

/* ========================================================================== */
/*                    API-Specific Test Macros - timerGetCfg                  */
/* ========================================================================== */
#define TIMER_TEST_POS_TIMERGETCFG() \
    /* Positive tests for timerGetCfg are combined with timerSetCfg tests */

#define TIMER_TEST_NEG_TIMERGETCFG() \
    PLATFORM_RUN_TEST(test_neg_timer_timerGetCfg_nullCfg); \
    PLATFORM_RUN_TEST(test_neg_timer_timerGetCfg_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_timer_timerGetCfg_validParamsZero)

#define TIMER_TEST_TIMERGETCFG() \
    TIMER_TEST_POS_TIMERGETCFG(); \
    TIMER_TEST_NEG_TIMERGETCFG()

/* ========================================================================== */
/*                   API-Specific Test Macros - timerSetCnt                   */
/* ========================================================================== */
#define TIMER_TEST_POS_TIMERSETCNT() \
    PLATFORM_RUN_TEST(test_pos_timer_timerSetCnt_minValueVerify); \
    PLATFORM_RUN_TEST(test_pos_timer_timerSetCnt_maxValueVerify); \
    PLATFORM_RUN_TEST(test_pos_timer_timerSetCnt_midValueVerify); \
    PLATFORM_RUN_TEST(test_pos_timer_timerSetCnt_boundary1Verify); \
    PLATFORM_RUN_TEST(test_pos_timer_timerSetCnt_boundary2Verify)

#define TIMER_TEST_NEG_TIMERSETCNT() \
    PLATFORM_RUN_TEST(test_neg_timer_timerSetCnt_outOfBounds); \
    PLATFORM_RUN_TEST(test_neg_timer_timerSetCnt_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_timer_timerSetCnt_overflowValue)

#define TIMER_TEST_TIMERSETCNT() \
    TIMER_TEST_POS_TIMERSETCNT(); \
    TIMER_TEST_NEG_TIMERSETCNT()

/* ========================================================================== */
/*                   API-Specific Test Macros - timerGetCnt                   */
/* ========================================================================== */
#define TIMER_TEST_POS_TIMERGETCNT() \
    /* Positive tests for timerGetCnt are combined with timerSetCnt tests */

#define TIMER_TEST_NEG_TIMERGETCNT() \
    PLATFORM_RUN_TEST(test_neg_timer_timerGetCnt_nullCnt); \
    PLATFORM_RUN_TEST(test_neg_timer_timerGetCnt_nullHandle)

#define TIMER_TEST_TIMERGETCNT() \
    TIMER_TEST_POS_TIMERGETCNT(); \
    TIMER_TEST_NEG_TIMERGETCNT()

/* ========================================================================== */
/*                   API-Specific Test Macros - timerClr                      */
/* ========================================================================== */
#define TIMER_TEST_POS_TIMERCLR() \
    PLATFORM_RUN_TEST(test_pos_timer_timerClr_resetCounter); \
    PLATFORM_RUN_TEST(test_pos_timer_timerClr_verifyZero); \
    PLATFORM_RUN_TEST(test_pos_timer_timerClr_sequenceVerify)

#define TIMER_TEST_NEG_TIMERCLR() \
    PLATFORM_RUN_TEST(test_neg_timer_timerClr_nullHandle)

#define TIMER_TEST_TIMERCLR() \
    TIMER_TEST_POS_TIMERCLR(); \
    TIMER_TEST_NEG_TIMERCLR()

/* ========================================================================== */
/*                   API-Specific Test Macros - timerStop                     */
/* ========================================================================== */
#define TIMER_TEST_POS_TIMERSTOP() \
    PLATFORM_RUN_TEST(test_pos_timer_timerStop_fromMode1); \
    PLATFORM_RUN_TEST(test_pos_timer_timerStop_fromMode2); \
    PLATFORM_RUN_TEST(test_pos_timer_timerStop_fromMode3); \
    PLATFORM_RUN_TEST(test_pos_timer_timerStop_fromMode4); \
    PLATFORM_RUN_TEST(test_pos_timer_timerStop_fromMode5); \
    PLATFORM_RUN_TEST(test_pos_timer_timerStop_verifyStopped)

#define TIMER_TEST_NEG_TIMERSTOP() \
    PLATFORM_RUN_TEST(test_neg_timer_timerStop_nullHandle)

#define TIMER_TEST_TIMERSTOP() \
    TIMER_TEST_POS_TIMERSTOP(); \
    TIMER_TEST_NEG_TIMERSTOP()

/* ========================================================================== */
/*                API-Specific Test Macros - timerSetWakeupValue              */
/* ========================================================================== */
#define TIMER_TEST_POS_TIMERSETWAKEUPVALUE() \
    PLATFORM_RUN_TEST(test_pos_timer_timerSetWakeupValue_minValueVerify); \
    PLATFORM_RUN_TEST(test_pos_timer_timerSetWakeupValue_maxValueVerify); \
    PLATFORM_RUN_TEST(test_pos_timer_timerSetWakeupValue_midValueVerify); \
    PLATFORM_RUN_TEST(test_pos_timer_timerSetWakeupValue_boundary1Verify); \
    PLATFORM_RUN_TEST(test_pos_timer_timerSetWakeupValue_boundary2Verify); \
    PLATFORM_RUN_TEST(test_pos_timer_timerSetWakeupValue_persistenceVerify)

#define TIMER_TEST_NEG_TIMERSETWAKEUPVALUE() \
    PLATFORM_RUN_TEST(test_neg_timer_timerSetWakeupValue_outOfBounds); \
    PLATFORM_RUN_TEST(test_neg_timer_timerSetWakeupValue_nullHandle)

#define TIMER_TEST_TIMERSETWAKEUPVALUE() \
    TIMER_TEST_POS_TIMERSETWAKEUPVALUE(); \
    TIMER_TEST_NEG_TIMERSETWAKEUPVALUE()

/* ========================================================================== */
/*                API-Specific Test Macros - timerGetWakeupValue              */
/* ========================================================================== */
#define TIMER_TEST_POS_TIMERGETWAKEUPVALUE() \
    /* Positive tests for timerGetWakeupValue are combined with timerSetWakeupValue tests */

#define TIMER_TEST_NEG_TIMERGETWAKEUPVALUE() \
    PLATFORM_RUN_TEST(test_neg_timer_timerGetWakeupValue_nullWakeup); \
    PLATFORM_RUN_TEST(test_neg_timer_timerGetWakeupValue_nullHandle)

#define TIMER_TEST_TIMERGETWAKEUPVALUE() \
    TIMER_TEST_POS_TIMERGETWAKEUPVALUE(); \
    TIMER_TEST_NEG_TIMERGETWAKEUPVALUE()

/* ========================================================================== */
/*                          Aggregate Test Runners                            */
/* ========================================================================== */
#define TIMER_TEST_RUN_POSITIVE() \
    TIMER_TEST_POS_TIMERSETCFG(); \
    TIMER_TEST_POS_TIMERGETCFG(); \
    TIMER_TEST_POS_TIMERSETCNT(); \
    TIMER_TEST_POS_TIMERGETCNT(); \
    TIMER_TEST_POS_TIMERCLR(); \
    TIMER_TEST_POS_TIMERSTOP(); \
    TIMER_TEST_POS_TIMERSETWAKEUPVALUE(); \
    TIMER_TEST_POS_TIMERGETWAKEUPVALUE()

#define TIMER_TEST_RUN_NEGATIVE() \
    TIMER_TEST_NEG_TIMERSETCFG(); \
    TIMER_TEST_NEG_TIMERGETCFG(); \
    TIMER_TEST_NEG_TIMERSETCNT(); \
    TIMER_TEST_NEG_TIMERGETCNT(); \
    TIMER_TEST_NEG_TIMERCLR(); \
    TIMER_TEST_NEG_TIMERSTOP(); \
    TIMER_TEST_NEG_TIMERSETWAKEUPVALUE(); \
    TIMER_TEST_NEG_TIMERGETWAKEUPVALUE()

#define TIMER_TEST_RUN_ALL() \
    TIMER_TEST_RUN_POSITIVE(); \
    TIMER_TEST_RUN_NEGATIVE()

/* ========================================================================== */
/*                         Test Function Declarations                         */
/* ========================================================================== */

/* Positive Tests - timerSetCfg */
void test_pos_timer_timerSetCfg_prescale64us(void);
void test_pos_timer_timerSetCfg_prescale16ms(void);
void test_pos_timer_timerSetCfg_prescale131ms(void);
void test_pos_timer_timerSetCfg_prescale1049ms(void);
void test_pos_timer_timerSetCfg_modeStopped(void);
void test_pos_timer_timerSetCfg_modeOperSeq(void);
void test_pos_timer_timerSetCfg_modeStdby(void);
void test_pos_timer_timerSetCfg_modeStdbyWu(void);
void test_pos_timer_timerSetCfg_modeOperSeqStdby(void);
void test_pos_timer_timerSetCfg_modeOperSeqStdbyWu(void);
void test_pos_timer_timerSetCfg_prescaleAndModeVerify(void);

/* Positive Tests - timerGetCfg */
/* (Verified through timerSetCfg tests) */

/* Positive Tests - timerSetCnt */
void test_pos_timer_timerSetCnt_minValueVerify(void);
void test_pos_timer_timerSetCnt_maxValueVerify(void);
void test_pos_timer_timerSetCnt_midValueVerify(void);
void test_pos_timer_timerSetCnt_boundary1Verify(void);
void test_pos_timer_timerSetCnt_boundary2Verify(void);

/* Positive Tests - timerGetCnt */
/* (Verified through timerSetCnt tests) */

/* Positive Tests - timerClr */
void test_pos_timer_timerClr_resetCounter(void);
void test_pos_timer_timerClr_verifyZero(void);
void test_pos_timer_timerClr_sequenceVerify(void);

/* Positive Tests - timerStop */
void test_pos_timer_timerStop_fromMode1(void);
void test_pos_timer_timerStop_fromMode2(void);
void test_pos_timer_timerStop_fromMode3(void);
void test_pos_timer_timerStop_fromMode4(void);
void test_pos_timer_timerStop_fromMode5(void);
void test_pos_timer_timerStop_verifyStopped(void);

/* Positive Tests - timerSetWakeupValue */
void test_pos_timer_timerSetWakeupValue_minValueVerify(void);
void test_pos_timer_timerSetWakeupValue_maxValueVerify(void);
void test_pos_timer_timerSetWakeupValue_midValueVerify(void);
void test_pos_timer_timerSetWakeupValue_boundary1Verify(void);
void test_pos_timer_timerSetWakeupValue_boundary2Verify(void);
void test_pos_timer_timerSetWakeupValue_persistenceVerify(void);

/* Positive Tests - timerGetWakeupValue */
/* (Verified through timerSetWakeupValue tests) */

/* Negative Tests - timerSetCfg */
void test_neg_timer_timerSetCfg_invalidPrescale(void);
void test_neg_timer_timerSetCfg_nullCfg(void);
void test_neg_timer_timerSetCfg_invalidMode(void);
void test_neg_timer_timerSetCfg_nullHandle(void);
void test_neg_timer_timerSetCfg_validParamsZero(void);

/* Negative Tests - timerGetCfg */
void test_neg_timer_timerGetCfg_nullCfg(void);
void test_neg_timer_timerGetCfg_nullHandle(void);
void test_neg_timer_timerGetCfg_validParamsZero(void);

/* Negative Tests - timerSetCnt */
void test_neg_timer_timerSetCnt_outOfBounds(void);
void test_neg_timer_timerSetCnt_nullHandle(void);
void test_neg_timer_timerSetCnt_overflowValue(void);

/* Negative Tests - timerGetCnt */
void test_neg_timer_timerGetCnt_nullCnt(void);
void test_neg_timer_timerGetCnt_nullHandle(void);

/* Negative Tests - timerClr */
void test_neg_timer_timerClr_nullHandle(void);

/* Negative Tests - timerStop */
void test_neg_timer_timerStop_nullHandle(void);

/* Negative Tests - timerSetWakeupValue */
void test_neg_timer_timerSetWakeupValue_outOfBounds(void);
void test_neg_timer_timerSetWakeupValue_nullHandle(void);

/* Negative Tests - timerGetWakeupValue */
void test_neg_timer_timerGetWakeupValue_nullWakeup(void);
void test_neg_timer_timerGetWakeupValue_nullHandle(void);

/**
 * @brief Run all Timer tests
 * @param args Test arguments (unused)
 */
void timer_test(void *args);

#ifdef __cplusplus
}
#endif

#endif /* TIMER_TEST_H */
