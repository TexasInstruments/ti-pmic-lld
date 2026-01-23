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
#ifndef ESM_TEST_H
#define ESM_TEST_H

/**
 * @file esm_test.h
 * @brief Contains macros/defines and test declarations specific to testing the
 * ESM module.
 */

/* ========================================================================== */
/*                              Include Files                                 */
/* ========================================================================== */

#include "test_utils.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/* ========================================================================== */
/*            API-Specific Test Macros - esmSetCfg, esmGetCfg                 */
/* ========================================================================== */
#define ESM_TEST_POS_ESMSETGETCFG() \
    PLATFORM_RUN_TEST(test_pos_esm_esmSetGetCfg_enable); \
    PLATFORM_RUN_TEST(test_pos_esm_esmSetGetCfg_mode_level); \
    PLATFORM_RUN_TEST(test_pos_esm_esmSetGetCfg_mode_pwm); \
    PLATFORM_RUN_TEST(test_pos_esm_esmSetGetCfg_errCntThr); \
    PLATFORM_RUN_TEST(test_pos_esm_esmSetGetCfg_delay1); \
    PLATFORM_RUN_TEST(test_pos_esm_esmSetGetCfg_delay2); \
    PLATFORM_RUN_TEST(test_pos_esm_esmSetGetCfg_hmax); \
    PLATFORM_RUN_TEST(test_pos_esm_esmSetGetCfg_hmin); \
    PLATFORM_RUN_TEST(test_pos_esm_esmSetGetCfg_lmax); \
    PLATFORM_RUN_TEST(test_pos_esm_esmSetGetCfg_lmin)

#define ESM_TEST_NEG_ESMSETGETCFG() \
    PLATFORM_RUN_TEST(test_neg_esm_esmSetCfg_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_esm_esmSetCfg_nullEsmCfg); \
    PLATFORM_RUN_TEST(test_neg_esm_esmSetCfg_invalidValidParams); \
    PLATFORM_RUN_TEST(test_neg_esm_esmSetCfg_outOfBounds_mode); \
    PLATFORM_RUN_TEST(test_neg_esm_esmSetCfg_outOfBounds_errCntThr); \
    PLATFORM_RUN_TEST(test_neg_esm_esmGetCfg_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_esm_esmGetCfg_nullEsmCfg); \
    PLATFORM_RUN_TEST(test_neg_esm_esmGetCfg_invalidValidParams)

#define ESM_TEST_ESMSETGETCFG() \
    ESM_TEST_NEG_ESMSETGETCFG(); \
    ESM_TEST_POS_ESMSETGETCFG()

/* ========================================================================== */
/*       API-Specific Test Macros - esmStart, esmStop, esmGetStartState       */
/* ========================================================================== */
#define ESM_TEST_POS_ESMSTARTSTOP() \
    PLATFORM_RUN_TEST(test_pos_esm_esmStartStop); \
    PLATFORM_RUN_TEST(test_pos_esm_esmStart); \
    PLATFORM_RUN_TEST(test_pos_esm_esmStop); \
    PLATFORM_RUN_TEST(test_pos_esm_esmGetStartState)

#define ESM_TEST_NEG_ESMSTARTSTOP() \
    PLATFORM_RUN_TEST(test_neg_esm_esmSetStartState_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_esm_esmStart_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_esm_esmStop_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_esm_esmGetStartState_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_esm_esmGetStartState_nullStarted)

#define ESM_TEST_ESMSTARTSTOP() \
    ESM_TEST_NEG_ESMSTARTSTOP(); \
    ESM_TEST_POS_ESMSTARTSTOP()

/* ========================================================================== */
/*            API-Specific Test Macros - esmGetStatus, esmClrStatus           */
/* ========================================================================== */
#define ESM_TEST_POS_ESMSTATUS() \
    PLATFORM_RUN_TEST(test_pos_esm_esmGetStatus); \
    PLATFORM_RUN_TEST(test_pos_esm_esmClrStatus)

#define ESM_TEST_NEG_ESMSTATUS() \
    PLATFORM_RUN_TEST(test_neg_esm_esmGetStatus_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_esm_esmGetStatus_nullEsmStat); \
    PLATFORM_RUN_TEST(test_neg_esm_esmGetStatus_invalidValidParams_zero); \
    PLATFORM_RUN_TEST(test_neg_esm_esmGetStatus_invalidValidParams_outOfBounds); \
    PLATFORM_RUN_TEST(test_neg_esm_esmClrStatus_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_esm_esmClrStatus_nullEsmStat); \
    PLATFORM_RUN_TEST(test_neg_esm_esmClrStatus_invalidValidParams_zero); \
    PLATFORM_RUN_TEST(test_neg_esm_esmClrStatus_invalidValidParams_outOfBounds)

#define ESM_TEST_ESMSTATUS() \
    ESM_TEST_NEG_ESMSTATUS(); \
    ESM_TEST_POS_ESMSTATUS()

/* ========================================================================== */
/*                 API-Specific Test Macros - esmGetErrCnt                    */
/* ========================================================================== */
#define ESM_TEST_POS_ESMGETERRCNT() \
    PLATFORM_RUN_TEST(test_pos_esm_esmGetErrCnt)

#define ESM_TEST_NEG_ESMGETERRCNT() \
    PLATFORM_RUN_TEST(test_neg_esm_esmGetErrCnt_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_esm_esmGetErrCnt_nullErrCnt)

#define ESM_TEST_ESMGETERRCNT() \
    ESM_TEST_NEG_ESMGETERRCNT(); \
    ESM_TEST_POS_ESMGETERRCNT()

/* ========================================================================== */
/*                     Aggregate Test Macros                                  */
/* ========================================================================== */

/* Run all ESM positive tests */
#define ESM_TEST_RUN_POSITIVE() \
    ESM_TEST_POS_ESMSETGETCFG(); \
    ESM_TEST_POS_ESMSTARTSTOP(); \
    ESM_TEST_POS_ESMSTATUS(); \
    ESM_TEST_POS_ESMGETERRCNT()

/* Run all ESM negative tests */
#define ESM_TEST_RUN_NEGATIVE() \
    ESM_TEST_NEG_ESMSETGETCFG(); \
    ESM_TEST_NEG_ESMSTARTSTOP(); \
    ESM_TEST_NEG_ESMSTATUS(); \
    ESM_TEST_NEG_ESMGETERRCNT()

/* Run all ESM tests */
#define ESM_TEST_RUN_ALL() \
    ESM_TEST_ESMSETGETCFG(); \
    ESM_TEST_ESMSTARTSTOP(); \
    ESM_TEST_ESMSTATUS(); \
    ESM_TEST_ESMGETERRCNT()

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

void esm_test(void *args);

/* Negative Tests - Pmic_esmSetCfg */
void test_neg_esm_esmSetCfg_nullHandle(void);
void test_neg_esm_esmSetCfg_nullEsmCfg(void);
void test_neg_esm_esmSetCfg_invalidValidParams(void);
void test_neg_esm_esmSetCfg_outOfBounds_mode(void);
void test_neg_esm_esmSetCfg_outOfBounds_errCntThr(void);

/* Negative Tests - Pmic_esmGetCfg */
void test_neg_esm_esmGetCfg_nullHandle(void);
void test_neg_esm_esmGetCfg_nullEsmCfg(void);
void test_neg_esm_esmGetCfg_invalidValidParams(void);

/* Negative Tests - Pmic_esmSetStartState */
void test_neg_esm_esmSetStartState_nullHandle(void);

/* Negative Tests - Pmic_esmStart */
void test_neg_esm_esmStart_nullHandle(void);

/* Negative Tests - Pmic_esmStop */
void test_neg_esm_esmStop_nullHandle(void);

/* Negative Tests - Pmic_esmGetStartState */
void test_neg_esm_esmGetStartState_nullHandle(void);
void test_neg_esm_esmGetStartState_nullStarted(void);

/* Negative Tests - Pmic_esmGetStatus */
void test_neg_esm_esmGetStatus_nullHandle(void);
void test_neg_esm_esmGetStatus_nullEsmStat(void);
void test_neg_esm_esmGetStatus_invalidValidParams_zero(void);
void test_neg_esm_esmGetStatus_invalidValidParams_outOfBounds(void);

/* Negative Tests - Pmic_esmClrStatus */
void test_neg_esm_esmClrStatus_nullHandle(void);
void test_neg_esm_esmClrStatus_nullEsmStat(void);
void test_neg_esm_esmClrStatus_invalidValidParams_zero(void);
void test_neg_esm_esmClrStatus_invalidValidParams_outOfBounds(void);

/* Negative Tests - Pmic_esmGetErrCnt */
void test_neg_esm_esmGetErrCnt_nullHandle(void);
void test_neg_esm_esmGetErrCnt_nullErrCnt(void);

/* Positive Tests - Set/Get ESM Configuration */
void test_pos_esm_esmSetGetCfg_enable(void);
void test_pos_esm_esmSetGetCfg_mode_level(void);
void test_pos_esm_esmSetGetCfg_mode_pwm(void);
void test_pos_esm_esmSetGetCfg_errCntThr(void);
void test_pos_esm_esmSetGetCfg_delay1(void);
void test_pos_esm_esmSetGetCfg_delay2(void);
void test_pos_esm_esmSetGetCfg_hmax(void);
void test_pos_esm_esmSetGetCfg_hmin(void);
void test_pos_esm_esmSetGetCfg_lmax(void);
void test_pos_esm_esmSetGetCfg_lmin(void);

/* Positive Tests - Start/Stop ESM */
void test_pos_esm_esmStartStop(void);
void test_pos_esm_esmStart(void);
void test_pos_esm_esmStop(void);
void test_pos_esm_esmGetStartState(void);

/* Positive Tests - ESM Status */
void test_pos_esm_esmGetStatus(void);
void test_pos_esm_esmClrStatus(void);

/* Positive Tests - ESM Error Count */
void test_pos_esm_esmGetErrCnt(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /*__ESM_TEST_H__*/
