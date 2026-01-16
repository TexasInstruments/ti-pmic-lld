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

#include "test_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

void esm_test(void *args);

/* Negative Tests - Pmic_esmSetCfg */
void test_negative_Pmic_esmSetCfg_nullParam_pmicHandle(void);
void test_negative_Pmic_esmSetCfg_nullParam_esmCfg(void);
void test_negative_Pmic_esmSetCfg_invalid_validParams(void);
void test_negative_Pmic_esmSetCfg_outOfBounds_mode(void);
void test_negative_Pmic_esmSetCfg_outOfBounds_errCntThr(void);

/* Negative Tests - Pmic_esmGetCfg */
void test_negative_Pmic_esmGetCfg_nullParam_pmicHandle(void);
void test_negative_Pmic_esmGetCfg_nullParam_esmCfg(void);
void test_negative_Pmic_esmGetCfg_invalid_validParams(void);

/* Negative Tests - Pmic_esmSetStartState */
void test_negative_Pmic_esmSetStartState_nullParam_pmicHandle(void);

/* Negative Tests - Pmic_esmStart */
void test_negative_Pmic_esmStart_nullParam_pmicHandle(void);

/* Negative Tests - Pmic_esmStop */
void test_negative_Pmic_esmStop_nullParam_pmicHandle(void);

/* Negative Tests - Pmic_esmGetStartState */
void test_negative_Pmic_esmGetStartState_nullParam_pmicHandle(void);
void test_negative_Pmic_esmGetStartState_nullParam_start(void);

/* Negative Tests - Pmic_esmGetStatus */
void test_negative_Pmic_esmGetStatus_nullParam_pmicHandle(void);
void test_negative_Pmic_esmGetStatus_nullParam_esmStat(void);
void test_negative_Pmic_esmGetStatus_invalid_validParams_zero(void);
void test_negative_Pmic_esmGetStatus_invalid_validParams_outOfBounds(void);

/* Negative Tests - Pmic_esmClrStatus */
void test_negative_Pmic_esmClrStatus_nullParam_pmicHandle(void);
void test_negative_Pmic_esmClrStatus_nullParam_esmStat(void);
void test_negative_Pmic_esmClrStatus_invalid_validParams_zero(void);
void test_negative_Pmic_esmClrStatus_invalid_validParams_outOfBounds(void);

/* Negative Tests - Pmic_esmGetErrCnt */
void test_negative_Pmic_esmGetErrCnt_nullParam_pmicHandle(void);
void test_negative_Pmic_esmGetErrCnt_nullParam_errCnt(void);

/* Positive Tests - Set/Get ESM Configuration */
void test_positive_esmSetGetCfg_enable(void);
void test_positive_esmSetGetCfg_mode_level(void);
void test_positive_esmSetGetCfg_mode_pwm(void);
void test_positive_esmSetGetCfg_errCntThr(void);
void test_positive_esmSetGetCfg_delay1(void);
void test_positive_esmSetGetCfg_delay2(void);
void test_positive_esmSetGetCfg_hmax(void);
void test_positive_esmSetGetCfg_hmin(void);
void test_positive_esmSetGetCfg_lmax(void);
void test_positive_esmSetGetCfg_lmin(void);

/* Positive Tests - Start/Stop ESM */
void test_positive_esmStartStop(void);
void test_positive_esmStart(void);
void test_positive_esmStop(void);
void test_positive_esmGetStartState(void);

/* Positive Tests - ESM Status */
void test_positive_esmGetStatus(void);
void test_positive_esmClrStatus(void);

/* Positive Tests - ESM Error Count */
void test_positive_esmGetErrCnt(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /*__ESM_TEST_H__*/
