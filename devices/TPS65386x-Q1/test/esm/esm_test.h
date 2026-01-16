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



/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include "../platform.h"
#include "pmic_esm.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/**
 * @brief Convenience macro for common ESM configuration parameters
 */
#define PMIC_ESM_CFG_VALID \
    (PMIC_CFG_ESM_ENABLE_VALID | \
     PMIC_CFG_ESM_MODE_VALID | \
     PMIC_CFG_ESM_ERR_THR_VALID | \
     PMIC_CFG_ESM_POLARITY_VALID | \
     PMIC_CFG_ESM_DEGLITCH_VALID | \
     PMIC_CFG_ESM_TIME_BASE_VALID | \
     PMIC_CFG_ESM_HMAX_VALID | \
     PMIC_CFG_ESM_HMIN_VALID | \
     PMIC_CFG_ESM_LMAX_VALID | \
     PMIC_CFG_ESM_LMIN_VALID)

/**
 * @brief Convenience macro for ESM status valid params - use actual status names
 */
#define PMIC_ESM_ERR_VALID_SHIFT PMIC_ESM_ERR_VALID
#define PMIC_ESM_DELAY1_ERR_VALID_SHIFT PMIC_ESM_DELAY1_ERR_VALID
#define PMIC_ESM_DELAY2_ERR_VALID_SHIFT PMIC_ESM_DELAY2_ERR_VALID
#define PMIC_ESM_ERR_CNT_VALID_SHIFT PMIC_ESM_ERR_CNT_VALID
#define PMIC_CFG_ESM_DELAY1_VALID_SHIFT PMIC_CFG_ESM_DELAY1_VALID
#define PMIC_CFG_ESM_DELAY2_VALID_SHIFT PMIC_CFG_ESM_DELAY2_VALID

/**
 * @brief Run all ESM tests (positive and negative).
 */
#define ESM_TEST_RUN_ALL() \
    ESM_TEST_RUN_POSITIVE(); \
    ESM_TEST_RUN_NEGATIVE()

/**
 * @brief Run all positive ESM tests.
 */
#define ESM_TEST_RUN_POSITIVE() \
    PLATFORM_RUN_TEST(test_positive_esm_setGetStartState); \
    PLATFORM_RUN_TEST(test_positive_esm_start); \
    PLATFORM_RUN_TEST(test_positive_esm_stop); \
    PLATFORM_RUN_TEST(test_positive_esm_setGetCfg_enable); \
    PLATFORM_RUN_TEST(test_positive_esm_setGetCfg_mode); \
    PLATFORM_RUN_TEST(test_positive_esm_setGetCfg_errThr); \
    PLATFORM_RUN_TEST(test_positive_esm_setGetCfg_polarity); \
    PLATFORM_RUN_TEST(test_positive_esm_setGetCfg_deglitch); \
    PLATFORM_RUN_TEST(test_positive_esm_setGetCfg_timeBase); \
    PLATFORM_RUN_TEST(test_positive_esm_setGetCfg_delay1); \
    PLATFORM_RUN_TEST(test_positive_esm_setGetCfg_delay2); \
    PLATFORM_RUN_TEST(test_positive_esm_setGetCfg_hmax); \
    PLATFORM_RUN_TEST(test_positive_esm_setGetCfg_hmin); \
    PLATFORM_RUN_TEST(test_positive_esm_setGetCfg_lmax); \
    PLATFORM_RUN_TEST(test_positive_esm_setGetCfg_lmin); \
    PLATFORM_RUN_TEST(test_positive_esm_setGetCfg_multiple); \
    PLATFORM_RUN_TEST(test_positive_esm_getStatus_esmErr); \
    PLATFORM_RUN_TEST(test_positive_esm_getStatus_delay1Err); \
    PLATFORM_RUN_TEST(test_positive_esm_getStatus_delay2Err); \
    PLATFORM_RUN_TEST(test_positive_esm_getStatus_errCnt); \
    PLATFORM_RUN_TEST(test_positive_esm_clrStatus)

/**
 * @brief Run all negative ESM tests.
 */
#define ESM_TEST_RUN_NEGATIVE() \
    PLATFORM_RUN_TEST(test_negative_esm_setStartState_nullHandle); \
    PLATFORM_RUN_TEST(test_negative_esm_getStartState_nullHandle); \
    PLATFORM_RUN_TEST(test_negative_esm_getStartState_nullPointer); \
    PLATFORM_RUN_TEST(test_negative_esm_start_nullHandle); \
    PLATFORM_RUN_TEST(test_negative_esm_stop_nullHandle); \
    PLATFORM_RUN_TEST(test_negative_esm_setCfg_nullHandle); \
    PLATFORM_RUN_TEST(test_negative_esm_setCfg_nullPointer); \
    PLATFORM_RUN_TEST(test_negative_esm_setCfg_invalidParams); \
    PLATFORM_RUN_TEST(test_negative_esm_setCfg_invalidMode); \
    PLATFORM_RUN_TEST(test_negative_esm_setCfg_invalidErrThr); \
    PLATFORM_RUN_TEST(test_negative_esm_setCfg_invalidPolarity); \
    PLATFORM_RUN_TEST(test_negative_esm_setCfg_invalidDeglitch); \
    PLATFORM_RUN_TEST(test_negative_esm_setCfg_invalidTimeBase); \
    PLATFORM_RUN_TEST(test_negative_esm_getCfg_nullHandle); \
    PLATFORM_RUN_TEST(test_negative_esm_getCfg_nullPointer); \
    PLATFORM_RUN_TEST(test_negative_esm_getCfg_invalidParams); \
    PLATFORM_RUN_TEST(test_negative_esm_getStatus_nullHandle); \
    PLATFORM_RUN_TEST(test_negative_esm_getStatus_nullPointer); \
    PLATFORM_RUN_TEST(test_negative_esm_getStatus_invalidParams); \
    PLATFORM_RUN_TEST(test_negative_esm_clrStatus_nullHandle); \
    PLATFORM_RUN_TEST(test_negative_esm_clrStatus_nullPointer); \
    PLATFORM_RUN_TEST(test_negative_esm_clrStatus_invalidParams); \
    PLATFORM_RUN_TEST(test_negative_esm_clrStatus_unsupportedErrCnt)

/* ========================================================================== */
/*                       Positive Test Declarations                           */
/* ========================================================================== */

/**
 * @brief Test: Set and get ESM start state.
 */
void test_positive_esm_setGetStartState(void);

/**
 * @brief Test: Start ESM using Pmic_esmStart().
 */
void test_positive_esm_start(void);

/**
 * @brief Test: Stop ESM using Pmic_esmStop().
 */
void test_positive_esm_stop(void);

/**
 * @brief Test: Set and get ESM enable configuration.
 */
void test_positive_esm_setGetCfg_enable(void);

/**
 * @brief Test: Set and get ESM mode configuration.
 */
void test_positive_esm_setGetCfg_mode(void);

/**
 * @brief Test: Set and get ESM error threshold configuration.
 */
void test_positive_esm_setGetCfg_errThr(void);

/**
 * @brief Test: Set and get ESM polarity configuration.
 */
void test_positive_esm_setGetCfg_polarity(void);

/**
 * @brief Test: Set and get ESM deglitch configuration.
 */
void test_positive_esm_setGetCfg_deglitch(void);

/**
 * @brief Test: Set and get ESM time base configuration.
 */
void test_positive_esm_setGetCfg_timeBase(void);

/**
 * @brief Test: Set and get ESM delay1 configuration.
 */
void test_positive_esm_setGetCfg_delay1(void);

/**
 * @brief Test: Set and get ESM delay2 configuration.
 */
void test_positive_esm_setGetCfg_delay2(void);

/**
 * @brief Test: Set and get ESM hmax configuration.
 */
void test_positive_esm_setGetCfg_hmax(void);

/**
 * @brief Test: Set and get ESM hmin configuration.
 */
void test_positive_esm_setGetCfg_hmin(void);

/**
 * @brief Test: Set and get ESM lmax configuration.
 */
void test_positive_esm_setGetCfg_lmax(void);

/**
 * @brief Test: Set and get ESM lmin configuration.
 */
void test_positive_esm_setGetCfg_lmin(void);

/**
 * @brief Test: Set and get multiple ESM configuration parameters simultaneously.
 */
void test_positive_esm_setGetCfg_multiple(void);

/**
 * @brief Test: Get ESM error status.
 */
void test_positive_esm_getStatus_esmErr(void);

/**
 * @brief Test: Get ESM delay1 error status.
 */
void test_positive_esm_getStatus_delay1Err(void);

/**
 * @brief Test: Get ESM delay2 error status.
 */
void test_positive_esm_getStatus_delay2Err(void);

/**
 * @brief Test: Get ESM error count status.
 */
void test_positive_esm_getStatus_errCnt(void);

/**
 * @brief Test: Clear ESM status flags.
 */
void test_positive_esm_clrStatus(void);

/* ========================================================================== */
/*                       Negative Test Declarations                           */
/* ========================================================================== */

/**
 * @brief Test: Pmic_esmSetStartState() with NULL handle.
 */
void test_negative_esm_setStartState_nullHandle(void);

/**
 * @brief Test: Pmic_esmGetStartState() with NULL handle.
 */
void test_negative_esm_getStartState_nullHandle(void);

/**
 * @brief Test: Pmic_esmGetStartState() with NULL start pointer.
 */
void test_negative_esm_getStartState_nullPointer(void);

/**
 * @brief Test: Pmic_esmStart() with NULL handle.
 */
void test_negative_esm_start_nullHandle(void);

/**
 * @brief Test: Pmic_esmStop() with NULL handle.
 */
void test_negative_esm_stop_nullHandle(void);

/**
 * @brief Test: Pmic_esmSetCfg() with NULL handle.
 */
void test_negative_esm_setCfg_nullHandle(void);

/**
 * @brief Test: Pmic_esmSetCfg() with NULL configuration pointer.
 */
void test_negative_esm_setCfg_nullPointer(void);

/**
 * @brief Test: Pmic_esmSetCfg() with invalid validParams (0).
 */
void test_negative_esm_setCfg_invalidParams(void);

/**
 * @brief Test: Pmic_esmSetCfg() with invalid mode value.
 */
void test_negative_esm_setCfg_invalidMode(void);

/**
 * @brief Test: Pmic_esmSetCfg() with invalid error threshold value.
 */
void test_negative_esm_setCfg_invalidErrThr(void);

/**
 * @brief Test: Pmic_esmSetCfg() with invalid polarity value.
 */
void test_negative_esm_setCfg_invalidPolarity(void);

/**
 * @brief Test: Pmic_esmSetCfg() with invalid deglitch value.
 */
void test_negative_esm_setCfg_invalidDeglitch(void);

/**
 * @brief Test: Pmic_esmSetCfg() with invalid time base value.
 */
void test_negative_esm_setCfg_invalidTimeBase(void);

/**
 * @brief Test: Pmic_esmGetCfg() with NULL handle.
 */
void test_negative_esm_getCfg_nullHandle(void);

/**
 * @brief Test: Pmic_esmGetCfg() with NULL configuration pointer.
 */
void test_negative_esm_getCfg_nullPointer(void);

/**
 * @brief Test: Pmic_esmGetCfg() with invalid validParams (0).
 */
void test_negative_esm_getCfg_invalidParams(void);

/**
 * @brief Test: Pmic_esmGetStatus() with NULL handle.
 */
void test_negative_esm_getStatus_nullHandle(void);

/**
 * @brief Test: Pmic_esmGetStatus() with NULL status pointer.
 */
void test_negative_esm_getStatus_nullPointer(void);

/**
 * @brief Test: Pmic_esmGetStatus() with invalid validParams (0).
 */
void test_negative_esm_getStatus_invalidParams(void);

/**
 * @brief Test: Pmic_esmClrStatus() with NULL handle.
 */
void test_negative_esm_clrStatus_nullHandle(void);

/**
 * @brief Test: Pmic_esmClrStatus() with NULL status pointer.
 */
void test_negative_esm_clrStatus_nullPointer(void);

/**
 * @brief Test: Pmic_esmClrStatus() with invalid validParams (0).
 */
void test_negative_esm_clrStatus_invalidParams(void);

/**
 * @brief Test: Pmic_esmClrStatus() with unsupported ERR_CNT parameter.
 */
void test_negative_esm_clrStatus_unsupportedErrCnt(void);

/* ========================================================================== */
/*                         Unity Framework Functions                          */
/* ========================================================================== */

/* setUp() and tearDown() are defined in test_runner.c, not here */

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* ESM_TEST_H */
