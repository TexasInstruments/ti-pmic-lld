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

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 * @brief Entry point for ESM module tests.
 *
 * @param args [IN] Test arguments (unused).
 */
void esm_test(void *args);

/* ========================================================================== */
/*                         Test Execution Macros                              */
/* ========================================================================== */

/* Pmic_esmSetEnableState / Pmic_esmGetEnableState Tests */
#define ESM_TEST_POS_SETENABLESTATE() \
    PLATFORM_RUN_TEST(test_pos_esm_setGetEnableState)

#define ESM_TEST_NEG_SETENABLESTATE() \
    PLATFORM_RUN_TEST(test_neg_esm_setEnableState_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_esm_getEnableState_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_esm_getEnableState_nullIsEnabled)

#define ESM_TEST_SETENABLESTATE() \
    ESM_TEST_POS_SETENABLESTATE(); \
    ESM_TEST_NEG_SETENABLESTATE()

/* Pmic_esmSetStartState / Pmic_esmGetStartState Tests */
#define ESM_TEST_POS_SETSTARTSTATE() \
    PLATFORM_RUN_TEST(test_pos_esm_setGetStartState)

#define ESM_TEST_NEG_SETSTARTSTATE() \
    PLATFORM_RUN_TEST(test_neg_esm_setStartState_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_esm_getStartState_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_esm_getStartState_nullStarted)

#define ESM_TEST_SETSTARTSTATE() \
    ESM_TEST_POS_SETSTARTSTATE(); \
    ESM_TEST_NEG_SETSTARTSTATE()

/* Pmic_esmSetCfg / Pmic_esmGetCfg Tests */
#define ESM_TEST_POS_SETCFG() \
    PLATFORM_RUN_TEST(test_pos_esm_setCfg_mode); \
    PLATFORM_RUN_TEST(test_pos_esm_setCfg_errCntThr); \
    PLATFORM_RUN_TEST(test_pos_esm_setCfg_delay1); \
    PLATFORM_RUN_TEST(test_pos_esm_setCfg_delay2); \
    PLATFORM_RUN_TEST(test_pos_esm_setCfg_hmax); \
    PLATFORM_RUN_TEST(test_pos_esm_setCfg_hmin); \
    PLATFORM_RUN_TEST(test_pos_esm_setCfg_lmax); \
    PLATFORM_RUN_TEST(test_pos_esm_setCfg_lmin); \
    PLATFORM_RUN_TEST(test_pos_esm_setCfg_clrEnDrvOnFailInt); \
    PLATFORM_RUN_TEST(test_pos_esm_setCfg_combined); \
    PLATFORM_RUN_TEST(test_pos_esm_setCfg_pwmMode); \
    PLATFORM_RUN_TEST(test_pos_esm_getCfg_readback)

#define ESM_TEST_NEG_SETCFG() \
    PLATFORM_RUN_TEST(test_neg_esm_setCfg_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_esm_setCfg_nullEsmCfg); \
    PLATFORM_RUN_TEST(test_neg_esm_setCfg_invalidValidParams); \
    PLATFORM_RUN_TEST(test_neg_esm_setCfg_invalidMode); \
    PLATFORM_RUN_TEST(test_neg_esm_setCfg_invalidErrCntThr); \
    PLATFORM_RUN_TEST(test_neg_esm_getCfg_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_esm_getCfg_nullEsmCfg); \
    PLATFORM_RUN_TEST(test_neg_esm_getCfg_invalidValidParams)

#define ESM_TEST_SETCFG() \
    ESM_TEST_POS_SETCFG(); \
    ESM_TEST_NEG_SETCFG()

/* Pmic_esmGetErrCnt Tests */
#define ESM_TEST_POS_GETERRCNT() \
    PLATFORM_RUN_TEST(test_pos_esm_getErrCnt)

#define ESM_TEST_NEG_GETERRCNT() \
    PLATFORM_RUN_TEST(test_neg_esm_getErrCnt_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_esm_getErrCnt_nullEsmErrCnt)

#define ESM_TEST_GETERRCNT() \
    ESM_TEST_POS_GETERRCNT(); \
    ESM_TEST_NEG_GETERRCNT()

/* Integration Tests */
#define ESM_TEST_INTEGRATION() \
    PLATFORM_RUN_TEST(test_pos_esm_completeSequence); \
    PLATFORM_RUN_TEST(test_pos_esm_enableCfgStartSequence)

/* Aggregate Test Macros */
#define ESM_TEST_RUN_POSITIVE() \
    ESM_TEST_POS_SETENABLESTATE(); \
    ESM_TEST_POS_SETSTARTSTATE(); \
    ESM_TEST_POS_SETCFG(); \
    ESM_TEST_POS_GETERRCNT(); \
    ESM_TEST_INTEGRATION()

#define ESM_TEST_RUN_NEGATIVE() \
    ESM_TEST_NEG_SETENABLESTATE(); \
    ESM_TEST_NEG_SETSTARTSTATE(); \
    ESM_TEST_NEG_SETCFG(); \
    ESM_TEST_NEG_GETERRCNT()

#define ESM_TEST_RUN_ALL() \
    ESM_TEST_SETENABLESTATE(); \
    ESM_TEST_SETSTARTSTATE(); \
    ESM_TEST_SETCFG(); \
    ESM_TEST_GETERRCNT(); \
    ESM_TEST_INTEGRATION()

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* Negative test functions */
void test_neg_esm_setEnableState_nullHandle(void);
void test_neg_esm_getEnableState_nullHandle(void);
void test_neg_esm_getEnableState_nullIsEnabled(void);
void test_neg_esm_setStartState_nullHandle(void);
void test_neg_esm_getStartState_nullHandle(void);
void test_neg_esm_getStartState_nullStarted(void);
void test_neg_esm_setCfg_nullHandle(void);
void test_neg_esm_setCfg_nullEsmCfg(void);
void test_neg_esm_setCfg_invalidValidParams(void);
void test_neg_esm_setCfg_invalidMode(void);
void test_neg_esm_setCfg_invalidErrCntThr(void);
void test_neg_esm_getCfg_nullHandle(void);
void test_neg_esm_getCfg_nullEsmCfg(void);
void test_neg_esm_getCfg_invalidValidParams(void);
void test_neg_esm_getErrCnt_nullHandle(void);
void test_neg_esm_getErrCnt_nullEsmErrCnt(void);

/* Positive test functions */
void test_pos_esm_setGetEnableState(void);
void test_pos_esm_setGetStartState(void);
void test_pos_esm_setCfg_mode(void);
void test_pos_esm_setCfg_errCntThr(void);
void test_pos_esm_setCfg_delay1(void);
void test_pos_esm_setCfg_delay2(void);
void test_pos_esm_setCfg_hmax(void);
void test_pos_esm_setCfg_hmin(void);
void test_pos_esm_setCfg_lmax(void);
void test_pos_esm_setCfg_lmin(void);
void test_pos_esm_setCfg_clrEnDrvOnFailInt(void);
void test_pos_esm_getErrCnt(void);
void test_pos_esm_setCfg_combined(void);
void test_pos_esm_setCfg_pwmMode(void);
void test_pos_esm_completeSequence(void);
void test_pos_esm_getCfg_readback(void);
void test_pos_esm_enableCfgStartSequence(void);

#endif /* ESM_TEST_H */
