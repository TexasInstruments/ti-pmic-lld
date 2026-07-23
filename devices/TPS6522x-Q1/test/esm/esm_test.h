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
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/* ======================================================================== */
/*             Test APIs: esmSetEnableState, esmGetEnableState              */
/* ======================================================================== */
#define ESM_TEST_POS_SETENABLESTATE() \
    PLATFORM_RUN_TEST(test_pos_esm_setGetEnableState); \
    PLATFORM_RUN_TEST(test_neg_esm_esmGetEnableState_ioRxByteCSFail); \
    PLATFORM_RUN_TEST(test_neg_esm_esmSetEnableState_ioRxByteFail)

#define ESM_TEST_NEG_SETENABLESTATE() \
    PLATFORM_RUN_TEST(test_neg_esm_setEnableState_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_esm_getEnableState_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_esm_getEnableState_nullIsEnabled)

/* Test: TC-ESM-0010 */
#define ESM_TEST_SETENABLESTATE() \
    ESM_TEST_POS_SETENABLESTATE(); \
    ESM_TEST_NEG_SETENABLESTATE()

/* ======================================================================== */
/*              Test APIs: esmSetStartState, esmGetStartState               */
/* ======================================================================== */
#define ESM_TEST_POS_SETSTARTSTATE() \
    PLATFORM_RUN_TEST(test_pos_esm_setGetStartState); \
    PLATFORM_RUN_TEST(test_neg_esm_esmGetStartState_ioRxByteCSFail); \
    PLATFORM_RUN_TEST(test_neg_esm_esmSetStartState_ioRxByteFail)

#define ESM_TEST_NEG_SETSTARTSTATE() \
    PLATFORM_RUN_TEST(test_neg_esm_setStartState_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_esm_getStartState_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_esm_getStartState_nullStarted)

/* Test: TC-ESM-0011 */
#define ESM_TEST_SETSTARTSTATE() \
    ESM_TEST_POS_SETSTARTSTATE(); \
    ESM_TEST_NEG_SETSTARTSTATE()

/* ======================================================================== */
/*                     Test APIs: esmSetCfg, esmGetCfg                      */
/* ======================================================================== */
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
    PLATFORM_RUN_TEST(test_pos_esm_getCfg_readback); \
    PLATFORM_RUN_TEST(test_neg_esm_esmGetCfg_ioRxByteFail); \
    PLATFORM_RUN_TEST(test_neg_esm_esmSetCfg_ioTxByteFail)

#define ESM_TEST_NEG_SETCFG() \
    PLATFORM_RUN_TEST(test_neg_esm_setCfg_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_esm_setCfg_nullEsmCfg); \
    PLATFORM_RUN_TEST(test_neg_esm_setCfg_invalidValidParams); \
    PLATFORM_RUN_TEST(test_neg_esm_setCfg_invalidMode); \
    PLATFORM_RUN_TEST(test_neg_esm_setCfg_invalidErrCntThr); \
    PLATFORM_RUN_TEST(test_neg_esm_getCfg_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_esm_getCfg_nullEsmCfg); \
    PLATFORM_RUN_TEST(test_neg_esm_getCfg_invalidValidParams)

/* Test: TC-ESM-0012 */
#define ESM_TEST_SETCFG() \
    ESM_TEST_POS_SETCFG(); \
    ESM_TEST_NEG_SETCFG()

/* ======================================================================== */
/*                    Negative Test APIs: esmGetCfg (read failures)         */
/* ======================================================================== */
#define ESM_TEST_NEG_ESMGETCFG() \
    PLATFORM_RUN_TEST(test_neg_esm_esmGetCfg_modeCfgReadFail); \
    PLATFORM_RUN_TEST(test_neg_esm_esmGetCfg_secondReadFail)

/* ========================================================================== */
/*   Static Helper Coverage Tests (BUILD_MOCK) — internal I/O failures        */
/* ========================================================================== */

#ifdef BUILD_MOCK
#define ESM_TEST_STATIC_HELPER_COVERAGE() \
    PLATFORM_RUN_TEST(test_neg_esm_esmSetCfg_delay2ReadFail); \
    PLATFORM_RUN_TEST(test_neg_esm_esmSetCfg_hmaxReadFail); \
    PLATFORM_RUN_TEST(test_neg_esm_esmSetCfg_hminReadFail); \
    PLATFORM_RUN_TEST(test_neg_esm_esmSetCfg_lmaxReadFail); \
    PLATFORM_RUN_TEST(test_neg_esm_esmSetCfg_lminReadFail); \
    PLATFORM_RUN_TEST(test_neg_esm_esmGetCfg_delay2ReadFail); \
    PLATFORM_RUN_TEST(test_neg_esm_esmGetCfg_hmaxReadFail); \
    PLATFORM_RUN_TEST(test_neg_esm_esmGetCfg_hminReadFail); \
    PLATFORM_RUN_TEST(test_neg_esm_esmGetCfg_lmaxReadFail); \
    PLATFORM_RUN_TEST(test_neg_esm_esmGetCfg_lminReadFail)
#else
#define ESM_TEST_STATIC_HELPER_COVERAGE()
#endif

/* ======================================================================== */
/*                         Test APIs: esmGetErrCnt                          */
/* ======================================================================== */
#define ESM_TEST_POS_GETERRCNT() \
    PLATFORM_RUN_TEST(test_pos_esm_getErrCnt); \
    PLATFORM_RUN_TEST(test_neg_esm_esmGetErrCnt_ioRxByteCSFail)

#define ESM_TEST_NEG_GETERRCNT() \
    PLATFORM_RUN_TEST(test_neg_esm_getErrCnt_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_esm_getErrCnt_nullEsmErrCnt)

/* Test: TC-ESM-0013 */
#define ESM_TEST_GETERRCNT() \
    ESM_TEST_POS_GETERRCNT(); \
    ESM_TEST_NEG_GETERRCNT()

/* ========================================================================== */
/*                         Integration Tests                                  */
/* ========================================================================== */
#define ESM_TEST_INTEGRATION() \
    PLATFORM_RUN_TEST(test_pos_esm_completeSequence)

/* ========================================================================== */
/*                        Aggregate Test Macros                               */
/* ========================================================================== */
#define ESM_TEST_RUN_POSITIVE() \
    ESM_TEST_POS_SETENABLESTATE(); \
    ESM_TEST_POS_SETSTARTSTATE(); \
    ESM_TEST_POS_SETCFG(); \
    ESM_TEST_POS_GETERRCNT(); \
    ESM_TEST_INTEGRATION(); \
    ESM_TEST_STATIC_HELPER_COVERAGE()

#define ESM_TEST_RUN_NEGATIVE() \
    ESM_TEST_NEG_SETENABLESTATE(); \
    ESM_TEST_NEG_SETSTARTSTATE(); \
    ESM_TEST_NEG_SETCFG(); \
    ESM_TEST_NEG_ESMGETCFG(); \
    ESM_TEST_NEG_GETERRCNT()

#define ESM_TEST_RUN_ALL() \
    ESM_TEST_SETENABLESTATE(); \
    ESM_TEST_SETSTARTSTATE(); \
    ESM_TEST_SETCFG(); \
    ESM_TEST_NEG_ESMGETCFG(); \
    ESM_TEST_GETERRCNT(); \
    ESM_TEST_INTEGRATION(); \
    ESM_TEST_STATIC_HELPER_COVERAGE()

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 * @brief Entry point for ESM module tests.
 *
 * @param args [IN] Test arguments (unused).
 */
void esm_test(void *args);

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

/* Negative test functions (read fail variants) */
void test_neg_esm_esmGetCfg_modeCfgReadFail(void);
void test_neg_esm_esmGetCfg_secondReadFail(void);

/* Dynamic analysis / error injection test functions */
void test_neg_esm_esmGetEnableState_ioRxByteCSFail(void);
void test_neg_esm_esmGetErrCnt_ioRxByteCSFail(void);
void test_neg_esm_esmGetStartState_ioRxByteCSFail(void);
void test_neg_esm_esmSetEnableState_ioRxByteFail(void);
void test_neg_esm_esmSetStartState_ioRxByteFail(void);
void test_neg_esm_esmGetCfg_ioRxByteFail(void);
void test_neg_esm_esmSetCfg_ioTxByteFail(void);

/* Static helper coverage test functions (BUILD_MOCK) — internal I/O failures */
void test_neg_esm_esmSetCfg_delay2ReadFail(void);
void test_neg_esm_esmSetCfg_hmaxReadFail(void);
void test_neg_esm_esmSetCfg_hminReadFail(void);
void test_neg_esm_esmSetCfg_lmaxReadFail(void);
void test_neg_esm_esmSetCfg_lminReadFail(void);
void test_neg_esm_esmGetCfg_delay2ReadFail(void);
void test_neg_esm_esmGetCfg_hmaxReadFail(void);
void test_neg_esm_esmGetCfg_hminReadFail(void);
void test_neg_esm_esmGetCfg_lmaxReadFail(void);
void test_neg_esm_esmGetCfg_lminReadFail(void);

#endif /* ESM_TEST_H */
