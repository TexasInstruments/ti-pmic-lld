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
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* ========================================================================== */
/*                    API-Specific Test Macros - esmClrStatus                 */
/* ========================================================================== */

#define ESM_TEST_POS_ESMCLRSTATUS() \
    PLATFORM_RUN_TEST(test_pos_esm_esmClrStatus_allFields); \
    PLATFORM_RUN_TEST(test_pos_esm_esmClrStatus_failInt); \
    PLATFORM_RUN_TEST(test_pos_esm_esmClrStatus_pinInt); \
    PLATFORM_RUN_TEST(test_pos_esm_esmClrStatus_rstInt)

#define ESM_TEST_NEG_ESMCLRSTATUS() \
    PLATFORM_RUN_TEST(test_neg_esm_esmClrStatus_invalidValidParams); \
    PLATFORM_RUN_TEST(test_neg_esm_esmClrStatus_ioWriteFailure); \
    PLATFORM_RUN_TEST(test_neg_esm_esmClrStatus_nullEsmStat); \
    PLATFORM_RUN_TEST(test_neg_esm_esmClrStatus_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_esm_esmClrStatus_zeroValidParams)

#define ESM_TEST_ESMCLRSTATUS() \
    ESM_TEST_POS_ESMCLRSTATUS(); \
    ESM_TEST_NEG_ESMCLRSTATUS()

/* ========================================================================== */
/*                    API-Specific Test Macros - esmGetCfg                    */
/* ========================================================================== */

#define ESM_TEST_POS_ESMGETCFG() \
    PLATFORM_RUN_TEST(test_pos_esm_esmGetCfg_delay1); \
    PLATFORM_RUN_TEST(test_pos_esm_esmGetCfg_delay2); \
    PLATFORM_RUN_TEST(test_pos_esm_esmGetCfg_delay2Only); \
    PLATFORM_RUN_TEST(test_pos_esm_esmGetCfg_errCntThr); \
    PLATFORM_RUN_TEST(test_pos_esm_esmGetCfg_hmax); \
    PLATFORM_RUN_TEST(test_pos_esm_esmGetCfg_hmin); \
    PLATFORM_RUN_TEST(test_pos_esm_esmGetCfg_hminOnly); \
    PLATFORM_RUN_TEST(test_pos_esm_esmGetCfg_lmax); \
    PLATFORM_RUN_TEST(test_pos_esm_esmGetCfg_lmin); \
    PLATFORM_RUN_TEST(test_pos_esm_esmGetCfg_lminOnly); \
    PLATFORM_RUN_TEST(test_pos_esm_esmGetCfg_mode)

#define ESM_TEST_NEG_ESMGETCFG() \
    PLATFORM_RUN_TEST(test_neg_esm_esmGetCfg_cascadeFailure_delayToHmax); \
    PLATFORM_RUN_TEST(test_neg_esm_esmGetCfg_cascadeFailure_modeToDelay); \
    PLATFORM_RUN_TEST(test_neg_esm_esmGetCfg_delay1ReadFailure); \
    PLATFORM_RUN_TEST(test_neg_esm_esmGetCfg_hmaxReadFailure); \
    PLATFORM_RUN_TEST(test_neg_esm_esmGetCfg_invalidValidParams); \
    PLATFORM_RUN_TEST(test_neg_esm_esmGetCfg_lmaxReadFailure); \
    PLATFORM_RUN_TEST(test_neg_esm_esmGetCfg_modeCfgReadFailure); \
    PLATFORM_RUN_TEST(test_neg_esm_esmGetCfg_nullEsmCfg); \
    PLATFORM_RUN_TEST(test_neg_esm_esmGetCfg_nullHandle)

#define ESM_TEST_ESMGETCFG() \
    ESM_TEST_POS_ESMGETCFG(); \
    ESM_TEST_NEG_ESMGETCFG()

/* ========================================================================== */
/*                    API-Specific Test Macros - esmGetEnableState            */
/* ========================================================================== */

#define ESM_TEST_POS_ESMGETENABLESTATE() \
    PLATFORM_RUN_TEST(test_pos_esm_esmGetEnableState_enableDisable)

#define ESM_TEST_NEG_ESMGETENABLESTATE() \
    PLATFORM_RUN_TEST(test_neg_esm_esmGetEnableState_ioReadFailure); \
    PLATFORM_RUN_TEST(test_neg_esm_esmGetEnableState_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_esm_esmGetEnableState_nullIsEnabled)

#define ESM_TEST_ESMGETENABLESTATE() \
    ESM_TEST_POS_ESMGETENABLESTATE(); \
    ESM_TEST_NEG_ESMGETENABLESTATE()

/* ========================================================================== */
/*                    API-Specific Test Macros - esmGetErrCnt                 */
/* ========================================================================== */

#define ESM_TEST_POS_ESMGETERRCNT() \
    PLATFORM_RUN_TEST(test_pos_esm_esmGetErrCnt_getCount)

#define ESM_TEST_NEG_ESMGETERRCNT() \
    PLATFORM_RUN_TEST(test_neg_esm_esmGetErrCnt_ioReadFailure); \
    PLATFORM_RUN_TEST(test_neg_esm_esmGetErrCnt_nullEsmErrCnt); \
    PLATFORM_RUN_TEST(test_neg_esm_esmGetErrCnt_nullHandle)

#define ESM_TEST_ESMGETERRCNT() \
    ESM_TEST_POS_ESMGETERRCNT(); \
    ESM_TEST_NEG_ESMGETERRCNT()

/* ========================================================================== */
/*                    API-Specific Test Macros - esmGetStartState             */
/* ========================================================================== */

#define ESM_TEST_POS_ESMGETSTARTSTATE() \
    PLATFORM_RUN_TEST(test_pos_esm_esmGetStartState_startStop)

#define ESM_TEST_NEG_ESMGETSTARTSTATE() \
    PLATFORM_RUN_TEST(test_neg_esm_esmGetStartState_ioReadFailure); \
    PLATFORM_RUN_TEST(test_neg_esm_esmGetStartState_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_esm_esmGetStartState_nullStarted)

#define ESM_TEST_ESMGETSTARTSTATE() \
    ESM_TEST_POS_ESMGETSTARTSTATE(); \
    ESM_TEST_NEG_ESMGETSTARTSTATE()

/* ========================================================================== */
/*                    API-Specific Test Macros - esmGetStatus                 */
/* ========================================================================== */

#define ESM_TEST_POS_ESMGETSTATUS() \
    PLATFORM_RUN_TEST(test_pos_esm_esmGetStatus_allFields); \
    PLATFORM_RUN_TEST(test_pos_esm_esmGetStatus_failInt); \
    PLATFORM_RUN_TEST(test_pos_esm_esmGetStatus_pinInt); \
    PLATFORM_RUN_TEST(test_pos_esm_esmGetStatus_rstInt)

#define ESM_TEST_NEG_ESMGETSTATUS() \
    PLATFORM_RUN_TEST(test_neg_esm_esmGetStatus_invalidValidParams); \
    PLATFORM_RUN_TEST(test_neg_esm_esmGetStatus_ioReadFailure); \
    PLATFORM_RUN_TEST(test_neg_esm_esmGetStatus_nullEsmStat); \
    PLATFORM_RUN_TEST(test_neg_esm_esmGetStatus_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_esm_esmGetStatus_zeroValidParams)

#define ESM_TEST_ESMGETSTATUS() \
    ESM_TEST_POS_ESMGETSTATUS(); \
    ESM_TEST_NEG_ESMGETSTATUS()

/* ========================================================================== */
/*                    API-Specific Test Macros - esmSetCfg                    */
/* ========================================================================== */

#define ESM_TEST_POS_ESMSETCFG() \
    PLATFORM_RUN_TEST(test_pos_esm_esmSetCfg_combinedConfiguration); \
    PLATFORM_RUN_TEST(test_pos_esm_esmSetCfg_configurationReadbackVerification); \
    PLATFORM_RUN_TEST(test_pos_esm_esmSetCfg_delay1); \
    PLATFORM_RUN_TEST(test_pos_esm_esmSetCfg_delay2); \
    PLATFORM_RUN_TEST(test_pos_esm_esmSetCfg_delay2Only); \
    PLATFORM_RUN_TEST(test_pos_esm_esmSetCfg_errCntThr); \
    PLATFORM_RUN_TEST(test_pos_esm_esmSetCfg_hmax); \
    PLATFORM_RUN_TEST(test_pos_esm_esmSetCfg_hmin); \
    PLATFORM_RUN_TEST(test_pos_esm_esmSetCfg_hminOnly); \
    PLATFORM_RUN_TEST(test_pos_esm_esmSetCfg_lmax); \
    PLATFORM_RUN_TEST(test_pos_esm_esmSetCfg_lmin); \
    PLATFORM_RUN_TEST(test_pos_esm_esmSetCfg_lminOnly); \
    PLATFORM_RUN_TEST(test_pos_esm_esmSetCfg_mode); \
    PLATFORM_RUN_TEST(test_pos_esm_esmSetCfg_pwmModeConfiguration)

#define ESM_TEST_NEG_ESMSETCFG() \
    PLATFORM_RUN_TEST(test_neg_esm_esmSetCfg_cascadeFailure_delayToHmax); \
    PLATFORM_RUN_TEST(test_neg_esm_esmSetCfg_cascadeFailure_hmaxToLmax); \
    PLATFORM_RUN_TEST(test_neg_esm_esmSetCfg_delay1ReadFailure); \
    PLATFORM_RUN_TEST(test_neg_esm_esmSetCfg_hmaxReadFailure); \
    PLATFORM_RUN_TEST(test_neg_esm_esmSetCfg_invalidErrCntThr); \
    PLATFORM_RUN_TEST(test_neg_esm_esmSetCfg_invalidMode); \
    PLATFORM_RUN_TEST(test_neg_esm_esmSetCfg_invalidValidParams); \
    PLATFORM_RUN_TEST(test_neg_esm_esmSetCfg_lmaxReadFailure); \
    PLATFORM_RUN_TEST(test_neg_esm_esmSetCfg_modeCfgReadFailure); \
    PLATFORM_RUN_TEST(test_neg_esm_esmSetCfg_nullEsmCfg); \
    PLATFORM_RUN_TEST(test_neg_esm_esmSetCfg_nullHandle)

#define ESM_TEST_ESMSETCFG() \
    ESM_TEST_POS_ESMSETCFG(); \
    ESM_TEST_NEG_ESMSETCFG()

/* ========================================================================== */
/*                    API-Specific Test Macros - esmSetEnableState            */
/* ========================================================================== */

#define ESM_TEST_POS_ESMSETENABLESTATE() \
    /* Positive tests combined with esmGetEnableState */

#define ESM_TEST_NEG_ESMSETENABLESTATE() \
    PLATFORM_RUN_TEST(test_neg_esm_esmSetEnableState_ioReadFailure); \
    PLATFORM_RUN_TEST(test_neg_esm_esmSetEnableState_nullHandle)

#define ESM_TEST_ESMSETENABLESTATE() \
    ESM_TEST_POS_ESMSETENABLESTATE(); \
    ESM_TEST_NEG_ESMSETENABLESTATE()

/* ========================================================================== */
/*                    API-Specific Test Macros - esmSetStartState             */
/* ========================================================================== */

#define ESM_TEST_POS_ESMSETSTARTSTATE() \
    /* None */

#define ESM_TEST_NEG_ESMSETSTARTSTATE() \
    PLATFORM_RUN_TEST(test_neg_esm_esmSetStartState_ioReadFailure); \
    PLATFORM_RUN_TEST(test_neg_esm_esmSetStartState_nullHandle)

#define ESM_TEST_ESMSETSTARTSTATE() \
    ESM_TEST_POS_ESMSETSTARTSTATE(); \
    ESM_TEST_NEG_ESMSETSTARTSTATE()

/* ========================================================================== */
/*                    API-Specific Test Macros - esmStart                     */
/* ========================================================================== */

#define ESM_TEST_POS_ESMSTART() \
    PLATFORM_RUN_TEST(test_pos_esm_esmStart_start)

#define ESM_TEST_NEG_ESMSTART() \
    PLATFORM_RUN_TEST(test_neg_esm_esmStart_nullHandle)

#define ESM_TEST_ESMSTART() \
    ESM_TEST_POS_ESMSTART(); \
    ESM_TEST_NEG_ESMSTART()

/* ========================================================================== */
/*                    API-Specific Test Macros - esmStop                      */
/* ========================================================================== */

#define ESM_TEST_POS_ESMSTOP() \
    PLATFORM_RUN_TEST(test_pos_esm_esmStop_stop)

#define ESM_TEST_NEG_ESMSTOP() \
    PLATFORM_RUN_TEST(test_neg_esm_esmStop_nullHandle)

#define ESM_TEST_ESMSTOP() \
    ESM_TEST_POS_ESMSTOP(); \
    ESM_TEST_NEG_ESMSTOP()

/* ========================================================================== */
/*                         Aggregate Test Runners                             */
/* ========================================================================== */

#define ESM_TEST_RUN_POSITIVE() \
    ESM_TEST_POS_ESMCLRSTATUS(); \
    ESM_TEST_POS_ESMGETCFG(); \
    ESM_TEST_POS_ESMGETENABLESTATE(); \
    ESM_TEST_POS_ESMGETERRCNT(); \
    ESM_TEST_POS_ESMGETSTARTSTATE(); \
    ESM_TEST_POS_ESMGETSTATUS(); \
    ESM_TEST_POS_ESMSETCFG(); \
    ESM_TEST_POS_ESMSETENABLESTATE(); \
    ESM_TEST_POS_ESMSETSTARTSTATE(); \
    ESM_TEST_POS_ESMSTART(); \
    ESM_TEST_POS_ESMSTOP(); \
    PLATFORM_RUN_TEST(test_pos_esm_integration_completeConfigurationSequence); \
    PLATFORM_RUN_TEST(test_pos_esm_integration_enableConfigureStartSequence)

#define ESM_TEST_RUN_NEGATIVE() \
    ESM_TEST_NEG_ESMCLRSTATUS(); \
    ESM_TEST_NEG_ESMGETCFG(); \
    ESM_TEST_NEG_ESMGETENABLESTATE(); \
    ESM_TEST_NEG_ESMGETERRCNT(); \
    ESM_TEST_NEG_ESMGETSTARTSTATE(); \
    ESM_TEST_NEG_ESMGETSTATUS(); \
    ESM_TEST_NEG_ESMSETCFG(); \
    ESM_TEST_NEG_ESMSETENABLESTATE(); \
    ESM_TEST_NEG_ESMSETSTARTSTATE(); \
    ESM_TEST_NEG_ESMSTART(); \
    ESM_TEST_NEG_ESMSTOP()

#define ESM_TEST_RUN_ALL() \
    ESM_TEST_RUN_POSITIVE(); \
    ESM_TEST_RUN_NEGATIVE()

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
/*                  esmClrStatus API Tests                                    */
/* ========================================================================== */
void test_pos_esm_esmClrStatus_allFields(void);
void test_pos_esm_esmClrStatus_failInt(void);
void test_pos_esm_esmClrStatus_pinInt(void);
void test_pos_esm_esmClrStatus_rstInt(void);
void test_neg_esm_esmClrStatus_invalidValidParams(void);
void test_neg_esm_esmClrStatus_nullEsmStat(void);
void test_neg_esm_esmClrStatus_nullHandle(void);
void test_neg_esm_esmClrStatus_zeroValidParams(void);
void test_neg_esm_esmClrStatus_ioWriteFailure(void);

/* ========================================================================== */
/*                  esmGetCfg API Tests                                       */
/* ========================================================================== */
void test_pos_esm_esmGetCfg_delay1(void);
void test_pos_esm_esmGetCfg_delay2(void);
void test_pos_esm_esmGetCfg_delay2Only(void);
void test_pos_esm_esmGetCfg_errCntThr(void);
void test_pos_esm_esmGetCfg_hmax(void);
void test_pos_esm_esmGetCfg_hmaxOnly(void);
void test_pos_esm_esmGetCfg_hmin(void);
void test_pos_esm_esmGetCfg_hminOnly(void);
void test_pos_esm_esmGetCfg_lmax(void);
void test_pos_esm_esmGetCfg_lmaxOnly(void);
void test_pos_esm_esmGetCfg_lmin(void);
void test_pos_esm_esmGetCfg_lminOnly(void);
void test_pos_esm_esmGetCfg_mode(void);
void test_neg_esm_esmGetCfg_invalidValidParams(void);
void test_neg_esm_esmGetCfg_nullEsmCfg(void);
void test_neg_esm_esmGetCfg_nullHandle(void);
void test_neg_esm_esmGetCfg_cascadeFailure_delayToHmax(void);
void test_neg_esm_esmGetCfg_cascadeFailure_modeToDelay(void);
void test_neg_esm_esmGetCfg_delay1ReadFailure(void);
void test_neg_esm_esmGetCfg_hmaxReadFailure(void);
void test_neg_esm_esmGetCfg_lmaxReadFailure(void);
void test_neg_esm_esmGetCfg_modeCfgReadFailure(void);

/* ========================================================================== */
/*                  esmGetEnableState API Tests                               */
/* ========================================================================== */
void test_pos_esm_esmGetEnableState_enableDisable(void);
void test_neg_esm_esmGetEnableState_nullHandle(void);
void test_neg_esm_esmGetEnableState_nullIsEnabled(void);
void test_neg_esm_esmGetEnableState_ioReadFailure(void);

/* ========================================================================== */
/*                  esmGetErrCnt API Tests                                    */
/* ========================================================================== */
void test_pos_esm_esmGetErrCnt_getCount(void);
void test_neg_esm_esmGetErrCnt_nullEsmErrCnt(void);
void test_neg_esm_esmGetErrCnt_nullHandle(void);
void test_neg_esm_esmGetErrCnt_ioReadFailure(void);

/* ========================================================================== */
/*                  esmGetStartState API Tests                                */
/* ========================================================================== */
void test_pos_esm_esmGetStartState_startStop(void);
void test_neg_esm_esmGetStartState_nullHandle(void);
void test_neg_esm_esmGetStartState_nullStarted(void);
void test_neg_esm_esmGetStartState_ioReadFailure(void);

/* ========================================================================== */
/*                  esmGetStatus API Tests                                    */
/* ========================================================================== */
void test_pos_esm_esmGetStatus_allFields(void);
void test_pos_esm_esmGetStatus_failInt(void);
void test_pos_esm_esmGetStatus_pinInt(void);
void test_pos_esm_esmGetStatus_rstInt(void);
void test_neg_esm_esmGetStatus_invalidValidParams(void);
void test_neg_esm_esmGetStatus_nullEsmStat(void);
void test_neg_esm_esmGetStatus_nullHandle(void);
void test_neg_esm_esmGetStatus_zeroValidParams(void);
void test_neg_esm_esmGetStatus_ioReadFailure(void);

/* ========================================================================== */
/*                  esmSetCfg API Tests                                       */
/* ========================================================================== */
void test_pos_esm_esmSetCfg_combinedConfiguration(void);
void test_pos_esm_esmSetCfg_configurationReadbackVerification(void);
void test_pos_esm_esmSetCfg_delay1(void);
void test_pos_esm_esmSetCfg_delay2(void);
void test_pos_esm_esmSetCfg_delay2Only(void);
void test_pos_esm_esmSetCfg_errCntThr(void);
void test_pos_esm_esmSetCfg_hmax(void);
void test_pos_esm_esmSetCfg_hmin(void);
void test_pos_esm_esmSetCfg_hminOnly(void);
void test_pos_esm_esmSetCfg_lmax(void);
void test_pos_esm_esmSetCfg_lmin(void);
void test_pos_esm_esmSetCfg_lminOnly(void);
void test_pos_esm_esmSetCfg_mode(void);
void test_pos_esm_esmSetCfg_pwmModeConfiguration(void);
void test_neg_esm_esmSetCfg_invalidErrCntThr(void);
void test_neg_esm_esmSetCfg_invalidMode(void);
void test_neg_esm_esmSetCfg_invalidValidParams(void);
void test_neg_esm_esmSetCfg_nullEsmCfg(void);
void test_neg_esm_esmSetCfg_nullHandle(void);
void test_neg_esm_esmSetCfg_cascadeFailure_delayToHmax(void);
void test_neg_esm_esmSetCfg_cascadeFailure_hmaxToLmax(void);
void test_neg_esm_esmSetCfg_delay1ReadFailure(void);
void test_neg_esm_esmSetCfg_hmaxReadFailure(void);
void test_neg_esm_esmSetCfg_lmaxReadFailure(void);
void test_neg_esm_esmSetCfg_modeCfgReadFailure(void);

/* ========================================================================== */
/*                  esmSetEnableState API Tests                               */
/* ========================================================================== */
void test_neg_esm_esmSetEnableState_nullHandle(void);
void test_neg_esm_esmSetEnableState_ioReadFailure(void);

/* ========================================================================== */
/*                  esmSetStartState API Tests                                */
/* ========================================================================== */
void test_neg_esm_esmSetStartState_nullHandle(void);
void test_neg_esm_esmSetStartState_ioReadFailure(void);

/* ========================================================================== */
/*                  esmStart API Tests                                        */
/* ========================================================================== */
void test_pos_esm_esmStart_start(void);
void test_neg_esm_esmStart_nullHandle(void);

/* ========================================================================== */
/*                  esmStop API Tests                                         */
/* ========================================================================== */
void test_pos_esm_esmStop_stop(void);
void test_neg_esm_esmStop_nullHandle(void);

/* ========================================================================== */
/*                  Integration Tests                                         */
/* ========================================================================== */
void test_pos_esm_integration_completeConfigurationSequence(void);
void test_pos_esm_integration_enableConfigureStartSequence(void);


#endif /* ESM_TEST_H */
