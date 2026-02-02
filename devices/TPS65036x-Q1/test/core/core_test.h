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
#ifndef PMIC_TEST_CORE_H
#define PMIC_TEST_CORE_H

/* ========================================================================== */
/*                              Include Files                                 */
/* ========================================================================== */

#include "test_utils.h"
#include "pmic_fsm.h"
#include "regmap/core.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* ========================================================================== */
/*                 API-Specific Test Macros - getNvmRev                       */
/* ========================================================================== */
#define CORE_TEST_POS_GETNVMREV() \
    PLATFORM_RUN_TEST(test_pos_core_getNvmRev)

#define CORE_TEST_NEG_GETNVMREV() \
    PLATFORM_RUN_TEST(test_neg_core_getNvmRev_nullParam_pmicHandle); \
    PLATFORM_RUN_TEST(test_neg_core_getNvmRev_nullParam_nvmRev)

/* Test: TC-CORE-0033 */
#define CORE_TEST_GETNVMREV() \
    CORE_TEST_POS_GETNVMREV(); \
    CORE_TEST_NEG_GETNVMREV()

/* ========================================================================== */
/*                 API-Specific Test Macros - getSiliconRev                   */
/* ========================================================================== */
#define CORE_TEST_POS_GETSILICONREV() \
    PLATFORM_RUN_TEST(test_pos_core_getSiliconRev)

#define CORE_TEST_NEG_GETSILICONREV() \
    PLATFORM_RUN_TEST(test_neg_core_getSiliconRev_nullParam_pmicHandle); \
    PLATFORM_RUN_TEST(test_neg_core_getSiliconRev_nullParam_siliconRev)

/* Test: TC-CORE-0034 */
#define CORE_TEST_GETSILICONREV() \
    CORE_TEST_POS_GETSILICONREV(); \
    CORE_TEST_NEG_GETSILICONREV()

/* ================================================================================================================================== */
/* API-Specific Test Macros - setRegLockState, getRegLockState, setScratchPadValue, getScratchPadValue, disableRegLock, enableRegLock */
/* ================================================================================================================================== */
#define CORE_TEST_POS_SETREGLOCKSTATE() \
    PLATFORM_RUN_TEST(test_pos_core_setGetRegLock)

#define CORE_TEST_NEG_SETREGLOCKSTATE() \
    PLATFORM_RUN_TEST(test_neg_core_setRegLockState_nullParam_pmicHandle); \
    PLATFORM_RUN_TEST(test_neg_core_disableRegLock_nullParam_pmicHandle); \
    PLATFORM_RUN_TEST(test_neg_core_enableRegLock_nullParam_pmicHandle)

/* Test: TC-CORE-0035 */
#define CORE_TEST_SETREGLOCKSTATE() \
    CORE_TEST_POS_SETREGLOCKSTATE(); \
    CORE_TEST_NEG_SETREGLOCKSTATE()

/* ========================================================================== */
/*                 API-Specific Test Macros - getRegLockState                 */
/* ========================================================================== */
#define CORE_TEST_NEG_GETREGLOCKSTATE() \
    PLATFORM_RUN_TEST(test_neg_core_getRegLockState_nullParam_pmicHandle); \
    PLATFORM_RUN_TEST(test_neg_core_getRegLockState_nullParam_regLockStat)

/* Test: TC-CORE-0036 */
#define CORE_TEST_GETREGLOCKSTATE() \
    CORE_TEST_NEG_GETREGLOCKSTATE()

/* ============================================================================================== */
/* API-Specific Test Macros - ioSetCrcEnableState, ioGetCrcEnableState, ioCrcEnable, ioCrcDisable */
/* ============================================================================================== */
#define CORE_TEST_POS_IOSETCRCENABLESTATE() \
    PLATFORM_RUN_TEST(test_pos_core_enableDisableCRC8)

#define CORE_TEST_NEG_IOSETCRCENABLESTATE() \
    PLATFORM_RUN_TEST(test_neg_core_ioSetCrcEnableState_nullParam_pmicHandle); \
    PLATFORM_RUN_TEST(test_neg_core_ioCrcEnable_nullParam_pmicHandle); \
    PLATFORM_RUN_TEST(test_neg_core_ioCrcDisable_nullParam_pmicHandle)

/* Test: TC-CORE-0037 */
#define CORE_TEST_IOSETCRCENABLESTATE() \
    CORE_TEST_POS_IOSETCRCENABLESTATE(); \
    CORE_TEST_NEG_IOSETCRCENABLESTATE()

/* ========================================================================== */
/*                 API-Specific Test Macros - ioGetCrcEnableState             */
/* ========================================================================== */
#define CORE_TEST_NEG_IOGETCRCENABLESTATE() \
    PLATFORM_RUN_TEST(test_neg_core_ioGetCrcEnableState_nullParam_pmicHandle); \
    PLATFORM_RUN_TEST(test_neg_core_ioGetCrcEnableState_nullParam_crcEnabled)

/* Test: TC-CORE-0038 */
#define CORE_TEST_IOGETCRCENABLESTATE() \
    CORE_TEST_NEG_IOGETCRCENABLESTATE()

/* ========================================================================== */
/*                 API-Specific Test Macros - fsmSetDevState                  */
/* ========================================================================== */
#define CORE_TEST_NEG_FSMSETDEVSTATE() \
    PLATFORM_RUN_TEST(test_neg_core_fsmSetDevState_nullParam_pmicHandle); \
    PLATFORM_RUN_TEST(test_neg_core_fsmSetDevState_invalid_fsmCmd)

/* Test: TC-CORE-0039 */
#define CORE_TEST_FSMSETDEVSTATE() \
    CORE_TEST_NEG_FSMSETDEVSTATE()

/* ========================================================================== */
/*                  API-Specific Test Macros - setPwrOn                       */
/* ========================================================================== */
#define CORE_TEST_POS_SETPWRON() \
    PLATFORM_RUN_TEST(test_pos_core_setGetPwrOn)

#define CORE_TEST_NEG_SETPWRON() \
    PLATFORM_RUN_TEST(test_neg_core_setPwrOn_nullParam_pmicHandle)

/* Test: TC-CORE-0040 */
#define CORE_TEST_SETPWRON() \
    CORE_TEST_POS_SETPWRON(); \
    CORE_TEST_NEG_SETPWRON()

/* ========================================================================== */
/*                 API-Specific Test Macros - getPwrOn                        */
/* ========================================================================== */
#define CORE_TEST_NEG_GETPWRON() \
    PLATFORM_RUN_TEST(test_neg_core_getPwrOn_nullParam_pmicHandle); \
    PLATFORM_RUN_TEST(test_neg_core_getPwrOn_nullParam_pwrOnStat)

/* Test: TC-CORE-0041 */
#define CORE_TEST_GETPWRON() \
    CORE_TEST_NEG_GETPWRON()

/* ========================================================================== */
/*                 API-Specific Test Macros - setLpmCfg                       */
/* ========================================================================== */
#define CORE_TEST_POS_SETLPMCFG() \
    PLATFORM_RUN_TEST(test_pos_core_setGetLpmCfg_pinDetection); \
    PLATFORM_RUN_TEST(test_pos_core_setGetLpmCfg_detectionDelay); \
    PLATFORM_RUN_TEST(test_pos_core_setGetLpmCfg_vmonEn); \
    PLATFORM_RUN_TEST(test_pos_core_setGetLpmCfg_esmEn); \
    PLATFORM_RUN_TEST(test_pos_core_setGetLpmCfg_wdgEn); \
    PLATFORM_RUN_TEST(test_pos_core_setLpmCfg_pinDetection_allValues); \
    PLATFORM_RUN_TEST(test_pos_core_setLpmCfg_detectionDelay_allValues); \
    PLATFORM_RUN_TEST(test_pos_core_setLpmCfg_vmonEn_enable); \
    PLATFORM_RUN_TEST(test_pos_core_setLpmCfg_vmonEn_disable); \
    PLATFORM_RUN_TEST(test_pos_core_setLpmCfg_esmEn_enable); \
    PLATFORM_RUN_TEST(test_pos_core_setLpmCfg_esmEn_disable); \
    PLATFORM_RUN_TEST(test_pos_core_setLpmCfg_wdgEn_enable); \
    PLATFORM_RUN_TEST(test_pos_core_setLpmCfg_wdgEn_disable); \
    PLATFORM_RUN_TEST(test_pos_core_setLpmCfg_multipleEnables); \
    PLATFORM_RUN_TEST(test_pos_core_setLpmCfg_allParams); \
    PLATFORM_RUN_TEST(test_pos_core_setLpmCfg_pinDetection_boundaryMin); \
    PLATFORM_RUN_TEST(test_pos_core_setLpmCfg_pinDetection_boundaryMax); \
    PLATFORM_RUN_TEST(test_pos_core_setLpmCfg_detectionDelay_boundaryMin); \
    PLATFORM_RUN_TEST(test_pos_core_setLpmCfg_detectionDelay_boundaryMax); \
    PLATFORM_RUN_TEST(test_pos_core_setLpmCfg_pinDetectionAndDelay)

#define CORE_TEST_NEG_SETLPMCFG() \
    PLATFORM_RUN_TEST(test_neg_core_setLpmCfg_nullParam_pmicHandle); \
    PLATFORM_RUN_TEST(test_neg_core_setLpmCfg_nullParam_lpmCfg); \
    PLATFORM_RUN_TEST(test_neg_core_setLpmCfg_outOfBounds_pinDetection); \
    PLATFORM_RUN_TEST(test_neg_core_setLpmCfg_outOfBounds_detectionDelay); \
    PLATFORM_RUN_TEST(test_neg_core_setLpmCfg_zeroValidParams)

/* Test: TC-CORE-0042 */
#define CORE_TEST_SETLPMCFG() \
    CORE_TEST_POS_SETLPMCFG(); \
    CORE_TEST_NEG_SETLPMCFG()

/* ========================================================================== */
/*                 API-Specific Test Macros - getLpmCfg                       */
/* ========================================================================== */
#define CORE_TEST_POS_GETLPMCFG() \
    PLATFORM_RUN_TEST(test_pos_core_getLpmCfg_pinDetection); \
    PLATFORM_RUN_TEST(test_pos_core_getLpmCfg_detectionDelay); \
    PLATFORM_RUN_TEST(test_pos_core_getLpmCfg_vmonEn); \
    PLATFORM_RUN_TEST(test_pos_core_getLpmCfg_esmEn); \
    PLATFORM_RUN_TEST(test_pos_core_getLpmCfg_wdgEn); \
    PLATFORM_RUN_TEST(test_pos_core_getLpmCfg_multipleParams)

#define CORE_TEST_NEG_GETLPMCFG() \
    PLATFORM_RUN_TEST(test_neg_core_getLpmCfg_nullParam_pmicHandle); \
    PLATFORM_RUN_TEST(test_neg_core_getLpmCfg_nullParam_lpmCfg); \
    PLATFORM_RUN_TEST(test_neg_core_getLpmCfg_invalidParam_validParams); \
    PLATFORM_RUN_TEST(test_neg_core_getLpmCfg_zeroValidParams)

/* Test: TC-CORE-0043 */
#define CORE_TEST_GETLPMCFG() \
    CORE_TEST_POS_GETLPMCFG(); \
    CORE_TEST_NEG_GETLPMCFG()

/* ========================================================================== */
/*                 API-Specific Test Macros - runABIST                        */
/* ========================================================================== */
#define CORE_TEST_POS_RUNABIST() \
    PLATFORM_RUN_TEST(test_pos_core_runABIST)

#define CORE_TEST_NEG_RUNABIST() \
    PLATFORM_RUN_TEST(test_neg_core_runABIST_nullParam_pmicHandle)

/* Test: TC-CORE-0044 */
#define CORE_TEST_RUNABIST() \
    CORE_TEST_POS_RUNABIST(); \
    CORE_TEST_NEG_RUNABIST()

/* ========================================================================== */
/*                 API-Specific Test Macros - getABISTStat                    */
/* ========================================================================== */
#define CORE_TEST_POS_GETABISTSTAT() \
    PLATFORM_RUN_TEST(test_pos_core_getABISTStat_active)

#define CORE_TEST_NEG_GETABISTSTAT() \
    PLATFORM_RUN_TEST(test_neg_core_getABISTStat_nullParam_pmicHandle); \
    PLATFORM_RUN_TEST(test_neg_core_getABISTStat_nullParam_isActive)

/* Test: TC-CORE-0045 */
#define CORE_TEST_GETABISTSTAT() \
    CORE_TEST_POS_GETABISTSTAT(); \
    CORE_TEST_NEG_GETABISTSTAT()

/* ========================================================================== */
/*                 API-Specific Test Macros - setScratchPadValue              */
/* ========================================================================== */
#define CORE_TEST_POS_SETSCRATCHPADVALUE() \
    PLATFORM_RUN_TEST(test_pos_core_setGetScratchPadVal)

#define CORE_TEST_NEG_SETSCRATCHPADVALUE() \
    PLATFORM_RUN_TEST(test_neg_core_setScratchPadValue_nullParam_pmicHandle); \
    PLATFORM_RUN_TEST(test_neg_core_setScratchPadValue_outOfBounds_scratchPadRegNum)

/* Test: TC-CORE-0046 */
#define CORE_TEST_SETSCRATCHPADVALUE() \
    CORE_TEST_POS_SETSCRATCHPADVALUE(); \
    CORE_TEST_NEG_SETSCRATCHPADVALUE()

/* ========================================================================== */
/*                 API-Specific Test Macros - getScratchPadValue              */
/* ========================================================================== */
#define CORE_TEST_NEG_GETSCRATCHPADVALUE() \
    PLATFORM_RUN_TEST(test_neg_core_getScratchPadValue_nullParam_pmicHandle); \
    PLATFORM_RUN_TEST(test_neg_core_getScratchPadValue_outOfBounds_scratchPadRegNum); \
    PLATFORM_RUN_TEST(test_neg_core_getScratchPadValue_nullParam_value)

/* Test: TC-CORE-0047 */
#define CORE_TEST_GETSCRATCHPADVALUE() \
    CORE_TEST_NEG_GETSCRATCHPADVALUE()

/* ========================================================================== */
/*                 API-Specific Test Macros - fsmSetRecovCntThr               */
/* ========================================================================== */
#define CORE_TEST_POS_FSMSETRECOVCNTTHR() \
    PLATFORM_RUN_TEST(test_pos_core_setGetRecovCntThr)

#define CORE_TEST_NEG_FSMSETRECOVCNTTHR() \
    PLATFORM_RUN_TEST(test_neg_core_fsmSetRecovCntThr_nullParam_pmicHandle); \
    PLATFORM_RUN_TEST(test_neg_core_fsmSetRecovCntThr_outOfBounds_threshold)

/* Test: TC-CORE-0048 */
#define CORE_TEST_FSMSETRECOVCNTTHR() \
    CORE_TEST_POS_FSMSETRECOVCNTTHR(); \
    CORE_TEST_NEG_FSMSETRECOVCNTTHR()

/* ========================================================================== */
/*                 API-Specific Test Macros - fsmGetRecovCntThr               */
/* ========================================================================== */
#define CORE_TEST_NEG_FSMGETRECOVCNTTHR() \
    PLATFORM_RUN_TEST(test_neg_core_fsmGetRecovCntThr_nullParam_pmicHandle); \
    PLATFORM_RUN_TEST(test_neg_core_fsmGetRecovCntThr_nullParam_threshold)

/* Test: TC-CORE-0049 */
#define CORE_TEST_FSMGETRECOVCNTTHR() \
    CORE_TEST_NEG_FSMGETRECOVCNTTHR()

/* ========================================================================== */
/*                 API-Specific Test Macros - fsmGetRecovCnt                  */
/* ========================================================================== */
#define CORE_TEST_POS_FSMGETRECOVCNT() \
    PLATFORM_RUN_TEST(test_pos_core_getClrRecovCnt)

#define CORE_TEST_NEG_FSMGETRECOVCNT() \
    PLATFORM_RUN_TEST(test_neg_core_fsmGetRecovCnt_nullParam_pmicHandle); \
    PLATFORM_RUN_TEST(test_neg_core_fsmGetRecovCnt_nullParam_recovCnt)

/* Test: TC-CORE-0050 */
#define CORE_TEST_FSMGETRECOVCNT() \
    CORE_TEST_POS_FSMGETRECOVCNT(); \
    CORE_TEST_NEG_FSMGETRECOVCNT()

/* ========================================================================== */
/*                 API-Specific Test Macros - fsmClrRecovCnt                  */
/* ========================================================================== */
#define CORE_TEST_NEG_FSMCLRRECOVCNT() \
    PLATFORM_RUN_TEST(test_neg_core_fsmClrRecovCnt_nullParam_pmicHandle)

/* Test: TC-CORE-0051 */
#define CORE_TEST_FSMCLRRECOVCNT() \
    CORE_TEST_NEG_FSMCLRRECOVCNT()

/* ========================================================================== */
/*                 API-Specific Test Macros - fsmSetResetCntThr               */
/* ========================================================================== */
#define CORE_TEST_POS_FSMSETRESETCNTTHR() \
    PLATFORM_RUN_TEST(test_pos_core_setGetResetCntThr)

#define CORE_TEST_NEG_FSMSETRESETCNTTHR() \
    PLATFORM_RUN_TEST(test_neg_core_fsmSetResetCntThr_nullParam_pmicHandle); \
    PLATFORM_RUN_TEST(test_neg_core_fsmSetResetCntThr_outOfBounds_threshold)

/* Test: TC-CORE-0052 */
#define CORE_TEST_FSMSETRESETCNTTHR() \
    CORE_TEST_POS_FSMSETRESETCNTTHR(); \
    CORE_TEST_NEG_FSMSETRESETCNTTHR()

/* ========================================================================== */
/*                API-Specific Test Macros - fsmGetResetCntThr                */
/* ========================================================================== */
#define CORE_TEST_NEG_FSMGETRESETCNTTHR() \
    PLATFORM_RUN_TEST(test_neg_core_fsmGetResetCntThr_nullParam_pmicHandle); \
    PLATFORM_RUN_TEST(test_neg_core_fsmGetResetCntThr_nullParam_threshold)

/* Test: TC-CORE-0053 */
#define CORE_TEST_FSMGETRESETCNTTHR() \
    CORE_TEST_NEG_FSMGETRESETCNTTHR()

/* ========================================================================== */
/*                 API-Specific Test Macros - fsmGetResetCnt                  */
/* ========================================================================== */
#define CORE_TEST_POS_FSMGETRESETCNT() \
    PLATFORM_RUN_TEST(test_pos_core_getClrResetCnt)

#define CORE_TEST_NEG_FSMGETRESETCNT() \
    PLATFORM_RUN_TEST(test_neg_core_fsmGetResetCnt_nullParam_pmicHandle); \
    PLATFORM_RUN_TEST(test_neg_core_fsmGetResetCnt_nullParam_resetCnt)

/* Test: TC-CORE-0054 */
#define CORE_TEST_FSMGETRESETCNT() \
    CORE_TEST_POS_FSMGETRESETCNT(); \
    CORE_TEST_NEG_FSMGETRESETCNT()

/* ========================================================================== */
/*              API-Specific Test Macros - fsmClrResetCnt                     */
/* ========================================================================== */
#define CORE_TEST_NEG_FSMCLRRESETCNT() \
    PLATFORM_RUN_TEST(test_neg_core_fsmClrResetCnt_nullParam_pmicHandle)

/* Test: TC-CORE-0055 */
#define CORE_TEST_FSMCLRRESETCNT() \
    CORE_TEST_NEG_FSMCLRRESETCNT()

/* ========================================================================== */
/*                 API-Specific Test Macros - setCRC16Cfg                     */
/* ========================================================================== */
#define CORE_TEST_POS_SETCRC16CFG() \
    PLATFORM_RUN_TEST(test_pos_core_setCRC16Cfg_enable); \
    PLATFORM_RUN_TEST(test_pos_core_setCRC16Cfg_activateCalc); \
    PLATFORM_RUN_TEST(test_pos_core_setCRC16Cfg_combinedParams)

#define CORE_TEST_NEG_SETCRC16CFG() \
    PLATFORM_RUN_TEST(test_neg_core_setCRC16Cfg_nullParam_handle); \
    PLATFORM_RUN_TEST(test_neg_core_setCRC16Cfg_nullParam_crc16Cfg); \
    PLATFORM_RUN_TEST(test_neg_core_setCRC16Cfg_invalidParam_validParams); \
    PLATFORM_RUN_TEST(test_neg_core_setCRC16Cfg_zeroValidParams)

/* Test: TC-CORE-0056 */
#define CORE_TEST_SETCRC16CFG() \
    CORE_TEST_POS_SETCRC16CFG(); \
    CORE_TEST_NEG_SETCRC16CFG()

/* ========================================================================== */
/*                 API-Specific Test Macros - getCRC16Cfg                     */
/* ========================================================================== */
#define CORE_TEST_POS_GETCRC16CFG() \
    PLATFORM_RUN_TEST(test_pos_core_getCRC16Cfg_enable); \
    PLATFORM_RUN_TEST(test_pos_core_getCRC16Cfg_activateCalc); \
    PLATFORM_RUN_TEST(test_pos_core_getCRC16Cfg_combinedParams)

#define CORE_TEST_NEG_GETCRC16CFG() \
    PLATFORM_RUN_TEST(test_neg_core_getCRC16Cfg_nullParam_handle); \
    PLATFORM_RUN_TEST(test_neg_core_getCRC16Cfg_nullParam_crc16Cfg); \
    PLATFORM_RUN_TEST(test_neg_core_getCRC16Cfg_invalidParam_validParams); \
    PLATFORM_RUN_TEST(test_neg_core_getCRC16Cfg_zeroValidParams)

/* Test: TC-CORE-0057 */
#define CORE_TEST_GETCRC16CFG() \
    CORE_TEST_POS_GETCRC16CFG(); \
    CORE_TEST_NEG_GETCRC16CFG()

/* =========================================================================== */
/* API-Specific Test Macros - setCRC16Cfg, getCRC16Cfg, init, deinit, ioRxByte */
/* =========================================================================== */
#define CORE_TEST_POS_SILICON() \
    PLATFORM_RUN_TEST(test_pos_core_silicon_A0_crc16_at_0x61); \
    PLATFORM_RUN_TEST(test_pos_core_silicon_B0_crc16_at_0x64); \
    PLATFORM_RUN_TEST(test_pos_core_silicon_B1_crc16_at_0x64); \
    PLATFORM_RUN_TEST(test_pos_core_init_A0_silicon_with_locked_registers)

/* Test: TC-CORE-0058 */
#define CORE_TEST_SILICON() \
    CORE_TEST_POS_SILICON()

/* ========================================================================== */
/*                        Aggregate Test Macros                               */
/* ========================================================================== */

#define CORE_TEST_RUN_POSITIVE() \
    CORE_TEST_POS_GETNVMREV(); \
    CORE_TEST_POS_GETSILICONREV(); \
    CORE_TEST_POS_SETREGLOCKSTATE(); \
    CORE_TEST_POS_IOSETCRCENABLESTATE(); \
    CORE_TEST_POS_SETPWRON(); \
    CORE_TEST_POS_SETLPMCFG(); \
    CORE_TEST_POS_GETLPMCFG(); \
    CORE_TEST_POS_RUNABIST(); \
    CORE_TEST_POS_GETABISTSTAT(); \
    CORE_TEST_POS_SETSCRATCHPADVALUE(); \
    CORE_TEST_POS_FSMSETRECOVCNTTHR(); \
    CORE_TEST_POS_FSMGETRECOVCNT(); \
    CORE_TEST_POS_FSMSETRESETCNTTHR(); \
    CORE_TEST_POS_FSMGETRESETCNT(); \
    CORE_TEST_POS_SETCRC16CFG(); \
    CORE_TEST_POS_GETCRC16CFG(); \
    CORE_TEST_POS_SILICON()

#define CORE_TEST_RUN_NEGATIVE() \
    CORE_TEST_NEG_GETNVMREV(); \
    CORE_TEST_NEG_GETSILICONREV(); \
    CORE_TEST_NEG_SETREGLOCKSTATE(); \
    CORE_TEST_NEG_GETREGLOCKSTATE(); \
    CORE_TEST_NEG_IOSETCRCENABLESTATE(); \
    CORE_TEST_NEG_IOGETCRCENABLESTATE(); \
    CORE_TEST_NEG_FSMSETDEVSTATE(); \
    CORE_TEST_NEG_SETPWRON(); \
    CORE_TEST_NEG_GETPWRON(); \
    CORE_TEST_NEG_SETLPMCFG(); \
    CORE_TEST_NEG_GETLPMCFG(); \
    CORE_TEST_NEG_RUNABIST(); \
    CORE_TEST_NEG_GETABISTSTAT(); \
    CORE_TEST_NEG_SETSCRATCHPADVALUE(); \
    CORE_TEST_NEG_GETSCRATCHPADVALUE(); \
    CORE_TEST_NEG_FSMSETRECOVCNTTHR(); \
    CORE_TEST_NEG_FSMGETRECOVCNTTHR(); \
    CORE_TEST_NEG_FSMGETRECOVCNT(); \
    CORE_TEST_NEG_FSMCLRRECOVCNT(); \
    CORE_TEST_NEG_FSMSETRESETCNTTHR(); \
    CORE_TEST_NEG_FSMGETRESETCNTTHR(); \
    CORE_TEST_NEG_FSMGETRESETCNT(); \
    CORE_TEST_NEG_FSMCLRRESETCNT(); \
    CORE_TEST_NEG_SETCRC16CFG(); \
    CORE_TEST_NEG_GETCRC16CFG()

#define CORE_TEST_RUN_ALL() \
    CORE_TEST_RUN_POSITIVE(); \
    CORE_TEST_RUN_NEGATIVE()

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

void core_test(void *args);

/* getNvmRev API tests */
void test_neg_core_getNvmRev_nullParam_pmicHandle(void);
void test_neg_core_getNvmRev_nullParam_nvmRev(void);
void test_pos_core_getNvmRev(void);

/* getSiliconRev API tests */
void test_neg_core_getSiliconRev_nullParam_pmicHandle(void);
void test_neg_core_getSiliconRev_nullParam_siliconRev(void);
void test_pos_core_getSiliconRev(void);

/* setRegLockState API tests */
void test_neg_core_setRegLockState_nullParam_pmicHandle(void);
void test_neg_core_disableRegLock_nullParam_pmicHandle(void);
void test_neg_core_enableRegLock_nullParam_pmicHandle(void);
void test_pos_core_setGetRegLock(void);

/* getRegLockState API tests */
void test_neg_core_getRegLockState_nullParam_pmicHandle(void);
void test_neg_core_getRegLockState_nullParam_regLockStat(void);

/* ioSetCrcEnableState API tests */
void test_neg_core_ioSetCrcEnableState_nullParam_pmicHandle(void);
void test_neg_core_ioCrcEnable_nullParam_pmicHandle(void);
void test_neg_core_ioCrcDisable_nullParam_pmicHandle(void);
void test_pos_core_enableDisableCRC8(void);

/* ioGetCrcEnableState API tests */
void test_neg_core_ioGetCrcEnableState_nullParam_pmicHandle(void);
void test_neg_core_ioGetCrcEnableState_nullParam_crcEnabled(void);

/* fsmSetDevState API tests */
void test_neg_core_fsmSetDevState_nullParam_pmicHandle(void);
void test_neg_core_fsmSetDevState_invalid_fsmCmd(void);

/* setPwrOn API tests */
void test_neg_core_setPwrOn_nullParam_pmicHandle(void);
void test_pos_core_setGetPwrOn(void);

/* getPwrOn API tests */
void test_neg_core_getPwrOn_nullParam_pmicHandle(void);
void test_neg_core_getPwrOn_nullParam_pwrOnStat(void);

/* setLpmCfg API tests */
void test_neg_core_setLpmCfg_nullParam_pmicHandle(void);
void test_neg_core_setLpmCfg_nullParam_lpmCfg(void);
void test_neg_core_setLpmCfg_outOfBounds_pinDetection(void);
void test_neg_core_setLpmCfg_outOfBounds_detectionDelay(void);
void test_neg_core_setLpmCfg_zeroValidParams(void);
void test_pos_core_setGetLpmCfg_pinDetection(void);
void test_pos_core_setGetLpmCfg_detectionDelay(void);
void test_pos_core_setGetLpmCfg_vmonEn(void);
void test_pos_core_setGetLpmCfg_esmEn(void);
void test_pos_core_setGetLpmCfg_wdgEn(void);
void test_pos_core_setLpmCfg_pinDetection_allValues(void);
void test_pos_core_setLpmCfg_detectionDelay_allValues(void);
void test_pos_core_setLpmCfg_vmonEn_enable(void);
void test_pos_core_setLpmCfg_vmonEn_disable(void);
void test_pos_core_setLpmCfg_esmEn_enable(void);
void test_pos_core_setLpmCfg_esmEn_disable(void);
void test_pos_core_setLpmCfg_wdgEn_enable(void);
void test_pos_core_setLpmCfg_wdgEn_disable(void);
void test_pos_core_setLpmCfg_multipleEnables(void);
void test_pos_core_setLpmCfg_allParams(void);
void test_pos_core_setLpmCfg_pinDetection_boundaryMin(void);
void test_pos_core_setLpmCfg_pinDetection_boundaryMax(void);
void test_pos_core_setLpmCfg_detectionDelay_boundaryMin(void);
void test_pos_core_setLpmCfg_detectionDelay_boundaryMax(void);
void test_pos_core_setLpmCfg_pinDetectionAndDelay(void);

/* getLpmCfg API tests */
void test_neg_core_getLpmCfg_nullParam_pmicHandle(void);
void test_neg_core_getLpmCfg_nullParam_lpmCfg(void);
void test_neg_core_getLpmCfg_invalidParam_validParams(void);
void test_neg_core_getLpmCfg_zeroValidParams(void);
void test_pos_core_getLpmCfg_pinDetection(void);
void test_pos_core_getLpmCfg_detectionDelay(void);
void test_pos_core_getLpmCfg_vmonEn(void);
void test_pos_core_getLpmCfg_esmEn(void);
void test_pos_core_getLpmCfg_wdgEn(void);
void test_pos_core_getLpmCfg_multipleParams(void);

/* runABIST API tests */
void test_neg_core_runABIST_nullParam_pmicHandle(void);
void test_pos_core_runABIST(void);

/* getABISTStat API tests */
void test_neg_core_getABISTStat_nullParam_pmicHandle(void);
void test_neg_core_getABISTStat_nullParam_isActive(void);
void test_pos_core_getABISTStat_active(void);

/* setScratchPadValue API tests */
void test_neg_core_setScratchPadValue_nullParam_pmicHandle(void);
void test_neg_core_setScratchPadValue_outOfBounds_scratchPadRegNum(void);
void test_pos_core_setGetScratchPadVal(void);

/* getScratchPadValue API tests */
void test_neg_core_getScratchPadValue_nullParam_pmicHandle(void);
void test_neg_core_getScratchPadValue_outOfBounds_scratchPadRegNum(void);
void test_neg_core_getScratchPadValue_nullParam_value(void);

/* fsmSetRecovCntThr API tests */
void test_neg_core_fsmSetRecovCntThr_nullParam_pmicHandle(void);
void test_neg_core_fsmSetRecovCntThr_outOfBounds_threshold(void);
void test_pos_core_setGetRecovCntThr(void);

/* fsmGetRecovCntThr API tests */
void test_neg_core_fsmGetRecovCntThr_nullParam_pmicHandle(void);
void test_neg_core_fsmGetRecovCntThr_nullParam_threshold(void);

/* fsmGetRecovCnt API tests */
void test_neg_core_fsmGetRecovCnt_nullParam_pmicHandle(void);
void test_neg_core_fsmGetRecovCnt_nullParam_recovCnt(void);
void test_pos_core_getClrRecovCnt(void);

/* fsmClrRecovCnt API tests */
void test_neg_core_fsmClrRecovCnt_nullParam_pmicHandle(void);

/* fsmSetResetCntThr API tests */
void test_neg_core_fsmSetResetCntThr_nullParam_pmicHandle(void);
void test_neg_core_fsmSetResetCntThr_outOfBounds_threshold(void);
void test_pos_core_setGetResetCntThr(void);

/* fsmGetResetCntThr API tests */
void test_neg_core_fsmGetResetCntThr_nullParam_pmicHandle(void);
void test_neg_core_fsmGetResetCntThr_nullParam_threshold(void);

/* fsmGetResetCnt API tests */
void test_neg_core_fsmGetResetCnt_nullParam_pmicHandle(void);
void test_neg_core_fsmGetResetCnt_nullParam_resetCnt(void);
void test_pos_core_getClrResetCnt(void);

/* fsmClrResetCnt API tests */
void test_neg_core_fsmClrResetCnt_nullParam_pmicHandle(void);

/* setCRC16Cfg API tests */
void test_neg_core_setCRC16Cfg_nullParam_handle(void);
void test_neg_core_setCRC16Cfg_nullParam_crc16Cfg(void);
void test_neg_core_setCRC16Cfg_invalidParam_validParams(void);
void test_neg_core_setCRC16Cfg_zeroValidParams(void);
void test_pos_core_setCRC16Cfg_enable(void);
void test_pos_core_setCRC16Cfg_activateCalc(void);
void test_pos_core_setCRC16Cfg_combinedParams(void);

/* getCRC16Cfg API tests */
void test_neg_core_getCRC16Cfg_nullParam_handle(void);
void test_neg_core_getCRC16Cfg_nullParam_crc16Cfg(void);
void test_neg_core_getCRC16Cfg_invalidParam_validParams(void);
void test_neg_core_getCRC16Cfg_zeroValidParams(void);
void test_pos_core_getCRC16Cfg_enable(void);
void test_pos_core_getCRC16Cfg_activateCalc(void);
void test_pos_core_getCRC16Cfg_combinedParams(void);

/* Silicon revision tests */
void test_pos_core_silicon_A0_crc16_at_0x61(void);
void test_pos_core_silicon_B0_crc16_at_0x64(void);
void test_pos_core_silicon_B1_crc16_at_0x64(void);
void test_pos_core_init_A0_silicon_with_locked_registers(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* PMIC_TEST_CORE_H */
