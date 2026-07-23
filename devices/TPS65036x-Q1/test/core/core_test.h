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
#include "regmap/core.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* ======================================================================== */
/*                           Test APIs: getNvmRev                           */
/* ======================================================================== */
#define CORE_TEST_POS_GETNVMREV() \
    PLATFORM_RUN_TEST(test_pos_core_getNvmRev)

#define CORE_TEST_NEG_GETNVMREV() \
    PLATFORM_RUN_TEST(test_neg_core_getNvmRev_nullParam_pmicHandle); \
    PLATFORM_RUN_TEST(test_neg_core_getNvmRev_nullParam_nvmRev)

/* Test: TC-CORE-0033 */
#define CORE_TEST_GETNVMREV() \
    CORE_TEST_POS_GETNVMREV(); \
    CORE_TEST_NEG_GETNVMREV()

/* ======================================================================== */
/*                         Test APIs: getSiliconRev                         */
/* ======================================================================== */
#define CORE_TEST_POS_GETSILICONREV() \
    PLATFORM_RUN_TEST(test_pos_core_getSiliconRev)

#define CORE_TEST_NEG_GETSILICONREV() \
    PLATFORM_RUN_TEST(test_neg_core_getSiliconRev_nullParam_pmicHandle); \
    PLATFORM_RUN_TEST(test_neg_core_getSiliconRev_nullParam_siliconRev)

/* Test: TC-CORE-0034 */
#define CORE_TEST_GETSILICONREV() \
    CORE_TEST_POS_GETSILICONREV(); \
    CORE_TEST_NEG_GETSILICONREV()

/* ======================================================================== */
/*     Test APIs: setRegLockState, getRegLockState, setScratchPadValue,     */
/*                getScratchPadValue, disableRegLock, enableRegLock         */
/* ======================================================================== */
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

/* ======================================================================== */
/*                        Test APIs: getRegLockState                        */
/* ======================================================================== */
#define CORE_TEST_NEG_GETREGLOCKSTATE() \
    PLATFORM_RUN_TEST(test_neg_core_getRegLockState_nullParam_pmicHandle); \
    PLATFORM_RUN_TEST(test_neg_core_getRegLockState_nullParam_regLockStat)

/* Test: TC-CORE-0036 */
#define CORE_TEST_GETREGLOCKSTATE() \
    CORE_TEST_NEG_GETREGLOCKSTATE()

/* ======================================================================== */
/*    Test APIs: ioSetCrcEnableState, ioGetCrcEnableState, ioCrcEnable,     */
/*               ioCrcDisable                                               */
/* ======================================================================== */
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

/* ======================================================================== */
/*                      Test APIs: ioGetCrcEnableState                      */
/* ======================================================================== */
#define CORE_TEST_NEG_IOGETCRCENABLESTATE() \
    PLATFORM_RUN_TEST(test_neg_core_ioGetCrcEnableState_nullParam_pmicHandle); \
    PLATFORM_RUN_TEST(test_neg_core_ioGetCrcEnableState_nullParam_crcEnabled)

/* Test: TC-CORE-0038 */
#define CORE_TEST_IOGETCRCENABLESTATE() \
    CORE_TEST_NEG_IOGETCRCENABLESTATE()

/* ======================================================================== */
/*                           Test APIs: setPwrOn                            */
/* ======================================================================== */
#define CORE_TEST_POS_SETPWRON() \
    PLATFORM_RUN_TEST(test_pos_core_setGetPwrOn)

#define CORE_TEST_NEG_SETPWRON() \
    PLATFORM_RUN_TEST(test_neg_core_setPwrOn_nullParam_pmicHandle)

/* Test: TC-CORE-0040 */
#define CORE_TEST_SETPWRON() \
    CORE_TEST_POS_SETPWRON(); \
    CORE_TEST_NEG_SETPWRON()

/* ======================================================================== */
/*                           Test APIs: getPwrOn                            */
/* ======================================================================== */
#define CORE_TEST_NEG_GETPWRON() \
    PLATFORM_RUN_TEST(test_neg_core_getPwrOn_nullParam_pmicHandle); \
    PLATFORM_RUN_TEST(test_neg_core_getPwrOn_nullParam_pwrOnStat)

/* Test: TC-CORE-0041 */
#define CORE_TEST_GETPWRON() \
    CORE_TEST_NEG_GETPWRON()

/* ======================================================================== */
/*                           Test APIs: setLpmCfg                           */
/* ======================================================================== */
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

/* ======================================================================== */
/*                           Test APIs: getLpmCfg                           */
/* ======================================================================== */
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

/* ======================================================================== */
/*                           Test APIs: runABIST                            */
/* ======================================================================== */
#define CORE_TEST_POS_RUNABIST() \
    PLATFORM_RUN_TEST(test_pos_core_runABIST)

#define CORE_TEST_NEG_RUNABIST() \
    PLATFORM_RUN_TEST(test_neg_core_runABIST_nullParam_pmicHandle)

/* Test: TC-CORE-0044 */
#define CORE_TEST_RUNABIST() \
    CORE_TEST_POS_RUNABIST(); \
    CORE_TEST_NEG_RUNABIST()

/* ======================================================================== */
/*                         Test APIs: getABISTStat                          */
/* ======================================================================== */
#define CORE_TEST_POS_GETABISTSTAT() \
    PLATFORM_RUN_TEST(test_pos_core_getABISTStat_active)

#define CORE_TEST_NEG_GETABISTSTAT() \
    PLATFORM_RUN_TEST(test_neg_core_getABISTStat_nullParam_pmicHandle); \
    PLATFORM_RUN_TEST(test_neg_core_getABISTStat_nullParam_isActive)

/* Test: TC-CORE-0045 */
#define CORE_TEST_GETABISTSTAT() \
    CORE_TEST_POS_GETABISTSTAT(); \
    CORE_TEST_NEG_GETABISTSTAT()

/* ======================================================================== */
/*                      Test APIs: setScratchPadValue                       */
/* ======================================================================== */
#define CORE_TEST_POS_SETSCRATCHPADVALUE() \
    PLATFORM_RUN_TEST(test_pos_core_setGetScratchPadVal)

#define CORE_TEST_NEG_SETSCRATCHPADVALUE() \
    PLATFORM_RUN_TEST(test_neg_core_setScratchPadValue_nullParam_pmicHandle); \
    PLATFORM_RUN_TEST(test_neg_core_setScratchPadValue_outOfBounds_scratchPadRegNum)

/* Test: TC-CORE-0046 */
#define CORE_TEST_SETSCRATCHPADVALUE() \
    CORE_TEST_POS_SETSCRATCHPADVALUE(); \
    CORE_TEST_NEG_SETSCRATCHPADVALUE()

/* ======================================================================== */
/*                      Test APIs: getScratchPadValue                       */
/* ======================================================================== */
#define CORE_TEST_NEG_GETSCRATCHPADVALUE() \
    PLATFORM_RUN_TEST(test_neg_core_getScratchPadValue_nullParam_pmicHandle); \
    PLATFORM_RUN_TEST(test_neg_core_getScratchPadValue_outOfBounds_scratchPadRegNum); \
    PLATFORM_RUN_TEST(test_neg_core_getScratchPadValue_nullParam_value)

/* Test: TC-CORE-0047 */
#define CORE_TEST_GETSCRATCHPADVALUE() \
    CORE_TEST_NEG_GETSCRATCHPADVALUE()

/* ======================================================================== */
/*                 API-Specific Test Macros - checkHandle                     */
/* ======================================================================== */
#define CORE_TEST_POS_CHECKHANDLE() \
    PLATFORM_RUN_TEST(test_pos_core_checkHandle_validCriticalSection)

#define CORE_TEST_NEG_CHECKHANDLE() \
    PLATFORM_RUN_TEST(test_neg_core_checkHandle_nullCritSecStart); \
    PLATFORM_RUN_TEST(test_neg_core_checkHandle_nullCritSecStop)

#define CORE_TEST_CHECKHANDLE() \
    CORE_TEST_POS_CHECKHANDLE(); \
    CORE_TEST_NEG_CHECKHANDLE()

/* ======================================================================== */
/*              Test APIs: init, deinit, ioRxByte                           */
/* ======================================================================== */
#define CORE_TEST_POS_SILICON() \
    PLATFORM_RUN_TEST(test_pos_core_init_A0_silicon_with_locked_registers); \
    PLATFORM_RUN_TEST(test_pos_core_init_B0_silicon_with_locked_registers); \
    PLATFORM_RUN_TEST(test_pos_core_init_B0_silicon_with_unlocked_registers)

/* Test: TC-CORE-0058 */
#define CORE_TEST_SILICON() \
    CORE_TEST_POS_SILICON()

/* ======================================================================== */
/*                      Test APIs: configCrcEnable                          */
/* ======================================================================== */
#define CORE_TEST_POS_CONFIGCRCENABLE() \
    PLATFORM_RUN_TEST(test_pos_core_configCrcEnable_enableOnly); \
    PLATFORM_RUN_TEST(test_pos_core_configCrcEnable_recalculate)

#define CORE_TEST_NEG_CONFIGCRCENABLE() \
    PLATFORM_RUN_TEST(test_neg_core_configCrcEnable_nullHandle)

/* Test: TC-CORE-0071 */
#define CORE_TEST_CONFIGCRCENABLE() \
    CORE_TEST_POS_CONFIGCRCENABLE(); \
    CORE_TEST_NEG_CONFIGCRCENABLE()

/* ======================================================================== */
/*                      Test APIs: configCrcDisable                         */
/* ======================================================================== */
#define CORE_TEST_POS_CONFIGCRCDISABLE() \
    PLATFORM_RUN_TEST(test_pos_core_configCrcDisable_disable)

#define CORE_TEST_NEG_CONFIGCRCDISABLE() \
    PLATFORM_RUN_TEST(test_neg_core_configCrcDisable_nullHandle)

/* Test: TC-CORE-0072 */
#define CORE_TEST_CONFIGCRCDISABLE() \
    CORE_TEST_POS_CONFIGCRCDISABLE(); \
    CORE_TEST_NEG_CONFIGCRCDISABLE()

/* ======================================================================== */
/*                     Test APIs: getConfigCrcStatus                        */
/* ======================================================================== */
#define CORE_TEST_POS_GETCONFIGCRCSTATUS() \
    PLATFORM_RUN_TEST(test_pos_core_getConfigCrcStatus_crcEnabled); \
    PLATFORM_RUN_TEST(test_pos_core_getConfigCrcStatus_crcDisabled)

#define CORE_TEST_NEG_GETCONFIGCRCSTATUS() \
    PLATFORM_RUN_TEST(test_neg_core_getConfigCrcStatus_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_core_getConfigCrcStatus_nullStatus)

/* Test: TC-CORE-0073 */
#define CORE_TEST_GETCONFIGCRCSTATUS() \
    CORE_TEST_POS_GETCONFIGCRCSTATUS(); \
    CORE_TEST_NEG_GETCONFIGCRCSTATUS()

/* ======================================================================== */
/*                     Test APIs: configCrcCalculate                        */
/* ======================================================================== */
#define CORE_TEST_POS_CONFIGCRCCALCULATE() \
    PLATFORM_RUN_TEST(test_pos_core_configCrcCalculate_calculate)

#define CORE_TEST_NEG_CONFIGCRCCALCULATE() \
    PLATFORM_RUN_TEST(test_neg_core_configCrcCalculate_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_core_configCrcCalculate_crcEnabled)

/* Test: TC-CORE-0074 */
#define CORE_TEST_CONFIGCRCCALCULATE() \
    CORE_TEST_POS_CONFIGCRCCALCULATE(); \
    CORE_TEST_NEG_CONFIGCRCCALCULATE()

/* ======================================================================== */
/*                        Test APIs: getConfigCrc                           */
/* ======================================================================== */
#define CORE_TEST_POS_GETCONFIGCRC() \
    PLATFORM_RUN_TEST(test_pos_core_getConfigCrc_readValue)

#define CORE_TEST_NEG_GETCONFIGCRC() \
    PLATFORM_RUN_TEST(test_neg_core_getConfigCrc_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_core_getConfigCrc_nullValue)

/* Test: TC-CORE-0075 */
#define CORE_TEST_GETCONFIGCRC() \
    CORE_TEST_POS_GETCONFIGCRC(); \
    CORE_TEST_NEG_GETCONFIGCRC()

/* ======================================================================== */
/*                        Test APIs: setConfigCrc                           */
/* ======================================================================== */
#define CORE_TEST_POS_SETCONFIGCRC() \
    PLATFORM_RUN_TEST(test_pos_core_setConfigCrc_writeValue)

#define CORE_TEST_NEG_SETCONFIGCRC() \
    PLATFORM_RUN_TEST(test_neg_core_setConfigCrc_nullHandle)

/* Test: TC-CORE-0076 */
#define CORE_TEST_SETCONFIGCRC() \
    CORE_TEST_POS_SETCONFIGCRC(); \
    CORE_TEST_NEG_SETCONFIGCRC()

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
    CORE_TEST_POS_CHECKHANDLE(); \
    CORE_TEST_POS_SILICON(); \
    CORE_TEST_POS_CONFIGCRCENABLE(); \
    CORE_TEST_POS_CONFIGCRCDISABLE(); \
    CORE_TEST_POS_GETCONFIGCRCSTATUS(); \
    CORE_TEST_POS_CONFIGCRCCALCULATE(); \
    CORE_TEST_POS_GETCONFIGCRC(); \
    CORE_TEST_POS_SETCONFIGCRC()

#define CORE_TEST_RUN_NEGATIVE() \
    CORE_TEST_NEG_GETNVMREV(); \
    CORE_TEST_NEG_GETSILICONREV(); \
    CORE_TEST_NEG_SETREGLOCKSTATE(); \
    CORE_TEST_NEG_GETREGLOCKSTATE(); \
    CORE_TEST_NEG_IOSETCRCENABLESTATE(); \
    CORE_TEST_NEG_IOGETCRCENABLESTATE(); \
    CORE_TEST_NEG_SETPWRON(); \
    CORE_TEST_NEG_GETPWRON(); \
    CORE_TEST_NEG_SETLPMCFG(); \
    CORE_TEST_NEG_GETLPMCFG(); \
    CORE_TEST_NEG_RUNABIST(); \
    CORE_TEST_NEG_GETABISTSTAT(); \
    CORE_TEST_NEG_SETSCRATCHPADVALUE(); \
    CORE_TEST_NEG_GETSCRATCHPADVALUE(); \
    CORE_TEST_NEG_CHECKHANDLE(); \
    CORE_TEST_NEG_CONFIGCRCENABLE(); \
    CORE_TEST_NEG_CONFIGCRCDISABLE(); \
    CORE_TEST_NEG_GETCONFIGCRCSTATUS(); \
    CORE_TEST_NEG_CONFIGCRCCALCULATE(); \
    CORE_TEST_NEG_GETCONFIGCRC(); \
    CORE_TEST_NEG_SETCONFIGCRC()

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

/* checkHandle API tests */
void test_neg_core_checkHandle_nullCritSecStart(void);
void test_neg_core_checkHandle_nullCritSecStop(void);
void test_pos_core_checkHandle_validCriticalSection(void);

/* Silicon revision tests */
void test_pos_core_init_A0_silicon_with_locked_registers(void);
void test_pos_core_init_B0_silicon_with_locked_registers(void);
void test_pos_core_init_B0_silicon_with_unlocked_registers(void);

/* configCrcEnable API tests */
void test_pos_core_configCrcEnable_enableOnly(void);
void test_pos_core_configCrcEnable_recalculate(void);
void test_neg_core_configCrcEnable_nullHandle(void);

/* configCrcDisable API tests */
void test_pos_core_configCrcDisable_disable(void);
void test_neg_core_configCrcDisable_nullHandle(void);

/* getConfigCrcStatus API tests */
void test_pos_core_getConfigCrcStatus_crcEnabled(void);
void test_pos_core_getConfigCrcStatus_crcDisabled(void);
void test_neg_core_getConfigCrcStatus_nullHandle(void);
void test_neg_core_getConfigCrcStatus_nullStatus(void);

/* configCrcCalculate API tests */
void test_pos_core_configCrcCalculate_calculate(void);
void test_neg_core_configCrcCalculate_nullHandle(void);
void test_neg_core_configCrcCalculate_crcEnabled(void);

/* getConfigCrc API tests */
void test_pos_core_getConfigCrc_readValue(void);
void test_neg_core_getConfigCrc_nullHandle(void);
void test_neg_core_getConfigCrc_nullValue(void);

/* setConfigCrc API tests */
void test_pos_core_setConfigCrc_writeValue(void);
void test_neg_core_setConfigCrc_nullHandle(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* PMIC_TEST_CORE_H */
