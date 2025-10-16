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
#ifndef __CORE_TEST_H__
#define __CORE_TEST_H__

/**
 * @file core_test.h
 * @brief Contains macros/defines and test declarations specific to testing the
 * Core module.
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

void core_test(void *args);

void test_negative_Pmic_getDevId_nullParam_pmicHandle(void);
void test_negative_Pmic_getDevId_nullParam_devId(void);
void test_negative_Pmic_getNvmId_nullParam_pmicHandle(void);
void test_negative_Pmic_getNvmId_nullParam_nvmId(void);
void test_negative_Pmic_getNvmRev_nullParam_pmicHandle(void);
void test_negative_Pmic_getNvmRev_nullParam_nvmRev(void);
void test_negative_Pmic_getSiliconRev_nullParam_pmicHandle(void);
void test_negative_Pmic_getSiliconRev_nullParam_siliconRev(void);
void test_negative_Pmic_setRegLock_nullParam_pmicHandle(void);
void test_negative_Pmic_unlockRegs_nullParam_pmicHandle(void);
void test_negative_Pmic_lockRegs_nullParam_pmicHandle(void);
void test_negative_Pmic_getRegLock_nullParam_pmicHandle(void);
void test_negative_Pmic_getRegLock_nullParam_regLockStat(void);
void test_negative_Pmic_enableDisableCRC8_nullParam_pmicHandle(void);
void test_negative_Pmic_enableCRC8_nullParam_pmicHandle(void);
void test_negative_Pmic_disableCRC8_nullParam_pmicHandle(void);
void test_negative_Pmic_getCRC8Enable_nullParam_pmicHandle(void);
void test_negative_Pmic_getCRC8Enable_nullParam_crcEnabled(void);
void test_negative_Pmic_sendFsmCmd_nullParam_pmicHandle(void);
void test_negative_Pmic_sendFsmCmd_invalid_fsmCmd(void);
void test_negative_Pmic_setPwrOn_nullParam_pmicHandle(void);
void test_negative_Pmic_getPwrOn_nullParam_pmicHandle(void);
void test_negative_Pmic_getPwrOn_nullParam_pwrOnStat(void);
void test_negative_Pmic_setLpmCfg_nullParam_pmicHandle(void);
void test_negative_Pmic_setLpmCfg_nullParam_lpmCfg(void);
void test_negative_Pmic_setLpmCfg_outOfBounds_pinDetection(void);
void test_negative_Pmic_setLpmCfg_outOfBounds_detectionDelay(void);
void test_negative_Pmic_getLpmCfg_nullParam_pmicHandle(void);
void test_negative_Pmic_getLpmCfg_nullParam_lpmCfg(void);
void test_negative_Pmic_runABIST_nullParam_pmicHandle(void);
void test_negative_Pmic_getABISTStat_nullParam_pmicHandle(void);
void test_negative_Pmic_getABISTStat_nullParam_isActive(void);
void test_negative_Pmic_setScratchPadVal_nullParam_pmicHandle(void);
void test_negative_Pmic_setScratchPadVal_outOfBounds_scratchPadRegNum(void);
void test_negative_Pmic_getScratchPadVal_nullParam_pmicHandle(void);
void test_negative_Pmic_getScratchPadVal_outOfBounds_scratchPadRegNum(void);
void test_negative_Pmic_getScratchPadVal_nullParam_value(void);
void test_negative_Pmic_setRecovCntThr_nullParam_pmicHandle(void);
void test_negative_Pmic_setRecovCntThr_outOfBounds_threshold(void);
void test_negative_Pmic_getRecovCntThr_nullParam_pmicHandle(void);
void test_negative_Pmic_getRecovCntThr_nullParam_threshold(void);
void test_negative_Pmic_getRecovCnt_nullParam_pmicHandle(void);
void test_negative_Pmic_getRecovCnt_nullParam_recovCnt(void);
void test_negative_Pmic_clrRecovCnt_nullParam_pmicHandle(void);
void test_negative_Pmic_setResetCntThr_nullParam_pmicHandle(void);
void test_negative_Pmic_setResetCntThr_outOfBounds_threshold(void);
void test_negative_Pmic_getResetCntThr_nullParam_pmicHandle(void);
void test_negative_Pmic_getResetCntThr_nullParam_threshold(void);
void test_negative_Pmic_getResetCnt_nullParam_pmicHandle(void);
void test_negative_Pmic_getResetCnt_nullParam_resetCnt(void);
void test_negative_Pmic_clrResetCnt_nullParam_pmicHandle(void);
void test_positive_Pmic_getDevId(void);
void test_positive_Pmic_getNvmId(void);
void test_positive_Pmic_getNvmRev(void);
void test_positive_Pmic_getSiliconRev(void);
void test_positive_setGetRegLock(void);
void test_positive_enableDisableCRC8(void);
void test_positive_setGetPwrOn(void);
void test_positive_setGetLpmCfg_pinDetection(void);
void test_positive_setGetLpmCfg_detectionDelay(void);
void test_positive_setGetLpmCfg_vmonEn(void);
void test_positive_setGetLpmCfg_esmEn(void);
void test_positive_setGetLpmCfg_wdgEn(void);
void test_positive_Pmic_runABIST(void);
void test_positive_setGetScratchPadVal(void);
void test_positive_setGetRecovCntThr(void);
void test_positive_setGetResetCntThr(void);
void test_positive_getClrRecovCnt(void);
void test_positive_getClrResetCnt(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /*__CORE_TEST_H__*/
