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
#ifndef PMIC_TEST_CORE_H
#define PMIC_TEST_CORE_H



/* ========================================================================== */
/*                              Include Files                                 */
/* ========================================================================== */

#include "test_common.h"
#include "pmic_fsm.h"
#include "regmap/core.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

void core_test(void *args);

void test_negative_Pmic_getNvmRev_nullParam_pmicHandle(void);
void test_negative_Pmic_getNvmRev_nullParam_nvmRev(void);
void test_negative_Pmic_getSiliconRev_nullParam_pmicHandle(void);
void test_negative_Pmic_getSiliconRev_nullParam_siliconRev(void);
void test_negative_Pmic_setRegLockState_nullParam_pmicHandle(void);
void test_negative_Pmic_enableRegLock_nullParam_pmicHandle(void);
void test_negative_Pmic_disableRegLock_nullParam_pmicHandle(void);
void test_negative_Pmic_getRegLockState_nullParam_pmicHandle(void);
void test_negative_Pmic_getRegLockState_nullParam_regLockStat(void);
void test_negative_Pmic_ioSetCrcEnableState_nullParam_pmicHandle(void);
void test_negative_Pmic_ioCrcEnable_nullParam_pmicHandle(void);
void test_negative_Pmic_ioCrcDisable_nullParam_pmicHandle(void);
void test_negative_Pmic_ioGetCrcEnableState_nullParam_pmicHandle(void);
void test_negative_Pmic_ioGetCrcEnableState_nullParam_crcEnabled(void);
void test_negative_Pmic_fsmSetDevState_nullParam_pmicHandle(void);
void test_negative_Pmic_fsmSetDevState_invalid_fsmCmd(void);
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
void test_negative_Pmic_setScratchPadValue_nullParam_pmicHandle(void);
void test_negative_Pmic_setScratchPadValue_outOfBounds_scratchPadRegNum(void);
void test_negative_Pmic_getScratchPadValue_nullParam_pmicHandle(void);
void test_negative_Pmic_getScratchPadValue_outOfBounds_scratchPadRegNum(void);
void test_negative_Pmic_getScratchPadValue_nullParam_value(void);
void test_negative_Pmic_fsmSetRecovCntThr_nullParam_pmicHandle(void);
void test_negative_Pmic_fsmSetRecovCntThr_outOfBounds_threshold(void);
void test_negative_Pmic_fsmGetRecovCntThr_nullParam_pmicHandle(void);
void test_negative_Pmic_fsmGetRecovCntThr_nullParam_threshold(void);
void test_negative_Pmic_fsmGetRecovCnt_nullParam_pmicHandle(void);
void test_negative_Pmic_fsmGetRecovCnt_nullParam_recovCnt(void);
void test_negative_Pmic_fsmClrRecovCnt_nullParam_pmicHandle(void);
void test_negative_Pmic_fsmSetResetCntThr_nullParam_pmicHandle(void);
void test_negative_Pmic_fsmSetResetCntThr_outOfBounds_threshold(void);
void test_negative_Pmic_fsmGetResetCntThr_nullParam_pmicHandle(void);
void test_negative_Pmic_fsmGetResetCntThr_nullParam_threshold(void);
void test_negative_Pmic_fsmGetResetCnt_nullParam_pmicHandle(void);
void test_negative_Pmic_fsmGetResetCnt_nullParam_resetCnt(void);
void test_negative_Pmic_fsmClrResetCnt_nullParam_pmicHandle(void);
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

/* CRC16 Negative Tests */
void test_negative_Pmic_setCRC16Cfg_nullParam_handle(void);
void test_negative_Pmic_setCRC16Cfg_nullParam_crc16Cfg(void);
void test_negative_Pmic_setCRC16Cfg_invalidParam_validParams(void);
void test_negative_Pmic_getCRC16Cfg_nullParam_handle(void);
void test_negative_Pmic_getCRC16Cfg_nullParam_crc16Cfg(void);
void test_negative_Pmic_getCRC16Cfg_invalidParam_validParams(void);

/* CRC16 Positive Tests */
void test_positive_setCRC16Cfg_enable(void);
void test_positive_getCRC16Cfg_enable(void);
void test_positive_setCRC16Cfg_activateCalc(void);
void test_positive_getCRC16Cfg_activateCalc(void);
void test_positive_setCRC16Cfg_combinedParams(void);
void test_positive_getCRC16Cfg_combinedParams(void);

/* LPM Get Tests */
void test_positive_getLpmCfg_pinDetection(void);
void test_positive_getLpmCfg_detectionDelay(void);
void test_positive_getLpmCfg_vmonEn(void);
void test_positive_getLpmCfg_esmEn(void);
void test_positive_getLpmCfg_wdgEn(void);
void test_positive_getLpmCfg_multipleParams(void);
void test_negative_Pmic_getLpmCfg_invalidParam_validParams(void);

/* ABIST Status Test */
void test_positive_getABISTStat_active(void);

/* Silicon Revision Tests */
void test_positive_silicon_A0_crc16_at_0x61(void);
void test_positive_silicon_B0_crc16_at_0x64(void);
void test_positive_silicon_B1_crc16_at_0x64(void);
void test_positive_init_A0_silicon_with_locked_registers(void);

/* Additional Error Path Tests */
void test_negative_Pmic_setCRC16Cfg_zeroValidParams(void);
void test_negative_Pmic_getCRC16Cfg_zeroValidParams(void);
void test_negative_Pmic_setLpmCfg_zeroValidParams(void);

/* Additional LPM Setter Tests */
void test_positive_setLpmCfg_pinDetection_allValues(void);
void test_positive_setLpmCfg_detectionDelay_allValues(void);
void test_positive_setLpmCfg_vmonEn_enable(void);
void test_positive_setLpmCfg_vmonEn_disable(void);
void test_positive_setLpmCfg_esmEn_enable(void);
void test_positive_setLpmCfg_esmEn_disable(void);
void test_positive_setLpmCfg_wdgEn_enable(void);
void test_positive_setLpmCfg_wdgEn_disable(void);
void test_positive_setLpmCfg_multipleEnables(void);
void test_positive_setLpmCfg_allParams(void);
void test_positive_setLpmCfg_pinDetection_boundaryMin(void);
void test_positive_setLpmCfg_pinDetection_boundaryMax(void);
void test_positive_setLpmCfg_detectionDelay_boundaryMin(void);
void test_positive_setLpmCfg_detectionDelay_boundaryMax(void);
void test_positive_setLpmCfg_pinDetectionAndDelay(void);
void test_negative_core_getLpmCfg_zeroValidParams(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* PMIC_TEST_CORE_H */
