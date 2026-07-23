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
/**
 * @file power_test.h
 *
 * @brief Header file for PMIC Power tests for TPS65036x-Q1.
 */
#ifndef POWER_TEST_H
#define POWER_TEST_H

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include "pmic.h"
#include "pmic_power.h"
#include "test_utils.h"
#include "platform.h"
#include "test_constants.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/* Mirrors the internal bounds defined in pmic_power.c; kept out of the public
 * pmic_power.h since they are implementation details, not part of the API. */
/** @brief Maximum sequence trigger bit position per power resource (bits 0-5) */
#define PMIC_PWR_SEQ_TRIG_BIT_POS_MAX       5U

/** @brief Maximum number of sequence trigger configurations that can be processed
 *  Derived: PMIC_POWER_RESOURCE_MAX (4) × 6 bits = 24 */
#define PMIC_PWR_SEQ_TRIG_LEN_MAX           (24U)

/** @brief Maximum number of sequence delay configurations that can be processed
 *  Same as trigger max: PMIC_POWER_RESOURCE_MAX (4) × 6 bits = 24 */
#define PMIC_PWR_SEQ_DLY_LEN_MAX            (24U)

/* ======================================================================== */
/*                         Test APIs: pwrSetLdoCfg                          */
/* ======================================================================== */
#define POWER_TEST_POS_PWRSETLDOCFG() \
    PLATFORM_RUN_TEST(test_pos_power_ldoSetGetCfg_enable); \
    PLATFORM_RUN_TEST(test_pos_power_ldoSetGetCfg_mode); \
    PLATFORM_RUN_TEST(test_pos_power_ldoSetGetCfg_vset); \
    PLATFORM_RUN_TEST(test_pos_power_ldoSetGetCfg_vmonOnly); \
    PLATFORM_RUN_TEST(test_pos_power_ldoSetGetCfg_dischargeEn); \
    PLATFORM_RUN_TEST(test_pos_power_ldoSetGetCfg_dischargeSel); \
    PLATFORM_RUN_TEST(test_pos_power_ldoSetGetCfg_deglitchSel); \
    PLATFORM_RUN_TEST(test_pos_power_ldoSetGetCfg_uvThr); \
    PLATFORM_RUN_TEST(test_pos_power_ldoSetGetCfg_ovThr); \
    PLATFORM_RUN_TEST(test_pos_power_ldoSetGetCfg_ilimSel); \
    PLATFORM_RUN_TEST(test_pos_power_ldoSetGetCfg_ovpSel); \
    PLATFORM_RUN_TEST(test_pos_power_ldoSetGetCfg_ovSel); \
    PLATFORM_RUN_TEST(test_pos_power_ldoSetGetCfg_uvSel); \
    PLATFORM_RUN_TEST(test_pos_power_ldoSetGetCfg_scSel); \
    PLATFORM_RUN_TEST(test_pos_power_ldoSetGetCfg_rvConf);

#define POWER_TEST_NEG_PWRSETLDOCFG() \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetLdoCfg_nullParam_pmicHandle); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetLdoCfg_nullParam_ldoCfg); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetLdoCfg_outOfBounds_mode); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetLdoCfg_outOfBounds_vset); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetLdoCfg_outOfBounds_dischargeSel); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetLdoCfg_outOfBounds_deglitchSel); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetLdoCfg_outOfBounds_uvThr); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetLdoCfg_outOfBounds_ovThr); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetLdoCfg_outOfBounds_ilimSel); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetLdoCfg_outOfBounds_ovpSel); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetLdoCfg_outOfBounds_ovSel); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetLdoCfg_outOfBounds_uvSel); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetLdoCfg_outOfBounds_scSel); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetLdoCfg_outOfBounds_rvConf); \
    PLATFORM_RUN_TEST(test_neg_power_pwr_setLdoCfg_zeroValidParams); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetLdoCfg_ldoConf_vsetOutOfBounds_ioPath); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetLdoCfg_ldoOvpResponse_ioRxByteFail); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetLdoCfg_ldoOvResponse_ioRxByteFail); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetLdoCfg_ldoUvResponse_ioRxByteFail); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetLdoCfg_ldoScResponse_ioRxByteFail); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetLdoCfg_setLdoConf_vsetBelowMin)

/* Test: TC-POWER-0032 */
#define POWER_TEST_PWRSETLDOCFG() \
    POWER_TEST_POS_PWRSETLDOCFG(); \
    POWER_TEST_NEG_PWRSETLDOCFG()

/* ======================================================================== */
/*                         Test APIs: pwrGetLdoCfg                          */
/* ======================================================================== */
#define POWER_TEST_POS_PWRGETLDOCFG()
    /* All positive tests covered by SET */

#define POWER_TEST_NEG_PWRGETLDOCFG() \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetLdoCfg_nullParam_pmicHandle); \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetLdoCfg_nullParam_ldoCfg); \
    PLATFORM_RUN_TEST(test_neg_power_pwr_getLdoCfg_zeroValidParams); \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetLdoCfg_ldoConf_ioRxByteCSFail); \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetLdoCfg_ldoCtrl_ioRxByteCSFail); \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetLdoCfg_ldoMonConf_ioRxByteCSFail); \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetLdoCfg_ldoOvpResponse_ioRxByteCSFail); \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetLdoCfg_ldoOvResponse_ioRxByteCSFail); \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetLdoCfg_ldoUvResponse_ioRxByteCSFail); \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetLdoCfg_ldoScResponse_ioRxByteCSFail)

/* Test: TC-POWER-0033 */
#define POWER_TEST_PWRGETLDOCFG() \
    POWER_TEST_POS_PWRGETLDOCFG(); \
    POWER_TEST_NEG_PWRGETLDOCFG()

/* ======================================================================== */
/*        Test APIs: pwrSetBuckCfg, pwrGetBuckCfg, pwrGetRsrcStatus         */
/* ======================================================================== */
#define POWER_TEST_POS_PWRSETBUCKCFG() \
    PLATFORM_RUN_TEST(test_pos_power_buckSetGetCfg_enable_buck1); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetGetCfg_enable_buck2); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetGetCfg_enable_buck3); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetGetCfg_disable_buck1); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetGetCfg_disable_buck2); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetGetCfg_disable_buck3); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetGetCfg_fpwmEn_enable_buck1); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetGetCfg_fpwmEn_enable_buck2); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetGetCfg_fpwmEn_enable_buck3); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetGetCfg_fpwmEn_disable_buck1); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetGetCfg_fpwmEn_disable_buck2); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetGetCfg_fpwmEn_disable_buck3); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetGetCfg_pldnEn_enable_buck1); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetGetCfg_pldnEn_enable_buck2); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetGetCfg_pldnEn_enable_buck3); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetGetCfg_pldnEn_disable_buck1); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetGetCfg_pldnEn_disable_buck2); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetGetCfg_pldnEn_disable_buck3); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetGetCfg_dischargeSel_buck1_active); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetGetCfg_dischargeSel_buck1_resistive); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetGetCfg_dischargeSel_buck2_active); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetGetCfg_dischargeSel_buck2_resistive); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetGetCfg_dischargeSel_buck3_active); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetGetCfg_dischargeSel_buck3_resistive); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetGetCfg_ssEn_enable_buck1); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetGetCfg_ssEn_enable_buck2); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetGetCfg_ssEn_enable_buck3); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetGetCfg_ssEn_disable_buck1); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetGetCfg_ssEn_disable_buck2); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetGetCfg_ssEn_disable_buck3)

#define POWER_TEST_NEG_PWRSETBUCKCFG() \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetBuckCfg_nullParam_pmicHandle); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetBuckCfg_nullParam_buckCfg); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetBuckCfg_invalidParam_resource); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetBuckCfg_zeroValidParams); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetBuckCfg_outOfBounds_vset); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetBuckCfg_outOfBounds_vsetActive); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetBuckCfg_outOfBounds_vsetLPwr); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetBuckCfg_outOfBounds_uvThr); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetBuckCfg_outOfBounds_ovThr); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetBuckCfg_outOfBounds_ilimSel_buck1); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetBuckCfg_outOfBounds_ilimSel_buck2); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetBuckCfg_outOfBounds_ovpSel); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetBuckCfg_outOfBounds_ovSel); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetBuckCfg_outOfBounds_uvSel); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetBuckCfg_outOfBounds_scSel); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetBuckCfg_outOfBounds_rvConf); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetBuckCfg_outOfBounds_slewRate); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetBuckCfg_outOfBounds_deglitchSel); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetBuckCfg_outOfBounds_dischargeSel); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetBuckCfg_outOfBounds_uvloRising); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetBuckCfg_outOfBounds_uvloFalling); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetBuckCfg_outOfBounds_highSideSlewRate); \
    PLATFORM_RUN_TEST(test_neg_power_buckSetCfg_ssmSel_outOfBounds); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetBuckCfg_buck1_vsetActiveNotSupported); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetBuckCfg_buck1_vsetLpwrNotSupported); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetBuckCfg_buck2_vsetNotSupported); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetBuckCfg_buck3_vsetNotSupported); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetBuckCfg_buck1_vmonOnlyNotSupported); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetBuckCfg_buck2_highSideSlewRateNotSupported); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetBuckCfg_buck3_highSideSlewRateNotSupported); \
    PLATFORM_RUN_TEST(test_neg_power_pwr_setBuck_invalidDischargeSel); \
    PLATFORM_RUN_TEST(test_neg_power_pwr_setBuck_invalidSlewRate); \
    PLATFORM_RUN_TEST(test_neg_power_pwr_setUvlo_invalidBuck); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetBuckCfg_buckActiveVSET_ioRxByteFail); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetBuckCfg_buckLPwrVSET_ioRxByteFail); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetBuckCfg_buckOvpResponse_ioRxByteFail); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetBuckCfg_buckOvResponse_ioRxByteFail); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetBuckCfg_buckUvResponse_ioRxByteFail); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetBuckCfg_buckScResponse_ioRxByteFail); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetBuckCfg_buck2_3Ctrl_ioRxByteFail); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetBuckCfg_setBuckActiveVSET_belowMin); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetBuckCfg_setBuckLPwrVSET_belowMin)

#define POWER_TEST_POS_PWRSETBUCKCFG_EXTENDED() \
    PLATFORM_RUN_TEST(test_pos_power_buckSetGetCfg_ilimSel_buck1_allValues); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetGetCfg_ilimSel_buck2_allValues); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetGetCfg_ilimSel_buck3_allValues); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetGetCfg_ilimSel_buck1_min); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetGetCfg_ilimSel_buck1_max); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetGetCfg_ilimSel_buck2_min); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetGetCfg_ilimSel_buck2_max); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetGetCfg_ilimSel_buck3_min); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetGetCfg_ilimSel_buck3_max); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetGetCfg_ovpSel_buck1_ignore); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetGetCfg_ovpSel_buck1_assertNint); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetGetCfg_ovpSel_buck2_waitPwrCycle); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetGetCfg_ovpSel_buck3_ordShutdown); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetGetCfg_ovpSel_buck1_immShutdown); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetGetCfg_ovpSel_buck2_allResponses); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetGetCfg_ovSel_buck1_ignore); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetGetCfg_ovSel_buck2_assertNint); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetGetCfg_ovSel_buck3_waitPwrCycle); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetGetCfg_ovSel_buck1_allResponses); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetGetCfg_uvSel_buck1_ignore); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetGetCfg_uvSel_buck2_assertNint); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetGetCfg_uvSel_buck3_allResponses); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetGetCfg_uvSel_buck1_waitPwrCycle); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetGetCfg_scSel_buck1_ignore); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetGetCfg_scSel_buck2_assertNint); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetGetCfg_scSel_buck3_allResponses); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetGetCfg_scSel_buck1_ordShutdown); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetGetCfg_rvConf_buck1_wait); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetGetCfg_rvConf_buck2_ignore); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetGetCfg_rvConf_buck3_allValues); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetGetCfg_multiParam_buck1_enableVsetPldnFpwm); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetGetCfg_multiParam_buck2_enableVsetActiveThresholds); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetGetCfg_multiParam_buck3_allCommonParams); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetGetCfg_multiParam_buck1_allFaultResponses); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetGetCfg_multiParam_buck2_fullConfig); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetGetCfg_vset_buck1_min); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetGetCfg_vset_buck1_mid); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetGetCfg_vset_buck1_max); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetGetCfg_vset_buck1_boundary_low); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetGetCfg_vset_buck1_boundary_high); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetGetCfg_vsetActive_buck2_min); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetGetCfg_vsetActive_buck2_mid); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetGetCfg_vsetActive_buck2_max); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetGetCfg_vsetActive_buck3_min); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetGetCfg_vsetActive_buck3_mid); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetGetCfg_vsetActive_buck3_max); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetGetCfg_vsetActive_buck2_boundary); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetGetCfg_vsetActive_buck3_boundary); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetGetCfg_vsetLPwr_buck2_min); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetGetCfg_vsetLPwr_buck2_max); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetGetCfg_vsetLPwr_buck3_min); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetGetCfg_vsetLPwr_buck3_max); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetGetCfg_vsetLPwr_buck2_mid); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetGetCfg_vsetLPwr_buck3_mid); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetGetCfg_vmonOnly_buck2_enable); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetGetCfg_vmonOnly_buck3_enable); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetGetCfg_uvThr_buck1_val0); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetGetCfg_uvThr_buck1_val1); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetGetCfg_uvThr_buck2_val2); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetGetCfg_uvThr_buck2_val3); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetGetCfg_uvThr_buck3_val0); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetGetCfg_uvThr_buck3_val3); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetGetCfg_ovThr_buck1_val0); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetGetCfg_ovThr_buck1_val1); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetGetCfg_ovThr_buck2_val2); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetGetCfg_ovThr_buck2_val3); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetGetCfg_ovThr_buck3_val0); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetGetCfg_ovThr_buck3_val3); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetGetCfg_slewRate_buck1_allValues); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetGetCfg_slewRate_buck2_minMax); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetGetCfg_slewRate_buck3_minMax); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetGetCfg_highSideSlewRate_buck1_fast); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetGetCfg_highSideSlewRate_buck1_slow); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetGetCfg_highSideSlewRate_buck1_slowest); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetGetCfg_deglitchSel_allBucks); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetGetCfg_ssmSel_buck1); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetGetCfg_ssmSel_buck2); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetGetCfg_ssmSel_buck3); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetGetCfg_ssmSel_allValues); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetGetCfg_uvloRising_buck1_allValues); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetGetCfg_uvloFalling_buck1_allValues); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetGetCfg_uvloRisingAndFalling_buck1); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetCfg_buck2OvpSel); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetCfg_buck3OvpSel); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetCfg_buck2OvSel); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetCfg_buck3OvSel); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetCfg_buck2UvSel); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetCfg_buck3UvSel); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetCfg_buck2ScSel); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetCfg_buck3ScSel); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetCfg_buck1RvConf); \
    PLATFORM_RUN_TEST(test_pos_power_buckSetCfg_buck1IlimSel); \
    PLATFORM_RUN_TEST(test_pos_power_powerGetCfg_buck2Enable); \
    PLATFORM_RUN_TEST(test_pos_power_powerSetCfg_buck2Enable); \
    PLATFORM_RUN_TEST(test_pos_power_powerGetVoutCfg_buck2Voltage); \
    PLATFORM_RUN_TEST(test_pos_power_powerSetVoutCfg_buck2Voltage); \
    PLATFORM_RUN_TEST(test_pos_power_powerGetCfg_buck3Enable); \
    PLATFORM_RUN_TEST(test_pos_power_powerSetCfg_buck3Enable); \
    PLATFORM_RUN_TEST(test_pos_power_powerGetVoutCfg_buck3Voltage); \
    PLATFORM_RUN_TEST(test_pos_power_powerSetVoutCfg_buck3Voltage); \
    PLATFORM_RUN_TEST(test_pos_power_powerGetStat_buck2); \
    PLATFORM_RUN_TEST(test_pos_power_powerGetStat_buck3)

/* Test: TC-POWER-0034 */
#define POWER_TEST_PWRSETBUCKCFG() \
    POWER_TEST_POS_PWRSETBUCKCFG(); \
    POWER_TEST_NEG_PWRSETBUCKCFG(); \
    POWER_TEST_POS_PWRSETBUCKCFG_EXTENDED()

/* ======================================================================== */
/*                         Test APIs: pwrGetBuckCfg                         */
/* ======================================================================== */

#define POWER_TEST_POS_PWRGETBUCKCFG() \
    PLATFORM_RUN_TEST(test_pos_power_buckGetCfg_ovpSel_buck1); \
    PLATFORM_RUN_TEST(test_pos_power_buckGetCfg_ovpSel_buck2); \
    PLATFORM_RUN_TEST(test_pos_power_buckGetCfg_ovpSel_buck3); \
    PLATFORM_RUN_TEST(test_pos_power_buckGetCfg_ovSel_buck1); \
    PLATFORM_RUN_TEST(test_pos_power_buckGetCfg_ovSel_buck2); \
    PLATFORM_RUN_TEST(test_pos_power_buckGetCfg_ovSel_buck3); \
    PLATFORM_RUN_TEST(test_pos_power_buckGetCfg_uvSel_buck1); \
    PLATFORM_RUN_TEST(test_pos_power_buckGetCfg_uvSel_buck2); \
    PLATFORM_RUN_TEST(test_pos_power_buckGetCfg_uvSel_buck3); \
    PLATFORM_RUN_TEST(test_pos_power_buckGetCfg_scSel_buck1); \
    PLATFORM_RUN_TEST(test_pos_power_buckGetCfg_scSel_buck2); \
    PLATFORM_RUN_TEST(test_pos_power_buckGetCfg_scSel_buck3); \
    PLATFORM_RUN_TEST(test_pos_power_buckGetCfg_rvConf_buck1); \
    PLATFORM_RUN_TEST(test_pos_power_buckGetCfg_rvConf_buck2); \
    PLATFORM_RUN_TEST(test_pos_power_buckGetCfg_rvConf_buck3); \
    PLATFORM_RUN_TEST(test_pos_power_buckGetCfg_ilimSel_buck1); \
    PLATFORM_RUN_TEST(test_pos_power_buckGetCfg_ilimSel_buck2); \
    PLATFORM_RUN_TEST(test_pos_power_buckGetCfg_ilimSel_buck3)

#define POWER_TEST_NEG_PWRGETBUCKCFG() \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetBuckCfg_nullParam_buckCfg); \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetBuckCfg_invalidParam_resource); \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetBuckCfg_zeroValidParams); \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetBuckCfg_buck1_vsetActiveNotSupported); \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetBuckCfg_buck1_vsetLpwrNotSupported); \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetBuckCfg_buck2_vsetNotSupported); \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetBuckCfg_buck3_vsetNotSupported); \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetBuckCfg_buck1_vmonOnlyNotSupported); \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetBuckCfg_buck2_highSideSlewRateNotSupported); \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetBuckCfg_buck3_highSideSlewRateNotSupported); \
    PLATFORM_RUN_TEST(test_neg_power_pwr_getUvlo_invalidBuck); \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetBuckCfg_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetBuckCfg_spreadSpectrum_ioRxByteCSFail); \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetBuckCfg_buckVSET_ioRxByteCSFail); \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetBuckCfg_buckVsetActive_ioRxByteCSFail); \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetBuckCfg_buckVsetLPwr_ioRxByteCSFail); \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetBuckCfg_buck1Ctrl_ioRxByteCSFail); \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetBuckCfg_buck2_3Ctrl_ioRxByteCSFail); \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetBuckCfg_buckMonConf_ioRxByteCSFail); \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetBuckCfg_buckOvpResponse_ioRxByteCSFail); \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetBuckCfg_buckOvResponse_ioRxByteCSFail); \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetBuckCfg_buckUvResponse_ioRxByteCSFail); \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetBuckCfg_buckScResponse_ioRxByteCSFail)

/* Test: TC-POWER-0035 */
#define POWER_TEST_PWRGETBUCKCFG() \
    POWER_TEST_POS_PWRGETBUCKCFG(); \
    POWER_TEST_NEG_PWRGETBUCKCFG()

/* ======================================================================== */
/*        Test APIs: pwrSetTsdCfg, pwrGetTsdCfg, pwrGetTsdImmStatus         */
/* ======================================================================== */
#define POWER_TEST_POS_PWRSETTSDCFG() \
    PLATFORM_RUN_TEST(test_pos_power_tsdSetGetCfg_twarnStayInSafeState); \
    PLATFORM_RUN_TEST(test_pos_power_tsdSetGetCfg_tsdImmLevel); \
    PLATFORM_RUN_TEST(test_pos_power_tsdSetGetCfg_twarnLevel); \
    PLATFORM_RUN_TEST(test_pos_power_tsdGetImmStatus)

#define POWER_TEST_NEG_PWRSETTSDCFG() \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetTsdCfg_nullParam_pmicHandle); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetTsdCfg_nullParam_tsdCfg); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetTsdCfg_outOfBounds_tsdImmLevel); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetTsdCfg_outOfBounds_twarnLevel); \
    PLATFORM_RUN_TEST(test_neg_power_pwr_setTsdCfg_zeroValidParams)

/* Test: TC-POWER-0036 */
#define POWER_TEST_PWRSETTSDCFG() \
    POWER_TEST_POS_PWRSETTSDCFG(); \
    POWER_TEST_NEG_PWRSETTSDCFG()

/* ======================================================================== */
/*               Test APIs: pwrGetTsdCfg, pwrGetTsdImmStatus                */
/* ======================================================================== */
#define POWER_TEST_POS_PWRGETTSDCFG() /* All positive tests covered by SET */

#define POWER_TEST_NEG_PWRGETTSDCFG() \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetTsdCfg_nullParam_pmicHandle); \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetTsdCfg_nullParam_tsdCfg); \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetTsdImmStatus_nullParam_pmicHandle); \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetTsdImmStatus_nullParam_tsdImmStat); \
    PLATFORM_RUN_TEST(test_neg_power_pwr_getTsdCfg_zeroValidParams)

/* Test: TC-POWER-0037 */
#define POWER_TEST_PWRGETTSDCFG() \
    POWER_TEST_POS_PWRGETTSDCFG(); \
    POWER_TEST_NEG_PWRGETTSDCFG()

/* ======================================================================== */
/*                       Test APIs: pwrGetRsrcStatus                        */
/* ======================================================================== */
#define POWER_TEST_POS_PWRGETRSRCSTATUS() \
    PLATFORM_RUN_TEST(test_pos_power_rsrcGetStatus_buck1); \
    PLATFORM_RUN_TEST(test_pos_power_rsrcGetStatus_buck2); \
    PLATFORM_RUN_TEST(test_pos_power_rsrcGetStatus_buck3); \
    PLATFORM_RUN_TEST(test_pos_power_rsrcGetStatus_ldo)

#define POWER_TEST_NEG_PWRGETRSRCSTATUS() \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetRsrcStatus_nullParam_pmicHandle); \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetRsrcStatus_nullParam_pwrRsrcStat); \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetRsrcStatus_outOfBounds_resource); \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetRsrcStatus_statStartup_ioRxByteCSFail); \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetRsrcStatus_statBuck1_2_ioRxByteCSFail); \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetRsrcStatus_statBuck3Ldo_ioRxByteCSFail)

/* Test: TC-POWER-0038 */
#define POWER_TEST_PWRGETRSRCSTATUS() \
    POWER_TEST_POS_PWRGETRSRCSTATUS(); \
    POWER_TEST_NEG_PWRGETRSRCSTATUS()

/* ======================================================================== */
/*                     Test APIs: pwrSetBuckLdoSeqTrig                      */
/* ======================================================================== */
#define POWER_TEST_POS_PWRSETBUCKLDOSEQTRIG() \
    PLATFORM_RUN_TEST(test_pos_power_seqTrigSetGet_buck1); \
    PLATFORM_RUN_TEST(test_pos_power_seqTrigSetGet_buck2); \
    PLATFORM_RUN_TEST(test_pos_power_seqTrigSetGet_buck3); \
    PLATFORM_RUN_TEST(test_pos_power_seqTrigSetGet_ldo); \
    PLATFORM_RUN_TEST(test_pos_power_seqTrigSetGet_allResources)

#define POWER_TEST_NEG_PWRSETBUCKLDOSEQTRIG() \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetBuckLdoSeqTrig_nullParam_pmicHandle); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetBuckLdoSeqTrig_nullParam_seqTrigCfg); \
    PLATFORM_RUN_TEST(test_neg_power_pwr_setSeqTrig_zeroLen); \
    PLATFORM_RUN_TEST(test_neg_power_pwr_setSeqTrig_lenExceedsMax); \
    PLATFORM_RUN_TEST(test_neg_power_pwr_setSeqTrig_invalidPwrRsrc); \
    PLATFORM_RUN_TEST(test_neg_power_pwr_setSeqTrig_invalidBitPos); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetBuckLdoSeqTrig_ioFailure); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetBuckLdoSeqTrig_lenMax_ioFailure)

/* Test: TC-POWER-0039 */
#define POWER_TEST_PWRSETBUCKLDOSEQTRIG() \
    POWER_TEST_POS_PWRSETBUCKLDOSEQTRIG(); \
    POWER_TEST_NEG_PWRSETBUCKLDOSEQTRIG()

/* ======================================================================== */
/*                     Test APIs: pwrGetBuckLdoSeqTrig                      */
/* ======================================================================== */

#define POWER_TEST_POS_PWRGETBUCKLDOSEQTRIG()  /* All positive tests covered by SET */

#define POWER_TEST_NEG_PWRGETBUCKLDOSEQTRIG() \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetBuckLdoSeqTrig_nullParam_pmicHandle); \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetBuckLdoSeqTrig_nullParam_seqTrigCfg); \
    PLATFORM_RUN_TEST(test_neg_power_pwr_getSeqTrig_zeroLen); \
    PLATFORM_RUN_TEST(test_neg_power_pwr_getSeqTrig_lenExceedsMax); \
    PLATFORM_RUN_TEST(test_neg_power_pwr_getSeqTrig_invalidResource); \
    PLATFORM_RUN_TEST(test_neg_power_pwr_getSeqTrig_invalidBitPos); \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetBuckLdoSeqTrig_ioFailure)

/* Test: TC-POWER-0040 */
#define POWER_TEST_PWRGETBUCKLDOSEQTRIG() \
    POWER_TEST_POS_PWRGETBUCKLDOSEQTRIG(); \
    POWER_TEST_NEG_PWRGETBUCKLDOSEQTRIG()

/* ======================================================================== */
/*                      Test APIs: pwrSetBuckLdoSeqDly                      */
/* ======================================================================== */
#define POWER_TEST_POS_PWRSETBUCKLDOSEQDLY() \
    PLATFORM_RUN_TEST(test_pos_power_seqDlySetGet_buck1); \
    PLATFORM_RUN_TEST(test_pos_power_seqDlySetGet_buck2); \
    PLATFORM_RUN_TEST(test_pos_power_seqDlySetGet_buck3); \
    PLATFORM_RUN_TEST(test_pos_power_seqDlySetGet_ldo); \
    PLATFORM_RUN_TEST(test_pos_power_seqDlySetGet_allResources)

#define POWER_TEST_NEG_PWRSETBUCKLDOSEQDLY() \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetBuckLdoSeqDly_nullParam_pmicHandle); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetBuckLdoSeqDly_nullParam_seqDlyCfg); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetBuckLdoSeqDly_outOfBounds_seqDlyOn); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetBuckLdoSeqDly_outOfBounds_seqDlyOff); \
    PLATFORM_RUN_TEST(test_neg_power_pwr_setSeqDelay_zeroLen); \
    PLATFORM_RUN_TEST(test_neg_power_pwr_setSeqDelay_lenExceedsMax); \
    PLATFORM_RUN_TEST(test_neg_power_pwr_setSeqDelay_invalidConfig); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetBuckLdoSeqDly_zeroValidParams); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetBuckLdoSeqDly_invalidResource)

/* Test: TC-POWER-0041 */
#define POWER_TEST_PWRSETBUCKLDOSEQDLY() \
    POWER_TEST_POS_PWRSETBUCKLDOSEQDLY(); \
    POWER_TEST_NEG_PWRSETBUCKLDOSEQDLY()

/* ======================================================================== */
/*                      Test APIs: pwrGetBuckLdoSeqDly                      */
/* ======================================================================== */

#define POWER_TEST_POS_PWRGETBUCKLDOSEQDLY() \
    PLATFORM_RUN_TEST(test_pos_power_pwrGetBuckLdoSeqDly_onlyOffDelay); \
    PLATFORM_RUN_TEST(test_pos_power_pwrGetBuckLdoSeqDly_onlyOnDelay)

#define POWER_TEST_NEG_PWRGETBUCKLDOSEQDLY() \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetBuckLdoSeqDly_nullParam_pmicHandle); \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetBuckLdoSeqDly_nullParam_seqDlyCfg); \
    PLATFORM_RUN_TEST(test_neg_power_pwr_getSeqDelay_zeroLen); \
    PLATFORM_RUN_TEST(test_neg_power_pwr_getSeqDelay_lenExceedsMax); \
    PLATFORM_RUN_TEST(test_neg_power_pwr_getSeqDelay_invalidConfig); \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetBuckLdoSeqDly_zeroValidParams); \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetBuckLdoSeqDly_ioFailure); \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetBuckLdoSeqDly_lenMax_ioFailure)

/* Test: TC-POWER-0042 */
#define POWER_TEST_PWRGETBUCKLDOSEQDLY() \
    POWER_TEST_POS_PWRGETBUCKLDOSEQDLY(); \
    POWER_TEST_NEG_PWRGETBUCKLDOSEQDLY()

/* ========================================================================== */
/*                        Aggregate Test Macros                               */
/* ========================================================================== */

#define POWER_TEST_RUN_POSITIVE() \
    POWER_TEST_POS_PWRSETLDOCFG(); \
    POWER_TEST_POS_PWRGETLDOCFG(); \
    POWER_TEST_POS_PWRSETBUCKCFG(); \
    POWER_TEST_POS_PWRGETBUCKCFG(); \
    POWER_TEST_POS_PWRSETBUCKCFG_EXTENDED(); \
    POWER_TEST_POS_PWRSETTSDCFG(); \
    POWER_TEST_POS_PWRGETTSDCFG(); \
    POWER_TEST_POS_PWRGETRSRCSTATUS(); \
    POWER_TEST_POS_PWRSETBUCKLDOSEQTRIG(); \
    POWER_TEST_POS_PWRGETBUCKLDOSEQTRIG(); \
    POWER_TEST_POS_PWRSETBUCKLDOSEQDLY(); \
    POWER_TEST_POS_PWRGETBUCKLDOSEQDLY()

#define POWER_TEST_RUN_NEGATIVE() \
    POWER_TEST_NEG_PWRSETLDOCFG(); \
    POWER_TEST_NEG_PWRGETLDOCFG(); \
    POWER_TEST_NEG_PWRSETBUCKCFG(); \
    POWER_TEST_NEG_PWRGETBUCKCFG(); \
    POWER_TEST_NEG_PWRSETTSDCFG(); \
    POWER_TEST_NEG_PWRGETTSDCFG(); \
    POWER_TEST_NEG_PWRGETRSRCSTATUS(); \
    POWER_TEST_NEG_PWRSETBUCKLDOSEQTRIG(); \
    POWER_TEST_NEG_PWRGETBUCKLDOSEQTRIG(); \
    POWER_TEST_NEG_PWRSETBUCKLDOSEQDLY(); \
    POWER_TEST_NEG_PWRGETBUCKLDOSEQDLY()

#define POWER_TEST_RUN_ALL() \
    POWER_TEST_RUN_POSITIVE(); \
    POWER_TEST_RUN_NEGATIVE()

/* ========================================================================== */
/*                           Function Declarations                            */
/* ========================================================================== */
void power_test(void *args);

/* LDO Configuration Tests - Negative */
void test_neg_power_pwrSetLdoCfg_nullParam_pmicHandle(void);
void test_neg_power_pwrSetLdoCfg_nullParam_ldoCfg(void);
void test_neg_power_pwrSetLdoCfg_outOfBounds_mode(void);
void test_neg_power_pwrSetLdoCfg_outOfBounds_vset(void);
void test_neg_power_pwrSetLdoCfg_outOfBounds_dischargeSel(void);
void test_neg_power_pwrSetLdoCfg_outOfBounds_deglitchSel(void);
void test_neg_power_pwrSetLdoCfg_outOfBounds_uvThr(void);
void test_neg_power_pwrSetLdoCfg_outOfBounds_ovThr(void);
void test_neg_power_pwrSetLdoCfg_outOfBounds_ilimSel(void);
void test_neg_power_pwrSetLdoCfg_outOfBounds_ovpSel(void);
void test_neg_power_pwrSetLdoCfg_outOfBounds_ovSel(void);
void test_neg_power_pwrSetLdoCfg_outOfBounds_uvSel(void);
void test_neg_power_pwrSetLdoCfg_outOfBounds_scSel(void);
void test_neg_power_pwrSetLdoCfg_outOfBounds_rvConf(void);
void test_neg_power_pwrGetLdoCfg_nullParam_pmicHandle(void);
void test_neg_power_pwrGetLdoCfg_nullParam_ldoCfg(void);

/* LDO Configuration Tests - Positive */
void test_pos_power_ldoSetGetCfg_enable(void);
void test_pos_power_ldoSetGetCfg_mode(void);
void test_pos_power_ldoSetGetCfg_vset(void);
void test_pos_power_ldoSetGetCfg_vmonOnly(void);
void test_pos_power_ldoSetGetCfg_dischargeEn(void);
void test_pos_power_ldoSetGetCfg_dischargeSel(void);
void test_pos_power_ldoSetGetCfg_deglitchSel(void);
void test_pos_power_ldoSetGetCfg_uvThr(void);
void test_pos_power_ldoSetGetCfg_ovThr(void);
void test_pos_power_ldoSetGetCfg_ilimSel(void);
void test_pos_power_ldoSetGetCfg_ovpSel(void);
void test_pos_power_ldoSetGetCfg_ovSel(void);
void test_pos_power_ldoSetGetCfg_uvSel(void);
void test_pos_power_ldoSetGetCfg_scSel(void);
void test_pos_power_ldoSetGetCfg_rvConf(void);

/* TSD Configuration Tests - Negative */
void test_neg_power_pwrSetTsdCfg_nullParam_pmicHandle(void);
void test_neg_power_pwrSetTsdCfg_nullParam_tsdCfg(void);
void test_neg_power_pwrSetTsdCfg_outOfBounds_tsdImmLevel(void);
void test_neg_power_pwrSetTsdCfg_outOfBounds_twarnLevel(void);
void test_neg_power_pwrGetTsdCfg_nullParam_pmicHandle(void);
void test_neg_power_pwrGetTsdCfg_nullParam_tsdCfg(void);
void test_neg_power_pwrGetTsdImmStatus_nullParam_pmicHandle(void);
void test_neg_power_pwrGetTsdImmStatus_nullParam_tsdImmStat(void);

/* TSD Configuration Tests - Positive */
void test_pos_power_tsdSetGetCfg_twarnStayInSafeState(void);
void test_pos_power_tsdSetGetCfg_tsdImmLevel(void);
void test_pos_power_tsdSetGetCfg_twarnLevel(void);
void test_pos_power_tsdGetImmStatus(void);

/* Resource Status Tests - Negative */
void test_neg_power_pwrGetRsrcStatus_nullParam_pmicHandle(void);
void test_neg_power_pwrGetRsrcStatus_nullParam_pwrRsrcStat(void);
void test_neg_power_pwrGetRsrcStatus_outOfBounds_resource(void);

/* Resource Status Tests - Positive */
void test_pos_power_rsrcGetStatus_buck1(void);
void test_pos_power_rsrcGetStatus_buck2(void);
void test_pos_power_rsrcGetStatus_buck3(void);
void test_pos_power_rsrcGetStatus_ldo(void);

/* Sequencing Tests - Negative */
void test_neg_power_pwrSetBuckLdoSeqTrig_nullParam_pmicHandle(void);
void test_neg_power_pwrSetBuckLdoSeqTrig_nullParam_seqTrigCfg(void);
void test_neg_power_pwrGetBuckLdoSeqTrig_nullParam_pmicHandle(void);
void test_neg_power_pwrGetBuckLdoSeqTrig_nullParam_seqTrigCfg(void);
void test_neg_power_pwrSetBuckLdoSeqDly_nullParam_pmicHandle(void);
void test_neg_power_pwrSetBuckLdoSeqDly_nullParam_seqDlyCfg(void);
void test_neg_power_pwrSetBuckLdoSeqDly_outOfBounds_seqDlyOn(void);
void test_neg_power_pwrSetBuckLdoSeqDly_outOfBounds_seqDlyOff(void);
void test_neg_power_pwrGetBuckLdoSeqDly_nullParam_pmicHandle(void);
void test_neg_power_pwrGetBuckLdoSeqDly_nullParam_seqDlyCfg(void);

/* Sequencing Tests - Positive */
void test_pos_power_seqTrigSetGet_buck1(void);
void test_pos_power_seqTrigSetGet_buck2(void);
void test_pos_power_seqTrigSetGet_buck3(void);
void test_pos_power_seqTrigSetGet_ldo(void);
void test_pos_power_seqTrigSetGet_allResources(void);
void test_pos_power_seqDlySetGet_buck1(void);
void test_pos_power_seqDlySetGet_buck2(void);
void test_pos_power_seqDlySetGet_buck3(void);
void test_pos_power_seqDlySetGet_ldo(void);
void test_pos_power_seqDlySetGet_allResources(void);

/* Buck Configuration Tests - Positive */
void test_pos_power_buckSetGetCfg_enable_buck1(void);
void test_pos_power_buckSetGetCfg_enable_buck2(void);
void test_pos_power_buckSetGetCfg_enable_buck3(void);
void test_pos_power_buckSetGetCfg_disable_buck1(void);
void test_pos_power_buckSetGetCfg_disable_buck2(void);
void test_pos_power_buckSetGetCfg_disable_buck3(void);
void test_pos_power_buckSetGetCfg_fpwmEn_enable_buck1(void);
void test_pos_power_buckSetGetCfg_fpwmEn_enable_buck2(void);
void test_pos_power_buckSetGetCfg_fpwmEn_enable_buck3(void);
void test_pos_power_buckSetGetCfg_fpwmEn_disable_buck1(void);
void test_pos_power_buckSetGetCfg_fpwmEn_disable_buck2(void);
void test_pos_power_buckSetGetCfg_fpwmEn_disable_buck3(void);
void test_pos_power_buckSetGetCfg_pldnEn_enable_buck1(void);
void test_pos_power_buckSetGetCfg_pldnEn_enable_buck2(void);
void test_pos_power_buckSetGetCfg_pldnEn_enable_buck3(void);
void test_pos_power_buckSetGetCfg_pldnEn_disable_buck1(void);
void test_pos_power_buckSetGetCfg_pldnEn_disable_buck2(void);
void test_pos_power_buckSetGetCfg_pldnEn_disable_buck3(void);
void test_pos_power_buckSetGetCfg_dischargeSel_buck1_active(void);
void test_pos_power_buckSetGetCfg_dischargeSel_buck1_resistive(void);
void test_pos_power_buckSetGetCfg_dischargeSel_buck2_active(void);
void test_pos_power_buckSetGetCfg_dischargeSel_buck2_resistive(void);
void test_pos_power_buckSetGetCfg_dischargeSel_buck3_active(void);
void test_pos_power_buckSetGetCfg_dischargeSel_buck3_resistive(void);
void test_pos_power_buckSetGetCfg_ssEn_enable_buck1(void);
void test_pos_power_buckSetGetCfg_ssEn_enable_buck2(void);
void test_pos_power_buckSetGetCfg_ssEn_enable_buck3(void);
void test_pos_power_buckSetGetCfg_ssEn_disable_buck1(void);
void test_pos_power_buckSetGetCfg_ssEn_disable_buck2(void);
void test_pos_power_buckSetGetCfg_ssEn_disable_buck3(void);

/* Buck Configuration GET-only Tests */
void test_pos_power_buckGetCfg_ovpSel_buck1(void);
void test_pos_power_buckGetCfg_ovpSel_buck2(void);
void test_pos_power_buckGetCfg_ovpSel_buck3(void);
void test_pos_power_buckGetCfg_ovSel_buck1(void);
void test_pos_power_buckGetCfg_ovSel_buck2(void);
void test_pos_power_buckGetCfg_ovSel_buck3(void);
void test_pos_power_buckGetCfg_uvSel_buck1(void);
void test_pos_power_buckGetCfg_uvSel_buck2(void);
void test_pos_power_buckGetCfg_uvSel_buck3(void);
void test_pos_power_buckGetCfg_scSel_buck1(void);
void test_pos_power_buckGetCfg_scSel_buck2(void);
void test_pos_power_buckGetCfg_scSel_buck3(void);
void test_pos_power_buckGetCfg_rvConf_buck1(void);
void test_pos_power_buckGetCfg_rvConf_buck2(void);
void test_pos_power_buckGetCfg_rvConf_buck3(void);
void test_pos_power_buckGetCfg_ilimSel_buck1(void);
void test_pos_power_buckGetCfg_ilimSel_buck2(void);
void test_pos_power_buckGetCfg_ilimSel_buck3(void);

/* Buck Configuration Tests - Negative */
void test_neg_power_pwrSetBuckCfg_nullParam_pmicHandle(void);
void test_neg_power_pwrSetBuckCfg_nullParam_buckCfg(void);
void test_neg_power_pwrGetBuckCfg_nullParam_buckCfg(void);
void test_neg_power_pwrSetBuckCfg_invalidParam_resource(void);
void test_neg_power_pwrGetBuckCfg_invalidParam_resource(void);
void test_neg_power_pwrSetBuckCfg_zeroValidParams(void);
void test_neg_power_pwrGetBuckCfg_zeroValidParams(void);
void test_neg_power_pwrSetBuckCfg_outOfBounds_vset(void);
void test_neg_power_pwrSetBuckCfg_outOfBounds_vsetActive(void);
void test_neg_power_pwrSetBuckCfg_outOfBounds_vsetLPwr(void);
void test_neg_power_pwrSetBuckCfg_outOfBounds_uvThr(void);
void test_neg_power_pwrSetBuckCfg_outOfBounds_ovThr(void);
void test_neg_power_pwrSetBuckCfg_outOfBounds_ilimSel_buck1(void);
void test_neg_power_pwrSetBuckCfg_outOfBounds_ilimSel_buck2(void);
void test_neg_power_pwrSetBuckCfg_outOfBounds_ovpSel(void);
void test_neg_power_pwrSetBuckCfg_outOfBounds_ovSel(void);
void test_neg_power_pwrSetBuckCfg_outOfBounds_uvSel(void);
void test_neg_power_pwrSetBuckCfg_outOfBounds_scSel(void);
void test_neg_power_pwrSetBuckCfg_outOfBounds_rvConf(void);
void test_neg_power_pwrSetBuckCfg_outOfBounds_slewRate(void);
void test_neg_power_pwrSetBuckCfg_outOfBounds_deglitchSel(void);
void test_neg_power_pwrSetBuckCfg_outOfBounds_dischargeSel(void);
void test_neg_power_pwrSetBuckCfg_outOfBounds_uvloRising(void);
void test_neg_power_pwrSetBuckCfg_outOfBounds_uvloFalling(void);
void test_neg_power_pwrSetBuckCfg_outOfBounds_highSideSlewRate(void);

/* Buck Current Limit Selection Tests */
void test_pos_power_buckSetGetCfg_ilimSel_buck1_allValues(void);
void test_pos_power_buckSetGetCfg_ilimSel_buck2_allValues(void);
void test_pos_power_buckSetGetCfg_ilimSel_buck3_allValues(void);
void test_pos_power_buckSetGetCfg_ilimSel_buck1_min(void);
void test_pos_power_buckSetGetCfg_ilimSel_buck1_max(void);
void test_pos_power_buckSetGetCfg_ilimSel_buck2_min(void);
void test_pos_power_buckSetGetCfg_ilimSel_buck2_max(void);
void test_pos_power_buckSetGetCfg_ilimSel_buck3_min(void);
void test_pos_power_buckSetGetCfg_ilimSel_buck3_max(void);

/* Buck OVP Fault Response Tests */
void test_pos_power_buckSetGetCfg_ovpSel_buck1_ignore(void);
void test_pos_power_buckSetGetCfg_ovpSel_buck1_assertNint(void);
void test_pos_power_buckSetGetCfg_ovpSel_buck2_waitPwrCycle(void);
void test_pos_power_buckSetGetCfg_ovpSel_buck3_ordShutdown(void);
void test_pos_power_buckSetGetCfg_ovpSel_buck1_immShutdown(void);
void test_pos_power_buckSetGetCfg_ovpSel_buck2_allResponses(void);

/* Buck OV Fault Response Tests */
void test_pos_power_buckSetGetCfg_ovSel_buck1_ignore(void);
void test_pos_power_buckSetGetCfg_ovSel_buck2_assertNint(void);
void test_pos_power_buckSetGetCfg_ovSel_buck3_waitPwrCycle(void);
void test_pos_power_buckSetGetCfg_ovSel_buck1_allResponses(void);

/* Buck UV Fault Response Tests */
void test_pos_power_buckSetGetCfg_uvSel_buck1_ignore(void);
void test_pos_power_buckSetGetCfg_uvSel_buck2_assertNint(void);
void test_pos_power_buckSetGetCfg_uvSel_buck3_allResponses(void);
void test_pos_power_buckSetGetCfg_uvSel_buck1_waitPwrCycle(void);

/* Buck SC Fault Response Tests */
void test_pos_power_buckSetGetCfg_scSel_buck1_ignore(void);
void test_pos_power_buckSetGetCfg_scSel_buck2_assertNint(void);
void test_pos_power_buckSetGetCfg_scSel_buck3_allResponses(void);
void test_pos_power_buckSetGetCfg_scSel_buck1_ordShutdown(void);

/* Buck RV Configuration Tests */
void test_pos_power_buckSetGetCfg_rvConf_buck1_wait(void);
void test_pos_power_buckSetGetCfg_rvConf_buck2_ignore(void);
void test_pos_power_buckSetGetCfg_rvConf_buck3_allValues(void);

/* Buck Multi-Parameter Configuration Tests */
void test_pos_power_buckSetGetCfg_multiParam_buck1_enableVsetPldnFpwm(void);
void test_pos_power_buckSetGetCfg_multiParam_buck2_enableVsetActiveThresholds(void);
void test_pos_power_buckSetGetCfg_multiParam_buck3_allCommonParams(void);
void test_pos_power_buckSetGetCfg_multiParam_buck1_allFaultResponses(void);
void test_pos_power_buckSetGetCfg_multiParam_buck2_fullConfig(void);

/* Buck Voltage and Threshold Configuration Tests */
void test_pos_power_buckSetGetCfg_vset_buck1_min(void);
void test_pos_power_buckSetGetCfg_vset_buck1_mid(void);
void test_pos_power_buckSetGetCfg_vset_buck1_max(void);
void test_pos_power_buckSetGetCfg_vset_buck1_boundary_low(void);
void test_pos_power_buckSetGetCfg_vset_buck1_boundary_high(void);
void test_pos_power_buckSetGetCfg_vsetActive_buck2_min(void);
void test_pos_power_buckSetGetCfg_vsetActive_buck2_mid(void);
void test_pos_power_buckSetGetCfg_vsetActive_buck2_max(void);
void test_pos_power_buckSetGetCfg_vsetActive_buck3_min(void);
void test_pos_power_buckSetGetCfg_vsetActive_buck3_mid(void);
void test_pos_power_buckSetGetCfg_vsetActive_buck3_max(void);
void test_pos_power_buckSetGetCfg_vsetActive_buck2_boundary(void);
void test_pos_power_buckSetGetCfg_vsetActive_buck3_boundary(void);
void test_pos_power_buckSetGetCfg_vsetLPwr_buck2_min(void);
void test_pos_power_buckSetGetCfg_vsetLPwr_buck2_max(void);
void test_pos_power_buckSetGetCfg_vsetLPwr_buck3_min(void);
void test_pos_power_buckSetGetCfg_vsetLPwr_buck3_max(void);
void test_pos_power_buckSetGetCfg_vsetLPwr_buck2_mid(void);
void test_pos_power_buckSetGetCfg_vsetLPwr_buck3_mid(void);
void test_pos_power_buckSetGetCfg_vmonOnly_buck2_enable(void);
void test_pos_power_buckSetGetCfg_vmonOnly_buck3_enable(void);
void test_pos_power_buckSetGetCfg_uvThr_buck1_val0(void);
void test_pos_power_buckSetGetCfg_uvThr_buck1_val1(void);
void test_pos_power_buckSetGetCfg_uvThr_buck2_val2(void);
void test_pos_power_buckSetGetCfg_uvThr_buck2_val3(void);
void test_pos_power_buckSetGetCfg_uvThr_buck3_val0(void);
void test_pos_power_buckSetGetCfg_uvThr_buck3_val3(void);
void test_pos_power_buckSetGetCfg_ovThr_buck1_val0(void);
void test_pos_power_buckSetGetCfg_ovThr_buck1_val1(void);
void test_pos_power_buckSetGetCfg_ovThr_buck2_val2(void);
void test_pos_power_buckSetGetCfg_ovThr_buck2_val3(void);
void test_pos_power_buckSetGetCfg_ovThr_buck3_val0(void);
void test_pos_power_buckSetGetCfg_ovThr_buck3_val3(void);
void test_pos_power_buckSetGetCfg_slewRate_buck1_allValues(void);
void test_pos_power_buckSetGetCfg_slewRate_buck2_minMax(void);
void test_pos_power_buckSetGetCfg_slewRate_buck3_minMax(void);
void test_pos_power_buckSetGetCfg_highSideSlewRate_buck1_fast(void);
void test_pos_power_buckSetGetCfg_highSideSlewRate_buck1_slow(void);
void test_pos_power_buckSetGetCfg_highSideSlewRate_buck1_slowest(void);
void test_pos_power_buckSetGetCfg_deglitchSel_allBucks(void);

/* SSM_SEL and UVLO Coverage Tests */
void test_pos_power_buckSetGetCfg_ssmSel_buck1(void);
void test_pos_power_buckSetGetCfg_ssmSel_buck2(void);
void test_pos_power_buckSetGetCfg_ssmSel_buck3(void);
void test_pos_power_buckSetGetCfg_ssmSel_allValues(void);
void test_neg_power_buckSetCfg_ssmSel_outOfBounds(void);
void test_pos_power_buckSetGetCfg_uvloRising_buck1_allValues(void);
void test_pos_power_buckSetGetCfg_uvloFalling_buck1_allValues(void);
void test_pos_power_buckSetGetCfg_uvloRisingAndFalling_buck1(void);

/* Parameter Mismatch Tests */
void test_neg_power_pwrSetBuckCfg_buck1_vsetActiveNotSupported(void);
void test_neg_power_pwrGetBuckCfg_buck1_vsetActiveNotSupported(void);
void test_neg_power_pwrSetBuckCfg_buck1_vsetLpwrNotSupported(void);
void test_neg_power_pwrGetBuckCfg_buck1_vsetLpwrNotSupported(void);
void test_neg_power_pwrSetBuckCfg_buck2_vsetNotSupported(void);
void test_neg_power_pwrGetBuckCfg_buck2_vsetNotSupported(void);
void test_neg_power_pwrSetBuckCfg_buck3_vsetNotSupported(void);
void test_neg_power_pwrGetBuckCfg_buck3_vsetNotSupported(void);
void test_neg_power_pwrSetBuckCfg_buck1_vmonOnlyNotSupported(void);
void test_neg_power_pwrGetBuckCfg_buck1_vmonOnlyNotSupported(void);
void test_neg_power_pwrSetBuckCfg_buck2_highSideSlewRateNotSupported(void);
void test_neg_power_pwrGetBuckCfg_buck2_highSideSlewRateNotSupported(void);
void test_neg_power_pwrSetBuckCfg_buck3_highSideSlewRateNotSupported(void);
void test_neg_power_pwrGetBuckCfg_buck3_highSideSlewRateNotSupported(void);

/* BUCK2/BUCK3 Specific Configuration Path Coverage Tests */
void test_pos_power_buckSetCfg_buck2OvpSel(void);
void test_pos_power_buckSetCfg_buck3OvpSel(void);
void test_pos_power_buckSetCfg_buck2OvSel(void);
void test_pos_power_buckSetCfg_buck3OvSel(void);
void test_pos_power_buckSetCfg_buck2UvSel(void);
void test_pos_power_buckSetCfg_buck3UvSel(void);
void test_pos_power_buckSetCfg_buck2ScSel(void);
void test_pos_power_buckSetCfg_buck3ScSel(void);
void test_pos_power_buckSetCfg_buck1RvConf(void);
void test_pos_power_buckSetCfg_buck1IlimSel(void);

void test_pos_power_powerGetCfg_buck2Enable(void);
void test_pos_power_powerSetCfg_buck2Enable(void);
void test_pos_power_powerGetVoutCfg_buck2Voltage(void);
void test_pos_power_powerSetVoutCfg_buck2Voltage(void);
void test_pos_power_powerGetCfg_buck3Enable(void);
void test_pos_power_powerSetCfg_buck3Enable(void);
void test_pos_power_powerGetVoutCfg_buck3Voltage(void);
void test_pos_power_powerSetVoutCfg_buck3Voltage(void);
void test_pos_power_powerGetStat_buck2(void);
void test_pos_power_powerGetStat_buck3(void);

/* Additional negative tests for error path coverage */
void test_neg_power_pwr_getUvlo_invalidBuck(void);
void test_neg_power_pwr_setUvlo_invalidBuck(void);
void test_neg_power_pwr_setBuck_invalidDischargeSel(void);
void test_neg_power_pwr_setBuck_invalidSlewRate(void);

void test_neg_power_pwr_setLdoCfg_zeroValidParams(void);
void test_neg_power_pwr_getLdoCfg_zeroValidParams(void);
void test_neg_power_pwr_setTsdCfg_zeroValidParams(void);
void test_neg_power_pwr_getTsdCfg_zeroValidParams(void);
void test_neg_power_pwr_setSeqTrig_zeroLen(void);
void test_neg_power_pwr_setSeqTrig_lenExceedsMax(void);
void test_neg_power_pwr_setSeqTrig_invalidPwrRsrc(void);
void test_neg_power_pwr_setSeqTrig_invalidBitPos(void);
void test_neg_power_pwr_getSeqTrig_zeroLen(void);
void test_neg_power_pwr_getSeqTrig_lenExceedsMax(void);
void test_neg_power_pwr_getSeqTrig_invalidResource(void);
void test_neg_power_pwr_getSeqTrig_invalidBitPos(void);
void test_neg_power_pwrGetBuckLdoSeqTrig_ioFailure(void);
void test_neg_power_pwr_setSeqDelay_zeroLen(void);
void test_neg_power_pwr_setSeqDelay_lenExceedsMax(void);
void test_neg_power_pwr_setSeqDelay_invalidConfig(void);
void test_neg_power_pwr_getSeqDelay_zeroLen(void);
void test_neg_power_pwr_getSeqDelay_lenExceedsMax(void);
void test_neg_power_pwr_getSeqDelay_invalidConfig(void);

/* Buck/LDO Sequencing Tests */
void test_neg_power_pwrGetBuckCfg_nullHandle(void);
void test_neg_power_pwrGetBuckLdoSeqDly_zeroValidParams(void);
void test_neg_power_pwrGetBuckLdoSeqDly_ioFailure(void);
void test_pos_power_pwrGetBuckLdoSeqDly_onlyOffDelay(void);
void test_pos_power_pwrGetBuckLdoSeqDly_onlyOnDelay(void);
void test_neg_power_pwrSetBuckLdoSeqDly_zeroValidParams(void);
void test_neg_power_pwrSetBuckLdoSeqDly_invalidResource(void);
void test_neg_power_pwrSetBuckLdoSeqTrig_ioFailure(void);

/* Power static function I/O failure tests - Pmic_pwrGetBuckCfg */
void test_neg_power_pwrGetBuckCfg_spreadSpectrum_ioRxByteCSFail(void);
void test_neg_power_pwrGetBuckCfg_buckVSET_ioRxByteCSFail(void);
void test_neg_power_pwrGetBuckCfg_buckVsetActive_ioRxByteCSFail(void);
void test_neg_power_pwrGetBuckCfg_buckVsetLPwr_ioRxByteCSFail(void);
void test_neg_power_pwrGetBuckCfg_buck1Ctrl_ioRxByteCSFail(void);
void test_neg_power_pwrGetBuckCfg_buck2_3Ctrl_ioRxByteCSFail(void);
void test_neg_power_pwrGetBuckCfg_buckMonConf_ioRxByteCSFail(void);
void test_neg_power_pwrGetBuckCfg_buckOvpResponse_ioRxByteCSFail(void);
void test_neg_power_pwrGetBuckCfg_buckOvResponse_ioRxByteCSFail(void);
void test_neg_power_pwrGetBuckCfg_buckUvResponse_ioRxByteCSFail(void);
void test_neg_power_pwrGetBuckCfg_buckScResponse_ioRxByteCSFail(void);

/* Power static function I/O failure tests - Pmic_pwrSetBuckCfg */
void test_neg_power_pwrSetBuckCfg_buckActiveVSET_ioRxByteFail(void);
void test_neg_power_pwrSetBuckCfg_buckLPwrVSET_ioRxByteFail(void);
void test_neg_power_pwrSetBuckCfg_buckOvpResponse_ioRxByteFail(void);
void test_neg_power_pwrSetBuckCfg_buckOvResponse_ioRxByteFail(void);
void test_neg_power_pwrSetBuckCfg_buckUvResponse_ioRxByteFail(void);
void test_neg_power_pwrSetBuckCfg_buckScResponse_ioRxByteFail(void);
void test_neg_power_pwrSetBuckCfg_buck2_3Ctrl_ioRxByteFail(void);

/* Power static function I/O failure tests - Pmic_pwrGetLdoCfg */
void test_neg_power_pwrGetLdoCfg_ldoConf_ioRxByteCSFail(void);
void test_neg_power_pwrGetLdoCfg_ldoCtrl_ioRxByteCSFail(void);
void test_neg_power_pwrGetLdoCfg_ldoMonConf_ioRxByteCSFail(void);
void test_neg_power_pwrGetLdoCfg_ldoOvpResponse_ioRxByteCSFail(void);
void test_neg_power_pwrGetLdoCfg_ldoOvResponse_ioRxByteCSFail(void);
void test_neg_power_pwrGetLdoCfg_ldoUvResponse_ioRxByteCSFail(void);
void test_neg_power_pwrGetLdoCfg_ldoScResponse_ioRxByteCSFail(void);

/* Power static function I/O failure tests - Pmic_pwrSetLdoCfg */
void test_neg_power_pwrSetLdoCfg_ldoConf_vsetOutOfBounds_ioPath(void);
void test_neg_power_pwrSetLdoCfg_ldoOvpResponse_ioRxByteFail(void);
void test_neg_power_pwrSetLdoCfg_ldoOvResponse_ioRxByteFail(void);
void test_neg_power_pwrSetLdoCfg_ldoUvResponse_ioRxByteFail(void);
void test_neg_power_pwrSetLdoCfg_ldoScResponse_ioRxByteFail(void);

/* Power static function I/O failure tests - Pmic_pwrGetRsrcStatus */
void test_neg_power_pwrGetRsrcStatus_statStartup_ioRxByteCSFail(void);
void test_neg_power_pwrGetRsrcStatus_statBuck1_2_ioRxByteCSFail(void);
void test_neg_power_pwrGetRsrcStatus_statBuck3Ldo_ioRxByteCSFail(void);

void test_neg_power_pwrSetBuckCfg_setBuckActiveVSET_belowMin(void);
void test_neg_power_pwrSetBuckCfg_setBuckLPwrVSET_belowMin(void);
void test_neg_power_pwrSetLdoCfg_setLdoConf_vsetBelowMin(void);

void test_neg_power_pwrGetBuckLdoSeqDly_lenMax_ioFailure(void);
void test_neg_power_pwrSetBuckLdoSeqTrig_lenMax_ioFailure(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* POWER_TEST_H */
