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
#include "../test_common.h"
#include "platform.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Function Declarations                            */
/* ========================================================================== */
void power_test(void *args);

/* LDO Configuration Tests - Negative */
void test_negative_Pmic_pwrSetLdoCfg_nullParam_pmicHandle(void);
void test_negative_Pmic_pwrSetLdoCfg_nullParam_ldoCfg(void);
void test_negative_Pmic_pwrSetLdoCfg_outOfBounds_mode(void);
void test_negative_Pmic_pwrSetLdoCfg_outOfBounds_vset(void);
void test_negative_Pmic_pwrSetLdoCfg_outOfBounds_dischargeSel(void);
void test_negative_Pmic_pwrSetLdoCfg_outOfBounds_deglitchSel(void);
void test_negative_Pmic_pwrSetLdoCfg_outOfBounds_uvThr(void);
void test_negative_Pmic_pwrSetLdoCfg_outOfBounds_ovThr(void);
void test_negative_Pmic_pwrSetLdoCfg_outOfBounds_ilimSel(void);
void test_negative_Pmic_pwrSetLdoCfg_outOfBounds_ovpSel(void);
void test_negative_Pmic_pwrSetLdoCfg_outOfBounds_ovSel(void);
void test_negative_Pmic_pwrSetLdoCfg_outOfBounds_uvSel(void);
void test_negative_Pmic_pwrSetLdoCfg_outOfBounds_scSel(void);
void test_negative_Pmic_pwrSetLdoCfg_outOfBounds_rvConf(void);
void test_negative_Pmic_pwrGetLdoCfg_nullParam_pmicHandle(void);
void test_negative_Pmic_pwrGetLdoCfg_nullParam_ldoCfg(void);

/* LDO Configuration Tests - Positive */
void test_positive_ldoSetGetCfg_enable(void);
void test_positive_ldoSetGetCfg_mode(void);
void test_positive_ldoSetGetCfg_vset(void);
void test_positive_ldoSetGetCfg_vmonOnly(void);
void test_positive_ldoSetGetCfg_dischargeEn(void);
void test_positive_ldoSetGetCfg_dischargeSel(void);
void test_positive_ldoSetGetCfg_deglitchSel(void);
void test_positive_ldoSetGetCfg_uvThr(void);
void test_positive_ldoSetGetCfg_ovThr(void);
void test_positive_ldoSetGetCfg_ilimSel(void);
void test_positive_ldoSetGetCfg_ovpSel(void);
void test_positive_ldoSetGetCfg_ovSel(void);
void test_positive_ldoSetGetCfg_uvSel(void);
void test_positive_ldoSetGetCfg_scSel(void);
void test_positive_ldoSetGetCfg_rvConf(void);

#ifdef BUILD_MOCK
/* LDO Configuration Tests - Property Tests */
void test_property_ldoSetGetCfg_vset_allValues(void);
void test_property_ldoSetGetCfg_dischargeSel_allValues(void);
void test_property_ldoSetGetCfg_deglitchSel_allValues(void);
#endif

/* TSD Configuration Tests - Negative */
void test_negative_Pmic_pwrSetTsdCfg_nullParam_pmicHandle(void);
void test_negative_Pmic_pwrSetTsdCfg_nullParam_tsdCfg(void);
void test_negative_Pmic_pwrSetTsdCfg_outOfBounds_tsdImmLevel(void);
void test_negative_Pmic_pwrSetTsdCfg_outOfBounds_twarnLevel(void);
void test_negative_Pmic_pwrGetTsdCfg_nullParam_pmicHandle(void);
void test_negative_Pmic_pwrGetTsdCfg_nullParam_tsdCfg(void);
void test_negative_Pmic_pwrGetTsdImmStatus_nullParam_pmicHandle(void);
void test_negative_Pmic_pwrGetTsdImmStatus_nullParam_tsdImmStat(void);

/* TSD Configuration Tests - Positive */
void test_positive_tsdSetGetCfg_twarnStayInSafeState(void);
void test_positive_tsdSetGetCfg_tsdImmLevel(void);
void test_positive_tsdSetGetCfg_twarnLevel(void);
void test_positive_tsdGetImmStatus(void);

/* Resource Status Tests - Negative */
void test_negative_Pmic_pwrGetRsrcStatus_nullParam_pmicHandle(void);
void test_negative_Pmic_pwrGetRsrcStatus_nullParam_pwrRsrcStat(void);
void test_negative_Pmic_pwrGetRsrcStatus_outOfBounds_resource(void);

/* Resource Status Tests - Positive */
void test_positive_rsrcGetStatus_buck1(void);
void test_positive_rsrcGetStatus_buck2(void);
void test_positive_rsrcGetStatus_buck3(void);
void test_positive_rsrcGetStatus_ldo(void);

/* Sequencing Tests - Negative */
void test_negative_Pmic_pwrSetBuckLdoSeqTrig_nullParam_pmicHandle(void);
void test_negative_Pmic_pwrSetBuckLdoSeqTrig_nullParam_seqTrigCfg(void);
void test_negative_Pmic_pwrGetBuckLdoSeqTrig_nullParam_pmicHandle(void);
void test_negative_Pmic_pwrGetBuckLdoSeqTrig_nullParam_seqTrigCfg(void);
void test_negative_Pmic_pwrSetBuckLdoSeqDly_nullParam_pmicHandle(void);
void test_negative_Pmic_pwrSetBuckLdoSeqDly_nullParam_seqDlyCfg(void);
void test_negative_Pmic_pwrSetBuckLdoSeqDly_outOfBounds_seqDlyOn(void);
void test_negative_Pmic_pwrSetBuckLdoSeqDly_outOfBounds_seqDlyOff(void);
void test_negative_Pmic_pwrGetBuckLdoSeqDly_nullParam_pmicHandle(void);
void test_negative_Pmic_pwrGetBuckLdoSeqDly_nullParam_seqDlyCfg(void);

/* Sequencing Tests - Positive */
void test_positive_seqTrigSetGet_buck1(void);
void test_positive_seqTrigSetGet_buck2(void);
void test_positive_seqTrigSetGet_buck3(void);
void test_positive_seqTrigSetGet_ldo(void);
void test_positive_seqTrigSetGet_allResources(void);
void test_positive_seqDlySetGet_buck1(void);
void test_positive_seqDlySetGet_buck2(void);
void test_positive_seqDlySetGet_buck3(void);
void test_positive_seqDlySetGet_ldo(void);
void test_positive_seqDlySetGet_allResources(void);

#ifdef BUILD_MOCK
/* Sequencing Tests - Property Tests */
void test_property_seqDlySetGet_allDelayValues_buck1(void);
void test_property_seqDlySetGet_allDelayValues_buck2(void);
void test_property_seqDlySetGet_allDelayValues_buck3(void);
void test_property_seqDlySetGet_allDelayValues_ldo(void);
#endif

/* Buck Configuration Tests - Positive */
void test_positive_buckSetGetCfg_enable_buck1(void);
void test_positive_buckSetGetCfg_enable_buck2(void);
void test_positive_buckSetGetCfg_enable_buck3(void);
void test_positive_buckSetGetCfg_disable_buck1(void);
void test_positive_buckSetGetCfg_disable_buck2(void);
void test_positive_buckSetGetCfg_disable_buck3(void);
void test_positive_buckSetGetCfg_fpwmEn_enable_buck1(void);
void test_positive_buckSetGetCfg_fpwmEn_enable_buck2(void);
void test_positive_buckSetGetCfg_fpwmEn_enable_buck3(void);
void test_positive_buckSetGetCfg_fpwmEn_disable_buck1(void);
void test_positive_buckSetGetCfg_fpwmEn_disable_buck2(void);
void test_positive_buckSetGetCfg_fpwmEn_disable_buck3(void);
void test_positive_buckSetGetCfg_pldnEn_enable_buck1(void);
void test_positive_buckSetGetCfg_pldnEn_enable_buck2(void);
void test_positive_buckSetGetCfg_pldnEn_enable_buck3(void);
void test_positive_buckSetGetCfg_pldnEn_disable_buck1(void);
void test_positive_buckSetGetCfg_pldnEn_disable_buck2(void);
void test_positive_buckSetGetCfg_pldnEn_disable_buck3(void);
void test_positive_buckSetGetCfg_dischargeSel_buck1_active(void);
void test_positive_buckSetGetCfg_dischargeSel_buck1_resistive(void);
void test_positive_buckSetGetCfg_dischargeSel_buck2_active(void);
void test_positive_buckSetGetCfg_dischargeSel_buck2_resistive(void);
void test_positive_buckSetGetCfg_dischargeSel_buck3_active(void);
void test_positive_buckSetGetCfg_dischargeSel_buck3_resistive(void);
void test_positive_buckSetGetCfg_ssEn_enable_buck1(void);
void test_positive_buckSetGetCfg_ssEn_enable_buck2(void);
void test_positive_buckSetGetCfg_ssEn_enable_buck3(void);
void test_positive_buckSetGetCfg_ssEn_disable_buck1(void);
void test_positive_buckSetGetCfg_ssEn_disable_buck2(void);
void test_positive_buckSetGetCfg_ssEn_disable_buck3(void);

/* Buck Configuration GET-only Tests */
void test_positive_buckGetCfg_ovpSel_buck1(void);
void test_positive_buckGetCfg_ovpSel_buck2(void);
void test_positive_buckGetCfg_ovpSel_buck3(void);
void test_positive_buckGetCfg_ovSel_buck1(void);
void test_positive_buckGetCfg_ovSel_buck2(void);
void test_positive_buckGetCfg_ovSel_buck3(void);
void test_positive_buckGetCfg_uvSel_buck1(void);
void test_positive_buckGetCfg_uvSel_buck2(void);
void test_positive_buckGetCfg_uvSel_buck3(void);
void test_positive_buckGetCfg_scSel_buck1(void);
void test_positive_buckGetCfg_scSel_buck2(void);
void test_positive_buckGetCfg_scSel_buck3(void);
void test_positive_buckGetCfg_rvConf_buck1(void);
void test_positive_buckGetCfg_rvConf_buck2(void);
void test_positive_buckGetCfg_rvConf_buck3(void);
void test_positive_buckGetCfg_ilimSel_buck1(void);
void test_positive_buckGetCfg_ilimSel_buck2(void);
void test_positive_buckGetCfg_ilimSel_buck3(void);

#ifdef BUILD_MOCK
/* Buck Configuration Tests - Negative */
void test_negative_Pmic_pwrSetBuckCfg_nullParam_pmicHandle(void);
void test_negative_Pmic_pwrSetBuckCfg_nullParam_buckCfg(void);
void test_negative_Pmic_pwrGetBuckCfg_nullParam_buckCfg(void);
void test_negative_Pmic_pwrSetBuckCfg_invalidParam_resource(void);
void test_negative_Pmic_pwrGetBuckCfg_invalidParam_resource(void);
void test_negative_Pmic_pwrSetBuckCfg_zeroValidParams(void);
void test_negative_Pmic_pwrGetBuckCfg_zeroValidParams(void);
void test_negative_Pmic_pwrSetBuckCfg_outOfBounds_vset(void);
void test_negative_Pmic_pwrSetBuckCfg_outOfBounds_vsetActive(void);
void test_negative_Pmic_pwrSetBuckCfg_outOfBounds_vsetLPwr(void);
void test_negative_Pmic_pwrSetBuckCfg_outOfBounds_uvThr(void);
void test_negative_Pmic_pwrSetBuckCfg_outOfBounds_ovThr(void);
void test_negative_Pmic_pwrSetBuckCfg_outOfBounds_ilimSel_buck1(void);
void test_negative_Pmic_pwrSetBuckCfg_outOfBounds_ilimSel_buck2(void);
void test_negative_Pmic_pwrSetBuckCfg_outOfBounds_ovpSel(void);
void test_negative_Pmic_pwrSetBuckCfg_outOfBounds_ovSel(void);
void test_negative_Pmic_pwrSetBuckCfg_outOfBounds_uvSel(void);
void test_negative_Pmic_pwrSetBuckCfg_outOfBounds_scSel(void);
void test_negative_Pmic_pwrSetBuckCfg_outOfBounds_rvConf(void);
void test_negative_Pmic_pwrSetBuckCfg_outOfBounds_slewRate(void);
void test_negative_Pmic_pwrSetBuckCfg_outOfBounds_deglitchSel(void);
void test_negative_Pmic_pwrSetBuckCfg_outOfBounds_dischargeSel(void);
void test_negative_Pmic_pwrSetBuckCfg_outOfBounds_uvloRising(void);
void test_negative_Pmic_pwrSetBuckCfg_outOfBounds_uvloFalling(void);
void test_negative_Pmic_pwrSetBuckCfg_outOfBounds_highSideSlewRate(void);

/* Buck Current Limit Selection Tests */
void test_positive_buckSetGetCfg_ilimSel_buck1_allValues(void);
void test_positive_buckSetGetCfg_ilimSel_buck2_allValues(void);
void test_positive_buckSetGetCfg_ilimSel_buck3_allValues(void);
void test_positive_buckSetGetCfg_ilimSel_buck1_min(void);
void test_positive_buckSetGetCfg_ilimSel_buck1_max(void);
void test_positive_buckSetGetCfg_ilimSel_buck2_min(void);
void test_positive_buckSetGetCfg_ilimSel_buck2_max(void);
void test_positive_buckSetGetCfg_ilimSel_buck3_min(void);
void test_positive_buckSetGetCfg_ilimSel_buck3_max(void);

/* Buck OVP Fault Response Tests */
void test_positive_buckSetGetCfg_ovpSel_buck1_ignore(void);
void test_positive_buckSetGetCfg_ovpSel_buck1_assertNint(void);
void test_positive_buckSetGetCfg_ovpSel_buck2_waitPwrCycle(void);
void test_positive_buckSetGetCfg_ovpSel_buck3_ordShutdown(void);
void test_positive_buckSetGetCfg_ovpSel_buck1_immShutdown(void);
void test_positive_buckSetGetCfg_ovpSel_buck2_allResponses(void);

/* Buck OV Fault Response Tests */
void test_positive_buckSetGetCfg_ovSel_buck1_ignore(void);
void test_positive_buckSetGetCfg_ovSel_buck2_assertNint(void);
void test_positive_buckSetGetCfg_ovSel_buck3_waitPwrCycle(void);
void test_positive_buckSetGetCfg_ovSel_buck1_allResponses(void);

/* Buck UV Fault Response Tests */
void test_positive_buckSetGetCfg_uvSel_buck1_ignore(void);
void test_positive_buckSetGetCfg_uvSel_buck2_assertNint(void);
void test_positive_buckSetGetCfg_uvSel_buck3_allResponses(void);
void test_positive_buckSetGetCfg_uvSel_buck1_waitPwrCycle(void);

/* Buck SC Fault Response Tests */
void test_positive_buckSetGetCfg_scSel_buck1_ignore(void);
void test_positive_buckSetGetCfg_scSel_buck2_assertNint(void);
void test_positive_buckSetGetCfg_scSel_buck3_allResponses(void);
void test_positive_buckSetGetCfg_scSel_buck1_ordShutdown(void);

/* Buck RV Configuration Tests */
void test_positive_buckSetGetCfg_rvConf_buck1_wait(void);
void test_positive_buckSetGetCfg_rvConf_buck2_ignore(void);
void test_positive_buckSetGetCfg_rvConf_buck3_allValues(void);

/* Buck Multi-Parameter Configuration Tests */
void test_positive_buckSetGetCfg_multiParam_buck1_enableVsetPldnFpwm(void);
void test_positive_buckSetGetCfg_multiParam_buck2_enableVsetActiveThresholds(void);
void test_positive_buckSetGetCfg_multiParam_buck3_allCommonParams(void);
void test_positive_buckSetGetCfg_multiParam_buck1_allFaultResponses(void);
void test_positive_buckSetGetCfg_multiParam_buck2_fullConfig(void);

/* Buck Voltage and Threshold Configuration Tests */
void test_positive_buckSetGetCfg_vset_buck1_min(void);
void test_positive_buckSetGetCfg_vset_buck1_mid(void);
void test_positive_buckSetGetCfg_vset_buck1_max(void);
void test_positive_buckSetGetCfg_vset_buck1_boundary_low(void);
void test_positive_buckSetGetCfg_vset_buck1_boundary_high(void);
void test_positive_buckSetGetCfg_vsetActive_buck2_min(void);
void test_positive_buckSetGetCfg_vsetActive_buck2_mid(void);
void test_positive_buckSetGetCfg_vsetActive_buck2_max(void);
void test_positive_buckSetGetCfg_vsetActive_buck3_min(void);
void test_positive_buckSetGetCfg_vsetActive_buck3_mid(void);
void test_positive_buckSetGetCfg_vsetActive_buck3_max(void);
void test_positive_buckSetGetCfg_vsetActive_buck2_boundary(void);
void test_positive_buckSetGetCfg_vsetActive_buck3_boundary(void);
void test_positive_buckSetGetCfg_vsetLPwr_buck2_min(void);
void test_positive_buckSetGetCfg_vsetLPwr_buck2_max(void);
void test_positive_buckSetGetCfg_vsetLPwr_buck3_min(void);
void test_positive_buckSetGetCfg_vsetLPwr_buck3_max(void);
void test_positive_buckSetGetCfg_vsetLPwr_buck2_mid(void);
void test_positive_buckSetGetCfg_vsetLPwr_buck3_mid(void);
void test_positive_buckSetGetCfg_vmonOnly_buck2_enable(void);
void test_positive_buckSetGetCfg_vmonOnly_buck3_enable(void);
void test_positive_buckSetGetCfg_uvThr_buck1_val0(void);
void test_positive_buckSetGetCfg_uvThr_buck1_val1(void);
void test_positive_buckSetGetCfg_uvThr_buck2_val2(void);
void test_positive_buckSetGetCfg_uvThr_buck2_val3(void);
void test_positive_buckSetGetCfg_uvThr_buck3_val0(void);
void test_positive_buckSetGetCfg_uvThr_buck3_val3(void);
void test_positive_buckSetGetCfg_ovThr_buck1_val0(void);
void test_positive_buckSetGetCfg_ovThr_buck1_val1(void);
void test_positive_buckSetGetCfg_ovThr_buck2_val2(void);
void test_positive_buckSetGetCfg_ovThr_buck2_val3(void);
void test_positive_buckSetGetCfg_ovThr_buck3_val0(void);
void test_positive_buckSetGetCfg_ovThr_buck3_val3(void);
void test_positive_buckSetGetCfg_slewRate_buck1_allValues(void);
void test_positive_buckSetGetCfg_slewRate_buck2_minMax(void);
void test_positive_buckSetGetCfg_slewRate_buck3_minMax(void);
void test_positive_buckSetGetCfg_highSideSlewRate_buck1_fast(void);
void test_positive_buckSetGetCfg_highSideSlewRate_buck1_slow(void);
void test_positive_buckSetGetCfg_highSideSlewRate_buck1_slowest(void);
void test_positive_buckSetGetCfg_deglitchSel_allBucks(void);

/* SSM_SEL and UVLO Coverage Tests */
void test_positive_buckSetGetCfg_ssmSel_buck1(void);
void test_positive_buckSetGetCfg_ssmSel_buck2(void);
void test_positive_buckSetGetCfg_ssmSel_buck3(void);
void test_positive_buckSetGetCfg_ssmSel_allValues(void);
void test_negative_buckSetCfg_ssmSel_outOfBounds(void);
void test_positive_buckSetGetCfg_uvloRising_buck1_allValues(void);
void test_positive_buckSetGetCfg_uvloFalling_buck1_allValues(void);
void test_positive_buckSetGetCfg_uvloRisingAndFalling_buck1(void);

/* Parameter Mismatch Tests */
void test_negative_pwrSetBuckCfg_buck1_vsetActiveNotSupported(void);
void test_negative_pwrGetBuckCfg_buck1_vsetActiveNotSupported(void);
void test_negative_pwrSetBuckCfg_buck1_vsetLpwrNotSupported(void);
void test_negative_pwrGetBuckCfg_buck1_vsetLpwrNotSupported(void);
void test_negative_pwrSetBuckCfg_buck2_vsetNotSupported(void);
void test_negative_pwrGetBuckCfg_buck2_vsetNotSupported(void);
void test_negative_pwrSetBuckCfg_buck3_vsetNotSupported(void);
void test_negative_pwrGetBuckCfg_buck3_vsetNotSupported(void);
void test_negative_pwrSetBuckCfg_buck1_vmonOnlyNotSupported(void);
void test_negative_pwrGetBuckCfg_buck1_vmonOnlyNotSupported(void);
void test_negative_pwrSetBuckCfg_buck2_highSideSlewRateNotSupported(void);
void test_negative_pwrGetBuckCfg_buck2_highSideSlewRateNotSupported(void);
void test_negative_pwrSetBuckCfg_buck3_highSideSlewRateNotSupported(void);
void test_negative_pwrGetBuckCfg_buck3_highSideSlewRateNotSupported(void);

/* BUCK2/BUCK3 Specific Configuration Path Coverage Tests */
void test_positive_buckSetCfg_buck2OvpSel(void);
void test_positive_buckSetCfg_buck3OvpSel(void);
void test_positive_buckSetCfg_buck2OvSel(void);
void test_positive_buckSetCfg_buck3OvSel(void);
void test_positive_buckSetCfg_buck2UvSel(void);
void test_positive_buckSetCfg_buck3UvSel(void);
void test_positive_buckSetCfg_buck2ScSel(void);
void test_positive_buckSetCfg_buck3ScSel(void);
void test_positive_buckSetCfg_buck1RvConf(void);
void test_positive_buckSetCfg_buck1IlimSel(void);

/* BUCK2/BUCK3 Coverage Gap Tests */
void test_positive_powerGetCfg_buck2Enable(void);
void test_positive_powerSetCfg_buck2Enable(void);
void test_positive_powerGetVoutCfg_buck2Voltage(void);
void test_positive_powerSetVoutCfg_buck2Voltage(void);
void test_positive_powerGetCfg_buck3Enable(void);
void test_positive_powerSetCfg_buck3Enable(void);
void test_positive_powerGetVoutCfg_buck3Voltage(void);
void test_positive_powerSetVoutCfg_buck3Voltage(void);
void test_positive_powerGetStat_buck2(void);
void test_positive_powerGetStat_buck3(void);

/* Additional negative tests for error path coverage */
void test_negative_pwr_getUvlo_invalidBuck(void);
void test_negative_pwr_setUvlo_invalidBuck(void);
void test_negative_pwr_setBuck_invalidDischargeSel(void);
void test_negative_pwr_setBuck_invalidSlewRate(void);
void test_negative_pwr_setLdoCfg_zeroValidParams(void);
void test_negative_pwr_getLdoCfg_zeroValidParams(void);
void test_negative_pwr_setTsdCfg_zeroValidParams(void);
void test_negative_pwr_getTsdCfg_zeroValidParams(void);
void test_negative_pwr_setSeqTrig_zeroLen(void);
void test_negative_pwr_setSeqTrig_invalidPwrRsrc(void);
void test_negative_pwr_setSeqTrig_invalidBitPos(void);
void test_negative_pwr_getSeqTrig_zeroLen(void);
void test_negative_pwr_getSeqTrig_invalidResource(void);
void test_negative_pwr_getSeqTrig_invalidBitPos(void);
void test_negative_pwr_setSeqDelay_zeroLen(void);
void test_negative_pwr_setSeqDelay_invalidConfig(void);
void test_negative_pwr_getSeqDelay_zeroLen(void);
void test_negative_pwr_getSeqDelay_invalidConfig(void);
#endif

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* POWER_TEST_H */
