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
#ifndef POWER_TEST_H
#define POWER_TEST_H



/* ========================================================================== */
/*                              Include Files                                 */
/* ========================================================================== */

#include "../platform.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

void power_test(void *args);

/* Negative tests - BuckBoost Set/Get API NULL parameter tests */
void test_negative_Pmic_pwrSetBuckBoostCfg_nullParam_handle(void);
void test_negative_Pmic_pwrSetBuckBoostCfg_nullParam_config(void);
void test_negative_Pmic_pwrGetBuckBoostCfg_nullParam_handle(void);
void test_negative_Pmic_pwrGetBuckBoostCfg_nullParam_config(void);

/* Negative tests - BuckBoost out of bounds tests */
void test_negative_Pmic_pwrSetBuckBoostCfg_outOfBounds_lvl(void);
void test_negative_Pmic_pwrSetBuckBoostCfg_outOfBounds_stbyLvl(void);
void test_negative_Pmic_pwrSetBuckBoostCfg_outOfBounds_vmonThr(void);
void test_negative_Pmic_pwrSetBuckBoostCfg_outOfBounds_vmonDgl(void);
void test_negative_Pmic_pwrSetBuckBoostCfg_outOfBounds_boostTmo(void);

/* Negative tests - LDO Set/Get API NULL parameter tests */
void test_negative_Pmic_pwrSetLdoCfg_nullParam_handle(void);
void test_negative_Pmic_pwrSetLdoCfg_nullParam_config(void);
void test_negative_Pmic_pwrGetLdoCfg_nullParam_handle(void);
void test_negative_Pmic_pwrGetLdoCfg_nullParam_config(void);

/* Negative tests - LDO out of bounds tests for LDO1 */
void test_negative_Pmic_pwrSetLdoCfg_invalidParam_ldo1_mode(void);
void test_negative_Pmic_pwrSetLdoCfg_outOfBounds_ldo1_lvl(void);
void test_negative_Pmic_pwrSetLdoCfg_outOfBounds_ldo1_ilimLvl(void);
void test_negative_Pmic_pwrSetLdoCfg_outOfBounds_ldo1_ilimDgl(void);
void test_negative_Pmic_pwrSetLdoCfg_outOfBounds_ldo1_vmonThr(void);
void test_negative_Pmic_pwrSetLdoCfg_outOfBounds_ldo1_vmonDgl(void);
void test_negative_Pmic_pwrSetLdoCfg_outOfBounds_ldo1_rampTime(void);

/* Negative tests - LDO out of bounds tests for LDO2 */
void test_negative_Pmic_pwrSetLdoCfg_invalidParam_ldo2_mode(void);
void test_negative_Pmic_pwrSetLdoCfg_outOfBounds_ldo2_lvl(void);
void test_negative_Pmic_pwrSetLdoCfg_outOfBounds_ldo2_ilimLvl(void);
void test_negative_Pmic_pwrSetLdoCfg_outOfBounds_ldo2_ilimDgl(void);
void test_negative_Pmic_pwrSetLdoCfg_outOfBounds_ldo2_vmonThr(void);
void test_negative_Pmic_pwrSetLdoCfg_outOfBounds_ldo2_vmonDgl(void);
void test_negative_Pmic_pwrSetLdoCfg_outOfBounds_ldo2_rampTime(void);

/* Negative tests - LDO out of bounds tests for LDO3 */
void test_negative_Pmic_pwrSetLdoCfg_invalidParam_ldo3_mode(void);
void test_negative_Pmic_pwrSetLdoCfg_outOfBounds_ldo3_lvl(void);
void test_negative_Pmic_pwrSetLdoCfg_outOfBounds_ldo3_ilimLvl(void);
void test_negative_Pmic_pwrSetLdoCfg_outOfBounds_ldo3_ilimDgl(void);
void test_negative_Pmic_pwrSetLdoCfg_outOfBounds_ldo3_vmonThr(void);
void test_negative_Pmic_pwrSetLdoCfg_outOfBounds_ldo3_vmonDgl(void);
void test_negative_Pmic_pwrSetLdoCfg_outOfBounds_ldo3_rampTime(void);

/* Negative tests - LDO out of bounds tests for LDO4 */
void test_negative_Pmic_pwrSetLdoCfg_invalidParam_ldo4_mode(void);
void test_negative_Pmic_pwrSetLdoCfg_outOfBounds_ldo4_lvl(void);
void test_negative_Pmic_pwrSetLdoCfg_outOfBounds_ldo4_ilimLvl(void);
void test_negative_Pmic_pwrSetLdoCfg_outOfBounds_ldo4_ilimDgl(void);
void test_negative_Pmic_pwrSetLdoCfg_outOfBounds_ldo4_vmonThr(void);
void test_negative_Pmic_pwrSetLdoCfg_outOfBounds_ldo4_vmonDgl(void);
void test_negative_Pmic_pwrSetLdoCfg_outOfBounds_ldo4_rampTime(void);

/* Negative tests - PLDO Set/Get API NULL parameter tests */
void test_negative_Pmic_pwrSetPldoCfg_nullParam_handle(void);
void test_negative_Pmic_pwrSetPldoCfg_nullParam_config(void);
void test_negative_Pmic_pwrGetPldoCfg_nullParam_handle(void);
void test_negative_Pmic_pwrGetPldoCfg_nullParam_config(void);

/* Negative tests - PLDO out of bounds tests for PLDO1 */
void test_negative_Pmic_pwrSetPldoCfg_invalidParam_pldo1_mode(void);
void test_negative_Pmic_pwrSetPldoCfg_outOfBounds_pldo1_lvl(void);
void test_negative_Pmic_pwrSetPldoCfg_outOfBounds_pldo1_ilimLvl(void);
void test_negative_Pmic_pwrSetPldoCfg_outOfBounds_pldo1_ilimDgl(void);
void test_negative_Pmic_pwrSetPldoCfg_outOfBounds_pldo1_vmonThr(void);
void test_negative_Pmic_pwrSetPldoCfg_outOfBounds_pldo1_vmonDgl(void);
void test_negative_Pmic_pwrSetPldoCfg_outOfBounds_pldo1_vtrackRange(void);
void test_negative_Pmic_pwrSetPldoCfg_outOfBounds_pldo1_rampTime(void);

/* Negative tests - PLDO out of bounds tests for PLDO2 */
void test_negative_Pmic_pwrSetPldoCfg_invalidParam_pldo2_mode(void);
void test_negative_Pmic_pwrSetPldoCfg_outOfBounds_pldo2_lvl(void);
void test_negative_Pmic_pwrSetPldoCfg_outOfBounds_pldo2_ilimLvl(void);
void test_negative_Pmic_pwrSetPldoCfg_outOfBounds_pldo2_ilimDgl(void);
void test_negative_Pmic_pwrSetPldoCfg_outOfBounds_pldo2_vmonThr(void);
void test_negative_Pmic_pwrSetPldoCfg_outOfBounds_pldo2_vmonDgl(void);
void test_negative_Pmic_pwrSetPldoCfg_outOfBounds_pldo2_vtrackRange(void);
void test_negative_Pmic_pwrSetPldoCfg_outOfBounds_pldo2_rampTime(void);

/* Negative tests - ExtVmon Set/Get API NULL parameter tests */
void test_negative_Pmic_pwrSetExtVmonCfg_nullParam_handle(void);
void test_negative_Pmic_pwrSetExtVmonCfg_nullParam_config(void);
void test_negative_Pmic_pwrGetExtVmonCfg_nullParam_handle(void);
void test_negative_Pmic_pwrGetExtVmonCfg_nullParam_config(void);

/* Negative tests - ExtVmon out of bounds tests for VMON1 */
void test_negative_Pmic_pwrSetExtVmonCfg_invalidParam_vmon1_mode(void);
void test_negative_Pmic_pwrSetExtVmonCfg_outOfBounds_vmon1_vmonThr(void);
void test_negative_Pmic_pwrSetExtVmonCfg_outOfBounds_vmon1_vmonDgl(void);

/* Negative tests - ExtVmon out of bounds tests for VMON2 */
void test_negative_Pmic_pwrSetExtVmonCfg_invalidParam_vmon2_mode(void);
void test_negative_Pmic_pwrSetExtVmonCfg_outOfBounds_vmon2_vmonThr(void);
void test_negative_Pmic_pwrSetExtVmonCfg_outOfBounds_vmon2_vmonDgl(void);

/* Negative tests - Resource Status API tests */
void test_negative_Pmic_pwrGetRsrcStatus_nullParam_handle(void);
void test_negative_Pmic_pwrGetRsrcStatus_nullParam_status(void);
void test_negative_Pmic_pwrClrRsrcStatus_nullParam_handle(void);
void test_negative_Pmic_pwrClrRsrcStatus_nullParam_status(void);
void test_negative_Pmic_pwrClrRsrcStatusAll_nullParam_handle(void);

/* Malformed resource ID tests (type bits don't match resource) */
void test_negative_pwrGetRsrcStatus_malformedBbResource(void);
void test_negative_pwrGetRsrcStatus_malformedLdoResource(void);
void test_negative_pwrGetRsrcStatus_malformedPldoResource(void);
void test_negative_pwrGetRsrcStatus_malformedExtVmonResource(void);
void test_negative_pwrClrRsrcStatus_malformedBbResource(void);
void test_negative_pwrClrRsrcStatus_malformedLdoResource(void);
void test_negative_pwrClrRsrcStatus_malformedPldoResource(void);
void test_negative_pwrClrRsrcStatus_malformedExtVmonResource(void);

/* Negative tests - PGOOD API tests */
void test_negative_Pmic_pwrSetPGoodInStby_nullParam_handle(void);
void test_negative_Pmic_pwrGetPGoodInStby_nullParam_handle(void);
void test_negative_Pmic_pwrGetPGoodInStby_nullParam_isEnabled(void);

/* Positive tests - BuckBoost Set/Get configuration tests */
void test_positive_setGetBuckBoostCfg_lvl(void);
void test_positive_setGetBuckBoostCfg_stbyLvl(void);
void test_positive_setGetBuckBoostCfg_vmonThr(void);
void test_positive_setGetBuckBoostCfg_vmonDgl(void);
void test_positive_setGetBuckBoostCfg_boostTmo(void);
void test_positive_setGetBuckBoostCfg_ssEn(void);
void test_positive_setGetBuckBoostCfg_includeOvUvStatInPGood(void);
void test_positive_setGetBuckBoostCfg_allCfg(void);

/* Positive tests - LDO1 Set/Get configuration tests */
void test_positive_setGetLdoCfg_ldo1_mode(void);
void test_positive_setGetLdoCfg_ldo1_lvl(void);
void test_positive_setGetLdoCfg_ldo1_ilimLvl(void);
void test_positive_setGetLdoCfg_ldo1_ilimDgl(void);
void test_positive_setGetLdoCfg_ldo1_vmonThr(void);
void test_positive_setGetLdoCfg_ldo1_vmonDgl(void);
void test_positive_setGetLdoCfg_ldo1_rampTime(void);
void test_positive_setGetLdoCfg_ldo1_disableDischarge(void);
void test_positive_setGetLdoCfg_ldo1_includeOvUvStatInPGood(void);

/* Positive tests - LDO2 Set/Get configuration tests */
void test_positive_setGetLdoCfg_ldo2_mode(void);
void test_positive_setGetLdoCfg_ldo2_lvl(void);
void test_positive_setGetLdoCfg_ldo2_ilimLvl(void);
void test_positive_setGetLdoCfg_ldo2_ilimDgl(void);
void test_positive_setGetLdoCfg_ldo2_vmonThr(void);
void test_positive_setGetLdoCfg_ldo2_vmonDgl(void);
void test_positive_setGetLdoCfg_ldo2_rampTime(void);
void test_positive_setGetLdoCfg_ldo2_disableDischarge(void);
void test_positive_setGetLdoCfg_ldo2_includeOvUvStatInPGood(void);

/* Positive tests - LDO3 Set/Get configuration tests */
void test_positive_setGetLdoCfg_ldo3_mode(void);
void test_positive_setGetLdoCfg_ldo3_lvl(void);
void test_positive_setGetLdoCfg_ldo3_ilimLvl(void);
void test_positive_setGetLdoCfg_ldo3_ilimDgl(void);
void test_positive_setGetLdoCfg_ldo3_vmonThr(void);
void test_positive_setGetLdoCfg_ldo3_vmonDgl(void);
void test_positive_setGetLdoCfg_ldo3_rampTime(void);
void test_positive_setGetLdoCfg_ldo3_disableDischarge(void);
void test_positive_setGetLdoCfg_ldo3_includeOvUvStatInPGood(void);

/* Positive tests - LDO4 Set/Get configuration tests */
void test_positive_setGetLdoCfg_ldo4_mode(void);
void test_positive_setGetLdoCfg_ldo4_lvl(void);
void test_positive_setGetLdoCfg_ldo4_ilimLvl(void);
void test_positive_setGetLdoCfg_ldo4_ilimDgl(void);
void test_positive_setGetLdoCfg_ldo4_vmonThr(void);
void test_positive_setGetLdoCfg_ldo4_vmonDgl(void);
void test_positive_setGetLdoCfg_ldo4_rampTime(void);
void test_positive_setGetLdoCfg_ldo4_disableDischarge(void);
void test_positive_setGetLdoCfg_ldo4_includeOvUvStatInPGood(void);

/* Positive tests - PLDO1 Set/Get configuration tests */
void test_positive_setGetPldoCfg_pldo1_mode(void);
void test_positive_setGetPldoCfg_pldo1_trackingMode(void);
void test_positive_setGetPldoCfg_pldo1_lvl(void);
void test_positive_setGetPldoCfg_pldo1_ilimLvl(void);
void test_positive_setGetPldoCfg_pldo1_ilimDgl(void);
void test_positive_setGetPldoCfg_pldo1_vmonThr(void);
void test_positive_setGetPldoCfg_pldo1_vmonDgl(void);
void test_positive_setGetPldoCfg_pldo1_vtrackRange(void);
void test_positive_setGetPldoCfg_pldo1_rampTime(void);
void test_positive_setGetPldoCfg_pldo1_disableDischarge(void);
void test_positive_setGetPldoCfg_pldo1_includeOvUvStatInPGood(void);

/* Positive tests - PLDO2 Set/Get configuration tests */
void test_positive_setGetPldoCfg_pldo2_mode(void);
void test_positive_setGetPldoCfg_pldo2_trackingMode(void);
void test_positive_setGetPldoCfg_pldo2_lvl(void);
void test_positive_setGetPldoCfg_pldo2_ilimLvl(void);
void test_positive_setGetPldoCfg_pldo2_ilimDgl(void);
void test_positive_setGetPldoCfg_pldo2_vmonThr(void);
void test_positive_setGetPldoCfg_pldo2_vmonDgl(void);
void test_positive_setGetPldoCfg_pldo2_vtrackRange(void);
void test_positive_setGetPldoCfg_pldo2_rampTime(void);
void test_positive_setGetPldoCfg_pldo2_disableDischarge(void);
void test_positive_setGetPldoCfg_pldo2_includeOvUvStatInPGood(void);

/* Positive tests - ExtVmon1 Set/Get configuration tests */
void test_positive_setGetExtVmonCfg_vmon1_mode(void);
void test_positive_setGetExtVmonCfg_vmon1_vmonThr(void);
void test_positive_setGetExtVmonCfg_vmon1_vmonDgl(void);
void test_positive_setGetExtVmonCfg_vmon1_includeOvUvStatInPGood(void);

/* Positive tests - ExtVmon2 Set/Get configuration tests */
void test_positive_setGetExtVmonCfg_vmon2_mode(void);
void test_positive_setGetExtVmonCfg_vmon2_vmonThr(void);
void test_positive_setGetExtVmonCfg_vmon2_vmonDgl(void);
void test_positive_setGetExtVmonCfg_vmon2_includeOvUvStatInPGood(void);

/* Positive tests - Resource Status tests */
void test_positive_getRsrcStatus_buckBoost(void);
void test_positive_getRsrcStatus_ldo1(void);
void test_positive_getRsrcStatus_ldo2(void);
void test_positive_getRsrcStatus_ldo3(void);
void test_positive_getRsrcStatus_ldo4(void);
void test_positive_getRsrcStatus_pldo1(void);
void test_positive_getRsrcStatus_pldo2(void);
void test_positive_getRsrcStatus_extVmon1(void);
void test_positive_getRsrcStatus_extVmon2(void);
void test_positive_clrRsrcStatus_buckBoost(void);
void test_positive_clrRsrcStatusAll(void);

/* Positive tests - PGOOD tests */
void test_positive_setGetPGoodInStby(void);

/* Positive tests - Comprehensive tests */
void test_positive_setGetLdoCfg_allLdos_allCfg(void);
void test_positive_setGetPldoCfg_allPldos_allCfg(void);
void test_positive_setGetExtVmonCfg_allVmons_allCfg(void);

/* Negative tests - Resource status invalid type tests */
void test_negative_pwrGetRsrcStatus_invalidResourceType(void);
void test_negative_pwrClrRsrcStatus_invalidResourceType(void);

/* Positive tests - Edge case tests */
void test_positive_pwrGetPldoCfg_redundantModeConversion(void);

/* Negative tests - Zero validParams tests */
void test_negative_pwrSetBuckBoostCfg_zeroValidParams(void);
void test_negative_pwrGetBuckBoostCfg_zeroValidParams(void);

/* Negative tests - Invalid resource ID tests */
void test_negative_pwrSetLdoCfg_invalidLdoId(void);
void test_negative_pwrGetLdoCfg_invalidLdoId(void);
void test_negative_pwrSetPldoCfg_invalidPldoId(void);
void test_negative_pwrGetPldoCfg_invalidPldoId(void);
void test_negative_pwrSetExtVmonCfg_invalidExtVmonId(void);
void test_negative_pwrGetExtVmonCfg_invalidExtVmonId(void);

/* Positive tests - BuckBoost status GET tests */
void test_positive_getRsrcStatus_buckBoost_bbLite(void);
void test_positive_getRsrcStatus_buckBoost_bbIlimLvl(void);
void test_positive_getRsrcStatus_buckBoost_bbMode(void);
void test_positive_getRsrcStatus_buckBoost_ovErr(void);
void test_positive_getRsrcStatus_buckBoost_uvErr(void);
void test_positive_getRsrcStatus_buckBoost_tsdErr(void);
void test_positive_getRsrcStatus_buckBoost_tsdWarn(void);

/* Negative tests - LDO status GET unsupported params */
void test_negative_getRsrcStatus_ldo_unsupportedBbLite(void);
void test_negative_getRsrcStatus_ldo_unsupportedBbIlimLvl(void);
void test_negative_getRsrcStatus_ldo_unsupportedBbMode(void);

/* Positive tests - LDO status GET tests */
void test_positive_getRsrcStatus_ldo_uvErr(void);
void test_positive_getRsrcStatus_ldo_ovErr(void);
void test_positive_getRsrcStatus_ldo_tsdErr(void);
void test_positive_getRsrcStatus_ldo_tsdWarn(void);

/* Negative tests - PLDO status GET unsupported params */
void test_negative_getRsrcStatus_pldo_unsupportedBbLite(void);

/* Positive tests - PLDO status GET tests */
void test_positive_getRsrcStatus_pldo_uvErr(void);
void test_positive_getRsrcStatus_pldo_ovErr(void);
void test_positive_getRsrcStatus_pldo_tsdErr(void);
void test_positive_getRsrcStatus_pldo_tsdWarn(void);

/* Negative tests - ExtVmon status GET unsupported params */
void test_negative_getRsrcStatus_extVmon_unsupportedIlimErr(void);
void test_negative_getRsrcStatus_extVmon_unsupportedTsdErr(void);

/* Positive tests - ExtVmon status GET tests */
void test_positive_getRsrcStatus_extVmon_uvErr(void);
void test_positive_getRsrcStatus_extVmon_ovErr(void);
void test_negative_getRsrcStatus_extVmon_unsupportedTsdWarn(void);

/* Positive tests - BuckBoost status CLEAR tests */
void test_positive_clrRsrcStatus_buckBoost_bbMode(void);
void test_positive_clrRsrcStatus_buckBoost_ilimErr(void);
void test_positive_clrRsrcStatus_buckBoost_tsdErr(void);
void test_positive_clrRsrcStatus_buckBoost_tsdWarn(void);

/* Positive tests - LDO status CLEAR tests */
void test_positive_clrRsrcStatus_ldo1_uvErr(void);
void test_positive_clrRsrcStatus_ldo2_ovErr(void);
void test_positive_clrRsrcStatus_ldo3_tsdErr(void);
void test_positive_clrRsrcStatus_ldo4_tsdWarn(void);
void test_positive_clrRsrcStatus_ldo_allStatus(void);

/* Positive tests - PLDO status CLEAR tests */
void test_positive_clrRsrcStatus_pldo1_uvErr(void);
void test_positive_clrRsrcStatus_pldo2_ovErr(void);
void test_positive_clrRsrcStatus_pldo_tsdErr(void);
void test_positive_clrRsrcStatus_pldo_allStatus(void);

/* Positive tests - ExtVmon status CLEAR tests */
void test_positive_clrRsrcStatus_extVmon1_uvErr(void);
void test_positive_clrRsrcStatus_extVmon2_ovErr(void);
void test_negative_clrRsrcStatus_extVmon_unsupportedTsdWarn(void);
void test_positive_clrRsrcStatus_extVmon_allStatus(void);

/* Coverage tests for pmic_power.c */
void test_positive_pwr_getPldoMode_disabledFallback(void);
void test_negative_pwr_clrLdoStat_unsupportedBbParams(void);
void test_negative_pwr_clrPldoStat_unsupportedBbParams(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /*__POWER_TEST_H__*/
