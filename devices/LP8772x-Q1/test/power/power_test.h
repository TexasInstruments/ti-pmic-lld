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
#ifndef POWER_TEST_H
#define POWER_TEST_H



/* ========================================================================== */
/*                              Include Files                                 */
/* ========================================================================== */

#include "platform.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/* ======================================================================== */
/*                     Test APIs: pwrSetResourceEnable                      */
/* ======================================================================== */

#define POWER_TEST_POS_PWRSETRESOURCEENABLE() \
    PLATFORM_RUN_TEST(test_pos_power_enableDisable_buck1); \
    PLATFORM_RUN_TEST(test_pos_power_enableDisable_buck2); \
    PLATFORM_RUN_TEST(test_pos_power_enableDisable_buck3); \
    PLATFORM_RUN_TEST(test_pos_power_enableDisable_gpo); \
    PLATFORM_RUN_TEST(test_pos_power_enableDisable_ldoLs1Vmon1); \
    PLATFORM_RUN_TEST(test_pos_power_enableDisable_ls2Vmon2); \
    PLATFORM_RUN_TEST(test_pos_power_enableDisable_vccaVmon)

#define POWER_TEST_NEG_PWRSETRESOURCEENABLE() \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceEnable_nullParam_handle); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceEnable_outOfBounds_resource)

/* Test: TC-POWER-0021 */
#define POWER_TEST_PWRSETRESOURCEENABLE() \
    POWER_TEST_POS_PWRSETRESOURCEENABLE(); \
    POWER_TEST_NEG_PWRSETRESOURCEENABLE()

/* ======================================================================== */
/*                     Test APIs: pwrGetResourceEnable                      */
/* ======================================================================== */

#define POWER_TEST_NEG_PWRGETRESOURCEENABLE() \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetResourceEnable_nullParam_handle); \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetResourceEnable_nullParam_isEnabled); \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetResourceEnable_outOfBounds_resource)

/* Test: TC-POWER-0022 */
#define POWER_TEST_PWRGETRESOURCEENABLE() \
    POWER_TEST_NEG_PWRGETRESOURCEENABLE()

/* ======================================================================== */
/*   Test APIs: pwrSetResourceCfg, pwrGetResourceCfg, pwrSetResourceCfgs,   */
/*              pwrGetResourceCfgs                                          */
/* ======================================================================== */

#define POWER_TEST_POS_PWRSETRESOURCECFG() \
    PLATFORM_RUN_TEST(test_pos_power_setGetResourceCfg_allRsrc_allCfg); \
    PLATFORM_RUN_TEST(test_pos_power_setGetResourceCfg_buck1_deglitch); \
    PLATFORM_RUN_TEST(test_pos_power_setGetResourceCfg_buck1_enable); \
    PLATFORM_RUN_TEST(test_pos_power_setGetResourceCfg_buck1_ilim); \
    PLATFORM_RUN_TEST(test_pos_power_setGetResourceCfg_buck1_mode); \
    PLATFORM_RUN_TEST(test_pos_power_setGetResourceCfg_buck1_ovReaction); \
    PLATFORM_RUN_TEST(test_pos_power_setGetResourceCfg_buck1_ovThresh); \
    PLATFORM_RUN_TEST(test_pos_power_setGetResourceCfg_buck1_rvReaction); \
    PLATFORM_RUN_TEST(test_pos_power_setGetResourceCfg_buck1_scReaction); \
    PLATFORM_RUN_TEST(test_pos_power_setGetResourceCfg_buck1_uvReaction); \
    PLATFORM_RUN_TEST(test_pos_power_setGetResourceCfg_buck1_uvThresh); \
    PLATFORM_RUN_TEST(test_pos_power_setGetResourceCfg_buck1_voltage_mV); \
    PLATFORM_RUN_TEST(test_pos_power_setGetResourceCfg_buck2_deglitch); \
    PLATFORM_RUN_TEST(test_pos_power_setGetResourceCfg_buck2_enable); \
    PLATFORM_RUN_TEST(test_pos_power_setGetResourceCfg_buck2_ilim); \
    PLATFORM_RUN_TEST(test_pos_power_setGetResourceCfg_buck2_mode); \
    PLATFORM_RUN_TEST(test_pos_power_setGetResourceCfg_buck2_ovReaction); \
    PLATFORM_RUN_TEST(test_pos_power_setGetResourceCfg_buck2_ovThresh); \
    PLATFORM_RUN_TEST(test_pos_power_setGetResourceCfg_buck2_rvReaction); \
    PLATFORM_RUN_TEST(test_pos_power_setGetResourceCfg_buck2_scReaction); \
    PLATFORM_RUN_TEST(test_pos_power_setGetResourceCfg_buck2_uvReaction); \
    PLATFORM_RUN_TEST(test_pos_power_setGetResourceCfg_buck2_uvThresh); \
    PLATFORM_RUN_TEST(test_pos_power_setGetResourceCfg_buck2_voltage_mV); \
    PLATFORM_RUN_TEST(test_pos_power_setGetResourceCfg_buck3_deglitch); \
    PLATFORM_RUN_TEST(test_pos_power_setGetResourceCfg_buck3_enable); \
    PLATFORM_RUN_TEST(test_pos_power_setGetResourceCfg_buck3_ilim); \
    PLATFORM_RUN_TEST(test_pos_power_setGetResourceCfg_buck3_mode); \
    PLATFORM_RUN_TEST(test_pos_power_setGetResourceCfg_buck3_ovReaction); \
    PLATFORM_RUN_TEST(test_pos_power_setGetResourceCfg_buck3_ovThresh); \
    PLATFORM_RUN_TEST(test_pos_power_setGetResourceCfg_buck3_rvReaction); \
    PLATFORM_RUN_TEST(test_pos_power_setGetResourceCfg_buck3_scReaction); \
    PLATFORM_RUN_TEST(test_pos_power_setGetResourceCfg_buck3_uvReaction); \
    PLATFORM_RUN_TEST(test_pos_power_setGetResourceCfg_buck3_uvThresh); \
    PLATFORM_RUN_TEST(test_pos_power_setGetResourceCfg_buck3_voltage_mV); \
    PLATFORM_RUN_TEST(test_pos_power_setGetResourceCfg_gpo_enable); \
    PLATFORM_RUN_TEST(test_pos_power_setGetResourceCfg_ldoLs1Vmon1_deglitch); \
    PLATFORM_RUN_TEST(test_pos_power_setGetResourceCfg_ldoLs1Vmon1_enable); \
    PLATFORM_RUN_TEST(test_pos_power_setGetResourceCfg_ldoLs1Vmon1_mode); \
    PLATFORM_RUN_TEST(test_pos_power_setGetResourceCfg_ldoLs1Vmon1_ovReaction); \
    PLATFORM_RUN_TEST(test_pos_power_setGetResourceCfg_ldoLs1Vmon1_ovThresh); \
    PLATFORM_RUN_TEST(test_pos_power_setGetResourceCfg_ldoLs1Vmon1_rvReaction); \
    PLATFORM_RUN_TEST(test_pos_power_setGetResourceCfg_ldoLs1Vmon1_scReaction); \
    PLATFORM_RUN_TEST(test_pos_power_setGetResourceCfg_ldoLs1Vmon1_uvReaction); \
    PLATFORM_RUN_TEST(test_pos_power_setGetResourceCfg_ldoLs1Vmon1_uvThresh); \
    PLATFORM_RUN_TEST(test_pos_power_setGetResourceCfg_ldoLs1Vmon1_voltage_mV); \
    PLATFORM_RUN_TEST(test_pos_power_setGetResourceCfg_ls2Vmon2_deglitch); \
    PLATFORM_RUN_TEST(test_pos_power_setGetResourceCfg_ls2Vmon2_enable); \
    PLATFORM_RUN_TEST(test_pos_power_setGetResourceCfg_ls2Vmon2_mode); \
    PLATFORM_RUN_TEST(test_pos_power_setGetResourceCfg_ls2Vmon2_ovReaction); \
    PLATFORM_RUN_TEST(test_pos_power_setGetResourceCfg_ls2Vmon2_ovThresh); \
    PLATFORM_RUN_TEST(test_pos_power_setGetResourceCfg_ls2Vmon2_rvReaction); \
    PLATFORM_RUN_TEST(test_pos_power_setGetResourceCfg_ls2Vmon2_scReaction); \
    PLATFORM_RUN_TEST(test_pos_power_setGetResourceCfg_ls2Vmon2_uvReaction); \
    PLATFORM_RUN_TEST(test_pos_power_setGetResourceCfg_ls2Vmon2_uvThresh); \
    PLATFORM_RUN_TEST(test_pos_power_setGetResourceCfg_ls2Vmon2_voltage_mV); \
    PLATFORM_RUN_TEST(test_pos_power_setGetResourceCfg_vccaVmon_deglitch); \
    PLATFORM_RUN_TEST(test_pos_power_setGetResourceCfg_vccaVmon_enable); \
    PLATFORM_RUN_TEST(test_pos_power_setGetResourceCfg_vccaVmon_mode); \
    PLATFORM_RUN_TEST(test_pos_power_setGetResourceCfg_vccaVmon_ovReaction); \
    PLATFORM_RUN_TEST(test_pos_power_setGetResourceCfg_vccaVmon_ovThresh); \
    PLATFORM_RUN_TEST(test_pos_power_setGetResourceCfg_vccaVmon_uvReaction); \
    PLATFORM_RUN_TEST(test_pos_power_setGetResourceCfg_vccaVmon_uvThresh); \
    PLATFORM_RUN_TEST(test_pos_power_setGetResourceCfg_vccaVmon_voltage_mV); \
    PLATFORM_RUN_TEST(test_pos_power_setGetResourceCfg_buck1_enable_mcdc); \
    PLATFORM_RUN_TEST(test_pos_power_powerSetPgoodLevel_validBuck)

#define POWER_TEST_NEG_PWRSETRESOURCECFG() \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfg_gpo_deglitch); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfg_gpo_ilim); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfg_gpo_mode); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfg_gpo_ovReaction); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfg_gpo_ovThresh); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfg_gpo_rvReaction); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfg_gpo_scReaction); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfg_gpo_uvReaction); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfg_gpo_uvThresh); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfg_gpo_voltage_mV); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfg_invalidParam_buck1_mode); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfg_invalidParam_buck2_mode); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfg_invalidParam_buck3_mode); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfg_invalidParam_ls2Vmon2_mode); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfg_invalidParam_vccaVmon_mode); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfg_ldoLs1Vmon1_ilim); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfg_ls2Vmon2_ilim); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfg_nullParam_config); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfg_nullParam_handle); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfg_outOfBounds_buck1_deglitch); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfg_outOfBounds_buck1_ilim); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfg_outOfBounds_buck1_ovReaction); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfg_outOfBounds_buck1_ovThresh); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfg_outOfBounds_buck1_rvReaction); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfg_outOfBounds_buck1_scReaction); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfg_outOfBounds_buck1_uvReaction); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfg_outOfBounds_buck1_uvThresh); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfg_outOfBounds_buck1_voltage_mV); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfg_outOfBounds_buck2_deglitch); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfg_outOfBounds_buck2_ilim); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfg_outOfBounds_buck2_ovReaction); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfg_outOfBounds_buck2_ovThresh); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfg_outOfBounds_buck2_rvReaction); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfg_outOfBounds_buck2_scReaction); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfg_outOfBounds_buck2_uvReaction); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfg_outOfBounds_buck2_uvThresh); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfg_outOfBounds_buck2_voltage_mV); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfg_outOfBounds_buck3_deglitch); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfg_outOfBounds_buck3_ilim); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfg_outOfBounds_buck3_ovReaction); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfg_outOfBounds_buck3_ovThresh); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfg_outOfBounds_buck3_rvReaction); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfg_outOfBounds_buck3_scReaction); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfg_outOfBounds_buck3_uvReaction); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfg_outOfBounds_buck3_uvThresh); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfg_outOfBounds_buck3_voltage_mV); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfg_outOfBounds_ldoLs1Vmon1_deglitch); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfg_outOfBounds_ldoLs1Vmon1_mode); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfg_outOfBounds_ldoLs1Vmon1_ovReaction); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfg_outOfBounds_ldoLs1Vmon1_ovThresh); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfg_outOfBounds_ldoLs1Vmon1_rvReaction); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfg_outOfBounds_ldoLs1Vmon1_scReaction); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfg_outOfBounds_ldoLs1Vmon1_uvReaction); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfg_outOfBounds_ldoLs1Vmon1_uvThresh); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfg_outOfBounds_ldoLs1Vmon1_voltage_mV); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfg_outOfBounds_ls2Vmon2_deglitch); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfg_outOfBounds_ls2Vmon2_ovReaction); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfg_outOfBounds_ls2Vmon2_ovThresh); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfg_outOfBounds_ls2Vmon2_rvReaction); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfg_outOfBounds_ls2Vmon2_scReaction); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfg_outOfBounds_ls2Vmon2_uvReaction); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfg_outOfBounds_ls2Vmon2_uvThresh); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfg_outOfBounds_ls2Vmon2_voltage_mV); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfg_outOfBounds_resource); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfg_outOfBounds_vccaVmon_deglitch); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfg_outOfBounds_vccaVmon_ovReaction); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfg_outOfBounds_vccaVmon_ovThresh); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfg_outOfBounds_vccaVmon_uvReaction); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfg_outOfBounds_vccaVmon_uvThresh); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfg_outOfBounds_vccaVmon_voltage_mV); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfg_vccaVmon_ilim); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfg_vccaVmon_rvReaction); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfg_vccaVmon_scReaction); \
    PLATFORM_RUN_TEST(test_neg_power_powerSetVoutCfg_unsupportedRegulator); \
    PLATFORM_RUN_TEST(test_neg_power_pwr_setIlimCfg_resourceOutOfBounds); \
    PLATFORM_RUN_TEST(test_neg_power_setPgLevel_invalidResource); \
    PLATFORM_RUN_TEST(test_neg_power_setVoltage_invalidResource)

/* Test: TC-POWER-0023 */
#define POWER_TEST_PWRSETRESOURCECFG() \
    POWER_TEST_POS_PWRSETRESOURCECFG(); \
    POWER_TEST_NEG_PWRSETRESOURCECFG()

/* ======================================================================== */
/*                       Test APIs: pwrGetResourceCfg                       */
/* ======================================================================== */

#define POWER_TEST_POS_PWRGETRESOURCECFG() \
    PLATFORM_RUN_TEST(test_pos_power_powerGetPgoodLevel_validBuck)

#define POWER_TEST_NEG_PWRGETRESOURCECFG() \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetResourceCfg_gpo_unsupportedDeglitch); \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetResourceCfg_gpo_unsupportedIlim); \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetResourceCfg_gpo_unsupportedOvReaction); \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetResourceCfg_gpo_unsupportedOvThresh); \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetResourceCfg_gpo_unsupportedRvReaction); \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetResourceCfg_gpo_unsupportedScReaction); \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetResourceCfg_gpo_unsupportedUvReaction); \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetResourceCfg_gpo_unsupportedUvThresh); \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetResourceCfg_ldoLs1Vmon1_invalidHwState); \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetResourceCfg_nullParam_config); \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetResourceCfg_nullParam_handle); \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetResourceCfg_outOfBounds_resource); \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetResourceCfg_vccaVmon_unsupportedIlim); \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetResourceCfg_vccaVmon_unsupportedRvReaction); \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetResourceCfg_vccaVmon_unsupportedScReaction); \
    PLATFORM_RUN_TEST(test_neg_power_powerGetVoutCfg_unsupportedRegulator); \
    PLATFORM_RUN_TEST(test_neg_power_pwr_getIlimCfg_resourceOutOfBounds); \
    PLATFORM_RUN_TEST(test_neg_power_pwr_getModeCfg_unsupportedResource); \
    PLATFORM_RUN_TEST(test_neg_power_pwr_getVoltageCfg_resourceOutOfBounds); \
    PLATFORM_RUN_TEST(test_neg_power_pwr_invalidModeCombination_ldoLs1Vmon1)

/* Test: TC-POWER-0024 */
#define POWER_TEST_PWRGETRESOURCECFG() \
    POWER_TEST_POS_PWRGETRESOURCECFG(); \
    POWER_TEST_NEG_PWRGETRESOURCECFG()

/* ======================================================================== */
/*                      Test APIs: pwrSetResourceCfgs                       */
/* ======================================================================== */

#define POWER_TEST_NEG_PWRSETRESOURCECFGS() \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfgs_gpo_deglitch); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfgs_gpo_ilim); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfgs_gpo_mode); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfgs_gpo_ovReaction); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfgs_gpo_ovThresh); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfgs_gpo_rvReaction); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfgs_gpo_scReaction); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfgs_gpo_uvReaction); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfgs_gpo_uvThresh); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfgs_gpo_voltage_mV); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfgs_invalidParam_buck1_mode); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfgs_invalidParam_buck2_mode); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfgs_invalidParam_buck3_mode); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfgs_invalidParam_ls2Vmon2_mode); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfgs_invalidParam_vccaVmon_mode); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfgs_ldoLs1Vmon1_ilim); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfgs_ls2Vmon2_ilim); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfgs_nullParam_config); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfgs_nullParam_handle); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfgs_outOfBounds_buck1_deglitch); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfgs_outOfBounds_buck1_ilim); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfgs_outOfBounds_buck1_ovReaction); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfgs_outOfBounds_buck1_ovThresh); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfgs_outOfBounds_buck1_rvReaction); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfgs_outOfBounds_buck1_scReaction); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfgs_outOfBounds_buck1_uvReaction); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfgs_outOfBounds_buck1_uvThresh); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfgs_outOfBounds_buck1_voltage_mV); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfgs_outOfBounds_buck2_deglitch); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfgs_outOfBounds_buck2_ilim); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfgs_outOfBounds_buck2_ovReaction); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfgs_outOfBounds_buck2_ovThresh); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfgs_outOfBounds_buck2_rvReaction); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfgs_outOfBounds_buck2_scReaction); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfgs_outOfBounds_buck2_uvReaction); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfgs_outOfBounds_buck2_uvThresh); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfgs_outOfBounds_buck2_voltage_mV); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfgs_outOfBounds_buck3_deglitch); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfgs_outOfBounds_buck3_ilim); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfgs_outOfBounds_buck3_ovReaction); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfgs_outOfBounds_buck3_ovThresh); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfgs_outOfBounds_buck3_rvReaction); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfgs_outOfBounds_buck3_scReaction); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfgs_outOfBounds_buck3_uvReaction); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfgs_outOfBounds_buck3_uvThresh); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfgs_outOfBounds_buck3_voltage_mV); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfgs_outOfBounds_ldoLs1Vmon1_deglitch); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfgs_outOfBounds_ldoLs1Vmon1_mode); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfgs_outOfBounds_ldoLs1Vmon1_ovReaction); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfgs_outOfBounds_ldoLs1Vmon1_ovThresh); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfgs_outOfBounds_ldoLs1Vmon1_rvReaction); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfgs_outOfBounds_ldoLs1Vmon1_scReaction); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfgs_outOfBounds_ldoLs1Vmon1_uvReaction); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfgs_outOfBounds_ldoLs1Vmon1_uvThresh); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfgs_outOfBounds_ldoLs1Vmon1_voltage_mV); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfgs_outOfBounds_ls2Vmon2_deglitch); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfgs_outOfBounds_ls2Vmon2_ovReaction); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfgs_outOfBounds_ls2Vmon2_ovThresh); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfgs_outOfBounds_ls2Vmon2_rvReaction); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfgs_outOfBounds_ls2Vmon2_scReaction); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfgs_outOfBounds_ls2Vmon2_uvReaction); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfgs_outOfBounds_ls2Vmon2_uvThresh); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfgs_outOfBounds_ls2Vmon2_voltage_mV); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfgs_outOfBounds_resource); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfgs_outOfBounds_vccaVmon_deglitch); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfgs_outOfBounds_vccaVmon_ovReaction); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfgs_outOfBounds_vccaVmon_ovThresh); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfgs_outOfBounds_vccaVmon_uvReaction); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfgs_outOfBounds_vccaVmon_uvThresh); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfgs_outOfBounds_vccaVmon_voltage_mV); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfgs_vccaVmon_ilim); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfgs_vccaVmon_rvReaction); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfgs_vccaVmon_scReaction); \
    PLATFORM_RUN_TEST(test_neg_power_pwr_setResourceCfg_excessiveNumConfigs); \
    PLATFORM_RUN_TEST(test_neg_power_pwr_setResourceCfg_zeroNumConfigs); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfgs_errorMidBatch_partialApply)

/* Test: TC-POWER-0025 */
#define POWER_TEST_PWRSETRESOURCECFGS() \
    POWER_TEST_NEG_PWRSETRESOURCECFGS()

/* ======================================================================== */
/*                      Test APIs: pwrGetResourceCfgs                       */
/* ======================================================================== */

#define POWER_TEST_NEG_PWRGETRESOURCECFGS() \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetResourceCfgs_nullParam_config); \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetResourceCfgs_nullParam_handle); \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetResourceCfgs_outOfBounds_resource); \
    PLATFORM_RUN_TEST(test_neg_power_pwr_getResourceCfgs_zeroNumConfigs); \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetResourceCfgs_errorMidBatch_outputUnchanged)

/* Test: TC-POWER-0026 */
#define POWER_TEST_PWRGETRESOURCECFGS() \
    POWER_TEST_NEG_PWRGETRESOURCECFGS()

/* ======================================================================== */
/*   Test APIs: pwrSetSequenceCfg, pwrGetSequenceCfg, pwrSetSequenceCfgs,   */
/*              pwrGetSequenceCfgs                                          */
/* ======================================================================== */

#define POWER_TEST_POS_PWRSETSEQUENCECFG() \
    PLATFORM_RUN_TEST(test_pos_power_setGetSequenceCfg_allRsrc_allCfg); \
    PLATFORM_RUN_TEST(test_pos_power_setGetSequenceCfg_buck1_shutdownDelay); \
    PLATFORM_RUN_TEST(test_pos_power_setGetSequenceCfg_buck1_startupDelay); \
    PLATFORM_RUN_TEST(test_pos_power_setGetSequenceCfg_buck2_shutdownDelay); \
    PLATFORM_RUN_TEST(test_pos_power_setGetSequenceCfg_buck2_startupDelay); \
    PLATFORM_RUN_TEST(test_pos_power_setGetSequenceCfg_buck3_shutdownDelay); \
    PLATFORM_RUN_TEST(test_pos_power_setGetSequenceCfg_buck3_startupDelay); \
    PLATFORM_RUN_TEST(test_pos_power_setGetSequenceCfg_gpo_shutdownDelay); \
    PLATFORM_RUN_TEST(test_pos_power_setGetSequenceCfg_gpo_startupDelay); \
    PLATFORM_RUN_TEST(test_pos_power_setGetSequenceCfg_ldoLs1Vmon1_shutdownDelay); \
    PLATFORM_RUN_TEST(test_pos_power_setGetSequenceCfg_ldoLs1Vmon1_startupDelay); \
    PLATFORM_RUN_TEST(test_pos_power_setGetSequenceCfg_ls2Vmon2_shutdownDelay); \
    PLATFORM_RUN_TEST(test_pos_power_setGetSequenceCfg_ls2Vmon2_startupDelay)

#define POWER_TEST_NEG_PWRSETSEQUENCECFG() \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetSequenceCfg_nullParam_config); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetSequenceCfg_nullParam_handle); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetSequenceCfg_outOfBounds_buck1_shutdownDelay); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetSequenceCfg_outOfBounds_buck1_startupDelay); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetSequenceCfg_outOfBounds_buck2_shutdownDelay); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetSequenceCfg_outOfBounds_buck2_startupDelay); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetSequenceCfg_outOfBounds_buck3_shutdownDelay); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetSequenceCfg_outOfBounds_buck3_startupDelay); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetSequenceCfg_outOfBounds_gpo_shutdownDelay); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetSequenceCfg_outOfBounds_gpo_startupDelay); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetSequenceCfg_outOfBounds_ldoLs1Vmon1_shutdownDelay); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetSequenceCfg_outOfBounds_ldoLs1Vmon1_startupDelay); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetSequenceCfg_outOfBounds_ls2Vmon2_shutdownDelay); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetSequenceCfg_outOfBounds_ls2Vmon2_startupDelay); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetSequenceCfg_outOfBounds_resource); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetSequenceCfg_vccaVmon_shutdownDelay); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetSequenceCfg_vccaVmon_startupDelay)

/* Test: TC-POWER-0027 */
#define POWER_TEST_PWRSETSEQUENCECFG() \
    POWER_TEST_POS_PWRSETSEQUENCECFG(); \
    POWER_TEST_NEG_PWRSETSEQUENCECFG()

/* ======================================================================== */
/*                       Test APIs: pwrGetSequenceCfg                       */
/* ======================================================================== */

#define POWER_TEST_POS_PWRGETSEQUENCECFG() \
    PLATFORM_RUN_TEST(test_pos_power_getNrstoutSequence)

#define POWER_TEST_NEG_PWRGETSEQUENCECFG() \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetSequenceCfg_nullParam_config); \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetSequenceCfg_nullParam_handle)

/* Test: TC-POWER-0028 */
#define POWER_TEST_PWRGETSEQUENCECFG() \
    POWER_TEST_POS_PWRGETSEQUENCECFG(); \
    POWER_TEST_NEG_PWRGETSEQUENCECFG()

/* ======================================================================== */
/*                      Test APIs: pwrSetSequenceCfgs                       */
/* ======================================================================== */

#define POWER_TEST_NEG_PWRSETSEQUENCECFGS() \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetSequenceCfgs_nullParam_config); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetSequenceCfgs_nullParam_handle); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetSequenceCfgs_numConfigs_zero); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetSequenceCfgs_outOfBounds_buck1_shutdownDelay); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetSequenceCfgs_outOfBounds_buck1_startupDelay); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetSequenceCfgs_outOfBounds_buck2_shutdownDelay); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetSequenceCfgs_outOfBounds_buck2_startupDelay); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetSequenceCfgs_outOfBounds_buck3_shutdownDelay); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetSequenceCfgs_outOfBounds_buck3_startupDelay); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetSequenceCfgs_outOfBounds_gpo_shutdownDelay); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetSequenceCfgs_outOfBounds_gpo_startupDelay); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetSequenceCfgs_outOfBounds_ldoLs1Vmon1_shutdownDelay); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetSequenceCfgs_outOfBounds_ldoLs1Vmon1_startupDelay); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetSequenceCfgs_outOfBounds_ls2Vmon2_shutdownDelay); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetSequenceCfgs_outOfBounds_ls2Vmon2_startupDelay); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetSequenceCfgs_outOfBounds_resource); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetSequenceCfgs_vccaVmon_shutdownDelay); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetSequenceCfgs_vccaVmon_startupDelay); \
    PLATFORM_RUN_TEST(test_neg_power_pwrSetSequenceCfgs_errorMidBatch_partialApply)

/* Test: TC-POWER-0029 */
#define POWER_TEST_PWRSETSEQUENCECFGS() \
    POWER_TEST_NEG_PWRSETSEQUENCECFGS()

/* ======================================================================== */
/*                      Test APIs: pwrGetSequenceCfgs                       */
/* ======================================================================== */

#define POWER_TEST_NEG_PWRGETSEQUENCECFGS() \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetSequenceCfgs_nullParam_config); \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetSequenceCfgs_nullParam_handle); \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetSequenceCfgs_numConfigs_zero); \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetSequenceCfgs_outOfBounds_resource); \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetSequenceCfgs_errorMidBatch_outputUnchanged)

/* Test: TC-POWER-0030 */
#define POWER_TEST_PWRGETSEQUENCECFGS() \
    POWER_TEST_NEG_PWRGETSEQUENCECFGS()

/* ======================================================================== */
/*       Test APIs: pwrSetThermalCfg, pwrGetThermalCfg                     */
/* ======================================================================== */

#define POWER_TEST_POS_PWRSETTHERMALCFG() \
    PLATFORM_RUN_TEST(test_pos_power_thermal_twarnLvl); \
    PLATFORM_RUN_TEST(test_pos_power_thermal_tsdOrdLvl); \
    PLATFORM_RUN_TEST(test_pos_power_thermal_twarnConfig); \
    PLATFORM_RUN_TEST(test_pos_power_thermal_all_params)

#define POWER_TEST_NEG_PWRSETTHERMALCFG() \
    PLATFORM_RUN_TEST(test_neg_power_thermal_null_handle); \
    PLATFORM_RUN_TEST(test_neg_power_thermal_null_param); \
    PLATFORM_RUN_TEST(test_neg_power_thermal_invalid_twarnLvl); \
    PLATFORM_RUN_TEST(test_neg_power_thermal_invalid_tsdOrdLvl); \
    PLATFORM_RUN_TEST(test_neg_power_thermal_invalid_twarnConfig); \
    PLATFORM_RUN_TEST(test_neg_power_thermal_no_valid_params)

/* Test: TC-POWER-0031 */
#define POWER_TEST_PWRSETTHERMALCFG() \
    POWER_TEST_POS_PWRSETTHERMALCFG(); \
    POWER_TEST_NEG_PWRSETTHERMALCFG()

/* ========================================================================== */
/*                         Aggregate Test Runners                             */
/* ========================================================================== */

#define POWER_TEST_RUN_POSITIVE() \
    POWER_TEST_POS_PWRSETRESOURCEENABLE(); \
    POWER_TEST_POS_PWRSETRESOURCECFG(); \
    POWER_TEST_POS_PWRGETRESOURCECFG(); \
    POWER_TEST_POS_PWRSETSEQUENCECFG(); \
    POWER_TEST_POS_PWRGETSEQUENCECFG(); \
    POWER_TEST_POS_PWRSETTHERMALCFG()

#define POWER_TEST_RUN_NEGATIVE() \
    POWER_TEST_NEG_PWRSETRESOURCEENABLE(); \
    POWER_TEST_NEG_PWRGETRESOURCEENABLE(); \
    POWER_TEST_NEG_PWRSETRESOURCECFG(); \
    POWER_TEST_NEG_PWRGETRESOURCECFG(); \
    POWER_TEST_NEG_PWRSETRESOURCECFGS(); \
    POWER_TEST_NEG_PWRGETRESOURCECFGS(); \
    POWER_TEST_NEG_PWRSETSEQUENCECFG(); \
    POWER_TEST_NEG_PWRGETSEQUENCECFG(); \
    POWER_TEST_NEG_PWRSETSEQUENCECFGS(); \
    POWER_TEST_NEG_PWRGETSEQUENCECFGS(); \
    POWER_TEST_NEG_PWRSETTHERMALCFG()

#define POWER_TEST_RUN_ALL() \
    POWER_TEST_RUN_POSITIVE(); \
    POWER_TEST_RUN_NEGATIVE()

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

void power_test(void *args);

/* ========================================================================== */
/*                    pwrSetResourceEnable API Tests                  */
/* ========================================================================== */

/* Positive tests */
void test_pos_power_enableDisable_buck1(void);
void test_pos_power_enableDisable_buck2(void);
void test_pos_power_enableDisable_buck3(void);
void test_pos_power_enableDisable_gpo(void);
void test_pos_power_enableDisable_ldoLs1Vmon1(void);
void test_pos_power_enableDisable_ls2Vmon2(void);
void test_pos_power_enableDisable_vccaVmon(void);

/* Negative tests */
void test_neg_power_pwrSetResourceEnable_nullParam_handle(void);
void test_neg_power_pwrSetResourceEnable_outOfBounds_resource(void);

/* ========================================================================== */
/*                    pwrGetResourceEnable API Tests                  */
/* ========================================================================== */

/* Negative tests */
void test_neg_power_pwrGetResourceEnable_nullParam_handle(void);
void test_neg_power_pwrGetResourceEnable_nullParam_isEnabled(void);
void test_neg_power_pwrGetResourceEnable_outOfBounds_resource(void);

/* ========================================================================== */
/*                    pwrSetResourceCfg API Tests                     */
/* ========================================================================== */

/* Positive tests */
void test_pos_power_setGetResourceCfg_allRsrc_allCfg(void);
void test_pos_power_setGetResourceCfg_buck1_deglitch(void);
void test_pos_power_setGetResourceCfg_buck1_enable(void);
void test_pos_power_setGetResourceCfg_buck1_ilim(void);
void test_pos_power_setGetResourceCfg_buck1_mode(void);
void test_pos_power_setGetResourceCfg_buck1_ovReaction(void);
void test_pos_power_setGetResourceCfg_buck1_ovThresh(void);
void test_pos_power_setGetResourceCfg_buck1_rvReaction(void);
void test_pos_power_setGetResourceCfg_buck1_scReaction(void);
void test_pos_power_setGetResourceCfg_buck1_uvReaction(void);
void test_pos_power_setGetResourceCfg_buck1_uvThresh(void);
void test_pos_power_setGetResourceCfg_buck1_voltage_mV(void);
void test_pos_power_setGetResourceCfg_buck2_deglitch(void);
void test_pos_power_setGetResourceCfg_buck2_enable(void);
void test_pos_power_setGetResourceCfg_buck2_ilim(void);
void test_pos_power_setGetResourceCfg_buck2_mode(void);
void test_pos_power_setGetResourceCfg_buck2_ovReaction(void);
void test_pos_power_setGetResourceCfg_buck2_ovThresh(void);
void test_pos_power_setGetResourceCfg_buck2_rvReaction(void);
void test_pos_power_setGetResourceCfg_buck2_scReaction(void);
void test_pos_power_setGetResourceCfg_buck2_uvReaction(void);
void test_pos_power_setGetResourceCfg_buck2_uvThresh(void);
void test_pos_power_setGetResourceCfg_buck2_voltage_mV(void);
void test_pos_power_setGetResourceCfg_buck3_deglitch(void);
void test_pos_power_setGetResourceCfg_buck3_enable(void);
void test_pos_power_setGetResourceCfg_buck3_ilim(void);
void test_pos_power_setGetResourceCfg_buck3_mode(void);
void test_pos_power_setGetResourceCfg_buck3_ovReaction(void);
void test_pos_power_setGetResourceCfg_buck3_ovThresh(void);
void test_pos_power_setGetResourceCfg_buck3_rvReaction(void);
void test_pos_power_setGetResourceCfg_buck3_scReaction(void);
void test_pos_power_setGetResourceCfg_buck3_uvReaction(void);
void test_pos_power_setGetResourceCfg_buck3_uvThresh(void);
void test_pos_power_setGetResourceCfg_buck3_voltage_mV(void);
void test_pos_power_setGetResourceCfg_gpo_enable(void);
void test_pos_power_setGetResourceCfg_ldoLs1Vmon1_deglitch(void);
void test_pos_power_setGetResourceCfg_ldoLs1Vmon1_enable(void);
void test_pos_power_setGetResourceCfg_ldoLs1Vmon1_mode(void);
void test_pos_power_setGetResourceCfg_ldoLs1Vmon1_ovReaction(void);
void test_pos_power_setGetResourceCfg_ldoLs1Vmon1_ovThresh(void);
void test_pos_power_setGetResourceCfg_ldoLs1Vmon1_rvReaction(void);
void test_pos_power_setGetResourceCfg_ldoLs1Vmon1_scReaction(void);
void test_pos_power_setGetResourceCfg_ldoLs1Vmon1_uvReaction(void);
void test_pos_power_setGetResourceCfg_ldoLs1Vmon1_uvThresh(void);
void test_pos_power_setGetResourceCfg_ldoLs1Vmon1_voltage_mV(void);
void test_pos_power_setGetResourceCfg_ls2Vmon2_deglitch(void);
void test_pos_power_setGetResourceCfg_ls2Vmon2_enable(void);
void test_pos_power_setGetResourceCfg_ls2Vmon2_mode(void);
void test_pos_power_setGetResourceCfg_ls2Vmon2_ovReaction(void);
void test_pos_power_setGetResourceCfg_ls2Vmon2_ovThresh(void);
void test_pos_power_setGetResourceCfg_ls2Vmon2_rvReaction(void);
void test_pos_power_setGetResourceCfg_ls2Vmon2_scReaction(void);
void test_pos_power_setGetResourceCfg_ls2Vmon2_uvReaction(void);
void test_pos_power_setGetResourceCfg_ls2Vmon2_uvThresh(void);
void test_pos_power_setGetResourceCfg_ls2Vmon2_voltage_mV(void);
void test_pos_power_setGetResourceCfg_vccaVmon_deglitch(void);
void test_pos_power_setGetResourceCfg_vccaVmon_enable(void);
void test_pos_power_setGetResourceCfg_vccaVmon_mode(void);
void test_pos_power_setGetResourceCfg_vccaVmon_ovReaction(void);
void test_pos_power_setGetResourceCfg_vccaVmon_ovThresh(void);
void test_pos_power_setGetResourceCfg_vccaVmon_uvReaction(void);
void test_pos_power_setGetResourceCfg_vccaVmon_uvThresh(void);
void test_pos_power_setGetResourceCfg_vccaVmon_voltage_mV(void);

/* Negative tests */
void test_neg_power_pwrSetResourceCfg_gpo_deglitch(void);
void test_neg_power_pwrSetResourceCfg_gpo_ilim(void);
void test_neg_power_pwrSetResourceCfg_gpo_mode(void);
void test_neg_power_pwrSetResourceCfg_gpo_ovReaction(void);
void test_neg_power_pwrSetResourceCfg_gpo_ovThresh(void);
void test_neg_power_pwrSetResourceCfg_gpo_rvReaction(void);
void test_neg_power_pwrSetResourceCfg_gpo_scReaction(void);
void test_neg_power_pwrSetResourceCfg_gpo_uvReaction(void);
void test_neg_power_pwrSetResourceCfg_gpo_uvThresh(void);
void test_neg_power_pwrSetResourceCfg_gpo_voltage_mV(void);
void test_neg_power_pwrSetResourceCfg_invalidParam_buck1_mode(void);
void test_neg_power_pwrSetResourceCfg_invalidParam_buck2_mode(void);
void test_neg_power_pwrSetResourceCfg_invalidParam_buck3_mode(void);
void test_neg_power_pwrSetResourceCfg_invalidParam_ls2Vmon2_mode(void);
void test_neg_power_pwrSetResourceCfg_invalidParam_vccaVmon_mode(void);
void test_neg_power_pwrSetResourceCfg_ldoLs1Vmon1_ilim(void);
void test_neg_power_pwrSetResourceCfg_ls2Vmon2_ilim(void);
void test_neg_power_pwrSetResourceCfg_nullParam_config(void);
void test_neg_power_pwrSetResourceCfg_nullParam_handle(void);
void test_neg_power_pwrSetResourceCfg_outOfBounds_buck1_deglitch(void);
void test_neg_power_pwrSetResourceCfg_outOfBounds_buck1_ilim(void);
void test_neg_power_pwrSetResourceCfg_outOfBounds_buck1_ovReaction(void);
void test_neg_power_pwrSetResourceCfg_outOfBounds_buck1_ovThresh(void);
void test_neg_power_pwrSetResourceCfg_outOfBounds_buck1_rvReaction(void);
void test_neg_power_pwrSetResourceCfg_outOfBounds_buck1_scReaction(void);
void test_neg_power_pwrSetResourceCfg_outOfBounds_buck1_uvReaction(void);
void test_neg_power_pwrSetResourceCfg_outOfBounds_buck1_uvThresh(void);
void test_neg_power_pwrSetResourceCfg_outOfBounds_buck1_voltage_mV(void);
void test_neg_power_pwrSetResourceCfg_outOfBounds_buck2_deglitch(void);
void test_neg_power_pwrSetResourceCfg_outOfBounds_buck2_ilim(void);
void test_neg_power_pwrSetResourceCfg_outOfBounds_buck2_ovReaction(void);
void test_neg_power_pwrSetResourceCfg_outOfBounds_buck2_ovThresh(void);
void test_neg_power_pwrSetResourceCfg_outOfBounds_buck2_rvReaction(void);
void test_neg_power_pwrSetResourceCfg_outOfBounds_buck2_scReaction(void);
void test_neg_power_pwrSetResourceCfg_outOfBounds_buck2_uvReaction(void);
void test_neg_power_pwrSetResourceCfg_outOfBounds_buck2_uvThresh(void);
void test_neg_power_pwrSetResourceCfg_outOfBounds_buck2_voltage_mV(void);
void test_neg_power_pwrSetResourceCfg_outOfBounds_buck3_deglitch(void);
void test_neg_power_pwrSetResourceCfg_outOfBounds_buck3_ilim(void);
void test_neg_power_pwrSetResourceCfg_outOfBounds_buck3_ovReaction(void);
void test_neg_power_pwrSetResourceCfg_outOfBounds_buck3_ovThresh(void);
void test_neg_power_pwrSetResourceCfg_outOfBounds_buck3_rvReaction(void);
void test_neg_power_pwrSetResourceCfg_outOfBounds_buck3_scReaction(void);
void test_neg_power_pwrSetResourceCfg_outOfBounds_buck3_uvReaction(void);
void test_neg_power_pwrSetResourceCfg_outOfBounds_buck3_uvThresh(void);
void test_neg_power_pwrSetResourceCfg_outOfBounds_buck3_voltage_mV(void);
void test_neg_power_pwrSetResourceCfg_outOfBounds_ldoLs1Vmon1_deglitch(void);
void test_neg_power_pwrSetResourceCfg_outOfBounds_ldoLs1Vmon1_mode(void);
void test_neg_power_pwrSetResourceCfg_outOfBounds_ldoLs1Vmon1_ovReaction(void);
void test_neg_power_pwrSetResourceCfg_outOfBounds_ldoLs1Vmon1_ovThresh(void);
void test_neg_power_pwrSetResourceCfg_outOfBounds_ldoLs1Vmon1_rvReaction(void);
void test_neg_power_pwrSetResourceCfg_outOfBounds_ldoLs1Vmon1_scReaction(void);
void test_neg_power_pwrSetResourceCfg_outOfBounds_ldoLs1Vmon1_uvReaction(void);
void test_neg_power_pwrSetResourceCfg_outOfBounds_ldoLs1Vmon1_uvThresh(void);
void test_neg_power_pwrSetResourceCfg_outOfBounds_ldoLs1Vmon1_voltage_mV(void);
void test_neg_power_pwrSetResourceCfg_outOfBounds_ls2Vmon2_deglitch(void);
void test_neg_power_pwrSetResourceCfg_outOfBounds_ls2Vmon2_ovReaction(void);
void test_neg_power_pwrSetResourceCfg_outOfBounds_ls2Vmon2_ovThresh(void);
void test_neg_power_pwrSetResourceCfg_outOfBounds_ls2Vmon2_rvReaction(void);
void test_neg_power_pwrSetResourceCfg_outOfBounds_ls2Vmon2_scReaction(void);
void test_neg_power_pwrSetResourceCfg_outOfBounds_ls2Vmon2_uvReaction(void);
void test_neg_power_pwrSetResourceCfg_outOfBounds_ls2Vmon2_uvThresh(void);
void test_neg_power_pwrSetResourceCfg_outOfBounds_ls2Vmon2_voltage_mV(void);
void test_neg_power_pwrSetResourceCfg_outOfBounds_resource(void);
void test_neg_power_pwrSetResourceCfg_outOfBounds_vccaVmon_deglitch(void);
void test_neg_power_pwrSetResourceCfg_outOfBounds_vccaVmon_ovReaction(void);
void test_neg_power_pwrSetResourceCfg_outOfBounds_vccaVmon_ovThresh(void);
void test_neg_power_pwrSetResourceCfg_outOfBounds_vccaVmon_uvReaction(void);
void test_neg_power_pwrSetResourceCfg_outOfBounds_vccaVmon_uvThresh(void);
void test_neg_power_pwrSetResourceCfg_outOfBounds_vccaVmon_voltage_mV(void);
void test_neg_power_pwrSetResourceCfg_vccaVmon_ilim(void);
void test_neg_power_pwrSetResourceCfg_vccaVmon_rvReaction(void);
void test_neg_power_pwrSetResourceCfg_vccaVmon_scReaction(void);

/* ========================================================================== */
/*                    pwrGetResourceCfg API Tests                     */
/* ========================================================================== */

/* Negative tests */
void test_neg_power_pwrGetResourceCfg_gpo_unsupportedDeglitch(void);
void test_neg_power_pwrGetResourceCfg_gpo_unsupportedIlim(void);
void test_neg_power_pwrGetResourceCfg_gpo_unsupportedOvReaction(void);
void test_neg_power_pwrGetResourceCfg_gpo_unsupportedOvThresh(void);
void test_neg_power_pwrGetResourceCfg_gpo_unsupportedRvReaction(void);
void test_neg_power_pwrGetResourceCfg_gpo_unsupportedScReaction(void);
void test_neg_power_pwrGetResourceCfg_gpo_unsupportedUvReaction(void);
void test_neg_power_pwrGetResourceCfg_gpo_unsupportedUvThresh(void);
void test_neg_power_pwrGetResourceCfg_ldoLs1Vmon1_invalidHwState(void);
void test_neg_power_pwrGetResourceCfg_nullParam_config(void);
void test_neg_power_pwrGetResourceCfg_nullParam_handle(void);
void test_neg_power_pwrGetResourceCfg_outOfBounds_resource(void);
void test_neg_power_pwrGetResourceCfg_vccaVmon_unsupportedIlim(void);
void test_neg_power_pwrGetResourceCfg_vccaVmon_unsupportedRvReaction(void);
void test_neg_power_pwrGetResourceCfg_vccaVmon_unsupportedScReaction(void);

/* ========================================================================== */
/*                    pwrSetResourceCfgs API Tests                    */
/* ========================================================================== */

/* Negative tests */
void test_neg_power_pwrSetResourceCfgs_gpo_deglitch(void);
void test_neg_power_pwrSetResourceCfgs_gpo_ilim(void);
void test_neg_power_pwrSetResourceCfgs_gpo_mode(void);
void test_neg_power_pwrSetResourceCfgs_gpo_ovReaction(void);
void test_neg_power_pwrSetResourceCfgs_gpo_ovThresh(void);
void test_neg_power_pwrSetResourceCfgs_gpo_rvReaction(void);
void test_neg_power_pwrSetResourceCfgs_gpo_scReaction(void);
void test_neg_power_pwrSetResourceCfgs_gpo_uvReaction(void);
void test_neg_power_pwrSetResourceCfgs_gpo_uvThresh(void);
void test_neg_power_pwrSetResourceCfgs_gpo_voltage_mV(void);
void test_neg_power_pwrSetResourceCfgs_invalidParam_buck1_mode(void);
void test_neg_power_pwrSetResourceCfgs_invalidParam_buck2_mode(void);
void test_neg_power_pwrSetResourceCfgs_invalidParam_buck3_mode(void);
void test_neg_power_pwrSetResourceCfgs_invalidParam_ls2Vmon2_mode(void);
void test_neg_power_pwrSetResourceCfgs_invalidParam_vccaVmon_mode(void);
void test_neg_power_pwrSetResourceCfgs_ldoLs1Vmon1_ilim(void);
void test_neg_power_pwrSetResourceCfgs_ls2Vmon2_ilim(void);
void test_neg_power_pwrSetResourceCfgs_nullParam_config(void);
void test_neg_power_pwrSetResourceCfgs_nullParam_handle(void);
void test_neg_power_pwrSetResourceCfgs_outOfBounds_buck1_deglitch(void);
void test_neg_power_pwrSetResourceCfgs_outOfBounds_buck1_ilim(void);
void test_neg_power_pwrSetResourceCfgs_outOfBounds_buck1_ovReaction(void);
void test_neg_power_pwrSetResourceCfgs_outOfBounds_buck1_ovThresh(void);
void test_neg_power_pwrSetResourceCfgs_outOfBounds_buck1_rvReaction(void);
void test_neg_power_pwrSetResourceCfgs_outOfBounds_buck1_scReaction(void);
void test_neg_power_pwrSetResourceCfgs_outOfBounds_buck1_uvReaction(void);
void test_neg_power_pwrSetResourceCfgs_outOfBounds_buck1_uvThresh(void);
void test_neg_power_pwrSetResourceCfgs_outOfBounds_buck1_voltage_mV(void);
void test_neg_power_pwrSetResourceCfgs_outOfBounds_buck2_deglitch(void);
void test_neg_power_pwrSetResourceCfgs_outOfBounds_buck2_ilim(void);
void test_neg_power_pwrSetResourceCfgs_outOfBounds_buck2_ovReaction(void);
void test_neg_power_pwrSetResourceCfgs_outOfBounds_buck2_ovThresh(void);
void test_neg_power_pwrSetResourceCfgs_outOfBounds_buck2_rvReaction(void);
void test_neg_power_pwrSetResourceCfgs_outOfBounds_buck2_scReaction(void);
void test_neg_power_pwrSetResourceCfgs_outOfBounds_buck2_uvReaction(void);
void test_neg_power_pwrSetResourceCfgs_outOfBounds_buck2_uvThresh(void);
void test_neg_power_pwrSetResourceCfgs_outOfBounds_buck2_voltage_mV(void);
void test_neg_power_pwrSetResourceCfgs_outOfBounds_buck3_deglitch(void);
void test_neg_power_pwrSetResourceCfgs_outOfBounds_buck3_ilim(void);
void test_neg_power_pwrSetResourceCfgs_outOfBounds_buck3_ovReaction(void);
void test_neg_power_pwrSetResourceCfgs_outOfBounds_buck3_ovThresh(void);
void test_neg_power_pwrSetResourceCfgs_outOfBounds_buck3_rvReaction(void);
void test_neg_power_pwrSetResourceCfgs_outOfBounds_buck3_scReaction(void);
void test_neg_power_pwrSetResourceCfgs_outOfBounds_buck3_uvReaction(void);
void test_neg_power_pwrSetResourceCfgs_outOfBounds_buck3_uvThresh(void);
void test_neg_power_pwrSetResourceCfgs_outOfBounds_buck3_voltage_mV(void);
void test_neg_power_pwrSetResourceCfgs_outOfBounds_ldoLs1Vmon1_deglitch(void);
void test_neg_power_pwrSetResourceCfgs_outOfBounds_ldoLs1Vmon1_mode(void);
void test_neg_power_pwrSetResourceCfgs_outOfBounds_ldoLs1Vmon1_ovReaction(void);
void test_neg_power_pwrSetResourceCfgs_outOfBounds_ldoLs1Vmon1_ovThresh(void);
void test_neg_power_pwrSetResourceCfgs_outOfBounds_ldoLs1Vmon1_rvReaction(void);
void test_neg_power_pwrSetResourceCfgs_outOfBounds_ldoLs1Vmon1_scReaction(void);
void test_neg_power_pwrSetResourceCfgs_outOfBounds_ldoLs1Vmon1_uvReaction(void);
void test_neg_power_pwrSetResourceCfgs_outOfBounds_ldoLs1Vmon1_uvThresh(void);
void test_neg_power_pwrSetResourceCfgs_outOfBounds_ldoLs1Vmon1_voltage_mV(void);
void test_neg_power_pwrSetResourceCfgs_outOfBounds_ls2Vmon2_deglitch(void);
void test_neg_power_pwrSetResourceCfgs_outOfBounds_ls2Vmon2_ovReaction(void);
void test_neg_power_pwrSetResourceCfgs_outOfBounds_ls2Vmon2_ovThresh(void);
void test_neg_power_pwrSetResourceCfgs_outOfBounds_ls2Vmon2_rvReaction(void);
void test_neg_power_pwrSetResourceCfgs_outOfBounds_ls2Vmon2_scReaction(void);
void test_neg_power_pwrSetResourceCfgs_outOfBounds_ls2Vmon2_uvReaction(void);
void test_neg_power_pwrSetResourceCfgs_outOfBounds_ls2Vmon2_uvThresh(void);
void test_neg_power_pwrSetResourceCfgs_outOfBounds_ls2Vmon2_voltage_mV(void);
void test_neg_power_pwrSetResourceCfgs_outOfBounds_resource(void);
void test_neg_power_pwrSetResourceCfgs_outOfBounds_vccaVmon_deglitch(void);
void test_neg_power_pwrSetResourceCfgs_outOfBounds_vccaVmon_ovReaction(void);
void test_neg_power_pwrSetResourceCfgs_outOfBounds_vccaVmon_ovThresh(void);
void test_neg_power_pwrSetResourceCfgs_outOfBounds_vccaVmon_uvReaction(void);
void test_neg_power_pwrSetResourceCfgs_outOfBounds_vccaVmon_uvThresh(void);
void test_neg_power_pwrSetResourceCfgs_outOfBounds_vccaVmon_voltage_mV(void);
void test_neg_power_pwrSetResourceCfgs_vccaVmon_ilim(void);
void test_neg_power_pwrSetResourceCfgs_vccaVmon_rvReaction(void);
void test_neg_power_pwrSetResourceCfgs_vccaVmon_scReaction(void);

/* ========================================================================== */
/*                    pwrGetResourceCfgs API Tests                    */
/* ========================================================================== */

/* Negative tests */
void test_neg_power_pwrGetResourceCfgs_nullParam_config(void);
void test_neg_power_pwrGetResourceCfgs_nullParam_handle(void);
void test_neg_power_pwrGetResourceCfgs_outOfBounds_resource(void);

/* ========================================================================== */
/*                    pwrSetSequenceCfg API Tests                     */
/* ========================================================================== */

/* Positive tests */
void test_pos_power_setGetSequenceCfg_allRsrc_allCfg(void);
void test_pos_power_setGetSequenceCfg_buck1_shutdownDelay(void);
void test_pos_power_setGetSequenceCfg_buck1_startupDelay(void);
void test_pos_power_setGetSequenceCfg_buck2_shutdownDelay(void);
void test_pos_power_setGetSequenceCfg_buck2_startupDelay(void);
void test_pos_power_setGetSequenceCfg_buck3_shutdownDelay(void);
void test_pos_power_setGetSequenceCfg_buck3_startupDelay(void);
void test_pos_power_setGetSequenceCfg_gpo_shutdownDelay(void);
void test_pos_power_setGetSequenceCfg_gpo_startupDelay(void);
void test_pos_power_setGetSequenceCfg_ldoLs1Vmon1_shutdownDelay(void);
void test_pos_power_setGetSequenceCfg_ldoLs1Vmon1_startupDelay(void);
void test_pos_power_setGetSequenceCfg_ls2Vmon2_shutdownDelay(void);
void test_pos_power_setGetSequenceCfg_ls2Vmon2_startupDelay(void);

/* Negative tests */
void test_neg_power_pwrSetSequenceCfg_nullParam_config(void);
void test_neg_power_pwrSetSequenceCfg_nullParam_handle(void);
void test_neg_power_pwrSetSequenceCfg_outOfBounds_buck1_shutdownDelay(void);
void test_neg_power_pwrSetSequenceCfg_outOfBounds_buck1_startupDelay(void);
void test_neg_power_pwrSetSequenceCfg_outOfBounds_buck2_shutdownDelay(void);
void test_neg_power_pwrSetSequenceCfg_outOfBounds_buck2_startupDelay(void);
void test_neg_power_pwrSetSequenceCfg_outOfBounds_buck3_shutdownDelay(void);
void test_neg_power_pwrSetSequenceCfg_outOfBounds_buck3_startupDelay(void);
void test_neg_power_pwrSetSequenceCfg_outOfBounds_gpo_shutdownDelay(void);
void test_neg_power_pwrSetSequenceCfg_outOfBounds_gpo_startupDelay(void);
void test_neg_power_pwrSetSequenceCfg_outOfBounds_ldoLs1Vmon1_shutdownDelay(void);
void test_neg_power_pwrSetSequenceCfg_outOfBounds_ldoLs1Vmon1_startupDelay(void);
void test_neg_power_pwrSetSequenceCfg_outOfBounds_ls2Vmon2_shutdownDelay(void);
void test_neg_power_pwrSetSequenceCfg_outOfBounds_ls2Vmon2_startupDelay(void);
void test_neg_power_pwrSetSequenceCfg_outOfBounds_resource(void);
void test_neg_power_pwrSetSequenceCfg_vccaVmon_shutdownDelay(void);
void test_neg_power_pwrSetSequenceCfg_vccaVmon_startupDelay(void);

/* ========================================================================== */
/*                    pwrGetSequenceCfg API Tests                     */
/* ========================================================================== */

/* Positive tests */
void test_pos_power_getNrstoutSequence(void);

/* Negative tests */
void test_neg_power_pwrGetSequenceCfg_nullParam_config(void);
void test_neg_power_pwrGetSequenceCfg_nullParam_handle(void);

/* ========================================================================== */
/*                    pwrSetSequenceCfgs API Tests                    */
/* ========================================================================== */

/* Negative tests */
void test_neg_power_pwrSetSequenceCfgs_nullParam_config(void);
void test_neg_power_pwrSetSequenceCfgs_nullParam_handle(void);
void test_neg_power_pwrSetSequenceCfgs_numConfigs_zero(void);
void test_neg_power_pwrSetSequenceCfgs_outOfBounds_buck1_shutdownDelay(void);
void test_neg_power_pwrSetSequenceCfgs_outOfBounds_buck1_startupDelay(void);
void test_neg_power_pwrSetSequenceCfgs_outOfBounds_buck2_shutdownDelay(void);
void test_neg_power_pwrSetSequenceCfgs_outOfBounds_buck2_startupDelay(void);
void test_neg_power_pwrSetSequenceCfgs_outOfBounds_buck3_shutdownDelay(void);
void test_neg_power_pwrSetSequenceCfgs_outOfBounds_buck3_startupDelay(void);
void test_neg_power_pwrSetSequenceCfgs_outOfBounds_gpo_shutdownDelay(void);
void test_neg_power_pwrSetSequenceCfgs_outOfBounds_gpo_startupDelay(void);
void test_neg_power_pwrSetSequenceCfgs_outOfBounds_ldoLs1Vmon1_shutdownDelay(void);
void test_neg_power_pwrSetSequenceCfgs_outOfBounds_ldoLs1Vmon1_startupDelay(void);
void test_neg_power_pwrSetSequenceCfgs_outOfBounds_ls2Vmon2_shutdownDelay(void);
void test_neg_power_pwrSetSequenceCfgs_outOfBounds_ls2Vmon2_startupDelay(void);
void test_neg_power_pwrSetSequenceCfgs_outOfBounds_resource(void);
void test_neg_power_pwrSetSequenceCfgs_vccaVmon_shutdownDelay(void);
void test_neg_power_pwrSetSequenceCfgs_vccaVmon_startupDelay(void);

/* ========================================================================== */
/*                    pwrGetSequenceCfgs API Tests                    */
/* ========================================================================== */

/* Negative tests */
void test_neg_power_pwrGetSequenceCfgs_nullParam_config(void);
void test_neg_power_pwrGetSequenceCfgs_nullParam_handle(void);
void test_neg_power_pwrGetSequenceCfgs_numConfigs_zero(void);
void test_neg_power_pwrGetSequenceCfgs_outOfBounds_resource(void);

/* ========================================================================== */
/*                          Coverage Tests                                    */
/* ========================================================================== */

/* Positive tests */
void test_pos_power_powerGetPgoodLevel_validBuck(void);
void test_pos_power_powerSetPgoodLevel_validBuck(void);

/* Negative tests */
void test_neg_power_powerGetVoutCfg_unsupportedRegulator(void);
void test_neg_power_powerSetVoutCfg_unsupportedRegulator(void);
void test_neg_power_pwr_getIlimCfg_resourceOutOfBounds(void);
void test_neg_power_pwr_getModeCfg_unsupportedResource(void);
void test_neg_power_pwr_getResourceCfgs_zeroNumConfigs(void);
void test_neg_power_pwr_getVoltageCfg_resourceOutOfBounds(void);
void test_neg_power_pwr_invalidModeCombination_ldoLs1Vmon1(void);
void test_neg_power_pwr_setIlimCfg_resourceOutOfBounds(void);
void test_neg_power_pwr_setResourceCfg_excessiveNumConfigs(void);
void test_neg_power_pwr_setResourceCfg_zeroNumConfigs(void);
void test_neg_power_setPgLevel_invalidResource(void);
void test_neg_power_setVoltage_invalidResource(void);

/* Batch error handling tests */
void test_neg_power_pwrGetResourceCfgs_errorMidBatch_outputUnchanged(void);
void test_neg_power_pwrSetResourceCfgs_errorMidBatch_partialApply(void);
void test_neg_power_pwrGetSequenceCfgs_errorMidBatch_outputUnchanged(void);
void test_neg_power_pwrSetSequenceCfgs_errorMidBatch_partialApply(void);

/* ========================================================================== */
/*        pwrSetThermalCfg and pwrGetThermalCfg API Tests                    */
/* ========================================================================== */

/* Positive tests */
void test_pos_power_thermal_twarnLvl(void);
void test_pos_power_thermal_tsdOrdLvl(void);
void test_pos_power_thermal_twarnConfig(void);
void test_pos_power_thermal_all_params(void);

/* Negative tests */
void test_neg_power_thermal_null_handle(void);
void test_neg_power_thermal_null_param(void);
void test_neg_power_thermal_invalid_twarnLvl(void);
void test_neg_power_thermal_invalid_tsdOrdLvl(void);
void test_neg_power_thermal_invalid_twarnConfig(void);
void test_neg_power_thermal_no_valid_params(void);

/* MC/DC coverage test functions */
void test_pos_power_setGetResourceCfg_buck1_enable_mcdc(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /*__POWER_TEST_H__*/
