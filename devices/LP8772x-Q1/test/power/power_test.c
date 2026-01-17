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


/* ========================================================================== */
/*                              Include Files                                 */
/* ========================================================================== */

#include "power_test.h"
#include "test_inject.h"

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/**
 * @brief Resource count and configuration flags
 */
#define POWER_TEST_NUM_RESOURCES           (7U)   /* BUCK1-3, LDO_LS1_VMON1, LS2_VMON2, VCCA_VMON, GPO */
#define POWER_TEST_NUM_RESOURCES_NO_VCCA   (6U)   /* All resources except VCCA_VMON */
#define POWER_TEST_IS_EXPECTED_CFG         (true)
#define POWER_TEST_IS_ACTUAL_CFG           (false)

/* ========================================================================== */
/*     API-Specific Test Macros - pwrSetResourceEnable                         */
/* ========================================================================== */

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

#define POWER_TEST_PWRSETRESOURCEENABLE() \
    POWER_TEST_POS_PWRSETRESOURCEENABLE(); \
    POWER_TEST_NEG_PWRSETRESOURCEENABLE()

/* ========================================================================== */
/*     API-Specific Test Macros - pwrGetResourceEnable                         */
/* ========================================================================== */

#define POWER_TEST_NEG_PWRGETRESOURCEENABLE() \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetResourceEnable_nullParam_handle); \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetResourceEnable_nullParam_isEnabled); \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetResourceEnable_outOfBounds_resource)

#define POWER_TEST_PWRGETRESOURCEENABLE() \
    POWER_TEST_NEG_PWRGETRESOURCEENABLE()

/* ========================================================================== */
/*     API-Specific Test Macros - pwrSetResourceCfg                            */
/* ========================================================================== */

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
    PLATFORM_RUN_TEST(test_pos_power_setGetResourceCfg_vccaVmon_voltage_mV)

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
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfg_vccaVmon_scReaction)

#define POWER_TEST_PWRSETRESOURCECFG() \
    POWER_TEST_POS_PWRSETRESOURCECFG(); \
    POWER_TEST_NEG_PWRSETRESOURCECFG()

/* ========================================================================== */
/*     API-Specific Test Macros - pwrGetResourceCfg                            */
/* ========================================================================== */

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
    PLATFORM_RUN_TEST(test_neg_power_pwrGetResourceCfg_vccaVmon_unsupportedScReaction)

#define POWER_TEST_PWRGETRESOURCECFG() \
    POWER_TEST_NEG_PWRGETRESOURCECFG()

/* ========================================================================== */
/*     API-Specific Test Macros - pwrSetResourceCfgs                           */
/* ========================================================================== */

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
    PLATFORM_RUN_TEST(test_neg_power_pwrSetResourceCfgs_vccaVmon_scReaction)

#define POWER_TEST_PWRSETRESOURCECFGS() \
    POWER_TEST_NEG_PWRSETRESOURCECFGS()

/* ========================================================================== */
/*     API-Specific Test Macros - pwrGetResourceCfgs                           */
/* ========================================================================== */

#define POWER_TEST_NEG_PWRGETRESOURCECFGS() \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetResourceCfgs_nullParam_config); \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetResourceCfgs_nullParam_handle); \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetResourceCfgs_outOfBounds_resource)

#define POWER_TEST_PWRGETRESOURCECFGS() \
    POWER_TEST_NEG_PWRGETRESOURCECFGS()

/* ========================================================================== */
/*     API-Specific Test Macros - pwrSetSequenceCfg                            */
/* ========================================================================== */

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

#define POWER_TEST_PWRSETSEQUENCECFG() \
    POWER_TEST_POS_PWRSETSEQUENCECFG(); \
    POWER_TEST_NEG_PWRSETSEQUENCECFG()

/* ========================================================================== */
/*     API-Specific Test Macros - pwrGetSequenceCfg                            */
/* ========================================================================== */

#define POWER_TEST_POS_PWRGETSEQUENCECFG() \
    PLATFORM_RUN_TEST(test_pos_power_getNrstoutSequence)

#define POWER_TEST_NEG_PWRGETSEQUENCECFG() \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetSequenceCfg_nullParam_config); \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetSequenceCfg_nullParam_handle)

#define POWER_TEST_PWRGETSEQUENCECFG() \
    POWER_TEST_POS_PWRGETSEQUENCECFG(); \
    POWER_TEST_NEG_PWRGETSEQUENCECFG()

/* ========================================================================== */
/*     API-Specific Test Macros - pwrSetSequenceCfgs                           */
/* ========================================================================== */

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
    PLATFORM_RUN_TEST(test_neg_power_pwrSetSequenceCfgs_vccaVmon_startupDelay)

#define POWER_TEST_PWRSETSEQUENCECFGS() \
    POWER_TEST_NEG_PWRSETSEQUENCECFGS()

/* ========================================================================== */
/*     API-Specific Test Macros - pwrGetSequenceCfgs                           */
/* ========================================================================== */

#define POWER_TEST_NEG_PWRGETSEQUENCECFGS() \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetSequenceCfgs_nullParam_config); \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetSequenceCfgs_nullParam_handle); \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetSequenceCfgs_numConfigs_zero); \
    PLATFORM_RUN_TEST(test_neg_power_pwrGetSequenceCfgs_outOfBounds_resource)

#define POWER_TEST_PWRGETSEQUENCECFGS() \
    POWER_TEST_NEG_PWRGETSEQUENCECFGS()

/* ========================================================================== */
/*                   API-Specific Test Macros - Coverage                     */
/* ========================================================================== */

#define POWER_TEST_POS_COVERAGE() \
    PLATFORM_RUN_TEST(test_pos_power_powerGetPgoodLevel_validBuck); \
    PLATFORM_RUN_TEST(test_pos_power_powerSetPgoodLevel_validBuck)

#define POWER_TEST_NEG_COVERAGE() \
    PLATFORM_RUN_TEST(test_neg_power_powerGetVoutCfg_unsupportedRegulator); \
    PLATFORM_RUN_TEST(test_neg_power_powerSetVoutCfg_unsupportedRegulator); \
    PLATFORM_RUN_TEST(test_neg_power_pwr_getIlimCfg_resourceOutOfBounds); \
    PLATFORM_RUN_TEST(test_neg_power_pwr_getModeCfg_unsupportedResource); \
    PLATFORM_RUN_TEST(test_neg_power_pwr_getResourceCfgs_zeroNumConfigs); \
    PLATFORM_RUN_TEST(test_neg_power_pwr_getVoltageCfg_resourceOutOfBounds); \
    PLATFORM_RUN_TEST(test_neg_power_pwr_invalidModeCombination_ldoLs1Vmon1); \
    PLATFORM_RUN_TEST(test_neg_power_pwr_setIlimCfg_resourceOutOfBounds); \
    PLATFORM_RUN_TEST(test_neg_power_pwr_setResourceCfg_excessiveNumConfigs); \
    PLATFORM_RUN_TEST(test_neg_power_pwr_setResourceCfg_zeroNumConfigs); \
    PLATFORM_RUN_TEST(test_neg_power_setPgLevel_invalidResource); \
    PLATFORM_RUN_TEST(test_neg_power_setVoltage_invalidResource)

#define POWER_TEST_COVERAGE() \
    POWER_TEST_POS_COVERAGE(); \
    POWER_TEST_NEG_COVERAGE()

/* ========================================================================== */
/*                         Aggregate Test Runners                             */
/* ========================================================================== */

#define POWER_TEST_RUN_POSITIVE() \
    POWER_TEST_POS_PWRSETRESOURCEENABLE(); \
    POWER_TEST_POS_PWRSETRESOURCECFG(); \
    POWER_TEST_POS_PWRSETSEQUENCECFG(); \
    POWER_TEST_POS_PWRGETSEQUENCECFG(); \
    POWER_TEST_POS_COVERAGE()

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
    POWER_TEST_NEG_COVERAGE()

#define POWER_TEST_RUN_ALL() \
    POWER_TEST_RUN_POSITIVE(); \
    POWER_TEST_RUN_NEGATIVE()
/* ========================================================================== */
/*                             Global Variables                               */
/* ========================================================================== */
static Pmic_Handle_t pmicHandle;

/* ========================================================================== */
/*                           Function Declarations                            */
/* ========================================================================== */
static void powerTest_enableDisableResource(uint8_t rsrc);
static void powerTest_Pmic_pwrSetResourceCfg_invalidParam_buckMode(uint8_t buck);
static void powerTest_Pmic_pwrSetResourceCfg_outOfBounds_buckIlim(uint8_t buck);
static void powerTest_Pmic_pwrSetResourceCfg_outOfBounds_buckVoltage_mV(uint8_t buck);
static void powerTest_Pmic_pwrSetResourceCfg_outOfBounds_buckDeglitch(uint8_t buck);
static void powerTest_Pmic_pwrSetResourceCfg_outOfBounds_buckUvThresh(uint8_t buck);
static void powerTest_Pmic_pwrSetResourceCfg_outOfBounds_buckUvReaction(uint8_t buck);
static void powerTest_Pmic_pwrSetResourceCfg_outOfBounds_buckOvThresh(uint8_t buck);
static void powerTest_Pmic_pwrSetResourceCfg_outOfBounds_buckOvReaction(uint8_t buck);
static void powerTest_Pmic_pwrSetResourceCfg_outOfBounds_buckRvReaction(uint8_t buck);
static void powerTest_Pmic_pwrSetResourceCfg_outOfBounds_buckScReaction(uint8_t buck);
static void powerTest_Pmic_pwrSetResourceCfg_outOfBounds_ldoLsVmon_voltage_mV(uint8_t ldoLsVmon);
static void powerTest_Pmic_pwrSetResourceCfgs_invalidParam_buckMode(uint8_t buck);
static void powerTest_Pmic_pwrSetResourceCfgs_outOfBounds_buckIlim(uint8_t buck);
static void powerTest_Pmic_pwrSetResourceCfgs_outOfBounds_buckVoltage_mV(uint8_t buck);
static void powerTest_Pmic_pwrSetResourceCfgs_outOfBounds_buckDeglitch(uint8_t buck);
static void powerTest_Pmic_pwrSetResourceCfgs_outOfBounds_buckUvThresh(uint8_t buck);
static void powerTest_Pmic_pwrSetResourceCfgs_outOfBounds_buckUvReaction(uint8_t buck);
static void powerTest_Pmic_pwrSetResourceCfgs_outOfBounds_buckOvThresh(uint8_t buck);
static void powerTest_Pmic_pwrSetResourceCfgs_outOfBounds_buckOvReaction(uint8_t buck);
static void powerTest_Pmic_pwrSetResourceCfgs_outOfBounds_buckRvReaction(uint8_t buck);
static void powerTest_Pmic_pwrSetResourceCfgs_outOfBounds_buckScReaction(uint8_t buck);
static void powerTest_Pmic_pwrSetResourceCfgs_outOfBounds_ldoLsVmon_voltage_mV(uint8_t ldoLsVmon);
static void powerTest_Pmic_pwrSetSequenceCfg_outOfBounds_startupDelay(uint8_t resource);
static void powerTest_Pmic_pwrSetSequenceCfg_outOfBounds_shutdownDelay(uint8_t resource);
static void powerTest_Pmic_pwrSetSequenceCfgs_outOfBounds_startupDelay(uint8_t resource);
static void powerTest_Pmic_pwrSetSequenceCfgs_outOfBounds_shutdownDelay(uint8_t resource);
static void powerTest_setGetResourceCfg_enable(uint8_t resource);
static void powerTest_setGetResourceCfg_buck_mode(uint8_t buck);
static void powerTest_setGetResourceCfg_buck_ilim(uint8_t buck);
static void powerTest_setGetResourceCfg_buck_voltage_mV(uint8_t buck);
static void powerTest_setGetResourceCfg_deglitch(uint8_t resource);
static void powerTest_setGetResourceCfg_uvReaction(uint8_t resource);
static void powerTest_setGetResourceCfg_ovReaction(uint8_t resource);
static void powerTest_setGetResourceCfg_rvReaction(uint8_t resource);
static void powerTest_setGetResourceCfg_scReaction(uint8_t resource);
static void powerTest_setGetResourceCfg_ldoLsVmon_voltage_mV(uint8_t ldoLsVmon);
static inline void powerTest_initAllRsrcCfg(Pmic_PowerResourceCfg_t *resourceCfg, bool isExpCfg);
static inline void powerTest_initBuckCfg(Pmic_PowerResourceCfg_t *resourceCfg, uint8_t rsrc, bool isExpCfg);
static inline void powerTest_initLdoLs1VmonCfg(Pmic_PowerResourceCfg_t *resourceCfg, uint8_t rsrc, bool isExpCfg);
static inline void powerTest_initLs2Vmon2Cfg(Pmic_PowerResourceCfg_t *resourceCfg, uint8_t rsrc, bool isExpCfg);
static inline void powerTest_initVccaVmonCfg(Pmic_PowerResourceCfg_t *resourceCfg, uint8_t rsrc, bool isExpCfg);
static inline void powerTest_initGpoCfg(Pmic_PowerResourceCfg_t *resourceCfg, uint8_t rsrc, bool isExpCfg);
static inline void powerTest_compareExpActResourceCfgs(Pmic_PowerResourceCfg_t *expResourceCfgs, Pmic_PowerResourceCfg_t *actResourceCfgs);
static inline void powerTest_compareExpActGpoCfg(Pmic_PowerResourceCfg_t *expResourceCfg, Pmic_PowerResourceCfg_t *actResourceCfg);
static inline void powerTest_compareExpActVccaVmonCfg(Pmic_PowerResourceCfg_t *expResourceCfg, Pmic_PowerResourceCfg_t *actResourceCfg);
static inline void powerTest_compareExpActLdoLsVmonCfg(Pmic_PowerResourceCfg_t *expResourceCfg, Pmic_PowerResourceCfg_t *actResourceCfg);
static inline void powerTest_compareExpActBuckCfg(Pmic_PowerResourceCfg_t *expResourceCfg, Pmic_PowerResourceCfg_t *actResourceCfg);
static void powerTest_initAllSeqCfg(Pmic_PowerSequenceCfg_t *sequenceCfg, bool isExpCfg);
static void powerTest_compareExpActSeqCfgs(Pmic_PowerSequenceCfg_t *expSequenceCfgs, Pmic_PowerSequenceCfg_t *actSequenceCfgs);

/* ========================================================================== */
/*                           Function Definitions                             */
/* ========================================================================== */

void power_test(void *args)
{
    (void)args;
    char msg[50U] = {0};
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_HandleCfg_t coreCfg = {
        .validParams = (PMIC_COMM_MODE_VALID |
                        PMIC_I2C_ADDR0_VALID |
                        PMIC_COMM_HANDLE_0_VALID |
                        PMIC_IO_READ_VALID |
                        PMIC_IO_WRITE_VALID |
                        PMIC_CRITICAL_SECTION_START_VALID |
                        PMIC_CRITICAL_SECTION_STOP_VALID |
                        PMIC_CRC_ENABLE_VALID |
                        PMIC_CONFIG_CRC_ENABLE_VALID |
                        PMIC_IRQ_RESPONSE_CALLBACK_VALID),
        .commMode = PMIC_INTF_I2C_SINGLE,
        .i2cAddr0 = PLATFORM_TARGET_I2C_ADDR,
        .crcEnable = PMIC_DISABLE,
        .configCrcEnable = PMIC_DISABLE,
        .commHandle0 = platform_getCommHandle(),
        .ioRead = &platform_rxByte,
        .ioWrite = &platform_txByte,
        .criticalSectionStart = &platform_critSecStart,
        .criticalSectionStop = &platform_critSecStop,
        .irqResponseCallback = &platform_irqResponse
    };

    platform_init();

    platform_printString("\r\n");
    platform_printString("POWER_TEST\r\n");
    platform_printString("-------\r\n\r\n");

    status = Pmic_init(&pmicHandle, &coreCfg);

    if (status == PMIC_ST_SUCCESS)
    {
        platform_setupTests();
        POWER_TEST_RUN_ALL();
        platform_tearDownTests();
    }
    else
    {
        (void)sprintf(msg, "Error in initializing PMIC LLD: %d\r\n", status);
        platform_printString(msg);
    }

    (void)Pmic_deinit(&pmicHandle);
    platform_deinit();
}

void test_neg_power_pwrSetResourceEnable_nullParam_handle(void)
{
    // Pass NULL handle into Pmic_pwrSetResourceEnable()
    int32_t status = Pmic_pwrSetResourceEnable(NULL, PMIC_PWR_RSRC_BUCK1, PMIC_ENABLE);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_power_pwrSetResourceEnable_outOfBounds_resource(void)
{
    // Pass out of bounds resource into Pmic_pwrSetResourceEnable()
    int32_t status = Pmic_pwrSetResourceEnable(&pmicHandle, PMIC_PWR_RSRC_MAX + 1U, PMIC_ENABLE);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrGetResourceEnable_nullParam_handle(void)
{
    // Pass NULL handle into Pmic_pwrGetResourceEnable()
    bool isEnabled = (bool)false;
    int32_t status = Pmic_pwrGetResourceEnable(NULL, PMIC_PWR_RSRC_BUCK1, &isEnabled);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_power_pwrGetResourceEnable_nullParam_isEnabled(void)
{
    // Pass NULL isEnabled into Pmic_pwrGetResourceEnable()
    int32_t status = Pmic_pwrGetResourceEnable(&pmicHandle, PMIC_PWR_RSRC_BUCK1, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_power_pwrGetResourceEnable_outOfBounds_resource(void)
{
    // Pass out of bounds resource into Pmic_pwrGetResourceEnable()
    bool isEnabled = (bool)false;
    int32_t status = Pmic_pwrGetResourceEnable(&pmicHandle, PMIC_PWR_RSRC_MAX + 1U, &isEnabled);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrSetResourceCfg_nullParam_handle(void)
{
    // Pass NULL handle into Pmic_pwrSetResourceCfg()
    Pmic_PowerResourceCfg_t resourceCfg = {
        .validParams = PMIC_PWR_CFG_ENABLE_VALID,
        .resource = PMIC_PWR_RSRC_BUCK1,
        .enable = PMIC_DISABLE
    };
    int32_t status = Pmic_pwrSetResourceCfg(NULL, &resourceCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_power_pwrSetResourceCfg_nullParam_config(void)
{
    // Pass NULL config into Pmic_pwrSetResourceCfg()
    int32_t status = Pmic_pwrSetResourceCfg(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_power_pwrSetResourceCfg_outOfBounds_resource(void)
{
    // Pass out of bounds resource into Pmic_pwrSetResourceCfg()
    Pmic_PowerResourceCfg_t resourceCfg = {
        .validParams = PMIC_PWR_CFG_ENABLE_VALID,
        .resource = PMIC_PWR_RSRC_MAX + 1U,
        .enable = PMIC_DISABLE
    };
    int32_t status = Pmic_pwrSetResourceCfg(&pmicHandle, &resourceCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

static void powerTest_Pmic_pwrSetResourceCfg_invalidParam_buckMode(uint8_t buck)
{
    Pmic_PowerResourceCfg_t resourceCfg = {
        .validParams = PMIC_PWR_CFG_MODE_VALID,
        .resource = buck
    };

    // Pass invalid mode into Pmic_pwrSetResourceCfg() for Buck
    for (uint8_t mode = PMIC_PWR_RSRC_MODE_MIN; mode <= PMIC_PWR_RSRC_MODE_MAX; mode++)
    {
        if (mode != PMIC_PWR_RSRC_MODE_REG)
        {
            resourceCfg.mode = mode;
            int32_t status = Pmic_pwrSetResourceCfg(&pmicHandle, &resourceCfg);
            PLATFORM_ASSERT(status == PMIC_ST_ERR_NOT_SUPPORTED);
        }
    }
}

static void powerTest_Pmic_pwrSetResourceCfg_outOfBounds_buckIlim(uint8_t buck)
{
    Pmic_PowerResourceCfg_t resourceCfg = {
        .validParams = PMIC_PWR_CFG_ILIM_VALID,
        .resource = buck,
        .ilim = PMIC_PWR_ILIM_MAX + 1U
    };

    // Pass out of bounds ilim into Pmic_pwrSetResourceCfg() for Buck
    int32_t status = Pmic_pwrSetResourceCfg(&pmicHandle, &resourceCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

static void powerTest_Pmic_pwrSetResourceCfg_outOfBounds_buckVoltage_mV(uint8_t buck)
{
    Pmic_PowerResourceCfg_t resourceCfg = {
        .validParams = PMIC_PWR_CFG_VOLTAGE_VALID,
        .resource = buck
    };

    // Pass out of bounds voltage_mV into Pmic_pwrSetResourceCfg() for Buck
    resourceCfg.voltage_mV = 880U; // Under range
    int32_t status = Pmic_pwrSetResourceCfg(&pmicHandle, &resourceCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
    resourceCfg.voltage_mV = 1920U; // Over range
    status = Pmic_pwrSetResourceCfg(&pmicHandle, &resourceCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);

    // Additionally, pass a value that is not a multiple of 20
    resourceCfg.voltage_mV = 1001U;
    status = Pmic_pwrSetResourceCfg(&pmicHandle, &resourceCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

static void powerTest_Pmic_pwrSetResourceCfg_outOfBounds_buckDeglitch(uint8_t buck)
{
    // Pass out of bounds deglitch into Pmic_pwrSetResourceCfg() for Buck
    Pmic_PowerResourceCfg_t resourceCfg = {
        .validParams = PMIC_PWR_CFG_DEGLITCH_VALID,
        .resource = buck,
        .deglitch = PMIC_PWR_DEGLITCH_MAX + 1U
    };
    int32_t status = Pmic_pwrSetResourceCfg(&pmicHandle, &resourceCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

static void powerTest_Pmic_pwrSetResourceCfg_outOfBounds_buckUvThresh(uint8_t buck)
{
    // Pass out of bounds uvThresh into Pmic_pwrSetResourceCfg() for Buck
    Pmic_PowerResourceCfg_t resourceCfg = {
        .validParams = PMIC_PWR_CFG_UV_THRESH_VALID,
        .resource = buck,
        .uvThresh = PMIC_PWR_BUCK_UV_OV_THR_MAX + 1U
    };
    int32_t status = Pmic_pwrSetResourceCfg(&pmicHandle, &resourceCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

static void powerTest_Pmic_pwrSetResourceCfg_outOfBounds_buckUvReaction(uint8_t buck)
{
    // Pass out of bounds uvReaction into Pmic_pwrSetResourceCfg() for Buck
    Pmic_PowerResourceCfg_t resourceCfg = {
        .validParams = PMIC_PWR_CFG_UV_REACT_VALID,
        .resource = buck,
        .uvReaction = PMIC_PWR_FAULT_REACT_MAX + 1U
    };
    int32_t status = Pmic_pwrSetResourceCfg(&pmicHandle, &resourceCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

static void powerTest_Pmic_pwrSetResourceCfg_outOfBounds_buckOvThresh(uint8_t buck)
{
    // Pass out of bounds ovThresh into Pmic_pwrSetResourceCfg() for Buck
    Pmic_PowerResourceCfg_t resourceCfg = {
        .validParams = PMIC_PWR_CFG_OV_THRESH_VALID,
        .resource = buck,
        .ovThresh = PMIC_PWR_BUCK_UV_OV_THR_MAX + 1U
    };
    int32_t status = Pmic_pwrSetResourceCfg(&pmicHandle, &resourceCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

static void powerTest_Pmic_pwrSetResourceCfg_outOfBounds_buckOvReaction(uint8_t buck)
{
    // Pass out of bounds ovReaction into Pmic_pwrSetResourceCfg() for Buck
    Pmic_PowerResourceCfg_t resourceCfg = {
        .validParams = PMIC_PWR_CFG_OV_REACT_VALID,
        .resource = buck,
        .ovReaction = PMIC_PWR_FAULT_REACT_MAX + 1U
    };
    int32_t status = Pmic_pwrSetResourceCfg(&pmicHandle, &resourceCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

static void powerTest_Pmic_pwrSetResourceCfg_outOfBounds_buckRvReaction(uint8_t buck)
{
    // Pass out of bounds rvReaction into Pmic_pwrSetResourceCfg() for Buck
    Pmic_PowerResourceCfg_t resourceCfg = {
        .validParams = PMIC_PWR_CFG_RV_REACT_VALID,
        .resource = buck,
        .rvReaction = PMIC_PWR_RV_REACT_MAX + 1U
    };
    int32_t status = Pmic_pwrSetResourceCfg(&pmicHandle, &resourceCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

static void powerTest_Pmic_pwrSetResourceCfg_outOfBounds_buckScReaction(uint8_t buck)
{
    // Pass out of bounds scReaction into Pmic_pwrSetResourceCfg() for Buck
    Pmic_PowerResourceCfg_t resourceCfg = {
        .validParams = PMIC_PWR_CFG_SC_REACT_VALID,
        .resource = buck,
        .scReaction = PMIC_PWR_FAULT_REACT_MAX + 1U
    };
    int32_t status = Pmic_pwrSetResourceCfg(&pmicHandle, &resourceCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrSetResourceCfg_invalidParam_buck1_mode(void)
{
    powerTest_Pmic_pwrSetResourceCfg_invalidParam_buckMode(PMIC_PWR_RSRC_BUCK1);
}

void test_neg_power_pwrSetResourceCfg_outOfBounds_buck1_ilim(void)
{
    powerTest_Pmic_pwrSetResourceCfg_outOfBounds_buckIlim(PMIC_PWR_RSRC_BUCK1);
}

void test_neg_power_pwrSetResourceCfg_outOfBounds_buck1_voltage_mV(void)
{
    powerTest_Pmic_pwrSetResourceCfg_outOfBounds_buckVoltage_mV(PMIC_PWR_RSRC_BUCK1);
}

void test_neg_power_pwrSetResourceCfg_outOfBounds_buck1_deglitch(void)
{
    powerTest_Pmic_pwrSetResourceCfg_outOfBounds_buckDeglitch(PMIC_PWR_RSRC_BUCK1);
}

void test_neg_power_pwrSetResourceCfg_outOfBounds_buck1_uvThresh(void)
{
    powerTest_Pmic_pwrSetResourceCfg_outOfBounds_buckUvThresh(PMIC_PWR_RSRC_BUCK1);
}

void test_neg_power_pwrSetResourceCfg_outOfBounds_buck1_uvReaction(void)
{
    powerTest_Pmic_pwrSetResourceCfg_outOfBounds_buckUvReaction(PMIC_PWR_RSRC_BUCK1);
}

void test_neg_power_pwrSetResourceCfg_outOfBounds_buck1_ovThresh(void)
{
    powerTest_Pmic_pwrSetResourceCfg_outOfBounds_buckOvThresh(PMIC_PWR_RSRC_BUCK1);
}

void test_neg_power_pwrSetResourceCfg_outOfBounds_buck1_ovReaction(void)
{
    powerTest_Pmic_pwrSetResourceCfg_outOfBounds_buckOvReaction(PMIC_PWR_RSRC_BUCK1);
}

void test_neg_power_pwrSetResourceCfg_outOfBounds_buck1_rvReaction(void)
{
    powerTest_Pmic_pwrSetResourceCfg_outOfBounds_buckRvReaction(PMIC_PWR_RSRC_BUCK1);
}

void test_neg_power_pwrSetResourceCfg_outOfBounds_buck1_scReaction(void)
{
    powerTest_Pmic_pwrSetResourceCfg_outOfBounds_buckScReaction(PMIC_PWR_RSRC_BUCK1);
}

void test_neg_power_pwrSetResourceCfg_invalidParam_buck2_mode(void)
{
    powerTest_Pmic_pwrSetResourceCfg_invalidParam_buckMode(PMIC_PWR_RSRC_BUCK2);
}

void test_neg_power_pwrSetResourceCfg_outOfBounds_buck2_ilim(void)
{
    powerTest_Pmic_pwrSetResourceCfg_outOfBounds_buckIlim(PMIC_PWR_RSRC_BUCK2);
}

void test_neg_power_pwrSetResourceCfg_outOfBounds_buck2_voltage_mV(void)
{
    powerTest_Pmic_pwrSetResourceCfg_outOfBounds_buckVoltage_mV(PMIC_PWR_RSRC_BUCK2);
}

void test_neg_power_pwrSetResourceCfg_outOfBounds_buck2_deglitch(void)
{
    powerTest_Pmic_pwrSetResourceCfg_outOfBounds_buckDeglitch(PMIC_PWR_RSRC_BUCK2);
}

void test_neg_power_pwrSetResourceCfg_outOfBounds_buck2_uvThresh(void)
{
    powerTest_Pmic_pwrSetResourceCfg_outOfBounds_buckUvThresh(PMIC_PWR_RSRC_BUCK2);
}

void test_neg_power_pwrSetResourceCfg_outOfBounds_buck2_uvReaction(void)
{
    powerTest_Pmic_pwrSetResourceCfg_outOfBounds_buckUvReaction(PMIC_PWR_RSRC_BUCK2);
}

void test_neg_power_pwrSetResourceCfg_outOfBounds_buck2_ovThresh(void)
{
    powerTest_Pmic_pwrSetResourceCfg_outOfBounds_buckOvThresh(PMIC_PWR_RSRC_BUCK2);
}

void test_neg_power_pwrSetResourceCfg_outOfBounds_buck2_ovReaction(void)
{
    powerTest_Pmic_pwrSetResourceCfg_outOfBounds_buckOvReaction(PMIC_PWR_RSRC_BUCK2);
}

void test_neg_power_pwrSetResourceCfg_outOfBounds_buck2_rvReaction(void)
{
    powerTest_Pmic_pwrSetResourceCfg_outOfBounds_buckRvReaction(PMIC_PWR_RSRC_BUCK2);
}

void test_neg_power_pwrSetResourceCfg_outOfBounds_buck2_scReaction(void)
{
    powerTest_Pmic_pwrSetResourceCfg_outOfBounds_buckScReaction(PMIC_PWR_RSRC_BUCK2);
}

void test_neg_power_pwrSetResourceCfg_invalidParam_buck3_mode(void)
{
    powerTest_Pmic_pwrSetResourceCfg_invalidParam_buckMode(PMIC_PWR_RSRC_BUCK3);
}

void test_neg_power_pwrSetResourceCfg_outOfBounds_buck3_ilim(void)
{
    powerTest_Pmic_pwrSetResourceCfg_outOfBounds_buckIlim(PMIC_PWR_RSRC_BUCK3);
}

void test_neg_power_pwrSetResourceCfg_outOfBounds_buck3_voltage_mV(void)
{
    powerTest_Pmic_pwrSetResourceCfg_outOfBounds_buckVoltage_mV(PMIC_PWR_RSRC_BUCK3);
}

void test_neg_power_pwrSetResourceCfg_outOfBounds_buck3_deglitch(void)
{
    powerTest_Pmic_pwrSetResourceCfg_outOfBounds_buckDeglitch(PMIC_PWR_RSRC_BUCK3);
}

void test_neg_power_pwrSetResourceCfg_outOfBounds_buck3_uvThresh(void)
{
    powerTest_Pmic_pwrSetResourceCfg_outOfBounds_buckUvThresh(PMIC_PWR_RSRC_BUCK3);
}

void test_neg_power_pwrSetResourceCfg_outOfBounds_buck3_uvReaction(void)
{
    powerTest_Pmic_pwrSetResourceCfg_outOfBounds_buckUvReaction(PMIC_PWR_RSRC_BUCK3);
}

void test_neg_power_pwrSetResourceCfg_outOfBounds_buck3_ovThresh(void)
{
    powerTest_Pmic_pwrSetResourceCfg_outOfBounds_buckOvThresh(PMIC_PWR_RSRC_BUCK3);
}

void test_neg_power_pwrSetResourceCfg_outOfBounds_buck3_ovReaction(void)
{
    powerTest_Pmic_pwrSetResourceCfg_outOfBounds_buckOvReaction(PMIC_PWR_RSRC_BUCK3);
}

void test_neg_power_pwrSetResourceCfg_outOfBounds_buck3_rvReaction(void)
{
    powerTest_Pmic_pwrSetResourceCfg_outOfBounds_buckRvReaction(PMIC_PWR_RSRC_BUCK3);
}

void test_neg_power_pwrSetResourceCfg_outOfBounds_buck3_scReaction(void)
{
    powerTest_Pmic_pwrSetResourceCfg_outOfBounds_buckScReaction(PMIC_PWR_RSRC_BUCK3);
}

void test_neg_power_pwrSetResourceCfg_outOfBounds_ldoLs1Vmon1_mode(void)
{
    // Pass out of bounds mode into Pmic_pwrSetResourceCfg() for LDO_LS1_VMON1
    Pmic_PowerResourceCfg_t resourceCfg = {
        .validParams = PMIC_PWR_CFG_MODE_VALID,
        .resource = PMIC_PWR_RSRC_LDO_LS1_VMON1,
        .mode = PMIC_PWR_RSRC_MODE_MAX + 1U
    };
    int32_t status = Pmic_pwrSetResourceCfg(&pmicHandle, &resourceCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrSetResourceCfg_ldoLs1Vmon1_ilim(void)
{
    Pmic_PowerResourceCfg_t resourceCfg = {
        .validParams = PMIC_PWR_CFG_ILIM_VALID,
        .resource = PMIC_PWR_RSRC_LDO_LS1_VMON1
    };

    // Pass all ILIM values into Pmic_pwrSetResourceCfg() for LDO_LS1_VMON1.
    // NOTE: For LDO_LS1_VMON1, ILIM is not a valid parameter
    for (uint8_t ilim = PMIC_PWR_ILIM_MIN; ilim <= PMIC_PWR_ILIM_MAX; ilim++)
    {
        resourceCfg.ilim = ilim;
        int32_t status = Pmic_pwrSetResourceCfg(&pmicHandle, &resourceCfg);
        PLATFORM_ASSERT(status == PMIC_ST_ERR_NOT_SUPPORTED);
    }
}

static void powerTest_Pmic_pwrSetResourceCfg_outOfBounds_ldoLsVmon_voltage_mV(uint8_t ldoLsVmon)
{
    Pmic_PowerResourceCfg_t resourceCfg = {
        .validParams = PMIC_PWR_CFG_VOLTAGE_VALID,
        .resource = ldoLsVmon
    };

    // Pass out of bounds voltage_mV into Pmic_pwrSetResourceCfg() for LDO/LS/VMON
    resourceCfg.voltage_mV = 575U; // Under range
    int32_t status = Pmic_pwrSetResourceCfg(&pmicHandle, &resourceCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
    resourceCfg.voltage_mV = 3425U; // Over range
    status = Pmic_pwrSetResourceCfg(&pmicHandle, &resourceCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);

    // Additionally, pass a value that is not a multiple of 25
    resourceCfg.voltage_mV = 1001U;
    status = Pmic_pwrSetResourceCfg(&pmicHandle, &resourceCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrSetResourceCfg_outOfBounds_ldoLs1Vmon1_voltage_mV(void)
{
    powerTest_Pmic_pwrSetResourceCfg_outOfBounds_ldoLsVmon_voltage_mV(PMIC_PWR_RSRC_LDO_LS1_VMON1);
}

void test_neg_power_pwrSetResourceCfg_outOfBounds_ldoLs1Vmon1_deglitch(void)
{
    // Pass out of bounds deglitch into Pmic_pwrSetResourceCfg() for LDO_LS1_VMON1
    Pmic_PowerResourceCfg_t resourceCfg = {
        .validParams = PMIC_PWR_CFG_DEGLITCH_VALID,
        .resource = PMIC_PWR_RSRC_LDO_LS1_VMON1,
        .deglitch = PMIC_PWR_DEGLITCH_MAX + 1U
    };
    int32_t status = Pmic_pwrSetResourceCfg(&pmicHandle, &resourceCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrSetResourceCfg_outOfBounds_ldoLs1Vmon1_uvThresh(void)
{
    // Pass out of bounds uvThresh into Pmic_pwrSetResourceCfg() for LDO_LS1_VMON1
    Pmic_PowerResourceCfg_t resourceCfg = {
        .validParams = PMIC_PWR_CFG_UV_THRESH_VALID,
        .resource = PMIC_PWR_RSRC_LDO_LS1_VMON1,
        .uvThresh = PMIC_PWR_LS_UV_OV_THR_MAX + 1U
    };
    int32_t status = Pmic_pwrSetResourceCfg(&pmicHandle, &resourceCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrSetResourceCfg_outOfBounds_ldoLs1Vmon1_uvReaction(void)
{
    // Pass out of bounds uvReaction into Pmic_pwrSetResourceCfg() for LDO_LS1_VMON1
    Pmic_PowerResourceCfg_t resourceCfg = {
        .validParams = PMIC_PWR_CFG_UV_REACT_VALID,
        .resource = PMIC_PWR_RSRC_LDO_LS1_VMON1,
        .uvReaction = PMIC_PWR_FAULT_REACT_MAX + 1U
    };
    int32_t status = Pmic_pwrSetResourceCfg(&pmicHandle, &resourceCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrSetResourceCfg_outOfBounds_ldoLs1Vmon1_ovThresh(void)
{
    // Pass out of bounds ovThresh into Pmic_pwrSetResourceCfg() for LDO_LS1_VMON1
    Pmic_PowerResourceCfg_t resourceCfg = {
        .validParams = PMIC_PWR_CFG_OV_THRESH_VALID,
        .resource = PMIC_PWR_RSRC_LDO_LS1_VMON1,
        .ovThresh = PMIC_PWR_LS_UV_OV_THR_MAX + 1U
    };
    int32_t status = Pmic_pwrSetResourceCfg(&pmicHandle, &resourceCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrSetResourceCfg_outOfBounds_ldoLs1Vmon1_ovReaction(void)
{
    // Pass out of bounds ovReaction into Pmic_pwrSetResourceCfg() for LDO_LS1_VMON1
    Pmic_PowerResourceCfg_t resourceCfg = {
        .validParams = PMIC_PWR_CFG_OV_REACT_VALID,
        .resource = PMIC_PWR_RSRC_LDO_LS1_VMON1,
        .ovReaction = PMIC_PWR_FAULT_REACT_MAX + 1U
    };
    int32_t status = Pmic_pwrSetResourceCfg(&pmicHandle, &resourceCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrSetResourceCfg_outOfBounds_ldoLs1Vmon1_rvReaction(void)
{
    // Pass out of bounds rvReaction into Pmic_pwrSetResourceCfg() for LDO_LS1_VMON1
    Pmic_PowerResourceCfg_t resourceCfg = {
        .validParams = PMIC_PWR_CFG_RV_REACT_VALID,
        .resource = PMIC_PWR_RSRC_LDO_LS1_VMON1,
        .rvReaction = PMIC_PWR_RV_REACT_MAX + 1U
    };
    int32_t status = Pmic_pwrSetResourceCfg(&pmicHandle, &resourceCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrSetResourceCfg_outOfBounds_ldoLs1Vmon1_scReaction(void)
{
    // Pass out of bounds scReaction into Pmic_pwrSetResourceCfg() for LDO_LS1_VMON1
    Pmic_PowerResourceCfg_t resourceCfg = {
        .validParams = PMIC_PWR_CFG_SC_REACT_VALID,
        .resource = PMIC_PWR_RSRC_LDO_LS1_VMON1,
        .scReaction = PMIC_PWR_FAULT_REACT_MAX + 1U
    };
    int32_t status = Pmic_pwrSetResourceCfg(&pmicHandle, &resourceCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrSetResourceCfg_invalidParam_ls2Vmon2_mode(void)
{
    Pmic_PowerResourceCfg_t resourceCfg = {
        .validParams = PMIC_PWR_CFG_MODE_VALID,
        .resource = PMIC_PWR_RSRC_LS2_VMON2
    };

    // Pass invalid mode into Pmic_pwrSetResourceCfg() for LS2_VMON2
    for (uint8_t mode = PMIC_PWR_RSRC_MODE_MIN; mode <= PMIC_PWR_RSRC_MODE_MAX; mode++)
    {
        if ((mode != PMIC_PWR_RSRC_MODE_LSW) && (mode != PMIC_PWR_RSRC_MODE_VMON))
        {
            resourceCfg.mode = mode;
            int32_t status = Pmic_pwrSetResourceCfg(&pmicHandle, &resourceCfg);
            PLATFORM_ASSERT(status == PMIC_ST_ERR_NOT_SUPPORTED);
        }
    }
}

void test_neg_power_pwrSetResourceCfg_ls2Vmon2_ilim(void)
{
    Pmic_PowerResourceCfg_t resourceCfg = {
        .validParams = PMIC_PWR_CFG_ILIM_VALID,
        .resource = PMIC_PWR_RSRC_LS2_VMON2
    };

    // Pass all ILIM values into Pmic_pwrSetResourceCfg() for LS2_VMON2.
    // NOTE: For LS2_VMON2, ILIM is not a valid parameter
    for (uint8_t ilim = PMIC_PWR_ILIM_MIN; ilim <= PMIC_PWR_ILIM_MAX; ilim++)
    {
        resourceCfg.ilim = ilim;
        int32_t status = Pmic_pwrSetResourceCfg(&pmicHandle, &resourceCfg);
        PLATFORM_ASSERT(status == PMIC_ST_ERR_NOT_SUPPORTED);
    }
}

void test_neg_power_pwrSetResourceCfg_outOfBounds_ls2Vmon2_voltage_mV(void)
{
    powerTest_Pmic_pwrSetResourceCfg_outOfBounds_ldoLsVmon_voltage_mV(PMIC_PWR_RSRC_LS2_VMON2);
}

void test_neg_power_pwrSetResourceCfg_outOfBounds_ls2Vmon2_deglitch(void)
{
    // Pass out of bounds deglitch into Pmic_pwrSetResourceCfg() for LS2_VMON2
    Pmic_PowerResourceCfg_t resourceCfg = {
        .validParams = PMIC_PWR_CFG_DEGLITCH_VALID,
        .resource = PMIC_PWR_RSRC_LS2_VMON2,
        .deglitch = PMIC_PWR_DEGLITCH_MAX + 1U
    };
    int32_t status = Pmic_pwrSetResourceCfg(&pmicHandle, &resourceCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrSetResourceCfg_outOfBounds_ls2Vmon2_uvThresh(void)
{
    // Pass out of bounds uvThresh into Pmic_pwrSetResourceCfg() for LS2_VMON2
    Pmic_PowerResourceCfg_t resourceCfg = {
        .validParams = PMIC_PWR_CFG_UV_THRESH_VALID,
        .resource = PMIC_PWR_RSRC_LS2_VMON2,
        .uvThresh = PMIC_PWR_LS_UV_OV_THR_MAX + 1U
    };
    int32_t status = Pmic_pwrSetResourceCfg(&pmicHandle, &resourceCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrSetResourceCfg_outOfBounds_ls2Vmon2_uvReaction(void)
{
    // Pass out of bounds uvReaction into Pmic_pwrSetResourceCfg() for LS2_VMON2
    Pmic_PowerResourceCfg_t resourceCfg = {
        .validParams = PMIC_PWR_CFG_UV_REACT_VALID,
        .resource = PMIC_PWR_RSRC_LS2_VMON2,
        .uvReaction = PMIC_PWR_FAULT_REACT_MAX + 1U
    };
    int32_t status = Pmic_pwrSetResourceCfg(&pmicHandle, &resourceCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrSetResourceCfg_outOfBounds_ls2Vmon2_ovThresh(void)
{
    // Pass out of bounds ovThresh into Pmic_pwrSetResourceCfg() for LS2_VMON2
    Pmic_PowerResourceCfg_t resourceCfg = {
        .validParams = PMIC_PWR_CFG_OV_THRESH_VALID,
        .resource = PMIC_PWR_RSRC_LS2_VMON2,
        .ovThresh = PMIC_PWR_LS_UV_OV_THR_MAX + 1U
    };
    int32_t status = Pmic_pwrSetResourceCfg(&pmicHandle, &resourceCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrSetResourceCfg_outOfBounds_ls2Vmon2_ovReaction(void)
{
    // Pass out of bounds ovReaction into Pmic_pwrSetResourceCfg() for LS2_VMON2
    Pmic_PowerResourceCfg_t resourceCfg = {
        .validParams = PMIC_PWR_CFG_OV_REACT_VALID,
        .resource = PMIC_PWR_RSRC_LS2_VMON2,
        .ovReaction = PMIC_PWR_FAULT_REACT_MAX + 1U
    };
    int32_t status = Pmic_pwrSetResourceCfg(&pmicHandle, &resourceCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrSetResourceCfg_outOfBounds_ls2Vmon2_rvReaction(void)
{
    // Pass out of bounds rvReaction into Pmic_pwrSetResourceCfg() for LS2_VMON2
    Pmic_PowerResourceCfg_t resourceCfg = {
        .validParams = PMIC_PWR_CFG_RV_REACT_VALID,
        .resource = PMIC_PWR_RSRC_LS2_VMON2,
        .rvReaction = PMIC_PWR_RV_REACT_MAX + 1U
    };
    int32_t status = Pmic_pwrSetResourceCfg(&pmicHandle, &resourceCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrSetResourceCfg_outOfBounds_ls2Vmon2_scReaction(void)
{
    // Pass out of bounds scReaction into Pmic_pwrSetResourceCfg() for LS2_VMON2
    Pmic_PowerResourceCfg_t resourceCfg = {
        .validParams = PMIC_PWR_CFG_SC_REACT_VALID,
        .resource = PMIC_PWR_RSRC_LS2_VMON2,
        .scReaction = PMIC_PWR_FAULT_REACT_MAX + 1U
    };
    int32_t status = Pmic_pwrSetResourceCfg(&pmicHandle, &resourceCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrSetResourceCfg_invalidParam_vccaVmon_mode(void)
{
    Pmic_PowerResourceCfg_t resourceCfg = {
        .validParams = PMIC_PWR_CFG_MODE_VALID,
        .resource = PMIC_PWR_RSRC_VCCA_VMON
    };

    // Pass invalid mode into Pmic_pwrSetResourceCfg() for VCCA_VMON
    for (uint8_t mode = 0U; mode <= PMIC_PWR_RSRC_MODE_MAX; mode++)
    {
        if (mode != PMIC_PWR_RSRC_MODE_VMON)
        {
            resourceCfg.mode = mode;
            int32_t status = Pmic_pwrSetResourceCfg(&pmicHandle, &resourceCfg);
            PLATFORM_ASSERT(status == PMIC_ST_ERR_NOT_SUPPORTED);
        }
    }
}

void test_neg_power_pwrSetResourceCfg_vccaVmon_ilim(void)
{
    Pmic_PowerResourceCfg_t resourceCfg = {
        .validParams = PMIC_PWR_CFG_ILIM_VALID,
        .resource = PMIC_PWR_RSRC_VCCA_VMON
    };

    // Pass all ILIM values into Pmic_pwrSetResourceCfg() for VCCA_VMON.
    // NOTE: For VCCA_VMON, ILIM is not a valid parameter
    for (uint8_t ilim = PMIC_PWR_ILIM_MIN; ilim <= PMIC_PWR_ILIM_MAX; ilim++)
    {
        resourceCfg.ilim = ilim;
        int32_t status = Pmic_pwrSetResourceCfg(&pmicHandle, &resourceCfg);
        PLATFORM_ASSERT(status == PMIC_ST_ERR_NOT_SUPPORTED);
    }
}

void test_neg_power_pwrSetResourceCfg_outOfBounds_vccaVmon_voltage_mV(void)
{
    powerTest_Pmic_pwrSetResourceCfg_outOfBounds_ldoLsVmon_voltage_mV(PMIC_PWR_RSRC_VCCA_VMON);
}

void test_neg_power_pwrSetResourceCfg_outOfBounds_vccaVmon_deglitch(void)
{
    // Pass out of bounds deglitch into Pmic_pwrSetResourceCfg() for VCCA_VMON
    Pmic_PowerResourceCfg_t resourceCfg = {
        .validParams = PMIC_PWR_CFG_DEGLITCH_VALID,
        .resource = PMIC_PWR_RSRC_VCCA_VMON,
        .deglitch = PMIC_PWR_DEGLITCH_MAX + 1U
    };
    int32_t status = Pmic_pwrSetResourceCfg(&pmicHandle, &resourceCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrSetResourceCfg_outOfBounds_vccaVmon_uvThresh(void)
{
    // Pass out of bounds uvThresh into Pmic_pwrSetResourceCfg() for VCCA_VMON
    Pmic_PowerResourceCfg_t resourceCfg = {
        .validParams = PMIC_PWR_CFG_UV_THRESH_VALID,
        .resource = PMIC_PWR_RSRC_VCCA_VMON,
        .uvThresh = PMIC_PWR_VCCA_UV_OV_THR_MAX + 1U
    };
    int32_t status = Pmic_pwrSetResourceCfg(&pmicHandle, &resourceCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrSetResourceCfg_outOfBounds_vccaVmon_uvReaction(void)
{
    // Pass out of bounds uvReaction into Pmic_pwrSetResourceCfg() for VCCA_VMON
    Pmic_PowerResourceCfg_t resourceCfg = {
        .validParams = PMIC_PWR_CFG_UV_REACT_VALID,
        .resource = PMIC_PWR_RSRC_VCCA_VMON,
        .uvReaction = PMIC_PWR_FAULT_REACT_MAX + 1U
    };
    int32_t status = Pmic_pwrSetResourceCfg(&pmicHandle, &resourceCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrSetResourceCfg_outOfBounds_vccaVmon_ovThresh(void)
{
    // Pass out of bounds ovThresh into Pmic_pwrSetResourceCfg() for VCCA_VMON
    Pmic_PowerResourceCfg_t resourceCfg = {
        .validParams = PMIC_PWR_CFG_OV_THRESH_VALID,
        .resource = PMIC_PWR_RSRC_VCCA_VMON,
        .ovThresh = PMIC_PWR_VCCA_UV_OV_THR_MAX + 1U
    };
    int32_t status = Pmic_pwrSetResourceCfg(&pmicHandle, &resourceCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrSetResourceCfg_outOfBounds_vccaVmon_ovReaction(void)
{
    // Pass out of bounds ovReaction into Pmic_pwrSetResourceCfg() for VCCA_VMON
    Pmic_PowerResourceCfg_t resourceCfg = {
        .validParams = PMIC_PWR_CFG_OV_REACT_VALID,
        .resource = PMIC_PWR_RSRC_VCCA_VMON,
        .ovReaction = PMIC_PWR_FAULT_REACT_MAX + 1U
    };
    int32_t status = Pmic_pwrSetResourceCfg(&pmicHandle, &resourceCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrSetResourceCfg_vccaVmon_rvReaction(void)
{
    Pmic_PowerResourceCfg_t resourceCfg = {
        .validParams = PMIC_PWR_CFG_RV_REACT_VALID,
        .resource = PMIC_PWR_RSRC_VCCA_VMON
    };

    // Pass all rvReaction values into Pmic_pwrSetResourceCfg() for VCCA_VMON.
    // NOTE: For VCCA_VMON, rvReaction is not a valid parameter
    for (uint8_t rvReaction = PMIC_PWR_RV_REACT_MIN; rvReaction <= PMIC_PWR_RV_REACT_MAX; rvReaction++)
    {
        resourceCfg.rvReaction = rvReaction;
        int32_t status = Pmic_pwrSetResourceCfg(&pmicHandle, &resourceCfg);
        PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
    }
}

void test_neg_power_pwrSetResourceCfg_vccaVmon_scReaction(void)
{
    Pmic_PowerResourceCfg_t resourceCfg = {
        .validParams = PMIC_PWR_CFG_SC_REACT_VALID,
        .resource = PMIC_PWR_RSRC_VCCA_VMON
    };

    // Pass all scReaction values into Pmic_pwrSetResourceCfg() for VCCA_VMON.
    // NOTE: For VCCA_VMON, scReaction is not a valid parameter
    for (uint8_t scReaction = PMIC_PWR_FAULT_REACT_MIN; scReaction <= PMIC_PWR_FAULT_REACT_MAX; scReaction++)
    {
        resourceCfg.scReaction = scReaction;
        int32_t status = Pmic_pwrSetResourceCfg(&pmicHandle, &resourceCfg);
        PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
    }
}

void test_neg_power_pwrSetResourceCfg_gpo_mode(void)
{
    Pmic_PowerResourceCfg_t resourceCfg = {
        .validParams = PMIC_PWR_CFG_MODE_VALID,
        .resource = PMIC_PWR_RSRC_GPO
    };

    // Pass invalid mode into Pmic_pwrSetResourceCfg() for GPO.
    // NOTE: For GPO, mode is not a valid parameter
    for (uint8_t mode = 0U; mode <= PMIC_PWR_RSRC_MODE_MAX; mode++)
    {
        resourceCfg.mode = mode;
        int32_t status = Pmic_pwrSetResourceCfg(&pmicHandle, &resourceCfg);
        PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
    }
}

void test_neg_power_pwrSetResourceCfg_gpo_ilim(void)
{
    Pmic_PowerResourceCfg_t resourceCfg = {
        .validParams = PMIC_PWR_CFG_ILIM_VALID,
        .resource = PMIC_PWR_RSRC_GPO
    };

    // Pass all ILIM values into Pmic_pwrSetResourceCfg() for GPO.
    // NOTE: For GPO, ILIM is not a valid parameter
    for (uint8_t ilim = PMIC_PWR_ILIM_MIN; ilim <= PMIC_PWR_ILIM_MAX; ilim++)
    {
        resourceCfg.ilim = ilim;
        int32_t status = Pmic_pwrSetResourceCfg(&pmicHandle, &resourceCfg);
        PLATFORM_ASSERT(status == PMIC_ST_ERR_NOT_SUPPORTED);
    }
}

void test_neg_power_pwrSetResourceCfg_gpo_voltage_mV(void)
{
    // Pass voltage_mV into Pmic_pwrSetResourceCfg() for GPO.
    // NOTE: For GPO, voltage_mV is not a valid parameter
    Pmic_PowerResourceCfg_t resourceCfg = {
        .validParams = PMIC_PWR_CFG_VOLTAGE_VALID,
        .resource = PMIC_PWR_RSRC_GPO,
        .voltage_mV = 1000U
    };
    int32_t status = Pmic_pwrSetResourceCfg(&pmicHandle, &resourceCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrSetResourceCfg_gpo_deglitch(void)
{
    Pmic_PowerResourceCfg_t resourceCfg = {
        .validParams = PMIC_PWR_CFG_DEGLITCH_VALID,
        .resource = PMIC_PWR_RSRC_GPO
    };

    // Pass all deglitch values into Pmic_pwrSetResourceCfg() for GPO.
    // NOTE: For GPO, deglitch is not a valid parameter
    for (uint8_t deglitch = PMIC_PWR_DEGLITCH_MIN; deglitch <= PMIC_PWR_DEGLITCH_MAX; deglitch++)
    {
        resourceCfg.deglitch = deglitch;
        int32_t status = Pmic_pwrSetResourceCfg(&pmicHandle, &resourceCfg);
        PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
    }
}

void test_neg_power_pwrSetResourceCfg_gpo_uvThresh(void)
{
    // Pass uvThresh into Pmic_pwrSetResourceCfg() for GPO.
    // NOTE: For GPO, uvThresh is not a valid parameter
    Pmic_PowerResourceCfg_t resourceCfg = {
        .validParams = PMIC_PWR_CFG_UV_THRESH_VALID,
        .resource = PMIC_PWR_RSRC_GPO,
        .uvThresh = 0U
    };
    int32_t status = Pmic_pwrSetResourceCfg(&pmicHandle, &resourceCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrSetResourceCfg_gpo_uvReaction(void)
{
    // Pass uvReaction into Pmic_pwrSetResourceCfg() for GPO.
    // NOTE: For GPO, uvReaction is not a valid parameter
    Pmic_PowerResourceCfg_t resourceCfg = {
        .validParams = PMIC_PWR_CFG_UV_REACT_VALID,
        .resource = PMIC_PWR_RSRC_GPO,
        .uvReaction = 0U
    };
    int32_t status = Pmic_pwrSetResourceCfg(&pmicHandle, &resourceCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrSetResourceCfg_gpo_ovThresh(void)
{
    // Pass ovThresh into Pmic_pwrSetResourceCfg() for GPO.
    // NOTE: For GPO, ovThresh is not a valid parameter
    Pmic_PowerResourceCfg_t resourceCfg = {
        .validParams = PMIC_PWR_CFG_OV_THRESH_VALID,
        .resource = PMIC_PWR_RSRC_GPO,
        .ovThresh = 0U
    };
    int32_t status = Pmic_pwrSetResourceCfg(&pmicHandle, &resourceCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrSetResourceCfg_gpo_ovReaction(void)
{
    // Pass ovReaction into Pmic_pwrSetResourceCfg() for GPO.
    // NOTE: For GPO, ovReaction is not a valid parameter
    Pmic_PowerResourceCfg_t resourceCfg = {
        .validParams = PMIC_PWR_CFG_OV_REACT_VALID,
        .resource = PMIC_PWR_RSRC_GPO,
        .ovReaction = 0U
    };
    int32_t status = Pmic_pwrSetResourceCfg(&pmicHandle, &resourceCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrSetResourceCfg_gpo_rvReaction(void)
{
    // Pass rvReaction into Pmic_pwrSetResourceCfg() for GPO.
    // NOTE: For GPO, rvReaction is not a valid parameter
    Pmic_PowerResourceCfg_t resourceCfg = {
        .validParams = PMIC_PWR_CFG_RV_REACT_VALID,
        .resource = PMIC_PWR_RSRC_GPO,
        .rvReaction = 0U
    };
    int32_t status = Pmic_pwrSetResourceCfg(&pmicHandle, &resourceCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrSetResourceCfg_gpo_scReaction(void)
{
    // Pass scReaction into Pmic_pwrSetResourceCfg() for GPO.
    // NOTE: For GPO, scReaction is not a valid parameter
    Pmic_PowerResourceCfg_t resourceCfg = {
        .validParams = PMIC_PWR_CFG_SC_REACT_VALID,
        .resource = PMIC_PWR_RSRC_GPO,
        .scReaction = 0U
    };
    int32_t status = Pmic_pwrSetResourceCfg(&pmicHandle, &resourceCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrGetResourceCfg_nullParam_handle(void)
{
    // Pass NULL handle into Pmic_pwrGetResourceCfg()
    Pmic_PowerResourceCfg_t resourceCfg = {
        .validParams = PMIC_PWR_CFG_ENABLE_VALID,
        .resource = PMIC_PWR_RSRC_BUCK1,
        .enable = PMIC_DISABLE
    };
    int32_t status = Pmic_pwrGetResourceCfg(NULL, &resourceCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_power_pwrGetResourceCfg_nullParam_config(void)
{
    // Pass NULL config into Pmic_pwrGetResourceCfg()
    int32_t status = Pmic_pwrGetResourceCfg(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_power_pwrGetResourceCfg_outOfBounds_resource(void)
{
    // Pass out of bounds resource into Pmic_pwrGetResourceCfg()
    Pmic_PowerResourceCfg_t resourceCfg = {
        .validParams = PMIC_PWR_CFG_ENABLE_VALID,
        .resource = PMIC_PWR_RSRC_MAX + 1U,
        .enable = PMIC_DISABLE
    };
    int32_t status = Pmic_pwrGetResourceCfg(&pmicHandle, &resourceCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrSetResourceCfgs_nullParam_handle(void)
{
    // Pass NULL handle into Pmic_pwrSetResourceCfgs()
    const uint8_t numConfig = 1U;
    Pmic_PowerResourceCfg_t resourceCfg = {
        .validParams = PMIC_PWR_CFG_ENABLE_VALID,
        .resource = PMIC_PWR_RSRC_BUCK1,
        .enable = PMIC_DISABLE
    };
    int32_t status = Pmic_pwrSetResourceCfgs(NULL, numConfig, &resourceCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_power_pwrSetResourceCfgs_nullParam_config(void)
{
    // Pass NULL config into Pmic_pwrSetResourceCfgs()
    const uint8_t numConfig = 1U;
    int32_t status = Pmic_pwrSetResourceCfgs(&pmicHandle, numConfig, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_power_pwrSetResourceCfgs_outOfBounds_resource(void)
{
    // Pass out of bounds resource into Pmic_pwrSetResourceCfgs()
    const uint8_t numConfig = 1U;
    Pmic_PowerResourceCfg_t resourceCfg = {
        .validParams = PMIC_PWR_CFG_ENABLE_VALID,
        .resource = PMIC_PWR_RSRC_MAX + 1U,
        .enable = PMIC_DISABLE
    };
    int32_t status = Pmic_pwrSetResourceCfgs(&pmicHandle, numConfig, &resourceCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

static void powerTest_Pmic_pwrSetResourceCfgs_invalidParam_buckMode(uint8_t buck)
{
    const uint8_t numConfig = 1U;
    Pmic_PowerResourceCfg_t resourceCfg = {
        .validParams = PMIC_PWR_CFG_MODE_VALID,
        .resource = buck
    };

    // Pass invalid mode into Pmic_pwrSetResourceCfgs() for buck
    for (uint8_t mode = PMIC_PWR_RSRC_MODE_MIN; mode <= PMIC_PWR_RSRC_MODE_MAX; mode++)
    {
        if (mode != PMIC_PWR_RSRC_MODE_REG)
        {
            resourceCfg.mode = mode;
            int32_t status = Pmic_pwrSetResourceCfgs(&pmicHandle, numConfig, &resourceCfg);
            PLATFORM_ASSERT(status == PMIC_ST_ERR_NOT_SUPPORTED);
        }
    }
}

static void powerTest_Pmic_pwrSetResourceCfgs_outOfBounds_buckIlim(uint8_t buck)
{
    // Pass out of bounds ILIM into Pmic_pwrSetResourceCfgs() for buck
    const uint8_t numConfig = 1U;
    Pmic_PowerResourceCfg_t resourceCfg = {
        .validParams = PMIC_PWR_CFG_ILIM_VALID,
        .resource = buck,
        .ilim = PMIC_PWR_ILIM_MAX + 1U
    };
    int32_t status = Pmic_pwrSetResourceCfgs(&pmicHandle, numConfig, &resourceCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

static void powerTest_Pmic_pwrSetResourceCfgs_outOfBounds_buckVoltage_mV(uint8_t buck)
{
    const uint8_t numConfig = 1U;
    Pmic_PowerResourceCfg_t resourceCfg = {
        .validParams = PMIC_PWR_CFG_VOLTAGE_VALID,
        .resource = buck
    };

    // Pass out of bounds voltage_mV into Pmic_pwrSetResourceCfgs() for buck
    resourceCfg.voltage_mV = 880U; // Under range
    int32_t status = Pmic_pwrSetResourceCfgs(&pmicHandle, numConfig, &resourceCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
    resourceCfg.voltage_mV = 1920U; // Over range
    status = Pmic_pwrSetResourceCfgs(&pmicHandle, numConfig, &resourceCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);

    // Additionally, pass a value that is not a multiple of 20
    resourceCfg.voltage_mV = 1001U;
    status = Pmic_pwrSetResourceCfgs(&pmicHandle, numConfig, &resourceCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

static void powerTest_Pmic_pwrSetResourceCfgs_outOfBounds_buckDeglitch(uint8_t buck)
{
    // Pass out of bounds deglitch into Pmic_pwrSetResourceCfgs() for buck
    const uint8_t numConfig = 1U;
    Pmic_PowerResourceCfg_t resourceCfg = {
        .validParams = PMIC_PWR_CFG_DEGLITCH_VALID,
        .resource = buck,
        .deglitch = PMIC_PWR_DEGLITCH_MAX + 1U
    };
    int32_t status = Pmic_pwrSetResourceCfgs(&pmicHandle, numConfig, &resourceCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

static void powerTest_Pmic_pwrSetResourceCfgs_outOfBounds_buckUvThresh(uint8_t buck)
{
    // Pass out of bounds uvThresh into Pmic_pwrSetResourceCfgs() for buck
    const uint8_t numConfig = 1U;
    Pmic_PowerResourceCfg_t resourceCfg = {
        .validParams = PMIC_PWR_CFG_UV_THRESH_VALID,
        .resource = buck,
        .uvThresh = PMIC_PWR_BUCK_UV_OV_THR_MAX + 1U
    };
    int32_t status = Pmic_pwrSetResourceCfgs(&pmicHandle, numConfig, &resourceCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

static void powerTest_Pmic_pwrSetResourceCfgs_outOfBounds_buckUvReaction(uint8_t buck)
{
    // Pass out of bounds uvReaction into Pmic_pwrSetResourceCfgs() for buck
    const uint8_t numConfig = 1U;
    Pmic_PowerResourceCfg_t resourceCfg = {
        .validParams = PMIC_PWR_CFG_UV_REACT_VALID,
        .resource = buck,
        .uvReaction = PMIC_PWR_FAULT_REACT_MAX + 1U
    };
    int32_t status = Pmic_pwrSetResourceCfgs(&pmicHandle, numConfig, &resourceCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

static void powerTest_Pmic_pwrSetResourceCfgs_outOfBounds_buckOvThresh(uint8_t buck)
{
    // Pass out of bounds ovThresh into Pmic_pwrSetResourceCfgs() for buck
    const uint8_t numConfig = 1U;
    Pmic_PowerResourceCfg_t resourceCfg = {
        .validParams = PMIC_PWR_CFG_OV_THRESH_VALID,
        .resource = buck,
        .ovThresh = PMIC_PWR_BUCK_UV_OV_THR_MAX + 1U
    };
    int32_t status = Pmic_pwrSetResourceCfgs(&pmicHandle, numConfig, &resourceCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

static void powerTest_Pmic_pwrSetResourceCfgs_outOfBounds_buckOvReaction(uint8_t buck)
{
    // Pass out of bounds ovReaction into Pmic_pwrSetResourceCfgs() for buck
    const uint8_t numConfig = 1U;
    Pmic_PowerResourceCfg_t resourceCfg = {
        .validParams = PMIC_PWR_CFG_OV_REACT_VALID,
        .resource = buck,
        .ovReaction = PMIC_PWR_FAULT_REACT_MAX + 1U
    };
    int32_t status = Pmic_pwrSetResourceCfgs(&pmicHandle, numConfig, &resourceCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

static void powerTest_Pmic_pwrSetResourceCfgs_outOfBounds_buckRvReaction(uint8_t buck)
{
    // Pass out of bounds rvReaction into Pmic_pwrSetResourceCfgs() for buck
    const uint8_t numConfig = 1U;
    Pmic_PowerResourceCfg_t resourceCfg = {
        .validParams = PMIC_PWR_CFG_RV_REACT_VALID,
        .resource = buck,
        .rvReaction = PMIC_PWR_RV_REACT_MAX + 1U
    };
    int32_t status = Pmic_pwrSetResourceCfgs(&pmicHandle, numConfig, &resourceCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

static void powerTest_Pmic_pwrSetResourceCfgs_outOfBounds_buckScReaction(uint8_t buck)
{
    // Pass out of bounds scReaction into Pmic_pwrSetResourceCfgs() for buck
    const uint8_t numConfig = 1U;
    Pmic_PowerResourceCfg_t resourceCfg = {
        .validParams = PMIC_PWR_CFG_SC_REACT_VALID,
        .resource = buck,
        .scReaction = PMIC_PWR_FAULT_REACT_MAX + 1U
    };
    int32_t status = Pmic_pwrSetResourceCfgs(&pmicHandle, numConfig, &resourceCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrSetResourceCfgs_invalidParam_buck1_mode(void)
{
    powerTest_Pmic_pwrSetResourceCfgs_invalidParam_buckMode(PMIC_PWR_RSRC_BUCK1);
}

void test_neg_power_pwrSetResourceCfgs_outOfBounds_buck1_ilim(void)
{
    powerTest_Pmic_pwrSetResourceCfgs_outOfBounds_buckIlim(PMIC_PWR_RSRC_BUCK1);
}

void test_neg_power_pwrSetResourceCfgs_outOfBounds_buck1_voltage_mV(void)
{
    powerTest_Pmic_pwrSetResourceCfgs_outOfBounds_buckVoltage_mV(PMIC_PWR_RSRC_BUCK1);
}

void test_neg_power_pwrSetResourceCfgs_outOfBounds_buck1_deglitch(void)
{
    powerTest_Pmic_pwrSetResourceCfgs_outOfBounds_buckDeglitch(PMIC_PWR_RSRC_BUCK1);
}

void test_neg_power_pwrSetResourceCfgs_outOfBounds_buck1_uvThresh(void)
{
    powerTest_Pmic_pwrSetResourceCfgs_outOfBounds_buckUvThresh(PMIC_PWR_RSRC_BUCK1);
}

void test_neg_power_pwrSetResourceCfgs_outOfBounds_buck1_uvReaction(void)
{
    powerTest_Pmic_pwrSetResourceCfgs_outOfBounds_buckUvReaction(PMIC_PWR_RSRC_BUCK1);
}

void test_neg_power_pwrSetResourceCfgs_outOfBounds_buck1_ovThresh(void)
{
    powerTest_Pmic_pwrSetResourceCfgs_outOfBounds_buckOvThresh(PMIC_PWR_RSRC_BUCK1);
}

void test_neg_power_pwrSetResourceCfgs_outOfBounds_buck1_ovReaction(void)
{
    powerTest_Pmic_pwrSetResourceCfgs_outOfBounds_buckOvReaction(PMIC_PWR_RSRC_BUCK1);
}

void test_neg_power_pwrSetResourceCfgs_outOfBounds_buck1_rvReaction(void)
{
    powerTest_Pmic_pwrSetResourceCfgs_outOfBounds_buckRvReaction(PMIC_PWR_RSRC_BUCK1);
}

void test_neg_power_pwrSetResourceCfgs_outOfBounds_buck1_scReaction(void)
{
    powerTest_Pmic_pwrSetResourceCfgs_outOfBounds_buckScReaction(PMIC_PWR_RSRC_BUCK1);
}

void test_neg_power_pwrSetResourceCfgs_invalidParam_buck2_mode(void)
{
    powerTest_Pmic_pwrSetResourceCfgs_invalidParam_buckMode(PMIC_PWR_RSRC_BUCK2);
}

void test_neg_power_pwrSetResourceCfgs_outOfBounds_buck2_ilim(void)
{
    powerTest_Pmic_pwrSetResourceCfgs_outOfBounds_buckIlim(PMIC_PWR_RSRC_BUCK2);
}

void test_neg_power_pwrSetResourceCfgs_outOfBounds_buck2_voltage_mV(void)
{
    powerTest_Pmic_pwrSetResourceCfgs_outOfBounds_buckVoltage_mV(PMIC_PWR_RSRC_BUCK2);
}

void test_neg_power_pwrSetResourceCfgs_outOfBounds_buck2_deglitch(void)
{
    powerTest_Pmic_pwrSetResourceCfgs_outOfBounds_buckDeglitch(PMIC_PWR_RSRC_BUCK2);
}

void test_neg_power_pwrSetResourceCfgs_outOfBounds_buck2_uvThresh(void)
{
    powerTest_Pmic_pwrSetResourceCfgs_outOfBounds_buckUvThresh(PMIC_PWR_RSRC_BUCK2);
}

void test_neg_power_pwrSetResourceCfgs_outOfBounds_buck2_uvReaction(void)
{
    powerTest_Pmic_pwrSetResourceCfgs_outOfBounds_buckUvReaction(PMIC_PWR_RSRC_BUCK2);
}

void test_neg_power_pwrSetResourceCfgs_outOfBounds_buck2_ovThresh(void)
{
    powerTest_Pmic_pwrSetResourceCfgs_outOfBounds_buckOvThresh(PMIC_PWR_RSRC_BUCK2);
}

void test_neg_power_pwrSetResourceCfgs_outOfBounds_buck2_ovReaction(void)
{
    powerTest_Pmic_pwrSetResourceCfgs_outOfBounds_buckOvReaction(PMIC_PWR_RSRC_BUCK2);
}

void test_neg_power_pwrSetResourceCfgs_outOfBounds_buck2_rvReaction(void)
{
    powerTest_Pmic_pwrSetResourceCfgs_outOfBounds_buckRvReaction(PMIC_PWR_RSRC_BUCK2);
}

void test_neg_power_pwrSetResourceCfgs_outOfBounds_buck2_scReaction(void)
{
    powerTest_Pmic_pwrSetResourceCfgs_outOfBounds_buckScReaction(PMIC_PWR_RSRC_BUCK2);
}

void test_neg_power_pwrSetResourceCfgs_invalidParam_buck3_mode(void)
{
    powerTest_Pmic_pwrSetResourceCfgs_invalidParam_buckMode(PMIC_PWR_RSRC_BUCK3);
}

void test_neg_power_pwrSetResourceCfgs_outOfBounds_buck3_ilim(void)
{
    powerTest_Pmic_pwrSetResourceCfgs_outOfBounds_buckIlim(PMIC_PWR_RSRC_BUCK3);
}

void test_neg_power_pwrSetResourceCfgs_outOfBounds_buck3_voltage_mV(void)
{
    powerTest_Pmic_pwrSetResourceCfgs_outOfBounds_buckVoltage_mV(PMIC_PWR_RSRC_BUCK3);
}

void test_neg_power_pwrSetResourceCfgs_outOfBounds_buck3_deglitch(void)
{
    powerTest_Pmic_pwrSetResourceCfgs_outOfBounds_buckDeglitch(PMIC_PWR_RSRC_BUCK3);
}

void test_neg_power_pwrSetResourceCfgs_outOfBounds_buck3_uvThresh(void)
{
    powerTest_Pmic_pwrSetResourceCfgs_outOfBounds_buckUvThresh(PMIC_PWR_RSRC_BUCK3);
}

void test_neg_power_pwrSetResourceCfgs_outOfBounds_buck3_uvReaction(void)
{
    powerTest_Pmic_pwrSetResourceCfgs_outOfBounds_buckUvReaction(PMIC_PWR_RSRC_BUCK3);
}

void test_neg_power_pwrSetResourceCfgs_outOfBounds_buck3_ovThresh(void)
{
    powerTest_Pmic_pwrSetResourceCfgs_outOfBounds_buckOvThresh(PMIC_PWR_RSRC_BUCK3);
}

void test_neg_power_pwrSetResourceCfgs_outOfBounds_buck3_ovReaction(void)
{
    powerTest_Pmic_pwrSetResourceCfgs_outOfBounds_buckOvReaction(PMIC_PWR_RSRC_BUCK3);
}

void test_neg_power_pwrSetResourceCfgs_outOfBounds_buck3_rvReaction(void)
{
    powerTest_Pmic_pwrSetResourceCfgs_outOfBounds_buckRvReaction(PMIC_PWR_RSRC_BUCK3);
}

void test_neg_power_pwrSetResourceCfgs_outOfBounds_buck3_scReaction(void)
{
    powerTest_Pmic_pwrSetResourceCfgs_outOfBounds_buckScReaction(PMIC_PWR_RSRC_BUCK3);
}

void test_neg_power_pwrSetResourceCfgs_outOfBounds_ldoLs1Vmon1_mode(void)
{
    // Pass out of bounds mode into Pmic_pwrSetResourceCfgs() for LDO_LS1_VMON1
    const uint8_t numConfig = 1U;
    Pmic_PowerResourceCfg_t resourceCfg = {
        .validParams = PMIC_PWR_CFG_MODE_VALID,
        .resource = PMIC_PWR_RSRC_LDO_LS1_VMON1,
        .mode = PMIC_PWR_RSRC_MODE_MAX + 1U
    };
    int32_t status = Pmic_pwrSetResourceCfgs(&pmicHandle, numConfig, &resourceCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrSetResourceCfgs_ldoLs1Vmon1_ilim(void)
{
    const uint8_t numConfig = 1U;
    Pmic_PowerResourceCfg_t resourceCfg = {
        .validParams = PMIC_PWR_CFG_ILIM_VALID,
        .resource = PMIC_PWR_RSRC_LDO_LS1_VMON1
    };

    // Pass all ILIM values into Pmic_pwrSetResourceCfgs() for LDO_LS1_VMON1.
    // NOTE: For LDO_LS1_VMON1, ILIM is not a valid parameter
    for (uint8_t ilim = PMIC_PWR_ILIM_MIN; ilim <= PMIC_PWR_ILIM_MAX; ilim++)
    {
        resourceCfg.ilim = ilim;
        int32_t status = Pmic_pwrSetResourceCfgs(&pmicHandle, numConfig, &resourceCfg);
        PLATFORM_ASSERT(status == PMIC_ST_ERR_NOT_SUPPORTED);
    }
}

static void powerTest_Pmic_pwrSetResourceCfgs_outOfBounds_ldoLsVmon_voltage_mV(uint8_t ldoLsVmon)
{
    const uint8_t numConfig = 1U;
    Pmic_PowerResourceCfg_t resourceCfg = {
        .validParams = PMIC_PWR_CFG_VOLTAGE_VALID,
        .resource = ldoLsVmon
    };

    // Pass out of bounds voltage_mV into Pmic_pwrSetResourceCfgs() for LDO/LS/VMON
    resourceCfg.voltage_mV = 575U; // Under range
    int32_t status = Pmic_pwrSetResourceCfgs(&pmicHandle, numConfig, &resourceCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
    resourceCfg.voltage_mV = 3425U; // Over range
    status = Pmic_pwrSetResourceCfgs(&pmicHandle, numConfig, &resourceCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);

    // Additionally, pass a value that is not a multiple of 25
    resourceCfg.voltage_mV = 1001U;
    status = Pmic_pwrSetResourceCfgs(&pmicHandle, numConfig, &resourceCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrSetResourceCfgs_outOfBounds_ldoLs1Vmon1_voltage_mV(void)
{
    powerTest_Pmic_pwrSetResourceCfgs_outOfBounds_ldoLsVmon_voltage_mV(PMIC_PWR_RSRC_LDO_LS1_VMON1);
}

void test_neg_power_pwrSetResourceCfgs_outOfBounds_ldoLs1Vmon1_deglitch(void)
{
    // Pass out of bounds deglitch into Pmic_pwrSetResourceCfgs() for LDO_LS1_VMON1
    const uint8_t numConfig = 1U;
    Pmic_PowerResourceCfg_t resourceCfg = {
        .validParams = PMIC_PWR_CFG_DEGLITCH_VALID,
        .resource = PMIC_PWR_RSRC_LDO_LS1_VMON1,
        .deglitch = PMIC_PWR_DEGLITCH_MAX + 1U
    };
    int32_t status = Pmic_pwrSetResourceCfgs(&pmicHandle, numConfig, &resourceCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrSetResourceCfgs_outOfBounds_ldoLs1Vmon1_uvThresh(void)
{
    // Pass out of bounds uvThresh into Pmic_pwrSetResourceCfgs() for LDO_LS1_VMON1
    const uint8_t numConfig = 1U;
    Pmic_PowerResourceCfg_t resourceCfg = {
        .validParams = PMIC_PWR_CFG_UV_THRESH_VALID,
        .resource = PMIC_PWR_RSRC_LDO_LS1_VMON1,
        .uvThresh = PMIC_PWR_LS_UV_OV_THR_MAX + 1U
    };
    int32_t status = Pmic_pwrSetResourceCfgs(&pmicHandle, numConfig, &resourceCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrSetResourceCfgs_outOfBounds_ldoLs1Vmon1_uvReaction(void)
{
    // Pass out of bounds uvReaction into Pmic_pwrSetResourceCfgs() for LDO_LS1_VMON1
    const uint8_t numConfig = 1U;
    Pmic_PowerResourceCfg_t resourceCfg = {
        .validParams = PMIC_PWR_CFG_UV_REACT_VALID,
        .resource = PMIC_PWR_RSRC_LDO_LS1_VMON1,
        .uvReaction = PMIC_PWR_FAULT_REACT_MAX + 1U
    };
    int32_t status = Pmic_pwrSetResourceCfgs(&pmicHandle, numConfig, &resourceCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrSetResourceCfgs_outOfBounds_ldoLs1Vmon1_ovThresh(void)
{
    // Pass out of bounds ovThresh into Pmic_pwrSetResourceCfgs() for LDO_LS1_VMON1
    const uint8_t numConfig = 1U;
    Pmic_PowerResourceCfg_t resourceCfg = {
        .validParams = PMIC_PWR_CFG_OV_THRESH_VALID,
        .resource = PMIC_PWR_RSRC_LDO_LS1_VMON1,
        .ovThresh = PMIC_PWR_LS_UV_OV_THR_MAX + 1U
    };
    int32_t status = Pmic_pwrSetResourceCfgs(&pmicHandle, numConfig, &resourceCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrSetResourceCfgs_outOfBounds_ldoLs1Vmon1_ovReaction(void)
{
    // Pass out of bounds ovReaction into Pmic_pwrSetResourceCfgs() for LDO_LS1_VMON1
    const uint8_t numConfig = 1U;
    Pmic_PowerResourceCfg_t resourceCfg = {
        .validParams = PMIC_PWR_CFG_OV_REACT_VALID,
        .resource = PMIC_PWR_RSRC_LDO_LS1_VMON1,
        .ovReaction = PMIC_PWR_FAULT_REACT_MAX + 1U
    };
    int32_t status = Pmic_pwrSetResourceCfgs(&pmicHandle, numConfig, &resourceCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrSetResourceCfgs_outOfBounds_ldoLs1Vmon1_rvReaction(void)
{
    // Pass out of bounds rvReaction into Pmic_pwrSetResourceCfgs() for LDO_LS1_VMON1
    const uint8_t numConfig = 1U;
    Pmic_PowerResourceCfg_t resourceCfg = {
        .validParams = PMIC_PWR_CFG_RV_REACT_VALID,
        .resource = PMIC_PWR_RSRC_LDO_LS1_VMON1,
        .rvReaction = PMIC_PWR_RV_REACT_MAX + 1U
    };
    int32_t status = Pmic_pwrSetResourceCfgs(&pmicHandle, numConfig, &resourceCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrSetResourceCfgs_outOfBounds_ldoLs1Vmon1_scReaction(void)
{
    // Pass out of bounds scReaction into Pmic_pwrSetResourceCfgs() for LDO_LS1_VMON1
    const uint8_t numConfig = 1U;
    Pmic_PowerResourceCfg_t resourceCfg = {
        .validParams = PMIC_PWR_CFG_SC_REACT_VALID,
        .resource = PMIC_PWR_RSRC_LDO_LS1_VMON1,
        .scReaction = PMIC_PWR_FAULT_REACT_MAX + 1U
    };
    int32_t status = Pmic_pwrSetResourceCfgs(&pmicHandle, numConfig, &resourceCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrSetResourceCfgs_invalidParam_ls2Vmon2_mode(void)
{
    uint8_t numConfig = 1U;
    Pmic_PowerResourceCfg_t resourceCfg = {
        .validParams = PMIC_PWR_CFG_MODE_VALID,
        .resource = PMIC_PWR_RSRC_LS2_VMON2
    };

    // Pass invalid mode into Pmic_pwrSetResourceCfgs() for LS2_VMON2
    for (uint8_t mode = PMIC_PWR_RSRC_MODE_MIN; mode <= PMIC_PWR_RSRC_MODE_MAX; mode++)
    {
        if ((mode != PMIC_PWR_RSRC_MODE_LSW) && (mode != PMIC_PWR_RSRC_MODE_VMON))
        {
            resourceCfg.mode = mode;
            int32_t status = Pmic_pwrSetResourceCfgs(&pmicHandle, numConfig, &resourceCfg);
            PLATFORM_ASSERT(status == PMIC_ST_ERR_NOT_SUPPORTED);
        }
    }
}

void test_neg_power_pwrSetResourceCfgs_ls2Vmon2_ilim(void)
{
    const uint8_t numConfig = 1U;
    Pmic_PowerResourceCfg_t resourceCfg = {
        .validParams = PMIC_PWR_CFG_ILIM_VALID,
        .resource = PMIC_PWR_RSRC_LS2_VMON2
    };

    // Pass all ILIM values into Pmic_pwrSetResourceCfgs() for LS2_VMON2.
    // NOTE: For LS2_VMON2, ILIM is not a valid parameter
    for (uint8_t ilim = PMIC_PWR_ILIM_MIN; ilim <= PMIC_PWR_ILIM_MAX; ilim++)
    {
        resourceCfg.ilim = ilim;
        int32_t status = Pmic_pwrSetResourceCfgs(&pmicHandle, numConfig, &resourceCfg);
        PLATFORM_ASSERT(status == PMIC_ST_ERR_NOT_SUPPORTED);
    }
}

void test_neg_power_pwrSetResourceCfgs_outOfBounds_ls2Vmon2_voltage_mV(void)
{
    powerTest_Pmic_pwrSetResourceCfgs_outOfBounds_ldoLsVmon_voltage_mV(PMIC_PWR_RSRC_LS2_VMON2);
}

void test_neg_power_pwrSetResourceCfgs_outOfBounds_ls2Vmon2_deglitch(void)
{
    // Pass out of bounds deglitch into Pmic_pwrSetResourceCfgs() for LS2_VMON2
    const uint8_t numConfig = 1U;
    Pmic_PowerResourceCfg_t resourceCfg = {
        .validParams = PMIC_PWR_CFG_DEGLITCH_VALID,
        .resource = PMIC_PWR_RSRC_LS2_VMON2,
        .deglitch = PMIC_PWR_DEGLITCH_MAX + 1U
    };
    int32_t status = Pmic_pwrSetResourceCfgs(&pmicHandle, numConfig, &resourceCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrSetResourceCfgs_outOfBounds_ls2Vmon2_uvThresh(void)
{
    // Pass out of bounds uvThresh into Pmic_pwrSetResourceCfgs() for LS2_VMON2
    const uint8_t numConfig = 1U;
    Pmic_PowerResourceCfg_t resourceCfg = {
        .validParams = PMIC_PWR_CFG_UV_THRESH_VALID,
        .resource = PMIC_PWR_RSRC_LS2_VMON2,
        .uvThresh = PMIC_PWR_LS_UV_OV_THR_MAX + 1U
    };
    int32_t status = Pmic_pwrSetResourceCfgs(&pmicHandle, numConfig, &resourceCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrSetResourceCfgs_outOfBounds_ls2Vmon2_uvReaction(void)
{
    // Pass out of bounds uvReaction into Pmic_pwrSetResourceCfgs() for LS2_VMON2
    const uint8_t numConfig = 1U;
    Pmic_PowerResourceCfg_t resourceCfg = {
        .validParams = PMIC_PWR_CFG_UV_REACT_VALID,
        .resource = PMIC_PWR_RSRC_LS2_VMON2,
        .uvReaction = PMIC_PWR_FAULT_REACT_MAX + 1U
    };
    int32_t status = Pmic_pwrSetResourceCfgs(&pmicHandle, numConfig, &resourceCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrSetResourceCfgs_outOfBounds_ls2Vmon2_ovThresh(void)
{
    // Pass out of bounds ovThresh into Pmic_pwrSetResourceCfgs() for LS2_VMON2
    const uint8_t numConfig = 1U;
    Pmic_PowerResourceCfg_t resourceCfg = {
        .validParams = PMIC_PWR_CFG_OV_THRESH_VALID,
        .resource = PMIC_PWR_RSRC_LS2_VMON2,
        .ovThresh = PMIC_PWR_LS_UV_OV_THR_MAX + 1U
    };
    int32_t status = Pmic_pwrSetResourceCfgs(&pmicHandle, numConfig, &resourceCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrSetResourceCfgs_outOfBounds_ls2Vmon2_ovReaction(void)
{
    // Pass out of bounds ovReaction into Pmic_pwrSetResourceCfgs() for LS2_VMON2
    const uint8_t numConfig = 1U;
    Pmic_PowerResourceCfg_t resourceCfg = {
        .validParams = PMIC_PWR_CFG_OV_REACT_VALID,
        .resource = PMIC_PWR_RSRC_LS2_VMON2,
        .ovReaction = PMIC_PWR_FAULT_REACT_MAX + 1U
    };
    int32_t status = Pmic_pwrSetResourceCfgs(&pmicHandle, numConfig, &resourceCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrSetResourceCfgs_outOfBounds_ls2Vmon2_rvReaction(void)
{
    // Pass out of bounds rvReaction into Pmic_pwrSetResourceCfgs() for LS2_VMON2
    const uint8_t numConfig = 1U;
    Pmic_PowerResourceCfg_t resourceCfg = {
        .validParams = PMIC_PWR_CFG_RV_REACT_VALID,
        .resource = PMIC_PWR_RSRC_LS2_VMON2,
        .rvReaction = PMIC_PWR_RV_REACT_MAX + 1U
    };
    int32_t status = Pmic_pwrSetResourceCfgs(&pmicHandle, numConfig, &resourceCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrSetResourceCfgs_outOfBounds_ls2Vmon2_scReaction(void)
{
    // Pass out of bounds scReaction into Pmic_pwrSetResourceCfgs() for LS2_VMON2
    const uint8_t numConfig = 1U;
    Pmic_PowerResourceCfg_t resourceCfg = {
        .validParams = PMIC_PWR_CFG_SC_REACT_VALID,
        .resource = PMIC_PWR_RSRC_LS2_VMON2,
        .scReaction = PMIC_PWR_FAULT_REACT_MAX + 1U
    };
    int32_t status = Pmic_pwrSetResourceCfgs(&pmicHandle, numConfig, &resourceCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrSetResourceCfgs_invalidParam_vccaVmon_mode(void)
{
    const uint8_t numConfig = 1U;
    Pmic_PowerResourceCfg_t resourceCfg = {
        .validParams = PMIC_PWR_CFG_MODE_VALID,
        .resource = PMIC_PWR_RSRC_VCCA_VMON
    };

    // Pass invalid mode into Pmic_pwrSetResourceCfgs() for VCCA_VMON
    for (uint8_t mode = PMIC_PWR_RSRC_MODE_MIN; mode <= PMIC_PWR_RSRC_MODE_MAX; mode++)
    {
        if (mode != PMIC_PWR_RSRC_MODE_VMON)
        {
            resourceCfg.mode = mode;
            int32_t status = Pmic_pwrSetResourceCfgs(&pmicHandle, numConfig, &resourceCfg);
            PLATFORM_ASSERT(status == PMIC_ST_ERR_NOT_SUPPORTED);
        }
    }
}

void test_neg_power_pwrSetResourceCfgs_vccaVmon_ilim(void)
{
    const uint8_t numConfig = 1U;
    Pmic_PowerResourceCfg_t resourceCfg = {
        .validParams = PMIC_PWR_CFG_ILIM_VALID,
        .resource = PMIC_PWR_RSRC_VCCA_VMON
    };

    // Pass all ILIM values into Pmic_pwrSetResourceCfgs() for VCCA_VMON.
    // NOTE: For VCCA_VMON, ILIM is not a valid parameter
    for (uint8_t ilim = PMIC_PWR_ILIM_MIN; ilim <= PMIC_PWR_ILIM_MAX; ilim++)
    {
        resourceCfg.ilim = ilim;
        int32_t status = Pmic_pwrSetResourceCfgs(&pmicHandle, numConfig, &resourceCfg);
        PLATFORM_ASSERT(status == PMIC_ST_ERR_NOT_SUPPORTED);
    }
}

void test_neg_power_pwrSetResourceCfgs_outOfBounds_vccaVmon_voltage_mV(void)
{
    powerTest_Pmic_pwrSetResourceCfgs_outOfBounds_ldoLsVmon_voltage_mV(PMIC_PWR_RSRC_VCCA_VMON);
}

void test_neg_power_pwrSetResourceCfgs_outOfBounds_vccaVmon_deglitch(void)
{
    // Pass out of bounds deglitch into Pmic_pwrSetResourceCfgs() for VCCA_VMON
    const uint8_t numConfig = 1U;
    Pmic_PowerResourceCfg_t resourceCfg = {
        .validParams = PMIC_PWR_CFG_DEGLITCH_VALID,
        .resource = PMIC_PWR_RSRC_VCCA_VMON,
        .deglitch = PMIC_PWR_DEGLITCH_MAX + 1U
    };
    int32_t status = Pmic_pwrSetResourceCfgs(&pmicHandle, numConfig, &resourceCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrSetResourceCfgs_outOfBounds_vccaVmon_uvThresh(void)
{
    // Pass out of bounds uvThresh into Pmic_pwrSetResourceCfgs() for VCCA_VMON
    const uint8_t numConfig = 1U;
    Pmic_PowerResourceCfg_t resourceCfg = {
        .validParams = PMIC_PWR_CFG_UV_THRESH_VALID,
        .resource = PMIC_PWR_RSRC_VCCA_VMON,
        .uvThresh = PMIC_PWR_VCCA_UV_OV_THR_MAX + 1U
    };
    int32_t status = Pmic_pwrSetResourceCfgs(&pmicHandle, numConfig, &resourceCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrSetResourceCfgs_outOfBounds_vccaVmon_uvReaction(void)
{
    // Pass out of bounds uvReaction into Pmic_pwrSetResourceCfgs() for VCCA_VMON
    const uint8_t numConfig = 1U;
    Pmic_PowerResourceCfg_t resourceCfg = {
        .validParams = PMIC_PWR_CFG_UV_REACT_VALID,
        .resource = PMIC_PWR_RSRC_VCCA_VMON,
        .uvReaction = PMIC_PWR_FAULT_REACT_MAX + 1U
    };
    int32_t status = Pmic_pwrSetResourceCfgs(&pmicHandle, numConfig, &resourceCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrSetResourceCfgs_outOfBounds_vccaVmon_ovThresh(void)
{
    // Pass out of bounds ovThresh into Pmic_pwrSetResourceCfgs() for VCCA_VMON
    const uint8_t numConfig = 1U;
    Pmic_PowerResourceCfg_t resourceCfg = {
        .validParams = PMIC_PWR_CFG_OV_THRESH_VALID,
        .resource = PMIC_PWR_RSRC_VCCA_VMON,
        .ovThresh = PMIC_PWR_VCCA_UV_OV_THR_MAX + 1U
    };
    int32_t status = Pmic_pwrSetResourceCfgs(&pmicHandle, numConfig, &resourceCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrSetResourceCfgs_outOfBounds_vccaVmon_ovReaction(void)
{
    // Pass out of bounds ovReaction into Pmic_pwrSetResourceCfgs() for VCCA_VMON
    const uint8_t numConfig = 1U;
    Pmic_PowerResourceCfg_t resourceCfg = {
        .validParams = PMIC_PWR_CFG_OV_REACT_VALID,
        .resource = PMIC_PWR_RSRC_VCCA_VMON,
        .ovReaction = PMIC_PWR_FAULT_REACT_MAX + 1U
    };
    int32_t status = Pmic_pwrSetResourceCfgs(&pmicHandle, numConfig, &resourceCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrSetResourceCfgs_vccaVmon_rvReaction(void)
{
    const uint8_t numConfig = 1U;
    Pmic_PowerResourceCfg_t resourceCfg = {
        .validParams = PMIC_PWR_CFG_RV_REACT_VALID,
        .resource = PMIC_PWR_RSRC_VCCA_VMON
    };

    // Pass all rvReaction values into Pmic_pwrSetResourceCfgs() for VCCA_VMON.
    // NOTE: For VCCA_VMON, rvReaction is not a valid parameter
    for (uint8_t rvReaction = PMIC_PWR_RV_REACT_MIN; rvReaction <= PMIC_PWR_RV_REACT_MAX; rvReaction++)
    {
        resourceCfg.rvReaction = rvReaction;
        int32_t status = Pmic_pwrSetResourceCfgs(&pmicHandle, numConfig, &resourceCfg);
        PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
    }
}

void test_neg_power_pwrSetResourceCfgs_vccaVmon_scReaction(void)
{
    const uint8_t numConfig = 1U;
    Pmic_PowerResourceCfg_t resourceCfg = {
        .validParams = PMIC_PWR_CFG_SC_REACT_VALID,
        .resource = PMIC_PWR_RSRC_VCCA_VMON
    };

    // Pass all scReaction values into Pmic_pwrSetResourceCfgs() for VCCA_VMON.
    // NOTE: For VCCA_VMON, scReaction is not a valid parameter
    for (uint8_t scReaction = PMIC_PWR_FAULT_REACT_MIN; scReaction <= PMIC_PWR_FAULT_REACT_MAX; scReaction++)
    {
        resourceCfg.scReaction = scReaction;
        int32_t status = Pmic_pwrSetResourceCfgs(&pmicHandle, numConfig, &resourceCfg);
        PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
    }
}

void test_neg_power_pwrSetResourceCfgs_gpo_mode(void)
{
    const uint8_t numConfig = 1U;
    Pmic_PowerResourceCfg_t resourceCfg = {
        .validParams = PMIC_PWR_CFG_MODE_VALID,
        .resource = PMIC_PWR_RSRC_GPO
    };

    // Pass all mode values into Pmic_pwrSetResourceCfgs() for GPO.
    // NOTE: For GPO, mode is not a valid parameter
    for (uint8_t mode = PMIC_PWR_RSRC_MODE_MIN; mode <= PMIC_PWR_RSRC_MODE_MAX; mode++)
    {
        resourceCfg.mode = mode;
        int32_t status = Pmic_pwrSetResourceCfgs(&pmicHandle, numConfig, &resourceCfg);
        PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
    }
}

void test_neg_power_pwrSetResourceCfgs_gpo_ilim(void)
{
    const uint8_t numConfig = 1U;
    Pmic_PowerResourceCfg_t resourceCfg = {
        .validParams = PMIC_PWR_CFG_ILIM_VALID,
        .resource = PMIC_PWR_RSRC_GPO
    };

    // Pass all ILIM values into Pmic_pwrSetResourceCfgs() for GPO.
    // NOTE: For GPO, ILIM is not a valid parameter
    for (uint8_t ilim = PMIC_PWR_ILIM_MIN; ilim <= PMIC_PWR_ILIM_MAX; ilim++)
    {
        resourceCfg.ilim = ilim;
        int32_t status = Pmic_pwrSetResourceCfgs(&pmicHandle, numConfig, &resourceCfg);
        PLATFORM_ASSERT(status == PMIC_ST_ERR_NOT_SUPPORTED);
    }
}

void test_neg_power_pwrSetResourceCfgs_gpo_voltage_mV(void)
{
    // Pass voltage_mV into Pmic_pwrSetResourceCfgs() for GPO.
    // NOTE: For GPO, voltage_mV is not a valid parameter
    const uint8_t numConfig = 1U;
    Pmic_PowerResourceCfg_t resourceCfg = {
        .validParams = PMIC_PWR_CFG_VOLTAGE_VALID,
        .resource = PMIC_PWR_RSRC_GPO,
        .voltage_mV = 1000U
    };
    int32_t status = Pmic_pwrSetResourceCfgs(&pmicHandle, numConfig, &resourceCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrSetResourceCfgs_gpo_deglitch(void)
{
    const uint8_t numConfig = 1U;
    Pmic_PowerResourceCfg_t resourceCfg = {
        .validParams = PMIC_PWR_CFG_DEGLITCH_VALID,
        .resource = PMIC_PWR_RSRC_GPO
    };

    // Pass all deglitch values into Pmic_pwrSetResourceCfgs() for GPO.
    // NOTE: For GPO, deglitch is not a valid parameter
    for (uint8_t deglitch = PMIC_PWR_DEGLITCH_MIN; deglitch <= PMIC_PWR_DEGLITCH_MAX; deglitch++)
    {
        resourceCfg.deglitch = deglitch;
        int32_t status = Pmic_pwrSetResourceCfgs(&pmicHandle, numConfig, &resourceCfg);
        PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
    }
}

void test_neg_power_pwrSetResourceCfgs_gpo_uvThresh(void)
{
    // Pass uvThresh into Pmic_pwrSetResourceCfgs() for GPO.
    // NOTE: For GPO, uvThresh is not a valid parameter
    const uint8_t numConfig = 1U;
    Pmic_PowerResourceCfg_t resourceCfg = {
        .validParams = PMIC_PWR_CFG_UV_THRESH_VALID,
        .resource = PMIC_PWR_RSRC_GPO,
        .uvThresh = 0U
    };
    int32_t status = Pmic_pwrSetResourceCfgs(&pmicHandle, numConfig, &resourceCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrSetResourceCfgs_gpo_uvReaction(void)
{
    // Pass uvReaction into Pmic_pwrSetResourceCfgs() for GPO.
    // NOTE: For GPO, uvReaction is not a valid parameter
    const uint8_t numConfig = 1U;
    Pmic_PowerResourceCfg_t resourceCfg = {
        .validParams = PMIC_PWR_CFG_UV_REACT_VALID,
        .resource = PMIC_PWR_RSRC_GPO,
        .uvReaction = 0U
    };
    int32_t status = Pmic_pwrSetResourceCfgs(&pmicHandle, numConfig, &resourceCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrSetResourceCfgs_gpo_ovThresh(void)
{
    // Pass ovThresh into Pmic_pwrSetResourceCfgs() for GPO.
    // NOTE: For GPO, ovThresh is not a valid parameter
    const uint8_t numConfig = 1U;
    Pmic_PowerResourceCfg_t resourceCfg = {
        .validParams = PMIC_PWR_CFG_OV_THRESH_VALID,
        .resource = PMIC_PWR_RSRC_GPO,
        .ovThresh = 0U
    };
    int32_t status = Pmic_pwrSetResourceCfgs(&pmicHandle, numConfig, &resourceCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrSetResourceCfgs_gpo_ovReaction(void)
{
    // Pass ovReaction into Pmic_pwrSetResourceCfgs() for GPO.
    // NOTE: For GPO, ovReaction is not a valid parameter
    const uint8_t numConfig = 1U;
    Pmic_PowerResourceCfg_t resourceCfg = {
        .validParams = PMIC_PWR_CFG_OV_REACT_VALID,
        .resource = PMIC_PWR_RSRC_GPO,
        .ovReaction = 0U
    };
    int32_t status = Pmic_pwrSetResourceCfgs(&pmicHandle, numConfig, &resourceCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrSetResourceCfgs_gpo_rvReaction(void)
{
    // Pass rvReaction into Pmic_pwrSetResourceCfgs() for GPO.
    // NOTE: For GPO, rvReaction is not a valid parameter
    const uint8_t numConfig = 1U;
    Pmic_PowerResourceCfg_t resourceCfg = {
        .validParams = PMIC_PWR_CFG_RV_REACT_VALID,
        .resource = PMIC_PWR_RSRC_GPO,
        .rvReaction = 0U
    };
    int32_t status = Pmic_pwrSetResourceCfgs(&pmicHandle, numConfig, &resourceCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrSetResourceCfgs_gpo_scReaction(void)
{
    // Pass scReaction into Pmic_pwrSetResourceCfgs() for GPO.
    // NOTE: For GPO, scReaction is not a valid parameter
    const uint8_t numConfig = 1U;
    Pmic_PowerResourceCfg_t resourceCfg = {
        .validParams = PMIC_PWR_CFG_SC_REACT_VALID,
        .resource = PMIC_PWR_RSRC_GPO,
        .scReaction = 0U
    };
    int32_t status = Pmic_pwrSetResourceCfgs(&pmicHandle, numConfig, &resourceCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrGetResourceCfgs_nullParam_handle(void)
{
    // Pass NULL handle into Pmic_pwrGetResourceCfgs()
    const uint8_t numConfig = 1U;
    Pmic_PowerResourceCfg_t resourceCfg = {
        .validParams = PMIC_PWR_CFG_ENABLE_VALID,
        .resource = PMIC_PWR_RSRC_BUCK1
    };
    int32_t status = Pmic_pwrGetResourceCfgs(NULL, numConfig, &resourceCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_power_pwrGetResourceCfgs_nullParam_config(void)
{
    // Pass NULL config into Pmic_pwrGetResourceCfgs()
    const uint8_t numConfig = 1U;
    int32_t status = Pmic_pwrGetResourceCfgs(&pmicHandle, numConfig, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_power_pwrGetResourceCfgs_outOfBounds_resource(void)
{
    // Pass out of bounds resource into Pmic_pwrGetResourceCfgs()
    const uint8_t numConfig = 1U;
    Pmic_PowerResourceCfg_t resourceCfg = {
        .validParams = PMIC_PWR_CFG_ENABLE_VALID,
        .resource = PMIC_PWR_RSRC_MAX + 1U
    };
    int32_t status = Pmic_pwrGetResourceCfgs(&pmicHandle, numConfig, &resourceCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrSetSequenceCfg_nullParam_handle(void)
{
    // Pass NULL handle into Pmic_pwrSetSequenceCfg()
    Pmic_PowerSequenceCfg_t sequenceCfg = {
        .validParams = PMIC_PWR_SEQ_STARTUP_VALID,
        .resource = PMIC_PWR_RSRC_BUCK1,
        .startupDelay = PMIC_PWR_SEQ_DLY_MAX
    };
    int32_t status = Pmic_pwrSetSequenceCfg(NULL, &sequenceCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_power_pwrSetSequenceCfg_nullParam_config(void)
{
    // Pass NULL config into Pmic_pwrSetSequenceCfg()
    int32_t status = Pmic_pwrSetSequenceCfg(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_power_pwrSetSequenceCfg_outOfBounds_resource(void)
{
    // Pass out of bounds resource into Pmic_pwrSetSequenceCfg()
    Pmic_PowerSequenceCfg_t sequenceCfg = {
        .validParams = PMIC_PWR_SEQ_STARTUP_VALID,
        .resource = PMIC_PWR_RSRC_MAX + 1U,
        .startupDelay = PMIC_PWR_SEQ_DLY_MAX
    };
    int32_t status = Pmic_pwrSetSequenceCfg(&pmicHandle, &sequenceCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

static void powerTest_Pmic_pwrSetSequenceCfg_outOfBounds_startupDelay(uint8_t resource)
{
    // Pass out of bounds startupDelay into Pmic_pwrSetSequenceCfg()
    Pmic_PowerSequenceCfg_t sequenceCfg = {
        .validParams = PMIC_PWR_SEQ_STARTUP_VALID,
        .resource = resource,
        .startupDelay = PMIC_PWR_SEQ_DLY_MAX + 1U
    };
    int32_t status = Pmic_pwrSetSequenceCfg(&pmicHandle, &sequenceCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

static void powerTest_Pmic_pwrSetSequenceCfg_outOfBounds_shutdownDelay(uint8_t resource)
{
    // Pass out of bounds shutdownDelay into Pmic_pwrSetSequenceCfg()
    Pmic_PowerSequenceCfg_t sequenceCfg = {
        .validParams = PMIC_PWR_SEQ_SHUTDOWN_VALID,
        .resource = resource,
        .shutdownDelay = PMIC_PWR_SEQ_DLY_MAX + 1U
    };
    int32_t status = Pmic_pwrSetSequenceCfg(&pmicHandle, &sequenceCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrSetSequenceCfg_outOfBounds_buck1_startupDelay(void)
{
    powerTest_Pmic_pwrSetSequenceCfg_outOfBounds_startupDelay(PMIC_PWR_RSRC_BUCK1);
}

void test_neg_power_pwrSetSequenceCfg_outOfBounds_buck1_shutdownDelay(void)
{
    powerTest_Pmic_pwrSetSequenceCfg_outOfBounds_shutdownDelay(PMIC_PWR_RSRC_BUCK1);
}

void test_neg_power_pwrSetSequenceCfg_outOfBounds_buck2_startupDelay(void)
{
    powerTest_Pmic_pwrSetSequenceCfg_outOfBounds_startupDelay(PMIC_PWR_RSRC_BUCK2);
}

void test_neg_power_pwrSetSequenceCfg_outOfBounds_buck2_shutdownDelay(void)
{
    powerTest_Pmic_pwrSetSequenceCfg_outOfBounds_shutdownDelay(PMIC_PWR_RSRC_BUCK2);
}

void test_neg_power_pwrSetSequenceCfg_outOfBounds_buck3_startupDelay(void)
{
    powerTest_Pmic_pwrSetSequenceCfg_outOfBounds_startupDelay(PMIC_PWR_RSRC_BUCK3);
}

void test_neg_power_pwrSetSequenceCfg_outOfBounds_buck3_shutdownDelay(void)
{
    powerTest_Pmic_pwrSetSequenceCfg_outOfBounds_shutdownDelay(PMIC_PWR_RSRC_BUCK3);
}

void test_neg_power_pwrSetSequenceCfg_outOfBounds_ldoLs1Vmon1_startupDelay(void)
{
    powerTest_Pmic_pwrSetSequenceCfg_outOfBounds_startupDelay(PMIC_PWR_RSRC_LDO_LS1_VMON1);
}

void test_neg_power_pwrSetSequenceCfg_outOfBounds_ldoLs1Vmon1_shutdownDelay(void)
{
    powerTest_Pmic_pwrSetSequenceCfg_outOfBounds_shutdownDelay(PMIC_PWR_RSRC_LDO_LS1_VMON1);
}

void test_neg_power_pwrSetSequenceCfg_outOfBounds_ls2Vmon2_startupDelay(void)
{
    powerTest_Pmic_pwrSetSequenceCfg_outOfBounds_startupDelay(PMIC_PWR_RSRC_LS2_VMON2);
}

void test_neg_power_pwrSetSequenceCfg_outOfBounds_ls2Vmon2_shutdownDelay(void)
{
    powerTest_Pmic_pwrSetSequenceCfg_outOfBounds_shutdownDelay(PMIC_PWR_RSRC_LS2_VMON2);
}

void test_neg_power_pwrSetSequenceCfg_outOfBounds_gpo_startupDelay(void)
{
    powerTest_Pmic_pwrSetSequenceCfg_outOfBounds_startupDelay(PMIC_PWR_RSRC_GPO);
}

void test_neg_power_pwrSetSequenceCfg_outOfBounds_gpo_shutdownDelay(void)
{
    powerTest_Pmic_pwrSetSequenceCfg_outOfBounds_shutdownDelay(PMIC_PWR_RSRC_GPO);
}

void test_neg_power_pwrSetSequenceCfg_vccaVmon_startupDelay(void)
{
    // Pass startupDelay into Pmic_pwrSetSequenceCfg() for VCCA_VMON.
    // NOTE: For VCCA_VMON, startupDelay is invalid
    Pmic_PowerSequenceCfg_t sequenceCfg = {
        .validParams = PMIC_PWR_SEQ_STARTUP_VALID,
        .resource = PMIC_PWR_RSRC_VCCA_VMON,
        .startupDelay = PMIC_PWR_SEQ_DLY_7P5MS
    };
    int32_t status = Pmic_pwrSetSequenceCfg(&pmicHandle, &sequenceCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrSetSequenceCfg_vccaVmon_shutdownDelay(void)
{
    // Pass shutdownDelay into Pmic_pwrSetSequenceCfg() for VCCA_VMON.
    // NOTE: For VCCA_VMON, shutdownDelay is invalid
    Pmic_PowerSequenceCfg_t sequenceCfg = {
        .validParams = PMIC_PWR_SEQ_SHUTDOWN_VALID,
        .resource = PMIC_PWR_RSRC_VCCA_VMON,
        .shutdownDelay = PMIC_PWR_SEQ_DLY_7P5MS
    };
    int32_t status = Pmic_pwrSetSequenceCfg(&pmicHandle, &sequenceCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrGetSequenceCfg_nullParam_handle(void)
{
    // Pass NULL handle into Pmic_pwrGetSequenceCfg()
    Pmic_PowerSequenceCfg_t sequenceCfg = {
        .validParams = PMIC_PWR_SEQ_STARTUP_VALID,
        .resource = PMIC_PWR_RSRC_BUCK1
    };
    int32_t status = Pmic_pwrGetSequenceCfg(NULL, &sequenceCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_power_pwrGetSequenceCfg_nullParam_config(void)
{
    // Pass NULL config into Pmic_pwrGetSequenceCfg()
    int32_t status = Pmic_pwrGetSequenceCfg(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_power_pwrSetSequenceCfgs_nullParam_handle(void)
{
    // Pass NULL handle into Pmic_pwrSetSequenceCfgs()
    const uint8_t numConfig = 1U;
    Pmic_PowerSequenceCfg_t sequenceCfg = {
        .validParams = PMIC_PWR_SEQ_STARTUP_VALID,
        .resource = PMIC_PWR_RSRC_BUCK1,
        .startupDelay = PMIC_PWR_SEQ_DLY_MAX
    };
    int32_t status = Pmic_pwrSetSequenceCfgs(NULL, numConfig, &sequenceCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_power_pwrSetSequenceCfgs_nullParam_config(void)
{
    // Pass NULL config into Pmic_pwrSetSequenceCfgs()
    const uint8_t numConfig = 1U;
    int32_t status = Pmic_pwrSetSequenceCfgs(&pmicHandle, numConfig, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_power_pwrSetSequenceCfgs_outOfBounds_resource(void)
{
    // Pass out of bounds resource into Pmic_pwrSetSequenceCfgs()
    const uint8_t numConfig = 1U;
    Pmic_PowerSequenceCfg_t sequenceCfg = {
        .validParams = PMIC_PWR_SEQ_STARTUP_VALID,
        .resource = PMIC_PWR_RSRC_MAX + 1U,
        .startupDelay = PMIC_PWR_SEQ_DLY_MAX
    };
    int32_t status = Pmic_pwrSetSequenceCfgs(&pmicHandle, numConfig, &sequenceCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

static void powerTest_Pmic_pwrSetSequenceCfgs_outOfBounds_startupDelay(uint8_t resource)
{
    // Pass out of bounds startupDelay into Pmic_pwrSetSequenceCfgs()
    const uint8_t numConfig = 1U;
    Pmic_PowerSequenceCfg_t sequenceCfg = {
        .validParams = PMIC_PWR_SEQ_STARTUP_VALID,
        .resource = resource,
        .startupDelay = PMIC_PWR_SEQ_DLY_MAX + 1U
    };
    int32_t status = Pmic_pwrSetSequenceCfgs(&pmicHandle, numConfig, &sequenceCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

static void powerTest_Pmic_pwrSetSequenceCfgs_outOfBounds_shutdownDelay(uint8_t resource)
{
    // Pass out of bounds shutdownDelay into Pmic_pwrSetSequenceCfgs()
    const uint8_t numConfig = 1U;
    Pmic_PowerSequenceCfg_t sequenceCfg = {
        .validParams = PMIC_PWR_SEQ_SHUTDOWN_VALID,
        .resource = resource,
        .shutdownDelay = PMIC_PWR_SEQ_DLY_MAX + 1U
    };
    int32_t status = Pmic_pwrSetSequenceCfgs(&pmicHandle, numConfig, &sequenceCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrSetSequenceCfgs_outOfBounds_buck1_startupDelay(void)
{
    powerTest_Pmic_pwrSetSequenceCfgs_outOfBounds_startupDelay(PMIC_PWR_RSRC_BUCK1);
}

void test_neg_power_pwrSetSequenceCfgs_outOfBounds_buck1_shutdownDelay(void)
{
    powerTest_Pmic_pwrSetSequenceCfgs_outOfBounds_shutdownDelay(PMIC_PWR_RSRC_BUCK1);
}

void test_neg_power_pwrSetSequenceCfgs_outOfBounds_buck2_startupDelay(void)
{
    powerTest_Pmic_pwrSetSequenceCfgs_outOfBounds_startupDelay(PMIC_PWR_RSRC_BUCK2);
}

void test_neg_power_pwrSetSequenceCfgs_outOfBounds_buck2_shutdownDelay(void)
{
    powerTest_Pmic_pwrSetSequenceCfgs_outOfBounds_shutdownDelay(PMIC_PWR_RSRC_BUCK2);
}

void test_neg_power_pwrSetSequenceCfgs_outOfBounds_buck3_startupDelay(void)
{
    powerTest_Pmic_pwrSetSequenceCfgs_outOfBounds_startupDelay(PMIC_PWR_RSRC_BUCK3);
}

void test_neg_power_pwrSetSequenceCfgs_outOfBounds_buck3_shutdownDelay(void)
{
    powerTest_Pmic_pwrSetSequenceCfgs_outOfBounds_shutdownDelay(PMIC_PWR_RSRC_BUCK3);
}

void test_neg_power_pwrSetSequenceCfgs_outOfBounds_ldoLs1Vmon1_startupDelay(void)
{
    powerTest_Pmic_pwrSetSequenceCfgs_outOfBounds_startupDelay(PMIC_PWR_RSRC_LDO_LS1_VMON1);
}

void test_neg_power_pwrSetSequenceCfgs_outOfBounds_ldoLs1Vmon1_shutdownDelay(void)
{
    powerTest_Pmic_pwrSetSequenceCfgs_outOfBounds_shutdownDelay(PMIC_PWR_RSRC_LDO_LS1_VMON1);
}

void test_neg_power_pwrSetSequenceCfgs_outOfBounds_ls2Vmon2_startupDelay(void)
{
    powerTest_Pmic_pwrSetSequenceCfgs_outOfBounds_startupDelay(PMIC_PWR_RSRC_LS2_VMON2);
}

void test_neg_power_pwrSetSequenceCfgs_outOfBounds_ls2Vmon2_shutdownDelay(void)
{
    powerTest_Pmic_pwrSetSequenceCfgs_outOfBounds_shutdownDelay(PMIC_PWR_RSRC_LS2_VMON2);
}

void test_neg_power_pwrSetSequenceCfgs_outOfBounds_gpo_startupDelay(void)
{
    powerTest_Pmic_pwrSetSequenceCfgs_outOfBounds_startupDelay(PMIC_PWR_RSRC_GPO);
}

void test_neg_power_pwrSetSequenceCfgs_outOfBounds_gpo_shutdownDelay(void)
{
    powerTest_Pmic_pwrSetSequenceCfgs_outOfBounds_shutdownDelay(PMIC_PWR_RSRC_GPO);
}

void test_neg_power_pwrSetSequenceCfgs_vccaVmon_startupDelay(void)
{
    // Pass startupDelay into Pmic_pwrSetSequenceCfgs() for VCCA_VMON.
    // NOTE: For VCCA_VMON, startupDelay is invalid
    const uint8_t numConfig = 1U;
    Pmic_PowerSequenceCfg_t sequenceCfg = {
        .validParams = PMIC_PWR_SEQ_STARTUP_VALID,
        .resource = PMIC_PWR_RSRC_VCCA_VMON,
        .startupDelay = PMIC_PWR_SEQ_DLY_7P5MS
    };
    int32_t status = Pmic_pwrSetSequenceCfgs(&pmicHandle, numConfig, &sequenceCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrSetSequenceCfgs_vccaVmon_shutdownDelay(void)
{
    // Pass shutdownDelay into Pmic_pwrSetSequenceCfgs() for VCCA_VMON.
    // NOTE: For VCCA_VMON, shutdownDelay is invalid
    const uint8_t numConfig = 1U;
    Pmic_PowerSequenceCfg_t sequenceCfg = {
        .validParams = PMIC_PWR_SEQ_SHUTDOWN_VALID,
        .resource = PMIC_PWR_RSRC_VCCA_VMON,
        .shutdownDelay = PMIC_PWR_SEQ_DLY_7P5MS
    };
    int32_t status = Pmic_pwrSetSequenceCfgs(&pmicHandle, numConfig, &sequenceCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrGetSequenceCfgs_nullParam_handle(void)
{
    // Pass NULL handle into Pmic_pwrGetSequenceCfgs()
    const uint8_t numConfig = 1U;
    Pmic_PowerSequenceCfg_t sequenceCfg = {
        .validParams = PMIC_PWR_SEQ_STARTUP_VALID,
        .resource = PMIC_PWR_RSRC_BUCK1
    };
    int32_t status = Pmic_pwrGetSequenceCfgs(NULL, numConfig, &sequenceCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_power_pwrGetSequenceCfgs_nullParam_config(void)
{
    // Pass NULL config into Pmic_pwrGetSequenceCfgs()
    const uint8_t numConfig = 1U;
    int32_t status = Pmic_pwrGetSequenceCfgs(&pmicHandle, numConfig, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_power_pwrGetSequenceCfgs_outOfBounds_resource(void)
{
    // Pass out of bounds resource into Pmic_pwrGetSequenceCfgs()
    const uint8_t numConfig = 1U;
    Pmic_PowerSequenceCfg_t sequenceCfg = {
        .validParams = PMIC_PWR_SEQ_STARTUP_VALID,
        .resource = PMIC_PWR_RSRC_MAX + 1U
    };
    int32_t status = Pmic_pwrGetSequenceCfgs(&pmicHandle, numConfig, &sequenceCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

static void powerTest_enableDisableResource(uint8_t rsrc)
{
    int32_t status = PMIC_ST_SUCCESS;
    bool isEnabled = PMIC_DISABLE;

    // Enable resource
    status = Pmic_pwrSetResourceEnable(&pmicHandle, rsrc, PMIC_ENABLE);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get actual resource enable state and compare expected vs. actual values
    status = Pmic_pwrGetResourceEnable(&pmicHandle, rsrc, &isEnabled);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(isEnabled == PMIC_ENABLE);

    // Disable resource
    status = Pmic_pwrSetResourceEnable(&pmicHandle, rsrc, PMIC_DISABLE);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get actual resource enable state and compare expected vs. actual values
    status = Pmic_pwrGetResourceEnable(&pmicHandle, rsrc, &isEnabled);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(isEnabled == PMIC_DISABLE);
}

/**
 * @brief Test Pmic_pwrGetResourceCfg for LDO_LS1_VMON1 with invalid hardware state
 *
 * This test injects an invalid bit combination into FUNC_CONF_REG that doesn't
 * match any valid mode (lines 461-464 in pmic_power.c). The function should
 * return PMIC_ST_ERR_FAIL when the hardware state is invalid.
 */
void test_neg_power_pwrGetResourceCfg_ldoLs1Vmon1_invalidHwState(void)
{
#ifdef BUILD_MOCK
    extern PmicMockDevice_t* platform_getMockDevice(void);
    extern int32_t PmicMock_WriteRegister(PmicMockDevice_t* mock, uint16_t addr, uint8_t value);

    PmicMockDevice_t* mock = platform_getMockDevice();
    int32_t status;
    Pmic_PowerResourceCfg_t config = {
        .validParams = PMIC_PWR_CFG_MODE_VALID,
        .resource = PMIC_PWR_RSRC_LDO_LS1_VMON1
    };

    // Inject invalid hardware state: bypConfig=0, vmon1Sel=1, lswConfig=0
    // This combination doesn't match any valid mode in PWR_getModeCfgLdoLs1Vmon1
    // FUNC_CONF_REG bit 0 = VMON1_SEL, bit 1 = LSW_CONFIG
    // Setting only bit 0 creates an invalid state
    status = PmicMock_WriteRegister(mock, 0x1EU, 0x01U);  // FUNC_CONF_REG = 0x1E
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Attempt to get resource config - should fail due to invalid HW state
    status = Pmic_pwrGetResourceCfg(&pmicHandle, &config);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_FAIL);
#endif
}

void test_pos_power_enableDisable_buck1(void)
{
    powerTest_enableDisableResource(PMIC_PWR_RSRC_BUCK1);
}
void test_pos_power_enableDisable_buck2(void)
{
    powerTest_enableDisableResource(PMIC_PWR_RSRC_BUCK2);
}

void test_pos_power_enableDisable_buck3(void)
{
    powerTest_enableDisableResource(PMIC_PWR_RSRC_BUCK3);
}

void test_pos_power_enableDisable_ldoLs1Vmon1(void)
{
    powerTest_enableDisableResource(PMIC_PWR_RSRC_LDO_LS1_VMON1);
}

void test_pos_power_enableDisable_ls2Vmon2(void)
{
    powerTest_enableDisableResource(PMIC_PWR_RSRC_LS2_VMON2);
}

void test_pos_power_enableDisable_vccaVmon(void)
{
    powerTest_enableDisableResource(PMIC_PWR_RSRC_VCCA_VMON);
}

void test_pos_power_enableDisable_gpo(void)
{
    powerTest_enableDisableResource(PMIC_PWR_RSRC_GPO);
}

static void powerTest_setGetResourceCfg_enable(uint8_t resource)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PowerResourceCfg_t expResourceCfg = {
        .validParams = PMIC_PWR_CFG_ENABLE_VALID,
        .resource = resource
    };
    Pmic_PowerResourceCfg_t actResourceCfg = {
        .validParams = PMIC_PWR_CFG_ENABLE_VALID,
        .resource = resource
    };

    // Enable resource
    expResourceCfg.enable = PMIC_ENABLE;
    status = Pmic_pwrSetResourceCfg(&pmicHandle, &expResourceCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get actual resource enable state and compare expected vs. actual values
    status = Pmic_pwrGetResourceCfg(&pmicHandle, &actResourceCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(PMIC_ENABLE == actResourceCfg.enable);

    // Disable resource
    expResourceCfg.enable = PMIC_DISABLE;
    status = Pmic_pwrSetResourceCfg(&pmicHandle, &expResourceCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get actual resource enable state and compare expected vs. actual values
    status = Pmic_pwrGetResourceCfg(&pmicHandle, &actResourceCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(PMIC_DISABLE == actResourceCfg.enable);
}

static void powerTest_setGetResourceCfg_buck_mode(uint8_t buck)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PowerResourceCfg_t expResourceCfg = {
        .validParams = PMIC_PWR_CFG_MODE_VALID,
        .resource = buck
    };
    Pmic_PowerResourceCfg_t actResourceCfg = {
        .validParams = PMIC_PWR_CFG_MODE_VALID,
        .resource = buck
    };

    // Set Buck mode
    expResourceCfg.mode = PMIC_PWR_RSRC_MODE_REG;
    status = Pmic_pwrSetResourceCfg(&pmicHandle, &expResourceCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get actual Buck mode and compare expected vs. actual values
    status = Pmic_pwrGetResourceCfg(&pmicHandle, &actResourceCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(PMIC_PWR_RSRC_MODE_REG == actResourceCfg.mode);
}

static void powerTest_setGetResourceCfg_buck_ilim(uint8_t buck)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PowerResourceCfg_t expResourceCfg = {
        .validParams = PMIC_PWR_CFG_ILIM_VALID,
        .resource = buck
    };
    Pmic_PowerResourceCfg_t actResourceCfg = {
        .validParams = PMIC_PWR_CFG_ILIM_VALID,
        .resource = buck
    };

    for (uint8_t ilim = PMIC_PWR_ILIM_MIN; ilim <= PMIC_PWR_ILIM_MAX; ilim++)
    {
        // Set Buck ILIM
        expResourceCfg.ilim = ilim;
        status = Pmic_pwrSetResourceCfg(&pmicHandle, &expResourceCfg);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        // Get actual Buck ILIM and compare expected vs. actual values
        status = Pmic_pwrGetResourceCfg(&pmicHandle, &actResourceCfg);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        PLATFORM_ASSERT(ilim == actResourceCfg.ilim);
    }
}

static void powerTest_setGetResourceCfg_buck_voltage_mV(uint8_t buck)
{
    const uint16_t voltage_mV_min = 900U, voltage_mV_max = 1900U, voltage_mV_steps = 20U;
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PowerResourceCfg_t expResourceCfg = {
        .validParams = PMIC_PWR_CFG_VOLTAGE_VALID,
        .resource = buck
    };
    Pmic_PowerResourceCfg_t actResourceCfg = {
        .validParams = PMIC_PWR_CFG_VOLTAGE_VALID,
        .resource = buck
    };

    for (uint16_t voltage_mV = voltage_mV_min; voltage_mV <= voltage_mV_max; voltage_mV += voltage_mV_steps)
    {
        // Set Buck voltage
        expResourceCfg.voltage_mV = voltage_mV;
        status = Pmic_pwrSetResourceCfg(&pmicHandle, &expResourceCfg);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        // Get actual Buck voltage and compare expected vs. actual values
        status = Pmic_pwrGetResourceCfg(&pmicHandle, &actResourceCfg);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        PLATFORM_ASSERT(voltage_mV == actResourceCfg.voltage_mV);
    }
}

static void powerTest_setGetResourceCfg_deglitch(uint8_t resource)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PowerResourceCfg_t expResourceCfg = {
        .validParams = PMIC_PWR_CFG_DEGLITCH_VALID,
        .resource = resource
    };
    Pmic_PowerResourceCfg_t actResourceCfg = {
        .validParams = PMIC_PWR_CFG_DEGLITCH_VALID,
        .resource = resource
    };

    for (uint8_t deglitch = PMIC_PWR_DEGLITCH_MIN; deglitch <= PMIC_PWR_DEGLITCH_MAX; deglitch++)
    {
        // Set resource deglitch
        expResourceCfg.deglitch = deglitch;
        status = Pmic_pwrSetResourceCfg(&pmicHandle, &expResourceCfg);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        // Get actual resource deglitch and compare expected vs. actual values
        status = Pmic_pwrGetResourceCfg(&pmicHandle, &actResourceCfg);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        PLATFORM_ASSERT(deglitch == actResourceCfg.deglitch);
    }
}

static void powerTest_setGetResourceCfg_buck_uvThresh(uint8_t buck)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PowerResourceCfg_t expResourceCfg = {
        .validParams = PMIC_PWR_CFG_UV_THRESH_VALID,
        .resource = buck
    };
    Pmic_PowerResourceCfg_t actResourceCfg = {
        .validParams = PMIC_PWR_CFG_UV_THRESH_VALID,
        .resource = buck
    };

    for (uint16_t uvThresh = PMIC_PWR_BUCK_UV_OV_THR_MIN; uvThresh <= PMIC_PWR_BUCK_UV_OV_THR_MAX; uvThresh++)
    {
        // Set Buck UV threshold
        expResourceCfg.uvThresh = uvThresh;
        status = Pmic_pwrSetResourceCfg(&pmicHandle, &expResourceCfg);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        // Get actual Buck UV threshold and compare expected vs. actual values
        status = Pmic_pwrGetResourceCfg(&pmicHandle, &actResourceCfg);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        PLATFORM_ASSERT(uvThresh == actResourceCfg.uvThresh);
    }
}

static void powerTest_setGetResourceCfg_uvReaction(uint8_t resource)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PowerResourceCfg_t expResourceCfg = {
        .validParams = PMIC_PWR_CFG_UV_REACT_VALID,
        .resource = resource
    };
    Pmic_PowerResourceCfg_t actResourceCfg = {
        .validParams = PMIC_PWR_CFG_UV_REACT_VALID,
        .resource = resource
    };

    for (uint8_t uvReaction = PMIC_PWR_FAULT_REACT_MIN; uvReaction <= PMIC_PWR_FAULT_REACT_MAX; uvReaction++)
    {
        // Set resource UV reaction
        expResourceCfg.uvReaction = uvReaction;
        status = Pmic_pwrSetResourceCfg(&pmicHandle, &expResourceCfg);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        // Get actual resource UV reaction and compare expected vs. actual values
        status = Pmic_pwrGetResourceCfg(&pmicHandle, &actResourceCfg);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        PLATFORM_ASSERT(uvReaction == actResourceCfg.uvReaction);
    }
}

static void powerTest_setGetResourceCfg_buck_ovThresh(uint8_t buck)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PowerResourceCfg_t expResourceCfg = {
        .validParams = PMIC_PWR_CFG_OV_THRESH_VALID,
        .resource = buck
    };
    Pmic_PowerResourceCfg_t actResourceCfg = {
        .validParams = PMIC_PWR_CFG_OV_THRESH_VALID,
        .resource = buck
    };

    for (uint16_t ovThresh = PMIC_PWR_BUCK_UV_OV_THR_MIN; ovThresh <= PMIC_PWR_BUCK_UV_OV_THR_MAX; ovThresh++)
    {
        // Set Buck OV threshold
        expResourceCfg.ovThresh = ovThresh;
        status = Pmic_pwrSetResourceCfg(&pmicHandle, &expResourceCfg);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        // Get actual Buck OV threshold and compare expected vs. actual values
        status = Pmic_pwrGetResourceCfg(&pmicHandle, &actResourceCfg);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        PLATFORM_ASSERT(ovThresh == actResourceCfg.ovThresh);
    }
}

static void powerTest_setGetResourceCfg_ovReaction(uint8_t resource)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PowerResourceCfg_t expResourceCfg = {
        .validParams = PMIC_PWR_CFG_OV_REACT_VALID,
        .resource = resource
    };
    Pmic_PowerResourceCfg_t actResourceCfg = {
        .validParams = PMIC_PWR_CFG_OV_REACT_VALID,
        .resource = resource
    };

    for (uint8_t ovReaction = PMIC_PWR_FAULT_REACT_MIN; ovReaction <= PMIC_PWR_FAULT_REACT_MAX; ovReaction++)
    {
        // Set resource OV reaction
        expResourceCfg.ovReaction = ovReaction;
        status = Pmic_pwrSetResourceCfg(&pmicHandle, &expResourceCfg);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        // Get actual resource OV reaction and compare expected vs. actual values
        status = Pmic_pwrGetResourceCfg(&pmicHandle, &actResourceCfg);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        PLATFORM_ASSERT(ovReaction == actResourceCfg.ovReaction);
    }
}

static void powerTest_setGetResourceCfg_rvReaction(uint8_t resource)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PowerResourceCfg_t expResourceCfg = {
        .validParams = PMIC_PWR_CFG_RV_REACT_VALID,
        .resource = resource
    };
    Pmic_PowerResourceCfg_t actResourceCfg = {
        .validParams = PMIC_PWR_CFG_RV_REACT_VALID,
        .resource = resource
    };

    for (uint8_t rvReaction = PMIC_PWR_RV_REACT_MIN; rvReaction <= PMIC_PWR_RV_REACT_MAX; rvReaction++)
    {
        // Set resource RV reaction
        expResourceCfg.rvReaction = rvReaction;
        status = Pmic_pwrSetResourceCfg(&pmicHandle, &expResourceCfg);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        // Get actual resource RV reaction and compare expected vs. actual values
        status = Pmic_pwrGetResourceCfg(&pmicHandle, &actResourceCfg);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        PLATFORM_ASSERT(rvReaction == actResourceCfg.rvReaction);
    }
}

static void powerTest_setGetResourceCfg_scReaction(uint8_t resource)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PowerResourceCfg_t expResourceCfg = {
        .validParams = PMIC_PWR_CFG_SC_REACT_VALID,
        .resource = resource
    };
    Pmic_PowerResourceCfg_t actResourceCfg = {
        .validParams = PMIC_PWR_CFG_SC_REACT_VALID,
        .resource = resource
    };

    for (uint8_t scReaction = PMIC_PWR_FAULT_REACT_MIN; scReaction <= PMIC_PWR_FAULT_REACT_MAX; scReaction++)
    {
        // Set resource SC reaction
        expResourceCfg.scReaction = scReaction;
        status = Pmic_pwrSetResourceCfg(&pmicHandle, &expResourceCfg);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        // Get actual resource SC reaction and compare expected vs. actual values
        status = Pmic_pwrGetResourceCfg(&pmicHandle, &actResourceCfg);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        PLATFORM_ASSERT(scReaction == actResourceCfg.scReaction);
    }
}

void test_pos_power_setGetResourceCfg_buck1_enable(void)
{
    powerTest_setGetResourceCfg_enable(PMIC_PWR_RSRC_BUCK1);
}

void test_pos_power_setGetResourceCfg_buck1_mode(void)
{
    powerTest_setGetResourceCfg_buck_mode(PMIC_PWR_RSRC_BUCK1);
}

void test_pos_power_setGetResourceCfg_buck1_ilim(void)
{
    powerTest_setGetResourceCfg_buck_ilim(PMIC_PWR_RSRC_BUCK1);
}

void test_pos_power_setGetResourceCfg_buck1_voltage_mV(void)
{
    powerTest_setGetResourceCfg_buck_voltage_mV(PMIC_PWR_RSRC_BUCK1);
}

void test_pos_power_setGetResourceCfg_buck1_deglitch(void)
{
    powerTest_setGetResourceCfg_deglitch(PMIC_PWR_RSRC_BUCK1);
}

void test_pos_power_setGetResourceCfg_buck1_uvThresh(void)
{
    powerTest_setGetResourceCfg_buck_uvThresh(PMIC_PWR_RSRC_BUCK1);
}

void test_pos_power_setGetResourceCfg_buck1_uvReaction(void)
{
    powerTest_setGetResourceCfg_uvReaction(PMIC_PWR_RSRC_BUCK1);
}

void test_pos_power_setGetResourceCfg_buck1_ovThresh(void)
{
    powerTest_setGetResourceCfg_buck_ovThresh(PMIC_PWR_RSRC_BUCK1);
}

void test_pos_power_setGetResourceCfg_buck1_ovReaction(void)
{
    powerTest_setGetResourceCfg_ovReaction(PMIC_PWR_RSRC_BUCK1);
}

void test_pos_power_setGetResourceCfg_buck1_rvReaction(void)
{
    powerTest_setGetResourceCfg_rvReaction(PMIC_PWR_RSRC_BUCK1);
}

void test_pos_power_setGetResourceCfg_buck1_scReaction(void)
{
    powerTest_setGetResourceCfg_scReaction(PMIC_PWR_RSRC_BUCK1);
}

void test_pos_power_setGetResourceCfg_buck2_enable(void)
{
    powerTest_setGetResourceCfg_enable(PMIC_PWR_RSRC_BUCK2);
}

void test_pos_power_setGetResourceCfg_buck2_mode(void)
{
    powerTest_setGetResourceCfg_buck_mode(PMIC_PWR_RSRC_BUCK2);
}

void test_pos_power_setGetResourceCfg_buck2_ilim(void)
{
    powerTest_setGetResourceCfg_buck_ilim(PMIC_PWR_RSRC_BUCK2);
}

void test_pos_power_setGetResourceCfg_buck2_voltage_mV(void)
{
    powerTest_setGetResourceCfg_buck_voltage_mV(PMIC_PWR_RSRC_BUCK2);
}

void test_pos_power_setGetResourceCfg_buck2_deglitch(void)
{
    powerTest_setGetResourceCfg_deglitch(PMIC_PWR_RSRC_BUCK2);
}

void test_pos_power_setGetResourceCfg_buck2_uvThresh(void)
{
    powerTest_setGetResourceCfg_buck_uvThresh(PMIC_PWR_RSRC_BUCK2);
}

void test_pos_power_setGetResourceCfg_buck2_uvReaction(void)
{
    powerTest_setGetResourceCfg_uvReaction(PMIC_PWR_RSRC_BUCK2);
}

void test_pos_power_setGetResourceCfg_buck2_ovThresh(void)
{
    powerTest_setGetResourceCfg_buck_ovThresh(PMIC_PWR_RSRC_BUCK2);
}

void test_pos_power_setGetResourceCfg_buck2_ovReaction(void)
{
    powerTest_setGetResourceCfg_ovReaction(PMIC_PWR_RSRC_BUCK2);
}

void test_pos_power_setGetResourceCfg_buck2_rvReaction(void)
{
    powerTest_setGetResourceCfg_rvReaction(PMIC_PWR_RSRC_BUCK2);
}

void test_pos_power_setGetResourceCfg_buck2_scReaction(void)
{
    powerTest_setGetResourceCfg_scReaction(PMIC_PWR_RSRC_BUCK2);
}

void test_pos_power_setGetResourceCfg_buck3_enable(void)
{
    powerTest_setGetResourceCfg_enable(PMIC_PWR_RSRC_BUCK3);
}

void test_pos_power_setGetResourceCfg_buck3_mode(void)
{
    powerTest_setGetResourceCfg_buck_mode(PMIC_PWR_RSRC_BUCK3);
}

void test_pos_power_setGetResourceCfg_buck3_ilim(void)
{
    powerTest_setGetResourceCfg_buck_ilim(PMIC_PWR_RSRC_BUCK3);
}

void test_pos_power_setGetResourceCfg_buck3_voltage_mV(void)
{
    powerTest_setGetResourceCfg_buck_voltage_mV(PMIC_PWR_RSRC_BUCK3);
}

void test_pos_power_setGetResourceCfg_buck3_deglitch(void)
{
    powerTest_setGetResourceCfg_deglitch(PMIC_PWR_RSRC_BUCK3);
}

void test_pos_power_setGetResourceCfg_buck3_uvThresh(void)
{
    powerTest_setGetResourceCfg_buck_uvThresh(PMIC_PWR_RSRC_BUCK3);
}

void test_pos_power_setGetResourceCfg_buck3_uvReaction(void)
{
    powerTest_setGetResourceCfg_uvReaction(PMIC_PWR_RSRC_BUCK3);
}

void test_pos_power_setGetResourceCfg_buck3_ovThresh(void)
{
    powerTest_setGetResourceCfg_buck_ovThresh(PMIC_PWR_RSRC_BUCK3);
}

void test_pos_power_setGetResourceCfg_buck3_ovReaction(void)
{
    powerTest_setGetResourceCfg_ovReaction(PMIC_PWR_RSRC_BUCK3);
}

void test_pos_power_setGetResourceCfg_buck3_rvReaction(void)
{
    powerTest_setGetResourceCfg_rvReaction(PMIC_PWR_RSRC_BUCK3);
}

void test_pos_power_setGetResourceCfg_buck3_scReaction(void)
{
    powerTest_setGetResourceCfg_scReaction(PMIC_PWR_RSRC_BUCK3);
}

void test_pos_power_setGetResourceCfg_ldoLs1Vmon1_enable(void)
{
    powerTest_setGetResourceCfg_enable(PMIC_PWR_RSRC_LDO_LS1_VMON1);
}

void test_pos_power_setGetResourceCfg_ldoLs1Vmon1_mode(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PowerResourceCfg_t expResourceCfg = {
        .validParams = PMIC_PWR_CFG_MODE_VALID,
        .resource = PMIC_PWR_RSRC_LDO_LS1_VMON1
    };
    Pmic_PowerResourceCfg_t actResourceCfg = {
        .validParams = PMIC_PWR_CFG_MODE_VALID,
        .resource = PMIC_PWR_RSRC_LDO_LS1_VMON1
    };

    // Pass all valid modes into Pmic_pwrSetResourceCfg() for LDO_LS1_VMON1
    for (uint8_t mode = PMIC_PWR_RSRC_MODE_REG; mode <= PMIC_PWR_RSRC_MODE_VMON; mode++)
    {
        // Set LDO_LS1_VMON1 mode
        expResourceCfg.mode = mode;
        status = Pmic_pwrSetResourceCfg(&pmicHandle, &expResourceCfg);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        // Get actual LDO_LS1_VMON1 mode and compare expected vs. actual values
        status = Pmic_pwrGetResourceCfg(&pmicHandle, &actResourceCfg);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        PLATFORM_ASSERT(mode == actResourceCfg.mode);
    }
}

static void powerTest_setGetResourceCfg_ldoLsVmon_voltage_mV(uint8_t ldoLsVmon)
{
    const uint16_t voltage_mV_min = 600U, voltage_mV_max = 3400U, voltage_mV_steps = 25U;
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PowerResourceCfg_t expResourceCfg = {
        .validParams = PMIC_PWR_CFG_VOLTAGE_VALID,
        .resource = ldoLsVmon
    };
    Pmic_PowerResourceCfg_t actResourceCfg = {
        .validParams = PMIC_PWR_CFG_VOLTAGE_VALID,
        .resource = ldoLsVmon
    };

    // Pass all valid voltages into Pmic_pwrSetResourceCfg() for LDO/LS/VMON
    for (uint16_t voltage_mV = voltage_mV_min; voltage_mV <= voltage_mV_max; voltage_mV += voltage_mV_steps)
    {
        // Set LDO/LS/VMON voltage
        expResourceCfg.voltage_mV = voltage_mV;
        status = Pmic_pwrSetResourceCfg(&pmicHandle, &expResourceCfg);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        // Get actual LDO/LS/VMON voltage and compare expected vs. actual values
        status = Pmic_pwrGetResourceCfg(&pmicHandle, &actResourceCfg);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        PLATFORM_ASSERT(voltage_mV == actResourceCfg.voltage_mV);
    }
}

void test_pos_power_setGetResourceCfg_ldoLs1Vmon1_voltage_mV(void)
{
    powerTest_setGetResourceCfg_ldoLsVmon_voltage_mV(PMIC_PWR_RSRC_LDO_LS1_VMON1);
}

void test_pos_power_setGetResourceCfg_ldoLs1Vmon1_deglitch(void)
{
    powerTest_setGetResourceCfg_deglitch(PMIC_PWR_RSRC_LDO_LS1_VMON1);
}

void test_pos_power_setGetResourceCfg_ldoLs1Vmon1_uvThresh(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PowerResourceCfg_t expResourceCfg = {
        .validParams = PMIC_PWR_CFG_UV_THRESH_VALID,
        .resource = PMIC_PWR_RSRC_LDO_LS1_VMON1
    };
    Pmic_PowerResourceCfg_t actResourceCfg = {
        .validParams = PMIC_PWR_CFG_UV_THRESH_VALID,
        .resource = PMIC_PWR_RSRC_LDO_LS1_VMON1
    };

    // Pass all valid UV thresholds into Pmic_pwrSetResourceCfg() for LDO_LS1_VMON1
    for (uint16_t uvThresh = PMIC_PWR_LS_UV_OV_THR_MIN; uvThresh <= PMIC_PWR_LS_UV_OV_THR_MAX; uvThresh++)
    {
        // Set LDO_LS1_VMON1 UV threshold
        expResourceCfg.uvThresh = uvThresh;
        status = Pmic_pwrSetResourceCfg(&pmicHandle, &expResourceCfg);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        // Get actual LDO_LS1_VMON1 UV threshold and compare expected vs. actual values
        status = Pmic_pwrGetResourceCfg(&pmicHandle, &actResourceCfg);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        PLATFORM_ASSERT(uvThresh == actResourceCfg.uvThresh);
    }
}

void test_pos_power_setGetResourceCfg_ldoLs1Vmon1_uvReaction(void)
{
    powerTest_setGetResourceCfg_uvReaction(PMIC_PWR_RSRC_LDO_LS1_VMON1);
}

void test_pos_power_setGetResourceCfg_ldoLs1Vmon1_ovThresh(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PowerResourceCfg_t expResourceCfg = {
        .validParams = PMIC_PWR_CFG_OV_THRESH_VALID,
        .resource = PMIC_PWR_RSRC_LDO_LS1_VMON1
    };
    Pmic_PowerResourceCfg_t actResourceCfg = {
        .validParams = PMIC_PWR_CFG_OV_THRESH_VALID,
        .resource = PMIC_PWR_RSRC_LDO_LS1_VMON1
    };

    // Pass all valid OV thresholds into Pmic_pwrSetResourceCfg() for LDO_LS1_VMON1
    for (uint16_t ovThresh = PMIC_PWR_LS_UV_OV_THR_MIN; ovThresh <= PMIC_PWR_LS_UV_OV_THR_MAX; ovThresh++)
    {
        // Set LDO_LS1_VMON1 OV threshold
        expResourceCfg.ovThresh = ovThresh;
        status = Pmic_pwrSetResourceCfg(&pmicHandle, &expResourceCfg);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        // Get actual LDO_LS1_VMON1 OV threshold and compare expected vs. actual values
        status = Pmic_pwrGetResourceCfg(&pmicHandle, &actResourceCfg);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        PLATFORM_ASSERT(ovThresh == actResourceCfg.ovThresh);
    }
}

void test_pos_power_setGetResourceCfg_ldoLs1Vmon1_ovReaction(void)
{
    powerTest_setGetResourceCfg_ovReaction(PMIC_PWR_RSRC_LDO_LS1_VMON1);
}

void test_pos_power_setGetResourceCfg_ldoLs1Vmon1_rvReaction(void)
{
    powerTest_setGetResourceCfg_rvReaction(PMIC_PWR_RSRC_LDO_LS1_VMON1);
}

void test_pos_power_setGetResourceCfg_ldoLs1Vmon1_scReaction(void)
{
    powerTest_setGetResourceCfg_scReaction(PMIC_PWR_RSRC_LDO_LS1_VMON1);
}

void test_pos_power_setGetResourceCfg_ls2Vmon2_enable(void)
{
    powerTest_setGetResourceCfg_enable(PMIC_PWR_RSRC_LS2_VMON2);
}

void test_pos_power_setGetResourceCfg_ls2Vmon2_mode(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PowerResourceCfg_t expResourceCfg = {
        .validParams = PMIC_PWR_CFG_MODE_VALID,
        .resource = PMIC_PWR_RSRC_LS2_VMON2
    };
    Pmic_PowerResourceCfg_t actResourceCfg = {
        .validParams = PMIC_PWR_CFG_MODE_VALID,
        .resource = PMIC_PWR_RSRC_LS2_VMON2
    };

    // Pass all valid modes into Pmic_pwrSetResourceCfg() for LS2_VMON2
    for (uint8_t mode = PMIC_PWR_RSRC_MODE_MIN; mode <= PMIC_PWR_RSRC_MODE_MAX; mode++)
    {
        if ((mode == PMIC_PWR_RSRC_MODE_LSW) || (mode == PMIC_PWR_RSRC_MODE_VMON))
        {
            // Set LS2_VMON2 mode
            expResourceCfg.mode = mode;
            status = Pmic_pwrSetResourceCfg(&pmicHandle, &expResourceCfg);
            PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

            // Get actual LS2_VMON2 mode and compare expected vs. actual values
            status = Pmic_pwrGetResourceCfg(&pmicHandle, &actResourceCfg);
            PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
            PLATFORM_ASSERT(mode == actResourceCfg.mode);
        }
    }
}

void test_pos_power_setGetResourceCfg_ls2Vmon2_voltage_mV(void)
{
    powerTest_setGetResourceCfg_ldoLsVmon_voltage_mV(PMIC_PWR_RSRC_LS2_VMON2);
}

void test_pos_power_setGetResourceCfg_ls2Vmon2_deglitch(void)
{
    powerTest_setGetResourceCfg_deglitch(PMIC_PWR_RSRC_LS2_VMON2);
}

void test_pos_power_setGetResourceCfg_ls2Vmon2_uvThresh(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PowerResourceCfg_t expResourceCfg = {
        .validParams = PMIC_PWR_CFG_UV_THRESH_VALID,
        .resource = PMIC_PWR_RSRC_LS2_VMON2
    };
    Pmic_PowerResourceCfg_t actResourceCfg = {
        .validParams = PMIC_PWR_CFG_UV_THRESH_VALID,
        .resource = PMIC_PWR_RSRC_LS2_VMON2
    };

    // Pass all valid UV thresholds into Pmic_pwrSetResourceCfg() for LS2_VMON2
    for (uint16_t uvThresh = PMIC_PWR_LS_UV_OV_THR_MIN; uvThresh <= PMIC_PWR_LS_UV_OV_THR_MAX; uvThresh++)
    {
        // Set LS2_VMON2 UV threshold
        expResourceCfg.uvThresh = uvThresh;
        status = Pmic_pwrSetResourceCfg(&pmicHandle, &expResourceCfg);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        // Get actual LS2_VMON2 UV threshold and compare expected vs. actual values
        status = Pmic_pwrGetResourceCfg(&pmicHandle, &actResourceCfg);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        PLATFORM_ASSERT(uvThresh == actResourceCfg.uvThresh);
    }
}

void test_pos_power_setGetResourceCfg_ls2Vmon2_uvReaction(void)
{
    powerTest_setGetResourceCfg_uvReaction(PMIC_PWR_RSRC_LS2_VMON2);
}

void test_pos_power_setGetResourceCfg_ls2Vmon2_ovThresh(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PowerResourceCfg_t expResourceCfg = {
        .validParams = PMIC_PWR_CFG_OV_THRESH_VALID,
        .resource = PMIC_PWR_RSRC_LS2_VMON2
    };
    Pmic_PowerResourceCfg_t actResourceCfg = {
        .validParams = PMIC_PWR_CFG_OV_THRESH_VALID,
        .resource = PMIC_PWR_RSRC_LS2_VMON2
    };

    // Pass all valid OV thresholds into Pmic_pwrSetResourceCfg() for LS2_VMON2
    for (uint16_t ovThresh = PMIC_PWR_LS_UV_OV_THR_MIN; ovThresh <= PMIC_PWR_LS_UV_OV_THR_MAX; ovThresh++)
    {
        // Set LS2_VMON2 OV threshold
        expResourceCfg.ovThresh = ovThresh;
        status = Pmic_pwrSetResourceCfg(&pmicHandle, &expResourceCfg);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        // Get actual LS2_VMON2 OV threshold and compare expected vs. actual values
        status = Pmic_pwrGetResourceCfg(&pmicHandle, &actResourceCfg);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        PLATFORM_ASSERT(ovThresh == actResourceCfg.ovThresh);
    }
}

void test_pos_power_setGetResourceCfg_ls2Vmon2_ovReaction(void)
{
    powerTest_setGetResourceCfg_ovReaction(PMIC_PWR_RSRC_LS2_VMON2);
}

void test_pos_power_setGetResourceCfg_ls2Vmon2_rvReaction(void)
{
    powerTest_setGetResourceCfg_rvReaction(PMIC_PWR_RSRC_LS2_VMON2);
}

void test_pos_power_setGetResourceCfg_ls2Vmon2_scReaction(void)
{
    powerTest_setGetResourceCfg_scReaction(PMIC_PWR_RSRC_LS2_VMON2);
}

void test_pos_power_setGetResourceCfg_vccaVmon_enable(void)
{
    powerTest_setGetResourceCfg_enable(PMIC_PWR_RSRC_VCCA_VMON);
}

void test_pos_power_setGetResourceCfg_vccaVmon_mode(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PowerResourceCfg_t expResourceCfg = {
        .validParams = PMIC_PWR_CFG_MODE_VALID,
        .resource = PMIC_PWR_RSRC_VCCA_VMON
    };
    Pmic_PowerResourceCfg_t actResourceCfg = {
        .validParams = PMIC_PWR_CFG_MODE_VALID,
        .resource = PMIC_PWR_RSRC_VCCA_VMON
    };

    // Set VCCA_VMON mode
    expResourceCfg.mode = PMIC_PWR_RSRC_MODE_VMON;
    status = Pmic_pwrSetResourceCfg(&pmicHandle, &expResourceCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get actual VCCA_VMON mode and compare expected vs. actual values
    status = Pmic_pwrGetResourceCfg(&pmicHandle, &actResourceCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(PMIC_PWR_RSRC_MODE_VMON == actResourceCfg.mode);
}

void test_pos_power_setGetResourceCfg_vccaVmon_voltage_mV(void)
{
    powerTest_setGetResourceCfg_ldoLsVmon_voltage_mV(PMIC_PWR_RSRC_VCCA_VMON);
}

void test_pos_power_setGetResourceCfg_vccaVmon_deglitch(void)
{
    powerTest_setGetResourceCfg_deglitch(PMIC_PWR_RSRC_VCCA_VMON);
}

void test_pos_power_setGetResourceCfg_vccaVmon_uvThresh(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PowerResourceCfg_t expResourceCfg = {
        .validParams = PMIC_PWR_CFG_UV_THRESH_VALID,
        .resource = PMIC_PWR_RSRC_VCCA_VMON
    };
    Pmic_PowerResourceCfg_t actResourceCfg = {
        .validParams = PMIC_PWR_CFG_UV_THRESH_VALID,
        .resource = PMIC_PWR_RSRC_VCCA_VMON
    };

    // Pass all valid UV thresholds into Pmic_pwrSetResourceCfg() for VCCA_VMON
    for (uint16_t uvThresh = PMIC_PWR_VCCA_UV_OV_THR_MIN; uvThresh <= PMIC_PWR_VCCA_UV_OV_THR_MAX; uvThresh++)
    {
        // Set VCCA_VMON UV threshold
        expResourceCfg.uvThresh = uvThresh;
        status = Pmic_pwrSetResourceCfg(&pmicHandle, &expResourceCfg);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        // Get actual VCCA_VMON UV threshold and compare expected vs. actual values
        status = Pmic_pwrGetResourceCfg(&pmicHandle, &actResourceCfg);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        PLATFORM_ASSERT(uvThresh == actResourceCfg.uvThresh);
    }
}

void test_pos_power_setGetResourceCfg_vccaVmon_uvReaction(void)
{
    powerTest_setGetResourceCfg_uvReaction(PMIC_PWR_RSRC_VCCA_VMON);
}

void test_pos_power_setGetResourceCfg_vccaVmon_ovThresh(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PowerResourceCfg_t expResourceCfg = {
        .validParams = PMIC_PWR_CFG_OV_THRESH_VALID,
        .resource = PMIC_PWR_RSRC_VCCA_VMON
    };
    Pmic_PowerResourceCfg_t actResourceCfg = {
        .validParams = PMIC_PWR_CFG_OV_THRESH_VALID,
        .resource = PMIC_PWR_RSRC_VCCA_VMON
    };

    // Pass all valid OV thresholds into Pmic_pwrSetResourceCfg() for VCCA_VMON
    for (uint16_t ovThresh = PMIC_PWR_VCCA_UV_OV_THR_MIN; ovThresh <= PMIC_PWR_VCCA_UV_OV_THR_MAX; ovThresh++)
    {
        // Set VCCA_VMON OV threshold
        expResourceCfg.ovThresh = ovThresh;
        status = Pmic_pwrSetResourceCfg(&pmicHandle, &expResourceCfg);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        // Get actual VCCA_VMON OV threshold and compare expected vs. actual values
        status = Pmic_pwrGetResourceCfg(&pmicHandle, &actResourceCfg);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        PLATFORM_ASSERT(ovThresh == actResourceCfg.ovThresh);
    }
}

void test_pos_power_setGetResourceCfg_vccaVmon_ovReaction(void)
{
    powerTest_setGetResourceCfg_ovReaction(PMIC_PWR_RSRC_VCCA_VMON);
}

void test_pos_power_setGetResourceCfg_gpo_enable(void)
{
    powerTest_setGetResourceCfg_enable(PMIC_PWR_RSRC_GPO);
}

static inline void powerTest_initBuckCfg(Pmic_PowerResourceCfg_t *resourceCfg, uint8_t rsrc, bool isExpCfg)
{
    resourceCfg->validParams = PMIC_PWR_CFG_ALL_VALID;
    resourceCfg->resource = rsrc;

    if (isExpCfg)
    {
        resourceCfg->enable = PMIC_DISABLE;
        resourceCfg->mode = PMIC_PWR_RSRC_MODE_REG;
        resourceCfg->ilim = PMIC_PWR_ILIM_MIN + 1U;
        resourceCfg->voltage_mV = 1000U;
        resourceCfg->deglitch = PMIC_PWR_DEGLITCH_MIN + 1U;
        resourceCfg->uvThresh = PMIC_PWR_BUCK_UV_OV_THR_MIN + 1U;
        resourceCfg->uvReaction = PMIC_PWR_FAULT_REACT_MIN + 1U;
        resourceCfg->ovThresh = PMIC_PWR_BUCK_UV_OV_THR_MIN + 1U;
        resourceCfg->ovReaction = PMIC_PWR_FAULT_REACT_MIN + 1U;
        resourceCfg->rvReaction = PMIC_PWR_RV_REACT_MIN + 1U;
        resourceCfg->scReaction = PMIC_PWR_FAULT_REACT_MIN + 1U;
    }
    else
    {
        resourceCfg->enable = PMIC_DISABLE;
        resourceCfg->mode = 0U;
        resourceCfg->ilim = 0U;
        resourceCfg->voltage_mV = 0U;
        resourceCfg->deglitch = 0U;
        resourceCfg->uvThresh = 0U;
        resourceCfg->uvReaction = 0U;
        resourceCfg->ovThresh = 0U;
        resourceCfg->ovReaction = 0U;
        resourceCfg->rvReaction = 0U;
        resourceCfg->scReaction = 0U;
    }
}

static inline void powerTest_initLdoLs1VmonCfg(Pmic_PowerResourceCfg_t *resourceCfg, uint8_t rsrc, bool isExpCfg)
{
    resourceCfg->validParams = PMIC_PWR_CFG_ENABLE_VALID |
                               PMIC_PWR_CFG_MODE_VALID |
                               PMIC_PWR_CFG_VOLTAGE_VALID |
                               PMIC_PWR_CFG_DEGLITCH_VALID |
                               PMIC_PWR_CFG_UV_THRESH_VALID |
                               PMIC_PWR_CFG_UV_REACT_VALID |
                               PMIC_PWR_CFG_OV_THRESH_VALID |
                               PMIC_PWR_CFG_OV_REACT_VALID |
                               PMIC_PWR_CFG_RV_REACT_VALID |
                               PMIC_PWR_CFG_SC_REACT_VALID;
    resourceCfg->resource = rsrc;

    if (isExpCfg)
    {
        resourceCfg->enable = PMIC_DISABLE;
        resourceCfg->mode = PMIC_PWR_RSRC_MODE_REG;
        resourceCfg->voltage_mV = 1000U;
        resourceCfg->deglitch = PMIC_PWR_DEGLITCH_MIN + 1U;
        resourceCfg->uvThresh = PMIC_PWR_LS_UV_OV_THR_MIN + 1U;
        resourceCfg->uvReaction = PMIC_PWR_FAULT_REACT_MIN + 1U;
        resourceCfg->ovThresh = PMIC_PWR_LS_UV_OV_THR_MIN + 1U;
        resourceCfg->ovReaction = PMIC_PWR_FAULT_REACT_MIN + 1U;
        resourceCfg->rvReaction = PMIC_PWR_RV_REACT_MIN + 1U;
        resourceCfg->scReaction = PMIC_PWR_FAULT_REACT_MIN + 1U;
    }
    else
    {
        resourceCfg->enable = PMIC_DISABLE;
        resourceCfg->mode = 0U;
        resourceCfg->voltage_mV = 0U;
        resourceCfg->deglitch = 0U;
        resourceCfg->uvThresh = 0U;
        resourceCfg->uvReaction = 0U;
        resourceCfg->ovThresh = 0U;
        resourceCfg->ovReaction = 0U;
        resourceCfg->rvReaction = 0U;
        resourceCfg->scReaction = 0U;
    }
}

static inline void powerTest_initLs2Vmon2Cfg(Pmic_PowerResourceCfg_t *resourceCfg, uint8_t rsrc, bool isExpCfg)
{
    resourceCfg->validParams = PMIC_PWR_CFG_ENABLE_VALID |
                               PMIC_PWR_CFG_MODE_VALID |
                               PMIC_PWR_CFG_VOLTAGE_VALID |
                               PMIC_PWR_CFG_DEGLITCH_VALID |
                               PMIC_PWR_CFG_UV_THRESH_VALID |
                               PMIC_PWR_CFG_UV_REACT_VALID |
                               PMIC_PWR_CFG_OV_THRESH_VALID |
                               PMIC_PWR_CFG_OV_REACT_VALID |
                               PMIC_PWR_CFG_RV_REACT_VALID |
                               PMIC_PWR_CFG_SC_REACT_VALID;
    resourceCfg->resource = rsrc;

    if (isExpCfg)
    {
        resourceCfg->enable = PMIC_DISABLE;
        resourceCfg->mode = PMIC_PWR_RSRC_MODE_LSW;
        resourceCfg->voltage_mV = 1000U;
        resourceCfg->deglitch = PMIC_PWR_DEGLITCH_MIN + 1U;
        resourceCfg->uvThresh = PMIC_PWR_LS_UV_OV_THR_MIN + 1U;
        resourceCfg->uvReaction = PMIC_PWR_FAULT_REACT_MIN + 1U;
        resourceCfg->ovThresh = PMIC_PWR_LS_UV_OV_THR_MIN + 1U;
        resourceCfg->ovReaction = PMIC_PWR_FAULT_REACT_MIN + 1U;
        resourceCfg->rvReaction = PMIC_PWR_RV_REACT_MIN + 1U;
        resourceCfg->scReaction = PMIC_PWR_FAULT_REACT_MIN + 1U;
    }
    else
    {
        resourceCfg->enable = PMIC_DISABLE;
        resourceCfg->mode = 0U;
        resourceCfg->voltage_mV = 0U;
        resourceCfg->deglitch = 0U;
        resourceCfg->uvThresh = 0U;
        resourceCfg->uvReaction = 0U;
        resourceCfg->ovThresh = 0U;
        resourceCfg->ovReaction = 0U;
        resourceCfg->rvReaction = 0U;
        resourceCfg->scReaction = 0U;
    }
}

static inline void powerTest_initVccaVmonCfg(Pmic_PowerResourceCfg_t *resourceCfg, uint8_t rsrc, bool isExpCfg)
{
    resourceCfg->validParams = PMIC_PWR_CFG_ENABLE_VALID |
                               PMIC_PWR_CFG_MODE_VALID |
                               PMIC_PWR_CFG_VOLTAGE_VALID |
                               PMIC_PWR_CFG_DEGLITCH_VALID |
                               PMIC_PWR_CFG_UV_THRESH_VALID |
                               PMIC_PWR_CFG_UV_REACT_VALID |
                               PMIC_PWR_CFG_OV_THRESH_VALID |
                               PMIC_PWR_CFG_OV_REACT_VALID;
    resourceCfg->resource = rsrc;

    if (isExpCfg)
    {
        resourceCfg->enable = PMIC_DISABLE;
        resourceCfg->mode = PMIC_PWR_RSRC_MODE_VMON;
        resourceCfg->voltage_mV = 1000U;
        resourceCfg->deglitch = PMIC_PWR_DEGLITCH_MIN + 1U;
        resourceCfg->uvThresh = PMIC_PWR_VCCA_UV_OV_THR_MIN + 1U;
        resourceCfg->uvReaction = PMIC_PWR_FAULT_REACT_MIN + 1U;
        resourceCfg->ovThresh = PMIC_PWR_VCCA_UV_OV_THR_MIN + 1U;
        resourceCfg->ovReaction = PMIC_PWR_FAULT_REACT_MIN + 1U;
    }
    else
    {
        resourceCfg->enable = PMIC_DISABLE;
        resourceCfg->mode = 0U;
        resourceCfg->voltage_mV = 0U;
        resourceCfg->deglitch = 0U;
        resourceCfg->uvThresh = 0U;
        resourceCfg->uvReaction = 0U;
        resourceCfg->ovThresh = 0U;
        resourceCfg->ovReaction = 0U;
    }
}

static inline void powerTest_initGpoCfg(Pmic_PowerResourceCfg_t *resourceCfg, uint8_t rsrc, bool isExpCfg)
{
    resourceCfg->validParams = PMIC_PWR_CFG_ENABLE_VALID;
    resourceCfg->resource = rsrc;
    resourceCfg->enable = isExpCfg ? PMIC_DISABLE : PMIC_ENABLE;
}

static inline void powerTest_initAllRsrcCfg(Pmic_PowerResourceCfg_t *resourceCfg, bool isExpCfg)
{
    for (uint8_t rsrc = PMIC_PWR_RSRC_BUCK1; rsrc <= PMIC_PWR_RSRC_GPO; rsrc++)
    {
        switch (rsrc)
        {
            case PMIC_PWR_RSRC_BUCK1:
            case PMIC_PWR_RSRC_BUCK2:
            case PMIC_PWR_RSRC_BUCK3:
                powerTest_initBuckCfg(&resourceCfg[rsrc], rsrc, isExpCfg);
                break;
            case PMIC_PWR_RSRC_LDO_LS1_VMON1:
                powerTest_initLdoLs1VmonCfg(&resourceCfg[rsrc], rsrc, isExpCfg);
                break;
            case PMIC_PWR_RSRC_LS2_VMON2:
                powerTest_initLs2Vmon2Cfg(&resourceCfg[rsrc], rsrc, isExpCfg);
                break;

            case PMIC_PWR_RSRC_VCCA_VMON:
                powerTest_initVccaVmonCfg(&resourceCfg[rsrc], rsrc, isExpCfg);
                break;

            case PMIC_PWR_RSRC_GPO:
                powerTest_initGpoCfg(&resourceCfg[rsrc], rsrc, isExpCfg);
                break;
            default:
                PLATFORM_ASSERT(0);
                break;
        }
    }
}

static inline void powerTest_compareExpActBuckCfg(Pmic_PowerResourceCfg_t *expResourceCfg,
                                                  Pmic_PowerResourceCfg_t *actResourceCfg)
{
    PLATFORM_ASSERT(expResourceCfg->enable == actResourceCfg->enable);
    PLATFORM_ASSERT(expResourceCfg->mode == actResourceCfg->mode);
    PLATFORM_ASSERT(expResourceCfg->ilim == actResourceCfg->ilim);
    PLATFORM_ASSERT(expResourceCfg->voltage_mV == actResourceCfg->voltage_mV);
    PLATFORM_ASSERT(expResourceCfg->deglitch == actResourceCfg->deglitch);
    PLATFORM_ASSERT(expResourceCfg->uvThresh == actResourceCfg->uvThresh);
    PLATFORM_ASSERT(expResourceCfg->uvReaction == actResourceCfg->uvReaction);
    PLATFORM_ASSERT(expResourceCfg->ovThresh == actResourceCfg->ovThresh);
    PLATFORM_ASSERT(expResourceCfg->ovReaction == actResourceCfg->ovReaction);
    PLATFORM_ASSERT(expResourceCfg->rvReaction == actResourceCfg->rvReaction);
    PLATFORM_ASSERT(expResourceCfg->scReaction == actResourceCfg->scReaction);
}

static inline void powerTest_compareExpActLdoLsVmonCfg(Pmic_PowerResourceCfg_t *expResourceCfg,
                                                       Pmic_PowerResourceCfg_t *actResourceCfg)
{
    PLATFORM_ASSERT(expResourceCfg->enable == actResourceCfg->enable);
    PLATFORM_ASSERT(expResourceCfg->mode == actResourceCfg->mode);
    PLATFORM_ASSERT(expResourceCfg->voltage_mV == actResourceCfg->voltage_mV);
    PLATFORM_ASSERT(expResourceCfg->deglitch == actResourceCfg->deglitch);
    PLATFORM_ASSERT(expResourceCfg->uvThresh == actResourceCfg->uvThresh);
    PLATFORM_ASSERT(expResourceCfg->uvReaction == actResourceCfg->uvReaction);
    PLATFORM_ASSERT(expResourceCfg->ovThresh == actResourceCfg->ovThresh);
    PLATFORM_ASSERT(expResourceCfg->ovReaction == actResourceCfg->ovReaction);
    PLATFORM_ASSERT(expResourceCfg->rvReaction == actResourceCfg->rvReaction);
    PLATFORM_ASSERT(expResourceCfg->scReaction == actResourceCfg->scReaction);
}

static inline void powerTest_compareExpActVccaVmonCfg(Pmic_PowerResourceCfg_t *expResourceCfg,
                                                      Pmic_PowerResourceCfg_t *actResourceCfg)
{
    PLATFORM_ASSERT(expResourceCfg->enable == actResourceCfg->enable);
    PLATFORM_ASSERT(expResourceCfg->mode == actResourceCfg->mode);
    PLATFORM_ASSERT(expResourceCfg->voltage_mV == actResourceCfg->voltage_mV);
    PLATFORM_ASSERT(expResourceCfg->deglitch == actResourceCfg->deglitch);
    PLATFORM_ASSERT(expResourceCfg->uvThresh == actResourceCfg->uvThresh);
    PLATFORM_ASSERT(expResourceCfg->uvReaction == actResourceCfg->uvReaction);
    PLATFORM_ASSERT(expResourceCfg->ovThresh == actResourceCfg->ovThresh);
    PLATFORM_ASSERT(expResourceCfg->ovReaction == actResourceCfg->ovReaction);
}

static inline void powerTest_compareExpActGpoCfg(Pmic_PowerResourceCfg_t *expResourceCfg,
                                                 Pmic_PowerResourceCfg_t *actResourceCfg)
{
    PLATFORM_ASSERT(expResourceCfg->enable == actResourceCfg->enable);
}

static inline void powerTest_compareExpActResourceCfgs(Pmic_PowerResourceCfg_t *expResourceCfgs,
                                                       Pmic_PowerResourceCfg_t *actResourceCfgs)
{
    for (uint8_t rsrc = PMIC_PWR_RSRC_BUCK1; rsrc <= PMIC_PWR_RSRC_GPO; rsrc++)
    {
        switch (rsrc)
        {
            case PMIC_PWR_RSRC_BUCK1:
            case PMIC_PWR_RSRC_BUCK2:
            case PMIC_PWR_RSRC_BUCK3:
                powerTest_compareExpActBuckCfg(&expResourceCfgs[rsrc], &actResourceCfgs[rsrc]);
                break;
            case PMIC_PWR_RSRC_LDO_LS1_VMON1:
            case PMIC_PWR_RSRC_LS2_VMON2:
                powerTest_compareExpActLdoLsVmonCfg(&expResourceCfgs[rsrc], &actResourceCfgs[rsrc]);
                break;
            case PMIC_PWR_RSRC_VCCA_VMON:
                powerTest_compareExpActVccaVmonCfg(&expResourceCfgs[rsrc], &actResourceCfgs[rsrc]);
                break;
            case PMIC_PWR_RSRC_GPO:
                powerTest_compareExpActGpoCfg(&expResourceCfgs[rsrc], &actResourceCfgs[rsrc]);
                break;
            default:
                PLATFORM_ASSERT(0);
                break;
        }
    }
}

void test_pos_power_setGetResourceCfg_allRsrc_allCfg(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PowerResourceCfg_t expResourceCfgs[POWER_TEST_NUM_RESOURCES];
    Pmic_PowerResourceCfg_t actResourceCfgs[POWER_TEST_NUM_RESOURCES];

    powerTest_initAllRsrcCfg(expResourceCfgs, POWER_TEST_IS_EXPECTED_CFG);
    powerTest_initAllRsrcCfg(actResourceCfgs, POWER_TEST_IS_ACTUAL_CFG);

    // Set all resources to expected configuration
    status = Pmic_pwrSetResourceCfgs(&pmicHandle, POWER_TEST_NUM_RESOURCES, expResourceCfgs);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get actual configurations of all resources and compare expected vs. actual values
    status = Pmic_pwrGetResourceCfgs(&pmicHandle, POWER_TEST_NUM_RESOURCES, actResourceCfgs);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    powerTest_compareExpActResourceCfgs(expResourceCfgs, actResourceCfgs);
}

static void powerTest_setGetSequenceCfg_startUpDelay(uint8_t rsrc)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PowerSequenceCfg_t expSequenceCfg = {
        .validParams = PMIC_PWR_SEQ_STARTUP_VALID,
        .resource = rsrc
    };
    Pmic_PowerSequenceCfg_t actSequenceCfg = {
        .validParams = PMIC_PWR_SEQ_STARTUP_VALID,
        .resource = rsrc
    };

    // Pass all valid startup delays into Pmic_pwrSetSequenceCfg() for resource
    for (uint16_t startupDelay = PMIC_PWR_SEQ_DLY_MIN; startupDelay <= PMIC_PWR_SEQ_DLY_MAX; startupDelay++)
    {
        // Set resource startup delay
        expSequenceCfg.startupDelay = startupDelay;
        status = Pmic_pwrSetSequenceCfg(&pmicHandle, &expSequenceCfg);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        // Get actual resource startup delay and compare expected vs. actual values
        status = Pmic_pwrGetSequenceCfg(&pmicHandle, &actSequenceCfg);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        PLATFORM_ASSERT(startupDelay == actSequenceCfg.startupDelay);
    }
}

static void powerTest_setGetSequenceCfg_shutdownDelay(uint8_t rsrc)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PowerSequenceCfg_t expSequenceCfg = {
        .validParams = PMIC_PWR_SEQ_SHUTDOWN_VALID,
        .resource = rsrc
    };
    Pmic_PowerSequenceCfg_t actSequenceCfg = {
        .validParams = PMIC_PWR_SEQ_SHUTDOWN_VALID,
        .resource = rsrc
    };

    // Pass all valid shutdown delays into Pmic_pwrSetSequenceCfg() for resource
    for (uint16_t shutdownDelay = PMIC_PWR_SEQ_DLY_MIN; shutdownDelay <= PMIC_PWR_SEQ_DLY_MAX; shutdownDelay++)
    {
        // Set resource shutdown delay
        expSequenceCfg.shutdownDelay = shutdownDelay;
        status = Pmic_pwrSetSequenceCfg(&pmicHandle, &expSequenceCfg);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        // Get actual resource shutdown delay and compare expected vs. actual values
        status = Pmic_pwrGetSequenceCfg(&pmicHandle, &actSequenceCfg);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        PLATFORM_ASSERT(shutdownDelay == actSequenceCfg.shutdownDelay);
    }
}

void test_pos_power_setGetSequenceCfg_buck1_startupDelay(void)
{
    powerTest_setGetSequenceCfg_startUpDelay(PMIC_PWR_RSRC_BUCK1);
}

void test_pos_power_setGetSequenceCfg_buck1_shutdownDelay(void)
{
    powerTest_setGetSequenceCfg_shutdownDelay(PMIC_PWR_RSRC_BUCK1);
}

void test_pos_power_setGetSequenceCfg_buck2_startupDelay(void)
{
    powerTest_setGetSequenceCfg_startUpDelay(PMIC_PWR_RSRC_BUCK2);
}

void test_pos_power_setGetSequenceCfg_buck2_shutdownDelay(void)
{
    powerTest_setGetSequenceCfg_shutdownDelay(PMIC_PWR_RSRC_BUCK2);
}

void test_pos_power_setGetSequenceCfg_buck3_startupDelay(void)
{
    powerTest_setGetSequenceCfg_startUpDelay(PMIC_PWR_RSRC_BUCK3);
}

void test_pos_power_setGetSequenceCfg_buck3_shutdownDelay(void)
{
    powerTest_setGetSequenceCfg_shutdownDelay(PMIC_PWR_RSRC_BUCK3);
}

void test_pos_power_setGetSequenceCfg_ldoLs1Vmon1_startupDelay(void)
{
    powerTest_setGetSequenceCfg_startUpDelay(PMIC_PWR_RSRC_LDO_LS1_VMON1);
}

void test_pos_power_setGetSequenceCfg_ldoLs1Vmon1_shutdownDelay(void)
{
    powerTest_setGetSequenceCfg_shutdownDelay(PMIC_PWR_RSRC_LDO_LS1_VMON1);
}

void test_pos_power_setGetSequenceCfg_ls2Vmon2_startupDelay(void)
{
    powerTest_setGetSequenceCfg_startUpDelay(PMIC_PWR_RSRC_LS2_VMON2);
}

void test_pos_power_setGetSequenceCfg_ls2Vmon2_shutdownDelay(void)
{
    powerTest_setGetSequenceCfg_shutdownDelay(PMIC_PWR_RSRC_LS2_VMON2);
}

void test_pos_power_setGetSequenceCfg_gpo_startupDelay(void)
{
    powerTest_setGetSequenceCfg_startUpDelay(PMIC_PWR_RSRC_GPO);
}

void test_pos_power_setGetSequenceCfg_gpo_shutdownDelay(void)
{
    powerTest_setGetSequenceCfg_shutdownDelay(PMIC_PWR_RSRC_GPO);
}

static void powerTest_initAllSeqCfg(Pmic_PowerSequenceCfg_t *sequenceCfg, bool isExpCfg)
{
    uint8_t index = 0U;
    for (uint8_t rsrc = PMIC_PWR_RSRC_BUCK1; rsrc <= PMIC_PWR_RSRC_GPO; rsrc++)
    {
        // Skip VCCA_VMON
        if (rsrc == PMIC_PWR_RSRC_VCCA_VMON)
        {
            continue;
        }

        sequenceCfg[index].validParams = PMIC_PWR_SEQ_STARTUP_VALID |
                                         PMIC_PWR_SEQ_SHUTDOWN_VALID;
        sequenceCfg[index].resource = rsrc;

        if (isExpCfg)
        {
            sequenceCfg[index].startupDelay = PMIC_PWR_SEQ_DLY_MIN + 1U;
            sequenceCfg[index].shutdownDelay = PMIC_PWR_SEQ_DLY_MIN + 1U;
        }
        else
        {
            sequenceCfg[index].startupDelay = 0U;
            sequenceCfg[index].shutdownDelay = 0U;
        }

        index++;
    }
}

static void powerTest_compareExpActSeqCfgs(Pmic_PowerSequenceCfg_t *expSequenceCfgs,
                                           Pmic_PowerSequenceCfg_t *actSequenceCfgs)
{
    for (uint8_t rsrc = PMIC_PWR_RSRC_BUCK1; rsrc < POWER_TEST_NUM_RESOURCES_NO_VCCA; rsrc++)
    {
        PLATFORM_ASSERT(expSequenceCfgs[rsrc].startupDelay == actSequenceCfgs[rsrc].startupDelay);
        PLATFORM_ASSERT(expSequenceCfgs[rsrc].shutdownDelay == actSequenceCfgs[rsrc].shutdownDelay);
    }
}

void test_pos_power_setGetSequenceCfg_allRsrc_allCfg(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PowerSequenceCfg_t expSequenceCfgs[POWER_TEST_NUM_RESOURCES_NO_VCCA];
    Pmic_PowerSequenceCfg_t actSequenceCfgs[POWER_TEST_NUM_RESOURCES_NO_VCCA];

    powerTest_initAllSeqCfg(expSequenceCfgs, POWER_TEST_IS_EXPECTED_CFG);
    powerTest_initAllSeqCfg(actSequenceCfgs, POWER_TEST_IS_ACTUAL_CFG);

    // Set all resources to expected configuration
    status = Pmic_pwrSetSequenceCfgs(&pmicHandle, POWER_TEST_NUM_RESOURCES_NO_VCCA, expSequenceCfgs);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get actual configurations of all resources and compare expected vs. actual values
    status = Pmic_pwrGetSequenceCfgs(&pmicHandle, POWER_TEST_NUM_RESOURCES_NO_VCCA, actSequenceCfgs);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    powerTest_compareExpActSeqCfgs(expSequenceCfgs, actSequenceCfgs);
}


/* ========================================================================== */
/*              GPO Resource GET Unsupported Tests                            */
/* ========================================================================== */

void test_neg_power_pwrGetResourceCfg_gpo_unsupportedIlim(void)
{
    Pmic_PowerResourceCfg_t cfg = {
        .validParams = PMIC_PWR_CFG_ILIM_VALID,
        .resource = PMIC_PWR_RSRC_GPO
    };
    int32_t status = Pmic_pwrGetResourceCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NOT_SUPPORTED);
}

void test_neg_power_pwrGetResourceCfg_gpo_unsupportedDeglitch(void)
{
    Pmic_PowerResourceCfg_t cfg = {
        .validParams = PMIC_PWR_CFG_DEGLITCH_VALID,
        .resource = PMIC_PWR_RSRC_GPO
    };
    int32_t status = Pmic_pwrGetResourceCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NOT_SUPPORTED);
}

void test_neg_power_pwrGetResourceCfg_gpo_unsupportedUvThresh(void)
{
    Pmic_PowerResourceCfg_t cfg = {
        .validParams = PMIC_PWR_CFG_UV_THRESH_VALID,
        .resource = PMIC_PWR_RSRC_GPO
    };
    int32_t status = Pmic_pwrGetResourceCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NOT_SUPPORTED);
}

void test_neg_power_pwrGetResourceCfg_gpo_unsupportedUvReaction(void)
{
    Pmic_PowerResourceCfg_t cfg = {
        .validParams = PMIC_PWR_CFG_UV_REACT_VALID,
        .resource = PMIC_PWR_RSRC_GPO
    };
    int32_t status = Pmic_pwrGetResourceCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NOT_SUPPORTED);
}

void test_neg_power_pwrGetResourceCfg_gpo_unsupportedOvThresh(void)
{
    Pmic_PowerResourceCfg_t cfg = {
        .validParams = PMIC_PWR_CFG_OV_THRESH_VALID,
        .resource = PMIC_PWR_RSRC_GPO
    };
    int32_t status = Pmic_pwrGetResourceCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NOT_SUPPORTED);
}

void test_neg_power_pwrGetResourceCfg_gpo_unsupportedOvReaction(void)
{
    Pmic_PowerResourceCfg_t cfg = {
        .validParams = PMIC_PWR_CFG_OV_REACT_VALID,
        .resource = PMIC_PWR_RSRC_GPO
    };
    int32_t status = Pmic_pwrGetResourceCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NOT_SUPPORTED);
}

void test_neg_power_pwrGetResourceCfg_gpo_unsupportedRvReaction(void)
{
    Pmic_PowerResourceCfg_t cfg = {
        .validParams = PMIC_PWR_CFG_RV_REACT_VALID,
        .resource = PMIC_PWR_RSRC_GPO
    };
    int32_t status = Pmic_pwrGetResourceCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NOT_SUPPORTED);
}

void test_neg_power_pwrGetResourceCfg_gpo_unsupportedScReaction(void)
{
    Pmic_PowerResourceCfg_t cfg = {
        .validParams = PMIC_PWR_CFG_SC_REACT_VALID,
        .resource = PMIC_PWR_RSRC_GPO
    };
    int32_t status = Pmic_pwrGetResourceCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NOT_SUPPORTED);
}


/* ========================================================================== */
/*              VCCA_VMON Resource GET Unsupported Tests                      */
/* ========================================================================== */

void test_neg_power_pwrGetResourceCfg_vccaVmon_unsupportedIlim(void)
{
    Pmic_PowerResourceCfg_t cfg = {
        .validParams = PMIC_PWR_CFG_ILIM_VALID,
        .resource = PMIC_PWR_RSRC_VCCA_VMON
    };
    int32_t status = Pmic_pwrGetResourceCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NOT_SUPPORTED);
}

void test_neg_power_pwrGetResourceCfg_vccaVmon_unsupportedRvReaction(void)
{
    Pmic_PowerResourceCfg_t cfg = {
        .validParams = PMIC_PWR_CFG_RV_REACT_VALID,
        .resource = PMIC_PWR_RSRC_VCCA_VMON
    };
    int32_t status = Pmic_pwrGetResourceCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NOT_SUPPORTED);
}

void test_neg_power_pwrGetResourceCfg_vccaVmon_unsupportedScReaction(void)
{
    Pmic_PowerResourceCfg_t cfg = {
        .validParams = PMIC_PWR_CFG_SC_REACT_VALID,
        .resource = PMIC_PWR_RSRC_VCCA_VMON
    };
    int32_t status = Pmic_pwrGetResourceCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NOT_SUPPORTED);
}

/* ========================================================================== */
/*              Sequence Config Edge Case Tests                               */
/* ========================================================================== */

void test_neg_power_pwrSetSequenceCfgs_numConfigs_zero(void)
{
    Pmic_PowerSequenceCfg_t cfg[1] = {{0}};
    int32_t status = Pmic_pwrSetSequenceCfgs(&pmicHandle, 0, cfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrGetSequenceCfgs_numConfigs_zero(void)
{
    Pmic_PowerSequenceCfg_t cfg[1] = {{0}};
    int32_t status = Pmic_pwrGetSequenceCfgs(&pmicHandle, 0, cfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/* ========================================================================== */
/*              NRSTOUT Sequence Tests (LP8772x-Q1 specific)                 */
/* ========================================================================== */

void test_pos_power_getNrstoutSequence(void)
{
    // Test get sequence for NRSTOUT resource to cover NRSTOUT_SEQUENCE_REG case
    Pmic_PowerSequenceCfg_t cfg = {
        .validParams = PMIC_PWR_SEQ_STARTUP_VALID | PMIC_PWR_SEQ_SHUTDOWN_VALID,
        .resource = PMIC_PWR_RSRC_NRSTOUT
    };
    int32_t status = Pmic_pwrGetSequenceCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_neg_power_setVoltage_invalidResource(void)
{
    // Test invalid resource ID for voltage setting
    Pmic_PowerResourceCfg_t cfg = {
        .validParams = PMIC_PWR_CFG_VOLTAGE_VALID,
        .resource = PMIC_PWR_RSRC_MAX + 1U,
        .voltage_mV = 1000U
    };
    int32_t status = Pmic_pwrSetResourceCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_setPgLevel_invalidResource(void)
{
    // Test invalid resource for power good level (voltage on VMON resources)
    Pmic_PowerResourceCfg_t cfg = {
        .validParams = PMIC_PWR_CFG_VOLTAGE_VALID,
        .resource = PMIC_PWR_RSRC_GPO,
        .voltage_mV = 1000U
    };
    int32_t status = Pmic_pwrSetResourceCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/* ========================================================================== */
/*              Additional Coverage Tests for Unsupported Regulators         */
/* ========================================================================== */

void test_neg_power_powerGetVoutCfg_unsupportedRegulator(void)
{
    // Test getting voltage configuration for unsupported regulator type
    // NRSTOUT doesn't support voltage configuration
    Pmic_PowerResourceCfg_t cfg = {
        .validParams = PMIC_PWR_CFG_VOLTAGE_VALID,
        .resource = PMIC_PWR_RSRC_NRSTOUT
    };
    int32_t status = Pmic_pwrGetResourceCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_powerSetVoutCfg_unsupportedRegulator(void)
{
    // Test setting voltage configuration for unsupported regulator type
    // NRSTOUT doesn't support voltage configuration
    Pmic_PowerResourceCfg_t cfg = {
        .validParams = PMIC_PWR_CFG_VOLTAGE_VALID,
        .resource = PMIC_PWR_RSRC_NRSTOUT,
        .voltage_mV = 1000U
    };
    int32_t status = Pmic_pwrSetResourceCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_pos_power_powerGetPgoodLevel_validBuck(void)
{
    // Test reading PGOOD level configuration for BUCK regulator
    // This tests the voltage read path for BUCK resources
    Pmic_PowerResourceCfg_t cfg = {
        .validParams = PMIC_PWR_CFG_VOLTAGE_VALID,
        .resource = PMIC_PWR_RSRC_BUCK1
    };
    int32_t status = Pmic_pwrGetResourceCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    // Voltage should be within valid BUCK range
    PLATFORM_ASSERT((cfg.voltage_mV >= 900U) && (cfg.voltage_mV <= 1900U));
}

void test_pos_power_powerSetPgoodLevel_validBuck(void)
{
    // Test setting PGOOD level configuration for BUCK regulator
    // Set a valid voltage within BUCK range
    Pmic_PowerResourceCfg_t cfg = {
        .validParams = PMIC_PWR_CFG_VOLTAGE_VALID,
        .resource = PMIC_PWR_RSRC_BUCK2,
        .voltage_mV = 1200U
    };
    int32_t status = Pmic_pwrSetResourceCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify the voltage was set correctly
    Pmic_PowerResourceCfg_t readCfg = {
        .validParams = PMIC_PWR_CFG_VOLTAGE_VALID,
        .resource = PMIC_PWR_RSRC_BUCK2
    };
    status = Pmic_pwrGetResourceCfg(&pmicHandle, &readCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(readCfg.voltage_mV == 1200U);
}

/* ========================================================================== */
/*          LP8772x-Q1 Tests for Uncovered Lines in pmic_power.c             */
/* ========================================================================== */

void test_neg_power_pwr_setResourceCfg_zeroNumConfigs(void)
{
    // Test coverage for lines 1526-1527: numConfigs == 0
    // Pmic_pwrSetResourceCfgs() should return error when numConfigs is 0

    Pmic_PowerResourceCfg_t resourceCfgs[1] = {{0}};
    uint8_t numConfigs = 0U;

    // Call with numConfigs = 0, should return PMIC_ST_ERR_INV_PARAM
    int32_t status = Pmic_pwrSetResourceCfgs(&pmicHandle, numConfigs, resourceCfgs);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwr_setResourceCfg_excessiveNumConfigs(void)
{
    // Test coverage for lines 1526-1527: numConfigs > PMIC_PWR_RSRC_MAX
    // Pmic_pwrSetResourceCfgs() should return error when numConfigs exceeds maximum

    Pmic_PowerResourceCfg_t resourceCfgs[PMIC_PWR_RSRC_MAX + 1U];
    uint8_t numConfigs = PMIC_PWR_RSRC_MAX + 1U;

    // Initialize configurations
    for (uint8_t i = 0U; i < numConfigs; i++)
    {
        resourceCfgs[i].resource = PMIC_PWR_RSRC_BUCK1;
        resourceCfgs[i].validParams = PMIC_PWR_CFG_VOLTAGE_VALID;
        resourceCfgs[i].voltage_mV = 1000U;
    }

    // Call with excessive numConfigs, should return PMIC_ST_ERR_INV_PARAM
    int32_t status = Pmic_pwrSetResourceCfgs(&pmicHandle, numConfigs, resourceCfgs);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwr_getResourceCfgs_zeroNumConfigs(void)
{
    // Test coverage for lines 1608-1609: numConfigs == 0
    // Pmic_pwrGetResourceCfgs() should return error when numConfigs is 0

    Pmic_PowerResourceCfg_t resourceCfgs[1] = {{0}};
    uint8_t numConfigs = 0U;

    // Call with numConfigs = 0, should return PMIC_ST_ERR_INV_PARAM
    int32_t status = Pmic_pwrGetResourceCfgs(&pmicHandle, numConfigs, resourceCfgs);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_pwrGetResourceCfg() with MODE valid for unsupported resource
 * Covers PWR_getModeCfg() default case (pmic_power.c:522-523)
 * NRSTOUT resource doesn't support mode configuration
 */
void test_neg_power_pwr_getModeCfg_unsupportedResource(void)
{
    Pmic_PowerResourceCfg_t cfg = {
        .validParams = PMIC_PWR_CFG_MODE_VALID,
        .resource = PMIC_PWR_RSRC_NRSTOUT
    };
    int32_t status = Pmic_pwrGetResourceCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_pwrSetResourceCfg() with ILIM valid and resource > MAX
 * Covers PWR_setIlimCfg() resource bounds check (pmic_power.c:537-539)
 */
void test_neg_power_pwr_setIlimCfg_resourceOutOfBounds(void)
{
    Pmic_PowerResourceCfg_t cfg = {
        .validParams = PMIC_PWR_CFG_ILIM_VALID,
        .resource = PMIC_PWR_RSRC_MAX + 1U,
        .ilim = 10U
    };
    int32_t status = Pmic_pwrSetResourceCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_pwrGetResourceCfg() with ILIM valid and resource > MAX
 * Covers PWR_getIlimCfg() resource bounds check (pmic_power.c:567-569)
 */
void test_neg_power_pwr_getIlimCfg_resourceOutOfBounds(void)
{
    Pmic_PowerResourceCfg_t cfg = {
        .validParams = PMIC_PWR_CFG_ILIM_VALID,
        .resource = PMIC_PWR_RSRC_MAX + 1U
    };
    int32_t status = Pmic_pwrGetResourceCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_pwrGetResourceCfg() with VOLTAGE valid and resource > MAX
 * Covers PWR_getVoltageCfg() resource bounds check (pmic_power.c:1402-1403)
 */
void test_neg_power_pwr_getVoltageCfg_resourceOutOfBounds(void)
{
    Pmic_PowerResourceCfg_t cfg = {
        .validParams = PMIC_PWR_CFG_VOLTAGE_VALID,
        .resource = PMIC_PWR_RSRC_MAX + 1U
    };
    int32_t status = Pmic_pwrGetResourceCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test PWR_getModeCfgLdoLs1Vmon1() with invalid mode bit combination
 * Covers lines 461-463 in pmic_power.c (invalid hardware state detection)
 *
 * The LDO_LS1_VMON1 resource has 4 valid mode combinations:
 * - (bypConfig=0, vmon1Sel=0, lswConfig=0) -> REG mode
 * - (bypConfig=1, vmon1Sel=0, lswConfig=0) -> BYP mode
 * - (bypConfig=1, vmon1Sel=0, lswConfig=1) -> LSW mode
 * - (bypConfig=1, vmon1Sel=1, lswConfig=1) -> VMON mode
 *
 * Any other combination is invalid. This test injects an invalid combination.
 */
void test_neg_power_pwr_invalidModeCombination_ldoLs1Vmon1(void)
{
    int32_t status;
    const uint16_t LDO_LS1_VMON1_PG_LEVEL_REG = 0x1DU;
    const uint16_t FUNC_CONF_REG = 0x48U;

    // Inject invalid combination: bypConfig=0, vmon1Sel=1, lswConfig=0
    // LDO_LS1_BYP_CONFIG is bit 0 of LDO_LS1_VMON1_PG_LEVEL_REG
    status = testInject_setRegister(LDO_LS1_VMON1_PG_LEVEL_REG, 0x00U);  /* bypConfig=0 */
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // LDO_LS1_VMON1_SEL is bit 3, LDO_LS1_LSW_CONFIG is bit 2 of FUNC_CONF_REG
    status = testInject_setRegister(FUNC_CONF_REG, (1U << 3U));  /* vmon1Sel=1, lswConfig=0 */
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Try to get resource config - should fail with invalid combination
    Pmic_PowerResourceCfg_t cfg = {
        .validParams = PMIC_PWR_CFG_MODE_VALID,
        .resource = PMIC_PWR_RSRC_LDO_LS1_VMON1
    };

    status = Pmic_pwrGetResourceCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_FAIL);
}
