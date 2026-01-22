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

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /*__POWER_TEST_H__*/