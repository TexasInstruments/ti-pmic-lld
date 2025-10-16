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
#ifndef __POWER_TEST_H__
#define __POWER_TEST_H__

/**
 * @file power_test.h
 * @brief Contains macros/defines and test declarations specific to testing the
 * Power module.
 */

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

void test_negative_Pmic_pwrSetResourceEnable_nullParam_handle(void);
void test_negative_Pmic_pwrSetResourceEnable_outOfBounds_resource(void);
void test_negative_Pmic_pwrGetResourceEnable_nullParam_handle(void);
void test_negative_Pmic_pwrGetResourceEnable_nullParam_isEnabled(void);
void test_negative_Pmic_pwrGetResourceEnable_outOfBounds_resource(void);
void test_negative_Pmic_pwrSetResourceCfg_nullParam_handle(void);
void test_negative_Pmic_pwrSetResourceCfg_nullParam_config(void);
void test_negative_Pmic_pwrSetResourceCfg_outOfBounds_resource(void);
void test_negative_Pmic_pwrSetResourceCfg_invalidParam_buck1_mode(void);
void test_negative_Pmic_pwrSetResourceCfg_outOfBounds_buck1_ilim(void);
void test_negative_Pmic_pwrSetResourceCfg_outOfBounds_buck1_voltage_mV(void);
void test_negative_Pmic_pwrSetResourceCfg_outOfBounds_buck1_deglitch(void);
void test_negative_Pmic_pwrSetResourceCfg_outOfBounds_buck1_uvThresh(void);
void test_negative_Pmic_pwrSetResourceCfg_outOfBounds_buck1_uvReaction(void);
void test_negative_Pmic_pwrSetResourceCfg_outOfBounds_buck1_ovThresh(void);
void test_negative_Pmic_pwrSetResourceCfg_outOfBounds_buck1_ovReaction(void);
void test_negative_Pmic_pwrSetResourceCfg_outOfBounds_buck1_rvReaction(void);
void test_negative_Pmic_pwrSetResourceCfg_outOfBounds_buck1_scReaction(void);
void test_negative_Pmic_pwrSetResourceCfg_invalidParam_buck2_mode(void);
void test_negative_Pmic_pwrSetResourceCfg_outOfBounds_buck2_ilim(void);
void test_negative_Pmic_pwrSetResourceCfg_outOfBounds_buck2_voltage_mV(void);
void test_negative_Pmic_pwrSetResourceCfg_outOfBounds_buck2_deglitch(void);
void test_negative_Pmic_pwrSetResourceCfg_outOfBounds_buck2_uvThresh(void);
void test_negative_Pmic_pwrSetResourceCfg_outOfBounds_buck2_uvReaction(void);
void test_negative_Pmic_pwrSetResourceCfg_outOfBounds_buck2_ovThresh(void);
void test_negative_Pmic_pwrSetResourceCfg_outOfBounds_buck2_ovReaction(void);
void test_negative_Pmic_pwrSetResourceCfg_outOfBounds_buck2_rvReaction(void);
void test_negative_Pmic_pwrSetResourceCfg_outOfBounds_buck2_scReaction(void);
void test_negative_Pmic_pwrSetResourceCfg_invalidParam_buck3_mode(void);
void test_negative_Pmic_pwrSetResourceCfg_outOfBounds_buck3_ilim(void);
void test_negative_Pmic_pwrSetResourceCfg_outOfBounds_buck3_voltage_mV(void);
void test_negative_Pmic_pwrSetResourceCfg_outOfBounds_buck3_deglitch(void);
void test_negative_Pmic_pwrSetResourceCfg_outOfBounds_buck3_uvThresh(void);
void test_negative_Pmic_pwrSetResourceCfg_outOfBounds_buck3_uvReaction(void);
void test_negative_Pmic_pwrSetResourceCfg_outOfBounds_buck3_ovThresh(void);
void test_negative_Pmic_pwrSetResourceCfg_outOfBounds_buck3_ovReaction(void);
void test_negative_Pmic_pwrSetResourceCfg_outOfBounds_buck3_rvReaction(void);
void test_negative_Pmic_pwrSetResourceCfg_outOfBounds_buck3_scReaction(void);
void test_negative_Pmic_pwrSetResourceCfg_outOfBounds_ldoLs1Vmon1_mode(void);
void test_negative_Pmic_pwrSetResourceCfg_ldoLs1Vmon1_ilim(void);
void test_negative_Pmic_pwrSetResourceCfg_outOfBounds_ldoLs1Vmon1_voltage_mV(void);
void test_negative_Pmic_pwrSetResourceCfg_outOfBounds_ldoLs1Vmon1_deglitch(void);
void test_negative_Pmic_pwrSetResourceCfg_outOfBounds_ldoLs1Vmon1_uvThresh(void);
void test_negative_Pmic_pwrSetResourceCfg_outOfBounds_ldoLs1Vmon1_uvReaction(void);
void test_negative_Pmic_pwrSetResourceCfg_outOfBounds_ldoLs1Vmon1_ovThresh(void);
void test_negative_Pmic_pwrSetResourceCfg_outOfBounds_ldoLs1Vmon1_ovReaction(void);
void test_negative_Pmic_pwrSetResourceCfg_outOfBounds_ldoLs1Vmon1_rvReaction(void);
void test_negative_Pmic_pwrSetResourceCfg_outOfBounds_ldoLs1Vmon1_scReaction(void);
void test_negative_Pmic_pwrSetResourceCfg_invalidParam_ls2Vmon2_mode(void);
void test_negative_Pmic_pwrSetResourceCfg_ls2Vmon2_ilim(void);
void test_negative_Pmic_pwrSetResourceCfg_outOfBounds_ls2Vmon2_voltage_mV(void);
void test_negative_Pmic_pwrSetResourceCfg_outOfBounds_ls2Vmon2_deglitch(void);
void test_negative_Pmic_pwrSetResourceCfg_outOfBounds_ls2Vmon2_uvThresh(void);
void test_negative_Pmic_pwrSetResourceCfg_outOfBounds_ls2Vmon2_uvReaction(void);
void test_negative_Pmic_pwrSetResourceCfg_outOfBounds_ls2Vmon2_ovThresh(void);
void test_negative_Pmic_pwrSetResourceCfg_outOfBounds_ls2Vmon2_ovReaction(void);
void test_negative_Pmic_pwrSetResourceCfg_outOfBounds_ls2Vmon2_rvReaction(void);
void test_negative_Pmic_pwrSetResourceCfg_outOfBounds_ls2Vmon2_scReaction(void);
void test_negative_Pmic_pwrSetResourceCfg_invalidParam_vccaVmon_mode(void);
void test_negative_Pmic_pwrSetResourceCfg_vccaVmon_ilim(void);
void test_negative_Pmic_pwrSetResourceCfg_outOfBounds_vccaVmon_voltage_mV(void);
void test_negative_Pmic_pwrSetResourceCfg_outOfBounds_vccaVmon_deglitch(void);
void test_negative_Pmic_pwrSetResourceCfg_outOfBounds_vccaVmon_uvThresh(void);
void test_negative_Pmic_pwrSetResourceCfg_outOfBounds_vccaVmon_uvReaction(void);
void test_negative_Pmic_pwrSetResourceCfg_outOfBounds_vccaVmon_ovThresh(void);
void test_negative_Pmic_pwrSetResourceCfg_outOfBounds_vccaVmon_ovReaction(void);
void test_negative_Pmic_pwrSetResourceCfg_vccaVmon_rvReaction(void);
void test_negative_Pmic_pwrSetResourceCfg_vccaVmon_scReaction(void);
void test_negative_Pmic_pwrSetResourceCfg_gpo_mode(void);
void test_negative_Pmic_pwrSetResourceCfg_gpo_ilim(void);
void test_negative_Pmic_pwrSetResourceCfg_gpo_voltage_mV(void);
void test_negative_Pmic_pwrSetResourceCfg_gpo_deglitch(void);
void test_negative_Pmic_pwrSetResourceCfg_gpo_uvThresh(void);
void test_negative_Pmic_pwrSetResourceCfg_gpo_uvReaction(void);
void test_negative_Pmic_pwrSetResourceCfg_gpo_ovThresh(void);
void test_negative_Pmic_pwrSetResourceCfg_gpo_ovReaction(void);
void test_negative_Pmic_pwrSetResourceCfg_gpo_rvReaction(void);
void test_negative_Pmic_pwrSetResourceCfg_gpo_scReaction(void);
void test_negative_Pmic_pwrGetResourceCfg_nullParam_handle(void);
void test_negative_Pmic_pwrGetResourceCfg_nullParam_config(void);
void test_negative_Pmic_pwrGetResourceCfg_outOfBounds_resource(void);
void test_negative_Pmic_pwrSetResourceCfgs_nullParam_handle(void);
void test_negative_Pmic_pwrSetResourceCfgs_nullParam_config(void);
void test_negative_Pmic_pwrSetResourceCfgs_outOfBounds_resource(void);
void test_negative_Pmic_pwrSetResourceCfgs_invalidParam_buck1_mode(void);
void test_negative_Pmic_pwrSetResourceCfgs_outOfBounds_buck1_ilim(void);
void test_negative_Pmic_pwrSetResourceCfgs_outOfBounds_buck1_voltage_mV(void);
void test_negative_Pmic_pwrSetResourceCfgs_outOfBounds_buck1_deglitch(void);
void test_negative_Pmic_pwrSetResourceCfgs_outOfBounds_buck1_uvThresh(void);
void test_negative_Pmic_pwrSetResourceCfgs_outOfBounds_buck1_uvReaction(void);
void test_negative_Pmic_pwrSetResourceCfgs_outOfBounds_buck1_ovThresh(void);
void test_negative_Pmic_pwrSetResourceCfgs_outOfBounds_buck1_ovReaction(void);
void test_negative_Pmic_pwrSetResourceCfgs_outOfBounds_buck1_rvReaction(void);
void test_negative_Pmic_pwrSetResourceCfgs_outOfBounds_buck1_scReaction(void);
void test_negative_Pmic_pwrSetResourceCfgs_invalidParam_buck2_mode(void);
void test_negative_Pmic_pwrSetResourceCfgs_outOfBounds_buck2_ilim(void);
void test_negative_Pmic_pwrSetResourceCfgs_outOfBounds_buck2_voltage_mV(void);
void test_negative_Pmic_pwrSetResourceCfgs_outOfBounds_buck2_deglitch(void);
void test_negative_Pmic_pwrSetResourceCfgs_outOfBounds_buck2_uvThresh(void);
void test_negative_Pmic_pwrSetResourceCfgs_outOfBounds_buck2_uvReaction(void);
void test_negative_Pmic_pwrSetResourceCfgs_outOfBounds_buck2_ovThresh(void);
void test_negative_Pmic_pwrSetResourceCfgs_outOfBounds_buck2_ovReaction(void);
void test_negative_Pmic_pwrSetResourceCfgs_outOfBounds_buck2_rvReaction(void);
void test_negative_Pmic_pwrSetResourceCfgs_outOfBounds_buck2_scReaction(void);
void test_negative_Pmic_pwrSetResourceCfgs_invalidParam_buck3_mode(void);
void test_negative_Pmic_pwrSetResourceCfgs_outOfBounds_buck3_ilim(void);
void test_negative_Pmic_pwrSetResourceCfgs_outOfBounds_buck3_voltage_mV(void);
void test_negative_Pmic_pwrSetResourceCfgs_outOfBounds_buck3_deglitch(void);
void test_negative_Pmic_pwrSetResourceCfgs_outOfBounds_buck3_uvThresh(void);
void test_negative_Pmic_pwrSetResourceCfgs_outOfBounds_buck3_uvReaction(void);
void test_negative_Pmic_pwrSetResourceCfgs_outOfBounds_buck3_ovThresh(void);
void test_negative_Pmic_pwrSetResourceCfgs_outOfBounds_buck3_ovReaction(void);
void test_negative_Pmic_pwrSetResourceCfgs_outOfBounds_buck3_rvReaction(void);
void test_negative_Pmic_pwrSetResourceCfgs_outOfBounds_buck3_scReaction(void);
void test_negative_Pmic_pwrSetResourceCfgs_outOfBounds_ldoLs1Vmon1_mode(void);
void test_negative_Pmic_pwrSetResourceCfgs_ldoLs1Vmon1_ilim(void);
void test_negative_Pmic_pwrSetResourceCfgs_outOfBounds_ldoLs1Vmon1_voltage_mV(void);
void test_negative_Pmic_pwrSetResourceCfgs_outOfBounds_ldoLs1Vmon1_deglitch(void);
void test_negative_Pmic_pwrSetResourceCfgs_outOfBounds_ldoLs1Vmon1_uvThresh(void);
void test_negative_Pmic_pwrSetResourceCfgs_outOfBounds_ldoLs1Vmon1_uvReaction(void);
void test_negative_Pmic_pwrSetResourceCfgs_outOfBounds_ldoLs1Vmon1_ovThresh(void);
void test_negative_Pmic_pwrSetResourceCfgs_outOfBounds_ldoLs1Vmon1_ovReaction(void);
void test_negative_Pmic_pwrSetResourceCfgs_outOfBounds_ldoLs1Vmon1_rvReaction(void);
void test_negative_Pmic_pwrSetResourceCfgs_outOfBounds_ldoLs1Vmon1_scReaction(void);
void test_negative_Pmic_pwrSetResourceCfgs_invalidParam_ls2Vmon2_mode(void);
void test_negative_Pmic_pwrSetResourceCfgs_ls2Vmon2_ilim(void);
void test_negative_Pmic_pwrSetResourceCfgs_outOfBounds_ls2Vmon2_voltage_mV(void);
void test_negative_Pmic_pwrSetResourceCfgs_outOfBounds_ls2Vmon2_deglitch(void);
void test_negative_Pmic_pwrSetResourceCfgs_outOfBounds_ls2Vmon2_uvThresh(void);
void test_negative_Pmic_pwrSetResourceCfgs_outOfBounds_ls2Vmon2_uvReaction(void);
void test_negative_Pmic_pwrSetResourceCfgs_outOfBounds_ls2Vmon2_ovThresh(void);
void test_negative_Pmic_pwrSetResourceCfgs_outOfBounds_ls2Vmon2_ovReaction(void);
void test_negative_Pmic_pwrSetResourceCfgs_outOfBounds_ls2Vmon2_rvReaction(void);
void test_negative_Pmic_pwrSetResourceCfgs_outOfBounds_ls2Vmon2_scReaction(void);
void test_negative_Pmic_pwrSetResourceCfgs_invalidParam_vccaVmon_mode(void);
void test_negative_Pmic_pwrSetResourceCfgs_vccaVmon_ilim(void);
void test_negative_Pmic_pwrSetResourceCfgs_outOfBounds_vccaVmon_voltage_mV(void);
void test_negative_Pmic_pwrSetResourceCfgs_outOfBounds_vccaVmon_deglitch(void);
void test_negative_Pmic_pwrSetResourceCfgs_outOfBounds_vccaVmon_uvThresh(void);
void test_negative_Pmic_pwrSetResourceCfgs_outOfBounds_vccaVmon_uvReaction(void);
void test_negative_Pmic_pwrSetResourceCfgs_outOfBounds_vccaVmon_ovThresh(void);
void test_negative_Pmic_pwrSetResourceCfgs_outOfBounds_vccaVmon_ovReaction(void);
void test_negative_Pmic_pwrSetResourceCfgs_vccaVmon_rvReaction(void);
void test_negative_Pmic_pwrSetResourceCfgs_vccaVmon_scReaction(void);
void test_negative_Pmic_pwrSetResourceCfgs_gpo_mode(void);
void test_negative_Pmic_pwrSetResourceCfgs_gpo_ilim(void);
void test_negative_Pmic_pwrSetResourceCfgs_gpo_voltage_mV(void);
void test_negative_Pmic_pwrSetResourceCfgs_gpo_deglitch(void);
void test_negative_Pmic_pwrSetResourceCfgs_gpo_uvThresh(void);
void test_negative_Pmic_pwrSetResourceCfgs_gpo_uvReaction(void);
void test_negative_Pmic_pwrSetResourceCfgs_gpo_ovThresh(void);
void test_negative_Pmic_pwrSetResourceCfgs_gpo_ovReaction(void);
void test_negative_Pmic_pwrSetResourceCfgs_gpo_rvReaction(void);
void test_negative_Pmic_pwrSetResourceCfgs_gpo_scReaction(void);
void test_negative_Pmic_pwrGetResourceCfgs_nullParam_handle(void);
void test_negative_Pmic_pwrGetResourceCfgs_nullParam_config(void);
void test_negative_Pmic_pwrGetResourceCfgs_outOfBounds_resource(void);
void test_negative_Pmic_pwrSetSequenceCfg_nullParam_handle(void);
void test_negative_Pmic_pwrSetSequenceCfg_nullParam_config(void);
void test_negative_Pmic_pwrSetSequenceCfg_outOfBounds_resource(void);
void test_negative_Pmic_pwrSetSequenceCfg_outOfBounds_buck1_startupDelay(void);
void test_negative_Pmic_pwrSetSequenceCfg_outOfBounds_buck1_shutdownDelay(void);
void test_negative_Pmic_pwrSetSequenceCfg_outOfBounds_buck2_startupDelay(void);
void test_negative_Pmic_pwrSetSequenceCfg_outOfBounds_buck2_shutdownDelay(void);
void test_negative_Pmic_pwrSetSequenceCfg_outOfBounds_buck3_startupDelay(void);
void test_negative_Pmic_pwrSetSequenceCfg_outOfBounds_buck3_shutdownDelay(void);
void test_negative_Pmic_pwrSetSequenceCfg_outOfBounds_ldoLs1Vmon1_startupDelay(void);
void test_negative_Pmic_pwrSetSequenceCfg_outOfBounds_ldoLs1Vmon1_shutdownDelay(void);
void test_negative_Pmic_pwrSetSequenceCfg_outOfBounds_ls2Vmon2_startupDelay(void);
void test_negative_Pmic_pwrSetSequenceCfg_outOfBounds_ls2Vmon2_shutdownDelay(void);
void test_negative_Pmic_pwrSetSequenceCfg_outOfBounds_gpo_startupDelay(void);
void test_negative_Pmic_pwrSetSequenceCfg_outOfBounds_gpo_shutdownDelay(void);
void test_negative_Pmic_pwrSetSequenceCfg_vccaVmon_startupDelay(void);
void test_negative_Pmic_pwrSetSequenceCfg_vccaVmon_shutdownDelay(void);
void test_negative_Pmic_pwrGetSequenceCfg_nullParam_handle(void);
void test_negative_Pmic_pwrGetSequenceCfg_nullParam_config(void);
void test_negative_Pmic_pwrSetSequenceCfgs_nullParam_handle(void);
void test_negative_Pmic_pwrSetSequenceCfgs_nullParam_config(void);
void test_negative_Pmic_pwrSetSequenceCfgs_outOfBounds_resource(void);
void test_negative_Pmic_pwrSetSequenceCfgs_outOfBounds_buck1_startupDelay(void);
void test_negative_Pmic_pwrSetSequenceCfgs_outOfBounds_buck1_shutdownDelay(void);
void test_negative_Pmic_pwrSetSequenceCfgs_outOfBounds_buck2_startupDelay(void);
void test_negative_Pmic_pwrSetSequenceCfgs_outOfBounds_buck2_shutdownDelay(void);
void test_negative_Pmic_pwrSetSequenceCfgs_outOfBounds_buck3_startupDelay(void);
void test_negative_Pmic_pwrSetSequenceCfgs_outOfBounds_buck3_shutdownDelay(void);
void test_negative_Pmic_pwrSetSequenceCfgs_outOfBounds_ldoLs1Vmon1_startupDelay(void);
void test_negative_Pmic_pwrSetSequenceCfgs_outOfBounds_ldoLs1Vmon1_shutdownDelay(void);
void test_negative_Pmic_pwrSetSequenceCfgs_outOfBounds_ls2Vmon2_startupDelay(void);
void test_negative_Pmic_pwrSetSequenceCfgs_outOfBounds_ls2Vmon2_shutdownDelay(void);
void test_negative_Pmic_pwrSetSequenceCfgs_outOfBounds_gpo_startupDelay(void);
void test_negative_Pmic_pwrSetSequenceCfgs_outOfBounds_gpo_shutdownDelay(void);
void test_negative_Pmic_pwrSetSequenceCfgs_vccaVmon_startupDelay(void);
void test_negative_Pmic_pwrSetSequenceCfgs_vccaVmon_shutdownDelay(void);
void test_negative_Pmic_pwrGetSequenceCfgs_nullParam_handle(void);
void test_negative_Pmic_pwrGetSequenceCfgs_nullParam_config(void);
void test_negative_Pmic_pwrGetSequenceCfgs_outOfBounds_resource(void);
void test_positive_enableDisable_buck1(void);
void test_positive_enableDisable_buck2(void);
void test_positive_enableDisable_buck3(void);
void test_positive_enableDisable_ldoLs1Vmon1(void);
void test_positive_enableDisable_ls2Vmon2(void);
void test_positive_enableDisable_vccaVmon(void);
void test_positive_enableDisable_gpo(void);
void test_positive_setGetResourceCfg_buck1_enable(void);
void test_positive_setGetResourceCfg_buck1_mode(void);
void test_positive_setGetResourceCfg_buck1_ilim(void);
void test_positive_setGetResourceCfg_buck1_voltage_mV(void);
void test_positive_setGetResourceCfg_buck1_deglitch(void);
void test_positive_setGetResourceCfg_buck1_uvThresh(void);
void test_positive_setGetResourceCfg_buck1_uvReaction(void);
void test_positive_setGetResourceCfg_buck1_ovThresh(void);
void test_positive_setGetResourceCfg_buck1_ovReaction(void);
void test_positive_setGetResourceCfg_buck1_rvReaction(void);
void test_positive_setGetResourceCfg_buck1_scReaction(void);
void test_positive_setGetResourceCfg_buck2_enable(void);
void test_positive_setGetResourceCfg_buck2_mode(void);
void test_positive_setGetResourceCfg_buck2_ilim(void);
void test_positive_setGetResourceCfg_buck2_voltage_mV(void);
void test_positive_setGetResourceCfg_buck2_deglitch(void);
void test_positive_setGetResourceCfg_buck2_uvThresh(void);
void test_positive_setGetResourceCfg_buck2_uvReaction(void);
void test_positive_setGetResourceCfg_buck2_ovThresh(void);
void test_positive_setGetResourceCfg_buck2_ovReaction(void);
void test_positive_setGetResourceCfg_buck2_rvReaction(void);
void test_positive_setGetResourceCfg_buck2_scReaction(void);
void test_positive_setGetResourceCfg_buck3_enable(void);
void test_positive_setGetResourceCfg_buck3_mode(void);
void test_positive_setGetResourceCfg_buck3_ilim(void);
void test_positive_setGetResourceCfg_buck3_voltage_mV(void);
void test_positive_setGetResourceCfg_buck3_deglitch(void);
void test_positive_setGetResourceCfg_buck3_uvThresh(void);
void test_positive_setGetResourceCfg_buck3_uvReaction(void);
void test_positive_setGetResourceCfg_buck3_ovThresh(void);
void test_positive_setGetResourceCfg_buck3_ovReaction(void);
void test_positive_setGetResourceCfg_buck3_rvReaction(void);
void test_positive_setGetResourceCfg_buck3_scReaction(void);
void test_positive_setGetResourceCfg_ldoLs1Vmon1_enable(void);
void test_positive_setGetResourceCfg_ldoLs1Vmon1_mode(void);
void test_positive_setGetResourceCfg_ldoLs1Vmon1_voltage_mV(void);
void test_positive_setGetResourceCfg_ldoLs1Vmon1_deglitch(void);
void test_positive_setGetResourceCfg_ldoLs1Vmon1_uvThresh(void);
void test_positive_setGetResourceCfg_ldoLs1Vmon1_uvReaction(void);
void test_positive_setGetResourceCfg_ldoLs1Vmon1_ovThresh(void);
void test_positive_setGetResourceCfg_ldoLs1Vmon1_ovReaction(void);
void test_positive_setGetResourceCfg_ldoLs1Vmon1_rvReaction(void);
void test_positive_setGetResourceCfg_ldoLs1Vmon1_scReaction(void);
void test_positive_setGetResourceCfg_ls2Vmon2_enable(void);
void test_positive_setGetResourceCfg_ls2Vmon2_mode(void);
void test_positive_setGetResourceCfg_ls2Vmon2_voltage_mV(void);
void test_positive_setGetResourceCfg_ls2Vmon2_deglitch(void);
void test_positive_setGetResourceCfg_ls2Vmon2_uvThresh(void);
void test_positive_setGetResourceCfg_ls2Vmon2_uvReaction(void);
void test_positive_setGetResourceCfg_ls2Vmon2_ovThresh(void);
void test_positive_setGetResourceCfg_ls2Vmon2_ovReaction(void);
void test_positive_setGetResourceCfg_ls2Vmon2_rvReaction(void);
void test_positive_setGetResourceCfg_ls2Vmon2_scReaction(void);
void test_positive_setGetResourceCfg_vccaVmon_enable(void);
void test_positive_setGetResourceCfg_vccaVmon_mode(void);
void test_positive_setGetResourceCfg_vccaVmon_voltage_mV(void);
void test_positive_setGetResourceCfg_vccaVmon_deglitch(void);
void test_positive_setGetResourceCfg_vccaVmon_uvThresh(void);
void test_positive_setGetResourceCfg_vccaVmon_uvReaction(void);
void test_positive_setGetResourceCfg_vccaVmon_ovThresh(void);
void test_positive_setGetResourceCfg_vccaVmon_ovReaction(void);
void test_positive_setGetResourceCfg_gpo_enable(void);
void test_positive_setGetResourceCfg_allRsrc_allCfg(void);
void test_positive_setGetSequenceCfg_buck1_startupDelay(void);
void test_positive_setGetSequenceCfg_buck1_shutdownDelay(void);
void test_positive_setGetSequenceCfg_buck2_startupDelay(void);
void test_positive_setGetSequenceCfg_buck2_shutdownDelay(void);
void test_positive_setGetSequenceCfg_buck3_startupDelay(void);
void test_positive_setGetSequenceCfg_buck3_shutdownDelay(void);
void test_positive_setGetSequenceCfg_ldoLs1Vmon1_startupDelay(void);
void test_positive_setGetSequenceCfg_ldoLs1Vmon1_shutdownDelay(void);
void test_positive_setGetSequenceCfg_ls2Vmon2_startupDelay(void);
void test_positive_setGetSequenceCfg_ls2Vmon2_shutdownDelay(void);
void test_positive_setGetSequenceCfg_gpo_startupDelay(void);
void test_positive_setGetSequenceCfg_gpo_shutdownDelay(void);
void test_positive_setGetSequenceCfg_allRsrc_allCfg(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /*__POWER_TEST_H__*/
