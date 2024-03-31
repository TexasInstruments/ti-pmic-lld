/******************************************************************************
 * Copyright (c) 2023-2024 Texas Instruments Incorporated - http://www.ti.com
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
 * \file power_test.h
 * \author John Bui (j-bui1@ti.com)
 * \brief Header file containing all definitions and test declarations for Burton Power tests
 * \version 1.0
 */
#ifndef POWER_TEST_H
#define POWER_TEST_H

#include "common_test.h"

#ifdef __cplusplus
extern "C"
{
#endif

/**
 *  \brief  tps6522xGetPwrResourceCfg: Test error handling for when PMIC handle is NULL
 */
void test_power_getConfiguration_pmicHandle_null(void);

/**
 *  \brief  tps6522xGetPwrResourceCfg: Test API error handling for when Power Resource CFG input parameter
 *                                     is NULL
 */
void test_power_getConfiguration_pwrRsrcCfg_null(void);

/**
 *  \brief  tps6522xGetPwrResourceCfg: Test API error handling for when there are no valid parameters
 *                                     within Power Resource CFG input param
 */
void test_power_getConfiguration_pwrRsrcCfg_noValidParam(void);

/**
 *  \brief  tps6522xGetPwrResourceCfg: Test API error handling for when there are no valid parameters
 *                                     within Buck Power Resource CFG
 */
void test_power_getConfiguration_buckCfg_noValidParam(void);

/**
 *  \brief  tps6522xGetPwrResourceCfg: Test API error handling for when there are no valid parameters
 *                                     within LDO Power Resource CFG
 */
void test_power_getConfiguration_ldoCfg_noValidParam(void);

/**
 *  \brief  tps6522xGetPwrResourceCfg: Test API error handling for when there are no valid parameters
 *                                     within VCCA_VMON/VMONx Power Resource CFG
 */
void test_power_getConfiguration_vccaVmonCfg_noValidParam(void);

/**
 *  \brief  tps6522xGetPwrResourceCfg: Test API response for when all its input parameters are valid
 *                                     (no null parameters, acceptable power resource CFG validParams,
 *                                     etc.)
 */
void test_power_getConfiguration_validParameters(void);

/**
 *  \brief  tps6522xSetPwrRsrcCfg: Test API error handling for when PMIC handle is NULL
 */
void test_power_setConfiguration_pmicHandle_null(void);

/**
 *  \brief  tps6522xSetPwrRsrcCfg: Test API error handling for when there are no valid parameters
 *                                 within Power Resource CFG input param
 */
void test_power_setConfiguration_pwrRsrcCfg_noValidParam(void);

/**
 *  \brief  tps6522xSetPwrRsrcCfg: Test API error handling for when there are no valid parameters
 *                                 within Buck Power Resource CFG
 */
void test_power_setConfiguration_buckCfg_noValidParam(void);

/**
 *  \brief  tps6522xSetPwrRsrcCfg: Test API error handling for when there are no valid parameters
 *                                 within LDO Power Resource CFG
 */
void test_power_setConfiguration_ldoCfg_noValidParam(void);

/**
 *  \brief  tps6522xSetPwrRsrcCfg: Test API error handling for when there are no valid parameters
 *                                 within VCCA_VMON/VMONx Power Resource CFG
 */
void test_power_setConfiguration_vccaVmonCfg_noValidParam(void);

/**
 *  \brief  tps6522xSetPwrRsrcCfg: Test whether API can enable/disable Buck Pull-down resistor
 */
void test_power_setConfiguration_buckPldnEnableDisable(void);

/**
 *  \brief  tps6522xSetPwrRsrcCfg: Test whether API can enable/disable Buck VMON
 */
void test_power_setConfiguration_buckVmonEnableDisable(void);

/**
 *  \brief  tps6522xSetPwrRsrcCfg: Test whether API can configure Buck to operate in AUTO mode
 *                                 or FPWM mode
 */
void test_power_setConfiguration_buckFPWM(void);

/**
 *  \brief  tps6522xSetPwrRsrcCfg: Test whether API can enable/disable Buck regulator
 */
void test_power_setConfiguration_buckEn(void);

/**
 *  \brief  tps6522xSetPwrRsrcCfg: Test whether API can set Buck slew rate
 */
void test_power_setConfiguration_buckSlewRate(void);

/**
 *  \brief  tps6522xSetPwrRsrcCfg: Test API error handling for when BUCK1, BUCK2, BUCK3,
 *                                 and BUCK4 VSET are below range
 */
void test_power_setConfiguration_buckVset_belowRange(void);

/**
 *  \brief  tps6522xSetPwrRsrcCfg: Test API error handling for when BUCK1, BUCK2, BUCK3, 
 *                                 and BUCK4 VSET are above range
 */
void test_power_setConfiguration_buckVset_aboveRange(void);

/**
 *  \brief  tps6522xSetPwrRsrcCfg: Test whether API can set BUCK1, BUCK2, BUCK3, and BUCK4 VSET
 */
void test_power_setConfiguration_buckVset(void);

/**
 *  \brief  tps6522xSetPwrRsrcCfg: Test whether API can set Buck VMON Threshold
 */
void test_power_setConfiguration_buckVmonThr(void);

/**
 *  \brief  tps6522xSetPwrRsrcCfg: Test whether API can configure Buck Rail Group Selection
 */
void test_power_setConfiguration_buckRailGrpSel(void);

/**
 *  \brief  tps6522xSetPwrRsrcCfg: Test whether API can enable/disable LDO discharge
 */
void test_power_setConfiguration_ldoDischargeEnableDisable(void);

/**
 *  \brief  tps6522xSetPwrRsrcCfg: Test whether API can enable/disable LDO VMON
 */
void test_power_setConfiguration_ldoVmonEnableDisable(void);

/**
 *  \brief  tps6522xSetPwrRsrcCfg: Test whether API can enable/disable LDO
 */
void test_power_setConfiguration_ldoEnableDisable(void);

/**
 *  \brief  tps6522xSetPwrRsrcCfg: Test whether API can configure LDO to operate in
 *                                 Bypass Mode or LDO Mode
 */
void test_power_setConfiguration_ldoBypassConfig(void);

/**
 *  \brief  tps6522xSetPwrRsrcCfg: Test API error handling for when LDO1, LDO2, 
 *                                 and LDO3 VSET are above range
 */
void test_power_setConfiguration_ldoVset_aboveRange(void);

/**
 *  \brief  tps6522xSetPwrRsrcCfg:  Test whether API can set LDO1, LDO2, and LDO3 VSET
 */
void test_power_setConfiguration_ldoVset(void);

/**
 *  \brief  tps6522xSetPwrRsrcCfg: Test whether API can set LDO VMON threshold
 */
void test_power_setConfiguration_ldoVmonThr(void);

/**
 *  \brief  tps6522xSetPwrRsrcCfg: Test whether API can configure LDO Rail Group Selection
 */
void test_power_setConfiguration_ldoRailGrpSel(void);

/**
 *  \brief  tps6522xSetPwrRsrcCfg: Test whether API can configure VMON Deglitch Selection
 */
void test_power_setConfiguration_vmonDeglitch(void);

/**
 *  \brief  tps6522xSetPwrRsrcCfg: Test whether API can enable/disable VMON1, VMON2, and VCCA_VMON
 */
void test_power_setConfiguration_VMON1_2_VCCA_VMON_EnableDisable(void);

/**
 *  \brief  tps6522xSetPwrRsrcCfg: Test whether API can set VCCA_VMON PG_SET
 */
void test_power_setConfiguration_vccaPgSet(void);

/**
 *  \brief  tps6522xSetPwrRsrcCfg: Test whether API can set VCCA_VMON Threshold
 */
void test_power_setConfiguration_vccaVmonThr(void);

/**
 *  \brief  tps6522xSetPwrRsrcCfg: Test whether API can configure VCCA_VMON Rail Group Selection
 */
void test_power_setConfiguration_vccaRailGrpSel(void);

/**
 *  \brief  tps6522xSetPwrRsrcCfg: Test API error handling for when VMON1 PG_SET is out of acceptable
 *                                 range
 */
void test_power_setConfiguration_vmon1PgSet_outOfRange(void);

/**
 *  \brief  tps6522xSetPwrRsrcCfg: Test API error handling for when VMON2 PG_SET is out of acceptable
 *                                 range
 */
void test_power_setConfiguration_vmon2PgSet_outOfRange(void);

/**
 *  \brief  tps6522xSetPwrRsrcCfg: Test whether API can set VMON1 and VMON2 PG_SET
 */
void test_power_setConfiguration_vmonPgSet(void);

/**
 *  \brief  tps6522xSetPwrRsrcCfg: Test whether API can set VMON1 and VMON2 Threshold
 */
void test_power_setConfiguration_vmon1_2_Thr(void);

/**
 *  \brief  tps6522xSetPwrRsrcCfg: Test whether API can configure VMON1 and VMON2 Rail Group Selection
 */
void test_power_setConfiguration_vmon1_2_RailGrpSel(void);

/**
 *  \brief  tps6522xGetPwrRsrcStat: Test API error handling for when parameters are NULL
 */
void test_power_getPwrRsrcStat_nullParam(void);

/**
 *  \brief  tps6522xGetPwrRsrcStat: Test whether API can read all Buck, LDO, and VCCA_VMON/VMONx status
 */
void test_power_getPwrRsrcStat_allPwrRsrc(void);

/**
 *  \brief   tps6522xGetPwrRsrcStat: Test whether API can detect a UVOV status on VMON1 and VMON2.
 *
 *  \note    This test assumes that the VMONs are not connected to any voltage sources
 */
void test_power_getPwrRsrcStat_vmon1_2_UVOVStatDetection(void);

/**
 *  \brief  tps6522xGetPwrThermalStat: Test API error handling for when parameters are NULL
 */
void test_power_getPwrThermalStat_nullParam(void);

/**
 *  \brief  tps6522xGetPwrThermalStat: Test API error handling for when there are no valid parameters
 */
void test_power_getPwrThermalStat_noValidParams(void);

/**
 *  \brief  tps6522xGetPwrThermalStat: Test whether API can get the thermal warning, orderly, and
 *                                     immediate thermal statuses
 */
void test_power_getPwrThermalStat_getAllStatus(void);

/**
 *  \brief  tps6522xGetThermalCfg: Test API error handling for when parameters are NULL
 */
void test_power_getThermalCfg_nullParam(void);

/**
 *  \brief  tps6522xGetThermalCfg: Test API error handling for when there are no valid parameters
 */
void test_power_getThermalCfg_noValidParams(void);

/**
 *  \brief  tps6522xSetThermalCfg: Test API error handling for when parameters are NULL
 */
void test_power_setThermalCfg_nullParam(void);

/**
 *  \brief  tps6522xSetThermalCfg: Test API error handling for when there are no valid parameters
 */
void test_power_setThermalCfg_noValidParams(void);

/**
 *  \brief  tps6522xSetThermalCfg: Test whether API can configure the orderly thermal shutdown level
 */
void test_power_setThermalCfg_TsdOrdLevel(void);

/**
 *  \brief  tps6522xSetThermalCfg: Test whether API can configure the thermal warning level
 */
void test_power_setThermalCfg_TwarnLevel(void);

#ifdef __cplusplus
}

#endif /* __cplusplus */

#endif /* POWER_TEST_H */
