/**
 * \file power_test.h
 * \author John Bui (j-bui1@ti.com)
 * \brief Header file containing all definitions and test declarations for Burton Power tests
 * \version 1.0
 * \date 2023-11-17
 *
 * \copyright Copyright (c) 2023
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
 *  \brief  tps6522xGetPwrResourceCfg: Test API error handing for when Power Resource CFG input parameter
 *                                     is NULL
 */
void test_power_getConfiguration_pwrRsrcCfg_null(void);

/**
 *  \brief  tps6522xGetPwrResourceCfg: Test API error handing for when there are no valid parameters
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
 *  \brief  tps6522xSetPwrRsrcCfg: Test API error handing for when there are no valid parameters
 *                                     within Power Resource CFG input param
 */
void test_power_setConfiguration_pwrRsrcCfg_noValidParam(void);

/**
 *  \brief  tps6522xSetPwrRsrcCfg: Test API error handling for when there are no valid parameters
 *                                     within Buck Power Resource CFG
 */
void test_power_setConfiguration_buckCfg_noValidParam(void);

/**
 *  \brief  tps6522xSetPwrRsrcCfg: Test API error handling for when there are no valid parameters
 *                                     within LDO Power Resource CFG
 */
void test_power_setConfiguration_ldoCfg_noValidParam(void);

/**
 *  \brief  tps6522xSetPwrRsrcCfg: Test API error handling for when there are no valid parameters
 *                                     within VCCA_VMON/VMONx Power Resource CFG
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
 *                                     or FPWM mode
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
 *  \brief  tps6522xSetPwrRsrcCfg: Test API error handing for when Buck voltage is below range
 */
void test_power_setConfiguration_buckVout_voltageBelowRange(void);

/**
 *  \brief  tps6522xSetPwrRsrcCfg: Test API error handing for when Buck voltage is above range
 */
void test_power_setConfiguration_buckVout_voltageAboveRange(void);

/**
 *  \brief  tps6522xSetPwrRsrcCfg: Test whether API can set Buck 1 output voltage
 */
void test_power_setConfiguration_buck1_buckVout(void);

/**
 *  \brief  tps6522xSetPwrRsrcCfg: Test whether API can set Buck 2, Buck 3, Buck 4 output voltage
 */
void test_power_setConfiguration_buck2_3_4_buckVout(void);

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
 *                                     Bypass Mode or LDO Mode
 */
void test_power_setConfiguration_ldoBypassConfig(void);

/**
 *  \brief  tps6522xSetPwrRsrcCfg: Test API error handing for when LDO voltage is below range
 */
void test_power_setConfiguration_ldo1Vout_voltageBelowRange(void);

/**
 *  \brief  tps6522xSetPwrRsrcCfg: Test API error handing for when LDO1 voltage is above range
 */
void test_power_setConfiguration_ldo1Vout_voltageAboveRange(void);

/**
 *  \brief  tps6522xSetPwrRsrcCfg: Test API error handing for when LDO2, LDO3 voltage is below range
 */
void test_power_setConfiguration_ldo2_3_Vout_voltageBelowRange(void);

/**
 *  \brief  tps6522xSetPwrRsrcCfg: Test API error handing for when LDO2, LDO3 voltage is above range
 */
void test_power_setConfiguration_ldo2_3_Vout_voltageAboveRange(void);

/**
 *  \brief  tps6522xSetPwrRsrcCfg:  Test whether API can set LDO1 output voltage
 */
void test_power_setConfiguration_ldo1_ldoVout(void);

/**
 *  \brief  tps6522xSetPwrRsrcCfg:  Test whether API can set LDO1 output voltage
 */
void test_power_setConfiguration_ldo2_3_ldoVout(void);

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
 *  \brief  tps6522xSetPwrRsrcCfg: Test whether API can set VCCA_VMON PG Level
 */
void test_power_setConfiguration_vccaPgLevel(void);

/**
 *  \brief  tps6522xSetPwrRsrcCfg: Test whether API can set VCCA_VMON Threshold
 */
void test_power_setConfiguration_vccaVmonThr(void);

/**
 *  \brief  tps6522xSetPwrRsrcCfg: Test whether API can configure VCCA_VMON Rail Group Selection
 */
void test_power_setConfiguration_vccaRailGrpSel(void);

/**
 *  \brief  tps6522xSetPwrRsrcCfg: Test API error handing for when VMON1 voltage is below range
 *                                     and above range
 */
void test_power_setConfiguration_vmon1PgSet_voltageOutOfRange(void);

/**
 *  \brief  tps6522xSetPwrRsrcCfg: Test API error handing for when VMON2 voltage is below range
 *                                     and above range
 */
void test_power_setConfiguration_vmon2PgSet_voltageOutOfRange(void);

/**
 *  \brief  tps6522xSetPwrRsrcCfg: Test whether API can set VMON1 PG Level
 */
void test_power_setConfiguration_vmon1PgSet(void);

/**
 *  \brief  tps6522xSetPwrRsrcCfg: Test whether API can set VMON2 PG Level
 */
void test_power_setConfiguration_vmon2PgSet(void);

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
