/******************************************************************************
 * Copyright (c) 2023 Texas Instruments Incorporated - http://www.ti.com
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
 *  \addtogroup DRV_PMIC_POWER_MODULE
 *
 *  @{
 */

/**
 * \file    pmic_power_tps6522x.h
 *
 * \brief   PMIC TPS6522x Burton PMIC Power Resources Driver API/interface file.
 */

#ifndef PMIC_POWER_TPS6522X_H_
#define PMIC_POWER_TPS6522X_H_
#include <stdint.h>

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#ifdef __cplusplus
extern "C"
{
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/**
 *  \anchor     Pmic_Tps6522xBurton_maxPwrRsrcNum
 *  \name       Maximum BUCKs, LDOs, and VMONs supported by TPS6522x Burton
 *
 *  @{
 */
#define PMIC_POWER_TPS6522X_MAX_BUCK_NUM                          (4U)
#define PMIC_POWER_TPS6522X_MAX_LDO_NUM                           (3U)
#define PMIC_POWER_TPS6522X_MAX_VOLTAGE_MONITOR_NUM               (3U)
/** @} */

/**
 *  \anchor     Pmic_Tps6522xBurton_buckPwrRsrcNum
 *  \name       BUCK power resources supported by TPS6522x Burton
 *
 *  @{
 */
#define PMIC_POWER_TPS6522X_REGULATOR_BUCK1                       (0U)
#define PMIC_POWER_TPS6522X_REGULATOR_BUCK2                       (1U)
#define PMIC_POWER_TPS6522X_REGULATOR_BUCK3                       (2U)
#define PMIC_POWER_TPS6522X_REGULATOR_BUCK4                       (3U)
/** @} */

/**
 *  \anchor     Pmic_Tps6522xBurton_ldoPwrRsrcNum
 *  \name       LDO power resources supported by TPS6522x Burton
 *
 *  @{
 */
#define PMIC_POWER_TPS6522X_REGULATOR_LDO1                        (0U)
#define PMIC_POWER_TPS6522X_REGULATOR_LDO2                        (1U)
#define PMIC_POWER_TPS6522X_REGULATOR_LDO3                        (2U)
/** @} */

/**
 *  \anchor     Pmic_Tps6522xBurton_vccaVmonPwrRsrcNum
 *  \name       voltage monitor power resources supported by TPS6522x Burton
 *
 *  @{
 */
#define PMIC_POWER_TPS6522X_VOLTAGE_MONITOR_VMON1                 (0U)
#define PMIC_POWER_TPS6522X_VOLTAGE_MONITOR_VMON2                 (1U)
#define PMIC_POWER_TPS6522X_VOLTAGE_MONITOR_VCCA_VMON             (2U)
/** @} */

/**
 *  \anchor     Pmic_Tps6522xBurton_voltageSteps
 *  \name       voltage steps for BUCKs, LDOs, VCCA_VMON/VMONs supported by TPS6522x Burton
 *
 *  @{
 */
#define PMIC_POWER_TPS6522X_VOLTAGE_STEP_0_MV                     (0U)
#define PMIC_POWER_TPS6522X_VOLTAGE_STEP_5_MV                     (5U)
#define PMIC_POWER_TPS6522X_VOLTAGE_STEP_10_MV                    (10U)
#define PMIC_POWER_TPS6522X_VOLTAGE_STEP_20_MV                    (20U)
#define PMIC_POWER_TPS6522X_VOLTAGE_STEP_25_MV                    (25U)
#define PMIC_POWER_TPS6522X_VOLTAGE_STEP_50_MV                    (50U)
/** @} */

/**
 *  \anchor     Pmic_Tps6522xBurton_buck1VoltageRangesMV
 *  \name       BUCK 1 Voltage Ranges in mV supported by TPS6522x Burton
 *
 *  \brief      Ranges supported by TPS6522x Burton:
 *              - Range 1: 500 mV to 580 mV in 20 mV steps
 *              - Range 2: 600 mV to 1095 mV in 5 mV steps
 *              - Range 3: 1100 mV to 1650 mV in 10 mV steps
 *              - Range 4: 1660 mV to 3300 mV in 20 mV steps
 *
 *  @{
 */
#define PMIC_POWER_TPS6522X_BUCK1_VOLTAGE_RANGE_1_MIN_VOLTAGE     (500U)
#define PMIC_POWER_TPS6522X_BUCK1_VOLTAGE_RANGE_1_MAX_VOLTAGE     (580U)
#define PMIC_POWER_TPS6522X_BUCK1_VOLTAGE_RANGE_2_MIN_VOLTAGE     (600U)
#define PMIC_POWER_TPS6522X_BUCK1_VOLTAGE_RANGE_2_MAX_VOLTAGE     (1095U)
#define PMIC_POWER_TPS6522X_BUCK1_VOLTAGE_RANGE_3_MIN_VOLTAGE     (1100U)
#define PMIC_POWER_TPS6522X_BUCK1_VOLTAGE_RANGE_3_MAX_VOLTAGE     (1650U)
#define PMIC_POWER_TPS6522X_BUCK1_VOLTAGE_RANGE_4_MIN_VOLTAGE     (1660U)
#define PMIC_POWER_TPS6522X_BUCK1_VOLTAGE_RANGE_4_MAX_VOLTAGE     (3300U)
/** @} */

/**
 *  \anchor     Pmic_Tps6522xBurton_buck1VoltageRangesVset
 *  \name       BUCK 1 voltage ranges in VSET supported by TPS6522x Burton
 *
 *  \brief      Ranges supported by TPS6522x Burton:
 *              - Range 1: 0xA to 0xE
 *              - Range 2: 0xF to 0x72
 *              - Range 3: 0x73 to 0xAA
 *              - Range 4: 0xAB to 0xFD
 *
 *  @{
 */
#define PMIC_POWER_TPS6522X_BUCK1_VOLTAGE_RANGE_1_MIN_VSET        (0xAU)
#define PMIC_POWER_TPS6522X_BUCK1_VOLTAGE_RANGE_1_MAX_VSET        (0xEU)
#define PMIC_POWER_TPS6522X_BUCK1_VOLTAGE_RANGE_2_MIN_VSET        (0xFU)
#define PMIC_POWER_TPS6522X_BUCK1_VOLTAGE_RANGE_2_MAX_VSET        (0x72U)
#define PMIC_POWER_TPS6522X_BUCK1_VOLTAGE_RANGE_3_MIN_VSET        (0x73U)
#define PMIC_POWER_TPS6522X_BUCK1_VOLTAGE_RANGE_3_MAX_VSET        (0xAAU)
#define PMIC_POWER_TPS6522X_BUCK1_VOLTAGE_RANGE_4_MIN_VSET        (0xABU)
#define PMIC_POWER_TPS6522X_BUCK1_VOLTAGE_RANGE_4_MAX_VSET        (0xFDU)
/** @} */

/**
 *  \anchor     Pmic_Tps6522xBurton_buck2_3_4_VoltageRangesMV
 *  \name       BUCK 2, 3, 4 voltage ranges in mV supported by TPS6522x Burton
 *
 *  \brief      Ranges supported by TPS6522x Burton:
 *              - Range 1: 500 mV to 1150 mV in 25 mV steps
 *              - Range 2: 1200 mV to 3300 mV in 50 mV steps
 *
 *  @{
 */
#define PMIC_POWER_TPS6522X_BUCK2_3_4_VOLTAGE_RANGE_1_MIN_VOLTAGE (500U)
#define PMIC_POWER_TPS6522X_BUCK2_3_4_VOLTAGE_RANGE_1_MAX_VOLTAGE (1150U)
#define PMIC_POWER_TPS6522X_BUCK2_3_4_VOLTAGE_RANGE_2_MIN_VOLTAGE (1200U)
#define PMIC_POWER_TPS6522X_BUCK2_3_4_VOLTAGE_RANGE_2_MAX_VOLTAGE (3300U)
/** @} */

/**
 *  \anchor     Pmic_Tps6522xBurton_buck2_3_4_VoltageRangesVset
 *  \name       BUCK 2, 3, 4 voltage ranges in VSET supported by TPS6522x Burton
 *
 *  \brief      Ranges supported by TPS6522x Burton:
 *              - Range 1: 0x0 to 0x1A
 *              - Range 2: 0x1B to 0x45
 *
 *  @{
 */
#define PMIC_POWER_TPS6522X_BUCK2_3_4_VOLTAGE_RANGE_1_MIN_VSET    (0x0U)
#define PMIC_POWER_TPS6522X_BUCK2_3_4_VOLTAGE_RANGE_1_MAX_VSET    (0x1AU)
#define PMIC_POWER_TPS6522X_BUCK2_3_4_VOLTAGE_RANGE_2_MIN_VSET    (0x1BU)
#define PMIC_POWER_TPS6522X_BUCK2_3_4_VOLTAGE_RANGE_2_MAX_VSET    (0x45U)
/** @} */

/**
 *  \anchor     Pmic_Tps6522xBurton_ldo1VoltageRangesMV
 *  \name       LDO 1 voltage ranges in mV supported by TPS6522x Burton
 *
 *  \brief      Ranges supported by TPS6522x Burton:
 *              - Range 1: 1200 mV to 1200 mV in 0 mV steps
 *              - Range 2: 1250 mV to 3250 mV in 50 mV steps
 *              - Range 3: 3300 mV to 3300 mV in 0 mV steps
 *
 *  @{
 */
#define PMIC_POWER_TPS6522X_LDO1_VOLTAGE_RANGE_1_MIN_VOLTAGE      (1200U)
#define PMIC_POWER_TPS6522X_LDO1_VOLTAGE_RANGE_1_MAX_VOLTAGE      (1200U)
#define PMIC_POWER_TPS6522X_LDO1_VOLTAGE_RANGE_2_MIN_VOLTAGE      (1250U)
#define PMIC_POWER_TPS6522X_LDO1_VOLTAGE_RANGE_2_MAX_VOLTAGE      (3250U)
#define PMIC_POWER_TPS6522X_LDO1_VOLTAGE_RANGE_3_MIN_VOLTAGE      (3300U)
#define PMIC_POWER_TPS6522X_LDO1_VOLTAGE_RANGE_3_MAX_VOLTAGE      (3300U)
/** @} */

/**
 *  \anchor     Pmic_Tps6522xBurton_ldo1VoltageRangesVset
 *  \name       LDO 1 voltage ranges in VSET supported by TPS6522x Burton
 *
 *  \brief      Ranges supported by TPS6522x Burton:
 *              - Range 1: 0x0 to 0xC
 *              - Range 2: 0xD to 0x35
 *              - Range 3: 0x36 to 0x3F
 *
 *  @{
 */
#define PMIC_POWER_TPS6522X_LDO1_VOLTAGE_RANGE_1_MIN_VSET         (0x0U)
#define PMIC_POWER_TPS6522X_LDO1_VOLTAGE_RANGE_1_MAX_VSET         (0xCU)
#define PMIC_POWER_TPS6522X_LDO1_VOLTAGE_RANGE_2_MIN_VSET         (0xDU)
#define PMIC_POWER_TPS6522X_LDO1_VOLTAGE_RANGE_2_MAX_VSET         (0x35U)
#define PMIC_POWER_TPS6522X_LDO1_VOLTAGE_RANGE_3_MIN_VSET         (0x36U)
#define PMIC_POWER_TPS6522X_LDO1_VOLTAGE_RANGE_3_MAX_VSET         (0x3FU)
/** @} */

/**
 *  \anchor     Pmic_Tps6522xBurton_ldo2_3_VoltageRangesMV
 *  \name       LDO 2, 3 voltage ranges in mV supported by TPS6522x Burton
 *
 *  \brief      Ranges supported by TPS6522x Burton:
 *              - Range 1: 600 mV to 3350 mV in 50 mV steps
 *              - Range 2: 3400 mV to 3400 mV in 0 mV steps
 *
 *  @{
 */
#define PMIC_POWER_TPS6522X_LDO2_3_VOLTAGE_RANGE_1_MIN_VOLTAGE    (600U)
#define PMIC_POWER_TPS6522X_LDO2_3_VOLTAGE_RANGE_1_MAX_VOLTAGE    (3350U)
#define PMIC_POWER_TPS6522X_LDO2_3_VOLTAGE_RANGE_2_MIN_VOLTAGE    (3400U)
#define PMIC_POWER_TPS6522X_LDO2_3_VOLTAGE_RANGE_2_MAX_VOLTAGE    (3400U)
/** @} */

/**
 *  \anchor     Pmic_Tps6522xBurton_ldo2_3_VoltageRangesVset
 *  \name       LDO 2, 3 volage ranges in VSET supported by TPS6522x Burton
 *
 *  \brief      Ranges supported by TPS6522x Burton:
 *              - Range 1: 0x0 to 0x37
 *              - Range 2: 0x38 to 0x3F
 *
 *  @{
 */
#define PMIC_POWER_TPS6522X_LDO2_3_VOLTAGE_RANGE_1_MIN_VSET       (0x0U)
#define PMIC_POWER_TPS6522X_LDO2_3_VOLTAGE_RANGE_1_MAX_VSET       (0x37U)
#define PMIC_POWER_TPS6522X_LDO2_3_VOLTAGE_RANGE_2_MIN_VSET       (0x38U)
#define PMIC_POWER_TPS6522X_LDO2_3_VOLTAGE_RANGE_2_MAX_VSET       (0x3FU)
/** @} */

/**
 * \anchor      Pmic_Tps6522xBurton_vmon1VoltageRangesMV
 * \name        VMON 1 voltage ranges in mV supported by TPS6522x Burton
 *
 * \brief       Ranges supported by TPS6522x Burton:
 *              - Range 1: 500 mV to 580 mV in 20 mV steps
 *              - Range 2: 600 mV to 1095 mV in 5 mV steps
 *              - Range 3: 1100 mV to 1650 mV in 10 mV steps
 *              - Range 4: 1660 mV to 3340 mV in 20 mV steps
 *
 *  @{
 */
#define PMIC_POWER_TPS6522X_VMON1_VOLTAGE_RANGE_1_MIN_VOLTAGE     (500U)
#define PMIC_POWER_TPS6522X_VMON1_VOLTAGE_RANGE_1_MAX_VOLTAGE     (580U)
#define PMIC_POWER_TPS6522X_VMON1_VOLTAGE_RANGE_2_MIN_VOLTAGE     (600U)
#define PMIC_POWER_TPS6522X_VMON1_VOLTAGE_RANGE_2_MAX_VOLTAGE     (1095U)
#define PMIC_POWER_TPS6522X_VMON1_VOLTAGE_RANGE_3_MIN_VOLTAGE     (1100U)
#define PMIC_POWER_TPS6522X_VMON1_VOLTAGE_RANGE_3_MAX_VOLTAGE     (1650U)
#define PMIC_POWER_TPS6522X_VMON1_VOLTAGE_RANGE_4_MIN_VOLTAGE     (1660U)
#define PMIC_POWER_TPS6522X_VMON1_VOLTAGE_RANGE_4_MAX_VOLTAGE     (3340U)
/** @} */

/**
 *  \anchor     Pmic_Tps6522xBurton_vmon1VoltageRangesVset
 *  \name       VMON 1 voltage ranges in VSET supported by TPS6522x Burton
 *
 *  \brief      Ranges supported by TPS6522x Burton:
 *              - Range 1: 0xA to 0xE
 *              - Range 2: 0xF to 0x72
 *              - Range 3: 0x73 to 0xAA
 *              - Range 4: 0xAB to 0xFD
 *
 *  @{
 */
#define PMIC_POWER_TPS6522X_VMON1_VOLTAGE_RANGE_1_MIN_VSET        (0xAU)
#define PMIC_POWER_TPS6522X_VMON1_VOLTAGE_RANGE_1_MAX_VSET        (0xEU)
#define PMIC_POWER_TPS6522X_VMON1_VOLTAGE_RANGE_2_MIN_VSET        (0xFU)
#define PMIC_POWER_TPS6522X_VMON1_VOLTAGE_RANGE_2_MAX_VSET        (0x72U)
#define PMIC_POWER_TPS6522X_VMON1_VOLTAGE_RANGE_3_MIN_VSET        (0x73U)
#define PMIC_POWER_TPS6522X_VMON1_VOLTAGE_RANGE_3_MAX_VSET        (0xAAU)
#define PMIC_POWER_TPS6522X_VMON1_VOLTAGE_RANGE_4_MIN_VSET        (0xABU)
#define PMIC_POWER_TPS6522X_VMON1_VOLTAGE_RANGE_4_MAX_VSET        (0xFFU)
/** @} */

/**
 *  \anchor     Pmic_Tps6522xBurton_vmon2VoltageRangesMV
 *  \name       VMON 2 voltage ranges in mV supported by TPS6522x Burton
 *
 *  \brief      Ranges supported by TPS6522x Burton:
 *              - Range 1: 500 mV to 1150 mV in 25 mV steps
 *              - Range 2: 1200 mV to 3300 mV in 50 mV steps
 *
 *  @{
 */
#define PMIC_POWER_TPS6522X_VMON2_VOLTAGE_RANGE_1_MIN_VOLTAGE     (500U)
#define PMIC_POWER_TPS6522X_VMON2_VOLTAGE_RANGE_1_MAX_VOLTAGE     (1150U)
#define PMIC_POWER_TPS6522X_VMON2_VOLTAGE_RANGE_2_MIN_VOLTAGE     (1200U)
#define PMIC_POWER_TPS6522X_VMON2_VOLTAGE_RANGE_2_MAX_VOLTAGE     (3300U)
/** @} */

/**
 *  \anchor     Pmic_Tps6522xBurton_vmon2VoltageRangesVset
 *  \name       VMON 2 voltage ranges in VSET supported by TPS6522x Burton
 *
 *  \brief      Ranges supported by TPS6522x Burton:
 *              - Range 1: 0x0 to 0x1A
 *              - Range 2: 0x1B to 0x45
 *
 *  @{
 */
#define PMIC_POWER_TPS6522X_VMON2_VOLTAGE_RANGE_1_MIN_VSET        (0x0U)
#define PMIC_POWER_TPS6522X_VMON2_VOLTAGE_RANGE_1_MAX_VSET        (0x1AU)
#define PMIC_POWER_TPS6522X_VMON2_VOLTAGE_RANGE_2_MIN_VSET        (0x1BU)
#define PMIC_POWER_TPS6522X_VMON2_VOLTAGE_RANGE_2_MAX_VSET        (0x45U)
/** @} */

/**
 *  \anchor     Pmic_Tps6522xBurton_PwrRsrcMaxMinVoltageMv
 *  \name       Power Resource Min and Max Voltage Levels (mV) for TPS6522x
 *
 *  \brief      BUCK, LDO, VMON max and min voltage levels in mV supported by TPS6522x Burton
 *
 *  @{
 */
#define PMIC_POWER_TPS6522X_BUCK1_VOLTAGE_MIN_500_MV              (500U)
#define PMIC_POWER_TPS6522X_BUCK1_VOLTAGE_MAX_3300_MV             (3300U)
#define PMIC_POWER_TPS6522X_BUCK2_3_4_VOLTAGE_MIN_500_MV          (500U)
#define PMIC_POWER_TPS6522X_BUCK2_3_4_VOLTAGE_MAX_3300_MV         (3300U)
#define PMIC_POWER_TPS6522X_LDO1_VOLTAGE_MIN_1200_MV              (1200U)
#define PMIC_POWER_TPS6522X_LDO1_VOLTAGE_MAX_3300_MV              (3300U)
#define PMIC_POWER_TPS6522X_LDO2_3_VOLTAGE_MIN_600_MV             (600U)
#define PMIC_POWER_TPS6522X_LDO2_3_VOLTAGE_MAX_3400_MV            (3400U)
#define PMIC_POWER_TPS6522X_VMON1_VOLTAGE_MIN_500_MV              (500U)
#define PMIC_POWER_TPS6522X_VMON1_VOLTAGE_MAX_3340_MV             (3340U)
#define PMIC_POWER_TPS6522X_VMON2_VOLTAGE_MIN_500_MV              (500U)
#define PMIC_POWER_TPS6522X_VMON2_VOLTAGE_MAX_3300_MV             (3300U)
/** @} */

/**
 *  \anchor     Pmic_Tps6522xBurton_buckCtrlRegShiftVal
 *  \name       BUCK_CTRL register shift values supported by TPS6522x Burton
 *
 *  @{
 */
#define PMIC_POWER_TPS6522X_BUCK_CTRL_PLDN_SHIFT                  (0x5U)
#define PMIC_POWER_TPS6522X_BUCK_CTRL_VMON_EN_SHIFT               (0x4U)
#define PMIC_POWER_TPS6522X_BUCK_CTRL_PWM_OPTION_SHIFT            (0x1U)
#define PMIC_POWER_TPS6522X_BUCK_CTRL_EN_SHIFT                    (0x0U)
/** @} */

/**
 *  \anchor     Pmic_Tps6522xBurton_buckCtrlRegMaskVal
 *  \name       BUCK_CTRL register mask values supported by TPS6522x Burton
 *
 *  @{
 */
#define PMIC_POWER_TPS6522X_BUCK_CTRL_PLDN_MASK                   (0x1U << PMIC_POWER_TPS6522X_BUCK_CTRL_PLDN_SHIFT)
#define PMIC_POWER_TPS6522X_BUCK_CTRL_VMON_EN_MASK                (0x1U << PMIC_POWER_TPS6522X_BUCK_CTRL_VMON_EN_SHIFT)
#define PMIC_POWER_TPS6522X_BUCK_CTRL_PWM_OPTION_MASK             (0x1U << PMIC_POWER_TPS6522X_BUCK_CTRL_PWM_OPTION_SHIFT)
#define PMIC_POWER_TPS6522X_BUCK_CTRL_EN_MASK                     (0x1U << PMIC_POWER_TPS6522X_BUCK_CTRL_EN_SHIFT)
/** @} */

/**
 *  \anchor     Pmic_Tps6522xBurton_buckConfRegShiftVal
 *  \name       BUCK_CONF register shift values supported by TPS6522x Burton
 */
#define PMIC_POWER_TPS6522X_BUCK_CONF_SLEW_RATE_SHIFT             (0x0U)

/**
 *  \anchor      Pmic_Tps6522xBurton_buckConfRegMaskVal
 *  \name        BUCK_CONF register mask values supported by TPS6522x Burton
 */
#define PMIC_POWER_TPS6522X_BUCK_CONF_SLEW_RATE_MASK              (0x3U << PMIC_POWER_TPS6522X_BUCK_CONF_SLEW_RATE_SHIFT)

/**
 *  \anchor     Pmic_Tps6522xBurton_buckPgWindowRegShiftVal
 *  \name       BUCK_PG_WINDOW register shift values supported by TPS6522x Burton
 */
#define PMIC_POWER_TPS6522X_BUCK_PG_WINDOW_VMON_THR_SHIFT         (0x0U)

/**
 *  \anchor     Pmic_Tps6522xBurton_buckPgWindowRegMaskVal
 *  \name       BUCK_PG_WINDOW register mask values supported by TPS6522x Burton
 */
#define PMIC_POWER_TPS6522X_BUCK_PG_WINDOW_VMON_THR_MASK          (0x3U << PMIC_POWER_TPS6522X_BUCK_PG_WINDOW_VMON_THR_SHIFT)

/**
 *  \anchor     Pmic_Tps6522xBurton_railSel1RegShiftVal
 *  \name       RAIL_SEL_1 register shift values supported by TPS6522x Burton
 *
 *  @{
 */
#define PMIC_POWER_TPS6522X_CFG_RAIL_SEL_1_BUCK4_GRP_SEL_SHIFT    (6U)
#define PMIC_POWER_TPS6522X_CFG_RAIL_SEL_1_BUCK3_GRP_SEL_SHIFT    (4U)
#define PMIC_POWER_TPS6522X_CFG_RAIL_SEL_1_BUCK2_GRP_SEL_SHIFT    (2U)
#define PMIC_POWER_TPS6522X_CFG_RAIL_SEL_1_BUCK1_GRP_SEL_SHIFT    (0U)
/** @} */

/**
 *  \anchor     Pmic_Tps6522xBurton_railSel1RegMaskVal
 *  \name       RAIL_SEL_1 register mask values supported by TPS6522x Burton
 *
 *  @{
 */
#define PMIC_POWER_TPS6522X_CFG_RAIL_SEL_1_BUCK4_GRP_SEL_MASK                                                          \
    (3U << PMIC_POWER_TPS6522X_CFG_RAIL_SEL_1_BUCK4_GRP_SEL_SHIFT)
#define PMIC_POWER_TPS6522X_CFG_RAIL_SEL_1_BUCK3_GRP_SEL_MASK                                                          \
    (3U << PMIC_POWER_TPS6522X_CFG_RAIL_SEL_1_BUCK3_GRP_SEL_SHIFT)
#define PMIC_POWER_TPS6522X_CFG_RAIL_SEL_1_BUCK2_GRP_SEL_MASK                                                          \
    (3U << PMIC_POWER_TPS6522X_CFG_RAIL_SEL_1_BUCK2_GRP_SEL_SHIFT)
#define PMIC_POWER_TPS6522X_CFG_RAIL_SEL_1_BUCK1_GRP_SEL_MASK                                                          \
    (3U << PMIC_POWER_TPS6522X_CFG_RAIL_SEL_1_BUCK1_GRP_SEL_SHIFT)
/** @} */

/**
 *  \anchor     Pmic_Tps6522xBurton_ldoCtrlRegShiftVal
 *  \name       LDO_CTRL register shift values supported by TPS6522x Burton
 *
 *  @{
 */
#define PMIC_POWER_TPS6522X_LDO_CTRL_DISCHARGE_EN_SHIFT      (0x5U)
#define PMIC_POWER_TPS6522X_LDO_CTRL_VMON_EN_SHIFT           (0x4U)
#define PMIC_POWER_TPS6522X_LDO_CTRL_EN_SHIFT                (0x0U)
/** @} */

/**
 *  \anchor     Pmic_Tps6522xBurton_ldoCtrlRegMaskVal
 *  \name       LDO_CTRL register mask values supported by TPS6522x Burton
 *
 *  @{
 */
#define PMIC_POWER_TPS6522X_LDO_CTRL_DISCHARGE_EN_MASK       (0x1U << PMIC_POWER_TPS6522X_LDO_CTRL_DISCHARGE_EN_SHIFT)
#define PMIC_POWER_TPS6522X_LDO_CTRL_VMON_EN_MASK            (0x1U << PMIC_POWER_TPS6522X_LDO_CTRL_VMON_EN_SHIFT)
#define PMIC_POWER_TPS6522X_LDO_CTRL_EN_MASK                 (0x1U << PMIC_POWER_TPS6522X_LDO_CTRL_EN_SHIFT)
/** @} */

/**
 *  \anchor     Pmic_Tps6522xBurton_ldoVoutRegShiftVal
 *  \name       LDO_VOUT register shift values supported by TPS6522x Burton
 *
 *  @{
 */
#define PMIC_POWER_TPS6522X_LDO_VOUT_LDO_MODE_SHIFT          (0x7U)
#define PMIC_POWER_TPS6522X_LDO_VOUT_LDO_VSET_SHIFT          (0x1U)
/** @} */

/**
 *  \anchor     Pmic_Tps6522xBurton_ldoVoutRegMaskVal
 *  \name       LDO_VOUT register mask values supported by TPS6522x Burton
 */
#define PMIC_POWER_TPS6522X_LDO_VOUT_LDO_MODE_MASK           (0x1U << PMIC_POWER_TPS6522X_LDO_VOUT_LDO_MODE_SHIFT)
#define PMIC_POWER_TPS6522X_LDO_VOUT_LDO_VSET_MASK           (0x3FU << PMIC_POWER_TPS6522X_LDO_VOUT_LDO_VSET_SHIFT)
/** @} */

/**
 *  \anchor     Pmic_Tps6522xBurton_ldoPgWindowRegShiftVal
 *  \name       LDO_PG_WINDOW register shift values supported by TPS6522x Burton
 */
#define PMIC_POWER_TPS6522X_LDO_PG_WINDOW_LDO_VMON_THR_SHIFT (0x0U)

/**
 *  \anchor     Pmic_Tps6522xBurton_ldoPgWindowRegMaskVal
 *  \name       LDO_PG_WINDOW register mask values supported by TPS6522x Burton
 */
#define PMIC_POWER_TPS6522X_LDO_PG_WINDOW_LDO_VMON_THR_MASK                                                            \
    (0x3U << PMIC_POWER_TPS6522X_LDO_PG_WINDOW_LDO_VMON_THR_SHIFT)

/**
 *  \anchor     Pmic_Tps6522xBurton_railSel2RegShiftVal
 *  \name       RAIL_SEL_2 register shift values supported by TPS6522x Burton
 *
 *  @{
 */
#define PMIC_POWER_TPS6522X_RAIL_SEL_2_LDO3_GRP_SEL_SHIFT     (0x6U)
#define PMIC_POWER_TPS6522X_RAIL_SEL_2_LDO2_GRP_SEL_SHIFT     (0x4U)
#define PMIC_POWER_TPS6522X_RAIL_SEL_2_LDO1_GRP_SEL_SHIFT     (0x2U)
/** @} */

/**
 *  \anchor     Pmic_Tps6522xBurton_railSel2RegMaskVal
 *  \name       RAIL_SEL_2 register mask values supported by TPS6522x Burton
 *
 *  @{
 */
#define PMIC_POWER_TPS6522X_RAIL_SEL_2_LDO3_GRP_SEL_MASK      (0x3U << PMIC_POWER_TPS6522X_RAIL_SEL_2_LDO3_GRP_SEL_SHIFT)
#define PMIC_POWER_TPS6522X_RAIL_SEL_2_LDO2_GRP_SEL_MASK      (0x3U << PMIC_POWER_TPS6522X_RAIL_SEL_2_LDO2_GRP_SEL_SHIFT)
#define PMIC_POWER_TPS6522X_RAIL_SEL_2_LDO1_GRP_SEL_MASK      (0x3U << PMIC_POWER_TPS6522X_RAIL_SEL_2_LDO1_GRP_SEL_SHIFT)
/** @} */

/**
 *  \anchor     Pmic_Tps6522xBurton_vccaVmonCtrlRegShiftVal
 *  \name       VCCA_VMON_CTRL register shift values supported by TPS6522x Burton
 *
 *  @{
 */
#define PMIC_POWER_TPS6522X_VCCA_VMON_CTRL_DEGLITCH_SEL_SHIFT (5U)
#define PMIC_POWER_TPS6522X_VCCA_VMON_CTRL_VMON2_EN_SHIFT     (3U)
#define PMIC_POWER_TPS6522X_VCCA_VMON_CTRL_VMON1_EN_SHIFT     (1U)
#define PMIC_POWER_TPS6522X_VCCA_VMON_CTRL_VCCA_VMON_EN_SHIFT (0U)
/** @} */

/**
 *  \anchor     Pmic_Tps6522xBurton_vccaVmonCtrlRegMaskVal
 *  \name       VCCA_VMON_CTRL register mask values supported by TPS6522x Burton
 *
 *  @{
 */
#define PMIC_POWER_TPS6522X_VCCA_VMON_CTRL_DEGLITCH_SEL_MASK                                                           \
    (7U << PMIC_POWER_TPS6522X_VCCA_VMON_CTRL_DEGLITCH_SEL_SHIFT)
#define PMIC_POWER_TPS6522X_VCCA_VMON_CTRL_VMON2_EN_MASK (1U << PMIC_POWER_TPS6522X_VCCA_VMON_CTRL_VMON2_EN_SHIFT)
#define PMIC_POWER_TPS6522X_VCCA_VMON_CTRL_VMON1_EN_MASK (1U << PMIC_POWER_TPS6522X_VCCA_VMON_CTRL_VMON1_EN_SHIFT)
#define PMIC_POWER_TPS6522X_VCCA_VMON_CTRL_VCCA_VMON_EN_MASK                                                           \
    (1U << PMIC_POWER_TPS6522X_VCCA_VMON_CTRL_VCCA_VMON_EN_SHIFT)
/** @} */

/**
 *  \anchor     Pmic_Tps6522xBurton_vccaPgWindowRegShiftVal
 *  \name       VCCA_PG_WINDOW register shift values supported by TPS6522x Burton
 *
 *  @{
 */
#define PMIC_POWER_TPS6522X_VCCA_PG_WINDOW_VCCA_PG_SET_SHIFT   (6U)
#define PMIC_POWER_TPS6522X_VCCA_PG_WINDOW_VCCA_VMON_THR_SHIFT (0U)
/** @} */

/**
 *  \anchor     Pmic_Tps6522xBurton_vccaPgWindowRegMaskVal
 *  \name       VCCA_PG_WINDOW register mask values supported by TPS6522x Burton
 *
 *  @{
 */
#define PMIC_POWER_TPS6522X_VCCA_PG_WINDOW_VCCA_PG_SET_MASK    (1U << PMIC_POWER_TPS6522X_VCCA_PG_WINDOW_VCCA_PG_SET_SHIFT)
#define PMIC_POWER_TPS6522X_VCCA_PG_WINDOW_VCCA_VMON_THR_MASK                                                          \
    (3U << PMIC_POWER_TPS6522X_VCCA_PG_WINDOW_VCCA_VMON_THR_SHIFT)
/** @} */

/**
 *  \anchor     Pmic_Tps6522xBurton_vmonPgWindowRegShiftVal
 *  \name       VMON_PG_WINDOW register shift values supported by TPS6522x Burton
 *
 *  @{
 */
#define PMIC_POWER_TPS6522X_VMON1_PG_WINDOW_VMON1_THR_SHIFT   (0U)
#define PMIC_POWER_TPS6522X_VMON2_PG_WINDOW_VMON2_THR_SHIFT   (0U)
/** @} */

/**
 *  \anchor     Pmic_Tps6522xBurton_vmonPgWindowRegMaskVal
 *  \name       VMON_PG_WINDOW register mask values supported by TPS6522x Burton
 *
 *  @{
 */
#define PMIC_POWER_TPS6522X_VMON1_PG_WINDOW_VMON1_THR_MASK    (3U << PMIC_POWER_TPS6522X_VMON1_PG_WINDOW_VMON1_THR_SHIFT)
#define PMIC_POWER_TPS6522X_VMON2_PG_WINDOW_VMON2_THR_MASK    (3U << PMIC_POWER_TPS6522X_VMON2_PG_WINDOW_VMON2_THR_SHIFT)
/** @} */

/**
 *  \anchor     Pmic_Tps6522xBurton_vmonPgLevelRegShiftVal
 *  \name       VMON_PG_LEVEL register shift values supported by TPS6522x Burton
 *
 *  @{
 */
#define PMIC_POWER_TPS6522X_VMON1_PG_LEVEL_VMON1_PG_SET_SHIFT (0U)
#define PMIC_POWER_TPS6522X_VMON2_PG_LEVEL_VMON2_PG_SET_SHIFT (0U)
/** @} */

/**
 *  \anchor     Pmic_Tps6522xBurton_vmonPgLevelRegMaskVal
 *  \name       VMON_PG_LEVEL register mask values supported by TPS6522x Burton
 *
 *  @{
 */
#define PMIC_POWER_TPS6522X_VMON1_PG_LEVEL_VMON1_PG_SET_MASK                                                           \
    (0xFFU << PMIC_POWER_TPS6522X_VMON1_PG_LEVEL_VMON1_PG_SET_SHIFT)
#define PMIC_POWER_TPS6522X_VMON2_PG_LEVEL_VMON2_PG_SET_MASK                                                           \
    (0x7FU << PMIC_POWER_TPS6522X_VMON2_PG_LEVEL_VMON2_PG_SET_SHIFT)
/** @} */

/**
 *  \anchor     Pmic_Tps6522xBurton_railSel3RegShiftVal
 *  \name       RAIL_SEL_3 register shift values supported by TPS6522x Burton
 *
 *  @{
 */
#define PMIC_POWER_TPS6522X_RAIL_SEL_3_VMON2_GRP_SEL_SHIFT      (6U)
#define PMIC_POWER_TPS6522X_RAIL_SEL_3_VMON1_GRP_SEL_SHIFT      (4U)
#define PMIC_POWER_TPS6522X_RAIL_SEL_3_VCCA_GRP_SEL_SHIFT       (2U)
/** @} */

/**
 *  \anchor     Pmic_Tps6522xBurton_railSel3RegMaskVal
 *  \name       RAIL_SEL_3 register mask values supported by TPS6522x Burton
 *
 *  @{
 */
#define PMIC_POWER_TPS6522X_RAIL_SEL_3_VMON2_GRP_SEL_MASK       (3U << PMIC_POWER_TPS6522X_RAIL_SEL_3_VMON2_GRP_SEL_SHIFT)
#define PMIC_POWER_TPS6522X_RAIL_SEL_3_VMON1_GRP_SEL_MASK       (3U << PMIC_POWER_TPS6522X_RAIL_SEL_3_VMON1_GRP_SEL_SHIFT)
#define PMIC_POWER_TPS6522X_RAIL_SEL_3_VCCA_GRP_SEL_MASK        (3U << PMIC_POWER_TPS6522X_RAIL_SEL_3_VCCA_GRP_SEL_SHIFT)
/** @} */

/**
 *  \anchor     Pmic_Tps6522xBurton_statBuckRegShiftVal
 *  \name       STAT_BUCK register shift values supported by TPS6522x Burton
 *
 *  @{
 */
#define PMIC_POWER_TPS6522X_STAT_BUCK_BUCK4_UVOV_STAT_SHIFT     (3U)
#define PMIC_POWER_TPS6522X_STAT_BUCK_BUCK3_UVOV_STAT_SHIFT     (2U)
#define PMIC_POWER_TPS6522X_STAT_BUCK_BUCK2_UVOV_STAT_SHIFT     (1U)
#define PMIC_POWER_TPS6522X_STAT_BUCK_BUCK1_UVOV_STAT_SHIFT     (0U)
/** @} */

/**
 *  \anchor     Pmic_Tps6522xBurton_statBuckRegMaskVal
 *  \name       STAT_BUCK register mask values supported by TPS6522x Burton
 *
 *  @{
 */
#define PMIC_POWER_TPS6522X_STAT_BUCK_BUCK4_UVOV_STAT_MASK      (1U << PMIC_POWER_TPS6522X_STAT_BUCK_BUCK4_UVOV_STAT_SHIFT)
#define PMIC_POWER_TPS6522X_STAT_BUCK_BUCK3_UVOV_STAT_MASK      (1U << PMIC_POWER_TPS6522X_STAT_BUCK_BUCK3_UVOV_STAT_SHIFT)
#define PMIC_POWER_TPS6522X_STAT_BUCK_BUCK2_UVOV_STAT_MASK      (1U << PMIC_POWER_TPS6522X_STAT_BUCK_BUCK2_UVOV_STAT_SHIFT)
#define PMIC_POWER_TPS6522X_STAT_BUCK_BUCK1_UVOV_STAT_MASK      (1U << PMIC_POWER_TPS6522X_STAT_BUCK_BUCK1_UVOV_STAT_SHIFT)
/** @} */

/**
 *  \anchor     Pmic_Tps6522xBurton_statLdoVmonRegShiftVal
 *  \name       STAT_LDO_VMON register shift values supported by TPS6522x Burton
 *
 *  @{
 */
#define PMIC_POWER_TPS6522X_STAT_LDO_VMON_VMON2_UVOV_STAT_SHIFT (6U)
#define PMIC_POWER_TPS6522X_STAT_LDO_VMON_VMON1_UVOV_STAT_SHIFT (5U)
#define PMIC_POWER_TPS6522X_STAT_LDO_VMON_VCCA_UVOV_STAT_SHIFT  (4U)
#define PMIC_POWER_TPS6522X_STAT_LDO_VMON_LDO3_UVOV_STAT_SHIFT  (2U)
#define PMIC_POWER_TPS6522X_STAT_LDO_VMON_LDO2_UVOV_STAT_SHIFT  (1U)
#define PMIC_POWER_TPS6522X_STAT_LDO_VMON_LDO1_UVOV_STAT_SHIFT  (0U)
/** @} */

/**
 *  \anchor     Pmic_Tps6522xBurton_statLdoVmonRegMaskVal
 *  \name       STAT_LDO_VMON register mask values supported by TPS6522x Burton
 *
 *  @{
 */
#define PMIC_POWER_TPS6522X_STAT_LDO_VMON_VMON2_UVOV_STAT_MASK                                                         \
    (1U << PMIC_POWER_TPS6522X_STAT_LDO_VMON_VMON2_UVOV_STAT_SHIFT)
#define PMIC_POWER_TPS6522X_STAT_LDO_VMON_VMON1_UVOV_STAT_MASK                                                         \
    (1U << PMIC_POWER_TPS6522X_STAT_LDO_VMON_VMON1_UVOV_STAT_SHIFT)
#define PMIC_POWER_TPS6522X_STAT_LDO_VMON_VCCA_UVOV_STAT_MASK                                                          \
    (1U << PMIC_POWER_TPS6522X_STAT_LDO_VMON_VCCA_UVOV_STAT_SHIFT)
#define PMIC_POWER_TPS6522X_STAT_LDO_VMON_LDO3_UVOV_STAT_MASK                                                          \
    (1U << PMIC_POWER_TPS6522X_STAT_LDO_VMON_LDO3_UVOV_STAT_SHIFT)
#define PMIC_POWER_TPS6522X_STAT_LDO_VMON_LDO2_UVOV_STAT_MASK                                                          \
    (1U << PMIC_POWER_TPS6522X_STAT_LDO_VMON_LDO2_UVOV_STAT_SHIFT)
#define PMIC_POWER_TPS6522X_STAT_LDO_VMON_LDO1_UVOV_STAT_MASK                                                          \
    (1U << PMIC_POWER_TPS6522X_STAT_LDO_VMON_LDO1_UVOV_STAT_SHIFT)
/** @} */

/**
 *  \anchor     Pmic_Tps6522xBurton_thermalStatShiftVal
 *  \name       Register shift values for thermal status supported by TPS6522x Burton
 *
 *  @{
 */
#define PMIC_POWER_TPS6522X_TWARN_STAT_SHIFT                   (3U)
#define PMIC_POWER_TPS6522X_TSD_ORD_STAT_SHIFT                 (0U)
#define PMIC_POWER_TPS6522X_TSD_IMM_STAT_SHIFT                 (0U)
/** @} */

/**
 *  \anchor     Pmic_Tps6522xBurton_thermalStatMaskVal
 *  \name       Register mask values for thermal status supported by TPS6522x Burton
 *
 *  @{
 */
#define PMIC_POWER_TPS6522X_TWARN_STAT_MASK                    (1U << PMIC_POWER_TPS6522X_TWARN_STAT_SHIFT)
#define PMIC_POWER_TPS6522X_TSD_ORD_STAT_MASK                  (1U << PMIC_POWER_TPS6522X_TSD_ORD_STAT_SHIFT)
#define PMIC_POWER_TPS6522X_TSD_IMM_STAT_MASK                  (1U << PMIC_POWER_TPS6522X_TSD_IMM_STAT_SHIFT)
/** @} */

/**
 *  \anchor     Pmic_Tps6522xBurton_thermalCfgShiftVal
 *  \name       Register shift values for thermal configuration supported by TPS6522x Burton
 *
 *  @{
 */
#define PMIC_POWER_TPS6522X_TSD_ORD_LEVEL_SHIFT                (1U)
#define PMIC_POWER_TPS6522X_TWARN_LEVEL_SHIFT                  (0U)
/** @} */

/**
 *  \anchor     Pmic_Tps6522xBurton_thermalCfgMaskVal
 *  \name       Register mask values for thermal configuration supported by TPS6522x Burton
 *
 *  @{
 */
#define PMIC_POWER_TPS6522X_TWARN_LEVEL_MASK                   (1U << PMIC_POWER_TPS6522X_TWARN_LEVEL_SHIFT)
#define PMIC_POWER_TPS6522X_TSD_ORD_LEVEL_MASK                 (1U << PMIC_POWER_TPS6522X_TSD_ORD_LEVEL_SHIFT)
/** @} */

/**
 *  \anchor     Pmic_Tps6522xBurton_thermalCfgValidParam
 *  \name       Valid parameters of Thermal Configuration struct for TPS6522x Burton
 *
 *  @{
 */
#define PMIC_POWER_TPS6522X_TWARN_LEVEL_VALID                  (0U)
#define PMIC_POWER_TPS6522X_TSD_ORD_LEVEL_VALID                (1U)
/** @} */

/**
 *  \anchor     Pmic_Tps6522xBurton_thermalCfgValidParamShift
 *  \name       Valid parameter shift values of Thermal Configuration struct for TPS6522x Burton
 *
 *  \brief      Application can use these shift values to configure validParams of Thermal Configuration
 *              struct.
 *
 *  @{
 */
#define PMIC_POWER_TPS6522X_TWARN_LEVEL_VALID_SHIFT            (1U << PMIC_POWER_TPS6522X_TWARN_LEVEL_VALID)
#define PMIC_POWER_TPS6522X_TSD_ORD_LEVEL_VALID_SHIFT          (1U << PMIC_POWER_TPS6522X_TSD_ORD_LEVEL_VALID)
/** @} */

/**
 *  \anchor     Pmic_Tps6522xBurton_buckPwrRsrcCfgValidParam
 *  \name       Valid parameters of BUCK Power Resource Configuration struct for TPS6522x Burton
 *
 *  @{
 */
#define PMIC_POWER_TPS6522X_CFG_BUCK_PLDN_VALID                (0U)
#define PMIC_POWER_TPS6522X_CFG_BUCK_VMON_EN_VALID             (1U)
#define PMIC_POWER_TPS6522X_CFG_BUCK_PWM_OPTION_VALID          (2U)
#define PMIC_POWER_TPS6522X_CFG_BUCK_EN_VALID                  (3U)
#define PMIC_POWER_TPS6522X_CFG_BUCK_SLEW_RATE_VALID           (4U)
#define PMIC_POWER_TPS6522X_CFG_BUCK_VOLTAGE_MV_VALID          (5U)
#define PMIC_POWER_TPS6522X_CFG_BUCK_VMON_THR_VALID            (6U)
#define PMIC_POWER_TPS6522X_CFG_BUCK_RAIL_GRP_SEL_VALID        (7U)
/** @} */

/**
 *  \anchor     Pmic_Tps6522xBurton_buckPwrRsrcCfgValidParamShift
 *  \name       Valid parameter shift values of BUCK Power Resource Configuration struct for TPS6522x Burton
 *
 *  \brief      Application can use these shift values to configure validParams of BUCK Power Resource Configuration
 *              struct.
 *
 *  @{
 */
#define PMIC_POWER_TPS6522X_CFG_BUCK_PLDN_VALID_SHIFT          (1U << PMIC_POWER_TPS6522X_CFG_BUCK_PLDN_VALID)
#define PMIC_POWER_TPS6522X_CFG_BUCK_VMON_EN_VALID_SHIFT       (1U << PMIC_POWER_TPS6522X_CFG_BUCK_VMON_EN_VALID)
#define PMIC_POWER_TPS6522X_CFG_BUCK_PWM_OPTION_VALID_SHIFT    (1U << PMIC_POWER_TPS6522X_CFG_BUCK_PWM_OPTION_VALID)
#define PMIC_POWER_TPS6522X_CFG_BUCK_EN_VALID_SHIFT            (1U << PMIC_POWER_TPS6522X_CFG_BUCK_EN_VALID)
#define PMIC_POWER_TPS6522X_CFG_BUCK_SLEW_RATE_VALID_SHIFT     (1U << PMIC_POWER_TPS6522X_CFG_BUCK_SLEW_RATE_VALID)
#define PMIC_POWER_TPS6522X_CFG_BUCK_VOLTAGE_MV_VALID_SHIFT    (1U << PMIC_POWER_TPS6522X_CFG_BUCK_VOLTAGE_MV_VALID)
#define PMIC_POWER_TPS6522X_CFG_BUCK_VMON_THR_VALID_SHIFT      (1U << PMIC_POWER_TPS6522X_CFG_BUCK_VMON_THR_VALID)
#define PMIC_POWER_TPS6522X_CFG_BUCK_RAIL_GRP_SEL_VALID_SHIFT  (1U << PMIC_POWER_TPS6522X_CFG_BUCK_RAIL_GRP_SEL_VALID)
/** @} */

/**
 *  \anchor     Pmic_Tps6522xBurton_ldoPwrRsrcCfgValidParam
 *  \name       Valid parameters of LDO Power Resource Configuration struct for TPS6522x Burton
 *
 *  @{
 */
#define PMIC_POWER_TPS6522X_CFG_LDO_DISCHARGE_EN_VALID         (0U)
#define PMIC_POWER_TPS6522X_CFG_LDO_VMON_EN_VALID              (1U)
#define PMIC_POWER_TPS6522X_CFG_LDO_EN_VALID                   (2U)
#define PMIC_POWER_TPS6522X_CFG_LDO_MODE_VALID                 (3U)
#define PMIC_POWER_TPS6522X_CFG_LDO_VOLTAGE_MV_VALID           (4U)
#define PMIC_POWER_TPS6522X_CFG_LDO_VMON_THR_VALID             (5U)
#define PMIC_POWER_TPS6522X_CFG_LDO_RAIL_GRP_SEL_VALID         (6U)
/** @} */

/**
 * \anchor  Pmic_Tps6522xBurton_ldoPwrRsrcCfgValidParamShift
 * \name    Valid parameter shift values of LDO Power Resource Configuration struct for TPS6522x Burton
 *
 * \brief   Application can use these shift values to configure validParams of LDO Power Resource Configuration struct.
 *
 *  @{
 */
#define PMIC_POWER_TPS6522X_CFG_LDO_DISCHARGE_EN_VALID_SHIFT   (1U << PMIC_POWER_TPS6522X_CFG_LDO_DISCHARGE_EN_VALID)
#define PMIC_POWER_TPS6522X_CFG_LDO_VMON_EN_VALID_SHIFT        (1U << PMIC_POWER_TPS6522X_CFG_LDO_VMON_EN_VALID)
#define PMIC_POWER_TPS6522X_CFG_LDO_EN_VALID_SHIFT             (1U << PMIC_POWER_TPS6522X_CFG_LDO_EN_VALID)
#define PMIC_POWER_TPS6522X_CFG_LDO_MODE_VALID_SHIFT           (1U << PMIC_POWER_TPS6522X_CFG_LDO_MODE_VALID)
#define PMIC_POWER_TPS6522X_CFG_LDO_VOLTAGE_MV_VALID_SHIFT     (1U << PMIC_POWER_TPS6522X_CFG_LDO_VOLTAGE_MV_VALID)
#define PMIC_POWER_TPS6522X_CFG_LDO_VMON_THR_VALID_SHIFT       (1U << PMIC_POWER_TPS6522X_CFG_LDO_VMON_THR_VALID)
#define PMIC_POWER_TPS6522X_CFG_LDO_RAIL_GRP_SEL_VALID_SHIFT   (1U << PMIC_POWER_TPS6522X_CFG_LDO_RAIL_GRP_SEL_VALID)
/** @} */

/**
 *  \anchor     Pmic_Tps6522xBurton_vccaVmonPwrRsrcCfgValidParam
 *  \name       Valid parameters of VCCA_VMON Power Resource Configuration struct for TPS6522x Burton
 *
 *  @{
 */
#define PMIC_POWER_TPS6522X_CFG_VMON_DEGLITCH_SEL_VALID        (0U)
#define PMIC_POWER_TPS6522X_CFG_VMON2_EN_VALID                 (1U)
#define PMIC_POWER_TPS6522X_CFG_VMON1_EN_VALID                 (2U)
#define PMIC_POWER_TPS6522X_CFG_VCCA_VMON_EN_VALID             (3U)
#define PMIC_POWER_TPS6522X_CFG_VCCA_PG_LEVEL_VALID            (4U)
#define PMIC_POWER_TPS6522X_CFG_VCCA_VMON_THR_VALID            (5U)
#define PMIC_POWER_TPS6522X_CFG_VCCA_RAIL_GRP_SEL_VALID        (6U)
#define PMIC_POWER_TPS6522X_CFG_VMON1_THR_VALID                (7U)
#define PMIC_POWER_TPS6522X_CFG_VMON1_PG_LEVEL_MV_VALID        (8U)
#define PMIC_POWER_TPS6522X_CFG_VMON1_RAIL_GRP_SEL_VALID       (9U)
#define PMIC_POWER_TPS6522X_CFG_VMON2_THR_VALID                (10U)
#define PMIC_POWER_TPS6522X_CFG_VMON2_PG_LEVEL_MV_VALID        (11U)
#define PMIC_POWER_TPS6522X_CFG_VMON2_RAIL_GRP_SEL_VALID       (12U)
/** @} */

/**
 *  \anchor     Pmic_Tps6522xBurton_vccaVmonPwrRsrcCfgValidParamShift
 *  \name       Valid parameter shift values of VCCA_VMON Power Resource Configuration struct for TPS6522x Burton
 *
 *  \brief      Application can use these shift values to configure validParams of VCCA_VMON Power Resource
 *              Configuration struct.
 *
 *  @{
 */
#define PMIC_POWER_TPS6522X_CFG_VMON_DEGLITCH_SEL_VALID_SHIFT  (1U << PMIC_POWER_TPS6522X_CFG_VMON_DEGLITCH_SEL_VALID)
#define PMIC_POWER_TPS6522X_CFG_VMON2_EN_VALID_SHIFT           (1U << PMIC_POWER_TPS6522X_CFG_VMON2_EN_VALID)
#define PMIC_POWER_TPS6522X_CFG_VMON1_EN_VALID_SHIFT           (1U << PMIC_POWER_TPS6522X_CFG_VMON1_EN_VALID)
#define PMIC_POWER_TPS6522X_CFG_VCCA_VMON_EN_VALID_SHIFT       (1U << PMIC_POWER_TPS6522X_CFG_VCCA_VMON_EN_VALID)
#define PMIC_POWER_TPS6522X_CFG_VCCA_PG_LEVEL_VALID_SHIFT      (1U << PMIC_POWER_TPS6522X_CFG_VCCA_PG_LEVEL_VALID)
#define PMIC_POWER_TPS6522X_CFG_VCCA_VMON_THR_VALID_SHIFT      (1U << PMIC_POWER_TPS6522X_CFG_VCCA_VMON_THR_VALID)
#define PMIC_POWER_TPS6522X_CFG_VCCA_RAIL_GRP_SEL_VALID_SHIFT  (1U << PMIC_POWER_TPS6522X_CFG_VCCA_RAIL_GRP_SEL_VALID)
#define PMIC_POWER_TPS6522X_CFG_VMON1_THR_VALID_SHIFT          (1U << PMIC_POWER_TPS6522X_CFG_VMON1_THR_VALID)
#define PMIC_POWER_TPS6522X_CFG_VMON1_PG_LEVEL_MV_VALID_SHIFT  (1U << PMIC_POWER_TPS6522X_CFG_VMON1_PG_LEVEL_MV_VALID)
#define PMIC_POWER_TPS6522X_CFG_VMON1_RAIL_GRP_SEL_VALID_SHIFT (1U << PMIC_POWER_TPS6522X_CFG_VMON1_RAIL_GRP_SEL_VALID)
#define PMIC_POWER_TPS6522X_CFG_VMON2_THR_VALID_SHIFT          (1U << PMIC_POWER_TPS6522X_CFG_VMON2_THR_VALID)
#define PMIC_POWER_TPS6522X_CFG_VMON2_PG_LEVEL_MV_VALID_SHIFT  (1U << PMIC_POWER_TPS6522X_CFG_VMON2_PG_LEVEL_MV_VALID)
#define PMIC_POWER_TPS6522X_CFG_VMON2_RAIL_GRP_SEL_VALID_SHIFT (1U << PMIC_POWER_TPS6522X_CFG_VMON2_RAIL_GRP_SEL_VALID)
/** @} */

/**
 *  \anchor     Pmic_Tps6522xBurton_pwrRsrcCfgValidParam
 *  \name       Valid parameters of TPS6522x Power Resource Configuration struct
 *
 *  @{
 */
#define PMIC_POWER_TPS6522X_CFG_BUCK1_VALID                    (0U)
#define PMIC_POWER_TPS6522X_CFG_BUCK2_VALID                    (1U)
#define PMIC_POWER_TPS6522X_CFG_BUCK3_VALID                    (2U)
#define PMIC_POWER_TPS6522X_CFG_BUCK4_VALID                    (3U)
#define PMIC_POWER_TPS6522X_CFG_LDO1_VALID                     (4U)
#define PMIC_POWER_TPS6522X_CFG_LDO2_VALID                     (5U)
#define PMIC_POWER_TPS6522X_CFG_LDO3_VALID                     (6U)
#define PMIC_POWER_TPS6522X_CFG_VMON1_VALID                    (7U)
#define PMIC_POWER_TPS6522X_CFG_VMON2_VALID                    (8U)
#define PMIC_POWER_TPS6522X_CFG_VCCA_VALID                     (9U)
/** @} */

/**
 *  \anchor     Pmic_Tps6522xBurton_pwrRsrcCfgValidParamShift
 *  \name       Valid parameter shift values of TPS6522x Power Resource Configuration struct
 *
 *  \brief      Application can use these shift values to configure validParams of TPS6522x Power Resource
 *              Configuration struct.
 *
 *  @{
 */
#define PMIC_POWER_TPS6522X_CFG_BUCK1_VALID_SHIFT              (1U << PMIC_POWER_TPS6522X_CFG_BUCK1_VALID)
#define PMIC_POWER_TPS6522X_CFG_BUCK2_VALID_SHIFT              (1U << PMIC_POWER_TPS6522X_CFG_BUCK2_VALID)
#define PMIC_POWER_TPS6522X_CFG_BUCK3_VALID_SHIFT              (1U << PMIC_POWER_TPS6522X_CFG_BUCK3_VALID)
#define PMIC_POWER_TPS6522X_CFG_BUCK4_VALID_SHIFT              (1U << PMIC_POWER_TPS6522X_CFG_BUCK4_VALID)
#define PMIC_POWER_TPS6522X_CFG_LDO1_VALID_SHIFT               (1U << PMIC_POWER_TPS6522X_CFG_LDO1_VALID)
#define PMIC_POWER_TPS6522X_CFG_LDO2_VALID_SHIFT               (1U << PMIC_POWER_TPS6522X_CFG_LDO2_VALID)
#define PMIC_POWER_TPS6522X_CFG_LDO3_VALID_SHIFT               (1U << PMIC_POWER_TPS6522X_CFG_LDO3_VALID)
#define PMIC_POWER_TPS6522X_CFG_VMON1_VALID_SHIFT              (1U << PMIC_POWER_TPS6522X_CFG_VMON1_VALID)
#define PMIC_POWER_TPS6522X_CFG_VMON2_VALID_SHIFT              (1U << PMIC_POWER_TPS6522X_CFG_VMON2_VALID)
#define PMIC_POWER_TPS6522X_CFG_VCCA_VALID_SHIFT               (1U << PMIC_POWER_TPS6522X_CFG_VCCA_VALID)
/** @} */

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/**
 *  \anchor     Pmic_Tps6522xBurton_pwrRsrcUVOVStatus
 *  \name       Power Resource UVOV Status Enumeration for TPS6522x Burton
 *
 *  \brief      Enumeration of all the possible UVOV statuses for power resources.
 *
 *  @{
 */
#define PMIC_POWER_TPS6522X_BUCK1_UVOV_STAT (0U)
#define PMIC_POWER_TPS6522X_BUCK2_UVOV_STAT (1U)
#define PMIC_POWER_TPS6522X_BUCK3_UVOV_STAT (2U)
#define PMIC_POWER_TPS6522X_BUCK4_UVOV_STAT (3U)
#define PMIC_POWER_TPS6522X_LDO1_UVOV_STAT  (4U)
#define PMIC_POWER_TPS6522X_LDO2_UVOV_STAT  (5U)
#define PMIC_POWER_TPS6522X_LDO3_UVOV_STAT  (6U)
#define PMIC_POWER_TPS6522X_VMON1_UVOV_STAT (7U)
#define PMIC_POWER_TPS6522X_VMON2_UVOV_STAT (8U)
#define PMIC_POWER_TPS6522X_VCCA_UVOV_STAT  (9U)
/** @} */

/**
 *  \anchor     Pmic_Tps6522xBurton_tsdOrdLvlBitField
 *  \name       Orderly Thermal Shutdown Level Bit Field Enumeration for TPS6522x Burton
 *
 *  \brief      Orderly TSD threshold configuration.
 *
 *  @{
 */
#define PMIC_POWER_TPS6522X_TSD_ORD_LVL_140C (0U)
#define PMIC_POWER_TPS6522X_TSD_ORD_LVL_145C (1U)
/** @} */

/**
 *  \anchor     Pmic_Tps6522xBurton_TwarnLvlBitField
 *  \name       Thermal Warning Level Bit Field Enumeration for TPS6522x Burton
 *
 *  \brief      Thermal warning threshold configuration.
 *
 *  @{
 */
#define PMIC_POWER_TPS6522X_TWARN_LVL_130C (0U)
#define PMIC_POWER_TPS6522X_TWARN_LVL_140C (1U)
/** @} */

/**
 *  \anchor     Pmic_Tps6522xBurton_buckPldnEnBitField
 *  \name       BUCK_PLDN Bit Field Enumeration for TPS6522x Burton
 *
 *  \brief      Enable/disable output pull-down resistor when BUCK is disabled.
 *
 *  @{
 */
#define PMIC_POWER_TPS6522X_BUCK_PLDN_DISABLE (0U)
#define PMIC_POWER_TPS6522X_BUCK_PLDN_ENABLE  (1U)
/** @} */

/**
 *  \anchor     Pmic_Tps6522xBurton_buckVmonEnBitField
 *  \name       BUCK_VMON_EN Bit Field Enumeration for TPS6522x Burton
 *
 *  \brief      Enable/disable BUCK OV and UV comparators.
 *
 *  @{
 */
#define PMIC_POWER_TPS6522X_BUCK_VMON_DISABLE (0U)
#define PMIC_POWER_TPS6522X_BUCK_VMON_ENABLE  (1U)
/** @} */

/**
 *  \anchor     Pmic_Tps6522xBurton_buckFPWMBitField
 *  \name       BUCK_FPWM Bit Field Enumeration for TPS6522x Burton
 *
 *  \brief      Choose between automatic transitions between PFM and PWM mode or forced PWM mode.
 *
 *  @{
 */
#define PMIC_POWER_TPS6522X_BUCK_PWM_AUTO   (0U)
#define PMIC_POWER_TPS6522X_BUCK_PWM_FORCED (1U)
/** @} */

/**
 * \anchor  Pmic_Tps6522xBurton_buckEnBitField
 * \name    BUCK_EN Bit Field Enumeration for TPS6522x Burton
 *
 * \brief   Enable/disable BUCK regulator.
 *
 *  @{
 */
#define PMIC_POWER_TPS6522X_BUCK_DISABLE (0U)
#define PMIC_POWER_TPS6522X_BUCK_ENABLE  (1U)
/** @} */

/**
 *  \anchor     Pmic_Tps6522xBurton_buckSlewRateBitField
 *  \name       BUCK_SLEW_RATE Bit Field Enumeration for TPS6522x Burton
 *
 *  \brief      BUCK ouput voltage slew rate (rising and falling edges).
 *
 *  @{
 */
#define PMIC_POWER_TPS6522X_BUCK_SLEW_RATE_10_MV_PER_US   (0U)
#define PMIC_POWER_TPS6522X_BUCK_SLEW_RATE_5_MV_PER_US    (1U)
#define PMIC_POWER_TPS6522X_BUCK_SLEW_RATE_2_5_MV_PER_US  (2U)
#define PMIC_POWER_TPS6522X_BUCK_SLEW_RATE_1_25_MV_PER_US (3U)
/** @} */

/**
 *  \anchor     Pmic_Tps6522xBurton_buckVmonThrBitField
 *  \name       BUCK_VMON_THR Bit Field Enumeration for TPS6522x Burton
 *
 *  \brief      BUCK powergood high/low threshold level.
 *
 *  @{
 */
#define PMIC_POWER_TPS6522X_BUCK_VMON_THR_3_PCT_OR_30_MV (0U)
#define PMIC_POWER_TPS6522X_BUCK_VMON_THR_4_PCT_OR_40_MV (1U)
#define PMIC_POWER_TPS6522X_BUCK_VMON_THR_6_PCT_OR_60_MV (2U)
#define PMIC_POWER_TPS6522X_BUCK_VMON_THR_8_PCT_OR_80_MV (3U)
/** @} */

/**
 *  \anchor     Pmic_Tps6522xBurton_buckRailSelBitField
 *  \name       BUCK_GRP_SEL Bit Field Enumeration for TPS6522x Burton
 *
 *  \brief      BUCK rail group selection.
 *
 *  @{
 */
#define PMIC_POWER_TPS6522X_BUCK_RAIL_SEL_NONE  (0U)
#define PMIC_POWER_TPS6522X_BUCK_RAIL_SEL_MCU   (1U)
#define PMIC_POWER_TPS6522X_BUCK_RAIL_SEL_SOC   (2U)
#define PMIC_POWER_TPS6522X_BUCK_RAIL_SEL_OTHER (3U)
/** @} */

/**
 *  \anchor     Pmic_Tps6522xBurton_ldoDischargeEnBitField
 *  \name       LDO_DISCHARGE_EN Bit Field Enumeration for TPS6522x Burton
 *
 *  \brief      LDO discharge enable setting.
 *
 *  @{
 */
#define PMIC_POWER_TPS6522X_LDO_DISCHARGE_DISABLE (0U)
#define PMIC_POWER_TPS6522X_LDO_DISCHARGE_ENABLE  (1U)
/** @} */

/**
 *  \anchor     Pmic_Tps6522xBurton_ldoVmonEnBitField
 *  \name       LDO_VMON_EN Bit Field Enumeration for TPS6522x Burton
 *
 *  \brief      LDO Enable/Disable OV and UV comparators.
 *
 *  @{
 */
#define PMIC_POWER_TPS6522X_LDO_VMON_DISABLE (0U)
#define PMIC_POWER_TPS6522X_LDO_VMON_ENABLE  (1U)
/** @} */

/**
 *  \anchor     Pmic_Tps6522xBurton_ldoEnBitField
 *  \name       LDO_EN Bit Field Enumeration for TPS6522x Burton
 *
 *  \brief      LDO Enable/disable regulator.
 *
 *  @{
 */
#define PMIC_POWER_TPS6522X_LDO_DISABLE (0U)
#define PMIC_POWER_TPS6522X_LDO_ENABLE  (1U)
/** @} */

/**
 *  \anchor     Pmic_Tps6522xBurton_bypConfigBitField
 *  \name       LDO_BYP_CONFIG Bit Field Enumeration for TPS6522x Burton
 *
 *  \brief      LDO modes of operation.
 *
 *  @{
 */
#define PMIC_POWER_TPS6522X_LDO_BYP_CONFIG_LDO_MODE    (0U)
#define PMIC_POWER_TPS6522X_LDO_BYP_CONFIG_BYPASS_MODE (1U)
/** @} */

/**
 *  \anchor     Pmic_Tps6522xBurton_ldoVmonThrBitField
 *  \name       LDO_VMON_THR Bit Field Enumeration for TPS6522x Burton
 *
 *  \brief      LDO powergood high/low threshold level for LDO.
 *
 *  @{
 */
#define PMIC_POWER_TPS6522X_LDO_VMON_THR_3_PCT (0U)
#define PMIC_POWER_TPS6522X_LDO_VMON_THR_4_PCT (1U)
#define PMIC_POWER_TPS6522X_LDO_VMON_THR_6_PCT (2U)
#define PMIC_POWER_TPS6522X_LDO_VMON_THR_8_PCT (3U)
/** @} */

/**
 *  \anchor     Pmic_Tps6522xBurton_ldoRailSelBitField
 *  \name       LDO_GRP_SEL Bit Field Enumeration for TPS6522x Burton
 *
 *  \brief      LDO rail group selection.
 *
 *  @{
 */
#define PMIC_POWER_TPS6522X_LDO_RAIL_SEL_NONE  (0U)
#define PMIC_POWER_TPS6522X_LDO_RAIL_SEL_MCU   (1U)
#define PMIC_POWER_TPS6522X_LDO_RAIL_SEL_SOC   (2U)
#define PMIC_POWER_TPS6522X_LDO_RAIL_SEL_OTHER (3U)
/** @} */

/**
 *  \anchor     Pmic_Tps6522xBurton_vmonDeglitchSelBitField
 *  \name       VMON_DEGLITCH_SEL Bit Field Enumeration for TPS6522x Burton
 *
 *  \brief      VMON deglitch time select for BUCKx_VMON, LDOx_VMON, VMONx, and VCCA_VMON.
 *
 *  @{
 */
#define PMIC_POWER_TPS6522X_VMON_DEGLITCH_SEL_BUCK_LDO_VMON_4_US_VCCA_4_US     (0U)
#define PMIC_POWER_TPS6522X_VMON_DEGLITCH_SEL_BUCK_LDO_VMON_20_US_VCCA_20_US   (1U)
#define PMIC_POWER_TPS6522X_VMON_DEGLITCH_SEL_BUCK_LDO_VMON_0_5_US_VCCA_0_5_US (2U)
#define PMIC_POWER_TPS6522X_VMON_DEGLITCH_SEL_BUCK_LDO_VMON_0_5_US_VCCA_4_US   (3U)
#define PMIC_POWER_TPS6522X_VMON_DEGLITCH_SEL_BUCK_LDO_VMON_0_5_US_VCCA_20_US  (4U)
#define PMIC_POWER_TPS6522X_VMON_DEGLITCH_SEL_BUCK_LDO_VMON_4_US_VCCA_0_5_US   (5U)
#define PMIC_POWER_TPS6522X_VMON_DEGLITCH_SEL_BUCK_LDO_VMON_4_US_VCCA_20_US    (6U)
/** @} */

/**
 *  \anchor     Pmic_Tps6522xBurton_vmon2EnBitField
 *  \name       VMON2_EN Bit Field Enumeration for TPS6522x Burton
 *
 *  \brief      VMON2 enable/disable.
 *
 *  @{
 */
#define PMIC_POWER_TPS6522X_VMON2_DISABLE (0U)
#define PMIC_POWER_TPS6522X_VMON2_ENABLE  (1U)
/** @} */

/**
 *  \anchor     Pmic_Tps6522xBurton_vmon1EnBitField
 *  \name       VMON1_EN Bit Field Enumeration for TPS6522x Burton
 *
 *  \brief      VMON1 enable/disable.
 *
 *  @{
 */
#define PMIC_POWER_TPS6522X_VMON1_DISABLE (0U)
#define PMIC_POWER_TPS6522X_VMON1_ENABLE  (1U)
/** @} */

/**
 *  \anchor     Pmic_Tps6522xBurton_vccaVmonEnBitField
 *  \name       VCCA_VMON_EN Bit Field Enumeration for TPS6522x Burton
 *
 *  \brief      VCCA_VMON enable/disable.
 *
 *  @{
 */
#define PMIC_POWER_TPS6522X_VCCA_VMON_DISABLE (0U)
#define PMIC_POWER_TPS6522X_VCCA_VMON_ENABLE  (1U)
/** @} */

/**
 *  \anchor     Pmic_Tps6522xBurton_vccaPgSetBitField
 *  \name       VCCA_PG_SET Bit Field Enumeration for TPS6522x Burton
 *
 *  \brief      VCCA_VMON powergood level.
 *
 *  @{
 */
#define PMIC_POWER_TPS6522X_VCCA_PG_LEVEL_3_3_V (0U)
#define PMIC_POWER_TPS6522X_VCCA_PG_LEVEL_5_0_V (1U)
/** @} */

/**
 *  \anchor     Pmic_Tps6522xBurton_vccaVmonThrBitField
 *  \name       VCCA_VMON_THR Bit Field Enumeration for TPS6522x Burton
 *
 *  \brief      VCCA_VMON powergood high/low threshold level.
 *
 *  @{
 */
#define PMIC_POWER_TPS6522X_VCCA_VMON_THR_3_PCT  (0U)
#define PMIC_POWER_TPS6522X_VCCA_VMON_THR_4_PCT  (1U)
#define PMIC_POWER_TPS6522X_VCCA_VMON_THR_6_PCT  (2U)
#define PMIC_POWER_TPS6522X_VCCA_VMON_THR_10_PCT (3U)
/** @} */

/**
 *  \anchor     Pmic_Tps6522xBurton_vccaRailSelBitField
 *  \name       VCCA_GRP_SEL Bit Field Enumeration for TPS6522x Burton
 *
 *  \brief      VCCA_VMON rail group selection.
 *
 *  @{
 */
#define PMIC_POWER_TPS6522X_VCCA_RAIL_SEL_NONE  (0U)
#define PMIC_POWER_TPS6522X_VCCA_RAIL_SEL_MCU   (1U)
#define PMIC_POWER_TPS6522X_VCCA_RAIL_SEL_SOC   (2U)
#define PMIC_POWER_TPS6522X_VCCA_RAIL_SEL_OTHER (3U)
/** @} */

/**
 *  \anchor     Pmic_Tps6522xBurton_vmon1ThrBitField
 *  \name       VMON1_THR Bit Field Enumeration for TPS6522x Burton
 *
 *  \brief      VMON1 powergood high/low threshold level for VMON.
 *
 *  @{
 */
#define PMIC_POWER_TPS6522X_VMON1_THR_3_PCT_OR_30_MV (0U)
#define PMIC_POWER_TPS6522X_VMON1_THR_4_PCT_OR_40_MV (1U)
#define PMIC_POWER_TPS6522X_VMON1_THR_6_PCT_OR_60_MV (2U)
#define PMIC_POWER_TPS6522X_VMON1_THR_8_PCT_OR_80_MV (3U)
/** @} */

/**
 *  \anchor     Pmic_Tps6522xBurton_vmon1RailSelBitField
 *  \name       VMON1_GRP_SEL Bit Field Enumeration for TPS6522x Burton
 *
 *  \brief      VMON1 rail group selection.
 *
 *  @{
 */
#define PMIC_POWER_TPS6522X_VMON1_RAIL_SEL_NONE  (0U)
#define PMIC_POWER_TPS6522X_VMON1_RAIL_SEL_MCU   (1U)
#define PMIC_POWER_TPS6522X_VMON1_RAIL_SEL_SOC   (2U)
#define PMIC_POWER_TPS6522X_VMON1_RAIL_SEL_OTHER (3U)
/** @} */

/**
 *  \anchor     Pmic_Tps6522xBurton_vmon2ThrBitField
 *  \name       VMON2_THR Bit Field Enumeration for TPS6522x Burton
 *
 *  \brief      VMON2 powergood high/low threshold level for VMON.
 *
 *  @{
 */
#define PMIC_POWER_TPS6522X_VMON2_THR_3_PCT (0U)
#define PMIC_POWER_TPS6522X_VMON2_THR_4_PCT (1U)
#define PMIC_POWER_TPS6522X_VMON2_THR_6_PCT (2U)
#define PMIC_POWER_TPS6522X_VMON2_THR_8_PCT (3U)
/** @} */

/**
 *  \anchor     Pmic_Tps6522xBurton_vmon2RailSelBitField
 *  \name       VMON2_GRP_SEL Bit Field Enumeration for TPS6522x Burton
 *
 *  \brief      VMON2 rail group selection.
 *
 *  @{
 */
#define PMIC_POWER_TPS6522X_VMON2_RAIL_SEL_NONE  (0U)
#define PMIC_POWER_TPS6522X_VMON2_RAIL_SEL_MCU   (1U)
#define PMIC_POWER_TPS6522X_VMON2_RAIL_SEL_SOC   (2U)
#define PMIC_POWER_TPS6522X_VMON2_RAIL_SEL_OTHER (3U)
/** @} */

/**
 *  \anchor     Pmic_powerTps6522xBuckPowerResourceCfg_s
 *  \name       BUCK Power Configuration Struct for TPS6522x Burton
 *
 *  \brief      This struct holds information relevent to the BUCK power resource. For validParams,
 *              refer to \ref Pmic_Tps6522xBurton_buckPwrRsrcCfgValidParamShift.
 *
 *  \note       ValidParams is input param for all Set and Get APIs. Other
 *              params except validParams are input params for Set APIs and output
 *              params for Get APIs.
 *
 *  \param      buckPldn            BUCK pull-down resistor enable. For valid values,
 *                                      refer to \ref Pmic_Tps6522xBurton_buckPldnEnBitField
 *  \param      buckVmonEn          BUCK VMON enable. For valid values, refer to
 *                                      \ref Pmic_Tps6522xBurton_buckVmonEnBitField
 *  \param      buckPwmOption       BUCK PWM selection. For valid values, refer to
 *                                      \ref Pmic_Tps6522xBurton_buckFPWMBitField
 *  \param      buckEn              BUCK enable. For valid values, refer to
 *                                      \ref Pmic_Tps6522xBurton_buckEnBitField
 *  \param      buckSlewRate        BUCK slew rate. For valid values, refer to
 *                                      \ref Pmic_Tps6522xBurton_buckSlewRateBitField
 *  \param      buckVoltage_mv      BUCK voltage (mV). For possible ranges of BUCK1,
 *                                      refer to \ref Pmic_Tps6522xBurton_buck1VoltageRangesMV.
 *                                  For possible ranges of BUCK2, BUCK3, BUCK4, refer to
 *                                      \ref Pmic_Tps6522xBurton_buck2_3_4_VoltageRangesMV
 *  \param      buckVmonThr         BUCK VMON threshold. For valid values, refer to
 *                                      \ref Pmic_Tps6522xBurton_buckVmonThrBitField
 *  \param      buckRailGrpSel      BUCK rail group selection. For valid values, refer to
 *                                      \ref Pmic_Tps6522xBurton_buckRailSelBitField
 *
 *  @{
 */
typedef struct Pmic_powerTps6522xBuckPowerResourceCfg_s
{
    uint8_t validParams;
    uint8_t buckPldn;
    uint8_t buckVmonEn;
    uint8_t buckPwmOption;
    uint8_t buckEn;
    uint8_t buckSlewRate;
    uint16_t buckVoltage_mv;
    uint8_t buckVmonThr;
    uint8_t buckRailGrpSel;
} Pmic_powerTps6522xBuckPowerResourceCfg_t;
/** @} */

/**
 *  \anchor     Pmic_powerTps6522xLdoPowerResourceCfg_s
 *  \name       LDO Power Configuration Struct for TPS6522x Burton
 *
 *  \brief      This struct holds information relevent to the LDO power resource. For validParams,
 *              refer to \ref Pmic_Tps6522xBurton_ldoPwrRsrcCfgValidParamShift.
 *
 *  \note       ValidParams is input param for all Set and Get APIs. Other
 *              params except validParams are input params for Set APIs and output
 *              params for Get APIs.
 *
 *  \param      ldoDischargeEn      LDO discharge enable. For valid values, refer to
 *                                      \ref Pmic_Tps6522xBurton_ldoDischargeEnBitField
 *  \param      ldoVmonEn           LDO VMON enable. For valid values, refer to
 *                                      \ref Pmic_Tps6522xBurton_ldoVmonEnBitField
 *  \param      ldoEn               LDO enable. For valid values, refer to
 *                                      \ref Pmic_Tps6522xBurton_ldoEnBitField
 *  \param      ldoMode             LDO mode. For valid values, refer to
 *                                      \ref Pmic_Tps6522xBurton_bypConfigBitField
 *  \param      ldoVoltage_mv       LDO voltage (mV). For possible ranges of LDO1, refer to
 *                                      \ref Pmic_Tps6522xBurton_ldo1VoltageRangesMV.
 *                                  For possible ranges of LDO2 and LDO3, refer to
 *                                      \ref Pmic_Tps6522xBurton_ldo2_3_VoltageRangesMV
 *  \param      ldoVmonThr          LDO VMON threshold. For valid values, refer to
 *                                      \ref Pmic_Tps6522xBurton_ldoVmonThrBitField
 *  \param      ldoRailGrpSel       LDO rail group selection. For valid values, refer to
 *                                      \ref Pmic_Tps6522xBurton_ldoRailSelBitField
 *
 *  @{
 */
typedef struct Pmic_powerTps6522xLdoPowerResourceCfg_s
{
    uint8_t validParams;
    uint8_t ldoDischargeEn;
    uint8_t ldoVmonEn;
    uint8_t ldoEn;
    uint8_t ldoMode;
    uint16_t ldoVoltage_mv;
    uint8_t ldoVmonThr;
    uint8_t ldoRailGrpSel;
} Pmic_powerTps6522xLdoPowerResourceCfg_t;
/** @} */

/**
 *  \anchor     Pmic_powerTps6522xVccaVmonPowerResourceCfg_s
 *  \name       VCCA_VMON/VMONx Power Configuration Struct for TPS6522x Burton
 *
 *  \brief      This struct holds information relevent to the VCCA_VMON and VMONx power resources.
 *              For validParams, refer to \ref Pmic_Tps6522xBurton_vccaVmonPwrRsrcCfgValidParamShift.
 *
 *  \note       ValidParams is input param for all Set and Get APIs. Other
 *              params except validParams are input params for Set APIs and output
 *              params for Get APIs.
 *
 *  \param      vmonDeglitchSel     Deglitech selection for voltage monitors.
 *                                  For valid values, refer to \ref Pmic_Tps6522xBurton_vmonDeglitchSelBitField
 *  \param      vmon2En             VMON2 enable. For valid values, refer to \ref Pmic_Tps6522xBurton_vmon2EnBitField
 *  \param      vmon1En             VMON1 enable. For valid values, refer to \ref Pmic_Tps6522xBurton_vmon1EnBitField
 *  \param      vccaVmonEn          VCCA_VMON enable. For valid values, refer to
 *                                      \ref Pmic_Tps6522xBurton_vccaVmonEnBitField
 *  \param      vccaPgLevel         VCCA_VMON PG level. For valid values, refer to
 *                                      \ref Pmic_Tps6522xBurton_vccaPgSetBitField
 *  \param      vccaVmonThr         VCCA_VMON threshold. For valid values, refer to
 *                                      \ref Pmic_Tps6522xBurton_vccaVmonThrBitField
 *  \param      vccaRailGrpSel      VCCA_VMON rail group selection. For valid values,
 *                                      refer to \ref Pmic_Tps6522xBurton_vccaRailSelBitField
 *  \param      vmon1Thr            VMON1 threshold. For valid values, refer to
 *                                      \ref Pmic_Tps6522xBurton_vmon1ThrBitField
 *  \param      vmon1PgLevel_mv     VMON1 PG level (mV). For possible ranges,
 *                                      \ref Pmic_Tps6522xBurton_vmon1VoltageRangesMV
 *  \param      vmon1RailGrpSel     VMON1 rail group selection. For valid values, refer to
 *                                      \ref Pmic_Tps6522xBurton_vmon1RailSelBitField
 *  \param      vmon2Thr            VMON2 treshold. For valid values, refer to
 *                                      \ref Pmic_Tps6522xBurton_vmon2ThrBitField
 *  \param      vmon2PgLevel_mv     VMON2 PG level (mV). For possible ranges,
 *                                      \ref Pmic_Tps6522xBurton_vmon2VoltageRangesMV
 *  \param      vmon2RailGrpSel     VMON2 rail group selection. For valid values, refer to
 *                                      \ref Pmic_Tps6522xBurton_vmon2RailSelBitField
 *  @{
 */
typedef struct Pmic_powerTps6522xVccaVmonPowerResourceCfg_s
{
    uint16_t validParams;
    uint8_t vmonDeglitchSel;
    uint8_t vmon2En;
    uint8_t vmon1En;
    uint8_t vccaVmonEn;
    uint8_t vccaPgLevel;
    uint8_t vccaVmonThr;
    uint8_t vccaRailGrpSel;
    uint8_t vmon1Thr;
    uint16_t vmon1PgLevel_mv;
    uint8_t vmon1RailGrpSel;
    uint8_t vmon2Thr;
    uint16_t vmon2PgLevel_mv;
    uint8_t vmon2RailGrpSel;
} Pmic_powerTps6522xVccaVmonPowerResourceCfg_t;
/** @} */

/**
 *  \anchor     Pmic_powerTps6522xPowerResourceCfg_s
 *  \name       Power Configuration Struct for TPS6522x Burton
 *
 *  \brief      This struct holds information relevent to all power resource of TPS6522x Burton.
 *              For validParams, refer to \ref Pmic_Tps6522xBurton_pwrRsrcCfgValidParamShift.
 *
 *  \note       ValidParams is input param for all Set and Get APIs. Other
 *              params except validParams are input params for Set APIs and output
 *              params for Get APIs.
 *
 *  \param      buckPwrRsrcCfg      Array of BUCK power resource configuration structures.
 *                                  Each element in this array is a BUCK - for example, element
 *                                  zero is the power resource configuration structure for BUCK1.
 *                                  For more information, \ref Pmic_Tps6522xBurton_buckPwrRsrcCfg
 *  \param      ldoPwrRsrcCfg       Array of LDO power resource configuration structures.
 *                                  Each element in this array is a LDO - for example, element
 *                                  zero is the power resource configuration structure for LDO1.
 *                                  For more information, \ref Pmic_Tps6522xBurton_ldoPwrRsrcCfg
 *  \param      vccaVmonPwrRsrcCfg  VCCA/VMON power resource configuration struct.
 *                                  For more information, \ref Pmic_Tps6522xBurton_vccaVmonPwrRsrcCfg
 *
 *  @{
 */
typedef struct Pmic_powerTps6522xPowerResourceCfg_s
{
    uint16_t                                     validParams;
    Pmic_powerTps6522xBuckPowerResourceCfg_t     buckPwrRsrcCfg[PMIC_POWER_TPS6522X_MAX_BUCK_NUM];
    Pmic_powerTps6522xLdoPowerResourceCfg_t      ldoPwrRsrcCfg[PMIC_POWER_TPS6522X_MAX_LDO_NUM];
    Pmic_powerTps6522xVccaVmonPowerResourceCfg_t vccaVmonPwrRsrcCfg;
} Pmic_powerTps6522xPowerResourceCfg_t;
/** @} */

/**
 *  \anchor     Pmic_powerTps6522xBuckRegisters_s
 *  \name       TPS6522x Burton BUCK Registers
 *
 *  \brief      This struct holds registers relevent to the BUCK power resource.
 *
 *  @{
 */
typedef struct Pmic_powerTps6522xBuckRegisters_s
{
    uint8_t buckCtrlRegAddr;
    uint8_t buckConfRegAddr;
    uint8_t buckVoutRegAddr;
    uint8_t buckPgWindowRegAddr;
    uint8_t buckRailSelRegAddr;
} Pmic_powerTps6522xBuckRegisters_t;
/** @} */

/**
 *  \anchor     Pmic_powerTps6522xLdoRegisters_s
 *  \name       TPS6522x Burton LDO Registers
 *
 *  \brief      This struct holds registers relevent to the LDO power resource.
 *
 *  @{
 */
typedef struct Pmic_powerTps6522xLdoRegisters_s
{
    uint8_t ldoCtrlRegAddr;
    uint8_t ldoVoutRegAddr;
    uint8_t ldoPgWindowRegAddr;
    uint8_t ldoRailSelRegAddr;
} Pmic_powerTps6522xLdoRegisters_t;
/** @} */

/**
 *  \anchor     Pmic_powerTps6522xVccaVmonRegisters_s
 *  \name       TPS6522x Burton VCCA_VMON and VMONx Registers
 *
 *  \brief      This struct holds registers relevent to the VCCA_VMON and VMONx power resources.
 *
 *  @{
 */
typedef struct Pmic_powerTps6522xVccaVmonRegisters_s
{
    uint8_t vccaVmonCtrlRegAddr;
    uint8_t vccaPgWindowRegAddr;
    uint8_t vmonPgWindowRegAddr;
    uint8_t vmonPgLevelRegAddr;
    uint8_t vccaVmonRailSelRegAddr;
} Pmic_powerTps6522xVccaVmonRegisters_t;
/** @} */

/**
 *  \anchor     Pmic_powerTps6522xThermalStat_s
 *  \name       TPS6522x Burton Thermal Statuses
 *
 *  \brief      This struct holds all the thermal statuses. For validParams,
 *              refer to \ref Pmic_PowerThermalStatValidParamCfg.
 *
 * \note        ValidParams is input param for all Get APIs. Other
 *              params except validParams are output params for Get APIs.
 *
 * \param       twarnStat       Temperature warning status
 * \param       tsdOrdStat      Orderly thermal shutdown status
 * \param       tsdImmStat      Immediate thermal shutdown status
 *
 *  @{
 */
typedef struct Pmic_powerTps6522xThermalStat_s
{
    uint8_t validParams;
    bool    twarnStat;
    bool    tsdOrdStat;
    bool    tsdImmStat;
} Pmic_powerTps6522xThermalStat_t;
/** @} */

/**
 *  \anchor     Pmic_powerTps6522xThermalCfg_s
 *  \name       TPS6522x Burton Thermal Configuration Struct
 *
 *  \brief      This struct holds information that is relevant to PMIC thermal configurations.
 *              For possible validParams, refer to \ref Pmic_Tps6522xBurton_thermalCfgValidParam.
 *
 *  \note       ValidParams is input param for all Set and Get APIs. Other
 *              params except validParams are input params for Set APIs and output
 *              params for Get APIs.
 *
 *  \param      tsdOrdLvl       Used to set the orderly thermal stutdown level.
 *                              For valid values, refer to \ref Pmic_Tps6522xBurton_TwarnLvlBitField
 *  \param      twarnLvl        Used to set the temperature warning level.
 *                              For valid values, refer to \ref Pmic_Tps6522xBurton_TwarnLvlBitField
 *
 *  @{
 */
typedef struct Pmic_powerTps6522xThermalCfg_s
{
    uint8_t validParams;
    uint8_t tsdOrdLvl;
    uint8_t twarnLvl;
} Pmic_powerTps6522xThermalCfg_t;
/** @} */

/**
 *  \name       TPS6522x BUCK Registers
 *  \brief      Array of BUCK register sets. Each element in this array is a set of configuration
 *              registers for a particular BUCK. For instance, element zero is the set of registers
 *              for BUCK1. This array is mainly used interally by the Power APIs, but end-user could
 *              utilize this array if direct register access is desired.
 *
 *  @{
 */
extern const Pmic_powerTps6522xBuckRegisters_t gTps6522xBuckRegisters[];
/** @} */

/**
 *  \name       TPS6522x LDO Registers
 *  \brief      Array of LDO register sets. Each element in this array is a set of configuration
 *              registers for a particular LDO. For instance, element zero is the set of registers
 *              for LDO1. This array is mainly used interally by the Power APIs, but end-user could
 *              utilize this array if direct register access is desired.
 *
 *  @{
 */
extern const Pmic_powerTps6522xLdoRegisters_t gTps6522xLdoRegisters[];
/** @} */

/**
 *  \name       TPS6522x VCCA/VMONx Registers
 *  \brief      Array of VCCA/VMONx register sets. Each element in this array is a set of configuration
 *              registers for a particular VCCA/VMONx. For instance, element zero is the set of registers
 *              for VMON1. This array is mainly used interally by the Power APIs, but end-user could utilize
 *              this array if direct register access is desired.
 *
 *  @{
 */
extern const Pmic_powerTps6522xVccaVmonRegisters_t gTps6522xVccaVmonRegisters[];
/** @} */

/* ========================================================================== */
/*                         Function Declarations                              */
/* ========================================================================== */

/**
 *  \brief      This function is used to get the configuration of any number of TPS6522x
 *              Burton power resources (BUCKs, LDOs, and voltage monitors). In particular,
 *              The API can be used to get the configurations of a single power resource,
 *              get the configurations of multiple power resources, or get the configurations
 *              of all power resources at once via single API call.
 *
 *  \details    To use this API, end user needs to set the desired power resource's
 *              validParam within \p pPwrResourceCfg struct. For valid values, refer to
 *              \ref Pmic_Tps6522xBurton_pwrRsrcCfgValidParamShift. Setting the validParam
 *              indicates that the end user is interested in that specific power resource.
 *              Afterwards, end user will need to set the desired power resource's configuration
 *              validParam(s) within buckPwrRsrcCfg, ldoPwrRsrcCfg, or vccaVmonPwrRsrcCfg. For
 *              valid values, refer to \ref Pmic_Tps6522xBurton_buckPwrRsrcCfgValidParamShift,
 *              \ref Pmic_Tps6522xBurton_ldoPwrRsrcCfgValidParamShift, and
 *              \ref Pmic_Tps6522xBurton_vccaVmonPwrRsrcCfgValidParamShift.
 *              Setting a validParam indicates that the end user is interested in getting
 *              a specific configuration of a power resource.
 *
 *              Supported obtainable configurations by this API for a BUCK are:
 *              1. Buck pull-down resistor
 *              2. Buck VMON enable
 *              3. Buck PWM mode (auto or forced)
 *              4. Buck enable
 *              5. Buck slew-rate
 *              6. Buck voltage (mV)
 *              7. Buck VMON powergood threshold
 *              8. Buck rail group selection
 *
 *              Supported obtainable configurations by this API for a LDO are:
 *              1. LDO discharge enable
 *              2. LDO VMON enable
 *              3. LDO enable
 *              4. LDO mode (bypass mode, LDO mode)
 *              5. LDO voltage (mV)
 *              6. LDO VMON powergood threshold
 *              7. LDO rail group selection
 *
 *              Supported obtainable configurations by this API for a VCCA_VMON/VMONx are:
 *              1. VMON deglitch selection
 *              2. VCCA_VMON/VMONx enable
 *              3. VCCA_VMON/VMONx powergood threshold
 *              4. VCCA_VMON/VMONx powergood level (mV)
 *              5. VCCA_VMON/VMONx rail group selection
 *
 *  \param      pPmicCoreHandle     [IN]        PMIC interface handle
 *  \param      pPwrResourceCfg     [OUT]       Pointer to power resource configuration struct
 *                                              in which API will use to store information
 *
 *  \return     Success code if power resource configurations are obtained, error code otherwise.
 *              For valid success/error codes, refer to \ref Pmic_ErrorCodes
 */
int32_t Pmic_powerTps6522xGetPwrResourceCfg(Pmic_CoreHandle_t                    *pPmicCoreHandle,
                                            Pmic_powerTps6522xPowerResourceCfg_t *pPwrResourceCfg);

/**
 *  \brief      This function is used to get all desired information for a BUCK.
 *              Desired information is specified by the user via setting validParams
 *              within \p pBuckPowerResourceCfg struct. For values of validParams,
 *              refer to \ref Pmic_Tps6522xBurton_buckPwrRsrcCfgValidParamShift.
 *
 *  \details    End user could use this API if they only want to get the configuration
 *              of a single BUCK power resource. End user could also use this API to get
 *              BUCK power resource configuration if their application is memory constrained.
 *              The API Pmic_powerTps6522xGetPwrResourceCfg() is a superset of this
 *              API and will accomplish more in a single API call than this API,
 *              however it is more memory intensive.
 *
 *              Supported obtainable configurations by this API for a BUCK are:
 *              1. Buck pull-down resistor
 *              2. Buck VMON enable
 *              3. Buck PWM mode (auto or forced)
 *              4. Buck enable
 *              5. Buck slew-rate
 *              6. Buck voltage (mV)
 *              7. Buck VMON powergood threshold
 *              8. Buck rail group selection
 *
 *  \param      pPmicCoreHandle             [IN]        PMIC interface handle
 *  \param      pBuckPowerResourceCfg       [IN/OUT]    BUCK power resource configuration
 *  \param      buckNum                     [IN]        Indicates which BUCK the API is working with.
 *                                                      For valid values, refer to
 *                                                          \ref Pmic_Tps6522xBurton_buckPwrRsrcNum
 *
 *  \return     Success code if BUCK information is stored at pBuckPowerResourceCfg,
 *              error code otherwise. For valid success/error codes, refer to \ref Pmic_ErrorCodes
 */
int32_t Pmic_powerTps6522xGetBuckPwrResourceCfg(Pmic_CoreHandle_t                        *pPmicCoreHandle,
                                                Pmic_powerTps6522xBuckPowerResourceCfg_t *pBuckPowerResourceCfg,
                                                const uint8_t                             buckNum);

/**
 *  \brief      This function is used to get all desired information for a LDO.
 *              Desired information is specified by the user via setting validParams
 *              within \p pLdoPowerResourceCfg struct. For values of validParams,
 *              refer to \ref Pmic_Tps6522xBurton_ldoPwrRsrcCfgValidParamShift.
 *
 *  \details    End user could use this API if they only want to get the configuration
 *              of a single LDO power resource. End user could also use this API to get
 *              LDO power resource configuration if their application is memory constrained.
 *              The API Pmic_powerTps6522xGetPwrResourceCfg() is a superset of this
 *              API and will accomplish more in a single API call than this API,
 *              however it is more memory intensive.
 *
 *              Supported configurable options by this API for a LDO are:
 *              1. LDO discharge enable
 *              2. LDO VMON enable
 *              3. LDO enable
 *              4. LDO mode (bypass mode, LDO mode)
 *              5. LDO voltage (mV)
 *              6. LDO VMON powergood threshold
 *              7. LDO rail group selection
 *
 *  \param      pPmicCoreHandle         [IN]        PMIC interface handle
 *  \param      pLdoPowerResourceCfg    [IN/OUT]    LDO power resource configuration
 *  \param      ldoNum                  [IN]        Indicates which LDO the API is working with.
 *                                                  For valid values, refer to
 *                                                      \ref Pmic_Tps6522xBurton_ldoPwrRsrcNum
 *
 *  \return     Success code if desired LDO information is stored at pLdoPowerResourceCfg,
 *              error code otherwise. For valid success/error codes, refer to \ref Pmic_ErrorCodes
 */
int32_t Pmic_powerTps6522xGetLdoPwrResourceCfg(Pmic_CoreHandle_t                       *pPmicCoreHandle,
                                               Pmic_powerTps6522xLdoPowerResourceCfg_t *pLdoPowerResourceCfg,
                                               const uint8_t                            ldoNum);

/**
 *  \brief      This function is used to get all desired information for a VCCA/VMONx.
 *              Desired information is specified by the user via setting validParams
 *              within \p pVccaVmonPwrRsrcCfg struct. For values of validParams,
 *              refer to \ref Pmic_Tps6522xBurton_vccaVmonPwrRsrcCfgValidParamShift.
 *
 *  \details    End user could use this API if they only want to get the configuration
 *              of a single VCCA/VMONx power resource. End user could also use this API to
 *              get VCCA/VMONx power resource configuration if their application is memory
 *              constrained. The API Pmic_powerTps6522xGetPwrResourceCfg() is a superset of
 *              this API and will accomplish more in a single API call than this API, however
 *              it is more memory intensive.
 *
 *              Supported obtainable configurations by this API for a VCCA_VMON/VMONx are:
 *              1. VMON deglitch selection
 *              2. VCCA_VMON/VMONx enable
 *              3. VCCA_VMON/VMONx powergood threshold
 *              4. VCCA_VMON/VMONx powergood level (mV)
 *              5. VCCA_VMON/VMONx rail group selection
 *
 *  \param      pPmicCoreHandle         [IN]        PMIC interface handle
 *  \param      pVccaVmonPwrRsrcCfg     [IN/OUT]    VCCA/VMONx power resource configuration
 *  \param      vmonNum                 [IN]        Indicates which VCCA/VMONx the API is working with
 *                                                  For valid values, refer to
 *                                                      \ref Pmic_Tps6522xBurton_vccaVmonPwrRsrcNum
 *
 *  \return     Success code if VCCA/VMON information is stored at pVccaVmonPwrRsrcCfg,
 *              error code otherwise. For valid success/error codes, refer to \ref Pmic_ErrorCodes
 */
int32_t Pmic_powerTps6522xGetVccaVmonPwrResourceCfg(Pmic_CoreHandle_t                            *pPmicCoreHandle,
                                                    Pmic_powerTps6522xVccaVmonPowerResourceCfg_t *pVccaVmonPwrRsrcCfg,
                                                    const uint8_t                                 vmonNum);

/**
 *  \brief      This function is used to set the configuration of any number of TPS6522x Burton
 *              power resources (BUCKs, LDOs, and voltage monitors). In particular, The API can
 *              be used to set the configurations of a single power resource, set the configurations
 *              of multiple power resources, or set the configurations of all power resources at once
 *              via single API call.
 *
 *  \details    To use this API, end user needs to set the desired power resource's
 *              validParam within \p pPwrResourceCfg struct. For valid values, refer to
 *              \ref Pmic_Tps6522xBurton_pwrRsrcCfgValidParamShift. Setting the validParam
 *              indicates that the end user is interested in that specific power resource.
 *              Afterwards, end user will need to set the desired power resource's configuration
 *              validParam(s) within buckPwrRsrcCfg, ldoPwrRsrcCfg, or vccaVmonPwrRsrcCfg. For
 *              valid values, refer to \ref Pmic_Tps6522xBurton_buckPwrRsrcCfgValidParamShift,
 *              \ref Pmic_Tps6522xBurton_ldoPwrRsrcCfgValidParamShift, and
 *              \ref Pmic_Tps6522xBurton_vccaVmonPwrRsrcCfgValidParamShift.
 *              Setting a validParam indicates that the end user is interested in setting
 *              a specific configuration of a power resource.
 *
 *              Supported configurable options by this API for a BUCK are:
 *              1. Buck pull-down resistor
 *              2. Buck VMON enable
 *              3. Buck PWM mode (auto or forced)
 *              4. Buck enable
 *              5. Buck slew-rate
 *              6. Buck voltage (mV)
 *              7. Buck VMON powergood threshold
 *              8. Buck rail group selection
 *
 *              Supported configurable options by this API for a LDO are:
 *              1. LDO discharge enable
 *              2. LDO VMON enable
 *              3. LDO enable
 *              4. LDO mode (bypass mode, LDO mode)
 *              5. LDO voltage (mV)
 *              6. LDO VMON powergood threshold
 *              7. LDO rail group selection
 *
 *              Supported configurable options by this API for a VCCA_VMON/VMONx are:
 *              1. VMON deglitch selection
 *              2. VCCA_VMON/VMONx enable
 *              3. VCCA_VMON/VMONx powergood threshold
 *              4. VCCA_VMON/VMONx powergood level (mV)
 *              5. VCCA_VMON/VMONx rail group selection
 *
 *  \param      pPmicCoreHandle     [IN]        PMIC interface handle
 *  \param      pwrResourceCfg      [IN]        Power resource configuration struct used by the API
 *                                              to set power configurations of TPS6522x
 *
 *  \return     Success code if power resource configurations are set, error code otherwise.
 *              For valid success/error codes, refer to \ref Pmic_ErrorCodes
 */
int32_t Pmic_powerTps6522xSetPwrResourceCfg(Pmic_CoreHandle_t                         *pPmicCoreHandle,
                                            const Pmic_powerTps6522xPowerResourceCfg_t pwrResourceCfg);

/**
 *  \brief      This function is used to set all desired information for a BUCK.
 *              Desired information is specified by the user via setting validParams
 *              within \p pBuckPowerResourceCfg struct. For values of validParams,
 *              refer to \ref Pmic_Tps6522xBurton_buckPwrRsrcCfgValidParamShift.
 *
 *  \details    End user could use this API if they only want to set the configuration
 *              of a single BUCK power resource. End user could also use this API to set
 *              BUCK power resource configuration if their application is memory constrained.
 *              The API Pmic_powerTps6522xSetPwrResourceCfg() is a superset of this
 *              API and will accomplish more in a single API call than this API,
 *              however it is more memory intensive.
 *
 *              Supported configurable options by this API for a BUCK are:
 *              1. Buck pull-down resistor
 *              2. Buck VMON enable
 *              3. Buck PWM mode (auto or forced)
 *              4. Buck enable
 *              5. Buck slew-rate
 *              6. Buck voltage (mV)
 *              7. Buck VMON powergood threshold
 *              8. Buck rail group selection
 *
 *  \param      pPmicCoreHandle             [IN]        PMIC interface handle
 *  \param      buckPowerResourceCfg        [IN]        BUCK power resource configuration struct
 *  \param      buckNum                     [IN]        Indicates which BUCK the API is working with.
 *                                                      For valid values, refer to
 *                                                          \ref Pmic_Tps6522xBurton_buckPwrRsrcNum
 *
 *  \return     Success code if Buck power resource configurations are set, error code otherwise.
 *              For valid success/error codes, refer to \ref Pmic_ErrorCodes
 */
int32_t Pmic_powerTps6522xSetBuckPwrResourceCfg(Pmic_CoreHandle_t                             *pPmicCoreHandle,
                                                const Pmic_powerTps6522xBuckPowerResourceCfg_t buckPowerResourceCfg,
                                                const uint8_t                                  buckNum);

/**
 *  \brief      This function is used to set all desired information for a LDO.
 *              Desired information is specified by the user via setting validParams
 *              within \p pLdoPowerResourceCfg struct. For values of validParams,
 *              refer to \ref Pmic_Tps6522xBurton_ldoPwrRsrcCfgValidParamShift.
 *
 *  \details    End user could use this API if they only want to set the configuration
 *              of a single LDO power resource. End user could also use this API to set
 *              LDO power resource configuration if their application is memory constrained.
 *              The API Pmic_powerTps6522xSetPwrResourceCfg() is a superset of this
 *              API and will accomplish more in a single API call than this API,
 *              however it is more memory intensive.
 *
 *              Supported configurable options by this API for a LDO are:
 *              1. LDO discharge enable
 *              2. LDO VMON enable
 *              3. LDO enable
 *              4. LDO mode (bypass mode, LDO mode)
 *              5. LDO voltage (mV)
 *              6. LDO VMON powergood threshold
 *              7. LDO rail group selection
 *
 *  \param      pPmicCoreHandle         [IN]        PMIC interface handle
 *  \param      ldoPowerResourceCfg     [IN]        LDO power resource configuration struct
 *  \param      ldoNum                  [IN]        Indicates which LDO the API is working with.
 *                                                  For valid values, refer to
 *                                                      \ref Pmic_Tps6522xBurton_ldoPwrRsrcNum
 *
 *  \return     Success code if LDO power resource configurations are set, error code otherwise.
 *              For valid success/error codes, refer to \ref Pmic_ErrorCodes
 */
int32_t Pmic_powerTps6522xSetLdoPwrResourceCfg(Pmic_CoreHandle_t                            *pPmicCoreHandle,
                                               const Pmic_powerTps6522xLdoPowerResourceCfg_t ldoPowerResourceCfg,
                                               const uint8_t                                 ldoNum);

/**
 *  \brief      This function is used to get all desired information for a VCCA/VMONx.
 *              Desired information is specified by the user via setting validParams
 *              within \p pVccaVmonPwrRsrcCfg struct. For values of validParams,
 *              refer to \ref Pmic_Tps6522xBurton_vccaVmonPwrRsrcCfgValidParamShift.
 *
 *  \details    End user could use this API if they only want to get the configuration
 *              of a single VCCA/VMONx power resource. End user could also use this API to
 *              get VCCA/VMONx power resource configuration if their application is memory
 *              constrained. The API Pmic_powerTps6522xGetPwrResourceCfg() is a superset of
 *              this API and will accomplish more in a single API call than this API, however
 *              it is more memory intensive.
 *
 *              Supported configurable options by this API for a VCCA_VMON/VMONx are:
 *              1. VMON deglitch selection
 *              2. VCCA_VMON/VMONx enable
 *              3. VCCA_VMON/VMONx powergood threshold
 *              4. VCCA_VMON/VMONx powergood level (mV)
 *              5. VCCA_VMON/VMONx rail group selection
 *
 *  \param      pPmicCoreHandle         [IN]        PMIC interface handle
 *  \param      vccaVmonPwrRsrcCfg      [IN]        VCCA/VMON power resource configuration struct
 *  \param      vmonNum                 [IN]        Indicates which VCCA/VMONx the API is working with.
 *                                                  For valid values, refer to
 *                                                      \ref Pmic_Tps6522xBurton_vccaVmonPwrRsrcNum
 *
 *  \return     Success code if VCCA_VMON/VMONx configurations are set, error code otherwise.
 *              For valid success/error codes, refer to \ref Pmic_ErrorCodes
 */
int32_t
Pmic_powerTps6522xSetVccaVmonPwrResourceCfg(Pmic_CoreHandle_t                                 *pPmicCoreHandle,
                                            const Pmic_powerTps6522xVccaVmonPowerResourceCfg_t vccaVmonPwrRsrcCfg,
                                            const uint8_t                                      vmonNum);

/**
 *  \brief      This function is used to get the UVOV status of any TPS6522x Burton power resource
 *              (BUCKs, LDOs, and voltage monitors).
 *
 *  \param      pPmicCoreHandle     [IN]        PMIC interface handle
 *  \param      pwrRsrcUVOVStatus   [IN]        Desired power resource UVOV status to be obtained
 *  \param      pUnderOverVoltStat  [OUT]       Pointer to boolean holding power resource UVOV status
 *
 *  \return     Success code if power resource UVOV status is obtained, error code otherwise.
 *              For valid success/error codes, refer to \ref Pmic_ErrorCodes
 */
int32_t Pmic_powerTps6522xGetPwrRsrcStat(Pmic_CoreHandle_t        *pPmicCoreHandle,
                                         const uint8_t pwrRsrcUVOVStatus,
                                         bool                     *pUnderOverVoltStat);

/**
 *  \brief      This function is used to get the thermal statuses of TPS6522x Burton (TWARN_STAT, TSD_ORD_STAT,
 *              TSD_IMM_STAT).
 *
 *  \param      pPmicCoreHandle     [IN]        PMIC interface handle
 *  \param      pThermalStat        [OUT]       Pointer to thermal status struct that will hold the thermal statuses
 *
 *  \return     Success code if thermal statuses are obtained, error code otherwise.
 *              For valid success/error codes, refer to \ref Pmic_ErrorCodes
 */
int32_t Pmic_powerTps6522xGetThermalStat(Pmic_CoreHandle_t               *pPmicCoreHandle,
                                         Pmic_powerTps6522xThermalStat_t *pThermalStat);

/**
 *  \brief      This function is used to get the thermal configuration of TPS6522x Burton.
 *
 *  \param      pPmicCoreHandle     [IN]        PMIC interface handle
 *  \param      pThermalCfg         [OUT]       Pointer to thermal configuration struct that will hold the thermal
 *                                              configurations
 *
 *  \return     Success code if thermal configurations are obtained, error code otherwise.
 *              For valid success/error codes, refer to \ref Pmic_ErrorCodes
 */
int32_t Pmic_powerTps6522xGetThermalCfg(Pmic_CoreHandle_t              *pPmicCoreHandle,
                                        Pmic_powerTps6522xThermalCfg_t *pThermalCfg);

/**
 *  \brief      This function is used to set the thermal configuration of TPS6522x Burton.
 *
 *  \param      pPmicCoreHandle     [IN]        PMIC interface handle
 *  \param      thermalCfg          [IN]        Thermal configuration struct
 *
 *  \return     Success code if PMIC thermal configurations are set, error code otherwise.
 *              For valid success/error codes, refer to \ref Pmic_ErrorCodes
 */
int32_t Pmic_powerTps6522xSetThermalCfg(Pmic_CoreHandle_t                   *pPmicCoreHandle,
                                        const Pmic_powerTps6522xThermalCfg_t thermalCfg);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* PMIC_POWER_TPS6522X_H_ */

/** @} */
