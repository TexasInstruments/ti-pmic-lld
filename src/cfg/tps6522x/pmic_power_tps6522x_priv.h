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
 * \file   pmic_power_tps6522x_priv.h
 *
 * \brief  The macro definitions, structures and function prototypes for
 *         TPS6522x BURTON PMIC driver specific PMIC power configuration
 *
 */

#ifndef PMIC_POWER_TPS6522X_PRIV_H_
#define PMIC_POWER_TPS6522X_PRIV_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include "pmic_power_priv.h"

#ifdef __cplusplus
extern "C"
{
#endif

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/**
 *  \anchor     Tps6522x_voltageSteps
 *  \name       voltage steps for BUCKs, LDOs, VCCA_VMON/VMONs supported by TPS6522x Burton
 *
 *  @{
 */
#define TPS6522X_VOLTAGE_STEP_0_MV                     (0U)
#define TPS6522X_VOLTAGE_STEP_5_MV                     (5U)
#define TPS6522X_VOLTAGE_STEP_10_MV                    (10U)
#define TPS6522X_VOLTAGE_STEP_20_MV                    (20U)
#define TPS6522X_VOLTAGE_STEP_25_MV                    (25U)
#define TPS6522X_VOLTAGE_STEP_50_MV                    (50U)
/** @} */

/**
 *  \anchor     Tps6522x_buck1VoltageRangesMV
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
#define TPS6522X_BUCK1_VOLT_RANGE_1_MIN_VOLT     (500U)
#define TPS6522X_BUCK1_VOLT_RANGE_1_MAX_VOLT     (580U)
#define TPS6522X_BUCK1_VOLT_RANGE_2_MIN_VOLT     (600U)
#define TPS6522X_BUCK1_VOLT_RANGE_2_MAX_VOLT     (1095U)
#define TPS6522X_BUCK1_VOLT_RANGE_3_MIN_VOLT     (1100U)
#define TPS6522X_BUCK1_VOLT_RANGE_3_MAX_VOLT     (1650U)
#define TPS6522X_BUCK1_VOLT_RANGE_4_MIN_VOLT     (1660U)
#define TPS6522X_BUCK1_VOLT_RANGE_4_MAX_VOLT     (3300U)
/** @} */

/**
 *  \anchor     Tps6522x_buck1VoltageRangesVset
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
#define TPS6522X_BUCK1_VOLT_RANGE_1_MIN_VSET        (0xAU)
#define TPS6522X_BUCK1_VOLT_RANGE_1_MAX_VSET        (0xEU)
#define TPS6522X_BUCK1_VOLT_RANGE_2_MIN_VSET        (0xFU)
#define TPS6522X_BUCK1_VOLT_RANGE_2_MAX_VSET        (0x72U)
#define TPS6522X_BUCK1_VOLT_RANGE_3_MIN_VSET        (0x73U)
#define TPS6522X_BUCK1_VOLT_RANGE_3_MAX_VSET        (0xAAU)
#define TPS6522X_BUCK1_VOLT_RANGE_4_MIN_VSET        (0xABU)
#define TPS6522X_BUCK1_VOLT_RANGE_4_MAX_VSET        (0xFDU)
/** @} */

/**
 *  \anchor     Tps6522x_buck2_3_4_VoltageRangesMV
 *  \name       BUCK 2, 3, 4 voltage ranges in mV supported by TPS6522x Burton
 *
 *  \brief      Ranges supported by TPS6522x Burton:
 *              - Range 1: 500 mV to 1150 mV in 25 mV steps
 *              - Range 2: 1200 mV to 3300 mV in 50 mV steps
 *
 *  @{
 */
#define TPS6522X_BUCK2_3_4_VOLT_RANGE_1_MIN_VOLT (500U)
#define TPS6522X_BUCK2_3_4_VOLT_RANGE_1_MAX_VOLT (1150U)
#define TPS6522X_BUCK2_3_4_VOLT_RANGE_2_MIN_VOLT (1200U)
#define TPS6522X_BUCK2_3_4_VOLT_RANGE_2_MAX_VOLT (3300U)
/** @} */

/**
 *  \anchor     Tps6522x_buck2_3_4_VoltageRangesVset
 *  \name       BUCK 2, 3, 4 voltage ranges in VSET supported by TPS6522x Burton
 *
 *  \brief      Ranges supported by TPS6522x Burton:
 *              - Range 1: 0x0 to 0x1A
 *              - Range 2: 0x1B to 0x45
 *
 *  @{
 */
#define TPS6522X_BUCK2_3_4_VOLT_RANGE_1_MIN_VSET    (0x0U)
#define TPS6522X_BUCK2_3_4_VOLT_RANGE_1_MAX_VSET    (0x1AU)
#define TPS6522X_BUCK2_3_4_VOLT_RANGE_2_MIN_VSET    (0x1BU)
#define TPS6522X_BUCK2_3_4_VOLT_RANGE_2_MAX_VSET    (0x45U)
/** @} */

/**
 *  \anchor     Tps6522x_ldo1VoltageRangesMV
 *  \name       LDO 1 voltage ranges in mV supported by TPS6522x Burton
 *
 *  \brief      Ranges supported by TPS6522x Burton:
 *              - Range 1: 1200 mV to 1200 mV in 0 mV steps
 *              - Range 2: 1250 mV to 3250 mV in 50 mV steps
 *              - Range 3: 3300 mV to 3300 mV in 0 mV steps
 *
 *  @{
 */
#define TPS6522X_LDO1_VOLT_RANGE_1_MIN_VOLT      (1200U)
#define TPS6522X_LDO1_VOLT_RANGE_1_MAX_VOLT      (1200U)
#define TPS6522X_LDO1_VOLT_RANGE_2_MIN_VOLT      (1250U)
#define TPS6522X_LDO1_VOLT_RANGE_2_MAX_VOLT      (3250U)
#define TPS6522X_LDO1_VOLT_RANGE_3_MIN_VOLT      (3300U)
#define TPS6522X_LDO1_VOLT_RANGE_3_MAX_VOLT      (3300U)
/** @} */

/**
 *  \anchor     Tps6522x_ldo1VoltageRangesVset
 *  \name       LDO 1 voltage ranges in VSET supported by TPS6522x Burton
 *
 *  \brief      Ranges supported by TPS6522x Burton:
 *              - Range 1: 0x0 to 0xC
 *              - Range 2: 0xD to 0x35
 *              - Range 3: 0x36 to 0x3F
 *
 *  @{
 */
#define TPS6522X_LDO1_VOLT_RANGE_1_MIN_VSET         (0x0U)
#define TPS6522X_LDO1_VOLT_RANGE_1_MAX_VSET         (0xCU)
#define TPS6522X_LDO1_VOLT_RANGE_2_MIN_VSET         (0xDU)
#define TPS6522X_LDO1_VOLT_RANGE_2_MAX_VSET         (0x35U)
#define TPS6522X_LDO1_VOLT_RANGE_3_MIN_VSET         (0x36U)
#define TPS6522X_LDO1_VOLT_RANGE_3_MAX_VSET         (0x3FU)
/** @} */

/**
 *  \anchor     Tps6522x_ldo2_3_VoltageRangesMV
 *  \name       LDO 2, 3 voltage ranges in mV supported by TPS6522x Burton
 *
 *  \brief      Ranges supported by TPS6522x Burton:
 *              - Range 1: 600 mV to 3350 mV in 50 mV steps
 *              - Range 2: 3400 mV to 3400 mV in 0 mV steps
 *
 *  @{
 */
#define TPS6522X_LDO2_3_VOLT_RANGE_1_MIN_VOLT    (600U)
#define TPS6522X_LDO2_3_VOLT_RANGE_1_MAX_VOLT    (3350U)
#define TPS6522X_LDO2_3_VOLT_RANGE_2_MIN_VOLT    (3400U)
#define TPS6522X_LDO2_3_VOLT_RANGE_2_MAX_VOLT    (3400U)
/** @} */

/**
 *  \anchor     Tps6522x_ldo2_3_VoltageRangesVset
 *  \name       LDO 2, 3 volage ranges in VSET supported by TPS6522x Burton
 *
 *  \brief      Ranges supported by TPS6522x Burton:
 *              - Range 1: 0x0 to 0x37
 *              - Range 2: 0x38 to 0x3F
 *
 *  @{
 */
#define TPS6522X_LDO2_3_VOLT_RANGE_1_MIN_VSET       (0x0U)
#define TPS6522X_LDO2_3_VOLT_RANGE_1_MAX_VSET       (0x37U)
#define TPS6522X_LDO2_3_VOLT_RANGE_2_MIN_VSET       (0x38U)
#define TPS6522X_LDO2_3_VOLT_RANGE_2_MAX_VSET       (0x3FU)
/** @} */

/**
 * \anchor      Tps6522x_vmon1VoltageRangesMV
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
#define TPS6522X_VMON1_VOLT_RANGE_1_MIN_VOLT     (500U)
#define TPS6522X_VMON1_VOLT_RANGE_1_MAX_VOLT     (580U)
#define TPS6522X_VMON1_VOLT_RANGE_2_MIN_VOLT     (600U)
#define TPS6522X_VMON1_VOLT_RANGE_2_MAX_VOLT     (1095U)
#define TPS6522X_VMON1_VOLT_RANGE_3_MIN_VOLT     (1100U)
#define TPS6522X_VMON1_VOLT_RANGE_3_MAX_VOLT     (1650U)
#define TPS6522X_VMON1_VOLT_RANGE_4_MIN_VOLT     (1660U)
#define TPS6522X_VMON1_VOLT_RANGE_4_MAX_VOLT     (3340U)
/** @} */

/**
 *  \anchor     Tps6522x_vmon1VoltageRangesVset
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
#define TPS6522X_VMON1_VOLT_RANGE_1_MIN_VSET        (0xAU)
#define TPS6522X_VMON1_VOLT_RANGE_1_MAX_VSET        (0xEU)
#define TPS6522X_VMON1_VOLT_RANGE_2_MIN_VSET        (0xFU)
#define TPS6522X_VMON1_VOLT_RANGE_2_MAX_VSET        (0x72U)
#define TPS6522X_VMON1_VOLT_RANGE_3_MIN_VSET        (0x73U)
#define TPS6522X_VMON1_VOLT_RANGE_3_MAX_VSET        (0xAAU)
#define TPS6522X_VMON1_VOLT_RANGE_4_MIN_VSET        (0xABU)
#define TPS6522X_VMON1_VOLT_RANGE_4_MAX_VSET        (0xFFU)
/** @} */

/**
 *  \anchor     Tps6522x_vmon2VoltageRangesMV
 *  \name       VMON 2 voltage ranges in mV supported by TPS6522x Burton
 *
 *  \brief      Ranges supported by TPS6522x Burton:
 *              - Range 1: 500 mV to 1150 mV in 25 mV steps
 *              - Range 2: 1200 mV to 3300 mV in 50 mV steps
 *
 *  @{
 */
#define TPS6522X_VMON2_VOLT_RANGE_1_MIN_VOLT     (500U)
#define TPS6522X_VMON2_VOLT_RANGE_1_MAX_VOLT     (1150U)
#define TPS6522X_VMON2_VOLT_RANGE_2_MIN_VOLT     (1200U)
#define TPS6522X_VMON2_VOLT_RANGE_2_MAX_VOLT     (3300U)
/** @} */

/**
 *  \anchor     Tps6522x_vmon2VoltageRangesVset
 *  \name       VMON 2 voltage ranges in VSET supported by TPS6522x Burton
 *
 *  \brief      Ranges supported by TPS6522x Burton:
 *              - Range 1: 0x0 to 0x1A
 *              - Range 2: 0x1B to 0x45
 *
 *  @{
 */
#define TPS6522X_VMON2_VOLT_RANGE_1_MIN_VSET        (0x0U)
#define TPS6522X_VMON2_VOLT_RANGE_1_MAX_VSET        (0x1AU)
#define TPS6522X_VMON2_VOLT_RANGE_2_MIN_VSET        (0x1BU)
#define TPS6522X_VMON2_VOLT_RANGE_2_MAX_VSET        (0x45U)
/** @} */

/**
 *  \anchor     Tps6522x_PwrRsrcMaxMinVoltageMv
 *  \name       Power Resource Min and Max Voltage Levels (mV) for TPS6522x
 *
 *  \brief      BUCK, LDO, VMON max and min voltage levels in mV supported by TPS6522x Burton
 *
 *  @{
 */
#define TPS6522X_BUCK1_VOLTAGE_MIN_500_MV              (500U)
#define TPS6522X_BUCK1_VOLTAGE_MAX_3300_MV             (3300U)
#define TPS6522X_BUCK2_3_4_VOLTAGE_MIN_500_MV          (500U)
#define TPS6522X_BUCK2_3_4_VOLTAGE_MAX_3300_MV         (3300U)
#define TPS6522X_LDO1_VOLTAGE_MIN_1200_MV              (1200U)
#define TPS6522X_LDO1_VOLTAGE_MAX_3300_MV              (3300U)
#define TPS6522X_LDO2_3_VOLTAGE_MIN_600_MV             (600U)
#define TPS6522X_LDO2_3_VOLTAGE_MAX_3400_MV            (3400U)
#define TPS6522X_VMON1_VOLTAGE_MIN_500_MV              (500U)
#define TPS6522X_VMON1_VOLTAGE_MAX_3340_MV             (3340U)
#define TPS6522X_VMON2_VOLTAGE_MIN_500_MV              (500U)
#define TPS6522X_VMON2_VOLTAGE_MAX_3300_MV             (3300U)
/** @} */

/**
 *  \anchor     Tps6522x_buckCtrlRegShiftVal
 *  \name       BUCK_CTRL register shift values supported by TPS6522x Burton
 *
 *  @{
 */
#define TPS6522X_BUCK_CTRL_PLDN_SHIFT                  (0x5U)
#define TPS6522X_BUCK_CTRL_VMON_EN_SHIFT               (0x4U)
#define TPS6522X_BUCK_CTRL_PWM_OPTION_SHIFT            (0x1U)
#define TPS6522X_BUCK_CTRL_EN_SHIFT                    (0x0U)
/** @} */

/**
 *  \anchor     Tps6522x_buckCtrlRegMaskVal
 *  \name       BUCK_CTRL register mask values supported by TPS6522x Burton
 *
 *  @{
 */
#define TPS6522X_BUCK_CTRL_PLDN_MASK                   (0x1U << TPS6522X_BUCK_CTRL_PLDN_SHIFT)
#define TPS6522X_BUCK_CTRL_VMON_EN_MASK                (0x1U << TPS6522X_BUCK_CTRL_VMON_EN_SHIFT)
#define TPS6522X_BUCK_CTRL_PWM_OPTION_MASK             (0x1U << TPS6522X_BUCK_CTRL_PWM_OPTION_SHIFT)
#define TPS6522X_BUCK_CTRL_EN_MASK                     (0x1U << TPS6522X_BUCK_CTRL_EN_SHIFT)
/** @} */

/**
 *  \anchor     Tps6522x_buckConfRegShiftVal
 *  \name       BUCK_CONF register shift values supported by TPS6522x Burton
 */
#define TPS6522X_BUCK_CONF_SLEW_RATE_SHIFT             (0x0U)

/**
 *  \anchor      Tps6522x_buckConfRegMaskVal
 *  \name        BUCK_CONF register mask values supported by TPS6522x Burton
 */
#define TPS6522X_BUCK_CONF_SLEW_RATE_MASK              (0x3U << TPS6522X_BUCK_CONF_SLEW_RATE_SHIFT)

/**
 *  \anchor     Tps6522x_buckPgWindowRegShiftVal
 *  \name       BUCK_PG_WINDOW register shift values supported by TPS6522x Burton
 */
#define TPS6522X_BUCK_PG_WINDOW_VMON_THR_SHIFT         (0x0U)

/**
 *  \anchor     Tps6522x_buckPgWindowRegMaskVal
 *  \name       BUCK_PG_WINDOW register mask values supported by TPS6522x Burton
 */
#define TPS6522X_BUCK_PG_WINDOW_VMON_THR_MASK          (0x3U << TPS6522X_BUCK_PG_WINDOW_VMON_THR_SHIFT)

/**
 *  \anchor     Tps6522x_railSel1RegShiftVal
 *  \name       RAIL_SEL_1 register shift values supported by TPS6522x Burton
 *
 *  @{
 */
#define TPS6522X_RAIL_SEL_1_BUCK4_GRP_SEL_SHIFT    (6U)
#define TPS6522X_RAIL_SEL_1_BUCK3_GRP_SEL_SHIFT    (4U)
#define TPS6522X_RAIL_SEL_1_BUCK2_GRP_SEL_SHIFT    (2U)
#define TPS6522X_RAIL_SEL_1_BUCK1_GRP_SEL_SHIFT    (0U)
/** @} */

/**
 *  \anchor     Tps6522x_railSel1RegMaskVal
 *  \name       RAIL_SEL_1 register mask values supported by TPS6522x Burton
 *
 *  @{
 */
#define TPS6522X_RAIL_SEL_1_BUCK4_GRP_SEL_MASK                                                          \
    (3U << TPS6522X_RAIL_SEL_1_BUCK4_GRP_SEL_SHIFT)
#define TPS6522X_RAIL_SEL_1_BUCK3_GRP_SEL_MASK                                                          \
    (3U << TPS6522X_RAIL_SEL_1_BUCK3_GRP_SEL_SHIFT)
#define TPS6522X_RAIL_SEL_1_BUCK2_GRP_SEL_MASK                                                          \
    (3U << TPS6522X_RAIL_SEL_1_BUCK2_GRP_SEL_SHIFT)
#define TPS6522X_RAIL_SEL_1_BUCK1_GRP_SEL_MASK                                                          \
    (3U << TPS6522X_RAIL_SEL_1_BUCK1_GRP_SEL_SHIFT)
/** @} */

/**
 *  \anchor     Tps6522x_ldoCtrlRegShiftVal
 *  \name       LDO_CTRL register shift values supported by TPS6522x Burton
 *
 *  @{
 */
#define TPS6522X_LDO_CTRL_DISCHARGE_EN_SHIFT      (0x5U)
#define TPS6522X_LDO_CTRL_VMON_EN_SHIFT           (0x4U)
#define TPS6522X_LDO_CTRL_EN_SHIFT                (0x0U)
/** @} */

/**
 *  \anchor     Tps6522x_ldoCtrlRegMaskVal
 *  \name       LDO_CTRL register mask values supported by TPS6522x Burton
 *
 *  @{
 */
#define TPS6522X_LDO_CTRL_DISCHARGE_EN_MASK       (0x1U << TPS6522X_LDO_CTRL_DISCHARGE_EN_SHIFT)
#define TPS6522X_LDO_CTRL_VMON_EN_MASK            (0x1U << TPS6522X_LDO_CTRL_VMON_EN_SHIFT)
#define TPS6522X_LDO_CTRL_EN_MASK                 (0x1U << TPS6522X_LDO_CTRL_EN_SHIFT)
/** @} */

/**
 *  \anchor     Tps6522x_ldoVoutRegShiftVal
 *  \name       LDO_VOUT register shift values supported by TPS6522x Burton
 *
 *  @{
 */
#define TPS6522X_LDO_VOUT_LDO_MODE_SHIFT          (0x7U)
#define TPS6522X_LDO_VOUT_LDO_VSET_SHIFT          (0x1U)
/** @} */

/**
 *  \anchor     Tps6522x_ldoVoutRegMaskVal
 *  \name       LDO_VOUT register mask values supported by TPS6522x Burton
 */
#define TPS6522X_LDO_VOUT_LDO_MODE_MASK           (0x1U << TPS6522X_LDO_VOUT_LDO_MODE_SHIFT)
#define TPS6522X_LDO_VOUT_LDO_VSET_MASK           (0x3FU << TPS6522X_LDO_VOUT_LDO_VSET_SHIFT)
/** @} */

/**
 *  \anchor     Tps6522x_ldoPgWindowRegShiftVal
 *  \name       LDO_PG_WINDOW register shift values supported by TPS6522x Burton
 */
#define TPS6522X_LDO_PG_WINDOW_LDO_VMON_THR_SHIFT (0x0U)

/**
 *  \anchor     Tps6522x_ldoPgWindowRegMaskVal
 *  \name       LDO_PG_WINDOW register mask values supported by TPS6522x Burton
 */
#define TPS6522X_LDO_PG_WINDOW_LDO_VMON_THR_MASK                                                            \
    (0x3U << TPS6522X_LDO_PG_WINDOW_LDO_VMON_THR_SHIFT)

/**
 *  \anchor     Tps6522x_railSel2RegShiftVal
 *  \name       RAIL_SEL_2 register shift values supported by TPS6522x Burton
 *
 *  @{
 */
#define TPS6522X_RAIL_SEL_2_LDO3_GRP_SEL_SHIFT     (0x6U)
#define TPS6522X_RAIL_SEL_2_LDO2_GRP_SEL_SHIFT     (0x4U)
#define TPS6522X_RAIL_SEL_2_LDO1_GRP_SEL_SHIFT     (0x2U)
/** @} */

/**
 *  \anchor     Tps6522x_railSel2RegMaskVal
 *  \name       RAIL_SEL_2 register mask values supported by TPS6522x Burton
 *
 *  @{
 */
#define TPS6522X_RAIL_SEL_2_LDO3_GRP_SEL_MASK      (0x3U << TPS6522X_RAIL_SEL_2_LDO3_GRP_SEL_SHIFT)
#define TPS6522X_RAIL_SEL_2_LDO2_GRP_SEL_MASK      (0x3U << TPS6522X_RAIL_SEL_2_LDO2_GRP_SEL_SHIFT)
#define TPS6522X_RAIL_SEL_2_LDO1_GRP_SEL_MASK      (0x3U << TPS6522X_RAIL_SEL_2_LDO1_GRP_SEL_SHIFT)
/** @} */

/**
 *  \anchor     Tps6522x_vccaVmonCtrlRegShiftVal
 *  \name       VCCA_VMON_CTRL register shift values supported by TPS6522x Burton
 *
 *  @{
 */
#define TPS6522X_VCCA_VMON_CTRL_DEGLITCH_SEL_SHIFT (5U)
#define TPS6522X_VCCA_VMON_CTRL_VMON2_EN_SHIFT     (3U)
#define TPS6522X_VCCA_VMON_CTRL_VMON1_EN_SHIFT     (1U)
#define TPS6522X_VCCA_VMON_CTRL_VCCA_VMON_EN_SHIFT (0U)
/** @} */

/**
 *  \anchor     Tps6522x_vccaVmonCtrlRegMaskVal
 *  \name       VCCA_VMON_CTRL register mask values supported by TPS6522x Burton
 *
 *  @{
 */
#define TPS6522X_VCCA_VMON_CTRL_DEGLITCH_SEL_MASK                                                           \
    (7U << TPS6522X_VCCA_VMON_CTRL_DEGLITCH_SEL_SHIFT)
#define TPS6522X_VCCA_VMON_CTRL_VMON2_EN_MASK (1U << TPS6522X_VCCA_VMON_CTRL_VMON2_EN_SHIFT)
#define TPS6522X_VCCA_VMON_CTRL_VMON1_EN_MASK (1U << TPS6522X_VCCA_VMON_CTRL_VMON1_EN_SHIFT)
#define TPS6522X_VCCA_VMON_CTRL_VCCA_VMON_EN_MASK                                                           \
    (1U << TPS6522X_VCCA_VMON_CTRL_VCCA_VMON_EN_SHIFT)
/** @} */

/**
 *  \anchor     Tps6522x_vccaPgWindowRegShiftVal
 *  \name       VCCA_PG_WINDOW register shift values supported by TPS6522x Burton
 *
 *  @{
 */
#define TPS6522X_VCCA_PG_WINDOW_VCCA_PG_SET_SHIFT   (6U)
#define TPS6522X_VCCA_PG_WINDOW_VCCA_VMON_THR_SHIFT (0U)
/** @} */

/**
 *  \anchor     Tps6522x_vccaPgWindowRegMaskVal
 *  \name       VCCA_PG_WINDOW register mask values supported by TPS6522x Burton
 *
 *  @{
 */
#define TPS6522X_VCCA_PG_WINDOW_VCCA_PG_SET_MASK    (1U << TPS6522X_VCCA_PG_WINDOW_VCCA_PG_SET_SHIFT)
#define TPS6522X_VCCA_PG_WINDOW_VCCA_VMON_THR_MASK                                                          \
    (3U << TPS6522X_VCCA_PG_WINDOW_VCCA_VMON_THR_SHIFT)
/** @} */

/**
 *  \anchor     Tps6522x_vmonPgWindowRegShiftVal
 *  \name       VMON_PG_WINDOW register shift values supported by TPS6522x Burton
 *
 *  @{
 */
#define TPS6522X_VMON1_PG_WINDOW_VMON1_THR_SHIFT   (0U)
#define TPS6522X_VMON2_PG_WINDOW_VMON2_THR_SHIFT   (0U)
/** @} */

/**
 *  \anchor     Tps6522x_vmonPgWindowRegMaskVal
 *  \name       VMON_PG_WINDOW register mask values supported by TPS6522x Burton
 *
 *  @{
 */
#define TPS6522X_VMON1_PG_WINDOW_VMON1_THR_MASK    (3U << TPS6522X_VMON1_PG_WINDOW_VMON1_THR_SHIFT)
#define TPS6522X_VMON2_PG_WINDOW_VMON2_THR_MASK    (3U << TPS6522X_VMON2_PG_WINDOW_VMON2_THR_SHIFT)
/** @} */

/**
 *  \anchor     Tps6522x_vmonPgLevelRegShiftVal
 *  \name       VMON_PG_LEVEL register shift values supported by TPS6522x Burton
 *
 *  @{
 */
#define TPS6522X_VMON1_PG_LEVEL_VMON1_PG_SET_SHIFT (0U)
#define TPS6522X_VMON2_PG_LEVEL_VMON2_PG_SET_SHIFT (0U)
/** @} */

/**
 *  \anchor     Tps6522x_vmonPgLevelRegMaskVal
 *  \name       VMON_PG_LEVEL register mask values supported by TPS6522x Burton
 *
 *  @{
 */
#define TPS6522X_VMON1_PG_LEVEL_VMON1_PG_SET_MASK                                                           \
    (0xFFU << TPS6522X_VMON1_PG_LEVEL_VMON1_PG_SET_SHIFT)
#define TPS6522X_VMON2_PG_LEVEL_VMON2_PG_SET_MASK                                                           \
    (0x7FU << TPS6522X_VMON2_PG_LEVEL_VMON2_PG_SET_SHIFT)
/** @} */

/**
 *  \anchor     Tps6522x_railSel3RegShiftVal
 *  \name       RAIL_SEL_3 register shift values supported by TPS6522x Burton
 *
 *  @{
 */
#define TPS6522X_RAIL_SEL_3_VMON2_GRP_SEL_SHIFT      (6U)
#define TPS6522X_RAIL_SEL_3_VMON1_GRP_SEL_SHIFT      (4U)
#define TPS6522X_RAIL_SEL_3_VCCA_GRP_SEL_SHIFT       (2U)
/** @} */

/**
 *  \anchor     Tps6522x_railSel3RegMaskVal
 *  \name       RAIL_SEL_3 register mask values supported by TPS6522x Burton
 *
 *  @{
 */
#define TPS6522X_RAIL_SEL_3_VMON2_GRP_SEL_MASK       (3U << TPS6522X_RAIL_SEL_3_VMON2_GRP_SEL_SHIFT)
#define TPS6522X_RAIL_SEL_3_VMON1_GRP_SEL_MASK       (3U << TPS6522X_RAIL_SEL_3_VMON1_GRP_SEL_SHIFT)
#define TPS6522X_RAIL_SEL_3_VCCA_GRP_SEL_MASK        (3U << TPS6522X_RAIL_SEL_3_VCCA_GRP_SEL_SHIFT)
/** @} */

/**
 *  \anchor     Tps6522x_statBuckRegShiftVal
 *  \name       STAT_BUCK register shift values supported by TPS6522x Burton
 *
 *  @{
 */
#define TPS6522X_STAT_BUCK_BUCK4_UVOV_STAT_SHIFT     (3U)
#define TPS6522X_STAT_BUCK_BUCK3_UVOV_STAT_SHIFT     (2U)
#define TPS6522X_STAT_BUCK_BUCK2_UVOV_STAT_SHIFT     (1U)
#define TPS6522X_STAT_BUCK_BUCK1_UVOV_STAT_SHIFT     (0U)
/** @} */

/**
 *  \anchor     Tps6522x_statBuckRegMaskVal
 *  \name       STAT_BUCK register mask values supported by TPS6522x Burton
 *
 *  @{
 */
#define TPS6522X_STAT_BUCK_BUCK4_UVOV_STAT_MASK      (1U << TPS6522X_STAT_BUCK_BUCK4_UVOV_STAT_SHIFT)
#define TPS6522X_STAT_BUCK_BUCK3_UVOV_STAT_MASK      (1U << TPS6522X_STAT_BUCK_BUCK3_UVOV_STAT_SHIFT)
#define TPS6522X_STAT_BUCK_BUCK2_UVOV_STAT_MASK      (1U << TPS6522X_STAT_BUCK_BUCK2_UVOV_STAT_SHIFT)
#define TPS6522X_STAT_BUCK_BUCK1_UVOV_STAT_MASK      (1U << TPS6522X_STAT_BUCK_BUCK1_UVOV_STAT_SHIFT)
/** @} */

/**
 *  \anchor     Tps6522x_statLdoVmonRegShiftVal
 *  \name       STAT_LDO_VMON register shift values supported by TPS6522x Burton
 *
 *  @{
 */
#define TPS6522X_STAT_LDO_VMON_VMON2_UVOV_STAT_SHIFT (6U)
#define TPS6522X_STAT_LDO_VMON_VMON1_UVOV_STAT_SHIFT (5U)
#define TPS6522X_STAT_LDO_VMON_VCCA_UVOV_STAT_SHIFT  (4U)
#define TPS6522X_STAT_LDO_VMON_LDO3_UVOV_STAT_SHIFT  (2U)
#define TPS6522X_STAT_LDO_VMON_LDO2_UVOV_STAT_SHIFT  (1U)
#define TPS6522X_STAT_LDO_VMON_LDO1_UVOV_STAT_SHIFT  (0U)
/** @} */

/**
 *  \anchor     Tps6522x_statLdoVmonRegMaskVal
 *  \name       STAT_LDO_VMON register mask values supported by TPS6522x Burton
 *
 *  @{
 */
#define TPS6522X_STAT_LDO_VMON_VMON2_UVOV_STAT_MASK          \
    (1U << TPS6522X_STAT_LDO_VMON_VMON2_UVOV_STAT_SHIFT)
#define TPS6522X_STAT_LDO_VMON_VMON1_UVOV_STAT_MASK          \
    (1U << TPS6522X_STAT_LDO_VMON_VMON1_UVOV_STAT_SHIFT)
#define TPS6522X_STAT_LDO_VMON_VCCA_UVOV_STAT_MASK           \
    (1U << TPS6522X_STAT_LDO_VMON_VCCA_UVOV_STAT_SHIFT)
#define TPS6522X_STAT_LDO_VMON_LDO3_UVOV_STAT_MASK           \
    (1U << TPS6522X_STAT_LDO_VMON_LDO3_UVOV_STAT_SHIFT)
#define TPS6522X_STAT_LDO_VMON_LDO2_UVOV_STAT_MASK           \
    (1U << TPS6522X_STAT_LDO_VMON_LDO2_UVOV_STAT_SHIFT)
#define TPS6522X_STAT_LDO_VMON_LDO1_UVOV_STAT_MASK           \
    (1U << TPS6522X_STAT_LDO_VMON_LDO1_UVOV_STAT_SHIFT)
/** @} */

/**
 *  \anchor     Tps6522x_thermalStatShiftVal
 *  \name       Register shift values for thermal status supported by TPS6522x Burton
 *
 *  @{
 */
#define TPS6522X_TWARN_STAT_SHIFT                   (3U)
#define TPS6522X_TSD_ORD_STAT_SHIFT                 (0U)
#define TPS6522X_TSD_IMM_STAT_SHIFT                 (0U)
/** @} */

/**
 *  \anchor     Tps6522x_thermalStatMaskVal
 *  \name       Register mask values for thermal status supported by TPS6522x Burton
 *
 *  @{
 */
#define TPS6522X_TWARN_STAT_MASK                    (1U << TPS6522X_TWARN_STAT_SHIFT)
#define TPS6522X_TSD_ORD_STAT_MASK                  (1U << TPS6522X_TSD_ORD_STAT_SHIFT)
#define TPS6522X_TSD_IMM_STAT_MASK                  (1U << TPS6522X_TSD_IMM_STAT_SHIFT)
/** @} */

/**
 *  \anchor     Tps6522x_thermalCfgShiftVal
 *  \name       Register shift values for thermal configuration supported by TPS6522x Burton
 *
 *  @{
 */
#define TPS6522X_TSD_ORD_LEVEL_SHIFT                (1U)
#define TPS6522X_TWARN_LEVEL_SHIFT                  (0U)
/** @} */

/**
 *  \anchor     Tps6522x_thermalCfgMaskVal
 *  \name       Register mask values for thermal configuration supported by TPS6522x Burton
 *
 *  @{
 */
#define TPS6522X_TWARN_LEVEL_MASK                   (1U << TPS6522X_TWARN_LEVEL_SHIFT)
#define TPS6522X_TSD_ORD_LEVEL_MASK                 (1U << TPS6522X_TSD_ORD_LEVEL_SHIFT)
/** @} */

/*!
 *  \anchor     Tps6522x_powerRegAddr
 *  \brief      Burton power register addresses for internal use
 */
#define TPS6522X_INVALID_REGADDR         (0x00U)
#define TPS6522X_VCCA_VMON_CTRL_REGADDR  (0x2BU)
#define TPS6522X_VCCA_PG_WINDOW_REGADDR  (0x2CU)
#define TPS6522X_VMON1_PG_WINDOW_REGADDR (0x2DU)
#define TPS6522X_VMON1_PG_LEVEL_REGADDR  (0x2EU)
#define TPS6522X_VMON2_PG_WINDOW_REGADDR (0x2FU)
#define TPS6522X_VMON2_PG_LEVEL_REGADDR  (0x30U)
#define TPS6522X_RAIL_SEL_3_REGADDR      (0x43U)
#define TPS6522X_STAT_BUCK_REGADDR       (0x6DU)
#define TPS6522X_STAT_LDO_VMON_REGADDR   (0x70U)

/* ========================================================================== */
/*                          Structures and Enums                              */
/* ========================================================================== */

/* ========================================================================== */
/*                         Function Declarations                              */
/* ========================================================================== */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* PMIC_POWER_TPS6522X_PRIV_H_ */
