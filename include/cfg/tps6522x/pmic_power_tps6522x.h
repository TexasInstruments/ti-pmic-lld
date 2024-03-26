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
 *  \anchor     Tps6522x_maxPwrRsrcNum
 *  \name       Maximum BUCKs, LDOs, and VMONs supported by TPS6522x Burton
 *
 *  @{
 */
#define TPS6522X_MAX_BUCK_NUM                          (4U)
#define TPS6522X_MAX_LDO_NUM                           (3U)
#define TPS6522X_MAX_VOLTAGE_MONITOR_NUM               (3U)
/** @} */

/**
 *  \anchor     Tps6522x_buckPwrRsrcNum
 *  \name       BUCK power resources supported by TPS6522x Burton
 *
 *  @{
 */
#define TPS6522X_REGULATOR_BUCK1                       (0U)
#define TPS6522X_REGULATOR_BUCK2                       (1U)
#define TPS6522X_REGULATOR_BUCK3                       (2U)
#define TPS6522X_REGULATOR_BUCK4                       (3U)
/** @} */

/**
 *  \anchor     Tps6522x_ldoPwrRsrcNum
 *  \name       LDO power resources supported by TPS6522x Burton
 *
 *  @{
 */
#define TPS6522X_REGULATOR_LDO1                        (0U)
#define TPS6522X_REGULATOR_LDO2                        (1U)
#define TPS6522X_REGULATOR_LDO3                        (2U)
/** @} */

/**
 *  \anchor     Tps6522x_vccaVmonPwrRsrcNum
 *  \name       Voltage monitor power resources supported by TPS6522x Burton
 *
 *  @{
 */
#define TPS6522X_VOLTAGE_MONITOR_VMON1                 (0U)
#define TPS6522X_VOLTAGE_MONITOR_VMON2                 (1U)
#define TPS6522X_VOLTAGE_MONITOR_VCCA_VMON             (2U)
/** @} */

/**
 *  \anchor     Tps6522x_buckLdoMinMaxVset
 *  \name       Minimum and maximum VSET values for TPS6522x Bucks and LDOs
 *
 *  @{
 */
#define TPS6522X_BUCK1_MIN_VSET                         (0xAU)
#define TPS6522X_BUCK1_MAX_VSET                         (0xFDU)
#define TPS6522X_BUCK2_3_4_MIN_VSET                     (0x0U)
#define TPS6522X_BUCK2_3_4_MAX_VSET                     (0x45U)
#define TPS6522X_LDO1_2_3_MIN_VSET                      (0x0U)
#define TPS6522X_LDO1_2_3_MAX_VSET                      (0x3FU)
/** @} */

/**
 *  \anchor     Tps6522x_vmonMinMaxPgSet
 *  \name       Minimum and maximum PG_SET values for TPS6522x VMON1 and VMON2 
 *
 *  @{
 */
#define TPS6522X_VMON1_MIN_PG_SET                       (0xAU)
#define TPS6522X_VMON1_MAX_PG_SET                       (0xFFU)
#define TPS6522X_VMON2_MIN_PG_SET                       (0x0U)
#define TPS6522X_VMON2_MAX_PG_SET                       (0x45U)
/** @} */

/**
 *  \anchor     Tps6522x_thermalCfgValidParam
 *  \name       Valid parameters of Thermal Configuration struct for TPS6522x Burton
 *
 *  @{
 */
#define TPS6522X_TWARN_LEVEL_VALID                  (0U)
#define TPS6522X_TSD_ORD_LEVEL_VALID                (1U)
/** @} */

/**
 *  \anchor     Tps6522x_thermalCfgValidParamShift
 *  \name       Valid parameter shift values of Thermal Configuration struct for TPS6522x Burton
 *
 *  \brief      Application can use these shift values to configure validParams of Thermal Configuration
 *              struct.
 *
 *  @{
 */
#define TPS6522X_TWARN_LEVEL_VALID_SHIFT            (1U << TPS6522X_TWARN_LEVEL_VALID)
#define TPS6522X_TSD_ORD_LEVEL_VALID_SHIFT          (1U << TPS6522X_TSD_ORD_LEVEL_VALID)
/** @} */

/**
 *  \anchor     Tps6522x_buckPwrRsrcCfgValidParam
 *  \name       Valid parameters of BUCK Power Resource Configuration struct for TPS6522x Burton
 *
 *  @{
 */
#define TPS6522X_BUCK_PLDN_VALID                (0U)
#define TPS6522X_BUCK_VMON_EN_VALID             (1U)
#define TPS6522X_BUCK_PWM_OPTION_VALID          (2U)
#define TPS6522X_BUCK_EN_VALID                  (3U)
#define TPS6522X_BUCK_SLEW_RATE_VALID           (4U)
#define TPS6522X_BUCK_VSET_VALID                (5U)
#define TPS6522X_BUCK_VMON_THR_VALID            (6U)
#define TPS6522X_BUCK_RAIL_GRP_SEL_VALID        (7U)
/** @} */

/**
 *  \anchor     Tps6522x_buckPwrRsrcCfgValidParamShift
 *  \name       Valid parameter shift values of BUCK Power Resource Configuration struct for TPS6522x Burton
 *
 *  \brief      Application can use these shift values to configure validParams of BUCK Power Resource Configuration
 *              struct.
 *
 *  @{
 */
#define TPS6522X_BUCK_PLDN_VALID_SHIFT          (1U << TPS6522X_BUCK_PLDN_VALID)
#define TPS6522X_BUCK_VMON_EN_VALID_SHIFT       (1U << TPS6522X_BUCK_VMON_EN_VALID)
#define TPS6522X_BUCK_PWM_OPTION_VALID_SHIFT    (1U << TPS6522X_BUCK_PWM_OPTION_VALID)
#define TPS6522X_BUCK_EN_VALID_SHIFT            (1U << TPS6522X_BUCK_EN_VALID)
#define TPS6522X_BUCK_SLEW_RATE_VALID_SHIFT     (1U << TPS6522X_BUCK_SLEW_RATE_VALID)
#define TPS6522X_BUCK_VSET_VALID_SHIFT          (1U << TPS6522X_BUCK_VSET_VALID)
#define TPS6522X_BUCK_VMON_THR_VALID_SHIFT      (1U << TPS6522X_BUCK_VMON_THR_VALID)
#define TPS6522X_BUCK_RAIL_GRP_SEL_VALID_SHIFT  (1U << TPS6522X_BUCK_RAIL_GRP_SEL_VALID)
/** @} */

/**
 *  \anchor     Tps6522x_ldoPwrRsrcCfgValidParam
 *  \name       Valid parameters of LDO Power Resource Configuration struct for TPS6522x Burton
 *
 *  @{
 */
#define TPS6522X_LDO_DISCHARGE_EN_VALID         (0U)
#define TPS6522X_LDO_VMON_EN_VALID              (1U)
#define TPS6522X_LDO_EN_VALID                   (2U)
#define TPS6522X_LDO_MODE_VALID                 (3U)
#define TPS6522X_LDO_VSET_VALID                 (4U)
#define TPS6522X_LDO_VMON_THR_VALID             (5U)
#define TPS6522X_LDO_RAIL_GRP_SEL_VALID         (6U)
/** @} */

/**
 * \anchor  Tps6522x_ldoPwrRsrcCfgValidParamShift
 * \name    Valid parameter shift values of LDO Power Resource Configuration struct for TPS6522x Burton
 *
 * \brief   Application can use these shift values to configure validParams of LDO Power Resource Configuration struct.
 *
 *  @{
 */
#define TPS6522X_LDO_DISCHARGE_EN_VALID_SHIFT   (1U << TPS6522X_LDO_DISCHARGE_EN_VALID)
#define TPS6522X_LDO_VMON_EN_VALID_SHIFT        (1U << TPS6522X_LDO_VMON_EN_VALID)
#define TPS6522X_LDO_EN_VALID_SHIFT             (1U << TPS6522X_LDO_EN_VALID)
#define TPS6522X_LDO_MODE_VALID_SHIFT           (1U << TPS6522X_LDO_MODE_VALID)
#define TPS6522X_LDO_VSET_VALID_SHIFT           (1U << TPS6522X_LDO_VSET_VALID)
#define TPS6522X_LDO_VMON_THR_VALID_SHIFT       (1U << TPS6522X_LDO_VMON_THR_VALID)
#define TPS6522X_LDO_RAIL_GRP_SEL_VALID_SHIFT   (1U << TPS6522X_LDO_RAIL_GRP_SEL_VALID)
/** @} */

/**
 *  \anchor     Tps6522x_vccaVmonPwrRsrcCfgValidParam
 *  \name       Valid parameters of VCCA_VMON Power Resource Configuration struct for TPS6522x Burton
 *
 *  @{
 */
#define TPS6522X_VMON_DEGLITCH_SEL_VALID        (0U)
#define TPS6522X_VMON2_EN_VALID                 (1U)
#define TPS6522X_VMON1_EN_VALID                 (2U)
#define TPS6522X_VCCA_VMON_EN_VALID             (3U)
#define TPS6522X_VCCA_PG_SET_VALID              (4U)
#define TPS6522X_VCCA_VMON_THR_VALID            (5U)
#define TPS6522X_VCCA_RAIL_GRP_SEL_VALID        (6U)
#define TPS6522X_VMON1_THR_VALID                (7U)
#define TPS6522X_VMON1_PG_SET_VALID             (8U)
#define TPS6522X_VMON1_RAIL_GRP_SEL_VALID       (9U)
#define TPS6522X_VMON2_THR_VALID                (10U)
#define TPS6522X_VMON2_PG_SET_VALID             (11U)
#define TPS6522X_VMON2_RAIL_GRP_SEL_VALID       (12U)
/** @} */

/**
 *  \anchor     Tps6522x_vccaVmonPwrRsrcCfgValidParamShift
 *  \name       Valid parameter shift values of VCCA_VMON Power Resource Configuration struct for TPS6522x Burton
 *
 *  \brief      Application can use these shift values to configure validParams of VCCA_VMON Power Resource
 *              Configuration struct.
 *
 *  @{
 */
#define TPS6522X_VMON_DEGLITCH_SEL_VALID_SHIFT  (1U << TPS6522X_VMON_DEGLITCH_SEL_VALID)
#define TPS6522X_VMON2_EN_VALID_SHIFT           (1U << TPS6522X_VMON2_EN_VALID)
#define TPS6522X_VMON1_EN_VALID_SHIFT           (1U << TPS6522X_VMON1_EN_VALID)
#define TPS6522X_VCCA_VMON_EN_VALID_SHIFT       (1U << TPS6522X_VCCA_VMON_EN_VALID)
#define TPS6522X_VCCA_PG_SET_VALID_SHIFT        (1U << TPS6522X_VCCA_PG_SET_VALID)
#define TPS6522X_VCCA_VMON_THR_VALID_SHIFT      (1U << TPS6522X_VCCA_VMON_THR_VALID)
#define TPS6522X_VCCA_RAIL_GRP_SEL_VALID_SHIFT  (1U << TPS6522X_VCCA_RAIL_GRP_SEL_VALID)
#define TPS6522X_VMON1_THR_VALID_SHIFT          (1U << TPS6522X_VMON1_THR_VALID)
#define TPS6522X_VMON1_PG_SET_VALID_SHIFT       (1U << TPS6522X_VMON1_PG_SET_VALID)
#define TPS6522X_VMON1_RAIL_GRP_SEL_VALID_SHIFT (1U << TPS6522X_VMON1_RAIL_GRP_SEL_VALID)
#define TPS6522X_VMON2_THR_VALID_SHIFT          (1U << TPS6522X_VMON2_THR_VALID)
#define TPS6522X_VMON2_PG_SET_VALID_SHIFT       (1U << TPS6522X_VMON2_PG_SET_VALID)
#define TPS6522X_VMON2_RAIL_GRP_SEL_VALID_SHIFT (1U << TPS6522X_VMON2_RAIL_GRP_SEL_VALID)
/** @} */

/**
 *  \anchor     Tps6522x_pwrRsrcCfgValidParam
 *  \name       Valid parameters of TPS6522x Power Resource Configuration struct
 *
 *  @{
 */
#define TPS6522X_BUCK1_VALID                    (0U)
#define TPS6522X_BUCK2_VALID                    (1U)
#define TPS6522X_BUCK3_VALID                    (2U)
#define TPS6522X_BUCK4_VALID                    (3U)
#define TPS6522X_LDO1_VALID                     (4U)
#define TPS6522X_LDO2_VALID                     (5U)
#define TPS6522X_LDO3_VALID                     (6U)
#define TPS6522X_VMON1_VALID                    (7U)
#define TPS6522X_VMON2_VALID                    (8U)
#define TPS6522X_VCCA_VALID                     (9U)
/** @} */

/**
 *  \anchor     Tps6522x_pwrRsrcCfgValidParamShift
 *  \name       Valid parameter shift values of TPS6522x Power Resource Configuration struct
 *
 *  \brief      Application can use these shift values to configure validParams of TPS6522x Power Resource
 *              Configuration struct.
 *
 *  @{
 */
#define TPS6522X_BUCK1_VALID_SHIFT              (1U << TPS6522X_BUCK1_VALID)
#define TPS6522X_BUCK2_VALID_SHIFT              (1U << TPS6522X_BUCK2_VALID)
#define TPS6522X_BUCK3_VALID_SHIFT              (1U << TPS6522X_BUCK3_VALID)
#define TPS6522X_BUCK4_VALID_SHIFT              (1U << TPS6522X_BUCK4_VALID)
#define TPS6522X_LDO1_VALID_SHIFT               (1U << TPS6522X_LDO1_VALID)
#define TPS6522X_LDO2_VALID_SHIFT               (1U << TPS6522X_LDO2_VALID)
#define TPS6522X_LDO3_VALID_SHIFT               (1U << TPS6522X_LDO3_VALID)
#define TPS6522X_VMON1_VALID_SHIFT              (1U << TPS6522X_VMON1_VALID)
#define TPS6522X_VMON2_VALID_SHIFT              (1U << TPS6522X_VMON2_VALID)
#define TPS6522X_VCCA_VALID_SHIFT               (1U << TPS6522X_VCCA_VALID)
/** @} */

/**
 *  \anchor     Tps6522x_pwrRsrcUVOVStatus
 *  \name       Power Resource UVOV Status Enumeration for TPS6522x Burton
 *
 *  \brief      Enumeration of all the possible UVOV statuses for power resources.
 *
 *  @{
 */
#define TPS6522X_BUCK1_UVOV_STAT (0U)
#define TPS6522X_BUCK2_UVOV_STAT (1U)
#define TPS6522X_BUCK3_UVOV_STAT (2U)
#define TPS6522X_BUCK4_UVOV_STAT (3U)
#define TPS6522X_LDO1_UVOV_STAT  (4U)
#define TPS6522X_LDO2_UVOV_STAT  (5U)
#define TPS6522X_LDO3_UVOV_STAT  (6U)
#define TPS6522X_VMON1_UVOV_STAT (7U)
#define TPS6522X_VMON2_UVOV_STAT (8U)
#define TPS6522X_VCCA_UVOV_STAT  (9U)
/** @} */

/**
 *  \anchor     Tps6522x_tsdOrdLvlBitField
 *  \name       Orderly Thermal Shutdown Level Bit Fields for TPS6522x Burton
 *
 *  \brief      Orderly TSD threshold configuration.
 *
 *  @{
 */
#define TPS6522X_TSD_ORD_LVL_140C (0U)
#define TPS6522X_TSD_ORD_LVL_145C (1U)
/** @} */

/**
 *  \anchor     Tps6522x_TwarnLvlBitField
 *  \name       Thermal Warning Level Bit Fields for TPS6522x Burton
 *
 *  \brief      Thermal warning threshold configuration.
 *
 *  @{
 */
#define TPS6522X_TWARN_LVL_130C (0U)
#define TPS6522X_TWARN_LVL_140C (1U)
/** @} */

/**
 *  \anchor     Tps6522x_buckPldnEnBitField
 *  \name       BUCK_PLDN Bit Fields for TPS6522x Burton
 *
 *  \brief      Enable/disable output pull-down resistor when BUCK is disabled.
 *
 *  @{
 */
#define TPS6522X_BUCK_PLDN_DISABLE (0U)
#define TPS6522X_BUCK_PLDN_ENABLE  (1U)
/** @} */

/**
 *  \anchor     Tps6522x_buckVmonEnBitField
 *  \name       BUCK_VMON_EN Bit Fields for TPS6522x Burton
 *
 *  \brief      Enable/disable BUCK OV and UV comparators.
 *
 *  @{
 */
#define TPS6522X_BUCK_VMON_DISABLE (0U)
#define TPS6522X_BUCK_VMON_ENABLE  (1U)
/** @} */

/**
 *  \anchor     Tps6522x_buckFPWMBitField
 *  \name       BUCK_FPWM Bit Fields for TPS6522x Burton
 *
 *  \brief      Choose between automatic transitions between PFM and PWM mode or forced PWM mode.
 *
 *  @{
 */
#define TPS6522X_BUCK_PWM_AUTO   (0U)
#define TPS6522X_BUCK_PWM_FORCED (1U)
/** @} */

/**
 * \anchor  Tps6522x_buckEnBitField
 * \name    BUCK_EN Bit Fields for TPS6522x Burton
 *
 * \brief   Enable/disable BUCK regulator.
 *
 *  @{
 */
#define TPS6522X_BUCK_DISABLE (0U)
#define TPS6522X_BUCK_ENABLE  (1U)
/** @} */

/**
 *  \anchor     Tps6522x_buckSlewRateBitField
 *  \name       BUCK_SLEW_RATE Bit Fields for TPS6522x Burton
 *
 *  \brief      BUCK ouput voltage slew rate (rising and falling edges).
 *
 *  @{
 */
#define TPS6522X_BUCK_SLEW_RATE_10_MV_PER_US   (0U)
#define TPS6522X_BUCK_SLEW_RATE_5_MV_PER_US    (1U)
#define TPS6522X_BUCK_SLEW_RATE_2_5_MV_PER_US  (2U)
#define TPS6522X_BUCK_SLEW_RATE_1_25_MV_PER_US (3U)
/** @} */

/**
 *  \anchor     Tps6522x_buckVmonThrBitField
 *  \name       BUCK_VMON_THR Bit Fields for TPS6522x Burton
 *
 *  \brief      BUCK powergood high/low threshold level.
 *
 *  @{
 */
#define TPS6522X_BUCK_VMON_THR_3_PCT_OR_30_MV (0U)
#define TPS6522X_BUCK_VMON_THR_4_PCT_OR_40_MV (1U)
#define TPS6522X_BUCK_VMON_THR_6_PCT_OR_60_MV (2U)
#define TPS6522X_BUCK_VMON_THR_8_PCT_OR_80_MV (3U)
/** @} */

/**
 *  \anchor     Tps6522x_buckRailSelBitField
 *  \name       BUCK_GRP_SEL Bit Fields for TPS6522x Burton
 *
 *  \brief      BUCK rail group selection.
 *
 *  @{
 */
#define TPS6522X_BUCK_RAIL_SEL_NONE  (0U)
#define TPS6522X_BUCK_RAIL_SEL_MCU   (1U)
#define TPS6522X_BUCK_RAIL_SEL_SOC   (2U)
#define TPS6522X_BUCK_RAIL_SEL_OTHER (3U)
/** @} */

/**
 *  \anchor     Tps6522x_ldoDischargeEnBitField
 *  \name       LDO_DISCHARGE_EN Bit Fields for TPS6522x Burton
 *
 *  \brief      LDO discharge enable setting.
 *
 *  @{
 */
#define TPS6522X_LDO_DISCHARGE_DISABLE (0U)
#define TPS6522X_LDO_DISCHARGE_ENABLE  (1U)
/** @} */

/**
 *  \anchor     Tps6522x_ldoVmonEnBitField
 *  \name       LDO_VMON_EN Bit Fields for TPS6522x Burton
 *
 *  \brief      LDO Enable/Disable OV and UV comparators.
 *
 *  @{
 */
#define TPS6522X_LDO_VMON_DISABLE (0U)
#define TPS6522X_LDO_VMON_ENABLE  (1U)
/** @} */

/**
 *  \anchor     Tps6522x_ldoEnBitField
 *  \name       LDO_EN Bit Fields for TPS6522x Burton
 *
 *  \brief      LDO Enable/disable regulator.
 *
 *  @{
 */
#define TPS6522X_LDO_DISABLE (0U)
#define TPS6522X_LDO_ENABLE  (1U)
/** @} */

/**
 *  \anchor     Tps6522x_bypConfigBitField
 *  \name       LDO_BYP_CONFIG Bit Fields for TPS6522x Burton
 *
 *  \brief      LDO modes of operation.
 *
 *  @{
 */
#define TPS6522X_LDO_BYP_CONFIG_LDO_MODE    (0U)
#define TPS6522X_LDO_BYP_CONFIG_BYPASS_MODE (1U)
/** @} */

/**
 *  \anchor     Tps6522x_ldoVmonThrBitField
 *  \name       LDO_VMON_THR Bit Fields for TPS6522x Burton
 *
 *  \brief      LDO powergood high/low threshold level for LDO.
 *
 *  @{
 */
#define TPS6522X_LDO_VMON_THR_3_PCT (0U)
#define TPS6522X_LDO_VMON_THR_4_PCT (1U)
#define TPS6522X_LDO_VMON_THR_6_PCT (2U)
#define TPS6522X_LDO_VMON_THR_8_PCT (3U)
/** @} */

/**
 *  \anchor     Tps6522x_ldoRailSelBitField
 *  \name       LDO_GRP_SEL Bit Fields for TPS6522x Burton
 *
 *  \brief      LDO rail group selection.
 *
 *  @{
 */
#define TPS6522X_LDO_RAIL_SEL_NONE  (0U)
#define TPS6522X_LDO_RAIL_SEL_MCU   (1U)
#define TPS6522X_LDO_RAIL_SEL_SOC   (2U)
#define TPS6522X_LDO_RAIL_SEL_OTHER (3U)
/** @} */

/**
 *  \anchor     Tps6522x_vmonDeglitchSelBitField
 *  \name       VMON_DEGLITCH_SEL Bit Fields for TPS6522x Burton
 *
 *  \brief      VMON deglitch time select for BUCKx_VMON, LDOx_VMON, VMONx, and VCCA_VMON.
 *
 *  @{
 */
#define TPS6522X_VMON_DEGLITCH_SEL_BUCK_LDO_VMON_4_US_VCCA_4_US     (0U)
#define TPS6522X_VMON_DEGLITCH_SEL_BUCK_LDO_VMON_20_US_VCCA_20_US   (1U)
#define TPS6522X_VMON_DEGLITCH_SEL_BUCK_LDO_VMON_0_5_US_VCCA_0_5_US (2U)
#define TPS6522X_VMON_DEGLITCH_SEL_BUCK_LDO_VMON_0_5_US_VCCA_4_US   (3U)
#define TPS6522X_VMON_DEGLITCH_SEL_BUCK_LDO_VMON_0_5_US_VCCA_20_US  (4U)
#define TPS6522X_VMON_DEGLITCH_SEL_BUCK_LDO_VMON_4_US_VCCA_0_5_US   (5U)
#define TPS6522X_VMON_DEGLITCH_SEL_BUCK_LDO_VMON_4_US_VCCA_20_US    (6U)
/** @} */

/**
 *  \anchor     Tps6522x_vmon2EnBitField
 *  \name       VMON2_EN Bit Fields for TPS6522x Burton
 *
 *  \brief      VMON2 enable/disable.
 *
 *  @{
 */
#define TPS6522X_VMON2_DISABLE (0U)
#define TPS6522X_VMON2_ENABLE  (1U)
/** @} */

/**
 *  \anchor     Tps6522x_vmon1EnBitField
 *  \name       VMON1_EN Bit Fields for TPS6522x Burton
 *
 *  \brief      VMON1 enable/disable.
 *
 *  @{
 */
#define TPS6522X_VMON1_DISABLE (0U)
#define TPS6522X_VMON1_ENABLE  (1U)
/** @} */

/**
 *  \anchor     Tps6522x_vccaVmonEnBitField
 *  \name       VCCA_VMON_EN Bit Fields for TPS6522x Burton
 *
 *  \brief      VCCA_VMON enable/disable.
 *
 *  @{
 */
#define TPS6522X_VCCA_VMON_DISABLE (0U)
#define TPS6522X_VCCA_VMON_ENABLE  (1U)
/** @} */

/**
 *  \anchor     Tps6522x_vccaPgSetBitField
 *  \name       VCCA_PG_SET Bit Fields for TPS6522x Burton
 *
 *  \brief      VCCA_VMON powergood level.
 *
 *  @{
 */
#define TPS6522X_VCCA_PG_SET_3_3_V (0U)
#define TPS6522X_VCCA_PG_SET_5_0_V (1U)
/** @} */

/**
 *  \anchor     Tps6522x_vccaVmonThrBitField
 *  \name       VCCA_VMON_THR Bit Fields for TPS6522x Burton
 *
 *  \brief      VCCA_VMON powergood high/low threshold level.
 *
 *  @{
 */
#define TPS6522X_VCCA_VMON_THR_3_PCT  (0U)
#define TPS6522X_VCCA_VMON_THR_4_PCT  (1U)
#define TPS6522X_VCCA_VMON_THR_6_PCT  (2U)
#define TPS6522X_VCCA_VMON_THR_10_PCT (3U)
/** @} */

/**
 *  \anchor     Tps6522x_vccaRailSelBitField
 *  \name       VCCA_GRP_SEL Bit Fields for TPS6522x Burton
 *
 *  \brief      VCCA_VMON rail group selection.
 *
 *  @{
 */
#define TPS6522X_VCCA_RAIL_SEL_NONE  (0U)
#define TPS6522X_VCCA_RAIL_SEL_MCU   (1U)
#define TPS6522X_VCCA_RAIL_SEL_SOC   (2U)
#define TPS6522X_VCCA_RAIL_SEL_OTHER (3U)
/** @} */

/**
 *  \anchor     Tps6522x_vmon1ThrBitField
 *  \name       VMON1_THR Bit Fields for TPS6522x Burton
 *
 *  \brief      VMON1 powergood high/low threshold level for VMON.
 *
 *  @{
 */
#define TPS6522X_VMON1_THR_3_PCT_OR_30_MV (0U)
#define TPS6522X_VMON1_THR_4_PCT_OR_40_MV (1U)
#define TPS6522X_VMON1_THR_6_PCT_OR_60_MV (2U)
#define TPS6522X_VMON1_THR_8_PCT_OR_80_MV (3U)
/** @} */

/**
 *  \anchor     Tps6522x_vmon1RailSelBitField
 *  \name       VMON1_GRP_SEL Bit Fields for TPS6522x Burton
 *
 *  \brief      VMON1 rail group selection.
 *
 *  @{
 */
#define TPS6522X_VMON1_RAIL_SEL_NONE  (0U)
#define TPS6522X_VMON1_RAIL_SEL_MCU   (1U)
#define TPS6522X_VMON1_RAIL_SEL_SOC   (2U)
#define TPS6522X_VMON1_RAIL_SEL_OTHER (3U)
/** @} */

/**
 *  \anchor     Tps6522x_vmon2ThrBitField
 *  \name       VMON2_THR Bit Fields for TPS6522x Burton
 *
 *  \brief      VMON2 powergood high/low threshold level for VMON.
 *
 *  @{
 */
#define TPS6522X_VMON2_THR_3_PCT (0U)
#define TPS6522X_VMON2_THR_4_PCT (1U)
#define TPS6522X_VMON2_THR_6_PCT (2U)
#define TPS6522X_VMON2_THR_8_PCT (3U)
/** @} */

/**
 *  \anchor     Tps6522x_vmon2RailSelBitField
 *  \name       VMON2_GRP_SEL Bit Fields for TPS6522x Burton
 *
 *  \brief      VMON2 rail group selection.
 *
 *  @{
 */
#define TPS6522X_VMON2_RAIL_SEL_NONE  (0U)
#define TPS6522X_VMON2_RAIL_SEL_MCU   (1U)
#define TPS6522X_VMON2_RAIL_SEL_SOC   (2U)
#define TPS6522X_VMON2_RAIL_SEL_OTHER (3U)
/** @} */

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/**
 *  \anchor     Pmic_powerTps6522xBuckPowerResourceCfg_s
 *  \name       BUCK Power Configuration Struct for TPS6522x Burton
 *
 *  \brief      This struct holds information relevent to the BUCK power resource. 
 *
 *  \note       ValidParams is input param for all Set and Get APIs. Other
 *              params except validParams are input params for Set APIs and output
 *              params for Get APIs.
 *
 *  \param      validParams         Indication of which struct members are valid. That is 
 *                                  to say, each bit in the variable indicates whether or 
 *                                  not a struct member is valid. For valid values, 
 *                                  refer to \ref Tps6522x_buckPwrRsrcCfgValidParamShift
 *  \param      buckPldn            BUCK pull-down resistor enable. For valid values,
 *                                      refer to \ref Tps6522x_buckPldnEnBitField
 *  \param      buckVmonEn          BUCK VMON enable. For valid values, refer to
 *                                      \ref Tps6522x_buckVmonEnBitField
 *  \param      buckPwmOption       BUCK PWM selection. For valid values, refer to
 *                                      \ref Tps6522x_buckFPWMBitField
 *  \param      buckEn              BUCK enable. For valid values, refer to
 *                                      \ref Tps6522x_buckEnBitField
 *  \param      buckSlewRate        BUCK slew rate. For valid values, refer to
 *                                      \ref Tps6522x_buckSlewRateBitField
 *  \param      buckVset            BUCK voltage in VSET form
 *                                  For possible ranges of BUCK2, BUCK3, BUCK4, refer to
 *                                      \ref Tps6522x_buck2_3_4_VoltageRangesMV
 *  \param      buckVmonThr         BUCK VMON threshold. For valid values, refer to
 *                                      \ref Tps6522x_buckVmonThrBitField
 *  \param      buckRailGrpSel      BUCK rail group selection. For valid values, refer to
 *                                      \ref Tps6522x_buckRailSelBitField
 *
 *  @{
 */
typedef struct tps6522xBuckCfg_s
{
    uint8_t validParams;
    uint8_t buckPldn;
    uint8_t buckVmonEn;
    uint8_t buckPwmOption;
    uint8_t buckEn;
    uint8_t buckSlewRate;
    uint8_t buckVset;
    uint8_t buckVmonThr;
    uint8_t buckRailGrpSel;
} tps6522xBuckCfg_t;
/** @} */

/**
 *  \anchor     Pmic_powerTps6522xLdoPowerResourceCfg_s
 *  \name       LDO Power Configuration Struct for TPS6522x Burton
 *
 *  \brief      This struct holds information relevent to the LDO power resource.
 *
 *  \note       ValidParams is input param for all Set and Get APIs. Other
 *              params except validParams are input params for Set APIs and output
 *              params for Get APIs.
 *
 *  \param      validParams         Indication of which struct members are valid. More 
 *                                  precisely, each bit in the variable indicates whether 
 *                                  or not a struct member is valid. For valid values, 
 *                                  refer to \ref Tps6522x_ldoPwrRsrcCfgValidParamShift
 *  \param      ldoDischargeEn      LDO discharge enable. For valid values, refer to
 *                                      \ref Tps6522x_ldoDischargeEnBitField
 *  \param      ldoVmonEn           LDO VMON enable. For valid values, refer to
 *                                      \ref Tps6522x_ldoVmonEnBitField
 *  \param      ldoEn               LDO enable. For valid values, refer to
 *                                      \ref Tps6522x_ldoEnBitField
 *  \param      ldoMode             LDO mode. For valid values, refer to
 *                                      \ref Tps6522x_bypConfigBitField
 *  \param      ldoVset             LDO voltage in VSET form
 *                                  For possible ranges of LDO2 and LDO3, refer to
 *                                      \ref Tps6522x_ldo2_3_VoltageRangesMV
 *  \param      ldoVmonThr          LDO VMON threshold. For valid values, refer to
 *                                      \ref Tps6522x_ldoVmonThrBitField
 *  \param      ldoRailGrpSel       LDO rail group selection. For valid values, refer to
 *                                      \ref Tps6522x_ldoRailSelBitField
 *
 *  @{
 */
typedef struct tps6522xLdoCfg_s
{
    uint8_t validParams;
    uint8_t ldoDischargeEn;
    uint8_t ldoVmonEn;
    uint8_t ldoEn;
    uint8_t ldoMode;
    uint8_t ldoVset;
    uint8_t ldoVmonThr;
    uint8_t ldoRailGrpSel;
} tps6522xLdoCfg_t;
/** @} */

/**
 *  \anchor     Pmic_powerTps6522xVccaVmonPowerResourceCfg_s
 *  \name       VCCA_VMON/VMONx Power Configuration Struct for TPS6522x Burton
 *
 *  \brief      This struct holds information relevent to the VCCA_VMON and VMONx power resources.
 *
 *  \note       ValidParams is input param for all Set and Get APIs. Other
 *              params except validParams are input params for Set APIs and output
 *              params for Get APIs.
 *
 *  \param      validParams         Indication of which struct members are valid. More 
 *                                  precisely, each bit in the variable indicates whether 
 *                                  or not a struct member is valid. For valid values, 
 *                                  refer to \ref Tps6522x_vccaVmonPwrRsrcCfgValidParamShift
 *  \param      vmonDeglitchSel     Deglitech selection for voltage monitors.
 *                                  For valid values, refer to \ref Tps6522x_vmonDeglitchSelBitField
 *  \param      vmon2En             VMON2 enable. For valid values, refer to \ref Tps6522x_vmon2EnBitField
 *  \param      vmon1En             VMON1 enable. For valid values, refer to \ref Tps6522x_vmon1EnBitField
 *  \param      vccaVmonEn          VCCA_VMON enable. For valid values, refer to
 *                                      \ref Tps6522x_vccaVmonEnBitField
 *  \param      vccaPgSet           VCCA_VMON PG level in PG_SET from. For valid values, refer to
 *                                      \ref Tps6522x_vccaPgSetBitField
 *  \param      vccaVmonThr         VCCA_VMON threshold. For valid values, refer to
 *                                      \ref Tps6522x_vccaVmonThrBitField
 *  \param      vccaRailGrpSel      VCCA_VMON rail group selection. For valid values,
 *                                      refer to \ref Tps6522x_vccaRailSelBitField
 *  \param      vmon1Thr            VMON1 threshold. For valid values, refer to
 *                                      \ref Tps6522x_vmon1ThrBitField
 *  \param      vmon1PgSet          VMON1 PG level in PG_SET form
 *  \param      vmon1RailGrpSel     VMON1 rail group selection. For valid values, refer to
 *                                      \ref Tps6522x_vmon1RailSelBitField
 *  \param      vmon2Thr            VMON2 treshold. For valid values, refer to
 *                                      \ref Tps6522x_vmon2ThrBitField
 *  \param      vmon2PgSet          VMON2 PG level in PG_SET form
 *  \param      vmon2RailGrpSel     VMON2 rail group selection. For valid values, refer to
 *                                      \ref Tps6522x_vmon2RailSelBitField
 *  @{
 */
typedef struct tps6522xVccaVmonCfg_s
{
    uint16_t validParams;
    uint8_t vmonDeglitchSel;
    uint8_t vmon2En;
    uint8_t vmon1En;
    uint8_t vccaVmonEn;
    uint8_t vccaPgSet;
    uint8_t vccaVmonThr;
    uint8_t vccaRailGrpSel;
    uint8_t vmon1Thr;
    uint8_t vmon1PgSet;
    uint8_t vmon1RailGrpSel;
    uint8_t vmon2Thr;
    uint8_t vmon2PgSet;
    uint8_t vmon2RailGrpSel;
} tps6522xVccaVmonCfg_t;
/** @} */

/**
 *  \anchor     Pmic_powerTps6522xPowerResourceCfg_s
 *  \name       Power Configuration Struct for TPS6522x Burton
 *
 *  \brief      This struct holds information relevent to all power resource of TPS6522x Burton.
 *
 *  \note       ValidParams is input param for all Set and Get APIs. Other
 *              params except validParams are input params for Set APIs and output
 *              params for Get APIs.
 *
 *  \param      validParams         Indication of which struct members are valid. More 
 *                                  precisely, each bit in the variable indicates whether 
 *                                  or not a struct member is valid. For valid values, 
 *                                  refer to \ref Tps6522x_pwrRsrcCfgValidParamShift
 *  \param      buckCfg             Array of BUCK power resource configuration structures.
 *                                  Each element in this array is a BUCK - for example, element
 *                                  zero is the power resource configuration structure for BUCK1.
 *                                  For more information, \ref Tps6522x_buckPwrRsrcCfg
 *  \param      ldoCfg              Array of LDO power resource configuration structures.
 *                                  Each element in this array is a LDO - for example, element
 *                                  zero is the power resource configuration structure for LDO1.
 *                                  For more information, \ref Tps6522x_ldoPwrRsrcCfg
 *  \param      vccaVmonCfg         VCCA/VMON power resource configuration struct.
 *                                  For more information, \ref Tps6522x_vccaVmonPwrRsrcCfg
 *
 *  @{
 */
typedef struct tps6522xPwrRsrcCfg_s
{
    uint16_t validParams;
    tps6522xBuckCfg_t buckCfg[TPS6522X_MAX_BUCK_NUM];
    tps6522xLdoCfg_t ldoCfg[TPS6522X_MAX_LDO_NUM];
    tps6522xVccaVmonCfg_t vccaVmonCfg;
} tps6522xPwrRsrcCfg_t;
/** @} */

/**
 *  \anchor     Pmic_powerTps6522xBuckRegisters_s
 *  \name       TPS6522x Burton BUCK Registers
 *
 *  \brief      This struct holds registers relevent to the BUCK power resource.
 *
 *  @{
 */
typedef struct tps6522xBuckRegs_s
{
    uint8_t buckCtrlRegAddr;
    uint8_t buckConfRegAddr;
    uint8_t buckVoutRegAddr;
    uint8_t buckPgWindowRegAddr;
    uint8_t buckRailSelRegAddr;
} tps6522xBuckRegs_t;
/** @} */

/**
 *  \anchor     Pmic_powerTps6522xLdoRegisters_s
 *  \name       TPS6522x Burton LDO Registers
 *
 *  \brief      This struct holds registers relevent to the LDO power resource.
 *
 *  @{
 */
typedef struct tps6522xLdoRegs_s
{
    uint8_t ldoCtrlRegAddr;
    uint8_t ldoVoutRegAddr;
    uint8_t ldoPgWindowRegAddr;
    uint8_t ldoRailSelRegAddr;
} tps6522xLdoRegs_t;
/** @} */

/**
 *  \anchor     Pmic_powerTps6522xVccaVmonRegisters_s
 *  \name       TPS6522x Burton VCCA_VMON and VMONx Registers
 *
 *  \brief      This struct holds registers relevent to the VCCA_VMON and VMONx power resources.
 *
 *  @{
 */
typedef struct tps6522xVccaVmonRegs_s
{
    uint8_t vccaVmonCtrlRegAddr;
    uint8_t vccaPgWindowRegAddr;
    uint8_t vmonPgWindowRegAddr;
    uint8_t vmonPgLevelRegAddr;
    uint8_t vccaVmonRailSelRegAddr;
} tps6522xVccaVmonRegs_t;
/** @} */

/**
 *  \anchor     Pmic_powerTps6522xThermalStat_s
 *  \name       TPS6522x Burton Thermal Statuses
 *
 *  \brief      This struct holds all the thermal statuses of the PMIC.
 *
 * \note        ValidParams is input param for all Get APIs. Other
 *              params except validParams are output params for Get APIs.
 *
 *  \param      validParams     Indication of which struct members are valid. 
 *                              More precisely, each bit in the variable 
 *                              indicates whether or not a struct member is 
 *                              valid. For valid values, refer to  
 *                                  \ref Pmic_PowerThermalStatValidParamCfg
 * \param       twarnStat       Temperature warning status
 * \param       tsdOrdStat      Orderly thermal shutdown status
 * \param       tsdImmStat      Immediate thermal shutdown status
 *
 *  @{
 */
typedef struct tps6522xThermalStat_s
{
    uint8_t validParams;
    bool    twarnStat;
    bool    tsdOrdStat;
    bool    tsdImmStat;
} tps6522xThermalStat_t;
/** @} */

/**
 *  \anchor     Pmic_powerTps6522xThermalCfg_s
 *  \name       TPS6522x Burton Thermal Configuration Struct
 *
 *  \brief      This struct holds information that is relevant to PMIC thermal configurations.
 *
 *  \note       ValidParams is input param for all Set and Get APIs. Other
 *              params except validParams are input params for Set APIs and output
 *              params for Get APIs.
 *
 *  \param      validParams     Indication of which struct members are valid. 
 *                              More precisely, each bit in the variable 
 *                              indicates whether or not a struct member is 
 *                              valid. For valid values, refer to  
 *                                  \ref Tps6522x_thermalCfgValidParam
 *  \param      tsdOrdLvl       Used to set the orderly thermal stutdown level.
 *                              For valid values, refer to \ref Tps6522x_tsdOrdLvlBitField
 *  \param      twarnLvl        Used to set the temperature warning level.
 *                              For valid values, refer to \ref Tps6522x_TwarnLvlBitField
 *
 *  @{
 */
typedef struct tps6522xThermalCfg_s
{
    uint8_t validParams;
    uint8_t tsdOrdLvl;
    uint8_t twarnLvl;
} tps6522xThermalCfg_t;
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
extern const tps6522xBuckRegs_t gTps6522xBuckRegisters[];
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
extern const tps6522xLdoRegs_t gTps6522xLdoRegisters[];
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
extern const tps6522xVccaVmonRegs_t gTps6522xVccaVmonRegisters[];
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
 *              validParam within \p pPwrRsrcCfg struct. For valid values, refer to
 *              \ref Tps6522x_pwrRsrcCfgValidParamShift. Setting the validParam
 *              indicates that the end user is interested in that specific power resource.
 *              Afterwards, end user will need to set the desired power resource's 
 *              configuration validParam(s) within buckCfg, ldoCfg, or vccaVmonCfg. 
 *              For  valid values, refer to \ref Tps6522x_buckPwrRsrcCfgValidParamShift,
 *              \ref Tps6522x_ldoPwrRsrcCfgValidParamShift, and
 *              \ref Tps6522x_vccaVmonPwrRsrcCfgValidParamShift.
 *              Setting a validParam indicates that the end user is interested in getting
 *              a specific configuration of a power resource.
 *
 *              Supported obtainable configurations by this API for a BUCK are:
 *              1. Buck pull-down resistor
 *              2. Buck VMON enable
 *              3. Buck PWM mode (auto or forced)
 *              4. Buck enable
 *              5. Buck slew-rate
 *              6. Buck VSET
 *              7. Buck VMON powergood threshold
 *              8. Buck rail group selection
 *
 *              Supported obtainable configurations by this API for a LDO are:
 *              1. LDO discharge enable
 *              2. LDO VMON enable
 *              3. LDO enable
 *              4. LDO mode (bypass mode, LDO mode)
 *              5. LDO VSET
 *              6. LDO VMON powergood threshold
 *              7. LDO rail group selection
 *
 *              Supported obtainable configurations by this API for a VCCA_VMON/VMONx are:
 *              1. VMON deglitch selection
 *              2. VCCA_VMON/VMONx enable
 *              3. VCCA_VMON/VMONx powergood threshold
 *              4. VCCA_VMON/VMONx powergood level (PG_SET)
 *              5. VCCA_VMON/VMONx rail group selection
 *
 *  \param      pPmicCoreHandle     [IN]        PMIC interface handle
 *  \param      pPwrRsrcCfg         [OUT]       Pointer to power resource configuration struct
 *                                              in which API will use to store information
 *
 *  \return     Success code if power resource configurations are obtained, error code otherwise.
 *              For valid success/error codes, refer to \ref Pmic_ErrorCodes
 */
int32_t tps6522xGetPwrRsrcCfg(Pmic_CoreHandle_t *pPmicCoreHandle, tps6522xPwrRsrcCfg_t *pPwrRsrcCfg);

/**
 *  \brief      This function is used to get all desired information for a BUCK.
 *              Desired information is specified by the user via setting validParams
 *              within \p pBuckCfg struct. For values of validParams,
 *              refer to \ref Tps6522x_buckPwrRsrcCfgValidParamShift.
 *
 *  \details    End user could use this API if they only want to get the configuration
 *              of a single BUCK power resource. End user could also use this API to get
 *              BUCK power resource configuration if their application is memory constrained.
 *              The API tps6522xGetPwrRsrcCfg() is a superset of this API and will accomplish 
 *              more in a single API call than this API, however it is more memory intensive.
 *
 *              Supported obtainable configurations by this API for a BUCK are:
 *              1. Buck pull-down resistor
 *              2. Buck VMON enable
 *              3. Buck PWM mode (auto or forced)
 *              4. Buck enable
 *              5. Buck slew-rate
 *              6. Buck VSET
 *              7. Buck VMON powergood threshold
 *              8. Buck rail group selection
 *
 *  \param      pPmicCoreHandle             [IN]        PMIC interface handle
 *  \param      pBuckCfg                    [IN/OUT]    BUCK power resource configuration
 *  \param      buckNum                     [IN]        Indicates which BUCK the API is working with.
 *                                                      For valid values, refer to
 *                                                          \ref Tps6522x_buckPwrRsrcNum
 *
 *  \return     Success code if BUCK information is stored at pBuckPowerResourceCfg,
 *              error code otherwise. For valid success/error codes, refer to \ref Pmic_ErrorCodes
 */
int32_t tps6522xGetBuckCfg(Pmic_CoreHandle_t *pPmicCoreHandle, tps6522xBuckCfg_t *pBuckCfg,  const uint8_t buckNum);

/**
 *  \brief      This function is used to get all desired information for a LDO.
 *              Desired information is specified by the user via setting validParams
 *              within \p pLdoCfg struct. For values of validParams, refer to 
 *              \ref Tps6522x_ldoPwrRsrcCfgValidParamShift.
 *
 *  \details    End user could use this API if they only want to get the configuration
 *              of a single LDO power resource. End user could also use this API to get
 *              LDO power resource configuration if their application is memory constrained.
 *              The API tps6522xGetPwrRsrcCfg() is a superset of this API and will accomplish 
 *              more in a single API call than this API, however it is more memory intensive.
 *
 *              Supported configurable options by this API for a LDO are:
 *              1. LDO discharge enable
 *              2. LDO VMON enable
 *              3. LDO enable
 *              4. LDO mode (bypass mode, LDO mode)
 *              5. LDO VSET
 *              6. LDO VMON powergood threshold
 *              7. LDO rail group selection
 *
 *  \param      pPmicCoreHandle         [IN]        PMIC interface handle
 *  \param      pLdoCfg                 [IN/OUT]    LDO power resource configuration
 *  \param      ldoNum                  [IN]        Indicates which LDO the API is working with.
 *                                                  For valid values, refer to
 *                                                      \ref Tps6522x_ldoPwrRsrcNum
 *
 *  \return     Success code if desired LDO information is stored at pLdoPowerResourceCfg,
 *              error code otherwise. For valid success/error codes, refer to \ref Pmic_ErrorCodes
 */
int32_t tps6522xGetLdoCfg(Pmic_CoreHandle_t *pPmicCoreHandle, tps6522xLdoCfg_t *pLdoCfg, const uint8_t ldoNum);

/**
 *  \brief      This function is used to get all desired information for a VCCA/VMONx.
 *              Desired information is specified by the user via setting validParams
 *              within \p pVccaVmonCfg struct. For values of validParams, refer to 
 *              \ref Tps6522x_vccaVmonPwrRsrcCfgValidParamShift.
 *
 *  \details    End user could use this API if they only want to get the configuration
 *              of a single VCCA/VMONx power resource. End user could also use this API to
 *              get VCCA/VMONx power resource configuration if their application is memory
 *              constrained. The API tps6522xGetCfg() is a superset of this API and will 
 *              accomplish more in a single API call than this API, however it is more 
 *              memory intensive.
 *
 *              Supported obtainable configurations by this API for a VCCA_VMON/VMONx are:
 *              1. VMON deglitch selection
 *              2. VCCA_VMON/VMONx enable
 *              3. VCCA_VMON/VMONx powergood threshold
 *              4. VCCA_VMON/VMONx powergood level (PG_SET)
 *              5. VCCA_VMON/VMONx rail group selection
 *
 *  \param      pPmicCoreHandle         [IN]        PMIC interface handle
 *  \param      pVccaVmonCfg            [IN/OUT]    VCCA/VMONx power resource configuration
 *  \param      vmonNum                 [IN]        Indicates which VCCA/VMONx the API is working with
 *                                                  For valid values, refer to
 *                                                      \ref Tps6522x_vccaVmonPwrRsrcNum
 *
 *  \return     Success code if VCCA/VMON information is stored at pVccaVmonPwrRsrcCfg,
 *              error code otherwise. For valid success/error codes, refer to \ref Pmic_ErrorCodes
 */
int32_t tps6522xGetVccaVmonCfg(Pmic_CoreHandle_t *pPmicCoreHandle,
                               tps6522xVccaVmonCfg_t *pVccaVmonCfg,
                               const uint8_t vmonNum);

/**
 *  \brief      This function is used to set the configuration of any number of TPS6522x 
 *              Burton power resources (BUCKs, LDOs, and voltage monitors). In particular, 
 *              The API can be used to set the configurations of a single power resource, 
 *              set the configurations of multiple power resources, or set the configurations 
 *              of all power resources at once via single API call.
 *
 *  \details    To use this API, end user needs to set the desired power resource's
 *              validParam within \p pPwrRsrcCfg struct. For valid values, refer to
 *              \ref Tps6522x_pwrRsrcCfgValidParamShift. Setting the validParam
 *              indicates that the end user is interested in that specific power 
 *              resource. Afterwards, end user will need to set the desired power 
 *              resource's configuration validParam(s) within buckPwrRsrcCfg, 
 *              ldoPwrRsrcCfg, or vccaVmonPwrRsrcCfg. For valid values, refer to 
 *              \ref Tps6522x_buckPwrRsrcCfgValidParamShift,
 *              \ref Tps6522x_ldoPwrRsrcCfgValidParamShift, and
 *              \ref Tps6522x_vccaVmonPwrRsrcCfgValidParamShift.
 *              Setting a validParam indicates that the end user is interested in 
 *              setting a specific configuration of a power resource.
 *
 *              Supported configurable options by this API for a BUCK are:
 *              1. Buck pull-down resistor
 *              2. Buck VMON enable
 *              3. Buck PWM mode (auto or forced)
 *              4. Buck enable
 *              5. Buck slew-rate
 *              6. Buck VSET
 *              7. Buck VMON powergood threshold
 *              8. Buck rail group selection
 *
 *              Supported configurable options by this API for a LDO are:
 *              1. LDO discharge enable
 *              2. LDO VMON enable
 *              3. LDO enable
 *              4. LDO mode (bypass mode, LDO mode)
 *              5. LDO VSET
 *              6. LDO VMON powergood threshold
 *              7. LDO rail group selection
 *
 *              Supported configurable options by this API for a VCCA_VMON/VMONx are:
 *              1. VMON deglitch selection
 *              2. VCCA_VMON/VMONx enable
 *              3. VCCA_VMON/VMONx powergood threshold
 *              4. VCCA_VMON/VMONx powergood level (PG_SET)
 *              5. VCCA_VMON/VMONx rail group selection
 *
 *  \param      pPmicCoreHandle     [IN]        PMIC interface handle
 *  \param      pwrRsrcCfg      [IN]            Power resource configuration struct used by the API
 *                                              to set power configurations of TPS6522x
 *
 *  \return     Success code if power resource configurations are set, error code otherwise.
 *              For valid success/error codes, refer to \ref Pmic_ErrorCodes
 */
int32_t tps6522xSetPwrRsrcCfg(Pmic_CoreHandle_t *pPmicCoreHandle, const tps6522xPwrRsrcCfg_t pwrRsrcCfg);

/**
 *  \brief      This function is used to set all desired information for a BUCK.
 *              Desired information is specified by the user via setting validParams
 *              within \p buckCfg struct. For values of validParams, refer to 
 *              \ref Tps6522x_buckPwrRsrcCfgValidParamShift.
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
 *              6. Buck VSET
 *              7. Buck VMON powergood threshold
 *              8. Buck rail group selection
 *
 *  \param      pPmicCoreHandle             [IN]        PMIC interface handle
 *  \param      buckCfg                     [IN]        BUCK power resource configuration struct
 *  \param      buckNum                     [IN]        Indicates which BUCK the API is working with.
 *                                                      For valid values, refer to
 *                                                          \ref Tps6522x_buckPwrRsrcNum
 *
 *  \return     Success code if Buck power resource configurations are set, error code otherwise.
 *              For valid success/error codes, refer to \ref Pmic_ErrorCodes
 */
int32_t tps6522xSetBuckCfg(Pmic_CoreHandle_t *pPmicCoreHandle, const tps6522xBuckCfg_t buckCfg, const uint8_t buckNum);

/**
 *  \brief      This function is used to set all desired information for a LDO.
 *              Desired information is specified by the user via setting validParams
 *              within \p ldoCfg struct. For values of validParams, refer to 
 *              \ref Tps6522x_ldoPwrRsrcCfgValidParamShift.
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
 *              5. LDO VSET
 *              6. LDO VMON powergood threshold
 *              7. LDO rail group selection
 *
 *  \param      pPmicCoreHandle         [IN]        PMIC interface handle
 *  \param      ldoCfg                  [IN]        LDO power resource configuration struct
 *  \param      ldoNum                  [IN]        Indicates which LDO the API is working with.
 *                                                  For valid values, refer to
 *                                                      \ref Tps6522x_ldoPwrRsrcNum
 *
 *  \return     Success code if LDO power resource configurations are set, error code otherwise.
 *              For valid success/error codes, refer to \ref Pmic_ErrorCodes
 */
int32_t tps6522xSetLdoCfg(Pmic_CoreHandle_t *pPmicCoreHandle, const tps6522xLdoCfg_t ldoCfg, const uint8_t ldoNum);

/**
 *  \brief      This function is used to get all desired information for a VCCA/VMONx.
 *              Desired information is specified by the user via setting validParams
 *              within \p vccaVmonCfg struct. For values of validParams, refer to 
 *              \ref Tps6522x_vccaVmonPwrRsrcCfgValidParamShift.
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
 *              4. VCCA_VMON/VMONx powergood level (PG_SET)
 *              5. VCCA_VMON/VMONx rail group selection
 *
 *  \param      pPmicCoreHandle         [IN]        PMIC interface handle
 *  \param      vccaVmonCfg             [IN]        VCCA/VMON power resource configuration struct
 *  \param      vmonNum                 [IN]        Indicates which VCCA/VMONx the API is working with.
 *                                                  For valid values, refer to
 *                                                      \ref Tps6522x_vccaVmonPwrRsrcNum
 *
 *  \return     Success code if VCCA_VMON/VMONx configurations are set, error code otherwise.
 *              For valid success/error codes, refer to \ref Pmic_ErrorCodes
 */
int32_t tps6522xSetVccaVmonCfg(Pmic_CoreHandle_t *pPmicCoreHandle,
                               const tps6522xVccaVmonCfg_t vccaVmonCfg,
                               const uint8_t vmonNum);

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
int32_t tps6522xGetPwrRsrcStat(Pmic_CoreHandle_t *pPmicCoreHandle,
                               const uint8_t pwrRsrcUVOVStatus,
                               bool *pUnderOverVoltStat);

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
int32_t tps6522xGetThermalStat(Pmic_CoreHandle_t *pPmicCoreHandle, tps6522xThermalStat_t *pThermalStat);

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
int32_t tps6522xGetThermalCfg(Pmic_CoreHandle_t *pPmicCoreHandle, tps6522xThermalCfg_t *pThermalCfg);

/**
 *  \brief      This function is used to set the thermal configuration of TPS6522x Burton.
 *
 *  \param      pPmicCoreHandle     [IN]        PMIC interface handle
 *  \param      thermalCfg          [IN]        Thermal configuration struct
 *
 *  \return     Success code if PMIC thermal configurations are set, error code otherwise.
 *              For valid success/error codes, refer to \ref Pmic_ErrorCodes
 */
int32_t tps6522xSetThermalCfg(Pmic_CoreHandle_t *pPmicCoreHandle, const tps6522xThermalCfg_t thermalCfg);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* PMIC_POWER_TPS6522X_H_ */

/** @} */
