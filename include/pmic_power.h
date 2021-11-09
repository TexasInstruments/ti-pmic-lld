/******************************************************************************
 * Copyright (c) 2020 Texas Instruments Incorporated - http://www.ti.com
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
 *  \ingroup DRV_PMIC_MODULE
 *  \defgroup DRV_PMIC_POWER_MODULE PMIC Power Driver API
 *      This module explains about PMIC Power Resources driver parameters and
 *      API usage. PMIC Power Module covers all power resources feature APIs,
 *      which includes set/get BUCK and LDO regulator output voltage
 *      configurations, set/get volatge monitor, current monitor, power good
 *      monitor and short circuit protection configuration for external power
 *      sources, set/get thermal monitoring/shutdown of the PMIC module,
 *      APIs to configure regulator and VMON interrupts to notify the
 *      application when PMIC power related errors are found on the power
 *      Rails
 *
 *      Supported PMIC devices for Power Module:
 *      1. TPS6594x (Leo PMIC Device)
 *      2. LP8764x  (Hera PMIC Device)
 *  @{
 */

/**
 * \file   pmic_power.h
 *
 * \brief  PMIC Power Resources Driver Interface file
 */

#ifndef PMIC_POWER_H_
#define PMIC_POWER_H_

/* ========================================================================= */
/*                             Include Files                                 */
/* ========================================================================= */

#include <pmic_core.h>
#include <cfg/tps6594x/pmic_power_tps6594x.h>
#include <cfg/lp8764x/pmic_power_lp8764x.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================= */
/*                             Macros & Typedefs                             */
/* ========================================================================= */

/**
 *  \anchor Pmic_PowerResourcesValidParamCfg
 *  \name   PMIC Power Resources Config Structure Param Bits
 *
 *  PMIC Power Resources valid params configuration type for
 *  the structure member validParams of Pmic_PowerResourceCfg_t
 *  structure
 *
 *  @{
 */
  /** \brief validParams value used to set/get to Enable/Disable residual
   *         voltage checking for regulator/VMON pin
   *         Valid only for VMON1/VMON2/BUCK/LDO */
#define PMIC_CFG_REGULATOR_VMON_RV_SEL_VALID              (0U)
 /** \brief validParams value used to set/get to Enable/Disable output pull-down
  *         resistor when BUCK is disabled.
  *         Valid only for BUCK regulator */
#define PMIC_CFG_REGULATOR_BUCK_PLDN_EN_VALID             (1U)
 /** \brief validParams value used to set/get to Enable /Disable the Voltage
  *         monitor feature
  *         Valid only for VMON1/VMON2/VMON of BUCK/VMON of LDO/VMON of VCCA
  *         Power Resources */
#define PMIC_CFG_VMON_EN_VALID                            (2U)
 /** \brief validParams value used to set/get to Select output voltage register
  *         for BUCK
  *         Valid only for BUCK regulator */
#define PMIC_CFG_REGULATOR_BUCK_VOUT_SEL_VALID            (3U)
 /** \brief validParams value used to set/get to Select PWM or Auto Mode for
  *         BUCK
  *         Valid only for BUCK regulator */
#define PMIC_CFG_REGULATOR_BUCK_FPWM_VALID                (4U)
 /** \brief validParams value used to set/get to Select between Multi phase with
  *         PWM OR AUTO mode with Automatic phase adding and shedding for BUCK
  *         Valid only for BUCK regulator */
#define PMIC_CFG_REGULATOR_BUCK_PWM_MP_VALID              (5U)
 /** \brief validParams value used to set/get to Enable/Disable the power
  *         regulators
  *         Valid only for BUCK/LDO regulator */
#define PMIC_CFG_REGULATOR_EN_VALID                       (6U)
 /** \brief validParams value used to set/get to Enable/Disable Slow Ramp for
  *         LDO
  *         Valid only for LDO regulator
  *         Valid only for TPS6594x Leo PMIC PG2.0 */
#define PMIC_CFG_REGULATOR_LDO_SLOW_RAMP_EN_VALID         (7U)
 /** \brief validParams value used to set/get to Select Bypass/Linear Regulator
  *         LDO mode
  *         Valid only for LDO regulator */
#define PMIC_CFG_REGULATOR_LDO_BYPASS_MODE_EN_VALID       (8U)
 /** \brief validParams value used to set/get Powergood level for VCCA pin
  *         Valid only for VCCA */
#define PMIC_CFG_VCCA_PWR_GOOD_LVL_VALID                  (9U)
 /** \brief validParams value used to set/get to Select OV/UV voltage monitoring
  *         range for VMON
  *         Valid only for VMON1/VMON2 */
#define PMIC_CFG_VMON_RANGE_VALID                         (10U)
 /** \brief validParams value used to set/get Switch peak current limit for BUCK
  *         regulator
  *         Valid only for BUCK regulator */
#define PMIC_CFG_REGULATOR_BUCK_ILIM_VALID                (12U)
 /** \brief validParams value used to set/get Output voltage slew rate for
  *         BUCK/VMON
  *         Valid only for VMON1/VMON2/BUCK */
#define PMIC_CFG_REGULATOR_BUCK_VMON_SLEW_RATE_VALID      (13U)
 /** \brief validParams value used to set/get to Selects the resistor value for
  *         output pull-down resistor for LDO regulator
  *         Valid only for LDO regulator */
#define PMIC_CFG_REGULATOR_LDO_PLDN_SEL_VALID             (14U)
 /** \brief validParams value used to set/get LDO residual voltage check timeout
  *         select
  *         Valid only for LDO regulator */
#define PMIC_CFG_REGULATOR_LDO_RV_TIMEOUT_SEL_VALID       (15U)
 /** \brief validParams value used to set/get Power good low threshold level for
  *         the power resources
  *         Valid for all power resources */
#define PMIC_CFG_PWR_RESOURCE_PG_UV_THRESHOLD_LVL_VALID   (16U)
 /** \brief validParams value used to set/get Power good high threshold level
  *         for the power resources
  *         Valid for all power resources */
#define PMIC_CFG_PWR_RESOURCE_PG_OV_THRESHOLD_LVL_VALID   (17U)
 /** \brief validParams value used to set/get  Rail group selection for the
  *         power resources
  *         Valid for all power resources */
#define PMIC_CFG_PWR_RESOURCE_RAIL_GRP_SEL_VALID          (18U)
 /** \brief validParams value used to set/get Voltage level (For LDO/BUCK)/
  *         Powergood voltage level(For VMON1/VMON2) in mv
  *         Valid only for VMON1/VMMON2/BUCK/LDO */
#define PMIC_CFG_REGULATOR_VMON_VOLTAGE_SET_VALID         (19U)
/* @} */

/**
 *  \anchor Pmic_PowerResourcesValidParamBitShiftValues
 *  \name   PMIC Power Resources valid param bit shift values
 *
 *  Application can use below shifted values to set the validParam
 *  member defined in Pmic_PowerResourceCfg_t structure
 *
 *  @{
 */
#define PMIC_CFG_REGULATOR_VMON_RV_SEL_VALID_SHIFT       \
                         (1U << PMIC_CFG_REGULATOR_VMON_RV_SEL_VALID)
#define PMIC_CFG_REGULATOR_BUCK_PLDN_EN_VALID_SHIFT      \
                         (1U << PMIC_CFG_REGULATOR_BUCK_PLDN_EN_VALID)
#define PMIC_CFG_VMON_EN_VALID_SHIFT         (1U << PMIC_CFG_VMON_EN_VALID)
#define PMIC_CFG_REGULATOR_BUCK_VOUT_SEL_VALID_SHIFT     \
                         (1U << PMIC_CFG_REGULATOR_BUCK_VOUT_SEL_VALID)
#define PMIC_CFG_REGULATOR_BUCK_FPWM_VALID_SHIFT         \
                         (1U << PMIC_CFG_REGULATOR_BUCK_FPWM_VALID)
#define PMIC_CFG_REGULATOR_BUCK_PWM_MP_VALID_SHIFT       \
                         (1U << PMIC_CFG_REGULATOR_BUCK_PWM_MP_VALID)
#define PMIC_CFG_REGULATOR_EN_VALID_SHIFT    (1U << PMIC_CFG_REGULATOR_EN_VALID)
#define PMIC_CFG_REGULATOR_LDO_SLOW_RAMP_EN_VALID_SHIFT  \
                         (1U << PMIC_CFG_REGULATOR_LDO_SLOW_RAMP_EN_VALID)
#define PMIC_CFG_REGULATOR_LDO_BYPASS_MODE_EN_VALID_SHIFT       \
                         (1U << PMIC_CFG_REGULATOR_LDO_BYPASS_MODE_EN_VALID)
#define PMIC_CFG_VCCA_PWR_GOOD_LVL_VALID_SHIFT             \
                         (1U << PMIC_CFG_VCCA_PWR_GOOD_LVL_VALID)
#define PMIC_CFG_VMON_RANGE_VALID_SHIFT      (1U << PMIC_CFG_VMON_RANGE_VALID)
#define PMIC_CFG_REGULATOR_BUCK_ILIM_VALID_SHIFT         \
                         (1U << PMIC_CFG_REGULATOR_BUCK_ILIM_VALID)
#define PMIC_CFG_REGULATOR_BUCK_VMON_SLEW_RATE_VALID_SHIFT    \
                         (1U << PMIC_CFG_REGULATOR_BUCK_VMON_SLEW_RATE_VALID)
#define PMIC_CFG_REGULATOR_LDO_PLDN_SEL_VALID_SHIFT      \
                         (1U << PMIC_CFG_REGULATOR_LDO_PLDN_SEL_VALID)
#define PMIC_CFG_REGULATOR_LDO_RV_TIMEOUT_SEL_VALID_SHIFT \
                         (1U << PMIC_CFG_REGULATOR_LDO_RV_TIMEOUT_SEL_VALID)
#define PMIC_CFG_PWR_RESOURCE_PG_UV_THRESHOLD_LVL_VALID_SHIFT     \
                         (1U << PMIC_CFG_PWR_RESOURCE_PG_UV_THRESHOLD_LVL_VALID)
#define PMIC_CFG_PWR_RESOURCE_PG_OV_THRESHOLD_LVL_VALID_SHIFT     \
                         (1U << PMIC_CFG_PWR_RESOURCE_PG_OV_THRESHOLD_LVL_VALID)
#define PMIC_CFG_PWR_RESOURCE_RAIL_GRP_SEL_VALID_SHIFT         \
                         (1U << PMIC_CFG_PWR_RESOURCE_RAIL_GRP_SEL_VALID)
#define PMIC_CFG_REGULATOR_VMON_VOLTAGE_SET_VALID_SHIFT  \
                         (1U << PMIC_CFG_REGULATOR_VMON_VOLTAGE_SET_VALID)
/* @} */

/**
 *  \anchor Pmic_PowerCommonParamCfg
 *  \name   PMIC Pmic_PowerCommonCfg_s member configuration type.
 *
 *  @{
 */
  /** \brief validParams value used to set/get to Select the type of voltage
   *         monitoring for PGOOD signal  */
#define PMIC_POWER_PGOOD_WINDOW_VALID             (0U)
 /** \brief validParams value used to set/get to Select the PGOOD signal
  *         polarity */
#define PMIC_POWER_PGOOD_POL_VALID                (1U)
 /** \brief validParams value used to set/get Deglitch time select for all power
  *         resources
  *         Valid only for VMON1/VMON2/VCCA/VMON of LDO/VMON of BUCK power
  *         resources
  *         Valid only for TPS6594x Leo PMIC PG2.0 and LP8764x Hera PMIC PG1.0
  *         and PG2.0
  */
#define PMIC_CFG_DEGLITCH_TIME_SEL_VALID          (2U)
 /** \brief validParams value used to set/get to Select the trigger selection
  *         for severe Error  */
#define PMIC_SEVERE_ERR_TRIG_VALID                (3U)
 /** \brief validParams value used to set/get to Select the trigger selection
  *         for other rail group  */
#define PMIC_OTHER_RAIL_TRIG_VALID                (4U)
 /** \brief validParams value used to set/get to Select the trigger selection
  *         for soc rail group  */
#define PMIC_SOC_RAIL_TRIG_VALID                  (5U)
 /** \brief validParams value used to set/get to Select the trigger selection
  *         for mcu rail group  */
#define PMIC_MCU_RAIL_TRIG_VALID                  (6U)
 /** \brief validParams value used to set/get Select the trigger selection for
  *         Moderate Error  */
#define PMIC_MODERATE_ERR_TRIG_VALID              (7U)
/* @} */

/**
 *  \anchor Pmic_PowerCommonParamBitShiftValues
 *  \name   PMIC valid params bit shift values
 *
 *  Application can use below shifted values to set the validParam
 *  member defined in Pmic_PowerCommonCfg_t structure
 *
 *  @{
 */
#define PMIC_POWER_PGOOD_WINDOW_VALID_SHIFT               \
                                        (1U <<  PMIC_POWER_PGOOD_WINDOW_VALID)
#define PMIC_POWER_PGOOD_POL_VALID_SHIFT                  \
                                        (1U <<  PMIC_POWER_PGOOD_POL_VALID)
#define PMIC_CFG_DEGLITCH_TIME_SEL_VALID_SHIFT            \
                                        (1U << PMIC_CFG_DEGLITCH_TIME_SEL_VALID)
#define PMIC_SEVERE_ERR_TRIG_VALID_SHIFT                  \
                                        (1U <<  PMIC_SEVERE_ERR_TRIG_VALID)
#define PMIC_OTHER_RAIL_TRIG_VALID_SHIFT                  \
                                        (1U <<  PMIC_OTHER_RAIL_TRIG_VALID)
#define PMIC_SOC_RAIL_TRIG_VALID_SHIFT                    \
                                        (1U <<  PMIC_SOC_RAIL_TRIG_VALID)
#define PMIC_MCU_RAIL_TRIG_VALID_SHIFT                    \
                                        (1U <<  PMIC_MCU_RAIL_TRIG_VALID)
#define PMIC_MODERATE_ERR_TRIG_VALID_SHIFT                \
                                        (1U <<  PMIC_MODERATE_ERR_TRIG_VALID)

/**
 *  \anchor Pmic_PowerStatusValidParamCfg
 *  \name   PMIC Power Status Config Structure Param Bits
 *
 *  PMIC Power Status valid params configuration type for
 *  the structure member validParams of Pmic_PowerResourceStat_t
 *  structure.
 *
 *  @{
 */
  /** \brief validParams value used to get output current limit status for LDO
   *         and buck regulators  */
#define PMIC_POWER_REGULATOR_ILIM_STAT_VALID              (0U)
/** \brief validParams value used to get output under voltage status for LDO/
 *         BUCK and input under voltage status for VCCA/VMON
 *         Valid for all power resources */
#define PMIC_POWER_RESOURCE_UV_STAT_VALID                 (1U)
/** \brief validParams value used to get output over voltage status for LDO/BUCK
 *         and input over voltage status for VCCA/VMON
 *         Valid for all power resources */
#define PMIC_POWER_RESOURCE_OV_STAT_VALID                 (2U)
/** \brief validParams value used to get voltage level status for VCCA
 *         Valid only for VCCA */
#define PMIC_POWER_VCCA_OV_LVL_STAT_VALID                 (3U)
/* @} */

/**
 *  \anchor Pmic_PowerStatusValidParamCfgBitShiftValues
 *  \name   PMIC Power Resources valid param bit shift values
 *
 *  Application can use below shifted values to set the validParam
 *  member defined in Pmic_PowerResourceCfg_t structure
 *
 *  @{
 */
#define PMIC_POWER_REGULATOR_ILIM_STAT_VALID_SHIFT      \
                                    (1U << PMIC_POWER_REGULATOR_ILIM_STAT_VALID)
#define PMIC_POWER_RESOURCE_UV_STAT_VALID_SHIFT         \
                                    (1U << PMIC_POWER_RESOURCE_UV_STAT_VALID)
#define PMIC_POWER_RESOURCE_OV_STAT_VALID_SHIFT         \
                                    (1U << PMIC_POWER_RESOURCE_OV_STAT_VALID)
#define PMIC_POWER_VCCA_OV_LVL_STAT_VALID_SHIFT         \
                                    (1U << PMIC_POWER_VCCA_OV_LVL_STAT_VALID)

/**
 *  \anchor Pmic_PowerThermalThresholdValidParamCfg
 *  \name PMIC Power Thermal threshold type
 *
 *  @{
 */
 /** \brief validParams value used to set/get thermal Warning Threshold
  *         temperature value  */
#define PMIC_THERMAL_WARN_VALID                   (1U)
/** \brief validParams value used to set/get Thermal Shutdown Threshold
 *         temperature value
 *         valid only for TPS6594x Leo PMIC PG2.0 and LP8764x Hera PMIC PG1.0
 *         and PG2.0
 */
#define PMIC_THERMAL_SHTDWN_VALID                 (2U)
/* @} */

/**
 *  \anchor Pmic_PowerThermalThresholdValidParamCfgBits
 *  \name   PMIC Power Thermal threshold type structure Param Bits shift values
 *
 *  Application can use below shifted values to set the validParam
 *  member defined in Pmic_PowerThermalCfg_t structure
 *
 *  @{
 */
#define PMIC_THERMAL_WARN_VALID_SHIFT                       \
                                       (1U << PMIC_THERMAL_WARN_VALID)
#define PMIC_THERMAL_SHTDWN_VALID_SHIFT                     \
                                       (1U << PMIC_THERMAL_SHTDWN_VALID)
/* @} */

/**
 *  \anchor Pmic_PowerThermalStatValidParamCfg
 *  \name PMIC Power Thermal status type
 *
 *  @{
 */
 /** \brief validParams value used to get Thermal warning status */
#define PMIC_THERMAL_STAT_WARN_VALID                       (1U)
/** \brief validParams value used to get Orderly Shutdown status  */
#define PMIC_THERMAL_STAT_ORD_SHTDWN_VALID                 (2U)
/** \brief validParams value used to get Immediate Shutdown status */
#define PMIC_THERMAL_STAT_IMM_SHTDWN_VALID                 (3U)
/* @} */

/**
 *  \anchor Pmic_PowerThermalStatValidParamCfgBits
 *  \name   PMIC Power Thermal status type structure Param Bits shift values
 *
 *  Application can use below shifted values to set the validParam
 *  member defined in Pmic_PowerThermalStat_t structure
 *
 *  @{
 */
#define PMIC_THERMAL_STAT_WARN_VALID_SHIFT                       \
                                      (1U << PMIC_THERMAL_STAT_WARN_VALID)
#define PMIC_THERMAL_STAT_ORD_SHTDWN_VALID_SHIFT                 \
                                      (1U << PMIC_THERMAL_STAT_ORD_SHTDWN_VALID)
#define PMIC_THERMAL_STAT_IMM_SHTDWN_VALID_SHIFT                 \
                                      (1U << PMIC_THERMAL_STAT_IMM_SHTDWN_VALID)
/* @} */

/**
 *  \anchor Pmic_PowerInterruptCfg
 *  \name   PMIC Power Interrupt selection
 *
 *  @{
 */
#define PMIC_POWER_INTERRUPT_ENABLE           (0U)
#define PMIC_POWER_INTERRUPT_DISABLE          (1U)
/*  @} */

/**
 *  \anchor Pmic_Power_Good_Window
 *  \name Type of voltage monitoring for PGOOD signal:
 *
 *  @{
 */
/** \brief Only undervoltage is monitored */
#define PMIC_POWER_GOOD_UV_MONITOR_ENABLE         (0x0U)
/** \brief Both undervoltage and overvoltage are monitored */
#define PMIC_POWER_GOOD_UV_OV_MONITOR_ENABLE      (0x1U)
/* @} */

/**
 *  \anchor Pmic_Power_Good_Polarity
 *  \name PGOOD signal polarity
 *
 *  @{
 */
/** \brief PGOOD signal is high when monitored inputs are valid */
#define PMIC_POWER_PGOOD_POL_HIGH                 (0x0U)
/** \brief PGOOD signal is low when monitored inputs are valid */
#define PMIC_POWER_PGOOD_POL_LOW                  (0x1U)
/* @} */

/**
 *  \anchor Pmic_Power_Trigger_Sel
 *  \name   PMIC Power Trigger selection
 *
 *  @{
 */
#define PMIC_POWER_TRIG_IMM_SHUTDOWN             (0U)
#define PMIC_POWER_TRIG_ODERLY_SHUTDOWN          (1U)
#define PMIC_POWER_TRIG_MCU_PWR_ERR              (2U)
#define PMIC_POWER_TRIG_SOC_PWR_ERR              (3U)
/*  @} */
/*==========================================================================*/
/*                         Structures and Enums                             */
/*==========================================================================*/

/*!
 *  \anchor  Pmic_PowerResourceCfg_s
 *  \brief   Pmic Power resources control and configuration structure.
 *           Note: validParams is input param for all Set and Get APIs. other
 *           params except validParams is input param for Set APIs and output
 *           param for Get APIs
 *
 *  \param   validParams         Selection of structure parameters to be set,
 *                               from the combination of
 *                               \ref Pmic_PowerResourcesValidParamCfg
 *                               and the corresponding member value must be
 *                               updated.
 *  \param   rvCheckEn           Enable/Disable residual voltage checking for
 *                               regulator/VMON pin.
 *                               Valid values for TPS6594x Leo Device
 *                               \ref Pmic_TPS6594x_Regulator_RV_Check.
 *                               Valid values for LP8764x HERA Device
 *                               \ref Pmic_LP8764x_Regulator_Vmon_RV_Check.
 *                               Valid only when
 *                               PMIC_CFG_REGULATOR_VMON_RV_SEL_VALID bit is
 *                               set.
 *  \param   buckPullDownEn      Enable/Disable output pull-down resistor when
 *                               BUCK is disabled.
 *                               Valid values for TPS6594x Leo Device
 *                               \ref Pmic_TPS6594x_Buck_Pull_Down_Resistor.
 *                               Valid values for LP8764x HERA Device
 *                               \ref Pmic_LP8764x_Buck_Pull_Down_Resistor.
 *                               Valid only when
 *                               PMIC_CFG_REGULATOR_BUCK_PLDN_EN_VALID bit is
 *                               set.
 *  \param   vmonEn              Enable /Disable the Voltage monitor feature.
 *                               For LDO, VCCA, VMON1 and VMON2:
 *                                   Enable/Disable OV and UV comparators.
 *                               For BUCK:
 *                                   Enable/Disable OV, UV, SC and ILIM
 *                                   comparators.
 *                               Note:  For VMON1 and VMON2, Need to configure
 *                               respective GPIO pin functionality before
 *                               enabling VMON feature
 *                               Valid values for TPS6594x Leo Device
 *                               \ref Pmic_TPS6594x_Vmon_Enable.
 *                               Valid values for LP8764x HERA Device
 *                               \ref Pmic_LP8764x_Vmon_Enable.
 *                               Valid only when PMIC_CFG_VMON_EN_VALID bit is
 *                               set.
 *  \param   buckVoutSel         Select output voltage register for BUCK.
 *                               Valid values for TPS6594x Leo Device
 *                               \ref Pmic_TPS6594x_Regulator_Buck_Vout_Sel.
 *                               Valid values for LP8764x HERA Device
 *                               \ref Pmic_LP8764x_Regulator_Buck_Vout_Sel.
 *                               Valid only when
 *                               PMIC_CFG_REGULATOR_BUCK_VOUT_SEL_VALID bit is
 *                               set.
 *  \param   buckFpwmMode        Select PWM or Auto Mode for BUCK.
 *                               Valid values for TPS6594x Leo Device
 *                               \ref Pmic_TPS6594x_Regulator_Pwm_Pfm_Mode.
 *                               Valid values for LP8764x HERA Device
 *                               \ref Pmic_LP8764x_Regulator_Pwm_Pfm_Mode.
 *                               Valid only when
 *                               PMIC_CFG_REGULATOR_BUCK_FPWM_VALID bit is set.
 *  \param   buckFpwmMpMode      Select between Multi phase with PWM OR AUTO
 *                               mode with Automatic phase adding and shedding
 *                               for BUCK.
 *                               Valid values for TPS6594x Leo Device
 *                               \ref Pmic_TPS6594x_Regulator_Pwm_Mp_Mode.
 *                               Valid values for LP8764x HERA Device
 *                               \ref PMIC_LP8764X_Regulator_Pwm_Mp_Mode.
 *                               Valid only when
 *                               PMIC_CFG_REGULATOR_BUCK_PWM_MP_VALID bit
 *                               is set.
 *  \param   regulatorEn         Enable/Disable the power regulators.
 *                               Valid values for TPS6594x Leo Device
 *                               \ref Pmic_TPS6594x_Power_Regulator_enable.
 *                               Valid values for LP8764x HERA Device
 *                               \ref Pmic_LP8764x_Buck_Regulator_enable.
 *                               Valid only when PMIC_CFG_REGULATOR_EN_VALID
 *                               bit is set.
 *  \param   ldoSlowRampEn       Enable/Disable Slow Ramp for LDO
 *                               Valid only for TPS6594X Leo. For Valid Values
 *                               \ref Pmic_TPS6594x_Regulator_Ldo_Slow_Ramp.
 *                               Valid only when
 *                               PMIC_CFG_REGULATOR_LDO_SLOW_RAMP_EN_VALID
 *                               bit is set.
 *                               Valid only for TPS6594x Leo PMIC PG2.0
 *  \param   ldoBypassModeEn     Selects Bypass/Linear Regulator LDO mode.
 *                               Valid only for TPS6594X Leo. For Valid Values
 *                               \ref Pmic_TPS6594x_Regulator_Ldo_Mode.
 *                               Valid only when
 *                               PMIC_CFG_REGULATOR_LDO_BYPASS_MODE_EN_VALID
 *                               bit is set.
 *  \param   vccaPwrGudLvl       Powergood level for VCCA pin.
 *                               Valid values for TPS6594x Leo Device
 *                               \ref Pmic_TPS6594x_VccaPowerGoodLimit.
 *                               Valid values for LP8764x HERA Device
 *                               \ref Pmic_LP8764x_VccaPowerGoodLimit.
 *                               Valid only when
 *                               PMIC_CFG_VCCA_PWR_GOOD_LVL_VALID bit is set.
 *  \param   vmonRange           Select OV/UV voltage monitoring range for
 *                               VMON
 *                               Valid only for LP8764X Hera. For Valid Values
 *                               \ref Pmic_LP8764x_Power_Vmon_Range.
 *                               Valid only when
 *                               PMIC_CFG_VMON_RANGE_VALID bit is set
 *  \param   buckCurrentLimit    Switch peak current limit for BUCK regulator.
 *                               Valid values for TPS6594x Leo Device
 *                               \ref Pmic_TPS6594x_Buck_Current_Limit.
 *                               Valid values for LP8764x HERA Device
 *                               \ref Pmic_LP8764x_Buck_Current_Limit.
 *                               Valid only when
 *                               PMIC_CFG_REGULATOR_BUCK_ILIM_VALID bit is set.
 *  \param   buckVmonSlewRate    Output voltage slew rate for BUCK/VMON
 *                               Valid values for TPS6594x Leo Device
 *                               \ref Pmic_TPS6594x_Buck_Slew_Rate.
 *                               Valid values for LP8764x HERA Device
 *                               \ref Pmic_LP8764x_Buck_Vmon_Slew_Rate.
 *                               Valid only when
 *                               PMIC_CFG_REGULATOR_BUCK_VMON_SLEW_RATE_VALID
 *                               bit is set
 *  \param   ldoPullDownSel      Selects the resistor value for output
 *                               pull-down resistor for LDO regulator
 *                               Valid only for TPS6594X Leo. For Valid Values
 *                               \ref Pmic_TPS6594x_Ldo_Pldn_Resistor_Val.
 *                               Valid only when
 *                               PMIC_CFG_REGULATOR_LDO_PLDN_SEL_VALID bit is
 *                               set.
 *  \param   ldoRvTimeoutSel     LDO residual voltage check timeout select.
 *                               Valid only for TPS6594X Leo. For Valid Values
 *                               \ref Pmic_TPS6594x_Ldo_RV_Timeout.
 *                               Valid only when
 *                               PMIC_CFG_REGULATOR_LDO_RV_TIMEOUT_SEL_VALID
 *                               bit is set.
 *  \param   pgUvThresholdLvl    Power good low threshold level for the power
 *                               resources.
 *                               Valid values for TPS6594x Leo Device
 *                               \ref Pmic_TPS6594x_Pg_Ov_Uv_Threshold_lvl.
 *                               Valid values for LP8764x HERA Device
 *                               \ref Pmic_LP8764x_Pg_Ov_Uv_Threshold_lvl.
 *                               Valid only when
 *                               PMIC_CFG_PWR_RESOURCE_PG_UV_THRESHOLD_LVL_VALID
 *                               bit is set.
 *  \param   pgOvThresholdLvl    Power good high threshold level for the power
 *                               resources.
 *                               Valid values for TPS6594x Leo Device
 *                               \ref Pmic_TPS6594x_Pg_Ov_Uv_Threshold_lvl.
 *                               Valid values for LP8764x HERA Device
 *                               \ref Pmic_LP8764x_Pg_Ov_Uv_Threshold_lvl.
 *                               PMIC_CFG_PWR_RESOURCE_PG_OV_THRESHOLD_LVL_VALID
 *                               bit is set.
 *  \param   railGrpSel          Rail group selection for the power resources
 *                               Valid values for TPS6594x Leo Device
 *                               \ref Pmic_TPS6594x_Power_Rail_Sel.
 *                               Valid values for LP8764x HERA Device
 *                               \ref Pmic_LP8764x_Power_Rail_Sel.
 *                               Valid only when
 *                               PMIC_CFG_PWR_RESOURCE_RAIL_GRP_SEL_VALID bit is
 *                               set
 *  \param   voltage_mV          For LDO/BUCK: Voltage level in mv.
 *                               For VMON1/VMON2: Powergood voltage level in mv
 *                               Valid only when
 *                               PMIC_CFG_REGULATOR_VMON_VOLTAGE_SET_VALID
 *                               bit is set
 */
typedef struct Pmic_PowerResourceCfg_s
{
    uint32_t validParams;
    bool     rvCheckEn;
    bool     buckPullDownEn;
    bool     vmonEn;
    bool     buckVoutSel;
    bool     buckFpwmMode;
    bool     buckFpwmMpMode;
    bool     regulatorEn;
    bool     ldoSlowRampEn;
    bool     ldoBypassModeEn;
    bool     vccaPwrGudLvl;
    bool     vmonRange;
    uint8_t  buckCurrentLimit;
    uint8_t  buckVmonSlewRate;
    uint8_t  ldoPullDownSel;
    uint8_t  ldoRvTimeoutSel;
    uint8_t  pgUvThresholdLvl;
    uint8_t  pgOvThresholdLvl;
    uint8_t  railGrpSel;
    uint16_t voltage_mV;
}Pmic_PowerResourceCfg_t;

/*!
 *  \anchor  Pmic_PowerCommonCfg_s
 *  \brief   Power configuration
 *           The power control and config structure
 *           Note: validParams is input param for all Set and Get APIs. other
 *           params except validParams is input param for Set APIs and output
 *           param for Get APIs
 *
 *  \param   validParams         Selection of structure parameters to be set,
 *                               from the combination of
 *                               \ref Pmic_PowerCommonParamCfg
 *                               and the corresponding member value must be
 *                               updated
 *
 *  \param   pgoodWindow         Select the type of voltage monitoring for PGOOD
 *                               signal
 *                               For valid values
 *                               \ref Pmic_Power_Good_Window
 *                               Valid only when
 *                               PMIC_POWER_PGOOD_WINDOW_VALID bit set.
 *
 *  \param   pgoodPolarity       Select the PGOOD signal polarity
 *                               For valid values
 *                               Valid values
 *                               \ref Pmic_Power_Good_Polarity
 *                               Valid only when
 *                               PMIC_POWER_PGOOD_POL_VALID bit is set.
 *
 *  \param   deglitchTimeSel     Deglitch time select for all power resources
 *                               Valid values for TPS6594x Leo Device
 *                               Valid only for TPS6594x Leo PMIC PG2.0
 *                               \ref Pmic_TPS6594x_Vmon_DeglitchTime_Sel.
 *                               Valid values for LP8764x HERA Device
 *                               Valid for both LP8764x Hera PMIC PG1.0 and
 *                               PG2.0
 *                               \ref Pmic_LP8764x_Vmon_DeglitchTime_Sel.
 *                               Valid only when
 *                               PMIC_CFG_DEGLITCH_TIME_SEL_VALID bit is set.
 *
 *  \param   severeErrorTrig     Select the trigger selection for severe Error
 *                               For valid values
 *                               \ref Pmic_Power_Trigger_Sel
 *                               Valid only when
 *                               PMIC_SEVERE_ERR_TRIG_VALID bit set.
 *
 *  \param   otherRailTrig       Select the trigger selection for other
 *                               rail group
 *                               For valid values
 *                               \ref Pmic_Power_Trigger_Sel
 *                               Valid only when
 *                               PMIC_OTHER_RAIL_TRIG_VALID bit set.
 *
 *  \param   socRailTrig         Select the trigger selection for soc rail group
 *                               For valid values
 *                               \ref Pmic_Power_Trigger_Sel
 *                               Valid only when
 *                               PMIC_SOC_RAIL_TRIG_VALID bit set.
 *
 *  \param   mcuRailTrig         Select the trigger selection for mcu rail group
 *                               For valid values
 *                               \ref Pmic_Power_Trigger_Sel
 *                               Valid only when
 *                               PMIC_MCU_RAIL_TRIG_VALID bit set.
 *
 *  \param   moderateRailTrig    Select the trigger selection for Moderate Error
 *                               For valid values
 *                               \ref Pmic_Power_Trigger_Sel
 *                               Valid only when
 *                               PMIC_MODERATE_ERR_TRIG_VALID bit set
 *
 */
typedef struct Pmic_PowerCommonCfg_s
{
    uint32_t validParams;
    bool     pgoodWindow;
    bool     pgoodPolarity;
    bool     deglitchTimeSel;
    uint8_t  severeErrorTrig;
    uint8_t  otherRailTrig;
    uint8_t  socRailTrig;
    uint8_t  mcuRailTrig;
    uint8_t  moderateRailTrig;
}Pmic_PowerCommonCfg_t;

/*!
 *  \anchor  Pmic_PowerResourceStat_s
 *  \brief   PMIC power status.
 *           The PMIC power and thermal status structure.
 *           Note: validParams is input param for all Get APIs. other params
 *           except validParams is output param for Get APIs
 *
 *  \param   validParams       Selection of structure parameters to be set,
 *                             from the combination of
 *                             \ref Pmic_PowerStatusValidParamCfg
 *                             and the corresponding member value must be
 *                             updated
 *
 *  \param   currentLimitLvlStat
 *                             Used to read the output current limit status
 *                             for LDO and buck regulators.
 *                             This checks if output current is above current
 *                             limit level.
 *                             For valid values
 *                             Valid values for TPS6594x Leo Device
 *                             \ref Pmic_TPS6594x_Power_Current_Status
 *                             Valid values for LP8764x HERA Device
 *                             \ref Pmic_LP8764x_Power_Current_Status
 *                             Valid only when
 *                             PMIC_POWER_REGULATOR_ILIM_STAT_VALID bit set.
 *
 *  \param   underVoltageTholdStat
 *                             Used to read the output under voltage status
 *                             for LDO/BUCK and input under voltage status for
 *                             VCCA/VMON.
 *                             This is used to read if output/input voltage
 *                             is below under-voltage threshold/level
 *                             VCCA/VMON.
 *                             For valid values
 *                             Valid values for TPS6594x Leo Device
 *                             For Power Regulator(BUCK/LDO)
 *                             \ref Pmic_TPS6594x_Regulator_Under_Voltage_Status
 *                             For VCCA
 *                             \ref Pmic_TPS6594x_Vcca_Under_Voltage_Status
 *                             Valid values for LP8764x HERA Device
 *                             For Power Regulator(BUCK/LDO)
 *                             \ref Pmic_LP8764x_Regulator_Under_Voltage_Status
 *                             For VCCA/VMON
 *                             \ref Pmic_LP8764x_Vcca_Vmon_Under_Voltage_Status
 *                             Valid only when
 *                             PMIC_POWER_RESOURCE_UV_STAT_VALID bit set.
 *
 *  \param   overVoltageTholdStat
 *                             Used to read the output over voltage status
 *                             LDO/BUCK and input over voltage status for
 *                             VCCA/VMON.
 *                             This is used to read if output/input voltage
 *                             is above over-voltage threshold/level
 *
 *                             For valid values
 *                             Valid values for TPS6594x Leo Device
 *                             For Power Regulator(BUCK/LDO)
 *                             \ref Pmic_TPS6594x_Regulator_Over_Voltage_Status
 *                             For VCCA/VMON
 *                             \ref Pmic_TPS6594x_Vcca_Over_Voltage_Status
 *                             Valid values for LP8764x HERA Device
 *                             For Power Regulator(BUCK/LDO)
 *                             \ref Pmic_LP8764x_Regulator_Over_Voltage_Status
 *                             For VCCA/VMON
 *                             \ref Pmic_LP8764x_Vcca_Vmon_Over_Voltage_Status
 *                             Valid only when
 *                             PMIC_POWER_RESOURCE_OV_STAT_VALID bit set.
 *
 *  \param   overVoltageProtectionLvlStat
 *                             Used to read the voltage level status for
 *                             VCCA.
 *                             This is used to read if voltage is above
 *                             overvoltage protection level.
 *                             For valid values
 *                             Valid values for TPS6594x Leo Device
 *                             \ref Pmic_TPS6594x_Vcca_Voltage_Status
 *                             Valid values for LP8764x HERA Device
 *                             \ref Pmic_LP8764x_Vcca_Voltage_Status
 *                             Valid only when
 *                             PMIC_POWER_VCCA_OV_LVL_STAT_VALID bit set
 */
typedef struct Pmic_PowerResourceStat_s
{
    uint32_t validParams;
    bool     currentLimitLvlStat;
    bool     underVoltageTholdStat;
    bool     overVoltageTholdStat;
    bool     overVoltageProtectionLvlStat;
}Pmic_PowerResourceStat_t;

/*!
 *  \anchor  Pmic_PowerThermalCfg_s
 *  \brief   PMIC Power Thermal configuration structure
 *           Note: validParams is input param for all Set and Get APIs. other
 *           params except validParams is input param for Set APIs and output
 *           param for Get APIs
 *
 *  \param   validParams         Selection of structure parameters to be set,
 *                               from the combination of
 *                               \ref Pmic_PowerThermalThresholdValidParamCfg
 *                               and the corresponding member value must be
 *                               updated.
 *  \param   thermalWarnThold
 *                               Set/Get the thermal Warning Threshold
 *                               temperature value for PMIC.
 *                               For valid values
 *                               Valid values for TPS6594x Leo Device PG 1.0
 *                               \ref Pmic_TPS6594x_Pwr_Thermal_Warn_Lvl_PG_1_0
 *                               Valid values for TPS6594x Leo Device PG 2.0
 *                               \ref Pmic_TPS6594x_Pwr_Thermal_Warn_Lvl_PG_2_0
 *                               Valid values for LP8764x HERA Device
 *                               \ref Pmic_LP8764x_Pwr_Thermal_Warn_Lvl
 *                               Valid only when
 *                               PMIC_THERMAL_WARN_VALID bit is set
 *
 *  \param   thermalShutdownThold
 *                               Set/Get the Thermal Shutdown Threshold
 *                               temperature value for PMIC.
 *                               Only supported by TPS6594x Leo PMIC PG2.0
 *                               and LP8764x Hera PMIC PG1.0 and PG2.0
 *                               For valid values
 *                               Valid values for TPS6594x Leo Device PG2.0
 *                               \ref Pmic_TPS6594x_Power_Thermal_Shutdown_Level
 *                               Valid values for LP8764x Leo Device PG2.0 and
 *                               PG1.0
 *                               \ref Pmic_LP8764x_Power_Thermal_Shutdown_Level
 *                               Valid only when
 *                               PMIC_THERMAL_SHTDWN_VALID bit of
 *                               validParams is set.
 *
 */
typedef struct Pmic_PowerThermalCfg_s
{
    uint32_t validParams;
    bool     thermalWarnThold;
    bool     thermalShutdownThold;
}Pmic_PowerThermalCfg_t;

/*!
 *  \anchor  Pmic_PowerThermalStat_t
 *  \brief   PMIC Power Thermal status structure
 *           Note: validParams is input param for all Get APIs. other params
 *           except validParams is output param for Get APIs
 *
 *  \param   validParams         Selection of structure parameters to be set,
 *                               from the combination of
 *                               \ref Pmic_PowerThermalStatValidParamCfg
 *                               and the corresponding member value must be
 *                               updated.
 *  \param   thermalStateWarning
 *                               Set/Get the Thermal warning status
 *                               Status bit indicating that die junction
 *                               temperature is above the thermal warning level.
 *                               Valid only when
 *                               PMIC_THERMAL_STAT_WARN_VALID bit is set
 *
 *  \param   thermalStateOderlyShtDwn
 *                               Set/Get the Orderly Shutdown status
 *                               Status bit indicating that the die junction
 *                               temperature is above the thermal level causing
 *                               a sequenced shutdown.
 *                               Valid only when
 *                               PMIC_THERMAL_STAT_ORD_SHTDWN_VALID bit is set
 *
 *  \param   thermalStateImmShtDwn
 *                               Set/Get the Immediate Shutdown status
 *                               Status bit indicating that the die junction
 *                               temperature is above the thermal level causing
 *                               an immediate shutdown
 *                               Valid only when
 *                               PMIC_THERMAL_STAT_IMM_SHTDWN_VALID bit is set
 *
 */
typedef struct Pmic_PowerThermalStat_s
{
    uint16_t validParams;
    bool     thermalStateWarning;
    bool     thermalStateOderlyShtDwn;
    bool     thermalStateImmShtDwn;
}Pmic_PowerThermalStat_t;

/*==========================================================================*/
/*                         Function Declarations                            */
/*==========================================================================*/

/**
 * \brief   API to set power resources configurations.
 *
 * Requirement: REQ_TAG(PDK-5829), REQ_TAG(PDK-5841), REQ_TAG(PDK-5848),
 *              REQ_TAG(PDK-9111), REQ_TAG(PDK-9163), REQ_TAG(PDK-9149),
 *              REQ_TAG(PDK-9159), REQ_TAG(PDK-9329)
 * Design: did_pmic_power_cfg_readback
 * Architecture: aid_pmic_power_cfg
 *
 *          This function can be used to configure the various control and
 *          configuration parameters for BUCK/LDO/VCCA/VMON power resources and
 *          also used to set the various control and configuration of
 *          voltage monitor parameters for BUCK/LDO/VCCA/VMON power resources
 *          when corresponding validParam bit field is set in the
 *          Pmic_PowerResourceCfg_t structure.
 *          For more information \ref Pmic_PowerResourceCfg_t
 *
 *          To set control and configuration params for BUCK, the application
 *          has to configure the below defined structure members of
 *          Pmic_PowerResourceCfg_t:
 *          rvCheckEn, buckPullDownEn, vmonEn, buckVoutRegSel, buckFpwmMode,
 *          buckFpwmMpMode, regulatorEn, buckCurrentLimit,
 *          buckVmonSlewRate, voltage_mV, pgUvThresholdLvl,
 *          pgOvThresholdLvl, railGrpSel
 *
 *          To set control and configuration params for LDO, the application
 *          has to configure the below defined structure members of
 *          Pmic_PowerResourceCfg_t:
 *          rvCheckEn, vmonEn, regulatorEn, ldoPullDownSel, ldoSlowRampEn,
 *          ldoBypassModeEn, ldoRvTimeoutSel, voltage_mV,
 *          pgUvThresholdLvl, pgOvThresholdLvl, railGrpSel
 *
 *          To set control and configuration params for VCCA, the application
 *          has to configure the below defined structure members of
 *          Pmic_PowerResourceCfg_t:
 *          vmonEn, vccaPwrGudLvl, pgUvThresholdLvl,
 *          pgOvThresholdLvl, railGrpSel
 *
 *          To set control and configuration params for VMON, the application
 *          has to configure the below defined structure members of
 *          Pmic_PowerResourceCfg_t:
 *          rvCheckEn, vmonEn, vccaPwrGudLvl, vmonRange,
 *          pgUvThresholdLvl, pgOvThresholdLvl, railGrpSel, voltage_mV
 *          Valid only for LP8764x HERA Device
 *          Note: Application has to ensure configured regulator voltage is
 *                within the operating voltages of the connected component.If
 *                not configured properly then it may break the system or
 *                component
 *                Application has to ensure that external component connected to
 *                PMIC device will generates volatge/current/power is within the
 *                configured limits of PMIC device.If not configured properly
 *                then it may damage the PMIC device
 *
 * \param   pPmicCoreHandle    [IN]    PMIC Interface Handle.
 * \param   pwrResource        [IN]    PMIC Power resource
 *                                     Valid values for TPS6594x Leo Device
 *                                     \ref Pmic_Tps6594xLeo_Power_Resource.
 *                                     Valid values for LP8764x HERA Device
 *                                     \ref Pmic_Lp8764xHera_Power_Resource.
 * \param   pwrResourceCfg     [IN]    Power Resource configuration for
 *                                     BUCK/LDO/VMON/VCCA
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code.
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_powerSetPwrResourceCfg(
                             Pmic_CoreHandle_t                 *pPmicCoreHandle,
                             const uint16_t                     pwrResource,
                             const Pmic_PowerResourceCfg_t      pwrResourceCfg);

/**
 * \brief   API to get power resources configurations.
 *
 * Requirement: REQ_TAG(PDK-5829), REQ_TAG(PDK-5848), REQ_TAG(PDK-5850),
 *              REQ_TAG(PDK-9163)
 * Design: did_pmic_power_cfg_readback
 * Architecture: aid_pmic_power_cfg
 *
 *          This function can be used to get the various control and
 *          configuration parameters for BUCK/LDO/VCCA/VMON power resources and
 *          also used to get the various control and configuration of
 *          voltage monitor parameters for BUCK/LDO/VCCA/VMON power resources
 *          when corresponding validParam bit field is set in the
 *          Pmic_PowerResourceCfg_t structure.
 *          For more information \ref Pmic_PowerResourceCfg_t
 *
 *          Application can get these control and configuration params for BUCK,
 *          which is stored in the below defined structure members of
 *          Pmic_PowerResourceCfg_t:
 *          rvCheckEn, buckPullDownEn, vmonEn, buckVoutRegSel, buckFpwmMode,
 *          buckFpwmMpMode, regulatorEn, buckCurrentLimit,
 *          buckVmonSlewRate, voltage_mV, pgUvThresholdLvl,
 *          pgOvThresholdLvl, railGrpSel
 *
 *          Application can get these control and configuration params for LDO,
 *          which is stored in the below defined structure members of
 *          Pmic_PowerResourceCfg_t:
 *          rvCheckEn, vmonEn, regulatorEn, ldoPullDownSel, ldoSlowRampEn,
 *          ldoBypassModeEn, ldoRvTimeoutSel, voltage_mV,
 *          pgUvThresholdLvl, pgOvThresholdLvl, railGrpSel
 *
 *          Application can get these control and configuration params for VCCA,
 *          which is stored in the below defined structure members of
 *          Pmic_PowerResourceCfg_t:
 *          vmonEn, vccaPwrGudLvl, pgUvThresholdLvl,
 *          pgOvThresholdLvl, railGrpSel
 *
 *          Application can get these control and configuration params for VMON,
 *          which is stored in the below defined structure members of
 *          Pmic_PowerResourceCfg_t:
 *          rvCheckEn, vmonEn, vccaPwrGudLvl, vmonRange,
 *          pgUvThresholdLvl, pgOvThresholdLvl, railGrpSel, voltage_mV
 *          Valid only for LP8764x HERA Device
 *
 * \param   pPmicCoreHandle    [IN]      PMIC Interface Handle.
 * \param   pwrResource        [IN]      PMIC Power resource
 *                                       Valid values for TPS6594x Leo Device
 *                                       \ref Pmic_Tps6594xLeo_Power_Resource.
 *                                       Valid values for LP8764x HERA Device
 *                                       \ref Pmic_Lp8764xHera_Power_Resource.
 * \param   pPwrResourceCfg    [IN/OUT]  Pointer to store Power Resource
 *                                       configuration for BUCK/LDO/VMON/VCCA
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code.
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_powerGetPwrResourceCfg(
                                 Pmic_CoreHandle_t           *pPmicCoreHandle,
                                 const uint16_t               pwrResource,
                                 Pmic_PowerResourceCfg_t     *pPwrResourceCfg);

/**
 * \brief   API to Set Power configuration
 *
 * Requirement: REQ_TAG(PDK-5829), REQ_TAG(PDK-5848), REQ_TAG(PDK-5847),
 *              REQ_TAG(PDK-9111), REQ_TAG(PDK-9149), REQ_TAG(PDK-9159),
 *              REQ_TAG(PDK-9329)
 * Design: did_pmic_power_cfg_readback, did_pmic_power_pgood_cfg_readback
 * Architecture: aid_pmic_power_cfg
 *
 *          This function is used to set the power configuration
 *          parameters such as selection of type of voltage monitoring, and
 *          polarity of the power-good signal, deglitch time select for all
 *          power resources when corresponding validParam bit field is set in
 *          the Pmic_PowerCommonCfg_t structure.
 *          For more information \ref Pmic_PowerCommonCfg_t
 *
 *          Application can set the voltage monitoring for PGOOD
 *          by configuring the following structure members of
 *          Pmic_PowerCommonCfg_t:
 *          pgoodWindow
 *
 *          Application can set the PGOOD signal polarity
 *          by configuring the following structure members of
 *          Pmic_PowerCommonCfg_t:
 *          pgoodPolarity
 *
 *          Application can set the Deglitch time select for all power resources
 *          by configuring the following structure members of
 *          Pmic_PowerCommonCfg_t:
 *          deglitchTimeSel
 *
 *          Application can set/select trigger selection for :
 *          severe Error, other rail group, soc rail group, mcu rail group and
 *          Moderate Error
 *          by configuring the following structure members of
 *          Pmic_PowerCommonCfg_t:
 *          severeErrorTrig, otherRailTrig, socRailTrig, mcuRailTrig
 *          moderateRailTrig
 *
 *
 * \param   pPmicCoreHandle  [IN]    PMIC Interface Handle.
 * \param   powerCommonCfg   [IN]    Power configuration.
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code.
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_powerSetCommonConfig(Pmic_CoreHandle_t           *pPmicCoreHandle,
                                  const Pmic_PowerCommonCfg_t  powerCommonCfg);

/**
 * \brief   API to Get Power configuration
 *
 * Requirement: REQ_TAG(PDK-5829), REQ_TAG(PDK-5848), REQ_TAG(PDK_5847)
 * Design: did_pmic_power_cfg_readback, did_pmic_power_pgood_cfg_readback
 * Architecture: aid_pmic_power_cfg
 *
 *          This function is used to get the power configuration
 *          parameters such as selection of type of voltage monitoring, and
 *          polarity of the power-good signal, deglitch time select for all
 *          power resources when corresponding validParam bit field is set in
 *          the Pmic_PowerCommonCfg_t structure.
 *          For more information \ref Pmic_PowerCommonCfg_t
 *
 *          Application can get the voltage monitoring for PGOOD
 *          which is stored in the below defined structure members of
 *          Pmic_PowerCommonCfg_t:
 *          pgoodWindow
 *
 *          Application can get the PGOOD signal polarity
 *          which is stored in the below defined structure members of
 *          Pmic_PowerCommonCfg_t:
 *          pgoodPolarity
 *
 *          Application can get the Deglitch time select for all power resources
 *          which is stored in the below defined structure members of
 *          Pmic_PowerCommonCfg_t:
 *          deglitchTimeSel
 *
 *          Application can get trigger selection for :
 *          severe Error, other rail group, soc rail group, mcu rail group and
 *          Moderate Error
 *          which is stored in the below defined structure members of
 *          Pmic_PowerCommonCfg_t:
 *          severeErrorTrig, otherRailTrig, socRailTrig, mcuRailTrig
 *          moderateRailTrig
 *
 * \param   pPmicCoreHandle  [IN]       PMIC Interface Handle.
 * \param   pPowerCommonCfg  [IN/OUT]   Pointer to hold Power configuration.
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code.
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_powerGetCommonConfig(Pmic_CoreHandle_t     *pPmicCoreHandle,
                                  Pmic_PowerCommonCfg_t *pPowerCommonCfg);

/**
 * \brief   API to Set Power good configuration
 *
 * Requirement: REQ_TAG(PDK-5847), REQ_TAG(PDK-9111)
 * Design: did_pmic_power_pgood_cfg_readback
 * Architecture: aid_pmic_power_cfg
 *
 *          This function is used to control and configure the power good
 *          source control. For the
 *          following, power good signal control can be selected:
 *          All supported Bucks and Ldo by the PMIC, VCCA , thermal warning,
 *          nRSTOUT pin and nRSTOUT_SOC pin.
 *
 * \param   pPmicCoreHandle  [IN]     PMIC Interface Handle.
 * \param   pgoodSrcSel      [IN]     Power Good Source.
 *                                    Valid values for TPS6594x Leo Device
 *                                    \ref Pmic_Tps6594xLeo_Pgood_Source.
 *                                    Valid values for LP8764x HERA Device
 *                                    \ref Pmic_Lp8764xHera_Pgood_Source.
 * \param   pgoodSelType     [IN]     Power Good configuration.
 *                                    Valid values for TPS6594x Leo Device:
 *                                For LDO/BUCK:
 *                                \ref Pmic_TPS6594x_Power_Good_Regulator_Signal
 *                                For VCCA:
 *                                    \ref Pmic_TPS6594x_Power_Good_Vcca
 *                                For Thermal Warning
 *                                    \ref Pmic_TPS6594x_Power_Good_Thermal_Warn
 *                                For nRSTOUT:
 *                                    \ref Pmic_TPS6594x_Power_Good_Nrstout
 *                                For nRSTOUT_SOC:
 *                                    \ref Pmic_TPS6594x_Power_Good_Nrstout_Soc
 *                                    Valid values for LP8764x HERA Device
 *                                For BUCK:
 *                                    \ref Pmic_LP8764x_Power_Good_Buck_Signal
 *                                For VCCA/VMON:
 *                                    \ref Pmic_LP8764x_Power_Good_Vcca_Vmon
 *                                For Thermal Warning:
 *                                    \ref Pmic_LP8764x_Power_Good_Thermal_Warn
 *                                For nRSTOUT
 *                                    \ref Pmic_LP8764x_Power_Good_Nrstout
 *                                For nRSTOUT_SOC:
 *                                    \ref Pmic_LP8764x_Power_Good_Nrstout_Soc
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code.
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_powerSetConfigPowerGood(Pmic_CoreHandle_t *pPmicCoreHandle,
                                     const uint16_t     pgoodSrcSel,
                                     const uint8_t      pgoodSelType);

/**
 * \brief   Get Power good configuration
 *
 * Requirement: REQ_TAG(PDK-5847)
 * Design: did_pmic_power_pgood_cfg_readback
 * Architecture: aid_pmic_power_cfg
 *
 *          This function is used to get various power good conifg.
 *          This function also provides the pgood source control for
 *          different power resources and pins.
 *
 * \param   pPmicCoreHandle  [IN]        PMIC Interface Handle.
 * \param   pgoodSrcSel      [IN]        Power Good Source.
 *                                       Valid values for TPS6594x Leo Device
 *                                       \ref Pmic_Tps6594xLeo_Pgood_Source.
 *                                       Valid values for LP8764x HERA Device
 *                                       \ref Pmic_Lp8764xHera_Pgood_Source.
 * \param   pPgoodSelType    [OUT]       Power Good configuration.
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code.
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_powerGetConfigPowerGood(Pmic_CoreHandle_t  *pPmicCoreHandle,
                                     const uint16_t      pgoodSrcSel,
                                     uint8_t            *pPgoodSelType);

/**
 * \brief   API to get power resources status.
 *
 * Requirement: REQ_TAG(PDK-5829), REQ_TAG(PDK-5848), REQ_TAG(PDK-5850)
 * Design: did_pmic_power_cfg_readback
 * Architecture: aid_pmic_power_cfg
 *
 *          This function can be used to get the status related to current limit
 *          , voltage over and under limit for BUCK/LDO/VCCA/VMON power
 *          resources when corresponding validParam bit field is set in
 *          the Pmic_PowerResourceStat_t
 *          For more information \ref Pmic_PowerResourceStat_t
 *
 *          Application can get the current limit status that if the output
 *          current is above current limit level for BUCK which is stored in the
 *          below defined structure members of
 *          Pmic_PowerResourceStat_t:
 *          currentLimitLvlStat
 *
 *          Application can get the output voltage status that if the
 *          output voltage is below undervoltage threshold for BUCK/LDO which is
 *          stored in the below defined structure members of
 *          Pmic_PowerResourceStat_t :
 *          underVoltageTholdStat
 *          For VMON/VCCA the same member is used to get the input voltage
 *          status that if input voltage is below undervoltage level.
 *
 *          Application can get the output voltage status that if the
 *          output voltage is above overvoltage threshold for BUCK/LDO which is
 *          stored in the below defined structure members of
 *          Pmic_PowerResourceStat_t :
 *          overVoltageTholdStat
 *          For VMON/VCCA the same member is used to get the input voltage
 *          status that if input voltage is above overvoltage level.
 *
 *          Application can get the VCCA voltage status that if the VCCA
 *          voltage is above overvoltage protection level which is stored in
 *          the below defined structure members of
 *          Pmic_PowerResourceStat_t :
 *          overVoltageProtectionLvlStat
 *
 * \param   pPmicCoreHandle    [IN]       PMIC Interface Handle.
 * \param   pwrResource        [IN]       PMIC Power resource
 *                                        Valid values for TPS6594x Leo Device
 *                                        \ref Pmic_Tps6594xLeo_Power_Resource.
 *                                        Valid values for LP8764x HERA Device
 *                                        \ref Pmic_Lp8764xHera_Power_Resource.
 * \param   pPwrRsrcStatCfg    [IN/OUT]   Pointer to store Power Resource
 *                                        Status for BUCK/LDO/VMON/VCCA
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code.
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_powerGetPwrRsrcStat(Pmic_CoreHandle_t        *pPmicCoreHandle,
                                 const uint16_t            pwrResource,
                                 Pmic_PowerResourceStat_t *pPwrRsrcStatCfg);
/**
 * \brief   API to get PMIC die temperature thermal status.
 *
 * Requirement: REQ_TAG(PDK-5840)
 * Design: did_pmic_power_thermal_cfg_readback
 * Architecture: aid_pmic_power_cfg
 *
 *          This function is used to get the thermal status of the PMIC
 *          (die temperature) when corresponding validParam bit field is set in
 *          the Pmic_PowerThermalStat_t
 *          For more information \ref Pmic_PowerThermalStat_t
 *
 *          Application can get the thermal status that if the  die junction
 *          above the thermal warning level  which is stored in
 *          the below defined structure members of
 *          Pmic_PowerThermalStat_t:
 *          thermalStateWarning
 *
 *          Application can get the thermal status that if the  die junction
 *          above the thermal level causing a sequenced shutdown which is
 *          stored in the below defined structure members of
 *          Pmic_PowerThermalStat_t:
 *          thermalStateOderlyShtDwn
 *
 *          Application can get the thermal status that if the  die junction
 *          above the thermal level causing an immediate shutdown which is
 *          stored in the below defined structure members of
 *          Pmic_PowerThermalStat_t:
 *          thermalStateImmShtDwn
 *
 * \param   pPmicCoreHandle       [IN]       PMIC Interface Handle.
 * \param   pPwrThermalStatCfg    [IN/OUT]   Pointer to store Thermal
 *                                           configuration for PMIC
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code.
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_powerGetPwrThermalStat(
                                   Pmic_CoreHandle_t       *pPmicCoreHandle,
                                   Pmic_PowerThermalStat_t *pPwrThermalStatCfg);

/**
 * \brief    API to configure the thermal temperature threshold level for PMIC.
 *
 * Requirement: REQ_TAG(PDK-5840), REQ_TAG(PDK-9111), REQ_TAG(PDK-9117)
 * Design: did_pmic_power_thermal_cfg_readback
 * Architecture: aid_pmic_power_cfg
 *
 *           This function is used to set the thermal threshold level for PMIC
 *          (die temperature) when corresponding validParam bit field is set in
 *          the Pmic_PowerThermalCfg_t
 *          For more information \ref Pmic_PowerThermalCfg_t
 *
 *          To configure the the thermal warning threshold temperature level,
 *          the application has to configure the below defined structure
 *          member of the Pmic_PowerThermalCfg_t:
 *          thermalWarnThold
 *
 *          To configure the the thermal shutdown threshold temperature level,
 *          the application has to configure the below defined structure member
 *          of the Pmic_PowerThermalCfg_t:
 *          thermalShutdownThold
 *
 * \param   pPmicCoreHandle       [IN]    PMIC Interface Handle.
 * \param   thermalThreshold      [IN]    Thermal Configuration.
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code.
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_powerSetThermalConfig(
                                Pmic_CoreHandle_t           *pPmicCoreHandle,
                                const Pmic_PowerThermalCfg_t thermalThreshold);

/**
 * \brief   Get the PMIC thermal threshold value function.
 *
 * Requirement: REQ_TAG(PDK-5840), REQ_TAG(PDK-9117)
 * Design: did_pmic_power_thermal_cfg_readback
 * Architecture: aid_pmic_power_cfg
 *
 *          This function is used to get the thermal temperature threshold
 *          value for the PMIC when corresponding validParam bit field is set in
 *          the Pmic_PowerThermalCfg_t
 *          For more information \ref Pmic_PowerThermalCfg_t
 *
 *          Application can get the the thermal wrarning threshold temperature
 *          level which is stored in the below defined structure member of
 *          member of the Pmic_PowerThermalCfg_t:
 *          thermalWarnThold
 *
 *          Application can get the the thermal shutdown threshold temperature
 *          level which is stored in the below defined structure member
 *          of the Pmic_PowerThermalCfg_t:
 *          thermalShutdownThold
 *
 * \param   pPmicCoreHandle       [IN]        PMIC Interface Handle.
 * \param   pThermalThreshold     [IN/OUT]    Pointer to hold Thermal Cfg
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code.
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_powerGetThermalConfig(Pmic_CoreHandle_t      *pPmicCoreHandle,
                                   Pmic_PowerThermalCfg_t *pThermalThreshold);

/*!
 * \brief   API to enable/disable Power interrupt.
 *
 * Requirement: REQ_TAG(PDK-5829), REQ_TAG(PDK-5848)
 * Design: did_pmic_power_cfg_readback
 * Architecture: aid_pmic_power_cfg
 *
 *          This function is used to enable/disable Power Resource Interrupts
 *
 *
 * \param   pPmicCoreHandle    [IN]    PMIC Interface Handle.
 * \param   pwrResource        [IN]    PMIC Power resource
 *                                     Valid values for TPS6594x Leo Device
 *                                     \ref Pmic_Tps6594xLeo_Power_Resource.
 *                                     Valid values for LP8764x HERA Device
 *                                     \ref Pmic_Lp8764xHera_Power_Resource
 * \param   intrType           [IN]    Interrupt type
 *                                     Valid values for TPS6594x Leo Device
 *                                     \ref Pmic_Tps6594x_PowerInterruptType
 *                                     Valid values for LP8764x HERA Device
 *                                     \ref Pmic_LP8764x_PowerInterruptType
 * \param   intrEnable         [IN]    Enable/Disable the interrupt.
 *                                     For Vaild values:
 *                                     \ref Pmic_PowerInterruptCfg
 *
 * \return  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_powerSetPwrRsrcIntr(Pmic_CoreHandle_t *pPmicCoreHandle,
                                 const uint16_t     pwrResource,
                                 const uint8_t      intrType,
                                 const bool         intrEnable);

/*!
 * \brief   API to enable/disable Power interrupt.
 *
 * Requirement: REQ_TAG(PDK-5841), REQ_TAG(PDK-5840)
 * Design: did_pmic_power_cfg_readback, did_pmic_power_thermal_cfg_readback
 * Architecture: aid_pmic_power_cfg
 *
 *          This function is used to enable/disable power Interrupts
 *
 * \param   pPmicCoreHandle  [IN]    PMIC Interface Handle.
 * \param   intrType         [IN]    Interrupt type
 *                                   Valid values for TPS6594x Leo Device
 *                                   \ref Pmic_Tps6594x_PowerInterruptCommonType
 *                                   Valid values for LP8764x HERA Device
 *                                   \ref Pmic_LP8764x_PowerInterruptCommonType
 * \param   intrEnable       [IN]    Enable/Disable the interrupt.
 *                                   For Vaild values:
 *                                   \ref Pmic_PowerInterruptCfg
 *
 * \return  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_powerSetIntr(Pmic_CoreHandle_t *pPmicCoreHandle,
                          const uint8_t      intrType,
                          const bool         intrEnable);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif/* PMIC_POWER_H_ */

/* @} */

