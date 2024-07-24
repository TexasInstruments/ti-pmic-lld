/******************************************************************************
 * Copyright (c) 2024 Texas Instruments Incorporated - http://www.ti.com
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
 * @file pmic_power.h
 *
 * @brief PMIC LLD Power module header file.
 *
 * @details This module contains declarations/definitions of macros, data
 * structures, and APIs used to interact with PMIC regulators and related
 * components. Some components of the PMIC Power module are as follows:
 * setting/getting regulator configurations, setting/getting thermal
 * shutdown configurations, and getting regulator and TSD statuses.
 */
#ifndef __PMIC_POWER_H__
#define __PMIC_POWER_H__

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                Include Files                               */
/* ========================================================================== */
#include <stdint.h>

#include "pmic_common.h"

/* ========================================================================== */
/*                               Macros & Typedefs                            */
/* ========================================================================== */

/**
 * @anchor Pmic_PowerResources
 * @name TPS65036x Power Resources
 *
 * @brief Valid power resources for the TPS65036x PMIC.
 */
#define PMIC_BUCK1                                  ((uint8_t)1U)
#define PMIC_BUCK2                                  ((uint8_t)2U)
#define PMIC_BUCK3                                  ((uint8_t)3U)
#define PMIC_BUCK_MAX                               (PMIC_BUCK3)
#define PMIC_LDO                                    ((uint8_t)4U)
#define PMIC_POWER_RESOURCE_MAX                     (PMIC_LDO)
/** @} */

/**
 * @anchor Pmic_PwrBuckCfgValidParams
 * @name TPS65036x Buck Configuration Valid Parameters
 *
 * @brief Indication of which parameters are valid within the Pmic_PwrBuckCfg_t
 * struct.
 *
 * @details For more information on the parameters, refer to @ref Pmic_PwrBuckCfg.
 *
 * @note End-user could combine multiple valid parameters using the OR operator.
 *
 * @{
 */
#define PMIC_BUCK_ENABLE_VALID                      ((uint32_t)(1U << 0U))
#define PMIC_BUCK_PLDN_EN_VALID                     ((uint32_t)(1U << 1U))
#define PMIC_BUCK_FPWM_EN_VALID                     ((uint32_t)(1U << 2U))
#define PMIC_BUCK_UV_THR_VALID                      ((uint32_t)(1U << 3U))
#define PMIC_BUCK_OV_THR_VALID                      ((uint32_t)(1U << 4U))
#define PMIC_BUCK_ILIM_SEL_VALID                    ((uint32_t)(1U << 5U))
#define PMIC_BUCK_OVP_SEL_VALID                     ((uint32_t)(1U << 6U))
#define PMIC_BUCK_OV_SEL_VALID                      ((uint32_t)(1U << 7U))
#define PMIC_BUCK_UV_SEL_VALID                      ((uint32_t)(1U << 8U))
#define PMIC_BUCK_SC_SEL_VALID                      ((uint32_t)(1U << 9U))
#define PMIC_BUCK_RV_CONF_VALID                     ((uint32_t)(1U << 10U))
#define PMIC_BUCK_SLEW_RATE_VALID                   ((uint32_t)(1U << 11U))
#define PMIC_BUCK_DEGLITCH_SEL_VALID                ((uint32_t)(1U << 12U))
#define PMIC_BUCK_DISCHARGE_SEL_VALID               ((uint32_t)(1U << 13U))
#define PMIC_BUCK_SS_EN_VALID                       ((uint32_t)(1U << 14U))
#define PMIC_BUCK_SSM_SEL_VALID                     ((uint32_t)(1U << 15U))
#define PMIC_BUCK_VSET_VALID                        ((uint32_t)(1U << 16U))
#define PMIC_BUCK_UVLO_RISING_VALID                 ((uint32_t)(1U << 17U))
#define PMIC_BUCK_UVLO_FALLING_VALID                ((uint32_t)(1U << 18U))
#define PMIC_BUCK_HIGH_SIDE_SLEW_RATE_VALID         ((uint32_t)(1U << 19U))
#define PMIC_BUCK_VSET_ACTIVE_VALID                 ((uint32_t)(1U << 20U))
#define PMIC_BUCK_VSET_LPWR_VALID                   ((uint32_t)(1U << 21U))
#define PMIC_BUCK_VMON_ONLY_VALID                   ((uint32_t)(1U << 22U))
/** @} */

/**
 * @anchor Pmic_PwrLdoCfgValidParams
 * @name TPS65036x LDO Configuration Valid Parameters
 *
 * @brief Indication of which parameters are valid within the Pmic_PwrLdoCfg_t
 * struct.
 *
 * @details For more information on the parameters, refer to @ref Pmic_PwrLdoCfg.
 *
 * @note End-user could combine multiple valid parameters using the OR operator.
 *
 * @{
 */
#define PMIC_LDO_ENABLE_VALID                       ((uint32_t)(1U << 0U))
#define PMIC_LDO_MODE_VALID                         ((uint32_t)(1U << 1U))
#define PMIC_LDO_VSET_VALID                         ((uint32_t)(1U << 2U))
#define PMIC_LDO_VMON_ONLY_VALID                    ((uint32_t)(1U << 3U))
#define PMIC_LDO_DISCHARGE_EN_VALID                 ((uint32_t)(1U << 4U))
#define PMIC_LDO_DISCHARGE_SEL_VALID                ((uint32_t)(1U << 5U))
#define PMIC_LDO_DEGLITCH_SEL_VALID                 ((uint32_t)(1U << 6U))
#define PMIC_LDO_UV_THR_VALID                       ((uint32_t)(1U << 7U))
#define PMIC_LDO_OV_THR_VALID                       ((uint32_t)(1U << 8U))
#define PMIC_LDO_ILIM_SEL_VALID                     ((uint32_t)(1U << 9U))
#define PMIC_LDO_OVP_SEL_VALID                      ((uint32_t)(1U << 10U))
#define PMIC_LDO_OV_SEL_VALID                       ((uint32_t)(1U << 11U))
#define PMIC_LDO_UV_SEL_VALID                       ((uint32_t)(1U << 12U))
#define PMIC_LDO_SC_SEL_VALID                       ((uint32_t)(1U << 13U))
#define PMIC_LDO_RV_CONF_VALID                      ((uint32_t)(1U << 14U))
/** @} */

/**
 * @anchor Pmic_PwrTsdCfgValidParams
 * @name TPS65036x Thermal Shutdown Configuration Valid Parameters
 *
 * @brief Indication of which parameters are valid within the Pmic_PwrTsdCfg_t
 * struct.
 *
 * @details For more information on the parameters, refer to @ref Pmic_PwrTsdCfg.
 *
 * @note End-user could combine multiple valid parameters using the OR operator.
 *
 * @{
 */
#define PMIC_TWARN_STAY_IN_SAFE_STATE_VALID         ((uint32_t)(1U << 0U))
#define PMIC_TSD_IMM_LEVEL_VALID                    ((uint32_t)(1U << 1U))
#define PMIC_TWARN_LEVEL_VALID                      ((uint32_t)(1U << 2U))
/** @} */

/**
 * @anchor Pmic_tsdImmLevelValues
 * @name TPS65036x Immediate Thermal Shutdown Level Values
 *
 * @brief Valid values of the TSD_IMM_LEVEL bit field.
 *
 * @{
 */
#define PMIC_TSD_IMM_LEVEL_150C                     ((uint8_t)0U)
#define PMIC_TSD_IMM_LEVEL_160C                     ((uint8_t)1U)
#define PMIC_TSD_IMM_LEVEL_MAX                      (PMIC_TSD_IMM_LEVEL_160C)
/** @} */

/**
 * @anchor Pmic_twarnLevelValues
 * @name TPS65036x TWARN Level Values
 *
 * @brief Valid values of the TWARN_LEVEL bit field.
 *
 * @{
 */
#define PMIC_TWARN_LEVEL_130C                       ((uint8_t)0U)
#define PMIC_TWARN_LEVEL_140C                       ((uint8_t)1U)
#define PMIC_TWARN_LEVEL_MAX                        (PMIC_TWARN_LEVEL_140C)
/** @} */

/**
 * @anchor Pmic_PwrBuckLdoSequenceTriggers
 * @name TPS65036x Buck and LDO Sequence Triggers
 *
 * @brief Valid PMIC regulator sequence triggers.
 *
 * @details The defines are deciphered by the regulator set and get sequence
 * trigger APIs. The most significant 8 bits denote the PMIC power resource.
 * The least significant 8 bits denote the trigger bit positions in the register
 * map.
 */
#define PMIC_BUCK1_TRIGGER_PWR_ON_BIT   (((uint16_t)PMIC_BUCK1 << 8U) | 5U)
#define PMIC_BUCK1_TRIGGER_LDO_PG       (((uint16_t)PMIC_BUCK1 << 8U) | 4U)
#define PMIC_BUCK1_TRIGGER_BUCK3_PG     (((uint16_t)PMIC_BUCK1 << 8U) | 3U)
#define PMIC_BUCK1_TRIGGER_BUCK2_PG     (((uint16_t)PMIC_BUCK1 << 8U) | 2U)
#define PMIC_BUCK1_TRIGGER_GPIO_PIN     (((uint16_t)PMIC_BUCK1 << 8U) | 1U)
#define PMIC_BUCK1_TRIGGER_SEQ_PIN      (((uint16_t)PMIC_BUCK1 << 8U) | 0U)
#define PMIC_BUCK2_TRIGGER_PWR_ON_BIT   (((uint16_t)PMIC_BUCK2 << 8U) | 5U)
#define PMIC_BUCK2_TRIGGER_LDO_PG       (((uint16_t)PMIC_BUCK2 << 8U) | 4U)
#define PMIC_BUCK2_TRIGGER_BUCK3_PG     (((uint16_t)PMIC_BUCK2 << 8U) | 3U)
#define PMIC_BUCK2_TRIGGER_BUCK1_PG     (((uint16_t)PMIC_BUCK2 << 8U) | 2U)
#define PMIC_BUCK2_TRIGGER_GPIO_PIN     (((uint16_t)PMIC_BUCK2 << 8U) | 1U)
#define PMIC_BUCK2_TRIGGER_SEQ_PIN      (((uint16_t)PMIC_BUCK2 << 8U) | 0U)
#define PMIC_BUCK3_TRIGGER_PWR_ON_BIT   (((uint16_t)PMIC_BUCK3 << 8U) | 5U)
#define PMIC_BUCK3_TRIGGER_LDO_PG       (((uint16_t)PMIC_BUCK3 << 8U) | 4U)
#define PMIC_BUCK3_TRIGGER_BUCK2_PG     (((uint16_t)PMIC_BUCK3 << 8U) | 3U)
#define PMIC_BUCK3_TRIGGER_BUCK1_PG     (((uint16_t)PMIC_BUCK3 << 8U) | 2U)
#define PMIC_BUCK3_TRIGGER_GPIO_PIN     (((uint16_t)PMIC_BUCK3 << 8U) | 1U)
#define PMIC_BUCK3_TRIGGER_SEQ_PIN      (((uint16_t)PMIC_BUCK3 << 8U) | 0U)
#define PMIC_LDO_TRIGGER_PWR_ON_BIT     (((uint16_t)PMIC_LDO << 8U) | 5U)
#define PMIC_LDO_TRIGGER_BUCK3_PG       (((uint16_t)PMIC_LDO << 8U) | 4U)
#define PMIC_LDO_TRIGGER_BUCK2_PG       (((uint16_t)PMIC_LDO << 8U) | 3U)
#define PMIC_LDO_TRIGGER_BUCK1_PG       (((uint16_t)PMIC_LDO << 8U) | 2U)
#define PMIC_LDO_TRIGGER_GPIO_PIN       (((uint16_t)PMIC_LDO << 8U) | 1U)
#define PMIC_LDO_TRIGGER_SEQ_PIN        (((uint16_t)PMIC_LDO << 8U) | 0U)
/** @} */

/**
 * @anchor Pmic_PwrBuckLdoSeqDlyValidParams
 * @name TPS65036x Buck and LDO Sequence Delay Valid Parameters
 *
 * @brief Indication of which parameters are valid within the Pmic_PwrBuckLdoSeqDly_t
 * struct.
 *
 * @details For more information on the parameters, refer to @ref Pmic_PwrBuckLdoSeqDly.
 *
 * @note End-user could combine multiple valid parameters using the OR operator.
 *
 * @{
 */
#define PMIC_SEQ_DLY_OFF_VALID                      ((uint32_t)1U << 0U)
#define PMIC_SEQ_DLY_ON_VALID                       ((uint32_t)1U << 1U)
/** @} */

/**
 * @anchor Pmic_ldoBypConfigValues
 * @name TPS65036x LDO Bypass Configuration Values
 *
 * @brief Valid values of the LDO_BYP_CONFIG bit field.
 *
 * @{
 */
#define PMIC_LDO_MODE                               ((uint8_t)0U)
#define PMIC_BYPASS_MODE                            ((uint8_t)1U)
#define PMIC_LDO_BYP_CONFIG_MAX                     (PMIC_BYPASS_MODE)
/** @} */

/**
 * @anchor Pmic_ldoDischargeSelValues
 * @name TPS65036x LDO Discharge Selection Values
 *
 * @brief Valid values of the LDO_DISCHARGE_SEL bit field.
 *
 * @{
 */
#define PMIC_LDO_DISCHARGE_SEL_50K_OHM              ((uint8_t)0U)
#define PMIC_LDO_DISCHARGE_SEL_125_OHM              ((uint8_t)1U)
#define PMIC_LDO_DISCHARGE_SEL_250_OHM              ((uint8_t)2U)
#define PMIC_LDO_DISCHARGE_SEL_500_OHM              ((uint8_t)3U)
#define PMIC_LDO_DISCHARGE_SEL_MAX                  (PMIC_LDO_DISCHARGE_SEL_500_OHM)
/** @} */

/**
 * @anchor Pmic_ldoUvThrValues
 * @name TPS65036x LDO Undervoltage Threshold Values
 *
 * @brief Valid values of the LDO_UV_THR bit field.
 *
 * @{
 */
#define PMIC_LDO_UV_THR_3P5_PCT                     ((uint8_t)0U)
#define PMIC_LDO_UV_THR_4_PCT                       ((uint8_t)1U)
#define PMIC_LDO_UV_THR_4P5_PCT                     ((uint8_t)2U)
#define PMIC_LDO_UV_THR_5_PCT                       ((uint8_t)3U)
#define PMIC_LDO_UV_THR_MAX                         (PMIC_LDO_UV_THR_5_PCT)
/** @} */

/**
 * @anchor Pmic_ldoOvThrValues
 * @name TPS65036x LDO Overvoltage Threshold Values
 *
 * @brief Valid values of the LDO_OV_THR bit field.
 *
 * @{
 */
#define PMIC_LDO_OV_THR_4_PCT                       ((uint8_t)0U)
#define PMIC_LDO_OV_THR_4P5_PCT                     ((uint8_t)1U)
#define PMIC_LDO_OV_THR_5_PCT                       ((uint8_t)2U)
#define PMIC_LDO_OV_THR_5P5_PCT                     ((uint8_t)3U)
#define PMIC_LDO_OV_THR_MAX                         (PMIC_LDO_OV_THR_5P5_PCT)
/** @} */

/**
 * @anchor Pmic_ldoRvConfValues
 * @name TPS65036x LDO Residual Voltage Configuration Values
 *
 * @brief Valid values of the LDO_RV_CONF bit field.
 *
 * @{
 */
#define PMIC_LDO_RV_DISCHARGE                       ((uint8_t)0U)
#define PMIC_LDO_RV_IGNORE                          ((uint8_t)1U)
#define PMIC_LDO_RV_CONF_MAX                        (PMIC_LDO_RV_IGNORE)
/** @} */

/**
 * @anchor Pmic_ldoIlimSelValues
 * @name TPS65036x LDO Current Limit Selection Values
 *
 * @brief Valid values of the LDO_ILIM_SEL bit field.
 *
 * @{
 */
#define PMIC_LDO_ILIM_SEL_200_MA                    ((uint8_t)0U)
#define PMIC_LDO_ILIM_SEL_400_MA                    ((uint8_t)1U)
#define PMIC_LDO_ILIM_SEL_MAX                       (PMIC_LDO_ILIM_SEL_400_MA)
/** @} */

/**
 * @anchor Pmic_ldoDeglitchSelValues
 * @name TPS65036x LDO Deglitch Selection Values
 *
 * @brief Valid values of the LDO_DEGLITCH_SEL bit field.
 *
 * @{
 */
#define PMIC_LDO_DEGLITCH_SEL_4_US                  ((uint8_t)0U)
#define PMIC_LDO_DEGLITCH_SEL_20_US                 ((uint8_t)1U)
#define PMIC_LDO_DEGLITCH_SEL_30_US                 ((uint8_t)2U)
#define PMIC_LDO_DEGLITCH_SEL_50_US                 ((uint8_t)3U)
#define PMIC_LDO_DEGLITCH_SEL_MAX                   (PMIC_LDO_DEGLITCH_SEL_50_US)
/** @} */

/**
 * @anchor Pmic_regulatorVSetRanges
 * @name TPS65036x Regulator VSET Ranges
 *
 * @brief Minimum and maximum VSET values of BUCK1, BUCK2, BUCK3, and LDO.
 *
 * @{
 */
#define PMIC_BUCK1_VSET_MIN                         ((uint8_t)0x00U)
#define PMIC_BUCK1_VSET_MAX                         ((uint8_t)0x0DU)
#define PMIC_BUCK2_3_VSET_MIN                       ((uint8_t)0x04U)
#define PMIC_BUCK2_3_VSET_MAX                       ((uint8_t)0x45U)
#define PMIC_LDO_VSET_MIN                           ((uint8_t)0x18U)
#define PMIC_LDO_VSET_MAX                           ((uint8_t)0x58U)
/** @} */

/**
 * @anchor Pmic_buck1UvloMaxValues
 * @name TPS65036x BUCK1 UVLO Max Values
 *
 * @brief Maximum value of BUCK1_UVLO_FALLING and BUCK1_UVLO_RISING bit fields.
 *
 * @{
 */
#define PMIC_BUCK1_UVLO_FALLING_MAX                 ((uint8_t)0xFU)
#define PMIC_BUCK1_UVLO_RISING_MAX                  ((uint8_t)0xFU)
/** @} */

/**
 * @anchor Pmic_Buck1HighSideSlewRateCtrlValues
 * @name TPS65036x BUCK1 High Side Slew Rate Control Values
 *
 * @brief Valid values of the BUCK1_EN_HS_ON_SR bit field.
 *
 * @{
 */
#define PMIC_HIGH_SIDE_SLEW_RATE_FASTEST            ((uint8_t)0U)
#define PMIC_HIGH_SIDE_SLEW_RATE_SLOWEST            ((uint8_t)1U)
#define PMIC_BUCK1_EN_HS_ON_SR_MAX                  (PMIC_HIGH_SIDE_SLEW_RATE_SLOWEST)
/** @} */

/**
 * @anchor Pmic_BuckDischargeSelValues
 * @name TPS65036x Buck Discharge Select Values
 *
 * @brief Valid values of the BUCKx_DISCHARGE_SEL bit field, where x can be
 * 1, 2, or 3.
 *
 * @{
 */
#define PMIC_BUCK_SLEW_RATE_CONTROLLED              ((uint8_t)0U)
#define PMIC_BUCK_RESISTIVE_DISCHARGE               ((uint8_t)1U)
#define PMIC_BUCK_DISCHARGE_SEL_MAX                 (PMIC_BUCK_RESISTIVE_DISCHARGE)
/** @} */

/**
 * @anchor Pmic_BuckSlewRateValues
 * @name TPS65036x Buck Slew Rate Values
 *
 * @brief Valid values of the BUCKx_SLEW_RATE bit field, where x can be
 * 1, 2, or 3.
 *
 * @{
 */
#define PMIC_BUCK_SLEW_RATE_10_MV_PER_US            ((uint8_t)0U)
#define PMIC_BUCK_SLEW_RATE_5_MV_PER_US             ((uint8_t)1U)
#define PMIC_BUCK_SLEW_RATE_2P5_MV_PER_US           ((uint8_t)2U)
#define PMIC_BUCK_SLEW_RATE_1P25_MV_PER_US          ((uint8_t)3U)
#define PMIC_BUCK_SLEW_RATE_MAX                     (PMIC_BUCK_SLEW_RATE_1P25_MV_PER_US)
/** @} */

/**
 * @anchor Pmic_buckUvThrValues
 * @name TPS65036x Buck UV Threshold Values
 *
 * @brief Valid values of the BUCKx_UV_THR bit field, where x can be 1, 2, or 3.
 *
 * @{
 */
#define PMIC_BUCK_UV_THR_4_PCT                      ((uint8_t)0U)
#define PMIC_BUCK_UV_THR_4P5_PCT                    ((uint8_t)1U)
#define PMIC_BUCK_UV_THR_5_PCT                      ((uint8_t)2U)
#define PMIC_BUCK_UV_THR_5P5_PCT                    ((uint8_t)3U)
#define PMIC_BUCK_UV_THR_MAX                        (PMIC_BUCK_UV_THR_5P5_PCT)
/** @} */

/**
 * @anchor Pmic_buckOvThrValues
 * @name TPS65036x Buck OV Threshold Values
 *
 * @brief Valid values of the BUCKx_OV_THR bit field, where x can be 1, 2, or 3.
 *
 * @{
 */
#define PMIC_BUCK_OV_THR_4_PCT                      ((uint8_t)0U)
#define PMIC_BUCK_OV_THR_4P5_PCT                    ((uint8_t)1U)
#define PMIC_BUCK_OV_THR_5_PCT                      ((uint8_t)2U)
#define PMIC_BUCK_OV_THR_5P5_PCT                    ((uint8_t)3U)
#define PMIC_BUCK_OV_THR_MAX                        (PMIC_BUCK_OV_THR_5P5_PCT)
/** @} */

/**
 * @anchor Pmic_rvConfValues
 * @name TPS65036x Buck Residual Voltage Configuration Values
 *
 * @brief Valid values of the BUCKx_RV_CONF bit field, where x can be 1, 2, or 3.
 *
 * @{
 */
#define PMIC_BUCK_RAIL_DISCHARGE                    ((uint8_t)0U)
#define PMIC_BUCK_RV_IGNORE                         ((uint8_t)1U)
#define PMIC_BUCK_RV_CONF_MAX                       (PMIC_BUCK_RV_IGNORE)
/** @} */

/**
 * @anchor Pmic_buck1IlimSelValues
 * @name TPS65036x BUCK1 Current Limit Selection Values
 *
 * @brief Valid values of the BUCK1_ILIM_SEL bit field.
 *
 * @{
 */
#define PMIC_BUCK1_ILIM_3P1_A                       ((uint8_t)0U)
#define PMIC_BUCK1_ILIM_3P6_A                       ((uint8_t)1U)
#define PMIC_BUCK1_ILIM_MAX                         (PMIC_BUCK1_ILIM_3P6_A)
/** @} */

/**
 * @anchor Pmic_buck2_3IlimSelValues
 * @name TPS65036x BUCK2 and BUCK3 Current Limit Selection Values
 *
 * @brief Valid values of the BUCKx_ILIM_SEL bit field, where x can be 2 or 3.
 *
 * @{
 */
#define PMIC_BUCK2_3_ILIM_4P5_A                     ((uint8_t)0U)
#define PMIC_BUCK2_3_ILIM_3_A                       ((uint8_t)1U)
#define PMIC_BUCK2_3_ILIM_MAX                       (PMIC_BUCK2_3_ILIM_3_A)
/** @} */

/**
 * @anchor Pmic_buckDeglitchSelValues
 * @name TPS65036x Buck Deglitch Selection Values
 *
 * @brief Valid values of the BUCKx_DEGLITCH_SEL bit field, where x can be
 * 1, 2, or 3.
 *
 * @{
 */
#define PMIC_BUCK_DEGLITCH_SEL_10_US                ((uint8_t)0U)
#define PMIC_BUCK_DEGLITCH_SEL_20_US                ((uint8_t)1U)
#define PMIC_BUCK_DEGLITCH_SEL_30_US                ((uint8_t)2U)
#define PMIC_BUCK_DEGLITCH_SEL_50_US                ((uint8_t)3U)
#define PMIC_BUCK_DEGLITCH_SEL_MAX                  (PMIC_BUCK_DEGLITCH_SEL_50_US)
/** @} */

/**
 * @anchor Pmic_regulatorFaultResponses
 * @name TPS65036x Regulator Fault Responses
 *
 * @brief Valid values for regulator fault responses.
 *
 * @{
 */
#define PMIC_ASSERT_NINT_PIN                        ((uint8_t)0U)
#define PMIC_WARM_RESET_AND_ASSERT_NINT_PIN         ((uint8_t)1U)
#define PMIC_ORD_SHUTDOWN_AND_ASSERT_NINT_PIN       ((uint8_t)2U)
#define PMIC_WAIT_PWR_CYCLE_AND_ASSERT_NINT_PIN     ((uint8_t)3U)
#define PMIC_REGULATOR_FAULT_RESPONSE_MAX           (PMIC_WAIT_PWR_CYCLE_AND_ASSERT_NINT_PIN)
/** @} */

/**
 * @anchor Pmic_ssmSelValues
 * @name TPS65036x Spread Spectrum Modulation Values
 *
 * @brief Valid values of the SSM_SEL bit field.
 *
 * @{
 */
#define PMIC_SSMDEPTH_T125                          ((uint8_t)0U)
#define PMIC_SSMDEPTH_T150                          ((uint8_t)1U)
#define PMIC_SSM_SEL_MAX                            (PMIC_SSMDEPTH_T150)
/** @} */

/**
 * @anchor Pmic_PwrSeqDlyValues
 * @name TPS65036x Sequence Delay Values
 *
 * @brief Valid Regulator sequence delay values.
 *
 */
#define PMIC_SEQ_DLY_0_MS                           ((uint8_t)0x0U)
#define PMIC_SEQ_DLY_0P5_MS                         ((uint8_t)0x1U)
#define PMIC_SEQ_DLY_1_MS                           ((uint8_t)0x2U)
#define PMIC_SEQ_DLY_2_MS                           ((uint8_t)0x3U)
#define PMIC_SEQ_DLY_3_MS                           ((uint8_t)0x4U)
#define PMIC_SEQ_DLY_4_MS                           ((uint8_t)0x5U)
#define PMIC_SEQ_DLY_5_MS                           ((uint8_t)0x6U)
#define PMIC_SEQ_DLY_6_MS                           ((uint8_t)0x7U)
#define PMIC_SEQ_DLY_7_MS                           ((uint8_t)0x8U)
#define PMIC_SEQ_DLY_8_MS                           ((uint8_t)0x9U)
#define PMIC_SEQ_DLY_9_MS                           ((uint8_t)0xAU)
#define PMIC_SEQ_DLY_10_MS                          ((uint8_t)0xBU)
#define PMIC_SEQ_DLY_12_MS                          ((uint8_t)0xCU)
#define PMIC_SEQ_DLY_14_MS                          ((uint8_t)0xDU)
#define PMIC_SEQ_DLY_16_MS                          ((uint8_t)0xEU)
#define PMIC_SEQ_DLY_20_MS                          ((uint8_t)0xFU)
#define PMIC_SEQ_DLY_MAX                            (PMIC_SEQ_DLY_20_MS)
/** @} */

/* ========================================================================== */
/*                             Structures and Enums                           */
/* ========================================================================== */

/**
 * @anchor Pmic_PwrBuckCfg
 * @name PMIC Buck Configuration Struct
 *
 * @brief Struct used to set and get PMIC buck regulator configurations.
 *
 * @note BUCK1 configurations are slightly different from BUCK2 and BUCK3. Some
 * configurations of BUCK1 are not available for BUCK2 and BUCK3, vice versa. As
 * a result, if an incorrect validParam for a buck is set, an error will occur
 * when calling the set/get configuration APIs.
 *
 * @param validParams Each bit in this variable represents whether a struct
 * member is valid. For valid values, refer to @ref Pmic_PwrBuckCfgValidParams.
 *
 * @param resource To be specified by the user to indicate which buck power
 * resource the configuration struct pertains to. For valid values, refer to
 * @ref Pmic_PowerResources.
 *
 * @param enable BUCK1, BUCK2, or BUCK3 regulator enable. When set to true,
 * the regulator is activated. Otherwise the regulator is deactivated.
 *
 * @param pldnEn Activation of the output discharge resistor when BUCK1, BUCK2,
 * or BUCK3 is deactivated. When set to true, resistive discharge is activated.
 * Otherwise, resistive discharge is deactivated.
 *
 * @param fpwmEn BUCK1, BUCK2, or BUCK3 forced PWM configuration. When set to
 * true, the regulator is forced to PWM operation. Else, automatic transitions
 * between PFM and PWM modes will occur (AUTO mode).
 *
 * @param uvThr BUCK1, BUCK2, or BUCK3 powergood low threshold level. For valid
 * values, refer to @ref Pmic_buckUvThrValues.
 *
 * @param ovThr BUCK1, BUCK2, or BUCK3 powergood high threshold level. For valid
 * values, refer to @ref Pmic_buckOvThrValues.
 *
 * @param ilimSel BUCK1, BUCK2, or BUCK3 current limit selection. For BUCK1 valid
 * values, refer to @ref Pmic_buck1IlimSelValues. For BUCK2 and BUCK3 valid values,
 * refer to @ref Pmic_buck2_3IlimSelValues.
 *
 * @param ovpSel BUCK1, BUCK2, or BUCK3 reaction to OVP fault. For valid values,
 * refer to @ref Pmic_regulatorFaultResponses.
 *
 * @param ovSel BUCK1, BUCK2, or BUCK3 reaction to OV fault. For valid values,
 * refer to @ref Pmic_regulatorFaultResponses.
 *
 * @param uvSel BUCK1, BUCK2, or BUCK3 reaction to UV fault. For valid values,
 * refer to @ref Pmic_regulatorFaultResponses.
 *
 * @param scSel BUCK1, BUCK2, or BUCK3 reaction to SC fault. For valid values,
 * refer to @ref Pmic_regulatorFaultResponses.
 *
 * @param rvConf BUCK1, BUCK2, BUCK3 residual voltage configuration. For valid
 * values, refer to @ref Pmic_rvConfValues.
 *
 * @param slewRate BUCK1, BUCK2, or BUCK3 output voltage slew rate. For valid
 * values, refer to @ref Pmic_BuckSlewRateValues.
 *
 * @param deglitchSel BUCK1, BUCK2, or BUCK3 deglitch selection. For valid
 * values, refer to @ref Pmic_buckDeglitchSelValues.
 *
 * @param dischargeSel BUCK1, BUCK2, or BUCK3 discharge selection. For valid
 * values, refer to @ref Pmic_BuckDischargeSelValues.
 *
 * @param ssEn Device internal spread spectrum for buck regulators. When set to
 * true, spread spectrum is enabled. Otherwise, spread spectrum is disabled.
 *
 * @param ssmSel Spread spectrum modulation selection. For valid values, refer
 * to @ref Pmic_ssmSelValues.
 *
 * @param vset BUCK1 voltage selection. For the maximum BUCK1 VSET value,
 * refer to @ref Pmic_regulatorVSetRanges.
 *
 * @param uvloRising BUCK1 rising under-voltage lockout. For the maximum
 * BUCK1 rising UVLO, refer to @ref Pmic_buck1UvloMaxValues.
 *
 * @param uvloFalling BUCK1 falling under-voltage lockout. For the maximum
 * BUCK1 falling UVLO, refer to @ref Pmic_buck1UvloMaxValues.
 *
 * @param highSideSlewRate BUCK1 high side slew rate control. For valid values,
 * refer to @ref Pmic_Buck1HighSideSlewRateCtrlValues.
 *
 * @param vsetActive BUCK2 or BUCK3 active voltage selection. For the maximum
 * BUCK2 or BUCK3 active VSET value, refer to @ref Pmic_regulatorVSetRanges.
 *
 * @param vsetLPwr BUCK2 or BUCK3 low power voltage selection. For the maximum
 * BUCK2 or BUCK3 low power VSET value, refer to @ref Pmic_regulatorVSetRanges.
 *
 * @param vmonOnly BUCK2 or BUCK3 VMON only configuration. When set to true,
 * only the VMON of the regulator is used. When set to false, both regulator
 * and VMON are used.
 */
typedef struct Pmic_PwrBuckCfg_s
{
    uint32_t validParams;
    uint8_t resource;

    /* All bucks */
    bool enable;
    bool pldnEn;
    bool fpwmEn;
    uint8_t uvThr;
    uint8_t ovThr;
    uint8_t ilimSel;
    uint8_t ovpSel;
    uint8_t ovSel;
    uint8_t uvSel;
    uint8_t scSel;
    uint8_t rvConf;
    uint8_t slewRate;
    uint8_t deglitchSel;
    uint8_t dischargeSel;
    bool ssEn;
    uint8_t ssmSel;

    /* BUCK1 only */
    uint8_t vset;
    uint8_t uvloRising;
    uint8_t uvloFalling;
    uint8_t highSideSlewRate;

    /* BUCK2 and BUCK3 only  */
    uint8_t vsetActive;
    uint8_t vsetLPwr;
    bool vmonOnly;
} Pmic_PwrBuckCfg_t;

/**
 * @anchor Pmic_PwrLdoCfg
 * @name PMIC LDO Configuration Struct
 *
 * @brief Struct used to set and get PMIC LDO regulator configurations.
 *
 * @param validParams Each bit in this variable represents whether a struct
 * member is valid. For valid values, refer to @ref Pmic_PwrLdoCfgValidParams.
 *
 * @param enable LDO regulator enable. When set to true, the regulator is
 * activated. Otherwise the regulator is deactivated.
 *
 * @param mode LDO mode configuration. For valid values, refer to
 * @ref Pmic_ldoBypConfigValues.
 *
 * @param vset LDO voltage configuration. For the maximum LDO VSET value,
 * refer to @ref Pmic_regulatorVSetRanges
 *
 * @param vmonOnly LDO VMON only configuration. When set to true, only the
 * VMON of the regulator is used. When set to false, both the regulator and
 * VMON are used.
 *
 * @param dischargeEn LDO discharge enable configuration. When set to true,
 * resistive discharge is activated. When set to false, resistive discharge
 * is deactivated.
 *
 * @param dischargeSel LDO discharge selection. For valid values, refer to
 * @ref Pmic_ldoDischargeSelValues
 *
 * @param deglitchSel LDO deglitch selection. For valid values, refer to
 * @ref Pmic_ldoDeglitchSelValues
 *
 * @param uvThr LDO powergood low threshold level. For valid values, refer to
 * @ref Pmic_ldoUvThrValues
 *
 * @param ovThr LDO powergood high threshold level. For valid values, refer to
 * @ref Pmic_ldoOvThrValues
 *
 * @param ilimSel LDO current limit selection. For valid values, refer to
 * @ref Pmic_ldoIlimSelValues
 *
 * @param ovpSel LDO reaction to OVP fault. For valid values, refer to
 * @ref Pmic_regulatorFaultResponses
 *
 * @param ovSel LDO reaction to OV fault. For valid values, refer to
 * @ref Pmic_regulatorFaultResponses
 *
 * @param uvSel LDO reaction to UV fault. For valid values, refer to
 * @ref Pmic_regulatorFaultResponses
 *
 * @param scSel LDO reaction to SC fault. For valid values, refer to
 * @ref Pmic_regulatorFaultResponses
 *
 * @param rvConf LDO residual voltage configuration. For valid values, refer to
 * @ref Pmic_ldoRvConfValues
 */
typedef struct Pmic_PwrLdoCfg_s
{
    uint32_t validParams;

    bool enable;
    uint8_t mode;
    uint8_t vset;
    bool vmonOnly;
    bool dischargeEn;
    uint8_t dischargeSel;
    uint8_t deglitchSel;
    uint8_t uvThr;
    uint8_t ovThr;
    uint8_t ilimSel;
    uint8_t ovpSel;
    uint8_t ovSel;
    uint8_t uvSel;
    uint8_t scSel;
    uint8_t rvConf;
} Pmic_PwrLdoCfg_t;

/**
 * @anchor Pmic_PwrRsrcStat
 * @name PMIC Power Resource Status Struct
 *
 * @brief Struct used to get power resource (buck/LDO) statuses.
 *
 * @param resource To be specified by the user to indicate which power resource
 * the configuration struct pertains to. For valid values, refer to
 * @ref Pmic_PowerResources.
 *
 * @param active Status indicating whether resource is on or off.
 *
 * @param ov Status indicating whether resource output voltage is above
 * overvoltage threshold.
 *
 * @param uv Status indicating whether resource output voltage is below
 * undervoltage threshold.
 *
 * @param ovp Status indicating whether resource output voltage is above
 * overvoltage protection threshold.
 */
typedef struct Pmic_PwrRsrcStat_s
{
    uint8_t resource;

    bool active;
    bool ov;
    bool uv;
    bool ovp;
} Pmic_PwrRsrcStat_t;

/**
 * @anchor Pmic_PwrTsdCfg
 * @name PMIC Thermal Shutdown Configuration Struct
 *
 * @brief Struct used to set and get PMIC TSD configuration.
 *
 * @param twarnStayInSafeState PMIC action in response to TWARN flag. When set
 * to true, PMIC will stay in SAFE state as along as TWARN flag is active.
 * Otherwise, PMIC will be able to leave SAFE state regardless of whether
 * TWARN flag is active.
 *
 * @param tsdImmLevel Immediate thermal shutdown level/threshold. For valid
 * values, refer to @ref Pmic_tsdImmLevelValues.
 *
 * @param twarnLevel Temperature warning level/threshold. For valid values,
 * refer to @ref Pmic_twarnLevelValues.
 */
typedef struct Pmic_PwrTsdCfg_s
{
    uint32_t validParams;

    bool twarnStayInSafeState;
    uint8_t tsdImmLevel;
    uint8_t twarnLevel;
} Pmic_PwrTsdCfg_t;

/**
 * @anchor Pmic_PwrBuckLdoSeqTrig
 * @name PMIC Buck and LDO Sequence Trigger Struct
 *
 * @brief Struct used to set and get PMIC Buck and LDO sequence triggers.
 *
 * @param trigger PMIC buck/LDO sequence trigger type. For valid values, refer
 * to @ref Pmic_PwrBuckLdoSequenceTriggers.
 *
 * @param exclude When set to true, the sequence trigger is excluded from the
 * power ON/OFF sequence logic. Otherwise, the sequence trigger is included
 * as part of the power ON/OFF sequence logic.
 */
typedef struct Pmic_PwrBuckLdoSeqTrig_s
{
  uint16_t trigger;
  bool exclude;
} Pmic_PwrBuckLdoSeqTrig_t;

/**
 * @anchor Pmic_PwrBuckLdoSeqDly
 * @name PMIC Buck and LDO Sequence Delay Struct
 *
 * @brief Struct used to set and get PMIC Buck and LDO sequence delays.
 *
 * @param validParams Each bit in this variable represents whether a struct
 * member is valid. For valid values, refer to @ref Pmic_PwrBuckLdoSeqDlyValidParams.
 *
 * @param resource To be specified by the user to indicate which power
 * resource the configuration struct pertains to. For valid values, refer to
 * @ref Pmic_PowerResources.
 *
 * @param seqDlyOn Power resource sequence delay from the ON trigger point.
 * For valid values, refer to @ref Pmic_PwrSeqDlyValues.
 *
 * @param seqDlyOff Power resource sequence delay from the OFF trigger point.
 * For valid values, refer to @ref Pmic_PwrSeqDlyValues.
 */
typedef struct Pmic_PwrBuckLdoSeqDly_s
{
    uint32_t validParams;
    uint8_t resource;

    uint8_t seqDlyOn;
    uint8_t seqDlyOff;
} Pmic_PwrBuckLdoSeqDly_t;

/* ========================================================================== */
/*                             Function Declarations                          */
/* ========================================================================== */

/**
 * @brief Set PMIC buck configuration.
 *
 * @note Before setting a buck's configuration, it is recommended to ensure that
 * the resource is disabled first. This API configures the regulator enable last
 * so that the end-user could set configurations and enable the regulator in one
 * API call.
 *
 * @details The following options are configurable via this API
 * 1. Enable (validParam: PMIC_BUCK_ENABLE_VALID)
 * 2. Pulldown enable (validParam: PMIC_BUCK_PLDN_EN_VALID)
 * 3. Forced PWM option (validParam: PMIC_BUCK_FPWM_EN_VALID)
 * 4. Undervoltage threshold (validParam: PMIC_BUCK_UV_THR_VALID)
 * 5. Overvoltage threshold (validParam: PMIC_BUCK_OV_THR_VALID)
 * 6. Current limit selection (validParam: PMIC_BUCK_ILIM_SEL_VALID)
 * 7. Overvoltage protection fault response (validParam: PMIC_BUCK_OVP_SEL_VALID)
 * 8. Overvoltage fault response (validParam: PMIC_BUCK_OV_SEL_VALID)
 * 9. Undervoltage fault response (validParam: PMIC_BUCK_UV_SEL_VALID)
 * 10. Short circuit fault response (validParam: PMIC_BUCK_SC_SEL_VALID)
 * 11. Residual voltage configuration (validParam: PMIC_BUCK_RV_CONF_VALID)
 * 12. Slew rate (validParam: PMIC_BUCK_SLEW_RATE_VALID)
 * 13. Deglitch selection (validParam: PMIC_BUCK_DEGLITCH_SEL_VALID)
 * 14. Discharge selection (validParam: PMIC_BUCK_DISCHARGE_SEL_VALID)
 * 15. Spread spectrum enable (validParam: PMIC_BUCK_SS_EN_VALID)
 * 16. Spread spectrum modulation selection (validParam: PMIC_BUCK_SSM_SEL_VALID)
 * 17. VSET (BUCK1 only; validParam: PMIC_BUCK_VSET_VALID)
 * 18. Undervoltage lockout rising (BUCK1 only; validParam: PMIC_BUCK_UVLO_RISING_VALID)
 * 19. Undervoltage lockout falling (BUCK1 only; validParam: PMIC_BUCK_UVLO_FALLING_VALID)
 * 20. High side slew rate (BUCK1 only; validParam: PMIC_BUCK_HIGH_SIDE_SLEW_RATE_VALID)
 * 21. VSET_ACTIVE (BUCK2 and BUCK3 only; validParam: PMIC_BUCK_VSET_ACTIVE_VALID)
 * 22. VSET_LPWR (BUCK2 and BUCK3 only; validParam: PMIC_BUCK_VSET_LPWR_VALID)
 * 23. VMON only option (BUCK2 and BUCK3 only; validParam: PMIC_BUCK_VMON_ONLY_VALID)
 * For more information on the buck configurations, refer to @ref Pmic_PwrBuckCfg.
 *
 * @param pmicHandle [IN]  PMIC interface handle.
 * @param buckCfg    [OUT] Buck configurations to write to PMIC.
 *
 * @return Success code if PMIC buck configurations have been set, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_pwrSetBuckCfg(const Pmic_CoreHandle_t *pmicHandle, const Pmic_PwrBuckCfg_t *buckCfg);

/**
 * @brief Get PMIC buck configuration. This "get" API supports obtaining the
 * same parameters that are settable through the "Set" API
 * (`Pmic_pwrSetBuckCfg`).
 *
 * @param pmicHandle [IN]  PMIC interface handle.
 * @param buckCfg    [OUT] Buck configurations obtained from the PMIC.
 *
 * @return Success code if PMIC buck configurations have been obtained, error
 * code otherwise. For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_pwrGetBuckCfg(const Pmic_CoreHandle_t *pmicHandle, Pmic_PwrBuckCfg_t *buckCfg);

/**
 * @brief Set PMIC LDO configurations.
 *
 * @note Before setting the LDO configuration, it is recommended to ensure that
 * the resource is disabled first. This API configures the regulator enable last
 * so that the end-user could set configurations and enable the regulator in one
 * API call.
 *
 * @details The following options are configurable via this API
 * 1. Enable (validParam: PMIC_LDO_ENABLE_VALID)
 * 2. Mode of operation (validParam: PMIC_LDO_MODE_VALID)
 * 3. VSET (validParam: PMIC_LDO_VSET_VALID)
 * 4. VMON only option (validParam: PMIC_LDO_VMON_ONLY_VALID)
 * 5. Discharge enable (validParam: PMIC_LDO_DISCHARGE_EN_VALID)
 * 6. discharge selection (validParam: PMIC_LDO_DISCHARGE_SEL_VALID)
 * 7. Deglitch selection (validParam: PMIC_LDO_DEGLITCH_SEL_VALID)
 * 8. Undervoltage threshold (validParam: PMIC_LDO_UV_THR_VALID)
 * 9. Overvoltage threshold (validParam: PMIC_LDO_OV_THR_VALID)
 * 10. Current limit selection (validParam: PMIC_LDO_ILIM_SEL_VALID)
 * 11. Overvoltage protection fault response (validParam: PMIC_LDO_OVP_SEL_VALID)
 * 12. Overvoltage fault response (validParam: PMIC_LDO_OV_SEL_VALID)
 * 13. Undervoltage fault response (validParam: PMIC_LDO_UV_SEL_VALID)
 * 14. Short circuit fault response (validParam: PMIC_LDO_SC_SEL_VALID)
 * 15. Residual voltage configuration (validParam: PMIC_LDO_RV_CONF_VALID)
 * For more information on the LDO configurations, refer to @ref Pmic_PwrLdoCfg.
 *
 * @param pmicHandle [IN] PMIC interface handle.
 * @param ldoCfg     [IN] LDO configurations to write to PMIC.
 *
 * @return Success code if PMIC LDO configurations have been set, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_pwrSetLdoCfg(const Pmic_CoreHandle_t *pmicHandle, const Pmic_PwrLdoCfg_t *ldoCfg);

/**
 * @brief Get PMIC LDO configurations. This "get" API supports obtaining the
 * same parameters that are settable through the "Set" API
 * (`Pmic_pwrSetLdoCfg`).

 *
 * @param pmicHandle [IN]  PMIC interface handle.
 * @param ldoCfg     [OUT] LDO configurations obtained from the PMIC.
 *
 * @return Success code if PMIC LDO configurations have been obtained, error
 * code otherwise. For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_pwrGetLdoCfg(const Pmic_CoreHandle_t *pmicHandle, Pmic_PwrLdoCfg_t *ldoCfg);

/**
 * @brief Get the statuses of a power resource (buck/LDO).
 *
 * @details The following power resource statuses are obtainable from this API
 * 1. active
 * 2. overvoltage
 * 3. undervoltage
 * 4. overvoltage protection
 * For more information on the power resource statuses, refer to
 * @ref Pmic_PwrRsrcStat.
 *
 * @param pmicHandle  [IN]  PMIC interface handle.
 * @param pwrRsrcStat [OUT] Power resource statuses obtained from the PMIC.
 *
 * @return Success code if PMIC power resource statuses have been obtained, error
 * code otherwise. For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_pwrGetRsrcStat(const Pmic_CoreHandle_t *pmicHandle, Pmic_PwrRsrcStat_t *pwrRsrcStat);

/**
 * @brief Set PMIC thermal shutdown configurations.
 *
 * @details The following options are configurable via this API
 * 1. Option to stay in safe state as long as TWARN flag is active
 * 2. Immediate thermal shutdown level
 * 3. TWARN level
 * For more information on the TSD configurations, refer to @ref Pmic_PwrTsdCfg.
 *
 * @param pmicHandle [IN] PMIC interface handle.
 * @param tsdCfg     [IN] TSD configurations to write to PMIC.
 *
 * @return Success code if PMIC TSD configurations have been set, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_pwrSetTsdCfg(const Pmic_CoreHandle_t *pmicHandle, const Pmic_PwrTsdCfg_t *tsdCfg);

/**
 * @brief Get PMIC thermal shutdown configurations. This "get" API supports
 * obtaining the same parameters that are settable through the "Set" API
 * (`Pmic_pwrSetTsdCfg`).
 *
 * @param pmicHandle [IN]  PMIC interface handle.
 * @param tsdCfg     [OUT] TSD configurations obtained from the PMIC.
 *
 * @return Success code if PMIC TSD configurations have been obtained, error
 * code otherwise. For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_pwrGetTsdCfg(const Pmic_CoreHandle_t *pmicHandle, Pmic_PwrTsdCfg_t *tsdCfg);

/**
 * @brief Get the PMIC immediate thermal shutdown status.
 *
 * @param pmicHandle [IN] PMIC interface handle.
 * @param tsdImmStat [OUT] When set to true, the die junction temperature is
 * above the thermal level causing an immediate shutdown. Otherwise, the die
 * junction temperature is below the thermal level.
 *
 * @return Success code if the PMIC immediate TSD status has been obtained, error
 * code otherwise. For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_pwrGetTsdImmStat(const Pmic_CoreHandle_t *pmicHandle, bool *tsdImmStat);

/**
 * @brief Set PMIC buck and LDO sequence triggers.
 *
 * @details Given an array of type `Pmic_PwrBuckLdoSeqTrig_t` and length of array,
 * the API sets sequence trigger configurations of the regulators (whether or not
 * the sequence trigger is excluded/included in the power ON/OFF sequence logic).
 *
 * @param pmicHandle [IN] PMIC interface handle.
 * @param seqTrigCfg [IN] Array of regulator sequence trigger configurations.
 * @param len        [IN] Length of `seqTrigCfg` array.
 *
 * @return Success code if sequence trigger configurations have been set, error
 * code otherwise. For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_pwrSetBuckLdoSeqTrig(const Pmic_CoreHandle_t *pmicHandle, const Pmic_PwrBuckLdoSeqTrig_t seqTrigCfg[], uint8_t len);

/**
 * @brief Get PMIC buck and LDO sequence triggers.
 *
 * @details Given an array of type `Pmic_PwrBuckLdoSeqTrig_t` and length of array,
 * the API gets sequence trigger configurations of the regulators (whether or not
 * the sequence trigger is excluded/included in the power ON/OFF sequence logic).
 *
 * @param pmicHandle [IN]  PMIC interface handle.
 * @param seqTrigCfg [OUT] Array of regulator sequence trigger configurations.
 * @param len        [IN]  Length of `seqTrigCfg` array.
 *
 * @return Success code if sequence trigger configurations have been obtained, error
 * code otherwise. For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_pwrGetBuckLdoSeqTrig(const Pmic_CoreHandle_t *pmicHandle, Pmic_PwrBuckLdoSeqTrig_t seqTrigCfg[], uint8_t len);

/**
 * @brief Set buck and LDO sequence delays.
 *
 * @details Given an array of type `Pmic_PwrBuckLdoSeqDly_t` and length of array,
 * the API sets sequence delay configurations of the regulators.
 *
 * @param pmicHandle [IN] PMIC interface handle.
 * @param seqDlyCfg  [IN] Array of regulator sequence delay configurations.
 * @param len        [IN] Length of `seqDlyCfg` array.
 *
 * @return Success code if sequence delay configurations have been set, error
 * code otherwise. For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_pwrSetBuckLdoSeqDly(const Pmic_CoreHandle_t *pmicHandle, const Pmic_PwrBuckLdoSeqDly_t seqDlyCfg[], uint8_t len);

/**
 * @brief Get buck and LDO sequence delays.
 *
 * @details Given an array of type `Pmic_PwrBuckLdoSeqDly_t` and length of array,
 * the API gets sequence delay configurations of the regulators.
 *
 * @param pmicHandle [IN]  PMIC interface handle.
 * @param seqDlyCfg  [OUT] Array of regulator sequence delay configurations.
 * @param len        [IN]  Length of `seqDlyCfg` array.
 *
 * @return Success code if sequence delay configurations have been obtained, error
 * code otherwise. For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_pwrGetBuckLdoSeqDly(const Pmic_CoreHandle_t *pmicHandle, Pmic_PwrBuckLdoSeqDly_t seqDlyCfg[], uint8_t len);

#ifdef __cplusplus
}

#endif /* __cplusplus */
#endif /* __PMIC_POWER_H__ */
