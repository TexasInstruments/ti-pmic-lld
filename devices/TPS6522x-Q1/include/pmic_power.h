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
#ifndef PMIC_POWER_H
#define PMIC_POWER_H

/**
 * @file pmic_power.h
 *
 * @brief PMIC power interface. Contains APIs, macros/defines, and data structures
 * used to configure, control, and interact with PMIC power-related features, such
 * as buck/LDO regulators and voltage monitors.
 */

/* ========================================================================= */
/*                              Include Files                                */
/* ========================================================================= */

#include "pmic_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                            Macros & Typedefs                               */
/* ========================================================================== */

/**
 * @anchor Pmic_PwrResourceType
 * @name PMIC Power Resource Type
 *
 * @brief Enumeration of PMIC power resource types.
 *
 * @{
 */
#define PMIC_POWER_RESOURCE_TYPE_BUCK ((uint16_t)1U)
#define PMIC_POWER_RESOURCE_TYPE_LDO  ((uint16_t)2U)
#define PMIC_POWER_RESOURCE_TYPE_VMON ((uint16_t)3U)
/** @} */

/**
 * @anchor Pmic_PwrResourceId
 * @name PMIC Power Resource ID
 *
 * @brief Enumeration of PMIC power resource IDs.
 *
 * @{
 */
#define PMIC_POWER_RESOURCE_ID_BUCK1     ((uint8_t)1U)
#define PMIC_POWER_RESOURCE_ID_BUCK2     ((uint8_t)2U)
#define PMIC_POWER_RESOURCE_ID_BUCK3     ((uint8_t)3U)
#define PMIC_POWER_RESOURCE_ID_BUCK4     ((uint8_t)4U)
#define PMIC_POWER_RESOURCE_ID_LDO1      ((uint8_t)1U)
#define PMIC_POWER_RESOURCE_ID_LDO2      ((uint8_t)2U)
#define PMIC_POWER_RESOURCE_ID_LDO3      ((uint8_t)3U)
#define PMIC_POWER_RESOURCE_ID_VMON1     ((uint8_t)1U)
#define PMIC_POWER_RESOURCE_ID_VMON2     ((uint8_t)2U)
#define PMIC_POWER_RESOURCE_ID_VCCA_VMON ((uint8_t)3U)
/** @} */

/**
 * @anchor Pmic_PwrResource
 * @name PMIC Power Resource
 *
 * @brief Enumeration of PMIC power resources.
 *
 * @{
 */
#define PMIC_POWER_RESOURCE_BUCK1     ((uint16_t)((PMIC_POWER_RESOURCE_TYPE_BUCK << 8U) | (PMIC_POWER_RESOURCE_ID_BUCK1)))
#define PMIC_POWER_RESOURCE_BUCK2     ((uint16_t)((PMIC_POWER_RESOURCE_TYPE_BUCK << 8U) | (PMIC_POWER_RESOURCE_ID_BUCK2)))
#define PMIC_POWER_RESOURCE_BUCK3     ((uint16_t)((PMIC_POWER_RESOURCE_TYPE_BUCK << 8U) | (PMIC_POWER_RESOURCE_ID_BUCK3)))
#define PMIC_POWER_RESOURCE_BUCK4     ((uint16_t)((PMIC_POWER_RESOURCE_TYPE_BUCK << 8U) | (PMIC_POWER_RESOURCE_ID_BUCK4)))
#define PMIC_POWER_RESOURCE_LDO1      ((uint16_t)((PMIC_POWER_RESOURCE_TYPE_LDO << 8U) | (PMIC_POWER_RESOURCE_ID_LDO1)))
#define PMIC_POWER_RESOURCE_LDO2      ((uint16_t)((PMIC_POWER_RESOURCE_TYPE_LDO << 8U) | (PMIC_POWER_RESOURCE_ID_LDO2)))
#define PMIC_POWER_RESOURCE_LDO3      ((uint16_t)((PMIC_POWER_RESOURCE_TYPE_LDO << 8U) | (PMIC_POWER_RESOURCE_ID_LDO3)))
#define PMIC_POWER_RESOURCE_VMON1     ((uint16_t)((PMIC_POWER_RESOURCE_TYPE_VMON << 8U) | (PMIC_POWER_RESOURCE_ID_VMON1)))
#define PMIC_POWER_RESOURCE_VMON2     ((uint16_t)((PMIC_POWER_RESOURCE_TYPE_VMON << 8U) | (PMIC_POWER_RESOURCE_ID_VMON2)))
#define PMIC_POWER_RESOURCE_VCCA_VMON ((uint16_t)((PMIC_POWER_RESOURCE_TYPE_VMON << 8U) | (PMIC_POWER_RESOURCE_ID_VCCA_VMON)))
#define PMIC_POWER_RESOURCE_BUCK_MIN  ((uint16_t)PMIC_POWER_RESOURCE_BUCK1)
#define PMIC_POWER_RESOURCE_BUCK_MAX  ((uint16_t)PMIC_POWER_RESOURCE_BUCK4)
#define PMIC_POWER_RESOURCE_LDO_MIN   ((uint16_t)PMIC_POWER_RESOURCE_LDO1)
#define PMIC_POWER_RESOURCE_LDO_MAX   ((uint16_t)PMIC_POWER_RESOURCE_LDO3)
#define PMIC_POWER_RESOURCE_VMON_MIN  ((uint16_t)PMIC_POWER_RESOURCE_VMON1)
#define PMIC_POWER_RESOURCE_VMON_MAX  ((uint16_t)PMIC_POWER_RESOURCE_VCCA_VMON)
/** @} */

/**
 * @anchor Pmic_PwrBuck1Vset
 * @name PMIC Power Buck 1 VSET
 *
 * @brief Range of values for PMIC Buck 1 VSET.
 *
 * @{
 */
#define PMIC_POWER_BUCK1_VSET_MIN ((uint8_t)0x0AU)
#define PMIC_POWER_BUCK1_VSET_MAX ((uint8_t)0xFDU)
/** @} */

/**
 * @anchor Pmic_PwrBuck2_3_4Vset
 * @name PMIC Power Buck 2/3/4 VSET
 *
 * @brief Range of values for PMIC Buck 2/3/4 VSET.
 *
 * @{
 */
#define PMIC_POWER_BUCK2_3_4_VSET_MIN ((uint8_t)0x00U)
#define PMIC_POWER_BUCK2_3_4_VSET_MAX ((uint8_t)0x45U)
/** @} */

/**
 * @anchor Pmic_PwrBuckSlewRate
 * @name PMIC Power Buck Slew Rate
 *
 * @brief Enumeration of PMIC buck slew rates.
 *
 * @{
 */
#define PMIC_POWER_BUCK_SLEW_RATE_10_MV_PER_US   (0U)
#define PMIC_POWER_BUCK_SLEW_RATE_5_MV_PER_US    (1U)
#define PMIC_POWER_BUCK_SLEW_RATE_2P5_MV_PER_US  (2U)
#define PMIC_POWER_BUCK_SLEW_RATE_1P25_MV_PER_US (3U)
#define PMIC_POWER_BUCK_SLEW_RATE_MIN            ((uint8_t)PMIC_POWER_BUCK_SLEW_RATE_10_MV_PER_US)
#define PMIC_POWER_BUCK_SLEW_RATE_MAX            ((uint8_t)PMIC_POWER_BUCK_SLEW_RATE_1P25_MV_PER_US)
/** @} */

/**
 * @anchor Pmic_PwrLdo1Vset
 * @name PMIC Power LDO1 VSET
 *
 * @brief Range of values for PMIC LDO1 VSET.
 *
 * @{
 */
#define PMIC_POWER_LDO1_VSET_MIN ((uint8_t)0x00U)
#define PMIC_POWER_LDO1_VSET_MAX ((uint8_t)0x3FU)
/** @} */

/**
 * @anchor Pmic_PwrLdo2_3Vset
 * @name PMIC Power LDO2/3 VSET
 *
 * @brief Range of values for PMIC LDO2/3 VSET.
 *
 * @{
 */
#define PMIC_POWER_LDO2_3_VSET_MIN ((uint8_t)0x00U)
#define PMIC_POWER_LDO2_3_VSET_MAX ((uint8_t)0x36U)
/** @} */

/**
 * @anchor Pmic_PwrVccaVmonPgSet
 * @name PMIC Power VCCA_VMON PG Set
 *
 * @brief Enumeration of PMIC VCCA_VMON PG Set values.
 *
 * @{
 */
#define PMIC_POWER_VCCA_VMON_PG_SET_3P3_V (0U)
#define PMIC_POWER_VCCA_VMON_PG_SET_5_V   (1U)
#define PMIC_POWER_VCCA_VMON_PG_SET_MIN   ((uint8_t)PMIC_POWER_VCCA_VMON_PG_SET_3P3_V)
#define PMIC_POWER_VCCA_VMON_PG_SET_MAX   ((uint8_t)PMIC_POWER_VCCA_VMON_PG_SET_5_V)
/** @} */

/**
 * @anchor Pmic_PwrVccaVmonThr
 * @name PMIC Power VCCA_VMON Threshold
 *
 * @brief Enumeration of PMIC VCCA_VMON threshold values.
 *
 * @{
 */
#define PMIC_POWER_VCCA_VMON_THR_3_PCT  (0U)
#define PMIC_POWER_VCCA_VMON_THR_4_PCT  (1U)
#define PMIC_POWER_VCCA_VMON_THR_6_PCT  (2U)
#define PMIC_POWER_VCCA_VMON_THR_10_PCT (3U)
#define PMIC_POWER_VCCA_VMON_THR_MIN    ((uint8_t)PMIC_POWER_VCCA_VMON_THR_3_PCT)
#define PMIC_POWER_VCCA_VMON_THR_MAX    ((uint8_t)PMIC_POWER_VCCA_VMON_THR_10_PCT)
/** @} */

/**
 * @anchor Pmic_PwrVmon1PgSet
 * @name PMIC Power VMON1 PG Set
 *
 * @brief Range of values for PMIC VMON1 PG Set values.
 *
 * @{
 */
#define PMIC_POWER_VMON1_PG_SET_MIN ((uint8_t)0x0AU)
#define PMIC_POWER_VMON1_PG_SET_MAX ((uint8_t)0xFFU)
/** @} */

/**
 * @anchor Pmic_PwrVmon2PgSet
 * @name PMIC Power VMON2 PG Set
 *
 * @brief Range of values for PMIC VMON2 PG Set values.
 *
 * @{
 */
#define PMIC_POWER_VMON2_PG_SET_MIN ((uint8_t)0x00U)
#define PMIC_POWER_VMON2_PG_SET_MAX ((uint8_t)0x45U)
/** @} */

/**
 * @anchor Pmic_PwrVmonThr
 * @name PMIC Power VMON Threshold
 *
 * @brief Enumeration of PMIC VMON threshold values.
 *
 * @note Used only for buck, LDO, and VMONs; not for VCCA_VMON.
 *
 * @{
 */
#define PMIC_POWER_VMON_THR_3_PCT_30_MV (0U)
#define PMIC_POWER_VMON_THR_4_PCT_40_MV (1U)
#define PMIC_POWER_VMON_THR_6_PCT_60_MV (2U)
#define PMIC_POWER_VMON_THR_8_PCT_80_MV (3U)
#define PMIC_POWER_VMON_THR_MIN         ((uint8_t)PMIC_POWER_VMON_THR_3_PCT_30_MV)
#define PMIC_POWER_VMON_THR_MAX         ((uint8_t)PMIC_POWER_VMON_THR_8_PCT_80_MV)
/** @} */

/**
 * @anchor Pmic_PwrVmonDeglitch
 * @name PMIC Power VMON Deglitch
 *
 * @brief Enumerations for PMIC Power VMON Deglitch settings.
 *
 * Deglitch time select for BUCKx_VMON, LDOx_VMON, VMONx and VCCA_VMON.
 * Format: PMIC_PWR_VMON_DEGL_<BUCK/LDO/VMON time>_<VCCA time>
 *
 * @{
 */
#define PMIC_PWR_VMON_DEGL_4US_4US     (0U)
#define PMIC_PWR_VMON_DEGL_20US_20US   (1U)
#define PMIC_PWR_VMON_DEGL_0P5US_0P5US (2U)
#define PMIC_PWR_VMON_DEGL_0P5US_4US   (3U)
#define PMIC_PWR_VMON_DEGL_0P5US_20US  (4U)
#define PMIC_PWR_VMON_DEGL_4US_0P5US   (5U)
#define PMIC_PWR_VMON_DEGL_4US_4US_ALT (6U)  /* Functionally identical to value 0 */
#define PMIC_PWR_VMON_DEGL_4US_20US    (7U)
#define PMIC_POWER_VMON_DEGL_SEL_MIN   ((uint8_t)PMIC_PWR_VMON_DEGL_4US_4US)
#define PMIC_POWER_VMON_DEGL_SEL_MAX   ((uint8_t)PMIC_PWR_VMON_DEGL_4US_20US)
/** @} */

/**
 * @anchor Pmic_PwrGrpSel
 * @name PMIC Power Group Select
 *
 * @brief Enumeration of PMIC power group select values.
 *
 * @{
 */
#define PMIC_POWER_GRP_SEL_NONE  (0U)
#define PMIC_POWER_GRP_SEL_MCU   (1U)
#define PMIC_POWER_GRP_SEL_SOC   (2U)
#define PMIC_POWER_GRP_SEL_OTHER (3U)
#define PMIC_POWER_GRP_SEL_MIN   ((uint8_t)PMIC_POWER_GRP_SEL_NONE)
#define PMIC_POWER_GRP_SEL_MAX   ((uint8_t)PMIC_POWER_GRP_SEL_OTHER)
/** @} */

/**
 * @anchor Pmic_pwrTwarnLvl
 * @name PMIC Power Temperature Warning Level
 *
 * @brief Enumeration of PMIC temperature warning level values.
 *
 * @{
 */
#define PMIC_POWER_TWARN_LEVEL_130C (0U)
#define PMIC_POWER_TWARN_LEVEL_140C (1U)
#define PMIC_POWER_TWARN_LEVEL_MIN  ((uint8_t)PMIC_POWER_TWARN_LEVEL_130C)
#define PMIC_POWER_TWARN_LEVEL_MAX  ((uint8_t)PMIC_POWER_TWARN_LEVEL_140C)
/** @} */

/**
 * @anchor Pmic_pwrTsdOrdLvl
 * @name PMIC Power Thermal Orderly Shutdown Level
 *
 * @brief Enumeration of PMIC thermal orderly shutdown level values.
 *
 * @{
 */
#define PMIC_POWER_TSD_ORD_LEVEL_140C (0U)
#define PMIC_POWER_TSD_ORD_LEVEL_145C (1U)
#define PMIC_POWER_TSD_ORD_LEVEL_MIN  ((uint8_t)PMIC_POWER_TSD_ORD_LEVEL_140C)
#define PMIC_POWER_TSD_ORD_LEVEL_MAX  ((uint8_t)PMIC_POWER_TSD_ORD_LEVEL_145C)
/** @} */

/**
 * @anchor Pmic_PwrSsDepth
 * @name PMIC Power Spread Spectrum Depth
 *
 * @brief Enumeration of PMIC spread spectrum depth values.
 *
 * @{
 */
#define PMIC_POWER_SS_DEPTH_4_PCT (0U)
#define PMIC_POWER_SS_DEPTH_7_PCT (1U)
/** @} */

/**
 * @anchor Pmic_PwrBuckCfgValidParams
 * @name PMIC Power Buck Configuration Structure Valid Parameters
 *
 * @brief Definitions used to indicate valid parameters of `Pmic_PwrBuckCfg_t`.
 * Set the `validParams` member of `Pmic_PwrBuckCfg_t` equal to a combination
 * of these defines by using the `OR` operator.
 *
 * @{
 */
#define PMIC_POWER_BUCK_EN_VALID        (1UL << 0U)
#define PMIC_POWER_BUCK_PLDN_EN_VALID   (1UL << 1U)
#define PMIC_POWER_BUCK_VMON_EN_VALID   (1UL << 2U)
#define PMIC_POWER_BUCK_FPWM_EN_VALID   (1UL << 3U)
#define PMIC_POWER_BUCK_VSET_VALID      (1UL << 4U)
#define PMIC_POWER_BUCK_SLEW_RATE_VALID (1UL << 5U)
#define PMIC_POWER_BUCK_VMON_THR_VALID  (1UL << 6U)
#define PMIC_POWER_BUCK_GRP_SEL_VALID   (1UL << 7U)
/** @} */

/**
 * @anchor Pmic_PwrLdoCfgValidParams
 * @name PMIC Power LDO Configuration Structure Valid Parameters
 *
 * @brief Definitions used to indicate valid parameters of `Pmic_PwrLdoCfg_t`.
 * Set the `validParams` member of `Pmic_PwrLdoCfg_t` equal to a combination
 * of these defines by using the `OR` operator.
 *
 * @{
 */
#define PMIC_POWER_LDO_EN_VALID           (1UL << 0U)
#define PMIC_POWER_LDO_BYP_EN_VALID       (1UL << 1U)
#define PMIC_POWER_LDO_VMON_EN_VALID      (1UL << 2U)
#define PMIC_POWER_LDO_DISCHARGE_EN_VALID (1UL << 3U)
#define PMIC_POWER_LDO_VSET_VALID         (1UL << 4U)
#define PMIC_POWER_LDO_VMON_THR_VALID     (1UL << 5U)
#define PMIC_POWER_LDO_GRP_SEL_VALID      (1UL << 6U)
/** @} */

/**
 * @anchor Pmic_PwrVccaVmonCfgValidParams
 * @name PMIC Power VCCA_VMON Configuration Structure Valid Parameters
 *
 * @brief Definitions used to indicate valid parameters of `Pmic_PwrVccaVmonCfg_t`.
 * Set the `validParams` member of `Pmic_PwrVccaVmonCfg_t` equal to a combination
 * of these defines by using the `OR` operator.
 *
 * @{
 */
#define PMIC_POWER_VCCA_VMON_EN_VALID      (1UL << 0U)
#define PMIC_POWER_VCCA_VMON_PG_SET_VALID  (1UL << 1U)
#define PMIC_POWER_VCCA_VMON_THR_VALID     (1UL << 2U)
#define PMIC_POWER_VCCA_VMON_GRP_SEL_VALID (1UL << 3U)
/** @} */

/**
 * @anchor Pmic_PwrRsrcStatusValidParams
 * @name PMIC Power Resource Status Valid Parameters
 *
 * @brief Definitions used to indicate valid parameters of `Pmic_PwrRsrcStatus_t`.
 * Set the `validParams` member of `Pmic_PwrRsrcStatus_t` equal to a combination
 * of these defines by using the `OR` operator.
 *
 * @{
 */
#define PMIC_POWER_BUCK1_UVOV_VALID     (1UL << 0U)
#define PMIC_POWER_BUCK2_UVOV_VALID     (1UL << 1U)
#define PMIC_POWER_BUCK3_UVOV_VALID     (1UL << 2U)
#define PMIC_POWER_BUCK4_UVOV_VALID     (1UL << 3U)
#define PMIC_POWER_LDO1_UVOV_VALID      (1UL << 4U)
#define PMIC_POWER_LDO2_UVOV_VALID      (1UL << 5U)
#define PMIC_POWER_LDO3_UVOV_VALID      (1UL << 6U)
#define PMIC_POWER_VMON1_UVOV_VALID     (1UL << 7U)
#define PMIC_POWER_VMON2_UVOV_VALID     (1UL << 8U)
#define PMIC_POWER_VCCA_VMON_UVOV_VALID (1UL << 9U)
/** @} */

/**
 * @anchor Pmic_PwrThermalCfgValidParams
 * @name PMIC Power Thermal Configuration Structure Valid Parameters
 *
 * @brief Definitions used to indicate valid parameters of `Pmic_PwrThermalCfg_t`.
 * Set the `validParams` member of `Pmic_PwrThermalCfg_t` equal to a combination
 * of these defines by using the `OR` operator.
 *
 * @{
 */
#define PMIC_POWER_TWARN_LEVEL_VALID   (1UL << 0U)
#define PMIC_POWER_TSD_ORD_LEVEL_VALID (1UL << 1U)
/** @} */

/**
 * @anchor Pmic_PwrSpreadSpectrumCfgValidParams
 * @name PMIC Power Spread Spectrum Configuration Structure Valid Parameters
 *
 * @brief Definitions used to indicate valid parameters of `Pmic_PwrSpreadSpectrumCfg_t`.
 * Set the `validParams` member of `Pmic_PwrSpreadSpectrumCfg_t` equal to a
 * combination of these defines by using the `OR` operator.
 *
 * @{
 */
#define PMIC_POWER_SS_EN_VALID    (1UL << 0U)
#define PMIC_POWER_SS_DEPTH_VALID (1UL << 1U)
/** @} */

/* ========================================================================== */
/*                           Structures and Enums                             */
/* ========================================================================== */

/**
 * @anchor Pmic_PwrBuckCfg
 * @name PMIC Power Buck Configuration Structure
 *
 * @brief Structure used to set and get PMIC power buck configurations.
 *
 * @param validParams Each bit in this variable corresponds to a member in this
 * structure. Specifically, if a bit is set to 1 in this variable, the corresponding
 * structure member is valid and will be considered by the driver API that is using
 * this data structure. Otherwise, if a bit is set to 0, the corresponding structure
 * member is invalid and will not be considered by the driver API that is using
 * this data structure. For possible valid parameter values, refer to
 * @ref Pmic_PwrBuckCfgValidParams.
 *
 * @param resource Buck resource. For valid values, refer to @ref Pmic_PwrResource.
 *
 * @param buckEn Enable/disable buck regulator.
 *
 * @param pldnEn Enable/disable output pull-down resistor when BUCK is de-activated.
 *
 * @param vmonEn Enable/disable buck UV and OV comparators.
 *
 * @param fpwmEn Enable/disable forced PWM mode.
 *
 * @param vset Set the output voltage for the buck regulator. For valid values,
 * refer to @ref Pmic_PwrBuck1Vset and @ref Pmic_PwrBuck2_3_4Vset.
 *
 * @param slewRate Set the slew rate for the buck regulator. For valid values,
 * refer to @ref Pmic_PwrBuckSlewRate.
 *
 * @param vmonThr Set the voltage monitoring threshold for the buck regulator.
 * For valid values, refer to @ref Pmic_PwrVmonThr.
 *
 * @param grpSel Set the group selection for the buck regulator. For valid
 * values, refer to @ref Pmic_PwrGrpSel.
 */
typedef struct Pmic_PwrBuckCfg_s {
    uint32_t validParams;
    uint16_t resource;

    bool buckEn;
    bool pldnEn;
    bool vmonEn;
    bool fpwmEn;

    uint8_t vset;
    uint8_t slewRate;
    uint8_t vmonThr;
    uint8_t grpSel;
} Pmic_PwrBuckCfg_t;

/**
 * @anchor Pmic_PwrLdoCfg
 * @name PMIC Power LDO Configuration Structure
 *
 * @brief Structure used to set and get PMIC power LDO configurations.
 *
 * @param validParams Each bit in this variable corresponds to a member in this
 * structure. Specifically, if a bit is set to 1 in this variable, the corresponding
 * structure member is valid and will be considered by the driver API that is using
 * this data structure. Otherwise, if a bit is set to 0, the corresponding structure
 * member is invalid and will not be considered by the driver API that is using
 * this data structure. For possible valid parameter values, refer to
 * @ref Pmic_PwrLdoCfgValidParams.
 *
 * @param resource LDO resource. For valid values, refer to @ref Pmic_PwrResource.
 *
 * @param ldoEn Enable/disable LDO regulator.
 *
 * @param bypEn Enable/disable LDO bypass mode.
 *
 * @param vmonEn Enable/disable LDO UV and OV comparators.
 *
 * @param dischargeEn Enable/disable LDO resistive discharge. When enabled, the
 * LDO will discharge onto a 200 Ohm resistor when it is disabled.
 *
 * @param vset Set the output voltage for the LDO regulator. For valid values,
 * refer to @ref Pmic_PwrLdo1Vset and @ref Pmic_PwrLdo2_3Vset.
 *
 * @param vmonThr Set the voltage monitoring threshold for the LDO regulator.
 * For valid values, refer to @ref Pmic_PwrVmonThr.
 *
 * @param grpSel Set the group selection for the LDO regulator. For valid
 * values, refer to @ref Pmic_PwrGrpSel.
 *
 */
typedef struct Pmic_PwrLdoCfg_s {
    uint32_t validParams;
    uint16_t resource;

    bool ldoEn;
    bool bypEn;
    bool vmonEn;
    bool dischargeEn;

    uint8_t vset;
    uint8_t vmonThr;
    uint8_t grpSel;
} Pmic_PwrLdoCfg_t;

/**
 * @anchor Pmic_PwrVccaVmonCfg
 * @name PMIC Power VCCA_VMON/VMONx Configuration Structure
 *
 * @brief Structure used to set and get PMIC power VCCA_VMON/VMONx configurations.
 *
 * @param validParams Each bit in this variable corresponds to a member in this
 * structure. Specifically, if a bit is set to 1 in this variable, the corresponding
 * structure member is valid and will be considered by the driver API that is using
 * this data structure. Otherwise, if a bit is set to 0, the corresponding structure
 * member is invalid and will not be considered by the driver API that is using
 * this data structure. For possible valid parameter values, refer to
 * @ref Pmic_PwrVccaVmonCfgValidParams.
 *
 * @param resource VCCA_VMON/VMONx resource. For valid values, refer to
 * @ref Pmic_PwrResource.
 *
 * @param vmonEn Enable/disable VMON for the specified resource.
 *
 * @param pgSet Power good threshold setting for the specified VMON. For valid
 * values, refer to @ref Pmic_PwrVccaVmonPgSet, @ref Pmic_PwrVmon1PgSet, and
 * @ref Pmic_PwrVmon2PgSet.
 *
 * @param vmonThr Set the voltage monitoring threshold for the specified VMON.
 * For valid values, refer to @ref Pmic_PwrVccaVmonThr and @ref Pmic_PwrVmonThr.
 *
 * @param grpSel Set the group selection for the specified VMON. For valid
 * values, refer to @ref Pmic_PwrGrpSel.
 */
typedef struct Pmic_PwrVccaVmonCfg_s {
    uint32_t validParams;
    uint16_t resource;

    bool vmonEn;

    uint8_t pgSet;
    uint8_t vmonThr;
    uint8_t grpSel;
} Pmic_PwrVccaVmonCfg_t;

/**
 * @anchor Pmic_PwrRsrcStatus
 * @name PMIC Power Resource Status Structure
 *
 * @brief Structure used to get the status of PMIC power resources.
 *
 * @param validParams Each bit in this variable corresponds to a member in this
 * structure. Specifically, if a bit is set to 1 in this variable, the corresponding
 * structure member is valid and will be considered by the driver API that is using
 * this data structure. Otherwise, if a bit is set to 0, the corresponding structure
 * member is invalid and will not be considered by the driver API that is using
 * this data structure. For possible valid parameter values, refer to
 * @ref Pmic_PwrRsrcStatusValidParams.
 *
 * @param buck1UVOV Indicates if the Buck 1 voltage is under or over the threshold.
 *
 * @param buck2UVOV Indicates if the Buck 2 voltage is under or over the threshold.
 *
 * @param buck3UVOV Indicates if the Buck 3 voltage is under or over the threshold.
 *
 * @param buck4UVOV Indicates if the Buck 4 voltage is under or over the threshold.
 *
 * @param ldo1UVOV Indicates if the LDO 1 voltage is under or over the threshold.
 *
 * @param ldo2UVOV Indicates if the LDO 2 voltage is under or over the threshold.
 *
 * @param ldo3UVOV Indicates if the LDO 3 voltage is under or over the threshold.
 *
 * @param vmon1UVOV Indicates if the VMON 1 voltage is under or over the threshold.
 *
 * @param vmon2UVOV Indicates if the VMON 2 voltage is under or over the threshold.
 *
 * @param vccaVmonUVOV Indicates if the VCCA_VMON voltage is under or over the
 * threshold.
 */
typedef struct Pmic_PwrRsrcStatus_s {
    uint32_t validParams;

    bool buck1UVOV;
    bool buck2UVOV;
    bool buck3UVOV;
    bool buck4UVOV;
    bool ldo1UVOV;
    bool ldo2UVOV;
    bool ldo3UVOV;
    bool vmon1UVOV;
    bool vmon2UVOV;
    bool vccaVmonUVOV;
} Pmic_PwrRsrcStatus_t;

/**
 * @anchor Pmic_PwrThermalCfg
 * @name PMIC Power Thermal Configuration Structure
 *
 * @brief Structure used to set and get PMIC power thermal configurations.
 *
 * @param validParams Each bit in this variable corresponds to a member in this
 * structure. Specifically, if a bit is set to 1 in this variable, the corresponding
 * structure member is valid and will be considered by the driver API that is using
 * this data structure. Otherwise, if a bit is set to 0, the corresponding structure
 * member is invalid and will not be considered by the driver API that is using
 * this data structure. For possible valid parameter values, refer to
 * @ref Pmic_PwrThermalCfgValidParams.
 *
 * @param twarnLvl Warning temperature level. For valid values, refer to
 * @ref Pmic_pwrTwarnLvl.
 *
 * @param tsdOrdLvl Orderly shutdown level. For valid values, refer to
 * @ref Pmic_pwrTsdOrdLvl.
 */
typedef struct Pmic_PwrThermalCfg_s {
    uint32_t validParams;

    uint8_t twarnLvl;
    uint8_t tsdOrdLvl;
} Pmic_PwrThermalCfg_t;

/**
 * @anchor Pmic_PwrSpreadSpectrumCfg
 * @name PMIC Power Spread Spectrum Configuration Structure
 *
 * @brief Structure used to set and get PMIC power spread spectrum configurations.
 *
 * @param validParams Each bit in this variable corresponds to a member in this
 * structure. Specifically, if a bit is set to 1 in this variable, the corresponding
 * structure member is valid and will be considered by the driver API that is using
 * this data structure. Otherwise, if a bit is set to 0, the corresponding structure
 * member is invalid and will not be considered by the driver API that is using
 * this data structure. For possible valid parameter values, refer to
 * @ref Pmic_PwrSpreadSpectrumCfgValidParams.
 *
 * @param ssEn Enable/disable spread spectrum.
 *
 * @param ssDepth Spread spectrum modulation depth.
 */
typedef struct Pmic_PwrSpreadSpectrumCfg_s {
    uint32_t validParams;

    bool ssEn;
    bool ssDepth;
} Pmic_PwrSpreadSpectrumCfg_t;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 * @brief Set PMIC power buck configurations.
 *
 * Design: PMICDRV-719
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-521, PMICDRV-522, PMICDRV-523, PMICDRV-535
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param buckCfg [IN] Desired buck configurations to set. For more information
 * on buck configurations, refer to @ref Pmic_PwrBuckCfg.
 *
 * @return PMIC_ST_SUCCESS if buck configurations have been set, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_pwrSetBuckCfg(const Pmic_Handle_t *handle, const Pmic_PwrBuckCfg_t *buckCfg);

/**
 * @brief Get PMIC power buck configurations.
 *
 * Design: PMICDRV-720
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-521, PMICDRV-522, PMICDRV-528, PMICDRV-535
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param buckCfg [OUT] Buck configurations obtained from the PMIC. For more
 * information on buck configurations, refer to @ref Pmic_PwrBuckCfg.
 *
 * @return PMIC_ST_SUCCESS if buck configurations have been obtained, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_pwrGetBuckCfg(const Pmic_Handle_t *handle, Pmic_PwrBuckCfg_t *buckCfg);

/**
 * @brief Set PMIC LDO configurations.
 *
 * Design: PMICDRV-721
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-521, PMICDRV-522, PMICDRV-523, PMICDRV-535
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param ldoCfg [IN] Desired LDO configurations to set. For more information
 * on LDO configurations, refer to @ref Pmic_PwrLdoCfg.
 *
 * @return PMIC_ST_SUCCESS if LDO configurations have been set, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_pwrSetLdoCfg(const Pmic_Handle_t *handle, const Pmic_PwrLdoCfg_t *ldoCfg);

/**
 * @brief Get PMIC LDO configurations.
 *
 * Design: PMICDRV-722
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-521, PMICDRV-522, PMICDRV-528, PMICDRV-535
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param ldoCfg [OUT] LDO configurations obtained from the PMIC. For more
 * information on LDO configurations, refer to @ref Pmic_PwrLdoCfg.
 *
 * @return PMIC_ST_SUCCESS if LDO configurations have been obtained, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_pwrGetLdoCfg(const Pmic_Handle_t *handle, Pmic_PwrLdoCfg_t *ldoCfg);

/**
 * @brief Set PMIC VCCA_VMON/VMONx configurations.
 *
 * Design: PMICDRV-723
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-521, PMICDRV-522, PMICDRV-523, PMICDRV-535
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param vccaVmonCfg [IN] Desired VCCA_VMON/VMONx configurations to set. For
 * more information on VCCA_VMON/VMONx configurations, refer to
 * @ref Pmic_PwrVccaVmonCfg.
 *
 * @return PMIC_ST_SUCCESS if VCCA_VMON/VMONx configurations have been set, error
 * code otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_pwrSetVccaVmonCfg(const Pmic_Handle_t *handle, const Pmic_PwrVccaVmonCfg_t *vccaVmonCfg);

/**
 * @brief Get PMIC VCCA_VMON/VMONx configurations.
 *
 * Design: PMICDRV-724
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-521, PMICDRV-522, PMICDRV-528, PMICDRV-535
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param vccaVmonCfg [OUT] VCCA_VMON/VMONx configurations obtained from the PMIC. For
 * more information on VCCA_VMON/VMONx configurations, refer to
 * @ref Pmic_PwrVccaVmonCfg.
 *
 * @return PMIC_ST_SUCCESS if VCCA_VMON/VMONx configurations have been obtained, error
 * code otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_pwrGetVccaVmonCfg(const Pmic_Handle_t *handle, Pmic_PwrVccaVmonCfg_t *vccaVmonCfg);

/**
 * @brief Set deglitch configuration for all VMONs.
 *
 * Design: PMICDRV-725
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-521, PMICDRV-522, PMICDRV-523, PMICDRV-535
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param vmonDegl [IN] Desired VMON deglitch configuration value to set. For
 * valid values, refer to @ref Pmic_PwrVmonDegl.
 *
 * @return PMIC_ST_SUCCESS if VMON deglitch configuration has been set, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_pwrSetGlobalVmonDegl(const Pmic_Handle_t *handle, uint8_t vmonDegl);

/**
 * @brief Set PMIC thermal configurations.
 *
 * Design: PMICDRV-726
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-521, PMICDRV-522, PMICDRV-523, PMICDRV-535
 *               PMICDRV-536
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param thermalCfg [IN] Desired thermal configurations to set. For more
 * information on thermal configurations, refer to @ref Pmic_PwrThermalCfg.
 *
 * @return PMIC_ST_SUCCESS if thermal configurations have been set, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_pwrSetThermalCfg(const Pmic_Handle_t *handle, const Pmic_PwrThermalCfg_t *thermalCfg);

/**
 * @brief Get PMIC thermal configurations.
 *
 * Design: PMICDRV-727
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-521, PMICDRV-522, PMICDRV-528, PMICDRV-535
 *               PMICDRV-536
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param thermalCfg [OUT] Thermal configurations obtained from the PMIC. For more
 * information on thermal configurations, refer to @ref Pmic_PwrThermalCfg.
 *
 * @return PMIC_ST_SUCCESS if thermal configurations have been obtained, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_pwrGetThermalCfg(const Pmic_Handle_t *handle, Pmic_PwrThermalCfg_t *thermalCfg);

/**
 * @brief Set PMIC spread spectrum configurations.
 *
 * Design: PMICDRV-728
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-521, PMICDRV-522, PMICDRV-523, PMICDRV-535
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param spreadSpectrumCfg [IN] Desired spread spectrum configurations to set. For
 * more information on spread spectrum configurations, refer to
 * @ref Pmic_PwrSpreadSpectrumCfg.
 *
 * @return PMIC_ST_SUCCESS if spread spectrum configurations have been set, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_pwrSetSpreadSpectrumCfg(const Pmic_Handle_t *handle, const Pmic_PwrSpreadSpectrumCfg_t *spreadSpectrumCfg);

/**
 * @brief Get PMIC spread spectrum configurations.
 *
 * Design: PMICDRV-729
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-521, PMICDRV-522, PMICDRV-528, PMICDRV-535
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param spreadSpectrumCfg [OUT] Spread spectrum configurations obtained from the PMIC. For
 * more information on spread spectrum configurations, refer to
 * @ref Pmic_PwrSpreadSpectrumCfg.
 *
 * @return PMIC_ST_SUCCESS if spread spectrum configurations have been obtained, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_pwrGetSpreadSpectrumCfg(const Pmic_Handle_t *handle, Pmic_PwrSpreadSpectrumCfg_t *spreadSpectrumCfg);

/**
 * @brief Get PMIC resource status.
 *
 * Design: PMICDRV-647
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-508, PMICDRV-511, PMICDRV-512,
 *               PMICDRV-515, PMICDRV-516, PMICDRV-521, PMICDRV-522, PMICDRV-527,
 *               PMICDRV-528, PMICDRV-535, PMICDRV-536, PMICDRV-551
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param rsrcStatus [OUT] Resource statuses obtained from the PMIC. For more
 * information on resource status, refer to @ref Pmic_PwrRsrcStatus.
 *
 * @return PMIC_ST_SUCCESS if resource statuses has been obtained, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_pwrGetRsrcStatus(const Pmic_Handle_t *handle, Pmic_PwrRsrcStatus_t *rsrcStatus);

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* PMIC_POWER_H */
