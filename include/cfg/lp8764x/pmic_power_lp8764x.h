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
 *  \addtogroup DRV_PMIC_MODULE
 *
 *  @{
 */

/**
 * \file   pmic_power_lp8764x.h
 *
 * \brief  PMIC LP8764x Leo PMIC Power Resources Driver API/interface file
 *
 */

#ifndef PMIC_POWER_LP8764X_H_
#define PMIC_POWER_LP8764X_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/**
 *  \anchor Pmic_Lp8764xHera_Power_ResourceType
 *  \name   PMIC Power Resource Type for LEO TPS6594x
 *
 *  @{
 */
#define PMIC_LP8764X_POWER_RESOURCE_TYPE_VCCA           (0U)
#define PMIC_LP8764X_POWER_RESOURCE_TYPE_BUCK           (1U)
#define PMIC_LP8764X_POWER_RESOURCE_TYPE_LDO            (2U)
#define PMIC_LP8764X_POWER_RESOURCE_TYPE_VMON           (3U)
/*  @} */

/**
 *  \anchor Pmic_Lp8764xHera_Power_Resource
 *  \name   PMIC Power Resources for HERA lp8764x
 *
 *  @{
 */
#define PMIC_LP8764X_POWER_SOURCE_VCCA     \
               ((uint16_t)((PMIC_LP8764X_POWER_RESOURCE_TYPE_VCCA << 8) | 0x0))
#define PMIC_LP8764X_REGULATOR_BUCK1       \
               ((uint16_t) ((PMIC_LP8764X_POWER_RESOURCE_TYPE_BUCK << 8) | 0x1))
#define PMIC_LP8764X_REGULATOR_BUCK2       \
               ((uint16_t) ((PMIC_LP8764X_POWER_RESOURCE_TYPE_BUCK << 8) | 0x2))
#define PMIC_LP8764X_REGULATOR_BUCK3       \
               ((uint16_t) ((PMIC_LP8764X_POWER_RESOURCE_TYPE_BUCK << 8) | 0x3))
#define PMIC_LP8764X_REGULATOR_BUCK4       \
               ((uint16_t) ((PMIC_LP8764X_POWER_RESOURCE_TYPE_BUCK << 8) | 0x4))
#define PMIC_LP8764X_POWER_SOURCE_VMON1    \
               ((uint16_t) ((PMIC_LP8764X_POWER_RESOURCE_TYPE_VMON << 8) | 0x5))
#define PMIC_LP8764X_POWER_SOURCE_VMON2    \
               ((uint16_t) ((PMIC_LP8764X_POWER_RESOURCE_TYPE_VMON << 8) | 0x6))
/*  @} */

/**
 *  \anchor Pmic_LP8764x_Regulator_Vmon_RV_Check
 *  \name   PMIC Residual voltage check Enable/Disable
 *          Valid only for VMON1, VMON2, LDO, BUCK
 *
 *  @{
 */
/** \brief Used to enable the residual voltage check */
#define PMIC_LP8764X_REGULATOR_VMON_RV_SEL_ENABLE              (0x1U)
/** \brief Used to disable the residual voltage check */
#define PMIC_LP8764X_REGULATOR_VMON_RV_SEL_DISABLE             (0x0U)
/* @} */

/**
 *  \anchor Pmic_LP8764x_Buck_Pull_Down_Resistor
 *  \name   PMIC Pull-down resistor Enable/Disable for BUCK Regulator.
 *
 *  @{
 */
/** \brief Used to enable the pull down resistor for BUCK regulator */
#define PMIC_LP8764X_REGULATOR_BUCK_PLDN_ENABLE           (0x1U)
/** \brief Used to disable the pull down resistor for BUCK regulator */
#define PMIC_LP8764X_REGULATOR_BUCK_PLDN_DISABLE          (0x0U)
/* @} */

/**
 *  \anchor Pmic_LP8764x_Vmon_Enable
 *  \name   PMIC Voltage monitor Enable/Disable for BUCK/VMON1/VMON2/VCCA
 *          Enable/Disable OV and UV comparators for VMON1/VMON2/VCCA.
 *          Enable/Disable OV, UV, SC and ILIM for BUCK
 *
 *  @{
 */
/** \brief Used to disable the voltage monitor */
#define PMIC_LP8764X_VMON_DISABLE          (0x0U)
/** \brief Used to enable the voltage monitor */
#define PMIC_LP8764X_VMON_ENABLE           (0x1U)
/* @} */

/**
 *  \anchor Pmic_LP8764x_Regulator_Buck_Vout_Sel
 *  \name   PMIC Select output voltage register for BUCK.
 *          Valid only for BUCK Regulator
 *
 *  @{
 */
/** \brief Used to select VOUT2 register for voltage selection */
#define PMIC_LP8764X_REGULATOR_BUCK_VOUT_SEL_VOUT2        (0x1U)
/** \brief Used to select VOUT1 register for voltage selection */
#define PMIC_LP8764X_REGULATOR_BUCK_VOUT_SEL_VOUT1        (0x0U)
/* @} */

/**
 *  \anchor Pmic_LP8764x_Regulator_Pwm_Pfm_Mode
 *  \name   PMIC Select between Automatic transitions between PFM and PWM modes
 *          OR Forced to PWM operation
 *          Valid only for BUCK regulators.
 *
 *  @{
 */
/** \brief Used to select PWM mode */
#define PMIC_LP8764X_REGULATOR_PWM_MODE                   (0x1U)
/** \brief Used to select Automatic transition between PFM and PWM modes */
#define PMIC_LP8764X_REGULATOR_AUTO_PWM_PFM_MODE          (0x0U)
/* @} */

/**
 *  \anchor PMIC_LP8764X_Regulator_Pwm_Mp_Mode
 *  \name   PMIC Select btetween multi-phase operation OR
 *          AUTO mode with Automatic phase adding and shedding.
 *          Valid only for BUCK regulators.
 *
 *  @{
 */
/** \brief Used to select multi-phase operation */
#define PMIC_LP8764X_REGULATOR_PWM_MP_MODE                (0x1U)
/** \brief Used to select Automatic phase adding and shedding mode */
#define PMIC_LP8764X_REGULATOR_AUTO_PHASE_MODE            (0x0U)
/* @} */

/**
 *  \anchor Pmic_LP8764x_Buck_Regulator_enable
 *  \name   PMIC Enable/Disable BUCK Regulator.
 *
 *  @{
 */
/** \brief Used to enable the BUCK regulator */
#define PMIC_LP8764X_BUCK_REGULATOR_ENABLE                     (0x1U)
/** \brief Used to disable the BUCK regulator */
#define PMIC_LP8764X_BUCK_REGULATOR_DISABLE                    (0x0U)
/* @} */

/**
 *  \anchor Pmic_LP8764x_Vmon_DeglitchTime_Sel
 *  \name   PMIC Deglitch time select for BUCKx_VMON/VCCA_VMON/VMONx
 *
 *  @{
 */
/** \brief Used to select the degitch time as 4 usec */
#define PMIC_LP8764X_POWER_RESOURCE_DEGLITCH_SEL_4US      (0x0U)
/** \brief Used to select the degitch time as 20 usec */
#define PMIC_LP8764X_POWER_RESOURCE_DEGLITCH_SEL_20US     (0x1U)
/*  @} */

/**
 *  \anchor Pmic_LP8764x_VccaPowerGoodLimit
 *  \name   PMIC Powergood level for VCCA
 *
 *  @{
 */
/** \brief Used to select the powergood level for VCCA to be 3.3v */
#define PMIC_LP8764X_VCCA_PG_3V3_LEVEL                    (0x0U)
/** \brief Used to select the powergood level for VCCA to be 5v */
#define PMIC_LP8764X_VCCA_PG_5V_LEVEL                     (0x1U)
/* @} */

/**
 *  \anchor Pmic_LP8764x_Power_Vmon_Range
 *  \name   OV/UV voltage monitoring range selection for VMON1/VMON2
 *
 *  @{
 */
/** \brief Used to select range 0.3 -3.34V */
#define PMIC_LP8764X_VMON_RANGE_0V3_3V34                 (0x0U)
/** \brief Used to select range 3.5 -5V */
#define PMIC_LP8764X_VMON_RANGE_3V35_5V                  (0x1U)
/* @} */

/**
 *  \anchor Pmic_LP8764x_Buck_Freq_Sel
 *  \name   PMIC Select switching frequency for BUCK
 *
 *  @{
 */
/** \brief Used to select frequency as 2.2 Mhz */
#define PMIC_LP8764X_BUCK_FREQ_SEL_2M2                (0x0U)
/** \brief Used to select frequency as 4.4 Mhz */
#define PMIC_LP8764X_BUCK_FREQ_SEL_4M4                (0x1U)
/** \brief Used to select frequency as 8.8 Mhz */
#define PMIC_LP8764X_BUCK_FREQ_SEL_8M8                (0x2U)
/* @} */

/**
 *  \anchor Pmic_LP8764x_Buck_Current_Limit
 *  \name   PMIC Switch Peak Current limit for BUCK Regulator
 *
 *  @{
 */
/** \brief Used to configure BUCK current limit as 2.5 Ampere */
#define PMIC_LP8764X_REGULATOR_BUCK_CURRENT_LIMIT_2A5     (0x2U)
/** \brief Used to configure BUCK current limit as 3.5 Ampere */
#define PMIC_LP8764X_REGULATOR_BUCK_CURRENT_LIMIT_3A5     (0x3U)
/** \brief Used to configure BUCK current limit as 4.5 Ampere */
#define PMIC_LP8764X_REGULATOR_BUCK_CURRENT_LIMIT_4A5     (0x4U)
/** \brief Used to configure BUCK current limit as 5.5 Ampere */
#define PMIC_LP8764X_REGULATOR_BUCK_CURRENT_LIMIT_5A5     (0x5U)
/** \brief Used to configure BUCK current limit as 6.5 Ampere */
#define PMIC_LP8764X_REGULATOR_BUCK_CURRENT_LIMIT_6A5     (0x6U)
/*  @} */

/**
 *  \anchor Pmic_LP8764x_Buck_Vmon_Slew_Rate
 *  \name   PMIC Output voltage slew rate for BUCKx/VMONx
 *
 *  @{
 */
/** \brief Used to configure BUCK current limit as 30mv */
#define PMIC_LP8764X_REGULATOR_BUCK_VMON_SLEW_RATE_33MV     (0x0U)
/** \brief Used to configure BUCK current limit as 20mv */
#define PMIC_LP8764X_REGULATOR_BUCK_VMON_SLEW_RATE_20MV     (0x1U)
/** \brief Used to configure BUCK current limit as 10mv */
#define PMIC_LP8764X_REGULATOR_BUCK_VMON_SLEW_RATE_10MV     (0x2U)
/** \brief Used to configure BUCK current limit as 5mv */
#define PMIC_LP8764X_REGULATOR_BUCK_VMON_SLEW_RATE_05MV     (0x3U)
/** \brief Used to configure BUCK current limit as 2.5mv */
#define PMIC_LP8764X_REGULATOR_BUCK_VMON_SLEW_RATE_2MV5     (0x4U)
/** \brief Used to configure BUCK current limit as 1.3mv */
#define PMIC_LP8764X_REGULATOR_BUCK_VMON_SLEW_RATE_1MV3     (0x5U)
/** \brief Used to configure BUCK current limit as 0.63mv */
#define PMIC_LP8764X_REGULATOR_BUCK_VMON_SLEW_RATE_0MV63    (0x6U)
/** \brief Used to configure BUCK current limit as 0.31mv */
#define PMIC_LP8764X_REGULATOR_BUCK_VMON_SLEW_RATE_0MV31    (0x7U)
/*  @} */

/**
 *  \anchor Pmic_LP8764x_Pg_Ov_Uv_Threshold_lvl
 *  \name   PMIC Power Good Over/Under voltage threshold level for BUCK/LDO/VCCA
 *          For LDO/BUCK - Over/Under Volatge thershold level are +x1 mv/ +x2 %
 *          or -x1 mv/ -x2 % respectively. For VCCA only perc
 *          For VCCA - Over/Under Volatge thershold level are +x2 % or -x2 %
 *          respectively.
 *  @{
 */
#define PMIC_LP8764X_PG_OV_UV_THRESHOLD_LVL_30_OR_3       (0U)
/** \brief Used to select over/under voltage threshold level as +/-35mv or
 *         +/-3.5%
 */
#define PMIC_LP8764X_PG_OV_UV_THRESHOLD_LVL_35_OR_3P5     (1U)
/** \brief Used to select over/under voltage threshold level as +/-40mv or
 *         +/-4%
 */
#define PMIC_LP8764X_PG_OV_UV_THRESHOLD_LVL_40_OR_4       (2U)
/** \brief Used to select over/under voltage threshold level as +/-50mv or
 *         +/-5%
 */
#define PMIC_LP8764X_PG_OV_UV_THRESHOLD_LVL_50_OR_5       (3U)
/** \brief Used to select over/under voltage threshold level as +/-60mv or
 *         +/-6%
 */
#define PMIC_LP8764X_PG_OV_UV_THRESHOLD_LVL_60_OR_6       (4U)
/** \brief Used to select over/under voltage threshold level as +/-70mv or
 *         +/-7%
 */
#define PMIC_LP8764X_PG_OV_UV_THRESHOLD_LVL_70_OR_7       (5U)
/** \brief Used to select over/under voltage threshold level as +/-80mv or
 *         +/-8%
 */
#define PMIC_LP8764X_PG_OV_UV_THRESHOLD_LVL_80_OR_8       (6U)
/** \brief Used to select over/under voltage threshold level as +/-100mv or
 *         +/-10%
 */
#define PMIC_LP8764X_PG_OV_UV_THRESHOLD_LVL_100_OR_10     (7U)
/*  @} */


/**
 *  \anchor Pmic_LP8764x_Power_Rail_Sel
 *  \name   PMIC Rail group selection for all power resources.
 *
 *  @{
 */
/** \brief Used to select rail group as no group */
#define PMIC_LP8764X_POWER_RAIL_SEL_NONE                  (0x0U)
/** \brief Used to select rail group as MCU rail group */
#define PMIC_LP8764X_POWER_RAIL_SEL_MCU                   (0x1U)
/** \brief Used to select rail group as SOC rail group */
#define PMIC_LP8764X_POWER_RAIL_SEL_SOC                   (0x2U)
/** \brief Used to select rail group as other rail group */
#define PMIC_LP8764X_POWER_RAIL_SEL_OTHER                 (0x3U)
/* @} */

/*==========================================================================*/
/*                         Structures and Enums                             */
/*==========================================================================*/

/*==========================================================================*/
/*                         Function Declarations                            */
/*==========================================================================*/

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif/* PMIC_POWER_LP8764X_H_ */

/* @} */
