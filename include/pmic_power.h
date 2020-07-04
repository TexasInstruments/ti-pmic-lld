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
 *          PMIC Power Resources valid params configuration type for
 *          the structure member validParams of \ref Pmic_PowerResourceCfg_s
 *
 *  @{
 */
/** \brief Valid only for VMON1/VMON2/BUCK/LDO */
#define PMIC_CFG_REGULATOR_VMON_RV_SEL_VALID              (0U)
/** \brief Valid only for BUCK regulator */
#define PMIC_CFG_REGULATOR_BUCK_PLDN_EN_VALID             (1U)
/** \brief Valid only for VMON1/VMON2/VMON of BUCK/VMON of LDO/VMON of VCCA
 *         Power Resources
 */
#define PMIC_CFG_VMON_EN_VALID                            (2U)
/** \brief Valid only for BUCK regulator */
#define PMIC_CFG_REGULATOR_BUCK_VOUT_SEL_VALID            (3U)
/** \brief Valid only for BUCK regulator */
#define PMIC_CFG_REGULATOR_BUCK_FPWM_VALID                (4U)
/** \brief Valid only for BUCK regulator */
#define PMIC_CFG_REGULATOR_BUCK_PWM_MP_VALID              (5U)
/** \brief Valid only for BUCK/LDO regulator */
#define PMIC_CFG_REGULATOR_EN_VALID                       (6U)
/** \brief Valid only for LDO regulator */
#define PMIC_CFG_REGULATOR_LDO_SLOW_RAMP_EN_VALID         (7U)
/** \brief Valid only for LDO regulator */
#define PMIC_CFG_REGULATOR_LDO_BYPASS_MODE_EN_VALID       (8U)
/** \brief Valid only for VMON1/VMON2/VCCA/VMON of LDO/VMON of BUCK power
 *         resources
 */
#define PMIC_CFG_DEGLITCH_TIME_SEL_VALID                  (9U)
/** \brief Valid only for VCCA */
#define PMIC_CFG_VCCA_PWR_GOOD_LVL_VALID                  (10U)
/** \brief Valid only for VMON1/VMON2 */
#define PMIC_CFG_VMON_RANGE_VALID                         (11U)
/** \brief Valid only for BUCK regulator */
#define PMIC_CFG_REGULATOR_BUCK_FREQ_SEL_VALID            (12U)
/** \brief Valid only for BUCK regulator */
#define PMIC_CFG_REGULATOR_BUCK_ILIM_VALID                (13U)
/** \brief Valid only for VMON1/VMON2/BUCK */
#define PMIC_CFG_REGULATOR_BUCK_VMON_SLEW_RATE_VALID      (14U)
/** \brief Valid only for LDO regulator */
#define PMIC_CFG_REGULATOR_LDO_PLDN_SEL_VALID             (15U)
/** \brief Valid only for LDO regulator */
#define PMIC_CFG_REGULATOR_LDO_RV_TIMEOUT_SEL_VALID       (16U)
/** \brief Valid for all power resources */
#define PMIC_CFG_PWR_RESOURCE_PG_UV_THRESHOLD_LVL_VALID   (17U)
/** \brief Valid for all power resources */
#define PMIC_CFG_PWR_RESOURCE_PG_OV_THRESHOLD_LVL_VALID   (18U)
/** \brief Valid for all power resources */
#define PMIC_CFG_PWR_RESOURCE_RAIL_GRP_SEL_VALID          (19U)
/** \brief Valid only for VMON1/VMMON2/BUCK/LDO */
#define PMIC_CFG_REGULATOR_VMON_VOLTAGE_SET_VALID         (20U)
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
#define PMIC_CFG_DEGLITCH_TIME_SEL_VALID_SHIFT \
                         (1U << PMIC_CFG_DEGLITCH_TIME_SEL_VALID)
#define PMIC_CFG_VCCA_PWR_GOOD_LVL_VALID_SHIFT             \
                         (1U << PMIC_CFG_VCCA_PWR_GOOD_LVL_VALID)
#define PMIC_CFG_VMON_RANGE_VALID_SHIFT      (1U << PMIC_CFG_VMON_RANGE_VALID)
#define PMIC_CFG_REGULATOR_BUCK_FREQ_SEL_VALID_SHIFT     \
                         (1U << PMIC_CFG_REGULATOR_BUCK_FREQ_SEL_VALID)
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

/*==========================================================================*/
/*                         Structures and Enums                             */
/*==========================================================================*/

/*!
 *  \anchor  Pmic_PowerResourceCfg_s
 *  \brief   Pmic Power resources control and configuration structure.
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
 *  \param   ldoBypassModeEn     Selects Bypass/Linear Regulator LDO mode.
 *                               Valid only for TPS6594X Leo. For Valid Values
 *                               \ref Pmic_TPS6594x_Regulator_Ldo_Mode.
 *                               Valid only when
 *                               PMIC_CFG_REGULATOR_LDO_BYPASS_MODE_EN_VALID
 *                               bit is set.
 *  \param   deglitchTimeSel     Deglitch time select for all power resources
 *                               Valid values for TPS6594x Leo Device
 *                               \ref Pmic_TPS6594x_Vmon_DeglitchTime_Sel.
 *                               Valid values for LP8764x HERA Device
 *                               \ref Pmic_LP8764x_Vmon_DeglitchTime_Sel.
 *                               Valid only when
 *                               PMIC_CFG_DEGLITCH_TIME_SEL_VALID bit is set.
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
 *  \param   buckFreqSel         Select the BUCK switching frequency.
 *                               Note: Should not be changed while BUCK is in
 *                               operation.
 *                               Valid values for TPS6594x Leo Device
 *                               \ref Pmic_TPS6594x_Buck_Freq_Sel.
 *                               Valid values for LP8764x HERA Device
 *                               \ref Pmic_LP8764x_Buck_Freq_Sel.
 *                               Valid only when
 *                               PMIC_CFG_REGULATOR_BUCK_FREQ_SEL_VALID bit
 *                               is set
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
    bool     deglitchTimeSel;
    bool     vccaPwrGudLvl;
    bool     vmonRange;
    uint8_t  buckFreqSel;
    uint8_t  buckCurrentLimit;
    uint8_t  buckVmonSlewRate;
    uint8_t  ldoPullDownSel;
    uint8_t  ldoRvTimeoutSel;
    uint8_t  pgUvThresholdLvl;
    uint8_t  pgOvThresholdLvl;
    uint8_t  railGrpSel;
    uint16_t voltage_mV;
}Pmic_PowerResourceCfg_t;

/*==========================================================================*/
/*                         Function Declarations                            */
/*==========================================================================*/

/**
 * \brief   API to set power resources configurations.
 *          This function can be used to configure the various control and
 *          configuration parameters for BUCK/LDO/VCCA/VMON power resources and
 *          also used to set the various control and configuration of
 *          voltage monitor parameters for BUCK/LDO/VCCA/VMON power resources
 *
 *          To set control and configuration params for BUCK, the application
 *          has to configure the below defined structure members of
 *          Pmic_PowerResourceCfg_t:
 *          rvCheckEn, buckPullDownEn, vmonEn, buckVoutRegSel, buckFpwmMode,
 *          buckFpwmMpMode, buckFreqSel, regulatorEn, buckCurrentLimit,
 *          buckVmonSlewRate, voltage_mV, deglitchTimeSel, pgUvThresholdLvl,
 *          pgOvThresholdLvl, railGrpSel
 *
 *          To set control and configuration params for LDO, the application
 *          has to configure the below defined structure members of
 *          Pmic_PowerResourceCfg_t:
 *          rvCheckEn, vmonEn, regulatorEn, ldoPullDownSel, ldoSlowRampEn,
 *          ldoBypassModeEn, ldoRvTimeoutSel, voltage_mV, deglitchTimeSel,
 *          pgUvThresholdLvl, pgOvThresholdLvl, railGrpSel
 *
 *          To set control and configuration params for VCCA, the application
 *          has to configure the below defined structure members of
 *          Pmic_PowerResourceCfg_t:
 *          vmonEn, deglitchTimeSel, vccaPwrGudLvl, pgUvThresholdLvl,
 *          pgOvThresholdLvl, railGrpSel
 *
 *          To set control and configuration params for VMON, the application
 *          has to configure the below defined structure members of
 *          Pmic_PowerResourceCfg_t:
 *          rvCheckEn, vmonEn, deglitchTimeSel, vccaPwrGudLvl, vmonRange,
 *          pgUvThresholdLvl, pgOvThresholdLvl, railGrpSel, voltage_mV
 *          Valid only for LP8764x HERA Device
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
 *          This function can be used to get the various control and
 *          configuration parameters for BUCK/LDO/VCCA/VMON power resources and
 *          also used to get the various control and configuration of
 *          voltage monitor parameters for BUCK/LDO/VCCA/VMON power resources
 *
 *          Application can get these control and configuration params for BUCK,
 *          which is stored in the below defined structure members of
 *          Pmic_PowerResourceCfg_t:
 *          rvCheckEn, buckPullDownEn, vmonEn, buckVoutRegSel, buckFpwmMode,
 *          buckFpwmMpMode, buckFreqSel, regulatorEn, buckCurrentLimit,
 *          buckVmonSlewRate, voltage_mV, deglitchTimeSel, pgUvThresholdLvl,
 *          pgOvThresholdLvl, railGrpSel
 *
 *          Application can get these control and configuration params for LDO,
 *          which is stored in the below defined structure members of
 *          Pmic_PowerResourceCfg_t:
 *          rvCheckEn, vmonEn, regulatorEn, ldoPullDownSel, ldoSlowRampEn,
 *          ldoBypassModeEn, ldoRvTimeoutSel, voltage_mV, deglitchTimeSel,
 *          pgUvThresholdLvl, pgOvThresholdLvl, railGrpSel
 *
 *          Application can get these control and configuration params for VCCA,
 *          which is stored in the below defined structure members of
 *          Pmic_PowerResourceCfg_t:
 *          vmonEn, deglitchTimeSel, vccaPwrGudLvl, pgUvThresholdLvl,
 *          pgOvThresholdLvl, railGrpSel
 *
 *          Application can get these control and configuration params for VMON,
 *          which is stored in the below defined structure members of
 *          Pmic_PowerResourceCfg_t:
 *          rvCheckEn, vmonEn, deglitchTimeSel, vccaPwrGudLvl, vmonRange,
 *          pgUvThresholdLvl, pgOvThresholdLvl, railGrpSel, voltage_mV
 *          Valid only for LP8764x HERA Device
 *
 * \param   pPmicCoreHandle    [IN]    PMIC Interface Handle.
 * \param   pwrResource        [IN]    PMIC Power resource
 *                                     Valid values for TPS6594x Leo Device
 *                                     \ref Pmic_Tps6594xLeo_Power_Resource.
 *                                     Valid values for LP8764x HERA Device
 *                                     \ref Pmic_Lp8764xHera_Power_Resource.
 * \param   pPwrResourceCfg    [OUT]   Pointer to store Power Resource
 *                                     configuration for BUCK/LDO/VMON/VCCA
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code.
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_powerGetPwrResourceCfg(
                                 Pmic_CoreHandle_t           *pPmicCoreHandle,
                                 const uint16_t               pwrResource,
                                 Pmic_PowerResourceCfg_t     *pPwrResourceCfg);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif/* PMIC_POWER_H_ */

/* @} */

