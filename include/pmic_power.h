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
#ifndef PMIC_POWER_H
#define PMIC_POWER_H

/**
 * @file pmic_power.h
 * @brief PMIC Driver Power API/Interface
 */

/**
 * @defgroup DRV_PMIC_PWR_MODULE PMIC Power Module
 * @brief Power Resource control, configuration, and status information.
 */

#include "pmic_common.h"
#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                Include Files                               */
/* ========================================================================== */
#include <stdint.h>
#include <stdbool.h>

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/**
 * @anchor Pmic_PwrResource
 * @name PMIC Power Resource identifiers
 *
 * @{
 */
#define PMIC_PWR_RSRC_BUCK1                   (0U)
#define PMIC_PWR_RSRC_BUCK2                   (1U)
#define PMIC_PWR_RSRC_BUCK3                   (2U)
#define PMIC_PWR_RSRC_LDO_LS1_VMON1           (3U)
#define PMIC_PWR_RSRC_LS2_VMON2               (4U)
#define PMIC_PWR_RSRC_VCCA_VMON               (5U)
#define PMIC_PWR_RSRC_GPO                     (6U)
#define PMIC_PWR_RSRC_NRSTOUT                 (7U)
#define PMIC_PWR_RSRC_MAX                     (PMIC_PWR_RSRC_NRSTOUT)
/** @} */

/**
 * @anchor Pmic_PwrResourceModeConfig
 * @name PMIC Power Resource mode configuration options
 *
 * @{
 */
#define PMIC_PWR_RSRC_MODE_REG                (0U)
#define PMIC_PWR_RSRC_MODE_BYP                (1U)
#define PMIC_PWR_RSRC_MODE_LSW                (2U)
#define PMIC_PWR_RSRC_MODE_VMON               (3U)
#define PMIC_PWR_RSRC_MODE_MIN                (PMIC_PWR_RSRC_MODE_REG)
#define PMIC_PWR_RSRC_MODE_MAX                (PMIC_PWR_RSRC_MODE_VMON)
/** @} */

/**
 * @anchor Pmic_PwrSequenceDelay
 * @name PMIC Power resource startup/shutdown sequence delays
 *
 * @{
 */
#define PMIC_PWR_SEQ_DLY_0P0MS                (0x0U)
#define PMIC_PWR_SEQ_DLY_0P5MS                (0x1U)
#define PMIC_PWR_SEQ_DLY_1P0MS                (0x2U)
#define PMIC_PWR_SEQ_DLY_1P5MS                (0x3U)
#define PMIC_PWR_SEQ_DLY_2P0MS                (0x4U)
#define PMIC_PWR_SEQ_DLY_2P5MS                (0x5U)
#define PMIC_PWR_SEQ_DLY_3P0MS                (0x6U)
#define PMIC_PWR_SEQ_DLY_3P5MS                (0x7U)
#define PMIC_PWR_SEQ_DLY_4P0MS                (0x8U)
#define PMIC_PWR_SEQ_DLY_4P5MS                (0x9U)
#define PMIC_PWR_SEQ_DLY_5P0MS                (0xAU)
#define PMIC_PWR_SEQ_DLY_5P5MS                (0xBU)
#define PMIC_PWR_SEQ_DLY_6P0MS                (0xCU)
#define PMIC_PWR_SEQ_DLY_6P5MS                (0xDU)
#define PMIC_PWR_SEQ_DLY_7P0MS                (0xEU)
#define PMIC_PWR_SEQ_DLY_7P5MS                (0xFU)
#define PMIC_PWR_SEQ_DLY_MIN                  (PMIC_PWR_SEQ_DLY_0P0MS)
#define PMIC_PWR_SEQ_DLY_MAX                  (PMIC_PWR_SEQ_DLY_7P5MS)
/** @} */

/**
 * @anchor Pmic_PwrFaultReaction
 * @name PMIC Power resource fault reaction configurations
 *
 * @{
 */
#define PMIC_PWR_FAULT_REACT_INT_ONLY         (0U)
#define PMIC_PWR_FAULT_REACT_WARM_RESET       (1U)
#define PMIC_PWR_FAULT_REACT_POWER_ERROR      (2U)
#define PMIC_PWR_FAULT_REACT_REGULATOR_ERROR  (3U)
#define PMIC_PWR_FAULT_REACT_MIN              (PMIC_PWR_FAULT_REACT_INT_ONLY)
#define PMIC_PWR_FAULT_REACT_MAX              (PMIC_PWR_FAULT_REACT_REGULATOR_ERROR)
/** @} */

/**
 * @anchor Pmic_PwrBuckUvOvThresholds
 * @name PMIC Power resource undervoltage and overvoltage thresholds for Buck
 * regulators
 *
 * @{
 */
#define PMIC_PWR_BUCK_UV_OV_THR_3PERCENT      (0U)
#define PMIC_PWR_BUCK_UV_OV_THR_4PERCENT      (1U)
#define PMIC_PWR_BUCK_UV_OV_THR_5PERCENT      (2U)
#define PMIC_PWR_BUCK_UV_OV_THR_6PERCENT      (3U)
#define PMIC_PWR_BUCK_UV_OV_THR_MIN           (PMIC_PWR_UV_OV_THR_3PERCENT)
#define PMIC_PWR_BUCK_UV_OV_THR_MAX           (PMIC_PWR_UV_OV_THR_6PERCENT)
/** @} */

/**
 * @anchor Pmic_PwrLswUvOvThresholds
 * @name PMIC Power resource undervoltage and overvoltage thresholds for LDO and
 * Load switches
 *
 * @{
 */
#define PMIC_PWR_LS_UV_OV_THR_3PERCENT        (0U)
#define PMIC_PWR_LS_UV_OV_THR_4PERCENT        (1U)
#define PMIC_PWR_LS_UV_OV_THR_6PERCENT        (2U)
#define PMIC_PWR_LS_UV_OV_THR_8PERCENT        (3U)
#define PMIC_PWR_LS_UV_OV_THR_MIN             (PMIC_PWR_LS_UV_OV_THR_3PERCENT)
#define PMIC_PWR_LS_UV_OV_THR_MAX             (PMIC_PWR_LS_UV_OV_THR_8PERCENT)
/** @} */

/**
 * @anchor Pmic_PwrVccaUvOvThresholds
 * @name PMIC Power resource undervoltage and overvoltage thresholds for VCCA
 * VMON.
 *
 * @{
 */
#define PMIC_PWR_VCCA_UV_OV_THR_4PERCENT      (0U)
#define PMIC_PWR_VCCA_UV_OV_THR_5PERCENT      (1U)
#define PMIC_PWR_VCCA_UV_OV_THR_6PERCENT      (2U)
#define PMIC_PWR_VCCA_UV_OV_THR_8PERCENT      (3U)
#define PMIC_PWR_VCCA_UV_OV_THR_MIN           (PMIC_PWR_VCCA_UV_OV_THR_4PERCENT)
#define PMIC_PWR_VCCA_UV_OV_THR_MAX           (PMIC_PWR_VCCA_UV_OV_THR_8PERCENT)
/** @} */

/**
 * @anchor Pmic_PwrRvReaction
 * @name PMIC Power resource reverse voltage reaction configuration
 *
 * @{
 */
#define PMIC_PWR_RV_CONF_INT_ONLY             (0U)
#define PMIC_PWR_RV_CONF_SHUT_DOWN            (1U)
/** @} */

/**
 * @anchor Pmic_PwrIlimConfiguration
 * @name PMIC Power resource current limit configuration
 *
 * @{
 */
#define PMIC_PWR_ILIM_2P5A                     (2U)
#define PMIC_PWR_ILIM_3P0A                     (3U)
#define PMIC_PWR_ILIM_3P5A                     (4U)
#define PMIC_PWR_ILIM_4P0A                     (5U)
#define PMIC_PWR_ILIM_4P5A                     (6U)
#define PMIC_PWR_ILIM_5P0A                     (7U)
// NOTE: While 2P5A is the lowest recommended ILIM setting, the values for 0 and
// 1 are also supported but are intended for TI use only. The MIN macro is
// defined to support these, though no named macro will be provided.
#define PMIC_PWR_ILIM_MIN                      (0U)
#define PMIC_PWR_ILIM_MAX                      (PMIC_PWR_ILIM_5P0A)
/** @} */

/**
 * @anchor Pmic_PwrDeglitchSelect
 * @name PMIC Power resource deglitch filter configuration
 *
 * @{
 */
#define PMIC_PWR_DEGLITCH_0P5US                (0U)
#define PMIC_PWR_DEGLITCH_5US                  (1U)
#define PMIC_PWR_DEGLITCH_10US                 (2U)
#define PMIC_PWR_DEGLITCH_20US                 (3U)
#define PMIC_PWR_DEGLITCH_MIN                  (PMIC_PWR_DEGLITCH_0P5US)
#define PMIC_PWR_DEGLITCH_MAX                  (PMIC_PWR_DEGLITCH_20US)
/** @} */

/**
 * @anchor Pmic_PwrResourceCfgValidParamBitPos
 * @name PMIC Power resource configuration valid params bit positions.
 *
 * @{
 */
#define PMIC_PWR_CFG_ENABLE_VALID              (0U)
#define PMIC_PWR_CFG_MODE_VALID                (1U)
#define PMIC_PWR_CFG_ILIM_VALID                (2U)
#define PMIC_PWR_CFG_VOLTAGE_VALID             (3U)
#define PMIC_PWR_CFG_DEGLITCH_VALID            (4U)
#define PMIC_PWR_CFG_UV_THRESH_VALID           (5U)
#define PMIC_PWR_CFG_OV_THRESH_VALID           (6U)
#define PMIC_PWR_CFG_UV_REACT_VALID            (7U)
#define PMIC_PWR_CFG_OV_REACT_VALID            (8U)
#define PMIC_PWR_CFG_RV_REACT_VALID            (9U)
#define PMIC_PWR_CFG_SC_REACT_VALID            (10U)
/** @} */

/**
 * @anchor Pmic_PwrResourceCfgValidParamShiftVal
 * @name PMIC Power resource configuration valid params bit shift values.
 **
 * @brief Application can use these values to set the validParams structure
 * member defined in @ref Pmic_PowerResourceCfg_t structure.
 *
 * @{
 */
#define PMIC_PWR_CFG_ENABLE_VALID_SHIFT        (1U << PMIC_PWR_CFG_ENABLE_VALID)
#define PMIC_PWR_CFG_MODE_VALID_SHIFT          (1U << PMIC_PWR_CFG_MODE_VALID)
#define PMIC_PWR_CFG_ILIM_VALID_SHIFT          (1U << PMIC_PWR_CFG_ILIM_VALID)
#define PMIC_PWR_CFG_VOLTAGE_VALID_SHIFT       (1U << PMIC_PWR_CFG_VOLTAGE_VALID)
#define PMIC_PWR_CFG_DEGLITCH_VALID_SHIFT      (1U << PMIC_PWR_CFG_DEGLITCH_VALID)
#define PMIC_PWR_CFG_UV_THRESH_VALID_SHIFT     (1U << PMIC_PWR_CFG_UV_THRESH_VALID)
#define PMIC_PWR_CFG_OV_THRESH_VALID_SHIFT     (1U << PMIC_PWR_CFG_OV_THRESH_VALID)
#define PMIC_PWR_CFG_UV_REACT_VALID_SHIFT      (1U << PMIC_PWR_CFG_UV_REACT_VALID)
#define PMIC_PWR_CFG_OV_REACT_VALID_SHIFT      (1U << PMIC_PWR_CFG_OV_REACT_VALID)
#define PMIC_PWR_CFG_RV_REACT_VALID_SHIFT      (1U << PMIC_PWR_CFG_RV_REACT_VALID)
#define PMIC_PWR_CFG_SC_REACT_VALID_SHIFT      (1U << PMIC_PWR_CFG_SC_REACT_VALID)
#define PMIC_PWR_CFG_ALL_VALID_SHIFT           (\
    PMIC_PWR_CFG_ENABLE_VALID_SHIFT    |\
    PMIC_PWR_CFG_ILIM_VALID_SHIFT      |\
    PMIC_PWR_CFG_VOLTAGE_VALID_SHIFT   |\
    PMIC_PWR_CFG_DEGLITCH_VALID_SHIFT  |\
    PMIC_PWR_CFG_UV_THRESH_VALID_SHIFT |\
    PMIC_PWR_CFG_OV_THRESH_VALID_SHIFT |\
    PMIC_PWR_CFG_UV_REACT_VALID_SHIFT  |\
    PMIC_PWR_CFG_OV_REACT_VALID_SHIFT  |\
    PMIC_PWR_CFG_RV_REACT_VALID_SHIFT  |\
    PMIC_PWR_CFG_SC_REACT_VALID_SHIFT)
/** @} */

/**
 * @anchor Pmic_PwrResourceSeqValidParamBitPos
 * @name PMIC Power resource sequencing valid params bit positions.
 *
 * @{
 */
#define PMIC_PWR_SEQ_STARTUP_VALID             (0U)
#define PMIC_PWR_SEQ_SHUTDOWN_VALID            (1U)
/** @} */

/**
 * @anchor Pmic_PwrResourceSeqValidParamShiftVal
 * @name PMIC Power resource sequencing valid params bit shift values.
 **
 * @brief Application can use these values to set the validParams structure
 * member defined in @ref Pmic_PowerSequenceCfg_t structure.
 *
 * @{
 */
#define PMIC_PWR_SEQ_STARTUP_VALID_SHIFT        (1U << PMIC_PWR_SEQ_STARTUP_VALID)
#define PMIC_PWR_SEQ_SHUTDOWN_VALID_SHIFT       (1U << PMIC_PWR_SEQ_SHUTDOWN_VALID)
#define PMIC_PWR_SEQ_ALL_VALID_SHIFT            (\
    PMIC_PWR_SEQ_STARTUP_VALID_SHIFT |\
    PMIC_PWR_SEQ_SHUTDOWN_VALID_SHIFT)
/** @} */

/* ========================================================================== */
/*                            Structures and Enums                            */
/* ========================================================================== */
/**
 * @brief This structure is used in setting or getting the Power resource
 * configuration of the supported PMICs (Bucks, LDOs, etc.).
 *
 * @note `validParams` is an input parameter for all Set and Get APIs. Other
 * struct members are input params for Set APIs and output params for Get APIs.
 *
 * @param validParams Selection of structure parameters to be set from the
 * combination of the @ref Pmic_PwrResourceCfgValidParamBitPos and the
 * corresponding member value will be updated or retrieved.
 *
 * @param resource Which power resource to apply the other settings to. See @ref
 * Pmic_PwrResource.
 *
 * @param enable Control whether this power resource is enabled or disabled.
 * Note that if this bit is set to PMIC_ENABLE, then other configuration will be
 * performed prior to enabling the resource, if it is set to PMIC_DISABLE then
 * the resource will be disabled prior to performing any other configuration.
 * See @ref Pmic_EnableDisable.
 * The resources for which this parameter is valid are:
 * - PMIC_PWR_RSRC_BUCK1
 * - PMIC_PWR_RSRC_BUCK2
 * - PMIC_PWR_RSRC_BUCK3
 * - PMIC_PWR_RSRC_LDO_LS1_VMON1
 * - PMIC_PWR_RSRC_LS2_VMON2
 * - PMIC_PWR_RSRC_GPO
 * - PMIC_PWR_RSRC_VCCA_VMON
 *
 * @param mode Configure the operating mode of this power resource.
 * - Bucks1/2/3 have non-configurable modes and do not require this parameter,
 * though no error will be raised if PMIC_PWR_RSRC_MODE_REG is selected (other
 * options will raise an error).
 * - LDO_LS1_VMON1 can select from all valid modes.
 * - LS2_VMON2 can select between PMIC_PWR_RSRC_LSW and PMIC_PWR_RSRC_VMON.
 * - VCCA_VMON does not have a configurable mode and does not require this
 *   parameter, though no error will be raised if PMIC_PWR_RSRC_MODE_VMON is
 *   selected (other options will raise an error).
 * The resources for which this parameter is valid are:
 * - PMIC_PWR_RSRC_BUCK1
 * - PMIC_PWR_RSRC_BUCK2
 * - PMIC_PWR_RSRC_BUCK3
 * - PMIC_PWR_RSRC_LDO_LS1_VMON1
 * - PMIC_PWR_RSRC_LS2_VMON2
 * - PMIC_PWR_RSRC_VCCA_VMON
 *
 * @param ilim Set the peak current limit of the selected power resource. Can be
 * programmed at any time during operation. Maximum programmable current limit
 * may be limited based on device settings. Recommended to be 2A more than
 * maximum rated load current. See @ref Pmic_PwrIlimConfiguration.
 * The resources for which this parameter is valid are:
 * - PMIC_PWR_RSRC_BUCK1
 * - PMIC_PWR_RSRC_BUCK2
 * - PMIC_PWR_RSRC_BUCK3
 *
 * @param voltage_mV Set the desired voltage level of the power resource, in
 * millivolt units. This parameter has different purposes and ranges depending
 * on the power resource being configured.
 * - For Bucks1/2/3, this parameter sets the output voltage.
 * - For LDO_LS1, this parameter sets the output voltage if configured as LDO,
 *   or sets the target power-good voltage if configured as LS1.
 * - For LS2, this parameter sets the target power-good voltage.
 * - For VCCA, this parameter sets the target power-good voltage.
 * The range of Bucks1/2/3 are 900mV to 1900mV, in 20mV steps.  Attempting to
 * set voltages lower, higher, or not of 20mV multiples will result in a
 * configuration error.
 * The range of LDO_LS1_VMON1, LS2_VMON2, and VCCA_VMON is 600mV to 3400mV in
 * 25mV steps. Attempting to set voltages lower, higher, or not of 25mV
 * multiples will result in a configuration error.
 * The resources for which the parameter is valid are:
 * - PMIC_PWR_RSRC_BUCK1
 * - PMIC_PWR_RSRC_BUCK2
 * - PMIC_PWR_RSRC_BUCK3
 * - PMIC_PWR_RSRC_LDO_LS1_VMON1
 * - PMIC_PWR_RSRC_LS2_VMON2
 * - PMIC_PWR_RSRC_VCCA_VMON
 *
 * @param deglitch Set the deglitch filter for power resource monitoring. See
 * @ref Pmic_PwrDeglitchSelect.
 * The resources for which this parameter is valid are:
 * - PMIC_PWR_RSRC_BUCK1
 * - PMIC_PWR_RSRC_BUCK2
 * - PMIC_PWR_RSRC_BUCK3
 * - PMIC_PWR_RSRC_LDO_LS1_VMON1
 * - PMIC_PWR_RSRC_LS2_VMON2
 * - PMIC_PWR_RSRC_VCCA_VMON
 *
 * @param uvThresh Set the undervoltage threshold percentage. Note that
 * undervoltage thresholds are expressed as a percentage **below** the target
 * voltage and different power resources have different supported percentages.
 * - For BUCK1/2/3, see @ref Pmic_PwrBuckUvOvThresholds
 * - For LDO_LS1 and LS2, see @ref Pmic_PwrLswUvOvThresholds
 * - For VCCA VMON, see @ref Pmic_PwrVccaUvOvThresholds
 * The resources for which this parameter is valid are:
 * - PMIC_PWR_RSRC_BUCK1
 * - PMIC_PWR_RSRC_BUCK2
 * - PMIC_PWR_RSRC_BUCK3
 * - PMIC_PWR_RSRC_LDO_LS1_VMON1
 * - PMIC_PWR_RSRC_LS2_VMON2
 * - PMIC_PWR_RSRC_VCCA_VMON
 *
 * @param uvReaction Set the device reaction to detection of an undervoltage
 * event on this power resource. See @ref Pmic_PwrFaultReaction.
 * The resources for which this parameter is valid are:
 * - PMIC_PWR_RSRC_BUCK1
 * - PMIC_PWR_RSRC_BUCK2
 * - PMIC_PWR_RSRC_BUCK3
 * - PMIC_PWR_RSRC_LDO_LS1_VMON1
 * - PMIC_PWR_RSRC_LS2_VMON2
 * - PMIC_PWR_RSRC_VCCA_VMON
 *
 * @param ovThresh Se the overvoltage threshold percente. Note that overvoltage
 * thresholds are expressed as a percentage **above** the target voltage and
 * different power resources have different supported percentages.
 * - For BUCK1/2/3, see @ref Pmic_PwrBuckUvOvThresholds
 * - For LDO_LS1 and LS2, see @ref Pmic_PwrLswUvOvThresholds
 * - For VCCA VMON, see @ref Pmic_PwrVccaUvOvThresholds
 * The resources for which this parameter is valid are:
 * - PMIC_PWR_RSRC_BUCK1
 * - PMIC_PWR_RSRC_BUCK2
 * - PMIC_PWR_RSRC_BUCK3
 * - PMIC_PWR_RSRC_LDO_LS1_VMON1
 * - PMIC_PWR_RSRC_LS2_VMON2
 * - PMIC_PWR_RSRC_VCCA_VMON
 *
 * @param ovReaction Set the device reaction to detection of an overvoltage
 * event on this power resource. See @ref Pmic_PwrFaultReaction.
 * The resources for which this parameter is valid are:
 * - PMIC_PWR_RSRC_BUCK1
 * - PMIC_PWR_RSRC_BUCK2
 * - PMIC_PWR_RSRC_BUCK3
 * - PMIC_PWR_RSRC_LDO_LS1_VMON1
 * - PMIC_PWR_RSRC_LS2_VMON2
 * - PMIC_PWR_RSRC_VCCA_VMON
 *
 * @param rvReaction Set the device reaction to detection of a reverse voltage
 * event on this power resource. See @ref Pmic_PwrRvReaction. NOTE: This uses a
 * different "reaction" enum than the other reaction struct members.
 * The resources for which this parameter is valid are:
 * - PMIC_PWR_RSRC_BUCK1
 * - PMIC_PWR_RSRC_BUCK2
 * - PMIC_PWR_RSRC_BUCK3
 * - PMIC_PWR_RSRC_LDO_LS1_VMON1
 * - PMIC_PWR_RSRC_LS2_VMON2
 *
 * @param scReaction Set the device reaction to detection of a short circuit
 * event on this power resource. See @ref Pmic_PwrFaultReaction.
 * The resources for which this parameter is valid are:
 * - PMIC_PWR_RSRC_BUCK1
 * - PMIC_PWR_RSRC_BUCK2
 * - PMIC_PWR_RSRC_BUCK3
 * - PMIC_PWR_RSRC_LDO_LS1_VMON1
 * - PMIC_PWR_RSRC_LS2_VMON2
 */
typedef struct Pmic_PowerResourceCfg_s {
    uint16_t validParams;
    uint8_t resource;

    bool enable;
    uint8_t mode;
    uint8_t ilim;
    uint16_t voltage_mV;
    uint8_t deglitch;
    uint8_t uvThresh;
    uint8_t uvReaction;
    uint8_t ovThresh;
    uint8_t ovReaction;
    uint8_t rvReaction;
    uint8_t scReaction;
} Pmic_PowerResourceCfg_t;

/**
 * @brief This structure is used in setting or getting the Power resource
 * sequencing information of the supported PMICs (Bucks, LDOs, etc.).
 *
 * @param resource Which power resource to apply sequencing settings to. See
 * @ref Pmic_PwrResource.
 *
 * @param startupDelay Set the delay between rising edge of ENABLE and
 * enablement of this power resource.

 * @param shutdownDelay Set the delay between falling edge of ENABLE and
 * disablement of this power resource.
 */
typedef struct Pmic_PowerSequenceCfg_s {
    uint8_t validParams;
    uint8_t resource;

    uint8_t startupDelay;
    uint8_t shutdownDelay;
} Pmic_PowerSequenceCfg_t;

/* ========================================================================== */
/*                            Function Declarations                           */
/* ========================================================================== */
/**
 * @ingroup DRV_PMIC_PWR_MODULE
 * @brief API to control the enable state of PMIC power resources.
 *
 * @param handle   [IN] PMIC Interface Handle
 * @param resource [IN] Power resource to control, see @ref Pmic_PwrResource.
 * @param enable   [IN] If set to true (PMIC_ENABLE) the power resource will be
 * enabled, otherwise the resource will be disabled. See @ref
 * Pmic_EnableDisable.
 *
 * @return PMIC_ST_SUCCESS in case of success or appropriate error code. For
 * possible values, see @ref Pmic_ErrorCodes.
 */
int32_t Pmic_pwrSetResourceEnable(Pmic_CoreHandle_t *handle, uint8_t resource, bool enable);

/**
 * @ingroup DRV_PMIC_PWR_MODULE
 * @brief API to control the enable state of PMIC power resources.
 *
 * @param handle   [IN]  PMIC Interface Handle
 * @param resource [IN]  Power resource to get status of, see @ref Pmic_PwrResource.
 * @param enable   [OUT] If set to true (PMIC_ENABLE) the power resource is *
 * enabled, otherwise the resource is disabled. See @ref Pmic_EnableDisable.
 *
 * @return PMIC_ST_SUCCESS in case of success or appropriate error code. For
 * possible values, see @ref Pmic_ErrorCodes.
 */
int32_t Pmic_pwrGetResourceEnable(Pmic_CoreHandle_t *handle, uint8_t resource, bool *isEnabled);

/**
 * @ingroup DRV_PMIC_PWR_MODULE
 * @brief API to set the configuration of a PMIC power resources.
 *
 * @param handle [IN] PMIC Interface Handle
 * @param config [IN] Configuration options for this power resource. See @ref
 * Pmic_PowerResourceCfg_t.
 *
 * @return PMIC_ST_SUCCESS in case of success or appropriate error code. For
 * possible values, see @ref Pmic_ErrorCodes.
 */
int32_t Pmic_pwrSetResourceCfg(Pmic_CoreHandle_t *handle, const Pmic_PowerResourceCfg_t *config);

/**
 * @ingroup DRV_PMIC_PWR_MODULE
 * @brief API to get the configuration of a PMIC power resources.
 *
 * @param handle [IN]     PMIC Interface Handle
 * @param config [IN/OUT] Configuration options for this power resource. See @ref
 * Pmic_PowerResourceCfg_t.
 *
 * @return PMIC_ST_SUCCESS in case of success or appropriate error code. For
 * possible values, see @ref Pmic_ErrorCodes.
 */
int32_t Pmic_pwrGetResourceCfg(Pmic_CoreHandle_t *handle, Pmic_PowerResourceCfg_t *config);

/**
 * @ingroup DRV_PMIC_PWR_MODULE
 * @brief API to set the configuration of multiple PMIC power resources.
 *
 * @param handle     [IN] PMIC Interface Handle
 * @param numConfigs [IN] The number of configurations in the `config` array.
 * @param config     [IN] An array of configuration options for power resources.
 * See @ref Pmic_PowerResourceCfg_t.
 *
 * @return PMIC_ST_SUCCESS in case of success or appropriate error code. For
 * possible values, see @ref Pmic_ErrorCodes.
 */
int32_t Pmic_pwrSetResourceCfgs(Pmic_CoreHandle_t *handle, uint8_t numConfigs, const Pmic_PowerResourceCfg_t config[]);

/**
 * @ingroup DRV_PMIC_PWR_MODULE
 * @brief API to get the configuration of multiple PMIC power resources.
 *
 * @param handle     [IN]     PMIC Interface Handle

 * @param numConfigs [IN] The number of configurations to retrieve. NOTE the
 * `config` array provided must have enough storage for all configurations
 * requested.

 * @param config     [IN/OUT] Configuration options for each power resource
 * requested. See @ref Pmic_PowerResourceCfg_t.
 *
 * @return PMIC_ST_SUCCESS in case of success or appropriate error code. For
 * possible values, see @ref Pmic_ErrorCodes.
 */
int32_t Pmic_pwrGetResourceCfgs(Pmic_CoreHandle_t *handle, uint8_t numConfigs, Pmic_PowerResourceCfg_t config[]);

/**
 * @ingroup DRV_PMIC_PWR_MODULE
 * @brief API to set the power sequencing configuration of a PMIC power
 * resource.
 *
 * @param handle [IN] PMIC Interface Handle
 * @param config [IN] Sequencing configuration options for this power
 * resource. See @ref Pmic_PowerSequenceCfg_t.
 *
 * @return PMIC_ST_SUCCESS in case of success or appropriate error code. For
 * possible values, see @ref Pmic_ErrorCodes.
 */
int32_t Pmic_pwrSetSequenceCfg(Pmic_CoreHandle_t *handle, const Pmic_PowerSequenceCfg_t *config);

/**
 * @ingroup DRV_PMIC_PWR_MODULE
 * @brief API to get the power sequencing configuration of a PMIC power
 * resource.
 *
 * @param handle [IN]     PMIC Interface Handle
 * @param config [IN/OUT] Sequencing configuration for this power resource. See
 * @ref Pmic_PowerSequenceCfg_t.
 *
 * @return PMIC_ST_SUCCESS in case of success or appropriate error code. For
 * possible values, see @ref Pmic_ErrorCodes.
 */
int32_t Pmic_pwrGetSequenceCfg(Pmic_CoreHandle_t *handle, Pmic_PowerSequenceCfg_t *config);

/**
 * @ingroup DRV_PMIC_PWR_MODULE
 * @brief API to set the power sequencing configuration of multiple PMIC power
 * resources.
 *
 * @param handle     [IN] PMIC Interface Handle
 * @param numConfigs [IN] The number of configurations in the `config` array.
 * @param config     [IN] An array of configuration options for power resources.
 * See @ref Pmic_PowerSequenceCfg_t.
 *
 * @return PMIC_ST_SUCCESS in case of success or appropriate error code. For
 * possible values, see @ref Pmic_ErrorCodes.
 */
int32_t Pmic_pwrSetSequenceCfgs(Pmic_CoreHandle_t *handle, uint8_t numConfigs, const Pmic_PowerSequenceCfg_t config[]);

/**
 * @ingroup DRV_PMIC_PWR_MODULE
 * @brief API to get the power sequencing configuration of multiple PMIC power
 * resources.
 *
 * @param handle     [IN] PMIC Interface Handle
 * @param numConfigs [IN] The number of configurations in the `config` array.
 * @param config     [IN/OUT] Configuration options for each power resource
 * requested. See @ref Pmic_PowerSequenceCfg_t.
 *
 * @return PMIC_ST_SUCCESS in case of success or appropriate error code. For
 * possible values, see @ref Pmic_ErrorCodes.
 */
int32_t Pmic_pwrGetSequenceCfgs(Pmic_CoreHandle_t *handle, uint8_t numConfigs, Pmic_PowerSequenceCfg_t config[]);

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif // PMIC_POWER_H
