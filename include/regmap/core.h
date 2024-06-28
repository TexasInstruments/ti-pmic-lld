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
 * @file core.h
 *
 * @brief PMIC LLD register addresses and bit fields pertaining to the Core
 * module.
 */
#ifndef __CORE_H__
#define __CORE_H__

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                Include Files                               */
/* ========================================================================== */
#include "pmic_common.h"

/* ========================================================================== */
/*                     TPS65036x Core Module Register Map                     */
/* ========================================================================== */

#define PMIC_DEV_REV_REGADDR                ((uint8_t)0x01U)
#define PMIC_NVM_CODE_1_REGADDR             ((uint8_t)0x02U)
#define PMIC_NVM_CODE_2_REGADDR             ((uint8_t)0x03U)
#define PMIC_MANUFACTURING_VER_REGADDR      ((uint8_t)0x04U)
#define PMIC_FSM_COMMAND_REG_REGADDR        ((uint8_t)0x05U)
#define PMIC_ABIST_RUN_CMD_REGADDR          ((uint8_t)0x06U)
#define PMIC_RECOV_CNT_CONTROL_REGADDR      ((uint8_t)0x07U)
#define PMIC_REGISTER_LOCK_REGADDR          ((uint8_t)0x09U)
#define PMIC_SCRATCH_PAD_REG_1_REGADDR      ((uint8_t)0x0AU)
#define PMIC_SCRATCH_PAD_REG_2_REGADDR      ((uint8_t)0x0BU)
#define PMIC_SCRATCH_PAD_REG_3_REGADDR      ((uint8_t)0x0CU)
#define PMIC_SCRATCH_PAD_REG_4_REGADDR      ((uint8_t)0x0DU)
#define PMIC_LOW_PWR_CONFIG_REGADDR         ((uint8_t)0x17U)
#define PMIC_CLK_CONF_REGADDR               ((uint8_t)0x22U)
#define PMIC_LPM_CONF_REGADDR               ((uint8_t)0x23U)
#define PMIC_INTERFACE_CONF_REGADDR         ((uint8_t)0x24U)
#define PMIC_FUNC_CONF_REGADDR              ((uint8_t)0x25U)
#define PMIC_CONFIG_1_REGADDR               ((uint8_t)0x3CU)
#define PMIC_STAT_STARTUP_REGADDR           ((uint8_t)0x58U)
#define PMIC_STAT_MISC_REGADDR              ((uint8_t)0x59U)
#define PMIC_STAT_SEVERE_ERR_REGADDR        ((uint8_t)0x5BU)
#define PMIC_CONFIG_CRC_CONFIG_REGADDR      ((uint8_t)0x61U)

/* ========================================================================== */
/*                               Macros & Typedefs                            */
/* ========================================================================== */

/**
 * @anchor Pmic_regLockStatBitField
 * @name TPS65036x Register Lock Status Bit Field
 *
 * @brief Bit mask and shift values to access the REGISTER_LOCK_STATUS
 * bit field of the REGISTER_LOCK register.
 *
 * @{
 */
#define PMIC_REGISTER_LOCK_STATUS_SHIFT            ((uint8_t)0U)
#define PMIC_REGISTER_LOCK_STATUS_MASK             ((uint8_t)1U << PMIC_REGISTER_LOCK_STATUS_SHIFT)
/** @} */

/**
 * @anchor Pmic_regLockKeys
 * @name TPS65036x Register Lock Keys
 *
 * @brief Values used to unlock and lock TPS65036x registers. The values
 * are to be written to REGISTER_LOCK.
 *
 * @{
 */
#define PMIC_REG_UNLOCK                     ((uint8_t)0x9BU)
#define PMIC_REG_LOCK                       ((uint8_t)0xAAU)
/** @} */

/**
 * @anchor Pmic_interfaceConfRegBitFields
 * @name TPS65036x INTERFACE_CONF Register Bit Fields
 *
 * @brief Register bit shifts and masks of the INTERFACE_CONF register.
 *
 * @{
 */
#define PMIC_I2C_CRC_EN_SHIFT               ((uint8_t)7U)
#define PMIC_I2C_CRC_EN_MASK                ((uint8_t)1U << PMIC_I2C_CRC_EN_SHIFT)
#define PMIC_NRSTOUT_OD_SHIFT               ((uint8_t)5U)
#define PMIC_NRSTOUT_OD_MASK                ((uint8_t)1U << PMIC_NRSTOUT_OD_SHIFT)
#define PMIC_NRSTOUT_POL_SHIFT              ((uint8_t)4U)
#define PMIC_NRSTOUT_POL_MASK               ((uint8_t)1U << PMIC_NRSTOUT_POL_SHIFT)
#define PMIC_GPIO_POL_SHIFT                 ((uint8_t)3U)
#define PMIC_GPIO_POL_MASK                  ((uint8_t)1U << PMIC_GPIO_POL_SHIFT)
#define PMIC_GPO_EN_SHIFT                   ((uint8_t)2U)
#define PMIC_GPO_EN_MASK                    ((uint8_t)1U << PMIC_GPO_EN_SHIFT)
#define PMIC_GPIO_SEL_SHIFT                 ((uint8_t)0U)
#define PMIC_GPIO_SEL_MASK                  ((uint8_t)3U << PMIC_GPIO_SEL_SHIFT)
/** @} */

/**
 * @anchor Pmic_recovCntControlBitFields
 * @name TPS65036x RECOV_CNT_CONTROL Register Bit Fields
 *
 * @brief Mask and shift values for all bit fields of the RECOV_CNT_CONTROL
 * register.
 *
 * @{
 */
#define PMIC_WD_TRIGGER_SHIFT               ((uint8_t)7U)
#define PMIC_WD_TRIGGER_MASK                ((uint8_t)1U << PMIC_WD_TRIGGER_SHIFT)
#define PMIC_RESET_CNT_CLR_SHIFT            ((uint8_t)1U)
#define PMIC_RESET_CNT_CLR_MASK             ((uint8_t)1U << PMIC_RESET_CNT_CLR_SHIFT)
#define PMIC_RECOV_CNT_CLR_SHIFT            ((uint8_t)0U)
#define PMIC_RECOV_CNT_CLR_MASK             ((uint8_t)1U << PMIC_RECOV_CNT_CLR_SHIFT)
/** @} */

/**
 * @anchor Pmic_funcConfBitFields
 * @name TPS65036x FUNC_CONF Register Bit Fields
 *
 * @brief Mask and shift values for all bit fields of the FUNC_CONF register.
 *
 * @{
 */
#define PMIC_PWR_ON_SHIFT                   ((uint8_t)7U)
#define PMIC_PWR_ON_MASK                    ((uint8_t)1U << PMIC_PWR_ON_SHIFT)
#define PMIC_SEQ_ENABLE_POL_SHIFT           ((uint8_t)5U)
#define PMIC_SEQ_ENABLE_POL_MASK            ((uint8_t)1U << PMIC_SEQ_ENABLE_POL_SHIFT)
#define PMIC_NINT_GPI_PU_PD_SEL_SHIFT       ((uint8_t)4U)
#define PMIC_NINT_GPI_PU_PD_SEL_MASK        ((uint8_t)1U << PMIC_NINT_GPI_PU_PD_SEL_SHIFT)
#define PMIC_NINT_PP_OD_SHIFT               ((uint8_t)3U)
#define PMIC_NINT_PP_OD_MASK                ((uint8_t)1U << PMIC_NINT_PP_OD_SHIFT)
#define PMIC_NINT_GPI_POL_SHIFT             ((uint8_t)2U)
#define PMIC_NINT_GPI_POL_MASK              ((uint8_t)1U << PMIC_NINT_GPI_POL_SHIFT)
#define PMIC_NINT_GPI_SEL_SHIFT             ((uint8_t)0U)
#define PMIC_NINT_GPI_SEL_MASK              ((uint8_t)3U << PMIC_NINT_GPI_SEL_SHIFT)
/** @} */

/**
 * @anchor Pmic_ConfigCrcConfigBitFields
 * @name TPS65036x CONFIG_CRC_CONFIG Register Bit Fields
 *
 * @brief Mask and shift values for all bit fields of the CONFIG_CRC_CONFIG register.
 *
 * @{
 */
#define PMIC_CONFIG_CRC_STATUS_SHIFT        ((uint8_t)2U)
#define PMIC_CONFIG_CRC_STATUS_MASK         ((uint8_t)1U << PMIC_CONFIG_CRC_STATUS_SHIFT)
#define PMIC_CONFIG_CRC_CALC_SHIFT          ((uint8_t)1U)
#define PMIC_CONFIG_CRC_CALC_MASK           ((uint8_t)1U << PMIC_CONFIG_CRC_CALC_SHIFT)
#define PMIC_CONFIG_CRC_EN_SHIFT            ((uint8_t)0U)
#define PMIC_CONFIG_CRC_EN_MASK             ((uint8_t)1U << PMIC_CONFIG_CRC_EN_SHIFT)
/** @} */

/**
 * @anchor Pmic_lowPwrConfigBitFields
 * @name TPS65036x LOW_PWR_CONFIG Register Bit Fields
 *
 * @brief Mask and shift values for all bit fields of the LOW_PWR_CONFIG register.
 *
 * @{
 */
#define PMIC_LOWPWR_SEL_SHIFT               ((uint8_t)2U)
#define PMIC_LOWPWR_SEL_MASK                ((uint8_t)1U << PMIC_LOWPWR_SEL_SHIFT)
#define PMIC_LOWPWR_DELAY_SHIFT             ((uint8_t)0U)
#define PMIC_LOWPWR_DELAY_MASK              ((uint8_t)3U << PMIC_LOWPWR_DELAY_SHIFT)
/** @} */

/**
 * @anchor Pmic_lpmConfBitFields
 * @name TPS65036x LPM_CONF Register Bit Fields
 *
 * @brief Mask and shift values for all bit fields of the LPM_CONF register.
 *
 * @{
 */
#define PMIC_LOWPWR_VMON_EN_SHIFT           ((uint8_t)3U)
#define PMIC_LOWPWR_VMON_EN_MASK            ((uint8_t)1U << PMIC_LOWPWR_VMON_EN_SHIFT)
#define PMIC_LOWPWR_ESM_EN_SHIFT            ((uint8_t)2U)
#define PMIC_LOWPWR_ESM_EN_MASK             ((uint8_t)1U << PMIC_LOWPWR_ESM_EN_SHIFT)
#define PMIC_LOWPWR_WD_EN_SHIFT             ((uint8_t)1U)
#define PMIC_LOWPWR_WD_EN_MASK              ((uint8_t)1U << PMIC_LOWPWR_WD_EN_SHIFT)
/** @} */

/**
 * @anchor Pmic_statMiscBitFields
 * @name TPS65036x STAT_MISC Register Bit Fields
 *
 * @brief Mask and shift values for all bit fields of the STAT_MISC register.
 *
 * @{
 */
#define PMIC_TWARN_STAT_SHIFT               ((uint8_t)7U)
#define PMIC_TWARN_STAT_MASK                ((uint8_t)1U << PMIC_TWARN_STAT_SHIFT)
#define PMIC_B1_PVIN_UVLO_STAT_SHIFT        ((uint8_t)6U)
#define PMIC_B1_PVIN_UVLO_STAT_MASK         ((uint8_t)1U << PMIC_B1_PVIN_UVLO_STAT_SHIFT)
#define PMIC_BUCKS_VSET_ERR_STAT_SHIFT      ((uint8_t)5U)
#define PMIC_BUCKS_VSET_ERR_STAT_MASK       ((uint8_t)1U << PMIC_BUCKS_VSET_ERR_STAT_SHIFT)
#define PMIC_ABIST_FAIL_STAT_SHIFT          ((uint8_t)2U)
#define PMIC_ABIST_FAIL_STAT_MASK           ((uint8_t)1U << PMIC_ABIST_FAIL_STAT_SHIFT)
#define PMIC_ABIST_ACTIVE_STAT_SHIFT        ((uint8_t)1U)
#define PMIC_ABIST_ACTIVE_STAT_MASK         ((uint8_t)1U << PMIC_ABIST_ACTIVE_STAT_SHIFT)
/** @} */

/**
 * @anchor Pmic_clkConfBitFields
 * @name TPS65036x CLK_CONF Register Bit Fields
 *
 * @brief Mask and shift values for all bit fields of the CLK_CONF register.
 *
 * @{
 */
#define PMIC_SS_EN_SHIFT                    ((uint8_t)7U)
#define PMIC_SS_EN_MASK                     ((uint8_t)1U << PMIC_SS_EN_SHIFT)
#define PMIC_SSM_SEL_SHIFT                  ((uint8_t)6U)
#define PMIC_SSM_SEL_MASK                   ((uint8_t)1U << PMIC_SSM_SEL_SHIFT)
#define PMIC_FORCE_INTERRUPT_SHIFT          ((uint8_t)0U)
#define PMIC_FORCE_INTERRUPT_MASK           ((uint8_t)1U << PMIC_FORCE_INTERRUPT_SHIFT)
/** @} */

/**
 * @anchor Pmic_statStartupBitFields
 * @name TPS65036x STAT_STARTUP Register Bit Fields
 *
 * @brief Mask and shift values for all bit fields of the STAT_STARTUP register.
 *
 * @{
 */
#define PMIC_BUCK3_ACTIVE_SHIFT             ((uint8_t)7U)
#define PMIC_BUCK3_ACTIVE_MASK              ((uint8_t)1U << PMIC_BUCK3_ACTIVE_SHIFT)
#define PMIC_BUCK2_ACTIVE_SHIFT             ((uint8_t)6U)
#define PMIC_BUCK2_ACTIVE_MASK              ((uint8_t)1U << PMIC_BUCK2_ACTIVE_SHIFT)
#define PMIC_BUCK1_ACTIVE_SHIFT             ((uint8_t)5U)
#define PMIC_BUCK1_ACTIVE_MASK              ((uint8_t)1U << PMIC_BUCK1_ACTIVE_SHIFT)
#define PMIC_LDO_ACTIVE_SHIFT               ((uint8_t)3U)
#define PMIC_LDO_ACTIVE_MASK                ((uint8_t)1U << PMIC_LDO_ACTIVE_SHIFT)
/** @} */

/**
 * @anchor Pmic_config1BitFields
 * @name TPS65036x CONFIG_1 Register Bit Fields
 *
 * @brief Mask and shift values for all bit fields of the CONFIG_1 register.
 *
 * @{
 */
#define PMIC_TWARN_CONFIG_SHIFT             ((uint8_t)2U)
#define PMIC_TWARN_CONFIG_MASK              ((uint8_t)1U << PMIC_TWARN_CONFIG_SHIFT)
#define PMIC_TSD_IMM_LEVEL_SHIFT            ((uint8_t)1U)
#define PMIC_TSD_IMM_LEVEL_MASK             ((uint8_t)1U << PMIC_TSD_IMM_LEVEL_SHIFT)
#define PMIC_TWARN_LEVEL_SHIFT              ((uint8_t)0U)
#define PMIC_TWARN_LEVEL_MASK               ((uint8_t)1U << PMIC_TWARN_LEVEL_SHIFT)
/** @} */

/**
 * @anchor Pmic_statSevereErrBitFields
 * @name TPS65036x STAT_SEVERE_ERR Register Bit Fields
 *
 * @brief Mask and shift values for all bit fields of the STAT_SEVERE_ERR register.
 *
 * @{
 */
#define PMIC_TSD_IMM_STAT_SHIFT             ((uint8_t)0U)
#define PMIC_TSD_IMM_STAT_MASK              ((uint8_t)1U << PMIC_TSD_IMM_STAT_SHIFT)
/** @} */

/**
 * @anchor Pmic_runAbistCommand
 * @name TPS65036x Run ABIST Command
 *
 * @brief Used by PMIC LLD to trigger PMIC runtime ABIST.
 *
 * @{
 */
#define PMIC_RUN_ABIST_COMMAND              ((uint8_t)0xDAU)
/** @} */

#ifdef __cplusplus
}

#endif /* __cplusplus */
#endif /* __CORE_H__ */
