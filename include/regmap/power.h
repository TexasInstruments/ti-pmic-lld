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
 * @file power.h
 *
 * @brief PMIC LLD register addresses and bit fields pertaining to the Power
 * module
 */
#ifndef __POWER_H__
#define __POWER_H__

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                Include Files                               */
/* ========================================================================== */
#include "pmic_common.h"

/* ========================================================================== */
/*                               Macros & Typedefs                            */
/* ========================================================================== */

/**
 * @anchor Pmic_buck1VoutBitFields
 * @name TPS65036x BUCK1_VOUT Register Bit Fields
 *
 * @brief Mask and shift values for all bit fields of the BUCK1_VOUT register
 *
 * @{
 */
#define PMIC_BUCK1_VSET_SHIFT                   ((uint8_t)0U)
#define PMIC_BUCK1_VSET_MASK                    ((uint8_t)127U << PMIC_BUCK1_VSET_SHIFT)
/** @} */

/**
 * @anchor Pmic_buck2VoutActiveBitFields
 * @name TPS65036x BUCK2_VOUT_ACTIVE Register Bit Fields
 *
 * @brief Mask and shift values for all bit fields of the BUCK2_VOUT_ACTIVE register
 *
 * @{
 */
#define PMIC_BUCK2_VSET_ACT_SHIFT               ((uint8_t)0U)
#define PMIC_BUCK2_VSET_ACT_MASK                ((uint8_t)127U << PMIC_BUCK2_VSET_ACT_SHIFT)
/** @} */

/**
 * @anchor Pmic_buck3VoutActiveBitFields
 * @name TPS65036x BUCK3_VOUT_ACTIVE Register Bit Fields
 *
 * @brief Mask and shift values for all bit fields of the BUCK3_VOUT_ACTIVE register
 *
 * @{
 */
#define PMIC_BUCK3_VSET_ACT_SHIFT               ((uint8_t)0U)
#define PMIC_BUCK3_VSET_ACT_MASK                ((uint8_t)127U << PMIC_BUCK3_VSET_ACT_SHIFT)
/** @} */

/**
 * @anchor Pmic_buck2VoutLowpwrBitFields
 * @name TPS65036x BUCK2_VOUT_LOWPWR Register Bit Fields
 *
 * @brief Mask and shift values for all bit fields of the BUCK2_VOUT_LOWPWR register
 *
 * @{
 */
#define PMIC_BUCK2_VSET_LPWR_SHIFT              ((uint8_t)0U)
#define PMIC_BUCK2_VSET_LPWR_MASK               ((uint8_t)127U << PMIC_BUCK2_VSET_LPWR_SHIFT)
/** @} */

/**
 * @anchor Pmic_buck3VoutLowpwrBitFields
 * @name TPS65036x BUCK3_VOUT_LOWPWR Register Bit Fields
 *
 * @brief Mask and shift values for all bit fields of the BUCK3_VOUT_LOWPWR register
 *
 * @{
 */
#define PMIC_BUCK3_VSET_LPWR_SHIFT              ((uint8_t)0U)
#define PMIC_BUCK3_VSET_LPWR_MASK               ((uint8_t)127U << PMIC_BUCK3_VSET_LPWR_SHIFT)
/** @} */

/**
 * @anchor Pmic_buck1CtrlBitFields
 * @name TPS65036x BUCK1_CTRL Register Bit Fields
 *
 * @brief Mask and shift values for all bit fields of the BUCK1_CTRL register
 *
 * @{
 */
#define PMIC_BUCK1_EN_HS_ON_SR_SHIFT            ((uint8_t)6U)
#define PMIC_BUCK1_EN_HS_ON_SR_MASK             ((uint8_t)3U << PMIC_BUCK1_EN_HS_ON_SR_SHIFT)
#define PMIC_BUCK1_DISCHARGE_SEL_SHIFT          ((uint8_t)5U)
#define PMIC_BUCK1_DISCHARGE_SEL_MASK           ((uint8_t)1U << PMIC_BUCK1_DISCHARGE_SEL_SHIFT)
#define PMIC_BUCK1_PLDN_SHIFT                   ((uint8_t)4U)
#define PMIC_BUCK1_PLDN_MASK                    ((uint8_t)1U << PMIC_BUCK1_PLDN_SHIFT)
#define PMIC_BUCK1_SLEW_RATE_SHIFT              ((uint8_t)2U)
#define PMIC_BUCK1_SLEW_RATE_MASK               ((uint8_t)3U << PMIC_BUCK1_SLEW_RATE_SHIFT)
#define PMIC_BUCK1_FPWM_SHIFT                   ((uint8_t)1U)
#define PMIC_BUCK1_FPWM_MASK                    ((uint8_t)1U << PMIC_BUCK1_FPWM_SHIFT)
#define PMIC_BUCK1_EN_SHIFT                     ((uint8_t)0U)
#define PMIC_BUCK1_EN_MASK                      ((uint8_t)1U << PMIC_BUCK1_EN_SHIFT)
/** @} */

/**
 * @anchor Pmic_buck2CtrlBitFields
 * @name TPS65036x BUCK2_CTRL Register Bit Fields
 *
 * @brief Mask and shift values for all bit fields of the BUCK2_CTRL register
 *
 * @{
 */
#define PMIC_BUCK2_VMON_ONLY_SHIFT              ((uint8_t)6U)
#define PMIC_BUCK2_VMON_ONLY_MASK               ((uint8_t)1U << PMIC_BUCK2_VMON_ONLY_SHIFT)
#define PMIC_BUCK2_DISCHARGE_SEL_SHIFT          ((uint8_t)5U)
#define PMIC_BUCK2_DISCHARGE_SEL_MASK           ((uint8_t)1U << PMIC_BUCK2_DISCHARGE_SEL_SHIFT)
#define PMIC_BUCK2_PLDN_SHIFT                   ((uint8_t)4U)
#define PMIC_BUCK2_PLDN_MASK                    ((uint8_t)1U << PMIC_BUCK2_PLDN_SHIFT)
#define PMIC_BUCK2_SLEW_RATE_SHIFT              ((uint8_t)2U)
#define PMIC_BUCK2_SLEW_RATE_MASK               ((uint8_t)3U << PMIC_BUCK2_SLEW_RATE_SHIFT)
#define PMIC_BUCK2_FPWM_SHIFT                   ((uint8_t)1U)
#define PMIC_BUCK2_FPWM_MASK                    ((uint8_t)1U << PMIC_BUCK2_FPWM_SHIFT)
#define PMIC_BUCK2_EN_SHIFT                     ((uint8_t)0U)
#define PMIC_BUCK2_EN_MASK                      ((uint8_t)1U << PMIC_BUCK2_EN_SHIFT)
/** @} */

/**
 * @anchor Pmic_buck3CtrlBitFields
 * @name TPS65036x BUCK3_CTRL Register Bit Fields
 *
 * @brief Mask and shift values for all bit fields of the BUCK3_CTRL register
 *
 * @{
 */
#define PMIC_BUCK3_VMON_ONLY_SHIFT              ((uint8_t)6U)
#define PMIC_BUCK3_VMON_ONLY_MASK               ((uint8_t)1U << PMIC_BUCK3_VMON_ONLY_SHIFT)
#define PMIC_BUCK3_DISCHARGE_SEL_SHIFT          ((uint8_t)5U)
#define PMIC_BUCK3_DISCHARGE_SEL_MASK           ((uint8_t)1U << PMIC_BUCK3_DISCHARGE_SEL_SHIFT)
#define PMIC_BUCK3_PLDN_SHIFT                   ((uint8_t)4U)
#define PMIC_BUCK3_PLDN_MASK                    ((uint8_t)1U << PMIC_BUCK3_PLDN_SHIFT)
#define PMIC_BUCK3_SLEW_RATE_SHIFT              ((uint8_t)2U)
#define PMIC_BUCK3_SLEW_RATE_MASK               ((uint8_t)3U << PMIC_BUCK3_SLEW_RATE_SHIFT)
#define PMIC_BUCK3_FPWM_SHIFT                   ((uint8_t)1U)
#define PMIC_BUCK3_FPWM_MASK                    ((uint8_t)1U << PMIC_BUCK3_FPWM_SHIFT)
#define PMIC_BUCK3_EN_SHIFT                     ((uint8_t)0U)
#define PMIC_BUCK3_EN_MASK                      ((uint8_t)1U << PMIC_BUCK3_EN_SHIFT)
/** @} */

/**
 * @anchor Pmic_buck1UVLOBitFields
 * @name TPS65036x BUCK1_UVLO Register Bit Fields
 *
 * @brief Mask and shift values for all bit fields of the BUCK1_UVLO register
 *
 * @{
 */
#define PMIC_BUCK1_UVLO_FALLING_SHIFT           ((uint8_t)4U)
#define PMIC_BUCK1_UVLO_FALLING_MASK            ((uint8_t)0x0FU << PMIC_BUCK1_UVLO_FALLING_SHIFT)
#define PMIC_BUCK1_UVLO_RISING_SHIFT            ((uint8_t)0U)
#define PMIC_BUCK1_UVLO_RISING_MASK             ((uint8_t)0x0FU << PMIC_BUCK1_UVLO_RISING_SHIFT)
/** @} */

/**
 * @anchor Pmic_ldoConfBitFields
 * @name TPS65036x LDO_CONF Register Bit Fields
 *
 * @brief Mask and shift values for all bit fields of the LDO_CONF register
 *
 * @{
 */
#define PMIC_LDO_BYP_CONFIG_SHIFT               ((uint8_t)7U)
#define PMIC_LDO_BYP_CONFIG_MASK                ((uint8_t)1U << PMIC_LDO_BYP_CONFIG_SHIFT)
#define PMIC_LDO_VSET_SHIFT                     ((uint8_t)0U)
#define PMIC_LDO_VSET_MASK                      ((uint8_t)127U << PMIC_LDO_VSET_SHIFT)
/** @} */

/**
 * @anchor Pmic_ldoCtrlBitFields
 * @name TPS65036x LDO_CTRL Register Bit Fields
 *
 * @brief Mask and shift values for all bit fields of the LDO_CTRL register
 *
 * @{
 */
#define PMIC_LDO_VMON_ONLY_SHIFT                ((uint8_t)7U)
#define PMIC_LDO_VMON_ONLY_MASK                 ((uint8_t)1U << PMIC_LDO_VMON_ONLY_SHIFT)
#define PMIC_LDO_DISCHARGE_SEL_SHIFT            ((uint8_t)5U)
#define PMIC_LDO_DISCHARGE_SEL_MASK             ((uint8_t)3U << PMIC_LDO_DISCHARGE_SEL_SHIFT)
#define PMIC_LDO_DISCHARGE_EN_SHIFT             ((uint8_t)4U)
#define PMIC_LDO_DISCHARGE_EN_MASK              ((uint8_t)1U << PMIC_LDO_DISCHARGE_EN_SHIFT)
#define PMIC_LDO_EN_SHIFT                       ((uint8_t)0U)
#define PMIC_LDO_EN_MASK                        ((uint8_t)1U << PMIC_LDO_EN_SHIFT)
/** @} */

/**
 * @anchor Pmic_buck1MonConfBitFields
 * @name TPS65036x BUCK1_MON_CONF Register Bit Fields
 *
 * @brief Mask and shift values for all bit fields of the BUCK1_MON_CONF register
 *
 * @{
 */
#define PMIC_BUCK1_UV_THR_SHIFT                 ((uint8_t)6U)
#define PMIC_BUCK1_UV_THR_MASK                  ((uint8_t)3U << PMIC_BUCK1_UV_THR_SHIFT)
#define PMIC_BUCK1_OV_THR_SHIFT                 ((uint8_t)4U)
#define PMIC_BUCK1_OV_THR_MASK                  ((uint8_t)3U << PMIC_BUCK1_OV_THR_SHIFT)
#define PMIC_BUCK1_RV_CONF_SHIFT                ((uint8_t)3U)
#define PMIC_BUCK1_RV_CONF_MASK                 ((uint8_t)1U << PMIC_BUCK1_RV_CONF_SHIFT)
#define PMIC_BUCK1_ILIM_SEL_SHIFT               ((uint8_t)2U)
#define PMIC_BUCK1_ILIM_SEL_MASK                ((uint8_t)1U << PMIC_BUCK1_ILIM_SEL_SHIFT)
#define PMIC_BUCK1_DEGLITCH_SEL_SHIFT           ((uint8_t)0U)
#define PMIC_BUCK1_DEGLITCH_SEL_MASK            ((uint8_t)3U << PMIC_BUCK1_DEGLITCH_SEL_SHIFT)
/** @} */

/**
 * @anchor Pmic_buck2MonConfBitFields
 * @name TPS65036x BUCK2_MON_CONF Register Bit Fields
 *
 * @brief Mask and shift values for all bit fields of the BUCK2_MON_CONF register
 *
 * @{
 */
#define PMIC_BUCK2_UV_THR_SHIFT                 ((uint8_t)6U)
#define PMIC_BUCK2_UV_THR_MASK                  ((uint8_t)3U << PMIC_BUCK2_UV_THR_SHIFT)
#define PMIC_BUCK2_OV_THR_SHIFT                 ((uint8_t)4U)
#define PMIC_BUCK2_OV_THR_MASK                  ((uint8_t)3U << PMIC_BUCK2_OV_THR_SHIFT)
#define PMIC_BUCK2_RV_CONF_SHIFT                ((uint8_t)3U)
#define PMIC_BUCK2_RV_CONF_MASK                 ((uint8_t)1U << PMIC_BUCK2_RV_CONF_SHIFT)
#define PMIC_BUCK2_ILIM_SEL_SHIFT               ((uint8_t)2U)
#define PMIC_BUCK2_ILIM_SEL_MASK                ((uint8_t)1U << PMIC_BUCK2_ILIM_SEL_SHIFT)
#define PMIC_BUCK2_DEGLITCH_SEL_SHIFT           ((uint8_t)0U)
#define PMIC_BUCK2_DEGLITCH_SEL_MASK            ((uint8_t)3U << PMIC_BUCK2_DEGLITCH_SEL_SHIFT)
/** @} */

/**
 * @anchor Pmic_buck3MonConfBitFields
 * @name TPS65036x BUCK3_MON_CONF Register Bit Fields
 *
 * @brief Mask and shift values for all bit fields of the BUCK3_MON_CONF register
 *
 * @{
 */
#define PMIC_BUCK3_UV_THR_SHIFT                 ((uint8_t)6U)
#define PMIC_BUCK3_UV_THR_MASK                  ((uint8_t)3U << PMIC_BUCK3_UV_THR_SHIFT)
#define PMIC_BUCK3_OV_THR_SHIFT                 ((uint8_t)4U)
#define PMIC_BUCK3_OV_THR_MASK                  ((uint8_t)3U << PMIC_BUCK3_OV_THR_SHIFT)
#define PMIC_BUCK3_RV_CONF_SHIFT                ((uint8_t)3U)
#define PMIC_BUCK3_RV_CONF_MASK                 ((uint8_t)1U << PMIC_BUCK3_RV_CONF_SHIFT)
#define PMIC_BUCK3_ILIM_SEL_SHIFT               ((uint8_t)2U)
#define PMIC_BUCK3_ILIM_SEL_MASK                ((uint8_t)1U << PMIC_BUCK3_ILIM_SEL_SHIFT)
#define PMIC_BUCK3_DEGLITCH_SEL_SHIFT           ((uint8_t)0U)
#define PMIC_BUCK3_DEGLITCH_SEL_MASK            ((uint8_t)3U << PMIC_BUCK3_DEGLITCH_SEL_SHIFT)
/** @} */

/**
 * @anchor Pmic_ldoMonConfBitFields
 * @name TPS65036x LDO_MON_CONF Register Bit Fields
 *
 * @brief Mask and shift values for all bit fields of the LDO_MON_CONF register
 *
 * @{
 */
#define PMIC_LDO_UV_THR_SHIFT                 ((uint8_t)6U)
#define PMIC_LDO_UV_THR_MASK                  ((uint8_t)3U << PMIC_LDO_UV_THR_SHIFT)
#define PMIC_LDO_OV_THR_SHIFT                 ((uint8_t)4U)
#define PMIC_LDO_OV_THR_MASK                  ((uint8_t)3U << PMIC_LDO_OV_THR_SHIFT)
#define PMIC_LDO_RV_CONF_SHIFT                ((uint8_t)3U)
#define PMIC_LDO_RV_CONF_MASK                 ((uint8_t)1U << PMIC_LDO_RV_CONF_SHIFT)
#define PMIC_LDO_ILIM_SEL_SHIFT               ((uint8_t)2U)
#define PMIC_LDO_ILIM_SEL_MASK                ((uint8_t)1U << PMIC_LDO_ILIM_SEL_SHIFT)
#define PMIC_LDO_DEGLITCH_SEL_SHIFT           ((uint8_t)0U)
#define PMIC_LDO_DEGLITCH_SEL_MASK            ((uint8_t)3U << PMIC_LDO_DEGLITCH_SEL_SHIFT)
/** @} */

/**
 * @anchor Pmic_seqTrigBuck1BitFields
 * @name TPS65036x SEQ_TRIG_BUCK1 Register Bit Fields
 *
 * @brief Mask and shift values for all bit fields of the SEQ_TRIG_BUCK1 register
 *
 * @{
 */
#define PMIC_PWR_ON_BIT_BUCK1_SHIFT         ((uint8_t)5U)
#define PMIC_PWR_ON_BIT_BUCK1_MASK          ((uint8_t)1U << PMIC_PWR_ON_BIT_BUCK1_SHIFT)
#define PMIC_LDO_PG_BUCK1_SHIFT             ((uint8_t)4U)
#define PMIC_LDO_PG_BUCK1_MASK              ((uint8_t)1U << PMIC_LDO_PG_BUCK1_SHIFT)
#define PMIC_BUCK3_PG_BUCK1_SHIFT           ((uint8_t)3U)
#define PMIC_BUCK3_PG_BUCK1_MASK            ((uint8_t)1U << PMIC_BUCK3_PG_BUCK1_SHIFT)
#define PMIC_BUCK2_PG_BUCK1_SHIFT           ((uint8_t)2U)
#define PMIC_BUCK2_PG_BUCK1_MASK            ((uint8_t)1U << PMIC_BUCK2_PG_BUCK1_SHIFT)
#define PMIC_GPIO_PIN_BUCK1_SHIFT           ((uint8_t)1U)
#define PMIC_GPIO_PIN_BUCK1_MASK            ((uint8_t)1U << PMIC_GPIO_PIN_BUCK1_SHIFT)
#define PMIC_SEQ_PIN_BUCK1_SHIFT            ((uint8_t)0U)
#define PMIC_SEQ_PIN_BUCK1_MASK             ((uint8_t)1U << PMIC_SEQ_PIN_BUCK1_SHIFT)
/** @} */

/**
 * @anchor Pmic_seqTrigBuck2BitFields
 * @name TPS65036x SEQ_TRIG_BUCK2 Register Bit Fields
 *
 * @brief Mask and shift values for all bit fields of the SEQ_TRIG_BUCK2 register
 *
 * @{
 */
#define PMIC_PWR_ON_BIT_BUCK2_SHIFT         ((uint8_t)5U)
#define PMIC_PWR_ON_BIT_BUCK2_MASK          ((uint8_t)1U << PMIC_PWR_ON_BIT_BUCK2_SHIFT)
#define PMIC_LDO_PG_BUCK2_SHIFT             ((uint8_t)4U)
#define PMIC_LDO_PG_BUCK2_MASK              ((uint8_t)1U << PMIC_LDO_PG_BUCK2_SHIFT)
#define PMIC_BUCK3_PG_BUCK2_SHIFT           ((uint8_t)3U)
#define PMIC_BUCK3_PG_BUCK2_MASK            ((uint8_t)1U << PMIC_BUCK3_PG_BUCK2_SHIFT)
#define PMIC_BUCK1_PG_BUCK2_SHIFT           ((uint8_t)2U)
#define PMIC_BUCK1_PG_BUCK2_MASK            ((uint8_t)1U << PMIC_BUCK1_PG_BUCK2_SHIFT)
#define PMIC_GPIO_PIN_BUCK2_SHIFT           ((uint8_t)1U)
#define PMIC_GPIO_PIN_BUCK2_MASK            ((uint8_t)1U << PMIC_GPIO_PIN_BUCK2_SHIFT)
#define PMIC_SEQ_PIN_BUCK2_SHIFT            ((uint8_t)0U)
#define PMIC_SEQ_PIN_BUCK2_MASK             ((uint8_t)1U << PMIC_SEQ_PIN_BUCK2_SHIFT)
/** @} */

/**
 * @anchor Pmic_seqTrigBuck3BitFields
 * @name TPS65036x SEQ_TRIG_BUCK3 Register Bit Fields
 *
 * @brief Mask and shift values for all bit fields of the SEQ_TRIG_BUCK3 register
 *
 * @{
 */
#define PMIC_PWR_ON_BIT_BUCK3_SHIFT         ((uint8_t)5U)
#define PMIC_PWR_ON_BIT_BUCK3_MASK          ((uint8_t)1U << PMIC_PWR_ON_BIT_BUCK3_SHIFT)
#define PMIC_LDO_PG_BUCK3_SHIFT             ((uint8_t)4U)
#define PMIC_LDO_PG_BUCK3_MASK              ((uint8_t)1U << PMIC_LDO_PG_BUCK3_SHIFT)
#define PMIC_BUCK2_PG_BUCK3_SHIFT           ((uint8_t)3U)
#define PMIC_BUCK2_PG_BUCK3_MASK            ((uint8_t)1U << PMIC_BUCK2_PG_BUCK3_SHIFT)
#define PMIC_BUCK1_PG_BUCK3_SHIFT           ((uint8_t)2U)
#define PMIC_BUCK1_PG_BUCK3_MASK            ((uint8_t)1U << PMIC_BUCK1_PG_BUCK3_SHIFT)
#define PMIC_GPIO_PIN_BUCK3_SHIFT           ((uint8_t)1U)
#define PMIC_GPIO_PIN_BUCK3_MASK            ((uint8_t)1U << PMIC_GPIO_PIN_BUCK3_SHIFT)
#define PMIC_SEQ_PIN_BUCK3_SHIFT            ((uint8_t)0U)
#define PMIC_SEQ_PIN_BUCK3_MASK             ((uint8_t)1U << PMIC_SEQ_PIN_BUCK3_SHIFT)
/** @} */

/**
 * @anchor Pmic_seqTrigLdoBitFields
 * @name TPS65036x SEQ_TRIG_LDO Register Bit Fields
 *
 * @brief Mask and shift values for all bit fields of the SEQ_TRIG_LDO register
 *
 * @{
 */
#define PMIC_PWR_ON_BIT_LDO_SHIFT           ((uint8_t)5U)
#define PMIC_PWR_ON_BIT_LDO_MASK            ((uint8_t)1U << PMIC_PWR_ON_BIT_LDO_SHIFT)
#define PMIC_BUCK3_PG_LDO_SHIFT             ((uint8_t)4U)
#define PMIC_BUCK3_PG_LDO_MASK              ((uint8_t)1U << PMIC_BUCK3_PG_LDO_SHIFT)
#define PMIC_BUCK2_PG_LDO_SHIFT             ((uint8_t)3U)
#define PMIC_BUCK2_PG_LDO_MASK              ((uint8_t)1U << PMIC_BUCK2_PG_LDO_SHIFT)
#define PMIC_BUCK1_PG_LDO_SHIFT             ((uint8_t)2U)
#define PMIC_BUCK1_PG_LDO_MASK              ((uint8_t)1U << PMIC_BUCK1_PG_LDO_SHIFT)
#define PMIC_GPIO_PIN_LDO_SHIFT             ((uint8_t)1U)
#define PMIC_GPIO_PIN_LDO_MASK              ((uint8_t)1U << PMIC_GPIO_PIN_LDO_SHIFT)
#define PMIC_SEQ_PIN_LDO_SHIFT              ((uint8_t)0U)
#define PMIC_SEQ_PIN_LDO_MASK               ((uint8_t)1U << PMIC_SEQ_PIN_LDO_SHIFT)
/** @} */

/**
 * @anchor Pmic_buck1SeqDlyBitFields
 * @name TPS65036x BUCK1_SEQ_DLY Register Bit Fields
 *
 * @brief Mask and shift values for all bit fields of the BUCK1_SEQ_DLY register
 *
 * @{
 */
#define PMIC_BUCK1_SEQ_DLY_OFF_SHIFT        ((uint8_t)4U)
#define PMIC_BUCK1_SEQ_DLY_OFF_MASK         ((uint8_t)0x0FU << PMIC_BUCK1_SEQ_DLY_OFF_SHIFT)
#define PMIC_BUCK1_SEQ_DLY_ON_SHIFT         ((uint8_t)0U)
#define PMIC_BUCK1_SEQ_DLY_ON_MASK          ((uint8_t)0x0FU << PMIC_BUCK1_SEQ_DLY_ON_SHIFT)
/** @} */

/**
 * @anchor Pmic_buck2SeqDlyBitFields
 * @name TPS65036x BUCK2_SEQ_DLY Register Bit Fields
 *
 * @brief Mask and shift values for all bit fields of the BUCK2_SEQ_DLY register
 *
 * @{
 */
#define PMIC_BUCK2_SEQ_DLY_OFF_SHIFT        ((uint8_t)4U)
#define PMIC_BUCK2_SEQ_DLY_OFF_MASK         ((uint8_t)0x0FU << PMIC_BUCK2_SEQ_DLY_OFF_SHIFT)
#define PMIC_BUCK2_SEQ_DLY_ON_SHIFT         ((uint8_t)0U)
#define PMIC_BUCK2_SEQ_DLY_ON_MASK          ((uint8_t)0x0FU << PMIC_BUCK2_SEQ_DLY_ON_SHIFT)
/** @} */

/**
 * @anchor Pmic_buck3SeqDlyBitFields
 * @name TPS65036x BUCK3_SEQ_DLY Register Bit Fields
 *
 * @brief Mask and shift values for all bit fields of the BUCK3_SEQ_DLY register
 *
 * @{
 */
#define PMIC_BUCK3_SEQ_DLY_OFF_SHIFT        ((uint8_t)4U)
#define PMIC_BUCK3_SEQ_DLY_OFF_MASK         ((uint8_t)0x0FU << PMIC_BUCK3_SEQ_DLY_OFF_SHIFT)
#define PMIC_BUCK3_SEQ_DLY_ON_SHIFT         ((uint8_t)0U)
#define PMIC_BUCK3_SEQ_DLY_ON_MASK          ((uint8_t)0x0FU << PMIC_BUCK3_SEQ_DLY_ON_SHIFT)
/** @} */

/**
 * @anchor Pmic_ldoSeqDlyBitFields
 * @name TPS65036x LDO_SEQ_DLY Register Bit Fields
 *
 * @brief Mask and shift values for all bit fields of the LDO_SEQ_DLY register
 *
 * @{
 */
#define PMIC_LDO_SEQ_DLY_OFF_SHIFT        ((uint8_t)4U)
#define PMIC_LDO_SEQ_DLY_OFF_MASK         ((uint8_t)0x0FU << PMIC_LDO_SEQ_DLY_OFF_SHIFT)
#define PMIC_LDO_SEQ_DLY_ON_SHIFT         ((uint8_t)0U)
#define PMIC_LDO_SEQ_DLY_ON_MASK          ((uint8_t)0x0FU << PMIC_LDO_SEQ_DLY_ON_SHIFT)
/** @} */

/**
 * @anchor Pmic_regOvpConfBitFields
 * @name TPS65036x REG_OVP_CONF Register Bit Fields
 *
 * @brief Mask and shift values for all bit fields of the REG_OVP_CONF register
 *
 * @{
 */
#define PMIC_LDO_OVP_SEL_SHIFT          ((uint8_t)6U)
#define PMIC_LDO_OVP_SEL_MASK           ((uint8_t)3U << PMIC_LDO_OVP_SEL_SHIFT)
#define PMIC_BUCK3_OVP_SEL_SHIFT        ((uint8_t)4U)
#define PMIC_BUCK3_OVP_SEL_MASK         ((uint8_t)3U << PMIC_BUCK3_OVP_SEL_SHIFT)
#define PMIC_BUCK2_OVP_SEL_SHIFT        ((uint8_t)2U)
#define PMIC_BUCK2_OVP_SEL_MASK         ((uint8_t)3U << PMIC_BUCK2_OVP_SEL_SHIFT)
#define PMIC_BUCK1_OVP_SEL_SHIFT        ((uint8_t)0U)
#define PMIC_BUCK1_OVP_SEL_MASK         ((uint8_t)3U << PMIC_BUCK1_OVP_SEL_SHIFT)
/** @} */

/**
 * @anchor Pmic_regOvConfBitFields
 * @name TPS65036x REG_OV_CONF Register Bit Fields
 *
 * @brief Mask and shift values for all bit fields of the REG_OV_CONF register
 *
 * @{
 */
#define PMIC_LDO_OV_SEL_SHIFT          ((uint8_t)6U)
#define PMIC_LDO_OV_SEL_MASK           ((uint8_t)3U << PMIC_LDO_OV_SEL_SHIFT)
#define PMIC_BUCK3_OV_SEL_SHIFT        ((uint8_t)4U)
#define PMIC_BUCK3_OV_SEL_MASK         ((uint8_t)3U << PMIC_BUCK3_OV_SEL_SHIFT)
#define PMIC_BUCK2_OV_SEL_SHIFT        ((uint8_t)2U)
#define PMIC_BUCK2_OV_SEL_MASK         ((uint8_t)3U << PMIC_BUCK2_OV_SEL_SHIFT)
#define PMIC_BUCK1_OV_SEL_SHIFT        ((uint8_t)0U)
#define PMIC_BUCK1_OV_SEL_MASK         ((uint8_t)3U << PMIC_BUCK1_OV_SEL_SHIFT)
/** @} */

/**
 * @anchor Pmic_regUvConfBitFields
 * @name TPS65036x REG_UV_CONF Register Bit Fields
 *
 * @brief Mask and shift values for all bit fields of the REG_UV_CONF register
 *
 * @{
 */
#define PMIC_LDO_UV_SEL_SHIFT          ((uint8_t)6U)
#define PMIC_LDO_UV_SEL_MASK           ((uint8_t)3U << PMIC_LDO_UV_SEL_SHIFT)
#define PMIC_BUCK3_UV_SEL_SHIFT        ((uint8_t)4U)
#define PMIC_BUCK3_UV_SEL_MASK         ((uint8_t)3U << PMIC_BUCK3_UV_SEL_SHIFT)
#define PMIC_BUCK2_UV_SEL_SHIFT        ((uint8_t)2U)
#define PMIC_BUCK2_UV_SEL_MASK         ((uint8_t)3U << PMIC_BUCK2_UV_SEL_SHIFT)
#define PMIC_BUCK1_UV_SEL_SHIFT        ((uint8_t)0U)
#define PMIC_BUCK1_UV_SEL_MASK         ((uint8_t)3U << PMIC_BUCK1_UV_SEL_SHIFT)
/** @} */

/**
 * @anchor Pmic_regScConfBitFields
 * @name TPS65036x REG_SC_CONF Register Bit Fields
 *
 * @brief Mask and shift values for all bit fields of the REG_SC_CONF register
 *
 * @{
 */
#define PMIC_LDO_SC_SEL_SHIFT          ((uint8_t)6U)
#define PMIC_LDO_SC_SEL_MASK           ((uint8_t)3U << PMIC_LDO_SC_SEL_SHIFT)
#define PMIC_BUCK3_SC_SEL_SHIFT        ((uint8_t)4U)
#define PMIC_BUCK3_SC_SEL_MASK         ((uint8_t)3U << PMIC_BUCK3_SC_SEL_SHIFT)
#define PMIC_BUCK2_SC_SEL_SHIFT        ((uint8_t)2U)
#define PMIC_BUCK2_SC_SEL_MASK         ((uint8_t)3U << PMIC_BUCK2_SC_SEL_SHIFT)
#define PMIC_BUCK1_SC_SEL_SHIFT        ((uint8_t)0U)
#define PMIC_BUCK1_SC_SEL_MASK         ((uint8_t)3U << PMIC_BUCK1_SC_SEL_SHIFT)
/** @} */

/**
 * @anchor Pmic_statBuck1_2BitFields
 * @name TPS65036x STAT_BUCK1_2 Register Bit Fields
 *
 * @brief Mask and shift values for all bit fields of the STAT_BUCK1_2 register
 *
 * @{
 */
#define PMIC_BUCK2_OVP_STAT_SHIFT       ((uint8_t)6U)
#define PMIC_BUCK2_OVP_STAT_MASK        ((uint8_t)1U << PMIC_BUCK2_OVP_STAT_SHIFT)
#define PMIC_BUCK2_UV_STAT_SHIFT        ((uint8_t)5U)
#define PMIC_BUCK2_UV_STAT_MASK         ((uint8_t)1U << PMIC_BUCK2_UV_STAT_SHIFT)
#define PMIC_BUCK2_OV_STAT_SHIFT        ((uint8_t)4U)
#define PMIC_BUCK2_OV_STAT_MASK         ((uint8_t)1U << PMIC_BUCK2_OV_STAT_SHIFT)
#define PMIC_BUCK1_OVP_STAT_SHIFT       ((uint8_t)2U)
#define PMIC_BUCK1_OVP_STAT_MASK        ((uint8_t)1U << PMIC_BUCK1_OVP_STAT_SHIFT)
#define PMIC_BUCK1_UV_STAT_SHIFT        ((uint8_t)1U)
#define PMIC_BUCK1_UV_STAT_MASK         ((uint8_t)1U << PMIC_BUCK1_UV_STAT_SHIFT)
#define PMIC_BUCK1_OV_STAT_SHIFT        ((uint8_t)0U)
#define PMIC_BUCK1_OV_STAT_MASK         ((uint8_t)1U << PMIC_BUCK1_OV_STAT_SHIFT)
/** @} */

/**
 * @anchor Pmic_statBuck3LdoBitFields
 * @name TPS65036x STAT_BUCK3_LDO Register Bit Fields
 *
 * @brief Mask and shift values for all bit fields of the STAT_BUCK3_LDO register
 *
 * @{
 */
#define PMIC_LDO_OVP_STAT_SHIFT         ((uint8_t)6U)
#define PMIC_LDO_OVP_STAT_MASK          ((uint8_t)1U << PMIC_LDO_OVP_STAT_SHIFT)
#define PMIC_LDO_UV_STAT_SHIFT          ((uint8_t)5U)
#define PMIC_LDO_UV_STAT_MASK           ((uint8_t)1U << PMIC_LDO_UV_STAT_SHIFT)
#define PMIC_LDO_OV_STAT_SHIFT          ((uint8_t)4U)
#define PMIC_LDO_OV_STAT_MASK           ((uint8_t)1U << PMIC_LDO_OV_STAT_SHIFT)
#define PMIC_BUCK3_OVP_STAT_SHIFT       ((uint8_t)2U)
#define PMIC_BUCK3_OVP_STAT_MASK        ((uint8_t)1U << PMIC_BUCK3_OVP_STAT_SHIFT)
#define PMIC_BUCK3_UV_STAT_SHIFT        ((uint8_t)1U)
#define PMIC_BUCK3_UV_STAT_MASK         ((uint8_t)1U << PMIC_BUCK3_UV_STAT_SHIFT)
#define PMIC_BUCK3_OV_STAT_SHIFT        ((uint8_t)0U)
#define PMIC_BUCK3_OV_STAT_MASK         ((uint8_t)1U << PMIC_BUCK3_OV_STAT_SHIFT)
/** @} */

/* ========================================================================== */
/*                    TPS65036x Power Module Register Map                     */
/* ========================================================================== */

#define PMIC_BUCK1_VOUT_REGADDR                 ((uint8_t)0x11U)
#define PMIC_BUCK2_VOUT_ACTIVE_REGADDR          ((uint8_t)0x12U)
#define PMIC_BUCK3_VOUT_ACTIVE_REGADDR          ((uint8_t)0x13U)
#define PMIC_BUCK2_VOUT_LOWPWR_REGADDR          ((uint8_t)0x14U)
#define PMIC_BUCK3_VOUT_LOWPWR_REGADDR          ((uint8_t)0x15U)
#define PMIC_BUCK1_CTRL_REGADDR                 ((uint8_t)0x18U)
#define PMIC_BUCK2_CTRL_REGADDR                 ((uint8_t)0x19U)
#define PMIC_BUCK3_CTRL_REGADDR                 ((uint8_t)0x1AU)
#define PMIC_BUCK1_UVLO_REGADDR                 ((uint8_t)0x1BU)
#define PMIC_LDO_CONF_REGADDR                   ((uint8_t)0x1CU)
#define PMIC_LDO_CTRL_REGADDR                   ((uint8_t)0x1DU)
#define PMIC_BUCK1_MON_CONF_REGADDR             ((uint8_t)0x1EU)
#define PMIC_BUCK2_MON_CONF_REGADDR             ((uint8_t)0x1FU)
#define PMIC_BUCK3_MON_CONF_REGADDR             ((uint8_t)0x20U)
#define PMIC_LDO_MON_CONF_REGADDR               ((uint8_t)0x21U)
#define PMIC_SEQ_TRIG_BUCK1_REGADDR             ((uint8_t)0x26U)
#define PMIC_SEQ_TRIG_BUCK2_REGADDR             ((uint8_t)0x27U)
#define PMIC_SEQ_TRIG_BUCK3_REGADDR             ((uint8_t)0x28U)
#define PMIC_SEQ_TRIG_LDO_REGADDR               ((uint8_t)0x29U)
#define PMIC_BUCK1_SEQ_DLY_REGADDR              ((uint8_t)0x2CU)
#define PMIC_BUCK2_SEQ_DLY_REGADDR              ((uint8_t)0x2DU)
#define PMIC_BUCK3_SEQ_DLY_REGADDR              ((uint8_t)0x2EU)
#define PMIC_LDO_SEQ_DLY_REGADDR                ((uint8_t)0x2FU)
#define PMIC_REG_OVP_CONF_REGADDR               ((uint8_t)0x32U)
#define PMIC_REG_OV_CONF_REGADDR                ((uint8_t)0x33U)
#define PMIC_REG_UV_CONF_REGADDR                ((uint8_t)0x34U)
#define PMIC_REG_SC_CONF_REGADDR                ((uint8_t)0x35U)
#define PMIC_STAT_BUCK1_2_REGADDR               ((uint8_t)0x56U)
#define PMIC_STAT_BUCK3_LDO_REGADDR             ((uint8_t)0x57U)

#ifdef __cplusplus
}

#endif /* __cplusplus */
#endif /* __POWER_H__ */
