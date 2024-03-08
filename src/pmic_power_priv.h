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
 *  \file pmic_power_priv.h
 *
 * \brief: This file contains macro definitions, structures and function
 *         prototypes for driver specific PMIC power configuration
 */

#ifndef PMIC_POWER_PRIV_H_
#define PMIC_POWER_PRIV_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include "pmic_irq_priv.h"

#ifdef __cplusplus
extern "C"
{
#endif

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/*!
 * \brief  BUCK Control Register Address
 */
#define PMIC_BUCK1_CTRL_REGADDR                                     (0x04U)
#define PMIC_BUCK2_CTRL_REGADDR                                     (0x06U)
#define PMIC_BUCK3_CTRL_REGADDR                                     (0x08U)
#define PMIC_BUCK4_CTRL_REGADDR                                     (0x0AU)
#define PMIC_BUCK5_CTRL_REGADDR                                     (0x0CU)

/*!
 * \brief  BUCK configuration Register Address
 */
#define PMIC_BUCK1_CONF_REGADDR                                     (0x05U)
#define PMIC_BUCK2_CONF_REGADDR                                     (0x07U)
#define PMIC_BUCK3_CONF_REGADDR                                     (0x09U)
#define PMIC_BUCK4_CONF_REGADDR                                     (0x0BU)
#define PMIC_BUCK5_CONF_REGADDR                                     (0x0DU)

/*!
 * \brief  BUCK voltage selection Register Address
 */
#define PMIC_BUCK1_VOUT_1_REGADDR                                   (0x0EU)
#define PMIC_BUCK1_VOUT_2_REGADDR                                   (0x0FU)
#define PMIC_BUCK2_VOUT_1_REGADDR                                   (0x10U)
#define PMIC_BUCK2_VOUT_2_REGADDR                                   (0x11U)
#define PMIC_BUCK3_VOUT_1_REGADDR                                   (0x12U)
#define PMIC_BUCK3_VOUT_2_REGADDR                                   (0x13U)
#define PMIC_BUCK4_VOUT_1_REGADDR                                   (0x14U)
#define PMIC_BUCK4_VOUT_2_REGADDR                                   (0x15U)
#define PMIC_BUCK5_VOUT_1_REGADDR                                   (0x16U)
#define PMIC_BUCK5_VOUT_2_REGADDR                                   (0x17U)

/*!
 * \brief  BUCK power-good window Register Address
 */
#define PMIC_BUCK1_PG_WIN_REGADDR                                   (0x18U)
#define PMIC_BUCK2_PG_WIN_REGADDR                                   (0x19U)
#define PMIC_BUCK3_PG_WIN_REGADDR                                   (0x1AU)
#define PMIC_BUCK4_PG_WIN_REGADDR                                   (0x1BU)
#define PMIC_BUCK5_PG_WIN_REGADDR                                   (0x1CU)

/*!
 * \brief  LDO control Register Address
 */
#define PMIC_LDO1_CTRL_REGADDR                                      (0x1DU)
#define PMIC_LDO2_CTRL_REGADDR                                      (0x1EU)
#define PMIC_LDO3_CTRL_REGADDR                                      (0x1FU)
#define PMIC_LDO4_CTRL_REGADDR                                      (0x20U)

/*!
 * \brief  LDORTC control Register Address
 */
#define PMIC_LDORTC_CTRL_REGADDR                                    (0x22U)

/*!
 * \brief  LDO voltage selection Register Address
 */
#define PMIC_LDO1_VOUT_REGADDR                                      (0x23U)
#define PMIC_LDO2_VOUT_REGADDR                                      (0x24U)
#define PMIC_LDO3_VOUT_REGADDR                                      (0x25U)
#define PMIC_LDO4_VOUT_REGADDR                                      (0x26U)

/*!
 * \brief  LDO power-good window Register Address
 */
#define PMIC_LDO1_PG_WIN_REGADDR                                    (0x27U)
#define PMIC_LDO2_PG_WIN_REGADDR                                    (0x28U)
#define PMIC_LDO3_PG_WIN_REGADDR                                    (0x29U)
#define PMIC_LDO4_PG_WIN_REGADDR                                    (0x2AU)

/*!
 * \brief  VCCA Vmon control and power-good window Register Address
 */
#define PMIC_VCCA_VMON_CTRL_REGADDR                                 (0x2BU)
#define PMIC_VCCA_PG_WINDOW_REGADDR                                 (0x2CU)

/*!
 * \brief  PMIC PGOOD SEL Register Address
 */
#define PMIC_PGOOD_SEL_1_REGADDR                                    (0x78U)
#define PMIC_PGOOD_SEL_2_REGADDR                                    (0x79U)
#define PMIC_PGOOD_SEL_3_REGADDR                                    (0x7AU)
#define PMIC_PGOOD_SEL_4_REGADDR                                    (0x7BU)

/*!
 * \brief  PMIC Power status Register Address
 */
#define PMIC_STAT_BUCK1_2_REGADDR                                   (0x6DU)
#define PMIC_STAT_BUCK3_4_REGADDR                                   (0x6EU)
#define PMIC_STAT_BUCK5_REGADDR                                     (0x6FU)
#define PMIC_STAT_LDO1_2_REGADDR                                    (0x70U)
#define PMIC_STAT_LDO3_4_REGADDR                                    (0x71U)
#define PMIC_STAT_VMON_REGADDR                                      (0x72U)
#define PMIC_STAT_MISC_REGADDR                                      (0x74U)
#define PMIC_STAT_MODERATE_ERR_REGADDR                              (0x75U)
#define PMIC_STAT_SEVERE_ERR_REGADDR                                (0x76U)

/*!
 * \brief  PMIC RAIL SEL Register Address
 */
#define PMIC_RAIL_SEL_1_REGADDR                                     (0x41U)
#define PMIC_RAIL_SEL_2_REGADDR                                     (0x42U)
#define PMIC_RAIL_SEL_3_REGADDR                                     (0x43U)

/*!
 * \brief  PMIC LDO residual voltage timeout Register Address
 */
#define PMIC_LDO_RV_TIMEOUT_REG_1_REGADDR                           (0x8CU)
#define PMIC_LDO_RV_TIMEOUT_REG_2_REGADDR                           (0x8DU)

/*!
 * \brief  FSM Register Address
 */
#define PMIC_FSM_TRIG_SEL_1_REGADDR                                 (0x44U)
#define PMIC_FSM_TRIG_SEL_2_REGADDR                                 (0x45U)

/*!
 * \brief  FSM MASK Register Address
 */
#define PMIC_FSM_TRIG_MASK_1_REGADDR                                (0x46U)
#define PMIC_FSM_TRIG_MASK_2_REGADDR                                (0x47U)
#define PMIC_FSM_TRIG_MASK_3_REGADDR                                (0x48U)

/*!
 * \brief  PMIC power voltage levels
 */
#define PMIC_POWER_VOLTAGE_300MV                                    (uint16_t)(300U)
#define PMIC_POWER_VOLTAGE_580MV                                    (uint16_t)(580U)
#define PMIC_POWER_VOLTAGE_600MV                                    (uint16_t)(600U)
#define PMIC_POWER_VOLTAGE_1095MV                                   (uint16_t)(1095U)
#define PMIC_POWER_VOLTAGE_1100MV                                   (uint16_t)(1100U)
#define PMIC_POWER_VOLTAGE_1200MV                                   (uint16_t)(1200U)
#define PMIC_POWER_VOLTAGE_1650MV                                   (uint16_t)(1650U)
#define PMIC_POWER_VOLTAGE_1660MV                                   (uint16_t)(1660U)
#define PMIC_POWER_VOLTAGE_3300MV                                   (uint16_t)(3300U)
#define PMIC_POWER_VOLTAGE_3340MV                                   (uint16_t)(3340U)
#define PMIC_POWER_VOLTAGE_3350MV                                   (uint16_t)(3350U)
#define PMIC_POWER_VOLTAGE_5000MV                                   (uint16_t)(5000U)

/*!
 * \brief  PMIC power voltage step levels
 */
#define PMIC_POWER_VOLTAGE_STEP_5V                                  (5U)
#define PMIC_POWER_VOLTAGE_STEP_10V                                 (10U)
#define PMIC_POWER_VOLTAGE_STEP_20V                                 (20U)
#define PMIC_POWER_VOLTAGE_STEP_25V                                 (25U)
#define PMIC_POWER_VOLTAGE_STEP_50V                                 (50U)

/*!
 * \brief  PMIC power voltage minimum and maximum levels
 */
#define PMIC_POWER_BUCK_MIN_VOLTAGE                                 (300U)
#define PMIC_POWER_BUCK_MAX_VOLTAGE                                 (3340U)
#define PMIC_POWER_LDO1_2_3_MIN_VOLTAGE                             (600U)
#define PMIC_POWER_LDO4_MIN_VOLTAGE                                 (1200U)
#define PMIC_POWER_LDO_MAX_VOLTAGE                                  (3300U)

/**
 * \brief  Power Resource (Buck, LDO, VMON) control bit positions
 *          1. BUCKX_CTRL
 *          2. LDOX_CTRL
 */
#define PMIC_POWER_RESOURCEX_RV_SEL_SHIFT               (7U)
#define PMIC_POWER_RESOURCEX_EN_SHIFT                   (0U)
/** 
 * \brief   VCCA_VMON_CTRL register bit positions   
 */ 
#define PMIC_VMON1_RV_SEL_SHIFT                         (2U)
#define PMIC_VMON2_RV_SEL_SHIFT                         (4U)
#define PMIC_VCCA_VMON_EN_SHIFT                         (0U)
#define PMIC_VMON2_EN_SHIFT                             (3U)
#define PMIC_VMON1_EN_SHIFT                             (1U)
#define PMIC_VMON_DEGLITCH_SEL_SHIFT                    (5U)
/**         
 * @brief   BUCKX_CTRL register bit positions           
 */         
#define PMIC_BUCKX_PLDN_SHIFT                           (5U)
#define PMIC_BUCKX_VSEL_SHIFT                           (3U)
#define PMIC_BUCKX_FPWM_MP_SHIFT                        (2U)
#define PMIC_BUCKX_FPWM_SHIFT                           (1U)
/**         
 * \brief   LDOX_CTRL register bit positions            
 */         
#define PMIC_LDOX_PLDN_SHIFT                            (5U)
#define PMIC_LDOX_SLOW_RAMP_EN_SHIFT                    (1U)
/**
 * \brief   VMON enable bit position in power resource control registers
 *          1. BUCKX_CTRL
 *          2. LDOX_CTRL
 */
#define PMIC_REGULATOR_VMON_EN_SHIFT                    (4U)

/*!
 * \brief  PMIC BUCKX_CONF bit positions 
 */
#define PMIC_BUCKX_ILIM_SHIFT                           (3U)
#define PMIC_BUCKX_SLEW_RATE_SHIFT                      (0U)

/*!
 * \brief  BUCK BUCKX_VOUT_X bit positions
 */
#define PMIC_BUCKX_VSETX_SHIFT                          (0U)

/*!
 * \brief  PMIC Over and Under Voltage Threshold bit positions
 *          1. LDOX_PG_WINDOW
 *          2. BUCKX_PG_WINDOW
 *          3. VCCA_PG_WINDOW
 *          4. VMONX_PG_WINDOW
 */
#define PMIC_POWER_RESOURCEX_UV_THR_SHIFT               (3U)
#define PMIC_POWER_RESOURCEX_OV_THR_SHIFT               (0U)

/*!
 * \brief  PMIC LDORTC_CTRL bit positions
 */
#define PMIC_LDORTC_DIS_SHIFT                           (0U)

/*!
 * \brief  LDOX_VOUT register bit positions
 */
#define PMIC_LDOX_BYPASS_SHIFT                          (7U)
#define PMIC_LDOX_VSET_SHIFT                            (1U)
#define PMIC_LDO4_VSET_SHIFT                            (0U)

/*!
 * \brief  PMIC VCCA_PG_WINDOW register bit positions
 */
#define PMIC_VCCA_PG_SET_SHIFT                          (6U)

/*!
 * \brief  PMIC RAIL_SEL_X bit positions 
 */
#define PMIC_BUCK1_GRP_SEL_SHIFT                        (0U)
#define PMIC_BUCK2_GRP_SEL_SHIFT                        (2U)
#define PMIC_BUCK3_GRP_SEL_SHIFT                        (4U)
#define PMIC_BUCK4_GRP_SEL_SHIFT                        (6U)
#define PMIC_BUCK5_GRP_SEL_SHIFT                        (0U)
#define PMIC_LDO1_GRP_SEL_SHIFT                         (2U)
#define PMIC_LDO2_GRP_SEL_SHIFT                         (4U)
#define PMIC_LDO3_GRP_SEL_SHIFT                         (6U)
#define PMIC_LDO4_GRP_SEL_SHIFT                         (0U)
#define PMIC_VCCA_GRP_SEL_SHIFT                         (2U)
#define PMIC_VMON1_GRP_SEL_SHIFT                        (4U)
#define PMIC_VMON2_GRP_SEL_SHIFT                        (6U)

/*!
 * \brief  PMIC RAIL_SEL_X register GRP_SEL bit mask
 */
#define PMIC_PWR_RSRC_X_GRP_SEL_BITFIELD                (0x3U)

/*!
 * \brief  PMIC MASK_BUCKX_Y register bit positions
 */
#define PMIC_BUCK2_4_ILIM_MASK_SHIFT                    (7U)
#define PMIC_BUCK2_4_UV_MASK_SHIFT                      (5U)
#define PMIC_BUCK2_4_OV_MASK_SHIFT                      (4U)
#define PMIC_BUCK1_3_ILIM_MASK_SHIFT                    (3U)
#define PMIC_BUCK1_3_UV_MASK_SHIFT                      (1U)
#define PMIC_BUCK1_3_OV_MASK_SHIFT                      (0U)
/*!
* \brief   MASK_BUCK5 register bit positions 
*/
#define PMIC_BUCK5_ILIM_MASK_SHIFT                      (3U)
#define PMIC_BUCK5_UV_MASK_SHIFT                        (1U)
#define PMIC_BUCK5_OV_MASK_SHIFT                        (0U)
/*!
 * \brief  PMIC MASK_LDOX_Y bit positions
 */
#define PMIC_LDO2_4_ILIM_MASK_SHIFT                     (7U)
#define PMIC_LDO2_4_UV_MASK_SHIFT                       (5U)
#define PMIC_LDO2_4_OV_MASK_SHIFT                       (4U)
#define PMIC_LDO1_3_ILIM_MASK_SHIFT                     (3U)
#define PMIC_LDO1_3_UV_MASK_SHIFT                       (1U)
#define PMIC_LDO1_3_OV_MASK_SHIFT                       (0U)
/*!
 * \brief  PMIC MASK_VMON bit poisitions
 */
#define PMIC_VCCA_UV_MASK_SHIFT                         (1U)
#define PMIC_VCCA_OV_MASK_SHIFT                         (0U)
#define PMIC_VMON2_UV_MASK_SHIFT                        (6U)
#define PMIC_VMON2_OV_MASK_SHIFT                        (5U)
#define PMIC_VMON1_UV_MASK_SHIFT                        (3U)
#define PMIC_VMON1_OV_MASK_SHIFT                        (2U)

/*!
 * \brief  PMIC MASK_MISC register bit positions
 */
#define PMIC_TWARN_MASK_SHIFT                           (3U)

/*!
 * \brief  PMIC Power Resource Interrupt bit field
 */
#define PMIC_PWR_RSRC_INTR_MASK_BITFIELD                (1U)

/*!
 * \brief  PMIC STAT_BUCKX_Y register bit positions
 */
#define PMIC_BUCK2_4_ILIM_STAT_SHIFT                    (7U)
#define PMIC_BUCK2_4_UV_STAT_SHIFT                      (5U)
#define PMIC_BUCK2_4_OV_STAT_SHIFT                      (4U)
#define PMIC_BUCK1_3_ILIM_STAT_SHIFT                    (3U)
#define PMIC_BUCK1_3_UV_STAT_SHIFT                      (1U)
#define PMIC_BUCK1_3_OV_STAT_SHIFT                      (0U)
/*!
 * \brief  PMIC STAT_BUCK5 register bit positions
 */
#define PMIC_BUCK5_ILIM_STAT_SHIFT                      (3U)
#define PMIC_BUCK5_UV_STAT_SHIFT                        (1U)
#define PMIC_BUCK5_OV_STAT_SHIFT                        (0U)
/*!
 * \brief  PMIC STAT_LDOX_Y register bit positions
 */
#define PMIC_LDO2_4_ILIM_STAT_SHIFT                     (7U)
#define PMIC_LDO2_4_UV_STAT_SHIFT                       (5U)
#define PMIC_LDO2_4_OV_STAT_SHIFT                       (4U)
#define PMIC_LDO1_3_ILIM_STAT_SHIFT                     (3U)
#define PMIC_LDO1_3_UV_STAT_SHIFT                       (1U)
#define PMIC_LDO1_3_OV_STAT_SHIFT                       (0U)
/*!
 * \brief  PMIC STAT_VMON bit positions
 */
#define PMIC_VCCA_UV_STAT_SHIFT                         (1U)
#define PMIC_VCCA_OV_STAT_SHIFT                         (0U)
#define PMIC_VMON2_UV_STAT_SHIFT                        (6U)
#define PMIC_VMON2_OV_STAT_SHIFT                        (5U)
#define PMIC_VMON1_UV_STAT_SHIFT                        (3U)
#define PMIC_VMON1_OV_STAT_SHIFT                        (2U)
/*!
 * \brief  PMIC Power Resources Status bit positions
 */
#define PMIC_POWER_RESOURCE_STATUS_BITFIELD             (1U)
/*!
 * \brief  PMIC Thermal status for die bit positions
 *         1. STAT_MISC
 *         2. STAT_MODERATE_ERR
 *         3. STAT_SEVERE_ERR
 */
#define PMIC_TWARN_STAT_SHIFT                           (3U)
#define PMIC_TSD_ORD_STAT_SHIFT                         (0U)
#define PMIC_TSD_IMM_STAT_SHIFT                         (0U)

/*!
 * \brief  PMIC severe error for VCCA OV bit positions
            1. STAT_SEVERE_ERR
 */
#define PMIC_VCCA_OVP_STAT_SHIFT                        (1U)

/*!
 * \brief  PMIC PGOOD_SEL_X bit positions, where X is in the range of [1, 4]
 */
#define PMIC_PGOOD_SEL_BUCK1_SHIFT                      (0U)
#define PMIC_PGOOD_SEL_BUCK2_SHIFT                      (2U)
#define PMIC_PGOOD_SEL_BUCK3_SHIFT                      (4U)
#define PMIC_PGOOD_SEL_BUCK4_SHIFT                      (6U)
#define PMIC_PGOOD_SEL_BUCK5_SHIFT                      (0U)
#define PMIC_PGOOD_SEL_LDO1_SHIFT                       (0U)
#define PMIC_PGOOD_SEL_LDO2_SHIFT                       (2U)
#define PMIC_PGOOD_SEL_LDO3_SHIFT                       (4U)
#define PMIC_PGOOD_SEL_LDO4_SHIFT                       (6U)
#define PMIC_PGOOD_WINDOW_SHIFT                         (7U)
#define PMIC_PGOOD_POL_SHIFT                            (6U)
#define PMIC_PGOOD_SEL_NRSTOUT_SOC_SHIFT                (5U)
#define PMIC_PGOOD_SEL_NRSTOUT_SHIFT                    (4U)
#define PMIC_PGOOD_SEL_TDIE_WARN_SHIFT                  (3U)
#define PMIC_PGOOD_SEL_VMON2_SHIFT                      (2U)
#define PMIC_PGOOD_SEL_VMON1_SHIFT                      (1U)
#define PMIC_PGOOD_SEL_VCCA_SHIFT                       (0U)

/*!
 * \brief  PMIC Power-good signal source control bit field
 */
#define PMIC_PGOOD_SRC_BITFIELD                         (0x1U)
#define PMIC_PGOOD_SRC_REGULATOR_BITFIELD               (0x3U)

/*!
 * \brief  PMIC 2 LDO_RV_TIMEOUT_REG_X register bit positions
 */
#define PMIC_LDO2_4_RV_TIMEOUT_SHIFT                    (4U)
#define PMIC_LDO1_3_RV_TIMEOUT_SHIFT                    (0U)

/*!
 * \brief  PMIC FSM_TRIG_SEL_X register bit positions
 */
#define PMIC_SEVERE_ERR_TRIG_SHIFT                      (6U)
#define PMIC_OTHER_RAIL_TRIG_SHIFT                      (4U)
#define PMIC_SOC_RAIL_TRIG_SHIFT                        (2U)
#define PMIC_MCU_RAIL_TRIG_SHIFT                        (0U)
#define PMIC_MODERATE_ERR_TRIG_SHIFT                    (0U)

/*!
 * \brief  Power Resource (Buck, LDO, VMON) control bit masks
 *          1. BUCKX_CTRL
 *          2. LDOX_CTRL
 */
#define PMIC_POWER_RESOURCEX_RV_SEL_MASK                (uint8_t)(0x01U << PMIC_POWER_RESOURCEX_RV_SEL_SHIFT)
#define PMIC_POWER_RESOURCEX_EN_MASK                    (uint8_t)(0x01U << PMIC_POWER_RESOURCEX_EN_SHIFT)
/*! 
 * \brief   VCCA_VMON_CTRL register bit masks 
 */ 
#define PMIC_VMON1_RV_SEL_MASK                          (uint8_t)(0x01U << PMIC_VMON1_RV_SEL_SHIFT)
#define PMIC_VMON2_RV_SEL_MASK                          (uint8_t)(0x01U << PMIC_VMON2_RV_SEL_SHIFT)
#define PMIC_VCCA_VMON_EN_MASK                          (uint8_t)(0x01U << PMIC_VCCA_VMON_EN_SHIFT)
#define PMIC_VMON2_EN_MASK                              (uint8_t)(0x01U << PMIC_VMON2_EN_SHIFT)
#define PMIC_VMON1_EN_MASK                              (uint8_t)(0x01U << PMIC_VMON1_EN_SHIFT)
#define PMIC_VMON_DEGLITCH_SEL_MASK                     (uint8_t)(0x01U << PMIC_VMON_DEGLITCH_SEL_SHIFT)
/*!         
 * \brief   BUCKX_CTRL register bit masks           
 */    
#define PMIC_BUCKX_PLDN_MASK                            (uint8_t)(0x01U << PMIC_BUCKX_PLDN_SHIFT)
#define PMIC_BUCKX_VSEL_MASK                            (uint8_t)(0x01U << PMIC_BUCKX_VSEL_SHIFT)
#define PMIC_BUCKX_FPWM_MP_MASK                         (uint8_t)(0x01U << PMIC_BUCKX_FPWM_MP_SHIFT)
#define PMIC_BUCKX_FPWM_MASK                            (uint8_t)(0x01U << PMIC_BUCKX_FPWM_SHIFT)
/**         
 * \brief   LDOX_CTRL register bit masks           
 */         
#define PMIC_LDOX_PLDN_MASK                             (uint8_t)(0x3U << PMIC_LDOX_PLDN_SHIFT)
#define PMIC_LDOX_SLOW_RAMP_EN_MASK                     (uint8_t)(0x01U << PMIC_LDOX_SLOW_RAMP_EN_SHIFT)

/**
 * \brief   VMON enable bit masks in power resource control registers
 *          1. BUCKX_CTRL
 *          2. LDOX_CTRL
 */
#define PMIC_REGULATOR_VMON_EN_MASK                     (uint8_t)(0x01U << PMIC_REGULATOR_VMON_EN_SHIFT)

/*!
 * \brief  PMIC BUCKX_CONF register bit masks
 */
#define PMIC_BUCKX_ILIM_MASK                            (uint8_t)(0x07U << PMIC_BUCKX_ILIM_SHIFT)
#define PMIC_BUCKX_SLEW_RATE_MASK                       (uint8_t)(0x07U << PMIC_BUCKX_SLEW_RATE_SHIFT)

/*!
 * \brief  PMIC BUCKX_VOUT_X register bit masks
 */
#define PMIC_BUCKX_VSETX_MASK                           (uint8_t)(0xFFU << PMIC_BUCKX_VSETX_SHIFT)

/*!
 * \brief  PMIC Over and Under Voltage Threshold bit masks
 *          1. LDOX_PG_WINDOW
 *          2. BUCKX_PG_WINDOW
 *          3. VCCA_PG_WINDOW
 *          4. VMONX_PG_WINDOW
 */
#define PMIC_POWER_RESOURCEX_UV_THR_MASK                (uint8_t)(0x7U << PMIC_POWER_RESOURCEX_UV_THR_SHIFT)
#define PMIC_POWER_RESOURCEX_OV_THR_MASK                (uint8_t)(0x7U << PMIC_POWER_RESOURCEX_OV_THR_SHIFT)

/*!
 * \brief  PMIC LDORTC_CTRL register bit masks
 */
#define PMIC_LDORTC_DIS_MASK                            (uint8_t)(0x01U << PMIC_LDORTC_DIS_SHIFT)

/*!
 * \brief  LDOX_VOUT register bit masks
 */
#define PMIC_LDOX_BYPASS_MASK                           (uint8_t)(0x01U << PMIC_LDOX_BYPASS_SHIFT)
#define PMIC_LDOX_VSET_MASK                             (uint8_t)(0x3FU << PMIC_LDOX_VSET_SHIFT)
#define PMIC_LDO4_VSET_MASK                             (uint8_t)(0x7FU << PMIC_LDO4_VSET_SHIFT)

/*!
 * \brief  PMIC VCCA_PG_WINDOW register bit masks
 */
#define PMIC_VCCA_PG_SET_MASK                           (uint8_t)(0x01U << PMIC_VCCA_PG_SET_SHIFT)

/*!
 * \brief  PMIC RAIL_SEL_X register bit masks
 */
#define PMIC_BUCK1_GRP_SEL_MASK                         (uint8_t)(0x03U << PMIC_BUCK1_GRP_SEL_SHIFT)
#define PMIC_BUCK2_GRP_SEL_MASK                         (uint8_t)(0x03U << PMIC_BUCK2_GRP_SEL_SHIFT)
#define PMIC_BUCK3_GRP_SEL_MASK                         (uint8_t)(0x03U << PMIC_BUCK3_GRP_SEL_SHIFT)
#define PMIC_BUCK4_GRP_SEL_MASK                         (uint8_t)(0x03U << PMIC_BUCK4_GRP_SEL_SHIFT)
#define PMIC_BUCK5_GRP_SEL_MASK                         (uint8_t)(0x03U << PMIC_BUCK5_GRP_SEL_SHIFT)
#define PMIC_LDO1_GRP_SEL_MASK                          (uint8_t)(0x03U << PMIC_LDO1_GRP_SEL_SHIFT)
#define PMIC_LDO2_GRP_SEL_MASK                          (uint8_t)(0x03U << PMIC_LDO2_GRP_SEL_SHIFT)
#define PMIC_LDO3_GRP_SEL_MASK                          (uint8_t)(0x03U << PMIC_LDO3_GRP_SEL_SHIFT)
#define PMIC_LDO4_GRP_SEL_MASK                          (uint8_t)(0x03U << PMIC_LDO4_GRP_SEL_SHIFT)
#define PMIC_VCCA_GRP_SEL_MASK                          (uint8_t)(0x03U << PMIC_VCCA_GRP_SEL_SHIFT)
#define PMIC_VMON1_GRP_SEL_MASK                         (uint8_t)(0x03U << PMIC_VMON1_GRP_SEL_SHIFT)
#define PMIC_VMON2_GRP_SEL_MASK                         (uint8_t)(0x03U << PMIC_VMON2_GRP_SEL_SHIFT)

/*!
 * \brief  PMIC MASK_BUCKX_Y register bit masks
 */
#define PMIC_BUCK2_4_ILIM_MASK_MASK                     (uint8_t)(0x01U << PMIC_BUCK2_4_ILIM_MASK_SHIFT)
#define PMIC_BUCK2_4_UV_MASK_MASK                       (uint8_t)(0x01U << PMIC_BUCK2_4_UV_MASK_SHIFT)
#define PMIC_BUCK2_4_OV_MASK_MASK                       (uint8_t)(0x01U << PMIC_BUCK2_4_OV_MASK_SHIFT)
#define PMIC_BUCK1_3_ILIM_MASK_MASK                     (uint8_t)(0x01U << PMIC_BUCK1_3_ILIM_MASK_SHIFT)
#define PMIC_BUCK1_3_UV_MASK_MASK                       (uint8_t)(0x01U << PMIC_BUCK1_3_UV_MASK_SHIFT)
#define PMIC_BUCK1_3_OV_MASK_MASK                       (uint8_t)(0x01U << PMIC_BUCK1_3_OV_MASK_SHIFT)
/*!
 * \brief  PMIC MASK_BUCK5 register bit masks
 */
#define PMIC_BUCK5_ILIM_MASK_MASK                       (uint8_t)(0x01U << PMIC_BUCK5_ILIM_MASK_SHIFT)
#define PMIC_BUCK5_UV_MASK_MASK                         (uint8_t)(0x01U << PMIC_BUCK5_UV_MASK_SHIFT)
#define PMIC_BUCK5_OV_MASK_MASK                         (uint8_t)(0x01U << PMIC_BUCK5_OV_MASK_SHIFT)
/*!
 * \brief  PMIC MASK_LDOX_Y register bit masks 
 */
#define PMIC_LDO2_4_ILIM_MASK_MASK                      (uint8_t)(0x01U << PMIC_LDO2_4_ILIM_MASK_SHIFT)
#define PMIC_LDO2_4_UV_MASK_MASK                        (uint8_t)(0x01U << PMIC_LDO2_4_UV_MASK_SHIFT)
#define PMIC_LDO2_4_OV_MASK_MASK                        (uint8_t)(0x01U << PMIC_LDO2_4_OV_MASK_SHIFT)
#define PMIC_LDO1_3_ILIM_MASK_MASK                      (uint8_t)(0x01U << PMIC_LDO1_3_ILIM_MASK_SHIFT)
#define PMIC_LDO1_3_UV_MASK_MASK                        (uint8_t)(0x01U << PMIC_LDO1_3_UV_MASK_SHIFT)
#define PMIC_LDO1_3_OV_MASK_MASK                        (uint8_t)(0x01U << PMIC_LDO1_3_OV_MASK_SHIFT)
/*!
 * \brief  PMIC MASK_VMON register bit masks 
 */
#define PMIC_VCCA_UV_MASK_MASK                          (uint8_t)(0x01U << PMIC_VCCA_UV_MASK_SHIFT)
#define PMIC_VCCA_OV_MASK_MASK                          (uint8_t)(0x01U << PMIC_VCCA_OV_MASK_SHIFT)
#define PMIC_VMON2_UV_MASK_MASK                         (uint8_t)(0x01U << PMIC_VMON2_UV_MASK_SHIFT)
#define PMIC_VMON2_OV_MASK_MASK                         (uint8_t)(0x01U << PMIC_VMON2_OV_MASK_SHIFT)
#define PMIC_VMON1_UV_MASK_MASK                         (uint8_t)(0x01U << PMIC_VMON1_UV_MASK_SHIFT)
#define PMIC_VMON1_OV_MASK_MASK                         (uint8_t)(0x01U << PMIC_VMON1_OV_MASK_SHIFT)

/*!
 * \brief PMIC MASK_MISC register bit masks 
 */
#define PMIC_TWARN_MASK_MASK                            (uint8_t)(0x01U << PMIC_TWARN_MASK_SHIFT)
/*!
 * \brief  PMIC STAT_BUCKX_Y register bit masks
 */
#define PMIC_BUCK2_4_ILIM_STAT_MASK                     (uint8_t)(0x01U << PMIC_BUCK2_4_ILIM_STAT_SHIFT)
#define PMIC_BUCK2_4_UV_STAT_MASK                       (uint8_t)(0x01U << PMIC_BUCK2_4_UV_STAT_SHIFT)
#define PMIC_BUCK2_4_OV_STAT_MASK                       (uint8_t)(0x01U << PMIC_BUCK2_4_OV_STAT_SHIFT)
#define PMIC_BUCK1_3_ILIM_STAT_MASK                     (uint8_t)(0x01U << PMIC_BUCK1_3_ILIM_STAT_SHIFT)
#define PMIC_BUCK1_3_UV_STAT_MASK                       (uint8_t)(0x01U << PMIC_BUCK1_3_UV_STAT_SHIFT)
#define PMIC_BUCK1_3_OV_STAT_MASK                       (uint8_t)(0x01U << PMIC_BUCK1_3_OV_STAT_SHIFT)
/*!
 * \brief PMIC STAT_BUCK5 register bit masks
 */
#define PMIC_BUCK5_ILIM_STAT_MASK                       (uint8_t)(0x01U << PMIC_BUCK5_ILIM_STAT_SHIFT)
#define PMIC_BUCK5_UV_STAT_MASK                         (uint8_t)(0x01U << PMIC_BUCK5_UV_STAT_SHIFT)
#define PMIC_BUCK5_OV_STAT_MASK                         (uint8_t)(0x01U << PMIC_BUCK5_OV_STAT_SHIFT)
/*!
 * \brief PMIC STAT_LDOX_Y register bit masks 
 */
#define PMIC_LDO2_4_ILIM_STAT_MASK                      (uint8_t)(0x01U << PMIC_LDO2_4_ILIM_STAT_SHIFT)
#define PMIC_LDO2_4_UV_STAT_MASK                        (uint8_t)(0x01U << PMIC_LDO2_4_UV_STAT_SHIFT)
#define PMIC_LDO2_4_OV_STAT_MASK                        (uint8_t)(0x01U << PMIC_LDO2_4_OV_STAT_SHIFT)
#define PMIC_LDO1_3_ILIM_STAT_MASK                      (uint8_t)(0x01U << PMIC_LDO1_3_ILIM_STAT_SHIFT)
#define PMIC_LDO1_3_UV_STAT_MASK                        (uint8_t)(0x01U << PMIC_LDO1_3_UV_STAT_SHIFT)
#define PMIC_LDO1_3_OV_STAT_MASK                        (uint8_t)(0x01U << PMIC_LDO1_3_OV_STAT_SHIFT)
/*!
 * \brief STAT_VMON register bit masks 
 */
#define PMIC_VCCA_UV_STAT_MASK                          (uint8_t)(0x01U << PMIC_VCCA_UV_STAT_SHIFT)
#define PMIC_VCCA_OV_STAT_MASK                          (uint8_t)(0x01U << PMIC_VCCA_OV_STAT_SHIFT)
#define PMIC_VMON2_UV_STAT_MASK                         (uint8_t)(0x01U << PMIC_VMON2_UV_STAT_SHIFT)
#define PMIC_VMON2_OV_STAT_MASK                         (uint8_t)(0x01U << PMIC_VMON2_OV_STAT_SHIFT)
#define PMIC_VMON1_UV_STAT_MASK                         (uint8_t)(0x01U << PMIC_VMON1_UV_STAT_SHIFT)
#define PMIC_VMON1_OV_STAT_MASK                         (uint8_t)(0x01U << PMIC_VMON1_OV_STAT_SHIFT)

/*!
 * \brief  PMIC Thermal status for die bit masks
 *          1. STAT_MISC
 *          2. STAT_MODERATE_ERR
 *          3. STAT_SEVERE_ERR
 */
#define PMIC_TWARN_STAT_MASK                            (uint8_t)(0x01U << PMIC_TWARN_STAT_SHIFT)
#define PMIC_TSD_ORD_STAT_MASK                          (uint8_t)(0x01U << PMIC_TSD_ORD_STAT_SHIFT)
#define PMIC_TSD_IMM_STAT_MASK                          (uint8_t)(0x01U << PMIC_TSD_IMM_STAT_SHIFT)
/*!
 * \brief  PMIC severe error for VCCA OV bit masks
 *         1. STAT_SEVERE_ERR
 */
#define PMIC_VCCA_OVP_STAT_MASK                         (uint8_t)(0x01U << PMIC_VCCA_OVP_STAT_SHIFT)

/*!
 * \brief  PMIC PGOOD_SEL_X register bit masks, where X is in the range of [1, 4]
 */
#define PMIC_PGOOD_SEL_BUCK1_MASK           (uint8_t)(0x03U << PMIC_PGOOD_SEL_BUCK1_SHIFT)
#define PMIC_PGOOD_SEL_BUCK2_MASK           (uint8_t)(0x03U << PMIC_PGOOD_SEL_BUCK2_SHIFT)
#define PMIC_PGOOD_SEL_BUCK3_MASK           (uint8_t)(0x03U << PMIC_PGOOD_SEL_BUCK3_SHIFT)
#define PMIC_PGOOD_SEL_BUCK4_MASK           (uint8_t)(0x03U << PMIC_PGOOD_SEL_BUCK4_SHIFT)
#define PMIC_PGOOD_SEL_BUCK5_MASK           (uint8_t)(0x03U << PMIC_PGOOD_SEL_BUCK5_SHIFT)
#define PMIC_PGOOD_SEL_LDO1_MASK            (uint8_t)(0x03U << PMIC_PGOOD_SEL_LDO1_SHIFT)
#define PMIC_PGOOD_SEL_LDO2_MASK            (uint8_t)(0x03U << PMIC_PGOOD_SEL_LDO2_SHIFT)
#define PMIC_PGOOD_SEL_LDO3_MASK            (uint8_t)(0x03U << PMIC_PGOOD_SEL_LDO3_SHIFT)
#define PMIC_PGOOD_SEL_LDO4_MASK            (uint8_t)(0x03U << PMIC_PGOOD_SEL_LDO4_SHIFT)
#define PMIC_PGOOD_WINDOW_MASK              (uint8_t)(0x01U << PMIC_PGOOD_WINDOW_SHIFT)
#define PMIC_PGOOD_POL_MASK                 (uint8_t)(0x01U << PMIC_PGOOD_POL_SHIFT)
#define PMIC_PGOOD_SEL_NRSTOUT_SOC_MASK     (uint8_t)(0x01U << PMIC_PGOOD_SEL_NRSTOUT_SOC_SHIFT)
#define PMIC_PGOOD_SEL_NRSTOUT_MASK         (uint8_t)(0x01U << PMIC_PGOOD_SEL_NRSTOUT_SHIFT)
#define PMIC_PGOOD_SEL_TDIE_WARN_MASK       (uint8_t)(0x01U << PMIC_PGOOD_SEL_TDIE_WARN_SHIFT)
#define PMIC_PGOOD_SEL_VMON2_MASK           (uint8_t)(0x01U << PMIC_PGOOD_SEL_VMON2_SHIFT)
#define PMIC_PGOOD_SEL_VMON1_MASK           (uint8_t)(0x01U << PMIC_PGOOD_SEL_VMON1_SHIFT)
#define PMIC_PGOOD_SEL_VCCA_MASK            (uint8_t)(0x01U << PMIC_PGOOD_SEL_VCCA_SHIFT)

/*!
 * \brief  PMIC LDO_RV_TIMEOUT_REG_X register bit masks, where X is in the range of [1, 2]
 */
#define PMIC_LDO2_4_RV_TIMEOUT_MASK     (uint8_t)(0x0FU << PMIC_LDO2_4_RV_TIMEOUT_SHIFT)
#define PMIC_LDO1_3_RV_TIMEOUT_MASK     (uint8_t)(0x0FU << PMIC_LDO1_3_RV_TIMEOUT_SHIFT)

/*!
 * \brief  PMIC FSM_TRIG_SEL_1 register bit masks, where X is in the range of [1, 2]
 */
#define PMIC_SEVERE_ERR_TRIG_MASK       (uint8_t)(0x03U << PMIC_SEVERE_ERR_TRIG_SHIFT)
#define PMIC_OTHER_RAIL_TRIG_MASK       (uint8_t)(0x03U << PMIC_OTHER_RAIL_TRIG_SHIFT)
#define PMIC_SOC_RAIL_TRIG_MASK         (uint8_t)(0x03FU << PMIC_SOC_RAIL_TRIG_SHIFT)
#define PMIC_MCU_RAIL_TRIG_MASK         (uint8_t)(0x03U << PMIC_MCU_RAIL_TRIG_SHIFT)
#define PMIC_MODERATE_ERR_TRIG_MASK     (uint8_t)(0x03U << PMIC_MODERATE_ERR_TRIG_SHIFT)

/*!
 * \brief  PMIC power VSET Range Values
 */
#define PMIC_POWER_VSET_VAL_0xE         (0xEU)
#define PMIC_POWER_VSET_VAL_0xF         (0xFU)
#define PMIC_POWER_VSET_VAL_0x1D        (0x1DU)
#define PMIC_POWER_VSET_VAL_0x72        (0x72U)
#define PMIC_POWER_VSET_VAL_0x73        (0x73U)
#define PMIC_POWER_VSET_VAL_0xAA        (0xAAU)
#define PMIC_POWER_VSET_VAL_0xAB        (0xABU)
#define PMIC_POWER_VSET_VAL_0xFF        (0xFFU)

/*!
 * \brief  PMIC power VSET Values
 */
#define PMIC_POWER_VSET_VAL_0x0          (0x0U)
#define PMIC_POWER_VSET_VAL_0x4          (0x4U)
#define PMIC_POWER_VSET_VAL_0x20         (0x20U)

/*!
 * \brief   Bit field Value for buckFreq
 */
#define PMIC_POWER_BUCKX_FREQ_SEL_BITFIELD         (1U)

/*==========================================================================*/
/*                         Structures and Enums                             */
/*==========================================================================*/
/**
 * \brief   PMIC power resource details object structure
 *
 * \param ctrlRegAddr           Control Register address of the power resources
 *
 * \param configRegAddr         Configuration Register address of the power
 *                              resources
 *
 * \param vout1RegAddr          Register address of the power resource to set
 *                              the Voltage when output voltage register is
 *                              selected as VOUT1
 *
 * \param vout2RegAddr          Register address of the power resource to set
 *                              the Voltage when output voltage register is
 *                              selected as VOUT2
 *
 * \param pgWindowRegAddr       Register address of the power resource to
 *                              configure PowerGood Window configuration values.
 *
 * \param irqRegAddr            Interrupt masking/unmasking register address of
 *                              the power resource
 *
 * \param statusRegAddr         Status Register address of the power resource
 *                              which indicates when output volatage is above
 *                              over volatge, below under voltage and above
 *                              current limit level
 *
 * \param railGrpRegAddr        Rail group selection Register address of the
 *                              power resources
 *
 * \param railGrpBitShiftVal    Shift bits for the rail group selection of
 *                              the power resources
 *
 * \param ldoRvTimeOutRegAddr   Residual voltage timeout register address of
 *                              the LDO power regulator.
 *
 * \param ldoRvTimeOutBitShiftVal
 *                              Shift bits for the Residual voltage timeout for
 *                              the LDO power regulator.
 *
 */
typedef struct Pmic_powerResourceRegCfg_s
{
    uint8_t ctrlRegAddr;
    uint8_t configRegAddr;
    uint8_t vout1RegAddr;
    uint8_t vout2RegAddr;
    uint8_t pgWindowRegAddr;
    uint8_t irqNumber;
    uint8_t statusRegAddr;
    uint8_t railGrpRegAddr;
    uint8_t railGrpBitShiftVal;
    uint8_t ldoRvTimeOutRegAddr;
    uint8_t ldoRvTimeOutBitShiftVal;
    uint8_t iLimStatBitShift;
    uint8_t uvStatBitShift;
    uint8_t ovStatBitShift;
} Pmic_powerRsrcRegCfg_t;

/**
 * \brief   PMIC power resource details object structure
 *
 * \param regAddr          Register address of the Pgood sources.
 *
 * \param maskValue        Register mask value for the Pgood sources.
 *
 * \param shiftValue         Shift bits for the Pgood sources.
 *
 *
 */
typedef struct Pmic_powerPgoodSrcRegCfg_s
{
    uint8_t regAddr;
    uint8_t shiftValue;
} Pmic_powerPgoodSrcRegCfg_t;

/**
 * \brief   PMIC power interrupts Numbers
 *
 * \param inqNum         Interrupt number for respective interrupt
 */
typedef struct Pmic_powerIntCfg_s
{
    uint8_t irqNum;

} Pmic_powerIntCfg_t;

/*==========================================================================*/
/*                         Function Declarations                            */
/*==========================================================================*/

/*!
 * \brief   This function is used to get the id of power resource.
 *          BUCK1/BUCK2/LDO1...etc
 */
static inline uint8_t Pmic_powerGetPwrRsrcId(uint16_t pwrRsrc)
{
    uint8_t pwrRsrcId;
    pwrRsrcId = (uint8_t)(pwrRsrc & 0xFFU);

    return pwrRsrcId;
}

/*!
 * \brief   This function is used to get the type of power resource.
 *          BUCK/LDO/VCCA/VMON
 */
static inline uint8_t Pmic_powerGetPwrRsrcType(uint16_t pwrRsrc)
{
    uint8_t pwrRsrcType;
    pwrRsrcType = (uint8_t)((pwrRsrc >> 0x8U) & 0xFFU);

    return pwrRsrcType;
}

/*!
 * \brief   This function is used to get OV/UV voltage monitoring range for
 *          VMON2 and VMON1
 */
int32_t Pmic_powerGetVmonRange(Pmic_CoreHandle_t *pPmicCoreHandle, uint16_t pwrRsrc, bool *pVmonRange);

/*!
 * \brief   This function is used to convert the millivolt value to vset value
 *          for BUCK/VMON (For VMON : When range = 0)
 */
int32_t Pmic_powerBuckVmonConvertVoltage2VSetVal(uint16_t  millivolt,
                                                 uint16_t *pBaseMillivolt,
                                                 uint8_t  *pMillivoltStep,
                                                 uint8_t  *pBaseVoutCode);

/*!
 * \brief   This function is used to convert the millivolt value to vset value
 *          for LDO Regulators
 */
void Pmic_powerLdoConvertVoltage2VSetVal(uint16_t  pwrRsrc,
                                         uint16_t *pBaseMillivolt,
                                         uint8_t  *pMillivoltStep,
                                         uint8_t  *pBaseVoutCode);

/*!
 * \brief   This function is used to convert the millivolt value to vset code
 *          when the selected voltage monitoring range for VMON is
 *          PMIC_LP8764X_VMON_RANGE_3V35_5V
 */
void Pmic_powerVmonRange1ConvertVoltage2VSetVal(uint16_t *pBaseMillivolt,
                                                uint8_t  *pMillivoltStep,
                                                uint8_t  *pBaseVoutCode);

/*!
 * \brief   This function is used to convert the vset value to voltage in mv
            for BUCK/VMON
 */
void Pmic_powerBuckVmonConvertVSetVal2Voltage(const uint8_t *pVSetVal,
                                              uint16_t      *pBaseMillivolt,
                                              uint8_t       *pMillivoltStep,
                                              uint8_t       *pBaseVoutCode);

/*!
 * \brief   This function is used to convert the vset value to voltage in mv
 *          for LDO
 */
void Pmic_powerLdoConvertVSetVal2Voltage(uint16_t  pwrRsrc,
                                         uint16_t *pBaseMillivolt,
                                         uint8_t  *pMillivoltStep,
                                         uint8_t  *pBaseVoutCode);

/*!
 * \brief   This function is used to convert the vset value to voltage in mv
 *          when the selected voltage monitoring range for VMON is
 *          PMIC_LP8764X_VMON_RANGE_3V35_5V
 */
void Pmic_powerVmonRange1ConvertVSetVal2Voltage(uint16_t *pBaseMillivolt,
                                                uint8_t  *pMillivoltStep,
                                                uint8_t  *pBaseVoutCode);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* PMIC_POWER_PRIV_H_ */
