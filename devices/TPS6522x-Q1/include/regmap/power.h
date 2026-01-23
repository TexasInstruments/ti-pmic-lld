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
/**
 * @file power.h
 *
 * @brief PMIC LLD Power module register addresses and bit fields.
 */
#ifndef REGMAP_POWER_H
#define REGMAP_POWER_H

/* ========================================================================== */
/*                                Include Files                               */
/* ========================================================================== */
#include "pmic_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                              Register Addresses                            */
/* ========================================================================== */

#define BUCK1_CTRL_REG        ((uint16_t)0x04U)
#define BUCK1_CONF_REG        ((uint16_t)0x05U)
#define BUCK2_CTRL_REG        ((uint16_t)0x06U)
#define BUCK2_CONF_REG        ((uint16_t)0x07U)
#define BUCK3_CTRL_REG        ((uint16_t)0x08U)
#define BUCK3_CONF_REG        ((uint16_t)0x09U)
#define BUCK4_CTRL_REG        ((uint16_t)0x0AU)
#define BUCK4_CONF_REG        ((uint16_t)0x0BU)
#define BUCK1_VOUT_REG        ((uint16_t)0x0EU)
#define BUCK2_VOUT_REG        ((uint16_t)0x10U)
#define BUCK3_VOUT_REG        ((uint16_t)0x12U)
#define BUCK4_VOUT_REG        ((uint16_t)0x14U)
#define BUCK1_PG_WINDOW_REG   ((uint16_t)0x18U)
#define BUCK2_PG_WINDOW_REG   ((uint16_t)0x19U)
#define BUCK3_PG_WINDOW_REG   ((uint16_t)0x1AU)
#define BUCK4_PG_WINDOW_REG   ((uint16_t)0x1BU)
#define LDO1_CTRL_REG         ((uint16_t)0x1DU)
#define LDO2_CTRL_REG         ((uint16_t)0x1EU)
#define LDO3_CTRL_REG         ((uint16_t)0x1FU)
#define LDO1_VOUT_REG         ((uint16_t)0x23U)
#define LDO2_VOUT_REG         ((uint16_t)0x24U)
#define LDO3_VOUT_REG         ((uint16_t)0x25U)
#define LDO1_PG_WINDOW_REG    ((uint16_t)0x27U)
#define LDO2_PG_WINDOW_REG    ((uint16_t)0x28U)
#define LDO3_PG_WINDOW_REG    ((uint16_t)0x29U)
#define VCCA_VMON_CTRL_REG    ((uint16_t)0x2BU)
#define VCCA_PG_WINDOW_REG    ((uint16_t)0x2CU)
#define VMON1_PG_WINDOW_REG   ((uint16_t)0x2DU)
#define VMON1_PG_LEVEL_REG    ((uint16_t)0x2EU)
#define VMON2_PG_WINDOW_REG   ((uint16_t)0x2FU)
#define VMON2_PG_LEVEL_REG    ((uint16_t)0x30U)
#define RAIL_SEL_1_REG        ((uint16_t)0x41U)
#define RAIL_SEL_2_REG        ((uint16_t)0x42U)
#define RAIL_SEL_3_REG        ((uint16_t)0x43U)
#define BUCK_RESET_REG_REG    ((uint16_t)0x87U)
#define SPREAD_SPECTRUM_1_REG ((uint16_t)0x88U)

/* ========================================================================== */
/*                              Register Bit Fields                           */
/* ========================================================================== */

// BUCK1_CTRL, BUCK2_CTRL, BUCK3_CTRL, BUCK4_CTRL
#define BUCK_PLDN_SHIFT    (5U)
#define BUCK_VMON_EN_SHIFT (4U)
#define BUCK_FPWM_SHIFT    (1U)
#define BUCK_EN_SHIFT      (0U)
#define BUCK_PLDN_MASK     (0x01UL << BUCK_PLDN_SHIFT)
#define BUCK_VMON_EN_MASK  (0x01UL << BUCK_VMON_EN_SHIFT)
#define BUCK_FPWM_MASK     (0x01UL << BUCK_FPWM_SHIFT)
#define BUCK_EN_MASK       (0x01UL << BUCK_EN_SHIFT)

// BUCK1_CONF, BUCK2_CONF, BUCK3_CONF, BUCK4_CONF
#define BUCK_SLEW_RATE_SHIFT (0U)
#define BUCK_SLEW_RATE_MASK  (0x03U << BUCK_SLEW_RATE_SHIFT)

// BUCK1_VOUT
#define BUCK1_VSET_SHIFT (0U)
#define BUCK1_VSET_MASK  (0xFFU << BUCK1_VSET_SHIFT)

// BUCK2_VOUT, BUCK3_VOUT, BUCK4_VOUT
#define BUCK2_3_4_VSET_SHIFT (0U)
#define BUCK2_3_4_VSET_MASK  (0x7FU << BUCK2_3_4_VSET_SHIFT)

// BUCK1_PG_WINDOW, BUCK2_PG_WINDOW, BUCK3_PG_WINDOW, BUCK4_PG_WINDOW
#define BUCK_VMON_THR_SHIFT (0U)
#define BUCK_VMON_THR_MASK  (0x03U << BUCK_VMON_THR_SHIFT)

// LDO1_CTRL, LDO2_CTRL, LDO3_CTRL
#define LDO_DISCHARGE_EN_SHIFT (5U)
#define LDO_VMON_EN_SHIFT      (4U)
#define LDO_EN_SHIFT           (0U)
#define LDO_DISCHARGE_EN_MASK  (0x01UL << LDO_DISCHARGE_EN_SHIFT)
#define LDO_VMON_EN_MASK       (0x01UL << LDO_VMON_EN_SHIFT)
#define LDO_EN_MASK            (0x01UL << LDO_EN_SHIFT)

// LDO1_VOUT, LDO2_VOUT, LDO3_VOUT
#define LDO_BYP_CONFIG_SHIFT (7U)
#define LDO_VSET_SHIFT       (1U)
#define LDO_BYP_CONFIG_MASK  (0x01UL << LDO_BYP_CONFIG_SHIFT)
#define LDO_VSET_MASK        (0x3FU << LDO_VSET_SHIFT)

// LDO1_PG_WINDOW, LDO2_PG_WINDOW, LDO3_PG_WINDOW
#define LDO_VMON_THR_SHIFT (0U)
#define LDO_VMON_THR_MASK  (0x03U << LDO_VMON_THR_SHIFT)

// VCCA_VMON_CTRL
#define VMON_DEGLITCH_SEL_SHIFT (5U)
#define VMON2_EN_SHIFT          (3U)
#define VMON1_EN_SHIFT          (1U)
#define VCCA_VMON_EN_SHIFT      (0U)
#define VMON_DEGLITCH_SEL_MASK  (0x07U << VMON_DEGLITCH_SEL_SHIFT)
#define VMON2_EN_MASK           (0x01UL << VMON2_EN_SHIFT)
#define VMON1_EN_MASK           (0x01UL << VMON1_EN_SHIFT)
#define VCCA_VMON_EN_MASK       (0x01UL << VCCA_VMON_EN_SHIFT)

// VCCA_PG_WINDOW
#define VCCA_PG_SET_SHIFT   (6U)
#define VCCA_VMON_THR_SHIFT (0U)
#define VCCA_PG_SET_MASK    (0x01UL << VCCA_PG_SET_SHIFT)
#define VCCA_VMON_THR_MASK  (0x03U << VCCA_VMON_THR_SHIFT)

// VMON1_PG_WINDOW, VMON2_PG_WINDOW
#define VMON_THR_SHIFT (0U)
#define VMON_THR_MASK  (0x03U << VMON_THR_SHIFT)

// VMON1_PG_LEVEL
#define VMON1_PG_SET_SHIFT (0U)
#define VMON1_PG_SET_MASK  (0xFFU << VMON1_PG_SET_SHIFT)

// VMON2_PG_LEVEL
#define VMON2_PG_SET_SHIFT (0U)
#define VMON2_PG_SET_MASK  (0x7FU << VMON2_PG_SET_SHIFT)

// RAIL_SEL_1
#define BUCK4_GRP_SEL_SHIFT (6U)
#define BUCK3_GRP_SEL_SHIFT (4U)
#define BUCK2_GRP_SEL_SHIFT (2U)
#define BUCK1_GRP_SEL_SHIFT (0U)
#define BUCK4_GRP_SEL_MASK  (0x03U << BUCK4_GRP_SEL_SHIFT)
#define BUCK3_GRP_SEL_MASK  (0x03U << BUCK3_GRP_SEL_SHIFT)
#define BUCK2_GRP_SEL_MASK  (0x03U << BUCK2_GRP_SEL_SHIFT)
#define BUCK1_GRP_SEL_MASK  (0x03U << BUCK1_GRP_SEL_SHIFT)

// RAIL_SEL_2
#define LDO3_GRP_SEL_SHIFT (6U)
#define LDO2_GRP_SEL_SHIFT (4U)
#define LDO1_GRP_SEL_SHIFT (2U)
#define LDO3_GRP_SEL_MASK (0x03U << LDO3_GRP_SEL_SHIFT)
#define LDO2_GRP_SEL_MASK (0x03U << LDO2_GRP_SEL_SHIFT)
#define LDO1_GRP_SEL_MASK (0x03U << LDO1_GRP_SEL_SHIFT)

// RAIL_SEL_3
#define VMON2_GRP_SEL_SHIFT (6U)
#define VMON1_GRP_SEL_SHIFT (4U)
#define VCCA_GRP_SEL_SHIFT  (2U)
#define VMON2_GRP_SEL_MASK  (0x03U << VMON2_GRP_SEL_SHIFT)
#define VMON1_GRP_SEL_MASK  (0x03U << VMON1_GRP_SEL_SHIFT)
#define VCCA_GRP_SEL_MASK   (0x03U << VCCA_GRP_SEL_SHIFT)

// BUCK_RESET_REG
#define BUCK4_RESET_SHIFT (3U)
#define BUCK3_RESET_SHIFT (2U)
#define BUCK2_RESET_SHIFT (1U)
#define BUCK1_RESET_SHIFT (0U)
#define BUCK4_RESET_MASK  (0x01UL << BUCK4_RESET_SHIFT)
#define BUCK3_RESET_MASK  (0x01UL << BUCK3_RESET_SHIFT)
#define BUCK2_RESET_MASK  (0x01UL << BUCK2_RESET_SHIFT)
#define BUCK1_RESET_MASK  (0x01UL << BUCK1_RESET_SHIFT)

// SPREAD_SPECTRUM_1
#define SS_EN_SHIFT    (2U)
#define SS_DEPTH_SHIFT (0U)
#define SS_EN_MASK     (1UL << SS_EN_SHIFT)
#define SS_DEPTH_MASK  (1UL << SS_DEPTH_SHIFT)

#ifdef __cplusplus
}

#endif /* __cplusplus */
#endif /* REGMAP_POWER_H */
