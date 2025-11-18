/******************************************************************************
 * Copyright (c) 2025 Texas Instruments Incorporated - http://www.ti.com
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

#define BUCK1_CTRL_REGADDR        (0x04U)
#define BUCK1_CONF_REGADDR        (0x05U)
#define BUCK2_CTRL_REGADDR        (0x06U)
#define BUCK2_CONF_REGADDR        (0x07U)
#define BUCK3_CTRL_REGADDR        (0x08U)
#define BUCK3_CONF_REGADDR        (0x09U)
#define BUCK4_CTRL_REGADDR        (0x0AU)
#define BUCK4_CONF_REGADDR        (0x0BU)
#define BUCK1_VOUT_REGADDR        (0x0EU)
#define BUCK2_VOUT_REGADDR        (0x10U)
#define BUCK3_VOUT_REGADDR        (0x12U)
#define BUCK4_VOUT_REGADDR        (0x14U)
#define BUCK1_PG_WINDOW_REGADDR   (0x18U)
#define BUCK2_PG_WINDOW_REGADDR   (0x19U)
#define BUCK3_PG_WINDOW_REGADDR   (0x1AU)
#define BUCK4_PG_WINDOW_REGADDR   (0x1BU)
#define LDO1_CTRL_REGADDR         (0x1DU)
#define LDO2_CTRL_REGADDR         (0x1EU)
#define LDO3_CTRL_REGADDR         (0x1FU)
#define LDO1_VOUT_REGADDR         (0x23U)
#define LDO2_VOUT_REGADDR         (0x24U)
#define LDO3_VOUT_REGADDR         (0x25U)
#define LDO1_PG_WINDOW_REGADDR    (0x27U)
#define LDO2_PG_WINDOW_REGADDR    (0x28U)
#define LDO3_PG_WINDOW_REGADDR    (0x29U)
#define VCCA_VMON_CTRL_REGADDR    (0x2BU)
#define VCCA_PG_WINDOW_REGADDR    (0x2CU)
#define VMON1_PG_WINDOW_REGADDR   (0x2DU)
#define VMON1_PG_LEVEL_REGADDR    (0x2EU)
#define VMON2_PG_WINDOW_REGADDR   (0x2FU)
#define VMON2_PG_LEVEL_REGADDR    (0x30U)
#define RAIL_SEL_1_REGADDR        (0x41U)
#define RAIL_SEL_2_REGADDR        (0x42U)
#define RAIL_SEL_3_REGADDR        (0x43U)
#define BUCK_RESET_REG_REGADDR    (0x87U)
#define SPREAD_SPECTRUM_1_REGADDR (0x88U)

/* ========================================================================== */
/*                              Register Bit Fields                           */
/* ========================================================================== */

// BUCKx_CTRL
#define BUCK_PLDN_SHIFT    (5U)
#define BUCK_VMON_EN_SHIFT (4U)
#define BUCK_FPWM_SHIFT    (1U)
#define BUCK_EN_SHIFT      (0U)
#define BUCK_PLDN_MASK     (0x01U << BUCK_PLDN_SHIFT)
#define BUCK_VMON_EN_MASK  (0x01U << BUCK_VMON_EN_SHIFT)
#define BUCK_FPWM_MASK     (0x01U << BUCK_FPWM_SHIFT)
#define BUCK_EN_MASK       (0x01U << BUCK_EN_SHIFT)

// BUCKx_CONF
#define BUCK_SLEW_RATE_SHIFT (0U)
#define BUCK_SLEW_RATE_MASK  (0x03U << BUCK_SLEW_RATE_SHIFT)

// BUCK1_VOUT
#define BUCK1_VSET_SHIFT (0U)
#define BUCK1_VSET_MASK  (0xFFU << BUCK1_VSET_SHIFT)

// BUCK2_VOUT, BUCK3_VOUT, BUCK4_VOUT
#define BUCK2_3_4_VSET_SHIFT (0U)
#define BUCK2_3_4_VSET_MASK  (0x7FU < BUCK2_3_4_VSET_SHIFT)

// BUCKx_PG_WINDOW
#define BUCK_VMON_THR_SHIFT (0U)
#define BUCK_VMON_THR_MASK  (0x03U << BUCK_VMON_THR_SHIFT)

// LDO1_CTRL
#define _SHIFT (0U)
#define _MASK (0x00U << 0U)

// LDO2_CTRL
#define _SHIFT (0U)
#define _MASK (0x00U << 0U)

// LDO3_CTRL
#define _SHIFT (0U)
#define _MASK (0x00U << 0U)

// LDO1_VOUT
#define _SHIFT (0U)
#define _MASK (0x00U << 0U)

// LDO2_VOUT
#define _SHIFT (0U)
#define _MASK (0x00U << 0U)

// LDO3_VOUT
#define _SHIFT (0U)
#define _MASK (0x00U << 0U)

// LDO1_PG_WINDOW
#define _SHIFT (0U)
#define _MASK (0x00U << 0U)

// LDO2_PG_WINDOW
#define _SHIFT (0U)
#define _MASK (0x00U << 0U)

// LDO3_PG_WINDOW
#define _SHIFT (0U)
#define _MASK (0x00U << 0U)

// VCCA_VMON_CTRL
#define _SHIFT (0U)
#define _MASK (0x00U << 0U)

// VCCA_PG_WINDOW
#define _SHIFT (0U)
#define _MASK (0x00U << 0U)

// VMON1_PG_WINDOW
#define _SHIFT (0U)
#define _MASK (0x00U << 0U)

// VMON1_PG_LEVEL
#define _SHIFT (0U)
#define _MASK (0x00U << 0U)

// VMON2_PG_WINDOW
#define _SHIFT (0U)
#define _MASK (0x00U << 0U)

// VMON2_PG_LEVEL
#define _SHIFT (0U)
#define _MASK (0x00U << 0U)

// RAIL_SEL_1
#define _SHIFT (0U)
#define _MASK (0x00U << 0U)

// RAIL_SEL_2
#define _SHIFT (0U)
#define _MASK (0x00U << 0U)

// RAIL_SEL_3
#define _SHIFT (0U)
#define _MASK (0x00U << 0U)

// BUCK_RESET_REG
#define _SHIFT (0U)
#define _MASK (0x00U << 0U)

// SPREAD_SPECTRUM_1
#define _SHIFT (0U)
#define _MASK (0U << 0U)

#ifdef __cplusplus
}

#endif /* __cplusplus */
#endif /* REGMAP_POWER_H */
