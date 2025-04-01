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
#ifndef REGMAP_POWER_H
#define REGMAP_POWER_H

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                Include Files                               */
/* ========================================================================== */
#include <stdint.h>

// Power Control Register Definitions
#define BLOCK_EN_CTRL_REG             (0x06U)
#define BUCK1_VOUT_REG                (0x11U)
#define BUCK2_VOUT_REG                (0x12U)
#define BUCK3_VOUT_REG                (0x13U)
#define LDO_LS1_VMON1_PG_LEVEL_REG    (0x14U)
#define LS2_VMON2_PG_LEVEL_REG        (0x15U)
#define VCCA_PG_LEVEL_REG             (0x16U)
#define BUCK1_MON_CONF_REG            (0x17U)
#define BUCK2_MON_CONF_REG            (0x18U)
#define BUCK3_MON_CONF_REG            (0x19U)
#define LDO_LS1_VMON1_MON_CONF_REG    (0x1AU)
#define LS2_VMON2_MON_CONF_REG        (0x1BU)
#define CLK_CONF_REG                  (0x1CU) // SS_EN is this needed here?
#define FUNC_CONF_REG                 (0x1EU)
#define VCCA_MON_CONF_REG             (0x1FU)
#define BUCK_LDO_LS1_VMON1_DEGLIT_REG (0x20U)
#define BUCK1_SEQUENCE_REG            (0x21U)
#define BUCK2_SEQUENCE_REG            (0x22U)
#define BUCK3_SEQUENCE_REG            (0x23U)
#define LDO_LS1_VMON1_SEQUENCE_REG    (0x24U)
#define LS2_VMON2_SEQUENCE_REG        (0x25U)
#define GPO_SEQUENCE_REG              (0x26U)
#define NRSTOUT_SEQUENCE_REG          (0x27U)
#define REG_OV_CONF_REG               (0x28U)
#define REG_UV_CONF_REG               (0x29U)
#define REG_SC_CONF_REG               (0x2AU)
#define VCCA_LS2_VMON2_OV_CONF_REG    (0x2BU)
#define VCCA_LS2_VMON2_UV_CONF_REG    (0x2CU)

// Power Status Register Definitions
#define STAT_BUCK_12_REG              (0x53U)
#define STAT_BUCK3_LDO_LS1_VMON1_REG  (0x54U)
#define STAT_LS2_VMON2_REG            (0x55U)
#define STAT_VCCA_REG                 (0x56U)
#define STAT_STARTUP_REG              (0x57U)
#define STAT_MISC_REG                 (0x58U)

// LDO_LS1_VMON1_PG_LEVEL_REG Bitfields
#define LDO_LS1_VMON1_PG_SET_SHIFT    (0U)
#define LDO_LS1_BYP_CONFIG_SHIFT      (7U)
#define LDO_LS1_VMON1_PG_SET_MASK     (0x7FU << LDO_LS1_VMON1_PG_SET_SHIFT)
#define LDO_LS1_BYP_CONFIG_MASK       (0x1U << LDO_LS1_BYP_CONFIG_SHIFT)

// BUCK1_MON_CONF_REG, BUCK2_MON_CONF_REG, BUCK3_MON_CONF_REG Bitfields
#define BUCKx_ILIM_SHIFT              (0U)
#define BUCKx_RV_CONF_SHIFT           (3U)
#define BUCKx_OV_THR_SHIFT            (4U)
#define BUCKx_UV_THR_SHIFT            (6U)
#define BUCKx_ILIM_MASK               (0x7U << BUCKx_ILIM_SHIFT)
#define BUCKx_RV_CONF_MASK            (0x1U << BUCKx_RV_CONF_SHIFT)
#define BUCKx_OV_THR_MASK             (0x3U << BUCKx_OV_THR_SHIFT)
#define BUCKx_UV_THR_MASK             (0x3U << BUCKx_UV_THR_SHIFT)

// LDO_LS1_VMON1_MON_CONF_REG, LS2_VMON2_MON_CONF_REG Bitfields
#define LDO_LS12_VMON_RV_CONF_SHIFT   (1U)
#define LDO_LS12_VMON_EN_SHIFT        (2U)
#define LDO_LS12_VMON_DIS_PD_SHIFT    (3U)
#define LDO_LS12_VMON_OV_THR_SHIFT    (4U)
#define LDO_LS12_VMON_UV_THR_SHIFT    (6U)
#define LDO_LS12_VMON_RV_CONF_MASK    (0x1U << LDO_LS12_VMON_RV_CONF_SHIFT)
#define LDO_LS12_VMON_EN_MASK         (0x1U << LDO_LS12_VMON_EN_SHIFT)
#define LDO_LS12_VMON_DIS_PD_MASK     (0x1U << LDO_LS12_VMON_DIS_PD_SHIFT)
#define LDO_LS12_VMON_OV_THR_MASK     (0x3U << LDO_LS12_VMON_OV_THR_SHIFT)
#define LDO_LS12_VMON_UV_THR_MASK     (0x3U << LDO_LS12_VMON_UV_THR_SHIFT)

// FUNC_CONF_REG Bitfields
#define LDO_LS1_VMON1_SEL_SHIFT       (0U)
#define LDO_LS1_LSW_CONFIG_SHIFT      (1U)
#define LS2_VMON2_GPO_SEL_SHIFT       (4U)
#define NINT_GPO_SEL_SHIFT            (6U)
#define LDO_LS1_VMON1_SEL_MASK        (0x1U << LDO_LS1_VMON1_SEL_SHIFT)
#define LDO_LS1_LSW_CONFIG_MASK       (0x1U << LDO_LS1_LSW_CONFIG_SHIFT)
#define LS2_VMON2_GPO_SEL_MASK        (0x3U << LS2_VMON2_GPO_SEL_SHIFT)
#define NINT_GPO_SEL_MASK             (0x1U << NINT_GPO_SEL_SHIFT)

// All sequence register bitfields, including:
// - BUCK1_SEQUENCE_REG
// - BUCK2_SEQUENCE_REG
// - BUCK3_SEQUENCE_REG
// - LDO_LS1_VMON1_SEQUENCE_REG
// - LS2_VMON2_SEQUENCE_REG
// - LS2_VMON2_SEQUENCE_REG
// - GPO_SEQUENCE_REG
// - NRSTOUT_SEQUENCE_REG
#define STARTUP_DELAY_SHIFT           (0U)
#define SHUTDOWN_DELAY_SHIFT          (4U)
#define STARTUP_DELAY_MASK            (0x7U << STARTUP_DELAY_SHIFT)
#define SHUTDOWN_DELAY_MASK           (0x7U << SHUTDOWN_DELAY_SHIFT)

// REG_OV_CONF_REG Bitfields
//
// NOTE: These select fields use the same ordering as the definition of the
// PWR_RSRC types, the specific offset in the register can be calculated from
// the requested PWR_RSRC
#define BUCK1_OV_SEL_SHIFT            (0U)
#define BUCK2_OV_SEL_SHIFT            (2U)
#define BUCK3_OV_SEL_SHIFT            (4U)
#define LDO_LS1_VMON1_OV_SEL_SHIFT    (6U)
#define BUCK1_OV_SEL_MASK             (0x3U << BUCK1_OV_SEL_SHIFT)
#define BUCK2_OV_SEL_MASK             (0x3U << BUCK2_OV_SEL_SHIFT)
#define BUCK3_OV_SEL_MASK             (0x3U << BUCK3_OV_SEL_SHIFT)
#define LDO_LS1_VMON1_OV_SEL_MASK     (0x3U << LDO_LS1_VMON1_OV_SEL_SHIFT)

// VCCA_LS2_VMON2_OV_CONF_REG Bitfields
#define VCCA_OV_SEL_SHIFT             (0U)
#define LS2_VMON2_OV_SEL_SHIFT        (4U)
#define PWRERR_OSD_SEL_SHIFT          (7U)
#define VCCA_OV_SEL_MASK              (0x3U << VCCA_UV_SEL_SHIFT)
#define LS2_VMON2_OV_SEL_MASK         (0x3U << LS2_VMON2_UV_SEL_SHIFT)
#define PWRERR_OSD_SEL_MASK           (0x1U << PWRERR_OSD_SEL_SHIFT)

// REG_UV_CONF_REG Bitfields
//
// NOTE: These select fields use the same ordering as the definition of the
// PWR_RSRC types, the specific offset in the register can be calculated from
// the requested PWR_RSRC
#define BUCK1_UV_SEL_SHIFT            (0U)
#define BUCK2_UV_SEL_SHIFT            (2U)
#define BUCK3_UV_SEL_SHIFT            (4U)
#define LDO_LS1_VMON1_UV_SEL_SHIFT    (6U)
#define BUCK1_UV_SEL_MASK             (0x3U << BUCK1_UV_SEL_SHIFT)
#define BUCK2_UV_SEL_MASK             (0x3U << BUCK2_UV_SEL_SHIFT)
#define BUCK3_UV_SEL_MASK             (0x3U << BUCK3_UV_SEL_SHIFT)
#define LDO_LS1_VMON1_UV_SEL_MASK     (0x3U << LDO_LS1_VMON1_UV_SEL_SHIFT)

// VCCA_LS2_VMON2_UV_CONF_REG Bitfields
#define VCCA_UV_SEL_SHIFT             (0U)
#define LS2_VMON2_UV_SEL_SHIFT        (4U)
#define LS2_VMON2_SC_SEL_SHIFT        (6U)
#define VCCA_UV_SEL_MASK              (0x3U << VCCA_UV_SEL_SHIFT)
#define LS2_VMON2_UV_SEL_MASK         (0x3U << LS2_VMON2_UV_SEL_SHIFT)
#define LS2_VMON2_SC_SEL_MASK         (0x3U << LS2_VMON2_SC_SEL_SHIFT)

// REG_SC_CONF_REG Bitfields
//
// NOTE: These select fields use the same ordering as the definition of the
// PWR_RSRC types, the specific offset in the register can be calculated from
// the requested PWR_RSRC
#define BUCK1_SC_SEL_SHIFT            (0U)
#define BUCK2_SC_SEL_SHIFT            (2U)
#define BUCK3_SC_SEL_SHIFT            (4U)
#define LDO_LS1_VMON1_SC_SEL_SHIFT    (6U)
#define BUCK1_SC_SEL_MASK             (0x3U << BUCK1_SC_SEL_SHIFT)
#define BUCK2_SC_SEL_MASK             (0x3U << BUCK2_SC_SEL_SHIFT)
#define BUCK3_SC_SEL_MASK             (0x3U << BUCK3_SC_SEL_SHIFT)
#define LDO_LS1_VMON1_SC_SEL_MASK     (0x3U << LDO_LS1_VMON1_SC_SEL_SHIFT)

// LDO_LS1_VMON1_MON_CONF_REG Bitfields
#define LDO_LS1_VMON1_RV_CONF_SHIFT   (1U)
#define LDO_LS1_VMON1_VMON_EN_SHIFT   (2U)
#define LDO_LS1_VMON1_DIS_PD_SHIFT    (3U)
#define LDO_LS1_VMON1_OV_THR_SHIFT    (4U)
#define LDO_LS1_VMON1_UV_THR_SHIFT    (5U)
#define LDO_LS1_VMON1_OV_THR_MASK     (0x3U << LDO_LS1_VMON1_OV_THR_SHIFT)
#define LDO_LS1_VMON1_UV_THR_MASK     (0x3U << LDO_LS1_VMON1_UV_THR_SHIFT)

// LS2_VMON2_MON_CONF_REG Bitfields
#define LS2_VMON2_RV_CONF_SHIFT       (1U)
#define LS2_VMON2_VMON_EN_SHIFT       (2U)
#define LS2_VMON2_DIS_PD_SHIFT        (3U)
#define LS2_VMON2_OV_THR_SHIFT        (4U)
#define LS2_VMON2_UV_THR_SHIFT        (5U)
#define LS2_VMON2_OV_THR_MASK         (0x3U << LS2_VMON2_OV_THR_SHIFT)
#define LS2_VMON2_UV_THR_MASK         (0x3U << LS2_VMON2_UV_THR_SHIFT)

// VCCA_MON_CONF_REG Bitfields
#define LS2_VMON2_DEGLITCH_SEL_SHIFT  (0U)
#define VMONIN_DEGLITCH_SEL_SHIFT     (2U)
#define VCCA_OV_THR_SHIFT             (4U)
#define VCCA_UV_THR_SHIFT             (5U)
#define LS2_VMON2_DEGLITCH_SEL_MASK   (0x3U << LS2_VMON2_DEGLITCH_SEL_SHIFT)
#define VMONIN_DEGLITCH_SEL_MASK      (0x3U << VMONIN_DEGLITCH_SEL_SHIFT)
#define VCCA_OV_THR_MASK              (0x3U << VCCA_OV_THR_SHIFT)
#define VCCA_UV_THR_MASK              (0x3U << VCCA_UV_THR_SHIFT)

// Bitfields for all *_SEQUENCE_REG
// - BUCK1_SEQUENCE_REG
// - BUCK2_SEQUENCE_REG
// - BUCK3_SEQUENCE_REG
// - LDO_LS1_VMON1_SEQUENCE_REG
// - LS2_VMON2_SEQUENCE_REG
// - GPO_SEQUENCE_REG
// - NRSTOUT_SEQUENCE_REG
#define SEQ_STARTUP_DELAY_SHIFT       (0U)
#define SEQ_SHUTDOWN_DELAY_SHIFT      (4U)
#define SEQ_STARTUP_DELAY_MASK        (0xFU << SEQ_STARTUP_DELAY_SHIFT)
#define SEQ_SHUTDOWN_DELAY_MASK       (0xFU << SEQ_SHUTDOWN_DELAY_SHIFT)

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif // REGMAP_POWER_H
