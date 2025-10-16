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
#ifndef __PMIC_REGMAP_POWER_H__
#define __PMIC_REGMAP_POWER_H__

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/** @brief Power module register addressses */
#define BUCK_BST_CFG_REG        (0x1BU)
#define LDO1_CFG_REG            (0x1EU)
#define LDO2_CFG_REG            (0x1FU)
#define LDO3_CFG_REG            (0x20U)
#define LDO4_CFG_REG            (0x21U)
#define PLDO1_CFG_REG           (0x22U)
#define PLDO2_CFG_REG           (0x23U)
#define PLDO_CFG_REG            (0x24U)
#define LDO_PGOOD_CFG_REG       (0x25U)
#define LDO_CTRL_REG            (0x26U)
#define PLDO_EN_OUT_CTRL_REG    (0x27U)
#define LDO_DSCG_CFG_REG        (0x28U)
#define VMON_TH_CFG1_REG        (0x29U)
#define VMON_TH_CFG2_REG        (0x2AU)
#define VMON_TH_CFG3_REG        (0x2BU)
#define LP_VMON_CTRL_REG        (0x2CU)
#define LP_CFG_REG              (0x2FU)
#define VMON_DGL_CFG1_REG       (0x30U)
#define VMON_DGL_CFG2_REG       (0x31U)
#define VMON_DGL_CFG3_REG       (0x32U)
#define EXT_VMON_CFG_CTRL_REG   (0x33U)
#define DCDC_STAT_REG           (0x5AU)
#define VMON_DEV_STAT_REG       (0x5BU)
#define VMON_LDO_STAT_REG       (0x5CU)
#define VMON_PLDO_STAT_REG      (0x5EU)
#define EXT_VMON_STAT_REG       (0x5FU)
#define ILIM_CFG_REG            (0x60U)
#define ILIM_DGL_CFG_REG        (0x61U)
#define ILIM_STAT_REG           (0x62U)
#define THERMAL_STAT1_REG       (0x63U)
#define THERMAL_STAT2_REG       (0x64U)

/** @brief BUCK_BST_CFG - Buck Boost Configuration */
#define BB_LVL_CFG_SHIFT        (0U)
#define BB_STBY_LVL_CFG_SHIFT   (3U)
#define BB_SS_EN_SHIFT          (4U)
#define BB_PGOOD_CFG_SHIFT      (5U)
#define BB_LVL_CFG_MASK         (7U << BB_LVL_CFG_SHIFT)
#define BB_STBY_LVL_CFG_MASK    (1U << BB_STBY_LVL_CFG_SHIFT)
#define BB_SS_EN_MASK           (1U << BB_SS_EN_SHIFT)
#define BB_PGOOD_CFG_MASK       (1U << BB_PGOOD_CFG_SHIFT)

/** @brief LDOx_CFG - LDOx Configuration */
#define LDO_LVL_CFG_SHIFT       (0U)
#define LDO_ILIM_LVL_CFG_SHIFT  (5U)
#define LDO_RT_CFG_SHIFT        (7U)
#define LDO_LVL_CFG_MASK        (0x1FU << LDO_LVL_CFG_SHIFT)
#define LDO_ILIM_LVL_CFG_MASK   (3U << LDO_ILIM_LVL_CFG_SHIFT)
#define LDO_RT_CFG_MASK         (1U << LDO_RT_CFG_SHIFT)

/** @brief PLDOx_CFG - PLDOx Configuration */
#define PLDO_LVL_CFG_SHIFT          (0U)
#define PLDO_ILIM_LVL_CFG_SHIFT     (5U)
#define PLDO_MODE_SHIFT             (7U)
#define PLDO_LVL_CFG_MASK           (0x1FU << PLDO_LVL_CFG_SHIFT)
#define PLDO_ILIM_LVL_CFG_MASK      (3U << PLDO_ILIM_LVL_CFG_SHIFT)
#define PLDO_MODE_MASK              (1U << PLDO_MODE_SHIFT)

/** @brief PLDO_CFG - PLDO Configuration */
#define PLDO1_RT_CFG_SHIFT  (0U)
#define PLDO2_RT_CFG_SHIFT  (1U)
#define VTRACK_RNG_SHIFT    (2U)
#define PLDO1_RT_CFG_MASK   (1U << PLDO1_RT_CFG_SHIFT)
#define PLDO2_RT_CFG_MASK   (1U << PLDO2_RT_CFG_SHIFT)
#define VTRACK_RNG_MASK     (1U << VTRACK_RNG_SHIFT)

/** @brief LDO_PGOOD_CFG - LDO and Power-Good Configuration */
#define LDO1_PGOOD_CFG_SHIFT    (0U)
#define LDO2_PGOOD_CFG_SHIFT    (1U)
#define LDO3_PGOOD_CFG_SHIFT    (2U)
#define LDO4_PGOOD_CFG_SHIFT    (3U)
#define PLDO1_PGOOD_CFG_SHIFT   (4U)
#define PLDO2_PGOOD_CFG_SHIFT   (5U)
#define LDO1_PGOOD_CFG_MASK     (1U << LDO1_PGOOD_CFG_SHIFT)
#define LDO2_PGOOD_CFG_MASK     (1U << LDO2_PGOOD_CFG_SHIFT)
#define LDO3_PGOOD_CFG_MASK     (1U << LDO3_PGOOD_CFG_SHIFT)
#define LDO4_PGOOD_CFG_MASK     (1U << LDO4_PGOOD_CFG_SHIFT)
#define PLDO1_PGOOD_CFG_MASK    (1U << PLDO1_PGOOD_CFG_SHIFT)
#define PLDO2_PGOOD_CFG_MASK    (1U << PLDO2_PGOOD_CFG_SHIFT)

/** @brief LDO_CTRL - LDO Control */
#define LDO1_CTRL_SHIFT (0U)
#define LDO2_CTRL_SHIFT (2U)
#define LDO3_CTRL_SHIFT (4U)
#define LDO4_CTRL_SHIFT (6U)
#define LDO1_CTRL_MASK  (3U << LDO1_CTRL_SHIFT)
#define LDO2_CTRL_MASK  (3U << LDO2_CTRL_SHIFT)
#define LDO3_CTRL_MASK  (3U << LDO3_CTRL_SHIFT)
#define LDO4_CTRL_MASK  (3U << LDO4_CTRL_SHIFT)

/** @brief PLDO_EN_OUT_CTRL - PLDO Enable Output Control */
#define PLDO1_CTRL_SHIFT    (0U)
#define PLDO2_CTRL_SHIFT    (2U)
#define EN_OUT_CTRL_SHIFT   (5U)
#define PGOOD_CTRL_SHIFT    (7U)
#define PLDO1_CTRL_MASK     (3U << PLDO1_CTRL_SHIFT)
#define PLDO2_CTRL_MASK     (7U << PLDO2_CTRL_SHIFT)
#define EN_OUT_CTRL_MASK    (3U << EN_OUT_CTRL_SHIFT)
#define PGOOD_CTRL_MASK     (1U << PGOOD_CTRL_SHIFT)

/** @brief LDO_DSCG_CFG - LDO Discharge Configuration */
#define LDO1_DSCG_DIS_SHIFT     (0U)
#define LDO2_DSCG_DIS_SHIFT     (1U)
#define LDO3_DSCG_DIS_SHIFT     (2U)
#define LDO4_DSCG_DIS_SHIFT     (3U)
#define PLDO1_DSCG_DIS_SHIFT    (4U)
#define PLDO2_DSCG_DIS_SHIFT    (5U)
#define EN_OUT2_CTRL_SHIFT      (6U)
#define LDO1_DSCG_DIS_MASK      (1U << LDO1_DSCG_DIS_SHIFT)
#define LDO2_DSCG_DIS_MASK      (1U << LDO2_DSCG_DIS_SHIFT)
#define LDO3_DSCG_DIS_MASK      (1U << LDO3_DSCG_DIS_SHIFT)
#define LDO4_DSCG_DIS_MASK      (1U << LDO4_DSCG_DIS_SHIFT)
#define PLDO1_DSCG_DIS_MASK     (1U << PLDO1_DSCG_DIS_SHIFT)
#define PLDO2_DSCG_DIS_MASK     (1U << PLDO2_DSCG_DIS_SHIFT)
#define EN_OUT2_CTRL_MASK       (3U << EN_OUT2_CTRL_SHIFT)

/** @brief VMON_TH_CFG1 - VMON Threshold Configuration 1 */
#define LDO1_VMON_TH_SHIFT  (0U)
#define LDO2_VMON_TH_SHIFT  (2U)
#define LDO3_VMON_TH_SHIFT  (4U)
#define LDO4_VMON_TH_SHIFT  (6U)
#define LDO1_VMON_TH_MASK   (3U << LDO1_VMON_TH_SHIFT)
#define LDO2_VMON_TH_MASK   (3U << LDO2_VMON_TH_SHIFT)
#define LDO3_VMON_TH_MASK   (3U << LDO3_VMON_TH_SHIFT)
#define LDO4_VMON_TH_MASK   (3U << LDO4_VMON_TH_SHIFT)

/** @brief VMON_TH_CFG2 - VMON Threshold Configuration 2 */
#define PLDO1_VMON_TH_SHIFT (0U)
#define PLDO2_VMON_TH_SHIFT (2U)
#define EXT_VMON1_TH_SHIFT  (4U)
#define PLDO1_VMON_TH_MASK  (3U << PLDO1_VMON_TH_SHIFT)
#define PLDO2_VMON_TH_MASK  (3U << PLDO2_VMON_TH_SHIFT)
#define EXT_VMON1_TH_MASK   (7U << EXT_VMON1_TH_SHIFT)

/** @brief VMON_TH_CFG3 - VMON Threshold Configuration 3 */
#define EXT_VMON2_TH_SHIFT      (0U)
#define BB_VMON_TH_SHIFT        (3U)
#define BB_BST_TMO_CFG_SHIFT    (5U)
#define EXT_VMON2_TH_MASK       (7U << EXT_VMON2_TH_SHIFT)
#define BB_VMON_TH_MASK         (3U << BB_VMON_TH_SHIFT)
#define BB_BST_TMO_CFG_MASK     (3U << BB_BST_TMO_CFG_SHIFT)

/** @brief LP_VMON_CTRL - Low Power VMON Control */
#define LP_LDO1_VMON_CTRL_SHIFT     (0U)
#define LP_LDO2_VMON_CTRL_SHIFT     (1U)
#define LP_LDO3_VMON_CTRL_SHIFT     (2U)
#define LP_LDO4_VMON_CTRL_SHIFT     (3U)
#define LP_PLDO1_VMON_CTRL_SHIFT    (4U)
#define LP_PLDO2_VMON_CTRL_SHIFT    (5U)
#define LP_EXT_VMON1_CTRL_SHIFT     (6U)
#define LP_EXT_VMON2_CTRL_SHIFT     (7U)
#define LP_LDO1_VMON_CTRL_MASK      (1U << LP_LDO1_VMON_CTRL_SHIFT)
#define LP_LDO2_VMON_CTRL_MASK      (1U << LP_LDO2_VMON_CTRL_SHIFT)
#define LP_LDO3_VMON_CTRL_MASK      (1U << LP_LDO3_VMON_CTRL_SHIFT)
#define LP_LDO4_VMON_CTRL_MASK      (1U << LP_LDO4_VMON_CTRL_SHIFT)
#define LP_PLDO1_VMON_CTRL_MASK     (1U << LP_PLDO1_VMON_CTRL_SHIFT)
#define LP_PLDO2_VMON_CTRL_MASK     (1U << LP_PLDO2_VMON_CTRL_SHIFT)
#define LP_EXT_VMON1_CTRL_MASK      (1U << LP_EXT_VMON1_CTRL_SHIFT)
#define LP_EXT_VMON2_CTRL_MASK      (1U << LP_EXT_VMON2_CTRL_SHIFT)

/** @brief LP_CFG - Low Power Configuration */
#define LP_VMON_PER_CFG_SHIFT   (0U)
#define LP_TSD_PER_CFG_SHIFT    (1U)
#define LP_BB_VMON_CTRL_SHIFT   (6U)
#define LP_BB_OVP_CTRL_SHIFT    (7U)
#define LP_VMON_PER_CFG_MASK    (1U << LP_VMON_PER_CFG_SHIFT)
#define LP_TSD_PER_CFG_MASK     (1U << LP_TSD_PER_CFG_SHIFT)
#define LP_BB_VMON_CTRL_MASK    (1U << LP_BB_VMON_CTRL_SHIFT)
#define LP_BB_OVP_CTRL_MASK     (1U << LP_BB_OVP_CTRL_SHIFT)

/** @brief VMON_DGL_CFG1 - VMON Deglitch Configuration 1 */
#define BB_VMON_DGL_SHIFT   (0U)
#define VBAT_VMON_DGL_SHIFT (7U)
#define BB_VMON_DGL_MASK    (3U << BB_VMON_DGL_SHIFT)
#define VBAT_VMON_DGL_MASK  (1U << VBAT_VMON_DGL_SHIFT)

/** @brief VMON_DGL_CFG2 - VMON Deglitch Configuration 2 */
#define LDO1_VMON_DGL_SHIFT (0U)
#define LDO2_VMON_DGL_SHIFT (2U)
#define LDO3_VMON_DGL_SHIFT (4U)
#define LDO4_VMON_DGL_SHIFT (6U)
#define LDO1_VMON_DGL_MASK  (3U << LDO1_VMON_DGL_SHIFT)
#define LDO2_VMON_DGL_MASK  (3U << LDO2_VMON_DGL_SHIFT)
#define LDO3_VMON_DGL_MASK  (3U << LDO3_VMON_DGL_SHIFT)
#define LDO4_VMON_DGL_MASK  (3U << LDO4_VMON_DGL_SHIFT)

/** @brief VMON_DGL_CFG3 - VMON Deglitch Configuration 3 */
#define PLDO1_VMON_DGL_SHIFT    (0U)
#define PLDO2_VMON_DGL_SHIFT    (2U)
#define EXT_VMON2_DGL_SHIFT     (4U)
#define EXT_VMON1_DGL_SHIFT     (6U)
#define PLDO1_VMON_DGL_MASK     (3U << PLDO1_VMON_DGL_SHIFT)
#define PLDO2_VMON_DGL_MASK     (3U << PLDO2_VMON_DGL_SHIFT)
#define EXT_VMON2_DGL_MASK      (3U << EXT_VMON2_DGL_SHIFT)
#define EXT_VMON1_DGL_MASK      (3U << EXT_VMON1_DGL_SHIFT)

/** @brief EXT_VMON_CFG_CTRL - External VMON Configuration Control */
#define EXT_VMON1_CTRL_SHIFT        (0U)
#define EXT_VMON2_CTRL_SHIFT        (2U)
#define EXT_VMON1_PGOOD_CFG_SHIFT   (6U)
#define EXT_VMON2_PGOOD_CFG_SHIFT   (7U)
#define EXT_VMON1_CTRL_MASK         (3U << EXT_VMON1_CTRL_SHIFT)
#define EXT_VMON2_CTRL_MASK         (3U << EXT_VMON2_CTRL_SHIFT)
#define EXT_VMON1_PGOOD_CFG_MASK    (1U << EXT_VMON1_PGOOD_CFG_SHIFT)
#define EXT_VMON2_PGOOD_CFG_MASK    (1U << EXT_VMON2_PGOOD_CFG_SHIFT)

/** @brief DCDC_STAT - DC-DC Status */
#define BB_UV_ERR_SHIFT     (0U)
#define BB_OV_ERR_SHIFT     (1U)
#define BB_BST_SHIFT        (2U)
#define BB_ILIM_LVL_SHIFT   (3U)
#define BB_LITE_SHIFT       (4U)
#define BB_UV_ERR_MASK      (1U << BB_UV_ERR_SHIFT)
#define BB_OV_ERR_MASK      (1U << BB_OV_ERR_SHIFT)
#define BB_BST_MASK         (1U << BB_BST_SHIFT)
#define BB_ILIM_LVL_MASK    (1U << BB_ILIM_LVL_SHIFT)
#define BB_LITE_MASK        (1U << BB_LITE_SHIFT)

/** @brief VMON_DEV_STAT - VMON Device Status */
#define VBAT_UV_ERR_SHIFT   (0U)
#define VBAT_OV_ERR_SHIFT   (1U)
#define VBAT_UV_ERR_MASK    (1U << VBAT_UV_ERR_SHIFT)
#define VBAT_OV_ERR_MASK    (1U << VBAT_OV_ERR_SHIFT)

/** @brief VMON_LDO_STAT - VMON LDO Status */
#define LDO1_UV_ERR_SHIFT   (0U)
#define LDO1_OV_ERR_SHIFT   (1U)
#define LDO2_UV_ERR_SHIFT   (2U)
#define LDO2_OV_ERR_SHIFT   (3U)
#define LDO3_UV_ERR_SHIFT   (4U)
#define LDO3_OV_ERR_SHIFT   (5U)
#define LDO4_UV_ERR_SHIFT   (6U)
#define LDO4_OV_ERR_SHIFT   (7U)
#define LDO1_UV_ERR_MASK    (1U << LDO1_UV_ERR_SHIFT)
#define LDO1_OV_ERR_MASK    (1U << LDO1_OV_ERR_SHIFT)
#define LDO2_UV_ERR_MASK    (1U << LDO2_UV_ERR_SHIFT)
#define LDO2_OV_ERR_MASK    (1U << LDO2_OV_ERR_SHIFT)
#define LDO3_UV_ERR_MASK    (1U << LDO3_UV_ERR_SHIFT)
#define LDO3_OV_ERR_MASK    (1U << LDO3_OV_ERR_SHIFT)
#define LDO4_UV_ERR_MASK    (1U << LDO4_UV_ERR_SHIFT)
#define LDO4_OV_ERR_MASK    (1U << LDO4_OV_ERR_SHIFT)

/** @brief VMON_PLDO_STAT - VMON PLDO Status */
#define PLDO1_UV_ERR_SHIFT  (0U)
#define PLDO1_OV_ERR_SHIFT  (1U)
#define PLDO2_UV_ERR_SHIFT  (2U)
#define PLDO2_OV_ERR_SHIFT  (3U)
#define PLDO1_UV_ERR_MASK   (1U << PLDO1_UV_ERR_SHIFT)
#define PLDO1_OV_ERR_MASK   (1U << PLDO1_OV_ERR_SHIFT)
#define PLDO2_UV_ERR_MASK   (1U << PLDO2_UV_ERR_SHIFT)
#define PLDO2_OV_ERR_MASK   (1U << PLDO2_OV_ERR_SHIFT)

/** @brief EXT_VMON_STAT - External VMON Status */
#define EXT_VMON1_UV_ERR_SHIFT  (0U)
#define EXT_VMON1_OV_ERR_SHIFT  (1U)
#define EXT_VMON2_UV_ERR_SHIFT  (2U)
#define EXT_VMON2_OV_ERR_SHIFT  (3U)
#define EXT_VMON1_UV_ERR_MASK   (1U << EXT_VMON1_UV_ERR_SHIFT)
#define EXT_VMON1_OV_ERR_MASK   (1U << EXT_VMON1_OV_ERR_SHIFT)
#define EXT_VMON2_UV_ERR_MASK   (1U << EXT_VMON2_UV_ERR_SHIFT)
#define EXT_VMON2_OV_ERR_MASK   (1U << EXT_VMON2_OV_ERR_SHIFT)

/** @brief ILIM_CFG - Current Limit Configuration */
#define LDO1_ILIM_CFG_SHIFT     (0U)
#define LDO2_ILIM_CFG_SHIFT     (1U)
#define LDO3_ILIM_CFG_SHIFT     (2U)
#define LDO4_ILIM_CFG_SHIFT     (3U)
#define PLDO1_ILIM_CFG_SHIFT    (4U)
#define PLDO2_ILIM_CFG_SHIFT    (5U)
#define LDO1_ILIM_CFG_MASK      (1U << LDO1_ILIM_CFG_SHIFT)
#define LDO2_ILIM_CFG_MASK      (1U << LDO2_ILIM_CFG_SHIFT)
#define LDO3_ILIM_CFG_MASK      (1U << LDO3_ILIM_CFG_SHIFT)
#define LDO4_ILIM_CFG_MASK      (1U << LDO4_ILIM_CFG_SHIFT)
#define PLDO1_ILIM_CFG_MASK     (1U << PLDO1_ILIM_CFG_SHIFT)
#define PLDO2_ILIM_CFG_MASK     (1U << PLDO2_ILIM_CFG_SHIFT)

/** @brief ILIM_DGL_CFG - Current Limit Deglitch Configuration */
#define LDO1_ILIM_DGL_CFG_SHIFT     (0U)
#define LDO2_ILIM_DGL_CFG_SHIFT     (1U)
#define LDO3_ILIM_DGL_CFG_SHIFT     (2U)
#define LDO4_ILIM_DGL_CFG_SHIFT     (3U)
#define PLDO1_ILIM_DGL_CFG_SHIFT    (4U)
#define PLDO2_ILIM_DGL_CFG_SHIFT    (5U)
#define LDO1_ILIM_DGL_CFG_MASK      (1U << LDO1_ILIM_DGL_CFG_SHIFT)
#define LDO2_ILIM_DGL_CFG_MASK      (1U << LDO2_ILIM_DGL_CFG_SHIFT)
#define LDO3_ILIM_DGL_CFG_MASK      (1U << LDO3_ILIM_DGL_CFG_SHIFT)
#define LDO4_ILIM_DGL_CFG_MASK      (1U << LDO4_ILIM_DGL_CFG_SHIFT)
#define PLDO1_ILIM_DGL_CFG_MASK     (1U << PLDO1_ILIM_DGL_CFG_SHIFT)
#define PLDO2_ILIM_DGL_CFG_MASK     (1U << PLDO2_ILIM_DGL_CFG_SHIFT)

/** @brief ILIM_STAT - Current Limit Status */
#define LDO1_ILIM_ERR_SHIFT     (0U)
#define LDO2_ILIM_ERR_SHIFT     (1U)
#define LDO3_ILIM_ERR_SHIFT     (2U)
#define LDO4_ILIM_ERR_SHIFT     (3U)
#define PLDO1_ILIM_ERR_SHIFT    (4U)
#define PLDO2_ILIM_ERR_SHIFT    (5U)
#define BB_AVG_ILIM_ERR_SHIFT   (6U)
#define LDO1_ILIM_ERR_MASK      (1U << LDO1_ILIM_ERR_SHIFT)
#define LDO2_ILIM_ERR_MASK      (1U << LDO2_ILIM_ERR_SHIFT)
#define LDO3_ILIM_ERR_MASK      (1U << LDO3_ILIM_ERR_SHIFT)
#define LDO4_ILIM_ERR_MASK      (1U << LDO4_ILIM_ERR_SHIFT)
#define PLDO1_ILIM_ERR_MASK     (1U << PLDO1_ILIM_ERR_SHIFT)
#define PLDO2_ILIM_ERR_MASK     (1U << PLDO2_ILIM_ERR_SHIFT)
#define BB_AVG_ILIM_ERR_MASK    (1U << BB_AVG_ILIM_ERR_SHIFT)

/** @brief THERMAL_STAT1 - Thermal Status 1 */
#define LDO1_T_PRE_ERR_SHIFT    (0U)
#define LDO1_TSD_ERR_SHIFT      (1U)
#define LDO2_T_PRE_ERR_SHIFT    (2U)
#define LDO2_TSD_ERR_SHIFT      (3U)
#define LDO3_T_PRE_ERR_SHIFT    (4U)
#define LDO3_TSD_ERR_SHIFT      (5U)
#define LDO4_T_PRE_ERR_SHIFT    (6U)
#define LDO4_TSD_ERR_SHIFT      (7U)
#define LDO1_T_PRE_ERR_MASK     (1U << LDO1_T_PRE_ERR_SHIFT)
#define LDO1_TSD_ERR_MASK       (1U << LDO1_TSD_ERR_SHIFT)
#define LDO2_T_PRE_ERR_MASK     (1U << LDO2_T_PRE_ERR_SHIFT)
#define LDO2_TSD_ERR_MASK       (1U << LDO2_TSD_ERR_SHIFT)
#define LDO3_T_PRE_ERR_MASK     (1U << LDO3_T_PRE_ERR_SHIFT)
#define LDO3_TSD_ERR_MASK       (1U << LDO3_TSD_ERR_SHIFT)
#define LDO4_T_PRE_ERR_MASK     (1U << LDO4_T_PRE_ERR_SHIFT)
#define LDO4_TSD_ERR_MASK       (1U << LDO4_TSD_ERR_SHIFT)

/** @brief THERMAL_STAT2 - Thermal Status 2 */
#define PLDO1_T_PRE_ERR_SHIFT   (0U)
#define PLDO1_TSD_ERR_SHIFT     (1U)
#define PLDO2_T_PRE_ERR_SHIFT   (2U)
#define PLDO2_TSD_ERR_SHIFT     (3U)
#define BB_T_PRE_ERR_SHIFT      (4U)
#define BB_TSD_ERR_SHIFT        (5U)
#define PLDO1_T_PRE_ERR_MASK    (1U << PLDO1_T_PRE_ERR_SHIFT)
#define PLDO1_TSD_ERR_MASK      (1U << PLDO1_TSD_ERR_SHIFT)
#define PLDO2_T_PRE_ERR_MASK    (1U << PLDO2_T_PRE_ERR_SHIFT)
#define PLDO2_TSD_ERR_MASK      (1U << PLDO2_TSD_ERR_SHIFT)
#define BB_T_PRE_ERR_MASK       (1U << BB_T_PRE_ERR_SHIFT)
#define BB_TSD_ERR_MASK         (1U << BB_TSD_ERR_SHIFT)

#ifdef __cplusplus
}
#endif
#endif /* __PMIC_REGMAP_POWER_H__ */
