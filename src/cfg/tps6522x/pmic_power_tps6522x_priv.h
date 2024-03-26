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
 * \file   pmic_power_tps6522x_priv.h
 *
 * \brief  The macro definitions, structures and function prototypes for
 *         TPS6522x BURTON PMIC driver specific PMIC power configuration
 *
 */

#ifndef PMIC_POWER_TPS6522X_PRIV_H_
#define PMIC_POWER_TPS6522X_PRIV_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include "pmic_power_priv.h"

#ifdef __cplusplus
extern "C"
{
#endif

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/*!
 *  \anchor     Tps6522x_powerRegAddr
 *  \brief      Burton power register addresses for internal use
 */
#define TPS6522X_INVALID_REGADDR         (0x00U)
#define TPS6522X_VCCA_VMON_CTRL_REGADDR  (0x2BU)
#define TPS6522X_VCCA_PG_WINDOW_REGADDR  (0x2CU)
#define TPS6522X_VMON1_PG_WINDOW_REGADDR (0x2DU)
#define TPS6522X_VMON1_PG_LEVEL_REGADDR  (0x2EU)
#define TPS6522X_VMON2_PG_WINDOW_REGADDR (0x2FU)
#define TPS6522X_VMON2_PG_LEVEL_REGADDR  (0x30U)
#define TPS6522X_RAIL_SEL_3_REGADDR      (0x43U)
#define TPS6522X_STAT_BUCK_REGADDR       (0x6DU)
#define TPS6522X_STAT_LDO_VMON_REGADDR   (0x70U)

/**
 *  \anchor     Tps6522x_buckCtrlRegShiftVal
 *  \name       BUCK_CTRL register shift values supported by TPS6522x Burton
 *
 *  @{
 */
#define TPS6522X_BUCK_PLDN_SHIFT                  (0x5U)
#define TPS6522X_BUCK_VMON_EN_SHIFT               (0x4U)
#define TPS6522X_BUCK_PWM_OPTION_SHIFT            (0x1U)
#define TPS6522X_BUCK_EN_SHIFT                    (0x0U)
/** @} */

/**
 *  \anchor     Tps6522x_buckCtrlRegMaskVal
 *  \name       BUCK_CTRL register mask values supported by TPS6522x Burton
 *
 *  @{
 */
#define TPS6522X_BUCK_PLDN_MASK                   (0x1U << TPS6522X_BUCK_PLDN_SHIFT)
#define TPS6522X_BUCK_VMON_EN_MASK                (0x1U << TPS6522X_BUCK_VMON_EN_SHIFT)
#define TPS6522X_BUCK_PWM_OPTION_MASK             (0x1U << TPS6522X_BUCK_PWM_OPTION_SHIFT)
#define TPS6522X_BUCK_EN_MASK                     (0x1U << TPS6522X_BUCK_EN_SHIFT)
/** @} */

/**
 *  \anchor     Tps6522x_buckConfRegShiftVal
 *  \name       BUCK_CONF register shift values supported by TPS6522x Burton
 */
#define TPS6522X_BUCK_SLEW_RATE_SHIFT             (0x0U)

/**
 *  \anchor      Tps6522x_buckConfRegMaskVal
 *  \name        BUCK_CONF register mask values supported by TPS6522x Burton
 */
#define TPS6522X_BUCK_SLEW_RATE_MASK              (0x3U << TPS6522X_BUCK_SLEW_RATE_SHIFT)

/**
 *  \anchor     Tps6522x_buckPgWindowRegShiftVal
 *  \name       BUCK_PG_WINDOW register shift values supported by TPS6522x Burton
 */
#define TPS6522X_BUCK_VMON_THR_SHIFT         (0x0U)

/**
 *  \anchor     Tps6522x_buckPgWindowRegMaskVal
 *  \name       BUCK_PG_WINDOW register mask values supported by TPS6522x Burton
 */
#define TPS6522X_BUCK_VMON_THR_MASK          (0x3U << TPS6522X_BUCK_VMON_THR_SHIFT)

/**
 *  \anchor     Tps6522x_buckVoutRegShiftVal
 *  \name       BUCK_VOUT register shift values supported by TPS6522x Burton
 */
#define TPS6522X_BUCK_VSET_SHIFT            (0x0U)

/**
 *  \anchor     Tps6522x_buckVoutRegMaskVal
 *  \name       BUCK_VOUT register mask values supported by TPS6522x Burton
 */
#define TPS6522X_BUCK1_VSET_MASK            (0xFFU << TPS6522X_BUCK_VSET_SHIFT)
#define TPS6522X_BUCK2_3_4_VSET_MASK        (0x7FU << TPS6522X_BUCK_VSET_SHIFT)

/**
 *  \anchor     Tps6522x_railSel1RegShiftVal
 *  \name       RAIL_SEL_1 register shift values supported by TPS6522x Burton
 *
 *  @{
 */
#define TPS6522X_BUCK4_GRP_SEL_SHIFT    (6U)
#define TPS6522X_BUCK3_GRP_SEL_SHIFT    (4U)
#define TPS6522X_BUCK2_GRP_SEL_SHIFT    (2U)
#define TPS6522X_BUCK1_GRP_SEL_SHIFT    (0U)
/** @} */

/**
 *  \anchor     Tps6522x_railSel1RegMaskVal
 *  \name       RAIL_SEL_1 register mask values supported by TPS6522x Burton
 *
 *  @{
 */
#define TPS6522X_BUCK4_GRP_SEL_MASK     (3U << TPS6522X_BUCK4_GRP_SEL_SHIFT)
#define TPS6522X_BUCK3_GRP_SEL_MASK     (3U << TPS6522X_BUCK3_GRP_SEL_SHIFT)
#define TPS6522X_BUCK2_GRP_SEL_MASK     (3U << TPS6522X_BUCK2_GRP_SEL_SHIFT)
#define TPS6522X_BUCK1_GRP_SEL_MASK     (3U << TPS6522X_BUCK1_GRP_SEL_SHIFT)
/** @} */

/**
 *  \anchor     Tps6522x_ldoCtrlRegShiftVal
 *  \name       LDO_CTRL register shift values supported by TPS6522x Burton
 *
 *  @{
 */
#define TPS6522X_LDO_DISCHARGE_EN_SHIFT      (0x5U)
#define TPS6522X_LDO_VMON_EN_SHIFT           (0x4U)
#define TPS6522X_LDO_EN_SHIFT                (0x0U)
/** @} */

/**
 *  \anchor     Tps6522x_ldoCtrlRegMaskVal
 *  \name       LDO_CTRL register mask values supported by TPS6522x Burton
 *
 *  @{
 */
#define TPS6522X_LDO_DISCHARGE_EN_MASK       (0x1U << TPS6522X_LDO_DISCHARGE_EN_SHIFT)
#define TPS6522X_LDO_VMON_EN_MASK            (0x1U << TPS6522X_LDO_VMON_EN_SHIFT)
#define TPS6522X_LDO_EN_MASK                 (0x1U << TPS6522X_LDO_EN_SHIFT)
/** @} */

/**
 *  \anchor     Tps6522x_ldoVoutRegShiftVal
 *  \name       LDO_VOUT register shift values supported by TPS6522x Burton
 *
 *  @{
 */
#define TPS6522X_LDO_MODE_SHIFT          (0x7U)
#define TPS6522X_LDO_VSET_SHIFT          (0x1U)
/** @} */

/**
 *  \anchor     Tps6522x_ldoVoutRegMaskVal
 *  \name       LDO_VOUT register mask values supported by TPS6522x Burton
 */
#define TPS6522X_LDO_MODE_MASK           (0x1U << TPS6522X_LDO_MODE_SHIFT)
#define TPS6522X_LDO_VSET_MASK           (0x3FU << TPS6522X_LDO_VSET_SHIFT)
/** @} */

/**
 *  \anchor     Tps6522x_ldoPgWindowRegShiftVal
 *  \name       LDO_PG_WINDOW register shift values supported by TPS6522x Burton
 */
#define TPS6522X_LDO_VMON_THR_SHIFT (0x0U)

/**
 *  \anchor     Tps6522x_ldoPgWindowRegMaskVal
 *  \name       LDO_PG_WINDOW register mask values supported by TPS6522x Burton
 */
#define TPS6522X_LDO_VMON_THR_MASK (0x3U << TPS6522X_LDO_VMON_THR_SHIFT)

/**
 *  \anchor     Tps6522x_railSel2RegShiftVal
 *  \name       RAIL_SEL_2 register shift values supported by TPS6522x Burton
 *
 *  @{
 */
#define TPS6522X_LDO3_GRP_SEL_SHIFT     (0x6U)
#define TPS6522X_LDO2_GRP_SEL_SHIFT     (0x4U)
#define TPS6522X_LDO1_GRP_SEL_SHIFT     (0x2U)
/** @} */

/**
 *  \anchor     Tps6522x_railSel2RegMaskVal
 *  \name       RAIL_SEL_2 register mask values supported by TPS6522x Burton
 *
 *  @{
 */
#define TPS6522X_LDO3_GRP_SEL_MASK      (0x3U << TPS6522X_LDO3_GRP_SEL_SHIFT)
#define TPS6522X_LDO2_GRP_SEL_MASK      (0x3U << TPS6522X_LDO2_GRP_SEL_SHIFT)
#define TPS6522X_LDO1_GRP_SEL_MASK      (0x3U << TPS6522X_LDO1_GRP_SEL_SHIFT)
/** @} */

/**
 *  \anchor     Tps6522x_vccaVmonCtrlRegShiftVal
 *  \name       VCCA_VMON_CTRL register shift values supported by TPS6522x Burton
 *
 *  @{
 */
#define TPS6522X_VCCA_VMON_DEGL_SEL_SHIFT   (5U)
#define TPS6522X_VMON2_EN_SHIFT             (3U)
#define TPS6522X_VMON1_EN_SHIFT             (1U)
#define TPS6522X_VCCA_VMON_EN_SHIFT         (0U)
/** @} */

/**
 *  \anchor     Tps6522x_vccaVmonCtrlRegMaskVal
 *  \name       VCCA_VMON_CTRL register mask values supported by TPS6522x Burton
 *
 *  @{
 */
#define TPS6522X_VCCA_VMON_DEGL_SEL_MASK    (7U << TPS6522X_VCCA_VMON_DEGL_SEL_SHIFT)
#define TPS6522X_VMON2_EN_MASK              (1U << TPS6522X_VMON2_EN_SHIFT)
#define TPS6522X_VMON1_EN_MASK              (1U << TPS6522X_VMON1_EN_SHIFT)
#define TPS6522X_VCCA_VMON_EN_MASK          (1U << TPS6522X_VCCA_VMON_EN_SHIFT)
/** @} */

/**
 *  \anchor     Tps6522x_vccaPgWindowRegShiftVal
 *  \name       VCCA_PG_WINDOW register shift values supported by TPS6522x Burton
 *
 *  @{
 */
#define TPS6522X_VCCA_PG_SET_SHIFT   (6U)
#define TPS6522X_VCCA_VMON_THR_SHIFT (0U)
/** @} */

/**
 *  \anchor     Tps6522x_vccaPgWindowRegMaskVal
 *  \name       VCCA_PG_WINDOW register mask values supported by TPS6522x Burton
 *
 *  @{
 */
#define TPS6522X_VCCA_PG_SET_MASK    (1U << TPS6522X_VCCA_PG_SET_SHIFT)
#define TPS6522X_VCCA_VMON_THR_MASK  (3U << TPS6522X_VCCA_VMON_THR_SHIFT)
/** @} */

/**
 *  \anchor     Tps6522x_vmonPgWindowRegShiftVal
 *  \name       VMON_PG_WINDOW register shift values supported by TPS6522x Burton
 *
 *  @{
 */
#define TPS6522X_VMON1_THR_SHIFT   (0U)
#define TPS6522X_VMON2_THR_SHIFT   (0U)
/** @} */

/**
 *  \anchor     Tps6522x_vmonPgWindowRegMaskVal
 *  \name       VMON_PG_WINDOW register mask values supported by TPS6522x Burton
 *
 *  @{
 */
#define TPS6522X_VMON1_THR_MASK    (3U << TPS6522X_VMON1_THR_SHIFT)
#define TPS6522X_VMON2_THR_MASK    (3U << TPS6522X_VMON2_THR_SHIFT)
/** @} */

/**
 *  \anchor     Tps6522x_vmonPgLevelRegShiftVal
 *  \name       VMON_PG_LEVEL register shift values supported by TPS6522x Burton
 *
 *  @{
 */
#define TPS6522X_VMON1_PG_SET_SHIFT (0U)
#define TPS6522X_VMON2_PG_SET_SHIFT (0U)
/** @} */

/**
 *  \anchor     Tps6522x_vmonPgLevelRegMaskVal
 *  \name       VMON_PG_LEVEL register mask values supported by TPS6522x Burton
 *
 *  @{
 */
#define TPS6522X_VMON1_PG_SET_MASK  (0xFFU << TPS6522X_VMON1_PG_SET_SHIFT)
#define TPS6522X_VMON2_PG_SET_MASK  (0x7FU << TPS6522X_VMON2_PG_SET_SHIFT)
/** @} */

/**
 *  \anchor     Tps6522x_railSel3RegShiftVal
 *  \name       RAIL_SEL_3 register shift values supported by TPS6522x Burton
 *
 *  @{
 */
#define TPS6522X_VMON2_GRP_SEL_SHIFT      (6U)
#define TPS6522X_VMON1_GRP_SEL_SHIFT      (4U)
#define TPS6522X_VCCA_GRP_SEL_SHIFT       (2U)
/** @} */

/**
 *  \anchor     Tps6522x_railSel3RegMaskVal
 *  \name       RAIL_SEL_3 register mask values supported by TPS6522x Burton
 *
 *  @{
 */
#define TPS6522X_VMON2_GRP_SEL_MASK       (3U << TPS6522X_VMON2_GRP_SEL_SHIFT)
#define TPS6522X_VMON1_GRP_SEL_MASK       (3U << TPS6522X_VMON1_GRP_SEL_SHIFT)
#define TPS6522X_VCCA_GRP_SEL_MASK        (3U << TPS6522X_VCCA_GRP_SEL_SHIFT)
/** @} */

/**
 *  \anchor     Tps6522x_statBuckRegShiftVal
 *  \name       STAT_BUCK register shift values supported by TPS6522x Burton
 *
 *  @{
 */
#define TPS6522X_BUCK4_UVOV_STAT_SHIFT     (3U)
#define TPS6522X_BUCK3_UVOV_STAT_SHIFT     (2U)
#define TPS6522X_BUCK2_UVOV_STAT_SHIFT     (1U)
#define TPS6522X_BUCK1_UVOV_STAT_SHIFT     (0U)
/** @} */

/**
 *  \anchor     Tps6522x_statBuckRegMaskVal
 *  \name       STAT_BUCK register mask values supported by TPS6522x Burton
 *
 *  @{
 */
#define TPS6522X_BUCK4_UVOV_STAT_MASK      (1U << TPS6522X_BUCK4_UVOV_STAT_SHIFT)
#define TPS6522X_BUCK3_UVOV_STAT_MASK      (1U << TPS6522X_BUCK3_UVOV_STAT_SHIFT)
#define TPS6522X_BUCK2_UVOV_STAT_MASK      (1U << TPS6522X_BUCK2_UVOV_STAT_SHIFT)
#define TPS6522X_BUCK1_UVOV_STAT_MASK      (1U << TPS6522X_BUCK1_UVOV_STAT_SHIFT)
/** @} */

/**
 *  \anchor     Tps6522x_statLdoVmonRegShiftVal
 *  \name       STAT_LDO_VMON register shift values supported by TPS6522x Burton
 *
 *  @{
 */
#define TPS6522X_VMON2_UVOV_STAT_SHIFT (6U)
#define TPS6522X_VMON1_UVOV_STAT_SHIFT (5U)
#define TPS6522X_VCCA_UVOV_STAT_SHIFT  (4U)
#define TPS6522X_LDO3_UVOV_STAT_SHIFT  (2U)
#define TPS6522X_LDO2_UVOV_STAT_SHIFT  (1U)
#define TPS6522X_LDO1_UVOV_STAT_SHIFT  (0U)
/** @} */

/**
 *  \anchor     Tps6522x_statLdoVmonRegMaskVal
 *  \name       STAT_LDO_VMON register mask values supported by TPS6522x Burton
 *
 *  @{
 */
#define TPS6522X_VMON2_UVOV_STAT_MASK   (1U << TPS6522X_VMON2_UVOV_STAT_SHIFT)
#define TPS6522X_VMON1_UVOV_STAT_MASK   (1U << TPS6522X_VMON1_UVOV_STAT_SHIFT)
#define TPS6522X_VCCA_UVOV_STAT_MASK    (1U << TPS6522X_VCCA_UVOV_STAT_SHIFT)
#define TPS6522X_LDO3_UVOV_STAT_MASK    (1U << TPS6522X_LDO3_UVOV_STAT_SHIFT)
#define TPS6522X_LDO2_UVOV_STAT_MASK    (1U << TPS6522X_LDO2_UVOV_STAT_SHIFT)
#define TPS6522X_LDO1_UVOV_STAT_MASK    (1U << TPS6522X_LDO1_UVOV_STAT_SHIFT)
/** @} */

/**
 *  \anchor     Tps6522x_thermalStatShiftVal
 *  \name       Register shift values for thermal status supported by TPS6522x Burton
 *
 *  @{
 */
#define TPS6522X_TWARN_STAT_SHIFT                   (3U)
#define TPS6522X_TSD_ORD_STAT_SHIFT                 (0U)
#define TPS6522X_TSD_IMM_STAT_SHIFT                 (0U)
/** @} */

/**
 *  \anchor     Tps6522x_thermalStatMaskVal
 *  \name       Register mask values for thermal status supported by TPS6522x Burton
 *
 *  @{
 */
#define TPS6522X_TWARN_STAT_MASK                    (1U << TPS6522X_TWARN_STAT_SHIFT)
#define TPS6522X_TSD_ORD_STAT_MASK                  (1U << TPS6522X_TSD_ORD_STAT_SHIFT)
#define TPS6522X_TSD_IMM_STAT_MASK                  (1U << TPS6522X_TSD_IMM_STAT_SHIFT)
/** @} */

/**
 *  \anchor     Tps6522x_thermalCfgShiftVal
 *  \name       Register shift values for thermal configuration supported by TPS6522x Burton
 *
 *  @{
 */
#define TPS6522X_TSD_ORD_LEVEL_SHIFT                (1U)
#define TPS6522X_TWARN_LEVEL_SHIFT                  (0U)
/** @} */

/**
 *  \anchor     Tps6522x_thermalCfgMaskVal
 *  \name       Register mask values for thermal configuration supported by TPS6522x Burton
 *
 *  @{
 */
#define TPS6522X_TWARN_LEVEL_MASK                   (1U << TPS6522X_TWARN_LEVEL_SHIFT)
#define TPS6522X_TSD_ORD_LEVEL_MASK                 (1U << TPS6522X_TSD_ORD_LEVEL_SHIFT)
/** @} */

/**
 *  \anchor     Tps6522x_vccaVmonRegDataIndex
 *  \name       Register data indices for VCCA/VMON
 *  
 *  \brief      These defines are used to access register data for the set/get
 *              VCCA/VMON power resource configuration APIs
 *  @{
 */
#define TPS6522X_VCCA_VMON_CTRL_REGDATA_INDEX       (0U)
#define TPS6522X_VCCA_PG_WINDOW_REGDATA_INDEX       (1U)
#define TPS6522X_VMON_PG_WINDOW_REGDATA_INDEX       (2U)
#define TPS6522X_VMON_PG_LEVEL_REGDATA_INDEX        (3U)
#define TPS6522X_VCCA_VMON_RAIL_SEL_REGDATA_INDEX   (4U)
#define TPS6522X_VCCA_VMON_REGDATA_NUM_INDICES      (5U)
/** @} */

/**
 *  \anchor     Tps6522x_ldoRegDataIndex
 *  \name       Register data indices for LDO
 *  
 *  \brief      These defines are used to access register data for the set/get
 *              LDO power resource configuration APIs
 *  @{
 */
#define TPS6522X_LDO_CTRL_REGDATA_INDEX         (0U)
#define TPS6522X_LDO_VOUT_REGDATA_INDEX         (1U)
#define TPS6522X_LDO_PG_WINDOW_REGDATA_INDEX    (2U)
#define TPS6522X_LDO_RAIL_SEL_REGDATA_INDEX     (3U)
#define TPS6522X_LDO_REGDATA_NUM_INDICES        (4U)
/** @} */

/**
 *  \anchor     Tps6522x_buckRegDataIndex
 *  \name       Register data indices for BUCK
 *  
 *  \brief      These defines are used to access register data for the set/get
 *              BUCK power resource configuration APIs
 *  @{
 */
#define TPS6522X_BUCK_CTRL_REGDATA_INDEX            (0U)
#define TPS6522X_BUCK_CONF_REGDATA_INDEX            (1U)
#define TPS6522X_BUCK_VOUT_REGDATA_INDEX            (2U)
#define TPS6522X_BUCK_PG_WINDOW_REGDATA_INDEX       (3U)
#define TPS6522X_BUCK_RAIL_SEL_REGDATA_INDEX        (4U)
#define TPS6522X_BUCK_REGDATA_NUM_INDICES           (5U)
/** @} */

/* ========================================================================== */
/*                          Structures and Enums                              */
/* ========================================================================== */

/* ========================================================================== */
/*                         Function Declarations                              */
/* ========================================================================== */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* PMIC_POWER_TPS6522X_PRIV_H_ */
