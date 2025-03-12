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
 * @file   pmic_power.h
 *
 * @brief  PMIC PMIC Power Resources Driver API/interface file.
 *
 */

#ifndef __PMIC_POWER_H__
#define __PMIC_POWER_H__

#include <stdint.h>

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include "pmic_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

// Intended for internal use only
#define PMIC_PWR_TYPE_BUCK_BOOST    (0U)
#define PMIC_PWR_TYPE_LDO           (1U)
#define PMIC_PWR_TYPE_PLDO          (2U)
#define PMIC_PWR_TYPE_EXT_VMON      (3U)
#define PMIC_PWR_RSRC_TYPE_SHIFT    (8U)

// Intended for internal use only
#define PMIC_PWR_ID_BUCK_BOOST  (0U)
#define PMIC_PWR_ID_LDO1        (0U)
#define PMIC_PWR_ID_LDO2        (1U)
#define PMIC_PWR_ID_LDO3        (2U)
#define PMIC_PWR_ID_LDO4        (3U)
#define PMIC_PWR_ID_PLDO1       (0U)
#define PMIC_PWR_ID_PLDO2       (1U)
#define PMIC_PWR_ID_EXT_VMON1   (0U)
#define PMIC_PWR_ID_EXT_VMON2   (1U)

/**
 * @anchor Pmic_PwrRsrc
 * @name PMIC Power Resource
 *
 * @brief Different power resources available on the PMIC.
 *
 * @{
 */
#define PMIC_PWR_BUCK_BOOST ((uint16_t)((PMIC_PWR_TYPE_BUCK_BOOST << PMIC_PWR_RSRC_TYPE_SHIFT) | PMIC_PWR_ID_BUCK_BOOST))
#define PMIC_PWR_LDO1       ((uint16_t)((PMIC_PWR_TYPE_LDO << PMIC_PWR_RSRC_TYPE_SHIFT) | PMIC_PWR_ID_LDO1))
#define PMIC_PWR_LDO2       ((uint16_t)((PMIC_PWR_TYPE_LDO << PMIC_PWR_RSRC_TYPE_SHIFT) | PMIC_PWR_ID_LDO2))
#define PMIC_PWR_LDO3       ((uint16_t)((PMIC_PWR_TYPE_LDO << PMIC_PWR_RSRC_TYPE_SHIFT) | PMIC_PWR_ID_LDO3))
#define PMIC_PWR_LDO4       ((uint16_t)((PMIC_PWR_TYPE_LDO << PMIC_PWR_RSRC_TYPE_SHIFT) | PMIC_PWR_ID_LDO4))
#define PMIC_PWR_PLDO1      ((uint16_t)((PMIC_PWR_TYPE_PLDO << PMIC_PWR_RSRC_TYPE_SHIFT) | PMIC_PWR_ID_PLDO1))
#define PMIC_PWR_PLDO2      ((uint16_t)((PMIC_PWR_TYPE_PLDO << PMIC_PWR_RSRC_TYPE_SHIFT) | PMIC_PWR_ID_PLDO2))
#define PMIC_PWR_EXT_VMON1  ((uint16_t)((PMIC_PWR_TYPE_EXT_VMON << PMIC_PWR_RSRC_TYPE_SHIFT) | PMIC_PWR_ID_EXT_VMON1))
#define PMIC_PWR_EXT_VMON2  ((uint16_t)((PMIC_PWR_TYPE_EXT_VMON << PMIC_PWR_RSRC_TYPE_SHIFT) | PMIC_PWR_ID_EXT_VMON2))
/** @} */

/**
 * @anchor Pmic_PwrRsrcMinMax
 * @name PMIC Power Resource Minimum and Maximum
 *
 * @brief Minimum and maximum power resources of the PMIC.
 *
 * @{
 */
#define PMIC_PWR_BUCK_BOOST_MIN (PMIC_PWR_BUCK_BOOST)
#define PMIC_PWR_BUCK_BOOST_MAX (PMIC_PWR_BUCK_BOOST)
#define PMIC_PWR_LDO_MIN        (PMIC_PWR_LDO1)
#define PMIC_PWR_LDO_MAX        (PMIC_PWR_LDO4)
#define PMIC_PWR_PLDO_MIN       (PMIC_PWR_PLDO1)
#define PMIC_PWR_PLDO_MAX       (PMIC_PWR_PLDO2)
#define PMIC_PWR_EXT_VMON_MIN   (PMIC_PWR_EXT_VMON1)
#define PMIC_PWR_EXT_VMON_MAX   (PMIC_PWR_EXT_VMON2)
/** @} */

/**
 * @anchor Pmic_PwrBuckBoostCfgValidParam
 * @name PMIC Power Buck/Boost Configuration Valid Parameters
 *
 * @brief Valid parameters of the Pmic_PwrBuckBoostCfg_t struct.
 *
 * @{
 */
#define PMIC_PWR_CFG_BB_LVL_VALID                           (0U)
#define PMIC_PWR_CFG_BB_STBY_LVL_VALID                      (1U)
#define PMIC_PWR_CFG_BB_VMON_THR_VALID                      (2U)
#define PMIC_PWR_CFG_BB_VMON_DGL_VALID                      (3U)
#define PMIC_PWR_CFG_BB_BOOST_TMO_VALID                     (4U)
#define PMIC_PWR_CFG_BB_SS_EN_VALID                         (5U)
#define PMIC_PWR_CFG_BB_INCLUDE_OV_UV_STAT_IN_PGOOD_VALID   (6U)
/** @} */

/**
 * @anchor Pmic_PwrBuckBoostCfgValidParamShift
 * @name PMIC Power Buck/Boost Configuration Valid Parameter Shifts
 *
 * @brief Valid parameter shifts of the Pmic_PwrBuckBoostCfg_t struct. End user
 * can use the defines listed below to indicate valid Buck/Boost configurations.
 * Multiple valid parameters can be indicated using the OR operator.
 *
 * @{
 */
#define PMIC_PWR_CFG_BB_LVL_VALID_SHIFT                         (1U << PMIC_PWR_CFG_BB_LVL_VALID)
#define PMIC_PWR_CFG_BB_STBY_LVL_VALID_SHIFT                    (1U << PMIC_PWR_CFG_BB_STBY_LVL_VALID)
#define PMIC_PWR_CFG_BB_VMON_THR_VALID_SHIFT                    (1U << PMIC_PWR_CFG_BB_VMON_THR_VALID)
#define PMIC_PWR_CFG_BB_VMON_DGL_VALID_SHIFT                    (1U << PMIC_PWR_CFG_BB_VMON_DGL_VALID)
#define PMIC_PWR_CFG_BB_BOOST_TMO_VALID_SHIFT                   (1U << PMIC_PWR_CFG_BB_BOOST_TMO_VALID)
#define PMIC_PWR_CFG_BB_SS_EN_VALID_SHIFT                       (1U << PMIC_PWR_CFG_BB_SS_EN_VALID)
#define PMIC_PWR_CFG_BB_INCLUDE_OV_UV_STAT_IN_PGOOD_VALID_SHIFT (1U << PMIC_PWR_CFG_BB_INCLUDE_OV_UV_STAT_IN_PGOOD_VALID)
#define PMIC_PWR_CFG_BB_ALL                                     (PMIC_PWR_CFG_BB_LVL_VALID_SHIFT  | \
                                                                 PMIC_PWR_CFG_BB_STBY_LVL_VALID_SHIFT | \
                                                                 PMIC_PWR_CFG_BB_VMON_THR_VALID_SHIFT | \
                                                                 PMIC_PWR_CFG_BB_VMON_DGL_VALID_SHIFT | \
                                                                 PMIC_PWR_CFG_BB_BOOST_TMO_VALID_SHIFT | \
                                                                 PMIC_PWR_CFG_BB_SS_EN_VALID_SHIFT | \
                                                                 PMIC_PWR_CFG_BB_INCLUDE_OV_UV_STAT_IN_PGOOD_VALID_SHIFT)
/** @} */

/**
 * @anchor Pmic_PwrLdoCfgValidParam
 * @name PMIC Power LDO Configuration Valid Parameters
 *
 * @brief Valid parameters of the Pmic_PwrLdoCfg_t struct.
 *
 * @{
 */
#define PMIC_PWR_CFG_LDO_MODE_VALID                         (0U)
#define PMIC_PWR_CFG_LDO_LVL_VALID                          (1U)
#define PMIC_PWR_CFG_LDO_ILIM_LVL_VALID                     (2U)
#define PMIC_PWR_CFG_LDO_ILIM_DGL_VALID                     (3U)
#define PMIC_PWR_CFG_LDO_VMON_THR_VALID                     (4U)
#define PMIC_PWR_CFG_LDO_VMON_DGL_VALID                     (5U)
#define PMIC_PWR_CFG_LDO_RAMP_TIME_VALID                    (6U)
#define PMIC_PWR_CFG_LDO_DISABLE_DISCHARGE_VALID            (7U)
#define PMIC_PWR_CFG_LDO_INCLUDE_OV_UV_STAT_IN_PGOOD_VALID  (8U)
/** @} */

/**
 * @anchor Pmic_PwrLdoCfgValidParamShifts
 * @name PMIC Power LDO Configuration Valid Parameter Shifts
 *
 * @brief Valid parameter shifts of the Pmic_PwrLdoCfg_t struct. End user can
 * use the defines listed below to indicate valid LDO configurations. Multiple
 * valid parameters can be indicated using the OR operator.
 *
 * @{
 */
#define PMIC_PWR_CFG_LDO_MODE_VALID_SHIFT                           (1U << PMIC_PWR_CFG_LDO_MODE_VALID)
#define PMIC_PWR_CFG_LDO_LVL_VALID_SHIFT                            (1U << PMIC_PWR_CFG_LDO_LVL_VALID)
#define PMIC_PWR_CFG_LDO_ILIM_LVL_VALID_SHIFT                       (1U << PMIC_PWR_CFG_LDO_ILIM_LVL_VALID)
#define PMIC_PWR_CFG_LDO_ILIM_DGL_VALID_SHIFT                       (1U << PMIC_PWR_CFG_LDO_ILIM_DGL_VALID)
#define PMIC_PWR_CFG_LDO_VMON_THR_VALID_SHIFT                       (1U << PMIC_PWR_CFG_LDO_VMON_THR_VALID)
#define PMIC_PWR_CFG_LDO_VMON_DGL_VALID_SHIFT                       (1U << PMIC_PWR_CFG_LDO_VMON_DGL_VALID)
#define PMIC_PWR_CFG_LDO_RAMP_TIME_VALID_SHIFT                      (1U << PMIC_PWR_CFG_LDO_RAMP_TIME_VALID)
#define PMIC_PWR_CFG_LDO_DISABLE_DISCHARGE_VALID_SHIFT              (1U << PMIC_PWR_CFG_LDO_DISABLE_DISCHARGE_VALID)
#define PMIC_PWR_CFG_LDO_INCLUDE_OV_UV_STAT_IN_PGOOD_VALID_SHIFT    (1U << PMIC_PWR_CFG_LDO_INCLUDE_OV_UV_STAT_IN_PGOOD_VALID)
#define PMIC_PWR_CFG_LDO_ALL                                        (PMIC_PWR_CFG_LDO_MODE_VALID_SHIFT | \
                                                                     PMIC_PWR_CFG_LDO_LVL_VALID_SHIFT | \
                                                                     PMIC_PWR_CFG_LDO_ILIM_LVL_VALID_SHIFT | \
                                                                     PMIC_PWR_CFG_LDO_ILIM_DGL_VALID_SHIFT | \
                                                                     PMIC_PWR_CFG_LDO_VMON_THR_VALID_SHIFT | \
                                                                     PMIC_PWR_CFG_LDO_VMON_DGL_VALID_SHIFT | \
                                                                     PMIC_PWR_CFG_LDO_RAMP_TIME_VALID_SHIFT | \
                                                                     PMIC_PWR_CFG_LDO_DISABLE_DISCHARGE_VALID_SHIFT | \
                                                                     PMIC_PWR_CFG_LDO_INCLUDE_OV_UV_STAT_IN_PGOOD_VALID_SHIFT)
/** @} */

/**
 * @anchor Pmic_PwrPldoCfgValidParam
 * @name PMIC Power PLDO Configuration Valid Parameters
 *
 * @brief Valid parameters of the Pmic_PwrPldoCfg_t struct.
 *
 * @{
 */
#define PMIC_PWR_CFG_PLDO_MODE_VALID                        (0U)
#define PMIC_PWR_CFG_PLDO_TRACKING_MODE_VALID               (1U)
#define PMIC_PWR_CFG_PLDO_LVL_VALID                         (2U)
#define PMIC_PWR_CFG_PLDO_ILIM_LVL_VALID                    (3U)
#define PMIC_PWR_CFG_PLDO_ILIM_DGL_VALID                    (4U)
#define PMIC_PWR_CFG_PLDO_VMON_THR_VALID                    (5U)
#define PMIC_PWR_CFG_PLDO_VMON_DGL_VALID                    (6U)
#define PMIC_PWR_CFG_PLDO_VTRACK_RANGE_VALID                (7U)
#define PMIC_PWR_CFG_PLDO_RT_VALID                          (8U)
#define PMIC_PWR_CFG_PLDO_DISABLE_DISCHARGE_VALID           (9U)
#define PMIC_PWR_CFG_PLDO_INCLUDE_OV_UV_STAT_IN_PGOOD_VALID (10U)
/** @} */

/**
 * @anchor Pmic_PwrPldoCfgValidParamShifts
 * @name PMIC Power PLDO Configuration Valid Parameter Shifts
 *
 * @brief Valid parameter shifts of the Pmic_PwrPldoCfg_t struct. End user can
 * use the defines listed below to indicate valid PLDO configurations. Multiple
 * valid parameters can be indicated using the OR operator.
 *
 * @{
 */
#define PMIC_PWR_CFG_PLDO_MODE_VALID_SHIFT                          (1U << PMIC_PWR_CFG_PLDO_MODE_VALID)
#define PMIC_PWR_CFG_PLDO_TRACKING_MODE_VALID_SHIFT                 (1U << PMIC_PWR_CFG_PLDO_TRACKING_MODE_VALID)
#define PMIC_PWR_CFG_PLDO_LVL_VALID_SHIFT                           (1U << PMIC_PWR_CFG_PLDO_LVL_VALID)
#define PMIC_PWR_CFG_PLDO_ILIM_LVL_VALID_SHIFT                      (1U << PMIC_PWR_CFG_PLDO_ILIM_LVL_VALID)
#define PMIC_PWR_CFG_PLDO_ILIM_DGL_VALID_SHIFT                      (1U << PMIC_PWR_CFG_PLDO_ILIM_DGL_VALID)
#define PMIC_PWR_CFG_PLDO_VMON_THR_VALID_SHIFT                      (1U << PMIC_PWR_CFG_PLDO_VMON_THR_VALID)
#define PMIC_PWR_CFG_PLDO_VMON_DGL_VALID_SHIFT                      (1U << PMIC_PWR_CFG_PLDO_VMON_DGL_VALID)
#define PMIC_PWR_CFG_PLDO_VTRACK_RANGE_VALID_SHIFT                  (1U << PMIC_PWR_CFG_PLDO_VTRACK_RANGE_VALID)
#define PMIC_PWR_CFG_PLDO_RT_VALID_SHIFT                            (1U << PMIC_PWR_CFG_PLDO_RT_VALID)
#define PMIC_PWR_CFG_PLDO_DISABLE_DISCHARGE_VALID_SHIFT             (1U << PMIC_PWR_CFG_PLDO_DISABLE_DISCHARGE_VALID)
#define PMIC_PWR_CFG_PLDO_INCLUDE_OV_UV_STAT_IN_PGOOD_VALID_SHIFT   (1U << PMIC_PWR_CFG_PLDO_INCLUDE_OV_UV_STAT_IN_PGOOD_VALID)
#define PMIC_PWR_CFG_PLDO_ALL                                       (PMIC_PWR_CFG_PLDO_MODE_VALID_SHIFT | \
                                                                     PMIC_PWR_CFG_PLDO_TRACKING_MODE_VALID_SHIFT | \
                                                                     PMIC_PWR_CFG_PLDO_LVL_VALID_SHIFT | \
                                                                     PMIC_PWR_CFG_PLDO_ILIM_LVL_VALID_SHIFT  | \
                                                                     PMIC_PWR_CFG_PLDO_ILIM_DGL_VALID_SHIFT | \
                                                                     PMIC_PWR_CFG_PLDO_VMON_THR_VALID_SHIFT | \
                                                                     PMIC_PWR_CFG_PLDO_VMON_DGL_VALID_SHIFT | \
                                                                     PMIC_PWR_CFG_PLDO_VTRACK_RANGE_VALID_SHIFT | \
                                                                     PMIC_PWR_CFG_PLDO_RT_VALID_SHIFT | \
                                                                     PMIC_PWR_CFG_PLDO_DISABLE_DISCHARGE_VALID_SHIFT | \
                                                                     PMIC_PWR_CFG_PLDO_INCLUDE_OV_UV_STAT_IN_PGOOD_VALID_SHIFT)
/** @} */

/**
 * @anchor Pmic_PwrExtVmonCfgValidParam
 * @name PMIC Power External VMON Configuration Valid Parameters
 *
 * @brief Valid parameters of the Pmic_PwrExtVmonCfg_t struct.
 *
 * @{
 */
#define PMIC_PWR_CFG_EXT_VMON_MODE_VALID                        (0U)
#define PMIC_PWR_CFG_EXT_VMON_THR_VALID                         (1U)
#define PMIC_PWR_CFG_EXT_VMON_DGL_VALID                         (2U)
#define PMIC_PWR_CFG_EXT_VMON_INCLUDE_OV_UV_STAT_IN_PGOOD_VALID (3U)
/** @} */

/**
 * @anchor Pmic_PwrExtVmonCfgValidParamShifts
 * @name PMIC Power External VMON Configuration Valid Parameter Shifts
 *
 * @brief Valid parameter shifts of the Pmic_PwrExtVmonCfg_t struct. End user
 * can use the defines listed below to indicate valid external VMON
 * configurations. Multiple valid parameters can be indicated using the OR
 * operator.
 *
 */
#define PMIC_PWR_CFG_EXT_VMON_MODE_VALID_SHIFT                          (1U << PMIC_PWR_CFG_EXT_VMON_MODE_VALID)
#define PMIC_PWR_CFG_EXT_VMON_THR_VALID_SHIFT                           (1U << PMIC_PWR_CFG_EXT_VMON_THR_VALID)
#define PMIC_PWR_CFG_EXT_VMON_DGL_VALID_SHIFT                           (1U << PMIC_PWR_CFG_EXT_VMON_DGL_VALID)
#define PMIC_PWR_CFG_EXT_VMON_INCLUDE_OV_UV_STAT_IN_PGOOD_VALID_SHIFT   (1U << PMIC_PWR_CFG_EXT_VMON_INCLUDE_OV_UV_STAT_IN_PGOOD_VALID)
#define PMIC_PWR_CFG_EXT_VMON_ALL                                       (PMIC_PWR_CFG_EXT_VMON_MODE_VALID_SHIFT | \
                                                                         PMIC_PWR_CFG_EXT_VMON_THR_VALID_SHIFT | \
                                                                         PMIC_PWR_CFG_EXT_VMON_DGL_VALID_SHIFT | \
                                                                         PMIC_PWR_CFG_EXT_VMON_INCLUDE_OV_UV_STAT_IN_PGOOD_VALID_SHIFT)
/** @} */

/**
 * @anchor Pmic_PwrRsrcStatValidParams
 * @name PMIC Power Resource Status Valid Parameters
 *
 * @brief Valid parameters of the Pmic_PwrRsrcStat_t struct.
 *
 * @{
 */
#define PMIC_PWR_RSRC_STAT_OV_ERR_VALID         (0U)
#define PMIC_PWR_RSRC_STAT_UV_ERR_VALID         (1U)
#define PMIC_PWR_RSRC_STAT_ILIM_ERR_VALID       (2U)
#define PMIC_PWR_RSRC_STAT_TSD_ERR_VALID        (3U)
#define PMIC_PWR_RSRC_STAT_TSD_WARN_VALID       (4U)
#define PMIC_PWR_RSRC_STAT_BB_LITE_VALID        (5U)
#define PMIC_PWR_RSRC_STAT_BB_ILIM_LVL_VALID    (6U)
#define PMIC_PWR_RSRC_STAT_BB_MODE_VALID        (7U)
/** @} */

/**
 * @anchor Pmic_PwrRsrcStatValidParamShifts
 * @name PMIC Power Resource Status Valid Parameter Shifts
 *
 * @brief Valid parameter shifts of the Pmic_PwrRsrcStat_t struct. End user
 * can use the defines listed below to indicate valid power resource statuses.
 * Multiple valid parameters can be indicated using the OR operator.
 *
 * @{
 */
#define PMIC_PWR_RSRC_STAT_OV_ERR_VALID_SHIFT       (1U << PMIC_PWR_RSRC_STAT_OV_ERR_VALID)
#define PMIC_PWR_RSRC_STAT_UV_ERR_VALID_SHIFT       (1U << PMIC_PWR_RSRC_STAT_UV_ERR_VALID)
#define PMIC_PWR_RSRC_STAT_ILIM_ERR_VALID_SHIFT     (1U << PMIC_PWR_RSRC_STAT_ILIM_ERR_VALID)
#define PMIC_PWR_RSRC_STAT_TSD_ERR_VALID_SHIFT      (1U << PMIC_PWR_RSRC_STAT_TSD_ERR_VALID)
#define PMIC_PWR_RSRC_STAT_TSD_WARN_VALID_SHIFT     (1U << PMIC_PWR_RSRC_STAT_TSD_WARN_VALID)
#define PMIC_PWR_RSRC_STAT_BB_LITE_VALID_SHIFT      (1U << PMIC_PWR_RSRC_STAT_BB_LITE_VALID)
#define PMIC_PWR_RSRC_STAT_BB_ILIM_LVL_VALID_SHIFT  (1U << PMIC_PWR_RSRC_STAT_BB_ILIM_LVL_VALID)
#define PMIC_PWR_RSRC_STAT_BB_MODE_VALID_SHIFT      (1U << PMIC_PWR_RSRC_STAT_BB_MODE_VALID)
#define PMIC_PWR_RSRC_STAT_BB_ALL                   (PMIC_PWR_RSRC_STAT_OV_ERR_VALID_SHIFT | \
                                                     PMIC_PWR_RSRC_STAT_UV_ERR_VALID_SHIFT | \
                                                     PMIC_PWR_RSRC_STAT_ILIM_ERR_VALID_SHIFT | \
                                                     PMIC_PWR_RSRC_STAT_BB_LITE_VALID_SHIFT | \
                                                     PMIC_PWR_RSRC_STAT_BB_ILIM_LVL_VALID_SHIFT | \
                                                     PMIC_PWR_RSRC_STAT_BB_MODE_VALID_SHIFT | \
                                                     PMIC_PWR_RSRC_STAT_TSD_ERR_VALID_SHIFT | \
                                                     PMIC_PWR_RSRC_STAT_TSD_WARN_VALID_SHIFT)
#define PMIC_PWR_RSRC_STAT_LDO_ALL                  (PMIC_PWR_RSRC_STAT_OV_ERR_VALID_SHIFT | \
                                                     PMIC_PWR_RSRC_STAT_UV_ERR_VALID_SHIFT | \
                                                     PMIC_PWR_RSRC_STAT_ILIM_ERR_VALID_SHIFT | \
                                                     PMIC_PWR_RSRC_STAT_TSD_ERR_VALID_SHIFT | \
                                                     PMIC_PWR_RSRC_STAT_TSD_WARN_VALID_SHIFT)
#define PMIC_PWR_RSRC_STAT_PLDO_ALL                 (PMIC_PWR_RSRC_STAT_OV_ERR_VALID_SHIFT | \
                                                     PMIC_PWR_RSRC_STAT_UV_ERR_VALID_SHIFT | \
                                                     PMIC_PWR_RSRC_STAT_ILIM_ERR_VALID_SHIFT | \
                                                     PMIC_PWR_RSRC_STAT_TSD_ERR_VALID_SHIFT | \
                                                     PMIC_PWR_RSRC_STAT_TSD_WARN_VALID_SHIFT)
#define PMIC_PWR_RSRC_STAT_EXT_VMON_ALL             (PMIC_PWR_RSRC_STAT_OV_ERR_VALID_SHIFT | \
                                                     PMIC_PWR_RSRC_STAT_UV_ERR_VALID_SHIFT)
/** @} */

/**
 * @anchor Pmic_BbStbyLvl
 * @name PMIC Buck/Boost Standby Level
 *
 * @brief PMIC Buck/Boost voltage level for STANDBY mode. See `stbyLvl` member
 * of @ref Pmic_PwrBuckBoostCfg for more information.
 *
 * @{
 */
#define PMIC_PWR_BB_STBY_LVL_4V             (0U)
#define PMIC_PWR_BB_STBY_LVL_SAME_AS_BB_LVL (1U)
#define PMIC_PWR_BB_STBY_LVL_MAX            (PMIC_PWR_BB_STBY_LVL_SAME_AS_BB_LVL)
/** @} */

/**
 * @anchor Pmic_BbLvl
 * @name PMIC Buck/Boost Level
 *
 * @brief PMIC Buck/Boost voltage level for operating and sequencing states. See
 * `lvl` member of @ref Pmic_PwrBuckBoostCfg for more information.
 *
 * @{
 */
#define PMIC_PWR_BB_LVL_4P3V    (0U)
#define PMIC_PWR_BB_LVL_5V      (1U)
#define PMIC_PWR_BB_LVL_6V      (2U)
#define PMIC_PWR_BB_LVL_6P8V    (3U)
#define PMIC_PWR_BB_LVL_MAX     (PMIC_PWR_BB_LVL_6P8V)
/** @} */

/**
 * @anchor Pmic_Rt
 * @name PMIC Ramp Time
 *
 * @brief PMIC ramp time for soft-start. See `rampTime` member of
 * @ref Pmic_PwrLdoCfg and @ref Pmic_PwrPldoCfg for more information.
 *
 * @{
 */
#define PMIC_PWR_RT_SHORTER (0U)
#define PMIC_PWR_RT_LONGER  (1U)
#define PMIC_PWR_RT_MAX     (PMIC_PWR_RT_LONGER)
/** @} */

/**
 * @anchor Pmic_LdoIlimLvl
 * @name PMIC LDO Current Limit Level
 *
 * @brief PMIC LDO current limit level options. See `ilimLvl` member of
 * @ref Pmic_PwrLdoCfg. For more information, refer to device user guide or data
 * sheet. Search for LDOx_ILIM_LVL_CFG, where x=1,2,3,4.
 *
 * @{
 */
#define PMIC_PWR_LDO_ILIM_LVL_OPTION_0  (0U)
#define PMIC_PWR_LDO_ILIM_LVL_OPTION_1  (1U)
#define PMIC_PWR_LDO_ILIM_LVL_OPTION_2  (2U)
#define PMIC_PWR_LDO_ILIM_LVL_OPTION_3  (3U)
#define PMIC_PWR_LDO_ILIM_LVL_MAX       (PMIC_PWR_LDO_ILIM_LVL_OPTION_3)
/** @} */

/**
 * @anchor Pmic_LdoLvl
 * @name PMIC LDO Level
 *
 * @brief PMIC LDO and PLDO voltage levels for LDO mode, Bypass mode (for LDO
 * only), and VMON mode. See `lvl` member of @ref Pmic_PwrLdoCfg and
 * @ref Pmic_PwrPldoCfg for more information.
 *
 * @{
 */
#define PMIC_PWR_LDO_LVL_1V             (0x0U)
#define PMIC_PWR_LDO_LVL_1P05V          (0x1U)
#define PMIC_PWR_LDO_LVL_1P1V           (0x2U)
#define PMIC_PWR_LDO_LVL_1P15V          (0x3U)
#define PMIC_PWR_LDO_LVL_1P2V           (0x4U)
#define PMIC_PWR_LDO_LVL_1P25V          (0x5U)
#define PMIC_PWR_LDO_LVL_1P3V           (0x6U)
#define PMIC_PWR_LDO_LVL_1P35V          (0x7U)
#define PMIC_PWR_LDO_LVL_1P4V           (0x8U)
#define PMIC_PWR_LDO_LVL_1P45V          (0x9U)
#define PMIC_PWR_LDO_LVL_1P5V           (0xAU)
#define PMIC_PWR_LDO_LVL_1P55V          (0xBU)
#define PMIC_PWR_LDO_LVL_1P6V           (0xCU)
#define PMIC_PWR_LDO_LVL_1P65V          (0xDU)
#define PMIC_PWR_LDO_LVL_1P7V           (0xEU)
#define PMIC_PWR_LDO_LVL_1P75V          (0xFU)
#define PMIC_PWR_LDO_LVL_1P8V           (0x10U)
#define PMIC_PWR_LDO_LVL_2P5V           (0x11U)
#define PMIC_PWR_LDO_LVL_3V             (0x12U)
#define PMIC_PWR_LDO_LVL_3P3V           (0x13U)
#define PMIC_PWR_LDO_LVL_5V             (0x14U)
#define PMIC_PWR_LDO_LVL_BYPASS_MODE    (0x15U)
#define PMIC_PWR_LDO_LVL_MAX            (PMIC_PWR_LDO_LVL_BYPASS_MODE)
#define PMIC_PWR_PLDO_LVL_MAX           (PMIC_PWR_LDO_LVL_5V)
/** @} */

/**
 * @anchor Pmic_PldoIlimLvl
 * @name PMIC PLDO Current Limit Level
 *
 * @brief PMIC PLDO current limit level. See `ilimLvl` member of
 * @ref Pmic_PwrPldoCfg. For more information, refer to device user guide or
 * data sheet. Search for PLDOx_ILIM_LVL_CFG, where x=1,2.
 *
 * @{
 */
#define PMIC_PWR_PLDO_ILIM_LVL_OPTION_0 (0U)
#define PMIC_PWR_PLDO_ILIM_LVL_OPTION_1 (1U)
#define PMIC_PWR_PLDO_ILIM_LVL_OPTION_2 (2U)
#define PMIC_PWR_PLDO_ILIM_LVL_OPTION_3 (3U)
#define PMIC_PWR_PLDO_ILIM_LVL_MAX      (PMIC_PWR_PLDO_ILIM_LVL_OPTION_3)
/** @} */

/**
 * @anchor Pmic_VTrackRange
 * @name PMIC VTRACK Range
 *
 * @brief PMIC VTRACK voltage monitoring range. See `vtrackRange` member of
 * @ref Pmic_PwrPldoCfg for more information.
 *
 * @{
 */
#define PMIC_PWR_VTRACK_LT_2P2V     (0U)
#define PMIC_PWR_VTRACK_GTE_2P2V    (1U)
#define PMIC_PWR_VTRACK_RANGE_MAX   (PMIC_PWR_VTRACK_GTE_2P2V)
/** @} */

/**
 * @anchor Pmic_LdoMode
 * @name PMIC LDO Mode
 *
 * @brief PMIC LDO control of function, mode, and states. See `mode` member of
 * @ref Pmic_PwrLdoCfg for more information.
 *
 * @{
 */
#define PMIC_PWR_LDO_DISABLED                   (0U)
#define PMIC_PWR_LDO_EN_AS_LDO_IN_OPER          (1U)
#define PMIC_PWR_LDO_EN_AS_LDO_IN_OPER_AND_STBY (2U)
#define PMIC_PWR_LDO_EN_AS_VMON_IN_OPER         (3U)
#define PMIC_PWR_LDO_MODE_MAX                   (PMIC_PWR_LDO_EN_AS_VMON_IN_OPER)
/** @} */

/**
 * @anchor Pmic_PldoMode
 * @name PMIC PLDO Mode
 *
 * @brief PMIC PLDO control of function, mode, and states. See `mode` member of
 * @ref Pmic_PwrPldoCfg for more information.
 *
 * @{
 */
#define PMIC_PWR_PLDO_DISABLED                      (0U)
#define PMIC_PWR_PLDO_EN_AS_LDO_IN_OPER             (1U)
#define PMIC_PWR_PLDO_EN_AS_LDO_IN_OPER_AND_STBY    (2U)
#define PMIC_PWR_PLDO_EN_AS_VMON_IN_OPER            (3U)
#define PMIC_PWR_PLDO_EN_AS_ROTATION_COUNTER        (4U)
#define PMIC_PWR_PLDO1_MODE_MAX                     (PMIC_PWR_PLDO_EN_AS_VMON_IN_OPER) // Max user-settable value
#define PMIC_PWR_PLDO2_MODE_MAX                     (PMIC_PWR_PLDO_EN_AS_ROTATION_COUNTER) // Max user-settable value
/** @} */

/**
 * @anchor Pmic_LdoVmonThr
 * @name PMIC LDO Voltage Monitor Threshold
 *
 * @brief PMIC voltage monitoring thresholds for LDO. See `vmonThr` member of
 * @ref Pmic_PwrLdoCfg for more information.
 *
 * @{
 */
#define PMIC_PWR_LDO_VMON_THR_3P5_PCT   (0U)
#define PMIC_PWR_LDO_VMON_THR_4_PCT     (1U)
#define PMIC_PWR_LDO_VMON_THR_5_PCT     (2U)
#define PMIC_PWR_LDO_VMON_THR_6_PCT     (3U)
#define PMIC_PWR_LDO_VMON_THR_MAX       (PMIC_PWR_LDO_VMON_THR_6_PCT)
/** @} */

/**
 * @anchor Pmic_ExtVmonThr
 * @name PMIC External Voltage Monitor Threshold
 *
 * @brief PMIC External voltage monitoring thresholds. See `vmonThr` member of
 * @ref Pmic_PwrExtVmonCfg for more information.
 *
 * @{
 */
#define PMIC_PWR_EXT_VMON_THR_DIGITAL_MODE_HYSTERESIS   (0U)
#define PMIC_PWR_EXT_VMON_THR_4_PCT_NO_HYSTERESIS       (1U)
#define PMIC_PWR_EXT_VMON_THR_6_PCT_HYSTERESIS          (2U)
#define PMIC_PWR_EXT_VMON_THR_6_PCT_NO_HYSTERESIS       (3U)
#define PMIC_PWR_EXT_VMON_THR_8_PCT_HYSTERESIS          (4U)
#define PMIC_PWR_EXT_VMON_THR_8_PCT_NO_HYSTERESIS       (5U)
#define PMIC_PWR_EXT_VMON_THR_10_PCT_HYSTERESIS         (6U)
#define PMIC_PWR_EXT_VMON_THR_10_PCT_NO_HYSTERESIS      (7U)
#define PMIC_PWR_EXT_VMON_THR_MAX                       (PMIC_PWR_EXT_VMON_THR_10_PCT_NO_HYSTERESIS)
/** @} */

/**
 * @anchor Pmic_PldoVmonThr
 * @name PMIC PLDO Voltage Monitor Threshold
 *
 * @brief PMIC voltage monitoring thresholds for PLDO. See `vmonThr` member of
 * @ref Pmic_PwrPldoCfg for more information.
 *
 * @{
 */
#define PMIC_PWR_PLDO_VMON_THR_4_PCT    (0U)
#define PMIC_PWR_PLDO_VMON_THR_6_PCT    (1U)
#define PMIC_PWR_PLDO_VMON_THR_8_PCT    (2U)
#define PMIC_PWR_PLDO_VMON_THR_10_PCT   (3U)
#define PMIC_PWR_PLDO_VMON_THR_MAX      (PMIC_PWR_PLDO_VMON_THR_10_PCT)
/** @} */

/**
 * @anchor Pmic_BoostTmo
 * @name PMIC Boost Timeout
 *
 * @brief PMIC boost timeout for Buck/Boost. See `boostTmo` member of
 * @ref Pmic_PwrBuckBoostCfg for more information.
 *
 * @{
 */
#define PMIC_PWR_BOOST_TMO_NONE     (0U)
#define PMIC_PWR_BOOST_TMO_8_SEC    (1U)
#define PMIC_PWR_BOOST_TMO_12_SEC   (2U)
#define PMIC_PWR_BOOST_TMO_16_SEC   (3U)
#define PMIC_PWR_BOOST_TMO_MAX      (PMIC_PWR_BOOST_TMO_16_SEC)
/** @} */

/**
 * @anchor Pmic_BbVmonThr
 * @name PMIC Buck/Boost Voltage Monitor Thresholds
 *
 * @brief PMIC voltage monitoring thresholds for Buck/Boost. See `vmonThr`
 * member of @ref Pmic_PwrBuckBoostCfg for more information.
 *
 * @{
 */
#define PMIC_PWR_BB_VMON_THR_5_PCT  (0U)
#define PMIC_PWR_BB_VMON_THR_6_PCT  (1U)
#define PMIC_PWR_BB_VMON_THR_7_PCT  (2U)
#define PMIC_PWR_BB_VMON_THR_8_PCT  (3U)
#define PMIC_PWR_BB_VMON_THR_MAX    (PMIC_PWR_BB_VMON_THR_8_PCT)
/** @} */

/**
 * @anchor Pmic_VBatVmonDgl
 * @name PMIC VBAT Voltage Monitor Deglitch
 *
 * @brief Voltage monitor deglitch for VBAT.
 *
 * @{
 */
#define PMIC_PWR_VBAT_VMON_DGL_32_US    (0U)
#define PMIC_PWR_VBAT_VMON_DGL_64_US    (1U)
#define PMIC_PWR_VBAT_VMON_DGL_MAX      (PMIC_PWR_VBAT_VMON_DGL_64_US)
/** @} */

/**
 * @anchor Pmic_PwrRsrcVmonDgl
 * @name PMIC Power Resource Voltage Monitor Deglitch
 *
 * @brief Voltage monitor deglitch for PMIC power resources (buck/boost, LDO,
 * PLDO, external VMON). See `vmonDgl` of @ref Pmic_PwrBuckBoostCfg,
 * @ref Pmic_PwrLdoCfg, @ref Pmic_PwrPldoCfg, and @ref Pmic_PwrExtVmonCfg for
 * more information.
 *
 * @{
 */
#define PMIC_PWR_RSRC_VMON_DGL_8_US     (0U)
#define PMIC_PWR_RSRC_VMON_DGL_16_US    (1U)
#define PMIC_PWR_RSRC_VMON_DGL_24_US    (2U)
#define PMIC_PWR_RSRC_VMON_DGL_32_US    (3U)
#define PMIC_PWR_RSRC_VMON_DGL_MAX      (PMIC_PWR_RSRC_VMON_DGL_32_US)
/** @} */

/**
 * @anchor Pmic_ExtVmonMode
 * @name PMIC External Voltage Monitor Mode
 *
 * @brief PMIC external voltage monitor modes of operation. See `mode` member
 * of @ref Pmic_PwrExtVmonCfg for more information.
 *
 * @{
 */
#define PMIC_PWR_EXT_VMON_DISABLED              (0U)
#define PMIC_PWR_EXT_VMON_EN_IN_OPER            (1U)
#define PMIC_PWR_EXT_VMON_EN_IN_OPER_AND_STBY   (2U)
#define PMIC_PWR_EXT_VMON_MODE_MAX              (PMIC_PWR_EXT_VMON_EN_IN_OPER_AND_STBY) // Max user-settable value
/** @} */

/**
 * @anchor Pmic_LdoPldoIlimDeglitch
 * @name PMIC LDO and PLDO Current Limit Deglitch
 *
 * @brief Current Limit deglitch for LDOs and PLDOs. See `ilimDgl` member of
 * @ref Pmic_PwrLdoCfg and @ref Pmic_PwrPldoCfg for more information.
 *
 * @{
 */
#define PMIC_PWR_LDO_ILIM_DEGLITCH_10_US    (0U)
#define PMIC_PWR_LDO_ILIM_DEGLITCH_1_MS     (1U)
#define PMIC_PWR_LDO_ILIM_DEGLITCH_MAX      (PMIC_PWR_LDO_ILIM_DEGLITCH_1_MS)
/** @} */

/**
 * @anchor Pmic_BbIlimLvl
 * @name PMIC Buck/Boost Current Limit Level
 *
 * @brief Current limit level statuses for Buck/Boost. See `bbIlimLvl` member of
 * @ref Pmic_PwrRsrcStat for more information.
 *
 * @{
 */
#define PMIC_PWR_BB_ILIM_LVL_2P8A   (0U)
#define PMIC_PWR_BB_ILIM_LVL_1P5A   (1U)
#define PMIC_PWR_BB_ILIM_LVL_MAX    (PMIC_PWR_BB_ILIM_LVL_1P5A)
/** @} */

/**
 * @anchor Pmic_BbMode
 * @name PMIC Buck/Boost Mode
 *
 * @brief Mode statuses for Buck/Boost. See `bbMode` member of
 * @ref Pmic_PwrRsrcStat for more information.
 *
 * @{
 */
#define PMIC_PWR_BB_MODE_BUCK   (0U)
#define PMIC_PWR_BB_MODE_BOOST  (1U)
#define PMIC_PWR_BB_MODE_MAX    (PMIC_PWR_BB_MODE_BOOST)
/** @} */

/* ========================================================================== */
/*                            Structures and Enums                            */
/* ========================================================================== */

/**
 * @anchor Pmic_PwrBuckBoostCfg
 * @name PMIC Power Buck/Boost Configuration
 *
 * @brief Struct used to set/get Buck/Boost configurations.
 *
 * @param validParams For valid values, refer to @ref Pmic_PwrBuckBoostCfgValidParamShift.
 *
 * @param lvl Voltage level for operating and sequencing states. For valid
 * values, refer to @ref Pmic_BbLvl.
 *
 * @param stbyLvl Voltage level for low-power STANDBY state operation. For valid
 * values, refer to @ref Pmic_BbStbyLvl.
 *
 * @param vmonThr VMON threshold. For valid values, refer to @ref Pmic_BbVmonThr.
 *
 * @param vmonDgl VMON deglitch. For valid values, refer to @ref Pmic_PwrRsrcVmonDgl.
 *
 * @param boostTmo Configuration for how long the Buck/Boost can operate in Boost
 * mode before a timeout to OFF state. For valid values, refer to @ref Pmic_BoostTmo.
 *
 * @param ssEn Dual random spread spectrum (DRSS) modulation enable.
 *
 * @param includeOvUvStatInPGood Include Buck/Boost OV and UV status in PGOOD
 * output status.
 */
typedef struct Pmic_PwrBuckBoostCfg_s {
    uint32_t validParams;

    uint8_t lvl;
    uint8_t stbyLvl;
    uint8_t vmonThr;
    uint8_t vmonDgl;
    uint8_t boostTmo;
    bool ssEn;
    bool includeOvUvStatInPGood;
} Pmic_PwrBuckBoostCfg_t;

/**
 * @anchor Pmic_PwrLdoCfg
 * @name PMIC Power LDO Configuration
 *
 * @brief Struct used to set/get LDO configurations.
 *
 * @attention `ldo` parameter must be specified.
 *
 * @param validParams For valid values, refer to @ref Pmic_PwrLdoCfgValidParamShifts.
 *
 * @param ldo LDO identifier. For valid values, refer to @ref Pmic_PwrRsrc.
 *
 * @param mode Control LDO function, mode, and states. For valid values, refer
 * to @ref Pmic_LdoMode.
 *
 * @param lvl Voltage level for operating in LDO mode, Bypass mode, or VMON mode.
 * For valid values, refer to @ref Pmic_LdoLvl.
 *
 * @param ilimLvl Current limit level. For valid values, refer to
 * @ref Pmic_LdoIlimLvl.
 *
 * @param ilimDgl Current limit deglitch. For valid values, refer to
 * @ref Pmic_LdoPldoIlimDeglitch.
 *
 * @param vmonThr VMON threshold. For valid values, refer to @ref Pmic_PldoVmonThr.
 *
 * @param vmonDgl VMON deglitch. For valid values, refer to @ref Pmic_PwrRsrcVmonDgl.
 *
 * @param rampTime Ramp time for soft-start. For valid values, refer to
 * @ref Pmic_Rt.
 *
 * @param disableDischarge Disable LDO discharge pull-down.
 *
 * @param includeOvUvStatInPGood Include LDO OV and UV status in PGOOD
 * output status.
 */
typedef struct Pmic_PwrLdoCfg_s {
    uint32_t validParams;
    uint16_t ldo;

    uint8_t mode;
    uint8_t lvl;
    uint8_t ilimLvl;
    uint8_t ilimDgl;
    uint8_t vmonThr;
    uint8_t vmonDgl;
    uint8_t rampTime;
    bool disableDischarge;
    bool includeOvUvStatInPGood;
} Pmic_PwrLdoCfg_t;

/**
 * @anchor Pmic_PwrPldoCfg
 * @name PMIC Power PLDO Configuration
 *
 * @brief Struct used to set/get PLDO configurations.
 *
 * @attention `pldo` parameter must be specified.
 *
 * @param validParams For valid values, refer to @ref Pmic_PwrPldoCfgValidParamShifts.
 *
 * @param pldo PLDO identifier. For valid values, refer to @ref Pmic_PwrRsrc.
 *
 * @param mode Control PLDO function, mode, and states. For valid values, refer
 * to @ref Pmic_PldoMode.
 *
 * @param trackingMode PLDO tracking mode enable. When true, PLDO is in tracking
 * mode during operating states, and the output voltage tracks the TRACK pin.
 * Otherwise, PLDO is in fixed output (non-tracking) mode during operating states,
 * and the output voltage is determined by the `lvl` parameter.
 *
 * @param lvl Voltage level for operating in Fixed Output Voltage LDO mode or
 * VMON mode. For valid values, refer to @ref Pmic_LdoLvl.
 *
 * @param ilimLvl Current limit level. For valid values, refer to
 * @ref Pmic_PldoIlimLvl.
 *
 * @param ilimDgl Current limit deglitch. For valid values, refer to
 * @ref Pmic_LdoPldoIlimDeglitch.
 *
 * @param vmonThr VMON threshold. For valid values, refer to @ref Pmic_PldoVmonThr.
 *
 * @param vmonDgl VMON deglitch. For valid values, refer to @ref Pmic_PwrRsrcVmonDgl.
 *
 * @param vtrackRange Voltage monitoring range for the VTRACK pin. For valid
 * values, refer to @ref Pmic_VTrackRange.
 *
 * @param rampTime Ramp time for soft-start. For valid values, refer to
 * @ref Pmic_Rt.
 *
 * @param disableDischarge Disable PLDO discharge pull-down.
 *
 * @param includeOvUvStatInPGood Include PLDO OV and UV status in PGOOD
 * output status.
 */
typedef struct Pmic_PwrPldoCfg_s {
    uint32_t validParams;
    uint16_t pldo;

    uint8_t mode;
    bool trackingMode;
    uint8_t lvl;
    uint8_t ilimLvl;
    uint8_t ilimDgl;
    uint8_t vmonThr;
    uint8_t vmonDgl;
    uint8_t vtrackRange;
    uint8_t rampTime;
    bool disableDischarge;
    bool includeOvUvStatInPGood;
} Pmic_PwrPldoCfg_t;

/**
 * @anchor Pmic_PwrExtVmonCfg
 * @name PMIC Power External VMON Configuration
 *
 * @brief Struct used to set/get external VMON configurations.
 *
 * @attention `extVmon` parameter must be specified.
 *
 * @param validParams For valid values, refer to @ref Pmic_PwrExtVmonCfgValidParamShifts.
 *
 * @param extVmon External VMON identifier. For valid values, refer to @ref Pmic_PwrRsrc.
 *
 * @param mode Control external VMON operation. For valid values, refer to
 * @ref Pmic_ExtVmonMode.
 *
 * @param vmonThr VMON threshold. For valid values, refer to @ref Pmic_ExtVmonThr.
 *
 * @param vmonDgl VMON deglitch. For valid values, refer to @ref Pmic_PwrRsrcVmonDgl.
 *
 * @param includeOvUvStatInPGood Include external VMON OV and UV status in PGOOD
 * output status.
 */
typedef struct Pmic_PwrExtVmonCfg_s {
    uint32_t validParams;
    uint16_t extVmon;

    uint8_t mode;
    uint8_t vmonThr;
    uint8_t vmonDgl;
    bool includeOvUvStatInPGood;
} Pmic_PwrExtVmonCfg_t;

/**
 * @anchor Pmic_PwrRsrcStat
 * @name PMIC Power Resource Status
 *
 * @brief Struct used to get/clear the status of a power resource (Buck/Boost,
 * LDO, PLDO, VMON).
 *
 * @attention `pwrRsrc` parameter must be specified.
 *
 * @param validParams For valid values, refer to @ref Pmic_PwrRsrcStatValidParams.
 *
 * @param pwrRsrc Power resource identifier. For valid values, refer to
 * @ref Pmic_PwrRsrc.
 *
 * @param ovErr Over voltage error. Applicable to all regulators and VMONs.
 * When returned as true, an over voltage error was detected for the power
 * resource.
 *
 * @param uvErr Under voltage error. Applicable to all regulators and VMONs.
 * When returned as true, an under voltage error was detected for the power
 * resource.
 *
 * @param ilimErr Current limit error. Only applicable to Buck/Boost, LDOs,
 * and PLDOs. When returned as true, a current limit error was detected for the
 * power resource.
 *
 * @param tsdErr Thermal shutdown error. When returned as true, the device has
 * shutdown due to overtemperature.
 *
 * @param tsdWarn Thermal shutdown warning. When returned as true, there is an
 * imminent overtemperature.
 *
 * @param bbLite Buck/Boost "lite" device status indicator. When returned as
 * true, the PMIC device has limited boost operating range.
 *
 * @param bbIlimLvl Buck/Boost current limit level status. For valid values that
 * can be returned, see @ref Pmic_BbIlimLvl.
 *
 * @param bbMode Boost mode operation status for Buck/Boost. For valid values that
 * can be returned, see @ref Pmic_BbMode.
 */
typedef struct Pmic_PwrRsrcStat_s {
    uint32_t validParams;
    uint16_t pwrRsrc;

    /* All regulators and VMONs */
    bool ovErr;
    bool uvErr;

    /* Buck/Boost LDO, and PLDO only */
    bool ilimErr;
    bool tsdErr;
    bool tsdWarn;

    /* Buck/Boost only */
    bool bbLite;
    uint8_t bbIlimLvl;
    uint8_t bbMode;
} Pmic_PwrRsrcStat_t;

/* ========================================================================== */
/*                           Function Declarations                            */
/* ========================================================================== */

/**
 * @brief Set PMIC Buck/Boost configurations.
 *
 * @details The following options are configurable via this API
 * 1. Voltage level (validParam: PMIC_PWR_CFG_BB_LVL_VALID_SHIFT)
 * 2. Standby voltage level (validParam: PMIC_PWR_CFG_BB_STBY_LVL_VALID_SHIFT)
 * 3. VMON threshold (validParam: PMIC_PWR_CFG_BB_VMON_THR_VALID_SHIFT)
 * 4. VMON deglitch (validParam: PMIC_PWR_CFG_BB_VMON_DGL_VALID_SHIFT)
 * 5. Boost timeout (validParam: PMIC_PWR_CFG_BB_BOOST_TMO_VALID_SHIFT)
 * 6. Spread spectrum enable (validParam: PMIC_PWR_CFG_BB_SS_EN_VALID_SHIFT)
 * 7. Include OV/UV status in PGOOD (validParam: PMIC_PWR_CFG_BB_INCLUDE_OV_UV_STAT_IN_PGOOD_VALID_SHIFT)
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param buckBoostCfg [IN] Buck/Boost configurations to be set.
 *
 * @return Success code if PMIC Buck/Boost configurations have been set, error
 * code otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes
 */
int32_t Pmic_pwrSetBuckBoostCfg(Pmic_CoreHandle_t *handle, const Pmic_PwrBuckBoostCfg_t *buckBoostCfg);

/**
 * @brief Get PMIC Buck/Boost configurations. This API supports getting the same
 * configurations that are settable through Pmic_pwrSetBuckBoostCfg().
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param buckBoostCfg [OUT] Buck/Boost configurations obtained from the PMIC.
 *
 * @return Success code if PMIC Buck/Boost configurations have been obtained,
 * error code otherwise. For valid success/error codes, refer to
 * @ref Pmic_ErrorCodes
 */
int32_t Pmic_pwrGetBuckBoostCfg(Pmic_CoreHandle_t *handle, Pmic_PwrBuckBoostCfg_t *buckBoostCfg);

/**
 * @brief Set PMIC LDO configurations.
 *
 * @details The following options are configurable via this API
 * 1. LDO mode (validParam: PMIC_PWR_CFG_LDO_MODE_VALID_SHIFT)
 * 2. Voltage level (validParam: PMIC_PWR_CFG_LDO_LVL_VALID_SHIFT)
 * 3. Current limit level (validParam: PMIC_PWR_CFG_LDO_ILIM_LVL_VALID_SHIFT)
 * 4. Current limit deglitch (validParam: PMIC_PWR_CFG_LDO_ILIM_DGL_VALID_SHIFT)
 * 5. VMON threshold (validParam: PMIC_PWR_CFG_LDO_VMON_THR_VALID_SHIFT)
 * 6. VMON deglitch (validParam: PMIC_PWR_CFG_LDO_VMON_DGL_VALID_SHIFT)
 * 7. Ramp time (validParam: PMIC_PWR_CFG_LDO_RAMP_TIME_VALID_SHIFT)
 * 8. Disable discharge (validParam: PMIC_PWR_CFG_LDO_DISABLE_DISCHARGE_VALID_SHIFT)
 * 9. Include OV/UV status in PGOOD (validParam: PMIC_PWR_CFG_LDO_INCLUDE_OV_UV_STAT_IN_PGOOD_VALID_SHIFT)
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param ldoCfg [IN] LDO configurations to be set. Note that `ldo` parameter
 * must be specified.
 *
 * @return Success code if PMIC LDO configurations have been set, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes
 */
int32_t Pmic_pwrSetLdoCfg(Pmic_CoreHandle_t *handle, const Pmic_PwrLdoCfg_t *ldoCfg);

/**
 * @brief Get PMIC LDO configurations. This API supports getting the same
 * configurations that are settable through Pmic_pwrSetLdoCfg().
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param ldoCfg [OUT] LDO configurations obtained from the PMIC. Note that
 * `ldo` parameter must be specified.
 *
 * @return Success code if PMIC LDO configurations have been obtained, error
 * code otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes
 */
int32_t Pmic_pwrGetLdoCfg(Pmic_CoreHandle_t *handle, Pmic_PwrLdoCfg_t *ldoCfg);

/**
 * @brief Set PMIC PLDO configurations.
 *
 * @details The following options are configurable via this API
 * 1. PLDO mode (validParam: PMIC_PWR_CFG_PLDO_MODE_VALID_SHIFT)
 * 2. PLDO tracking mode (validParam: PMIC_PWR_CFG_PLDO_TRACKING_MODE_VALID_SHIFT)
 * 3. Voltage level (validParam: PMIC_PWR_CFG_PLDO_LVL_VALID_SHIFT)
 * 4. Current limit level (validParam: PMIC_PWR_CFG_PLDO_ILIM_LVL_VALID_SHIFT)
 * 5. Current limit deglitch (validParam: PMIC_PWR_CFG_PLDO_ILIM_DGL_VALID_SHIFT)
 * 6. VMON threshold (validParam: PMIC_PWR_CFG_PLDO_VMON_THR_VALID_SHIFT)
 * 7. VMON deglitch (validParam: PMIC_PWR_CFG_PLDO_VMON_DGL_VALID_SHIFT)
 * 8. VTRACK range (validParam: PMIC_PWR_CFG_PLDO_VTRACK_RANGE_VALID_SHIFT)
 * 9. Ramp time (validParam: PMIC_PWR_CFG_PLDO_RT_VALID_SHIFT)
 * 10. Disable discharge (validParam: PMIC_PWR_CFG_PLDO_DISABLE_DISCHARGE_VALID_SHIFT)
 * 11. Include OV/UV status in PGOOD (validParam: PMIC_PWR_CFG_PLDO_INCLUDE_OV_UV_STAT_IN_PGOOD_VALID_SHIFT)
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param pldoCfg [IN] PLDO configurations to be set. Note that the `pldo`
 * parameter must be specified.
 *
 * @return Success code if PLDO configurations have been set, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes
 */
int32_t Pmic_pwrSetPldoCfg(Pmic_CoreHandle_t *handle, const Pmic_PwrPldoCfg_t *pldoCfg);

/**
 * @brief Get PMIC PLDO configurations. This API supports getting the same
 * configurations that are settable through Pmic_pwrSetPldoCfg().
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param pldoCfg [OUT] PLDO configurations to be set. Note that the `pldo`
 * parameter must be specified.
 *
 * @return Success code if PLDO configurations have been obtained, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes
 */
int32_t Pmic_pwrGetPldoCfg(Pmic_CoreHandle_t *handle, Pmic_PwrPldoCfg_t *pldoCfg);

/**
 * @brief Set PMIC external VMON configurations.
 *
 * @details The following options are configurable via this API
 * 1. External VMON mode (validParam: PMIC_PWR_CFG_EXT_VMON_MODE_VALID_SHIFT)
 * 2. VMON threshold (validParam: PMIC_PWR_CFG_EXT_VMON_THR_VALID_SHIFT)
 * 3. VMON deglitch (validParam: PMIC_PWR_CFG_EXT_VMON_DGL_VALID_SHIFT)
 * 4. Include OV/UV status in PGOOD (validParam: PMIC_PWR_CFG_EXT_VMON_INCLUDE_OV_UV_STAT_IN_PGOOD_VALID_SHIFT)
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param extVmonCfg [IN] External VMON configurations to be set. Note that the
 * `extVmon` parameter must be specified.
 *
 * @return Success code if external VMON configurations have been set, error
 * code otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes
 */
int32_t Pmic_pwrSetExtVmonCfg(Pmic_CoreHandle_t *handle, const Pmic_PwrExtVmonCfg_t *extVmonCfg);

/**
 * @brief Get PMIC external VMON configurations. This API supports getting the
 * same configurations that are settable through Pmic_pwrSetExtVmonCfg().
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param extVmonCfg [OUT] External VMON configurations obtained from the PMIC.
 * Note that the `extVmon` parameter must be specified.
 *
 * @return Success code if external VMON configurations have been set, error
 * code otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes
 */
int32_t Pmic_pwrGetExtVmonCfg(Pmic_CoreHandle_t *handle, Pmic_PwrExtVmonCfg_t *extVmonCfg);

/**
 * @brief Get PMIC power resource statuses.
 *
 * @attention Certain power resources do not have certain statuses. Please see
 * @ref Pmic_PwrRsrcStat for more information.
 *
 * @details Depending on the power resource specified, the following statuses
 * can be obtained via this API
 * 1. Over-voltage error (validParam: PMIC_PWR_RSRC_STAT_OV_ERR_VALID_SHIFT)
 * 2. Under-voltage error (validParam: PMIC_PWR_RSRC_STAT_UV_ERR_VALID_SHIFT)
 * 3. Current limit error (validParam: PMIC_PWR_RSRC_STAT_ILIM_ERR_VALID_SHIFT)
 * 4. Buck/Boost lite (validParam: PMIC_PWR_RSRC_STAT_BB_LITE_VALID_SHIFT)
 * 5. Buck/Boost current limit level (validParam: PMIC_PWR_RSRC_STAT_BB_ILIM_LVL_VALID_SHIFT)
 * 6. Buck/Boost mode (validParam: PMIC_PWR_RSRC_STAT_BB_MODE_VALID_SHIFT)
 * 7. Thermal shutdown error (validParam: PMIC_PWR_RSRC_STAT_TSD_ERR_VALID_SHIFT)
 * 8. Thermal shutdown warning (validParam: PMIC_PWR_RSRC_STAT_TSD_WARN_VALID_SHIFT)
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param pwrRsrcStat [OUT] Power resource status. Note that the `pwrRsrc`
 * parameter must be specified.
 *
 * @return Success code if power resource statuses have been obtained, error
 * code otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes
 */
int32_t Pmic_pwrGetRsrcStat(Pmic_CoreHandle_t *handle, Pmic_PwrRsrcStat_t *pwrRsrcStat);

/**
 * @brief Clear PMIC power resource statuses.
 *
 * @details The following statuses can be cleared via this API depending on the
 * specified power resource
 * 1. Over-voltage error (validParam: PMIC_PWR_RSRC_STAT_OV_ERR_VALID_SHIFT)
 * 2. Under-voltage error (validParam: PMIC_PWR_RSRC_STAT_UV_ERR_VALID_SHIFT)
 * 3. Current limit error (validParam: PMIC_PWR_RSRC_STAT_ILIM_ERR_VALID_SHIFT)
 * 4. Buck/Boost mode (validParam: PMIC_PWR_RSRC_STAT_BB_MODE_VALID_SHIFT)
 * 5. Thermal shutdown error (validParam: PMIC_PWR_RSRC_STAT_TSD_ERR_VALID_SHIFT)
 * 6. Thermal shutdown warning (validParam: PMIC_PWR_RSRC_STAT_TSD_WARN_VALID_SHIFT)
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param pwrRsrcStat [IN] Power resource statuses to be cleared. `validParams`
 * indicates the desired statuses to be cleared; all other parameters except for
 * `pwrRsrc` are ignored by this API. Note that the `pwrRsrc` parameter must be
 * specified.
 *
 * @return Success code if power resource statuses have been cleared, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes
 */
int32_t Pmic_pwrClrRsrcStat(Pmic_CoreHandle_t *handle, const Pmic_PwrRsrcStat_t *pwrRsrcStat);

/**
 * @brief Clear all PMIC power resource statuses. This API clears all statuses
 * that are clearable through Pmic_pwrClrRsrcStat().
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @return Success code if all power resources have been cleared, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes
 */
int32_t Pmic_pwrClrRsrcStatAll(Pmic_CoreHandle_t *handle);

/**
 * @brief Enable or disable PGOOD from being active in STANDBY state.
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param enable [IN] True - PGOOD is active in STANDBY state and holds level
 * when PMIC enters STANDBY state. False - PGOOD is not active in STANDBY state
 * and outputs low.
 *
 * @return Success code if PGOOD is configured, error code otherwise. For valid
 * success/error codes, refer to @ref Pmic_ErrorCodes
 */
int32_t Pmic_pwrSetPGoodInStby(Pmic_CoreHandle_t *handle, bool enable);

/**
 * @brief Get PMIC PGOOD enable/disable in STANDBY state.
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param isEnabled [OUT] True - PGOOD is active in STANDBY state and holds
 * level when PMIC enters STANDBY state. False - PGOOD is not active in STANDBY
 * state and outputs low.
 *
 * @return Success code if PGOOD enable/disable in STANDBY state has been
 * obtained, error code otherwise. For valid success/error codes, refer to
 * @ref Pmic_ErrorCodes
 */
int32_t Pmic_pwrGetPGoodInStby(Pmic_CoreHandle_t *handle, bool *isEnabled);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __PMIC_POWER_H__ */
