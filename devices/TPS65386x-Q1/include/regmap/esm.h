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
#ifndef __PMIC_REGMAP_ESM_H__
#define __PMIC_REGMAP_ESM_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief PMIC ESM module register addresses */
#define ESM_CTRL_REG       (0x47U)
#define ESM_CFG1_REG       (0x48U)
#define ESM_CFG2_REG       (0x49U)
#define ESM_DELAY1_REG     (0x4BU)
#define ESM_DELAY2_REG     (0x4CU)
#define ESM_HMAX_CFG_REG   (0x4DU)
#define ESM_HMIN_CFG_REG   (0x4EU)
#define ESM_LMAX_CFG_REG   (0x4FU)
#define ESM_LMIN_CFG_REG   (0x50U)
#define ESM_ERR_STAT_REG   (0x51U)

/** @brief ESM_CTRL - ESM Control */
#define ESM_START_SHIFT    (0U)
#define ESM_START_MASK     (1U << ESM_START_SHIFT)

/** @brief ESM_CFG1 - ESM Configuration 1 */
#define ESM_ERR_TH_SHIFT   (0U)
#define ESM_EN_SHIFT       (6U)
#define ESM_CFG_SHIFT      (7U)
#define ESM_ERR_TH_MASK    (0xFU << ESM_ERR_TH_SHIFT)
#define ESM_EN_MASK        (1U << ESM_EN_SHIFT)
#define ESM_CFG_MASK       (1U << ESM_CFG_SHIFT)

/** @brief ESM_CFG2 - ESM Configuration 2 */
#define ESM_TIME_CFG_SHIFT (0U)
#define ESM_DGL_SHIFT      (3U)
#define ESM_LVL_POL_SHIFT  (4U)
#define ESM_TIME_CFG_MASK  (7U << ESM_TIME_CFG_SHIFT)
#define ESM_DGL_MASK       (1U << ESM_DGL_SHIFT)
#define ESM_LVL_POL_MASK   (1U << ESM_LVL_POL_SHIFT)

/** @brief ESM_DELAY1 - ESM Delay 1 */
#define ESM_DLY1_SHIFT (0U)
#define ESM_DLY1_MASK  (0xFFU << ESM_DLY1)

/** @brief ESM_DELAY2 - ESM Delay 2 */
#define ESM_DLY2_SHIFT (0U)
#define ESM_DLY2_MASK  (0xFFU << ESM_DLY2_SHIFT)

/** @brief ESM_HMAX_CFG - ESM High Maximum Configuration */
#define ESM_HMAX_SHIFT (0U)
#define ESM_HMAX_MASK  (0xFFU << ESM_HMAX_SHIFT)

/** @brief ESM_HMIN_CFG - ESM High Minimum Configuration */
#define ESM_HMIN_SHIFT (0U)
#define ESM_HMIN_MASK  (0xFFU << ESM_HMIN_SHIFT)

/** @brief ESM_LMAX_CFG - ESM Low Maximum Configuration */
#define ESM_LMAX_SHIFT (0U)
#define ESM_LMAX_MASK  (0xFFU << ESM_LMAX_SHIFT)

/** @brief ESM_LMIN_CFG - ESM Low Minimum Configuration */
#define ESM_LMIN_SHIFT (0U)
#define ESM_LMIN_MASK  (0xFFU << ESM_LMIN_SHIFT)

/** @brief ESM_ERR_STAT - ESM Error Status */
#define ESM_ERR_CNT_SHIFT  (0U)
#define ESM_ERR_SHIFT      (5U)
#define ESM_DLY1_ERR_SHIFT (6U)
#define ESM_DLY2_ERR_SHIFT (7U)
#define ESM_ERR_CNT_MASK   (0x1FU << ESM_ERR_CNT_SHIFT)
#define ESM_ERR_MASK       (1U << ESM_ERR_SHIFT)
#define ESM_DLY1_ERR_MASK  (1U << ESM_DLY1_ERR_SHIFT)
#define ESM_DLY2_ERR_MASK  (1U << ESM_DLY2_ERR_SHIFT)

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* __PMIC_REGMAP_ESM_H__ */
