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
#ifndef __PMIC_REGMAP_GPIO_H__
#define __PMIC_REGMAP_GPIO_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief PMIC GPIO module register addresses */
#define RDBK_LVL_STAT_REG  (0x0AU)
#define GPO_CFG1_REG       (0x7CU)
#define GPO_CFG2_REG       (0x7DU)
#define GPI_CFG_REG        (0x7EU)
#define RDBK_DGL_CFG1_REG  (0x7FU)
#define RDBK_DGL_CFG2_REG  (0x80U)
#define RDBK_DGL_CFG3_REG  (0x81U)
#define RDBK_DGL_CFG4_REG  (0x82U)

/** @brief PMIC Readback Level Status (RDBK_LVL_STAT) */
#define GPO1_RDBK_LVL_SHIFT    (3U)
#define GPO2_RDBK_LVL_SHIFT    (4U)
#define GPO3_RDBK_LVL_SHIFT    (5U)
#define GPO4_RDBK_LVL_SHIFT    (6U)
#define GPO1_RDBK_LVL_MASK     ((uint8_t)(1U << GPO1_RDBK_LVL_SHIFT))
#define GPO2_RDBK_LVL_MASK     ((uint8_t)(1U << GPO2_RDBK_LVL_SHIFT))
#define GPO3_RDBK_LVL_MASK     ((uint8_t)(1U << GPO3_RDBK_LVL_SHIFT))
#define GPO4_RDBK_LVL_MASK     ((uint8_t)(1U << GPO4_RDBK_LVL_SHIFT))

/** @brief PMIC General Purpose Outout Configuration 1 (GPO_CFG1) */
#define GPO1_CFG_SHIFT         (0U)
#define GPO2_CFG_SHIFT         (3U)
#define M_PMIC_CFG_SHIFT       (6U)
#define GPO1_CFG_MASK          ((uint8_t)(7U << GPO1_CFG_SHIFT))
#define GPO2_CFG_MASK          ((uint8_t)(7U << GPO2_CFG_SHIFT))
#define M_PMIC_CFG_MASK        ((uint8_t)(1U << M_PMIC_CFG_SHIFT))

/** @brief PMIC General Purpose Output configuration 2 (GPO_CFG2) */
#define GPO3_CFG_SHIFT         (0U)
#define GPO4_CFG_SHIFT         (3U)
#define EN_OUT_PU_CFG_SHIFT    (6U)
#define GPO3_CFG_MASK          ((uint8_t)(7U << GPO3_CFG_SHIFT))
#define GPO4_CFG_MASK          ((uint8_t)(7U << GPO4_CFG_SHIFT))
#define EN_OUT_PU_CFG_MASK     ((uint8_t)(3U << EN_OUT_PU_CFG_SHIFT))

/** @brief PMIC General Purpose Input Configuration (GPI_CFG) */
#define GPI1_CFG_SHIFT         (0U)
#define GPI4_CFG_SHIFT         (1U)
#define GPI1_CFG_MASK          ((uint8_t)(1U << GPI1_CFG_SHIFT))
#define GPI4_CFG_MASK          ((uint8_t)(3U << GPI4_CFG_SHIFT))

/** @brief PMIC Readback Deglitch Configuration 1 (RDBK_DGL_CFG1) */
#define NRST_RDBK_F_DGL_CFG_SHIFT      (0U)
#define NRST_RDBK_R_DGL_CFG_SHIFT      (2U)
#define SAFE_OUT1_RDBK_F_DGL_CFG_SHIFT (4U)
#define SAFE_OUT1_RDBK_R_DGL_CFG_SHIFT (6U)
#define NRST_RDBK_F_DGL_CFG_MASK       ((uint8_t)(3U << NRST_RDBK_F_DGL_CFG_SHIFT))
#define NRST_RDBK_R_DGL_CFG_MASK       ((uint8_t)(3U << NRST_RDBK_R_DGL_CFG_SHIFT))
#define SAFE_OUT1_RDBK_F_DGL_CFG_MASK  ((uint8_t)(3U << SAFE_OUT1_RDBK_F_DGL_CFG_SHIFT))
#define SAFE_OUT1_RDBK_R_DGL_CFG_MASK  ((uint8_t)(3U << SAFE_OUT1_RDBK_R_DGL_CFG_SHIFT))

/** @brief PMIC Readback Deglitch Configuration 2 (RDBK_DGL_CFG2) */
#define GPO1_RDBK_F_DGL_CFG_SHIFT      (0U)
#define GPO1_RDBK_R_DGL_CFG_SHIFT      (2U)
#define GPO2_RDBK_F_DGL_CFG_SHIFT      (4U)
#define GPO2_RDBK_R_DGL_CFG_SHIFT      (6U)
#define GPO1_RDBK_F_DGL_CFG_MASK       ((uint8_t)(3U << GPO1_RDBK_F_DGL_CFG_SHIFT))
#define GPO1_RDBK_R_DGL_CFG_MASK       ((uint8_t)(3U << GPO1_RDBK_R_DGL_CFG_SHIFT))
#define GPO2_RDBK_F_DGL_CFG_MASK       ((uint8_t)(3U << GPO2_RDBK_F_DGL_CFG_SHIFT))
#define GPO2_RDBK_R_DGL_CFG_MASK       ((uint8_t)(3U << GPO2_RDBK_R_DGL_CFG_SHIFT))

/** @brief PMIC Readback Deglitch Configuration 3 (RDBK_DGL_CFG3) */
#define GPO3_RDBK_F_DGL_CFG_SHIFT      (0U)
#define GPO3_RDBK_R_DGL_CFG_SHIFT      (2U)
#define GPO4_RDBK_F_DGL_CFG_SHIFT      (4U)
#define GPO4_RDBK_R_DGL_CFG_SHIFT      (6U)
#define GPO3_RDBK_F_DGL_CFG_MASK       ((uint8_t)(3U << GPO3_RDBK_F_DGL_CFG_SHIFT))
#define GPO3_RDBK_R_DGL_CFG_MASK       ((uint8_t)(3U << GPO3_RDBK_R_DGL_CFG_SHIFT))
#define GPO4_RDBK_F_DGL_CFG_MASK       ((uint8_t)(3U << GPO4_RDBK_F_DGL_CFG_SHIFT))
#define GPO4_RDBK_R_DGL_CFG_MASK       ((uint8_t)(3U << GPO4_RDBK_R_DGL_CFG_SHIFT))

/** @brief PMIC Readback Deglitch Configuration 4 (RDBK_DGL_CFG4) */
#define EN_OUT_RDBK_F_DGL_CFG_SHIFT    (0U)
#define EN_OUT_RDBK_R_DGL_CFG_SHIFT    (2U)
#define EN_OUT_RDBK_R_DGL_CFG_MASK     ((uint8_t)(3U << EN_OUT_RDBK_R_DGL_CFG_SHIFT))
#define EN_OUT_RDBK_F_DGL_CFG_MASK     ((uint8_t)(3U << EN_OUT_RDBK_F_DGL_CFG_SHIFT))

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* __PMIC_REGMAP_GPIO_H__ */
