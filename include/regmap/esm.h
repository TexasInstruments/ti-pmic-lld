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
#ifndef PMIC_REGMAP_ESM_H
#define PMIC_REGMAP_ESM_H

/**
 * @file esm.h
 *
 * @brief PMIC LLD ESM module register map definitions.
 */

/* ========================================================================== */
/*                                Include Files                               */
/* ========================================================================== */

#include "pmic_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                        LP8774x ESM Module Register Map                     */
/* ========================================================================== */

// ESM module register addresses
#define ESM_START_REG    (0x09U)
#define ESM_DELAY1_REG   (0x39U)
#define ESM_DELAY2_REG   (0x3AU)
#define ESM_MODE_CFG_REG (0x3BU)
#define ESM_HMAX_REG     (0x3CU)
#define ESM_HMIN_REG     (0x3DU)
#define ESM_LMAX_REG     (0x3EU)
#define ESM_LMIN_REG     (0x3FU)
#define ESM_ERR_CNT_REG  (0x5DU)

// INT_ESM
#ifndef INT_ESM_REG
#define INT_ESM_REG            (0x53U)
#define ESM_MCU_RST_INT_SHIFT  (5U)
#define ESM_MCU_FAIL_INT_SHIFT (4U)
#define ESM_MCU_PIN_INT_SHIFT  (3U)
#define ESM_MCU_RST_INT_MASK   (0x01U << ESM_MCU_RST_INT_SHIFT)
#define ESM_MCU_FAIL_INT_MASK  (0x01U << ESM_MCU_FAIL_INT_SHIFT)
#define ESM_MCU_PIN_INT_MASK   (0x01U << ESM_MCU_PIN_INT_SHIFT)
#endif

// ESM_START_REG
#define ESM_MCU_START_SHIFT (0U)
#define ESM_MCU_START_MASK  (1U << ESM_MCU_START_SHIFT)

// ESM_MODE_CFG
#define ESM_MCU_MODE_SHIFT       (7U)
#define ESM_MCU_EN_SHIFT         (6U)
#define ESM_MCU_CAN_DIS_SHIFT    (5U)
#define ESM_MCU_ERR_CNT_TH_SHIFT (0U)
#define ESM_MCU_MODE_MASK        (0x01U << ESM_MCU_MODE_SHIFT)
#define ESM_MCU_EN_MASK          (0x01U << ESM_MCU_EN_SHIFT)
#define ESM_MCU_CAN_DIS_MASK     (0x01U << ESM_MCU_CAN_DIS_SHIFT)
#define ESM_MCU_ERR_CNT_TH_MASK  (0x0FU << ESM_MCU_ERR_CNT_TH_SHIFT)

// ESM_ERR_CNT_REG
#define ESM_MCU_ERR_CNT_SHIFT (0U)
#define ESM_MCU_ERR_CNT_MASK  (0x1FU << ESM_MCU_ERR_CNT_SHIFT)

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* PMIC_REGMAP_ESM_H */
