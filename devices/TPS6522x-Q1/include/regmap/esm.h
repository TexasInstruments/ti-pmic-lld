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
 * @file esm.h
 *
 * @brief PMIC LLD ESM module register addresses and bit fields.
 */
#ifndef REGMAP_ESM_H
#define REGMAP_ESM_H

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

#define ESM_MCU_START_REG_REG   ((uint16_t)0x8FU)
#define ESM_MCU_DELAY1_REG_REG  ((uint16_t)0x90U)
#define ESM_MCU_DELAY2_REG_REG  ((uint16_t)0x91U)
#define ESM_MCU_MODE_CFG_REG    ((uint16_t)0x92U)
#define ESM_MCU_HMAX_REG_REG    ((uint16_t)0x93U)
#define ESM_MCU_HMIN_REG_REG    ((uint16_t)0x94U)
#define ESM_MCU_LMAX_REG_REG    ((uint16_t)0x95U)
#define ESM_MCU_LMIN_REG_REG    ((uint16_t)0x96U)
#define ESM_MCU_ERR_CNT_REG_REG ((uint16_t)0x97U)

/* ========================================================================== */
/*                              Register Bit Fields                           */
/* ========================================================================== */

// ESM_MCU_START_REG
#define ESM_MCU_START_SHIFT (0U)
#define ESM_MCU_START_MASK  (1UL << ESM_MCU_START_SHIFT)

// ESM_MCU_DELAY1_REG
#define ESM_MCU_DELAY1_SHIFT (0U)
#define ESM_MCU_DELAY1_MASK  (0xFFU << ESM_MCU_DELAY1_SHIFT)

// ESM_MCU_DELAY2_REG
#define ESM_MCU_DELAY2_SHIFT (0U)
#define ESM_MCU_DELAY2_MASK  (0xFFU << ESM_MCU_DELAY2_SHIFT)

// ESM_MCU_MODE_CFG
#define ESM_MCU_MODE_SHIFT       (7U)
#define ESM_MCU_EN_SHIFT         (6U)
#define ESM_MCU_ENDRV_SHIFT      (5U)
#define ESM_MCU_ERR_CNT_TH_SHIFT (0U)
#define ESM_MCU_MODE_MASK        (0x01UL << ESM_MCU_MODE_SHIFT)
#define ESM_MCU_EN_MASK          (0x01UL << ESM_MCU_EN_SHIFT)
#define ESM_MCU_ENDRV_MASK       (0x01UL << ESM_MCU_ENDRV_SHIFT)
#define ESM_MCU_ERR_CNT_TH_MASK  (0x0FU << ESM_MCU_ERR_CNT_TH_SHIFT)

// ESM_MCU_HMAX_REG
#define ESM_MCU_HMAX_SHIFT (0U)
#define ESM_MCU_HMAX_MASK  (0xFFU << ESM_MCU_HMAX_SHIFT)

// ESM_MCU_HMIN_REG
#define ESM_MCU_HMIN_SHIFT (0U)
#define ESM_MCU_HMIN_MASK  (0xFFU << ESM_MCU_HMIN_SHIFT)

// ESM_MCU_LMAX_REG
#define ESM_MCU_LMAX_SHIFT (0U)
#define ESM_MCU_LMAX_MASK  (0xFFU << ESM_MCU_LMAX_SHIFT)

// ESM_MCU_LMIN_REG
#define ESM_MCU_LMIN_SHIFT (0U)
#define ESM_MCU_LMIN_MASK  (0xFFU << ESM_MCU_LMIN_SHIFT)

// ESM_MCU_ERR_CNT_REG
#define ESM_MCU_ERR_CNT_SHIFT (0U)
#define ESM_MCU_ERR_CNT_MASK  (0x1FU << ESM_MCU_ERR_CNT_SHIFT)

#ifdef __cplusplus
}

#endif /* __cplusplus */
#endif /* REGMAP_ESM_H */
