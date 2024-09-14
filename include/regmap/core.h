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
/**
 * @file core.h
 *
 * @brief PMIC LLD register bit fields pertaining to the Core module.
 */
#ifndef __CORE_H__
#define __CORE_H__

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                Include Files                               */
/* ========================================================================== */
#include "pmic_common.h"

/* ========================================================================== */
/*                               Macros & Typedefs                            */
/* ========================================================================== */

// SAFETY_ERR_CFG_1 register bit field shifts and masks.
#define SAFE_TO_SHIFT           ((uint8_t)5U)
#define SAFE_TO_MASK            ((uint8_t)7U << SAFE_TO_SHIFT)
#define SAFE_LOCK_THR_SHIFT     ((uint8_t)1U)
#define SAFE_LOCK_THR_MASK      ((uint8_t)0xFU << SAFE_LOCK_THR_SHIFT)
#define CFG_LOCK_SHIFT          ((uint8_t)0U)
#define CFG_LOCK_MASK           ((uint8_t)1U << CFG_LOCK_SHIFT)

// SAFETY_ERR_CFG_2 register bit field shifts and masks.
#define MCU_ERR_CNT_TH_SHIFT    ((uint8_t)0U)
#define MCU_ERR_CNT_TH_MASK     ((uint8_t)7U << MCU_ERR_CNT_TH_SHIFT)

// SAFETY_BIST_CTRL register bit field shifts and masks.
#define AUTO_BIST_DIS_SHIFT     ((uint8_t)5U)
#define AUTO_BIST_DIS_MASK      ((uint8_t)1U << AUTO_BIST_DIS_SHIFT)
#define EE_CRC_CHK_SHIFT        ((uint8_t)4U)
#define EE_CRC_CHK_MASK         ((uint8_t)1U << EE_CRC_CHK_SHIFT)
#define LBIST_EN_SHIFT          ((uint8_t)2U)
#define LBIST_EN_MASK           ((uint8_t)1U << LBIST_EN_SHIFT)
#define ABIST_EN_SHIFT          ((uint8_t)1U)
#define ABIST_EN_MASK           ((uint8_t)1U << ABIST_EN_SHIFT)

// SAFETY_CHECK_CTRL register bit field shifts and masks.
#define CFG_CRC_EN_SHIFT        ((uint8_t)7U)
#define CFG_CRC_EN_MASK         ((uint8_t)1U << CFG_CRC_EN_SHIFT)
#define ENABLE_DRV_SHIFT        ((uint8_t)5U)
#define ENABLE_DRV_MASK         ((uint8_t)1U << ENABLE_DRV_SHIFT)
#define NO_ERROR_SHIFT          ((uint8_t)2U)
#define NO_ERROR_MASK           ((uint8_t)1U << NO_ERROR_SHIFT)
#define DIAG_EXIT_MASK_SHIFT    ((uint8_t)1U)
#define DIAG_EXIT_MASK_MASK     ((uint8_t)1U << DIAG_EXIT_MASK_SHIFT)
#define DIAG_EXIT_SHIFT         ((uint8_t)0U)
#define DIAG_EXIT_MASK          ((uint8_t)1U << DIAG_EXIT_SHIFT)

// SAFETY_FUNC_CFG register bit field shifts and masks.
#define SAFE_LOCK_TO_DIS_SHIFT  ((uint8_t)7U)
#define SAFE_LOCK_TO_DIS_MASK   ((uint8_t)1U << SAFE_LOCK_TO_DIS_SHIFT)
#define ESM_CFG_SHIFT           ((uint8_t)6U)
#define ESM_CFG_MASK            ((uint8_t)1U << ESM_CFG_SHIFT)
#define WD_CFG_SHIFT            ((uint8_t)5U)
#define WD_CFG_MASK             ((uint8_t)1U << WD_CFG_SHIFT)
#define WD_RST_EN_SHIFT         ((uint8_t)3U)
#define WD_RST_EN_MASK          ((uint8_t)1U << WD_RST_EN_SHIFT)
#define DIS_NRES_MON_SHIFT      ((uint8_t)2U)
#define DIS_NRES_MON_MASK       ((uint8_t)1U << DIS_NRES_MON_SHIFT)
#define LOCK_STEP_RST_EN_SHIFT  ((uint8_t)1U)
#define LOCK_STEP_RST_EN_MASK   ((uint8_t)1U << LOCK_STEP_RST_EN_SHIFT)
#define WARM_RST_CFG_SHIFT      ((uint8_t)0U)
#define WARM_RST_CFG_MASK       ((uint8_t)1U << WARM_RST_CFG_SHIFT)

#ifdef __cplusplus
}

#endif /* __cplusplus */
#endif /* __CORE_H__ */
