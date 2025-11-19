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
 * @file wdg.h
 *
 * @brief PMIC LLD WDG module register addresses and bit fields.
 */
#ifndef REGMAP_WDG_H
#define REGMAP_WDG_H

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

#define WD_ANSWER_REG_REGADDR        (0x401U)
#define WD_QUESTION_ANSW_CNT_REGADDR (0x402U)
#define WD_WIN1_CFG_REGADDR          (0x403U)
#define WD_WIN2_CFG_REGADDR          (0x404U)
#define WD_LONGWIN_CFG_REGADDR       (0x405U)
#define WD_MODE_REG_REGADDR          (0x406U)
#define WD_QA_CFG_REGADDR            (0x407U)
#define WD_ERR_STATUS_REGADDR        (0x408U)
#define WD_THR_CFG_REGADDR           (0x409U)
#define WD_FAIL_CNT_REG_REGADDR      (0x40AU)

/* ========================================================================== */
/*                              Register Bit Fields                           */
/* ========================================================================== */

// WD_ANSWER_REG
#define WD_ANSWER_SHIFT (0U)
#define WD_ANSWER_MASK  (0xFFU << WD_ANSWER_SHIFT)

// WD_QUESTION_ANSW_CNT
#define INT_TOP_STATUS_SHIFT (7U)
#define WD_ANSW_CNT_SHIFT    (4U)
#define WD_QUESTION_SHIFT    (0U)
#define INT_TOP_STATUS_MASK  (0x01U << INT_TOP_STATUS_SHIFT)
#define WD_ANSW_CNT_MASK     (0x03U << WD_ANSW_CNT_SHIFT)
#define WD_QUESTION_MASK     (0x0FU << WD_QUESTION_SHIFT)

// WD_WIN1_CFG, WD_WIN2_CFG
#define WD_WIN_SHIFT (0U)
#define WD_WIN_MASK  (0x7FU << WD_WIN_SHIFT)

// WD_LONGWIN_CFG
#define WD_LONGWIN_SHIFT (0U)
#define WD_LONGWIN_MASK  (0xFFU << WD_LONGWIN_SHIFT)

// WD_MODE_REG
#define WD_CNT_SEL_SHIFT        (7U)
#define WD_ENDRV_SEL_SHIFT      (6U)
#define WD_PWRHOLD_SHIFT        (2U)
#define WD_MODE_SELECT_SHIFT    (1U)
#define WD_RETURN_LONGWIN_SHIFT (0U)
#define WD_CNT_SEL_MASK         (0x01U << WD_CNT_SEL_SHIFT)
#define WD_ENDRV_SEL_MASK       (0x01U << WD_ENDRV_SEL_SHIFT)
#define WD_PWRHOLD_MASK         (0x01U << WD_PWRHOLD_SHIFT)
#define WD_MODE_SELECT_MASK     (0x01U << WD_MODE_SELECT_SHIFT)
#define WD_RETURN_LONGWIN_MASK  (0x01U << WD_RETURN_LONGWIN_SHIFT)

// WD_QA_CFG
#define WD_QA_FDBK_SHIFT        (6U)
#define WD_QA_LFSR_SHIFT        (4U)
#define WD_QUESTION_SEED_SHIFT  (0U)
#define WD_QA_FDBK_MASK         (0x03U << WD_QA_FDBK_SHIFT)
#define WD_QA_LFSR_MASK         (0x03U << WD_QA_LFSR_SHIFT)
#define WD_QUESTION_SEED_MASK   (0x0FU << WD_QUESTION_SEED_SHIFT)

// WD_ERR_STATUS
#define WD_RST_INT_SHIFT             (7U)
#define WD_FAIL_INT_SHIFT            (6U)
#define WD_ANSW_ERR_SHIFT            (5U)
#define WD_SEQ_ERR_SHIFT             (4U)
#define WD_ANSW_EARLY_SHIFT          (3U)
#define WD_TRIG_EARLY_SHIFT          (2U)
#define WD_TIMEOUT_SHIFT             (1U)
#define WD_LONGWIN_TIMEOUT_INT_SHIFT (0U)
#define WD_RST_INT_MASK              (0x01U << WD_RST_INT_SHIFT)
#define WD_FAIL_INT_MASK             (0x01U << WD_FAIL_INT_SHIFT)
#define WD_ANSW_ERR_MASK             (0x01U << WD_ANSW_ERR_SHIFT)
#define WD_SEQ_ERR_MASK              (0x01U << WD_SEQ_ERR_SHIFT)
#define WD_ANSW_EARLY_MASK           (0x01U << WD_ANSW_EARLY_SHIFT)
#define WD_TRIG_EARLY_MASK           (0x01U << WD_TRIG_EARLY_SHIFT)
#define WD_TIMEOUT_MASK              (0x01U << WD_TIMEOUT_SHIFT)
#define WD_LONGWIN_TIMEOUT_INT_MASK  (0x01U << WD_LONGWIN_TIMEOUT_INT_SHIFT)

// WD_THR_CFG
#define WD_RST_EN_SHIFT  (7U)
#define WD_EN_SHIFT      (6U)
#define WD_FAIL_TH_SHIFT (3U)
#define WD_RST_TH_SHIFT  (0U)
#define WD_RST_EN_MASK   (0x01U << WD_RST_EN_SHIFT)
#define WD_EN_MASK       (0x01U << WD_EN_SHIFT)
#define WD_FAIL_TH_MASK  (0x07U << WD_FAIL_TH_SHIFT)
#define WD_RST_TH_MASK   (0x07U << WD_RST_TH_SHIFT)

// WD_FAIL_CNT_REG
#define WD_BAD_EVENT_SHIFT (6U)
#define WD_FIRST_OK_SHIFT  (5U)
#define WD_FAIL_CNT_SHIFT  (0U)
#define WD_BAD_EVENT_MASK  (0x01U << WD_BAD_EVENT_SHIFT)
#define WD_FIRST_OK_MASK   (0x01U << WD_FIRST_OK_SHIFT)
#define WD_FAIL_CNT_MASK   (0x0FU << WD_FAIL_CNT_SHIFT)

#ifdef __cplusplus
}

#endif /* __cplusplus */
#endif /* REGMAP_WDG_H */
