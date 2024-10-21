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
 * @file wdg.h
 *
 * @brief PMIC LLD register bit fields pertaining to the Watchdog module.
 */
#ifndef __WDG_H__
#define __WDG_H__

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

// WD_QUESTION_FDBCK register bit field shifts and masks.
#define FDBK_SHIFT              ((uint8_t)6U)
#define FDBK_MASK               ((uint8_t)3U << FDBK_SHIFT)
#define LFSR_CHG_SHIFT          ((uint8_t)4U)
#define LFSR_CHG_MASK           ((uint8_t)3U << LFSR_CHG_SHIFT)
#define QUESTION_SEED_SHIFT     ((uint8_t)0U)
#define QUESTION_SEED_MASK      ((uint8_t)0xFU << QUESTION_SEED_SHIFT)

// WD_WIN1_CFG register bit field shifts and masks.
#define RT_SHIFT                ((uint8_t)0U)
#define RT_MASK                 ((uint8_t)0x7FU)

// WD_WIN2_CFG register bit field shifts and masks.
#define RW_SHIFT                ((uint8_t)0U)
#define RW_MASK                 ((uint8_t)0x1FU)

// WD_QUESTION register bit field shifts and masks.
#define WD_FAIL_TH_SHIFT        ((uint8_t)7U)
#define WD_FAIL_TH_MASK         ((uint8_t)1U << WD_FAIL_TH_SHIFT)
#define QUESTION_SHIFT          ((uint8_t)0U)
#define QUESTION_MASK           ((uint8_t)0xFU << QUESTION_SHIFT)

// WD_STATUS register bit field shifts and masks.
#define WD_ANSW_CNT_SHIFT       ((uint8_t)6U)
#define WD_ANSW_CNT_MASK        ((uint8_t)3U << WD_ANSW_CNT_SHIFT)
#define ANSWER_ERR_SHIFT        ((uint8_t)5U)
#define ANSWER_ERR_MASK         ((uint8_t)1U << ANSWER_ERR_SHIFT)
#define WD_CFG_CHG_SHIFT        ((uint8_t)3U)
#define WD_CFG_CHG_MASK         ((uint8_t)1U << WD_CFG_CHG_SHIFT)
#define SEQ_ERR_SHIFT           ((uint8_t)2U)
#define SEQ_ERR_MASK            ((uint8_t)1U << SEQ_ERR_SHIFT)
#define TIME_OUT_SHIFT          ((uint8_t)1U)
#define TIME_OUT_MASK           ((uint8_t)1U << TIME_OUT_SHIFT)
#define ANSWER_EARLY_SHIFT      ((uint8_t)0U)
#define ANSWER_EARLY_MASK       ((uint8_t)1U << ANSWER_EARLY)

#ifdef __cplusplus
}

#endif /* __cplusplus */
#endif /* __WDG_H__ */
