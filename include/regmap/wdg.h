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
#ifndef __PMIC_REGMAP_WDG_H__
#define __PMIC_REGMAP_WDG_H__

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================*/
/*                             Include Files                                 */
/* ==========================================================================*/
#include <stdint.h>

/** @brief PMIC Watchdog Register Offsets */
#define PMIC_WD_ANSWER_REG      (0x0EU)
#define PMIC_WD_ENABLE_REG      (0x0FU)
#define PMIC_WD_MODE_REG        (0x10U)
#define PMIC_WD_WIN1_CFG_REG    (0x3FU)
#define PMIC_WD_WIN2_CFG_REG    (0x40U)
#define PMIC_WD_LONGWIN_CFG_REG (0x41U)
#define PMIC_WD_QA_CFG_REG      (0x42U)
#define PMIC_WD_THR_CFG_REG     (0x43U)
#define PMIC_WD_QA_CNT_REG      (0x5DU)
#define PMIC_WD_ERR_STAT_REG    (0x5EU)
#define PMIC_WD_FAIL_CNT_REG    (0x5FU)

/** @brief PMIC Watchdog Answer Register (WD_ANSWER_REG) */
#define PMIC_WD_ANSWER_SHIFT         (0U)
#define PMIC_WD_ANSWER_MASK          (uint8_t)(0xFFU << PMIC_WD_ANSWER_SHIFT)

/** @brief PMIC Watchdog Enable Register (WD_ENABLE_REG) */
#define PMIC_WD_EN_SHIFT             (0U)
#define PMIC_WD_EN_MASK              (uint8_t)(1U << PMIC_WD_EN_SHIFT)
#define PMIC_WD_RST_EN_SHIFT         (1U)
#define PMIC_WD_RST_EN_MASK          (uint8_t)(1U << PMIC_WD_RST_EN_SHIFT)

/** @brief PMIC Watchdog Configuration Register (WD_MODE_REG) */
#define PMIC_WD_RETURN_LONGWIN_SHIFT (0U)
#define PMIC_WD_RETURN_LONGWIN_MASK  (uint8_t)(1U << PMIC_WD_RETURN_LONGWIN_SHIFT)
#define PMIC_WD_PWRHOLD_SHIFT        (2U)
#define PMIC_WD_PWRHOLD_MASK         (uint8_t)(1U << PMIC_WD_PWRHOLD_SHIFT)

/** @brief PMIC Watchdog Window 1 Configuration Register (WD_WIN1_CFG) */
#define PMIC_WD_WIN1_SHIFT           (0U)
#define PMIC_WD_WIN1_MASK            (uint8_t)(0x7FU << PMIC_WD_WIN1_SHIFT)

/** @brief PMIC Watchdog Window 2 Configuration Register (WD_WIN2_CFG) */
#define PMIC_WD_WIN2_SHIFT           (0U)
#define PMIC_WD_WIN2_MASK            (uint8_t)(0x7FU << PMIC_WD_WIN2_SHIFT)

/** @brief PMIC Watchdog Long Window Time Configuration Register (WD_LONGWIN_CFG) */
#define PMIC_WD_LONGWIN_SHIFT        (0U)
#define PMIC_WD_LONGWIN_MASK         (uint8_t)(0xFFU << PMIC_WD_LONGWIN_SHIFT)

/** @brief PMIC Watchdog Question & Answer Configuration Register (WD_QA_CFG) */
#define PMIC_WD_QA_SEED_SHIFT        (0U)
#define PMIC_WD_QA_SEED_MASK         (uint8_t)(0xFU << PMIC_WD_QA_SEED_SHIFT)
#define PMIC_WD_QA_LFSR_SHIFT        (4U)
#define PMIC_WD_QA_LFSR_MASK         (uint8_t)(0x3U << PMIC_WD_QA_LFSR_SHIFT)
#define PMIC_WD_QA_FDBK_SHIFT        (6U)
#define PMIC_WD_QA_FDBK_MASK         (uint8_t)(0x3U << PMIC_WD_QA_FDBK_SHIFT)

/** @brief PMIC Watchdog Threshold Value Configuration Register (WD_THR_CFG) */
#define PMIC_WD_THR_RST_SHIFT        (0U)
#define PMIC_WD_THR_RST_MASK         (uint8_t)(0x7U << PMIC_WD_THR_RST_SHIFT)
#define PMIC_WD_THR_FAIL_SHIFT       (3U)
#define PMIC_WD_THR_FAIL_MASK        (uint8_t)(0x7U << PMIC_WD_THR_FAIL_SHIFT)

/** @brief PMIC Watchdog Question & Answer Count Register (WD_QA_CNT) */
#define PMIC_WD_QUESTION_SHIFT       (0U)
#define PMIC_WD_QUESTION_MASK        (uint8_t)(0xFU << PMIC_WD_QUESTION_SHIFT)
#define PMIC_WD_ANSW_CNT_SHIFT       (4U)
#define PMIC_WD_ANSW_CNT_MASK        (uint8_t)(0x3U << PMIC_WD_ANSW_CNT_SHIFT)
#define PMIC_INT_TOP_STATUS_SHIFT    (7U)

/** @brief PMIC Watchdog Error Status Register (WD_ERR_STAT) */
#define PMIC_WD_LONGWIN_TMO_SHIFT    (0U)
#define PMIC_WD_TMO_SHIFT            (1U)
#define PMIC_WD_ANSW_EARLY_SHIFT     (3U)
#define PMIC_WD_SEQ_ERR_SHIFT        (4U)
#define PMIC_WD_ANSW_ERR_SHIFT       (5U)
#define PMIC_WD_FAIL_ERR_SHIFT       (6U)
#define PMIC_WD_RST_ERR_SHIFT        (7U)

/** @brief PMIC Watchdog Status Register (WD_FAIL_CNT) */
#define PMIC_WD_ERR_CNT_SHIFT        (0U)
#define PMIC_WD_ERR_CNT_MASK         (uint8_t)(0xFU << PMIC_WD_ERR_CNT_SHIFT)
#define PMIC_WD_FIRST_OK_SHIFT       (5U)
#define PMIC_WD_BAD_EVENT_SHIFT      (6U)

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* __PMIC_REGMAP_WDG_H__ */
