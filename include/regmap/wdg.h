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
 * @brief PMIC LLD register addresses and bit fields pertaining to the
 * Watchdog module.
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
/*                      TPS65036x Watchdog Register Map                       */
/* ========================================================================== */

#define PMIC_WD_ANSWER_REG_REGADDR              ((uint8_t)0x0EU)
#define PMIC_WD_ENABLE_REG_REGADDR              ((uint8_t)0x0FU)
#define PMIC_WD_MODE_REG_REGADDR                ((uint8_t)0x10U)
#define PMIC_WD_CONFIG_REGADDR                  ((uint8_t)0x16U)
#define PMIC_WD_WIN1_CFG_REGADDR                ((uint8_t)0x45U)
#define PMIC_WD_WIN2_CFG_REGADDR                ((uint8_t)0x46U)
#define PMIC_WD_LONGWIN_CFG_REGADDR             ((uint8_t)0x47U)
#define PMIC_WD_QA_CFG_REGADDR                  ((uint8_t)0x48U)
#define PMIC_WD_THR_CFG_REGADDR                 ((uint8_t)0x49U)
#define PMIC_WD_QUESTION_ANSW_CNT_REGADDR       ((uint8_t)0x5EU)
#define PMIC_WD_ERR_STATUS_REGADDR              ((uint8_t)0x5FU)
#define PMIC_WD_FAIL_CNT_REG_REGADDR            ((uint8_t)0x60U)

/* ========================================================================== */
/*                               Macros & Typedefs                            */
/* ========================================================================== */

/**
 * @anchor Pmic_wdEnableRegBitFields
 * @name TPS65036x WD_ENABLE_REG Register Bit Fields
 *
 * @brief Mask and shift values for all bit fields of the WD_ENABLE_REG register.
 *
 * @{
 */
#define PMIC_WD_RST_EN_SHIFT                    ((uint8_t)1U)
#define PMIC_WD_RST_EN_MASK                     ((uint8_t)1U << PMIC_WD_RST_EN_SHIFT)
#define PMIC_WD_EN_SHIFT                        ((uint8_t)0U)
#define PMIC_WD_EN_MASK                         ((uint8_t)1U << PMIC_WD_EN_SHIFT)
/** @} */

/**
 * @anchor Pmic_wdModeRegBitFields
 * @name TPS65036x WD_MODE_REG Register Bit Fields
 *
 * @brief Mask and shift values for all bit fields of the WD_MODE_REG register.
 *
 * @{
 */
#define PMIC_WD_PWRHOLD_SHIFT                   ((uint8_t)2U)
#define PMIC_WD_PWRHOLD_MASK                    ((uint8_t)1U << PMIC_WD_PWRHOLD_SHIFT)
#define PMIC_WD_RETURN_LONGWIN_SHIFT            ((uint8_t)0U)
#define PMIC_WD_RETURN_LONGWIN_MASK             ((uint8_t)1U << PMIC_WD_RETURN_LONGWIN_SHIFT)
/** @} */

/**
 * @anchor Pmic_wdConfigBitFields
 * @name TPS65036x WD_CONFIG Register Bit Fields
 *
 * @brief Mask and shift values for all bit fields of the WD_CONFIG register.
 *
 * @{
 */
#define PMIC_WD_TRIGGER_SEL_SHIFT               ((uint8_t)1U)
#define PMIC_WD_TRIGGER_SEL_MASK                ((uint8_t)1U << PMIC_WD_TRIGGER_SEL_SHIFT)
#define PMIC_WD_MODE_SELECT_SHIFT               ((uint8_t)0U)
#define PMIC_WD_MODE_SELECT_MASK                ((uint8_t)1U << PMIC_WD_MODE_SELECT_SHIFT)
/** @} */

/**
 * @anchor Pmic_wdWin1CfgBitFields
 * @name TPS65036x WD_WIN1_CFG Register Bit Fields
 *
 * @brief Mask and shift values for all bit fields of the WD_WIN1_CFG register.
 *
 * @{
 */
#define PMIC_WD_WIN1_SHIFT                      ((uint8_t)0U)
#define PMIC_WD_WIN1_MASK                       ((uint8_t)0x7FU << PMIC_WD_WIN1_SHIFT)
/** @} */

/**
 * @anchor Pmic_wdWin2CfgBitFields
 * @name TPS65036x WD_WIN2_CFG Register Bit Fields
 *
 * @brief Mask and shift values for all bit fields of the WD_WIN2_CFG register.
 *
 * @{
 */
#define PMIC_WD_WIN2_SHIFT                      ((uint8_t)0U)
#define PMIC_WD_WIN2_MASK                       ((uint8_t)0x7FU << PMIC_WD_WIN2_SHIFT)
/** @} */

/**
 * @anchor Pmic_wdQaCfgBitFields
 * @name TPS65036x WD_QA_CFG Register Bit Fields
 *
 * @brief Mask and shift values for all bit fields of the WD_QA_CFG register.
 *
 * @{
 */
#define PMIC_WD_QA_FDBK_SHIFT                   ((uint8_t)6U)
#define PMIC_WD_QA_FDBK_MASK                    ((uint8_t)3U << PMIC_WD_QA_FDBK_SHIFT)
#define PMIC_WD_QA_LFSR_SHIFT                   ((uint8_t)4U)
#define PMIC_WD_QA_LFSR_MASK                    ((uint8_t)3U << PMIC_WD_QA_LFSR_SHIFT)
#define PMIC_WD_QUESTION_SEED_SHIFT             ((uint8_t)0U)
#define PMIC_WD_QUESTION_SEED_MASK              ((uint8_t)0xFU << PMIC_WD_QUESTION_SEED_SHIFT)
/** @} */

/**
 * @anchor Pmic_wdThrCfgBitFields
 * @name TPS65036x WD_THR_CFG Register Bit Fields
 *
 * @brief Mask and shift values for all bit fields of the WD_THR_CFG register.
 *
 * @{
 */
#define PMIC_WD_FAIL_TH_SHIFT                   ((uint8_t)3U)
#define PMIC_WD_FAIL_TH_MASK                    ((uint8_t)7U << PMIC_WD_FAIL_TH_SHIFT)
#define PMIC_WD_RST_TH_SHIFT                    ((uint8_t)0U)
#define PMIC_WD_RST_TH_MASK                     ((uint8_t)7U << PMIC_WD_RST_TH_SHIFT)
/** @} */

/**
 * @anchor Pmic_wdQuestionAnswCntBitFields
 * @name TPS65036x WD_QUESTION_ANSW_CNT Register Bit Fields
 *
 * @brief Mask and shift values for all bit fields of the WD_QUESTION_ANSW_CNT
 * register.
 *
 * @{
 */
#define PMIC_INT_TOP_STATUS_SHIFT               ((uint8_t)7U)
#define PMIC_INT_TOP_STATUS_MASK                ((uint8_t)1U << PMIC_INT_TOP_STATUS_SHIFT)
#define PMIC_WD_ANSW_CNT_SHIFT                  ((uint8_t)4U)
#define PMIC_WD_ANSW_CNT_MASK                   ((uint8_t)3U << PMIC_WD_ANSW_CNT_SHIFT)
#define PMIC_WD_QUESTION_SHIFT                  ((uint8_t)0U)
#define PMIC_WD_QUESTION_MASK                   ((uint8_t)0xFU << PMIC_WD_QUESTION_SHIFT)
/** @} */

/**
 * @anchor Pmic_wdErrStatusBitFields
 * @name  TPS65036x WD_ERR_STATUS Register Bit Fields
 *
 * @brief Mask and shift values for all bit fields of the WD_ERR_STATUS
 * register.
 *
 * @{
 */
#define PMIC_WD_RST_INT_SHIFT                   ((uint8_t)7U)
#define PMIC_WD_RST_INT_MASK                    ((uint8_t)1U << PMIC_WD_RST_INT_SHIFT)
#define PMIC_WD_FAIL_INT_SHIFT                  ((uint8_t)6U)
#define PMIC_WD_FAIL_INT_MASK                   ((uint8_t)1U << PMIC_WD_FAIL_INT_SHIFT)
#define PMIC_WD_ANSW_ERR_SHIFT                  ((uint8_t)5U)
#define PMIC_WD_ANSW_ERR_MASK                   ((uint8_t)1U << PMIC_WD_ANSW_ERR_SHIFT)
#define PMIC_WD_SEQ_ERR_SHIFT                   ((uint8_t)4U)
#define PMIC_WD_SEQ_ERR_MASK                    ((uint8_t)1U << PMIC_WD_SEQ_ERR_SHIFT)
#define PMIC_WD_ANSW_EARLY_SHIFT                ((uint8_t)3U)
#define PMIC_WD_ANSW_EARLY_MASK                 ((uint8_t)1U << PMIC_WD_ANSW_EARLY_SHIFT)
#define PMIC_WD_TIMEOUT_SHIFT                   ((uint8_t)1U)
#define PMIC_WD_TIMEOUT_MASK                    ((uint8_t)1U << PMIC_WD_TIMEOUT_SHIFT)
#define PMIC_WD_LONGWIN_TIMEOUT_INT_SHIFT       ((uint8_t)0U)
#define PMIC_WD_LONGWIN_TIMEOUT_INT_MASK        ((uint8_t)1U << PMIC_WD_LONGWIN_TIMEOUT_INT_SHIFT)
/** @} */

/**
 * @anchor Pmic_wdFailCntRegBitFields
 * @name TPS65036x WD_FAIL_CNT_REG Register Bit Fields
 *
 * @brief Mask and shift values for all bit fields of the WD_FAIL_CNT_REG
 * register.
 *
 * @{
 */
#define PMIC_WD_BAD_EVENT_SHIFT                 ((uint8_t)6U)
#define PMIC_WD_BAD_EVENT_MASK                  ((uint8_t)1U << PMIC_WD_BAD_EVENT_SHIFT)
#define PMIC_WD_FIRST_OK_SHIFT                  ((uint8_t)5U)
#define PMIC_WD_FIRST_OK_MASK                   ((uint8_t)1U << PMIC_WD_FIRST_OK_SHIFT)
#define PMIC_WD_FAIL_CNT_SHIFT                  ((uint8_t)0U)
#define PMIC_WD_FAIL_CNT_MASK                   ((uint8_t)0xFU << PMIC_WD_FAIL_CNT_SHIFT)
/** @} */

#ifdef __cplusplus
}

#endif /* __cplusplus */
#endif /* __WDG_H__ */
