/******************************************************************************
 * Copyright (c) 2020 Texas Instruments Incorporated - http://www.ti.com
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
 * \file   pmic_wdg_priv.h
 *
 * \brief  This file contains PMIC WDG Driver specific Macros.
 *
 */

#ifndef PMIC_WDG_PRIV_H_
#define PMIC_WDG_PRIV_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <pmic.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/*!
 * \brief  PMIC Watch Dog Register Offsets
 */
#define PMIC_WD_ANSWER_REG_REGADDR                   (0x401U)
#define PMIC_WD_QUESTION_ANSW_CNT_REGADDR            (0x402U)
#define PMIC_WD_WIN1_CFG_REGADDR                     (0x403U)
#define PMIC_WD_WIN2_CFG_REGADDR                     (0x404U)
#define PMIC_WD_LONGWIN_CFG_REGADDR                  (0x405U)
#define PMIC_WD_MODE_REG_REGADDR                     (0x406U)
#define PMIC_WD_QA_CFG_REGADDR                       (0x407U)
#define PMIC_WD_ERR_STATUS_REGADDR                   (0x408U)
#define PMIC_WD_THR_CFG_REGADDR                      (0x409U)
#define PMIC_WD_FAIL_CNT_REG_REGADDR                 (0x40AU)

/*!
 * \brief WD_QUESTION_ANSW_CNT Register bit shift values
 */
#define PMIC_WD_QUESTION_ANSW_CNT_WD_ANSW_CNT_SHIFT  (0x04U)
#define PMIC_WD_QUESTION_ANSW_CNT_WD_QUESTION_SHIFT  (0x00U)

/*!
 * \brief WD_WIN1_CFG Register bit shift values
 */
#define PMIC_WD_WIN1_CFG_WD_WIN1_SHIFT               (0x00U)

/*!
 * \brief WD_WIN2_CFG Register bit shift values
 */
#define PMIC_WD_WIN2_CFG_WD_WIN2_SHIFT               (0x00U)

/*!
 * \brief WD_MODE_REG Register bit shift values
 */
#define PMIC_WD_MODE_REG_WD_RETURN_LONGWIN_SHIFT     (0x00U)
#define PMIC_WD_MODE_REG_WD_MODE_SELECT_SHIFT        (0x01U)
#define PMIC_WD_MODE_REG_WD_PWRHOLD_SHIFT            (0x02U)

/*!
 * \brief WD_QA_CFG Register bit shift values
 */
#define PMIC_WD_QA_CFG_WD_QUESTION_SEED_SHIFT        (0x00U)
#define PMIC_WD_QA_CFG_WD_QA_LFSR_SHIFT              (0x04U)
#define PMIC_WD_QA_CFG_WD_QA_FDBK_SHIFT              (0x06U)

/*!
 * \brief WD_ERR_STATUS Register bit shift values
 */
#define PMIC_WD_ERR_STATUS_WD_LONGWIN_TIMEOUT_INT_SHIFT (0x00U)
#define PMIC_WD_ERR_STATUS_WD_TIMEOUT_SHIFT             (0x01U)
#define PMIC_WD_ERR_STATUS_WD_TRIG_EARLY_SHIFT          (0x02U)
#define PMIC_WD_ERR_STATUS_WD_ANSW_EARLY_SHIFT          (0x03U)
#define PMIC_WD_ERR_STATUS_WD_SEQ_ERR_SHIFT             (0x04U)
#define PMIC_WD_ERR_STATUS_WD_ANSW_ERR_SHIFT            (0x05U)
#define PMIC_WD_ERR_STATUS_WD_FAIL_INT_SHIFT            (0x06U)
#define PMIC_WD_ERR_STATUS_WD_RST_INT_SHIFT             (0x07U)

/*!
 * \brief WD_THR_CFG Register bit shift values
 */
#define PMIC_WD_THR_CFG_WD_RST_TH_SHIFT              (0x00U)
#define PMIC_WD_THR_CFG_WD_FAIL_TH_SHIFT             (0x03U)
#define PMIC_WD_THR_CFG_WD_EN_SHIFT                  (0x06U)
#define PMIC_WD_THR_CFG_WD_RST_EN_SHIFT              (0x07U)

/*!
 * \brief WD_FAIL_CNT_REG Register bit shift values
 */
#define PMIC_WD_FAIL_CNT_REG_WD_FAIL_CNT_SHIFT       (0x00U)
#define PMIC_WD_FAIL_CNT_REG_WD_FIRST_OK_SHIFT       (0x05U)
#define PMIC_WD_FAIL_CNT_REG_WD_BAD_EVENT_SHIFT      (0x06U)

/*!
 * \brief WD_QUESTION_ANSW_CNT Register bit mask values
 */
#define PMIC_WD_QUESTION_ANSW_CNT_WD_ANSW_CNT_MASK   \
                         (0x3U << PMIC_WD_QUESTION_ANSW_CNT_WD_ANSW_CNT_SHIFT)
#define PMIC_WD_QUESTION_ANSW_CNT_WD_QUESTION_MASK   \
                         (0xFU << PMIC_WD_QUESTION_ANSW_CNT_WD_QUESTION_SHIFT)

/*!
 * \brief WD_WIN1_CFG Register bit mask values
 */
#define PMIC_WD_WIN1_CFG_WD_WIN1_MASK                \
                         (0x7FU << PMIC_WD_WIN1_CFG_WD_WIN1_SHIFT)

/*!
 * \brief WD_WIN2_CFG Register bit mask values
 */
#define PMIC_WD_WIN2_CFG_WD_WIN2_MASK                \
                         (0x7FU << PMIC_WD_WIN2_CFG_WD_WIN2_SHIFT)

/*!
 * \brief WD_MODE_REG Register bit mask values
 */
#define PMIC_WD_MODE_REG_WD_RETURN_LONGWIN_MASK      \
                         (0x1U << PMIC_WD_MODE_REG_WD_RETURN_LONGWIN_SHIFT)
#define PMIC_WD_MODE_REG_WD_MODE_SELECT_MASK         \
                         (0x1U << PMIC_WD_MODE_REG_WD_MODE_SELECT_SHIFT)
#define PMIC_WD_MODE_REG_WD_PWRHOLD_MASK             \
                         (0x1U << PMIC_WD_MODE_REG_WD_PWRHOLD_SHIFT)

/*!
 * \brief WD_QA_CFG Register bit mask values
 */
#define PMIC_WD_QA_CFG_WD_QUESTION_SEED_MASK         \
                         (0xFU << PMIC_WD_QA_CFG_WD_QUESTION_SEED_SHIFT)
#define PMIC_WD_QA_CFG_WD_QA_LFSR_MASK               \
                         (0x3U << PMIC_WD_QA_CFG_WD_QA_LFSR_SHIFT)
#define PMIC_WD_QA_CFG_WD_QA_FDBK_MASK               \
                         (0x3U << PMIC_WD_QA_CFG_WD_QA_FDBK_SHIFT)

/*!
 * \brief WD_ERR_STATUS Register bit mask values
 */
#define PMIC_WD_ERR_STATUS_WD_LONGWIN_TIMEOUT_INT_MASK   \
                    (0x1U << PMIC_WD_ERR_STATUS_WD_LONGWIN_TIMEOUT_INT_SHIFT)
#define PMIC_WD_ERR_STATUS_WD_TIMEOUT_MASK               \
                    (0x1U << PMIC_WD_ERR_STATUS_WD_TIMEOUT_SHIFT)
#define PMIC_WD_ERR_STATUS_WD_TRIG_EARLY_MASK            \
                    (0x1U << PMIC_WD_ERR_STATUS_WD_TRIG_EARLY_SHIFT)
#define PMIC_WD_ERR_STATUS_WD_ANSW_EARLY_MASK            \
                    (0x1U << PMIC_WD_ERR_STATUS_WD_ANSW_EARLY_SHIFT)
#define PMIC_WD_ERR_STATUS_WD_SEQ_ERR_MASK               \
                    (0x1U << PMIC_WD_ERR_STATUS_WD_SEQ_ERR_SHIFT)
#define PMIC_WD_ERR_STATUS_WD_ANSW_ERR_MASK              \
                    (0x1U << PMIC_WD_ERR_STATUS_WD_ANSW_ERR_SHIFT)
#define PMIC_WD_ERR_STATUS_WD_FAIL_INT_MASK              \
                    (0x1U << PMIC_WD_ERR_STATUS_WD_FAIL_INT_SHIFT)
#define PMIC_WD_ERR_STATUS_WD_RST_INT_MASK               \
                    (0x1U << PMIC_WD_ERR_STATUS_WD_RST_INT_SHIFT)

/*!
 * \brief WD_THR_CFG Register bit mask values
 */
#define PMIC_WD_THR_CFG_WD_RST_TH_MASK               \
                         (0x7U << PMIC_WD_THR_CFG_WD_RST_TH_SHIFT)
#define PMIC_WD_THR_CFG_WD_FAIL_TH_MASK              \
                         (0x7U << PMIC_WD_THR_CFG_WD_FAIL_TH_SHIFT)
#define PMIC_WD_THR_CFG_WD_EN_MASK                   \
                         (0x1U << PMIC_WD_THR_CFG_WD_EN_SHIFT)
#define PMIC_WD_THR_CFG_WD_RST_EN_MASK               \
                         (0x1U << PMIC_WD_THR_CFG_WD_RST_EN_SHIFT)

/*!
 * \brief WD_FAIL_CNT_REG Register bit mask values
 */
#define PMIC_WD_FAIL_CNT_REG_WD_FAIL_CNT_MASK        \
                         (0xFU << PMIC_WD_FAIL_CNT_REG_WD_FAIL_CNT_SHIFT)
#define PMIC_WD_FAIL_CNT_REG_WD_FIRST_OK_MASK        \
                         (0x1U << PMIC_WD_FAIL_CNT_REG_WD_FIRST_OK_SHIFT)
#define PMIC_WD_FAIL_CNT_REG_WD_BAD_EVENT_MASK       \
                         (0x1U << PMIC_WD_FAIL_CNT_REG_WD_BAD_EVENT_SHIFT)

/*!
 * \brief  Watchdog Long Window Max, Min and Divisor macros
 */
/*! \brief  Valid only for PG2.0 */
#define PMIG_WD_LONGWIN_80_MILLISEC                 (80U)
#define PMIG_WD_LONGWIN_125_MILLISEC                (125U)
#define PMIG_WD_LONGWIN_8000_MILLISEC               (8000U)

#define PMIG_WD_LONGWIN_MILLISEC_DIV_125            (125U)
#define PMIG_WD_LONGWIN_MILLISEC_DIV_4000           (4000U)


#define PMIG_WD_LONGWIN_MILLISEC_MIN_PG_2_0         (125U)
#define PMIG_WD_LONGWIN_MILLISEC_MAX_PG_2_0         (772000U)


#define PMIG_WD_LONGWIN_REG_VAL_0                   (0x0U)
#define PMIG_WD_LONGWIN_REG_VAL_1                   (0x1U)
#define PMIG_WD_LONGWIN_REG_VAL_64                  (0x40U)

/*! \brief  Valid only for PG1.0 */
#define PMIG_WD_LONGWIN_100_MILLISEC                (100U)
#define PMIG_WD_LONGWIN_MILLISEC_MIN                (3000U)
#define PMIG_WD_LONGWIN_MILLISEC_MAX                (765000U)
#define PMIG_WD_LONGWIN_MILLISEC_DIV                (3000U)

/*!
 * \brief  Watchdog Window1 Max, Min and Divisor macros
 */
#define PMIG_WD_WIN1_2_MICROSEC_MIN  (550U)
#define PMIG_WD_WIN1_2_MICROSEC_MAX  (70400U)
#define PMIG_WD_WIN1_2_MICROSEC_DIV  (550U)

#endif /* PMIC_WDG_PRIV_H_ */

#ifdef __cplusplus
}
#endif /* __cplusplus */
