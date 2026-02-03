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

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "pmic.h"
#include "pmic_wdg.h"

#include "pmic_io.h"
#include "pmic_common.h"
#include "regmap/wdg.h"

/* ========================================================================== */
/*                               Macros & Typedefs                            */
/* ========================================================================== */
#define CLEAR_ALL_STAT_BITS     (0xFFU)

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */
static inline void WDG_copyWdgCfg(const Pmic_WdgCfg_t *src, Pmic_WdgCfg_t *dst) {
    (void)memmove((void *)dst, (const void *)src, sizeof(Pmic_WdgCfg_t));
}

static inline void WDG_copyWdgError(const Pmic_WdgErrStatus_t *src, Pmic_WdgErrStatus_t *dst) {
    (void)memmove((void *)dst, (const void *)src, sizeof(Pmic_WdgErrStatus_t));
}

static inline void WDG_copyWdgFailCntStat(const Pmic_WdgFailCntStatus_t *src, Pmic_WdgFailCntStatus_t *dst) {
    (void)memmove((void *)dst, (const void *)src, sizeof(Pmic_WdgFailCntStatus_t));
}

static inline void WDG_copyWdgAnsInfo(const Pmic_WdgAnsInfo_t *src, Pmic_WdgAnsInfo_t *dst) {
    (void)memmove((void *)dst, (const void *)src, sizeof(Pmic_WdgAnsInfo_t));
}

static int32_t WDG_validatePmicCoreHandle(const Pmic_Handle_t *handle) {
    int32_t status = Pmic_checkHandle(handle);

    return status;
}

/**
 * @brief Check if WDG is in valid state for configuration
 *
 * Validates that WDG is enabled and in Long Window mode before allowing
 * configuration changes via Pmic_wdgSetCfg().
 *
 * @return PMIC_ST_SUCCESS if state is valid, PMIC_ST_ERR_NOT_SUPPORTED otherwise
 */
static int32_t WDG_checkCfgState(const Pmic_Handle_t *handle) {
    int32_t status;
    bool isEnabled = false;

    status = Pmic_wdgGetEnableState(handle, &isEnabled);
    if ((status == PMIC_ST_SUCCESS) && !isEnabled) {
        status = PMIC_ST_ERR_NOT_SUPPORTED;
    }

    if (status == PMIC_ST_SUCCESS) {
        status = Pmic_wdgGetReturnToLongWindow(handle, &isEnabled);
        if ((status == PMIC_ST_SUCCESS) && !isEnabled) {
            status = PMIC_ST_ERR_NOT_SUPPORTED;
        }
    }

    return status;
}

static int32_t WDG_setWindowsTimeIntervals(const Pmic_Handle_t *handle, const Pmic_WdgCfg_t *config) {
    int32_t status = PMIC_ST_SUCCESS;

    /* Set wdg long window time interval */
    if (Pmic_validParamCheck(config->validParams, PMIC_CFG_WDG_LONGWINDURATION_VALID)) {
        status = Pmic_ioTxByte_CS(handle, PMIC_WD_LONGWIN_CFG_REG, config->longWinCode);
    }

    /* Set Window 1 time interval */
    if (Pmic_validParamStatusCheck(config->validParams, PMIC_CFG_WDG_WIN1DURATION_VALID, status)) {
        if (config->win1Code > PMIC_WDG_WIN_CODE_MAX) {
            status = PMIC_ST_ERR_INV_PARAM;
        } else {
            status = Pmic_ioTxByte_CS(handle, PMIC_WD_WIN1_CFG_REG, config->win1Code);
        }
    }

    /* Set Window 2 time interval */
    if (Pmic_validParamStatusCheck(config->validParams, PMIC_CFG_WDG_WIN2DURATION_VALID, status)) {
        if (config->win2Code > PMIC_WDG_WIN_CODE_MAX) {
            status = PMIC_ST_ERR_INV_PARAM;
        } else {
            status = Pmic_ioTxByte_CS(handle, PMIC_WD_WIN2_CFG_REG, config->win2Code);
        }
    }

    return status;
}

static int32_t WDG_getLongWindowTimeInterval(const Pmic_Handle_t *handle, Pmic_WdgCfg_t *config) {
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regVal = 0U;

    if (Pmic_validParamCheck(config->validParams, PMIC_CFG_WDG_LONGWINDURATION_VALID)) {
        status = Pmic_ioRxByte_CS(handle, PMIC_WD_LONGWIN_CFG_REG, &regVal);

        if (status == PMIC_ST_SUCCESS) {
            config->longWinCode = regVal;
        }
    }

    return status;
}

static int32_t WDG_getWindow1TimeInterval(const Pmic_Handle_t *handle, Pmic_WdgCfg_t *config) {
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regVal = 0U;

    if (Pmic_validParamCheck(config->validParams, PMIC_CFG_WDG_WIN1DURATION_VALID)) {
        status = Pmic_ioRxByte_CS(handle, PMIC_WD_WIN1_CFG_REG, &regVal);

        if (status == PMIC_ST_SUCCESS) {
            config->win1Code = regVal;
        }
    }

    return status;
}

static int32_t WDG_getWindow2TimeInterval(const Pmic_Handle_t *handle, Pmic_WdgCfg_t *config) {
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regVal = 0U;

    if (Pmic_validParamCheck(config->validParams, PMIC_CFG_WDG_WIN2DURATION_VALID)) {
        status = Pmic_ioRxByte_CS(handle, PMIC_WD_WIN2_CFG_REG, &regVal);

        if (PMIC_ST_SUCCESS == status) {
            config->win2Code = regVal;
        }
    }

    return status;
}

static int32_t WDG_getWindowsTimeIntervals(const Pmic_Handle_t *handle, Pmic_WdgCfg_t *config) {
    int32_t status = PMIC_ST_SUCCESS;

    /* Get wdg long window time interval */
    status = WDG_getLongWindowTimeInterval(handle, config);

    /* Get Window 1 time interval */
    if (status == PMIC_ST_SUCCESS) {
        status = WDG_getWindow1TimeInterval(handle, config);
    }

    /* Get Window 2 time interval */
    if (status == PMIC_ST_SUCCESS) {
        status = WDG_getWindow2TimeInterval(handle, config);
    }

    return status;
}

static int32_t WDG_setThresholds(const Pmic_Handle_t *handle, const Pmic_WdgCfg_t *config) {
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regVal = 0U;

    Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);

    // Read WD_TH_CFG register
    status = Pmic_ioRxByte(handle, PMIC_WD_THR_CFG_REG, &regVal);

    if (status == PMIC_ST_SUCCESS) {
        /* Set wdg Reset threshold value */
        if (Pmic_validParamCheck(config->validParams, PMIC_CFG_WDG_THRESHOLD_RESET_VALID)) {
            if (config->thresholdReset > PMIC_WDG_THRESHOLD_COUNT_MAX) {
                status = PMIC_ST_ERR_INV_PARAM;
            } else {
                Pmic_setBitField(&regVal, PMIC_WD_THR_RST_SHIFT, PMIC_WD_THR_RST_MASK, config->thresholdReset);
            }
        }

        /* Set wdg Fail threshold value */
        if ((status == PMIC_ST_SUCCESS) &&
            Pmic_validParamCheck(config->validParams, PMIC_CFG_WDG_THRESHOLD_FAIL_VALID)) {
            if (config->thresholdFail > PMIC_WDG_THRESHOLD_COUNT_MAX) {
                status = PMIC_ST_ERR_INV_PARAM;
            } else {
                Pmic_setBitField(&regVal, PMIC_WD_THR_FAIL_SHIFT, PMIC_WD_THR_FAIL_MASK, config->thresholdFail);
            }
        }

        // Write back modified WD_TH_CFG if all modifications were successful
        if (status == PMIC_ST_SUCCESS) {
            status = Pmic_ioTxByte(handle, PMIC_WD_THR_CFG_REG, regVal);
        }
    }

    Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);

    return status;
}

static int32_t WDG_getThresholds(const Pmic_Handle_t *handle, Pmic_WdgCfg_t *config) {
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regVal = 0U;

    status = Pmic_ioRxByte_CS(handle, PMIC_WD_THR_CFG_REG, &regVal);

    /* Get wdg reset threshold value */
    if (Pmic_validParamStatusCheck(config->validParams, PMIC_CFG_WDG_THRESHOLD_RESET_VALID, status)) {
        config->thresholdReset = Pmic_getBitField(regVal, PMIC_WD_THR_RST_SHIFT, PMIC_WD_THR_RST_MASK);
    }

    /* Get wdg fail threshold value */
    if (Pmic_validParamStatusCheck(config->validParams, PMIC_CFG_WDG_THRESHOLD_FAIL_VALID, status)) {
        config->thresholdFail = Pmic_getBitField(regVal, PMIC_WD_THR_FAIL_SHIFT, PMIC_WD_THR_FAIL_MASK);
    }

    return status;
}

/*! \brief  Function to set watchdog QA configurations */
static int32_t WDG_setQAConfigurations(const Pmic_Handle_t *handle, const Pmic_WdgCfg_t *config) {
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regVal = 0U;

    Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);

    // Read the WD_QA_CFG register
    status = Pmic_ioRxByte(handle, PMIC_WD_QA_CFG_REG, &regVal);

    if (status == PMIC_ST_SUCCESS) {
        /* Set wdg QA Feedback value */
        if (Pmic_validParamCheck(config->validParams, PMIC_CFG_WDG_QA_FDBK_VALID)) {
            if (config->qaFdbk > PMIC_WDG_QA_FEEDBACK_VALUE_MAX) {
                status = PMIC_ST_ERR_INV_PARAM;
            } else {
                Pmic_setBitField(&regVal, PMIC_WD_QA_FDBK_SHIFT, PMIC_WD_QA_FDBK_MASK, config->qaFdbk);
            }
        }

        /* Set wdg QA LFSR value */
        if ((status == PMIC_ST_SUCCESS) &&
            Pmic_validParamCheck(config->validParams, PMIC_CFG_WDG_QA_LFSR_VALID)) {
            if (config->qaLfsr > PMIC_WDG_QA_LFSR_VALUE_MAX) {
                status = PMIC_ST_ERR_INV_PARAM;
            } else {
                Pmic_setBitField(&regVal, PMIC_WD_QA_LFSR_SHIFT, PMIC_WD_QA_LFSR_MASK, config->qaLfsr);
            }
        }

        /* Set wdg QA Question Seed value */
        if ((status == PMIC_ST_SUCCESS) &&
            Pmic_validParamCheck(config->validParams, PMIC_CFG_WDG_QA_QUES_SEED_VALID)) {
            if (config->qaQuesSeed > PMIC_WDG_QA_QUES_SEED_VALUE_MAX) {
                status = PMIC_ST_ERR_INV_PARAM;
            } else {
                Pmic_setBitField(&regVal, PMIC_WD_QA_SEED_SHIFT, PMIC_WD_QA_SEED_MASK, config->qaQuesSeed);
            }
        }

        // Write back the modified WD_QA_CFG register if all modifications were successful
        if (status == PMIC_ST_SUCCESS) {
            status = Pmic_ioTxByte(handle, PMIC_WD_QA_CFG_REG, regVal);
        }
    }

    Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);

    return status;
}

static int32_t WDG_getQAConfigurations(const Pmic_Handle_t *handle, Pmic_WdgCfg_t *config) {
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regVal = 0U;

    status = Pmic_ioRxByte_CS(handle, PMIC_WD_QA_CFG_REG, &regVal);

    /* Get wdg QA Feedback value */
    if (Pmic_validParamStatusCheck(config -> validParams, PMIC_CFG_WDG_QA_FDBK_VALID, status)) {
        config->qaFdbk = Pmic_getBitField(regVal, PMIC_WD_QA_FDBK_SHIFT, PMIC_WD_QA_FDBK_MASK);
    }

    /* Get wdg QA LFSR value */
    if (Pmic_validParamStatusCheck(config->validParams, PMIC_CFG_WDG_QA_LFSR_VALID, status)) {
        config->qaLfsr = Pmic_getBitField(regVal, PMIC_WD_QA_LFSR_SHIFT, PMIC_WD_QA_LFSR_MASK);
    }

    /* Get wdg QA Question Seed value */
    if (Pmic_validParamStatusCheck(config->validParams, PMIC_CFG_WDG_QA_QUES_SEED_VALID, status)) {
        config->qaQuesSeed = Pmic_getBitField(regVal, PMIC_WD_QA_SEED_SHIFT, PMIC_WD_QA_SEED_MASK);
    }

    return status;
}

static int32_t WDG_setResetEnableCfg(const Pmic_Handle_t *handle, const Pmic_WdgCfg_t *config)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);

    // Read WD_ENABLE_REG
    status = Pmic_ioRxByte(handle, PMIC_WD_ENABLE_REG, &regData);

    // Modify WD_RST_EN
    if (Pmic_validParamStatusCheck(config->validParams, PMIC_CFG_WDG_RST_EN_VALID, status)) {
        Pmic_setBitField_b(&regData, PMIC_WD_RST_EN_SHIFT, config->rstEn);
    }

    // Write WD_ENABLE_REG
    if (status == PMIC_ST_SUCCESS) {
        status = Pmic_ioTxByte(handle, PMIC_WD_ENABLE_REG, regData);
    }

    Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);

    return status;
}

static int32_t WDG_getResetEnableCfg(const Pmic_Handle_t *handle, Pmic_WdgCfg_t *config)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    // Read WD_ENABLE_REG
    status = Pmic_ioRxByte_CS(handle, PMIC_WD_ENABLE_REG, &regData);

    // Extract WD_RST_EN
    if (Pmic_validParamStatusCheck(config->validParams, PMIC_CFG_WDG_RST_EN_VALID, status)) {
        config->rstEn = Pmic_getBitField_b(regData, PMIC_WD_RST_EN_SHIFT);
    }

    return status;
}

/*!
 * \brief  Function to get watchdog QA answer count and question value
 */
static int32_t WDG_getQuestionAndAnswer(const Pmic_Handle_t *handle, uint8_t *ansCnt, uint8_t *quesVal) {
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regVal = 0U;

    /* Reading answer count and question value */
    status = Pmic_ioRxByte_CS(handle, PMIC_WD_QA_CNT_REG, &regVal);

    /* Extract bits from register */
    if (status == PMIC_ST_SUCCESS) {
        *ansCnt = Pmic_getBitField(regVal, PMIC_WD_ANSW_CNT_SHIFT, PMIC_WD_ANSW_CNT_MASK);
        *quesVal = Pmic_getBitField(regVal, PMIC_WD_QUESTION_SHIFT, PMIC_WD_QUESTION_MASK);
    }

    /* Check whether INT_TOP_STATUS is set or not, if so indicate this to the user. */
    if ((status == PMIC_ST_SUCCESS) && Pmic_getBitField_b(regVal, PMIC_INT_TOP_STATUS_SHIFT)) {
        Pmic_irqResponseCallback(handle);
    }

    return status;
}

/** @brief Implements a 4x4 mux function to be used for calculation of WDG answers. */
static inline uint8_t mux(uint8_t x0, uint8_t x1, uint8_t x2, uint8_t x3, uint8_t qaFdk) {
    uint8_t y = 0U;

    switch (qaFdk) {
    case 0U:
        y = x0;
        break;
    case 1U:
        y = x1;
        break;
    case 2U:
        y = x2;
        break;
    case 3U:
    default:
        y = x3;
        break;
    }

    return y;
}

/** @brief Function to Evaluate Watchdog Answers */
static uint8_t WDG_evalAnswerByte(uint8_t qaQuesCnt, uint8_t qaAnsCnt, uint8_t qaFbk) {
    uint8_t q0 = 0U, q1 = 0U, q2 = 0U, q3 = 0U;
    uint8_t a0 = 0U, a1 = 0U;
    uint8_t qaAns = 0U;

    q0 = (qaQuesCnt >> 0U) & 1U;
    q1 = (qaQuesCnt >> 1U) & 1U;
    q2 = (qaQuesCnt >> 2U) & 1U;
    q3 = (qaQuesCnt >> 3U) & 1U;

    a0 = (qaAnsCnt >> 0U) & 1U;
    a1 = (qaAnsCnt >> 1U) & 1U;

    /* Reference-Answer-X[0] */
    qaAns = mux(q0, q1, q2, q3, qaFbk) ^ (mux(q3, q2, q1, q0, qaFbk) ^ a1);
    /* Reference-Answer-X[1] */
    qaAns |= (uint8_t)((mux(q0, q1, q2, q3, qaFbk) ^ (mux(q2, q1, q0, q3, qaFbk) ^ q1) ^ a1) << 1U);
    /* Reference-Answer-X[2] */
    qaAns |= (uint8_t)((mux(q0, q3, q1, q1, qaFbk) ^ (mux(q3, q2, q1, q0, qaFbk) ^ q1) ^ a1) << 2U);
    /* Reference-Answer-X[3] */
    qaAns |= (uint8_t)((mux(q2, q1, q0, q3, qaFbk) ^ (mux(q0, q3, q2, q1, qaFbk) ^ q3) ^ a1) << 3U);
    /* Reference-Answer-X[4] */
    qaAns |= (uint8_t)((mux(q1, q0, q2, q3, qaFbk) ^ a0) << 4U);
    /* Reference-Answer-X[5] */
    qaAns |= (uint8_t)((mux(q3, q2, q1, q0, qaFbk) ^ a0) << 5U);
    /* Reference-Answer-X[6] */
    qaAns |= (uint8_t)((mux(q0, q3, q2, q1, qaFbk) ^ a0) << 6U);
    /* Reference-Answer-X[7] */
    qaAns |= (uint8_t)((mux(q2, q1, q0, q3, qaFbk) ^ a0) << 7U);

    return qaAns;
}

/**
 * @brief Function to Evaluate and write Watchdog Answer based on qaFdbk,
 * qaAnsCnt and qaQuesCnt Value
 */
static int32_t WDG_qaEvalAndWriteAnswer(
    const Pmic_Handle_t *handle,
    uint8_t qaAnsCnt,
    uint8_t qaQuesCnt,
    uint8_t qaFbk)
{
    int32_t status = PMIC_ST_SUCCESS;
    const uint8_t answer = WDG_evalAnswerByte(qaQuesCnt, qaAnsCnt, qaFbk);

    /* Write watchdog Answers byte */
    status = Pmic_ioTxByte_CS(handle, PMIC_WD_ANSWER_REG, answer);

    return status;
}

/* ========================================================================== */
/*                        Interface Implementations                           */
/* ========================================================================== */
int32_t Pmic_wdgEnable(const Pmic_Handle_t *handle) {
    return Pmic_wdgSetEnableState(handle, PMIC_ENABLE);
}

int32_t Pmic_wdgDisable(const Pmic_Handle_t *handle) {
    return Pmic_wdgSetEnableState(handle, PMIC_DISABLE);
}

int32_t Pmic_wdgSetEnableState(const Pmic_Handle_t *handle, bool enable) {
    int32_t status = WDG_validatePmicCoreHandle(handle);
    uint8_t regVal = 0U;

    Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);

    if (status == PMIC_ST_SUCCESS) {
        status = Pmic_ioRxByte(handle, PMIC_WD_ENABLE_REG, &regVal);
    }

    if (status == PMIC_ST_SUCCESS) {
        Pmic_setBitField_b(&regVal, PMIC_WD_EN_SHIFT, enable);
        status = Pmic_ioTxByte(handle, PMIC_WD_ENABLE_REG, regVal);
    }

    Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_wdgGetEnableState(const Pmic_Handle_t *handle, bool *isEnabled) {
    int32_t status = WDG_validatePmicCoreHandle(handle);
    uint8_t regData = 0;

    // Validate parameters
    if ((status == PMIC_ST_SUCCESS) && (isEnabled == NULL)) {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if (status == PMIC_ST_SUCCESS) {
        status = Pmic_ioRxByte_CS(handle, PMIC_WD_ENABLE_REG, &regData);
    }

    if (status == PMIC_ST_SUCCESS) {
        *isEnabled = Pmic_getBitField_b(regData, PMIC_WD_EN_SHIFT);
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_wdgSetCfg(const Pmic_Handle_t *handle, const Pmic_WdgCfg_t *config) {
    Pmic_WdgCfg_t localConfig;
    int32_t status = WDG_validatePmicCoreHandle(handle);

    if ((status == PMIC_ST_SUCCESS) && (config == NULL)) {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if (status == PMIC_ST_SUCCESS) {
        WDG_copyWdgCfg(config, &localConfig);
    }

    if (status == PMIC_ST_SUCCESS) {
        status = WDG_checkCfgState(handle);
    }

    if (status == PMIC_ST_SUCCESS) {
        status = WDG_setWindowsTimeIntervals(handle, &localConfig);
    }

    if (status == PMIC_ST_SUCCESS) {
        status = WDG_setThresholds(handle, &localConfig);
    }

    if (status == PMIC_ST_SUCCESS) {
        status = WDG_setQAConfigurations(handle, &localConfig);
    }

    // Set WD_RST_EN configuration
    if (status == PMIC_ST_SUCCESS) {
        status = WDG_setResetEnableCfg(handle, &localConfig);
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_wdgGetCfg(const Pmic_Handle_t *handle, Pmic_WdgCfg_t *config) {
    Pmic_WdgCfg_t localConfig;
    int32_t status = WDG_validatePmicCoreHandle(handle);

    if ((status == PMIC_ST_SUCCESS) && (config == NULL)) {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if (status == PMIC_ST_SUCCESS) {
        WDG_copyWdgCfg(config, &localConfig);
    }

    if (status == PMIC_ST_SUCCESS) {
        status = WDG_getWindowsTimeIntervals(handle, &localConfig);
    }

    if (status == PMIC_ST_SUCCESS) {
        status = WDG_getThresholds(handle, &localConfig);
    }

    if (status == PMIC_ST_SUCCESS) {
        status = WDG_getQAConfigurations(handle, &localConfig);
    }

    // Get WD_RST_EN configuration
    if (status == PMIC_ST_SUCCESS) {
        status = WDG_getResetEnableCfg(handle, &localConfig);
    }

    if (status == PMIC_ST_SUCCESS) {
        WDG_copyWdgCfg(&localConfig, config);
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_wdgSetPowerHold(const Pmic_Handle_t *handle, bool enable) {
    int32_t status = WDG_validatePmicCoreHandle(handle);
    uint8_t regVal = 0U;

    Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);

    if (status == PMIC_ST_SUCCESS) {
        status = Pmic_ioRxByte(handle, PMIC_WD_MODE_REG, &regVal);
    }

    if (status == PMIC_ST_SUCCESS) {
        Pmic_setBitField_b(&regVal, PMIC_WD_PWRHOLD_SHIFT, enable);
        status = Pmic_ioTxByte(handle, PMIC_WD_MODE_REG, regVal);
    }

    Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_wdgGetPowerHold(const Pmic_Handle_t *handle, bool *isEnabled) {
    int32_t status = WDG_validatePmicCoreHandle(handle);
    uint8_t regData = 0U;

    // Validate parameters
    if ((status == PMIC_ST_SUCCESS) && (isEnabled == NULL)) {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);

    if (status == PMIC_ST_SUCCESS) {
        status = Pmic_ioRxByte(handle, PMIC_WD_MODE_REG, &regData);
    }

    if (status == PMIC_ST_SUCCESS) {
        *isEnabled = Pmic_getBitField_b(regData, PMIC_WD_PWRHOLD_SHIFT);
    }

    Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_wdgSetReturnToLongWindow(const Pmic_Handle_t *handle, bool enable) {
    int32_t status = WDG_validatePmicCoreHandle(handle);
    uint8_t regVal = 0U;

    Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);

    if (status == PMIC_ST_SUCCESS) {
        status = Pmic_ioRxByte(handle, PMIC_WD_MODE_REG, &regVal);
    }

    if (status == PMIC_ST_SUCCESS) {
        Pmic_setBitField_b(&regVal, PMIC_WD_RETURN_LONGWIN_SHIFT, enable);
        status = Pmic_ioTxByte(handle, PMIC_WD_MODE_REG, regVal);
    }

    Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_wdgGetReturnToLongWindow(const Pmic_Handle_t *handle, bool *isEnabled) {
    int32_t status = WDG_validatePmicCoreHandle(handle);
    uint8_t regData = 0U;

    // Validate parameters
    if ((status == PMIC_ST_SUCCESS) && (isEnabled == NULL)) {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if (status == PMIC_ST_SUCCESS) {
        status = Pmic_ioRxByte_CS(handle, PMIC_WD_MODE_REG, &regData);
    }

    if (status == PMIC_ST_SUCCESS) {
        *isEnabled = Pmic_getBitField_b(regData, PMIC_WD_RETURN_LONGWIN_SHIFT);
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_wdgGetErrStatus(const Pmic_Handle_t *handle, Pmic_WdgErrStatus_t *errors) {
    Pmic_WdgErrStatus_t localErrors;
    int32_t status = WDG_validatePmicCoreHandle(handle);
    uint8_t regVal = 0x0U;

    if ((status == PMIC_ST_SUCCESS) && (errors == NULL)) {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if (status == PMIC_ST_SUCCESS) {
        WDG_copyWdgError(errors, &localErrors);
    }

    /* Reading error status register */
    if (status == PMIC_ST_SUCCESS) {
        status = Pmic_ioRxByte_CS(handle, PMIC_WD_ERR_STAT_REG, &regVal);
    }

    /* Extract watchdog error status fields */
    if (status == PMIC_ST_SUCCESS) {
        if (Pmic_validParamCheck(localErrors.validParams, PMIC_CFG_WD_LONGWIN_TIMEOUT_ERR_VALID)) {
            localErrors.longWindowTimeout = Pmic_getBitField_b(regVal, PMIC_WD_LONGWIN_TMO_SHIFT);
        }

        if (Pmic_validParamCheck(localErrors.validParams, PMIC_CFG_WD_TIMEOUT_ERR_VALID)) {
            localErrors.timeout = Pmic_getBitField_b(regVal, PMIC_WD_TMO_SHIFT);
        }

        if (Pmic_validParamCheck(localErrors.validParams, PMIC_CFG_WD_ANSW_EARLY_ERR_VALID)) {
            localErrors.answerEarlyError = Pmic_getBitField_b(regVal, PMIC_WD_ANSW_EARLY_SHIFT);
        }

        if (Pmic_validParamCheck(localErrors.validParams, PMIC_CFG_WD_SEQ_ERR_ERR_VALID)) {
            localErrors.sequenceError = Pmic_getBitField_b(regVal, PMIC_WD_SEQ_ERR_SHIFT);
        }

        if (Pmic_validParamCheck(localErrors.validParams, PMIC_CFG_WD_ANSW_ERR_ERR_VALID)) {
            localErrors.answerError = Pmic_getBitField_b(regVal, PMIC_WD_ANSW_ERR_SHIFT);
        }

        if (Pmic_validParamCheck(localErrors.validParams, PMIC_CFG_WD_FAIL_INT_ERR_VALID)) {
            localErrors.failInt = Pmic_getBitField_b(regVal, PMIC_WD_FAIL_ERR_SHIFT);
        }

        if (Pmic_validParamCheck(localErrors.validParams, PMIC_CFG_WD_RST_INT_ERR_VALID)) {
            localErrors.resetInt = Pmic_getBitField_b(regVal, PMIC_WD_RST_ERR_SHIFT);
        }
    }

    if (status == PMIC_ST_SUCCESS) {
        WDG_copyWdgError(&localErrors, errors);
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_wdgClrErrStatus(const Pmic_Handle_t *handle, const Pmic_WdgErrStatus_t *errors) {
    Pmic_WdgErrStatus_t localErrors;
    int32_t status = WDG_validatePmicCoreHandle(handle);
    uint8_t regVal = 0x0U;

    if ((status == PMIC_ST_SUCCESS) && (errors == NULL)) {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if (status == PMIC_ST_SUCCESS) {
        WDG_copyWdgError(errors, &localErrors);
    }

    if (status == PMIC_ST_SUCCESS) {
        if (Pmic_validParamCheck(localErrors.validParams, PMIC_CFG_WD_LONGWIN_TIMEOUT_ERR_VALID)) {
            Pmic_setBitField_b(&regVal, PMIC_WD_LONGWIN_TMO_SHIFT, localErrors.longWindowTimeout);
        }

        if (Pmic_validParamCheck(localErrors.validParams, PMIC_CFG_WD_TIMEOUT_ERR_VALID)) {
            Pmic_setBitField_b(&regVal, PMIC_WD_TMO_SHIFT, localErrors.timeout);
        }

        if (Pmic_validParamCheck(localErrors.validParams, PMIC_CFG_WD_ANSW_EARLY_ERR_VALID)) {
            Pmic_setBitField_b(&regVal, PMIC_WD_ANSW_EARLY_SHIFT, localErrors.answerEarlyError);
        }

        if (Pmic_validParamCheck(localErrors.validParams, PMIC_CFG_WD_SEQ_ERR_ERR_VALID)) {
            Pmic_setBitField_b(&regVal, PMIC_WD_SEQ_ERR_SHIFT, localErrors.sequenceError);
        }

        if (Pmic_validParamCheck(localErrors.validParams, PMIC_CFG_WD_ANSW_ERR_ERR_VALID)) {
            Pmic_setBitField_b(&regVal, PMIC_WD_ANSW_ERR_SHIFT, localErrors.answerError);
        }

        if (Pmic_validParamCheck(localErrors.validParams, PMIC_CFG_WD_FAIL_INT_ERR_VALID)) {
            Pmic_setBitField_b(&regVal, PMIC_WD_FAIL_ERR_SHIFT, localErrors.failInt);
        }

        if (Pmic_validParamCheck(localErrors.validParams, PMIC_CFG_WD_RST_INT_ERR_VALID)) {
            Pmic_setBitField_b(&regVal, PMIC_WD_RST_ERR_SHIFT, localErrors.resetInt);
        }
    }

    // WD_ERR_STAT register is write 1 to clear, write composed data back to register (if any bits are set).
    if ((status == PMIC_ST_SUCCESS) && (regVal != 0U)) {
        Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);
        status = Pmic_ioTxByte(handle, PMIC_WD_ERR_STAT_REG, regVal);
        Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_wdgClrErrStatusAll(const Pmic_Handle_t *handle) {
    int32_t status = WDG_validatePmicCoreHandle(handle);

    // WD_ERR_STAT register is write 1 to clear, write all bits as 1 to clear.
    if (status == PMIC_ST_SUCCESS) {
        Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);
        status = Pmic_ioTxByte(handle, PMIC_WD_ERR_STAT_REG, CLEAR_ALL_STAT_BITS);
        Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_wdgGetFailCntStatus(const Pmic_Handle_t *handle, Pmic_WdgFailCntStatus_t *failCount) {
    Pmic_WdgFailCntStatus_t localFailCount;
    int32_t status = WDG_validatePmicCoreHandle(handle);
    uint8_t regVal = 0x00U;

    if ((status == PMIC_ST_SUCCESS) && (failCount == NULL)) {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if (status == PMIC_ST_SUCCESS) {
        WDG_copyWdgFailCntStat(failCount, &localFailCount);
    }

    /* Reading error status register */
    if (status == PMIC_ST_SUCCESS) {
        status = Pmic_ioRxByte_CS(handle, PMIC_WD_FAIL_CNT_REG, &regVal);
    }

    /* Get watchdog Bad Event status */
    if ((status == PMIC_ST_SUCCESS) && Pmic_validParamCheck(localFailCount.validParams, PMIC_CFG_WD_BAD_EVENT_STAT_VALID)) {
        localFailCount.badEvent = Pmic_getBitField_b(regVal, PMIC_WD_BAD_EVENT_SHIFT);
    }

    /* Get watchdog Good Event status */
    if ((status == PMIC_ST_SUCCESS) && Pmic_validParamCheck(localFailCount.validParams, PMIC_CFG_WD_GOOD_EVENT_STAT_VALID)) {
        localFailCount.goodEvent = Pmic_getBitField_b(regVal, PMIC_WD_FIRST_OK_SHIFT);
    }

    /* Get watchdog Fail count Value */
    if ((status == PMIC_ST_SUCCESS) && Pmic_validParamCheck(localFailCount.validParams, PMIC_CFG_WD_FAIL_CNT_VAL_VALID)) {
        localFailCount.wdFailCnt = Pmic_getBitField(regVal, PMIC_WD_ERR_CNT_SHIFT, PMIC_WD_ERR_CNT_MASK);
    }

    if (status == PMIC_ST_SUCCESS) {
        WDG_copyWdgFailCntStat(&localFailCount, failCount);
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_wdgQaWriteAnswer(const Pmic_Handle_t *handle) {
    int32_t status = WDG_validatePmicCoreHandle(handle);

    uint8_t qaAnsCnt = 0U;
    uint8_t qaQuesCnt = 0U;
    uint8_t qaFbk = 0U;

    /* Get wdg QA Feedback value */
    if (status == PMIC_ST_SUCCESS) {
        status = Pmic_ioRxByte_CS(handle, PMIC_WD_QA_CFG_REG, &qaFbk);
        qaFbk = Pmic_getBitField(qaFbk, PMIC_WD_QA_FDBK_SHIFT, PMIC_WD_QA_FDBK_MASK);
    }

    /* Read Question value and Answer count */
    if (status == PMIC_ST_SUCCESS) {
        status = WDG_getQuestionAndAnswer(handle, &qaAnsCnt, &qaQuesCnt);
    }

    /* Write Answer-X */
    if (status == PMIC_ST_SUCCESS) {
        status = WDG_qaEvalAndWriteAnswer(handle, qaAnsCnt, qaQuesCnt, qaFbk);
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_wdgGetFdbkRegData(const Pmic_Handle_t *handle, uint8_t *regData) {
    int32_t status = Pmic_checkHandle(handle);

    if ((status == PMIC_ST_SUCCESS) && (regData == NULL)) {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if (status == PMIC_ST_SUCCESS) {
        status = Pmic_ioRxByte_CS(handle, PMIC_WD_QA_CFG_REG, regData);
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_wdgExtractFdbk(const Pmic_Handle_t *handle, uint8_t regData, Pmic_WdgAnsInfo_t *wdgAnsInfo) {
    Pmic_WdgAnsInfo_t localWdgAnsInfo = {0};
    int32_t status = PMIC_ST_SUCCESS;

    if (wdgAnsInfo == NULL) {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if (status == PMIC_ST_SUCCESS) {
        localWdgAnsInfo.fdbk = Pmic_getBitField(regData, PMIC_WD_QA_FDBK_SHIFT, PMIC_WD_QA_FDBK_MASK);
        WDG_copyWdgAnsInfo(&localWdgAnsInfo, wdgAnsInfo);
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_wdgGetAnsCntAndQuesRegData(const Pmic_Handle_t *handle, uint8_t *regData) {
    int32_t status = Pmic_checkHandle(handle);

    if ((status == PMIC_ST_SUCCESS) && (regData == NULL)) {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if (status == PMIC_ST_SUCCESS) {
        status = Pmic_ioRxByte_CS(handle, PMIC_WD_QA_CNT_REG, regData);
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_wdgExtractAnsCntAndQues(const Pmic_Handle_t *handle, uint8_t regData, Pmic_WdgAnsInfo_t *wdgAnsInfo) {
    Pmic_WdgAnsInfo_t localWdgAnsInfo = {0};
    int32_t status = Pmic_checkHandle(handle);

    if ((status == PMIC_ST_SUCCESS) && (wdgAnsInfo == NULL)) {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if (status == PMIC_ST_SUCCESS) {
        localWdgAnsInfo.ansCnt = Pmic_getBitField(regData, PMIC_WD_ANSW_CNT_SHIFT, PMIC_WD_ANSW_CNT_MASK);
        localWdgAnsInfo.question = Pmic_getBitField(regData, PMIC_WD_QUESTION_SHIFT, PMIC_WD_QUESTION_MASK);

        // Execute application-specific IRQ response if INT_TOP_STATUS is 1
        if (Pmic_getBitField_b(regData, PMIC_INT_TOP_STATUS_SHIFT)) {
            Pmic_irqResponseCallback(handle);
        }

        WDG_copyWdgAnsInfo(&localWdgAnsInfo, wdgAnsInfo);
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_wdgWriteAnswer(const Pmic_Handle_t *handle, const Pmic_WdgAnsInfo_t *wdgAnsInfo) {
    Pmic_WdgAnsInfo_t localWdgAnsInfo;
    uint8_t regData;
    uint8_t question;
    uint8_t ansCnt;
    uint8_t fdbk;
    int32_t status = Pmic_checkHandle(handle);

    if ((status == PMIC_ST_SUCCESS) && (wdgAnsInfo == NULL)) {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if (status == PMIC_ST_SUCCESS) {
        WDG_copyWdgAnsInfo(wdgAnsInfo, &localWdgAnsInfo);
        question = localWdgAnsInfo.question;
        ansCnt = localWdgAnsInfo.ansCnt;
        fdbk = localWdgAnsInfo.fdbk;
        regData = WDG_evalAnswerByte(question, ansCnt, fdbk);
        status = Pmic_ioTxByte_CS(handle, (uint8_t)PMIC_WD_ANSWER_REG, regData);
    }

    return Pmic_logStatus(handle, status);
}
