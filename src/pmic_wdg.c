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

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include "pmic.h"
#include "pmic_wdg.h"
#include "pmic_core.h"
#include "pmic_io.h"
#include "regmap/wdg.h"

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */
/**
 * @brief Watchdog Long Window Max, Min and Divisor macros
 * @note  Valid only for PG2.0
 */
#define PMIC_WD_LONGWIN_80_MS         (uint32_t)(80U)
#define PMIC_WD_LONGWIN_125_MS        (uint32_t)(125U)
#define PMIC_WD_LONGWIN_8000_MS       (uint32_t)(8000U)

#define PMIC_WD_LONGWIN_MS_DIV_125    (uint32_t)(125U)
#define PMIC_WD_LONGWIN_MS_DIV_4000   (uint32_t)(4000U)

#define PMIC_WD_LONGWIN_MS_MIN_PG_2_0 (uint32_t)(125U)
#define PMIC_WD_LONGWIN_MS_MAX_PG_2_0 (uint32_t)(772000U)

#define PMIC_WD_LONGWIN_RANGE_LOW     (0x0U)
#define PMIC_WD_LONGWIN_RANGE_MID     (0x1U)
#define PMIC_WD_LONGWIN_RANGE_HI      (0x40U)

/** @note Valid only for PG1.0 */
#define PMIC_WD_LONGWIN_100_MS        (uint32_t)(100U)
#define PMIC_WD_LONGWIN_MS_MIN        (uint32_t)(3000U)
#define PMIC_WD_LONGWIN_MS_MAX        (uint32_t)(765000U)
#define PMIC_WD_LONGWIN_MS_DIV        (uint32_t)(3000U)

/** @brief  Watchdog Window1 Max, Min and Divisor macros */
#define PMIC_WD_WIN1_2_US_MIN         (uint32_t)(550U)
#define PMIC_WD_WIN1_2_US_MAX         (uint32_t)(70400U)
#define PMIC_WD_WIN1_2_US_DIV         (uint32_t)(550U)

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */
static int32_t WDG_validatePmicCoreHandle(const Pmic_CoreHandle_t *handle) {
    int32_t status = Pmic_checkPmicCoreHandle(handle);

    /* Check the watch dog sub-system supported by pmic device */
    if ((status == PMIC_ST_SUCCESS) && !handle->pPmic_SubSysInfo->wdgEnable) {
        status = PMIC_ST_ERR_INV_DEVICE;
    }

    return status;
}

static uint8_t WDG_convertLongWinTimeToBits(uint32_t longWinDuration_ms) {
    uint8_t regVal = 0U;

    if (longWinDuration_ms == PMIC_WD_LONGWIN_80_MS) {
        regVal = PMIC_WD_LONGWIN_RANGE_LOW;
    } else if (longWinDuration_ms <= PMIC_WD_LONGWIN_8000_MS) {
        regVal = (uint8_t)(longWinDuration_ms / PMIC_WD_LONGWIN_MS_DIV_125);
    } else {
        regVal = (uint8_t)(longWinDuration_ms / PMIC_WD_LONGWIN_MS_DIV_4000) + 62U;
    }

    return regVal;
}

static uint32_t WDG_convertLongWinTimeFromBits(uint32_t longWinDuration) {
    uint32_t valInMs = 0U;

    if (longWinDuration == PMIC_WD_LONGWIN_RANGE_LOW) {
        valInMs = PMIC_WD_LONGWIN_80_MS;
    } else if (longWinDuration <= PMIC_WD_LONGWIN_RANGE_HI) {
        valInMs = (uint32_t)(longWinDuration * PMIC_WD_LONGWIN_MS_DIV_125);
    } else {
        valInMs = (uint32_t)((longWinDuration * PMIC_WD_LONGWIN_MS_DIV_4000) - 24800U);
    }

    return valInMs;
}

static int32_t WDG_setLongWindowTimeInterval(Pmic_CoreHandle_t *handle, const Pmic_WdgCfg_t *config) {
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regVal = 0U;

    if (Pmic_validParamCheck(config->validParams, PMIC_CFG_WDG_LONGWINDURATION_VALID)) {
        const bool longWinNot80ms = (config->longWinDuration_ms != PMIC_WD_LONGWIN_80_MS);
        const bool longWinBelowMin = (config->longWinDuration_ms < PMIC_WD_LONGWIN_MS_MIN_PG_2_0);
        const bool longWinAboveMax = (config->longWinDuration_ms > PMIC_WD_LONGWIN_MS_MAX_PG_2_0);

        if (longWinNot80ms && (longWinBelowMin || longWinAboveMax)) {
            status = PMIC_ST_ERR_INV_PARAM;
        }

        if (status == PMIC_ST_SUCCESS) {
            regVal = WDG_convertLongWinTimeToBits(config->longWinDuration_ms);
        }

        if (status == PMIC_ST_SUCCESS) {
            Pmic_criticalSectionStart(handle);
            status = Pmic_commIntf_sendByte(handle, PMIC_WD_LONGWIN_CFG_REG, regVal);
            Pmic_criticalSectionStop(handle);
        }
    }

    return status;
}

static int32_t WDG_setWindow1TimeInterval(Pmic_CoreHandle_t *handle, const Pmic_WdgCfg_t *config) {
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regVal = 0U;

    if (Pmic_validParamCheck(config->validParams, PMIC_CFG_WDG_WIN1DURATION_VALID)) {
        const bool winBelowMin = (config->win1Duration_us < PMIC_WD_WIN1_2_US_MIN);
        const bool winAboveMax = (config->win1Duration_us > PMIC_WD_WIN1_2_US_MAX);

        if (winBelowMin || winAboveMax) {
            status = PMIC_ST_ERR_INV_PARAM;
        }

        if (status == PMIC_ST_SUCCESS) {
            Pmic_setBitField(&regVal,
                PMIC_WD_WIN1_SHIFT,
                PMIC_WD_WIN1_MASK,
                (uint8_t)(config->win1Duration_us / PMIC_WD_WIN1_2_US_DIV) - 1U);

            Pmic_criticalSectionStart(handle);
            status = Pmic_commIntf_sendByte(handle, PMIC_WD_WIN1_CFG_REG, regVal);
            Pmic_criticalSectionStop(handle);
        }
    }

    return status;
}

static int32_t WDG_setWindow2TimeInterval(Pmic_CoreHandle_t *handle, const Pmic_WdgCfg_t *config) {
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regVal = 0U;

    if (Pmic_validParamCheck(config->validParams, PMIC_CFG_WDG_WIN2DURATION_VALID)) {
        const bool winBelowMin = (config->win2Duration_us < PMIC_WD_WIN1_2_US_MIN);
        const bool winAboveMax = (config->win2Duration_us > PMIC_WD_WIN1_2_US_MAX);

        if (winBelowMin || winAboveMax) {
            status = PMIC_ST_ERR_INV_PARAM;
        }

        if (status == PMIC_ST_SUCCESS) {
            Pmic_setBitField(&regVal,
                PMIC_WD_WIN2_SHIFT,
                PMIC_WD_WIN2_MASK,
                (uint8_t)(config->win2Duration_us / PMIC_WD_WIN1_2_US_DIV) - 1U);

            Pmic_criticalSectionStart(handle);
            status = Pmic_commIntf_sendByte(handle, PMIC_WD_WIN2_CFG_REG, regVal);
            Pmic_criticalSectionStop(handle);
        }
    }

    return status;
}

static int32_t WDG_setWindowsTimeIntervals(Pmic_CoreHandle_t *handle, const Pmic_WdgCfg_t *config) {
    int32_t status = PMIC_ST_SUCCESS;

    /* Set wdg long window time interval */
    status = WDG_setLongWindowTimeInterval(handle, config);

    /* Set Window 1 time interval */
    if (status == PMIC_ST_SUCCESS) {
        status = WDG_setWindow1TimeInterval(handle, config);
    }

    /* Set Window 2 time interval */
    if (status == PMIC_ST_SUCCESS) {
        status = WDG_setWindow2TimeInterval(handle, config);
    }

    return status;
}

static int32_t WDG_getLongWindowTimeInterval(Pmic_CoreHandle_t *handle, Pmic_WdgCfg_t *config) {
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regVal = 0U;

    if (Pmic_validParamCheck(config->validParams, PMIC_CFG_WDG_LONGWINDURATION_VALID)) {
        Pmic_criticalSectionStart(handle);
        status = Pmic_commIntf_recvByte(handle, PMIC_WD_LONGWIN_CFG_REG, &regVal);
        Pmic_criticalSectionStop(handle);

        if (status == PMIC_ST_SUCCESS) {
            config->longWinDuration_ms = WDG_convertLongWinTimeFromBits(regVal);
        }
    }

    return status;
}

static int32_t WDG_getWindow1TimeInterval(Pmic_CoreHandle_t *handle, Pmic_WdgCfg_t *config) {
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regVal = 0U;

    if (Pmic_validParamCheck(config->validParams, PMIC_CFG_WDG_WIN1DURATION_VALID)) {
        Pmic_criticalSectionStart(handle);
        status = Pmic_commIntf_recvByte(handle, PMIC_WD_WIN1_CFG_REG, &regVal);
        Pmic_criticalSectionStop(handle);

        if (PMIC_ST_SUCCESS == status) {
            regVal = Pmic_getBitField(regVal, PMIC_WD_WIN1_SHIFT, PMIC_WD_WIN1_MASK);
            config->win1Duration_us = ((uint32_t)regVal + 1U) * PMIC_WD_WIN1_2_US_DIV;
        }
    }

    return status;
}

static int32_t WDG_getWindow2TimeInterval(Pmic_CoreHandle_t *handle, Pmic_WdgCfg_t *config) {
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regVal = 0U;

    if (Pmic_validParamCheck(config->validParams, PMIC_CFG_WDG_WIN2DURATION_VALID)) {
        Pmic_criticalSectionStart(handle);
        status = Pmic_commIntf_recvByte(handle, PMIC_WD_WIN2_CFG_REG, &regVal);
        Pmic_criticalSectionStop(handle);

        if (PMIC_ST_SUCCESS == status) {
            regVal = Pmic_getBitField(regVal, PMIC_WD_WIN2_SHIFT, PMIC_WD_WIN2_MASK);
            config->win2Duration_us = ((uint32_t)regVal + 1U) * PMIC_WD_WIN1_2_US_DIV;
        }
    }

    return status;
}

static int32_t WDG_getWindowsTimeIntervals(Pmic_CoreHandle_t *handle, Pmic_WdgCfg_t *config) {
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

static int32_t WDG_setThresholds(Pmic_CoreHandle_t *handle, const Pmic_WdgCfg_t *config) {
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regVal = 0U;

    // Read WD_TH_CFG register
    Pmic_criticalSectionStart(handle);
    status = Pmic_commIntf_recvByte(handle, PMIC_WD_TH_CFG_REG, &regVal);
    Pmic_criticalSectionStop(handle);

    /* Set wdg threshold-2 value */
    if (Pmic_validParamCheck(config->validParams, PMIC_CFG_WDG_THRESHOLD_1_VALID)) {
        if (config->threshold1 > PMIC_WDG_THRESHOLD_COUNT_MAX) {
            status = PMIC_ST_ERR_INV_PARAM;
        }

        Pmic_setBitField(&regVal, PMIC_WD_TH1_SHIFT, PMIC_WD_TH1_MASK, config->threshold1);
    }

    /* Set wdg threshold-2 value */
    if (Pmic_validParamCheck(config->validParams, PMIC_CFG_WDG_THRESHOLD_1_VALID)) {
        if (config->threshold2 > PMIC_WDG_THRESHOLD_COUNT_MAX) {
            status = PMIC_ST_ERR_INV_PARAM;
        }

        Pmic_setBitField(&regVal, PMIC_WD_TH2_SHIFT, PMIC_WD_TH2_MASK, config->threshold2);
    }

    // Write back modified WD_TH_CFG if all modifications were successful
    if (status == PMIC_ST_SUCCESS) {
        Pmic_criticalSectionStart(handle);
        status = Pmic_commIntf_sendByte(handle, PMIC_WD_TH_CFG_REG, regVal);
        Pmic_criticalSectionStop(handle);
    }

    return status;
}

static int32_t WDG_getThresholds(Pmic_CoreHandle_t *handle, Pmic_WdgCfg_t *config) {
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regVal = 0U;

    Pmic_criticalSectionStart(handle);
    status = Pmic_commIntf_recvByte(handle, PMIC_WD_TH_CFG_REG, &regVal);
    Pmic_criticalSectionStop(handle);

    /* Get wdg threshold-1 value */
    if (Pmic_validParamStatusCheck(config->validParams, PMIC_CFG_WDG_THRESHOLD_1_VALID, status)) {
        config->threshold1 = Pmic_getBitField(regVal, PMIC_WD_TH1_SHIFT, PMIC_WD_TH1_MASK);
    }

    /* Get wdg threshold-2 value */
    if (Pmic_validParamStatusCheck(config->validParams, PMIC_CFG_WDG_THRESHOLD_2_VALID, status)) {
        config->threshold2 = Pmic_getBitField(regVal, PMIC_WD_TH2_SHIFT, PMIC_WD_TH2_MASK);
    }

    return status;
}

static int32_t WDG_setCfgParams(Pmic_CoreHandle_t *handle, const Pmic_WdgCfg_t *config) {
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regVal = 0U;

    // Validate all parameters before changing anything.
    //
    // pwrHold and returnLongWin are treated as boolean values, i.e. 0 means
    // off and anything else means 1, so no validation is required.
    if ((config->mode > PMIC_WDG_MODE_MAX) || (config->timeBase > PMIC_WDG_TIME_BASE_MAX))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    // Read WD_CFG data
    if (status == PMIC_ST_SUCCESS) {
        Pmic_criticalSectionStart(handle);
        status = Pmic_commIntf_recvByte(handle, PMIC_WD_CFG_REG, &regVal);
        Pmic_criticalSectionStop(handle);
    }

    if (Pmic_validParamCheck(config->validParams, PMIC_CFG_WDG_MODE_VALID)) {
        Pmic_setBitField(&regVal, PMIC_WD_MODE_SHIFT, PMIC_WD_MODE_MASK, config->mode);
    }

    if (Pmic_validParamCheck(config->validParams, PMIC_CFG_WDG_TIME_BASE_VALID)) {
        Pmic_setBitField(&regVal, PMIC_WD_TIME_CFG_SHIFT, PMIC_WD_TIME_CFG_MASK, config->timeBase);
    }

    // Write modified WD_CFG data back
    if (status == PMIC_ST_SUCCESS) {
        Pmic_criticalSectionStart(handle);
        status = Pmic_commIntf_sendByte(handle, PMIC_WD_CFG_REG, regVal);
        Pmic_criticalSectionStop(handle);
    }

    return status;
}

/*!
 * \brief  Function to get watchdog control parameters
 */
static int32_t WDG_getCfgParams(Pmic_CoreHandle_t *handle, Pmic_WdgCfg_t *config) {
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regVal = 0U;

    /* Read from register while in a critical section */
    Pmic_criticalSectionStart(handle);
    status = Pmic_commIntf_recvByte(handle, PMIC_WD_CFG_REG, &regVal);
    Pmic_criticalSectionStop(handle);

    /* Extract all relevant bitfields from register */
    if (Pmic_validParamStatusCheck(config->validParams, PMIC_CFG_WDG_MODE_VALID, status)) {
        config->mode = Pmic_getBitField(regVal, PMIC_WD_MODE_SHIFT, PMIC_WD_MODE_MASK);
    }

    if (Pmic_validParamStatusCheck(config->validParams, PMIC_CFG_WDG_TIME_BASE_VALID, status)) {
        config->timeBase = Pmic_getBitField(regVal, PMIC_WD_TIME_CFG_SHIFT, PMIC_WD_TIME_CFG_MASK);
    }

    return status;
}

/*! \brief  Function to set watchdog QA configurations */
static int32_t WDG_setQAConfigurations(Pmic_CoreHandle_t *handle, const Pmic_WdgCfg_t *config) {
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regVal = 0U;

    // Read the WD_QA_CFG register
    Pmic_criticalSectionStart(handle);
    status = Pmic_commIntf_recvByte(handle, PMIC_WD_QA_CFG_REG, &regVal);
    Pmic_criticalSectionStop(handle);

    /* Set wdg QA Feedback value */
    if (Pmic_validParamCheck(config->validParams, PMIC_CFG_WDG_QA_FDBK_VALID)) {
        if (config->qaFdbk > PMIC_WDG_QA_FEEDBACK_VALUE_MAX) {
            status = PMIC_ST_ERR_INV_PARAM;
        }

        Pmic_setBitField(&regVal, PMIC_WD_QA_FDBK_SHIFT, PMIC_WD_QA_FDBK_MASK, config->qaFdbk);
    }

    /* Set wdg QA LFSR value */
    if (Pmic_validParamCheck(config->validParams, PMIC_CFG_WDG_QA_LFSR_VALID)) {
        if (config->qaLfsr > PMIC_WDG_QA_LFSR_VALUE_MAX) {
            status = PMIC_ST_ERR_INV_PARAM;
        }

        Pmic_setBitField(&regVal, PMIC_WD_QA_LFSR_SHIFT, PMIC_WD_QA_LFSR_MASK, config->qaLfsr);
    }

    /* Set wdg QA Question Seed value */
    if (Pmic_validParamCheck(config->validParams, PMIC_CFG_WDG_QA_QUES_SEED_VALID)) {
        if (config->qaQuesSeed > PMIC_WDG_QA_QUES_SEED_VALUE_MAX) {
            status = PMIC_ST_ERR_INV_PARAM;
        }

        Pmic_setBitField(&regVal, PMIC_WD_QA_SEED_SHIFT, PMIC_WD_QA_SEED_MASK, config->qaQuesSeed);
    }

    // Write back the modified WD_QA_CFG register if all modifications were successful
    if (status == PMIC_ST_SUCCESS) {
        Pmic_criticalSectionStart(handle);
        status = Pmic_commIntf_sendByte(handle, PMIC_WD_QA_CFG_REG, regVal);
        Pmic_criticalSectionStop(handle);
    }

    return status;
}

static int32_t WDG_getQAConfigurations(Pmic_CoreHandle_t *handle, Pmic_WdgCfg_t *config) {
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regVal = 0U;

    Pmic_criticalSectionStart(handle);
    status = Pmic_commIntf_recvByte(handle, PMIC_WD_QA_CFG_REG, &regVal);
    Pmic_criticalSectionStop(handle);

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

/*!
 * \brief  Function to get watchdog QA answer count and question value
 */
static int32_t WDG_getQuestionAndAnswer(Pmic_CoreHandle_t *handle, uint8_t *ansCnt, uint8_t *quesVal) {
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regVal = 0U;

    /* Reading answer count and question value */
    Pmic_criticalSectionStart(handle);
    status = Pmic_commIntf_recvByte(handle, PMIC_WD_QA_CNT_REG, &regVal);
    Pmic_criticalSectionStop(handle);

    /* Extract bits from register */
    if (status == PMIC_ST_SUCCESS) {
        *ansCnt = Pmic_getBitField(regVal, PMIC_WD_ANSW_CNT_SHIFT, PMIC_WD_ANSW_CNT_MASK);
        *quesVal = Pmic_getBitField(regVal, PMIC_WD_QUESTION_SHIFT, PMIC_WD_QUESTION_MASK);
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

/*!
 * \brief  Function to Evaluate and write Watchdog Answer based on
 *         qaFdbk, qaAnsCnt and qaQuesCnt Value
 */
static int32_t WDG_qaEvalAndWriteAnswer(
    Pmic_CoreHandle_t *handle,
    uint8_t qaAnsCnt,
    uint8_t qaQuesCnt,
    uint8_t qaFbk)
{
    int32_t status = PMIC_ST_SUCCESS;
    const uint8_t answer = WDG_evalAnswerByte(qaQuesCnt, qaAnsCnt, qaFbk);

    /* Write watchdog Answers byte */
    Pmic_criticalSectionStart(handle);
    status = Pmic_commIntf_sendByte(handle, PMIC_WD_ANSWER_REG_REG, answer);
    Pmic_criticalSectionStop(handle);

    return status;
}

/* ========================================================================== */
/*                        Interface Implementations                           */
/* ========================================================================== */
int32_t Pmic_wdgEnable(Pmic_CoreHandle_t *handle) {
    return Pmic_wdgSetEnableState(handle, true);
}

int32_t Pmic_wdgDisable(Pmic_CoreHandle_t *handle) {
    return Pmic_wdgSetEnableState(handle, false);
}

int32_t Pmic_wdgSetEnableState(Pmic_CoreHandle_t *handle, bool enable) {
    int32_t status = WDG_validatePmicCoreHandle(handle);
    uint8_t regVal = 0U;

    Pmic_criticalSectionStart(handle);

    status = Pmic_commIntf_recvByte(handle, PMIC_WD_CFG_REG, &regVal);

    if (status == PMIC_ST_SUCCESS) {
        Pmic_setBitField_b(&regVal, PMIC_WD_EN_SHIFT, PMIC_WD_EN_MASK, enable);
        status = Pmic_commIntf_sendByte(handle, PMIC_WD_TH_CFG_REG, regVal);
    }

    Pmic_criticalSectionStop(handle);

    return status;
}

int32_t Pmic_wdgGetEnableState(Pmic_CoreHandle_t *handle, bool *isEnabled) {
    int32_t status = WDG_validatePmicCoreHandle(handle);
    uint8_t regData = 0;

    // Validate parameters
    if ((status == PMIC_ST_SUCCESS) && (isEnabled == NULL)) {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    Pmic_criticalSectionStart(handle);

    if (status == PMIC_ST_SUCCESS) {
        status = Pmic_commIntf_recvByte(handle, PMIC_WD_CFG_REG, &regData);
    }

    if (status == PMIC_ST_SUCCESS) {
        *isEnabled = Pmic_getBitField_b(regData, PMIC_WD_EN_SHIFT, PMIC_WD_EN_MASK);
    }

    Pmic_criticalSectionStop(handle);

    return status;
}

int32_t Pmic_wdgSetCfg(Pmic_CoreHandle_t *handle, const Pmic_WdgCfg_t *config) {
    int32_t status = WDG_validatePmicCoreHandle(handle);

    if (status == PMIC_ST_SUCCESS) {
        status = WDG_setWindowsTimeIntervals(handle, config);
    }

    if (status == PMIC_ST_SUCCESS) {
        status = WDG_setThresholds(handle, config);
    }

    if (status == PMIC_ST_SUCCESS) {
        status = WDG_setCfgParams(handle, config);
    }

    if (status == PMIC_ST_SUCCESS) {
        status = WDG_setQAConfigurations(handle, config);
    }

    return status;
}

int32_t Pmic_wdgGetCfg(Pmic_CoreHandle_t *handle, Pmic_WdgCfg_t *config) {
    int32_t status = WDG_validatePmicCoreHandle(handle);

    if ((status == PMIC_ST_SUCCESS) && (config == NULL)) {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if (status == PMIC_ST_SUCCESS) {
        status = WDG_getWindowsTimeIntervals(handle, config);
    }

    if (status == PMIC_ST_SUCCESS) {
        status = WDG_getThresholds(handle, config);
    }

    if (status == PMIC_ST_SUCCESS) {
        status = WDG_getCfgParams(handle, config);
    }

    if (status == PMIC_ST_SUCCESS) {
        status = WDG_getQAConfigurations(handle, config);
    }

    return status;
}

int32_t Pmic_wdgSetMode(Pmic_CoreHandle_t *handle, uint8_t mode) {
    int32_t status = WDG_validatePmicCoreHandle(handle);
    uint8_t regVal = 0x0U;

    /* Validate input parameter */
    if ((status == PMIC_ST_SUCCESS) && (mode > PMIC_WDG_MODE_MAX)) {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    if (status == PMIC_ST_SUCCESS) {
        /* Start Critical Section */
        Pmic_criticalSectionStart(handle);

        /* Reading watchdog mode value */
        status = Pmic_commIntf_recvByte(handle, PMIC_WD_CFG_REG, &regVal);

        /* Set watchdog mode accordingly */
        if (status == PMIC_ST_SUCCESS) {
            Pmic_setBitField(&regVal, PMIC_WD_MODE_SHIFT, PMIC_WD_MODE_MASK, mode);
            status = Pmic_commIntf_sendByte(handle, PMIC_WD_CFG_REG, regVal);
        }

        /* Stop Critical Section */
        Pmic_criticalSectionStop(handle);
    }

    return status;
}

int32_t Pmic_wdgGetMode(Pmic_CoreHandle_t *handle, uint8_t *mode) {
    int32_t status = WDG_validatePmicCoreHandle(handle);
    uint8_t regVal = 0x0U;

    /* Validate input parameter */
    if ((status == PMIC_ST_SUCCESS) && (mode == NULL)) {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    /* Reading watchdog mode value with critical section */
    if (status == PMIC_ST_SUCCESS) {
        Pmic_criticalSectionStart(handle);
        status = Pmic_commIntf_recvByte(handle, PMIC_WD_CFG_REG, &regVal);
        Pmic_criticalSectionStop(handle);

        /* Set the mode parameter accordingly */
        *mode = Pmic_getBitField(regVal, PMIC_WD_MODE_SHIFT, PMIC_WD_MODE_MASK);
    }

    return status;
}

int32_t Pmic_wdgSetPowerHold(Pmic_CoreHandle_t *handle, bool enable) {
    int32_t status = WDG_validatePmicCoreHandle(handle);
    uint8_t regVal = 0U;

    Pmic_criticalSectionStart(handle);

    status = Pmic_commIntf_recvByte(handle, PMIC_WD_CFG_REG, &regVal);

    if (PMIC_ST_SUCCESS == status) {
        Pmic_setBitField_b(&regVal, PMIC_WD_PWRHOLD_SHIFT, PMIC_WD_PWRHOLD_MASK, enable);
        status = Pmic_commIntf_sendByte(handle, PMIC_WD_CFG_REG, regVal);
    }

    Pmic_criticalSectionStop(handle);

    return status;
}

int32_t Pmic_wdgGetPowerHold(Pmic_CoreHandle_t *handle, bool *isEnabled) {
    int32_t status = WDG_validatePmicCoreHandle(handle);
    uint8_t regData = 0U;

    // Validate parameters
    if ((status == PMIC_ST_SUCCESS) && (isEnabled == NULL)) {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    Pmic_criticalSectionStart(handle);

    if (status == PMIC_ST_SUCCESS) {
        status = Pmic_commIntf_recvByte(handle, PMIC_WD_CFG_REG, &regData);
    }

    if (status == PMIC_ST_SUCCESS) {
        *isEnabled = Pmic_getBitField_b(regData, PMIC_WD_PWRHOLD_SHIFT, PMIC_WD_PWRHOLD_MASK);
    }

    Pmic_criticalSectionStop(handle);

    return status;
}

int32_t Pmic_wdgSetReturnToLongWindow(Pmic_CoreHandle_t *handle, bool enable) {
    int32_t status = WDG_validatePmicCoreHandle(handle);
    uint8_t regVal = 0U;

    Pmic_criticalSectionStart(handle);

    if (status == PMIC_ST_SUCCESS) {
        status = Pmic_commIntf_recvByte(handle, PMIC_WD_CFG_REG, &regVal);
    }

    if (status == PMIC_ST_SUCCESS) {
        Pmic_setBitField_b(&regVal, PMIC_WD_RETURN_LONGWIN_SHIFT, PMIC_WD_RETURN_LONGWIN_MASK, enable);
        status = Pmic_commIntf_sendByte(handle, PMIC_WD_CFG_REG, regVal);
    }

    Pmic_criticalSectionStop(handle);

    return status;
}

int32_t Pmic_wdgGetReturnToLongWindow(Pmic_CoreHandle_t *handle, bool *isEnabled) {
    int32_t status = WDG_validatePmicCoreHandle(handle);
    uint8_t regData = 0U;

    // Validate parameters
    if ((status == PMIC_ST_SUCCESS) && (isEnabled == NULL)) {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if (status == PMIC_ST_SUCCESS) {
        Pmic_criticalSectionStart(handle);
        status = Pmic_commIntf_recvByte(handle, PMIC_WD_CFG_REG, &regData);
        Pmic_criticalSectionStop(handle);
    }

    if (status == PMIC_ST_SUCCESS) {
        *isEnabled = Pmic_getBitField_b(regData, PMIC_WD_RETURN_LONGWIN_SHIFT, PMIC_WD_RETURN_LONGWIN_MASK);
    }

    return status;
}

int32_t Pmic_wdgGetErrorStatus(Pmic_CoreHandle_t *handle, Pmic_WdgError_t *errors) {
    int32_t status = WDG_validatePmicCoreHandle(handle);
    uint8_t regVal = 0x0U;

    if ((status == PMIC_ST_SUCCESS) && (errors == NULL)) {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    /* Reading error status register */
    if (status == PMIC_ST_SUCCESS) {
        Pmic_criticalSectionStart(handle);
        status = Pmic_commIntf_recvByte(handle, PMIC_WD_ERR_STAT_REG, &regVal);
        Pmic_criticalSectionStop(handle);
    }

    /* Extract watchdog error status fields */
    if (status == PMIC_ST_SUCCESS) {
        if (Pmic_validParamCheck(errors->validParams, PMIC_CFG_WD_LONGWIN_TIMEOUT_ERR_VALID)) {
            errors->longWindowTimeout = Pmic_getBitField_b(regVal,
                PMIC_WD_LONGWIN_TMO_SHIFT,
                PMIC_WD_LONGWIN_TMO_MASK);
        }

        if (Pmic_validParamCheck(errors->validParams, PMIC_CFG_WD_TIMEOUT_ERR_VALID)) {
            errors->timeout = Pmic_getBitField_b(regVal,
                PMIC_WD_TMO_SHIFT,
                PMIC_WD_TMO_MASK);
        }

        if (Pmic_validParamCheck(errors->validParams, PMIC_CFG_WD_TRIG_EARLY_ERR_VALID)) {
            errors->triggerEarlyError = Pmic_getBitField_b(regVal,
                PMIC_WD_TRIG_EARLY_SHIFT,
                PMIC_WD_TRIG_EARLY_MASK);
        }

        if (Pmic_validParamCheck(errors->validParams, PMIC_CFG_WD_ANSW_EARLY_ERR_VALID)) {
            errors->answerEarlyError = Pmic_getBitField_b(regVal,
                PMIC_WD_ANSW_EARLY_SHIFT,
                PMIC_WD_ANSW_EARLY_MASK);
        }

        if (Pmic_validParamCheck(errors->validParams, PMIC_CFG_WD_SEQ_ERR_ERR_VALID)) {
            errors->sequenceError = Pmic_getBitField_b(regVal,
                PMIC_WD_SEQ_ERR_SHIFT,
                PMIC_WD_SEQ_ERR_MASK);
        }

        if (Pmic_validParamCheck(errors->validParams, PMIC_CFG_WD_ANSW_ERR_ERR_VALID)) {
            errors->answerError = Pmic_getBitField_b(regVal,
                PMIC_WD_ANSW_ERR_SHIFT,
                PMIC_WD_ANSW_ERR_MASK);
        }

        if (Pmic_validParamCheck(errors->validParams, PMIC_CFG_WD_TH1_INT_ERR_VALID)) {
            errors->threshold1Error = Pmic_getBitField_b(regVal,
                PMIC_WD_TH1_ERR_SHIFT,
                PMIC_WD_TH1_ERR_MASK);
        }

        if (Pmic_validParamCheck(errors -> validParams, PMIC_CFG_WD_TH2_INT_ERR_VALID)) {
            errors->threshold2Error = Pmic_getBitField_b(regVal,
                PMIC_WD_TH2_ERR_SHIFT,
                PMIC_WD_TH2_ERR_MASK);
        }
    }

    return status;
}

int32_t Pmic_wdgClrErrStatus(Pmic_CoreHandle_t *handle, const Pmic_WdgError_t *errors) {
    int32_t status = WDG_validatePmicCoreHandle(handle);
    uint8_t regVal = 0x0U;

    if (status == PMIC_ST_SUCCESS) {
        if (Pmic_validParamCheck(errors->validParams, PMIC_CFG_WD_LONGWIN_TIMEOUT_ERR_VALID)) {
            Pmic_setBitField_b(&regVal,
                PMIC_WD_LONGWIN_TMO_SHIFT,
                PMIC_WD_LONGWIN_TMO_MASK,
                errors->longWindowTimeout);
        }

        if (Pmic_validParamCheck(errors->validParams, PMIC_CFG_WD_TIMEOUT_ERR_VALID)) {
            Pmic_setBitField_b(&regVal, PMIC_WD_TMO_SHIFT, PMIC_WD_TMO_MASK, errors->timeout);
        }

        if (Pmic_validParamCheck(errors->validParams, PMIC_CFG_WD_TRIG_EARLY_ERR_VALID)) {
            Pmic_setBitField_b(&regVal,
                PMIC_WD_TRIG_EARLY_SHIFT,
                PMIC_WD_TRIG_EARLY_MASK,
                errors->triggerEarlyError);
        }

        if (Pmic_validParamCheck(errors->validParams, PMIC_CFG_WD_ANSW_EARLY_ERR_VALID)) {
            Pmic_setBitField_b(&regVal,
                PMIC_WD_ANSW_EARLY_SHIFT,
                PMIC_WD_ANSW_EARLY_MASK,
                errors->answerEarlyError);
        }

        if (Pmic_validParamCheck(errors->validParams, PMIC_CFG_WD_SEQ_ERR_ERR_VALID)) {
            Pmic_setBitField_b(&regVal, PMIC_WD_SEQ_ERR_SHIFT, PMIC_WD_SEQ_ERR_MASK, errors->sequenceError);
        }

        if (Pmic_validParamCheck(errors->validParams, PMIC_CFG_WD_ANSW_ERR_ERR_VALID)) {
            Pmic_setBitField_b(&regVal, PMIC_WD_ANSW_ERR_SHIFT, PMIC_WD_ANSW_ERR_MASK, errors->answerError);
        }

        if (Pmic_validParamCheck(errors->validParams, PMIC_CFG_WD_TH1_INT_ERR_VALID)) {
            Pmic_setBitField_b(&regVal, PMIC_WD_TH1_ERR_SHIFT, PMIC_WD_TH1_ERR_MASK, errors->threshold1Error);
        }

        if (Pmic_validParamCheck(errors->validParams, PMIC_CFG_WD_TH2_INT_ERR_VALID)) {
            Pmic_setBitField_b(&regVal, PMIC_WD_TH2_ERR_SHIFT, PMIC_WD_TH2_ERR_MASK, errors->threshold2Error);
        }
    }

    // WD_ERR_STAT register is write 1 to clear, write composed data back to register (if any bits are set).
    if ((status == PMIC_ST_SUCCESS) && (regVal != 0U)) {
        Pmic_criticalSectionStart(handle);
        status = Pmic_commIntf_sendByte(handle, PMIC_WD_ERR_STAT_REG, regVal);
        Pmic_criticalSectionStop(handle);
    }

    return status;
}

int32_t Pmic_wdgClrErrStatusAll(Pmic_CoreHandle_t *handle) {
    int32_t status = WDG_validatePmicCoreHandle(handle);

    // WD_ERR_STAT register is write 1 to clear, write all bits as 1 to clear.
    if (status == PMIC_ST_SUCCESS) {
        Pmic_criticalSectionStart(handle);
        status = Pmic_commIntf_sendByte(handle, PMIC_WD_ERR_STAT_REG, 0xFFU);
        Pmic_criticalSectionStop(handle);
    }

    return status;
}

int32_t Pmic_wdgGetFailCntStat(Pmic_CoreHandle_t *handle, Pmic_WdgFailCntStat_t *failCount) {
    int32_t status = WDG_validatePmicCoreHandle(handle);
    uint8_t regVal = 0x00U;

    if ((status == PMIC_ST_SUCCESS) && (failCount == NULL)) {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    /* Reading error status register */
    if (status == PMIC_ST_SUCCESS) {
        Pmic_criticalSectionStart(handle);
        status = Pmic_commIntf_recvByte(handle, PMIC_WD_STAT_REG, &regVal);
        Pmic_criticalSectionStop(handle);
    }

    /* Get watchdog Bad Event status */
    if (Pmic_validParamStatusCheck(failCount->validParams, PMIC_CFG_WD_BAD_EVENT_STAT_VALID, status)) {
        failCount->badEvent = Pmic_getBitField_b(regVal, PMIC_WD_BAD_EVENT_SHIFT, PMIC_WD_BAD_EVENT_MASK);
    }

    /* Get watchdog Good Event status */
    if (Pmic_validParamStatusCheck(failCount->validParams, PMIC_CFG_WD_GOOD_EVENT_STAT_VALID, status)) {
        failCount->goodEvent = Pmic_getBitField_b(regVal, PMIC_WD_FIRST_OK_SHIFT, PMIC_WD_FIRST_OK_MASK);
    }

    /* Get watchdog Fail count Value */
    if (Pmic_validParamStatusCheck(failCount->validParams, PMIC_CFG_WD_FAIL_CNT_VAL_VALID, status)) {
        failCount->wdFailCnt = Pmic_getBitField_b(regVal, PMIC_WD_ERR_CNT_SHIFT, PMIC_WD_ERR_CNT_MASK);
    }

    return status;
}

int32_t Pmic_wdgQaSequenceWriteAnswer(Pmic_CoreHandle_t *handle) {
    int32_t status = WDG_validatePmicCoreHandle(handle);

    uint8_t qaAnsCnt = 0U;
    uint8_t qaQuesCnt = 0U;
    uint8_t qaFbk = 0U;

    /* Get wdg QA Feedback value */
    if (status == PMIC_ST_SUCCESS) {
        Pmic_criticalSectionStart(handle);
        status = Pmic_commIntf_recvByte(handle, PMIC_WD_QA_CFG_REG, &qaFbk);
        Pmic_criticalSectionStop(handle);
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

    return status;
}
