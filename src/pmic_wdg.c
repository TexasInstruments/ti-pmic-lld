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
#include "pmic_wdg.h"
#include "pmic_core_priv.h"

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */
static int32_t WDG_validatePmicCoreHandle(const Pmic_CoreHandle_t *handle) {
    int32_t status = Pmic_checkPmicCoreHandle(handle);

    /* Check the watch dog sub-system supported by pmic device */
    if (PMIC_ST_SUCCESS == status) {
        if (!handle->pPmic_SubSysInfo->wdgEnable) {
            status = PMIC_ST_ERR_INV_DEVICE;
        }
    }

    return status;
}

static uint8_t WDG_convertLongWinTimeToBits(const Pmic_WdgCfg_t *config) {
    uint8_t regVal = 0U;
    uint8_t baseVal = 0U;

    if (config->longWinDuration_ms == PMIC_WD_LONGWIN_80_MS) {
        regVal = PMIC_WD_LONGWIN_REG_VAL_0;
    } else if (config->longWinDuration_ms <= PMIC_WD_LONGWIN_8000_MS) {
        regVal = (uint8_t)(config->longWinDuration_ms / PMIC_WD_LONGWIN_MS_DIV_125);
    } else {
        baseVal = (uint8_t)(PMIC_WD_LONGWIN_8000_MS / PMIC_WD_LONGWIN_MS_DIV_125);
        regVal = baseVal + (uint8_t)((config->longWinDuration_ms - PMIC_WD_LONGWIN_8000_MS) / PMIC_WD_LONGWIN_MS_DIV_4000);
    }

    return regVal;
}

static int32_t WDG_setWin1Win2TimeIntervals(Pmic_CoreHandle_t *handle, const Pmic_WdgCfg_t *config)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regVal = 0U;
    bool winBelowMin, winAboveMax;

    /* Set wdg window1 time interval */
    if (pmic_validParamCheck(config->validParams, PMIC_CFG_WDG_WIN1DURATION_VALID)) {
        winBelowMin = (config->win1Duration_us < PMIC_WD_WIN1_2_US_MIN);
        winAboveMax = (config->win1Duration_us > PMIC_WD_WIN1_2_US_MAX);

        if (winBelowMin || winAboveMax) {
            status = PMIC_ST_ERR_INV_WDG_WINDOW;
        }

        if (PMIC_ST_SUCCESS == status) {
            regVal = (uint8_t)(((config->win1Duration_us / PMIC_WD_WIN1_2_US_DIV) - 1U) & PMIC_WD_WIN1_MASK);

            Pmic_criticalSectionStart(handle);
            status = Pmic_commIntf_sendByte(handle, PMIC_WD_WIN1_CFG_REG, regVal);
            Pmic_criticalSectionStop(handle);
        }
    }

    /* Set wdg window2 time interval */
    if ((PMIC_ST_SUCCESS == status) &&
        pmic_validParamCheck(config->validParams, PMIC_CFG_WDG_WIN2DURATION_VALID)) {
        winBelowMin = (config->win2Duration_us < PMIC_WD_WIN1_2_US_MIN);
        winAboveMax = (config->win2Duration_us > PMIC_WD_WIN1_2_US_MAX);

        if (winBelowMin || winAboveMax) {
            status = PMIC_ST_ERR_INV_WDG_WINDOW;
        }

        if (status == PMIC_ST_SUCCESS) {
            regVal = (uint8_t)(((config->win2Duration_us / PMIC_WD_WIN1_2_US_DIV) - 1U) & PMIC_WD_WIN2_MASK);

            Pmic_criticalSectionStart(handle);
            status = Pmic_commIntf_sendByte(handle, PMIC_WD_WIN2_CFG_REG, regVal);
            Pmic_criticalSectionStop(handle);
        }
    }

    return status;
}

static int32_t WDG_setWindowsTimeIntervals(Pmic_CoreHandle_t *handle, const Pmic_WdgCfg_t *config) {
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regVal = 0U;

    /* Set wdg long window time interval */
    if (pmic_validParamCheck(config->validParams, PMIC_CFG_WDG_LONGWINDURATION_VALID)) {
        const bool longWinNot80ms = (config->longWinDuration_ms != PMIC_WD_LONGWIN_80_MS);
        const bool longWinBelowMin = (config->longWinDuration_ms < PMIC_WD_LONGWIN_MS_MIN_PG_2_0);
        const bool longWinAboveMax = (config->longWinDuration_ms > PMIC_WD_LONGWIN_MS_MAX_PG_2_0);

        if (longWinNot80ms && (longWinBelowMin || longWinAboveMax)) {
            status = PMIC_ST_ERR_INV_WDG_WINDOW;
        }

        if (status == PMIC_ST_SUCCESS) {
            regVal = WDG_convertLongWinTimeToBits(config);
        }

        if (status == PMIC_ST_SUCCESS) {
            Pmic_criticalSectionStart(handle);
            status = Pmic_commIntf_sendByte(handle, PMIC_WD_LONGWIN_CFG_REG, regVal);
            Pmic_criticalSectionStop(handle);
        }
    }

    /* Set wdg window1 and window2 time interval */
    if (status == PMIC_ST_SUCCESS) {
        status = WDG_setWin1Win2TimeIntervals(handle, config);
    }

    return status;
}

static int32_t
Pmic_WdgGetWindow1Window2TimeIntervals(Pmic_CoreHandle_t * handle,
    Pmic_WdgCfg_t * config) {
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regVal = 0U;

    /* Get wdg window1 time interval */
    if (true == pmic_validParamCheck(config -> validParams,
            PMIC_CFG_WDG_WIN1DURATION_VALID)) {
        Pmic_criticalSectionStart(handle);

        status = Pmic_commIntf_recvByte(handle, PMIC_WD_WIN1_CFG_REG, &
            regVal);

        Pmic_criticalSectionStop(handle);

        if (PMIC_ST_SUCCESS == status) {
            regVal &= (uint8_t) PMIC_WD_WIN1_MASK;

            config -> win1Duration_us =
                ((uint32_t) regVal + 1U) * (uint32_t)PMIC_WD_WIN1_2_US_DIV;
        }
    }

    /* Get wdg window2 time interval */
    if ((PMIC_ST_SUCCESS == status) &&
        (true == pmic_validParamCheck(config -> validParams,
            PMIC_CFG_WDG_WIN2DURATION_VALID))) {
        Pmic_criticalSectionStart(handle);

        status = Pmic_commIntf_recvByte(handle, PMIC_WD_WIN2_CFG_REG, &
            regVal);

        Pmic_criticalSectionStop(handle);

        if (PMIC_ST_SUCCESS == status) {
            regVal &= (uint8_t) PMIC_WD_WIN2_MASK;

            config -> win2Duration_us =
                ((uint32_t) regVal + 1U) * (uint32_t)PMIC_WD_WIN1_2_US_DIV;
        }
    }

    return status;
}

static int32_t
Pmic_WdgGetWindowsTimeIntervals(Pmic_CoreHandle_t * handle,
    Pmic_WdgCfg_t * config) {
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regVal = 0U;

    /* Get wdg long window time interval */
    if (true == pmic_validParamCheck(config -> validParams,
            PMIC_CFG_WDG_LONGWINDURATION_VALID)) {
        Pmic_criticalSectionStart(handle);

        status = Pmic_commIntf_recvByte(handle,
            PMIC_WD_LONGWIN_CFG_REG, & regVal);

        Pmic_criticalSectionStop(handle);

        if (PMIC_ST_SUCCESS == status) {
            if (((handle -> pmicDeviceType == PMIC_DEV_HERA_LP8764X) ||
                    (handle -> pmicDeviceType == PMIC_DEV_LEO_TPS6594X)) &&
                (PMIC_SILICON_REV_ID_PG_1_0 == handle -> pmicDevSiliconRev)) {
                if (PMIC_WD_LONGWIN_REG_VAL_0 == regVal) {
                    config -> longWinDuration_ms = PMIC_WD_LONGWIN_100_MS;
                } else {
                    config -> longWinDuration_ms =
                        (uint32_t) regVal * PMIC_WD_LONGWIN_MS_DIV;
                }
            } else {
                if (PMIC_WD_LONGWIN_REG_VAL_0 == regVal) {
                    config -> longWinDuration_ms = PMIC_WD_LONGWIN_80_MS;
                } else if (regVal <= PMIC_WD_LONGWIN_REG_VAL_64) {
                    config -> longWinDuration_ms =
                        (uint32_t) regVal * PMIC_WD_LONGWIN_MS_DIV_125;
                } else {
                    config -> longWinDuration_ms =
                        regVal - ((PMIC_WD_LONGWIN_REG_VAL_64 *
                                PMIC_WD_LONGWIN_MS_DIV_4000) +
                            PMIC_WD_LONGWIN_8000_MS);
                }
            }
        }
    }

    if (PMIC_ST_SUCCESS == status) {
        /* Get wdg window1 and window2 time interval */
        status = Pmic_WdgGetWindow1Window2TimeIntervals(handle, config);
    }

    return status;
}

static int32_t WDG_setThresholdValues(Pmic_CoreHandle_t *handle, const Pmic_WdgCfg_t *config) {
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regVal = 0U;

    /* Set wdg threshold-2 value */
    if (pmic_validParamCheck(config->validParams, PMIC_CFG_WDG_THRESHOLD_1_VALID)) {
        if (config->threshold1 > PMIC_WDG_THRESHOLD_COUNT_MAX) {
            status = PMIC_ST_ERR_INV_PARAM;
        }

        Pmic_criticalSectionStart(handle);

        if (PMIC_ST_SUCCESS == status) {
            status = Pmic_commIntf_recvByte(handle, PMIC_WD_TH_CFG_REG, &regVal);
        }

        if (PMIC_ST_SUCCESS == status) {
            Pmic_setBitField(&regVal, PMIC_WD_TH1_SHIFT, PMIC_WD_TH1_MASK, config->threshold1);
            status = Pmic_commIntf_sendByte(handle, PMIC_WD_TH_CFG_REG, regVal);
        }

        Pmic_criticalSectionStop(handle);
    }

    /* Set wdg threshold-2 value */
    if ((PMIC_ST_SUCCESS == status) &&
        pmic_validParamCheck(config->validParams, PMIC_CFG_WDG_THRESHOLD_1_VALID)) {
        if (config->threshold2 > PMIC_WDG_THRESHOLD_COUNT_MAX) {
            status = PMIC_ST_ERR_INV_PARAM;
        }

        Pmic_criticalSectionStart(handle);

        if (PMIC_ST_SUCCESS == status) {
            status = Pmic_commIntf_recvByte(handle, PMIC_WD_TH_CFG_REG, &regVal);
        }

        if (PMIC_ST_SUCCESS == status) {
            Pmic_setBitField(&regVal, PMIC_WD_TH2_SHIFT, PMIC_WD_TH2_MASK, config->threshold2);
            status = Pmic_commIntf_sendByte(handle, PMIC_WD_TH_CFG_REG, regVal);
        }

        Pmic_criticalSectionStop(handle);
    }

    return status;
}

static int32_t Pmic_WdgGetThresholdValues(Pmic_CoreHandle_t *handle, Pmic_WdgCfg_t *config) {
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regVal = 0U;

    Pmic_criticalSectionStart(handle);
    status = Pmic_commIntf_recvByte(handle, PMIC_WD_TH_CFG_REG, &regVal);
    Pmic_criticalSectionStop(handle);

    /* Get wdg threshold-2 value */
    if ((PMIC_ST_SUCCESS == status) &&
        pmic_validParamCheck(config->validParams, PMIC_CFG_WDG_THRESHOLD_1_VALID)) {
        config->threshold1 = Pmic_getBitField(regVal, PMIC_WD_TH1_SHIFT, PMIC_WD_TH1_MASK);
    }

    /* Get wdg threshold-2 value */
    if ((PMIC_ST_SUCCESS == status) &&
        pmic_validParamCheck(config->validParams, PMIC_CFG_WDG_THRESHOLD_2_VALID)) {
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

    if (pmic_validParamStatusCheck(config->validParams, PMIC_CFG_WDG_MODE_VALID, status)) {
        Pmic_setBitField(&regVal, PMIC_WD_MODE_SHIFT, PMIC_WD_MODE_MASK, config->mode);
    }

    if (pmic_validParamStatusCheck(config->validParams, PMIC_CFG_WDG_TIME_BASE_VALID, status)) {
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
static int32_t Pmic_WdgGetCtrlParams(Pmic_CoreHandle_t *handle, Pmic_WdgCfg_t *config) {
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regVal = 0U;

    /* Read from register while in a critical section */
    Pmic_criticalSectionStart(handle);
    status = Pmic_commIntf_recvByte(handle, PMIC_WD_CFG_REG, &regVal);
    Pmic_criticalSectionStop(handle);

    /* Extract all relevant bitfields from register */
    if ((PMIC_ST_SUCCESS == status) &&
        pmic_validParamCheck(config->validParams, PMIC_CFG_WDG_MODE_VALID)) {
        config->mode = Pmic_getBitField(regVal, PMIC_WD_MODE_SHIFT, PMIC_WD_MODE_MASK);
    }

    if ((PMIC_ST_SUCCESS == status) &&
        pmic_validParamCheck(config->validParams, PMIC_CFG_WDG_TIME_BASE_VALID)) {
        config->timeBase = Pmic_getBitField(regVal, PMIC_WD_TIME_CFG_SHIFT, PMIC_WD_TIME_CFG_MASK);
    }

    return status;
}

/*!
 * \brief  Function to set watchdog QA Question Seed value
 */
static int32_t Pmic_wdgSetQaQuesSeedValue(Pmic_CoreHandle_t *handle, const Pmic_WdgCfg_t *config) {
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regVal = 0U;

    /* Set wdg QA Question Seed value */
    if (true == pmic_validParamCheck(config->validParams, PMIC_CFG_WDG_QA_QUES_SEED_VALID)) {
        if (config->qaQuesSeed > PMIC_WDG_QA_QUES_SEED_VALUE_15) {
            status = PMIC_ST_ERR_INV_PARAM;
        }

        Pmic_criticalSectionStart(handle);

        if (PMIC_ST_SUCCESS == status) {
            status = Pmic_commIntf_recvByte(handle, PMIC_WD_QA_CFG_REG, &
                regVal);
        }

        if (PMIC_ST_SUCCESS == status) {
            Pmic_setBitField( &regVal, PMIC_WD_QA_CFG_WD_QA_SEED_SHIFT,
                              (uint8_t)PMIC_WD_QA_CFG_WD_QA_SEED_MASK, config->qaQuesSeed);

            status = Pmic_commIntf_sendByte(handle, PMIC_WD_QA_CFG_REG,
                regVal);
        }

        Pmic_criticalSectionStop(handle);
    }

    return status;
}

/*!
 * \brief  Function to set watchdog QA configurations
 */
static int32_t Pmic_WdgSetQaConfigurations(Pmic_CoreHandle_t * handle, const Pmic_WdgCfg_t *config) {
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regVal = 0U;

    /* Set wdg QA Feedback value */
    if (true ==
        pmic_validParamCheck(config->validParams, PMIC_CFG_WDG_QA_FDBK_VALID)) {
        if (config->qaFdbk > PMIC_WDG_QA_FEEDBACK_VALUE_3) {
            status = PMIC_ST_ERR_INV_PARAM;
        }

        Pmic_criticalSectionStart(handle);

        if (PMIC_ST_SUCCESS == status) {
            status = Pmic_commIntf_recvByte(handle, PMIC_WD_QA_CFG_REG, &
                regVal);
        }
        if (PMIC_ST_SUCCESS == status) {
            Pmic_setBitField( & regVal, PMIC_WD_QA_CFG_WD_QA_FDBK_SHIFT,
                              (uint8_t)PMIC_WD_QA_CFG_WD_QA_FDBK_MASK, config->qaFdbk);

            status = Pmic_commIntf_sendByte(handle, PMIC_WD_QA_CFG_REG,
                regVal);
        }

        Pmic_criticalSectionStop(handle);
    }

    /* Set wdg QA LFSR value */
    if ((PMIC_ST_SUCCESS == status) &&
        (true ==
            pmic_validParamCheck(config->validParams, PMIC_CFG_WDG_QA_LFSR_VALID))) {
        if (config->qaLfsr > PMIC_WDG_QA_LFSR_VALUE_3) {
            status = PMIC_ST_ERR_INV_PARAM;
        }

        Pmic_criticalSectionStart(handle);

        if (PMIC_ST_SUCCESS == status) {
            status = Pmic_commIntf_recvByte(handle, PMIC_WD_QA_CFG_REG, &
                regVal);
        }

        if (PMIC_ST_SUCCESS == status) {
            Pmic_setBitField( &regVal, PMIC_WD_QA_CFG_WD_QA_LFSR_SHIFT,
                              (uint8_t)PMIC_WD_QA_CFG_WD_QA_LFSR_MASK, config->qaLfsr);

            status = Pmic_commIntf_sendByte(handle, PMIC_WD_QA_CFG_REG,
                regVal);
        }

        Pmic_criticalSectionStop(handle);
    }

    if (PMIC_ST_SUCCESS == status) {
        /* Set wdg QA Question Seed value */
        status = Pmic_wdgSetQaQuesSeedValue(handle, config);
    }

    return status;
}

/*!
 * \brief  Function to get watchdog QA configurations
 */
static int32_t Pmic_WdgGetQaConfigurations(Pmic_CoreHandle_t * handle,
    Pmic_WdgCfg_t * config) {
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regVal = 0U;

    Pmic_criticalSectionStart(handle);

    status =
        Pmic_commIntf_recvByte(handle, PMIC_WD_QA_CFG_REG, & regVal);

    Pmic_criticalSectionStop(handle);

    /* Get wdg QA Feedback value */
    if ((PMIC_ST_SUCCESS == status) &&
        (true == pmic_validParamCheck(config -> validParams,
            PMIC_CFG_WDG_QA_FDBK_VALID))) {
        config -> qaFdbk = Pmic_getBitField(regVal, PMIC_WD_QA_CFG_WD_QA_FDBK_SHIFT,
                                             (uint8_t)PMIC_WD_QA_CFG_WD_QA_FDBK_MASK);
    }

    /* Get wdg QA LFSR value */
    if ((PMIC_ST_SUCCESS == status) &&
        (true == pmic_validParamCheck(config -> validParams,
            PMIC_CFG_WDG_QA_LFSR_VALID))) {
        config -> qaLfsr = Pmic_getBitField(regVal, PMIC_WD_QA_CFG_WD_QA_LFSR_SHIFT,
                                             (uint8_t)PMIC_WD_QA_CFG_WD_QA_LFSR_MASK);
    }

    /* Get wdg QA Question Seed value */
    if ((PMIC_ST_SUCCESS == status) &&
        (true == pmic_validParamCheck(config -> validParams,
            PMIC_CFG_WDG_QA_QUES_SEED_VALID))) {
        config -> qaQuesSeed =
            Pmic_getBitField(regVal, PMIC_WD_QA_CFG_WD_QA_SEED_SHIFT,
                             (uint8_t)PMIC_WD_QA_CFG_WD_QA_SEED_MASK);
    }

    return status;
}

/*!
 * \brief  Function to get watchdog QA answer count and question value
 */
static int32_t
Pmic_wdgReadQuesandAnswerCount(Pmic_CoreHandle_t * handle,
    uint8_t * pQaAnsCnt, uint8_t * pQaQuesCnt) {
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regVal = 0U;

    /*! Start Critical Section */
    Pmic_criticalSectionStart(handle);

    /*! Reading answer count and question value */
    status =
        Pmic_commIntf_recvByte(handle, PMIC_WD_QA_CNT_REG, & regVal);

    /*! Stop Critical Section */
    Pmic_criticalSectionStop(handle);

    if (PMIC_ST_SUCCESS == status) {
        * pQaAnsCnt =
            Pmic_getBitField(regVal, PMIC_WD_QA_CNT_WD_ANSW_CNT_SHIFT,
                             (uint8_t)PMIC_WD_QA_CNT_WD_ANSW_CNT_MASK);
        * pQaQuesCnt =
            Pmic_getBitField(regVal, PMIC_WD_QA_CNT_WD_QUESTION_SHIFT,
                             (uint8_t)PMIC_WD_QA_CNT_WD_QUESTION_MASK);
    }

    return status;
}

static bool is_wdgBadEventDetected(Pmic_CoreHandle_t * handle) {
    uint8_t regVal = 0U;
    bool bitFieldVal = false;
    int32_t status = PMIC_ST_SUCCESS;

    /* Start Critical Section */
    Pmic_criticalSectionStart(handle);

    status =
        Pmic_commIntf_recvByte(handle, PMIC_WD_STAT_REG, & regVal);

    /* Stop Critical Section */
    Pmic_criticalSectionStop(handle);

    if ((PMIC_ST_SUCCESS == status) &&
        (Pmic_getBitField(regVal, PMIC_WD_STAT_REG_WD_BAD_EVENT_SHIFT,
                          (uint8_t)PMIC_WD_STAT_REG_WD_BAD_EVENT_MASK) != 0U)) {
        bitFieldVal = true;
    }

    return bitFieldVal;
}

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

/*!
 * \brief  Function to Evaluate Watchdog Answers
 */
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
    qaAns = mux(q0, q1, q2, q3, qaFbk) ^ mux(q3, q2, q1, q0, qaFbk) ^ a1;
    /* Reference-Answer-X[1] */
    qaAns |= (uint8_t)((mux(q0, q1, q2, q3, qaFbk) ^ mux(q2, q1, q0, q3, qaFbk) ^ a1 ^ q1) << 1U);
    /* Reference-Answer-X[2] */
    qaAns |= (uint8_t)((mux(q0, q3, q1, q1, qaFbk) ^ mux(q3, q2, q1, q0, qaFbk) ^ a1 ^ q1) << 2U);
    /* Reference-Answer-X[3] */
    qaAns |= (uint8_t)((mux(q2, q1, q0, q3, qaFbk) ^ mux(q0, q3, q2, q1, qaFbk) ^ a1 ^ q3) << 3U);
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
    uint8_t answer = 0U;

    answer = WDG_evalAnswerByte(qaQuesCnt, qaAnsCnt, qaFbk);

    /* Start Critical Section */
    Pmic_criticalSectionStart(handle);

    /*! Writing watch dog four Answers */
    status = Pmic_commIntf_sendByte(handle, PMIC_WD_ANSWER_REG_REG, answer);

    /* Stop Critical Section */
    Pmic_criticalSectionStop(handle);

    return status;
}

/*!
 * \brief  Function to Evaluate and write Watchdog Four Answers based on
 *         qaFdbk Value
 */
static int32_t WDG_qaEvalAndWriteAnswers(Pmic_CoreHandle_t *handle, uint8_t qaFbk) {
    int32_t status = PMIC_ST_SUCCESS;
    int8_t ansIndex = 0;
    uint8_t qaAnsCnt = 0U;
    uint8_t qaQuesCnt = 0U;

    for (ansIndex = 3; ansIndex >= 0; ansIndex--) {
        if (PMIC_ST_SUCCESS == status) {
            status = Pmic_wdgReadQuesandAnswerCount(handle, &qaAnsCnt, &qaQuesCnt);
        }

        if ((PMIC_ST_SUCCESS == status) && (qaAnsCnt == (uint8_t) ansIndex)) {
            status = WDG_qaEvalAndWriteAnswer(handle, qaAnsCnt, qaQuesCnt, qaFbk);

            if ((PMIC_ST_SUCCESS == status) && is_wdgBadEventDetected(handle)) {
                status = PMIC_ST_ERR_INV_WDG_ANSWER;
                break;
            }
        }
    }

    return status;
}

/*!
 * \brief  Function to Evaluate and write Watchdog Four Answers
 */
static int32_t WDG_qaWriteAnswers(Pmic_CoreHandle_t * handle) {
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t qaFbk = 0U;

    Pmic_criticalSectionStart(handle);
    status = Pmic_commIntf_recvByte(handle, PMIC_WD_QA_CFG_REG, &qaFbk);
    Pmic_criticalSectionStop(handle);

    /* Get wdg QA Feedback value */
    if (status == PMIC_ST_SUCCESS) {
        qaFbk = Pmic_getBitField(
            qaFbk,
            PMIC_WD_QA_CFG_WD_QA_FDBK_SHIFT,
            PMIC_WD_QA_CFG_WD_QA_FDBK_MASK);
    }

    if (status == PMIC_ST_SUCCESS) {
        /* Evaluate and write Watchdog Four Answers based on qaFdbk Value*/
        status = WDG_qaEvalAndWriteAnswers(handle, qaFbk);
    }

    return status;
}

static void WDG_extractIntErrStatusField(Pmic_WdgErrStatus_t *pErrStatus, uint8_t regVal) {
    if (pmic_validParamCheck(pErrStatus->validParams, PMIC_CFG_WD_LONGWIN_TIMEOUT_ERRSTAT_VALID)) {
        const uint8_t bitVal = Pmic_getBitField(regVal,
            PMIC_WD_ERR_STAT_REG_WD_LONGWIN_TMO_SHIFT,
            PMIC_WD_ERR_STAT_REG_WD_LONGWIN_TMO_MASK);

        pErrStatus->longWindowTimeout = (bitVal != 0U);
    }

    if (pmic_validParamCheck(pErrStatus->validParams, PMIC_CFG_WD_TIMEOUT_ERRSTAT_VALID)) {
        const uint8_t bitVal = Pmic_getBitField(regVal,
            PMIC_WD_ERR_STAT_REG_WD_TMO_SHIFT,
            PMIC_WD_ERR_STAT_REG_WD_TMO_MASK);

        pErrStatus->timeout = (bitVal != 0U);
    }

    if (pmic_validParamCheck(pErrStatus->validParams, PMIC_CFG_WD_TRIG_EARLY_ERRSTAT_VALID)) {
        const uint8_t bitVal = Pmic_getBitField(regVal,
            PMIC_WD_ERR_STAT_REG_WD_TRIG_EARLY_SHIFT,
            PMIC_WD_ERR_STAT_REG_WD_TRIG_EARLY_MASK);

        pErrStatus->triggerEarlyError = (bitVal != 0U);
    }

    if (pmic_validParamCheck(pErrStatus->validParams, PMIC_CFG_WD_ANSW_EARLY_ERRSTAT_VALID)) {
        const uint8_t bitVal = Pmic_getBitField(regVal,
            PMIC_WD_ERR_STAT_REG_WD_ANSW_EARLY_SHIFT,
            PMIC_WD_ERR_STAT_REG_WD_ANSW_EARLY_MASK);

        pErrStatus->answerEarlyError = (bitVal != 0U);
    }

    if (pmic_validParamCheck(pErrStatus->validParams, PMIC_CFG_WD_SEQ_ERR_ERRSTAT_VALID)) {
        const uint8_t bitVal = Pmic_getBitField(regVal,
            PMIC_WD_ERR_STAT_REG_WD_SEQ_ERR_SHIFT,
            PMIC_WD_ERR_STAT_REG_WD_SEQ_ERR_MASK);

        pErrStatus->sequenceError = (bitVal != 0U);
    }

    if (pmic_validParamCheck(pErrStatus->validParams, PMIC_CFG_WD_ANSW_ERR_ERRSTAT_VALID)) {
        const uint8_t bitVal = Pmic_getBitField(regVal,
            PMIC_WD_ERR_STAT_REG_WD_ANSW_ERR_SHIFT,
            PMIC_WD_ERR_STAT_REG_WD_ANSW_ERR_MASK);

        pErrStatus->answerError = (bitVal != 0U);
    }

    if (pmic_validParamCheck(pErrStatus->validParams, PMIC_CFG_WD_FAIL_INT_ERRSTAT_VALID)) {
        const uint8_t bitVal = Pmic_getBitField(regVal,
            PMIC_WD_ERR_STAT_REG_WD_TH1_ERR_SHIFT,
            PMIC_WD_ERR_STAT_REG_WD_TH1_ERR_MASK);

        pErrStatus->threshold1Error = (bitVal != 0U);
    }

    if (pmic_validParamCheck(pErrStatus -> validParams, PMIC_CFG_WD_RST_INT_ERRSTAT_VALID)) {
        const uint8_t bitVal = Pmic_getBitField(regVal,
            PMIC_WD_ERR_STAT_REG_WD_TH2_ERR_SHIFT,
            PMIC_WD_ERR_STAT_REG_WD_TH2_ERR_MASK);

        pErrStatus->threshold2Error = (bitVal != 0U);
    }
}

/*!
 * \brief  API to Write QA Answers for given numbers of sequences
 */
static int32_t WDG_qaWriteAnswersNumSequence(Pmic_CoreHandle_t *handle, uint32_t sequences, uint32_t maxCnt) {
    int32_t status = PMIC_ST_SUCCESS;

    uint32_t loopCount = 0U;
    uint8_t failCnt = 0U;
    int8_t flag = 0;
    uint8_t regVal = 0x0U;
    uint32_t qaSequences = sequences;

    /* Write QA Answers for given numbers of sequences */
    while ((status == PMIC_ST_SUCCESS) && ((qaSequences == PMIC_WD_QA_INFINITE_SEQ) || (qaSequences > 0U))) {
        /*! Write Answer to WDOG for the sequence */
        status = WDG_qaWriteAnswers(handle);

        if (status == PMIC_ST_ERR_INV_WDG_ANSWER) {
            Pmic_criticalSectionStart(handle);
            status = Pmic_commIntf_recvByte(handle, PMIC_WD_ERR_STAT_REG, &regVal);
            Pmic_criticalSectionStop(handle);

            if (status == PMIC_ST_SUCCESS) {
                if ((regVal & 0x10U) != 0U) {
                    status = PMIC_ST_ERR_INV_WDG_WINDOW;
                    break;
                }
            }
        }

        /* Update loopCount value for while loop */
        loopCount = maxCnt;

        while ((PMIC_ST_SUCCESS == status) && (loopCount > 0U)) {
            /* Start Critical Section */
            Pmic_criticalSectionStart(handle);

            status = Pmic_commIntf_recvByte(handle, PMIC_WD_STAT_REG, &
                failCnt);

            if ((PMIC_ST_SUCCESS == status) && ((uint8_t)0U != (failCnt & (uint8_t)0x40U))) {
                status = Pmic_commIntf_recvByte(handle,
                    PMIC_WD_ERR_STAT_REG, & regVal);
                if ((PMIC_ST_SUCCESS == status) && ((uint8_t)0U != (regVal & (uint8_t)0x08U))) {
                    status = PMIC_ST_ERR_WDG_EARLY_ANSWER;
                }

                qaSequences = 0U;
                flag = 1;
            } else {
                if ((PMIC_ST_SUCCESS == status) && ((uint8_t)0U != (failCnt & (uint8_t)0x20U))) {
                    qaSequences--;
                    flag = 1;
                }
            }

            /* Stop Critical Section */
            Pmic_criticalSectionStop(handle);

            if (flag == 1) {
                break;
            }

            loopCount--;
        }
    }

    return status;
}

/*!
 * \brief  API to set Watch Dog QA Mode, Disable ret to Long Window and
 *         Write Answers for Long Window
 */
static int32_t Pmic_wdgQaSetModeRetlongwinCfgWriteAnswersLongwindow(
    Pmic_CoreHandle_t *handle) {
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regVal = 0x0U;

    /* Start Critical Section */
    Pmic_criticalSectionStart(handle);

    /* Reading watchdog mode value */
    status = Pmic_commIntf_recvByte(handle, PMIC_WD_CFG_REG, &regVal);

    /* Set watchdog mode to QA mode */
    if (status == PMIC_ST_SUCCESS) {
        Pmic_setBitField(&regVal, PMIC_WD_MODE_SHIFT, PMIC_WD_MODE_MASK, PMIC_WDG_QA_MODE);
        status = Pmic_commIntf_sendByte(handle, PMIC_WD_CFG_REG, regVal);
    }

    /* Stop Critical Section */
    Pmic_criticalSectionStop(handle);

    /* Disable ret to Long Window */
    if (status == PMIC_ST_SUCCESS) {
        status = Pmic_wdgSetReturnToLongWindow(handle, false);
    }

    /* Clear WDG Error bits */
    if (status == PMIC_ST_SUCCESS) {
        /* Start Critical Section */
        Pmic_criticalSectionStart(handle);

        status = Pmic_commIntf_recvByte(handle, PMIC_WD_ERR_STAT_REG, &regVal);

        if (status == PMIC_ST_SUCCESS) {
            status = Pmic_commIntf_sendByte(handle, PMIC_WD_ERR_STAT_REG, regVal);
        }

        /* Stop Critical Section */
        Pmic_criticalSectionStop(handle);
    }

    /* Write Answers for Long Window */
    if (status == PMIC_ST_SUCCESS) {
        status = WDG_qaWriteAnswers(handle);

        if (status == PMIC_ST_ERR_INV_WDG_ANSWER) {
            Pmic_criticalSectionStart(handle);
            status = Pmic_commIntf_recvByte(handle, PMIC_WD_ERR_STAT_REG, &regVal);
            Pmic_criticalSectionStop(handle);

            if ((status == PMIC_ST_SUCCESS) && (0U != (regVal & 0x01U))) {
                status = PMIC_ST_ERR_INV_WDG_WINDOW;
            }
        }
    }

    return status;
}

/*!
 * \brief   API to clear PMIC watchdog error status based on wdgErrType
 */
static int32_t Pmic_wdgClrErrStatusWdgErrType(Pmic_CoreHandle_t * handle,
    const uint8_t wdgErrType, uint8_t regVal) {
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t errStatus = 1U, regData = 0U;

    if ((PMIC_WDG_ERR_LONG_WIN_TIMEOUT == wdgErrType) &&
        (Pmic_getBitField(regVal, PMIC_WD_ERR_STAT_REG_WD_LONGWIN_TMO_SHIFT,
                          (uint8_t)PMIC_WD_ERR_STAT_REG_WD_LONGWIN_TMO_MASK) != 0U)) {
        Pmic_setBitField( &regData, PMIC_WD_ERR_STAT_REG_WD_LONGWIN_TMO_SHIFT,
                          (uint8_t)PMIC_WD_ERR_STAT_REG_WD_LONGWIN_TMO_MASK, errStatus);
    } else if ((PMIC_WDG_ERR_TIMEOUT == wdgErrType) &&
        (Pmic_getBitField(regVal, PMIC_WD_ERR_STAT_REG_WD_TMO_SHIFT,
                          (uint8_t)PMIC_WD_ERR_STAT_REG_WD_TMO_MASK) != 0U)) {
        Pmic_setBitField( &regData, PMIC_WD_ERR_STAT_REG_WD_TMO_SHIFT,
                          (uint8_t)PMIC_WD_ERR_STAT_REG_WD_TMO_MASK, errStatus);
    } else if ((PMIC_WDG_ERR_TRIGGER_EARLY == wdgErrType) &&
        (Pmic_getBitField(regVal, PMIC_WD_ERR_STAT_REG_WD_TRIG_EARLY_SHIFT,
                          (uint8_t)PMIC_WD_ERR_STAT_REG_WD_TRIG_EARLY_MASK) !=
            0U)) {
        Pmic_setBitField( &regData, PMIC_WD_ERR_STAT_REG_WD_TRIG_EARLY_SHIFT,
                          (uint8_t)PMIC_WD_ERR_STAT_REG_WD_TRIG_EARLY_MASK, errStatus);
    } else if ((PMIC_WDG_ERR_ANSWER_EARLY == wdgErrType) &&
        (Pmic_getBitField(regVal, PMIC_WD_ERR_STAT_REG_WD_ANSW_EARLY_SHIFT,
                          (uint8_t)PMIC_WD_ERR_STAT_REG_WD_ANSW_EARLY_MASK) !=
            0U)) {
        Pmic_setBitField( &regData, PMIC_WD_ERR_STAT_REG_WD_ANSW_EARLY_SHIFT,
                          (uint8_t)PMIC_WD_ERR_STAT_REG_WD_ANSW_EARLY_MASK, errStatus);
    } else if ((PMIC_WDG_ERR_SEQ_ERR == wdgErrType) &&
        (Pmic_getBitField(regVal, PMIC_WD_ERR_STAT_REG_WD_SEQ_ERR_SHIFT,
                          (uint8_t)PMIC_WD_ERR_STAT_REG_WD_SEQ_ERR_MASK) != 0U)) {
        Pmic_setBitField( &regData, PMIC_WD_ERR_STAT_REG_WD_SEQ_ERR_SHIFT,
                          (uint8_t)PMIC_WD_ERR_STAT_REG_WD_SEQ_ERR_MASK, errStatus);
    } else if ((PMIC_WDG_ERR_ANS_ERR == wdgErrType) &&
        (Pmic_getBitField(regVal, PMIC_WD_ERR_STAT_REG_WD_ANSW_ERR_SHIFT,
                          (uint8_t)PMIC_WD_ERR_STAT_REG_WD_ANSW_ERR_MASK) != 0U)) {
        Pmic_setBitField( &regData, PMIC_WD_ERR_STAT_REG_WD_ANSW_ERR_SHIFT,
                          (uint8_t)PMIC_WD_ERR_STAT_REG_WD_ANSW_ERR_MASK, errStatus);
    } else if ((PMIC_WDG_ERR_THRES1 == wdgErrType) &&
        (Pmic_getBitField(regVal, PMIC_WD_ERR_STAT_REG_WD_TH1_ERR_SHIFT,
                          (uint8_t)PMIC_WD_ERR_STAT_REG_WD_TH1_ERR_MASK) != 0U)) {
        Pmic_setBitField( &regData, PMIC_WD_ERR_STAT_REG_WD_TH1_ERR_SHIFT,
                          (uint8_t)PMIC_WD_ERR_STAT_REG_WD_TH1_ERR_MASK, errStatus);
    } else {
        if (Pmic_getBitField(regVal, PMIC_WD_ERR_STAT_REG_WD_TH2_ERR_SHIFT,
                             (uint8_t)PMIC_WD_ERR_STAT_REG_WD_TH2_ERR_MASK) != 0U) {
            Pmic_setBitField( &regData, PMIC_WD_ERR_STAT_REG_WD_TH2_ERR_SHIFT,
                              (uint8_t)PMIC_WD_ERR_STAT_REG_WD_TH2_ERR_MASK, errStatus);
        }
    }

    if (0U != regData) {
        /*! Start Critical Section */
        Pmic_criticalSectionStart(handle);

        status = Pmic_commIntf_sendByte(handle, PMIC_WD_ERR_STAT_REG, regData);

        /*! Stop Critical Section */
        Pmic_criticalSectionStop(handle);
    }

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
        Pmic_setBitField(&regVal, PMIC_WD_EN_SHIFT, PMIC_WD_EN_MASK, enable ? 1U : 0U);
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
        *isEnabled = (Pmic_getBitField(regData, PMIC_WD_EN_SHIFT, PMIC_WD_EN_MASK) == 1U);
    }

    Pmic_criticalSectionStop(handle);

    return status;
}

int32_t Pmic_wdgSetCfg(Pmic_CoreHandle_t *handle, const Pmic_WdgCfg_t *config) {
    int32_t status = WDG_validatePmicCoreHandle(handle);

    if (PMIC_ST_SUCCESS == status) {
        status = WDG_setWindowsTimeIntervals(handle, config);
    }

    if (PMIC_ST_SUCCESS == status) {
        status = WDG_setThresholdValues(handle, config);
    }

    if (PMIC_ST_SUCCESS == status) {
        status = WDG_setCfgParams(handle, config);
    }

    if (PMIC_ST_SUCCESS == status) {
        status = Pmic_WdgSetQaConfigurations(handle, config);
    }

    return status;
}

int32_t Pmic_wdgGetCfg(Pmic_CoreHandle_t *handle, Pmic_WdgCfg_t *config) {
    int32_t status = WDG_validatePmicCoreHandle(handle);

    if ((PMIC_ST_SUCCESS == status) && (NULL == config)) {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if (PMIC_ST_SUCCESS == status) {
        status = Pmic_WdgGetWindowsTimeIntervals(handle, config);
    }

    if (PMIC_ST_SUCCESS == status) {
        status = Pmic_WdgGetThresholdValues(handle, config);
    }

    if (PMIC_ST_SUCCESS == status) {
        status = Pmic_WdgGetCtrlParams(handle, config);
    }

    if (PMIC_ST_SUCCESS == status) {
        status = Pmic_WdgGetQaConfigurations(handle, config);
    }

    return status;
}

int32_t Pmic_wdgSetPowerHold(Pmic_CoreHandle_t *handle, bool enable) {
    int32_t status = WDG_validatePmicCoreHandle(handle);
    uint8_t regVal = 0U;

    Pmic_criticalSectionStart(handle);

    status = Pmic_commIntf_recvByte(handle, PMIC_WD_CFG_REG, &regVal);

    if (PMIC_ST_SUCCESS == status) {
        Pmic_setBitField(&regVal, PMIC_WD_PWRHOLD_SHIFT, PMIC_WD_PWRHOLD_MASK, enable ? 1U : 0U);
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
        *isEnabled = (Pmic_getBitField(regData, PMIC_WD_PWRHOLD_SHIFT, PMIC_WD_PWRHOLD_MASK) == 1U);
    }

    Pmic_criticalSectionStop(handle);

    return status;
}

int32_t Pmic_wdgSetReturnToLongWindow(Pmic_CoreHandle_t * handle, bool enable) {
    int32_t status = WDG_validatePmicCoreHandle(handle);
    uint8_t regVal = 0U;

    Pmic_criticalSectionStart(handle);

    if (status == PMIC_ST_SUCCESS) {
        status = Pmic_commIntf_recvByte(handle, PMIC_WD_CFG_REG, &regVal);
    }

    if (status == PMIC_ST_SUCCESS) {
        Pmic_setBitField(&regVal,
            PMIC_WD_RETURN_LONGWIN_SHIFT,
            PMIC_WD_RETURN_LONGWIN_MASK,
            enable ? 1U : 0U);

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

    Pmic_criticalSectionStart(handle);

    if (status == PMIC_ST_SUCCESS) {
        status = Pmic_commIntf_recvByte(handle, PMIC_WD_CFG_REG, &regData);
    }

    if (status == PMIC_ST_SUCCESS) {
        *isEnabled = (Pmic_getBitField(regData, PMIC_WD_RETURN_LONGWIN_SHIFT, PMIC_WD_RETURN_LONGWIN_MASK) == 1U);
    }

    Pmic_criticalSectionStop(handle);

    return status;
}

int32_t Pmic_wdgGetErrorStatus(Pmic_CoreHandle_t *handle, Pmic_WdgErrStatus_t *pErrStatus) {
    int32_t status = WDG_validatePmicCoreHandle(handle);
    uint8_t regVal = 0x0U;

    if ((status == PMIC_ST_SUCCESS) && (pErrStatus == NULL)) {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    /* Reading error status register */
    if (status == PMIC_ST_SUCCESS) {
        Pmic_criticalSectionStart(handle);
        status = Pmic_commIntf_recvByte(handle, PMIC_WD_ERR_STAT_REG, &regVal);
        Pmic_criticalSectionStop(handle);
    }

    /* Get watchdog error status */
    if (status == PMIC_ST_SUCCESS) {
        // Extract struct data from the register read data
        WDG_extractIntErrStatusField(pErrStatus, regVal);
    }

    return status;
}

int32_t Pmic_wdgClrErrStatus(Pmic_CoreHandle_t *handle, const uint8_t wdgErrType) {
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regVal = 0x0U;

    /* Validate handle and WDG subsystem */
    status = WDG_validatePmicCoreHandle(handle);

    if ((status == PMIC_ST_SUCCESS) && (wdgErrType > PMIC_WDG_ERR_ALL)) {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    if (PMIC_ST_SUCCESS == status) {
        /* Start Critical Section */
        Pmic_criticalSectionStart(handle);

        /* Reading error status register */
        status = Pmic_commIntf_recvByte(handle, PMIC_WD_ERR_STAT_REG, &regVal);

        if (0U == regVal) {
            status = PMIC_ST_ERR_FAIL;
        }

        if ((PMIC_ST_SUCCESS == status) && (PMIC_WDG_ERR_ALL == wdgErrType)) {
            status = Pmic_commIntf_sendByte(handle, PMIC_WD_ERR_STAT_REG, regVal);
        }

        /* Stop Critical Section */
        Pmic_criticalSectionStop(handle);
    }

    if ((PMIC_ST_SUCCESS == status) && (PMIC_WDG_ERR_ALL != wdgErrType)) {
        status =
            Pmic_wdgClrErrStatusWdgErrType(handle, wdgErrType, regVal);
    }

    return status;
}

int32_t Pmic_wdgGetFailCntStat(Pmic_CoreHandle_t * handle,
    Pmic_WdgFailCntStat_t * pFailCount) {
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regVal = 0x00U;

    /* Validate handle and WDG subsystem */
    status = WDG_validatePmicCoreHandle(handle);

    if ((PMIC_ST_SUCCESS == status) && (NULL == pFailCount)) {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if (PMIC_ST_SUCCESS == status) {
        /* Start Critical Section */
        Pmic_criticalSectionStart(handle);

        /* Reading error status register */
        status = Pmic_commIntf_recvByte(handle, PMIC_WD_STAT_REG, &
            regVal);

        /* Stop Critical Section */
        Pmic_criticalSectionStop(handle);
    }

    /* Get watchdog Bad Event status */
    if ((PMIC_ST_SUCCESS == status) &&
        (true == pmic_validParamCheck(pFailCount -> validParams,
            PMIC_CFG_WD_BAD_EVENT_STAT_VALID))) {
        if (Pmic_getBitField(regVal, PMIC_WD_STAT_REG_WD_BAD_EVENT_SHIFT,
                (uint8_t)PMIC_WD_STAT_REG_WD_BAD_EVENT_MASK) != 0U) {
            pFailCount -> wdBadEvent = true;
        } else {
            pFailCount -> wdBadEvent = false;
        }
    }

    /* Get watchdog Good Event status */
    if ((PMIC_ST_SUCCESS == status) &&
        (true == pmic_validParamCheck(pFailCount -> validParams,
            PMIC_CFG_WD_GOOD_EVENT_STAT_VALID))) {
        if (Pmic_getBitField(regVal, PMIC_WD_STAT_REG_WD_FIRST_OK_SHIFT,
                (uint8_t)PMIC_WD_STAT_REG_WD_FIRST_OK_MASK) != 0U) {
            pFailCount -> wdGudEvent = true;
        } else {
            pFailCount -> wdGudEvent = false;
        }
    }

    /* Get watchdog Fail count Value */
    if ((PMIC_ST_SUCCESS == status) &&
        (true == pmic_validParamCheck(pFailCount -> validParams,
            PMIC_CFG_WD_FAIL_CNT_VAL_VALID))) {
        pFailCount -> wdFailCnt =
            Pmic_getBitField(regVal, PMIC_WD_STAT_REG_WD_ERR_CNT_SHIFT,
                (uint8_t)PMIC_WD_STAT_REG_WD_ERR_CNT_MASK);
    }

    return status;
}

int32_t Pmic_wdgStartQaSequence(Pmic_CoreHandle_t * handle,
    uint32_t num_of_sequences, uint32_t maxCnt) {
    int32_t status = PMIC_ST_SUCCESS;
    uint32_t sequences = num_of_sequences;
    uint8_t regVal = 0x0U;

    /* Validate handle and WDG subsystem */
    status = WDG_validatePmicCoreHandle(handle);

    if ((PMIC_ST_SUCCESS == status) && (maxCnt < PMIC_WDG_WAIT_CNT_MIN_VAL)) {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    /* Set Watchdog QA Mode, disable return to Long Window, and
     * write answers for Long Window */
    if (PMIC_ST_SUCCESS == status) {
        status =
            Pmic_wdgQaSetModeRetlongwinCfgWriteAnswersLongwindow(handle);
    }

    /* Dummy Read operations to sync the WatchDog */
    if (PMIC_ST_SUCCESS == status) {
        /* Start Critical Section */
        Pmic_criticalSectionStart(handle);

        status = Pmic_commIntf_recvByte(handle, PMIC_WD_ERR_STAT_REG, &
            regVal);

        if (PMIC_ST_SUCCESS == status) {
            status = Pmic_commIntf_recvByte(handle, PMIC_WD_STAT_REG, &
                regVal);
        }

        /* Stop Critical Section */
        Pmic_criticalSectionStop(handle);
    }

    /* Write QA Answers for given numbers of sequences */
    if (PMIC_ST_SUCCESS == status) {
        status =
            WDG_qaWriteAnswersNumSequence(handle, sequences, maxCnt);
    }

    if (PMIC_ST_SUCCESS == status) {
        /* Enable Return long window Enable */
        status = Pmic_wdgSetReturnToLongWindow(handle, true);
    }

    return status;
}

int32_t Pmic_wdgQaSequenceWriteAnswer(Pmic_CoreHandle_t * handle) {
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t qaAnsCnt = 0U;
    uint8_t qaQuesCnt = 0U;
    uint8_t qaFbk = 0U;

    /* Validate handle and WDG subsystem */
    status = WDG_validatePmicCoreHandle(handle);

    /* Write Answers for Long Window */
    if (PMIC_ST_SUCCESS == status) {
        /* Start Critical Section */
        Pmic_criticalSectionStart(handle);

        status =
            Pmic_commIntf_recvByte(handle, PMIC_WD_QA_CFG_REG, & qaFbk);

        /* Stop Critical Section */
        Pmic_criticalSectionStop(handle);
    }

    /* Get wdg QA Feedback value */
    if (PMIC_ST_SUCCESS == status) {
        qaFbk = Pmic_getBitField(qaFbk, PMIC_WD_QA_CFG_WD_QA_FDBK_SHIFT,
                                 (uint8_t)PMIC_WD_QA_CFG_WD_QA_FDBK_MASK);
    }

    if (PMIC_ST_SUCCESS == status) {
        status =
            Pmic_wdgReadQuesandAnswerCount(handle, & qaAnsCnt, & qaQuesCnt);
    }

    if (PMIC_ST_SUCCESS == status) {
        status = WDG_qaEvalAndWriteAnswer(handle, qaAnsCnt,
            qaQuesCnt, qaFbk);
    }

    return status;
}

int32_t Pmic_wdgStartTriggerSequence(Pmic_CoreHandle_t * handle) {
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regVal = 0x0U;

    /* Validate handle and WDG subsystem */
    status = WDG_validatePmicCoreHandle(handle);

    if (PMIC_ST_SUCCESS == status) {
        /* Start Critical Section */
        Pmic_criticalSectionStart(handle);

        /* Reading watchdog mode value */
        status = Pmic_commIntf_recvByte(handle, PMIC_WD_CFG_REG, &regVal);

        /* Set watchdog mode to trigger mode */
        if (PMIC_ST_SUCCESS == status) {
            Pmic_setBitField(&regVal,
                PMIC_WD_MODE_SHIFT,
                PMIC_WD_MODE_MASK,
                PMIC_WDG_TRIGGER_MODE);

            status = Pmic_commIntf_sendByte(handle, PMIC_WD_CFG_REG, regVal);
        }

        /* Stop Critical Section */
        Pmic_criticalSectionStop(handle);
    }

    return status;
}

int32_t Pmic_wdgBeginSequences(Pmic_CoreHandle_t *handle, const uint8_t mode) {
    int32_t status = PMIC_ST_SUCCESS;
    bool wdgEnabled = false;
    uint8_t regVal = 0U;

    /* Parameter validation */
    status = WDG_validatePmicCoreHandle(handle);
    if ((status == PMIC_ST_SUCCESS) && (mode > PMIC_WDG_MODE_MAX)) {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    /* Check if WDG is enabled */
    if (status == PMIC_ST_SUCCESS) {
        status = Pmic_wdgGetEnableState(handle, &wdgEnabled);

        if ((status == PMIC_ST_SUCCESS) && !wdgEnabled) {
            status = PMIC_ST_ERR_WDG_DISABLED;
        }
    }

    /* Start critical section before serial comm. operations */
    Pmic_criticalSectionStart(handle);

    /* Clear all WDG error statuses (all bit fields in the */
    /* WD_ERR_STATUS register are W1C - write 1 to clear) */
    regVal = 0xFFU;
    status = Pmic_commIntf_sendByte(handle, PMIC_WD_ERR_STAT_REG, regVal);

    if (status == PMIC_ST_SUCCESS) {
        /* Read WD_MODE_REG register */
        status = Pmic_commIntf_recvByte(handle, PMIC_WD_CFG_REG, &regVal);
    }

    if (status == PMIC_ST_SUCCESS) {
        /* Select WDG mode of operation */
        Pmic_setBitField(&regVal, PMIC_WD_MODE_SHIFT, PMIC_WD_MODE_MASK, mode);

        /* Clear WD_PWRHOLD bit field to enable WDG to exit Long Window */
        Pmic_setBitField(&regVal, PMIC_WD_PWRHOLD_SHIFT, PMIC_WD_PWRHOLD_MASK, 0U);

        /* Clear WD_RETURN_LONGWIN bit field to disable return to Long Window */
        Pmic_setBitField(&regVal, PMIC_WD_RETURN_LONGWIN_SHIFT, PMIC_WD_RETURN_LONGWIN_MASK, 0U);

        /* Write new register value back to PMIC */
        status = Pmic_commIntf_sendByte(handle, PMIC_WD_CFG_REG, regVal);
    }

    /* Stop critical section after serial comm. operations */
    Pmic_criticalSectionStop(handle);

    return status;
}
