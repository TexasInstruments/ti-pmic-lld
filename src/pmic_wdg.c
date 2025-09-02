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
 * @file pmic_wdg.c
 *
 * @brief Contains watchdog module API definitions.
 */

/* ========================================================================== */
/*                              Include Files                                 */
/* ========================================================================== */

#include "pmic.h"

#include "regmap/wdg.h"

/* ========================================================================== */
/*                            Macros & Typedefs                               */
/* ========================================================================== */

#define WDG_SET_CFG ((bool)true)
#define WDG_GET_CFG ((bool)false)

/* ========================================================================== */
/*                           Function Definitions                             */
/* ========================================================================== */

int32_t Pmic_wdgSetEnableState(const Pmic_Handle_t *handle, bool enable) {
    int32_t status = Pmic_checkPmicHandle(handle);
    if (status != PMIC_ST_SUCCESS) {
        return status;
    }

    return Pmic_ioReadModifyWrite_bCS(handle, WD_ENABLE_REG_REGADDR, WD_EN_SHIFT, enable);
}

int32_t Pmic_wdgGetEnableState(const Pmic_Handle_t *handle, bool *isEnabled) {
    uint8_t regData = 0U;

    int32_t status = Pmic_checkPmicHandle(handle);
    if (status != PMIC_ST_SUCCESS) {
        return status;
    }

    if (isEnabled == NULL) {
        return PMIC_ST_ERR_NULL_PARAM;
    }

    status = Pmic_ioRxByte_CS(handle, WD_ENABLE_REG_REGADDR, &regData);
    if (status != PMIC_ST_SUCCESS) {
        return status;
    }

    *isEnabled = Pmic_getBitField_b(regData, WD_EN_SHIFT);
    return PMIC_ST_SUCCESS;
}

static int32_t WDG_setWinDuration(const Pmic_Handle_t *handle, const Pmic_WdgCfg_t *wdgCfg) {
    int32_t status = PMIC_ST_SUCCESS;

    if (Pmic_validParamCheck(wdgCfg->validParams, PMIC_WDG_LONG_WIN_DURATION_VALID)) {
        status = Pmic_ioTxByte_CS(handle, WD_LONGWIN_CFG_REGADDR, wdgCfg->longWinDuration);
        if (status != PMIC_ST_SUCCESS) {
            return status;
        }
    }

    if (Pmic_validParamCheck(wdgCfg->validParams, PMIC_WDG_WIN1_DURATION_VALID)) {
        if (wdgCfg->win1Duration > PMIC_WDG_WIN_DURATION_MAX) {
            return PMIC_ST_ERR_INV_PARAM;
        }

        status = Pmic_ioReadModifyWrite_CS(handle, WD_WIN1_CFG_REGADDR, WD_WIN1_SHIFT, WD_WIN1_MASK, wdgCfg->win1Duration);
        if (status != PMIC_ST_SUCCESS) {
            return status;
        }
    }

    if (Pmic_validParamCheck(wdgCfg->validParams, PMIC_WDG_WIN2_DURATION_VALID)) {
        if (wdgCfg->win2Duration > PMIC_WDG_WIN_DURATION_MAX) {
            return PMIC_ST_ERR_INV_PARAM;
        }

        status = Pmic_ioReadModifyWrite_CS(handle, WD_WIN2_CFG_REGADDR, WD_WIN2_SHIFT, WD_WIN1_MASK, wdgCfg->win2Duration);
        if (status != PMIC_ST_SUCCESS) {
            return status;
        }
    }

    return PMIC_ST_SUCCESS;
}

static int32_t WDG_setQaCfg(const Pmic_Handle_t *handle, const Pmic_WdgCfg_t *wdgCfg) {
    uint8_t regData = 0U;

    int32_t status = Pmic_ioRxByte_startCS(handle, WD_QA_CFG_REGADDR, &regData);
    if (status != PMIC_ST_SUCCESS) {
        return status;
    }

    if (Pmic_validParamCheck(wdgCfg->validParams, PMIC_WDG_QA_FDBK_VALID)) {
        if (wdgCfg->qaFdbk > PMIC_WDG_QA_FDBK_VAL_MAX) {
            return PMIC_ST_ERR_INV_PARAM;
        }

        Pmic_setBitField(&regData, WD_QA_FDBK_SHIFT, WD_QA_FDBK_MASK, wdgCfg->qaFdbk);
    }

    if (Pmic_validParamCheck(wdgCfg->validParams, PMIC_WDG_QA_LFSR_VALID)) {
        if (wdgCfg->qaLfsr > PMIC_WDG_QA_LFSR_VAL_MAX) {
            return PMIC_ST_ERR_INV_PARAM;
        }

        Pmic_setBitField(&regData, WD_QA_LFSR_SHIFT, WD_QA_LFSR_MASK, wdgCfg->qaLfsr);
    }

    if (Pmic_validParamCheck(wdgCfg->validParams, PMIC_WDG_QA_SEED_VALID)) {
        if (wdgCfg->qaSeed > PMIC_WDG_QA_SEED_VAL_MAX) {
            return PMIC_ST_ERR_INV_PARAM;
        }

        Pmic_setBitField(&regData, WD_QUESTION_SEED_SHIFT, WD_QUESTION_SEED_MASK, wdgCfg->qaSeed);
    }

    return Pmic_ioTxByte_endCS(handle, WD_QA_CFG_REGADDR, regData);
}

static int32_t WDG_setThr(const Pmic_Handle_t *handle, const Pmic_WdgCfg_t *wdgCfg) {
    uint8_t regData = 0U;

    int32_t status = Pmic_ioRxByte_startCS(handle, WD_THR_CFG_REGADDR, &regData);
    if (status != PMIC_ST_SUCCESS) {
        return status;
    }

    if (Pmic_validParamCheck(wdgCfg->validParams, PMIC_WDG_FAIL_THR_VALID)) {
        if (wdgCfg->failThr > PMIC_WDG_THR_CNT_MAX) {
            return PMIC_ST_ERR_INV_PARAM;
        }

        Pmic_setBitField(&regData, WD_FAIL_TH_SHIFT, WD_FAIL_TH_MASK, wdgCfg->failThr);
    }

    if (Pmic_validParamCheck(wdgCfg->validParams, PMIC_WDG_RST_THR_VALID)) {
        if (wdgCfg->rstThr > PMIC_WDG_THR_CNT_MAX) {
            return PMIC_ST_ERR_INV_PARAM;
        }

        Pmic_setBitField(&regData, WD_RST_TH_SHIFT, WD_RST_TH_MASK, wdgCfg->rstThr);
    }

    return Pmic_ioTxByte_endCS(handle, WD_THR_CFG_REGADDR, regData);
}

static int32_t WDG_setRstEn(const Pmic_Handle_t *handle, const Pmic_WdgCfg_t *wdgCfg) {
    return Pmic_ioReadModifyWrite_bCS(handle, WD_ENABLE_REG_REGADDR, WD_RST_EN_SHIFT, wdgCfg->rstEn);
}

static int32_t WDG_getWinDuration(const Pmic_Handle_t *handle, Pmic_WdgCfg_t *wdgCfg) {
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    if (Pmic_validParamCheck(wdgCfg->validParams, PMIC_WDG_LONG_WIN_DURATION_VALID)) {
        status = Pmic_ioRxByte_CS(handle, WD_LONGWIN_CFG_REGADDR, &regData);
        if (status != PMIC_ST_SUCCESS) {
            return status;
        }

        wdgCfg->longWinDuration = regData;
    }

    if (Pmic_validParamCheck(wdgCfg->validParams, PMIC_WDG_WIN1_DURATION_VALID)) {
        status = Pmic_ioRxByte_CS(handle, WD_WIN1_CFG_REGADDR, &regData);
        if (status != PMIC_ST_SUCCESS) {
            return status;
        }

        wdgCfg->win1Duration = Pmic_getBitField(regData, WD_WIN1_SHIFT, WD_WIN1_MASK);
    }

    if (Pmic_validParamCheck(wdgCfg->validParams, PMIC_WDG_WIN2_DURATION_VALID)) {
        status = Pmic_ioRxByte_CS(handle, WD_WIN2_CFG_REGADDR, &regData);
        if (status != PMIC_ST_SUCCESS) {
            return status;
        }

        wdgCfg->win2Duration = Pmic_getBitField(regData, WD_WIN2_SHIFT, WD_WIN2_MASK);
    }

    return PMIC_ST_SUCCESS;
}

static int32_t WDG_getQaCfg(const Pmic_Handle_t *handle, Pmic_WdgCfg_t *wdgCfg) {
    uint8_t regData = 0U;

    int32_t status = Pmic_ioRxByte_CS(handle, WD_QA_CFG_REGADDR, &regData);
    if (status != PMIC_ST_SUCCESS) {
        return status;
    }

    if (Pmic_validParamCheck(wdgCfg->validParams, PMIC_WDG_QA_FDBK_VALID)) {
        wdgCfg->qaFdbk = Pmic_getBitField(regData, WD_QA_FDBK_SHIFT, WD_QA_FDBK_MASK);
    }

    if (Pmic_validParamCheck(wdgCfg->validParams, PMIC_WDG_QA_LFSR_VALID)) {
        wdgCfg->qaLfsr = Pmic_getBitField(regData, WD_QA_LFSR_SHIFT, WD_QA_LFSR_MASK);
    }

    if (Pmic_validParamCheck(wdgCfg->validParams, PMIC_WDG_QA_SEED_VALID)) {
        wdgCfg->qaSeed = Pmic_getBitField(regData, WD_QUESTION_SEED_SHIFT, WD_QUESTION_SEED_MASK);
    }

    return PMIC_ST_SUCCESS;
}

static int32_t WDG_getThr(const Pmic_Handle_t *handle, Pmic_WdgCfg_t *wdgCfg) {
    uint8_t regData = 0U;

    int32_t status = Pmic_ioRxByte_CS(handle, WD_THR_CFG_REGADDR, &regData);
    if (status != PMIC_ST_SUCCESS) {
        return status;
    }

    if (Pmic_validParamCheck(wdgCfg->validParams, PMIC_WDG_FAIL_THR_VALID)) {
        wdgCfg->failThr = Pmic_getBitField(regData, WD_FAIL_TH_SHIFT, WD_FAIL_TH_MASK);
    }

    if (Pmic_validParamCheck(wdgCfg->validParams, PMIC_WDG_RST_THR_VALID)) {
        wdgCfg->rstThr = Pmic_getBitField(regData, WD_RST_TH_SHIFT, WD_RST_TH_MASK);
    }

    return PMIC_ST_SUCCESS;
}

static int32_t WDG_getRstEn(const Pmic_Handle_t *handle, Pmic_WdgCfg_t *wdgCfg) {
    uint8_t regData = 0U;

    int32_t status = Pmic_ioRxByte_CS(handle, WD_ENABLE_REG_REGADDR, &regData);
    if (status != PMIC_ST_SUCCESS) {
        return status;
    }

    wdgCfg->rstEn = Pmic_getBitField_b(regData, WD_RST_EN_SHIFT);
    return PMIC_ST_SUCCESS;
}

static int32_t WDG_setGetCfg(const Pmic_Handle_t *handle, Pmic_WdgCfg_t *wdgCfg, bool setCfg) {
    const bool winDurationValid = \
        PMIC_WDG_LONG_WIN_DURATION_VALID | PMIC_WDG_WIN1_DURATION_VALID | PMIC_WDG_WIN2_DURATION_VALID;
    const bool qaCfgValid = \
        PMIC_WDG_QA_FDBK_VALID | PMIC_WDG_QA_LFSR_VALID | PMIC_WDG_QA_SEED_VALID;
    const bool thrValid = \
        PMIC_WDG_FAIL_THR_VALID | PMIC_WDG_RST_THR_VALID;

    int32_t status = Pmic_checkPmicHandle(handle);
    if (status != PMIC_ST_SUCCESS) {
        return status;
    }

    if (wdgCfg == NULL) {
        return PMIC_ST_ERR_NULL_PARAM;
    }

    if (wdgCfg->validParams == 0U) {
        return PMIC_ST_ERR_INV_PARAM;
    }

    if (Pmic_validParamCheck(wdgCfg->validParams, winDurationValid)) {
        status = setCfg ? WDG_setWinDuration(handle, wdgCfg) : WDG_getWinDuration(handle, wdgCfg);
        if (status != PMIC_ST_SUCCESS) {
            return status;
        }
    }

    if (Pmic_validParamCheck(wdgCfg->validParams, qaCfgValid)) {
        status = setCfg ? WDG_setQaCfg(handle, wdgCfg) : WDG_getQaCfg(handle, wdgCfg);
        if (status != PMIC_ST_SUCCESS) {
            return status;
        }
    }

    if (Pmic_validParamCheck(wdgCfg->validParams, thrValid)) {
        status = setCfg ? WDG_setThr(handle, wdgCfg) : WDG_getThr(handle, wdgCfg);
        if (status != PMIC_ST_SUCCESS) {
            return status;
        }
    }

    if (Pmic_validParamCheck(wdgCfg->validParams, PMIC_WDG_RST_EN_VALID)) {
        status = setCfg ? WDG_setRstEn(handle, wdgCfg) : WDG_getRstEn(handle, wdgCfg);
    }

    return status;
}

int32_t Pmic_wdgSetCfg(const Pmic_Handle_t *handle, Pmic_WdgCfg_t *wdgCfg) {
    return WDG_setGetCfg(handle, wdgCfg, WDG_SET_CFG);
}

int32_t Pmic_wdgGetCfg(const Pmic_Handle_t *handle, Pmic_WdgCfg_t *wdgCfg) {
    return WDG_setGetCfg(handle, wdgCfg, WDG_GET_CFG);
}

int32_t Pmic_wdgSetPwrHold(const Pmic_Handle_t *handle, bool enable) {
    int32_t status = Pmic_checkPmicHandle(handle);
    if (status != PMIC_ST_SUCCESS) {
        return status;
    }

    return Pmic_ioReadModifyWrite_bCS(handle, WD_MODE_REG_REGADDR, WD_PWRHOLD_SHIFT, enable);
}

int32_t Pmic_wdgGetPwrHold(const Pmic_Handle_t *handle, bool *isEnabled) {
    uint8_t regData = 0U;

    int32_t status = Pmic_checkPmicHandle(handle);
    if (status != PMIC_ST_SUCCESS) {
        return status;
    }

    if (isEnabled == NULL) {
        return PMIC_ST_ERR_NULL_PARAM;
    }

    status = Pmic_ioRxByte_CS(handle, WD_MODE_REG_REGADDR, &regData);
    if (status != PMIC_ST_SUCCESS) {
        return status;
    }

    *isEnabled = Pmic_getBitField_b(regData, WD_PWRHOLD_SHIFT);
    return PMIC_ST_SUCCESS;
}

int32_t Pmic_wdgSetRetLongWin(const Pmic_Handle_t *handle, bool enable) {
    int32_t status = Pmic_checkPmicHandle(handle);
    if (status != PMIC_ST_SUCCESS) {
        return status;
    }

    return Pmic_ioReadModifyWrite_bCS(handle, WD_MODE_REG_REGADDR, WD_RETURN_LONGWIN_SHIFT, enable);
}

int32_t Pmic_wdgGetRetLongWin(const Pmic_Handle_t *handle, bool *isEnabled) {
    uint8_t regData = 0U;

    int32_t status = Pmic_checkPmicHandle(handle);
    if (status != PMIC_ST_SUCCESS) {
        return status;
    }

    if (isEnabled == NULL) {
        return PMIC_ST_ERR_NULL_PARAM;
    }

    status = Pmic_ioRxByte_CS(handle, WD_MODE_REG_REGADDR, &regData);
    if (status != PMIC_ST_SUCCESS) {
        return status;
    }

    *isEnabled = Pmic_getBitField_b(regData, WD_RETURN_LONGWIN_SHIFT);
    return PMIC_ST_SUCCESS;
}

int32_t Pmic_wdgClrErrStat(const Pmic_Handle_t *handle, const Pmic_WdgErrStat_t *wdgErrStat) {
    uint8_t regData = 0U;

    int32_t status = Pmic_checkPmicHandle(handle);
    if (status != PMIC_ST_SUCCESS) {
        return status;
    }

    if (wdgErrStat == NULL) {
        return PMIC_ST_ERR_NULL_PARAM;
    }

    if (wdgErrStat->validParams == 0U) {
        return PMIC_ST_ERR_INV_PARAM;
    }

    if (Pmic_validParamCheck(wdgErrStat->validParams, PMIC_WDG_RST_INT_VALID)) {
        Pmic_setBitFieldByName(&regData, WD_RST_INT, 1U);
    }

    if (Pmic_validParamCheck(wdgErrStat->validParams, PMIC_WDG_FAIL_INT_VALID)) {
        Pmic_setBitFieldByName(&regData, WD_FAIL_INT, 1U);
    }

    if (Pmic_validParamCheck(wdgErrStat->validParams, PMIC_WDG_ANSW_ERR_VALID)) {
        Pmic_setBitFieldByName(&regData, WD_ANSW_ERR, 1U);
    }

    if (Pmic_validParamCheck(wdgErrStat->validParams, PMIC_WDG_SEQ_ERR_VALID)) {
        Pmic_setBitFieldByName(&regData, WD_SEQ_ERR, 1U);
    }

    if (Pmic_validParamCheck(wdgErrStat->validParams, PMIC_WDG_ANSW_EARLY_ERR_VALID)) {
        Pmic_setBitFieldByName(&regData, WD_ANSW_EARLY, 1U);
    }

    if (Pmic_validParamCheck(wdgErrStat->validParams, PMIC_WDG_TIMEOUT_ERR_VALID)) {
        Pmic_setBitFieldByName(&regData, WD_TIMEOUT, 1U);
    }

    if (Pmic_validParamCheck(wdgErrStat->validParams, PMIC_WDG_LONG_WIN_TIMEOUT_INT_VALID)) {
        Pmic_setBitFieldByName(&regData, WD_LONGWIN_TIMEOUT_INT, 1U);
    }

    return Pmic_ioTxByte_CS(handle, WD_ERR_STATUS_REGADDR, regData);
}

int32_t Pmic_wdgClrErrStatAll(const Pmic_Handle_t *handle) {
    int32_t status = Pmic_checkPmicHandle(handle);
    if (status != PMIC_ST_SUCCESS) {
        return status;
    }

    return Pmic_ioTxByte_CS(handle, WD_ERR_STATUS_REGADDR, 0xFFU);
}

int32_t Pmic_wdgGetErrStat(const Pmic_Handle_t *handle, Pmic_WdgErrStat_t *wdgErrStat) {
    uint8_t regData = 0U;

    int32_t status = Pmic_checkPmicHandle(handle);
    if (status != PMIC_ST_SUCCESS) {
        return status;
    }

    if (wdgErrStat == NULL) {
        return PMIC_ST_ERR_NULL_PARAM;
    }

    if (wdgErrStat->validParams == 0U) {
        return PMIC_ST_ERR_INV_PARAM;
    }

    status = Pmic_ioRxByte_CS(handle, WD_ERR_STATUS_REGADDR, &regData);
    if (status != PMIC_ST_SUCCESS) {
        return status;
    }

    if (Pmic_validParamCheck(wdgErrStat->validParams, PMIC_WDG_RST_INT_VALID)) {
        wdgErrStat->rstInt = Pmic_getBitField_b(regData, WD_RST_INT_SHIFT);
    }

    if (Pmic_validParamCheck(wdgErrStat->validParams, PMIC_WDG_FAIL_INT_VALID)) {
        wdgErrStat->failInt = Pmic_getBitField_b(regData, WD_FAIL_INT_SHIFT);
    }

    if (Pmic_validParamCheck(wdgErrStat->validParams, PMIC_WDG_ANSW_ERR_VALID)) {
        wdgErrStat->answErr = Pmic_getBitField_b(regData, WD_ANSW_ERR_SHIFT);
    }

    if (Pmic_validParamCheck(wdgErrStat->validParams, PMIC_WDG_SEQ_ERR_VALID)) {
        wdgErrStat->seqErr = Pmic_getBitField_b(regData, WD_SEQ_ERR_SHIFT);
    }

    if (Pmic_validParamCheck(wdgErrStat->validParams, PMIC_WDG_ANSW_EARLY_ERR_VALID)) {
        wdgErrStat->answEarlyErr = Pmic_getBitField_b(regData, WD_ANSW_EARLY_SHIFT);
    }

    if (Pmic_validParamCheck(wdgErrStat->validParams, PMIC_WDG_TIMEOUT_ERR_VALID)) {
        wdgErrStat->timeoutErr = Pmic_getBitField_b(regData, WD_TIMEOUT_SHIFT);
    }

    if (Pmic_validParamCheck(wdgErrStat->validParams, PMIC_WDG_LONG_WIN_TIMEOUT_INT_VALID)) {
        wdgErrStat->longWinTimeoutInt = Pmic_getBitField_b(regData, WD_LONGWIN_TIMEOUT_INT_SHIFT);
    }

    return PMIC_ST_SUCCESS;
}

int32_t Pmic_wdgGetFailCntStat(const Pmic_Handle_t *handle, Pmic_WdgFailCntStat_t *wdgFailCntStat) {
    uint8_t regData = 0U;

    int32_t status = Pmic_checkPmicHandle(handle);
    if (status != PMIC_ST_SUCCESS) {
        return status;
    }

    status = Pmic_ioRxByte_CS(handle, WD_FAIL_CNT_REG_REGADDR, &regData);
    if (status != PMIC_ST_SUCCESS) {
        return status;
    }

    if (Pmic_validParamCheck(wdgFailCntStat->validParams, PMIC_WDG_BAD_EVENT_VALID)) {
        wdgFailCntStat->badEvent = Pmic_getBitField_b(regData, WD_BAD_EVENT_SHIFT);
    }

    if (Pmic_validParamCheck(wdgFailCntStat->validParams, PMIC_WDG_GOOD_EVENT_VALID)) {
        wdgFailCntStat->goodEvent = Pmic_getBitField_b(regData, WD_FIRST_OK_SHIFT);
    }

    if (Pmic_validParamCheck(wdgFailCntStat->validParams, PMIC_WDG_FAIL_CNT_VALID)) {
        wdgFailCntStat->failCnt = Pmic_getBitField(regData, WD_FAIL_CNT_SHIFT, WD_FAIL_CNT_MASK);;
    }

    return PMIC_ST_SUCCESS;
}

static uint8_t mux_4x1(uint8_t x0, uint8_t x1, uint8_t x2, uint8_t x3, uint8_t qaFdbk)
{
    uint8_t y = 0U;

    switch (qaFdbk)
    {
        case 0U:
            y = x0;
            break;
        case 1U:
            y = x1;
            break;
        case 2U:
            y = x2;
            break;
        default:
            y = x3;
            break;
    }

    return y;
}

static uint8_t WDG_getAnswerByte(uint8_t question, uint8_t qaAnsCnt, uint8_t qaFdbk)
{
    uint8_t q0 = 0U, q1 = 0U, q2 = 0U, q3 = 0U;
    uint8_t a0 = 0U, a1 = 0U;
    uint8_t qaAns = 0U;

    q0 = ((question >> 0U) & 1U);
    q1 = ((question >> 1U) & 1U);
    q2 = ((question >> 2U) & 1U);
    q3 = ((question >> 3U) & 1U);

    a0 = ((qaAnsCnt >> 0U) & 1U);
    a1 = ((qaAnsCnt >> 1U) & 1U);

    /* Reference-Answer-X[0] */
    qaAns = (mux_4x1(q0, q1, q2, q3, qaFdbk) ^ (mux_4x1(q3, q2, q1, q0, qaFdbk) ^ a1));
    /* Reference-Answer-X[1] */
    qaAns |= ((mux_4x1(q0, q1, q2, q3, qaFdbk) ^ (mux_4x1(q2, q1, q0, q3, qaFdbk) ^ q1) ^ a1) << 1U);
    /* Reference-Answer-X[2] */
    qaAns |= ((mux_4x1(q0, q3, q1, q1, qaFdbk) ^ (mux_4x1(q3, q2, q1, q0, qaFdbk) ^ q1) ^ a1) << 2U);
    /* Reference-Answer-X[3] */
    qaAns |= ((mux_4x1(q2, q1, q0, q3, qaFdbk) ^ (mux_4x1(q0, q3, q2, q1, qaFdbk) ^ q3) ^ a1) << 3U);
    /* Reference-Answer-X[4] */
    qaAns |= ((mux_4x1(q1, q0, q2, q3, qaFdbk) ^ a0) << 4U);
    /* Reference-Answer-X[5] */
    qaAns |= ((mux_4x1(q3, q2, q1, q0, qaFdbk) ^ a0) << 5U);
    /* Reference-Answer-X[6] */
    qaAns |= ((mux_4x1(q0, q3, q2, q1, qaFdbk) ^ a0) << 6U);
    /* Reference-Answer-X[7] */
    qaAns |= ((mux_4x1(q2, q1, q0, q3, qaFdbk) ^ a0) << 7U);

    return qaAns;
}

int32_t Pmic_wdgQaSequenceWriteAnswer(const Pmic_Handle_t *handle) {
    uint8_t regData = 0U, qaFdbk = 0U, qaAnsCnt = 0U, question = 0U;

    int32_t status = Pmic_checkPmicHandle(handle);
    if (status != PMIC_ST_SUCCESS) {
        return status;
    }

    status = Pmic_ioRxByte_CS(handle, WD_QA_CFG_REGADDR, &regData);
    if (status != PMIC_ST_SUCCESS) {
        return status;
    }

    qaFdbk = Pmic_getBitField(regData, WD_QA_FDBK_SHIFT, WD_QA_FDBK_MASK);

    status = Pmic_ioRxByte_CS(handle, WD_QUESTION_ANSW_CNT_REGADDR, &regData);
    if (status != PMIC_ST_SUCCESS) {
        return status;
    }

    qaAnsCnt = Pmic_getBitField(regData, WD_ANSW_CNT_SHIFT, WD_ANSW_CNT_MASK);
    question = Pmic_getBitField(regData, WD_QUESTION_SHIFT, WD_QUESTION_MASK);

    regData = WDG_getAnswerByte(question, qaAnsCnt, qaFdbk);
    return Pmic_ioTxByte_CS(handle, WD_ANSWER_REG_REGADDR, regData);
}

int32_t Pmic_wdgGetFdbkRegData(const Pmic_Handle_t *handle, uint8_t *regData) {
    int32_t status = Pmic_checkPmicHandle(handle);
    if (status != PMIC_ST_SUCCESS) {
        return status;
    }

    if (regData == NULL) {
        return PMIC_ST_ERR_NULL_PARAM;
    }

    return Pmic_ioRxByte_CS(handle, WD_QA_CFG_REGADDR, regData);
}

int32_t Pmic_wdgExtractFdbk(uint8_t regData, Pmic_WdgAnsInfo_t *wdgAnsInfo) {
    if (wdgAnsInfo == NULL) {
        return PMIC_ST_ERR_NULL_PARAM;
    }

    wdgAnsInfo->fdbk = Pmic_getBitField(regData, WD_QA_FDBK_SHIFT, WD_QA_FDBK_MASK);
    return PMIC_ST_SUCCESS;
}

int32_t Pmic_wdgGetAnsCntAndQuesRegData(const Pmic_Handle_t *handle, uint8_t *regData) {
    int32_t status = Pmic_checkPmicHandle(handle);
    if (status != PMIC_ST_SUCCESS) {
        return status;
    }

    if (regData == NULL) {
        return PMIC_ST_ERR_NULL_PARAM;
    }

    return Pmic_ioRxByte_CS(handle, WD_QUESTION_ANSW_CNT_REGADDR, regData);
}

int32_t Pmic_wdgExtractAnsCntAndQues(const Pmic_Handle_t *handle, uint8_t regData, Pmic_WdgAnsInfo_t *wdgAnsInfo) {
    int32_t status = Pmic_checkPmicHandle(handle);
    if (status != PMIC_ST_SUCCESS) {
        return status;
    }

    if (wdgAnsInfo == NULL) {
        return PMIC_ST_ERR_NULL_PARAM;
    }

    wdgAnsInfo->ansCnt = Pmic_getBitField(regData, WD_ANSW_CNT_SHIFT, WD_ANSW_CNT_MASK);
    wdgAnsInfo->question = Pmic_getBitField(regData, WD_QUESTION_SHIFT, WD_QUESTION_MASK);
    return PMIC_ST_SUCCESS;
}

int32_t Pmic_wdgWriteAnswer(const Pmic_Handle_t *handle, const Pmic_WdgAnsInfo_t *wdgAnsInfo) {
    uint8_t regData = 0U;

    int32_t status = Pmic_checkPmicHandle(handle);
    if (status != PMIC_ST_SUCCESS) {
        return status;
    }

    if (wdgAnsInfo == NULL) {
        return PMIC_ST_ERR_NULL_PARAM;
    }

    regData = WDG_getAnswerByte(wdgAnsInfo->question, wdgAnsInfo->ansCnt, wdgAnsInfo->fdbk);
    return Pmic_ioTxByte_CS(handle, WD_ANSWER_REG_REGADDR, regData);
}
