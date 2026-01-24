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
/**
 * @file pmic_wdg.c
 *
 * @brief PMIC LLD watchdog module implementation for TPS6522x-Q1.
 */

#include <stdint.h>

#include "pmic.h"
#include "pmic_wdg.h"
#include "pmic_io.h"
#include "regmap/wdg.h"

#include <string.h>

#define CLEAR_ALL_STAT_BITS (0xFFU)

/* ========================================================================== */
/*                           Internal Helper Functions                        */
/* ========================================================================== */

/**
 * @brief Copy Pmic_WdgCfg_t structure member-wise
 */
static inline void WDG_copyWdgCfg(const Pmic_WdgCfg_t *src, Pmic_WdgCfg_t *dst)
{
    (void)memmove((void *)dst, (const void *)src, sizeof(Pmic_WdgCfg_t));
}

/**
 * @brief Copy Pmic_WdgErrStatus_t structure member-wise
 */
static inline void WDG_copyWdgErrStatus(const Pmic_WdgErrStatus_t *src, Pmic_WdgErrStatus_t *dst)
{
    (void)memmove((void *)dst, (const void *)src, sizeof(Pmic_WdgErrStatus_t));
}

/**
 * @brief Copy Pmic_WdgFailCntStatus_t structure member-wise
 */
static inline void WDG_copyWdgFailCntStatus(const Pmic_WdgFailCntStatus_t *src, Pmic_WdgFailCntStatus_t *dst)
{
    (void)memmove((void *)dst, (const void *)src, sizeof(Pmic_WdgFailCntStatus_t));
}

/* ========================================================================== */
/*                           Internal Helper Functions                        */
/* ========================================================================== */

/**
 * @brief 4-to-1 multiplexer for WDG Q&A answer calculation
 *
 * @param x0 Input bit 0
 * @param x1 Input bit 1
 * @param x2 Input bit 2
 * @param x3 Input bit 3
 * @param qaFdbk Selector (0-3) to choose which input to return
 *
 * @return Selected input bit
 */
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

/**
 * @brief Calculate WDG Q&A answer byte using Markov chain combinational logic
 *
 * @param question 4-bit question value from WD_QUESTION register
 * @param qaAnsCnt 2-bit answer count from WD_ANSW_CNT register
 * @param qaFdbk 2-bit feedback configuration from WD_QA_FDBK register
 * @param answer Pointer to output: 8-bit answer byte to write to WD_ANSWER register
 */
static void WDG_getAnswerByte(uint8_t question, uint8_t qaAnsCnt, uint8_t qaFdbk, uint8_t *answer)
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

    *answer = qaAns;
}

/**
 * @brief Validate watchdog configuration parameters
 */
static int32_t Pmic_wdgValidateCfg(const Pmic_WdgCfg_t *wdgCfg)
{
    int32_t status = PMIC_ST_SUCCESS;

    if (Pmic_validParamCheck(wdgCfg->validParams, PMIC_WDG_MODE_SEL_VALID))
    {
        if (wdgCfg->mode > PMIC_WDG_MODE_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
    }

    if (Pmic_validParamCheck(wdgCfg->validParams, PMIC_WDG_WIN1_CODE_VALID))
    {
        if (wdgCfg->win1Code > PMIC_WDG_WIN1_CODE_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
    }

    if (Pmic_validParamCheck(wdgCfg->validParams, PMIC_WDG_WIN2_CODE_VALID))
    {
        if (wdgCfg->win2Code > PMIC_WDG_WIN2_CODE_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
    }

    if (Pmic_validParamCheck(wdgCfg->validParams, PMIC_WDG_QA_FDBK_VALID))
    {
        if (wdgCfg->qaFdbk > PMIC_WDG_QA_FDBK_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
    }

    if (Pmic_validParamCheck(wdgCfg->validParams, PMIC_WDG_QA_LFSR_VALID))
    {
        if (wdgCfg->qaLfsr > PMIC_WDG_QA_LFSR_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
    }

    if (Pmic_validParamCheck(wdgCfg->validParams, PMIC_WDG_QA_SEED_VALID))
    {
        if (wdgCfg->qaSeed > PMIC_WDG_QA_SEED_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
    }

    if (Pmic_validParamCheck(wdgCfg->validParams, PMIC_WDG_FAIL_THR_VALID))
    {
        if (wdgCfg->failThr > PMIC_WDG_FAIL_THR_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
    }

    if (Pmic_validParamCheck(wdgCfg->validParams, PMIC_WDG_RST_THR_VALID))
    {
        if (wdgCfg->rstThr > PMIC_WDG_RST_THR_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
    }

    if (Pmic_validParamCheck(wdgCfg->validParams, PMIC_WDG_CNT_SEL_VALID))
    {
        if (wdgCfg->cntSel > PMIC_WDG_CNT_SEL_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
    }

    return status;
}

/**
 * @brief Configure WD_MODE_REG register
 */
static int32_t Pmic_wdgSetModeReg(const Pmic_Handle_t *handle, const Pmic_WdgCfg_t *wdgCfg)
{
    int32_t status;
    uint8_t regData = 0U;

    Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);
    status = Pmic_ioRxByte(handle, WD_MODE_REG_REG, &regData);

    if (status == PMIC_ST_SUCCESS)
    {
        if (Pmic_validParamCheck(wdgCfg->validParams, PMIC_WDG_MODE_SEL_VALID))
        {
            Pmic_setBitField_b(&regData, WD_MODE_SELECT_SHIFT, WD_MODE_SELECT_MASK, (bool)wdgCfg->mode);
        }

        if (Pmic_validParamCheck(wdgCfg->validParams, PMIC_WDG_CNT_SEL_VALID))
        {
            Pmic_setBitField_b(&regData, WD_CNT_SEL_SHIFT, WD_CNT_SEL_MASK, (bool)wdgCfg->cntSel);
        }

        if (Pmic_validParamCheck(wdgCfg->validParams, PMIC_WDG_EN_DRV_SEL_VALID))
        {
            Pmic_setBitField_b(&regData, WD_ENDRV_SEL_SHIFT, WD_ENDRV_SEL_MASK, wdgCfg->clrEnDrvOnFailInt);
        }

        status = Pmic_ioTxByte(handle, WD_MODE_REG_REG, regData);
    }
    Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);

    return status;
}

/**
 * @brief Configure WD_QA_CFG register
 */
static int32_t Pmic_wdgSetQaCfg(const Pmic_Handle_t *handle, const Pmic_WdgCfg_t *wdgCfg)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);
    if (Pmic_validParamCheck(wdgCfg->validParams, PMIC_WDG_QA_FDBK_VALID) ||
        Pmic_validParamCheck(wdgCfg->validParams, PMIC_WDG_QA_LFSR_VALID) ||
        Pmic_validParamCheck(wdgCfg->validParams, PMIC_WDG_QA_SEED_VALID))
    {
        status = Pmic_ioRxByte(handle, WD_QA_CFG_REG, &regData);

        if (status == PMIC_ST_SUCCESS)
        {
            if (Pmic_validParamCheck(wdgCfg->validParams, PMIC_WDG_QA_FDBK_VALID))
            {
                Pmic_setBitField(&regData, WD_QA_FDBK_SHIFT, WD_QA_FDBK_MASK, wdgCfg->qaFdbk);
            }

            if (Pmic_validParamCheck(wdgCfg->validParams, PMIC_WDG_QA_LFSR_VALID))
            {
                Pmic_setBitField(&regData, WD_QA_LFSR_SHIFT, WD_QA_LFSR_MASK, wdgCfg->qaLfsr);
            }

            if (Pmic_validParamCheck(wdgCfg->validParams, PMIC_WDG_QA_SEED_VALID))
            {
                Pmic_setBitField(&regData, WD_QUESTION_SEED_SHIFT, WD_QUESTION_SEED_MASK, wdgCfg->qaSeed);
            }

            status = Pmic_ioTxByte(handle, WD_QA_CFG_REG, regData);
        }
    }
    Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);

    return status;
}

/**
 * @brief Configure WD_THR_CFG register
 */
static int32_t Pmic_wdgSetThrCfg(const Pmic_Handle_t *handle, const Pmic_WdgCfg_t *wdgCfg)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);
    if (Pmic_validParamCheck(wdgCfg->validParams, PMIC_WDG_RST_EN_VALID) ||
        Pmic_validParamCheck(wdgCfg->validParams, PMIC_WDG_FAIL_THR_VALID) ||
        Pmic_validParamCheck(wdgCfg->validParams, PMIC_WDG_RST_THR_VALID))
    {
        status = Pmic_ioRxByte(handle, WD_THR_CFG_REG, &regData);

        if (status == PMIC_ST_SUCCESS)
        {
            if (Pmic_validParamCheck(wdgCfg->validParams, PMIC_WDG_RST_EN_VALID))
            {
                Pmic_setBitField_b(&regData, WD_RST_EN_SHIFT, WD_RST_EN_MASK, wdgCfg->rstEn);
            }

            if (Pmic_validParamCheck(wdgCfg->validParams, PMIC_WDG_FAIL_THR_VALID))
            {
                Pmic_setBitField(&regData, WD_FAIL_TH_SHIFT, WD_FAIL_TH_MASK, wdgCfg->failThr);
            }

            if (Pmic_validParamCheck(wdgCfg->validParams, PMIC_WDG_RST_THR_VALID))
            {
                Pmic_setBitField(&regData, WD_RST_TH_SHIFT, WD_RST_TH_MASK, wdgCfg->rstThr);
            }

            status = Pmic_ioTxByte(handle, WD_THR_CFG_REG, regData);
        }
    }
    Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);

    return status;
}

/**
 * @brief Read WD_MODE_REG register
 */
static int32_t Pmic_wdgGetModeReg(const Pmic_Handle_t *handle, Pmic_WdgCfg_t *wdgCfg)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    if (Pmic_validParamCheck(wdgCfg->validParams, PMIC_WDG_MODE_SEL_VALID) ||
        Pmic_validParamCheck(wdgCfg->validParams, PMIC_WDG_CNT_SEL_VALID) ||
        Pmic_validParamCheck(wdgCfg->validParams, PMIC_WDG_EN_DRV_SEL_VALID))
    {
        status = Pmic_ioRxByte_CS(handle, WD_MODE_REG_REG, &regData);

        if (status == PMIC_ST_SUCCESS)
        {
            if (Pmic_validParamCheck(wdgCfg->validParams, PMIC_WDG_MODE_SEL_VALID))
            {
                wdgCfg->mode = (Pmic_getBitField_b(regData, WD_MODE_SELECT_SHIFT) != false) ? 1U : 0U;
            }

            if (Pmic_validParamCheck(wdgCfg->validParams, PMIC_WDG_CNT_SEL_VALID))
            {
                wdgCfg->cntSel = (Pmic_getBitField_b(regData, WD_CNT_SEL_SHIFT) != false) ? 1U : 0U;
            }

            if (Pmic_validParamCheck(wdgCfg->validParams, PMIC_WDG_EN_DRV_SEL_VALID))
            {
                wdgCfg->clrEnDrvOnFailInt = Pmic_getBitField_b(regData, WD_ENDRV_SEL_SHIFT);
            }
        }
    }

    return status;
}

/**
 * @brief Read WD_QA_CFG register
 */
static int32_t Pmic_wdgGetQaCfg(const Pmic_Handle_t *handle, Pmic_WdgCfg_t *wdgCfg)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    if (Pmic_validParamCheck(wdgCfg->validParams, PMIC_WDG_QA_FDBK_VALID) ||
        Pmic_validParamCheck(wdgCfg->validParams, PMIC_WDG_QA_LFSR_VALID) ||
        Pmic_validParamCheck(wdgCfg->validParams, PMIC_WDG_QA_SEED_VALID))
    {
        status = Pmic_ioRxByte_CS(handle, WD_QA_CFG_REG, &regData);

        if (status == PMIC_ST_SUCCESS)
        {
            if (Pmic_validParamCheck(wdgCfg->validParams, PMIC_WDG_QA_FDBK_VALID))
            {
                wdgCfg->qaFdbk = Pmic_getBitField(regData, WD_QA_FDBK_SHIFT, WD_QA_FDBK_MASK);
            }

            if (Pmic_validParamCheck(wdgCfg->validParams, PMIC_WDG_QA_LFSR_VALID))
            {
                wdgCfg->qaLfsr = Pmic_getBitField(regData, WD_QA_LFSR_SHIFT, WD_QA_LFSR_MASK);
            }

            if (Pmic_validParamCheck(wdgCfg->validParams, PMIC_WDG_QA_SEED_VALID))
            {
                wdgCfg->qaSeed = Pmic_getBitField(regData, WD_QUESTION_SEED_SHIFT, WD_QUESTION_SEED_MASK);
            }
        }
    }

    return status;
}

/**
 * @brief Read WD_THR_CFG register
 */
static int32_t Pmic_wdgGetThrCfg(const Pmic_Handle_t *handle, Pmic_WdgCfg_t *wdgCfg)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    if (Pmic_validParamCheck(wdgCfg->validParams, PMIC_WDG_RST_EN_VALID) ||
        Pmic_validParamCheck(wdgCfg->validParams, PMIC_WDG_FAIL_THR_VALID) ||
        Pmic_validParamCheck(wdgCfg->validParams, PMIC_WDG_RST_THR_VALID))
    {
        status = Pmic_ioRxByte_CS(handle, WD_THR_CFG_REG, &regData);

        if (status == PMIC_ST_SUCCESS)
        {
            if (Pmic_validParamCheck(wdgCfg->validParams, PMIC_WDG_RST_EN_VALID))
            {
                wdgCfg->rstEn = Pmic_getBitField_b(regData, WD_RST_EN_SHIFT);
            }

            if (Pmic_validParamCheck(wdgCfg->validParams, PMIC_WDG_FAIL_THR_VALID))
            {
                wdgCfg->failThr = Pmic_getBitField(regData, WD_FAIL_TH_SHIFT, WD_FAIL_TH_MASK);
            }

            if (Pmic_validParamCheck(wdgCfg->validParams, PMIC_WDG_RST_THR_VALID))
            {
                wdgCfg->rstThr = Pmic_getBitField(regData, WD_RST_TH_SHIFT, WD_RST_TH_MASK);
            }
        }
    }

    return status;
}

/* ========================================================================== */
/*                         Public API Implementations                         */
/* ========================================================================== */

int32_t Pmic_wdgSetEnableState(const Pmic_Handle_t *handle, bool enable)
{
    int32_t status = Pmic_checkHandle(handle);

    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioUpdateByte_bCS(handle, WD_THR_CFG_REG, WD_EN_SHIFT, enable);
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_wdgGetEnableState(const Pmic_Handle_t *handle, bool *isEnabled)
{
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    if ((status == PMIC_ST_SUCCESS) && (isEnabled == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioRxByte_CS(handle, WD_THR_CFG_REG, &regData);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        *isEnabled = Pmic_getBitField_b(regData, WD_EN_SHIFT);
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_wdgSetCfg(const Pmic_Handle_t *handle, const Pmic_WdgCfg_t *wdgCfg)
{
    Pmic_WdgCfg_t wdgCfgLocal;
    int32_t status = Pmic_checkHandle(handle);

    if ((status == PMIC_ST_SUCCESS) && (wdgCfg == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        WDG_copyWdgCfg(wdgCfg, &wdgCfgLocal);
        status = Pmic_wdgValidateCfg(&wdgCfgLocal);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_wdgSetModeReg(handle, &wdgCfgLocal);
    }

    if ((status == PMIC_ST_SUCCESS) && Pmic_validParamCheck(wdgCfgLocal.validParams, PMIC_WDG_WIN1_CODE_VALID))
    {
        status = Pmic_ioUpdateByte_CS(handle, WD_WIN1_CFG_REG, WD_WIN_SHIFT, WD_WIN_MASK, wdgCfgLocal.win1Code);
    }

    if ((status == PMIC_ST_SUCCESS) && Pmic_validParamCheck(wdgCfgLocal.validParams, PMIC_WDG_WIN2_CODE_VALID))
    {
        status = Pmic_ioUpdateByte_CS(handle, WD_WIN2_CFG_REG, WD_WIN_SHIFT, WD_WIN_MASK, wdgCfgLocal.win2Code);
    }

    if ((status == PMIC_ST_SUCCESS) && Pmic_validParamCheck(wdgCfgLocal.validParams, PMIC_WDG_LONG_WIN_CODE_VALID))
    {
        status = Pmic_ioTxByte_CS(handle, WD_LONGWIN_CFG_REG, wdgCfgLocal.longWinCode);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_wdgSetQaCfg(handle, &wdgCfgLocal);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_wdgSetThrCfg(handle, &wdgCfgLocal);
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_wdgGetCfg(const Pmic_Handle_t *handle, Pmic_WdgCfg_t *wdgCfg)
{
    Pmic_WdgCfg_t wdgCfgLocal;
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    if ((status == PMIC_ST_SUCCESS) && (wdgCfg == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        WDG_copyWdgCfg(wdgCfg, &wdgCfgLocal);
        status = Pmic_wdgGetModeReg(handle, &wdgCfgLocal);
    }

    if ((status == PMIC_ST_SUCCESS) && Pmic_validParamCheck(wdgCfgLocal.validParams, PMIC_WDG_WIN1_CODE_VALID))
    {
        status = Pmic_ioRxByte_CS(handle, WD_WIN1_CFG_REG, &regData);
        if (status == PMIC_ST_SUCCESS)
        {
            wdgCfgLocal.win1Code = Pmic_getBitField(regData, WD_WIN_SHIFT, WD_WIN_MASK);
        }
    }

    if ((status == PMIC_ST_SUCCESS) && Pmic_validParamCheck(wdgCfgLocal.validParams, PMIC_WDG_WIN2_CODE_VALID))
    {
        status = Pmic_ioRxByte_CS(handle, WD_WIN2_CFG_REG, &regData);
        if (status == PMIC_ST_SUCCESS)
        {
            wdgCfgLocal.win2Code = Pmic_getBitField(regData, WD_WIN_SHIFT, WD_WIN_MASK);
        }
    }

    if ((status == PMIC_ST_SUCCESS) && Pmic_validParamCheck(wdgCfgLocal.validParams, PMIC_WDG_LONG_WIN_CODE_VALID))
    {
        status = Pmic_ioRxByte_CS(handle, WD_LONGWIN_CFG_REG, &wdgCfgLocal.longWinCode);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_wdgGetQaCfg(handle, &wdgCfgLocal);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_wdgGetThrCfg(handle, &wdgCfgLocal);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        WDG_copyWdgCfg(&wdgCfgLocal, wdgCfg);
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_wdgSetPowerHold(const Pmic_Handle_t *handle, bool enable)
{
    int32_t status = Pmic_checkHandle(handle);

    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioUpdateByte_bCS(handle, WD_MODE_REG_REG, WD_PWRHOLD_SHIFT, enable);
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_wdgGetPowerHold(const Pmic_Handle_t *handle, bool *isEnabled)
{
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    if ((status == PMIC_ST_SUCCESS) && (isEnabled == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioRxByte_CS(handle, WD_MODE_REG_REG, &regData);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        *isEnabled = Pmic_getBitField_b(regData, WD_PWRHOLD_SHIFT);
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_wdgSetReturnToLongWindow(const Pmic_Handle_t *handle, bool enable)
{
    int32_t status = Pmic_checkHandle(handle);

    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioUpdateByte_bCS(handle, WD_MODE_REG_REG, WD_RETURN_LONGWIN_SHIFT, enable);
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_wdgGetReturnToLongWindow(const Pmic_Handle_t *handle, bool *isEnabled)
{
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    if ((status == PMIC_ST_SUCCESS) && (isEnabled == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioRxByte_CS(handle, WD_MODE_REG_REG, &regData);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        *isEnabled = Pmic_getBitField_b(regData, WD_RETURN_LONGWIN_SHIFT);
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_wdgQaWriteAnswer(const Pmic_Handle_t *handle)
{
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;
    uint8_t qaFdbk = 0U;
    uint8_t qaAnsCnt = 0U;
    uint8_t question = 0U;

    // Read Q&A feedback configuration from WD_QA_CFG register
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioRxByte_CS(handle, WD_QA_CFG_REG, &regData);

        if (status == PMIC_ST_SUCCESS)
        {
            qaFdbk = Pmic_getBitField(regData, WD_QA_FDBK_SHIFT, WD_QA_FDBK_MASK);
        }
    }

    // Read question and answer count from WD_QUESTION_ANSW_CNT register
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioRxByte_CS(handle, WD_QUESTION_ANSW_CNT_REG, &regData);

        if (status == PMIC_ST_SUCCESS)
        {
            qaAnsCnt = Pmic_getBitField(regData, WD_ANSW_CNT_SHIFT, WD_ANSW_CNT_MASK);
            question = Pmic_getBitField(regData, WD_QUESTION_SHIFT, WD_QUESTION_MASK);
        }
    }

    // Calculate Q&A answer byte and write to WD_ANSWER register
    if (status == PMIC_ST_SUCCESS)
    {
        uint8_t answerByte = 0U;
        WDG_getAnswerByte(question, qaAnsCnt, qaFdbk, &answerByte);
        status = Pmic_ioTxByte_CS(handle, WD_ANSWER_REG_REG, answerByte);
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_wdgClrErrStatus(const Pmic_Handle_t *handle, const Pmic_WdgErrStatus_t *wdgErrStatus)
{
    Pmic_WdgErrStatus_t wdgErrStatusLocal;
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    if ((status == PMIC_ST_SUCCESS) && (wdgErrStatus == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        WDG_copyWdgErrStatus(wdgErrStatus, &wdgErrStatusLocal);

        // Build the clear mask from validParams (write-1-to-clear)
        if (Pmic_validParamCheck(wdgErrStatusLocal.validParams, PMIC_WDG_RST_INT_VALID))
        {
            Pmic_setBitField_b(&regData, WD_RST_INT_SHIFT, WD_RST_INT_MASK, true);
        }

        if (Pmic_validParamCheck(wdgErrStatusLocal.validParams, PMIC_WDG_FAIL_INT_VALID))
        {
            Pmic_setBitField_b(&regData, WD_FAIL_INT_SHIFT, WD_FAIL_INT_MASK, true);
        }

        if (Pmic_validParamCheck(wdgErrStatusLocal.validParams, PMIC_WDG_ANSW_ERR_VALID))
        {
            Pmic_setBitField_b(&regData, WD_ANSW_ERR_SHIFT, WD_ANSW_ERR_MASK, true);
        }

        if (Pmic_validParamCheck(wdgErrStatusLocal.validParams, PMIC_WDG_SEQ_ERR_VALID))
        {
            Pmic_setBitField_b(&regData, WD_SEQ_ERR_SHIFT, WD_SEQ_ERR_MASK, true);
        }

        if (Pmic_validParamCheck(wdgErrStatusLocal.validParams, PMIC_WDG_ANSW_EARLY_ERR_VALID))
        {
            Pmic_setBitField_b(&regData, WD_ANSW_EARLY_SHIFT, WD_ANSW_EARLY_MASK, true);
        }

        if (Pmic_validParamCheck(wdgErrStatusLocal.validParams, PMIC_WDG_TRIG_EARLY_ERR_VALID))
        {
            Pmic_setBitField_b(&regData, WD_TRIG_EARLY_SHIFT, WD_TRIG_EARLY_MASK, true);
        }

        if (Pmic_validParamCheck(wdgErrStatusLocal.validParams, PMIC_WDG_TIMEOUT_ERR_VALID))
        {
            Pmic_setBitField_b(&regData, WD_TIMEOUT_SHIFT, WD_TIMEOUT_MASK, true);
        }

        if (Pmic_validParamCheck(wdgErrStatusLocal.validParams, PMIC_WDG_LONG_WIN_TIMEOUT_ERR_VALID))
        {
            Pmic_setBitField_b(&regData, WD_LONGWIN_TIMEOUT_INT_SHIFT, WD_LONGWIN_TIMEOUT_INT_MASK, true);
        }

        status = Pmic_ioTxByte_CS(handle, WD_ERR_STATUS_REG, regData);
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_wdgClrErrStatusAll(const Pmic_Handle_t *handle)
{
    int32_t status = Pmic_checkHandle(handle);

    if (status == PMIC_ST_SUCCESS)
    {
        // Write all 1s to clear all error status bits (write-1-to-clear)
        status = Pmic_ioTxByte_CS(handle, WD_ERR_STATUS_REG, CLEAR_ALL_STAT_BITS);
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_wdgGetErrStatus(const Pmic_Handle_t *handle, Pmic_WdgErrStatus_t *wdgErrStatus)
{
    Pmic_WdgErrStatus_t wdgErrStatusLocal;
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    if ((status == PMIC_ST_SUCCESS) && (wdgErrStatus == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        WDG_copyWdgErrStatus(wdgErrStatus, &wdgErrStatusLocal);
        status = Pmic_ioRxByte_CS(handle, WD_ERR_STATUS_REG, &regData);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        if (Pmic_validParamCheck(wdgErrStatusLocal.validParams, PMIC_WDG_RST_INT_VALID))
        {
            wdgErrStatusLocal.rstInt = Pmic_getBitField_b(regData, WD_RST_INT_SHIFT);
        }

        if (Pmic_validParamCheck(wdgErrStatusLocal.validParams, PMIC_WDG_FAIL_INT_VALID))
        {
            wdgErrStatusLocal.failInt = Pmic_getBitField_b(regData, WD_FAIL_INT_SHIFT);
        }

        if (Pmic_validParamCheck(wdgErrStatusLocal.validParams, PMIC_WDG_ANSW_ERR_VALID))
        {
            wdgErrStatusLocal.answErr = Pmic_getBitField_b(regData, WD_ANSW_ERR_SHIFT);
        }

        if (Pmic_validParamCheck(wdgErrStatusLocal.validParams, PMIC_WDG_SEQ_ERR_VALID))
        {
            wdgErrStatusLocal.seqErr = Pmic_getBitField_b(regData, WD_SEQ_ERR_SHIFT);
        }

        if (Pmic_validParamCheck(wdgErrStatusLocal.validParams, PMIC_WDG_ANSW_EARLY_ERR_VALID))
        {
            wdgErrStatusLocal.answEarlyErr = Pmic_getBitField_b(regData, WD_ANSW_EARLY_SHIFT);
        }

        if (Pmic_validParamCheck(wdgErrStatusLocal.validParams, PMIC_WDG_TRIG_EARLY_ERR_VALID))
        {
            wdgErrStatusLocal.trigEarlyErr = Pmic_getBitField_b(regData, WD_TRIG_EARLY_SHIFT);
        }

        if (Pmic_validParamCheck(wdgErrStatusLocal.validParams, PMIC_WDG_TIMEOUT_ERR_VALID))
        {
            wdgErrStatusLocal.timeoutErr = Pmic_getBitField_b(regData, WD_TIMEOUT_SHIFT);
        }

        if (Pmic_validParamCheck(wdgErrStatusLocal.validParams, PMIC_WDG_LONG_WIN_TIMEOUT_ERR_VALID))
        {
            wdgErrStatusLocal.longWinTimeoutErr = Pmic_getBitField_b(regData, WD_LONGWIN_TIMEOUT_INT_SHIFT);
        }

        WDG_copyWdgErrStatus(&wdgErrStatusLocal, wdgErrStatus);
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_wdgGetFailCntStatus(const Pmic_Handle_t *handle, Pmic_WdgFailCntStatus_t *wdgFailCntStatus)
{
    Pmic_WdgFailCntStatus_t wdgFailCntStatusLocal;
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    if ((status == PMIC_ST_SUCCESS) && (wdgFailCntStatus == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        WDG_copyWdgFailCntStatus(wdgFailCntStatus, &wdgFailCntStatusLocal);
        status = Pmic_ioRxByte_CS(handle, WD_FAIL_CNT_REG_REG, &regData);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        if (Pmic_validParamCheck(wdgFailCntStatusLocal.validParams, PMIC_WDG_BAD_EVENT_VALID))
        {
            wdgFailCntStatusLocal.badEvent = Pmic_getBitField_b(regData, WD_BAD_EVENT_SHIFT);
        }

        if (Pmic_validParamCheck(wdgFailCntStatusLocal.validParams, PMIC_WDG_GOOD_EVENT_VALID))
        {
            wdgFailCntStatusLocal.goodEvent = Pmic_getBitField_b(regData, WD_FIRST_OK_SHIFT);
        }

        if (Pmic_validParamCheck(wdgFailCntStatusLocal.validParams, PMIC_WDG_FAIL_CNT_VALID))
        {
            wdgFailCntStatusLocal.failCnt = Pmic_getBitField(regData, WD_FAIL_CNT_SHIFT, WD_FAIL_CNT_MASK);
        }

        WDG_copyWdgFailCntStatus(&wdgFailCntStatusLocal, wdgFailCntStatus);
    }

    return Pmic_logStatus(handle, status);
}
