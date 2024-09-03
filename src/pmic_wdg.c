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
 * @file pmic_wdg.c
 *
 * @brief This file contains definitions to APIs that interact with the PMIC
 * watchdog.
 */
#include "pmic.h"
#include "pmic_wdg.h"

#include "pmic_io.h"

#include "regmap/wdg.h"
#include "regmap/core.h"

int32_t Pmic_wdgSendSwTrigger(const Pmic_CoreHandle_t *pmicHandle)
{
    uint8_t regData = 0U;
    int32_t status = Pmic_checkPmicCoreHandle(pmicHandle);

    // Read RECOV_CNT_CONTROL register
    if (status == PMIC_ST_SUCCESS)
    {
        Pmic_criticalSectionStart(pmicHandle);
        status = Pmic_ioRx(pmicHandle, PMIC_RECOV_CNT_CONTROL_REGADDR, &regData);
        Pmic_criticalSectionStop(pmicHandle);
    }

    // Set WD_TRIGGER bit and write new register value back to PMIC
    if (status == PMIC_ST_SUCCESS)
    {
        Pmic_setBitField(&regData, PMIC_WD_TRIGGER_SHIFT, PMIC_WD_TRIGGER_MASK, 1U);

        Pmic_criticalSectionStart(pmicHandle);
        status = Pmic_ioTx(pmicHandle, PMIC_RECOV_CNT_CONTROL_REGADDR, regData);
        Pmic_criticalSectionStop(pmicHandle);
    }

    return status;
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

int32_t Pmic_wdgWriteAnswer(const Pmic_CoreHandle_t *pmicHandle)
{
    uint8_t regData = 0U, qaFdbk = 0U, qaAnsCnt = 0U, question = 0U, intTopStatus = 0U;
    int32_t status = Pmic_checkPmicCoreHandle(pmicHandle);

    // Get the Q&A feedback value
    if (status == PMIC_ST_SUCCESS)
    {
        Pmic_criticalSectionStart(pmicHandle);
        status = Pmic_ioRx(pmicHandle, PMIC_WD_QA_CFG_REGADDR, &regData);
        Pmic_criticalSectionStop(pmicHandle);

        if (status == PMIC_ST_SUCCESS)
        {
            qaFdbk = Pmic_getBitField(regData, PMIC_WD_QA_FDBK_SHIFT, PMIC_WD_QA_FDBK_MASK);
        }
    }

    // Get the Q&A answer count, question, and status of INT_TOP
    if (status == PMIC_ST_SUCCESS)
    {
        Pmic_criticalSectionStart(pmicHandle);
        status = Pmic_ioRx(pmicHandle, PMIC_WD_QUESTION_ANSW_CNT_REGADDR, &regData);
        Pmic_criticalSectionStop(pmicHandle);

        if (status == PMIC_ST_SUCCESS)
        {
            intTopStatus = Pmic_getBitField(regData, PMIC_INT_TOP_STATUS_SHIFT, PMIC_INT_TOP_STATUS_MASK);
            qaAnsCnt = Pmic_getBitField(regData, PMIC_WD_ANSW_CNT_SHIFT, PMIC_WD_ANSW_CNT_MASK);
            question = Pmic_getBitField(regData, PMIC_WD_QUESTION_SHIFT, PMIC_WD_QUESTION_MASK);

            // Call the IRQ response API hook if INT_TOP_STATUS bit is set to 1
            if (intTopStatus != 0U)
            {
                Pmic_irqResponse(pmicHandle);
            }
        }
    }

    // Calculate Q&A answer byte; write the Q&A answer byte to the WD_ANSWER register
    if (status == PMIC_ST_SUCCESS)
    {
        regData = WDG_getAnswerByte(question, qaAnsCnt, qaFdbk);

        Pmic_criticalSectionStart(pmicHandle);
        status = Pmic_ioTx(pmicHandle, PMIC_WD_ANSWER_REG_REGADDR, regData);
        Pmic_criticalSectionStop(pmicHandle);
    }

    return status;
}

int32_t Pmic_wdgSetPwrHold(const Pmic_CoreHandle_t *pmicHandle, bool pwrHold)
{
    uint8_t regData = 0U;
    int32_t status = Pmic_checkPmicCoreHandle(pmicHandle);

    // Read WD_MODE_REG register
    if (status == PMIC_ST_SUCCESS)
    {
        Pmic_criticalSectionStart(pmicHandle);
        status = Pmic_ioRx(pmicHandle, PMIC_WD_MODE_REG_REGADDR, &regData);
        Pmic_criticalSectionStop(pmicHandle);
    }

    // Modify WD_PWRHOLD bit field and write new register value back to PMIC
    if (status == PMIC_ST_SUCCESS)
    {
        Pmic_setBitField_b(&regData, PMIC_WD_PWRHOLD_SHIFT, PMIC_WD_PWRHOLD_MASK, pwrHold);

        Pmic_criticalSectionStart(pmicHandle);
        status = Pmic_ioTx(pmicHandle, PMIC_WD_MODE_REG_REGADDR, regData);
        Pmic_criticalSectionStop(pmicHandle);
    }

    return status;
}

int32_t Pmic_wdgGetPwrHold(const Pmic_CoreHandle_t *pmicHandle, bool *pwrHoldStat)
{
    uint8_t regData = 0U;
    int32_t status = Pmic_checkPmicCoreHandle(pmicHandle);

    if ((status == PMIC_ST_SUCCESS) && (pwrHoldStat == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    // Read WD_MODE_REG register
    if (status == PMIC_ST_SUCCESS)
    {
        Pmic_criticalSectionStart(pmicHandle);
        status = Pmic_ioRx(pmicHandle, PMIC_WD_MODE_REG_REGADDR, &regData);
        Pmic_criticalSectionStop(pmicHandle);
    }

    // Extract WD_PWRHOLD bit field (cast as boolean)
    if (status == PMIC_ST_SUCCESS)
    {
        *pwrHoldStat = Pmic_getBitField_b(regData, PMIC_WD_PWRHOLD_SHIFT);
    }

    return status;
}

int32_t Pmic_wdgSetRetLongWin(const Pmic_CoreHandle_t *pmicHandle, bool retLongWin)
{
    uint8_t regData = 0U;
    int32_t status = Pmic_checkPmicCoreHandle(pmicHandle);

    // Read WD_MODE_REG register
    if (status == PMIC_ST_SUCCESS)
    {
        Pmic_criticalSectionStart(pmicHandle);
        status = Pmic_ioRx(pmicHandle, PMIC_WD_MODE_REG_REGADDR, &regData);
        Pmic_criticalSectionStop(pmicHandle);
    }

    // Modify WD_RETURN_LONGWIN bit field and write new register value back to PMIC
    if (status == PMIC_ST_SUCCESS)
    {
        Pmic_setBitField_b(&regData, PMIC_WD_RETURN_LONGWIN_SHIFT, PMIC_WD_RETURN_LONGWIN_MASK, retLongWin);

        Pmic_criticalSectionStart(pmicHandle);
        status = Pmic_ioTx(pmicHandle, PMIC_WD_MODE_REG_REGADDR, regData);
        Pmic_criticalSectionStop(pmicHandle);
    }

    return status;
}

int32_t Pmic_wdgGetRetLongWin(const Pmic_CoreHandle_t *pmicHandle, bool *retLongWinStat)
{
    uint8_t regData = 0U;
    int32_t status = Pmic_checkPmicCoreHandle(pmicHandle);

    if ((status == PMIC_ST_SUCCESS) && (retLongWinStat == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    // Read WD_MODE_REG register
    if (status == PMIC_ST_SUCCESS)
    {
        Pmic_criticalSectionStart(pmicHandle);
        status = Pmic_ioRx(pmicHandle, PMIC_WD_MODE_REG_REGADDR, &regData);
        Pmic_criticalSectionStop(pmicHandle);
    }

    // Extract WD_RETURN_LONGWIN bit field (cast as boolean)
    if (status == PMIC_ST_SUCCESS)
    {
        *retLongWinStat = Pmic_getBitField_b(regData, PMIC_WD_RETURN_LONGWIN_SHIFT);
    }

    return status;
}

static int32_t WDG_setOtherCfg(const Pmic_CoreHandle_t *pmicHandle, const Pmic_WdgCfg_t *wdgCfg)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    // Read WD_CONFIG register
    if (Pmic_validParamCheck(wdgCfg->validParams, PMIC_WD_TRIG_SEL_VALID | PMIC_WD_MODE_VALID))
    {
        Pmic_criticalSectionStart(pmicHandle);
        status = Pmic_ioRx(pmicHandle, PMIC_WD_CONFIG_REGADDR, &regData);
        Pmic_criticalSectionStop(pmicHandle);
    }

    // Modify WD_TRIGGER_SEL bit field
    if (Pmic_validParamStatusCheck(wdgCfg->validParams, PMIC_WD_TRIG_SEL_VALID, status))
    {
        if (wdgCfg->trigSel > PMIC_TRIG_SEL_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
        else
        {
            Pmic_setBitField(&regData, PMIC_WD_TRIGGER_SEL_SHIFT, PMIC_WD_TRIGGER_SEL_MASK, wdgCfg->trigSel);
        }
    }

    // Modify WD_MODE_SELECT bit field
    if (Pmic_validParamStatusCheck(wdgCfg->validParams, PMIC_WD_MODE_VALID, status))
    {
        if (wdgCfg->mode > PMIC_WD_MODE_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
        else
        {
            Pmic_setBitField(&regData, PMIC_WD_MODE_SELECT_SHIFT, PMIC_WD_MODE_SELECT_MASK, wdgCfg->mode);
        }
    }

    // Write new WD_CONFIG register value back to PMIC
    if (Pmic_validParamStatusCheck(wdgCfg->validParams, PMIC_WD_TRIG_SEL_VALID | PMIC_WD_MODE_VALID, status))
    {
        Pmic_criticalSectionStart(pmicHandle);
        status = Pmic_ioTx(pmicHandle, PMIC_WD_CONFIG_REGADDR, regData);
        Pmic_criticalSectionStop(pmicHandle);
    }

    if (Pmic_validParamStatusCheck(wdgCfg->validParams, PMIC_WD_RST_EN_VALID, status))
    {
        // Read WD_ENABLE_REG register
        Pmic_criticalSectionStart(pmicHandle);
        status = Pmic_ioRx(pmicHandle, PMIC_WD_ENABLE_REG_REGADDR, &regData);
        Pmic_criticalSectionStop(pmicHandle);

        if (status == PMIC_ST_SUCCESS)
        {
            // Modify WD_RST_EN bit field
            Pmic_setBitField_b(&regData, PMIC_WD_RST_EN_SHIFT, PMIC_WD_RST_EN_MASK, wdgCfg->rstEn);

            // Write new WD_ENABLE_REG register value back to PMIC
            Pmic_criticalSectionStart(pmicHandle);
            status = Pmic_ioTx(pmicHandle, PMIC_WD_ENABLE_REG_REGADDR, regData);
            Pmic_criticalSectionStop(pmicHandle);
        }
    }

    return status;
}

static int32_t WDG_setQaCfg(const Pmic_CoreHandle_t *pmicHandle, const Pmic_WdgCfg_t *wdgCfg)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    if (Pmic_validParamCheck(wdgCfg->validParams, PMIC_WD_QA_FDBK_VALID | PMIC_WD_QA_LFSR_VALID | PMIC_WD_QA_SEED_VALID))
    {
        // Read WD_QA_CFG register
        Pmic_criticalSectionStart(pmicHandle);
        status = Pmic_ioRx(pmicHandle, PMIC_WD_QA_CFG_REGADDR, &regData);
        Pmic_criticalSectionStop(pmicHandle);
    }

    // Modify WD_QA_FDBK bit field
    if (Pmic_validParamStatusCheck(wdgCfg->validParams, PMIC_WD_QA_FDBK_VALID, status))
    {
        if (wdgCfg->qaFdbk > PMIC_WD_QA_FDBK_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
        else
        {
            Pmic_setBitField(&regData, PMIC_WD_QA_FDBK_SHIFT, PMIC_WD_QA_FDBK_MASK, wdgCfg->qaFdbk);
        }
    }

    // Modify WD_QA_LFSR bit field
    if (Pmic_validParamStatusCheck(wdgCfg->validParams, PMIC_WD_QA_LFSR_VALID, status))
    {
        if (wdgCfg->qaLfsr > PMIC_WD_QA_LFSR_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
        else
        {
            Pmic_setBitField(&regData, PMIC_WD_QA_LFSR_SHIFT, PMIC_WD_QA_LFSR_MASK, wdgCfg->qaLfsr);
        }
    }

    // Modify WD_QUESTION_SEED bit field
    if (Pmic_validParamStatusCheck(wdgCfg->validParams, PMIC_WD_QA_SEED_VALID, status))
    {
        if (wdgCfg->qaSeed > PMIC_WD_QA_SEED_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
        else
        {
            Pmic_setBitField(&regData, PMIC_WD_QUESTION_SEED_SHIFT, PMIC_WD_QUESTION_SEED_MASK, wdgCfg->qaSeed);
        }
    }

    // Write new register value back to PMIC
    if (Pmic_validParamStatusCheck(wdgCfg->validParams, PMIC_WD_QA_FDBK_VALID | PMIC_WD_QA_LFSR_VALID | PMIC_WD_QA_SEED_VALID, status))
    {
        Pmic_criticalSectionStart(pmicHandle);
        status = Pmic_ioTx(pmicHandle, PMIC_WD_QA_CFG_REGADDR, regData);
        Pmic_criticalSectionStop(pmicHandle);
    }

    return status;
}

static int32_t WDG_setThresholds(const Pmic_CoreHandle_t *pmicHandle, const Pmic_WdgCfg_t *wdgCfg)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    if (Pmic_validParamCheck(wdgCfg->validParams, PMIC_WD_FAIL_THR_VALID | PMIC_WD_RST_THR_VALID))
    {
        // Read WD_THR_CFG register
        Pmic_criticalSectionStart(pmicHandle);
        status = Pmic_ioRx(pmicHandle, PMIC_WD_THR_CFG_REGADDR, &regData);
        Pmic_criticalSectionStop(pmicHandle);
    }

    // Modify WD_FAIL_TH bit field
    if (Pmic_validParamStatusCheck(wdgCfg->validParams, PMIC_WD_FAIL_THR_VALID, status))
    {
        if (wdgCfg->failThr > PMIC_WD_FAIL_THR_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
        else
        {
            Pmic_setBitField(&regData, PMIC_WD_FAIL_TH_SHIFT, PMIC_WD_FAIL_TH_MASK, wdgCfg->failThr);
        }
    }

    // Modify WD_RST_TH bit field
    if (Pmic_validParamStatusCheck(wdgCfg->validParams, PMIC_WD_RST_THR_VALID, status))
    {
        if (wdgCfg->rstThr > PMIC_WD_RST_THR_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
        else
        {
            Pmic_setBitField(&regData, PMIC_WD_RST_TH_SHIFT, PMIC_WD_RST_TH_MASK, wdgCfg->rstThr);
        }
    }

    // Write new register value back to PMIC
    if (Pmic_validParamStatusCheck(wdgCfg->validParams, PMIC_WD_FAIL_THR_VALID | PMIC_WD_RST_THR_VALID, status))
    {
        Pmic_criticalSectionStart(pmicHandle);
        status = Pmic_ioTx(pmicHandle, PMIC_WD_THR_CFG_REGADDR, regData);
        Pmic_criticalSectionStop(pmicHandle);
    }

    return status;
}

static int32_t WDG_setWindowTimeIntervals(const Pmic_CoreHandle_t *pmicHandle, const Pmic_WdgCfg_t *wdgCfg)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    // Set WD_WIN1 bit field
    if (Pmic_validParamCheck(wdgCfg->validParams, PMIC_WD_WIN1_DURATION_VALID))
    {
        // Check for out of bounds Window-1 duration
        if (wdgCfg->win1Duration > PMIC_WD_WIN1_DURATION_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }

        // Read WD_WIN1_CFG register
        if (status == PMIC_ST_SUCCESS)
        {
            Pmic_criticalSectionStart(pmicHandle);
            status = Pmic_ioRx(pmicHandle, PMIC_WD_WIN1_CFG_REGADDR, &regData);
            Pmic_criticalSectionStop(pmicHandle);
        }

        if (status == PMIC_ST_SUCCESS)
        {
            // Modify WD_WIN1 bit field
            Pmic_setBitField(&regData, PMIC_WD_WIN1_SHIFT, PMIC_WD_WIN1_MASK, wdgCfg->win1Duration);

            // Write new register value back to PMIC
            Pmic_criticalSectionStart(pmicHandle);
            status = Pmic_ioTx(pmicHandle, PMIC_WD_WIN1_CFG_REGADDR, regData);
            Pmic_criticalSectionStop(pmicHandle);
        }
    }

    // Set WD_WIN2 bit field
    if (Pmic_validParamStatusCheck(wdgCfg->validParams, PMIC_WD_WIN2_DURATION_VALID, status))
    {
        // Check for out of bounds Window-2 duration
        if (wdgCfg->win2Duration > PMIC_WD_WIN2_DURATION_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }

        // Read WD_WIN2_CFG register
        if (status == PMIC_ST_SUCCESS)
        {
            Pmic_criticalSectionStart(pmicHandle);
            status = Pmic_ioRx(pmicHandle, PMIC_WD_WIN2_CFG_REGADDR, &regData);
            Pmic_criticalSectionStop(pmicHandle);
        }

        if (status == PMIC_ST_SUCCESS)
        {
            // Modify WD_WIN2 bit field
            Pmic_setBitField(&regData, PMIC_WD_WIN2_SHIFT, PMIC_WD_WIN2_MASK, wdgCfg->win2Duration);

            // Write new register value back to PMIC
            Pmic_criticalSectionStart(pmicHandle);
            status = Pmic_ioTx(pmicHandle, PMIC_WD_WIN2_CFG_REGADDR, regData);
            Pmic_criticalSectionStop(pmicHandle);
        }
    }

    // Set WD_LONGWIN bit field (the entire register is the WD_LONGWIN bit field)
    if (Pmic_validParamStatusCheck(wdgCfg->validParams, PMIC_WD_LONG_WIN_DURATION_VALID, status))
    {
        Pmic_criticalSectionStart(pmicHandle);
        status = Pmic_ioTx(pmicHandle, PMIC_WD_LONGWIN_CFG_REGADDR, wdgCfg->longWinDuration);
        Pmic_criticalSectionStop(pmicHandle);
    }

    return status;
}

int32_t Pmic_wdgSetCfg(const Pmic_CoreHandle_t *pmicHandle, const Pmic_WdgCfg_t *wdgCfg)
{
    int32_t status = Pmic_checkPmicCoreHandle(pmicHandle);

    if ((status == PMIC_ST_SUCCESS) && (wdgCfg == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) && (wdgCfg->validParams == 0U))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    // Set Long Window, Window-1, and Window-2 durations
    if (status == PMIC_ST_SUCCESS)
    {
        status = WDG_setWindowTimeIntervals(pmicHandle, wdgCfg);
    }

    // Set fail and reset thresholds
    if (status == PMIC_ST_SUCCESS)
    {
        status = WDG_setThresholds(pmicHandle, wdgCfg);
    }

    // Set Q&A configurations (LFSR, FDBK, seed)
    if (status == PMIC_ST_SUCCESS)
    {
        status = WDG_setQaCfg(pmicHandle, wdgCfg);
    }

    // Set other configurations (WD_RST_EN, WD_TRIGGER_SEL, WD_MODE_SEL)
    if (status == PMIC_ST_SUCCESS)
    {
        status = WDG_setOtherCfg(pmicHandle, wdgCfg);
    }

    return status;
}

int32_t Pmic_wdgGetEnable(const Pmic_CoreHandle_t *pmicHandle, bool *wdgEnabled)
{
    uint8_t regData = 0U;
    int32_t status = Pmic_checkPmicCoreHandle(pmicHandle);

    if ((status == PMIC_ST_SUCCESS) && (wdgEnabled == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Read WD_ENABLE_REG register
        Pmic_criticalSectionStart(pmicHandle);
        status = Pmic_ioRx(pmicHandle, PMIC_WD_ENABLE_REG_REGADDR, &regData);
        Pmic_criticalSectionStop(pmicHandle);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Extract WD_EN bit field (cast as boolean)
        *wdgEnabled = Pmic_getBitField_b(regData, PMIC_WD_EN_SHIFT);
    }

    return status;
}

int32_t Pmic_wdgSetEnableState(const Pmic_CoreHandle_t *pmicHandle, bool enable)
{
    uint8_t regData = 0U;
    int32_t status = Pmic_checkPmicCoreHandle(pmicHandle);

    // Read WD_ENABLE_REG register
    if (status == PMIC_ST_SUCCESS)
    {
        Pmic_criticalSectionStart(pmicHandle);
        status = Pmic_ioRx(pmicHandle, PMIC_WD_ENABLE_REG_REGADDR, &regData);
        Pmic_criticalSectionStop(pmicHandle);
    }

    // Modify WD_EN bit field; write new register value back to PMIC
    if (status == PMIC_ST_SUCCESS)
    {
        Pmic_setBitField_b(&regData, PMIC_WD_EN_SHIFT, PMIC_WD_EN_MASK, enable);

        Pmic_criticalSectionStart(pmicHandle);
        status = Pmic_ioTx(pmicHandle, PMIC_WD_ENABLE_REG_REGADDR, regData);
        Pmic_criticalSectionStop(pmicHandle);
    }

    return status;
}

int32_t Pmic_wdgDisable(const Pmic_CoreHandle_t *pmicHandle)
{
    return Pmic_wdgSetEnableState(pmicHandle, PMIC_DISABLE);
}


int32_t Pmic_wdgEnable(const Pmic_CoreHandle_t *pmicHandle)
{
    return Pmic_wdgSetEnableState(pmicHandle, PMIC_ENABLE);
}

static int32_t WDG_getOtherCfg(const Pmic_CoreHandle_t *pmicHandle, Pmic_WdgCfg_t *wdgCfg)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    if (Pmic_validParamCheck(wdgCfg->validParams, PMIC_WD_RST_EN_VALID))
    {
        // Read WD_ENABLE_REG register
        Pmic_criticalSectionStart(pmicHandle);
        status = Pmic_ioRx(pmicHandle, PMIC_WD_ENABLE_REG_REGADDR, &regData);
        Pmic_criticalSectionStop(pmicHandle);

        if (status == PMIC_ST_SUCCESS)
        {
            // Extract WD_RST_EN bit field
            wdgCfg->rstEn = Pmic_getBitField_b(regData, PMIC_WD_RST_EN_SHIFT);
        }
    }

    if (Pmic_validParamStatusCheck(wdgCfg->validParams, PMIC_WD_TRIG_SEL_VALID | PMIC_WD_MODE_VALID, status))
    {
        // Read WD_CONFIG register
        Pmic_criticalSectionStart(pmicHandle);
        status = Pmic_ioRx(pmicHandle, PMIC_WD_CONFIG_REGADDR, &regData);
        Pmic_criticalSectionStop(pmicHandle);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Extract WD_TRIGGER_SEL bit field
        if (Pmic_validParamCheck(wdgCfg->validParams, PMIC_WD_TRIG_SEL_VALID))
        {
            wdgCfg->trigSel = Pmic_getBitField(regData, PMIC_WD_TRIGGER_SEL_SHIFT, PMIC_WD_TRIGGER_SEL_MASK);
        }

        // Extract WD_MODE_SELECT bit field
        if (Pmic_validParamCheck(wdgCfg->validParams, PMIC_WD_MODE_VALID))
        {
            wdgCfg->mode = Pmic_getBitField(regData, PMIC_WD_MODE_SELECT_SHIFT, PMIC_WD_MODE_SELECT_MASK);
        }
    }

    return status;
}

static int32_t WDG_getQaCfg(const Pmic_CoreHandle_t *pmicHandle, Pmic_WdgCfg_t *wdgCfg)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    if (Pmic_validParamCheck(wdgCfg->validParams, PMIC_WD_QA_FDBK_VALID | PMIC_WD_QA_LFSR_VALID | PMIC_WD_QA_SEED_VALID))
    {
        // Read WD_QA_CFG register
        Pmic_criticalSectionStart(pmicHandle);
        status = Pmic_ioRx(pmicHandle, PMIC_WD_QA_CFG_REGADDR, &regData);
        Pmic_criticalSectionStop(pmicHandle);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Extract WD_QA_FDBK bit field
        if (Pmic_validParamCheck(wdgCfg->validParams,PMIC_WD_QA_FDBK_VALID))
        {
            wdgCfg->qaFdbk = Pmic_getBitField(regData, PMIC_WD_QA_FDBK_SHIFT, PMIC_WD_QA_FDBK_MASK);
        }

        // Extract WD_QA_LFSR bit field
        if (Pmic_validParamCheck(wdgCfg->validParams,PMIC_WD_QA_LFSR_VALID))
        {
            wdgCfg->qaLfsr = Pmic_getBitField(regData, PMIC_WD_QA_LFSR_SHIFT, PMIC_WD_QA_LFSR_MASK);
        }

        // Extract WD_QUESTION_SEED bit field
        if (Pmic_validParamCheck(wdgCfg->validParams,PMIC_WD_QA_SEED_VALID))
        {
            wdgCfg->qaSeed = Pmic_getBitField(regData, PMIC_WD_QUESTION_SEED_SHIFT, PMIC_WD_QUESTION_SEED_MASK);
        }
    }

    return status;
}

static int32_t WDG_getThresholds(const Pmic_CoreHandle_t *pmicHandle, Pmic_WdgCfg_t *wdgCfg)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    if (Pmic_validParamCheck(wdgCfg->validParams, PMIC_WD_FAIL_THR_VALID | PMIC_WD_RST_THR_VALID))
    {
        // Read WD_THR_CFG register
        Pmic_criticalSectionStart(pmicHandle);
        status = Pmic_ioRx(pmicHandle, PMIC_WD_THR_CFG_REGADDR, &regData);
        Pmic_criticalSectionStop(pmicHandle);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Extract WD_FAIL_TH bit field
        if (Pmic_validParamCheck(wdgCfg->validParams, PMIC_WD_FAIL_THR_VALID))
        {
            wdgCfg->failThr = Pmic_getBitField(regData, PMIC_WD_FAIL_TH_SHIFT, PMIC_WD_FAIL_TH_MASK);
        }

        // Extract WD_RST_TH bit field
        if (Pmic_validParamCheck(wdgCfg->validParams, PMIC_WD_RST_THR_VALID))
        {
            wdgCfg->rstThr = Pmic_getBitField(regData, PMIC_WD_RST_TH_SHIFT, PMIC_WD_RST_TH_MASK);
        }
    }

    return status;
}

static int32_t WDG_getWindowTimeIntervals(const Pmic_CoreHandle_t *pmicHandle, Pmic_WdgCfg_t *wdgCfg)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    // Get Window-1 duration
    if (Pmic_validParamCheck(wdgCfg->validParams, PMIC_WD_WIN1_DURATION_VALID))
    {
        // Read WD_WIN1_CFG register
        Pmic_criticalSectionStart(pmicHandle);
        status = Pmic_ioRx(pmicHandle, PMIC_WD_WIN1_CFG_REGADDR, &regData);
        Pmic_criticalSectionStop(pmicHandle);

        // Extract WD_WIN1 bit field
        if (status == PMIC_ST_SUCCESS)
        {
            wdgCfg->win1Duration = Pmic_getBitField(regData, PMIC_WD_WIN1_SHIFT, PMIC_WD_WIN1_MASK);
        }
    }

    // get Window-2 duration
    if (Pmic_validParamStatusCheck(wdgCfg->validParams, PMIC_WD_WIN2_DURATION_VALID, status))
    {
        // read WD_WIN2_CFG register
        Pmic_criticalSectionStart(pmicHandle);
        status = Pmic_ioRx(pmicHandle, PMIC_WD_WIN2_CFG_REGADDR, &regData);
        Pmic_criticalSectionStop(pmicHandle);

        // Extract WD_WIN2 bit field
        if (status == PMIC_ST_SUCCESS)
        {
            wdgCfg->win2Duration = Pmic_getBitField(regData, PMIC_WD_WIN2_SHIFT, PMIC_WD_WIN2_MASK);
        }
    }

    // Get Long Window duration
    if (Pmic_validParamStatusCheck(wdgCfg->validParams, PMIC_WD_LONG_WIN_DURATION_VALID, status))
    {
        // Read WD_LONGWIN_CFG register
        Pmic_criticalSectionStart(pmicHandle);
        status = Pmic_ioRx(pmicHandle, PMIC_WD_LONGWIN_CFG_REGADDR, &regData);
        Pmic_criticalSectionStop(pmicHandle);

        // Extract WD_LONGWIN bit field (the entire register is the WD_LONGWIN bit field)
        if (status == PMIC_ST_SUCCESS)
        {
            wdgCfg->longWinDuration = regData;
        }
    }

    return status;
}

int32_t Pmic_wdgGetCfg(const Pmic_CoreHandle_t *pmicHandle, Pmic_WdgCfg_t *wdgCfg)
{
    int32_t status = Pmic_checkPmicCoreHandle(pmicHandle);

    if ((status == PMIC_ST_SUCCESS) && (wdgCfg == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) && (wdgCfg->validParams == 0U))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    // Get Long Window, Window-1, and Window-2 durations
    if (status == PMIC_ST_SUCCESS)
    {
        status = WDG_getWindowTimeIntervals(pmicHandle, wdgCfg);
    }

    // Get fail and reset thresholds
    if (status == PMIC_ST_SUCCESS)
    {
        status = WDG_getThresholds(pmicHandle, wdgCfg);
    }

    // Get Q&A configurations (LFSR, FDBK, seed)
    if (status == PMIC_ST_SUCCESS)
    {
        status = WDG_getQaCfg(pmicHandle, wdgCfg);
    }

    // Get other configurations (WD_RST_EN, WD_TRIGGER_SEL, WD_MODE_SEL)
    if (status == PMIC_ST_SUCCESS)
    {
        status = WDG_getOtherCfg(pmicHandle, wdgCfg);
    }

    return status;
}

int32_t Pmic_wdgClrErrStatAll(const Pmic_CoreHandle_t *pmicHandle)
{
    int32_t status = Pmic_checkPmicCoreHandle(pmicHandle);

    // Bits of WD_ERR_STATUS register are W1C - write 1 to clear
    if (status == PMIC_ST_SUCCESS)
    {
        Pmic_criticalSectionStart(pmicHandle);
        status = Pmic_ioTx(pmicHandle, PMIC_WD_ERR_STATUS_REGADDR, 0xFFU);
        Pmic_criticalSectionStop(pmicHandle);
    }

    return status;
}

int32_t Pmic_wdgClrErrStat(const Pmic_CoreHandle_t *pmicHandle, const Pmic_WdgErrStat_t *wdgErrStat)
{
    uint8_t regData = 0U;
    int32_t status = Pmic_checkPmicCoreHandle(pmicHandle);

    if ((status == PMIC_ST_SUCCESS) && (wdgErrStat == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) &&
        ((wdgErrStat->validParams == 0U) || (wdgErrStat->validParams > PMIC_WDG_ERR_STAT_ALL_VALID)))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    // All watchdog error statuses are W1C - write 1 to clear
    if (status == PMIC_ST_SUCCESS)
    {
        // Clear WD_RST_INT
        if (Pmic_validParamCheck(wdgErrStat->validParams, PMIC_WDG_RST_INT_VALID))
        {
            Pmic_setBitField(&regData, PMIC_WD_RST_INT_SHIFT, PMIC_WD_RST_INT_MASK, 1U);
        }
        // Clear WD_FAIL_INT
        if (Pmic_validParamCheck(wdgErrStat->validParams, PMIC_WDG_FAIL_INT_VALID))
        {
            Pmic_setBitField(&regData, PMIC_WD_FAIL_INT_SHIFT, PMIC_WD_FAIL_INT_MASK, 1U);
        }
        // Clear WD_ANSW_ERR
        if (Pmic_validParamCheck(wdgErrStat->validParams, PMIC_WDG_ANSW_ERR_VALID))
        {
            Pmic_setBitField(&regData, PMIC_WD_ANSW_ERR_SHIFT, PMIC_WD_ANSW_ERR_MASK, 1U);
        }
        // Clear WD_SEQ_ERR
        if (Pmic_validParamCheck(wdgErrStat->validParams, PMIC_WDG_SEQ_ERR_VALID))
        {
            Pmic_setBitField(&regData, PMIC_WD_SEQ_ERR_SHIFT, PMIC_WD_SEQ_ERR_MASK, 1U);
        }
        // Clear WD_ANSW_EARLY
        if (Pmic_validParamCheck(wdgErrStat->validParams, PMIC_WDG_ANSW_EARLY_ERR_VALID))
        {
            Pmic_setBitField(&regData, PMIC_WD_ANSW_EARLY_SHIFT, PMIC_WD_ANSW_EARLY_MASK, 1U);
        }
        // Clear WD_TIMEOUT
        if (Pmic_validParamCheck(wdgErrStat->validParams, PMIC_WDG_TIMEOUT_ERR_VALID))
        {
            Pmic_setBitField(&regData, PMIC_WD_TIMEOUT_SHIFT, PMIC_WD_TIMEOUT_MASK, 1U);
        }
        // Clear WD_LONGWIN_TIMEOUT_INT
        if (Pmic_validParamCheck(wdgErrStat->validParams, PMIC_WDG_LONGWIN_TIMEOUT_INT_VALID))
        {
            Pmic_setBitField(&regData, PMIC_WD_LONGWIN_TIMEOUT_INT_SHIFT, PMIC_WD_LONGWIN_TIMEOUT_INT_MASK, 1U);
        }

        // Write new WD_ERR_STATUS register value back to PMIC
        Pmic_criticalSectionStart(pmicHandle);
        status = Pmic_ioTx(pmicHandle, PMIC_WD_ERR_STATUS_REGADDR, regData);
        Pmic_criticalSectionStop(pmicHandle);
    }

    return status;
}

int32_t Pmic_wdgGetErrStat(const Pmic_CoreHandle_t *pmicHandle, Pmic_WdgErrStat_t *wdgErrStat)
{
    uint8_t regData = 0U;
    int32_t status = Pmic_checkPmicCoreHandle(pmicHandle);

    if ((status == PMIC_ST_SUCCESS) && (wdgErrStat == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) &&
        ((wdgErrStat->validParams == 0U) || (wdgErrStat->validParams > PMIC_WDG_ERR_STAT_ALL_VALID)))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    // Read WD_ERR_STATUS register
    if (status == PMIC_ST_SUCCESS)
    {
        Pmic_criticalSectionStart(pmicHandle);
        status = Pmic_ioRx(pmicHandle, PMIC_WD_ERR_STATUS_REGADDR, &regData);
        Pmic_criticalSectionStop(pmicHandle);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Extract WD_RST_INT bit field
        if (Pmic_validParamCheck(wdgErrStat->validParams, PMIC_WDG_RST_INT_VALID))
        {
            wdgErrStat->rstInt = Pmic_getBitField_b(regData, PMIC_WD_RST_INT_SHIFT);
        }
        // Extract WD_FAIL_INT bit field
        if (Pmic_validParamCheck(wdgErrStat->validParams, PMIC_WDG_FAIL_INT_VALID))
        {
            wdgErrStat->failInt = Pmic_getBitField_b(regData, PMIC_WD_FAIL_INT_SHIFT);
        }
        // Extract WD_ANSW_ERR bit field
        if (Pmic_validParamCheck(wdgErrStat->validParams, PMIC_WDG_ANSW_ERR_VALID))
        {
            wdgErrStat->answErr = Pmic_getBitField_b(regData, PMIC_WD_ANSW_ERR_SHIFT);
        }
        // Extract WD_SEQ_ERR bit field
        if (Pmic_validParamCheck(wdgErrStat->validParams, PMIC_WDG_SEQ_ERR_VALID))
        {
            wdgErrStat->seqErr = Pmic_getBitField_b(regData, PMIC_WD_SEQ_ERR_SHIFT);
        }
        // Extract WD_ANSW_EARLY bit field
        if (Pmic_validParamCheck(wdgErrStat->validParams, PMIC_WDG_ANSW_EARLY_ERR_VALID))
        {
            wdgErrStat->answEarlyErr = Pmic_getBitField_b(regData, PMIC_WD_ANSW_EARLY_SHIFT);
        }
        // Extract WD_TIMEOUT bit field
        if (Pmic_validParamCheck(wdgErrStat->validParams, PMIC_WDG_TIMEOUT_ERR_VALID))
        {
            wdgErrStat->timeoutErr = Pmic_getBitField_b(regData, PMIC_WD_TIMEOUT_SHIFT);
        }
        // Extract WD_LONGWIN_TIMEOUT_INT bit field
        if (Pmic_validParamCheck(wdgErrStat->validParams, PMIC_WDG_LONGWIN_TIMEOUT_INT_VALID))
        {
            wdgErrStat->longWinTimeoutInt = Pmic_getBitField_b(regData, PMIC_WD_LONGWIN_TIMEOUT_INT_SHIFT);
        }
    }

    return status;
}

int32_t Pmic_wdgGetFailCntStat(const Pmic_CoreHandle_t *pmicHandle, Pmic_WdgFailCntStat_t *wdgFailCntStat)
{
    uint8_t regData = 0U;
    int32_t status = Pmic_checkPmicCoreHandle(pmicHandle);

    if ((status == PMIC_ST_SUCCESS) && (wdgFailCntStat == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) && (wdgFailCntStat->validParams == 0U))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    // Read WD_FAIL_CNT_REG register
    if (status == PMIC_ST_SUCCESS)
    {
        Pmic_criticalSectionStart(pmicHandle);
        status = Pmic_ioRx(pmicHandle, PMIC_WD_FAIL_CNT_REG_REGADDR, &regData);
        Pmic_criticalSectionStop(pmicHandle);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Extract WD_BAD_EVENT bit field
        if (Pmic_validParamCheck(wdgFailCntStat->validParams, PMIC_BAD_EVENT_VALID))
        {
            wdgFailCntStat->badEvent = Pmic_getBitField_b(regData, PMIC_WD_BAD_EVENT_SHIFT);
        }
        // Extract WD_FIRST_OK bit field
        if (Pmic_validParamCheck(wdgFailCntStat->validParams, PMIC_GOOD_EVENT_VALID))
        {
            wdgFailCntStat->goodEvent = Pmic_getBitField_b(regData, PMIC_WD_FIRST_OK_SHIFT);
        }
        // Extract WD_FAIL_CNT bit field
        if (Pmic_validParamCheck(wdgFailCntStat->validParams, PMIC_FAIL_CNT_VALID))
        {
            wdgFailCntStat->failCnt = Pmic_getBitField(regData, PMIC_WD_FAIL_CNT_SHIFT, PMIC_WD_FAIL_CNT_MASK);
        }
    }

    return status;
}
