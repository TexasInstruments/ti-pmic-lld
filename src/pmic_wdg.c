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
 * @brief PMIC LLD IRQ module source file containing definitions to APIs that
 * interact with PMIC Watchdog module.
 */
#include "pmic.h"
#include "pmic_io.h"
#include "pmic_wdg.h"

#include "regmap/wdg.h"
#include "regmap/irq.h"
#include "regmap/core.h"

static int32_t WDG_setOtherCfg(const Pmic_CoreHandle_t *pmicHandle, const Pmic_WdgCfg_t *wdgCfg)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    // Read SAFETY_FUNC_CFG register
    Pmic_criticalSectionStart(pmicHandle);
    if (Pmic_validParamCheck(wdgCfg->validParams, PMIC_WD_RST_EN_VALID | PMIC_WD_MODE_VALID))
    {
        status = Pmic_ioRxByte(pmicHandle, PMIC_CMD_RD_SAFETY_FUNC_CFG, &regData);
    }

    // Modify WD_RST_EN bit field
    if (Pmic_validParamStatusCheck(wdgCfg->validParams, PMIC_WD_RST_EN_VALID, status))
    {
        Pmic_setBitField_b(&regData, WD_RST_EN_SHIFT, wdgCfg->rstEn);
    }

    // Modify WD_CFG bit field
    if (Pmic_validParamStatusCheck(wdgCfg->validParams, PMIC_WD_MODE_VALID, status))
    {
        if (wdgCfg->mode > PMIC_WD_MODE_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
        else
        {
            Pmic_setBitField(&regData, WD_RST_EN_SHIFT, WD_RST_EN_MASK, wdgCfg->mode);
        }
    }

    // Write new register value back to PMIC
    if (Pmic_validParamStatusCheck(wdgCfg->validParams, PMIC_WD_RST_EN_VALID | PMIC_WD_MODE_VALID, status))
    {
        status = Pmic_ioTxByte(pmicHandle, PMIC_CMD_WR_SAFETY_FUNC_CFG, regData);
    }
    Pmic_criticalSectionStop(pmicHandle);

    return status;
}

static int32_t WDG_setQACfg(const Pmic_CoreHandle_t *pmicHandle, const Pmic_WdgCfg_t *wdgCfg)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;
    const uint32_t qaCfgValidParams = PMIC_WD_QA_FDBK_VALID | PMIC_WD_QA_LFSR_VALID | PMIC_WD_QA_SEED_VALID;

    // Read WD_QUESTION_FDBCK register
    Pmic_criticalSectionStart(pmicHandle);
    if (Pmic_validParamCheck(wdgCfg->validParams, qaCfgValidParams))
    {
        status = Pmic_ioRxByte(pmicHandle, PMIC_CMD_RD_WD_QUESTION_FDBCK, &regData);
    }

    // Modify FDBK[1:0] bit field
    if (Pmic_validParamStatusCheck(wdgCfg->validParams, PMIC_WD_QA_FDBK_VALID, status))
    {
        if (wdgCfg->qaFdbk > PMIC_WD_QA_FDBK_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
        else
        {
            Pmic_setBitField(&regData, FDBK_SHIFT, FDBK_MASK, wdgCfg->qaFdbk);
        }
    }

    // Modify LFSR_CHG[1:0] bit field
    if (Pmic_validParamStatusCheck(wdgCfg->validParams, PMIC_WD_QA_LFSR_VALID, status))
    {
        if (wdgCfg->qaLfsr > PMIC_WD_QA_LFSR_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
        else
        {
            Pmic_setBitField(&regData, LFSR_CHG_SHIFT, LFSR_CHG_MASK, wdgCfg->qaLfsr);
        }
    }

    // Modify QUESTION_SEED[3:0] bit field
    if (Pmic_validParamStatusCheck(wdgCfg->validParams, PMIC_WD_QA_SEED_VALID, status))
    {
        if (wdgCfg->qaSeed > PMIC_WD_QA_SEED_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
        else
        {
            Pmic_setBitField(&regData, QUESTION_SEED_SHIFT, QUESTION_SEED_MASK, wdgCfg->qaSeed);
        }
    }

    // Write new register value back to PMIC
    if (Pmic_validParamStatusCheck(wdgCfg->validParams, qaCfgValidParams, status))
    {
        status = Pmic_ioTxByte(pmicHandle, PMIC_CMD_WR_WD_QUESTION_FDBCK, regData);
    }
    Pmic_criticalSectionStop(pmicHandle);

    return status;
}

static int32_t WDG_setWindowDurations(const Pmic_CoreHandle_t *pmicHandle, const Pmic_WdgCfg_t *wdgCfg)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    if (Pmic_validParamCheck(wdgCfg->validParams, PMIC_WD_WIN1_DURATION_VALID))
    {
        // Read WD_WIN1_CFG register
        Pmic_criticalSectionStart(pmicHandle);
        status = Pmic_ioRxByte(pmicHandle, PMIC_CMD_RD_WD_WIN1_CFG, &regData);
        if (status == PMIC_ST_SUCCESS)
        {
            // Modify RT[6:0] bit field
            if (wdgCfg->win1Duration > PMIC_WD_WIN1_DURATION_MAX)
            {
                status = PMIC_ST_ERR_INV_PARAM;
            }
            else
            {
                Pmic_setBitField(&regData, RT_SHIFT, RT_MASK, wdgCfg->win1Duration);
            }

            // Write new register value back to PMIC
            if (status == PMIC_ST_SUCCESS)
            {
                status = Pmic_ioTxByte(pmicHandle, PMIC_CMD_WR_WD_WIN1_CFG, regData);
            }
        }
        Pmic_criticalSectionStop(pmicHandle);
    }

    if (Pmic_validParamStatusCheck(wdgCfg->validParams, PMIC_WD_WIN2_DURATION_VALID, status))
    {
        // Read WD_WIN2_CFG register
        Pmic_criticalSectionStart(pmicHandle);
        status = Pmic_ioRxByte(pmicHandle, PMIC_CMD_RD_WD_WIN2_CFG, &regData);
        if (status == PMIC_ST_SUCCESS)
        {
            // Modify RT[6:0] bit field
            if (wdgCfg->win2Duration > PMIC_WD_WIN2_DURATION_MAX)
            {
                status = PMIC_ST_ERR_INV_PARAM;
            }
            else
            {
                Pmic_setBitField(&regData, RW_SHIFT, RW_MASK, wdgCfg->win2Duration);
            }

            // Write new register value back to PMIC
            if (status == PMIC_ST_SUCCESS)
            {
                status = Pmic_ioTxByte(pmicHandle, PMIC_CMD_WR_WD_WIN2_CFG, regData);
            }
        }
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

    // Set Window durations
    if (status == PMIC_ST_SUCCESS)
    {
        status = WDG_setWindowDurations(pmicHandle, wdgCfg);
    }

    // Set Q&A configurations
    if (status == PMIC_ST_SUCCESS)
    {
        status = WDG_setQACfg(pmicHandle, wdgCfg);
    }

    // Set other WDG configurations (like WDG mode and WD_RST_EN)
    if (status == PMIC_ST_SUCCESS)
    {
        status = WDG_setOtherCfg(pmicHandle, wdgCfg);
    }

    return status;
}

static int32_t WDG_getOtherCfg(const Pmic_CoreHandle_t *pmicHandle, Pmic_WdgCfg_t *wdgCfg)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    if (Pmic_validParamCheck(wdgCfg->validParams, PMIC_WD_RST_EN_VALID | PMIC_WD_MODE_VALID))
    {
        // Read SAFETY_FUNC_CFG register
        status = Pmic_ioRxByte_CS(pmicHandle, PMIC_CMD_RD_SAFETY_FUNC_CFG, &regData);

        if (status == PMIC_ST_SUCCESS)
        {
            // Extract WD_RST_EN bit field
            if (Pmic_validParamCheck(wdgCfg->validParams, PMIC_WD_RST_EN_VALID))
            {
                wdgCfg->rstEn = Pmic_getBitField_b(regData, WD_RST_EN_SHIFT);
            }

            // Extract WD_CFG bit field
            if (Pmic_validParamCheck(wdgCfg->validParams, PMIC_WD_MODE_VALID))
            {
                wdgCfg->mode = Pmic_getBitField(regData, WD_CFG_SHIFT, WD_CFG_MASK);
            }
        }
    }

    return status;
}


static int32_t WDG_getQACfg(const Pmic_CoreHandle_t *pmicHandle, Pmic_WdgCfg_t *wdgCfg)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;
    const uint32_t qaCfgValidParams = PMIC_WD_QA_FDBK_VALID | PMIC_WD_QA_LFSR_VALID | PMIC_WD_QA_SEED_VALID;

    if (Pmic_validParamCheck(wdgCfg->validParams, qaCfgValidParams))
    {
        // Read WD_QUESTION_FDBCK register
        status = Pmic_ioRxByte_CS(pmicHandle, PMIC_CMD_RD_WD_QUESTION_FDBCK, &regData);
        if (status == PMIC_ST_SUCCESS)
        {
            // Extract FDBK[1:0] bit field
            if (Pmic_validParamCheck(wdgCfg->validParams, PMIC_WD_QA_FDBK_VALID))
            {
                wdgCfg->qaFdbk = Pmic_getBitField(regData, FDBK_SHIFT, FDBK_MASK);
            }

            // Extract LFSR_CHG[1:0] bit field
            if (Pmic_validParamCheck(wdgCfg->validParams, PMIC_WD_QA_LFSR_VALID))
            {
                wdgCfg->qaLfsr = Pmic_getBitField(regData, LFSR_CHG_SHIFT, LFSR_CHG_MASK);
            }

            // Extract QUESTION_SEED[3:0] bit field
            if (Pmic_validParamCheck(wdgCfg->validParams, PMIC_WD_QA_SEED_VALID))
            {
                wdgCfg->qaSeed = Pmic_getBitField(regData, QUESTION_SEED_SHIFT, QUESTION_SEED_MASK);
            }
        }
    }

    return status;
}

static int32_t WDG_getWindowDurations(const Pmic_CoreHandle_t *pmicHandle, Pmic_WdgCfg_t *wdgCfg)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    if (Pmic_validParamCheck(wdgCfg->validParams, PMIC_WD_WIN1_DURATION_VALID))
    {
        // Read WD_WIN1_CFG register
        status = Pmic_ioRxByte_CS(pmicHandle, PMIC_CMD_RD_WD_WIN1_CFG, &regData);
        if (status == PMIC_ST_SUCCESS)
        {
            // Extract RT[6:0] bit field
            wdgCfg->win1Duration = Pmic_getBitField(regData, RT_SHIFT, RT_MASK);
        }
    }

    if (Pmic_validParamCheck(wdgCfg->validParams, PMIC_WD_WIN2_DURATION_VALID))
    {
        // Read WD_WIN2_CFG register
        status = Pmic_ioRxByte_CS(pmicHandle, PMIC_CMD_RD_WD_WIN2_CFG, &regData);
        if (status == PMIC_ST_SUCCESS)
        {
            // Extract RW[4:0] bit field
            wdgCfg->win2Duration = Pmic_getBitField(regData, RW_SHIFT, RW_MASK);
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

    // Get Window durations
    if (status == PMIC_ST_SUCCESS)
    {
        status = WDG_getWindowDurations(pmicHandle, wdgCfg);
    }

    // Get Q&A configurations
    if (status == PMIC_ST_SUCCESS)
    {
        status = WDG_getQACfg(pmicHandle, wdgCfg);
    }

    // Get other WDG configurations (like WDG mode and WD_RST_EN)
    if (status == PMIC_ST_SUCCESS)
    {
        status = WDG_getOtherCfg(pmicHandle, wdgCfg);
    }

    return status;
}

static inline uint8_t mux_4x1(uint8_t x0, uint8_t x1, uint8_t x2, uint8_t x3, uint8_t qaFdbk)
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

static uint8_t WDG_getAnswerByte(uint8_t qaQuesCnt, uint8_t qaAnsCnt, uint8_t qaFdbk)
{
    uint8_t q0 = 0U, q1 = 0U, q2 = 0U, q3 = 0U;
    uint8_t a0 = 0U, a1 = 0U;
    uint8_t qaAns = 0U;

    q0 = ((qaQuesCnt >> 0U) & 1U);
    q1 = ((qaQuesCnt >> 1U) & 1U);
    q2 = ((qaQuesCnt >> 2U) & 1U);
    q3 = ((qaQuesCnt >> 3U) & 1U);

    a0 = ((qaAnsCnt >> 0U) & 1U);
    a1 = ((qaAnsCnt >> 1U) & 1U);

    /* Reference-Answer-X[0] */
    qaAns = (mux_4x1(q0, q1, q2, q3, qaFdbk) ^ (mux_4x1(q3, q0, q1, q2, qaFdbk) ^ a1));
    /* Reference-Answer-X[1] */
    qaAns |= (((mux_4x1(q0, q1, q2, q3, qaFdbk) ^ (mux_4x1(q2, q0, q1, q3, qaFdbk) ^ q1) ^ a1)) << 1U);
    /* Reference-Answer-X[2] */
    qaAns |= (((mux_4x1(q0, q1, q2, q3, qaFdbk) ^ (mux_4x1(q3, q0, q1, q2, qaFdbk) ^ q1) ^ a1)) << 2U);
    /* Reference-Answer-X[3] */
    qaAns |= (((mux_4x1(q2, q0, q1, q3, qaFdbk) ^ (mux_4x1(q0, q1, q2, q3, qaFdbk) ^ q3) ^ a1)) << 3U);
    /* Reference-Answer-X[4] */
    qaAns |= (((mux_4x1(q1, q0, q2, q3, qaFdbk) ^ a0)) << 4U);
    /* Reference-Answer-X[5] */
    qaAns |= (((mux_4x1(q3, q0, q1, q2, qaFdbk) ^ a0)) << 5U);
    /* Reference-Answer-X[6] */
    qaAns |= (((mux_4x1(q0, q1, q2, q3, qaFdbk) ^ a0)) << 6U);
    /* Reference-Answer-X[7] */
    qaAns |= (((mux_4x1(q2, q0, q1, q2, qaFdbk) ^ a0)) << 7U);

    return qaAns;
}

int32_t Pmic_wdgQaSequenceWriteAnswer(const Pmic_CoreHandle_t *pmicHandle)
{
    uint8_t regData = 0U, qaFdbk = 0U, qaAnsCnt = 0U, qaQuesCnt = 0U;
    int32_t status = Pmic_checkPmicCoreHandle(pmicHandle);

    // Get the Q&A feedback value
    Pmic_criticalSectionStart(pmicHandle);
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioRxByte(pmicHandle, PMIC_CMD_RD_WD_QUESTION_FDBCK, &regData);
        if (status == PMIC_ST_SUCCESS)
        {
            qaFdbk = Pmic_getBitField(regData, FDBK_SHIFT, FDBK_MASK);
        }
    }

    // Get the Q&A answer count
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioRxByte(pmicHandle, PMIC_CMD_RD_WD_STATUS, &regData);
        if (status == PMIC_ST_SUCCESS)
        {
            qaAnsCnt = Pmic_getBitField(regData, WD_ANSW_CNT_SHIFT, WD_ANSW_CNT_MASK);
        }
    }

    // Get the question value
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioRxByte(pmicHandle, PMIC_CMD_RD_WD_QUESTION, &regData);
        if (status == PMIC_ST_SUCCESS)
        {
            qaQuesCnt = Pmic_getBitField(regData, QUESTION_SHIFT, QUESTION_MASK);
        }
    }

    // Calculate Q&A answer byte; write the Q&A answer byte to the WD_ANSWER register
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioTxByte(pmicHandle, PMIC_CMD_WR_WD_ANSWER, WDG_getAnswerByte(qaQuesCnt, qaAnsCnt, qaFdbk));
    }
    Pmic_criticalSectionStop(pmicHandle);

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

    // Read WD_STATUS register
    Pmic_criticalSectionStart(pmicHandle);
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioRxByte(pmicHandle, PMIC_CMD_RD_WD_STATUS, &regData);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Extract error statuses from the WD_STATUS register
        wdgErrStat->answerErr = Pmic_getBitField_b(regData, ANSWER_ERR_SHIFT);
        wdgErrStat->wdgCfgChg = Pmic_getBitField_b(regData, WD_CFG_CHG_SHIFT);
        wdgErrStat->seqErr = Pmic_getBitField_b(regData, SEQ_ERR_SHIFT);
        wdgErrStat->timeout = Pmic_getBitField_b(regData, TIME_OUT_SHIFT);
        wdgErrStat->answerEarly = Pmic_getBitField_b(regData, ANSWER_EARLY_SHIFT);

        // Read WD_QUESTION register
        status = Pmic_ioRxByte(pmicHandle, PMIC_CMD_RD_WD_QUESTION, &regData);
    }
    Pmic_criticalSectionStop(pmicHandle);

    // Extract WD_FAIL_TH error status
    if (status == PMIC_ST_SUCCESS)
    {
        wdgErrStat->failThr = Pmic_getBitField_b(regData, WD_FAIL_TH_SHIFT);
    }

    return status;
}

int32_t Pmic_wdgGetFailCnt(const Pmic_CoreHandle_t *pmicHandle, uint8_t *failCntVal)
{
    uint8_t regData = 0U;
    int32_t status = Pmic_checkPmicCoreHandle(pmicHandle);

    if ((status == PMIC_ST_SUCCESS) && (failCntVal == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    // Read SAFETY_STAT_5 register
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioRxByte_CS(pmicHandle, PMIC_CMD_RD_SAFETY_STAT_5, &regData);
    }

    // Extract WD_FAIL_CNT[2:0] bit field
    if (status == PMIC_ST_SUCCESS)
    {
        *failCntVal = Pmic_getBitField(regData, WD_FAIL_CNT_SHIFT, WD_FAIL_CNT_MASK);
    }

    return status;
}
