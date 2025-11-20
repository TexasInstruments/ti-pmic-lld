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
 * @brief PMIC LLD watchdog module implementation for TPS6522x-Q1.
 */

#include <stdint.h>

#include "pmic.h"
#include "pmic_wdg.h"
#include "pmic_io.h"
#include "regmap/wdg.h"

/* ========================================================================== */
/*                           Internal Helper Functions                        */
/* ========================================================================== */

/**
 * @brief Calculate CRC4 for WDG Q&A answer calculation
 */
static uint8_t calculateCrc4(uint8_t data)
{
    uint8_t crc = 0U;

    for (uint8_t i = 0U; i < 4U; i++)
    {
        uint8_t bit = ((data >> i) & 0x01U) ^ ((crc >> 3U) & 0x01U);
        crc = (uint8_t)((crc << 1U) | bit);
        if (bit != 0U)
        {
            crc ^= 0x03U;  // CRC4 polynomial
        }
    }

    return crc & 0x0FU;
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

    return status;
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

    return status;
}

int32_t Pmic_wdgSetCfg(const Pmic_Handle_t *handle, const Pmic_WdgCfg_t *wdgCfg)
{
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    if ((status == PMIC_ST_SUCCESS) && (wdgCfg == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    // Validate parameters
    if (status == PMIC_ST_SUCCESS)
    {
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
    }

    // Set WD_MODE_REG fields
    Pmic_criticalSectionStart(handle);
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioRxByte(handle, WD_MODE_REG_REG, &regData);
    }

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
    Pmic_criticalSectionStop(handle);

    // Set WD_WIN1_CFG
    if ((status == PMIC_ST_SUCCESS) && Pmic_validParamCheck(wdgCfg->validParams, PMIC_WDG_WIN1_CODE_VALID))
    {
        status = Pmic_ioUpdateByte_CS(handle, WD_WIN1_CFG_REG, WD_WIN_SHIFT, WD_WIN_MASK, wdgCfg->win1Code);
    }

    // Set WD_WIN2_CFG
    if ((status == PMIC_ST_SUCCESS) && Pmic_validParamCheck(wdgCfg->validParams, PMIC_WDG_WIN2_CODE_VALID))
    {
        status = Pmic_ioUpdateByte_CS(handle, WD_WIN2_CFG_REG, WD_WIN_SHIFT, WD_WIN_MASK, wdgCfg->win2Code);
    }

    // Set WD_LONGWIN_CFG
    if ((status == PMIC_ST_SUCCESS) && Pmic_validParamCheck(wdgCfg->validParams, PMIC_WDG_LONG_WIN_CODE_VALID))
    {
        status = Pmic_ioTxByte_CS(handle, WD_LONGWIN_CFG_REG, wdgCfg->longWinCode);
    }

    // Set WD_QA_CFG fields
    Pmic_criticalSectionStart(handle);
    if (status == PMIC_ST_SUCCESS)
    {
        if (Pmic_validParamCheck(wdgCfg->validParams, PMIC_WDG_QA_FDBK_VALID) ||
            Pmic_validParamCheck(wdgCfg->validParams, PMIC_WDG_QA_LFSR_VALID) ||
            Pmic_validParamCheck(wdgCfg->validParams, PMIC_WDG_QA_SEED_VALID))
        {
            status = Pmic_ioRxByte(handle, WD_QA_CFG_REG, &regData);
        }
    }

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

        if (Pmic_validParamCheck(wdgCfg->validParams, PMIC_WDG_QA_FDBK_VALID) ||
            Pmic_validParamCheck(wdgCfg->validParams, PMIC_WDG_QA_LFSR_VALID) ||
            Pmic_validParamCheck(wdgCfg->validParams, PMIC_WDG_QA_SEED_VALID))
        {
            status = Pmic_ioTxByte(handle, WD_QA_CFG_REG, regData);
        }
    }
    Pmic_criticalSectionStop(handle);

    // Set WD_THR_CFG fields
    Pmic_criticalSectionStart(handle);
    if (status == PMIC_ST_SUCCESS)
    {
        if (Pmic_validParamCheck(wdgCfg->validParams, PMIC_WDG_RST_EN_VALID) ||
            Pmic_validParamCheck(wdgCfg->validParams, PMIC_WDG_FAIL_THR_VALID) ||
            Pmic_validParamCheck(wdgCfg->validParams, PMIC_WDG_RST_THR_VALID))
        {
            status = Pmic_ioRxByte(handle, WD_THR_CFG_REG, &regData);
        }
    }

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

        if (Pmic_validParamCheck(wdgCfg->validParams, PMIC_WDG_RST_EN_VALID) ||
            Pmic_validParamCheck(wdgCfg->validParams, PMIC_WDG_FAIL_THR_VALID) ||
            Pmic_validParamCheck(wdgCfg->validParams, PMIC_WDG_RST_THR_VALID))
        {
            status = Pmic_ioTxByte(handle, WD_THR_CFG_REG, regData);
        }
    }
    Pmic_criticalSectionStop(handle);

    return status;
}

int32_t Pmic_wdgGetCfg(const Pmic_Handle_t *handle, Pmic_WdgCfg_t *wdgCfg)
{
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    if ((status == PMIC_ST_SUCCESS) && (wdgCfg == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    // Get WD_MODE_REG fields
    if (status == PMIC_ST_SUCCESS)
    {
        if (Pmic_validParamCheck(wdgCfg->validParams, PMIC_WDG_MODE_SEL_VALID) ||
            Pmic_validParamCheck(wdgCfg->validParams, PMIC_WDG_CNT_SEL_VALID) ||
            Pmic_validParamCheck(wdgCfg->validParams, PMIC_WDG_EN_DRV_SEL_VALID))
        {
            status = Pmic_ioRxByte_CS(handle, WD_MODE_REG_REG, &regData);
        }
    }

    if (status == PMIC_ST_SUCCESS)
    {
        if (Pmic_validParamCheck(wdgCfg->validParams, PMIC_WDG_MODE_SEL_VALID))
        {
            wdgCfg->mode = Pmic_getBitField_b(regData, WD_MODE_SELECT_SHIFT) ? 1U : 0U;
        }

        if (Pmic_validParamCheck(wdgCfg->validParams, PMIC_WDG_CNT_SEL_VALID))
        {
            wdgCfg->cntSel = Pmic_getBitField_b(regData, WD_CNT_SEL_SHIFT) ? 1U : 0U;
        }

        if (Pmic_validParamCheck(wdgCfg->validParams, PMIC_WDG_EN_DRV_SEL_VALID))
        {
            wdgCfg->clrEnDrvOnFailInt = Pmic_getBitField_b(regData, WD_ENDRV_SEL_SHIFT);
        }
    }

    // Get WD_WIN1_CFG
    if ((status == PMIC_ST_SUCCESS) && Pmic_validParamCheck(wdgCfg->validParams, PMIC_WDG_WIN1_CODE_VALID))
    {
        status = Pmic_ioRxByte_CS(handle, WD_WIN1_CFG_REG, &regData);
        if (status == PMIC_ST_SUCCESS)
        {
            wdgCfg->win1Code = Pmic_getBitField(regData, WD_WIN_SHIFT, WD_WIN_MASK);
        }
    }

    // Get WD_WIN2_CFG
    if ((status == PMIC_ST_SUCCESS) && Pmic_validParamCheck(wdgCfg->validParams, PMIC_WDG_WIN2_CODE_VALID))
    {
        status = Pmic_ioRxByte_CS(handle, WD_WIN2_CFG_REG, &regData);
        if (status == PMIC_ST_SUCCESS)
        {
            wdgCfg->win2Code = Pmic_getBitField(regData, WD_WIN_SHIFT, WD_WIN_MASK);
        }
    }

    // Get WD_LONGWIN_CFG
    if ((status == PMIC_ST_SUCCESS) && Pmic_validParamCheck(wdgCfg->validParams, PMIC_WDG_LONG_WIN_CODE_VALID))
    {
        status = Pmic_ioRxByte_CS(handle, WD_LONGWIN_CFG_REG, &wdgCfg->longWinCode);
    }

    // Get WD_QA_CFG fields
    if (status == PMIC_ST_SUCCESS)
    {
        if (Pmic_validParamCheck(wdgCfg->validParams, PMIC_WDG_QA_FDBK_VALID) ||
            Pmic_validParamCheck(wdgCfg->validParams, PMIC_WDG_QA_LFSR_VALID) ||
            Pmic_validParamCheck(wdgCfg->validParams, PMIC_WDG_QA_SEED_VALID))
        {
            status = Pmic_ioRxByte_CS(handle, WD_QA_CFG_REG, &regData);
        }
    }

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

    // Get WD_THR_CFG fields
    if (status == PMIC_ST_SUCCESS)
    {
        if (Pmic_validParamCheck(wdgCfg->validParams, PMIC_WDG_RST_EN_VALID) ||
            Pmic_validParamCheck(wdgCfg->validParams, PMIC_WDG_FAIL_THR_VALID) ||
            Pmic_validParamCheck(wdgCfg->validParams, PMIC_WDG_RST_THR_VALID))
        {
            status = Pmic_ioRxByte_CS(handle, WD_THR_CFG_REG, &regData);
        }
    }

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

    return status;
}

int32_t Pmic_wdgSetPowerHold(const Pmic_Handle_t *handle, bool enable)
{
    int32_t status = Pmic_checkHandle(handle);

    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioUpdateByte_bCS(handle, WD_MODE_REG_REG, WD_PWRHOLD_SHIFT, enable);
    }

    return status;
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

    return status;
}

int32_t Pmic_wdgSetReturnToLongWindow(const Pmic_Handle_t *handle, bool enable)
{
    int32_t status = Pmic_checkHandle(handle);

    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioUpdateByte_bCS(handle, WD_MODE_REG_REG, WD_RETURN_LONGWIN_SHIFT, enable);
    }

    return status;
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

    return status;
}

int32_t Pmic_wdgQaWriteAnswer(const Pmic_Handle_t *handle)
{
    int32_t status = Pmic_checkHandle(handle);
    uint8_t questionReg = 0U;
    uint8_t question = 0U;
    uint8_t answer = 0U;

    // Read question from WD_QUESTION_ANSW_CNT register
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioRxByte_CS(handle, WD_QUESTION_ANSW_CNT_REG, &questionReg);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        uint8_t answerData;

        question = Pmic_getBitField(questionReg, WD_QUESTION_SHIFT, WD_QUESTION_MASK);

        // Calculate answer using CRC4
        answerData = calculateCrc4(question);
        answer = (uint8_t)((question << 4U) | answerData);

        // Write answer to WD_ANSWER_REG
        status = Pmic_ioTxByte_CS(handle, WD_ANSWER_REG_REG, answer);
    }

    return status;
}

int32_t Pmic_wdgClrErrStatus(const Pmic_Handle_t *handle, const Pmic_WdgErrStatus_t *wdgErrStatus)
{
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    if ((status == PMIC_ST_SUCCESS) && (wdgErrStatus == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Build the clear mask from validParams (write-1-to-clear)
        if (Pmic_validParamCheck(wdgErrStatus->validParams, PMIC_WDG_RST_INT_VALID))
        {
            Pmic_setBitField_b(&regData, WD_RST_INT_SHIFT, WD_RST_INT_MASK, true);
        }

        if (Pmic_validParamCheck(wdgErrStatus->validParams, PMIC_WDG_FAIL_INT_VALID))
        {
            Pmic_setBitField_b(&regData, WD_FAIL_INT_SHIFT, WD_FAIL_INT_MASK, true);
        }

        if (Pmic_validParamCheck(wdgErrStatus->validParams, PMIC_WDG_ANSW_ERR_VALID))
        {
            Pmic_setBitField_b(&regData, WD_ANSW_ERR_SHIFT, WD_ANSW_ERR_MASK, true);
        }

        if (Pmic_validParamCheck(wdgErrStatus->validParams, PMIC_WDG_SEQ_ERR_VALID))
        {
            Pmic_setBitField_b(&regData, WD_SEQ_ERR_SHIFT, WD_SEQ_ERR_MASK, true);
        }

        if (Pmic_validParamCheck(wdgErrStatus->validParams, PMIC_WDG_ANSW_EARLY_ERR_VALID))
        {
            Pmic_setBitField_b(&regData, WD_ANSW_EARLY_SHIFT, WD_ANSW_EARLY_MASK, true);
        }

        if (Pmic_validParamCheck(wdgErrStatus->validParams, PMIC_WDG_TRIG_EARLY_ERR_VALID))
        {
            Pmic_setBitField_b(&regData, WD_TRIG_EARLY_SHIFT, WD_TRIG_EARLY_MASK, true);
        }

        if (Pmic_validParamCheck(wdgErrStatus->validParams, PMIC_WDG_TIMEOUT_ERR_VALID))
        {
            Pmic_setBitField_b(&regData, WD_TIMEOUT_SHIFT, WD_TIMEOUT_MASK, true);
        }

        if (Pmic_validParamCheck(wdgErrStatus->validParams, PMIC_WDG_LONG_WIN_TIMEOUT_ERR_VALID))
        {
            Pmic_setBitField_b(&regData, WD_LONGWIN_TIMEOUT_INT_SHIFT, WD_LONGWIN_TIMEOUT_INT_MASK, true);
        }

        status = Pmic_ioTxByte_CS(handle, WD_ERR_STATUS_REG, regData);
    }

    return status;
}

int32_t Pmic_wdgClrErrStatusAll(const Pmic_Handle_t *handle)
{
    int32_t status = Pmic_checkHandle(handle);

    if (status == PMIC_ST_SUCCESS)
    {
        // Write all 1s to clear all error status bits (write-1-to-clear)
        status = Pmic_ioTxByte_CS(handle, WD_ERR_STATUS_REG, 0xFFU);
    }

    return status;
}

int32_t Pmic_wdgGetErrStatus(const Pmic_Handle_t *handle, Pmic_WdgErrStatus_t *wdgErrStatus)
{
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    if ((status == PMIC_ST_SUCCESS) && (wdgErrStatus == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioRxByte_CS(handle, WD_ERR_STATUS_REG, &regData);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        if (Pmic_validParamCheck(wdgErrStatus->validParams, PMIC_WDG_RST_INT_VALID))
        {
            wdgErrStatus->rstInt = Pmic_getBitField_b(regData, WD_RST_INT_SHIFT);
        }

        if (Pmic_validParamCheck(wdgErrStatus->validParams, PMIC_WDG_FAIL_INT_VALID))
        {
            wdgErrStatus->failInt = Pmic_getBitField_b(regData, WD_FAIL_INT_SHIFT);
        }

        if (Pmic_validParamCheck(wdgErrStatus->validParams, PMIC_WDG_ANSW_ERR_VALID))
        {
            wdgErrStatus->answErr = Pmic_getBitField_b(regData, WD_ANSW_ERR_SHIFT);
        }

        if (Pmic_validParamCheck(wdgErrStatus->validParams, PMIC_WDG_SEQ_ERR_VALID))
        {
            wdgErrStatus->seqErr = Pmic_getBitField_b(regData, WD_SEQ_ERR_SHIFT);
        }

        if (Pmic_validParamCheck(wdgErrStatus->validParams, PMIC_WDG_ANSW_EARLY_ERR_VALID))
        {
            wdgErrStatus->answEarlyErr = Pmic_getBitField_b(regData, WD_ANSW_EARLY_SHIFT);
        }

        if (Pmic_validParamCheck(wdgErrStatus->validParams, PMIC_WDG_TRIG_EARLY_ERR_VALID))
        {
            wdgErrStatus->trigEarlyErr = Pmic_getBitField_b(regData, WD_TRIG_EARLY_SHIFT);
        }

        if (Pmic_validParamCheck(wdgErrStatus->validParams, PMIC_WDG_TIMEOUT_ERR_VALID))
        {
            wdgErrStatus->timeoutErr = Pmic_getBitField_b(regData, WD_TIMEOUT_SHIFT);
        }

        if (Pmic_validParamCheck(wdgErrStatus->validParams, PMIC_WDG_LONG_WIN_TIMEOUT_ERR_VALID))
        {
            wdgErrStatus->longWinTimeoutErr = Pmic_getBitField_b(regData, WD_LONGWIN_TIMEOUT_INT_SHIFT);
        }
    }

    return status;
}

int32_t Pmic_wdgGetFailCntStatus(const Pmic_Handle_t *handle, Pmic_WdgFailCntStatus_t *wdgFailCntStatus)
{
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    if ((status == PMIC_ST_SUCCESS) && (wdgFailCntStatus == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioRxByte_CS(handle, WD_FAIL_CNT_REG_REG, &regData);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        if (Pmic_validParamCheck(wdgFailCntStatus->validParams, PMIC_WDG_BAD_EVENT_VALID))
        {
            wdgFailCntStatus->badEvent = Pmic_getBitField_b(regData, WD_BAD_EVENT_SHIFT);
        }

        if (Pmic_validParamCheck(wdgFailCntStatus->validParams, PMIC_WDG_GOOD_EVENT_VALID))
        {
            wdgFailCntStatus->goodEvent = Pmic_getBitField_b(regData, WD_FIRST_OK_SHIFT);
        }

        if (Pmic_validParamCheck(wdgFailCntStatus->validParams, PMIC_WDG_FAIL_CNT_VALID))
        {
            wdgFailCntStatus->failCnt = Pmic_getBitField(regData, WD_FAIL_CNT_SHIFT, WD_FAIL_CNT_MASK);
        }
    }

    return status;
}
