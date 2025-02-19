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

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#include "pmic.h"
#include "pmic_common.h"

#include "pmic_esm.h"
#include "pmic_io.h"
#include "regmap/esm.h"

/* ========================================================================== */
/*                        Interface Implementations                           */
/* ========================================================================== */

int32_t Pmic_esmSetStartState(Pmic_CoreHandle_t *handle, bool start)
{
    int32_t status = Pmic_checkPmicCoreHandle(handle);
    uint8_t regData = 0U;

    Pmic_criticalSectionStart(handle);
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioRxByte(handle, ESM_CTRL_REG, &regData);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        Pmic_setBitField_b(&regData, ESM_START_SHIFT, start);
        status = Pmic_ioTxByte(handle, ESM_CTRL_REG, regData);
    }
    Pmic_criticalSectionStop(handle);

    return status;
}

int32_t Pmic_esmStart(Pmic_CoreHandle_t *handle)
{
    return Pmic_esmSetStartState(handle, PMIC_ENABLE);
}

int32_t Pmic_esmStop(Pmic_CoreHandle_t *handle)
{
    return Pmic_esmSetStartState(handle, PMIC_DISABLE);
}

int32_t Pmic_esmGetStartState(Pmic_CoreHandle_t *handle, bool *start)
{
    int32_t status = Pmic_checkPmicCoreHandle(handle);
    uint8_t regData = 0U;

    if ((status == PMIC_ST_SUCCESS) && (start == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        Pmic_criticalSectionStart(handle);
        status = Pmic_ioRxByte(handle, ESM_CTRL_REG, &regData);
        Pmic_criticalSectionStop(handle);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        *start = Pmic_getBitField_b(regData, ESM_START_SHIFT);
    }

    return status;
}

static int32_t ESM_setDelays(Pmic_CoreHandle_t *handle, const Pmic_EsmCfg_t *esmCfg)
{
    int32_t status = PMIC_ST_SUCCESS;

    Pmic_criticalSectionStart(handle);
    if (Pmic_validParamCheck(esmCfg->validParams, PMIC_CFG_ESM_DELAY1_VALID))
    {
        status = Pmic_ioTxByte(handle, ESM_DELAY1_REG, esmCfg->delay1);
    }

    if (Pmic_validParamStatusCheck(esmCfg->validParams, PMIC_CFG_ESM_DELAY2_VALID, status))
    {
        status = Pmic_ioTxByte(handle, ESM_DELAY2_REG, esmCfg->delay2);
    }
    Pmic_criticalSectionStop(handle);

    return status;
}

static int32_t ESM_setHMaxMinLMaxMin(Pmic_CoreHandle_t *handle, const Pmic_EsmCfg_t *esmCfg)
{
    int32_t status = PMIC_ST_SUCCESS;

    Pmic_criticalSectionStart(handle);
    if (Pmic_validParamCheck(esmCfg->validParams, PMIC_CFG_ESM_HMAX_VALID))
    {
        status = Pmic_ioTxByte(handle, ESM_HMAX_CFG_REG, esmCfg->hmax);
    }

    if (Pmic_validParamStatusCheck(esmCfg->validParams, PMIC_CFG_ESM_HMIN_VALID, status))
    {
        status = Pmic_ioTxByte(handle, ESM_HMIN_CFG_REG, esmCfg->hmin);
    }

    if (Pmic_validParamStatusCheck(esmCfg->validParams, PMIC_CFG_ESM_LMAX_VALID, status))
    {
        status = Pmic_ioTxByte(handle, ESM_LMAX_CFG_REG, esmCfg->lmax);
    }

    if (Pmic_validParamStatusCheck(esmCfg->validParams, PMIC_CFG_ESM_LMIN_VALID, status))
    {
        status = Pmic_ioTxByte(handle, ESM_LMIN_CFG_REG, esmCfg->lmin);
    }
    Pmic_criticalSectionStop(handle);

    return status;
}

static int32_t ESM_setCfg1(Pmic_CoreHandle_t *handle, const Pmic_EsmCfg_t *esmCfg)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    Pmic_criticalSectionStart(handle);
    status = Pmic_ioRxByte(handle, ESM_CFG1_REG, &regData);

    if (Pmic_validParamStatusCheck(esmCfg->validParams, PMIC_CFG_ESM_ENABLE_VALID, status))
    {
        Pmic_setBitField_b(&regData, ESM_EN_SHIFT, esmCfg->enable);
    }

    if (Pmic_validParamStatusCheck(esmCfg->validParams, PMIC_CFG_ESM_MODE_VALID, status))
    {
        if (esmCfg->mode > PMIC_ESM_MODE_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
        else
        {
            Pmic_setBitField(&regData, ESM_CFG_SHIFT, ESM_CFG_MASK, esmCfg->mode);
        }
    }

    if (Pmic_validParamStatusCheck(esmCfg->validParams, PMIC_CFG_ESM_ERR_THR_VALID, status))
    {
        if (esmCfg->errThr > PMIC_ESM_ERR_THR_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
        else
        {
            Pmic_setBitField(&regData, ESM_ERR_TH_SHIFT, ESM_ERR_TH_MASK, esmCfg->errThr);
        }
    }

    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioTxByte(handle, ESM_CFG1_REG, regData);
    }
    Pmic_criticalSectionStop(handle);

    return status;
}

static int32_t ESM_setCfg2(Pmic_CoreHandle_t *handle, const Pmic_EsmCfg_t *esmCfg)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    Pmic_criticalSectionStart(handle);
    status = Pmic_ioRxByte(handle, ESM_CFG2_REG, &regData);

    if (Pmic_validParamStatusCheck(esmCfg->validParams, PMIC_CFG_ESM_POLARITY_VALID, status))
    {
        if (esmCfg->polarity > PMIC_ESM_POLARITY_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
        else
        {
            Pmic_setBitField(&regData, ESM_LVL_POL_SHIFT, ESM_LVL_POL_MASK, esmCfg->polarity);
        }
    }

    if (Pmic_validParamStatusCheck(esmCfg->validParams, PMIC_CFG_ESM_DEGLITCH_VALID, status))
    {
        if (esmCfg->deglitch > PMIC_ESM_DEGLITCH_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
        else
        {
            Pmic_setBitField(&regData, ESM_DGL_SHIFT, ESM_DGL_MASK, esmCfg->deglitch);
        }
    }

    if (Pmic_validParamStatusCheck(esmCfg->validParams, PMIC_CFG_ESM_TIME_BASE_VALID, status))
    {
        if (esmCfg->timeBase > PMIC_ESM_TIME_BASE_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
        else
        {
            Pmic_setBitField(&regData, ESM_TIME_CFG_SHIFT, ESM_TIME_CFG_MASK, esmCfg->timeBase);
        }
    }

    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioTxByte(handle, ESM_CFG2_REG, regData);
    }
    Pmic_criticalSectionStop(handle);

    return status;
}

static int32_t ESM_setCtrlConfig(Pmic_CoreHandle_t *handle, const Pmic_EsmCfg_t *esmCfg)
{
    int32_t status = PMIC_ST_SUCCESS;

    if (Pmic_validParamCheck(esmCfg->validParams, PMIC_CFG_ESM_MODE_VALID) ||
        Pmic_validParamCheck(esmCfg->validParams, PMIC_CFG_ESM_ERR_THR_VALID))
    {
        status = ESM_setCfg1(handle, esmCfg);
    }

    if (Pmic_validParamCheck(esmCfg->validParams, PMIC_CFG_ESM_POLARITY_VALID) ||
        Pmic_validParamCheck(esmCfg->validParams, PMIC_CFG_ESM_DEGLITCH_VALID) ||
        Pmic_validParamCheck(esmCfg->validParams, PMIC_CFG_ESM_TIME_BASE_VALID))
    {
        status = ESM_setCfg2(handle, esmCfg);
    }

    return status;
}

int32_t Pmic_esmSetCfg(Pmic_CoreHandle_t *handle, const Pmic_EsmCfg_t *esmCfg)
{
    int32_t status = Pmic_checkPmicCoreHandle(handle);

    if ((status == PMIC_ST_SUCCESS) && (esmCfg == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) && (esmCfg->validParams == 0U))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        status = ESM_setDelays(handle, esmCfg);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        status = ESM_setHMaxMinLMaxMin(handle, esmCfg);
    }

    // Set mode, error threshold, polarity, deglitch, timebase
    if (status == PMIC_ST_SUCCESS)
    {
        status = ESM_setCtrlConfig(handle, esmCfg);
    }

    return status;
}

static int32_t ESM_getDelays(Pmic_CoreHandle_t *handle, Pmic_EsmCfg_t *esmCfg)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    Pmic_criticalSectionStart(handle);
    if (Pmic_validParamCheck(esmCfg->validParams, PMIC_CFG_ESM_DELAY1_VALID))
    {
        status = Pmic_ioRxByte(handle, ESM_DELAY1_REG, &regData);

        if (status == PMIC_ST_SUCCESS)
        {
            esmCfg->delay1 = regData;
        }
    }

    if (Pmic_validParamStatusCheck(esmCfg->validParams, PMIC_CFG_ESM_DELAY2_VALID, status))
    {
        status = Pmic_ioRxByte(handle, ESM_DELAY2_REG, &regData);

        if (status == PMIC_ST_SUCCESS)
        {
            esmCfg->delay2 = regData;
        }
    }
    Pmic_criticalSectionStop(handle);

    return status;
}

static int32_t ESM_getHMaxMinLMaxMin(Pmic_CoreHandle_t *handle, Pmic_EsmCfg_t *esmCfg)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    Pmic_criticalSectionStart(handle);
    if (Pmic_validParamCheck(esmCfg->validParams, PMIC_CFG_ESM_HMAX_VALID))
    {
        status = Pmic_ioRxByte(handle, ESM_HMAX_CFG_REG, &regData);

        if (status == PMIC_ST_SUCCESS)
        {
            esmCfg->hmax = regData;
        }
    }

    if (Pmic_validParamStatusCheck(esmCfg->validParams, PMIC_CFG_ESM_HMIN_VALID, status))
    {
        status = Pmic_ioRxByte(handle, ESM_HMIN_CFG_REG, &regData);

        if (status == PMIC_ST_SUCCESS)
        {
            esmCfg->hmin = regData;
        }
    }

    if (Pmic_validParamStatusCheck(esmCfg->validParams, PMIC_CFG_ESM_LMAX_VALID, status))
    {
        status = Pmic_ioRxByte(handle, ESM_LMAX_CFG_REG, &regData);

        if (status == PMIC_ST_SUCCESS)
        {
            esmCfg->lmax = regData;
        }
    }

    if (Pmic_validParamStatusCheck(esmCfg->validParams, PMIC_CFG_ESM_LMIN_VALID, status))
    {
        status = Pmic_ioRxByte(handle, ESM_LMIN_CFG_REG, &regData);

        if (status == PMIC_ST_SUCCESS)
        {
            esmCfg->lmin = regData;
        }
    }
    Pmic_criticalSectionStop(handle);

    return status;
}

static int32_t ESM_getCfg1(Pmic_CoreHandle_t *handle, Pmic_EsmCfg_t *esmCfg)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    Pmic_criticalSectionStart(handle);
    status = Pmic_ioRxByte(handle, ESM_CFG1_REG, &regData);
    Pmic_criticalSectionStop(handle);

    if (status == PMIC_ST_SUCCESS)
    {
        if (Pmic_validParamCheck(esmCfg->validParams, PMIC_CFG_ESM_ENABLE_VALID))
        {
            esmCfg->enable = Pmic_getBitField_b(regData, ESM_EN_SHIFT);
        }

        if (Pmic_validParamCheck(esmCfg->validParams, PMIC_CFG_ESM_MODE_VALID))
        {
            esmCfg->mode = Pmic_getBitField(regData, ESM_CFG_SHIFT, ESM_CFG_MASK);
        }

        if (Pmic_validParamCheck(esmCfg->validParams, PMIC_CFG_ESM_ERR_THR_VALID))
        {
            esmCfg->errThr = Pmic_getBitField(regData, ESM_ERR_TH_SHIFT, ESM_ERR_TH_MASK);
        }
    }

    return status;
}

static int32_t ESM_getCfg2(Pmic_CoreHandle_t *handle, Pmic_EsmCfg_t *esmCfg)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    Pmic_criticalSectionStart(handle);
    status = Pmic_ioRxByte(handle, ESM_CFG2_REG, &regData);
    Pmic_criticalSectionStop(handle);

    if (status == PMIC_ST_SUCCESS)
    {
        if (Pmic_validParamCheck(esmCfg->validParams, PMIC_CFG_ESM_POLARITY_VALID))
        {
            esmCfg->polarity = Pmic_getBitField(regData, ESM_LVL_POL_SHIFT, ESM_LVL_POL_MASK);
        }

        if (Pmic_validParamCheck(esmCfg->validParams, PMIC_CFG_ESM_DEGLITCH_VALID))
        {
            esmCfg->deglitch = Pmic_getBitField(regData, ESM_DGL_SHIFT, ESM_DGL_MASK);
        }

        if (Pmic_validParamCheck(esmCfg->validParams, PMIC_CFG_ESM_TIME_BASE_VALID))
        {
            esmCfg->timeBase = Pmic_getBitField(regData, ESM_TIME_CFG_SHIFT, ESM_TIME_CFG_MASK);
        }
    }

    return status;
}

static int32_t ESM_getCtrlConfig(Pmic_CoreHandle_t *handle, Pmic_EsmCfg_t *esmCfg)
{
    int32_t status = PMIC_ST_SUCCESS;

    if (Pmic_validParamCheck(esmCfg->validParams, PMIC_CFG_ESM_MODE_VALID) ||
        Pmic_validParamCheck(esmCfg->validParams, PMIC_CFG_ESM_ERR_THR_VALID))
    {
        status = ESM_getCfg1(handle, esmCfg);
    }

    if (Pmic_validParamCheck(esmCfg->validParams, PMIC_CFG_ESM_POLARITY_VALID) ||
        Pmic_validParamCheck(esmCfg->validParams, PMIC_CFG_ESM_DEGLITCH_VALID) ||
        Pmic_validParamCheck(esmCfg->validParams, PMIC_CFG_ESM_TIME_BASE_VALID))
    {
        status = ESM_getCfg2(handle, esmCfg);
    }

    return status;
}

int32_t Pmic_esmGetCfg(Pmic_CoreHandle_t *handle, Pmic_EsmCfg_t *esmCfg)
{
    int32_t status = Pmic_checkPmicCoreHandle(handle);

    if ((status == PMIC_ST_SUCCESS) && (esmCfg == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) && (esmCfg->validParams == 0U))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        status = ESM_getDelays(handle, esmCfg);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        status = ESM_getHMaxMinLMaxMin(handle, esmCfg);
    }

    // Get mode, error threshold, polarity, deglitch, timebase
    if (status == PMIC_ST_SUCCESS)
    {
        status = ESM_getCtrlConfig(handle, esmCfg);
    }

    return status;
}

int32_t Pmic_esmGetStatus(Pmic_CoreHandle_t *handle, Pmic_EsmStatus_t *esmStat)
{
    int32_t status = Pmic_checkPmicCoreHandle(handle);
    uint8_t regData = 0U;

    if ((status == PMIC_ST_SUCCESS) && (esmStat == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) && (esmStat->validParams == 0U))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        Pmic_criticalSectionStart(handle);
        status = Pmic_ioRxByte(handle, ESM_ERR_STAT_REG, &regData);
        Pmic_criticalSectionStop(handle);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        if (Pmic_validParamCheck(esmStat->validParams, PMIC_DELAY2_ERR_VALID))
        {
            esmStat->delay2Err = Pmic_getBitField_b(regData, ESM_DLY2_ERR_SHIFT);
        }

        if (Pmic_validParamCheck(esmStat->validParams, PMIC_DELAY1_ERR_VALID))
        {
            esmStat->delay1Err = Pmic_getBitField_b(regData, ESM_DLY1_ERR_SHIFT);
        }

        if (Pmic_validParamCheck(esmStat->validParams, PMIC_ESM_ERR_VALID))
        {
            esmStat->esmErr = Pmic_getBitField_b(regData, ESM_ERR_SHIFT);
        }

        if (Pmic_validParamCheck(esmStat->validParams, PMIC_ERR_CNT_VALID))
        {
            esmStat->errCnt = Pmic_getBitField(regData, ESM_ERR_CNT_SHIFT, ESM_ERR_CNT_MASK);
        }
    }

    return status;
}

int32_t Pmic_esmClrStatus(Pmic_CoreHandle_t *handle, const Pmic_EsmStatus_t *esmStat)
{
    int32_t status = Pmic_checkPmicCoreHandle(handle);
    uint8_t regData = 0U;

    if ((status == PMIC_ST_SUCCESS) && (esmStat == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) && (esmStat->validParams == 0U))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    // ESM statuses are W1C (write 1 to clear)
    if (status == PMIC_ST_SUCCESS)
    {
        if (Pmic_validParamCheck(esmStat->validParams, PMIC_DELAY2_ERR_VALID))
        {
            Pmic_setBitField_b(&regData, ESM_DLY2_ERR_SHIFT, esmStat->delay2Err);
        }

        if (Pmic_validParamCheck(esmStat->validParams, PMIC_DELAY1_ERR_VALID))
        {
            Pmic_setBitField_b(&regData, ESM_DLY1_ERR_SHIFT, esmStat->delay1Err);
        }

        if (Pmic_validParamCheck(esmStat->validParams, PMIC_ESM_ERR_VALID))
        {
            Pmic_setBitField_b(&regData, ESM_ERR_SHIFT, esmStat->esmErr);
        }

        Pmic_criticalSectionStart(handle);
        status = Pmic_ioTxByte(handle, ESM_ERR_STAT_REG, regData);
        Pmic_criticalSectionStop(handle);
    }

    return status;
}
