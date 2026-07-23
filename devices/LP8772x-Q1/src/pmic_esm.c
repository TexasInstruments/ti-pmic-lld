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
 * @file pmic_esm.c
 *
 * @brief PMIC LLD ESM module source file containing definitions to APIs that
 * interact with the PMIC ESM.
 */
#include "pmic.h"
#include "pmic_esm.h"

#include "pmic_io.h"

#include "regmap/esm.h"
#include "regmap/irq.h"

static int32_t ESM_setModeCfg(const Pmic_Handle_t *handle, const Pmic_EsmCfg_t *esmCfg)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;
    const uint32_t esmModeCfgValidParams = PMIC_CFG_ESM_MODE_VALID | PMIC_CFG_ESM_ERR_CNT_THR_VALID;

    // Read ESM_MCU_MODE_CFG register
    Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);
    if (Pmic_validParamCheck(esmCfg->validParams, esmModeCfgValidParams))
    {
        status = Pmic_ioRxByte(handle, ESM_MCU_MODE_CFG_REG, &regData);
    }

    // Modify ESM_MCU_MODE bit field
    if (Pmic_validParamStatusCheck(esmCfg->validParams, PMIC_CFG_ESM_MODE_VALID, status))
    {
        if (esmCfg->mode > PMIC_ESM_MODE_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
        else
        {
            Pmic_setBitField(&regData, ESM_MCU_MODE_SHIFT, ESM_MCU_MODE_MASK, esmCfg->mode);
        }
    }

    // Modify ESM_MCU_ERR_CNT_TH bit field
    if (Pmic_validParamStatusCheck(esmCfg->validParams, PMIC_CFG_ESM_ERR_CNT_THR_VALID, status))
    {
        if (esmCfg->errCntThr > PMIC_ESM_ERR_CNT_THR_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
        else
        {
            Pmic_setBitField(&regData, ESM_MCU_ERR_CNT_TH_SHIFT, ESM_MCU_ERR_CNT_TH_MASK, esmCfg->errCntThr);
        }
    }

    // Write new register value back to PMIC
    if (Pmic_validParamStatusCheck(esmCfg->validParams, esmModeCfgValidParams, status))
    {
        status = Pmic_ioTxByte(handle, ESM_MCU_MODE_CFG_REG, regData);
    }
    Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);

    return status;
}

static int32_t ESM_setDelayRegs(const Pmic_Handle_t *handle, const Pmic_EsmCfg_t *esmCfg)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    if (Pmic_validParamCheck(esmCfg->validParams, PMIC_CFG_ESM_DELAY1_VALID))
    {
        // Read ESM_MCU_DELAY1_REG register
        Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);
        status = Pmic_ioRxByte(handle, ESM_MCU_DELAY1_REG_REG, &regData);

        if (status == PMIC_ST_SUCCESS)
        {
            // Modify ESM_MCU_DELAY1 bit field
            Pmic_setBitField(&regData, ESM_MCU_DELAY1_SHIFT, ESM_MCU_DELAY1_MASK, esmCfg->delay1);

            // Write new register value back to PMIC
            status = Pmic_ioTxByte(handle, ESM_MCU_DELAY1_REG_REG, regData);
        }
        Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);
    }

    if (Pmic_validParamStatusCheck(esmCfg->validParams, PMIC_CFG_ESM_DELAY2_VALID, status))
    {
        // Read ESM_MCU_DELAY2_REG register
        Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);
        status = Pmic_ioRxByte(handle, ESM_MCU_DELAY2_REG_REG, &regData);

        if (status == PMIC_ST_SUCCESS)
        {
            // Modify ESM_MCU_DELAY2 bit field
            Pmic_setBitField(&regData, ESM_MCU_DELAY2_SHIFT, ESM_MCU_DELAY2_MASK, esmCfg->delay2);

            // Write new register value back to PMIC
            status = Pmic_ioTxByte(handle, ESM_MCU_DELAY2_REG_REG, regData);
        }
        Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);
    }

    return status;
}

static int32_t ESM_setHmaxHminRegs(const Pmic_Handle_t *handle, const Pmic_EsmCfg_t *esmCfg)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    if (Pmic_validParamCheck(esmCfg->validParams, PMIC_CFG_ESM_HMAX_VALID))
    {
        // Read ESM_MCU_HMAX_REG register
        Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);
        status = Pmic_ioRxByte(handle, ESM_MCU_HMAX_REG_REG, &regData);

        if (status == PMIC_ST_SUCCESS)
        {
            // Modify ESM_MCU_HMAX bit field
            Pmic_setBitField(&regData, ESM_MCU_HMAX_SHIFT, ESM_MCU_HMAX_MASK, esmCfg->hmax);

            // Write new register value back to PMIC
            status = Pmic_ioTxByte(handle, ESM_MCU_HMAX_REG_REG, regData);
        }
        Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);
    }

    if (Pmic_validParamStatusCheck(esmCfg->validParams, PMIC_CFG_ESM_HMIN_VALID, status))
    {
        // Read ESM_MCU_HMIN_REG register
        Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);
        status = Pmic_ioRxByte(handle, ESM_MCU_HMIN_REG_REG, &regData);

        if (status == PMIC_ST_SUCCESS)
        {
            // Modify ESM_MCU_HMIN bit field
            Pmic_setBitField(&regData, ESM_MCU_HMIN_SHIFT, ESM_MCU_HMIN_MASK, esmCfg->hmin);

            // Write new register value back to PMIC
            status = Pmic_ioTxByte(handle, ESM_MCU_HMIN_REG_REG, regData);
        }
        Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);
    }

    return status;
}

static int32_t ESM_setLmaxLminRegs(const Pmic_Handle_t *handle, const Pmic_EsmCfg_t *esmCfg)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    if (Pmic_validParamCheck(esmCfg->validParams, PMIC_CFG_ESM_LMAX_VALID))
    {
        // Read ESM_MCU_LMAX_REG register
        Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);
        status = Pmic_ioRxByte(handle, ESM_MCU_LMAX_REG_REG, &regData);

        if (status == PMIC_ST_SUCCESS)
        {
            // Modify ESM_MCU_LMAX bit field
            Pmic_setBitField(&regData, ESM_MCU_LMAX_SHIFT, ESM_MCU_LMAX_MASK, esmCfg->lmax);

            // Write new register value back to PMIC
            status = Pmic_ioTxByte(handle, ESM_MCU_LMAX_REG_REG, regData);
        }
        Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);
    }

    if (Pmic_validParamStatusCheck(esmCfg->validParams, PMIC_CFG_ESM_LMIN_VALID, status))
    {
        // Read ESM_MCU_LMIN_REG register
        Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);
        status = Pmic_ioRxByte(handle, ESM_MCU_LMIN_REG_REG, &regData);

        if (status == PMIC_ST_SUCCESS)
        {
            // Modify ESM_MCU_LMIN bit field
            Pmic_setBitField(&regData, ESM_MCU_LMIN_SHIFT, ESM_MCU_LMIN_MASK, esmCfg->lmin);

            // Write new register value back to PMIC
            status = Pmic_ioTxByte(handle, ESM_MCU_LMIN_REG_REG, regData);
        }
        Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);
    }

    return status;
}

int32_t Pmic_esmSetEnableState(const Pmic_Handle_t *handle, bool enable)
{
    uint8_t regData = 0U;
    int32_t status = Pmic_checkHandle(handle);

    Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);
    if (status == PMIC_ST_SUCCESS)
    {
        // Read ESM_MCU_MODE_CFG register
        status = Pmic_ioRxByte(handle, ESM_MCU_MODE_CFG_REG, &regData);

        if (status == PMIC_ST_SUCCESS)
        {
            // Modify ESM_MCU_EN bit
            Pmic_setBitField_b(&regData, ESM_MCU_EN_SHIFT, enable);

            // Write new register value back to PMIC
            status = Pmic_ioTxByte(handle, ESM_MCU_MODE_CFG_REG, regData);
        }
    }
    Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);

    return status;
}

int32_t Pmic_esmGetEnableState(const Pmic_Handle_t *handle, bool *isEnabled)
{
    uint8_t regData = 0U;
    int32_t status = Pmic_checkHandle(handle);

    if ((status == PMIC_ST_SUCCESS) && (isEnabled == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Read ESM_MCU_MODE_CFG register
        status = Pmic_ioRxByte_CS(handle, ESM_MCU_MODE_CFG_REG, &regData);

        if (status == PMIC_ST_SUCCESS)
        {
            // Extract ESM_MCU_EN bit
            *isEnabled = Pmic_getBitField_b(regData, ESM_MCU_EN_SHIFT);
        }
    }

    return status;
}

int32_t Pmic_esmSetStartState(const Pmic_Handle_t *handle, bool start)
{
    uint8_t regData = 0U;
    int32_t status = Pmic_checkHandle(handle);

    Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);
    if (status == PMIC_ST_SUCCESS)
    {
        // Read ESM_MCU_START_REG register
        status = Pmic_ioRxByte(handle, ESM_MCU_START_REG_REG, &regData);

        if (status == PMIC_ST_SUCCESS)
        {
            // Modify ESM_MCU_START bit
            Pmic_setBitField_b(&regData, ESM_MCU_START_SHIFT, start);

            // Write new register value back to PMIC
            status = Pmic_ioTxByte(handle, ESM_MCU_START_REG_REG, regData);
        }
    }
    Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);

    return status;
}

int32_t Pmic_esmGetStartState(const Pmic_Handle_t *handle, bool *started)
{
    uint8_t regData = 0U;
    int32_t status = Pmic_checkHandle(handle);

    if ((status == PMIC_ST_SUCCESS) && (started == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Read ESM_MCU_START_REG register
        status = Pmic_ioRxByte_CS(handle, ESM_MCU_START_REG_REG, &regData);

        if (status == PMIC_ST_SUCCESS)
        {
            // Extract ESM_MCU_START bit
            *started = Pmic_getBitField_b(regData, ESM_MCU_START_SHIFT);
        }
    }

    return status;
}

int32_t Pmic_esmSetCfg(const Pmic_Handle_t *handle, const Pmic_EsmCfg_t *esmCfg)
{
    int32_t status = Pmic_checkHandle(handle);

    if (status != PMIC_ST_SUCCESS)
    {
        return status;
    }

    if (esmCfg == NULL)
    {
        return PMIC_ST_ERR_NULL_PARAM;
    }

    if (esmCfg->validParams == 0U)
    {
        return PMIC_ST_ERR_INV_PARAM;
    }

    // Set ESM_MCU_DELAY1_REG and ESM_MCU_DELAY2_REG registers
    status = ESM_setDelayRegs(handle, esmCfg);

    // Set ESM_MCU_HMAX_REG and ESM_MCU_HMIN_REG registers
    if (status == PMIC_ST_SUCCESS)
    {
        status = ESM_setHmaxHminRegs(handle, esmCfg);
    }

    // Set ESM_MCU_LMAX_REG and ESM_MCU_LMIN_REG registers
    if (status == PMIC_ST_SUCCESS)
    {
        status = ESM_setLmaxLminRegs(handle, esmCfg);
    }

    // Set ESM_MCU_MODE_CFG register
    if (status == PMIC_ST_SUCCESS)
    {
        status = ESM_setModeCfg(handle, esmCfg);
    }

    return status;
}

static int32_t ESM_readModeCfg(const Pmic_Handle_t *handle, Pmic_EsmCfg_t *esmCfg)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    // Read ESM_MCU_MODE_CFG register
    if (Pmic_validParamCheck(esmCfg->validParams, PMIC_CFG_ESM_MODE_VALID | PMIC_CFG_ESM_ERR_CNT_THR_VALID))
    {
        status = Pmic_ioRxByte_CS(handle, ESM_MCU_MODE_CFG_REG, &regData);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Extract ESM_MCU_MODE bit field
        if (Pmic_validParamCheck(esmCfg->validParams, PMIC_CFG_ESM_MODE_VALID))
        {
            esmCfg->mode = Pmic_getBitField(regData, ESM_MCU_MODE_SHIFT, ESM_MCU_MODE_MASK);
        }

        // Extract ESM_MCU_ERR_CNT_TH bit field
        if (Pmic_validParamCheck(esmCfg->validParams, PMIC_CFG_ESM_ERR_CNT_THR_VALID))
        {
            esmCfg->errCntThr = Pmic_getBitField(regData, ESM_MCU_ERR_CNT_TH_SHIFT, ESM_MCU_ERR_CNT_TH_MASK);
        }
    }

    return status;
}

static int32_t ESM_readDelayRegs(const Pmic_Handle_t *handle, Pmic_EsmCfg_t *esmCfg)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    if (Pmic_validParamCheck(esmCfg->validParams, PMIC_CFG_ESM_DELAY1_VALID))
    {
        // Read ESM_MCU_DELAY1_REG register
        status = Pmic_ioRxByte_CS(handle, ESM_MCU_DELAY1_REG_REG, &regData);

        // Extract ESM_MCU_DELAY1 bit field
        if (status == PMIC_ST_SUCCESS)
        {
            esmCfg->delay1 = Pmic_getBitField(regData, ESM_MCU_DELAY1_SHIFT, ESM_MCU_DELAY1_MASK);
        }
    }

    if (Pmic_validParamStatusCheck(esmCfg->validParams, PMIC_CFG_ESM_DELAY2_VALID, status))
    {
        // Read ESM_MCU_DELAY2_REG register
        status = Pmic_ioRxByte_CS(handle, ESM_MCU_DELAY2_REG_REG, &regData);

        // Extract ESM_MCU_DELAY2 bit field
        if (status == PMIC_ST_SUCCESS)
        {
            esmCfg->delay2 = Pmic_getBitField(regData, ESM_MCU_DELAY2_SHIFT, ESM_MCU_DELAY2_MASK);
        }
    }

    return status;
}

static int32_t ESM_readHmaxHminRegs(const Pmic_Handle_t *handle, Pmic_EsmCfg_t *esmCfg)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    if (Pmic_validParamCheck(esmCfg->validParams, PMIC_CFG_ESM_HMAX_VALID))
    {
        // Read ESM_MCU_HMAX_REG register
        status = Pmic_ioRxByte_CS(handle, ESM_MCU_HMAX_REG_REG, &regData);

        // Extract ESM_MCU_HMAX bit field
        if (status == PMIC_ST_SUCCESS)
        {
            esmCfg->hmax = Pmic_getBitField(regData, ESM_MCU_HMAX_SHIFT, ESM_MCU_HMAX_MASK);
        }
    }

    if (Pmic_validParamStatusCheck(esmCfg->validParams, PMIC_CFG_ESM_HMIN_VALID, status))
    {
        // Read ESM_MCU_HMIN_REG register
        status = Pmic_ioRxByte_CS(handle, ESM_MCU_HMIN_REG_REG, &regData);

        // Extract ESM_MCU_HMIN bit field
        if (status == PMIC_ST_SUCCESS)
        {
            esmCfg->hmin = Pmic_getBitField(regData, ESM_MCU_HMIN_SHIFT, ESM_MCU_HMIN_MASK);
        }
    }

    return status;
}

static int32_t ESM_readLmaxLminRegs(const Pmic_Handle_t *handle, Pmic_EsmCfg_t *esmCfg)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    if (Pmic_validParamCheck(esmCfg->validParams, PMIC_CFG_ESM_LMAX_VALID))
    {
        // Read ESM_MCU_LMAX_REG register
        status = Pmic_ioRxByte_CS(handle, ESM_MCU_LMAX_REG_REG, &regData);

        // Extract ESM_MCU_LMAX bit field
        if (status == PMIC_ST_SUCCESS)
        {
            esmCfg->lmax = Pmic_getBitField(regData, ESM_MCU_LMAX_SHIFT, ESM_MCU_LMAX_MASK);
        }
    }

    if (Pmic_validParamStatusCheck(esmCfg->validParams, PMIC_CFG_ESM_LMIN_VALID, status))
    {
        // Read ESM_MCU_LMIN_REG register
        status = Pmic_ioRxByte_CS(handle, ESM_MCU_LMIN_REG_REG, &regData);

        // Extract ESM_MCU_LMIN bit field
        if (status == PMIC_ST_SUCCESS)
        {
            esmCfg->lmin = Pmic_getBitField(regData, ESM_MCU_LMIN_SHIFT, ESM_MCU_LMIN_MASK);
        }
    }

    return status;
}

int32_t Pmic_esmGetCfg(const Pmic_Handle_t *handle, Pmic_EsmCfg_t *esmCfg)
{
    int32_t status = Pmic_checkHandle(handle);

    if (status != PMIC_ST_SUCCESS)
    {
        return status;
    }

    if (esmCfg == NULL)
    {
        return PMIC_ST_ERR_NULL_PARAM;
    }

    if (esmCfg->validParams == 0U)
    {
        return PMIC_ST_ERR_INV_PARAM;
    }

    // Read ESM_MCU_MODE_CFG register
    status = ESM_readModeCfg(handle, esmCfg);

    // Read ESM_MCU_DELAY1_REG and ESM_MCU_DELAY2_REG registers
    if (status == PMIC_ST_SUCCESS)
    {
        status = ESM_readDelayRegs(handle, esmCfg);
    }

    // Read ESM_MCU_HMAX_REG and ESM_MCU_HMIN_REG registers
    if (status == PMIC_ST_SUCCESS)
    {
        status = ESM_readHmaxHminRegs(handle, esmCfg);
    }

    // Read ESM_MCU_LMAX_REG and ESM_MCU_LMIN_REG registers
    if (status == PMIC_ST_SUCCESS)
    {
        status = ESM_readLmaxLminRegs(handle, esmCfg);
    }

    return status;
}

int32_t Pmic_esmGetErrCnt(const Pmic_Handle_t *handle, uint8_t *esmErrCnt)
{
    uint8_t regData = 0U;
    int32_t status = Pmic_checkHandle(handle);

    if ((status == PMIC_ST_SUCCESS) && (esmErrCnt == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Read ESM_MCU_ERR_CNT_REG register
        status = Pmic_ioRxByte_CS(handle, ESM_MCU_ERR_CNT_REG_REG, &regData);

        // Extract ESM_MCU_ERR_CNT bit field
        if (status == PMIC_ST_SUCCESS)
        {
            *esmErrCnt = Pmic_getBitField(regData, ESM_MCU_ERR_CNT_SHIFT, ESM_MCU_ERR_CNT_MASK);
        }
    }

    return status;
}

int32_t Pmic_esmStart(const Pmic_Handle_t *handle)
{
    return Pmic_esmSetStartState(handle, true);
}

int32_t Pmic_esmStop(const Pmic_Handle_t *handle)
{
    return Pmic_esmSetStartState(handle, false);
}

int32_t Pmic_esmGetStatus(const Pmic_Handle_t *handle, Pmic_EsmStatus_t *esmStat)
{
    uint8_t regData = 0U;
    int32_t status = Pmic_checkHandle(handle);

    if ((status == PMIC_ST_SUCCESS) && (esmStat == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) &&
        ((esmStat->validParams == 0U) || (esmStat->validParams > PMIC_ESM_STATUS_ALL_VALID)))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioRxByte_CS(handle, INT_ESM_REG, &regData);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        if (Pmic_validParamCheck(esmStat->validParams, PMIC_ESM_RST_INT_VALID))
        {
            esmStat->rstInt = Pmic_getBitField_b(regData, ESM_MCU_RST_INT_SHIFT);
        }
        if (Pmic_validParamCheck(esmStat->validParams, PMIC_ESM_FAIL_INT_VALID))
        {
            esmStat->failInt = Pmic_getBitField_b(regData, ESM_MCU_FAIL_INT_SHIFT);
        }
        if (Pmic_validParamCheck(esmStat->validParams, PMIC_ESM_PIN_INT_VALID))
        {
            esmStat->pinInt = Pmic_getBitField_b(regData, ESM_MCU_PIN_INT_SHIFT);
        }
    }

    return status;
}

int32_t Pmic_esmClrStatus(const Pmic_Handle_t *handle, const Pmic_EsmStatus_t *esmStat)
{
    uint8_t regData = 0U;
    int32_t status = Pmic_checkHandle(handle);

    if ((status == PMIC_ST_SUCCESS) && (esmStat == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) &&
        ((esmStat->validParams == 0U) || (esmStat->validParams > PMIC_ESM_STATUS_ALL_VALID)))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        if (Pmic_validParamCheck(esmStat->validParams, PMIC_ESM_RST_INT_VALID))
        {
            Pmic_setBitField(&regData, ESM_MCU_RST_INT_SHIFT, ESM_MCU_RST_INT_MASK, 1U);
        }
        if (Pmic_validParamCheck(esmStat->validParams, PMIC_ESM_FAIL_INT_VALID))
        {
            Pmic_setBitField(&regData, ESM_MCU_FAIL_INT_SHIFT, ESM_MCU_FAIL_INT_MASK, 1U);
        }
        if (Pmic_validParamCheck(esmStat->validParams, PMIC_ESM_PIN_INT_VALID))
        {
            Pmic_setBitField(&regData, ESM_MCU_PIN_INT_SHIFT, ESM_MCU_PIN_INT_MASK, 1U);
        }

        status = Pmic_ioTxByte_CS(handle, INT_ESM_REG, regData);
    }

    return status;
}
