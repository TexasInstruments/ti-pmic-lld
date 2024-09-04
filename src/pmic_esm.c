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
 * @file pmic_esm.c
 *
 * @brief PMIC LLD ESM module source file containing definitions to APIs that
 * interact with the PMIC ESM.
 */
#include "pmic.h"
#include "pmic_esm.h"

#include "pmic_io.h"

#include "regmap/irq.h"
#include "regmap/esm.h"

static int32_t ESM_setModeCfg(const Pmic_CoreHandle_t *pmicHandle, const Pmic_EsmCfg_t *esmCfg)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;
    const uint32_t esmModeCfgValidParams = PMIC_ESM_MODE_VALID | PMIC_ESM_ENABLE_VALID | PMIC_ESM_ERR_CNT_THR_VALID;

    // Read ESM_MODE_CFG register
    if (Pmic_validParamCheck(esmCfg->validParams, esmModeCfgValidParams))
    {
        Pmic_criticalSectionStart(pmicHandle);
        status = Pmic_ioRxByte(pmicHandle, PMIC_ESM_MODE_CFG_REGADDR, &regData);
        Pmic_criticalSectionStop(pmicHandle);
    }

    // Modify ESM_MCU_MODE bit field
    if (Pmic_validParamStatusCheck(esmCfg->validParams, PMIC_ESM_MODE_VALID, status))
    {
        if (esmCfg->mode > ESM_MODE_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
        else
        {
            Pmic_setBitField(&regData, PMIC_ESM_MCU_MODE_SHIFT, PMIC_ESM_MCU_MODE_MASK, esmCfg->mode);
        }
    }

    // Modify ESM_MCU_EN bit field
    if (Pmic_validParamStatusCheck(esmCfg->validParams, PMIC_ESM_ENABLE_VALID, status))
    {
        Pmic_setBitField_b(&regData, PMIC_ESM_MCU_EN_SHIFT, PMIC_ESM_MCU_EN_MASK, esmCfg->enable);
    }

    // Modify ESM_MCU_ERR_CNT_TH bit field
    if (Pmic_validParamStatusCheck(esmCfg->validParams, PMIC_ESM_ERR_CNT_THR_VALID, status))
    {
        if (esmCfg->errCntThr > ESM_ERR_CNT_THR_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
        else
        {
            Pmic_setBitField(&regData, PMIC_ESM_MCU_ERR_CNT_TH_SHIFT, PMIC_ESM_MCU_ERR_CNT_TH_MASK, esmCfg->errCntThr);
        }
    }

    // Write new register value back to PMIC
    if (Pmic_validParamStatusCheck(esmCfg->validParams, esmModeCfgValidParams, status))
    {
        Pmic_criticalSectionStart(pmicHandle);
        status = Pmic_ioTxByte(pmicHandle, PMIC_ESM_MODE_CFG_REGADDR, regData);
        Pmic_criticalSectionStop(pmicHandle);
    }

    return status;
}

static int32_t ESM_setDelayRegs(const Pmic_CoreHandle_t *pmicHandle, const Pmic_EsmCfg_t *esmCfg)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    if (Pmic_validParamCheck(esmCfg->validParams, PMIC_ESM_DELAY1_VALID))
    {
        // Read ESM_DELAY1_REG register
        Pmic_criticalSectionStart(pmicHandle);
        status = Pmic_ioRxByte(pmicHandle, PMIC_ESM_DELAY1_REG_REGADDR, &regData);
        Pmic_criticalSectionStop(pmicHandle);

        if (status == PMIC_ST_SUCCESS)
        {
            // Modify ESM_MCU_DELAY1 bit field
            Pmic_setBitField(&regData, PMIC_ESM_MCU_DELAY1_SHIFT, PMIC_ESM_MCU_DELAY1_MASK, esmCfg->delay1);

            // Write new register value back to PMIC
            Pmic_criticalSectionStart(pmicHandle);
            status = Pmic_ioTxByte(pmicHandle, PMIC_ESM_DELAY1_REG_REGADDR, regData);
            Pmic_criticalSectionStop(pmicHandle);
        }
    }

    if (Pmic_validParamStatusCheck(esmCfg->validParams, PMIC_ESM_DELAY2_VALID, status))
    {
        // Read ESM_DELAY2_REG register
        Pmic_criticalSectionStart(pmicHandle);
        status = Pmic_ioRxByte(pmicHandle, PMIC_ESM_DELAY2_REG_REGADDR, &regData);
        Pmic_criticalSectionStop(pmicHandle);

        if (status == PMIC_ST_SUCCESS)
        {
            // Modify ESM_MCU_DELAY2 bit field
            Pmic_setBitField(&regData, PMIC_ESM_MCU_DELAY2_SHIFT, PMIC_ESM_MCU_DELAY2_MASK, esmCfg->delay2);

            // Write new register value back to PMIC
            Pmic_criticalSectionStart(pmicHandle);
            status = Pmic_ioTxByte(pmicHandle, PMIC_ESM_DELAY2_REG_REGADDR, regData);
            Pmic_criticalSectionStop(pmicHandle);
        }
    }

    return status;
}

static int32_t ESM_setHmaxHminRegs(const Pmic_CoreHandle_t *pmicHandle, const Pmic_EsmCfg_t *esmCfg)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    if (Pmic_validParamCheck(esmCfg->validParams, PMIC_ESM_HMAX_VALID))
    {
        // Read ESM_HMAX_REG register
        Pmic_criticalSectionStart(pmicHandle);
        status = Pmic_ioRxByte(pmicHandle, PMIC_ESM_HMAX_REG_REGADDR, &regData);
        Pmic_criticalSectionStop(pmicHandle);

        if (status == PMIC_ST_SUCCESS)
        {
            // Modify ESM_MCU_HMAX bit field
            Pmic_setBitField(&regData, PMIC_ESM_MCU_HMAX_SHIFT, PMIC_ESM_MCU_HMAX_MASK, esmCfg->hmax);

            // Write new register value back to PMIC
            Pmic_criticalSectionStart(pmicHandle);
            status = Pmic_ioTxByte(pmicHandle, PMIC_ESM_HMAX_REG_REGADDR, regData);
            Pmic_criticalSectionStop(pmicHandle);
        }
    }

    if (Pmic_validParamStatusCheck(esmCfg->validParams, PMIC_ESM_HMIN_VALID, status))
    {
        // Read ESM_HMIN_REG register
        Pmic_criticalSectionStart(pmicHandle);
        status = Pmic_ioRxByte(pmicHandle, PMIC_ESM_HMIN_REG_REGADDR, &regData);
        Pmic_criticalSectionStop(pmicHandle);

        if (status == PMIC_ST_SUCCESS)
        {
            // Modify ESM_MCU_HMIN bit field
            Pmic_setBitField(&regData, PMIC_ESM_MCU_HMIN_SHIFT, PMIC_ESM_MCU_HMIN_MASK, esmCfg->hmin);

            // Write new register value back to PMIC
            Pmic_criticalSectionStart(pmicHandle);
            status = Pmic_ioTxByte(pmicHandle, PMIC_ESM_HMIN_REG_REGADDR, regData);
            Pmic_criticalSectionStop(pmicHandle);
        }
    }

    return status;
}

static int32_t ESM_setLmaxLminRegs(const Pmic_CoreHandle_t *pmicHandle, const Pmic_EsmCfg_t *esmCfg)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    if (Pmic_validParamCheck(esmCfg->validParams, PMIC_ESM_LMAX_VALID))
    {
        // Read ESM_LMAX_REG register
        Pmic_criticalSectionStart(pmicHandle);
        status = Pmic_ioRxByte(pmicHandle, PMIC_ESM_LMAX_REG_REGADDR, &regData);
        Pmic_criticalSectionStop(pmicHandle);

        if (status == PMIC_ST_SUCCESS)
        {
            // Modify ESM_MCU_LMAX bit field
            Pmic_setBitField(&regData, PMIC_ESM_MCU_LMAX_SHIFT, PMIC_ESM_MCU_LMAX_MASK, esmCfg->lmax);

            // Write new register value back to PMIC
            Pmic_criticalSectionStart(pmicHandle);
            status = Pmic_ioTxByte(pmicHandle, PMIC_ESM_LMAX_REG_REGADDR, regData);
            Pmic_criticalSectionStop(pmicHandle);
        }
    }

    if (Pmic_validParamStatusCheck(esmCfg->validParams, PMIC_ESM_LMIN_VALID, status))
    {
        // Read ESM_LMIN_REG register
        Pmic_criticalSectionStart(pmicHandle);
        status = Pmic_ioRxByte(pmicHandle, PMIC_ESM_LMIN_REG_REGADDR, &regData);
        Pmic_criticalSectionStop(pmicHandle);

        if (status == PMIC_ST_SUCCESS)
        {
            // Modify ESM_MCU_LMIN bit field
            Pmic_setBitField(&regData, PMIC_ESM_MCU_LMIN_SHIFT, PMIC_ESM_MCU_LMIN_MASK, esmCfg->lmin);

            // Write new register value back to PMIC
            Pmic_criticalSectionStart(pmicHandle);
            status = Pmic_ioTxByte(pmicHandle, PMIC_ESM_LMIN_REG_REGADDR, regData);
            Pmic_criticalSectionStop(pmicHandle);
        }
    }

    return status;
}

int32_t Pmic_esmSetCfg(const Pmic_CoreHandle_t *pmicHandle, const Pmic_EsmCfg_t *esmCfg)
{
    int32_t status = Pmic_checkPmicCoreHandle(pmicHandle);

    if ((status == PMIC_ST_SUCCESS) && (esmCfg == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) && (esmCfg->validParams == 0U))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    // Set ESM_DELAY1_REG and ESM_DELAY2_REG registers
    if (status == PMIC_ST_SUCCESS)
    {
        status = ESM_setDelayRegs(pmicHandle, esmCfg);
    }

    // Set ESM_HMAX_REG and ESM_HMIN_REG registers
    if (status == PMIC_ST_SUCCESS)
    {
        status = ESM_setHmaxHminRegs(pmicHandle, esmCfg);
    }

    // Set ESM_LMAX_REG and ESM_LMIN_REG registers
    if (status == PMIC_ST_SUCCESS)
    {
        status = ESM_setLmaxLminRegs(pmicHandle, esmCfg);
    }

    // Set ESM_MODE_CFG register
    if (status == PMIC_ST_SUCCESS)
    {
        status = ESM_setModeCfg(pmicHandle, esmCfg);
    }

    return status;
}

static int32_t ESM_readModeCfg(const Pmic_CoreHandle_t *pmicHandle, Pmic_EsmCfg_t *esmCfg)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    // Read ESM_MODE_CFG register
    if (Pmic_validParamCheck(esmCfg->validParams, PMIC_ESM_MODE_VALID | PMIC_ESM_ENABLE_VALID | PMIC_ESM_ERR_CNT_THR_VALID))
    {
        Pmic_criticalSectionStart(pmicHandle);
        status = Pmic_ioRxByte(pmicHandle, PMIC_ESM_MODE_CFG_REGADDR, &regData);
        Pmic_criticalSectionStop(pmicHandle);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Extract ESM_MCU_MODE bit field
        if (Pmic_validParamCheck(esmCfg->validParams, PMIC_ESM_MODE_VALID))
        {
            esmCfg->mode = Pmic_getBitField(regData, PMIC_ESM_MCU_MODE_SHIFT, PMIC_ESM_MCU_MODE_MASK);
        }

        // Extract ESM_MCU_EN bit field
        if (Pmic_validParamCheck(esmCfg->validParams, PMIC_ESM_ENABLE_VALID))
        {
            esmCfg->enable = Pmic_getBitField_b(regData, PMIC_ESM_MCU_EN_SHIFT);
        }

        // Extract ESM_MCU_ERR_CNT_TH bit field
        if (Pmic_validParamCheck(esmCfg->validParams, PMIC_ESM_ERR_CNT_THR_VALID))
        {
            esmCfg->errCntThr = Pmic_getBitField(regData, PMIC_ESM_MCU_ERR_CNT_TH_SHIFT, PMIC_ESM_MCU_ERR_CNT_TH_MASK);
        }
    }

    return status;
}

static int32_t ESM_readDelayRegs(const Pmic_CoreHandle_t *pmicHandle, Pmic_EsmCfg_t *esmCfg)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    if (Pmic_validParamCheck(esmCfg->validParams, PMIC_ESM_DELAY1_VALID))
    {
        // Read ESM_DELAY1_REG register
        Pmic_criticalSectionStart(pmicHandle);
        status = Pmic_ioRxByte(pmicHandle, PMIC_ESM_DELAY1_REG_REGADDR, &regData);
        Pmic_criticalSectionStop(pmicHandle);

        // Extract ESM_MCU_DELAY1 bit field
        if (status == PMIC_ST_SUCCESS)
        {
            esmCfg->delay1 = Pmic_getBitField(regData, PMIC_ESM_MCU_DELAY1_SHIFT, PMIC_ESM_MCU_DELAY1_MASK);
        }
    }

    if (Pmic_validParamStatusCheck(esmCfg->validParams, PMIC_ESM_DELAY2_VALID, status))
    {
        // Read ESM_DELAY2_REG register
        Pmic_criticalSectionStart(pmicHandle);
        status = Pmic_ioRxByte(pmicHandle, PMIC_ESM_DELAY2_REG_REGADDR, &regData);
        Pmic_criticalSectionStop(pmicHandle);

        // Extract ESM_MCU_DELAY2 bit field
        if (status == PMIC_ST_SUCCESS)
        {
            esmCfg->delay2 = Pmic_getBitField(regData, PMIC_ESM_MCU_DELAY2_SHIFT, PMIC_ESM_MCU_DELAY2_MASK);
        }
    }

    return status;
}

static int32_t ESM_readHmaxHminRegs(const Pmic_CoreHandle_t *pmicHandle, Pmic_EsmCfg_t *esmCfg)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    if (Pmic_validParamCheck(esmCfg->validParams, PMIC_ESM_HMAX_VALID))
    {
        // Read ESM_HMAX_REG register
        Pmic_criticalSectionStart(pmicHandle);
        status = Pmic_ioRxByte(pmicHandle, PMIC_ESM_HMAX_REG_REGADDR, &regData);
        Pmic_criticalSectionStop(pmicHandle);

        // Extract ESM_MCU_HMAX bit field
        if (status == PMIC_ST_SUCCESS)
        {
            esmCfg->hmax = Pmic_getBitField(regData, PMIC_ESM_MCU_HMAX_SHIFT, PMIC_ESM_MCU_HMAX_MASK);
        }
    }

    if (Pmic_validParamStatusCheck(esmCfg->validParams, PMIC_ESM_HMIN_VALID, status))
    {
        // Read ESM_HMIN_REG register
        Pmic_criticalSectionStart(pmicHandle);
        status = Pmic_ioRxByte(pmicHandle, PMIC_ESM_HMIN_REG_REGADDR, &regData);
        Pmic_criticalSectionStop(pmicHandle);

        // Extract ESM_MCU_HMIN bit field
        if (status == PMIC_ST_SUCCESS)
        {
            esmCfg->hmin = Pmic_getBitField(regData, PMIC_ESM_MCU_HMIN_SHIFT, PMIC_ESM_MCU_HMIN_MASK);
        }
    }

    return status;
}

static int32_t ESM_readLmaxLminRegs(const Pmic_CoreHandle_t *pmicHandle, Pmic_EsmCfg_t *esmCfg)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    if (Pmic_validParamCheck(esmCfg->validParams, PMIC_ESM_LMAX_VALID))
    {
        // Read ESM_LMAX_REG register
        Pmic_criticalSectionStart(pmicHandle);
        status = Pmic_ioRxByte(pmicHandle, PMIC_ESM_LMAX_REG_REGADDR, &regData);
        Pmic_criticalSectionStop(pmicHandle);

        // Extract ESM_MCU_LMAX bit field
        if (status == PMIC_ST_SUCCESS)
        {
            esmCfg->lmax = Pmic_getBitField(regData, PMIC_ESM_MCU_LMAX_SHIFT, PMIC_ESM_MCU_LMAX_MASK);
        }
    }

    if (Pmic_validParamStatusCheck(esmCfg->validParams, PMIC_ESM_LMIN_VALID, status))
    {
        // Read ESM_LMIN_REG register
        Pmic_criticalSectionStart(pmicHandle);
        status = Pmic_ioRxByte(pmicHandle, PMIC_ESM_LMIN_REG_REGADDR, &regData);
        Pmic_criticalSectionStop(pmicHandle);

        // Extract ESM_MCU_LMIN bit field
        if (status == PMIC_ST_SUCCESS)
        {
            esmCfg->lmin = Pmic_getBitField(regData, PMIC_ESM_MCU_LMIN_SHIFT, PMIC_ESM_MCU_LMIN_MASK);
        }
    }

    return status;
}

int32_t Pmic_esmGetCfg(const Pmic_CoreHandle_t *pmicHandle, Pmic_EsmCfg_t *esmCfg)
{
    int32_t status = Pmic_checkPmicCoreHandle(pmicHandle);

    if ((status == PMIC_ST_SUCCESS) && (esmCfg == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) && (esmCfg->validParams == 0U))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    // Read ESM_MODE_CFG register
    if (status == PMIC_ST_SUCCESS)
    {
        status = ESM_readModeCfg(pmicHandle, esmCfg);
    }

    // Read ESM_DELAY1_REG and ESM_DELAY2_REG registers
    if (status == PMIC_ST_SUCCESS)
    {
        status = ESM_readDelayRegs(pmicHandle, esmCfg);
    }

    // Read ESM_HMAX_REG and ESM_HMIN_REG registers
    if (status == PMIC_ST_SUCCESS)
    {
        status = ESM_readHmaxHminRegs(pmicHandle, esmCfg);
    }

    // Read ESM_LMAX_REG and ESM_LMIN_REG registers
    if (status == PMIC_ST_SUCCESS)
    {
        status = ESM_readLmaxLminRegs(pmicHandle, esmCfg);
    }

    return status;
}

int32_t Pmic_esmStartStop(const Pmic_CoreHandle_t *pmicHandle, bool start)
{
    uint8_t regData = 0U;
    int32_t status = Pmic_checkPmicCoreHandle(pmicHandle);

    if (status == PMIC_ST_SUCCESS)
    {
        // Read ESM_START_REG register
        Pmic_criticalSectionStart(pmicHandle);
        status = Pmic_ioRxByte(pmicHandle, PMIC_ESM_START_REG_REGADDR, &regData);
        Pmic_criticalSectionStop(pmicHandle);

        if (status == PMIC_ST_SUCCESS)
        {
            // Modify ESM_MCU_START bit
            Pmic_setBitField_b(&regData, PMIC_ESM_MCU_START_SHIFT, PMIC_ESM_MCU_START_MASK, start);

            // Write new register value back to PMIC
            Pmic_criticalSectionStart(pmicHandle);
            status = Pmic_ioTxByte(pmicHandle, PMIC_ESM_START_REG_REGADDR, regData);
            Pmic_criticalSectionStop(pmicHandle);
        }
    }

    return status;
}

int32_t Pmic_esmGetStartStop(const Pmic_CoreHandle_t *pmicHandle, bool *start)
{
    uint8_t regData = 0U;
    int32_t status = Pmic_checkPmicCoreHandle(pmicHandle);

    if ((status == PMIC_ST_SUCCESS) && (start == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Read ESM_START_REG register
        Pmic_criticalSectionStart(pmicHandle);
        status = Pmic_ioRxByte(pmicHandle, PMIC_ESM_START_REG_REGADDR, &regData);
        Pmic_criticalSectionStop(pmicHandle);

        if (status == PMIC_ST_SUCCESS)
        {
            // Extract ESM_MCU_START bit
            *start = Pmic_getBitField_b(regData, PMIC_ESM_MCU_START_SHIFT);
        }
    }

    return status;
}

int32_t Pmic_esmStart(const Pmic_CoreHandle_t *pmicHandle)
{
    return Pmic_esmStartStop(pmicHandle, PMIC_ESM_START);
}

int32_t Pmic_esmStop(const Pmic_CoreHandle_t *pmicHandle)
{
    return Pmic_esmStartStop(pmicHandle, PMIC_ESM_STOP);
}

int32_t Pmic_esmGetStat(const Pmic_CoreHandle_t *pmicHandle, Pmic_EsmStat_t *esmStat)
{
    uint8_t regData = 0U;
    int32_t status = Pmic_checkPmicCoreHandle(pmicHandle);

    if ((status == PMIC_ST_SUCCESS) && (esmStat == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) &&
        ((esmStat->validParams == 0U) || (esmStat->validParams > PMIC_ESM_STATUS_ALL_VALID)))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    // Read INT_ESM register
    if (status == PMIC_ST_SUCCESS)
    {
        Pmic_criticalSectionStart(pmicHandle);
        status = Pmic_ioRxByte(pmicHandle, PMIC_INT_ESM_REGADDR, &regData);
        Pmic_criticalSectionStop(pmicHandle);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Extract ESM_MCU_RST_INT bit field
        if (Pmic_validParamCheck(esmStat->validParams, PMIC_ESM_RST_INT_VALID))
        {
            esmStat->rstInt = Pmic_getBitField_b(regData, PMIC_ESM_MCU_RST_INT_SHIFT);
        }

        // Extract ESM_MCU_FAIL_INT bit field
        if (Pmic_validParamCheck(esmStat->validParams, PMIC_ESM_FAIL_INT_VALID))
        {
            esmStat->failInt = Pmic_getBitField_b(regData, PMIC_ESM_MCU_FAIL_INT_SHIFT);
        }

        // Extract ESM_MCU_PIN_INT bit field
        if (Pmic_validParamCheck(esmStat->validParams, PMIC_ESM_PIN_INT_VALID))
        {
            esmStat->pinInt = Pmic_getBitField_b(regData, PMIC_ESM_MCU_PIN_INT_SHIFT);
        }
    }

    return status;
}

int32_t Pmic_esmClrStat(const Pmic_CoreHandle_t *pmicHandle, const Pmic_EsmStat_t *esmStat)
{
    uint8_t regData = 0U;
    int32_t status = Pmic_checkPmicCoreHandle(pmicHandle);

    if ((status == PMIC_ST_SUCCESS) && (esmStat == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) &&
        ((esmStat->validParams == 0U) || (esmStat->validParams > PMIC_ESM_STATUS_ALL_VALID)))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    // All ESM error statuses are W1C - write 1 to clear
    if (status == PMIC_ST_SUCCESS)
    {
        // Clear ESM_MCU_RST_INT
        if (Pmic_validParamCheck(esmStat->validParams, PMIC_ESM_RST_INT_VALID))
        {
            Pmic_setBitField(&regData, PMIC_ESM_MCU_RST_INT_SHIFT, PMIC_ESM_MCU_RST_INT_MASK, 1U);
        }

        // Clear ESM_MCU_FAIL_INT
        if (Pmic_validParamCheck(esmStat->validParams, PMIC_ESM_FAIL_INT_VALID))
        {
            Pmic_setBitField(&regData, PMIC_ESM_MCU_FAIL_INT_SHIFT, PMIC_ESM_MCU_FAIL_INT_MASK, 1U);
        }

        // Clear ESM_MCU_PIN_INT
        if (Pmic_validParamCheck(esmStat->validParams, PMIC_ESM_PIN_INT_VALID))
        {
            Pmic_setBitField(&regData, PMIC_ESM_MCU_PIN_INT_SHIFT, PMIC_ESM_MCU_PIN_INT_MASK, 1U);
        }

        // Write new register value back to PMIC
        Pmic_criticalSectionStart(pmicHandle);
        status = Pmic_ioTxByte(pmicHandle, PMIC_INT_ESM_REGADDR, regData);
        Pmic_criticalSectionStop(pmicHandle);
    }

    return status;
}

int32_t Pmic_esmGetErrCnt(const Pmic_CoreHandle_t *pmicHandle, uint8_t *errCnt)
{
    uint8_t regData = 0U;
    int32_t status = Pmic_checkPmicCoreHandle(pmicHandle);

    if ((status == PMIC_ST_SUCCESS) && (errCnt == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Read ESM_ERR_CNT_REG register
        Pmic_criticalSectionStart(pmicHandle);
        status = Pmic_ioRxByte(pmicHandle, PMIC_ESM_ERR_CNT_REG_REGADDR, &regData);
        Pmic_criticalSectionStop(pmicHandle);

        // Extract ESM_MCU_ERR_CNT bit field
        if (status == PMIC_ST_SUCCESS)
        {
            *errCnt = Pmic_getBitField(regData, PMIC_ESM_MCU_ERR_CNT_SHIFT, PMIC_ESM_MCU_ERR_CNT_MASK);
        }
    }

    return status;
}
