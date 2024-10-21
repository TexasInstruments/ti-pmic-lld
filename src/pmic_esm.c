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
 * @file pmic_irq.c
 *
 * @brief PMIC LLD IRQ module source file containing definitions to APIs that
 * interact with PMIC IRQs.
 */
#include "pmic.h"
#include "pmic_io.h"
#include "pmic_esm.h"

#include "regmap/esm.h"
#include "regmap/irq.h"
#include "regmap/core.h"

static int32_t ESM_setLminLmax(const Pmic_CoreHandle_t *pmicHandle, const Pmic_EsmCfg_t *esmCfg)
{
    int32_t status = PMIC_ST_SUCCESS;

    Pmic_criticalSectionStart(pmicHandle);

    // Set LMIN (the entire register is the bit field)
    if (Pmic_validParamCheck(esmCfg->validParams, PMIC_ESM_LMIN_VALID))
    {
        status = Pmic_ioTxByte(pmicHandle, PMIC_CMD_WR_SAFETY_ERR_PWM_LMIN, esmCfg->lmin);
    }

    // Set LMAX (the entire register is the bit field)
    if (Pmic_validParamStatusCheck(esmCfg->validParams, PMIC_ESM_LMAX_VALID, status))
    {
        status = Pmic_ioTxByte(pmicHandle, PMIC_CMD_WR_SAFETY_ERR_PWM_LMAX, esmCfg->lmax);
    }

    Pmic_criticalSectionStop(pmicHandle);

    return status;
}

static int32_t ESM_setHminHmax(const Pmic_CoreHandle_t *pmicHandle, const Pmic_EsmCfg_t *esmCfg)
{
    int32_t status = PMIC_ST_SUCCESS;

    Pmic_criticalSectionStart(pmicHandle);

    // Set HMIN (the entire register is the bit field)
    if (Pmic_validParamCheck(esmCfg->validParams, PMIC_ESM_HMIN_VALID))
    {
        status = Pmic_ioTxByte(pmicHandle, PMIC_CMD_WR_SAFETY_ERR_PWM_HMIN, esmCfg->hmin);
    }

    // Set HMAX (the entire register is the bit field)
    if (Pmic_validParamStatusCheck(esmCfg->validParams, PMIC_ESM_HMAX_VALID, status))
    {
        status = Pmic_ioTxByte(pmicHandle, PMIC_CMD_WR_SAFETY_ERR_PWM_HMAX, esmCfg->hmax);
    }

    Pmic_criticalSectionStop(pmicHandle);

    return status;
}

static int32_t ESM_setThresholds(const Pmic_CoreHandle_t *pmicHandle, const Pmic_EsmCfg_t *esmCfg)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    // Set MCU error count threshold
    if (Pmic_validParamCheck(esmCfg->validParams, PMIC_ESM_MCU_ERR_CNT_THR_VALID))
    {
        // Read SAFETY_ERR_CFG_2 register
        Pmic_criticalSectionStart(pmicHandle);
        status = Pmic_ioRxByte(pmicHandle, PMIC_CMD_RD_SAFETY_ERR_CFG_2, &regData);

        // Modify MCU_ERR_CNT_TH[2:0] bit field
        if (status == PMIC_ST_SUCCESS)
        {
            if (esmCfg->mcuErrCntThr > PMIC_ESM_MCU_ERR_CNT_THR_MAX)
            {
                status = PMIC_ST_ERR_INV_PARAM;
            }
            else
            {
                Pmic_setBitField(&regData, MCU_ERR_CNT_TH_SHIFT, MCU_ERR_CNT_TH_MASK, esmCfg->mcuErrCntThr);
            }
        }

        // Write new register value back to PMIC
        if (status == PMIC_ST_SUCCESS)
        {
            status = Pmic_ioTxByte(pmicHandle, PMIC_CMD_WR_SAFETY_ERR_CFG_2, regData);
        }
        Pmic_criticalSectionStop(pmicHandle);
    }

    // Set ESM LMIN and LMAX
    if (Pmic_validParamStatusCheck(esmCfg->validParams, PMIC_ESM_LMIN_VALID | PMIC_ESM_LMAX_VALID, status))
    {
        status = ESM_setLminLmax(pmicHandle, esmCfg);
    }

    // Set ESM HMIN and HMAX
    if (Pmic_validParamStatusCheck(esmCfg->validParams, PMIC_ESM_HMIN_VALID | PMIC_ESM_HMAX_VALID, status))
    {
        status = ESM_setHminHmax(pmicHandle, esmCfg);
    }

    return status;
}

static int32_t ESM_setOtherCfg(const Pmic_CoreHandle_t *pmicHandle, const Pmic_EsmCfg_t *esmCfg)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    // Set ESM enable
    if (Pmic_validParamCheck(esmCfg->validParams, PMIC_ESM_ENABLE_VALID))
    {
        // Read SAFETY_CHECK_CTRL register
        Pmic_criticalSectionStart(pmicHandle);
        status = Pmic_ioRxByte(pmicHandle, PMIC_CMD_RD_SAFETY_CHECK_CTRL, &regData);

        if (status == PMIC_ST_SUCCESS)
        {
            // Modify NO_ERROR bit field
            Pmic_setBitField_b(&regData, NO_ERROR_SHIFT, esmCfg->enable);

            // Write new register value back to PMIC
            status = Pmic_ioTxByte(pmicHandle, PMIC_CMD_WR_SAFETY_CHECK_CTRL, regData);
        }
        Pmic_criticalSectionStop(pmicHandle);
    }

    // Set PWM enable and lock step reset enable
    if (Pmic_validParamStatusCheck(esmCfg->validParams, PMIC_ESM_PWM_EN_VALID | PMIC_ESM_LOCK_STEP_RST_EN_VALID, status))
    {
        // Read SAFETY_FUNC_CFG register
        Pmic_criticalSectionStart(pmicHandle);
        status = Pmic_ioRxByte(pmicHandle, PMIC_CMD_RD_SAFETY_FUNC_CFG, &regData);

        if (status == PMIC_ST_SUCCESS)
        {
            // Modify ESM_CFG bit field
            if (Pmic_validParamCheck(esmCfg->validParams, PMIC_ESM_PWM_EN_VALID))
            {
                Pmic_setBitField_b(&regData, ESM_CFG_SHIFT, esmCfg->pwmEn);
            }

            // Modify LOCK_STEP_RST_EN bit field
            if (Pmic_validParamCheck(esmCfg->validParams, PMIC_ESM_LOCK_STEP_RST_EN_VALID))
            {
                Pmic_setBitField_b(&regData, LOCK_STEP_RST_EN_SHIFT, esmCfg->lockStepRstEn);
            }

            // Write new register value back to PMIC
            status = Pmic_ioTxByte(pmicHandle, PMIC_CMD_WR_SAFETY_FUNC_CFG, regData);
        }
        Pmic_criticalSectionStop(pmicHandle);
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

    // Set MCU error count threshold along with L{MIN,MAX}, H{MIN,MAX}
    if (status == PMIC_ST_SUCCESS)
    {
        status = ESM_setThresholds(pmicHandle, esmCfg);
    }

    // Set other ESM configurations like ESM enable, PWM enable, and lock step reset enable
    if (status == PMIC_ST_SUCCESS)
    {
        status = ESM_setOtherCfg(pmicHandle, esmCfg);
    }

    return status;
}

static int32_t ESM_getOtherCfg(const Pmic_CoreHandle_t *pmicHandle, Pmic_EsmCfg_t *esmCfg)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    // Get ESM enable
    if (Pmic_validParamCheck(esmCfg->validParams, PMIC_ESM_ENABLE_VALID))
    {
        // Read SAFETY_CHECK_CTRL register
        status = Pmic_ioRxByte_CS(pmicHandle, PMIC_CMD_RD_SAFETY_CHECK_CTRL, &regData);

        // Extract NO_ERROR bit field
        if (status == PMIC_ST_SUCCESS)
        {
            esmCfg->enable = Pmic_getBitField_b(regData, NO_ERROR_SHIFT);
        }
    }

    // Get PWM enable and lock step reset enable
    if (Pmic_validParamStatusCheck(esmCfg->validParams, PMIC_ESM_PWM_EN_VALID | PMIC_ESM_LOCK_STEP_RST_EN_VALID, status))
    {
        // Read SAFETY_FUNC_CFG register
        status = Pmic_ioRxByte_CS(pmicHandle, PMIC_CMD_RD_SAFETY_FUNC_CFG, &regData);

        if (status == PMIC_ST_SUCCESS)
        {
            // Extract ESM_CFG bit field
            if (Pmic_validParamCheck(esmCfg->validParams, PMIC_ESM_PWM_EN_VALID))
            {
                esmCfg->pwmEn = Pmic_getBitField_b(regData, ESM_CFG_SHIFT);
            }

            // Extract LOCK_STEP_RST_EN bit field
            if (Pmic_validParamCheck(esmCfg->validParams, PMIC_ESM_LOCK_STEP_RST_EN_VALID))
            {
                esmCfg->lockStepRstEn = Pmic_getBitField_b(regData, LOCK_STEP_RST_EN_SHIFT);
            }
        }
    }

    return status;
}

static int32_t ESM_getLminLmax(const Pmic_CoreHandle_t *pmicHandle, Pmic_EsmCfg_t *esmCfg)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    // Get LMIN
    if (Pmic_validParamCheck(esmCfg->validParams, PMIC_ESM_LMIN_VALID))
    {
        // Read SAFETY_ERR_PWM_LMIN register
        status = Pmic_ioRxByte_CS(pmicHandle, PMIC_CMD_RD_SAFETY_ERR_PWM_LMIN, &regData);

        // Extract PWMLMIN[7:0] bit field (the entire register is the bit field)
        if (status == PMIC_ST_SUCCESS)
        {
            esmCfg->lmin = regData;
        }
    }

    // Get LMAX
    if (Pmic_validParamStatusCheck(esmCfg->validParams, PMIC_ESM_LMAX_VALID, status))
    {
        // Read SAFETY_ERR_PWM_LMAX register
        status = Pmic_ioRxByte_CS(pmicHandle, PMIC_CMD_RD_SAFETY_ERR_PWM_LMAX, &regData);

        // Extract PWMLMAX[7:0] bit field (the entire register is the bit field)
        if (status == PMIC_ST_SUCCESS)
        {
            esmCfg->lmax = regData;
        }
    }

    return status;
}

static int32_t ESM_getHminHmax(const Pmic_CoreHandle_t *pmicHandle, Pmic_EsmCfg_t *esmCfg)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    // Get HMIN
    if (Pmic_validParamCheck(esmCfg->validParams, PMIC_ESM_HMIN_VALID))
    {
        // Read SAFETY_ERR_PWM_HMIN register
        status = Pmic_ioRxByte_CS(pmicHandle, PMIC_CMD_RD_SAFETY_ERR_PWM_HMIN, &regData);

        // Extract PWMHMIN[7:0] bit field (the entire register is the bit field)
        if (status == PMIC_ST_SUCCESS)
        {
            esmCfg->hmin = regData;
        }
    }

    // Get HMAX
    if (Pmic_validParamStatusCheck(esmCfg->validParams, PMIC_ESM_HMAX_VALID, status))
    {
        // Read SAFETY_ERR_PWM_HMAX register
        status = Pmic_ioRxByte_CS(pmicHandle, PMIC_CMD_RD_SAFETY_ERR_PWM_HMAX, &regData);

        // Extract PWMHMAX[7:0] bit field (the entire register is the bit field)
        if (status == PMIC_ST_SUCCESS)
        {
            esmCfg->hmax = regData;
        }
    }

    return status;
}

static int32_t ESM_getThresholds(const Pmic_CoreHandle_t *pmicHandle, Pmic_EsmCfg_t *esmCfg)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    // Get MCU error count threshold
    if (Pmic_validParamCheck(esmCfg->validParams, PMIC_ESM_MCU_ERR_CNT_THR_VALID))
    {
        // Read SAFETY_ERR_CFG_2 register
        status = Pmic_ioRxByte_CS(pmicHandle, PMIC_CMD_RD_SAFETY_ERR_CFG_2, &regData);

        // Extract MCU_ERR_CNT_TH[2:0] bit field
        if (status == PMIC_ST_SUCCESS)
        {
            esmCfg->mcuErrCntThr = Pmic_getBitField(regData, MCU_ERR_CNT_TH_SHIFT, MCU_ERR_CNT_TH_MASK);
        }
    }

    // Get ESM LMIN and LMAX
    if (Pmic_validParamStatusCheck(esmCfg->validParams, PMIC_ESM_LMIN_VALID | PMIC_ESM_LMAX_VALID, status))
    {
        status = ESM_getLminLmax(pmicHandle, esmCfg);
    }

    // Get ESM HMIN and HMAX
    if (Pmic_validParamStatusCheck(esmCfg->validParams, PMIC_ESM_HMIN_VALID | PMIC_ESM_HMAX_VALID, status))
    {
        status = ESM_getHminHmax(pmicHandle, esmCfg);
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

    // Get MCU error count threshold along with L{MIN,MAX}, H{MIN,MAX}
    if (status == PMIC_ST_SUCCESS)
    {
        status = ESM_getThresholds(pmicHandle, esmCfg);
    }

    // Get other ESM configurations like ESM enable, PWM enable, and lock step reset enable
    if (status == PMIC_ST_SUCCESS)
    {
        status = ESM_getOtherCfg(pmicHandle, esmCfg);
    }

    return status;
}

int32_t Pmic_esmGetMcuErrCnt(const Pmic_CoreHandle_t *pmicHandle, uint8_t *mcuErrCnt)
{
    uint8_t regData = 0U;
    int32_t status = Pmic_checkPmicCoreHandle(pmicHandle);

    if ((status == PMIC_ST_SUCCESS) && (mcuErrCnt == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    // Read SAFETY_ERR_STAT_2 register
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioRxByte_CS(pmicHandle, PMIC_CMD_RD_SAFETY_ERR_STAT_2, &regData);
    }

    // Extract MCU_ERR_CNT[3:0] bit field
    if (status == PMIC_ST_SUCCESS)
    {
        *mcuErrCnt = Pmic_getBitField(regData, MCU_ERR_CNT_SHIFT, MCU_ERR_CNT_MASK);
    }

    return status;
}
