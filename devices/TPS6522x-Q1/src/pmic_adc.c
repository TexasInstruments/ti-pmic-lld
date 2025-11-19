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
 * @file pmic_adc.c
 *
 * @brief PMIC LLD ADC module source file containing definitions to APIs that
 * interact with and control the PMIC ADC.
 */
#include "pmic.h"
#include "pmic_adc.h"

#include "pmic_io.h"

#include "regmap/adc.h"

int32_t Pmic_adcSetCfg(const Pmic_Handle_t *handle, const Pmic_AdcCfg_t *adcCfg)
{
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    if ((status == PMIC_ST_SUCCESS) && (adcCfg == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) && (adcCfg->validParams == 0U))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    // Read ADC_CTRL register
    if (status == PMIC_ST_SUCCESS)
    {
        Pmic_criticalSectionStart(handle);
        status = Pmic_ioRxByte(handle, ADC_CTRL_REGADDR, &regData);
    }

    // Set resistor divider enable
    if (Pmic_validParamStatusCheck(adcCfg->validParams, PMIC_ADC_RDIV_EN_VALID, status))
    {
        Pmic_setBitField_b(&regData, ADC_RDIV_EN_SHIFT, ADC_RDIV_EN_MASK, adcCfg->rDivEn);
    }

    // Set continuous conversion enable
    if (Pmic_validParamStatusCheck(adcCfg->validParams, PMIC_ADC_CONT_CONV_EN_VALID, status))
    {
        Pmic_setBitField_b(&regData, ADC_CONT_CONV_SHIFT, ADC_CONT_CONV_MASK, adcCfg->contConvEn);
    }

    // Set ADC source select
    if (Pmic_validParamStatusCheck(adcCfg->validParams, PMIC_ADC_SRC_SEL_VALID, status))
    {
        if (adcCfg->srcSel > PMIC_ADC_SRC_SEL_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
        else
        {
            Pmic_setBitField_b(&regData, ADC_THERMAL_SEL_SHIFT, ADC_THERMAL_SEL_MASK, adcCfg->srcSel);
        }
    }

    // Write ADC_CTRL register
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioTxByte(handle, ADC_CTRL_REGADDR, regData);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        Pmic_criticalSectionStop(handle);
    }

    return status;
}

int32_t Pmic_adcGetCfg(const Pmic_Handle_t *handle, Pmic_AdcCfg_t *adcCfg)
{
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    if ((status == PMIC_ST_SUCCESS) && (adcCfg == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    // Read ADC_CTRL register
    if (status == PMIC_ST_SUCCESS)
    {
        Pmic_criticalSectionStart(handle);
        status = Pmic_ioRxByte(handle, ADC_CTRL_REGADDR, &regData);
        Pmic_criticalSectionStop(handle);
    }

    // Get resistor divider enable
    if (status == PMIC_ST_SUCCESS)
    {
        adcCfg->rDivEn = Pmic_getBitField_b(regData, ADC_RDIV_EN_SHIFT);
    }

    // Get continuous conversion enable
    if (status == PMIC_ST_SUCCESS)
    {
        adcCfg->contConvEn = Pmic_getBitField_b(regData, ADC_CONT_CONV_SHIFT);
    }

    // Get ADC source select
    if (status == PMIC_ST_SUCCESS)
    {
        adcCfg->srcSel = Pmic_getBitField_b(regData, ADC_THERMAL_SEL_SHIFT);
    }

    return status;
}

int32_t Pmic_adcStartSingleConversion(const Pmic_Handle_t *handle)
{
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    // Read ADC_CTRL register to check if ADC is busy
    if (status == PMIC_ST_SUCCESS)
    {
        Pmic_criticalSectionStart(handle);
        status = Pmic_ioRxByte(handle, ADC_CTRL_REGADDR, &regData);
    }

    // Check if ADC is busy
    if (status == PMIC_ST_SUCCESS)
    {
        bool adcBusy = Pmic_getBitField_b(regData, ADC_STATUS_SHIFT);

        // If ADC is busy, return without starting conversion
        if (adcBusy)
        {
            Pmic_criticalSectionStop(handle);
            return PMIC_ST_SUCCESS;
        }
    }

    // Set ADC_START bit to start conversion
    if (status == PMIC_ST_SUCCESS)
    {
        Pmic_setBitField_b(&regData, ADC_START_SHIFT, ADC_START_MASK, true);
        status = Pmic_ioTxByte(handle, ADC_CTRL_REGADDR, regData);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        Pmic_criticalSectionStop(handle);
    }

    return status;
}

int32_t Pmic_adcStartSingleConversionBlocking(const Pmic_Handle_t *handle)
{
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;
    bool adcBusy = true;

    // Wait for ADC to become idle
    while ((status == PMIC_ST_SUCCESS) && adcBusy)
    {
        Pmic_criticalSectionStart(handle);
        status = Pmic_ioRxByte(handle, ADC_CTRL_REGADDR, &regData);

        if (status == PMIC_ST_SUCCESS)
        {
            adcBusy = Pmic_getBitField_b(regData, ADC_STATUS_SHIFT);
        }

        Pmic_criticalSectionStop(handle);
    }

    // Set ADC_START bit to start conversion
    if (status == PMIC_ST_SUCCESS)
    {
        Pmic_criticalSectionStart(handle);
        Pmic_setBitField_b(&regData, ADC_START_SHIFT, ADC_START_MASK, true);
        status = Pmic_ioTxByte(handle, ADC_CTRL_REGADDR, regData);
        Pmic_criticalSectionStop(handle);
    }

    return status;
}

int32_t Pmic_adcGetStatus(const Pmic_Handle_t *handle, bool *adcBusy)
{
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    if ((status == PMIC_ST_SUCCESS) && (adcBusy == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    // Read ADC_CTRL register
    if (status == PMIC_ST_SUCCESS)
    {
        Pmic_criticalSectionStart(handle);
        status = Pmic_ioRxByte(handle, ADC_CTRL_REGADDR, &regData);
        Pmic_criticalSectionStop(handle);
    }

    // Get ADC status (busy/idle)
    if (status == PMIC_ST_SUCCESS)
    {
        *adcBusy = Pmic_getBitField_b(regData, ADC_STATUS_SHIFT);
    }

    return status;
}

int32_t Pmic_adcGetResultCode(const Pmic_Handle_t *handle, uint16_t *adcResult)
{
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData1 = 0U;
    uint8_t regData2 = 0U;

    if ((status == PMIC_ST_SUCCESS) && (adcResult == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    // Read ADC result registers
    if (status == PMIC_ST_SUCCESS)
    {
        Pmic_criticalSectionStart(handle);
        status = Pmic_ioRxByte(handle, ADC_RESULT_REG_1_REGADDR, &regData1);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioRxByte(handle, ADC_RESULT_REG_2_REGADDR, &regData2);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        Pmic_criticalSectionStop(handle);
    }

    // Combine the two registers to form 12-bit result
    // ADC_RESULT_REG_1 contains bits [11:4]
    // ADC_RESULT_REG_2 contains bits [3:0] in bits [7:4]
    if (status == PMIC_ST_SUCCESS)
    {
        uint16_t result11_4 = (uint16_t)Pmic_getBitField(regData1, ADC_RESULT_11_4_SHIFT, ADC_RESULT_11_4_MASK);
        uint16_t result3_0 = (uint16_t)Pmic_getBitField(regData2, ADC_RESULT_3_0_SHIFT, ADC_RESULT_3_0_MASK);

        *adcResult = (uint16_t)((result11_4 << 4U) | result3_0);
    }

    return status;
}
