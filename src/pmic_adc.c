/******************************************************************************
 * Copyright (c) 2023 Texas Instruments Incorporated - http://www.ti.com
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
 *  \file   pmic_adc.c
 *
 *  \brief  This source file contains the PMIC ADC API definitions for
 *          setting/getting the configurations of the ADC, starting ADC
 *          conversions, getting ADC status, and getting ADC result.
 */
#include "pmic_types.h"
#include "pmic_adc.h"
#include "pmic_core_priv.h"
#include "pmic_io_priv.h"
#include "pmic_adc_priv.h"

/**
 *  \brief      This function is used to verify the PMIC handle that is passed into ADC APIs.
 *
 *  \param      pPmicCoreHandle     [IN]    PMIC interface handle
 *
 *  \return     Success code if PMIC handle is valid, error code otherwise.
 *              For valid success/error codes, refer to \ref Pmic_ErrorCodes
 */
static int32_t Pmic_ADCParamCheck_pmicHandle(const Pmic_CoreHandle_t *pPmicCoreHandle)
{
    int32_t status = PMIC_ST_SUCCESS;

    // NULL handle
    if (pPmicCoreHandle == NULL)
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    // ADC subsystem disabled
    if ((status == PMIC_ST_SUCCESS) && (pPmicCoreHandle->pPmic_SubSysInfo->adcEnable == (bool)false))
    {
        status = PMIC_ST_ERR_INV_DEVICE;
    }

    return status;
}

/**
 *  \brief      This function is used to verify whether an ADC CFG reference is valid.
 *
 *  \param      pAdcCfg     [IN]    Pointer to ADC configuration struct
 *
 *  \return     Success code if ADC CFG reference is valid, error code otherwise.
 *              For valid success/error codes, refer to \ref Pmic_ErrorCodes
 */
static int32_t Pmic_ADCParamCheck_pAdcCfg(const Pmic_adcCfg_t *pAdcCfg)
{
    int32_t status = PMIC_ST_SUCCESS;

    // NULL ADC CFG
    if (pAdcCfg == NULL)
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    // No validparams
    if ((status == PMIC_ST_SUCCESS) && (pAdcCfg->validParams == 0U))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    return status;
}

int32_t Pmic_ADCSetConfiguration(Pmic_CoreHandle_t *pPmicCoreHandle, const Pmic_adcCfg_t adcCfg)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t adcCtrlRegData = 0U;

    // Parameter check
    status = Pmic_ADCParamCheck_pmicHandle(pPmicCoreHandle);
    if ((status == PMIC_ST_SUCCESS) && (adcCfg.validParams == 0U))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    // Start critical section before read-modify-write
    Pmic_criticalSectionStart(pPmicCoreHandle);

    // Read ADC_CTRL register
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_ADC_CTRL_REGADDR, &adcCtrlRegData);
    }

    // Modify ADC_CTRL bit fields with respect to validParams
    if (status == PMIC_ST_SUCCESS)
    {
        if (pmic_validParamCheck(adcCfg.validParams, PMIC_ADC_CFG_RDIV_EN_VALID))
        {
            Pmic_setBitField(
                &adcCtrlRegData, PMIC_ADC_RDIV_EN_SHIFT, PMIC_ADC_RDIV_EN_MASK, adcCfg.rDivEn);
        }
        if (pmic_validParamCheck(adcCfg.validParams, PMIC_ADC_CFG_THERMAL_SEL_VALID))
        {
            Pmic_setBitField(&adcCtrlRegData,
                             PMIC_ADC_THERMAL_SEL_SHIFT,
                             PMIC_ADC_THERMAL_SEL_MASK,
                             adcCfg.thermalSel);
        }
        if (pmic_validParamCheck(adcCfg.validParams, PMIC_ADC_CFG_CONT_CONV_VALID))
        {
            Pmic_setBitField(
                &adcCtrlRegData, PMIC_ADC_CONT_CONV_SHIFT, PMIC_ADC_CONT_CONV_MASK, adcCfg.contConv);
        }
    }

    // Write new ADC_CTRL value back to PMIC
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_commIntf_sendByte(pPmicCoreHandle, PMIC_ADC_CTRL_REGADDR, adcCtrlRegData);
    }

    // Stop critical section after read-modify-write
    Pmic_criticalSectionStop(pPmicCoreHandle);

    return status;
}

int32_t Pmic_ADCGetConfiguration(Pmic_CoreHandle_t *pPmicCoreHandle, Pmic_adcCfg_t *pAdcCfg)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t adcCtrlRegData = 0U;

    // Parameter check
    status = Pmic_ADCParamCheck_pmicHandle(pPmicCoreHandle);
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ADCParamCheck_pAdcCfg(pAdcCfg);
    }

    // Start critical section before reading
    Pmic_criticalSectionStart(pPmicCoreHandle);

    // Read ADC_CTRL register
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_ADC_CTRL_REGADDR, &adcCtrlRegData);
    }

    // Get relevant bit fields (relevancy decided by validParams)
    if (status == PMIC_ST_SUCCESS)
    {
        if (pmic_validParamCheck(pAdcCfg->validParams, PMIC_ADC_CFG_RDIV_EN_VALID))
        {
            pAdcCfg->rDivEn = Pmic_getBitField(
                adcCtrlRegData, PMIC_ADC_RDIV_EN_SHIFT, PMIC_ADC_RDIV_EN_MASK);
        }
        if (pmic_validParamCheck(pAdcCfg->validParams, PMIC_ADC_CFG_THERMAL_SEL_VALID))
        {
            pAdcCfg->thermalSel = Pmic_getBitField(
                adcCtrlRegData, PMIC_ADC_THERMAL_SEL_SHIFT, PMIC_ADC_THERMAL_SEL_MASK);
        }
        if (pmic_validParamCheck(pAdcCfg->validParams, PMIC_ADC_CFG_CONT_CONV_VALID))
        {
            pAdcCfg->contConv = Pmic_getBitField(
                adcCtrlRegData, PMIC_ADC_CONT_CONV_SHIFT, PMIC_ADC_CONT_CONV_MASK);
        }
    }

    // Stop critical section after reading
    Pmic_criticalSectionStop(pPmicCoreHandle);

    return status;
}

/**
 *  \brief      This function is used to check whether the ADC_CONT_CONV bit is set
 *              within the ADC_CTRL register.
 *
 *  \param      adcCtrlRegData      [IN]        ADC_CTRL register data
 *
 *  \return     Success code if ADC_CONT_CONV bit is zero, PMIC_ST_ERR_ADC_CONT_CONV_EN error
 *              code otherwise.
 */
static int32_t Pmic_ADCContConvCheck(const uint8_t adcCtrlRegData)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t contConv = PMIC_ADC_CONTINUOUS_CONVERSION_DISABLED;

    contConv = Pmic_getBitField(adcCtrlRegData, PMIC_ADC_CONT_CONV_SHIFT, PMIC_ADC_CONT_CONV_MASK);

    if (contConv == PMIC_ADC_CONTINUOUS_CONVERSION_ENABLED)
    {
        status = PMIC_ST_ERR_ADC_CONT_CONV_EN;
    }
    else
    {
        status = PMIC_ST_SUCCESS;
    }

    return status;
}

int32_t Pmic_ADCStartSingleConversion(Pmic_CoreHandle_t *pPmicCoreHandle)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t adcCtrlRegData = 0U;
    bool adcBusy = (bool)false;

    // Parameter check
    status = Pmic_ADCParamCheck_pmicHandle(pPmicCoreHandle);

    // Start critical section before serial comm. operations
    Pmic_criticalSectionStart(pPmicCoreHandle);

    // Read ADC_CTRL register
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_ADC_CTRL_REGADDR, &adcCtrlRegData);
    }

    // Check if continuous conversion is enabled; return error if it is enabled
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ADCContConvCheck(adcCtrlRegData);
    }

    // Check if ADC is busy; set the start bit if ADC not busy
    adcBusy = Pmic_getBitField_b(adcCtrlRegData, PMIC_ADC_STATUS_SHIFT);
    if ((status == PMIC_ST_SUCCESS) && !adcBusy)
    {
        Pmic_setBitField(&adcCtrlRegData, PMIC_ADC_START_SHIFT, PMIC_ADC_START_MASK, PMIC_ADC_START);
    }

    // Write new ADC_CTRL value back to PMIC
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_commIntf_sendByte(pPmicCoreHandle, PMIC_ADC_CTRL_REGADDR, adcCtrlRegData);
    }

    // Stop critical section after serial comm. operations
    Pmic_criticalSectionStop(pPmicCoreHandle);

    return status;
}

int32_t Pmic_ADCStartSingleConversionBlocking(Pmic_CoreHandle_t *pPmicCoreHandle)
{
    uint8_t iter = 0U;
    const uint8_t maxIter = 50U;
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t adcCtrlRegData = 0U;
    bool adcBusy = (bool)true;

    // Parameter check
    status = Pmic_ADCParamCheck_pmicHandle(pPmicCoreHandle);

    // Start critical section before reading
    Pmic_criticalSectionStart(pPmicCoreHandle);

    // Read ADC_CTRL register
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_ADC_CTRL_REGADDR, &adcCtrlRegData);
    }

    // Check if continuous conversion is enabled; return error if it is enabled
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ADCContConvCheck(adcCtrlRegData);
    }

    // Stop critical section after reading
    Pmic_criticalSectionStop(pPmicCoreHandle);

    // Wait while ADC is busy and max iterations has not been met
    while ((status == PMIC_ST_SUCCESS) && adcBusy && (iter != maxIter))
    {
        status = Pmic_ADCGetStatus(pPmicCoreHandle, &adcBusy);
        iter++;
    }

    // Check if max iterations have been met
    if ((status == PMIC_ST_SUCCESS) && (iter >= maxIter))
    {
        status = PMIC_ST_ERR_FAIL;
    }

    // Start critical section before writing
    Pmic_criticalSectionStart(pPmicCoreHandle);

    // Set the ADC start bit and write new ADC_CTRL value back to PMIC
    if (status == PMIC_ST_SUCCESS)
    {
        Pmic_setBitField(&adcCtrlRegData, PMIC_ADC_START_SHIFT, PMIC_ADC_START_MASK, PMIC_ADC_START);
        status = Pmic_commIntf_sendByte(pPmicCoreHandle, PMIC_ADC_CTRL_REGADDR, adcCtrlRegData);
    }

    // Stop critical section after writing
    Pmic_criticalSectionStop(pPmicCoreHandle);

    return status;
}

int32_t Pmic_ADCGetStatus(Pmic_CoreHandle_t *pPmicCoreHandle, bool *pAdcBusy)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t adcCtrlRegData = 0;

    // Parameter check
    status = Pmic_ADCParamCheck_pmicHandle(pPmicCoreHandle);
    if ((status == PMIC_ST_SUCCESS) && (pAdcBusy == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    // Start critical section before reading
    Pmic_criticalSectionStart(pPmicCoreHandle);

    // Read ADC_CTRL register
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_ADC_CTRL_REGADDR, &adcCtrlRegData);
    }

    // Extract ADC_STATUS bit
    if (status == PMIC_ST_SUCCESS)
    {
        *pAdcBusy = Pmic_getBitField_b(PMIC_ADC_CTRL_REGADDR, PMIC_ADC_STATUS_SHIFT);
    }

    // Stop critical section after reading
    Pmic_criticalSectionStop(pPmicCoreHandle);

    return status;
}

int32_t Pmic_ADCGetResultCode(Pmic_CoreHandle_t *pPmicCoreHandle, uint16_t *pAdcResult)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t adcResultReg1Data = 0;
    uint8_t adcResultReg2Data = 0;

    // Parameter check
    status = Pmic_ADCParamCheck_pmicHandle(pPmicCoreHandle);
    if ((status == PMIC_ST_SUCCESS) && (pAdcResult == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    // Start critical section before reading
    Pmic_criticalSectionStart(pPmicCoreHandle);

    // Read ADC_RESULT_REG_1 register (this gets bits 11-4 of the ADC result code)
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_ADC_RESULT_REG_1_REGADDR, &adcResultReg1Data);
    }

    // Read ADC_RESULT_REG_2 register (this gets bits 3-0 of the ADC result code)
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_ADC_RESULT_REG_2_REGADDR, &adcResultReg2Data);
    }

    // Combine bits 11-4 and 3-0 and store the value
    // at the location that pAdcResult is pointing to
    if (status == PMIC_ST_SUCCESS)
    {
        *pAdcResult = ((uint16_t)adcResultReg1Data << 4) | Pmic_getBitField(adcResultReg2Data,
                                                                            PMIC_ADC_RESULT_3_0_SHIFT,
                                                                            PMIC_ADC_RESULT_3_0_MASK);
    }
    
    // Stop critical section after reading
    Pmic_criticalSectionStop(pPmicCoreHandle);

    return status;
}
