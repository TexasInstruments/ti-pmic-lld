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
 *  \ingroup    DRV_PMIC_MODULE
 *  \defgroup   DRV_PMIC_ADC_MODULE PMIC ADC Driver API
 *
 *  \brief      PMIC ADC driver module that describes API function prototypes and
 *              data structures for setting/getting the configurations of the ADC,
 *              starting ADC conversions, getting ADC status, and getting ADC result.
 *
 *              Supported PMIC devices for ADC module:
 *              1. TPS6522x
 *
 *  @{
 */

/**
 *  \file pmic_adc.h
 *
 *  \brief PMIC Driver ADC API/interface file.
 */

#ifndef PMIC_ADC_H_
#define PMIC_ADC_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stdint.h>

#include "pmic_core.h"

#ifdef __cplusplus
extern "C"
{
#endif

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/**
 *  \anchor     Pmic_AdcCfgValidParam
 *  \name       Valid parameters of ADC Configuration struct
 *
 *  @{
 */
#define PMIC_ADC_CFG_RDIV_EN_VALID              (0U)
#define PMIC_ADC_CFG_THERMAL_SEL_VALID          (1U)
#define PMIC_ADC_CFG_CONT_CONV_VALID            (2U)
/** @} */

/**
 *  \anchor     Pmic_AdcCfgValidParamShift
 *  \name       Valid parameter shift values of ADC Configuration struct
 *
 *  \brief      Appliation can use these shift values to configure validParams of ADC
 *              configuration struct
 *
 *  @{
 */
#define PMIC_ADC_CFG_RDIV_EN_VALID_SHIFT        (1U << PMIC_ADC_CFG_RDIV_EN_VALID)
#define PMIC_ADC_CFG_THERMAL_SEL_VALID_SHIFT    (1U << PMIC_ADC_CFG_THERMAL_SEL_VALID)
#define PMIC_ADC_CFG_CONT_CONV_VALID_SHIFT      (1U << PMIC_ADC_CFG_CONT_CONV_VALID)
/** @} */

/* ========================================================================== */
/*                            Structures and Enums                            */
/* ========================================================================== */

/**
 *  \anchor     Pmic_adcRDivEnBitField
 *  \name       ADC_RDIV_EN Bit Field Enumeration
 *
 *  \brief      ADC resistor divider enable/disable options
 *
 *  \param  PMIC_ADC_RESISTOR_DIVIDER_DISABLED      When selected, ADC resistor divider
 *                                                  will be disabled.
 *  \param  PMIC_ADC_RESISTOR_DIVIDER_ENABLED       When selected, ADC resistor divider
 *                                                  will be enabled.
 *  @{
 */
#define PMIC_ADC_RESISTOR_DIVIDER_DISABLED      (0U)
#define PMIC_ADC_RESISTOR_DIVIDER_ENABLED       (1U)
/** @} */

/**
 *  \anchor     Pmic_adcThermalSelBitField
 *  \name       ADC_THERMAL_SEL Bit Field Enumeration
 *
 *  \brief      ADC conversion source selection
 *
 *  \param  PMIC_ADC_THERMAL_SEL_ADC_INPUT          When selected, ADC input will be
 *                                                  selected as the ADC conversion source.
 *  \param  PMIC_ADC_THERMAL_SEL_THERMAL_SENSOR     When selected, thermal sensor will
 *                                                  be selected as the ADC conversion source.
 *
 *  @{
 */
#define PMIC_ADC_THERMAL_SEL_ADC_INPUT          (0U)
#define PMIC_ADC_THERMAL_SEL_THERMAL_SENSOR     (1U)
/** @} */

/**
 *  \anchor     Pmic_adcContConvBitField
 *  \name       ADC_CONT_CONV Bit Field Enumeration
 *
 *  \brief      ADC continuous conversion enable/disable options
 *
 *  \param  PMIC_ADC_CONTINUOUS_CONVERSION_DISABLED     When selected, disables
 *                                                      ADC continuous conversion.
 *  \param  PMIC_ADC_CONTINUOUS_CONVERSION_ENABLED      When selected, enables
 *                                                      ADC continuous conversion.
 *
 *  @{
 */
#define PMIC_ADC_CONTINUOUS_CONVERSION_DISABLED (0U)
#define PMIC_ADC_CONTINUOUS_CONVERSION_ENABLED  (1U)
/** @} */

/**
 *  \anchor     Pmic_adcCfg
 *  \name       PMIC ADC Configuration Structure
 *  \brief      This struct is used to set/get PMIC ADC configuration.
 *              For possible validParams, \ref Pmic_AdcCfgValidParamShift.
 *
 *  \note       ValidParams is input param for all Set and Get APIs. Other
 *              params except validParams are input params for Set APIs and
 *              output params for Get APIs.
 *
 *  \param      rDivEn          Resistor Divider Enable. For valid values,
 *                              \ref Pmic_adcRDivEnBitField
 *  \param      thermalSel      Thermal Selection. For valid values,
 *                              \ref Pmic_adcThermalSelBitField
 *  \param      contConv        Continuous Conversion. For valid values,
 *                              \ref Pmic_adcContConvBitField
 *
 *  @{
 */
typedef struct Pmic_adcCfg_s
{
    uint8_t validParams;
    uint8_t rDivEn;
    uint8_t thermalSel;
    uint8_t contConv;
} Pmic_adcCfg_t;
/** @} */

/* ========================================================================== */
/*                           Function Declarations                            */
/* ========================================================================== */

/**
 *  \brief      This function is used to set the following PMIC ADC configurations within ADC_CTRL register:
 *              1. ADC_RDIV_EN
 *              2. ADC_THERMAL_SEL
 *              3. ADC_CONT_CONV
 *
 *  \param      pPmicCoreHandle     [IN]    PMIC interface handle
 *  \param      adcCfg              [IN]    ADC configuration
 *
 *  \return     Success code if ADC configurations are set, error code otherwise.
 *              For valid success/error codes, refer to \ref Pmic_ErrorCodes
 */
int32_t Pmic_ADCSetConfiguration(Pmic_CoreHandle_t *pPmicCoreHandle, const Pmic_adcCfg_t adcCfg);

/**
 *  \brief      This function is used to get the following PMIC ADC configurations within ADC_CTRL register:
 *              1. ADC_RDIV_EN
 *              2. ADC_THERMAL_SEL
 *              3. ADC_CONT_CONV
 *
 *  \param      pPmicCoreHandle     [IN]    PMIC interface handle
 *  \param      pAdcCfg             [OUT]   Pointer to ADC configuration struct
 *
 *  \return     Success code if ADC configurations are obtained, error code otherwise.
 *              For valid success/error codes, refer to \ref Pmic_ErrorCodes
 */
int32_t Pmic_ADCGetConfiguration(Pmic_CoreHandle_t *pPmicCoreHandle, Pmic_adcCfg_t *pAdcCfg);

/**
 *  \brief      This function is used to start a single ADC conversion. If the ADC is busy,
 *              the function does nothing and returns.
 *
 *  \param      pPmicCoreHandle     [IN]    PMIC interface handle
 *
 *  \return     Success code if ADC single conversion is started, error code otherwise.
 *              For valid success/error codes, refer to \ref Pmic_ErrorCodes
 */
int32_t Pmic_ADCStartSingleConversion(Pmic_CoreHandle_t *pPmicCoreHandle);

/**
 *  \brief      This function is used to start a single ADC conversion. If the ADC is busy,
 *              it waits until the ADC becomes idle.
 *
 *  \param      pPmicCoreHandle     [IN]    PMIC interface handle
 *
 *  \return     Success code if ADC single conversion is started, error code otherwise.
 *              For valid success/error codes, refer to \ref Pmic_ErrorCodes
 */
int32_t Pmic_ADCStartSingleConversionBlocking(Pmic_CoreHandle_t *pPmicCoreHandle);

/**
 *  \brief      This function is used to get the status of the ADC (whether it is busy or idle).
 *
 *  \param      pPmicCoreHandle     [IN]    PMIC interface handle
 *  \param      pAdcBusy            [OUT]   Pointer to ADC Busy boolean variable
 *
 *  \return     Success code if ADC status is obtained, error code otherwise.
 *              For valid success/error codes, refer to \ref Pmic_ErrorCodes
 */
int32_t Pmic_ADCGetStatus(Pmic_CoreHandle_t *pPmicCoreHandle, bool *pAdcBusy);

/**
 *  \brief      This function is used to get the ADC result code that is outputted after an ADC conversion.
 *
 *  \param      pPmicCoreHandle     [IN]    PMIC interface handle
 *  \param      pAdcResult          [OUT]   Pointer to ADC result variable
 *
 *  \return     Success code if ADC result code is obtained, error code otherwise.
 *              For valid success/error codes, refer to \ref Pmic_ErrorCodes
 */
int32_t Pmic_ADCGetResultCode(Pmic_CoreHandle_t *pPmicCoreHandle, uint16_t *pAdcResult);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* PMIC_ADC_H_ */

/** @} */
