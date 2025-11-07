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
#ifndef PMIC_ADC_H
#define PMIC_ADC_H

/**
 * @file pmic_adc.h
 *
 * @brief PMIC analog-to-digital converter (ADC) interface. Contains APIs,
 * macros/defines, and data structures used to configure, control, and interact
 * with the PMIC ADC.
 */

/* ========================================================================= */
/*                              Include Files                                */
/* ========================================================================= */

#include "pmic_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                            Macros & Typedefs                               */
/* ========================================================================== */

/**
 * @anchor Pmic_AdcSrcSel
 * @name PMIC ADC Source Select
 *
 * @brief Possible sources of the PMIC ADC.
 *
 * @{
 */
#define PMIC_ADC_SRC_SEL_INPUT          (0U)
#define PMIC_ADC_SRC_SEL_THERMAL_SENSOR (1U)
#define PMIC_ADC_SRC_SEL_MIN            (PMIC_ADC_INPUT)
#define PMIC_ADC_SRC_SEL_MAX            (PMIC_THERMAL_SENSOR)
/** @} */

/**
 * @anchor Pmic_AdcCfgValidParams
 * @name PMIC ADC Configuration Structure Valid Parameters
 *
 * @brief Definitions used to indicate valid parameters of `Pmic_AdcCfg_t`.
 * Set the `validParams` member of `Pmic_AdcCfg_t` equal to a combination
 * of these defines by using the `OR` operator.
 *
 * @{
 */
#define PMIC_ADC_RDIV_EN_VALID      (1U << 0U)
#define PMIC_ADC_CONT_CONV_EN_VALID (2U << 1U)
#define PMIC_ADC_SRC_SEL_VALID      (3U << 2U)
/** @} */

/* ========================================================================== */
/*                           Structures and Enums                             */
/* ========================================================================== */

/**
 * @anchor Pmic_AdcCfg
 * @name PMIC ADC Configuration Structure
 *
 * @brief Structure used to set and get ADC configurations.
 *
 * @param validParams Each bit in this variable corresponds to a member in this
 * structure. Specifically, if a bit is set to 1 in this variable, the corresponding
 * structure member is valid and will be considered by the driver API that is using
 * this data structure. Otherwise, if a bit is set to 0, the corresponding structure
 * member is invalid and will not be considered by the driver API that is using
 * this data structure. For possible valid parameter values, refer to
 * @ref Pmic_AdcCfgValidParams.
 *
 * @param rDivEn Resistor divider enable. If set to true, the input range of the
 * ADC is [0V, 6V]. If set to false, the input range of the ADC is [0V, 1V].
 *
 * @param contConvEn Continous ADC conversion enable. If set to true, the ADC
 * continuously converts analog signals to digital (rather than stopping after
 * a single conversion). If set to false, the ADC must be manually started and
 * stops after a single conversion.
 *
 * @param srcSel Select the ADC input source. For valid values, refer to
 * @ref Pmic_AdcSrcSel.
 *
 * @{
 */
typedef struct Pmic_AdcCfg_s {
    uint32_t validParams;

    bool rDivEn;
    bool contConvEn;

    uint8_t srcSel;
} Pmic_AdcCfg_t;
/** @} */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 * @brief Set PMIC ADC configurations.
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param adcCfg [IN] Desired ADC configurations to set. For more information on
 * ADC configurations, refer to @ref Pmic_AdcCfg_t.
 *
 * @return PMIC_ST_SUCCESS if PMIC ADC configurations have been set, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_adcSetCfg(const Pmic_Handle_t *handle, const Pmic_AdcCfg_t *adcCfg);

/**
 * @brief Get PMIC ADC configurations.
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param adcCfg [OUT] ADC configurations obtained from the PMIC. For more
 * information on ADC configurations, refer to @ref Pmic_AdcCfg_t.
 *
 * @return PMIC_ST_SUCCESS if PMIC ADC configurations have been obtained, error
 * code otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_adcGetCfg(const Pmic_Handle_t *handle, Pmic_AdcCfg_t *adcCfg);

/**
 * @brief Start a single PMIC ADC conversion. If the ADC is busy, this API
 * does nothing and returns.
 *
 * @attention This API should only be called if ADC continuous conversion is
 * disabled.
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @return PMIC_ST_SUCCESS if a single PMIC ADC conversion has been started,
 * error code otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_adcStartSingleConversion(const Pmic_Handle_t *handle);

/**
 * @brief Start a single PMIC ADC conversion. If the ADC is busy, this API
 * waits for the ADC to become idle.
 *
 * @attention This API should only be called if ADC continuous conversion is
 * disabled.
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @return PMIC_ST_SUCCESS if a single PMIC ADC conversion has been started,
 * error code otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_adcStartSingleConversionBlocking(const Pmic_Handle_t *handle);

/**
 * @brief Get PMIC ADC status; specifically, whether the ADC is busy or idle.
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param adcBusy [OUT] If returned as true, the ADC is busy. Otherwise, the ADC
 * is idle.
 *
 * @return PMIC_ST_SUCCESS if PMIC ADC status has been obtained, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_adcGetStatus(const Pmic_Handle_t *handle, bool *adcBusy);

/**
 * @brief Get the result code that is outputted after an ADC conversion. Refer to
 * the PMIC device TRM for how to convert the result code into a human-readable
 * value (i.e., voltage or temperature).
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param adcResult [OUT] PMIC ADC result code.
 *
 * @return PMIC_ST_SUCCESS if the result code has been obtained, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_adcGetResultCode(const Pmic_Handle_t *handle, uint16_t *adcResult);

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* PMIC_ADC_H */
