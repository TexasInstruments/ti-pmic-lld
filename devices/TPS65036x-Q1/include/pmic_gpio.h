/******************************************************************************
 * Copyright (c) 2024 - 2025 Texas Instruments Incorporated - http://www.ti.com
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
 * @file pmic_gpio.h
 *
 * @brief PMIC LLD GPIO module header file.
 *
 * @details This module contains declarations/definitions of macros, data
 * structures, and APIs used to interact with and control PMIC GPIOs. Some
 * components of the PMIC GPIO module are as follows: set/get GPIO
 * configurations and activation/deactivation of GPIOs.
 */
#ifndef __PMIC_GPIO_H__
#define __PMIC_GPIO_H__

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                Include Files                               */
/* ========================================================================== */
#include "pmic_common.h"

/* ========================================================================== */
/*                              Macros & Typedefs                             */
/* ========================================================================== */

/**
 * @anchor Pmic_GpioCfgValidParams
 * @name TPS65036x GPIO Configuration Valid Parameters
 *
 * @brief Indication of which parameters are valid within the Pmic_GpioCfg_t
 * struct.
 *
 * @details For more information on the parameters, refer to @ref Pmic_GpioCfg.
 *
 * @note End-user could combine multiple valid parameters using the OR operator.
 *
 * @{
 */
#define PMIC_FUNCTIONALITY_VALID    ((uint32_t)(1U << 0U))
#define PMIC_POLARITY_VALID         ((uint32_t)(1U << 1U))
#define PMIC_PU_PD_CFG_VALID        ((uint32_t)(1U << 2U))
#define PMIC_OD_PP_CFG_VALID        ((uint32_t)(1U << 3U))
/** @} */

/**
 * @anchor Pmic_GpioPin
 * @name PMIC GPIO Pin
 *
 * @brief TPS65036x GPIO pin enumeration. The PMIC has two GPIO pins nammed GPIO
 * and nINT_GPI.
 *
 * @{
 */
#define PMIC_NINT_GPI       ((uint8_t)1U)
#define PMIC_GPIO           ((uint8_t)2U)
#define PMIC_GPIO_PIN_MAX   (PMIC_GPIO)
/** @} */

/**
 * @anchor Pmic_GpioFunctionality
 * @name PMIC GPIO Functionality
 *
 * @brief Different functionalities of the GPIO and NINT_GPI pins.
 *
 * @{
 */
#define PMIC_GPIO_INPUT                     ((uint8_t)0U)
#define PMIC_GPIO_WDG_TRIG_MODE_INPUT       ((uint8_t)1U)
#define PMIC_GPIO_ESM_INPUT                 ((uint8_t)2U)
#define PMIC_GPIO_OUTPUT                    ((uint8_t)3U)
#define PMIC_GPIO_FUNCTIONALITY_MAX         (PMIC_GPIO_OUTPUT)
#define PMIC_NINT_GPI_NINT                  ((uint8_t)0U)
#define PMIC_NINT_GPI_WDG_TRIG_MODE_INPUT   ((uint8_t)1U)
#define PMIC_NINT_GPI_LPM_CTRL_MODE_INPUT   ((uint8_t)2U)
#define PMIC_NINT_GPI_FUNCTIONALITY_MAX     (PMIC_NINT_GPI_LPM_CTRL_MODE_INPUT)
/** @} */

/**
 * @anchor Pmic_GpioPuPdCfg
 * @name PMIC GPIO Pullup/Pulldown Resistor Configuration.
 *
 * @brief Valid pullup/pulldown resistor configurations.
 *
 * @{
 */
#define PMIC_PU_RESISTOR_ACTIVATED  ((uint8_t)0U)
#define PMIC_PD_RESISTOR_ACTIVATED  ((uint8_t)1U)
#define PMIC_PU_PD_CFG_MAX          (PMIC_PD_RESISTOR_ACTIVATED)
/** @} */

/**
 * @anchor Pmic_GpioPolarity
 * @name PMIC GPIO Polarity
 *
 * @brief Valid GPIO polarities.
 *
 * @{
 */
#define PMIC_NORMAL_POLARITY    ((uint8_t)0U)
#define PMIC_INVERTED_POLARITY  ((uint8_t)1U)
#define PMIC_POLARITY_MAX       (PMIC_INVERTED_POLARITY)
/** @} */

/**
 * @anchor Pmic_GpioOdPpCfg
 * @name PMIC GPIO Open-drain/Push-pull Configurations.
 *
 * @brief Valid GPIO Open-drain/Push-pull configurations.
 *
 * @{
 */
#define PMIC_PUSH_PULL      ((uint8_t)0U)
#define PMIC_OPEN_DRAIN     ((uint8_t)1U)
#define PMIC_OD_PP_CFG_MAX  (PMIC_OPEN_DRAIN)
/** @} */

/* ========================================================================== */
/*                             Structures and Enums                           */
/* ========================================================================== */

/**
 * @anchor Pmic_GpioCfg
 * @name PMIC GPIO Configuration Struct
 *
 * @brief Struct used to set and get PMIC GPIO configurations.
 *
 * @note The TPS65036x PMIC has two GPIO pins called nINT_GPI and GPIO.
 *
 * @param validParams Each bit in this variable represents whether a struct member
 * is valid. For valid values, refer to @ref Pmic_GpioCfgValidParams.
 *
 * @param functionality GPIO functionality. For valid values, refer to
 * @ref Pmic_GpioFunctionality.
 *
 * @param polarity GPIO polarity configuration. For valid values, see
 * @ref Pmic_GpioPolarity.
 *
 * @param puPdCfg Pullup/pulldown resistor configuration. For valid values, see
 * @ref Pmic_GpioPuPdCfg.
 *
 * @param odPpCfg Open-drain/push-pull operation configuration. For valid values,
 * see @ref Pmic_GpioOdPpCfg.
 */
typedef struct Pmic_GpioCfg_s {
    uint32_t validParams;

    /* Both nINT_GPI and GPIO */
    uint8_t functionality;
    uint8_t polarity;

    /* nINT_GPI only */
    uint8_t puPdCfg;
    uint8_t odPpCfg;
} Pmic_GpioCfg_t;

/* ========================================================================== */
/*                             Function Declarations                          */
/* ========================================================================== */

/**
 * @brief Set PMIC GPIO configurations.
 *
 * @details The options that are configurable using this API are listed below.
 * 1. Functionality (validParam: PMIC_FUNCTIONALITY_VALID)
 * 2. Polarity (validParam: PMIC_POLARITY_VALID)
 * 3. Pulldown resistor (validParam: PMIC_PU_PD_CFG_VALID)
 * 4. Open-drain/push-pull operation (validParam: PMIC_OD_PP_CFG_VALID)
 *
 * @note NINT_GPI has two configurations that GPIO does not: pullup/pulldown
 * resistor configuration and push-pull/open-drain configuration.
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @param gpioPin [IN] PMIC GPIO pin identifier. For valid values, see
 * @ref Pmic_gpioPin.
 *
 * @param gpioCfg [IN] PMIC GPIO configurations to be set.
 *
 * @return Success code if PMIC GPIO configurations have been set, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_gpioSetCfg(const Pmic_CoreHandle_t *pmicHandle, uint8_t gpioPin, const Pmic_GpioCfg_t *gpioCfg);

/**
 * @brief Get PMIC GPIO configurations. This API supports getting the same
 * configurations that are settable through Pmic_gpioSetCfg().
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @param gpioPin [IN] PMIC GPIO pin identifier. For valid values, see
 * @ref Pmic_gpioPin.
 *
 * @param gpioCfg [OUT] GPIO configurations obtained from PMIC.
 *
 * @return Success code if PMIC GPIO configurations have been obtained, error
 * code otherwise. For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_gpioGetCfg(const Pmic_CoreHandle_t *pmicHandle, uint8_t gpioPin, Pmic_GpioCfg_t *gpioCfg);

/**
 * @brief Activate or deactivate PMIC GPIO. This API is a superset of
 * Pmic_gpioActivate() and Pmic_gpioDeactivate().
 *
 * @note The TPS65036x PMIC has two GPIO pins called nINT_GPI and GPIO. This API
 * is not valid for nINT_GPI.
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @param activate [IN] When set to true, GPIO is activated and its output state
 * is high (depending on configured polarity). When set to false, GPIO is
 * deactivated and its output state is low.
 *
 * @return Success code if GPIO is activated/deactivated, error code otherwise.
 * For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_gpioSetActivationState(const Pmic_CoreHandle_t *pmicHandle, bool activate);

/**
 * @brief Activate PMIC GPIO. This API is a subset of Pmic_gpioSetActiveState().
 *
 * @details When GPIO is activated, its output functionality is enabled and its
 * output state is high (depending on configured polarity).
 *
 * @note The TPS65036x PMIC has two GPIO pins called nINT_GPI and GPIO. This API
 * is not valid for nINT_GPI.
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @return Success code if GPIO is activated, error code otherwise. For valid
 * success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_gpioActivate(const Pmic_CoreHandle_t *pmicHandle);

/**
 * @brief Deactivate PMIC GPIO. This API is a subset of Pmic_gpioSetActiveState().
 *
 * @details When GPIO is deactivated, its output functionality is disabled and its
 * output state is low.
 *
 * @note The TPS65036x PMIC has two GPIO pins called nINT_GPI and GPIO. This API
 * is not valid for nINT_GPI.
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @return Success code if GPIO is deactivated, error code otherwise. For valid
 * success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_gpioDeactivate(const Pmic_CoreHandle_t *pmicHandle);

/**
 * @brief Get PMIC GPIO activation state.
 *
 * @note The TPS65036x PMIC has two GPIO pins called nINT_GPI and GPIO. This API
 * is not valid for nINT_GPI.
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @param activated [OUT] Activation state. When set to true, GPIO is activated.
 * When set to false, GPIO is deactivated.
 *
 * @return Success code if GPIO activation status has been obtained, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_gpioGetActivationState(const Pmic_CoreHandle_t *pmicHandle, bool *activated);

#ifdef __cplusplus
}

#endif /* __cplusplus */
#endif /* __PMIC_GPIO_H__ */
