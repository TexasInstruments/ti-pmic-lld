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
#ifndef PMIC_GPIO_H
#define PMIC_GPIO_H

/**
 * @file pmic_gpio.h
 *
 * @brief PMIC general purpose input/output (GPIO) interface. Contains APIs,
 * macros/defines, and data structures used to configure, control, and interact
 * with PMIC GPIOs.
 */

/* ========================================================================= */
/*                              Include Files                                */
/* ========================================================================= */

#include "pmic_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================= */
/*                             Macros & Typedefs                             */
/* ========================================================================= */

/**
 * @anchor Pmic_GpioPin
 * @name PMIC GPIO Pin
 *
 * @brief PMIC GPIO pin enumerations.
 *
 * @{
 */
#define PMIC_GPIO_GPO1 (1U)
#define PMIC_GPIO_GPO2 (2U)
#define PMIC_GPIO_PIN_MIN  (PMIC_GPIO_GPO1)
#define PMIC_GPIO_PIN_MAX  (PMIC_GPIO_GPO2)
/** @} */

/**
 * @anchor Pmic_GpioGpo1FxnSel
 * @name PMIC GPIO GPO1 Function Select
 *
 * @brief Function select options for PMIC GPIO GPO1.
 *
 * @{
 */
#define PMIC_GPIO_FXN_SEL_VMON1   (0U)
#define PMIC_GPIO_FXN_SEL_GPO1    (1U)
#define PMIC_GPIO_FXN_SEL_FAULT1  (2U)
#define PMIC_GPIO_FXN_SEL_CAN_DIS (3U)
#define PMIC_GPIO_FXN_GPO_1_MIN   (PMIC_GPIO_FXN_SEL_VMON1)
#define PMIC_GPIO_FXN_GPO_1_MAX   (PMIC_GPIO_FXN_SEL_CAN_DIS)
/** @} */

/**
 * @anchor Pmic_GpioGpo2FxnSel
 * @name PMIC GPIO GPO2 Function Select
 *
 * @brief Function select options for PMIC GPIO GPO2.
 *
 * @{
 */
#define PMIC_GPIO_FXN_SEL_NERR   (0U)
#define PMIC_GPIO_FXN_SEL_GPO2   (1U)
#define PMIC_GPIO_FXN_SEL_FAULT2 (2U)
#define PMIC_GPIO_FXN_GPO_2_MIN  (PMIC_GPIO_FXN_SEL_NERR)
#define PMIC_GPIO_FXN_GPO_2_MAX  (PMIC_GPIO_FXN_SEL_FAULT2)
/** @} */

/**
 * @anchor Pmic_GpioPolarity
 * @name PMIC GPIO Polarity
 *
 * @brief Polarity options for PMIC GPIO pins (including nERR, nRSTOUT, and nINT).
 *
 * @{
 */
#define PMIC_GPIO_POLARITY_ACTIVE_LOW  (0U)
#define PMIC_GPIO_POLARITY_ACTIVE_HIGH (1U)
#define PMIC_GPIO_POLARITY_MIN         (PMIC_GPIO_POLARITY_ACTIVE_LOW)
#define PMIC_GPIO_POLARITY_MAX         (PMIC_GPIO_POLARITY_ACTIVE_HIGH)
/** @} */

/**
 * @anchor Pmic_GpioType
 * @name PMIC GPIO Type
 *
 * @brief Type options for PMIC GPIO pins (nRSTOUT, and nINT).
 *
 * @{
 */
#define PMIC_GPIO_TYPE_PUSH_PULL  (0U)
#define PMIC_GPIO_TYPE_OPEN_DRAIN (1U)
#define PMIC_GPIO_TYPE_MIN        (PMIC_GPIO_TYPE_PUSH_PULL)
#define PMIC_GPIO_TYPE_MAX        (PMIC_GPIO_TYPE_OPEN_DRAIN)
/** @} */

/**
 * @anchor Pmic_GpioDelay
 * @name PMIC GPIO Delay
 *
 * @brief Valid startup and shutdown delay options for PMIC GPIO pins (including
 * nERR, nRSTOUT, and nINT).
 *
 * @{
 */
#define PMIC_GPIO_DELAY_0P0_MS (0x0U)
#define PMIC_GPIO_DELAY_0P5_MS (0x1U)
#define PMIC_GPIO_DELAY_1P0_MS (0x2U)
#define PMIC_GPIO_DELAY_1P5_MS (0x3U)
#define PMIC_GPIO_DELAY_2P0_MS (0x4U)
#define PMIC_GPIO_DELAY_2P5_MS (0x5U)
#define PMIC_GPIO_DELAY_3P0_MS (0x6U)
#define PMIC_GPIO_DELAY_3P5_MS (0x7U)
#define PMIC_GPIO_DELAY_4P0_MS (0x8U)
#define PMIC_GPIO_DELAY_4P5_MS (0x9U)
#define PMIC_GPIO_DELAY_5P0_MS (0xAU)
#define PMIC_GPIO_DELAY_5P5_MS (0xBU)
#define PMIC_GPIO_DELAY_6P0_MS (0xCU)
#define PMIC_GPIO_DELAY_6P5_MS (0xDU)
#define PMIC_GPIO_DELAY_7P0_MS (0xEU)
#define PMIC_GPIO_DELAY_7P5_MS (0xFU)
#define PMIC_GPIO_DELAY_MIN    (PMIC_GPIO_DELAY_0P0_MS)
#define PMIC_GPIO_DELAY_MAX    (PMIC_GPIO_DELAY_7P5_MS)
/** @} */

/**
 * @anchor Pmic_GpioPinCfgValidParams
 * @name PMIC GPIO Pin Configuration Structure
 *
 * @brief Definitions used to indicate valid parameters of `Pmic_GpioPinCfg_t`.
 * Set the `validParams` member of `Pmic_GpioPinCfg_t` equal to a combination of
 * these defines by using the `OR` operator.
 *
 * @{
 */
#define PMIC_GPIO_PIN_FXN_SEL_VALID        (1U << 0U)
#define PMIC_GPIO_PIN_FAULT_TYPE_VALID     (1U << 1U)
#define PMIC_GPIO_PIN_FAULT_POLARITY_VALID (1U << 2U)
#define PMIC_GPIO_PIN_STARTUP_DELAY_VALID  (1U << 3U)
#define PMIC_GPIO_PIN_SHUTDOWN_DELAY_VALID (1U << 4U)
#define PMIC_GPIO_PIN_VALID_ALL            (\
    PMIC_GPIO_PIN_FXN_SEL_VALID | \
    PMIC_GPIO_PIN_FAULT_TYPE_VALID | \
    PMIC_GPIO_PIN_FAULT_POLARITY_VALID | \
    PMIC_GPIO_PIN_STARTUP_DELAY_VALID | \
    PMIC_GPIO_PIN_SHUTDOWN_DELAY_VALID)
/** @} */

/**
 * @anchor Pmic_GpioNIntCfgValidParams
 * @name PMIC GPIO nINT Configuration Structure
 *
 * @brief Definitions used to indicate valid parameters of `Pmic_GpioNIntCfg_t`.
 * Set the `validParams` member of `Pmic_GpioNIntCfg_t` equal to a combination of
 * these defines by using the `OR` operator.
 *
 * @{
 */
#define PMIC_GPIO_NINT_PU_EN_VALID    (1U << 0U)
#define PMIC_GPIO_NINT_TYPE_VALID     (1U << 1U)
#define PMIC_GPIO_NINT_POLARITY_VALID (1U << 2U)
#define PMIC_GPIO_NINT_VALID_ALL      (\
    PMIC_GPIO_NINT_PU_EN_VALID | \
    PMIC_GPIO_NINT_TYPE_VALID | \
    PMIC_GPIO_NINT_POLARITY_VALID)
/** @} */

/**
 * @anchor Pmic_GpioNRstOutCfgValidParams
 * @name PMIC GPIO nRSTOUT Configuration Structure
 *
 * @brief Definitions used to indicate valid parameters of `Pmic_GpioNRstOutCfg_t`.
 * Set the `validParams` member of `Pmic_GpioNRstOutCfg_t` equal to a combination of
 * these defines by using the `OR` operator.
 *
 * @{
 */
#define PMIC_GPIO_NRSTOUT_PU_EN_VALID          (1U << 0U)
#define PMIC_GPIO_NRSTOUT_TYPE_VALID           (1U << 1U)
#define PMIC_GPIO_NRSTOUT_POLARITY_VALID       (1U << 2U)
#define PMIC_GPIO_NRSTOUT_STARTUP_DELAY_VALID  (1U << 3U)
#define PMIC_GPIO_NRSTOUT_SHUTDOWN_DELAY_VALID (1U << 4U)
#define PMIC_GPIO_NRSTOUT_VALID_ALL            (\
    PMIC_GPIO_NRSTOUT_PU_EN_VALID | \
    PMIC_GPIO_NRSTOUT_TYPE_VALID | \
    PMIC_GPIO_NRSTOUT_POLARITY_VALID | \
    PMIC_GPIO_NRSTOUT_STARTUP_DELAY_VALID | \
    PMIC_GPIO_NRSTOUT_SHUTDOWN_DELAY_VALID)
/** @} */

/* ========================================================================== */
/*                            Structures and Enums                            */
/* ========================================================================== */

/**
 * @anchor Pmic_GpioPinCfg
 * @name PMIC GPIO Pin Configuration Structure
 *
 * @brief Structure used to set/get PMIC GPIO pin configurations.
 *
 * @note This structure is not used to set/get configurations for nERR, nRSTOUT,
 * or nINT. this structure is specifically for GPO1 and GPO2.
 *
 * @param validParams Each bit in this variable corresponds to a member in this
 * structure. Specifically, if a bit is set to 1 in this variable, the corresponding
 * structure member is valid and will be considered by the driver API that is using
 * this data structure. Otherwise, if a bit is set to 0, the corresponding structure
 * member is invalid and will not be considered by the driver API that is using
 * this data structure. For possible valid parameter values, refer to
 * @ref Pmic_GpioPinCfgValidParams.
 *
 * @param pin Must be specified. Driver will set/get configurations of the specified
 * pin. For valid values, refer to @ref Pmic_GpioPin.
 *
 * @param fxnSel GPIO pin function select. For valid values, refer to
 * @ref Pmic_GpioGpo1FxnSel and @ref Pmic_GpioGpo2FxnSel.
 *
 * @param faultType Type when GPO{1,2} is configured to FAULT{1,2} functionality.
 * For valid values, refer to @ref Pmic_GpioType.
 *
 * @param faultPolarity Polarity when GPO{1,2} is configured to FAULT{1,2}
 * functionality. For valid values, refer to @ref Pmic_GpioPolarity.
 *
 * @param startupDelay Startup delay for the pin. For valid values, refer to
 * @ref Pmic_GpioDelay.
 *
 * @param shutdownDelay Shutdown delay for the pin. For valid values, refer to
 * @ref Pmic_GpioDelay.
 *
 */
typedef struct Pmic_GpioPinCfg_s {
    uint32_t validParams;
    uint16_t pin;

    uint8_t fxnSel;

    uint8_t faultType;
    uint8_t faultPolarity;

    uint8_t startupDelay;
    uint8_t shutdownDelay;
} Pmic_GpioPinCfg_t;

/**
 * @anchor Pmic_GpioNErrCfg
 * @name PMIC GPIO nERR Configuration Structure
 *
 * @brief Structure used to set/get PMIC nERR pin configurations.
 *
 * @param puEn Enable or disable pull-up resistor.
 *
 * @{
 */
typedef struct Pmic_GpioNErrCfg_s {
    bool puEn;
} Pmic_GpioNErrCfg_t;
/** @} */

/**
 * @anchor Pmic_GpioNIntCfg
 * @name PMIC GPIO nINT Configuration Structure
 *
 * @brief Structure used to set/get PMIC nINT pin configurations.
 *
 * @param validParams Each bit in this variable corresponds to a member in this
 * structure. Specifically, if a bit is set to 1 in this variable, the corresponding
 * structure member is valid and will be considered by the driver API that is using
 * this data structure. Otherwise, if a bit is set to 0, the corresponding structure
 * member is invalid and will not be considered by the driver API that is using
 * this data structure. For possible valid parameter values, refer to
 * @ref Pmic_GpioNIntCfgValidParams.
 *
 * @param puEn Enable or disable pull-up resistor.
 *
 * @param type Type of the pin. For valid values, refer to @ref Pmic_GpioType.
 *
 * @param polarity Polarity of the pin. For valid values, refer to @ref Pmic_GpioPolarity.
 *
 * @{
 */
typedef struct Pmic_GpioNIntCfg_s {
    uint32_t validParams;

    bool puEn;
    uint8_t type;
    uint8_t polarity;
} Pmic_GpioNIntCfg_t;
/** @} */

/**
 * @anchor Pmic_GpioNRstOutCfg
 * @name PMIC GPIO nRSTOUT Configuration Structure
 *
 * @brief Structure used to set/get PMIC nRSTOUT pin configurations.
 *
 * @param validParams Each bit in this variable corresponds to a member in this
 * structure. Specifically, if a bit is set to 1 in this variable, the corresponding
 * structure member is valid and will be considered by the driver API that is using
 * this data structure. Otherwise, if a bit is set to 0, the corresponding structure
 * member is invalid and will not be considered by the driver API that is using
 * this data structure. For possible valid parameter values, refer to
 * @ref Pmic_GpioNRstOutCfgValidParams.
 *
 * @param puEn Enable or disable pull-up resistor.
 *
 * @param type Type of the pin. For valid values, refer to @ref Pmic_GpioType.
 *
 * @param polarity Polarity of the pin. For valid values, refer to
 * @ref Pmic_GpioPolarity.
 *
 * @param startupDelay Startup delay for the pin. For valid values, refer to
 * @ref Pmic_GpioDelay.
 *
 * @param shutdownDelay Shutdown delay for the pin. For valid values, refer to
 * @ref Pmic_GpioDelay.
 *
 * @{
 */
typedef struct Pmic_GpioNRstOutCfg_s {
    uint32_t validParams;

    bool puEn;
    uint8_t type;
    uint8_t polarity;
    uint8_t startupDelay;
    uint8_t shutdownDelay;
} Pmic_GpioNRstOutCfg_t;
/** @} */

/* ========================================================================== */
/*                            Function Declarations                           */
/* ========================================================================== */

/**
 * @brief Enable or disable a GPIO pin (GPO1 or GPO2).
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param gpioPin [IN] GPIO pin number. Choose between GPO1 (PMIC_GPIO_GPO1) or
 * GPO2 (PMIC_GPIO_GPO2).
 *
 * @param enable [IN] `PMIC_ENABLE` - enable the GPIO pin,
 * `PMIC_DISABLE` - disable the GPIO pin.
 *
 * @return PMIC_ST_SUCCESS if GPIO pin has been successfully enabled or disabled,
 * error code otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_gpioSetPinEnableState(const Pmic_Handle_t *handle, uint8_t gpioPin, bool enable);

/**
 * @brief Get the status of whether a GPIO pin (GPO1 or GPO2) is enabled or disabled.
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param gpioPin [IN] GPIO pin number. Choose between GPO1 (PMIC_GPIO_GPO1) or
 * GPO2 (PMIC_GPIO_GPO2).
 *
 * @param isEnabled [OUT] `PMIC_ENABLE` - GPIO pin is enabled,
 * `PMIC_DISABLE` - GPIO pin is disabled.
 *
 * @return PMIC_ST_SUCCESS if GPIO pin enable state has been successfully obtained
 * from the PMIC, error code otherwise. For valid success/error codes, refer to
 * @ref Pmic_ErrorCodes.
 */
int32_t Pmic_gpioGetPinEnableState(const Pmic_Handle_t *handle, uint8_t gpioPin, bool *isEnabled);

/**
 * @brief Set GPIO pin (GPO1 or GPO2) configurations.
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param gpioPinCfg [IN] Desired GPIO pin configurations to set. For more
 * information on GPIO pin configurations, refer to @ref Pmic_GpioPinCfg.
 *
 * @return PMIC_ST_SUCCESS if GPIO pin configuration has been successfully set,
 * error code otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_gpioSetPinCfg(const Pmic_Handle_t *handle, const Pmic_GpioPinCfg_t *gpioPinCfg);

/**
 * @brief Get GPIO pin (GPO1 or GPO2) configurations.
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param gpioPinCfg [OUT] GPIO pin configurations obtained form the PMIC. For
 * more information on GPIO pin configurations, refer to @ref Pmic_GpioPinCfg.
 *
 * @return PMIC_ST_SUCCESS if GPIO pin configuration has been successfully obtained,
 * error code otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_gpioGetPinCfg(const Pmic_Handle_t *handle, Pmic_GpioPinCfg_t *gpioPinCfg);

/**
 * @brief Set nErr pin configurations.
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param nErrCfg [IN] Desired nErr pin configurations to set. For more information
 * on nErr pin configurations, refer to @ref Pmic_GpioNErrCfg.
 *
 * @return PMIC_ST_SUCCESS if nErr pin configuration has been successfully set,
 * error code otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_gpioSetNErrCfg(const Pmic_Handle_t *handle, const Pmic_GpioNErrCfg_t *nErrCfg);

/**
 * @brief Get nErr pin configurations.
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param nErrCfg [OUT] nErr pin configurations obtained from the PMIC. For more
 * information on nErr pin configurations, refer to @ref Pmic_GpioNErrCfg.
 *
 * @return PMIC_ST_SUCCESS if nErr pin configuration has been successfully obtained,
 * error code otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_gpioGetNErrCfg(const Pmic_Handle_t *handle, Pmic_GpioNErrCfg_t *nErrCfg);

/**
 * @brief Set nINT pin configurations.
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param nIntCfg [IN] Desired nINT pin configurations to set. For more information
 * on nINT pin configurations, refer to @ref Pmic_GpioNIntCfg.
 *
 * @return PMIC_ST_SUCCESS if nINT pin configuration has been successfully set,
 * error code otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_gpioSetNIntCfg(const Pmic_Handle_t *handle, const Pmic_GpioNIntCfg_t *nIntCfg);

/**
 * @brief Get nINT pin configurations.
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param nIntCfg [OUT] nINT pin configurations obtained from the PMIC. For more
 * information on nINT pin configurations, refer to @ref Pmic_GpioNIntCfg.
 *
 * @return PMIC_ST_SUCCESS if nINT pin configuration has been successfully obtained,
 * error code otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_gpioGetNIntCfg(const Pmic_Handle_t *handle, Pmic_GpioNIntCfg_t *nIntCfg);

/**
 * @brief Set nRSTOUT pin configurations.
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param nRstOutCfg [IN] Desired nRSTOUT pin configurations to set. For more information
 * on nRSTOUT pin configurations, refer to @ref Pmic_GpioNRstOutCfg.
 *
 * @return PMIC_ST_SUCCESS if nRSTOUT pin configuration has been successfully set,
 * error code otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_gpioSetNRstOutCfg(const Pmic_Handle_t *handle, const Pmic_GpioNRstOutCfg_t *nRstOutCfg);

/**
 * @brief Get nRSTOUT pin configurations.
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param nRstOutCfg [OUT] nRSTOUT pin configurations obtained from the PMIC. For more
 * information on nRSTOUT pin configurations, refer to @ref Pmic_GpioNRstOutCfg.
 *
 * @return PMIC_ST_SUCCESS if nRSTOUT pin configuration has been successfully obtained,
 * error code otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_gpioGetNRstOutCfg(const Pmic_Handle_t *handle, Pmic_GpioNRstOutCfg_t *nRstOutCfg);

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* PMIC_GPIO_H */
