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
 * with PMIC GPIO pins.
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
 * @anchor Pmic_GpioPinNum
 * @name PMIC GPIO Pin Number.
 *
 * @brief GPIO identification.
 *
 * @{
 */
#define PMIC_GPIO_PIN1    (1U)
#define PMIC_GPIO_PIN2    (2U)
#define PMIC_GPIO_PIN3    (3U)
#define PMIC_GPIO_PIN4    (4U)
#define PMIC_GPIO_PIN5    (5U)
#define PMIC_GPIO_PIN6    (6U)
#define PMIC_GPIO_PIN_MIN (PMIC_GPIO_PIN1)
#define PMIC_GPIO_PIN_MAX (PMIC_GPIO_PIN6)
/** @} */

/**
 * @anchor Pmic_GpioPin1FxnSel
 * @name PMIC GPIO Pin 1 Function Select
 *
 * @brief GPIO pin 1 functionalities.
 *
 * @{
 */
#define PMIC_GPIO_PIN1_FXN_SEL_GPIO    (0U)
#define PMIC_GPIO_PIN1_FXN_SEL_SDO_SPI (1U)
#define PMIC_GPIO_PIN1_FXN_SEL_NSLEEP2 (2U)
#define PMIC_GPIO_PIN1_FXN_SEL_NINT    (3U)
#define PMIC_GPIO_PIN1_FXN_SEL_MIN     (PMIC_GPIO_PIN1_FXN_SEL_GPIO)
#define PMIC_GPIO_PIN1_FXN_SEL_MAX     (PMIC_GPIO_PIN1_FXN_SEL_NINT)
/** @} */

/**
 * @anchor Pmic_GpioPin2FxnSel
 * @name PMIC GPIO Pin 2 Function Select
 *
 * @brief GPIO pin 2 functionalities.
 *
 * @{
 */
#define PMIC_GPIO_PIN2_FXN_SEL_GPIO      (0U)
#define PMIC_GPIO_PIN2_FXN_SEL_CS_SPI    (1U)
#define PMIC_GPIO_PIN2_FXN_SEL_NSLEEP1   (2U)
#define PMIC_GPIO_PIN2_FXN_SEL_TRIG_WDOG (3U)
#define PMIC_GPIO_PIN2_FXN_SEL_MIN       (PMIC_GPIO_PIN2_FXN_SEL_GPIO)
#define PMIC_GPIO_PIN2_FXN_SEL_MAX       (PMIC_GPIO_PIN2_FXN_SEL_TRIG_WDOG)
/** @} */

/**
 * @anchor Pmic_GpioPin3FxnSel
 * @name PMIC GPIO Pin 3 Function Select
 *
 * @brief GPIO pin 3 functionalities.
 *
 * @{
 */
#define PMIC_GPIO_PIN3_FXN_SEL_GPIO    (0U)
#define PMIC_GPIO_PIN3_FXN_SEL_NSLEEP1 (2U)
#define PMIC_GPIO_PIN3_FXN_SEL_PB      (3U)
#define PMIC_GPIO_PIN3_FXN_SEL_MIN     (PMIC_GPIO_PIN3_FXN_SEL_GPIO)
#define PMIC_GPIO_PIN3_FXN_SEL_MAX     (PMIC_GPIO_PIN3_FXN_SEL_PB)
/** @} */

/**
 * @anchor Pmic_GpioPin4FxnSel
 * @name PMIC GPIO Pin 4 Function Select
 *
 * @brief GPIO pin 4 functionalities.
 *
 * @{
 */
#define PMIC_GPIO_PIN4_FXN_SEL_GPIO    (0U)
#define PMIC_GPIO_PIN4_FXN_SEL_NSLEEP1 (2U)
#define PMIC_GPIO_PIN4_FXN_SEL_ADC_IN  (3U)
#define PMIC_GPIO_PIN4_FXN_SEL_MIN     (PMIC_GPIO_PIN4_FXN_SEL_GPIO)
#define PMIC_GPIO_PIN4_FXN_SEL_MAX     (PMIC_GPIO_PIN4_FXN_SEL_ADC_IN)
/** @} */

/**
 * @anchor Pmic_GpioPin5FxnSel
 * @name PMIC GPIO Pin 5 Function Select
 *
 * @brief GPIO pin 5 functionalities.
 *
 * @{
 */
#define PMIC_GPIO_PIN5_FXN_SEL_GPIO      (0U)
#define PMIC_GPIO_PIN5_FXN_SEL_WKUP      (1U)
#define PMIC_GPIO_PIN5_FXN_SEL_SYNCCLKIN (2U)
#define PMIC_GPIO_PIN5_FXN_SEL_ADC_IN    (3U)
#define PMIC_GPIO_PIN5_FXN_SEL_MIN       (PMIC_GPIO_PIN5_FXN_SEL_GPIO)
#define PMIC_GPIO_PIN5_FXN_SEL_MAX       (PMIC_GPIO_PIN5_FXN_SEL_ADC_IN)
/** @} */

/**
 * @anchor Pmic_GpioPin6FxnSel
 * @name PMIC GPIO Pin 6 Function Select
 *
 * @brief GPIO pin 6 functionalities.
 *
 * @{
 */
#define PMIC_GPIO_PIN6_FXN_SEL_GPIO      (0U)
#define PMIC_GPIO_PIN6_FXN_SEL_NSLEEP2   (1U)
#define PMIC_GPIO_PIN6_FXN_SEL_SYNCCLKIN (3U)
#define PMIC_GPIO_PIN6_FXN_SEL_WKUP      (4U)
#define PMIC_GPIO_PIN6_FXN_SEL_MIN       (PMIC_GPIO_PIN6_FXN_SEL_GPIO)
#define PMIC_GPIO_PIN6_FXN_SEL_MAX       (PMIC_GPIO_PIN6_FXN_SEL_WKUP)
/** @} */

/**
 * @anchor Pmic_GpioPinPuSel
 * @name PMIC GPIO Pull-Up/Pull-Down Resistor Selection
 *
 * @brief GPIO resistor selection.
 *
 * @{
 */
#define PMIC_GPIO_PIN_PULL_DOWN_RESISTOR (0U)
#define PMIC_GPIO_PIN_PULL_UP_RESISTOR   (1U)
#define PMIC_GPIO_PIN_PU_SEL_MIN         (PMIC_GPIO_PIN_PULL_DOWN_RESISTOR)
#define PMIC_GPIO_PIN_PU_SEL_MAX         (PMIC_GPIO_PIN_PULL_UP_RESISTOR)
/** @} */

/**
 * @anchor Pmic_GpioPinType
 * @name PMIC GPIO Type
 *
 * @brief GPIO signal type.
 *
 * @{
 */
#define PMIC_GPIO_PIN_PUSH_PULL  (0U)
#define PMIC_GPIO_PIN_OPEN_DRAIN (1U)
#define PMIC_GPIO_PIN_TYPE_MIN   (PMIC_GPIO_PIN_PUSH_PULL)
#define PMIC_GPIO_PIN_TYPE_MAX   (PMIC_GPIO_PIN_OPEN_DRAIN)
/** @} */

/**
 * @anchor Pmic_GpioPinDir
 * @name PMIC GPIO Direction
 *
 * @brief PMIC signal direction.
 *
 * @{
 */
#define PMIC_GPIO_PIN_INPUT   (0U)
#define PMIC_GPIO_PIN_OUTPUT  (1U)
#define PMIC_GPIO_PIN_DIR_MIN (PMIC_GPIO_PIN_INPUT)
#define PMIC_GPIO_PIN_DIR_MAX (PMIC_GPIO_PIN_OUTPUT)
/** @} */

/**
 * @anchor Pmic_GpioEnPbVSenseFxnSel
 * @name PMIC GPIO ENABLE/PB/VSENSE Pin Function Select
 *
 * @brief Functionalities of the ENABLE/PB/VSENSE pin.
 *
 * @{
 */
#define PMIC_GPIO_EN_PB_VSENSE_FXN_SEL_ENABLE (0U)
#define PMIC_GPIO_EN_PB_VSENSE_FXN_SEL_PB     (1U)
#define PMIC_GPIO_EN_PB_VSENSE_FXN_SEL_VSENSE (2U)
#define PMIC_GPIO_EN_PB_VSENSE_FXN_SEL_MIN    (PMIC_GPIO_EN_PB_VSENSE_FXN_SEL_ENABLE)
#define PMIC_GPIO_EN_PB_VSENSE_FXN_SEL_MAX    (PMIC_EN_PB_VSENSE_FXN_SEL_VSENSE)
/** @} */

/**
 * @anchor Pmic_GpioEnPbDegl
 * @name PMIC GPIO ENABLE/PB Deglitch
 *
 * @brief Deglitch configuration for ENABLE and PB functionalities of the
 * EN/PB/VSENSE pin.
 *
 * @{
 */
#define PMIC_GPIO_EN_DEGLITCH_120_US (0U)
#define PMIC_GPIO_EN_DEGLITCH_50_MS  (1U)
#define PMIC_GPIO_PB_DEGLITCH_200_MS (0U)
#define PMIC_GPIO_PB_DEGLITCH_600_MS (1U)
#define PMIC_GPIO_EN_DEGL_MIN        (PMIC_GPIO_EN_DEGLITCH_120_US)
#define PMIC_GPIO_EN_DEGL_MAX        (PMIC_GPIO_EN_DEGLITCH_50_MS)
#define PMIC_GPIO_PB_DEGL_MIN        (PMIC_GPIO_PB_DEGLITCH_200_MS)
#define PMIC_GPIO_PB_DEGL_MAX        (PMIC_GPIO_PB_DEGLITCH_600_MS)
/** @} */

/**
 * @anchor Pmic_GpioNIntEnDrvFxnSel
 * @name PMIC GPIO nINT/EN_DRV Function Select
 *
 * @brief Functionalities of the nINT/EN_DRV pin.
 *
 * @{
 */
#define PMIC_GPIO_NINT_ENDRV_FXN_SEL_NINT   (0U)
#define PMIC_GPIO_NINT_ENDRV_FXN_SEL_EN_DRV (1U)
#define PMIC_GPIO_NINT_ENDRV_FXN_SEL_MIN    (PMIC_GPIO_NINT_ENDRV_FXN_SEL_NINT)
#define PMIC_GPIO_NINT_ENDRV_FXN_SEL_MAX    (PMIC_GPIO_NINT_ENDRV_FXN_SEL_EN_DRV)
/** @} */

/**
 * @anchor Pmic_GpioPinCfgValidParams
 * @name PMIC GPIO Pin Configuration Structure Valid Parameters
 *
 * @brief Definitions used to indicate valid parameters of `Pmic_GpioPinCfg_t`.
 * Set the `validParams` member of `Pmic_GpioPinCfg_t` equal to a combination
 * of these defines by using the `OR` operator.
 *
 * @{
 */
#define PMIC_GPIO_FXN_SEL_VALID     (1U << 0U)
#define PMIC_GPIO_PU_SEL_VALID      (1U << 1U)
#define PMIC_GPIO_TYPE_VALID        (1U << 2U)
#define PMIC_GPIO_DIR_VALID         (1U << 3U)
#define PMIC_GPIO_DEGL_EN_VALID     (1U << 4U)
#define PMIC_GPIO_RESISTOR_EN_VALID (1U << 5U)
/** @} */

/**
 * @anchor Pmic_GpioEnPbVSenseCfgValidParams
 * @name PMIC GPIO EN/PB/VSENSE Pin Configuration Structure Valid Parameters
 *
 * @brief Definitions used to indicate valid parameters of `Pmic_GpioEnPbVSenseCfg_t`.
 * Set the `validParams` member of `Pmic_GpioEnPbVSenseCfg_t` equal to a combination
 * of these defines by using the `OR` operator.
 *
 * @{
 */
#define PMIC_GPIO_EN_PB_VSENSE_FXN_SEL_VALID    (1U << 0U)
#define PMIC_GPIO_EN_PB_VSENSE_EN_PB_DEGL_VALID (1U << 1U)
/** @} */

/**
 * @anchor Pmic_GpioNIntEnDrvCfgValidParams
 * @name PMIC GPIO nINT/EN_DRV Pin Configuration Structure Valid Parameters
 *
 * @brief Definitions used to indicate valid parameters of `Pmic_GpioNIntEnDrvCfg_t`.
 * Set the `validParams` member of `Pmic_GpioNIntEnDrvCfg_t` equal to a combination
 * of these defines by using the `OR` operator.
 *
 * @{
 */
#define PMIC_GPIO_NINT_ENDRV_FXN_SEL_VALID        (1U << 0U)
#define PMIC_GPIO_NINT_ENDRV_EN_PU_RESISTOR_VALID (1U << 1U)
/** @} */

/**
 * @brief Pmic_GpioEnPbVSenseStatusValidParams
 * @name PMIC GPIO EN/PB/VSENSE Pin Status Structure Valid Parameters
 *
 * @brief Definitions used to indicate valid parameters of `Pmic_GpioEnPbVSenseStatus_t`.
 * Set the `validParams` member of `Pmic_GpioEnPbVSenseStatus_t` equal to a combination
 * of these defines by using the `OR` operator.
 */
#define PMIC_GPIO_PB_LVL_HIGH_VALID     (1U << 0U)
#define PMIC_GPIO_EN_LVL_HIGH_VALID     (1U << 1U)
#define PMIC_GPIO_VSENSE_LVL_HIGH_VALID (1U << 2U)
/** @} */

/* ========================================================================== */
/*                           Structures and Enums                             */
/* ========================================================================== */

/**
 * @anchor Pmic_GpioPinCfg
 * @name PMIC GPIO Pin Configuration Structure
 *
 * @brief Structure used to set and get GPIO pin configurations.
 *
 * @param validParams Each bit in this variable corresponds to a member in this
 * structure. Specifically, if a bit is set to 1 in this variable, the corresponding
 * structure member is valid and will be considered by the driver API that is using
 * this data structure. Otherwise, if a bit is set to 0, the corresponding structure
 * member is invalid and will not be considered by the driver API that is using
 * this data structure. For possible valid parameter values, refer to
 * @ref Pmic_GpioPinCfgValidParams.
 *
 * @param pinNum GPIO identifier. For valid values, refer to @ref Pmic_GpioPinNum.
 * There is no validParam for this member - the pin number must be specified.
 *
 * @param fxnSel GPIO function select.
 * For GPIO1, refer to @ref Pmic_GpioPin1FxnSel.
 * For GPIO2, refer to @ref Pmic_GpioPin2FxnSel.
 * For GPIO3, refer to @ref Pmic_GpioPin3FxnSel.
 * For GPIO4, refer to @ref Pmic_GpioPin4FxnSel.
 * For GPIO5, refer to @ref Pmic_GpioPin5FxnSel.
 * For GPIO6, refer to @ref Pmic_GpioPin6FxnSel.
 *
 * @param puSel GPIO pull-up/pull-down resistor selection. For valid values,
 * refer to @ref Pmic_GpioPinPuSel.
 *
 * @param type GPIO push-pull/open-drain output selection (valid only for GPIO
 * outputs). For valid values, refer to @ref Pmic_GpioPinType.
 *
 * @param dir GPIO signal direction selection. For valid values, refer to
 * @ref Pmic_GpioPinDir.
 *
 * @param deglEn GPIO deglitch enable (valid only for GPIO inputs). If set to
 * true, there will be an 8 microsecond deglitch time for input signals.
 * Otherwise, there will be no deglitch, only synchronization.
 *
 * @param resistorEn GPIO pull-up/pull-down resistor enable. If set to true,
 * pull-up/pull-down resistor is enabled for the GPIO. Otherwise, if set to
 * false, pull-up/pull-down resistor is disabled for the GPIO.
 *
 * @{
 */
typedef struct Pmic_GpioPinCfg_s {
    uint32_t validParams;

    uint8_t pinNum;
    uint8_t fxnSel;
    uint8_t puSel;
    uint8_t type;
    uint8_t dir;

    bool deglEn;
    bool resistorEn;
} Pmic_GpioPinCfg_t;
/** @} */

/**
 * @anchor Pmic_GpioEnPbVSenseCfg
 * @name PMIC GPIO Enable/PB/VSENSE Pin Configuration Structure
 *
 * @brief Structure used to set and get configurations of the Enable/PB/VSENSE
 * pin on the PMIC.
 *
 * @param validParams Each bit in this variable corresponds to a member in this
 * structure. Specifically, if a bit is set to 1 in this variable, the corresponding
 * structure member is valid and will be considered by the driver API that is using
 * this data structure. Otherwise, if a bit is set to 0, the corresponding structure
 * member is invalid and will not be considered by the driver API that is using
 * this data structure. For possible valid parameter values, refer to
 * @ref Pmic_GpioEnPbVSenseCfgValidParams.
 *
 * @param fxnSel ENABLE/PB/VSENSE pin functionality selection. For valid values,
 * refer to @ref Pmic_GpioEnPbVSenseFxnSel.
 *
 * @param enPbDegl Deglitch configuraton for when the EN/PB/VSENSE pin is
 * configured to ENABLE or PB functionality. For valid values, refer to
 * @ref Pmic_GpioEnPbDegl.
 *
 * @{
 */
typedef struct Pmic_GpioEnPbVSenseCfg_s {
    uint32_t validParams;

    uint8_t fxnSel;
    uint8_t enPbDegl;
} Pmic_GpioEnPbVSenseCfg_t;
/** @} */

/**
 * @anchor Pmic_GpioNIntEnDrvCfg
 * @name PMIC GPIO nInt/EN_DRV Pin Configuration Structure
 *
 * @brief Structure used to set and get configurations of the nInt/EN_DRV pin on
 * the PMIC.
 *
 * @param validParams Each bit in this variable corresponds to a member in this
 * structure. Specifically, if a bit is set to 1 in this variable, the corresponding
 * structure member is valid and will be considered by the driver API that is using
 * this data structure. Otherwise, if a bit is set to 0, the corresponding structure
 * member is invalid and will not be considered by the driver API that is using
 * this data structure. For possible valid parameter values, refer to
 * @ref Pmic_GpioNIntEnDrvCfgValidParams.
 *
 * @param fxnSel nInt/EN_DRV pin function select. For valid values, refer to
 * @ref Pmic_GpioNIntEnDrvFxnSel.
 *
 * @param enPuResistor Activate or deactivate the pull-up resistor for the
 * nINT/EN_DRV pin. If set to false, the pull-up resistor is deactivated.
 * If set to true, the pull-up resistor is activated when the signal is driven
 * high.
 *
 * @{
 */
typedef struct Pmic_GpioNIntEnDrvCfg_s {
    uint32_t validParams;

    uint8_t fxnSel;
    bool enPuResistor;
} Pmic_GpioNIntEnDrvCfg_t;
/** @} */

/**
 * @anchor Pmic_GpioEnPbVSenseStatus
 * @name PMIC GPIO EN/PB/VSENSE Pin Status
 *
 * @brief Structure used to read the status of the EN/PB/VSENSE pin.
 *
 * @param validParams Each bit in this variable corresponds to a member in this
 * structure. Specifically, if a bit is set to 1 in this variable, the corresponding
 * structure member is valid and will be considered by the driver API that is using
 * this data structure. Otherwise, if a bit is set to 0, the corresponding structure
 * member is invalid and will not be considered by the driver API that is using
 * this data structure. For possible valid parameter values, refer to
 * @ref Pmic_GpioEnPbVSenseStatusValidParams.
 *
 * @param pbLvlHigh PB status indication.
 *
 * @param enLvlHigh ENABLE status indication.
 *
 * @param vsenseLvlHigh VSENSE status indication.
 *
 * @{
 */
typedef struct Pmic_GpioEnPbVSenseStatus_s {
    uint32_t validParams;

    bool pbLvlHigh;
    bool enLvlHigh;
    bool vsenseLvlHigh;
} Pmic_GpioEnPbVSenseStatus_t;
/** @} */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 * @brief Set PMIC GPIO pin configurations.
 *
 * Design: PMICDRV-700
 * Architecture: PMICDRV-507, PMICDRV-516, PMICDRV-508, PMICDRV-523, PMICDRV-551, PMICDRV-506
 *               PMICDRV-504, PMICDRV-522, PMICDRV-541, PMICDRV-521, PMICDRV-512
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param gpioPinCfg [IN] Desired PMIC GPIO pin configurations to set. For more
 * information on GPIO pin configurations, refer to @ref Pmic_GpioPinCfg.
 *
 * @return PMIC_ST_SUCCESS if PMIC GPIO pin configurations have been set, error
 * code otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_gpioSetPinCfg(const Pmic_Handle_t *handle, const Pmic_GpioPinCfg_t *gpioPinCfg);

/**
 * @brief Get PMIC GPIO pin configurations.
 *
 * Design: PMICDRV-701
 * Architecture: PMICDRV-507, PMICDRV-516, PMICDRV-508, PMICDRV-551, PMICDRV-506, PMICDRV-504
 *               PMICDRV-522, PMICDRV-528, PMICDRV-541, PMICDRV-521, PMICDRV-512
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param gpioPinCfg [IN] GPIO pin configurations obtained from the PMIC. For more
 * information on GPIO pin configurations, refer to @ref Pmic_GpioPinCfg.
 *
 * @return PMIC_ST_SUCCESS if PMIC GPIO pin configurations have been obtained,
 * error code otherwise. For valid success/error codes, refer to
 * @ref Pmic_ErrorCodes.
 */
int32_t Pmic_gpioGetPinCfg(const Pmic_Handle_t *handle, Pmic_GpioPinCfg_t *gpioPinCfg);

/**
 * @brief Set PMIC GPIO pin value to be high or low.
 *
 * Design: PMICDRV-702
 * Architecture: PMICDRV-507, PMICDRV-516, PMICDRV-508, PMICDRV-523, PMICDRV-551, PMICDRV-506
 *               PMICDRV-504, PMICDRV-522, PMICDRV-541, PMICDRV-521, PMICDRV-512
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param gpioPin [IN] GPIO pin identifier. For valid values, refer to
 * @ref Pmic_GpioPinNum.
 *
 * @param high [IN] True - GPIO pin signal level is set high; false - GPIO pin
 * signal level is set low.
 *
 * @return PMIC_ST_SUCCESS if PMIC GPIO pin signal level has been set, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_gpioSetPinVal(const Pmic_Handle_t *handle, uint8_t gpioPin, bool high);

/**
 * @brief Get PMIC GPIO pin value.
 *
 * Design: PMICDRV-703
 * Architecture: PMICDRV-507, PMICDRV-516, PMICDRV-508, PMICDRV-551, PMICDRV-506, PMICDRV-504
 *               PMICDRV-522, PMICDRV-528, PMICDRV-541, PMICDRV-521, PMICDRV-512
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param gpioPin [IN] GPIO pin identifier. For valid values, refer to
 * @ref Pmic_GpioPinNum.
 *
 * @param high [OUT] True - GPIO pin signal level is high; false - GPIO pin signal
 * level is low.
 *
 * @return PMIC_ST_SUCCESS if PMIC GPIO pin signal level has been obtained,
 * error code otherwise. For valid success/error codes, refer to
 * @ref Pmic_ErrorCodes.
 */
int32_t Pmic_gpioGetPinVal(const Pmic_Handle_t *handle, uint8_t gpioPin, bool *high);

/**
 * @brief Set nINT/EN_DRV pin configurations.
 *
 * Design: PMICDRV-704
 * Architecture: PMICDRV-507, PMICDRV-516, PMICDRV-508, PMICDRV-523, PMICDRV-551, PMICDRV-506
 *               PMICDRV-504, PMICDRV-522, PMICDRV-541, PMICDRV-521, PMICDRV-512
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param nIntEnDrvCfg [IN] Desired nINT/EN_DRV pin configurations to set. For
 * more information on nINT/EN_DRV pin configurations, refer to
 * @ref Pmic_GpioEnPbVSenseCfg.
 *
 * @return PMIC_ST_SUCCESS if the nINT/EN_DRV pin has been configured, error
 * code otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_gpioSetNIntEnDrvCfg(const Pmic_Handle_t *handle, const Pmic_GpioNIntEnDrvCfg_t *nIntEnDrvCfg);

/**
 * @brief Get nINT/EN_DRV pin configurations. This API supports getting the same
 * configurations that are settable by 'Pmic_GpioSetNIntEnDrvCfg()'.
 *
 * Design: PMICDRV-705
 * Architecture: PMICDRV-507, PMICDRV-516, PMICDRV-508, PMICDRV-551, PMICDRV-506, PMICDRV-504
 *               PMICDRV-522, PMICDRV-528, PMICDRV-541, PMICDRV-521, PMICDRV-512
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param nIntEnDrvCfg [OUT] nINT/EN_DRV pin configurations obtained from the PMIC.
 * For more information on nINT/EN_DRV pin configurations, refer to
 * @ref Pmic_GpioEnPbVSenseCfg.
 *
 * @return PMIC_ST_SUCCESS if nINT/EN_DRV configurations have been obtained,
 * error code otherwise. For valid success/error codes, refer to
 * @ref Pmic_ErrorCodes.
 */
int32_t Pmic_gpioGetNIntEnDrvCfg(const Pmic_Handle_t *handle, Pmic_GpioNIntEnDrvCfg_t *nIntEnDrvCfg);

/**
 * @brief Get nINT/EN_DRV pin value (signal level).
 *
 * Design: PMICDRV-706
 * Architecture: PMICDRV-507, PMICDRV-516, PMICDRV-508, PMICDRV-551, PMICDRV-506, PMICDRV-504
 *               PMICDRV-522, PMICDRV-528, PMICDRV-541, PMICDRV-521, PMICDRV-512
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param high [OUT] True - nINT/EN_DRV pin signal level is high; false - nINT/EN_DRV
 * pin signal level is low.
 *
 * @return PMIC_ST_SUCCESS if nINT/EN_DRV pin value (signal level) has been
 * obtained, error code otherwise. For valid success/error codes, refer to
 * @ref Pmic_ErrorCodes.
 */
int32_t Pmic_gpioGetNIntEnDrvVal(const Pmic_Handle_t *handle, bool *high);

/**
 * @brief Set PMIC EN/PB/VSENSE pin configurations.
 *
 * Design: PMICDRV-707
 * Architecture: PMICDRV-507, PMICDRV-516, PMICDRV-508, PMICDRV-523, PMICDRV-551, PMICDRV-506
 *               PMICDRV-504, PMICDRV-522, PMICDRV-541, PMICDRV-521, PMICDRV-512
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param enPbVSenseCfg [IN] Desired EN/PB/VSENSE pin configurations to set. For
 * more information on EN/PB/VSENSE pin configurations, refer to
 * @ref Pmic_GpioNIntEnDrvCfg.
 *
 * @return PMIC_ST_SUCCESS if EN/PB/VSENSE pin configurations have been set,
 * error code otherwise. For valid success/error codes, refer to
 * @ref Pmic_ErrorCodes.
 */
int32_t Pmic_gpioSetEnPbVSenseCfg(const Pmic_Handle_t *handle, const Pmic_GpioNIntEnDrvCfg_t *enPbVSenseCfg);

/**
 * @brief Get PMIC EN/PB/VSENSE pin configurations.
 *
 * Design: PMICDRV-708
 * Architecture: PMICDRV-507, PMICDRV-516, PMICDRV-508, PMICDRV-551, PMICDRV-506, PMICDRV-504
 *               PMICDRV-522, PMICDRV-528, PMICDRV-541, PMICDRV-521, PMICDRV-512
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param enPbVSenseCfg [OUT] EN/PB/VSENSE pin configurations obtained from the
 * PMIC. For more information on EN/PB/VSENSE pin configurations, refer to
 * @ref Pmic_GpioNIntEnDrvCfg.
 *
 * @return PMIC_ST_SUCCESS if EN/PB/VSENSE pin configurations have been obtained,
 * error code otherwise. For valid success/error codes, refer to
 * @ref Pmic_ErrorCodes.
 */
int32_t Pmic_gpioGetEnPbVSenseCfg(const Pmic_Handle_t *handle, Pmic_GpioNIntEnDrvCfg_t *enPbVSenseCfg);

/**
 * @brief Get PMIC EN/PB/VSENSE pin status.
 *
 * Design: PMICDRV-709
 * Architecture: PMICDRV-507, PMICDRV-516, PMICDRV-508, PMICDRV-551, PMICDRV-506, PMICDRV-504
 *               PMICDRV-522, PMICDRV-528, PMICDRV-541, PMICDRV-521, PMICDRV-512
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param enPbVSenseStatus [OUT] EN/PB/VSENSE pin status obtained from the PMIC.
 * For more information on EN/PB/VSENSE pin statuses, refer to
 * @ref Pmic_GpioEnPbVSenseStatus.
 *
 * @return PMIC_ST_SUCCESS if EN/PB/VSENSE pin statuses have been obtained, error
 * code otherwise. For valid success/error codes refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_gpioGetEnPbVSenseStatus(const Pmic_Handle_t *handle, Pmic_GpioEnPbVSenseStatus_t *enPbVSenseStatus);

/**
 * @brief Get PMIC nRSTOUT pin value (signal level).
 *
 * Design: PMICDRV-710
 * Architecture: PMICDRV-507, PMICDRV-516, PMICDRV-508, PMICDRV-551, PMICDRV-506, PMICDRV-504
 *               PMICDRV-522, PMICDRV-528, PMICDRV-541, PMICDRV-521, PMICDRV-512
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param high [OUT] Signal level of the nRSTOUT pin. True - signal level is high.
 * False - signal level is low.
 *
 * @return PMIC_ST_SUCCESS if nRSTOUT pin value has been obtained, error code
 * otherwise. For valid success/error codes refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_gpioGetNRstOutVal(const Pmic_Handle_t *handle, bool *high);

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* PMIC_GPIO_H */
