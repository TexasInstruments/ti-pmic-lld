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
#ifndef PMIC_GPIO_H
#define PMIC_GPIO_H

/**
 * @file pmic_gpio.h
 * @brief PMIC Driver GPIO API/Interface
 */

/**
 * @defgroup DRV_PMIC_GPIO_MODULE PMIC GPIO Module
 * @brief APIs used to configure and interact with PMIC GPIOs.
 */

/*==========================================================================*/
/*                             Include Files                                */
/*==========================================================================*/

#include <stdbool.h>
#include <stdint.h>

#include "pmic_common.h"
#include "pmic_core.h"

#ifdef __cplusplus
extern "C" {
#endif

/*==========================================================================*/
/*                           Macros & Typedefs                              */
/*==========================================================================*/

/**
 * @anchor Pmic_configurableGpi
 * @name PMIC Configurable GPIs
 *
 * @brief Enumeration of the configurable PMIC GPIs.
 *
 * @{
 */
#define PMIC_GPI1                   (1U)
#define PMIC_GPI4                   (2U)
#define PMIC_GPI_MIN                (PMIC_GPI1)
#define PMIC_GPI_MAX                (PMIC_GPI4)
/** @} */

/**
 * @anchor Pmic_configurableGpo
 * @name PMIC Configurable GPOs
 *
 * @brief Enumeration of the configurable PMIC GPOs.
 *
 * @{
 */
#define PMIC_GPO1                   (3U)
#define PMIC_GPO2                   (4U)
#define PMIC_GPO3                   (5U)
#define PMIC_GPO4                   (6U)
#define PMIC_GPO_MIN                (PMIC_GPO1)
#define PMIC_GPO_MAX                (PMIC_GPO4)
/** @} */

/**
 * @anchor Pmic_numConfigurableGpio
 * @name PMIC Number of Configurable GPIOs
 *
 * @brief Number of Configurable GPIOs.
 *
 * @{
 */
#define PMIC_NUM_CONFIGURABLE_GPIO (6U)
/** @} */

/**
 * @anchor Pmic_gpioCfgValidParamBitPos
 * @name PMIC GPIO Configuration Structure Param Bit Positions
 *
 * @{
 */
#define PMIC_CFG_GPI1_VALID (1U << 0U)
#define PMIC_CFG_GPI4_VALID (1U << 1U)
#define PMIC_CFG_GPO1_VALID (1U << 2U)
#define PMIC_CFG_GPO2_VALID (1U << 3U)
#define PMIC_CFG_GPO3_VALID (1U << 4U)
#define PMIC_CFG_GPO4_VALID (1U << 5U)
/** @} */

/**
 * @anchor Pmic_GpioSafeOutCfgValidParam
 * @name PMIC GPIO SAFEOUT Configuration Valid Params
 *
 * @{
 */
#define PMIC_GPIO_SAFEOUT1_EN_VALID (1UL << 0U)
#define PMIC_GPIO_SAFEOUT2_EN_VALID (1UL << 1U)
/** @} */

/**
 * @anchor Pmic_gpi1Cfg
 * @name PMIC GPI1 Configuration Values
 *
 * @brief Possible functionalities of GPI1.
 *
 * @{
 */
#define PMIC_GPI1_ESM_IN    (0U)
#define PMIC_GPI1_WD_IN     (1U)
#define PMIC_GPI1_CFG_MAX   (PMIC_GPI1_WD_IN)
/** @} */

/**
 * @anchor Pmic_gpi4Cfg
 * @name PMIC GPI4 Configuration Values
 *
 * @brief Possible functionalities of GPI4.
 *
 * @attention It is not recommended to set GPI4 to COS_N functionality if your
 * PMIC device does not have rotational counter.
 *
 * @{
 */
#define PMIC_GPI4_COMPARATOR    (0U)
#define PMIC_GPI4_WD_IN         (1U)
#define PMIC_GPI4_COS_N         (2U)
#define PMIC_GPI4_CFG_MAX       (PMIC_GPI4_COS_N)
/** @} */

/**
 * @anchor Pmic_gpo1Cfg
 * @name PMIC GPO1 Configuration Values
 *
 * @brief Possible functionalities of GPO1.
 *
 * @attention It is not recommended to set GPO1 to SIN_N_O functionality if your
 * PMIC device does not have rotational counter.
 *
 * @{
 */
#define PMIC_GPO1_LOW_LVL   (0U)
#define PMIC_GPO1_HIGH_LVL  (1U)
#define PMIC_GPO1_HIZ       (2U)
#define PMIC_GPO1_NINT      (3U)
#define PMIC_GPO1_EN_OUT    (4U)
#define PMIC_GPO1_EN_OUT2   (5U)
#define PMIC_GPO1_SIN_N_O   (7U)
#define PMIC_GPO1_CFG_MAX   (PMIC_GPO1_SIN_N_O)
/** @} */

/**
 * @anchor Pmic_gpo2Cfg
 * @name PMIC GPO2 Configuration Values
 *
 * @brief Possible functionalities of GPO2.
 *
 * @attention It is not recommended to set GPO2 to SIN_P_O functionality if your
 * PMIC device does not have rotational counter.
 *
 * @{
 */
#define PMIC_GPO2_LOW_LVL       (0U)
#define PMIC_GPO2_HIGH_LVL      (1U)
#define PMIC_GPO2_HIZ           (2U)
#define PMIC_GPO2_COMP1_OUT     (3U)
#define PMIC_GPO2_EN_OUT2       (4U)
#define PMIC_GPO2_SYNCCLKOUT    (5U)
#define PMIC_GPO2_PGOOD         (6U)
#define PMIC_GPO2_SIN_P_O       (7U)
#define PMIC_GPO2_CFG_MAX       (PMIC_GPO2_SIN_P_O)
/** @} */

/**
 * @anchor Pmic_gpo3Cfg
 * @name PMIC GPO3 Configuration Values
 *
 * @brief Possible functionalities of GPO3.
 *
 * @attention It is not recommended to set GPO3 to COS_P_O functionality if your
 * PMIC device does not have rotational counter.
 *
 * @{
 */
#define PMIC_GPO3_LOW_LVL   (0U)
#define PMIC_GPO3_HIGH_LVL  (1U)
#define PMIC_GPO3_HIZ       (2U)
#define PMIC_GPO3_PGOOD     (3U)
#define PMIC_GPO3_COMP2_OUT (4U)
#define PMIC_GPO3_EN_OUT2   (5U)
#define PMIC_GPO3_SAFE_OUT2 (6U)
#define PMIC_GPO3_COS_P_O   (7U)
#define PMIC_GPO3_CFG_MAX   (PMIC_GPO3_COS_P_O)
/** @} */

/**
 * @anchor Pmic_gpo4Cfg
 * @name PMIC GPO4 Configuration Values
 *
 * @brief Possible functionalities of GPO4.
 *
 * @attention It is not recommended to set GPO4 to COS_N_O functionality if your
 * PMIC device does not have rotational counter.
 *
 * @{
 */
#define PMIC_GPO4_LOW_LVL   (0U)
#define PMIC_GPO4_HIGH_LVL  (1U)
#define PMIC_GPO4_HIZ       (2U)
#define PMIC_GPO4_SAFE_OUT2 (3U)
#define PMIC_GPO4_EN_OUT    (4U)
#define PMIC_GPO4_NINT      (5U)
#define PMIC_GPO4_PGOOD     (6U)
#define PMIC_GPO4_COS_N_O   (7U)
#define PMIC_GPO4_CFG_MAX   (PMIC_GPO4_COS_N_O)
/** @} */

/*==========================================================================*/
/*                          Structures and Enums                            */
/*==========================================================================*/

/**
 * @anchor Pmic_GpioCfg
 * @name PMIC GPIO Configuration Struct
 *
 * @brief Used to set/get configurations of PMIC GPIOs.
 *
 * @note GPI2 and GPI3 are compare module inputs and do not have the same
 * configurability as the other GPIOs. Thus, they are omitted from this
 * configuration data structure.
 *
 * @param validParams Indicate valid parameters. See
 * @ref Pmic_gpioCfgValidParamBitShiftVal for valid values.
 *
 * @param gpi1 General purpose input 1. For valid values, see @ref Pmic_gpi1Cfg
 *
 * @param gpi4 General purpose input 4. For valid values, see @ref Pmic_gpi4Cfg
 *
 * @param gpo1 General purpose output 1. For valid values, see @ref Pmic_gpo1Cfg
 *
 * @param gpo2 General purpose output 2. For valid values, see @ref Pmic_gpo2Cfg
 *
 * @param gpo3 General purpose output 3. For valid values, see @ref Pmic_gpo3Cfg
 *
 * @param gpo4 General purpose output 4. For valid values, see @ref Pmic_gpo4Cfg
 *
 */
typedef struct Pmic_GpioCfg_s {
    uint32_t validParams;

    /* General purpose inputs */
    uint8_t gpi1;
    uint8_t gpi4;

    /* General purpose outputs */
    uint8_t gpo1;
    uint8_t gpo2;
    uint8_t gpo3;
    uint8_t gpo4;
} Pmic_GpioCfg_t;

/**
 * @anchor Pmic_GpioSafeOutCfg
 * @name PMIC GPIO SAFEOUT Configuration Struct
 *
 * @brief Used to enable/disable SAFEOUT1 and SAFEOUT2 output pins
 *
 * @param validParams Selection of structure parameters to be set, from
 * @ref Pmic_GpioSafeOutCfgValidParam
 * @param safeOut1En SAFEOUT1 pin enable (false=disable, true=enable).
 * Valid when PMIC_GPIO_SAFEOUT1_EN_VALID is set
 * @param safeOut2En SAFEOUT2 pin enable (false=disable, true=enable).
 * Valid when PMIC_GPIO_SAFEOUT2_EN_VALID is set
 */
typedef struct Pmic_GpioSafeOutCfg_s {
    uint32_t validParams;
    bool safeOut1En;
    bool safeOut2En;
} Pmic_GpioSafeOutCfg_t;

/*==========================================================================*/
/*                         Function Declarations                            */
/*==========================================================================*/

/**
 * @brief Set PMIC GPIO configurations.
 *
 * Design: PMICDRV-615
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-521, PMICDRV-522, PMICDRV-523, PMICDRV-541
 *
 * @details The following GPIOs are configurable by this API
 * 1. GPI1 (validParams: PMIC_CFG_GPI1_VALID_SHIFT)
 * 2. GPI4 (validParams: PMIC_CFG_GPI4_VALID_SHIFT)
 * 3. GPO1 (validParams: PMIC_CFG_GPO1_VALID_SHIFT)
 * 4. GPO2 (validParams: PMIC_CFG_GPO2_VALID_SHIFT)
 * 5. GPO3 (validParams: PMIC_CFG_GPO3_VALID_SHIFT)
 * 6. GPO4 (validParams: PMIC_CFG_GPO4_VALID_SHIFT)
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param gpioCfg [IN] GPIO configurations to write to PMIC. For more
 * information, see @ref Pmic_GpioCfg.
 *
 * @return Success code if GPIO configurations have been set, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes
 */
int32_t Pmic_gpioSetCfg(const Pmic_Handle_t *handle, const Pmic_GpioCfg_t *gpioCfg);

/**
 * @brief Get PMIC GPIO configurations. This API supports getting the same
 * configurations that are settable by `Pmic_gpioSetCfg()`.
 *
 * Design: PMICDRV-616
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-521, PMICDRV-522, PMICDRV-528, PMICDRV-541
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param gpioCfg [OUT] GPIO configurations obtained from the PMIC. For more
 * information, see @ref Pmic_GpioCfg.
 *
 * @return Success code if GPIO configurations have been obtained, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes
 */
int32_t Pmic_gpioGetCfg(const Pmic_Handle_t *handle, Pmic_GpioCfg_t *gpioCfg);

/**
 * @brief Get the output value of a general purpose output pin on the PMIC.
 *
 * Design: PMICDRV-617
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-521, PMICDRV-522, PMICDRV-528, PMICDRV-541
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param gpo [IN] General purpose output pin number. For valid GPOs, see
 * @ref Pmic_configurableGpo
 *
 * @param high [OUT] When returned as true, the GPO pin is high. Else, the GPO
 * pin is low.
 *
 * @return Success code if GPO output value has been obtained, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes
 */
int32_t Pmic_gpioGetOutputValue(const Pmic_Handle_t *handle, uint8_t gpo, bool *high);

/**
 * @brief Enable or disable SAFEOUT pins.
 *
 * Design: PMICDRV-615
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-521, PMICDRV-522, PMICDRV-523, PMICDRV-541
 *
 * @param handle [IN] PMIC interface handle.
 * @param config [IN] GPIO SAFEOUT configuration with safeOut1En/safeOut2En set.
 *
 * @return PMIC_ST_SUCCESS if successful, error code otherwise.
 * For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_gpioSetSafeOutCfg(const Pmic_Handle_t *handle, const Pmic_GpioSafeOutCfg_t *config);

/**
 * @brief Get SAFEOUT pin configuration status.
 *
 * Design: PMICDRV-616
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-521, PMICDRV-522, PMICDRV-528, PMICDRV-541
 *
 * @param handle [IN]  PMIC interface handle.
 * @param config [OUT] GPIO SAFEOUT configuration to store SAFEOUT status.
 *
 * @return PMIC_ST_SUCCESS if successful, error code otherwise.
 * For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_gpioGetSafeOutCfg(const Pmic_Handle_t *handle, Pmic_GpioSafeOutCfg_t *config);

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* PMIC_GPIO_H */
