/******************************************************************************
 * Copyright (c) 2020 Texas Instruments Incorporated - http://www.ti.com
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
 *  \ingroup DRV_PMIC_MODULE
 *  \defgroup DRV_PMIC_GPIO_MODULE PMIC GPIO Driver API
 *      This Module explains about PMIC GPIO driver parameters and APIs usage.
 *  PMIC GPIO Driver module covers all GPIO features APIs. Like, set/get gpio
 *  pin functions, pull up/down, drive strength, output drain, pin value,
 *  enable/disable gpio interrupt and configure nPWRON or ENABLE pin features.
 *
 *  Supported PMIC devices for GPIO Module:
 *  1. TPS6594x (Leo PMIC Device)
 *  2. LP8764x  (Hera PMIC Device)
 *
 *  @{
 */

/**
 * \file   pmic_gpio.h
 *
 * \brief  PMIC Low Level Driver API/interface file for GPIO API
 */

#ifndef PMIC_GPIO_H_
#define PMIC_GPIO_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <pmic_core.h>
#include <cfg/tps6594x/pmic_gpio_tps6594x.h>
#include <cfg/lp8764x/pmic_gpio_lp8764x.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/**
 *  \anchor Pmic_Gpio_DeglitchTimeCfg
 *  \name   PMIC GPIO Deglitch Time Enable or Disable Configuration
 *
 *  @{
 */
#define PMIC_GPIO_DEGLITCH_DISABLE     (0U)
#define PMIC_GPIO_DEGLITCH_ENABLE      (1U)
/*  @} */

/**
 *  \anchor Pmic_Gpio_SignalDir
 *  \name   PMIC GPIO signal direction
 *
 *  @{
 */
#define PMIC_GPIO_INPUT                (0U)
#define PMIC_GPIO_OUTPUT               (1U)
/*  @} */

/**
 *  \anchor Pmic_Gpio_SignalType
 *  \name   PMIC GPIO signal type when configured as output
 *
 *  @{
 */
#define PMIC_GPIO_PUSH_PULL_OUTPUT     (0U)
#define PMIC_GPIO_OPEN_DRAIN_OUTPUT    (1U)
/*  @} */

/**
 *  \anchor Pmic_Gpio_SignalLvl
 *  \name   PMIC GPIO/NPWRON/ENABLE signal level
 *
 *  @{
 */
#define PMIC_GPIO_LOW                  (0U)
#define PMIC_GPIO_HIGH                 (1U)
/*  @} */

/**
 *  \anchor Pmic_Gpio_PU_PD_Sel
 *  \name   PMIC GPIO Pull-up/pull-down select
 *
 *  @{
 */
#define PMIC_GPIO_PD_SELECT            (0U)
#define PMIC_GPIO_PU_SELECT            (1U)
/*  @} */

/**
 *  \anchor Pmic_Gpio_PU_PD_En
 *  \name   PMIC GPIO Pull-up/pull-down enable/disable
 *
 *  @{
 */
#define PMIC_GPIO_PU_PD_DISABLE        (0U)
#define PMIC_GPIO_PU_PD_ENABLE         (1U)
/*  @} */

/**
 *  \anchor Pmic_GpioCflag
 *  \name   PMIC Pmic_GpioCfg_s member configuration type
 *
 *  @{
 */
 /** \brief validParams value used to set/get gpio pin Direction
  *         Valid only for GPIO pins only. Invalid for NPWRON/Enable */
#define PMIC_GPIO_CFG_DIR_VALID            (0x00U)
 /** \brief validParams value used to set/get output signal type
  *         Valid only for GPIO pins only. Invalid for NPWRON/Enable */
#define PMIC_GPIO_CFG_OD_VALID             (0x01U)
/** \brief validParams value used to set/get pullup/pull down control
 */
#define PMIC_GPIO_CFG_PULL_VALID           (0x02U)
/** \brief validParams value used to set/get signal deglitch time
 *         enable/disable */
#define PMIC_GPIO_CFG_DEGLITCH_VALID       (0x03U)
/** \brief validParams value used to set/get pin mux function */
#define PMIC_GPIO_CFG_PINFUNC_VALID        (0x04U)
/** \brief validParams value used to set/get pin polarity
 *         Valid only for Enable pin for TPS6594x Leo and LP8764x Hera device
 *         Invalid for NPWRON pin for TPS6594x Leo device*/
#define PMIC_ENABLE_CFG_POLARITY_VALID     (0x05U)
/*  @} */

/**
 *  \anchor Pmic_GpioPinCfgStructPrmBitShiftVal
 *  \name   PMIC GPIO Pin Configuration Structure Param Bit shift values
 *
 *  Application can use below shifted values to set the validParams
 *  struct member defined in Pmic_GpioCfg_t structure
 *
 *  @{
 */
#define PMIC_GPIO_CFG_DIR_VALID_SHIFT        \
                          (0x01U << PMIC_GPIO_CFG_DIR_VALID)
#define PMIC_GPIO_CFG_OD_VALID_SHIFT         \
                          (0x01U << PMIC_GPIO_CFG_OD_VALID)
#define PMIC_GPIO_CFG_PULL_VALID_SHIFT       \
                          (0x01U << PMIC_GPIO_CFG_PULL_VALID)
#define PMIC_GPIO_CFG_DEGLITCH_VALID_SHIFT   \
                          (0x01U << PMIC_GPIO_CFG_DEGLITCH_VALID)
#define PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT    \
                          (0x01U << PMIC_GPIO_CFG_PINFUNC_VALID)
#define PMIC_ENABLE_CFG_POLARITY_VALID_SHIFT \
                          (0x01U << PMIC_ENABLE_CFG_POLARITY_VALID)
/* @} */

/**
 *  \anchor Pmic_GpioPinPullCtrl
 *  \name   PMIC GPIO pull up/pull down selectionn
 *
 *  @{
 */
#define PMIC_GPIO_PULL_DISABLED        (0x0U)
#define PMIC_GPIO_PULL_DOWN            (0x1U)
#define PMIC_GPIO_PULL_UP              (0x2U)
/*  @} */

/**
 *  \anchor Pmic_GpioInterruptCfg
 *  \name   PMIC GPIO Interrupt selection
 *
 *  @{
 */
#define PMIC_GPIO_FALL_INTERRUPT        (0U)
#define PMIC_GPIO_RISE_INTERRUPT        (1U)
#define PMIC_GPIO_FALL_RISE_INTERRUPT   (2U)
#define PMIC_GPIO_DISABLE_INTERRUPT     (3U)
/*  @} */

/**
 *  \anchor Pmic_GpioInterruptPolCfg
 *  \name   PMIC GPIO Interrupt Polarity selection
 *
 *  @{
 */
#define PMIC_GPIO_POL_LOW              (0U)
#define PMIC_GPIO_POL_HIGH             (1U)
/*  @} */

/*==========================================================================*/
/*                         Structures and Enums                             */
/*==========================================================================*/

/*!
 * \brief  PMIC GPIO/NPWRON/ENABLE pin configuration structure.
 *         Note: validParams is input param for all Set and Get APIs. other
 *         params except validParams is input param for Set APIs and output
 *         param for Get APIs
 *
 * \param   validParams         Selection of structure parameters to be set,
 *                              from the combination of \ref Pmic_GpioCflag
 *                              and the corresponding member value must be
 *                              updated.
 *                              Valid values \ref Pmic_GpioCflag.
 * \param   pinDir              gpio pin Direction. Valid only for GPIO pins.
 *                              Valid values \ref Pmic_Gpio_SignalDir.
 *                                Valid only when PMIC_GPIO_CFG_DIR_VALID
 *                                bit is set.
 * \param   outputSignalType    output signal type.
 *                              Valid values \ref Pmic_Gpio_SignalType.
 *                                Valid only when PMIC_GPIO_CFG_OD_VALID
 *                                bit is set.
 *                                Valid only for GPIO Pins.
 * \param   pullCtrl            pullup/pull down control.
 *                              Valid values \ref Pmic_GpioPinPullCtrl.
 *                                Valid only when PMIC_GPIO_CFG_PULL_VALID
 *                                bit is set.
 * \param   deglitchEnable      signal deglitch time enable/disable.
 *                              Valid values \ref Pmic_Gpio_DeglitchTimeCfg.
 *                                Valid only when PMIC_GPIO_CFG_DEGLITCH_VALID
 *                                bit is set.
 * \param   pinFunc             pin mux function.
 *                              Valid values for TPS6594x Leo Device
 *                              \ref Pmic_Tps6594xLeo_GpioPinFunc.
 *                              Valid values for LP8764x HERA Device
 *                              \ref Pmic_Lp8764xHera_GpioPinFunc.
 *                                Valid only when PMIC_GPIO_CFG_PINFUNC_VALID
 *                                bit is set.
 * \param   pinPolarity         Configure pin polarity.
 *                              Valid only for Enable pin.
 *                                Valid only when PMIC_ENABLE_CFG_POLARITY_VALID
 *                                bit is set.
 */
typedef struct Pmic_GpioCfg_s
{
    uint8_t                   validParams;
    uint8_t                   pinDir;
    uint8_t                   outputSignalType;
    uint8_t                   pullCtrl;
    uint8_t                   deglitchEnable;
    uint8_t                   pinFunc;
    uint8_t                   pinPolarity;
} Pmic_GpioCfg_t;

/*==========================================================================*/
/*                         Function Declarations                            */
/*==========================================================================*/
/*!
 * \brief   API to set PMIC GPIO configuration.
 *
 * Requirement: REQ_TAG(PDK-5808), REQ_TAG(PDK-5844), REQ_TAG(PDK-9111),
 *              REQ_TAG(PDK-9157)
 * Design: did_pmic_gpio_cfg_readback, did_pmic_lpstandby_wkup_cfg
 * Architecture: aid_pmic_gpio_cfg
 *
 *          This function is used to set the required configuration for the
 *          specified GPIO pin when corresponding validParam bit field is set in
 *          the Pmic_GpioCfg_t
 *          For more information \ref Pmic_GpioCfg_t
 *          Note: Application has to ensure to do proper configuration of GPIO
 *                pin when connected to Enable pin of other peripherals on the
 *                board. If not configured properly then it may down the
 *                peripheral or system
 *
 * \param   pPmicCoreHandle [IN]    PMIC Interface Handle.
 * \param   pin             [IN]    PMIC GPIO pin number.
 *                                   Valid values for TPS6594x Leo Device
 *                                   \ref Pmic_Tps6594xLeo_GpioPin.
 *                                   Valid values for LP8764x HERA Device
 *                                   \ref Pmic_Lp8764xHera_GpioPin.
 * \param   gpioCfg         [IN]    Set required configuration for
 *                                  the specified GPIO pin.
 *
 * \return  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_gpioSetConfiguration(Pmic_CoreHandle_t   *pPmicCoreHandle,
                                  const uint8_t        pin,
                                  const Pmic_GpioCfg_t gpioCfg);

/*!
 * \brief   API to get PMIC GPIO configuration.
 *
 * Requirement: REQ_TAG(PDK-5808)
 * Design: did_pmic_gpio_cfg_readback
 * Architecture: aid_pmic_gpio_cfg
 *
 *          This function is used to read the configuration for the specified
 *          GPIO pin when corresponding validParam bit field is set in
 *          the Pmic_GpioCfg_t
 *          For more information \ref Pmic_GpioCfg_t
 *
 * \param   pPmicCoreHandle [IN]     PMIC Interface Handle
 * \param   pin             [IN]     PMIC GPIO pin number.
 *                                    Valid values for TPS6594x Leo Device
 *                                    \ref Pmic_Tps6594xLeo_GpioPin.
 *                                    Valid values for LP8764x HERA Device
 *                                    \ref Pmic_Lp8764xHera_GpioPin.
 * \param   pGpioCfg        [IN/OUT] Pointer to store specified GPIO pin
 *                                   configuration
 *
 * \return  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_gpioGetConfiguration(Pmic_CoreHandle_t *pPmicCoreHandle,
                                  const uint8_t      pin,
                                  Pmic_GpioCfg_t    *pGpioCfg);

/*!
 * \brief   API to set PMIC GPIO value.
 *
 * Requirement: REQ_TAG(PDK-5808), REQ_TAG(PDK-9111)
 * Design: did_pmic_gpio_cfg_readback
 * Architecture: aid_pmic_gpio_cfg
 *
 *          This function is used to configure the signal level of the
 *          specified GPIO pin.
 *
 * \param   pPmicCoreHandle [IN]    PMIC Interface Handle.
 * \param   pin             [IN]    PMIC GPIO pin number.
 *                                   Valid values for TPS6594x Leo Device
 *                                   \ref Pmic_Tps6594xLeo_GpioPin.
 *                                   Valid values for LP8764x HERA Device
 *                                   \ref Pmic_Lp8764xHera_GpioPin.
 * \param   pinValue        [IN]    PMIC GPIO signal level High/Low to be
 *                                  configured.
 *                                  Valid values \ref Pmic_Gpio_SignalLvl.
 *
 * \return  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_gpioSetValue(Pmic_CoreHandle_t *pPmicCoreHandle,
                          const uint8_t      pin,
                          const uint8_t      pinValue);

/*!
 * \brief   API to get PMIC GPIO value.
 *
 * Requirement: REQ_TAG(PDK-5808)
 * Design: did_pmic_gpio_cfg_readback
 * Architecture: aid_pmic_gpio_cfg
 *
 *          This function is used to read the signal level of the gpio pin
 *
 * \param   pPmicCoreHandle [IN]    PMIC Interface Handle
 * \param   pin             [IN]    PMIC GPIO pin number.
 *                                   Valid values for TPS6594x Leo Device
 *                                   \ref Pmic_Tps6594xLeo_GpioPin.
 *                                   Valid values for LP8764x HERA Device
 *                                   \ref Pmic_Lp8764xHera_GpioPin.
 * \param   pPinValue       [OUT]   To store PMIC GPIO signal level High/Low.
 *                                  Valid values \ref Pmic_Gpio_SignalLvl
 *
 * \return  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_gpioGetValue(Pmic_CoreHandle_t *pPmicCoreHandle,
                          const uint8_t      pin,
                          uint8_t           *pPinValue);

/*!
 * \brief   API to enable/disable GPIO interrupt.
 *
 * Requirement: REQ_TAG(PDK-5808), REQ_TAG(PDK-9159), REQ_TAG(PDK-9329)
 * Design: did_pmic_gpio_cfg_readback
 * Architecture: aid_pmic_gpio_cfg
 *
 *          This function is used to enable/disable GPIO pin Interrupts
 *
 * \param   pPmicCoreHandle [IN]    PMIC Interface Handle.
 * \param   pin             [IN]    PMIC GPIO number.
 *                                   Valid values for TPS6594x Leo Device
 *                                   \ref Pmic_Tps6594xLeo_GpioPin.
 *                                   Valid values for LP8764x HERA Device
 *                                   \ref Pmic_Lp8764xHera_GpioPin.
 * \param   intrType        [IN]    Interrupt type \ref Pmic_GpioInterruptCfg
 * \param   maskPol         [IN]    FSM trigger masking polarity select for GPIO
 *                                  Valid values refer
 *                                  \ref Pmic_GpioInterruptPolCfg.
 *
 * \return  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_gpioSetIntr(Pmic_CoreHandle_t *pPmicCoreHandle,
                         const uint8_t      pin,
                         const uint8_t      intrType,
                         const uint8_t      maskPol);

/*!
 * \brief   API to set configuration for NPWRON/Enable pin.
 *
 * Requirement: REQ_TAG(PDK-5808), REQ_TAG(PDK-9111), REQ_TAG(PDK-9162)
 * Design: did_pmic_gpio_cfg_readback
 * Architecture: aid_pmic_gpio_cfg
 *
 *          This function is used to set the required configuration for the
 *          NPWRON OR ENABLE pin when corresponding validParam bit field is set
 *          in the Pmic_GpioCfg_t
 *          For more information \ref Pmic_GpioCfg_t
 *          NPWRON is valid only for TPS6594x Leo Device.
 *
 *          Note: In this API, the default PMIC device is assumed as TPS6594x
 *                LEO PMIC. While adding support for New PMIC device, developer
 *                need to update the API functionality for New PMIC device
 *                accordingly.
 *
 * \param   pPmicCoreHandle [IN]    PMIC Interface Handle
 * \param   gpioCfg         [IN]    Set NPWRON or ENABLE GPIO pin
 *                                  configuration
 *
 * \return  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_gpioSetNPwronEnablePinConfiguration(
                                        Pmic_CoreHandle_t   *pPmicCoreHandle,
                                        const Pmic_GpioCfg_t gpioCfg);

/*!
 * \brief   API to get configuration for NPWRON/Enable pin.
 *
 * Requirement: REQ_TAG(PDK-5808)
 * Design: did_pmic_gpio_cfg_readback
 * Architecture: aid_pmic_gpio_cfg
 *
 *          This function is used to read the configuration for the
 *          NPWRON OR ENABLE pin when corresponding validParam bit field is set
 *          in the Pmic_GpioCfg_t
 *          For more information \ref Pmic_GpioCfg_t
 *          NPWRON is valid only for TPS6594x Leo Device.
 *
 *          Note: In this API, the default PMIC device is assumed as TPS6594x
 *                LEO PMIC. While adding support for New PMIC device, developer
 *                need to update the API functionality for New PMIC device
 *                accordingly.
 *
 * \param   pPmicCoreHandle [IN]       PMIC Interface Handle
 * \param   pGpioCfg        [IN/OUT]   Pointer to store NPWRON OR ENABLE GPIO
 *                                     pin configuration
 *
 * \return  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_gpioGetNPwronEnablePinConfiguration(
                                            Pmic_CoreHandle_t *pPmicCoreHandle,
                                            Pmic_GpioCfg_t    *pGpioCfg);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* PMIC_GPIO_H_ */

/* @} */
