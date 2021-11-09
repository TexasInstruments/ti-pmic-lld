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
*   \file    pmic_gpio.c
*
*   \brief   This file contains the default API's for PMIC gpio
*            configuration
*
*/

#include <pmic_types.h>
#include <pmic_gpio.h>
#include <pmic_irq.h>

#include <pmic_core_priv.h>
#include <pmic_io_priv.h>
#include <pmic_gpio_tps6594x_priv.h>
#include <pmic_gpio_lp8764x_priv.h>

/*!
 * \brief   This function is used to get the PMIC GPIO configuration.
 *
 *          Note: In this API, the default PMIC device is assumed as TPS6594x
 *                LEO PMIC. While adding support for New PMIC device, developer
 *                need to update the API functionality for New PMIC device
 *                accordingly.
 */
static void Pmic_get_gpioInOutCfg(const Pmic_CoreHandle_t *pPmicCoreHandle,
                                  Pmic_GpioInOutCfg_t    **pGpioInOutCfg)
{
    switch(pPmicCoreHandle->pmicDeviceType)
    {
        case PMIC_DEV_HERA_LP8764X:
            pmic_get_lp8764x_gpioInOutCfg(pGpioInOutCfg);
            break;
        default:
            /* Default case is valid only for TPS6594x LEO PMIC */
            pmic_get_tps6594x_gpioInOutCfg(pGpioInOutCfg);
            break;
    }
}

/*!
 * \brief   This function is used to get the PMIC GPIO configuration.
 *
 *          Note: In this API, the default PMIC device is assumed as TPS6594x
 *                LEO PMIC. While adding support for New PMIC device, developer
 *                need to update the API functionality for New PMIC device
 *                accordingly.
 */
static void Pmic_get_gpioIntRegCfg(const Pmic_CoreHandle_t *pPmicCoreHandle,
                                   Pmic_GpioIntRegCfg_t   **pGpioIntRegCfg)
{
    switch(pPmicCoreHandle->pmicDeviceType)
    {
        case PMIC_DEV_HERA_LP8764X:
            pmic_get_lp8764x_gpioIntRegCfg(pGpioIntRegCfg);
            break;
        default:
            /* Default case is valid only for TPS6594x LEO PMIC */
            pmic_get_tps6594x_gpioIntRegCfg(pGpioIntRegCfg);
            break;
    }
}

/*!
 * \brief   This function is used to validate the PMIC specific pin is valid or
 *          not.
 *
 *          Note: In this API, the default PMIC device is assumed as TPS6594x
 *                LEO PMIC. While adding support for New PMIC device, developer
 *                need to update the API functionality for New PMIC device
 *                accordingly.
 */
static int32_t Pmic_gpioValidatePin(const uint8_t pmicDeviceType,
                                    const uint8_t pin)
{
    int32_t status = PMIC_ST_SUCCESS;

    switch(pmicDeviceType)
    {
        case PMIC_DEV_HERA_LP8764X:
            if((pin < PMIC_LP8764X_GPIO_PIN_MIN) ||
               (pin > PMIC_LP8764X_GPIO_PIN_MAX))
            {
                status = PMIC_ST_ERR_INV_PARAM;
            }
            break;
        default:
            /* Default case is valid only for TPS6594x LEO PMIC */
            if((pin < PMIC_TPS6594X_GPIO_PIN_MIN) ||
               (pin > PMIC_TPS6594X_GPIO_PIN_MAX))
            {
                status = PMIC_ST_ERR_INV_PARAM;
            }
            break;
    }

    return status;
}

/*!
 * \brief   This function is used to validate whether GPIO subsystem is
 *          valid for the programmed pmic device and does the NULL param
 *          check for pPmicCoreHandle
 */

static int32_t Pmic_gpioValidateParams(const Pmic_CoreHandle_t *pPmicCoreHandle)
{
    int32_t status = PMIC_ST_ERR_INV_HANDLE;

    if(NULL != pPmicCoreHandle)
    {
        if ((bool)true == pPmicCoreHandle->pPmic_SubSysInfo->gpioEnable)
        {
            status = PMIC_ST_SUCCESS;
        }
        else
        {
            status = PMIC_ST_ERR_INV_DEVICE;
        }
    }

    return status;
}

/*!
 * \brief   This function is used to validate params and the given gpio pin
 *          is valid for the specific pmic device.
 */
static int32_t Pmic_gpioParamCheck(const Pmic_CoreHandle_t *pPmicCoreHandle,
                                   const uint8_t            pin)
{
    int32_t status = PMIC_ST_SUCCESS;

    status = Pmic_gpioValidateParams(pPmicCoreHandle);
    if(PMIC_ST_SUCCESS == status)
    {
        status = Pmic_gpioValidatePin(pPmicCoreHandle->pmicDeviceType, pin);
    }

    return status;
}

/*!
 * \brief   This function is used to validate the parameters for NPWRON or
 *          ENABLE pin
 */
static int32_t Pmic_gpioNpoweronEnableParamCheck(
                                      const Pmic_CoreHandle_t *pPmicCoreHandle,
                                      const Pmic_GpioCfg_t    *pGpioCfg)
{
    int32_t status = PMIC_ST_SUCCESS;

    status = Pmic_gpioValidateParams(pPmicCoreHandle);
    if((PMIC_ST_SUCCESS == status) && (NULL == pGpioCfg))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    return status;
}

/*!
 * \brief   This function is used to select the device specific register for
 *          GPIO pins.
 *
 *          Note: In this API, the default PMIC device is assumed as TPS6594x
 *                LEO PMIC. While adding support for New PMIC device, developer
 *                need to update the API functionality for New PMIC device
 *                accordingly.
 */
static void Pmic_gpioSelectRegister(uint8_t        pmicDeviceType,
                                    const uint8_t  pin,
                                    const uint8_t  inOutCfgRegAddr,
                                    uint8_t       *pRegAddr)
{
    switch(pmicDeviceType)
    {
        case PMIC_DEV_HERA_LP8764X:
            *pRegAddr = inOutCfgRegAddr;
            break;
        default:
            /* Default case is valid only for TPS6594x LEO PMIC */
            if(PMIC_NPWRON_ENABLE_PIN == pin)
            {
                *pRegAddr = PMIC_NPWRON_CONF_REGADDR;
            }
            else
            {
                *pRegAddr = inOutCfgRegAddr;
            }
            break;
    }
}

/*!
 * \brief   This function is used to set the GPIO and NPWRON Pin Functionality
 */
int32_t Pmic_gpioSetPinFunc(Pmic_CoreHandle_t   *pPmicCoreHandle,
                            const uint8_t        pin,
                            const Pmic_GpioCfg_t gpioCfg)
{
    int32_t status  = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    uint8_t regAddr = 0U;
    uint8_t index   = 0U;
    Pmic_GpioInOutCfg_t *pGpioInOutCfg = NULL;

    if(PMIC_NPWRON_ENABLE_PIN == pin)
    {
        if(gpioCfg.pinFunc > PMIC_TPS6594X_NPWRON_PINFUNC_NONE)
        {
            status = PMIC_ST_ERR_INV_GPIO_FUNC;
        }
    }
    else
    {
        if(gpioCfg.pinFunc > PMIC_GPIO_PINFUNC_MAX)
        {
            status = PMIC_ST_ERR_INV_GPIO_FUNC;
        }
    }

    if(PMIC_ST_SUCCESS == status)
    {
        /* Set Pmic_GpioIntRegCfg_t array index for given GPIO Pin */
        index = pin - 1U;

        /* Get PMIC gpio configuration */
        Pmic_get_gpioInOutCfg(pPmicCoreHandle, &pGpioInOutCfg);

        Pmic_gpioSelectRegister(pPmicCoreHandle->pmicDeviceType,
                                pin,
                                (uint8_t)pGpioInOutCfg[index].regAddr,
                                &regAddr);

        /* Start Critical Section */
        Pmic_criticalSectionStart(pPmicCoreHandle);

        /* Reading GPIO conf register */
        status = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        regAddr,
                                        &regData);

        if (PMIC_ST_SUCCESS == status)
        {
            if(PMIC_NPWRON_ENABLE_PIN == pin)
            {
                /* For nPWRON pin function */
                Pmic_setBitField(&regData,
                                 PMIC_NPWRON_CONF_NPWRON_SEL_SHIFT,
                                 PMIC_NPWRON_CONF_NPWRON_SEL_MASK,
                                 gpioCfg.pinFunc);
            }
            else
            {
                /* For GPIO pin function */
                Pmic_setBitField(&regData,
                                 PMIC_GPIOX_CONF_GPIO_SEL_SHIFT,
                                 PMIC_GPIOX_CONF_GPIO_SEL_MASK,
                                 gpioCfg.pinFunc);
            }
            /* Setting GPIO pin function */
            status = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                            regAddr,
                                            regData);
        }

        /* Stop Critical Section Handle */
        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    return status;
}

/*!
 * \brief   This function is used to get the GPIO and NPWRON Pin Functionality
 */
int32_t Pmic_gpioGetPinFunc(Pmic_CoreHandle_t *pPmicCoreHandle,
                            const uint8_t      pin,
                            Pmic_GpioCfg_t    *pGpioCfg)
{
    int32_t status  = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    uint8_t regAddr = 0U;
    uint8_t index   = 0U;
    Pmic_GpioInOutCfg_t *pGpioInOutCfg = NULL;

    /* Set Pmic_GpioIntRegCfg_t array index for given GPIO Pin */
    index = pin - 1U;

    /* Get PMIC gpio configuration */
    Pmic_get_gpioInOutCfg(pPmicCoreHandle, &pGpioInOutCfg);

    Pmic_gpioSelectRegister(pPmicCoreHandle->pmicDeviceType,
                            pin,
                            (uint8_t)pGpioInOutCfg[index].regAddr,
                            &regAddr);

    /* Start Critical Section */
    Pmic_criticalSectionStart(pPmicCoreHandle);

    /* Reading the GPIO configuration */
    status = Pmic_commIntf_recvByte(pPmicCoreHandle, regAddr, &regData);

    /* Stop Critical Section */
    Pmic_criticalSectionStop(pPmicCoreHandle);

    if(PMIC_ST_SUCCESS == status)
    {
        if(PMIC_NPWRON_ENABLE_PIN == pin)
        {
            /* For nPWRON pin function */
            pGpioCfg->pinFunc =
                            Pmic_getBitField(regData,
                                             PMIC_NPWRON_CONF_NPWRON_SEL_SHIFT,
                                             PMIC_NPWRON_CONF_NPWRON_SEL_MASK);
        }
        else
        {
            /* For GPIO pin function */
            pGpioCfg->pinFunc =
                            Pmic_getBitField(regData,
                                             PMIC_GPIOX_CONF_GPIO_SEL_SHIFT,
                                             PMIC_GPIOX_CONF_GPIO_SEL_MASK);
        }
    }

    return status;
}

/*!
 * \brief   This function is used to set the NPWRON Pin Polarity.
 *
 *          Note: In this API, the default PMIC device is assumed as TPS6594x
 *                LEO PMIC. While adding support for New PMIC device, developer
 *                need to update the API functionality for New PMIC device
 *                accordingly.
 */
int32_t Pmic_gpioSetPinPolarity(Pmic_CoreHandle_t    *pPmicCoreHandle,
                                const Pmic_GpioCfg_t  gpioCfg)
{
    int32_t status  = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    uint8_t regAddr = 0U;
    uint8_t bitPos  = 0U;
    uint8_t bitMask = 0U;

    switch(pPmicCoreHandle->pmicDeviceType)
    {
        case PMIC_DEV_HERA_LP8764X:
            regAddr = PMIC_ENABLE_CONF_REGADDR;
            bitPos = PMIC_ENABLE_CONF_ENABLE_POL_SHIFT;
            bitMask = PMIC_ENABLE_CONF_ENABLE_POL_MASK;
            break;
        default:
            /* Default case is valid only for TPS6594x LEO PMIC */
            regAddr = PMIC_NPWRON_CONF_REGADDR;
            bitPos = PMIC_NPWRON_CONF_ENABLE_POL_SHIFT;
            bitMask = PMIC_NPWRON_CONF_ENABLE_POL_MASK;
            break;
    }

    /* Start Critical Section */
    Pmic_criticalSectionStart(pPmicCoreHandle);

    /* Reading NPWRON/ENABLE conf register */
    status = Pmic_commIntf_recvByte(pPmicCoreHandle, regAddr, &regData);

    if (PMIC_ST_SUCCESS == status)
    {
        /* Setting ENABLE pin polarity */
        Pmic_setBitField(&regData,
                         bitPos,
                         bitMask,
                         gpioCfg.pinPolarity);

        status = Pmic_commIntf_sendByte(pPmicCoreHandle, regAddr, regData);
    }

    /* Stop Critical Section Handle */
    Pmic_criticalSectionStop(pPmicCoreHandle);

    return status;
}

/*!
 * \brief   This function is used to get the NPWRON Pin Polarity.
 *
 *          Note: In this API, the default PMIC device is assumed as TPS6594x
 *                LEO PMIC. While adding support for New PMIC device, developer
 *                need to update the API functionality for New PMIC device
 *                accordingly.
 */
int32_t Pmic_gpioGetPinPolarity(Pmic_CoreHandle_t *pPmicCoreHandle,
                                Pmic_GpioCfg_t    *pGpioCfg)
{
    int32_t status  = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    uint8_t bitPos  = 0U;
    uint8_t regAddr = 0U;
    uint8_t bitMask = 0U;

    switch(pPmicCoreHandle->pmicDeviceType)
    {
        case PMIC_DEV_HERA_LP8764X:
            regAddr = PMIC_ENABLE_CONF_REGADDR;
            bitPos = PMIC_ENABLE_CONF_ENABLE_POL_SHIFT;
            bitMask = PMIC_ENABLE_CONF_ENABLE_POL_MASK;
            break;
        default:
            /* Default case is valid only for TPS6594x LEO PMIC */
            regAddr = PMIC_NPWRON_CONF_REGADDR;
            bitPos = PMIC_NPWRON_CONF_ENABLE_POL_SHIFT;
            bitMask = PMIC_NPWRON_CONF_ENABLE_POL_MASK;
            break;
    }

    /* Start Critical Section */
    Pmic_criticalSectionStart(pPmicCoreHandle);

    /* Reading the NPWRON/ENABLE configuration */
    status = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                    regAddr,
                                    &regData);

    /* Stop Critical Section */
    Pmic_criticalSectionStop(pPmicCoreHandle);

    if(PMIC_ST_SUCCESS == status)
    {
        /* Reading Enable pin polarity */
        pGpioCfg->pinPolarity = Pmic_getBitField(regData, bitPos, bitMask);
    }

    return status;
}

/*!
 * \brief   This function is used to set the GPIO and NPWRON Pin Pull Control
 */
static int32_t Pmic_gpioSetPullCtrl(Pmic_CoreHandle_t    *pPmicCoreHandle,
                                    const uint8_t         pin,
                                    const Pmic_GpioCfg_t  gpioCfg)
{
    int32_t status  = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    uint8_t index   = 0;
    uint8_t regAddr = 0U;
    Pmic_GpioInOutCfg_t *pGpioInOutCfg = NULL;

    if(gpioCfg.pullCtrl > PMIC_GPIO_PULL_UP)
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    if(PMIC_ST_SUCCESS == status)
    {
        /* Set Pmic_GpioIntRegCfg_t array index for given GPIO Pin */
        index = pin - 1U;

        /* Get PMIC gpio configuration */
        Pmic_get_gpioInOutCfg(pPmicCoreHandle, &pGpioInOutCfg);

        Pmic_gpioSelectRegister(pPmicCoreHandle->pmicDeviceType,
                                pin,
                                (uint8_t)pGpioInOutCfg[index].regAddr,
                                &regAddr);

        /* Start Critical Section */
        Pmic_criticalSectionStart(pPmicCoreHandle);

        /* Reading GPIO/NPWRON conf register */
        status = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        regAddr,
                                        &regData);

        if (PMIC_ST_SUCCESS == status)
        {
            if(PMIC_GPIO_PULL_DISABLED == gpioCfg.pullCtrl)
            {
                /* Disable pull-up/pull-down feature */
                Pmic_setBitField(&regData,
                                 PMIC_GPIOX_CONF_GPIO_PU_PD_EN_SHIFT,
                                 PMIC_GPIOX_CONF_GPIO_PU_PD_EN_MASK,
                                 PMIC_GPIO_PU_PD_DISABLE);
            }

            if((PMIC_GPIO_PULL_UP == gpioCfg.pullCtrl) ||
               (PMIC_GPIO_PULL_DOWN == gpioCfg.pullCtrl))
            {
                /* Enable pull-up/pull-down feature */
                Pmic_setBitField(&regData,
                                 PMIC_GPIOX_CONF_GPIO_PU_PD_EN_SHIFT,
                                 PMIC_GPIOX_CONF_GPIO_PU_PD_EN_MASK,
                                 PMIC_GPIO_PU_PD_ENABLE);

                if(PMIC_GPIO_PULL_UP == gpioCfg.pullCtrl)
                {
                    /* select pull-up resistor */
                    Pmic_setBitField(&regData,
                                     PMIC_GPIOX_CONF_GPIO_PU_SEL_SHIFT,
                                     PMIC_GPIOX_CONF_GPIO_PU_SEL_MASK,
                                     PMIC_GPIO_PU_SELECT);
                }
                else
                {
                    /* Select pull-down resistor */
                    Pmic_setBitField(&regData,
                                     PMIC_GPIOX_CONF_GPIO_PU_SEL_SHIFT,
                                     PMIC_GPIOX_CONF_GPIO_PU_SEL_MASK,
                                     PMIC_GPIO_PD_SELECT);
                }
            }
            status = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                            regAddr,
                                            regData);
        }

        /* Stop Critical Section */
        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    return status;
}

/*!
 * \brief   This function is used to get the GPIO and NPWRON Pin Pull Control
 */
int32_t Pmic_gpioGetPullCtrl(Pmic_CoreHandle_t *pPmicCoreHandle,
                             const uint8_t      pin,
                             Pmic_GpioCfg_t    *pGpioCfg)
{
    int32_t status  = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    uint8_t regAddr = 0U;
    Pmic_GpioInOutCfg_t *pGpioInOutCfg = NULL;
    uint8_t index = 0;

    /* Set Pmic_GpioIntRegCfg_t array index for given GPIO Pin */
    index = pin - 1U;

    /* Get PMIC gpio configuration */
    Pmic_get_gpioInOutCfg(pPmicCoreHandle, &pGpioInOutCfg);

    Pmic_gpioSelectRegister(pPmicCoreHandle->pmicDeviceType,
                            pin,
                            (uint8_t)pGpioInOutCfg[index].regAddr,
                            &regAddr);

    /* Start Critical Section */
    Pmic_criticalSectionStart(pPmicCoreHandle);

    /* Reading the GPIO configuration */
    status = Pmic_commIntf_recvByte(pPmicCoreHandle, regAddr, &regData);

    /* Stop Critical Section */
    Pmic_criticalSectionStop(pPmicCoreHandle);

    if(PMIC_ST_SUCCESS == status)
    {
        /* Reading gpio pull control */
        if(Pmic_getBitField(regData,
                            PMIC_GPIOX_CONF_GPIO_PU_PD_EN_SHIFT,
                            PMIC_GPIOX_CONF_GPIO_PU_PD_EN_MASK) == 0U)
        {
            pGpioCfg->pullCtrl = PMIC_GPIO_PULL_DISABLED;
        }
        else if(Pmic_getBitField(regData,
                                 PMIC_GPIOX_CONF_GPIO_PU_SEL_SHIFT,
                                 PMIC_GPIOX_CONF_GPIO_PU_SEL_MASK) != 0U)
        {
            pGpioCfg->pullCtrl = PMIC_GPIO_PULL_UP;
        }
        else
        {
            pGpioCfg->pullCtrl = PMIC_GPIO_PULL_DOWN;
        }
    }

    return status;
}

/*!
 * \brief   This function is used to set the GPIO Pin Direction
 */
static int32_t Pmic_gpioSetPinDir(Pmic_CoreHandle_t   *pPmicCoreHandle,
                                  const uint8_t        pin,
                                  const Pmic_GpioCfg_t gpioCfg)
{
    int32_t status  = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    uint8_t index   = 0U;
    Pmic_GpioInOutCfg_t *pGpioInOutCfg = NULL;

    if(gpioCfg.pinDir > PMIC_GPIO_OUTPUT)
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    if(PMIC_ST_SUCCESS == status)
    {
        /* Set Pmic_GpioIntRegCfg_t array index for given GPIO Pin */
        index = pin - 1U;

        /* Get PMIC gpio configuration */
        Pmic_get_gpioInOutCfg(pPmicCoreHandle, &pGpioInOutCfg);

        /* Start Critical Section */
        Pmic_criticalSectionStart(pPmicCoreHandle);

        /* Reading GPIO/NPWRON conf register */
        status = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        pGpioInOutCfg[index].regAddr,
                                        &regData);

        if (PMIC_ST_SUCCESS == status)
        {
            /* set gpio pin direction */
            Pmic_setBitField(&regData,
                             PMIC_GPIOX_CONF_GPIO_DIR_SHIFT,
                             PMIC_GPIOX_CONF_GPIO_DIR_MASK,
                             gpioCfg.pinDir);

            status = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                            pGpioInOutCfg[index].regAddr,
                                            regData);
        }

        /* Stop Critical Section */
        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    return status;
}

/*!
 * \brief   This function is used to get the GPIO Pin Direction
 */
static int32_t Pmic_gpioGetPinDir(Pmic_CoreHandle_t *pPmicCoreHandle,
                                  const uint8_t      pin,
                                  Pmic_GpioCfg_t    *pGpioCfg)
{
    int32_t status                     = PMIC_ST_SUCCESS;
    uint8_t regData                    = 0U;
    Pmic_GpioInOutCfg_t *pGpioInOutCfg = NULL;
    uint8_t index = 0U;

    /* Set Pmic_GpioIntRegCfg_t array index for given GPIO Pin */
    index = pin - 1U;

    /* Get PMIC gpio configuration */
    Pmic_get_gpioInOutCfg(pPmicCoreHandle, &pGpioInOutCfg);

    /* Start Critical Section */
    Pmic_criticalSectionStart(pPmicCoreHandle);

    /* Reading the GPIO configuration */
    status = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                    pGpioInOutCfg[index].regAddr,
                                    &regData);

    /* Stop Critical Section */
    Pmic_criticalSectionStop(pPmicCoreHandle);

    if(PMIC_ST_SUCCESS == status)
    {
        /* Reading gpio direction */
        pGpioCfg->pinDir = Pmic_getBitField(regData,
                                            PMIC_GPIOX_CONF_GPIO_DIR_SHIFT,
                                            PMIC_GPIOX_CONF_GPIO_DIR_MASK);
    }

    return status;
}

/*!
 * \brief   This function is used to set the GPIO and NPWRON Deglitch Time
 */
static int32_t Pmic_gpioSetDeglitchTime(Pmic_CoreHandle_t   *pPmicCoreHandle,
                                        const uint8_t        pin,
                                        const Pmic_GpioCfg_t gpioCfg)
{
    int32_t status  = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    uint8_t index   = 0U;
    uint8_t regAddr = 0U;
    Pmic_GpioInOutCfg_t *pGpioInOutCfg = NULL;

    if(gpioCfg.deglitchEnable > PMIC_GPIO_DEGLITCH_ENABLE)
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    if(PMIC_ST_SUCCESS == status)
    {
        /* Set Pmic_GpioIntRegCfg_t array index for given GPIO Pin */
        index = pin - 1U;

        /* Get PMIC gpio configuration */
        Pmic_get_gpioInOutCfg(pPmicCoreHandle, &pGpioInOutCfg);

        Pmic_gpioSelectRegister(pPmicCoreHandle->pmicDeviceType,
                                pin,
                                (uint8_t)pGpioInOutCfg[index].regAddr,
                                &regAddr);

        /* Start Critical Section */
        Pmic_criticalSectionStart(pPmicCoreHandle);

        /* Reading GPIO/NPWRON conf register */
        status = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        regAddr,
                                        &regData);

        if (PMIC_ST_SUCCESS == status)
        {
            /* setting deglitch time */
            Pmic_setBitField(&regData,
                             PMIC_GPIOX_CONF_GPIO_DEGLITCH_EN_SHIFT,
                             PMIC_GPIOX_CONF_GPIO_DEGLITCH_EN_MASK,
                             gpioCfg.deglitchEnable);

            status = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                            regAddr,
                                            regData);
        }

        /* Stop Critical Section */
        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    return status;
}

/*!
 * \brief   This function is used to get the GPIO and NPWRON Deglitch time
 */
int32_t Pmic_gpioGetDeglitchTime(Pmic_CoreHandle_t *pPmicCoreHandle,
                                 const uint8_t      pin,
                                 Pmic_GpioCfg_t    *pGpioCfg)
{
    int32_t status  = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    uint8_t regAddr = 0U;
    uint8_t index   = 0U;
    Pmic_GpioInOutCfg_t *pGpioInOutCfg = NULL;

    /* Set Pmic_GpioIntRegCfg_t array index for given GPIO Pin */
    index = pin - 1U;

    /* Get PMIC gpio configuration */
    Pmic_get_gpioInOutCfg(pPmicCoreHandle, &pGpioInOutCfg);

    Pmic_gpioSelectRegister(pPmicCoreHandle->pmicDeviceType,
                            pin,
                            (uint8_t)pGpioInOutCfg[index].regAddr,
                            &regAddr);

    /* Start Critical Section */
    Pmic_criticalSectionStart(pPmicCoreHandle);

    /* Reading the GPIO configuration */
    status = Pmic_commIntf_recvByte(pPmicCoreHandle, regAddr, &regData);

    /* Stop Critical Section */
    Pmic_criticalSectionStop(pPmicCoreHandle);

    if(PMIC_ST_SUCCESS == status)
    {
        /* Reading signal deglitch time */
        pGpioCfg->deglitchEnable = Pmic_getBitField(
                                        regData,
                                        PMIC_GPIOX_CONF_GPIO_DEGLITCH_EN_SHIFT,
                                        PMIC_GPIOX_CONF_GPIO_DEGLITCH_EN_MASK);
    }

    return status;
}

/*!
 * \brief   This function is used to set the GPIO Output Signal Type
 */
static int32_t Pmic_gpioSetOutputSignalType(
                                       Pmic_CoreHandle_t    *pPmicCoreHandle,
                                       const uint8_t         pin,
                                       const Pmic_GpioCfg_t  gpioCfg)
{
    int32_t status  = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    uint8_t index   = 0U;
    uint8_t regAddr = 0U;
    Pmic_GpioInOutCfg_t *pGpioInOutCfg = NULL;

    if(gpioCfg.outputSignalType > PMIC_GPIO_OPEN_DRAIN_OUTPUT)
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    if(PMIC_ST_SUCCESS == status)
    {
        /* Set Pmic_GpioIntRegCfg_t array index for given GPIO Pin */
        index = pin - 1U;

        /* Get PMIC gpio configuration */
        Pmic_get_gpioInOutCfg(pPmicCoreHandle, &pGpioInOutCfg);

        Pmic_gpioSelectRegister(pPmicCoreHandle->pmicDeviceType,
                                pin,
                                (uint8_t)pGpioInOutCfg[index].regAddr,
                                &regAddr);

        /* Start Critical Section */
        Pmic_criticalSectionStart(pPmicCoreHandle);

        /* Reading GPIO/NPWRON conf register */
        status = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        regAddr,
                                        &regData);

        if(PMIC_ST_SUCCESS == status)
        {
            /* selecting output type */
            Pmic_setBitField(&regData,
                             PMIC_GPIOX_CONF_GPIO_OD_SHIFT,
                             PMIC_GPIOX_CONF_GPIO_OD_MASK,
                             gpioCfg.outputSignalType);

            status = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                            regAddr,
                                            regData);
        }

        /* Stop Critical Section */
        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    return status;
}

/*!
 * \brief   This function is used to get the GPIO Output Signal Type
 */
static int32_t Pmic_gpioGetOutputSignalType(Pmic_CoreHandle_t *pPmicCoreHandle,
                                            const uint8_t      pin,
                                            Pmic_GpioCfg_t    *pGpioCfg)
{
    int32_t status  = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    uint8_t regAddr = 0U;
    uint8_t index   = 0U;
    Pmic_GpioInOutCfg_t *pGpioInOutCfg = NULL;

    /* Set Pmic_GpioIntRegCfg_t array index for given GPIO Pin */
    index = pin - 1U;

    /* Get PMIC gpio configuration */
    Pmic_get_gpioInOutCfg(pPmicCoreHandle, &pGpioInOutCfg);

    Pmic_gpioSelectRegister(pPmicCoreHandle->pmicDeviceType,
                            pin,
                            (uint8_t)pGpioInOutCfg[index].regAddr,
                            &regAddr);

    /* Start Critical Section */
    Pmic_criticalSectionStart(pPmicCoreHandle);

    /* Reading the GPIO configuration */
    status = Pmic_commIntf_recvByte(pPmicCoreHandle, regAddr, &regData);

    /* Stop Critical Section */
    Pmic_criticalSectionStop(pPmicCoreHandle);

    if(PMIC_ST_SUCCESS == status)
    {
        /* Reading output signal type */
        pGpioCfg->outputSignalType = Pmic_getBitField(
                                            regData,
                                            PMIC_GPIOX_CONF_GPIO_OD_SHIFT,
                                            PMIC_GPIOX_CONF_GPIO_OD_MASK);

    }

    return status;
}

/*!
 * \brief   This function is used to Enable GPIO Interrupt
 */
static int32_t Pmic_gpioIntrEnable(Pmic_CoreHandle_t *pPmicCoreHandle,
                                   const uint8_t      pin,
                                   const uint8_t      intrType,
                                   const uint8_t      maskPol)
{
    int32_t status                       = PMIC_ST_SUCCESS;
    uint8_t regData                      = 0U;
    Pmic_GpioIntRegCfg_t *pGpioIntRegCfg = NULL;
    uint8_t bitMask                      = 0U;

    Pmic_get_gpioIntRegCfg(pPmicCoreHandle, &pGpioIntRegCfg);

    /* Start Critical section */
    Pmic_criticalSectionStart(pPmicCoreHandle);

    status = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                    pGpioIntRegCfg[pin].intRegAddr,
                                    &regData);

    if(PMIC_ST_SUCCESS == status)
    {
        /* Configuring the GPIOx_FSM_MASK and GPIOx_FSM_MASK_POL fields in the
           PMIC_FSM_TRIG_MASK Register */
        bitMask = (PMIC_GPIO_IN_OUT_X_GPIOX_IN_OUT_BITFIELD <<
                   pGpioIntRegCfg[pin].intRegBitPos);
        Pmic_setBitField(&regData,
                         pGpioIntRegCfg[pin].intRegBitPos,
                         bitMask,
                         PMIC_GPIO_INT_ENABLE);
        bitMask = (PMIC_GPIO_IN_OUT_X_GPIOX_IN_OUT_BITFIELD <<
                   pGpioIntRegCfg[pin].intRegPolBitPos);
        Pmic_setBitField(&regData,
                         pGpioIntRegCfg[pin].intRegPolBitPos,
                         bitMask,
                         maskPol);
        status = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                        pGpioIntRegCfg[pin].intRegAddr,
                                        regData);
    }

    /* Stop Critical Section */
    Pmic_criticalSectionStop(pPmicCoreHandle);

    if(PMIC_ST_SUCCESS == status)
    {
        /* Configuring GPIO MASK Registers */
        status = Pmic_irqGpioMaskIntr(pPmicCoreHandle,
                                      pin,
                                      PMIC_IRQ_UNMASK,
                                      intrType);
    }

    return status;
}

/*!
 * \brief   This function is used to Disable GPIO Interrupt
 */
static int32_t Pmic_gpioIntrDisable(Pmic_CoreHandle_t *pPmicCoreHandle,
                                    const uint8_t      pin)
{
    int32_t status                       = PMIC_ST_SUCCESS;
    uint8_t regData                      = 0U;
    uint8_t bitMask                      = 0U;
    Pmic_GpioIntRegCfg_t *pGpioIntRegCfg = NULL;

    Pmic_get_gpioIntRegCfg(pPmicCoreHandle, &pGpioIntRegCfg);

    /* Start Critical Section */
    Pmic_criticalSectionStart(pPmicCoreHandle);

    status = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                    pGpioIntRegCfg[pin].intRegAddr,
                                    &regData);

    if(PMIC_ST_SUCCESS == status)
    {
        /* Masking the GPIOx_FSM_MASK register */
        bitMask = (PMIC_GPIO_IN_OUT_X_GPIOX_IN_OUT_BITFIELD <<
                   pGpioIntRegCfg[pin].intRegBitPos);
        Pmic_setBitField(&regData,
                         pGpioIntRegCfg[pin].intRegBitPos,
                         bitMask,
                         PMIC_GPIO_INT_MASK);
        status = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                        pGpioIntRegCfg[pin].intRegAddr,
                                        regData);
    }

    /* Stop Critical Section */
    Pmic_criticalSectionStop(pPmicCoreHandle);

    if(PMIC_ST_SUCCESS == status)
    {
        /* Masking the GPIO MASK Register */
        status = Pmic_irqGpioMaskIntr(pPmicCoreHandle,
                                      pin,
                                      PMIC_IRQ_MASK,
                                      PMIC_IRQ_GPIO_RISE_FALL_INT_TYPE);
    }

    return status;
}

/*!
 * \brief   This function is used to set the GPIO deglitch time, Output Signal
 *          Type and Pull UP/Down
 */
static int32_t Pmic_gpioSetDeglitchOutsigtypePulCtrlCfg(
                                         Pmic_CoreHandle_t    *pPmicCoreHandle,
                                         const uint8_t         pin,
                                         const Pmic_GpioCfg_t  gpioCfg)
{
    int32_t status = PMIC_ST_SUCCESS;

    if((bool)true == pmic_validParamCheck(gpioCfg.validParams,
                                           PMIC_GPIO_CFG_DEGLITCH_VALID))
    {
        /* setting deglitch time */
        status = Pmic_gpioSetDeglitchTime(pPmicCoreHandle, pin, gpioCfg);
    }

    if((PMIC_ST_SUCCESS == status) &&
       ((bool)true == pmic_validParamCheck(gpioCfg.validParams,
                                           PMIC_GPIO_CFG_OD_VALID)))
    {
        /* setting open drain */
        status = Pmic_gpioSetOutputSignalType(pPmicCoreHandle,
                                              pin,
                                              gpioCfg);
    }

    if((PMIC_ST_SUCCESS == status) &&
       ((bool)true == pmic_validParamCheck(gpioCfg.validParams,
                                              PMIC_GPIO_CFG_PULL_VALID)))
    {
        /* setting Pull UP/Down */
        status = Pmic_gpioSetPullCtrl(pPmicCoreHandle, pin, gpioCfg);
    }

    return status;
}

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
 * \param   gpioCfg         [IN]    set required configuration for
 *                                  the specified GPIO pin.
 *
 * \return  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_gpioSetConfiguration(Pmic_CoreHandle_t   *pPmicCoreHandle,
                                  const uint8_t        pin,
                                  const Pmic_GpioCfg_t gpioCfg)
{
    int32_t status = PMIC_ST_SUCCESS;

    /* Validation of input Parameters */
    status = Pmic_gpioParamCheck(pPmicCoreHandle, pin);

    if((PMIC_ST_SUCCESS == status) &&
       ((bool)true == pmic_validParamCheck(gpioCfg.validParams,
                                           PMIC_GPIO_CFG_PINFUNC_VALID)))
    {
        /* Setting pin function */
        status = Pmic_gpioSetPinFunc(pPmicCoreHandle, pin, gpioCfg);
    }

    if((PMIC_ST_SUCCESS == status) &&
       ((bool)true == pmic_validParamCheck(gpioCfg.validParams,
                                           PMIC_GPIO_CFG_DIR_VALID)))
    {
        /* set gpio pin direction */
        status = Pmic_gpioSetPinDir(pPmicCoreHandle, pin, gpioCfg);
    }

    if(PMIC_ST_SUCCESS == status)
    {
        /* setting deglitch time/ open drain/ Pull UP/Down*/
        status = Pmic_gpioSetDeglitchOutsigtypePulCtrlCfg(pPmicCoreHandle,
                                                          pin,
                                                          gpioCfg);
    }

    return status;
}

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
                                  Pmic_GpioCfg_t    *pGpioCfg)
{
    int32_t status = PMIC_ST_SUCCESS;

    /* Parameter Validation */
    status = Pmic_gpioParamCheck(pPmicCoreHandle, pin);

    if((PMIC_ST_SUCCESS == status) && (NULL == pGpioCfg))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if((PMIC_ST_SUCCESS == status) &&
       ((bool)true == pmic_validParamCheck(pGpioCfg->validParams,
                                           PMIC_GPIO_CFG_DIR_VALID)))
    {
        status = Pmic_gpioGetPinDir(pPmicCoreHandle, pin, pGpioCfg);
    }

    if((PMIC_ST_SUCCESS == status) &&
       ((bool)true == pmic_validParamCheck(pGpioCfg->validParams,
                                           PMIC_GPIO_CFG_OD_VALID)))
    {
        status = Pmic_gpioGetOutputSignalType(pPmicCoreHandle, pin, pGpioCfg);
    }

    if((PMIC_ST_SUCCESS == status) &&
       ((bool)true == pmic_validParamCheck(pGpioCfg->validParams,
                                           PMIC_GPIO_CFG_DEGLITCH_VALID)))
    {
        status = Pmic_gpioGetDeglitchTime(pPmicCoreHandle, pin, pGpioCfg);
    }

    if((PMIC_ST_SUCCESS == status) &&
       ((bool)true == pmic_validParamCheck(pGpioCfg->validParams,
                                           PMIC_GPIO_CFG_PINFUNC_VALID)))
    {
        status = Pmic_gpioGetPinFunc(pPmicCoreHandle, pin, pGpioCfg);
    }

    if((PMIC_ST_SUCCESS == status) &&
       ((bool)true == pmic_validParamCheck(pGpioCfg->validParams,
                                           PMIC_GPIO_CFG_PULL_VALID)))
    {
        status = Pmic_gpioGetPullCtrl(pPmicCoreHandle, pin, pGpioCfg);
    }

    return status;
}

/*!
 * \brief   This function is used to set the PMIC GPIO Pin Value
 */
static int32_t Pmic_gpioSetPinValue(Pmic_CoreHandle_t          *pPmicCoreHandle,
                                    const Pmic_GpioInOutCfg_t  *pGpioInOutCfg,
                                    const uint8_t               pinValue,
                                    uint8_t                     index)
{
    uint8_t regData = 0U;
    int32_t status  = PMIC_ST_SUCCESS;
    uint8_t bitMask = 0U;

    /* Start Critical Section */
    Pmic_criticalSectionStart(pPmicCoreHandle);

    /* checking for the pin direction to be output */
    status = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                    pGpioInOutCfg[index].regAddr,
                                    &regData);

    if(PMIC_ST_SUCCESS == status)
    {
        if(Pmic_getBitField(regData,
                            PMIC_GPIOX_CONF_GPIO_DIR_SHIFT,
                            PMIC_GPIOX_CONF_GPIO_DIR_MASK) ==
                                               PMIC_GPIO_OUTPUT)
        {
            /* Setting the GPIO value */
            status = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                            pGpioInOutCfg[index].outRegAddr,
                                            &regData);
            if(PMIC_ST_SUCCESS == status)
            {
                bitMask = (PMIC_GPIO_IN_OUT_X_GPIOX_IN_OUT_BITFIELD <<
                           pGpioInOutCfg[index].outRegBitPos);
                Pmic_setBitField(&regData,
                                 pGpioInOutCfg[index].outRegBitPos,
                                 bitMask,
                                 pinValue);
                status = Pmic_commIntf_sendByte(
                                           pPmicCoreHandle,
                                           pGpioInOutCfg[index].outRegAddr,
                                           regData);
            }
        }
        else
        {
            /* setValue is valid only for output pins */
            status = PMIC_ST_ERR_INV_PARAM;
        }
    }

    /* Stop Critical Section */
    Pmic_criticalSectionStop(pPmicCoreHandle);

    return status;
}

/*!
 * \brief   API to set PMIC GPIO value.
 *
 * Requirement: REQ_TAG(PDK-5808)
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
                          const uint8_t      pinValue)
{
    int32_t status  = PMIC_ST_SUCCESS;
    uint8_t index   = 0U;
    Pmic_GpioInOutCfg_t *pGpioInOutCfg = NULL;

    status = Pmic_gpioParamCheck(pPmicCoreHandle, pin);

    /* Set Pmic_GpioIntRegCfg_t array index for given GPIO Pin */
    index = pin - 1U;

    if((PMIC_ST_SUCCESS == status) &&
       (pinValue > PMIC_GPIO_HIGH))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    if(PMIC_ST_SUCCESS == status)
    {
        /* Get PMIC gpio configuration */
        Pmic_get_gpioInOutCfg(pPmicCoreHandle, &pGpioInOutCfg);

        /* Set PMIC gpio pin value */
        status = Pmic_gpioSetPinValue(pPmicCoreHandle,
                                      pGpioInOutCfg,
                                      pinValue,
                                      index);
    }

    return status;
}

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
                          uint8_t           *pPinValue)
{
    int32_t status  = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    uint8_t index   = 0U;
    Pmic_GpioInOutCfg_t *pGpioInOutCfg = NULL;
    uint8_t bitMask = 0U;

    /* Parameter Validation */
    status = Pmic_gpioParamCheck(pPmicCoreHandle, pin);

    /* Set Pmic_GpioIntRegCfg_t array index for given GPIO Pin */
    index = pin - 1U;

    if((PMIC_ST_SUCCESS == status) && (NULL == pPinValue))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if(PMIC_ST_SUCCESS == status)
    {
        /* Get PMIC gpio configuration */
        Pmic_get_gpioInOutCfg(pPmicCoreHandle, &pGpioInOutCfg);

        /* Start Critical Section */
        Pmic_criticalSectionStart(pPmicCoreHandle);

        /* Reading the pin value */
        status = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        pGpioInOutCfg[index].inRegAddr,
                                        &regData);
        /* Stop Critical Section */
        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    if(PMIC_ST_SUCCESS == status)
    {
        bitMask = (PMIC_GPIO_IN_OUT_X_GPIOX_IN_OUT_BITFIELD <<
                   pGpioInOutCfg[index].inRegBitPos);

        if(Pmic_getBitField(regData,
                            pGpioInOutCfg[index].inRegBitPos,
                            bitMask) != 0U)
        {
            *pPinValue = PMIC_GPIO_HIGH;
        }
        else
        {
            *pPinValue = PMIC_GPIO_LOW;
        }
    }

    return status;
}

/*!
 * \brief   API to enable/disable GPIO interrupt.
 *
 * Requirement: REQ_TAG(PDK-5808), REQ_TAG(PDK-9159), REQ_TAG(PDK-9329)
 * Design: did_pmic_gpio_cfg_readback
 * Architecture: aid_pmic_gpio_cfg
 *
 *          This function is used to enable GPIO pin Interrupts
 *
 * \param   pPmicCoreHandle [IN]    PMIC Interface Handle.
 * \param   pin             [IN]    PMIC GPIO number.
 *                                   Valid values for TPS6594x Leo Device
 *                                   \ref Pmic_Tps6594xLeo_GpioPin.
 *                                   Valid values for LP8764x HERA Device
 *                                   \ref Pmic_Lp8764xHera_GpioPin.
 * \param   intrType        [IN]    Interrupt type \ref Pmic_GpioInterruptCfg
 * \param   maskPol         [IN]    FSM trigger masking polarity select for GPIO.
 *                                  Valid values
 *                                  \ref Pmic_GpioInterruptPolCfg.
 *
 * \return  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_gpioSetIntr(Pmic_CoreHandle_t *pPmicCoreHandle,
                         const uint8_t      pin,
                         const uint8_t      intrType,
                         const uint8_t      maskPol)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t index  = 0U;

    /* Parameter Validation */
    status = Pmic_gpioParamCheck(pPmicCoreHandle, pin);

    /* Set Pmic_GpioIntRegCfg_t array index for given GPIO Pin */
    index = pin - 1U;

    if((PMIC_ST_SUCCESS == status) && (intrType > PMIC_GPIO_DISABLE_INTERRUPT))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    if((PMIC_ST_SUCCESS == status) && (maskPol > PMIC_GPIO_POL_HIGH))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    if(PMIC_ST_SUCCESS == status)
    {
        if(PMIC_GPIO_DISABLE_INTERRUPT == intrType)
        {
            status = Pmic_gpioIntrDisable(pPmicCoreHandle, index);
        }
        else
        {
            status = Pmic_gpioIntrEnable(pPmicCoreHandle,
                                         index,
                                         intrType,
                                         maskPol);
        }
    }

    return status;
}

/*!
 * \brief   This function is used to set the NPWRON/Enable deglitch time and
 *          NPWRON/Enable Pull UP/Down configuration
 *          Valid only for TPS6594x PMIC
 */
int32_t Pmic_gpioSetNPwronEnableDeglitchPullCtrlCfg(
                                          Pmic_CoreHandle_t   *pPmicCoreHandle,
                                          const Pmic_GpioCfg_t gpioCfg)
{
    int32_t status = PMIC_ST_SUCCESS;

    if((bool)true == pmic_validParamCheck(gpioCfg.validParams,
                                           PMIC_GPIO_CFG_DEGLITCH_VALID))
    {
        if(gpioCfg.deglitchEnable > PMIC_GPIO_DEGLITCH_ENABLE)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }

        if(PMIC_ST_SUCCESS == status)
        {
            /* setting NPWRON/Enable deglitch time */
            status = Pmic_gpioSetDeglitchTime(pPmicCoreHandle,
                                              PMIC_NPWRON_ENABLE_PIN,
                                              gpioCfg);
        }
    }

    if((PMIC_ST_SUCCESS == status) &&
       ((bool)true == pmic_validParamCheck(gpioCfg.validParams,
                                           PMIC_GPIO_CFG_PULL_VALID)))
    {
        if(gpioCfg.pullCtrl > PMIC_GPIO_PULL_UP)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }

        if(PMIC_ST_SUCCESS == status)
        {
            /* setting NPWRON/Enable Pull UP/Down */
            status = Pmic_gpioSetPullCtrl(pPmicCoreHandle,
                                          PMIC_NPWRON_ENABLE_PIN,
                                          gpioCfg);
        }
    }

    return status;
}

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
                                        const Pmic_GpioCfg_t gpioCfg)
{
    int32_t status = PMIC_ST_SUCCESS;

    /* Validation of input Parameters */
    status = Pmic_gpioValidateParams(pPmicCoreHandle);

    if(PMIC_ST_SUCCESS == status)
    {
        switch(pPmicCoreHandle->pmicDeviceType)
        {
            case PMIC_DEV_HERA_LP8764X:
                status = Pmic_gpioLp8764xSetEnablePinConfiguration(
                                                               pPmicCoreHandle,
                                                               gpioCfg);
                break;
            default:
                /* Default case is valid only for TPS6594x LEO PMIC */
                status = Pmic_gpioTps6594xSetNPwronPinConfiguration(
                                                               pPmicCoreHandle,
                                                               gpioCfg);
                break;
        }
    }

    return status;
}

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
                                            Pmic_GpioCfg_t    *pGpioCfg)
{
    int32_t status = PMIC_ST_SUCCESS;

    /* Parameter Validation */
    status = Pmic_gpioNpoweronEnableParamCheck(pPmicCoreHandle, pGpioCfg);

    if(PMIC_ST_SUCCESS == status)
    {
        switch(pPmicCoreHandle->pmicDeviceType)
        {
            case PMIC_DEV_HERA_LP8764X:
                status = Pmic_gpioLp8764xGetEnablePinConfiguration(
                                                               pPmicCoreHandle,
                                                               pGpioCfg);
                break;
            default:
                /* Default case is valid only for TPS6594x LEO PMIC */
                status = Pmic_gpioTps6594xGetNPwronPinConfiguration(
                                                               pPmicCoreHandle,
                                                               pGpioCfg);
                break;
        }
    }

    return status;
}
