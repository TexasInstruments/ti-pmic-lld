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

#include <pmic_core_priv.h>
#include <pmic_io_priv.h>
#include <pmic_gpio_tps65941_priv.h>

/*!
 * \brief   This function is used to get the PMIC GPIO configuration
 */
static int32_t Pmic_get_gpioInOutCfg(Pmic_CoreHandle_t *pPmicCoreHandle,
                                     Pmic_GpioInOutCfg_t **pGpioInOutCfg)
{
    int32_t status = PMIC_ST_SUCCESS;

    if(NULL == pGpioInOutCfg)
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if(PMIC_ST_SUCCESS == status)
    {
        switch(pPmicCoreHandle->pmicDeviceType)
        {
        case PMIC_DEV_LEO_TPS6594:
            pmic_get_tps65941_gpioInOutCfg(pGpioInOutCfg);
            break;
        default:
            status = PMIC_ST_ERR_INV_DEVICE;
            break;
        }
    }

    return status;
}

/*!
 * \brief   This function is used to get the PMIC GPIO configuration
 */
static int32_t Pmic_get_gpioIntRegCfg(Pmic_CoreHandle_t     *pPmicCoreHandle,
                                      Pmic_GpioIntRegCfg_t **pGpioIntRegCfg)
{
    int32_t status = PMIC_ST_SUCCESS;

    if(NULL == pGpioIntRegCfg)
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if(PMIC_ST_SUCCESS == status)
    {
        switch(pPmicCoreHandle->pmicDeviceType)
        {
        case PMIC_DEV_LEO_TPS6594:
            pmic_get_tps65941_gpioIntRegCfg(pGpioIntRegCfg);
            break;
        default:
            status = PMIC_ST_ERR_INV_DEVICE;
            break;
        }
    }

    return status;
}

/*!
 * \brief   This function is used to set the GPIO Pin Functionality
 */
static int32_t Pmic_gpioParamCheck(Pmic_CoreHandle_t *pPmicCoreHandle,
                                   uint8_t            pin)
{
    int32_t status = PMIC_ST_SUCCESS;

    if(NULL == pPmicCoreHandle)
    {
        status = PMIC_ST_ERR_INV_HANDLE;
    }

    if((PMIC_ST_SUCCESS == status) &&
       (false == pPmicCoreHandle->pPmic_SubSysInfo->gpioEnable))
    {
        status = PMIC_ST_ERR_INV_DEVICE;
    }

    if((PMIC_ST_SUCCESS == status) && (pin > PMIC_GPIO_PIN_MAX))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    return status;
}

/*!
 * \brief   This function is used to set the GPIO Pin Functionality
 */
static int32_t Pmic_gpioSetPinFunc(Pmic_CoreHandle_t *pPmicCoreHandle,
                                   uint8_t            pin,
                                   Pmic_GpioCfg_t    *pGpioCfg)
{
    int32_t status                     = PMIC_ST_SUCCESS;
    uint8_t regData                    = 0U;
    Pmic_GpioInOutCfg_t *pGpioInOutCfg = NULL;

    /* Get PMIC gpio configuration */
    status = Pmic_get_gpioInOutCfg(pPmicCoreHandle, &pGpioInOutCfg);

    /* Start Critical Section */
    Pmic_criticalSectionStart(pPmicCoreHandle);

    if(PMIC_ST_SUCCESS == status)
    {
        /* Reading GPIO/NPWRON conf register */
        status = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        pGpioInOutCfg[pin].regAddr,
                                        &regData);
    }

    if (PMIC_ST_SUCCESS == status)
    {
        /* Setting pin function */
        if(PMIC_NPWRON_PIN != pin)
        {
            /* setting GPIO pin function */
            HW_REG_SET_FIELD(regData,
                             PMIC_GPIOX_CONF_GPIO_SEL,
                             pGpioCfg->pinFunc);
        }
        else
        {
            /* setting NPWRON pin function */
            HW_REG_SET_FIELD(regData,
                             PMIC_NPWRON_CONF_NPWRON_SEL,
                             pGpioCfg->pinFunc);
        }

        status = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                        pGpioInOutCfg[pin].regAddr,
                                        regData);
    }

    /* Stop Critical Section Handle */
    Pmic_criticalSectionStop(pPmicCoreHandle);

    return status;
}

/*!
 * \brief   This function is used to get the GPIO Pin Functionality
 */
static int32_t Pmic_gpioGetPinFunc(Pmic_CoreHandle_t *pPmicCoreHandle,
                                   uint8_t            pin,
                                   Pmic_GpioCfg_t    *pGpioCfg)
{
    int32_t status                     = PMIC_ST_SUCCESS;
    uint8_t regData                    = 0U;
    Pmic_GpioInOutCfg_t *pGpioInOutCfg = NULL;

    /* Get PMIC gpio configuration */
    status = Pmic_get_gpioInOutCfg(pPmicCoreHandle, &pGpioInOutCfg);

    /* Start Critical Section */
    Pmic_criticalSectionStart(pPmicCoreHandle);

    if(PMIC_ST_SUCCESS == status)
    {
        /* Reading the GPIO configuration */
        status = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        pGpioInOutCfg[pin].regAddr,
                                        &regData);
    }

    /* Stop Critical Section */
    Pmic_criticalSectionStop(pPmicCoreHandle);

    if(PMIC_ST_SUCCESS == status)
    {
        if(PMIC_NPWRON_PIN != pin)
        {
            pGpioCfg->pinFunc = HW_REG_GET_FIELD(regData,
                                                 PMIC_GPIOX_CONF_GPIO_SEL);
        }
        else
        {
            pGpioCfg->pinFunc = HW_REG_GET_FIELD(regData,
                                                 PMIC_NPWRON_CONF_NPWRON_SEL);
        }
    }

    return status;
}

/*!
 * \brief   This function is used to set the NPWRON Pin Polarity
 */
static int32_t Pmic_gpioSetPinPolarity(Pmic_CoreHandle_t *pPmicCoreHandle,
                                       uint8_t            pin,
                                       Pmic_GpioCfg_t    *pGpioCfg)
{
    int32_t status                     = PMIC_ST_SUCCESS;
    uint8_t regData                    = 0U;
    Pmic_GpioInOutCfg_t *pGpioInOutCfg = NULL;

    /* Get PMIC gpio configuration */
    status = Pmic_get_gpioInOutCfg(pPmicCoreHandle, &pGpioInOutCfg);

    /* Start Critical Section */
    Pmic_criticalSectionStart(pPmicCoreHandle);

    if(PMIC_ST_SUCCESS == status)
    {
        /* Reading GPIO/NPWRON conf register */
        status = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        pGpioInOutCfg[pin].regAddr,
                                        &regData);
    }

    if (PMIC_ST_SUCCESS == status)
    {
        /* Setting ENABLE pin polarity (valid only for NPWRON pin) */
        HW_REG_SET_FIELD(regData,
                         PMIC_NPWRON_CONF_NPWRON_POL,
                         pGpioCfg->pinPolarity);

        status = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                        pGpioInOutCfg[pin].regAddr,
                                        regData);
    }

    /* Stop Critical Section Handle */
    Pmic_criticalSectionStop(pPmicCoreHandle);

    return status;
}

/*!
 * \brief   This function is used to get the NPWRON Pin Polarity
 */
static int32_t Pmic_gpioGetPinPolarity(Pmic_CoreHandle_t *pPmicCoreHandle,
                                       uint8_t            pin,
                                       Pmic_GpioCfg_t    *pGpioCfg)
{
    int32_t status                     = PMIC_ST_SUCCESS;
    uint8_t regData                    = 0U;
    Pmic_GpioInOutCfg_t *pGpioInOutCfg = NULL;

    /* Get PMIC gpio configuration */
    status = Pmic_get_gpioInOutCfg(pPmicCoreHandle, &pGpioInOutCfg);

    /* Start Critical Section */
    Pmic_criticalSectionStart(pPmicCoreHandle);

    if(PMIC_ST_SUCCESS == status)
    {
        /* Reading the GPIO configuration */
        status = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        pGpioInOutCfg[pin].regAddr,
                                        &regData);
    }

    /* Stop Critical Section */
    Pmic_criticalSectionStop(pPmicCoreHandle);

    if((PMIC_ST_SUCCESS == status) && (PMIC_NPWRON_PIN == pin))
    {
        /* Reading NPWRON/ENABLE pin polarity */
        pGpioCfg->pinPolarity = HW_REG_GET_FIELD(regData,
                                                 PMIC_NPWRON_CONF_NPWRON_POL);
    }

    return status;
}

/*!
 * \brief   This function is used to set the GPIO Pin Pull Control
 */
static int32_t Pmic_gpioSetPullCtrl(Pmic_CoreHandle_t *pPmicCoreHandle,
                                    uint8_t            pin,
                                    Pmic_GpioCfg_t    *pGpioCfg)
{
    int32_t status                     = PMIC_ST_SUCCESS;
    uint8_t regData                    = 0U;
    Pmic_GpioInOutCfg_t *pGpioInOutCfg = NULL;

    /* Get PMIC gpio configuration */
    status = Pmic_get_gpioInOutCfg(pPmicCoreHandle, &pGpioInOutCfg);

    /* Start Critical Section */
    Pmic_criticalSectionStart(pPmicCoreHandle);

    if(PMIC_ST_SUCCESS == status)
    {
        /* Reading GPIO/NPWRON conf register */
        status = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        pGpioInOutCfg[pin].regAddr,
                                        &regData);
    }

    if (PMIC_ST_SUCCESS == status)
    {
        if(PMIC_GPIO_PULL_DISABLED == pGpioCfg->pullCtrl)
        {
            /* Disable pull-up/pull-down feature */
            HW_REG_SET_FIELD(regData,
                             PMIC_GPIOX_CONF_GPIO_PU_PD_EN,
                             PMIC_GPIO_PU_PD_DISABLE);
        }

        if((PMIC_GPIO_PULL_UP == pGpioCfg->pullCtrl) ||
           (PMIC_GPIO_PULL_DOWN == pGpioCfg->pullCtrl))
        {
            /* Enable pull-up/pull-down feature */
            HW_REG_SET_FIELD(regData,
                             PMIC_GPIOX_CONF_GPIO_PU_PD_EN,
                             PMIC_GPIO_PU_PD_ENABLE);

            /* clear pull-up/pull-down select field */
            /* Select pull-down resistor */
            HW_REG_SET_FIELD(regData,
                             PMIC_GPIOX_CONF_GPIO_PU_SEL,
                             PMIC_GPIO_PD_SELECT);
        }

        if(PMIC_GPIO_PULL_UP == pGpioCfg->pullCtrl)
        {
            /* select pull-up resistor */
            HW_REG_SET_FIELD(regData,
                             PMIC_GPIOX_CONF_GPIO_PU_SEL,
                             PMIC_GPIO_PU_SELECT);
        }

        status = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                        pGpioInOutCfg[pin].regAddr,
                                        regData);
    }

    /* Stop Critical Section */
    Pmic_criticalSectionStop(pPmicCoreHandle);

    return status;
}

/*!
 * \brief   This function is used to get the GPIO Pin Pull Control
 */
static int32_t Pmic_gpioGetPullCtrl(Pmic_CoreHandle_t *pPmicCoreHandle,
                                    uint8_t            pin,
                                    Pmic_GpioCfg_t    *pGpioCfg)
{
    int32_t status                     = PMIC_ST_SUCCESS;
    uint8_t regData                    = 0U;
    Pmic_GpioInOutCfg_t *pGpioInOutCfg = NULL;

    /* Get PMIC gpio configuration */
    status = Pmic_get_gpioInOutCfg(pPmicCoreHandle, &pGpioInOutCfg);

    /* Start Critical Section */
    Pmic_criticalSectionStart(pPmicCoreHandle);

    if(PMIC_ST_SUCCESS == status)
    {
        /* Reading the GPIO configuration */
        status = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        pGpioInOutCfg[pin].regAddr,
                                        &regData);
    }

    /* Stop Critical Section */
    Pmic_criticalSectionStop(pPmicCoreHandle);

    if(PMIC_ST_SUCCESS == status)
    {
        /* Reading gpio pull control */
        if(!(HW_REG_GET_FIELD(regData,
                              PMIC_GPIOX_CONF_GPIO_PU_PD_EN)))
        {
            pGpioCfg->pullCtrl = PMIC_GPIO_PULL_DISABLED;
        }
        else if(HW_REG_GET_FIELD(regData,
                                 PMIC_GPIOX_CONF_GPIO_PU_SEL))
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
static int32_t Pmic_gpioSetPinDir(Pmic_CoreHandle_t *pPmicCoreHandle,
                                  uint8_t            pin,
                                  Pmic_GpioCfg_t    *pGpioCfg)
{
    int32_t status                     = PMIC_ST_SUCCESS;
    uint8_t regData                    = 0U;
    Pmic_GpioInOutCfg_t *pGpioInOutCfg = NULL;

    /* Get PMIC gpio configuration */
    status = Pmic_get_gpioInOutCfg(pPmicCoreHandle, &pGpioInOutCfg);

    /* Start Critical Section */
    Pmic_criticalSectionStart(pPmicCoreHandle);

    if(PMIC_ST_SUCCESS == status)
    {
        /* Reading GPIO/NPWRON conf register */
        status = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        pGpioInOutCfg[pin].regAddr,
                                        &regData);
    }

    if (PMIC_ST_SUCCESS == status)
    {
        /* set gpio pin direction */
        HW_REG_SET_FIELD(regData,
                         PMIC_GPIOX_CONF_GPIO_DIR,
                         pGpioCfg->pinDir);

        status = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                        pGpioInOutCfg[pin].regAddr,
                                        regData);
    }

    /* Stop Critical Section */
    Pmic_criticalSectionStop(pPmicCoreHandle);

    return status;
}

/*!
 * \brief   This function is used to get the GPIO Pin Direction
 */
static int32_t Pmic_gpioGetPinDir(Pmic_CoreHandle_t *pPmicCoreHandle,
                                  uint8_t            pin,
                                  Pmic_GpioCfg_t    *pGpioCfg)
{
    int32_t status                     = PMIC_ST_SUCCESS;
    uint8_t regData                    = 0U;
    Pmic_GpioInOutCfg_t *pGpioInOutCfg = NULL;

    /* Get PMIC gpio configuration */
    status = Pmic_get_gpioInOutCfg(pPmicCoreHandle, &pGpioInOutCfg);

    /* Start Critical Section */
    Pmic_criticalSectionStart(pPmicCoreHandle);

    if(PMIC_ST_SUCCESS == status)
    {
        /* Reading the GPIO configuration */
        status = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        pGpioInOutCfg[pin].regAddr,
                                        &regData);
    }

    /* Stop Critical Section */
    Pmic_criticalSectionStop(pPmicCoreHandle);

    if((PMIC_ST_SUCCESS == status) && (PMIC_NPWRON_PIN != pin))
    {
        /* Reading gpio direction */
        pGpioCfg->pinDir = HW_REG_GET_FIELD(regData, PMIC_GPIOX_CONF_GPIO_DIR);
    }

    return status;
}

/*!
 * \brief   This function is used to set the GPIO Deglitch Time
 */
static int32_t Pmic_gpioSetDeglitchTime(Pmic_CoreHandle_t *pPmicCoreHandle,
                                        uint8_t            pin,
                                        Pmic_GpioCfg_t    *pGpioCfg)
{
    int32_t status                     = PMIC_ST_SUCCESS;
    uint8_t regData                    = 0U;
    Pmic_GpioInOutCfg_t *pGpioInOutCfg = NULL;

    /* Get PMIC gpio configuration */
    status = Pmic_get_gpioInOutCfg(pPmicCoreHandle, &pGpioInOutCfg);

    /* Start Critical Section */
    Pmic_criticalSectionStart(pPmicCoreHandle);

    if(PMIC_ST_SUCCESS == status)
    {
        /* Reading GPIO/NPWRON conf register */
        status = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        pGpioInOutCfg[pin].regAddr,
                                        &regData);
    }

    if (PMIC_ST_SUCCESS == status)
    {
        /* setting deglitch time */
        HW_REG_SET_FIELD(regData,
                         PMIC_GPIOX_CONF_GPIO_DEGLITCH_EN,
                         pGpioCfg->deglitchEnable);

        status = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                        pGpioInOutCfg[pin].regAddr,
                                        regData);
    }

    /* Stop Critical Section */
    Pmic_criticalSectionStop(pPmicCoreHandle);

    return status;
}

/*!
 * \brief   This function is used to get the GPIO Deglitch time
 */
static int32_t Pmic_gpioGetDeglitchTime(Pmic_CoreHandle_t *pPmicCoreHandle,
                                        uint8_t            pin,
                                        Pmic_GpioCfg_t    *pGpioCfg)
{
    int32_t status                     = PMIC_ST_SUCCESS;
    uint8_t regData                    = 0U;
    Pmic_GpioInOutCfg_t *pGpioInOutCfg = NULL;

    /* Get PMIC gpio configuration */
    status = Pmic_get_gpioInOutCfg(pPmicCoreHandle, &pGpioInOutCfg);

    /* Start Critical Section */
    Pmic_criticalSectionStart(pPmicCoreHandle);

    if(PMIC_ST_SUCCESS == status)
    {
        /* Reading the GPIO configuration */
        status = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        pGpioInOutCfg[pin].regAddr,
                                        &regData);
    }

    /* Stop Critical Section */
    Pmic_criticalSectionStop(pPmicCoreHandle);

    if(PMIC_ST_SUCCESS == status)
    {
        /* Reading signal deglitch time */
        pGpioCfg->deglitchEnable = HW_REG_GET_FIELD(
                                            regData,
                                            PMIC_GPIOX_CONF_GPIO_DEGLITCH_EN);
    }

    return status;
}

/*!
 * \brief   This function is used to set the GPIO Output Signal Type
 */
static int32_t Pmic_gpioSetOutputSignalType(Pmic_CoreHandle_t *pPmicCoreHandle,
                                            uint8_t            pin,
                                            Pmic_GpioCfg_t    *pGpioCfg)
{
    int32_t status                     = PMIC_ST_SUCCESS;
    uint8_t regData                    = 0U;
    Pmic_GpioInOutCfg_t *pGpioInOutCfg = NULL;

    /* Get PMIC gpio configuration */
    status = Pmic_get_gpioInOutCfg(pPmicCoreHandle, &pGpioInOutCfg);

    /* Start Critical Section */
    Pmic_criticalSectionStart(pPmicCoreHandle);

    if(PMIC_ST_SUCCESS == status)
    {
        /* Reading GPIO/NPWRON conf register */
        status = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        pGpioInOutCfg[pin].regAddr,
                                        &regData);
    }

    if (PMIC_ST_SUCCESS == status)
    {
        if(PMIC_NPWRON_PIN != pin)
        {
            /* selecting output type */
            HW_REG_SET_FIELD(regData,
                             PMIC_GPIOX_CONF_GPIO_OD,
                             pGpioCfg->outputSignalType);
        }
        else
        {
            /* selecting NPWRON output type */
            HW_REG_SET_FIELD(regData,
                             PMIC_NPWRON_CONF_NPWRON_OD,
                             pGpioCfg->outputSignalType);
        }

        status = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                        pGpioInOutCfg[pin].regAddr,
                                        regData);
    }

    /* Stop Critical Section */
    Pmic_criticalSectionStop(pPmicCoreHandle);

    return status;
}

/*!
 * \brief   This function is used to get the GPIO Output Signal Type
 */
static int32_t Pmic_gpioGetOutputSignalType(Pmic_CoreHandle_t *pPmicCoreHandle,
                                            uint8_t            pin,
                                            Pmic_GpioCfg_t    *pGpioCfg)
{
    int32_t status                     = PMIC_ST_SUCCESS;
    uint8_t regData                    = 0U;
    Pmic_GpioInOutCfg_t *pGpioInOutCfg = NULL;

    /* Get PMIC gpio configuration */
    status = Pmic_get_gpioInOutCfg(pPmicCoreHandle, &pGpioInOutCfg);

    /* Start Critical Section */
    Pmic_criticalSectionStart(pPmicCoreHandle);

    if(PMIC_ST_SUCCESS == status)
    {
        /* Reading the GPIO configuration */
        status = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        pGpioInOutCfg[pin].regAddr,
                                        &regData);
    }

    /* Stop Critical Section */
    Pmic_criticalSectionStop(pPmicCoreHandle);

    if(PMIC_ST_SUCCESS == status)
    {
        if(PMIC_NPWRON_PIN != pin)
        {
            /* Reading output signal type */
            pGpioCfg->outputSignalType = HW_REG_GET_FIELD(
                                                regData,
                                                PMIC_GPIOX_CONF_GPIO_OD);
        }
        else
        {
            /* Reading output signal type for NPWRON pin */
            pGpioCfg->outputSignalType = HW_REG_GET_FIELD(
                                               regData,
                                               PMIC_NPWRON_CONF_NPWRON_OD);
        }
    }

    return status;
}

/*!
 * \brief   This function is used to Enable GPIO Interrupt
 */
static int32_t Pmic_gpioIntrEnable(Pmic_CoreHandle_t *pPmicCoreHandle,
                                   uint8_t            pin,
                                   uint8_t            intrType,
                                   uint8_t            maskPol)
{
    int32_t status                       = PMIC_ST_SUCCESS;
    uint8_t regData                      = 0U;
    Pmic_GpioIntRegCfg_t *pGpioIntRegCfg = NULL;

    status = Pmic_get_gpioIntRegCfg(pPmicCoreHandle, &pGpioIntRegCfg);

    if (PMIC_ST_SUCCESS == status)
    {
        Pmic_criticalSectionStart(pPmicCoreHandle);

        status = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        pGpioIntRegCfg[pin].intRegAddr,
                                        &regData);
    }

    if(PMIC_ST_SUCCESS == status)
    {
        BIT_POS_SET_VAL(regData,
                        pGpioIntRegCfg[pin].intRegBitPos,
                        PMIC_GPIO_INT_ENABLE);
        BIT_POS_SET_VAL(regData,
                        pGpioIntRegCfg[pin].intRegPolBitPos,
                        maskPol);
        status = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                        pGpioIntRegCfg[pin].intRegAddr,
                                        regData);
    }

    if((PMIC_ST_SUCCESS == status) &&
       (PMIC_GPIO_FALL_INTERRUPT == intrType))
    {
        status = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        pGpioIntRegCfg[pin].intFallRegAddr,
                                        &regData);
        if(PMIC_ST_SUCCESS == status)
        {
            BIT_POS_SET_VAL(regData,
                            pGpioIntRegCfg[pin].intFallRegBitPos,
                            PMIC_GPIO_INT_ENABLE);
            status = Pmic_commIntf_sendByte(
                                        pPmicCoreHandle,
                                        pGpioIntRegCfg[pin].intFallRegAddr,
                                        regData);
        }
    }

    if((PMIC_ST_SUCCESS == status) &&
        (PMIC_GPIO_RISE_INTERRUPT == intrType))
    {
            status = Pmic_commIntf_recvByte(
                                        pPmicCoreHandle,
                                        pGpioIntRegCfg[pin].intRiseRegAddr,
                                        &regData);
        if(PMIC_ST_SUCCESS == status)
        {
            BIT_POS_SET_VAL(regData,
                            pGpioIntRegCfg[pin].intRiseRegBitPos,
                            PMIC_GPIO_INT_ENABLE);
            status = Pmic_commIntf_sendByte(
                                        pPmicCoreHandle,
                                        pGpioIntRegCfg[pin].intRiseRegAddr,
                                        regData);
        }
    }

    /* Stop Critical Section */
    Pmic_criticalSectionStop(pPmicCoreHandle);

    return status;
}

/*!
 * \brief   This function is used to Disable GPIO Interrupt
 */
static int32_t Pmic_gpioIntrDisable(Pmic_CoreHandle_t *pPmicCoreHandle,
                                    uint8_t            pin)
{
    int32_t status                       = PMIC_ST_SUCCESS;
    uint8_t regData                      = 0U;
    Pmic_GpioIntRegCfg_t *pGpioIntRegCfg = NULL;

    status = Pmic_get_gpioIntRegCfg(pPmicCoreHandle, &pGpioIntRegCfg);

    if (PMIC_ST_SUCCESS == status)
    {
        Pmic_criticalSectionStart(pPmicCoreHandle);

        status = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        pGpioIntRegCfg[pin].intRegAddr,
                                        &regData);
    }

    if(PMIC_ST_SUCCESS == status)
    {
        BIT_POS_SET_VAL(regData,
                        pGpioIntRegCfg[pin].intRegBitPos,
                        PMIC_GPIO_INT_MASK);
        status = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                        pGpioIntRegCfg[pin].intRegAddr,
                                        regData);
    }

    /* Stop Critical Section */
    Pmic_criticalSectionStop(pPmicCoreHandle);

    return status;
}

/*
 * \brief   PMIC GPIO set configuration function
 *          This function is used to set the required configuration for the
 *          specified GPIO pin when corresponding bit field is set.
 *
 * \param   pPmicCoreHandle [IN]    PMIC Interface Handle
 * \param   pin             [IN]    PMIC GPIO pin number
 *                                  Valid values \ref Pmic_GpioPin
 * \param   pGpioCfg        [IN]    pointer to set required configuration for
 *                                  the specified GPIO pin
 *
 * \return  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_gpioSetConfiguration(Pmic_CoreHandle_t *pPmicCoreHandle,
                                  uint8_t            pin,
                                  Pmic_GpioCfg_t    *pGpioCfg)
{
    int32_t status  = PMIC_ST_SUCCESS;

    /* Validation of input Parameters */
    status = Pmic_gpioParamCheck(pPmicCoreHandle, pin);

    if((PMIC_ST_SUCCESS == status) && (NULL == pGpioCfg))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if((PMIC_ST_SUCCESS == status) &&
       (true == pmic_validParamCheck(pGpioCfg->validParams,
                                     PMIC_GPIO_CFG_PINFUNC_VALID)))
    {
        if(PMIC_NPWRON_PIN != pin)
        {
            if(pGpioCfg->pinFunc >= PMIC_GPIO_PINFUNC_MAX)
            {
                status = PMIC_ST_ERR_INV_GPIO_FUNC;
            }
        }
        else
        {
            if(pGpioCfg->pinFunc > PMIC_NPWRON_PINFUNC_NONE)
            {
                status = PMIC_ST_ERR_INV_GPIO_FUNC;
            }
        }

        if(PMIC_ST_SUCCESS == status)
        {
            /* Setting pin function */
            status = Pmic_gpioSetPinFunc(pPmicCoreHandle, pin , pGpioCfg);
        }
    }

    if((PMIC_ST_SUCCESS == status) &&
       (PMIC_NPWRON_PIN == pin) &&
       (true == pmic_validParamCheck(pGpioCfg->validParams,
                                     PMIC_NPWRON_CFG_POLARITY_VALID)))
    {
        if(pGpioCfg->pinPolarity > PMIC_GPIO_HIGH)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }

        if(PMIC_ST_SUCCESS == status)
        {
            /* Setting NPWRON pin polarity */
            status = Pmic_gpioSetPinPolarity(pPmicCoreHandle, pin, pGpioCfg);
        }
    }

    if((PMIC_ST_SUCCESS == status) &&
       (PMIC_NPWRON_PIN != pin) &&
       (true == pmic_validParamCheck(pGpioCfg->validParams,
                                     PMIC_GPIO_CFG_DIR_VALID)))
    {
        if(pGpioCfg->pinDir > PMIC_GPIO_HIGH)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }

        if(PMIC_ST_SUCCESS == status)
        {
            /* set gpio pin direction */
            status = Pmic_gpioSetPinDir(pPmicCoreHandle, pin, pGpioCfg);
        }
    }

    if((PMIC_ST_SUCCESS == status) &&
       (true == pmic_validParamCheck(pGpioCfg->validParams,
                                     PMIC_GPIO_CFG_DEGLITCH_VALID)))
    {
        if(pGpioCfg->deglitchEnable > PMIC_GPIO_DEGLITCH_ENABLE)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }

        if(PMIC_ST_SUCCESS == status)
        {
            /* setting deglitch time */
            status = Pmic_gpioSetDeglitchTime(pPmicCoreHandle, pin, pGpioCfg);
        }
    }

    if((PMIC_ST_SUCCESS == status) &&
       (true == pmic_validParamCheck(pGpioCfg->validParams,
                                     PMIC_GPIO_CFG_OD_VALID)))
    {
        if(pGpioCfg->outputSignalType > PMIC_GPIO_OPEN_DRAIN_OUTPUT)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }

        if(PMIC_ST_SUCCESS == status)
        {
            /* setting open drain */
            status = Pmic_gpioSetOutputSignalType(pPmicCoreHandle, pin,
                                                  pGpioCfg);
        }
    }

    if (PMIC_ST_SUCCESS == status)
    {
        if(true == pmic_validParamCheck(pGpioCfg->validParams,
                                        PMIC_GPIO_CFG_PULL_VALID))
        {
            /* setting Pull UP/Down */
            status = Pmic_gpioSetPullCtrl(pPmicCoreHandle, pin, pGpioCfg);
        }
    }

    return status;
}

/*
 * \brief   PMIC GPIO get configuration function
 *          This function is used to read the configuration for the specified
 *          GPIO pin when corresponding bit field is set.
 *
 * \param   pPmicCoreHandle [IN]    PMIC Interface Handle
 * \param   pin             [IN]    PMIC GPIO pin number
 *                                  Valid values \ref Pmic_GpioPin
 * \param   pGpioCfg        [OUT]   Pointer to store specified GPIO pin
 *                                  configuration
 *
 * \return  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_gpioGetConfiguration(Pmic_CoreHandle_t *pPmicCoreHandle,
                                  uint8_t            pin,
                                  Pmic_GpioCfg_t    *pGpioCfg)
{
    int32_t status  = PMIC_ST_SUCCESS;

    /* Parameter Validation */
    status = Pmic_gpioParamCheck(pPmicCoreHandle, pin);

    if((PMIC_ST_SUCCESS == status) && (NULL == pGpioCfg))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if(PMIC_ST_SUCCESS == status)
    {
        if(PMIC_NPWRON_PIN != pin)
        {
            if(true == pmic_validParamCheck(pGpioCfg->validParams,
                                            PMIC_GPIO_CFG_DIR_VALID))
            {
                status = Pmic_gpioGetPinDir(pPmicCoreHandle, pin, pGpioCfg);
            }
        }
        else
        {
            if(true == pmic_validParamCheck(pGpioCfg->validParams,
                                            PMIC_NPWRON_CFG_POLARITY_VALID))
            {
                status = Pmic_gpioGetPinPolarity(pPmicCoreHandle, pin,
                                                 pGpioCfg);
            }
        }
    }

    if((PMIC_ST_SUCCESS == status) &&
       (true == pmic_validParamCheck(pGpioCfg->validParams,
                                     PMIC_GPIO_CFG_OD_VALID)))
    {
        status = Pmic_gpioGetOutputSignalType(pPmicCoreHandle,
                                              pin,
                                              pGpioCfg);
    }

    if((PMIC_ST_SUCCESS == status) &&
       (true == pmic_validParamCheck(pGpioCfg->validParams,
                                     PMIC_GPIO_CFG_DEGLITCH_VALID)))
    {
        status = Pmic_gpioGetDeglitchTime(pPmicCoreHandle, pin, pGpioCfg);
    }

    if((PMIC_ST_SUCCESS == status) &&
       (true == pmic_validParamCheck(pGpioCfg->validParams,
                                     PMIC_GPIO_CFG_PINFUNC_VALID)))
    {
        status = Pmic_gpioGetPinFunc(pPmicCoreHandle, pin, pGpioCfg);
    }

    if((PMIC_ST_SUCCESS == status) &&
       (true == pmic_validParamCheck(pGpioCfg->validParams,
                                        PMIC_GPIO_CFG_PULL_VALID)))
    {
        status = Pmic_gpioGetPullCtrl(pPmicCoreHandle, pin, pGpioCfg);
    }

    return status;
}

/*
 * \brief   PMIC GPIO set value function
 *          This function is used to configure the signal level of the
 *          specified GPIO pin
 *
 * \param   pPmicCoreHandle [IN]    PMIC Interface Handle
 * \param   pin             [IN]    PMIC GPIO pin number
 *                                  Valid values \ref Pmic_GpioPin
 * \param   pinValue        [IN]    PMIC GPIO signal level High/Low to be
 *                                  configured
 *                                  Valid values \ref Pmic_Gpio_SignalLvl
 *
 * \return  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_gpioSetValue(Pmic_CoreHandle_t *pPmicCoreHandle,
                          uint8_t            pin,
                          uint8_t            pinValue)
{
    int32_t status  = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    Pmic_GpioInOutCfg_t *pGpioInOutCfg = NULL;

    status = Pmic_gpioParamCheck(pPmicCoreHandle, pin);

    if((PMIC_ST_SUCCESS == status) &&
       ((PMIC_NPWRON_PIN == pin)))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    if((PMIC_ST_SUCCESS == status) &&
       (PMIC_GPIO_LOW != pinValue) &&
       (PMIC_GPIO_HIGH != pinValue))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    if(PMIC_ST_SUCCESS == status)
    {
        /* Get PMIC gpio configuration */
        status = Pmic_get_gpioInOutCfg(pPmicCoreHandle, &pGpioInOutCfg);
    }

    if(PMIC_ST_SUCCESS == status)
    {
        /* Start Critical Section */
        Pmic_criticalSectionStart(pPmicCoreHandle);

        /* checking for the pin direction to be output */
        status = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        pGpioInOutCfg[pin].regAddr,
                                        &regData);

        if(PMIC_ST_SUCCESS == status)
        {
            if(HW_REG_GET_FIELD(regData, PMIC_GPIOX_CONF_GPIO_DIR) ==
               PMIC_GPIO_OPEN_DRAIN_OUTPUT)
            {
                /* Setting the GPIO value */
                status = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                                pGpioInOutCfg[pin].outRegAddr,
                                                &regData);
                if(PMIC_ST_SUCCESS == status)
                {
                    BIT_POS_SET_VAL(regData,
                                    pGpioInOutCfg[pin].outRegBitPos,
                                    pinValue);
                    status = Pmic_commIntf_sendByte(
                                                pPmicCoreHandle,
                                                pGpioInOutCfg[pin].outRegAddr,
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

    }

    return status;
}

/*
 * \brief   PMIC GPIO get value function
 *          This function is used to read the signal level of the gpio pin
 *
 * \param   pPmicCoreHandle [IN]    PMIC Interface Handle
 * \param   pin             [IN]    PMIC GPIO pin number
 *                                  Valid values \ref Pmic_GpioPin
 * \param   pPinValue       [OUT]   To store PMIC GPIO signal level High/Low
 *                                  Valid values \ref Pmic_Gpio_SignalLvl
 *
 * \return  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_gpioGetValue(Pmic_CoreHandle_t *pPmicCoreHandle,
                          uint8_t            pin,
                          uint8_t           *pPinValue)
{
    int32_t status                     = PMIC_ST_SUCCESS;
    uint8_t regData                    = 0U;
    Pmic_GpioInOutCfg_t *pGpioInOutCfg = NULL;

    /* Parameter Validation */
    status = Pmic_gpioParamCheck(pPmicCoreHandle, pin);

    if((PMIC_ST_SUCCESS == status) && (NULL == pPinValue))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if(PMIC_ST_SUCCESS == status)
    {
        /* Get PMIC gpio configuration */
        status = Pmic_get_gpioInOutCfg(pPmicCoreHandle, &pGpioInOutCfg);
    }

    if(PMIC_ST_SUCCESS == status)
    {
        /* Start Critical Section */
        Pmic_criticalSectionStart(pPmicCoreHandle);

        /* Reading the GPIO/NPWRON pin value */
        status = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        pGpioInOutCfg[pin].inRegAddr,
                                        &regData);
        /* Stop Critical Section */
        Pmic_criticalSectionStop(pPmicCoreHandle);

        if(PMIC_ST_SUCCESS == status)
        {
            if(BIT_POS_GET_VAL(regData, pGpioInOutCfg[pin].inRegBitPos) != 0U)
            {
                *pPinValue = PMIC_GPIO_HIGH;
            }
            else
            {
                *pPinValue = PMIC_GPIO_LOW;
            }
        }
    }

    return status;
}

/*
* \brief   PMIC GPIO interrupt configuration function
*          This function is used to enable GPIO pin Interrupts
*
* \param   pPmicCoreHandle [IN]    PMIC Interface Handle
* \param   pin             [IN]    PMIC GPIO number
*                                  Valid values \ref Pmic_GpioPin
* \param   intrType        [IN]    Interrupt type \ref Pmic_GpioInterruptCfg
* \param   maskPol         [IN]    FSM trigger masking polarity select for GPIO
*                                  Valid values refer 
*                                  \ref Pmic_GpioInterruptPolCfg
*
* \return  PMIC_ST_SUCCESS in case of success or appropriate error code
*          For valid values \ref Pmic_ErrorCodes
*/
int32_t Pmic_gpioSetIntr(Pmic_CoreHandle_t *pPmicCoreHandle,
                         uint8_t            pin,
                         uint8_t            intrType,
                         uint8_t            maskPol)
{
    int32_t status       = PMIC_ST_SUCCESS;

    /* Parameter Validation */
    status = Pmic_gpioParamCheck(pPmicCoreHandle, pin);

    if((PMIC_ST_SUCCESS == status) && (PMIC_NPWRON_PIN == pin))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

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
            status = Pmic_gpioIntrDisable(pPmicCoreHandle, pin);
        }
        else
        {
            status = Pmic_gpioIntrEnable(pPmicCoreHandle,
                                        pin,
                                        intrType,
                                        maskPol);
        }
    }

    return status;
}

