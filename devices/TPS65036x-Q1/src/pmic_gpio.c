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
/**
 * @file pmic_gpio.c
 *
 * @brief PMIC LLD GPIO module source file containing definitions to APIs that
 * interact with and control PMIC GPIOs.
 */
#include "pmic.h"
#include "pmic_gpio.h"

#include "pmic_io.h"

#include "regmap/gpio.h"

/**
 * @brief The NINT_GPI_SEL bit field of FUNC_CONF register has two bit fields
 * that are the same. This define is to indicate the repeated bit field.
 */
#define PMIC_NINT_GPI_LPM_CTRL_MODE_INPUT_REPEATED ((uint8_t)3U)

static inline void GPIO_copyGpioCfg(const Pmic_GpioCfg_t *src, Pmic_GpioCfg_t *dst)
{
    (void)memmove((void *)dst, (const void *)src, sizeof(Pmic_GpioCfg_t));
}

/** @brief Set PMIC GPIO configuration (the GPIO pin that's not nINT_GPI) */
static int32_t GPIO_setGPIOCfg(const Pmic_Handle_t *handle, const Pmic_GpioCfg_t *gpioCfg)
{
    uint8_t regData = 0U;
    int32_t status;

    // Read INTERFACE_CONF
    Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);
    status = Pmic_ioRxByte(handle, PMIC_INTERFACE_CONF_REG, &regData);

    // Set GPIO polarity
    if (Pmic_validParamStatusCheck(gpioCfg->validParams, (uint32_t)PMIC_POLARITY_VALID, status))
    {
        if (gpioCfg->polarity > PMIC_POLARITY_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
        else
        {
            Pmic_setBitField(&regData, PMIC_GPIO_POL_SHIFT, PMIC_GPIO_POL_MASK, gpioCfg->polarity);
        }
    }

    // Set GPIO functionality
    if (Pmic_validParamStatusCheck(gpioCfg->validParams, (uint32_t)PMIC_FUNCTIONALITY_VALID, status))
    {
        if (gpioCfg->functionality > PMIC_GPIO_FUNCTIONALITY_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
        else
        {
            Pmic_setBitField(&regData, PMIC_GPIO_SEL_SHIFT, PMIC_GPIO_SEL_MASK, gpioCfg->functionality);
        }
    }

    // Write INTERFACE_CONF
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioTxByte(handle, PMIC_INTERFACE_CONF_REG, regData);
    }
    Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);

    return status;
}

/** @brief Set NINT_GPI configuration */
static int32_t GPIO_setNINTGPICfg(const Pmic_Handle_t *handle, const Pmic_GpioCfg_t *gpioCfg)
{
    uint8_t regData = 0U;
    int32_t status;

    // Read FUNC_CONF
    Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);
    status = Pmic_ioRxByte(handle, PMIC_FUNC_CONF_REG, &regData);

    // Set NINT_GPI pullup/pulldown resistor configuration
    if (Pmic_validParamStatusCheck(gpioCfg->validParams, (uint32_t)PMIC_PU_PD_CFG_VALID, status))
    {
        if (gpioCfg->puPdCfg > PMIC_PU_PD_CFG_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
        else
        {
            Pmic_setBitField(&regData, PMIC_NINT_GPI_PU_PD_SEL_SHIFT, PMIC_NINT_GPI_PU_PD_SEL_MASK, gpioCfg->puPdCfg);
        }
    }

    // Set NINT_GPI open-drain/push-pull output configuration
    if (Pmic_validParamStatusCheck(gpioCfg->validParams, (uint32_t)PMIC_OD_PP_CFG_VALID, status))
    {
        if (gpioCfg->odPpCfg > PMIC_OD_PP_CFG_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
        else
        {
            Pmic_setBitField(&regData, PMIC_NINT_PP_OD_SHIFT, PMIC_NINT_PP_OD_MASK, gpioCfg->odPpCfg);
        }
    }

    // Set NINT_GPI polarity
    if (Pmic_validParamStatusCheck(gpioCfg->validParams, (uint32_t)PMIC_POLARITY_VALID, status))
    {
        if (gpioCfg->polarity > PMIC_POLARITY_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
        else
        {
            Pmic_setBitField(&regData, PMIC_NINT_GPI_POL_SHIFT, PMIC_NINT_GPI_POL_MASK, gpioCfg->polarity);
        }
    }

    // Set NINT_GPI functionality
    if (Pmic_validParamStatusCheck(gpioCfg->validParams, (uint32_t)PMIC_FUNCTIONALITY_VALID, status))
    {
        if (gpioCfg->functionality > PMIC_NINT_GPI_FUNCTIONALITY_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
        else
        {
            Pmic_setBitField(&regData, PMIC_NINT_GPI_SEL_SHIFT, PMIC_NINT_GPI_SEL_MASK, gpioCfg->functionality);
        }
    }

    // Write FUNC_CONF
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioTxByte(handle, PMIC_FUNC_CONF_REG, regData);
    }
    Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);

    return status;
}

int32_t Pmic_gpioSetCfg(const Pmic_Handle_t *handle, uint8_t gpioPin, const Pmic_GpioCfg_t *gpioCfg)
{
    Pmic_GpioCfg_t localCfg;
    int32_t status = Pmic_checkHandle(handle);
    const uint32_t gpioValidParams = PMIC_POLARITY_VALID | PMIC_FUNCTIONALITY_VALID;
    const uint32_t nIntGpiValidParams = PMIC_PU_PD_CFG_VALID | PMIC_OD_PP_CFG_VALID | PMIC_POLARITY_VALID | PMIC_FUNCTIONALITY_VALID;

    if ((status == PMIC_ST_SUCCESS) && (gpioCfg == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) && (gpioCfg->validParams == 0U))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) && (gpioPin != PMIC_GPIO) && (gpioPin != PMIC_NINT_GPI))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        GPIO_copyGpioCfg(gpioCfg, &localCfg);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        if ((gpioPin == PMIC_GPIO) && Pmic_validParamCheck(localCfg.validParams, gpioValidParams))
        {
            status = GPIO_setGPIOCfg(handle, &localCfg);
        }
        else if ((gpioPin == PMIC_NINT_GPI) && Pmic_validParamCheck(localCfg.validParams, nIntGpiValidParams))
        {
            status = GPIO_setNINTGPICfg(handle, &localCfg);
        }
        else
        {
            status = PMIC_ST_ERR_FAIL;
        }
    }

    return Pmic_logStatus(handle, status);
}

/** @brief Get PMIC GPIO configuration (the GPIO pin that's not nINT_GPI) */
static int32_t GPIO_getGPIOCfg(const Pmic_Handle_t *handle, Pmic_GpioCfg_t *gpioCfg)
{
    uint8_t regData = 0U;

    // Read INTERFACE_CONF
    int32_t status = Pmic_ioRxByte_CS(handle, PMIC_INTERFACE_CONF_REG, &regData);

    if (status == PMIC_ST_SUCCESS)
    {
        // Get GPIO polarity
        if (Pmic_validParamCheck(gpioCfg->validParams, PMIC_POLARITY_VALID))
        {
            gpioCfg->polarity = Pmic_getBitField(regData, PMIC_GPIO_POL_SHIFT, PMIC_GPIO_POL_MASK);
        }

        // Get GPIO functionality
        if (Pmic_validParamCheck(gpioCfg->validParams, PMIC_FUNCTIONALITY_VALID))
        {
            gpioCfg->functionality = Pmic_getBitField(regData, PMIC_GPIO_SEL_SHIFT, PMIC_GPIO_SEL_MASK);
        }
    }

    return status;
}

/** @brief Get NINT_GPI configuration */
static int32_t GPIO_getNINTGPICfg(const Pmic_Handle_t *handle, Pmic_GpioCfg_t *gpioCfg)
{
    uint8_t regData = 0U;

    // Read FUNC_CONF
    int32_t status = Pmic_ioRxByte_CS(handle, PMIC_FUNC_CONF_REG, &regData);

    if (status == PMIC_ST_SUCCESS)
    {
        // Get NINT_GPI resistor configuration
        if (Pmic_validParamCheck(gpioCfg->validParams, PMIC_PU_PD_CFG_VALID))
        {
            gpioCfg->puPdCfg = Pmic_getBitField(regData, PMIC_NINT_GPI_PU_PD_SEL_SHIFT, PMIC_NINT_GPI_PU_PD_SEL_MASK);
        }

        // Get NINT_GPI open-drain/push-pull output configuration
        if (Pmic_validParamCheck(gpioCfg->validParams, PMIC_OD_PP_CFG_VALID))
        {
            gpioCfg->odPpCfg = Pmic_getBitField(regData, PMIC_NINT_PP_OD_SHIFT, PMIC_NINT_PP_OD_MASK);
        }

        // Get NINT_GPI polarity
        if (Pmic_validParamCheck(gpioCfg->validParams, PMIC_POLARITY_VALID))
        {
            gpioCfg->polarity = Pmic_getBitField(regData, PMIC_NINT_GPI_POL_SHIFT, PMIC_NINT_GPI_POL_MASK);
        }

        // Get NINT_GPI functionality
        if (Pmic_validParamCheck(gpioCfg->validParams, PMIC_FUNCTIONALITY_VALID))
        {
            gpioCfg->functionality = Pmic_getBitField(regData, PMIC_NINT_GPI_SEL_SHIFT, PMIC_NINT_GPI_SEL_MASK);
            if (gpioCfg->functionality == PMIC_NINT_GPI_LPM_CTRL_MODE_INPUT_REPEATED)
            {
                gpioCfg->functionality = PMIC_NINT_GPI_LPM_CTRL_MODE_INPUT;
            }
        }
    }

    return status;
}

int32_t Pmic_gpioGetCfg(const Pmic_Handle_t *handle, uint8_t gpioPin, Pmic_GpioCfg_t *gpioCfg)
{
    Pmic_GpioCfg_t localCfg;
    int32_t status = Pmic_checkHandle(handle);
    const uint32_t gpioValidParams = PMIC_POLARITY_VALID | PMIC_FUNCTIONALITY_VALID;
    const uint32_t nIntGpiValidParams = PMIC_PU_PD_CFG_VALID | PMIC_OD_PP_CFG_VALID | PMIC_POLARITY_VALID | PMIC_FUNCTIONALITY_VALID;

    if ((status == PMIC_ST_SUCCESS) && (gpioCfg == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) && (gpioCfg->validParams == 0U))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) && (gpioPin != PMIC_GPIO) && (gpioPin != PMIC_NINT_GPI))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        GPIO_copyGpioCfg(gpioCfg, &localCfg);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        if ((gpioPin == PMIC_GPIO) && Pmic_validParamCheck(localCfg.validParams, gpioValidParams))
        {
            status = GPIO_getGPIOCfg(handle, &localCfg);
        }
        else if ((gpioPin == PMIC_NINT_GPI) && Pmic_validParamCheck(localCfg.validParams, nIntGpiValidParams))
        {
            status = GPIO_getNINTGPICfg(handle, &localCfg);
        }
        else
        {
            status = PMIC_ST_ERR_FAIL;
        }
    }

    if (status == PMIC_ST_SUCCESS)
    {
        GPIO_copyGpioCfg(&localCfg, gpioCfg);
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_gpioSetActivationState(const Pmic_Handle_t *handle, bool activate)
{
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    if (status == PMIC_ST_SUCCESS)
    {
        // Read INTERFACE_CONF
        Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);
        status = Pmic_ioRxByte(handle, PMIC_INTERFACE_CONF_REG, &regData);

        // Modify GPO_EN bit field; Write INTERFACE_CONF
        if (status == PMIC_ST_SUCCESS)
        {
            Pmic_setBitField_b(&regData, PMIC_GPO_EN_SHIFT, PMIC_GPO_EN_MASK, activate);
            status = Pmic_ioTxByte(handle, PMIC_INTERFACE_CONF_REG, regData);
        }
        Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_gpioActivate(const Pmic_Handle_t *handle)
{
    return Pmic_gpioSetActivationState(handle, (bool)true);
}

int32_t Pmic_gpioDeactivate(const Pmic_Handle_t *handle)
{
    return Pmic_gpioSetActivationState(handle, (bool)false);
}

int32_t Pmic_gpioGetActivationState(const Pmic_Handle_t *handle, bool *activated)
{
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    if ((status == PMIC_ST_SUCCESS) && (activated == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Read INTERFACE_CONF
        status = Pmic_ioRxByte_CS(handle, PMIC_INTERFACE_CONF_REG, &regData);

        // Get GPO_EN bit field
        if (status == PMIC_ST_SUCCESS)
        {
            *activated = Pmic_getBitField_b(regData, PMIC_GPO_EN_SHIFT);
        }
    }

    return Pmic_logStatus(handle, status);
}
