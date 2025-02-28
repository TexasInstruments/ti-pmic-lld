/******************************************************************************
 * Copyright (c) 2024 Texas Instruments Incorporated - http://www.ti.com
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

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#include "pmic.h"
#include "pmic_common.h"

#include "pmic_gpio.h"
#include "pmic_io.h"
#include "regmap/gpio.h"

/* ========================================================================== */
/*                        Interface Implementations                           */
/* ========================================================================== */

static int32_t GPIO_setCfgGpi1_4(Pmic_CoreHandle_t *handle, const Pmic_GpioCfg_t *gpioCfg)
{
    uint8_t regData = 0U;

    // Start critical section; read GPI_CFG register
    Pmic_criticalSectionStart(handle);
    int32_t status = Pmic_ioRxByte(handle, GPI_CFG_REG, &regData);

    // Set GPI1 configuration
    if (Pmic_validParamStatusCheck(gpioCfg->validParams, PMIC_CFG_GPI1_VALID, status))
    {
        if (gpioCfg->gpi1 > PMIC_GPI1_CFG_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
        else
        {
            Pmic_setBitField(&regData, GPI1_CFG_SHIFT, GPI1_CFG_MASK, gpioCfg->gpi1);
        }
    }

    // Set GPI4 configuration
    if (Pmic_validParamStatusCheck(gpioCfg->validParams, PMIC_CFG_GPI4_VALID, status))
    {
        if (gpioCfg->gpi4 > PMIC_GPI4_CFG_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
        else
        {
            Pmic_setBitField(&regData, GPI4_CFG_SHIFT, GPI4_CFG_MASK, gpioCfg->gpi4);
        }
    }

    // Write new register value back to PMIC; stop critical section
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioTxByte(handle, GPI_CFG_REG, regData);
    }
    Pmic_criticalSectionStop(handle);

    return status;
}

static int32_t GPIO_setCfgGpo1_2(Pmic_CoreHandle_t *handle, const Pmic_GpioCfg_t *gpioCfg)
{
    uint8_t regData = 0U;

    // Start critical section; read GPO_CFG1 register
    Pmic_criticalSectionStart(handle);
    int32_t status = Pmic_ioRxByte(handle, GPO_CFG1_REG, &regData);

    // Set GPO1 configuration
    if (Pmic_validParamStatusCheck(gpioCfg->validParams, PMIC_CFG_GPO1_VALID, status))
    {
        if (gpioCfg->gpo1 > PMIC_GPO1_CFG_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
        else
        {
            Pmic_setBitField(&regData, GPO1_CFG_SHIFT, GPO1_CFG_MASK, gpioCfg->gpo1);
        }
    }

    // Set GPO2 configuration
    if (Pmic_validParamStatusCheck(gpioCfg->validParams, PMIC_CFG_GPO2_VALID, status))
    {
        if (gpioCfg->gpo2 > PMIC_GPO2_CFG_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
        else
        {
            Pmic_setBitField(&regData, GPO2_CFG_SHIFT, GPO2_CFG_MASK, gpioCfg->gpo2);
        }
    }

    // Write new register value back to PMIC; stop critical section
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioTxByte(handle, GPO_CFG1_REG, regData);
    }
    Pmic_criticalSectionStop(handle);

    return status;
}

static int32_t GPIO_setCfgGpo3_4(Pmic_CoreHandle_t *handle, const Pmic_GpioCfg_t *gpioCfg)
{
    uint8_t regData = 0U;

    // Start critical section; read GPO_CFG2 register
    Pmic_criticalSectionStart(handle);
    int32_t status = Pmic_ioRxByte(handle,GPO_CFG2_REG, &regData);

    // Set GPO3 configuration
    if (Pmic_validParamStatusCheck(gpioCfg->validParams, PMIC_CFG_GPO3_VALID, status))
    {
        if (gpioCfg->gpo3 > PMIC_GPO3_CFG_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
        else
        {
            Pmic_setBitField(&regData, GPO3_CFG_SHIFT, GPO3_CFG_MASK, gpioCfg->gpo3);
        }
    }

    // Set GPO4 configuration
    if (Pmic_validParamStatusCheck(gpioCfg->validParams, PMIC_CFG_GPO4_VALID, status))
    {
        if (gpioCfg->gpo4 > PMIC_GPO4_CFG_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
        else
        {
            Pmic_setBitField(&regData, GPO4_CFG_SHIFT, GPO4_CFG_MASK, gpioCfg->gpo4);
        }
    }

    // Write new register value back to PMIC; stop critical section
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioTxByte(handle, GPO_CFG2_REG, regData);
    }
    Pmic_criticalSectionStop(handle);

    return status;
}

int32_t Pmic_gpioSetCfg(Pmic_CoreHandle_t *handle, const Pmic_GpioCfg_t *gpioCfg)
{
    int32_t status = Pmic_checkPmicCoreHandle(handle);

    if ((status == PMIC_ST_SUCCESS) && (gpioCfg == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) && (gpioCfg->validParams == 0U))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    // Set GPI1 and GPI4 configurations
    if ((status == PMIC_ST_SUCCESS) &&
        ((Pmic_validParamCheck(gpioCfg->validParams, PMIC_CFG_GPI1_VALID)) ||
         (Pmic_validParamCheck(gpioCfg->validParams, PMIC_CFG_GPI4_VALID))))
    {
        status = GPIO_setCfgGpi1_4(handle, gpioCfg);
    }

    // Set GPO1 and GPO2 configurations
    if ((status == PMIC_ST_SUCCESS) &&
        ((Pmic_validParamCheck(gpioCfg->validParams, PMIC_CFG_GPO1_VALID)) ||
         (Pmic_validParamCheck(gpioCfg->validParams, PMIC_CFG_GPO2_VALID))))
    {
        status = GPIO_setCfgGpo1_2(handle, gpioCfg);
    }

    // Set GPO3 and GPO4 configurations
    if ((status == PMIC_ST_SUCCESS) &&
        ((Pmic_validParamCheck(gpioCfg->validParams, PMIC_CFG_GPO3_VALID)) ||
         (Pmic_validParamCheck(gpioCfg->validParams, PMIC_CFG_GPO4_VALID))))
    {
        status = GPIO_setCfgGpo3_4(handle, gpioCfg);
    }

    return status;
}

static int32_t GPIO_getCfgGpi1_4(Pmic_CoreHandle_t *handle, Pmic_GpioCfg_t *gpioCfg)
{
    uint8_t regData = 0U;

    // Read GPI_CFG register
    Pmic_criticalSectionStart(handle);
    int32_t status = Pmic_ioRxByte(handle, GPI_CFG_REG, &regData);
    Pmic_criticalSectionStop(handle);

    if (status == PMIC_ST_SUCCESS)
    {
        // Get GPI1 configuration
        if (Pmic_validParamCheck(gpioCfg->validParams, PMIC_CFG_GPI1_VALID))
        {
            gpioCfg->gpi1 = Pmic_getBitField(regData, GPI1_CFG_SHIFT, GPI1_CFG_MASK);
        }

        // Get GPI4 configuration
        if (Pmic_validParamCheck(gpioCfg->validParams, PMIC_CFG_GPI4_VALID))
        {
            gpioCfg->gpi4 = Pmic_getBitField(regData, GPI4_CFG_SHIFT, GPI4_CFG_MASK);
        }
    }

    return status;
}

static int32_t GPIO_getCfgGpo1_2(Pmic_CoreHandle_t *handle, Pmic_GpioCfg_t *gpioCfg)
{
    uint8_t regData = 0U;

    // Read GPO_CFG1 register
    Pmic_criticalSectionStart(handle);
    int32_t status = Pmic_ioRxByte(handle, GPO_CFG1_REG, &regData);
    Pmic_criticalSectionStop(handle);

    if (status == PMIC_ST_SUCCESS)
    {
        // Get GPO1 configuration
        if (Pmic_validParamCheck(gpioCfg->validParams, PMIC_CFG_GPO1_VALID))
        {
            gpioCfg->gpo1 = Pmic_getBitField(regData, GPO1_CFG_SHIFT, GPO1_CFG_MASK);
        }

        // Get GPO2 configuration
        if (Pmic_validParamCheck(gpioCfg->validParams, PMIC_CFG_GPO2_VALID))
        {
            gpioCfg->gpo2 = Pmic_getBitField(regData, GPO2_CFG_SHIFT, GPO2_CFG_MASK);
        }
    }

    return status;
}

static int32_t GPIO_getCfgGpo3_4(Pmic_CoreHandle_t *handle, Pmic_GpioCfg_t *gpioCfg)
{
    uint8_t regData = 0U;

    // Read GPO_CFG2 register
    Pmic_criticalSectionStart(handle);
    int32_t status = Pmic_ioRxByte(handle, GPO_CFG2_REG, &regData);
    Pmic_criticalSectionStop(handle);

    if (status == PMIC_ST_SUCCESS)
    {
        // Get GPO3 configuration
        if (Pmic_validParamCheck(gpioCfg->validParams, PMIC_CFG_GPO3_VALID))
        {
            gpioCfg->gpo3 = Pmic_getBitField(regData, GPO3_CFG_SHIFT, GPO3_CFG_MASK);
        }

        // Get GPO4 configuration
        if (Pmic_validParamCheck(gpioCfg->validParams, PMIC_CFG_GPO4_VALID))
        {
            gpioCfg->gpo4 = Pmic_getBitField(regData, GPO4_CFG_SHIFT, GPO4_CFG_MASK);
        }
    }

    return status;
}

int32_t Pmic_gpioGetCfg(Pmic_CoreHandle_t *handle, Pmic_GpioCfg_t *gpioCfg)
{
    int32_t status = Pmic_checkPmicCoreHandle(handle);

    if ((status == PMIC_ST_SUCCESS) && (gpioCfg == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) && (gpioCfg->validParams == 0U))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    // Get GPI1 and GPI4 configurations
    if ((status == PMIC_ST_SUCCESS) &&
        ((Pmic_validParamCheck(gpioCfg->validParams, PMIC_CFG_GPI1_VALID)) ||
         (Pmic_validParamCheck(gpioCfg->validParams, PMIC_CFG_GPI4_VALID))))
    {
        status = GPIO_getCfgGpi1_4(handle, gpioCfg);
    }

    // Get GPO1 and GPO2 configurations
    if ((status == PMIC_ST_SUCCESS) &&
        ((Pmic_validParamCheck(gpioCfg->validParams, PMIC_CFG_GPO1_VALID)) ||
         (Pmic_validParamCheck(gpioCfg->validParams, PMIC_CFG_GPO2_VALID))))
    {
        status = GPIO_getCfgGpo1_2(handle, gpioCfg);
    }

    // Get GPO3 and GPO4 configurations
    if ((status == PMIC_ST_SUCCESS) &&
        ((Pmic_validParamCheck(gpioCfg->validParams, PMIC_CFG_GPO3_VALID)) ||
         (Pmic_validParamCheck(gpioCfg->validParams, PMIC_CFG_GPO4_VALID))))
    {
        status = GPIO_getCfgGpo3_4(handle, gpioCfg);
    }

    return status;
}

int32_t Pmic_gpioGetOutputVal(Pmic_CoreHandle_t *handle, uint8_t gpo, bool *high)
{
    uint8_t regData = 0U;
    int32_t status = Pmic_checkPmicCoreHandle(handle);

    if ((status == PMIC_ST_SUCCESS) && (gpo > PMIC_GPO_MAX))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) && (high == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    // Read RDBK_LVL_STAT register
    if (status == PMIC_ST_SUCCESS)
    {
        Pmic_criticalSectionStart(handle);
        status = Pmic_ioRxByte(handle, RDBK_LVL_STAT_REG, &regData);
        Pmic_criticalSectionStop(handle);
    }

    // Extract GPO value
    if (status == PMIC_ST_SUCCESS)
    {
        *high = Pmic_getBitField_b(regData, GPO1_RDBK_LVL_SHIFT + (gpo - 1U));
    }

    return status;
}
