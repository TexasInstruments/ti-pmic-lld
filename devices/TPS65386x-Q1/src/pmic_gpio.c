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

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "pmic.h"
#include "pmic_common.h"

#include "pmic_gpio.h"
#include "pmic_io.h"
#include "regmap/core.h"
#include "regmap/gpio.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/**
 * @brief GPO1 has two bit fields that have the same functionality; namely,
 * bit field 2 has the same functionality as bit field 6. This define is to
 * indicate the duplicate bit field.
 */
#define GPO1_HIZ_DUPLICATE (6U)

/* ========================================================================== */
/*                        Interface Implementations                           */
/* ========================================================================== */
static inline void GPIO_copyGpioCfg(const Pmic_GpioCfg_t *src, Pmic_GpioCfg_t *dst) {
    (void)memmove((void *)dst, (const void *)src, sizeof(Pmic_GpioCfg_t));
}

static int32_t GPIO_applyGpi1_4Cfg(const Pmic_Handle_t *handle, const Pmic_GpioCfg_t *gpioCfg)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);
    status = Pmic_ioRxByte(handle, GPI_CFG_REG, &regData);

    if (Pmic_validParamStatusCheck(gpioCfg->validParams, PMIC_CFG_GPIO_GPI1_VALID, status))
    {
        Pmic_setBitField(&regData, GPI1_CFG_SHIFT, GPI1_CFG_MASK, gpioCfg->gpi1);
    }

    if (Pmic_validParamStatusCheck(gpioCfg->validParams, PMIC_CFG_GPIO_GPI4_VALID, status))
    {
        Pmic_setBitField(&regData, GPI4_CFG_SHIFT, GPI4_CFG_MASK, gpioCfg->gpi4);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioTxByte(handle, GPI_CFG_REG, regData);
    }
    Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);

    return status;
}

static int32_t GPIO_setCfgGpi1_4(const Pmic_Handle_t *handle, const Pmic_GpioCfg_t *gpioCfg)
{
    int32_t status = PMIC_ST_SUCCESS;

    if (Pmic_validParamStatusCheck(gpioCfg->validParams, PMIC_CFG_GPIO_GPI1_VALID, status))
    {
        if (gpioCfg->gpi1 > PMIC_GPI1_CFG_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
    }

    if (Pmic_validParamStatusCheck(gpioCfg->validParams, PMIC_CFG_GPIO_GPI4_VALID, status))
    {
        if (gpioCfg->gpi4 > PMIC_GPI4_CFG_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
    }

    if (status == PMIC_ST_SUCCESS)
    {
        status = GPIO_applyGpi1_4Cfg(handle, gpioCfg);
    }

    return status;
}

static int32_t GPIO_applyGpo1_2Cfg(const Pmic_Handle_t *handle, const Pmic_GpioCfg_t *gpioCfg)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);
    status = Pmic_ioRxByte(handle, GPO_CFG1_REG, &regData);

    if (Pmic_validParamStatusCheck(gpioCfg->validParams, PMIC_CFG_GPIO_GPO1_VALID, status))
    {
        Pmic_setBitField(&regData, GPO1_CFG_SHIFT, GPO1_CFG_MASK, gpioCfg->gpo1);
    }

    if (Pmic_validParamStatusCheck(gpioCfg->validParams, PMIC_CFG_GPIO_GPO2_VALID, status))
    {
        Pmic_setBitField(&regData, GPO2_CFG_SHIFT, GPO2_CFG_MASK, gpioCfg->gpo2);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioTxByte(handle, GPO_CFG1_REG, regData);
    }
    Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);

    return status;
}

static int32_t GPIO_setCfgGpo1_2(const Pmic_Handle_t *handle, const Pmic_GpioCfg_t *gpioCfg)
{
    int32_t status = PMIC_ST_SUCCESS;

    if (Pmic_validParamStatusCheck(gpioCfg->validParams, PMIC_CFG_GPIO_GPO1_VALID, status))
    {
        if (gpioCfg->gpo1 > PMIC_GPO1_CFG_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
    }

    if (Pmic_validParamStatusCheck(gpioCfg->validParams, PMIC_CFG_GPIO_GPO2_VALID, status))
    {
        if (gpioCfg->gpo2 > PMIC_GPO2_CFG_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
    }

    if (status == PMIC_ST_SUCCESS)
    {
        status = GPIO_applyGpo1_2Cfg(handle, gpioCfg);
    }

    return status;
}

static int32_t GPIO_applyGpo3_4Cfg(const Pmic_Handle_t *handle, const Pmic_GpioCfg_t *gpioCfg)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);
    status = Pmic_ioRxByte(handle, GPO_CFG2_REG, &regData);

    if (Pmic_validParamStatusCheck(gpioCfg->validParams, PMIC_CFG_GPIO_GPO3_VALID, status))
    {
        Pmic_setBitField(&regData, GPO3_CFG_SHIFT, GPO3_CFG_MASK, gpioCfg->gpo3);
    }

    if (Pmic_validParamStatusCheck(gpioCfg->validParams, PMIC_CFG_GPIO_GPO4_VALID, status))
    {
        Pmic_setBitField(&regData, GPO4_CFG_SHIFT, GPO4_CFG_MASK, gpioCfg->gpo4);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioTxByte(handle, GPO_CFG2_REG, regData);
    }
    Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);

    return status;
}

static int32_t GPIO_setCfgGpo3_4(const Pmic_Handle_t *handle, const Pmic_GpioCfg_t *gpioCfg)
{
    int32_t status = PMIC_ST_SUCCESS;

    if (Pmic_validParamStatusCheck(gpioCfg->validParams, PMIC_CFG_GPIO_GPO3_VALID, status))
    {
        if (gpioCfg->gpo3 > PMIC_GPO3_CFG_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
    }

    if (Pmic_validParamStatusCheck(gpioCfg->validParams, PMIC_CFG_GPIO_GPO4_VALID, status))
    {
        if (gpioCfg->gpo4 > PMIC_GPO4_CFG_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
    }

    if (status == PMIC_ST_SUCCESS)
    {
        status = GPIO_applyGpo3_4Cfg(handle, gpioCfg);
    }

    return status;
}

int32_t Pmic_gpioSetCfg(const Pmic_Handle_t *handle, const Pmic_GpioCfg_t *gpioCfg)
{
    int32_t status = Pmic_checkHandle(handle);
    Pmic_GpioCfg_t localGpioCfg = (Pmic_GpioCfg_t){0};

    if ((status == PMIC_ST_SUCCESS) && (gpioCfg == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) && (gpioCfg->validParams == 0U))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        GPIO_copyGpioCfg(gpioCfg, &localGpioCfg);
    }

    // Set GPI1 and GPI4 configurations
    if ((status == PMIC_ST_SUCCESS) &&
        ((Pmic_validParamCheck(localGpioCfg.validParams, PMIC_CFG_GPIO_GPI1_VALID)) ||
         (Pmic_validParamCheck(localGpioCfg.validParams, PMIC_CFG_GPIO_GPI4_VALID))))
    {
        status = GPIO_setCfgGpi1_4(handle, &localGpioCfg);
    }

    // Set GPO1 and GPO2 configurations
    if ((status == PMIC_ST_SUCCESS) &&
        ((Pmic_validParamCheck(localGpioCfg.validParams, PMIC_CFG_GPIO_GPO1_VALID)) ||
         (Pmic_validParamCheck(localGpioCfg.validParams, PMIC_CFG_GPIO_GPO2_VALID))))
    {
        status = GPIO_setCfgGpo1_2(handle, &localGpioCfg);
    }

    // Set GPO3 and GPO4 configurations
    if ((status == PMIC_ST_SUCCESS) &&
        ((Pmic_validParamCheck(localGpioCfg.validParams, PMIC_CFG_GPIO_GPO3_VALID)) ||
         (Pmic_validParamCheck(localGpioCfg.validParams, PMIC_CFG_GPIO_GPO4_VALID))))
    {
        status = GPIO_setCfgGpo3_4(handle, &localGpioCfg);
    }

    return Pmic_logStatus(handle, status);
}

static int32_t GPIO_getCfgGpi1_4(const Pmic_Handle_t *handle, Pmic_GpioCfg_t *gpioCfg)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    // Read GPI_CFG register
    status = Pmic_ioRxByte_CS(handle, GPI_CFG_REG, &regData);

    if (status == PMIC_ST_SUCCESS)
    {
        // Get GPI1 configuration
        if (Pmic_validParamCheck(gpioCfg->validParams, PMIC_CFG_GPIO_GPI1_VALID))
        {
            gpioCfg->gpi1 = Pmic_getBitField(regData, GPI1_CFG_SHIFT, GPI1_CFG_MASK);
        }

        // Get GPI4 configuration
        if (Pmic_validParamCheck(gpioCfg->validParams, PMIC_CFG_GPIO_GPI4_VALID))
        {
            gpioCfg->gpi4 = Pmic_getBitField(regData, GPI4_CFG_SHIFT, GPI4_CFG_MASK);
        }
    }

    return status;
}

static int32_t GPIO_getCfgGpo1_2(const Pmic_Handle_t *handle, Pmic_GpioCfg_t *gpioCfg)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    // Read GPO_CFG1 register
    status = Pmic_ioRxByte_CS(handle, GPO_CFG1_REG, &regData);

    if (status == PMIC_ST_SUCCESS)
    {
        // Get GPO1 configuration
        if (Pmic_validParamCheck(gpioCfg->validParams, PMIC_CFG_GPIO_GPO1_VALID))
        {
            gpioCfg->gpo1 = Pmic_getBitField(regData, GPO1_CFG_SHIFT, GPO1_CFG_MASK);

            // Account for duplicate functionality
            if (gpioCfg->gpo1 == GPO1_HIZ_DUPLICATE)
            {
                gpioCfg->gpo1 = PMIC_GPO1_HIZ;
            }
        }

        // Get GPO2 configuration
        if (Pmic_validParamCheck(gpioCfg->validParams, PMIC_CFG_GPIO_GPO2_VALID))
        {
            gpioCfg->gpo2 = Pmic_getBitField(regData, GPO2_CFG_SHIFT, GPO2_CFG_MASK);
        }
    }

    return status;
}

static int32_t GPIO_getCfgGpo3_4(const Pmic_Handle_t *handle, Pmic_GpioCfg_t *gpioCfg)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    // Read GPO_CFG2 register
    Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);
    status = Pmic_ioRxByte(handle, GPO_CFG2_REG, &regData);
    Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);

    if (status == PMIC_ST_SUCCESS)
    {
        // Get GPO3 configuration
        if (Pmic_validParamCheck(gpioCfg->validParams, PMIC_CFG_GPIO_GPO3_VALID))
        {
            gpioCfg->gpo3 = Pmic_getBitField(regData, GPO3_CFG_SHIFT, GPO3_CFG_MASK);
        }

        // Get GPO4 configuration
        if (Pmic_validParamCheck(gpioCfg->validParams, PMIC_CFG_GPIO_GPO4_VALID))
        {
            gpioCfg->gpo4 = Pmic_getBitField(regData, GPO4_CFG_SHIFT, GPO4_CFG_MASK);
        }
    }

    return status;
}

static int32_t GPIO_getAllCfg(const Pmic_Handle_t *handle, Pmic_GpioCfg_t *gpioCfg)
{
    int32_t status = PMIC_ST_SUCCESS;

    if ((status == PMIC_ST_SUCCESS) && /* DA_JUSTIFY: PMICDRV-2356 */
        ((Pmic_validParamCheck(gpioCfg->validParams, PMIC_CFG_GPIO_GPI1_VALID)) ||
         (Pmic_validParamCheck(gpioCfg->validParams, PMIC_CFG_GPIO_GPI4_VALID))))
    {
        status = GPIO_getCfgGpi1_4(handle, gpioCfg);
    }

    if ((status == PMIC_ST_SUCCESS) &&
        ((Pmic_validParamCheck(gpioCfg->validParams, PMIC_CFG_GPIO_GPO1_VALID)) ||
         (Pmic_validParamCheck(gpioCfg->validParams, PMIC_CFG_GPIO_GPO2_VALID))))
    {
        status = GPIO_getCfgGpo1_2(handle, gpioCfg);
    }

    if ((status == PMIC_ST_SUCCESS) &&
        ((Pmic_validParamCheck(gpioCfg->validParams, PMIC_CFG_GPIO_GPO3_VALID)) ||
         (Pmic_validParamCheck(gpioCfg->validParams, PMIC_CFG_GPIO_GPO4_VALID))))
    {
        status = GPIO_getCfgGpo3_4(handle, gpioCfg);
    }

    return status;
}

int32_t Pmic_gpioGetCfg(const Pmic_Handle_t *handle, Pmic_GpioCfg_t *gpioCfg)
{
    int32_t status = Pmic_checkHandle(handle);
    Pmic_GpioCfg_t localGpioCfg = (Pmic_GpioCfg_t){0};

    if ((status == PMIC_ST_SUCCESS) && (gpioCfg == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) && (gpioCfg->validParams == 0U))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        GPIO_copyGpioCfg(gpioCfg, &localGpioCfg);
        status = GPIO_getAllCfg(handle, &localGpioCfg);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        GPIO_copyGpioCfg(&localGpioCfg, gpioCfg);
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_gpioGetOutputValue(const Pmic_Handle_t *handle, uint8_t gpo, bool *high)
{
    uint8_t regData = 0U;
    int32_t status = Pmic_checkHandle(handle);

    if ((status == PMIC_ST_SUCCESS) && ((gpo < PMIC_GPO_MIN) || (gpo > PMIC_GPO_MAX)))
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
        status = Pmic_ioRxByte_CS(handle, RDBK_LVL_STAT_REG, &regData);
    }

    // Extract GPO value
    if (status == PMIC_ST_SUCCESS)
    {
        *high = Pmic_getBitField_b(regData, GPO1_RDBK_LVL_SHIFT + (gpo - PMIC_GPO1));
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_gpioSetSafeOutCfg(const Pmic_Handle_t *handle, const Pmic_GpioSafeOutCfg_t *config) {
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    // Validate validParams: must have at least one valid bit
    if ((status == PMIC_ST_SUCCESS) && (config->validParams == 0U)) {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    if (status == PMIC_ST_SUCCESS) {
        Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);
        status = Pmic_ioRxByte(handle, PMIC_BIST_CTRL_REG, &regData);

        if (Pmic_validParamStatusCheck(config->validParams, PMIC_CFG_GPIO_SAFEOUT1_EN_VALID, status)) {
            Pmic_setBitField_b(&regData, PMIC_SAFE_OUT1_EN_SHIFT, config->safeOut1En);
        }

        if (Pmic_validParamStatusCheck(config->validParams, PMIC_CFG_GPIO_SAFEOUT2_EN_VALID, status)) {
            Pmic_setBitField_b(&regData, PMIC_SAFE_OUT2_EN_SHIFT, config->safeOut2En);
        }

        if (status == PMIC_ST_SUCCESS) {
            status = Pmic_ioTxByte(handle, PMIC_BIST_CTRL_REG, regData);
        }
        Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);
    }

    return status;
}

int32_t Pmic_gpioGetSafeOutCfg(const Pmic_Handle_t *handle, Pmic_GpioSafeOutCfg_t *config) {
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    if ((status == PMIC_ST_SUCCESS) && (config == NULL)) {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if (status == PMIC_ST_SUCCESS) {
        status = Pmic_ioRxByte_CS(handle, PMIC_BIST_CTRL_REG, &regData);
    }

    if (status == PMIC_ST_SUCCESS) {
        config->safeOut1En = Pmic_getBitField_b(regData, PMIC_SAFE_OUT1_EN_SHIFT);
        config->safeOut2En = Pmic_getBitField_b(regData, PMIC_SAFE_OUT2_EN_SHIFT);
    }

    return status;
}
