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
/*                         Static Function Declarations                       */
/* ========================================================================== */

static int32_t GPIO_validatePinNum(uint8_t pinNum);
static int32_t GPIO_setPinCfgFields(const Pmic_GpioPinCfg_t *gpioPinCfg, uint8_t *regData);
static int32_t GPIO_getPinCfgFields(const uint8_t regData, Pmic_GpioPinCfg_t *gpioPinCfg);
static uint8_t GPIO_getConfRegAddr(uint8_t pinNum);
static uint8_t GPIO_getOutShift(uint8_t pinNum);
static uint8_t GPIO_getOutMask(uint8_t pinNum);
static uint8_t GPIO_getInShift(uint8_t pinNum);

/* ========================================================================== */
/*                         Static Function Definitions                        */
/* ========================================================================== */

/* Validate GPIO pin number. Returns PMIC_ST_SUCCESS if valid,
 * PMIC_ST_ERR_INV_PARAM otherwise. */
static int32_t GPIO_validatePinNum(uint8_t pinNum)
{
    int32_t status = PMIC_ST_SUCCESS;

    if ((pinNum < PMIC_GPIO_PIN_MIN) || (pinNum > PMIC_GPIO_PIN_MAX))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    return status;
}

/* Get GPIO configuration register address for a given pin (1-6). */
static uint8_t GPIO_getConfRegAddr(uint8_t pinNum)
{
    return (uint8_t)(GPIO1_CONF_REG + (pinNum - PMIC_GPIO_PIN1));
}

/* Get GPIO output bit shift for a given pin (1-6). */
static uint8_t GPIO_getOutShift(uint8_t pinNum)
{
    return (uint8_t)(GPIO1_OUT_SHIFT + (pinNum - PMIC_GPIO_PIN1));
}

/* Get GPIO output bit mask for a given pin (1-6). */
static uint8_t GPIO_getOutMask(uint8_t pinNum)
{
    return (uint8_t)(0x01U << GPIO_getOutShift(pinNum));
}

/* Get GPIO input bit shift for a given pin (1-6). */
static uint8_t GPIO_getInShift(uint8_t pinNum)
{
    return (uint8_t)(GPIO1_IN_SHIFT + (pinNum - PMIC_GPIO_PIN1));
}

/* Set GPIO pin configuration fields in register data. Returns PMIC_ST_SUCCESS
 * if successful, error code otherwise. */
static int32_t GPIO_setPinCfgFields(const Pmic_GpioPinCfg_t *gpioPinCfg, uint8_t *regData)
{
    int32_t status = PMIC_ST_SUCCESS;

    // Set function select
    if (Pmic_validParamStatusCheck(gpioPinCfg->validParams, PMIC_GPIO_FXN_SEL_VALID, status))
    {
        // Validate function select based on pin number
        bool validFxn = false;

        switch (gpioPinCfg->pinNum)
        {
            case PMIC_GPIO_PIN1:
                validFxn = (gpioPinCfg->fxnSel <= PMIC_GPIO_PIN1_FXN_SEL_MAX);
                break;
            case PMIC_GPIO_PIN2:
                validFxn = (gpioPinCfg->fxnSel <= PMIC_GPIO_PIN2_FXN_SEL_MAX);
                break;
            case PMIC_GPIO_PIN3:
                validFxn = (gpioPinCfg->fxnSel <= PMIC_GPIO_PIN3_FXN_SEL_MAX);
                break;
            case PMIC_GPIO_PIN4:
                validFxn = (gpioPinCfg->fxnSel <= PMIC_GPIO_PIN4_FXN_SEL_MAX);
                break;
            case PMIC_GPIO_PIN5:
                validFxn = (gpioPinCfg->fxnSel <= PMIC_GPIO_PIN5_FXN_SEL_MAX);
                break;
            case PMIC_GPIO_PIN6:
                validFxn = (gpioPinCfg->fxnSel <= PMIC_GPIO_PIN6_FXN_SEL_MAX);
                break;
            default:
                validFxn = false;
                break;
        }

        if (!validFxn)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
        else
        {
            Pmic_setBitField(regData, GPIO_SEL_SHIFT, GPIO_SEL_MASK, gpioPinCfg->fxnSel);
        }
    }

    // Set pull-up/pull-down selection
    if (Pmic_validParamStatusCheck(gpioPinCfg->validParams, PMIC_GPIO_PU_SEL_VALID, status))
    {
        if (gpioPinCfg->puSel > PMIC_GPIO_PIN_PU_SEL_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
        else
        {
            Pmic_setBitField(regData, GPIO_PU_SEL_SHIFT, GPIO_PU_SEL_MASK, gpioPinCfg->puSel);
        }
    }

    // Set GPIO type (push-pull or open-drain)
    if (Pmic_validParamStatusCheck(gpioPinCfg->validParams, PMIC_GPIO_TYPE_VALID, status))
    {
        if (gpioPinCfg->type > PMIC_GPIO_PIN_TYPE_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
        else
        {
            Pmic_setBitField(regData, GPIO_OD_SHIFT, GPIO_OD_MASK, gpioPinCfg->type);
        }
    }

    // Set GPIO direction
    if (Pmic_validParamStatusCheck(gpioPinCfg->validParams, PMIC_GPIO_DIR_VALID, status))
    {
        if (gpioPinCfg->dir > PMIC_GPIO_PIN_DIR_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
        else
        {
            Pmic_setBitField(regData, GPIO_DIR_SHIFT, GPIO_DIR_MASK, gpioPinCfg->dir);
        }
    }

    // Set deglitch enable
    if (Pmic_validParamStatusCheck(gpioPinCfg->validParams, PMIC_GPIO_DEGL_EN_VALID, status))
    {
        Pmic_setBitField(regData, GPIO_DEGLITCH_EN_SHIFT, GPIO_DEGLITCH_EN_MASK,
                        (uint8_t)(gpioPinCfg->deglEn ? 1U : 0U));
    }

    // Set resistor enable
    if (Pmic_validParamStatusCheck(gpioPinCfg->validParams, PMIC_GPIO_RESISTOR_EN_VALID, status))
    {
        Pmic_setBitField(regData, GPIO_PU_PD_EN_SHIFT, GPIO_PU_PD_EN_MASK,
                        (uint8_t)(gpioPinCfg->resistorEn ? 1U : 0U));
    }

    return status;
}

/* Get GPIO pin configuration fields from register data. Returns PMIC_ST_SUCCESS
 * if successful, error code otherwise. */
static int32_t GPIO_getPinCfgFields(const uint8_t regData, Pmic_GpioPinCfg_t *gpioPinCfg)
{
    int32_t status = PMIC_ST_SUCCESS;

    // Get function select
    if (Pmic_validParamCheck(gpioPinCfg->validParams, PMIC_GPIO_FXN_SEL_VALID))
    {
        gpioPinCfg->fxnSel = Pmic_getBitField(regData, GPIO_SEL_SHIFT, GPIO_SEL_MASK);
    }

    // Get pull-up/pull-down selection
    if (Pmic_validParamCheck(gpioPinCfg->validParams, PMIC_GPIO_PU_SEL_VALID))
    {
        gpioPinCfg->puSel = Pmic_getBitField(regData, GPIO_PU_SEL_SHIFT, GPIO_PU_SEL_MASK);
    }

    // Get GPIO type
    if (Pmic_validParamCheck(gpioPinCfg->validParams, PMIC_GPIO_TYPE_VALID))
    {
        gpioPinCfg->type = Pmic_getBitField(regData, GPIO_OD_SHIFT, GPIO_OD_MASK);
    }

    // Get GPIO direction
    if (Pmic_validParamCheck(gpioPinCfg->validParams, PMIC_GPIO_DIR_VALID))
    {
        gpioPinCfg->dir = Pmic_getBitField(regData, GPIO_DIR_SHIFT, GPIO_DIR_MASK);
    }

    // Get deglitch enable
    if (Pmic_validParamCheck(gpioPinCfg->validParams, PMIC_GPIO_DEGL_EN_VALID))
    {
        gpioPinCfg->deglEn = Pmic_getBitField_b(regData, GPIO_DEGLITCH_EN_SHIFT);
    }

    // Get resistor enable
    if (Pmic_validParamCheck(gpioPinCfg->validParams, PMIC_GPIO_RESISTOR_EN_VALID))
    {
        gpioPinCfg->resistorEn = Pmic_getBitField_b(regData, GPIO_PU_PD_EN_SHIFT);
    }

    return status;
}

/* ========================================================================== */
/*                        Interface Implementations                           */
/* ========================================================================== */

int32_t Pmic_gpioSetPinCfg(const Pmic_Handle_t *handle, const Pmic_GpioPinCfg_t *gpioPinCfg)
{
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;
    uint8_t regAddr = 0U;

    // Validate input parameters
    if ((status == PMIC_ST_SUCCESS) && (gpioPinCfg == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) && (gpioPinCfg->validParams == 0U))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    // Validate pin number
    if (status == PMIC_ST_SUCCESS)
    {
        status = GPIO_validatePinNum(gpioPinCfg->pinNum);
    }

    // Get register address for this GPIO pin
    if (status == PMIC_ST_SUCCESS)
    {
        regAddr = GPIO_getConfRegAddr(gpioPinCfg->pinNum);
    }

    // Start critical section and read current register value
    if (status == PMIC_ST_SUCCESS)
    {
        Pmic_criticalSectionStart(handle);
        status = Pmic_ioRxByte(handle, regAddr, &regData);
    }

    // Modify register data with new configuration
    if (status == PMIC_ST_SUCCESS)
    {
        status = GPIO_setPinCfgFields(gpioPinCfg, &regData);
    }

    // Write modified register data back to PMIC
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioTxByte(handle, regAddr, regData);
    }

    // End critical section
    Pmic_criticalSectionStop(handle);

    return status;
}

int32_t Pmic_gpioGetPinCfg(const Pmic_Handle_t *handle, Pmic_GpioPinCfg_t *gpioPinCfg)
{
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;
    uint8_t regAddr = 0U;

    // Validate input parameters
    if ((status == PMIC_ST_SUCCESS) && (gpioPinCfg == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) && (gpioPinCfg->validParams == 0U))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    // Validate pin number
    if (status == PMIC_ST_SUCCESS)
    {
        status = GPIO_validatePinNum(gpioPinCfg->pinNum);
    }

    // Get register address for this GPIO pin
    if (status == PMIC_ST_SUCCESS)
    {
        regAddr = GPIO_getConfRegAddr(gpioPinCfg->pinNum);
    }

    // Read register value
    if (status == PMIC_ST_SUCCESS)
    {
        Pmic_criticalSectionStart(handle);
        status = Pmic_ioRxByte(handle, regAddr, &regData);
        Pmic_criticalSectionStop(handle);
    }

    // Extract configuration fields from register data
    if (status == PMIC_ST_SUCCESS)
    {
        status = GPIO_getPinCfgFields(regData, gpioPinCfg);
    }

    return status;
}

int32_t Pmic_gpioSetPinVal(const Pmic_Handle_t *handle, uint8_t gpioPin, bool high)
{
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;
    uint8_t bitShift = 0U;
    uint8_t bitMask = 0U;

    // Validate pin number
    if (status == PMIC_ST_SUCCESS)
    {
        status = GPIO_validatePinNum(gpioPin);
    }

    // Get bit position for this GPIO pin
    if (status == PMIC_ST_SUCCESS)
    {
        bitShift = GPIO_getOutShift(gpioPin);
        bitMask = GPIO_getOutMask(gpioPin);
    }

    // Start critical section and read current GPIO_OUT register
    if (status == PMIC_ST_SUCCESS)
    {
        Pmic_criticalSectionStart(handle);
        status = Pmic_ioRxByte(handle, GPIO_OUT_1_REG, &regData);
    }

    // Set or clear the bit for this GPIO
    if (status == PMIC_ST_SUCCESS)
    {
        Pmic_setBitField(&regData, bitShift, bitMask, (uint8_t)(high ? 1U : 0U));
        status = Pmic_ioTxByte(handle, GPIO_OUT_1_REG, regData);
    }

    // End critical section
    Pmic_criticalSectionStop(handle);

    return status;
}

int32_t Pmic_gpioGetPinVal(const Pmic_Handle_t *handle, uint8_t gpioPin, bool *high)
{
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;
    uint8_t bitShift = 0U;

    // Validate input parameters
    if ((status == PMIC_ST_SUCCESS) && (high == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    // Validate pin number
    if (status == PMIC_ST_SUCCESS)
    {
        status = GPIO_validatePinNum(gpioPin);
    }

    // Get bit position for this GPIO pin
    if (status == PMIC_ST_SUCCESS)
    {
        bitShift = GPIO_getInShift(gpioPin);
    }

    // Read GPIO_IN register
    if (status == PMIC_ST_SUCCESS)
    {
        Pmic_criticalSectionStart(handle);
        status = Pmic_ioRxByte(handle, GPIO_IN_1_REG, &regData);
        Pmic_criticalSectionStop(handle);
    }

    // Extract pin value
    if (status == PMIC_ST_SUCCESS)
    {
        *high = Pmic_getBitField_b(regData, bitShift);
    }

    return status;
}

int32_t Pmic_gpioSetNIntEnDrvCfg(const Pmic_Handle_t *handle, const Pmic_GpioNIntEnDrvCfg_t *nIntEnDrvCfg)
{
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    // Validate input parameters
    if ((status == PMIC_ST_SUCCESS) && (nIntEnDrvCfg == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) && (nIntEnDrvCfg->validParams == 0U))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    // Start critical section and read POWER_ON_CONFIG register
    if (status == PMIC_ST_SUCCESS)
    {
        Pmic_criticalSectionStart(handle);
        status = Pmic_ioRxByte(handle, POWER_ON_CONFIG_REG, &regData);
    }

    // Set function select
    if (Pmic_validParamStatusCheck(nIntEnDrvCfg->validParams, PMIC_GPIO_NINT_ENDRV_FXN_SEL_VALID, status))
    {
        if (nIntEnDrvCfg->fxnSel > PMIC_GPIO_NINT_ENDRV_FXN_SEL_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
        else
        {
            Pmic_setBitField(&regData, NINT_ENDRV_SEL_SHIFT, NINT_ENDRV_SEL_MASK, nIntEnDrvCfg->fxnSel);
        }
    }

    // Set pull-up resistor enable
    if (Pmic_validParamStatusCheck(nIntEnDrvCfg->validParams, PMIC_GPIO_NINT_ENDRV_EN_PU_RESISTOR_VALID, status))
    {
        Pmic_setBitField(&regData, NINT_ENDRV_PU_SEL_SHIFT, NINT_ENDRV_PU_SEL_MASK,
                        (uint8_t)(nIntEnDrvCfg->enPuResistor ? 1U : 0U));
    }

    // Write modified register data back to PMIC
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioTxByte(handle, POWER_ON_CONFIG_REG, regData);
    }

    // End critical section
    Pmic_criticalSectionStop(handle);

    return status;
}

int32_t Pmic_gpioGetNIntEnDrvCfg(const Pmic_Handle_t *handle, Pmic_GpioNIntEnDrvCfg_t *nIntEnDrvCfg)
{
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    // Validate input parameters
    if ((status == PMIC_ST_SUCCESS) && (nIntEnDrvCfg == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) && (nIntEnDrvCfg->validParams == 0U))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    // Read POWER_ON_CONFIG register
    if (status == PMIC_ST_SUCCESS)
    {
        Pmic_criticalSectionStart(handle);
        status = Pmic_ioRxByte(handle, POWER_ON_CONFIG_REG, &regData);
        Pmic_criticalSectionStop(handle);
    }

    // Extract configuration fields
    if (status == PMIC_ST_SUCCESS)
    {
        // Get function select
        if (Pmic_validParamCheck(nIntEnDrvCfg->validParams, PMIC_GPIO_NINT_ENDRV_FXN_SEL_VALID))
        {
            nIntEnDrvCfg->fxnSel = Pmic_getBitField(regData, NINT_ENDRV_SEL_SHIFT, NINT_ENDRV_SEL_MASK);
        }

        // Get pull-up resistor enable
        if (Pmic_validParamCheck(nIntEnDrvCfg->validParams, PMIC_GPIO_NINT_ENDRV_EN_PU_RESISTOR_VALID))
        {
            nIntEnDrvCfg->enPuResistor = Pmic_getBitField_b(regData, NINT_ENDRV_PU_SEL_SHIFT);
        }
    }

    return status;
}

int32_t Pmic_gpioGetNIntEnDrvVal(const Pmic_Handle_t *handle, bool *high)
{
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    // Validate input parameters
    if ((status == PMIC_ST_SUCCESS) && (high == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    // Read ENABLE_DRV_STAT register
    if (status == PMIC_ST_SUCCESS)
    {
        Pmic_criticalSectionStart(handle);
        status = Pmic_ioRxByte(handle, ENABLE_DRV_STAT_REG, &regData);
        Pmic_criticalSectionStop(handle);
    }

    // Extract nINT/EN_DRV pin value
    if (status == PMIC_ST_SUCCESS)
    {
        *high = Pmic_getBitField_b(regData, NINT_EN_DRV_IN_SHIFT);
    }

    return status;
}

int32_t Pmic_gpioSetEnPbVSenseCfg(const Pmic_Handle_t *handle, const Pmic_GpioNIntEnDrvCfg_t *enPbVSenseCfg)
{
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    // Validate input parameters
    if ((status == PMIC_ST_SUCCESS) && (enPbVSenseCfg == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) && (enPbVSenseCfg->validParams == 0U))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    // Start critical section and read POWER_ON_CONFIG register
    if (status == PMIC_ST_SUCCESS)
    {
        Pmic_criticalSectionStart(handle);
        status = Pmic_ioRxByte(handle, POWER_ON_CONFIG_REG, &regData);
    }

    // Set function select
    if (Pmic_validParamStatusCheck(enPbVSenseCfg->validParams, PMIC_GPIO_EN_PB_VSENSE_FXN_SEL_VALID, status))
    {
        if (enPbVSenseCfg->fxnSel > PMIC_GPIO_EN_PB_VSENSE_FXN_SEL_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
        else
        {
            Pmic_setBitField(&regData, EN_PB_VSENSE_CONFIG_SHIFT, EN_PB_VSENSE_CONFIG_MASK, enPbVSenseCfg->fxnSel);
        }
    }

    // Set EN/PB deglitch configuration
    if (Pmic_validParamStatusCheck(enPbVSenseCfg->validParams, PMIC_GPIO_EN_PB_VSENSE_EN_PB_DEGL_VALID, status))
    {
        // Validate deglitch value based on function
        bool validDegl = true;

        // Get current or new function selection to validate deglitch
        uint8_t fxnSel = Pmic_getBitField(regData, EN_PB_VSENSE_CONFIG_SHIFT, EN_PB_VSENSE_CONFIG_MASK);
        if (Pmic_validParamCheck(enPbVSenseCfg->validParams, PMIC_GPIO_EN_PB_VSENSE_FXN_SEL_VALID))
        {
            fxnSel = enPbVSenseCfg->fxnSel;
        }

        if (fxnSel == PMIC_GPIO_EN_PB_VSENSE_FXN_SEL_ENABLE)
        {
            validDegl = (enPbVSenseCfg->enPbDegl <= PMIC_GPIO_EN_DEGL_MAX);
        }
        else if (fxnSel == PMIC_GPIO_EN_PB_VSENSE_FXN_SEL_PB)
        {
            validDegl = (enPbVSenseCfg->enPbDegl <= PMIC_GPIO_PB_DEGL_MAX);
        }

        if (!validDegl)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
        else
        {
            Pmic_setBitField(&regData, EN_PB_DEGL_SHIFT, EN_PB_DEGL_MASK, enPbVSenseCfg->enPbDegl);
        }
    }

    // Write modified register data back to PMIC
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioTxByte(handle, POWER_ON_CONFIG_REG, regData);
    }

    // End critical section
    Pmic_criticalSectionStop(handle);

    return status;
}

int32_t Pmic_gpioGetEnPbVSenseCfg(const Pmic_Handle_t *handle, Pmic_GpioNIntEnDrvCfg_t *enPbVSenseCfg)
{
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    // Validate input parameters
    if ((status == PMIC_ST_SUCCESS) && (enPbVSenseCfg == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) && (enPbVSenseCfg->validParams == 0U))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    // Read POWER_ON_CONFIG register
    if (status == PMIC_ST_SUCCESS)
    {
        Pmic_criticalSectionStart(handle);
        status = Pmic_ioRxByte(handle, POWER_ON_CONFIG_REG, &regData);
        Pmic_criticalSectionStop(handle);
    }

    // Extract configuration fields
    if (status == PMIC_ST_SUCCESS)
    {
        // Get function select
        if (Pmic_validParamCheck(enPbVSenseCfg->validParams, PMIC_GPIO_EN_PB_VSENSE_FXN_SEL_VALID))
        {
            enPbVSenseCfg->fxnSel = Pmic_getBitField(regData, EN_PB_VSENSE_CONFIG_SHIFT, EN_PB_VSENSE_CONFIG_MASK);
        }

        // Get EN/PB deglitch
        if (Pmic_validParamCheck(enPbVSenseCfg->validParams, PMIC_GPIO_EN_PB_VSENSE_EN_PB_DEGL_VALID))
        {
            enPbVSenseCfg->enPbDegl = Pmic_getBitField(regData, EN_PB_DEGL_SHIFT, EN_PB_DEGL_MASK);
        }
    }

    return status;
}

int32_t Pmic_gpioGetEnPbVSenseStatus(const Pmic_Handle_t *handle, Pmic_GpioEnPbVSenseStatus_t *enPbVSenseStatus)
{
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    // Validate input parameters
    if ((status == PMIC_ST_SUCCESS) && (enPbVSenseStatus == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) && (enPbVSenseStatus->validParams == 0U))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    // Read ENABLE_DRV_STAT register
    if (status == PMIC_ST_SUCCESS)
    {
        Pmic_criticalSectionStart(handle);
        status = Pmic_ioRxByte(handle, ENABLE_DRV_STAT_REG, &regData);
        Pmic_criticalSectionStop(handle);
    }

    // Note: The ENABLE_DRV_STAT register doesn't appear to have separate status bits
    // for PB, EN, and VSENSE in the register map provided. This implementation
    // may need to be adjusted based on actual hardware behavior or additional
    // register definitions.

    if (status == PMIC_ST_SUCCESS)
    {
        // Set all status fields to indicate that the information is not available
        // in the current register map. This may need hardware clarification.
        if (Pmic_validParamCheck(enPbVSenseStatus->validParams, PMIC_GPIO_PB_LVL_HIGH_VALID))
        {
            enPbVSenseStatus->pbLvlHigh = false;
        }

        if (Pmic_validParamCheck(enPbVSenseStatus->validParams, PMIC_GPIO_EN_LVL_HIGH_VALID))
        {
            enPbVSenseStatus->enLvlHigh = false;
        }

        if (Pmic_validParamCheck(enPbVSenseStatus->validParams, PMIC_GPIO_VSENSE_LVL_HIGH_VALID))
        {
            enPbVSenseStatus->vsenseLvlHigh = false;
        }
    }

    return status;
}

int32_t Pmic_gpioGetNRstOutVal(const Pmic_Handle_t *handle, bool *high)
{
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    // Validate input parameters
    if ((status == PMIC_ST_SUCCESS) && (high == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    // Read ENABLE_DRV_STAT register
    if (status == PMIC_ST_SUCCESS)
    {
        Pmic_criticalSectionStart(handle);
        status = Pmic_ioRxByte(handle, ENABLE_DRV_STAT_REG, &regData);
        Pmic_criticalSectionStop(handle);
    }

    // Extract nRSTOUT pin value
    if (status == PMIC_ST_SUCCESS)
    {
        *high = Pmic_getBitField_b(regData, NRSTOUT_IN_SHIFT);
    }

    return status;
}
