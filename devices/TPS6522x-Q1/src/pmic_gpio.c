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

#include "pmic.h"
#include "pmic_common.h"

#include "pmic_gpio.h"
#include "pmic_io.h"
#include "regmap/gpio.h"

#include <string.h>

/* ========================================================================== */
/*                         Static Function Declarations                       */
/* ========================================================================== */

/**
 * @brief Copy Pmic_GpioPinCfg_t structure member-wise
 */
static inline void GPIO_copyGpioPinCfg(const Pmic_GpioPinCfg_t *src, Pmic_GpioPinCfg_t *dst)
{
    (void)memmove((void *)dst, (const void *)src, sizeof(Pmic_GpioPinCfg_t));
}

/**
 * @brief Copy Pmic_GpioNIntEnDrvCfg_t structure member-wise
 */
static inline void GPIO_copyGpioNIntEnDrvCfg(const Pmic_GpioNIntEnDrvCfg_t *src, Pmic_GpioNIntEnDrvCfg_t *dst)
{
    (void)memmove((void *)dst, (const void *)src, sizeof(Pmic_GpioNIntEnDrvCfg_t));
}

/**
 * @brief Copy Pmic_GpioEnPbVSenseStatus_t structure member-wise
 */
static inline void GPIO_copyGpioEnPbVSenseStatus(const Pmic_GpioEnPbVSenseStatus_t *src, Pmic_GpioEnPbVSenseStatus_t *dst)
{
    (void)memmove((void *)dst, (const void *)src, sizeof(Pmic_GpioEnPbVSenseStatus_t));
}

static int32_t GPIO_validatePinNum(uint8_t pinNum);
static int32_t GPIO_setPinCfgFields(const Pmic_GpioPinCfg_t *gpioPinCfg, uint8_t *regData);
static int32_t GPIO_getPinCfgFields(const uint8_t regData, Pmic_GpioPinCfg_t *gpioPinCfg);
static void GPIO_getConfRegAddr(uint8_t pinNum, uint8_t *regAddr);
static void GPIO_getOutShift(uint8_t pinNum, uint8_t *bitShift);
static void GPIO_getOutMask(uint8_t pinNum, uint8_t *bitMask);
static void GPIO_getInShift(uint8_t pinNum, uint8_t *bitShift);

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
static void GPIO_getConfRegAddr(uint8_t pinNum, uint8_t *regAddr)
{
    const uint8_t offset = ((pinNum >= PMIC_GPIO_PIN1) && (pinNum <= PMIC_GPIO_PIN_MAX))
                           ? (uint8_t)(pinNum - PMIC_GPIO_PIN1)
                           : 0U;
    *regAddr = (uint8_t)(GPIO1_CONF_REG + offset);
}

/* Get GPIO output bit shift for a given pin (1-6). */
static void GPIO_getOutShift(uint8_t pinNum, uint8_t *bitShift)
{
    *bitShift = (uint8_t)(GPIO1_OUT_SHIFT + (pinNum - PMIC_GPIO_PIN1));
}

/* Get GPIO output bit mask for a given pin (1-6). */
static void GPIO_getOutMask(uint8_t pinNum, uint8_t *bitMask)
{
    uint8_t bitShift = 0U;
    GPIO_getOutShift(pinNum, &bitShift);
    *bitMask = (uint8_t)(0x01UL << bitShift);
}

/* Get GPIO input bit shift for a given pin (1-6). */
static void GPIO_getInShift(uint8_t pinNum, uint8_t *bitShift)
{
    *bitShift = (uint8_t)(GPIO1_IN_SHIFT + (pinNum - PMIC_GPIO_PIN1));
}

static bool GPIO_isFxnSelValid(uint8_t pinNum, uint8_t fxnSel)
{
    bool validFxn = (bool)false;

    switch (pinNum)
    {
        case PMIC_GPIO_PIN1:
            validFxn = (fxnSel <= PMIC_GPIO_PIN1_FXN_SEL_MAX);
            break;
        case PMIC_GPIO_PIN2:
            validFxn = (fxnSel <= PMIC_GPIO_PIN2_FXN_SEL_MAX);
            break;
        case PMIC_GPIO_PIN3:
            validFxn = (fxnSel <= PMIC_GPIO_PIN3_FXN_SEL_MAX);
            break;
        case PMIC_GPIO_PIN4:
            validFxn = (fxnSel <= PMIC_GPIO_PIN4_FXN_SEL_MAX);
            break;
        case PMIC_GPIO_PIN5:
            validFxn = (fxnSel <= PMIC_GPIO_PIN5_FXN_SEL_MAX);
            break;
        case PMIC_GPIO_PIN6:
            validFxn = (fxnSel <= PMIC_GPIO_PIN6_FXN_SEL_MAX);
            break;
        default: /* LCOV_EXCL_LINE */
            validFxn = (bool)false; /* LCOV_EXCL_LINE */
            break; /* LCOV_EXCL_LINE */
    }

    return validFxn;
}

static int32_t GPIO_setPuSelField(int32_t status, uint8_t *regData, const Pmic_GpioPinCfg_t *cfg)
{
    if (Pmic_validParamStatusCheck(cfg->validParams, PMIC_CFG_GPIO_PU_SEL_VALID, status))
    {
        if (cfg->puSel > PMIC_GPIO_PIN_PU_SEL_MAX) { return PMIC_ST_ERR_INV_PARAM; }
        Pmic_setBitField(regData, GPIO_PU_SEL_SHIFT, GPIO_PU_SEL_MASK, cfg->puSel);
    }

    return status;
}

static int32_t GPIO_setTypeField(int32_t status, uint8_t *regData, const Pmic_GpioPinCfg_t *cfg)
{
    if (Pmic_validParamStatusCheck(cfg->validParams, PMIC_CFG_GPIO_TYPE_VALID, status))
    {
        if (cfg->type > PMIC_GPIO_PIN_TYPE_MAX) { return PMIC_ST_ERR_INV_PARAM; }
        Pmic_setBitField(regData, GPIO_OD_SHIFT, GPIO_OD_MASK, cfg->type);
    }

    return status;
}

static int32_t GPIO_setDirField(int32_t status, uint8_t *regData, const Pmic_GpioPinCfg_t *cfg)
{
    if (Pmic_validParamStatusCheck(cfg->validParams, PMIC_CFG_GPIO_DIR_VALID, status))
    {
        if (cfg->dir > PMIC_GPIO_PIN_DIR_MAX) { return PMIC_ST_ERR_INV_PARAM; }
        Pmic_setBitField(regData, GPIO_DIR_SHIFT, GPIO_DIR_MASK, cfg->dir);
    }

    return status;
}

static int32_t GPIO_setNonFxnFields(const Pmic_GpioPinCfg_t *gpioPinCfg, uint8_t *regData)
{
    int32_t status = PMIC_ST_SUCCESS;

    status = GPIO_setPuSelField(status, regData, gpioPinCfg);
    status = GPIO_setTypeField(status, regData, gpioPinCfg);
    status = GPIO_setDirField(status, regData, gpioPinCfg);

    if (Pmic_validParamStatusCheck(gpioPinCfg->validParams, PMIC_CFG_GPIO_DEGL_EN_VALID, status))
    {
        Pmic_setBitField(regData, GPIO_DEGLITCH_EN_SHIFT, GPIO_DEGLITCH_EN_MASK,
                        (uint8_t)(gpioPinCfg->deglEn ? 1U : 0U));
    }

    if (Pmic_validParamStatusCheck(gpioPinCfg->validParams, PMIC_CFG_GPIO_RESISTOR_EN_VALID, status))
    {
        Pmic_setBitField(regData, GPIO_PU_PD_EN_SHIFT, GPIO_PU_PD_EN_MASK,
                        (uint8_t)(gpioPinCfg->resistorEn ? 1U : 0U));
    }

    return status;
}

/* Set GPIO pin configuration fields in register data. Returns PMIC_ST_SUCCESS
 * if successful, error code otherwise. */
static int32_t GPIO_setPinCfgFields(const Pmic_GpioPinCfg_t *gpioPinCfg, uint8_t *regData)
{
    int32_t status = PMIC_ST_SUCCESS;

    // Set function select
    if (Pmic_validParamStatusCheck(gpioPinCfg->validParams, PMIC_CFG_GPIO_FXN_SEL_VALID, status))
    {
        if (!GPIO_isFxnSelValid(gpioPinCfg->pinNum, gpioPinCfg->fxnSel))
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
        else
        {
            Pmic_setBitField(regData, GPIO_SEL_SHIFT, GPIO_SEL_MASK, gpioPinCfg->fxnSel);
        }
    }

    if (status == PMIC_ST_SUCCESS)
    {
        status = GPIO_setNonFxnFields(gpioPinCfg, regData);
    }

    return status;
}

/* Get GPIO pin configuration fields from register data. Returns PMIC_ST_SUCCESS
 * if successful, error code otherwise. */
static int32_t GPIO_getPinCfgFields(const uint8_t regData, Pmic_GpioPinCfg_t *gpioPinCfg)
{
    int32_t status = PMIC_ST_SUCCESS;

    // Get function select
    if (Pmic_validParamCheck(gpioPinCfg->validParams, PMIC_CFG_GPIO_FXN_SEL_VALID))
    {
        gpioPinCfg->fxnSel = Pmic_getBitField(regData, GPIO_SEL_SHIFT, GPIO_SEL_MASK);
    }

    // Get pull-up/pull-down selection
    if (Pmic_validParamCheck(gpioPinCfg->validParams, PMIC_CFG_GPIO_PU_SEL_VALID))
    {
        gpioPinCfg->puSel = Pmic_getBitField(regData, GPIO_PU_SEL_SHIFT, GPIO_PU_SEL_MASK);
    }

    // Get GPIO type
    if (Pmic_validParamCheck(gpioPinCfg->validParams, PMIC_CFG_GPIO_TYPE_VALID))
    {
        gpioPinCfg->type = Pmic_getBitField(regData, GPIO_OD_SHIFT, GPIO_OD_MASK);
    }

    // Get GPIO direction
    if (Pmic_validParamCheck(gpioPinCfg->validParams, PMIC_CFG_GPIO_DIR_VALID))
    {
        gpioPinCfg->dir = Pmic_getBitField(regData, GPIO_DIR_SHIFT, GPIO_DIR_MASK);
    }

    // Get deglitch enable
    if (Pmic_validParamCheck(gpioPinCfg->validParams, PMIC_CFG_GPIO_DEGL_EN_VALID))
    {
        gpioPinCfg->deglEn = Pmic_getBitField_b(regData, GPIO_DEGLITCH_EN_SHIFT);
    }

    // Get resistor enable
    if (Pmic_validParamCheck(gpioPinCfg->validParams, PMIC_CFG_GPIO_RESISTOR_EN_VALID))
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
    Pmic_GpioPinCfg_t gpioPinCfgLocal = (Pmic_GpioPinCfg_t){0};
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    uint8_t regAddr = 0U;

    status = Pmic_checkHandle(handle);
    if (status != PMIC_ST_SUCCESS)
    {
        return Pmic_logStatus(handle, status);
    }

    if (gpioPinCfg == NULL)
    {
        return Pmic_logStatus(handle, PMIC_ST_ERR_NULL_PARAM);
    }

    if (gpioPinCfg->validParams == 0U)
    {
        return Pmic_logStatus(handle, PMIC_ST_ERR_INV_PARAM);
    }

    // Validate pin number
    status = GPIO_validatePinNum((uint8_t)((uint8_t)gpioPinCfg->pinNum));
    if (status != PMIC_ST_SUCCESS)
    {
        return Pmic_logStatus(handle, status);
    }

    GPIO_copyGpioPinCfg(gpioPinCfg, &gpioPinCfgLocal);

    // Get register address for this GPIO pin
    GPIO_getConfRegAddr(gpioPinCfgLocal.pinNum, &regAddr);

    // Start critical section and read current register value
    Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);
    status = Pmic_ioRxByte(handle, regAddr, &regData);

    // Modify register data with new configuration
    if (status == PMIC_ST_SUCCESS)
    {
        status = GPIO_setPinCfgFields(&gpioPinCfgLocal, &regData);
    }

    // Write modified register data back to PMIC
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioTxByte(handle, regAddr, regData);
    }

    // End critical section
    Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_gpioGetPinCfg(const Pmic_Handle_t *handle, Pmic_GpioPinCfg_t *gpioPinCfg)
{
    Pmic_GpioPinCfg_t gpioPinCfgLocal = (Pmic_GpioPinCfg_t){0};
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    uint8_t regAddr = 0U;

    status = Pmic_checkHandle(handle);
    if (status != PMIC_ST_SUCCESS)
    {
        return Pmic_logStatus(handle, status);
    }

    if (gpioPinCfg == NULL)
    {
        return Pmic_logStatus(handle, PMIC_ST_ERR_NULL_PARAM);
    }

    if (gpioPinCfg->validParams == 0U)
    {
        return Pmic_logStatus(handle, PMIC_ST_ERR_INV_PARAM);
    }

    // Validate pin number
    status = GPIO_validatePinNum((uint8_t)((uint8_t)gpioPinCfg->pinNum));
    if (status != PMIC_ST_SUCCESS)
    {
        return Pmic_logStatus(handle, status);
    }

    GPIO_copyGpioPinCfg(gpioPinCfg, &gpioPinCfgLocal);

    // Get register address for this GPIO pin
    GPIO_getConfRegAddr(gpioPinCfgLocal.pinNum, &regAddr);

    // Read register value
    status = Pmic_ioRxByte_CS(handle, regAddr, &regData);

    // Extract configuration fields from register data
    if (status == PMIC_ST_SUCCESS)
    {
        status = GPIO_getPinCfgFields(regData, &gpioPinCfgLocal);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        GPIO_copyGpioPinCfg(&gpioPinCfgLocal, gpioPinCfg);
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_gpioSetPinVal(const Pmic_Handle_t *handle, uint8_t gpioPin, bool high)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    uint8_t bitShift = 0U;
    uint8_t bitMask = 0U;

    status = Pmic_checkHandle(handle);

    // Validate pin number
    if (status == PMIC_ST_SUCCESS)
    {
        status = GPIO_validatePinNum(gpioPin);
    }

    // Get bit position for this GPIO pin
    if (status == PMIC_ST_SUCCESS)
    {
        GPIO_getOutShift(gpioPin, &bitShift);
        GPIO_getOutMask(gpioPin, &bitMask);
    }

    // Start critical section and read current GPIO_OUT register
    Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioRxByte(handle, GPIO_OUT_1_REG, &regData);
    }

    // Set or clear the bit for this GPIO
    if (status == PMIC_ST_SUCCESS)
    {
        Pmic_setBitField(&regData, bitShift, bitMask, (uint8_t)(high ? 1U : 0U));
        status = Pmic_ioTxByte(handle, GPIO_OUT_1_REG, regData);
    }

    // End critical section
    Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_gpioGetPinVal(const Pmic_Handle_t *handle, uint8_t gpioPin, bool *high)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    uint8_t bitShift = 0U;

    status = Pmic_checkHandle(handle);

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
        GPIO_getInShift(gpioPin, &bitShift);
    }

    // Read GPIO_IN register
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioRxByte_CS(handle, GPIO_IN_1_REG, &regData);
    }

    // Extract pin value
    if (status == PMIC_ST_SUCCESS)
    {
        *high = Pmic_getBitField_b(regData, bitShift);
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_gpioSetNIntEnDrvCfg(const Pmic_Handle_t *handle, const Pmic_GpioNIntEnDrvCfg_t *nIntEnDrvCfg)
{
    Pmic_GpioNIntEnDrvCfg_t nIntEnDrvCfgLocal = (Pmic_GpioNIntEnDrvCfg_t){0};
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    if (status != PMIC_ST_SUCCESS)
    {
        return Pmic_logStatus(handle, status);
    }

    if (nIntEnDrvCfg == NULL)
    {
        return Pmic_logStatus(handle, PMIC_ST_ERR_NULL_PARAM);
    }

    if (nIntEnDrvCfg->validParams == 0U)
    {
        return Pmic_logStatus(handle, PMIC_ST_ERR_INV_PARAM);
    }

    GPIO_copyGpioNIntEnDrvCfg(nIntEnDrvCfg, &nIntEnDrvCfgLocal);

    // Start critical section and read POWER_ON_CONFIG register
    Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);
    status = Pmic_ioRxByte(handle, POWER_ON_CONFIG_REG, &regData);

    // Set function select
    if (Pmic_validParamStatusCheck(nIntEnDrvCfgLocal.validParams, PMIC_CFG_GPIO_NINT_ENDRV_FXN_SEL_VALID, status))
    {
        if (nIntEnDrvCfgLocal.fxnSel > PMIC_GPIO_NINT_ENDRV_FXN_SEL_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
        else
        {
            Pmic_setBitField(&regData, NINT_ENDRV_SEL_SHIFT, NINT_ENDRV_SEL_MASK, nIntEnDrvCfgLocal.fxnSel);
        }
    }

    // Set pull-up resistor enable
    if (Pmic_validParamStatusCheck(nIntEnDrvCfgLocal.validParams, PMIC_CFG_GPIO_NINT_ENDRV_EN_PU_RESISTOR_VALID, status))
    {
        Pmic_setBitField(&regData, NINT_ENDRV_PU_SEL_SHIFT, NINT_ENDRV_PU_SEL_MASK,
                        (uint8_t)(nIntEnDrvCfgLocal.enPuResistor ? 1U : 0U));
    }

    // Write modified register data back to PMIC
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioTxByte(handle, POWER_ON_CONFIG_REG, regData);
    }

    // End critical section
    Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_gpioGetNIntEnDrvCfg(const Pmic_Handle_t *handle, Pmic_GpioNIntEnDrvCfg_t *nIntEnDrvCfg)
{
    Pmic_GpioNIntEnDrvCfg_t nIntEnDrvCfgLocal = (Pmic_GpioNIntEnDrvCfg_t){0};
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    if (status != PMIC_ST_SUCCESS)
    {
        return Pmic_logStatus(handle, status);
    }

    if (nIntEnDrvCfg == NULL)
    {
        return Pmic_logStatus(handle, PMIC_ST_ERR_NULL_PARAM);
    }

    if (nIntEnDrvCfg->validParams == 0U)
    {
        return Pmic_logStatus(handle, PMIC_ST_ERR_INV_PARAM);
    }

    GPIO_copyGpioNIntEnDrvCfg(nIntEnDrvCfg, &nIntEnDrvCfgLocal);

    // Read POWER_ON_CONFIG register
    status = Pmic_ioRxByte_CS(handle, POWER_ON_CONFIG_REG, &regData);

    // Extract configuration fields
    if (status == PMIC_ST_SUCCESS)
    {
        // Get function select
        if (Pmic_validParamCheck(nIntEnDrvCfgLocal.validParams, PMIC_CFG_GPIO_NINT_ENDRV_FXN_SEL_VALID))
        {
            nIntEnDrvCfgLocal.fxnSel = Pmic_getBitField(regData, NINT_ENDRV_SEL_SHIFT, NINT_ENDRV_SEL_MASK);
        }

        // Get pull-up resistor enable
        if (Pmic_validParamCheck(nIntEnDrvCfgLocal.validParams, PMIC_CFG_GPIO_NINT_ENDRV_EN_PU_RESISTOR_VALID))
        {
            nIntEnDrvCfgLocal.enPuResistor = Pmic_getBitField_b(regData, NINT_ENDRV_PU_SEL_SHIFT);
        }

        GPIO_copyGpioNIntEnDrvCfg(&nIntEnDrvCfgLocal, nIntEnDrvCfg);
    }

    return Pmic_logStatus(handle, status);
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
        status = Pmic_ioRxByte_CS(handle, ENABLE_DRV_STAT_REG, &regData);
    }

    // Extract nINT/EN_DRV pin value
    if (status == PMIC_ST_SUCCESS)
    {
        *high = Pmic_getBitField_b(regData, NINT_EN_DRV_IN_SHIFT);
    }

    return Pmic_logStatus(handle, status);
}

static int32_t GPIO_validateAndSetEnPbDegl(uint8_t *regData, uint8_t fxnSel, uint8_t enPbDegl)
{
    bool validDegl = (bool)true;

    if (fxnSel == PMIC_GPIO_EN_PB_VSENSE_FN_ENABLE)
    {
        validDegl = (enPbDegl <= PMIC_GPIO_EN_DEGL_MAX);
    }
    else if (fxnSel == PMIC_GPIO_EN_PB_VSENSE_FN_PB)
    {
        validDegl = (enPbDegl <= PMIC_GPIO_PB_DEGL_MAX);
    }
    else
    {
        /* Function selection not requiring validation */
    }

    if (!validDegl)
    {
        return PMIC_ST_ERR_INV_PARAM;
    }

    Pmic_setBitField(regData, EN_PB_DEGL_SHIFT, EN_PB_DEGL_MASK, enPbDegl);
    return PMIC_ST_SUCCESS;
}

int32_t Pmic_gpioSetEnPbVSenseCfg(const Pmic_Handle_t *handle, const Pmic_GpioNIntEnDrvCfg_t *enPbVSenseCfg)
{
    Pmic_GpioNIntEnDrvCfg_t enPbVSenseCfgLocal = (Pmic_GpioNIntEnDrvCfg_t){0};
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    if (status != PMIC_ST_SUCCESS)
    {
        return Pmic_logStatus(handle, status);
    }

    if (enPbVSenseCfg == NULL)
    {
        return Pmic_logStatus(handle, PMIC_ST_ERR_NULL_PARAM);
    }

    if (enPbVSenseCfg->validParams == 0U)
    {
        return Pmic_logStatus(handle, PMIC_ST_ERR_INV_PARAM);
    }

    GPIO_copyGpioNIntEnDrvCfg(enPbVSenseCfg, &enPbVSenseCfgLocal);

    // Start critical section and read POWER_ON_CONFIG register
    Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);
    status = Pmic_ioRxByte(handle, POWER_ON_CONFIG_REG, &regData);

    // Set function select
    if (Pmic_validParamStatusCheck(enPbVSenseCfgLocal.validParams, PMIC_CFG_GPIO_EN_PB_VSENSE_FN_VALID, status))
    {
        if (enPbVSenseCfgLocal.fxnSel > PMIC_GPIO_EN_PB_VSENSE_FN_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
        else
        {
            Pmic_setBitField(&regData, EN_PB_VSENSE_CONFIG_SHIFT, EN_PB_VSENSE_CONFIG_MASK, enPbVSenseCfgLocal.fxnSel);
        }
    }

    // Set EN/PB deglitch configuration
    if (Pmic_validParamStatusCheck(enPbVSenseCfgLocal.validParams, PMIC_CFG_GPIO_EN_PB_VSENSE_EN_PB_DEGL_VALID, status))
    {
        uint8_t fxnSel = Pmic_getBitField(regData, EN_PB_VSENSE_CONFIG_SHIFT, EN_PB_VSENSE_CONFIG_MASK);
        if (Pmic_validParamCheck(enPbVSenseCfgLocal.validParams, PMIC_CFG_GPIO_EN_PB_VSENSE_FN_VALID))
        {
            fxnSel = enPbVSenseCfgLocal.fxnSel;
        }
        status = GPIO_validateAndSetEnPbDegl(&regData, fxnSel, enPbVSenseCfgLocal.enPbDegl);
    }

    // Write modified register data back to PMIC
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioTxByte(handle, POWER_ON_CONFIG_REG, regData);
    }

    // End critical section
    Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_gpioGetEnPbVSenseCfg(const Pmic_Handle_t *handle, Pmic_GpioNIntEnDrvCfg_t *enPbVSenseCfg)
{
    Pmic_GpioNIntEnDrvCfg_t enPbVSenseCfgLocal = (Pmic_GpioNIntEnDrvCfg_t){0};
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    if (status != PMIC_ST_SUCCESS)
    {
        return Pmic_logStatus(handle, status);
    }

    if (enPbVSenseCfg == NULL)
    {
        return Pmic_logStatus(handle, PMIC_ST_ERR_NULL_PARAM);
    }

    if (enPbVSenseCfg->validParams == 0U)
    {
        return Pmic_logStatus(handle, PMIC_ST_ERR_INV_PARAM);
    }

    GPIO_copyGpioNIntEnDrvCfg(enPbVSenseCfg, &enPbVSenseCfgLocal);

    // Read POWER_ON_CONFIG register
    status = Pmic_ioRxByte_CS(handle, POWER_ON_CONFIG_REG, &regData);

    // Extract configuration fields
    if (status == PMIC_ST_SUCCESS)
    {
        // Get function select
        if (Pmic_validParamCheck(enPbVSenseCfgLocal.validParams, PMIC_CFG_GPIO_EN_PB_VSENSE_FN_VALID))
        {
            enPbVSenseCfgLocal.fxnSel = Pmic_getBitField(regData, EN_PB_VSENSE_CONFIG_SHIFT, EN_PB_VSENSE_CONFIG_MASK);
        }

        // Get EN/PB deglitch
        if (Pmic_validParamCheck(enPbVSenseCfgLocal.validParams, PMIC_CFG_GPIO_EN_PB_VSENSE_EN_PB_DEGL_VALID))
        {
            enPbVSenseCfgLocal.enPbDegl = Pmic_getBitField(regData, EN_PB_DEGL_SHIFT, EN_PB_DEGL_MASK);
        }

        GPIO_copyGpioNIntEnDrvCfg(&enPbVSenseCfgLocal, enPbVSenseCfg);
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_gpioGetEnPbVSenseStatus(const Pmic_Handle_t *handle, Pmic_GpioEnPbVSenseStatus_t *enPbVSenseStatus)
{
    Pmic_GpioEnPbVSenseStatus_t enPbVSenseStatusLocal = (Pmic_GpioEnPbVSenseStatus_t){0};
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    if (status != PMIC_ST_SUCCESS)
    {
        return Pmic_logStatus(handle, status);
    }

    if (enPbVSenseStatus == NULL)
    {
        return Pmic_logStatus(handle, PMIC_ST_ERR_NULL_PARAM);
    }

    if (enPbVSenseStatus->validParams == 0U)
    {
        return Pmic_logStatus(handle, PMIC_ST_ERR_INV_PARAM);
    }

    GPIO_copyGpioEnPbVSenseStatus(enPbVSenseStatus, &enPbVSenseStatusLocal);

    // Read STAT_STARTUP register for EN/PB/VSENSE status
    // Status guaranteed SUCCESS here due to early returns above
    status = Pmic_ioRxByte_CS(handle, STAT_STARTUP_REG, &regData);

    if (status == PMIC_ST_SUCCESS)
    {
        if (Pmic_validParamCheck(enPbVSenseStatusLocal.validParams, PMIC_GPIO_PB_LVL_HIGH_VALID))
        {
            enPbVSenseStatusLocal.pbLvlHigh = ((regData & PB_LEVEL_STAT_MASK) != 0U);
        }

        if (Pmic_validParamCheck(enPbVSenseStatusLocal.validParams, PMIC_GPIO_EN_LVL_HIGH_VALID))
        {
            enPbVSenseStatusLocal.enLvlHigh = ((regData & ENABLE_STAT_MASK) != 0U);
        }

        if (Pmic_validParamCheck(enPbVSenseStatusLocal.validParams, PMIC_GPIO_VSENSE_LVL_HIGH_VALID))
        {
            enPbVSenseStatusLocal.vsenseLvlHigh = ((regData & VSENSE_STAT_MASK) != 0U);
        }

        GPIO_copyGpioEnPbVSenseStatus(&enPbVSenseStatusLocal, enPbVSenseStatus);
    }

    return Pmic_logStatus(handle, status);
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
        status = Pmic_ioRxByte_CS(handle, ENABLE_DRV_STAT_REG, &regData);
    }

    // Extract nRSTOUT pin value
    if (status == PMIC_ST_SUCCESS)
    {
        *high = Pmic_getBitField_b(regData, NRSTOUT_IN_SHIFT);
    }

    return Pmic_logStatus(handle, status);
}
