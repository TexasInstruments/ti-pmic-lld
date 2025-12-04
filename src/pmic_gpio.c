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
/**
 * @file pmic_gpio.c
 *
 * @brief GPIO module API definitions.
 */

/* ========================================================================== */
/*                              Include Files                                 */
/* ========================================================================== */

#include "pmic.h"

#include "regmap/gpio.h"
#include "regmap/core.h"

/* ========================================================================== */
/*                            Macros & Typedefs                               */
/* ========================================================================== */

/* ========================================================================== */
/*                           Function Definitions                             */
/* ========================================================================== */

int32_t Pmic_gpioSetPinEnableState(const Pmic_Handle_t *handle, uint8_t gpioPin, bool enable) {
    int32_t status = Pmic_checkHandle(handle);
    if (status != PMIC_ST_SUCCESS) {
        return status;
    }

    if (gpioPin > PMIC_GPIO_PIN_MAX) {
        return PMIC_ST_ERR_INV_PARAM;
    }

    const uint8_t shift = (gpioPin == PMIC_GPIO_GPO1) ? VMON1_GPO1_EN_SHIFT : GPO2_EN_SHIFT;
    status = Pmic_ioUpdateByte_bCS(handle, BLOCK_EN_CTRL_REG, shift, enable);

    return PMIC_ST_SUCCESS;
}

int32_t Pmic_gpioGetPinEnableState(const Pmic_Handle_t *handle, uint8_t gpioPin, bool *isEnabled) {
    uint8_t regData = 0U;

    int32_t status = Pmic_checkHandle(handle);
    if (status != PMIC_ST_SUCCESS) {
        return status;
    }

    if (gpioPin > PMIC_GPIO_PIN_MAX) {
        return PMIC_ST_ERR_INV_PARAM;
    }

    if (isEnabled == NULL) {
        return PMIC_ST_ERR_NULL_PARAM;
    }

    status = Pmic_ioRxByte_CS(handle, BLOCK_EN_CTRL_REG, &regData);
    if (status != PMIC_ST_SUCCESS) {
        return status;
    }

    const uint8_t shift = (gpioPin == PMIC_GPIO_GPO1) ? VMON1_GPO1_EN_SHIFT : GPO2_EN_SHIFT;
    *isEnabled = Pmic_getBitField_b(regData, shift);
    return PMIC_ST_SUCCESS;
}

static int32_t GPIO_setPrimaryCfg(const Pmic_Handle_t *handle, const Pmic_GpioPinCfg_t *gpioPinCfg) {
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    Pmic_criticalSectionStart(handle);
    status = Pmic_ioRxByte(handle, GPO_CONF_REG, &regData);

    if (Pmic_validParamStatusCheck(gpioPinCfg->validParams, PMIC_GPIO_PIN_FAULT_POLARITY_VALID, status)) {
        const uint8_t polShift = (gpioPinCfg->pin == PMIC_GPIO_GPO2) ? GPO2_FAULT2_POL_SHIFT : GPO1_FAULT1_POL_SHIFT;
        const uint8_t polMask = (gpioPinCfg->pin == PMIC_GPIO_GPO2) ? GPO2_FAULT2_POL_MASK : GPO1_FAULT1_POL_MASK;

        if (gpioPinCfg->faultPolarity > PMIC_GPIO_POLARITY_MAX) {
            status = PMIC_ST_ERR_INV_PARAM;
        } else {
            Pmic_setBitField(&regData, polShift, polMask, gpioPinCfg->faultPolarity);
        }
    }

    if (Pmic_validParamStatusCheck(gpioPinCfg->validParams, PMIC_GPIO_PIN_FAULT_TYPE_VALID, status)) {
        const uint8_t typeShift = (gpioPinCfg->pin == PMIC_GPIO_GPO2) ? GPO2_FAULT2_OD_SHIFT : GPO1_FAULT1_OD_SHIFT;
        const uint8_t typeMask = (gpioPinCfg->pin == PMIC_GPIO_GPO2) ? GPO2_FAULT2_OD_MASK : GPO1_FAULT1_OD_MASK;

        if (gpioPinCfg->faultType > PMIC_GPIO_TYPE_MAX) {
            status = PMIC_ST_ERR_INV_PARAM;
        } else {
            Pmic_setBitField(&regData, typeShift, typeMask, gpioPinCfg->faultType);
        }
    }

    if (Pmic_validParamStatusCheck(gpioPinCfg->validParams, PMIC_GPIO_PIN_FXN_SEL_VALID, status)) {
        const uint8_t fxnSelShift = (gpioPinCfg->pin == PMIC_GPIO_GPO2) ? GPO2_SEL_SHIFT : GPO1_SEL_SHIFT;
        const uint8_t fxnSelMask = (gpioPinCfg->pin == PMIC_GPIO_GPO2) ? GPO2_SEL_MASK :  GPO1_SEL_MASK;
        const uint8_t fxnSelMax = (gpioPinCfg->pin == PMIC_GPIO_GPO2) ? PMIC_GPIO_FXN_GPO_2_MAX : PMIC_GPIO_FXN_GPO_1_MAX;

        if (gpioPinCfg->fxnSel > fxnSelMax) {
            status = PMIC_ST_ERR_INV_PARAM;
        } else {
            Pmic_setBitField(&regData, fxnSelShift, fxnSelMask, gpioPinCfg->fxnSel);
        }
    }

    if (status == PMIC_ST_SUCCESS) {
        status = Pmic_ioTxByte(handle, GPO_CONF_REG, regData);
    }
    Pmic_criticalSectionStop(handle);

    return status;
}

static int32_t GPIO_setStartupShutdownDelay(const Pmic_Handle_t *handle, const Pmic_GpioPinCfg_t *gpioPinCfg) {
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regAddr = (gpioPinCfg->pin == PMIC_GPIO_GPO1) ? VMON1_GPO1_SEQUENCE_REG : GPO2_SEQUENCE_REG;

    Pmic_criticalSectionStart(handle);
    status = Pmic_ioRxByte(handle, regAddr, &regData);

    if (Pmic_validParamStatusCheck(gpioPinCfg->validParams, PMIC_GPIO_PIN_STARTUP_DELAY_VALID, status)) {
        if (gpioPinCfg->startupDelay > PMIC_GPIO_DELAY_MAX) {
            status = PMIC_ST_ERR_INV_PARAM;
        } else {
            Pmic_setBitField(&regData, STARTUP_DELAY_SHIFT, STARTUP_DELAY_MASK, gpioPinCfg->startupDelay);
        }
    }

    if (Pmic_validParamStatusCheck(gpioPinCfg->validParams, PMIC_GPIO_PIN_SHUTDOWN_DELAY_VALID, status)) {
        if (gpioPinCfg->shutdownDelay > PMIC_GPIO_DELAY_MAX) {
            status = PMIC_ST_ERR_INV_PARAM;
        } else {
            Pmic_setBitField(&regData, SHUTDOWN_DELAY_SHIFT, SHUTDOWN_DELAY_MASK, gpioPinCfg->shutdownDelay);
        }
    }

    if (status == PMIC_ST_SUCCESS) {
        status = Pmic_ioTxByte(handle, regAddr, regData);
    }
    Pmic_criticalSectionStop(handle);

    return status;
}

int32_t Pmic_gpioSetPinCfg(const Pmic_Handle_t *handle, const Pmic_GpioPinCfg_t *gpioPinCfg) {
    const uint32_t primaryCfgValid = (\
        PMIC_GPIO_PIN_FXN_SEL_VALID |
        PMIC_GPIO_PIN_FAULT_POLARITY_VALID |
        PMIC_GPIO_PIN_FAULT_TYPE_VALID);
    const uint32_t startupShutdownDelayValid = (\
        PMIC_GPIO_PIN_STARTUP_DELAY_VALID |
        PMIC_GPIO_PIN_SHUTDOWN_DELAY_VALID);

    int32_t status = Pmic_checkHandle(handle);
    if (status != PMIC_ST_SUCCESS) {
        return status;
    }

    if (gpioPinCfg == NULL) {
        return PMIC_ST_ERR_NULL_PARAM;
    }

    if ((gpioPinCfg->validParams == 0U) || (gpioPinCfg->pin > PMIC_GPIO_PIN_MAX)) {
        return PMIC_ST_ERR_INV_PARAM;
    }

    if (Pmic_validParamCheck(gpioPinCfg->validParams, primaryCfgValid)) {
        status = GPIO_setPrimaryCfg(handle, gpioPinCfg);
        if (status != PMIC_ST_SUCCESS) {
            return status;
        }
    }

    if (Pmic_validParamCheck(gpioPinCfg->validParams, startupShutdownDelayValid)) {
        status = GPIO_setStartupShutdownDelay(handle, gpioPinCfg);
        if (status != PMIC_ST_SUCCESS) {
            return status;
        }
    }

    return PMIC_ST_SUCCESS;
}

static int32_t GPIO_getPrimaryCfg(const Pmic_Handle_t *handle, Pmic_GpioPinCfg_t *gpioPinCfg) {
    uint8_t regData = 0U;

    int32_t status = Pmic_ioRxByte_CS(handle, GPO_CONF_REG, &regData);
    if (status != PMIC_ST_SUCCESS) {
        return status;
    }

    if (Pmic_validParamCheck(gpioPinCfg->validParams, PMIC_GPIO_PIN_FAULT_POLARITY_VALID)) {
        const uint8_t polShift = (gpioPinCfg->pin == PMIC_GPIO_GPO2) ? GPO2_FAULT2_POL_SHIFT : GPO1_FAULT1_POL_SHIFT;
        const uint8_t polMask = (gpioPinCfg->pin == PMIC_GPIO_GPO2) ? GPO2_FAULT2_POL_MASK : GPO1_FAULT1_POL_MASK;
        gpioPinCfg->faultPolarity = Pmic_getBitField(regData, polShift, polMask);
    }

    if (Pmic_validParamCheck(gpioPinCfg->validParams, PMIC_GPIO_PIN_FAULT_TYPE_VALID)) {
        const uint8_t typeShift = (gpioPinCfg->pin == PMIC_GPIO_GPO2) ? GPO2_FAULT2_OD_SHIFT : GPO1_FAULT1_OD_SHIFT;
        const uint8_t typeMask = (gpioPinCfg->pin == PMIC_GPIO_GPO2) ? GPO2_FAULT2_OD_MASK : GPO1_FAULT1_OD_MASK;
        gpioPinCfg->faultType = Pmic_getBitField(regData, typeShift, typeMask);
    }

    if (Pmic_validParamCheck(gpioPinCfg->validParams, PMIC_GPIO_PIN_FXN_SEL_VALID)) {
        const uint8_t fxnSelShift = (gpioPinCfg->pin == PMIC_GPIO_GPO2) ? GPO2_SEL_SHIFT : GPO1_SEL_SHIFT;
        const uint8_t fxnSelMask = (gpioPinCfg->pin == PMIC_GPIO_GPO2) ? GPO2_SEL_MASK :  GPO1_SEL_MASK;
        gpioPinCfg->fxnSel = Pmic_getBitField(regData, fxnSelShift, fxnSelMask);
    }

    return status;
}

static int32_t GPIO_getStartupShutdownDelay(const Pmic_Handle_t *handle, Pmic_GpioPinCfg_t *gpioPinCfg) {
    uint8_t regData = 0U;
    uint8_t regAddr = (gpioPinCfg->pin == PMIC_GPIO_GPO1) ? VMON1_GPO1_SEQUENCE_REG : GPO2_SEQUENCE_REG;

    int32_t status = Pmic_ioRxByte_CS(handle, regAddr, &regData);
    if (status != PMIC_ST_SUCCESS) {
        return status;
    }

    if (Pmic_validParamCheck(gpioPinCfg->validParams, PMIC_GPIO_PIN_STARTUP_DELAY_VALID)) {
        gpioPinCfg->startupDelay = Pmic_getBitField(regData, STARTUP_DELAY_SHIFT, STARTUP_DELAY_MASK);
    }

    if (Pmic_validParamCheck(gpioPinCfg->validParams, PMIC_GPIO_PIN_SHUTDOWN_DELAY_VALID)) {
        gpioPinCfg->shutdownDelay = Pmic_getBitField(regData, SHUTDOWN_DELAY_SHIFT, SHUTDOWN_DELAY_MASK);
    }

    return PMIC_ST_SUCCESS;
}

int32_t Pmic_gpioGetPinCfg(const Pmic_Handle_t *handle, Pmic_GpioPinCfg_t *gpioPinCfg) {
    const uint32_t primaryCfgValid = (\
        PMIC_GPIO_PIN_FXN_SEL_VALID |
        PMIC_GPIO_PIN_FAULT_POLARITY_VALID |
        PMIC_GPIO_PIN_FAULT_TYPE_VALID);
    const uint32_t startupShutdownDelayValid = (\
        PMIC_GPIO_PIN_STARTUP_DELAY_VALID |
        PMIC_GPIO_PIN_SHUTDOWN_DELAY_VALID);

    int32_t status = Pmic_checkHandle(handle);
    if (status != PMIC_ST_SUCCESS) {
        return status;
    }

    if (gpioPinCfg == NULL) {
        return PMIC_ST_ERR_NULL_PARAM;
    }

    if ((gpioPinCfg->validParams == 0U) || (gpioPinCfg->pin > PMIC_GPIO_PIN_MAX)) {
        return PMIC_ST_ERR_INV_PARAM;
    }

    if (Pmic_validParamCheck(gpioPinCfg->validParams, primaryCfgValid)) {
        status = GPIO_getPrimaryCfg(handle, gpioPinCfg);
        if (status != PMIC_ST_SUCCESS) {
            return status;
        }
    }

    if (Pmic_validParamCheck(gpioPinCfg->validParams, startupShutdownDelayValid)) {
        status = GPIO_getStartupShutdownDelay(handle, gpioPinCfg);
        if (status != PMIC_ST_SUCCESS) {
            return status;
        }
    }

    return PMIC_ST_SUCCESS;
}

int32_t Pmic_gpioSetNErrCfg(const Pmic_Handle_t *handle, const Pmic_GpioNErrCfg_t *nErrCfg) {
    int32_t status = Pmic_checkHandle(handle);
    if (status != PMIC_ST_SUCCESS) {
        return status;
    }

    if (nErrCfg == NULL) {
        return PMIC_ST_ERR_NULL_PARAM;
    }

    return Pmic_ioUpdateByte_bCS(handle, INTERFACE_CONF_REG, NERR_PU_DIS_SHIFT, (nErrCfg->puEn ? (bool)false : (bool)true));
}

int32_t Pmic_gpioGetNErrCfg(const Pmic_Handle_t *handle, Pmic_GpioNErrCfg_t *nErrCfg) {
    uint8_t regData = 0U;

    int32_t status = Pmic_checkHandle(handle);
    if (status != PMIC_ST_SUCCESS) {
        return status;
    }

    if (nErrCfg == NULL) {
        return PMIC_ST_ERR_NULL_PARAM;
    }

    status = Pmic_ioRxByte_CS(handle, INTERFACE_CONF_REG, &regData);
    if (status != PMIC_ST_SUCCESS) {
        return status;
    }

    nErrCfg->puEn = Pmic_getBitField_b(regData, NERR_PU_DIS_SHIFT) ? (bool)false : (bool)true;
    return PMIC_ST_SUCCESS;
}

int32_t Pmic_gpioSetNIntCfg(const Pmic_Handle_t *handle, const Pmic_GpioNIntCfg_t *nIntCfg) {
    uint8_t regData = 0U;

    int32_t status = Pmic_checkHandle(handle);
    if (status != PMIC_ST_SUCCESS) {
        return status;
    }

    if (nIntCfg == NULL) {
        return PMIC_ST_ERR_NULL_PARAM;
    }

    if (nIntCfg->validParams == 0U) {
        return PMIC_ST_ERR_INV_PARAM;
    }

    Pmic_criticalSectionStart(handle);
    status = Pmic_ioRxByte(handle, INTERFACE_CONF_REG, &regData);

    if (Pmic_validParamStatusCheck(nIntCfg->validParams, PMIC_GPIO_NINT_PU_EN_VALID, status)) {
        Pmic_setBitField_b(&regData, NINT_PU_EN_SHIFT, nIntCfg->puEn);
    }

    if (Pmic_validParamStatusCheck(nIntCfg->validParams, PMIC_GPIO_NINT_TYPE_VALID, status)) {
        if (nIntCfg->type > PMIC_GPIO_TYPE_MAX) {
            status = PMIC_ST_ERR_INV_PARAM;
        } else {
            Pmic_setBitField(&regData, NINT_OD_SHIFT, NINT_OD_MASK, nIntCfg->type);
        }
    }

    if (Pmic_validParamStatusCheck(nIntCfg->validParams, PMIC_GPIO_NINT_POLARITY_VALID, status)) {
        if (nIntCfg->polarity > PMIC_GPIO_POLARITY_MAX) {
            status = PMIC_ST_ERR_INV_PARAM;
        } else {
            Pmic_setBitField(&regData, NINT_POL_SHIFT, NINT_POL_MASK, nIntCfg->polarity);
        }
    }

    if (status == PMIC_ST_SUCCESS) {
        status = Pmic_ioTxByte(handle, INTERFACE_CONF_REG, regData);
    }
    Pmic_criticalSectionStop(handle);

    return status;
}

int32_t Pmic_gpioGetNIntCfg(const Pmic_Handle_t *handle, Pmic_GpioNIntCfg_t *nIntCfg) {
    uint8_t regData = 0U;

    int32_t status = Pmic_checkHandle(handle);
    if (status != PMIC_ST_SUCCESS) {
        return status;
    }

    if (nIntCfg == NULL) {
        return PMIC_ST_ERR_NULL_PARAM;
    }

    if (nIntCfg->validParams == 0U) {
        return PMIC_ST_ERR_INV_PARAM;
    }

    status = Pmic_ioRxByte_CS(handle, INTERFACE_CONF_REG, &regData);
    if (status != PMIC_ST_SUCCESS) {
        return status;
    }

    if (Pmic_validParamCheck(nIntCfg->validParams, PMIC_GPIO_NINT_PU_EN_VALID)) {
        nIntCfg->puEn = Pmic_getBitField_b(regData, NINT_PU_EN_SHIFT);
    }

    if (Pmic_validParamCheck(nIntCfg->validParams, PMIC_GPIO_NINT_TYPE_VALID)) {
        nIntCfg->type = Pmic_getBitField(regData, NINT_OD_SHIFT, NINT_OD_MASK);
    }

    if (Pmic_validParamCheck(nIntCfg->validParams, PMIC_GPIO_NINT_POLARITY_VALID)) {
        nIntCfg->polarity = Pmic_getBitField(regData, NINT_POL_SHIFT, NINT_POL_MASK);
    }

    return PMIC_ST_SUCCESS;
}

static int32_t GPIO_setNRstOutPrimaryCfg(const Pmic_Handle_t *handle, const Pmic_GpioNRstOutCfg_t *nRstOutCfg) {
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    Pmic_criticalSectionStart(handle);
    status = Pmic_ioRxByte(handle, INTERFACE_CONF_REG, &regData);

    if (Pmic_validParamStatusCheck(nRstOutCfg->validParams, PMIC_GPIO_NRSTOUT_PU_EN_VALID, status)) {
        Pmic_setBitField_b(&regData, NRSTOUT_PU_EN_SHIFT, nRstOutCfg->puEn);
    }

    if (Pmic_validParamStatusCheck(nRstOutCfg->validParams, PMIC_GPIO_NRSTOUT_TYPE_VALID, status)) {
        if (nRstOutCfg->type > PMIC_GPIO_TYPE_MAX) {
            status = PMIC_ST_ERR_INV_PARAM;
        } else {
            Pmic_setBitField(&regData, NRSTOUT_OD_SHIFT, NRSTOUT_OD_MASK, nRstOutCfg->type);
        }
    }

    if (Pmic_validParamStatusCheck(nRstOutCfg->validParams, PMIC_GPIO_NRSTOUT_POLARITY_VALID, status)) {
        if (nRstOutCfg->polarity > PMIC_GPIO_POLARITY_MAX) {
            status = PMIC_ST_ERR_INV_PARAM;
        } else {
            Pmic_setBitField(&regData, NRSTOUT_POL_SHIFT, NRSTOUT_POL_MASK, nRstOutCfg->polarity);
        }
    }

    if (status == PMIC_ST_SUCCESS) {
        status = Pmic_ioTxByte(handle, INTERFACE_CONF_REG, regData);
    }
    Pmic_criticalSectionStop(handle);

    return status;
}

static int32_t GPIO_setNRstOutStartupShutdownDelayCfg(const Pmic_Handle_t *handle, const Pmic_GpioNRstOutCfg_t *nRstOutCfg) {
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    Pmic_criticalSectionStart(handle);
    status = Pmic_ioRxByte(handle, NRSTOUT_SEQUENCE_REG, &regData);

    if (Pmic_validParamStatusCheck(nRstOutCfg->validParams, PMIC_GPIO_NRSTOUT_STARTUP_DELAY_VALID, status)) {
        if (nRstOutCfg->startupDelay > PMIC_GPIO_DELAY_MAX) {
            status = PMIC_ST_ERR_INV_PARAM;
        } else {
            Pmic_setBitField(&regData, STARTUP_DELAY_SHIFT, STARTUP_DELAY_MASK, nRstOutCfg->startupDelay);
        }
    }

    if (Pmic_validParamStatusCheck(nRstOutCfg->validParams, PMIC_GPIO_NRSTOUT_SHUTDOWN_DELAY_VALID, status)) {
        if (nRstOutCfg->shutdownDelay > PMIC_GPIO_DELAY_MAX) {
            status = PMIC_ST_ERR_INV_PARAM;
        } else {
            Pmic_setBitField(&regData, SHUTDOWN_DELAY_SHIFT, SHUTDOWN_DELAY_MASK, nRstOutCfg->shutdownDelay);
        }
    }

    if (status == PMIC_ST_SUCCESS) {
        status = Pmic_ioTxByte(handle, NRSTOUT_SEQUENCE_REG, regData);
    }
    Pmic_criticalSectionStop(handle);

    return status;
}

int32_t Pmic_gpioSetNRstOutCfg(const Pmic_Handle_t *handle, const Pmic_GpioNRstOutCfg_t *nRstOutCfg) {
    const uint32_t primaryCfgValid = \
        PMIC_GPIO_NRSTOUT_PU_EN_VALID | PMIC_GPIO_NRSTOUT_TYPE_VALID | PMIC_GPIO_NRSTOUT_POLARITY_VALID;
    const uint32_t startupShutdownDelayCfgValid = \
        PMIC_GPIO_NRSTOUT_STARTUP_DELAY_VALID | PMIC_GPIO_NRSTOUT_SHUTDOWN_DELAY_VALID;

    int32_t status = Pmic_checkHandle(handle);
    if (status != PMIC_ST_SUCCESS) {
        return status;
    }

    if (nRstOutCfg == NULL) {
        return PMIC_ST_ERR_NULL_PARAM;
    }

    if (nRstOutCfg->validParams == 0U) {
        return PMIC_ST_ERR_INV_PARAM;
    }

    if (Pmic_validParamCheck(nRstOutCfg->validParams, primaryCfgValid)) {
        status = GPIO_setNRstOutPrimaryCfg(handle, nRstOutCfg);
        if (status != PMIC_ST_SUCCESS) {
            return status;
        }
    }

    if (Pmic_validParamCheck(nRstOutCfg->validParams, startupShutdownDelayCfgValid)) {
        status = GPIO_setNRstOutStartupShutdownDelayCfg(handle, nRstOutCfg);
        if (status != PMIC_ST_SUCCESS) {
            return status;
        }
    }

    return PMIC_ST_SUCCESS;
}

static int32_t GPIO_getNRstOutPrimaryCfg(const Pmic_Handle_t *handle, Pmic_GpioNRstOutCfg_t *nRstOutCfg) {
    uint8_t regData = 0U;

    int32_t status = Pmic_ioRxByte_CS(handle, INTERFACE_CONF_REG, &regData);
    if (status != PMIC_ST_SUCCESS) {
        return status;
    }

    if (Pmic_validParamCheck(nRstOutCfg->validParams, PMIC_GPIO_NRSTOUT_PU_EN_VALID)) {
        nRstOutCfg->puEn = Pmic_getBitField_b(regData, NRSTOUT_PU_EN_SHIFT);
    }

    if (Pmic_validParamCheck(nRstOutCfg->validParams, PMIC_GPIO_NRSTOUT_TYPE_VALID)) {
        nRstOutCfg->type = Pmic_getBitField(regData, NRSTOUT_OD_SHIFT, NRSTOUT_OD_MASK);
    }

    if (Pmic_validParamCheck(nRstOutCfg->validParams, PMIC_GPIO_NRSTOUT_POLARITY_VALID)) {
        nRstOutCfg->polarity = Pmic_getBitField(regData, NRSTOUT_POL_SHIFT, NRSTOUT_POL_MASK);
    }

    return PMIC_ST_SUCCESS;
}

static int32_t GPIO_getNRstOutStartupShutdownDelayCfg(const Pmic_Handle_t *handle, Pmic_GpioNRstOutCfg_t *nRstOutCfg) {
    uint8_t regData = 0U;

    int32_t status = Pmic_ioRxByte_CS(handle, NRSTOUT_SEQUENCE_REG, &regData);
    if (status != PMIC_ST_SUCCESS) {
        return status;
    }

    if (Pmic_validParamCheck(nRstOutCfg->validParams, PMIC_GPIO_NRSTOUT_STARTUP_DELAY_VALID)) {
        nRstOutCfg->startupDelay = Pmic_getBitField(regData, STARTUP_DELAY_SHIFT, STARTUP_DELAY_MASK);
    }

    if (Pmic_validParamCheck(nRstOutCfg->validParams, PMIC_GPIO_NRSTOUT_SHUTDOWN_DELAY_VALID)) {
        nRstOutCfg->shutdownDelay = Pmic_getBitField(regData, SHUTDOWN_DELAY_SHIFT, SHUTDOWN_DELAY_MASK);
    }

    return PMIC_ST_SUCCESS;
}

int32_t Pmic_gpioGetNRstOutCfg(const Pmic_Handle_t *handle, Pmic_GpioNRstOutCfg_t *nRstOutCfg) {
    const uint32_t primaryCfgValid = \
        PMIC_GPIO_NRSTOUT_PU_EN_VALID | PMIC_GPIO_NRSTOUT_TYPE_VALID | PMIC_GPIO_NRSTOUT_POLARITY_VALID;
    const uint32_t startupShutdownDelayCfgValid = \
        PMIC_GPIO_NRSTOUT_STARTUP_DELAY_VALID | PMIC_GPIO_NRSTOUT_SHUTDOWN_DELAY_VALID;

    int32_t status = Pmic_checkHandle(handle);
    if (status != PMIC_ST_SUCCESS) {
        return status;
    }

    if (nRstOutCfg == NULL) {
        return PMIC_ST_ERR_NULL_PARAM;
    }

    if (nRstOutCfg->validParams == 0U) {
        return PMIC_ST_ERR_INV_PARAM;
    }

    if (Pmic_validParamCheck(nRstOutCfg->validParams, primaryCfgValid)) {
        status = GPIO_getNRstOutPrimaryCfg(handle, nRstOutCfg);
        if (status != PMIC_ST_SUCCESS) {
            return status;
        }
    }

    if (Pmic_validParamCheck(nRstOutCfg->validParams, startupShutdownDelayCfgValid)) {
        status = GPIO_getNRstOutStartupShutdownDelayCfg(handle, nRstOutCfg);
        if (status != PMIC_ST_SUCCESS) {
            return status;
        }
    }

    return PMIC_ST_SUCCESS;
}
