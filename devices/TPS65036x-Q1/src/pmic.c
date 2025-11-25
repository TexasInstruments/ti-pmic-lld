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
/**
 * @file pmic.c
 *
 * @brief This file contains definitions of the PMIC LLD initialization and
 * deinitialization APIs.
 */
#include "pmic.h"
#include "pmic_io.h"
#include "pmic_common.h"

#include "regmap/core.h"
#include "regmap/irq.h"

// BIT3 of SILICON_REV[7:0] identifies whether the PMIC is PG1 (A0) or PG2 (B1)
#define DEVICE_PG_IDENTIFER (3U)

// Used to unlock PMIC registers
#define REG_LOCK_KEY (0x9BU)

// Used to lock PMIC registers
#define REG_LOCK_VALUE (0xAAU)

static int32_t validateAndSetI2CConfig(Pmic_Handle_t *handle, const Pmic_HandleCfg_t *config)
{
    int32_t status = PMIC_ST_SUCCESS;

    // i2cAddr0
    if (Pmic_validParamCheck(config->validParams, PMIC_I2C_ADDR0_VALID))
    {
        handle->i2cAddr0 = config->i2cAddr0;
    }

    return status;
}

static int32_t validateAndSetUserHandles(Pmic_Handle_t *handle, const Pmic_HandleCfg_t *config)
{
    int32_t status = PMIC_ST_SUCCESS;

    // commHandle0
    if (Pmic_validParamCheck(config->validParams, PMIC_COMM_HANDLE_0_VALID))
    {
        if (config->commHandle0 == NULL)
        {
            status = PMIC_ST_ERR_NULL_PARAM;
        }
        else
        {
            handle->commHandle0 = config->commHandle0;
        }
    }

    return status;
}

static int32_t validateAndSetUserHooks(Pmic_Handle_t *handle, const Pmic_HandleCfg_t *config)
{
    int32_t status = PMIC_ST_SUCCESS;

    // ioRead
    if (Pmic_validParamCheck(config->validParams, PMIC_IO_READ_VALID))
    {
        if (config->ioRead == NULL)
        {
            status = PMIC_ST_ERR_NULL_FPTR;
        }
        else
        {
            handle->ioRead = config->ioRead;
        }
    }

    // ioWrite
    if (Pmic_validParamStatusCheck(config->validParams, PMIC_IO_WRITE_VALID, status))
    {
        if (config->ioWrite == NULL)
        {
            status = PMIC_ST_ERR_NULL_FPTR;
        }
        else
        {
            handle->ioWrite = config->ioWrite;
        }
    }

    // criticalSectionStart
    if (Pmic_validParamStatusCheck(config->validParams, PMIC_CRITICAL_SECTION_START_VALID, status))
    {
        if (config->criticalSectionStart == NULL)
        {
            status = PMIC_ST_ERR_NULL_FPTR;
        }
        else
        {
            handle->criticalSectionStart = config->criticalSectionStart;
        }
    }

    // criticalSectionStop
    if (Pmic_validParamStatusCheck(config->validParams, PMIC_CRITICAL_SECTION_STOP_VALID, status))
    {
        if (config->criticalSectionStop == NULL)
        {
            status = PMIC_ST_ERR_NULL_FPTR;
        }
        else
        {
            handle->criticalSectionStop = config->criticalSectionStop;
        }
    }

    // irqResponseCallback
    if (Pmic_validParamStatusCheck(config->validParams, PMIC_IRQ_RESPONSE_CALLBACK_VALID, status))
    {
        if (config->irqResponseCallback == NULL)
        {
            status = PMIC_ST_ERR_NULL_FPTR;
        }
        else
        {
            handle->irqResponseCallback = config->irqResponseCallback;
        }
    }

    return status;
}

static int32_t validateAndSetHandleCfg(Pmic_Handle_t *handle, const Pmic_HandleCfg_t *config)
{
    int32_t status = PMIC_ST_SUCCESS;

    // Validate I2C configuration
    status = validateAndSetI2CConfig(handle, config);

    // Validate communication handles
    if (status == PMIC_ST_SUCCESS)
    {
        status = validateAndSetUserHandles(handle, config);
    }

    // Validate user-implemented hooks
    if (status == PMIC_ST_SUCCESS)
    {
        status = validateAndSetUserHooks(handle, config);
    }

    return status;
}

static int32_t getPmicInfo(Pmic_Handle_t *pmicHandle)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    // Read INTERFACE_CONF register
    Pmic_criticalSectionStart(pmicHandle);
    status = Pmic_ioRxByte(pmicHandle, PMIC_INTERFACE_CONF_REG, &regData);

    if (status == PMIC_ST_SUCCESS)
    {
        // Extract I2C_CRC_EN bit field and read DEV_REV register
        pmicHandle->crcEnable = Pmic_getBitField_b(regData, PMIC_I2C_CRC_EN_SHIFT);
        status = Pmic_ioRxByte(pmicHandle, PMIC_DEV_REV_REG, &regData);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Extract TI_DEVICE_ID bit field and read NVM_CODE_1 register
        pmicHandle->devRev = regData;
        status = Pmic_ioRxByte(pmicHandle, PMIC_NVM_CODE_1_REG, &regData);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Extract TI_NVM_ID bit field and read NVM_CODE_2 register
        pmicHandle->nvmCode = regData;
        status = Pmic_ioRxByte(pmicHandle, PMIC_NVM_CODE_2_REG, &regData);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Extract TI_NVM_REV bit field and read MANUFACTURING_VER register
        pmicHandle->nvmRev = regData;
        status = Pmic_ioRxByte(pmicHandle, PMIC_MANUFACTURING_VER_REG, &regData);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Extract SILICON_REV bit field
        pmicHandle->devSiRev = regData;
    }
    Pmic_criticalSectionStop(pmicHandle);

    return status;
}

static int32_t decipherWhetherA0(Pmic_Handle_t *pmicHandle)
{
    uint8_t regData = 0U;
    bool regsLocked = (bool)false;
    int32_t status = PMIC_ST_SUCCESS;

    // Get register lock status
    status = Pmic_ioRxByte(pmicHandle, PMIC_REGISTER_LOCK_REG, &regData);

    // Unlock registers if they are locked
    if (status == PMIC_ST_SUCCESS)
    {
        regsLocked = Pmic_getBitField_b(regData, PMIC_REGISTER_LOCK_STATUS_SHIFT);

        if (regsLocked)
        {
            status = Pmic_ioTxByte(pmicHandle, PMIC_REGISTER_LOCK_REG, REG_LOCK_KEY);
        }
    }

    // Set NRSTOUT_READBACK_MASK to 1. Only A0 silicon has this bit field.
    // So if NRSTOUT_READBACK_MASK is writable, the device is A0
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioUpdateByte_b(pmicHandle, PMIC_MASK_MODERATE_ERR_REG, PMIC_NRSTOUT_READBACK_MASK_SHIFT, (bool)true);
    }

    // Get actual NRSTOUT_READBACK_MASK value
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioRxByte(pmicHandle, PMIC_MASK_MODERATE_ERR_REG, &regData);
    }

    // Device is A0 if NRSTOUT_READBACK_MASK is 1. Otherwise, device is B0.
    // NOTE: NRSTOUT_READBACK_MASK is recommended to be set to 1; it is not
    // necessary to revert it back to 0 if it was previously 0
    if ((status == PMIC_ST_SUCCESS) && Pmic_getBitField_b(regData, PMIC_NRSTOUT_READBACK_MASK_SHIFT))
    {
        pmicHandle->isA0 = (bool)true;
    }

    // Re-lock PMIC registers if they were previously locked
    if ((status == PMIC_ST_SUCCESS) && regsLocked)
    {

        status = Pmic_ioTxByte(pmicHandle, PMIC_REGISTER_LOCK_REG, REG_LOCK_VALUE);
    }

    return status;
}

int32_t Pmic_init(Pmic_Handle_t *handle, const Pmic_HandleCfg_t *config)
{
    int32_t status = PMIC_ST_SUCCESS;

    // Check whether parameters are valid
    if ((handle == NULL) || (config == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    // Validate and set configuration parameters
    if (status == PMIC_ST_SUCCESS)
    {
        status = validateAndSetHandleCfg(handle, config);
    }

    // Get PMIC info, store info in pmic handle
    if (status == PMIC_ST_SUCCESS)
    {
        status = getPmicInfo(handle);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        const bool isB1 = Pmic_getBitField_b(handle->devSiRev, DEVICE_PG_IDENTIFER);

        handle->isA0 = (bool)false;
        if (!isB1)
        {
            // Device is not B1. Decipher whether device is A0 or B0
            status = decipherWhetherA0(handle);
        }
    }

    // Set the driver initialization status
    handle->drvInitStat = (status == PMIC_ST_SUCCESS) ? PMIC_DRV_INIT_SUCCESS : ~PMIC_DRV_INIT_SUCCESS;

    return status;
}

int32_t Pmic_deinit(Pmic_Handle_t *pmicHandle)
{
    int32_t status = PMIC_ST_SUCCESS;

    if (pmicHandle == NULL)
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        pmicHandle->drvInitStat = 0U;
        pmicHandle->i2cAddr0 = 0U;
        pmicHandle->devRev = 0U;
        pmicHandle->nvmCode = 0U;
        pmicHandle->nvmRev = 0U;
        pmicHandle->devSiRev = 0U;
        pmicHandle->isA0 = (bool)false;
        pmicHandle->crcEnable = PMIC_DISABLE;
        pmicHandle->commHandle0 = NULL;
        pmicHandle->ioRead = NULL;
        pmicHandle->ioWrite = NULL;
        pmicHandle->criticalSectionStart = NULL;
        pmicHandle->criticalSectionStop = NULL;
        pmicHandle->irqResponseCallback = NULL;
    }

    return status;
}

int32_t Pmic_checkHandle(const Pmic_Handle_t *pmicHandle)
{
    int32_t status = PMIC_ST_SUCCESS;

    if (pmicHandle == NULL)
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        if (pmicHandle->commHandle0 == NULL)
        {
            status = PMIC_ST_ERR_NULL_PARAM;
        }
        else if ((pmicHandle->ioRead == NULL) || (pmicHandle->ioWrite == NULL) ||
                 (pmicHandle->criticalSectionStart == NULL) || (pmicHandle->criticalSectionStop == NULL))
        {
            status = PMIC_ST_ERR_NULL_FPTR;
        }
        else if (pmicHandle->drvInitStat != PMIC_DRV_INIT_SUCCESS)
        {
            status = PMIC_ST_ERR_INV_HANDLE;
        }
    }

    return status;
}
