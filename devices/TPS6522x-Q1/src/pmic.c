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
 * @file pmic.c
 *
 * @brief This file contains definitions of the PMIC LLD initialization and
 * deinitialization APIs.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include "pmic.h"
#include "pmic_io.h"
#include "pmic_common.h"

#include "regmap/core.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
/**
 * @brief PMIC driver initialization status magic number. Used to validate
 * handle to avoid corrupted PMIC handle usage.
 */
#define PMIC_DRV_INIT_SUCCESS ((uint32_t)0x504D4943U)

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */
static inline void setPmicHandleMembers(const Pmic_HandleCfg_t *handleCfg, Pmic_Handle_t *handle)
{
    handle->commMode = handleCfg->commMode;
    handle->i2cAddr0 = handleCfg->i2cAddr0;
    handle->i2cAddr1 = handleCfg->i2cAddr1;
    handle->i2cAddr2 = handleCfg->i2cAddr2;
    handle->i2c1Speed = handleCfg->i2c1Speed;
    handle->i2c2Speed = handleCfg->i2c2Speed;
    handle->crcEnable = handleCfg->crcEnable;
    handle->asyncEnable = handleCfg->asyncEnable;
    handle->commHandle0 = handleCfg->commHandle0;
    handle->commHandle1 = handleCfg->commHandle1;
    handle->taskHandle = handleCfg->taskHandle;
    handle->ioRead = handleCfg->ioRead;
    handle->ioWrite = handleCfg->ioWrite;
    handle->asyncRxStart = handleCfg->asyncRxStart;
    handle->asyncTxStart = handleCfg->asyncTxStart;
    handle->asyncRxAwait = handleCfg->asyncRxAwait;
    handle->asyncTxAwait = handleCfg->asyncTxAwait;
    handle->criticalSectionStart = handleCfg->criticalSectionStart;
    handle->criticalSectionStop = handleCfg->criticalSectionStop;
    handle->irqResponseCallback = handleCfg->irqResponseCallback;
}

static int32_t getPmicInfo(Pmic_Handle_t *handle)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    // Read DEV_REV register
    Pmic_criticalSectionStart(handle);
    status = Pmic_ioRxByte(handle, DEV_REV_REGADDR, &regData);

    if (status == PMIC_ST_SUCCESS)
    {
        // Extract TI_DEVICE_ID bit field and read NVM_CODE_1 register
        handle->devId = Pmic_getBitField(regData, TI_DEVICE_ID_SHIFT, TI_DEVICE_ID_MASK);
        status = Pmic_ioRxByte(handle, NVM_CODE_1_REGADDR, &regData);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Extract TI_NVM_ID bit field and read NVM_CODE_2 register
        handle->nvmId = Pmic_getBitField(regData, TI_NVM_ID_SHIFT, TI_NVM_ID_MASK);
        status = Pmic_ioRxByte(handle, NVM_CODE_2_REGADDR, &regData);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Extract TI_NVM_REV bit field and read MANUFACTURING_VER register
        handle->nvmRev = Pmic_getBitField(regData, TI_NVM_REV_SHIFT, TI_NVM_REV_MASK);
        status = Pmic_ioRxByte(handle, MANUFACTURING_VER_REGADDR, &regData);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Extract SILICON_REV bit field
        handle->devSiRev = Pmic_getBitField(regData, SILICON_REV_SHIFT, SILICON_REV_MASK);
    }
    Pmic_criticalSectionStop(handle);

    return status;
}

static int32_t validateHandleCfg(const Pmic_HandleCfg_t *handleCfg)
{
    int32_t status = PMIC_ST_SUCCESS;

    // Check for NULL parameters
    if ((handleCfg->ioRead == NULL) || (handleCfg->ioWrite == NULL) ||
        (handleCfg->criticalSectionStart == NULL) || (handleCfg->criticalSectionStop == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    // Validate communication mode
    if ((status == PMIC_ST_SUCCESS) &&
        ((handleCfg->commMode < PMIC_INTF_MIN) || (handleCfg->commMode > PMIC_INTF_MAX)))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    // Validate communication handles based on mode
    if (status == PMIC_ST_SUCCESS)
    {
        if ((handleCfg->commMode == PMIC_INTF_I2C_DUAL) &&
            ((handleCfg->commHandle0 == NULL) || (handleCfg->commHandle1 == NULL)))
        {
            status = PMIC_ST_ERR_NULL_PARAM;
        }
        else if (((handleCfg->commMode == PMIC_INTF_I2C_SINGLE) ||
                  (handleCfg->commMode == PMIC_INTF_SPI)) &&
                 (handleCfg->commHandle0 == NULL))
        {
            status = PMIC_ST_ERR_NULL_PARAM;
        }
    }

    // Validate async mode parameters
    if ((status == PMIC_ST_SUCCESS) && (handleCfg->asyncEnable == PMIC_ENABLE))
    {
        if ((handleCfg->asyncRxStart == NULL) || (handleCfg->asyncTxStart == NULL) ||
            (handleCfg->asyncRxAwait == NULL) || (handleCfg->asyncTxAwait == NULL) ||
            (handleCfg->taskHandle == NULL))
        {
            status = PMIC_ST_ERR_NULL_PARAM;
        }
    }

    return status;
}

/* ========================================================================== */
/*                        Interface Implementations                           */
/* ========================================================================== */
int32_t Pmic_init(Pmic_Handle_t *handle, const Pmic_HandleCfg_t *handleCfg)
{
    int32_t status = PMIC_ST_SUCCESS;

    // Check whether parameters are valid
    if ((handle == NULL) || (handleCfg == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Validate configuration parameters
        status = validateHandleCfg(handleCfg);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Initialize PMIC handle with values from handleCfg
        setPmicHandleMembers(handleCfg, handle);

        // Get PMIC info, store info in pmic handle
        status = getPmicInfo(handle);
    }

    // Set the driver initialization status
    handle->drvInitStat = (status == PMIC_ST_SUCCESS) ? PMIC_DRV_INIT_SUCCESS : ~PMIC_DRV_INIT_SUCCESS;

    return status;
}

int32_t Pmic_deinit(Pmic_Handle_t *handle)
{
    int32_t status = PMIC_ST_SUCCESS;

    if (handle == NULL)
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        handle->drvInitStat = 0U;
        handle->devId = 0U;
        handle->devSiRev = 0U;
        handle->nvmId = 0U;
        handle->nvmRev = 0U;
        handle->commMode = 0U;
        handle->i2cAddr0 = 0U;
        handle->i2cAddr1 = 0U;
        handle->i2cAddr2 = 0U;
        handle->i2c1Speed = 0U;
        handle->i2c2Speed = 0U;
        handle->crcEnable = PMIC_DISABLE;
        handle->asyncEnable = PMIC_DISABLE;
        handle->commHandle0 = NULL;
        handle->commHandle1 = NULL;
        handle->taskHandle = NULL;
        handle->ioRead = NULL;
        handle->ioWrite = NULL;
        handle->asyncRxStart = NULL;
        handle->asyncTxStart = NULL;
        handle->asyncRxAwait = NULL;
        handle->asyncTxAwait = NULL;
        handle->criticalSectionStart = NULL;
        handle->criticalSectionStop = NULL;
        handle->irqResponseCallback = NULL;
    }

    return status;
}

int32_t Pmic_checkHandle(const Pmic_Handle_t *handle)
{
    int32_t status = PMIC_ST_SUCCESS;

    if (handle == NULL)
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) &&
        ((handle->commHandle0 == NULL) || (handle->ioRead == NULL) ||
         (handle->ioWrite == NULL) || (handle->criticalSectionStart == NULL) ||
         (handle->criticalSectionStop == NULL) ||
         (handle->drvInitStat != PMIC_DRV_INIT_SUCCESS)))
    {
        status = PMIC_ST_ERR_INV_HANDLE;
    }

    return status;
}
