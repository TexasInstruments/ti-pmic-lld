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

#include <string.h>

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

/**
 * @brief Copy Pmic_HandleCfg_t structure member-wise
 */
static inline void copyHandleCfg(const Pmic_HandleCfg_t *src, Pmic_HandleCfg_t *dst)
{
    (void)memmove((void *)dst, (const void *)src, sizeof(Pmic_HandleCfg_t));
}

static int32_t getPmicInfo(Pmic_Handle_t *handle)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    // Read DEV_REV register
    status = Pmic_ioRxByte_CS(handle, DEV_REV_REG, &regData);

    if (status == PMIC_ST_SUCCESS)
    {
        // Store device revision and read NVM_CODE_1 register
        handle->devRev = regData;
        status = Pmic_ioRxByte_CS(handle, NVM_CODE_1_REG, &regData);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Extract TI_NVM_ID bit field and read NVM_CODE_2 register
        handle->nvmCode = Pmic_getBitField(regData, TI_NVM_ID_SHIFT, TI_NVM_ID_MASK);
        status = Pmic_ioRxByte_CS(handle, NVM_CODE_2_REG, &regData);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Extract TI_NVM_REV bit field and read MANUFACTURING_VER register
        handle->nvmRev = Pmic_getBitField(regData, TI_NVM_REV_SHIFT, TI_NVM_REV_MASK);
        status = Pmic_ioRxByte_CS(handle, MANUFACTURING_VER_REG, &regData);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Extract SILICON_REV bit field
        handle->devSiRev = Pmic_getBitField(regData, SILICON_REV_SHIFT, SILICON_REV_MASK);
    }

    return status;
}

static int32_t validateAndSetI2CCfg(Pmic_Handle_t *handle, const Pmic_HandleCfg_t *config)
{
    int32_t status = PMIC_ST_SUCCESS;

    // i2cAddr0
    if (Pmic_validParamCheck(config->validParams, PMIC_I2C_ADDR0_VALID))
    {
        handle->i2cAddr0 = config->i2cAddr0;
    }

    // i2cAddr1
    if (Pmic_validParamCheck(config->validParams, PMIC_I2C_ADDR1_VALID))
    {
        handle->i2cAddr1 = config->i2cAddr1;
    }

    // i2cAddr2
    if (Pmic_validParamCheck(config->validParams, PMIC_I2C_ADDR2_VALID))
    {
        handle->i2cAddr2 = config->i2cAddr2;
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

    // taskHandle
    if (Pmic_validParamStatusCheck(config->validParams, PMIC_TASK_HANDLE_VALID, status))
    {
        if (config->taskHandle == NULL)
        {
            status = PMIC_ST_ERR_NULL_PARAM;
        }
        else
        {
            handle->taskHandle = config->taskHandle;
        }
    }

    return status;
}

static int32_t validateAndSetSyncHooks(Pmic_Handle_t *handle, const Pmic_HandleCfg_t *config)
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

    return status;
}

static int32_t validateAndSetAsyncHooks(Pmic_Handle_t *handle, const Pmic_HandleCfg_t *config)
{
    int32_t status = PMIC_ST_SUCCESS;

    // asyncRxStart
    if (Pmic_validParamStatusCheck(config->validParams, PMIC_ASYNC_RX_START_VALID, status))
    {
        if (config->asyncRxStart == NULL)
        {
            status = PMIC_ST_ERR_NULL_FPTR;
        }
        else
        {
            handle->asyncRxStart = config->asyncRxStart;
        }
    }

    // asyncTxStart
    if (Pmic_validParamStatusCheck(config->validParams, PMIC_ASYNC_TX_START_VALID, status))
    {
        if (config->asyncTxStart == NULL)
        {
            status = PMIC_ST_ERR_NULL_FPTR;
        }
        else
        {
            handle->asyncTxStart = config->asyncTxStart;
        }
    }

    // asyncRxAwait
    if (Pmic_validParamStatusCheck(config->validParams, PMIC_ASYNC_RX_AWAIT_VALID, status))
    {
        if (config->asyncRxAwait == NULL)
        {
            status = PMIC_ST_ERR_NULL_FPTR;
        }
        else
        {
            handle->asyncRxAwait = config->asyncRxAwait;
        }
    }

    // asyncTxAwait
    if (Pmic_validParamStatusCheck(config->validParams, PMIC_ASYNC_TX_AWAIT_VALID, status))
    {
        if (config->asyncTxAwait == NULL)
        {
            status = PMIC_ST_ERR_NULL_FPTR;
        }
        else
        {
            handle->asyncTxAwait = config->asyncTxAwait;
        }
    }

    return status;
}

static int32_t validateAndSetOtherHooks(Pmic_Handle_t *handle, const Pmic_HandleCfg_t *config)
{
    int32_t status = PMIC_ST_SUCCESS;

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

    // timerWaitMs
    if (Pmic_validParamStatusCheck(config->validParams, PMIC_TIMER_WAIT_MS_VALID, status))
    {
        if (config->timerWaitMs == NULL)
        {
            status = PMIC_ST_ERR_NULL_FPTR;
        }
        else
        {
            handle->timerWaitMs = config->timerWaitMs;
        }
    }

    return status;
}

static int32_t validateAndSetUserHooks(Pmic_Handle_t *handle, const Pmic_HandleCfg_t *config)
{
    // Synchronous hooks
    int32_t status = validateAndSetSyncHooks(handle, config);

    // Asynchronous hooks
    if (status == PMIC_ST_SUCCESS)
    {
        status = validateAndSetAsyncHooks(handle, config);
    }

    // Other hooks (critical section, IRQ response callback)
    if (status == PMIC_ST_SUCCESS)
    {
        status = validateAndSetOtherHooks(handle, config);
    }

    return status;
}

static int32_t validateAndSetHandleCfg(Pmic_Handle_t *handle, const Pmic_HandleCfg_t *config)
{
    int32_t status = PMIC_ST_SUCCESS;

    // commMode
    if (Pmic_validParamCheck(config->validParams, PMIC_COMM_MODE_VALID))
    {
        if (config->commMode > PMIC_INTF_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
        else
        {
            handle->commMode = config->commMode;
        }
    }

    // retryCnt
    if (Pmic_validParamStatusCheck(config->validParams, PMIC_RETRY_CNT_VALID, status))
    {
        handle->retryCnt = config->retryCnt;
    }

    // retryIntervalMs
    if (Pmic_validParamStatusCheck(config->validParams, PMIC_RETRY_INTERVAL_MS_VALID, status))
    {
        handle->retryIntervalMs = config->retryIntervalMs;
    }

    // maxLoopCnt
    if (Pmic_validParamStatusCheck(config->validParams, PMIC_MAX_LOOP_CNT_VALID, status))
    {
        handle->maxLoopCnt = config->maxLoopCnt;
    }

    // crcEnable
    if (Pmic_validParamStatusCheck(config->validParams, PMIC_CRC_ENABLE_VALID, status))
    {
        handle->crcEnable = config->crcEnable;
    }

    // asyncEnable
    if (Pmic_validParamStatusCheck(config->validParams, PMIC_ASYNC_ENABLE_VALID, status))
    {
        handle->asyncEnable = config->asyncEnable;
    }

    // Validate I2C configuration
    if (status == PMIC_ST_SUCCESS)
    {
        status = validateAndSetI2CCfg(handle, config);
    }

    // Validate communication/task handles
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

static int32_t validatePmicHandle(const Pmic_Handle_t *handle)
{
    // Check commMode
    if (handle->commMode > PMIC_INTF_MAX)
    {
        return PMIC_ST_ERR_INV_PARAM;
    }

    // Check commHandle0
    if (handle->commHandle0 == NULL)
    {
        return PMIC_ST_ERR_NULL_PARAM;
    }

    // Check criticalSectionStart, criticalSectionStop
    if ((handle->criticalSectionStart == NULL) || (handle->criticalSectionStop == NULL))
    {
        return PMIC_ST_ERR_NULL_FPTR;
    }

    // Check asyncRxStart, asyncTxStart, asyncRxAwait, asyncTxAwait
    if (handle->asyncEnable != false)
    {
        if ((handle->asyncRxStart == NULL) || (handle->asyncTxStart == NULL) || (handle->asyncRxAwait == NULL) || (handle->asyncTxAwait == NULL))
        {
            return PMIC_ST_ERR_NULL_FPTR;
        }
    }
    // Check ioRead, ioWrite
    else
    {
        if ((handle->ioRead == NULL) || (handle->ioWrite == NULL))
        {
            return PMIC_ST_ERR_NULL_FPTR;
        }
    }

    // Check timerWaitMs if retry interval is non-zero
    if (handle->retryIntervalMs != 0U)
    {
        if (handle->timerWaitMs == NULL)
        {
            return PMIC_ST_ERR_NULL_FPTR;
        }
    }

    return PMIC_ST_SUCCESS;
}

/* ========================================================================== */
/*                        Interface Implementations                           */
/* ========================================================================== */

int32_t Pmic_init(Pmic_Handle_t *handle, const Pmic_HandleCfg_t *config)
{
    Pmic_HandleCfg_t configLocal;
    int32_t status = PMIC_ST_SUCCESS;

    // Check whether parameters are valid
    if ((handle == NULL) || (config == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        copyHandleCfg(config, &configLocal);
        status = validateAndSetHandleCfg(handle, &configLocal);
    }

    // Validate PMIC handle once configurations have been set
    if (status == PMIC_ST_SUCCESS)
    {
        status = validatePmicHandle(handle);
    }

    // Get PMIC info, store info in pmic handle
    if (status == PMIC_ST_SUCCESS)
    {
        status = getPmicInfo(handle);
    }

    // Set the driver initialization status (only if handle is valid)
    if (handle != NULL)
    {
        handle->drvInitStat = (status == PMIC_ST_SUCCESS) ? PMIC_DRV_INIT_SUCCESS : ~PMIC_DRV_INIT_SUCCESS;
    }

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
        handle->devRev = 0U;
        handle->devSiRev = 0U;
        handle->nvmCode = 0U;
        handle->nvmRev = 0U;
        handle->commMode = 0U;
        handle->i2cAddr0 = 0U;
        handle->i2cAddr1 = 0U;
        handle->i2cAddr2 = 0U;
        handle->retryCnt = 0U;
        handle->retryIntervalMs = 0U;
        handle->crcEnable = PMIC_DISABLE;
        handle->asyncEnable = PMIC_DISABLE;
        handle->commHandle0 = NULL;
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
        handle->timerWaitMs = NULL;
    }

    return status;
}

int32_t Pmic_checkHandle(const Pmic_Handle_t *handle)
{
    if (handle == NULL)
    {
        return PMIC_ST_ERR_NULL_PARAM;
    }

    if (handle->drvInitStat != PMIC_DRV_INIT_SUCCESS)
    {
        return PMIC_ST_ERR_INV_HANDLE;
    }

    return validatePmicHandle(handle);
}
