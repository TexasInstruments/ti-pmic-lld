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

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "pmic.h"
#include "pmic_common.h"
#include "pmic_io.h"
#include "regmap/core.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/**
 * @brief PMIC driver initialization status magic numbers. Used to validate
 * handle to avoid corrupted PMIC handle usage.
 */
#define PMIC_DRV_INIT_SUCCESS ((uint32_t)0x504D4943U) /* "PMIC" in ASCII */
#define PMIC_DRV_INIT_UNINIT  ((uint32_t)0x00000000U)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

static int32_t initHandleBasicDevCfg(const Pmic_HandleCfg_t *config, Pmic_Handle_t *handle) {
    int32_t status = PMIC_ST_SUCCESS;

    /* Check and update PMIC Handle Comm Mode */
    if (Pmic_validParamStatusCheck(config->validParams, PMIC_COMM_MODE_VALID, status)) {
        if (config->commMode != PMIC_INTF_SPI) {
            status = PMIC_ST_ERR_INV_PARAM;
        } else {
            handle->commMode = config->commMode;
        }
    }

    /* Check and update PMIC Handle Comm Handle */
    if (Pmic_validParamStatusCheck(config->validParams, PMIC_COMM_HANDLE_0_VALID, status)) {
        if (config->commHandle0 == NULL) {
            status = PMIC_ST_ERR_NULL_PARAM;
        } else {
            handle->commHandle0 = config->commHandle0;
        }
    }

    /* Update retry count */
    if (Pmic_validParamStatusCheck(config->validParams, PMIC_RETRY_CNT_VALID, status)) {
        handle->retryCnt = config->retryCnt;
    }

    /* Update retry interval */
    if (Pmic_validParamStatusCheck(config->validParams, PMIC_RETRY_INTERVAL_MS_VALID, status)) {
        handle->retryIntervalMs = config->retryIntervalMs;
    }

    /* Update timer hook */
    if (Pmic_validParamStatusCheck(config->validParams, PMIC_TIMER_WAIT_MS_VALID, status)) {
        if (config->timerWaitMs == NULL) {
            status = PMIC_ST_ERR_NULL_FPTR;
        } else {
            handle->timerWaitMs = config->timerWaitMs;
        }
    }

    return status;
}

static int32_t initCommsFunctions(const Pmic_HandleCfg_t *config, Pmic_Handle_t *handle) {
    int32_t status = PMIC_ST_SUCCESS;

    /* Check and update PMIC Handle Comm IO RD Fn */
    if (Pmic_validParamCheck(config->validParams, PMIC_IO_READ_VALID)) {
        if (config->ioRead == NULL) {
            status = PMIC_ST_ERR_NULL_FPTR;
        } else {
            handle->ioRead = config->ioRead;
        }
    }

    /* Check and update PMIC Handle Comm IO WR Fn */
    if (Pmic_validParamStatusCheck(config->validParams, PMIC_IO_WRITE_VALID, status)) {
        if (config->ioWrite == NULL) {
            status = PMIC_ST_ERR_NULL_FPTR;
        } else {
            handle->ioWrite = config->ioWrite;
        }
    }

    return status;
}

static int32_t initCritSecFunctions(const Pmic_HandleCfg_t *config, Pmic_Handle_t *handle) {
    int32_t status = PMIC_ST_SUCCESS;

    if (Pmic_validParamStatusCheck(config->validParams, PMIC_CRITICAL_SECTION_START_VALID, status)) {
        if (config->criticalSectionStart == NULL) {
            status = PMIC_ST_ERR_NULL_FPTR;
        } else {
            handle->criticalSectionStart = config->criticalSectionStart;
        }
    }

    /* Check and update PMIC Handle Critical Section Stop Fn */
    if (Pmic_validParamStatusCheck(config->validParams, PMIC_CRITICAL_SECTION_STOP_VALID, status)) {
        if (config->criticalSectionStop == NULL) {
            status = PMIC_ST_ERR_NULL_FPTR;
        } else {
            handle->criticalSectionStop = config->criticalSectionStop;
        }
    }

    return status;
}

static int32_t initAsyncFunctions(const Pmic_HandleCfg_t *config, Pmic_Handle_t *handle) {
    int32_t status = PMIC_ST_SUCCESS;

    /* Check and update async enable flag */
    if (Pmic_validParamStatusCheck(config->validParams, PMIC_ASYNC_ENABLE_VALID, status)) {
        handle->asyncEnable = config->asyncEnable;
    }

    /* Check and update task handle */
    if (Pmic_validParamStatusCheck(config->validParams, PMIC_TASK_HANDLE_VALID, status)) {
        handle->taskHandle = config->taskHandle;
    }

    /* Check and update async RX start function */
    if (Pmic_validParamStatusCheck(config->validParams, PMIC_ASYNC_RX_START_VALID, status)) {
        handle->asyncRxStart = config->asyncRxStart;
    }

    /* Check and update async TX start function */
    if (Pmic_validParamStatusCheck(config->validParams, PMIC_ASYNC_TX_START_VALID, status)) {
        handle->asyncTxStart = config->asyncTxStart;
    }

    /* Check and update async RX await function */
    if (Pmic_validParamStatusCheck(config->validParams, PMIC_ASYNC_RX_AWAIT_VALID, status)) {
        handle->asyncRxAwait = config->asyncRxAwait;
    }

    /* Check and update async TX await function */
    if (Pmic_validParamStatusCheck(config->validParams, PMIC_ASYNC_TX_AWAIT_VALID, status)) {
        handle->asyncTxAwait = config->asyncTxAwait;
    }

    return status;
}

static int32_t getPmicInfo(Pmic_Handle_t *handle) {
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    /* Read DEV_ID register for device revision */
    status = Pmic_ioRxByte_CS(handle, PMIC_DEV_ID_REG, &regData);

    if (status == PMIC_ST_SUCCESS) {
        handle->devRev = Pmic_getBitField(regData, PMIC_DEV_ID_SHIFT, PMIC_DEV_ID_MASK);

        /* Read DEV_REV register for silicon revision */
        status = Pmic_ioRxByte_CS(handle, PMIC_DEV_REV_REG, &regData);
    }

    if (status == PMIC_ST_SUCCESS) {
        handle->devSiRev = Pmic_getBitField(regData, PMIC_DEV_REV_SHIFT, PMIC_DEV_REV_MASK);
    }

    /* Read NVM_CODE register */
    if (status == PMIC_ST_SUCCESS) {
        status = Pmic_ioRxByte_CS(handle, PMIC_NVM_CODE_REG, &regData);
    }

    if (status == PMIC_ST_SUCCESS) {
        /* Store NVM code */
        handle->nvmCode = Pmic_getBitField(regData, PMIC_NVM_CODE_SHIFT, PMIC_NVM_CODE_MASK);
    }

    /* Read NVM_REV register */
    if (status == PMIC_ST_SUCCESS) {
        status = Pmic_ioRxByte_CS(handle, PMIC_NVM_REV_REG, &regData);
    }

    if (status == PMIC_ST_SUCCESS) {
        /* Store NVM revision */
        handle->nvmRev = Pmic_getBitField(regData, PMIC_NVM_REV_SHIFT, PMIC_NVM_REV_MASK);
    }

    return status;
}

static int32_t validatePmicHandle(const Pmic_Handle_t *handle) {
    /* Validate async hooks if async mode enabled */
    if (handle->asyncEnable != false) {
        if ((handle->asyncRxStart == NULL) || (handle->asyncTxStart == NULL) ||
            (handle->asyncRxAwait == NULL) || (handle->asyncTxAwait == NULL)) {
            return PMIC_ST_ERR_NULL_FPTR;
        }
    }
    /* Validate sync I/O hooks if async mode disabled */
    else {
        if ((handle->ioRead == NULL) || (handle->ioWrite == NULL)) {
            return PMIC_ST_ERR_INSUFFICIENT_CFG;
        }
    }

    /* Validate critical section hooks */
    if ((handle->criticalSectionStart == NULL) || (handle->criticalSectionStop == NULL)) {
        return PMIC_ST_ERR_NULL_FPTR;
    }

    return PMIC_ST_SUCCESS;
}

static int32_t validateComms(const Pmic_Handle_t *handle) {
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regVal = 0U;

    /* Start Critical Section */
    status = Pmic_ioRxByte_CS(handle, PMIC_WD_LONGWIN_CFG_REG, &regVal);

    return status;
}

/* ========================================================================== */
/*                        Interface Implementations                           */
/* ========================================================================== */
int32_t Pmic_init(Pmic_Handle_t *handle, const Pmic_HandleCfg_t *config) {
    int32_t status = PMIC_ST_SUCCESS;

    if ((handle == NULL) || (config == NULL)) {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    /* Initialize handle to safe defaults */
    if (status == PMIC_ST_SUCCESS) {
        (void)memset(handle, 0, sizeof(Pmic_Handle_t));
    }

    /* Check and update PMIC Handle for device type, Comm Mode, Main Slave Address
     * and NVM Slave Address */
    if (status == PMIC_ST_SUCCESS) {
        status = initHandleBasicDevCfg(config, handle);
    }

    /* Check and update PMIC Handle for Comm IO RD Fn, Comm IO Wr Fn */
    if (status == PMIC_ST_SUCCESS) {
        status = initCommsFunctions(config, handle);
    }

    /* Check and update PMIC handle for Critical section Start/Stop */
    if (status == PMIC_ST_SUCCESS) {
        status = initCritSecFunctions(config, handle);
    }

    /* Check and update PMIC handle for async functions */
    if (status == PMIC_ST_SUCCESS) {
        status = initAsyncFunctions(config, handle);
    }

    /* Validate PMIC handle once configurations have been set */
    if (status == PMIC_ST_SUCCESS) {
        status = validatePmicHandle(handle);
    }

    // Get PMIC info, store info in pmic handle
    if (status == PMIC_ST_SUCCESS) {
        status = getPmicInfo(handle);
    }

    // Validate communication with the device.
    if (status == PMIC_ST_SUCCESS) {
        status = validateComms(handle);
    }

    // Set the driver initialization status (only if handle is valid)
    if (handle != NULL) {
        handle->drvInitStat = (status == PMIC_ST_SUCCESS) ? PMIC_DRV_INIT_SUCCESS : ~PMIC_DRV_INIT_SUCCESS;
    }

    return status;
}

int32_t Pmic_deinit(Pmic_Handle_t *handle) {
    int32_t status = PMIC_ST_SUCCESS;

    if (handle == NULL) {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if (status == PMIC_ST_SUCCESS) {
        handle->commHandle0 = NULL;
        handle->criticalSectionStart = NULL;
        handle->criticalSectionStop = NULL;
        handle->ioRead = NULL;
        handle->ioWrite = NULL;
        handle->asyncEnable = false;
        handle->taskHandle = NULL;
        handle->asyncRxStart = NULL;
        handle->asyncTxStart = NULL;
        handle->asyncRxAwait = NULL;
        handle->asyncTxAwait = NULL;
        handle->drvInitStat = PMIC_DRV_INIT_UNINIT;
        handle->devRev = 0U;
        handle->devSiRev = 0U;
        handle->nvmCode = 0U;
        handle->nvmRev = 0U;
    }

    return status;
}

int32_t Pmic_checkHandle(const Pmic_Handle_t *handle) {
    if (handle == NULL) {
        return PMIC_ST_ERR_NULL_PARAM;
    }

    if (handle->drvInitStat != PMIC_DRV_INIT_SUCCESS) {
        return PMIC_ST_ERR_INV_HANDLE;
    }

    if (handle->commMode != PMIC_INTF_SPI) {
        return PMIC_ST_ERR_INV_PARAM;
    }

    if (handle->commHandle0 == NULL) {
        return PMIC_ST_ERR_NULL_PARAM;
    }

    if ((handle->ioRead == NULL) || (handle->ioWrite == NULL)) {
        return PMIC_ST_ERR_NULL_FPTR;
    }

    if ((handle->criticalSectionStart == NULL) || (handle->criticalSectionStop == NULL)) {
        return PMIC_ST_ERR_NULL_FPTR;
    }

    if ((handle->retryIntervalMs != 0U) && (handle->timerWaitMs == NULL)) {
        return PMIC_ST_ERR_NULL_FPTR;
    }

    return PMIC_ST_SUCCESS;
}
