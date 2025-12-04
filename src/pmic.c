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
 * @file pmic_io.c
 *
 * @brief This file contains definitions to PMIC handle initialization,
 * de-initialization, and validation APIs.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "pmic.h"

#include "regmap/core.h"

/* ========================================================================== */
/*                            Macros & Typedefs                               */
/* ========================================================================== */

// Conversion from hex to ASCII yields "PMIC"
#define DRV_INIT_SUCCESS (0x504D4943U)

/* ========================================================================== */
/*                           Function Definitions                             */
/* ========================================================================== */

static inline int32_t Pmic_setCommonInfo(Pmic_Handle_t *handle, const Pmic_HandleCfg_t *handleCfg) {
    if (Pmic_validParamCheck(handleCfg->validParams, PMIC_CRC_ENABLE_VALID)) {
        handle->crcEnable = handleCfg->crcEnable;
    }

    if (Pmic_validParamCheck(handleCfg->validParams, PMIC_COMM_HANDLE_0_VALID)) {
        if (handleCfg->commHandle0 != NULL) {
            handle->commHandle0 = handleCfg->commHandle0;
        } else {
            return PMIC_ST_ERR_NULL_PARAM;
        }
    }

    if (Pmic_validParamCheck(handleCfg->validParams, PMIC_CRITICAL_SECTION_START_VALID)) {
        if (handleCfg->criticalSectionStart != NULL) {
            handle->criticalSectionStart = handleCfg->criticalSectionStart;
        } else {
            return PMIC_ST_ERR_NULL_PARAM;
        }
    }

    if (Pmic_validParamCheck(handleCfg->validParams, PMIC_CRITICAL_SECTION_STOP_VALID)) {
        if (handleCfg->criticalSectionStop != NULL) {
            handle->criticalSectionStop = handleCfg->criticalSectionStop;
        } else {
            return PMIC_ST_ERR_NULL_PARAM;
        }
    }

    if (Pmic_validParamCheck(handleCfg->validParams, PMIC_IRQ_RESPONSE_CALLBACK_VALID)) {
        if (handleCfg->irqResponseCallback != NULL) {
            handle->irqResponseCallback = handleCfg->irqResponseCallback;
        } else {
            return PMIC_ST_ERR_NULL_PARAM;
        }
    }

    return PMIC_ST_SUCCESS;
}

static inline int32_t Pmic_setSyncInfo(Pmic_Handle_t *handle, const Pmic_HandleCfg_t *handleCfg) {
    if (Pmic_validParamCheck(handleCfg->validParams, PMIC_IO_READ_VALID)) {
        if (handleCfg->ioRead != NULL) {
            handle->ioRead = handleCfg->ioRead;
        } else {
            return PMIC_ST_ERR_NULL_PARAM;
        }
    }

    if (Pmic_validParamCheck(handleCfg->validParams, PMIC_IO_WRITE_VALID)) {
        if (handleCfg->ioWrite != NULL) {
            handle->ioWrite = handleCfg->ioWrite;
        } else {
            return PMIC_ST_ERR_NULL_PARAM;
        }
    }

    return PMIC_ST_SUCCESS;
}

static inline int32_t Pmic_setAsyncInfo(Pmic_Handle_t *handle, const Pmic_HandleCfg_t *handleCfg) {
    if (Pmic_validParamCheck(handleCfg->validParams, PMIC_ASYNC_ENABLE_VALID)) {
        handle->asyncEnable = handleCfg->asyncEnable;
    }

    if (Pmic_validParamCheck(handleCfg->validParams, PMIC_TASK_HANDLE_VALID)) {
        if (handleCfg->taskHandle != NULL) {
            handle->taskHandle = handleCfg->taskHandle;
        } else {
            return PMIC_ST_ERR_NULL_PARAM;
        }
    }

    if (Pmic_validParamCheck(handleCfg->validParams, PMIC_ASYNC_RX_START_VALID)) {
        if (handleCfg->asyncRxStart != NULL) {
            handle->asyncRxStart = handleCfg->asyncRxStart;
        } else {
            return PMIC_ST_ERR_NULL_PARAM;
        }
    }

    if (Pmic_validParamCheck(handleCfg->validParams, PMIC_ASYNC_TX_START_VALID)) {
        if (handleCfg->asyncTxStart != NULL) {
            handle->asyncTxStart = handleCfg->asyncTxStart;
        } else {
            return PMIC_ST_ERR_NULL_PARAM;
        }
    }

    if (Pmic_validParamCheck(handleCfg->validParams, PMIC_ASYNC_RX_AWAIT_VALID)) {
        if (handleCfg->asyncRxAwait != NULL) {
            handle->asyncRxAwait = handleCfg->asyncRxAwait;
        } else {
            return PMIC_ST_ERR_NULL_PARAM;
        }
    }

    if (Pmic_validParamCheck(handleCfg->validParams, PMIC_ASYNC_TX_AWAIT_VALID)) {
        if (handleCfg->asyncTxAwait != NULL) {
            handle->asyncTxAwait = handleCfg->asyncTxAwait;
        } else {
            return PMIC_ST_ERR_NULL_PARAM;
        }
    }

    return PMIC_ST_SUCCESS;
}

static int32_t setHandleMembers(Pmic_Handle_t *handle, const Pmic_HandleCfg_t *handleCfg) {
    int32_t status = PMIC_ST_SUCCESS;
    const bool commonInfoValidParams = \
        PMIC_CRC_ENABLE_VALID | PMIC_COMM_HANDLE_0_VALID | PMIC_CRITICAL_SECTION_START_VALID |
        PMIC_CRITICAL_SECTION_STOP_VALID | PMIC_IRQ_RESPONSE_CALLBACK_VALID;
    const bool syncInfoValidParams = \
        PMIC_IO_READ_VALID | PMIC_IO_WRITE_VALID;
    const bool asyncInfoValidParams = \
        PMIC_ASYNC_RX_START_VALID | PMIC_ASYNC_TX_START_VALID |
        PMIC_ASYNC_RX_AWAIT_VALID | PMIC_ASYNC_TX_AWAIT_VALID;

    if (Pmic_validParamCheck(handleCfg->validParams, commonInfoValidParams)) {
        status = Pmic_setCommonInfo(handle, handleCfg);
    }

    if (Pmic_validParamStatusCheck(handleCfg->validParams, syncInfoValidParams, status)) {
        status = Pmic_setSyncInfo(handle, handleCfg);
    }

    if (Pmic_validParamStatusCheck(handleCfg->validParams, asyncInfoValidParams, status)) {
        status = Pmic_setAsyncInfo(handle, handleCfg);
    }

    return status;
}

static inline int32_t validatePmicHandle(const Pmic_Handle_t *handle) {
    const bool invalidHandleCondition_common = \
        ((handle->commHandle0 == NULL) || (handle->criticalSectionStart == NULL) || (handle->criticalSectionStop == NULL));
    const bool invalidHandleCondition_sync = \
        ((handle->ioRead == NULL) || (handle->ioWrite == NULL));
    const bool invalidHandleCondition_async = \
        ((handle->asyncRxStart == NULL) || (handle->asyncTxStart == NULL) ||
         (handle->asyncRxAwait == NULL) || (handle->asyncTxAwait == NULL));

    if (invalidHandleCondition_common) {
        return PMIC_ST_ERR_INV_HANDLE;
    }

    if (handle->asyncEnable && invalidHandleCondition_async) {
        return PMIC_ST_ERR_INV_HANDLE;
    }

    if ((handle->asyncEnable == (bool)false) && invalidHandleCondition_sync) {
        return PMIC_ST_ERR_INV_HANDLE;
    }

    return PMIC_ST_SUCCESS;
}

static int32_t getPmicInfo(Pmic_Handle_t *handle) {
    int32_t status = Pmic_ioRxByte(handle, DEV_REV_REG, &(handle->devId));
    if (status != PMIC_ST_SUCCESS) {
        return status;
    }

    status = Pmic_ioRxByte(handle, MANUFACTURING_VER_REG, &(handle->siRev));
    if (status != PMIC_ST_SUCCESS) {
        return status;
    }

    status = Pmic_ioRxByte(handle, NVM_CODE_1_REG, &(handle->nvmId));
    if (status != PMIC_ST_SUCCESS) {
        return status;
    }

    return Pmic_ioRxByte(handle, NVM_CODE_2_REG, &(handle->nvmRev));
}

int32_t Pmic_init(Pmic_Handle_t *handle, const Pmic_HandleCfg_t *handleCfg) {
    int32_t status = PMIC_ST_SUCCESS;

    if ((handle == NULL) || (handleCfg == NULL)) {
        return PMIC_ST_ERR_NULL_PARAM;
    }

    if (handleCfg->validParams == 0U) {
        return PMIC_ST_ERR_INV_PARAM;
    }

    status = setHandleMembers(handle, handleCfg);
    if (status != PMIC_ST_SUCCESS) {
        return status;
    }

    status = validatePmicHandle(handle);
    if (status != PMIC_ST_SUCCESS) {
        return status;
    }

    status = getPmicInfo(handle);
    if (status != PMIC_ST_SUCCESS) {
        return status;
    }

    handle->drvInitStat = DRV_INIT_SUCCESS;
    return PMIC_ST_SUCCESS;
}

int32_t Pmic_deinit(Pmic_Handle_t *handle) {
    int32_t status = Pmic_checkHandle(handle);
    if (status != PMIC_ST_SUCCESS) {
        return status;
    }

    handle->drvInitStat = 0U;
    handle->devId = 0U;
    handle->nvmId = 0U;
    handle->nvmRev = 0U;
    handle->siRev = 0U;
    handle->crcEnable = (bool)false;
    handle->asyncEnable = (bool)false;
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

    return PMIC_ST_SUCCESS;
}

int32_t Pmic_checkHandle(const Pmic_Handle_t *handle) {
    if ((handle == NULL) || (handle->drvInitStat != DRV_INIT_SUCCESS)) {
        return PMIC_ST_ERR_INV_HANDLE;
    }

    return validatePmicHandle(handle);
}
