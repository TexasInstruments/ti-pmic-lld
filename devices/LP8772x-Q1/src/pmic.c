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

/*
 * PMIC driver Handle INIT status. Used to validate the PMIC handle to avoid
 * corrupted handle usage.
 */
#define PMIC_DRV_INIT_SUCCESS (uint32_t)(0x504D4943U) /* "PMIC" in ASCII */
#define PMIC_DRV_INIT_UNINIT  (uint32_t)(0x00000000U)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

static int32_t initTimerWaitHook(const Pmic_HandleCfg_t *config, Pmic_Handle_t *handle,
                                  int32_t status)
{
    int32_t localStatus = status;

    if (Pmic_validParamStatusCheck(config->validParams, PMIC_CFG_INIT_TIMER_WAIT_MS_VALID, localStatus)) {
        if (config->timerWaitMs == NULL) {
            localStatus = PMIC_ST_ERR_NULL_FPTR;
        } else {
            handle->timerWaitMs = config->timerWaitMs;
        }
    }

    return localStatus;
}

static int32_t initHandleAddrAndTiming(const Pmic_HandleCfg_t *config, Pmic_Handle_t *handle, int32_t status)
{
    int32_t localStatus = status;

    /* Assign PMIC i2cAddr0 */
    if (Pmic_validParamStatusCheck(config->validParams, PMIC_CFG_INIT_I2C_ADDR0_VALID, localStatus)) {
        handle->i2cAddr0 = config->i2cAddr0;
    }

    /* Assign PMIC i2cAddr1 */
    if (Pmic_validParamStatusCheck(config->validParams, PMIC_CFG_INIT_I2C_ADDR1_VALID, localStatus)) {
        handle->i2cAddr1 = config->i2cAddr1;
    }

    /* Assign PMIC i2cAddr2 */
    if (Pmic_validParamStatusCheck(config->validParams, PMIC_CFG_INIT_I2C_ADDR2_VALID, localStatus)) {
        handle->i2cAddr2 = config->i2cAddr2;
    }

    /* Assign PMIC retry count */
    if (Pmic_validParamStatusCheck(config->validParams, PMIC_CFG_INIT_RETRY_CNT_VALID, localStatus)) {
        handle->retryCnt = config->retryCnt;
    }

    /* Assign PMIC retry interval */
    if (Pmic_validParamStatusCheck(config->validParams, PMIC_CFG_INIT_RETRY_INTERVAL_MS_VALID, localStatus)) {
        handle->retryIntervalMs = config->retryIntervalMs;
    }

    return initTimerWaitHook(config, handle, localStatus);
}

static int32_t initHandleBasicDevCfg(const Pmic_HandleCfg_t *config, Pmic_Handle_t *handle) {
    int32_t status = PMIC_ST_SUCCESS;

    /* Check and update PMIC Handle Comm Mode */
    if (Pmic_validParamCheck(config->validParams, PMIC_CFG_INIT_COMM_MODE_VALID)) {
        if (config->commMode != PMIC_INTF_I2C_SINGLE) {
            status = PMIC_ST_ERR_INV_PARAM;
        } else {
            handle->commMode = config->commMode;
        }
    }

    /* Check and update PMIC Handle Comm Handle */
    if (Pmic_validParamStatusCheck(config->validParams, PMIC_CFG_INIT_COMM_HANDLE_0_VALID, status)) {
        if (config->commHandle0 == NULL) {
            status = PMIC_ST_ERR_NULL_PARAM;
        } else {
            handle->commHandle0 = config->commHandle0;
        }
    }

    return initHandleAddrAndTiming(config, handle, status);
}

static int32_t initCommsFunctions(const Pmic_HandleCfg_t *config, Pmic_Handle_t *handle) {
    int32_t status = PMIC_ST_SUCCESS;

    /* Check and update PMIC Handle Comm IO RD Fn */
    if (Pmic_validParamCheck(config->validParams, PMIC_CFG_INIT_IO_READ_VALID)) {
        if (!config->ioRead) {
            status = PMIC_ST_ERR_NULL_FPTR;
        } else {
            handle->ioRead = config->ioRead;
        }
    }

    /* Check and update PMIC Handle Comm IO WR Fn */
    if (Pmic_validParamStatusCheck(config->validParams, PMIC_CFG_INIT_IO_WRITE_VALID, status)) {
        if (!config->ioWrite) {
            status = PMIC_ST_ERR_NULL_FPTR;
        } else {
            handle->ioWrite = config->ioWrite;
        }
    }

    return status;
}

static int32_t initCritSecFunctions(const Pmic_HandleCfg_t *config, Pmic_Handle_t *handle) {
    int32_t status = PMIC_ST_SUCCESS;

    if (Pmic_validParamStatusCheck(config->validParams, PMIC_CFG_INIT_CRITICAL_SECTION_START_VALID, status)) {
        if (!config->criticalSectionStart) {
            status = PMIC_ST_ERR_NULL_FPTR;
        } else {
            handle->criticalSectionStart = config->criticalSectionStart;
        }
    }

    /* Check and update PMIC Handle Critical Section Stop Fn */
    if (Pmic_validParamStatusCheck(config->validParams, PMIC_CFG_INIT_CRITICAL_SECTION_STOP_VALID, status)) {
        if (!config->criticalSectionStop) {
            status = PMIC_ST_ERR_NULL_FPTR;
        } else {
            handle->criticalSectionStop = config->criticalSectionStop;
        }
    }

    return status;
}

static int32_t initHandleFnPtrs(const Pmic_HandleCfg_t *config, Pmic_Handle_t *handle)
{
    int32_t status = initCommsFunctions(config, handle);

    if (status == PMIC_ST_SUCCESS) {
        status = initCritSecFunctions(config, handle);
    }

    return status;
}

static int32_t initCallbackFunctions(const Pmic_HandleCfg_t *config, Pmic_Handle_t *handle) {
    int32_t status = PMIC_ST_SUCCESS;

    if (Pmic_validParamStatusCheck(config->validParams, PMIC_CFG_INIT_IRQ_RESPONSE_CALLBACK_VALID, status)) {
        if (!config->irqResponseCallback) {
            status = PMIC_ST_ERR_NULL_FPTR;
        } else {
            handle->irqResponseCallback = config->irqResponseCallback;
        }
    }

    return status;
}

static int32_t getPmicInfo(Pmic_Handle_t *handle) {
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    /* Read DEV_REV register */
    status = Pmic_ioRxByte_CS(handle, PMIC_DEV_REV_REG, &regData);

    if (status == PMIC_ST_SUCCESS) {
        /* Store device revision */
        handle->devRev = regData;

        /* Read MANUFACTURING_VER register */
        status = Pmic_ioRxByte_CS(handle, PMIC_MANUFACTURING_VER_REG, &regData);
    }

    if (status == PMIC_ST_SUCCESS) {
        /* Store silicon revision */
        handle->devSiRev = regData;
    }

    return status;
}

static int32_t validateComms(Pmic_Handle_t *handle) {
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regVal = 0U;

    /* Start Critical Section */
    status = Pmic_ioRxByte_CS(handle, PMIC_DEV_REV_REG, &regVal);

    if (status == PMIC_ST_SUCCESS) {
        handle->drvInitStat = PMIC_DRV_INIT_SUCCESS;
    }

    return status;
}

static int32_t configureDeviceCrc(const Pmic_HandleCfg_t *config, Pmic_Handle_t *handle) {
    int32_t status = PMIC_ST_SUCCESS;

    // Determine the initial state of the communications CRC, this must be known
    // in order to configure the CRC state later as the config CRC must be
    // disabled and the register space needs to be unlocked
    status = Pmic_ioGetCrcEnableState(handle, &handle->crcEnable);

    // Unconditionally unlock config register space to ensure comms CRC can be
    // configured
    if (status == PMIC_ST_SUCCESS) {
        status = Pmic_setRegLockState(handle, PMIC_LOCK_DISABLE);
    }

    // Unconditionally disable config CRC to ensure comms CRC can be configured
    if (status == PMIC_ST_SUCCESS) {
        status = Pmic_configCrcDisable(handle);
    }

    // Update PMIC Comms CRC status, note that this performs communications with
    // the device to ensure handle `crcEnable` property and HW status are in
    // sync, and this relies on the handle being fully initialized so it must
    // come after DRV_INIT_SUCCESS.
    if (Pmic_validParamStatusCheck(config->validParams, PMIC_CFG_INIT_CRC_ENABLE_VALID, status)) {
        status = Pmic_ioSetCrcEnableState(handle, config->crcEnable);
    }

    // If the user requested that register CRC be enabled, re-enable it here, if
    // they listed this as a don't care, or explicitly marked it as disabled,
    // just leave it disabled.
    if (Pmic_validParamStatusCheck(config->validParams, PMIC_CFG_INIT_CONFIG_CRC_ENABLE_VALID, status)) {
        handle->configCrcEnable = config->configCrcEnable;

        if (config->configCrcEnable != (bool)false) {
            status = Pmic_configCrcEnable(handle, PMIC_CFG_CRC_RECALCULATE);
        }
    }

    // Re-lock config register space
    if (status == PMIC_ST_SUCCESS) {
        status = Pmic_setRegLockState(handle, PMIC_LOCK_ENABLE);
    }

    return status;
}

static int32_t validatePmicHandle(const Pmic_Handle_t *handle) {
    /* Validate sync I/O hooks */
    if ((handle->ioRead == NULL) || (handle->ioWrite == NULL)) {
        return PMIC_ST_ERR_NULL_FPTR;
    }

    /* Validate critical section hooks */
    if ((handle->criticalSectionStart == NULL) || (handle->criticalSectionStop == NULL)) {
        return PMIC_ST_ERR_NULL_FPTR;
    }

    return PMIC_ST_SUCCESS;
}

static int32_t validateHandle(const Pmic_Handle_t *handle) {
    if (handle->commMode > PMIC_INTF_MAX) {
        return PMIC_ST_ERR_INV_PARAM;
    }

    if (handle->commHandle0 == NULL) {
        return PMIC_ST_ERR_NULL_PARAM;
    }

    return validatePmicHandle(handle);
}

/* ========================================================================== */
/*                        Interface Implementations                           */
/* ========================================================================== */

static int32_t finalizeInit(const Pmic_HandleCfg_t *config, Pmic_Handle_t *handle, int32_t status)
{
    int32_t localStatus = status;

    if (localStatus == PMIC_ST_SUCCESS) { localStatus = validateComms(handle); }
    if (handle != NULL) {
        handle->drvInitStat = (localStatus == PMIC_ST_SUCCESS)
                              ? PMIC_DRV_INIT_SUCCESS : ~PMIC_DRV_INIT_SUCCESS;
    }
    if (localStatus == PMIC_ST_SUCCESS) { localStatus = configureDeviceCrc(config, handle); }
    return localStatus;
}

int32_t Pmic_init(Pmic_Handle_t *handle, const Pmic_HandleCfg_t *config) {
    int32_t status = PMIC_ST_SUCCESS;

    if ((handle == NULL) || (config == NULL)) {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if (status == PMIC_ST_SUCCESS) {
        *handle = (Pmic_Handle_t){0};
        status = initHandleBasicDevCfg(config, handle);
    }

    if (status == PMIC_ST_SUCCESS) {
        status = initHandleFnPtrs(config, handle);
    }

    if (status == PMIC_ST_SUCCESS) {
        status = validatePmicHandle(handle);
    }

    if (status == PMIC_ST_SUCCESS) {
        status = getPmicInfo(handle);
    }

    if (status == PMIC_ST_SUCCESS) {
        status = initCallbackFunctions(config, handle);
    }

    status = finalizeInit(config, handle, status);

    return status;
}

int32_t Pmic_deinit(Pmic_Handle_t *handle) {
    int32_t status = PMIC_ST_SUCCESS;

    if (handle == NULL) {
        status = PMIC_ST_ERR_INV_HANDLE;
    }

    if (status == PMIC_ST_SUCCESS) {
        handle->drvInitStat = PMIC_DRV_INIT_UNINIT;
        handle->devRev = 0U;
        handle->devSiRev = 0U;
        handle->commMode = 0U;
        handle->i2cAddr0 = 0U;
        handle->i2cAddr1 = 0U;
        handle->i2cAddr2 = 0U;
        handle->retryCnt = 0U;
        handle->retryIntervalMs = 0U;
        handle->crcEnable = PMIC_DISABLE;
        handle->configCrcEnable = PMIC_DISABLE;
        handle->commHandle0 = NULL;
        handle->ioRead = (void *)0U;
        handle->ioWrite = (void *)0U;
        handle->criticalSectionStart = (void *)0U;
        handle->criticalSectionStop = (void *)0U;
        handle->irqResponseCallback = (void *)0U;
        handle->timerWaitMs = (void *)0U;
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

    if ((handle->retryIntervalMs != 0U) && (handle->timerWaitMs == NULL)) {
        return PMIC_ST_ERR_NULL_FPTR;
    }

    return validateHandle(handle);
}
