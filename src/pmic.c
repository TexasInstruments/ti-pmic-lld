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

// Conversion from hex -> string yields "PMIC"
#define DRV_INIT_SUCCESS (0x504D4943U)

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

static int32_t setBasicInfo(Pmic_Handle_t *handle, const Pmic_HandleCfg_t *handleCfg) {
    if (Pmic_validParamCheck(handleCfg->validParams, PMIC_DEVICE_TYPE_VALID)) {
        if (handleCfg->deviceType != PMIC_DEV_CHARIOT_LP8774X) {
            return PMIC_ST_ERR_INV_DEVICE;
        } else {
            handle->deviceType = handleCfg->deviceType;
        }
    }

    if (Pmic_validParamCheck(handleCfg->validParams, PMIC_COMM_MODE_VALID)) {
        if (handleCfg->commMode != PMIC_INTF_SPI) {
            return PMIC_ST_ERR_INV_PARAM;
        } else {
            handle->commMode = handleCfg->commMode;
        }
    }

    return PMIC_ST_SUCCESS;
}

static int32_t setI2CInfo(Pmic_Handle_t *handle, const Pmic_HandleCfg_t *handleCfg) {
    if (Pmic_validParamCheck(handleCfg->validParams, PMIC_SLAVE_ADDR_VALID)) {
        handle->slaveAddr = handleCfg->slaveAddr;
    }

    if (Pmic_validParamCheck(handleCfg->validParams, PMIC_QA_SLAVE_ADDR_VALID)) {
        handle->qaSlaveAddr = handleCfg->qaSlaveAddr;
    }

    if (Pmic_validParamCheck(handleCfg->validParams, PMIC_NVM_SLAVE_ADDR_VALID)) {
        handle->nvmSlaveAddr = handleCfg->nvmSlaveAddr;
    }

    if (Pmic_validParamCheck(handleCfg->validParams, PMIC_I2C1_SPEED_VALID)) {
        if (handleCfg->i2c1Speed > PMIC_I2C_SPEED_SEL_MAX) {
            return PMIC_ST_ERR_INV_PARAM;
        } else {
            handle->i2c1Speed = handleCfg->i2c1Speed;
        }
    }

    if (Pmic_validParamCheck(handleCfg->validParams, PMIC_I2C2_SPEED_VALID)) {
        if (handleCfg->i2c2Speed > PMIC_I2C_SPEED_SEL_MAX) {
            return PMIC_ST_ERR_INV_PARAM;
        } else {
            handle->i2c2Speed = handleCfg->i2c2Speed;
        }
    }

    return PMIC_ST_SUCCESS;
}

static int32_t setCommHandles(Pmic_Handle_t *handle, const Pmic_HandleCfg_t *handleCfg) {
    if (Pmic_validParamCheck(handleCfg->validParams, PMIC_COMM_HANDLE_VALID)) {
        if (handleCfg->commHandle == NULL) {
            return PMIC_ST_ERR_NULL_PARAM;
        } else {
            handle->commHandle = handleCfg->commHandle;
        }
    }

    if (Pmic_validParamCheck(handleCfg->validParams, PMIC_QA_COMM_HANDLE_VALID)) {
        if (handleCfg->qaCommHandle == NULL) {
            return PMIC_ST_ERR_NULL_PARAM;
        } else {
            handle->qaCommHandle = handleCfg->qaCommHandle;
        }
    }

    return PMIC_ST_SUCCESS;
}

static int32_t setApiHooks(Pmic_Handle_t *handle, const Pmic_HandleCfg_t *handleCfg) {
    if (Pmic_validParamCheck(handleCfg->validParams, PMIC_IO_READ_VALID)) {
        if (handleCfg->ioRead == NULL) {
            return PMIC_ST_ERR_NULL_PARAM;
        } else {
            handle->ioRead = handleCfg->ioRead;
        }
    }

    if (Pmic_validParamCheck(handleCfg->validParams, PMIC_IO_WRITE_VALID)) {
        if (handleCfg->ioWrite == NULL) {
            return PMIC_ST_ERR_NULL_PARAM;
        } else {
            handle->ioWrite = handleCfg->ioWrite;
        }
    }

    if (Pmic_validParamCheck(handleCfg->validParams, PMIC_CRIT_SEC_START_VALID)) {
        if (handleCfg->critSecStart == NULL) {
            return PMIC_ST_ERR_NULL_PARAM;
        } else {
            handle->critSecStart = handleCfg->critSecStart;
        }
    }

    if (Pmic_validParamCheck(handleCfg->validParams, PMIC_CRIT_SEC_STOP_VALID)) {
        if (handleCfg->critSecStop == NULL) {
            return PMIC_ST_ERR_NULL_PARAM;
        } else {
            handle->critSecStop = handleCfg->critSecStop;
        }
    }

    if (Pmic_validParamCheck(handleCfg->validParams, PMIC_IRQ_RESPONSE_VALID)) {
        if (handleCfg->irqResponse == NULL) {
            return PMIC_ST_ERR_NULL_PARAM;
        } else {
            handle->irqResponse = handleCfg->irqResponse;
        }
    }

    return PMIC_ST_SUCCESS;
}

static int32_t setHandleMembers(Pmic_Handle_t *handle, const Pmic_HandleCfg_t *handleCfg) {
    int32_t status = PMIC_ST_SUCCESS;
    const uint32_t basicInfoValidParams = \
        PMIC_DEVICE_TYPE_VALID | PMIC_COMM_MODE_VALID;
    const uint32_t i2cInfoValidParams = \
        PMIC_SLAVE_ADDR_VALID | PMIC_QA_SLAVE_ADDR_VALID | PMIC_NVM_SLAVE_ADDR_VALID |
        PMIC_I2C1_SPEED_VALID | PMIC_I2C2_SPEED_VALID;
    const uint32_t commHandleValidParams = \
        PMIC_COMM_HANDLE_VALID | PMIC_QA_COMM_HANDLE_VALID;
    const uint32_t apiHookValidParams = \
        PMIC_IO_READ_VALID | PMIC_IO_WRITE_VALID | PMIC_CRIT_SEC_START_VALID |
        PMIC_CRIT_SEC_STOP_VALID | PMIC_IRQ_RESPONSE_VALID;

    if (Pmic_validParamCheck(handleCfg->validParams, basicInfoValidParams)) {
        status = setBasicInfo(handle, handleCfg);
    }

    if (status != PMIC_ST_SUCCESS) {
        return status;
    } else if (Pmic_validParamCheck(handleCfg->validParams, i2cInfoValidParams)) {
        status = setI2CInfo(handle, handleCfg);
    }

    if (status != PMIC_ST_SUCCESS) {
        return status;
    } else if (Pmic_validParamCheck(handleCfg->validParams, commHandleValidParams)) {
        status = setCommHandles(handle, handleCfg);
    }

    if (status != PMIC_ST_SUCCESS) {
        return status;
    } else if (Pmic_validParamCheck(handleCfg->validParams, apiHookValidParams)) {
        status = setApiHooks(handle, handleCfg);
    }

    return PMIC_ST_SUCCESS;
}

static inline int32_t validatePmicHandle(const Pmic_Handle_t *handle) {
    const bool invalidHandleCondition = \
        (handle->deviceType != PMIC_DEV_CHARIOT_LP8774X) ||
        (handle->commMode != PMIC_INTF_SPI) ||
        (handle->commHandle == NULL) ||
        (handle->ioRead == NULL) ||
        (handle->ioWrite == NULL) ||
        (handle->critSecStart == NULL) ||
        (handle->critSecStop == NULL);

    if (invalidHandleCondition) {
        return PMIC_ST_ERR_INV_HANDLE;
    } else {
        return PMIC_ST_SUCCESS;
    }
}

static int32_t enableDisableCommCrc(Pmic_Handle_t *handle, const Pmic_HandleCfg_t *handleCfg) {
    uint8_t regData = 0U;

    int32_t status = Pmic_ioRxByte(handle, INTERFACE_CONF_REGADDR, &regData);
    if (status != PMIC_ST_SUCCESS) {
        return status;
    }

    // Obtained initial CRC enable. Modify PMIC CRC enable if associated validParam is set
    handle->crcEnable = Pmic_getBitField_b(regData, SPI_CRC_EN_SHIFT);
    if (Pmic_validParamCheck(handleCfg->crcEnable, PMIC_CRC_ENABLE_VALID)) {
        Pmic_setBitField_b(&regData, SPI_CRC_EN_SHIFT, handleCfg->crcEnable);

        status = Pmic_ioTxByte(handle, INTERFACE_CONF_REGADDR, regData);
        if (status != PMIC_ST_SUCCESS) {
            return status;
        } else {
            handle->crcEnable = handleCfg->crcEnable;
        }
    }

    return PMIC_ST_SUCCESS;
}

static int32_t getPmicInfo(Pmic_Handle_t *handle) {
    int32_t status = Pmic_ioRxByte(handle, DEV_REV_REGADDR, &(handle->deviceId));
    if (status != PMIC_ST_SUCCESS) {
        return status;
    }

    status = Pmic_ioRxByte(handle, MANUFACTURING_VER_REGADDR, &(handle->deviceSiRev));
    if (status != PMIC_ST_SUCCESS) {
        return status;
    }

    status = Pmic_ioRxByte(handle, NVM_CODE_1_REGADDR, &(handle->deviceNvmId));
    if (status != PMIC_ST_SUCCESS) {
        return status;
    }

    return Pmic_ioRxByte(handle, NVM_CODE_2_REGADDR, &(handle->deviceNvmRev));
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

    status = enableDisableCommCrc(handle, handleCfg);
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
    int32_t status = Pmic_checkPmicHandle(handle);
    if (status != PMIC_ST_SUCCESS) {
        return status;
    }

    handle->drvInitStat = 0U;
    handle->deviceType = 0U;
    handle->deviceId = 0U;
    handle->deviceSiRev = 0U;
    handle->deviceNvmId = 0U;
    handle->deviceNvmRev = 0U;
    handle->commMode = 0U;
    handle->slaveAddr = 0U;
    handle->qaSlaveAddr = 0U;
    handle->nvmSlaveAddr = 0U;
    handle->i2c1Speed = 0U;
    handle->i2c2Speed = 0U;
    handle->crcEnable = (bool)false;
    handle->commHandle = NULL;
    handle->qaCommHandle = NULL;
    handle->ioRead = NULL;
    handle->ioWrite = NULL;
    handle->critSecStart = NULL;
    handle->critSecStop = NULL;
    handle->irqResponse = NULL;

    return PMIC_ST_SUCCESS;
}

int32_t Pmic_checkPmicHandle(const Pmic_Handle_t *handle) {
    if (handle == NULL) {
        return PMIC_ST_ERR_NULL_PARAM;
    }

    if (handle->drvInitStat != DRV_INIT_SUCCESS) {
        return PMIC_ST_ERR_INV_HANDLE;
    }

    return validatePmicHandle(handle);
}
