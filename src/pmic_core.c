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

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#include "pmic.h"
#include "pmic_types.h"

#include "pmic_core.h"
#include "pmic_io.h"
#include "regmap/core.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
#define PMIC_TPS65386X_DEV_ID  (0x3CU)

/* PMIC driver Core Handle INIT status Magic Number. Used to validate Handle to
   avoid corrupted PmicHandle usage.

   Note that this is used as the upper 16-bits of a 32-bit bitfield, the lower
   16-bits should be left 0. */
#define DRV_INIT_SUCCESS (uint32_t)(0xBEEF0000)
#define DRV_INIT_UNINIT  (uint32_t)(0x00000000)

#define PMIC_SILICON_REV_ID_PG_1_0 (0x0U)
#define PMIC_SILICON_REV_ID_PG_2_0 (0x08U)

#define REG_UNLOCK_DATA1    (0x98U)
#define REG_UNLOCK_DATA2    (0xB8U)
#define CNT_UNLOCK_DATA1    (0x13U)
#define CNT_UNLOCK_DATA2    (0x7DU)

#define PMIC_CRC_STATUS_DISABLED (0U)
#define PMIC_CRC_STATUS_ENABLED  (1U)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */
const Pmic_DevSubSysInfo_t pmicSubSysInfo[] = {
    /* PMIC_DEV_BB_TPS65386x */
    {
        .gpioEnable = true,
        .wdgEnable = true,
        .buckEnable = true,
        .ldoEnable = true,
        .esmEnable = true
    }
};

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */
/**
 * @brief Initialize the PMIC core handle with basic device configuration
 * parameters. This function initializes the PMIC core handle with basic device
 * configuration parameters.
 *
 * @param config Pointer to the PMIC core configuration data structure.
 * @param handle Pointer to the PMIC core handle structure to be
 * initialized.
 * @return status Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
static int32_t CORE_initHandleBasicDevCfg(const Pmic_CoreCfg_t *config, Pmic_CoreHandle_t *handle) {
    int32_t status = PMIC_ST_SUCCESS;

    /* Check and update PMIC Handle device type */
    if (Pmic_validParamCheck(config->validParams, PMIC_CFG_DEVICE_TYPE_VALID)) {
        if (config->pmicDeviceType != PMIC_DEV_BB_TPS65386X) {
            status = PMIC_ST_ERR_INV_PARAM;
        }

        if (status == PMIC_ST_SUCCESS) {
            handle->pmicDeviceType = config->pmicDeviceType;
        }
    }

    /* Check and update PMIC Handle Comm Mode */
    if (Pmic_validParamStatusCheck(config->validParams, PMIC_CFG_COMM_MODE_VALID, status)) {
        if (config->commMode != PMIC_INTF_SPI) {
            status = PMIC_ST_ERR_INV_PARAM;
        } else {
            handle->commMode = config->commMode;
        }
    }

    /* Check and update PMIC Handle Comm Handle */
    if (Pmic_validParamStatusCheck(config->validParams, PMIC_CFG_COMM_HANDLE_VALID, status)) {
        if (config->pCommHandle == NULL) {
            status = PMIC_ST_ERR_NULL_PARAM;
        } else {
            handle->pCommHandle = config->pCommHandle;
        }
    }

    return status;
}

/**
 * @brief Initialize the PMIC core handle with communication I/O critical
 * section function pointers. This function initializes the PMIC core handle
 * with communication I/O critical section function pointers.
 *
 * @param config Pointer to the PMIC core configuration data structure.
 * @param handle Pointer to the PMIC core handle structure to be
 * initialized.
 *
 * @return status Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
static int32_t CORE_initApplicationFunctions(const Pmic_CoreCfg_t *config, Pmic_CoreHandle_t *handle) {
    int32_t status = PMIC_ST_SUCCESS;

    /* Check and update PMIC Handle Comm IO RD Fn */
    if (Pmic_validParamCheck(config->validParams, PMIC_CFG_COMM_IO_RD_VALID)) {
        if (config->pFnPmicCommIoRead == NULL) {
            status = PMIC_ST_ERR_NULL_FPTR;
        } else {
            handle->pFnPmicCommIoRead = config->pFnPmicCommIoRead;
        }
    }

    /* Check and update PMIC Handle Comm IO WR Fn */
    if (Pmic_validParamStatusCheck(config->validParams, PMIC_CFG_COMM_IO_WR_VALID, status)) {
        if (config->pFnPmicCommIoWrite == NULL) {
            status = PMIC_ST_ERR_NULL_FPTR;
        } else {
            handle->pFnPmicCommIoWrite = config->pFnPmicCommIoWrite;
        }
    }

    /* Check and update PMIC Handle Critical Section Start Fn */
    if (Pmic_validParamStatusCheck(config->validParams, PMIC_CFG_CRITSEC_START_VALID, status)) {
        if (config->pFnPmicCritSecStart == NULL) {
            status = PMIC_ST_ERR_NULL_FPTR;
        } else {
            handle->pFnPmicCritSecStart = config->pFnPmicCritSecStart;
        }
    }

    /* Check and update PMIC Handle Critical Section Stop Fn */
    if (Pmic_validParamStatusCheck(config->validParams, PMIC_CFG_CRITSEC_STOP_VALID, status)) {
        if (config->pFnPmicCritSecStop == NULL) {
            status = PMIC_ST_ERR_NULL_FPTR;
        } else {
            handle->pFnPmicCritSecStop = config->pFnPmicCritSecStop;
        }
    }

    return status;
}

/**
 * @brief Validate the presence of the device on the bus.
 * This function validates the presence of the device on the bus.
 *
 * @param handle Pointer to the PMIC core handle structure.
 * @return status Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
static int32_t CORE_validateDeviceOnBus(Pmic_CoreHandle_t *handle) {
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regVal = 0U;

    /* Read DEV_ID register with critical section */
    Pmic_criticalSectionStart(handle);
    status = Pmic_commIntf_recvByte(handle, PMIC_DEV_ID_REGADDR, &regVal);
    Pmic_criticalSectionStop(handle);

    if (status == PMIC_ST_SUCCESS) {
        handle -> pmicDevRev = Pmic_getBitField(regVal, PMIC_DEV_ID_SHIFT, PMIC_DEV_ID_MASK);

        /* Validate if the device requested is the one on the bus */
        const bool handleForBB = (handle->pmicDeviceType == PMIC_DEV_BB_TPS65386X);
        const bool devIsNotBB = (handle->pmicDevRev != PMIC_TPS65386X_DEV_ID);
        if (handleForBB && devIsNotBB) {
            status = PMIC_ST_WARN_INV_DEVICE_ID;
        }
    }

    return status;
}

/**
 * @brief Update subsystem information and validate main Q&A communication
 * interface read/write.
 * This function updates subsystem information and validates the main Q&A
 * communication interface read/write.
 *
 * @param config Pointer to the PMIC core configuration data structure.
 * @param handle Pointer to the PMIC core handle structure to be
 * updated.
 * @return status Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
static int32_t CORE_updateSubSysInfoAndValidateCommsIF(const Pmic_CoreCfg_t *config, Pmic_CoreHandle_t *handle) {
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regVal = 0U;

    /* Update PMIC subsystem info to PMIC handle */
    handle->pPmic_SubSysInfo = &pmicSubSysInfo[handle->pmicDeviceType];

    /* Start Critical Section */
    Pmic_criticalSectionStart(handle);
    status = Pmic_commIntf_recvByte(handle, PMIC_WD_LONGWIN_CFG_REG, &regVal);
    Pmic_criticalSectionStop(handle);

    if (status == PMIC_ST_SUCCESS) {
        handle->drvInitStatus |= config->instType;
    }

    return status;
}

/* ========================================================================== */
/*                        Interface Implementations                           */
/* ========================================================================== */
inline bool Pmic_validParamCheck(uint32_t validParamVal, uint8_t bitPos) {
    return (((validParamVal >> bitPos) & 0x01U) != 0U);
}

inline bool Pmic_validParamStatusCheck(uint32_t validParamVal, uint8_t bitPos, int32_t status) {
    return (status == PMIC_ST_SUCCESS) && Pmic_validParamCheck(validParamVal, bitPos);
}

/**
 * @brief Initialize the PMIC core.
 * This function initializes the PMIC core based on the provided configuration
 * data.
 *
 * @param handle Pointer to the PMIC core handle structure to be
 * initialized.
 * @param config Pointer to the PMIC core configuration data structure.
 *
 * @return status Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
int32_t Pmic_init(Pmic_CoreHandle_t *handle, const Pmic_CoreCfg_t *config) {
    int32_t status = PMIC_ST_SUCCESS;

    if ((handle == NULL) || (config == NULL)) {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    /* Check and update PMIC Handle for device type, Comm Mode, Main Slave Address
     * and NVM Slave Address */
    if (status == PMIC_ST_SUCCESS) {
        status = CORE_initHandleBasicDevCfg(config, handle);
    }

    /* Check and update PMIC Handle for Comm IO RD Fn, Comm IO Wr Fn, Critical
     * Section Start Fn and Critical Section Stop Fn */
    if (status == PMIC_ST_SUCCESS) {
        status = CORE_initApplicationFunctions(config, handle);
    }

    // Set up the valid subsystems for this device and ensure that we can
    // communicate with the selected IO interface.
    if (status == PMIC_ST_SUCCESS) {
        status = CORE_updateSubSysInfoAndValidateCommsIF(config, handle);
    }

    /* Check for required members for I2C/SPI Main handle comm */
    if ((status == PMIC_ST_SUCCESS) &&
         ((handle->pFnPmicCritSecStart == NULL) ||
          (handle->pFnPmicCritSecStop == NULL) ||
          (handle->pFnPmicCommIoRead == NULL) ||
          (handle->pFnPmicCommIoWrite == NULL))) {
        status = PMIC_ST_ERR_INSUFFICIENT_CFG;
    }

    // Initialization is complete, mark it with magic.
    handle->drvInitStatus |= DRV_INIT_SUCCESS;

    return status;
}

/**
 * @brief Deinitialize the PMIC core.
 * This function deinitializes the PMIC core.
 *
 * @param handle Pointer to the PMIC core handle structure.
 * @return status Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
int32_t Pmic_deinit(Pmic_CoreHandle_t *handle) {
    int32_t status = PMIC_ST_SUCCESS;

    if (handle == NULL) {
        status = PMIC_ST_ERR_INV_HANDLE;
    }

    if (status == PMIC_ST_SUCCESS) {
        handle->pCommHandle = NULL;
        handle->pQACommHandle = NULL;
        handle->pFnPmicCritSecStart = NULL;
        handle->pFnPmicCritSecStop = NULL;
        handle->pFnPmicCommIoRead = NULL;
        handle->pFnPmicCommIoWrite = NULL;
        handle->pPmic_SubSysInfo = NULL;
        handle->drvInitStatus = 0x00U;
    }

    return status;
}

/**
 * @brief Start a critical section for PMIC operations.
 * This function starts a critical section for PMIC operations, if the critical
 * section start function pointer is not NULL.
 *
 * @param handle Pointer to the PMIC core handle structure.
 * @return void No return value.
 */
void Pmic_criticalSectionStart(const Pmic_CoreHandle_t *handle) {
    if (handle->pFnPmicCritSecStart != NULL) {
        handle->pFnPmicCritSecStart();
    }
}

/**
 * @brief Stop a critical section for PMIC operations.
 * This function stops a critical section for PMIC operations, if the critical
 * section stop function pointer is not NULL.
 *
 * @param handle Pointer to the PMIC core handle structure.
 * @return void No return value.
 */
void Pmic_criticalSectionStop(const Pmic_CoreHandle_t *handle) {
    if (handle->pFnPmicCritSecStop != NULL) {
        handle->pFnPmicCritSecStop();
    }
}

int32_t Pmic_checkPmicCoreHandle(const Pmic_CoreHandle_t *handle) {
    int32_t status = PMIC_ST_SUCCESS;
    uint32_t expectedInitStatus = 0U;

    if ((handle == NULL) || (handle->pCommHandle == NULL)) {
        status = PMIC_ST_ERR_INV_HANDLE;
    }

    if ((status == PMIC_ST_SUCCESS) && (handle->pFnPmicCommIoRead == NULL)) {
        status = PMIC_ST_ERR_NULL_FPTR;
    }

    if (status == PMIC_ST_SUCCESS) {
        if (handle->commMode == PMIC_INTF_I2C_SINGLE) {
            expectedInitStatus = (uint32_t)(DRV_INIT_SUCCESS | (uint8_t)PMIC_MAIN_INST);
        } else if (handle->commMode == PMIC_INTF_I2C_DUAL) {
            expectedInitStatus = (uint32_t)(DRV_INIT_SUCCESS | (uint8_t)PMIC_MAIN_INST | (uint8_t)PMIC_QA_INST);
        } else if (handle->commMode == PMIC_INTF_SPI) {
            expectedInitStatus = (uint32_t)(DRV_INIT_SUCCESS | (uint8_t)PMIC_MAIN_INST);
        } else {
            expectedInitStatus = DRV_INIT_UNINIT;
        }

        if (expectedInitStatus != handle->drvInitStatus) {
            status = PMIC_ST_ERR_INV_HANDLE;
        }
    }

    return status;
}

int32_t Pmic_setRegLockState(Pmic_CoreHandle_t *handle, uint8_t lockState) {
    int32_t status = Pmic_checkPmicCoreHandle(handle);

    // Holds the sequence used for register lock/unlock. For locking, writing
    // any sequence other than the correct one will lock the registers, in
    // which case 0->0 is fine (which is why it is the default).
    uint8_t seq[2] = {0, 0};

    // If unlocking registers, set the correct unlock sequence.
    if ((status == PMIC_ST_SUCCESS) && (lockState == PMIC_LOCK_DISABLE)) {
        seq[0] = REG_UNLOCK_DATA1;
        seq[1] = REG_UNLOCK_DATA2;
    }

    // Obtain Critical Section
    Pmic_criticalSectionStart(handle);

    if (status == PMIC_ST_SUCCESS) {
        status = Pmic_commIntf_sendByte(handle, CFG_REG_UNLOCK_SEQ_REG, seq[0]);
    }

    if (status == PMIC_ST_SUCCESS) {
        status = Pmic_commIntf_sendByte(handle, CFG_REG_UNLOCK_SEQ_REG, seq[1]);
    }

    // Release Critical Section
    Pmic_criticalSectionStop(handle);

    return status;
}

int32_t Pmic_setCntLockState(Pmic_CoreHandle_t *handle, uint8_t lockState) {
    int32_t status = Pmic_checkPmicCoreHandle(handle);

    // Holds the sequence used for register lock/unlock. For locking, writing
    // any sequence other than the correct one will lock the registers, in
    // which case 0->0 is fine (which is why it is the default).
    uint8_t seq[2] = {0, 0};

    // If unlocking registers, set the correct unlock sequence.
    if ((status == PMIC_ST_SUCCESS) && (lockState == PMIC_LOCK_DISABLE)) {
        seq[0] = CNT_UNLOCK_DATA1;
        seq[1] = CNT_UNLOCK_DATA2;
    }

    // Obtain Critical Section
    Pmic_criticalSectionStart(handle);

    if (status == PMIC_ST_SUCCESS) {
        status = Pmic_commIntf_sendByte(handle, CNT_REG_UNLOCK_SEQ_REG, seq[0]);
    }

    if (status == PMIC_ST_SUCCESS) {
        status = Pmic_commIntf_sendByte(handle, CNT_REG_UNLOCK_SEQ_REG, seq[1]);
    }

    // Release Critical Section
    Pmic_criticalSectionStop(handle);

    return status;
}

int32_t Pmic_setLockCfg(Pmic_CoreHandle_t *handle, const Pmic_Lock_t *config) {
    // Skip core handle check, this function uses other user facing APIs to do
    // all handle related work, it does not need to check the handle itself.
    int32_t status = PMIC_ST_SUCCESS;

    if (Pmic_validParamStatusCheck(config->validParams, PMIC_CFG_REG_LOCK_VALID, status)) {
        const uint8_t lockState = config->cfgLock ? PMIC_LOCK_ENABLE : PMIC_LOCK_DISABLE;
        Pmic_setRegLockState(handle, lockState);
    }

    if (Pmic_validParamStatusCheck(config->validParams, PMIC_CFG_CNT_LOCK_VALID, status)) {
        const uint8_t lockState = config->cntLock ? PMIC_LOCK_ENABLE : PMIC_LOCK_DISABLE;
        Pmic_setCntLockState(handle, lockState);
    }

    return status;
}

int32_t Pmic_getLockCfg(Pmic_CoreHandle_t *handle, Pmic_Lock_t *config) {
    int32_t status = Pmic_checkPmicCoreHandle(handle);
    uint8_t regData = 0U;

    // Read from REG_STAT_REG with critical section
    if (status == PMIC_ST_SUCCESS) {
        Pmic_criticalSectionStart(handle);
        status = Pmic_commIntf_recvByte(handle, REG_STAT_REG, &regData);
        Pmic_criticalSectionStop(handle);
    }

    // Extract requested bitfields from register data
    if (Pmic_validParamStatusCheck(config->validParams, PMIC_CFG_REG_LOCK_VALID, status)) {
        config->cfgLock = Pmic_getBitField_b(regData, CFG_REG_LOCKED_SHIFT, CFG_REG_LOCKED_MASK);
    }

    if (Pmic_validParamStatusCheck(config->validParams, PMIC_CFG_CNT_LOCK_VALID, status)) {
        config->cntLock = Pmic_getBitField_b(regData, CNT_REG_LOCKED_SHIFT, CNT_REG_LOCKED_MASK);
    }

    return status;
}

int32_t Pmic_getRegLockState(Pmic_CoreHandle_t *handle, uint8_t *lockState) {
    // Skip core handle check, this function uses other user facing APIs to do
    // all handle related work, it does not need to check the handle itself.
    int32_t status = PMIC_ST_SUCCESS;

    Pmic_Lock_t lockStatus = { .validParams = PMIC_CFG_REG_LOCK_VALID_SHIFT };
    status = Pmic_getLockCfg(handle, &lockStatus);

    if (status == PMIC_ST_SUCCESS) {
        *lockState = lockStatus.cfgLock ? PMIC_LOCK_ENABLE : PMIC_LOCK_DISABLE;
    }

    return status;
}

int32_t Pmic_getCntLockState(Pmic_CoreHandle_t *handle, uint8_t *lockState) {
    // Skip core handle check, this function uses other user facing APIs to do
    // all handle related work, it does not need to check the handle itself.
    int32_t status = PMIC_ST_SUCCESS;

    Pmic_Lock_t lockStatus = { .validParams = PMIC_CFG_CNT_LOCK_VALID_SHIFT };
    status = Pmic_getLockCfg(handle, &lockStatus);

    if (status == PMIC_ST_SUCCESS) {
        *lockState = lockStatus.cntLock ? PMIC_LOCK_ENABLE : PMIC_LOCK_DISABLE;
    }

    return status;
}