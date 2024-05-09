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
 *  @file  pmic_core.c
 *
 *  @brief This file contains PMIC generic driver APIs
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include "pmic_core.h"

#include <stddef.h>

#include "pmic_io.h"
#include "regmap/core.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
#define PMIC_TPS65386X_DEV_ID  (0x3CU)

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
inline bool Pmic_validParamCheck(uint32_t validParamVal, uint8_t bitPos) {
    return (((validParamVal >> bitPos) & 0x01U) != 0U);
}

inline bool Pmic_validParamStatusCheck(uint32_t validParamVal, uint8_t bitPos, int32_t status) {
    return (status == PMIC_ST_SUCCESS) && Pmic_validParamCheck(validParamVal, bitPos);
}

/**
 * @brief Start a critical section for PMIC operations.
 * This function starts a critical section for PMIC operations, if the critical
 * section start function pointer is not NULL.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle structure.
 * @return void No return value.
 */
void Pmic_criticalSectionStart(const Pmic_CoreHandle_t *pPmicCoreHandle) {
    if (NULL != pPmicCoreHandle->pFnPmicCritSecStart) {
        pPmicCoreHandle->pFnPmicCritSecStart();
    }
}

/**
 * @brief Stop a critical section for PMIC operations.
 * This function stops a critical section for PMIC operations, if the critical
 * section stop function pointer is not NULL.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle structure.
 * @return void No return value.
 */
void Pmic_criticalSectionStop(const Pmic_CoreHandle_t *pPmicCoreHandle) {
    if (NULL != pPmicCoreHandle->pFnPmicCritSecStop) {
        pPmicCoreHandle->pFnPmicCritSecStop();
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
        if (handle->commMode == PMIC_INTF_SINGLE_I2C) {
            expectedInitStatus = (uint32_t)(DRV_INIT_SUCCESS | (uint8_t)PMIC_MAIN_INST);
        } else if (handle->commMode == PMIC_INTF_DUAL_I2C) {
            expectedInitStatus = (uint32_t)(DRV_INIT_SUCCESS | (uint8_t)PMIC_MAIN_INST | (uint8_t)PMIC_QA_INST);
        } else if (handle->commMode == PMIC_INTF_SPI) {
            expectedInitStatus = (uint32_t)(DRV_INIT_SUCCESS | (uint8_t)PMIC_MAIN_INST);
        } else {
            expectedInitStatus = 0x00U;
        }

        if (expectedInitStatus != handle->drvInitStatus) {
            status = PMIC_ST_ERR_INV_HANDLE;
        }
    }

    return status;
}

/**
 * @brief Set register lock/unlock configuration.
 * This function sets the register lock/unlock configuration based on the
 * provided parameters.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle structure.
 * @param commonCtrlCfg Common control configuration structure.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
int32_t Pmic_setRegisterLockUnlock(Pmic_CoreHandle_t *pPmicCoreHandle,
    const Pmic_CommonCtrlCfg_t commonCtrlCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    if (PMIC_ST_SUCCESS == pmicStatus) {
        Pmic_criticalSectionStart(pPmicCoreHandle);

        pmicStatus = Pmic_commIntf_sendByte(
            pPmicCoreHandle, PMIC_REGISTER_UNLOCK_REGADDR, commonCtrlCfg.regLock_1);

        pmicStatus = Pmic_commIntf_sendByte(
            pPmicCoreHandle, PMIC_REGISTER_UNLOCK_REGADDR, commonCtrlCfg.regLock_2);

        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    return pmicStatus;
}

/**
 * @brief Get register lock status.
 * This function retrieves the register lock status.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle structure.
 * @param pCommonCtrlStat Pointer to the common control status structure to
 * store the retrieved status.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
int32_t Pmic_getRegLockStat(Pmic_CoreHandle_t *pPmicCoreHandle,
    Pmic_CommonCtrlStat_t *pCommonCtrlStat) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    Pmic_criticalSectionStart(pPmicCoreHandle);

    pmicStatus = Pmic_commIntf_recvByte(
        pPmicCoreHandle, PMIC_REG_LOCK_STATUS_REGADDR, &pCommonCtrlStat->cfgregLockStat);

    Pmic_criticalSectionStop(pPmicCoreHandle);

    if (PMIC_ST_SUCCESS == pmicStatus) {
        pCommonCtrlStat->cfgregLockStat = Pmic_getBitField(
            pCommonCtrlStat->cfgregLockStat,
            PMIC_CFGREG_LOCKED_STATUS_SHIFT,
            PMIC_CFGREG_LOCK_STATUS_RD_MASK);
    }

    return pmicStatus;
}

/**
 * @brief Initialize the PMIC core handle with basic device configuration
 * parameters. This function initializes the PMIC core handle with basic device
 * configuration parameters.
 *
 * @param pPmicConfigData Pointer to the PMIC core configuration data structure.
 * @param pPmicCoreHandle Pointer to the PMIC core handle structure to be
 * initialized.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
int32_t Pmic_initCoreHandleBasicDevCfgParams(
    const Pmic_CoreCfg_t *pPmicConfigData,
    Pmic_CoreHandle_t *pPmicCoreHandle)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    /* Check and update PMIC Handle device type */
    if (Pmic_validParamCheck(pPmicConfigData->validParams, PMIC_CFG_DEVICE_TYPE_VALID)) {
        if (PMIC_DEV_BB_TPS65386X != pPmicConfigData->pmicDeviceType) {
            pmicStatus = PMIC_ST_ERR_INV_PARAM;
        }

        if (PMIC_ST_SUCCESS == pmicStatus) {
            pPmicCoreHandle->pmicDeviceType = pPmicConfigData->pmicDeviceType;
        }
    }

    /* Check and update PMIC Handle Comm Mode */
    if ((PMIC_ST_SUCCESS == pmicStatus) &&
        (Pmic_validParamCheck(pPmicConfigData->validParams, PMIC_CFG_COMM_MODE_VALID))) {
        if (PMIC_INTF_SPI != pPmicConfigData->commMode) {
            pmicStatus = PMIC_ST_ERR_INV_PARAM;
        } else {
            pPmicCoreHandle->commMode = pPmicConfigData->commMode;
        }

        if (PMIC_ST_SUCCESS != pmicStatus) {
            pmicStatus = PMIC_ST_ERR_FAIL;
        }
    }

    /* Check and update PMIC Handle Comm Handle */
    if ((PMIC_ST_SUCCESS == pmicStatus) &&
        (Pmic_validParamCheck(pPmicConfigData->validParams, PMIC_CFG_COMM_HANDLE_VALID))) {
        if (NULL == pPmicConfigData->pCommHandle) {
            pmicStatus = PMIC_ST_ERR_INV_PARAM;
        } else {
            pPmicCoreHandle->pCommHandle = pPmicConfigData->pCommHandle;
        }

        if (PMIC_ST_SUCCESS != pmicStatus) {
            pmicStatus = PMIC_ST_ERR_FAIL;
        }
    }

    return pmicStatus;
}

/**
 * @brief Initialize the PMIC core handle with communication I/O critical
 * section function pointers. This function initializes the PMIC core handle
 * with communication I/O critical section function pointers.
 *
 * @param pPmicConfigData Pointer to the PMIC core configuration data structure.
 * @param pPmicCoreHandle Pointer to the PMIC core handle structure to be
 * initialized.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
int32_t Pmic_initCoreHandleCommIOCriticalSectionFns(
    const Pmic_CoreCfg_t *pPmicConfigData,
    Pmic_CoreHandle_t *pPmicCoreHandle)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    /* Check and update PMIC Handle Comm IO RD Fn */
    if (Pmic_validParamCheck(pPmicConfigData->validParams, PMIC_CFG_COMM_IO_RD_VALID)) {
        if (NULL == pPmicConfigData->pFnPmicCommIoRead) {
            pmicStatus = PMIC_ST_ERR_NULL_FPTR;
        } else {
            pPmicCoreHandle->pFnPmicCommIoRead = pPmicConfigData->pFnPmicCommIoRead;
        }
    }

    /* Check and update PMIC Handle Comm IO WR Fn */
    if ((PMIC_ST_SUCCESS == pmicStatus) &&
        (Pmic_validParamCheck(pPmicConfigData->validParams, PMIC_CFG_COMM_IO_WR_VALID))) {
        if (NULL == pPmicConfigData->pFnPmicCommIoWrite) {
            pmicStatus = PMIC_ST_ERR_NULL_FPTR;
        } else {
            pPmicCoreHandle->pFnPmicCommIoWrite = pPmicConfigData->pFnPmicCommIoWrite;
        }
    }

    /* Check and update PMIC Handle Critical Section Start Fn */
    if ((PMIC_ST_SUCCESS == pmicStatus) &&
        (Pmic_validParamCheck(pPmicConfigData->validParams, PMIC_CFG_CRITSEC_START_VALID))) {
        if (NULL == pPmicConfigData->pFnPmicCritSecStart) {
            pmicStatus = PMIC_ST_ERR_NULL_FPTR;
        } else {
            pPmicCoreHandle->pFnPmicCritSecStart = pPmicConfigData->pFnPmicCritSecStart;
        }
    }

    /* Check and update PMIC Handle Critical Section Stop Fn */
    if ((PMIC_ST_SUCCESS == pmicStatus) &&
        (Pmic_validParamCheck(pPmicConfigData->validParams, PMIC_CFG_CRITSEC_STOP_VALID))) {
        if (NULL == pPmicConfigData->pFnPmicCritSecStop) {
            pmicStatus = PMIC_ST_ERR_NULL_FPTR;
        } else {
            pPmicCoreHandle->pFnPmicCritSecStop = pPmicConfigData->pFnPmicCritSecStop;
        }
    }

    return pmicStatus;
}

/**
 * @brief Validate the presence of the device on the bus.
 * This function validates the presence of the device on the bus.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle structure.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
int32_t Pmic_validateDevOnBus(Pmic_CoreHandle_t *pPmicCoreHandle) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regVal = 0U;
    if (PMIC_ST_SUCCESS != pmicStatus) {
        pmicStatus = PMIC_ST_ERR_FAIL;
    }

    /* Start Critical Section */
    Pmic_criticalSectionStart(pPmicCoreHandle);

    pmicStatus =
        Pmic_commIntf_sendByte(pPmicCoreHandle, PMIC_DEV_ID_REGADDR, regVal);

    if (PMIC_ST_SUCCESS != pmicStatus) {
        pmicStatus = PMIC_ST_ERR_FAIL;
    }

    pmicStatus =
        Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_DEV_ID_REGADDR, & regVal);

    Pmic_criticalSectionStop(pPmicCoreHandle);
    if (PMIC_ST_SUCCESS != pmicStatus) {
        pmicStatus = PMIC_ST_ERR_FAIL;
    }

    if (PMIC_ST_SUCCESS == pmicStatus) {
        pPmicCoreHandle -> pmicDevRev = Pmic_getBitField(regVal, PMIC_DEV_ID_SHIFT, PMIC_DEV_ID_MASK);

        /* Validate if the device requested is the one on the bus */
        if (PMIC_DEV_BB_TPS65386X == (pPmicCoreHandle->pmicDeviceType)) {
            if (PMIC_TPS65386X_DEV_ID != pPmicCoreHandle->pmicDevRev) {
                pmicStatus = PMIC_ST_WARN_INV_DEVICE_ID;
            }
        }
    }

    if (PMIC_ST_SUCCESS != pmicStatus) {
        pmicStatus = PMIC_ST_ERR_FAIL;
    }

    return pmicStatus;
}

/**
 * @brief Update subsystem information and validate main Q&A communication
 * interface read/write.
 * This function updates subsystem information and validates the main Q&A
 * communication interface read/write.
 *
 * @param pPmicConfigData Pointer to the PMIC core configuration data structure.
 * @param pPmicCoreHandle Pointer to the PMIC core handle structure to be
 * updated.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
int32_t Pmic_updateSubSysInfoValidateMainQaCommIFRdWr(
    const Pmic_CoreCfg_t *pPmicConfigData, Pmic_CoreHandle_t *pPmicCoreHandle) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regVal = 0U;

    /* Update PMIC subsystem info to PMIC handle */
    pPmicCoreHandle->pPmic_SubSysInfo = &pmicSubSysInfo[pPmicCoreHandle->pmicDeviceType];

    if (PMIC_ST_SUCCESS != pmicStatus) {
        pmicStatus = PMIC_ST_ERR_FAIL;
    }

    if (PMIC_ST_SUCCESS == pmicStatus) {
        /* Start Critical Section */
        Pmic_criticalSectionStart(pPmicCoreHandle);

        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
            PMIC_WD_LONGWIN_CFG_REG, &regVal);

        Pmic_criticalSectionStop(pPmicCoreHandle);

        if (PMIC_ST_SUCCESS != pmicStatus) {
            pmicStatus = PMIC_ST_ERR_FAIL;
        }

        if (PMIC_ST_SUCCESS == pmicStatus) {
            pPmicCoreHandle->drvInitStatus |= pPmicConfigData->instType;
        }
    }

    if (PMIC_ST_SUCCESS != pmicStatus) {
        pmicStatus = PMIC_ST_ERR_FAIL;
    }

    return pmicStatus;
}

/**
 * @brief Initialize the PMIC core.
 * This function initializes the PMIC core based on the provided configuration
 * data.
 *
 * @param pPmicConfigData Pointer to the PMIC core configuration data structure.
 * @param pPmicCoreHandle Pointer to the PMIC core handle structure to be
 * initialized.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
int32_t Pmic_init(const Pmic_CoreCfg_t *pPmicConfigData,
    Pmic_CoreHandle_t *pPmicCoreHandle) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    if ((NULL == pPmicCoreHandle) || (NULL == pPmicConfigData)) {
        pmicStatus = PMIC_ST_ERR_NULL_PARAM;
    }

    if (PMIC_ST_SUCCESS != pmicStatus) {
        pmicStatus = PMIC_ST_ERR_FAIL;
    }

    /* Check and update PMIC Handle for device type, Comm Mode, Main Slave Address
     * and NVM Slave Address */
    if (PMIC_ST_SUCCESS == pmicStatus) {
        pmicStatus = Pmic_initCoreHandleBasicDevCfgParams(pPmicConfigData, pPmicCoreHandle);
    }

    if (PMIC_ST_SUCCESS != pmicStatus) {
        pmicStatus = PMIC_ST_ERR_FAIL;
    }

    /* Check and update PMIC Handle for Comm IO RD Fn, Comm IO Wr Fn, Critical
     * Section Start Fn and Critical Section Stop Fn */
    if (PMIC_ST_SUCCESS == pmicStatus) {
        pmicStatus = Pmic_initCoreHandleCommIOCriticalSectionFns(pPmicConfigData, pPmicCoreHandle);
    }

    if (PMIC_ST_SUCCESS != pmicStatus) {
        pmicStatus = PMIC_ST_ERR_FAIL;
    }

    /* Check for required members for I2C/SPI Main handle comm */
    if ((PMIC_ST_SUCCESS == pmicStatus) &&
        ((NULL == pPmicCoreHandle->pFnPmicCritSecStart) ||
            (NULL == pPmicCoreHandle->pFnPmicCritSecStop) ||
            (NULL == pPmicCoreHandle->pFnPmicCommIoRead) ||
            (NULL == pPmicCoreHandle->pFnPmicCommIoWrite))) {
        pmicStatus = PMIC_ST_ERR_INSUFFICIENT_CFG;
    }

    return pmicStatus;
}

/**
 * @brief Deinitialize the PMIC core.
 * This function deinitializes the PMIC core.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle structure.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
int32_t Pmic_deinit(Pmic_CoreHandle_t *pPmicCoreHandle) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    if (NULL == pPmicCoreHandle) {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    }

    if (PMIC_ST_SUCCESS == pmicStatus) {
        pPmicCoreHandle->pCommHandle = NULL;
        pPmicCoreHandle->pQACommHandle = NULL;
        pPmicCoreHandle->pFnPmicCritSecStart = NULL;
        pPmicCoreHandle->pFnPmicCritSecStop = NULL;
        pPmicCoreHandle->pFnPmicCommIoRead = NULL;
        pPmicCoreHandle->pFnPmicCommIoWrite = NULL;
        pPmicCoreHandle->pPmic_SubSysInfo = NULL;
        pPmicCoreHandle->drvInitStatus = 0x00U;
    }

    return pmicStatus;
}
