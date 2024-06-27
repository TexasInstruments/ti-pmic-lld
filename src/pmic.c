#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#include "pmic.h"
#include "pmic_common.h"
#include "pmic_io.h"
#include "regmap/core.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
/* PMIC driver Core Handle INIT status Magic Number. Used to validate Handle to
   avoid corrupted PmicHandle usage.

   Note that this is used as the upper 16-bits of a 32-bit bitfield, the lower
   16-bits should be left 0. */
#define DRV_INIT_SUCCESS (uint32_t)(0xBEEF0000U)
#define DRV_INIT_UNINIT  (uint32_t)(0x00000000U)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */
static const Pmic_DevSubSysInfo_t pmicSubSysInfo[] = {
    {
        .gpioEnable = PMIC_DISABLE,
        .wdgEnable = PMIC_ENABLE,
        .buckEnable = PMIC_DISABLE,
        .ldoEnable = PMIC_DISABLE,
        .esmEnable = PMIC_DISABLE,
    }
};

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */
static int32_t initHandleBasicDevCfg(const Pmic_CoreCfg_t *config, Pmic_CoreHandle_t *handle) {
    int32_t status = PMIC_ST_SUCCESS;

    /* Check and update PMIC Handle device type */
    if (Pmic_validParamCheck(config->validParams, PMIC_CFG_DEVICE_TYPE_VALID)) {
        if (config->pmicDeviceType != PMIC_DEV_COACH_LP8772X) {
            status = PMIC_ST_ERR_INV_PARAM;
        }

        if (status == PMIC_ST_SUCCESS) {
            handle->pmicDeviceType = config->pmicDeviceType;
        }
    }

    /* Check and update PMIC Handle Comm Mode */
    if (Pmic_validParamStatusCheck(config->validParams, PMIC_CFG_COMM_MODE_VALID, status)) {
        if (config->commMode != PMIC_INTF_I2C_SINGLE) {
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

    /* Assign PMIC slaveAddr */
    if (Pmic_validParamStatusCheck(config->validParams, PMIC_CFG_SLAVEADDR_VALID, status)) {
        handle->slaveAddr = config->slaveAddr;
    }

    return status;
}

static int32_t initCommsFunctions(const Pmic_CoreCfg_t *config, Pmic_CoreHandle_t *handle) {
    int32_t status = PMIC_ST_SUCCESS;

    /* Check and update PMIC Handle Comm IO RD Fn */
    if (Pmic_validParamCheck(config->validParams, PMIC_CFG_COMM_IO_RD_VALID)) {
        if (config->pFnPmicCommIoRd == NULL) {
            status = PMIC_ST_ERR_NULL_FPTR;
        } else {
            handle->pFnPmicCommIoRd = config->pFnPmicCommIoRd;
        }
    }

    /* Check and update PMIC Handle Comm IO WR Fn */
    if (Pmic_validParamStatusCheck(config->validParams, PMIC_CFG_COMM_IO_WR_VALID, status)) {
        if (config->pFnPmicCommIoWr == NULL) {
            status = PMIC_ST_ERR_NULL_FPTR;
        } else {
            handle->pFnPmicCommIoWr = config->pFnPmicCommIoWr;
        }
    }

    return status;
}

static int32_t initCritSecFunctions(const Pmic_CoreCfg_t *config, Pmic_CoreHandle_t *handle) {
    int32_t status = PMIC_ST_SUCCESS;

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

static int32_t updateSubSysInfoAndValidateComms(const Pmic_CoreCfg_t *config, Pmic_CoreHandle_t *handle) {
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regVal = 0U;

    /* Update PMIC subsystem info to PMIC handle */
    handle->pPmic_SubSysInfo = &pmicSubSysInfo[handle->pmicDeviceType];

    /* Start Critical Section */
    Pmic_criticalSectionStart(handle);
    status = Pmic_ioRxByte(handle, PMIC_DEV_ID_REG, &regVal);
    Pmic_criticalSectionStop(handle);

    if (status == PMIC_ST_SUCCESS) {
        handle->drvInitStatus |= config->instType;
    }

    return status;
}

/* ========================================================================== */
/*                        Interface Implementations                           */
/* ========================================================================== */
int32_t Pmic_init(Pmic_CoreHandle_t *handle, const Pmic_CoreCfg_t *config) {
    int32_t status = PMIC_ST_SUCCESS;

    if ((handle == NULL) || (config == NULL)) {
        status = PMIC_ST_ERR_NULL_PARAM;
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

    // Set up the valid subsystems for this device and ensure that we can
    // communicate with the selected IO interface.
    if (status == PMIC_ST_SUCCESS) {
        status = updateSubSysInfoAndValidateComms(config, handle);
    }

    /* Check for required members for I2C/SPI Main handle comm */
    if ((status == PMIC_ST_SUCCESS) &&
         ((handle->pFnPmicCritSecStart == NULL) ||
          (handle->pFnPmicCritSecStop == NULL) ||
          (handle->pFnPmicCommIoRd == NULL) ||
          (handle->pFnPmicCommIoWr == NULL))) {
        status = PMIC_ST_ERR_INSUFFICIENT_CFG;
    }

    // Initialization is complete, mark it with magic.
    if (status == PMIC_ST_SUCCESS) {
        handle->drvInitStatus |= DRV_INIT_SUCCESS;
    }

    return status;
}

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
        handle->pFnPmicCommIoRd = NULL;
        handle->pFnPmicCommIoWr = NULL;
        handle->pPmic_SubSysInfo = NULL;
        handle->drvInitStatus = 0x00U;
    }

    return status;
}

int32_t Pmic_checkPmicCoreHandle(const Pmic_CoreHandle_t *handle) {
    int32_t status = PMIC_ST_SUCCESS;
    uint32_t expectedInitStatus = 0U;

    if ((handle == NULL) || (handle->pCommHandle == NULL)) {
        status = PMIC_ST_ERR_INV_HANDLE;
    }

    if ((status == PMIC_ST_SUCCESS) && (handle->pFnPmicCommIoRd == NULL)) {
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
    }

    if ((status == PMIC_ST_SUCCESS) && (expectedInitStatus != handle->drvInitStatus)) {
        status = PMIC_ST_ERR_INV_HANDLE;
    }

    return status;
}
