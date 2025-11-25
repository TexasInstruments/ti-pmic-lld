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
   avoid corrupted PmicHandle usage. */
#define PMIC_DRV_INIT_SUCCESS (uint32_t)(0x504D4943U) /* "PMIC" in ASCII */
#define PMIC_DRV_INIT_UNINIT  (uint32_t)(0x00000000U)

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

static int32_t validateComms(Pmic_Handle_t *handle) {
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regVal = 0U;

    /* Start Critical Section */
    status = Pmic_ioRxByte_CS(handle, PMIC_WD_LONGWIN_CFG_REG, &regVal);

    if (status == PMIC_ST_SUCCESS) {
        handle->drvInitStat = PMIC_DRV_INIT_SUCCESS;
    }

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

    // Get PMIC info, store info in pmic handle
    if (status == PMIC_ST_SUCCESS) {
        status = getPmicInfo(handle);
    }

    // Validate communication with the device.
    if (status == PMIC_ST_SUCCESS) {
        status = validateComms(handle);
    }

    /* Check for required members for I2C/SPI Main handle comm */
    if ((status == PMIC_ST_SUCCESS) &&
         ((handle->criticalSectionStart == NULL) ||
          (handle->criticalSectionStop == NULL) ||
          (handle->ioRead == NULL) ||
          (handle->ioWrite == NULL))) {
        status = PMIC_ST_ERR_INSUFFICIENT_CFG;
    }

    // Initialization is complete, mark it with magic.
    if (status == PMIC_ST_SUCCESS) {
        handle->drvInitStat = PMIC_DRV_INIT_SUCCESS;
    }

    return status;
}

int32_t Pmic_deinit(Pmic_Handle_t *handle) {
    int32_t status = PMIC_ST_SUCCESS;

    if (handle == NULL) {
        status = PMIC_ST_ERR_INV_HANDLE;
    }

    if (status == PMIC_ST_SUCCESS) {
        handle->commHandle0 = NULL;
        handle->criticalSectionStart = NULL;
        handle->criticalSectionStop = NULL;
        handle->ioRead = NULL;
        handle->ioWrite = NULL;
        handle->drvInitStat = PMIC_DRV_INIT_UNINIT;
        handle->devRev = 0U;
        handle->devSiRev = 0U;
        handle->nvmCode = 0U;
        handle->nvmRev = 0U;
    }

    return status;
}

int32_t Pmic_checkHandle(const Pmic_Handle_t *handle) {
    int32_t status = PMIC_ST_SUCCESS;

    if ((handle == NULL) || (handle->commHandle0 == NULL)) {
        status = PMIC_ST_ERR_INV_HANDLE;
    }

    if ((status == PMIC_ST_SUCCESS) && (handle->ioRead == NULL)) {
        status = PMIC_ST_ERR_NULL_FPTR;
    }

    if ((status == PMIC_ST_SUCCESS) && (PMIC_DRV_INIT_SUCCESS != handle->drvInitStat)) {
        status = PMIC_ST_ERR_INV_HANDLE;
    }

    return status;
}
