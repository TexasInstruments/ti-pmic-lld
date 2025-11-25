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
    if (Pmic_validParamCheck(config->validParams, PMIC_COMM_MODE_VALID)) {
        if (config->commMode != PMIC_INTF_I2C_SINGLE) {
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

    /* Assign PMIC i2cAddr0 */
    if (Pmic_validParamStatusCheck(config->validParams, PMIC_I2C_ADDR0_VALID, status)) {
        handle->i2cAddr0 = config->i2cAddr0;
    }

    /* Assign PMIC i2cAddr1 */
    if (Pmic_validParamStatusCheck(config->validParams, PMIC_I2C_ADDR1_VALID, status)) {
        handle->i2cAddr1 = config->i2cAddr1;
    }

    /* Assign PMIC i2cAddr2 */
    if (Pmic_validParamStatusCheck(config->validParams, PMIC_I2C_ADDR2_VALID, status)) {
        handle->i2cAddr2 = config->i2cAddr2;
    }

    return status;
}

static int32_t initCommsFunctions(const Pmic_HandleCfg_t *config, Pmic_Handle_t *handle) {
    int32_t status = PMIC_ST_SUCCESS;

    /* Check and update PMIC Handle Comm IO RD Fn */
    if (Pmic_validParamCheck(config->validParams, PMIC_IO_READ_VALID)) {
        if (!config->ioRead) {
            status = PMIC_ST_ERR_NULL_FPTR;
        } else {
            handle->ioRead = config->ioRead;
        }
    }

    /* Check and update PMIC Handle Comm IO WR Fn */
    if (Pmic_validParamStatusCheck(config->validParams, PMIC_IO_WRITE_VALID, status)) {
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

    if (Pmic_validParamStatusCheck(config->validParams, PMIC_CRITICAL_SECTION_START_VALID, status)) {
        if (!config->criticalSectionStart) {
            status = PMIC_ST_ERR_NULL_FPTR;
        } else {
            handle->criticalSectionStart = config->criticalSectionStart;
        }
    }

    /* Check and update PMIC Handle Critical Section Stop Fn */
    if (Pmic_validParamStatusCheck(config->validParams, PMIC_CRITICAL_SECTION_STOP_VALID, status)) {
        if (!config->criticalSectionStop) {
            status = PMIC_ST_ERR_NULL_FPTR;
        } else {
            handle->criticalSectionStop = config->criticalSectionStop;
        }
    }

    return status;
}

static int32_t initCallbackFunctions(const Pmic_HandleCfg_t *config, Pmic_Handle_t *handle) {
    int32_t status = PMIC_ST_SUCCESS;

    if (Pmic_validParamStatusCheck(config->validParams, PMIC_IRQ_RESPONSE_CALLBACK_VALID, status)) {
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
    if (Pmic_validParamStatusCheck(config->validParams, PMIC_CRC_ENABLE_VALID, status)) {
        status = Pmic_ioSetCrcEnableState(handle, config->crcEnable);
    }

    // If the user requested that register CRC be enabled, re-enable it here, if
    // they listed this as a don't care, or explicitly marked it as disabled,
    // just leave it disabled.
    if (Pmic_validParamStatusCheck(config->validParams, PMIC_CONFIG_CRC_ENABLE_VALID, status)) {
        handle->configCrcEnable = config->configCrcEnable;

        if (config->configCrcEnable) {
            status = Pmic_configCrcEnable(handle, PMIC_CFG_CRC_RECALCULATE);
        }
    }

    // Re-lock config register space
    if (status == PMIC_ST_SUCCESS) {
        status = Pmic_setRegLockState(handle, PMIC_LOCK_ENABLE);
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

    /* Check and update PMIC Handle for device type, Comm Mode, I2C addresses */
    if (status == PMIC_ST_SUCCESS) {
        handle->drvInitStat = PMIC_DRV_INIT_UNINIT;
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

    // Initialize any other user provided callback functions
    if (status == PMIC_ST_SUCCESS) {
        status = initCallbackFunctions(config, handle);
    }

    // Validate communication with the device.
    if (status == PMIC_ST_SUCCESS) {
        status = validateComms(handle);
    }

    /* Check for required members for I2C/SPI Main handle comm */
    if ((status == PMIC_ST_SUCCESS) &&
         (!handle->criticalSectionStart ||
          !handle->criticalSectionStop  ||
          !handle->ioRead     ||
          !handle->ioWrite)) {
        status = PMIC_ST_ERR_INSUFFICIENT_CFG;
    }

    // Initialization is complete, mark it with magic.
    if (status == PMIC_ST_SUCCESS) {
        handle->drvInitStat = PMIC_DRV_INIT_SUCCESS;
    }

    // Configure CRC for HW and PMIC handle
    if (status == PMIC_ST_SUCCESS) {
        status = configureDeviceCrc(config, handle);
    }

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
        handle->crcEnable = PMIC_DISABLE;
        handle->configCrcEnable = PMIC_DISABLE;
        handle->commHandle0 = NULL;
        handle->ioRead = (void *)0U;
        handle->ioWrite = (void *)0U;
        handle->criticalSectionStart = (void *)0U;
        handle->criticalSectionStop = (void *)0U;
        handle->irqResponseCallback = (void *)0U;
    }

    return status;
}

int32_t Pmic_checkHandle(const Pmic_Handle_t *handle) {
    int32_t status = PMIC_ST_SUCCESS;

    if ((handle == NULL) || (handle->commHandle0 == NULL)) {
        status = PMIC_ST_ERR_INV_HANDLE;
    }

    if ((status == PMIC_ST_SUCCESS) && !handle->ioRead) {
        status = PMIC_ST_ERR_NULL_FPTR;
    }

    if ((status == PMIC_ST_SUCCESS) && (PMIC_DRV_INIT_SUCCESS != handle->drvInitStat)) {
        status = PMIC_ST_ERR_INV_HANDLE;
    }

    return status;
}
