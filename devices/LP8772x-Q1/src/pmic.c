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
static int32_t initHandleBasicDevCfg(const Pmic_CoreCfg_t *config, Pmic_Handle_t *handle) {
    int32_t status = PMIC_ST_SUCCESS;

    /* Check and update PMIC Handle device type */
    if (Pmic_validParamCheck(config->validParams, PMIC_CFG_DEVICE_TYPE_VALID)) {
        if (config->pmicDeviceType != PMIC_DEV_COACH_LP8772X) {
            status = PMIC_ST_ERR_INV_PARAM;
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
            handle->commHandle0 = config->pCommHandle;
        }
    }

    /* Check and update PMIC Handle Comm Handle */
    if (Pmic_validParamStatusCheck(config->validParams, PMIC_CFG_QACOMM_HANDLE_VALID, status)) {
        if (config->pQACommHandle == NULL) {
            status = PMIC_ST_ERR_NULL_PARAM;
        } else {
            handle->commHandle1 = config->pQACommHandle;
        }
    }

    /* Assign PMIC slaveAddr */
    if (Pmic_validParamStatusCheck(config->validParams, PMIC_CFG_SLAVEADDR_VALID, status)) {
        handle->i2cAddr0 = config->slaveAddr;
    }

    /* Assign PMIC QA address */
    if (Pmic_validParamStatusCheck(config->validParams, PMIC_CFG_QASLAVEADDR_VALID, status)) {
        handle->i2cAddr1 = config->qaSlaveAddr;
    }

    /* Assign PMIC NVM address */
    if (Pmic_validParamStatusCheck(config->validParams, PMIC_CFG_NVMSLAVEADDR_VALID, status)) {
        handle->i2cAddr2 = config->nvmSlaveAddr;
    }

    /* Assign I2C1 speed */
    if (Pmic_validParamStatusCheck(config->validParams, PMIC_CFG_I2C1_SPEED_VALID, status)) {
        handle->i2c1Speed = config->i2c1Speed;
    }

    /* Assign I2C2 speed */
    if (Pmic_validParamStatusCheck(config->validParams, PMIC_CFG_I2C2_SPEED_VALID, status)) {
        handle->i2c2Speed = config->i2c2Speed;
    }

    return status;
}

static int32_t initCommsFunctions(const Pmic_CoreCfg_t *config, Pmic_Handle_t *handle) {
    int32_t status = PMIC_ST_SUCCESS;

    /* Check and update PMIC Handle Comm IO RD Fn */
    if (Pmic_validParamCheck(config->validParams, PMIC_CFG_COMM_IO_RD_VALID)) {
        if (!config->pFnPmicCommIoRd) {
            status = PMIC_ST_ERR_NULL_FPTR;
        } else {
            handle->ioRead = config->pFnPmicCommIoRd;
        }
    }

    /* Check and update PMIC Handle Comm IO WR Fn */
    if (Pmic_validParamStatusCheck(config->validParams, PMIC_CFG_COMM_IO_WR_VALID, status)) {
        if (!config->pFnPmicCommIoWr) {
            status = PMIC_ST_ERR_NULL_FPTR;
        } else {
            handle->ioWrite = config->pFnPmicCommIoWr;
        }
    }

    return status;
}

static int32_t initCritSecFunctions(const Pmic_CoreCfg_t *config, Pmic_Handle_t *handle) {
    int32_t status = PMIC_ST_SUCCESS;

    if (Pmic_validParamStatusCheck(config->validParams, PMIC_CFG_CRITSEC_START_VALID, status)) {
        if (!config->pFnPmicCritSecStart) {
            status = PMIC_ST_ERR_NULL_FPTR;
        } else {
            handle->criticalSectionStart = config->pFnPmicCritSecStart;
        }
    }

    /* Check and update PMIC Handle Critical Section Stop Fn */
    if (Pmic_validParamStatusCheck(config->validParams, PMIC_CFG_CRITSEC_STOP_VALID, status)) {
        if (!config->pFnPmicCritSecStop) {
            status = PMIC_ST_ERR_NULL_FPTR;
        } else {
            handle->criticalSectionStop = config->pFnPmicCritSecStop;
        }
    }

    return status;
}

static int32_t initCallbackFunctions(const Pmic_CoreCfg_t *config, Pmic_Handle_t *handle) {
    int32_t status = PMIC_ST_SUCCESS;

    if (Pmic_validParamStatusCheck(config->validParams, PMIC_CFG_PSEUDO_IRQ_VALID, status)) {
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
    Pmic_criticalSectionStart(handle);
    status = Pmic_ioRxByte(handle, PMIC_DEV_REV_REG, &regData);
    Pmic_criticalSectionStop(handle);

    if (status == PMIC_ST_SUCCESS) {
        /* Store device revision */
        handle->devRev = regData;

        /* Read MANUFACTURING_VER register */
        Pmic_criticalSectionStart(handle);
        status = Pmic_ioRxByte(handle, PMIC_MANUFACTURING_VER_REG, &regData);
        Pmic_criticalSectionStop(handle);
    }

    if (status == PMIC_ST_SUCCESS) {
        /* Store silicon revision */
        handle->devSiRev = regData;
    }

    return status;
}

static int32_t updateSubSysInfoAndValidateComms(const Pmic_CoreCfg_t *config, Pmic_Handle_t *handle) {
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regVal = 0U;

    /* Update PMIC subsystem info to PMIC handle */
    handle->pPmic_SubSysInfo = &pmicSubSysInfo[PMIC_DEV_COACH_LP8772X];

    /* Start Critical Section */
    Pmic_criticalSectionStart(handle);
    status = Pmic_ioRxByte(handle, PMIC_DEV_REV_REG, &regVal);
    Pmic_criticalSectionStop(handle);

    if (status == PMIC_ST_SUCCESS) {
        handle->drvInitStat |= config->instType;
    }

    return status;
}

static int32_t configureDeviceCrc(const Pmic_CoreCfg_t *config, Pmic_Handle_t *handle) {
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
    if (Pmic_validParamStatusCheck(config->validParams, PMIC_CFG_CRC_ENABLE_VALID, status)) {
        status = Pmic_ioSetCrcEnableState(handle, config->crcEnable);
    }

    // If the user requested that register CRC be enabled, re-enable it here, if
    // they listed this as a don't care, or explicitly marked it as disabled,
    // just leave it disabled.
    if (Pmic_validParamStatusCheck(config->validParams, PMIC_CFG_CFG_CRC_ENABLE_VALID, status)) {
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
int32_t Pmic_init(Pmic_Handle_t *handle, const Pmic_CoreCfg_t *config) {
    int32_t status = PMIC_ST_SUCCESS;

    if ((handle == NULL) || (config == NULL)) {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    /* Check and update PMIC Handle for device type, Comm Mode, I2C addresses */
    if (status == PMIC_ST_SUCCESS) {
        handle->drvInitStat = DRV_INIT_UNINIT;
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

    // Set up the valid subsystems for this device and ensure that we can
    // communicate with the selected IO interface.
    if (status == PMIC_ST_SUCCESS) {
        status = updateSubSysInfoAndValidateComms(config, handle);
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
        handle->drvInitStat |= DRV_INIT_SUCCESS;
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
        handle->pPmic_SubSysInfo = NULL;
        handle->drvInitStat = 0U;
        handle->devRev = 0U;
        handle->devSiRev = 0U;
        handle->commMode = 0U;
        handle->i2cAddr0 = 0U;
        handle->i2cAddr1 = 0U;
        handle->i2cAddr2 = 0U;
        handle->i2c1Speed = 0U;
        handle->i2c2Speed = 0U;
        handle->crcEnable = PMIC_DISABLE;
        handle->configCrcEnable = PMIC_DISABLE;
        handle->commHandle0 = NULL;
        handle->commHandle1 = NULL;
        handle->ioRead = (void *)0U;
        handle->ioWrite = (void *)0U;
        handle->criticalSectionStart = (void *)0U;
        handle->criticalSectionStop = (void *)0U;
        handle->irqResponseCallback = (void *)0U;
    }

    return status;
}

static inline uint32_t GetExpectedInitStatus(uint8_t commMode) {
    uint32_t expectedInitStatus = 0U;

    switch (commMode) {
        case PMIC_INTF_I2C_SINGLE:
            expectedInitStatus = (uint32_t)(DRV_INIT_SUCCESS | (uint8_t)PMIC_MAIN_INST);
            break;
        case PMIC_INTF_I2C_DUAL:
            expectedInitStatus = (uint32_t)(DRV_INIT_SUCCESS | (uint8_t)PMIC_MAIN_INST | (uint8_t)PMIC_QA_INST);
            break;
        case PMIC_INTF_SPI:
            expectedInitStatus = (uint32_t)(DRV_INIT_SUCCESS | (uint8_t)PMIC_MAIN_INST);
            break;
        default:
            expectedInitStatus = DRV_INIT_UNINIT;
            break;
    }

    return expectedInitStatus;
}

int32_t Pmic_checkHandle(const Pmic_Handle_t *handle) {
    int32_t status = PMIC_ST_SUCCESS;
    uint32_t expectedInitStatus = 0U;

    if ((handle == NULL) || (handle->commHandle0 == NULL)) {
        status = PMIC_ST_ERR_INV_HANDLE;
    }

    if ((status == PMIC_ST_SUCCESS) && !handle->ioRead) {
        status = PMIC_ST_ERR_NULL_FPTR;
    }

    if (status == PMIC_ST_SUCCESS) {
        expectedInitStatus = GetExpectedInitStatus(handle->commMode);
    }

    if ((status == PMIC_ST_SUCCESS) && (expectedInitStatus != handle->drvInitStat)) {
        status = PMIC_ST_ERR_INV_HANDLE;
    }

    return status;
}
