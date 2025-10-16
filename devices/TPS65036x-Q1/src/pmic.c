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
 * @file pmic.c
 *
 * @brief This file contains definitions of the PMIC LLD initialization and
 * deinitialization APIs.
 */
#include "pmic.h"
#include "pmic_io.h"
#include "pmic_common.h"

#include "regmap/core.h"
#include "regmap/irq.h"

// BIT3 of SILICON_REV[7:0] identifies whether the PMIC is PG1 (A0) or PG2 (B1)
#define DEVICE_PG_IDENTIFER (3U)

// Used to unlock PMIC registers
#define REG_LOCK_KEY (0x9BU)

// Used to lock PMIC registers
#define REG_LOCK_VALUE (0xAAU)

static inline void setPmicHandleMembers(const Pmic_CoreCfg_t *pmicCfg, Pmic_CoreHandle_t *pmicHandle)
{
    pmicHandle->i2cAddr = pmicCfg->i2cAddr;
    pmicHandle->commHandle = pmicCfg->commHandle;
    pmicHandle->ioRead = pmicCfg->ioRead;
    pmicHandle->ioWrite = pmicCfg->ioWrite;
    pmicHandle->critSecStart = pmicCfg->critSecStart;
    pmicHandle->critSecStop = pmicCfg->critSecStop;
    pmicHandle->irqResponse = pmicCfg->irqResponse;
}

static int32_t getPmicInfo(Pmic_CoreHandle_t *pmicHandle)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    // Read INTERFACE_CONF register
    Pmic_criticalSectionStart(pmicHandle);
    status = Pmic_ioRxByte(pmicHandle, PMIC_INTERFACE_CONF_REGADDR, &regData);

    if (status == PMIC_ST_SUCCESS)
    {
        // Extract I2C_CRC_EN bit field and read DEV_REV register
        pmicHandle->crcEnable = Pmic_getBitField_b(regData, PMIC_I2C_CRC_EN_SHIFT);
        status = Pmic_ioRxByte(pmicHandle, PMIC_DEV_REV_REGADDR, &regData);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Extract TI_DEVICE_ID bit field and read NVM_CODE_1 register
        pmicHandle->devRev = regData;
        status = Pmic_ioRxByte(pmicHandle, PMIC_NVM_CODE_1_REGADDR, &regData);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Extract TI_NVM_ID bit field and read NVM_CODE_2 register
        pmicHandle->nvmId = regData;
        status = Pmic_ioRxByte(pmicHandle, PMIC_NVM_CODE_2_REGADDR, &regData);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Extract TI_NVM_REV bit field and read MANUFACTURING_VER register
        pmicHandle->nvmRev = regData;
        status = Pmic_ioRxByte(pmicHandle, PMIC_MANUFACTURING_VER_REGADDR, &regData);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Extract SILICON_REV bit field
        pmicHandle->siliconRev = regData;
    }
    Pmic_criticalSectionStop(pmicHandle);

    return status;
}

static int32_t decipherWhetherA0(Pmic_CoreHandle_t *pmicHandle)
{
    uint8_t regData = 0U;
    bool regsLocked = (bool)false;
    int32_t status = PMIC_ST_SUCCESS;

    // Get register lock status
    status = Pmic_ioRxByte(pmicHandle, PMIC_REGISTER_LOCK_REGADDR, &regData);

    // Unlock registers if they are locked
    if (status == PMIC_ST_SUCCESS)
    {
        regsLocked = Pmic_getBitField_b(regData, PMIC_REGISTER_LOCK_STATUS_SHIFT);

        if (regsLocked)
        {
            status = Pmic_ioTxByte(pmicHandle, PMIC_REGISTER_LOCK_REGADDR, REG_LOCK_KEY);
        }
    }

    // Set NRSTOUT_READBACK_MASK to 1. Only A0 silicon has this bit field.
    // So if NRSTOUT_READBACK_MASK is writable, the device is A0
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioReadModifyWrite_b(pmicHandle, PMIC_MASK_MODERATE_ERR_REGADDR, PMIC_NRSTOUT_READBACK_MASK_SHIFT, (bool)true);
    }

    // Get actual NRSTOUT_READBACK_MASK value
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioRxByte(pmicHandle, PMIC_MASK_MODERATE_ERR_REGADDR, &regData);
    }

    // Device is A0 if NRSTOUT_READBACK_MASK is 1. Otherwise, device is B0.
    // NOTE: NRSTOUT_READBACK_MASK is recommended to be set to 1; it is not
    // necessary to revert it back to 0 if it was previously 0
    if ((status == PMIC_ST_SUCCESS) && Pmic_getBitField_b(regData, PMIC_NRSTOUT_READBACK_MASK_SHIFT))
    {
        pmicHandle->isA0 = (bool)true;
    }

    // Re-lock PMIC registers if they were previously locked
    if ((status == PMIC_ST_SUCCESS) && regsLocked)
    {

        status = Pmic_ioTxByte(pmicHandle, PMIC_REGISTER_LOCK_REGADDR, REG_LOCK_VALUE);
    }

    return status;
}

int32_t Pmic_init(const Pmic_CoreCfg_t *pmicCfg, Pmic_CoreHandle_t *pmicHandle)
{
    int32_t status = PMIC_ST_SUCCESS;

    // Check whether parameters are valid
    if ((pmicCfg == NULL) || (pmicCfg->commHandle == NULL) || (pmicCfg->ioRead == NULL) ||
        (pmicCfg->ioWrite == NULL) || (pmicCfg->critSecStart == NULL) ||
        (pmicCfg->critSecStop == NULL) || (pmicHandle == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Initialize PMIC handle with values from pmicCfg
        setPmicHandleMembers(pmicCfg, pmicHandle);

        // Get PMIC info, store info in pmic handle
        status = getPmicInfo(pmicHandle);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        pmicHandle->isA0 = (bool)false;
        const bool isB1 = Pmic_getBitField_b(pmicHandle->siliconRev, DEVICE_PG_IDENTIFER);
        if (!isB1)
        {
            // Device is not B1. Decipher whether device is A0 or B0
            status = decipherWhetherA0(pmicHandle);
        }
    }

    // Set the driver initialization status
    pmicHandle->drvInitStat = (status == PMIC_ST_SUCCESS) ? PMIC_DRV_INIT_SUCCESS : ~PMIC_DRV_INIT_SUCCESS;

    return status;
}

int32_t Pmic_deinit(Pmic_CoreHandle_t *pmicHandle)
{
    int32_t status = PMIC_ST_SUCCESS;

    if (pmicHandle == NULL)
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        pmicHandle->drvInitStat = 0U;
        pmicHandle->i2cAddr = 0U;
        pmicHandle->devRev = 0U;
        pmicHandle->nvmId = 0U;
        pmicHandle->nvmRev = 0U;
        pmicHandle->siliconRev = 0U;
        pmicHandle->isA0 = (bool)false;
        pmicHandle->crcEnable = PMIC_DISABLE;
        pmicHandle->commHandle = NULL;
        pmicHandle->ioRead = NULL;
        pmicHandle->ioWrite = NULL;
        pmicHandle->critSecStart = NULL;
        pmicHandle->critSecStop = NULL;
        pmicHandle->irqResponse = NULL;
    }

    return status;
}
