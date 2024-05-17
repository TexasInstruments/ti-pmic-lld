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

static inline void setPmicHandleMembers(const Pmic_CoreCfg_t *pmicCfg, Pmic_CoreHandle_t *pmicHandle)
{
    pmicHandle->i2cAddr = pmicCfg->i2cAddr;
    pmicHandle->commHandle = pmicCfg->commHandle;
    pmicHandle->ioRead = pmicCfg->ioRead;
    pmicHandle->ioWrite = pmicCfg->ioWrite;
    pmicHandle->critSecStart = pmicCfg->critSecStart;
    pmicHandle->critSecStop = pmicCfg->critSecStop;
}

static int32_t getPmicInfo(Pmic_CoreHandle_t *pmicHandle)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    // Read INTERFACE_CONF register
    Pmic_criticalSectionStart(pmicHandle);
    status = Pmic_ioRx(pmicHandle, PMIC_INTERFACE_CONF_REGADDR, &regData);
    Pmic_criticalSectionStop(pmicHandle);

    if (status == PMIC_ST_SUCCESS)
    {
        // Extract I2C_CRC_EN bit field
        pmicHandle->crcEnable = Pmic_getBitField_b(regData, PMIC_I2C_CRC_EN_SHIFT);

        // Read DEV_REV register
        Pmic_criticalSectionStart(pmicHandle);
        status = Pmic_ioRx(pmicHandle, PMIC_DEV_REV_REGADDR, &regData);
        Pmic_criticalSectionStop(pmicHandle);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Extract TI_DEVICE_ID bit field
        pmicHandle->devRev = regData;

        // Read NVM_CODE_1 register
        Pmic_criticalSectionStart(pmicHandle);
        status = Pmic_ioRx(pmicHandle, PMIC_NVM_CODE_1_REGADDR, &regData);
        Pmic_criticalSectionStop(pmicHandle);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Extract TI_NVM_ID bit field
        pmicHandle->nvmId = regData;

        // Read NVM_CODE_2 register
        Pmic_criticalSectionStart(pmicHandle);
        status = Pmic_ioRx(pmicHandle, PMIC_NVM_CODE_2_REGADDR, &regData);
        Pmic_criticalSectionStop(pmicHandle);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Extract TI_NVM_REV bit field
        pmicHandle->nvmRev = regData;

        // Read MANUFACTURING_VER register
        Pmic_criticalSectionStart(pmicHandle);
        status = Pmic_ioRx(pmicHandle, PMIC_MANUFACTURING_VER_REGADDR, &regData);
        Pmic_criticalSectionStop(pmicHandle);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Extract SILICON_REV bit field
        pmicHandle->siliconRev = regData;
    }

    return status;
}

int32_t Pmic_init(const Pmic_CoreCfg_t *pmicCfg, Pmic_CoreHandle_t *pmicHandle)
{
    int32_t status = PMIC_ST_SUCCESS;

    // Parameter check
    if ((pmicCfg == NULL) || (pmicCfg->commHandle == NULL) || (pmicCfg->ioRead == NULL) ||
        (pmicCfg->ioWrite == NULL) || (pmicCfg->critSecStart == NULL) ||
        (pmicCfg->critSecStop == NULL) || (pmicHandle == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    // Initialize PMIC handle
    if (status == PMIC_ST_SUCCESS)
    {
        setPmicHandleMembers(pmicCfg, pmicHandle);
    }

    // Get PMIC info, store info in pmic handle
    if (status == PMIC_ST_SUCCESS)
    {
        status = getPmicInfo(pmicHandle);
    }

    // Set the driver initialization status
    if (status == PMIC_ST_SUCCESS)
    {
        pmicHandle->drvInitStat = PMIC_DRV_INIT_SUCCESS;
    }

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
        pmicHandle->crcEnable = false;
        pmicHandle->commHandle = NULL;
        pmicHandle->ioRead = NULL;
        pmicHandle->ioWrite = NULL;
        pmicHandle->critSecStart = NULL;
        pmicHandle->critSecStop = NULL;
    }

    return status;
}
