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
 * @file pmic_core.c
 *
 * @brief This file contains definitions to APIs that interact with the PMIC Core.
 */
#include "pmic.h"
#include "pmic_io.h"
#include "pmic_core.h"

#include "regmap/core.h"

int32_t Pmic_checkPmicCoreHandle(const Pmic_CoreHandle_t *pmicHandle)
{
    int32_t status = PMIC_ST_SUCCESS;
    const bool invalidHandleConditions = (
        (pmicHandle->ioTransfer == NULL) ||
        (pmicHandle->drvInitStat != PMIC_DRV_INIT));

    if (pmicHandle == NULL)
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) && invalidHandleConditions)
    {
        status = PMIC_ST_ERR_INV_HANDLE;
    }

    return status;
}

int32_t Pmic_setRegLock(const Pmic_CoreHandle_t *pmicHandle, bool lock)
{
    int32_t status = Pmic_checkPmicCoreHandle(pmicHandle);

    if (status == PMIC_ST_SUCCESS)
    {
        if (lock)
        {
            status = Pmic_ioTxByte_CS(pmicHandle, PMIC_CMD_SW_LOCK, PMIC_CMD_SW_LOCK_DATA);
        }
        else
        {
            status = Pmic_ioTxByte_CS(pmicHandle, PMIC_CMD_SW_UNLOCK, PMIC_CMD_SW_UNLOCK_DATA);
        }
    }

    return status;
}

int32_t Pmic_unlockRegs(const Pmic_CoreHandle_t *pmicHandle)
{
    return Pmic_setRegLock(pmicHandle, PMIC_UNLOCK);
}

int32_t Pmic_lockRegs(const Pmic_CoreHandle_t *pmicHandle)
{
    return Pmic_setRegLock(pmicHandle, PMIC_LOCK);
}

int32_t Pmic_getRegLock(const Pmic_CoreHandle_t *pmicHandle, bool *locked)
{
    uint8_t regData = 0U;
    int32_t status = Pmic_checkPmicCoreHandle(pmicHandle);

    if ((status == PMIC_ST_SUCCESS) && (locked == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    // Read SAFETY_ERR_CFG_1 register
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioRxByte_CS(pmicHandle, PMIC_CMD_RD_SAFETY_ERR_CFG_1, &regData);
    }

    // Extract CFG_LOCK bit field
    if (status == PMIC_ST_SUCCESS)
    {
        *locked = Pmic_getBitField_b(regData, CFG_LOCK_SHIFT);
    }

    return status;
}
