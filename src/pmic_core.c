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
#include <stdint.h>
#include <stddef.h>

#include "pmic.h"
#include "pmic_io.h"
#include "pmic_core.h"
#include "regmap/core.h"

#define PMIC_REG_UNLOCK ((uint8_t)0x9BU)
#define PMIC_REG_LOCK   ((uint8_t)0x00U)

int32_t Pmic_setScratchPadVal(Pmic_CoreHandle_t *handle, uint8_t scratchPadRegNum, uint8_t value)
{
    int32_t status = Pmic_checkPmicCoreHandle(handle);

    if ((status == PMIC_ST_SUCCESS) && (scratchPadRegNum > PMIC_SCRATCH_PAD_REG_MAX))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    // Set scratchpad value
    if (status == PMIC_ST_SUCCESS)
    {
        Pmic_criticalSectionStart(handle);
        status = Pmic_ioTxByte(handle, PMIC_SCRATCH_PAD_REG_1_REG + scratchPadRegNum, value);
        Pmic_criticalSectionStop(handle);
    }

    return status;
}

int32_t Pmic_getScratchPadVal(Pmic_CoreHandle_t *handle, uint8_t scratchPadRegNum, uint8_t *value)
{
    uint8_t regData = 0U;
    int32_t status = Pmic_checkPmicCoreHandle(handle);

    if ((status == PMIC_ST_SUCCESS) && (scratchPadRegNum > PMIC_SCRATCH_PAD_REG_MAX))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) && (value == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    // Get scratchpad value
    if (status == PMIC_ST_SUCCESS)
    {
        Pmic_criticalSectionStart(handle);
        status = Pmic_ioRxByte(handle, PMIC_SCRATCH_PAD_REG_1_REG + scratchPadRegNum, &regData);
        Pmic_criticalSectionStop(handle);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        *value = regData;
    }

    return status;
}

int32_t Pmic_setRegLockState(Pmic_CoreHandle_t *handle, bool lockState)
{
    int32_t status = Pmic_checkPmicCoreHandle(handle);
    const uint8_t key = (lockState == PMIC_LOCK_ENABLE) ? PMIC_REG_LOCK : PMIC_REG_UNLOCK;

    // Write the key to REGISTER_LOCK
    if (status == PMIC_ST_SUCCESS)
    {
        Pmic_criticalSectionStart(handle);
        status = Pmic_ioTxByte(handle, PMIC_REGISTER_LOCK_REG, key);
        Pmic_criticalSectionStop(handle);
    }

    return status;
}

int32_t Pmic_getRegLockState(Pmic_CoreHandle_t *handle, bool *lockState)
{
    int32_t status = Pmic_checkPmicCoreHandle(handle);
    uint8_t regData = 0U;

    if ((status == PMIC_ST_SUCCESS) && (lockState == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Read REGISTER_LOCK
        Pmic_criticalSectionStart(handle);
        status = Pmic_ioRxByte(handle, PMIC_REGISTER_LOCK_REG, &regData);
        Pmic_criticalSectionStop(handle);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Extract REGISTER_LOCK_STATUS bit field
        *lockState = Pmic_getBitField_b(regData, PMIC_REGISTER_LOCK_STATUS_SHIFT);
    }

    return status;
}
