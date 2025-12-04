/******************************************************************************
 * Copyright (c) 2025 Texas Instruments Incorporated - http://www.ti.com
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
 * @file pmic_io.c
 *
 * @brief Core module API definitions.
 */

/* ========================================================================== */
/*                              Include Files                                 */
/* ========================================================================== */

#include "pmic.h"

#include "regmap/core.h"

/* ========================================================================== */
/*                            Macros & Typedefs                               */
/* ========================================================================== */

/* ========================================================================== */
/*                           Function Definitions                             */
/* ========================================================================== */

int32_t Pmic_getDeviceId(const Pmic_Handle_t *handle, uint8_t *devId) {
    int32_t status = Pmic_checkHandle(handle);
    if (status != PMIC_ST_SUCCESS) {
        return status;
    }

    *devId = handle->devId;
    return PMIC_ST_SUCCESS;
}

int32_t Pmic_getDeviceSiRev(const Pmic_Handle_t *handle, uint8_t *siRev) {
    int32_t status = Pmic_checkHandle(handle);
    if (status != PMIC_ST_SUCCESS) {
        return status;
    }

    *siRev = handle->siRev;
    return PMIC_ST_SUCCESS;
}

int32_t Pmic_getDeviceNvmId(const Pmic_Handle_t *handle, uint8_t *nvmId) {
    int32_t status = Pmic_checkHandle(handle);
    if (status != PMIC_ST_SUCCESS) {
        return status;
    }

    *nvmId = handle->nvmId;
    return PMIC_ST_SUCCESS;
}

int32_t Pmic_getDeviceNvmRev(const Pmic_Handle_t *handle, uint8_t *nvmRev) {
    int32_t status = Pmic_checkHandle(handle);
    if (status != PMIC_ST_SUCCESS) {
        return status;
    }

    *nvmRev = handle->nvmRev;
    return PMIC_ST_SUCCESS;
}

int32_t Pmic_setRegLockState(const Pmic_Handle_t *handle, bool lock) {
    const uint8_t unlockVal = 0x9BU;
    const uint8_t lockVal = 0xAAU;
    int32_t status = Pmic_checkHandle(handle);
    if (status != PMIC_ST_SUCCESS) {
        return status;
    }

    return Pmic_ioTxByte_CS(handle, REGISTER_LOCK_REG, lock ? lockVal : unlockVal);
}

int32_t Pmic_getRegLockState(const Pmic_Handle_t *handle, bool *isLocked) {
    uint8_t regData = 0U;
    int32_t status = Pmic_checkHandle(handle);
    if (status != PMIC_ST_SUCCESS) {
        return status;
    }

    if (isLocked == NULL) {
        return PMIC_ST_ERR_NULL_PARAM;
    }

    status = Pmic_ioRxByte_CS(handle, REGISTER_LOCK_REG, &regData);
    if (status != PMIC_ST_SUCCESS) {
        return status;
    }

    *isLocked = Pmic_getBitField_b(regData, REGISTER_LOCK_STATUS_SHIFT);
    return PMIC_ST_SUCCESS;
}

int32_t Pmic_setScratchPadValue(const Pmic_Handle_t *handle, uint8_t scratchPadRegNum, uint8_t value) {
    int32_t status = Pmic_checkHandle(handle);
    if (status != PMIC_ST_SUCCESS) {
        return status;
    }

    return Pmic_ioTxByte_CS(handle, SCRATCH_PAD_REG_1_REG + scratchPadRegNum, value);
}

int32_t Pmic_getScratchPadValue(const Pmic_Handle_t *handle, uint8_t scratchPadRegNum, uint8_t *value) {
    int32_t status = Pmic_checkHandle(handle);
    if (status != PMIC_ST_SUCCESS) {
        return status;
    }

    if (value == NULL) {
        return PMIC_ST_ERR_NULL_PARAM;
    }

    return Pmic_ioRxByte_CS(handle, SCRATCH_PAD_REG_1_REG + scratchPadRegNum, value);
}
