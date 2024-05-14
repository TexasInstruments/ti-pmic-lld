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

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#include "pmic.h"
#include "pmic_common.h"

#include "pmic_core.h"
#include "pmic_io.h"
#include "regmap/core.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
#define PMIC_SILICON_REV_ID_PG_1_0 (0x0U)
#define PMIC_SILICON_REV_ID_PG_2_0 (0x08U)

#define REG_UNLOCK_DATA1    (0x98U)
#define REG_UNLOCK_DATA2    (0xB8U)
#define CNT_UNLOCK_DATA1    (0x13U)
#define CNT_UNLOCK_DATA2    (0x7DU)

/* ========================================================================== */
/*                        Interface Implementations                           */
/* ========================================================================== */
int32_t Pmic_setRegLockState(Pmic_CoreHandle_t *handle, uint8_t lockState) {
    int32_t status = Pmic_checkPmicCoreHandle(handle);

    // Holds the sequence used for register lock/unlock. For locking, writing
    // any sequence other than the correct one will lock the registers, in
    // which case 0->0 is fine (which is why it is the default).
    uint8_t seq[2] = {0, 0};

    // If unlocking registers, set the correct unlock sequence.
    if ((status == PMIC_ST_SUCCESS) && (lockState == PMIC_LOCK_DISABLE)) {
        seq[0] = REG_UNLOCK_DATA1;
        seq[1] = REG_UNLOCK_DATA2;
    }

    // Obtain Critical Section
    Pmic_criticalSectionStart(handle);

    if (status == PMIC_ST_SUCCESS) {
        status = Pmic_commIntf_sendByte(handle, CFG_REG_UNLOCK_SEQ_REG, seq[0]);
    }

    if (status == PMIC_ST_SUCCESS) {
        status = Pmic_commIntf_sendByte(handle, CFG_REG_UNLOCK_SEQ_REG, seq[1]);
    }

    // Release Critical Section
    Pmic_criticalSectionStop(handle);

    return status;
}

int32_t Pmic_setCntLockState(Pmic_CoreHandle_t *handle, uint8_t lockState) {
    int32_t status = Pmic_checkPmicCoreHandle(handle);

    // Holds the sequence used for register lock/unlock. For locking, writing
    // any sequence other than the correct one will lock the registers, in
    // which case 0->0 is fine (which is why it is the default).
    uint8_t seq[2] = {0, 0};

    // If unlocking registers, set the correct unlock sequence.
    if ((status == PMIC_ST_SUCCESS) && (lockState == PMIC_LOCK_DISABLE)) {
        seq[0] = CNT_UNLOCK_DATA1;
        seq[1] = CNT_UNLOCK_DATA2;
    }

    // Obtain Critical Section
    Pmic_criticalSectionStart(handle);

    if (status == PMIC_ST_SUCCESS) {
        status = Pmic_commIntf_sendByte(handle, CNT_REG_UNLOCK_SEQ_REG, seq[0]);
    }

    if (status == PMIC_ST_SUCCESS) {
        status = Pmic_commIntf_sendByte(handle, CNT_REG_UNLOCK_SEQ_REG, seq[1]);
    }

    // Release Critical Section
    Pmic_criticalSectionStop(handle);

    return status;
}

int32_t Pmic_setLockCfg(Pmic_CoreHandle_t *handle, const Pmic_Lock_t *config) {
    // Skip core handle check, this function uses other user facing APIs to do
    // all handle related work, it does not need to check the handle itself.
    int32_t status = PMIC_ST_SUCCESS;

    if (Pmic_validParamStatusCheck(config->validParams, PMIC_CFG_REG_LOCK_VALID, status)) {
        const uint8_t lockState = config->cfgLock ? PMIC_LOCK_ENABLE : PMIC_LOCK_DISABLE;
        Pmic_setRegLockState(handle, lockState);
    }

    if (Pmic_validParamStatusCheck(config->validParams, PMIC_CFG_CNT_LOCK_VALID, status)) {
        const uint8_t lockState = config->cntLock ? PMIC_LOCK_ENABLE : PMIC_LOCK_DISABLE;
        Pmic_setCntLockState(handle, lockState);
    }

    return status;
}

int32_t Pmic_getLockCfg(Pmic_CoreHandle_t *handle, Pmic_Lock_t *config) {
    int32_t status = Pmic_checkPmicCoreHandle(handle);
    uint8_t regData = 0U;

    // Read from REG_STAT_REG with critical section
    if (status == PMIC_ST_SUCCESS) {
        Pmic_criticalSectionStart(handle);
        status = Pmic_commIntf_recvByte(handle, REG_STAT_REG, &regData);
        Pmic_criticalSectionStop(handle);
    }

    // Extract requested bitfields from register data
    if (Pmic_validParamStatusCheck(config->validParams, PMIC_CFG_REG_LOCK_VALID, status)) {
        config->cfgLock = Pmic_getBitField_b(regData, CFG_REG_LOCKED_SHIFT, CFG_REG_LOCKED_MASK);
    }

    if (Pmic_validParamStatusCheck(config->validParams, PMIC_CFG_CNT_LOCK_VALID, status)) {
        config->cntLock = Pmic_getBitField_b(regData, CNT_REG_LOCKED_SHIFT, CNT_REG_LOCKED_MASK);
    }

    return status;
}

int32_t Pmic_getRegLockState(Pmic_CoreHandle_t *handle, uint8_t *lockState) {
    // Skip core handle check, this function uses other user facing APIs to do
    // all handle related work, it does not need to check the handle itself.
    int32_t status = PMIC_ST_SUCCESS;

    Pmic_Lock_t lockStatus = { .validParams = PMIC_CFG_REG_LOCK_VALID_SHIFT };
    status = Pmic_getLockCfg(handle, &lockStatus);

    if (status == PMIC_ST_SUCCESS) {
        *lockState = lockStatus.cfgLock ? PMIC_LOCK_ENABLE : PMIC_LOCK_DISABLE;
    }

    return status;
}

int32_t Pmic_getCntLockState(Pmic_CoreHandle_t *handle, uint8_t *lockState) {
    // Skip core handle check, this function uses other user facing APIs to do
    // all handle related work, it does not need to check the handle itself.
    int32_t status = PMIC_ST_SUCCESS;

    Pmic_Lock_t lockStatus = { .validParams = PMIC_CFG_CNT_LOCK_VALID_SHIFT };
    status = Pmic_getLockCfg(handle, &lockStatus);

    if (status == PMIC_ST_SUCCESS) {
        *lockState = lockStatus.cntLock ? PMIC_LOCK_ENABLE : PMIC_LOCK_DISABLE;
    }

    return status;
}