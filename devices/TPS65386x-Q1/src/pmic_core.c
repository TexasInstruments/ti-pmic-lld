/******************************************************************************
 * Copyright (c) 2026 Texas Instruments Incorporated - http://www.ti.com
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
#include <string.h>

#include "pmic.h"
#include "pmic_common.h"

#include "pmic_core.h"
#include "pmic_io.h"
#include "regmap/core.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
#define REG_UNLOCK_DATA1    (0x98U)
#define REG_UNLOCK_DATA2    (0xB8U)
#define CNT_UNLOCK_DATA1    (0x13U)
#define CNT_UNLOCK_DATA2    (0x7DU)

/* Multiplexer validation constants */
#define PMIC_MUX_AMUX_CHANNEL_MAX_INTERNAL  ((uint8_t)0x1FU)
#define PMIC_MUX_DMUX_GROUP_MAX_INTERNAL    ((uint8_t)0x1FU)

/* ========================================================================== */
/*                        Interface Implementations                           */
/* ========================================================================== */
static inline void CORE_copyLock(const Pmic_Lock_t *src, Pmic_Lock_t *dst) {
    dst->validParams = src->validParams;
    dst->cfgLock = src->cfgLock;
    dst->cntLock = src->cntLock;
}

int32_t Pmic_setRegLockState(const Pmic_Handle_t *handle, bool lockState) {
    int32_t status = Pmic_checkHandle(handle);
    // Holds the sequence used for register lock/unlock. For locking, writing
    // any sequence other than the correct one will lock the registers, in
    // which case 0->0 is fine (which is why it is the default).
    uint8_t seq[2] = {0, 0};

    // If unlocking registers, set the correct unlock sequence.
    if ((status == PMIC_ST_SUCCESS) && (lockState == (bool)PMIC_LOCK_DISABLE)) {
        seq[0] = REG_UNLOCK_DATA1;
        seq[1] = REG_UNLOCK_DATA2;
    }

    // Obtain Critical Section
    Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);

    if (status == PMIC_ST_SUCCESS) {
        status = Pmic_ioTxByte(handle, CFG_REG_UNLOCK_SEQ_REG, seq[0]);
    }

    if (status == PMIC_ST_SUCCESS) {
        status = Pmic_ioTxByte(handle, CFG_REG_UNLOCK_SEQ_REG, seq[1]);
    }

    // Release Critical Section
    Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_setCntLockState(const Pmic_Handle_t *handle, uint8_t lockState) {
    int32_t status = Pmic_checkHandle(handle);
    // Holds the sequence used for register lock/unlock. For locking, writing
    // any sequence other than the correct one will lock the registers, in
    // which case 0->0 is fine (which is why it is the default).
    uint8_t seq[2] = {0, 0};

    if ((status == PMIC_ST_SUCCESS) && (lockState != PMIC_LOCK_ENABLE) && (lockState != PMIC_LOCK_DISABLE)) {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    // If unlocking registers, set the correct unlock sequence.
    if ((status == PMIC_ST_SUCCESS) && (lockState == PMIC_LOCK_DISABLE)) {
        seq[0] = CNT_UNLOCK_DATA1;
        seq[1] = CNT_UNLOCK_DATA2;
    }

    // Obtain Critical Section
    Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);

    if (status == PMIC_ST_SUCCESS) {
        status = Pmic_ioTxByte(handle, CNT_REG_UNLOCK_SEQ_REG, seq[0]);
    }

    if (status == PMIC_ST_SUCCESS) {
        status = Pmic_ioTxByte(handle, CNT_REG_UNLOCK_SEQ_REG, seq[1]);
    }

    // Release Critical Section
    Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_setLockCfg(const Pmic_Handle_t *handle, const Pmic_Lock_t *config) {
    // Skip core handle check, this function uses other user facing APIs to do
    // all handle related work, it does not need to check the handle itself.
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_Lock_t localConfig;

    if (config == NULL) {
        return Pmic_logStatus(handle, PMIC_ST_ERR_NULL_PARAM);
    }

    // Validate validParams: must have at least one valid bit and no invalid bits
    if ((config->validParams == 0U) || ((config->validParams & ~PMIC_CFG_LOCK_ALL_VALID_SHIFT) != 0U)) {
        return Pmic_logStatus(handle, PMIC_ST_ERR_INV_PARAM);
    }

    CORE_copyLock(config, &localConfig);

    if (Pmic_validParamStatusCheck(localConfig.validParams, PMIC_CFG_REG_LOCK_VALID, status)) {
        status = Pmic_setRegLockState(handle, localConfig.cfgLock);
    }

    if (Pmic_validParamStatusCheck(localConfig.validParams, PMIC_CFG_CNT_LOCK_VALID, status)) {
        status = Pmic_setCntLockState(handle, (localConfig.cntLock != false) ? PMIC_LOCK_ENABLE : PMIC_LOCK_DISABLE);
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_getLockCfg(const Pmic_Handle_t *handle, Pmic_Lock_t *config) {
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;
    Pmic_Lock_t localConfig = {0};

    if ((status == PMIC_ST_SUCCESS) && (config == NULL)) {
        return Pmic_logStatus(handle, PMIC_ST_ERR_NULL_PARAM);
    }

    if (status == PMIC_ST_SUCCESS) {
        CORE_copyLock(config, &localConfig);
    }

    // Read from REG_STAT_REG with critical section
    if (status == PMIC_ST_SUCCESS) {
        status = Pmic_ioRxByte_CS(handle, REG_STAT_REG, &regData);
    }

    // Extract requested bitfields from register data
    if (Pmic_validParamStatusCheck(localConfig.validParams, PMIC_CFG_REG_LOCK_VALID, status)) {
        localConfig.cfgLock = Pmic_getBitField_b(regData, CFG_REG_LOCKED_SHIFT);
    }

    if (Pmic_validParamStatusCheck(localConfig.validParams, PMIC_CFG_CNT_LOCK_VALID, status)) {
        localConfig.cntLock = Pmic_getBitField_b(regData, CNT_REG_LOCKED_SHIFT);
    }

    if (status == PMIC_ST_SUCCESS) {
        CORE_copyLock(&localConfig, config);
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_getRegLockState(const Pmic_Handle_t *handle, uint8_t *lockState) {
    // Skip core handle check, this function uses other user facing APIs to do
    // all handle related work, it does not need to check the handle itself.
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_Lock_t lockStatus = { .validParams = PMIC_CFG_REG_LOCK_VALID };

    if (lockState == NULL) {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if (status == PMIC_ST_SUCCESS) {
        status = Pmic_getLockCfg(handle, &lockStatus);
    }

    if (status == PMIC_ST_SUCCESS) {
        *lockState = (lockStatus.cfgLock != false) ? PMIC_LOCK_ENABLE : PMIC_LOCK_DISABLE;
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_getCntLockState(const Pmic_Handle_t *handle, uint8_t *lockState) {
    // Skip core handle check, this function uses other user facing APIs to do
    // all handle related work, it does not need to check the handle itself.
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_Lock_t lockStatus = { .validParams = PMIC_CFG_CNT_LOCK_VALID };

    if (lockState == NULL) {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if (status == PMIC_ST_SUCCESS) {
        status = Pmic_getLockCfg(handle, &lockStatus);
    }

    if (status == PMIC_ST_SUCCESS) {
        *lockState = (lockStatus.cntLock != false) ? PMIC_LOCK_ENABLE : PMIC_LOCK_DISABLE;
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_getSiliconRev(const Pmic_Handle_t *handle, uint8_t *siliconRev) {
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    if ((status == PMIC_ST_SUCCESS) && (siliconRev == NULL)) {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if (status == PMIC_ST_SUCCESS) {
        status = Pmic_ioRxByte_CS(handle, PMIC_DEV_REV_REG, &regData);
    }

    if (status == PMIC_ST_SUCCESS) {
        *siliconRev = Pmic_getBitField(regData, PMIC_DEV_REV_SHIFT, PMIC_DEV_REV_MASK);
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_getNvmRev(const Pmic_Handle_t *handle, uint8_t *nvmRev) {
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    if ((status == PMIC_ST_SUCCESS) && (nvmRev == NULL)) {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if (status == PMIC_ST_SUCCESS) {
        status = Pmic_ioRxByte_CS(handle, PMIC_NVM_REV_REG, &regData);
    }

    if (status == PMIC_ST_SUCCESS) {
        *nvmRev = Pmic_getBitField(regData, PMIC_NVM_REV_SHIFT, PMIC_NVM_REV_MASK);
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_setScratchPadValue(const Pmic_Handle_t *handle, uint8_t scratchPadRegNum, uint8_t value) {
    int32_t status = Pmic_checkHandle(handle);

    if ((status == PMIC_ST_SUCCESS) && (scratchPadRegNum > PMIC_SCRATCH_PAD_REG_MAX)) {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    if (status == PMIC_ST_SUCCESS) {
        status = Pmic_ioTxByte_CS(handle, PMIC_CUSTOMER_SCRATCH1_REG + scratchPadRegNum, value);
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_getScratchPadValue(const Pmic_Handle_t *handle, uint8_t scratchPadRegNum, uint8_t *value) {
    int32_t status = Pmic_checkHandle(handle);

    if ((status == PMIC_ST_SUCCESS) && (scratchPadRegNum > PMIC_SCRATCH_PAD_REG_MAX)) {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) && (value == NULL)) {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if (status == PMIC_ST_SUCCESS) {
        status = Pmic_ioRxByte_CS(handle, PMIC_CUSTOMER_SCRATCH1_REG + scratchPadRegNum, value);
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_setMuxCfg(const Pmic_Handle_t *handle, const Pmic_MuxCfg_t *config) {
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regDataCtrl = 0U;
    uint8_t regDataCfg = 0U;
    uint8_t ctrlValue = 0U;
    bool updateCtrl = false;
    bool updateCfg = false;

    if ((status == PMIC_ST_SUCCESS) && (config == NULL)) {
        return Pmic_logStatus(handle, PMIC_ST_ERR_NULL_PARAM);
    }

    // Validate AMUX channel range
    if (Pmic_validParamStatusCheck(config->validParams, PMIC_CFG_MUX_AMUX_CHANNEL_VALID, status)) {
        if (config->amuxChannel > PMIC_MUX_AMUX_CHANNEL_MAX_INTERNAL) {
            status = PMIC_ST_ERR_INV_PARAM;
        }
    }

    // Validate DMUX group range
    if (Pmic_validParamStatusCheck(config->validParams, PMIC_CFG_MUX_DMUX_GROUP_VALID, status)) {
        if (config->dmuxGroup > PMIC_MUX_DMUX_GROUP_MAX_INTERNAL) {
            status = PMIC_ST_ERR_INV_PARAM;
        }
    }

    if (status == PMIC_ST_SUCCESS) {
        Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);

        // Determine which registers need to be updated
        if (Pmic_validParamCheck(config->validParams,
                                 PMIC_CFG_MUX_MODE_VALID |
                                 PMIC_CFG_MUX_AMUX_EN_VALID |
                                 PMIC_CFG_MUX_DMUX_EN_VALID |
                                 PMIC_CFG_MUX_DMUX_GROUP_VALID)) {
            updateCtrl = true;
        }

        if (Pmic_validParamCheck(config->validParams, PMIC_CFG_MUX_AMUX_CHANNEL_VALID)) {
            updateCfg = true;
        }

        // Read current control register if we're updating it
        if ((status == PMIC_ST_SUCCESS) && updateCtrl) {
            status = Pmic_ioRxByte(handle, PMIC_DIAG_OUT_CFG_CTRL_REG, &regDataCtrl);
        }

        // Read current config register if we're updating it
        if ((status == PMIC_ST_SUCCESS) && updateCfg) {
            status = Pmic_ioRxByte(handle, PMIC_DIAG_OUT_CFG_REG, &regDataCfg);
        }

        // Process mode/enable settings for control register
        if (status == PMIC_ST_SUCCESS) {
            // Handle AMUX enable
            if (Pmic_validParamStatusCheck(config->validParams, PMIC_CFG_MUX_AMUX_EN_VALID, status)) {
                if (config->amuxEnable) {
                    ctrlValue = PMIC_DIAG_OUT_CTRL_AMUX_VALUE;
                }
            }

            // Handle DMUX enable (overwrites AMUX if both set)
            if (Pmic_validParamStatusCheck(config->validParams, PMIC_CFG_MUX_DMUX_EN_VALID, status)) {
                if (config->dmuxEnable) {
                    ctrlValue = PMIC_DIAG_OUT_CTRL_DMUX_VALUE;
                }
            }

            // Handle explicit mode value (highest priority)
            if (Pmic_validParamStatusCheck(config->validParams, PMIC_CFG_MUX_MODE_VALID, status)) {
                ctrlValue = config->muxMode;
            }

            // Update control register DIAG_OUT_CTRL field if needed
            if (updateCtrl && (status == PMIC_ST_SUCCESS)) {
                Pmic_setBitField(&regDataCtrl, PMIC_DIAG_OUT_CTRL_SHIFT,
                                PMIC_DIAG_OUT_CTRL_MASK, ctrlValue);
            }
        }

        // Update DMUX group selection in control register
        if (Pmic_validParamStatusCheck(config->validParams, PMIC_CFG_MUX_DMUX_GROUP_VALID, status)) {
            Pmic_setBitField(&regDataCtrl, PMIC_DIAG_GRP_SEL_SHIFT,
                            PMIC_DIAG_GRP_SEL_MASK, config->dmuxGroup);
        }

        // Update AMUX channel selection in config register
        if (Pmic_validParamStatusCheck(config->validParams, PMIC_CFG_MUX_AMUX_CHANNEL_VALID, status)) {
            Pmic_setBitField(&regDataCfg, PMIC_DIAG_CH_SEL_SHIFT,
                            PMIC_DIAG_CH_SEL_MASK, config->amuxChannel);
        }

        // Write back control register if updated
        if ((status == PMIC_ST_SUCCESS) && updateCtrl) {
            status = Pmic_ioTxByte(handle, PMIC_DIAG_OUT_CFG_CTRL_REG, regDataCtrl);
        }

        // Write back config register if updated
        if ((status == PMIC_ST_SUCCESS) && updateCfg) {
            status = Pmic_ioTxByte(handle, PMIC_DIAG_OUT_CFG_REG, regDataCfg);
        }

        Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_getMuxCfg(const Pmic_Handle_t *handle, Pmic_MuxCfg_t *config) {
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regDataCtrl = 0U;
    uint8_t regDataCfg = 0U;
    bool readCtrl = false;
    bool readCfg = false;

    if ((status == PMIC_ST_SUCCESS) && (config == NULL)) {
        return Pmic_logStatus(handle, PMIC_ST_ERR_NULL_PARAM);
    }

    if (status == PMIC_ST_SUCCESS) {
        // Determine which registers need to be read
        if (Pmic_validParamCheck(config->validParams,
                                 PMIC_CFG_MUX_MODE_VALID |
                                 PMIC_CFG_MUX_AMUX_EN_VALID |
                                 PMIC_CFG_MUX_DMUX_EN_VALID |
                                 PMIC_CFG_MUX_DMUX_GROUP_VALID)) {
            readCtrl = true;
        }

        if (Pmic_validParamCheck(config->validParams, PMIC_CFG_MUX_AMUX_CHANNEL_VALID)) {
            readCfg = true;
        }

        // Read control register if needed
        if (readCtrl) {
            status = Pmic_ioRxByte_CS(handle, PMIC_DIAG_OUT_CFG_CTRL_REG, &regDataCtrl);
        }

        // Read config register if needed
        if ((status == PMIC_ST_SUCCESS) && readCfg) {
            status = Pmic_ioRxByte_CS(handle, PMIC_DIAG_OUT_CFG_REG, &regDataCfg);
        }
    }

    if (status == PMIC_ST_SUCCESS) {
        // Extract mode value
        if (Pmic_validParamStatusCheck(config->validParams, PMIC_CFG_MUX_MODE_VALID, status)) {
            config->muxMode = Pmic_getBitField(regDataCtrl, PMIC_DIAG_OUT_CTRL_SHIFT,
                                              PMIC_DIAG_OUT_CTRL_MASK);
        }

        // Extract enable flags from mode
        if (readCtrl) {
            uint8_t mode = Pmic_getBitField(regDataCtrl, PMIC_DIAG_OUT_CTRL_SHIFT,
                                           PMIC_DIAG_OUT_CTRL_MASK);

            if (Pmic_validParamStatusCheck(config->validParams, PMIC_CFG_MUX_AMUX_EN_VALID, status)) {
                config->amuxEnable = (mode == PMIC_DIAG_OUT_CTRL_AMUX_VALUE);
            }

            if (Pmic_validParamStatusCheck(config->validParams, PMIC_CFG_MUX_DMUX_EN_VALID, status)) {
                config->dmuxEnable = (mode == PMIC_DIAG_OUT_CTRL_DMUX_VALUE);
            }
        }

        // Extract DMUX group
        if (Pmic_validParamStatusCheck(config->validParams, PMIC_CFG_MUX_DMUX_GROUP_VALID, status)) {
            config->dmuxGroup = Pmic_getBitField(regDataCtrl, PMIC_DIAG_GRP_SEL_SHIFT,
                                                PMIC_DIAG_GRP_SEL_MASK);
        }

        // Extract AMUX channel
        if (Pmic_validParamStatusCheck(config->validParams, PMIC_CFG_MUX_AMUX_CHANNEL_VALID, status)) {
            config->amuxChannel = Pmic_getBitField(regDataCfg, PMIC_DIAG_CH_SEL_SHIFT,
                                                  PMIC_DIAG_CH_SEL_MASK);
        }
    }

    return Pmic_logStatus(handle, status);
}
