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
#define REG_UNLOCK_DATA1    (0x98U)
#define REG_UNLOCK_DATA2    (0xB8U)
#define CNT_UNLOCK_DATA1    (0x13U)
#define CNT_UNLOCK_DATA2    (0x7DU)

/* ========================================================================== */
/*                        Interface Implementations                           */
/* ========================================================================== */
int32_t Pmic_setRegLockState(Pmic_Handle_t *handle, uint8_t lockState) {
    int32_t status = Pmic_checkHandle(handle);

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
        status = Pmic_ioTxByte(handle, CFG_REG_UNLOCK_SEQ_REG, seq[0]);
    }

    if (status == PMIC_ST_SUCCESS) {
        status = Pmic_ioTxByte(handle, CFG_REG_UNLOCK_SEQ_REG, seq[1]);
    }

    // Release Critical Section
    Pmic_criticalSectionStop(handle);

    return status;
}

int32_t Pmic_setCntLockState(Pmic_Handle_t *handle, uint8_t lockState) {
    int32_t status = Pmic_checkHandle(handle);

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
        status = Pmic_ioTxByte(handle, CNT_REG_UNLOCK_SEQ_REG, seq[0]);
    }

    if (status == PMIC_ST_SUCCESS) {
        status = Pmic_ioTxByte(handle, CNT_REG_UNLOCK_SEQ_REG, seq[1]);
    }

    // Release Critical Section
    Pmic_criticalSectionStop(handle);

    return status;
}

int32_t Pmic_setLockCfg(Pmic_Handle_t *handle, const Pmic_Lock_t *config) {
    // Skip core handle check, this function uses other user facing APIs to do
    // all handle related work, it does not need to check the handle itself.
    int32_t status = PMIC_ST_SUCCESS;

    if (Pmic_validParamStatusCheck(config->validParams, PMIC_CFG_REG_LOCK_VALID, status)) {
        const uint8_t lockState = config->cfgLock ? PMIC_LOCK_ENABLE : PMIC_LOCK_DISABLE;
        status = Pmic_setRegLockState(handle, lockState);
    }

    if (Pmic_validParamStatusCheck(config->validParams, PMIC_CFG_CNT_LOCK_VALID, status)) {
        const uint8_t lockState = config->cntLock ? PMIC_LOCK_ENABLE : PMIC_LOCK_DISABLE;
        status = Pmic_setCntLockState(handle, lockState);
    }

    return status;
}

int32_t Pmic_getLockCfg(Pmic_Handle_t *handle, Pmic_Lock_t *config) {
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    // Read from REG_STAT_REG with critical section
    if (status == PMIC_ST_SUCCESS) {
        Pmic_criticalSectionStart(handle);
        status = Pmic_ioRxByte(handle, REG_STAT_REG, &regData);
        Pmic_criticalSectionStop(handle);
    }

    // Extract requested bitfields from register data
    if (Pmic_validParamStatusCheck(config->validParams, PMIC_CFG_REG_LOCK_VALID, status)) {
        config->cfgLock = Pmic_getBitField_b(regData, CFG_REG_LOCKED_SHIFT);
    }

    if (Pmic_validParamStatusCheck(config->validParams, PMIC_CFG_CNT_LOCK_VALID, status)) {
        config->cntLock = Pmic_getBitField_b(regData, CNT_REG_LOCKED_SHIFT);
    }

    return status;
}

int32_t Pmic_getRegLockState(Pmic_Handle_t *handle, uint8_t *lockState) {
    // Skip core handle check, this function uses other user facing APIs to do
    // all handle related work, it does not need to check the handle itself.
    int32_t status = PMIC_ST_SUCCESS;

    Pmic_Lock_t lockStatus = { .validParams = PMIC_CFG_REG_LOCK_VALID };
    status = Pmic_getLockCfg(handle, &lockStatus);

    if (status == PMIC_ST_SUCCESS) {
        *lockState = lockStatus.cfgLock ? PMIC_LOCK_ENABLE : PMIC_LOCK_DISABLE;
    }

    return status;
}

int32_t Pmic_getCntLockState(Pmic_Handle_t *handle, uint8_t *lockState) {
    // Skip core handle check, this function uses other user facing APIs to do
    // all handle related work, it does not need to check the handle itself.
    int32_t status = PMIC_ST_SUCCESS;

    Pmic_Lock_t lockStatus = { .validParams = PMIC_CFG_CNT_LOCK_VALID };
    status = Pmic_getLockCfg(handle, &lockStatus);

    if (status == PMIC_ST_SUCCESS) {
        *lockState = lockStatus.cntLock ? PMIC_LOCK_ENABLE : PMIC_LOCK_DISABLE;
    }

    return status;
}

int32_t Pmic_getSiliconRev(Pmic_Handle_t *handle, uint8_t *siliconRev) {
    uint8_t regData = 0U;
    int32_t status = Pmic_checkHandle(handle);

    if ((status == PMIC_ST_SUCCESS) && (siliconRev == NULL)) {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if (status == PMIC_ST_SUCCESS) {
        status = Pmic_ioRxByte_CS(handle, PMIC_DEV_REV_REG, &regData);
    }

    if (status == PMIC_ST_SUCCESS) {
        *siliconRev = Pmic_getBitField(regData, PMIC_DEV_REV_SHIFT, PMIC_DEV_REV_MASK);
    }

    return status;
}

int32_t Pmic_getNvmRev(Pmic_Handle_t *handle, uint8_t *nvmRev) {
    uint8_t regData = 0U;
    int32_t status = Pmic_checkHandle(handle);

    if ((status == PMIC_ST_SUCCESS) && (nvmRev == NULL)) {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if (status == PMIC_ST_SUCCESS) {
        status = Pmic_ioRxByte_CS(handle, PMIC_NVM_REV_REG, &regData);
    }

    if (status == PMIC_ST_SUCCESS) {
        *nvmRev = Pmic_getBitField(regData, PMIC_NVM_REV_SHIFT, PMIC_NVM_REV_MASK);
    }

    return status;
}

int32_t Pmic_setScratchPadValue(Pmic_Handle_t *handle, uint8_t scratchPadRegNum, uint8_t value) {
    int32_t status = Pmic_checkHandle(handle);

    if ((status == PMIC_ST_SUCCESS) && (scratchPadRegNum > PMIC_SCRATCH_PAD_REG_MAX)) {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    if (status == PMIC_ST_SUCCESS) {
        status = Pmic_ioTxByte_CS(handle, PMIC_CUSTOMER_SCRATCH1_REG + scratchPadRegNum, value);
    }

    return status;
}

int32_t Pmic_getScratchPadValue(Pmic_Handle_t *handle, uint8_t scratchPadRegNum, uint8_t *value) {
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

    return status;
}

int32_t Pmic_spreadSpectrumEnable(Pmic_Handle_t *handle, Pmic_CommonCtrlCfg_t config) {
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    if (status == PMIC_ST_SUCCESS) {
        Pmic_criticalSectionStart(handle);
        status = Pmic_ioRxByte(handle, PMIC_BUCK_BST_CFG_REG, &regData);

        if (Pmic_validParamStatusCheck(config.validParams, PMIC_COMMON_CTRL_SPREAD_SPECTRUM_EN_VALID, status)) {
            Pmic_setBitField_b(&regData, PMIC_DRSS_SS_EN_SHIFT, config.spreadSpectrumEn);
        }

        if (status == PMIC_ST_SUCCESS) {
            status = Pmic_ioTxByte(handle, PMIC_BUCK_BST_CFG_REG, regData);
        }
        Pmic_criticalSectionStop(handle);
    }

    return status;
}

int32_t Pmic_getSpreadSpectrumEnable(Pmic_Handle_t *handle, Pmic_CommonCtrlCfg_t *config) {
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    if ((status == PMIC_ST_SUCCESS) && (config == NULL)) {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if (status == PMIC_ST_SUCCESS) {
        status = Pmic_ioRxByte_CS(handle, PMIC_BUCK_BST_CFG_REG, &regData);
    }

    if (status == PMIC_ST_SUCCESS) {
        config->spreadSpectrumEn = Pmic_getBitField_b(regData, PMIC_DRSS_SS_EN_SHIFT);
    }

    return status;
}

int32_t Pmic_setEnableSafeOutCfg(Pmic_Handle_t *handle, Pmic_CommonCtrlCfg_t config) {
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    if (status == PMIC_ST_SUCCESS) {
        Pmic_criticalSectionStart(handle);
        status = Pmic_ioRxByte(handle, PMIC_SAFE_OUT_CFG_CTRL_REG, &regData);

        if (Pmic_validParamStatusCheck(config.validParams, PMIC_COMMON_CTRL_ENSAFEOUT1_VALID, status)) {
            Pmic_setBitField_b(&regData, PMIC_SAFE_OUT1_EN_SHIFT, config.enSafeOut1);
        }

        if (Pmic_validParamStatusCheck(config.validParams, PMIC_COMMON_CTRL_ENSAFEOUT2_VALID, status)) {
            Pmic_setBitField_b(&regData, PMIC_SAFE_OUT2_EN_SHIFT, config.enSafeOut2);
        }

        if (status == PMIC_ST_SUCCESS) {
            status = Pmic_ioTxByte(handle, PMIC_SAFE_OUT_CFG_CTRL_REG, regData);
        }
        Pmic_criticalSectionStop(handle);
    }

    return status;
}

int32_t Pmic_getSafeOutPinCfg(Pmic_Handle_t *handle, Pmic_CommonCtrlCfg_t *config) {
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    if ((status == PMIC_ST_SUCCESS) && (config == NULL)) {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if (status == PMIC_ST_SUCCESS) {
        status = Pmic_ioRxByte_CS(handle, PMIC_SAFE_OUT_CFG_CTRL_REG, &regData);
    }

    if (status == PMIC_ST_SUCCESS) {
        config->enSafeOut1 = Pmic_getBitField_b(regData, PMIC_SAFE_OUT1_EN_SHIFT);
        config->enSafeOut2 = Pmic_getBitField_b(regData, PMIC_SAFE_OUT2_EN_SHIFT);
    }

    return status;
}

int32_t Pmic_getCommonStat(Pmic_Handle_t *handle, Pmic_CommonCtrlStat_t *stat) {
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData1 = 0U;
    uint8_t regData2 = 0U;

    if ((status == PMIC_ST_SUCCESS) && (stat == NULL)) {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    // Read pin status from STAT_READBACK_ERR register
    if (status == PMIC_ST_SUCCESS) {
        status = Pmic_ioRxByte_CS(handle, PMIC_RDBK_ERR_STAT_REG, &regData1);
    }

    // Read lock status from REG_STAT register
    if (status == PMIC_ST_SUCCESS) {
        status = Pmic_ioRxByte_CS(handle, REG_STAT_REG, &regData2);
    }

    if (status == PMIC_ST_SUCCESS) {
        stat->nRstPin = Pmic_getBitField_b(regData1, PMIC_NRST_RDBK_LVL_SHIFT);
        stat->safeOut1Pin = Pmic_getBitField_b(regData1, PMIC_SAFE_OUT1_RDBK_LVL_SHIFT);
        stat->enOutPin = Pmic_getBitField_b(regData1, PMIC_EN_OUT_RDBK_LVL_SHIFT);
        stat->cfgregLockStat = Pmic_getBitField_b(regData2, CFG_REG_LOCKED_SHIFT);
    }

    return status;
}

int32_t Pmic_setDiagOutCtrlConfig(Pmic_Handle_t *handle, Pmic_DiagOutCfgCtrl_t config) {
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;
    uint8_t ctrlValue = 0U;

    if (status == PMIC_ST_SUCCESS) {
        Pmic_criticalSectionStart(handle);
        status = Pmic_ioRxByte(handle, PMIC_DIAG_OUT_CFG_CTRL_REG, &regData);

        if (Pmic_validParamStatusCheck(config.validParams, PMIC_DIAG_OUT_CTRL_AMUX_EN_VALID, status)) {
            if (config.diagOutCtrl_AMUXEn) {
                ctrlValue = PMIC_DIAG_OUT_CTRL_AMUX_VALUE;
            }
        }

        if (Pmic_validParamStatusCheck(config.validParams, PMIC_DIAG_OUT_CTRL_DMUX_EN_VALID, status)) {
            if (config.diagOutCtrl_DMUXEn) {
                ctrlValue = PMIC_DIAG_OUT_CTRL_DMUX_VALUE;
            }
        }

        if (Pmic_validParamStatusCheck(config.validParams, PMIC_DIAG_OUT_CTRL_VALID, status)) {
            ctrlValue = config.diagOutCtrl;
        }

        if ((status == PMIC_ST_SUCCESS) &&
            ((config.validParams & (PMIC_DIAG_OUT_CTRL_AMUX_EN_VALID |
                                    PMIC_DIAG_OUT_CTRL_DMUX_EN_VALID |
                                    PMIC_DIAG_OUT_CTRL_VALID)) != 0U)) {
            Pmic_setBitField(&regData, PMIC_DIAG_OUT_CTRL_SHIFT, PMIC_DIAG_OUT_CTRL_MASK, ctrlValue);
            status = Pmic_ioTxByte(handle, PMIC_DIAG_OUT_CFG_CTRL_REG, regData);
        }
        Pmic_criticalSectionStop(handle);
    }

    return status;
}

int32_t Pmic_getDiagOutCtrlConfig(Pmic_Handle_t *handle, Pmic_DiagOutCfgCtrl_t *config) {
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    if ((status == PMIC_ST_SUCCESS) && (config == NULL)) {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if (status == PMIC_ST_SUCCESS) {
        status = Pmic_ioRxByte_CS(handle, PMIC_DIAG_OUT_CFG_CTRL_REG, &regData);
    }

    if (status == PMIC_ST_SUCCESS) {
        config->diagOutCtrl = Pmic_getBitField(regData, PMIC_DIAG_OUT_CTRL_SHIFT, PMIC_DIAG_OUT_CTRL_MASK);
        config->diagOutCtrl_AMUXEn = (config->diagOutCtrl == PMIC_DIAG_OUT_CTRL_AMUX_VALUE) ? 1U : 0U;
        config->diagOutCtrl_DMUXEn = (config->diagOutCtrl == PMIC_DIAG_OUT_CTRL_DMUX_VALUE) ? 1U : 0U;
    }

    return status;
}

int32_t Pmic_setDiagAMUXFeatureCfg(Pmic_Handle_t *handle, uint8_t channel) {
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    if ((status == PMIC_ST_SUCCESS) && (channel > 0x1FU)) {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    if (status == PMIC_ST_SUCCESS) {
        Pmic_criticalSectionStart(handle);
        status = Pmic_ioRxByte(handle, PMIC_DIAG_OUT_CFG_REG, &regData);

        if (status == PMIC_ST_SUCCESS) {
            Pmic_setBitField(&regData, PMIC_DIAG_CH_SEL_SHIFT, PMIC_DIAG_CH_SEL_MASK, channel);
            status = Pmic_ioTxByte(handle, PMIC_DIAG_OUT_CFG_REG, regData);
        }
        Pmic_criticalSectionStop(handle);
    }

    return status;
}

int32_t Pmic_getDiagAMUXFeatureCfg(Pmic_Handle_t *handle, uint8_t *channel) {
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    if ((status == PMIC_ST_SUCCESS) && (channel == NULL)) {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if (status == PMIC_ST_SUCCESS) {
        status = Pmic_ioRxByte_CS(handle, PMIC_DIAG_OUT_CFG_REG, &regData);
    }

    if (status == PMIC_ST_SUCCESS) {
        *channel = Pmic_getBitField(regData, PMIC_DIAG_CH_SEL_SHIFT, PMIC_DIAG_CH_SEL_MASK);
    }

    return status;
}

int32_t Pmic_setDiagDMUXFeatureCfg(Pmic_Handle_t *handle, uint8_t group) {
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    if ((status == PMIC_ST_SUCCESS) && (group > 0x1FU)) {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    if (status == PMIC_ST_SUCCESS) {
        Pmic_criticalSectionStart(handle);
        status = Pmic_ioRxByte(handle, PMIC_DIAG_OUT_CFG_CTRL_REG, &regData);

        if (status == PMIC_ST_SUCCESS) {
            Pmic_setBitField(&regData, PMIC_DIAG_GRP_SEL_SHIFT, PMIC_DIAG_GRP_SEL_MASK, group);
            status = Pmic_ioTxByte(handle, PMIC_DIAG_OUT_CFG_CTRL_REG, regData);
        }
        Pmic_criticalSectionStop(handle);
    }

    return status;
}

int32_t Pmic_getDiagDMUXFeatureCfg(Pmic_Handle_t *handle, uint8_t *group) {
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    if ((status == PMIC_ST_SUCCESS) && (group == NULL)) {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if (status == PMIC_ST_SUCCESS) {
        status = Pmic_ioRxByte_CS(handle, PMIC_DIAG_OUT_CFG_CTRL_REG, &regData);
    }

    if (status == PMIC_ST_SUCCESS) {
        *group = Pmic_getBitField(regData, PMIC_DIAG_GRP_SEL_SHIFT, PMIC_DIAG_GRP_SEL_MASK);
    }

    return status;
}
