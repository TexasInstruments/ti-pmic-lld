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

// CRC-16 calculation over configuration register range (polynomial 0xBAAD,
// normal form 0x755B: 1+x^1+x^3+x^4+x^6+x^8+x^10+x^12+x^13+x^14+x^16)
#define CONFIG_CRC_INIT   (0xFFFFU)
#define CONFIG_CRC_REG_LO (0x00U)
#define CONFIG_CRC_REG_HI (0xFFU)

/* Multiplexer validation constants */
#define PMIC_MUX_AMUX_CHANNEL_MAX_INTERNAL  ((uint8_t)0x1FU)
#define PMIC_MUX_DMUX_GROUP_MAX_INTERNAL    ((uint8_t)0x1FU)

/* ========================================================================== */
/*                        Interface Implementations                           */
/* ========================================================================== */
/** @brief CRC-16 table lookup for polynomial 0xBAAD (normal form 0x755B) */
static uint16_t CORE_Crc16Calc(uint16_t crc, uint16_t data)
{
    static const uint16_t CRC16LUT[256] = {
        0x0000U, 0x755BU, 0xEAB6U, 0x9FEDU, 0xA037U, 0xD56CU, 0x4A81U, 0x3FDAU,
        0x3535U, 0x406EU, 0xDF83U, 0xAAD8U, 0x9502U, 0xE059U, 0x7FB4U, 0x0AEFU,
        0x6A6AU, 0x1F31U, 0x80DCU, 0xF587U, 0xCA5DU, 0xBF06U, 0x20EBU, 0x55B0U,
        0x5F5FU, 0x2A04U, 0xB5E9U, 0xC0B2U, 0xFF68U, 0x8A33U, 0x15DEU, 0x6085U,
        0xD4D4U, 0xA18FU, 0x3E62U, 0x4B39U, 0x74E3U, 0x01B8U, 0x9E55U, 0xEB0EU,
        0xE1E1U, 0x94BAU, 0x0B57U, 0x7E0CU, 0x41D6U, 0x348DU, 0xAB60U, 0xDE3BU,
        0xBEBEU, 0xCBE5U, 0x5408U, 0x2153U, 0x1E89U, 0x6BD2U, 0xF43FU, 0x8164U,
        0x8B8BU, 0xFED0U, 0x613DU, 0x1466U, 0x2BBCU, 0x5EE7U, 0xC10AU, 0xB451U,
        0xDCF3U, 0xA9A8U, 0x3645U, 0x431EU, 0x7CC4U, 0x099FU, 0x9672U, 0xE329U,
        0xE9C6U, 0x9C9DU, 0x0370U, 0x762BU, 0x49F1U, 0x3CAAU, 0xA347U, 0xD61CU,
        0xB699U, 0xC3C2U, 0x5C2FU, 0x2974U, 0x16AEU, 0x63F5U, 0xFC18U, 0x8943U,
        0x83ACU, 0xF6F7U, 0x691AU, 0x1C41U, 0x239BU, 0x56C0U, 0xC92DU, 0xBC76U,
        0x0827U, 0x7D7CU, 0xE291U, 0x97CAU, 0xA810U, 0xDD4BU, 0x42A6U, 0x37FDU,
        0x3D12U, 0x4849U, 0xD7A4U, 0xA2FFU, 0x9D25U, 0xE87EU, 0x7793U, 0x02C8U,
        0x624DU, 0x1716U, 0x88FBU, 0xFDA0U, 0xC27AU, 0xB721U, 0x28CCU, 0x5D97U,
        0x5778U, 0x2223U, 0xBDCEU, 0xC895U, 0xF74FU, 0x8214U, 0x1DF9U, 0x68A2U,
        0xCCBDU, 0xB9E6U, 0x260BU, 0x5350U, 0x6C8AU, 0x19D1U, 0x863CU, 0xF367U,
        0xF988U, 0x8CD3U, 0x133EU, 0x6665U, 0x59BFU, 0x2CE4U, 0xB309U, 0xC652U,
        0xA6D7U, 0xD38CU, 0x4C61U, 0x393AU, 0x06E0U, 0x73BBU, 0xEC56U, 0x990DU,
        0x93E2U, 0xE6B9U, 0x7954U, 0x0C0FU, 0x33D5U, 0x468EU, 0xD963U, 0xAC38U,
        0x1869U, 0x6D32U, 0xF2DFU, 0x8784U, 0xB85EU, 0xCD05U, 0x52E8U, 0x27B3U,
        0x2D5CU, 0x5807U, 0xC7EAU, 0xB2B1U, 0x8D6BU, 0xF830U, 0x67DDU, 0x1286U,
        0x7203U, 0x0758U, 0x98B5U, 0xEDEEU, 0xD234U, 0xA76FU, 0x3882U, 0x4DD9U,
        0x4736U, 0x326DU, 0xAD80U, 0xD8DBU, 0xE701U, 0x925AU, 0x0DB7U, 0x78ECU,
        0x104EU, 0x6515U, 0xFAF8U, 0x8FA3U, 0xB079U, 0xC522U, 0x5ACFU, 0x2F94U,
        0x257BU, 0x5020U, 0xCFCDU, 0xBA96U, 0x854CU, 0xF017U, 0x6FFAU, 0x1AA1U,
        0x7A24U, 0x0F7FU, 0x9092U, 0xE5C9U, 0xDA13U, 0xAF48U, 0x30A5U, 0x45FEU,
        0x4F11U, 0x3A4AU, 0xA5A7U, 0xD0FCU, 0xEF26U, 0x9A7DU, 0x0590U, 0x70CBU,
        0xC49AU, 0xB1C1U, 0x2E2CU, 0x5B77U, 0x64ADU, 0x11F6U, 0x8E1BU, 0xFB40U,
        0xF1AFU, 0x84F4U, 0x1B19U, 0x6E42U, 0x5198U, 0x24C3U, 0xBB2EU, 0xCE75U,
        0xAEF0U, 0xDBABU, 0x4446U, 0x311DU, 0x0EC7U, 0x7B9CU, 0xE471U, 0x912AU,
        0x9BC5U, 0xEE9EU, 0x7173U, 0x0428U, 0x3BF2U, 0x4EA9U, 0xD144U, 0xA41FU,
    };

    const uint8_t idx = (uint8_t)(data ^ (uint16_t)((crc & 0xFFFFU) >> 8U));
    return (uint16_t)((uint16_t)(crc << 8U) ^ CRC16LUT[idx]);
}

/** @brief Calculate config CRC over register range 0x00-0xFF */
static int32_t CORE_calculateCrc(const Pmic_Handle_t *handle, uint16_t *crc)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    *crc = CONFIG_CRC_INIT;

    for (uint8_t regAddr = CONFIG_CRC_REG_LO; regAddr <= CONFIG_CRC_REG_HI; regAddr++) {
        status = Pmic_ioRxByte(handle, regAddr, &regData);
        if (status != PMIC_ST_SUCCESS) {
            break;
        }
        *crc = CORE_Crc16Calc(*crc, (uint16_t)regData);
    }

    return status;
}

/** @brief Trigger hardware CRC re-check and confirm no mismatch error */
static int32_t CORE_configCrcValidate(const Pmic_Handle_t *handle)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);

    status = Pmic_ioRxByte(handle, SAFETY_CTRL_REG, &regData);

    // Validation should only be performed when config CRC is not yet enabled
    if ((status == PMIC_ST_SUCCESS) && Pmic_getBitField_b(regData, CFG_REG_CRC_EN_SHIFT)) {
        status = PMIC_ST_ERR_NOT_SUPPORTED;
    }

    // Ensure CFG_REG_CRC_CALC is low first to produce a clean rising edge
    if ((status == PMIC_ST_SUCCESS) && Pmic_getBitField_b(regData, CFG_REG_CRC_CALC_SHIFT)) {
        Pmic_setBitField_b(&regData, CFG_REG_CRC_CALC_SHIFT, (bool)false);
        Pmic_setBitField_b(&regData, CFG_REG_CRC_CALC_DONE_SHIFT, (bool)false); // W1C: writing 0 preserves
        status = Pmic_ioTxByte(handle, SAFETY_CTRL_REG, regData);
    }

    // Assert CFG_REG_CRC_CALC - rising edge triggers hardware calculation
    if (status == PMIC_ST_SUCCESS) {
        Pmic_setBitField_b(&regData, CFG_REG_CRC_CALC_SHIFT, (bool)true);
        Pmic_setBitField_b(&regData, CFG_REG_CRC_CALC_DONE_SHIFT, (bool)false); // W1C: writing 0 preserves
        status = Pmic_ioTxByte(handle, SAFETY_CTRL_REG, regData);
    }

    // SPI transaction time is sufficient for hardware to complete the
    // calculation; read back to capture the updated CALC_DONE status
    if (status == PMIC_ST_SUCCESS) {
        status = Pmic_ioRxByte(handle, SAFETY_CTRL_REG, &regData);
    }

    // De-assert CFG_REG_CRC_CALC; writing 0x00 is safe since CRC_EN is
    // confirmed 0 above and CFG_REG_CRC_CALC_DONE is W1C (0 preserves it)
    if (status == PMIC_ST_SUCCESS) {
        status = Pmic_ioTxByte(handle, SAFETY_CTRL_REG, 0x00U);
    }

    // Check REG_STAT for a CRC mismatch error
    if (status == PMIC_ST_SUCCESS) {
        status = Pmic_ioRxByte(handle, REG_STAT_REG, &regData);
    }

    if ((status == PMIC_ST_SUCCESS) && Pmic_getBitField_b(regData, CFG_REG_CRC_ERR_SHIFT)) {
        status = PMIC_ST_ERR_CONFIG_REG_CRC;
    }

    Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);
    return status;
}

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

int32_t Pmic_configCrcEnable(const Pmic_Handle_t *handle, bool calculate)
{
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    if ((status == PMIC_ST_SUCCESS) && (calculate == PMIC_CFG_CRC_RECALCULATE)) {
        status = Pmic_configCrcCalculate(handle);
    }

    Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);
    if (status == PMIC_ST_SUCCESS) {
        status = Pmic_ioRxByte(handle, SAFETY_CTRL_REG, &regData);
    }

    if (status == PMIC_ST_SUCCESS) {
        Pmic_setBitField_b(&regData, CFG_REG_CRC_EN_SHIFT, (bool)true);
        Pmic_setBitField_b(&regData, CFG_REG_CRC_CALC_DONE_SHIFT, (bool)false);
        status = Pmic_ioTxByte(handle, SAFETY_CTRL_REG, regData);
    }
    Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_configCrcDisable(const Pmic_Handle_t *handle)
{
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);
    if (status == PMIC_ST_SUCCESS) {
        status = Pmic_ioRxByte(handle, SAFETY_CTRL_REG, &regData);
    }

    if (status == PMIC_ST_SUCCESS) {
        Pmic_setBitField_b(&regData, CFG_REG_CRC_EN_SHIFT, (bool)false);
        Pmic_setBitField_b(&regData, CFG_REG_CRC_CALC_DONE_SHIFT, (bool)false);
        status = Pmic_ioTxByte(handle, SAFETY_CTRL_REG, regData);
    }
    Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_getConfigCrcEnableState(const Pmic_Handle_t *handle, bool *isEnabled)
{
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    if ((status == PMIC_ST_SUCCESS) && (isEnabled == NULL)) {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if (status == PMIC_ST_SUCCESS) {
        status = Pmic_ioRxByte_CS(handle, SAFETY_CTRL_REG, &regData);
    }

    if (status == PMIC_ST_SUCCESS) {
        *isEnabled = Pmic_getBitField_b(regData, CFG_REG_CRC_EN_SHIFT);
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_getConfigCrcStatus(const Pmic_Handle_t *handle, Pmic_ConfigCrcStat_t *configCrcStat)
{
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    if ((status == PMIC_ST_SUCCESS) && (configCrcStat == NULL)) {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) && (configCrcStat->validParams == 0U)) {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    if (Pmic_validParamStatusCheck(configCrcStat->validParams, PMIC_CONFIG_CRC_STAT_CALC_DONE_VALID, status)) {
        status = Pmic_ioRxByte_CS(handle, SAFETY_CTRL_REG, &regData);

        if (status == PMIC_ST_SUCCESS) {
            configCrcStat->calcDone = Pmic_getBitField_b(regData, CFG_REG_CRC_CALC_DONE_SHIFT);
        }
    }

    if (Pmic_validParamStatusCheck(configCrcStat->validParams, PMIC_CONFIG_CRC_STAT_ERROR_VALID, status)) {
        status = Pmic_ioRxByte_CS(handle, REG_STAT_REG, &regData);

        if (status == PMIC_ST_SUCCESS) {
            configCrcStat->error = Pmic_getBitField_b(regData, CFG_REG_CRC_ERR_SHIFT);
        }
    }

    return Pmic_logStatus(handle, status);
}
int32_t Pmic_clrConfigCrcStatus(const Pmic_Handle_t *handle, const Pmic_ConfigCrcStat_t *configCrcStat)
{
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    if ((status == PMIC_ST_SUCCESS) && (configCrcStat == NULL)) {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) && (configCrcStat->validParams == 0U)) {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    if (Pmic_validParamStatusCheck(configCrcStat->validParams, PMIC_CONFIG_CRC_STAT_CALC_DONE_VALID, status)) {
        Pmic_setBitField_b(&regData, CFG_REG_CRC_CALC_DONE_SHIFT, (bool)true);
        status = Pmic_ioTxByte_CS(handle, SAFETY_CTRL_REG, regData);
    }

    if (Pmic_validParamStatusCheck(configCrcStat->validParams, PMIC_CONFIG_CRC_STAT_ERROR_VALID, status)) {
        regData = 0U;
        Pmic_setBitField_b(&regData, CFG_REG_CRC_ERR_SHIFT, (bool)true);
        status = Pmic_ioTxByte_CS(handle, REG_STAT_REG, regData);
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_configCrcCalculate(const Pmic_Handle_t *handle)
{
    int32_t status = Pmic_checkHandle(handle);
    uint16_t crc = 0U;

    Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);

    if (status == PMIC_ST_SUCCESS) {
        status = CORE_calculateCrc(handle, &crc);
    }

    // Write calculated CRC LSB to CFG_REG_CRC0, MSB to CFG_REG_CRC1
    if (status == PMIC_ST_SUCCESS) {
        status = Pmic_ioTxByte(handle, CFG_REG_CRC0_REG, (uint8_t)(crc & 0xFFU));
    }

    if (status == PMIC_ST_SUCCESS) {
        status = Pmic_ioTxByte(handle, CFG_REG_CRC1_REG, (uint8_t)((crc >> 8U) & 0xFFU));
    }

    Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);

    // Trigger hardware re-check to confirm the stored CRC is correct
    if (status == PMIC_ST_SUCCESS) {
        status = CORE_configCrcValidate(handle);
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_setConfigCrc(const Pmic_Handle_t *handle, uint16_t value)
{
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);

    if (status == PMIC_ST_SUCCESS) {
        regData = value & 0xFFU;
        status = Pmic_ioTxByte(handle, CFG_REG_CRC0_REG, regData);
    }

    if (status == PMIC_ST_SUCCESS) {
        regData = (value >> 8U) & 0xFFU;
        status = Pmic_ioTxByte(handle, CFG_REG_CRC1_REG, regData);
    }

    Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_getConfigCrc(const Pmic_Handle_t *handle, uint16_t *value)
{
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData0 = 0U;
    uint8_t regData1 = 0U;

    if ((status == PMIC_ST_SUCCESS) && (value == NULL)) {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);

    if (status == PMIC_ST_SUCCESS) {
        status = Pmic_ioRxByte(handle, CFG_REG_CRC0_REG, &regData0);
    }

    if (status == PMIC_ST_SUCCESS) {
        status = Pmic_ioRxByte(handle, CFG_REG_CRC1_REG, &regData1);
    }

    Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);

    if (status == PMIC_ST_SUCCESS) {
        *value = (uint16_t)(((uint16_t)regData1 << 8U) | (uint16_t)regData0);
    }

    return Pmic_logStatus(handle, status);
}
