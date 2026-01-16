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
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#include "pmic.h"
#include "pmic_common.h"
#include "pmic_io.h"
#include "pmic_core.h"
#include "regmap/core.h"

// Values for locking and unlocking registers
#define PMIC_REG_UNLOCK ((uint8_t)0x9BU)
#define PMIC_REG_LOCK   ((uint8_t)0x00U)

// Values for config CRC calculation, register ranges are inclusive
#define CONFIG_CRC_INIT   (0xFFFFU)
#define CONFIG_CRC_REG_LO (0x14U)
#define CONFIG_CRC_REG_HI (0x43U)

static uint16_t CORE_Crc16Calc(uint16_t crc, uint16_t data)
{
    static const uint16_t CRC16LUT[] = {
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

    const uint16_t index = MIN(data ^ (uint16_t)((crc & 0xFFFFU) >> 8U), 0xFFU);
    const uint16_t calculatedCrc = (uint16_t)(crc << 8U) ^ CRC16LUT[index];

    return (uint16_t)(calculatedCrc & 0xFFFFU);
}

static inline void CORE_copyConfigCrcStat(const Pmic_ConfigCrcStat_t *src, Pmic_ConfigCrcStat_t *dst) {
    memmove((void *)dst, (const void *)src, sizeof(Pmic_ConfigCrcStat_t));
}

int32_t Pmic_setScratchPadValue(const Pmic_Handle_t *handle, uint8_t scratchPadRegNum, uint8_t value)
{
    int32_t status = Pmic_checkHandle(handle);

    if ((status == PMIC_ST_SUCCESS) && (scratchPadRegNum > PMIC_SCRATCH_PAD_REG_MAX))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    // Set scratchpad value
    if (status == PMIC_ST_SUCCESS)
    {
        Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);
        status = Pmic_ioTxByte(handle, (uint8_t)(PMIC_SCRATCH_PAD_REG_1_REG + scratchPadRegNum), value);
        Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_getScratchPadValue(const Pmic_Handle_t *handle, uint8_t scratchPadRegNum, uint8_t *value)
{
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

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
        status = Pmic_ioRxByte_CS(handle, (uint8_t)(PMIC_SCRATCH_PAD_REG_1_REG + scratchPadRegNum), &regData);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        *value = regData;
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_setRegLockState(const Pmic_Handle_t *handle, bool lockState)
{
    int32_t status = Pmic_checkHandle(handle);
    const uint8_t key = (lockState == PMIC_LOCK_ENABLE) ? PMIC_REG_LOCK : PMIC_REG_UNLOCK;

    // Write the key to REGISTER_LOCK
    if (status == PMIC_ST_SUCCESS)
    {
        Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);
        status = Pmic_ioTxByte(handle, PMIC_REGISTER_LOCK_REG, key);
        Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_getRegLockState(const Pmic_Handle_t *handle, bool *lockState)
{
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    if ((status == PMIC_ST_SUCCESS) && (lockState == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Read REGISTER_LOCK
        status = Pmic_ioRxByte_CS(handle, PMIC_REGISTER_LOCK_REG, &regData);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Extract REGISTER_LOCK_STATUS bit field
        *lockState = Pmic_getBitField_b(regData, PMIC_REGISTER_LOCK_STATUS_SHIFT);
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_configCrcEnable(Pmic_Handle_t *handle, bool calculate)
{
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    // If user requested CRC calculation, do that now
    if ((status == PMIC_ST_SUCCESS) && (calculate == PMIC_CFG_CRC_RECALCULATE)) {
        status = Pmic_configCrcCalculate(handle);
    }

    // Set the CRC_EN bit (only) and write it to the CONFIG_CRC_CONFIG register
    if (status == PMIC_ST_SUCCESS) {
        Pmic_setBitField_b(&regData, CONFIG_CRC_EN_SHIFT, PMIC_ENABLE);

        Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);
        status = Pmic_ioTxByte(handle, CONFIG_CRC_CONFIG_REG, regData);
        Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);
    }

    // Update the handle
    if (status == PMIC_ST_SUCCESS) {
        handle->configCrcEnable = PMIC_ENABLE;
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_configCrcDisable(Pmic_Handle_t *handle)
{
    int32_t status = Pmic_checkHandle(handle);

    // Write 0x00 to the CONFIG_CRC_CONFIG register in order to disable this
    // feature as described in the TRM.
    if (status == PMIC_ST_SUCCESS) {
        Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);
        status = Pmic_ioTxByte(handle, CONFIG_CRC_CONFIG_REG, 0x00U);
        Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);
    }

    // Update the handle
    if (status == PMIC_ST_SUCCESS) {
        handle->configCrcEnable = PMIC_DISABLE;
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_getConfigCrcStatus(const Pmic_Handle_t *handle, Pmic_ConfigCrcStat_t *configCrcStat)
{
    Pmic_ConfigCrcStat_t localConfigCrcStat;
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    if ((status == PMIC_ST_SUCCESS) && (configCrcStat == NULL)) {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    // Read CONFIG_CRC_CONFIG
    if (status == PMIC_ST_SUCCESS) {
        status = Pmic_ioRxByte_CS(handle, CONFIG_CRC_CONFIG_REG, &regData);
    }

    if (status == PMIC_ST_SUCCESS) {
        // Extract CONFIG_CRC_EN
        localConfigCrcStat.crcEn = Pmic_getBitField_b(regData, CONFIG_CRC_EN_SHIFT);

        // Extract CONFIG_CRC_CALC
        localConfigCrcStat.crcCalc = Pmic_getBitField_b(regData, CONFIG_CRC_CALC_SHIFT);
    }

    // Read STAT_MODERATE_ERR
    if (status == PMIC_ST_SUCCESS) {
        status = Pmic_ioRxByte_CS(handle, STAT_MODERATE_ERR_REG, &regData);
    }

    // Extract CONFIG_CRC_STAT
    if (status == PMIC_ST_SUCCESS) {
        localConfigCrcStat.errorDetected = Pmic_getBitField_b(regData, CONFIG_CRC_STAT_SHIFT);
    }

    if (status == PMIC_ST_SUCCESS) {
        CORE_copyConfigCrcStat(&localConfigCrcStat, configCrcStat);
    }

    return Pmic_logStatus(handle, status);
}

static int32_t CORE_configCrcValidate(const Pmic_Handle_t *handle)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);
    status = Pmic_ioRxByte(handle, CONFIG_CRC_CONFIG_REG, &regData);

    // This operation should only be performed if Config CRC feature is
    // currently disabled
    if (Pmic_getBitField_b(regData, CONFIG_CRC_EN_SHIFT)) {
        status = PMIC_ST_ERR_NOT_SUPPORTED;
    }

    // If the CRC_CALC bit is already high, set it low, calculation is triggered
    // by rising edge of this signal
    if ((status == PMIC_ST_SUCCESS) && Pmic_getBitField_b(regData, CONFIG_CRC_CALC_SHIFT)) {
        Pmic_setBitField_b(&regData, CONFIG_CRC_CALC_SHIFT, PMIC_DISABLE);
        status = Pmic_ioTxByte(handle, CONFIG_CRC_CONFIG_REG, regData);
    }

    // Set the CRC_CALC bit and write to the register
    if (status == PMIC_ST_SUCCESS) {
        Pmic_setBitField_b(&regData, CONFIG_CRC_CALC_SHIFT, PMIC_ENABLE);
        status = Pmic_ioTxByte(handle, CONFIG_CRC_CONFIG_REG, regData);
    }

    // Now must allow the CONFIG_CRC_CALC bit to remain high for at least 30us,
    // this timing should be met automatically by the transmission speed of I2C
    if (status == PMIC_ST_SUCCESS) {
        status = Pmic_ioRxByte(handle, CONFIG_CRC_CONFIG_REG, &regData);
    }

    // Set the CRC_CALC bit back low as a good citizen, it is not self clearing,
    // we can write all 0's at this point as CONFIG_CRC_STATUS is RO, and we
    // have already confirmed that CONFIG_CRC_EN must be zero.
    if (status == PMIC_ST_SUCCESS) {
        status = Pmic_ioTxByte(handle, CONFIG_CRC_CONFIG_REG, 0x00U);
    }

    // If the CRC_STATUS bit is set then the calculated CRC does not match,
    // return an error code, otherwise we can return success
    if ((status == PMIC_ST_SUCCESS) && Pmic_getBitField_b(regData, CONFIG_CRC_STATUS_SHIFT)) {
        status = PMIC_ST_ERR_CONFIG_REG_CRC;
    }

    Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);
    return status;
}

/** @brief Calculate config CRC over register range */
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

int32_t Pmic_configCrcCalculate(const Pmic_Handle_t *handle)
{
    int32_t status = Pmic_checkHandle(handle);
    uint16_t crc = 0U;

    Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);

    if (status == PMIC_ST_SUCCESS) {
        status = CORE_calculateCrc(handle, &crc);
    }

    // Write calculated CRC (LSB)
    if (status == PMIC_ST_SUCCESS) {
        status = Pmic_ioTxByte(handle, CONFIG_CRC_REG_1_REG, (uint8_t)(crc & 0xFFU));
    }

    // Write calculated CRC (MSB)
    if (status == PMIC_ST_SUCCESS) {
        status = Pmic_ioTxByte(handle, CONFIG_CRC_REG_2_REG, (uint8_t)((crc >> 8U) & 0xFFU));
    }

    Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);

    // Perform CRC validation process
    if (status == PMIC_ST_SUCCESS) {
        status = CORE_configCrcValidate(handle);
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_configCrcGetFromDevice(const Pmic_Handle_t *handle, uint16_t *crc)
{
    int32_t status = Pmic_checkHandle(handle);
    uint8_t crcMsb = 0U;
    uint8_t crcLsb = 0U;

    if ((status == PMIC_ST_SUCCESS) && (crc == NULL)) {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if (status == PMIC_ST_SUCCESS) {
        status = Pmic_ioRxByte_CS(handle, CALCUL_CONFIG_CRC_1_REG, &crcLsb);
    }

    if (status == PMIC_ST_SUCCESS) {
        status = Pmic_ioRxByte_CS(handle, CALCUL_CONFIG_CRC_2_REG, &crcMsb);
    }

    if (status == PMIC_ST_SUCCESS) {
        *crc = (uint16_t)(((uint16_t)crcMsb << 8U) | crcLsb);
    }

    return Pmic_logStatus(handle, status);
}
