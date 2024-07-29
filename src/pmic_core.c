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

static const uint16_t CRC16LUT[] = {
    0x0000, 0x755B, 0xEAB6, 0x9FED, 0xA037, 0xD56C, 0x4A81, 0x3FDA,
    0x3535, 0x406E, 0xDF83, 0xAAD8, 0x9502, 0xE059, 0x7FB4, 0x0AEF,
    0x6A6A, 0x1F31, 0x80DC, 0xF587, 0xCA5D, 0xBF06, 0x20EB, 0x55B0,
    0x5F5F, 0x2A04, 0xB5E9, 0xC0B2, 0xFF68, 0x8A33, 0x15DE, 0x6085,
    0xD4D4, 0xA18F, 0x3E62, 0x4B39, 0x74E3, 0x01B8, 0x9E55, 0xEB0E,
    0xE1E1, 0x94BA, 0x0B57, 0x7E0C, 0x41D6, 0x348D, 0xAB60, 0xDE3B,
    0xBEBE, 0xCBE5, 0x5408, 0x2153, 0x1E89, 0x6BD2, 0xF43F, 0x8164,
    0x8B8B, 0xFED0, 0x613D, 0x1466, 0x2BBC, 0x5EE7, 0xC10A, 0xB451,
    0xDCF3, 0xA9A8, 0x3645, 0x431E, 0x7CC4, 0x099F, 0x9672, 0xE329,
    0xE9C6, 0x9C9D, 0x0370, 0x762B, 0x49F1, 0x3CAA, 0xA347, 0xD61C,
    0xB699, 0xC3C2, 0x5C2F, 0x2974, 0x16AE, 0x63F5, 0xFC18, 0x8943,
    0x83AC, 0xF6F7, 0x691A, 0x1C41, 0x239B, 0x56C0, 0xC92D, 0xBC76,
    0x0827, 0x7D7C, 0xE291, 0x97CA, 0xA810, 0xDD4B, 0x42A6, 0x37FD,
    0x3D12, 0x4849, 0xD7A4, 0xA2FF, 0x9D25, 0xE87E, 0x7793, 0x02C8,
    0x624D, 0x1716, 0x88FB, 0xFDA0, 0xC27A, 0xB721, 0x28CC, 0x5D97,
    0x5778, 0x2223, 0xBDCE, 0xC895, 0xF74F, 0x8214, 0x1DF9, 0x68A2,
    0xCCBD, 0xB9E6, 0x260B, 0x5350, 0x6C8A, 0x19D1, 0x863C, 0xF367,
    0xF988, 0x8CD3, 0x133E, 0x6665, 0x59BF, 0x2CE4, 0xB309, 0xC652,
    0xA6D7, 0xD38C, 0x4C61, 0x393A, 0x06E0, 0x73BB, 0xEC56, 0x990D,
    0x93E2, 0xE6B9, 0x7954, 0x0C0F, 0x33D5, 0x468E, 0xD963, 0xAC38,
    0x1869, 0x6D32, 0xF2DF, 0x8784, 0xB85E, 0xCD05, 0x52E8, 0x27B3,
    0x2D5C, 0x5807, 0xC7EA, 0xB2B1, 0x8D6B, 0xF830, 0x67DD, 0x1286,
    0x7203, 0x0758, 0x98B5, 0xEDEE, 0xD234, 0xA76F, 0x3882, 0x4DD9,
    0x4736, 0x326D, 0xAD80, 0xD8DB, 0xE701, 0x925A, 0x0DB7, 0x78EC,
    0x104E, 0x6515, 0xFAF8, 0x8FA3, 0xB079, 0xC522, 0x5ACF, 0x2F94,
    0x257B, 0x5020, 0xCFCD, 0xBA96, 0x854C, 0xF017, 0x6FFA, 0x1AA1,
    0x7A24, 0x0F7F, 0x9092, 0xE5C9, 0xDA13, 0xAF48, 0x30A5, 0x45FE,
    0x4F11, 0x3A4A, 0xA5A7, 0xD0FC, 0xEF26, 0x9A7D, 0x0590, 0x70CB,
    0xC49A, 0xB1C1, 0x2E2C, 0x5B77, 0x64AD, 0x11F6, 0x8E1B, 0xFB40,
    0xF1AF, 0x84F4, 0x1B19, 0x6E42, 0x5198, 0x24C3, 0xBB2E, 0xCE75,
    0xAEF0, 0xDBAB, 0x4446, 0x311D, 0x0EC7, 0x7B9C, 0xE471, 0x912A,
    0x9BC5, 0xEE9E, 0x7173, 0x0428, 0x3BF2, 0x4EA9, 0xD144, 0xA41F,
};

static uint16_t CORE_Crc16Calc(uint16_t crc, uint16_t data)
{
    // MIN ensures index does not exceed bounds of the CRC16LUT array
    const uint16_t index = MIN(data ^ ((crc & 0xFFFF) >> 8), 0xFFU);
    crc = (uint16_t)(crc << 8U) ^ CRC16LUT[index];

    return crc & 0xFFFF;
}

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

int32_t Pmic_configCrcEnable(Pmic_CoreHandle_t *handle, bool calculate)
{
    int32_t status = Pmic_checkPmicCoreHandle(handle);
    uint8_t regData = 0U;

    // If user requested CRC calculation, do that now
    if ((status == PMIC_ST_SUCCESS) && (calculate == PMIC_CFG_CRC_RECALCULATE)) {
        status = Pmic_configCrcCalculate(handle);
    }

    // Set the CRC_EN bit (only) and write it to the CONFIG_CRC_CONFIG register
    if (status == PMIC_ST_SUCCESS) {
        Pmic_setBitField_b(&regData, CONFIG_CRC_EN_SHIFT, PMIC_ENABLE);

        Pmic_criticalSectionStart(handle);
        status = Pmic_ioTxByte(handle, CONFIG_CRC_CONFIG_REG, regData);
        Pmic_criticalSectionStop(handle);
    }

    // Update the handle
    if (status == PMIC_ST_SUCCESS) {
        handle->configCrcEnable = PMIC_ENABLE;
    }

    return status;
}

int32_t Pmic_configCrcDisable(Pmic_CoreHandle_t *handle)
{
    int32_t status = Pmic_checkPmicCoreHandle(handle);

    // Write 0x00 to the CONFIG_CRC_CONFIG register in order to disable this
    // feature as described in the TRM.
    if (status == PMIC_ST_SUCCESS) {
        Pmic_criticalSectionStart(handle);
        status = Pmic_ioTxByte(handle, CONFIG_CRC_CONFIG_REG, 0x00U);
        Pmic_criticalSectionStop(handle);
    }

    // Update the handle
    if (status == PMIC_ST_SUCCESS) {
        handle->configCrcEnable = PMIC_DISABLE;
    }

    return status;
}

static int32_t CORE_configCrcValidate(Pmic_CoreHandle_t *handle)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    Pmic_criticalSectionStart(handle);
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

    Pmic_criticalSectionStop(handle);
    return status;
}

int32_t Pmic_configCrcCalculate(Pmic_CoreHandle_t *handle)
{
    int32_t status = Pmic_checkPmicCoreHandle(handle);
    uint8_t regData = 0U;
    uint16_t crc = CONFIG_CRC_INIT;

    // Obtain critical section for the entirety of this operation
    Pmic_criticalSectionStart(handle);

    // Read each register in the config CRC range, and calculate cumulative CRC
    for (uint8_t regAddr = CONFIG_CRC_REG_LO; regAddr <= CONFIG_CRC_REG_HI; regAddr++) {
        status = Pmic_ioRxByte(handle, regAddr, &regData);
        crc = CORE_Crc16Calc(crc, (uint16_t)regData);

        // If a comms failure has occured, exit out of the calculation loop
        if (status != PMIC_ST_SUCCESS) {
            break;
        }
    }

    // Write calculated CRC (LSB)
    if (status == PMIC_ST_SUCCESS) {
        status = Pmic_ioTxByte(handle, CONFIG_CRC_REG_1_REG, (uint8_t)(crc & 0xFFU));
    }

    // Write calculated CRC (MSB)
    if (status == PMIC_ST_SUCCESS) {
        status = Pmic_ioTxByte(handle, CONFIG_CRC_REG_2_REG, (uint8_t)((crc >> 8U) & 0xFFU));
    }

    // Release critical section
    Pmic_criticalSectionStop(handle);

    // Perform CRC validation process
    if (status == PMIC_ST_SUCCESS) {
        status = CORE_configCrcValidate(handle);
    }

    return status;
}

int32_t Pmic_configCrcGetFromDevice(Pmic_CoreHandle_t *handle, uint16_t *crc)
{
    int32_t status = Pmic_checkPmicCoreHandle(handle);
    uint8_t crcMsb = 0U;
    uint8_t crcLsb = 0U;

    if ((status == PMIC_ST_SUCCESS) && (crc == NULL)) {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    // Obtain critical section
    Pmic_criticalSectionStart(handle);
    if (status == PMIC_ST_SUCCESS) {
        status = Pmic_ioRxByte(handle, CALCUL_CONFIG_CRC_1_REG, &crcLsb);
    }

    if (status == PMIC_ST_SUCCESS) {
        status = Pmic_ioRxByte(handle, CALCUL_CONFIG_CRC_2_REG, &crcMsb);
    }

    // Release critical section
    Pmic_criticalSectionStop(handle);
    *crc = (uint16_t)(((uint16_t)crcMsb << 8U) | crcLsb);

    return status;
}
