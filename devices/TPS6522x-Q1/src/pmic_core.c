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
/**
 * @file pmic_core.c
 *
 * @brief PMIC LLD Core module source file for TPS6522x-Q1.
 */
#include "pmic.h"
#include "pmic_core.h"
#include "pmic_io.h"
#include "regmap/core.h"
#include "regmap/irq.h"

#define PMIC_REG_LOCK_KEY   (0xAAU)
#define PMIC_REG_UNLOCK_KEY (0x9BU)

#define CONFIG_CRC_INIT       (0xFFFFU)
#define CONFIG_CRC_REG_LO     ((uint16_t)0x000U)
#define CONFIG_CRC_REG_HI     ((uint16_t)0x0EFU)
#define CONFIG_CRC_EXT_REG_LO ((uint16_t)0x401U)
#define CONFIG_CRC_EXT_REG_HI ((uint16_t)0x40AU)

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
        0x7CCBU, 0x0990U, 0x967DU, 0xE326U, 0xDCFCU, 0xA9A7U, 0x364AU, 0x4311U,
        0x49FEU, 0x3CA5U, 0xA348U, 0xD613U, 0xE9C9U, 0x9C92U, 0x037FU, 0x7624U,
        0x16A1U, 0x63FAU, 0xFC17U, 0x894CU, 0xB696U, 0xC3CDU, 0x5C20U, 0x297BU,
        0x2394U, 0x56CFU, 0xC922U, 0xBC79U, 0x83A3U, 0xF6F8U, 0x6915U, 0x1C4EU,
        0xA81FU, 0xDD44U, 0x42A9U, 0x37F2U, 0x0828U, 0x7D73U, 0xE29EU, 0x97C5U,
        0x9D2AU, 0xE871U, 0x779CU, 0x02C7U, 0x3D1DU, 0x4846U, 0xD7ABU, 0xA2F0U,
        0xC275U, 0xB72EU, 0x28C3U, 0x5D98U, 0x6242U, 0x1719U, 0x88F4U, 0xFDAFU,
        0xF740U, 0x821BU, 0x1DF6U, 0x68ADU, 0x5777U, 0x222CU, 0xBDC1U, 0xC89AU,
        0xF996U, 0x8CCDU, 0x1320U, 0x667BU, 0x59A1U, 0x2CFAU, 0xB317U, 0xC64CU,
        0xCCA3U, 0xB9F8U, 0x2615U, 0x534EU, 0x6C94U, 0x19CFU, 0x8622U, 0xF379U,
        0x93FCU, 0xE6A7U, 0x794AU, 0x0C11U, 0x33CBU, 0x4690U, 0xD97DU, 0xAC26U,
        0xA6C9U, 0xD392U, 0x4C7FU, 0x3924U, 0x06FEU, 0x73A5U, 0xEC48U, 0x9913U,
        0x2D42U, 0x5819U, 0xC7F4U, 0xB2AFU, 0x8D75U, 0xF82EU, 0x67C3U, 0x1298U,
        0x1877U, 0x6D2CU, 0xF2C1U, 0x879AU, 0xB840U, 0xCD1BU, 0x52F6U, 0x27ADU,
        0x4728U, 0x3273U, 0xAD9EU, 0xD8C5U, 0xE71FU, 0x9244U, 0x0DA9U, 0x78F2U,
        0x721DU, 0x0746U, 0x98ABU, 0xEDF0U, 0xD22AU, 0xA771U, 0x389CU, 0x4DC7U,
        0x8B5DU, 0xFE06U, 0x61EBU, 0x14B0U, 0x2B6AU, 0x5E31U, 0xC1DCU, 0xB487U,
        0xBE68U, 0xCB33U, 0x54DEU, 0x2185U, 0x1E5FU, 0x6B04U, 0xF4E9U, 0x81B2U,
        0xE137U, 0x946CU, 0x0B81U, 0x7EDAU, 0x4100U, 0x345BU, 0xABB6U, 0xDEEDU,
        0xD402U, 0xA159U, 0x3EB4U, 0x4BEFU, 0x7435U, 0x016EU, 0x9E83U, 0xEBD8U,
        0x5F89U, 0x2AD2U, 0xB53FU, 0xC064U, 0xFFBEU, 0x8AE5U, 0x1508U, 0x6053U,
        0x6ABCU, 0x1FE7U, 0x800AU, 0xF551U, 0xCA8BU, 0xBFD0U, 0x203DU, 0x5566U,
        0x35E3U, 0x40B8U, 0xDF55U, 0xAA0EU, 0x95D4U, 0xE08FU, 0x7F62U, 0x0A39U,
        0x00D6U, 0x758DU, 0xEA60U, 0x9F3BU, 0xA0E1U, 0xD5BAU, 0x4A57U, 0x3F0CU,
    };
    const uint8_t idx = (uint8_t)(data ^ (uint16_t)((crc & 0xFFFFU) >> 8U));
    return (uint16_t)((uint16_t)(crc << 8U) ^ CRC16LUT[idx]);
}

static int32_t CORE_calculateCrc(const Pmic_Handle_t *handle, uint16_t *crc)
{
    int32_t  status  = PMIC_ST_SUCCESS;
    uint8_t  regData = 0U;
    uint16_t regAddr;

    *crc = CONFIG_CRC_INIT;

    // Range 1: 0x000-0x0EF
    for (regAddr = CONFIG_CRC_REG_LO; regAddr <= CONFIG_CRC_REG_HI; regAddr++)
    {
        status = Pmic_ioRxByte(handle, regAddr, &regData);
        if (status != PMIC_ST_SUCCESS) { break; }
        *crc = CORE_Crc16Calc(*crc, (uint16_t)regData);
    }

    // Range 2: 0x401-0x40A (continue accumulating; uint16_t regAddr handles extended address)
    if (status == PMIC_ST_SUCCESS)
    {
        for (regAddr = CONFIG_CRC_EXT_REG_LO; regAddr <= CONFIG_CRC_EXT_REG_HI; regAddr++)
        {
            status = Pmic_ioRxByte(handle, regAddr, &regData);
            if (status != PMIC_ST_SUCCESS) { break; }
            *crc = CORE_Crc16Calc(*crc, (uint16_t)regData);
        }
    }

    return status;
}

static int32_t CORE_configCrcValidate(const Pmic_Handle_t *handle)
{
    int32_t status  = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);

    // Clear any pre-existing REG_CRC_ERR_INT latch (W1C) before triggering BIST
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioTxByte(handle, INT_MODERATE_ERR_REG,
                               (uint8_t)REG_CRC_ERR_INT_MASK);
    }

    // Assert RUN_CRC_BIST — triggers hardware to compare stored CRC against a
    // freshly computed value; bit auto-clears when BIST is complete
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioUpdateByte_b(handle, CRC_CALC_CONTROL_REG,
                                     RUN_CRC_BIST_SHIFT, true);
    }

    // I2C bus round-trip time is sufficient for hardware to complete the BIST;
    // read INT_MODERATE_ERR to check for a CRC mismatch
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioRxByte(handle, INT_MODERATE_ERR_REG, &regData);
    }

    if ((status == PMIC_ST_SUCCESS) &&
        Pmic_getBitField_b(regData, REG_CRC_ERR_INT_SHIFT))
    {
        status = PMIC_ST_ERR_CONFIG_REG_CRC;
    }

    Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);
    return status;
}

int32_t Pmic_setRegLockState(const Pmic_Handle_t *handle, bool lock)
{
    int32_t status = Pmic_checkHandle(handle);
    const uint8_t key = lock ? PMIC_REG_LOCK_KEY : PMIC_REG_UNLOCK_KEY;

    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioTxByte_CS(handle, REGISTER_LOCK_REG, key);
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_getRegLockState(const Pmic_Handle_t *handle, bool *isLocked)
{
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    if ((status == PMIC_ST_SUCCESS) && (isLocked == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioRxByte_CS(handle, REGISTER_LOCK_REG, &regData);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        *isLocked = Pmic_getBitField_b(regData, REGISTER_LOCK_STATUS_SHIFT);
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_getNvmRev(const Pmic_Handle_t *handle, uint8_t *nvmRev)
{
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    if ((status == PMIC_ST_SUCCESS) && (nvmRev == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioRxByte_CS(handle, NVM_CODE_2_REG, &regData);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        *nvmRev = Pmic_getBitField(regData, TI_NVM_REV_SHIFT, TI_NVM_REV_MASK);
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_getSiliconRev(const Pmic_Handle_t *handle, uint8_t *siliconRev)
{
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    if ((status == PMIC_ST_SUCCESS) && (siliconRev == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioRxByte_CS(handle, MANUFACTURING_VER_REG, &regData);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        *siliconRev = Pmic_getBitField(regData, SILICON_REV_SHIFT, SILICON_REV_MASK);
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_setScratchPadValue(const Pmic_Handle_t *handle, uint8_t scratchPadRegNum, uint8_t value)
{
    int32_t status = Pmic_checkHandle(handle);

    if ((status == PMIC_ST_SUCCESS) && (scratchPadRegNum > PMIC_SCRATCH_PAD_REG_MAX))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioTxByte_CS(handle, SCRATCH_PAD_REG_1_REG + scratchPadRegNum, value);
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_getScratchPadValue(const Pmic_Handle_t *handle, uint8_t scratchPadRegNum, uint8_t *value)
{
    int32_t status = Pmic_checkHandle(handle);

    if ((status == PMIC_ST_SUCCESS) && (scratchPadRegNum > PMIC_SCRATCH_PAD_REG_MAX))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) && (value == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioRxByte_CS(handle, SCRATCH_PAD_REG_1_REG + scratchPadRegNum, value);
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_configCrcRun(const Pmic_Handle_t *handle, bool update)
{
    int32_t status = Pmic_checkHandle(handle);

    if (status == PMIC_ST_SUCCESS)
    {
        if (update)
        {
            status = Pmic_ioUpdateByte_bCS(handle, CRC_CALC_CONTROL_REG, RUN_CRC_UPDATE_SHIFT, true);
        }
        else
        {
            status = Pmic_ioUpdateByte_bCS(handle, CRC_CALC_CONTROL_REG, RUN_CRC_BIST_SHIFT, true);
        }
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_setConfigCrc(const Pmic_Handle_t *handle, uint16_t value)
{
    int32_t status = Pmic_checkHandle(handle);

    Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioTxByte(handle, REGMAP_USER_CRC_HIGH_REG, (uint8_t)((value >> 8) & 0xFFU));
    }

    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioTxByte(handle, REGMAP_USER_CRC_LOW_REG, (uint8_t)(value & 0xFFU));
    }
    Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_getConfigCrc(const Pmic_Handle_t *handle, uint16_t *value)
{
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regDataHigh = 0U;
    uint8_t regDataLow = 0U;

    if ((status == PMIC_ST_SUCCESS) && (value == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioRxByte(handle, REGMAP_USER_CRC_HIGH_REG, &regDataHigh);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioRxByte(handle, REGMAP_USER_CRC_LOW_REG, &regDataLow);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        *value = (uint16_t)(((uint16_t)regDataHigh << 8U) | (uint16_t)regDataLow);
    }
    Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_configCrcCalculate(const Pmic_Handle_t *handle)
{
    int32_t  status = Pmic_checkHandle(handle);
    uint16_t crc    = 0U;

    Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);

    if (status == PMIC_ST_SUCCESS)
    {
        status = CORE_calculateCrc(handle, &crc);
    }

    // Write CRC to device: HIGH byte first then LOW, matching
    // Pmic_setConfigCrc convention (REGMAP_USER_CRC_HIGH_REG = 0xF1,
    // REGMAP_USER_CRC_LOW_REG = 0xF0)
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioTxByte(handle, REGMAP_USER_CRC_HIGH_REG,
                               (uint8_t)((crc >> 8U) & 0xFFU));
    }

    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioTxByte(handle, REGMAP_USER_CRC_LOW_REG,
                               (uint8_t)(crc & 0xFFU));
    }

    Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);

    // Trigger hardware BIST to verify that the stored CRC matches the
    // freshly computed hardware value
    if (status == PMIC_ST_SUCCESS)
    {
        status = CORE_configCrcValidate(handle);
    }

    return Pmic_logStatus(handle, status);
}
