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
 * @brief This file contains definitions to APIs that interact with the PMIC Core.
 */
#include "pmic.h"
#include "pmic_io.h"

#include "pmic_core.h"
#include "regmap/core.h"

#define CONFIG_CRC_INIT    (0xFFFFU)
#define CONFIG_CRC_REG_LO  ((uint8_t)0x14U)
#define CONFIG_CRC_REG_HI  ((uint8_t)0x4CU)

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

static int32_t CORE_calculateCrc(const Pmic_Handle_t *handle, uint16_t *crc)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    uint8_t regAddr;

    *crc = CONFIG_CRC_INIT;
    for (regAddr = CONFIG_CRC_REG_LO; regAddr <= CONFIG_CRC_REG_HI; regAddr++)
    {
        status = Pmic_ioRxByte(handle, regAddr, &regData);
        if (status != PMIC_ST_SUCCESS)
        {
            break;
        }
        *crc = CORE_Crc16Calc(*crc, (uint16_t)regData);
    }

    return status;
}

static int32_t CORE_configCrcValidate(const Pmic_Handle_t *handle)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);

    status = Pmic_ioRxByte(handle, PMIC_CONFIG_CRC_CONFIG_REG, &regData);

    if ((status == PMIC_ST_SUCCESS) && Pmic_getBitField_b(regData, PMIC_CONFIG_CRC_EN_SHIFT))
    {
        status = PMIC_ST_ERR_NOT_SUPPORTED;
    }

    // Ensure clean rising edge on CONFIG_CRC_CALC
    if ((status == PMIC_ST_SUCCESS) && Pmic_getBitField_b(regData, PMIC_CONFIG_CRC_CALC_SHIFT))
    {
        Pmic_setBitField_b(&regData, PMIC_CONFIG_CRC_CALC_SHIFT, PMIC_CONFIG_CRC_CALC_MASK, (bool)false);
        status = Pmic_ioTxByte(handle, PMIC_CONFIG_CRC_CONFIG_REG, regData);
    }

    // Assert CONFIG_CRC_CALC to trigger hardware CRC comparison
    if (status == PMIC_ST_SUCCESS)
    {
        Pmic_setBitField_b(&regData, PMIC_CONFIG_CRC_CALC_SHIFT, PMIC_CONFIG_CRC_CALC_MASK, (bool)true);
        status = Pmic_ioTxByte(handle, PMIC_CONFIG_CRC_CONFIG_REG, regData);
    }

    // Read back CONFIG_CRC_CONFIG to capture CONFIG_CRC_STATUS
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioRxByte(handle, PMIC_CONFIG_CRC_CONFIG_REG, &regData);
    }

    // Clear CONFIG_CRC_CALC
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioTxByte(handle, PMIC_CONFIG_CRC_CONFIG_REG, 0x00U);
    }

    Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);

    if ((status == PMIC_ST_SUCCESS) && Pmic_getBitField_b(regData, PMIC_CONFIG_CRC_STATUS_SHIFT))
    {
        status = PMIC_ST_ERR_CONFIG_REG_CRC;
    }

    return status;
}

static inline void CORE_copyLpmCfg(const Pmic_CoreLpmCfg_t *src, Pmic_CoreLpmCfg_t *dst)
{
    (void)memmove((void *)dst, (const void *)src, sizeof(Pmic_CoreLpmCfg_t));
}

int32_t Pmic_getNvmRev(const Pmic_Handle_t *handle, uint8_t *nvmRev)
{
    int32_t status = Pmic_checkHandle(handle);

    if ((status == PMIC_ST_SUCCESS) && (nvmRev == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioRxByte_CS(handle, PMIC_NVM_CODE_2_REG, nvmRev);
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_getSiliconRev(const Pmic_Handle_t *handle, uint8_t *siliconRev)
{
    int32_t status = Pmic_checkHandle(handle);

    if ((status == PMIC_ST_SUCCESS) && (siliconRev == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioRxByte_CS(handle, PMIC_MANUFACTURING_VER_REG, siliconRev);
    }

    return Pmic_logStatus(handle, status);
}


static int32_t CORE_setLpmDetectionCfg(const Pmic_Handle_t *handle, const Pmic_CoreLpmCfg_t *lpmCfg)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    // Read LOW_PWR_CONFIG
    Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);
    if (Pmic_validParamCheck(lpmCfg->validParams, PMIC_LPM_PIN_DETECTION_VALID | PMIC_LPM_DETECTION_DELAY_VALID))
    {
        status = Pmic_ioRxByte(handle, PMIC_LOW_PWR_CONFIG_REG, &regData);
    }

    // Modify LOWPWR_SEL bit field
    if (Pmic_validParamStatusCheck(lpmCfg->validParams, PMIC_LPM_PIN_DETECTION_VALID, status))
    {
        if ((lpmCfg->pinDetection > PMIC_PIN_DETECTION_CONDITION_MAX))
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
        else
        {
            Pmic_setBitField(&regData, PMIC_LOWPWR_SEL_SHIFT, PMIC_LOWPWR_SEL_MASK, lpmCfg->pinDetection);
        }
    }

    // Modify LOWPWR_DELAY bit field
    if (Pmic_validParamStatusCheck(lpmCfg->validParams, PMIC_LPM_DETECTION_DELAY_VALID, status))
    {
        if (lpmCfg->detectionDelay > PMIC_DETECTION_DELAY_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
        else
        {
            Pmic_setBitField(&regData, PMIC_LOWPWR_DELAY_SHIFT, PMIC_LOWPWR_DELAY_MASK, lpmCfg->detectionDelay);
        }
    }

    // Write new register value back to PMIC
    if (Pmic_validParamStatusCheck(lpmCfg->validParams, PMIC_LPM_PIN_DETECTION_VALID | PMIC_LPM_DETECTION_DELAY_VALID, status))
    {
        status = Pmic_ioTxByte(handle, PMIC_LOW_PWR_CONFIG_REG, regData);
    }
    Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_setLpmCfg(const Pmic_Handle_t *handle, const Pmic_CoreLpmCfg_t *lpmCfg)
{
    Pmic_CoreLpmCfg_t localCfg = {0};
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    if ((status == PMIC_ST_SUCCESS) && (lpmCfg == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if (status != PMIC_ST_SUCCESS)
    {
        return Pmic_logStatus(handle, status);
    }

    if (lpmCfg->validParams == 0U)
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        CORE_copyLpmCfg(lpmCfg, &localCfg);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        status = CORE_setLpmDetectionCfg(handle, &localCfg);
    }

    Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);
    if (Pmic_validParamStatusCheck(
            localCfg.validParams, PMIC_LPM_ENABLE_ALL_VALID, status))
    {
        // Read LPM_CONF register
        status = Pmic_ioRxByte(handle, PMIC_LPM_CONF_REG, &regData);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Modify LOWPWR_VMON_EN bit field
        if (Pmic_validParamCheck(localCfg.validParams, PMIC_LPM_VMON_EN_VALID))
        {
            Pmic_setBitField_b(&regData, PMIC_LOWPWR_VMON_EN_SHIFT, PMIC_LOWPWR_VMON_EN_MASK, localCfg.vmonEn);
        }

        // Modify LOWPWR_ESM_EN bit field
        if (Pmic_validParamCheck(localCfg.validParams, PMIC_LPM_ESM_EN_VALID))
        {
            Pmic_setBitField_b(&regData, PMIC_LOWPWR_ESM_EN_SHIFT, PMIC_LOWPWR_ESM_EN_MASK, localCfg.esmEn);
        }

        // Modify LOWPWR_WD_EN bit field
        if (Pmic_validParamCheck(localCfg.validParams, PMIC_LPM_WDG_EN_VALID))
        {
            Pmic_setBitField_b(&regData, PMIC_LOWPWR_WD_EN_SHIFT, PMIC_LOWPWR_WD_EN_MASK, localCfg.wdgEn);
        }
    }

    // Write new register value back to PMIC
    if (Pmic_validParamStatusCheck(
        localCfg.validParams, PMIC_LPM_ENABLE_ALL_VALID, status))
    {
        status = Pmic_ioTxByte(handle, PMIC_LPM_CONF_REG, regData);
    }
    Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);

    return Pmic_logStatus(handle, status);
}

static int32_t CORE_getLpmDetectionCfg(const Pmic_Handle_t *handle, Pmic_CoreLpmCfg_t *lpmCfg)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    if (Pmic_validParamCheck(lpmCfg->validParams, PMIC_LPM_PIN_DETECTION_VALID | PMIC_LPM_DETECTION_DELAY_VALID))
    {
        // Read LOW_PWR_CONFIG
        status = Pmic_ioRxByte_CS(handle, PMIC_LOW_PWR_CONFIG_REG, &regData);

        // Extract LOWPWR_SEL bit field
        if (Pmic_validParamStatusCheck(lpmCfg->validParams, PMIC_LPM_PIN_DETECTION_VALID, status))
        {
            lpmCfg->pinDetection = Pmic_getBitField(regData, PMIC_LOWPWR_SEL_SHIFT, PMIC_LOWPWR_SEL_MASK);
        }

        // Extract LOWPWR_DELAY bit field
        if (Pmic_validParamStatusCheck(lpmCfg->validParams, PMIC_LPM_DETECTION_DELAY_VALID, status))
        {
            lpmCfg->detectionDelay = Pmic_getBitField(regData, PMIC_LOWPWR_DELAY_SHIFT, PMIC_LOWPWR_DELAY_MASK);
        }
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_getLpmCfg(const Pmic_Handle_t *handle, Pmic_CoreLpmCfg_t *lpmCfg)
{
    Pmic_CoreLpmCfg_t localCfg = {0};
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    if ((status == PMIC_ST_SUCCESS) && (lpmCfg == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if (status != PMIC_ST_SUCCESS)
    {
        return Pmic_logStatus(handle, status);
    }

    if (lpmCfg->validParams == 0U)
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        CORE_copyLpmCfg(lpmCfg, &localCfg);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        status = CORE_getLpmDetectionCfg(handle, &localCfg);
    }

    if (Pmic_validParamStatusCheck(localCfg.validParams, PMIC_LPM_VMON_EN_VALID | PMIC_LPM_ESM_EN_VALID | PMIC_LPM_WDG_EN_VALID, status))
    {
        // Read LPM_CONF register
        status = Pmic_ioRxByte_CS(handle, PMIC_LPM_CONF_REG, &regData);

        if (status == PMIC_ST_SUCCESS)
        {
            // Extract LOWPWR_VMON_EN bit field
            if (Pmic_validParamCheck(localCfg.validParams, PMIC_LPM_VMON_EN_VALID))
            {
                localCfg.vmonEn = Pmic_getBitField_b(regData, PMIC_LOWPWR_VMON_EN_SHIFT);
            }

            // Extract LOWPWR_ESM_EN bit field
            if (Pmic_validParamCheck(localCfg.validParams, PMIC_LPM_ESM_EN_VALID))
            {
                localCfg.esmEn = Pmic_getBitField_b(regData, PMIC_LOWPWR_ESM_EN_SHIFT);
            }

            // Extract LOWPWR_WD_EN bit field
            if (Pmic_validParamCheck(localCfg.validParams, PMIC_LPM_WDG_EN_VALID))
            {
                localCfg.wdgEn = Pmic_getBitField_b(regData, PMIC_LOWPWR_WD_EN_SHIFT);
            }
        }
    }

    if (status == PMIC_ST_SUCCESS)
    {
        CORE_copyLpmCfg(&localCfg, lpmCfg);
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_getABISTStat(const Pmic_Handle_t *handle, bool *isActive)
{
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    if ((status == PMIC_ST_SUCCESS) && (isActive == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Read STAT_MISC register
        status = Pmic_ioRxByte_CS(handle, PMIC_STAT_MISC_REG, &regData);

        // Extract ABIST_ACTIVE_STAT bit field
        if (status == PMIC_ST_SUCCESS)
        {
            *isActive = Pmic_getBitField_b(regData, PMIC_ABIST_ACTIVE_STAT_SHIFT);
        }
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_runABIST(const Pmic_Handle_t *handle)
{
    int32_t status = Pmic_checkHandle(handle);

    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioTxByte_CS(handle, PMIC_ABIST_RUN_CMD_REG, PMIC_RUN_ABIST_COMMAND);
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_getPwrOn(const Pmic_Handle_t *handle, bool *pwrOnStat)
{
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    if ((status == PMIC_ST_SUCCESS) && (pwrOnStat == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    // Read FUNC_CONF register
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioRxByte_CS(handle, PMIC_FUNC_CONF_REG, &regData);
    }

    // Extract PWR_ON bit field
    if (status == PMIC_ST_SUCCESS)
    {
        *pwrOnStat = Pmic_getBitField_b(regData, PMIC_PWR_ON_SHIFT);
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_setPwrOn(const Pmic_Handle_t *handle, bool pwrOn)
{
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);
    if (status == PMIC_ST_SUCCESS)
    {
        // Read FUNC_CONF register
        status = Pmic_ioRxByte(handle, PMIC_FUNC_CONF_REG, &regData);
    }

    // Modify PWR_ON bit field and write new register value back to PMIC
    if (status == PMIC_ST_SUCCESS)
    {
        Pmic_setBitField_b(&regData, PMIC_PWR_ON_SHIFT, PMIC_PWR_ON_MASK, pwrOn);

        status = Pmic_ioTxByte(handle, PMIC_FUNC_CONF_REG, regData);
    }
    Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);

    return Pmic_logStatus(handle, status);
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
        status = Pmic_ioTxByte_CS(handle, PMIC_SCRATCH_PAD_REG_1_REG + scratchPadRegNum, value);
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
        status = Pmic_ioRxByte_CS(handle, PMIC_SCRATCH_PAD_REG_1_REG + scratchPadRegNum, &regData);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        *value = regData;
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_getRegLockState(const Pmic_Handle_t *handle, bool *regLockStat)
{
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    if ((status == PMIC_ST_SUCCESS) && (regLockStat == NULL))
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
        *regLockStat = Pmic_getBitField_b(regData, PMIC_REGISTER_LOCK_STATUS_SHIFT);
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_setRegLockState(const Pmic_Handle_t *handle, bool lock)
{
    int32_t status = Pmic_checkHandle(handle);
    const uint8_t key = lock ? PMIC_REG_LOCK : PMIC_REG_UNLOCK;

    // Write the key to REGISTER_LOCK
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioTxByte_CS(handle, PMIC_REGISTER_LOCK_REG, key);
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_enableRegLock(const Pmic_Handle_t *handle)
{
    return Pmic_setRegLockState(handle, PMIC_LOCK);
}

int32_t Pmic_disableRegLock(const Pmic_Handle_t *handle)
{
    return Pmic_setRegLockState(handle, PMIC_UNLOCK);
}

int32_t Pmic_setConfigCrc(const Pmic_Handle_t *handle, uint16_t value)
{
    int32_t status = Pmic_checkHandle(handle);

    Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);

    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioTxByte(handle, PMIC_CONFIG_CRC_REG_1_REG, (uint8_t)(value & 0xFFU));
    }

    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioTxByte(handle, PMIC_CONFIG_CRC_REG_2_REG, (uint8_t)((value >> 8U) & 0xFFU));
    }

    Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_getConfigCrc(const Pmic_Handle_t *handle, uint16_t *value)
{
    int32_t status = Pmic_checkHandle(handle);
    uint8_t lsb = 0U, msb = 0U;

    if ((status == PMIC_ST_SUCCESS) && (value == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);

    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioRxByte(handle, PMIC_CALCUL_CONFIG_CRC_1_REG, &lsb);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioRxByte(handle, PMIC_CALCUL_CONFIG_CRC_2_REG, &msb);
    }

    Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);

    if (status == PMIC_ST_SUCCESS)
    {
        *value = (uint16_t)((uint16_t)((uint16_t)msb << 8U) | (uint16_t)lsb);
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_configCrcCalculate(const Pmic_Handle_t *handle)
{
    int32_t status = Pmic_checkHandle(handle);
    uint16_t crc = 0U;

    Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);
    if (status == PMIC_ST_SUCCESS)
    {
        status = CORE_calculateCrc(handle, &crc);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioTxByte(handle, PMIC_CONFIG_CRC_REG_1_REG, (uint8_t)(crc & 0xFFU));
    }

    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioTxByte(handle, PMIC_CONFIG_CRC_REG_2_REG, (uint8_t)((crc >> 8U) & 0xFFU));
    }
    Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);

    if (status == PMIC_ST_SUCCESS)
    {
        status = CORE_configCrcValidate(handle);
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_configCrcEnable(const Pmic_Handle_t *handle, bool calculate)
{
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    if ((status == PMIC_ST_SUCCESS) && (calculate == PMIC_CFG_CRC_RECALCULATE))
    {
        status = Pmic_configCrcCalculate(handle);
    }

    Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioRxByte(handle, PMIC_CONFIG_CRC_CONFIG_REG, &regData);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        Pmic_setBitField_b(&regData, PMIC_CONFIG_CRC_EN_SHIFT, PMIC_CONFIG_CRC_EN_MASK, (bool)true);
        status = Pmic_ioTxByte(handle, PMIC_CONFIG_CRC_CONFIG_REG, regData);
    }
    Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_configCrcDisable(const Pmic_Handle_t *handle)
{
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioUpdateByte_bCS(handle, PMIC_CONFIG_CRC_CONFIG_REG, PMIC_CONFIG_CRC_EN_SHIFT, (bool)false);
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_getConfigCrcStatus(const Pmic_Handle_t *handle, Pmic_ConfigCrcStat_t *configCrcStat)
{
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    if ((status == PMIC_ST_SUCCESS) && (configCrcStat == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioRxByte_CS(handle, PMIC_CONFIG_CRC_CONFIG_REG, &regData);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        configCrcStat->crcEn = Pmic_getBitField_b(regData, PMIC_CONFIG_CRC_EN_SHIFT);
        configCrcStat->crcCalc = Pmic_getBitField_b(regData, PMIC_CONFIG_CRC_CALC_SHIFT);

        status = Pmic_ioRxByte_CS(handle, PMIC_STAT_MODERATE_ERR_REG, &regData);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        configCrcStat->errorDetected = Pmic_getBitField_b(regData, PMIC_CONFIG_CRC_STAT_SHIFT);
    }

    return Pmic_logStatus(handle, status);
}
