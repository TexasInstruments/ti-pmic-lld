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

static inline void CORE_copyGpioCfg(const Pmic_CoreCrc16Cfg_t *src, Pmic_CoreCrc16Cfg_t *dst)
{
    (void)memmove((void *)dst, (const void *)src, sizeof(Pmic_CoreCrc16Cfg_t));
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


int32_t Pmic_setCRC16Cfg(const Pmic_Handle_t *handle, const Pmic_CoreCrc16Cfg_t *crc16Cfg)
{
    Pmic_CoreCrc16Cfg_t localCfg;
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    if ((status == PMIC_ST_SUCCESS) && (crc16Cfg == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if (status != PMIC_ST_SUCCESS)
    {
        return Pmic_logStatus(handle, status);
    }

    if (crc16Cfg->validParams == 0U)
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        CORE_copyGpioCfg(crc16Cfg, &localCfg);
    }

    // Read CONFIG_CRC_CONFIG register
    Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioRxByte(handle, PMIC_CONFIG_CRC_CONFIG_REG, &regData);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Modify CONFIG_CRC_CALC bit field
        if (Pmic_validParamCheck(localCfg.validParams, PMIC_CRC16_ACTIVATE_CALC_VALID))
        {
            Pmic_setBitField_b(&regData, PMIC_CONFIG_CRC_CALC_SHIFT, PMIC_CONFIG_CRC_CALC_MASK, localCfg.activateCalc);
        }

        // Modify CONFIG_CRC_EN bit field
        if (Pmic_validParamCheck(localCfg.validParams, PMIC_CRC16_ENABLE_VALID))
        {
            Pmic_setBitField_b(&regData, PMIC_CONFIG_CRC_EN_SHIFT, PMIC_CONFIG_CRC_EN_MASK, localCfg.enable);
        }

        // Write new register value back to PMIC
        status = Pmic_ioTxByte(handle, PMIC_CONFIG_CRC_CONFIG_REG, regData);
    }
    Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_getCRC16Cfg(const Pmic_Handle_t *handle, Pmic_CoreCrc16Cfg_t *crc16Cfg)
{
    Pmic_CoreCrc16Cfg_t localCfg;
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    if ((status == PMIC_ST_SUCCESS) && (crc16Cfg == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if (status != PMIC_ST_SUCCESS)
    {
        return Pmic_logStatus(handle, status);
    }

    if (crc16Cfg->validParams == 0U)
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        CORE_copyGpioCfg(crc16Cfg, &localCfg);
    }

    // Read CONFIG_CRC_CONFIG register
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioRxByte_CS(handle, PMIC_CONFIG_CRC_CONFIG_REG, &regData);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Extract CONFIG_CRC_CALC bit field
        if (Pmic_validParamCheck(localCfg.validParams, PMIC_CRC16_ACTIVATE_CALC_VALID))
        {
            localCfg.activateCalc = Pmic_getBitField_b(regData, PMIC_CONFIG_CRC_CALC_SHIFT);
        }

        // Extract CONFIG_CRC_EN bit field
        if (Pmic_validParamCheck(localCfg.validParams, PMIC_CRC16_ENABLE_VALID))
        {
            localCfg.enable = Pmic_getBitField_b(regData, PMIC_CONFIG_CRC_EN_SHIFT);
        }
    }

    if (status == PMIC_ST_SUCCESS)
    {
        CORE_copyGpioCfg(&localCfg, crc16Cfg);
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
