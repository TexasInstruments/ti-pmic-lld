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
/**
 * @file pmic_core.c
 *
 * @brief This file contains definitions to APIs that interact with the PMIC Core.
 */
#include "pmic.h"
#include "pmic_io.h"

#include "pmic_core.h"
#include "regmap/core.h"

int32_t Pmic_checkPmicCoreHandle(const Pmic_CoreHandle_t *pmicHandle)
{
    int32_t status = PMIC_ST_SUCCESS;

    if (pmicHandle == NULL)
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) && ((pmicHandle->commHandle == NULL) ||
        (pmicHandle->ioRead == NULL) || (pmicHandle->ioWrite == NULL) ||
        (pmicHandle->critSecStart == NULL) || (pmicHandle->critSecStop == NULL) ||
        (pmicHandle->drvInitStat != PMIC_DRV_INIT_SUCCESS)))
    {
        status = PMIC_ST_ERR_INV_HANDLE;
    }

    return status;
}

int32_t Pmic_getDevId(const Pmic_CoreHandle_t *pmicHandle, uint8_t *devId)
{
    int32_t status = Pmic_checkPmicCoreHandle(pmicHandle);

    if ((status == PMIC_ST_SUCCESS) && (devId == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        *devId = pmicHandle->devRev;
    }

    return status;
}

int32_t Pmic_getNvmId(const Pmic_CoreHandle_t *pmicHandle, uint8_t *nvmId)
{
    int32_t status = Pmic_checkPmicCoreHandle(pmicHandle);

    if ((status == PMIC_ST_SUCCESS) && (nvmId == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        *nvmId = pmicHandle->nvmId;
    }

    return status;
}

int32_t Pmic_getNvmRev(const Pmic_CoreHandle_t *pmicHandle, uint8_t *nvmRev)
{
    int32_t status = Pmic_checkPmicCoreHandle(pmicHandle);

    if ((status == PMIC_ST_SUCCESS) && (nvmRev == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        *nvmRev = pmicHandle->nvmRev;
    }

    return status;
}

int32_t Pmic_getSiliconRev(const Pmic_CoreHandle_t *pmicHandle, uint8_t *siliconRev)
{
    int32_t status = Pmic_checkPmicCoreHandle(pmicHandle);

    if ((status == PMIC_ST_SUCCESS) && (siliconRev == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        *siliconRev = pmicHandle->siliconRev;
    }

    return status;
}


int32_t Pmic_setCRC16Cfg(const Pmic_CoreHandle_t *pmicHandle, const Pmic_CoreCrc16Cfg_t *crc16Cfg)
{
    uint8_t regData = 0U;
    int32_t status = Pmic_checkPmicCoreHandle(pmicHandle);

    if ((status == PMIC_ST_SUCCESS) && (crc16Cfg == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) && (crc16Cfg->validParams == 0U))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    // Read CONFIG_CRC_CONFIG register
    if (status == PMIC_ST_SUCCESS)
    {
        Pmic_criticalSectionStart(pmicHandle);
        status = Pmic_ioRxByte(pmicHandle, PMIC_CONFIG_CRC_CONFIG_REGADDR, &regData);
        Pmic_criticalSectionStop(pmicHandle);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Modify CONFIG_CRC_CALC bit field
        if (Pmic_validParamCheck(crc16Cfg->validParams, PMIC_CRC16_ACTIVATE_CALC_VALID))
        {
            Pmic_setBitField_b(&regData, PMIC_CONFIG_CRC_CALC_SHIFT, PMIC_CONFIG_CRC_CALC_MASK, crc16Cfg->activateCalc);
        }

        // Modify CONFIG_CRC_EN bit field
        if (Pmic_validParamCheck(crc16Cfg->validParams, PMIC_CRC16_ENABLE_VALID))
        {
            Pmic_setBitField_b(&regData, PMIC_CONFIG_CRC_EN_SHIFT, PMIC_CONFIG_CRC_EN_MASK, crc16Cfg->enable);
        }

        // Write new register value back to PMIC
        Pmic_criticalSectionStart(pmicHandle);
        status = Pmic_ioTxByte(pmicHandle, PMIC_CONFIG_CRC_CONFIG_REGADDR, regData);
        Pmic_criticalSectionStop(pmicHandle);
    }

    return status;
}

int32_t Pmic_getCRC16Cfg(const Pmic_CoreHandle_t *pmicHandle, Pmic_CoreCrc16Cfg_t *crc16Cfg)
{
    uint8_t regData = 0U;
    int32_t status = Pmic_checkPmicCoreHandle(pmicHandle);

    if ((status == PMIC_ST_SUCCESS) && (crc16Cfg == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) && (crc16Cfg->validParams == 0U))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    // Read CONFIG_CRC_CONFIG register
    if (status == PMIC_ST_SUCCESS)
    {
        Pmic_criticalSectionStart(pmicHandle);
        status = Pmic_ioRxByte(pmicHandle, PMIC_CONFIG_CRC_CONFIG_REGADDR, &regData);
        Pmic_criticalSectionStop(pmicHandle);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Extract CONFIG_CRC_CALC bit field
        if (Pmic_validParamCheck(crc16Cfg->validParams, PMIC_CRC16_ACTIVATE_CALC_VALID))
        {
            crc16Cfg->activateCalc = Pmic_getBitField_b(regData, PMIC_CONFIG_CRC_CALC_SHIFT);
        }

        // Extract CONFIG_CRC_EN bit field
        if (Pmic_validParamCheck(crc16Cfg->validParams, PMIC_CRC16_ENABLE_VALID))
        {
            crc16Cfg->enable = Pmic_getBitField_b(regData, PMIC_CONFIG_CRC_EN_SHIFT);
        }
    }

    return status;
}

static int32_t CORE_setLpmDetectionCfg(const Pmic_CoreHandle_t *pmicHandle, const Pmic_CoreLpmCfg_t *lpmCfg)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    // Read LOW_PWR_CONFIG
    if (Pmic_validParamCheck(lpmCfg->validParams, PMIC_LPM_PIN_DETECTION_VALID | PMIC_LPM_DETECTION_DELAY_VALID))
    {
        Pmic_criticalSectionStart(pmicHandle);
        status = Pmic_ioRxByte(pmicHandle, PMIC_LOW_PWR_CONFIG_REGADDR, &regData);
        Pmic_criticalSectionStop(pmicHandle);
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
        Pmic_criticalSectionStart(pmicHandle);
        status = Pmic_ioTxByte(pmicHandle, PMIC_LOW_PWR_CONFIG_REGADDR, regData);
        Pmic_criticalSectionStop(pmicHandle);
    }

    return status;
}

int32_t Pmic_setLpmCfg(const Pmic_CoreHandle_t *pmicHandle, const Pmic_CoreLpmCfg_t *lpmCfg)
{
    uint8_t regData = 0U;
    int32_t status = Pmic_checkPmicCoreHandle(pmicHandle);

    if ((status == PMIC_ST_SUCCESS) && (lpmCfg == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) && (lpmCfg->validParams == 0U))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        status = CORE_setLpmDetectionCfg(pmicHandle, lpmCfg);
    }

    if (Pmic_validParamStatusCheck(
            lpmCfg->validParams, PMIC_LPM_VMON_EN_VALID | PMIC_LPM_ESM_EN_VALID | PMIC_LPM_WDG_EN_VALID, status))
    {
        // Read LPM_CONF register
        Pmic_criticalSectionStart(pmicHandle);
        status = Pmic_ioRxByte(pmicHandle, PMIC_LPM_CONF_REGADDR, &regData);
        Pmic_criticalSectionStop(pmicHandle);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Modify LOWPWR_VMON_EN bit field
        if (Pmic_validParamCheck(lpmCfg->validParams, PMIC_LPM_VMON_EN_VALID))
        {
            Pmic_setBitField_b(&regData, PMIC_LOWPWR_VMON_EN_SHIFT, PMIC_LOWPWR_VMON_EN_MASK, lpmCfg->vmonEn);
        }

        // Modify LOWPWR_ESM_EN bit field
        if (Pmic_validParamCheck(lpmCfg->validParams, PMIC_LPM_ESM_EN_VALID))
        {
            Pmic_setBitField_b(&regData, PMIC_LOWPWR_ESM_EN_SHIFT, PMIC_LOWPWR_ESM_EN_MASK, lpmCfg->esmEn);
        }

        // Modify LOWPWR_WD_EN bit field
        if (Pmic_validParamCheck(lpmCfg->validParams, PMIC_LPM_WDG_EN_VALID))
        {
            Pmic_setBitField_b(&regData, PMIC_LOWPWR_WD_EN_SHIFT, PMIC_LOWPWR_WD_EN_MASK, lpmCfg->wdgEn);
        }
    }

    // Write new register value back to PMIC
    if (Pmic_validParamStatusCheck(
        lpmCfg->validParams, PMIC_LPM_VMON_EN_VALID | PMIC_LPM_ESM_EN_VALID | PMIC_LPM_WDG_EN_VALID, status))
    {
        Pmic_criticalSectionStart(pmicHandle);
        status = Pmic_ioTxByte(pmicHandle, PMIC_LPM_CONF_REGADDR, regData);
        Pmic_criticalSectionStop(pmicHandle);
    }

    return status;
}

static int32_t CORE_getLpmDetectionCfg(const Pmic_CoreHandle_t *pmicHandle, Pmic_CoreLpmCfg_t *lpmCfg)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    if (Pmic_validParamCheck(lpmCfg->validParams, PMIC_LPM_PIN_DETECTION_VALID | PMIC_LPM_DETECTION_DELAY_VALID))
    {
        // Read LOW_PWR_CONFIG
        Pmic_criticalSectionStart(pmicHandle);
        status = Pmic_ioRxByte(pmicHandle, PMIC_LOW_PWR_CONFIG_REGADDR, &regData);
        Pmic_criticalSectionStop(pmicHandle);

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

    return status;
}

int32_t Pmic_getLpmCfg(const Pmic_CoreHandle_t *pmicHandle, Pmic_CoreLpmCfg_t *lpmCfg)
{
    uint8_t regData = 0U;
    int32_t status = Pmic_checkPmicCoreHandle(pmicHandle);

    if ((status == PMIC_ST_SUCCESS) && (lpmCfg == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) && (lpmCfg->validParams == 0U))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        status = CORE_getLpmDetectionCfg(pmicHandle, lpmCfg);
    }

    if (Pmic_validParamStatusCheck(lpmCfg->validParams, PMIC_LPM_VMON_EN_VALID | PMIC_LPM_ESM_EN_VALID | PMIC_LPM_WDG_EN_VALID, status))
    {
        // Read LPM_CONF register
        Pmic_criticalSectionStart(pmicHandle);
        status = Pmic_ioRxByte(pmicHandle, PMIC_LPM_CONF_REGADDR, &regData);
        Pmic_criticalSectionStop(pmicHandle);

        if (status == PMIC_ST_SUCCESS)
        {
            // Extract LOWPWR_VMON_EN bit field
            if (Pmic_validParamCheck(lpmCfg->validParams, PMIC_LPM_VMON_EN_VALID))
            {
                lpmCfg->vmonEn = Pmic_getBitField_b(regData, PMIC_LOWPWR_VMON_EN_SHIFT);
            }

            // Extract LOWPWR_ESM_EN bit field
            if (Pmic_validParamCheck(lpmCfg->validParams, PMIC_LPM_ESM_EN_VALID))
            {
                lpmCfg->esmEn = Pmic_getBitField_b(regData, PMIC_LOWPWR_ESM_EN_SHIFT);
            }

            // Extract LOWPWR_WD_EN bit field
            if (Pmic_validParamCheck(lpmCfg->validParams, PMIC_LPM_WDG_EN_VALID))
            {
                lpmCfg->wdgEn = Pmic_getBitField_b(regData, PMIC_LOWPWR_WD_EN_SHIFT);
            }
        }
    }

    return status;
}

int32_t Pmic_getABISTStat(const Pmic_CoreHandle_t *pmicHandle, bool *isActive)
{
    uint8_t regData = 0U;
    int32_t status = Pmic_checkPmicCoreHandle(pmicHandle);

    if ((status == PMIC_ST_SUCCESS) && (isActive == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Read STAT_MISC register
        Pmic_criticalSectionStart(pmicHandle);
        status = Pmic_ioRxByte(pmicHandle, PMIC_STAT_MISC_REGADDR, &regData);
        Pmic_criticalSectionStop(pmicHandle);

        // Extract ABIST_ACTIVE_STAT bit field
        if (status == PMIC_ST_SUCCESS)
        {
            *isActive = Pmic_getBitField_b(regData, PMIC_ABIST_ACTIVE_STAT_SHIFT);
        }
    }

    return status;
}

int32_t Pmic_runABIST(const Pmic_CoreHandle_t *pmicHandle)
{
    int32_t status = Pmic_checkPmicCoreHandle(pmicHandle);

    if (status == PMIC_ST_SUCCESS)
    {
        Pmic_criticalSectionStart(pmicHandle);
        status = Pmic_ioTxByte(pmicHandle, PMIC_ABIST_RUN_CMD_REGADDR, PMIC_RUN_ABIST_COMMAND);
        Pmic_criticalSectionStop(pmicHandle);
    }

    return status;
}

static int32_t CORE_checkFsmCmd(uint8_t fsmCmd)
{
    int32_t status = PMIC_ST_SUCCESS;

    if ((fsmCmd != PMIC_SAFE_RECOVERY_REQUEST) &&
        (fsmCmd != PMIC_COLD_BOOT_REQUEST) &&
        (fsmCmd != PMIC_LOW_POWER_ENTRY_REQUEST) &&
        (fsmCmd != PMIC_OFF_REQUEST) &&
        (fsmCmd != PMIC_LOW_POWER_EXIT_REQUEST) &&
        (fsmCmd != PMIC_WARM_RESET_REQUEST))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    return status;
}

int32_t Pmic_sendFsmCmd(const Pmic_CoreHandle_t *pmicHandle, uint8_t fsmCmd)
{
    int32_t status = Pmic_checkPmicCoreHandle(pmicHandle);

    if (status == PMIC_ST_SUCCESS)
    {
        status = CORE_checkFsmCmd(fsmCmd);
    }

    // Write FSM command to FSM_COMMAND_REG
    if (status == PMIC_ST_SUCCESS)
    {
        Pmic_criticalSectionStart(pmicHandle);
        status = Pmic_ioTxByte(pmicHandle, PMIC_FSM_COMMAND_REG_REGADDR, fsmCmd);
        Pmic_criticalSectionStop(pmicHandle);
    }

    return status;
}

int32_t Pmic_getPwrOn(const Pmic_CoreHandle_t *pmicHandle, bool *pwrOnStat)
{
    uint8_t regData = 0U;
    int32_t status = Pmic_checkPmicCoreHandle(pmicHandle);

    if ((status == PMIC_ST_SUCCESS) && (pwrOnStat == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    // Read FUNC_CONF register
    if (status == PMIC_ST_SUCCESS)
    {
        Pmic_criticalSectionStart(pmicHandle);
        status = Pmic_ioRxByte(pmicHandle, PMIC_FUNC_CONF_REGADDR, &regData);
        Pmic_criticalSectionStop(pmicHandle);
    }

    // Extract PWR_ON bit field
    if (status == PMIC_ST_SUCCESS)
    {
        *pwrOnStat = Pmic_getBitField_b(regData, PMIC_PWR_ON_SHIFT);
    }

    return status;
}

int32_t Pmic_setPwrOn(const Pmic_CoreHandle_t *pmicHandle, bool pwrOn)
{
    uint8_t regData = 0U;
    int32_t status = Pmic_checkPmicCoreHandle(pmicHandle);

    if (status == PMIC_ST_SUCCESS)
    {
        // Read FUNC_CONF register
        Pmic_criticalSectionStart(pmicHandle);
        status = Pmic_ioRxByte(pmicHandle, PMIC_FUNC_CONF_REGADDR, &regData);
        Pmic_criticalSectionStop(pmicHandle);
    }

    // Modify PWR_ON bit field and write new register value back to PMIC
    if (status == PMIC_ST_SUCCESS)
    {
        Pmic_setBitField_b(&regData, PMIC_PWR_ON_SHIFT, PMIC_PWR_ON_MASK, pwrOn);

        Pmic_criticalSectionStart(pmicHandle);
        status = Pmic_ioTxByte(pmicHandle, PMIC_FUNC_CONF_REGADDR, regData);
        Pmic_criticalSectionStop(pmicHandle);
    }

    return status;
}

int32_t Pmic_setScratchPadVal(const Pmic_CoreHandle_t *pmicHandle, uint8_t scratchPadRegNum, uint8_t value)
{
    int32_t status = Pmic_checkPmicCoreHandle(pmicHandle);

    if ((status == PMIC_ST_SUCCESS) && (scratchPadRegNum > PMIC_SCRATCH_PAD_REG_MAX))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    // Set scratchpad value
    if (status == PMIC_ST_SUCCESS)
    {
        Pmic_criticalSectionStart(pmicHandle);
        status = Pmic_ioTxByte(pmicHandle, PMIC_SCRATCH_PAD_REG_1_REGADDR + scratchPadRegNum, value);
        Pmic_criticalSectionStop(pmicHandle);
    }

    return status;
}

int32_t Pmic_getScratchPadVal(const Pmic_CoreHandle_t *pmicHandle, uint8_t scratchPadRegNum, uint8_t *value)
{
    uint8_t regData = 0U;
    int32_t status = Pmic_checkPmicCoreHandle(pmicHandle);

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
        Pmic_criticalSectionStart(pmicHandle);
        status = Pmic_ioRxByte(pmicHandle, PMIC_SCRATCH_PAD_REG_1_REGADDR + scratchPadRegNum, &regData);
        Pmic_criticalSectionStop(pmicHandle);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        *value = regData;
    }

    return status;
}

int32_t Pmic_getCRC8Enable(Pmic_CoreHandle_t *pmicHandle, bool *crcEnabled)
{
    uint8_t regData = 0U;
    int32_t status = Pmic_checkPmicCoreHandle(pmicHandle);

    if ((status == PMIC_ST_SUCCESS) && (crcEnabled == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    // Read INTERFACE_CONF register
    if (status == PMIC_ST_SUCCESS)
    {
        Pmic_criticalSectionStart(pmicHandle);
        status = Pmic_ioRxByte(pmicHandle, PMIC_INTERFACE_CONF_REGADDR, &regData);
        Pmic_criticalSectionStop(pmicHandle);
    }

    // Extract the CRC8 enable status (cast as boolean)
    if (status == PMIC_ST_SUCCESS)
    {
        *crcEnabled = Pmic_getBitField_b(regData, PMIC_I2C_CRC_EN_SHIFT);
        pmicHandle->crcEnable = *crcEnabled;
    }

    return status;
}

int32_t Pmic_enableDisableCRC8(Pmic_CoreHandle_t *pmicHandle, bool crc8Enable)
{
    uint8_t regData = 0U;
    int32_t status = Pmic_checkPmicCoreHandle(pmicHandle);

    if (status == PMIC_ST_SUCCESS)
    {
        // Read INTERFACE_CONF register
        Pmic_criticalSectionStart(pmicHandle);
        status = Pmic_ioRxByte(pmicHandle, PMIC_INTERFACE_CONF_REGADDR, &regData);
        Pmic_criticalSectionStop(pmicHandle);
    }

    // Modify I2C_CRC_EN bit then write new register value back to PMIC
    if (status == PMIC_ST_SUCCESS)
    {
        Pmic_setBitField_b(&regData, PMIC_I2C_CRC_EN_SHIFT, PMIC_I2C_CRC_EN_MASK, crc8Enable);

        Pmic_criticalSectionStart(pmicHandle);
        status = Pmic_ioTxByte(pmicHandle, PMIC_INTERFACE_CONF_REGADDR, regData);
        Pmic_criticalSectionStop(pmicHandle);
    }

    // Change crcEnable struct member of PMIC handle
    if (status == PMIC_ST_SUCCESS)
    {
        pmicHandle->crcEnable = crc8Enable;
    }

    return status;
}

int32_t Pmic_enableCRC8(Pmic_CoreHandle_t *pmicHandle)
{
    return Pmic_enableDisableCRC8(pmicHandle, PMIC_ENABLE);
}

int32_t Pmic_disableCRC8(Pmic_CoreHandle_t *pmicHandle)
{
    return Pmic_enableDisableCRC8(pmicHandle, PMIC_DISABLE);
}

int32_t Pmic_getRegLock(const Pmic_CoreHandle_t *pmicHandle, bool *regLockStat)
{
    uint8_t regData = 0U;
    int32_t status = Pmic_checkPmicCoreHandle(pmicHandle);

    if ((status == PMIC_ST_SUCCESS) && (regLockStat == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Read REGISTER_LOCK
        Pmic_criticalSectionStart(pmicHandle);
        status = Pmic_ioRxByte(pmicHandle, PMIC_REGISTER_LOCK_REGADDR, &regData);
        Pmic_criticalSectionStop(pmicHandle);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Extract REGISTER_LOCK_STATUS bit field
        *regLockStat = Pmic_getBitField_b(regData, PMIC_REGISTER_LOCK_STATUS_SHIFT);
    }

    return status;
}

int32_t Pmic_setRegLock(const Pmic_CoreHandle_t *pmicHandle, bool lock)
{
    const uint8_t key = lock ? PMIC_REG_LOCK : PMIC_REG_UNLOCK;
    int32_t status = Pmic_checkPmicCoreHandle(pmicHandle);

    // Write the key to REGISTER_LOCK
    if (status == PMIC_ST_SUCCESS)
    {
        Pmic_criticalSectionStart(pmicHandle);
        status = Pmic_ioTxByte(pmicHandle, PMIC_REGISTER_LOCK_REGADDR, key);
        Pmic_criticalSectionStop(pmicHandle);
    }

    return status;
}

int32_t Pmic_unlockRegs(const Pmic_CoreHandle_t *pmicHandle)
{
    return Pmic_setRegLock(pmicHandle, PMIC_UNLOCK);
}

int32_t Pmic_lockRegs(const Pmic_CoreHandle_t *pmicHandle)
{
    return Pmic_setRegLock(pmicHandle, PMIC_LOCK);
}
