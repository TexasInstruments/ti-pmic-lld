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

#include "core_test.h"
#include "test_constants.h"

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/* ========================================================================== */
/*                             Global Variables                               */
/* ========================================================================== */

static Pmic_Handle_t pmicHandle = {0U};

/* ========================================================================== */
/*                           Function Definitions                             */
/* ========================================================================== */

/* ========================================================================== */
/*                           Negative Test Functions                          */
/* ========================================================================== */

/**
 * @brief Test Pmic_setScratchPadValue with NULL handle
 */
void test_neg_core_setScratchPadValue_nullHandle(void)
{
    int32_t status = Pmic_setScratchPadValue(NULL, PMIC_SCRATCH_PAD_REG_1, TEST_PATTERN_AA);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_setScratchPadValue with invalid register number
 */
void test_neg_core_setScratchPadValue_invalidRegNum(void)
{
    int32_t status = Pmic_setScratchPadValue(&pmicHandle, PMIC_SCRATCH_PAD_REG_MAX + 1U, TEST_PATTERN_AA);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_getScratchPadValue with NULL handle
 */
void test_neg_core_getScratchPadValue_nullHandle(void)
{
    uint8_t value = 0U;
    int32_t status = Pmic_getScratchPadValue(NULL, PMIC_SCRATCH_PAD_REG_1, &value);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_getScratchPadValue with NULL value pointer
 */
void test_neg_core_getScratchPadValue_nullValue(void)
{
    int32_t status = Pmic_getScratchPadValue(&pmicHandle, PMIC_SCRATCH_PAD_REG_1, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_getScratchPadValue with invalid register number
 */
void test_neg_core_getScratchPadValue_invalidRegNum(void)
{
    uint8_t value = 0U;
    int32_t status = Pmic_getScratchPadValue(&pmicHandle, PMIC_SCRATCH_PAD_REG_MAX + 1U, &value);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_setRegLockState with NULL handle
 */
void test_neg_core_setRegLockState_nullHandle(void)
{
    int32_t status = Pmic_setRegLockState(NULL, PMIC_LOCK_DISABLE);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_getRegLockState with NULL handle
 */
void test_neg_core_getRegLockState_nullHandle(void)
{
    uint8_t lockState = 0U;
    int32_t status = Pmic_getRegLockState(NULL, &lockState);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_getRegLockState with NULL lockState pointer
 */
void test_neg_core_getRegLockState_nullLockState(void)
{
    int32_t status = Pmic_getRegLockState(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_setCntLockState with NULL handle
 */
void test_neg_core_setCntLockState_nullHandle(void)
{
    int32_t status = Pmic_setCntLockState(NULL, PMIC_LOCK_DISABLE);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_setCntLockState with invalid lock state parameter
 */
void test_neg_core_setCntLockState_invalidLockState(void)
{
    int32_t status = Pmic_setCntLockState(&pmicHandle, TEST_INVALID_PARAM_255);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_getCntLockState with NULL handle
 */
void test_neg_core_getCntLockState_nullHandle(void)
{
    uint8_t lockState = 0U;
    int32_t status = Pmic_getCntLockState(NULL, &lockState);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_getCntLockState with NULL lockState pointer
 */
void test_neg_core_getCntLockState_nullLockState(void)
{
    int32_t status = Pmic_getCntLockState(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_setLockCfg with NULL config pointer
 */
void test_neg_core_setLockCfg_nullConfig(void)
{
    int32_t status = Pmic_setLockCfg(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_setLockCfg with invalid validParams
 */
void test_neg_core_setLockCfg_invalidValidParams(void)
{
    Pmic_Lock_t lockCfg = {
        .validParams = 0U,
        .cfgLock = false,
        .cntLock = false
    };
    int32_t status = Pmic_setLockCfg(&pmicHandle, &lockCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_getLockCfg with NULL handle
 */
void test_neg_core_getLockCfg_nullHandle(void)
{
    Pmic_Lock_t lockCfg = { .validParams = PMIC_CFG_CORE_LOCK_REG_VALID };
    int32_t status = Pmic_getLockCfg(NULL, &lockCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_getLockCfg with NULL config pointer
 */
void test_neg_core_getLockCfg_nullConfig(void)
{
    int32_t status = Pmic_getLockCfg(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_getNvmRev with NULL handle
 */
void test_neg_core_getNvmRev_nullHandle(void)
{
    uint8_t nvmRev = 0U;
    int32_t status = Pmic_getNvmRev(NULL, &nvmRev);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_getNvmRev with NULL nvmRev pointer
 */
void test_neg_core_getNvmRev_nullNvmRev(void)
{
    int32_t status = Pmic_getNvmRev(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_getSiliconRev with NULL handle
 */
void test_neg_core_getSiliconRev_nullHandle(void)
{
    uint8_t siliconRev = 0U;
    int32_t status = Pmic_getSiliconRev(NULL, &siliconRev);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_getSiliconRev with NULL siliconRev pointer
 */
void test_neg_core_getSiliconRev_nullSiliconRev(void)
{
    int32_t status = Pmic_getSiliconRev(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_setMuxCfg with NULL handle
 */
void test_neg_core_diagSetOutCtrlCfg_nullHandle(void)
{
    Pmic_MuxCfg_t config = {
        .validParams = PMIC_CFG_CORE_MUX_AMUX_EN_VALID,
        .amuxEnable = true
    };
    int32_t status = Pmic_setMuxCfg(NULL, &config);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_getMuxCfg with NULL handle
 */
void test_neg_core_diagGetOutCtrlCfg_nullHandle(void)
{
    Pmic_MuxCfg_t config = {0U};
    int32_t status = Pmic_getMuxCfg(NULL, &config);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_getMuxCfg with NULL config pointer
 */
void test_neg_core_diagGetOutCtrlCfg_nullConfig(void)
{
    int32_t status = Pmic_getMuxCfg(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_setMuxCfg with NULL handle (AMUX channel)
 */
void test_neg_core_diagSetAmuxCfg_nullHandle(void)
{
    Pmic_MuxCfg_t config = {
        .validParams = PMIC_CFG_CORE_MUX_AMUX_CHANNEL_VALID,
        .amuxChannel = 0U
    };
    int32_t status = Pmic_setMuxCfg(NULL, &config);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_setMuxCfg with invalid channel (out of range)
 */
void test_neg_core_diagSetAmuxCfg_invalidChannel(void)
{
    Pmic_MuxCfg_t config = {
        .validParams = PMIC_CFG_CORE_MUX_AMUX_CHANNEL_VALID,
        .amuxChannel = 0x20U
    };
    int32_t status = Pmic_setMuxCfg(&pmicHandle, &config);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_getMuxCfg with NULL handle (AMUX channel)
 */
void test_neg_core_diagGetAmuxCfg_nullHandle(void)
{
    Pmic_MuxCfg_t config = {
        .validParams = PMIC_CFG_CORE_MUX_AMUX_CHANNEL_VALID
    };
    int32_t status = Pmic_getMuxCfg(NULL, &config);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_getMuxCfg with NULL config pointer (AMUX channel)
 */
void test_neg_core_diagGetAmuxCfg_nullChannel(void)
{
    int32_t status = Pmic_getMuxCfg(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_setMuxCfg with NULL handle (DMUX group)
 */
void test_neg_core_diagSetDmuxCfg_nullHandle(void)
{
    Pmic_MuxCfg_t config = {
        .validParams = PMIC_CFG_CORE_MUX_DMUX_GROUP_VALID,
        .dmuxGroup = 0U
    };
    int32_t status = Pmic_setMuxCfg(NULL, &config);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_setMuxCfg with invalid group (out of range)
 */
void test_neg_core_diagSetDmuxCfg_invalidGroup(void)
{
    Pmic_MuxCfg_t config = {
        .validParams = PMIC_CFG_CORE_MUX_DMUX_GROUP_VALID,
        .dmuxGroup = 0x20U
    };
    int32_t status = Pmic_setMuxCfg(&pmicHandle, &config);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_getMuxCfg with NULL handle (DMUX group)
 */
void test_neg_core_diagGetDmuxCfg_nullHandle(void)
{
    Pmic_MuxCfg_t config = {
        .validParams = PMIC_CFG_CORE_MUX_DMUX_GROUP_VALID
    };
    int32_t status = Pmic_getMuxCfg(NULL, &config);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_getMuxCfg with NULL config pointer (DMUX group)
 */
void test_neg_core_diagGetDmuxCfg_nullGroup(void)
{
    int32_t status = Pmic_getMuxCfg(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_core_configCrcEnable_nullHandle(void)
{
    int32_t status = Pmic_configCrcEnable(NULL, PMIC_CFG_CRC_ENABLE_ONLY);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_core_configCrcDisable_nullHandle(void)
{
    int32_t status = Pmic_configCrcDisable(NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_core_getConfigCrcEnableState_nullHandle(void)
{
    bool isEnabled = false;
    int32_t status = Pmic_getConfigCrcEnableState(NULL, &isEnabled);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_core_getConfigCrcEnableState_nullIsEnabled(void)
{
    int32_t status = Pmic_getConfigCrcEnableState(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_core_getConfigCrcStatus_nullHandle(void)
{
    Pmic_ConfigCrcStat_t stat = { .validParams = PMIC_CONFIG_CRC_STAT_CALC_DONE_VALID };
    int32_t status = Pmic_getConfigCrcStatus(NULL, &stat);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_core_getConfigCrcStatus_nullStatus(void)
{
    int32_t status = Pmic_getConfigCrcStatus(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_core_getConfigCrcStatus_zeroValidParams(void)
{
    Pmic_ConfigCrcStat_t stat = { .validParams = 0U };
    int32_t status = Pmic_getConfigCrcStatus(&pmicHandle, &stat);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_core_clrConfigCrcStatus_nullHandle(void)
{
    Pmic_ConfigCrcStat_t stat = { .validParams = PMIC_CONFIG_CRC_STAT_CALC_DONE_VALID };
    int32_t status = Pmic_clrConfigCrcStatus(NULL, &stat);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_core_clrConfigCrcStatus_nullStatus(void)
{
    int32_t status = Pmic_clrConfigCrcStatus(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_core_clrConfigCrcStatus_zeroValidParams(void)
{
    Pmic_ConfigCrcStat_t stat = { .validParams = 0U };
    int32_t status = Pmic_clrConfigCrcStatus(&pmicHandle, &stat);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_core_setConfigCrcVal_nullHandle(void)
{
    int32_t status = Pmic_setConfigCrc(NULL, 0xA55AU);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_core_getConfigCrcVal_nullHandle(void)
{
    uint16_t value = 0U;
    int32_t status = Pmic_getConfigCrc(NULL, &value);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_core_getConfigCrcVal_nullValue(void)
{
    int32_t status = Pmic_getConfigCrc(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_core_configCrcCalculate_nullHandle(void)
{
    int32_t status = Pmic_configCrcCalculate(NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/* ========================================================================== */
/*                           Positive Test Functions                          */
/* ========================================================================== */

/**
 * @brief Test scratchpad register set and get operations
 */
void test_pos_core_scratchPad_setGet(void)
{
    int32_t status;
    uint8_t writeData1 = 0x38U;
    uint8_t writeData2 = 0x69U;
    uint8_t readData1 = 0U;
    uint8_t readData2 = 0U;

    /* Write to scratchpad registers */
    status = Pmic_setScratchPadValue(&pmicHandle, PMIC_SCRATCH_PAD_REG_1, writeData1);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_setScratchPadValue(&pmicHandle, PMIC_SCRATCH_PAD_REG_2, writeData2);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Read from scratchpad registers */
    status = Pmic_getScratchPadValue(&pmicHandle, PMIC_SCRATCH_PAD_REG_1, &readData1);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_getScratchPadValue(&pmicHandle, PMIC_SCRATCH_PAD_REG_2, &readData2);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Verify read data matches written data */
    PLATFORM_ASSERT(readData1 == writeData1);
    PLATFORM_ASSERT(readData2 == writeData2);
}

/**
 * @brief Test register lock control set and get operations
 */
void test_pos_core_regLock_setGet(void)
{
    int32_t status;
    uint8_t lockState = 0U;

    /* Enable register lock */
    status = Pmic_setRegLockState(&pmicHandle, PMIC_LOCK_ENABLE);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Verify lock is enabled */
    status = Pmic_getRegLockState(&pmicHandle, &lockState);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(lockState == PMIC_LOCK_ENABLE);

    /* Disable register lock */
    status = Pmic_setRegLockState(&pmicHandle, PMIC_LOCK_DISABLE);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Verify lock is disabled */
    status = Pmic_getRegLockState(&pmicHandle, &lockState);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(lockState == PMIC_LOCK_DISABLE);
}

/**
 * @brief Test counter lock control set and get operations
 */
void test_pos_core_cntLock_setGet(void)
{
    int32_t status;
    uint8_t lockState = 0U;

    /* Enable counter lock */
    status = Pmic_setCntLockState(&pmicHandle, PMIC_LOCK_ENABLE);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Verify lock is enabled */
    status = Pmic_getCntLockState(&pmicHandle, &lockState);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(lockState == PMIC_LOCK_ENABLE);

    /* Disable counter lock */
    status = Pmic_setCntLockState(&pmicHandle, PMIC_LOCK_DISABLE);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Verify lock is disabled */
    status = Pmic_getCntLockState(&pmicHandle, &lockState);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(lockState == PMIC_LOCK_DISABLE);
}

/**
 * @brief Test lock configuration set and get operations
 */
void test_pos_core_lockCfg_setGet(void)
{
    int32_t status;
    Pmic_Lock_t lockCfg = {0U};
    Pmic_Lock_t readCfg = {0U};

    /* Enable both locks using setLockCfg */
    lockCfg.validParams = PMIC_CFG_CORE_LOCK_REG_VALID | PMIC_CFG_CORE_LOCK_CNT_VALID;
    lockCfg.cfgLock = true;
    lockCfg.cntLock = true;
    status = Pmic_setLockCfg(&pmicHandle, &lockCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Read back lock configuration */
    readCfg.validParams = PMIC_CFG_CORE_LOCK_REG_VALID | PMIC_CFG_CORE_LOCK_CNT_VALID;
    status = Pmic_getLockCfg(&pmicHandle, &readCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(readCfg.cfgLock == true);
    PLATFORM_ASSERT(readCfg.cntLock == true);

    /* Disable both locks */
    lockCfg.cfgLock = false;
    lockCfg.cntLock = false;
    status = Pmic_setLockCfg(&pmicHandle, &lockCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Verify both locks are disabled */
    status = Pmic_getLockCfg(&pmicHandle, &readCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(readCfg.cfgLock == false);
    PLATFORM_ASSERT(readCfg.cntLock == false);
}

/**
 * @brief Test device ID and revision read operations
 */
void test_pos_core_deviceId_revision(void)
{
    int32_t status;
    uint8_t nvmRev = 0U;
    uint8_t siliconRev = 0U;

    /* Read NVM revision */
    status = Pmic_getNvmRev(&pmicHandle, &nvmRev);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Read silicon revision */
    status = Pmic_getSiliconRev(&pmicHandle, &siliconRev);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

/**
 * @brief Test multiplexer output control set and get operations
 */
void test_pos_core_diagOutCtrl_setGet(void)
{
    int32_t status;
    Pmic_MuxCfg_t setCfg = {0U};
    Pmic_MuxCfg_t getCfg = {0U};

    /* Enable AMUX */
    setCfg.validParams = PMIC_CFG_CORE_MUX_AMUX_EN_VALID;
    setCfg.amuxEnable = true;
    status = Pmic_setMuxCfg(&pmicHandle, &setCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Verify AMUX is enabled */
    getCfg.validParams = PMIC_CFG_CORE_MUX_AMUX_EN_VALID;
    status = Pmic_getMuxCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(getCfg.amuxEnable == true);

    /* Enable DMUX */
    setCfg.validParams = PMIC_CFG_CORE_MUX_DMUX_EN_VALID;
    setCfg.amuxEnable = false;
    setCfg.dmuxEnable = true;
    status = Pmic_setMuxCfg(&pmicHandle, &setCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Verify DMUX is enabled */
    getCfg.validParams = PMIC_CFG_CORE_MUX_DMUX_EN_VALID;
    status = Pmic_getMuxCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(getCfg.dmuxEnable == true);

    /* Disable multiplexer output */
    setCfg.validParams = PMIC_CFG_CORE_MUX_MODE_VALID;
    setCfg.muxMode = PMIC_MUX_MODE_DISABLED;
    status = Pmic_setMuxCfg(&pmicHandle, &setCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Verify multiplexer output is disabled */
    getCfg.validParams = PMIC_CFG_CORE_MUX_MODE_VALID;
    status = Pmic_getMuxCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(getCfg.muxMode == PMIC_MUX_MODE_DISABLED);
}

/**
 * @brief Test AMUX channel configuration set and get operations
 */
void test_pos_core_diagAMUX_setGet(void)
{
    int32_t status;
    Pmic_MuxCfg_t setCfg = {0U};
    Pmic_MuxCfg_t getCfg = {0U};

    /* Set AMUX channel */
    setCfg.validParams = PMIC_CFG_CORE_MUX_AMUX_CHANNEL_VALID;
    setCfg.amuxChannel = 0x0AU;
    status = Pmic_setMuxCfg(&pmicHandle, &setCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Verify AMUX channel */
    getCfg.validParams = PMIC_CFG_CORE_MUX_AMUX_CHANNEL_VALID;
    status = Pmic_getMuxCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(getCfg.amuxChannel == 0x0AU);

    /* Test boundary channel (max valid) */
    setCfg.validParams = PMIC_CFG_CORE_MUX_AMUX_CHANNEL_VALID;
    setCfg.amuxChannel = 0x1FU;
    status = Pmic_setMuxCfg(&pmicHandle, &setCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    getCfg.validParams = PMIC_CFG_CORE_MUX_AMUX_CHANNEL_VALID;
    status = Pmic_getMuxCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(getCfg.amuxChannel == 0x1FU);

    /* Reset to channel 0 */
    setCfg.validParams = PMIC_CFG_CORE_MUX_AMUX_CHANNEL_VALID;
    setCfg.amuxChannel = 0U;
    status = Pmic_setMuxCfg(&pmicHandle, &setCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

/**
 * @brief Test DMUX group configuration set and get operations
 */
void test_pos_core_diagDMUX_setGet(void)
{
    int32_t status;
    Pmic_MuxCfg_t setCfg = {0U};
    Pmic_MuxCfg_t getCfg = {0U};

    /* Set DMUX group */
    setCfg.validParams = PMIC_CFG_CORE_MUX_DMUX_GROUP_VALID;
    setCfg.dmuxGroup = 0x05U;
    status = Pmic_setMuxCfg(&pmicHandle, &setCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Verify DMUX group */
    getCfg.validParams = PMIC_CFG_CORE_MUX_DMUX_GROUP_VALID;
    status = Pmic_getMuxCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(getCfg.dmuxGroup == 0x05U);

    /* Test boundary group (max valid) */
    setCfg.validParams = PMIC_CFG_CORE_MUX_DMUX_GROUP_VALID;
    setCfg.dmuxGroup = 0x1FU;
    status = Pmic_setMuxCfg(&pmicHandle, &setCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    getCfg.validParams = PMIC_CFG_CORE_MUX_DMUX_GROUP_VALID;
    status = Pmic_getMuxCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(getCfg.dmuxGroup == 0x1FU);

    /* Reset to group 0 */
    setCfg.validParams = PMIC_CFG_CORE_MUX_DMUX_GROUP_VALID;
    setCfg.dmuxGroup = 0U;
    status = Pmic_setMuxCfg(&pmicHandle, &setCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_core_configCrcEnable_enableOnly(void)
{
#ifdef BUILD_MOCK
    int32_t status;
    bool isEnabled = false;

    status = Pmic_configCrcDisable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_configCrcEnable(&pmicHandle, PMIC_CFG_CRC_ENABLE_ONLY);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_getConfigCrcEnableState(&pmicHandle, &isEnabled);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(isEnabled == true);

    (void)Pmic_configCrcDisable(&pmicHandle);
#else
    TEST_IGNORE_MESSAGE("Requires BUILD_MOCK for config register CRC");
#endif
}

void test_pos_core_configCrcEnable_recalculate(void)
{
#ifdef BUILD_MOCK
    int32_t status;
    bool isEnabled = false;

    status = Pmic_configCrcDisable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_configCrcEnable(&pmicHandle, PMIC_CFG_CRC_RECALCULATE);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_getConfigCrcEnableState(&pmicHandle, &isEnabled);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(isEnabled == true);

    (void)Pmic_configCrcDisable(&pmicHandle);
#else
    TEST_IGNORE_MESSAGE("Requires BUILD_MOCK for config register CRC");
#endif
}

void test_pos_core_configCrcDisable_disable(void)
{
#ifdef BUILD_MOCK
    int32_t status;
    bool isEnabled = true;

    status = Pmic_configCrcEnable(&pmicHandle, PMIC_CFG_CRC_ENABLE_ONLY);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_configCrcDisable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_getConfigCrcEnableState(&pmicHandle, &isEnabled);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(isEnabled == false);
#else
    TEST_IGNORE_MESSAGE("Requires BUILD_MOCK for config register CRC");
#endif
}

void test_pos_core_getConfigCrcEnableState_enabled(void)
{
#ifdef BUILD_MOCK
    int32_t status;
    bool isEnabled = false;

    status = Pmic_configCrcEnable(&pmicHandle, PMIC_CFG_CRC_ENABLE_ONLY);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_getConfigCrcEnableState(&pmicHandle, &isEnabled);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(isEnabled == true);

    (void)Pmic_configCrcDisable(&pmicHandle);
#else
    TEST_IGNORE_MESSAGE("Requires BUILD_MOCK for config register CRC");
#endif
}

void test_pos_core_getConfigCrcEnableState_disabled(void)
{
#ifdef BUILD_MOCK
    int32_t status;
    bool isEnabled = true;

    status = Pmic_configCrcDisable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_getConfigCrcEnableState(&pmicHandle, &isEnabled);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(isEnabled == false);
#else
    TEST_IGNORE_MESSAGE("Requires BUILD_MOCK for config register CRC");
#endif
}

void test_pos_core_getConfigCrcStatus_calcDone(void)
{
#ifdef BUILD_MOCK
    Pmic_ConfigCrcStat_t stat = { .validParams = PMIC_CONFIG_CRC_STAT_CALC_DONE_VALID };
    int32_t status = Pmic_getConfigCrcStatus(&pmicHandle, &stat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Requires BUILD_MOCK for config register CRC");
#endif
}

void test_pos_core_getConfigCrcStatus_error(void)
{
#ifdef BUILD_MOCK
    Pmic_ConfigCrcStat_t stat = { .validParams = PMIC_CONFIG_CRC_STAT_ERROR_VALID };
    int32_t status = Pmic_getConfigCrcStatus(&pmicHandle, &stat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Requires BUILD_MOCK for config register CRC");
#endif
}

void test_pos_core_clrConfigCrcStatus_clearCalcDone(void)
{
#ifdef BUILD_MOCK
    Pmic_ConfigCrcStat_t stat = {
        .validParams = PMIC_CONFIG_CRC_STAT_CALC_DONE_VALID,
        .calcDone = true
    };
    int32_t status = Pmic_clrConfigCrcStatus(&pmicHandle, &stat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Requires BUILD_MOCK for config register CRC");
#endif
}

void test_pos_core_clrConfigCrcStatus_clearError(void)
{
#ifdef BUILD_MOCK
    Pmic_ConfigCrcStat_t stat = {
        .validParams = PMIC_CONFIG_CRC_STAT_ERROR_VALID,
        .error = true
    };
    int32_t status = Pmic_clrConfigCrcStatus(&pmicHandle, &stat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Requires BUILD_MOCK for config register CRC");
#endif
}

void test_pos_core_setConfigCrcVal_writeAndVerify(void)
{
#ifdef BUILD_MOCK
    int32_t status;
    uint16_t readBack = 0U;

    status = Pmic_setConfigCrc(&pmicHandle, 0xA55AU);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_getConfigCrc(&pmicHandle, &readBack);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(readBack == 0xA55AU);
#else
    TEST_IGNORE_MESSAGE("Requires BUILD_MOCK for config register CRC");
#endif
}

void test_pos_core_getConfigCrcVal_readValue(void)
{
#ifdef BUILD_MOCK
    int32_t status;
    uint16_t value = 0U;

    status = Pmic_setConfigCrc(&pmicHandle, 0x5AA5U);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_getConfigCrc(&pmicHandle, &value);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(value == 0x5AA5U);
#else
    TEST_IGNORE_MESSAGE("Requires BUILD_MOCK for config register CRC");
#endif
}

void test_pos_core_configCrcCalculate_calculate(void)
{
#ifdef BUILD_MOCK
    int32_t status = Pmic_configCrcCalculate(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Requires BUILD_MOCK for config register CRC");
#endif
}

/**
 * @brief Core test suite entry point
 * @param args Test arguments (unused)
 */
void core_test(void *args)
{
    char msg[50U] = {0};
    int32_t status = PMIC_ST_SUCCESS;

    platform_init();
    testTimer_startModule("Core");

    Pmic_HandleCfg_t pmicCfg = {
        .validParams = (PMIC_CFG_INIT_COMM_MODE_VALID |
                        PMIC_CFG_INIT_COMM_HANDLE_0_VALID |
                        PMIC_CFG_INIT_IO_READ_VALID |
                        PMIC_CFG_INIT_IO_WRITE_VALID |
                        PMIC_CFG_INIT_CRITICAL_SECTION_START_VALID |
                        PMIC_CFG_INIT_CRITICAL_SECTION_STOP_VALID),
        .commMode = PMIC_INTF_SPI,
        .commHandle0 = platform_getCommHandle(),
        .ioRead = &platform_rxByte,
        .ioWrite = &platform_txByte,
        .criticalSectionStart = &platform_critSecStart,
        .criticalSectionStop = &platform_critSecStop
    };

    status = Pmic_init(&pmicHandle, &pmicCfg);

    if (status == PMIC_ST_SUCCESS)
    {
        testUtils_printSiRev(&pmicHandle);

        platform_setupTests();
        CORE_TEST_RUN_ALL();
        platform_tearDownTests();
    }
    else
    {
        (void)sprintf(msg, "Error in initializing PMIC LLD: %d\r\n", status);
        platform_printString(msg);
    }

    testTimer_endModule();
    (void)Pmic_deinit(&pmicHandle);
    platform_deinit();
}
