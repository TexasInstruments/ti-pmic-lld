/******************************************************************************
 * Copyright (c) 2025 Texas Instruments Incorporated - http://www.ti.com
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

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/* Run all Core tests */
#define CORE_TEST_RUN_ALL() \
    PLATFORM_RUN_TEST(test_negative_Pmic_setScratchPadValue_nullParam_handle); \
    PLATFORM_RUN_TEST(test_negative_Pmic_setScratchPadValue_invalidParam_regNum); \
    PLATFORM_RUN_TEST(test_negative_Pmic_getScratchPadValue_nullParam_handle); \
    PLATFORM_RUN_TEST(test_negative_Pmic_getScratchPadValue_nullParam_value); \
    PLATFORM_RUN_TEST(test_negative_Pmic_getScratchPadValue_invalidParam_regNum); \
    PLATFORM_RUN_TEST(test_negative_Pmic_setRegLockState_nullParam_handle); \
    PLATFORM_RUN_TEST(test_negative_Pmic_getRegLockState_nullParam_handle); \
    PLATFORM_RUN_TEST(test_negative_Pmic_getRegLockState_nullParam_lockState); \
    PLATFORM_RUN_TEST(test_negative_Pmic_setCntLockState_nullParam_handle); \
    PLATFORM_RUN_TEST(test_negative_Pmic_setCntLockState_invalidParam_lockState); \
    PLATFORM_RUN_TEST(test_negative_Pmic_getCntLockState_nullParam_handle); \
    PLATFORM_RUN_TEST(test_negative_Pmic_getCntLockState_nullParam_lockState); \
    PLATFORM_RUN_TEST(test_negative_Pmic_setLockCfg_nullParam_config); \
    PLATFORM_RUN_TEST(test_negative_Pmic_setLockCfg_invalidParam_validParams); \
    PLATFORM_RUN_TEST(test_negative_Pmic_getLockCfg_nullParam_handle); \
    PLATFORM_RUN_TEST(test_negative_Pmic_getLockCfg_nullParam_config); \
    PLATFORM_RUN_TEST(test_negative_Pmic_getNvmRev_nullParam_handle); \
    PLATFORM_RUN_TEST(test_negative_Pmic_getNvmRev_nullParam_nvmRev); \
    PLATFORM_RUN_TEST(test_negative_Pmic_getSiliconRev_nullParam_handle); \
    PLATFORM_RUN_TEST(test_negative_Pmic_getSiliconRev_nullParam_siliconRev); \
    PLATFORM_RUN_TEST(test_negative_Pmic_getCommonStat_nullParam_handle); \
    PLATFORM_RUN_TEST(test_negative_Pmic_getCommonStat_nullParam_stat); \
    PLATFORM_RUN_TEST(test_negative_Pmic_diagSetOutCtrlCfg_nullParam_handle); \
    PLATFORM_RUN_TEST(test_negative_Pmic_diagGetOutCtrlCfg_nullParam_handle); \
    PLATFORM_RUN_TEST(test_negative_Pmic_diagGetOutCtrlCfg_nullParam_config); \
    PLATFORM_RUN_TEST(test_negative_Pmic_diagSetAmuxCfg_nullParam_handle); \
    PLATFORM_RUN_TEST(test_negative_Pmic_diagSetAmuxCfg_invalidParam_channel); \
    PLATFORM_RUN_TEST(test_negative_Pmic_diagGetAmuxCfg_nullParam_handle); \
    PLATFORM_RUN_TEST(test_negative_Pmic_diagGetAmuxCfg_nullParam_channel); \
    PLATFORM_RUN_TEST(test_negative_Pmic_diagSetDmuxCfg_nullParam_handle); \
    PLATFORM_RUN_TEST(test_negative_Pmic_diagSetDmuxCfg_invalidParam_group); \
    PLATFORM_RUN_TEST(test_negative_Pmic_diagGetDmuxCfg_nullParam_handle); \
    PLATFORM_RUN_TEST(test_negative_Pmic_diagGetDmuxCfg_nullParam_group); \
    PLATFORM_RUN_TEST(test_positive_scratchPadSetGet); \
    PLATFORM_RUN_TEST(test_positive_regLockSetGet); \
    PLATFORM_RUN_TEST(test_positive_cntLockSetGet); \
    PLATFORM_RUN_TEST(test_positive_lockCfgSetGet); \
    PLATFORM_RUN_TEST(test_positive_deviceIdRevision); \
    PLATFORM_RUN_TEST(test_positive_commonStat); \
    PLATFORM_RUN_TEST(test_positive_diagOutCtrlSetGet); \
    PLATFORM_RUN_TEST(test_positive_diagAMUXSetGet); \
    PLATFORM_RUN_TEST(test_positive_diagDMUXSetGet)

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
void test_negative_Pmic_setScratchPadValue_nullParam_handle(void)
{
    int32_t status = Pmic_setScratchPadValue(NULL, PMIC_SCRATCH_PAD_REG_1, 0xAAU);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_setScratchPadValue with invalid register number
 */
void test_negative_Pmic_setScratchPadValue_invalidParam_regNum(void)
{
    int32_t status = Pmic_setScratchPadValue(&pmicHandle, PMIC_SCRATCH_PAD_REG_MAX + 1U, 0xAAU);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_getScratchPadValue with NULL handle
 */
void test_negative_Pmic_getScratchPadValue_nullParam_handle(void)
{
    uint8_t value = 0U;
    int32_t status = Pmic_getScratchPadValue(NULL, PMIC_SCRATCH_PAD_REG_1, &value);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_getScratchPadValue with NULL value pointer
 */
void test_negative_Pmic_getScratchPadValue_nullParam_value(void)
{
    int32_t status = Pmic_getScratchPadValue(&pmicHandle, PMIC_SCRATCH_PAD_REG_1, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_getScratchPadValue with invalid register number
 */
void test_negative_Pmic_getScratchPadValue_invalidParam_regNum(void)
{
    uint8_t value = 0U;
    int32_t status = Pmic_getScratchPadValue(&pmicHandle, PMIC_SCRATCH_PAD_REG_MAX + 1U, &value);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_setRegLockState with NULL handle
 */
void test_negative_Pmic_setRegLockState_nullParam_handle(void)
{
    int32_t status = Pmic_setRegLockState(NULL, PMIC_LOCK_DISABLE);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_getRegLockState with NULL handle
 */
void test_negative_Pmic_getRegLockState_nullParam_handle(void)
{
    uint8_t lockState = 0U;
    int32_t status = Pmic_getRegLockState(NULL, &lockState);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_getRegLockState with NULL lockState pointer
 */
void test_negative_Pmic_getRegLockState_nullParam_lockState(void)
{
    int32_t status = Pmic_getRegLockState(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_setCntLockState with NULL handle
 */
void test_negative_Pmic_setCntLockState_nullParam_handle(void)
{
    int32_t status = Pmic_setCntLockState(NULL, PMIC_LOCK_DISABLE);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_setCntLockState with invalid lock state parameter
 */
void test_negative_Pmic_setCntLockState_invalidParam_lockState(void)
{
    int32_t status = Pmic_setCntLockState(&pmicHandle, 0xFFU);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_getCntLockState with NULL handle
 */
void test_negative_Pmic_getCntLockState_nullParam_handle(void)
{
    uint8_t lockState = 0U;
    int32_t status = Pmic_getCntLockState(NULL, &lockState);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_getCntLockState with NULL lockState pointer
 */
void test_negative_Pmic_getCntLockState_nullParam_lockState(void)
{
    int32_t status = Pmic_getCntLockState(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_setLockCfg with NULL config pointer
 */
void test_negative_Pmic_setLockCfg_nullParam_config(void)
{
    int32_t status = Pmic_setLockCfg(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_setLockCfg with invalid validParams
 */
void test_negative_Pmic_setLockCfg_invalidParam_validParams(void)
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
void test_negative_Pmic_getLockCfg_nullParam_handle(void)
{
    Pmic_Lock_t lockCfg = { .validParams = PMIC_CFG_REG_LOCK_VALID };
    int32_t status = Pmic_getLockCfg(NULL, &lockCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_getLockCfg with NULL config pointer
 */
void test_negative_Pmic_getLockCfg_nullParam_config(void)
{
    int32_t status = Pmic_getLockCfg(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_getNvmRev with NULL handle
 */
void test_negative_Pmic_getNvmRev_nullParam_handle(void)
{
    uint8_t nvmRev = 0U;
    int32_t status = Pmic_getNvmRev(NULL, &nvmRev);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_getNvmRev with NULL nvmRev pointer
 */
void test_negative_Pmic_getNvmRev_nullParam_nvmRev(void)
{
    int32_t status = Pmic_getNvmRev(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_getSiliconRev with NULL handle
 */
void test_negative_Pmic_getSiliconRev_nullParam_handle(void)
{
    uint8_t siliconRev = 0U;
    int32_t status = Pmic_getSiliconRev(NULL, &siliconRev);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_getSiliconRev with NULL siliconRev pointer
 */
void test_negative_Pmic_getSiliconRev_nullParam_siliconRev(void)
{
    int32_t status = Pmic_getSiliconRev(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_getCommonStat with NULL handle
 */
void test_negative_Pmic_getCommonStat_nullParam_handle(void)
{
    Pmic_CommonCtrlStat_t stat = {0U};
    int32_t status = Pmic_getCommonStat(NULL, &stat);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_getCommonStat with NULL stat pointer
 */
void test_negative_Pmic_getCommonStat_nullParam_stat(void)
{
    int32_t status = Pmic_getCommonStat(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_diagSetOutCtrlCfg with NULL handle
 */
void test_negative_Pmic_diagSetOutCtrlCfg_nullParam_handle(void)
{
    Pmic_DiagOutCfgCtrl_t config = {
        .validParams = PMIC_DIAG_OUT_CTRL_AMUX_EN_VALID,
        .diagOutCtrl_AMUXEn = 1U
    };
    int32_t status = Pmic_diagSetOutCtrlCfg(NULL, &config);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_diagGetOutCtrlCfg with NULL handle
 */
void test_negative_Pmic_diagGetOutCtrlCfg_nullParam_handle(void)
{
    Pmic_DiagOutCfgCtrl_t config = {0U};
    int32_t status = Pmic_diagGetOutCtrlCfg(NULL, &config);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_diagGetOutCtrlCfg with NULL config pointer
 */
void test_negative_Pmic_diagGetOutCtrlCfg_nullParam_config(void)
{
    int32_t status = Pmic_diagGetOutCtrlCfg(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_diagSetAmuxCfg with NULL handle
 */
void test_negative_Pmic_diagSetAmuxCfg_nullParam_handle(void)
{
    int32_t status = Pmic_diagSetAmuxCfg(NULL, 0U);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_diagSetAmuxCfg with invalid channel (out of range)
 */
void test_negative_Pmic_diagSetAmuxCfg_invalidParam_channel(void)
{
    int32_t status = Pmic_diagSetAmuxCfg(&pmicHandle, 0x20U);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_diagGetAmuxCfg with NULL handle
 */
void test_negative_Pmic_diagGetAmuxCfg_nullParam_handle(void)
{
    uint8_t channel = 0U;
    int32_t status = Pmic_diagGetAmuxCfg(NULL, &channel);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_diagGetAmuxCfg with NULL channel pointer
 */
void test_negative_Pmic_diagGetAmuxCfg_nullParam_channel(void)
{
    int32_t status = Pmic_diagGetAmuxCfg(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_diagSetDmuxCfg with NULL handle
 */
void test_negative_Pmic_diagSetDmuxCfg_nullParam_handle(void)
{
    int32_t status = Pmic_diagSetDmuxCfg(NULL, 0U);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_diagSetDmuxCfg with invalid group (out of range)
 */
void test_negative_Pmic_diagSetDmuxCfg_invalidParam_group(void)
{
    int32_t status = Pmic_diagSetDmuxCfg(&pmicHandle, 0x20U);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_diagGetDmuxCfg with NULL handle
 */
void test_negative_Pmic_diagGetDmuxCfg_nullParam_handle(void)
{
    uint8_t group = 0U;
    int32_t status = Pmic_diagGetDmuxCfg(NULL, &group);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_diagGetDmuxCfg with NULL group pointer
 */
void test_negative_Pmic_diagGetDmuxCfg_nullParam_group(void)
{
    int32_t status = Pmic_diagGetDmuxCfg(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/* ========================================================================== */
/*                           Positive Test Functions                          */
/* ========================================================================== */

/**
 * @brief Test scratchpad register set and get operations
 */
void test_positive_scratchPadSetGet(void)
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
void test_positive_regLockSetGet(void)
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
void test_positive_cntLockSetGet(void)
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
void test_positive_lockCfgSetGet(void)
{
    int32_t status;
    Pmic_Lock_t lockCfg = {0U};
    Pmic_Lock_t readCfg = {0U};

    /* Enable both locks using setLockCfg */
    lockCfg.validParams = PMIC_CFG_REG_LOCK_VALID | PMIC_CFG_CNT_LOCK_VALID;
    lockCfg.cfgLock = true;
    lockCfg.cntLock = true;
    status = Pmic_setLockCfg(&pmicHandle, &lockCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Read back lock configuration */
    readCfg.validParams = PMIC_CFG_REG_LOCK_VALID | PMIC_CFG_CNT_LOCK_VALID;
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
void test_positive_deviceIdRevision(void)
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
 * @brief Test common status read operations
 */
void test_positive_commonStat(void)
{
    int32_t status;
    Pmic_CommonCtrlStat_t stat = {0U};

    /* Read common status */
    status = Pmic_getCommonStat(&pmicHandle, &stat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

/**
 * @brief Test diagnostic output control set and get operations
 */
void test_positive_diagOutCtrlSetGet(void)
{
    int32_t status;
    Pmic_DiagOutCfgCtrl_t setCfg = {0U};
    Pmic_DiagOutCfgCtrl_t getCfg = {0U};

    /* Enable AMUX */
    setCfg.validParams = PMIC_DIAG_OUT_CTRL_AMUX_EN_VALID;
    setCfg.diagOutCtrl_AMUXEn = 1U;
    status = Pmic_diagSetOutCtrlCfg(&pmicHandle, &setCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Verify AMUX is enabled */
    status = Pmic_diagGetOutCtrlCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(getCfg.diagOutCtrl_AMUXEn == 1U);

    /* Enable DMUX */
    setCfg.validParams = PMIC_DIAG_OUT_CTRL_DMUX_EN_VALID;
    setCfg.diagOutCtrl_AMUXEn = 0U;
    setCfg.diagOutCtrl_DMUXEn = 1U;
    status = Pmic_diagSetOutCtrlCfg(&pmicHandle, &setCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Verify DMUX is enabled */
    status = Pmic_diagGetOutCtrlCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(getCfg.diagOutCtrl_DMUXEn == 1U);

    /* Disable diagnostic output */
    setCfg.validParams = PMIC_DIAG_OUT_CTRL_VALID;
    setCfg.diagOutCtrl = 0U;
    status = Pmic_diagSetOutCtrlCfg(&pmicHandle, &setCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Verify diagnostic output is disabled */
    status = Pmic_diagGetOutCtrlCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(getCfg.diagOutCtrl == 0U);
}

/**
 * @brief Test AMUX channel configuration set and get operations
 */
void test_positive_diagAMUXSetGet(void)
{
    int32_t status;
    uint8_t setChannel = 0x0AU;
    uint8_t getChannel = 0U;

    /* Set AMUX channel */
    status = Pmic_diagSetAmuxCfg(&pmicHandle, setChannel);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Verify AMUX channel */
    status = Pmic_diagGetAmuxCfg(&pmicHandle, &getChannel);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(getChannel == setChannel);

    /* Test boundary channel (max valid) */
    setChannel = 0x1FU;
    status = Pmic_diagSetAmuxCfg(&pmicHandle, setChannel);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_diagGetAmuxCfg(&pmicHandle, &getChannel);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(getChannel == setChannel);

    /* Reset to channel 0 */
    status = Pmic_diagSetAmuxCfg(&pmicHandle, 0U);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

/**
 * @brief Test DMUX group configuration set and get operations
 */
void test_positive_diagDMUXSetGet(void)
{
    int32_t status;
    uint8_t setGroup = 0x05U;
    uint8_t getGroup = 0U;

    /* Set DMUX group */
    status = Pmic_diagSetDmuxCfg(&pmicHandle, setGroup);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Verify DMUX group */
    status = Pmic_diagGetDmuxCfg(&pmicHandle, &getGroup);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(getGroup == setGroup);

    /* Test boundary group (max valid) */
    setGroup = 0x1FU;
    status = Pmic_diagSetDmuxCfg(&pmicHandle, setGroup);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_diagGetDmuxCfg(&pmicHandle, &getGroup);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(getGroup == setGroup);

    /* Reset to group 0 */
    status = Pmic_diagSetDmuxCfg(&pmicHandle, 0U);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

/**
 * @brief Core test suite entry point
 * @param args Test arguments (unused)
 */
void core_test(void *args)
{
    char msg[50U] = {0};
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_HandleCfg_t pmicCfg = {
        .validParams = (PMIC_COMM_MODE_VALID |
                        PMIC_I2C_ADDR0_VALID |
                        PMIC_COMM_HANDLE_0_VALID |
                        PMIC_IO_READ_VALID |
                        PMIC_IO_WRITE_VALID |
                        PMIC_CRITICAL_SECTION_START_VALID |
                        PMIC_CRITICAL_SECTION_STOP_VALID),
        .commMode = PMIC_INTF_SPI,
        .i2cAddr0 = PLATFORM_TARGET_I2C_ADDR,
        .i2cAddr1 = 0,
        .i2cAddr2 = 0,
        .commHandle0 = platform_getCommHandle(),
        .ioRead = &test_pmic_regRead,
        .ioWrite = &test_pmic_regWrite,
        .criticalSectionStart = &platform_critSecStart,
        .criticalSectionStop = &platform_critSecStop
    };

    platform_init();

    platform_printString("\r\n");
    platform_printString("CORE_TEST\r\n");
    platform_printString("-------\r\n\r\n");

    status = Pmic_init(&pmicHandle, &pmicCfg);

    if (status == PMIC_ST_SUCCESS)
    {
        testCommon_printSiRev(&pmicHandle);

        platform_setupTests();
        CORE_TEST_RUN_ALL();
        platform_tearDownTests();
    }
    else
    {
        (void)sprintf(msg, "Error in initializing PMIC LLD: %d\r\n", status);
        platform_printString(msg);
    }

    (void)Pmic_deinit(&pmicHandle);
    platform_deinit();
}
