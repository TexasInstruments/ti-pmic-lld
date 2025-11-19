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
 * @file platform.c
 * @brief Source file containing definitions to PMIC Core tests.
 */

/* ========================================================================== */
/*                              Include Files                                 */
/* ========================================================================== */

#include "core_test.h"

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/* Run all Core tests */
#define CORE_TEST_RUN_ALL() PLATFORM_RUN_TEST(test_negative_Pmic_setScratchPadVal_nullParam_handle); \
                            PLATFORM_RUN_TEST(test_negative_Pmic_setScratchPadVal_outOfBounds_scratchPadRegNum); \
                            PLATFORM_RUN_TEST(test_negative_Pmic_getScratchPadVal_nullParam_handle); \
                            PLATFORM_RUN_TEST(test_negative_Pmic_getScratchPadVal_nullParam_value); \
                            PLATFORM_RUN_TEST(test_negative_Pmic_getScratchPadVal_outOfBounds_scratchPadRegNum); \
                            PLATFORM_RUN_TEST(test_negative_Pmic_setRegLockState_nullParam_handle); \
                            PLATFORM_RUN_TEST(test_negative_Pmic_getRegLockState_nullParam_handle); \
                            PLATFORM_RUN_TEST(test_negative_Pmic_getRegLockState_nullParam_lockState); \
                            PLATFORM_RUN_TEST(test_negative_Pmic_configCrcEnable_nullParam_handle); \
                            PLATFORM_RUN_TEST(test_negative_Pmic_configCrcDisable_nullParam_handle); \
                            PLATFORM_RUN_TEST(test_negative_Pmic_getConfigCrcStat_nullParam_handle); \
                            PLATFORM_RUN_TEST(test_negative_Pmic_getConfigCrcStat_nullParam_configCrcStat); \
                            PLATFORM_RUN_TEST(test_negative_Pmic_configCrcCalculate_nullParam_handle); \
                            PLATFORM_RUN_TEST(test_negative_Pmic_configCrcGetFromDevice_nullParam_handle); \
                            PLATFORM_RUN_TEST(test_negative_Pmic_configCrcGetFromDevice_nullParam_crc); \
                            PLATFORM_RUN_TEST(test_positive_setGetRegLockState); \
                            PLATFORM_RUN_TEST(test_positive_setGetScratchpadReg1to4); \
                            PLATFORM_RUN_TEST(test_positive_enableDisableConfigRegCrc); \
                            PLATFORM_RUN_TEST(test_positive_configCrcCalclate); \
                            PLATFORM_RUN_TEST(test_positive_getConfigCrc)

/* Run all Core negative tests */
#define CORE_TEST_RUN_NEGATIVE() PLATFORM_RUN_TEST(test_negative_Pmic_setScratchPadVal_nullParam_handle); \
                                 PLATFORM_RUN_TEST(test_negative_Pmic_setScratchPadVal_outOfBounds_scratchPadRegNum); \
                                 PLATFORM_RUN_TEST(test_negative_Pmic_getScratchPadVal_nullParam_handle); \
                                 PLATFORM_RUN_TEST(test_negative_Pmic_getScratchPadVal_nullParam_value); \
                                 PLATFORM_RUN_TEST(test_negative_Pmic_getScratchPadVal_outOfBounds_scratchPadRegNum); \
                                 PLATFORM_RUN_TEST(test_negative_Pmic_setRegLockState_nullParam_handle); \
                                 PLATFORM_RUN_TEST(test_negative_Pmic_getRegLockState_nullParam_handle); \
                                 PLATFORM_RUN_TEST(test_negative_Pmic_getRegLockState_nullParam_lockState); \
                                 PLATFORM_RUN_TEST(test_negative_Pmic_configCrcEnable_nullParam_handle); \
                                 PLATFORM_RUN_TEST(test_negative_Pmic_configCrcDisable_nullParam_handle); \
                                 PLATFORM_RUN_TEST(test_negative_Pmic_getConfigCrcStat_nullParam_handle); \
                                 PLATFORM_RUN_TEST(test_negative_Pmic_getConfigCrcStat_nullParam_configCrcStat); \
                                 PLATFORM_RUN_TEST(test_negative_Pmic_configCrcCalculate_nullParam_handle); \
                                 PLATFORM_RUN_TEST(test_negative_Pmic_configCrcGetFromDevice_nullParam_handle); \
                                 PLATFORM_RUN_TEST(test_negative_Pmic_configCrcGetFromDevice_nullParam_crc)

/* Run all Core positive tests */
#define CORE_TEST_RUN_POSITIVE() PLATFORM_RUN_TEST(test_positive_setGetRegLockState); \
                                 PLATFORM_RUN_TEST(test_positive_setGetScratchpadReg1to4); \
                                 PLATFORM_RUN_TEST(test_positive_enableDisableConfigRegCrc); \
                                 PLATFORM_RUN_TEST(test_positive_configCrcCalclate); \
                                 PLATFORM_RUN_TEST(test_positive_getConfigCrc)

#define CORE_TEST_CALCUL_CONFIG_CRC_1_REG (0x61U)
#define CORE_TEST_CALCUL_CONFIG_CRC_2_REG (0x62U)

/* ========================================================================== */
/*                             Global Variables                               */
/* ========================================================================== */
Pmic_CoreHandle_t pmicHandle;

/* ========================================================================== */
/*                           Function Definitions                             */
/* ========================================================================== */

void core_test(void *args)
{
    char msg[50U] = {0};
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_CoreCfg_t coreCfg = {
        .validParams = (PMIC_CFG_DEVICE_TYPE_VALID_SHIFT |
                        PMIC_CFG_COMM_MODE_VALID_SHIFT |
                        PMIC_CFG_SLAVEADDR_VALID_SHIFT |
                        PMIC_CFG_COMM_HANDLE_VALID_SHIFT |
                        PMIC_CFG_COMM_IO_RD_VALID_SHIFT |
                        PMIC_CFG_COMM_IO_WR_VALID_SHIFT |
                        PMIC_CFG_CRITSEC_START_VALID_SHIFT |
                        PMIC_CFG_CRITSEC_STOP_VALID_SHIFT |
                        PMIC_CFG_CRC_ENABLE_VALID_SHIFT |
                        PMIC_CFG_CFG_CRC_ENABLE_VALID_SHIFT |
                        PMIC_CFG_PSEUDO_IRQ_VALID_SHIFT),
        .instType = PMIC_MAIN_INST,
        .pmicDeviceType = PLATFORM_TARGET_DEV_TYPE,
        .commMode = PMIC_INTF_I2C_SINGLE,
        .slaveAddr = PLATFORM_TARGET_I2C_ADDR,
        .crcEnable = PMIC_DISABLE,
        .configCrcEnable = PMIC_DISABLE,
        .pCommHandle = platform_getCommHandle(),
        .pFnPmicCommIoRd = &platform_rxByte,
        .pFnPmicCommIoWr = &platform_txByte,
        .pFnPmicCritSecStart = &platform_critSecStart,
        .pFnPmicCritSecStop = &platform_critSecStop,
        .irqResponseCallback = &platform_irqResponse
    };

    platform_init();

    platform_printString("\r\n");
    platform_printString("CORE_TEST\r\n");
    platform_printString("-------\r\n\r\n");

    status = Pmic_init(&pmicHandle, &coreCfg);

    if (status == PMIC_ST_SUCCESS)
    {
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

void test_negative_Pmic_setScratchPadVal_nullParam_handle(void)
{
    // Pass null handle into Pmic_setScratchPadVal()
    int32_t status = Pmic_setScratchPadVal(NULL, PMIC_SCRATCH_PAD_REG_1, 0xAAU);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_HANDLE);
}

void test_negative_Pmic_setScratchPadVal_outOfBounds_scratchPadRegNum(void)
{
    // Pass out of bounds scratchpad register number into Pmic_setScratchPadVal()
    int32_t status = Pmic_setScratchPadVal(&pmicHandle, PMIC_SCRATCH_PAD_REG_MAX + 1U, 0xAAU);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_negative_Pmic_getScratchPadVal_nullParam_handle(void)
{
    // Pass null handle into Pmic_getScratchPadVal()
    uint8_t value = 0U;
    int32_t status = Pmic_getScratchPadVal(NULL, PMIC_SCRATCH_PAD_REG_1, &value);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_HANDLE);
}

void test_negative_Pmic_getScratchPadVal_nullParam_value(void)
{
    // Pass null value into Pmic_getScratchPadVal()
    int32_t status = Pmic_getScratchPadVal(&pmicHandle, PMIC_SCRATCH_PAD_REG_1, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_getScratchPadVal_outOfBounds_scratchPadRegNum(void)
{
    // Pass out of bounds scratchpad register number into Pmic_getScratchPadVal()
    uint8_t value = 0U;
    int32_t status = Pmic_getScratchPadVal(&pmicHandle, PMIC_SCRATCH_PAD_REG_MAX + 1U, &value);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_negative_Pmic_setRegLockState_nullParam_handle(void)
{
    // Pass null handle into Pmic_setRegLockState()
    int32_t status = Pmic_setRegLockState(NULL, PMIC_LOCK_DISABLE);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_HANDLE);
}

void test_negative_Pmic_getRegLockState_nullParam_handle(void)
{
    bool lockState = PMIC_LOCK_DISABLE;

    // Pass null handle into Pmic_getRegLockState()
    int32_t status = Pmic_getRegLockState(NULL, &lockState);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_HANDLE);
}

void test_negative_Pmic_getRegLockState_nullParam_lockState(void)
{
    // Pass null lockState into Pmic_getRegLockState()
    int32_t status = Pmic_getRegLockState(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_configCrcEnable_nullParam_handle(void)
{
    // Pass null handle into Pmic_configCrcEnable
    int32_t status = Pmic_configCrcEnable(NULL, PMIC_CFG_CRC_ENABLE_ONLY);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_HANDLE);
}

void test_negative_Pmic_configCrcDisable_nullParam_handle(void)
{
    // Pass null handle into Pmic_configCrcDisable()
    int32_t status = Pmic_configCrcDisable(NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_HANDLE);
}

void test_negative_Pmic_getConfigCrcStat_nullParam_handle(void)
{
    // Pass null handle into Pmic_getConfigCrcStat()
    Pmic_ConfigCrcStat_t configCrcStat = {0U};
    int32_t status = Pmic_getConfigCrcStat(NULL, &configCrcStat);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_HANDLE);
}

void test_negative_Pmic_getConfigCrcStat_nullParam_configCrcStat(void)
{
    // Pass null configCrcStat into Pmic_getConfigCrcStat()
    int32_t status = Pmic_getConfigCrcStat(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_configCrcCalculate_nullParam_handle(void)
{
    // Pass null handle into Pmic_configCrcCalculate()
    int32_t status = Pmic_configCrcCalculate(NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_HANDLE);
}

void test_negative_Pmic_configCrcGetFromDevice_nullParam_handle(void)
{
    // Pass null handle into Pmic_configCrcGetFromDevice()
    uint16_t crc = 0U;
    int32_t status = Pmic_configCrcGetFromDevice(NULL, &crc);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_HANDLE);
}

void test_negative_Pmic_configCrcGetFromDevice_nullParam_crc(void)
{
    // Pass null crc into Pmic_configCrcGetFromDevice
    int32_t status = Pmic_configCrcGetFromDevice(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_positive_setGetRegLockState(void)
{
    bool actLockState = PMIC_LOCK_DISABLE;

    // Lock registers
    int32_t status = Pmic_setRegLockState(&pmicHandle, PMIC_LOCK_ENABLE);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get actual register lock state and compare expected vs. actual values
    status = Pmic_getRegLockState(&pmicHandle, &actLockState);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(actLockState == PMIC_LOCK_ENABLE);

    // Unlock registers
    status = Pmic_setRegLockState(&pmicHandle, PMIC_LOCK_DISABLE);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get actual register lock state and compare expected vs. actual values
    status = Pmic_getRegLockState(&pmicHandle, &actLockState);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(actLockState == PMIC_LOCK_DISABLE);
}

void test_positive_setGetScratchpadReg1to4(void)
{
    uint8_t initVal = 0U, expVal = 0U, actVal = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    // For each scratchpad register...
    for (uint8_t scratchPadReg = PMIC_SCRATCH_PAD_REG_1; scratchPadReg <= PMIC_SCRATCH_PAD_REG_MAX; scratchPadReg++)
    {
        // Get initial value
        status = Pmic_getScratchPadVal(&pmicHandle, scratchPadReg, &initVal);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        // Write expected value
        expVal = ~initVal;
        status = Pmic_setScratchPadVal(&pmicHandle, scratchPadReg, expVal);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        // Get actual value and compare against initial and expected values
        status = Pmic_getScratchPadVal(&pmicHandle, scratchPadReg, &actVal);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        PLATFORM_ASSERT(actVal != initVal);
        PLATFORM_ASSERT(actVal == expVal);
    }
}

void test_positive_enableDisableConfigRegCrc(void)
{
    Pmic_ConfigCrcStat_t configCrcStat = {0U};

    // Enable configuration register CRC
    int32_t status = Pmic_configCrcEnable(&pmicHandle, PMIC_CFG_CRC_ENABLE_ONLY);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get actual enable status and compare expected vs. actual value
    status = Pmic_getConfigCrcStat(&pmicHandle, &configCrcStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(configCrcStat.crcEn == PMIC_ENABLE);

    // Disable configuration register CRC
    status = Pmic_configCrcDisable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get actual enable status and compare expected vs. actual value
    status = Pmic_getConfigCrcStat(&pmicHandle, &configCrcStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(configCrcStat.crcEn == PMIC_DISABLE);
}

void test_positive_configCrcCalclate(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_ConfigCrcStat_t configCrcStat = {0U};

    // Verify that there is no prior configuration register CRC error
    status = Pmic_getConfigCrcStat(&pmicHandle, &configCrcStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(configCrcStat.errorDetected == (bool)false);

    // Enable and calculate configuration CRC
    status = Pmic_configCrcEnable(&pmicHandle, PMIC_CFG_CRC_RECALCULATE);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Disable configuration register CRC
    status = Pmic_configCrcDisable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify that there is no configuration register CRC error after calculation
    status = Pmic_getConfigCrcStat(&pmicHandle, &configCrcStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(configCrcStat.errorDetected == (bool)false);
}

static int32_t coreTest_getConfigCrc(uint16_t *crc)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t crcMsb = 0U, crcLsb = 0U;
    const uint8_t bufLen = 1U;

    status = platform_rxByte(&pmicHandle, PMIC_MAIN_INST, CORE_TEST_CALCUL_CONFIG_CRC_1_REG, &crcLsb, bufLen);

    if (status == PMIC_ST_SUCCESS)
    {
        status = platform_rxByte(&pmicHandle, PMIC_MAIN_INST, CORE_TEST_CALCUL_CONFIG_CRC_2_REG, &crcMsb, bufLen);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        *crc = (uint16_t)(((uint16_t)crcMsb << 8U) | crcLsb);
    }

    return status;
}

void test_positive_getConfigCrc(void)
{
    uint16_t expCrc = 0U, actCrc = 0U;

    // Get expected configuration CRC
    int32_t status = Pmic_configCrcGetFromDevice(&pmicHandle, &expCrc);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get actual configuration CRC and compare expected vs. actual values
    status = coreTest_getConfigCrc(&actCrc);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(expCrc == actCrc);
}

/**
 * @brief Some testing frameworks require an API to setup tests. Rename/rewrite
 * as necessary.
 */
void setUp(void)
{
}

/**
 * @brief Some testing frameworks require an API to teardown tests.
 * Rename/rewrite as necessary.
 */
void tearDown(void)
{
}
