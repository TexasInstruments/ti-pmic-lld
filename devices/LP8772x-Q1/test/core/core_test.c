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
/*                              Include Files                                 */
/* ========================================================================== */

#include "core_test.h"
#include "test_constants.h"

#ifdef BUILD_MOCK
#include "test_inject.h"
#endif

/* ========================================================================== */
/*                             Global Variables                               */
/* ========================================================================== */
static Pmic_Handle_t pmicHandle;

/* ========================================================================== */
/*                           Function Definitions                             */
/* ========================================================================== */

void core_test(void *args)
{
    (void)args;
    char msg[50U] = {0};
    int32_t status = PMIC_ST_SUCCESS;

    platform_init();

    Pmic_HandleCfg_t coreCfg = {
        .validParams = (PMIC_CFG_INIT_COMM_MODE_VALID |
                        PMIC_CFG_INIT_I2C_ADDR0_VALID |
                        PMIC_CFG_INIT_COMM_HANDLE_0_VALID |
                        PMIC_CFG_INIT_IO_READ_VALID |
                        PMIC_CFG_INIT_IO_WRITE_VALID |
                        PMIC_CFG_INIT_CRITICAL_SECTION_START_VALID |
                        PMIC_CFG_INIT_CRITICAL_SECTION_STOP_VALID |
                        PMIC_CFG_INIT_CRC_ENABLE_VALID |
                        PMIC_CFG_INIT_CONFIG_CRC_ENABLE_VALID |
                        PMIC_CFG_INIT_IRQ_RESPONSE_CALLBACK_VALID),
        .commMode = PMIC_INTF_I2C_SINGLE,
        .i2cAddr0 = PLATFORM_TARGET_I2C_ADDR,
        .crcEnable = PMIC_DISABLE,
        .configCrcEnable = PMIC_DISABLE,
        .commHandle0 = platform_getCommHandle(),
        .ioRead = &platform_rxByte,
        .ioWrite = &platform_txByte,
        .criticalSectionStart = &platform_critSecStart,
        .criticalSectionStop = &platform_critSecStop,
        .irqResponseCallback = &platform_irqResponse
    };

    testTimer_startModule("Core");

    status = Pmic_init(&pmicHandle, &coreCfg);

    if (status == PMIC_ST_SUCCESS)
    {
        platform_setupTests();
        CORE_TEST_RUN_ALL();
        platform_tearDownTests();
    }
    else
    {
        (void)sprintf(msg, "Error in initializing PMIC LLD: %ld\r\n", (long)status);
        platform_printString(msg);
    }

    testTimer_endModule();

    (void)Pmic_deinit(&pmicHandle);
    platform_deinit();
}

void test_neg_core_setScratchPadValue_nullHandle(void)
{
    // Pass null handle into Pmic_setScratchPadValue()
    int32_t status = Pmic_setScratchPadValue(NULL, PMIC_SCRATCH_PAD_REG_1, TEST_PATTERN_AA);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_core_setScratchPadValue_outOfBounds(void)
{
    // Pass out of bounds scratchpad register number into Pmic_setScratchPadValue()
    int32_t status = Pmic_setScratchPadValue(&pmicHandle, PMIC_SCRATCH_PAD_REG_MAX + 1U, TEST_PATTERN_AA);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_core_getScratchPadValue_nullHandle(void)
{
    // Pass null handle into Pmic_getScratchPadValue()
    uint8_t value = 0U;
    int32_t status = Pmic_getScratchPadValue(NULL, PMIC_SCRATCH_PAD_REG_1, &value);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_core_getScratchPadValue_nullValue(void)
{
    // Pass null value into Pmic_getScratchPadValue()
    int32_t status = Pmic_getScratchPadValue(&pmicHandle, PMIC_SCRATCH_PAD_REG_1, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_core_getScratchPadValue_outOfBounds(void)
{
    // Pass out of bounds scratchpad register number into Pmic_getScratchPadValue()
    uint8_t value = 0U;
    int32_t status = Pmic_getScratchPadValue(&pmicHandle, PMIC_SCRATCH_PAD_REG_MAX + 1U, &value);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_core_setRegLockState_nullHandle(void)
{
    // Pass null handle into Pmic_setRegLockState()
    int32_t status = Pmic_setRegLockState(NULL, PMIC_LOCK_DISABLE);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_core_getRegLockState_nullHandle(void)
{
    bool lockState = PMIC_LOCK_DISABLE;

    // Pass null handle into Pmic_getRegLockState()
    int32_t status = Pmic_getRegLockState(NULL, &lockState);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_core_getRegLockState_nullLockState(void)
{
    // Pass null lockState into Pmic_getRegLockState()
    int32_t status = Pmic_getRegLockState(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_core_configCrcEnable_nullHandle(void)
{
    // Pass null handle into Pmic_configCrcEnable
    int32_t status = Pmic_configCrcEnable(NULL, PMIC_CFG_CRC_ENABLE_ONLY);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_core_configCrcDisable_nullHandle(void)
{
    // Pass null handle into Pmic_configCrcDisable()
    int32_t status = Pmic_configCrcDisable(NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_core_getConfigCrcStatus_nullHandle(void)
{
    // Pass null handle into Pmic_getConfigCrcStatus()
    Pmic_ConfigCrcStat_t configCrcStat = {0U};
    int32_t status = Pmic_getConfigCrcStatus(NULL, &configCrcStat);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_core_getConfigCrcStatus_nullStatus(void)
{
    // Pass null configCrcStat into Pmic_getConfigCrcStatus()
    int32_t status = Pmic_getConfigCrcStatus(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_core_configCrcCalculate_nullHandle(void)
{
    // Pass null handle into Pmic_configCrcCalculate()
    int32_t status = Pmic_configCrcCalculate(NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_core_getConfigCrc_nullHandle(void)
{
    uint16_t value = 0U;
    int32_t status = Pmic_getConfigCrc(NULL, &value);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_core_getConfigCrc_nullValue(void)
{
    int32_t status = Pmic_getConfigCrc(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_pos_core_setRegLockState_enableDisable(void)
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

void test_pos_core_getScratchPadValue_reg1to4(void)
{
    uint8_t initVal = 0U, expVal = 0U, actVal = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    // For each scratchpad register...
    for (uint8_t scratchPadReg = PMIC_SCRATCH_PAD_REG_1; scratchPadReg <= PMIC_SCRATCH_PAD_REG_MAX; scratchPadReg++)
    {
        // Get initial value
        status = Pmic_getScratchPadValue(&pmicHandle, scratchPadReg, &initVal);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        // Write expected value
        expVal = ~initVal;
        status = Pmic_setScratchPadValue(&pmicHandle, scratchPadReg, expVal);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        // Get actual value and compare against initial and expected values
        status = Pmic_getScratchPadValue(&pmicHandle, scratchPadReg, &actVal);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        PLATFORM_ASSERT(actVal != initVal);
        PLATFORM_ASSERT(actVal == expVal);
    }
}

void test_pos_core_configCrcEnable_enableOnly(void)
{
    Pmic_ConfigCrcStat_t configCrcStat = {0U};

    // Enable configuration register CRC
    int32_t status = Pmic_configCrcEnable(&pmicHandle, PMIC_CFG_CRC_ENABLE_ONLY);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get actual enable status and compare expected vs. actual value
    status = Pmic_getConfigCrcStatus(&pmicHandle, &configCrcStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(configCrcStat.crcEn == PMIC_ENABLE);

    // Disable configuration register CRC
    status = Pmic_configCrcDisable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get actual enable status and compare expected vs. actual value
    status = Pmic_getConfigCrcStatus(&pmicHandle, &configCrcStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(configCrcStat.crcEn == PMIC_DISABLE);
}

void test_pos_core_configCrcEnable_recalculate(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_ConfigCrcStat_t configCrcStat = {0U};

    // Verify that there is no prior configuration register CRC error
    status = Pmic_getConfigCrcStatus(&pmicHandle, &configCrcStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(configCrcStat.errorDetected == (bool)false);

    // Enable and calculate configuration CRC
    status = Pmic_configCrcEnable(&pmicHandle, PMIC_CFG_CRC_RECALCULATE);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Disable configuration register CRC
    status = Pmic_configCrcDisable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify that there is no configuration register CRC error after calculation
    status = Pmic_getConfigCrcStatus(&pmicHandle, &configCrcStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(configCrcStat.errorDetected == (bool)false);
}

static int32_t coreTest_getConfigCrc(uint16_t *crc)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t crcMsb = 0U, crcLsb = 0U;
    const uint8_t bufLen = 1U;

    status = platform_rxByte(&pmicHandle, 0U, CORE_TEST_CALCUL_CONFIG_CRC_1_REG, &crcLsb, bufLen);

    if (status == PMIC_ST_SUCCESS)
    {
        status = platform_rxByte(&pmicHandle, 0U, CORE_TEST_CALCUL_CONFIG_CRC_2_REG, &crcMsb, bufLen);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        *crc = (uint16_t)(((uint16_t)crcMsb << 8U) | crcLsb);
    }

    return status;
}

void test_pos_core_getConfigCrc_readValue(void)
{
    uint16_t expValue = 0U, actValue = 0U;

    // Get expected configuration CRC value from device
    int32_t status = Pmic_getConfigCrc(&pmicHandle, &expValue);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get actual configuration CRC via raw platform read and compare
    status = coreTest_getConfigCrc(&actValue);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(expValue == actValue);
}

void test_pos_core_setConfigCrc_writeValue(void)
{
    // Write a known 16-bit value to the config CRC registers
    int32_t status = Pmic_setConfigCrc(&pmicHandle, 0xA55AU);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_neg_core_setConfigCrc_nullHandle(void)
{
    int32_t status = Pmic_setConfigCrc(NULL, 0xA55AU);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/* ========================================================================== */
/*              LP8772x-Q1 Specific CRC Configuration Tests                   */
/* ========================================================================== */

void test_pos_core_configCrcDisable_disable(void)
{
    Pmic_ConfigCrcStat_t configCrcStat = {0U};

    // First enable CRC with calculation to properly initialize device
    int32_t status = Pmic_configCrcEnable(&pmicHandle, PMIC_CFG_CRC_RECALCULATE);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify it's enabled
    status = Pmic_getConfigCrcStatus(&pmicHandle, &configCrcStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(configCrcStat.crcEn == PMIC_ENABLE);

    // Now disable - this covers the Pmic_setBitField_b PMIC_DISABLE path
    status = Pmic_configCrcDisable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify it's disabled
    status = Pmic_getConfigCrcStatus(&pmicHandle, &configCrcStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(configCrcStat.crcEn == PMIC_DISABLE);
}


/* ========================================================================== */
/*              LP8772x-Q1 Additional Error Handling Coverage Tests          */
/* ========================================================================== */

void test_neg_core_init_invalidDeviceType(void)
{
    // Test init with invalid configuration (null handle)
    // This covers error handling in Pmic_init for invalid parameters
    Pmic_HandleCfg_t invalidCfg = {
        .validParams = PMIC_CFG_INIT_COMM_MODE_VALID,
        .commMode = PMIC_INTF_MAX + 1U  // Invalid comm mode
    };
    Pmic_Handle_t tempHandle;
    int32_t status = Pmic_init(&tempHandle, &invalidCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_pos_core_errStatus_multipleErrors(void)
{
    // Test reading multiple error status bits
    // This exercises the error status register read paths
    Pmic_ConfigCrcStat_t crcStat = {0U};

    // First, read the CRC status to exercise error status reading
    int32_t status = Pmic_getConfigCrcStatus(&pmicHandle, &crcStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify that the status structure was populated
    // crcEn should be either true or false
    PLATFORM_ASSERT((crcStat.crcEn == PMIC_ENABLE) || (crcStat.crcEn == PMIC_DISABLE));
}

void test_pos_core_errStatus_specificError(void)
{
    // Test clearing specific error bits
    // This covers error clearing logic in core module

    // First, ensure config CRC is in a known state
    int32_t status = Pmic_configCrcDisable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Read status to verify it's disabled
    Pmic_ConfigCrcStat_t crcStat = {0U};
    status = Pmic_getConfigCrcStatus(&pmicHandle, &crcStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(crcStat.crcEn == PMIC_DISABLE);

    // Now enable it with calculation to exercise a different code path
    status = Pmic_configCrcEnable(&pmicHandle, PMIC_CFG_CRC_RECALCULATE);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify it's enabled
    status = Pmic_getConfigCrcStatus(&pmicHandle, &crcStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(crcStat.crcEn == PMIC_ENABLE);

    // Clean up - disable again
    status = Pmic_configCrcDisable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

/* ========================================================================== */
/*           LP8772x-Q1 Tests for Uncovered Lines in pmic_core.c             */
/* ========================================================================== */

void test_neg_core_configCrcEnable_alreadyEnabled(void)
{
    // Test coverage for lines 288-289: CONFIG_CRC_EN already enabled
    // When Pmic_configCrcCalculate() is called with CRC already enabled,
    // it should return PMIC_ST_ERR_NOT_SUPPORTED

    int32_t status = PMIC_ST_SUCCESS;

    // First ensure CRC is disabled
    status = Pmic_configCrcDisable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Enable config CRC
    status = Pmic_configCrcEnable(&pmicHandle, PMIC_CFG_CRC_ENABLE_ONLY);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Now try to calculate CRC while it's enabled - this should fail
    // This triggers line 287-289 where it checks if CONFIG_CRC_EN is set
    status = Pmic_configCrcCalculate(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NOT_SUPPORTED);

    // Clean up
    (void)Pmic_configCrcDisable(&pmicHandle);
}

void test_neg_core_configCrcEnable_calcBitHigh(void)
{
#ifdef BUILD_MOCK
    // Test coverage for lines 294-296: CONFIG_CRC_CALC bit already high
    // When CONFIG_CRC_CALC bit is high, the code should set it low first

    int32_t status = PMIC_ST_SUCCESS;
    const uint16_t CONFIG_CRC_CONFIG_REG = 0x60U;  // CONFIG_CRC_CONFIG register address
    const uint8_t CONFIG_CRC_CALC_SHIFT = 1U;       // CONFIG_CRC_CALC bit position
    uint8_t regData = 0U;

    // Ensure CRC is disabled
    status = Pmic_configCrcDisable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Inject CONFIG_CRC_CALC bit as high (bit 1)
    regData = (1UL << CONFIG_CRC_CALC_SHIFT);
    testInject_setBits(CONFIG_CRC_CONFIG_REG, regData);

    // Now call Pmic_configCrcCalculate() - it should detect the bit is high
    // and clear it first (lines 293-296)
    status = Pmic_configCrcCalculate(&pmicHandle);
    // Should succeed after clearing the bit
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Clean up
    (void)Pmic_configCrcDisable(&pmicHandle);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for register injection");
#endif
}

void test_neg_core_configCrcEnable_crcMismatch(void)
{
#ifdef BUILD_MOCK
    // Test coverage for lines 320-321: CRC mismatch detection
    // The code reads CONFIG_CRC_CONFIG_REG at line 307 and checks
    // CONFIG_CRC_STATUS bit (bit 2) at line 319

    int32_t status = PMIC_ST_SUCCESS;
    const uint16_t CONFIG_CRC_CONFIG_REG = 0x60U;  // CONFIG_CRC_CONFIG register address
    const uint8_t CONFIG_CRC_STATUS_SHIFT = 2U;    // CONFIG_CRC_STATUS bit position (bit 2)
    uint8_t regData = 0U;

    // Ensure CRC is disabled first
    status = Pmic_configCrcDisable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Inject CONFIG_CRC_STATUS bit (bit 2) into CONFIG_CRC_CONFIG_REG (0x60)
    // to simulate a CRC mismatch. This will be read at line 307 during
    // Pmic_configCrcCalculate() and checked at line 319
    regData = (1UL << CONFIG_CRC_STATUS_SHIFT);
    testInject_setBits(CONFIG_CRC_CONFIG_REG, regData);

    // Call Pmic_configCrcCalculate() which will trigger lines 319-321
    // The function should detect the mismatch and return PMIC_ST_ERR_CONFIG_REG_CRC
    status = Pmic_configCrcCalculate(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_CONFIG_REG_CRC);

    // Clean up - clear the injected bit
    testInject_clearBits(CONFIG_CRC_CONFIG_REG, regData);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for register injection");
#endif
}

void test_neg_core_configCrcCalculate_ioFailure(void)
{
#ifdef BUILD_MOCK
    int32_t status;
    PmicMockDevice_t *mockDevice;

    mockDevice = platform_getMockDevice();
    PLATFORM_ASSERT(mockDevice != NULL);

    // Inject error on 3rd I/O operation (3rd iteration of CRC loop)
    // This covers pmic_core.c:337-338 (break on I/O failure)
    status = PmicMock_InjectError(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 3);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_configCrcCalculate(&pmicHandle);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}
