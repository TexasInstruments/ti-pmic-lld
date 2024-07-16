/**
 * @file core_test.c
 *
 * @brief Source file for PMIC LLD Core module testing.
 */
#include "core_test.h"

#define RUN_CORE_TESTS()    RUN_TEST(test_faultHandling_setScratchPadVal_nullParams);           \
                            RUN_TEST(test_faultHandling_setScratchPadVal_invalidScratchPadReg); \
                            RUN_TEST(test_faultHandling_getScratchPadVal_nullParams);           \
                            RUN_TEST(test_faultHandling_getScratchPadVal_invalidScratchPadReg); \
                            RUN_TEST(test_functionality_setGetScratchPadVal);                   \
                            RUN_TEST(test_faultHandling_enableCRC8_nullParams);                 \
                            RUN_TEST(test_faultHandling_disableCRC8_nullParams);                \
                            RUN_TEST(test_faultHandling_getCRC8Enable_nullParams);              \
                            RUN_TEST(test_functionality_enableDisableCRC8);                     \
                            RUN_TEST(test_faultHandling_unlockRegs_nullParams);                 \
                            RUN_TEST(test_faultHandling_lockRegs_nullParams);                   \
                            RUN_TEST(test_faultHandling_getRegLock_nullParams);                 \
                            RUN_TEST(test_functionality_LockUnlockRegs);                        \
                            RUN_TEST(test_faultHandling_setPwrOn_nullParams);                   \
                            RUN_TEST(test_faultHandling_getPwrOn_nullParams);                   \
                            RUN_TEST(test_functionality_setPwrOn);                              \
                            RUN_TEST(test_faultHandling_sendFsmCmd_nullParams);                 \
                            RUN_TEST(test_faultHandling_sendFsmCmd_invalidFsmCmd);              \
                            RUN_TEST(test_functionality_sendFsmCmd_lpmEntryExit);               \
                            RUN_TEST(test_faultHandling_runABIST_nullParams);                   \
                            RUN_TEST(test_functionality_runABIST);                              \
                            RUN_TEST(test_faultHandling_setLpmCfg_nullParams);                  \
                            RUN_TEST(test_faultHandling_setLpmCfg_noValidParams);               \
                            RUN_TEST(test_faultHandling_getLpmCfg_nullParams);                  \
                            RUN_TEST(test_faultHandling_getLpmCfg_noValidParams);               \
                            RUN_TEST(test_functionality_SetGetLpmCfg_pinDetection);             \
                            RUN_TEST(test_functionality_SetGetLpmCfg_detectionDelay);           \
                            RUN_TEST(test_functionality_SetGetLpmCfg_vmonEn);                   \
                            RUN_TEST(test_functionality_SetGetLpmCfg_esmEn);                    \
                            RUN_TEST(test_functionality_SetGetLpmCfg_wdEn)

Pmic_CoreHandle_t pmicHandle;

void coreTest(void *args)
{
    Pmic_CoreCfg_t pmicCfg = {
        .i2cAddr = 0x60U,
        .commHandle = &(gI2cHandle[PMIC_I2C1]),
        .ioRead = &app_ioRead,
        .ioWrite = &app_ioWrite,
        .critSecStart = &app_critSecStart,
        .critSecStop = &app_critSecStop
    };
    int32_t status = PMIC_ST_SUCCESS;

    Drivers_open();
    Board_driversOpen();

    DebugP_log("\r\nWelcome to PMIC LLD Core module testing\r\n\r\n");

    status = Pmic_init(&pmicCfg, &pmicHandle);

    if (status == PMIC_ST_SUCCESS)
    {
        DebugP_log("Running PMIC Core tests...\r\n");
        UNITY_BEGIN();
        RUN_CORE_TESTS();
        UNITY_END();
    }
    else
    {
        DebugP_log("Failed to initialize PMIC handle\r\n");
        DebugP_log("\tStatus code: %d\r\n", status);
    }

    (void)Pmic_deinit(&pmicHandle);

    Board_driversClose();
    Drivers_close();
}

void test_faultHandling_setScratchPadVal_nullParams(void)
{
    int32_t status = PMIC_ST_SUCCESS;

    status = Pmic_setScratchPadVal(NULL, PMIC_SCRATCH_PAD_REG_1, 0xFFU);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

void test_faultHandling_setScratchPadVal_invalidScratchPadReg(void)
{
    int32_t status = PMIC_ST_SUCCESS;

    status = Pmic_setScratchPadVal(&pmicHandle, PMIC_SCRATCH_PAD_REG_MAX + 1U, 0xFFU);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);
}

void test_faultHandling_getScratchPadVal_nullParams(void)
{
    uint8_t regVal = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    status = Pmic_getScratchPadVal(NULL, PMIC_SCRATCH_PAD_REG_1, &regVal);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    status = Pmic_getScratchPadVal(&pmicHandle, PMIC_SCRATCH_PAD_REG_1, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    status = Pmic_getScratchPadVal(NULL, PMIC_SCRATCH_PAD_REG_1, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

void test_faultHandling_getScratchPadVal_invalidScratchPadReg(void)
{
    uint8_t regVal = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    status = Pmic_getScratchPadVal(&pmicHandle, PMIC_SCRATCH_PAD_REG_MAX + 1U, &regVal);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);
}

void test_functionality_setGetScratchPadVal(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t i = 0U, initVal = 0U, expVal = 0U, actVal = 0U;

    // For each scratchpad register...
    for (i = PMIC_SCRATCH_PAD_REG_1; i <= PMIC_SCRATCH_PAD_REG_4; i++)
    {
        // Get the initial register value
        status = Pmic_getScratchPadVal(&pmicHandle, i, &initVal);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Write expected register value
        expVal = ~initVal;
        status = Pmic_setScratchPadVal(&pmicHandle, i, expVal);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Get actual register value
        status = Pmic_getScratchPadVal(&pmicHandle, i, &actVal);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Compare initial vs. actual, expected vs. actual
        TEST_ASSERT_NOT_EQUAL(initVal, actVal);
        TEST_ASSERT_EQUAL(expVal, actVal);
    }
}

void test_faultHandling_enableCRC8_nullParams(void)
{
    int32_t status = PMIC_ST_SUCCESS;

    status = Pmic_enableCRC8(NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

void test_faultHandling_disableCRC8_nullParams(void)
{
    int32_t status = PMIC_ST_SUCCESS;

    status = Pmic_disableCRC8(NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

void test_faultHandling_getCRC8Enable_nullParams(void)
{
    bool crcEnabled = (bool)false;
    int32_t status = PMIC_ST_SUCCESS;

    status = Pmic_getCRC8Enable(NULL, &crcEnabled);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    status = Pmic_getCRC8Enable(&pmicHandle, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    status = Pmic_getCRC8Enable(NULL, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

void test_functionality_enableDisableCRC8(void)
{
    bool crcEnabled = (bool)false;
    int32_t status = PMIC_ST_SUCCESS;

    // Enable CRC8
    status = Pmic_enableCRC8(&pmicHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get actual CRC8 enable status; compare expected vs. actual status
    status = Pmic_getCRC8Enable(&pmicHandle, &crcEnabled);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL((bool)true, crcEnabled);

    // Disable CRC8
    status = Pmic_disableCRC8(&pmicHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get actual CRC8 enable status; compare expected vs. actual status
    status = Pmic_getCRC8Enable(&pmicHandle, &crcEnabled);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL((bool)false, crcEnabled);
}

void test_faultHandling_unlockRegs_nullParams(void)
{
    int32_t status = PMIC_ST_SUCCESS;

    status = Pmic_unlockRegs(NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

void test_faultHandling_lockRegs_nullParams(void)
{
    int32_t status = PMIC_ST_SUCCESS;

    status = Pmic_lockRegs(NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

void test_faultHandling_getRegLock_nullParams(void)
{
    bool regLocked = (bool)false;
    int32_t status = PMIC_ST_SUCCESS;

    status = Pmic_getRegLock(NULL, &regLocked);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    status = Pmic_getRegLock(&pmicHandle, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    status = Pmic_getRegLock(NULL, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

void test_functionality_LockUnlockRegs(void)
{
    bool regLocked = (bool)false;
    int32_t status = PMIC_ST_SUCCESS;

    // Lock user configuration registers
    status = Pmic_lockRegs(&pmicHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get actual register lock status; compare expected vs. actual status
    status = Pmic_getRegLock(&pmicHandle, &regLocked);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL((bool)true, regLocked);

    // Unlock user configuration registers
    status = Pmic_unlockRegs(&pmicHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get actual register lock status; compare expected vs. actual status
    status = Pmic_getRegLock(&pmicHandle, &regLocked);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL((bool)false, regLocked);
}

void test_faultHandling_setPwrOn_nullParams(void)
{
    int32_t status = PMIC_ST_SUCCESS;

    status = Pmic_setPwrOn(NULL, (bool)true);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

void test_faultHandling_getPwrOn_nullParams(void)
{
    bool pwrOnStat = (bool)false;
    int32_t status = PMIC_ST_SUCCESS;

    status = Pmic_getPwrOn(NULL, &pwrOnStat);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    status = Pmic_getPwrOn(&pmicHandle, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    status = Pmic_getPwrOn(NULL, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

void test_functionality_setPwrOn(void)
{
    bool pwrOnStat = (bool)false;
    int32_t status = PMIC_ST_SUCCESS;

    // Set PWR_ON to 1
    status = Pmic_setPwrOn(&pmicHandle, (bool)true);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get actual PWR_ON status; compare expected vs. actual status
    status = Pmic_getPwrOn(&pmicHandle, &pwrOnStat);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL((bool)true, pwrOnStat);

    // Set PWR_ON to 0
    status = Pmic_setPwrOn(&pmicHandle, (bool)false);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get actual PWR_ON status; compare expected vs. actual status
    status = Pmic_getPwrOn(&pmicHandle, &pwrOnStat);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL((bool)false, pwrOnStat);
}

void test_faultHandling_sendFsmCmd_nullParams(void)
{
    int32_t status = PMIC_ST_SUCCESS;

    status = Pmic_sendFsmCmd(NULL, PMIC_LOW_POWER_ENTRY_REQUEST);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

void test_faultHandling_sendFsmCmd_invalidFsmCmd(void)
{
    int32_t status = PMIC_ST_SUCCESS;

    status = Pmic_sendFsmCmd(NULL, 0xAAU);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);
}

void test_functionality_sendFsmCmd_lpmEntryExit(void)
{
    int32_t status = PMIC_ST_SUCCESS;

    // Send FSM command to enter LPM
    status = Pmic_sendFsmCmd(&pmicHandle, PMIC_LOW_POWER_ENTRY_REQUEST);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Send FSM command to exit LPM
    status = Pmic_sendFsmCmd(&pmicHandle, PMIC_LOW_POWER_EXIT_REQUEST);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
}

void test_faultHandling_runABIST_nullParams(void)
{
    int32_t status = PMIC_ST_SUCCESS;

    status = Pmic_runABIST(NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

void test_functionality_runABIST(void)
{
    uint8_t regData = 0U;
    bool abistActive = (bool)false;
    int32_t status = PMIC_ST_SUCCESS;
    const uint8_t maskMiscRegAddr = 0x38U;
    const uint8_t abistDoneMask_shift = 0U;
    const uint8_t abistDoneMask_mask = 1U << abistDoneMask_shift;
    const uint8_t intMiscRegAddr = 0x50U;
    const uint8_t abistDoneInt_shift = 0U;
    const uint8_t abistDoneInt_mask = 1U << abistDoneInt_shift;

    // Unmask ABIST_DONE_INT
    status = Pmic_ioRx(&pmicHandle, maskMiscRegAddr, &regData);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    Pmic_setBitField(&regData, abistDoneMask_shift, abistDoneMask_mask, 0U);
    status = Pmic_ioTx(&pmicHandle, maskMiscRegAddr, regData);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Clear ABIST_DONE_INT
    regData = 0U;
    Pmic_setBitField(&regData, abistDoneInt_shift, abistDoneInt_mask, 1U);
    status = Pmic_ioTx(&pmicHandle, intMiscRegAddr, regData);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Initiate ABIST
    status = Pmic_runABIST(&pmicHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Wait while ABIST is active by polling the ABIST_ACTIVE_STAT bit
    do
    {
        status = Pmic_getABISTStat(&pmicHandle, &abistActive);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    } while(abistActive == (bool)true);

    // Check whether ABIST_DONE_INT is raised
    status = Pmic_ioRx(&pmicHandle, intMiscRegAddr, &regData);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(1U, Pmic_getBitField(regData, abistDoneInt_shift, abistDoneInt_mask));

    // Mask ABIST_DONE_INT
    status = Pmic_ioRx(&pmicHandle, maskMiscRegAddr, &regData);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    Pmic_setBitField(&regData, abistDoneMask_shift, abistDoneMask_mask, 1U);
    status = Pmic_ioTx(&pmicHandle, maskMiscRegAddr, regData);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Clear ABIST_DONE_INT
    regData = 0U;
    Pmic_setBitField(&regData, abistDoneInt_shift, abistDoneInt_mask, 1U);
    status = Pmic_ioTx(&pmicHandle, intMiscRegAddr, regData);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
}

void test_faultHandling_setLpmCfg_nullParams(void)
{
    Pmic_CoreLpmCfg_t lpmCfg;
    int32_t status = PMIC_ST_SUCCESS;

    status = Pmic_setLpmCfg(NULL, &lpmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_setLpmCfg(&pmicHandle, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_setLpmCfg(NULL, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
}

void test_faultHandling_setLpmCfg_noValidParams(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_CoreLpmCfg_t lpmCfg = {.validParams = 0U};

    status = Pmic_setLpmCfg(&pmicHandle, &lpmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);
}

void test_faultHandling_getLpmCfg_nullParams(void)
{
    Pmic_CoreLpmCfg_t lpmCfg;
    int32_t status = PMIC_ST_SUCCESS;

    status = Pmic_getLpmCfg(NULL, &lpmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_getLpmCfg(&pmicHandle, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_getLpmCfg(NULL, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
}

void test_faultHandling_getLpmCfg_noValidParams(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_CoreLpmCfg_t lpmCfg = {.validParams = 0U};

    status = Pmic_getLpmCfg(&pmicHandle, &lpmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);
}

void test_functionality_SetGetLpmCfg_pinDetection(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_CoreLpmCfg_t expLpmCfg = {.validParams = PMIC_LPM_PIN_DETECTION_VALID};
    Pmic_CoreLpmCfg_t actLpmCfg = {.validParams = PMIC_LPM_PIN_DETECTION_VALID};

    // Test Pmic_setLpmCfg() API error handling for when pin detection
    // configuration is greater than the max value
    expLpmCfg.pinDetection = PMIC_PIN_DETECTION_CONDITION_MAX + 1U;
    status = Pmic_setLpmCfg(&pmicHandle, &expLpmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

    // Set pin detection configuration to be 0 - all IRQs must be cleared
    expLpmCfg.pinDetection = PMIC_ALL_IRQ_CLEARED_CONDITION;
    status = Pmic_setLpmCfg(&pmicHandle, &expLpmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get actual pin detection configuration and compare expected vs actual
    status = Pmic_getLpmCfg(&pmicHandle, &actLpmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(PMIC_ALL_IRQ_CLEARED_CONDITION, actLpmCfg.pinDetection);

    // Set pin detection configuration to be 1 - delay value defined by LOWPWR_DELAY must be met
    expLpmCfg.pinDetection = PMIC_DELAY_VALUE_MET_CONDITION;
    status = Pmic_setLpmCfg(&pmicHandle, &expLpmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get actual pin detection configuration and compare expected vs actual
    status = Pmic_getLpmCfg(&pmicHandle, &actLpmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(PMIC_DELAY_VALUE_MET_CONDITION, actLpmCfg.pinDetection);
}

void test_functionality_SetGetLpmCfg_detectionDelay(void)
{
    uint8_t i = 0U;
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_CoreLpmCfg_t expLpmCfg = {.validParams = PMIC_LPM_DETECTION_DELAY_VALID};
    Pmic_CoreLpmCfg_t actLpmCfg = {.validParams = PMIC_LPM_DETECTION_DELAY_VALID};

    // Test Pmic_setLpmCfg() API error handling for when pin detection
    // delay configuration is greater than the max value
    expLpmCfg.pinDetection = PMIC_DETECTION_DELAY_MAX + 1U;
    status = Pmic_setLpmCfg(&pmicHandle, &expLpmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

    // Test all LPM pin detection delay configurations
    for (i = PMIC_DETECTION_DELAY_50_MS; i <= PMIC_DETECTION_DELAY_500_MS; i++)
    {
        // Set pin detection delay configuration
        expLpmCfg.detectionDelay = i;
        status = Pmic_setLpmCfg(&pmicHandle, &expLpmCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Get actual pin detection delay configuration and compare expected vs actual
        status = Pmic_getLpmCfg(&pmicHandle, &actLpmCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        TEST_ASSERT_EQUAL(expLpmCfg.detectionDelay, actLpmCfg.detectionDelay);
    }
}

void test_functionality_SetGetLpmCfg_vmonEn(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_CoreLpmCfg_t expLpmCfg = {.validParams = PMIC_LPM_VMON_EN_VALID};
    Pmic_CoreLpmCfg_t actLpmCfg = {.validParams = PMIC_LPM_VMON_EN_VALID};

    // Disable VMON in LPM (LOWPWR_VMON_EN = 0)
    expLpmCfg.vmonEn = (bool)false;
    status = Pmic_setLpmCfg(&pmicHandle, &expLpmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get actual LPM VMON_EN and compare expected vs. actual
    status = Pmic_getLpmCfg(&pmicHandle, &actLpmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL((bool)false, actLpmCfg.vmonEn);

    // Enable VMON in LPM (LOWPWR_VMON_EN = 1)
    expLpmCfg.vmonEn = (bool)true;
    status = Pmic_setLpmCfg(&pmicHandle, &expLpmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get actual LPM VMON_EN and compare expected vs. actual
    status = Pmic_getLpmCfg(&pmicHandle, &actLpmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL((bool)true, actLpmCfg.vmonEn);
}

void test_functionality_SetGetLpmCfg_esmEn(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_CoreLpmCfg_t expLpmCfg = {.validParams = PMIC_LPM_ESM_EN_VALID};
    Pmic_CoreLpmCfg_t actLpmCfg = {.validParams = PMIC_LPM_ESM_EN_VALID};

    // Disable ESM in LPM (LOWPWR_ESM_EN = 0)
    expLpmCfg.esmEn = (bool)false;
    status = Pmic_setLpmCfg(&pmicHandle, &expLpmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get actual LPM ESM_EN and compare expected vs. actual
    status = Pmic_getLpmCfg(&pmicHandle, &actLpmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL((bool)false, actLpmCfg.esmEn);

    // Enable ESM in LPM (LOWPWR_ESM_EN = 1)
    expLpmCfg.esmEn = (bool)true;
    status = Pmic_setLpmCfg(&pmicHandle, &expLpmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get actual LPM ESM_EN and compare expected vs. actual
    status = Pmic_getLpmCfg(&pmicHandle, &actLpmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL((bool)true, actLpmCfg.esmEn);
}

void test_functionality_SetGetLpmCfg_wdEn(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_CoreLpmCfg_t expLpmCfg = {.validParams = PMIC_LPM_WDG_EN_VALID};
    Pmic_CoreLpmCfg_t actLpmCfg = {.validParams = PMIC_LPM_WDG_EN_VALID};

    // Disable WDG in LPM (LOWPWR_WD_EN = 0)
    expLpmCfg.wdgEn = (bool)false;
    status = Pmic_setLpmCfg(&pmicHandle, &expLpmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get actual LPM WD_EN and compare expected vs. actual
    status = Pmic_getLpmCfg(&pmicHandle, &actLpmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL((bool)false, actLpmCfg.wdgEn);

    // Enable WDG in LPM (LOWPWR_WD_EN = 1)
    expLpmCfg.wdgEn = (bool)true;
    status = Pmic_setLpmCfg(&pmicHandle, &expLpmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get actual LPM WD_EN and compare expected vs. actual
    status = Pmic_getLpmCfg(&pmicHandle, &actLpmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL((bool)true, actLpmCfg.wdgEn);
}
