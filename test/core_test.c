/**
 * @file core_test.c
 *
 * @brief Source file for PMIC LLD Core module testing.
 */
#include "core_test.h"

#define RUN_CORE_TESTS()    RUN_TEST(test_faultHandling_setRegLock_nullParams);                     \
                            RUN_TEST(test_faultHandling_getRegLock_nullParams);                     \
                            RUN_TEST(test_faultHandling_unlockRegs_nullParams);                     \
                            RUN_TEST(test_faultHandling_lockRegs_nullParams);                       \
                            RUN_TEST(test_functionality_setGetRegLock);                             \
                            RUN_TEST(test_faultHandling_writeCfgCrc_nullParams);                    \
                            RUN_TEST(test_faultHandling_setCfgCrc_nullParams);                      \
                            RUN_TEST(test_faultHandling_enableCfgCrc_nullParams);                   \
                            RUN_TEST(test_faultHandling_disableCfgCrc_nullParams);                  \
                            RUN_TEST(test_faultHandling_getCfgCrcErrStat_nullParams);               \
                            RUN_TEST(test_functionality_writeCfgCrc_setCfgCrc);                     \
                            RUN_TEST(test_faultHandling_getDevState_nullParams);                    \
                            RUN_TEST(test_functionality_getDevState);                               \
                            RUN_TEST(test_faultHandling_sendRstReq_nullParams);                     \
                            RUN_TEST(test_faultHandling_sendSafeExitReq_nullParams);                \
                            RUN_TEST(test_faultHandling_getBistStat_nullParams);                    \
                            RUN_TEST(test_faultHandling_startLBIST_nullParams);                     \
                            RUN_TEST(test_faultHandling_startABIST_nullParams);                     \
                            RUN_TEST(test_faultHandling_setDeviceCfg_nullParams);                   \
                            RUN_TEST(test_faultHandling_getDeviceCfg_nullParams);                   \
                            RUN_TEST(test_functionality_setGetDeviceCfg_pwrDwnThr);                 \
                            RUN_TEST(test_functionality_setGetDeviceCfg_safeLockThr);               \
                            RUN_TEST(test_functionality_setGetDeviceCfg_devErrCnt);                 \
                            RUN_TEST(test_functionality_setGetDeviceCfg_safeTimeoutDuration);       \
                            RUN_TEST(test_functionality_setGetDeviceCfg_disableSafeLockTimeout);    \
                            RUN_TEST(test_functionality_setGetDeviceCfg_disableAutoBIST);           \
                            RUN_TEST(test_functionality_setGetDeviceCfg_enableDrv);                 \
                            RUN_TEST(test_functionality_setGetDeviceCfg_diagExitMask);              \
                            RUN_TEST(test_functionality_setGetDeviceCfg_diagExit)

static int32_t coreTest_setup(void);

Pmic_CoreHandle_t pmicHandle;

void coreTest(void *args)
{
    Pmic_CoreCfg_t pmicCfg = {
        .validParams = (PMIC_CRIT_SEC_START_VALID | PMIC_CRIT_SEC_STOP_VALID |
            PMIC_IO_TRANSFER_VALID | PMIC_IRQ_RESPONSE_VALID),
        .critSecStart = &app_critSecStart,
        .critSecStop = &app_critSecStop,
        .ioTransfer = &app_spiTransfer,
        .irqResponse = &app_irqResponse
    };
    int32_t status = PMIC_ST_SUCCESS;

    Drivers_open();
    Board_driversOpen();

    DebugP_log("\r\nWelcome to PMIC LLD Core module testing\r\n\r\n");

    status = Pmic_init(&pmicCfg, &pmicHandle);

    if (status == PMIC_ST_SUCCESS)
    {
        status = coreTest_setup();

        if (status == PMIC_ST_SUCCESS)
        {
            DebugP_log("Running PMIC Core tests...\r\n");
            UNITY_BEGIN();
            RUN_CORE_TESTS();
            UNITY_END();
        }
        else
        {
            DebugP_log("Failed to setup Core module test\r\n");
            DebugP_log("\tStatus code: %d\r\n", status);
        }
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

static int32_t coreTest_setup(void)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    Pmic_criticalSectionStart(&pmicHandle);

    // Send MCU_RST_RQ to put PMIC into a known state
    status = Pmic_ioTxByte(&pmicHandle, PMIC_CMD_MCU_RST_RQ, PMIC_CMD_MCU_RST_RQ_DATA);

    // Read DEV_CFG_3 register
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioRxByte(&pmicHandle, PMIC_CMD_RD_DEV_CFG_3, &regData);
    }

    // Ensure MCU_RST_RQ_FLAG bit is set to 1 to ensure command was acknowledged
    if ((status == PMIC_ST_SUCCESS) && (Pmic_getBitField_b(regData, MCU_RST_RQ_FLAG_SHIFT) == (bool)false))
    {
        DebugP_log("Expected MCU_RST_RQ_FLAG to be 1; actual value is 0\r\n");
        status = PMIC_ST_ERR_FAIL;
    }

    // Read SAFETY_CHECK_CTRL register
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioRxByte(&pmicHandle, PMIC_CMD_RD_SAFETY_CHECK_CTRL, &regData);
    }

    // Set DIAG_EXIT to 0 to prevent PMIC from transitioning to ACTIVE state
    // Set DIAG_EXIT_MASK to 1 to enable PMIC to stay in DIAGNOSTIC state
    if (status == PMIC_ST_SUCCESS)
    {
        Pmic_setBitField_b(&regData, DIAG_EXIT_SHIFT, (bool)false);
        Pmic_setBitField_b(&regData, DIAG_EXIT_MASK_SHIFT, (bool)true);
        status = Pmic_ioTxByte(&pmicHandle, PMIC_CMD_WR_SAFETY_CHECK_CTRL, regData);
    }

    // Read SAFETY_STAT_5 register
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioRxByte(&pmicHandle, PMIC_CMD_RD_SAFETY_STAT_5, &regData);
    }

    // Ensure that PMIC is in DIAGNOSTIC state
    if ((status == PMIC_ST_SUCCESS) && (Pmic_getBitField(regData, STATE_SHIFT, STATE_MASK) != PMIC_DIAGNOSTIC_STATE))
    {
        DebugP_log("Expected PMIC to be in Diagnostic state; actual device state value: %d\r\n",
                    Pmic_getBitField(regData, STATE_SHIFT, STATE_MASK));

        status = PMIC_ST_ERR_FAIL;
    }

    // Unlock PMIC configuration registers
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioTxByte(&pmicHandle, PMIC_CMD_SW_UNLOCK, PMIC_CMD_SW_UNLOCK_DATA);
    }

    // Read SAFETY_ERR_STAT_1
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioRxByte(&pmicHandle, PMIC_CMD_RD_SAFETY_ERR_STAT_1, &regData);
    }

    // Set DEV_ERR_CNT to 0
    if (status == PMIC_ST_SUCCESS)
    {
        Pmic_setBitField(&regData, DEV_ERR_CNT_SHIFT, DEV_ERR_CNT_MASK, 0U);
        status = Pmic_ioTxByte(&pmicHandle, PMIC_CMD_WR_SAFETY_ERR_STAT_1, regData);
    }

    // Read SAFETY_PWD_THR_CFG register
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioRxByte(&pmicHandle, PMIC_CMD_RD_SAFETY_PWD_THR_CFG, &regData);
    }

    // Set PWD_THR to 15
    if (status == PMIC_ST_SUCCESS)
    {
        Pmic_setBitField(&regData, PWD_THR_SHIFT, PWD_THR_MASK, 15U);
        status = Pmic_ioTxByte(&pmicHandle, PMIC_CMD_WR_SAFETY_PWD_THR_CFG, regData);
    }

    Pmic_criticalSectionStop(&pmicHandle);

    return status;
}

// Unity testing framework calls this API before each test.
void setUp(void)
{
}

// Unity testing framework calls this API after each test.
void tearDown(void)
{
}


void test_faultHandling_setRegLock_nullParams(void)
{
    int32_t status = Pmic_setRegLock(NULL, (bool)true);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

void test_faultHandling_getRegLock_nullParams(void)
{
    bool locked = (bool)false;

    int32_t status = Pmic_getRegLock(NULL, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    status = Pmic_getRegLock(&pmicHandle, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    status = Pmic_getRegLock(NULL, &locked);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

void test_faultHandling_unlockRegs_nullParams(void)
{
    int32_t status = Pmic_unlockRegs(NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

void test_faultHandling_lockRegs_nullParams(void)
{
    int32_t status = Pmic_lockRegs(NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

void test_functionality_setGetRegLock(void)
{
    bool locked = (bool)false;
    int32_t status = PMIC_ST_SUCCESS;

    // Lock PMIC configuration registers
    status = Pmic_setRegLock(&pmicHandle, (bool)true);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get lock status and compare expected vs. actual value
    status = Pmic_getRegLock(&pmicHandle, &locked);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL((bool)true, locked);

    // Unlock PMIC configuration registers
    status = Pmic_setRegLock(&pmicHandle, (bool)false);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get lock status and compare expected vs. actual value
    status = Pmic_getRegLock(&pmicHandle, &locked);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL((bool)false, locked);
}

void test_faultHandling_writeCfgCrc_nullParams(void)
{
    int32_t status = Pmic_writeCfgCrc(NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

void test_faultHandling_setCfgCrc_nullParams(void)
{
    int32_t status = Pmic_setCfgCrc(NULL, (bool)true);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

void test_faultHandling_enableCfgCrc_nullParams(void)
{
    int32_t status = Pmic_enableCfgCrc(NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

void test_faultHandling_disableCfgCrc_nullParams(void)
{
    int32_t status = Pmic_disableCfgCrc(NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

void test_faultHandling_getCfgCrcErrStat_nullParams(void)
{
    bool CfgCrcErr = (bool)false;

    int32_t status = Pmic_getCfgCrcErrStat(NULL, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    status = Pmic_getCfgCrcErrStat(&pmicHandle, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    status = Pmic_getCfgCrcErrStat(NULL, &CfgCrcErr);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

void test_functionality_writeCfgCrc_setCfgCrc(void)
{
    bool CfgCrcErr = (bool)true;

    // Calculate and write configuration register CRC
    int32_t status = Pmic_writeCfgCrc(&pmicHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Enable configuration register CRC
    status = Pmic_setCfgCrc(&pmicHandle, PMIC_ENABLE);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // delay for 1 second for the PMIC to conduct multiple CRC calculations and comparisons
    app_wait(1000U);

    // Disable configuration register CRC
    status = Pmic_setCfgCrc(&pmicHandle, PMIC_DISABLE);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Check whether there is a configuration register CRC error
    status = Pmic_getCfgCrcErrStat(&pmicHandle, &CfgCrcErr);
    TEST_ASSERT_EQUAL((bool)false, CfgCrcErr);
}

void test_faultHandling_getDevState_nullParams(void)
{
    uint8_t devState = 0U;

    int32_t status = Pmic_getDevState(NULL, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    status = Pmic_getDevState(&pmicHandle, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    status = Pmic_getDevState(NULL, &devState);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

void test_faultHandling_sendRstReq_nullParams(void)
{
    int32_t status = Pmic_sendRstReq(NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

void test_faultHandling_sendSafeExitReq_nullParams(void)
{
    int32_t status = Pmic_sendSafeExitReq(NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

void test_faultHandling_getBistStat_nullParams(void)
{
    Pmic_BistStat_t bistStat;

    int32_t status = Pmic_getBistStat(NULL, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    status = Pmic_getBistStat(&pmicHandle, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    status = Pmic_getBistStat(NULL, &bistStat);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

void test_faultHandling_startLBIST_nullParams(void)
{
    int32_t status = Pmic_startLBIST(NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

void test_faultHandling_startABIST_nullParams(void)
{
    int32_t status = Pmic_startABIST(NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

void test_functionality_getDevState(void)
{
    uint8_t devState = 0U;

    int32_t status = Pmic_getDevState(&pmicHandle, &devState);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    if ((devState != PMIC_DIAGNOSTIC_STATE) && (devState != PMIC_SAFE_STATE) && (devState != PMIC_ACTIVE_STATE))
    {
        TEST_FAIL_MESSAGE("Unrecognized device state");
    }
}

void test_faultHandling_setDeviceCfg_nullParams(void)
{
    Pmic_DeviceCfg_t deviceCfg = {
        .validParams = PMIC_DIAG_EXIT_MASK_VALID,
        .diagExitMask = (bool)true
    };

    int32_t status = Pmic_setDeviceCfg(NULL, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    status = Pmic_setDeviceCfg(&pmicHandle, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    status = Pmic_setDeviceCfg(NULL, &deviceCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

void test_faultHandling_getDeviceCfg_nullParams(void)
{
    Pmic_DeviceCfg_t deviceCfg = {.validParams = PMIC_DIAG_EXIT_MASK_VALID};

    int32_t status = Pmic_getDeviceCfg(NULL, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    status = Pmic_getDeviceCfg(&pmicHandle, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    status = Pmic_getDeviceCfg(NULL, &deviceCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

void test_functionality_setGetDeviceCfg_pwrDwnThr(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_DeviceCfg_t expDeviceCfg = {.validParams = PMIC_PWR_DWN_THR_VALID};
    Pmic_DeviceCfg_t actDeviceCfg = {.validParams = PMIC_PWR_DWN_THR_VALID};

    // Test Pmic_setDeviceCfg() API error handling for out of bounds value
    expDeviceCfg.pwrDwnThr = PMIC_PWR_DWN_THR_MAX + 1U;
    status = Pmic_setDeviceCfg(&pmicHandle, &expDeviceCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

    // Set power down threshold to its maximum value
    expDeviceCfg.pwrDwnThr = PMIC_PWR_DWN_THR_MAX;
    status = Pmic_setDeviceCfg(&pmicHandle, &expDeviceCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get actual power down threshold and compare expected vs. actual values
    status = Pmic_getDeviceCfg(&pmicHandle, &actDeviceCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(expDeviceCfg.pwrDwnThr, actDeviceCfg.pwrDwnThr);
}

void test_functionality_setGetDeviceCfg_safeLockThr(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_DeviceCfg_t expDeviceCfg = {.validParams = PMIC_SAFE_LOCK_THR_VALID};
    Pmic_DeviceCfg_t actDeviceCfg = {.validParams = PMIC_SAFE_LOCK_THR_VALID};

    // Test Pmic_setDeviceCfg() API error handling for out of bounds value
    expDeviceCfg.safeLockThr = PMIC_SAFE_LOCK_THR_MAX + 1U;
    status = Pmic_setDeviceCfg(&pmicHandle, &expDeviceCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

    // Set safe lock threshold to its maximum value
    expDeviceCfg.safeLockThr = PMIC_SAFE_LOCK_THR_MAX;
    status = Pmic_setDeviceCfg(&pmicHandle, &expDeviceCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get actual safe lock threshold and compare expected vs. actual values
    status = Pmic_getDeviceCfg(&pmicHandle, &actDeviceCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(expDeviceCfg.safeLockThr, actDeviceCfg.safeLockThr);
}

void test_functionality_setGetDeviceCfg_devErrCnt(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_DeviceCfg_t expDeviceCfg = {.validParams = PMIC_DEV_ERR_CNT_VALID};
    Pmic_DeviceCfg_t actDeviceCfg = {.validParams = PMIC_DEV_ERR_CNT_VALID};

    // Test Pmic_setDeviceCfg() API error handling for out of bounds value
    expDeviceCfg.devErrCnt = PMIC_DEV_ERR_CNT_MAX + 1U;
    status = Pmic_setDeviceCfg(&pmicHandle, &expDeviceCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

    // Set device error count to zero
    expDeviceCfg.devErrCnt = 0U;
    status = Pmic_setDeviceCfg(&pmicHandle, &expDeviceCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get actual device error count and compare expected vs. actual values
    status = Pmic_getDeviceCfg(&pmicHandle, &actDeviceCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(expDeviceCfg.devErrCnt, actDeviceCfg.devErrCnt);
}

void test_functionality_setGetDeviceCfg_safeTimeoutDuration()
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_DeviceCfg_t expDeviceCfg = {.validParams = PMIC_SAFE_TIMEOUT_DURATION_VALID};
    Pmic_DeviceCfg_t actDeviceCfg = {.validParams = PMIC_SAFE_TIMEOUT_DURATION_VALID};

    // Test Pmic_setDeviceCfg() API error handling for out of bounds value
    expDeviceCfg.safeTimeoutDuration = PMIC_SAFE_TO_DURATION_MAX + 1U;
    status = Pmic_setDeviceCfg(&pmicHandle, &expDeviceCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

    // Set safe timeout duration to its maximum value
    expDeviceCfg.safeTimeoutDuration = PMIC_SAFE_TO_DURATION_MAX;
    status = Pmic_setDeviceCfg(&pmicHandle, &expDeviceCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get actual safe timeout duration and compare expected vs. actual values
    status = Pmic_getDeviceCfg(&pmicHandle, &actDeviceCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(expDeviceCfg.safeTimeoutDuration, actDeviceCfg.safeTimeoutDuration);
}

void test_functionality_setGetDeviceCfg_disableSafeLockTimeout(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_DeviceCfg_t expDeviceCfg = {.validParams = PMIC_DISABLE_SAFE_LOCK_TIMEOUT_VALID};
    Pmic_DeviceCfg_t actDeviceCfg = {.validParams = PMIC_DISABLE_SAFE_LOCK_TIMEOUT_VALID};

    // Disable safe lock timeout
    expDeviceCfg.disableSafeLockTimeout = (bool)true;
    status = Pmic_setDeviceCfg(&pmicHandle, &expDeviceCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get actual safe lock timeout enable/disable status and compare expected vs. actual values
    status = Pmic_getDeviceCfg(&pmicHandle, &actDeviceCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(expDeviceCfg.disableSafeLockTimeout, actDeviceCfg.disableSafeLockTimeout);

    // Enable safe lock timeout
    expDeviceCfg.disableSafeLockTimeout = (bool)false;
    status = Pmic_setDeviceCfg(&pmicHandle, &expDeviceCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get actual safe lock timeout enable/disable status and compare expected vs. actual values
    status = Pmic_getDeviceCfg(&pmicHandle, &actDeviceCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(expDeviceCfg.disableSafeLockTimeout, actDeviceCfg.disableSafeLockTimeout);
}

void test_functionality_setGetDeviceCfg_disableAutoBIST(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_DeviceCfg_t expDeviceCfg = {.validParams = PMIC_DISABLE_AUTO_BIST_VALID};
    Pmic_DeviceCfg_t actDeviceCfg = {.validParams = PMIC_DISABLE_AUTO_BIST_VALID};

    // Disable automatic BIST
    expDeviceCfg.disableAutoBIST = (bool)true;
    status = Pmic_setDeviceCfg(&pmicHandle, &expDeviceCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get actual automatic BIST enable/disable status and compare expected vs. actual values
    status = Pmic_getDeviceCfg(&pmicHandle, &actDeviceCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(expDeviceCfg.disableAutoBIST, actDeviceCfg.disableAutoBIST);

    // Enable automatic BIST
    expDeviceCfg.disableAutoBIST = (bool)false;
    status = Pmic_setDeviceCfg(&pmicHandle, &expDeviceCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get actual automatic BIST enable/disable status and compare expected vs. actual values
    status = Pmic_getDeviceCfg(&pmicHandle, &actDeviceCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(expDeviceCfg.disableAutoBIST, actDeviceCfg.disableAutoBIST);
}

void test_functionality_setGetDeviceCfg_enableDrv(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_DeviceCfg_t expDeviceCfg = {.validParams = PMIC_ENABLE_DRV_VALID};
    Pmic_DeviceCfg_t actDeviceCfg = {.validParams = PMIC_ENABLE_DRV_VALID};

    // Enable ENABLE_DRV output
    expDeviceCfg.enableDrv = (bool)true;
    status = Pmic_setDeviceCfg(&pmicHandle, &expDeviceCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get actual ENABLE_DRV output enable/disable status and compare expected vs. actual values
    status = Pmic_getDeviceCfg(&pmicHandle, &actDeviceCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(expDeviceCfg.enableDrv, actDeviceCfg.enableDrv);

    // Disable ENABLE_DRV output
    expDeviceCfg.enableDrv = (bool)false;
    status = Pmic_setDeviceCfg(&pmicHandle, &expDeviceCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get actual ENABLE_DRV output enable/disable status and compare expected vs. actual values
    status = Pmic_getDeviceCfg(&pmicHandle, &actDeviceCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(expDeviceCfg.enableDrv, actDeviceCfg.enableDrv);
}

void test_functionality_setGetDeviceCfg_diagExitMask(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_DeviceCfg_t expDeviceCfg = {.validParams = PMIC_ENABLE_DRV_VALID};
    Pmic_DeviceCfg_t actDeviceCfg = {.validParams = PMIC_ENABLE_DRV_VALID};

    // Unmask Diagnostic Exit
    expDeviceCfg.diagExitMask = (bool)false;
    status = Pmic_setDeviceCfg(&pmicHandle, &expDeviceCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get actual Diagnostic Exit Mask status and compare expected vs. actual values
    status = Pmic_getDeviceCfg(&pmicHandle, &actDeviceCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(expDeviceCfg.diagExitMask, actDeviceCfg.diagExitMask);

    // Mask Diagnostic Exit
    expDeviceCfg.diagExitMask = (bool)true;
    status = Pmic_setDeviceCfg(&pmicHandle, &expDeviceCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get actual Diagnostic Exit Mask status and compare expected vs. actual values
    status = Pmic_getDeviceCfg(&pmicHandle, &actDeviceCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(expDeviceCfg.diagExitMask, actDeviceCfg.diagExitMask);
}

void test_functionality_setGetDeviceCfg_diagExit(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_DeviceCfg_t expDeviceCfg = {.validParams = PMIC_ENABLE_DRV_VALID};
    Pmic_DeviceCfg_t actDeviceCfg = {.validParams = PMIC_ENABLE_DRV_VALID};

    // Enable Diagnostic Exit
    expDeviceCfg.diagExit = (bool)true;
    status = Pmic_setDeviceCfg(&pmicHandle, &expDeviceCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get actual Diagnostic Exit enable/disable status and compare expected vs. actual values
    status = Pmic_getDeviceCfg(&pmicHandle, &actDeviceCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(expDeviceCfg.diagExit, actDeviceCfg.diagExit);

    // Disable Diagnostic Exit
    expDeviceCfg.diagExit = (bool)false;
    status = Pmic_setDeviceCfg(&pmicHandle, &expDeviceCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get actual Diagnostic Exit enable/disable status and compare expected vs. actual values
    status = Pmic_getDeviceCfg(&pmicHandle, &actDeviceCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(expDeviceCfg.diagExit, actDeviceCfg.diagExit);
}
