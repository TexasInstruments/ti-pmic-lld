/**
 * @file misc_test.c
 *
 * @brief Source file for PMIC LLD miscellaneous testing.
 *
 * @details Testing includes
 *
 * 1. PMIC LLD initialization and deinitialization
 */
#include "misc_test.h"

#define RUN_MISC_TESTS()    RUN_TEST(test_faultHandling_pmicInit_nullParams);   \
                            RUN_TEST(test_faultHandling_pmicInit_invalidCfg);   \
                            RUN_TEST(test_functionality_pmicInit);              \
                            RUN_TEST(test_faultHandling_pmicDeinit_nullParams); \
                            RUN_TEST(test_functionality_pmicDeinit)

Pmic_CoreHandle_t pmicHandle;

void miscTest(void *args)
{
    Drivers_open();
    Board_driversOpen();

    DebugP_log("\r\nWelcome to PMIC LLD miscellaneous testing\r\n\r\n");

    DebugP_log("Running PMIC miscellaneous tests...\r\n");
    UNITY_BEGIN();
    RUN_MISC_TESTS();
    UNITY_END();

    Board_driversClose();
    Drivers_close();
}

void test_faultHandling_pmicInit_nullParams(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_CoreCfg_t pmicCfg = {
        .validParams = PMIC_CRIT_SEC_START_VALID,
        .critSecStart = &app_critSecStart
    };

    status = Pmic_init(NULL, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    status = Pmic_init(NULL, &pmicHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    status = Pmic_init(&pmicCfg, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

void test_faultHandling_pmicInit_invalidCfg(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_CoreCfg_t pmicCfg;

    // No valid parameters
    pmicCfg.validParams = 0U;
    status = Pmic_init(&pmicCfg, &pmicHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

    // Invalid communication handle
    pmicCfg.validParams = PMIC_COMM_HANDLE_VALID;
    pmicCfg.commHandle = NULL;
    status = Pmic_init(&pmicCfg, &pmicHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    // Invalid I/O transfer API
    pmicCfg.validParams = PMIC_IO_TRANSFER_VALID;
    pmicCfg.ioTransfer = NULL;
    status = Pmic_init(&pmicCfg, &pmicHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    // Invalid critical section start API
    pmicCfg.validParams = PMIC_CRIT_SEC_START_VALID;
    pmicCfg.critSecStart = NULL;
    status = Pmic_init(&pmicCfg, &pmicHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    // Invalid critical section stop API
    pmicCfg.validParams = PMIC_CRIT_SEC_STOP_VALID;
    pmicCfg.critSecStop = NULL;
    status = Pmic_init(&pmicCfg, &pmicHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    // Invalid IRQ response API
    pmicCfg.validParams = PMIC_IRQ_RESPONSE_VALID;
    pmicCfg.irqResponse = NULL;
    status = Pmic_init(&pmicCfg, &pmicHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

}

void test_functionality_pmicInit(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_CoreCfg_t pmicCfg = {
        .validParams = (PMIC_IO_TRANSFER_VALID | PMIC_CRIT_SEC_START_VALID |
            PMIC_CRIT_SEC_STOP_VALID | PMIC_IRQ_RESPONSE_VALID),
        .ioTransfer = &app_spiTransfer,
        .critSecStart = &app_critSecStart,
        .critSecStop = &app_critSecStop,
        .irqResponse = &app_irqResponse
    };

    status = Pmic_init(&pmicCfg, &pmicHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
}

void test_faultHandling_pmicDeinit_nullParams(void)
{
    int32_t status = Pmic_deinit(NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

void test_functionality_pmicDeinit(void)
{
    // A previous test has already initialized the handle
    int32_t status = Pmic_deinit(&pmicHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Verify the expected PMIC handle values after deinitialization
    TEST_ASSERT_EQUAL(PMIC_DRV_UNINIT, pmicHandle.drvInitStat);
    TEST_ASSERT_EQUAL(0U, pmicHandle.devRev);
    TEST_ASSERT_EQUAL(0U, pmicHandle.devId);
    TEST_ASSERT_EQUAL(NULL, pmicHandle.commHandle);
    TEST_ASSERT_EQUAL(NULL, pmicHandle.ioTransfer);
    TEST_ASSERT_EQUAL(NULL, pmicHandle.critSecStart);
    TEST_ASSERT_EQUAL(NULL, pmicHandle.critSecStop);
    TEST_ASSERT_EQUAL(NULL, pmicHandle.irqResponse);
}

// Unity testing framework calls this API before each test.
void setUp(void)
{
}

// Unity testing framework calls this API after each test.
void tearDown(void)
{
}
