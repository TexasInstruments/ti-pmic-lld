/**
 * @file irq_test.c
 *
 * @brief Source file for PMIC LLD IRQ module testing.
 */
#include "irq_test.h"

#define RUN_IRQ_TESTS()     RUN_TEST(test_faultHandling_irqGetStat_nullParams);     \
                            RUN_TEST(test_faultHandling_irqGetNextFlag_nullParams); \
                            RUN_TEST(test_faultHandling_irqGetFlag_nullParams);     \
                            RUN_TEST(test_faultHandling_irqGetFlag_invalidIrqNum);  \
                            RUN_TEST(test_functionality_irqGetStat_irqGetNextFlag)

Pmic_CoreHandle_t pmicHandle;

void irqTest(void *args)
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

    DebugP_log("\r\nWelcome to PMIC LLD IRQ module testing\r\n\r\n");

    status = Pmic_init(&pmicCfg, &pmicHandle);

    if (status == PMIC_ST_SUCCESS)
    {
        DebugP_log("Running PMIC IRQ tests...\r\n");
        UNITY_BEGIN();
        RUN_IRQ_TESTS();
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

void test_faultHandling_irqGetStat_nullParams(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_IrqStat_t irqStat;

    status = Pmic_irqGetStat(NULL, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    status = Pmic_irqGetStat(&pmicHandle, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    status = Pmic_irqGetStat(NULL, &irqStat);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

void test_faultHandling_irqGetNextFlag_nullParams(void)
{
    int32_t status = PMIC_ST_ERR_NULL_PARAM;
    Pmic_IrqStat_t irqStat;
    uint8_t irqNum = 0U;

    status = Pmic_irqGetNextFlag(NULL, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    status = Pmic_irqGetNextFlag(&irqStat, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    status = Pmic_irqGetNextFlag(NULL, &irqNum);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

void test_faultHandling_irqGetFlag_nullParams(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    bool flag = (bool)false;

    status = Pmic_irqGetFlag(NULL, PMIC_IRQ_SPI_CMD_ERR, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    status = Pmic_irqGetFlag(&pmicHandle, PMIC_IRQ_SPI_CMD_ERR, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    status = Pmic_irqGetFlag(NULL, PMIC_IRQ_SPI_CMD_ERR, &flag);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

void test_faultHandling_irqGetFlag_invalidIrqNum(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    bool flag = (bool)false;

    status = Pmic_irqGetFlag(&pmicHandle, PMIC_IRQ_MAX + 1U, &flag);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);
}

void test_functionality_irqGetStat_irqGetNextFlag(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_IrqStat_t irqStat;
    uint8_t irqNum = 0U;

    status = Pmic_irqGetStat(&pmicHandle, &irqStat);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    for (uint8_t i = 0U; i < PMIC_IRQ_NUM; i++)
    {
        status = Pmic_irqGetNextFlag(&irqStat, &irqNum);
        TEST_ASSERT_LESS_OR_EQUAL(PMIC_IRQ_MAX, irqNum);

        if (status == PMIC_ST_SUCCESS)
        {
            DebugP_log("\tIRQ found: %d\r\n", irqNum);
        }
        else if (status == PMIC_ST_WARN_NO_IRQ_REMAINING)
        {
            break;
        }
        else
        {
            TEST_FAIL();
        }
    }
}

// Unity testing framework calls this API before each test.
void setUp(void)
{
}

// Unity testing framework calls this API after each test.
void tearDown(void)
{
}
