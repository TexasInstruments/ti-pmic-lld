/**
 * @file irq_test.c
 *
 * @brief Source file for PMIC LLD IRQ module testing.
 */
#include "irq_test.h"

#define RUN_IRQ_TESTS()     RUN_TEST(test_faultHandling_irqSetMasks_invalidIrqNum);             \
                            RUN_TEST(test_faultHandling_irqSetMasks_invalidNumIrqMasks);        \
                            RUN_TEST(test_faultHandling_irqSetMasks_nullParams);                \
                            RUN_TEST(test_faultHandling_irqGetMasks_invalidIrqNum);             \
                            RUN_TEST(test_faultHandling_irqGetMasks_invalidNumIrqMasks);        \
                            RUN_TEST(test_faultHandling_irqGetMasks_nullParams);                \
                            RUN_TEST(test_faultHandling_irqSetGetMask_LDO_SC_INT);              \
                            RUN_TEST(test_faultHandling_irqSetGetMask_BUCK3_SC_INT);            \
                            RUN_TEST(test_faultHandling_irqSetGetMask_BUCK2_SC_INT);            \
                            RUN_TEST(test_faultHandling_irqSetGetMask_BUCK1_SC_INT);            \
                            RUN_TEST(test_functionality_irqSetGetMask_BUCK2_OVP_INT);           \
                            RUN_TEST(test_functionality_irqSetGetMask_BUCK2_UV_INT);            \
                            RUN_TEST(test_functionality_irqSetGetMask_BUCK2_OV_INT);            \
                            RUN_TEST(test_functionality_irqSetGetMask_BUCK1_OVP_INT);           \
                            RUN_TEST(test_functionality_irqSetGetMask_BUCK1_UV_INT);            \
                            RUN_TEST(test_functionality_irqSetGetMask_BUCK1_OV_INT);            \
                            RUN_TEST(test_functionality_irqSetGetMask_LDO_OVP_INT);             \
                            RUN_TEST(test_functionality_irqSetGetMask_LDO_UV_INT);              \
                            RUN_TEST(test_functionality_irqSetGetMask_LDO_OV_INT);              \
                            RUN_TEST(test_functionality_irqSetGetMask_BUCK3_OVP_INT);           \
                            RUN_TEST(test_functionality_irqSetGetMask_BUCK3_UV_INT);            \
                            RUN_TEST(test_functionality_irqSetGetMask_BUCK3_OV_INT);            \
                            RUN_TEST(test_functionality_irqSetGetMask_TWARN_INT);               \
                            RUN_TEST(test_functionality_irqSetGetMask_B1_PVIN_UVLO_INT);        \
                            RUN_TEST(test_functionality_irqSetGetMask_BUCKS_VSET_ERR_INT);      \
                            RUN_TEST(test_faultHandling_irqSetGetMask_CFG_NVM_VERIFY_ERR);      \
                            RUN_TEST(test_faultHandling_irqSetGetMask_CFG_NVM_VERIFY_DONE);     \
                            RUN_TEST(test_faultHandling_irqSetGetMask_CFG_NVM_PRG_DONE);        \
                            RUN_TEST(test_functionality_irqSetGetMask_ABIST_FAIL_INT);          \
                            RUN_TEST(test_functionality_irqSetGetMask_ABIST_DONE_INT);          \
                            RUN_TEST(test_functionality_irqSetGetMask_GPO_READBACK_INT);        \
                            RUN_TEST(test_functionality_irqSetGetMask_NINT_READBACK_INT);       \
                            RUN_TEST(test_functionality_irqSetGetMask_CONFIG_CRC_INT);          \
                            RUN_TEST(test_functionality_irqSetGetMask_TRIM_TEST_CRC_INT);       \
                            RUN_TEST(test_faultHandling_irqSetGetMask_RECOV_CNT_INT);           \
                            RUN_TEST(test_faultHandling_irqSetGetMask_TSD_IMM_INT);             \
                            RUN_TEST(test_faultHandling_irqSetGetMask_WD_FIRST_NOK_INT);        \
                            RUN_TEST(test_faultHandling_irqSetGetMask_WAIT_FOR_PWRCYCLE_INT);   \
                            RUN_TEST(test_faultHandling_irqSetGetMask_WARM_RESET_INT);          \
                            RUN_TEST(test_faultHandling_irqSetGetMask_ORD_SHUTDOWN_INT);        \
                            RUN_TEST(test_faultHandling_irqSetGetMask_IMM_SHUTDOWN_INT);        \
                            RUN_TEST(test_functionality_irqSetGetMask_MCU_COMM_ERR_INT);        \
                            RUN_TEST(test_functionality_irqSetGetMask_COMM_ADR_ERR_INT);        \
                            RUN_TEST(test_functionality_irqSetGetMask_COMM_CRC_ERR_INT);        \
                            RUN_TEST(test_functionality_irqSetGetMask_ESM_MCU_RST_INT);         \
                            RUN_TEST(test_functionality_irqSetGetMask_ESM_MCU_FAIL_INT);        \
                            RUN_TEST(test_functionality_irqSetGetMask_ESM_MCU_PIN_INT);         \
                            RUN_TEST(test_faultHandling_irqSetGetMask_WD_RST_INT);              \
                            RUN_TEST(test_faultHandling_irqSetGetMask_WD_FAIL_INT);             \
                            RUN_TEST(test_faultHandling_irqSetGetMask_WD_LONGWIN_TIMEOUT_INT);  \
                            RUN_TEST(test_functionality_irqSetGetMask_multipleMasks_esm);       \
                            RUN_TEST(test_functionality_irqSetGetMask_multipleMasks_esmBuck);   \
                            RUN_TEST(test_faultHandling_irqGetStat_nullParams);                 \
                            RUN_TEST(test_faultHandling_irqGetNextFlag_nullParams);             \
                            RUN_TEST(test_faultHandling_irqGetFlag_invalidIrqNum);              \
                            RUN_TEST(test_faultHandling_irqGetFlag_nullParams);                 \
                            RUN_TEST(test_functionality_irqGetFlags);                           \
                            RUN_TEST(test_faultHandling_irqClrFlag_invalidIrqNum);              \
                            RUN_TEST(test_faultHandling_irqClrFlag_nullParams);                 \
                            RUN_TEST(test_functionality_irqClrFlags)

Pmic_CoreHandle_t pmicHandle;

void irqTest(void *args)
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

void test_faultHandling_irqSetMasks_invalidIrqNum(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_IrqMask_t irqMask = {.irqNum = PMIC_IRQ_MAX + 1U, .mask = (bool)true};

    status = Pmic_irqSetMasks(&pmicHandle, 1U, &irqMask);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);
}

void test_faultHandling_irqSetMasks_invalidNumIrqMasks(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_IrqMask_t irqMask = {.irqNum = PMIC_TWARN_INT, .mask = (bool)true};

    status = Pmic_irqSetMasks(&pmicHandle, 0U, &irqMask);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);
}

void test_faultHandling_irqSetMasks_nullParams(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_IrqMask_t irqMask = {.irqNum = PMIC_TWARN_INT, .mask = (bool)true};

    status = Pmic_irqSetMasks(&pmicHandle, 1U, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    status = Pmic_irqSetMasks(NULL, 1U, &irqMask);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    status = Pmic_irqSetMasks(NULL, 1U, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

void test_faultHandling_irqGetMasks_invalidIrqNum(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_IrqMask_t irqMask = {.irqNum = PMIC_IRQ_MAX + 1U};

    status = Pmic_irqGetMasks(&pmicHandle, 1U, &irqMask);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);
}

void test_faultHandling_irqGetMasks_invalidNumIrqMasks(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_IrqMask_t irqMask = {.irqNum = PMIC_TWARN_INT};

    status = Pmic_irqGetMasks(&pmicHandle, 0U, &irqMask);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);
}

void test_faultHandling_irqGetMasks_nullParams(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_IrqMask_t irqMask = {.irqNum = PMIC_TWARN_INT};

    status = Pmic_irqGetMasks(NULL, 1U, &irqMask);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    status = Pmic_irqGetMasks(&pmicHandle, 1U, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    status = Pmic_irqGetMasks(NULL, 1U, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

static void test_setGetIrqMask(uint8_t irqNum)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_IrqMask_t expIrqMask = {.irqNum = irqNum}, actIrqMask = {.irqNum = irqNum};

    // Unmask IRQ
    expIrqMask.mask = (bool)false;
    status = Pmic_irqSetMasks(&pmicHandle, 1U, &expIrqMask);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get actual IRQ mask configuration and compare expected vs. actual values
    status = Pmic_irqGetMasks(&pmicHandle, 1U, &actIrqMask);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(expIrqMask.mask, actIrqMask.mask);

    // Mask IRQ
    expIrqMask.mask = (bool)true;
    status = Pmic_irqSetMasks(&pmicHandle, 1U, &expIrqMask);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get actual IRQ mask configuration and compare expected vs. actual values
    status = Pmic_irqGetMasks(&pmicHandle, 1U, &actIrqMask);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(expIrqMask.mask, actIrqMask.mask);
}

static void test_setGetIrqMaskFault(uint8_t irqNum)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_IrqMask_t expIrqMask = {.irqNum = irqNum}, actIrqMask = {.irqNum = irqNum};

    // Test error handling for setting mask configuration of NMI
    expIrqMask.mask = (bool)true;
    status = Pmic_irqSetMasks(&pmicHandle, 1U, &expIrqMask);
    TEST_ASSERT_EQUAL(PMIC_ST_WARN_NON_MASKABLE_INT, status);

    // Test error handling for getting mask configuration of NMI
    status = Pmic_irqGetMasks(&pmicHandle, 1U, &actIrqMask);
    TEST_ASSERT_EQUAL(PMIC_ST_WARN_NON_MASKABLE_INT, status);
}

void test_faultHandling_irqSetGetMask_LDO_SC_INT(void)
{
    test_setGetIrqMaskFault(PMIC_LDO_SC_INT);
}

void test_faultHandling_irqSetGetMask_BUCK3_SC_INT(void)
{
    test_setGetIrqMaskFault(PMIC_BUCK3_SC_INT);
}

void test_faultHandling_irqSetGetMask_BUCK2_SC_INT(void)
{
    test_setGetIrqMaskFault(PMIC_BUCK2_SC_INT);
}

void test_faultHandling_irqSetGetMask_BUCK1_SC_INT(void)
{
    test_setGetIrqMaskFault(PMIC_BUCK1_SC_INT);
}

void test_functionality_irqSetGetMask_BUCK2_OVP_INT(void)
{
    test_setGetIrqMask(PMIC_BUCK2_OVP_INT);
}

void test_functionality_irqSetGetMask_BUCK2_UV_INT(void)
{
    test_setGetIrqMask(PMIC_BUCK2_UV_INT);
}

void test_functionality_irqSetGetMask_BUCK2_OV_INT(void)
{
    test_setGetIrqMask(PMIC_BUCK2_OV_INT);
}

void test_functionality_irqSetGetMask_BUCK1_OVP_INT(void)
{
    test_setGetIrqMask(PMIC_BUCK1_OVP_INT);
}

void test_functionality_irqSetGetMask_BUCK1_UV_INT(void)
{
    test_setGetIrqMask(PMIC_BUCK1_UV_INT);
}

void test_functionality_irqSetGetMask_BUCK1_OV_INT(void)
{
    test_setGetIrqMask(PMIC_BUCK1_OV_INT);
}

void test_functionality_irqSetGetMask_LDO_OVP_INT(void)
{
    test_setGetIrqMask(PMIC_LDO_OVP_INT);
}

void test_functionality_irqSetGetMask_LDO_UV_INT(void)
{
    test_setGetIrqMask(PMIC_LDO_UV_INT);
}

void test_functionality_irqSetGetMask_LDO_OV_INT(void)
{
    test_setGetIrqMask(PMIC_LDO_OV_INT);
}

void test_functionality_irqSetGetMask_BUCK3_OVP_INT(void)
{
    test_setGetIrqMask(PMIC_BUCK3_OVP_INT);
}

void test_functionality_irqSetGetMask_BUCK3_UV_INT(void)
{
    test_setGetIrqMask(PMIC_BUCK3_UV_INT);
}

void test_functionality_irqSetGetMask_BUCK3_OV_INT(void)
{
    test_setGetIrqMask(PMIC_BUCK3_OV_INT);
}

void test_functionality_irqSetGetMask_TWARN_INT(void)
{
    test_setGetIrqMask(PMIC_TWARN_INT);
}

void test_functionality_irqSetGetMask_B1_PVIN_UVLO_INT(void)
{
    test_setGetIrqMask(PMIC_B1_PVIN_UVLO_INT);
}

void test_functionality_irqSetGetMask_BUCKS_VSET_ERR_INT(void)
{
    test_setGetIrqMask(PMIC_BUCKS_VSET_ERR_INT);
}

void test_faultHandling_irqSetGetMask_CFG_NVM_VERIFY_ERR(void)
{
    test_setGetIrqMaskFault(PMIC_CFG_NVM_VERIFY_ERR);
}

void test_faultHandling_irqSetGetMask_CFG_NVM_VERIFY_DONE(void)
{
    test_setGetIrqMaskFault(PMIC_CFG_NVM_VERIFY_DONE);
}

void test_faultHandling_irqSetGetMask_CFG_NVM_PRG_DONE(void)
{
    test_setGetIrqMaskFault(PMIC_CFG_NVM_PRG_DONE);
}

void test_functionality_irqSetGetMask_ABIST_FAIL_INT(void)
{
    test_setGetIrqMask(PMIC_ABIST_FAIL_INT);
}

void test_functionality_irqSetGetMask_ABIST_DONE_INT(void)
{
    test_setGetIrqMask(PMIC_ABIST_DONE_INT);
}

void test_functionality_irqSetGetMask_GPO_READBACK_INT(void)
{
    test_setGetIrqMask(PMIC_GPO_READBACK_INT);
}

void test_functionality_irqSetGetMask_NINT_READBACK_INT(void)
{
    test_setGetIrqMask(PMIC_NINT_READBACK_INT);
}

void test_functionality_irqSetGetMask_CONFIG_CRC_INT(void)
{
    test_setGetIrqMask(PMIC_CONFIG_CRC_INT);
}

void test_functionality_irqSetGetMask_TRIM_TEST_CRC_INT(void)
{
    test_setGetIrqMask(PMIC_TRIM_TEST_CRC_INT);
}

void test_faultHandling_irqSetGetMask_RECOV_CNT_INT(void)
{
    test_setGetIrqMaskFault(PMIC_RECOV_CNT_INT);
}

void test_faultHandling_irqSetGetMask_TSD_IMM_INT(void)
{
    test_setGetIrqMaskFault(PMIC_TSD_IMM_INT);
}

void test_faultHandling_irqSetGetMask_WD_FIRST_NOK_INT(void)
{
    test_setGetIrqMaskFault(PMIC_WD_FIRST_NOK_INT);
}

void test_faultHandling_irqSetGetMask_WAIT_FOR_PWRCYCLE_INT(void)
{
    test_setGetIrqMaskFault(PMIC_WAIT_FOR_PWRCYCLE_INT);
}

void test_faultHandling_irqSetGetMask_WARM_RESET_INT(void)
{
    test_setGetIrqMaskFault(PMIC_WARM_RESET_INT);
}

void test_faultHandling_irqSetGetMask_ORD_SHUTDOWN_INT(void)
{
    test_setGetIrqMaskFault(PMIC_ORD_SHUTDOWN_INT);
}

void test_faultHandling_irqSetGetMask_IMM_SHUTDOWN_INT(void)
{
    test_setGetIrqMaskFault(PMIC_IMM_SHUTDOWN_INT);
}

void test_functionality_irqSetGetMask_MCU_COMM_ERR_INT(void)
{
    test_setGetIrqMask(PMIC_MCU_COMM_ERR_INT);
}

void test_functionality_irqSetGetMask_COMM_ADR_ERR_INT(void)
{
    test_setGetIrqMask(PMIC_COMM_ADR_ERR_INT);
}

void test_functionality_irqSetGetMask_COMM_CRC_ERR_INT(void)
{
    test_setGetIrqMask(PMIC_COMM_CRC_ERR_INT);
}

void test_functionality_irqSetGetMask_ESM_MCU_RST_INT(void)
{
    test_setGetIrqMask(PMIC_ESM_MCU_RST_INT);
}

void test_functionality_irqSetGetMask_ESM_MCU_FAIL_INT(void)
{
    test_setGetIrqMask(PMIC_ESM_MCU_FAIL_INT);
}

void test_functionality_irqSetGetMask_ESM_MCU_PIN_INT(void)
{
    test_setGetIrqMask(PMIC_ESM_MCU_PIN_INT);
}

void test_faultHandling_irqSetGetMask_WD_RST_INT(void)
{
    test_setGetIrqMaskFault(PMIC_WD_RST_INT);
}

void test_faultHandling_irqSetGetMask_WD_FAIL_INT(void)
{
    test_setGetIrqMaskFault(PMIC_WD_FAIL_INT);
}

void test_faultHandling_irqSetGetMask_WD_LONGWIN_TIMEOUT_INT(void)
{
    test_setGetIrqMaskFault(PMIC_WD_LONGWIN_TIMEOUT_INT);
}

void test_functionality_irqSetGetMask_multipleMasks_esm(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_IrqMask_t expIrqMasks[] = {
        {.irqNum = PMIC_ESM_MCU_RST_INT},
        {.irqNum = PMIC_ESM_MCU_FAIL_INT},
        {.irqNum = PMIC_ESM_MCU_PIN_INT}
    };
    Pmic_IrqMask_t actIrqMasks[] = {
        {.irqNum = PMIC_ESM_MCU_RST_INT},
        {.irqNum = PMIC_ESM_MCU_FAIL_INT},
        {.irqNum = PMIC_ESM_MCU_PIN_INT}
    };

    // Mask ESM interrupts
    expIrqMasks[0U].mask = (bool)true;
    expIrqMasks[1U].mask = (bool)true;
    expIrqMasks[2U].mask = (bool)true;
    status = Pmic_irqSetMasks(&pmicHandle, 3U, expIrqMasks);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get actual ESM mask configurations and compare expected vs. actual values
    status = Pmic_irqGetMasks(&pmicHandle, 3U, actIrqMasks);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(expIrqMasks[0U].mask, actIrqMasks[0U].mask);
    TEST_ASSERT_EQUAL(expIrqMasks[1U].mask, actIrqMasks[1U].mask);
    TEST_ASSERT_EQUAL(expIrqMasks[2U].mask, actIrqMasks[2U].mask);

    // Unmask ESM interrupts
    expIrqMasks[0U].mask = (bool)false;
    expIrqMasks[1U].mask = (bool)false;
    expIrqMasks[2U].mask = (bool)false;
    status = Pmic_irqSetMasks(&pmicHandle, 3U, expIrqMasks);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get actual ESM mask configurations and compare expected vs. actual values
    status = Pmic_irqGetMasks(&pmicHandle, 3U, actIrqMasks);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(expIrqMasks[0U].mask, actIrqMasks[0U].mask);
    TEST_ASSERT_EQUAL(expIrqMasks[1U].mask, actIrqMasks[1U].mask);
    TEST_ASSERT_EQUAL(expIrqMasks[2U].mask, actIrqMasks[2U].mask);
}

void test_functionality_irqSetGetMask_multipleMasks_esmBuck(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_IrqMask_t expIrqMasks[] = {
        {.irqNum = PMIC_ESM_MCU_RST_INT},
        {.irqNum = PMIC_BUCK3_OVP_INT},
    };
    Pmic_IrqMask_t actIrqMasks[] = {
        {.irqNum = PMIC_ESM_MCU_RST_INT},
        {.irqNum = PMIC_BUCK3_OVP_INT},
    };

    // Mask interrupts
    expIrqMasks[0U].mask = (bool)true;
    expIrqMasks[1U].mask = (bool)true;
    status = Pmic_irqSetMasks(&pmicHandle, 2U, expIrqMasks);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get actual mask configurations and compare expected vs. actual values
    status = Pmic_irqGetMasks(&pmicHandle, 2U, actIrqMasks);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(expIrqMasks[0U].mask, actIrqMasks[0U].mask);
    TEST_ASSERT_EQUAL(expIrqMasks[1U].mask, actIrqMasks[1U].mask);

    // Unmask interrupts
    expIrqMasks[0U].mask = (bool)false;
    expIrqMasks[1U].mask = (bool)false;
    status = Pmic_irqSetMasks(&pmicHandle, 2U, expIrqMasks);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get actual mask configurations and compare expected vs. actual values
    status = Pmic_irqGetMasks(&pmicHandle, 2U, actIrqMasks);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(expIrqMasks[0U].mask, actIrqMasks[0U].mask);
    TEST_ASSERT_EQUAL(expIrqMasks[1U].mask, actIrqMasks[1U].mask);
}

void test_faultHandling_irqGetStat_nullParams(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_IrqStat_t irqStat;

    status = Pmic_irqGetStat(NULL, &irqStat);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    status = Pmic_irqGetStat(&pmicHandle, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    status = Pmic_irqGetStat(NULL, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

void test_faultHandling_irqGetNextFlag_nullParams(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_IrqStat_t irqStat;
    uint8_t irqNum = 0U;

    status = Pmic_irqGetNextFlag(NULL, &irqNum);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    status = Pmic_irqGetNextFlag(&irqStat, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    status = Pmic_irqGetNextFlag(NULL, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

void test_faultHandling_irqGetFlag_invalidIrqNum(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    bool flag = (bool)false;

    status = Pmic_irqGetFlag(&pmicHandle, PMIC_IRQ_MAX + 1U, &flag);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);
}

void test_faultHandling_irqGetFlag_nullParams(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    bool flag = (bool)false;

    status = Pmic_irqGetFlag(NULL, PMIC_TWARN_INT, &flag);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    status = Pmic_irqGetFlag(&pmicHandle, PMIC_TWARN_INT, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    status = Pmic_irqGetFlag(NULL, PMIC_TWARN_INT, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

void test_functionality_irqGetFlags(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t irqNum = 0U, iter = 0U;
    bool flag = (bool)false;
    Pmic_IrqStat_t irqStat;

    // Get status of all PMIC IRQs
    status = Pmic_irqGetStat(&pmicHandle, &irqStat);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // While there are still IRQ statuses remaining...
    while ((status != PMIC_ST_WARN_NO_IRQ_REMAINING) && (iter != UINT8_MAX))
    {
        // Get next IRQ via Pmic_irqGetNextFlag
        status = Pmic_irqGetNextFlag(&irqStat, &irqNum);

        if (status == PMIC_ST_SUCCESS)
        {
            // Validate IRQ status via Pmic_irqGetFlag
            status = Pmic_irqGetFlag(&pmicHandle, irqNum, &flag);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
            TEST_ASSERT_EQUAL((bool)true, flag);

            flag = (bool)false;
        }
        else if (status != PMIC_ST_WARN_NO_IRQ_REMAINING)
        {
            TEST_FAIL();
        }
        else
        {
            /* Empty */
        }

        iter++;
    }

    TEST_ASSERT_NOT_EQUAL(UINT8_MAX, iter);
}

void test_faultHandling_irqClrFlag_invalidIrqNum(void)
{
    int32_t status = Pmic_irqClrFlag(&pmicHandle, PMIC_IRQ_MAX + 1U);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);
}

void test_faultHandling_irqClrFlag_nullParams(void)
{
    int32_t status = Pmic_irqClrFlag(NULL, PMIC_TWARN_INT);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

void test_functionality_irqClrFlags(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t irqNum = 0U, iter = 0U;
    bool flag = (bool)false;
    Pmic_IrqStat_t irqStat;

    // Get status of all PMIC IRQs
    status = Pmic_irqGetStat(&pmicHandle, &irqStat);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // While there are still IRQ statuses remaining...
    while ((status != PMIC_ST_WARN_NO_IRQ_REMAINING) && (iter != UINT8_MAX))
    {
        // Get next IRQ
        status = Pmic_irqGetNextFlag(&irqStat, &irqNum);

        if (status == PMIC_ST_SUCCESS)
        {
            // Check whether IRQ is set
            status = Pmic_irqGetFlag(&pmicHandle, irqNum, &flag);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
            TEST_ASSERT_EQUAL((bool)true, flag);

            // Clear IRQ
            status = Pmic_irqClrFlag(&pmicHandle, irqNum);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

            // Check whether IRQ is cleared
            status = Pmic_irqGetFlag(&pmicHandle, irqNum, &flag);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
            TEST_ASSERT_EQUAL((bool)false, flag);
        }
        else if (status != PMIC_ST_WARN_NO_IRQ_REMAINING)
        {
            TEST_FAIL();
        }
        else
        {
            /* Empty */
        }

        iter++;
    }

    TEST_ASSERT_NOT_EQUAL(UINT8_MAX, iter);
}

/**
 * @brief Unity testing framework calls this API before each test.
 */
void setUp(void)
{
}

/**
 * @brief Unity testing framework calls this API after each test.
 */
void tearDown(void)
{
}
