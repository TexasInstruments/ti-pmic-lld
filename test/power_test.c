/**
 * @file power_test.c
 *
 * @brief Source file for PMIC LLD Power module testing.
 */
#include "power_test.h"

#define RUN_POWER_TESTS()   RUN_TEST(test_faultHandling_pwrSetBuckCfg_nullParams);                          \
                            RUN_TEST(test_faultHandling_pwrSetBuckCfg_noValidParams);                       \
                            RUN_TEST(test_faultHandling_pwrSetBuckCfg_invalidResource);                     \
                            RUN_TEST(test_faultHandling_pwrGetBuckCfg_nullParams);                          \
                            RUN_TEST(test_faultHandling_pwrGetBuckCfg_noValidParams);                       \
                            RUN_TEST(test_faultHandling_pwrGetBuckCfg_invalidResource);                     \
                            RUN_TEST(test_functionality_pwrSetGetBuckCfg_enable);                           \
                            RUN_TEST(test_functionality_pwrSetGetBuckCfg_pulldownResistor);                 \
                            RUN_TEST(test_functionality_pwrSetGetBuckCfg_fpwm);                             \
                            RUN_TEST(test_functionality_pwrSetGetBuckCfg_uvThr);                            \
                            RUN_TEST(test_functionality_pwrSetGetBuckCfg_ovThr);                            \
                            RUN_TEST(test_functionality_pwrSetGetBuckCfg_ilimSel);                          \
                            RUN_TEST(test_functionality_pwrSetGetBuckCfg_ovpSel);                           \
                            RUN_TEST(test_functionality_pwrSetGetBuckCfg_ovSel);                            \
                            RUN_TEST(test_functionality_pwrSetGetBuckCfg_uvSel);                            \
                            RUN_TEST(test_functionality_pwrSetGetBuckCfg_scSel);                            \
                            RUN_TEST(test_functionality_pwrSetGetBuckCfg_rvConf);                           \
                            RUN_TEST(test_functionality_pwrSetGetBuckCfg_slewRate);                         \
                            RUN_TEST(test_functionality_pwrSetGetBuckCfg_deglitchSel);                      \
                            RUN_TEST(test_functionality_pwrSetGetBuckCfg_dischargeSel);                     \
                            RUN_TEST(test_functionality_pwrSetGetBuckCfg_ssEn);                             \
                            RUN_TEST(test_functionality_pwrSetGetBuckCfg_ssmSel);                           \
                            RUN_TEST(test_functionality_pwrSetGetBuckCfg_vset);                             \
                            RUN_TEST(test_functionality_pwrSetGetBuckCfg_highSideSlewRate);                 \
                            RUN_TEST(test_functionality_pwrSetGetBuckCfg_vsetActive);                       \
                            RUN_TEST(test_functionality_pwrSetGetBuckCfg_vsetLPwr);                         \
                            RUN_TEST(test_functionality_pwrSetGetBuckCfg_vmonOnly);                         \
                            RUN_TEST(test_faultHandling_pwrSetLdoCfg_nullParams);                           \
                            RUN_TEST(test_faultHandling_pwrSetLdoCfg_noValidParams);                        \
                            RUN_TEST(test_faultHandling_pwrGetLdoCfg_nullParams);                           \
                            RUN_TEST(test_faultHandling_pwrGetLdoCfg_noValidParams);                        \
                            RUN_TEST(test_functionality_pwrSetGetLdoCfg_enable);                            \
                            RUN_TEST(test_functionality_pwrSetGetLdoCfg_mode);                              \
                            RUN_TEST(test_functionality_pwrSetGetLdoCfg_vset);                              \
                            RUN_TEST(test_functionality_pwrSetGetLdoCfg_vmonOnly);                          \
                            RUN_TEST(test_functionality_pwrSetGetLdoCfg_dischargeEn);                       \
                            RUN_TEST(test_functionality_pwrSetGetLdoCfg_dischargeSel);                      \
                            RUN_TEST(test_functionality_pwrSetGetLdoCfg_deglitchSel);                       \
                            RUN_TEST(test_functionality_pwrSetGetLdoCfg_uvThr);                             \
                            RUN_TEST(test_functionality_pwrSetGetLdoCfg_ovThr);                             \
                            RUN_TEST(test_functionality_pwrSetGetLdoCfg_ilimSel);                           \
                            RUN_TEST(test_functionality_pwrSetGetLdoCfg_ovpSel);                            \
                            RUN_TEST(test_functionality_pwrSetGetLdoCfg_ovSel);                             \
                            RUN_TEST(test_functionality_pwrSetGetLdoCfg_uvSel);                             \
                            RUN_TEST(test_functionality_pwrSetGetLdoCfg_scSel);                             \
                            RUN_TEST(test_functionality_pwrSetGetLdoCfg_rvConf);                            \
                            RUN_TEST(test_faultHandling_pwrGetRsrcStat_nullParams);                         \
                            RUN_TEST(test_faultHandling_pwrGetRsrcStat_invalidResource);                    \
                            RUN_TEST(test_functionality_pwrGetRsrcStat);                                    \
                            RUN_TEST(test_faultHandling_pwrSetTsdCfg_nullParams);                           \
                            RUN_TEST(test_faultHandling_pwrSetTsdCfg_noValidParams);                        \
                            RUN_TEST(test_faultHandling_pwrGetTsdCfg_nullParams);                           \
                            RUN_TEST(test_faultHandling_pwrGetTsdCfg_noValidParams);                        \
                            RUN_TEST(test_functionality_pwrSetGetTsdCfg_twarnStayInSafeState);              \
                            RUN_TEST(test_functionality_pwrSetGetTsdCfg_tsdImmLevel);                       \
                            RUN_TEST(test_functionality_pwrSetGetTsdCfg_twarnLevel);                        \
                            RUN_TEST(test_faultHandling_pwrSetBuckLdoSeqTrig_nullParams);                   \
                            RUN_TEST(test_faultHandling_pwrSetBuckLdoSeqTrig_invalidTrigger);               \
                            RUN_TEST(test_faultHandling_pwrGetBuckLdoSeqTrig_nullParams);                   \
                            RUN_TEST(test_faultHandling_pwrGetBuckLdoSeqTrig_invalidTrigger);               \
                            RUN_TEST(test_functionality_pwrSetGetBuckLdoSeqTrig_BUCK1_TRIGGER_PWR_ON_BIT);  \
                            RUN_TEST(test_functionality_pwrSetGetBuckLdoSeqTrig_BUCK1_TRIGGER_LDO_PG);      \
                            RUN_TEST(test_functionality_pwrSetGetBuckLdoSeqTrig_BUCK1_TRIGGER_BUCK3_PG);    \
                            RUN_TEST(test_functionality_pwrSetGetBuckLdoSeqTrig_BUCK1_TRIGGER_BUCK2_PG);    \
                            RUN_TEST(test_functionality_pwrSetGetBuckLdoSeqTrig_BUCK1_TRIGGER_GPIO_PIN);    \
                            RUN_TEST(test_functionality_pwrSetGetBuckLdoSeqTrig_BUCK1_TRIGGER_SEQ_PIN);     \
                            RUN_TEST(test_functionality_pwrSetGetBuckLdoSeqTrig_BUCK2_TRIGGER_PWR_ON_BIT);  \
                            RUN_TEST(test_functionality_pwrSetGetBuckLdoSeqTrig_BUCK2_TRIGGER_LDO_PG);      \
                            RUN_TEST(test_functionality_pwrSetGetBuckLdoSeqTrig_BUCK2_TRIGGER_BUCK3_PG);    \
                            RUN_TEST(test_functionality_pwrSetGetBuckLdoSeqTrig_BUCK2_TRIGGER_BUCK1_PG);    \
                            RUN_TEST(test_functionality_pwrSetGetBuckLdoSeqTrig_BUCK2_TRIGGER_GPIO_PIN);    \
                            RUN_TEST(test_functionality_pwrSetGetBuckLdoSeqTrig_BUCK2_TRIGGER_SEQ_PIN);     \
                            RUN_TEST(test_functionality_pwrSetGetBuckLdoSeqTrig_BUCK3_TRIGGER_PWR_ON_BIT);  \
                            RUN_TEST(test_functionality_pwrSetGetBuckLdoSeqTrig_BUCK3_TRIGGER_LDO_PG);      \
                            RUN_TEST(test_functionality_pwrSetGetBuckLdoSeqTrig_BUCK3_TRIGGER_BUCK2_PG);    \
                            RUN_TEST(test_functionality_pwrSetGetBuckLdoSeqTrig_BUCK3_TRIGGER_BUCK1_PG);    \
                            RUN_TEST(test_functionality_pwrSetGetBuckLdoSeqTrig_BUCK3_TRIGGER_GPIO_PIN);    \
                            RUN_TEST(test_functionality_pwrSetGetBuckLdoSeqTrig_BUCK3_TRIGGER_SEQ_PIN);     \
                            RUN_TEST(test_functionality_pwrSetGetBuckLdoSeqTrig_LDO_TRIGGER_PWR_ON_BIT);    \
                            RUN_TEST(test_functionality_pwrSetGetBuckLdoSeqTrig_LDO_TRIGGER_BUCK3_PG);      \
                            RUN_TEST(test_functionality_pwrSetGetBuckLdoSeqTrig_LDO_TRIGGER_BUCK2_PG);      \
                            RUN_TEST(test_functionality_pwrSetGetBuckLdoSeqTrig_LDO_TRIGGER_BUCK1_PG);      \
                            RUN_TEST(test_functionality_pwrSetGetBuckLdoSeqTrig_LDO_TRIGGER_GPIO_PIN);      \
                            RUN_TEST(test_functionality_pwrSetGetBuckLdoSeqTrig_LDO_TRIGGER_SEQ_PIN);       \
                            RUN_TEST(test_functionality_pwrSetGetBuckLdoSeqTrig_multipleTriggers);          \
                            RUN_TEST(test_faultHandling_pwrSetBuckLdoSeqDly_nullParams);                    \
                            RUN_TEST(test_faultHandling_pwrSetBuckLdoSeqDly_invalidResource);               \
                            RUN_TEST(test_faultHandling_pwrSetBuckLdoSeqDly_invalidSeqDlyOff);              \
                            RUN_TEST(test_faultHandling_pwrSetBuckLdoSeqDly_invalidSeqDlyOn);               \
                            RUN_TEST(test_faultHandling_pwrGetBuckLdoSeqDly_nullParams);                    \
                            RUN_TEST(test_faultHandling_pwrGetBuckLdoSeqDly_invalidResource);               \
                            RUN_TEST(test_functionality_pwrSetGetBuckLdoSeqDly_seqDlyOn);                   \
                            RUN_TEST(test_functionality_pwrSetGetBuckLdoSeqDly_seqDlyOff)

Pmic_CoreHandle_t pmicHandle;

void powerTest(void *args)
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

    DebugP_log("\r\nWelcome to PMIC LLD Power module testing\r\n\r\n");

    status = Pmic_init(&pmicCfg, &pmicHandle);

    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_unlockRegs(&pmicHandle);

        if (status == PMIC_ST_SUCCESS)
        {
            DebugP_log("Running PMIC Power tests...\r\n");

            UNITY_BEGIN();
            RUN_POWER_TESTS();
            UNITY_END();
        }
        else
        {
            DebugP_log("Failed to unlock PMIC registers\r\n");
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

void test_faultHandling_pwrSetBuckCfg_nullParams(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckCfg_t buckCfg = {
        .validParams = PMIC_BUCK_ENABLE_VALID,
        .resource = PMIC_BUCK1,
        .enable = PMIC_ENABLE
    };

    status = Pmic_pwrSetBuckCfg(NULL, &buckCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    status = Pmic_pwrSetBuckCfg(&pmicHandle, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    status = Pmic_pwrSetBuckCfg(NULL, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

void test_faultHandling_pwrSetBuckCfg_noValidParams(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckCfg_t buckCfg = {
        .validParams = 0U,
        .resource = PMIC_BUCK1,
    };

    status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);
}

void test_faultHandling_pwrSetBuckCfg_invalidResource(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckCfg_t buckCfg = {
        .validParams = PMIC_BUCK_ENABLE_VALID,
        .resource = PMIC_BUCK_MAX + 1U,
        .enable = PMIC_ENABLE
    };

    status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);
}

void test_faultHandling_pwrGetBuckCfg_nullParams(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckCfg_t buckCfg = {
        .validParams = PMIC_BUCK_ENABLE_VALID,
        .resource = PMIC_BUCK1,
    };

    status = Pmic_pwrGetBuckCfg(NULL, &buckCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    status = Pmic_pwrGetBuckCfg(&pmicHandle, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    status = Pmic_pwrGetBuckCfg(NULL, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

void test_faultHandling_pwrGetBuckCfg_noValidParams(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckCfg_t buckCfg = {
        .validParams = 0U,
        .resource = PMIC_BUCK1,
    };

    status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);
}

void test_faultHandling_pwrGetBuckCfg_invalidResource(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckCfg_t buckCfg = {
        .validParams = PMIC_BUCK_ENABLE_VALID,
        .resource = PMIC_BUCK_MAX + 1U,
    };

    status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);
}

void test_functionality_pwrSetGetBuckCfg_enable(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckCfg_t expBuckCfg = {.validParams = PMIC_BUCK_ENABLE_VALID};
    Pmic_PwrBuckCfg_t actBuckCfg = {.validParams = PMIC_BUCK_ENABLE_VALID};

    // For every buck regulator...
    for (uint8_t resource = PMIC_BUCK1; resource <= PMIC_BUCK_MAX; resource++)
    {
        // Set the buck as the resource
        expBuckCfg.resource = resource;
        actBuckCfg.resource = resource;

        // Enable the buck
        expBuckCfg.enable = PMIC_ENABLE;
        status = Pmic_pwrSetBuckCfg(&pmicHandle, &expBuckCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Get actual buck configuration and compare expected vs. actual
        status = Pmic_pwrGetBuckCfg(&pmicHandle, &actBuckCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        TEST_ASSERT_EQUAL(expBuckCfg.enable, actBuckCfg.enable);

        // Disable the buck
        expBuckCfg.enable = PMIC_DISABLE;
        status = Pmic_pwrSetBuckCfg(&pmicHandle, &expBuckCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Get actual buck configuration and compare expected vs. actual
        status = Pmic_pwrGetBuckCfg(&pmicHandle, &actBuckCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        TEST_ASSERT_EQUAL(expBuckCfg.enable, actBuckCfg.enable);
    }
}

void test_functionality_pwrSetGetBuckCfg_pulldownResistor(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckCfg_t expBuckCfg = {.validParams = PMIC_BUCK_PLDN_EN_VALID};
    Pmic_PwrBuckCfg_t actBuckCfg = {.validParams = PMIC_BUCK_PLDN_EN_VALID};

    // For every buck regulator...
    for (uint8_t resource = PMIC_BUCK1; resource <= PMIC_BUCK_MAX; resource++)
    {
        // Set the buck as the resource
        expBuckCfg.resource = resource;
        actBuckCfg.resource = resource;

        // Enable the buck pulldown resistor
        expBuckCfg.pldnEn = PMIC_ENABLE;
        status = Pmic_pwrSetBuckCfg(&pmicHandle, &expBuckCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Get actual buck configuration and compare expected vs. actual
        status = Pmic_pwrGetBuckCfg(&pmicHandle, &actBuckCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        TEST_ASSERT_EQUAL(expBuckCfg.pldnEn, actBuckCfg.pldnEn);

        // Disable the buck pulldown resistor
        expBuckCfg.pldnEn = PMIC_DISABLE;
        status = Pmic_pwrSetBuckCfg(&pmicHandle, &expBuckCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Get actual buck configuration and compare expected vs. actual
        status = Pmic_pwrGetBuckCfg(&pmicHandle, &actBuckCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        TEST_ASSERT_EQUAL(expBuckCfg.pldnEn, actBuckCfg.pldnEn);
    }
}

void test_functionality_pwrSetGetBuckCfg_fpwm(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckCfg_t expBuckCfg = {.validParams = PMIC_BUCK_FPWM_EN_VALID};
    Pmic_PwrBuckCfg_t actBuckCfg = {.validParams = PMIC_BUCK_FPWM_EN_VALID};

    // For every buck regulator...
    for (uint8_t resource = PMIC_BUCK1; resource <= PMIC_BUCK_MAX; resource++)
    {
        // Set the buck as the resource
        expBuckCfg.resource = resource;
        actBuckCfg.resource = resource;

        // Enable buck FPWM
        expBuckCfg.fpwmEn = PMIC_ENABLE;
        status = Pmic_pwrSetBuckCfg(&pmicHandle, &expBuckCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Get actual buck configuration and compare expected vs. actual
        status = Pmic_pwrGetBuckCfg(&pmicHandle, &actBuckCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        TEST_ASSERT_EQUAL(expBuckCfg.fpwmEn, actBuckCfg.fpwmEn);

        // Disable buck FPWM
        expBuckCfg.fpwmEn = PMIC_DISABLE;
        status = Pmic_pwrSetBuckCfg(&pmicHandle, &expBuckCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Get actual buck configuration and compare expected vs. actual
        status = Pmic_pwrGetBuckCfg(&pmicHandle, &actBuckCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        TEST_ASSERT_EQUAL(expBuckCfg.fpwmEn, actBuckCfg.fpwmEn);
    }
}

void test_functionality_pwrSetGetBuckCfg_uvThr(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckCfg_t expBuckCfg = {.validParams = PMIC_BUCK_UV_THR_VALID};
    Pmic_PwrBuckCfg_t actBuckCfg = {.validParams = PMIC_BUCK_UV_THR_VALID};

    // For every buck regulator...
    for (uint8_t resource = PMIC_BUCK1; resource <= PMIC_BUCK_MAX; resource++)
    {
        // Set the buck as the resource
        expBuckCfg.resource = resource;
        actBuckCfg.resource = resource;

        // Test Pmic_pwrSetBuckCfg() API error handling for out of bounds value
        expBuckCfg.uvThr = PMIC_BUCK_UV_THR_MAX + 1U;
        status = Pmic_pwrSetBuckCfg(&pmicHandle, &expBuckCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

        // For every valid buck UV threshold value...
        for (uint8_t uvThr = 0U; uvThr <= PMIC_BUCK_UV_THR_MAX; uvThr++)
        {
            // Set buck UV threshold
            expBuckCfg.uvThr = uvThr;
            status = Pmic_pwrSetBuckCfg(&pmicHandle, &expBuckCfg);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

            // Get actual buck configuration and compare expected vs. actual
            status = Pmic_pwrGetBuckCfg(&pmicHandle, &actBuckCfg);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
            TEST_ASSERT_EQUAL(expBuckCfg.uvThr, actBuckCfg.uvThr);
        }
    }
}

void test_functionality_pwrSetGetBuckCfg_ovThr(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckCfg_t expBuckCfg = {.validParams = PMIC_BUCK_OV_THR_VALID};
    Pmic_PwrBuckCfg_t actBuckCfg = {.validParams = PMIC_BUCK_OV_THR_VALID};

    // For every buck regulator...
    for (uint8_t resource = PMIC_BUCK1; resource <= PMIC_BUCK_MAX; resource++)
    {
        // Set the buck as the resource
        expBuckCfg.resource = resource;
        actBuckCfg.resource = resource;

        // Test Pmic_pwrSetBuckCfg() API error handling for out of bounds value
        expBuckCfg.ovThr = PMIC_BUCK_OV_THR_MAX + 1U;
        status = Pmic_pwrSetBuckCfg(&pmicHandle, &expBuckCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

        // For every valid buck OV threshold value...
        for (uint8_t ovThr = 0U; ovThr <= PMIC_BUCK_OV_THR_MAX; ovThr++)
        {
            // Set buck OV threshold
            expBuckCfg.ovThr = ovThr;
            status = Pmic_pwrSetBuckCfg(&pmicHandle, &expBuckCfg);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

            // Get actual buck configuration and compare expected vs. actual
            status = Pmic_pwrGetBuckCfg(&pmicHandle, &actBuckCfg);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
            TEST_ASSERT_EQUAL(expBuckCfg.ovThr, actBuckCfg.ovThr);
        }
    }
}

static void setGetBuck1IlimSel(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckCfg_t expBuckCfg = {.validParams = PMIC_BUCK_ILIM_SEL_VALID, .resource = PMIC_BUCK1};
    Pmic_PwrBuckCfg_t actBuckCfg = {.validParams = PMIC_BUCK_ILIM_SEL_VALID, .resource = PMIC_BUCK1};

    // Test Pmic_pwrSetBuckCfg() API error handling for when out of bounds value
    expBuckCfg.ilimSel = PMIC_BUCK1_ILIM_MAX + 1U;
    status = Pmic_pwrSetBuckCfg(&pmicHandle, &expBuckCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

    // For each BUCK1 ILIM selection...
    for (uint8_t ilimSel = PMIC_BUCK1_ILIM_3P1_A; ilimSel <= PMIC_BUCK1_ILIM_MAX; ilimSel++)
    {
        // Set the ILIM selection
        expBuckCfg.ilimSel = ilimSel;
        status = Pmic_pwrSetBuckCfg(&pmicHandle, &expBuckCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Get actual BUCK1 ILIM selection and compare expected vs. actual
        status = Pmic_pwrGetBuckCfg(&pmicHandle, &actBuckCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        TEST_ASSERT_EQUAL(expBuckCfg.ilimSel, actBuckCfg.ilimSel);
    }
}

static void setGetBuck2_3IlimSel(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckCfg_t expBuckCfg = {.validParams = PMIC_BUCK_ILIM_SEL_VALID};
    Pmic_PwrBuckCfg_t actBuckCfg = {.validParams = PMIC_BUCK_ILIM_SEL_VALID};

    // For BUCK2 and BUCK3...
    for (uint8_t resource = PMIC_BUCK2; resource <= PMIC_BUCK3; resource++)
    {
        // Set the buck as the resource
        expBuckCfg.resource = resource;
        actBuckCfg.resource = resource;

        // Test Pmic_pwrSetBuckCfg() API error handling for out of bounds value
        expBuckCfg.ilimSel = PMIC_BUCK2_3_ILIM_MAX + 1U;
        status = Pmic_pwrSetBuckCfg(&pmicHandle, &expBuckCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

        // For each BUCK2/BUCK3 ILIM selection...
        for (uint8_t ilimSel = PMIC_BUCK2_3_ILIM_4P5_A; ilimSel <= PMIC_BUCK2_3_ILIM_MAX; ilimSel++)
        {
            // Set the ILIM selection
            expBuckCfg.ilimSel = ilimSel;
            status = Pmic_pwrSetBuckCfg(&pmicHandle, &expBuckCfg);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

            // Get actual BUCK2/BUCK3 ILIM selection and compare expected vs. actual
            status = Pmic_pwrGetBuckCfg(&pmicHandle, &actBuckCfg);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
            TEST_ASSERT_EQUAL(expBuckCfg.ilimSel, actBuckCfg.ilimSel);
        }
    }
}

void test_functionality_pwrSetGetBuckCfg_ilimSel(void)
{
    setGetBuck1IlimSel();
    setGetBuck2_3IlimSel();
}

void test_functionality_pwrSetGetBuckCfg_ovpSel(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckCfg_t expBuckCfg = {.validParams = PMIC_BUCK_OVP_SEL_VALID};
    Pmic_PwrBuckCfg_t actBuckCfg = {.validParams = PMIC_BUCK_OVP_SEL_VALID};

    // For every buck regulator...
    for (uint8_t resource = PMIC_BUCK1; resource <= PMIC_BUCK_MAX; resource++)
    {
        // Set the buck as the resource
        expBuckCfg.resource = resource;
        actBuckCfg.resource = resource;

        // Test Pmic_pwrSetBuckCfg() API error handling for out of bounds value
        expBuckCfg.ovpSel = PMIC_REGULATOR_FAULT_RESPONSE_MAX + 1U;
        status = Pmic_pwrSetBuckCfg(&pmicHandle, &expBuckCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

        // For every valid buck OVP fault response value...
        for (uint8_t ovpSel = 0U; ovpSel <= PMIC_REGULATOR_FAULT_RESPONSE_MAX; ovpSel++)
        {
            // Set buck OVP response
            expBuckCfg.ovpSel = ovpSel;
            status = Pmic_pwrSetBuckCfg(&pmicHandle, &expBuckCfg);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

            // Get actual buck configuration and compare expected vs. actual
            status = Pmic_pwrGetBuckCfg(&pmicHandle, &actBuckCfg);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
            TEST_ASSERT_EQUAL(expBuckCfg.ovpSel, actBuckCfg.ovpSel);
        }
    }
}

void test_functionality_pwrSetGetBuckCfg_ovSel(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckCfg_t expBuckCfg = {.validParams = PMIC_BUCK_OV_SEL_VALID};
    Pmic_PwrBuckCfg_t actBuckCfg = {.validParams = PMIC_BUCK_OV_SEL_VALID};

    // For every buck regulator...
    for (uint8_t resource = PMIC_BUCK1; resource <= PMIC_BUCK_MAX; resource++)
    {
        // Set the buck as the resource
        expBuckCfg.resource = resource;
        actBuckCfg.resource = resource;

        // Test Pmic_pwrSetBuckCfg() API error handling for out of bounds value
        expBuckCfg.ovSel = PMIC_REGULATOR_FAULT_RESPONSE_MAX + 1U;
        status = Pmic_pwrSetBuckCfg(&pmicHandle, &expBuckCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

        // For every valid buck OV fault response value...
        for (uint8_t ovSel = 0U; ovSel <= PMIC_REGULATOR_FAULT_RESPONSE_MAX; ovSel++)
        {
            // Set buck OV response
            expBuckCfg.ovSel = ovSel;
            status = Pmic_pwrSetBuckCfg(&pmicHandle, &expBuckCfg);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

            // Get actual buck configuration and compare expected vs. actual
            status = Pmic_pwrGetBuckCfg(&pmicHandle, &actBuckCfg);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
            TEST_ASSERT_EQUAL(expBuckCfg.ovSel, actBuckCfg.ovSel);
        }
    }
}

void test_functionality_pwrSetGetBuckCfg_uvSel(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckCfg_t expBuckCfg = {.validParams = PMIC_BUCK_UV_SEL_VALID};
    Pmic_PwrBuckCfg_t actBuckCfg = {.validParams = PMIC_BUCK_UV_SEL_VALID};

    // For every buck regulator...
    for (uint8_t resource = PMIC_BUCK1; resource <= PMIC_BUCK_MAX; resource++)
    {
        // Set the buck as the resource
        expBuckCfg.resource = resource;
        actBuckCfg.resource = resource;

        // Test Pmic_pwrSetBuckCfg() API error handling for out of bounds value
        expBuckCfg.uvSel = PMIC_REGULATOR_FAULT_RESPONSE_MAX + 1U;
        status = Pmic_pwrSetBuckCfg(&pmicHandle, &expBuckCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

        // For every valid buck UV fault response value...
        for (uint8_t uvSel = 0U; uvSel <= PMIC_REGULATOR_FAULT_RESPONSE_MAX; uvSel++)
        {
            // Set buck UV response
            expBuckCfg.uvSel = uvSel;
            status = Pmic_pwrSetBuckCfg(&pmicHandle, &expBuckCfg);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

            // Get actual buck configuration and compare expected vs. actual
            status = Pmic_pwrGetBuckCfg(&pmicHandle, &actBuckCfg);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
            TEST_ASSERT_EQUAL(expBuckCfg.uvSel, actBuckCfg.uvSel);
        }
    }
}

void test_functionality_pwrSetGetBuckCfg_scSel(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckCfg_t expBuckCfg = {.validParams = PMIC_BUCK_SC_SEL_VALID};
    Pmic_PwrBuckCfg_t actBuckCfg = {.validParams = PMIC_BUCK_SC_SEL_VALID};

    // For every buck regulator...
    for (uint8_t resource = PMIC_BUCK1; resource <= PMIC_BUCK_MAX; resource++)
    {
        // Set the buck as the resource
        expBuckCfg.resource = resource;
        actBuckCfg.resource = resource;

        // Test Pmic_pwrSetBuckCfg() API error handling for out of bounds value
        expBuckCfg.scSel = PMIC_REGULATOR_FAULT_RESPONSE_MAX + 1U;
        status = Pmic_pwrSetBuckCfg(&pmicHandle, &expBuckCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

        // For every valid buck SC fault response value...
        for (uint8_t scSel = 0U; scSel <= PMIC_REGULATOR_FAULT_RESPONSE_MAX; scSel++)
        {
            // Set buck SC response
            expBuckCfg.scSel = scSel;
            status = Pmic_pwrSetBuckCfg(&pmicHandle, &expBuckCfg);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

            // Get actual buck configuration and compare expected vs. actual
            status = Pmic_pwrGetBuckCfg(&pmicHandle, &actBuckCfg);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
            TEST_ASSERT_EQUAL(expBuckCfg.scSel, actBuckCfg.scSel);
        }
    }
}

void test_functionality_pwrSetGetBuckCfg_rvConf(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckCfg_t expBuckCfg = {.validParams = PMIC_BUCK_RV_CONF_VALID};
    Pmic_PwrBuckCfg_t actBuckCfg = {.validParams = PMIC_BUCK_RV_CONF_VALID};

    // For every buck regulator...
    for (uint8_t resource = PMIC_BUCK1; resource <= PMIC_BUCK_MAX; resource++)
    {
        // Set the buck as the resource
        expBuckCfg.resource = resource;
        actBuckCfg.resource = resource;

        // Test Pmic_pwrSetBuckCfg() API error handling for out of bounds value
        expBuckCfg.rvConf = PMIC_BUCK_RV_CONF_MAX + 1U;
        status = Pmic_pwrSetBuckCfg(&pmicHandle, &expBuckCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

        // Configure the buck to wait for the rail to discharge before powering up
        expBuckCfg.rvConf = PMIC_BUCK_RAIL_DISCHARGE;
        status = Pmic_pwrSetBuckCfg(&pmicHandle, &expBuckCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Get actual buck configuration and compare expected vs. actual
        status = Pmic_pwrGetBuckCfg(&pmicHandle, &actBuckCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        TEST_ASSERT_EQUAL(expBuckCfg.rvConf, actBuckCfg.rvConf);

        // Configure the buck to ignore RV during power up
        expBuckCfg.rvConf = PMIC_BUCK_RV_IGNORE;
        status = Pmic_pwrSetBuckCfg(&pmicHandle, &expBuckCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Get actual buck configuration and compare expected vs. actual
        status = Pmic_pwrGetBuckCfg(&pmicHandle, &actBuckCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        TEST_ASSERT_EQUAL(expBuckCfg.rvConf, actBuckCfg.rvConf);
    }
}

void test_functionality_pwrSetGetBuckCfg_slewRate(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckCfg_t expBuckCfg = {.validParams = PMIC_BUCK_SLEW_RATE_VALID};
    Pmic_PwrBuckCfg_t actBuckCfg = {.validParams = PMIC_BUCK_SLEW_RATE_VALID};

    // For every buck regulator...
    for (uint8_t resource = PMIC_BUCK1; resource <= PMIC_BUCK_MAX; resource++)
    {
        // Set the buck as the resource
        expBuckCfg.resource = resource;
        actBuckCfg.resource = resource;

        // Test Pmic_pwrSetBuckCfg() API error handling for out of bounds value
        expBuckCfg.slewRate = PMIC_BUCK_SLEW_RATE_MAX + 1U;
        status = Pmic_pwrSetBuckCfg(&pmicHandle, &expBuckCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

        // For every valid buck slew rate value...
        for (uint8_t slewRate = 0U; slewRate <= PMIC_BUCK_SLEW_RATE_MAX; slewRate++)
        {
            // Set buck slew rate
            expBuckCfg.slewRate = slewRate;
            status = Pmic_pwrSetBuckCfg(&pmicHandle, &expBuckCfg);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

            // Get actual buck configuration and compare expected vs. actual
            status = Pmic_pwrGetBuckCfg(&pmicHandle, &actBuckCfg);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
            TEST_ASSERT_EQUAL(expBuckCfg.slewRate, actBuckCfg.slewRate);
        }
    }
}

void test_functionality_pwrSetGetBuckCfg_deglitchSel(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckCfg_t expBuckCfg = {.validParams = PMIC_BUCK_DEGLITCH_SEL_VALID};
    Pmic_PwrBuckCfg_t actBuckCfg = {.validParams = PMIC_BUCK_DEGLITCH_SEL_VALID};

    // For every buck regulator...
    for (uint8_t resource = PMIC_BUCK1; resource <= PMIC_BUCK_MAX; resource++)
    {
        // Set the buck as the resource
        expBuckCfg.resource = resource;
        actBuckCfg.resource = resource;

        // Test Pmic_pwrSetBuckCfg() API error handling for out of bounds value
        expBuckCfg.deglitchSel = PMIC_BUCK_DEGLITCH_SEL_MAX + 1U;
        status = Pmic_pwrSetBuckCfg(&pmicHandle, &expBuckCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

        // For every valid buck deglitch selection value...
        for (uint8_t deglitchSel = 0U; deglitchSel <= PMIC_BUCK_DEGLITCH_SEL_MAX; deglitchSel++)
        {
            // Set buck deglitch selection
            expBuckCfg.deglitchSel = deglitchSel;
            status = Pmic_pwrSetBuckCfg(&pmicHandle, &expBuckCfg);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

            // Get actual buck configuration and compare expected vs. actual
            status = Pmic_pwrGetBuckCfg(&pmicHandle, &actBuckCfg);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
            TEST_ASSERT_EQUAL(expBuckCfg.deglitchSel, actBuckCfg.deglitchSel);
        }
    }
}

void test_functionality_pwrSetGetBuckCfg_dischargeSel(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckCfg_t expBuckCfg = {.validParams = PMIC_BUCK_DISCHARGE_SEL_VALID};
    Pmic_PwrBuckCfg_t actBuckCfg = {.validParams = PMIC_BUCK_DISCHARGE_SEL_VALID};

    // For every buck regulator...
    for (uint8_t resource = PMIC_BUCK1; resource <= PMIC_BUCK_MAX; resource++)
    {
        // Set the buck as the resource
        expBuckCfg.resource = resource;
        actBuckCfg.resource = resource;

        // Test Pmic_pwrSetBuckCfg() API error handling for out of bounds value
        expBuckCfg.dischargeSel = PMIC_BUCK_DISCHARGE_SEL_MAX + 1U;
        status = Pmic_pwrSetBuckCfg(&pmicHandle, &expBuckCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

        // Set buck discharge configuration to be slew rate controlled
        expBuckCfg.dischargeSel = PMIC_BUCK_SLEW_RATE_CONTROLLED;
        status = Pmic_pwrSetBuckCfg(&pmicHandle, &expBuckCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Get actual buck configuration and compare expected vs. actual
        status = Pmic_pwrGetBuckCfg(&pmicHandle, &actBuckCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        TEST_ASSERT_EQUAL(expBuckCfg.dischargeSel, actBuckCfg.dischargeSel);

        // Set buck discharge configuration to be through resistive discharge
        expBuckCfg.dischargeSel = PMIC_BUCK_RESISTIVE_DISCHARGE;
        status = Pmic_pwrSetBuckCfg(&pmicHandle, &expBuckCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Get actual buck configuration and compare expected vs. actual
        status = Pmic_pwrGetBuckCfg(&pmicHandle, &actBuckCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        TEST_ASSERT_EQUAL(expBuckCfg.dischargeSel, actBuckCfg.dischargeSel);
    }
}

void test_functionality_pwrSetGetBuckCfg_ssEn(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckCfg_t expBuckCfg = {.validParams = PMIC_BUCK_SS_EN_VALID};
    Pmic_PwrBuckCfg_t actBuckCfg = {.validParams = PMIC_BUCK_SS_EN_VALID};

    // For every buck regulator...
    for (uint8_t resource = PMIC_BUCK1; resource <= PMIC_BUCK_MAX; resource++)
    {
        // Set the buck as the resource
        expBuckCfg.resource = resource;
        actBuckCfg.resource = resource;

        // Enable buck spread spectrum
        expBuckCfg.ssEn = PMIC_ENABLE;
        status = Pmic_pwrSetBuckCfg(&pmicHandle, &expBuckCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Get actual buck configuration and compare expected vs. actual
        status = Pmic_pwrGetBuckCfg(&pmicHandle, &actBuckCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        TEST_ASSERT_EQUAL(expBuckCfg.ssEn, actBuckCfg.ssEn);

        // Disable buck spread spectrum
        expBuckCfg.ssEn = PMIC_DISABLE;
        status = Pmic_pwrSetBuckCfg(&pmicHandle, &expBuckCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Get actual buck configuration and compare expected vs. actual
        status = Pmic_pwrGetBuckCfg(&pmicHandle, &actBuckCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        TEST_ASSERT_EQUAL(expBuckCfg.ssEn, actBuckCfg.ssEn);
    }
}

void test_functionality_pwrSetGetBuckCfg_ssmSel(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckCfg_t expBuckCfg = {.validParams = PMIC_BUCK_SSM_SEL_VALID};
    Pmic_PwrBuckCfg_t actBuckCfg = {.validParams = PMIC_BUCK_SSM_SEL_VALID};

    // For every buck regulator...
    for (uint8_t resource = PMIC_BUCK1; resource <= PMIC_BUCK_MAX; resource++)
    {
        // Set the buck as the resource
        expBuckCfg.resource = resource;
        actBuckCfg.resource = resource;

        // Test Pmic_pwrSetBuckCfg() API error handling for out of bounds value
        expBuckCfg.ssmSel = PMIC_SSM_SEL_MAX + 1U;
        status = Pmic_pwrSetBuckCfg(&pmicHandle, &expBuckCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

        // Set buck SSM to be SSMDEPTH_T150
        expBuckCfg.ssmSel = PMIC_SSMDEPTH_T150;
        status = Pmic_pwrSetBuckCfg(&pmicHandle, &expBuckCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Get actual buck configuration and compare expected vs. actual
        status = Pmic_pwrGetBuckCfg(&pmicHandle, &actBuckCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        TEST_ASSERT_EQUAL(expBuckCfg.ssmSel, actBuckCfg.ssmSel);

        // Set buck SSM to be SSMDEPTH_T125
        expBuckCfg.ssmSel = PMIC_SSMDEPTH_T125;
        status = Pmic_pwrSetBuckCfg(&pmicHandle, &expBuckCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Get actual buck configuration and compare expected vs. actual
        status = Pmic_pwrGetBuckCfg(&pmicHandle, &actBuckCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        TEST_ASSERT_EQUAL(expBuckCfg.ssmSel, actBuckCfg.ssmSel);
    }
}

void test_functionality_pwrSetGetBuckCfg_vset(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckCfg_t expBuckCfg = {.validParams = PMIC_BUCK_VSET_VALID};
    Pmic_PwrBuckCfg_t actBuckCfg = {.validParams = PMIC_BUCK_VSET_VALID};

    // Test Pmic_pwrSetBuckCfg() and Pmic_pwrGetBuckCfg() API error
    // handling for invalid resource
    for (uint8_t resource = PMIC_BUCK2; resource <= PMIC_BUCK3; resource++)
    {
        expBuckCfg.resource = resource;
        expBuckCfg.vset = PMIC_BUCK1_VSET_MIN;
        status = Pmic_pwrSetBuckCfg(&pmicHandle, &expBuckCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

        actBuckCfg.resource = resource;
        status = Pmic_pwrGetBuckCfg(&pmicHandle, &actBuckCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);
    }

    // Test Pmic_pwrSetBuckCfg() API error handling for out of bounds value
    expBuckCfg.resource = PMIC_BUCK1;
    expBuckCfg.vset = PMIC_BUCK1_VSET_MAX + 1U;
    status = Pmic_pwrSetBuckCfg(&pmicHandle, &expBuckCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

    // Set resource to be BUCK1
    actBuckCfg.resource = PMIC_BUCK1;

    // For every valid buck VSET value...
    for (uint8_t vset = PMIC_BUCK1_VSET_MIN; vset <= PMIC_BUCK1_VSET_MAX; vset++)
    {
        // Set buck VSET
        expBuckCfg.vset = vset;
        status = Pmic_pwrSetBuckCfg(&pmicHandle, &expBuckCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Get actual buck configuration and compare expected vs. actual
        status = Pmic_pwrGetBuckCfg(&pmicHandle, &actBuckCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        TEST_ASSERT_EQUAL(expBuckCfg.vset, actBuckCfg.vset);
    }
}

void test_functionality_pwrSetGetBuckCfg_uvloRising(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckCfg_t expBuckCfg = {.validParams = PMIC_BUCK_UVLO_RISING_VALID};
    Pmic_PwrBuckCfg_t actBuckCfg = {.validParams = PMIC_BUCK_UVLO_RISING_VALID};

    // Test Pmic_pwrSetBuckCfg() and Pmic_pwrGetBuckCfg() API error
    // handling for invalid resource
    for (uint8_t resource = PMIC_BUCK2; resource <= PMIC_BUCK3; resource++)
    {
        expBuckCfg.resource = resource;
        expBuckCfg.uvloRising = 0x01U;
        status = Pmic_pwrSetBuckCfg(&pmicHandle, &expBuckCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

        actBuckCfg.resource = resource;
        status = Pmic_pwrGetBuckCfg(&pmicHandle, &actBuckCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);
    }

    // Test Pmic_pwrSetBuckCfg() API error handling for out of bounds value
    expBuckCfg.resource = PMIC_BUCK1;
    expBuckCfg.uvloRising = PMIC_BUCK1_UVLO_RISING_MAX + 1U;
    status = Pmic_pwrSetBuckCfg(&pmicHandle, &expBuckCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

    // Set resource to be BUCK1
    actBuckCfg.resource = PMIC_BUCK1;

    // For every valid buck rising UVLO value...
    for (uint8_t uvloRising = 0U; uvloRising <= PMIC_BUCK1_UVLO_RISING_MAX; uvloRising++)
    {
        // Set buck rising UVLO
        expBuckCfg.uvloRising = uvloRising;
        status = Pmic_pwrSetBuckCfg(&pmicHandle, &expBuckCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Get actual buck configuration and compare expected vs. actual
        status = Pmic_pwrGetBuckCfg(&pmicHandle, &actBuckCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        TEST_ASSERT_EQUAL(expBuckCfg.uvloRising, actBuckCfg.uvloRising);
    }
}

void test_functionality_pwrSetGetBuckCfg_uvloFalling(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckCfg_t expBuckCfg = {.validParams = PMIC_BUCK_UVLO_FALLING_VALID};
    Pmic_PwrBuckCfg_t actBuckCfg = {.validParams = PMIC_BUCK_UVLO_FALLING_VALID};

    // Test Pmic_pwrSetBuckCfg() and Pmic_pwrGetBuckCfg() API error
    // handling for invalid resource
    for (uint8_t resource = PMIC_BUCK2; resource <= PMIC_BUCK3; resource++)
    {
        expBuckCfg.resource = resource;
        expBuckCfg.uvloFalling = 0x01U;
        status = Pmic_pwrSetBuckCfg(&pmicHandle, &expBuckCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

        actBuckCfg.resource = resource;
        status = Pmic_pwrGetBuckCfg(&pmicHandle, &actBuckCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);
    }

    // Test Pmic_pwrSetBuckCfg() API error handling for out of bounds value
    expBuckCfg.resource = PMIC_BUCK1;
    expBuckCfg.uvloFalling = PMIC_BUCK1_UVLO_FALLING_MAX + 1U;
    status = Pmic_pwrSetBuckCfg(&pmicHandle, &expBuckCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

    // Set resource to be BUCK1
    actBuckCfg.resource = PMIC_BUCK1;

    // For every valid buck falling UVLO value...
    for (uint8_t uvloFalling = 0U; uvloFalling <= PMIC_BUCK1_UVLO_FALLING_MAX; uvloFalling++)
    {
        // Set buck falling UVLO
        expBuckCfg.uvloFalling = uvloFalling;
        status = Pmic_pwrSetBuckCfg(&pmicHandle, &expBuckCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Get actual buck configuration and compare expected vs. actual
        status = Pmic_pwrGetBuckCfg(&pmicHandle, &actBuckCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        TEST_ASSERT_EQUAL(expBuckCfg.uvloFalling, actBuckCfg.uvloFalling);
    }
}

void test_functionality_pwrSetGetBuckCfg_highSideSlewRate(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckCfg_t expBuckCfg = {.validParams = PMIC_BUCK_HIGH_SIDE_SLEW_RATE_VALID};
    Pmic_PwrBuckCfg_t actBuckCfg = {.validParams = PMIC_BUCK_HIGH_SIDE_SLEW_RATE_VALID};

    // Test Pmic_pwrSetBuckCfg() and Pmic_pwrGetBuckCfg() API error
    // handling for invalid resource
    for (uint8_t resource = PMIC_BUCK2; resource <= PMIC_BUCK3; resource++)
    {
        expBuckCfg.resource = resource;
        expBuckCfg.highSideSlewRate = PMIC_HIGH_SIDE_SLEW_RATE_SLOWEST;
        status = Pmic_pwrSetBuckCfg(&pmicHandle, &expBuckCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

        actBuckCfg.resource = resource;
        status = Pmic_pwrGetBuckCfg(&pmicHandle, &actBuckCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);
    }

    // Test Pmic_pwrSetBuckCfg() API error handling for out of bounds value
    expBuckCfg.resource = PMIC_BUCK1;
    expBuckCfg.highSideSlewRate = PMIC_BUCK1_EN_HS_ON_SR_MAX + 1U;
    status = Pmic_pwrSetBuckCfg(&pmicHandle, &expBuckCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

    // Set resource to be BUCK1
    actBuckCfg.resource = PMIC_BUCK1;

    // Set buck high side slew rate to be slowest
    expBuckCfg.highSideSlewRate = PMIC_HIGH_SIDE_SLEW_RATE_SLOWEST;
    status = Pmic_pwrSetBuckCfg(&pmicHandle, &expBuckCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get actual buck configuration and compare expected vs. actual
    status = Pmic_pwrGetBuckCfg(&pmicHandle, &actBuckCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(expBuckCfg.uvloFalling, actBuckCfg.uvloFalling);

    // Set buck high side slew rate to be fastest
    expBuckCfg.highSideSlewRate = PMIC_HIGH_SIDE_SLEW_RATE_FASTEST;
    status = Pmic_pwrSetBuckCfg(&pmicHandle, &expBuckCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get actual buck configuration and compare expected vs. actual
    status = Pmic_pwrGetBuckCfg(&pmicHandle, &actBuckCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(expBuckCfg.uvloFalling, actBuckCfg.uvloFalling);
}

void test_functionality_pwrSetGetBuckCfg_vsetActive(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckCfg_t expBuckCfg = {.validParams = PMIC_BUCK_VSET_ACTIVE_VALID};
    Pmic_PwrBuckCfg_t actBuckCfg = {.validParams = PMIC_BUCK_VSET_ACTIVE_VALID};

    // Test Pmic_pwrSetBuckCfg() and Pmic_pwrGetBuckCfg() API error handling for
    // invalid resource
    expBuckCfg.resource = PMIC_BUCK1;
    actBuckCfg.resource = PMIC_BUCK1;
    expBuckCfg.vsetActive = PMIC_BUCK2_3_VSET_MIN;
    status = Pmic_pwrSetBuckCfg(&pmicHandle, &expBuckCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);
    status = Pmic_pwrGetBuckCfg(&pmicHandle, &actBuckCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

    // Test Pmic_pwrSetBuckCfg() API error handling for out of bounds value
    expBuckCfg.resource = PMIC_BUCK2;
    expBuckCfg.vsetActive = PMIC_BUCK2_3_VSET_MAX + 1U;
    status = Pmic_pwrSetBuckCfg(&pmicHandle, &expBuckCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);
    expBuckCfg.vsetLPwr = PMIC_BUCK2_3_VSET_MAX - 1U;
    status = Pmic_pwrSetBuckCfg(&pmicHandle, &expBuckCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

    // For BUCK2 and BUCK3...
    for (uint8_t resource = PMIC_BUCK2; resource <= PMIC_BUCK3; resource++)
    {
        // Set the buck as the resource
        expBuckCfg.resource = resource;
        actBuckCfg.resource = resource;

        // For every valid buck active VSET value...
        for (uint8_t vsetActive = PMIC_BUCK2_3_VSET_MIN; vsetActive <= PMIC_BUCK2_3_VSET_MAX; vsetActive++)
        {
            // Set buck active VSET
            expBuckCfg.vsetActive = vsetActive;
            status = Pmic_pwrSetBuckCfg(&pmicHandle, &expBuckCfg);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

            // Get actual buck configuration and compare expected vs. actual
            status = Pmic_pwrGetBuckCfg(&pmicHandle, &actBuckCfg);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
            TEST_ASSERT_EQUAL(expBuckCfg.vsetActive, actBuckCfg.vsetActive);
        }
    }
}

void test_functionality_pwrSetGetBuckCfg_vsetLPwr(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckCfg_t expBuckCfg = {.validParams = PMIC_BUCK_VSET_LPWR_VALID};
    Pmic_PwrBuckCfg_t actBuckCfg = {.validParams = PMIC_BUCK_VSET_LPWR_VALID};

    // Test Pmic_pwrSetBuckCfg() and Pmic_pwrGetBuckCfg() API error handling for
    // invalid resource
    expBuckCfg.resource = PMIC_BUCK1;
    actBuckCfg.resource = PMIC_BUCK1;
    expBuckCfg.vsetLPwr = PMIC_BUCK2_3_VSET_MIN;
    status = Pmic_pwrSetBuckCfg(&pmicHandle, &expBuckCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);
    status = Pmic_pwrGetBuckCfg(&pmicHandle, &actBuckCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

    // Test Pmic_pwrSetBuckCfg() API error handling for out of bounds value
    expBuckCfg.resource = PMIC_BUCK2;
    expBuckCfg.vsetLPwr = PMIC_BUCK2_3_VSET_MAX + 1U;
    status = Pmic_pwrSetBuckCfg(&pmicHandle, &expBuckCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);
    expBuckCfg.vsetLPwr = PMIC_BUCK2_3_VSET_MIN - 1U;
    status = Pmic_pwrSetBuckCfg(&pmicHandle, &expBuckCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

    // For BUCK2 and BUCK3...
    for (uint8_t resource = PMIC_BUCK2; resource <= PMIC_BUCK3; resource++)
    {
        // Set the buck as the resource
        expBuckCfg.resource = resource;
        actBuckCfg.resource = resource;

        // For every valid buck low power VSET value...
        for (uint8_t vsetLPwr = PMIC_BUCK2_3_VSET_MIN; vsetLPwr <= PMIC_BUCK2_3_VSET_MAX; vsetLPwr++)
        {
            // Set buck low power VSET
            expBuckCfg.vsetLPwr = vsetLPwr;
            status = Pmic_pwrSetBuckCfg(&pmicHandle, &expBuckCfg);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

            // Get actual buck configuration and compare expected vs. actual
            status = Pmic_pwrGetBuckCfg(&pmicHandle, &actBuckCfg);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
            TEST_ASSERT_EQUAL(expBuckCfg.vsetLPwr, actBuckCfg.vsetLPwr);
        }
    }
}

void test_functionality_pwrSetGetBuckCfg_vmonOnly(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckCfg_t expBuckCfg = {.validParams = PMIC_BUCK_VMON_ONLY_VALID};
    Pmic_PwrBuckCfg_t actBuckCfg = {.validParams = PMIC_BUCK_VMON_ONLY_VALID};

    // Test Pmic_pwrSetBuckCfg() and Pmic_pwrGetBuckCfg() API error handling for
    // invalid resource
    expBuckCfg.resource = PMIC_BUCK1;
    actBuckCfg.resource = PMIC_BUCK1;
    expBuckCfg.vmonOnly = (bool)true;
    status = Pmic_pwrSetBuckCfg(&pmicHandle, &expBuckCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);
    status = Pmic_pwrGetBuckCfg(&pmicHandle, &actBuckCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

    // For BUCK2 and BUCK3...
    for (uint8_t resource = PMIC_BUCK2; resource <= PMIC_BUCK3; resource++)
    {
        // Set the buck as the resource
        expBuckCfg.resource = resource;
        actBuckCfg.resource = resource;

        // Set VMON_ONLY to true
        expBuckCfg.vmonOnly = (bool)true;
        status = Pmic_pwrSetBuckCfg(&pmicHandle, &expBuckCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Get actual buck configuration and compare expected vs. actual
        status = Pmic_pwrGetBuckCfg(&pmicHandle, &actBuckCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        TEST_ASSERT_EQUAL(expBuckCfg.vmonOnly, actBuckCfg.vmonOnly);

        // Set VMON_ONLY to false
        expBuckCfg.vmonOnly = (bool)false;
        status = Pmic_pwrSetBuckCfg(&pmicHandle, &expBuckCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Get actual buck configuration and compare expected vs. actual
        status = Pmic_pwrGetBuckCfg(&pmicHandle, &actBuckCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        TEST_ASSERT_EQUAL(expBuckCfg.vmonOnly, actBuckCfg.vmonOnly);
    }
}

void test_faultHandling_pwrSetLdoCfg_nullParams(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrLdoCfg_t ldoCfg = {.validParams = PMIC_LDO_ENABLE_VALID, .enable = PMIC_ENABLE};

    status = Pmic_pwrSetLdoCfg(NULL, &ldoCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    status = Pmic_pwrSetLdoCfg(&pmicHandle, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    status = Pmic_pwrSetLdoCfg(NULL, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

void test_faultHandling_pwrSetLdoCfg_noValidParams(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrLdoCfg_t ldoCfg = {.validParams = 0U};

    status = Pmic_pwrSetLdoCfg(&pmicHandle, &ldoCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);
}

void test_faultHandling_pwrGetLdoCfg_nullParams(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrLdoCfg_t ldoCfg = {.validParams = PMIC_LDO_ENABLE_VALID};

    status = Pmic_pwrGetLdoCfg(NULL, &ldoCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    status = Pmic_pwrGetLdoCfg(&pmicHandle, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    status = Pmic_pwrGetLdoCfg(NULL, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

void test_faultHandling_pwrGetLdoCfg_noValidParams(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrLdoCfg_t ldoCfg = {.validParams = 0U};

    status = Pmic_pwrGetLdoCfg(&pmicHandle, &ldoCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);
}

void test_functionality_pwrSetGetLdoCfg_enable(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrLdoCfg_t expLdoCfg = {.validParams = PMIC_LDO_ENABLE_VALID};
    Pmic_PwrLdoCfg_t actLdoCfg = {.validParams = PMIC_LDO_ENABLE_VALID};

    // Enable LDO
    expLdoCfg.enable = PMIC_ENABLE;
    status = Pmic_pwrSetLdoCfg(&pmicHandle, &expLdoCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get actual LDO configuration and compare expected vs. actual
    status = Pmic_pwrGetLdoCfg(&pmicHandle, &actLdoCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(expLdoCfg.enable, actLdoCfg.enable);

    // Disable LDO
    expLdoCfg.enable = PMIC_DISABLE;
    status = Pmic_pwrSetLdoCfg(&pmicHandle, &expLdoCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get actual LDO configuration and compare expected vs. actual
    status = Pmic_pwrGetLdoCfg(&pmicHandle, &actLdoCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(expLdoCfg.enable, actLdoCfg.enable);
}

void test_functionality_pwrSetGetLdoCfg_mode(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrLdoCfg_t expLdoCfg = {.validParams = PMIC_LDO_MODE_VALID};
    Pmic_PwrLdoCfg_t actLdoCfg = {.validParams = PMIC_LDO_MODE_VALID};

    // Test Pmic_pwrSetLdoCfg() API error handling for out of bounds value
    expLdoCfg.mode = PMIC_LDO_BYP_CONFIG_MAX + 1U;
    status = Pmic_pwrSetLdoCfg(&pmicHandle, &expLdoCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

    // Set mode to be Bypass mode
    expLdoCfg.mode = PMIC_BYPASS_MODE;
    status = Pmic_pwrSetLdoCfg(&pmicHandle, &expLdoCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get actual LDO configuration and compare expected vs. actual
    status = Pmic_pwrGetLdoCfg(&pmicHandle, &actLdoCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(expLdoCfg.mode, actLdoCfg.mode);

    // Set mode to be LDO mode
    expLdoCfg.mode = PMIC_LDO_MODE;
    status = Pmic_pwrSetLdoCfg(&pmicHandle, &expLdoCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get actual LDO configuration and compare expected vs. actual
    status = Pmic_pwrGetLdoCfg(&pmicHandle, &actLdoCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(expLdoCfg.mode, actLdoCfg.mode);
}

void test_functionality_pwrSetGetLdoCfg_vset(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrLdoCfg_t expLdoCfg = {.validParams = PMIC_LDO_VSET_VALID};
    Pmic_PwrLdoCfg_t actLdoCfg = {.validParams = PMIC_LDO_VSET_VALID};

    // Test Pmic_pwrSetLdoCfg() API error handling for out of bounds value
    expLdoCfg.vset = PMIC_LDO_VSET_MAX + 1U;
    status = Pmic_pwrSetLdoCfg(&pmicHandle, &expLdoCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);
    expLdoCfg.vset = PMIC_LDO_VSET_MIN - 1U;
    status = Pmic_pwrSetLdoCfg(&pmicHandle, &expLdoCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

    // For each valid LDO VSET value...
    for (uint8_t vset = PMIC_LDO_VSET_MIN; vset <= PMIC_LDO_VSET_MAX; vset++)
    {
        // Set VSET value
        expLdoCfg.vset = vset;
        status = Pmic_pwrSetLdoCfg(&pmicHandle, &expLdoCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Get actual LDO configurations and compare expected vs. actual
        status = Pmic_pwrGetLdoCfg(&pmicHandle, &actLdoCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        TEST_ASSERT_EQUAL(expLdoCfg.vset, actLdoCfg.vset);
    }
}

void test_functionality_pwrSetGetLdoCfg_vmonOnly(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrLdoCfg_t expLdoCfg = {.validParams = PMIC_LDO_VMON_ONLY_VALID};
    Pmic_PwrLdoCfg_t actLdoCfg = {.validParams = PMIC_LDO_VMON_ONLY_VALID};

    // Set LDO VMON_ONLY to true
    expLdoCfg.vmonOnly = (bool)true;
    status = Pmic_pwrSetLdoCfg(&pmicHandle, &expLdoCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get actual LDO configuration and compare expected vs. actual
    status = Pmic_pwrGetLdoCfg(&pmicHandle, &actLdoCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(expLdoCfg.vmonOnly, actLdoCfg.vmonOnly);

    // Set LDO VMON_ONLY to false
    expLdoCfg.vmonOnly = (bool)false;
    status = Pmic_pwrSetLdoCfg(&pmicHandle, &expLdoCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get actual LDO configuration and compare expected vs. actual
    status = Pmic_pwrGetLdoCfg(&pmicHandle, &actLdoCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(expLdoCfg.vmonOnly, actLdoCfg.vmonOnly);
}

void test_functionality_pwrSetGetLdoCfg_dischargeEn(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrLdoCfg_t expLdoCfg = {.validParams = PMIC_LDO_DISCHARGE_EN_VALID};
    Pmic_PwrLdoCfg_t actLdoCfg = {.validParams = PMIC_LDO_DISCHARGE_EN_VALID};

    // Set discharge enable to true
    expLdoCfg.dischargeEn = (bool)true;
    status = Pmic_pwrSetLdoCfg(&pmicHandle, &expLdoCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get actual LDO configuration and compare expected vs. actual
    status = Pmic_pwrGetLdoCfg(&pmicHandle, &actLdoCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(expLdoCfg.dischargeEn, actLdoCfg.dischargeEn);

    // Set discharge enable to false
    expLdoCfg.dischargeEn = (bool)false;
    status = Pmic_pwrSetLdoCfg(&pmicHandle, &expLdoCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get actual LDO configuration and compare expected vs. actual
    status = Pmic_pwrGetLdoCfg(&pmicHandle, &actLdoCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(expLdoCfg.dischargeEn, actLdoCfg.dischargeEn);
}

void test_functionality_pwrSetGetLdoCfg_dischargeSel(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrLdoCfg_t expLdoCfg = {.validParams = PMIC_LDO_DISCHARGE_SEL_VALID};
    Pmic_PwrLdoCfg_t actLdoCfg = {.validParams = PMIC_LDO_DISCHARGE_SEL_VALID};

    // Test Pmic_pwrSetLdoCfg() API error handling for out of bounds value
    expLdoCfg.dischargeSel = PMIC_LDO_DISCHARGE_SEL_MAX + 1U;
    status = Pmic_pwrSetLdoCfg(&pmicHandle, &expLdoCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

    // For each valid LDO discharge select value...
    for (uint8_t dischargeSel = PMIC_LDO_DISCHARGE_SEL_50K_OHM; dischargeSel <= PMIC_LDO_DISCHARGE_SEL_MAX; dischargeSel++)
    {
        // Set discharge select
        expLdoCfg.dischargeSel = dischargeSel;
        status = Pmic_pwrSetLdoCfg(&pmicHandle, &expLdoCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Get actual LDO configuration and compare expected vs. actual
        status = Pmic_pwrGetLdoCfg(&pmicHandle, &actLdoCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        TEST_ASSERT_EQUAL(expLdoCfg.dischargeSel, actLdoCfg.dischargeSel);
    }
}

void test_functionality_pwrSetGetLdoCfg_deglitchSel(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrLdoCfg_t expLdoCfg = {.validParams = PMIC_LDO_DEGLITCH_SEL_VALID};
    Pmic_PwrLdoCfg_t actLdoCfg = {.validParams = PMIC_LDO_DEGLITCH_SEL_VALID};

    // Test Pmic_pwrSetLdoCfg() API error handling for out of bounds value
    expLdoCfg.deglitchSel = PMIC_LDO_DEGLITCH_SEL_MAX + 1U;
    status = Pmic_pwrSetLdoCfg(&pmicHandle, &expLdoCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

    // For each valid LDO deglitch select value...
    for (uint8_t deglitchSel = PMIC_LDO_DEGLITCH_SEL_4_US; deglitchSel <= PMIC_LDO_DEGLITCH_SEL_MAX; deglitchSel++)
    {
        // Set deglitch select
        expLdoCfg.deglitchSel = deglitchSel;
        status = Pmic_pwrSetLdoCfg(&pmicHandle, &expLdoCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Get actual LDO configuration and compare expected vs. actual
        status = Pmic_pwrGetLdoCfg(&pmicHandle, &actLdoCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        TEST_ASSERT_EQUAL(expLdoCfg.deglitchSel, actLdoCfg.deglitchSel);
    }
}

void test_functionality_pwrSetGetLdoCfg_uvThr(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrLdoCfg_t expLdoCfg = {.validParams = PMIC_LDO_UV_THR_VALID};
    Pmic_PwrLdoCfg_t actLdoCfg = {.validParams = PMIC_LDO_UV_THR_VALID};

    // Test Pmic_pwrSetLdoCfg() API error handling for out of bounds value
    expLdoCfg.uvThr = PMIC_LDO_UV_THR_MAX + 1U;
    status = Pmic_pwrSetLdoCfg(&pmicHandle, &expLdoCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

    // For each valid LDO UV threshold value...
    for (uint8_t uvThr = PMIC_LDO_UV_THR_3P5_PCT; uvThr <= PMIC_LDO_UV_THR_MAX; uvThr++)
    {
        // Set UV threshold
        expLdoCfg.uvThr = uvThr;
        status = Pmic_pwrSetLdoCfg(&pmicHandle, &expLdoCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Get actual LDO configuration and compare expected vs. actual
        status = Pmic_pwrGetLdoCfg(&pmicHandle, &actLdoCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        TEST_ASSERT_EQUAL(expLdoCfg.uvThr, actLdoCfg.uvThr);
    }
}

void test_functionality_pwrSetGetLdoCfg_ovThr(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrLdoCfg_t expLdoCfg = {.validParams = PMIC_LDO_OV_THR_VALID};
    Pmic_PwrLdoCfg_t actLdoCfg = {.validParams = PMIC_LDO_OV_THR_VALID};

    // Test Pmic_pwrSetLdoCfg() API error handling for out of bounds value
    expLdoCfg.ovThr = PMIC_LDO_OV_THR_MAX + 1U;
    status = Pmic_pwrSetLdoCfg(&pmicHandle, &expLdoCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

    // For each valid LDO OV threshold value...
    for (uint8_t ovThr = PMIC_LDO_OV_THR_4_PCT; ovThr <= PMIC_LDO_OV_THR_MAX; ovThr++)
    {
        // Set OV threshold
        expLdoCfg.ovThr = ovThr;
        status = Pmic_pwrSetLdoCfg(&pmicHandle, &expLdoCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Get actual LDO configuration and compare expected vs. actual
        status = Pmic_pwrGetLdoCfg(&pmicHandle, &actLdoCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        TEST_ASSERT_EQUAL(expLdoCfg.ovThr, actLdoCfg.ovThr);
    }
}

void test_functionality_pwrSetGetLdoCfg_ilimSel(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrLdoCfg_t expLdoCfg = {.validParams = PMIC_LDO_ILIM_SEL_VALID};
    Pmic_PwrLdoCfg_t actLdoCfg = {.validParams = PMIC_LDO_ILIM_SEL_VALID};

    // Test Pmic_pwrSetLdoCfg() API error handling for out of bounds value
    expLdoCfg.ilimSel = PMIC_LDO_ILIM_SEL_MAX + 1U;
    status = Pmic_pwrSetLdoCfg(&pmicHandle, &expLdoCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

    // For each valid ILIM selection threshold value...
    for (uint8_t ilimSel = PMIC_LDO_ILIM_SEL_200_MA; ilimSel <= PMIC_LDO_ILIM_SEL_MAX; ilimSel++)
    {
        // Set ILIM selection
        expLdoCfg.ilimSel = ilimSel;
        status = Pmic_pwrSetLdoCfg(&pmicHandle, &expLdoCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Get actual LDO configuration and compare expected vs. actual
        status = Pmic_pwrGetLdoCfg(&pmicHandle, &actLdoCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        TEST_ASSERT_EQUAL(expLdoCfg.ilimSel, actLdoCfg.ilimSel);
    }
}

void test_functionality_pwrSetGetLdoCfg_ovpSel(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrLdoCfg_t expLdoCfg = {.validParams = PMIC_LDO_OVP_SEL_VALID};
    Pmic_PwrLdoCfg_t actLdoCfg = {.validParams = PMIC_LDO_OVP_SEL_VALID};

    // Test Pmic_pwrSetLdoCfg() API error handling for out of bounds value
    expLdoCfg.ovpSel = PMIC_REGULATOR_FAULT_RESPONSE_MAX + 1U;
    status = Pmic_pwrSetLdoCfg(&pmicHandle, &expLdoCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

    // For each valid OVP fault response value...
    for (uint8_t ovpSel = PMIC_ASSERT_NINT_PIN; ovpSel <= PMIC_REGULATOR_FAULT_RESPONSE_MAX; ovpSel++)
    {
        // Set OVP fault response
        expLdoCfg.ovpSel = ovpSel;
        status = Pmic_pwrSetLdoCfg(&pmicHandle, &expLdoCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Get actual LDO configuration and compare expected vs. actual
        status = Pmic_pwrGetLdoCfg(&pmicHandle, &actLdoCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        TEST_ASSERT_EQUAL(expLdoCfg.ovpSel, actLdoCfg.ovpSel);
    }
}

void test_functionality_pwrSetGetLdoCfg_ovSel(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrLdoCfg_t expLdoCfg = {.validParams = PMIC_LDO_OV_SEL_VALID};
    Pmic_PwrLdoCfg_t actLdoCfg = {.validParams = PMIC_LDO_OV_SEL_VALID};

    // Test Pmic_pwrSetLdoCfg() API error handling for out of bounds value
    expLdoCfg.ovSel = PMIC_REGULATOR_FAULT_RESPONSE_MAX + 1U;
    status = Pmic_pwrSetLdoCfg(&pmicHandle, &expLdoCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

    // For each valid OV fault response value...
    for (uint8_t ovSel = PMIC_ASSERT_NINT_PIN; ovSel <= PMIC_REGULATOR_FAULT_RESPONSE_MAX; ovSel++)
    {
        // Set OV fault response
        expLdoCfg.ovSel = ovSel;
        status = Pmic_pwrSetLdoCfg(&pmicHandle, &expLdoCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Get actual LDO configuration and compare expected vs. actual
        status = Pmic_pwrGetLdoCfg(&pmicHandle, &actLdoCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        TEST_ASSERT_EQUAL(expLdoCfg.ovSel, actLdoCfg.ovSel);
    }
}

void test_functionality_pwrSetGetLdoCfg_uvSel(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrLdoCfg_t expLdoCfg = {.validParams = PMIC_LDO_UV_SEL_VALID};
    Pmic_PwrLdoCfg_t actLdoCfg = {.validParams = PMIC_LDO_UV_SEL_VALID};

    // Test Pmic_pwrSetLdoCfg() API error handling for out of bounds value
    expLdoCfg.uvSel = PMIC_REGULATOR_FAULT_RESPONSE_MAX + 1U;
    status = Pmic_pwrSetLdoCfg(&pmicHandle, &expLdoCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

    // For each valid UV fault response value...
    for (uint8_t uvSel = PMIC_ASSERT_NINT_PIN; uvSel <= PMIC_REGULATOR_FAULT_RESPONSE_MAX; uvSel++)
    {
        // Set UV fault response
        expLdoCfg.uvSel = uvSel;
        status = Pmic_pwrSetLdoCfg(&pmicHandle, &expLdoCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Get actual LDO configuration and compare expected vs. actual
        status = Pmic_pwrGetLdoCfg(&pmicHandle, &actLdoCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        TEST_ASSERT_EQUAL(expLdoCfg.uvSel, actLdoCfg.uvSel);
    }
}

void test_functionality_pwrSetGetLdoCfg_scSel(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrLdoCfg_t expLdoCfg = {.validParams = PMIC_LDO_SC_SEL_VALID};
    Pmic_PwrLdoCfg_t actLdoCfg = {.validParams = PMIC_LDO_SC_SEL_VALID};

    // Test Pmic_pwrSetLdoCfg() API error handling for out of bounds value
    expLdoCfg.scSel = PMIC_REGULATOR_FAULT_RESPONSE_MAX + 1U;
    status = Pmic_pwrSetLdoCfg(&pmicHandle, &expLdoCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

    // For each valid SC fault response value...
    for (uint8_t scSel = PMIC_ASSERT_NINT_PIN; scSel <= PMIC_REGULATOR_FAULT_RESPONSE_MAX; scSel++)
    {
        // Set UV fault response
        expLdoCfg.scSel = scSel;
        status = Pmic_pwrSetLdoCfg(&pmicHandle, &expLdoCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Get actual LDO configuration and compare expected vs. actual
        status = Pmic_pwrGetLdoCfg(&pmicHandle, &actLdoCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        TEST_ASSERT_EQUAL(expLdoCfg.scSel, actLdoCfg.scSel);
    }
}

void test_functionality_pwrSetGetLdoCfg_rvConf(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrLdoCfg_t expLdoCfg = {.validParams = PMIC_LDO_RV_CONF_VALID};
    Pmic_PwrLdoCfg_t actLdoCfg = {.validParams = PMIC_LDO_RV_CONF_VALID};

    // Test Pmic_pwrSetLdoCfg() API error handling for out of bounds value
    expLdoCfg.rvConf = PMIC_LDO_RV_CONF_MAX + 1U;
    status = Pmic_pwrSetLdoCfg(&pmicHandle, &expLdoCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

    // Configure LDO to wait for rail to discharge before powering up
    expLdoCfg.rvConf = PMIC_LDO_RV_DISCHARGE;
    status = Pmic_pwrSetLdoCfg(&pmicHandle, &expLdoCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get actual LDO configuration and compare expected vs. actual
    status = Pmic_pwrGetLdoCfg(&pmicHandle, &actLdoCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(expLdoCfg.rvConf, actLdoCfg.rvConf);

    // Configure LDO to ignore residual voltage during power up
    expLdoCfg.rvConf = PMIC_LDO_RV_IGNORE;
    status = Pmic_pwrSetLdoCfg(&pmicHandle, &expLdoCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get actual LDO configuration and compare expected vs. actual
    status = Pmic_pwrGetLdoCfg(&pmicHandle, &actLdoCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(expLdoCfg.rvConf, actLdoCfg.rvConf);
}

void test_faultHandling_pwrGetRsrcStat_nullParams(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrRsrcStat_t pwrRsrcStat = {.resource = PMIC_BUCK1};

    status = Pmic_pwrGetRsrcStat(NULL, &pwrRsrcStat);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    status = Pmic_pwrGetRsrcStat(&pmicHandle, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    status = Pmic_pwrGetRsrcStat(NULL, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

void test_faultHandling_pwrGetRsrcStat_invalidResource(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrRsrcStat_t pwrRsrcStat = {.resource = PMIC_POWER_RESOURCE_MAX + 1U};

    status = Pmic_pwrGetRsrcStat(&pmicHandle, &pwrRsrcStat);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);
}

void test_functionality_pwrGetRsrcStat(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrRsrcStat_t buck1Stat = {.resource = PMIC_BUCK1};
    Pmic_PwrBuckCfg_t buckCfg = {.validParams = PMIC_BUCK_ENABLE_VALID, .resource = PMIC_BUCK1};

    // Enable BUCK1 regulator
    buckCfg.enable = PMIC_ENABLE;
    status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get BUCK1 regulator status and check whether BUCK1 is active
    status = Pmic_pwrGetRsrcStat(&pmicHandle, &buck1Stat);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL((bool)true, buck1Stat.active);

    // Disable BUCK1 regulator
    buckCfg.enable = PMIC_DISABLE;
    status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get BUCK1 regulator status and check whether BUCK1 is inactive
    status = Pmic_pwrGetRsrcStat(&pmicHandle, &buck1Stat);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL((bool)false, buck1Stat.active);
}

void test_faultHandling_pwrSetTsdCfg_nullParams(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrTsdCfg_t tsdCfg = {
        .validParams = PMIC_TWARN_STAY_IN_SAFE_STATE_VALID,
        .twarnStayInSafeState = (bool)true
    };

    status = Pmic_pwrSetTsdCfg(NULL, &tsdCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    status = Pmic_pwrSetTsdCfg(&pmicHandle, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    status = Pmic_pwrSetTsdCfg(NULL, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

void test_faultHandling_pwrSetTsdCfg_noValidParams(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrTsdCfg_t tsdCfg = {.validParams = 0U};

    status = Pmic_pwrSetTsdCfg(&pmicHandle, &tsdCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);
}

void test_faultHandling_pwrGetTsdCfg_nullParams(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrTsdCfg_t tsdCfg = {.validParams = PMIC_TWARN_STAY_IN_SAFE_STATE_VALID};

    status = Pmic_pwrGetTsdCfg(NULL, &tsdCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    status = Pmic_pwrGetTsdCfg(&pmicHandle, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    status = Pmic_pwrGetTsdCfg(NULL, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

void test_faultHandling_pwrGetTsdCfg_noValidParams(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrTsdCfg_t tsdCfg = {.validParams = 0U};

    status = Pmic_pwrGetTsdCfg(&pmicHandle, &tsdCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);
}

void test_functionality_pwrSetGetTsdCfg_twarnStayInSafeState(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrTsdCfg_t expTsdCfg = {.validParams = PMIC_TWARN_STAY_IN_SAFE_STATE_VALID};
    Pmic_PwrTsdCfg_t actTsdCfg = {.validParams = PMIC_TWARN_STAY_IN_SAFE_STATE_VALID};

    // Enable PMIC to stay in SAFE state for as long as TWARN flag is active
    expTsdCfg.twarnStayInSafeState = (bool)true;
    status = Pmic_pwrSetTsdCfg(&pmicHandle, &expTsdCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get actual TSD configuration and compare expected vs. actual
    status = Pmic_pwrGetTsdCfg(&pmicHandle, &actTsdCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(expTsdCfg.twarnStayInSafeState, actTsdCfg.twarnStayInSafeState);

    // Enable PMIC to leave SAFE state regardless of TWARN flag
    expTsdCfg.twarnStayInSafeState = (bool)true;
    status = Pmic_pwrSetTsdCfg(&pmicHandle, &expTsdCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get actual TSD configuration and compare expected vs. actual
    status = Pmic_pwrGetTsdCfg(&pmicHandle, &actTsdCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(expTsdCfg.twarnStayInSafeState, actTsdCfg.twarnStayInSafeState);
}

void test_functionality_pwrSetGetTsdCfg_tsdImmLevel(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrTsdCfg_t expTsdCfg = {.validParams = PMIC_TSD_IMM_LEVEL_VALID};
    Pmic_PwrTsdCfg_t actTsdCfg = {.validParams = PMIC_TSD_IMM_LEVEL_VALID};

    // Test Pmic_pwrSetTsdCfg() API error handling for out of bounds value
    expTsdCfg.tsdImmLevel = PMIC_TSD_IMM_LEVEL_MAX + 1U;
    status = Pmic_pwrSetTsdCfg(&pmicHandle, &expTsdCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

    // Set immediate TSD level to be 150C
    expTsdCfg.tsdImmLevel = PMIC_TSD_IMM_LEVEL_150C;
    status = Pmic_pwrSetTsdCfg(&pmicHandle, &expTsdCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get actual TSD configuration and compare expected vs. actual
    status = Pmic_pwrGetTsdCfg(&pmicHandle, &actTsdCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(expTsdCfg.tsdImmLevel, actTsdCfg.tsdImmLevel);

    // Set immediate TSD level to be 160C
    expTsdCfg.tsdImmLevel = PMIC_TSD_IMM_LEVEL_160C;
    status = Pmic_pwrSetTsdCfg(&pmicHandle, &expTsdCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get actual TSD configuration and compare expected vs. actual
    status = Pmic_pwrGetTsdCfg(&pmicHandle, &actTsdCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(expTsdCfg.tsdImmLevel, actTsdCfg.tsdImmLevel);
}

void test_functionality_pwrSetGetTsdCfg_twarnLevel(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrTsdCfg_t expTsdCfg = {.validParams = PMIC_TWARN_LEVEL_VALID};
    Pmic_PwrTsdCfg_t actTsdCfg = {.validParams = PMIC_TWARN_LEVEL_VALID};

    // Test Pmic_pwrSetTsdCfg() API error handling for out of bounds value
    expTsdCfg.twarnLevel = PMIC_TWARN_LEVEL_MAX + 1U;
    status = Pmic_pwrSetTsdCfg(&pmicHandle, &expTsdCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

    // Set TWARN level to be 130C
    expTsdCfg.twarnLevel = PMIC_TWARN_LEVEL_130C;
    status = Pmic_pwrSetTsdCfg(&pmicHandle, &expTsdCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get actual TSD configuration and compare expected vs. actual
    status = Pmic_pwrGetTsdCfg(&pmicHandle, &actTsdCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(expTsdCfg.twarnLevel, actTsdCfg.twarnLevel);

    // Set TWARN level to be 140C
    expTsdCfg.twarnLevel = PMIC_TWARN_LEVEL_140C;
    status = Pmic_pwrSetTsdCfg(&pmicHandle, &expTsdCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get actual TSD configuration and compare expected vs. actual
    status = Pmic_pwrGetTsdCfg(&pmicHandle, &actTsdCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(expTsdCfg.twarnLevel, actTsdCfg.twarnLevel);
}

void test_faultHandling_pwrSetBuckLdoSeqTrig_nullParams(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckLdoSeqTrig_t seqTrig = {
        .trigger = PMIC_BUCK1_TRIGGER_PWR_ON_BIT,
        .exclude = (bool)true
    };

    status = Pmic_pwrSetBuckLdoSeqTrig(NULL, &seqTrig, 1U);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    status = Pmic_pwrSetBuckLdoSeqTrig(&pmicHandle, NULL, 1U);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    status = Pmic_pwrSetBuckLdoSeqTrig(NULL, NULL, 1U);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

void test_faultHandling_pwrSetBuckLdoSeqTrig_invalidTrigger(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckLdoSeqTrig_t seqTrig = {
        .trigger = 0xFFU,
        .exclude = (bool)true
    };

    status = Pmic_pwrSetBuckLdoSeqTrig(&pmicHandle, &seqTrig, 1U);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);
}

void test_faultHandling_pwrGetBuckLdoSeqTrig_nullParams(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckLdoSeqTrig_t seqTrig = {.trigger = PMIC_BUCK1_TRIGGER_PWR_ON_BIT};

    status = Pmic_pwrGetBuckLdoSeqTrig(NULL, &seqTrig, 1U);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    status = Pmic_pwrGetBuckLdoSeqTrig(&pmicHandle, NULL, 1U);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    status = Pmic_pwrGetBuckLdoSeqTrig(NULL, NULL, 1U);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

void test_faultHandling_pwrGetBuckLdoSeqTrig_invalidTrigger(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckLdoSeqTrig_t seqTrig = {.trigger = 0xFFU};

    status = Pmic_pwrSetBuckLdoSeqTrig(&pmicHandle, &seqTrig, 1U);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);
}

void setGetBuckLdoSeqTrig(uint16_t trigger)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckLdoSeqTrig_t expSeqTrigCfg = {.trigger = trigger};
    Pmic_PwrBuckLdoSeqTrig_t actSeqTrigCfg = {.trigger = trigger};

    // Exclude trigger from power ON/OFF sequence logic
    expSeqTrigCfg.exclude = (bool)true;
    status = Pmic_pwrSetBuckLdoSeqTrig(&pmicHandle, &expSeqTrigCfg, 1U);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get actual sequence trigger configuration and compare expected vs actual
    status = Pmic_pwrGetBuckLdoSeqTrig(&pmicHandle, &actSeqTrigCfg, 1U);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(expSeqTrigCfg.exclude, actSeqTrigCfg.exclude);

    // Include trigger in power ON/OFF sequence logic
    expSeqTrigCfg.exclude = (bool)false;
    status = Pmic_pwrSetBuckLdoSeqTrig(&pmicHandle, &expSeqTrigCfg, 1U);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get actual sequence trigger configuration and compare expected vs actual
    status = Pmic_pwrGetBuckLdoSeqTrig(&pmicHandle, &actSeqTrigCfg, 1U);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(expSeqTrigCfg.exclude, actSeqTrigCfg.exclude);
}

void test_functionality_pwrSetGetBuckLdoSeqTrig_BUCK1_TRIGGER_PWR_ON_BIT(void)
{
    setGetBuckLdoSeqTrig(PMIC_BUCK1_TRIGGER_PWR_ON_BIT);
}

void test_functionality_pwrSetGetBuckLdoSeqTrig_BUCK1_TRIGGER_LDO_PG(void)
{
    setGetBuckLdoSeqTrig(PMIC_BUCK1_TRIGGER_LDO_PG);
}

void test_functionality_pwrSetGetBuckLdoSeqTrig_BUCK1_TRIGGER_BUCK3_PG(void)
{
    setGetBuckLdoSeqTrig(PMIC_BUCK1_TRIGGER_BUCK3_PG);
}

void test_functionality_pwrSetGetBuckLdoSeqTrig_BUCK1_TRIGGER_BUCK2_PG(void)
{
    setGetBuckLdoSeqTrig(PMIC_BUCK1_TRIGGER_BUCK2_PG);
}

void test_functionality_pwrSetGetBuckLdoSeqTrig_BUCK1_TRIGGER_GPIO_PIN(void)
{
    setGetBuckLdoSeqTrig(PMIC_BUCK1_TRIGGER_GPIO_PIN);
}

void test_functionality_pwrSetGetBuckLdoSeqTrig_BUCK1_TRIGGER_SEQ_PIN(void)
{
    setGetBuckLdoSeqTrig(PMIC_BUCK1_TRIGGER_SEQ_PIN);
}

void test_functionality_pwrSetGetBuckLdoSeqTrig_BUCK2_TRIGGER_PWR_ON_BIT(void)
{
    setGetBuckLdoSeqTrig(PMIC_BUCK2_TRIGGER_PWR_ON_BIT);
}

void test_functionality_pwrSetGetBuckLdoSeqTrig_BUCK2_TRIGGER_LDO_PG(void)
{
    setGetBuckLdoSeqTrig(PMIC_BUCK2_TRIGGER_LDO_PG);
}

void test_functionality_pwrSetGetBuckLdoSeqTrig_BUCK2_TRIGGER_BUCK3_PG(void)
{
    setGetBuckLdoSeqTrig(PMIC_BUCK2_TRIGGER_BUCK3_PG);
}

void test_functionality_pwrSetGetBuckLdoSeqTrig_BUCK2_TRIGGER_BUCK1_PG(void)
{
    setGetBuckLdoSeqTrig(PMIC_BUCK2_TRIGGER_BUCK1_PG);
}

void test_functionality_pwrSetGetBuckLdoSeqTrig_BUCK2_TRIGGER_GPIO_PIN(void)
{
    setGetBuckLdoSeqTrig(PMIC_BUCK2_TRIGGER_GPIO_PIN);
}

void test_functionality_pwrSetGetBuckLdoSeqTrig_BUCK2_TRIGGER_SEQ_PIN(void)
{
    setGetBuckLdoSeqTrig(PMIC_BUCK2_TRIGGER_SEQ_PIN);
}

void test_functionality_pwrSetGetBuckLdoSeqTrig_BUCK3_TRIGGER_PWR_ON_BIT(void)
{
    setGetBuckLdoSeqTrig(PMIC_BUCK3_TRIGGER_PWR_ON_BIT);
}

void test_functionality_pwrSetGetBuckLdoSeqTrig_BUCK3_TRIGGER_LDO_PG(void)
{
    setGetBuckLdoSeqTrig(PMIC_BUCK3_TRIGGER_LDO_PG);
}

void test_functionality_pwrSetGetBuckLdoSeqTrig_BUCK3_TRIGGER_BUCK2_PG(void)
{
    setGetBuckLdoSeqTrig(PMIC_BUCK3_TRIGGER_BUCK2_PG);
}

void test_functionality_pwrSetGetBuckLdoSeqTrig_BUCK3_TRIGGER_BUCK1_PG(void)
{
    setGetBuckLdoSeqTrig(PMIC_BUCK3_TRIGGER_BUCK1_PG);
}

void test_functionality_pwrSetGetBuckLdoSeqTrig_BUCK3_TRIGGER_GPIO_PIN(void)
{
    setGetBuckLdoSeqTrig(PMIC_BUCK3_TRIGGER_GPIO_PIN);
}

void test_functionality_pwrSetGetBuckLdoSeqTrig_BUCK3_TRIGGER_SEQ_PIN(void)
{
    setGetBuckLdoSeqTrig(PMIC_BUCK3_TRIGGER_SEQ_PIN);
}

void test_functionality_pwrSetGetBuckLdoSeqTrig_LDO_TRIGGER_PWR_ON_BIT(void)
{
    setGetBuckLdoSeqTrig(PMIC_LDO_TRIGGER_PWR_ON_BIT);
}

void test_functionality_pwrSetGetBuckLdoSeqTrig_LDO_TRIGGER_BUCK3_PG(void)
{
    setGetBuckLdoSeqTrig(PMIC_LDO_TRIGGER_BUCK3_PG);
}

void test_functionality_pwrSetGetBuckLdoSeqTrig_LDO_TRIGGER_BUCK2_PG(void)
{
    setGetBuckLdoSeqTrig(PMIC_LDO_TRIGGER_BUCK2_PG);
}

void test_functionality_pwrSetGetBuckLdoSeqTrig_LDO_TRIGGER_BUCK1_PG(void)
{
    setGetBuckLdoSeqTrig(PMIC_LDO_TRIGGER_BUCK1_PG);
}

void test_functionality_pwrSetGetBuckLdoSeqTrig_LDO_TRIGGER_GPIO_PIN(void)
{
    setGetBuckLdoSeqTrig(PMIC_LDO_TRIGGER_GPIO_PIN);
}

void test_functionality_pwrSetGetBuckLdoSeqTrig_LDO_TRIGGER_SEQ_PIN(void)
{
    setGetBuckLdoSeqTrig(PMIC_LDO_TRIGGER_SEQ_PIN);
}

void test_functionality_pwrSetGetBuckLdoSeqTrig_multipleTriggers(void)
{
    // There are 24 unique sequence triggers and this test considers all of them
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckLdoSeqTrig_t expSeqTrigCfg[24U] = {
        {.trigger = PMIC_BUCK1_TRIGGER_PWR_ON_BIT},
        {.trigger = PMIC_BUCK1_TRIGGER_LDO_PG},
        {.trigger = PMIC_BUCK1_TRIGGER_BUCK3_PG},
        {.trigger = PMIC_BUCK1_TRIGGER_BUCK2_PG},
        {.trigger = PMIC_BUCK1_TRIGGER_GPIO_PIN},
        {.trigger = PMIC_BUCK1_TRIGGER_SEQ_PIN},
        {.trigger = PMIC_BUCK2_TRIGGER_PWR_ON_BIT},
        {.trigger = PMIC_BUCK2_TRIGGER_LDO_PG},
        {.trigger = PMIC_BUCK2_TRIGGER_BUCK3_PG},
        {.trigger = PMIC_BUCK2_TRIGGER_BUCK1_PG},
        {.trigger = PMIC_BUCK2_TRIGGER_GPIO_PIN},
        {.trigger = PMIC_BUCK2_TRIGGER_SEQ_PIN},
        {.trigger = PMIC_BUCK3_TRIGGER_PWR_ON_BIT},
        {.trigger = PMIC_BUCK3_TRIGGER_LDO_PG},
        {.trigger = PMIC_BUCK3_TRIGGER_BUCK2_PG},
        {.trigger = PMIC_BUCK3_TRIGGER_BUCK1_PG},
        {.trigger = PMIC_BUCK3_TRIGGER_GPIO_PIN},
        {.trigger = PMIC_BUCK3_TRIGGER_SEQ_PIN},
        {.trigger = PMIC_LDO_TRIGGER_PWR_ON_BIT},
        {.trigger = PMIC_LDO_TRIGGER_BUCK3_PG},
        {.trigger = PMIC_LDO_TRIGGER_BUCK2_PG},
        {.trigger = PMIC_LDO_TRIGGER_BUCK1_PG},
        {.trigger = PMIC_LDO_TRIGGER_GPIO_PIN},
        {.trigger = PMIC_LDO_TRIGGER_SEQ_PIN}
    };
    Pmic_PwrBuckLdoSeqTrig_t actSeqTrigCfg[24U] = {
        {.trigger = PMIC_BUCK1_TRIGGER_PWR_ON_BIT},
        {.trigger = PMIC_BUCK1_TRIGGER_LDO_PG},
        {.trigger = PMIC_BUCK1_TRIGGER_BUCK3_PG},
        {.trigger = PMIC_BUCK1_TRIGGER_BUCK2_PG},
        {.trigger = PMIC_BUCK1_TRIGGER_GPIO_PIN},
        {.trigger = PMIC_BUCK1_TRIGGER_SEQ_PIN},
        {.trigger = PMIC_BUCK2_TRIGGER_PWR_ON_BIT},
        {.trigger = PMIC_BUCK2_TRIGGER_LDO_PG},
        {.trigger = PMIC_BUCK2_TRIGGER_BUCK3_PG},
        {.trigger = PMIC_BUCK2_TRIGGER_BUCK1_PG},
        {.trigger = PMIC_BUCK2_TRIGGER_GPIO_PIN},
        {.trigger = PMIC_BUCK2_TRIGGER_SEQ_PIN},
        {.trigger = PMIC_BUCK3_TRIGGER_PWR_ON_BIT},
        {.trigger = PMIC_BUCK3_TRIGGER_LDO_PG},
        {.trigger = PMIC_BUCK3_TRIGGER_BUCK2_PG},
        {.trigger = PMIC_BUCK3_TRIGGER_BUCK1_PG},
        {.trigger = PMIC_BUCK3_TRIGGER_GPIO_PIN},
        {.trigger = PMIC_BUCK3_TRIGGER_SEQ_PIN},
        {.trigger = PMIC_LDO_TRIGGER_PWR_ON_BIT},
        {.trigger = PMIC_LDO_TRIGGER_BUCK3_PG},
        {.trigger = PMIC_LDO_TRIGGER_BUCK2_PG},
        {.trigger = PMIC_LDO_TRIGGER_BUCK1_PG},
        {.trigger = PMIC_LDO_TRIGGER_GPIO_PIN},
        {.trigger = PMIC_LDO_TRIGGER_SEQ_PIN}
    };

    // Include trigger in power ON/OFF sequence logic
    for (uint8_t i = 0U; i < 24U; i++)
    {
        expSeqTrigCfg[i].exclude = (bool)false;
    }
    status = Pmic_pwrSetBuckLdoSeqTrig(&pmicHandle, expSeqTrigCfg, 24U);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get actual sequence trigger configuration and compare expected vs actual
    status = Pmic_pwrGetBuckLdoSeqTrig(&pmicHandle, actSeqTrigCfg, 24U);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    for (uint8_t i = 0U; i < 24U; i++)
    {
        TEST_ASSERT_EQUAL(expSeqTrigCfg[i].exclude, actSeqTrigCfg[i].exclude);
    }

    // Exclude trigger from power ON/OFF sequence logic
    for (uint8_t i = 0U; i < 24U; i++)
    {
        expSeqTrigCfg[i].exclude = (bool)true;
    }
    status = Pmic_pwrSetBuckLdoSeqTrig(&pmicHandle, expSeqTrigCfg, 24U);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get actual sequence trigger configuration and compare expected vs actual
    status = Pmic_pwrGetBuckLdoSeqTrig(&pmicHandle, actSeqTrigCfg, 24U);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    for (uint8_t i = 0U; i < 24U; i++)
    {
        TEST_ASSERT_EQUAL(expSeqTrigCfg[i].exclude, actSeqTrigCfg[i].exclude);
    }
}

void test_faultHandling_pwrSetBuckLdoSeqDly_nullParams(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckLdoSeqDly_t seqDlyCfg = {
        .validParams = PMIC_SEQ_DLY_OFF_VALID,
        .resource = PMIC_BUCK1,
        .seqDlyOff = PMIC_SEQ_DLY_1_MS
    };

    status = Pmic_pwrSetBuckLdoSeqDly(NULL, &seqDlyCfg, 1U);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    status = Pmic_pwrSetBuckLdoSeqDly(&pmicHandle, NULL, 1U);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    status = Pmic_pwrSetBuckLdoSeqDly(NULL, NULL, 1U);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

void test_faultHandling_pwrSetBuckLdoSeqDly_invalidResource(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckLdoSeqDly_t seqDlyCfg = {
        .validParams = PMIC_SEQ_DLY_OFF_VALID,
        .resource = PMIC_POWER_RESOURCE_MAX + 1U,
        .seqDlyOff = PMIC_SEQ_DLY_1_MS
    };

    status = Pmic_pwrSetBuckLdoSeqDly(&pmicHandle, &seqDlyCfg, 1U);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);
}

void test_faultHandling_pwrSetBuckLdoSeqDly_invalidSeqDlyOff(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckLdoSeqDly_t seqDlyCfg = {
        .validParams = PMIC_SEQ_DLY_OFF_VALID,
        .resource = PMIC_BUCK1,
        .seqDlyOff = PMIC_SEQ_DLY_MAX + 1U
    };

    status = Pmic_pwrSetBuckLdoSeqDly(&pmicHandle, &seqDlyCfg, 1U);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);
}

void test_faultHandling_pwrSetBuckLdoSeqDly_invalidSeqDlyOn(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckLdoSeqDly_t seqDlyCfg = {
        .validParams = PMIC_SEQ_DLY_ON_VALID,
        .resource = PMIC_BUCK1,
        .seqDlyOn = PMIC_SEQ_DLY_MAX + 1U
    };

    status = Pmic_pwrSetBuckLdoSeqDly(&pmicHandle, &seqDlyCfg, 1U);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);
}

void test_faultHandling_pwrGetBuckLdoSeqDly_nullParams(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckLdoSeqDly_t seqDlyCfg = {
        .validParams = PMIC_SEQ_DLY_OFF_VALID,
        .resource = PMIC_BUCK1
    };

    status = Pmic_pwrGetBuckLdoSeqDly(NULL, &seqDlyCfg, 1U);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    status = Pmic_pwrGetBuckLdoSeqDly(&pmicHandle, NULL, 1U);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    status = Pmic_pwrGetBuckLdoSeqDly(NULL, NULL, 1U);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

void test_faultHandling_pwrGetBuckLdoSeqDly_invalidResource(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckLdoSeqDly_t seqDlyCfg = {
        .validParams = PMIC_SEQ_DLY_OFF_VALID,
        .resource = PMIC_POWER_RESOURCE_MAX + 1U
    };

    status = Pmic_pwrGetBuckLdoSeqDly(&pmicHandle, &seqDlyCfg, 1U);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);
}

void test_functionality_pwrSetGetBuckLdoSeqDly_seqDlyOn(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckLdoSeqDly_t expSeqDlyCfg[PMIC_POWER_RESOURCE_MAX] = {
        {
            .validParams = PMIC_SEQ_DLY_ON_VALID,
            .resource = PMIC_BUCK1,
        },
        {
            .validParams = PMIC_SEQ_DLY_ON_VALID,
            .resource = PMIC_BUCK2,
        },
        {
            .validParams = PMIC_SEQ_DLY_ON_VALID,
            .resource = PMIC_BUCK3,
        },
        {
            .validParams = PMIC_SEQ_DLY_ON_VALID,
            .resource = PMIC_LDO,
        },
    };
    Pmic_PwrBuckLdoSeqDly_t actSeqDlyCfg[PMIC_POWER_RESOURCE_MAX] = {
        {
            .validParams = PMIC_SEQ_DLY_ON_VALID,
            .resource = PMIC_BUCK1,
        },
        {
            .validParams = PMIC_SEQ_DLY_ON_VALID,
            .resource = PMIC_BUCK2,
        },
        {
            .validParams = PMIC_SEQ_DLY_ON_VALID,
            .resource = PMIC_BUCK3,
        },
        {
            .validParams = PMIC_SEQ_DLY_ON_VALID,
            .resource = PMIC_LDO,
        },
    };

    // For each valid sequence delay ON value...
    for (uint8_t dlyOn = 0U; dlyOn <= PMIC_SEQ_DLY_MAX; dlyOn++)
    {
        // Set sequence delay ON value for each resource
        for (uint8_t rsrc = 0U; rsrc < PMIC_POWER_RESOURCE_MAX; rsrc++)
        {
            expSeqDlyCfg[rsrc].seqDlyOn = dlyOn;
        }
        status = Pmic_pwrSetBuckLdoSeqDly(&pmicHandle, expSeqDlyCfg, PMIC_POWER_RESOURCE_MAX);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Get actual sequence delay configuration and compare expected vs. actual
        status = Pmic_pwrGetBuckLdoSeqDly(&pmicHandle, actSeqDlyCfg, PMIC_POWER_RESOURCE_MAX);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        for (uint8_t rsrc = 0U; rsrc < PMIC_POWER_RESOURCE_MAX; rsrc++)
        {
            TEST_ASSERT_EQUAL(expSeqDlyCfg[rsrc].seqDlyOn, actSeqDlyCfg[rsrc].seqDlyOn);
        }
    }
}

void test_functionality_pwrSetGetBuckLdoSeqDly_seqDlyOff(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckLdoSeqDly_t expSeqDlyCfg[PMIC_POWER_RESOURCE_MAX] = {
        {
            .validParams = PMIC_SEQ_DLY_OFF_VALID,
            .resource = PMIC_BUCK1,
        },
        {
            .validParams = PMIC_SEQ_DLY_OFF_VALID,
            .resource = PMIC_BUCK2,
        },
        {
            .validParams = PMIC_SEQ_DLY_OFF_VALID,
            .resource = PMIC_BUCK3,
        },
        {
            .validParams = PMIC_SEQ_DLY_OFF_VALID,
            .resource = PMIC_LDO,
        },
    };
    Pmic_PwrBuckLdoSeqDly_t actSeqDlyCfg[PMIC_POWER_RESOURCE_MAX] = {
        {
            .validParams = PMIC_SEQ_DLY_OFF_VALID,
            .resource = PMIC_BUCK1,
        },
        {
            .validParams = PMIC_SEQ_DLY_OFF_VALID,
            .resource = PMIC_BUCK2,
        },
        {
            .validParams = PMIC_SEQ_DLY_OFF_VALID,
            .resource = PMIC_BUCK3,
        },
        {
            .validParams = PMIC_SEQ_DLY_OFF_VALID,
            .resource = PMIC_LDO,
        },
    };

    // For each valid sequence delay OFF value...
    for (uint8_t dlyOff = 0U; dlyOff <= PMIC_SEQ_DLY_MAX; dlyOff++)
    {
        // Set sequence delay OFF value for each resource
        for (uint8_t rsrc = 0U; rsrc < PMIC_POWER_RESOURCE_MAX; rsrc++)
        {
            expSeqDlyCfg[rsrc].seqDlyOff = dlyOff;
        }
        status = Pmic_pwrSetBuckLdoSeqDly(&pmicHandle, expSeqDlyCfg, PMIC_POWER_RESOURCE_MAX);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Get actual sequence delay configuration and compare expected vs. actual
        status = Pmic_pwrGetBuckLdoSeqDly(&pmicHandle, actSeqDlyCfg, PMIC_POWER_RESOURCE_MAX);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        for (uint8_t rsrc = 0U; rsrc < PMIC_POWER_RESOURCE_MAX; rsrc++)
        {
            TEST_ASSERT_EQUAL(expSeqDlyCfg[rsrc].seqDlyOff, actSeqDlyCfg[rsrc].seqDlyOff);
        }
    }
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
