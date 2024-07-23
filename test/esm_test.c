/**
 * @file esm_test.c
 *
 * @brief Source file for PMIC LLD ESM module testing.
 */
#include "esm_test.h"

#define RUN_ESM_TESTS() RUN_TEST(test_faultHandling_esmStartStop_nullParams);       \
                        RUN_TEST(test_faultHandling_esmGetStartStop_nullParams);    \
                        RUN_TEST(test_functionality_esmStartStop);                  \
                        RUN_TEST(test_faultHandling_esmSetCfg_nullParams);          \
                        RUN_TEST(test_faultHandling_esmSetCfg_noValidParams);       \
                        RUN_TEST(test_faultHandling_esmGetCfg_nullParams);          \
                        RUN_TEST(test_faultHandling_esmGetCfg_noValidParams);       \
                        RUN_TEST(test_functionality_esmSetGetCfg_enable);           \
                        RUN_TEST(test_functionality_esmSetGetCfg_mode);             \
                        RUN_TEST(test_functionality_esmSetGetCfg_errCntThr);        \
                        RUN_TEST(test_functionality_esmSetGetCfg_delay1);           \
                        RUN_TEST(test_functionality_esmSetGetCfg_delay2);           \
                        RUN_TEST(test_functionality_esmSetGetCfg_hmax);             \
                        RUN_TEST(test_functionality_esmSetGetCfg_hmin);             \
                        RUN_TEST(test_functionality_esmSetGetCfg_lmax);             \
                        RUN_TEST(test_functionality_esmSetGetCfg_lmin);             \
                        RUN_TEST(test_faulthandling_esmGetStat_nullParams);         \
                        RUN_TEST(test_faulthandling_esmGetStat_noValidParams);      \
                        RUN_TEST(test_faulthandling_esmClrStat_nullParams);         \
                        RUN_TEST(test_faulthandling_esmClrStat_noValidParams);      \
                        RUN_TEST(test_faulthandling_esmGetErrCnt_nullParams);       \
                        RUN_TEST(test_functionality_esm_levelMode);                 \
                        RUN_TEST(test_functionality_esm_pwmMode);                   \
                        RUN_TEST(test_errorDetection_esm_rstInt);                   \
                        RUN_TEST(test_errorDetection_esm_failInt);                  \
                        RUN_TEST(test_errorDetection_esm_pinInt)

Pmic_CoreHandle_t pmicHandle;

void esmTest(void *args)
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

    DebugP_log("\r\nWelcome to PMIC LLD ESM module testing\r\n\r\n");

    status = Pmic_init(&pmicCfg, &pmicHandle);

    if (status == PMIC_ST_SUCCESS)
    {
        DebugP_log("Running PMIC ESM tests...\r\n");
        UNITY_BEGIN();
        RUN_ESM_TESTS();
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

void test_faultHandling_esmStartStop_nullParams(void)
{
    int32_t status = Pmic_esmStartStop(NULL, PMIC_ESM_START);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

void test_faultHandling_esmGetStartStop_nullParams(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    bool start = (bool)false;

    status = Pmic_esmGetStartStop(NULL, &start);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    status = Pmic_esmGetStartStop(&pmicHandle, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    status = Pmic_esmGetStartStop(NULL, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

void test_functionality_esmStartStop(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    bool start = (bool)false;

    // Start ESM
    status = Pmic_esmStartStop(&pmicHandle, PMIC_ESM_START);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get actual ESM start/stop status and compare expected vs. actual value
    status = Pmic_esmGetStartStop(&pmicHandle, &start);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL((bool)true, start);

    // Stop ESM
    status = Pmic_esmStartStop(&pmicHandle, PMIC_ESM_STOP);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get actual ESM start/stop status and compare expected vs. actual value
    status = Pmic_esmGetStartStop(&pmicHandle, &start);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL((bool)false, start);
}

void test_faultHandling_esmSetCfg_nullParams(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_EsmCfg_t esmCfg = {.validParams = PMIC_ESM_ENABLE_VALID, .enable = (bool)true};

    status = Pmic_esmSetCfg(NULL, &esmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    status = Pmic_esmSetCfg(&pmicHandle, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    status = Pmic_esmSetCfg(NULL, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

void test_faultHandling_esmSetCfg_noValidParams(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_EsmCfg_t esmCfg = {.validParams = 0U};

    status = Pmic_esmSetCfg(&pmicHandle, &esmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);
}

void test_faultHandling_esmGetCfg_nullParams(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_EsmCfg_t esmCfg = {.validParams = PMIC_ESM_ENABLE_VALID};

    status = Pmic_esmGetCfg(NULL, &esmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    status = Pmic_esmGetCfg(&pmicHandle, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    status = Pmic_esmGetCfg(NULL, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

void test_faultHandling_esmGetCfg_noValidParams(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_EsmCfg_t esmCfg = {.validParams = 0U};

    status = Pmic_esmGetCfg(&pmicHandle, &esmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);
}

void test_functionality_esmSetGetCfg_enable(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_EsmCfg_t expEsmCfg = {.validParams = PMIC_ESM_ENABLE_VALID};
    Pmic_EsmCfg_t actEsmCfg = {.validParams = PMIC_ESM_ENABLE_VALID};

    // Enable the ESM
    expEsmCfg.enable = PMIC_ENABLE;
    status = Pmic_esmSetCfg(&pmicHandle, &expEsmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get the actual ESM enable and compare expected vs. actual value
    status = Pmic_esmGetCfg(&pmicHandle, &actEsmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(expEsmCfg.enable, actEsmCfg.enable);

    // Disable the ESM
    expEsmCfg.enable = PMIC_DISABLE;
    status = Pmic_esmSetCfg(&pmicHandle, &expEsmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get the actual ESM enable and compare expected vs. actual value
    status = Pmic_esmGetCfg(&pmicHandle, &actEsmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(expEsmCfg.enable, actEsmCfg.enable);
}

void test_functionality_esmSetGetCfg_mode(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_EsmCfg_t expEsmCfg = {.validParams = PMIC_ESM_MODE_VALID};
    Pmic_EsmCfg_t actEsmCfg = {.validParams = PMIC_ESM_MODE_VALID};

    // Set the ESM mode to be PWM mode
    expEsmCfg.mode = ESM_PWM_MODE;
    status = Pmic_esmSetCfg(&pmicHandle, &expEsmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get the actual ESM mode and compare expected vs. actual value
    status = Pmic_esmGetCfg(&pmicHandle, &actEsmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(expEsmCfg.mode, actEsmCfg.mode);

    // Set the ESM mode to be Level mode
    expEsmCfg.enable = ESM_LEVEL_MODE;
    status = Pmic_esmSetCfg(&pmicHandle, &expEsmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get the actual ESM mode and compare expected vs. actual value
    status = Pmic_esmGetCfg(&pmicHandle, &actEsmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(expEsmCfg.mode, actEsmCfg.mode);
}

void test_functionality_esmSetGetCfg_errCntThr(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_EsmCfg_t expEsmCfg = {.validParams = PMIC_ESM_ERR_CNT_THR_VALID};
    Pmic_EsmCfg_t actEsmCfg = {.validParams = PMIC_ESM_ERR_CNT_THR_VALID};

    // Check Pmic_esmSetCfg() API error handling for when errCntThr member is
    // greater than ESM_ERR_CNT_THR_MAX
    expEsmCfg.errCntThr = ESM_ERR_CNT_THR_MAX + 1U;
    status = Pmic_esmSetCfg(&pmicHandle, &expEsmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

    // Test all possible error count threshold values
    for (uint8_t i = 0U; i <= ESM_ERR_CNT_THR_MAX; i++)
    {
        // Set the error count threshold
        expEsmCfg.errCntThr = i;
        status = Pmic_esmSetCfg(&pmicHandle, &expEsmCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Get the actual error count threshold and compare expected vs. actual value
        status = Pmic_esmGetCfg(&pmicHandle, &actEsmCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        TEST_ASSERT_EQUAL(expEsmCfg.errCntThr, actEsmCfg.errCntThr);
    }
}

void test_functionality_esmSetGetCfg_delay1(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_EsmCfg_t expEsmCfg = {.validParams = PMIC_ESM_DELAY1_VALID};
    Pmic_EsmCfg_t actEsmCfg = {.validParams = PMIC_ESM_DELAY1_VALID};

    // Test all possible delay1 values
    for (uint16_t i = 0U; i <= UINT8_MAX; i++)
    {
        // Set the delay1 value
        expEsmCfg.delay1 = (uint8_t)i;
        status = Pmic_esmSetCfg(&pmicHandle, &expEsmCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Get the actual delay1 value and compare expected vs. actual
        status = Pmic_esmGetCfg(&pmicHandle, &actEsmCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        TEST_ASSERT_EQUAL(expEsmCfg.delay1, actEsmCfg.delay1);
    }
}

void test_functionality_esmSetGetCfg_delay2(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_EsmCfg_t expEsmCfg = {.validParams = PMIC_ESM_DELAY2_VALID};
    Pmic_EsmCfg_t actEsmCfg = {.validParams = PMIC_ESM_DELAY2_VALID};

    // Test all possible delay2 values
    for (uint16_t i = 0U; i <= UINT8_MAX; i++)
    {
        // Set the delay2 value
        expEsmCfg.delay2 = (uint8_t)i;
        status = Pmic_esmSetCfg(&pmicHandle, &expEsmCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Get the actual delay2 value and compare expected vs. actual
        status = Pmic_esmGetCfg(&pmicHandle, &actEsmCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        TEST_ASSERT_EQUAL(expEsmCfg.delay2, actEsmCfg.delay2);
    }
}

void test_functionality_esmSetGetCfg_hmax(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_EsmCfg_t expEsmCfg = {.validParams = PMIC_ESM_HMAX_VALID};
    Pmic_EsmCfg_t actEsmCfg = {.validParams = PMIC_ESM_HMAX_VALID};

    // Test all possible HMAX values
    for (uint16_t i = 0U; i <= UINT8_MAX; i++)
    {
        // Set the HMAX value
        expEsmCfg.hmax = (uint8_t)i;
        status = Pmic_esmSetCfg(&pmicHandle, &expEsmCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Get the actual HMAX value and compare expected vs. actual
        status = Pmic_esmGetCfg(&pmicHandle, &actEsmCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        TEST_ASSERT_EQUAL(expEsmCfg.hmax, actEsmCfg.hmax);
    }
}

void test_functionality_esmSetGetCfg_hmin(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_EsmCfg_t expEsmCfg = {.validParams = PMIC_ESM_HMIN_VALID};
    Pmic_EsmCfg_t actEsmCfg = {.validParams = PMIC_ESM_HMIN_VALID};

    // Test all possible HMIN values
    for (uint16_t i = 0U; i <= UINT8_MAX; i++)
    {
        // Set the HMIN value
        expEsmCfg.hmin = (uint8_t)i;
        status = Pmic_esmSetCfg(&pmicHandle, &expEsmCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Get the actual HMIN value and compare expected vs. actual
        status = Pmic_esmGetCfg(&pmicHandle, &actEsmCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        TEST_ASSERT_EQUAL(expEsmCfg.hmin, actEsmCfg.hmin);
    }
}

void test_functionality_esmSetGetCfg_lmax(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_EsmCfg_t expEsmCfg = {.validParams = PMIC_ESM_LMAX_VALID};
    Pmic_EsmCfg_t actEsmCfg = {.validParams = PMIC_ESM_LMAX_VALID};

    // Test all possible LMAX values
    for (uint16_t i = 0U; i <= UINT8_MAX; i++)
    {
        // Set the LMAX value
        expEsmCfg.lmax = (uint8_t)i;
        status = Pmic_esmSetCfg(&pmicHandle, &expEsmCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Get the actual LMAX value and compare expected vs. actual
        status = Pmic_esmGetCfg(&pmicHandle, &actEsmCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        TEST_ASSERT_EQUAL(expEsmCfg.lmax, actEsmCfg.lmax);
    }
}

void test_functionality_esmSetGetCfg_lmin(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_EsmCfg_t expEsmCfg = {.validParams = PMIC_ESM_LMIN_VALID};
    Pmic_EsmCfg_t actEsmCfg = {.validParams = PMIC_ESM_LMIN_VALID};

    // Test all possible LMIN values
    for (uint16_t i = 0U; i <= UINT8_MAX; i++)
    {
        // Set the LMIN value
        expEsmCfg.lmin = (uint8_t)i;
        status = Pmic_esmSetCfg(&pmicHandle, &expEsmCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Get the actual LMIN value and compare expected vs. actual
        status = Pmic_esmGetCfg(&pmicHandle, &actEsmCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        TEST_ASSERT_EQUAL(expEsmCfg.lmin, actEsmCfg.lmin);
    }
}

void test_faulthandling_esmGetStat_nullParams(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_EsmStat_t esmStat = {.validParams = PMIC_ESM_STATUS_ALL_VALID};

    status = Pmic_esmGetStat(NULL, &esmStat);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    status = Pmic_esmGetStat(&pmicHandle, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    status = Pmic_esmGetStat(NULL, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

void test_faulthandling_esmGetStat_noValidParams(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_EsmStat_t esmStat = {.validParams = 0U};

    status = Pmic_esmGetStat(&pmicHandle, &esmStat);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);
}

void test_faulthandling_esmClrStat_nullParams(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_EsmStat_t esmStat = {.validParams = PMIC_ESM_STATUS_ALL_VALID};

    status = Pmic_esmClrStat(NULL, &esmStat);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    status = Pmic_esmClrStat(&pmicHandle, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    status = Pmic_esmClrStat(NULL, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

void test_faulthandling_esmClrStat_noValidParams(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_EsmStat_t esmStat = {.validParams = 0U};

    status = Pmic_esmClrStat(&pmicHandle, &esmStat);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);
}

void test_faulthandling_esmGetErrCnt_nullParams(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t errCnt = 0U;

    status = Pmic_esmGetErrCnt(NULL, &errCnt);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    status = Pmic_esmGetErrCnt(&pmicHandle, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    status = Pmic_esmGetErrCnt(NULL, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

void test_functionality_esm_levelMode(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_EsmCfg_t esmCfg = {
        .validParams = (PMIC_ESM_ENABLE_VALID | PMIC_ESM_MODE_VALID | PMIC_ESM_ERR_CNT_THR_VALID |
            PMIC_ESM_DELAY1_VALID | PMIC_ESM_DELAY2_VALID),
        .enable = PMIC_ENABLE,
        .mode = ESM_LEVEL_MODE,
        .errCntThr = 1U,
        .delay1 = 0xFFU,
        .delay2 = 0xFFU
    };
    Pmic_EsmStat_t esmStat = {.validParams = PMIC_ESM_STATUS_ALL_VALID};

    // Stop the ESM

    // Set ESM configurations for level mode

    // Set MCU GPIO pin configurations

    // Clear any ESM error statuses

    // Drive the ESM input pin high

    // Start the ESM

    // While five seconds has not elapsed and no ESM errors have been detected...

        // Wait one second

        // Check for any ESM errors

    // Stop the ESM

}

void test_functionality_esm_pwmMode(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_EsmCfg_t esmCfg = {
        .validParams = (PMIC_ESM_ENABLE_VALID | PMIC_ESM_MODE_VALID | PMIC_ESM_ERR_CNT_THR_VALID |
            PMIC_ESM_DELAY1_VALID | PMIC_ESM_DELAY2_VALID | PMIC_ESM_HMAX_VALID | PMIC_ESM_HMIN_VALID |
            PMIC_ESM_LMAX_VALID | PMIC_ESM_LMIN_VALID),
        .enable = PMIC_ENABLE,
        .mode = ESM_LEVEL_MODE,
        .errCntThr = 1U,
        .delay1 = 0xFFU,
        .delay2 = 0xFFU,
        .hmax = 0xFFU,
        .hmin = 0xFFU,
        .lmax = 0xFFU,
        .lmin = 0xFFU
    };
    Pmic_EsmStat_t esmStat = {.validParams = PMIC_ESM_STATUS_ALL_VALID};

    // Stop the ESM

    // Set ESM configurations for PWM mode

    // Set MCU PWM pin configurations

    // Clear any ESM error statuses

    // Drive the ESM input pin via the PWM signal

    // Start the ESM

    // While five seconds has not elapsed and no ESM errors have been detected...

        // Wait one second

        // Check for any ESM errors

    // Stop the ESM

}

void test_errorDetection_esm_rstInt(void)
{

}

void test_errorDetection_esm_failInt(void)
{

}

void test_errorDetection_esm_pinInt(void)
{

}
