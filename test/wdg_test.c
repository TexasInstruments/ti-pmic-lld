/**
 * @file wdg_test.c
 *
 * @brief Source file for PMIC LLD Watchdog module testing.
 */
#include "wdg_test.h"

#define RUN_WDG_TESTS()     RUN_TEST(test_faultHandling_wdgGetEnableStat_nullParams);       \
                            RUN_TEST(test_faultHandling_wdgDisable_nullParams);             \
                            RUN_TEST(test_faultHandling_wdgEnable_nullParams);              \
                            RUN_TEST(test_functionality_wdgEnableDisable);                  \
                            RUN_TEST(test_faultHandling_wdgGetPwrHold_nullParams);          \
                            RUN_TEST(test_faultHandling_wdgSetPwrHold_nullParams);          \
                            RUN_TEST(test_functionality_wdgSetClrPwrHold);                  \
                            RUN_TEST(test_faultHandling_wdgGetRetLongWin_nullParams);       \
                            RUN_TEST(test_faultHandling_wdgSetRetLongWin_nullParams);       \
                            RUN_TEST(test_functionality_wdgSetClrRetLongWin);               \
                            RUN_TEST(test_faultHandling_wdgGetCfg_nullParams);              \
                            RUN_TEST(test_faultHandling_wdgGetCfg_noValidParams);           \
                            RUN_TEST(test_faultHandling_wdgSetCfg_nullParams);              \
                            RUN_TEST(test_faultHandling_wdgSetCfg_noValidParams);           \
                            RUN_TEST(test_functionality_wdgSetGetCfg_rstEn);                \
                            RUN_TEST(test_functionality_wdgSetGetCfg_mode);                 \
                            RUN_TEST(test_functionality_wdgSetGetCfg_trigSel);              \
                            RUN_TEST(test_functionality_wdgSetGetCfg_failThr);              \
                            RUN_TEST(test_functionality_wdgSetGetCfg_rstThr);               \
                            RUN_TEST(test_functionality_wdgSetGetCfg_longWinDuration);      \
                            RUN_TEST(test_functionality_wdgSetGetCfg_win1Duration);         \
                            RUN_TEST(test_functionality_wdgSetGetCfg_win2Duration);         \
                            RUN_TEST(test_functionality_wdgSetGetCfg_qaFdbk);               \
                            RUN_TEST(test_functionality_wdgSetGetCfg_qaLfsr);               \
                            RUN_TEST(test_functionality_wdgSetGetCfg_qaSeed);               \
                            RUN_TEST(test_faultHandling_wdgGetErrStat_nullParams);          \
                            RUN_TEST(test_faultHandling_wdgClrErrStat_nullParams);          \
                            RUN_TEST(test_faultHandling_wdgGetFailCntStat_nullParams);      \
                            RUN_TEST(test_functionality_wdgSwTriggerMode_noErrors);         \
                            RUN_TEST(test_functionality_wdgQaMode_noErrors);                \
                            RUN_TEST(test_errorDetection_wdg_longWinTimeoutInt)

Pmic_CoreHandle_t pmicHandle;

void wdgTest(void *args)
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

    DebugP_log("\r\nWelcome to PMIC LLD Watchdog module testing\r\n\r\n");

    status = Pmic_init(&pmicCfg, &pmicHandle);

    if (status == PMIC_ST_SUCCESS)
    {
        // Set PWR_ON to be 1
        status = Pmic_setPwrOn(&pmicHandle, (bool)true);

        if (status == PMIC_ST_SUCCESS)
        {
            DebugP_log("Running PMIC Watchdog tests...\r\n");
            UNITY_BEGIN();
            RUN_WDG_TESTS();
            UNITY_END();
        }
        else
        {
            DebugP_log("Failed to set PWR_ON to 1\r\n");
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

void test_faultHandling_wdgGetEnableStat_nullParams(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    bool enable = (bool)false;

    // Both parameters NULL
    status = Pmic_wdgGetEnable(NULL, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    // Parameter 1 NULL
    status = Pmic_wdgGetEnable(NULL, &enable);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    // Parameter 2 NULL
    status = Pmic_wdgGetEnable(&pmicHandle, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

void test_faultHandling_wdgDisable_nullParams(void)
{
    int32_t status = Pmic_wdgDisable(NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

void test_faultHandling_wdgEnable_nullParams(void)
{
    int32_t status = Pmic_wdgEnable(NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

void test_functionality_wdgEnableDisable(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    bool wdgEnabled = (bool)false;

    // Disable WDG
    status = Pmic_wdgDisable(&pmicHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get WDG enable status, compare expected vs. actual
    status = Pmic_wdgGetEnable(&pmicHandle, &wdgEnabled);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL((bool)false, wdgEnabled);

    // Enable WDG
    status = Pmic_wdgEnable(&pmicHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get WDG enable status, compare expected vs. actual
    status = Pmic_wdgGetEnable(&pmicHandle, &wdgEnabled);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL((bool)true, wdgEnabled);
}

void test_faultHandling_wdgGetPwrHold_nullParams(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    bool pwrHold = (bool)false;

    // Both parameters NULL
    status = Pmic_wdgGetPwrHold(NULL, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    // Parameter 1 NULL
    status = Pmic_wdgGetPwrHold(NULL, &pwrHold);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    // Parameter 2 NULL
    status = Pmic_wdgGetPwrHold(&pmicHandle, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

void test_faultHandling_wdgSetPwrHold_nullParams(void)
{
    int32_t status = Pmic_wdgSetPwrHold(NULL, (bool)true);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

void test_functionality_wdgSetClrPwrHold(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    bool pwrHold = (bool)false;

    // Clear WD_PWR_HOLD
    status = Pmic_wdgSetPwrHold(&pmicHandle, (bool)false);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get WD_PWR_HOLD status, compare expected vs. actual value
    status = Pmic_wdgGetPwrHold(&pmicHandle, &pwrHold);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL((bool)false, pwrHold);

    // Set WD_PWR_HOLD
    status = Pmic_wdgSetPwrHold(&pmicHandle, (bool)true);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get WD_PWR_HOLD status, compare expected vs. actual value
    status = Pmic_wdgGetPwrHold(&pmicHandle, &pwrHold);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL((bool)true, pwrHold);
}

void test_faultHandling_wdgGetRetLongWin_nullParams(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    bool retLongWin = (bool)false;

    // Both parameters NULL
    status = Pmic_wdgGetRetLongWin(NULL, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    // Parameter 1 NULL
    status = Pmic_wdgGetRetLongWin(NULL, &retLongWin);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    // Parameter 2 NULL
    status = Pmic_wdgGetRetLongWin(&pmicHandle, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

void test_faultHandling_wdgSetRetLongWin_nullParams(void)
{
    int32_t status = Pmic_wdgSetRetLongWin(NULL, (bool)true);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

void test_functionality_wdgSetClrRetLongWin(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    bool retLongWin = (bool)false;

    // Clear WD_RETURN_LONGWIN
    status = Pmic_wdgSetRetLongWin(&pmicHandle, (bool)false);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get WD_RETURN_LONGWIN status, compare expected vs. actual
    status = Pmic_wdgGetRetLongWin(&pmicHandle, &retLongWin);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL((bool)false, retLongWin);

    // Set WD_RETURN_LONGWIN
    status = Pmic_wdgSetRetLongWin(&pmicHandle, (bool)true);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get WD_RETURN_LONGWIN status, compare expected vs. actual
    status = Pmic_wdgGetRetLongWin(&pmicHandle, &retLongWin);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL((bool)true, retLongWin);
}

void test_faultHandling_wdgGetCfg_nullParams(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_WdgCfg_t wdgCfg;

    // Both parameters NULL
    status = Pmic_wdgGetCfg(NULL, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    // Parameter 1 NULL
    status = Pmic_wdgGetCfg(NULL, &wdgCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    // Parameter 2 NULL
    status = Pmic_wdgGetCfg(&pmicHandle, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

void test_faultHandling_wdgGetCfg_noValidParams(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_WdgCfg_t wdgCfg = {.validParams = 0U};

    status = Pmic_wdgGetCfg(&pmicHandle, &wdgCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);
}

void test_faultHandling_wdgSetCfg_nullParams(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_WdgCfg_t wdgCfg;

    // Both parameters NULL
    status = Pmic_wdgSetCfg(NULL, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    // Parameter 1 NULL
    status = Pmic_wdgSetCfg(NULL, &wdgCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    // Parameter 2 NULL
    status = Pmic_wdgSetCfg(&pmicHandle, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

void test_faultHandling_wdgSetCfg_noValidParams(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_WdgCfg_t wdgCfg = {.validParams = 0U};

    status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);
}

void checkWdgConfigurable(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    bool enabled = PMIC_DISABLE;
    bool regLocked = (bool)true;

    status = Pmic_wdgGetEnable(&pmicHandle, &enabled);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    if (enabled == (bool)false)
    {
        status = Pmic_wdgEnable(&pmicHandle);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    }

    status = Pmic_wdgGetPwrHold(&pmicHandle, &enabled);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    if (enabled == (bool)false)
    {
        status = Pmic_wdgSetPwrHold(&pmicHandle, (bool)true);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    }

    status = Pmic_wdgGetRetLongWin(&pmicHandle, &enabled);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    if (enabled == (bool)false)
    {
        status = Pmic_wdgSetRetLongWin(&pmicHandle, (bool)true);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    }

    status = Pmic_getRegLock(&pmicHandle, &regLocked);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    if (regLocked == (bool)true)
    {
        status = Pmic_unlockRegs(&pmicHandle);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    }
}

void test_functionality_wdgSetGetCfg_rstEn(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_WdgCfg_t expWdgCfg = {.validParams = PMIC_WD_RST_EN_VALID};
    Pmic_WdgCfg_t actWdgCfg = {.validParams = PMIC_WD_RST_EN_VALID};

    // Check whether watchdog is configurable
    checkWdgConfigurable();

    // Set WD_RST_EN
    expWdgCfg.rstEn = PMIC_ENABLE;
    status = Pmic_wdgSetCfg(&pmicHandle, &expWdgCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get actual WD_RST_EN value, compare expected vs. actual
    status = Pmic_wdgGetCfg(&pmicHandle, &actWdgCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(expWdgCfg.rstEn, actWdgCfg.rstEn);

    // Clear WD_RST_EN
    expWdgCfg.rstEn = PMIC_DISABLE;
    status = Pmic_wdgSetCfg(&pmicHandle, &expWdgCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get actual WD_RST_EN value, compare expected vs. actual
    status = Pmic_wdgGetCfg(&pmicHandle, &actWdgCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(expWdgCfg.rstEn, actWdgCfg.rstEn);
}

void test_functionality_wdgSetGetCfg_mode(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_WdgCfg_t expWdgCfg = {.validParams = PMIC_WD_MODE_VALID};
    Pmic_WdgCfg_t actWdgCfg = {.validParams = PMIC_WD_MODE_VALID};

    // Check whether watchdog is configurable
    checkWdgConfigurable();

    // Test API error handling for when watchdog mode value is out of bounds
    expWdgCfg.mode = PMIC_WD_MODE_MAX + 1U;
    status = Pmic_wdgSetCfg(&pmicHandle, &expWdgCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

    // Set watchdog mode to be Q&A mode
    expWdgCfg.mode = PMIC_QA_MODE;
    status = Pmic_wdgSetCfg(&pmicHandle, &expWdgCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get actual watchdog mode, compare expected vs. actual
    status = Pmic_wdgGetCfg(&pmicHandle, &actWdgCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(expWdgCfg.mode, actWdgCfg.mode);

    // Set watchdog mode to be Trigger mode
    expWdgCfg.mode = PMIC_TRIGGER_MODE;
    status = Pmic_wdgSetCfg(&pmicHandle, &expWdgCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get actual watchdog mode, compare expected vs. actual
    status = Pmic_wdgGetCfg(&pmicHandle, &actWdgCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(expWdgCfg.mode, actWdgCfg.mode);
}

void test_functionality_wdgSetGetCfg_trigSel(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_WdgCfg_t expWdgCfg = {.validParams = PMIC_WD_TRIG_SEL_VALID};
    Pmic_WdgCfg_t actWdgCfg = {.validParams = PMIC_WD_TRIG_SEL_VALID};

    // Check whether watchdog is configurable
    checkWdgConfigurable();

    // Test API error handling for when trigger select value is out of bounds
    expWdgCfg.trigSel = PMIC_TRIG_SEL_MAX + 1U;
    status = Pmic_wdgSetCfg(&pmicHandle, &expWdgCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

    // Set trigger source to be HW trigger
    expWdgCfg.trigSel = PMIC_HW_TRIGGER;
    status = Pmic_wdgSetCfg(&pmicHandle, &expWdgCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get actual trigger source, compare expected vs. actual
    status = Pmic_wdgGetCfg(&pmicHandle, &actWdgCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(expWdgCfg.trigSel, actWdgCfg.trigSel);

    // Set trigger source to be SW trigger
    expWdgCfg.trigSel = PMIC_SW_TRIGGER;
    status = Pmic_wdgSetCfg(&pmicHandle, &expWdgCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get actual trigger source, compare expected vs. actual
    status = Pmic_wdgGetCfg(&pmicHandle, &actWdgCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(expWdgCfg.trigSel, actWdgCfg.trigSel);
}

void test_functionality_wdgSetGetCfg_failThr(void)
{
    uint8_t thrVal = 0U;
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_WdgCfg_t expWdgCfg = {.validParams = PMIC_WD_FAIL_THR_VALID};
    Pmic_WdgCfg_t actWdgCfg = {.validParams = PMIC_WD_FAIL_THR_VALID};

    // Check whether watchdog is configurable
    checkWdgConfigurable();

    // Test API error handling for when watchdog fail threshold value is out of bounds
    expWdgCfg.failThr = PMIC_WD_FAIL_THR_MAX + 1U;
    status = Pmic_wdgSetCfg(&pmicHandle, &expWdgCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

    // Test all the possible values of watchdog fail threshold
    for (thrVal = 0U; thrVal <= PMIC_WD_FAIL_THR_MAX; thrVal++)
    {
        // Set fail threshold value
        expWdgCfg.failThr = thrVal;
        status = Pmic_wdgSetCfg(&pmicHandle, &expWdgCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Get actual fail threshold value, compare expected vs. actual
        status = Pmic_wdgGetCfg(&pmicHandle, &actWdgCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        TEST_ASSERT_EQUAL(expWdgCfg.failThr, actWdgCfg.failThr);
    }
}

void test_functionality_wdgSetGetCfg_rstThr(void)
{
    uint8_t thrVal = 0U;
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_WdgCfg_t expWdgCfg = {.validParams = PMIC_WD_RST_THR_VALID};
    Pmic_WdgCfg_t actWdgCfg = {.validParams = PMIC_WD_RST_THR_VALID};

    // Check whether watchdog is configurable
    checkWdgConfigurable();

    // Test API error handling for when watchdog reset threshold value is out of bounds
    expWdgCfg.rstThr = PMIC_WD_RST_THR_MAX + 1U;
    status = Pmic_wdgSetCfg(&pmicHandle, &expWdgCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

    // Test all the possible values of watchdog reset threshold
    for (thrVal = 0U; thrVal <= PMIC_WD_RST_THR_MAX; thrVal++)
    {
        // Set reset threshold value
        expWdgCfg.rstThr = thrVal;
        status = Pmic_wdgSetCfg(&pmicHandle, &expWdgCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Get actual reset threshold value, compare expected vs. actual
        status = Pmic_wdgGetCfg(&pmicHandle, &actWdgCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        TEST_ASSERT_EQUAL(expWdgCfg.rstThr, actWdgCfg.rstThr);
    }
}

void test_functionality_wdgSetGetCfg_longWinDuration(void)
{
    uint16_t winDuration = 0U;
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_WdgCfg_t expWdgCfg = {.validParams = PMIC_WD_LONG_WIN_DURATION_VALID};
    Pmic_WdgCfg_t actWdgCfg = {.validParams = PMIC_WD_LONG_WIN_DURATION_VALID};

    // Check whether watchdog is configurable
    checkWdgConfigurable();

    // Test all the possible values of watchdog Long Window
    for (winDuration = 0U; winDuration <= 0xFFU; winDuration++)
    {
        // Set Long Window duration
        expWdgCfg.longWinDuration = winDuration;
        status = Pmic_wdgSetCfg(&pmicHandle, &expWdgCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Get actual Long Window duration, compare expected vs. actual
        status = Pmic_wdgGetCfg(&pmicHandle, &actWdgCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        TEST_ASSERT_EQUAL(expWdgCfg.longWinDuration, actWdgCfg.longWinDuration);
    }
}

void test_functionality_wdgSetGetCfg_win1Duration(void)
{
    uint8_t winDuration = 0U;
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_WdgCfg_t expWdgCfg = {.validParams = PMIC_WD_WIN1_DURATION_VALID};
    Pmic_WdgCfg_t actWdgCfg = {.validParams = PMIC_WD_WIN1_DURATION_VALID};

    // Check whether watchdog is configurable
    checkWdgConfigurable();

    // Test API error handling for when watchdog Window-1 value is out of bounds
    expWdgCfg.win1Duration = PMIC_WD_WIN1_DURATION_MAX + 1U;
    status = Pmic_wdgSetCfg(&pmicHandle, &expWdgCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

    // Test all the possible values of watchdog Window-1
    for (winDuration = 0U; winDuration <= PMIC_WD_WIN1_DURATION_MAX; winDuration++)
    {
        // Set Window-1 duration value
        expWdgCfg.win1Duration = winDuration;
        status = Pmic_wdgSetCfg(&pmicHandle, &expWdgCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Get actual Window-1 duration value, compare expected vs. actual
        status = Pmic_wdgGetCfg(&pmicHandle, &actWdgCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        TEST_ASSERT_EQUAL(expWdgCfg.win1Duration, actWdgCfg.win1Duration);
    }
}

void test_functionality_wdgSetGetCfg_win2Duration(void)
{
    uint8_t winDuration = 0U;
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_WdgCfg_t expWdgCfg = {.validParams = PMIC_WD_WIN2_DURATION_VALID};
    Pmic_WdgCfg_t actWdgCfg = {.validParams = PMIC_WD_WIN2_DURATION_VALID};

    // Check whether watchdog is configurable
    checkWdgConfigurable();

    // Test API error handling for when watchdog Window-2 value is out of bounds
    expWdgCfg.win2Duration = PMIC_WD_WIN2_DURATION_MAX + 1U;
    status = Pmic_wdgSetCfg(&pmicHandle, &expWdgCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

    // Test all the possible values of watchdog Window-2
    for (winDuration = 0U; winDuration <= PMIC_WD_WIN2_DURATION_MAX; winDuration++)
    {
        // Set Window-2 duration value
        expWdgCfg.win2Duration = winDuration;
        status = Pmic_wdgSetCfg(&pmicHandle, &expWdgCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Get actual Window-2 duration value, compare expected vs. actual
        status = Pmic_wdgGetCfg(&pmicHandle, &actWdgCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        TEST_ASSERT_EQUAL(expWdgCfg.win2Duration, actWdgCfg.win2Duration);
    }
}

void test_functionality_wdgSetGetCfg_qaFdbk(void)
{
    uint8_t qaFdbkVal = 0U;
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_WdgCfg_t expWdgCfg = {.validParams = PMIC_WD_QA_FDBK_VALID};
    Pmic_WdgCfg_t actWdgCfg = {.validParams = PMIC_WD_QA_FDBK_VALID};

    // Check whether watchdog is configurable
    checkWdgConfigurable();

    // Test API error handling for when watchdog Q&A feedback value is out of bounds
    expWdgCfg.qaFdbk = PMIC_WD_QA_FDBK_MAX + 1U;
    status = Pmic_wdgSetCfg(&pmicHandle, &expWdgCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

    // Test all the possible values of watchdog Q&A feedback
    for (qaFdbkVal = 0U; qaFdbkVal <= PMIC_WD_QA_FDBK_MAX; qaFdbkVal++)
    {
        // Set Q&A feedback value
        expWdgCfg.qaFdbk = qaFdbkVal;
        status = Pmic_wdgSetCfg(&pmicHandle, &expWdgCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Get actual Q&A feedback value, compare expected vs. actual
        status = Pmic_wdgGetCfg(&pmicHandle, &actWdgCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        TEST_ASSERT_EQUAL(expWdgCfg.qaFdbk, actWdgCfg.qaFdbk);
    }
}

void test_functionality_wdgSetGetCfg_qaLfsr(void)
{
    uint8_t qaLfsrVal = 0U;
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_WdgCfg_t expWdgCfg = {.validParams = PMIC_WD_QA_LFSR_VALID};
    Pmic_WdgCfg_t actWdgCfg = {.validParams = PMIC_WD_QA_LFSR_VALID};

    // Check whether watchdog is configurable
    checkWdgConfigurable();

    // Test API error handling for when watchdog Q&A LFSR value is out of bounds
    expWdgCfg.qaLfsr = PMIC_WD_QA_LFSR_MAX + 1U;
    status = Pmic_wdgSetCfg(&pmicHandle, &expWdgCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

    // Test all the possible values of watchdog Q&A LFSR
    for (qaLfsrVal = 0U; qaLfsrVal <= PMIC_WD_QA_FDBK_MAX; qaLfsrVal++)
    {
        // Set Q&A LFSR value
        expWdgCfg.qaLfsr = qaLfsrVal;
        status = Pmic_wdgSetCfg(&pmicHandle, &expWdgCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Get actual Q&A LFSR value, compare expected vs. actual
        status = Pmic_wdgGetCfg(&pmicHandle, &actWdgCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        TEST_ASSERT_EQUAL(expWdgCfg.qaLfsr, actWdgCfg.qaLfsr);
    }
}

void test_functionality_wdgSetGetCfg_qaSeed(void)
{
    uint8_t qaSeedVal = 0U;
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_WdgCfg_t expWdgCfg = {.validParams = PMIC_WD_QA_SEED_VALID};
    Pmic_WdgCfg_t actWdgCfg = {.validParams = PMIC_WD_QA_SEED_VALID};

    // Check whether watchdog is configurable
    checkWdgConfigurable();

    // Test API error handling for when watchdog Q&A question seed value is out of bounds
    expWdgCfg.qaSeed = PMIC_WD_QA_SEED_MAX + 1U;
    status = Pmic_wdgSetCfg(&pmicHandle, &expWdgCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

    // Test all the possible values of watchdog Q&A question seed
    for (qaSeedVal = 0U; qaSeedVal <= PMIC_WD_QA_SEED_MAX; qaSeedVal++)
    {
        // Set Q&A question seed value
        expWdgCfg.qaSeed = qaSeedVal;
        status = Pmic_wdgSetCfg(&pmicHandle, &expWdgCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Get actual Q&A question seed value, compare expected vs. actual
        status = Pmic_wdgGetCfg(&pmicHandle, &actWdgCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        TEST_ASSERT_EQUAL(expWdgCfg.qaSeed, actWdgCfg.qaSeed);
    }
}

void test_faultHandling_wdgGetErrStat_nullParams(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_WdgErrStat_t wdgErrStat;

    // Both parameters NULL
    status = Pmic_wdgGetErrStat(NULL, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    // Parameter 1 NULL
    status = Pmic_wdgGetErrStat(NULL, &wdgErrStat);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    // Parameter 2 NULL
    status = Pmic_wdgGetErrStat(&pmicHandle, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

void test_faultHandling_wdgClrErrStat_nullParams(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_WdgErrStat_t wdgErrStat;

    // Both parameters NULL
    status = Pmic_wdgClrErrStat(NULL, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    // Parameter 1 NULL
    status = Pmic_wdgClrErrStat(NULL, &wdgErrStat);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    // Parameter 2 NULL
    status = Pmic_wdgClrErrStat(&pmicHandle, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

void test_faultHandling_wdgGetFailCntStat_nullParams(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_WdgFailCntStat_t wdgFailCntStat;

    // Both parameters NULL
    status = Pmic_wdgGetFailCntStat(NULL, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    // Parameter 1 NULL
    status = Pmic_wdgGetFailCntStat(NULL, &wdgFailCntStat);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    // Parameter 2 NULL
    status = Pmic_wdgGetFailCntStat(&pmicHandle, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

static inline void waitWinDuration(void)
{
    // Period: 71 ms
    (void)RTI_counterEnable(PMIC_RTI1_BASE_ADDR, RTI_TMR_CNT_BLK_INDEX_0);
    (void)RTI_intStatusClear(PMIC_RTI1_BASE_ADDR, RTI_TMR_INT_REQ_ALL);
    while (RTI_intStatusGet(PMIC_RTI1_BASE_ADDR) == 0U);
    (void)RTI_counterDisable(PMIC_RTI1_BASE_ADDR, RTI_TMR_CNT_BLK_INDEX_0);
}

void checkWdgStatus(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_WdgErrStat_t wdgErrStat = {
        .validParams = (PMIC_WDG_RST_INT_VALID | PMIC_WDG_FAIL_INT_VALID |
                        PMIC_WDG_ANSW_ERR_VALID | PMIC_WDG_SEQ_ERR_VALID |
                        PMIC_WDG_ANSW_EARLY_ERR_VALID | PMIC_WDG_TIMEOUT_ERR_VALID |
                        PMIC_WDG_LONGWIN_TIMEOUT_INT_VALID)
    };
    Pmic_WdgFailCntStat_t wdgFailCntStat = {
        .validParams = (PMIC_BAD_EVENT_VALID | PMIC_GOOD_EVENT_VALID | PMIC_FAIL_CNT_VALID)
    };

    status = Pmic_wdgGetErrStat(&pmicHandle, &wdgErrStat);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL((bool)false, wdgErrStat.rstInt);
    TEST_ASSERT_EQUAL((bool)false, wdgErrStat.failInt);
    TEST_ASSERT_EQUAL((bool)false, wdgErrStat.answErr);
    TEST_ASSERT_EQUAL((bool)false, wdgErrStat.seqErr);
    TEST_ASSERT_EQUAL((bool)false, wdgErrStat.answEarlyErr);
    TEST_ASSERT_EQUAL((bool)false, wdgErrStat.timeoutErr);
    TEST_ASSERT_EQUAL((bool)false, wdgErrStat.longWinTimeoutInt);

    status = Pmic_wdgGetFailCntStat(&pmicHandle, &wdgFailCntStat);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL((bool)false, wdgFailCntStat.badEvent);
    TEST_ASSERT_EQUAL((bool)true, wdgFailCntStat.goodEvent);
    TEST_ASSERT_EQUAL(0U, wdgFailCntStat.failCnt);
}

void test_functionality_wdgSwTriggerMode_noErrors(void)
{
    uint8_t numSequences = 0U;
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_WdgCfg_t wdgCfg = {
        .validParams = (PMIC_WD_RST_EN_VALID | PMIC_WD_MODE_VALID |
                        PMIC_WD_TRIG_SEL_VALID | PMIC_WD_FAIL_THR_VALID |
                        PMIC_WD_RST_THR_VALID | PMIC_WD_LONG_WIN_DURATION_VALID |
                        PMIC_WD_WIN1_DURATION_VALID | PMIC_WD_WIN2_DURATION_VALID),
        .rstEn = (bool)false,
        .mode = PMIC_TRIGGER_MODE,
        .trigSel = PMIC_SW_TRIGGER,
        .failThr = 0U,
        .rstThr = 0U,
        .longWinDuration = PMIC_WD_LONG_WIN_DURATION_MAX,
        .win1Duration = PMIC_WD_WIN1_DURATION_MAX,
        .win2Duration = PMIC_WD_WIN2_DURATION_MAX,
    };

    // Check whether watchdog is configurable
    checkWdgConfigurable();

    // Set watchdog configuration
    status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Clear watchdog error statuses before exiting Long Window
    status = Pmic_wdgClrErrStatAll(&pmicHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Exit Long Window by clearing WD_PWR_HOLD, clearing WD_RETURN_LONGWIN,
    // and sending the SW trigger
    status = Pmic_wdgSetPwrHold(&pmicHandle, (bool)false);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    status = Pmic_wdgSetRetLongWin(&pmicHandle, (bool)false);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    status = Pmic_wdgSendSwTrigger(&pmicHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Undergo watchdog sequences
    for (numSequences = 100U; numSequences != 0U; numSequences--)
    {
        // Upon last sequence, indicate return to Long Window
        if (numSequences == 1U)
        {
            status = Pmic_wdgSetRetLongWin(&pmicHandle, (bool)true);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        }

        // Enter Window-1; wait duration of the window to get into Window-2
        waitWinDuration();

        // Enter Window-2; send SW trigger
        status = Pmic_wdgSendSwTrigger(&pmicHandle);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // check watchdog error statuses and fail count statuses at end of sequence
        checkWdgStatus();

        // Begin next sequence
    }

    // WDG has returned to Long Window; set WD_PWRHOLD so that WDG remains in Long Window
    status = Pmic_wdgSetPwrHold(&pmicHandle, (bool)true);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
}

static inline void checkForWdgErrors(const char *str)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    // Read WD_ERR_STATUS register and check whether register value is zero
    status = Pmic_ioRx(&pmicHandle, 0x5FU, &regData);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(0U, regData, str);
}

void test_functionality_wdgQaMode_noErrors(void)
{
    uint8_t answerCnt = 0U;
    uint8_t numSequences = 0U;
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_WdgCfg_t wdgCfg = {
        .validParams = (PMIC_WD_RST_EN_VALID | PMIC_WD_MODE_VALID | PMIC_WD_FAIL_THR_VALID |
                        PMIC_WD_RST_THR_VALID | PMIC_WD_LONG_WIN_DURATION_VALID |
                        PMIC_WD_WIN1_DURATION_VALID | PMIC_WD_WIN2_DURATION_VALID |
                        PMIC_WD_QA_FDBK_VALID | PMIC_WD_QA_LFSR_VALID | PMIC_WD_QA_SEED_VALID),
        .rstEn = (bool)false,
        .mode = PMIC_QA_MODE,
        .failThr = 0U,
        .rstThr = 0U,
        .longWinDuration = PMIC_WD_LONG_WIN_DURATION_MAX,
        .win1Duration = PMIC_WD_WIN1_DURATION_MAX,
        .win2Duration = PMIC_WD_WIN2_DURATION_MAX,
        .qaFdbk = 1U,
        .qaLfsr = 2U,
        .qaSeed = 3U
    };

    // Check whether watchdog is configurable
    checkWdgConfigurable();

    // Set watchdog configuration
    status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Clear watchdog error statuses before exiting Long Window
    status = Pmic_wdgClrErrStatAll(&pmicHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Enable watchdog to exit and stay out of Long Window
    status = Pmic_wdgSetPwrHold(&pmicHandle, (bool)false);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    status = Pmic_wdgSetRetLongWin(&pmicHandle, (bool)false);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Exit Long Window by sending all four answer bytes
    for (answerCnt = 4U; answerCnt != 0U; answerCnt--)
    {
        status = Pmic_wdgWriteAnswer(&pmicHandle);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    }

    // Check for any errors that could have occurred during Long Window exit
    checkForWdgErrors("Exit LW");

    // Undergo Q&A watchdog sequences
    for (numSequences = 100U; numSequences != 0U; numSequences--)
    {
        // Upon last sequence, indicate return to Long Window
        if (numSequences == 1U)
        {
            status = Pmic_wdgSetRetLongWin(&pmicHandle, (bool)true);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        }

        // Enter Window-1; calculate and send Answer-3,
        // Answer-2, and Answer-1; check for any WDG errors
        for (answerCnt = 3U; answerCnt >= 1U; answerCnt--)
        {
            status = Pmic_wdgWriteAnswer(&pmicHandle);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
            checkForWdgErrors("Win-1");
        }

        // Wait until Window-1 time elapses
        waitWinDuration();

        // Enter Window-2; calculate and send last answer byte; check for any WDG errors
        status = Pmic_wdgWriteAnswer(&pmicHandle);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        checkForWdgErrors("Win-2");

        // End of Q&A sequence; next question will be generated
        // and the next sequence will begin
    }

    // WDG has returned to Long Window; set WD_PWRHOLD so that WDG remains in Long Window
    status = Pmic_wdgSetPwrHold(&pmicHandle, (bool)true);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
}

void test_errorDetection_wdg_longWinTimeoutInt(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_WdgCfg_t wdgCfg = {
        .validParams = (PMIC_WD_RST_EN_VALID | PMIC_WD_MODE_VALID |
                        PMIC_WD_TRIG_SEL_VALID | PMIC_WD_FAIL_THR_VALID |
                        PMIC_WD_RST_THR_VALID | PMIC_WD_LONG_WIN_DURATION_VALID |
                        PMIC_WD_WIN1_DURATION_VALID | PMIC_WD_WIN2_DURATION_VALID),
        .rstEn = (bool)false,
        .mode = PMIC_TRIGGER_MODE,
        .trigSel = PMIC_SW_TRIGGER,
        .failThr = 7U,
        .rstThr = 7U,
        .longWinDuration = 0x00U,
        .win1Duration = PMIC_WD_WIN1_DURATION_MAX,
        .win2Duration = PMIC_WD_WIN2_DURATION_MAX,
    };
    Pmic_WdgErrStat_t wdgErrStat;

    // Check whether watchdog is configurable
    checkWdgConfigurable();

    // Set watchdog configuration
    status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Clear watchdog error statuses
    status = Pmic_wdgClrErrStatAll(&pmicHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Enable watchdog to exit and stay out of Long Window
    status = Pmic_wdgSetPwrHold(&pmicHandle, (bool)false);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    status = Pmic_wdgSetRetLongWin(&pmicHandle, (bool)false);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Wait for the watchdog to time out in Long Window
    (void)RTI_counterEnable(PMIC_RTI2_BASE_ADDR, RTI_TMR_CNT_BLK_INDEX_0);
    (void)RTI_intStatusClear(PMIC_RTI2_BASE_ADDR, RTI_TMR_INT_REQ_ALL);
    while (RTI_intStatusGet(PMIC_RTI2_BASE_ADDR) == 0U);
    (void)RTI_counterDisable(PMIC_RTI2_BASE_ADDR, RTI_TMR_CNT_BLK_INDEX_0);

    // Set WD_RETURN_LONGWIN to return to Long Window and WD_PWRHOLD to stay in Long Window
    status = Pmic_wdgSetRetLongWin(&pmicHandle, (bool)true);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    status = Pmic_wdgSetPwrHold(&pmicHandle, (bool)true);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // check whether WD_LONGWIN_TIMEOUT_INT error is set
    wdgErrStat.validParams = PMIC_WDG_LONGWIN_TIMEOUT_INT_VALID;
    status = Pmic_wdgGetErrStat(&pmicHandle, &wdgErrStat);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL((bool)true, wdgErrStat.longWinTimeoutInt);

    // Clear the error
    status = Pmic_wdgClrErrStat(&pmicHandle, &wdgErrStat);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Check whether error has been cleared
    status = Pmic_wdgGetErrStat(&pmicHandle, &wdgErrStat);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL((bool)false, wdgErrStat.longWinTimeoutInt);
}

void test_errorDetection_wdg_timeout(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_WdgCfg_t wdgCfg = {
        .validParams = (PMIC_WD_RST_EN_VALID | PMIC_WD_MODE_VALID |
                        PMIC_WD_TRIG_SEL_VALID | PMIC_WD_FAIL_THR_VALID |
                        PMIC_WD_RST_THR_VALID | PMIC_WD_LONG_WIN_DURATION_VALID |
                        PMIC_WD_WIN1_DURATION_VALID | PMIC_WD_WIN2_DURATION_VALID),
        .rstEn = (bool)false,
        .mode = PMIC_TRIGGER_MODE,
        .trigSel = PMIC_SW_TRIGGER,
        .failThr = 7U,
        .rstThr = 7U,
        .longWinDuration = PMIC_WD_LONG_WIN_DURATION_MAX,
        .win1Duration = PMIC_WD_WIN1_DURATION_MAX,
        .win2Duration = PMIC_WD_WIN2_DURATION_MAX,
    };
    Pmic_WdgErrStat_t wdgErrStat;

    // Check whether watchdog is configurable
    checkWdgConfigurable();

    // Set watchdog configuration
    status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Clear watchdog error statuses
    status = Pmic_wdgClrErrStatAll(&pmicHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Exit Long Window by clearing WD_PWR_HOLD, clearing WD_RETURN_LONGWIN,
    // and sending the SW trigger
    status = Pmic_wdgSetPwrHold(&pmicHandle, (bool)false);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    status = Pmic_wdgSetRetLongWin(&pmicHandle, (bool)false);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    status = Pmic_wdgSendSwTrigger(&pmicHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Enter Window-1; wait duration of the window to get into Window-2
    waitWinDuration();

    // Enter Window-2; wait duration of Window-2 to trigger WD_TIMEOUT
    waitWinDuration();

    // Set WD_RETURN_LONGWIN to return to Long Window and WD_PWRHOLD to stay in Long Window
    status = Pmic_wdgSetRetLongWin(&pmicHandle, (bool)true);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    status = Pmic_wdgSetPwrHold(&pmicHandle, (bool)true);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // check whether WD_TIMEOUT error is set
    wdgErrStat.validParams = PMIC_WDG_TIMEOUT_ERR_VALID;
    status = Pmic_wdgGetErrStat(&pmicHandle, &wdgErrStat);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL((bool)true, wdgErrStat.timeoutErr);

    // Clear the error
    status = Pmic_wdgClrErrStat(&pmicHandle, &wdgErrStat);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Check whether error has been cleared
    status = Pmic_wdgGetErrStat(&pmicHandle, &wdgErrStat);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL((bool)false, wdgErrStat.timeoutErr);
}

void test_errorDetection_wdg_failInt(void)
{

}

void test_errorDetection_wdg_RstInt(void)
{

}

/**
 *  @brief  Unity testing framework calls this API before each test
 */
void setUp(void)
{
}

/**
 *  @brief  Unity testing framework calls this API after each test
 */
void tearDown(void)
{
}
