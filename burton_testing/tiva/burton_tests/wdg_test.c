#include <stdio.h>

#include "pmic.h"

#include "wdg_test.h"

#include "unity.h"

// clang-format off
#define RUN_WDG_TESTS   RUN_TEST(test_wdg_enableDisable);                   \
                        RUN_TEST(test_wdg_setCfg_longWindowDuration);       \
                        RUN_TEST(test_wdg_setCfg_window1Duration);          \
                        RUN_TEST(test_wdg_setCfg_window2Duration);          \
                        RUN_TEST(test_wdg_setCfg_failThreshold);            \
                        RUN_TEST(test_wdg_setCfg_resetThreshold);           \
                        RUN_TEST(test_wdg_setCfg_resetEnable);              \
                        RUN_TEST(test_wdg_setCfg_wdgMode);                  \
                        RUN_TEST(test_wdg_setCfg_powerHold);                \
                        RUN_TEST(test_wdg_setCfg_ReturnLongWindow);         \
                        RUN_TEST(test_wdg_setCfg_QA_feedback);              \
                        RUN_TEST(test_wdg_setCfg_QA_LFSR);                  \
                        RUN_TEST(test_wdg_setCfg_QA_questionSeed);          \
                        RUN_TEST(test_wdg_setCfg_cntSel);                   \
                        RUN_TEST(test_wdg_setCfg_enDrvSel);                 \
                        RUN_TEST(test_wdg_QaMode_noErrors);                 \
                        RUN_TEST(test_wdg_QaMode_detect_longWindowTimeout); \
                        RUN_TEST(test_wdg_QaMode_detect_windowTimeout);     \
                        RUN_TEST(test_wdg_QaMode_detect_answerEarly);       \
                        RUN_TEST(test_wdg_QaMode_detect_sequenceError);     \
                        RUN_TEST(test_wdg_QaMode_detect_answerError);       \
                        RUN_TEST(test_wdg_QaMode_detect_failError);         \
                        RUN_TEST(test_wdg_QaMode_detect_resetError)

// clang-format on

int32_t configurePmicI2CPins(Pmic_CoreHandle_t pmicCoreHandle);

uartHandle_t      vcpHandle;
timerHandle_t     timerHandle;
Pmic_CoreHandle_t pmicCoreHandle;

int main(void)
{
    // clang-format off
    // uartHandle_t vcpHandle;
    i2cHandle_t I2C1Handle, I2C2Handle;
    const Pmic_CoreCfg_t pmicCoreCfg = {
        .validParams = (PMIC_CFG_DEVICE_TYPE_VALID_SHIFT    | PMIC_CFG_COMM_MODE_VALID_SHIFT      |
                        PMIC_CFG_SLAVEADDR_VALID_SHIFT      | PMIC_CFG_QASLAVEADDR_VALID_SHIFT    |
                        PMIC_CFG_COMM_HANDLE_VALID_SHIFT    | PMIC_CFG_QACOMM_HANDLE_VALID_SHIFT  |
                        PMIC_CFG_COMM_IO_RD_VALID_SHIFT     | PMIC_CFG_COMM_IO_WR_VALID_SHIFT     |
                        PMIC_CFG_CRITSEC_START_VALID_SHIFT  | PMIC_CFG_CRITSEC_STOP_VALID_SHIFT   |
                        PMIC_CFG_I2C1_SPEED_VALID_SHIFT     | PMIC_CFG_I2C2_SPEED_VALID_SHIFT),
        .instType = (PMIC_MAIN_INST | PMIC_QA_INST),
        .pmicDeviceType = PMIC_DEV_BURTON_TPS6522X,
        .commMode = PMIC_INTF_DUAL_I2C,
        .slaveAddr = BURTON_I2C_USER_PAGE_ADDRESS,
        .qaSlaveAddr = BURTON_I2C_WDG_PAGE_ADDRESS,
        .pCommHandle = &I2C1Handle,
        .pQACommHandle = &I2C2Handle,
        .pFnPmicCommIoRead = &pmicI2CRead,
        .pFnPmicCommIoWrite = &pmicI2CWrite,
        .pFnPmicCritSecStart = &pmicCritSecStart,
        .pFnPmicCritSecStop = &pmicCritSecStop,
        .i2c1Speed = PMIC_I2C_STANDARD_MODE,
        .i2c2Speed = PMIC_I2C_STANDARD_MODE
    };
    int32_t status = PMIC_ST_SUCCESS;
    // clang-format on

    // Initialize system clock
    SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                   SYSCTL_XTAL_16MHZ); // 400 / 2 / 4 = 50 MHz clock rate

    // Initialize timer
    initializeTimerHandle(&timerHandle);
    initializeTimer(&timerHandle);

    // Enable printing to console
    initializeVCPHandle(&vcpHandle);
    initializeVCP(&vcpHandle);

    // Initialize I2C1 and I2C2 for communicating with PMIC
    initializeI2C1Handle(&I2C1Handle);
    initializeI2C2Handle(&I2C2Handle);
    initializeI2C(&I2C1Handle);
    initializeI2C(&I2C2Handle);

    // Initialize PMIC handle
    initializePmicCoreHandle(&pmicCoreHandle);
    status = Pmic_init(&pmicCoreCfg, &pmicCoreHandle);

    // Clear console of any previous messages
    clearConsole(&vcpHandle);

    if (status == PMIC_ST_SUCCESS)
    {
        // Configure PMIC GPIO1 and GPIO2 to I2C functionality
        status = configurePmicI2CPins(pmicCoreHandle);

        if (status == PMIC_ST_SUCCESS)
        {
            // Disable all power resources and clear all interrupts
            disablePmicPowerResources(pmicCoreHandle);
            status = Pmic_irqClrErrStatus(&pmicCoreHandle, PMIC_IRQ_ALL);

            // Conduct unit testing
            if (status == PMIC_ST_SUCCESS)
            {
                UARTStrPut(&vcpHandle, "Running all PMIC Watchdog tests...\r\n\r\n");

                UNITY_BEGIN();
                RUN_WDG_TESTS;
                UNITY_END();
            }
            // Error message if IRQs were not able to be cleared
            else
            {
                UARTStrPut(&vcpHandle, "Error in clearing all PMIC IRQs. Error code: ");
                UARTInt32Put(&vcpHandle, status);
                UARTStrPut(&vcpHandle, "\r\n\r\n");
            }
        }
        // Error message if GPIO pins were not able to be configured
        else
        {
            UARTStrPut(&vcpHandle, "Error in configuring PMIC GPIO pins to I2C functionality. Error code: ");
            UARTInt32Put(&vcpHandle, status);
            UARTStrPut(&vcpHandle, "\r\n\r\n");
        }
    }
    // Error message if PMIC handle not initialized correctly
    else
    {
        UARTStrPut(&vcpHandle, "Error in initializing PMIC handle. Error code: ");
        UARTInt32Put(&vcpHandle, status);
        UARTStrPut(&vcpHandle, "\r\n\r\n");
    }

    return status;
}

int32_t configurePmicI2CPins(Pmic_CoreHandle_t pmicCoreHandle)
{
    int32_t              status = PMIC_ST_SUCCESS;
    const Pmic_GpioCfg_t gpio1Cfg = {.pinFunc = PMIC_TPS6522X_GPIO_PINFUNC_GPIO1_SDA_I2C2_SDO_SPI};
    const Pmic_GpioCfg_t gpio2Cfg = {.pinFunc = PMIC_TPS6522X_GPIO_PINFUNC_GPIO2_SCL_I2C2_CS_SPI};

    status = Pmic_gpioSetConfiguration(&pmicCoreHandle, PMIC_TPS6522X_GPIO1_PIN, gpio1Cfg);

    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_gpioSetConfiguration(&pmicCoreHandle, PMIC_TPS6522X_GPIO2_PIN, gpio2Cfg);
    }

    return status;
}

/**
 *  \brief  Pmic_wdgEnable/Pmic_wdgDisable: Test whether API can enable/disable Burton Watchdog
 */
void test_wdg_enableDisable(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    bool    wdgEnabled = false;

    // Enable Watchdog
    status = Pmic_wdgEnable(&pmicCoreHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get actual Watchdog enable/disable status and compare expected vs. actual value
    status = Pmic_wdgGetEnableState(&pmicCoreHandle, &wdgEnabled);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(true, wdgEnabled);

    // Disable Watchdog
    status = Pmic_wdgDisable(&pmicCoreHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get actual Watchdog enable/disable status and compare expected vs. actual value
    status = Pmic_wdgGetEnableState(&pmicCoreHandle, &wdgEnabled);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(false, wdgEnabled);
}

/**
 *  \brief  In order to configure the PMIC WDG, the WDG needs to be enabled.
 *          This helper function is used to check whether WDG is enabled. If
 *          disabled, the function will re-enable WDG.
 */
static void checkWdgEnabled(void)
{
    bool    wdgEnabled = false;
    int32_t status = Pmic_wdgGetEnableState(&pmicCoreHandle, &wdgEnabled);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    if (!wdgEnabled)
    {
        status = Pmic_wdgEnable(&pmicCoreHandle);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    }
}

void test_wdg_setCfg_longWindowDuration(void)
{
    int32_t       status = PMIC_ST_SUCCESS;
    Pmic_WdgCfg_t wdgCfg_expected, wdgCfg_actual;

    checkWdgEnabled();

    wdgCfg_expected.validParams = PMIC_CFG_WDG_LONGWINDURATION_VALID_SHIFT;
    wdgCfg_actual.validParams = PMIC_CFG_WDG_LONGWINDURATION_VALID_SHIFT;

    // Set Watchdog Long Window duration to be maximum duration
    wdgCfg_expected.longWinDuration_ms = 772000;
    status = Pmic_wdgSetCfg(&pmicCoreHandle, wdgCfg_expected);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get actual Long Window duration and compare expected vs. actual value
    status = Pmic_wdgGetCfg(&pmicCoreHandle, &wdgCfg_actual);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(wdgCfg_expected.longWinDuration_ms, wdgCfg_actual.longWinDuration_ms);
}

void test_wdg_setCfg_window1Duration(void)
{
    int32_t       status = PMIC_ST_SUCCESS;
    Pmic_WdgCfg_t wdgCfg_expected, wdgCfg_actual;

    checkWdgEnabled();

    wdgCfg_expected.validParams = PMIC_CFG_WDG_WIN1DURATION_VALID_SHIFT;
    wdgCfg_actual.validParams = PMIC_CFG_WDG_WIN1DURATION_VALID_SHIFT;

    // Set Watchdog Window-1 duration to be maximum duration
    wdgCfg_expected.win1Duration_us = 70400;
    status = Pmic_wdgSetCfg(&pmicCoreHandle, wdgCfg_expected);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get actual Window-1 duration and compare expected vs. actual value
    status = Pmic_wdgGetCfg(&pmicCoreHandle, &wdgCfg_actual);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(wdgCfg_expected.win1Duration_us, wdgCfg_actual.win1Duration_us);
}

void test_wdg_setCfg_window2Duration(void)
{
    int32_t       status = PMIC_ST_SUCCESS;
    Pmic_WdgCfg_t wdgCfg_expected, wdgCfg_actual;

    checkWdgEnabled();

    wdgCfg_expected.validParams = PMIC_CFG_WDG_WIN2DURATION_VALID_SHIFT;
    wdgCfg_actual.validParams = PMIC_CFG_WDG_WIN2DURATION_VALID_SHIFT;

    // Set Watchdog Window-2 duration to be maximum duration
    wdgCfg_expected.win2Duration_us = 70400;
    status = Pmic_wdgSetCfg(&pmicCoreHandle, wdgCfg_expected);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get actual Window-2 duration and compare expected vs. actual value
    status = Pmic_wdgGetCfg(&pmicCoreHandle, &wdgCfg_actual);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(wdgCfg_expected.win2Duration_us, wdgCfg_actual.win2Duration_us);
}

void test_wdg_setCfg_failThreshold(void)
{
    int32_t       status = PMIC_ST_SUCCESS;
    Pmic_WdgCfg_t wdgCfg_expected, wdgCfg_actual;

    checkWdgEnabled();

    wdgCfg_expected.validParams = PMIC_CFG_WDG_FAILTHRESHOLD_VALID_SHIFT;
    wdgCfg_actual.validParams = PMIC_CFG_WDG_FAILTHRESHOLD_VALID_SHIFT;

    // Set fail threshold to maximum value
    wdgCfg_expected.failThreshold = 7;
    status = Pmic_wdgSetCfg(&pmicCoreHandle, wdgCfg_expected);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get actual fail threshold and compare expected vs. actual value
    status = Pmic_wdgGetCfg(&pmicCoreHandle, &wdgCfg_actual);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(wdgCfg_expected.failThreshold, wdgCfg_actual.failThreshold);
}

void test_wdg_setCfg_resetThreshold(void)
{
    int32_t       status = PMIC_ST_SUCCESS;
    Pmic_WdgCfg_t wdgCfg_expected, wdgCfg_actual;

    checkWdgEnabled();

    wdgCfg_expected.validParams = PMIC_CFG_WDG_RSTTHRESHOLD_VALID_SHIFT;
    wdgCfg_actual.validParams = PMIC_CFG_WDG_RSTTHRESHOLD_VALID_SHIFT;

    // Set fail threshold to maximum value
    wdgCfg_expected.rstThreshold = 7;
    status = Pmic_wdgSetCfg(&pmicCoreHandle, wdgCfg_expected);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get actual fail threshold and compare expected vs. actual value
    status = Pmic_wdgGetCfg(&pmicCoreHandle, &wdgCfg_actual);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(wdgCfg_expected.rstThreshold, wdgCfg_actual.rstThreshold);
}

void test_wdg_setCfg_resetEnable(void)
{
    int32_t       status = PMIC_ST_SUCCESS;
    Pmic_WdgCfg_t wdgCfg_expected, wdgCfg_actual;

    checkWdgEnabled();

    wdgCfg_expected.validParams = PMIC_CFG_WDG_RSTENABLE_VALID_SHIFT;
    wdgCfg_actual.validParams = PMIC_CFG_WDG_RSTENABLE_VALID_SHIFT;

    // Enable Watchdog Reset
    wdgCfg_expected.rstEnable = PMIC_WDG_RESET_ENABLE;
    status = Pmic_wdgSetCfg(&pmicCoreHandle, wdgCfg_expected);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get actual Watchdog Reset Enable status and compare expected vs. actual value
    status = Pmic_wdgGetCfg(&pmicCoreHandle, &wdgCfg_actual);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(wdgCfg_expected.rstEnable, wdgCfg_actual.rstEnable);

    // Disable Watchdog Reset
    wdgCfg_expected.rstEnable = PMIC_WDG_RESET_DISABLE;
    status = Pmic_wdgSetCfg(&pmicCoreHandle, wdgCfg_expected);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get actual Watchdog Reset Enable status and compare expected vs. actual value
    status = Pmic_wdgGetCfg(&pmicCoreHandle, &wdgCfg_actual);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(wdgCfg_expected.rstEnable, wdgCfg_actual.rstEnable);
}

void test_wdg_setCfg_wdgMode(void)
{
    int32_t       status = PMIC_ST_SUCCESS;
    Pmic_WdgCfg_t wdgCfg_expected, wdgCfg_actual;

    checkWdgEnabled();

    wdgCfg_expected.validParams = PMIC_CFG_WDG_WDGMODE_VALID_SHIFT;
    wdgCfg_actual.validParams = PMIC_CFG_WDG_WDGMODE_VALID_SHIFT;

    // Set Watchdog mode to be Q&A mode
    wdgCfg_expected.wdgMode = PMIC_WDG_QA_MODE;
    status = Pmic_wdgSetCfg(&pmicCoreHandle, wdgCfg_expected);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get actual Watchdog mode and compare expected vs. actual value
    status = Pmic_wdgGetCfg(&pmicCoreHandle, &wdgCfg_actual);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(wdgCfg_expected.wdgMode, wdgCfg_actual.wdgMode);

    // Set Watchdog mode to be trigger mode
    wdgCfg_expected.wdgMode = PMIC_WDG_TRIGGER_MODE;
    status = Pmic_wdgSetCfg(&pmicCoreHandle, wdgCfg_expected);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get actual Watchdog mode status and compare expected vs. actual value
    status = Pmic_wdgGetCfg(&pmicCoreHandle, &wdgCfg_actual);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(wdgCfg_expected.wdgMode, wdgCfg_actual.wdgMode);
}

void test_wdg_setCfg_powerHold(void)
{
    int32_t       status = PMIC_ST_SUCCESS;
    Pmic_WdgCfg_t wdgCfg_expected, wdgCfg_actual;

    checkWdgEnabled();

    wdgCfg_expected.validParams = PMIC_CFG_WDG_PWRHOLD_VALID_SHIFT;
    wdgCfg_actual.validParams = PMIC_CFG_WDG_PWRHOLD_VALID_SHIFT;

    // Enable Watchdog PWRHOLD
    wdgCfg_expected.pwrHold = PMIC_WDG_PWRHOLD_ENABLE;
    status = Pmic_wdgSetCfg(&pmicCoreHandle, wdgCfg_expected);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get actual Watchdog PWRHOLD status and compare expected vs. actual value
    status = Pmic_wdgGetCfg(&pmicCoreHandle, &wdgCfg_actual);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(wdgCfg_expected.pwrHold, wdgCfg_actual.pwrHold);

    // Disable Watchdog PWRHOLD
    wdgCfg_expected.wdgMode = PMIC_WDG_PWRHOLD_DISABLE;
    status = Pmic_wdgSetCfg(&pmicCoreHandle, wdgCfg_expected);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get actual Watchdog PWRHOLD status and compare expected vs. actual value
    status = Pmic_wdgGetCfg(&pmicCoreHandle, &wdgCfg_actual);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(wdgCfg_expected.pwrHold, wdgCfg_actual.pwrHold);
}

void test_wdg_setCfg_ReturnLongWindow(void)
{
    int32_t       status = PMIC_ST_SUCCESS;
    Pmic_WdgCfg_t wdgCfg_expected, wdgCfg_actual;

    checkWdgEnabled();

    wdgCfg_expected.validParams = PMIC_CFG_WDG_RETLONGWIN_VALID_SHIFT;
    wdgCfg_actual.validParams = PMIC_CFG_WDG_RETLONGWIN_VALID_SHIFT;

    // Enable Watchdog return to Long Window
    wdgCfg_expected.retLongWin = PMIC_WDG_RETLONGWIN_ENABLE;
    status = Pmic_wdgSetCfg(&pmicCoreHandle, wdgCfg_expected);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get actual Watchdog return to Long Window status and compare expected vs. actual value
    status = Pmic_wdgGetCfg(&pmicCoreHandle, &wdgCfg_actual);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(wdgCfg_expected.retLongWin, wdgCfg_actual.retLongWin);

    // Disable Watchdog return to Long Window
    wdgCfg_expected.retLongWin = PMIC_WDG_RETLONGWIN_DISABLE;
    status = Pmic_wdgSetCfg(&pmicCoreHandle, wdgCfg_expected);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get actual Watchdog return to Long Window status and compare expected vs. actual value
    status = Pmic_wdgGetCfg(&pmicCoreHandle, &wdgCfg_actual);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(wdgCfg_expected.retLongWin, wdgCfg_actual.retLongWin);
}

void test_wdg_setCfg_QA_feedback(void)
{
    uint8_t i = 0;
    int32_t status = PMIC_ST_SUCCESS;

    Pmic_WdgCfg_t wdgCfg_expected, wdgCfg_actual;

    checkWdgEnabled();

    wdgCfg_expected.validParams = PMIC_CFG_WDG_QA_FDBK_VALID_SHIFT;
    wdgCfg_actual.validParams = PMIC_CFG_WDG_QA_FDBK_VALID_SHIFT;

    // For each possible feedback value...
    for (i = PMIC_WDG_QA_FEEDBACK_VALUE_3; i != PMIC_WDG_QA_FEEDBACK_VALUE_0; i--)
    {
        // Set the feedback configuration
        wdgCfg_expected.qaFdbk = i;
        status = Pmic_wdgSetCfg(&pmicCoreHandle, wdgCfg_expected);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Get actual feedback value and compare expected vs. actual
        status = Pmic_wdgGetCfg(&pmicCoreHandle, &wdgCfg_actual);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        TEST_ASSERT_EQUAL(wdgCfg_expected.qaFdbk, wdgCfg_actual.qaFdbk);
    }
}

void test_wdg_setCfg_QA_LFSR(void)
{
    uint8_t i = 0;
    int32_t status = PMIC_ST_SUCCESS;

    Pmic_WdgCfg_t wdgCfg_expected, wdgCfg_actual;

    checkWdgEnabled();

    wdgCfg_expected.validParams = PMIC_CFG_WDG_QA_LFSR_VALID_SHIFT;
    wdgCfg_actual.validParams = PMIC_CFG_WDG_QA_LFSR_VALID_SHIFT;

    // For each possible LFSR value...
    for (i = PMIC_WDG_QA_LFSR_VALUE_3; i != PMIC_WDG_QA_LFSR_VALUE_0; i--)
    {
        // Set the LFSR configuration
        wdgCfg_expected.qaLfsr = i;
        status = Pmic_wdgSetCfg(&pmicCoreHandle, wdgCfg_expected);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Get actual LFSR value and compare expected vs. actual
        status = Pmic_wdgGetCfg(&pmicCoreHandle, &wdgCfg_actual);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        TEST_ASSERT_EQUAL(wdgCfg_expected.qaLfsr, wdgCfg_actual.qaLfsr);
    }
}

void test_wdg_setCfg_QA_questionSeed(void)
{
    uint8_t i = 0;
    int32_t status = PMIC_ST_SUCCESS;

    Pmic_WdgCfg_t wdgCfg_expected, wdgCfg_actual;

    checkWdgEnabled();

    wdgCfg_expected.validParams = PMIC_CFG_WDG_QA_QUES_SEED_VALID_SHIFT;
    wdgCfg_actual.validParams = PMIC_CFG_WDG_QA_QUES_SEED_VALID_SHIFT;

    // For each possible question seed value...
    for (i = PMIC_WDG_QA_QUES_SEED_VALUE_15; i != PMIC_WDG_QA_QUES_SEED_VALUE_0; i--)
    {
        // Set the question seed
        wdgCfg_expected.qaQuesSeed = i;
        status = Pmic_wdgSetCfg(&pmicCoreHandle, wdgCfg_expected);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Get actual question seed value and compare expected vs. actual
        status = Pmic_wdgGetCfg(&pmicCoreHandle, &wdgCfg_actual);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        TEST_ASSERT_EQUAL(wdgCfg_expected.qaQuesSeed, wdgCfg_actual.qaQuesSeed);
    }
}

void test_wdg_setCfg_cntSel(void)
{
    int32_t       status = PMIC_ST_SUCCESS;
    Pmic_WdgCfg_t wdgCfg_expected = {.validParams = PMIC_CFG_WDG_CNT_SEL_VALID_SHIFT};
    Pmic_WdgCfg_t wdgCfg_actual = {.validParams = PMIC_CFG_WDG_CNT_SEL_VALID_SHIFT};

    // Set WD_CNT_SEL bit to 1
    wdgCfg_expected.cntSel = PMIC_WDG_CNT_SEL_2_1_SCHEME;
    status = Pmic_wdgSetCfg(&pmicCoreHandle, wdgCfg_expected);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get actual WD_CNT_SEL bit value and compare expected vs. actual
    status = Pmic_wdgGetCfg(&pmicCoreHandle, &wdgCfg_actual);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(wdgCfg_expected.cntSel, wdgCfg_actual.cntSel);

    // Set WD_CNT_SEL bit to 0
    wdgCfg_expected.cntSel = PMIC_WDG_CNT_SEL_1_1_SCHEME;
    status = Pmic_wdgSetCfg(&pmicCoreHandle, wdgCfg_expected);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get actual WD_CNT_SEL bit value and compare expected vs. actual
    status = Pmic_wdgGetCfg(&pmicCoreHandle, &wdgCfg_actual);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(wdgCfg_expected.cntSel, wdgCfg_actual.cntSel);
}

void test_wdg_setCfg_enDrvSel(void)
{
    int32_t       status = PMIC_ST_SUCCESS;
    Pmic_WdgCfg_t wdgCfg_expected = {.validParams = PMIC_CFG_WDG_ENDRV_SEL_VALID_SHIFT};
    Pmic_WdgCfg_t wdgCfg_actual = {.validParams = PMIC_CFG_WDG_ENDRV_SEL_VALID_SHIFT};

    // Set WD_ENDRV_SEL bit to 1
    wdgCfg_expected.enDrvSel = PMIC_WDG_ENDRV_SEL_CLR;
    status = Pmic_wdgSetCfg(&pmicCoreHandle, wdgCfg_expected);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get actual WD_ENDRV_SEL bit value and compare expected vs. actual
    status = Pmic_wdgGetCfg(&pmicCoreHandle, &wdgCfg_actual);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(wdgCfg_expected.enDrvSel, wdgCfg_actual.enDrvSel);

    // Set WD_ENDRV_SEL bit to 0
    wdgCfg_expected.cntSel = PMIC_WDG_ENDRV_SEL_NO_CLR;
    status = Pmic_wdgSetCfg(&pmicCoreHandle, wdgCfg_expected);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get actual WD_ENDRV_SEL bit value and compare expected vs. actual
    status = Pmic_wdgGetCfg(&pmicCoreHandle, &wdgCfg_actual);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(wdgCfg_expected.enDrvSel, wdgCfg_actual.enDrvSel);
}

static inline void checkForWdgErrors(const char *str)
{
    uint8_t wdgErrStatusRegData = 0;

    (void)pmicI2CRead(&pmicCoreHandle, PMIC_QA_INST, 0x408, &wdgErrStatusRegData, 1);
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(0, wdgErrStatusRegData, str);
}

void test_wdg_QaMode_noErrors(void)
{
    uint8_t       answerCnt = 0;
    uint16_t      numSeqeunces = 0;
    int32_t       status = PMIC_ST_SUCCESS;
    Pmic_WdgCfg_t wdgCfg = {.validParams =
                                (PMIC_CFG_WDG_LONGWINDURATION_VALID_SHIFT | PMIC_CFG_WDG_WIN1DURATION_VALID_SHIFT |
                                 PMIC_CFG_WDG_WIN2DURATION_VALID_SHIFT | PMIC_CFG_WDG_FAILTHRESHOLD_VALID_SHIFT |
                                 PMIC_CFG_WDG_RSTTHRESHOLD_VALID_SHIFT | PMIC_CFG_WDG_PWRHOLD_VALID_SHIFT |
                                 PMIC_CFG_WDG_RETLONGWIN_VALID_SHIFT | PMIC_CFG_WDG_QA_FDBK_VALID_SHIFT |
                                 PMIC_CFG_WDG_QA_LFSR_VALID_SHIFT | PMIC_CFG_WDG_QA_QUES_SEED_VALID_SHIFT),
                            .longWinDuration_ms = 5000,
                            .win1Duration_us = 70400,
                            .win2Duration_us = 70400,
                            .failThreshold = 7,
                            .rstThreshold = 7,
                            .pwrHold = true,
                            .retLongWin = true,
                            .qaFdbk = 0,
                            .qaLfsr = 0,
                            .qaQuesSeed = 0};

    checkWdgEnabled();

    // Configure Watchdog
    status = Pmic_wdgSetCfg(&pmicCoreHandle, wdgCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Wait a small period of time so that changes propagate
    delayTimeInMs(&timerHandle, 100);

    // Begin Q&A sequences
    status = Pmic_wdgBeginSequences(&pmicCoreHandle, PMIC_WDG_QA_MODE);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Exit Long Window by sending all 4 answer bytes
    for (answerCnt = 4; answerCnt != 0; answerCnt--)
    {
        status = Pmic_wdgQaSequenceWriteAnswer(&pmicCoreHandle);
        checkForWdgErrors("Exit LW");
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    }

    // Undergo Q&A sequences
    for (numSeqeunces = 20; numSeqeunces != 0; numSeqeunces--)
    {
        // Upon last iteration, indicate that we want to return to Long Window
        if (numSeqeunces == 1)
        {
            wdgCfg.validParams = PMIC_CFG_WDG_RETLONGWIN_VALID_SHIFT;
            wdgCfg.retLongWin = true;
            status = Pmic_wdgSetCfg(&pmicCoreHandle, wdgCfg);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        }

        // Enter Window-1; for Answer-3, Answer-2, and Answer-1,
        // calculate and send answer byte; check for any WDG errors
        for (answerCnt = 3; answerCnt >= 1; answerCnt--)
        {
            status = Pmic_wdgQaSequenceWriteAnswer(&pmicCoreHandle);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
            checkForWdgErrors("Win-1");
        }

        // Wait until Window-1 time elapses
        delayTimeInMs(&timerHandle, 71);

        // Enter Window-2; calculate and send last answer byte; check for any WDG errors
        status = Pmic_wdgQaSequenceWriteAnswer(&pmicCoreHandle);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        checkForWdgErrors("Win-2");

        // End of Q&A sequence; next question will be generated
        // and the next sequence will begin
    }

    // WDG has returned to Long Window; set WD_PWRHOLD so that WDG remains in Long Window
    wdgCfg.validParams = PMIC_CFG_WDG_PWRHOLD_VALID_SHIFT;
    wdgCfg.pwrHold = true;
    status = Pmic_wdgSetCfg(&pmicCoreHandle, wdgCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    status = Pmic_wdgGetCfg(&pmicCoreHandle, &wdgCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(true, wdgCfg.pwrHold);
}

void test_wdg_QaMode_detect_longWindowTimeout(void)
{
    int32_t             status = PMIC_ST_SUCCESS;
    Pmic_WdgCfg_t       wdgCfg = {.validParams =
                                      (PMIC_CFG_WDG_LONGWINDURATION_VALID_SHIFT | PMIC_CFG_WDG_WIN1DURATION_VALID_SHIFT |
                                 PMIC_CFG_WDG_WIN2DURATION_VALID_SHIFT | PMIC_CFG_WDG_FAILTHRESHOLD_VALID_SHIFT |
                                 PMIC_CFG_WDG_RSTTHRESHOLD_VALID_SHIFT | PMIC_CFG_WDG_PWRHOLD_VALID_SHIFT |
                                 PMIC_CFG_WDG_RETLONGWIN_VALID_SHIFT | PMIC_CFG_WDG_QA_FDBK_VALID_SHIFT |
                                 PMIC_CFG_WDG_QA_LFSR_VALID_SHIFT | PMIC_CFG_WDG_QA_QUES_SEED_VALID_SHIFT),
                                  .longWinDuration_ms = 500,
                                  .win1Duration_us = 70400,
                                  .win2Duration_us = 70400,
                                  .failThreshold = 7,
                                  .rstThreshold = 7,
                                  .pwrHold = true,
                                  .retLongWin = true,
                                  .qaFdbk = 0,
                                  .qaLfsr = 0,
                                  .qaQuesSeed = 0};
    Pmic_WdgErrStatus_t wdgErrStat = {.validParams = PMIC_CFG_WD_LONGWIN_TIMEOUT_ERRSTAT_VALID_SHIFT};

    checkWdgEnabled();

    // Configure Watchdog
    status = Pmic_wdgSetCfg(&pmicCoreHandle, wdgCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Wait a small period of time so that changes propagate
    delayTimeInMs(&timerHandle, 100);

    // Begin Q&A sequence
    status = Pmic_wdgBeginSequences(&pmicCoreHandle, PMIC_WDG_QA_MODE);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Watchdog is currently in Long Window mode awaiting the first four
    // answer bytes; wait the duration of the Long Window period without
    // sending anything to cause Long Window timeout error
    delayTimeInMs(&timerHandle, 600);

    // Enable return to Long Window and set WD_PWRHOLD to make WDG stay in Long Window
    wdgCfg.validParams = (PMIC_CFG_WDG_PWRHOLD_VALID_SHIFT | PMIC_CFG_WDG_RETLONGWIN_VALID_SHIFT);
    wdgCfg.pwrHold = true;
    wdgCfg.retLongWin = true;
    status = Pmic_wdgSetCfg(&pmicCoreHandle, wdgCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Check whether WD_LONGWIN_TIMEOUT_INT error is raised
    status = Pmic_wdgGetErrorStatus(&pmicCoreHandle, &wdgErrStat);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(true, wdgErrStat.wdLongWinTimeout);

    // Clear WD_LONGWIN_TIMEOUT_INT error
    status = Pmic_wdgClrErrStatus(&pmicCoreHandle, PMIC_WDG_ERR_LONG_WIN_TIMEOUT);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Check whether WD_LONGWIN_TIMEOUT_INT error is cleared
    status = Pmic_wdgGetErrorStatus(&pmicCoreHandle, &wdgErrStat);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(false, wdgErrStat.wdLongWinTimeout);
}

void test_wdg_QaMode_detect_windowTimeout(void)
{
    uint8_t             answerCnt = 0;
    int32_t             status = PMIC_ST_SUCCESS;
    Pmic_WdgCfg_t       wdgCfg = {.validParams =
                                      (PMIC_CFG_WDG_LONGWINDURATION_VALID_SHIFT | PMIC_CFG_WDG_WIN1DURATION_VALID_SHIFT |
                                 PMIC_CFG_WDG_WIN2DURATION_VALID_SHIFT | PMIC_CFG_WDG_FAILTHRESHOLD_VALID_SHIFT |
                                 PMIC_CFG_WDG_RSTTHRESHOLD_VALID_SHIFT | PMIC_CFG_WDG_PWRHOLD_VALID_SHIFT |
                                 PMIC_CFG_WDG_RETLONGWIN_VALID_SHIFT | PMIC_CFG_WDG_QA_FDBK_VALID_SHIFT |
                                 PMIC_CFG_WDG_QA_LFSR_VALID_SHIFT | PMIC_CFG_WDG_QA_QUES_SEED_VALID_SHIFT),
                                  .longWinDuration_ms = 5000,
                                  .win1Duration_us = 70400,
                                  .win2Duration_us = 70400,
                                  .failThreshold = 7,
                                  .rstThreshold = 7,
                                  .pwrHold = true,
                                  .retLongWin = true,
                                  .qaFdbk = 0,
                                  .qaLfsr = 0,
                                  .qaQuesSeed = 0};
    Pmic_WdgErrStatus_t wdgErrStat = {.validParams = PMIC_CFG_WD_TIMEOUT_ERRSTAT_VALID_SHIFT};

    checkWdgEnabled();

    // Configure Watchdog
    status = Pmic_wdgSetCfg(&pmicCoreHandle, wdgCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Wait a small period of time so that changes propagate
    delayTimeInMs(&timerHandle, 100);

    // Begin Q&A sequences
    status = Pmic_wdgBeginSequences(&pmicCoreHandle, PMIC_WDG_QA_MODE);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Exit Long Window by sending all 4 answer bytes
    for (answerCnt = 4; answerCnt != 0; answerCnt--)
    {
        status = Pmic_wdgQaSequenceWriteAnswer(&pmicCoreHandle);
        checkForWdgErrors("Exit LW");
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    }

    // Return to Long Window after this sequence
    wdgCfg.validParams = PMIC_CFG_WDG_RETLONGWIN_VALID_SHIFT;
    wdgCfg.retLongWin = true;
    status = Pmic_wdgSetCfg(&pmicCoreHandle, wdgCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Watchdog has entered Window-1; wait duration of Window-1 to cause WD_TIMEOUT error
    delayTimeInMs(&timerHandle, 71);

    // Wait duration of Window-2 to end the sequence
    delayTimeInMs(&timerHandle, 71);

    // WDG has returned to Long Window; set WD_PWRHOLD so that WDG remains in Long Window
    wdgCfg.validParams = PMIC_CFG_WDG_PWRHOLD_VALID_SHIFT;
    wdgCfg.pwrHold = true;
    status = Pmic_wdgSetCfg(&pmicCoreHandle, wdgCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    status = Pmic_wdgGetCfg(&pmicCoreHandle, &wdgCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(true, wdgCfg.pwrHold);

    // Check whether WD_TIMEOUT error is raised
    status = Pmic_wdgGetErrorStatus(&pmicCoreHandle, &wdgErrStat);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(true, wdgErrStat.wdTimeout);

    // Clear WD_TIMEOUT error
    status = Pmic_wdgClrErrStatus(&pmicCoreHandle, PMIC_WDG_ERR_TIMEOUT);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Check whether WD_TIMEOUT error is cleared
    status = Pmic_wdgGetErrorStatus(&pmicCoreHandle, &wdgErrStat);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(false, wdgErrStat.wdTimeout);
}

void test_wdg_QaMode_detect_answerEarly(void)
{
    uint8_t             answerCnt = 0;
    int32_t             status = PMIC_ST_SUCCESS;
    Pmic_WdgCfg_t       wdgCfg = {.validParams =
                                      (PMIC_CFG_WDG_LONGWINDURATION_VALID_SHIFT | PMIC_CFG_WDG_WIN1DURATION_VALID_SHIFT |
                                 PMIC_CFG_WDG_WIN2DURATION_VALID_SHIFT | PMIC_CFG_WDG_FAILTHRESHOLD_VALID_SHIFT |
                                 PMIC_CFG_WDG_RSTTHRESHOLD_VALID_SHIFT | PMIC_CFG_WDG_PWRHOLD_VALID_SHIFT |
                                 PMIC_CFG_WDG_RETLONGWIN_VALID_SHIFT | PMIC_CFG_WDG_QA_FDBK_VALID_SHIFT |
                                 PMIC_CFG_WDG_QA_LFSR_VALID_SHIFT | PMIC_CFG_WDG_QA_QUES_SEED_VALID_SHIFT),
                                  .longWinDuration_ms = 5000,
                                  .win1Duration_us = 70400,
                                  .win2Duration_us = 70400,
                                  .failThreshold = 7,
                                  .rstThreshold = 7,
                                  .pwrHold = true,
                                  .retLongWin = true,
                                  .qaFdbk = 0,
                                  .qaLfsr = 0,
                                  .qaQuesSeed = 0};
    Pmic_WdgErrStatus_t wdgErrStat = {.validParams = PMIC_CFG_WD_ANSW_EARLY_ERRSTAT_VALID_SHIFT};

    checkWdgEnabled();

    // Configure Watchdog
    status = Pmic_wdgSetCfg(&pmicCoreHandle, wdgCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Wait a small period of time so that changes propagate
    delayTimeInMs(&timerHandle, 100);

    // Begin Q&A sequences
    status = Pmic_wdgBeginSequences(&pmicCoreHandle, PMIC_WDG_QA_MODE);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Exit Long Window by sending all 4 answer bytes
    for (answerCnt = 4; answerCnt != 0; answerCnt--)
    {
        status = Pmic_wdgQaSequenceWriteAnswer(&pmicCoreHandle);
        checkForWdgErrors("Exit LW");
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    }

    // Return to Long Window after this sequence
    wdgCfg.validParams = PMIC_CFG_WDG_RETLONGWIN_VALID_SHIFT;
    wdgCfg.retLongWin = true;
    status = Pmic_wdgSetCfg(&pmicCoreHandle, wdgCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Watchdog has entered Window-1; send all answer bytes in this window to cause WD_ANSW_EARLY error
    for (answerCnt = 4; answerCnt != 0; answerCnt--)
    {
        status = Pmic_wdgQaSequenceWriteAnswer(&pmicCoreHandle);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    }

    // Wait until Window-1 elapses
    delayTimeInMs(&timerHandle, 71);

    // Watchdog has entered Window-2; wait until Window-2 elapses to end sequence
    delayTimeInMs(&timerHandle, 71);

    // WDG has returned to Long Window; set WD_PWRHOLD so that WDG remains in Long Window
    wdgCfg.validParams = PMIC_CFG_WDG_PWRHOLD_VALID_SHIFT;
    wdgCfg.pwrHold = true;
    status = Pmic_wdgSetCfg(&pmicCoreHandle, wdgCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    status = Pmic_wdgGetCfg(&pmicCoreHandle, &wdgCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(true, wdgCfg.pwrHold);

    // Check whether WD_ANSW_EARLY error is raised
    status = Pmic_wdgGetErrorStatus(&pmicCoreHandle, &wdgErrStat);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(true, wdgErrStat.wdAnswearly);

    // Clear WD_ANSW_EARLY error
    status = Pmic_wdgClrErrStatus(&pmicCoreHandle, PMIC_WDG_ERR_ANSWER_EARLY);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Check whether WD_ANSW_EARLY error is cleared
    status = Pmic_wdgGetErrorStatus(&pmicCoreHandle, &wdgErrStat);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(false, wdgErrStat.wdAnswearly);
}

void test_wdg_QaMode_detect_sequenceError(void)
{
    uint8_t             answerCnt = 0;
    int32_t             status = PMIC_ST_SUCCESS;
    Pmic_WdgCfg_t       wdgCfg = {.validParams =
                                      (PMIC_CFG_WDG_LONGWINDURATION_VALID_SHIFT | PMIC_CFG_WDG_WIN1DURATION_VALID_SHIFT |
                                 PMIC_CFG_WDG_WIN2DURATION_VALID_SHIFT | PMIC_CFG_WDG_FAILTHRESHOLD_VALID_SHIFT |
                                 PMIC_CFG_WDG_RSTTHRESHOLD_VALID_SHIFT | PMIC_CFG_WDG_PWRHOLD_VALID_SHIFT |
                                 PMIC_CFG_WDG_RETLONGWIN_VALID_SHIFT | PMIC_CFG_WDG_QA_FDBK_VALID_SHIFT |
                                 PMIC_CFG_WDG_QA_LFSR_VALID_SHIFT | PMIC_CFG_WDG_QA_QUES_SEED_VALID_SHIFT),
                                  .longWinDuration_ms = 5000,
                                  .win1Duration_us = 70400,
                                  .win2Duration_us = 70400,
                                  .failThreshold = 7,
                                  .rstThreshold = 7,
                                  .pwrHold = true,
                                  .retLongWin = true,
                                  .qaFdbk = 0,
                                  .qaLfsr = 0,
                                  .qaQuesSeed = 0};
    Pmic_WdgErrStatus_t wdgErrStat = {.validParams = PMIC_CFG_WD_SEQ_ERR_ERRSTAT_VALID_SHIFT};

    checkWdgEnabled();

    // Configure Watchdog
    status = Pmic_wdgSetCfg(&pmicCoreHandle, wdgCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Wait a small period of time so that changes propagate
    delayTimeInMs(&timerHandle, 100);

    // Begin Q&A sequences
    status = Pmic_wdgBeginSequences(&pmicCoreHandle, PMIC_WDG_QA_MODE);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Exit Long Window by sending all 4 answer bytes
    for (answerCnt = 4; answerCnt != 0; answerCnt--)
    {
        status = Pmic_wdgQaSequenceWriteAnswer(&pmicCoreHandle);
        checkForWdgErrors("Exit LW");
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    }

    // Return to Long Window after this sequence
    wdgCfg.validParams = PMIC_CFG_WDG_RETLONGWIN_VALID_SHIFT;
    wdgCfg.retLongWin = true;
    status = Pmic_wdgSetCfg(&pmicCoreHandle, wdgCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Watchdog has entered Window-1; wait until Window-1
    // elapses in order to send all answer bytes in Window-2
    delayTimeInMs(&timerHandle, 71);

    // Watchdog has entered Window-2; send all answer bytes in this window to cause WD_SEQ_ERR error
    for (answerCnt = 4; answerCnt != 0; answerCnt--)
    {
        status = Pmic_wdgQaSequenceWriteAnswer(&pmicCoreHandle);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    }

    // WDG has returned to Long Window; set WD_PWRHOLD so that WDG remains in Long Window
    wdgCfg.validParams = PMIC_CFG_WDG_PWRHOLD_VALID_SHIFT;
    wdgCfg.pwrHold = true;
    status = Pmic_wdgSetCfg(&pmicCoreHandle, wdgCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    status = Pmic_wdgGetCfg(&pmicCoreHandle, &wdgCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(true, wdgCfg.pwrHold);

    // Check whether WD_SEQ_ERR error is raised
    status = Pmic_wdgGetErrorStatus(&pmicCoreHandle, &wdgErrStat);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(true, wdgErrStat.wdSeqErr);

    // Clear WD_SEQ_ERR error
    status = Pmic_wdgClrErrStatus(&pmicCoreHandle, PMIC_WDG_ERR_SEQ_ERR);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Check whether WD_SEQ_ERR error is cleared
    status = Pmic_wdgGetErrorStatus(&pmicCoreHandle, &wdgErrStat);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(false, wdgErrStat.wdSeqErr);
}

void test_wdg_QaMode_detect_answerError(void)
{
    uint8_t             regData = 0;
    uint8_t             answerCnt = 0;
    const uint8_t       wdgAnswRegAddr = 0x1;
    int32_t             status = PMIC_ST_SUCCESS;
    Pmic_WdgCfg_t       wdgCfg = {.validParams =
                                      (PMIC_CFG_WDG_LONGWINDURATION_VALID_SHIFT | PMIC_CFG_WDG_WIN1DURATION_VALID_SHIFT |
                                 PMIC_CFG_WDG_WIN2DURATION_VALID_SHIFT | PMIC_CFG_WDG_FAILTHRESHOLD_VALID_SHIFT |
                                 PMIC_CFG_WDG_RSTTHRESHOLD_VALID_SHIFT | PMIC_CFG_WDG_PWRHOLD_VALID_SHIFT |
                                 PMIC_CFG_WDG_RETLONGWIN_VALID_SHIFT | PMIC_CFG_WDG_QA_FDBK_VALID_SHIFT |
                                 PMIC_CFG_WDG_QA_LFSR_VALID_SHIFT | PMIC_CFG_WDG_QA_QUES_SEED_VALID_SHIFT),
                                  .longWinDuration_ms = 5000,
                                  .win1Duration_us = 70400,
                                  .win2Duration_us = 70400,
                                  .failThreshold = 7,
                                  .rstThreshold = 7,
                                  .pwrHold = true,
                                  .retLongWin = true,
                                  .qaFdbk = 0,
                                  .qaLfsr = 0,
                                  .qaQuesSeed = 0};
    Pmic_WdgErrStatus_t wdgErrStat = {.validParams = PMIC_CFG_WD_ANSW_ERR_ERRSTAT_VALID_SHIFT};

    checkWdgEnabled();

    // Configure Watchdog
    status = Pmic_wdgSetCfg(&pmicCoreHandle, wdgCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Wait a small period of time so that changes propagate
    delayTimeInMs(&timerHandle, 100);

    // Begin Q&A sequences
    status = Pmic_wdgBeginSequences(&pmicCoreHandle, PMIC_WDG_QA_MODE);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Exit Long Window by sending all 4 answer bytes
    for (answerCnt = 4; answerCnt != 0; answerCnt--)
    {
        status = Pmic_wdgQaSequenceWriteAnswer(&pmicCoreHandle);
        checkForWdgErrors("Exit LW");
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    }

    // Return to Long Window after this sequence
    wdgCfg.validParams = PMIC_CFG_WDG_RETLONGWIN_VALID_SHIFT;
    wdgCfg.retLongWin = true;
    status = Pmic_wdgSetCfg(&pmicCoreHandle, wdgCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Watchdog has entered Window-1; send incorrect Window-1 answer bytes
    for (answerCnt = 3; answerCnt >= 1; answerCnt--)
    {
        status = pmicI2CWrite(&pmicCoreHandle, PMIC_QA_INST, wdgAnswRegAddr, &regData, 1);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        regData++;
    }

    // Wait until Window-1 elapses
    delayTimeInMs(&timerHandle, 71);

    // Watchdog has entered Window-2; send incorrect Window-2 answer byte
    status = pmicI2CWrite(&pmicCoreHandle, PMIC_QA_INST, wdgAnswRegAddr, &regData, 1);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // WDG has returned to Long Window; set WD_PWRHOLD so that WDG remains in Long Window
    wdgCfg.validParams = PMIC_CFG_WDG_PWRHOLD_VALID_SHIFT;
    wdgCfg.pwrHold = true;
    status = Pmic_wdgSetCfg(&pmicCoreHandle, wdgCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    status = Pmic_wdgGetCfg(&pmicCoreHandle, &wdgCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(true, wdgCfg.pwrHold);

    // Check whether WD_ANSW_ERR error is raised
    status = Pmic_wdgGetErrorStatus(&pmicCoreHandle, &wdgErrStat);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(true, wdgErrStat.wdAnswErr);

    // Clear WD_ANSW_ERR error
    status = Pmic_wdgClrErrStatus(&pmicCoreHandle, PMIC_WDG_ERR_ANS_ERR);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Check whether WD_ANSW_ERR error is cleared
    status = Pmic_wdgGetErrorStatus(&pmicCoreHandle, &wdgErrStat);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(false, wdgErrStat.wdAnswErr);
}

void test_wdg_QaMode_detect_failError(void)
{
    uint8_t               failCnt = 0;
    uint8_t               answerCnt = 0;
    uint16_t              numSeqeunces = 0;
    int32_t               status = PMIC_ST_SUCCESS;
    Pmic_WdgCfg_t         wdgCfg = {.validParams =
                                        (PMIC_CFG_WDG_LONGWINDURATION_VALID_SHIFT | PMIC_CFG_WDG_WIN1DURATION_VALID_SHIFT |
                                 PMIC_CFG_WDG_WIN2DURATION_VALID_SHIFT | PMIC_CFG_WDG_FAILTHRESHOLD_VALID_SHIFT |
                                 PMIC_CFG_WDG_RSTTHRESHOLD_VALID_SHIFT | PMIC_CFG_WDG_PWRHOLD_VALID_SHIFT |
                                 PMIC_CFG_WDG_RETLONGWIN_VALID_SHIFT | PMIC_CFG_WDG_QA_FDBK_VALID_SHIFT |
                                 PMIC_CFG_WDG_QA_LFSR_VALID_SHIFT | PMIC_CFG_WDG_QA_QUES_SEED_VALID_SHIFT),
                                    .longWinDuration_ms = 5000,
                                    .win1Duration_us = 70400,
                                    .win2Duration_us = 70400,
                                    .failThreshold = 7,
                                    .rstThreshold = 7,
                                    .pwrHold = true,
                                    .retLongWin = true,
                                    .qaFdbk = 0,
                                    .qaLfsr = 0,
                                    .qaQuesSeed = 0};
    Pmic_WdgErrStatus_t   wdgErrStat = {.validParams = PMIC_CFG_WD_FAIL_INT_ERRSTAT_VALID_SHIFT};
    Pmic_WdgFailCntStat_t failCntStat;

    checkWdgEnabled();

    // Configure Watchdog
    status = Pmic_wdgSetCfg(&pmicCoreHandle, wdgCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Wait a small period of time so that changes propagate
    delayTimeInMs(&timerHandle, 100);

    // Begin Q&A sequences
    status = Pmic_wdgBeginSequences(&pmicCoreHandle, PMIC_WDG_QA_MODE);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Exit Long Window by sending all 4 answer bytes
    for (answerCnt = 4; answerCnt != 0; answerCnt--)
    {
        status = Pmic_wdgQaSequenceWriteAnswer(&pmicCoreHandle);
        checkForWdgErrors("Exit LW");
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    }

    // Undergo Q&A sequences, incurring a WD_TIMEOUT error in each sequence
    for (numSeqeunces = (wdgCfg.failThreshold + 2); numSeqeunces != 0; numSeqeunces--)
    {
        // Upon last iteration, indicate that we want to return to Long Window
        if (numSeqeunces == 1)
        {
            wdgCfg.validParams = PMIC_CFG_WDG_RETLONGWIN_VALID_SHIFT;
            wdgCfg.retLongWin = true;
            status = Pmic_wdgSetCfg(&pmicCoreHandle, wdgCfg);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        }

        // Enter Window-1; wait until Window-1 elapses
        delayTimeInMs(&timerHandle, 71);

        // There should be a WD_TIMEOUT error; check whether there is a bad event
        failCntStat.validParams = PMIC_CFG_WD_BAD_EVENT_STAT_VALID_SHIFT;
        status = Pmic_wdgGetFailCntStat(&pmicCoreHandle, &failCntStat);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        TEST_ASSERT_EQUAL(true, failCntStat.wdBadEvent);

        // Enter Window-2; wait until Window-2 elapses
        delayTimeInMs(&timerHandle, 71);

        // Fail counter should increment at the end of every sequence as a result of WDG
        // not receiving any answers; Compare expected vs. actual fail count for every
        // sequence except for the last sequence (skip last sequence because at the end
        // of the sequence, the fail counter is reset to zero since WDG enters Long Window,
        // causing the assertion to fail)
        if (numSeqeunces != 1)
        {
            failCntStat.validParams = PMIC_CFG_WD_FAIL_CNT_VAL_VALID_SHIFT;
            status = Pmic_wdgGetFailCntStat(&pmicCoreHandle, &failCntStat);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
            TEST_ASSERT_EQUAL(++failCnt, failCntStat.wdFailCnt);
        }
    }

    // WDG has returned to Long Window; set WD_PWRHOLD so that WDG remains in Long Window
    wdgCfg.validParams = PMIC_CFG_WDG_PWRHOLD_VALID_SHIFT;
    wdgCfg.pwrHold = true;
    status = Pmic_wdgSetCfg(&pmicCoreHandle, wdgCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    status = Pmic_wdgGetCfg(&pmicCoreHandle, &wdgCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(true, wdgCfg.pwrHold);

    // Check whether WD_FAIL_INT error is raised
    status = Pmic_wdgGetErrorStatus(&pmicCoreHandle, &wdgErrStat);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(true, wdgErrStat.wdFailInt);

    // Clear WD_FAIL_INT error
    status = Pmic_wdgClrErrStatus(&pmicCoreHandle, PMIC_WDG_ERR_FAIL_INT);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Check whether WD_FAIL_INT error is cleared
    status = Pmic_wdgGetErrorStatus(&pmicCoreHandle, &wdgErrStat);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(false, wdgErrStat.wdFailInt);
}

void test_wdg_QaMode_detect_resetError(void)
{
    uint8_t             answerCnt = 0;
    uint16_t            numSeqeunces = 0;
    int32_t             status = PMIC_ST_SUCCESS;
    Pmic_WdgCfg_t       wdgCfg = {.validParams =
                                      (PMIC_CFG_WDG_LONGWINDURATION_VALID_SHIFT | PMIC_CFG_WDG_WIN1DURATION_VALID_SHIFT |
                                 PMIC_CFG_WDG_WIN2DURATION_VALID_SHIFT | PMIC_CFG_WDG_FAILTHRESHOLD_VALID_SHIFT |
                                 PMIC_CFG_WDG_RSTTHRESHOLD_VALID_SHIFT | PMIC_CFG_WDG_RSTENABLE_VALID_SHIFT |
                                 PMIC_CFG_WDG_PWRHOLD_VALID_SHIFT | PMIC_CFG_WDG_RETLONGWIN_VALID_SHIFT |
                                 PMIC_CFG_WDG_QA_FDBK_VALID_SHIFT | PMIC_CFG_WDG_QA_LFSR_VALID_SHIFT |
                                 PMIC_CFG_WDG_QA_QUES_SEED_VALID_SHIFT),
                                  .longWinDuration_ms = 5000,
                                  .win1Duration_us = 70400,
                                  .win2Duration_us = 70400,
                                  .failThreshold = 1,
                                  .rstThreshold = 7,
                                  .rstEnable = true, // WD_RST_INT will only raise if WD_RST_EN is set
                                  .pwrHold = true,
                                  .retLongWin = true,
                                  .qaFdbk = 0,
                                  .qaLfsr = 0,
                                  .qaQuesSeed = 0};
    Pmic_WdgErrStatus_t wdgErrStat = {.validParams = PMIC_CFG_WD_RST_INT_ERRSTAT_VALID_SHIFT};

    checkWdgEnabled();

    // Configure Watchdog
    status = Pmic_wdgSetCfg(&pmicCoreHandle, wdgCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Wait a small period of time so that changes propagate
    delayTimeInMs(&timerHandle, 100);

    // Begin Q&A sequences
    status = Pmic_wdgBeginSequences(&pmicCoreHandle, PMIC_WDG_QA_MODE);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Exit Long Window by sending all 4 answer bytes
    for (answerCnt = 4; answerCnt != 0; answerCnt--)
    {
        status = Pmic_wdgQaSequenceWriteAnswer(&pmicCoreHandle);
        checkForWdgErrors("Exit LW");
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    }

    // Undergo Q&A sequences, incurring a WD_TIMEOUT error in each sequence
    for (numSeqeunces = (wdgCfg.failThreshold + wdgCfg.rstThreshold + 2); numSeqeunces != 0; numSeqeunces--)
    {
        // Upon last iteration, indicate that we want to return to Long Window
        if (numSeqeunces == 1)
        {
            wdgCfg.validParams = PMIC_CFG_WDG_RETLONGWIN_VALID_SHIFT;
            wdgCfg.retLongWin = true;
            status = Pmic_wdgSetCfg(&pmicCoreHandle, wdgCfg);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        }

        // Enter Window-1; wait until Window-1 elapses
        delayTimeInMs(&timerHandle, 71);

        // Enter Window-2; wait until Window-2 elapses
        delayTimeInMs(&timerHandle, 71);
    }

    // WDG has returned to Long Window; set WD_PWRHOLD so that WDG remains in Long Window
    wdgCfg.validParams = PMIC_CFG_WDG_PWRHOLD_VALID_SHIFT;
    wdgCfg.pwrHold = true;
    status = Pmic_wdgSetCfg(&pmicCoreHandle, wdgCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    status = Pmic_wdgGetCfg(&pmicCoreHandle, &wdgCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(true, wdgCfg.pwrHold);

    // Check whether WD_RST_INT error is raised
    status = Pmic_wdgGetErrorStatus(&pmicCoreHandle, &wdgErrStat);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(true, wdgErrStat.wdRstInt);

    // Clear WD_FAIL_INT and WD_RST_INT error
    status = Pmic_wdgClrErrStatus(&pmicCoreHandle, PMIC_WDG_ERR_FAIL_INT);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    status = Pmic_wdgClrErrStatus(&pmicCoreHandle, PMIC_WDG_ERR_RST_INT);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Check whether WD_RST_INT error is cleared
    status = Pmic_wdgGetErrorStatus(&pmicCoreHandle, &wdgErrStat);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(false, wdgErrStat.wdRstInt);
}

/**
 *  \brief  This function is called by Unity when it starts a test
 */
void setUp(void)
{
}

/**
 *  \brief  This function is called by Unity when it finishes a test
 */
void tearDown(void)
{
    (void)Pmic_irqClrErrStatus(&pmicCoreHandle, PMIC_IRQ_ALL);
}
