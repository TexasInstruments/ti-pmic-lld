/**
 * \file wdg_test.c
 * \author John Bui (j-bui1@ti.com)
 * \brief Unity testing application that tests the PMIC driver Watchdog APIs
 * \version 1.0
 * \date 2023-12-12
 *
 * \copyright Copyright (c) 2023
 */

/* Standard includes */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

/* Tiva test app library */
#include "tiva_testLib.h"

/* Test specific include */
#include "wdg_test.h"

/* Unity testing library */
#include "unity.h"

/* PMIC driver */
#include "pmic.h"

timerHandle_t     timerHandle;
Pmic_CoreHandle_t pmicCoreHandle;

static void disablePmicPowerResources(void);

int main(void)
{
    /*** Variable declaration/initialization ***/
    // clang-format off
    uartHandle_t vcpHandle;
    i2cHandle_t I2C1Handle;
    Pmic_CoreCfg_t pmicConfigData = {
        .validParams =
            (PMIC_CFG_DEVICE_TYPE_VALID_SHIFT | PMIC_CFG_COMM_MODE_VALID_SHIFT    | PMIC_CFG_SLAVEADDR_VALID_SHIFT |
            PMIC_CFG_QASLAVEADDR_VALID_SHIFT  | PMIC_CFG_NVMSLAVEADDR_VALID_SHIFT | PMIC_CFG_COMM_HANDLE_VALID_SHIFT
            | PMIC_CFG_COMM_IO_RD_VALID_SHIFT   | PMIC_CFG_COMM_IO_WR_VALID_SHIFT   |
            PMIC_CFG_I2C1_SPEED_VALID_SHIFT),
        .instType           = PMIC_MAIN_INST,
        .pmicDeviceType     = PMIC_DEV_BURTON_TPS6522X,
        .commMode           = PMIC_INTF_SINGLE_I2C,
        .slaveAddr          = BURTON_I2C_USER_PAGE_ADDRESS,
        .qaSlaveAddr        = BURTON_I2C_WDG_PAGE_ADDRESS,
        .nvmSlaveAddr       = BURTON_I2C_NVM_PAGE_ADDRESS,
        .i2c1Speed          = PMIC_I2C_STANDARD_MODE,
        .pCommHandle        = &I2C1Handle,
        .pQACommHandle      = &I2C1Handle,
        .pFnPmicCommIoRead  = &pmicI2CRead,
        .pFnPmicCommIoWrite = &pmicI2CWrite};
    // clang-format on

    /*** System clock setup ***/
    SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                   SYSCTL_XTAL_16MHZ); // 400 / 2 / 4 = 50 MHz clock rate

    /*** Virtual communication port setup ***/
    initializeVCPHandle(&vcpHandle);
    initializeVCP(&vcpHandle);

    /*** I2C setup ***/
    initializeI2C1Handle(&I2C1Handle);
    initializeI2C(&I2C1Handle);

    /*** PMIC setup ***/
    initializePmicCoreHandle(&pmicCoreHandle);
    Pmic_init(&pmicConfigData, &pmicCoreHandle);

    /*** Timer setup ***/
    initializeTimerHandle(&timerHandle);
    initializeTimer(&timerHandle);

    /*** Clear the console before printing anything ***/
    clearConsole(&vcpHandle);

    /*** Put PMIC power resources into a known state for testing ***/
    disablePmicPowerResources();

    /*** Ensure changes are propagated by waiting a certain period of time ***/
    delayTimeInMs(&timerHandle, 1000);
    (void)Pmic_irqClrErrStatus(&pmicCoreHandle, PMIC_IRQ_ALL);

    /*** Print welcome message ***/
    UARTStrPut(&vcpHandle, "Running all PMIC Watchdog tests...\r\n\r\n");

    /*** Begin unity testing ***/
    UNITY_BEGIN();

    RUN_TEST(test_wdg_enableDisable);
    RUN_TEST(test_wdg_setCfg_longWindowDuration);

    /*** Finish unity testing ***/
    return UNITY_END();
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

    // Get actual watchdog enable state and compare expected vs. actual value
    status = Pmic_wdgGetEnableState(&pmicCoreHandle, &wdgEnabled);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(true, wdgEnabled);

    // Disable watchdog
    status = Pmic_wdgDisable(&pmicCoreHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get actual watchdog enable state and compare expected vs. actual values
    status = Pmic_wdgGetEnableState(&pmicCoreHandle, &wdgEnabled);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(false, wdgEnabled);
}

/**
 *  \brief  Pmic_wdgSetCfg: Test whether API can configure Watchdog Long Window Duration
 */
void test_wdg_setCfg_longWindowDuration(void)
{
    int32_t       status = 0;
    Pmic_WdgCfg_t wdgCfg_expected, wdgCfg_actual;

    // Initialize WDG CFGs
    wdgCfg_expected.validParams = PMIC_CFG_WDG_LONGWINDURATION_VALID_SHIFT;
    wdgCfg_actual.validParams = PMIC_CFG_WDG_LONGWINDURATION_VALID_SHIFT;
    wdgCfg_actual.longWinDuration_ms = 0;

    // Enable Watchdog before configuration
    status = Pmic_wdgEnable(&pmicCoreHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Set the min Long Window duration
    wdgCfg_expected.longWinDuration_ms = 80;
    Pmic_wdgSetCfg(&pmicCoreHandle, wdgCfg_expected);

    // Get actual Long Window duration and compare expected vs. actual
    status = Pmic_wdgGetCfg(&pmicCoreHandle, &wdgCfg_expected);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(wdgCfg_expected.longWinDuration_ms, wdgCfg_actual.longWinDuration_ms);

    // Set the mid Long Window duration
    wdgCfg_expected.longWinDuration_ms = 772000 / 2;
    Pmic_wdgSetCfg(&pmicCoreHandle, wdgCfg_expected);

    // Get actual Long Window duration and compare expected vs. actual
    status = Pmic_wdgGetCfg(&pmicCoreHandle, &wdgCfg_expected);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(wdgCfg_expected.longWinDuration_ms, wdgCfg_actual.longWinDuration_ms);

    // Set the max Long Window duration
    wdgCfg_expected.longWinDuration_ms = 772000;
    Pmic_wdgSetCfg(&pmicCoreHandle, wdgCfg_expected);

    // Get actual Long Window duration and compare expected vs. actual
    status = Pmic_wdgGetCfg(&pmicCoreHandle, &wdgCfg_expected);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(wdgCfg_expected.longWinDuration_ms, wdgCfg_actual.longWinDuration_ms);

    // Disable Watchdog after configuration
    status = Pmic_wdgDisable(&pmicCoreHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
}

static void resetAllTps6522xBuckRegisters(void)
{
    uint8_t                                  i = 0;
    uint8_t                                  txBuffer = 0;
    const Pmic_powerTps6522xBuckRegisters_t *pBuckRegisters = NULL;

    // Obtain BUCK registers
    Pmic_get_tps6522x_pwrBuckRegs(&pBuckRegisters);

    // Set values of all BUCK registers to be zero
    (void)pmicI2CWrite(&pmicCoreHandle, PMIC_MAIN_INST, pBuckRegisters[0].buckRailSelRegAddr, &txBuffer, 1);
    for (i = 0; i < PMIC_POWER_TPS6522X_MAX_BUCK_NUM; i++)
    {
        (void)pmicI2CWrite(&pmicCoreHandle, PMIC_MAIN_INST, pBuckRegisters[i].buckCtrlRegAddr, &txBuffer, 1);
        (void)pmicI2CWrite(&pmicCoreHandle, PMIC_MAIN_INST, pBuckRegisters[i].buckConfRegAddr, &txBuffer, 1);
        (void)pmicI2CWrite(&pmicCoreHandle, PMIC_MAIN_INST, pBuckRegisters[i].buckVoutRegAddr, &txBuffer, 1);
        (void)pmicI2CWrite(&pmicCoreHandle, PMIC_MAIN_INST, pBuckRegisters[i].buckPgWindowRegAddr, &txBuffer, 1);
    }
}

static void resetAllTps6522xLdoRegisters(void)
{
    uint8_t                                 i = 0;
    uint8_t                                 txBuffer = 0;
    const Pmic_powerTps6522xLdoRegisters_t *pLdoRegisters = NULL;

    // Obtain LDO registers
    Pmic_get_tps6522x_pwrLdoRegs(&pLdoRegisters);

    // Set values of all LDO registers to be zero
    (void)pmicI2CWrite(&pmicCoreHandle, PMIC_MAIN_INST, pLdoRegisters[0].ldoRailSelRegAddr, &txBuffer, 1);
    for (i = 0; i < PMIC_POWER_TPS6522X_MAX_LDO_NUM; i++)
    {
        (void)pmicI2CWrite(&pmicCoreHandle, PMIC_MAIN_INST, pLdoRegisters[i].ldoCtrlRegAddr, &txBuffer, 1);
        (void)pmicI2CWrite(&pmicCoreHandle, PMIC_MAIN_INST, pLdoRegisters[i].ldoVoutRegAddr, &txBuffer, 1);
        (void)pmicI2CWrite(&pmicCoreHandle, PMIC_MAIN_INST, pLdoRegisters[i].ldoPgWindowRegAddr, &txBuffer, 1);
    }
}

static void resetAllTps6522xVccaVmonRegisters(void)
{
    uint8_t                                      i = 0;
    uint8_t                                      txBuffer = 0;
    const Pmic_powerTps6522xVccaVmonRegisters_t *pVccaVmonRegisters = NULL;

    // Obtain VCCA_VMON/VMONx registers
    Pmic_get_tps6522x_PwrVccaVmonRegisters(&pVccaVmonRegisters);

    // Set values of all VCCA_MON/VMONx registers to be zero
    (void)pmicI2CWrite(&pmicCoreHandle, PMIC_MAIN_INST, pVccaVmonRegisters[0].vccaVmonCtrlRegAddr, &txBuffer, 1);
    (void)pmicI2CWrite(&pmicCoreHandle, PMIC_MAIN_INST, pVccaVmonRegisters[0].vccaVmonRailSelRegAddr, &txBuffer, 1);
    for (i = 0; i < PMIC_POWER_TPS6522X_MAX_VOLTAGE_MONITOR_NUM; i++)
    {
        switch (i)
        {
            case PMIC_POWER_TPS6522X_VOLTAGE_MONITOR_VMON1:
            case PMIC_POWER_TPS6522X_VOLTAGE_MONITOR_VMON2:
                (void)pmicI2CWrite(
                    &pmicCoreHandle, PMIC_MAIN_INST, pVccaVmonRegisters[i].vmonPgLevelRegAddr, &txBuffer, 1);
                (void)pmicI2CWrite(
                    &pmicCoreHandle, PMIC_MAIN_INST, pVccaVmonRegisters[i].vmonPgWindowRegAddr, &txBuffer, 1);

                break;
            case PMIC_POWER_TPS6522X_VOLTAGE_MONITOR_VCCA_VMON:
                (void)pmicI2CWrite(
                    &pmicCoreHandle, PMIC_MAIN_INST, pVccaVmonRegisters[i].vccaPgWindowRegAddr, &txBuffer, 1);

                break;
        }
    }
}

static void disablePmicPowerResources(void)
{
    const uint8_t devIdReg = 0x01;
    uint8_t       pmicDevId = 0x00;
    int32_t       status = PMIC_ST_SUCCESS;

    // Wait for PMIC connection
    while (1)
    {
        status = pmicI2CRead(&pmicCoreHandle, PMIC_MAIN_INST, devIdReg, &pmicDevId, 1);

        if (status == PMIC_ST_SUCCESS)
        {
            break;
        }
    }

    // Set all BUCK registers to zero
    resetAllTps6522xBuckRegisters();

    // Set all LDO registers to zero
    resetAllTps6522xLdoRegisters();

    // Set all VCCA_VMON/VMONx registers to zero
    resetAllTps6522xVccaVmonRegisters();
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
