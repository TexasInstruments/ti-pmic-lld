/**
 * \file gpio_test.c
 * \author John Bui (j-bui1@ti.com)
 * \brief Unity testing application that tests the PMIC driver GPIO set/get APIs
 * \version 1.0
 * \date 2023-10-11
 *
 * \copyright Copyright (c) 2023
 */

/* Standard includes */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

/* Tiva test app library */
#include "pmic_drv/burton_testAppLib/tiva/tiva_testLib.h"

/* Test specific include */
#include "pmic_drv/burton_testAppLib/tiva/burton_tests/gpio_test.h"

/* Unity testing library */
#include "unity/unity.h"

/* PMIC driver */
#include "pmic_drv/pmic.h"

Pmic_CoreHandle_t pmicCoreHandle;
gpioPinHandle_t   gpioPinHandle[PMIC_TPS6522X_GPIO_PIN_MAX];

int main(void)
{
    /*** Variable declaration/initialization ***/
    // clang-format off
    uartHandle_t vcpHandle;
    i2cHandle_t I2C1Handle;
    Pmic_CoreCfg_t pmicConfigData = {
        .validParams =
            PMIC_CFG_DEVICE_TYPE_VALID_SHIFT | PMIC_CFG_COMM_MODE_VALID_SHIFT | PMIC_CFG_SLAVEADDR_VALID_SHIFT |
            PMIC_CFG_QASLAVEADDR_VALID_SHIFT | PMIC_CFG_NVMSLAVEADDR_VALID_SHIFT | PMIC_CFG_COMM_HANDLE_VALID_SHIFT |
            PMIC_CFG_COMM_IO_RD_VALID_SHIFT | PMIC_CFG_COMM_IO_WR_VALID_SHIFT | PMIC_CFG_I2C1_SPEED_VALID_SHIFT,
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

    /*** GPIO setup ***/
    initializeGpioPinHandles(gpioPinHandle, false);
    initializeGpioPins(gpioPinHandle);

    /*** PMIC setup ***/
    initializePmicCoreHandle(&pmicCoreHandle);
    Pmic_init(&pmicConfigData, &pmicCoreHandle);

    /*** Clear the console before printing anything ***/
    clearConsole(&vcpHandle);

    /*** Print welcome message ***/
    UARTStrPut(&vcpHandle, "Running all PMIC GPIO tests...\r\n\r\n");

    /*** Begin unity testing ***/
    UNITY_BEGIN();

    RUN_TEST(test_Pmic_gpioSetValue_setGpioSignalLvl);
    RUN_TEST(test_Pmic_gpioGetValue_readGpioSignalLvl);
    RUN_TEST(test_Pmic_gpioSetConfiguration_VMON1_m);
    RUN_TEST(test_Pmic_gpioSetConfiguration_VMON2);
    RUN_TEST(test_Pmic_gpioSetConfiguration_pushButton);
    RUN_TEST(test_Pmic_gpioSetConfiguration_nSLEEP1);
    RUN_TEST(test_Pmic_gpioSetConfiguration_nSLEEP2);
    RUN_TEST(test_Pmic_gpioSetConfiguration_ADC_IN);
    RUN_TEST(test_Pmic_gpioSetConfiguration_WKUP);
    RUN_TEST(test_Pmic_gpioSetConfiguration_SYNCCLKIN);
    RUN_TEST(test_Pmic_gpioSetConfiguration_nERR_MCU);
    RUN_TEST(test_Pmic_gpioSetConfiguration_SDA_I2C2_SDO_SPI);
    RUN_TEST(test_Pmic_gpioSetConfiguration_SCL_I2C2_CS_SPI);
    RUN_TEST(test_Pmic_gpioSetConfiguration_nINT);
    RUN_TEST(test_Pmic_gpioSetConfiguration_TRIG_WDOG);

    /*** Finish unity testing ***/
    return UNITY_END();
}

/**
 * \brief test Pmic_gpioSetConfiguration: Configure a GPIO pin to TRIG_WDOG functionality
 *
 */
void test_Pmic_gpioSetConfiguration_TRIG_WDOG(void)
{
    const uint8_t  pin = PMIC_TPS6522X_GPIO2_PIN;
    int32_t        status = PMIC_ST_SUCCESS;
    Pmic_GpioCfg_t nvmGpioCfg, actualGpioCfg, expectedGpioCfg;

    // Initialize all config structs
    // clang-format off
   resetGpioCfg_withAllValidParams(&nvmGpioCfg);
   resetGpioCfg_withSpecificValidParams(&actualGpioCfg, PMIC_GPIO_CFG_DIR_VALID_SHIFT     |
                                                        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT |
                                                        PMIC_GPIO_CFG_PULL_VALID_SHIFT    |
                                                        PMIC_GPIO_CFG_DEGLITCH_VALID_SHIFT);
   resetGpioCfg_withSpecificValidParams(&expectedGpioCfg, PMIC_GPIO_CFG_DIR_VALID_SHIFT     |
                                                          PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT |
                                                          PMIC_GPIO_CFG_PULL_VALID_SHIFT    |
                                                          PMIC_GPIO_CFG_DEGLITCH_VALID_SHIFT);
    // clang-format on

    // Save NVM configuration of pin
    status = Pmic_gpioGetConfiguration(&pmicCoreHandle, pin, &nvmGpioCfg);
    TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);

    // Set new configuration of pin
    expectedGpioCfg.pinDir = PMIC_GPIO_INPUT;
    expectedGpioCfg.pinFunc = PMIC_TPS6522X_GPIO_PINFUNC_GPIO2_TRIG_WDOG;
    expectedGpioCfg.pullCtrl = PMIC_GPIO_PULL_DOWN;
    expectedGpioCfg.deglitchEnable = PMIC_GPIO_DEGLITCH_ENABLE;
    status = Pmic_gpioSetConfiguration(&pmicCoreHandle, pin, expectedGpioCfg);
    TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);

    // Get actual configuration of pin
    status = Pmic_gpioGetConfiguration(&pmicCoreHandle, pin, &actualGpioCfg);
    TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);

    // Compare actual vs. expected configuration
    TEST_ASSERT_EQUAL_UINT8(expectedGpioCfg.pinDir, actualGpioCfg.pinDir);
    TEST_ASSERT_EQUAL_UINT8(expectedGpioCfg.pinFunc, actualGpioCfg.pinFunc);
    TEST_ASSERT_EQUAL_UINT8(expectedGpioCfg.pullCtrl, actualGpioCfg.pullCtrl);
    TEST_ASSERT_EQUAL_UINT8(expectedGpioCfg.deglitchEnable, actualGpioCfg.deglitchEnable);

    // Restore NVM configuration of pin
    status = Pmic_gpioSetConfiguration(&pmicCoreHandle, pin, nvmGpioCfg);
    TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);
}

/**
 * \brief test Pmic_gpioSetConfiguration: Configure a GPIO pin to nINT functionality
 */
void test_Pmic_gpioSetConfiguration_nINT(void)
{
    const uint8_t  pin = PMIC_TPS6522X_GPIO1_PIN;
    int32_t        status = PMIC_ST_SUCCESS;
    Pmic_GpioCfg_t nvmGpioCfg, actualGpioCfg, expectedGpioCfg;

    // Initialize all config structs
    // clang-format off
   resetGpioCfg_withAllValidParams(&nvmGpioCfg);
   resetGpioCfg_withSpecificValidParams(&actualGpioCfg, PMIC_GPIO_CFG_DIR_VALID_SHIFT     |
                                                        PMIC_GPIO_CFG_OD_VALID_SHIFT      |
                                                        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT |
                                                        PMIC_GPIO_CFG_PULL_VALID_SHIFT);
   resetGpioCfg_withSpecificValidParams(&expectedGpioCfg, PMIC_GPIO_CFG_DIR_VALID_SHIFT     |
                                                          PMIC_GPIO_CFG_OD_VALID_SHIFT      |
                                                          PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT |
                                                          PMIC_GPIO_CFG_PULL_VALID_SHIFT);
    // clang-format on

    // Save NVM configuration of pin
    status = Pmic_gpioGetConfiguration(&pmicCoreHandle, pin, &nvmGpioCfg);
    TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);

    // Set new configuration of pin
    expectedGpioCfg.pinDir = PMIC_GPIO_OUTPUT;
    expectedGpioCfg.outputSignalType = PMIC_GPIO_OPEN_DRAIN_OUTPUT;
    expectedGpioCfg.pinFunc = PMIC_TPS6522X_GPIO_PINFUNC_GPIO1_NINT;
    expectedGpioCfg.pullCtrl = PMIC_GPIO_PULL_UP;
    status = Pmic_gpioSetConfiguration(&pmicCoreHandle, pin, expectedGpioCfg);
    TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);

    // Get actual configuration of pin
    status = Pmic_gpioGetConfiguration(&pmicCoreHandle, pin, &actualGpioCfg);
    TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);

    // Compare actual vs. expected configuration
    TEST_ASSERT_EQUAL_UINT8(expectedGpioCfg.pinDir, actualGpioCfg.pinDir);
    TEST_ASSERT_EQUAL_UINT8(expectedGpioCfg.outputSignalType, actualGpioCfg.outputSignalType);
    TEST_ASSERT_EQUAL_UINT8(expectedGpioCfg.pinFunc, actualGpioCfg.pinFunc);
    TEST_ASSERT_EQUAL_UINT8(expectedGpioCfg.pullCtrl, actualGpioCfg.pullCtrl);

    // Restore NVM configuration of pin
    status = Pmic_gpioSetConfiguration(&pmicCoreHandle, pin, nvmGpioCfg);
    TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);
}

/**
 * \brief test Pmic_gpioSetConfiguration: Configure a GPIO pin to SCL_I2C2/CS_SPI functionality
 */
void test_Pmic_gpioSetConfiguration_SCL_I2C2_CS_SPI(void)
{
    const uint8_t  pin = PMIC_TPS6522X_GPIO2_PIN;
    int32_t        status = PMIC_ST_SUCCESS;
    Pmic_GpioCfg_t nvmGpioCfg, actualGpioCfg, expectedGpioCfg;

    // Initialize all config structs
    // clang-format off
   resetGpioCfg_withAllValidParams(&nvmGpioCfg);
   resetGpioCfg_withSpecificValidParams(&actualGpioCfg, PMIC_GPIO_CFG_DIR_VALID_SHIFT     |
                                                        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT |
                                                        PMIC_GPIO_CFG_DEGLITCH_VALID_SHIFT);
   resetGpioCfg_withSpecificValidParams(&expectedGpioCfg, PMIC_GPIO_CFG_DIR_VALID_SHIFT     |
                                                          PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT |
                                                          PMIC_GPIO_CFG_DEGLITCH_VALID_SHIFT);
    // clang-format on

    // Save NVM configuration of pin
    status = Pmic_gpioGetConfiguration(&pmicCoreHandle, pin, &nvmGpioCfg);
    TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);

    // Set new configuration of pin
    expectedGpioCfg.pinDir = PMIC_GPIO_INPUT;
    expectedGpioCfg.pinFunc = PMIC_TPS6522X_GPIO_PINFUNC_GPIO2_SCL_I2C2_CS_SPI;
    expectedGpioCfg.deglitchEnable = PMIC_GPIO_DEGLITCH_ENABLE;
    status = Pmic_gpioSetConfiguration(&pmicCoreHandle, pin, expectedGpioCfg);
    TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);

    // Get actual configuration of pin
    status = Pmic_gpioGetConfiguration(&pmicCoreHandle, pin, &actualGpioCfg);
    TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);

    // Compare actual vs. expected configuration
    TEST_ASSERT_EQUAL_UINT8(expectedGpioCfg.pinDir, actualGpioCfg.pinDir);
    TEST_ASSERT_EQUAL_UINT8(expectedGpioCfg.pinFunc, actualGpioCfg.pinFunc);
    TEST_ASSERT_EQUAL_UINT8(expectedGpioCfg.deglitchEnable, actualGpioCfg.deglitchEnable);

    // Restore NVM configuration of pin
    status = Pmic_gpioSetConfiguration(&pmicCoreHandle, pin, nvmGpioCfg);
    TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);
}

/**
 * \brief test Pmic_gpioSetConfiguration: Configure a GPIO pin to SDA_I2C2/SDO_SPI functionality
 */
void test_Pmic_gpioSetConfiguration_SDA_I2C2_SDO_SPI(void)
{
    const uint8_t  pin = PMIC_TPS6522X_GPIO1_PIN;
    int32_t        status = PMIC_ST_SUCCESS;
    Pmic_GpioCfg_t nvmGpioCfg, actualGpioCfg, expectedGpioCfg;

    // Initialize all config structs
    // clang-format off
   resetGpioCfg_withAllValidParams(&nvmGpioCfg);
   resetGpioCfg_withSpecificValidParams(&actualGpioCfg, PMIC_GPIO_CFG_OD_VALID_SHIFT      |
                                                        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT |
                                                        PMIC_GPIO_CFG_DEGLITCH_VALID_SHIFT);
   resetGpioCfg_withSpecificValidParams(&expectedGpioCfg, PMIC_GPIO_CFG_OD_VALID_SHIFT      |
                                                          PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT |
                                                          PMIC_GPIO_CFG_DEGLITCH_VALID_SHIFT);
    // clang-format on

    // Save NVM configuration of pin
    status = Pmic_gpioGetConfiguration(&pmicCoreHandle, pin, &nvmGpioCfg);
    TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);

    // Set new configuration of pin
    expectedGpioCfg.outputSignalType = PMIC_GPIO_OPEN_DRAIN_OUTPUT;
    expectedGpioCfg.pinFunc = PMIC_TPS6522X_GPIO_PINFUNC_GPIO1_SDA_I2C2_SDO_SPI;
    expectedGpioCfg.deglitchEnable = PMIC_GPIO_DEGLITCH_ENABLE;
    status = Pmic_gpioSetConfiguration(&pmicCoreHandle, pin, expectedGpioCfg);
    TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);

    // Get actual configuration of pin
    status = Pmic_gpioGetConfiguration(&pmicCoreHandle, pin, &actualGpioCfg);
    TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);

    // Compare actual vs. expected configuration
    TEST_ASSERT_EQUAL_UINT8(expectedGpioCfg.outputSignalType, actualGpioCfg.outputSignalType);
    TEST_ASSERT_EQUAL_UINT8(expectedGpioCfg.pinFunc, actualGpioCfg.pinFunc);
    TEST_ASSERT_EQUAL_UINT8(expectedGpioCfg.deglitchEnable, actualGpioCfg.deglitchEnable);

    // Restore NVM configuration of pin
    status = Pmic_gpioSetConfiguration(&pmicCoreHandle, pin, nvmGpioCfg);
    TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);
}

/**
 * \brief test Pmic_gpioSetConfiguration: Configure a GPIO pin to nERR_MCU functionality
 */
void test_Pmic_gpioSetConfiguration_nERR_MCU(void)
{
    const uint8_t  pin = PMIC_TPS6522X_GPIO6_PIN;
    int32_t        status = PMIC_ST_SUCCESS;
    Pmic_GpioCfg_t nvmGpioCfg, actualGpioCfg, expectedGpioCfg;

    // Initialize all config structs
    // clang-format off
   resetGpioCfg_withAllValidParams(&nvmGpioCfg);
   resetGpioCfg_withSpecificValidParams(&actualGpioCfg, PMIC_GPIO_CFG_DIR_VALID_SHIFT     |
                                                        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT |
                                                        PMIC_GPIO_CFG_PULL_VALID_SHIFT    |
                                                        PMIC_GPIO_CFG_DEGLITCH_VALID_SHIFT);
   resetGpioCfg_withSpecificValidParams(&expectedGpioCfg, PMIC_GPIO_CFG_DIR_VALID_SHIFT     |
                                                          PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT |
                                                          PMIC_GPIO_CFG_PULL_VALID_SHIFT    |
                                                          PMIC_GPIO_CFG_DEGLITCH_VALID_SHIFT);
    // clang-format on

    // Save NVM configuration of pin
    status = Pmic_gpioGetConfiguration(&pmicCoreHandle, pin, &nvmGpioCfg);
    TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);

    // Set new configuration of pin
    expectedGpioCfg.pinDir = PMIC_GPIO_INPUT;
    expectedGpioCfg.pinFunc = PMIC_TPS6522X_GPIO_PINFUNC_GPIO6_NERR_MCU;
    expectedGpioCfg.pullCtrl = PMIC_GPIO_PULL_DOWN;
    expectedGpioCfg.deglitchEnable = PMIC_GPIO_DEGLITCH_ENABLE;
    status = Pmic_gpioSetConfiguration(&pmicCoreHandle, pin, expectedGpioCfg);
    TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);

    // Get actual configuration of pin
    status = Pmic_gpioGetConfiguration(&pmicCoreHandle, pin, &actualGpioCfg);
    TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);

    // Compare actual vs. expected configuration
    TEST_ASSERT_EQUAL_UINT8(expectedGpioCfg.pinDir, actualGpioCfg.pinDir);
    TEST_ASSERT_EQUAL_UINT8(expectedGpioCfg.pinFunc, actualGpioCfg.pinFunc);
    TEST_ASSERT_EQUAL_UINT8(expectedGpioCfg.pullCtrl, actualGpioCfg.pullCtrl);
    TEST_ASSERT_EQUAL_UINT8(expectedGpioCfg.deglitchEnable, actualGpioCfg.deglitchEnable);

    // Restore NVM configuration of pin
    status = Pmic_gpioSetConfiguration(&pmicCoreHandle, pin, nvmGpioCfg);
    TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);
}

/**
 * \brief test Pmic_gpioSetConfiguration: Configure a GPIO pin to SYNCCLKIN functionality
 */
void test_Pmic_gpioSetConfiguration_SYNCCLKIN(void)
{
    const uint8_t  pin = PMIC_TPS6522X_GPIO5_PIN;
    int32_t        status = PMIC_ST_SUCCESS;
    Pmic_GpioCfg_t nvmGpioCfg, actualGpioCfg, expectedGpioCfg;

    // Initialize all config structs
    // clang-format off
   resetGpioCfg_withAllValidParams(&nvmGpioCfg);
   resetGpioCfg_withSpecificValidParams(&actualGpioCfg, PMIC_GPIO_CFG_DIR_VALID_SHIFT     |
                                                        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT |
                                                        PMIC_GPIO_CFG_PULL_VALID_SHIFT);
   resetGpioCfg_withSpecificValidParams(&expectedGpioCfg, PMIC_GPIO_CFG_DIR_VALID_SHIFT     |
                                                          PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT |
                                                          PMIC_GPIO_CFG_PULL_VALID_SHIFT);
    // clang-format on

    // Save NVM configuration of pin
    status = Pmic_gpioGetConfiguration(&pmicCoreHandle, pin, &nvmGpioCfg);
    TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);

    // Set new configuration of pin
    expectedGpioCfg.pinDir = PMIC_GPIO_INPUT;
    expectedGpioCfg.pinFunc = PMIC_TPS6522X_GPIO_PINFUNC_GPIO5_SYNCCLKIN;
    expectedGpioCfg.pullCtrl = PMIC_GPIO_PULL_DOWN;
    status = Pmic_gpioSetConfiguration(&pmicCoreHandle, pin, expectedGpioCfg);
    TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);

    // Get actual configuration of pin
    status = Pmic_gpioGetConfiguration(&pmicCoreHandle, pin, &actualGpioCfg);
    TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);

    // Compare actual vs. expected configuration
    TEST_ASSERT_EQUAL_UINT8(expectedGpioCfg.pinDir, actualGpioCfg.pinDir);
    TEST_ASSERT_EQUAL_UINT8(expectedGpioCfg.pinFunc, actualGpioCfg.pinFunc);
    TEST_ASSERT_EQUAL_UINT8(expectedGpioCfg.pullCtrl, actualGpioCfg.pullCtrl);

    // Restore NVM configuration of pin
    status = Pmic_gpioSetConfiguration(&pmicCoreHandle, pin, nvmGpioCfg);
    TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);
}

/**
 * \brief test Pmic_gpioSetConfiguration: Configure a GPIO pin to WKUP functionality
 */
void test_Pmic_gpioSetConfiguration_WKUP(void)
{
    const uint8_t  pin = PMIC_TPS6522X_GPIO5_PIN;
    int32_t        status = PMIC_ST_SUCCESS;
    Pmic_GpioCfg_t nvmGpioCfg, actualGpioCfg, expectedGpioCfg;

    // Initialize all config structs
    // clang-format off
   resetGpioCfg_withAllValidParams(&nvmGpioCfg);
   resetGpioCfg_withSpecificValidParams(&actualGpioCfg, PMIC_GPIO_CFG_DIR_VALID_SHIFT     |
                                                        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT |
                                                        PMIC_GPIO_CFG_PULL_VALID_SHIFT    |
                                                        PMIC_GPIO_CFG_DEGLITCH_VALID_SHIFT);
   resetGpioCfg_withSpecificValidParams(&expectedGpioCfg, PMIC_GPIO_CFG_DIR_VALID_SHIFT     |
                                                          PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT |
                                                          PMIC_GPIO_CFG_PULL_VALID_SHIFT    |
                                                          PMIC_GPIO_CFG_DEGLITCH_VALID_SHIFT);
    // clang-format on

    // Save NVM configuration of pin
    status = Pmic_gpioGetConfiguration(&pmicCoreHandle, pin, &nvmGpioCfg);
    TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);

    // Set new configuration of pin
    expectedGpioCfg.pinDir = PMIC_GPIO_INPUT;
    expectedGpioCfg.pinFunc = PMIC_TPS6522X_GPIO_PINFUNC_GPIO5_WKUP;
    expectedGpioCfg.pullCtrl = PMIC_GPIO_PULL_DOWN;
    expectedGpioCfg.deglitchEnable = PMIC_GPIO_DEGLITCH_ENABLE;
    status = Pmic_gpioSetConfiguration(&pmicCoreHandle, pin, expectedGpioCfg);
    TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);

    // Get actual configuration of pin
    status = Pmic_gpioGetConfiguration(&pmicCoreHandle, pin, &actualGpioCfg);
    TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);

    // Compare actual vs. expected configuration
    TEST_ASSERT_EQUAL_UINT8(expectedGpioCfg.pinDir, actualGpioCfg.pinDir);
    TEST_ASSERT_EQUAL_UINT8(expectedGpioCfg.pinFunc, actualGpioCfg.pinFunc);
    TEST_ASSERT_EQUAL_UINT8(expectedGpioCfg.pullCtrl, actualGpioCfg.pullCtrl);
    TEST_ASSERT_EQUAL_UINT8(expectedGpioCfg.deglitchEnable, actualGpioCfg.deglitchEnable);

    // Restore NVM configuration of pin
    status = Pmic_gpioSetConfiguration(&pmicCoreHandle, pin, nvmGpioCfg);
    TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);
}

/**
 * \brief test Pmic_gpioSetConfiguration: Configure a GPIO pin to ADC_IN functionality
 */
void test_Pmic_gpioSetConfiguration_ADC_IN(void)
{
    const uint8_t  pin = PMIC_TPS6522X_GPIO4_PIN;
    int32_t        status = PMIC_ST_SUCCESS;
    Pmic_GpioCfg_t nvmGpioCfg, actualGpioCfg, expectedGpioCfg;

    // Initialize all config structs
    // clang-format off
   resetGpioCfg_withAllValidParams(&nvmGpioCfg);
   resetGpioCfg_withSpecificValidParams(&actualGpioCfg, PMIC_GPIO_CFG_DIR_VALID_SHIFT |
                                                        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT);
   resetGpioCfg_withSpecificValidParams(&expectedGpioCfg, PMIC_GPIO_CFG_DIR_VALID_SHIFT |
                                                          PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT);
    // clang-format on

    // Save NVM configuration of pin
    status = Pmic_gpioGetConfiguration(&pmicCoreHandle, pin, &nvmGpioCfg);
    TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);

    // Set new configuration of pin
    expectedGpioCfg.pinDir = PMIC_GPIO_INPUT;
    expectedGpioCfg.pinFunc = PMIC_TPS6522X_GPIO_PINFUNC_GPIO4_ADC_IN;
    status = Pmic_gpioSetConfiguration(&pmicCoreHandle, pin, expectedGpioCfg);
    TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);

    // Get actual configuration of pin
    status = Pmic_gpioGetConfiguration(&pmicCoreHandle, pin, &actualGpioCfg);
    TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);

    // Compare actual vs. expected configuration
    TEST_ASSERT_EQUAL_UINT8(expectedGpioCfg.pinDir, actualGpioCfg.pinDir);
    TEST_ASSERT_EQUAL_UINT8(expectedGpioCfg.pinFunc, actualGpioCfg.pinFunc);

    // Restore NVM configuration of pin
    status = Pmic_gpioSetConfiguration(&pmicCoreHandle, pin, nvmGpioCfg);
    TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);
}

/**
 * \brief test Pmic_gpioSetConfiguration: Configure a GPIO pin to nSLEEP2 functionality
 */
void test_Pmic_gpioSetConfiguration_nSLEEP2(void)
{
    const uint8_t  pin = PMIC_TPS6522X_GPIO6_PIN;
    int32_t        status = PMIC_ST_SUCCESS;
    Pmic_GpioCfg_t nvmGpioCfg, actualGpioCfg, expectedGpioCfg;

    // Initialize all config structs
    // clang-format off
   resetGpioCfg_withAllValidParams(&nvmGpioCfg);
   resetGpioCfg_withSpecificValidParams(&actualGpioCfg, PMIC_GPIO_CFG_DIR_VALID_SHIFT     |
                                                        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT |
                                                        PMIC_GPIO_CFG_PULL_VALID_SHIFT    |
                                                        PMIC_GPIO_CFG_DEGLITCH_VALID_SHIFT);
   resetGpioCfg_withSpecificValidParams(&expectedGpioCfg, PMIC_GPIO_CFG_DIR_VALID_SHIFT     |
                                                          PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT |
                                                          PMIC_GPIO_CFG_PULL_VALID_SHIFT    |
                                                          PMIC_GPIO_CFG_DEGLITCH_VALID_SHIFT);
    // clang-format on

    // Save NVM configuration of pin
    status = Pmic_gpioGetConfiguration(&pmicCoreHandle, pin, &nvmGpioCfg);
    TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);

    // Set new configuration of pin
    expectedGpioCfg.pinDir = PMIC_GPIO_INPUT;
    expectedGpioCfg.pinFunc = PMIC_TPS6522X_GPIO_PINFUNC_GPIO6_NSLEEP2;
    expectedGpioCfg.pullCtrl = PMIC_GPIO_PULL_UP;
    expectedGpioCfg.deglitchEnable = PMIC_GPIO_DEGLITCH_ENABLE;
    status = Pmic_gpioSetConfiguration(&pmicCoreHandle, pin, expectedGpioCfg);
    TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);

    // Get actual configuration of pin
    status = Pmic_gpioGetConfiguration(&pmicCoreHandle, pin, &actualGpioCfg);
    TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);

    // Compare actual vs. expected configuration
    TEST_ASSERT_EQUAL_UINT8(expectedGpioCfg.pinDir, actualGpioCfg.pinDir);
    TEST_ASSERT_EQUAL_UINT8(expectedGpioCfg.pinFunc, actualGpioCfg.pinFunc);
    TEST_ASSERT_EQUAL_UINT8(expectedGpioCfg.pullCtrl, actualGpioCfg.pullCtrl);
    TEST_ASSERT_EQUAL_UINT8(expectedGpioCfg.deglitchEnable, actualGpioCfg.deglitchEnable);

    // Restore NVM configuration of pin
    status = Pmic_gpioSetConfiguration(&pmicCoreHandle, pin, nvmGpioCfg);
    TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);
}

/**
 * \brief test Pmic_gpioSetConfiguration: Configure a GPIO pin to nSLEEP1 functionality
 */
void test_Pmic_gpioSetConfiguration_nSLEEP1(void)
{
    const uint8_t  pin = PMIC_TPS6522X_GPIO2_PIN;
    int32_t        status = PMIC_ST_SUCCESS;
    Pmic_GpioCfg_t nvmGpioCfg, actualGpioCfg, expectedGpioCfg;

    // Initialize all config structs
    // clang-format off
   resetGpioCfg_withAllValidParams(&nvmGpioCfg);
   resetGpioCfg_withSpecificValidParams(&actualGpioCfg, PMIC_GPIO_CFG_DIR_VALID_SHIFT     |
                                                        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT |
                                                        PMIC_GPIO_CFG_PULL_VALID_SHIFT    |
                                                        PMIC_GPIO_CFG_DEGLITCH_VALID_SHIFT);
   resetGpioCfg_withSpecificValidParams(&expectedGpioCfg, PMIC_GPIO_CFG_DIR_VALID_SHIFT     |
                                                          PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT |
                                                          PMIC_GPIO_CFG_PULL_VALID_SHIFT    |
                                                          PMIC_GPIO_CFG_DEGLITCH_VALID_SHIFT);
    // clang-format on

    // Save NVM configuration of pin
    status = Pmic_gpioGetConfiguration(&pmicCoreHandle, pin, &nvmGpioCfg);
    TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);

    // Set new configuration of pin
    expectedGpioCfg.pinDir = PMIC_GPIO_INPUT;
    expectedGpioCfg.pinFunc = PMIC_TPS6522X_GPIO_PINFUNC_GPIO2_NSLEEP1;
    expectedGpioCfg.pullCtrl = PMIC_GPIO_PULL_UP;
    expectedGpioCfg.deglitchEnable = PMIC_GPIO_DEGLITCH_ENABLE;
    status = Pmic_gpioSetConfiguration(&pmicCoreHandle, pin, expectedGpioCfg);
    TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);

    // Get actual configuration of pin
    status = Pmic_gpioGetConfiguration(&pmicCoreHandle, pin, &actualGpioCfg);
    TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);

    // Compare actual vs. expected configuration
    TEST_ASSERT_EQUAL_UINT8(expectedGpioCfg.pinDir, actualGpioCfg.pinDir);
    TEST_ASSERT_EQUAL_UINT8(expectedGpioCfg.pinFunc, actualGpioCfg.pinFunc);
    TEST_ASSERT_EQUAL_UINT8(expectedGpioCfg.pullCtrl, actualGpioCfg.pullCtrl);
    TEST_ASSERT_EQUAL_UINT8(expectedGpioCfg.deglitchEnable, actualGpioCfg.deglitchEnable);

    // Restore NVM configuration of pin
    status = Pmic_gpioSetConfiguration(&pmicCoreHandle, pin, nvmGpioCfg);
    TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);
}

/**
 * \brief test Pmic_gpioSetConfiguration: Configure a GPIO pin to Push Button functionality
 */
void test_Pmic_gpioSetConfiguration_pushButton(void)
{
    const uint8_t  pin = PMIC_TPS6522X_GPIO3_PIN;
    int32_t        status = PMIC_ST_SUCCESS;
    Pmic_GpioCfg_t nvmGpioCfg, actualGpioCfg, expectedGpioCfg;

    // Initialize all config structs
    // clang-format off
   resetGpioCfg_withAllValidParams(&nvmGpioCfg);
   resetGpioCfg_withSpecificValidParams(&actualGpioCfg, PMIC_GPIO_CFG_DIR_VALID_SHIFT     |
                                                        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT |
                                                        PMIC_GPIO_CFG_PULL_VALID_SHIFT    |
                                                        PMIC_GPIO_CFG_DEGLITCH_VALID_SHIFT);
   resetGpioCfg_withSpecificValidParams(&expectedGpioCfg, PMIC_GPIO_CFG_DIR_VALID_SHIFT     |
                                                          PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT |
                                                          PMIC_GPIO_CFG_PULL_VALID_SHIFT    |
                                                          PMIC_GPIO_CFG_DEGLITCH_VALID_SHIFT);
    // clang-format on

    // Save NVM configuration of pin
    status = Pmic_gpioGetConfiguration(&pmicCoreHandle, pin, &nvmGpioCfg);
    TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);

    // Set new configuration of pin
    expectedGpioCfg.pinDir = PMIC_GPIO_INPUT;
    expectedGpioCfg.pinFunc = PMIC_TPS6522X_GPIO_PINFUNC_GPIO3_PUSH_BUTTON;
    expectedGpioCfg.pullCtrl = PMIC_GPIO_PULL_UP;
    expectedGpioCfg.deglitchEnable = PMIC_GPIO_DEGLITCH_ENABLE;
    status = Pmic_gpioSetConfiguration(&pmicCoreHandle, pin, expectedGpioCfg);
    TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);

    // Get actual configuration of pin
    status = Pmic_gpioGetConfiguration(&pmicCoreHandle, pin, &actualGpioCfg);
    TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);

    // Compare actual vs. expected configuration
    TEST_ASSERT_EQUAL_UINT8(expectedGpioCfg.pinDir, actualGpioCfg.pinDir);
    TEST_ASSERT_EQUAL_UINT8(expectedGpioCfg.pinFunc, actualGpioCfg.pinFunc);
    TEST_ASSERT_EQUAL_UINT8(expectedGpioCfg.pullCtrl, actualGpioCfg.pullCtrl);
    TEST_ASSERT_EQUAL_UINT8(expectedGpioCfg.deglitchEnable, actualGpioCfg.deglitchEnable);

    // Restore NVM configuration of pin
    status = Pmic_gpioSetConfiguration(&pmicCoreHandle, pin, nvmGpioCfg);
    TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);
}

/**
 * \brief test Pmic_gpioSetConfiguration: Configure a GPIO pin to VMON2 functionality
 */
void test_Pmic_gpioSetConfiguration_VMON2(void)
{
    const uint8_t  pin = PMIC_TPS6522X_GPIO4_PIN;
    int32_t        status = PMIC_ST_SUCCESS;
    Pmic_GpioCfg_t nvmGpioCfg, actualGpioCfg, expectedGpioCfg;

    // Initialize all config structs
    // clang-format off
    resetGpioCfg_withAllValidParams(&nvmGpioCfg);
    resetGpioCfg_withSpecificValidParams(&actualGpioCfg, PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT |
                                                         PMIC_GPIO_CFG_DEGLITCH_VALID_SHIFT);
    resetGpioCfg_withSpecificValidParams(&expectedGpioCfg, PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT |
                                                           PMIC_GPIO_CFG_DEGLITCH_VALID_SHIFT);
    // clang-format on

    // Save NVM configuration of pin
    status = Pmic_gpioGetConfiguration(&pmicCoreHandle, pin, &nvmGpioCfg);
    TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);

    // Set new configuration of pin
    expectedGpioCfg.pinFunc = PMIC_TPS6522X_GPIO_PINFUNC_GPIO4_VMON2;
    expectedGpioCfg.deglitchEnable = PMIC_GPIO_DEGLITCH_ENABLE;
    status = Pmic_gpioSetConfiguration(&pmicCoreHandle, pin, expectedGpioCfg);
    TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);

    // Get actual configuration of pin
    status = Pmic_gpioGetConfiguration(&pmicCoreHandle, pin, &actualGpioCfg);
    TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);

    // Compare actual vs. expected configuration
    TEST_ASSERT_EQUAL_UINT8(expectedGpioCfg.pinFunc, actualGpioCfg.pinFunc);

    // Restore NVM configuration of pin
    status = Pmic_gpioSetConfiguration(&pmicCoreHandle, pin, nvmGpioCfg);
    TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);
}

/**
 * \brief test Pmic_gpioSetConfiguration: Configure a GPIO pin to VMON1_m functionality
 */
void test_Pmic_gpioSetConfiguration_VMON1_m(void)
{
    const uint8_t  pin = PMIC_TPS6522X_GPIO3_PIN;
    int32_t        status = PMIC_ST_SUCCESS;
    Pmic_GpioCfg_t nvmGpioCfg, actualGpioCfg, expectedGpioCfg;

    // Initialize all config structs
    // clang-format off
    resetGpioCfg_withAllValidParams(&nvmGpioCfg);
    resetGpioCfg_withSpecificValidParams(&actualGpioCfg, PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT|
                                                         PMIC_GPIO_CFG_DEGLITCH_VALID_SHIFT);
    resetGpioCfg_withSpecificValidParams(&expectedGpioCfg, PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT |
                                                           PMIC_GPIO_CFG_DEGLITCH_VALID_SHIFT);
    // clang-format on

    // Save NVM configuration of pin
    status = Pmic_gpioGetConfiguration(&pmicCoreHandle, pin, &nvmGpioCfg);
    TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);

    // Set new configuration of pin
    expectedGpioCfg.pinFunc = PMIC_TPS6522X_GPIO_PINFUNC_GPIO3_VMON1_M;
    expectedGpioCfg.deglitchEnable = PMIC_GPIO_DEGLITCH_ENABLE;
    status = Pmic_gpioSetConfiguration(&pmicCoreHandle, pin, expectedGpioCfg);
    TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);

    // Get actual configuration of pin
    status = Pmic_gpioGetConfiguration(&pmicCoreHandle, pin, &actualGpioCfg);
    TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);

    // Compare actual vs. expected configuration
    TEST_ASSERT_EQUAL_UINT8(expectedGpioCfg.pinFunc, actualGpioCfg.pinFunc);

    // Restore NVM configuration of pin
    status = Pmic_gpioSetConfiguration(&pmicCoreHandle, pin, nvmGpioCfg);
    TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);
}

/**
 * \brief test Pmic_gpioSetValue: Set all GPIO output values to one, then reset all GPIO output values to zero.
 *
 * \note For this test, there must be a total of 6 physical GPIO pin connections between Burton and test MCU
 */
void test_Pmic_gpioSetValue_setGpioSignalLvl(void)
{
    // clang-format off
    uint8_t pin             = 0;
    uint8_t actual_pinValue = 0;
    int32_t status          = PMIC_ST_SUCCESS;
    uint8_t signalLvl       = PMIC_GPIO_HIGH;
    Pmic_GpioCfg_t nvmGpioCfg;
    Pmic_GpioCfg_t outputGpioCfg = {
        .validParams        = PMIC_GPIO_CFG_DIR_VALID_SHIFT      |
                              PMIC_GPIO_CFG_OD_VALID_SHIFT       |
                              PMIC_GPIO_CFG_PULL_VALID_SHIFT     |
                              PMIC_GPIO_CFG_DEGLITCH_VALID_SHIFT |
                              PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,
        .pinDir             = PMIC_GPIO_OUTPUT,
        .outputSignalType   = PMIC_GPIO_PUSH_PULL_OUTPUT,
        .pullCtrl           = PMIC_GPIO_PULL_DISABLED,
        .deglitchEnable     = PMIC_GPIO_DEGLITCH_ENABLE,
        .pinFunc            = PMIC_TPS6522X_GPIO_PINFUNC_GPIO
    };
    // clang-format on

    // Re-initialize MCU's GPIO pins to be input
    initializeGpioPinHandles(gpioPinHandle, false);
    initializeGpioPins(gpioPinHandle);

    // For every pin...
    for (pin = PMIC_TPS6522X_GPIO1_PIN; pin <= PMIC_TPS6522X_GPIO_PIN_MAX; pin++)
    {
        // Save NVM GPIO configuration settings
        resetGpioCfg_withAllValidParams(&nvmGpioCfg);
        status = Pmic_gpioGetConfiguration(&pmicCoreHandle, pin, &nvmGpioCfg);
        TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);

        // Set pin's GPIO configuration to be output GPIO
        status = Pmic_gpioSetConfiguration(&pmicCoreHandle, pin, outputGpioCfg);
        TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);

        // Set value of GPIO output pin
        status = Pmic_gpioSetValue(&pmicCoreHandle, pin, signalLvl);
        TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);

        // Read actual value of GPIO output pin and compare expected vs. actual
        actual_pinValue = GPIOPinRead(gpioPinHandle[pin - 1].gpioPortBase, gpioPinHandle[pin - 1].gpioPin);
        actual_pinValue >>= gpioPinHandle[pin - 1].gpioShiftVal;
        TEST_ASSERT_EQUAL_UINT8(signalLvl, actual_pinValue);

        // Restore NVM default settings for the GPIO pin
        status = Pmic_gpioSetConfiguration(&pmicCoreHandle, pin, nvmGpioCfg);
        TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);

        // After setting the signal level of all GPIO pins to be high, begin setting the levels to be low
        if ((pin == PMIC_TPS6522X_GPIO_PIN_MAX) && (signalLvl == PMIC_GPIO_HIGH))
        {
            pin = PMIC_TPS6522X_GPIO1_PIN;
            signalLvl = PMIC_GPIO_LOW;
        }
    }
}

/**
 * \brief test Pmic_gpioGetValue: Send a high signal level from MCU to PMIC, then send a low signal level
 *                                 from MCU to PMIC.
 *
 * \note For this test, there must be a total of 6 physical GPIO pin connections between Burton and test MCU
 */
void test_Pmic_gpioGetValue_readGpioSignalLvl(void)
{
    // clang-format off
    uint8_t pin             = 0;
    uint8_t actualPinValue  = 0;
    int32_t status          = PMIC_ST_SUCCESS;
    Pmic_GpioCfg_t nvmGpioCfg;
    Pmic_GpioCfg_t inputGpioCfg = {
        .validParams        = PMIC_GPIO_CFG_DIR_VALID_SHIFT        |
                                PMIC_GPIO_CFG_OD_VALID_SHIFT       |
                                PMIC_GPIO_CFG_PULL_VALID_SHIFT     |
                                PMIC_GPIO_CFG_DEGLITCH_VALID_SHIFT |
                                PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,
        .pinDir             = PMIC_GPIO_INPUT,
        .outputSignalType   = PMIC_GPIO_PUSH_PULL_OUTPUT,
        .pullCtrl           = PMIC_GPIO_PULL_DOWN,
        .deglitchEnable     = PMIC_GPIO_DEGLITCH_ENABLE,
        .pinFunc            = PMIC_TPS6522X_GPIO_PINFUNC_GPIO
    };
    // clang-format on

    // Re-initialize MCU's GPIO pins to be output
    initializeGpioPinHandles(gpioPinHandle, true);
    initializeGpioPins(gpioPinHandle);

    // For each GPIO pin...
    for (pin = PMIC_TPS6522X_GPIO1_PIN; pin <= PMIC_TPS6522X_GPIO_PIN_MAX; pin++)
    {
        // Save PMIC GPIO NVM configuration
        status = Pmic_gpioGetConfiguration(&pmicCoreHandle, pin, &nvmGpioCfg);
        TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);

        // Configure PMIC GPIO pin to be input
        status = Pmic_gpioSetConfiguration(&pmicCoreHandle, pin, inputGpioCfg);
        TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);

        // Write a high signal from MCU pin to PMIC pin
        GPIOPinWrite(
            gpioPinHandle[pin - 1].gpioPortBase, gpioPinHandle[pin - 1].gpioPin, gpioPinHandle[pin - 1].gpioPin);

        // Get actual signal level
        status = Pmic_gpioGetValue(&pmicCoreHandle, pin, &actualPinValue);
        TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);

        // Compare expected vs. actual
        TEST_ASSERT_EQUAL_UINT8(PMIC_GPIO_HIGH, actualPinValue);

        // Write a low signal from MCU pin to PMIC pin
        GPIOPinWrite(gpioPinHandle[pin - 1].gpioPortBase, gpioPinHandle[pin - 1].gpioPin, 0x00);

        // Get actual signal level
        status = Pmic_gpioGetValue(&pmicCoreHandle, pin, &actualPinValue);
        TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);

        // Compare expected vs. actual
        TEST_ASSERT_EQUAL_UINT8(PMIC_GPIO_LOW, actualPinValue);

        // Restore PMIC GPIO NVM configuration for the pin
        status = Pmic_gpioSetConfiguration(&pmicCoreHandle, pin, nvmGpioCfg);
        TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);
    }
}

/**
 * \brief This function is called by Unity when it starts a test
 *
 */
void setUp(void)
{
}

/**
 * \brief This function is called by Unity when it finishes a test
 *
 */
void tearDown(void)
{
}

/**
 * \brief Unity uses this API in all its write, put, or print APIs
 *
 * \param ucData    [IN]    Character to write to the terminal
 *
 */
void unityCharPut(unsigned char ucData)
{
    UARTCharPut(UART0_BASE, ucData);
    if (ucData == '\n')
    {
        UARTCharPut(UART0_BASE, '\r');
    }
}
