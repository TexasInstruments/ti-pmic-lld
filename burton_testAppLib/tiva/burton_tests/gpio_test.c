/**
 * \file gpio_test.c
 * \author John Bui (j-bui1@ti.com)
 * \brief Unity testing application that tests the PMIC driver GPIO set/get APIs
 * \version 1.0
 * \date 2023-10-11
 *
 * \copyright Copyright (c) 2023
 *
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

Pmic_CoreHandle_t *pmicCoreHandle = NULL;
gpioPinHandle_t   *gpioPinHandle = NULL;

int main(void)
{
    /*** Variable declaration/initialization ***/
    // clang-format off
    uartHandle_t vcpHandle;
    i2cHandle_t I2C1Handle;
    static Pmic_CoreHandle_t pmicLocalCoreHandle;
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
    initializeGpioInputPinHandles(&gpioPinHandle);
    initializeGpioPins(gpioPinHandle);

    /*** PMIC setup ***/
    initializePmicCoreHandle(&pmicLocalCoreHandle);
    Pmic_init(&pmicConfigData, &pmicLocalCoreHandle);
    pmicCoreHandle = &pmicLocalCoreHandle;

    /*** Clear the console before printing anything ***/
    clearConsole(&vcpHandle);

    /*** Print welcome message ***/
    UARTStrPut(&vcpHandle, "Running all PMIC GPIO tests...\r\n\r\n");

    /*** Begin unity testing ***/
    UNITY_BEGIN();

    RUN_TEST(test_Pmic_gpioGetConfiguration_forCorrectReads);
    RUN_TEST(test_Pmic_gpioSetConfiguration_resetSetAllGpioCfg);
    RUN_TEST(test_Pmic_gpioGetValue_forCorrectReads);
    RUN_TEST(test_Pmic_gpioSetValue_setGpioSignalLvl);
    RUN_TEST(test_Pmic_gpioSetIntr_configIntOnAllGpio);
    RUN_TEST(test_Pmic_gpioSetIntr_disableIntOnAllGpio);

    /*** Finish unity testing ***/
    return UNITY_END();
}

/**
 * \brief Private helper function to reset a GPIO configuration struct with all valid parameters set
 *
 * \param pGpioCfg  [OUT]   GPIO configuration struct to reset with valid parameters
 *
 */
static void resetGpioCfg_withAllValidParams(Pmic_GpioCfg_t *pGpioCfg)
{
    pGpioCfg->validParams = PMIC_GPIO_CFG_DIR_VALID_SHIFT | PMIC_GPIO_CFG_OD_VALID_SHIFT |
                            PMIC_GPIO_CFG_PULL_VALID_SHIFT | PMIC_GPIO_CFG_DEGLITCH_VALID_SHIFT |
                            PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT;
    pGpioCfg->pinDir = 0;
    pGpioCfg->outputSignalType = 0;
    pGpioCfg->pullCtrl = 0;
    pGpioCfg->deglitchEnable = 0;
    pGpioCfg->pinFunc = 0;
    pGpioCfg->pinPolarity = 0;
}

/**
 * \brief Private helper function to reset a specific GPIO pin's configuration register to NVM default.
 *        Should only be called within certain tests.
 *
 * \param pin   [IN]    GPIO pin to reset
 *
 */
static void resetGpioPinCfgToDefault(const uint8_t pin)
{
    uint8_t status = 0;
    uint8_t actualCfgData = 0;
    uint8_t NVMCfgData[PMIC_TPS6522X_GPIO_PIN_MAX] = {0x20, 0x20, 0x18, 0x1C, 0x60, 0x18};

    TEST_ASSERT_LESS_OR_EQUAL(PMIC_TPS6522X_GPIO_PIN_MAX, pin);

    status = pmicI2CWrite(pmicCoreHandle, PMIC_MAIN_INST, 0x31 + pin - 1, NVMCfgData + pin - 1, 1);
    TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);

    status = pmicI2CRead(pmicCoreHandle, PMIC_MAIN_INST, 0x31 + pin - 1, &actualCfgData, 1);
    TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);

    TEST_ASSERT_EQUAL_UINT8(NVMCfgData[pin - 1], actualCfgData);
}

/**
 * \brief Private helper function to reset all GPIO configuration registers to NVM default.
 *        Should only be called within certain tests
 *
 */
static void resetAllGpioCfgToDefault(void)
{
    uint8_t pin = 0;
    uint8_t status = 0;
    uint8_t actualCfgData = 0;
    uint8_t NVMCfgData[PMIC_TPS6522X_GPIO_PIN_MAX] = {0x20, 0x20, 0x18, 0x1C, 0x60, 0x18};

    for (pin = PMIC_TPS6522X_GPIO1_PIN; pin <= PMIC_TPS6522X_GPIO_PIN_MAX; pin++)
    {
        status = pmicI2CWrite(pmicCoreHandle, PMIC_MAIN_INST, 0x31 + pin - 1, NVMCfgData + pin - 1, 1);
        TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);

        status = pmicI2CRead(pmicCoreHandle, PMIC_MAIN_INST, 0x31 + pin - 1, &actualCfgData, 1);
        TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);

        TEST_ASSERT_EQUAL_UINT8(NVMCfgData[pin - 1], actualCfgData);
    }
}

/**
 * \brief test Pmic_gpioGetConfiguration : Read all NVM default GPIO configurations
 *
 */
void test_Pmic_gpioGetConfiguration_forCorrectReads(void)
{
    Pmic_GpioCfg_t gpioCfg;
    uint8_t        pin = 0;
    int32_t        status = PMIC_ST_SUCCESS;
    uint8_t        actualCfgData = 0;
    uint8_t        expectedCfgData[PMIC_TPS6522X_GPIO_PIN_MAX] = {0x20, 0x20, 0x18, 0x1C, 0x60, 0x18};

    for (pin = PMIC_TPS6522X_GPIO1_PIN; pin <= PMIC_TPS6522X_GPIO_PIN_MAX; pin++)
    {
        resetGpioCfg_withAllValidParams(&gpioCfg);
        status = Pmic_gpioGetConfiguration(pmicCoreHandle, pin, &gpioCfg);
        TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);

        // clang-format off
        actualCfgData = (gpioCfg.pinFunc           << PMIC_GPIOX_CONF_GPIO_SEL_SHIFT)          |
                        (gpioCfg.deglitchEnable    << PMIC_GPIOX_CONF_GPIO_DEGLITCH_EN_SHIFT)  |
                        (gpioCfg.outputSignalType  << PMIC_GPIOX_CONF_GPIO_OD_SHIFT)           |
                        (gpioCfg.pinDir            << PMIC_GPIOX_CONF_GPIO_DIR_SHIFT);
        // clang-format on
        if (gpioCfg.pullCtrl == PMIC_GPIO_PULL_DOWN)
        {
            actualCfgData |= (0b1 << PMIC_GPIOX_CONF_GPIO_PU_PD_EN_SHIFT);
        }
        else if (gpioCfg.pullCtrl == PMIC_GPIO_PULL_UP)
        {
            actualCfgData |= (0b11 << PMIC_GPIOX_CONF_GPIO_PU_SEL_SHIFT);
        }

        TEST_ASSERT_EQUAL_UINT8(expectedCfgData[pin - 1], actualCfgData);
    }
}

/**
 * \brief test Pmic_gpioSetConfiguration : Reset values of all GPIO configuration registers
 *        to zero then set their values to the NVM default configuration
 *
 */
void test_Pmic_gpioSetConfiguration_resetSetAllGpioCfg(void)
{
    uint8_t        pin = 0;
    int8_t         status = PMIC_ST_SUCCESS;
    Pmic_GpioCfg_t expected_gpioCfg, actual_gpioCfg;

    resetGpioCfg_withAllValidParams(&expected_gpioCfg);
    resetGpioCfg_withAllValidParams(&actual_gpioCfg);

    for (pin = PMIC_TPS6522X_GPIO1_PIN; pin <= PMIC_TPS6522X_GPIO_PIN_MAX; pin++)
    {
        status = Pmic_gpioSetConfiguration(pmicCoreHandle, pin, expected_gpioCfg);
        TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);

        status = Pmic_gpioGetConfiguration(pmicCoreHandle, pin, &actual_gpioCfg);
        TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);

        TEST_ASSERT_EQUAL_UINT8(expected_gpioCfg.pinFunc, actual_gpioCfg.pinFunc);
        TEST_ASSERT_EQUAL_UINT8(expected_gpioCfg.deglitchEnable, actual_gpioCfg.deglitchEnable);
        TEST_ASSERT_EQUAL_UINT8(expected_gpioCfg.pullCtrl, actual_gpioCfg.pullCtrl);
        TEST_ASSERT_EQUAL_UINT8(expected_gpioCfg.outputSignalType, actual_gpioCfg.outputSignalType);
        TEST_ASSERT_EQUAL_UINT8(expected_gpioCfg.pinDir, actual_gpioCfg.pinDir);

        resetGpioCfg_withAllValidParams(&actual_gpioCfg);
    }

    resetAllGpioCfgToDefault();
}

/**
 * \brief test Pmic_gpioGetValue : Read all PMIC GPIOs values; test the value received
 *        from driver API against value received from application layer created read API
 *
 */
void test_Pmic_gpioGetValue_forCorrectReads(void)
{
    uint8_t       pin = 0;
    uint8_t       pinValue_fromDriverAPI = 0;
    uint8_t       regValue_fromApplicationAPI = 0;
    uint8_t       pinValue_fromApplicationAPI = 0;
    const uint8_t GPIO_IN_1_REG_ADDR = 0x3F;
    int32_t       status = PMIC_ST_SUCCESS;

    status = pmicI2CRead(pmicCoreHandle, PMIC_MAIN_INST, GPIO_IN_1_REG_ADDR, &regValue_fromApplicationAPI, 1);
    TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);

    for (pin = PMIC_TPS6522X_GPIO1_PIN; pin <= PMIC_TPS6522X_GPIO_PIN_MAX; pin++)
    {
        status = Pmic_gpioGetValue(pmicCoreHandle, pin, &pinValue_fromDriverAPI);
        TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);

        pinValue_fromApplicationAPI = ((regValue_fromApplicationAPI >> (pin - 1)) & 1);
        TEST_ASSERT_EQUAL_UINT8(pinValue_fromApplicationAPI, pinValue_fromDriverAPI);
    }
}

/**
 * \brief test Pmic_gpioSetValue : Set all GPIO output values to one. Afterwards, reset all GPIO output
 *                                 values to zero. In both scenarios, check for expected output values
 *
 * \note For this test, there must be a total of 6 physical GPIO pin connections between Burton and test MCU
 *
 */
void test_Pmic_gpioSetValue_setGpioSignalLvl(void)
{
    // clang-format off
    uint8_t pin             = 0;
    uint8_t actual_pinValue = 0;
    int32_t status          = PMIC_ST_SUCCESS;
    uint8_t signalLvl       = PMIC_GPIO_HIGH;
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

    for (pin = PMIC_TPS6522X_GPIO1_PIN; pin <= PMIC_TPS6522X_GPIO_PIN_MAX; pin++)
    {
        status = Pmic_gpioSetConfiguration(pmicCoreHandle, pin, outputGpioCfg);
        TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);
    }

    for (pin = PMIC_TPS6522X_GPIO1_PIN; pin <= PMIC_TPS6522X_GPIO_PIN_MAX; pin++)
    {
        status = Pmic_gpioSetValue(pmicCoreHandle, pin, signalLvl);
        TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);

        actual_pinValue = GPIOPinRead(gpioPinHandle[pin - 1].gpioPortBase, gpioPinHandle[pin - 1].gpioPin);
        actual_pinValue >>= gpioPinHandle[pin - 1].gpioShiftVal;
        TEST_ASSERT_EQUAL_UINT8(signalLvl, actual_pinValue);

        // Reset all output singal levels for next Unity test
        if ((pin == PMIC_TPS6522X_GPIO_PIN_MAX) && (signalLvl == PMIC_GPIO_HIGH))
        {
            pin = PMIC_TPS6522X_GPIO1_PIN;
            signalLvl = PMIC_GPIO_LOW;
        }
    }

    resetAllGpioCfgToDefault();
}

/**
 * \brief test Pmic_gpioSetIntr :  Configure interrupts for all GPIO pins
 *
 */
void test_Pmic_gpioSetIntr_configIntOnAllGpio(void)
{
    // clang-format off
    uint8_t pin = 0;
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0, actual_gpioIntMask = 0;
    uint8_t actual_maskPol = 0, expected_maskPol = 0;
    uint8_t actual_intrType = 0, expected_intrType = 0;
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

    // Configure all GPIO pins on the PMIC to be output
    for (pin = PMIC_TPS6522X_GPIO1_PIN; pin <= PMIC_TPS6522X_GPIO_PIN_MAX; pin++)
    {
        status = Pmic_gpioSetConfiguration(pmicCoreHandle, pin, outputGpioCfg);
        TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);
    }

    // Set interrupts on all GPIO pins
    expected_maskPol = PMIC_GPIO_POL_LOW;
    expected_intrType = PMIC_GPIO_FALL_INTERRUPT;
    for (pin = PMIC_TPS6522X_GPIO1_PIN; pin <= PMIC_TPS6522X_GPIO_PIN_MAX; pin++)
    {
        status = Pmic_gpioSetIntr(pmicCoreHandle, pin, expected_intrType, expected_maskPol);
        TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);
    }

    // For all pins up to but not including pin 5, get actual GPIO interrupt mask and polarity,
    // compare expected vs. actual
    for (pin = PMIC_TPS6522X_GPIO1_PIN; pin < PMIC_TPS6522X_GPIO5_PIN; pin++)
    {
        // Get actual interrupt mask and mask polarity from the PMIC
        status = pmicI2CRead(pmicCoreHandle, PMIC_MAIN_INST, FSM_TRIG_MASK_1_REG_ADDR, &regData, 1);
        TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);

        // Compare expected vs. actual mask and mask polarity
        actual_gpioIntMask = (regData >> (2 * (pin - 1))) & 1;
        actual_maskPol = (regData >> ((2 * pin) - 1)) & 1;
        TEST_ASSERT_EQUAL_UINT8(GPIO_INT_UNMASKED, actual_gpioIntMask);
        TEST_ASSERT_EQUAL_UINT8(expected_maskPol, actual_maskPol);
    }

    // For remaining pins, get actual GPIO interrupt mask and polarity, compare expected vs. actual
    for (pin = PMIC_TPS6522X_GPIO5_PIN; pin <= PMIC_TPS6522X_GPIO6_PIN; pin++)
    {
        // Get actual interrupt mask and mask polarity from the PMIC
        status = pmicI2CRead(pmicCoreHandle, PMIC_MAIN_INST, FSM_TRIG_MASK_2_REG_ADDR, &regData, 1);
        TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);
        switch (pin)
        {
            case PMIC_TPS6522X_GPIO5_PIN:
                actual_gpioIntMask = regData & 1;
                actual_maskPol = (regData >> 1) & 1;
                break;
            case PMIC_TPS6522X_GPIO6_PIN:
                actual_gpioIntMask = (regData >> 2) & 1;
                actual_maskPol = (regData >> 3) & 1;
                break;
            default:
                break;
        }

        // Compare expected vs. actual mask and mask polarity
        TEST_ASSERT_EQUAL_UINT8(GPIO_INT_UNMASKED, actual_gpioIntMask);
        TEST_ASSERT_EQUAL_UINT8(expected_maskPol, actual_maskPol);
    }

    // For all pins, get actual interrupt type and compare expected vs. actual
    for (pin = PMIC_TPS6522X_GPIO1_PIN; pin <= PMIC_TPS6522X_GPIO_PIN_MAX; pin++)
    {
        // Get actual interrupt type from the PMIC
        status = pmicI2CRead(pmicCoreHandle, PMIC_MAIN_INST, MASK_GPIO_FALL_REG_ADDR, &regData, 1);
        TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);

        // Compare expected vs. actual interrupt type
        if (((regData >> (pin - 1)) & 1) == 0)
        {
            actual_intrType = PMIC_GPIO_FALL_INTERRUPT;
        }
        else
        {
            actual_intrType = 0xFF;
        }
        TEST_ASSERT_EQUAL_UINT8(expected_intrType, actual_intrType);
    }
}

/**
 * \brief test Pmic_gpioSetIntr : Disable interrupts for all GPIO pins
 *
 */
void test_Pmic_gpioSetIntr_disableIntOnAllGpio(void)
{
    uint8_t regData = 0;
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t actual_gpioIntMask = 0;
    uint8_t pin = 0;

    // Disable interrupt on all GPIO pins
    for (pin = PMIC_TPS6522X_GPIO1_PIN; pin <= PMIC_TPS6522X_GPIO_PIN_MAX; pin++)
    {
        status = Pmic_gpioSetIntr(pmicCoreHandle, pin, PMIC_GPIO_DISABLE_INTERRUPT, PMIC_GPIO_POL_LOW);
        TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);
    }

    // For all pins up to but not including pin 5, get actual GPIO interrupt mask and compare expected vs. actual
    for (pin = PMIC_TPS6522X_GPIO1_PIN; pin < PMIC_TPS6522X_GPIO5_PIN; pin++)
    {
        // Get actual interrupt mask value from the PMIC
        status = pmicI2CRead(pmicCoreHandle, PMIC_MAIN_INST, FSM_TRIG_MASK_1_REG_ADDR, &regData, 1);
        TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);

        // Compare expected vs. actual GPIO interrupt mask value
        actual_gpioIntMask = (regData >> (2 * (pin - 1))) & 1;
        TEST_ASSERT_EQUAL_UINT8(GPIO_INT_MASKED, actual_gpioIntMask);
    }

    // For the remaining GPIO pins, get actual GPIO interrupt mask and compare expected vs. actual
    for (pin = PMIC_TPS6522X_GPIO5_PIN; pin <= PMIC_TPS6522X_GPIO6_PIN; pin++)
    {
        // Get actual interrupt mask from the PMIC
        status = pmicI2CRead(pmicCoreHandle, PMIC_MAIN_INST, FSM_TRIG_MASK_2_REG_ADDR, &regData, 1);
        TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);
        switch (pin)
        {
            case PMIC_TPS6522X_GPIO5_PIN:
                actual_gpioIntMask = regData & 1;
                break;
            case PMIC_TPS6522X_GPIO6_PIN:
                actual_gpioIntMask = (regData >> 2) & 1;
                break;
            default:
                break;
        }

        // Compare expected vs. actual interrupt mask value
        TEST_ASSERT_EQUAL_UINT8(GPIO_INT_MASKED, actual_gpioIntMask);
    }

    // For all pins, get actual interrupt type and compare expected vs. actual
    for (pin = PMIC_TPS6522X_GPIO1_PIN; pin <= PMIC_TPS6522X_GPIO_PIN_MAX; pin++)
    {
        // Get actual fall interrupt type from the PMIC
        status = pmicI2CRead(pmicCoreHandle, PMIC_MAIN_INST, MASK_GPIO_FALL_REG_ADDR, &regData, 1);
        TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);

        // compare expected vs. actual interrupt type
        TEST_ASSERT_EQUAL_UINT8(GPIO_FALL_INT_MASKED, ((regData >> (pin - 1)) & 1));

        // Get actual rise interrupt type from the PMIC
        status = pmicI2CRead(pmicCoreHandle, PMIC_MAIN_INST, MASK_GPIO_RISE_REG_ADDR, &regData, 1);
        TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);

        // compare expected vs. actual interrupt type
        TEST_ASSERT_EQUAL_UINT8(GPIO_RISE_INT_MASKED, ((regData >> (pin - 1)) & 1));

        resetGpioPinCfgToDefault(pin);
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
