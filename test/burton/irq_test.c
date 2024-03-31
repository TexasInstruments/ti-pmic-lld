/******************************************************************************
 * Copyright (c) 2023-2024 Texas Instruments Incorporated - http://www.ti.com
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *****************************************************************************/
/**
 * \file irq_test.c
 * \author John Bui (j-bui1@ti.com)
 * \brief Unity testing application that tests the PMIC driver interrupt APIs
 * \version 1.0
 */
/* Standard includes */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

/* Tiva test app library */
#include "tiva_testLib.h"

/* Test specific include */
#include "irq_test.h"

/* Unity testing library */
#include "unity.h"

/* PMIC driver */
#include "pmic.h"

/* Tiva system clock APIs and definitions */
#include "sysctl.h"

#define RUN_IRQ_TESTS()     RUN_TEST(test_GPIO1_interrupt_enableDisable);   \
                            RUN_TEST(test_GPIO2_interrupt_enableDisable);   \
                            RUN_TEST(test_GPIO3_interrupt_enableDisable);   \
                            RUN_TEST(test_GPIO4_interrupt_enableDisable);   \
                            RUN_TEST(test_GPIO5_interrupt_enableDisable);   \
                            RUN_TEST(test_GPIO6_interrupt_enableDisable);   \
                            RUN_TEST(test_GPIO1_interrupt_fallingEdge);     \
                            RUN_TEST(test_GPIO1_interrupt_risingEdge);      \
                            RUN_TEST(test_GPIO2_interrupt_fallingEdge);     \
                            RUN_TEST(test_GPIO2_interrupt_risingEdge);      \
                            RUN_TEST(test_GPIO3_interrupt_fallingEdge);     \
                            RUN_TEST(test_GPIO3_interrupt_risingEdge);      \
                            RUN_TEST(test_GPIO4_interrupt_fallingEdge);     \
                            RUN_TEST(test_GPIO4_interrupt_risingEdge);      \
                            RUN_TEST(test_GPIO5_interrupt_fallingEdge);     \
                            RUN_TEST(test_GPIO5_interrupt_risingEdge);      \
                            RUN_TEST(test_GPIO6_interrupt_fallingEdge);     \
                            RUN_TEST(test_GPIO6_interrupt_risingEdge)      

timerHandle_t tHandle;
Pmic_CoreHandle_t pmicCoreHandle;

/* For setup and teardown of tests */
Pmic_GpioCfg_t nvmGpioCfg[PMIC_TPS6522X_GPIO_PIN_MAX];
bool nvmRiseIntrMaskStat[PMIC_TPS6522X_GPIO_PIN_MAX];
bool nvmFallIntrMaskStat[PMIC_TPS6522X_GPIO_PIN_MAX];

int main(void)
{
    /*** Variable declaration/initialization ***/
    uartHandle_t         vcpHandle;
    i2cHandle_t          I2C1Handle;
    const Pmic_CoreCfg_t pmicConfigData = {
        .validParams =
            PMIC_CFG_DEVICE_TYPE_VALID_SHIFT | PMIC_CFG_COMM_MODE_VALID_SHIFT | PMIC_CFG_SLAVEADDR_VALID_SHIFT |
            PMIC_CFG_QASLAVEADDR_VALID_SHIFT | PMIC_CFG_NVMSLAVEADDR_VALID_SHIFT | PMIC_CFG_COMM_HANDLE_VALID_SHIFT |
            PMIC_CFG_COMM_IO_RD_VALID_SHIFT | PMIC_CFG_COMM_IO_WR_VALID_SHIFT | PMIC_CFG_I2C1_SPEED_VALID_SHIFT,
        .instType = PMIC_MAIN_INST,
        .pmicDeviceType = PMIC_DEV_BURTON_TPS6522X,
        .commMode = PMIC_INTF_SINGLE_I2C,
        .slaveAddr = BURTON_I2C_USER_PAGE_ADDRESS,
        .qaSlaveAddr = BURTON_I2C_WDG_PAGE_ADDRESS,
        .nvmSlaveAddr = BURTON_I2C_NVM_PAGE_ADDRESS,
        .i2c1Speed = PMIC_I2C_STANDARD_MODE,
        .pCommHandle = &I2C1Handle,
        .pQACommHandle = &I2C1Handle,
        .pFnPmicCommIoRead = &pmicI2CRead,
        .pFnPmicCommIoWrite = &pmicI2CWrite};

    /*** System clock setup ***/
    SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                   SYSCTL_XTAL_16MHZ); // 400 / 2 / 4 = 50 MHz clock rate

    /*** Virtual communication port setup ***/
    initializeVCPHandle(&vcpHandle);
    initializeVCP(&vcpHandle);

    /*** I2C setup ***/
    initializeI2C1Handle(&I2C1Handle);
    initializeI2C(&I2C1Handle);

    /*** Timer setup ***/
    initializeTimerHandle(&tHandle);
    initializeTimer(&tHandle);

    /*** PMIC setup ***/
    initializePmicCoreHandle(&pmicCoreHandle);
    Pmic_init(&pmicConfigData, &pmicCoreHandle);

    /*** Clear the console before printing anything ***/
    clearConsole(&vcpHandle);

    /*** Print welcome message ***/
    UARTStrPut(&vcpHandle, "Running all PMIC Interrupt tests...\r\n\r\n");

    /*** Begin unity testing ***/
    UNITY_BEGIN();

    RUN_IRQ_TESTS();

    /*** Finish unity testing ***/
    return UNITY_END();
}

/**
 * \brief This private helper function is used to test the falling edge interrupt on a GPIO
 *
 * \param pin               [IN]        Burton GPIO pin number
 * \param expectedIrqNum    [IN]        Expected flag to be raised during the test
 * \param gpioXIrqMaskNum   [IN]        Used to access the rise and fall interrupt masking
 *                                      configuration registers of GPIO X
 */
static void gpioXIntFallingEdgeTest(const uint8_t pin, const uint8_t expectedIrqNum, const uint8_t gpioIrqMaskNum)
{
    uint8_t          rxBuf = 0;
    uint8_t          iterations = 0;
    uint8_t          actualIrqNum = 0;
    int32_t          status = PMIC_ST_SUCCESS;
    Pmic_GpioCfg_t   outputGpioCfg;
    Pmic_IrqStatus_t errStat;

    // Configure GPIO to be output GPIO w/ deglitching and push-pull operation
    outputGpioCfg.validParams = PMIC_GPIO_CFG_DIR_VALID_SHIFT | PMIC_GPIO_CFG_OD_VALID_SHIFT |
                                PMIC_GPIO_CFG_PULL_VALID_SHIFT | PMIC_GPIO_CFG_DEGLITCH_VALID_SHIFT |
                                PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT;
    outputGpioCfg.pinDir = PMIC_GPIO_OUTPUT;
    outputGpioCfg.outputSignalType = PMIC_GPIO_PUSH_PULL_OUTPUT;
    outputGpioCfg.pullCtrl = PMIC_GPIO_PULL_DISABLED;
    outputGpioCfg.deglitchEnable = PMIC_GPIO_DEGLITCH_ENABLE;
    outputGpioCfg.pinFunc = PMIC_TPS6522X_GPIO_PINFUNC_GPIO;
    status = Pmic_gpioSetConfiguration(&pmicCoreHandle, pin, outputGpioCfg);
    TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);

    // Set GPIO signal level to high
    status = Pmic_gpioSetValue(&pmicCoreHandle, pin, PMIC_GPIO_HIGH);
    TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);

    // Clear GPIO interrupt flag before testing
    status = Pmic_irqClrErrStatus(&pmicCoreHandle, expectedIrqNum);
    TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);

    // Unmask GPIO falling edge interrupt
    status = Pmic_gpioSetIntr(&pmicCoreHandle, pin, PMIC_GPIO_FALL_INTERRUPT, PMIC_GPIO_POL_LOW);
    TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);

    // Set GPIO signal level to low, which should trigger the falling edge interrupt
    status = Pmic_gpioSetValue(&pmicCoreHandle, pin, PMIC_GPIO_LOW);
    TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);

    // Wait a certain time period to allow PMIC to raise flag (.25 seconds)
    delayTimeInMs(&tHandle, 250);

    // Ensure the GPIO interrupt flag is raised
    status = pmicI2CRead(&pmicCoreHandle, PMIC_MAIN_INST, INT_GPIO_REG_ADDR, &rxBuf, 1);
    TEST_ASSERT_EQUAL_INT32(INTERRUPT_RAISED, ((rxBuf >> gpioIrqMaskNum) & 1));

    // Get the current status of all interrupts
    status = Pmic_irqGetErrStatus(&pmicCoreHandle, &errStat, false);
    TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);

    // Find and clear the expected interrupt flag
    while ((actualIrqNum != expectedIrqNum) && (iterations != UINT8_MAX))
    {
        status = Pmic_getNextErrorStatus(&pmicCoreHandle, &errStat, &actualIrqNum);
        TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);

        status = Pmic_irqClrErrStatus(&pmicCoreHandle, actualIrqNum);
        TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);

        iterations++;
    }
    TEST_ASSERT_NOT_EQUAL_UINT8(UINT8_MAX, iterations);

    // Ensure the GPIO interrupt flag is cleared
    status = pmicI2CRead(&pmicCoreHandle, PMIC_MAIN_INST, INT_GPIO_REG_ADDR, &rxBuf, 1);
    TEST_ASSERT_EQUAL_INT32(INTERRUPT_CLEARED, ((rxBuf >> gpioIrqMaskNum) & 1));
}

/**
 * \brief This private helper function is used to test the rising edge interrupt on a GPIO
 *
 * \param pin               [IN]        Burton GPIO pin number
 * \param expectedIrqNum    [IN]        Expected flag to be raised during the test
 * \param gpioXIrqMaskNum   [IN]        Used to access the rise and fall interrupt masking
 *                                      configuration registers of GPIO X
 */
static void gpioXIntRisingEdgeTest(const uint8_t pin, const uint8_t expectedIrqNum, const uint8_t gpioIrqMaskNum)
{
    uint8_t          rxBuf = 0;
    uint8_t          iterations = 0;
    uint8_t          actualIrqNum = 0;
    int32_t          status = PMIC_ST_SUCCESS;
    Pmic_GpioCfg_t   outputGpioCfg;
    Pmic_IrqStatus_t errStat;

    // Configure GPIO to be output GPIO w/ deglitching and push-pull operation
    outputGpioCfg.validParams = PMIC_GPIO_CFG_DIR_VALID_SHIFT | PMIC_GPIO_CFG_OD_VALID_SHIFT |
                                PMIC_GPIO_CFG_PULL_VALID_SHIFT | PMIC_GPIO_CFG_DEGLITCH_VALID_SHIFT |
                                PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT;
    outputGpioCfg.pinDir = PMIC_GPIO_OUTPUT;
    outputGpioCfg.outputSignalType = PMIC_GPIO_PUSH_PULL_OUTPUT;
    outputGpioCfg.pullCtrl = PMIC_GPIO_PULL_DISABLED;
    outputGpioCfg.deglitchEnable = PMIC_GPIO_DEGLITCH_ENABLE;
    outputGpioCfg.pinFunc = PMIC_TPS6522X_GPIO_PINFUNC_GPIO;
    status = Pmic_gpioSetConfiguration(&pmicCoreHandle, pin, outputGpioCfg);
    TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);

    // Set GPIO signal level to low
    status = Pmic_gpioSetValue(&pmicCoreHandle, pin, PMIC_GPIO_LOW);
    TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);

    // Reset GPIO masking configuration
    status = Pmic_gpioSetIntr(&pmicCoreHandle, pin, PMIC_GPIO_DISABLE_INTERRUPT, PMIC_GPIO_POL_LOW);
    TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);

    // Clear GPIO interrupt flag before testing
    status = Pmic_irqClrErrStatus(&pmicCoreHandle, expectedIrqNum);
    TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);

    // Unmask GPIO rising edge interrupt
    status = Pmic_gpioSetIntr(&pmicCoreHandle, pin, PMIC_GPIO_RISE_INTERRUPT, PMIC_GPIO_POL_LOW);
    TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);

    // Set GPIO signal level to high, which should trigger the rising edge interrupt
    status = Pmic_gpioSetValue(&pmicCoreHandle, pin, PMIC_GPIO_HIGH);
    TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);

    // Wait a certain time period to allow PMIC to raise flag (.25 seconds)
    delayTimeInMs(&tHandle, 250);

    // Ensure the GPIO interrupt flag is raised
    status = pmicI2CRead(&pmicCoreHandle, PMIC_MAIN_INST, INT_GPIO_REG_ADDR, &rxBuf, 1);
    TEST_ASSERT_EQUAL_INT32(INTERRUPT_RAISED, ((rxBuf >> gpioIrqMaskNum) & 1));

    // Get the current status of all interrupts
    status = Pmic_irqGetErrStatus(&pmicCoreHandle, &errStat, false);
    TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);

    // Find and clear the expected interrupt flag
    while ((actualIrqNum != expectedIrqNum) && (iterations != UINT8_MAX))
    {
        status = Pmic_getNextErrorStatus(&pmicCoreHandle, &errStat, &actualIrqNum);
        TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);

        status = Pmic_irqClrErrStatus(&pmicCoreHandle, actualIrqNum);
        TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);

        iterations++;
    }
    TEST_ASSERT_NOT_EQUAL_UINT8(UINT8_MAX, iterations);

    // Ensure the GPIO interrupt flag is cleared
    status = pmicI2CRead(&pmicCoreHandle, PMIC_MAIN_INST, INT_GPIO_REG_ADDR, &rxBuf, 1);
    TEST_ASSERT_EQUAL_INT32(INTERRUPT_CLEARED, ((rxBuf >> gpioIrqMaskNum) & 1));
}

/**
 * \brief This private helper function is used to test the enable/disable interrupt functionality on a GPIO pin.
 *
 * \param pin                       [IN]        Burton GPIO pin number
 * \param gpioXIrqMaskNum           [IN]        Used to access the rise and fall interrupt masking
 *                                              configuration registers of GPIO X
 */
static void gpioXIntEnableDisableTest(const uint8_t pin, const uint8_t gpioXIrqMaskNum)
{
    int32_t status = PMIC_ST_SUCCESS;
    bool    actualGpioXRiseIntMask = false;
    bool    actualGpioXFallIntMask = false;

    // Unmask GPIO interrupts
    status = Pmic_gpioSetIntr(&pmicCoreHandle, pin, PMIC_GPIO_FALL_RISE_INTERRUPT, PMIC_GPIO_POL_LOW);
    TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);

    // Get GPIO interrupt masks
    status = Pmic_irqGetGpioMaskIntr(&pmicCoreHandle,
                                     gpioXIrqMaskNum,
                                     PMIC_IRQ_GPIO_RISE_FALL_INT_TYPE,
                                     &actualGpioXRiseIntMask,
                                     &actualGpioXFallIntMask);
    TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);

    // Compare expected vs. actual
    TEST_ASSERT_EQUAL(PMIC_IRQ_UNMASK, actualGpioXRiseIntMask);
    TEST_ASSERT_EQUAL(PMIC_IRQ_UNMASK, actualGpioXFallIntMask);

    // Mask GPIO interrupts
    status = Pmic_gpioSetIntr(&pmicCoreHandle, pin, PMIC_GPIO_DISABLE_INTERRUPT, PMIC_GPIO_POL_LOW);
    TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);

    // Get GPIO interrupt masks
    status = Pmic_irqGetGpioMaskIntr(&pmicCoreHandle,
                                     gpioXIrqMaskNum,
                                     PMIC_IRQ_GPIO_RISE_FALL_INT_TYPE,
                                     &actualGpioXRiseIntMask,
                                     &actualGpioXFallIntMask);
    TEST_ASSERT_EQUAL_INT32(PMIC_ST_SUCCESS, status);

    // Compare expected vs. actual
    TEST_ASSERT_EQUAL(PMIC_IRQ_MASK, actualGpioXRiseIntMask);
    TEST_ASSERT_EQUAL(PMIC_IRQ_MASK, actualGpioXFallIntMask);
}

/**
 * \brief test GPIO1 Falling Edge Interrupt: GPIO 1 falling edge interrupt test
 */
void test_GPIO1_interrupt_fallingEdge(void)
{
    const uint8_t pin = PMIC_TPS6522X_GPIO1_PIN;
    const uint8_t expectedIrqNum = PMIC_TPS6522X_GPIO1_INT;
    const uint8_t gpioIrqMaskNum = PMIC_TPS6522X_IRQ_GPIO_1_INT_MASK_NUM;

    gpioXIntFallingEdgeTest(pin, expectedIrqNum, gpioIrqMaskNum);
}

/**
 * \brief test GPIO1 Rising Edge Interrupt: GPIO 1 rising edge interrupt test
 */
void test_GPIO1_interrupt_risingEdge(void)
{
    const uint8_t pin = PMIC_TPS6522X_GPIO1_PIN;
    const uint8_t expectedIrqNum = PMIC_TPS6522X_GPIO1_INT;
    const uint8_t gpioIrqMaskNum = PMIC_TPS6522X_IRQ_GPIO_1_INT_MASK_NUM;

    gpioXIntRisingEdgeTest(pin, expectedIrqNum, gpioIrqMaskNum);
}

/**
 * \brief test GPIO2 Falling Edge Interrupt: GPIO 2 falling edge interrupt test
 */
void test_GPIO2_interrupt_fallingEdge(void)
{
    const uint8_t pin = PMIC_TPS6522X_GPIO2_PIN;
    const uint8_t expectedIrqNum = PMIC_TPS6522X_GPIO2_INT;
    const uint8_t gpioIrqMaskNum = PMIC_TPS6522X_IRQ_GPIO_2_INT_MASK_NUM;

    gpioXIntFallingEdgeTest(pin, expectedIrqNum, gpioIrqMaskNum);
}

/**
 * \brief test GPIO2 Rising Edge Interrupt: GPIO 2 rising edge interrupt test
 */
void test_GPIO2_interrupt_risingEdge(void)
{
    const uint8_t pin = PMIC_TPS6522X_GPIO2_PIN;
    const uint8_t expectedIrqNum = PMIC_TPS6522X_GPIO2_INT;
    const uint8_t gpioIrqMaskNum = PMIC_TPS6522X_IRQ_GPIO_2_INT_MASK_NUM;

    gpioXIntRisingEdgeTest(pin, expectedIrqNum, gpioIrqMaskNum);
}

/**
 * \brief test GPIO3 Falling Edge Interrupt: GPIO 3 falling edge interrupt test
 */
void test_GPIO3_interrupt_fallingEdge(void)
{
    const uint8_t pin = PMIC_TPS6522X_GPIO3_PIN;
    const uint8_t expectedIrqNum = PMIC_TPS6522X_GPIO3_INT;
    const uint8_t gpioIrqMaskNum = PMIC_TPS6522X_IRQ_GPIO_3_INT_MASK_NUM;

    gpioXIntFallingEdgeTest(pin, expectedIrqNum, gpioIrqMaskNum);
}

/**
 * \brief test GPIO3 Rising Edge Interrupt: GPIO 3 rising edge interrupt test
 */
void test_GPIO3_interrupt_risingEdge(void)
{
    const uint8_t pin = PMIC_TPS6522X_GPIO3_PIN;
    const uint8_t expectedIrqNum = PMIC_TPS6522X_GPIO3_INT;
    const uint8_t gpioIrqMaskNum = PMIC_TPS6522X_IRQ_GPIO_3_INT_MASK_NUM;

    gpioXIntRisingEdgeTest(pin, expectedIrqNum, gpioIrqMaskNum);
}

/**
 * \brief test GPIO4 Falling Edge Interrupt: GPIO 4 falling edge interrupt test
 */
void test_GPIO4_interrupt_fallingEdge(void)
{
    const uint8_t pin = PMIC_TPS6522X_GPIO4_PIN;
    const uint8_t expectedIrqNum = PMIC_TPS6522X_GPIO4_INT;
    const uint8_t gpioIrqMaskNum = PMIC_TPS6522X_IRQ_GPIO_4_INT_MASK_NUM;

    gpioXIntFallingEdgeTest(pin, expectedIrqNum, gpioIrqMaskNum);
}

/**
 * \brief test GPIO4 Rising Edge Interrupt: GPIO 4 rising edge interrupt test
 */
void test_GPIO4_interrupt_risingEdge(void)
{
    const uint8_t pin = PMIC_TPS6522X_GPIO4_PIN;
    const uint8_t expectedIrqNum = PMIC_TPS6522X_GPIO4_INT;
    const uint8_t gpioIrqMaskNum = PMIC_TPS6522X_IRQ_GPIO_4_INT_MASK_NUM;

    gpioXIntRisingEdgeTest(pin, expectedIrqNum, gpioIrqMaskNum);
}

/**
 * \brief test GPIO5 Falling Edge Interrupt: GPIO 5 falling edge interrupt test
 */
void test_GPIO5_interrupt_fallingEdge(void)
{
    const uint8_t pin = PMIC_TPS6522X_GPIO5_PIN;
    const uint8_t expectedIrqNum = PMIC_TPS6522X_GPIO5_INT;
    const uint8_t gpioIrqMaskNum = PMIC_TPS6522X_IRQ_GPIO_5_INT_MASK_NUM;

    gpioXIntFallingEdgeTest(pin, expectedIrqNum, gpioIrqMaskNum);
}

/**
 * \brief test GPIO5 Rising Edge Interrupt: GPIO 5 rising edge interrupt test
 */
void test_GPIO5_interrupt_risingEdge(void)
{
    const uint8_t pin = PMIC_TPS6522X_GPIO5_PIN;
    const uint8_t expectedIrqNum = PMIC_TPS6522X_GPIO5_INT;
    const uint8_t gpioIrqMaskNum = PMIC_TPS6522X_IRQ_GPIO_5_INT_MASK_NUM;

    gpioXIntRisingEdgeTest(pin, expectedIrqNum, gpioIrqMaskNum);
}

/**
 * \brief test GPIO6 Falling Edge Interrupt: GPIO 6 falling edge interrupt test
 */
void test_GPIO6_interrupt_fallingEdge(void)
{
    const uint8_t pin = PMIC_TPS6522X_GPIO6_PIN;
    const uint8_t expectedIrqNum = PMIC_TPS6522X_GPIO6_INT;
    const uint8_t gpioIrqMaskNum = PMIC_TPS6522X_IRQ_GPIO_6_INT_MASK_NUM;

    gpioXIntFallingEdgeTest(pin, expectedIrqNum, gpioIrqMaskNum);
}

/**
 * \brief test GPIO6 Rising Edge Interrupt: GPIO 6 rising edge interrupt test
 */
void test_GPIO6_interrupt_risingEdge(void)
{
    const uint8_t pin = PMIC_TPS6522X_GPIO6_PIN;
    const uint8_t expectedIrqNum = PMIC_TPS6522X_GPIO6_INT;
    const uint8_t gpioIrqMaskNum = PMIC_TPS6522X_IRQ_GPIO_6_INT_MASK_NUM;

    gpioXIntRisingEdgeTest(pin, expectedIrqNum, gpioIrqMaskNum);
}

/**
 * \brief test GPIO1 Interrupt Configuration: Enable and disable (that is to say, unmask and mask)
 *                                             interrupt on GPIO 1
 */
void test_GPIO1_interrupt_enableDisable(void)
{
    const uint8_t pin = PMIC_TPS6522X_GPIO1_PIN;
    const uint8_t gpioIrqMaskNum = PMIC_TPS6522X_IRQ_GPIO_1_INT_MASK_NUM;

    gpioXIntEnableDisableTest(pin, gpioIrqMaskNum);
}

/**
 * \brief test GPIO2 Interrupt Configuration: Enable and disable (that is to say, unmask and mask)
 *                                             interrupt on GPIO 2
 */
void test_GPIO2_interrupt_enableDisable(void)
{
    const uint8_t pin = PMIC_TPS6522X_GPIO2_PIN;
    const uint8_t gpioIrqMaskNum = PMIC_TPS6522X_IRQ_GPIO_2_INT_MASK_NUM;

    gpioXIntEnableDisableTest(pin, gpioIrqMaskNum);
}

/**
 * \brief test GPIO3 Interrupt Configuration: Enable and disable (that is to say, unmask and mask)
 *                                             interrupt on GPIO 3
 */
void test_GPIO3_interrupt_enableDisable(void)
{
    const uint8_t pin = PMIC_TPS6522X_GPIO3_PIN;
    const uint8_t gpioIrqMaskNum = PMIC_TPS6522X_IRQ_GPIO_3_INT_MASK_NUM;

    gpioXIntEnableDisableTest(pin, gpioIrqMaskNum);
}

/**
 * \brief test GPIO4 Interrupt Configuration: Enable and disable (that is to say, unmask and mask)
 *                                             interrupt on GPIO 4
 */
void test_GPIO4_interrupt_enableDisable(void)
{
    const uint8_t pin = PMIC_TPS6522X_GPIO4_PIN;
    const uint8_t gpioIrqMaskNum = PMIC_TPS6522X_IRQ_GPIO_4_INT_MASK_NUM;

    gpioXIntEnableDisableTest(pin, gpioIrqMaskNum);
}

/**
 * \brief test GPIO5 Interrupt Configuration: Enable and disable (that is to say, unmask and mask)
 *                                             interrupt on GPIO 5
 */
void test_GPIO5_interrupt_enableDisable(void)
{
    const uint8_t pin = PMIC_TPS6522X_GPIO5_PIN;
    const uint8_t gpioIrqMaskNum = PMIC_TPS6522X_IRQ_GPIO_5_INT_MASK_NUM;

    gpioXIntEnableDisableTest(pin, gpioIrqMaskNum);
}

/**
 * \brief test GPIO6 Interrupt Configuration: Enable and disable (that is to say, unmask and mask)
 *                                             interrupt on GPIO 6
 */
void test_GPIO6_interrupt_enableDisable(void)
{
    const uint8_t pin = PMIC_TPS6522X_GPIO6_PIN;
    const uint8_t gpioIrqMaskNum = PMIC_TPS6522X_IRQ_GPIO_6_INT_MASK_NUM;

    gpioXIntEnableDisableTest(pin, gpioIrqMaskNum);
}

/**
 * \brief This function is called by Unity when it starts a test
 */
void setUp(void)
{
    uint8_t pin = 0;

    // Save NVM default GPIO configurations
    for (pin = PMIC_TPS6522X_GPIO1_PIN; pin <= PMIC_TPS6522X_GPIO_PIN_MAX; pin++)
    {
        resetGpioCfg_withAllValidParams(&nvmGpioCfg[pin - 1]);
        (void)Pmic_gpioGetConfiguration(&pmicCoreHandle, pin, &nvmGpioCfg[pin - 1]);
    }

    // Save NVM default GPIO interrupt masking configurations
    for (pin = PMIC_TPS6522X_GPIO1_PIN; pin <= PMIC_TPS6522X_GPIO_PIN_MAX; pin++)
    {
        (void)Pmic_irqGetGpioMaskIntr(&pmicCoreHandle,
                                      (PMIC_TPS6522X_IRQ_GPIO_1_INT_MASK_NUM + (pin - 1)),
                                      PMIC_IRQ_GPIO_RISE_FALL_INT_TYPE,
                                      &nvmRiseIntrMaskStat[pin - 1],
                                      &nvmFallIntrMaskStat[pin - 1]);
    }

    // Reset GPIO masking configurations
    for (pin = PMIC_TPS6522X_GPIO1_PIN; pin <= PMIC_TPS6522X_GPIO_PIN_MAX; pin++)
    {
        (void)Pmic_gpioSetIntr(&pmicCoreHandle, pin, PMIC_GPIO_DISABLE_INTERRUPT, PMIC_GPIO_POL_LOW);
    }
}

/**
 * \brief This function is called by Unity when it finishes a test
 */
void tearDown(void)
{
    uint8_t pin = 0;

    // Restore NVM default GPIO configurations
    for (pin = PMIC_TPS6522X_GPIO1_PIN; pin <= PMIC_TPS6522X_GPIO_PIN_MAX; pin++)
    {
        (void)Pmic_gpioSetConfiguration(&pmicCoreHandle, pin, nvmGpioCfg[pin - 1]);
    }

    // Restore NVM default GPIO interrupt masking configurations
    for (pin = PMIC_TPS6522X_GPIO1_PIN; pin <= PMIC_TPS6522X_GPIO_PIN_MAX; pin++)
    {
        (void)Pmic_gpioSetIntr(&pmicCoreHandle, pin, PMIC_GPIO_DISABLE_INTERRUPT, PMIC_GPIO_POL_LOW);

        if ((nvmRiseIntrMaskStat[pin - 1] == PMIC_IRQ_MASK) && (nvmFallIntrMaskStat[pin - 1] == PMIC_IRQ_UNMASK))
        {
            (void)Pmic_gpioSetIntr(&pmicCoreHandle, pin, PMIC_GPIO_FALL_INTERRUPT, PMIC_GPIO_POL_LOW);
        }
        else if ((nvmRiseIntrMaskStat[pin - 1] == PMIC_IRQ_UNMASK) && (nvmFallIntrMaskStat[pin - 1] == PMIC_IRQ_MASK))
        {
            (void)Pmic_gpioSetIntr(&pmicCoreHandle, pin, PMIC_GPIO_RISE_INTERRUPT, PMIC_GPIO_POL_LOW);
        }
        else if ((nvmRiseIntrMaskStat[pin - 1] == PMIC_IRQ_UNMASK) && (nvmFallIntrMaskStat[pin - 1] == PMIC_IRQ_UNMASK))
        {
            (void)Pmic_gpioSetIntr(&pmicCoreHandle, pin, PMIC_GPIO_FALL_RISE_INTERRUPT, PMIC_GPIO_POL_LOW);
        }
    }
}
