/**
 * \file serial_comm_test.c
 * \author John Bui (j-bui1@ti.com)
 * \brief Test application that tests PMIC driver R/W APIs and the PMIC handle R/W APIs
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
#include "burton_testAppLib/tiva/tiva_testLib.h"

/* Test specific include */
#include "burton_testAppLib/tiva/burton_tests/serial_comm_test.h"

/* Unity testing library */
#include "unity/unity.h"

/* PMIC driver */
#include "pmic_drv/pmic.h"

/* Tiva system clock APIs and definitions */
#include "driverlib/sysctl.h"

const uint8_t burtonUserOTPRegAddr[BURTON_NUM_USER_OTP_REGS] = {
    0x0001, 0x0002, 0x0003, 0x0004, 0x0005, 0x0006, 0x0007, 0x0008, 0x0009, 0x000a, 0x000b, 0x000e,
    0x0010, 0x0012, 0x0014, 0x0018, 0x0019, 0x001a, 0x001b, 0x001d, 0x001e, 0x001f, 0x0023, 0x0024,
    0x0025, 0x0027, 0x0028, 0x0029, 0x002b, 0x002c, 0x002d, 0x002e, 0x002f, 0x0030, 0x0031, 0x0032,
    0x0033, 0x0034, 0x0035, 0x0036, 0x003c, 0x003d, 0x0041, 0x0042, 0x0043, 0x0044, 0x0045, 0x0046,
    0x0047, 0x0049, 0x004c, 0x004f, 0x0050, 0x0052, 0x0053, 0x0054, 0x0056, 0x0059, 0x007c, 0x007d,
    0x007e, 0x0084, 0x0087, 0x0088, 0x008b, 0x008e, 0x00a7, 0x00c3, 0x00cd, 0x00ce, 0x00cf};

const uint8_t burtonUserOTPRegVal[BURTON_NUM_USER_OTP_REGS] = {
    0x24, 0x30, 0x00, 0x22, 0x01, 0x22, 0x01, 0x22, 0x01, 0x22, 0x01, 0x2d, 0x0a, 0x27, 0x18, 0x01, 0x01, 0x01,
    0x01, 0x20, 0x20, 0x20, 0x30, 0xec, 0x0a, 0x01, 0x02, 0x01, 0x00, 0x02, 0x02, 0xfd, 0x01, 0x36, 0x20, 0x20,
    0x18, 0x1c, 0x60, 0x18, 0x80, 0x00, 0x55, 0x64, 0xa0, 0x1e, 0x01, 0x05, 0x01, 0x00, 0x40, 0x3f, 0x3f, 0x36,
    0x72, 0x04, 0x00, 0x00, 0x01, 0xc3, 0x00, 0x0f, 0x00, 0x00, 0x0a, 0x00, 0x00, 0x60, 0xd5, 0x00, 0x09};

Pmic_CoreHandle_t *pmicCoreHandle = NULL;

int main(void)
{
    /*** Variable declaration/initialization ***/
    uartHandle_t         vcpHandle;
    i2cHandle_t          I2C1Handle;
    Pmic_CoreHandle_t    pmicLocalCoreHandle;
    const Pmic_CoreCfg_t pmicConfigData = {
        .validParams =
            PMIC_CFG_DEVICE_TYPE_VALID_SHIFT | PMIC_CFG_COMM_MODE_VALID_SHIFT | PMIC_CFG_SLAVEADDR_VALID_SHIFT |
            PMIC_CFG_QASLAVEADDR_VALID_SHIFT | PMIC_CFG_NVMSLAVEADDR_VALID_SHIFT | PMIC_CFG_COMM_HANDLE_VALID_SHIFT |
            PMIC_CFG_COMM_IO_RD_VALID_SHIFT | PMIC_CFG_COMM_IO_WR_VALID_SHIFT | PMIC_CFG_I2C1_SPEED_VALID_SHIFT,
        .instType = PMIC_MAIN_INST,
        .pmicDeviceType = PMIC_DEV_LEO_TPS6594X, // Must change later
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

    /*** PMIC setup ***/
    initializePmicCoreHandle(&pmicLocalCoreHandle);
    Pmic_init(&pmicConfigData, &pmicLocalCoreHandle);
    pmicCoreHandle = &pmicLocalCoreHandle;

    /*** Clear the console before printing anything ***/
    clearConsole(&vcpHandle);

    /*** Print welcome message ***/
    UARTStrPut(&vcpHandle, "Running all PMIC read/write tests...\r\n\r\n");

    /*** Begin unity testing ***/
    UNITY_BEGIN();

    /*** Testing PMIC handle's R/W APIs ***/
    RUN_TEST(test_pmicCoreHandle_PmicCommIoRead_forProperOperation);
    RUN_TEST(test_pmicCoreHandle_PmicCommIoWrite_forProperOperation);

    /*** Testing PMIC driver's R/W APIs ***/
    RUN_TEST(test_Pmic_commIntf_recvByte_forProperOperation);
    RUN_TEST(test_Pmic_commIntf_sendByte_forProperOperation);

    /*** Finish unity testing ***/
    return UNITY_END();
}

/**
 * \brief Unity test to check if the driver's Receive Byte API is
 *        able to read correct values from Burton OTP-programmed user registers.
 *
 */
void test_Pmic_commIntf_recvByte_forProperOperation(void)
{
    uint8_t i = 0;
    uint8_t rxBuf = 0;
    int32_t status = PMIC_ST_SUCCESS;

    for (i = 0; i < BURTON_NUM_USER_OTP_REGS; i++)
    {
        status = Pmic_commIntf_recvByte(pmicCoreHandle, burtonUserOTPRegAddr[i], &rxBuf);
        TEST_ASSERT_EQUAL_UINT32(PMIC_ST_SUCCESS, status);

        TEST_ASSERT_EQUAL_UINT8(burtonUserOTPRegVal[i], rxBuf);
    }
}

/**
 * \brief Unity test to check if the PMIC handle's read function is
 *        able to read correct values from Burton OTP-programmed user registers
 *
 */
void test_pmicCoreHandle_PmicCommIoRead_forProperOperation(void)
{
    uint8_t rxBuf[11] = {0};
    int32_t status = PMIC_ST_SUCCESS;

    status = pmicCoreHandle->pFnPmicCommIoRead(pmicCoreHandle, PMIC_MAIN_INST, 0x1, rxBuf, 11);
    TEST_ASSERT_EQUAL_UINT32(PMIC_ST_SUCCESS, status);

    TEST_ASSERT_EQUAL_UINT8_ARRAY(burtonUserOTPRegVal, rxBuf, 11);
}

/**
 * \brief Unity test to check if the driver's send byte API is
 *        able to write the correct values to Burton OTP-programmed user registers.
 *
 */
void test_Pmic_commIntf_sendByte_forProperOperation(void)
{
    uint8_t  i = 0;
    uint8_t  actualVal[4] = {0x0};
    uint32_t status = PMIC_ST_SUCCESS;
    uint8_t  expectedVal[4] = {0xA, 0xB, 0xC, 0xD}; // Arbitrary values

    for (i = 0; i < 4; i++)
    {
        status = Pmic_commIntf_sendByte(pmicCoreHandle, 0xC9 + i, expectedVal[i]);
        TEST_ASSERT_EQUAL_UINT32(PMIC_ST_SUCCESS, status);

        status = Pmic_commIntf_recvByte(pmicCoreHandle, 0xC9 + i, actualVal[i]);
        TEST_ASSERT_EQUAL_UINT32(PMIC_ST_SUCCESS, status);

        TEST_ASSERT_EQUAL_UINT8(expectedVal[i], actualVal[i]);
    }
}

/**
 * \brief Unity test to check if the PMIC handle's write function is
 *        able to write the correct values to Burton OTP-programmed user registers.
 *
 */
void test_pmicCoreHandle_PmicCommIoWrite_forProperOperation(void)
{
    uint8_t  actualVal[4] = {0x0};
    uint32_t status = PMIC_ST_SUCCESS;
    uint8_t  expectedVal[4] = {0xFF, 0xEE, 0xDD, 0xCC}; // Arbitrary values

    status = pmicCoreHandle->pFnPmicCommIoWrite(pmicCoreHandle, PMIC_MAIN_INST, 0xC9, expectedVal, 4);
    TEST_ASSERT_EQUAL_UINT32(PMIC_ST_SUCCESS, status);

    status = pmicCoreHandle->pFnPmicCommIoRead(pmicCoreHandle, PMIC_MAIN_INST, 0xC9, actualVal, 4);
    TEST_ASSERT_EQUAL_UINT32(PMIC_ST_SUCCESS, status);

    TEST_ASSERT_EQUAL_UINT8_ARRAY(expectedVal, actualVal, 4);
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
 */
void unityCharPut(unsigned char ucData)
{
    UARTCharPut(UART0_BASE, ucData);
    if (ucData == '\n')
    {
        UARTCharPut(UART0_BASE, '\r');
    }
}
