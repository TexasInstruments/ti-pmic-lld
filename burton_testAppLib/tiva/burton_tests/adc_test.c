/**
 * \file adc_test.c
 * \author John Bui (j-bui1@ti.com)
 * \brief Unity testing application that tests the PMIC driver ADC APIs
 * \version 1.0
 * \date 2023-11-13
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
#include "pmic_drv/burton_testAppLib/tiva/burton_tests/adc_test.h"

/* Unity testing library */
#include "unity/unity.h"

/* PMIC driver */
#include "pmic_drv/pmic.h"

Pmic_CoreHandle_t pmicCoreHandle;
timerHandle_t     timerHandle;

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
    (void)Pmic_irqClrErrStatus(&pmicCoreHandle, PMIC_IRQ_ALL);

    /*** Ensure changes are propagated by waiting a certain period of time ***/
    delayTimeInMs(&timerHandle, 1000);

    /*** Print welcome message ***/
    UARTStrPut(&vcpHandle, "Running all PMIC ADC tests...\r\n\r\n");

    /*** Begin unity testing ***/
    UNITY_BEGIN();

    /*** Finish unity testing ***/
    return UNITY_END();
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
 *  \brief  Pmic_gpioPinTypeADC: Test API error handling for when PMIC handle is NULL
 */
void test_ADC_gpioPinTypeADC_nullPmicHandle(void)
{
    int32_t status = PMIC_ST_SUCCESS;

    // Pass in NULL PMIC handle into the API
    status = Pmic_gpioPinTypeADC(NULL, PMIC_TPS6522X_GPIO4_PIN);

    // Compare expected vs. actual return code
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

/**
 *  \brief  Pmic_gpioPinTypeADC: Test API error handling for when GPIO number is
 *                               unsupported
 */
void test_ADC_gpioPinTypeADC_invalidGpioNum(void)
{
    uint8_t gpioNum = 0;
    int32_t status = PMIC_ST_SUCCESS;

    // Pass invalid GPIO numbers into the API
    for (gpioNum = 0; gpioNum < 20; gpioNum++)
    {
        status = Pmic_gpioPinTypeADC(&pmicCoreHandle, gpioNum);
        if ((gpioNum == PMIC_TPS6522X_GPIO4_PIN) || (gpioNum == PMIC_TPS6522X_GPIO5_PIN))
        {
            continue;
        }
        else
        {
            TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);
        }
    }
}

static void adcGpioPinTypeADC_test(const uint8_t gpioNum)
{
    int32_t        status = PMIC_ST_SUCCESS;
    uint8_t        expectedPinFunc = 0;
    Pmic_GpioCfg_t gpioCfg;

    // Initialze gpioCfg for reading GPIO pin functionality
    resetGpioCfg_withSpecificValidParams(&gpioCfg, PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT);

    // Configure GPIO pin to ADC functionality
    status = Pmic_gpioPinTypeADC(&pmicCoreHandle, gpioNum);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get actual GPIO pin configuration
    status = Pmic_gpioGetConfiguration(&pmicCoreHandle, gpioNum, &gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Compare expected vs. actual pin functionality
    expectedPinFunc = ((gpioNum == PMIC_TPS6522X_GPIO4_PIN) ? PMIC_TPS6522X_GPIO_PINFUNC_GPIO4_ADC_IN :
                                                              PMIC_TPS6522X_GPIO_PINFUNC_GPIO5_ADC_IN);
    TEST_ASSERT_EQUAL(expectedPinFunc, gpioCfg.pinFunc);
}

/**
 *  \brief  Pmic_gpioPinTypeADC: Test whether API can configure GPIO 4 to ADC functionality
 */
void test_ADC_gpioPinTypeADC_gpio4(void)
{
    adcGpioPinTypeADC_test(PMIC_TPS6522X_GPIO4_PIN);
}

/**
 *  \brief  Pmic_gpioPinTypeADC: Test whether API can configure GPIO 5 to ADC functionality
 */
void test_ADC_gpioPinTypeADC_gpio5(void)
{
    adcGpioPinTypeADC_test(PMIC_TPS6522X_GPIO5_PIN);
}

static void resetAdcCfg_withAllValidParams(Pmic_adcCfg_t *pAdcCfg)
{
    pAdcCfg->validParams =
        PMIC_ADC_CFG_RDIV_EN_VALID_SHIFT | PMIC_ADC_CFG_THERMAL_SEL_VALID_SHIFT | PMIC_ADC_CFG_CONT_CONV_VALID_SHIFT;
    pAdcCfg->rDivEn = PMIC_ADC_RESISTOR_DIVIDER_DISABLED;
    pAdcCfg->thermalSel = PMIC_ADC_THERMAL_SEL_ADC_INPUT;
    pAdcCfg->contConv = PMIC_ADC_CONTINUOUS_CONVERSION_DISABLED;
}

static void resetAdcCfg_withNoValidParams(Pmic_adcCfg_t *pAdcCfg)
{
    pAdcCfg->validParams = 0;
    pAdcCfg->rDivEn = PMIC_ADC_RESISTOR_DIVIDER_DISABLED;
    pAdcCfg->thermalSel = PMIC_ADC_THERMAL_SEL_ADC_INPUT;
    pAdcCfg->contConv = PMIC_ADC_CONTINUOUS_CONVERSION_DISABLED;
}

static void resetAdcCfg_withSpecificValidParams(Pmic_adcCfg_t *pAdcCfg, uint8_t validParams)
{
    pAdcCfg->validParams = validParams;
    pAdcCfg->rDivEn = PMIC_ADC_RESISTOR_DIVIDER_DISABLED;
    pAdcCfg->thermalSel = PMIC_ADC_THERMAL_SEL_ADC_INPUT;
    pAdcCfg->contConv = PMIC_ADC_CONTINUOUS_CONVERSION_DISABLED;
}

/**
 *  \brief  Pmic_ADCGetConfiguration: Test API error handling for when PMIC handle is NULL
 */
void test_ADC_getConfiguration_nullPmicHandle(void)
{
    int32_t       status = PMIC_ST_SUCCESS;
    Pmic_adcCfg_t adcCfg;

    // Initialize ADC CFG
    resetAdcCfg_withAllValidParams(&adcCfg);

    // Pass in NULL PMIC handle into the API
    status = Pmic_ADCGetConfiguration(NULL, &adcCfg);

    // Compare expected vs. actual return code
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

/**
 *  \brief  Pmic_ADCGetConfiguration: Test API error handling for when ADC CFG struct is NULL
 */
void test_ADC_getConfiguration_nullAdcCfg(void)
{
    int32_t status = PMIC_ST_SUCCESS;

    // Pass in NULL ADC CFG into the API
    status = Pmic_ADCGetConfiguration(&pmicCoreHandle, NULL);

    // Compare expected vs. actual return code
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

/**
 *  \brief  Pmic_ADCGetConfiguration: Test API error handling for when validParams is zero
 */
void test_ADC_getConfiguration_noValidParam(void)
{
    int32_t       status = PMIC_ST_SUCCESS;
    Pmic_adcCfg_t adcCfg;

    // Initialize ADC CFG with no validParams
    resetAdcCfg_withNoValidParams(&adcCfg);

    // Pass in ADC CFG with no validParams into API
    status = Pmic_ADCGetConfiguration(&pmicCoreHandle, &adcCfg);

    // Compare expected vs. actual return code
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);
}

/**
 *  \brief  Pmic_ADCSetConfiguration: Test API error handling for when PMIC handle is NULL
 */
void test_ADC_setConfiguration_nullPmicHandle(void)
{
    int32_t       status = PMIC_ST_SUCCESS;
    Pmic_adcCfg_t adcCfg;

    // Initialize ADC CFG
    resetAdcCfg_withAllValidParams(&adcCfg);

    // Pass in NULL PMIC handle into the API
    status = Pmic_ADCSetConfiguration(NULL, adcCfg);

    // Compare expected vs. actual return code
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

/**
 *  \brief  Pmic_ADCSetConfiguration: Test API error handling for when validParams is zero
 */
void test_ADC_setConfiguration_noValidParam(void)
{
    int32_t       status = PMIC_ST_SUCCESS;
    Pmic_adcCfg_t adcCfg;

    // Initialize ADC CFG with no validParams
    resetAdcCfg_withNoValidParams(&adcCfg);

    // Pass in ADC CFG with no validParams into API
    status = Pmic_ADCSetConfiguration(&pmicCoreHandle, adcCfg);

    // Compare expected vs. actual return code
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);
}

/**
 *  \brief  Pmic_ADCSetConfiguration: Test whether API can enable/disable ADC resistor divider
 */
void test_ADC_setConfiguration_RDiv_EnableDisable(void)
{
    int32_t       status = PMIC_ST_SUCCESS;
    Pmic_adcCfg_t expectedAdcCfg, actualAdcCfg;

    // Initialize ADC CFGs
    resetAdcCfg_withSpecificValidParams(&expectedAdcCfg, PMIC_ADC_CFG_RDIV_EN_VALID_SHIFT);
    resetAdcCfg_withAllValidParams(&actualAdcCfg);

    // Enable ADC resistor divider
    expectedAdcCfg.rDivEn = PMIC_ADC_RESISTOR_DIVIDER_ENABLED;
    status = Pmic_ADCSetConfiguration(&pmicCoreHandle, expectedAdcCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get actual ADC configuration
    status = Pmic_ADCGetConfiguration(&pmicCoreHandle, &actualAdcCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Compare expected vs. actual value
    TEST_ASSERT_EQUAL(expectedAdcCfg.rDivEn, actualAdcCfg.rDivEn);

    // Disable ADC resistor divider
    expectedAdcCfg.rDivEn = PMIC_ADC_RESISTOR_DIVIDER_DISABLED;
    status = Pmic_ADCSetConfiguration(&pmicCoreHandle, expectedAdcCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get actual ADC configuration
    status = Pmic_ADCGetConfiguration(&pmicCoreHandle, &actualAdcCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Compare expected vs. actual value
    TEST_ASSERT_EQUAL(expectedAdcCfg.rDivEn, actualAdcCfg.rDivEn);
}

static void adcSetConfigurationThermalSel_test(Pmic_adcThermalSel_t thermalSel)
{
    int32_t       status = PMIC_ST_SUCCESS;
    Pmic_adcCfg_t expectedAdcCfg, actualAdcCfg;

    // Initialize ADC CFGs
    resetAdcCfg_withSpecificValidParams(&expectedAdcCfg, PMIC_ADC_CFG_THERMAL_SEL_VALID_SHIFT);
    resetAdcCfg_withAllValidParams(&actualAdcCfg);

    // Set ADC conversion source to be ADC input
    expectedAdcCfg.thermalSel = thermalSel;
    status = Pmic_ADCSetConfiguration(&pmicCoreHandle, expectedAdcCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get actual ADC configuration
    status = Pmic_ADCGetConfiguration(&pmicCoreHandle, &actualAdcCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Compare expected vs. actual value
    TEST_ASSERT_EQUAL(expectedAdcCfg.thermalSel, actualAdcCfg.thermalSel);
}

/**
 *  \brief  Pmic_ADCSetConfiguration: Test whether API can configure ADC conversion source
 *                                    to be ADC input
 */
void test_ADC_setConfiguration_thermalSel_adcInput(void)
{
    adcSetConfigurationThermalSel_test(PMIC_ADC_THERMAL_SEL_ADC_INPUT);
}

/**
 *  \brief  Pmic_ADCSetConfiguration: Test whether API can configure ADC conversion source
 *                                    to be thermal sensor
 */
void test_ADC_setConfiguration_thermalSel_thermalSensor(void)
{
    adcSetConfigurationThermalSel_test(PMIC_ADC_THERMAL_SEL_THERMAL_SENSOR);
}

/**
 *  \brief  Pmic_ADCSetConfiguration: Test whether API can enable/disable ADC continuous conversion
 */
void test_ADC_setConfiguration_contConv_enableDisable(void)
{
    int32_t       status = PMIC_ST_SUCCESS;
    Pmic_adcCfg_t expectedAdcCfg, actualAdcCfg;

    // Initialize ADC CFGs
    resetAdcCfg_withSpecificValidParams(&expectedAdcCfg, PMIC_ADC_CFG_CONT_CONV_VALID_SHIFT);
    resetAdcCfg_withAllValidParams(&actualAdcCfg);

    // Enable ADC continuous conversion
    expectedAdcCfg.contConv = PMIC_ADC_CONTINUOUS_CONVERSION_ENABLED;
    status = Pmic_ADCSetConfiguration(&pmicCoreHandle, expectedAdcCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get actual ADC configuration
    status = Pmic_ADCGetConfiguration(&pmicCoreHandle, &actualAdcCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Compare expected vs. actual value
    TEST_ASSERT_EQUAL(expectedAdcCfg.contConv, actualAdcCfg.contConv);

    // Disable ADC continuous conversion
    expectedAdcCfg.contConv = PMIC_ADC_CONTINUOUS_CONVERSION_DISABLED;
    status = Pmic_ADCSetConfiguration(&pmicCoreHandle, expectedAdcCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get actual ADC configuration
    status = Pmic_ADCGetConfiguration(&pmicCoreHandle, &actualAdcCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Compare expected vs. actual value
    TEST_ASSERT_EQUAL(expectedAdcCfg.contConv, actualAdcCfg.contConv);
}

/**
 *  \brief  Pmic_ADCStartSingleConversion: Test API error handling for when PMIC handle is NULL
 */
void test_ADC_startSingleConversion_nullPmicHandle(void)
{
    int32_t status = PMIC_ST_SUCCESS;

    // Pass in NULL PMIC handle into API and compare expected vs. actual return code
    status = Pmic_ADCStartSingleConversion(&pmicCoreHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

/**
 *  \brief  Pmic_ADCStartSingleConversion: Test API error handling for when ADC continuous conversion
 *                                         is enabled
 */
void test_ADC_startSingleConversion_contConvEnabled(void)
{
    int32_t       status = PMIC_ST_SUCCESS;
    Pmic_adcCfg_t adcCfg;

    // Initialize adcCfg
    resetAdcCfg_withSpecificValidParams(&adcCfg, PMIC_ADC_CFG_CONT_CONV_VALID);

    // Set ADC continuous conversion to be enabled
    adcCfg.contConv = PMIC_ADC_CONTINUOUS_CONVERSION_ENABLED;
    status = Pmic_ADCSetConfiguration(&pmicCoreHandle, adcCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Start ADC single conversion and compare expected vs. actual return code
    status = Pmic_ADCStartSingleConversion(&pmicCoreHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_ADC_CONT_CONV_EN, status);
}

/**
 *  \brief  Pmic_ADCStartSingleConversionBlocking: Test API error handling for when PMIC handle is NULL
 */
void test_ADC_startSingleConversionBlocking_nullPmicHandle(void)
{
    int32_t status = PMIC_ST_SUCCESS;

    // Pass in NULL PMIC handle into API and compare expected vs. actual return code
    status = Pmic_ADCStartSingleConversionBlocking(&pmicCoreHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

/**
 *  \brief  Pmic_ADCStartSingleConversionBlocking: Test API error handling for when continuous conversion
 *                                                 is enabled
 */
void test_ADC_startSingleConversionBlocking_contConvEnabled(void)
{
    int32_t       status = PMIC_ST_SUCCESS;
    Pmic_adcCfg_t adcCfg;

    // Initialize adcCfg
    resetAdcCfg_withSpecificValidParams(&adcCfg, PMIC_ADC_CFG_CONT_CONV_VALID);

    // Set ADC continuous conversion to be enabled
    adcCfg.contConv = PMIC_ADC_CONTINUOUS_CONVERSION_ENABLED;
    status = Pmic_ADCSetConfiguration(&pmicCoreHandle, adcCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Start ADC single conversion and compare expected vs. actual return code
    status = Pmic_ADCStartSingleConversionBlocking(&pmicCoreHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_ADC_CONT_CONV_EN, status);
}

/**
 *  \brief  Pmic_ADCGetStatus: Test API error handling for when its parameters are NULL
 */
void test_ADC_getStatus_nullParam(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    bool    adcBusy = false;

    // Pass NULL PMIC handle into API and compare expected vs. actual return code
    status = Pmic_ADCGetStatus(NULL, &adcBusy);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    // Pass NULL variable into API and compare expected vs. actual return code
    status = Pmic_ADCGetStatus(&pmicCoreHandle, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

/**
 *  \brief  Pmic_ADCGetStatus: Test whether API can detect whether ADC is idle
 */
void test_ADC_getStatus_idle(void)
{
    int32_t       status = PMIC_ST_SUCCESS;
    bool          adcBusy = false;
    Pmic_adcCfg_t adcCfg;

    // Configure PMIC GPIO4 to ADC_IN functionality
    status = Pmic_gpioPinTypeADC(&pmicCoreHandle, PMIC_TPS6522X_GPIO4_PIN);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Initialize adcCfg
    resetAdcCfg_withAllValidParams(&adcCfg);
    adcCfg.rDivEn = PMIC_ADC_RESISTOR_DIVIDER_DISABLED;
    adcCfg.thermalSel = PMIC_ADC_THERMAL_SEL_THERMAL_SENSOR;
    adcCfg.contConv = PMIC_ADC_CONTINUOUS_CONVERSION_DISABLED;

    // Configure ADC
    status = Pmic_ADCSetConfiguration(&pmicCoreHandle, adcCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Wait a short period of time so ADC stays idle
    delayTimeInMs(&timerHandle, 100);

    // Get ADC status and compare expected vs. actual status
    status = Pmic_ADCGetStatus(&pmicCoreHandle, &adcBusy);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(false, adcBusy);
}

/**
 *  \brief  Pmic_ADCGetStatus: Test whether API can detect whether ADC is busy
 */
void test_ADC_getStatus_busy(void)
{
    int32_t       status = PMIC_ST_SUCCESS;
    bool          adcBusy = false;
    Pmic_adcCfg_t adcCfg;

    // Configure PMIC GPIO4 to ADC_IN functionality
    status = Pmic_gpioPinTypeADC(&pmicCoreHandle, PMIC_TPS6522X_GPIO4_PIN);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Initialize adcCfg
    resetAdcCfg_withAllValidParams(&adcCfg);
    adcCfg.rDivEn = PMIC_ADC_RESISTOR_DIVIDER_DISABLED;
    adcCfg.thermalSel = PMIC_ADC_THERMAL_SEL_THERMAL_SENSOR;
    adcCfg.contConv = PMIC_ADC_CONTINUOUS_CONVERSION_DISABLED;

    // Configure ADC
    status = Pmic_ADCSetConfiguration(&pmicCoreHandle, adcCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Start single conversion
    status = Pmic_ADCStartSingleConversion(&pmicCoreHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get ADC status and compare expected vs. actual status
    status = Pmic_ADCGetStatus(&pmicCoreHandle, &adcBusy);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(true, adcBusy);
}

/**
 *  \brief  Pmic_ADCGetResultCode: Test API error handling for when its parameters are NULL
 */
void test_ADC_getResultCode_nullParam(void)
{
    int32_t  status = PMIC_ST_SUCCESS;
    uint16_t adcResultCode = 0;

    // Pass NULL PMIC handle into API and compare expected vs. actual return code
    status = Pmic_ADCGetResultCode(NULL, &adcResultCode);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    // Pass NULL variable into API and compare expected vs. actual return code
    status = Pmic_ADCGetResultCode(&pmicCoreHandle, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

/**
 *  \brief  Pmic_ADCGetResultCode: Test whether API can obtain the result code after a voltage
 *                                 value is processed by the ADC
 *
 *  \note   For this test, the voltage input must be connected to PMIC GPIO4
 */
void test_ADC_getResultCode_voltage(void)
{
    int32_t       status = PMIC_ST_SUCCESS;
    bool          adcBusy = true;
    uint16_t      adcResultCode = 0;
    Pmic_adcCfg_t adcCfg;

    // Configure PMIC GPIO4 to ADC_IN functionality
    status = Pmic_gpioPinTypeADC(&pmicCoreHandle, PMIC_TPS6522X_GPIO4_PIN);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Initialize adcCfg
    resetAdcCfg_withAllValidParams(&adcCfg);
    adcCfg.rDivEn = PMIC_ADC_RESISTOR_DIVIDER_DISABLED;
    adcCfg.thermalSel = PMIC_ADC_THERMAL_SEL_ADC_INPUT;
    adcCfg.contConv = PMIC_ADC_CONTINUOUS_CONVERSION_ENABLED;

    // Configure ADC
    status = Pmic_ADCSetConfiguration(&pmicCoreHandle, adcCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // With continuous conversion enabled, wait while ADC is busy
    do
    {
        status = Pmic_ADCGetStatus(&pmicCoreHandle, &adcBusy);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    }
    while (adcBusy);

    // Get voltage result code
    status = Pmic_ADCGetResultCode(&pmicCoreHandle, &adcResultCode);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
}

/**
 *  \brief  Pmic_ADCGetResultCode: Test whether API can obtain the result code after a temperature
 *                                 value is processed by the ADC
 */
void test_ADC_getResultCode_temperature(void)
{
    int32_t       status = PMIC_ST_SUCCESS;
    bool          adcBusy = true;
    uint16_t      adcResultCode = 0;
    Pmic_adcCfg_t adcCfg;

    // Initialize adcCfg
    resetAdcCfg_withAllValidParams(&adcCfg);
    adcCfg.rDivEn = PMIC_ADC_RESISTOR_DIVIDER_DISABLED;
    adcCfg.thermalSel = PMIC_ADC_THERMAL_SEL_THERMAL_SENSOR;
    adcCfg.contConv = PMIC_ADC_CONTINUOUS_CONVERSION_ENABLED;

    // Configure ADC
    status = Pmic_ADCSetConfiguration(&pmicCoreHandle, adcCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // With continuous conversion enabled, wait while ADC is busy
    do
    {
        status = Pmic_ADCGetStatus(&pmicCoreHandle, &adcBusy);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    }
    while (adcBusy);

    // Get temperature result code
    status = Pmic_ADCGetResultCode(&pmicCoreHandle, &adcResultCode);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
}

/**
 *  \brief  Pmic_ADCGetResultCode: Test API error handling for when its parameters are NULL
 */
void test_ADC_getResult_nullParam(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    int32_t adcResult = 0;

    // Pass NULL PMIC handle into API and compare expected vs. actual return code
    status = Pmic_ADCGetResult(NULL, &adcResult);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    // Pass NULL variable into API and compare expected vs. actual return code
    status = Pmic_ADCGetResult(&pmicCoreHandle, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

/**
 *  \brief  Pmic_ADCGetResultCode: Test whether API can obtain the result after a voltage value
 *                                 is processed by the ADC
 */
void test_ADC_getResult_voltage(void)
{
    int32_t       status = PMIC_ST_SUCCESS;
    bool          adcBusy = true;
    int32_t       adcResult = 0;
    Pmic_adcCfg_t adcCfg;

    // Configure PMIC GPIO4 to ADC_IN functionality
    status = Pmic_gpioPinTypeADC(&pmicCoreHandle, PMIC_TPS6522X_GPIO4_PIN);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Initialize adcCfg
    resetAdcCfg_withAllValidParams(&adcCfg);
    adcCfg.rDivEn = PMIC_ADC_RESISTOR_DIVIDER_DISABLED;
    adcCfg.thermalSel = PMIC_ADC_THERMAL_SEL_ADC_INPUT;
    adcCfg.contConv = PMIC_ADC_CONTINUOUS_CONVERSION_ENABLED;

    // Configure ADC
    status = Pmic_ADCSetConfiguration(&pmicCoreHandle, adcCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // With continuous conversion enabled, wait while ADC is busy
    do
    {
        status = Pmic_ADCGetStatus(&pmicCoreHandle, &adcBusy);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    }
    while (adcBusy);

    // Get voltage result
    status = Pmic_ADCGetResult(&pmicCoreHandle, &adcResult);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
}

/**
 *  \brief  Pmic_ADCGetResultCode: Test whether API can obtain the result after a temperature value
 *                                 is processed by the ADC
 */
void test_ADC_getResult_temperature(void)
{
    int32_t       status = PMIC_ST_SUCCESS;
    bool          adcBusy = true;
    int32_t       adcResult = 0;
    Pmic_adcCfg_t adcCfg;

    // Initialize adcCfg
    resetAdcCfg_withAllValidParams(&adcCfg);
    adcCfg.rDivEn = PMIC_ADC_RESISTOR_DIVIDER_DISABLED;
    adcCfg.thermalSel = PMIC_ADC_THERMAL_SEL_THERMAL_SENSOR;
    adcCfg.contConv = PMIC_ADC_CONTINUOUS_CONVERSION_ENABLED;

    // Configure ADC
    status = Pmic_ADCSetConfiguration(&pmicCoreHandle, adcCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // With continuous conversion enabled, wait while ADC is busy
    do
    {
        status = Pmic_ADCGetStatus(&pmicCoreHandle, &adcBusy);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    }
    while (adcBusy);

    // Get temperature result code
    status = Pmic_ADCGetResult(&pmicCoreHandle, &adcResult);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
}

/**
 * \brief This function is called by Unity when it starts a test
 */
void setUp(void)
{
}

/**
 * \brief This function is called by Unity when it finishes a test
 */
void tearDown(void)
{
    uint8_t       txBuf = 0;
    const uint8_t adcCtrlRegAddr = 0xAC;

    // Reset ADC_CTRL
    (void)pmicI2CWrite(&pmicCoreHandle, PMIC_MAIN_INST, adcCtrlRegAddr, &txBuf, 1);

    // Clear all interrupts
    (void)Pmic_irqClrErrStatus(&pmicCoreHandle, PMIC_IRQ_ALL);
}
