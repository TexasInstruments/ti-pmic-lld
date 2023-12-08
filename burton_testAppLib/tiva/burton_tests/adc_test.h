/**
 * \file adc_test.h
 * \author John Bui (j-bui1@ti.com)
 * \brief Header file for Burton ADC Unity tests
 * \version 1.0
 * \date 2023-12-06
 *
 * \copyright Copyright (c) 2023 Texas Instruments Incorporated
 *
 */
#ifndef ADC_TEST_H
#define ADC_TEST_H

#include "common_test.h"

#ifdef __cplusplus
extern "C"
{
#endif

/**
 *  \brief  Pmic_gpioPinTypeADC: Test API error handling for when PMIC handle is NULL
 */
void test_ADC_gpioPinTypeADC_nullPmicHandle(void);

/**
 *  \brief  Pmic_gpioPinTypeADC: Test API error handling for when GPIO number is
 *                               unsupported
 */
void test_ADC_gpioPinTypeADC_invalidGpioNum(void);

/**
 *  \brief  Pmic_gpioPinTypeADC: Test whether API can configure GPIO 4 to ADC functionality
 */
void test_ADC_gpioPinTypeADC_gpio4(void);

/**
 *  \brief  Pmic_gpioPinTypeADC: Test whether API can configure GPIO 5 to ADC functionality
 */
void test_ADC_gpioPinTypeADC_gpio5(void);

/**
 *  \brief  Pmic_ADCGetConfiguration: Test API error handling for when PMIC handle is NULL
 */
void test_ADC_getConfiguration_nullPmicHandle(void);

/**
 *  \brief  Pmic_ADCGetConfiguration: Test API error handling for when ADC CFG struct is NULL
 */
void test_ADC_getConfiguration_nullAdcCfg(void);

/**
 *  \brief  Pmic_ADCGetConfiguration: Test API error handling for when validParams is zero
 */
void test_ADC_getConfiguration_noValidParam(void);

/**
 *  \brief  Pmic_ADCSetConfiguration: Test API error handling for when PMIC handle is NULL
 */
void test_ADC_setConfiguration_nullPmicHandle(void);

/**
 *  \brief  Pmic_ADCSetConfiguration: Test API error handling for when validParams is zero
 */
void test_ADC_setConfiguration_noValidParam(void);

/**
 *  \brief  Pmic_ADCSetConfiguration: Test whether API can enable/disable ADC resistor divider
 */
void test_ADC_setConfiguration_RDiv_EnableDisable(void);

/**
 *  \brief  Pmic_ADCSetConfiguration: Test whether API can configure ADC conversion source
 *                                    to be ADC input
 */
void test_ADC_setConfiguration_thermalSel_adcInput(void);

/**
 *  \brief  Pmic_ADCSetConfiguration: Test whether API can configure ADC conversion source
 *                                    to be thermal sensor
 */
void test_ADC_setConfiguration_thermalSel_thermalSensor(void);

/**
 *  \brief  Pmic_ADCSetConfiguration: Test whether API can enable/disable ADC continuous conversion
 */
void test_ADC_setConfiguration_contConv_enableDisable(void);

/**
 *  \brief  Pmic_ADCStartSingleConversion: Test API error handling for when PMIC handle is NULL
 */
void test_ADC_startSingleConversion_nullPmicHandle(void);

/**
 *  \brief  Pmic_ADCStartSingleConversion: Test API error handling for when ADC continuous conversion
 *                                         is enabled
 */
void test_ADC_startSingleConversion_contConvEnabled(void);

/**
 *  \brief  Pmic_ADCStartSingleConversionBlocking: Test API error handling for when PMIC handle is NULL
 */
void test_ADC_startSingleConversionBlocking_nullPmicHandle(void);

/**
 *  \brief  Pmic_ADCStartSingleConversionBlocking: Test API error handling for when continuous conversion
 *                                                 is enabled
 */
void test_ADC_startSingleConversionBlocking_contConvEnabled(void);

/**
 *  \brief  Pmic_ADCGetStatus: Test API error handling for when its parameters are NULL
 */
void test_ADC_getStatus_nullParam(void);

/**
 *  \brief  Pmic_ADCGetStatus: Test whether API can detect whether ADC is idle
 */
void test_ADC_getStatus_idle(void);

/**
 *  \brief  Pmic_ADCGetStatus: Test whether API can detect whether ADC is busy
 */
void test_ADC_getStatus_busy(void);

/**
 *  \brief  Pmic_ADCGetResultCode: Test API error handling for when its parameters are NULL
 */
void test_ADC_getResultCode_nullParam(void);

/**
 *  \brief  Pmic_ADCGetResultCode: Test whether API can obtain the result code after a voltage
 *                                 value is processed by the ADC
 *
 *  \note   For this test, the voltage input must be connected to PMIC GPIO4
 */
void test_ADC_getResultCode_voltage(void);

/**
 *  \brief  Pmic_ADCGetResultCode: Test whether API can obtain the result code after a temperature
 *                                 value is processed by the ADC
 */
void test_ADC_getResultCode_temperature(void);

/**
 *  \brief  Pmic_ADCGetResultCode: Test API error handling for when its parameters are NULL
 */
void test_ADC_getResult_nullParam(void);

/**
 *  \brief  Pmic_ADCGetResultCode: Test whether API can obtain the result after a voltage value
 *                                 is processed by the ADC
 *
 *  \note   For this test, the voltage input must be connected to PMIC GPIO4
 */
void test_ADC_getResult_voltage(void);

/**
 *  \brief  Pmic_ADCGetResultCode: Test whether API can obtain the result after a temperature value
 *                                 is processed by the ADC
 */
void test_ADC_getResult_temperature(void);

#ifdef __cplusplus
}

#endif /* __cplusplus */

#endif /* ADC_TEST_H */
