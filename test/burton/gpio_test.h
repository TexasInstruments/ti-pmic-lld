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
 * \file gpio_test.h
 * \author John Bui (j-bui1@ti.com)
 * \brief Header file for Burton GPIO Unity tests
 * \version 1.0
 */
#ifndef GPIO_TEST_H
#define GPIO_TEST_H

#include "common_test.h"

#ifdef __cplusplus
extern "C"
{
#endif

#ifndef PMIC_GPIOX_CONF_GPIO_SEL_SHIFT
#define PMIC_GPIOX_CONF_GPIO_SEL_SHIFT (0x05U)
#endif
#ifndef PMIC_GPIOX_CONF_GPIO_DEGLITCH_EN_SHIFT
#define PMIC_GPIOX_CONF_GPIO_DEGLITCH_EN_SHIFT (0x04U)
#endif
#ifndef PMIC_GPIOX_CONF_GPIO_PU_PD_EN_SHIFT
#define PMIC_GPIOX_CONF_GPIO_PU_PD_EN_SHIFT (0x03U)
#endif
#ifndef PMIC_GPIOX_CONF_GPIO_PU_SEL_SHIFT
#define PMIC_GPIOX_CONF_GPIO_PU_SEL_SHIFT (0x02U)
#endif
#ifndef PMIC_GPIOX_CONF_GPIO_OD_SHIFT
#define PMIC_GPIOX_CONF_GPIO_OD_SHIFT (0x01U)
#endif
#ifndef PMIC_GPIOX_CONF_GPIO_DIR_SHIFT
#define PMIC_GPIOX_CONF_GPIO_DIR_SHIFT (0x00U)
#endif

#define FSM_TRIG_MASK_1_REG_ADDR (0x46U)
#define FSM_TRIG_MASK_2_REG_ADDR (0x47U)
#define MASK_GPIO_FALL_REG_ADDR  (0x4FU)
#define MASK_GPIO_RISE_REG_ADDR  (0x50U)
#define POWER_ON_CONFIG_REG_ADDR (0x3CU)

#define GPIO_INT_MASKED          (bool)(true)
#define GPIO_INT_UNMASKED        (bool)(false)
#define GPIO_FALL_INT_MASKED     (bool)(true)
#define GPIO_FALL_INT_UNMASKED   (bool)(false)
#define GPIO_RISE_INT_MASKED     (bool)(true)
#define GPIO_RISE_INT_UNMASKED   (bool)(false)

/**
 * \brief test Pmic_gpioSetConfiguration: Configure a GPIO pin to VMON1_m functionality
 */
void test_Pmic_gpioSetConfiguration_VMON1_m(void);

/**
 * \brief test Pmic_gpioSetConfiguration: Configure a GPIO pin to VMON2 functionality
 */
void test_Pmic_gpioSetConfiguration_VMON2(void);

/**
 * \brief test Pmic_gpioSetConfiguration: Configure a GPIO pin to Push Button functionality
 */
void test_Pmic_gpioSetConfiguration_pushButton(void);

/**
 * \brief test Pmic_gpioSetConfiguration: Configure a GPIO pin to nSLEEP1 functionality
 */
void test_Pmic_gpioSetConfiguration_nSLEEP1(void);

/**
 * \brief test Pmic_gpioSetConfiguration: Configure a GPIO pin to nSLEEP2 functionality
 */
void test_Pmic_gpioSetConfiguration_nSLEEP2(void);

/**
 * \brief test Pmic_gpioSetConfiguration: Configure a GPIO pin to ADC_IN functionality
 */
void test_Pmic_gpioSetConfiguration_ADC_IN(void);

/**
 * \brief test Pmic_gpioSetConfiguration: Configure a GPIO pin to WKUP functionality
 */
void test_Pmic_gpioSetConfiguration_WKUP(void);

/**
 * \brief test Pmic_gpioSetConfiguration: Configure a GPIO pin to SYNCCLKIN functionality
 */
void test_Pmic_gpioSetConfiguration_SYNCCLKIN(void);

/**
 * \brief test Pmic_gpioSetConfiguration: Configure a GPIO pin to nERR_MCU functionality
 */
void test_Pmic_gpioSetConfiguration_nERR_MCU(void);

/**
 * \brief test Pmic_gpioSetConfiguration: Configure a GPIO pin to SDA_I2C2/SDO_SPI functionality
 */
void test_Pmic_gpioSetConfiguration_SDA_I2C2_SDO_SPI(void);

/**
 * \brief test Pmic_gpioSetConfiguration: Configure a GPIO pin to SCL_I2C2/CS_SPI functionality
 */
void test_Pmic_gpioSetConfiguration_SCL_I2C2_CS_SPI(void);

/**
 * \brief test Pmic_gpioSetConfiguration: Configure a GPIO pin to nINT functionality
 */
void test_Pmic_gpioSetConfiguration_nINT(void);

/**
 * \brief test Pmic_gpioSetConfiguration: Configure a GPIO pin to TRIG_WDOG functionality
 */
void test_Pmic_gpioSetConfiguration_TRIG_WDOG(void);

/**
 * \brief test Pmic_gpioGetValue: Send a high signal level from MCU to PMIC and get PMIC GPIO input value
 *
 * \note For this test, there must be a total of 6 physical GPIO pin connections between Burton and test MCU
 */
void test_Pmic_gpioGetValue_readGpioSignalLvl(void);

/**
 * \brief test Pmic_gpioSetValue: Set all GPIO output values to one. Afterwards, reset all GPIO output
 *                                values to zero. In both scenarios, check for expected output values
 *
 * \note For this test, there must be a total of 6 physical GPIO pin connections between Burton and test MCU
 */
void test_Pmic_gpioSetValue_setGpioSignalLvl(void);

/**
 * \brief test Pmic_gpioSetEnPbVsensePinConfiguration: Parameter validation of PMIC handle
 */
void test_Pmic_gpioSetEnPbVsensePinConfiguration_validatePmicHandle(void);

/**
 * \brief test Pmic_gpioSetEnPbVsensePinConfiguration: Parameter validation of pin functionality struct member of
 *                                                     enPbVsenseCfg
 */
void test_Pmic_gpioSetEnPbVsensePinConfiguration_validatePinFunc(void);

/**
 * \brief test Pmic_gpioSetEnPbVsensePinConfiguration: Configure EN/PB/VSENSE pin to EN functionality
 */
void test_Pmic_gpioSetEnPbVsensePinConfiguration_enableFunc(void);

/**
 * \brief test Pmic_gpioSetEnPbVsensePinConfiguration: Configure EN/PB/VSENSE pin to PB functionality
 */
void test_Pmic_gpioSetEnPbVsensePinConfiguration_pushButtonFunc(void);

/**
 * \brief test Pmic_gpioSetEnPbVsensePinConfiguration: Configure EN/PB/VSENSE pin to VSENSE functionality
 */
void test_Pmic_gpioSetEnPbVsensePinConfiguration_vsenseFunc(void);

/**
 * \brief test Push Button On Request: Validate that the push button generates an On Request (PB_SHORT_INT
 *                                     flag is raised and nRSTOUT signal is set high)
 *
 * \note At the start of the test, it is required that the push button is to be pressed within 15 seconds
 *       or test will fail
 */
void test_pushButton_onRequest(void);

/**
 * \brief test Push Button Off Request: Validate that the push button generates an Off Request (PB_LONG_INT
 *                                      flag is raised and nRSTOUT signal is set low)
 *
 * \note At the start of the test, there will be 15 seconds to hold the push button for 8 seconds
 *       or else test will fail
 */
void test_pushButton_offRequest(void);

#ifdef __cplusplus
}

#endif /* __cplusplus */

#endif /* GPIO_TEST_H */
