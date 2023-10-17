/**
 * \file gpio_test.h
 * \author John Bui (j-bui1@ti.com)
 * \brief Header file for Burton GPIO Unity tests
 * \version 1.0
 * \date 2023-10-12
 *
 * \copyright Copyright (c) 2023
 *
 */
#ifndef GPIO_TEST_H
#define GPIO_TEST_H

#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"

#ifdef __cplusplus
extern "C"
{
#endif

#ifndef PMIC_TPS6522X_GPIO_PIN_MAX
#define PMIC_TPS6522X_GPIO_PIN_MAX (6U)
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

#define GPIO_INT_MASKED          (bool)(true)
#define GPIO_INT_UNMASKED        (bool)(false)
#define GPIO_FALL_INT_MASKED     (bool)(true)
#define GPIO_FALL_INT_UNMASKED   (bool)(false)
#define GPIO_RISE_INT_MASKED     (bool)(true)
#define GPIO_RISE_INT_UNMASKED   (bool)(false)

/**
 * \brief test Pmic_gpioGetConfiguration : Read all GPIO configurations
 *
 */
void test_Pmic_gpioGetConfiguration_forCorrectReads(void);

/**
 * \brief test Pmic_gpioSetConfiguration : Reset values of all GPIO configuration registers
 *        to zero then set their values to the NVM default configuration
 *
 */
void test_Pmic_gpioSetConfiguration_resetSetAllGpioCfg(void);

/**
 * \brief test Pmic_gpioGetValue : Read all PMIC GPIOs values; test the value received
 *        from driver API against value received from application layer created read API
 *
 */
void test_Pmic_gpioGetValue_forCorrectReads(void);

/**
 * \brief test Pmic_gpioSetValue : Set all GPIO output values to one. Afterwards, reset all GPIO output
 *                                 values to zero. In both scenarios, check for expected output values
 *
 * \note For this test, there must be a total of 6 physical GPIO pin connections between Burton and test MCU
 *
 */
void test_Pmic_gpioSetValue_setGpioSignalLvl(void);

/**
 * \brief test Pmic_gpioSetIntr :  Configure interrupts for all GPIO pins
 *
 */
void test_Pmic_gpioSetIntr_configIntOnAllGpio(void);

/**
 * \brief test Pmic_gpioSetIntr : Disable interrupts for all GPIO pins
 *
 */
void test_Pmic_gpioSetIntr_disableIntOnAllGpio(void);

#ifdef __cplusplus
}

#endif /* __cplusplus */

#endif /* GPIO_TEST_H */
