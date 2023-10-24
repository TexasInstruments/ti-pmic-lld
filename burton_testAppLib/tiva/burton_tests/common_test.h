/**
 * \file common_test.h
 * \author John Bui (j-bui1@ti.com)
 * \brief Header file for common test application definitions and APIs
 * \version 1.0
 * \date 2023-10-12
 *
 * \copyright Copyright (c) 2023
 */
#ifndef COMMON_TEST_H
#define COMMON_TEST_H

#include "pmic_drv/pmic.h"
#include "driverlib/uart.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "inc/hw_memmap.h"

#ifdef __cplusplus
extern "C"
{
#endif

#ifndef PMIC_TPS6522X_GPIO_PIN_MAX
#define PMIC_TPS6522X_GPIO_PIN_MAX (6U)
#endif

#define VCCA_VMON_CTRL_REG_ADDR (0x2BU)
#define LDO2_CTRL_REG_ADDR      (0x1EU)

/**
 * \brief Helper function to reset a GPIO configuration struct with all valid parameters set.
 *        Driver APIs will consider a struct member as invalid if its corresponding validParams is not set.
 *
 * \param pGpioCfg  [OUT]   GPIO configuration struct to reset with valid parameters
 */
void resetGpioCfg_withAllValidParams(Pmic_GpioCfg_t *pGpioCfg);

/**
 * \brief Helper function to reset a GPIO configuration struct with specific valid parameters set.
 *        Driver APIs will consider a struct member as invalid if its corresponding validParams is not set.
 *
 * \param pGpioCfg  [OUT]   GPIO configuration struct to reset with valid parameters
 */
void resetGpioCfg_withSpecificValidParams(Pmic_GpioCfg_t *pGpioCfg, uint8_t validParams);

/**
 * \brief Unity uses this API in all its write, put, or print APIs
 *
 * \param ucData    [IN]    Character to write to the terminal
 */
void unityCharPut(unsigned char ucData);

#ifdef __cplusplus
}

#endif /* __cplusplus */

#endif /* COMMON_TEST_H */
