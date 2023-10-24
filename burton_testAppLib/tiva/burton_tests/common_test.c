/**
 * \file common_test.c
 * \author John Bui (j-bui1@ti.com)
 * \brief Source file for common test application APIs
 * \version 1.0
 * \date 2023-10-12
 *
 * \copyright Copyright (c) 2023
 */
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

#include "common_test.h"

/**
 * \brief Helper function to reset a GPIO configuration struct with all valid parameters set.
 *        Driver APIs will consider a struct member as invalid if its corresponding validParams is not set.
 *
 * \param pGpioCfg  [OUT]   GPIO configuration struct to reset with valid parameters
 */
void resetGpioCfg_withAllValidParams(Pmic_GpioCfg_t *pGpioCfg)
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
 * \brief Helper function to reset a GPIO configuration struct with specific valid parameters set.
 *        Driver APIs will consider a struct member as invalid if its corresponding validParams is not set.
 *
 * \param pGpioCfg      [OUT]   GPIO configuration struct to reset with valid parameters
 * \param validParams   [IN]    Indication of the valid values contained within pGpioCfg
 */
void resetGpioCfg_withSpecificValidParams(Pmic_GpioCfg_t *pGpioCfg, uint8_t validParams)
{
    pGpioCfg->validParams = validParams;
    pGpioCfg->pinDir = 0;
    pGpioCfg->outputSignalType = 0;
    pGpioCfg->pullCtrl = 0;
    pGpioCfg->deglitchEnable = 0;
    pGpioCfg->pinFunc = 0;
    pGpioCfg->pinPolarity = 0;
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
