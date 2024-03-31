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
 * \file common_test.h
 * \author John Bui (j-bui1@ti.com)
 * \brief Header file for common test application definitions and APIs
 * \version 1.0
 */
#ifndef COMMON_TEST_H
#define COMMON_TEST_H

#include "tiva_testLib.h"

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
 *  \brief  Helper function to reset a GPIO configuration struct with all valid parameters set.
 *          Driver APIs will consider a struct member as invalid if its corresponding validParams is not set.
 *
 *  \param  pGpioCfg    [OUT]   GPIO configuration struct to reset with valid parameters
 */
void resetGpioCfg_withAllValidParams(Pmic_GpioCfg_t *pGpioCfg);

/**
 *  \brief  Helper function to reset a GPIO configuration struct with specific valid parameters set.
 *          Driver APIs will consider a struct member as invalid if its corresponding validParams is not set.
 *
 *  \param  pGpioCfg    [OUT]   GPIO configuration struct to reset with valid parameters
 */
void resetGpioCfg_withSpecificValidParams(Pmic_GpioCfg_t *pGpioCfg, uint8_t validParams);

/**
 *  \brief  Unity uses this API in all its write, put, or print APIs
 *
 *  \param  ucData      [IN]    Character to write to the terminal
 */
void unityCharPut(uint8_t ucData);

/**
 *  \brief  This function is used to disable all power resources on TPS6522x PMIC.
 *          The function is intended for testing purposes to isolate unit testing.
 *
 *  \param  pmicHandle      [IN]    PMIC interface handle
 */
void disablePmicPowerResources(Pmic_CoreHandle_t pmicHandle);

#ifdef __cplusplus
}

#endif /* __cplusplus */

#endif /* COMMON_TEST_H */
