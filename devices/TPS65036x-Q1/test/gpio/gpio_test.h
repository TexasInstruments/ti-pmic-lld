/******************************************************************************
 * Copyright (c) 2025 Texas Instruments Incorporated - http://www.ti.com
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
#ifndef GPIO_TEST_H
#define GPIO_TEST_H

/**
 * @file gpio_test.h
 * @brief Contains macros/defines and test declarations specific to testing the
 * GPIO module.
 */

/* ========================================================================== */
/*                              Include Files                                 */
/* ========================================================================== */

#include "test_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

void gpio_test(void *args);

/* Negative tests - Pmic_gpioSetCfg */
void test_negative_Pmic_gpioSetCfg_nullParam_pmicHandle(void);
void test_negative_Pmic_gpioSetCfg_nullParam_gpioCfg(void);
void test_negative_Pmic_gpioSetCfg_invalid_gpioPin(void);
void test_negative_Pmic_gpioSetCfg_invalid_validParams_zero(void);
void test_negative_Pmic_gpioSetCfg_gpio_invalid_validParams(void);
void test_negative_Pmic_gpioSetCfg_nIntGpi_invalid_validParams(void);
void test_negative_Pmic_gpioSetCfg_gpio_outOfBounds_functionality(void);
void test_negative_Pmic_gpioSetCfg_gpio_outOfBounds_polarity(void);
void test_negative_Pmic_gpioSetCfg_nIntGpi_outOfBounds_functionality(void);
void test_negative_Pmic_gpioSetCfg_nIntGpi_outOfBounds_polarity(void);
void test_negative_Pmic_gpioSetCfg_nIntGpi_outOfBounds_puPdCfg(void);
void test_negative_Pmic_gpioSetCfg_nIntGpi_outOfBounds_odPpCfg(void);

/* Negative tests - Pmic_gpioGetCfg */
void test_negative_Pmic_gpioGetCfg_nullParam_pmicHandle(void);
void test_negative_Pmic_gpioGetCfg_nullParam_gpioCfg(void);
void test_negative_Pmic_gpioGetCfg_invalid_gpioPin(void);
void test_negative_Pmic_gpioGetCfg_invalid_validParams_zero(void);
void test_negative_Pmic_gpioGetCfg_gpio_invalid_validParams(void);
void test_negative_Pmic_gpioGetCfg_nIntGpi_invalid_validParams(void);

/* Negative tests - Pmic_gpioSetActivationState */
void test_negative_Pmic_gpioSetActivationState_nullParam_pmicHandle(void);

/* Negative tests - Pmic_gpioActivate */
void test_negative_Pmic_gpioActivate_nullParam_pmicHandle(void);

/* Negative tests - Pmic_gpioDeactivate */
void test_negative_Pmic_gpioDeactivate_nullParam_pmicHandle(void);

/* Negative tests - Pmic_gpioGetActivationState */
void test_negative_Pmic_gpioGetActivationState_nullParam_pmicHandle(void);
void test_negative_Pmic_gpioGetActivationState_nullParam_activated(void);

/* Positive tests - PMIC_GPIO pin */
void test_positive_gpioSetGetCfg_gpio_functionality(void);
void test_positive_gpioSetGetCfg_gpio_polarity(void);
void test_positive_gpioSetGetCfg_gpio_all_params(void);

/* Positive tests - PMIC_NINT_GPI pin */
void test_positive_gpioSetGetCfg_nIntGpi_functionality(void);
void test_positive_gpioSetGetCfg_nIntGpi_polarity(void);
void test_positive_gpioSetGetCfg_nIntGpi_puPdCfg(void);
void test_positive_gpioSetGetCfg_nIntGpi_odPpCfg(void);
void test_positive_gpioSetGetCfg_nIntGpi_all_params(void);

/* Positive tests - Activation state */
void test_positive_gpioActivateDeactivate(void);
void test_positive_gpioSetActivationState(void);
void test_positive_gpio_nIntGpi_repeatedFunctionality(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /*__GPIO_TEST_H__*/
