/******************************************************************************
 * Copyright (c) 2026 Texas Instruments Incorporated - http://www.ti.com
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

#include "test_utils.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/* ========================================================================== */
/*            API-Specific Test Macros - gpioSetCfg, gpioGetCfg               */
/* ========================================================================== */
#define GPIO_TEST_POS_GPIOSETGETCFG_GPIO() \
    PLATFORM_RUN_TEST(test_pos_gpio_gpioSetGetCfg_gpio_functionality); \
    PLATFORM_RUN_TEST(test_pos_gpio_gpioSetGetCfg_gpio_polarity); \
    PLATFORM_RUN_TEST(test_pos_gpio_gpioSetGetCfg_gpio_all_params)

#define GPIO_TEST_NEG_GPIOSETGETCFG_GPIO() \
    PLATFORM_RUN_TEST(test_neg_gpio_gpioSetCfg_gpio_invalidValidParams); \
    PLATFORM_RUN_TEST(test_neg_gpio_gpioSetCfg_gpio_outOfBounds_functionality); \
    PLATFORM_RUN_TEST(test_neg_gpio_gpioSetCfg_gpio_outOfBounds_polarity); \
    PLATFORM_RUN_TEST(test_neg_gpio_gpioGetCfg_gpio_invalidValidParams)

#define GPIO_TEST_GPIOSETGETCFG_GPIO() \
    GPIO_TEST_NEG_GPIOSETGETCFG_GPIO(); \
    GPIO_TEST_POS_GPIOSETGETCFG_GPIO()

/* ========================================================================== */
/*            API-Specific Test Macros - gpioSetCfg, gpioGetCfg               */
/* ========================================================================== */

#define GPIO_TEST_POS_GPIOSETGETCFG_NINTGPI() \
    PLATFORM_RUN_TEST(test_pos_gpio_gpioSetGetCfg_nIntGpi_functionality); \
    PLATFORM_RUN_TEST(test_pos_gpio_gpioSetGetCfg_nIntGpi_polarity); \
    PLATFORM_RUN_TEST(test_pos_gpio_gpioSetGetCfg_nIntGpi_puPdCfg); \
    PLATFORM_RUN_TEST(test_pos_gpio_gpioSetGetCfg_nIntGpi_odPpCfg); \
    PLATFORM_RUN_TEST(test_pos_gpio_gpioSetGetCfg_nIntGpi_all_params)

#define GPIO_TEST_NEG_GPIOSETGETCFG_NINTGPI() \
    PLATFORM_RUN_TEST(test_neg_gpio_gpioSetCfg_nIntGpi_invalidValidParams); \
    PLATFORM_RUN_TEST(test_neg_gpio_gpioSetCfg_nIntGpi_outOfBounds_functionality); \
    PLATFORM_RUN_TEST(test_neg_gpio_gpioSetCfg_nIntGpi_outOfBounds_polarity); \
    PLATFORM_RUN_TEST(test_neg_gpio_gpioSetCfg_nIntGpi_outOfBounds_puPdCfg); \
    PLATFORM_RUN_TEST(test_neg_gpio_gpioSetCfg_nIntGpi_outOfBounds_odPpCfg); \
    PLATFORM_RUN_TEST(test_neg_gpio_gpioGetCfg_nIntGpi_invalidValidParams)

#define GPIO_TEST_GPIOSETGETCFG_NINTGPI() \
    GPIO_TEST_NEG_GPIOSETGETCFG_NINTGPI(); \
    GPIO_TEST_POS_GPIOSETGETCFG_NINTGPI()

/* ========================================================================== */
/*            API-Specific Test Macros - gpioSetCfg, gpioGetCfg               */
/* ========================================================================== */
#define GPIO_TEST_POS_GPIOSETGETCFG_COMMON() \
    PLATFORM_RUN_TEST(test_pos_gpio_gpio_nIntGpi_repeatedFunctionality)

#define GPIO_TEST_NEG_GPIOSETGETCFG_COMMON() \
    PLATFORM_RUN_TEST(test_neg_gpio_gpioSetCfg_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_gpio_gpioSetCfg_nullGpioCfg); \
    PLATFORM_RUN_TEST(test_neg_gpio_gpioSetCfg_invalidGpioPin); \
    PLATFORM_RUN_TEST(test_neg_gpio_gpioSetCfg_zeroValidParams); \
    PLATFORM_RUN_TEST(test_neg_gpio_gpioGetCfg_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_gpio_gpioGetCfg_nullGpioCfg); \
    PLATFORM_RUN_TEST(test_neg_gpio_gpioGetCfg_invalidGpioPin); \
    PLATFORM_RUN_TEST(test_neg_gpio_gpioGetCfg_zeroValidParams)

#define GPIO_TEST_GPIOSETGETCFG_COMMON() \
    GPIO_TEST_NEG_GPIOSETGETCFG_COMMON(); \
    GPIO_TEST_POS_GPIOSETGETCFG_COMMON()

/* ======================================================================================================= */
/* API-Specific Test Macros - gpioActivate, gpioDeactivate, gpioSetActivationState, gpioGetActivationState */
/* ======================================================================================================= */
#define GPIO_TEST_POS_GPIOACTIVATION() \
    PLATFORM_RUN_TEST(test_pos_gpio_gpioActivateDeactivate); \
    PLATFORM_RUN_TEST(test_pos_gpio_gpioSetActivationState)

#define GPIO_TEST_NEG_GPIOACTIVATION() \
    PLATFORM_RUN_TEST(test_neg_gpio_gpioSetActivationState_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_gpio_gpioActivate_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_gpio_gpioDeactivate_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_gpio_gpioGetActivationState_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_gpio_gpioGetActivationState_nullActivated)

#define GPIO_TEST_GPIOACTIVATION() \
    GPIO_TEST_NEG_GPIOACTIVATION(); \
    GPIO_TEST_POS_GPIOACTIVATION()

/* ========================================================================== */
/*                     Aggregate Test Macros                                  */
/* ========================================================================== */

/* Run all GPIO positive tests */
#define GPIO_TEST_RUN_POSITIVE() \
    GPIO_TEST_POS_GPIOSETGETCFG_GPIO(); \
    GPIO_TEST_POS_GPIOSETGETCFG_NINTGPI(); \
    GPIO_TEST_POS_GPIOSETGETCFG_COMMON(); \
    GPIO_TEST_POS_GPIOACTIVATION()

/* Run all GPIO negative tests */
#define GPIO_TEST_RUN_NEGATIVE() \
    GPIO_TEST_NEG_GPIOSETGETCFG_GPIO(); \
    GPIO_TEST_NEG_GPIOSETGETCFG_NINTGPI(); \
    GPIO_TEST_NEG_GPIOSETGETCFG_COMMON(); \
    GPIO_TEST_NEG_GPIOACTIVATION()

/* Run all GPIO tests */
#define GPIO_TEST_RUN_ALL() \
    GPIO_TEST_GPIOSETGETCFG_GPIO(); \
    GPIO_TEST_GPIOSETGETCFG_NINTGPI(); \
    GPIO_TEST_GPIOSETGETCFG_COMMON(); \
    GPIO_TEST_GPIOACTIVATION()

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

void gpio_test(void *args);

/* Negative tests - Pmic_gpioSetCfg */
void test_neg_gpio_gpioSetCfg_nullHandle(void);
void test_neg_gpio_gpioSetCfg_nullGpioCfg(void);
void test_neg_gpio_gpioSetCfg_invalidGpioPin(void);
void test_neg_gpio_gpioSetCfg_zeroValidParams(void);
void test_neg_gpio_gpioSetCfg_gpio_invalidValidParams(void);
void test_neg_gpio_gpioSetCfg_nIntGpi_invalidValidParams(void);
void test_neg_gpio_gpioSetCfg_gpio_outOfBounds_functionality(void);
void test_neg_gpio_gpioSetCfg_gpio_outOfBounds_polarity(void);
void test_neg_gpio_gpioSetCfg_nIntGpi_outOfBounds_functionality(void);
void test_neg_gpio_gpioSetCfg_nIntGpi_outOfBounds_polarity(void);
void test_neg_gpio_gpioSetCfg_nIntGpi_outOfBounds_puPdCfg(void);
void test_neg_gpio_gpioSetCfg_nIntGpi_outOfBounds_odPpCfg(void);

/* Negative tests - Pmic_gpioGetCfg */
void test_neg_gpio_gpioGetCfg_nullHandle(void);
void test_neg_gpio_gpioGetCfg_nullGpioCfg(void);
void test_neg_gpio_gpioGetCfg_invalidGpioPin(void);
void test_neg_gpio_gpioGetCfg_zeroValidParams(void);
void test_neg_gpio_gpioGetCfg_gpio_invalidValidParams(void);
void test_neg_gpio_gpioGetCfg_nIntGpi_invalidValidParams(void);

/* Negative tests - Pmic_gpioSetActivationState */
void test_neg_gpio_gpioSetActivationState_nullHandle(void);

/* Negative tests - Pmic_gpioActivate */
void test_neg_gpio_gpioActivate_nullHandle(void);

/* Negative tests - Pmic_gpioDeactivate */
void test_neg_gpio_gpioDeactivate_nullHandle(void);

/* Negative tests - Pmic_gpioGetActivationState */
void test_neg_gpio_gpioGetActivationState_nullHandle(void);
void test_neg_gpio_gpioGetActivationState_nullActivated(void);

/* Positive tests - PMIC_GPIO pin */
void test_pos_gpio_gpioSetGetCfg_gpio_functionality(void);
void test_pos_gpio_gpioSetGetCfg_gpio_polarity(void);
void test_pos_gpio_gpioSetGetCfg_gpio_all_params(void);

/* Positive tests - PMIC_NINT_GPI pin */
void test_pos_gpio_gpioSetGetCfg_nIntGpi_functionality(void);
void test_pos_gpio_gpioSetGetCfg_nIntGpi_polarity(void);
void test_pos_gpio_gpioSetGetCfg_nIntGpi_puPdCfg(void);
void test_pos_gpio_gpioSetGetCfg_nIntGpi_odPpCfg(void);
void test_pos_gpio_gpioSetGetCfg_nIntGpi_all_params(void);

/* Positive tests - Activation state */
void test_pos_gpio_gpioActivateDeactivate(void);
void test_pos_gpio_gpioSetActivationState(void);
void test_pos_gpio_gpio_nIntGpi_repeatedFunctionality(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /*__GPIO_TEST_H__*/
