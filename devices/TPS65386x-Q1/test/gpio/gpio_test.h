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

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "../platform.h"
#include "pmic_gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 * @brief GPIO test suite entry point
 * @param args Test arguments (unused)
 */
void gpio_test(void *args);

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/* Run all GPIO tests */
#define GPIO_TEST_RUN_ALL() \
    GPIO_TEST_RUN_POSITIVE(); \
    GPIO_TEST_RUN_NEGATIVE()

/* Run all GPIO positive tests */
#define GPIO_TEST_RUN_POSITIVE() \
    PLATFORM_RUN_TEST(test_positive_gpio_setGetCfg_gpi1_esmIn); \
    PLATFORM_RUN_TEST(test_positive_gpio_setGetCfg_gpi1_wdIn); \
    PLATFORM_RUN_TEST(test_positive_gpio_setGetCfg_gpi4_comparator); \
    PLATFORM_RUN_TEST(test_positive_gpio_setGetCfg_gpi4_wdIn); \
    PLATFORM_RUN_TEST(test_positive_gpio_setGetCfg_gpi4_cosN); \
    PLATFORM_RUN_TEST(test_positive_gpio_setGetCfg_gpo1_lowLvl); \
    PLATFORM_RUN_TEST(test_positive_gpio_setGetCfg_gpo1_highLvl); \
    PLATFORM_RUN_TEST(test_positive_gpio_setGetCfg_gpo1_hiz); \
    PLATFORM_RUN_TEST(test_positive_gpio_setGetCfg_gpo1_nint); \
    PLATFORM_RUN_TEST(test_positive_gpio_setGetCfg_gpo1_enOut); \
    PLATFORM_RUN_TEST(test_positive_gpio_setGetCfg_gpo1_enOut2); \
    PLATFORM_RUN_TEST(test_positive_gpio_setGetCfg_gpo1_sinNO); \
    PLATFORM_RUN_TEST(test_positive_gpio_setGetCfg_gpo2_lowLvl); \
    PLATFORM_RUN_TEST(test_positive_gpio_setGetCfg_gpo2_highLvl); \
    PLATFORM_RUN_TEST(test_positive_gpio_setGetCfg_gpo2_hiz); \
    PLATFORM_RUN_TEST(test_positive_gpio_setGetCfg_gpo2_comp1Out); \
    PLATFORM_RUN_TEST(test_positive_gpio_setGetCfg_gpo2_enOut2); \
    PLATFORM_RUN_TEST(test_positive_gpio_setGetCfg_gpo2_syncClkOut); \
    PLATFORM_RUN_TEST(test_positive_gpio_setGetCfg_gpo2_pgood); \
    PLATFORM_RUN_TEST(test_positive_gpio_setGetCfg_gpo2_sinPO); \
    PLATFORM_RUN_TEST(test_positive_gpio_setGetCfg_gpo3_lowLvl); \
    PLATFORM_RUN_TEST(test_positive_gpio_setGetCfg_gpo3_highLvl); \
    PLATFORM_RUN_TEST(test_positive_gpio_setGetCfg_gpo3_hiz); \
    PLATFORM_RUN_TEST(test_positive_gpio_setGetCfg_gpo3_pgood); \
    PLATFORM_RUN_TEST(test_positive_gpio_setGetCfg_gpo3_comp2Out); \
    PLATFORM_RUN_TEST(test_positive_gpio_setGetCfg_gpo3_enOut2); \
    PLATFORM_RUN_TEST(test_positive_gpio_setGetCfg_gpo3_safeOut2); \
    PLATFORM_RUN_TEST(test_positive_gpio_setGetCfg_gpo3_cosPO); \
    PLATFORM_RUN_TEST(test_positive_gpio_setGetCfg_gpo4_lowLvl); \
    PLATFORM_RUN_TEST(test_positive_gpio_setGetCfg_gpo4_highLvl); \
    PLATFORM_RUN_TEST(test_positive_gpio_setGetCfg_gpo4_hiz); \
    PLATFORM_RUN_TEST(test_positive_gpio_setGetCfg_gpo4_safeOut2); \
    PLATFORM_RUN_TEST(test_positive_gpio_setGetCfg_gpo4_enOut); \
    PLATFORM_RUN_TEST(test_positive_gpio_setGetCfg_gpo4_nint); \
    PLATFORM_RUN_TEST(test_positive_gpio_setGetCfg_gpo4_pgood); \
    PLATFORM_RUN_TEST(test_positive_gpio_setGetCfg_gpo4_cosNO); \
    PLATFORM_RUN_TEST(test_positive_gpio_getOutputValue_allGpos); \
    PLATFORM_RUN_TEST(test_positive_gpio_gpo1_hiz_duplicate); \
    PLATFORM_RUN_TEST(test_positive_gpio_safeOutSetGet); \
    PLATFORM_RUN_TEST(test_positive_gpio_safeOut_individual)

/* Run all GPIO negative tests */
#define GPIO_TEST_RUN_NEGATIVE() \
    PLATFORM_RUN_TEST(test_negative_Pmic_gpioSetCfg_nullParam_handle); \
    PLATFORM_RUN_TEST(test_negative_Pmic_gpioSetCfg_nullParam_gpioCfg); \
    PLATFORM_RUN_TEST(test_negative_Pmic_gpioSetCfg_invalidParam_validParams); \
    PLATFORM_RUN_TEST(test_negative_Pmic_gpioGetCfg_nullParam_handle); \
    PLATFORM_RUN_TEST(test_negative_Pmic_gpioGetCfg_nullParam_gpioCfg); \
    PLATFORM_RUN_TEST(test_negative_Pmic_gpioGetCfg_invalidParam_validParams); \
    PLATFORM_RUN_TEST(test_negative_Pmic_gpioGetOutputValue_nullParam_handle); \
    PLATFORM_RUN_TEST(test_negative_Pmic_gpioGetOutputValue_nullParam_high); \
    PLATFORM_RUN_TEST(test_negative_Pmic_gpioGetOutputValue_invalidParam_gpo); \
    PLATFORM_RUN_TEST(test_negative_Pmic_gpioSetCfg_invalidValue_gpi1); \
    PLATFORM_RUN_TEST(test_negative_Pmic_gpioSetCfg_invalidValue_gpi4); \
    PLATFORM_RUN_TEST(test_negative_Pmic_gpioSetCfg_invalidValue_gpo1); \
    PLATFORM_RUN_TEST(test_negative_Pmic_gpioSetCfg_invalidValue_gpo2); \
    PLATFORM_RUN_TEST(test_negative_Pmic_gpioSetCfg_invalidValue_gpo3); \
    PLATFORM_RUN_TEST(test_negative_Pmic_gpioSetCfg_invalidValue_gpo4); \
    PLATFORM_RUN_TEST(test_negative_Pmic_gpioSetSafeOutCfg_nullParam_handle); \
    PLATFORM_RUN_TEST(test_negative_Pmic_gpioGetSafeOutCfg_nullParam_handle); \
    PLATFORM_RUN_TEST(test_negative_Pmic_gpioGetSafeOutCfg_nullParam_config); \
    PLATFORM_RUN_TEST(test_negative_Pmic_gpioSetSafeOutCfg_invalidParam_validParams)

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* Positive test function declarations */
void test_positive_gpio_setGetCfg_gpi1_esmIn(void);
void test_positive_gpio_setGetCfg_gpi1_wdIn(void);
void test_positive_gpio_setGetCfg_gpi4_comparator(void);
void test_positive_gpio_setGetCfg_gpi4_wdIn(void);
void test_positive_gpio_setGetCfg_gpi4_cosN(void);
void test_positive_gpio_setGetCfg_gpo1_lowLvl(void);
void test_positive_gpio_setGetCfg_gpo1_highLvl(void);
void test_positive_gpio_setGetCfg_gpo1_hiz(void);
void test_positive_gpio_setGetCfg_gpo1_nint(void);
void test_positive_gpio_setGetCfg_gpo1_enOut(void);
void test_positive_gpio_setGetCfg_gpo1_enOut2(void);
void test_positive_gpio_setGetCfg_gpo1_sinNO(void);
void test_positive_gpio_setGetCfg_gpo2_lowLvl(void);
void test_positive_gpio_setGetCfg_gpo2_highLvl(void);
void test_positive_gpio_setGetCfg_gpo2_hiz(void);
void test_positive_gpio_setGetCfg_gpo2_comp1Out(void);
void test_positive_gpio_setGetCfg_gpo2_enOut2(void);
void test_positive_gpio_setGetCfg_gpo2_syncClkOut(void);
void test_positive_gpio_setGetCfg_gpo2_pgood(void);
void test_positive_gpio_setGetCfg_gpo2_sinPO(void);
void test_positive_gpio_setGetCfg_gpo3_lowLvl(void);
void test_positive_gpio_setGetCfg_gpo3_highLvl(void);
void test_positive_gpio_setGetCfg_gpo3_hiz(void);
void test_positive_gpio_setGetCfg_gpo3_pgood(void);
void test_positive_gpio_setGetCfg_gpo3_comp2Out(void);
void test_positive_gpio_setGetCfg_gpo3_enOut2(void);
void test_positive_gpio_setGetCfg_gpo3_safeOut2(void);
void test_positive_gpio_setGetCfg_gpo3_cosPO(void);
void test_positive_gpio_setGetCfg_gpo4_lowLvl(void);
void test_positive_gpio_setGetCfg_gpo4_highLvl(void);
void test_positive_gpio_setGetCfg_gpo4_hiz(void);
void test_positive_gpio_setGetCfg_gpo4_safeOut2(void);
void test_positive_gpio_setGetCfg_gpo4_enOut(void);
void test_positive_gpio_setGetCfg_gpo4_nint(void);
void test_positive_gpio_setGetCfg_gpo4_pgood(void);
void test_positive_gpio_setGetCfg_gpo4_cosNO(void);
void test_positive_gpio_getOutputValue_allGpos(void);
void test_positive_gpio_gpo1_hiz_duplicate(void);
void test_positive_gpio_safeOutSetGet(void);
void test_positive_gpio_safeOut_individual(void);

/* Negative test function declarations */
void test_negative_Pmic_gpioSetCfg_nullParam_handle(void);
void test_negative_Pmic_gpioSetCfg_nullParam_gpioCfg(void);
void test_negative_Pmic_gpioSetCfg_invalidParam_validParams(void);
void test_negative_Pmic_gpioGetCfg_nullParam_handle(void);
void test_negative_Pmic_gpioGetCfg_nullParam_gpioCfg(void);
void test_negative_Pmic_gpioGetCfg_invalidParam_validParams(void);
void test_negative_Pmic_gpioGetOutputValue_nullParam_handle(void);
void test_negative_Pmic_gpioGetOutputValue_nullParam_high(void);
void test_negative_Pmic_gpioGetOutputValue_invalidParam_gpo(void);
void test_negative_Pmic_gpioSetCfg_invalidValue_gpi1(void);
void test_negative_Pmic_gpioSetCfg_invalidValue_gpi4(void);
void test_negative_Pmic_gpioSetCfg_invalidValue_gpo1(void);
void test_negative_Pmic_gpioSetCfg_invalidValue_gpo2(void);
void test_negative_Pmic_gpioSetCfg_invalidValue_gpo3(void);
void test_negative_Pmic_gpioSetCfg_invalidValue_gpo4(void);
void test_negative_Pmic_gpioSetSafeOutCfg_nullParam_handle(void);
void test_negative_Pmic_gpioGetSafeOutCfg_nullParam_handle(void);
void test_negative_Pmic_gpioGetSafeOutCfg_nullParam_config(void);
void test_negative_Pmic_gpioSetSafeOutCfg_invalidParam_validParams(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* GPIO_TEST_H */
