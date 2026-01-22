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
/*                    Positive Test Declarations - gpioSetCfg/gpioGetCfg     */
/* ========================================================================== */

/* GPI1 Tests */
void test_pos_gpio_gpioSetGetCfg_gpi1_esmIn(void);
void test_pos_gpio_gpioSetGetCfg_gpi1_wdIn(void);

/* GPI4 Tests */
void test_pos_gpio_gpioSetGetCfg_gpi4_comparator(void);
void test_pos_gpio_gpioSetGetCfg_gpi4_wdIn(void);
void test_pos_gpio_gpioSetGetCfg_gpi4_cosN(void);

/* GPO1 Tests */
void test_pos_gpio_gpioSetGetCfg_gpo1_lowLvl(void);
void test_pos_gpio_gpioSetGetCfg_gpo1_highLvl(void);
void test_pos_gpio_gpioSetGetCfg_gpo1_hiz(void);
void test_pos_gpio_gpioSetGetCfg_gpo1_nint(void);
void test_pos_gpio_gpioSetGetCfg_gpo1_enOut(void);
void test_pos_gpio_gpioSetGetCfg_gpo1_enOut2(void);
void test_pos_gpio_gpioSetGetCfg_gpo1_sinNO(void);
void test_pos_gpio_gpioGpo1Hiz_duplicate(void);

/* GPO2 Tests */
void test_pos_gpio_gpioSetGetCfg_gpo2_lowLvl(void);
void test_pos_gpio_gpioSetGetCfg_gpo2_highLvl(void);
void test_pos_gpio_gpioSetGetCfg_gpo2_hiz(void);
void test_pos_gpio_gpioSetGetCfg_gpo2_comp1Out(void);
void test_pos_gpio_gpioSetGetCfg_gpo2_enOut2(void);
void test_pos_gpio_gpioSetGetCfg_gpo2_syncClkOut(void);
void test_pos_gpio_gpioSetGetCfg_gpo2_pgood(void);
void test_pos_gpio_gpioSetGetCfg_gpo2_sinPO(void);

/* GPO3 Tests */
void test_pos_gpio_gpioSetGetCfg_gpo3_lowLvl(void);
void test_pos_gpio_gpioSetGetCfg_gpo3_highLvl(void);
void test_pos_gpio_gpioSetGetCfg_gpo3_hiz(void);
void test_pos_gpio_gpioSetGetCfg_gpo3_pgood(void);
void test_pos_gpio_gpioSetGetCfg_gpo3_comp2Out(void);
void test_pos_gpio_gpioSetGetCfg_gpo3_enOut2(void);
void test_pos_gpio_gpioSetGetCfg_gpo3_safeOut2(void);
void test_pos_gpio_gpioSetGetCfg_gpo3_cosPO(void);

/* GPO4 Tests */
void test_pos_gpio_gpioSetGetCfg_gpo4_lowLvl(void);
void test_pos_gpio_gpioSetGetCfg_gpo4_highLvl(void);
void test_pos_gpio_gpioSetGetCfg_gpo4_hiz(void);
void test_pos_gpio_gpioSetGetCfg_gpo4_safeOut2(void);
void test_pos_gpio_gpioSetGetCfg_gpo4_enOut(void);
void test_pos_gpio_gpioSetGetCfg_gpo4_nint(void);
void test_pos_gpio_gpioSetGetCfg_gpo4_pgood(void);
void test_pos_gpio_gpioSetGetCfg_gpo4_cosNO(void);

/* ========================================================================== */
/*                    Positive Test Declarations - gpioGetOutputValue        */
/* ========================================================================== */

void test_pos_gpio_gpioGetOutputValue_allGpos(void);

/* ========================================================================== */
/*              Positive Test Declarations - gpioSetSafeOutCfg/gpioGetSafeOutCfg */
/* ========================================================================== */

void test_pos_gpio_gpioSafeOutSetGet(void);
void test_pos_gpio_gpioSafeOut_individual(void);

/* ========================================================================== */
/*                    Negative Test Declarations - gpioSetCfg/gpioGetCfg     */
/* ========================================================================== */

/* NULL Parameter Tests */
void test_neg_gpio_gpioSetCfg_nullParam_handle(void);
void test_neg_gpio_gpioSetCfg_nullParam_gpioCfg(void);
void test_neg_gpio_gpioSetCfg_invalidParam_validParams(void);
void test_neg_gpio_gpioGetCfg_nullParam_handle(void);
void test_neg_gpio_gpioGetCfg_nullParam_gpioCfg(void);
void test_neg_gpio_gpioGetCfg_invalidParam_validParams(void);

/* Invalid Value Tests */
void test_neg_gpio_gpioSetCfg_invalidValue_gpi1(void);
void test_neg_gpio_gpioSetCfg_invalidValue_gpi4(void);
void test_neg_gpio_gpioSetCfg_invalidValue_gpo1(void);
void test_neg_gpio_gpioSetCfg_invalidValue_gpo2(void);
void test_neg_gpio_gpioSetCfg_invalidValue_gpo3(void);
void test_neg_gpio_gpioSetCfg_invalidValue_gpo4(void);

/* ========================================================================== */
/*                    Negative Test Declarations - gpioGetOutputValue        */
/* ========================================================================== */

void test_neg_gpio_gpioGetOutputValue_nullParam_handle(void);
void test_neg_gpio_gpioGetOutputValue_nullParam_high(void);
void test_neg_gpio_gpioGetOutputValue_invalidParam_gpo(void);

/* ========================================================================== */
/*              Negative Test Declarations - gpioSetSafeOutCfg/gpioGetSafeOutCfg */
/* ========================================================================== */

void test_neg_gpio_gpioSetSafeOutCfg_nullParam_handle(void);
void test_neg_gpio_gpioSetSafeOutCfg_invalidParam_validParams(void);
void test_neg_gpio_gpioGetSafeOutCfg_nullParam_handle(void);
void test_neg_gpio_gpioGetSafeOutCfg_nullParam_config(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* GPIO_TEST_H */
