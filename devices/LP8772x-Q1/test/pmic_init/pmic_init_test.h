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
#ifndef __PMIC_INIT_TEST_H__
#define __PMIC_INIT_TEST_H__

/**
 * @file pmic_init_test.h
 * @brief Contains macros/defines and test declarations specific to testing PMIC
 * Init.
 */

/* ========================================================================== */
/*                              Include Files                                 */
/* ========================================================================== */

#include "platform.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

void pmic_init_test(void *args);

void test_negative_Pmic_init_nullParam_handle(void);
void test_negative_Pmic_init_nullParam_coreCfg(void);
void test_negative_Pmic_init_nullParam_coreCfg_pCommHandle(void);
void test_negative_Pmic_init_nullParam_coreCfg_pQACommHandle(void);
void test_negative_Pmic_init_nullParam_coreCfg_pFnPmicCommIoRd(void);
void test_negative_Pmic_init_nullParam_coreCfg_pFnPmicCommIoWr(void);
void test_negative_Pmic_init_nullParam_coreCfg_pFnPmicCritSecStart(void);
void test_negative_Pmic_init_nullParam_coreCfg_pFnPmicCritSecStop(void);
void test_negative_Pmic_init_nullParam_coreCfg_pFnPmicPseudoIrq(void);
void test_negative_Pmic_init_incorrect_coreCfg_instType(void);
void test_negative_Pmic_init_incorrect__coreCfg_pmicDeviceType(void);
void test_negative_Pmic_init_incorrect_coreCfg_commMode(void);
void test_negative_Pmic_deinit_nullParam_handle(void);
void test_negative_Pmic_checkPmicCoreHandle_nullParam_handle(void);
void test_negative_Pmic_checkPmicCoreHandle_nullParam_pCommHandle(void);
void test_negative_Pmic_checkPmicCoreHandle_nullParam_pFnPmicCommIoRd(void);
void test_negative_Pmic_checkPmicCoreHandle_incorrect_drvInitStatus(void);
void test_positive_Pmic_init(void);
void test_positive_Pmic_checkPmicCoreHandle(void);
void test_positive_Pmic_deinit(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /*__PMIC_INIT_TEST_H__*/
