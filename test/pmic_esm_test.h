/******************************************************************************
 * Copyright (c) 2024 Texas Instruments Incorporated - http://www.ti.com
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
 *  @file pmic_esm_test.h
 *
 *  @brief: This file contains PMIC ESM specific macros and definitions for the
 *          test cases
 */

#ifndef PMIC_ESM_TEST_H_
#define PMIC_ESM_TEST_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "pmic_esm.h"
#include "pmic_esm_priv.h"
#include "pmic_test_common.h"

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/*==========================================================================*/
/*                         Structures and Enums                             */
/*==========================================================================*/

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static void test_pmic_esm_startEsm_esmMcuStart(void);
static void test_pmic_esm_enableEsmPrmValTest_handle(void);
static void test_pmic_esm_setConfigurationPrmValTest_hmaxValue(void);
static void test_pmic_esm_setConfigurationPrmValTest_hminValue(void);
static void test_pmic_esm_setConfigurationPrmValTest_lmaxValue(void);
static void test_pmic_esm_setConfigurationPrmValTest_lminValue(void);
static void test_pmic_esm_getconfiguration_PrmValTest_validParams(void);
static void test_Pmic_esmGetErrCnt(void);
static void test_pmic_esm_getStatusEsmMcu(void);
void *test_pmic_ESM(void *args);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* PMIC_ESM_TEST_H_ */
