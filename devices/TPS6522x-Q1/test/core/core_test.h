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
#ifndef CORE_TEST_H
#define CORE_TEST_H



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

void core_test(void *args);

void test_negative_Pmic_getSiliconRev_nullParam_handle(void);
void test_negative_Pmic_getSiliconRev_nullParam_siliconRev(void);
void test_negative_Pmic_getNvmRev_nullParam_handle(void);
void test_negative_Pmic_getNvmRev_nullParam_nvmRev(void);
void test_negative_Pmic_getRegLockState_nullParam_handle(void);
void test_negative_Pmic_getRegLockState_nullParam_lockState(void);
void test_negative_Pmic_setScratchPadValue_nullParam_handle(void);
void test_negative_Pmic_setScratchPadValue_outOfBounds_scratchPadRegNum(void);
void test_negative_Pmic_getScratchPadValue_outOfBounds_scratchPadRegNum(void);
void test_negative_Pmic_getScratchPadValue_nullValue(void);
void test_positive_getSiliconRev(void);
void test_positive_getNvmRev(void);
void test_positive_getRegLockState(void);
void test_positive_setScratchPadValue_reg1(void);
void test_positive_getScratchPadValue_reg2(void);
void test_positive_setScratchPadValue_reg3(void);
void test_positive_getScratchPadValue_reg4(void);
void test_positive_scratchPadValue_boundary(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /*__CORE_TEST_H__*/
