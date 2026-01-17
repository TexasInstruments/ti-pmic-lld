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

/* ========================================================================== */
/*                  setScratchPadValue API Tests                              */
/* ========================================================================== */
void test_neg_core_setScratchPadValue_nullHandle(void);
void test_neg_core_setScratchPadValue_outOfBounds(void);

/* ========================================================================== */
/*                  getScratchPadValue API Tests                              */
/* ========================================================================== */
void test_pos_core_getScratchPadValue_reg1to4(void);
void test_neg_core_getScratchPadValue_nullHandle(void);
void test_neg_core_getScratchPadValue_nullValue(void);
void test_neg_core_getScratchPadValue_outOfBounds(void);

/* ========================================================================== */
/*                  setRegLockState API Tests                                 */
/* ========================================================================== */
void test_pos_core_setRegLockState_enableDisable(void);
void test_neg_core_setRegLockState_nullHandle(void);

/* ========================================================================== */
/*                  getRegLockState API Tests                                 */
/* ========================================================================== */
void test_neg_core_getRegLockState_nullHandle(void);
void test_neg_core_getRegLockState_nullLockState(void);

/* ========================================================================== */
/*                  configCrcEnable API Tests                                 */
/* ========================================================================== */
void test_pos_core_configCrcEnable_enableOnly(void);
void test_pos_core_configCrcEnable_recalculate(void);
void test_neg_core_configCrcEnable_nullHandle(void);
void test_neg_core_configCrcEnable_alreadyEnabled(void);
void test_neg_core_configCrcEnable_calcBitHigh(void);
void test_neg_core_configCrcEnable_crcMismatch(void);
void test_neg_core_configCrcEnable_error(void);

/* ========================================================================== */
/*                  configCrcDisable API Tests                                */
/* ========================================================================== */
void test_pos_core_configCrcDisable_disable(void);
void test_neg_core_configCrcDisable_nullHandle(void);

/* ========================================================================== */
/*                  getConfigCrcStatus API Tests                              */
/* ========================================================================== */
void test_neg_core_getConfigCrcStatus_nullHandle(void);
void test_neg_core_getConfigCrcStatus_nullStatus(void);

/* ========================================================================== */
/*                  configCrcCalculate API Tests                              */
/* ========================================================================== */
void test_neg_core_configCrcCalculate_nullHandle(void);
void test_neg_core_configCrcCalculate_ioFailure(void);

/* ========================================================================== */
/*               configCrcGetFromDevice API Tests                             */
/* ========================================================================== */
void test_pos_core_configCrcGetFromDevice_getCrc(void);
void test_neg_core_configCrcGetFromDevice_nullHandle(void);
void test_neg_core_configCrcGetFromDevice_nullCrc(void);

/* ========================================================================== */
/*                  init API Tests                                            */
/* ========================================================================== */
void test_neg_core_init_invalidDeviceType(void);

/* ========================================================================== */
/*                  Error Status Tests                                        */
/* ========================================================================== */
void test_pos_core_errStatus_multipleErrors(void);
void test_pos_core_errStatus_specificError(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /*__CORE_TEST_H__*/
