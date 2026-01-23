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
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* ========================================================================== */
/*             API-Specific Test Macros - configCrcCalculate                  */
/* ========================================================================== */

#define CORE_TEST_POS_CONFIGCRCCALCULATE() \
    PLATFORM_RUN_TEST(test_pos_core_configCrcEnable_recalculate)

#define CORE_TEST_NEG_CONFIGCRCCALCULATE() \
    PLATFORM_RUN_TEST(test_neg_core_configCrcCalculate_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_core_configCrcCalculate_ioFailure)

#define CORE_TEST_CONFIGCRCCALCULATE() \
    CORE_TEST_POS_CONFIGCRCCALCULATE(); \
    CORE_TEST_NEG_CONFIGCRCCALCULATE()

/* ========================================================================== */
/*             API-Specific Test Macros - configCrcDisable                    */
/* ========================================================================== */

#define CORE_TEST_POS_CONFIGCRCDISABLE() \
    PLATFORM_RUN_TEST(test_pos_core_configCrcDisable_disable)

#define CORE_TEST_NEG_CONFIGCRCDISABLE() \
    PLATFORM_RUN_TEST(test_neg_core_configCrcDisable_nullHandle)

#define CORE_TEST_CONFIGCRCDISABLE() \
    CORE_TEST_POS_CONFIGCRCDISABLE(); \
    CORE_TEST_NEG_CONFIGCRCDISABLE()

/* ========================================================================== */
/*             API-Specific Test Macros - configCrcEnable                     */
/* ========================================================================== */

#define CORE_TEST_POS_CONFIGCRCENABLE() \
    PLATFORM_RUN_TEST(test_pos_core_configCrcEnable_enableOnly)

#define CORE_TEST_NEG_CONFIGCRCENABLE() \
    PLATFORM_RUN_TEST(test_neg_core_configCrcEnable_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_core_configCrcEnable_alreadyEnabled); \
    PLATFORM_RUN_TEST(test_neg_core_configCrcEnable_calcBitHigh); \
    PLATFORM_RUN_TEST(test_neg_core_configCrcEnable_crcMismatch); \
    PLATFORM_RUN_TEST(test_neg_core_configCrcEnable_error)

#define CORE_TEST_CONFIGCRCENABLE() \
    CORE_TEST_POS_CONFIGCRCENABLE(); \
    CORE_TEST_NEG_CONFIGCRCENABLE()

/* ========================================================================== */
/*           API-Specific Test Macros - configCrcGetFromDevice                */
/* ========================================================================== */

#define CORE_TEST_POS_CONFIGCRCGETFROMDEVICE() \
    PLATFORM_RUN_TEST(test_pos_core_configCrcGetFromDevice_getCrc)

#define CORE_TEST_NEG_CONFIGCRCGETFROMDEVICE() \
    PLATFORM_RUN_TEST(test_neg_core_configCrcGetFromDevice_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_core_configCrcGetFromDevice_nullCrc)

#define CORE_TEST_CONFIGCRCGETFROMDEVICE() \
    CORE_TEST_POS_CONFIGCRCGETFROMDEVICE(); \
    CORE_TEST_NEG_CONFIGCRCGETFROMDEVICE()

/* ========================================================================== */
/*             API-Specific Test Macros - getConfigCrcStatus                  */
/* ========================================================================== */

#define CORE_TEST_POS_GETCONFIGCRCSTATUS() \
    /* Positive tests combined with other CRC tests */

#define CORE_TEST_NEG_GETCONFIGCRCSTATUS() \
    PLATFORM_RUN_TEST(test_neg_core_getConfigCrcStatus_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_core_getConfigCrcStatus_nullStatus)

#define CORE_TEST_GETCONFIGCRCSTATUS() \
    CORE_TEST_POS_GETCONFIGCRCSTATUS(); \
    CORE_TEST_NEG_GETCONFIGCRCSTATUS()

/* ========================================================================== */
/*             API-Specific Test Macros - getRegLockState                     */
/* ========================================================================== */

#define CORE_TEST_POS_GETREGLOCKSTATE() \
    /* Positive tests combined with setRegLockState */

#define CORE_TEST_NEG_GETREGLOCKSTATE() \
    PLATFORM_RUN_TEST(test_neg_core_getRegLockState_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_core_getRegLockState_nullLockState)

#define CORE_TEST_GETREGLOCKSTATE() \
    CORE_TEST_POS_GETREGLOCKSTATE(); \
    CORE_TEST_NEG_GETREGLOCKSTATE()

/* ========================================================================== */
/*             API-Specific Test Macros - getScratchPadValue                  */
/* ========================================================================== */

#define CORE_TEST_POS_GETSCRATCHPADVALUE() \
    PLATFORM_RUN_TEST(test_pos_core_getScratchPadValue_reg1to4)

#define CORE_TEST_NEG_GETSCRATCHPADVALUE() \
    PLATFORM_RUN_TEST(test_neg_core_getScratchPadValue_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_core_getScratchPadValue_nullValue); \
    PLATFORM_RUN_TEST(test_neg_core_getScratchPadValue_outOfBounds)

#define CORE_TEST_GETSCRATCHPADVALUE() \
    CORE_TEST_POS_GETSCRATCHPADVALUE(); \
    CORE_TEST_NEG_GETSCRATCHPADVALUE()

/* ========================================================================== */
/*             API-Specific Test Macros - init                                */
/* ========================================================================== */

#define CORE_TEST_POS_INIT() \
    /* Positive init tests in main setup */

#define CORE_TEST_NEG_INIT() \
    PLATFORM_RUN_TEST(test_neg_core_init_invalidDeviceType)

#define CORE_TEST_INIT() \
    CORE_TEST_POS_INIT(); \
    CORE_TEST_NEG_INIT()

/* ========================================================================== */
/*             API-Specific Test Macros - setRegLockState                     */
/* ========================================================================== */

#define CORE_TEST_POS_SETREGLOCKSTATE() \
    PLATFORM_RUN_TEST(test_pos_core_setRegLockState_enableDisable)

#define CORE_TEST_NEG_SETREGLOCKSTATE() \
    PLATFORM_RUN_TEST(test_neg_core_setRegLockState_nullHandle)

#define CORE_TEST_SETREGLOCKSTATE() \
    CORE_TEST_POS_SETREGLOCKSTATE(); \
    CORE_TEST_NEG_SETREGLOCKSTATE()

/* ========================================================================== */
/*             API-Specific Test Macros - setScratchPadValue                  */
/* ========================================================================== */

#define CORE_TEST_POS_SETSCRATCHPADVALUE() \
    /* Positive tests combined with getScratchPadValue */

#define CORE_TEST_NEG_SETSCRATCHPADVALUE() \
    PLATFORM_RUN_TEST(test_neg_core_setScratchPadValue_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_core_setScratchPadValue_outOfBounds)

#define CORE_TEST_SETSCRATCHPADVALUE() \
    CORE_TEST_POS_SETSCRATCHPADVALUE(); \
    CORE_TEST_NEG_SETSCRATCHPADVALUE()

/* ========================================================================== */
/*                         Aggregate Test Runners                             */
/* ========================================================================== */

#define CORE_TEST_RUN_POSITIVE() \
    CORE_TEST_POS_CONFIGCRCCALCULATE(); \
    CORE_TEST_POS_CONFIGCRCDISABLE(); \
    CORE_TEST_POS_CONFIGCRCENABLE(); \
    CORE_TEST_POS_CONFIGCRCGETFROMDEVICE(); \
    CORE_TEST_POS_GETCONFIGCRCSTATUS(); \
    CORE_TEST_POS_GETREGLOCKSTATE(); \
    CORE_TEST_POS_GETSCRATCHPADVALUE(); \
    CORE_TEST_POS_INIT(); \
    CORE_TEST_POS_SETREGLOCKSTATE(); \
    CORE_TEST_POS_SETSCRATCHPADVALUE(); \
    PLATFORM_RUN_TEST(test_pos_core_errStatus_multipleErrors); \
    PLATFORM_RUN_TEST(test_pos_core_errStatus_specificError)

#define CORE_TEST_RUN_NEGATIVE() \
    CORE_TEST_NEG_CONFIGCRCCALCULATE(); \
    CORE_TEST_NEG_CONFIGCRCDISABLE(); \
    CORE_TEST_NEG_CONFIGCRCENABLE(); \
    CORE_TEST_NEG_CONFIGCRCGETFROMDEVICE(); \
    CORE_TEST_NEG_GETCONFIGCRCSTATUS(); \
    CORE_TEST_NEG_GETREGLOCKSTATE(); \
    CORE_TEST_NEG_GETSCRATCHPADVALUE(); \
    CORE_TEST_NEG_INIT(); \
    CORE_TEST_NEG_SETREGLOCKSTATE(); \
    CORE_TEST_NEG_SETSCRATCHPADVALUE()

#define CORE_TEST_RUN_ALL() \
    CORE_TEST_RUN_POSITIVE(); \
    CORE_TEST_RUN_NEGATIVE()

#define CORE_TEST_CALCUL_CONFIG_CRC_1_REG TEST_REG_CALCUL_CRC_1
#define CORE_TEST_CALCUL_CONFIG_CRC_2_REG TEST_REG_CALCUL_CRC_2


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
