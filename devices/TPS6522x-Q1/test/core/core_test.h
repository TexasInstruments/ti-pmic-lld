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
 *    distribution and/or other materials provided with the
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
/*                       getSiliconRev Test Declarations                      */
/* ========================================================================== */

/* Negative Tests */
void test_neg_core_getSiliconRev_nullParam_handle(void);
void test_neg_core_getSiliconRev_nullParam_siliconRev(void);

/* Positive Tests */
void test_pos_core_getSiliconRev(void);

/* ========================================================================== */
/*                        getNvmRev Test Declarations                         */
/* ========================================================================== */

/* Negative Tests */
void test_neg_core_getNvmRev_nullParam_handle(void);
void test_neg_core_getNvmRev_nullParam_nvmRev(void);

/* Positive Tests */
void test_pos_core_getNvmRev(void);

/* ========================================================================== */
/*                     getRegLockState Test Declarations                      */
/* ========================================================================== */

/* Negative Tests */
void test_neg_core_getRegLockState_nullParam_handle(void);
void test_neg_core_getRegLockState_nullParam_lockState(void);

/* Positive Tests */
void test_pos_core_getRegLockState(void);

/* ========================================================================== */
/*                   setScratchPadValue Test Declarations                     */
/* ========================================================================== */

/* Negative Tests */
void test_neg_core_setScratchPadValue_nullParam_handle(void);
void test_neg_core_setScratchPadValue_outOfBounds_scratchPadRegNum(void);

/* Positive Tests */
void test_pos_core_setScratchPadValue_reg1(void);
void test_pos_core_setScratchPadValue_reg3(void);
void test_pos_core_scratchPadValue_boundary(void);

/* ========================================================================== */
/*                   getScratchPadValue Test Declarations                     */
/* ========================================================================== */

/* Negative Tests */
void test_neg_core_getScratchPadValue_outOfBounds_scratchPadRegNum(void);
void test_neg_core_getScratchPadValue_nullValue(void);

/* Positive Tests */
void test_pos_core_getScratchPadValue_reg2(void);
void test_pos_core_getScratchPadValue_reg4(void);

/* ========================================================================== */
/*                        Test Organization Macros                            */
/* ========================================================================== */

/* getSiliconRev API */
#define CORE_TEST_POS_GETSILICONREV() \
    PLATFORM_RUN_TEST(test_pos_core_getSiliconRev)

#define CORE_TEST_NEG_GETSILICONREV() \
    PLATFORM_RUN_TEST(test_neg_core_getSiliconRev_nullParam_handle); \
    PLATFORM_RUN_TEST(test_neg_core_getSiliconRev_nullParam_siliconRev)

#define CORE_TEST_GETSILICONREV() \
    CORE_TEST_POS_GETSILICONREV(); \
    CORE_TEST_NEG_GETSILICONREV()

/* getNvmRev API */
#define CORE_TEST_POS_GETNVMREV() \
    PLATFORM_RUN_TEST(test_pos_core_getNvmRev)

#define CORE_TEST_NEG_GETNVMREV() \
    PLATFORM_RUN_TEST(test_neg_core_getNvmRev_nullParam_handle); \
    PLATFORM_RUN_TEST(test_neg_core_getNvmRev_nullParam_nvmRev)

#define CORE_TEST_GETNVMREV() \
    CORE_TEST_POS_GETNVMREV(); \
    CORE_TEST_NEG_GETNVMREV()

/* getRegLockState API */
#define CORE_TEST_POS_GETREGLOCKSTATE() \
    PLATFORM_RUN_TEST(test_pos_core_getRegLockState)

#define CORE_TEST_NEG_GETREGLOCKSTATE() \
    PLATFORM_RUN_TEST(test_neg_core_getRegLockState_nullParam_handle); \
    PLATFORM_RUN_TEST(test_neg_core_getRegLockState_nullParam_lockState)

#define CORE_TEST_GETREGLOCKSTATE() \
    CORE_TEST_POS_GETREGLOCKSTATE(); \
    CORE_TEST_NEG_GETREGLOCKSTATE()

/* setScratchPadValue API */
#define CORE_TEST_POS_SETSCRATCHPADVALUE() \
    PLATFORM_RUN_TEST(test_pos_core_setScratchPadValue_reg1); \
    PLATFORM_RUN_TEST(test_pos_core_setScratchPadValue_reg3); \
    PLATFORM_RUN_TEST(test_pos_core_scratchPadValue_boundary)

#define CORE_TEST_NEG_SETSCRATCHPADVALUE() \
    PLATFORM_RUN_TEST(test_neg_core_setScratchPadValue_nullParam_handle); \
    PLATFORM_RUN_TEST(test_neg_core_setScratchPadValue_outOfBounds_scratchPadRegNum)

#define CORE_TEST_SETSCRATCHPADVALUE() \
    CORE_TEST_POS_SETSCRATCHPADVALUE(); \
    CORE_TEST_NEG_SETSCRATCHPADVALUE()

/* getScratchPadValue API */
#define CORE_TEST_POS_GETSCRATCHPADVALUE() \
    PLATFORM_RUN_TEST(test_pos_core_getScratchPadValue_reg2); \
    PLATFORM_RUN_TEST(test_pos_core_getScratchPadValue_reg4)

#define CORE_TEST_NEG_GETSCRATCHPADVALUE() \
    PLATFORM_RUN_TEST(test_neg_core_getScratchPadValue_outOfBounds_scratchPadRegNum); \
    PLATFORM_RUN_TEST(test_neg_core_getScratchPadValue_nullValue)

#define CORE_TEST_GETSCRATCHPADVALUE() \
    CORE_TEST_POS_GETSCRATCHPADVALUE(); \
    CORE_TEST_NEG_GETSCRATCHPADVALUE()

/* ========================================================================== */
/*                        Aggregate Test Macros                               */
/* ========================================================================== */

#define CORE_TEST_RUN_POSITIVE() \
    CORE_TEST_POS_GETSILICONREV(); \
    CORE_TEST_POS_GETNVMREV(); \
    CORE_TEST_POS_GETREGLOCKSTATE(); \
    CORE_TEST_POS_SETSCRATCHPADVALUE(); \
    CORE_TEST_POS_GETSCRATCHPADVALUE()

#define CORE_TEST_RUN_NEGATIVE() \
    CORE_TEST_NEG_GETSILICONREV(); \
    CORE_TEST_NEG_GETNVMREV(); \
    CORE_TEST_NEG_GETREGLOCKSTATE(); \
    CORE_TEST_NEG_SETSCRATCHPADVALUE(); \
    CORE_TEST_NEG_GETSCRATCHPADVALUE()

#define CORE_TEST_RUN_ALL() \
    CORE_TEST_RUN_POSITIVE(); \
    CORE_TEST_RUN_NEGATIVE()

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /*__CORE_TEST_H__*/
