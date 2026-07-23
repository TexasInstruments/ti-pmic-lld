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
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/* ======================================================================== */
/*                         Test APIs: getSiliconRev                         */
/* ======================================================================== */
#define CORE_TEST_POS_COREGETSILREV() \
    PLATFORM_RUN_TEST(test_pos_core_coreGetSilRev)

#define CORE_TEST_NEG_COREGETSILREV() \
    PLATFORM_RUN_TEST(test_neg_core_coreGetSilRev_nullParam_handle); \
    PLATFORM_RUN_TEST(test_neg_core_coreGetSilRev_nullParam_siliconRev)

/* Test: TC-CORE-0018 */
#define CORE_TEST_COREGETSILREV() \
    CORE_TEST_POS_COREGETSILREV(); \
    CORE_TEST_NEG_COREGETSILREV()

/* ======================================================================== */
/*                           Test APIs: getNvmRev                           */
/* ======================================================================== */
#define CORE_TEST_POS_COREGETNVMREV() \
    PLATFORM_RUN_TEST(test_pos_core_coreGetNvmRev)

#define CORE_TEST_NEG_COREGETNVMREV() \
    PLATFORM_RUN_TEST(test_neg_core_coreGetNvmRev_nullParam_handle); \
    PLATFORM_RUN_TEST(test_neg_core_coreGetNvmRev_nullParam_nvmRev)

/* Test: TC-CORE-0019 */
#define CORE_TEST_COREGETNVMREV() \
    CORE_TEST_POS_COREGETNVMREV(); \
    CORE_TEST_NEG_COREGETNVMREV()

/* ======================================================================== */
/*                        Test APIs: getRegLockState                        */
/* ======================================================================== */
#define CORE_TEST_POS_COREGETREGLOCKSTATE() \
    PLATFORM_RUN_TEST(test_pos_core_coreGetRegLockState)

#define CORE_TEST_NEG_COREGETREGLOCKSTATE() \
    PLATFORM_RUN_TEST(test_neg_core_coreGetRegLockState_nullParam_handle); \
    PLATFORM_RUN_TEST(test_neg_core_coreGetRegLockState_nullParam_lockState)

/* Test: TC-CORE-0020 */
#define CORE_TEST_COREGETREGLOCKSTATE() \
    CORE_TEST_POS_COREGETREGLOCKSTATE(); \
    CORE_TEST_NEG_COREGETREGLOCKSTATE()

/* ======================================================================== */
/*                      Test APIs: setRegLockState                          */
/* ======================================================================== */
#define CORE_TEST_POS_CORESETREGLOCKSTATE() \
    PLATFORM_RUN_TEST(test_pos_core_coreSetRegLockState_lock); \
    PLATFORM_RUN_TEST(test_pos_core_coreSetRegLockState_unlock)

#define CORE_TEST_NEG_CORESETREGLOCKSTATE() \
    PLATFORM_RUN_TEST(test_neg_core_coreSetRegLockState_nullHandle)

/* Test: TC-CORE-0059 */
#define CORE_TEST_CORESETREGLOCKSTATE() \
    CORE_TEST_POS_CORESETREGLOCKSTATE(); \
    CORE_TEST_NEG_CORESETREGLOCKSTATE()

/* ======================================================================== */
/*                      Test APIs: setScratchPadValue                       */
/* ======================================================================== */
#define CORE_TEST_POS_CORESETSCRATCHPADVALUE() \
    PLATFORM_RUN_TEST(test_pos_core_coreSetScratchPadValue_reg1); \
    PLATFORM_RUN_TEST(test_pos_core_coreSetScratchPadValue_reg3); \
    PLATFORM_RUN_TEST(test_pos_core_scratchPadValue_boundary)

#define CORE_TEST_NEG_CORESETSCRATCHPADVALUE() \
    PLATFORM_RUN_TEST(test_neg_core_coreSetScratchPadValue_nullParam_handle); \
    PLATFORM_RUN_TEST(test_neg_core_coreSetScratchPadValue_outOfBounds_scratchPadRegNum)

/* Test: TC-CORE-0021 */
#define CORE_TEST_CORESETSCRATCHPADVALUE() \
    CORE_TEST_POS_CORESETSCRATCHPADVALUE(); \
    CORE_TEST_NEG_CORESETSCRATCHPADVALUE()

/* ======================================================================== */
/*                      Test APIs: getScratchPadValue                       */
/* ======================================================================== */
#define CORE_TEST_POS_COREGETSCRATCHPADVALUE() \
    PLATFORM_RUN_TEST(test_pos_core_coreGetScratchPadValue_reg2); \
    PLATFORM_RUN_TEST(test_pos_core_coreGetScratchPadValue_reg4)

#define CORE_TEST_NEG_COREGETSCRATCHPADVALUE() \
    PLATFORM_RUN_TEST(test_neg_core_coreGetScratchPadValue_outOfBounds_scratchPadRegNum); \
    PLATFORM_RUN_TEST(test_neg_core_coreGetScratchPadValue_nullValue); \
    PLATFORM_RUN_TEST(test_neg_core_getScratchPadValue_invalidHandle)

/* Test: TC-CORE-0022 */
#define CORE_TEST_COREGETSCRATCHPADVALUE() \
    CORE_TEST_POS_COREGETSCRATCHPADVALUE(); \
    CORE_TEST_NEG_COREGETSCRATCHPADVALUE()

/* ======================================================================== */
/*              Test APIs: getScratchPadValue (additional positive)         */
/* ======================================================================== */
#define CORE_TEST_POS_GETSCRATCHPADVALUE() \
    PLATFORM_RUN_TEST(test_pos_core_getScratchPadValue_validReg)

/* ========================================================================== */
/*          API-Specific Test Macros - validatePmicHandle                     */
/* ========================================================================== */
#define CORE_TEST_POS_VALIDATEPMICHANDLE() \
    PLATFORM_RUN_TEST(test_pos_core_validatePmicHandle_validCriticalSection)

#define CORE_TEST_NEG_VALIDATEPMICHANDLE() \
    PLATFORM_RUN_TEST(test_neg_core_validatePmicHandle_nullCritSecStart); \
    PLATFORM_RUN_TEST(test_neg_core_validatePmicHandle_nullCritSecStop)

#define CORE_TEST_VALIDATEPMICHANDLE() \
    CORE_TEST_POS_VALIDATEPMICHANDLE(); \
    CORE_TEST_NEG_VALIDATEPMICHANDLE()

/* ========================================================================== */
/*                        Aggregate Test Macros                               */
/* ========================================================================== */

#define CORE_TEST_RUN_POSITIVE() \
    CORE_TEST_POS_COREGETSILREV(); \
    CORE_TEST_POS_COREGETNVMREV(); \
    CORE_TEST_POS_COREGETREGLOCKSTATE(); \
    CORE_TEST_POS_CORESETREGLOCKSTATE(); \
    CORE_TEST_POS_CORESETSCRATCHPADVALUE(); \
    CORE_TEST_POS_COREGETSCRATCHPADVALUE(); \
    CORE_TEST_POS_GETSCRATCHPADVALUE(); \
    CORE_TEST_POS_VALIDATEPMICHANDLE()

#define CORE_TEST_RUN_NEGATIVE() \
    CORE_TEST_NEG_COREGETSILREV(); \
    CORE_TEST_NEG_COREGETNVMREV(); \
    CORE_TEST_NEG_COREGETREGLOCKSTATE(); \
    CORE_TEST_NEG_CORESETREGLOCKSTATE(); \
    CORE_TEST_NEG_CORESETSCRATCHPADVALUE(); \
    CORE_TEST_NEG_COREGETSCRATCHPADVALUE(); \
    CORE_TEST_NEG_VALIDATEPMICHANDLE()

#define CORE_TEST_RUN_ALL() \
    CORE_TEST_RUN_POSITIVE(); \
    CORE_TEST_RUN_NEGATIVE()

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

void core_test(void *args);

/* Negative test functions */
void test_neg_core_coreGetSilRev_nullParam_handle(void);
void test_neg_core_coreGetSilRev_nullParam_siliconRev(void);
void test_neg_core_coreGetNvmRev_nullParam_handle(void);
void test_neg_core_coreGetNvmRev_nullParam_nvmRev(void);
void test_neg_core_coreGetRegLockState_nullParam_handle(void);
void test_neg_core_coreGetRegLockState_nullParam_lockState(void);
void test_neg_core_coreSetScratchPadValue_nullParam_handle(void);
void test_neg_core_coreSetScratchPadValue_outOfBounds_scratchPadRegNum(void);
void test_neg_core_coreGetScratchPadValue_outOfBounds_scratchPadRegNum(void);
void test_neg_core_coreGetScratchPadValue_nullValue(void);
void test_neg_core_getScratchPadValue_invalidHandle(void);
void test_neg_core_validatePmicHandle_nullCritSecStart(void);
void test_neg_core_validatePmicHandle_nullCritSecStop(void);

/* Positive test functions */
void test_pos_core_coreGetSilRev(void);
void test_pos_core_coreGetNvmRev(void);
void test_pos_core_coreGetRegLockState(void);
void test_pos_core_coreSetScratchPadValue_reg1(void);
void test_pos_core_coreGetScratchPadValue_reg2(void);
void test_pos_core_coreSetScratchPadValue_reg3(void);
void test_pos_core_coreGetScratchPadValue_reg4(void);
void test_pos_core_scratchPadValue_boundary(void);
void test_pos_core_validatePmicHandle_validCriticalSection(void);

/* setRegLockState test functions */
void test_pos_core_coreSetRegLockState_lock(void);
void test_pos_core_coreSetRegLockState_unlock(void);
void test_neg_core_coreSetRegLockState_nullHandle(void);
void test_pos_core_getScratchPadValue_validReg(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /*__CORE_TEST_H__*/
