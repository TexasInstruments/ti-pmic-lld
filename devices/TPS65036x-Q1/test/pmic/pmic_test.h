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
#ifndef PMIC_TEST_H
#define PMIC_TEST_H



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
/*                         Test APIs: init, deinit                          */
/* ======================================================================== */

#define PMIC_TEST_POS_INIT() \
    PLATFORM_RUN_TEST(test_pos_pmic_init); \
    PLATFORM_RUN_TEST(test_pos_pmic_deinit); \
    PLATFORM_RUN_TEST(test_pos_pmic_init_withRetryCnt); \
    PLATFORM_RUN_TEST(test_pos_pmic_init_withRetryInterval); \
    PLATFORM_RUN_TEST(test_pos_pmic_init_withTimerWaitMs)

#define PMIC_TEST_NEG_INIT() \
    PLATFORM_RUN_TEST(test_neg_pmic_init_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_pmic_init_nullPmicCfg); \
    PLATFORM_RUN_TEST(test_neg_pmic_init_nullCommHandle); \
    PLATFORM_RUN_TEST(test_neg_pmic_init_nullIoRead); \
    PLATFORM_RUN_TEST(test_neg_pmic_init_nullIoWrite); \
    PLATFORM_RUN_TEST(test_neg_pmic_init_nullCritSecStart); \
    PLATFORM_RUN_TEST(test_neg_pmic_init_nullCritSecStop); \
    PLATFORM_RUN_TEST(test_neg_pmic_deinit_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_pmic_init_timerWaitNull); \
    PLATFORM_RUN_TEST(test_neg_pmic_init_nullIrqResponseCallback); \
    PLATFORM_RUN_TEST(test_neg_pmic_init_syncMode_nullIoRead); \
    PLATFORM_RUN_TEST(test_neg_pmic_init_syncMode_nullIoWrite); \
    PLATFORM_RUN_TEST(test_neg_pmic_init_nullCriticalSectionStart); \
    PLATFORM_RUN_TEST(test_neg_pmic_init_nullCriticalSectionStop)

/* Test: TC-PMIC-0010 */
#define PMIC_TEST_INIT() \
    PMIC_TEST_POS_INIT(); \
    PMIC_TEST_NEG_INIT()

/* ======================================================================== */
/*                   Test APIs: checkHandle, init, deinit                   */
/* ======================================================================== */

#define PMIC_TEST_NEG_CHECKHANDLE() \
    PLATFORM_RUN_TEST(test_neg_pmic_checkHandle_nullCommHandle); \
    PLATFORM_RUN_TEST(test_neg_pmic_checkHandle_nullFptrs); \
    PLATFORM_RUN_TEST(test_neg_pmic_checkHandle_nullTimerWithRetry); \
    PLATFORM_RUN_TEST(test_neg_pmic_checkHandle_invalidDrvInitStat)

/* Test: TC-PMIC-0011 */
#define PMIC_TEST_CHECKHANDLE() \
    PMIC_TEST_NEG_CHECKHANDLE()

/* ========================================================================== */
/*                        Aggregate Test Macros                               */
/* ========================================================================== */

#define PMIC_TEST_RUN_POSITIVE() \
    PMIC_TEST_POS_INIT()

#define PMIC_TEST_RUN_NEGATIVE() \
    PMIC_TEST_NEG_INIT(); \
    PMIC_TEST_NEG_CHECKHANDLE()

#define PMIC_TEST_RUN_ALL() \
    PMIC_TEST_INIT(); \
    PMIC_TEST_CHECKHANDLE()

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 * @brief PMIC test suite entry point
 * @param args Test arguments (unused)
 */
void pmic_test(void *args);

/* ========================================================================== */
/*                      init / deinit API Tests                               */
/* ========================================================================== */

/* Positive tests */
void test_pos_pmic_init(void);
void test_pos_pmic_deinit(void);
void test_pos_pmic_init_withRetryCnt(void);
void test_pos_pmic_init_withRetryInterval(void);
void test_pos_pmic_init_withTimerWaitMs(void);

/* Negative tests */
void test_neg_pmic_init_nullHandle(void);
void test_neg_pmic_init_nullPmicCfg(void);
void test_neg_pmic_init_nullCommHandle(void);
void test_neg_pmic_init_nullIoRead(void);
void test_neg_pmic_init_nullIoWrite(void);
void test_neg_pmic_init_nullCritSecStart(void);
void test_neg_pmic_init_nullCritSecStop(void);
void test_neg_pmic_deinit_nullHandle(void);
void test_neg_pmic_init_timerWaitNull(void);
void test_neg_pmic_init_nullIrqResponseCallback(void);

/* ========================================================================== */
/*                      checkHandle API Tests                                 */
/* ========================================================================== */

/* Negative tests */
void test_neg_pmic_checkHandle_nullCommHandle(void);
void test_neg_pmic_checkHandle_nullFptrs(void);
void test_neg_pmic_checkHandle_nullTimerWithRetry(void);
void test_neg_pmic_checkHandle_invalidDrvInitStat(void);

/* Coverage Tests */
void test_neg_pmic_init_syncMode_nullIoRead(void);
void test_neg_pmic_init_syncMode_nullIoWrite(void);
void test_neg_pmic_init_nullCriticalSectionStart(void);
void test_neg_pmic_init_nullCriticalSectionStop(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* PMIC_TEST_H */
