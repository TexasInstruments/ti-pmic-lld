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
#ifndef WDG_TEST_H
#define WDG_TEST_H



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

/* ======================================================================== */
/*                           Test APIs: wdgEnable                           */
/* ======================================================================== */

#define WDG_TEST_POS_WDGENABLE() \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgEnable_enableDisable)

#define WDG_TEST_NEG_WDGENABLE() \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgEnable_nullHandle)

/* Test: TC-WDG-0041 */
#define WDG_TEST_WDGENABLE() \
    WDG_TEST_POS_WDGENABLE(); \
    WDG_TEST_NEG_WDGENABLE()

/* ======================================================================== */
/*                          Test APIs: wdgDisable                           */
/* ======================================================================== */

#define WDG_TEST_NEG_WDGDISABLE() \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgDisable_nullHandle)

/* Test: TC-WDG-0042 */
#define WDG_TEST_WDGDISABLE() \
    WDG_TEST_NEG_WDGDISABLE()

/* ======================================================================== */
/*                       Test APIs: wdgSetEnableState                       */
/* ======================================================================== */

#define WDG_TEST_NEG_WDGSETENABLESTATE() \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgSetEnableState_nullHandle)

/* Test: TC-WDG-0040 */
#define WDG_TEST_WDGSETENABLESTATE() \
    WDG_TEST_NEG_WDGSETENABLESTATE()

/* ======================================================================== */
/*                       Test APIs: wdgGetEnableState                       */
/* ======================================================================== */

#define WDG_TEST_NEG_WDGGETENABLESTATE() \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgGetEnableState_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgGetEnableState_nullParam)

/* Test: TC-WDG-0043 */
#define WDG_TEST_WDGGETENABLESTATE() \
    WDG_TEST_NEG_WDGGETENABLESTATE()

/* ======================================================================== */
/*                           Test APIs: wdgSetCfg                           */
/* ======================================================================== */

#define WDG_TEST_POS_WDGSETCFG() \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgSetCfg_resetEnable); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgSetCfg_threshold1); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgSetCfg_threshold2); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgSetCfg_longWindowCode); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgSetCfg_win1Code); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgSetCfg_win2Code); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgSetCfg_qaFdbk); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgSetCfg_qaLfsr); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgSetCfg_qaQuesSeed)

#define WDG_TEST_NEG_WDGSETCFG() \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgSetCfg_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgSetCfg_nullConfig); \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgSetCfg_invalidThreshold1); \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgSetCfg_invalidThreshold2); \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgSetCfg_invalidWin1Code); \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgSetCfg_invalidWin2Code); \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgSetCfg_invalidQaFdbk); \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgSetCfg_invalidQaLfsr); \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgSetCfg_invalidQaQuesSeed)

/* Test: TC-WDG-0044 */
#define WDG_TEST_WDGSETCFG() \
    WDG_TEST_POS_WDGSETCFG(); \
    WDG_TEST_NEG_WDGSETCFG()

/* ======================================================================== */
/*                           Test APIs: wdgGetCfg                           */
/* ======================================================================== */

#define WDG_TEST_NEG_WDGGETCFG() \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgGetCfg_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgGetCfg_nullConfig)

/* Test: TC-WDG-0045 */
#define WDG_TEST_WDGGETCFG() \
    WDG_TEST_NEG_WDGGETCFG()

/* ======================================================================== */
/*                        Test APIs: wdgSetPowerHold                        */
/* ======================================================================== */

#define WDG_TEST_POS_WDGSETPOWERHOLD() \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgSetPowerHold_enableDisable)

#define WDG_TEST_NEG_WDGSETPOWERHOLD() \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgSetPowerHold_nullHandle)

/* Test: TC-WDG-0046 */
#define WDG_TEST_WDGSETPOWERHOLD() \
    WDG_TEST_POS_WDGSETPOWERHOLD(); \
    WDG_TEST_NEG_WDGSETPOWERHOLD()

/* ======================================================================== */
/*                        Test APIs: wdgGetPowerHold                        */
/* ======================================================================== */

#define WDG_TEST_NEG_WDGGETPOWERHOLD() \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgGetPowerHold_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgGetPowerHold_nullParam)

/* Test: TC-WDG-0047 */
#define WDG_TEST_WDGGETPOWERHOLD() \
    WDG_TEST_NEG_WDGGETPOWERHOLD()

/* ======================================================================== */
/*                   Test APIs: wdgSetReturnToLongWindow                    */
/* ======================================================================== */

#define WDG_TEST_POS_WDGSETRETURNTOLONGWINDOW() \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgSetReturnToLongWindow_enableDisable)

#define WDG_TEST_NEG_WDGSETRETURNTOLONGWINDOW() \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgSetReturnToLongWindow_nullHandle)

/* Test: TC-WDG-0048 */
#define WDG_TEST_WDGSETRETURNTOLONGWINDOW() \
    WDG_TEST_POS_WDGSETRETURNTOLONGWINDOW(); \
    WDG_TEST_NEG_WDGSETRETURNTOLONGWINDOW()

/* ======================================================================== */
/*                   Test APIs: wdgGetReturnToLongWindow                    */
/* ======================================================================== */

#define WDG_TEST_NEG_WDGGETRETURNTOLONGWINDOW() \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgGetReturnToLongWindow_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgGetReturnToLongWindow_nullParam)

/* Test: TC-WDG-0049 */
#define WDG_TEST_WDGGETRETURNTOLONGWINDOW() \
    WDG_TEST_NEG_WDGGETRETURNTOLONGWINDOW()

/* ======================================================================== */
/*                        Test APIs: wdgGetErrStatus                        */
/* ======================================================================== */

#define WDG_TEST_POS_WDGGETERRORSTATUS() \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgGetErrStatus_timeout); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgGetErrStatus_longWindowTimeout); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgGetErrStatus_answerEarly); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgGetErrStatus_answerError)

#define WDG_TEST_NEG_WDGGETERRORSTATUS() \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgGetErrStatus_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgGetErrStatus_nullParam)

/* Test: TC-WDG-0057 */
#define WDG_TEST_WDGGETERRORSTATUS() \
    WDG_TEST_POS_WDGGETERRORSTATUS(); \
    WDG_TEST_NEG_WDGGETERRORSTATUS()

/* ======================================================================== */
/*                        Test APIs: wdgClrErrStatus                        */
/* ======================================================================== */

#define WDG_TEST_POS_WDGCLRERRSTATUS() \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgClrErrStatus_th1ErrorOnly); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgClrErrStatus_th2ErrorOnly); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgClrErrStatus_seqErrorOnly)

#define WDG_TEST_NEG_WDGCLRERRSTATUS() \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgClrErrStatus_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgClrErrStatus_nullParam)

/* Test: TC-WDG-0052 */
#define WDG_TEST_WDGCLRERRSTATUS() \
    WDG_TEST_POS_WDGCLRERRSTATUS(); \
    WDG_TEST_NEG_WDGCLRERRSTATUS()

/* ======================================================================== */
/*     Test APIs: wdgClrErrStatus, wdgGetErrStatus, wdgClrErrStatusAll      */
/* ======================================================================== */

#define WDG_TEST_POS_WDGCLRERRSTATUSALL() \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgClrErrStatusAll_optimization)

#define WDG_TEST_NEG_WDGCLRERRSTATUSALL() \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgClrErrStatusAll_nullHandle)

/* Test: TC-WDG-0053 */
#define WDG_TEST_WDGCLRERRSTATUSALL() \
    WDG_TEST_POS_WDGCLRERRSTATUSALL(); \
    WDG_TEST_NEG_WDGCLRERRSTATUSALL()

/* ======================================================================== */
/*                      Test APIs: wdgGetFailCntStatus                      */
/* ======================================================================== */

#define WDG_TEST_POS_WDGGETFAILCNTSTATUS() \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgGetFailCntStatus_failCntOnly); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgGetFailCntStatus_badCntOnly); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgGetFailCntStatus_goodEvent)

#define WDG_TEST_NEG_WDGGETFAILCNTSTATUS() \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgGetFailCntStatus_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgGetFailCntStatus_nullParam)

/* Test: TC-WDG-0055 */
#define WDG_TEST_WDGGETFAILCNTSTATUS() \
    WDG_TEST_POS_WDGGETFAILCNTSTATUS(); \
    WDG_TEST_NEG_WDGGETFAILCNTSTATUS()

/* ======================================================================== */
/*                       Test APIs: wdgQaWriteAnswer                        */
/* ======================================================================== */

#define WDG_TEST_POS_WDGQAWRITEANSWER() \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgQaWriteAnswer_qaFdbk0); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgQaWriteAnswer_qaFdbk1); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgQaWriteAnswer_qaFdbk2); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgQaWriteAnswer_qaFdbk3)

#define WDG_TEST_NEG_WDGQAWRITEANSWER() \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgQaWriteAnswer_nullHandle)

/* Test: TC-WDG-0051 */
#define WDG_TEST_WDGQAWRITEANSWER() \
    WDG_TEST_POS_WDGQAWRITEANSWER(); \
    WDG_TEST_NEG_WDGQAWRITEANSWER()

/* ======================================================================== */
/*     Test APIs: Pmic_wdgEnable, Pmic_wdgSetCfg, Pmic_wdgSetPowerHold,     */
/*                Pmic_wdgSetReturnToLongWindow, Pmic_wdgQaWriteAnswer      */
/* ======================================================================== */

#define WDG_TEST_POS_WDGQASEQUENCE() \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgQaSequence_noErrors); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgQaSequence_longWindowTimeout); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgQaSequence_answerEarly); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgQaSequence_sequenceError); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgQaSequence_answerError); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgQaSequence_failInt); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgQaSequence_resetInt); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgQaSequence_withIrqCallback); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgQaSequence_qaWithIrqCallback)

/* Test: TC-WDG-0056 */
#define WDG_TEST_WDGQASEQUENCE() \
    WDG_TEST_POS_WDGQASEQUENCE()

/* ======================================================================== */
/*Test APIs: wdgGetFdbkRegData, wdgExtractFdbk, wdgGetAnsCntAndQuesRegData, */
/*           wdgExtractAnsCntAndQues, wdgWriteAnswer                        */
/* ======================================================================== */

#define WDG_TEST_POS_LP8772X_SPECIFIC() \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgGetFdbkRegData_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgGetFdbkRegData_nullParam); \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgExtractFdbk_nullParam); \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgGetAnsCntAndQuesRegData_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgGetAnsCntAndQuesRegData_nullParam); \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgExtractAnsCntAndQues_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgExtractAnsCntAndQues_nullParam); \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgWriteAnswer_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgWriteAnswer_nullParam); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgWriteAnswer_success)

/* ========================================================================== */
/*                         Aggregate Test Runners                             */
/* ========================================================================== */

#define WDG_TEST_RUN_POSITIVE() \
    WDG_TEST_POS_WDGENABLE(); \
    WDG_TEST_POS_WDGSETCFG(); \
    WDG_TEST_POS_WDGSETPOWERHOLD(); \
    WDG_TEST_POS_WDGSETRETURNTOLONGWINDOW(); \
    WDG_TEST_POS_WDGGETERRORSTATUS(); \
    WDG_TEST_POS_WDGCLRERRSTATUS(); \
    WDG_TEST_POS_WDGCLRERRSTATUSALL(); \
    WDG_TEST_POS_WDGGETFAILCNTSTATUS(); \
    WDG_TEST_POS_WDGQAWRITEANSWER(); \
    WDG_TEST_POS_WDGQASEQUENCE()

#define WDG_TEST_RUN_NEGATIVE() \
    WDG_TEST_NEG_WDGENABLE(); \
    WDG_TEST_NEG_WDGDISABLE(); \
    WDG_TEST_NEG_WDGSETENABLESTATE(); \
    WDG_TEST_NEG_WDGGETENABLESTATE(); \
    WDG_TEST_NEG_WDGSETCFG(); \
    WDG_TEST_NEG_WDGGETCFG(); \
    WDG_TEST_NEG_WDGSETPOWERHOLD(); \
    WDG_TEST_NEG_WDGGETPOWERHOLD(); \
    WDG_TEST_NEG_WDGSETRETURNTOLONGWINDOW(); \
    WDG_TEST_NEG_WDGGETRETURNTOLONGWINDOW(); \
    WDG_TEST_NEG_WDGGETERRORSTATUS(); \
    WDG_TEST_NEG_WDGCLRERRSTATUS(); \
    WDG_TEST_NEG_WDGCLRERRSTATUSALL(); \
    WDG_TEST_NEG_WDGGETFAILCNTSTATUS(); \
    WDG_TEST_NEG_WDGQAWRITEANSWER(); \
    WDG_TEST_POS_LP8772X_SPECIFIC()

#define WDG_TEST_RUN_ALL() \
    WDG_TEST_RUN_POSITIVE(); \
    WDG_TEST_RUN_NEGATIVE()

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

void wdg_test(void *args);

/* Negative Test Functions */
void test_neg_wdg_wdgClrErrStatusAll_nullHandle(void);
void test_neg_wdg_wdgClrErrStatus_nullHandle(void);
void test_neg_wdg_wdgClrErrStatus_nullParam(void);
void test_neg_wdg_wdgDisable_nullHandle(void);
void test_neg_wdg_wdgEnable_nullHandle(void);
void test_neg_wdg_wdgExtractAnsCntAndQues_nullHandle(void);
void test_neg_wdg_wdgExtractAnsCntAndQues_nullParam(void);
void test_neg_wdg_wdgExtractFdbk_nullParam(void);
void test_neg_wdg_wdgGetAnsCntAndQuesRegData_nullHandle(void);
void test_neg_wdg_wdgGetAnsCntAndQuesRegData_nullParam(void);
void test_neg_wdg_wdgGetCfg_nullConfig(void);
void test_neg_wdg_wdgGetCfg_nullHandle(void);
void test_neg_wdg_wdgGetEnableState_nullHandle(void);
void test_neg_wdg_wdgGetEnableState_nullParam(void);
void test_neg_wdg_wdgGetErrStatus_nullHandle(void);
void test_neg_wdg_wdgGetErrStatus_nullParam(void);
void test_neg_wdg_wdgGetFailCntStatus_nullHandle(void);
void test_neg_wdg_wdgGetFailCntStatus_nullParam(void);
void test_neg_wdg_wdgGetFdbkRegData_nullHandle(void);
void test_neg_wdg_wdgGetFdbkRegData_nullParam(void);
void test_neg_wdg_wdgGetPowerHold_nullHandle(void);
void test_neg_wdg_wdgGetPowerHold_nullParam(void);
void test_neg_wdg_wdgGetReturnToLongWindow_nullHandle(void);
void test_neg_wdg_wdgGetReturnToLongWindow_nullParam(void);
void test_neg_wdg_wdgQaWriteAnswer_nullHandle(void);
void test_neg_wdg_wdgSetCfg_invalidQaFdbk(void);
void test_neg_wdg_wdgSetCfg_invalidQaLfsr(void);
void test_neg_wdg_wdgSetCfg_invalidQaQuesSeed(void);
void test_neg_wdg_wdgSetCfg_invalidThreshold1(void);
void test_neg_wdg_wdgSetCfg_invalidThreshold2(void);
void test_neg_wdg_wdgSetCfg_invalidWin1Code(void);
void test_neg_wdg_wdgSetCfg_invalidWin2Code(void);
void test_neg_wdg_wdgSetCfg_nullConfig(void);
void test_neg_wdg_wdgSetCfg_nullHandle(void);
void test_neg_wdg_wdgSetEnableState_nullHandle(void);
void test_neg_wdg_wdgSetPowerHold_nullHandle(void);
void test_neg_wdg_wdgSetReturnToLongWindow_nullHandle(void);
void test_neg_wdg_wdgWriteAnswer_nullHandle(void);
void test_neg_wdg_wdgWriteAnswer_nullParam(void);
void test_pos_wdg_wdgClrErrStatusAll_optimization(void);
void test_pos_wdg_wdgClrErrStatus_seqErrorOnly(void);
void test_pos_wdg_wdgClrErrStatus_th1ErrorOnly(void);
void test_pos_wdg_wdgClrErrStatus_th2ErrorOnly(void);
void test_pos_wdg_wdgEnable_enableDisable(void);
void test_pos_wdg_wdgGetErrStatus_answerEarly(void);
void test_pos_wdg_wdgGetErrStatus_answerError(void);
void test_pos_wdg_wdgGetErrStatus_longWindowTimeout(void);
void test_pos_wdg_wdgGetErrStatus_timeout(void);
void test_pos_wdg_wdgGetFailCntStatus_badCntOnly(void);
void test_pos_wdg_wdgGetFailCntStatus_failCntOnly(void);
void test_pos_wdg_wdgGetFailCntStatus_goodEvent(void);
void test_pos_wdg_wdgQaSequence_answerEarly(void);
void test_pos_wdg_wdgQaSequence_answerError(void);
void test_pos_wdg_wdgQaSequence_failInt(void);
void test_pos_wdg_wdgQaSequence_longWindowTimeout(void);
void test_pos_wdg_wdgQaSequence_noErrors(void);
void test_pos_wdg_wdgQaSequence_qaWithIrqCallback(void);
void test_pos_wdg_wdgQaSequence_resetInt(void);
void test_pos_wdg_wdgQaSequence_sequenceError(void);
void test_pos_wdg_wdgQaSequence_withIrqCallback(void);
void test_pos_wdg_wdgQaWriteAnswer_qaFdbk0(void);
void test_pos_wdg_wdgQaWriteAnswer_qaFdbk1(void);
void test_pos_wdg_wdgQaWriteAnswer_qaFdbk2(void);
void test_pos_wdg_wdgQaWriteAnswer_qaFdbk3(void);
void test_pos_wdg_wdgSetCfg_longWindowCode(void);
void test_pos_wdg_wdgSetCfg_qaFdbk(void);
void test_pos_wdg_wdgSetCfg_qaLfsr(void);
void test_pos_wdg_wdgSetCfg_qaQuesSeed(void);
void test_pos_wdg_wdgSetCfg_resetEnable(void);
void test_pos_wdg_wdgSetCfg_threshold1(void);
void test_pos_wdg_wdgSetCfg_threshold2(void);
void test_pos_wdg_wdgSetCfg_win1Code(void);
void test_pos_wdg_wdgSetCfg_win2Code(void);
void test_pos_wdg_wdgSetPowerHold_enableDisable(void);
void test_pos_wdg_wdgSetReturnToLongWindow_enableDisable(void);
void test_pos_wdg_wdgWriteAnswer_success(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* WDG_TEST_H */
