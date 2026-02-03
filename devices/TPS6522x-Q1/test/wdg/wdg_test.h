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

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/* ======================================================================== */
/*                       Test APIs: wdgSetEnableState                       */
/* ======================================================================== */

#define WDG_TEST_POS_WDGSETENABLESTATE() \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgSetEnableState_enableDisable)

#define WDG_TEST_NEG_WDGSETENABLESTATE() \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgSetEnableState_nullHandle)

/* Test: TC-WDG-0040 */
#define WDG_TEST_WDGSETENABLESTATE() \
    WDG_TEST_POS_WDGSETENABLESTATE(); \
    WDG_TEST_NEG_WDGSETENABLESTATE()

/* ======================================================================== */
/*                       Test APIs: wdgGetEnableState                       */
/* ======================================================================== */

#define WDG_TEST_NEG_WDGGETENABLESTATE() \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgGetEnableState_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgGetEnableState_nullIsEnabled)

/* Test: TC-WDG-0043 */
#define WDG_TEST_WDGGETENABLESTATE() \
    WDG_TEST_NEG_WDGGETENABLESTATE()

/* ======================================================================== */
/*                           Test APIs: wdgSetCfg                           */
/* ======================================================================== */

#define WDG_TEST_POS_WDGSETCFG() \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgSetCfg_rstEn); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgSetCfg_mode); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgSetCfg_win1Code); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgSetCfg_win2Code); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgSetCfg_longWinCode); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgSetCfg_qaFdbk); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgSetCfg_qaLfsr); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgSetCfg_qaSeed); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgSetCfg_failThr); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgSetCfg_rstThr); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgSetCfg_cntSel); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgSetCfg_combinedConfiguration)

#define WDG_TEST_NEG_WDGSETCFG() \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgSetCfg_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgSetCfg_nullWdgCfg); \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgSetCfg_invalidMode); \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgSetCfg_invalidWin1Code); \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgSetCfg_invalidWin2Code); \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgSetCfg_invalidQaFdbk); \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgSetCfg_invalidQaLfsr); \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgSetCfg_invalidQaSeed); \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgSetCfg_invalidFailThr); \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgSetCfg_invalidRstThr); \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgSetCfg_invalidCntSel)

/* Test: TC-WDG-0044 */
#define WDG_TEST_WDGSETCFG() \
    WDG_TEST_POS_WDGSETCFG(); \
    WDG_TEST_NEG_WDGSETCFG()

/* ======================================================================== */
/*                           Test APIs: wdgGetCfg                           */
/* ======================================================================== */

#define WDG_TEST_NEG_WDGGETCFG() \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgGetCfg_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgGetCfg_nullWdgCfg)

/* Test: TC-WDG-0045 */
#define WDG_TEST_WDGGETCFG() \
    WDG_TEST_NEG_WDGGETCFG()

/* ======================================================================== */
/*                        Test APIs: wdgSetPowerHold                        */
/* ======================================================================== */

#define WDG_TEST_POS_WDGSETPOWERHOLD() \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgSetPowerHold_powerHold)

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
    PLATFORM_RUN_TEST(test_neg_wdg_wdgGetPowerHold_nullIsEnabled)

/* Test: TC-WDG-0047 */
#define WDG_TEST_WDGGETPOWERHOLD() \
    WDG_TEST_NEG_WDGGETPOWERHOLD()

/* ======================================================================== */
/*                   Test APIs: wdgSetReturnToLongWindow                    */
/* ======================================================================== */

#define WDG_TEST_POS_WDGSETRETURNTOLONGWINDOW() \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgSetReturnToLongWindow_returnToLongWindow)

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
    PLATFORM_RUN_TEST(test_neg_wdg_wdgGetReturnToLongWindow_nullIsEnabled)

/* Test: TC-WDG-0049 */
#define WDG_TEST_WDGGETRETURNTOLONGWINDOW() \
    WDG_TEST_NEG_WDGGETRETURNTOLONGWINDOW()

/* ======================================================================== */
/*                       Test APIs: wdgQaWriteAnswer                        */
/* ======================================================================== */

#define WDG_TEST_POS_WDGQAWRITEANSWER() \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgQaWriteAnswer_qaSequenceCorrectAnswers); \
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
/*                        Test APIs: wdgClrErrStatus                        */
/* ======================================================================== */

#define WDG_TEST_POS_WDGCLRERRSTATUS() \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgClrErrStatus_th1ErrorOnly); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgClrErrStatus_th2ErrorOnly); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgClrErrStatus_seqErrorOnly); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgClrErrStatus_clrAnswErr); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgClrErrStatus_clrAnswEarlyErr); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgClrErrStatus_clrTrigEarlyErr); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgClrErrStatus_clrTimeoutErr); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgClrErrStatus_clrLongWinTimeoutErr)

#define WDG_TEST_NEG_WDGCLRERRSTATUS() \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgClrErrStatus_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgClrErrStatus_nullErrStatus)

/* Test: TC-WDG-0052 */
#define WDG_TEST_WDGCLRERRSTATUS() \
    WDG_TEST_POS_WDGCLRERRSTATUS(); \
    WDG_TEST_NEG_WDGCLRERRSTATUS()

/* ======================================================================== */
/*                      Test APIs: wdgClrErrStatusAll                       */
/* ======================================================================== */

#define WDG_TEST_NEG_WDGCLRERRSTATUSALL() \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgClrErrStatusAll_nullHandle)

/* Test: TC-WDG-0053 */
#define WDG_TEST_WDGCLRERRSTATUSALL() \
    WDG_TEST_NEG_WDGCLRERRSTATUSALL()

/* ======================================================================== */
/*                        Test APIs: wdgGetErrStatus                        */
/* ======================================================================== */

#define WDG_TEST_POS_WDGGETERRSTATUS() \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgGetErrStatus_errorStatusGetClear)

#define WDG_TEST_NEG_WDGGETERRSTATUS() \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgGetErrStatus_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgGetErrStatus_nullErrStatus)

/* Test: TC-WDG-0054 */
#define WDG_TEST_WDGGETERRSTATUS() \
    WDG_TEST_POS_WDGGETERRSTATUS(); \
    WDG_TEST_NEG_WDGGETERRSTATUS()

/* ======================================================================== */
/*                      Test APIs: wdgGetFailCntStatus                      */
/* ======================================================================== */

#define WDG_TEST_POS_WDGGETFAILCNTSTATUS() \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgGetFailCntStatus_failCounterStatus); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgGetFailCntStatus_failCntOnly); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgGetFailCntStatus_badCntOnly)

#define WDG_TEST_NEG_WDGGETFAILCNTSTATUS() \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgGetFailCntStatus_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgGetFailCntStatus_nullFailCntStatus)

/* Test: TC-WDG-0055 */
#define WDG_TEST_WDGGETFAILCNTSTATUS() \
    WDG_TEST_POS_WDGGETFAILCNTSTATUS(); \
    WDG_TEST_NEG_WDGGETFAILCNTSTATUS()

/* ======================================================================== */
/*                        Test APIs: Pmic_wdgSetCfg                         */
/* ======================================================================== */

#define WDG_TEST_POS_WDGSETENDRVSEL() \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgSetEnDrvSel_setEnDrvSel)

/* Test: TC-WDG-0022 */
#define WDG_TEST_WDGSETENDRVSEL() \
    WDG_TEST_POS_WDGSETENDRVSEL()

/* ======================================================================== */
/*                        Test APIs: Pmic_wdgGetCfg                         */
/* ======================================================================== */

#define WDG_TEST_POS_WDGGETENDRVSEL() \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgGetEnDrvSel_getEnDrvSel)

/* Test: TC-WDG-0023 */
#define WDG_TEST_WDGGETENDRVSEL() \
    WDG_TEST_POS_WDGGETENDRVSEL()

/* ========================================================================== */
/*                          Aggregate Test Runners                            */
/* ========================================================================== */

#define WDG_TEST_RUN_POSITIVE() \
    WDG_TEST_POS_WDGSETENABLESTATE(); \
    WDG_TEST_POS_WDGSETCFG(); \
    WDG_TEST_POS_WDGSETPOWERHOLD(); \
    WDG_TEST_POS_WDGSETRETURNTOLONGWINDOW(); \
    WDG_TEST_POS_WDGQAWRITEANSWER(); \
    WDG_TEST_POS_WDGCLRERRSTATUS(); \
    WDG_TEST_POS_WDGGETERRSTATUS(); \
    WDG_TEST_POS_WDGGETFAILCNTSTATUS(); \
    WDG_TEST_POS_WDGSETENDRVSEL(); \
    WDG_TEST_POS_WDGGETENDRVSEL()

#define WDG_TEST_RUN_NEGATIVE() \
    WDG_TEST_NEG_WDGSETENABLESTATE(); \
    WDG_TEST_NEG_WDGGETENABLESTATE(); \
    WDG_TEST_NEG_WDGSETCFG(); \
    WDG_TEST_NEG_WDGGETCFG(); \
    WDG_TEST_NEG_WDGSETPOWERHOLD(); \
    WDG_TEST_NEG_WDGGETPOWERHOLD(); \
    WDG_TEST_NEG_WDGSETRETURNTOLONGWINDOW(); \
    WDG_TEST_NEG_WDGGETRETURNTOLONGWINDOW(); \
    WDG_TEST_NEG_WDGQAWRITEANSWER(); \
    WDG_TEST_NEG_WDGCLRERRSTATUS(); \
    WDG_TEST_NEG_WDGCLRERRSTATUSALL(); \
    WDG_TEST_NEG_WDGGETERRSTATUS(); \
    WDG_TEST_NEG_WDGGETFAILCNTSTATUS()

#define WDG_TEST_RUN_ALL() \
    WDG_TEST_RUN_POSITIVE(); \
    WDG_TEST_RUN_NEGATIVE()

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 * @brief Entry point for watchdog module tests.
 *
 * @param args [IN] Optional arguments (unused).
 */
void wdg_test(void *args);

/* Negative test functions */
void test_neg_wdg_wdgSetEnableState_nullHandle(void);
void test_neg_wdg_wdgGetEnableState_nullHandle(void);
void test_neg_wdg_wdgGetEnableState_nullIsEnabled(void);
void test_neg_wdg_wdgSetCfg_nullHandle(void);
void test_neg_wdg_wdgSetCfg_nullWdgCfg(void);
void test_neg_wdg_wdgSetCfg_invalidMode(void);
void test_neg_wdg_wdgSetCfg_invalidWin1Code(void);
void test_neg_wdg_wdgSetCfg_invalidWin2Code(void);
void test_neg_wdg_wdgSetCfg_invalidQaFdbk(void);
void test_neg_wdg_wdgSetCfg_invalidQaLfsr(void);
void test_neg_wdg_wdgSetCfg_invalidQaSeed(void);
void test_neg_wdg_wdgSetCfg_invalidFailThr(void);
void test_neg_wdg_wdgSetCfg_invalidRstThr(void);
void test_neg_wdg_wdgSetCfg_invalidCntSel(void);
void test_neg_wdg_wdgGetCfg_nullHandle(void);
void test_neg_wdg_wdgGetCfg_nullWdgCfg(void);
void test_neg_wdg_wdgSetPowerHold_nullHandle(void);
void test_neg_wdg_wdgGetPowerHold_nullHandle(void);
void test_neg_wdg_wdgGetPowerHold_nullIsEnabled(void);
void test_neg_wdg_wdgSetReturnToLongWindow_nullHandle(void);
void test_neg_wdg_wdgGetReturnToLongWindow_nullHandle(void);
void test_neg_wdg_wdgGetReturnToLongWindow_nullIsEnabled(void);
void test_neg_wdg_wdgQaWriteAnswer_nullHandle(void);
void test_neg_wdg_wdgClrErrStatus_nullHandle(void);
void test_neg_wdg_wdgClrErrStatus_nullErrStatus(void);
void test_neg_wdg_wdgClrErrStatusAll_nullHandle(void);
void test_neg_wdg_wdgGetErrStatus_nullHandle(void);
void test_neg_wdg_wdgGetErrStatus_nullErrStatus(void);
void test_neg_wdg_wdgGetFailCntStatus_nullHandle(void);
void test_neg_wdg_wdgGetFailCntStatus_nullFailCntStatus(void);

/* Positive test functions */
void test_pos_wdg_wdgSetEnableState_enableDisable(void);
void test_pos_wdg_wdgSetPowerHold_powerHold(void);
void test_pos_wdg_wdgSetReturnToLongWindow_returnToLongWindow(void);
void test_pos_wdg_wdgSetCfg_rstEn(void);
void test_pos_wdg_wdgSetCfg_mode(void);
void test_pos_wdg_wdgSetCfg_win1Code(void);
void test_pos_wdg_wdgSetCfg_win2Code(void);
void test_pos_wdg_wdgSetCfg_longWinCode(void);
void test_pos_wdg_wdgSetCfg_qaFdbk(void);
void test_pos_wdg_wdgSetCfg_qaLfsr(void);
void test_pos_wdg_wdgSetCfg_qaSeed(void);
void test_pos_wdg_wdgSetCfg_failThr(void);
void test_pos_wdg_wdgSetCfg_rstThr(void);
void test_pos_wdg_wdgSetCfg_cntSel(void);
void test_pos_wdg_wdgQaWriteAnswer_qaSequenceCorrectAnswers(void);
void test_pos_wdg_wdgGetErrStatus_errorStatusGetClear(void);
void test_pos_wdg_wdgGetFailCntStatus_failCounterStatus(void);
void test_pos_wdg_wdgSetCfg_combinedConfiguration(void);
void test_pos_wdg_wdgQaWriteAnswer_qaFdbk0(void);
void test_pos_wdg_wdgQaWriteAnswer_qaFdbk1(void);
void test_pos_wdg_wdgQaWriteAnswer_qaFdbk2(void);
void test_pos_wdg_wdgQaWriteAnswer_qaFdbk3(void);
void test_pos_wdg_wdgClrErrStatus_th1ErrorOnly(void);
void test_pos_wdg_wdgClrErrStatus_th2ErrorOnly(void);
void test_pos_wdg_wdgClrErrStatus_seqErrorOnly(void);
void test_pos_wdg_wdgGetFailCntStatus_failCntOnly(void);
void test_pos_wdg_wdgGetFailCntStatus_badCntOnly(void);
void test_pos_wdg_wdgSetEnDrvSel_setEnDrvSel(void);
void test_pos_wdg_wdgGetEnDrvSel_getEnDrvSel(void);
void test_pos_wdg_wdgClrErrStatus_clrAnswErr(void);
void test_pos_wdg_wdgClrErrStatus_clrAnswEarlyErr(void);
void test_pos_wdg_wdgClrErrStatus_clrTrigEarlyErr(void);
void test_pos_wdg_wdgClrErrStatus_clrTimeoutErr(void);
void test_pos_wdg_wdgClrErrStatus_clrLongWinTimeoutErr(void);

#ifdef __cplusplus
}
#endif

#endif /* WDG_TEST_H */
