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

#ifndef WDG_TEST_MACROS_H
#define WDG_TEST_MACROS_H

/* ========================================================================== */
/*                API-Specific Test Macros - wdgSetEnableState                */
/* ========================================================================== */

#define WDG_TEST_NEG_WDGSETENABLESTATE() \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgSetEnableState_nullHandle)

#define WDG_TEST_WDGSETENABLESTATE() \
    WDG_TEST_NEG_WDGSETENABLESTATE()

/* ========================================================================== */
/*                    API-Specific Test Macros - wdgEnable                    */
/* ========================================================================== */

#define WDG_TEST_POS_WDGENABLE() \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgEnable_enableDisable)

#define WDG_TEST_NEG_WDGENABLE() \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgEnable_nullHandle)

#define WDG_TEST_WDGENABLE() \
    WDG_TEST_POS_WDGENABLE(); \
    WDG_TEST_NEG_WDGENABLE()

/* ========================================================================== */
/*                   API-Specific Test Macros - wdgDisable                    */
/* ========================================================================== */

#define WDG_TEST_NEG_WDGDISABLE() \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgDisable_nullHandle)

#define WDG_TEST_WDGDISABLE() \
    WDG_TEST_NEG_WDGDISABLE()

/* ========================================================================== */
/*                API-Specific Test Macros - wdgGetEnableState                */
/* ========================================================================== */

#define WDG_TEST_NEG_WDGGETENABLESTATE() \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgGetEnableState_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgGetEnableState_nullParam)

#define WDG_TEST_WDGGETENABLESTATE() \
    WDG_TEST_NEG_WDGGETENABLESTATE()

/* ========================================================================== */
/*                    API-Specific Test Macros - wdgSetCfg                    */
/* ========================================================================== */

#define WDG_TEST_POS_WDGSETCFG() \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgSetCfg_rstEn); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgSetCfg_mode); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgSetCfg_trigSel); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgSetCfg_failThr); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgSetCfg_rstThr); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgSetCfg_longWinDuration); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgSetCfg_win1Duration); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgSetCfg_win2Duration); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgSetCfg_qaFdbk); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgSetCfg_qaLfsr); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgSetCfg_qaSeed); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgSetCfg_qaFdbk1); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgSetCfg_qaFdbk2); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgSetCfg_qaFdbk3)

#define WDG_TEST_NEG_WDGSETCFG() \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgSetCfg_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgSetCfg_nullConfig); \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgSetCfg_invalidParam); \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgSetCfg_invalidMode); \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgSetCfg_invalidTrigSel); \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgSetCfg_invalidFailThr); \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgSetCfg_invalidRstThr); \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgSetCfg_invalidWin1Duration); \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgSetCfg_invalidWin2Duration); \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgSetCfg_invalidQaFdbk); \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgSetCfg_invalidQaLfsr); \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgSetCfg_invalidQaSeed)

#define WDG_TEST_WDGSETCFG() \
    WDG_TEST_POS_WDGSETCFG(); \
    WDG_TEST_NEG_WDGSETCFG()

/* ========================================================================== */
/*                    API-Specific Test Macros - wdgGetCfg                    */
/* ========================================================================== */

#define WDG_TEST_NEG_WDGGETCFG() \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgGetCfg_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgGetCfg_nullConfig); \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgGetCfg_invalidParam)

#define WDG_TEST_WDGGETCFG() \
    WDG_TEST_NEG_WDGGETCFG()

/* ========================================================================== */
/*                 API-Specific Test Macros - wdgSetPowerHold                 */
/* ========================================================================== */

#define WDG_TEST_POS_WDGSETPOWERHOLD() \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgSetPowerHold_enableDisable)

#define WDG_TEST_NEG_WDGSETPOWERHOLD() \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgSetPowerHold_nullHandle)

#define WDG_TEST_WDGSETPOWERHOLD() \
    WDG_TEST_POS_WDGSETPOWERHOLD(); \
    WDG_TEST_NEG_WDGSETPOWERHOLD()

/* ========================================================================== */
/*                 API-Specific Test Macros - wdgGetPowerHold                 */
/* ========================================================================== */

#define WDG_TEST_NEG_WDGGETPOWERHOLD() \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgGetPowerHold_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgGetPowerHold_nullParam)

#define WDG_TEST_WDGGETPOWERHOLD() \
    WDG_TEST_NEG_WDGGETPOWERHOLD()

/* ========================================================================== */
/*            API-Specific Test Macros - wdgSetReturnToLongWindow             */
/* ========================================================================== */

#define WDG_TEST_POS_WDGSETRETURNTOLONGWINDOW() \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgSetReturnToLongWindow_enableDisable)

#define WDG_TEST_NEG_WDGSETRETURNTOLONGWINDOW() \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgSetReturnToLongWindow_nullHandle)

#define WDG_TEST_WDGSETRETURNTOLONGWINDOW() \
    WDG_TEST_POS_WDGSETRETURNTOLONGWINDOW(); \
    WDG_TEST_NEG_WDGSETRETURNTOLONGWINDOW()

/* ========================================================================== */
/*            API-Specific Test Macros - wdgGetReturnToLongWindow             */
/* ========================================================================== */

#define WDG_TEST_NEG_WDGGETRETURNTOLONGWINDOW() \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgGetReturnToLongWindow_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgGetReturnToLongWindow_nullParam)

#define WDG_TEST_WDGGETRETURNTOLONGWINDOW() \
    WDG_TEST_NEG_WDGGETRETURNTOLONGWINDOW()

/* ========================================================================== */
/*               API-Specific Test Macros - wdgSendSwTrigger                  */
/* ========================================================================== */

#define WDG_TEST_POS_WDGSENDSWGRIGGER() \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgSendSwTrigger_detectNoErrors); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgSendSwTrigger_detectTrigEarlyErr)

#define WDG_TEST_NEG_WDGSENDSWGRIGGER() \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgSendSwTrigger_nullHandle)

#define WDG_TEST_WDGSENDSWGRIGGER() \
    WDG_TEST_POS_WDGSENDSWGRIGGER(); \
    WDG_TEST_NEG_WDGSENDSWGRIGGER()

/* ========================================================================== */
/*               API-Specific Test Macros - wdgQaWriteAnswer                  */
/* ========================================================================== */

#define WDG_TEST_POS_WDGQAWRITEANSWER() \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgQaWriteAnswer_qaFdbk0); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgQaWriteAnswer_qaFdbk1); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgQaWriteAnswer_qaFdbk2); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgQaWriteAnswer_qaFdbk3); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgQaWriteAnswer_fullSequence)

#define WDG_TEST_NEG_WDGQAWRITEANSWER() \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgQaWriteAnswer_nullHandle)

#define WDG_TEST_WDGQAWRITEANSWER() \
    WDG_TEST_POS_WDGQAWRITEANSWER(); \
    WDG_TEST_NEG_WDGQAWRITEANSWER()

/* ========================================================================== */
/*               API-Specific Test Macros - wdgClrErrStatus                   */
/* ========================================================================== */

#define WDG_TEST_POS_WDGCLRERRSTATUS() \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgClrErrStatus_rstInt); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgClrErrStatus_failInt); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgClrErrStatus_answErr); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgClrErrStatus_seqErr); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgClrErrStatus_answEarlyErr); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgClrErrStatus_trigEarlyErr); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgClrErrStatus_timeout); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgClrErrStatus_longwinTimeout); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgClrErrStatus_multipleFlags); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgClrErrStatus_th1ErrorOnly); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgClrErrStatus_th2ErrorOnly); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgClrErrStatus_seqErrorOnly)

#define WDG_TEST_NEG_WDGCLRERRSTATUS() \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgClrErrStatus_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgClrErrStatus_nullParam); \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgClrErrStatus_invalidParamZero); \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgClrErrStatus_invalidParamOutOfBounds)

#define WDG_TEST_WDGCLRERRSTATUS() \
    WDG_TEST_POS_WDGCLRERRSTATUS(); \
    WDG_TEST_NEG_WDGCLRERRSTATUS()

/* ========================================================================== */
/*              API-Specific Test Macros - wdgClrErrStatusAll                 */
/* ========================================================================== */

#define WDG_TEST_NEG_WDGCLRERRSTATUSALL() \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgClrErrStatusAll_nullHandle)

#define WDG_TEST_WDGCLRERRSTATUSALL() \
    WDG_TEST_NEG_WDGCLRERRSTATUSALL()

/* ========================================================================== */
/*               API-Specific Test Macros - wdgGetErrStatus                   */
/* ========================================================================== */

#define WDG_TEST_POS_WDGGETERRSTATUS() \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgGetErrStatus_allFields)

#define WDG_TEST_NEG_WDGGETERRSTATUS() \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgGetErrStatus_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgGetErrStatus_nullParam); \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgGetErrStatus_invalidParamZero); \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgGetErrStatus_invalidParamOutOfBounds)

#define WDG_TEST_WDGGETERRSTATUS() \
    WDG_TEST_POS_WDGGETERRSTATUS(); \
    WDG_TEST_NEG_WDGGETERRSTATUS()

/* ========================================================================== */
/*              API-Specific Test Macros - wdgGetFailCntStatus                */
/* ========================================================================== */

#define WDG_TEST_POS_WDGGETFAILCNTSTATUS() \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgGetFailCntStatus_copyFunction); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgGetFailCntStatus_failCntOnly); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgGetFailCntStatus_badCntOnly)

#define WDG_TEST_NEG_WDGGETFAILCNTSTATUS() \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgGetFailCntStatus_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgGetFailCntStatus_nullParam); \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgGetFailCntStatus_invalidParam)

#define WDG_TEST_WDGGETFAILCNTSTATUS() \
    WDG_TEST_POS_WDGGETFAILCNTSTATUS(); \
    WDG_TEST_NEG_WDGGETFAILCNTSTATUS()

/* ========================================================================== */
/*              API-Specific Test Macros - wdgQaSequence Tests                */
/* ========================================================================== */

#define WDG_TEST_POS_WDGQASEQUENCE() \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgQaSequence_detectNoErrors); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgQaSequence_detectAnswErr); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgQaSequence_detectSeqErr); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgQaSequence_detectAnswEarlyErr); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgQaSequence_detectTimeoutErr); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgQaSequence_detectLongWinTimeoutErr); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgQaSequence_detectFailInt); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgQaSequence_detectRstInt); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgQaSequence_qaWithIrqCallback)

#define WDG_TEST_WDGQASEQUENCE() \
    WDG_TEST_POS_WDGQASEQUENCE()

/* ========================================================================== */
/*               API-Specific Test Macros - wdgGetErrStatus                 */
/* ========================================================================== */

#define WDG_TEST_POS_WDGGETERRORSTATUS() \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgGetErrStatus_answerError)

#define WDG_TEST_WDGGETERRORSTATUS() \
    WDG_TEST_POS_WDGGETERRORSTATUS()

/* ========================================================================== */
/*                         Aggregate Test Runners                             */
/* ========================================================================== */

#define WDG_TEST_RUN_POSITIVE() \
    WDG_TEST_POS_WDGENABLE(); \
    WDG_TEST_POS_WDGSETCFG(); \
    WDG_TEST_POS_WDGSETPOWERHOLD(); \
    WDG_TEST_POS_WDGSETRETURNTOLONGWINDOW(); \
    WDG_TEST_POS_WDGSENDSWGRIGGER(); \
    WDG_TEST_POS_WDGQAWRITEANSWER(); \
    WDG_TEST_POS_WDGCLRERRSTATUS(); \
    WDG_TEST_POS_WDGGETERRSTATUS(); \
    WDG_TEST_POS_WDGGETFAILCNTSTATUS(); \
    WDG_TEST_POS_WDGQASEQUENCE(); \
    WDG_TEST_POS_WDGGETERRORSTATUS()

#define WDG_TEST_RUN_NEGATIVE() \
    WDG_TEST_NEG_WDGSETENABLESTATE(); \
    WDG_TEST_NEG_WDGENABLE(); \
    WDG_TEST_NEG_WDGDISABLE(); \
    WDG_TEST_NEG_WDGGETENABLESTATE(); \
    WDG_TEST_NEG_WDGSETCFG(); \
    WDG_TEST_NEG_WDGGETCFG(); \
    WDG_TEST_NEG_WDGSETPOWERHOLD(); \
    WDG_TEST_NEG_WDGGETPOWERHOLD(); \
    WDG_TEST_NEG_WDGSETRETURNTOLONGWINDOW(); \
    WDG_TEST_NEG_WDGGETRETURNTOLONGWINDOW(); \
    WDG_TEST_NEG_WDGSENDSWGRIGGER(); \
    WDG_TEST_NEG_WDGQAWRITEANSWER(); \
    WDG_TEST_NEG_WDGCLRERRSTATUS(); \
    WDG_TEST_NEG_WDGCLRERRSTATUSALL(); \
    WDG_TEST_NEG_WDGGETERRSTATUS(); \
    WDG_TEST_NEG_WDGGETFAILCNTSTATUS()

#define WDG_TEST_RUN_ALL() \
    WDG_TEST_RUN_POSITIVE(); \
    WDG_TEST_RUN_NEGATIVE()

#endif /* WDG_TEST_MACROS_H */
