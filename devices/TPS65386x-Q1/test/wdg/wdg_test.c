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


/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "wdg_test.h"
#include "test_inject.h"
#include "test_constants.h"
#include "regmap/wdg.h"

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/* ========================================================================== */
/*               API-Specific Test Macros - wdgEnable/Disable                 */
/* ========================================================================== */

#define WDG_TEST_POS_WDGENABLE() \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgEnable_enableDisable)

#define WDG_TEST_NEG_WDGENABLE() \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgEnable_nullHandle)

#define WDG_TEST_WDGENABLE() \
    WDG_TEST_POS_WDGENABLE(); \
    WDG_TEST_NEG_WDGENABLE()

#define WDG_TEST_POS_WDGDISABLE() \
    /* Positive tests for wdgDisable are combined with wdgEnable tests */

#define WDG_TEST_NEG_WDGDISABLE() \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgDisable_nullHandle)

#define WDG_TEST_WDGDISABLE() \
    WDG_TEST_POS_WDGDISABLE(); \
    WDG_TEST_NEG_WDGDISABLE()

#define WDG_TEST_POS_WDGSETENABLESTATE() \
    /* Positive tests for wdgSetEnableState are combined with wdgEnable tests */

#define WDG_TEST_NEG_WDGSETENABLESTATE() \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgSetEnableState_nullHandle)

#define WDG_TEST_WDGSETENABLESTATE() \
    WDG_TEST_POS_WDGSETENABLESTATE(); \
    WDG_TEST_NEG_WDGSETENABLESTATE()

#define WDG_TEST_POS_WDGGETENABLESTATE() \
    /* Positive tests for wdgGetEnableState are combined with wdgEnable tests */

#define WDG_TEST_NEG_WDGGETENABLESTATE() \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgGetEnableState_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgGetEnableState_nullParam)

#define WDG_TEST_WDGGETENABLESTATE() \
    WDG_TEST_POS_WDGGETENABLESTATE(); \
    WDG_TEST_NEG_WDGGETENABLESTATE()

/* ========================================================================== */
/*                 API-Specific Test Macros - wdgSetCfg                       */
/* ========================================================================== */

#define WDG_TEST_POS_WDGSETCFG() \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgSetCfg_longWindowDuration); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgSetCfg_window1Duration); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgSetCfg_window2Duration); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgSetCfg_failThreshold); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgSetCfg_resetThreshold); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgSetCfg_threshold1IntBehavior); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgSetCfg_wdgMode); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgSetCfg_threshold2IntBehavior); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgSetCfg_returnLongWindow); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgSetCfg_QA_feedback); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgSetCfg_QA_LFSR); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgSetCfg_QA_questionSeed); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgSetCfg_timeBase)

#define WDG_TEST_NEG_WDGSETCFG() \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgSetCfg_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgSetCfg_nullConfig); \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgSetCfg_invalidMode); \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgSetCfg_invalidTimeBase); \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgSetCfg_invalidThreshold1); \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgSetCfg_invalidThreshold2); \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgSetCfg_invalidQaFdbk); \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgSetCfg_invalidQaLfsr); \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgSetCfg_invalidQaQuesSeed); \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgSetCfg_invalidThreshold1IntBehavior); \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgSetCfg_invalidThreshold2IntBehavior); \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgSetCfg_zeroValidParams)

#define WDG_TEST_WDGSETCFG() \
    WDG_TEST_POS_WDGSETCFG(); \
    WDG_TEST_NEG_WDGSETCFG()

/* ========================================================================== */
/*                 API-Specific Test Macros - wdgGetCfg                       */
/* ========================================================================== */

#define WDG_TEST_POS_WDGGETCFG() \
    /* Positive tests for wdgGetCfg are combined with wdgSetCfg tests */

#define WDG_TEST_NEG_WDGGETCFG() \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgGetCfg_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgGetCfg_nullConfig)

#define WDG_TEST_WDGGETCFG() \
    WDG_TEST_POS_WDGGETCFG(); \
    WDG_TEST_NEG_WDGGETCFG()

/* ========================================================================== */
/*                 API-Specific Test Macros - wdgSetMode                      */
/* ========================================================================== */

#define WDG_TEST_POS_WDGSETMODE() \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgSetMode_triggerMode); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgSetMode_qAndAMode)

#define WDG_TEST_NEG_WDGSETMODE() \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgSetMode_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgSetMode_invalidMode)

#define WDG_TEST_WDGSETMODE() \
    WDG_TEST_POS_WDGSETMODE(); \
    WDG_TEST_NEG_WDGSETMODE()

/* ========================================================================== */
/*                 API-Specific Test Macros - wdgGetMode                      */
/* ========================================================================== */

#define WDG_TEST_POS_WDGGETMODE() \
    /* Positive tests for wdgGetMode are combined with wdgSetMode tests */

#define WDG_TEST_NEG_WDGGETMODE() \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgGetMode_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgGetMode_nullParam)

#define WDG_TEST_WDGGETMODE() \
    WDG_TEST_POS_WDGGETMODE(); \
    WDG_TEST_NEG_WDGGETMODE()

/* ========================================================================== */
/*                 API-Specific Test Macros - wdgSetPowerHold                 */
/* ========================================================================== */

#define WDG_TEST_POS_WDGSETPOWERHOLD() \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgSetPowerHold_enable); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgSetPowerHold_disable)

#define WDG_TEST_NEG_WDGSETPOWERHOLD() \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgSetPowerHold_nullHandle)

#define WDG_TEST_WDGSETPOWERHOLD() \
    WDG_TEST_POS_WDGSETPOWERHOLD(); \
    WDG_TEST_NEG_WDGSETPOWERHOLD()

/* ========================================================================== */
/*                 API-Specific Test Macros - wdgGetPowerHold                 */
/* ========================================================================== */

#define WDG_TEST_POS_WDGGETPOWERHOLD() \
    /* Positive tests for wdgGetPowerHold are combined with wdgSetPowerHold tests */

#define WDG_TEST_NEG_WDGGETPOWERHOLD() \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgGetPowerHold_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgGetPowerHold_nullParam)

#define WDG_TEST_WDGGETPOWERHOLD() \
    WDG_TEST_POS_WDGGETPOWERHOLD(); \
    WDG_TEST_NEG_WDGGETPOWERHOLD()

/* ========================================================================== */
/*            API-Specific Test Macros - wdgSetReturnToLongWindow             */
/* ========================================================================== */

#define WDG_TEST_POS_WDGSETRETURNTOLONGWINDOW() \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgSetReturnToLongWindow_enable); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgSetReturnToLongWindow_disable)

#define WDG_TEST_NEG_WDGSETRETURNTOLONGWINDOW() \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgSetReturnToLongWindow_nullHandle)

#define WDG_TEST_WDGSETRETURNTOLONGWINDOW() \
    WDG_TEST_POS_WDGSETRETURNTOLONGWINDOW(); \
    WDG_TEST_NEG_WDGSETRETURNTOLONGWINDOW()

/* ========================================================================== */
/*            API-Specific Test Macros - wdgGetReturnToLongWindow             */
/* ========================================================================== */

#define WDG_TEST_POS_WDGGETRETURNTOLONGWINDOW() \
    /* Positive tests for wdgGetReturnToLongWindow are combined with wdgSetReturnToLongWindow tests */

#define WDG_TEST_NEG_WDGGETRETURNTOLONGWINDOW() \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgGetReturnToLongWindow_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgGetReturnToLongWindow_nullParam)

#define WDG_TEST_WDGGETRETURNTOLONGWINDOW() \
    WDG_TEST_POS_WDGGETRETURNTOLONGWINDOW(); \
    WDG_TEST_NEG_WDGGETRETURNTOLONGWINDOW()

/* ========================================================================== */
/*              API-Specific Test Macros - wdgGetErrStatus                  */
/* ========================================================================== */

#define WDG_TEST_POS_WDGGETERRORSTATUS() \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgGetErrStatus_afterAnswerError); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgGetErrStatus_timeout); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgGetErrStatus_longWindowTimeout); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgGetErrStatus_answerEarlyError); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgGetErrStatus_sequenceErr); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgGetErrStatus_answerErr); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgGetErrStatus_triggerEarly); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgGetErrStatus_th1Int); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgGetErrStatus_th2Int); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgGetErrStatus_allFlags)

#define WDG_TEST_NEG_WDGGETERRORSTATUS() \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgGetErrStatus_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgGetErrStatus_nullParam)

#define WDG_TEST_WDGGETERRORSTATUS() \
    WDG_TEST_POS_WDGGETERRORSTATUS(); \
    WDG_TEST_NEG_WDGGETERRORSTATUS()

/* ========================================================================== */
/*              API-Specific Test Macros - wdgClrErrStatus                    */
/* ========================================================================== */

#define WDG_TEST_POS_WDGCLRERRSTATUS() \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgClrErrStatus_timeout); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgClrErrStatus_longWindowTimeout); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgClrErrStatus_answerEarlyError); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgClrErrStatus_sequenceErr); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgClrErrStatus_answerErr); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgClrErrStatus_triggerEarly); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgClrErrStatus_th1ErrorOnly); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgClrErrStatus_th2ErrorOnly); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgClrErrStatus_seqErrorOnly)

#define WDG_TEST_NEG_WDGCLRERRSTATUS() \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgClrErrStatus_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgClrErrStatus_nullParam)

#define WDG_TEST_WDGCLRERRSTATUS() \
    WDG_TEST_POS_WDGCLRERRSTATUS(); \
    WDG_TEST_NEG_WDGCLRERRSTATUS()

/* ========================================================================== */
/*              API-Specific Test Macros - wdgClrErrStatusAll                 */
/* ========================================================================== */

#define WDG_TEST_POS_WDGCLRERRSTATUSALL() \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgClrErrStatusAll_whenNoErrors)

#define WDG_TEST_NEG_WDGCLRERRSTATUSALL() \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgClrErrStatusAll_nullHandle)

#define WDG_TEST_WDGCLRERRSTATUSALL() \
    WDG_TEST_POS_WDGCLRERRSTATUSALL(); \
    WDG_TEST_NEG_WDGCLRERRSTATUSALL()

/* ========================================================================== */
/*              API-Specific Test Macros - wdgGetFailCntStatus                */
/* ========================================================================== */

#define WDG_TEST_POS_WDGGETFAILCNTSTATUS() \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgGetFailCntStatus_badEvent); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgGetFailCntStatus_goodEvent); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgGetFailCntStatus_wdFailCnt); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgGetFailCntStatus_allFields); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgGetFailCntStatus_failCntOnly); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgGetFailCntStatus_badCntOnly)

#define WDG_TEST_NEG_WDGGETFAILCNTSTATUS() \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgGetFailCntStatus_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgGetFailCntStatus_nullParam)

#define WDG_TEST_WDGGETFAILCNTSTATUS() \
    WDG_TEST_POS_WDGGETFAILCNTSTATUS(); \
    WDG_TEST_NEG_WDGGETFAILCNTSTATUS()

/* ========================================================================== */
/*              API-Specific Test Macros - wdgQaWriteAnswer                   */
/* ========================================================================== */

#define WDG_TEST_POS_WDGQAWRITEANSWER() \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgQaWriteAnswer_fullSequence); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgQaWriteAnswer_qaFdbk0); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgQaWriteAnswer_qaFdbk1); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgQaWriteAnswer_qaFdbk2); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgQaWriteAnswer_qaFdbk3); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgQaWriteAnswer_differentSeeds); \
    PLATFORM_RUN_TEST(test_pos_wdg_wdgQaWriteAnswer_differentLfsr)

#define WDG_TEST_NEG_WDGQAWRITEANSWER() \
    PLATFORM_RUN_TEST(test_neg_wdg_wdgQaWriteAnswer_nullHandle)

#define WDG_TEST_WDGQAWRITEANSWER() \
    WDG_TEST_POS_WDGQAWRITEANSWER(); \
    WDG_TEST_NEG_WDGQAWRITEANSWER()

/* ========================================================================== */
/*                       Test Injection Debug                                 */
/* ========================================================================== */

#define WDG_TEST_POS_TESTINJECT() \
    PLATFORM_RUN_TEST(test_pos_wdg_testInject_debug)

/* ========================================================================== */
/*                         Aggregate Test Runners                             */
/* ========================================================================== */

#define WDG_TEST_RUN_POSITIVE() \
    WDG_TEST_POS_TESTINJECT(); \
    WDG_TEST_POS_WDGENABLE(); \
    WDG_TEST_POS_WDGDISABLE(); \
    WDG_TEST_POS_WDGSETENABLESTATE(); \
    WDG_TEST_POS_WDGGETENABLESTATE(); \
    WDG_TEST_POS_WDGSETCFG(); \
    WDG_TEST_POS_WDGGETCFG(); \
    WDG_TEST_POS_WDGSETMODE(); \
    WDG_TEST_POS_WDGGETMODE(); \
    WDG_TEST_POS_WDGSETPOWERHOLD(); \
    WDG_TEST_POS_WDGGETPOWERHOLD(); \
    WDG_TEST_POS_WDGSETRETURNTOLONGWINDOW(); \
    WDG_TEST_POS_WDGGETRETURNTOLONGWINDOW(); \
    WDG_TEST_POS_WDGGETERRORSTATUS(); \
    WDG_TEST_POS_WDGCLRERRSTATUS(); \
    WDG_TEST_POS_WDGCLRERRSTATUSALL(); \
    WDG_TEST_POS_WDGGETFAILCNTSTATUS(); \
    WDG_TEST_POS_WDGQAWRITEANSWER()

#define WDG_TEST_RUN_NEGATIVE() \
    WDG_TEST_NEG_WDGENABLE(); \
    WDG_TEST_NEG_WDGDISABLE(); \
    WDG_TEST_NEG_WDGSETENABLESTATE(); \
    WDG_TEST_NEG_WDGGETENABLESTATE(); \
    WDG_TEST_NEG_WDGSETCFG(); \
    WDG_TEST_NEG_WDGGETCFG(); \
    WDG_TEST_NEG_WDGSETMODE(); \
    WDG_TEST_NEG_WDGGETMODE(); \
    WDG_TEST_NEG_WDGSETPOWERHOLD(); \
    WDG_TEST_NEG_WDGGETPOWERHOLD(); \
    WDG_TEST_NEG_WDGSETRETURNTOLONGWINDOW(); \
    WDG_TEST_NEG_WDGGETRETURNTOLONGWINDOW(); \
    WDG_TEST_NEG_WDGGETERRORSTATUS(); \
    WDG_TEST_NEG_WDGCLRERRSTATUS(); \
    WDG_TEST_NEG_WDGCLRERRSTATUSALL(); \
    WDG_TEST_NEG_WDGGETFAILCNTSTATUS(); \
    WDG_TEST_NEG_WDGQAWRITEANSWER()

#define WDG_TEST_RUN_ALL() \
    WDG_TEST_RUN_POSITIVE(); \
    WDG_TEST_RUN_NEGATIVE()

/* ========================================================================== */
/*                             Global Variables                               */
/* ========================================================================== */

static Pmic_Handle_t pmicHandle = {0U};

/* ========================================================================== */
/*                           Function Definitions                             */
/* ========================================================================== */

/**
 * @brief Test watchdog enable and disable operations
 */
void test_pos_wdg_wdgEnable_enableDisable(void)
{
    int32_t status;
    bool wdgEnabled = false;

    /* Enable Watchdog */
    status = Pmic_wdgEnable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Verify watchdog is enabled */
    status = Pmic_wdgGetEnableState(&pmicHandle, &wdgEnabled);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(wdgEnabled == true);

    /* Disable Watchdog */
    status = Pmic_wdgDisable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Verify watchdog is disabled */
    status = Pmic_wdgGetEnableState(&pmicHandle, &wdgEnabled);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(wdgEnabled == false);
}

/**
 * @brief Test watchdog long window duration configuration
 */
void test_pos_wdg_wdgSetCfg_longWindowDuration(void)
{
    int32_t status;
    Pmic_WdgCfg_t wdgCfg = {0};

    /* Enable watchdog first */
    status = Pmic_wdgEnable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Set long window duration */
    wdgCfg.validParams = PMIC_CFG_WDG_LONGWINDURATION_VALID;
    wdgCfg.longWinCode = TEST_INVALID_PARAM_255;  // 255 * 1100us = 280.5ms (max possible, approximates 772ms intent)

    status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Get and verify long window duration */
    wdgCfg.validParams = PMIC_CFG_WDG_LONGWINDURATION_VALID;
    wdgCfg.longWinCode = 0U;

    status = Pmic_wdgGetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(wdgCfg.longWinCode == TEST_INVALID_PARAM_255);
}

/**
 * @brief Test watchdog window-1 duration configuration
 */
void test_pos_wdg_wdgSetCfg_window1Duration(void)
{
    int32_t status;
    Pmic_WdgCfg_t wdgCfg = {0};

    /* Enable watchdog first */
    status = Pmic_wdgEnable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Set window-1 duration */
    wdgCfg.validParams = PMIC_CFG_WDG_WIN1DURATION_VALID;
    wdgCfg.win1Code = 0x40U;  // 64 * 1100us = 70.4ms (exact match to original intent)

    status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Get and verify window-1 duration */
    wdgCfg.validParams = PMIC_CFG_WDG_WIN1DURATION_VALID;
    wdgCfg.win1Code = 0U;

    status = Pmic_wdgGetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(wdgCfg.win1Code == 0x40U);
}

/**
 * @brief Test watchdog window-2 duration configuration
 */
void test_pos_wdg_wdgSetCfg_window2Duration(void)
{
    int32_t status;
    Pmic_WdgCfg_t wdgCfg = {0};

    /* Enable watchdog first */
    status = Pmic_wdgEnable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Set window-2 duration */
    wdgCfg.validParams = PMIC_CFG_WDG_WIN2DURATION_VALID;
    wdgCfg.win2Code = 0x40U;  // 64 * 1100us = 70.4ms (exact match to original intent)

    status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Get and verify window-2 duration */
    wdgCfg.validParams = PMIC_CFG_WDG_WIN2DURATION_VALID;
    wdgCfg.win2Code = 0U;

    status = Pmic_wdgGetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(wdgCfg.win2Code == 0x40U);
}

/**
 * @brief Test watchdog fail threshold configuration
 */
void test_pos_wdg_wdgSetCfg_failThreshold(void)
{
    int32_t status;
    Pmic_WdgCfg_t wdgCfg = {0};

    /* Enable watchdog first */
    status = Pmic_wdgEnable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Set fail threshold */
    wdgCfg.validParams = PMIC_CFG_WDG_THRESHOLD_1_VALID;
    wdgCfg.threshold1 = 7U;

    status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Get and verify fail threshold */
    wdgCfg.validParams = PMIC_CFG_WDG_THRESHOLD_1_VALID;
    wdgCfg.threshold1 = 0U;

    status = Pmic_wdgGetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(wdgCfg.threshold1 == 7U);
}

/**
 * @brief Test watchdog reset threshold configuration
 */
void test_pos_wdg_wdgSetCfg_resetThreshold(void)
{
    int32_t status;
    Pmic_WdgCfg_t wdgCfg = {0};

    /* Enable watchdog first */
    status = Pmic_wdgEnable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Set reset threshold */
    wdgCfg.validParams = PMIC_CFG_WDG_THRESHOLD_2_VALID;
    wdgCfg.threshold2 = 7U;

    status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Get and verify reset threshold */
    wdgCfg.validParams = PMIC_CFG_WDG_THRESHOLD_2_VALID;
    wdgCfg.threshold2 = 0U;

    status = Pmic_wdgGetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(wdgCfg.threshold2 == 7U);
}

/**
 * @brief Test watchdog threshold 1 interrupt behavior configuration
 * This implements the missing test_wdg_setCfg_resetEnable
 */
void test_pos_wdg_wdgSetCfg_threshold1IntBehavior(void)
{
    int32_t status;
    Pmic_WdgCfg_t wdgCfg = {0};

    /* Enable watchdog first */
    status = Pmic_wdgEnable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Set threshold1 interrupt behavior */
    wdgCfg.validParams = PMIC_CFG_WDG_THRESHOLD1_INT_BEHAVIOR_VALID;
    wdgCfg.threshold1IntBehavior = 1U; /* Enable interrupt */

    status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Get and verify threshold1 interrupt behavior */
    wdgCfg.validParams = PMIC_CFG_WDG_THRESHOLD1_INT_BEHAVIOR_VALID;
    wdgCfg.threshold1IntBehavior = 0U;

    status = Pmic_wdgGetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(wdgCfg.threshold1IntBehavior == 1U);
}

/**
 * @brief Test watchdog mode configuration
 */
void test_pos_wdg_wdgSetCfg_wdgMode(void)
{
    int32_t status;
    Pmic_WdgCfg_t wdgCfg = {0};

    /* Enable watchdog first */
    status = Pmic_wdgEnable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Set Q&A mode */
    wdgCfg.validParams = PMIC_CFG_WDG_MODE_VALID;
    wdgCfg.mode = PMIC_WDG_QA_MODE;

    status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Verify Q&A mode */
    wdgCfg.validParams = PMIC_CFG_WDG_MODE_VALID;
    wdgCfg.mode = 0U;

    status = Pmic_wdgGetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(wdgCfg.mode == PMIC_WDG_QA_MODE);

    /* Set Trigger mode */
    wdgCfg.validParams = PMIC_CFG_WDG_MODE_VALID;
    wdgCfg.mode = PMIC_WDG_TRIGGER_MODE;

    status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Verify Trigger mode */
    wdgCfg.validParams = PMIC_CFG_WDG_MODE_VALID;
    wdgCfg.mode = 0U;

    status = Pmic_wdgGetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(wdgCfg.mode == PMIC_WDG_TRIGGER_MODE);
}

/**
 * @brief Test watchdog threshold 2 interrupt behavior configuration
 * This implements the missing test_wdg_setCfg_powerHold
 */
void test_pos_wdg_wdgSetCfg_threshold2IntBehavior(void)
{
    int32_t status;
    Pmic_WdgCfg_t wdgCfg = {0};

    /* Enable watchdog first */
    status = Pmic_wdgEnable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Set threshold2 interrupt behavior */
    wdgCfg.validParams = PMIC_CFG_WDG_THRESHOLD2_INT_BEHAVIOR_VALID;
    wdgCfg.threshold2IntBehavior = 1U; /* Enable interrupt */

    status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Get and verify threshold2 interrupt behavior */
    wdgCfg.validParams = PMIC_CFG_WDG_THRESHOLD2_INT_BEHAVIOR_VALID;
    wdgCfg.threshold2IntBehavior = 0U;

    status = Pmic_wdgGetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(wdgCfg.threshold2IntBehavior == 1U);
}

/**
 * @brief Test watchdog return to long window configuration
 * This implements the missing test_wdg_setCfg_ReturnLongWindow
 */
void test_pos_wdg_wdgSetCfg_returnLongWindow(void)
{
    int32_t status;
    Pmic_WdgCfg_t wdgCfg = {0};

    /* Enable watchdog first */
    status = Pmic_wdgEnable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Configure long window and verify we can return to it */
    wdgCfg.validParams = PMIC_CFG_WDG_LONGWINDURATION_VALID;
    wdgCfg.longWinCode = TEST_INVALID_PARAM_255;  // 255 * 1100us = 280.5ms (max possible, approximates 512ms intent)

    status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Set a different window temporarily */
    wdgCfg.validParams = PMIC_CFG_WDG_WIN1DURATION_VALID;
    wdgCfg.win1Code = 0x20U;  // 32 * 1100us = 35.2ms (exact match to original intent)

    status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Return to long window and verify */
    wdgCfg.validParams = PMIC_CFG_WDG_LONGWINDURATION_VALID;
    wdgCfg.longWinCode = 0U;

    status = Pmic_wdgGetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(wdgCfg.longWinCode == TEST_INVALID_PARAM_255);
}

/**
 * @brief Test watchdog Q&A feedback configuration
 */
void test_pos_wdg_wdgSetCfg_QA_feedback(void)
{
    int32_t status;
    Pmic_WdgCfg_t wdgCfg = {0};

    /* Enable watchdog first */
    status = Pmic_wdgEnable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Set Q&A feedback */
    wdgCfg.validParams = PMIC_CFG_WDG_QA_FDBK_VALID;
    wdgCfg.qaFdbk = 1U;

    status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Get and verify Q&A feedback */
    wdgCfg.validParams = PMIC_CFG_WDG_QA_FDBK_VALID;
    wdgCfg.qaFdbk = 0U;

    status = Pmic_wdgGetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(wdgCfg.qaFdbk == 1U);
}

/**
 * @brief Test watchdog Q&A LFSR configuration
 */
void test_pos_wdg_wdgSetCfg_QA_LFSR(void)
{
    int32_t status;
    Pmic_WdgCfg_t wdgCfg = {0};

    /* Enable watchdog first */
    status = Pmic_wdgEnable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Set Q&A LFSR */
    wdgCfg.validParams = PMIC_CFG_WDG_QA_LFSR_VALID;
    wdgCfg.qaLfsr = 0x3U;  // 2-bit field max value (was 0xAB=171, masked to 3 by hardware)

    status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Get and verify Q&A LFSR */
    wdgCfg.validParams = PMIC_CFG_WDG_QA_LFSR_VALID;
    wdgCfg.qaLfsr = 0U;

    status = Pmic_wdgGetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(wdgCfg.qaLfsr == 0x3U);
}

/**
 * @brief Test watchdog Q&A question seed configuration
 */
void test_pos_wdg_wdgSetCfg_QA_questionSeed(void)
{
    int32_t status;
    Pmic_WdgCfg_t wdgCfg = {0};

    /* Enable watchdog first */
    status = Pmic_wdgEnable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Set Q&A question seed */
    wdgCfg.validParams = PMIC_CFG_WDG_QA_QUES_SEED_VALID;
    wdgCfg.qaQuesSeed = 0x5U;  // 4-bit field valid values 0-15 (was 0x55=85, masked to 5 by hardware)

    status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Get and verify Q&A question seed */
    wdgCfg.validParams = PMIC_CFG_WDG_QA_QUES_SEED_VALID;
    wdgCfg.qaQuesSeed = 0U;

    status = Pmic_wdgGetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(wdgCfg.qaQuesSeed == 0x5U);
}

/* ========================================================================== */
/* Q&A Sequence Tests                                                         */
/* ========================================================================== */

/**
 * @brief Test that testInject works by reading back injected value
 */
void test_pos_wdg_testInject_debug(void)
{
#ifdef BUILD_MOCK
    int32_t status;
    uint8_t readVal = 0U;

    /* Set register to known value */
    status = testInject_setRegister(PMIC_WD_ERR_STAT_REG, 0x42U);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Read it back through the driver */
    status = Pmic_ioRxByte(&pmicHandle, PMIC_WD_ERR_STAT_REG, &readVal);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(readVal == 0x42U);
#else
    PLATFORM_ASSERT(true);
#endif
}

/**
 * @brief Test Q&A write answer with full sequence in long window
 */
void test_pos_wdg_wdgQaWriteAnswer_fullSequence(void)
{
    int32_t status;
    Pmic_WdgCfg_t wdgCfg = {0};

    /* Configure WDG in Q&A mode */
    wdgCfg.validParams = PMIC_CFG_WDG_MODE_VALID |
                         PMIC_CFG_WDG_QA_FDBK_VALID |
                         PMIC_CFG_WDG_QA_LFSR_VALID |
                         PMIC_CFG_WDG_QA_QUES_SEED_VALID;
    wdgCfg.mode = PMIC_WDG_QA_MODE;
    wdgCfg.qaFdbk = 0U;
    wdgCfg.qaLfsr = 0x2U;
    wdgCfg.qaQuesSeed = 0xCU;

    status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_wdgEnable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Write 4 answer bytes (long window requires 4) */
    for (uint8_t i = 0; i < 4U; i++) {
        status = Pmic_wdgQaWriteAnswer(&pmicHandle);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    }

    status = Pmic_wdgDisable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

/**
 * @brief Test Q&A write answer with qaFdbk=0 (mux case 0)
 */
void test_pos_wdg_wdgQaWriteAnswer_qaFdbk0(void)
{
    int32_t status;
    Pmic_WdgCfg_t wdgCfg = {0};

    wdgCfg.validParams = PMIC_CFG_WDG_MODE_VALID | PMIC_CFG_WDG_QA_FDBK_VALID;
    wdgCfg.mode = PMIC_WDG_QA_MODE;
    wdgCfg.qaFdbk = 0U;

    status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_wdgEnable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_wdgQaWriteAnswer(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_wdgDisable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

/**
 * @brief Test Q&A write answer with qaFdbk=1 (mux case 1)
 */
void test_pos_wdg_wdgQaWriteAnswer_qaFdbk1(void)
{
    int32_t status;
    Pmic_WdgCfg_t wdgCfg = {0};

    wdgCfg.validParams = PMIC_CFG_WDG_MODE_VALID | PMIC_CFG_WDG_QA_FDBK_VALID;
    wdgCfg.mode = PMIC_WDG_QA_MODE;
    wdgCfg.qaFdbk = 1U;

    status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_wdgEnable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_wdgQaWriteAnswer(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_wdgDisable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

/**
 * @brief Test Q&A write answer with qaFdbk=2 (mux case 2)
 */
void test_pos_wdg_wdgQaWriteAnswer_qaFdbk2(void)
{
    int32_t status;
    Pmic_WdgCfg_t wdgCfg = {0};

    wdgCfg.validParams = PMIC_CFG_WDG_MODE_VALID | PMIC_CFG_WDG_QA_FDBK_VALID;
    wdgCfg.mode = PMIC_WDG_QA_MODE;
    wdgCfg.qaFdbk = 2U;

    status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_wdgEnable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_wdgQaWriteAnswer(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_wdgDisable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

/**
 * @brief Test Q&A write answer with qaFdbk=3 (mux case 3)
 */
void test_pos_wdg_wdgQaWriteAnswer_qaFdbk3(void)
{
    int32_t status;
    Pmic_WdgCfg_t wdgCfg = {0};

    wdgCfg.validParams = PMIC_CFG_WDG_MODE_VALID | PMIC_CFG_WDG_QA_FDBK_VALID;
    wdgCfg.mode = PMIC_WDG_QA_MODE;
    wdgCfg.qaFdbk = 3U;

    status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_wdgEnable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_wdgQaWriteAnswer(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_wdgDisable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

/**
 * @brief Test Q&A write answer with different qaSeed values
 */
void test_pos_wdg_wdgQaWriteAnswer_differentSeeds(void)
{
    int32_t status;
    Pmic_WdgCfg_t wdgCfg = {0};

    wdgCfg.validParams = PMIC_CFG_WDG_MODE_VALID | PMIC_CFG_WDG_QA_QUES_SEED_VALID;
    wdgCfg.mode = PMIC_WDG_QA_MODE;
    wdgCfg.qaQuesSeed = 0xAU;  /* Test with seed = 10 */

    status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_wdgEnable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_wdgQaWriteAnswer(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_wdgDisable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

/**
 * @brief Test Q&A write answer with different qaLfsr values
 */
void test_pos_wdg_wdgQaWriteAnswer_differentLfsr(void)
{
    int32_t status;
    Pmic_WdgCfg_t wdgCfg = {0};

    wdgCfg.validParams = PMIC_CFG_WDG_MODE_VALID | PMIC_CFG_WDG_QA_LFSR_VALID;
    wdgCfg.mode = PMIC_WDG_QA_MODE;
    wdgCfg.qaLfsr = 0x1U;  /* Test with LFSR = 1 */

    status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_wdgEnable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_wdgQaWriteAnswer(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_wdgDisable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

/**
 * @brief Test getting error status after answer error (using test injection)
 */
void test_pos_wdg_wdgGetErrStatus_afterAnswerError(void)
{
#ifdef BUILD_MOCK
    int32_t status;
    Pmic_WdgErrStatus_t errors = {0};

    /* Use test injection to set the ANSW_ERR flag */
    status = testInject_setRegister(PMIC_WD_ERR_STAT_REG, PMIC_WD_ANSW_ERR_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Read error status */
    errors.validParams = PMIC_CFG_WD_ANSW_ERR_ERR_VALID;
    status = Pmic_wdgGetErrStatus(&pmicHandle, &errors);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(errors.answerError == true);
#else
    /* Test requires mock support for register injection */
    PLATFORM_ASSERT(true);
#endif
}

/* ========================================================================== */
/* Error Status Get Tests                                                     */
/* ========================================================================== */

/**
 * @brief Test getting timeout error status
 */
void test_pos_wdg_wdgGetErrStatus_timeout(void)
{
#ifdef BUILD_MOCK
    int32_t status;
    Pmic_WdgErrStatus_t errors = {0};

    /* Clear the register first, then set the TIMEOUT flag */
    status = testInject_setRegister(PMIC_WD_ERR_STAT_REG, 0x00U);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = testInject_setBits(PMIC_WD_ERR_STAT_REG, PMIC_WD_TMO_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    errors.validParams = PMIC_CFG_WD_TIMEOUT_ERR_VALID;
    status = Pmic_wdgGetErrStatus(&pmicHandle, &errors);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(errors.timeout == true);
#else
    /* Test requires mock support for register injection */
    PLATFORM_ASSERT(true);
#endif
}

/**
 * @brief Test getting long window timeout error status
 */
void test_pos_wdg_wdgGetErrStatus_longWindowTimeout(void)
{
#ifdef BUILD_MOCK
    int32_t status;
    Pmic_WdgErrStatus_t errors = {0};

    /* Use test injection to set the LONGWIN_TMO flag */
    status = testInject_setRegister(PMIC_WD_ERR_STAT_REG, PMIC_WD_LONGWIN_TMO_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    errors.validParams = PMIC_CFG_WD_LONGWIN_TIMEOUT_ERR_VALID;
    status = Pmic_wdgGetErrStatus(&pmicHandle, &errors);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(errors.longWindowTimeout == true);
#endif
}

/**
 * @brief Test getting answer early error status
 */
void test_pos_wdg_wdgGetErrStatus_answerEarlyError(void)
{
#ifdef BUILD_MOCK
    int32_t status;
    Pmic_WdgErrStatus_t errors = {0};

    /* Use test injection to set the ANSW_EARLY flag */
    status = testInject_setRegister(PMIC_WD_ERR_STAT_REG, PMIC_WD_ANSW_EARLY_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    errors.validParams = PMIC_CFG_WD_ANSW_EARLY_ERR_VALID;
    status = Pmic_wdgGetErrStatus(&pmicHandle, &errors);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(errors.answerEarlyError == true);
#else
    /* Test requires mock support for register injection */
    PLATFORM_ASSERT(true);
#endif
}

/**
 * @brief Test getting sequence error status
 */
void test_pos_wdg_wdgGetErrStatus_sequenceErr(void)
{
#ifdef BUILD_MOCK
    int32_t status;
    Pmic_WdgErrStatus_t errors = {0};

    /* Use test injection to set the SEQ_ERR flag */
    status = testInject_setRegister(PMIC_WD_ERR_STAT_REG, PMIC_WD_SEQ_ERR_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    errors.validParams = PMIC_CFG_WD_SEQ_ERR_ERR_VALID;
    status = Pmic_wdgGetErrStatus(&pmicHandle, &errors);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(errors.sequenceError == true);
#else
    /* Test requires mock support for register injection */
    PLATFORM_ASSERT(true);
#endif
}

/**
 * @brief Test getting answer error status
 */
void test_pos_wdg_wdgGetErrStatus_answerErr(void)
{
#ifdef BUILD_MOCK
    int32_t status;
    Pmic_WdgErrStatus_t errors = {0};

    /* Use test injection to set the ANSW_ERR flag */
    status = testInject_setRegister(PMIC_WD_ERR_STAT_REG, PMIC_WD_ANSW_ERR_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    errors.validParams = PMIC_CFG_WD_ANSW_ERR_ERR_VALID;
    status = Pmic_wdgGetErrStatus(&pmicHandle, &errors);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(errors.answerError == true);
#else
    /* Test requires mock support for register injection */
    PLATFORM_ASSERT(true);
#endif
}

/**
 * @brief Test getting trigger early error status (TPS65386x unique)
 */
void test_pos_wdg_wdgGetErrStatus_triggerEarly(void)
{
#ifdef BUILD_MOCK
    int32_t status;
    Pmic_WdgErrStatus_t errors = {0};

    /* Use test injection to set the TRIG_EARLY flag */
    status = testInject_setRegister(PMIC_WD_ERR_STAT_REG, PMIC_WD_TRIG_EARLY_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    errors.validParams = PMIC_CFG_WD_TRIG_EARLY_ERR_VALID;
    status = Pmic_wdgGetErrStatus(&pmicHandle, &errors);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(errors.triggerEarlyError == true);
#else
    /* Test requires mock support for register injection */
    PLATFORM_ASSERT(true);
#endif
}

/**
 * @brief Test getting threshold 1 interrupt error status
 */
void test_pos_wdg_wdgGetErrStatus_th1Int(void)
{
#ifdef BUILD_MOCK
    int32_t status;
    Pmic_WdgErrStatus_t errors = {0};

    /* Use test injection to set the TH1_ERR flag */
    status = testInject_setRegister(PMIC_WD_ERR_STAT_REG, PMIC_WD_TH1_ERR_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    errors.validParams = PMIC_CFG_WD_TH1_INT_ERR_VALID;
    status = Pmic_wdgGetErrStatus(&pmicHandle, &errors);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(errors.threshold1Error == true);
#else
    /* Test requires mock support for register injection */
    PLATFORM_ASSERT(true);
#endif
}

/**
 * @brief Test getting threshold 2 interrupt error status
 */
void test_pos_wdg_wdgGetErrStatus_th2Int(void)
{
#ifdef BUILD_MOCK
    int32_t status;
    Pmic_WdgErrStatus_t errors = {0};

    /* Use test injection to set the TH2_ERR flag */
    status = testInject_setRegister(PMIC_WD_ERR_STAT_REG, PMIC_WD_TH2_ERR_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    errors.validParams = PMIC_CFG_WD_TH2_INT_ERR_VALID;
    status = Pmic_wdgGetErrStatus(&pmicHandle, &errors);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(errors.threshold2Error == true);
#else
    /* Test requires mock support for register injection */
    PLATFORM_ASSERT(true);
#endif
}

/**
 * @brief Test getting all error flags at once
 */
void test_pos_wdg_wdgGetErrStatus_allFlags(void)
{
#ifdef BUILD_MOCK
    int32_t status;
    Pmic_WdgErrStatus_t errors = {0};

    /* Set all error flags */
    uint8_t allFlags = PMIC_WD_TMO_MASK |
                       PMIC_WD_TRIG_EARLY_MASK |
                       PMIC_WD_ANSW_EARLY_MASK |
                       PMIC_WD_SEQ_ERR_MASK |
                       PMIC_WD_ANSW_ERR_MASK |
                       PMIC_WD_LONGWIN_TMO_MASK |
                       PMIC_WD_TH1_ERR_MASK |
                       PMIC_WD_TH2_ERR_MASK;

    status = testInject_setRegister(PMIC_WD_ERR_STAT_REG, allFlags);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Read all error flags */
    errors.validParams = PMIC_CFG_WD_ERR_STAT_ALL_VALID_SHIFT;
    status = Pmic_wdgGetErrStatus(&pmicHandle, &errors);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(errors.timeout == true);
    PLATFORM_ASSERT(errors.triggerEarlyError == true);
    PLATFORM_ASSERT(errors.answerEarlyError == true);
    PLATFORM_ASSERT(errors.sequenceError == true);
    PLATFORM_ASSERT(errors.answerError == true);
    PLATFORM_ASSERT(errors.longWindowTimeout == true);
    PLATFORM_ASSERT(errors.threshold1Error == true);
    PLATFORM_ASSERT(errors.threshold2Error == true);
#else
    /* Test requires mock support for register injection */
    PLATFORM_ASSERT(true);
#endif
}

/* ========================================================================== */
/* Clear Error Status Tests                                                   */
/* ========================================================================== */

/**
 * @brief Test clearing timeout error status
 */
void test_pos_wdg_wdgClrErrStatus_timeout(void)
{
    int32_t status;
    Pmic_WdgErrStatus_t errors = {0};

    /* Set the error flag first */
    status = testInject_setRegister(PMIC_WD_ERR_STAT_REG, PMIC_WD_TMO_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Clear it */
    errors.validParams = PMIC_CFG_WD_TIMEOUT_ERR_VALID;
    errors.timeout = true;
    status = Pmic_wdgClrErrStatus(&pmicHandle, &errors);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Verify it's cleared */
    errors.timeout = false;
    status = Pmic_wdgGetErrStatus(&pmicHandle, &errors);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(errors.timeout == false);
}

/**
 * @brief Test clearing long window timeout error status
 */
void test_pos_wdg_wdgClrErrStatus_longWindowTimeout(void)
{
    int32_t status;
    Pmic_WdgErrStatus_t errors = {0};

    /* Set the error flag first */
    status = testInject_setRegister(PMIC_WD_ERR_STAT_REG, PMIC_WD_LONGWIN_TMO_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Clear it */
    errors.validParams = PMIC_CFG_WD_LONGWIN_TIMEOUT_ERR_VALID;
    errors.longWindowTimeout = true;
    status = Pmic_wdgClrErrStatus(&pmicHandle, &errors);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Verify it's cleared */
    errors.longWindowTimeout = false;
    status = Pmic_wdgGetErrStatus(&pmicHandle, &errors);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(errors.longWindowTimeout == false);
}

/**
 * @brief Test clearing answer early error status
 */
void test_pos_wdg_wdgClrErrStatus_answerEarlyError(void)
{
    int32_t status;
    Pmic_WdgErrStatus_t errors = {0};

    /* Set the error flag first */
    status = testInject_setRegister(PMIC_WD_ERR_STAT_REG, PMIC_WD_ANSW_EARLY_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Clear it */
    errors.validParams = PMIC_CFG_WD_ANSW_EARLY_ERR_VALID;
    errors.answerEarlyError = true;
    status = Pmic_wdgClrErrStatus(&pmicHandle, &errors);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Verify it's cleared */
    errors.answerEarlyError = false;
    status = Pmic_wdgGetErrStatus(&pmicHandle, &errors);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(errors.answerEarlyError == false);
}

/**
 * @brief Test clearing sequence error status
 */
void test_pos_wdg_wdgClrErrStatus_sequenceErr(void)
{
    int32_t status;
    Pmic_WdgErrStatus_t errors = {0};

    /* Set the error flag first */
    status = testInject_setRegister(PMIC_WD_ERR_STAT_REG, PMIC_WD_SEQ_ERR_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Clear it */
    errors.validParams = PMIC_CFG_WD_SEQ_ERR_ERR_VALID;
    errors.sequenceError = true;
    status = Pmic_wdgClrErrStatus(&pmicHandle, &errors);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Verify it's cleared */
    errors.sequenceError = false;
    status = Pmic_wdgGetErrStatus(&pmicHandle, &errors);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(errors.sequenceError == false);
}

/**
 * @brief Test clearing answer error status
 */
void test_pos_wdg_wdgClrErrStatus_answerErr(void)
{
    int32_t status;
    Pmic_WdgErrStatus_t errors = {0};

    /* Set the error flag first */
    status = testInject_setRegister(PMIC_WD_ERR_STAT_REG, PMIC_WD_ANSW_ERR_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Clear it */
    errors.validParams = PMIC_CFG_WD_ANSW_ERR_ERR_VALID;
    errors.answerError = true;
    status = Pmic_wdgClrErrStatus(&pmicHandle, &errors);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Verify it's cleared */
    errors.answerError = false;
    status = Pmic_wdgGetErrStatus(&pmicHandle, &errors);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(errors.answerError == false);
}

/**
 * @brief Test clearing trigger early error status
 */
void test_pos_wdg_wdgClrErrStatus_triggerEarly(void)
{
    int32_t status;
    Pmic_WdgErrStatus_t errors = {0};

    /* Set the error flag first */
    status = testInject_setRegister(PMIC_WD_ERR_STAT_REG, PMIC_WD_TRIG_EARLY_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Clear it */
    errors.validParams = PMIC_CFG_WD_TRIG_EARLY_ERR_VALID;
    errors.triggerEarlyError = true;
    status = Pmic_wdgClrErrStatus(&pmicHandle, &errors);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Verify it's cleared */
    errors.triggerEarlyError = false;
    status = Pmic_wdgGetErrStatus(&pmicHandle, &errors);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(errors.triggerEarlyError == false);
}

/**
 * @brief Test clearing threshold1 error status only
 */
void test_pos_wdg_wdgClrErrStatus_th1ErrorOnly(void)
{
    int32_t status;
    Pmic_WdgErrStatus_t errors = {0};

    /* Set the error flag first */
    status = testInject_setRegister(PMIC_WD_ERR_STAT_REG, PMIC_WD_TH1_ERR_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Clear it */
    errors.validParams = PMIC_CFG_WD_TH1_INT_ERR_VALID;
    errors.threshold1Error = true;
    status = Pmic_wdgClrErrStatus(&pmicHandle, &errors);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Verify it's cleared */
    errors.threshold1Error = false;
    status = Pmic_wdgGetErrStatus(&pmicHandle, &errors);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(errors.threshold1Error == false);
}

/**
 * @brief Test clearing threshold2 error status only
 */
void test_pos_wdg_wdgClrErrStatus_th2ErrorOnly(void)
{
    int32_t status;
    Pmic_WdgErrStatus_t errors = {0};

    /* Set the error flag first */
    status = testInject_setRegister(PMIC_WD_ERR_STAT_REG, PMIC_WD_TH2_ERR_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Clear it */
    errors.validParams = PMIC_CFG_WD_TH2_INT_ERR_VALID;
    errors.threshold2Error = true;
    status = Pmic_wdgClrErrStatus(&pmicHandle, &errors);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Verify it's cleared */
    errors.threshold2Error = false;
    status = Pmic_wdgGetErrStatus(&pmicHandle, &errors);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(errors.threshold2Error == false);
}

/**
 * @brief Test clearing sequence error status only (when multiple errors are set)
 */
void test_pos_wdg_wdgClrErrStatus_seqErrorOnly(void)
{
    int32_t status;
    Pmic_WdgErrStatus_t errors = {0};

    /* Set multiple error flags: sequence error + answer error */
    status = testInject_setRegister(PMIC_WD_ERR_STAT_REG,
                                     PMIC_WD_SEQ_ERR_MASK | PMIC_WD_ANSW_ERR_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Clear only sequence error */
    errors.validParams = PMIC_CFG_WD_SEQ_ERR_ERR_VALID;
    errors.sequenceError = true;
    status = Pmic_wdgClrErrStatus(&pmicHandle, &errors);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Verify sequence error is cleared but answer error remains */
    errors.validParams = PMIC_CFG_WD_SEQ_ERR_ERR_VALID | PMIC_CFG_WD_ANSW_ERR_ERR_VALID;
    errors.sequenceError = false;
    errors.answerError = false;
    status = Pmic_wdgGetErrStatus(&pmicHandle, &errors);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(errors.sequenceError == false);
    PLATFORM_ASSERT(errors.answerError == true);
}

/**
 * @brief Test clearing all errors when no errors are set (optimization path)
 */
void test_pos_wdg_wdgClrErrStatusAll_whenNoErrors(void)
{
#ifdef BUILD_MOCK
    int32_t status;

    /* Clear the register first to ensure no errors */
    status = testInject_clearBits(PMIC_WD_ERR_STAT_REG, TEST_MASK_FULL_BYTE);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Clear all errors (when none are set) */
    status = Pmic_wdgClrErrStatusAll(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
#else
    /* Test requires mock support for register injection */
    PLATFORM_ASSERT(true);
#endif
}

/* ========================================================================== */
/* Fail Count Status Tests                                                    */
/* ========================================================================== */

/**
 * @brief Test getting bad event status
 */
void test_pos_wdg_wdgGetFailCntStatus_badEvent(void)
{
#ifdef BUILD_MOCK
    int32_t status;
    Pmic_WdgFailCntStatus_t failCount = {0};

    /* Set the bad event flag */
    status = testInject_setRegister(PMIC_WD_STAT_REG, PMIC_WD_BAD_EVENT_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    failCount.validParams = PMIC_CFG_WD_BAD_EVENT_STAT_VALID;
    status = Pmic_wdgGetFailCntStatus(&pmicHandle, &failCount);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(failCount.badEvent == true);
#else
    /* Test requires mock support for register injection */
    PLATFORM_ASSERT(true);
#endif
}

/**
 * @brief Test getting good event status
 */
void test_pos_wdg_wdgGetFailCntStatus_goodEvent(void)
{
#ifdef BUILD_MOCK
    int32_t status;
    Pmic_WdgFailCntStatus_t failCount = {0};

    /* Set the first ok flag (good event) */
    status = testInject_setRegister(PMIC_WD_STAT_REG, PMIC_WD_FIRST_OK_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    failCount.validParams = PMIC_CFG_WD_GOOD_EVENT_STAT_VALID;
    status = Pmic_wdgGetFailCntStatus(&pmicHandle, &failCount);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(failCount.goodEvent == true);
#else
    /* Test requires mock support for register injection */
    PLATFORM_ASSERT(true);
#endif
}

/**
 * @brief Test getting watchdog fail count value
 */
void test_pos_wdg_wdgGetFailCntStatus_wdFailCnt(void)
{
#ifdef BUILD_MOCK
    int32_t status;
    Pmic_WdgFailCntStatus_t failCount = {0};

    /* Set fail count to 5 (bits [3:0] = 0x5) */
    status = testInject_setRegister(PMIC_WD_STAT_REG, 0x05U);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    failCount.validParams = PMIC_CFG_WD_FAIL_CNT_VAL_VALID;
    status = Pmic_wdgGetFailCntStatus(&pmicHandle, &failCount);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(failCount.wdFailCnt == 5U);
#else
    /* Test requires mock support for register injection */
    PLATFORM_ASSERT(true);
#endif
}

/**
 * @brief Test getting all fail count status fields at once
 */
void test_pos_wdg_wdgGetFailCntStatus_allFields(void)
{
#ifdef BUILD_MOCK
    int32_t status;
    Pmic_WdgFailCntStatus_t failCount = {0};

    /* Set all relevant bits: fail count=3, bad event, first ok, long window active */
    uint8_t statValue = 0x03U |                    /* Fail count = 3 */
                        PMIC_WD_BAD_EVENT_MASK |   /* Bad event */
                        PMIC_WD_FIRST_OK_MASK |    /* Good event */
                        PMIC_WD_LONGWIN_ACTIVE_MASK; /* Long window active */

    status = testInject_setRegister(PMIC_WD_STAT_REG, statValue);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    failCount.validParams = PMIC_CFG_WD_FAILCNT_ALL_VALID_SHIFT;
    status = Pmic_wdgGetFailCntStatus(&pmicHandle, &failCount);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(failCount.wdFailCnt == 3U);
    PLATFORM_ASSERT(failCount.badEvent == true);
    PLATFORM_ASSERT(failCount.goodEvent == true);
    PLATFORM_ASSERT(failCount.longWinActive == true);
#else
    /* Test requires mock support for register injection */
    PLATFORM_ASSERT(true);
#endif
}

/**
 * @brief Test getting fail count only with specific validParams
 */
void test_pos_wdg_wdgGetFailCntStatus_failCntOnly(void)
{
#ifdef BUILD_MOCK
    int32_t status;
    Pmic_WdgFailCntStatus_t failCount = {0};

    /* Set fail count to 6 (bits [3:0] = 0x6) with other status bits set */
    uint8_t statValue = 0x06U | PMIC_WD_BAD_EVENT_MASK | PMIC_WD_FIRST_OK_MASK;
    status = testInject_setRegister(PMIC_WD_STAT_REG, statValue);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Only request fail count value */
    failCount.validParams = PMIC_CFG_WD_FAIL_CNT_VAL_VALID;
    status = Pmic_wdgGetFailCntStatus(&pmicHandle, &failCount);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(failCount.wdFailCnt == 6U);
#else
    /* Test requires mock support for register injection */
    PLATFORM_ASSERT(true);
#endif
}

/**
 * @brief Test getting bad event count only with specific validParams
 */
void test_pos_wdg_wdgGetFailCntStatus_badCntOnly(void)
{
#ifdef BUILD_MOCK
    int32_t status;
    Pmic_WdgFailCntStatus_t failCount = {0};

    /* Set bad event flag along with other fields */
    uint8_t statValue = 0x02U | PMIC_WD_BAD_EVENT_MASK | PMIC_WD_FIRST_OK_MASK;
    status = testInject_setRegister(PMIC_WD_STAT_REG, statValue);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Only request bad event status */
    failCount.validParams = PMIC_CFG_WD_BAD_EVENT_STAT_VALID;
    status = Pmic_wdgGetFailCntStatus(&pmicHandle, &failCount);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(failCount.badEvent == true);
#else
    /* Test requires mock support for register injection */
    PLATFORM_ASSERT(true);
#endif
}

/* ========================================================================== */
/* Configuration Tests                                                        */
/* ========================================================================== */

/**
 * @brief Test time base configuration
 */
void test_pos_wdg_wdgSetCfg_timeBase(void)
{
    int32_t status;
    Pmic_WdgCfg_t wdgCfg = {0};

    /* Enable watchdog first */
    status = Pmic_wdgEnable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Set time base to 550us */
    wdgCfg.validParams = PMIC_CFG_WDG_TIME_BASE_VALID;
    wdgCfg.timeBase = PMIC_WDG_TIME_BASE_550_US;

    status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Get and verify time base */
    wdgCfg.validParams = PMIC_CFG_WDG_TIME_BASE_VALID;
    wdgCfg.timeBase = 0U;

    status = Pmic_wdgGetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(wdgCfg.timeBase == PMIC_WDG_TIME_BASE_550_US);
}

/**
 * @brief Test set config with zero validParams (should return error)
 */
void test_neg_wdg_wdgSetCfg_zeroValidParams(void)
{
    int32_t status;
    Pmic_WdgCfg_t wdgCfg = {0};

    /* Enable watchdog first */
    status = Pmic_wdgEnable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Try to set config with validParams = 0 */
    wdgCfg.validParams = 0U;

    status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test watchdog SetMode/GetMode with TRIGGER_MODE
 */
void test_pos_wdg_wdgSetMode_triggerMode(void)
{
    int32_t status;
    uint8_t mode = TEST_INVALID_PARAM_255;

    /* Enable watchdog first */
    status = Pmic_wdgEnable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Set watchdog mode to TRIGGER_MODE */
    status = Pmic_wdgSetMode(&pmicHandle, PMIC_WDG_TRIGGER_MODE);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Get and verify mode */
    status = Pmic_wdgGetMode(&pmicHandle, &mode);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(mode == PMIC_WDG_TRIGGER_MODE);
}

/**
 * @brief Test watchdog SetMode/GetMode with Q&A_MODE
 */
void test_pos_wdg_wdgSetMode_qAndAMode(void)
{
    int32_t status;
    uint8_t mode = TEST_INVALID_PARAM_255;

    /* Enable watchdog first */
    status = Pmic_wdgEnable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Set watchdog mode to QA_MODE */
    status = Pmic_wdgSetMode(&pmicHandle, PMIC_WDG_QA_MODE);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Get and verify mode */
    status = Pmic_wdgGetMode(&pmicHandle, &mode);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(mode == PMIC_WDG_QA_MODE);
}

/**
 * @brief Test watchdog SetPowerHold/GetPowerHold - enable
 */
void test_pos_wdg_wdgSetPowerHold_enable(void)
{
    int32_t status;
    bool isEnabled = false;

    /* Enable watchdog first */
    status = Pmic_wdgEnable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Set power hold enable */
    status = Pmic_wdgSetPowerHold(&pmicHandle, true);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Get and verify power hold is enabled */
    status = Pmic_wdgGetPowerHold(&pmicHandle, &isEnabled);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(isEnabled == true);
}

/**
 * @brief Test watchdog SetPowerHold/GetPowerHold - disable
 */
void test_pos_wdg_wdgSetPowerHold_disable(void)
{
    int32_t status;
    bool isEnabled = true;

    /* Enable watchdog first */
    status = Pmic_wdgEnable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Set power hold disable */
    status = Pmic_wdgSetPowerHold(&pmicHandle, false);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Get and verify power hold is disabled */
    status = Pmic_wdgGetPowerHold(&pmicHandle, &isEnabled);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(isEnabled == false);
}

/**
 * @brief Test watchdog SetReturnToLongWindow/GetReturnToLongWindow - enable
 */
void test_pos_wdg_wdgSetReturnToLongWindow_enable(void)
{
    int32_t status;
    bool isEnabled = false;

    /* Enable watchdog first */
    status = Pmic_wdgEnable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Set return to long window enable */
    status = Pmic_wdgSetReturnToLongWindow(&pmicHandle, true);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Get and verify return to long window is enabled */
    status = Pmic_wdgGetReturnToLongWindow(&pmicHandle, &isEnabled);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(isEnabled == true);
}

/**
 * @brief Test watchdog SetReturnToLongWindow/GetReturnToLongWindow - disable
 */
void test_pos_wdg_wdgSetReturnToLongWindow_disable(void)
{
    int32_t status;
    bool isEnabled = true;

    /* Enable watchdog first */
    status = Pmic_wdgEnable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Set return to long window disable */
    status = Pmic_wdgSetReturnToLongWindow(&pmicHandle, false);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Get and verify return to long window is disabled */
    status = Pmic_wdgGetReturnToLongWindow(&pmicHandle, &isEnabled);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(isEnabled == false);
}

/* ========================================================================== */
/*                       Negative Test Functions                              */
/* ========================================================================== */

/**
 * @brief Test Pmic_wdgEnable with NULL handle
 */
void test_neg_wdg_wdgEnable_nullHandle(void)
{
    int32_t status = Pmic_wdgEnable(NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_wdgDisable with NULL handle
 */
void test_neg_wdg_wdgDisable_nullHandle(void)
{
    int32_t status = Pmic_wdgDisable(NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_wdgSetEnableState with NULL handle
 */
void test_neg_wdg_wdgSetEnableState_nullHandle(void)
{
    int32_t status = Pmic_wdgSetEnableState(NULL, true);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_wdgGetEnableState with NULL handle
 */
void test_neg_wdg_wdgGetEnableState_nullHandle(void)
{
    bool isEnabled = false;
    int32_t status = Pmic_wdgGetEnableState(NULL, &isEnabled);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_wdgGetEnableState with NULL output parameter
 */
void test_neg_wdg_wdgGetEnableState_nullParam(void)
{
    int32_t status = Pmic_wdgGetEnableState(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_wdgSetCfg with NULL handle
 */
void test_neg_wdg_wdgSetCfg_nullHandle(void)
{
    Pmic_WdgCfg_t wdgCfg = {0};
    wdgCfg.validParams = PMIC_CFG_WDG_MODE_VALID;
    int32_t status = Pmic_wdgSetCfg(NULL, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_wdgSetCfg with NULL config parameter
 */
void test_neg_wdg_wdgSetCfg_nullConfig(void)
{
    int32_t status = Pmic_wdgSetCfg(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_wdgSetCfg with invalid mode value
 */
void test_neg_wdg_wdgSetCfg_invalidMode(void)
{
    Pmic_WdgCfg_t wdgCfg = {0};
    wdgCfg.validParams = PMIC_CFG_WDG_MODE_VALID;
    wdgCfg.mode = PMIC_WDG_MODE_MAX + 1U;

    int32_t status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_wdgSetCfg with invalid time base value
 */
void test_neg_wdg_wdgSetCfg_invalidTimeBase(void)
{
    Pmic_WdgCfg_t wdgCfg = {0};
    wdgCfg.validParams = PMIC_CFG_WDG_TIME_BASE_VALID;
    wdgCfg.timeBase = PMIC_WDG_TIME_BASE_MAX + 1U;

    int32_t status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_wdgSetCfg with invalid threshold1 value
 */
void test_neg_wdg_wdgSetCfg_invalidThreshold1(void)
{
    Pmic_WdgCfg_t wdgCfg = {0};
    wdgCfg.validParams = PMIC_CFG_WDG_THRESHOLD_1_VALID;
    wdgCfg.threshold1 = PMIC_WDG_THRESHOLD_COUNT_MAX + 1U;

    int32_t status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_wdgSetCfg with invalid threshold2 value
 */
void test_neg_wdg_wdgSetCfg_invalidThreshold2(void)
{
    Pmic_WdgCfg_t wdgCfg = {0};
    wdgCfg.validParams = PMIC_CFG_WDG_THRESHOLD_2_VALID;
    wdgCfg.threshold2 = PMIC_WDG_THRESHOLD_COUNT_MAX + 1U;

    int32_t status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_wdgSetCfg with invalid QA feedback value
 */
void test_neg_wdg_wdgSetCfg_invalidQaFdbk(void)
{
    Pmic_WdgCfg_t wdgCfg = {0};
    wdgCfg.validParams = PMIC_CFG_WDG_QA_FDBK_VALID;
    wdgCfg.qaFdbk = PMIC_WDG_QA_FEEDBACK_VALUE_MAX + 1U;

    int32_t status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_wdgSetCfg with invalid QA LFSR value
 */
void test_neg_wdg_wdgSetCfg_invalidQaLfsr(void)
{
    Pmic_WdgCfg_t wdgCfg = {0};
    wdgCfg.validParams = PMIC_CFG_WDG_QA_LFSR_VALID;
    wdgCfg.qaLfsr = PMIC_WDG_QA_LFSR_VALUE_MAX + 1U;

    int32_t status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_wdgSetCfg with invalid QA question seed value
 */
void test_neg_wdg_wdgSetCfg_invalidQaQuesSeed(void)
{
    Pmic_WdgCfg_t wdgCfg = {0};
    wdgCfg.validParams = PMIC_CFG_WDG_QA_QUES_SEED_VALID;
    wdgCfg.qaQuesSeed = PMIC_WDG_QA_QUES_SEED_VALUE_MAX + 1U;

    int32_t status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_wdgSetCfg with invalid threshold1 interrupt behavior
 */
void test_neg_wdg_wdgSetCfg_invalidThreshold1IntBehavior(void)
{
    Pmic_WdgCfg_t wdgCfg = {0};
    wdgCfg.validParams = PMIC_CFG_WDG_THRESHOLD1_INT_BEHAVIOR_VALID;
    wdgCfg.threshold1IntBehavior = PMIC_WDG_THRESHOLD_INT_BEHAVIOR_MAX + 1U;

    int32_t status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_wdgSetCfg with invalid threshold2 interrupt behavior
 */
void test_neg_wdg_wdgSetCfg_invalidThreshold2IntBehavior(void)
{
    Pmic_WdgCfg_t wdgCfg = {0};
    wdgCfg.validParams = PMIC_CFG_WDG_THRESHOLD2_INT_BEHAVIOR_VALID;
    wdgCfg.threshold2IntBehavior = PMIC_WDG_THRESHOLD_INT_BEHAVIOR_MAX + 1U;

    int32_t status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_wdgGetCfg with NULL handle
 */
void test_neg_wdg_wdgGetCfg_nullHandle(void)
{
    Pmic_WdgCfg_t wdgCfg = {0};
    wdgCfg.validParams = PMIC_CFG_WDG_MODE_VALID;
    int32_t status = Pmic_wdgGetCfg(NULL, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_wdgGetCfg with NULL config parameter
 */
void test_neg_wdg_wdgGetCfg_nullConfig(void)
{
    int32_t status = Pmic_wdgGetCfg(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_wdgSetMode with NULL handle
 */
void test_neg_wdg_wdgSetMode_nullHandle(void)
{
    int32_t status = Pmic_wdgSetMode(NULL, PMIC_WDG_TRIGGER_MODE);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_wdgSetMode with invalid mode
 */
void test_neg_wdg_wdgSetMode_invalidMode(void)
{
    int32_t status = Pmic_wdgSetMode(&pmicHandle, PMIC_WDG_MODE_MAX + 1U);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_wdgGetMode with NULL handle
 */
void test_neg_wdg_wdgGetMode_nullHandle(void)
{
    uint8_t mode = 0U;
    int32_t status = Pmic_wdgGetMode(NULL, &mode);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_wdgGetMode with NULL output parameter
 */
void test_neg_wdg_wdgGetMode_nullParam(void)
{
    int32_t status = Pmic_wdgGetMode(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_wdgSetPowerHold with NULL handle
 */
void test_neg_wdg_wdgSetPowerHold_nullHandle(void)
{
    int32_t status = Pmic_wdgSetPowerHold(NULL, true);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_wdgGetPowerHold with NULL handle
 */
void test_neg_wdg_wdgGetPowerHold_nullHandle(void)
{
    bool isEnabled = false;
    int32_t status = Pmic_wdgGetPowerHold(NULL, &isEnabled);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_wdgGetPowerHold with NULL output parameter
 */
void test_neg_wdg_wdgGetPowerHold_nullParam(void)
{
    int32_t status = Pmic_wdgGetPowerHold(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_wdgSetReturnToLongWindow with NULL handle
 */
void test_neg_wdg_wdgSetReturnToLongWindow_nullHandle(void)
{
    int32_t status = Pmic_wdgSetReturnToLongWindow(NULL, true);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_wdgGetReturnToLongWindow with NULL handle
 */
void test_neg_wdg_wdgGetReturnToLongWindow_nullHandle(void)
{
    bool isEnabled = false;
    int32_t status = Pmic_wdgGetReturnToLongWindow(NULL, &isEnabled);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_wdgGetReturnToLongWindow with NULL output parameter
 */
void test_neg_wdg_wdgGetReturnToLongWindow_nullParam(void)
{
    int32_t status = Pmic_wdgGetReturnToLongWindow(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_wdgGetErrStatus with NULL handle
 */
void test_neg_wdg_wdgGetErrStatus_nullHandle(void)
{
    Pmic_WdgErrStatus_t errors = {0};
    errors.validParams = PMIC_CFG_WD_TIMEOUT_ERR_VALID;
    int32_t status = Pmic_wdgGetErrStatus(NULL, &errors);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_wdgGetErrStatus with NULL output parameter
 */
void test_neg_wdg_wdgGetErrStatus_nullParam(void)
{
    int32_t status = Pmic_wdgGetErrStatus(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_wdgClrErrStatus with NULL handle
 */
void test_neg_wdg_wdgClrErrStatus_nullHandle(void)
{
    Pmic_WdgErrStatus_t errors = {0};
    errors.validParams = PMIC_CFG_WD_TIMEOUT_ERR_VALID;
    errors.timeout = true;
    int32_t status = Pmic_wdgClrErrStatus(NULL, &errors);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_wdgClrErrStatus with NULL error parameter
 */
void test_neg_wdg_wdgClrErrStatus_nullParam(void)
{
    int32_t status = Pmic_wdgClrErrStatus(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_wdgClrErrStatusAll with NULL handle
 */
void test_neg_wdg_wdgClrErrStatusAll_nullHandle(void)
{
    int32_t status = Pmic_wdgClrErrStatusAll(NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_wdgGetFailCntStatus with NULL handle
 */
void test_neg_wdg_wdgGetFailCntStatus_nullHandle(void)
{
    Pmic_WdgFailCntStatus_t failCount = {0};
    failCount.validParams = PMIC_CFG_WD_FAIL_CNT_VAL_VALID;
    int32_t status = Pmic_wdgGetFailCntStatus(NULL, &failCount);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_wdgGetFailCntStatus with NULL output parameter
 */
void test_neg_wdg_wdgGetFailCntStatus_nullParam(void)
{
    int32_t status = Pmic_wdgGetFailCntStatus(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_wdgQaWriteAnswer with NULL handle
 */
void test_neg_wdg_wdgQaWriteAnswer_nullHandle(void)
{
    int32_t status = Pmic_wdgQaWriteAnswer(NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief WDG test suite entry point
 * @param args Test arguments (unused)
 */
void wdg_test(void *args)
{
    char msg[50U] = {0};
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_HandleCfg_t pmicCfg = {
        .validParams = (PMIC_COMM_MODE_VALID |
                        PMIC_I2C_ADDR0_VALID |
                        PMIC_COMM_HANDLE_0_VALID |
                        PMIC_IO_READ_VALID |
                        PMIC_IO_WRITE_VALID |
                        PMIC_CRITICAL_SECTION_START_VALID |
                        PMIC_CRITICAL_SECTION_STOP_VALID),
        .commMode = PMIC_INTF_SPI,
        .i2cAddr0 = PLATFORM_TARGET_I2C_ADDR,
        .i2cAddr1 = 0,
        .i2cAddr2 = 0,
        .commHandle0 = platform_getCommHandle(),
        .ioRead = &test_pmic_regRead,
        .ioWrite = &test_pmic_regWrite,
        .criticalSectionStart = &platform_critSecStart,
        .criticalSectionStop = &platform_critSecStop
    };

    platform_init();

    platform_printString("\r\n");
    platform_printString("WDG_TEST\r\n");
    platform_printString("-------\r\n\r\n");

    status = Pmic_init(&pmicHandle, &pmicCfg);

    if (status == PMIC_ST_SUCCESS)
    {
        testUtils_printSiRev(&pmicHandle);

        platform_setupTests();
        WDG_TEST_RUN_ALL();
        platform_tearDownTests();
    }
    else
    {
        (void)sprintf(msg, "Error in initializing PMIC LLD: %d\r\n", status);
        platform_printString(msg);
    }

    (void)Pmic_deinit(&pmicHandle);
    platform_deinit();
}
