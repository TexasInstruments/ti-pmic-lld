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
/**
 * @file platform.c
 * @brief Source file containing definitions to PMIC WDG tests.
 */

/* ========================================================================== */
/*                              Include Files                                 */
/* ========================================================================== */

#include "wdg_test.h"

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/* Run all WDG tests */
#define WDG_TEST_RUN_ALL() PLATFORM_RUN_TEST(test_negative_Pmic_wdgSetEnableState_nullParam_pmicHandle); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_wdgEnable_nullParam_pmicHandle); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_wdgDisable_nullParam_pmicHandle); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_wdgGetEnable_nullParam_pmicHandle); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_wdgGetEnable_nullParam_wdgEnabled); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_wdgSetCfg_nullParam_pmicHandle); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_wdgSetCfg_nullParam_wdgCfg); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_wdgSetCfg_outOfBounds_mode); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_wdgSetCfg_outOfBounds_trigSel); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_wdgSetCfg_outOfBounds_failThr); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_wdgSetCfg_outOfBounds_rstThr); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_wdgSetCfg_outOfBounds_win1Duration); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_wdgSetCfg_outOfBounds_win2Duration); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_wdgSetCfg_outOfBounds_qaFdbk); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_wdgSetCfg_outOfBounds_qaLfsr); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_wdgSetCfg_outOfBounds_qaSeed); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_wdgGetCfg_nullParam_pmicHandle); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_wdgGetCfg_nullParam_wdgCfg); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_wdgSetPwrHold_nullParam_pmicHandle); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_wdgGetPwrHold_nullParam_pmicHandle); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_wdgGetPwrHold_nullParam_pwrHoldStat); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_wdgSetRetLongWin_nullParam_pmicHandle); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_wdgGetRetLongWin_nullParam_pmicHandle); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_wdgGetRetLongWin_nullParam_retLongWinStat); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_wdgSendSwTrigger_nullParam_pmicHandle); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_wdgWriteAnswer_nullParam_pmicHandle); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_wdgClrErrStat_nullParam_pmicHandle); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_wdgClrErrStat_nullParam_wdgErrStat); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_wdgClrErrStatAll_nullParam_pmicHandle); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_wdgGetErrStat_nullParam_pmicHandle); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_wdgGetErrStat_nullParam_wdgErrStat); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_wdgGetFailCntStat_nullParam_pmicHandle); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_wdgGetFailCntStat_nullParam_wdgFailCntStat); \
                           PLATFORM_RUN_TEST(test_positive_wdgEnableDisable); \
                           PLATFORM_RUN_TEST(test_positive_wdgEnableDisablePowerHold); \
                           PLATFORM_RUN_TEST(test_positive_wdgEnableDisableReturnToLongWindow); \
                           PLATFORM_RUN_TEST(test_positive_wdgSetGetCfg_rstEn); \
                           PLATFORM_RUN_TEST(test_positive_wdgSetGetCfg_mode); \
                           PLATFORM_RUN_TEST(test_positive_wdgSetGetCfg_trigSel); \
                           PLATFORM_RUN_TEST(test_positive_wdgSetGetCfg_failThr); \
                           PLATFORM_RUN_TEST(test_positive_wdgSetGetCfg_rstThr); \
                           PLATFORM_RUN_TEST(test_positive_wdgSetGetCfg_longWinDuration); \
                           PLATFORM_RUN_TEST(test_positive_wdgSetGetCfg_win1Duration); \
                           PLATFORM_RUN_TEST(test_positive_wdgSetGetCfg_win2Duration); \
                           PLATFORM_RUN_TEST(test_positive_wdgSetGetCfg_qaFdbk); \
                           PLATFORM_RUN_TEST(test_positive_wdgSetGetCfg_qaLfsr); \
                           PLATFORM_RUN_TEST(test_positive_wdgSetGetCfg_qaSeed); \
                           PLATFORM_RUN_TEST(test_positive_wdgSwTrigger_detectNoErrors); \
                           PLATFORM_RUN_TEST(test_positive_wdgSwTrigger_detectTrigEarlyErr); \
                           PLATFORM_RUN_TEST(test_positive_wdgQaSequence_detectNoErrors); \
                           PLATFORM_RUN_TEST(test_positive_wdgQaSequence_detect_answErr); \
                           PLATFORM_RUN_TEST(test_positive_wdgQaSequence_detect_seqErr); \
                           PLATFORM_RUN_TEST(test_positive_wdgQaSequence_detect_AnswEarlyErr); \
                           PLATFORM_RUN_TEST(test_positive_wdgQaSequence_detect_timeoutErr); \
                           PLATFORM_RUN_TEST(test_positive_wdgQaSequence_detect_longWinTimeoutErr); \
                           PLATFORM_RUN_TEST(test_positive_wdgQaSequence_detect_failInt); \
                           PLATFORM_RUN_TEST(test_positive_wdgQaSequence_detect_RstInt)

/* Run all WDG negative tests */
#define WDG_TEST_RUN_NEGATIVE() PLATFORM_RUN_TEST(test_negative_Pmic_wdgSetEnableState_nullParam_pmicHandle); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_wdgEnable_nullParam_pmicHandle); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_wdgDisable_nullParam_pmicHandle); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_wdgGetEnable_nullParam_pmicHandle); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_wdgGetEnable_nullParam_wdgEnabled); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_wdgSetCfg_nullParam_pmicHandle); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_wdgSetCfg_nullParam_wdgCfg); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_wdgSetCfg_outOfBounds_mode); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_wdgSetCfg_outOfBounds_trigSel); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_wdgSetCfg_outOfBounds_failThr); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_wdgSetCfg_outOfBounds_rstThr); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_wdgSetCfg_outOfBounds_win1Duration); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_wdgSetCfg_outOfBounds_win2Duration); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_wdgSetCfg_outOfBounds_qaFdbk); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_wdgSetCfg_outOfBounds_qaLfsr); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_wdgSetCfg_outOfBounds_qaSeed); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_wdgGetCfg_nullParam_pmicHandle); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_wdgGetCfg_nullParam_wdgCfg); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_wdgSetPwrHold_nullParam_pmicHandle); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_wdgGetPwrHold_nullParam_pmicHandle); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_wdgGetPwrHold_nullParam_pwrHoldStat); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_wdgSetRetLongWin_nullParam_pmicHandle); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_wdgGetRetLongWin_nullParam_pmicHandle); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_wdgGetRetLongWin_nullParam_retLongWinStat); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_wdgSendSwTrigger_nullParam_pmicHandle); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_wdgWriteAnswer_nullParam_pmicHandle); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_wdgClrErrStat_nullParam_pmicHandle); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_wdgClrErrStat_nullParam_wdgErrStat); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_wdgClrErrStatAll_nullParam_pmicHandle); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_wdgGetErrStat_nullParam_pmicHandle); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_wdgGetErrStat_nullParam_wdgErrStat); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_wdgGetFailCntStat_nullParam_pmicHandle); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_wdgGetFailCntStat_nullParam_wdgFailCntStat)

/* Run all WDG positive tests */
#define WDG_TEST_RUN_POSITIVE() PLATFORM_RUN_TEST(test_positive_wdgEnableDisable); \
                                PLATFORM_RUN_TEST(test_positive_wdgEnableDisablePowerHold); \
                                PLATFORM_RUN_TEST(test_positive_wdgEnableDisableReturnToLongWindow); \
                                PLATFORM_RUN_TEST(test_positive_wdgSetGetCfg_rstEn); \
                                PLATFORM_RUN_TEST(test_positive_wdgSetGetCfg_mode); \
                                PLATFORM_RUN_TEST(test_positive_wdgSetGetCfg_trigSel); \
                                PLATFORM_RUN_TEST(test_positive_wdgSetGetCfg_failThr); \
                                PLATFORM_RUN_TEST(test_positive_wdgSetGetCfg_rstThr); \
                                PLATFORM_RUN_TEST(test_positive_wdgSetGetCfg_longWinDuration); \
                                PLATFORM_RUN_TEST(test_positive_wdgSetGetCfg_win1Duration); \
                                PLATFORM_RUN_TEST(test_positive_wdgSetGetCfg_win2Duration); \
                                PLATFORM_RUN_TEST(test_positive_wdgSetGetCfg_qaFdbk); \
                                PLATFORM_RUN_TEST(test_positive_wdgSetGetCfg_qaLfsr); \
                                PLATFORM_RUN_TEST(test_positive_wdgSetGetCfg_qaSeed); \
                                PLATFORM_RUN_TEST(test_positive_wdgSwTrigger_detectNoErrors); \
                                PLATFORM_RUN_TEST(test_positive_wdgSwTrigger_detectTrigEarlyErr); \
                                PLATFORM_RUN_TEST(test_positive_wdgQaSequence_detectNoErrors); \
                                PLATFORM_RUN_TEST(test_positive_wdgQaSequence_detect_answErr); \
                                PLATFORM_RUN_TEST(test_positive_wdgQaSequence_detect_seqErr); \
                                PLATFORM_RUN_TEST(test_positive_wdgQaSequence_detect_AnswEarlyErr); \
                                PLATFORM_RUN_TEST(test_positive_wdgQaSequence_detect_timeoutErr); \
                                PLATFORM_RUN_TEST(test_positive_wdgQaSequence_detect_longWinTimeoutErr); \
                                PLATFORM_RUN_TEST(test_positive_wdgQaSequence_detect_failInt); \
                                PLATFORM_RUN_TEST(test_positive_wdgQaSequence_detect_RstInt)

/* ========================================================================== */
/*                             Global Variables                               */
/* ========================================================================== */

static Pmic_CoreHandle_t pmicHandle = {0U};

/* ========================================================================== */
/*                           Function Declarations                            */
/* ========================================================================== */

static void wdgTest_checkForWdgErrors(void);

/* ========================================================================== */
/*                           Function Definitions                             */
/* ========================================================================== */

void wdg_test(void *args)
{
    char msg[50U] = {0};
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_CoreCfg_t pmicCfg = {
        .i2cAddr = PLATFORM_TARGET_I2C_ADDR,
        .commHandle = platform_getCommHandle(),
        .ioRead = &platform_rxByte,
        .ioWrite = &platform_txByte,
        .critSecStart = &platform_critSecStart,
        .critSecStop = &platform_critSecStop,
        .irqResponse = &platform_irqResponse
    };

    platform_init();

    platform_printString("\r\n");
    platform_printString("WDG_TEST\r\n");
    platform_printString("-------\r\n\r\n");

    status = Pmic_init(&pmicCfg, &pmicHandle);

    if (status == PMIC_ST_SUCCESS)
    {
        status = testCommon_unlockPmicRegs(&pmicHandle);

        if (status == PMIC_ST_SUCCESS)
        {
            status = testCommon_clrAllPmicIrq(&pmicHandle);

            if (status == PMIC_ST_SUCCESS)
            {
                platform_setupTests();
                WDG_TEST_RUN_ALL();
                platform_tearDownTests();
            }
            else
            {
                (void)sprintf(msg, "Error in clearing all PMIC IRQs: %d\r\n", status);
                platform_printString(msg);
            }
        }
        else
        {
            (void)sprintf(msg, "Error in unlocking PMIC registers: %d\r\n", status);
            platform_printString(msg);
        }
    }
    else
    {
        (void)sprintf(msg, "Error in initializing PMIC LLD: %d\r\n", status);
        platform_printString(msg);
    }

    (void)Pmic_deinit(&pmicHandle);
    platform_deinit();
}

void test_negative_Pmic_wdgSetEnableState_nullParam_pmicHandle(void)
{
    // Pass NULL pmicHandle into Pmic_wdgSetEnableState()
    int32_t status = Pmic_wdgSetEnableState(NULL, PMIC_DISABLE);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_wdgEnable_nullParam_pmicHandle(void)
{
    // Pass NULL pmicHandle into Pmic_wdgEnable()
    int32_t status = Pmic_wdgEnable(NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_wdgDisable_nullParam_pmicHandle(void)
{
    // Pass NULL pmicHandle into Pmic_wdgDisable()
    int32_t status = Pmic_wdgDisable(NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_wdgGetEnable_nullParam_pmicHandle(void)
{
    // Pass NULL pmicHandle into Pmic_wdgGetEnable()
    bool wdgEnabled = PMIC_DISABLE;
    int32_t status = Pmic_wdgGetEnable(NULL, &wdgEnabled);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_wdgGetEnable_nullParam_wdgEnabled(void)
{
    // Pass NULL wdgEnabled into Pmic_wdgGetEnable()
    int32_t status = Pmic_wdgGetEnable(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_wdgSetCfg_nullParam_pmicHandle(void)
{
    // Pass NULL pmicHandle into Pmic_wdgSetCfg()
    Pmic_WdgCfg_t wdgCfg = {
        .validParams = PMIC_WD_RST_EN_VALID,
        .rstEn = PMIC_DISABLE
    };
    int32_t status = Pmic_wdgSetCfg(NULL, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_wdgSetCfg_nullParam_wdgCfg(void)
{
    int32_t status = Pmic_wdgSetCfg(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_wdgSetCfg_outOfBounds_mode(void)
{
    // Pass out of bounds mode into Pmic_wdgSetCfg()
    Pmic_WdgCfg_t wdgCfg = {
        .validParams = PMIC_WD_MODE_VALID,
        .mode = PMIC_WD_MODE_MAX + 1U
    };
    int32_t status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_negative_Pmic_wdgSetCfg_outOfBounds_trigSel(void)
{
    // Pass out of bounds trigSel into Pmic_wdgSetCfg()
    Pmic_WdgCfg_t wdgCfg = {
        .validParams = PMIC_WD_TRIG_SEL_VALID,
        .trigSel = PMIC_TRIG_SEL_MAX + 1U
    };
    int32_t status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_negative_Pmic_wdgSetCfg_outOfBounds_failThr(void)
{
    // Pass out of bounds failThr into Pmic_wdgSetCfg()
    Pmic_WdgCfg_t wdgCfg = {
        .validParams = PMIC_WD_FAIL_THR_VALID,
        .failThr = PMIC_WD_FAIL_THR_MAX + 1U
    };
    int32_t status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_negative_Pmic_wdgSetCfg_outOfBounds_rstThr(void)
{
    // Pass out of bounds rstThr into Pmic_wdgSetCfg()
    Pmic_WdgCfg_t wdgCfg = {
        .validParams = PMIC_WD_RST_THR_VALID,
        .rstThr = PMIC_WD_RST_THR_MAX + 1U
    };
    int32_t status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_negative_Pmic_wdgSetCfg_outOfBounds_win1Duration(void)
{
    // Pass out of bounds win1Duration into Pmic_wdgSetCfg()
    Pmic_WdgCfg_t wdgCfg = {
        .validParams = PMIC_WD_WIN1_DURATION_VALID,
        .win1Duration = PMIC_WD_WIN1_DURATION_MAX + 1U
    };
    int32_t status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_negative_Pmic_wdgSetCfg_outOfBounds_win2Duration(void)
{
    // Pass out of bounds win2Duration into Pmic_wdgSetCfg()
    Pmic_WdgCfg_t wdgCfg = {
        .validParams = PMIC_WD_WIN2_DURATION_VALID,
        .win2Duration = PMIC_WD_WIN2_DURATION_MAX + 1U
    };
    int32_t status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_negative_Pmic_wdgSetCfg_outOfBounds_qaFdbk(void)
{
    // Pass out of bounds qaFdbk into Pmic_wdgSetCfg()
    Pmic_WdgCfg_t wdgCfg = {
        .validParams = PMIC_WD_QA_FDBK_VALID,
        .qaFdbk = PMIC_WD_QA_FDBK_MAX + 1U
    };
    int32_t status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_negative_Pmic_wdgSetCfg_outOfBounds_qaLfsr(void)
{
    // Pass out of bounds qaLfsr into Pmic_wdgSetCfg()
    Pmic_WdgCfg_t wdgCfg = {
        .validParams = PMIC_WD_QA_LFSR_VALID,
        .qaLfsr = PMIC_WD_QA_LFSR_MAX + 1U
    };
    int32_t status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_negative_Pmic_wdgSetCfg_outOfBounds_qaSeed(void)
{
    // Pass out of bounds qaSeed into Pmic_wdgSetCfg()
    Pmic_WdgCfg_t wdgCfg = {
        .validParams = PMIC_WD_QA_SEED_VALID,
        .qaSeed = PMIC_WD_QA_SEED_MAX + 1U
    };
    int32_t status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_negative_Pmic_wdgGetCfg_nullParam_pmicHandle(void)
{
    // Pass NULL pmicHandle into Pmic_wdgGetCfg
    Pmic_WdgCfg_t wdgCfg = {
        .validParams = PMIC_WD_RST_EN_VALID,
        .rstEn = PMIC_DISABLE
    };
    int32_t status = Pmic_wdgGetCfg(NULL, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_wdgGetCfg_nullParam_wdgCfg(void)
{
    // Pass NULL wdgCfg into Pmic_wdgGetCfg()
    int32_t status = Pmic_wdgGetCfg(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_wdgSetPwrHold_nullParam_pmicHandle(void)
{
    // Pass NULL pmicHandle into Pmic_wdgSetPwrHold()
    int32_t status = Pmic_wdgSetPwrHold(NULL, PMIC_DISABLE);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_wdgGetPwrHold_nullParam_pmicHandle(void)
{
    // Pass NULL pmicHandle into Pmic_wdgGetPwrHold()
    bool pwrHoldStat = PMIC_DISABLE;
    int32_t status = Pmic_wdgGetPwrHold(NULL, &pwrHoldStat);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_wdgGetPwrHold_nullParam_pwrHoldStat(void)
{
    // Pass NULL pwrHoldStat into Pmic_wdgGetPwrHold()
    int32_t status = Pmic_wdgGetPwrHold(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_wdgSetRetLongWin_nullParam_pmicHandle(void)
{
    // Pass NULL pmicHandle into Pmic_wdgSetRetLongWin()
    int32_t status = Pmic_wdgSetRetLongWin(NULL, PMIC_DISABLE);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_wdgGetRetLongWin_nullParam_pmicHandle(void)
{
    // Pass NULL pmicHandle into Pmic_wdgGetRetLongWin()
    bool retLongWinStat = PMIC_DISABLE;
    int32_t status = Pmic_wdgGetRetLongWin(NULL, &retLongWinStat);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_wdgGetRetLongWin_nullParam_retLongWinStat(void)
{
    // Pass NULL retLongWinStat into Pmic_wdgGetRetLongWin()
    int32_t status = Pmic_wdgGetRetLongWin(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_wdgSendSwTrigger_nullParam_pmicHandle(void)
{
    // Pass NULL pmicHandle into Pmic_wdgSendSwTrigger()
    int32_t status = Pmic_wdgSendSwTrigger(NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_wdgWriteAnswer_nullParam_pmicHandle(void)
{
    // Pass NULL pmicHandle into Pmic_wdgWriteAnswer()
    int32_t status = Pmic_wdgWriteAnswer(NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_wdgClrErrStat_nullParam_pmicHandle(void)
{
    // Pass NULL pmicHandle into Pmic_wdgClrErrStat()
    Pmic_WdgErrStat_t wdgErrStat = {
        .validParams = PMIC_WDG_ANSW_EARLY_ERR_VALID
    };
    int32_t status = Pmic_wdgClrErrStat(NULL, &wdgErrStat);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_wdgClrErrStat_nullParam_wdgErrStat(void)
{
    // Pass NULL wdgErrStat into Pmic_wdgClrErrStat()
    int32_t status = Pmic_wdgClrErrStat(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_wdgClrErrStatAll_nullParam_pmicHandle(void)
{
    // Pass NULL pmicHandle into Pmic_wdgClrErrStatAll()
    int32_t status = Pmic_wdgClrErrStatAll(NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_wdgGetErrStat_nullParam_pmicHandle(void)
{
    // Pass NULL pmicHandle into Pmic_wdgGetErrStat()
    Pmic_WdgErrStat_t wdgErrStat = {
        .validParams = PMIC_WDG_ANSW_EARLY_ERR_VALID
    };
    int32_t status = Pmic_wdgGetErrStat(NULL, &wdgErrStat);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_wdgGetErrStat_nullParam_wdgErrStat(void)
{
    // Pass NULL wdgErrStat into Pmic_wdgGetErrStat()
    int32_t status = Pmic_wdgGetErrStat(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_wdgGetFailCntStat_nullParam_pmicHandle(void)
{
    // Pass NULL pmicHandle into Pmic_wdgGetFailCntStat()
    Pmic_WdgFailCntStat_t wdgFailCntStat = {
        .validParams = PMIC_FAIL_CNT_VALID
    };
    int32_t status = Pmic_wdgGetFailCntStat(NULL, &wdgFailCntStat);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_wdgGetFailCntStat_nullParam_wdgFailCntStat(void)
{
    // Pass NULL wdgFailCntStat into Pmic_wdgGetFailCntStat()
    int32_t status = Pmic_wdgGetFailCntStat(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_positive_wdgEnableDisable(void)
{
    /* Test WDG enable/disable */
    bool isEnabled = PMIC_DISABLE;

    int32_t status = Pmic_wdgDisable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_wdgGetEnable(&pmicHandle, &isEnabled);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(isEnabled == PMIC_DISABLE);

    status = Pmic_wdgEnable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_wdgGetEnable(&pmicHandle, &isEnabled);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(isEnabled == PMIC_ENABLE);
}

void test_positive_wdgEnableDisablePowerHold(void)
{
    /* Test WDG power hold enable/disable */
    bool isEnabled = PMIC_DISABLE;

    int32_t status = Pmic_wdgSetPwrHold(&pmicHandle, PMIC_DISABLE);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_wdgGetPwrHold(&pmicHandle, &isEnabled);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(isEnabled == PMIC_DISABLE);

    status = Pmic_wdgSetPwrHold(&pmicHandle, PMIC_ENABLE);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_wdgGetPwrHold(&pmicHandle, &isEnabled);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(isEnabled == PMIC_ENABLE);
}

void test_positive_wdgEnableDisableReturnToLongWindow(void)
{
    /* Test WDG return to long window enable/disable */
    bool isEnabled = PMIC_DISABLE;

    int32_t status = Pmic_wdgSetRetLongWin(&pmicHandle, PMIC_DISABLE);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_wdgGetRetLongWin(&pmicHandle, &isEnabled);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(isEnabled == PMIC_DISABLE);

    status = Pmic_wdgSetRetLongWin(&pmicHandle, PMIC_ENABLE);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_wdgGetRetLongWin(&pmicHandle, &isEnabled);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(isEnabled == PMIC_ENABLE);
}

void test_positive_wdgSetGetCfg_rstEn(void)
{
    /* Test WDG reset enable enable/disable */
    Pmic_WdgCfg_t expWdgCfg = {.validParams = PMIC_WD_RST_EN_VALID};
    Pmic_WdgCfg_t actWdgCfg = {.validParams = PMIC_WD_RST_EN_VALID};

    expWdgCfg.rstEn = PMIC_ENABLE;
    int32_t status = Pmic_wdgSetCfg(&pmicHandle, &expWdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_wdgGetCfg(&pmicHandle, &actWdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(actWdgCfg.rstEn == PMIC_ENABLE);

    expWdgCfg.rstEn = PMIC_DISABLE;
    status = Pmic_wdgSetCfg(&pmicHandle, &expWdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_wdgGetCfg(&pmicHandle, &actWdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(actWdgCfg.rstEn == PMIC_DISABLE);
}

void test_positive_wdgSetGetCfg_mode(void)
{
    /* Test all valid WDG modes */
    Pmic_WdgCfg_t expWdgCfg = {.validParams = PMIC_WD_MODE_VALID};
    Pmic_WdgCfg_t actWdgCfg = {.validParams = PMIC_WD_MODE_VALID};
    int32_t status = PMIC_ST_SUCCESS;

    for (uint8_t mode = PMIC_TRIGGER_MODE; mode <= PMIC_WD_MODE_MAX; mode++)
    {
        expWdgCfg.mode = mode;
        status = Pmic_wdgSetCfg(&pmicHandle, &expWdgCfg);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        status = Pmic_wdgGetCfg(&pmicHandle, &actWdgCfg);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        PLATFORM_ASSERT(mode == actWdgCfg.mode);
    }
}

void test_positive_wdgSetGetCfg_trigSel(void)
{
    /* Test all valid WDG trigger selections */
    Pmic_WdgCfg_t expWdgCfg = {.validParams = PMIC_WD_TRIG_SEL_VALID};
    Pmic_WdgCfg_t actWdgCfg = {.validParams = PMIC_WD_TRIG_SEL_VALID};
    int32_t status = PMIC_ST_SUCCESS;

    for (uint8_t trigSel = PMIC_SW_TRIGGER; trigSel <= PMIC_TRIG_SEL_MAX; trigSel++)
    {
        expWdgCfg.trigSel = trigSel;
        status = Pmic_wdgSetCfg(&pmicHandle, &expWdgCfg);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        status = Pmic_wdgGetCfg(&pmicHandle, &actWdgCfg);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        PLATFORM_ASSERT(trigSel == actWdgCfg.trigSel);
    }
}

void test_positive_wdgSetGetCfg_failThr(void)
{
    /* Test all valid WDG failure thresholds */
    Pmic_WdgCfg_t expWdgCfg = {.validParams = PMIC_WD_FAIL_THR_VALID};
    Pmic_WdgCfg_t actWdgCfg = {.validParams = PMIC_WD_FAIL_THR_VALID};
    int32_t status = PMIC_ST_SUCCESS;

    for (uint8_t failThr = 0U; failThr <= PMIC_WD_FAIL_THR_MAX; failThr++)
    {
        expWdgCfg.failThr = failThr;
        status = Pmic_wdgSetCfg(&pmicHandle, &expWdgCfg);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        status = Pmic_wdgGetCfg(&pmicHandle, &actWdgCfg);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        PLATFORM_ASSERT(failThr == actWdgCfg.failThr);
    }
}

void test_positive_wdgSetGetCfg_rstThr(void)
{
    /* Test all valid WDG reset thresholds */
    Pmic_WdgCfg_t expWdgCfg = {.validParams = PMIC_WD_RST_THR_VALID};
    Pmic_WdgCfg_t actWdgCfg = {.validParams = PMIC_WD_RST_THR_VALID};
    int32_t status = PMIC_ST_SUCCESS;

    for (uint8_t rstThr = 0U; rstThr <= PMIC_WD_FAIL_THR_MAX; rstThr++)
    {
        expWdgCfg.rstThr = rstThr;
        status = Pmic_wdgSetCfg(&pmicHandle, &expWdgCfg);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        status = Pmic_wdgGetCfg(&pmicHandle, &actWdgCfg);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        PLATFORM_ASSERT(rstThr == actWdgCfg.rstThr);
    }
}

void test_positive_wdgSetGetCfg_longWinDuration(void)
{
    /* Test all valid WDG long window durations */
    Pmic_WdgCfg_t expWdgCfg = {.validParams = PMIC_WD_LONG_WIN_DURATION_VALID};
    Pmic_WdgCfg_t actWdgCfg = {.validParams = PMIC_WD_LONG_WIN_DURATION_VALID};
    int32_t status = PMIC_ST_SUCCESS;

    for (uint16_t longWinDuration = 0U; longWinDuration <= 0xFFU; longWinDuration++)
    {
        expWdgCfg.longWinDuration = (uint8_t)longWinDuration;
        status = Pmic_wdgSetCfg(&pmicHandle, &expWdgCfg);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        status = Pmic_wdgGetCfg(&pmicHandle, &actWdgCfg);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        PLATFORM_ASSERT(longWinDuration == actWdgCfg.longWinDuration);
    }
}

void test_positive_wdgSetGetCfg_win1Duration(void)
{
    /* Test all valid window-1 durations */
    Pmic_WdgCfg_t expWdgCfg = {.validParams = PMIC_WD_WIN1_DURATION_VALID};
    Pmic_WdgCfg_t actWdgCfg = {.validParams = PMIC_WD_WIN1_DURATION_VALID};
    int32_t status = PMIC_ST_SUCCESS;

    for (uint8_t win1Duration = 0U; win1Duration <= PMIC_WD_WIN1_DURATION_MAX; win1Duration++)
    {
        expWdgCfg.win1Duration = win1Duration;
        status = Pmic_wdgSetCfg(&pmicHandle, &expWdgCfg);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        status = Pmic_wdgGetCfg(&pmicHandle, &actWdgCfg);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        PLATFORM_ASSERT(win1Duration == actWdgCfg.win1Duration);
    }
}

void test_positive_wdgSetGetCfg_win2Duration(void)
{
    /* Test all valid window-2 durations */
    Pmic_WdgCfg_t expWdgCfg = {.validParams = PMIC_WD_WIN2_DURATION_VALID};
    Pmic_WdgCfg_t actWdgCfg = {.validParams = PMIC_WD_WIN2_DURATION_VALID};
    int32_t status = PMIC_ST_SUCCESS;

    for (uint8_t win2Duration = 0U; win2Duration <= PMIC_WD_WIN2_DURATION_MAX; win2Duration++)
    {
        expWdgCfg.win2Duration = win2Duration;
        status = Pmic_wdgSetCfg(&pmicHandle, &expWdgCfg);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        status = Pmic_wdgGetCfg(&pmicHandle, &actWdgCfg);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        PLATFORM_ASSERT(win2Duration == actWdgCfg.win2Duration);
    }
}

void test_positive_wdgSetGetCfg_qaFdbk(void)
{
    /* Test all valid WDG Q&A feedback values */
    Pmic_WdgCfg_t expWdgCfg = {.validParams = PMIC_WD_QA_FDBK_VALID};
    Pmic_WdgCfg_t actWdgCfg = {.validParams = PMIC_WD_QA_FDBK_VALID};
    int32_t status = PMIC_ST_SUCCESS;

    for (uint8_t qaFdbk = 0U; qaFdbk <= PMIC_WD_QA_FDBK_MAX; qaFdbk++)
    {
        expWdgCfg.qaFdbk = qaFdbk;
        status = Pmic_wdgSetCfg(&pmicHandle, &expWdgCfg);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        status = Pmic_wdgGetCfg(&pmicHandle, &actWdgCfg);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        PLATFORM_ASSERT(qaFdbk == actWdgCfg.qaFdbk);
    }
}

void test_positive_wdgSetGetCfg_qaLfsr(void)
{
    /* Test all valid WDG Q&A LFSR values */
    Pmic_WdgCfg_t expWdgCfg = {.validParams = PMIC_WD_QA_LFSR_VALID};
    Pmic_WdgCfg_t actWdgCfg = {.validParams = PMIC_WD_QA_LFSR_VALID};
    int32_t status = PMIC_ST_SUCCESS;

    for (uint8_t qaLfsr = 0U; qaLfsr <= PMIC_WD_QA_LFSR_MAX; qaLfsr++)
    {
        expWdgCfg.qaLfsr = qaLfsr;
        status = Pmic_wdgSetCfg(&pmicHandle, &expWdgCfg);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        status = Pmic_wdgGetCfg(&pmicHandle, &actWdgCfg);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        PLATFORM_ASSERT(qaLfsr == actWdgCfg.qaLfsr);
    }
}

void test_positive_wdgSetGetCfg_qaSeed(void)
{
    /* Test all valid WDG Q&A seeds */
    Pmic_WdgCfg_t expWdgCfg = {.validParams = PMIC_WD_QA_SEED_VALID};
    Pmic_WdgCfg_t actWdgCfg = {.validParams = PMIC_WD_QA_SEED_VALID};
    int32_t status = PMIC_ST_SUCCESS;

    for (uint8_t qaSeed = 0U; qaSeed <= PMIC_WD_QA_SEED_MAX; qaSeed++)
    {
        expWdgCfg.qaSeed = qaSeed;
        status = Pmic_wdgSetCfg(&pmicHandle, &expWdgCfg);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        status = Pmic_wdgGetCfg(&pmicHandle, &actWdgCfg);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        PLATFORM_ASSERT(qaSeed == actWdgCfg.qaSeed);
    }
}

static void wdgTest_checkForWdgErrors(void)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;
    const uint8_t wdErrStatusRegAddr = 0x62U, bufLen = 1U;

    status = platform_rxByte(&pmicHandle, wdErrStatusRegAddr, bufLen, &regData);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(regData == 0U);
}

void test_positive_wdgSwTrigger_detectNoErrors(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_WdgCfg_t wdgCfg = {
        .validParams = PMIC_WD_CFG_TRIGGER_VALID_ALL,
        .rstEn = PMIC_DISABLE,
        .mode = PMIC_TRIGGER_MODE,
        .trigSel = PMIC_SW_TRIGGER,
        .failThr = PMIC_WD_FAIL_THR_MAX,
        .rstThr = PMIC_WD_RST_THR_MAX,
        .longWinDuration = 0xFFU, // ~13 minutes
        .win1Duration = PMIC_WD_WIN1_DURATION_MAX, // 70.4 ms
        .win2Duration = PMIC_WD_WIN2_DURATION_MAX  // 70.4 ms
    };

    // Enable Watchdog and clear all watchdog statuses
    status = Pmic_wdgEnable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    status = Pmic_wdgClrErrStatAll(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Configure watchdog
    status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Disable power hold and disable return to long window to begin trigger sequences
    status = Pmic_wdgSetPwrHold(&pmicHandle, PMIC_DISABLE);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    status = Pmic_wdgSetRetLongWin(&pmicHandle, PMIC_DISABLE);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Exit long window by sending SW trigger
    status = Pmic_wdgSendSwTrigger(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    wdgTest_checkForWdgErrors();

    // Undergo trigger sequences
    for (uint16_t numSeqeunces = 20U; numSeqeunces != 0U; numSeqeunces--)
    {
        // Upon last iteration, indicate that we want to return to long window
        if (numSeqeunces == 1U)
        {
            status = Pmic_wdgSetRetLongWin(&pmicHandle, PMIC_ENABLE);
            PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        }

        // Enter Window-1; wait entire duration of Window-1 to enter Window-2
        platform_timerWaitMs(71U);

        // Enter Window-2; send SW trigger; check for any WDG errors
        status = Pmic_wdgSendSwTrigger(&pmicHandle);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        wdgTest_checkForWdgErrors();

        // End of trigger sequence; next sequence will begin
    }

    // WDG has returned to long Window; set WD_PWRHOLD so that WDG remains in long Window
    status = Pmic_wdgSetPwrHold(&pmicHandle, PMIC_ENABLE);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_positive_wdgSwTrigger_detectTrigEarlyErr(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_WdgCfg_t wdgCfg = {
        .validParams = PMIC_WD_CFG_TRIGGER_VALID_ALL,
        .rstEn = PMIC_DISABLE,
        .mode = PMIC_TRIGGER_MODE,
        .trigSel = PMIC_SW_TRIGGER,
        .failThr = PMIC_WD_FAIL_THR_MAX,
        .rstThr = PMIC_WD_RST_THR_MAX,
        .longWinDuration = 0xFFU, // ~13 minutes
        .win1Duration = PMIC_WD_WIN1_DURATION_MAX, // 70.4 ms
        .win2Duration = PMIC_WD_WIN2_DURATION_MAX  // 70.4 ms
    };
    Pmic_WdgErrStat_t wdgErrStat = {.validParams = PMIC_WDG_TRIG_EARLY_ERR_VALID};

    // Enable Watchdog and clear all watchdog statuses
    status = Pmic_wdgEnable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    status = Pmic_wdgClrErrStatAll(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Configure watchdog
    status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Disable power hold and disable return to long window to begin trigger sequences
    status = Pmic_wdgSetPwrHold(&pmicHandle, PMIC_DISABLE);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    status = Pmic_wdgSetRetLongWin(&pmicHandle, PMIC_DISABLE);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Exit long Window by sending SW trigger
    status = Pmic_wdgSendSwTrigger(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    wdgTest_checkForWdgErrors();

    // Enable return to long window
    status = Pmic_wdgSetRetLongWin(&pmicHandle, PMIC_ENABLE);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Enter Window-1; send SW trigger to incur WD_TRIG_EARLY error
    status = Pmic_wdgSendSwTrigger(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Wait until Window-1 elapses
    platform_timerWaitMs(71U);

    // Enter Window-2; wait until duration of Window-2 elapses to end sequence
    platform_timerWaitMs(71U);

    // WDG has returned to long Window; set WD_PWRHOLD so that WDG remains in long Window
    status = Pmic_wdgSetPwrHold(&pmicHandle, PMIC_ENABLE);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Validate WD_TRIG_EARLY error is set
    status = Pmic_wdgGetErrStat(&pmicHandle, &wdgErrStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(wdgErrStat.trigEarlyErr == (bool)true);

    // Clear the error and validate that it is cleared
    status = Pmic_wdgClrErrStat(&pmicHandle, &wdgErrStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    status = Pmic_wdgGetErrStat(&pmicHandle, &wdgErrStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(wdgErrStat.trigEarlyErr == (bool)false);
}

void test_positive_wdgQaSequence_detectNoErrors(void)
{
    uint8_t answerCnt = 0U;
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_WdgCfg_t wdgCfg = {
        .validParams = PMIC_WD_CFG_QA_VALID_ALL,
        .rstEn = PMIC_DISABLE,
        .mode = PMIC_QA_MODE,
        .failThr = PMIC_WD_FAIL_THR_MAX,
        .rstThr = PMIC_WD_RST_THR_MAX,
        .longWinDuration = 0xFFU, // ~13 minutes
        .win1Duration = PMIC_WD_WIN1_DURATION_MAX, // 70.4 ms
        .win2Duration = PMIC_WD_WIN2_DURATION_MAX, // 70.4 ms
        .qaFdbk = 0U,
        .qaLfsr = 1U,
        .qaSeed = 2U
    };

    // Enable Watchdog and clear all watchdog statuses
    status = Pmic_wdgEnable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    status = Pmic_wdgClrErrStatAll(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Configure watchdog
    status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Disable power hold and disable return to long window to begin Q&A sequences
    status = Pmic_wdgSetPwrHold(&pmicHandle, PMIC_DISABLE);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    status = Pmic_wdgSetRetLongWin(&pmicHandle, PMIC_DISABLE);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Exit long window by sending all 4 answer bytes
    for (answerCnt = 4U; answerCnt != 0U; answerCnt--)
    {
        status = Pmic_wdgWriteAnswer(&pmicHandle);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        wdgTest_checkForWdgErrors();
    }

    // Undergo Q&A sequences
    for (uint16_t numSeqeunces = 20U; numSeqeunces != 0U; numSeqeunces--)
    {
        // Upon last iteration, indicate that we want to return to long window
        if (numSeqeunces == 1U)
        {
            status = Pmic_wdgSetRetLongWin(&pmicHandle, PMIC_ENABLE);
            PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        }

        // Enter Window-1; calculate and send answer bytes Answer-3, Answer-2,
        // and Answer-1; check for any WDG errors
        for (answerCnt = 3U; answerCnt >= 1U; answerCnt--)
        {
            status = Pmic_wdgWriteAnswer(&pmicHandle);
            PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
            wdgTest_checkForWdgErrors();
        }

        // Wait until Window-1 time elapses
        platform_timerWaitMs(71U);

        // Enter Window-2; calculate and send last answer byte; check for any WDG errors
        status = Pmic_wdgWriteAnswer(&pmicHandle);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        wdgTest_checkForWdgErrors();

        // End of Q&A sequence; next question will be
        // generated and the next sequence will begin
    }

    // WDG has returned to long window; set WD_PWRHOLD so that WDG remains in long window
    status = Pmic_wdgSetPwrHold(&pmicHandle, PMIC_ENABLE);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_positive_wdgQaSequence_detect_answErr(void)
{
    const uint8_t bufLen = 1U;
    const uint16_t wdAnswerReg = 0x0EU;
    uint8_t answerCnt = 0U, regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_WdgCfg_t wdgCfg = {
        .validParams = PMIC_WD_CFG_QA_VALID_ALL,
        .rstEn = PMIC_DISABLE,
        .mode = PMIC_QA_MODE,
        .failThr = PMIC_WD_FAIL_THR_MAX,
        .rstThr = PMIC_WD_RST_THR_MAX,
        .longWinDuration = 0xFFU, // ~13 minutes
        .win1Duration = PMIC_WD_WIN1_DURATION_MAX, // 70.4 ms
        .win2Duration = PMIC_WD_WIN2_DURATION_MAX, // 70.4 ms
        .qaFdbk = 0U,
        .qaLfsr = 1U,
        .qaSeed = 2U
    };
    Pmic_WdgErrStat_t wdgErrStat = {.validParams = PMIC_WDG_ANSW_ERR_VALID};

    // Enable Watchdog and clear all watchdog statuses
    status = Pmic_wdgEnable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    status = Pmic_wdgClrErrStatAll(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Configure watchdog
    status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Disable power hold and disable return to long window to begin Q&A sequences
    status = Pmic_wdgSetPwrHold(&pmicHandle, PMIC_DISABLE);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    status = Pmic_wdgSetRetLongWin(&pmicHandle, PMIC_DISABLE);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Exit long window by sending all 4 answer bytes
    for (answerCnt = 4U; answerCnt != 0U; answerCnt--)
    {
        status = Pmic_wdgWriteAnswer(&pmicHandle);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        wdgTest_checkForWdgErrors();
    }

    // Enter Window-1; enable return to long window
    status = Pmic_wdgSetRetLongWin(&pmicHandle, PMIC_ENABLE);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Send incorrect Answer-3 but correct Answer-2 and Answer-1 to incur WD_ANSW_ERR
    status = platform_txByte(&pmicHandle, wdAnswerReg, bufLen, &regData); // Answer-3
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    for (answerCnt = 2U; answerCnt >= 1U; answerCnt--)
    {
        status = Pmic_wdgWriteAnswer(&pmicHandle);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    }

    // Wait until Window-1 duration is elapsed
    platform_timerWaitMs(71U);

    // Enter Window-2; send last answer byte
    status = Pmic_wdgWriteAnswer(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // PMIC has returned to long window after end of sequence; enable power hold
    status = Pmic_wdgSetPwrHold(&pmicHandle, PMIC_ENABLE);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Validate WD_ANSW_ERR error is set
    status = Pmic_wdgGetErrStat(&pmicHandle, &wdgErrStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(wdgErrStat.answErr == (bool)true);

    // Clear the error and validate that it is cleared
    status = Pmic_wdgClrErrStat(&pmicHandle, &wdgErrStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    status = Pmic_wdgGetErrStat(&pmicHandle, &wdgErrStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(wdgErrStat.answErr == (bool)false);
}

void test_positive_wdgQaSequence_detect_seqErr(void)
{
    uint8_t answerCnt = 0U;
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_WdgCfg_t wdgCfg = {
        .validParams = PMIC_WD_CFG_QA_VALID_ALL,
        .rstEn = PMIC_DISABLE,
        .mode = PMIC_QA_MODE,
        .failThr = PMIC_WD_FAIL_THR_MAX,
        .rstThr = PMIC_WD_RST_THR_MAX,
        .longWinDuration = 0xFFU, // ~13 minutes
        .win1Duration = PMIC_WD_WIN1_DURATION_MAX, // 70.4 ms
        .win2Duration = PMIC_WD_WIN2_DURATION_MAX, // 70.4 ms
        .qaFdbk = 0U,
        .qaLfsr = 1U,
        .qaSeed = 2U
    };
    Pmic_WdgErrStat_t wdgErrStat = {.validParams = PMIC_WDG_SEQ_ERR_VALID};

    // Enable Watchdog and clear all watchdog statuses
    status = Pmic_wdgEnable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    status = Pmic_wdgClrErrStatAll(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Configure watchdog
    status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Disable power hold and disable return to long window to begin Q&A sequences
    status = Pmic_wdgSetPwrHold(&pmicHandle, PMIC_DISABLE);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    status = Pmic_wdgSetRetLongWin(&pmicHandle, PMIC_DISABLE);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Exit long window by sending all 4 answer bytes
    for (answerCnt = 4U; answerCnt != 0U; answerCnt--)
    {
        status = Pmic_wdgWriteAnswer(&pmicHandle);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        wdgTest_checkForWdgErrors();
    }

    // Enter Window-1; enable return to long window
    status = Pmic_wdgSetRetLongWin(&pmicHandle, PMIC_ENABLE);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Send only answer bytes Answer-3 and Answer-2 to incur WD_SEQ_ERR
    status = Pmic_wdgWriteAnswer(&pmicHandle); // Answer-3
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    status = Pmic_wdgWriteAnswer(&pmicHandle); // Answer-2
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Wait until Window-1 duration is elapsed to enter Window-2
    platform_timerWaitMs(71U);

    // Enter Window-2; send answer bytes Answer-1 and Answer-0
    status = Pmic_wdgWriteAnswer(&pmicHandle); // Answer-1
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    status = Pmic_wdgWriteAnswer(&pmicHandle); // Answer-0
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // PMIC has returned to long window after end of sequence; enable power hold
    status = Pmic_wdgSetPwrHold(&pmicHandle, PMIC_ENABLE);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Validate WD_SEQ_ERR error is set
    status = Pmic_wdgGetErrStat(&pmicHandle, &wdgErrStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(wdgErrStat.seqErr == (bool)true);

    // Clear the error and validate that it is cleared
    status = Pmic_wdgClrErrStat(&pmicHandle, &wdgErrStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    status = Pmic_wdgGetErrStat(&pmicHandle, &wdgErrStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(wdgErrStat.seqErr == (bool)false);
}

void test_positive_wdgQaSequence_detect_AnswEarlyErr(void)
{
    uint8_t answerCnt = 0U;
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_WdgCfg_t wdgCfg = {
        .validParams = PMIC_WD_CFG_QA_VALID_ALL,
        .rstEn = PMIC_DISABLE,
        .mode = PMIC_QA_MODE,
        .failThr = PMIC_WD_FAIL_THR_MAX,
        .rstThr = PMIC_WD_RST_THR_MAX,
        .longWinDuration = 0xFFU, // ~13 minutes
        .win1Duration = PMIC_WD_WIN1_DURATION_MAX, // 70.4 ms
        .win2Duration = PMIC_WD_WIN2_DURATION_MAX, // 70.4 ms
        .qaFdbk = 0U,
        .qaLfsr = 1U,
        .qaSeed = 2U
    };
    Pmic_WdgErrStat_t wdgErrStat = {.validParams = PMIC_WDG_ANSW_EARLY_ERR_VALID};

    // Enable Watchdog and clear all watchdog statuses
    status = Pmic_wdgEnable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    status = Pmic_wdgClrErrStatAll(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Configure watchdog
    status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Disable power hold and disable return to long window to begin Q&A sequences
    status = Pmic_wdgSetPwrHold(&pmicHandle, PMIC_DISABLE);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    status = Pmic_wdgSetRetLongWin(&pmicHandle, PMIC_DISABLE);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Exit Long Window by sending all 4 answer bytes
    for (answerCnt = 4U; answerCnt != 0U; answerCnt--)
    {
        status = Pmic_wdgWriteAnswer(&pmicHandle);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        wdgTest_checkForWdgErrors();
    }

    // Enter Window-1; enable return to long window
    status = Pmic_wdgSetRetLongWin(&pmicHandle, PMIC_ENABLE);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Send all four answer bytes to incur WD_ANSW_EARLY error
    for (answerCnt = 4U; answerCnt != 0U; answerCnt--)
    {
        status = Pmic_wdgWriteAnswer(&pmicHandle);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    }

    // Wait until Window-1 duration is elapsed to enter Window-2
    platform_timerWaitMs(71U);

    // Enter Window-2; wait until Window-2 duration is elapsed to end sequence
    platform_timerWaitMs(71U);

    // PMIC has returned to long window; enable power hold
    status = Pmic_wdgSetPwrHold(&pmicHandle, PMIC_ENABLE);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Validate WD_ANSW_EARLY error is set
    status = Pmic_wdgGetErrStat(&pmicHandle, &wdgErrStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(wdgErrStat.answEarlyErr == (bool)true);

    // Clear the error and validate that it is cleared
    status = Pmic_wdgClrErrStat(&pmicHandle, &wdgErrStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    status = Pmic_wdgGetErrStat(&pmicHandle, &wdgErrStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(wdgErrStat.answEarlyErr == (bool)false);
}

void test_positive_wdgQaSequence_detect_timeoutErr(void)
{
    uint8_t answerCnt = 0U;
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_WdgCfg_t wdgCfg = {
        .validParams = PMIC_WD_CFG_QA_VALID_ALL,
        .rstEn = PMIC_DISABLE,
        .mode = PMIC_QA_MODE,
        .failThr = PMIC_WD_FAIL_THR_MAX,
        .rstThr = PMIC_WD_RST_THR_MAX,
        .longWinDuration = 0xFFU, // ~13 minutes
        .win1Duration = PMIC_WD_WIN1_DURATION_MAX, // 70.4 ms
        .win2Duration = PMIC_WD_WIN2_DURATION_MAX, // 70.4 ms
        .qaFdbk = 0U,
        .qaLfsr = 1U,
        .qaSeed = 2U
    };
    Pmic_WdgFailCntStat_t wdgFailCntStat = {
        .validParams = (PMIC_BAD_EVENT_VALID | PMIC_FAIL_CNT_VALID),
        .failCnt = 0U
    };
    Pmic_WdgErrStat_t wdgErrStat = {.validParams = PMIC_WDG_TIMEOUT_ERR_VALID};

    // Enable Watchdog and clear all watchdog statuses
    status = Pmic_wdgEnable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    status = Pmic_wdgClrErrStatAll(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Configure watchdog
    status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Disable power hold and disable return to long window to begin Q&A sequences
    status = Pmic_wdgSetPwrHold(&pmicHandle, PMIC_DISABLE);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    status = Pmic_wdgSetRetLongWin(&pmicHandle, PMIC_DISABLE);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Exit Long Window by sending all 4 answer bytes
    for (answerCnt = 4U; answerCnt != 0U; answerCnt--)
    {
        status = Pmic_wdgWriteAnswer(&pmicHandle);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        wdgTest_checkForWdgErrors();
    }

    // Enter Window-1; wait duration of Window-1 to enter Window-2
    platform_timerWaitMs(71U);

    // Validate bad event (no answers sent in Window-1)
    status = Pmic_wdgGetFailCntStat(&pmicHandle, &wdgFailCntStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(wdgFailCntStat.badEvent == (bool)true);

    // Enter Window-2; wait duration of Window-2 to end the sequence
    platform_timerWaitMs(71U);

    // End of sequence and start of new sequence; validate fail count
    status = Pmic_wdgGetFailCntStat(&pmicHandle, &wdgFailCntStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(wdgFailCntStat.failCnt != 0U);

    // Enable return to long window and wait until PMIC returns to long window
    status = Pmic_wdgSetRetLongWin(&pmicHandle, PMIC_ENABLE);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    platform_timerWaitMs(142U);

    // PMIC has entered long window; enable power hold
    status = Pmic_wdgSetPwrHold(&pmicHandle, PMIC_ENABLE);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Validate WD_TIMEOUT error is set
    status = Pmic_wdgGetErrStat(&pmicHandle, &wdgErrStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(wdgErrStat.timeoutErr == (bool)true);

    // Clear the error and validate that it is cleared
    status = Pmic_wdgClrErrStat(&pmicHandle, &wdgErrStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    status = Pmic_wdgGetErrStat(&pmicHandle, &wdgErrStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(wdgErrStat.timeoutErr == (bool)false);
}

void test_positive_wdgQaSequence_detect_longWinTimeoutErr(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_WdgCfg_t wdgCfg = {
        .validParams = PMIC_WD_CFG_QA_VALID_ALL,
        .rstEn = PMIC_DISABLE,
        .mode = PMIC_QA_MODE,
        .failThr = PMIC_WD_FAIL_THR_MAX,
        .rstThr = PMIC_WD_RST_THR_MAX,
        .longWinDuration = 2U, // 252 ms
        .win1Duration = PMIC_WD_WIN1_DURATION_MAX, // 70.4 ms
        .win2Duration = PMIC_WD_WIN2_DURATION_MAX, // 70.4 ms
        .qaFdbk = 0U,
        .qaLfsr = 1U,
        .qaSeed = 2U
    };
    Pmic_WdgErrStat_t wdgErrStat = {.validParams = PMIC_WDG_LONGWIN_TIMEOUT_INT_VALID};

    // Enable Watchdog and clear all watchdog statuses
    status = Pmic_wdgEnable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    status = Pmic_wdgClrErrStatAll(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Configure watchdog
    status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Disable power hold and disable return to long window to begin Q&A sequences
    status = Pmic_wdgSetPwrHold(&pmicHandle, PMIC_DISABLE);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    status = Pmic_wdgSetRetLongWin(&pmicHandle, PMIC_DISABLE);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Wait entire long window duration to incur long window timeout
    platform_timerWaitMs(253U);

    // PMIC has undergone warm reset; unlock PMIC registers
    status = testCommon_unlockPmicRegs(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Enable power hold and enable return to long window
    status = Pmic_wdgSetPwrHold(&pmicHandle, PMIC_ENABLE);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    status = Pmic_wdgSetRetLongWin(&pmicHandle, PMIC_ENABLE);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Validate WD_LONGWIN_TIMEOUT_INT error is set
    status = Pmic_wdgGetErrStat(&pmicHandle, &wdgErrStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(wdgErrStat.longWinTimeoutInt == (bool)true);

    // Clear the error and validate that it is cleared
    status = Pmic_wdgClrErrStat(&pmicHandle, &wdgErrStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    status = Pmic_wdgGetErrStat(&pmicHandle, &wdgErrStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(wdgErrStat.longWinTimeoutInt == (bool)false);

    // Clear all PMIC IRQs (some may have been set as a result of the prior warm reset)
    status = testCommon_clrAllPmicIrq(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_positive_wdgQaSequence_detect_failInt(void)
{
    uint8_t answerCnt = 0U, expFailCnt = 0U;
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_WdgCfg_t wdgCfg = {
        .validParams = PMIC_WD_CFG_QA_VALID_ALL,
        .rstEn = PMIC_DISABLE,
        .mode = PMIC_QA_MODE,
        .failThr = 3U,
        .rstThr = PMIC_WD_RST_THR_MAX,
        .longWinDuration = 0xFFU, // ~13 minutes
        .win1Duration = PMIC_WD_WIN1_DURATION_MAX, // 70.4 ms
        .win2Duration = PMIC_WD_WIN2_DURATION_MAX, // 70.4 ms
        .qaFdbk = 0U,
        .qaLfsr = 1U,
        .qaSeed = 2U
    };
    Pmic_WdgFailCntStat_t wdgFailCntStat = {
        .validParams = PMIC_FAIL_CNT_VALID,
        .failCnt = 0U
    };
    Pmic_WdgErrStat_t wdgErrStat = {.validParams = PMIC_WDG_FAIL_INT_VALID};

    // Enable Watchdog and clear all watchdog statuses
    status = Pmic_wdgEnable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    status = Pmic_wdgClrErrStatAll(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Configure watchdog
    status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Disable power hold and disable return to long window to begin Q&A sequences
    status = Pmic_wdgSetPwrHold(&pmicHandle, PMIC_DISABLE);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    status = Pmic_wdgSetRetLongWin(&pmicHandle, PMIC_DISABLE);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Exit Long Window by sending all 4 answer bytes
    for (answerCnt = 4U; answerCnt != 0U; answerCnt--)
    {
        status = Pmic_wdgWriteAnswer(&pmicHandle);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        wdgTest_checkForWdgErrors();
    }

    // Incur WD_FAIL_INT by causing WD_FAIL_CNT to be greater than WD_FAIL_TH
    for (uint8_t numSequences = (wdgCfg.failThr + 2U); numSequences != 0U; numSequences--)
    {
        expFailCnt++;

        // Enable return to long window upon last iteration
        if (numSequences == 1U)
        {
            status = Pmic_wdgSetRetLongWin(&pmicHandle, PMIC_ENABLE);
            PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        }

        // Enter Window-1; wait duration of Window-1 to enter Window-2
        platform_timerWaitMs(71U);

        // Enter Window-2; wait duration of Window-2 to end the sequence
        platform_timerWaitMs(71U);

        // End of sequence and start of new sequence; validate fail count.
        // NOTE: the fail counter resets to zero upon entering Long Window
        if (numSequences != 1U)
        {
            status = Pmic_wdgGetFailCntStat(&pmicHandle, &wdgFailCntStat);
            PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
            PLATFORM_ASSERT(wdgFailCntStat.failCnt == expFailCnt);
        }
        else
        {
            status = Pmic_wdgGetFailCntStat(&pmicHandle, &wdgFailCntStat);
            PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
            PLATFORM_ASSERT(wdgFailCntStat.failCnt == 0U);
        }
    }

    // PMIC has entered long window; enable power hold
    status = Pmic_wdgSetPwrHold(&pmicHandle, PMIC_ENABLE);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Validate WD_FAIL_INT error is set
    status = Pmic_wdgGetErrStat(&pmicHandle, &wdgErrStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(wdgErrStat.failInt == (bool)true);

    // Clear the error and validate that it is cleared
    status = Pmic_wdgClrErrStat(&pmicHandle, &wdgErrStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    status = Pmic_wdgGetErrStat(&pmicHandle, &wdgErrStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(wdgErrStat.failInt == (bool)false);
}

void test_positive_wdgQaSequence_detect_RstInt(void)
{
    uint8_t answerCnt = 0U, expFailCnt = 0U;
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_WdgCfg_t wdgCfg = {
        .validParams = PMIC_WD_CFG_QA_VALID_ALL,
        .rstEn = PMIC_ENABLE, // Enable warm reset
        .mode = PMIC_QA_MODE,
        .failThr = 3U,
        .rstThr = 3U,
        .longWinDuration = 0xFFU, // ~13 minutes
        .win1Duration = PMIC_WD_WIN1_DURATION_MAX, // 70.4 ms
        .win2Duration = PMIC_WD_WIN2_DURATION_MAX, // 70.4 ms
        .qaFdbk = 0U,
        .qaLfsr = 1U,
        .qaSeed = 2U
    };
    Pmic_WdgFailCntStat_t wdgFailCntStat = {
        .validParams = PMIC_FAIL_CNT_VALID,
        .failCnt = 0U
    };
    Pmic_WdgErrStat_t wdgErrStat = {.validParams = PMIC_WDG_RST_INT_VALID};

    // Enable Watchdog and clear all watchdog statuses
    status = Pmic_wdgEnable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    status = Pmic_wdgClrErrStatAll(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Configure watchdog
    status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Disable power hold and disable return to long window to begin Q&A sequences
    status = Pmic_wdgSetPwrHold(&pmicHandle, PMIC_DISABLE);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    status = Pmic_wdgSetRetLongWin(&pmicHandle, PMIC_DISABLE);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Exit Long Window by sending all 4 answer bytes
    for (answerCnt = 4U; answerCnt != 0U; answerCnt--)
    {
        status = Pmic_wdgWriteAnswer(&pmicHandle);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        wdgTest_checkForWdgErrors();
    }

    // Incur WD_RST_INT by causing WD_FAIL_CNT to be greater than WD_FAIL_TH + WD_RST_INT
    const uint8_t threshold = wdgCfg.failThr + wdgCfg.rstThr + 1U;
    for (uint8_t numSequences = threshold; numSequences != 0U; numSequences--)
    {
        expFailCnt++;

        // Enter Window-1; wait duration of Window-1 to enter Window-2
        platform_timerWaitMs(71U);

        // Enter Window-2; wait duration of Window-2 to end the sequence
        platform_timerWaitMs(71U);

        // End of sequence and start of new sequence; validate fail count.
        // NOTE: PMIC will warm reset at end of last sequence
        if (numSequences != 1U)
        {
            status = Pmic_wdgGetFailCntStat(&pmicHandle, &wdgFailCntStat);
            PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
            PLATFORM_ASSERT(wdgFailCntStat.failCnt == expFailCnt);
        }
    }

    // PMIC has undergone warm reset; unlock PMIC registers
    status = testCommon_unlockPmicRegs(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // PMIC is in long window; enable power hold and enable return to long window
    status = Pmic_wdgSetPwrHold(&pmicHandle, PMIC_ENABLE);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    status = Pmic_wdgSetRetLongWin(&pmicHandle, PMIC_ENABLE);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Validate WD_RST_INT error is set
    status = Pmic_wdgGetErrStat(&pmicHandle, &wdgErrStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(wdgErrStat.rstInt == (bool)true);

    // Clear the error and validate that it is cleared
    status = Pmic_wdgClrErrStat(&pmicHandle, &wdgErrStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    status = Pmic_wdgGetErrStat(&pmicHandle, &wdgErrStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(wdgErrStat.rstInt == (bool)false);

    // Clear all PMIC IRQs (some may have been set as a result of the prior warm reset)
    status = testCommon_clrAllPmicIrq(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

/**
 * @brief Some testing frameworks require an API to setup tests. Rename/rewrite
 * as necessary.
 */
void setUp(void)
{
}

/**
 * @brief Some testing frameworks require an API to teardown tests.
 * Rename/rewrite as necessary.
 */
void tearDown(void)
{
}
