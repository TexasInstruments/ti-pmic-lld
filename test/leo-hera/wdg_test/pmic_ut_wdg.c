/******************************************************************************
 * Copyright (c) 2020 Texas Instruments Incorporated - http://www.ti.com
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
 *  \file   pmic_ut_wdg.c
 *
 *  \brief  PMIC Unit Test for testing PMIC WDG APIs
 *
 */

#include <pmic_ut_wdg.h>

/* Pointer holds the pPmicCoreHandle */
Pmic_CoreHandle_t *pPmicCoreHandle = NULL;

extern uint16_t pmic_device_info;
extern int32_t gCrcTestFlag_J721E;
extern int32_t gCrcTestFlag_J7VCL;

extern Pmic_Ut_FaultInject_t gPmic_faultInjectCfg;
extern int8_t gFailCntStatTestFlag;
extern int8_t gFailCount;
extern int8_t gSkipFailStatusCount;

extern int8_t gErrorStatusTestFlag;
extern int8_t gErrStatusCount;
extern int8_t gSkipErrStatusCount;

extern int8_t gWdgErrStatusTestFlag;

/*!
 * \brief   PMIC WDG Test Cases
 */
static Pmic_Ut_Tests_t pmic_wdg_tests[] =
{
    /*! testID
     *  testDesc
     */
    {
        7327,
        "Pmic_wdgSetCfg : configure all watchdog parameters"
    },
    {
        7328,
        "Pmic_wdgSetCfg : Parameter validation for handle"
    },
    {
        7329,
        "Pmic_wdgSetCfg : Parameter validation for longWindowi_ms min value"
    },
    {
        7330,
        "Pmic_wdgSetCfg : Parameter validation for longWindow_ms max value"
    },
    {
        7331,
        "Pmic_wdgSetCfg : Parameter validation for win1Duration_us min value"
    },
    {
        7332,
        "Pmic_wdgSetCfg : Parameter validation for win1Duration_us max value"
    },
    {
        7333,
        "Pmic_wdgSetCfg : Parameter validation for win2Duration_us min value"
    },
    {
        7334,
        "Pmic_wdgSetCfg : Parameter validation for win2Duration_us max value"
    },
    {
        7335,
        "Pmic_wdgSetCfg : Parameter validation for failThreshold max value"
    },
    {
        7336,
        "Pmic_wdgSetCfg : Parameter validation for rstThreshold max Value"
    },
    {
        7337,
        "Pmic_wdgSetCfg : Parameter validation for qaFdbk max value"
    },
    {
        7338,
        "Pmic_wdgSetCfg : Parameter validation for qaLfsr max value"
    },
    {
        7339,
        "Pmic_wdgSetCfg : Parameter validation for qaQuesSeed max Value"
    },
    {
        7340,
        "Pmic_wdgGetCfg : Get all watchdog parameters"
    },
    {
        7341,
        "Pmic_wdgGetCfg : Parameter validation for handle"
    },
    {
        7342,
        "Pmic_wdgGetCfg : Parameter validation for WdgCfg"
    },
    {
        7343,
        "Pmic_wdgEnable : Parameter validation for handle"
    },
    {
        7344,
        "Pmic_wdgDisable : Parameter validation for WdgCfg"
    },
    {
        7345,
        "Pmic_wdgStartQaSequence : Parameter validation for handle"
    },
    {
        7346,
        "Pmic_wdgStartQaSequence : Test wdg QA sequences"
    },
    {
        7347,
        "Pmic_wdgStartQaSequence : Test wdg QA sequences with different QA feedback values"
    },
    {
        7348,
        "Pmic_wdgStartQaSequence : Test wdg QA sequences with different QA LFSR values"
    },
    {
        7349,
        "Pmic_wdgStartQaSequence : Test wdg QA sequences with different QA Question Seed values"
    },
    {
        7350,
        "Pmic_wdgGetFailCntStat : Test get wdg failcount status"
    },
    {
        7351,
        "Pmic_wdgGetFailCntStat : Parameter validation for handle"
    },
    {
        7352,
        "Pmic_wdgGetFailCntStat : Parameter validation for failCount"
    },
    {
        7353,
        "Pmic_wdgGetErrorStatus: Get all watchdog error status"
    },
    {
        7354,
        "Pmic_wdgGetErrorStatus : Parameter validation for handle"
    },
    {
        7355,
        "Pmic_wdgGetErrorStatus : Parameter validation for errStatus"
    },
    {
        0xAB17,
        "Pmic_wdgStartTriggerSequence : Test wdg trigger sequence"
    },
    {
        7357,
        "Pmic_wdgStartTriggerSequence : Parameter validation for handle"
    },
    {
        7958,
        "Pmic_wdgStartQaSequence : Parameter validation for maxCnt"
    },
    {
        10075,
        "Pmic_wdgClrErrStatus: Test to clear all watchdog error status"
    },
    {
        10076,
        "Pmic_wdgClrErrStatus : Parameter validation for handle"
    },
    {
        10077,
        "Pmic_wdgClrErrStatus : Parameter validation for wdgErrType"
    },
    {
        10071,
        "Pmic_wdgQaSequenceWriteAnswer : Test wdg QA sequences"
    },
    {
        10072,
        "Pmic_wdgQaSequenceWriteAnswer : Test wdg QA sequences with different QA feedback values"
    },
    {
        10073,
        "Pmic_wdgQaSequenceWriteAnswer : Test wdg QA sequences with different QA lfsr values"
    },
    {
        10074,
        "Pmic_wdgQaSequenceWriteAnswer : Test wdg QA sequences with different QA seed values"
    },
    {
        10780,
        "Pmic_wdgSetCfg : Test to set longWinDuration as 100ms"
    },
    {
        10781,
        "Pmic_wdgSetCfg : Parameter validation for longWinDuration min Value as 2999 ms"
    },
    {
        10782,
        "Pmic_wdgStartTriggerSequence - Code coverage test for wdg Trigger sequence"
    },
    {
        10783,
        "Pmic_wdgClrErrStatus : Test to clear wdg error based on WDG Error type - TimeOut, SeqErr, AnsEarly, FailInt"
    },
    {
        10784,
        "Pmic_wdgQaSequenceWriteAnswer : Manual Test case to test WDG RST INT and WDG LONGWIN TIMEOUT INT"
    },
    {
        10785,
        "Pmic_wdgStartQaSequence : Test wdg QA sequences for Early Answer Error"
    },
    {
        8917,
        "Pmic_wdgTests : Added for Coverage"
    },
};

/*!
 * \brief   Test to configure a wdg for all params
 */
static void test_pmic_wdg_setCfg_forallparams(void)
{
    int32_t pmicStatus      = PMIC_ST_SUCCESS;
    Pmic_WdgCfg_t wdgCfg_rd = {PMIC_WDG_CFG_SETPARAMS_FORALL, };
    Pmic_WdgCfg_t wdgCfg    =
    {
        PMIC_WDG_CFG_SETPARAMS_FORALL,
        750000U,
        4950U,
        4950U,
        PMIC_WDG_FAIL_THRESHOLD_COUNT_5,
        PMIC_WDG_RESET_THRESHOLD_COUNT_6,
        PMIC_WDG_QA_MODE,
        PMIC_WDG_PWRHOLD_ENABLE,
        PMIC_WDG_RESET_DISABLE,
        PMIC_WDG_RETLONGWIN_ENABLE,
        PMIC_WDG_QA_FEEDBACK_VALUE_2,
        PMIC_WDG_QA_LFSR_VALUE_1,
        PMIC_WDG_QA_QUES_SEED_VALUE_5,
    };

    test_pmic_print_unity_testcase_info(7327,
                                        pmic_wdg_tests,
                                        PMIC_WDG_NUM_OF_TESTCASES);

    if((PMIC_INTF_SINGLE_I2C == pPmicCoreHandle->commMode) ||
       (PMIC_INTF_SPI == pPmicCoreHandle->commMode))
    {
        pmic_testResultUpdate_ignore(7327,
                                     pmic_wdg_tests,
                                     PMIC_WDG_NUM_OF_TESTCASES);
    }

    if(PMIC_SILICON_REV_ID_PG_2_0 == pPmicCoreHandle->pmicDevSiliconRev)
    {
        wdgCfg.longWinDuration_ms = 752000U;
    }

    /* Enable WDG Timer */
    pmicStatus = Pmic_wdgEnable(pPmicCoreHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_wdgSetCfg(pPmicCoreHandle, wdgCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_wdgGetCfg(pPmicCoreHandle, &wdgCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    TEST_ASSERT_EQUAL(wdgCfg.longWinDuration_ms, wdgCfg_rd.longWinDuration_ms);
    TEST_ASSERT_EQUAL(wdgCfg.win1Duration_us, wdgCfg_rd.win1Duration_us);
    TEST_ASSERT_EQUAL(wdgCfg.win2Duration_us, wdgCfg_rd.win2Duration_us);
    TEST_ASSERT_EQUAL(wdgCfg.failThreshold, wdgCfg_rd.failThreshold);
    TEST_ASSERT_EQUAL(wdgCfg.rstThreshold, wdgCfg_rd.rstThreshold);
    TEST_ASSERT_EQUAL(wdgCfg.wdgMode, wdgCfg_rd.wdgMode);
    TEST_ASSERT_EQUAL(wdgCfg.pwrHold, wdgCfg_rd.pwrHold);
    TEST_ASSERT_EQUAL(wdgCfg.rstEnable, wdgCfg_rd.rstEnable);
    TEST_ASSERT_EQUAL(wdgCfg.retLongWin, wdgCfg_rd.retLongWin);
    TEST_ASSERT_EQUAL(wdgCfg.qaFdbk, wdgCfg_rd.qaFdbk);
    TEST_ASSERT_EQUAL(wdgCfg.qaLfsr, wdgCfg_rd.qaLfsr);
    TEST_ASSERT_EQUAL(wdgCfg.qaQuesSeed, wdgCfg_rd.qaQuesSeed);

    /* Disable WDG Timer */
    pmicStatus = Pmic_wdgDisable(pPmicCoreHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    /* Delay is needed to sattle down watchdog to longwindow */
    Osal_delay(20U);

    if((PMIC_SILICON_REV_ID_PG_2_0 == pPmicCoreHandle->pmicDevSiliconRev) &&
       (PMIC_INTF_DUAL_I2C == pPmicCoreHandle->commMode))
    {
        wdgCfg.validParams = PMIC_CFG_WDG_LONGWINDURATION_VALID_SHIFT;
        wdgCfg_rd.validParams = PMIC_CFG_WDG_LONGWINDURATION_VALID_SHIFT;

        wdgCfg.longWinDuration_ms = 80U;

        /* Enable WDG Timer */
        pmicStatus = Pmic_wdgEnable(pPmicCoreHandle);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        pmicStatus = Pmic_wdgSetCfg(pPmicCoreHandle, wdgCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        pmicStatus = Pmic_wdgGetCfg(pPmicCoreHandle, &wdgCfg_rd);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
        TEST_ASSERT_EQUAL(wdgCfg.longWinDuration_ms, wdgCfg_rd.longWinDuration_ms);

        wdgCfg.longWinDuration_ms = 250U;

        pmicStatus = Pmic_wdgSetCfg(pPmicCoreHandle, wdgCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        pmicStatus = Pmic_wdgGetCfg(pPmicCoreHandle, &wdgCfg_rd);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
        TEST_ASSERT_EQUAL(wdgCfg.longWinDuration_ms, wdgCfg_rd.longWinDuration_ms);

        /* Disable WDG Timer */
        pmicStatus = Pmic_wdgDisable(pPmicCoreHandle);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        /* Delay is needed to sattle down watchdog to longwindow */
        Osal_delay(20U);
    }

    /* Enable WDG Timer */
    pmicStatus = Pmic_wdgEnable(pPmicCoreHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    /* Set Reset Enable */
    wdgCfg.validParams = PMIC_CFG_WDG_RSTENABLE_VALID_SHIFT;
    wdgCfg_rd.validParams = PMIC_CFG_WDG_RSTENABLE_VALID_SHIFT;
    wdgCfg.rstEnable = PMIC_WDG_RESET_ENABLE;
    pmicStatus = Pmic_wdgSetCfg(pPmicCoreHandle, wdgCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_wdgGetCfg(pPmicCoreHandle, &wdgCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    TEST_ASSERT_EQUAL(wdgCfg.rstEnable, wdgCfg_rd.rstEnable);

    /* Disable WDG Timer */
    pmicStatus = Pmic_wdgDisable(pPmicCoreHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    /* Enable WDG Timer */
    pmicStatus = Pmic_wdgEnable(pPmicCoreHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    /* Set Reset Disable */
    wdgCfg.validParams = PMIC_CFG_WDG_RSTENABLE_VALID_SHIFT;
    wdgCfg_rd.validParams = PMIC_CFG_WDG_RSTENABLE_VALID_SHIFT;
    wdgCfg.rstEnable = PMIC_WDG_RESET_DISABLE;
    pmicStatus = Pmic_wdgSetCfg(pPmicCoreHandle, wdgCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_wdgGetCfg(pPmicCoreHandle, &wdgCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    TEST_ASSERT_EQUAL(wdgCfg.rstEnable, wdgCfg_rd.rstEnable);

    /* Set WDG Mode */
    wdgCfg.validParams = PMIC_CFG_WDG_WDGMODE_VALID_SHIFT;
    wdgCfg_rd.validParams = PMIC_CFG_WDG_WDGMODE_VALID_SHIFT;
    wdgCfg.wdgMode = PMIC_WDG_TRIGGER_MODE;
    pmicStatus = Pmic_wdgSetCfg(pPmicCoreHandle, wdgCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_wdgGetCfg(pPmicCoreHandle, &wdgCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    TEST_ASSERT_EQUAL(wdgCfg.wdgMode, wdgCfg_rd.wdgMode);

    /* Set Power Hold Enable */
    wdgCfg.validParams = PMIC_CFG_WDG_PWRHOLD_VALID_SHIFT;
    wdgCfg_rd.validParams = PMIC_CFG_WDG_PWRHOLD_VALID_SHIFT;
    wdgCfg.pwrHold = PMIC_WDG_PWRHOLD_ENABLE;
    pmicStatus = Pmic_wdgSetCfg(pPmicCoreHandle, wdgCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_wdgGetCfg(pPmicCoreHandle, &wdgCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    TEST_ASSERT_EQUAL(wdgCfg.pwrHold, wdgCfg_rd.pwrHold);

    /* Set Return to Long Window */
    wdgCfg.validParams = PMIC_CFG_WDG_RETLONGWIN_VALID_SHIFT;
    wdgCfg_rd.validParams = PMIC_CFG_WDG_RETLONGWIN_VALID_SHIFT;
    wdgCfg.retLongWin = PMIC_WDG_RETLONGWIN_DISABLE;
    pmicStatus = Pmic_wdgSetCfg(pPmicCoreHandle, wdgCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_wdgGetCfg(pPmicCoreHandle, &wdgCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    TEST_ASSERT_EQUAL(wdgCfg.retLongWin, wdgCfg_rd.retLongWin);

    /* Disable WDG Timer */
    pmicStatus = Pmic_wdgDisable(pPmicCoreHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    /* Delay is needed to sattle down watchdog to longwindow */
    Osal_delay(20U);

    pmic_testResultUpdate_pass(7327,
                               pmic_wdg_tests,
                               PMIC_WDG_NUM_OF_TESTCASES);
}

/*!
 * \brief   Parameter validation for handle
 */
static void test_pmic_wdg_setCfg_prmValTest_handle(void)
{
    int32_t pmicStatus      = PMIC_ST_SUCCESS;
    Pmic_WdgCfg_t wdgCfg    =
    {
        PMIC_WDG_CFG_SETPARAMS_FORALL,
        750000U,
        4950U,
        4950U,
        PMIC_WDG_FAIL_THRESHOLD_COUNT_5,
        PMIC_WDG_RESET_THRESHOLD_COUNT_6,
        PMIC_WDG_QA_MODE,
        PMIC_WDG_PWRHOLD_DISABLE,
        PMIC_WDG_RESET_DISABLE,
        PMIC_WDG_RETLONGWIN_ENABLE,
        PMIC_WDG_QA_FEEDBACK_VALUE_2,
        PMIC_WDG_QA_LFSR_VALUE_1,
        PMIC_WDG_QA_QUES_SEED_VALUE_5,
    };

    test_pmic_print_unity_testcase_info(7328,
                                        pmic_wdg_tests,
                                        PMIC_WDG_NUM_OF_TESTCASES);

    if(PMIC_SILICON_REV_ID_PG_2_0 == pPmicCoreHandle->pmicDevSiliconRev)
    {
        wdgCfg.longWinDuration_ms = 752000U;
    }

    pmicStatus = Pmic_wdgSetCfg(NULL, wdgCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, pmicStatus);

    pmic_testResultUpdate_pass(7328,
                               pmic_wdg_tests,
                               PMIC_WDG_NUM_OF_TESTCASES);
}

/*!
 * \brief   Parameter validation for longWinDuration_ms min Value as 98ms for
 *          Leo PMIC PG1.0 and 78ms for Leo PMIC PG2.0
 */
static void test_pmic_wdg_setCfg_prmValTest_longwinMin(void)
{
    int32_t pmicStatus      = PMIC_ST_SUCCESS;
    Pmic_WdgCfg_t wdgCfg    =
    {
        PMIC_CFG_WDG_LONGWINDURATION_VALID_SHIFT,
        98U,
        4950U,
        4950U,
        PMIC_WDG_FAIL_THRESHOLD_COUNT_5,
        PMIC_WDG_RESET_THRESHOLD_COUNT_6,
        PMIC_WDG_QA_MODE,
        PMIC_WDG_PWRHOLD_DISABLE,
        PMIC_WDG_RESET_DISABLE,
        PMIC_WDG_RETLONGWIN_ENABLE,
        PMIC_WDG_QA_FEEDBACK_VALUE_2,
        PMIC_WDG_QA_LFSR_VALUE_1,
        PMIC_WDG_QA_QUES_SEED_VALUE_5,
    };

    test_pmic_print_unity_testcase_info(7329,
                                        pmic_wdg_tests,
                                        PMIC_WDG_NUM_OF_TESTCASES);

    if(PMIC_SILICON_REV_ID_PG_2_0 == pPmicCoreHandle->pmicDevSiliconRev)
    {
        wdgCfg.longWinDuration_ms = 78U;
    }

    pmicStatus = Pmic_wdgSetCfg(pPmicCoreHandle, wdgCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_WDG_WINDOW, pmicStatus);

    pmic_testResultUpdate_pass(7329,
                               pmic_wdg_tests,
                               PMIC_WDG_NUM_OF_TESTCASES);
}

/*!
 * \brief   Parameter validation for longWinDuration_ms max Value as 768000 ms
 *          for Leo PMIC PG1.0 and 775000 ms for Leo PMIC PG2.0
 */
static void test_pmic_wdg_setCfg_prmValTest_longwinMax(void)
{
    int32_t pmicStatus      = PMIC_ST_SUCCESS;
    Pmic_WdgCfg_t wdgCfg    =
    {
        PMIC_CFG_WDG_LONGWINDURATION_VALID_SHIFT,
        768000U,
        4950U,
        4950U,
        PMIC_WDG_FAIL_THRESHOLD_COUNT_5,
        PMIC_WDG_RESET_THRESHOLD_COUNT_6,
        PMIC_WDG_QA_MODE,
        PMIC_WDG_PWRHOLD_DISABLE,
        PMIC_WDG_RESET_DISABLE,
        PMIC_WDG_RETLONGWIN_ENABLE,
        PMIC_WDG_QA_FEEDBACK_VALUE_2,
        PMIC_WDG_QA_LFSR_VALUE_1,
        PMIC_WDG_QA_QUES_SEED_VALUE_5,
    };

    test_pmic_print_unity_testcase_info(7330,
                                        pmic_wdg_tests,
                                        PMIC_WDG_NUM_OF_TESTCASES);

    if(PMIC_SILICON_REV_ID_PG_2_0 == pPmicCoreHandle->pmicDevSiliconRev)
    {
        wdgCfg.longWinDuration_ms = 775000U;
    }

    pmicStatus = Pmic_wdgSetCfg(pPmicCoreHandle, wdgCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_WDG_WINDOW, pmicStatus);

    pmic_testResultUpdate_pass(7330,
                               pmic_wdg_tests,
                               PMIC_WDG_NUM_OF_TESTCASES);
}

/*!
 * \brief   Parameter validation for win1Duration_us min Value
 */
static void test_pmic_wdg_setCfg_prmValTest_win1Min(void)
{
    int32_t pmicStatus      = PMIC_ST_SUCCESS;
    Pmic_WdgCfg_t wdgCfg    =
    {
        PMIC_CFG_WDG_WIN1DURATION_VALID_SHIFT,
        750000U,
        500U,
        4950U,
        PMIC_WDG_FAIL_THRESHOLD_COUNT_5,
        PMIC_WDG_RESET_THRESHOLD_COUNT_6,
        PMIC_WDG_QA_MODE,
        PMIC_WDG_PWRHOLD_DISABLE,
        PMIC_WDG_RESET_DISABLE,
        PMIC_WDG_RETLONGWIN_ENABLE,
        PMIC_WDG_QA_FEEDBACK_VALUE_2,
        PMIC_WDG_QA_LFSR_VALUE_1,
        PMIC_WDG_QA_QUES_SEED_VALUE_5,
    };

    test_pmic_print_unity_testcase_info(7331,
                                        pmic_wdg_tests,
                                        PMIC_WDG_NUM_OF_TESTCASES);

    if(PMIC_SILICON_REV_ID_PG_2_0 == pPmicCoreHandle->pmicDevSiliconRev)
    {
        wdgCfg.longWinDuration_ms = 752000U;
    }

    pmicStatus = Pmic_wdgSetCfg(pPmicCoreHandle, wdgCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_WDG_WINDOW, pmicStatus);

    pmic_testResultUpdate_pass(7331,
                               pmic_wdg_tests,
                               PMIC_WDG_NUM_OF_TESTCASES);
}

/*!
 * \brief   Parameter validation for win1Duration_us max Value
 */
static void test_pmic_wdg_setCfg_prmValTest_win1Max(void)
{
    int32_t pmicStatus      = PMIC_ST_SUCCESS;
    Pmic_WdgCfg_t wdgCfg    =
    {
        PMIC_CFG_WDG_WIN1DURATION_VALID_SHIFT,
        750000U,
        70500U,
        4950U,
        PMIC_WDG_FAIL_THRESHOLD_COUNT_5,
        PMIC_WDG_RESET_THRESHOLD_COUNT_6,
        PMIC_WDG_QA_MODE,
        PMIC_WDG_PWRHOLD_DISABLE,
        PMIC_WDG_RESET_DISABLE,
        PMIC_WDG_RETLONGWIN_ENABLE,
        PMIC_WDG_QA_FEEDBACK_VALUE_2,
        PMIC_WDG_QA_LFSR_VALUE_1,
        PMIC_WDG_QA_QUES_SEED_VALUE_5,
    };

    test_pmic_print_unity_testcase_info(7332,
                                        pmic_wdg_tests,
                                        PMIC_WDG_NUM_OF_TESTCASES);

    if(PMIC_SILICON_REV_ID_PG_2_0 == pPmicCoreHandle->pmicDevSiliconRev)
    {
        wdgCfg.longWinDuration_ms = 752000U;
    }

    pmicStatus = Pmic_wdgSetCfg(pPmicCoreHandle, wdgCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_WDG_WINDOW, pmicStatus);

    pmic_testResultUpdate_pass(7332,
                               pmic_wdg_tests,
                               PMIC_WDG_NUM_OF_TESTCASES);
}

/*!
 * \brief   Parameter validation for win2Duration_us min Value
 */
static void test_pmic_wdg_setCfg_prmValTest_win2Min(void)
{
    int32_t pmicStatus      = PMIC_ST_SUCCESS;
    Pmic_WdgCfg_t wdgCfg    =
    {
        PMIC_CFG_WDG_WIN2DURATION_VALID_SHIFT,
        750000U,
        4950U,
        500U,
        PMIC_WDG_FAIL_THRESHOLD_COUNT_5,
        PMIC_WDG_RESET_THRESHOLD_COUNT_6,
        PMIC_WDG_QA_MODE,
        PMIC_WDG_PWRHOLD_DISABLE,
        PMIC_WDG_RESET_DISABLE,
        PMIC_WDG_RETLONGWIN_ENABLE,
        PMIC_WDG_QA_FEEDBACK_VALUE_2,
        PMIC_WDG_QA_LFSR_VALUE_1,
        PMIC_WDG_QA_QUES_SEED_VALUE_5,
    };

    test_pmic_print_unity_testcase_info(7333,
                                        pmic_wdg_tests,
                                        PMIC_WDG_NUM_OF_TESTCASES);

    if(PMIC_SILICON_REV_ID_PG_2_0 == pPmicCoreHandle->pmicDevSiliconRev)
    {
        wdgCfg.longWinDuration_ms = 752000U;
    }

    pmicStatus = Pmic_wdgSetCfg(pPmicCoreHandle, wdgCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_WDG_WINDOW, pmicStatus);

    pmic_testResultUpdate_pass(7333,
                               pmic_wdg_tests,
                               PMIC_WDG_NUM_OF_TESTCASES);
}

/*!
 * \brief   Parameter validation for win2Duration_us max Value
 */
static void test_pmic_wdg_setCfg_prmValTest_win2Max(void)
{
    int32_t pmicStatus      = PMIC_ST_SUCCESS;
    Pmic_WdgCfg_t wdgCfg    =
    {
        PMIC_CFG_WDG_WIN2DURATION_VALID_SHIFT,
        750000U,
        4950U,
        70500U,
        PMIC_WDG_FAIL_THRESHOLD_COUNT_5,
        PMIC_WDG_RESET_THRESHOLD_COUNT_6,
        PMIC_WDG_QA_MODE,
        PMIC_WDG_PWRHOLD_DISABLE,
        PMIC_WDG_RESET_DISABLE,
        PMIC_WDG_RETLONGWIN_ENABLE,
        PMIC_WDG_QA_FEEDBACK_VALUE_2,
        PMIC_WDG_QA_LFSR_VALUE_1,
        PMIC_WDG_QA_QUES_SEED_VALUE_5,
    };

    test_pmic_print_unity_testcase_info(7334,
                                        pmic_wdg_tests,
                                        PMIC_WDG_NUM_OF_TESTCASES);

    if(PMIC_SILICON_REV_ID_PG_2_0 == pPmicCoreHandle->pmicDevSiliconRev)
    {
        wdgCfg.longWinDuration_ms = 752000U;
    }

    pmicStatus = Pmic_wdgSetCfg(pPmicCoreHandle, wdgCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_WDG_WINDOW, pmicStatus);

    pmic_testResultUpdate_pass(7334,
                               pmic_wdg_tests,
                               PMIC_WDG_NUM_OF_TESTCASES);
}

/*!
 * \brief   Parameter validation for failThreshold max Value
 */
static void test_pmic_wdg_setCfg_prmValTest_failThresholdMax(void)
{
    int32_t pmicStatus      = PMIC_ST_SUCCESS;
    Pmic_WdgCfg_t wdgCfg    =
    {
        PMIC_CFG_WDG_FAILTHRESHOLD_VALID_SHIFT,
        750000U,
        4950U,
        500U,
        8U,
        PMIC_WDG_RESET_THRESHOLD_COUNT_6,
        PMIC_WDG_QA_MODE,
        PMIC_WDG_PWRHOLD_DISABLE,
        PMIC_WDG_RESET_DISABLE,
        PMIC_WDG_RETLONGWIN_ENABLE,
        PMIC_WDG_QA_FEEDBACK_VALUE_2,
        PMIC_WDG_QA_LFSR_VALUE_1,
        PMIC_WDG_QA_QUES_SEED_VALUE_5,
    };

    test_pmic_print_unity_testcase_info(7335,
                                        pmic_wdg_tests,
                                        PMIC_WDG_NUM_OF_TESTCASES);

    if(PMIC_SILICON_REV_ID_PG_2_0 == pPmicCoreHandle->pmicDevSiliconRev)
    {
        wdgCfg.longWinDuration_ms = 752000U;
    }

    pmicStatus = Pmic_wdgSetCfg(pPmicCoreHandle, wdgCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(7335,
                               pmic_wdg_tests,
                               PMIC_WDG_NUM_OF_TESTCASES);
}

/*!
 * \brief   Parameter validation for rstThreshold max Value
 */
static void test_pmic_wdg_setCfg_prmValTest_rstThresholdMax(void)
{
    int32_t pmicStatus      = PMIC_ST_SUCCESS;
    Pmic_WdgCfg_t wdgCfg    =
    {
        PMIC_CFG_WDG_RSTTHRESHOLD_VALID_SHIFT,
        750000U,
        4950U,
        70500U,
        PMIC_WDG_FAIL_THRESHOLD_COUNT_5,
        8U,
        PMIC_WDG_QA_MODE,
        PMIC_WDG_PWRHOLD_DISABLE,
        PMIC_WDG_RESET_DISABLE,
        PMIC_WDG_RETLONGWIN_ENABLE,
        PMIC_WDG_QA_FEEDBACK_VALUE_2,
        PMIC_WDG_QA_LFSR_VALUE_1,
        PMIC_WDG_QA_QUES_SEED_VALUE_5,
    };

    test_pmic_print_unity_testcase_info(7336,
                                        pmic_wdg_tests,
                                        PMIC_WDG_NUM_OF_TESTCASES);

    if(PMIC_SILICON_REV_ID_PG_2_0 == pPmicCoreHandle->pmicDevSiliconRev)
    {
        wdgCfg.longWinDuration_ms = 752000U;
    }

    pmicStatus = Pmic_wdgSetCfg(pPmicCoreHandle, wdgCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(7336,
                               pmic_wdg_tests,
                               PMIC_WDG_NUM_OF_TESTCASES);
}

/*!
 * \brief   Parameter validation for qaFdbk max Value
 */
static void test_pmic_wdg_setCfg_prmValTest_qaFdbkMax(void)
{
    int32_t pmicStatus      = PMIC_ST_SUCCESS;
    Pmic_WdgCfg_t wdgCfg    =
    {
        PMIC_CFG_WDG_QA_FDBK_VALID_SHIFT,
        750000U,
        4950U,
        70500U,
        PMIC_WDG_FAIL_THRESHOLD_COUNT_5,
        PMIC_WDG_RESET_THRESHOLD_COUNT_6,
        PMIC_WDG_QA_MODE,
        PMIC_WDG_PWRHOLD_DISABLE,
        PMIC_WDG_RESET_DISABLE,
        PMIC_WDG_RETLONGWIN_ENABLE,
        4U,
        PMIC_WDG_QA_LFSR_VALUE_1,
        PMIC_WDG_QA_QUES_SEED_VALUE_5,
    };

    test_pmic_print_unity_testcase_info(7337,
                                        pmic_wdg_tests,
                                        PMIC_WDG_NUM_OF_TESTCASES);

    if(PMIC_SILICON_REV_ID_PG_2_0 == pPmicCoreHandle->pmicDevSiliconRev)
    {
        wdgCfg.longWinDuration_ms = 752000U;
    }

    pmicStatus = Pmic_wdgSetCfg(pPmicCoreHandle, wdgCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(7337,
                               pmic_wdg_tests,
                               PMIC_WDG_NUM_OF_TESTCASES);
}

/*!
 * \brief   Parameter validation for qaLfsr max Value
 */
static void test_pmic_wdg_setCfg_prmValTest_qaLfsrMax(void)
{
    int32_t pmicStatus      = PMIC_ST_SUCCESS;
    Pmic_WdgCfg_t wdgCfg    =
    {
        PMIC_CFG_WDG_QA_LFSR_VALID_SHIFT,
        750000U,
        4950U,
        500U,
        8U,
        PMIC_WDG_RESET_THRESHOLD_COUNT_6,
        PMIC_WDG_QA_MODE,
        PMIC_WDG_PWRHOLD_DISABLE,
        PMIC_WDG_RESET_DISABLE,
        PMIC_WDG_RETLONGWIN_ENABLE,
        PMIC_WDG_QA_FEEDBACK_VALUE_2,
        4U,
        PMIC_WDG_QA_QUES_SEED_VALUE_5,
    };

    test_pmic_print_unity_testcase_info(7338,
                                        pmic_wdg_tests,
                                        PMIC_WDG_NUM_OF_TESTCASES);

    if(PMIC_SILICON_REV_ID_PG_2_0 == pPmicCoreHandle->pmicDevSiliconRev)
    {
        wdgCfg.longWinDuration_ms = 752000U;
    }

    pmicStatus = Pmic_wdgSetCfg(pPmicCoreHandle, wdgCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(7338,
                               pmic_wdg_tests,
                               PMIC_WDG_NUM_OF_TESTCASES);
}

/*!
 * \brief   Parameter validation for qaQuesSeed max Value
 */
static void test_pmic_wdg_setCfg_prmValTest_qaQuesSeedMax(void)
{
    int32_t pmicStatus      = PMIC_ST_SUCCESS;
    Pmic_WdgCfg_t wdgCfg    =
    {
        PMIC_CFG_WDG_QA_QUES_SEED_VALID_SHIFT,
        750000U,
        4950U,
        70500U,
        PMIC_WDG_FAIL_THRESHOLD_COUNT_5,
        8U,
        PMIC_WDG_QA_MODE,
        PMIC_WDG_PWRHOLD_DISABLE,
        PMIC_WDG_RESET_DISABLE,
        PMIC_WDG_RETLONGWIN_ENABLE,
        PMIC_WDG_QA_FEEDBACK_VALUE_2,
        PMIC_WDG_QA_LFSR_VALUE_1,
        16,
    };

    test_pmic_print_unity_testcase_info(7339,
                                        pmic_wdg_tests,
                                        PMIC_WDG_NUM_OF_TESTCASES);

    if(PMIC_SILICON_REV_ID_PG_2_0 == pPmicCoreHandle->pmicDevSiliconRev)
    {
        wdgCfg.longWinDuration_ms = 752000U;
    }

    pmicStatus = Pmic_wdgSetCfg(pPmicCoreHandle, wdgCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(7339,
                               pmic_wdg_tests,
                               PMIC_WDG_NUM_OF_TESTCASES);
}

/*!
 * \brief   Test to get configuration of a wdg for all params
 */
static void test_pmic_wdg_getCfg_forallparams(void)
{
    int32_t pmicStatus      = PMIC_ST_SUCCESS;
    Pmic_WdgCfg_t wdgCfg_rd = {PMIC_WDG_CFG_SETPARAMS_FORALL, };
    Pmic_WdgCfg_t wdgCfg    =
    {
        PMIC_WDG_CFG_SETPARAMS_FORALL,
        750000U,
        4950U,
        4950U,
        PMIC_WDG_FAIL_THRESHOLD_COUNT_5,
        PMIC_WDG_RESET_THRESHOLD_COUNT_6,
        PMIC_WDG_QA_MODE,
        PMIC_WDG_PWRHOLD_DISABLE,
        PMIC_WDG_RESET_DISABLE,
        PMIC_WDG_RETLONGWIN_ENABLE,
        PMIC_WDG_QA_FEEDBACK_VALUE_2,
        PMIC_WDG_QA_LFSR_VALUE_1,
        PMIC_WDG_QA_QUES_SEED_VALUE_5,
    };

    test_pmic_print_unity_testcase_info(7340,
                                        pmic_wdg_tests,
                                        PMIC_WDG_NUM_OF_TESTCASES);

    if((PMIC_INTF_SINGLE_I2C == pPmicCoreHandle->commMode) ||
       (PMIC_INTF_SPI == pPmicCoreHandle->commMode))
    {
        pmic_testResultUpdate_ignore(7340,
                                     pmic_wdg_tests,
                                     PMIC_WDG_NUM_OF_TESTCASES);
    }

    if(PMIC_SILICON_REV_ID_PG_2_0 == pPmicCoreHandle->pmicDevSiliconRev)
    {
        wdgCfg.longWinDuration_ms = 752000U;
    }

    /* Enable WDG Timer */
    pmicStatus = Pmic_wdgEnable(pPmicCoreHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_wdgSetCfg(pPmicCoreHandle, wdgCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_wdgGetCfg(pPmicCoreHandle, &wdgCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(wdgCfg.longWinDuration_ms, wdgCfg_rd.longWinDuration_ms);
    TEST_ASSERT_EQUAL(wdgCfg.win1Duration_us, wdgCfg_rd.win1Duration_us);
    TEST_ASSERT_EQUAL(wdgCfg.win2Duration_us, wdgCfg_rd.win2Duration_us);
    TEST_ASSERT_EQUAL(wdgCfg.failThreshold, wdgCfg_rd.failThreshold);
    TEST_ASSERT_EQUAL(wdgCfg.rstThreshold, wdgCfg_rd.rstThreshold);
    TEST_ASSERT_EQUAL(wdgCfg.wdgMode, wdgCfg_rd.wdgMode);
    TEST_ASSERT_EQUAL(wdgCfg.pwrHold, wdgCfg_rd.pwrHold);
    TEST_ASSERT_EQUAL(wdgCfg.rstEnable, wdgCfg_rd.rstEnable);
    TEST_ASSERT_EQUAL(wdgCfg.retLongWin, wdgCfg_rd.retLongWin);
    TEST_ASSERT_EQUAL(wdgCfg.qaFdbk, wdgCfg_rd.qaFdbk);
    TEST_ASSERT_EQUAL(wdgCfg.qaLfsr, wdgCfg_rd.qaLfsr);
    TEST_ASSERT_EQUAL(wdgCfg.qaQuesSeed, wdgCfg_rd.qaQuesSeed);

    /* Disable WDG Timer */
    pmicStatus = Pmic_wdgDisable(pPmicCoreHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmic_testResultUpdate_pass(7340,
                               pmic_wdg_tests,
                               PMIC_WDG_NUM_OF_TESTCASES);
}

/*!
 * \brief   Parameter validation for handle
 */
static void test_pmic_wdg_getCfg_prmValTest_handle(void)
{
    int32_t pmicStatus      = PMIC_ST_SUCCESS;
    Pmic_WdgCfg_t wdgCfg = {PMIC_WDG_CFG_SETPARAMS_FORALL, };

    test_pmic_print_unity_testcase_info(7341,
                                        pmic_wdg_tests,
                                        PMIC_WDG_NUM_OF_TESTCASES);

    pmicStatus = Pmic_wdgGetCfg(NULL, &wdgCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, pmicStatus);

    pmic_testResultUpdate_pass(7341,
                               pmic_wdg_tests,
                               PMIC_WDG_NUM_OF_TESTCASES);
}

/*!
 * \brief   Parameter validation for wdgCfg
 */
static void test_pmic_wdg_getCfg_prmValTest_wdgcfgParam(void)
{
    int32_t pmicStatus      = PMIC_ST_SUCCESS;

    test_pmic_print_unity_testcase_info(7342,
                                        pmic_wdg_tests,
                                        PMIC_WDG_NUM_OF_TESTCASES);

    pmicStatus = Pmic_wdgGetCfg(pPmicCoreHandle, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(7342,
                               pmic_wdg_tests,
                               PMIC_WDG_NUM_OF_TESTCASES);
}

/*!
 * \brief   Parameter validation for handle
 */
static void test_pmic_wdg_enable_prmValTest_handle(void)
{
    int32_t pmicStatus      = PMIC_ST_SUCCESS;

    test_pmic_print_unity_testcase_info(7343,
                                        pmic_wdg_tests,
                                        PMIC_WDG_NUM_OF_TESTCASES);

    pmicStatus = Pmic_wdgEnable(NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, pmicStatus);

    pmic_testResultUpdate_pass(7343,
                               pmic_wdg_tests,
                               PMIC_WDG_NUM_OF_TESTCASES);
}

/*!
 * \brief   Parameter validation for handle
 */
static void test_pmic_wdg_disable_prmValTest_handle(void)
{
    int32_t pmicStatus      = PMIC_ST_SUCCESS;

    test_pmic_print_unity_testcase_info(7344,
                                        pmic_wdg_tests,
                                        PMIC_WDG_NUM_OF_TESTCASES);

    pmicStatus = Pmic_wdgDisable(NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, pmicStatus);

    pmic_testResultUpdate_pass(7344,
                               pmic_wdg_tests,
                               PMIC_WDG_NUM_OF_TESTCASES);
}

/*!
 * \brief   Test wdg QA sequence Parameter validation for handle
 */
static void test_pmic_wdg_startQaSequence_prmValTest_handle(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint32_t maxCnt = 0xFFFFFFFFU;
    Pmic_WdgCfg_t wdgCfg =
    {
        PMIC_WDG_CFG_SETPARAMS_FORALL,
        24000U,
        6050U,
        4950U,
        PMIC_WDG_FAIL_THRESHOLD_COUNT_7,
        PMIC_WDG_RESET_THRESHOLD_COUNT_7,
        PMIC_WDG_QA_MODE,
        PMIC_WDG_PWRHOLD_DISABLE,
        PMIC_WDG_RESET_DISABLE,
        PMIC_WDG_RETLONGWIN_ENABLE,
        PMIC_WDG_QA_FEEDBACK_VALUE_0,
        PMIC_WDG_QA_LFSR_VALUE_0,
        PMIC_WDG_QA_QUES_SEED_VALUE_10,
    };

    test_pmic_print_unity_testcase_info(7345,
                                        pmic_wdg_tests,
                                        PMIC_WDG_NUM_OF_TESTCASES);

    /* Enable WDG Timer */
    pmicStatus = Pmic_wdgEnable(pPmicCoreHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    /* Set QA parameters */
    pmicStatus = Pmic_wdgSetCfg(pPmicCoreHandle, wdgCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    /* Start Watchdog QA sequence */
    pmicStatus = Pmic_wdgStartQaSequence(NULL, 5U, maxCnt);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, pmicStatus);

    /* Disable WDG Timer */
    pmicStatus = Pmic_wdgDisable(pPmicCoreHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmic_testResultUpdate_pass(7345,
                               pmic_wdg_tests,
                               PMIC_WDG_NUM_OF_TESTCASES);
}

/*!
 * \brief   Test wdg QA sequence
 */
static void test_pmic_wdg_startQaSequence(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint32_t maxCnt = PMIC_WDG_WAIT_CNT_MIN_VAL;
    Pmic_WdgCfg_t wdgCfg  =
    {
        PMIC_WDG_CFG_SETPARAMS_FORALL,
        750000U,
        6600U,
        4950U,
        PMIC_WDG_FAIL_THRESHOLD_COUNT_7,
        PMIC_WDG_RESET_THRESHOLD_COUNT_7,
        PMIC_WDG_QA_MODE,
        PMIC_WDG_PWRHOLD_DISABLE,
        PMIC_WDG_RESET_DISABLE,
        PMIC_WDG_RETLONGWIN_DISABLE,
        PMIC_WDG_QA_FEEDBACK_VALUE_0,
        PMIC_WDG_QA_LFSR_VALUE_0,
        PMIC_WDG_QA_QUES_SEED_VALUE_10,
    };

    if((gCrcTestFlag_J721E == PMIC_STATUS_CRC_ENABLED)||
       (gCrcTestFlag_J7VCL == PMIC_STATUS_CRC_ENABLED))
    {
        wdgCfg.win1Duration_us = 8250U;
    }

    test_pmic_print_unity_testcase_info(7346,
                                        pmic_wdg_tests,
                                        PMIC_WDG_NUM_OF_TESTCASES);

    if(PMIC_SILICON_REV_ID_PG_2_0 == pPmicCoreHandle->pmicDevSiliconRev)
    {
        wdgCfg.longWinDuration_ms = 752000U;
    }

    /* Enable WDG Timer */
    pmicStatus = Pmic_wdgEnable(pPmicCoreHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    /* Set QA parameters */
    pmicStatus = Pmic_wdgSetCfg(pPmicCoreHandle, wdgCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    /* Start Watchdog QA sequence */
    pmicStatus = Pmic_wdgStartQaSequence(pPmicCoreHandle, 5U, maxCnt);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    /* Disable WDG Timer */
    pmicStatus = Pmic_wdgDisable(pPmicCoreHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmic_testResultUpdate_pass(7346,
                               pmic_wdg_tests,
                               PMIC_WDG_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_wdgStartQaSequence - Test wdg QA sequences for Early Answer Error
 */
static void test_pmic_wdg_startQaSequence_answerEarlyErr(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint32_t maxCnt = PMIC_WDG_WAIT_CNT_MIN_VAL;
    Pmic_WdgCfg_t wdgCfg  =
    {
        PMIC_WDG_CFG_SETPARAMS_FORALL,
        750000U,
        8800U,
        4950U,
        PMIC_WDG_FAIL_THRESHOLD_COUNT_7,
        PMIC_WDG_RESET_THRESHOLD_COUNT_7,
        PMIC_WDG_QA_MODE,
        PMIC_WDG_PWRHOLD_DISABLE,
        PMIC_WDG_RESET_DISABLE,
        PMIC_WDG_RETLONGWIN_DISABLE,
        PMIC_WDG_QA_FEEDBACK_VALUE_0,
        PMIC_WDG_QA_LFSR_VALUE_0,
        PMIC_WDG_QA_QUES_SEED_VALUE_10,
    };

    if((gCrcTestFlag_J721E == PMIC_STATUS_CRC_ENABLED)||
       (gCrcTestFlag_J7VCL == PMIC_STATUS_CRC_ENABLED))
    {
        wdgCfg.win1Duration_us = 10350U;
    }

    test_pmic_print_unity_testcase_info(10785,
                                        pmic_wdg_tests,
                                        PMIC_WDG_NUM_OF_TESTCASES);

    if(PMIC_SILICON_REV_ID_PG_2_0 == pPmicCoreHandle->pmicDevSiliconRev)
    {
        wdgCfg.longWinDuration_ms = 752000U;
    }

    /* Enable WDG Timer */
    pmicStatus = Pmic_wdgEnable(pPmicCoreHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    /* Set QA parameters */
    pmicStatus = Pmic_wdgSetCfg(pPmicCoreHandle, wdgCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    /* Start Watchdog QA sequence */
    pmicStatus = Pmic_wdgStartQaSequence(pPmicCoreHandle, 1U, maxCnt);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_WDG_EARLY_ANSWER, pmicStatus);

    /* Disable WDG Timer */
    pmicStatus = Pmic_wdgDisable(pPmicCoreHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmic_testResultUpdate_pass(10785,
                               pmic_wdg_tests,
                               PMIC_WDG_NUM_OF_TESTCASES);
}

/*!
 * \brief   Test wdg QA sequences with different QA feedback values
 */
static void test_pmic_wdg_startQaSequence_testFdbkValues(void)
{
    int32_t pmicStatus      = PMIC_ST_SUCCESS;
    uint8_t fdbk = 0U;
    Pmic_WdgCfg_t wdgCfg_rd = {0U};
    uint32_t maxCnt = 0xFFFFFFFFU;
    Pmic_WdgCfg_t wdgCfg    =
    {
        PMIC_WDG_CFG_SETPARAMS_FORALL,
        750000U,
        6600U,
        4950U,
        PMIC_WDG_FAIL_THRESHOLD_COUNT_7,
        PMIC_WDG_RESET_THRESHOLD_COUNT_7,
        PMIC_WDG_QA_MODE,
        PMIC_WDG_PWRHOLD_DISABLE,
        PMIC_WDG_RESET_DISABLE,
        PMIC_WDG_RETLONGWIN_ENABLE,
        PMIC_WDG_QA_FEEDBACK_VALUE_0,
        PMIC_WDG_QA_LFSR_VALUE_0,
        PMIC_WDG_QA_QUES_SEED_VALUE_10,
    };

    if((gCrcTestFlag_J721E == PMIC_STATUS_CRC_ENABLED)||
       (gCrcTestFlag_J7VCL == PMIC_STATUS_CRC_ENABLED))
    {
        wdgCfg.win1Duration_us = 8350U;
    }

    test_pmic_print_unity_testcase_info(7347,
                                        pmic_wdg_tests,
                                        PMIC_WDG_NUM_OF_TESTCASES);

    if(PMIC_SILICON_REV_ID_PG_2_0 == pPmicCoreHandle->pmicDevSiliconRev)
    {
        wdgCfg.longWinDuration_ms = 752000U;
    }

    for(fdbk = PMIC_WDG_QA_FEEDBACK_VALUE_0;
        fdbk <= PMIC_WDG_QA_FEEDBACK_VALUE_3;
        fdbk++)
    {
        /* Enable WDG Timer */
        pmicStatus = Pmic_wdgEnable(pPmicCoreHandle);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        wdgCfg.qaFdbk = fdbk;
        /* Set QA parameters */
        pmicStatus = Pmic_wdgSetCfg(pPmicCoreHandle, wdgCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        /* Get wdg config to check fdbk value */
        wdgCfg_rd.validParams = PMIC_CFG_WDG_QA_FDBK_VALID_SHIFT;
        pmicStatus = Pmic_wdgGetCfg(pPmicCoreHandle, &wdgCfg_rd);
        TEST_ASSERT_EQUAL(wdgCfg.qaFdbk, wdgCfg_rd.qaFdbk);

        /* Start Watchdog QA sequence */
        pmicStatus = Pmic_wdgStartQaSequence(pPmicCoreHandle, 5, maxCnt);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        /* Disable WDG Timer */
        pmicStatus = Pmic_wdgDisable(pPmicCoreHandle);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        /* Delay is needed to sattle down watchdog to longwindow */
        Osal_delay(20U);
    }

    pmic_testResultUpdate_pass(7347,
                               pmic_wdg_tests,
                               PMIC_WDG_NUM_OF_TESTCASES);
}

/*!
 * \brief   Test wdg QA sequences with different QA LFSR values
 */
static void test_pmic_wdg_startQaSequence_testLfsrValues(void)
{
    int32_t pmicStatus      = PMIC_ST_SUCCESS;
    uint8_t lfsr = 0U;
    Pmic_WdgCfg_t wdgCfg_rd = {0U};
    uint32_t maxCnt = 0xFFFFFFFU;
    Pmic_WdgCfg_t wdgCfg    =
    {
        PMIC_WDG_CFG_SETPARAMS_FORALL,
        750000U,
        6600U,
        4950U,
        PMIC_WDG_FAIL_THRESHOLD_COUNT_7,
        PMIC_WDG_RESET_THRESHOLD_COUNT_7,
        PMIC_WDG_QA_MODE,
        PMIC_WDG_PWRHOLD_DISABLE,
        PMIC_WDG_RESET_DISABLE,
        PMIC_WDG_RETLONGWIN_ENABLE,
        PMIC_WDG_QA_FEEDBACK_VALUE_0,
        PMIC_WDG_QA_LFSR_VALUE_0,
        PMIC_WDG_QA_QUES_SEED_VALUE_10,
    };

    if((gCrcTestFlag_J721E == PMIC_STATUS_CRC_ENABLED)||
       (gCrcTestFlag_J7VCL == PMIC_STATUS_CRC_ENABLED))
    {
        wdgCfg.win1Duration_us = 8350U;
    }

    test_pmic_print_unity_testcase_info(7348,
                                        pmic_wdg_tests,
                                        PMIC_WDG_NUM_OF_TESTCASES);

    if(PMIC_SILICON_REV_ID_PG_2_0 == pPmicCoreHandle->pmicDevSiliconRev)
    {
        wdgCfg.longWinDuration_ms = 752000U;
    }

    for(lfsr = PMIC_WDG_QA_LFSR_VALUE_0;
        lfsr <= PMIC_WDG_QA_LFSR_VALUE_3;
        lfsr++)
    {
        /* Enable WDG Timer */
        pmicStatus = Pmic_wdgEnable(pPmicCoreHandle);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        wdgCfg.qaLfsr = lfsr;
        /* Set QA parameters */
        pmicStatus = Pmic_wdgSetCfg(pPmicCoreHandle, wdgCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        /* Get wdg config to check lfsr value */
        wdgCfg_rd.validParams = PMIC_CFG_WDG_QA_LFSR_VALID_SHIFT;
        pmicStatus = Pmic_wdgGetCfg(pPmicCoreHandle, &wdgCfg_rd);
        TEST_ASSERT_EQUAL(wdgCfg.qaLfsr, wdgCfg_rd.qaLfsr);

        /* Start Watchdog QA sequence */
        pmicStatus = Pmic_wdgStartQaSequence(pPmicCoreHandle, 5, maxCnt);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        /* Disable WDG Timer */
        pmicStatus = Pmic_wdgDisable(pPmicCoreHandle);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        /* Delay is needed to sattle down watchdog to longwindow */
        Osal_delay(20U);
    }

    pmic_testResultUpdate_pass(7348,
                               pmic_wdg_tests,
                               PMIC_WDG_NUM_OF_TESTCASES);
}

/*!
 * \brief   Test wdg QA sequences with different QA Question Seed values
 */
static void test_pmic_wdg_startQaSequence_testQuesSeedValues(void)
{
    int32_t pmicStatus      = PMIC_ST_SUCCESS;
    uint8_t quesSeed = 0U;
    Pmic_WdgCfg_t wdgCfg_rd = {0U};
    uint32_t maxCnt = 0xFFFFFFFFU;
    Pmic_WdgCfg_t wdgCfg    =
    {
        PMIC_WDG_CFG_SETPARAMS_FORALL,
        750000U,
        6600U,
        4950U,
        PMIC_WDG_FAIL_THRESHOLD_COUNT_7,
        PMIC_WDG_RESET_THRESHOLD_COUNT_7,
        PMIC_WDG_QA_MODE,
        PMIC_WDG_PWRHOLD_DISABLE,
        PMIC_WDG_RESET_DISABLE,
        PMIC_WDG_RETLONGWIN_DISABLE,
        PMIC_WDG_QA_FEEDBACK_VALUE_0,
        PMIC_WDG_QA_LFSR_VALUE_0,
        PMIC_WDG_QA_QUES_SEED_VALUE_10,
    };

    if((gCrcTestFlag_J721E == PMIC_STATUS_CRC_ENABLED)||
       (gCrcTestFlag_J7VCL == PMIC_STATUS_CRC_ENABLED))
    {
        wdgCfg.win1Duration_us = 8350U;
    }

    test_pmic_print_unity_testcase_info(7349,
                                        pmic_wdg_tests,
                                        PMIC_WDG_NUM_OF_TESTCASES);

    if(PMIC_SILICON_REV_ID_PG_2_0 == pPmicCoreHandle->pmicDevSiliconRev)
    {
        wdgCfg.longWinDuration_ms = 752000U;
    }

    for(quesSeed = PMIC_WDG_QA_QUES_SEED_VALUE_0;
        quesSeed <= PMIC_WDG_QA_QUES_SEED_VALUE_15;
        quesSeed++)
    {
        /* Enable WDG Timer */
        pmicStatus = Pmic_wdgEnable(pPmicCoreHandle);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        wdgCfg.qaQuesSeed = quesSeed;
        /* Set QA parameters */
        pmicStatus = Pmic_wdgSetCfg(pPmicCoreHandle, wdgCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        /* Get wdg config to check Question Seed value */
        wdgCfg_rd.validParams = PMIC_CFG_WDG_QA_QUES_SEED_VALID_SHIFT;
        pmicStatus = Pmic_wdgGetCfg(pPmicCoreHandle, &wdgCfg_rd);
        TEST_ASSERT_EQUAL(wdgCfg.qaQuesSeed, wdgCfg_rd.qaQuesSeed);

        /* Start Watchdog QA sequence */
        pmicStatus = Pmic_wdgStartQaSequence(pPmicCoreHandle, 5, maxCnt);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        /* Disable WDG Timer */
        pmicStatus = Pmic_wdgDisable(pPmicCoreHandle);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        /* Delay is needed to sattle down watchdog to longwindow */
        Osal_delay(20U);
    }

    pmic_testResultUpdate_pass(7349,
                               pmic_wdg_tests,
                               PMIC_WDG_NUM_OF_TESTCASES);
}

/*!
 * \brief   Test get wdg failcount status
 */
static void test_pmic_wdg_GetFailCount(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_WdgFailCntStat_t failCntStat = {0U};

    test_pmic_print_unity_testcase_info(7350,
                                        pmic_wdg_tests,
                                        PMIC_WDG_NUM_OF_TESTCASES);

    failCntStat.validParams = PMIC_CFG_WD_ALL_FAILCNTSTAT_VALID_PARAMS;
    /* Get watchdog failcount status */
    pmicStatus = Pmic_wdgGetFailCntStat(pPmicCoreHandle, &failCntStat);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    failCntStat.validParams = PMIC_CFG_WD_GOOD_EVENT_STAT_VALID_SHIFT;
    /* Get watchdog failcount status */
    pmicStatus = Pmic_wdgGetFailCntStat(pPmicCoreHandle, &failCntStat);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmic_testResultUpdate_pass(7350,
                               pmic_wdg_tests,
                               PMIC_WDG_NUM_OF_TESTCASES);
}

/*!
 * \brief   Test get wdg failcount Parameter validation for handle
 */
static void test_pmic_wdg_GetFailCount_prmValTest_handle(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_WdgFailCntStat_t failCntStat = {0U};

    test_pmic_print_unity_testcase_info(7351,
                                        pmic_wdg_tests,
                                        PMIC_WDG_NUM_OF_TESTCASES);

    failCntStat.validParams = PMIC_CFG_WD_ALL_FAILCNTSTAT_VALID_PARAMS;
    /* Get watchdog failcount */
    pmicStatus = Pmic_wdgGetFailCntStat(NULL, &failCntStat);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, pmicStatus);

    pmic_testResultUpdate_pass(7351,
                               pmic_wdg_tests,
                               PMIC_WDG_NUM_OF_TESTCASES);
}

/*!
 * \brief   Test get wdg failcount Parameter validation for failCount
 */
static void test_pmic_wdg_GetFailCount_prmValTest_invFailCountParam(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    test_pmic_print_unity_testcase_info(7352,
                                        pmic_wdg_tests,
                                        PMIC_WDG_NUM_OF_TESTCASES);

    /* Get watchdog failcount */
    pmicStatus = Pmic_wdgGetFailCntStat(pPmicCoreHandle, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(7352,
                               pmic_wdg_tests,
                               PMIC_WDG_NUM_OF_TESTCASES);
}

/*!
 * \brief   Test get wdg error status
 */
static void test_pmic_wdg_GetErrorStatus(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_WdgErrStatus_t errStatus = {0U};

    test_pmic_print_unity_testcase_info(7353,
                                        pmic_wdg_tests,
                                        PMIC_WDG_NUM_OF_TESTCASES);

    errStatus.validParams = PMIC_CFG_WD_ALL_ERRSTAT_VALID_PARAMS;
    /* Get watchdog error Status */
    pmicStatus = Pmic_wdgGetErrorStatus(pPmicCoreHandle, &errStatus);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmic_testResultUpdate_pass(7353,
                               pmic_wdg_tests,
                               PMIC_WDG_NUM_OF_TESTCASES);
}

/*!
 * \brief   Test get wdg error status Parameter validation for handle
 */
static void test_pmic_wdg_GetErrorStatus_prmValTest_handle(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_WdgErrStatus_t errStatus = {0U};

    test_pmic_print_unity_testcase_info(7354,
                                        pmic_wdg_tests,
                                        PMIC_WDG_NUM_OF_TESTCASES);

    errStatus.validParams = PMIC_CFG_WD_ALL_ERRSTAT_VALID_PARAMS;
    /* Get watchdog error Status */
    pmicStatus = Pmic_wdgGetErrorStatus(NULL, &errStatus);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, pmicStatus);

    pmic_testResultUpdate_pass(7354,
                               pmic_wdg_tests,
                               PMIC_WDG_NUM_OF_TESTCASES);
}

/*!
 * \brief   Test get wdg failcount Parameter validation for errStatus
 */
static void test_pmic_wdg_GetErrorStatus_prmValTest_invErrStatParam(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    test_pmic_print_unity_testcase_info(7355,
                                        pmic_wdg_tests,
                                        PMIC_WDG_NUM_OF_TESTCASES);

    /* Get watchdog error Status */
    pmicStatus = Pmic_wdgGetErrorStatus(pPmicCoreHandle, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(7355,
                               pmic_wdg_tests,
                               PMIC_WDG_NUM_OF_TESTCASES);
}

#if defined(ENABLE_SAMPLE_TESTCASES)
/*!
 * Below test cases are not tested because of HW limitation.
 * Added below test cases as sample for reference.
 *
 * PDK-7461 : PMIC: WDG Trigger mode can't be tested on J721E EVM
 */

/*!
 * \brief   Test wdg Trigger sequence
 */
static void test_pmic_wdg_StartTriggerSequence(void)
{
    int32_t pmicStatus      = PMIC_ST_SUCCESS;
    Pmic_WdgCfg_t wdgCfg    =
    {
        PMIC_WDG_CFG_SETPARAMS_FORALL,
        750000U,
        6050U,
        4950U,
        PMIC_WDG_FAIL_THRESHOLD_COUNT_7,
        PMIC_WDG_RESET_THRESHOLD_COUNT_7,
        PMIC_WDG_TRIGGER_MODE,
        PMIC_WDG_PWRHOLD_DISABLE,
        PMIC_WDG_RESET_ENABLE,
        PMIC_WDG_RETLONGWIN_DISABLE,
        PMIC_WDG_QA_FEEDBACK_VALUE_0,
        PMIC_WDG_QA_LFSR_VALUE_0,
        PMIC_WDG_QA_QUES_SEED_VALUE_10,
    };

    test_pmic_print_unity_testcase_info(0xAB17,
                                        pmic_wdg_tests,
                                        PMIC_WDG_NUM_OF_TESTCASES);

    if(J721E_LEO_PMICB_DEVICE == pmic_device_info)
    {
        pmic_testResultUpdate_ignore(0xAB17,
                                     pmic_wdg_tests,
                                     PMIC_WDG_NUM_OF_TESTCASES);
    }

    /* Test ignored, due to unsupported HW */
    pmic_testResultUpdate_ignore(0xAB17,
                                     pmic_wdg_tests,
                                     PMIC_WDG_NUM_OF_TESTCASES);

    if(PMIC_SILICON_REV_ID_PG_2_0 == pPmicCoreHandle->pmicDevSiliconRev)
    {
        wdgCfg.longWinDuration_ms = 752000U;
    }

    /* Enable WDG Timer */
    pmicStatus = Pmic_wdgEnable(pPmicCoreHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    /* Set Trigger parameters */
    pmicStatus = Pmic_wdgSetCfg(pPmicCoreHandle, wdgCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    /* Start Watchdog trigger sequence */
    pmicStatus = Pmic_wdgStartTriggerSequence(pPmicCoreHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    /* Disable WDG Timer */
    pmicStatus = Pmic_wdgDisable(pPmicCoreHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmic_testResultUpdate_pass(0xAB17,
                               pmic_wdg_tests,
                               PMIC_WDG_NUM_OF_TESTCASES);
}
#endif

/*!
 * \brief   Test Watchdog trigger mode Parameter validation for handle
 */
static void test_pmic_wdg_StartTriggerSequence_prmValTest_handle(void)
{
    int32_t pmicStatus      = PMIC_ST_SUCCESS;
    Pmic_WdgCfg_t wdgCfg    =
    {
        PMIC_WDG_CFG_SETPARAMS_FORALL,
        750000U,
        6050U,
        4950U,
        PMIC_WDG_FAIL_THRESHOLD_COUNT_7,
        PMIC_WDG_RESET_THRESHOLD_COUNT_7,
        PMIC_WDG_TRIGGER_MODE,
        PMIC_WDG_PWRHOLD_DISABLE,
        PMIC_WDG_RESET_DISABLE,
        PMIC_WDG_RETLONGWIN_DISABLE,
        PMIC_WDG_QA_FEEDBACK_VALUE_0,
        PMIC_WDG_QA_LFSR_VALUE_0,
        PMIC_WDG_QA_QUES_SEED_VALUE_10,
    };

    test_pmic_print_unity_testcase_info(7357,
                                        pmic_wdg_tests,
                                        PMIC_WDG_NUM_OF_TESTCASES);

    if(PMIC_SILICON_REV_ID_PG_2_0 == pPmicCoreHandle->pmicDevSiliconRev)
    {
        wdgCfg.longWinDuration_ms = 752000U;
    }

    /* Enable WDG Timer */
    pmicStatus = Pmic_wdgEnable(pPmicCoreHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    /* Set Trigger parameters */
    pmicStatus = Pmic_wdgSetCfg(pPmicCoreHandle, wdgCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    /* Start Watchdog trigger sequence */
    pmicStatus = Pmic_wdgStartTriggerSequence(NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, pmicStatus);

    /* Disable WDG Timer */
    pmicStatus = Pmic_wdgDisable(pPmicCoreHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmic_testResultUpdate_pass(7357,
                               pmic_wdg_tests,
                               PMIC_WDG_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_wdgStartQaSequence : Parameter validation for maxCnt
 */
static void test_pmic_wdg_startQaSequence_prmValTest_maxCnt(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint32_t maxCnt = PMIC_WDG_WAIT_CNT_MIN_VAL - 1U;
    Pmic_WdgCfg_t wdgCfg =
    {
        PMIC_WDG_CFG_SETPARAMS_FORALL,
        24000U,
        6050U,
        4950U,
        PMIC_WDG_FAIL_THRESHOLD_COUNT_7,
        PMIC_WDG_RESET_THRESHOLD_COUNT_7,
        PMIC_WDG_QA_MODE,
        PMIC_WDG_PWRHOLD_DISABLE,
        PMIC_WDG_RESET_DISABLE,
        PMIC_WDG_RETLONGWIN_ENABLE,
        PMIC_WDG_QA_FEEDBACK_VALUE_0,
        PMIC_WDG_QA_LFSR_VALUE_0,
        PMIC_WDG_QA_QUES_SEED_VALUE_10,
    };

    test_pmic_print_unity_testcase_info(7958,
                                        pmic_wdg_tests,
                                        PMIC_WDG_NUM_OF_TESTCASES);

    /* Enable WDG Timer */
    pmicStatus = Pmic_wdgEnable(pPmicCoreHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    /* Set QA parameters */
    pmicStatus = Pmic_wdgSetCfg(pPmicCoreHandle, wdgCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    /* Start Watchdog QA sequence */
    pmicStatus = Pmic_wdgStartQaSequence(pPmicCoreHandle, 5U, maxCnt);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

    /* Disable WDG Timer */
    pmicStatus = Pmic_wdgDisable(pPmicCoreHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmic_testResultUpdate_pass(7958,
                               pmic_wdg_tests,
                               PMIC_WDG_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_wdgClrErrStatus: Test to Clear all watchdog error status and
 *          to clear watchdog error status based on WDG Error Type
 */
static void test_pmic_wdg_ClrErrorStatus(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_WdgErrStatus_t errStatus = {0U};

    test_pmic_print_unity_testcase_info(10075,
                                        pmic_wdg_tests,
                                        PMIC_WDG_NUM_OF_TESTCASES);

    /* Enable WDG Timer */
    pmicStatus = Pmic_wdgEnable(pPmicCoreHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    errStatus.validParams = PMIC_CFG_WD_ALL_ERRSTAT_VALID_PARAMS;
    /* Get watchdog error Status */
    pmicStatus = Pmic_wdgGetErrorStatus(pPmicCoreHandle, &errStatus);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    /* Clear watchdog error Status */
    pmicStatus = Pmic_wdgClrErrStatus(pPmicCoreHandle, PMIC_WDG_ERR_ALL);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    /* Disable WDG Timer */
    pmicStatus = Pmic_wdgDisable(pPmicCoreHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmic_testResultUpdate_pass(10075,
                               pmic_wdg_tests,
                               PMIC_WDG_NUM_OF_TESTCASES);
}

/*!
 * \brief   Test get wdg clear status Parameter validation for handle
 */
static void test_pmic_wdg_ClrErrorStatus_prmValTest_handle(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    test_pmic_print_unity_testcase_info(10076,
                                        pmic_wdg_tests,
                                        PMIC_WDG_NUM_OF_TESTCASES);

    /* Clear watchdog error Status */
    pmicStatus = Pmic_wdgClrErrStatus(NULL, PMIC_WDG_ERR_ALL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, pmicStatus);


    pmic_testResultUpdate_pass(10076,
                               pmic_wdg_tests,
                               PMIC_WDG_NUM_OF_TESTCASES);
}

/*!
 * \brief   Test get wdg clear Parameter validation for wdgErrType
 */
static void test_pmic_wdg_ClrErrorStatus_prmValTest_wdgErrType(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    test_pmic_print_unity_testcase_info(10077,
                                        pmic_wdg_tests,
                                        PMIC_WDG_NUM_OF_TESTCASES);
    /* Clear watchdog error Status */
    pmicStatus = Pmic_wdgClrErrStatus(pPmicCoreHandle, 0x9U);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(10077,
                               pmic_wdg_tests,
                               PMIC_WDG_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_wdgQaSequenceWriteAnswer : Test wdg QA sequences
 */
static void test_pmic_wdgQaSequenceWriteAnswer_startQaSequence(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    int8_t  ansIndex   = 0;
    int32_t test_failure = PMIC_ST_SUCCESS;
    uint32_t numSequences = 5U, sequenceId;
    Pmic_WdgErrStatus_t errStatus = {0U};
    Pmic_WdgFailCntStat_t failCntStat = {0U};
    bool wdgBadEventStat = false;
    bool wdgGudEventStat = true;
    uint32_t failCntVal = 0;
    Pmic_WdgCfg_t wdgCfg  =
    {
        PMIC_WDG_CFG_SETPARAMS_FORALL,
        750000U,
        6050U,
        8250U,
        PMIC_WDG_FAIL_THRESHOLD_COUNT_7,
        PMIC_WDG_RESET_THRESHOLD_COUNT_7,
        PMIC_WDG_QA_MODE,
        PMIC_WDG_PWRHOLD_DISABLE,
        PMIC_WDG_RESET_ENABLE,
        PMIC_WDG_RETLONGWIN_DISABLE,
        PMIC_WDG_QA_FEEDBACK_VALUE_0,
        PMIC_WDG_QA_LFSR_VALUE_0,
        PMIC_WDG_QA_QUES_SEED_VALUE_10,
    };

    test_pmic_print_unity_testcase_info(10071,
                                        pmic_wdg_tests,
                                        PMIC_WDG_NUM_OF_TESTCASES);

    if(PMIC_SILICON_REV_ID_PG_2_0 == pPmicCoreHandle->pmicDevSiliconRev)
    {
        wdgCfg.longWinDuration_ms = 752000U;
        wdgCfg.win1Duration_us    = 7150U;
    }

    /* Enable WDG Timer */
    pmicStatus = Pmic_wdgEnable(pPmicCoreHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    /* Set QA parameters */
    wdgCfg.wdgMode = PMIC_WDG_QA_MODE;
    wdgCfg.retLongWin = PMIC_WDG_RETLONGWIN_DISABLE;

    pmicStatus = Pmic_wdgSetCfg(pPmicCoreHandle, wdgCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    errStatus.validParams = PMIC_CFG_WD_ALL_ERRSTAT_VALID_PARAMS;
    /* Get watchdog error Status */
    pmicStatus = Pmic_wdgGetErrorStatus(pPmicCoreHandle, &errStatus);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    /* Clear WDG Error bits */
    if((errStatus.wdLongWinTimeout != 0U) ||
       (errStatus.wdTimeout != 0U) ||
       (errStatus.wdTrigEarly != 0U) ||
       (errStatus.wdAnswearly != 0U) ||
       (errStatus.wdSeqErr != 0U) ||
       (errStatus.wdAnswErr != 0U) ||
       (errStatus.wdFailInt != 0U) ||
       (errStatus.wdRstInt != 0U))
    {
        pmicStatus = Pmic_wdgClrErrStatus(pPmicCoreHandle, PMIC_WDG_ERR_ALL);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    }

    /* Write Answers for Long Window */
    for(ansIndex = 3; ansIndex >= 0; ansIndex--)
    {
        pmicStatus = Pmic_wdgQaSequenceWriteAnswer(pPmicCoreHandle);
        if(PMIC_ST_SUCCESS != pmicStatus)
        {
            errStatus.validParams = PMIC_CFG_WD_ALL_ERRSTAT_VALID_PARAMS;
            /* Get watchdog error Status */
            pmicStatus = Pmic_wdgGetErrorStatus(pPmicCoreHandle, &errStatus);
            TEST_ASSERT_EQUAL(0U, errStatus.wdLongWinTimeout);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
        }
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    }

    /* Start Watchdog QA sequence */
    for(sequenceId = 0; sequenceId < numSequences; sequenceId++)
    {
        for(ansIndex = 3; ansIndex >= 0; ansIndex--)
        {
            pmicStatus = Pmic_wdgQaSequenceWriteAnswer(pPmicCoreHandle);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
        }

        errStatus.validParams = PMIC_CFG_WD_ALL_ERRSTAT_VALID_PARAMS;
        /* Get watchdog error Status */
        pmicStatus = Pmic_wdgGetErrorStatus(pPmicCoreHandle, &errStatus);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        failCntStat.validParams = PMIC_CFG_WD_ALL_FAILCNTSTAT_VALID_PARAMS;
        /* Get watchdog Fail Count Status */
        pmicStatus = Pmic_wdgGetFailCntStat(pPmicCoreHandle, &failCntStat);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        if((errStatus.wdTimeout != 0U) || (errStatus.wdAnswearly != 0U) ||
           (errStatus.wdSeqErr != 0U) || (errStatus.wdAnswErr != 0U) ||
           (errStatus.wdFailInt != 0U))
        {
            test_failure = PMIC_ST_ERR_FAIL;
            break;
        }

        if((failCntStat.wdBadEvent != false) || (failCntStat.wdFailCnt != 0U))
        {
            test_failure = PMIC_ST_ERR_FAIL;
            break;
        }
    }

    if(failCntStat.wdGudEvent != true)
    {
        test_failure = PMIC_ST_ERR_FAIL;
    }

    /* Set QA parameters */
    wdgCfg.retLongWin = PMIC_WDG_RETLONGWIN_ENABLE;
    pmicStatus = Pmic_wdgSetCfg(pPmicCoreHandle, wdgCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    /* Disable WDG Timer */
    pmicStatus = Pmic_wdgDisable(pPmicCoreHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, test_failure);
    TEST_ASSERT_EQUAL(wdgGudEventStat, failCntStat.wdGudEvent);
    TEST_ASSERT_EQUAL(wdgBadEventStat, failCntStat.wdBadEvent);
    TEST_ASSERT_EQUAL(failCntVal, failCntStat.wdFailCnt);

    pmic_testResultUpdate_pass(10071,
                               pmic_wdg_tests,
                               PMIC_WDG_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_wdgQaSequenceWriteAnswer : Test wdg QA sequences with different QA feedback values
 */
static void test_pmic_wdgQaSeqWriteAns_testFdbkValues(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_WdgCfg_t wdgCfg_rd = {0U};
    int8_t  ansIndex   = 0;
    int32_t test_failure = PMIC_ST_SUCCESS;
    uint32_t numSequences = 5U, sequenceId;
    uint32_t fdbk = 0;
    Pmic_WdgErrStatus_t errStatus = {0U};
    Pmic_WdgCfg_t wdgCfg  =
    {
        PMIC_WDG_CFG_SETPARAMS_FORALL,
        750000U,
        6050U,
        8250U,
        PMIC_WDG_FAIL_THRESHOLD_COUNT_1,
        PMIC_WDG_RESET_THRESHOLD_COUNT_1,
        PMIC_WDG_QA_MODE,
        PMIC_WDG_PWRHOLD_DISABLE,
        PMIC_WDG_RESET_DISABLE,
        PMIC_WDG_RETLONGWIN_DISABLE,
        PMIC_WDG_QA_FEEDBACK_VALUE_0,
        PMIC_WDG_QA_LFSR_VALUE_0,
        PMIC_WDG_QA_QUES_SEED_VALUE_10,
    };
    Pmic_WdgFailCntStat_t failCntStat = {0U};
    bool wdgBadEventStat = false;
    bool wdgGudEventStat = true;
    uint32_t failCntVal = 0;

    test_pmic_print_unity_testcase_info(10072,
                                        pmic_wdg_tests,
                                        PMIC_WDG_NUM_OF_TESTCASES);

    /*Refer the Bug PDK-10395 for more details*/
    pmic_testResultUpdate_ignore(10072,
                                 pmic_wdg_tests,
                                 PMIC_WDG_NUM_OF_TESTCASES);

    if(PMIC_SILICON_REV_ID_PG_2_0 == pPmicCoreHandle->pmicDevSiliconRev)
    {
        wdgCfg.longWinDuration_ms = 752000U;
        wdgCfg.win1Duration_us    = 7150U;
    }

    for(fdbk = PMIC_WDG_QA_FEEDBACK_VALUE_0;
        fdbk <= PMIC_WDG_QA_FEEDBACK_VALUE_3;
        fdbk++)
    {
        /* Enable WDG Timer */
        pmicStatus = Pmic_wdgEnable(pPmicCoreHandle);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        /* Set QA parameters */
        wdgCfg.wdgMode = PMIC_WDG_QA_MODE;
        wdgCfg.retLongWin = PMIC_WDG_RETLONGWIN_DISABLE;
        wdgCfg.qaFdbk = fdbk;

        pmicStatus = Pmic_wdgSetCfg(pPmicCoreHandle, wdgCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        /* Get wdg config to check fdbk value */
        wdgCfg_rd.validParams = PMIC_CFG_WDG_QA_FDBK_VALID_SHIFT;
        pmicStatus = Pmic_wdgGetCfg(pPmicCoreHandle, &wdgCfg_rd);
        TEST_ASSERT_EQUAL(wdgCfg.qaFdbk, wdgCfg_rd.qaFdbk);

        errStatus.validParams = PMIC_CFG_WD_ALL_ERRSTAT_VALID_PARAMS;
        /* Get watchdog error Status */
        pmicStatus = Pmic_wdgGetErrorStatus(pPmicCoreHandle, &errStatus);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        /* Clear WDG Error bits */
        if((errStatus.wdLongWinTimeout != 0U) ||
           (errStatus.wdTimeout != 0U) ||
           (errStatus.wdTrigEarly != 0U) ||
           (errStatus.wdAnswearly != 0U) ||
           (errStatus.wdSeqErr != 0U) ||
           (errStatus.wdAnswErr != 0U) ||
           (errStatus.wdFailInt != 0U) ||
           (errStatus.wdRstInt != 0U))
        {
            pmicStatus = Pmic_wdgClrErrStatus(pPmicCoreHandle, PMIC_WDG_ERR_ALL);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
        }

        /* Write Answers for Long Window */
        for(ansIndex = 3; ansIndex >= 0; ansIndex--)
        {
            pmicStatus = Pmic_wdgQaSequenceWriteAnswer(pPmicCoreHandle);
            if(PMIC_ST_SUCCESS != pmicStatus)
            {
                errStatus.validParams = PMIC_CFG_WD_ALL_ERRSTAT_VALID_PARAMS;
                /* Get watchdog error Status */
                pmicStatus = Pmic_wdgGetErrorStatus(pPmicCoreHandle, &errStatus);
                TEST_ASSERT_EQUAL(0U, errStatus.wdLongWinTimeout);
                TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
            }
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
        }

        /* Start Watchdog QA sequence */
        for(sequenceId = 0; sequenceId < numSequences; sequenceId++)
        {
            for(ansIndex = 3; ansIndex >= 0; ansIndex--)
            {
                pmicStatus = Pmic_wdgQaSequenceWriteAnswer(pPmicCoreHandle);
                TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
            }

            errStatus.validParams = PMIC_CFG_WD_ALL_ERRSTAT_VALID_PARAMS;
            /* Get watchdog error Status */
            pmicStatus = Pmic_wdgGetErrorStatus(pPmicCoreHandle, &errStatus);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

            failCntStat.validParams = PMIC_CFG_WD_ALL_FAILCNTSTAT_VALID_PARAMS;
            /* Get watchdog Fail Count Status */
            pmicStatus = Pmic_wdgGetFailCntStat(pPmicCoreHandle, &failCntStat);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

            if((errStatus.wdTimeout != 0U) || (errStatus.wdAnswearly != 0U) ||
               (errStatus.wdSeqErr != 0U) || (errStatus.wdAnswErr != 0U) ||
               (errStatus.wdFailInt != 0U))
            {
                test_failure = PMIC_ST_ERR_FAIL;
                break;
            }

            if((failCntStat.wdBadEvent != false) || (failCntStat.wdFailCnt != 0U))
            {
                test_failure = PMIC_ST_ERR_FAIL;
                break;
            }
        }

        if(failCntStat.wdGudEvent != true)
        {
            test_failure = PMIC_ST_ERR_FAIL;
        }

        /* Set QA parameters */
        wdgCfg.retLongWin = PMIC_WDG_RETLONGWIN_ENABLE;
        pmicStatus = Pmic_wdgSetCfg(pPmicCoreHandle, wdgCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        /* Disable WDG Timer */
        pmicStatus = Pmic_wdgDisable(pPmicCoreHandle);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, test_failure);
        TEST_ASSERT_EQUAL(wdgGudEventStat, failCntStat.wdGudEvent);
        TEST_ASSERT_EQUAL(wdgBadEventStat, failCntStat.wdBadEvent);
        TEST_ASSERT_EQUAL(failCntVal, failCntStat.wdFailCnt);

       /* Delay is needed to sattle down watchdog to longwindow */
        Osal_delay(20U);
    }

    pmic_testResultUpdate_pass(10072,
                               pmic_wdg_tests,
                               PMIC_WDG_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_wdgQaSequenceWriteAnswer : Test wdg QA sequences with different QA lfsr values
 */
static void test_pmic_wdgQaSeqWriteAns_testLfsrValues(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t lfsr = 0U;
    Pmic_WdgCfg_t wdgCfg_rd = {0U};
    int8_t  ansIndex   = 0;
    int32_t test_failure = PMIC_ST_SUCCESS;
    uint32_t numSequences = 5U, sequenceId;
    Pmic_WdgErrStatus_t errStatus = {0U};
    Pmic_WdgCfg_t wdgCfg  =
    {
        PMIC_WDG_CFG_SETPARAMS_FORALL,
        750000U,
        6050U,
        8250U,
        PMIC_WDG_FAIL_THRESHOLD_COUNT_7,
        PMIC_WDG_RESET_THRESHOLD_COUNT_7,
        PMIC_WDG_QA_MODE,
        PMIC_WDG_PWRHOLD_DISABLE,
        PMIC_WDG_RESET_DISABLE,
        PMIC_WDG_RETLONGWIN_DISABLE,
        PMIC_WDG_QA_FEEDBACK_VALUE_0,
        PMIC_WDG_QA_LFSR_VALUE_0,
        PMIC_WDG_QA_QUES_SEED_VALUE_10,
    };
    Pmic_WdgFailCntStat_t failCntStat = {0U};
    bool wdgBadEventStat = false;
    bool wdgGudEventStat = true;
    uint32_t failCntVal = 0;

    test_pmic_print_unity_testcase_info(10073,
                                        pmic_wdg_tests,
                                        PMIC_WDG_NUM_OF_TESTCASES);

    if(PMIC_SILICON_REV_ID_PG_2_0 == pPmicCoreHandle->pmicDevSiliconRev)
    {
        wdgCfg.longWinDuration_ms = 752000U;
        wdgCfg.win1Duration_us    = 7150U;
    }

    for(lfsr = PMIC_WDG_QA_LFSR_VALUE_0;
        lfsr <= PMIC_WDG_QA_LFSR_VALUE_3;
        lfsr++)
    {
        /* Enable WDG Timer */
        pmicStatus = Pmic_wdgEnable(pPmicCoreHandle);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        /* Set QA parameters */
        wdgCfg.wdgMode = PMIC_WDG_QA_MODE;
        wdgCfg.retLongWin = PMIC_WDG_RETLONGWIN_DISABLE;
        wdgCfg.qaLfsr = lfsr;

        pmicStatus = Pmic_wdgSetCfg(pPmicCoreHandle, wdgCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        /* Get wdg config to check fdbk value */
        wdgCfg_rd.validParams = PMIC_CFG_WDG_QA_LFSR_VALID_SHIFT;
        pmicStatus = Pmic_wdgGetCfg(pPmicCoreHandle, &wdgCfg_rd);
        TEST_ASSERT_EQUAL(wdgCfg.qaLfsr, wdgCfg_rd.qaLfsr);

        errStatus.validParams = PMIC_CFG_WD_ALL_ERRSTAT_VALID_PARAMS;
        /* Get watchdog error Status */
        pmicStatus = Pmic_wdgGetErrorStatus(pPmicCoreHandle, &errStatus);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        /* Clear WDG Error bits */
        if((errStatus.wdLongWinTimeout != 0U) ||
           (errStatus.wdTimeout != 0U) ||
           (errStatus.wdTrigEarly != 0U) ||
           (errStatus.wdAnswearly != 0U) ||
           (errStatus.wdSeqErr != 0U) ||
           (errStatus.wdAnswErr != 0U) ||
           (errStatus.wdFailInt != 0U) ||
           (errStatus.wdRstInt != 0U))
        {
            pmicStatus = Pmic_wdgClrErrStatus(pPmicCoreHandle, PMIC_WDG_ERR_ALL);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
        }

        /* Write Answers for Long Window */
        for(ansIndex = 3; ansIndex >= 0; ansIndex--)
        {
            pmicStatus = Pmic_wdgQaSequenceWriteAnswer(pPmicCoreHandle);
            if(PMIC_ST_SUCCESS != pmicStatus)
            {
                errStatus.validParams = PMIC_CFG_WD_ALL_ERRSTAT_VALID_PARAMS;
                /* Get watchdog error Status */
                pmicStatus = Pmic_wdgGetErrorStatus(pPmicCoreHandle, &errStatus);
                TEST_ASSERT_EQUAL(0U, errStatus.wdLongWinTimeout);
                TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
            }
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
        }

        /* Start Watchdog QA sequence */
        for(sequenceId = 0; sequenceId < numSequences; sequenceId++)
        {
            for(ansIndex = 3; ansIndex >= 0; ansIndex--)
            {
                pmicStatus = Pmic_wdgQaSequenceWriteAnswer(pPmicCoreHandle);
                TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
            }

            errStatus.validParams = PMIC_CFG_WD_ALL_ERRSTAT_VALID_PARAMS;
            /* Get watchdog error Status */
            pmicStatus = Pmic_wdgGetErrorStatus(pPmicCoreHandle, &errStatus);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

            failCntStat.validParams = PMIC_CFG_WD_ALL_FAILCNTSTAT_VALID_PARAMS;
            /* Get watchdog Fail Count Status */
            pmicStatus = Pmic_wdgGetFailCntStat(pPmicCoreHandle, &failCntStat);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

            if((errStatus.wdTimeout != 0U) || (errStatus.wdAnswearly != 0U) ||
               (errStatus.wdSeqErr != 0U) || (errStatus.wdAnswErr != 0U) ||
               (errStatus.wdFailInt != 0U))
            {
                test_failure = PMIC_ST_ERR_FAIL;
                break;
            }

            if((failCntStat.wdBadEvent != false) || (failCntStat.wdFailCnt != 0U))
            {
                test_failure = PMIC_ST_ERR_FAIL;
                break;
            }
        }

        if(failCntStat.wdGudEvent != true)
        {
            test_failure = PMIC_ST_ERR_FAIL;
        }

        /* Set QA parameters */
        wdgCfg.retLongWin = PMIC_WDG_RETLONGWIN_ENABLE;
        pmicStatus = Pmic_wdgSetCfg(pPmicCoreHandle, wdgCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        /* Disable WDG Timer */
        pmicStatus = Pmic_wdgDisable(pPmicCoreHandle);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, test_failure);
        TEST_ASSERT_EQUAL(wdgGudEventStat, failCntStat.wdGudEvent);
        TEST_ASSERT_EQUAL(wdgBadEventStat, failCntStat.wdBadEvent);
        TEST_ASSERT_EQUAL(failCntVal, failCntStat.wdFailCnt);

       /* Delay is needed to sattle down watchdog to longwindow */
        Osal_delay(20U);
    }

    pmic_testResultUpdate_pass(10073,
                               pmic_wdg_tests,
                               PMIC_WDG_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_wdgQaSequenceWriteAnswer : Test wdg QA sequences with different QA seed values
 */
static void test_pmic_wdgQaSeqWriteAns_testSeedValues(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t quesSeed = 0U;
    Pmic_WdgCfg_t wdgCfg_rd = {0U};
    int8_t  ansIndex   = 0;
    int32_t test_failure = PMIC_ST_SUCCESS;
    uint32_t numSequences = 5U, sequenceId;
    Pmic_WdgErrStatus_t errStatus = {0U};
    Pmic_WdgCfg_t wdgCfg  =
    {
        PMIC_WDG_CFG_SETPARAMS_FORALL,
        750000U,
        6050U,
        8250U,
        PMIC_WDG_FAIL_THRESHOLD_COUNT_7,
        PMIC_WDG_RESET_THRESHOLD_COUNT_7,
        PMIC_WDG_QA_MODE,
        PMIC_WDG_PWRHOLD_DISABLE,
        PMIC_WDG_RESET_DISABLE,
        PMIC_WDG_RETLONGWIN_DISABLE,
        PMIC_WDG_QA_FEEDBACK_VALUE_0,
        PMIC_WDG_QA_LFSR_VALUE_0,
        PMIC_WDG_QA_QUES_SEED_VALUE_10,
    };
    Pmic_WdgFailCntStat_t failCntStat = {0U};
    bool wdgBadEventStat = false;
    bool wdgGudEventStat = true;
    uint32_t failCntVal = 0;

    test_pmic_print_unity_testcase_info(10074,
                                        pmic_wdg_tests,
                                        PMIC_WDG_NUM_OF_TESTCASES);

    if(PMIC_SILICON_REV_ID_PG_2_0 == pPmicCoreHandle->pmicDevSiliconRev)
    {
        wdgCfg.longWinDuration_ms = 752000U;
        wdgCfg.win1Duration_us    = 7150U;
    }

    for(quesSeed = PMIC_WDG_QA_QUES_SEED_VALUE_0;
        quesSeed <= PMIC_WDG_QA_QUES_SEED_VALUE_15;
        quesSeed++)
    {
        /* Enable WDG Timer */
        pmicStatus = Pmic_wdgEnable(pPmicCoreHandle);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        /* Set QA parameters */
        wdgCfg.wdgMode = PMIC_WDG_QA_MODE;
        wdgCfg.retLongWin = PMIC_WDG_RETLONGWIN_DISABLE;
        wdgCfg.qaQuesSeed = quesSeed;

        pmicStatus = Pmic_wdgSetCfg(pPmicCoreHandle, wdgCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        /* Get wdg config to check fdbk value */
        wdgCfg_rd.validParams = PMIC_CFG_WDG_QA_QUES_SEED_VALID_SHIFT;
        pmicStatus = Pmic_wdgGetCfg(pPmicCoreHandle, &wdgCfg_rd);
        TEST_ASSERT_EQUAL(wdgCfg.qaQuesSeed, wdgCfg_rd.qaQuesSeed);

        errStatus.validParams = PMIC_CFG_WD_ALL_ERRSTAT_VALID_PARAMS;
        /* Get watchdog error Status */
        pmicStatus = Pmic_wdgGetErrorStatus(pPmicCoreHandle, &errStatus);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        /* Clear WDG Error bits */
        if((errStatus.wdLongWinTimeout != 0U) ||
           (errStatus.wdTimeout != 0U) ||
           (errStatus.wdTrigEarly != 0U) ||
           (errStatus.wdAnswearly != 0U) ||
           (errStatus.wdSeqErr != 0U) ||
           (errStatus.wdAnswErr != 0U) ||
           (errStatus.wdFailInt != 0U) ||
           (errStatus.wdRstInt != 0U))
        {
            pmicStatus = Pmic_wdgClrErrStatus(pPmicCoreHandle, PMIC_WDG_ERR_ALL);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
        }

        /* Write Answers for Long Window */
        for(ansIndex = 3; ansIndex >= 0; ansIndex--)
        {
            pmicStatus = Pmic_wdgQaSequenceWriteAnswer(pPmicCoreHandle);
            if(PMIC_ST_SUCCESS != pmicStatus)
            {
                errStatus.validParams = PMIC_CFG_WD_ALL_ERRSTAT_VALID_PARAMS;
                /* Get watchdog error Status */
                pmicStatus = Pmic_wdgGetErrorStatus(pPmicCoreHandle, &errStatus);
                TEST_ASSERT_EQUAL(0U, errStatus.wdLongWinTimeout);
                TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
            }
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
        }

        /* Start Watchdog QA sequence */
        for(sequenceId = 0; sequenceId < numSequences; sequenceId++)
        {
            for(ansIndex = 3; ansIndex >= 0; ansIndex--)
            {
                pmicStatus = Pmic_wdgQaSequenceWriteAnswer(pPmicCoreHandle);
                TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
            }

            errStatus.validParams = PMIC_CFG_WD_ALL_ERRSTAT_VALID_PARAMS;
            /* Get watchdog error Status */
            pmicStatus = Pmic_wdgGetErrorStatus(pPmicCoreHandle, &errStatus);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

            failCntStat.validParams = PMIC_CFG_WD_ALL_FAILCNTSTAT_VALID_PARAMS;
            /* Get watchdog Fail Count Status */
            pmicStatus = Pmic_wdgGetFailCntStat(pPmicCoreHandle, &failCntStat);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

            if((errStatus.wdTimeout != 0U) || (errStatus.wdAnswearly != 0U) ||
               (errStatus.wdSeqErr != 0U) || (errStatus.wdAnswErr != 0U) ||
               (errStatus.wdFailInt != 0U))
            {
                test_failure = PMIC_ST_ERR_FAIL;
                break;
            }

            if((failCntStat.wdBadEvent != false) || (failCntStat.wdFailCnt != 0U))
            {
                test_failure = PMIC_ST_ERR_FAIL;
                break;
            }
        }

        if(failCntStat.wdGudEvent != true)
        {
            test_failure = PMIC_ST_ERR_FAIL;
        }

        /* Set QA parameters */
        wdgCfg.retLongWin = PMIC_WDG_RETLONGWIN_ENABLE;
        pmicStatus = Pmic_wdgSetCfg(pPmicCoreHandle, wdgCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        /* Disable WDG Timer */
        pmicStatus = Pmic_wdgDisable(pPmicCoreHandle);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, test_failure);
        TEST_ASSERT_EQUAL(wdgGudEventStat, failCntStat.wdGudEvent);
        TEST_ASSERT_EQUAL(wdgBadEventStat, failCntStat.wdBadEvent);
        TEST_ASSERT_EQUAL(failCntVal, failCntStat.wdFailCnt);

       /* Delay is needed to sattle down watchdog to longwindow */
        Osal_delay(20U);
    }

    pmic_testResultUpdate_pass(10074,
                               pmic_wdg_tests,
                               PMIC_WDG_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_wdgSetCfg : Test to set longWinDuration as 100ms
 */
static void test_pmic_wdgsetCfg_longwin(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_WdgCfg_t wdgCfg_rd = {PMIC_WDG_CFG_SETPARAMS_FORALL, };
     Pmic_WdgCfg_t wdgCfg    =
    {
        PMIC_WDG_CFG_SETPARAMS_FORALL,
        100U,
        4950U,
        4950U,
        PMIC_WDG_FAIL_THRESHOLD_COUNT_5,
        PMIC_WDG_RESET_THRESHOLD_COUNT_6,
        PMIC_WDG_TRIGGER_MODE,
        PMIC_WDG_PWRHOLD_ENABLE,
        PMIC_WDG_RESET_DISABLE,
        PMIC_WDG_RETLONGWIN_DISABLE,
        PMIC_WDG_QA_FEEDBACK_VALUE_2,
        PMIC_WDG_QA_LFSR_VALUE_1,
        PMIC_WDG_QA_QUES_SEED_VALUE_5,
    };

    test_pmic_print_unity_testcase_info(10780,
                                        pmic_wdg_tests,
                                        PMIC_WDG_NUM_OF_TESTCASES);

    if(PMIC_SILICON_REV_ID_PG_2_0 == pPmicCoreHandle->pmicDevSiliconRev)
    {
        pmic_testResultUpdate_ignore(10780,
                                     pmic_wdg_tests,
                                     PMIC_WDG_NUM_OF_TESTCASES);
    }

    status = Pmic_wdgSetCfg(pPmicCoreHandle, wdgCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_wdgGetCfg(pPmicCoreHandle, &wdgCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(wdgCfg.longWinDuration_ms, wdgCfg_rd.longWinDuration_ms);

    pmic_testResultUpdate_pass(10780,
                               pmic_wdg_tests,
                               PMIC_WDG_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_wdgSetCfg : Parameter validation for longWinDuration_ min Value as 2999 ms for
 *          Leo PMIC PG1.0
 */
static void test_pmic_wdgsetCfg_prmValTest_longwinMin_2999ms(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_WdgCfg_t wdgCfg    =
    {
        PMIC_WDG_CFG_SETPARAMS_FORALL,
        2999U,
        4950U,
        4950U,
        PMIC_WDG_FAIL_THRESHOLD_COUNT_5,
        PMIC_WDG_RESET_THRESHOLD_COUNT_6,
        PMIC_WDG_TRIGGER_MODE,
        PMIC_WDG_PWRHOLD_ENABLE,
        PMIC_WDG_RESET_DISABLE,
        PMIC_WDG_RETLONGWIN_DISABLE,
        PMIC_WDG_QA_FEEDBACK_VALUE_2,
        PMIC_WDG_QA_LFSR_VALUE_1,
        PMIC_WDG_QA_QUES_SEED_VALUE_5,
    };

    test_pmic_print_unity_testcase_info(10781,
                                        pmic_wdg_tests,
                                        PMIC_WDG_NUM_OF_TESTCASES);

    if(PMIC_SILICON_REV_ID_PG_2_0 == pPmicCoreHandle->pmicDevSiliconRev)
    {
        pmic_testResultUpdate_ignore(10781,
                                     pmic_wdg_tests,
                                     PMIC_WDG_NUM_OF_TESTCASES);
    }

    status = Pmic_wdgSetCfg(pPmicCoreHandle, wdgCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_WDG_WINDOW, status);

    pmic_testResultUpdate_pass(10781,
                               pmic_wdg_tests,
                               PMIC_WDG_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_wdgStartTriggerSequence - Code coverage test for wdg Trigger sequence
 *          Added for code coverage test. Functionality Test is not done due to
 *          EVM Limitation
 */
static void test_pmic_wdg_StartTriggerSequence(void)
{
    int32_t pmicStatus      = PMIC_ST_SUCCESS;
    Pmic_WdgCfg_t wdgCfg    =
    {
        PMIC_WDG_CFG_SETPARAMS_FORALL,
        750000U,
        6050U,
        4950U,
        PMIC_WDG_FAIL_THRESHOLD_COUNT_7,
        PMIC_WDG_RESET_THRESHOLD_COUNT_7,
        PMIC_WDG_TRIGGER_MODE,
        PMIC_WDG_PWRHOLD_DISABLE,
        PMIC_WDG_RESET_DISABLE,
        PMIC_WDG_RETLONGWIN_DISABLE,
        PMIC_WDG_QA_FEEDBACK_VALUE_0,
        PMIC_WDG_QA_LFSR_VALUE_0,
        PMIC_WDG_QA_QUES_SEED_VALUE_10,
    };

    test_pmic_print_unity_testcase_info(10782,
                                        pmic_wdg_tests,
                                        PMIC_WDG_NUM_OF_TESTCASES);

    if(J721E_LEO_PMICB_DEVICE == pmic_device_info)
    {
        pmic_testResultUpdate_ignore(10782,
                                     pmic_wdg_tests,
                                     PMIC_WDG_NUM_OF_TESTCASES);
    }

    if(PMIC_SILICON_REV_ID_PG_2_0 == pPmicCoreHandle->pmicDevSiliconRev)
    {
        wdgCfg.longWinDuration_ms = 752000U;
    }

    /* Enable WDG Timer */
    pmicStatus = Pmic_wdgEnable(pPmicCoreHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    /* Set Trigger parameters */
    pmicStatus = Pmic_wdgSetCfg(pPmicCoreHandle, wdgCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    /* Start Watchdog trigger sequence */
    pmicStatus = Pmic_wdgStartTriggerSequence(pPmicCoreHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    /* Disable WDG Timer */
    pmicStatus = Pmic_wdgDisable(pPmicCoreHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmic_testResultUpdate_pass(10782,
                               pmic_wdg_tests,
                               PMIC_WDG_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_wdgClrErrStatus : Test to clear wdg error based on WDG Error
 *          type - TimeOut, SeqErr, AnsEarly, FailInt
 */
static void test_pmic_wdgClrErrStatus_wdgErrType(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_WdgErrStatus_t errStatus = {0U};
    Pmic_IrqStatus_t errStat  = {0U};
    int8_t  ansIndex   = 0;
    uint32_t numSequences = 1U, sequenceId;
    Pmic_WdgFailCntStat_t failCntStat = {0U};

    Pmic_WdgCfg_t wdgCfg  =
    {
        PMIC_WDG_CFG_SETPARAMS_FORALL,
        750000U,
        8050U,
        8250U,
        PMIC_WDG_FAIL_THRESHOLD_COUNT_0,
        PMIC_WDG_RESET_THRESHOLD_COUNT_0,
        PMIC_WDG_QA_MODE,
        PMIC_WDG_PWRHOLD_DISABLE,
        PMIC_WDG_RESET_DISABLE,
        PMIC_WDG_RETLONGWIN_DISABLE,
        PMIC_WDG_QA_FEEDBACK_VALUE_0,
        PMIC_WDG_QA_LFSR_VALUE_0,
        PMIC_WDG_QA_QUES_SEED_VALUE_10,
    };

    test_pmic_print_unity_testcase_info(10783,
                                        pmic_wdg_tests,
                                        PMIC_WDG_NUM_OF_TESTCASES);

    if(PMIC_SILICON_REV_ID_PG_2_0 == pPmicCoreHandle->pmicDevSiliconRev)
    {
        wdgCfg.longWinDuration_ms = 752000U;
        wdgCfg.win1Duration_us    = 9150U;
    }

    pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, true);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    /* Enable WDG Timer */
    pmicStatus = Pmic_wdgEnable(pPmicCoreHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_wdgSetCfg(pPmicCoreHandle, wdgCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    /* Write Answers for Long Window  - To generate Long Window Timeout error*/
    for(ansIndex = 3; ansIndex >= 0; ansIndex--)
    {
        pmicStatus = Pmic_wdgQaSequenceWriteAnswer(pPmicCoreHandle);
        if(PMIC_ST_SUCCESS != pmicStatus)
        {
            errStatus.validParams = PMIC_CFG_WD_ALL_ERRSTAT_VALID_PARAMS;
            /* Get watchdog error Status */
            pmicStatus = Pmic_wdgGetErrorStatus(pPmicCoreHandle, &errStatus);
            TEST_ASSERT_EQUAL(0U, errStatus.wdLongWinTimeout);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
        }
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    }

    /* Start Watchdog QA sequence */
    for(sequenceId = 0; sequenceId < numSequences; sequenceId++)
    {
        for(ansIndex = 3; ansIndex >= 0; ansIndex--)
        {
            pmicStatus = Pmic_wdgQaSequenceWriteAnswer(pPmicCoreHandle);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
        }

        errStatus.validParams = PMIC_CFG_WD_ALL_ERRSTAT_VALID_PARAMS;
        /* Get watchdog error Status */
        pmicStatus = Pmic_wdgGetErrorStatus(pPmicCoreHandle, &errStatus);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        failCntStat.validParams = PMIC_CFG_WD_ALL_FAILCNTSTAT_VALID_PARAMS;
        /* Get watchdog Fail Count Status */
        pmicStatus = Pmic_wdgGetFailCntStat(pPmicCoreHandle, &failCntStat);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        if(failCntStat.wdFailCnt != 0U)
        {
            break;
        }
    }

    /* Set QA parameters */
    wdgCfg.retLongWin = PMIC_WDG_RETLONGWIN_ENABLE;
    pmicStatus = Pmic_wdgSetCfg(pPmicCoreHandle, wdgCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    /* Disable WDG Timer */
    pmicStatus = Pmic_wdgDisable(pPmicCoreHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    errStatus.validParams = PMIC_CFG_WD_ALL_ERRSTAT_VALID_PARAMS;
    /* Get watchdog error Status */
    pmicStatus = Pmic_wdgGetErrorStatus(pPmicCoreHandle, &errStatus);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, false);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    if((errStat.intStatus[PMIC_TPS6594X_WD_FAIL_INT/32U] &
            (1U << (PMIC_TPS6594X_WD_FAIL_INT % 32U))) == 0U)
    {
        pmicStatus = PMIC_ST_ERR_FAIL;
    }
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);


    /* Clear WDG Timeout Error bits */
    pmicStatus = Pmic_wdgClrErrStatus(pPmicCoreHandle, PMIC_WDG_ERR_TIMEOUT);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    /* Clear WDG Seq Error bits */
    pmicStatus = Pmic_wdgClrErrStatus(pPmicCoreHandle, PMIC_WDG_ERR_SEQ_ERR);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    /* Clear WDG Timeout Error bits - for code coverage false case of PMIC_WDG_ERR_TIMEOUT second if case*/
    pmicStatus = Pmic_wdgClrErrStatus(pPmicCoreHandle, PMIC_WDG_ERR_TIMEOUT);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    /* Clear WDG Seq Error bits - for code coverage false case of PMIC_WDG_ERR_SEQ_ERR second if case */
    pmicStatus = Pmic_wdgClrErrStatus(pPmicCoreHandle, PMIC_WDG_ERR_SEQ_ERR);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    /* Clear WDG Seq Error bits - for code coverage false case of PMIC_WDG_ERR_LONG_WIN_TIMEOUT second if case */
    pmicStatus = Pmic_wdgClrErrStatus(pPmicCoreHandle, PMIC_WDG_ERR_LONG_WIN_TIMEOUT);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    /* Clear WDG Seq Error bits - for code coverage false case of PMIC_WDG_ERR_TRIGGER_EARLY second if case */
    pmicStatus = Pmic_wdgClrErrStatus(pPmicCoreHandle, PMIC_WDG_ERR_TRIGGER_EARLY);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    /* Clear WDG Seq Error bits - for code coverage false case of PMIC_WDG_ERR_ANS_ERR second if case */
    pmicStatus = Pmic_wdgClrErrStatus(pPmicCoreHandle, PMIC_WDG_ERR_ANS_ERR);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    /* Clear WDG Answer Early timeout Error bits */
    pmicStatus = Pmic_wdgClrErrStatus(pPmicCoreHandle, PMIC_WDG_ERR_ANSWER_EARLY);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    /* Clear WDG Seq Error bits - for code coverage false case of PMIC_WDG_ERR_ANSWER_EARLY second if case */
    pmicStatus = Pmic_wdgClrErrStatus(pPmicCoreHandle, PMIC_WDG_ERR_ANSWER_EARLY);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    /* Clear WDG Fail Int timeout Error bits */
    pmicStatus = Pmic_wdgClrErrStatus(pPmicCoreHandle, PMIC_WDG_ERR_FAIL_INT);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, true);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmic_testResultUpdate_pass(10783,
                               pmic_wdg_tests,
                               PMIC_WDG_NUM_OF_TESTCASES);
}


/*!
 * \brief   Pmic_wdgQaSequenceWriteAnswer : Manual Test case to test WDG RST INT and
 *          WDG LONGWIN TIMEOUT INT
 */
static void test_pmic_wdgQaSequenceWriteAnswer_RstIntLongWinTimeoutInt(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_WdgErrStatus_t errStatus = {0U};
    Pmic_IrqStatus_t errStat  = {0U};
    int8_t  ansIndex   = 0;

    Pmic_WdgCfg_t wdgCfg  =
    {
        PMIC_WDG_CFG_SETPARAMS_FORALL,
        750000U,
        6050U,
        8250U,
        PMIC_WDG_FAIL_THRESHOLD_COUNT_0,
        PMIC_WDG_RESET_THRESHOLD_COUNT_0,
        PMIC_WDG_QA_MODE,
        PMIC_WDG_PWRHOLD_DISABLE,
        PMIC_WDG_RESET_ENABLE,
        PMIC_WDG_RETLONGWIN_DISABLE,
        PMIC_WDG_QA_FEEDBACK_VALUE_0,
        PMIC_WDG_QA_LFSR_VALUE_0,
        PMIC_WDG_QA_QUES_SEED_VALUE_10,
    };

    test_pmic_print_unity_testcase_info(10784,
                                        pmic_wdg_tests,
                                        PMIC_WDG_NUM_OF_TESTCASES);

    if(PMIC_SILICON_REV_ID_PG_2_0 == pPmicCoreHandle->pmicDevSiliconRev)
    {
        wdgCfg.longWinDuration_ms = 752000U;
        wdgCfg.win1Duration_us    = 7150U;
    }

    pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, true);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmic_log("\r\n After reset Rerun the test and check for WD_RST_INT and WD_LONGWIN_TIMEOUT_INT bit is set on Interrupt status register\n");

    /* Enable WDG Timer */
    pmicStatus = Pmic_wdgEnable(pPmicCoreHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_wdgSetCfg(pPmicCoreHandle, wdgCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    /* Write Answers for Long Window  - To generate Long Window Timeout error*/
    for(ansIndex = 3; ansIndex >= 0; ansIndex--)
    {
        pmicStatus = Pmic_wdgQaSequenceWriteAnswer(pPmicCoreHandle);
        if(PMIC_ST_SUCCESS != pmicStatus)
        {
            errStatus.validParams = PMIC_CFG_WD_ALL_ERRSTAT_VALID_PARAMS;
            /* Get watchdog error Status */
            pmicStatus = Pmic_wdgGetErrorStatus(pPmicCoreHandle, &errStatus);
            TEST_ASSERT_EQUAL(0U, errStatus.wdLongWinTimeout);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
        }
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    }

    pmic_testResultUpdate_pass(10784,
                               pmic_wdg_tests,
                               PMIC_WDG_NUM_OF_TESTCASES);
}

/*
 * \brief   Added for Coverage
 */
static void test_pmic_wdg_coverageGaps(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t testCommMode;
    Pmic_WdgFailCntStat_t failCntStat = {0U};
    Pmic_WdgErrStatus_t errStatus = {0U};
    Pmic_WdgCfg_t wdgCfg_rd = {PMIC_WDG_CFG_SETPARAMS_FORALL, };
    Pmic_WdgCfg_t wdgCfg    =
    {
        PMIC_WDG_CFG_SETPARAMS_FORALL,
        750000U,
        4950U,
        4950U,
        PMIC_WDG_FAIL_THRESHOLD_COUNT_5,
        PMIC_WDG_RESET_THRESHOLD_COUNT_6,
        PMIC_WDG_TRIGGER_MODE,
        PMIC_WDG_PWRHOLD_ENABLE,
        PMIC_WDG_RESET_DISABLE,
        PMIC_WDG_RETLONGWIN_DISABLE,
        PMIC_WDG_QA_FEEDBACK_VALUE_2,
        PMIC_WDG_QA_LFSR_VALUE_1,
        PMIC_WDG_QA_QUES_SEED_VALUE_5,
    };

    test_pmic_print_unity_testcase_info(8917,
                                        pmic_wdg_tests,
                                        PMIC_WDG_NUM_OF_TESTCASES);

    if(PMIC_INTF_SPI == pPmicCoreHandle->commMode)
    {
        pmic_testResultUpdate_ignore(8917,
                                     pmic_wdg_tests,
                                     PMIC_WDG_NUM_OF_TESTCASES);
    }

    if(PMIC_SILICON_REV_ID_PG_2_0 == pPmicCoreHandle->pmicDevSiliconRev)
    {
        wdgCfg.longWinDuration_ms = 752000U;
    }

    status = Pmic_wdgSetCfg(pPmicCoreHandle, wdgCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    //Fault Injection Tests
    gPmic_faultInjectCfg.enableFaultInjectionRead = 1U;
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    wdgCfg_rd.validParams = PMIC_WDG_CFG_SETPARAMS_FORALL;
    status = Pmic_wdgGetCfg(pPmicCoreHandle, &wdgCfg_rd);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 2;
    status = Pmic_wdgGetCfg(pPmicCoreHandle, &wdgCfg_rd);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 3;
    status = Pmic_wdgGetCfg(pPmicCoreHandle, &wdgCfg_rd);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 4;
    status = Pmic_wdgGetCfg(pPmicCoreHandle, &wdgCfg_rd);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 5;
    status = Pmic_wdgGetCfg(pPmicCoreHandle, &wdgCfg_rd);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    wdgCfg.validParams = PMIC_CFG_WDG_RETLONGWIN_VALID_SHIFT;
    status = Pmic_wdgSetCfg(pPmicCoreHandle, wdgCfg);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    wdgCfg.validParams = PMIC_CFG_WDG_WDGMODE_VALID_SHIFT |
                         PMIC_CFG_WDG_RSTENABLE_VALID_SHIFT |
                         PMIC_CFG_WDG_PWRHOLD_VALID_SHIFT |
                         PMIC_CFG_WDG_RETLONGWIN_VALID_SHIFT;
    status = Pmic_wdgSetCfg(pPmicCoreHandle, wdgCfg);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    wdgCfg.validParams = PMIC_CFG_WDG_RSTENABLE_VALID_SHIFT;
    status = Pmic_wdgSetCfg(pPmicCoreHandle, wdgCfg);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    wdgCfg.validParams = PMIC_CFG_WDG_RSTENABLE_VALID_SHIFT |
                         PMIC_CFG_WDG_PWRHOLD_VALID_SHIFT |
                         PMIC_CFG_WDG_RETLONGWIN_VALID_SHIFT;
    status = Pmic_wdgSetCfg(pPmicCoreHandle, wdgCfg);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    wdgCfg.validParams = PMIC_CFG_WDG_PWRHOLD_VALID_SHIFT |
                         PMIC_CFG_WDG_RETLONGWIN_VALID_SHIFT;
    status = Pmic_wdgSetCfg(pPmicCoreHandle, wdgCfg);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    wdgCfg.validParams = PMIC_CFG_WDG_RETLONGWIN_VALID_SHIFT;
    status = Pmic_wdgSetCfg(pPmicCoreHandle, wdgCfg);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    wdgCfg.validParams = PMIC_CFG_WDG_QA_FDBK_VALID_SHIFT |
                         PMIC_CFG_WDG_QA_LFSR_VALID_SHIFT;
    status = Pmic_wdgSetCfg(pPmicCoreHandle, wdgCfg);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    wdgCfg.validParams = PMIC_CFG_WDG_QA_LFSR_VALID_SHIFT |
                         PMIC_CFG_WDG_QA_QUES_SEED_VALID_SHIFT;
    status = Pmic_wdgSetCfg(pPmicCoreHandle, wdgCfg);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    wdgCfg.validParams = PMIC_CFG_WDG_QA_FDBK_VALID_SHIFT |
                         PMIC_CFG_WDG_QA_QUES_SEED_VALID_SHIFT;
    status = Pmic_wdgSetCfg(pPmicCoreHandle, wdgCfg);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 3;
    wdgCfg_rd.validParams = PMIC_CFG_WDG_QA_FDBK_VALID_SHIFT |
                            PMIC_CFG_WDG_QA_QUES_SEED_VALID_SHIFT |
                            PMIC_CFG_WDG_QA_LFSR_VALID_SHIFT;
    status = Pmic_wdgGetCfg(pPmicCoreHandle, &wdgCfg_rd);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    status = Pmic_wdgEnable(pPmicCoreHandle);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    gPmic_faultInjectCfg.enableFaultInjectionRead = 0U;
    //WdgQACOunt
    uint32_t maxCnt = PMIC_WDG_WAIT_CNT_MIN_VAL;
    Pmic_WdgCfg_t wdgCfg_QA  =
    {
        PMIC_WDG_CFG_SETPARAMS_FORALL,
        750000U,
        6150U,
        4950U,
        PMIC_WDG_FAIL_THRESHOLD_COUNT_7,
        PMIC_WDG_RESET_THRESHOLD_COUNT_7,
        PMIC_WDG_QA_MODE,
        PMIC_WDG_PWRHOLD_DISABLE,
        PMIC_WDG_RESET_DISABLE,
        PMIC_WDG_RETLONGWIN_DISABLE,
        PMIC_WDG_QA_FEEDBACK_VALUE_0,
        PMIC_WDG_QA_LFSR_VALUE_0,
        PMIC_WDG_QA_QUES_SEED_VALUE_10,
    };

    if(PMIC_SILICON_REV_ID_PG_2_0 == pPmicCoreHandle->pmicDevSiliconRev)
    {
        wdgCfg.longWinDuration_ms = 752000U;
    }

    /* Enable WDG Timer */
    status = Pmic_wdgEnable(pPmicCoreHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    /* Set QA parameters */
    status = Pmic_wdgSetCfg(pPmicCoreHandle, wdgCfg_QA);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    /* Start Watchdog QA sequence */
    gPmic_faultInjectCfg.enableFaultInjectionRead = 1U;
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 5;
    status = Pmic_wdgStartQaSequence(pPmicCoreHandle, 5U, maxCnt);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 7;
    status = Pmic_wdgStartQaSequence(pPmicCoreHandle, 5U, maxCnt);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 8;
    status = Pmic_wdgStartQaSequence(pPmicCoreHandle, 5U, maxCnt);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 4;
    status = Pmic_wdgStartQaSequence(pPmicCoreHandle, 5U, maxCnt);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    status = Pmic_wdgStartQaSequence(pPmicCoreHandle, 5U, maxCnt);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    status = Pmic_wdgStartTriggerSequence(pPmicCoreHandle);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    /* Disable WDG Timer */
    status = Pmic_wdgDisable(pPmicCoreHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    Osal_delay(20U);


    //Pmic_wdgClrErrStatus
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    status = Pmic_wdgClrErrStatus(pPmicCoreHandle, PMIC_WDG_ERR_ALL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_FAIL, status);


/* TBD - Due to these code coverage tests in this below if case  - if(PMIC_INTF_DUAL_I2C == pPmicCoreHandle->commMode)
 * 7327, 7340 tests is failed for Single I2C So ignored for time being
 */
    if(PMIC_INTF_DUAL_I2C == pPmicCoreHandle->commMode)
    {
        /* Enable WDG Timer */
        status = Pmic_wdgEnable(pPmicCoreHandle);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        //Pmic_wdgStartQaSequence
        gPmic_faultInjectCfg.readCount = 0;
        gPmic_faultInjectCfg.skipReadCount = 20;
        status = Pmic_wdgStartQaSequence(pPmicCoreHandle, 1U, maxCnt);
        TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

        /* Disable WDG Timer */
        status = Pmic_wdgDisable(pPmicCoreHandle);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        Osal_delay(20U);

        if(PMIC_SILICON_REV_ID_PG_1_0 == pPmicCoreHandle->pmicDevSiliconRev)
        {
            /* Enable WDG Timer */
            status = Pmic_wdgEnable(pPmicCoreHandle);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

            //is_wdgBadEventDetected
            gPmic_faultInjectCfg.readCount = 0;
            gPmic_faultInjectCfg.skipReadCount = 10;
            status = Pmic_wdgStartQaSequence(pPmicCoreHandle, 1U, maxCnt);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

            /* Disable WDG Timer */
            status = Pmic_wdgDisable(pPmicCoreHandle);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

            Osal_delay(20U);
        }

        /* Enable WDG Timer */
        status = Pmic_wdgEnable(pPmicCoreHandle);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        //Pmic_wdgQaSetModeRetlongwinCfgWriteAnswersLongwindow
        gFailCntStatTestFlag = 1U;
        gFailCount = 0;
        gSkipFailStatusCount = 1;
        gPmic_faultInjectCfg.readCount = 0;
        gPmic_faultInjectCfg.skipReadCount = 11;

        status = Pmic_wdgStartQaSequence(pPmicCoreHandle, 1U, maxCnt);
        TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);
        gFailCntStatTestFlag = 0U;

        /* Disable WDG Timer */
        status = Pmic_wdgDisable(pPmicCoreHandle);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        Osal_delay(20U);

        /* Enable WDG Timer */
        status = Pmic_wdgEnable(pPmicCoreHandle);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        //Pmic_wdgQaWriteAnswersNumSequence
        gFailCntStatTestFlag = 1U;
        gFailCount = 0;
        gSkipFailStatusCount = 5;
        gPmic_faultInjectCfg.readCount = 0;
        gPmic_faultInjectCfg.skipReadCount = 22;
        status = Pmic_wdgStartQaSequence(pPmicCoreHandle, 1U, maxCnt);
        TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);
        gFailCntStatTestFlag = 0U;

        /* Disable WDG Timer */
        status = Pmic_wdgDisable(pPmicCoreHandle);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        Osal_delay(20U);

        if(pmic_device_info == J721E_LEO_PMICA_DEVICE)
        {
            /* Enable WDG Timer */
            status = Pmic_wdgEnable(pPmicCoreHandle);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

            //Pmic_wdgQaWriteAnswersNumSequence
            gPmic_faultInjectCfg.readCount = 0;
            gPmic_faultInjectCfg.skipReadCount = 25;
            status = Pmic_wdgStartQaSequence(pPmicCoreHandle, 1U, maxCnt);
            TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

            /* Disable WDG Timer */
            status = Pmic_wdgDisable(pPmicCoreHandle);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

            Osal_delay(20U);
        }
    }

    if((PMIC_SILICON_REV_ID_PG_1_0 == pPmicCoreHandle->pmicDevSiliconRev) &&
       (PMIC_INTF_DUAL_I2C == pPmicCoreHandle->commMode))
    {
        /* Enable WDG Timer */
        status = Pmic_wdgEnable(pPmicCoreHandle);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        //Pmic_wdgQaSetModeRetlongwinCfgWriteAnswersLongwindow
        gErrorStatusTestFlag = 1U;
        gFailCntStatTestFlag = 1U;
        gFailCount = 0;
        gSkipFailStatusCount = 1;
        gErrStatusCount = 0;
        gSkipErrStatusCount = 2;
        status = Pmic_wdgStartQaSequence(pPmicCoreHandle, 1U, maxCnt);
        TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_WDG_WINDOW, status);
        gFailCntStatTestFlag = 0U;
        gErrorStatusTestFlag = 0U;

        /* Disable WDG Timer */
        status = Pmic_wdgDisable(pPmicCoreHandle);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    }

    gPmic_faultInjectCfg.enableFaultInjectionRead = 0U;

    if((PMIC_SILICON_REV_ID_PG_1_0 == pPmicCoreHandle->pmicDevSiliconRev) &&
       (PMIC_INTF_DUAL_I2C == pPmicCoreHandle->commMode))
    {
        /* Enable WDG Timer */
        status = Pmic_wdgEnable(pPmicCoreHandle);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        //Pmic_wdgQaSetModeRetlongwinCfgWriteAnswersLongwindow
        gErrorStatusTestFlag = 1U;
        gFailCntStatTestFlag = 1U;
        gFailCount = 0;
        gSkipFailStatusCount = 1;
        gErrStatusCount = 0;
        gSkipErrStatusCount = 2;
        status = Pmic_wdgStartQaSequence(pPmicCoreHandle, 1U, maxCnt);
        TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_WDG_WINDOW, status);
        gFailCntStatTestFlag = 0U;
        gErrorStatusTestFlag = 0U;

        /* Disable WDG Timer */
        status = Pmic_wdgDisable(pPmicCoreHandle);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    }

    gPmic_faultInjectCfg.enableFaultInjectionWrite = 1U;

    /* Enable WDG Timer */
    status = Pmic_wdgEnable(pPmicCoreHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    //Pmic_wdgQaEvaluateAndWriteAnswers
    gPmic_faultInjectCfg.writeCount = 0;
    gPmic_faultInjectCfg.skipWriteCount = 4;
    status = Pmic_wdgStartQaSequence(pPmicCoreHandle, 1U, maxCnt);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    gPmic_faultInjectCfg.enableFaultInjectionWrite = 0U;

    /* Disable WDG Timer */
    status = Pmic_wdgDisable(pPmicCoreHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    if(PMIC_SILICON_REV_ID_PG_1_0 == pPmicCoreHandle->pmicDevSiliconRev)
    {
        gFailCntStatTestFlag = 1U;
        gFailCount = 0;
        gSkipFailStatusCount = 1;
        failCntStat.validParams = PMIC_CFG_WD_BAD_EVENT_STAT_VALID_SHIFT;
        /* Get watchdog failcount status */
        status = Pmic_wdgGetFailCntStat(pPmicCoreHandle, &failCntStat);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        gFailCntStatTestFlag = 0U;

        gErrorStatusTestFlag = 1U;
        gErrStatusCount = 0;
        gSkipErrStatusCount = 1;
        errStatus.validParams = PMIC_CFG_WD_LONGWIN_TIMEOUT_ERRSTAT_VALID_SHIFT;
        /* Get watchdog error Status */
        status = Pmic_wdgGetErrorStatus(pPmicCoreHandle, &errStatus);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        gErrStatusCount = 0;
        gSkipErrStatusCount = 1;
        /* Clear WDG Long Window timeout Error bits */
        status = Pmic_wdgClrErrStatus(pPmicCoreHandle, PMIC_WDG_ERR_LONG_WIN_TIMEOUT);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        gErrStatusCount = 0;
        gSkipErrStatusCount = 1;
        errStatus.validParams = PMIC_CFG_WD_TIMEOUT_ERRSTAT_VALID_SHIFT;
        /* Get watchdog error Status */
        status = Pmic_wdgGetErrorStatus(pPmicCoreHandle, &errStatus);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        gErrStatusCount = 0;
        gSkipErrStatusCount = 1;
        /* Clear WDG Timeout Error bits */
        status = Pmic_wdgClrErrStatus(pPmicCoreHandle, PMIC_WDG_ERR_TIMEOUT);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        gErrStatusCount = 0;
        gSkipErrStatusCount = 1;
        errStatus.validParams = PMIC_CFG_WD_TRIG_EARLY_ERRSTAT_VALID_SHIFT;
        /* Get watchdog error Status */
        status = Pmic_wdgGetErrorStatus(pPmicCoreHandle, &errStatus);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        gErrStatusCount = 0;
        gSkipErrStatusCount = 1;
        /* Clear WDG Trigger Early timeout Error bits */
        status = Pmic_wdgClrErrStatus(pPmicCoreHandle, PMIC_WDG_ERR_TRIGGER_EARLY);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        gErrStatusCount = 0;
        gSkipErrStatusCount = 1;
        errStatus.validParams = PMIC_CFG_WD_ANSW_EARLY_ERRSTAT_VALID_SHIFT;
        /* Get watchdog error Status */
        status = Pmic_wdgGetErrorStatus(pPmicCoreHandle, &errStatus);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        gErrStatusCount = 0;
        gSkipErrStatusCount = 1;
        /* Clear WDG Answer Early timeout Error bits */
        status = Pmic_wdgClrErrStatus(pPmicCoreHandle, PMIC_WDG_ERR_ANSWER_EARLY);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        gErrStatusCount = 0;
        gSkipErrStatusCount = 1;
        errStatus.validParams = PMIC_CFG_WD_SEQ_ERR_ERRSTAT_VALID_SHIFT;
        /* Get watchdog error Status */
        status = Pmic_wdgGetErrorStatus(pPmicCoreHandle, &errStatus);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        gErrStatusCount = 0;
        gSkipErrStatusCount = 1;
        /* Clear WDG Sequence timeout Error bits */
        status = Pmic_wdgClrErrStatus(pPmicCoreHandle, PMIC_WDG_ERR_SEQ_ERR);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        gErrStatusCount = 0;
        gSkipErrStatusCount = 1;
        errStatus.validParams = PMIC_CFG_WD_ANSW_ERR_ERRSTAT_VALID_SHIFT;
        /* Get watchdog error Status */
        status = Pmic_wdgGetErrorStatus(pPmicCoreHandle, &errStatus);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        gErrStatusCount = 0;
        gSkipErrStatusCount = 1;
        /* Clear Answer timeout Error bits */
        status = Pmic_wdgClrErrStatus(pPmicCoreHandle, PMIC_WDG_ERR_ANS_ERR);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        gErrStatusCount = 0;
        gSkipErrStatusCount = 1;
        errStatus.validParams = PMIC_CFG_WD_FAIL_INT_ERRSTAT_VALID_SHIFT;
        /* Get watchdog error Status */
        status = Pmic_wdgGetErrorStatus(pPmicCoreHandle, &errStatus);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        gErrStatusCount = 0;
        gSkipErrStatusCount = 1;
        /* Clear WDG Fail timeout Error bits */
        status = Pmic_wdgClrErrStatus(pPmicCoreHandle, PMIC_WDG_ERR_FAIL_INT);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        gErrStatusCount = 0;
        gSkipErrStatusCount = 1;
        errStatus.validParams = PMIC_CFG_WD_RST_INT_ERRSTAT_VALID_SHIFT;
        /* Get watchdog error Status */
        status = Pmic_wdgGetErrorStatus(pPmicCoreHandle, &errStatus);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        gErrStatusCount = 0;
        gSkipErrStatusCount = 1;
        /* Clear WDG Reset timeout Error bits */
        status = Pmic_wdgClrErrStatus(pPmicCoreHandle, PMIC_WDG_ERR_RST_INT);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        gErrorStatusTestFlag = 0U;
        gWdgErrStatusTestFlag = 1U;

        /* Clear WDG Long Window timeout Error bits */
        status = Pmic_wdgClrErrStatus(pPmicCoreHandle, PMIC_WDG_ERR_LONG_WIN_TIMEOUT);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        /* Clear PMIC_WDG_ERR_FAIL_INT - false case of PMIC_WDG_ERR_FAIL_INT second if case */
        status = Pmic_wdgClrErrStatus(pPmicCoreHandle, PMIC_WDG_ERR_FAIL_INT);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        /* Clear WDG Trigger Early timeout Error bits */
        status = Pmic_wdgClrErrStatus(pPmicCoreHandle, PMIC_WDG_ERR_TRIGGER_EARLY);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        /* Clear WDG Answer timeout Error bits */
        status = Pmic_wdgClrErrStatus(pPmicCoreHandle, PMIC_WDG_ERR_ANS_ERR);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        /* Clear WDG Rest ERR timeout Error bits */
        status = Pmic_wdgClrErrStatus(pPmicCoreHandle, PMIC_WDG_ERR_RST_INT);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        gWdgErrStatusTestFlag = 0U;

    }

    Pmic_DevSubSysInfo_t pmicDevSubSysInfo =
    {
        .gpioEnable = (bool)true,
        .rtcEnable  = (bool)true,
        .wdgEnable  = (bool)false,
        .buckEnable = (bool)true,
        .ldoEnable  = (bool)true,
        .esmEnable  = (bool)true
    };

    pPmicCoreHandle->pPmic_SubSysInfo = (&pmicDevSubSysInfo);
    status = Pmic_wdgEnable(pPmicCoreHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_DEVICE, status);

    //Pmic_wdgQaSequenceWriteAnswer
    status = Pmic_wdgQaSequenceWriteAnswer(pPmicCoreHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_DEVICE, status);

    Pmic_DevSubSysInfo_t testpmicDevSubSysInfo =
    {
        .gpioEnable = (bool)true,
        .rtcEnable  = (bool)true,
        .wdgEnable  = (bool)true,
        .buckEnable = (bool)true,
        .ldoEnable  = (bool)true,
        .esmEnable  = (bool)true
    };

    pPmicCoreHandle->pPmic_SubSysInfo = (&testpmicDevSubSysInfo);

    testCommMode = pPmicCoreHandle->commMode;
    pPmicCoreHandle->commMode = 3U;
    status = Pmic_wdgEnable(pPmicCoreHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, status);
    pPmicCoreHandle->commMode = testCommMode;

    /* Disable WDG Timer */
    status = Pmic_wdgDisable(pPmicCoreHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    Osal_delay(1000U);

    pmic_log("\r\n WDG coverage test completed\n");

    pmic_testResultUpdate_pass(8917,
                               pmic_wdg_tests,
                               PMIC_WDG_NUM_OF_TESTCASES);
}

#if defined(UNITY_INCLUDE_CONFIG_V2_H) && \
    (defined(SOC_J721E) || defined(SOC_J7200))

/*!
 * \brief   Run wdg unity test cases
 */
static void test_pmic_run_testcases(void)
{
    pmic_log("\n\n%s(): %d: Begin Unity Test Cases...\n", __func__, __LINE__);
    UNITY_BEGIN();

    pmic_testResult_init(pmic_wdg_tests, PMIC_WDG_NUM_OF_TESTCASES);

    RUN_TEST(test_pmic_wdg_setCfg_forallparams);
    RUN_TEST(test_pmic_wdg_setCfg_prmValTest_handle);
    RUN_TEST(test_pmic_wdg_setCfg_prmValTest_longwinMin);
    RUN_TEST(test_pmic_wdg_setCfg_prmValTest_longwinMax);
    RUN_TEST(test_pmic_wdg_setCfg_prmValTest_win1Min);
    RUN_TEST(test_pmic_wdg_setCfg_prmValTest_win1Max);
    RUN_TEST(test_pmic_wdg_setCfg_prmValTest_win2Min);
    RUN_TEST(test_pmic_wdg_setCfg_prmValTest_win2Max);
    RUN_TEST(test_pmic_wdg_setCfg_prmValTest_failThresholdMax);
    RUN_TEST(test_pmic_wdg_setCfg_prmValTest_rstThresholdMax);
    RUN_TEST(test_pmic_wdg_setCfg_prmValTest_qaFdbkMax);
    RUN_TEST(test_pmic_wdg_setCfg_prmValTest_qaLfsrMax);
    RUN_TEST(test_pmic_wdg_setCfg_prmValTest_qaQuesSeedMax);

    RUN_TEST(test_pmic_wdg_getCfg_forallparams);
    RUN_TEST(test_pmic_wdg_getCfg_prmValTest_handle);
    RUN_TEST(test_pmic_wdg_getCfg_prmValTest_wdgcfgParam);

    RUN_TEST(test_pmic_wdg_enable_prmValTest_handle);
    RUN_TEST(test_pmic_wdg_disable_prmValTest_handle);

    RUN_TEST(test_pmic_wdg_startQaSequence_prmValTest_handle);
    RUN_TEST(test_pmic_wdg_startQaSequence);
    RUN_TEST(test_pmic_wdg_startQaSequence_testFdbkValues);
    RUN_TEST(test_pmic_wdg_startQaSequence_testLfsrValues);
    RUN_TEST(test_pmic_wdg_startQaSequence_testQuesSeedValues);

    RUN_TEST(test_pmic_wdg_GetFailCount);
    RUN_TEST(test_pmic_wdg_GetFailCount_prmValTest_handle);
    RUN_TEST(test_pmic_wdg_GetFailCount_prmValTest_invFailCountParam);

    RUN_TEST(test_pmic_wdg_GetErrorStatus);
    RUN_TEST(test_pmic_wdg_GetErrorStatus_prmValTest_handle);
    RUN_TEST(test_pmic_wdg_GetErrorStatus_prmValTest_invErrStatParam);

#if defined(ENABLE_SAMPLE_TESTCASES)
    RUN_TEST(test_pmic_wdg_StartTriggerSequence);
#endif

    RUN_TEST(test_pmic_wdg_StartTriggerSequence_prmValTest_handle);
    RUN_TEST(test_pmic_wdg_startQaSequence_prmValTest_maxCnt);
    RUN_TEST(test_pmic_wdg_ClrErrorStatus);
    RUN_TEST(test_pmic_wdg_ClrErrorStatus_prmValTest_handle);
    RUN_TEST(test_pmic_wdg_ClrErrorStatus_prmValTest_wdgErrType);

    RUN_TEST(test_pmic_wdgQaSequenceWriteAnswer_startQaSequence);
    RUN_TEST(test_pmic_wdgQaSeqWriteAns_testFdbkValues);
    RUN_TEST(test_pmic_wdgQaSeqWriteAns_testLfsrValues);
    RUN_TEST(test_pmic_wdgQaSeqWriteAns_testSeedValues);

    RUN_TEST(test_pmic_wdgsetCfg_longwin);
    RUN_TEST(test_pmic_wdgsetCfg_prmValTest_longwinMin_2999ms);
    RUN_TEST(test_pmic_wdg_StartTriggerSequence);

    RUN_TEST(test_pmic_wdgClrErrStatus_wdgErrType);
    RUN_TEST(test_pmic_wdg_startQaSequence_answerEarlyErr);
    pmic_updateTestResults(pmic_wdg_tests, PMIC_WDG_NUM_OF_TESTCASES);

    UNITY_END();
}

#if defined(SOC_J7200)
/*!
 * \brief   Run wdg unity test cases for slave device
 */
static void test_pmic_run_slave_device_testcases(void)
{
    pmic_log("\n\n%s(): %d: Begin Unity Test Cases...\n", __func__, __LINE__);
    UNITY_BEGIN();

    pmic_testResult_init(pmic_wdg_tests, PMIC_WDG_NUM_OF_TESTCASES);

    RUN_TEST(test_pmic_wdg_setCfg_forallparams);
    RUN_TEST(test_pmic_wdg_setCfg_prmValTest_handle);
    RUN_TEST(test_pmic_wdg_setCfg_prmValTest_longwinMin);
    RUN_TEST(test_pmic_wdg_setCfg_prmValTest_longwinMax);
    RUN_TEST(test_pmic_wdg_setCfg_prmValTest_win1Min);
    RUN_TEST(test_pmic_wdg_setCfg_prmValTest_win1Max);
    RUN_TEST(test_pmic_wdg_setCfg_prmValTest_win2Min);
    RUN_TEST(test_pmic_wdg_setCfg_prmValTest_win2Max);
    RUN_TEST(test_pmic_wdg_setCfg_prmValTest_failThresholdMax);
    RUN_TEST(test_pmic_wdg_setCfg_prmValTest_rstThresholdMax);
    RUN_TEST(test_pmic_wdg_setCfg_prmValTest_qaFdbkMax);
    RUN_TEST(test_pmic_wdg_setCfg_prmValTest_qaLfsrMax);
    RUN_TEST(test_pmic_wdg_setCfg_prmValTest_qaQuesSeedMax);

    RUN_TEST(test_pmic_wdg_getCfg_forallparams);
    RUN_TEST(test_pmic_wdg_getCfg_prmValTest_handle);
    RUN_TEST(test_pmic_wdg_getCfg_prmValTest_wdgcfgParam);

    RUN_TEST(test_pmic_wdg_enable_prmValTest_handle);
    RUN_TEST(test_pmic_wdg_disable_prmValTest_handle);

    RUN_TEST(test_pmic_wdg_startQaSequence_prmValTest_handle);

    RUN_TEST(test_pmic_wdg_GetFailCount);
    RUN_TEST(test_pmic_wdg_GetFailCount_prmValTest_handle);
    RUN_TEST(test_pmic_wdg_GetFailCount_prmValTest_invFailCountParam);

    RUN_TEST(test_pmic_wdg_GetErrorStatus);
    RUN_TEST(test_pmic_wdg_GetErrorStatus_prmValTest_handle);
    RUN_TEST(test_pmic_wdg_GetErrorStatus_prmValTest_invErrStatParam);

    RUN_TEST(test_pmic_wdg_StartTriggerSequence_prmValTest_handle);
    RUN_TEST(test_pmic_wdg_startQaSequence_prmValTest_maxCnt);
    RUN_TEST(test_pmic_wdg_ClrErrorStatus_prmValTest_handle);
    RUN_TEST(test_pmic_wdg_ClrErrorStatus_prmValTest_wdgErrType);

    pmic_updateTestResults(pmic_wdg_tests, PMIC_WDG_NUM_OF_TESTCASES);

    UNITY_END();
}
#endif

/*!
 * \brief   WDG Unity Test App wrapper Function for LEO PMIC-A
 */
static int32_t test_pmic_leo_pmicA_wdg_testApp(void)
{
    int32_t status                = PMIC_ST_SUCCESS;
    Pmic_CoreCfg_t pmicConfigData = {0U};

    /* Fill parameters to pmicConfigData */
    pmicConfigData.pmicDeviceType      = PMIC_DEV_LEO_TPS6594X;
    pmicConfigData.validParams        |= PMIC_CFG_DEVICE_TYPE_VALID_SHIFT;

    pmicConfigData.commMode            = PMIC_INTF_DUAL_I2C;
    pmicConfigData.validParams        |= PMIC_CFG_COMM_MODE_VALID_SHIFT;

    pmicConfigData.i2c1Speed            = PMIC_I2C_STANDARD_MODE;
    pmicConfigData.validParams         |= PMIC_CFG_I2C1_SPEED_VALID_SHIFT;

    pmicConfigData.i2c2Speed            = PMIC_I2C_STANDARD_MODE;
    pmicConfigData.validParams         |= PMIC_CFG_I2C2_SPEED_VALID_SHIFT;

    if(J721E_LEO_PMICA_DEVICE == pmic_device_info)
    {
        pmicConfigData.slaveAddr           = J721E_LEO_PMICA_SLAVE_ADDR;
        pmicConfigData.validParams        |= PMIC_CFG_SLAVEADDR_VALID_SHIFT;

        pmicConfigData.qaSlaveAddr         = J721E_LEO_PMICA_WDG_SLAVE_ADDR;
        pmicConfigData.validParams        |= PMIC_CFG_QASLAVEADDR_VALID_SHIFT;

        pmicConfigData.nvmSlaveAddr        = J721E_LEO_PMICA_PAGE1_SLAVE_ADDR;
        pmicConfigData.validParams        |= PMIC_CFG_NVMSLAVEADDR_VALID_SHIFT;

    }
    if(J7VCL_LEO_PMICA_DEVICE == pmic_device_info)
    {
        pmicConfigData.slaveAddr           = J7VCL_LEO_PMICA_SLAVE_ADDR;
        pmicConfigData.validParams        |= PMIC_CFG_SLAVEADDR_VALID_SHIFT;

        pmicConfigData.qaSlaveAddr         = J7VCL_LEO_PMICA_WDG_SLAVE_ADDR;
        pmicConfigData.validParams        |= PMIC_CFG_QASLAVEADDR_VALID_SHIFT;

        pmicConfigData.nvmSlaveAddr        = J7VCL_LEO_PMICA_PAGE1_SLAVE_ADDR;
        pmicConfigData.validParams        |= PMIC_CFG_NVMSLAVEADDR_VALID_SHIFT;

    }

    pmicConfigData.pFnPmicCommIoRead    = test_pmic_regRead;
    pmicConfigData.validParams         |= PMIC_CFG_COMM_IO_RD_VALID_SHIFT;

    pmicConfigData.pFnPmicCommIoWrite   = test_pmic_regWrite;
    pmicConfigData.validParams         |= PMIC_CFG_COMM_IO_WR_VALID_SHIFT;

    pmicConfigData.pFnPmicCritSecStart  = test_pmic_criticalSectionStartFn;
    pmicConfigData.validParams         |= PMIC_CFG_CRITSEC_START_VALID_SHIFT;

    pmicConfigData.pFnPmicCritSecStop   = test_pmic_criticalSectionStopFn;
    pmicConfigData.validParams         |= PMIC_CFG_CRITSEC_STOP_VALID_SHIFT;

    status = test_pmic_appInit(&pPmicCoreHandle, &pmicConfigData);
    return status;
}

#if defined(SOC_J721E)
/*!
 * \brief   WDG Unity Test App wrapper Function for LEO PMIC-A Single I2C
 */
static int32_t test_pmic_leo_pmicA_wdg_single_i2c_testApp(void)
{
    int32_t status                = PMIC_ST_SUCCESS;
    Pmic_CoreCfg_t pmicConfigData = {0U};

    /* Fill parameters to pmicConfigData */
    pmicConfigData.pmicDeviceType      = PMIC_DEV_LEO_TPS6594X;
    pmicConfigData.validParams        |= PMIC_CFG_DEVICE_TYPE_VALID_SHIFT;

    pmicConfigData.commMode            = PMIC_INTF_SINGLE_I2C;
    pmicConfigData.validParams        |= PMIC_CFG_COMM_MODE_VALID_SHIFT;

    pmicConfigData.i2c1Speed            = PMIC_I2C_STANDARD_MODE;
    pmicConfigData.validParams         |= PMIC_CFG_I2C1_SPEED_VALID_SHIFT;

    pmicConfigData.slaveAddr           = J721E_LEO_PMICA_SLAVE_ADDR;
    pmicConfigData.validParams        |= PMIC_CFG_SLAVEADDR_VALID_SHIFT;

    pmicConfigData.qaSlaveAddr         = J721E_LEO_PMICA_WDG_SLAVE_ADDR;
    pmicConfigData.validParams        |= PMIC_CFG_QASLAVEADDR_VALID_SHIFT;

    pmicConfigData.nvmSlaveAddr        = J721E_LEO_PMICA_PAGE1_SLAVE_ADDR;
    pmicConfigData.validParams        |= PMIC_CFG_NVMSLAVEADDR_VALID_SHIFT;

    pmicConfigData.pFnPmicCommIoRead    = test_pmic_regRead;
    pmicConfigData.validParams         |= PMIC_CFG_COMM_IO_RD_VALID_SHIFT;

    pmicConfigData.pFnPmicCommIoWrite   = test_pmic_regWrite;
    pmicConfigData.validParams         |= PMIC_CFG_COMM_IO_WR_VALID_SHIFT;

    pmicConfigData.pFnPmicCritSecStart  = test_pmic_criticalSectionStartFn;
    pmicConfigData.validParams         |= PMIC_CFG_CRITSEC_START_VALID_SHIFT;

    pmicConfigData.pFnPmicCritSecStop   = test_pmic_criticalSectionStopFn;
    pmicConfigData.validParams         |= PMIC_CFG_CRITSEC_STOP_VALID_SHIFT;

    status = test_pmic_appInit(&pPmicCoreHandle, &pmicConfigData);
    return status;

}
#endif

/*!
 * \brief  WDG  Unity Test App wrapper Function for HERA PMIC
 */
static int32_t test_pmic_hera_wdg_testApp(void)
{
    int32_t status                = PMIC_ST_SUCCESS;
    Pmic_CoreCfg_t pmicConfigData = {0U};

    /* Fill parameters to pmicConfigData */
    pmicConfigData.pmicDeviceType      = PMIC_DEV_HERA_LP8764X;
    pmicConfigData.validParams        |= PMIC_CFG_DEVICE_TYPE_VALID_SHIFT;

    pmicConfigData.commMode            = PMIC_INTF_SINGLE_I2C;
    pmicConfigData.validParams        |= PMIC_CFG_COMM_MODE_VALID_SHIFT;

    pmicConfigData.i2c1Speed            = PMIC_I2C_STANDARD_MODE;
    pmicConfigData.validParams         |= PMIC_CFG_I2C1_SPEED_VALID_SHIFT;

    pmicConfigData.slaveAddr           = J7VCL_HERA_PMIC_SLAVE_ADDR;
    pmicConfigData.validParams        |= PMIC_CFG_SLAVEADDR_VALID_SHIFT;

    pmicConfigData.qaSlaveAddr         = J7VCL_HERA_PMIC_WDG_SLAVE_ADDR;
    pmicConfigData.validParams        |= PMIC_CFG_QASLAVEADDR_VALID_SHIFT;

    pmicConfigData.nvmSlaveAddr        = J7VCL_HERA_PMIC_PAGE1_SLAVE_ADDR;
    pmicConfigData.validParams        |= PMIC_CFG_NVMSLAVEADDR_VALID_SHIFT;

    pmicConfigData.pFnPmicCommIoRead    = test_pmic_regRead;
    pmicConfigData.validParams         |= PMIC_CFG_COMM_IO_RD_VALID_SHIFT;

    pmicConfigData.pFnPmicCommIoWrite   = test_pmic_regWrite;
    pmicConfigData.validParams         |= PMIC_CFG_COMM_IO_WR_VALID_SHIFT;

    pmicConfigData.pFnPmicCritSecStart  = test_pmic_criticalSectionStartFn;
    pmicConfigData.validParams         |= PMIC_CFG_CRITSEC_START_VALID_SHIFT;

    pmicConfigData.pFnPmicCritSecStop   = test_pmic_criticalSectionStopFn;
    pmicConfigData.validParams         |= PMIC_CFG_CRITSEC_STOP_VALID_SHIFT;

    status = test_pmic_appInit(&pPmicCoreHandle, &pmicConfigData);
    return status;

}

/*!
 * \brief   RTC Unity Test App wrapper Function for LEO PMIC-B
 */
static int32_t test_pmic_leo_pmicB_wdg_testApp(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_CoreCfg_t pmicConfigData = {0U};

    /* Fill parameters to pmicConfigData */
    pmicConfigData.pmicDeviceType     = PMIC_DEV_LEO_TPS6594X;
    pmicConfigData.validParams        |= PMIC_CFG_DEVICE_TYPE_VALID_SHIFT;

    pmicConfigData.commMode           = PMIC_INTF_SINGLE_I2C;
    pmicConfigData.validParams        |= PMIC_CFG_COMM_MODE_VALID_SHIFT;

    pmicConfigData.i2c1Speed            = PMIC_I2C_STANDARD_MODE;
    pmicConfigData.validParams         |= PMIC_CFG_I2C1_SPEED_VALID_SHIFT;

    pmicConfigData.slaveAddr          = J721E_LEO_PMICB_SLAVE_ADDR;
    pmicConfigData.validParams        |= PMIC_CFG_SLAVEADDR_VALID_SHIFT;

    pmicConfigData.qaSlaveAddr        = J721E_LEO_PMICB_WDG_SLAVE_ADDR;
    pmicConfigData.validParams        |= PMIC_CFG_QASLAVEADDR_VALID_SHIFT;

    pmicConfigData.nvmSlaveAddr        = J721E_LEO_PMICB_PAGE1_SLAVE_ADDR;
    pmicConfigData.validParams        |= PMIC_CFG_NVMSLAVEADDR_VALID_SHIFT;

    pmicConfigData.pFnPmicCommIoRead   = test_pmic_regRead;
    pmicConfigData.validParams         |= PMIC_CFG_COMM_IO_RD_VALID_SHIFT;

    pmicConfigData.pFnPmicCommIoWrite  = test_pmic_regWrite;
    pmicConfigData.validParams         |= PMIC_CFG_COMM_IO_WR_VALID_SHIFT;

    pmicConfigData.pFnPmicCritSecStart = test_pmic_criticalSectionStartFn;
    pmicConfigData.validParams         |= PMIC_CFG_CRITSEC_START_VALID_SHIFT;

    pmicConfigData.pFnPmicCritSecStop  = test_pmic_criticalSectionStopFn;
    pmicConfigData.validParams         |= PMIC_CFG_CRITSEC_STOP_VALID_SHIFT;

    status = test_pmic_appInit(&pPmicCoreHandle, &pmicConfigData);
    return status;

}

/*!
 * \brief   WDG Unity Test App wrapper Function for LEO PMIC-A
 */
static int32_t test_pmic_leo_pmicA_spiStub_wdg_testApp(void)
{
    int32_t status                = PMIC_ST_SUCCESS;
    Pmic_CoreCfg_t pmicConfigData = {0U};

    /* Fill parameters to pmicConfigData */
    pmicConfigData.pmicDeviceType      = PMIC_DEV_LEO_TPS6594X;
    pmicConfigData.validParams        |= PMIC_CFG_DEVICE_TYPE_VALID_SHIFT;

    pmicConfigData.commMode            = PMIC_INTF_SPI;
    pmicConfigData.validParams        |= PMIC_CFG_COMM_MODE_VALID_SHIFT;

    pmicConfigData.pFnPmicCommIoRead    = test_pmic_regRead;
    pmicConfigData.validParams         |= PMIC_CFG_COMM_IO_RD_VALID_SHIFT;

    pmicConfigData.pFnPmicCommIoWrite   = test_pmic_regWrite;
    pmicConfigData.validParams         |= PMIC_CFG_COMM_IO_WR_VALID_SHIFT;

    pmicConfigData.pFnPmicCritSecStart  = test_pmic_criticalSectionStartFn;
    pmicConfigData.validParams         |= PMIC_CFG_CRITSEC_START_VALID_SHIFT;

    pmicConfigData.pFnPmicCritSecStop   = test_pmic_criticalSectionStopFn;
    pmicConfigData.validParams         |= PMIC_CFG_CRITSEC_STOP_VALID_SHIFT;

    status = test_pmic_appInit(&pPmicCoreHandle, &pmicConfigData);
    return status;
}

static int32_t setup_pmic_interrupt(uint32_t board)
{
    int32_t status = PMIC_ST_SUCCESS;

    if(J721E_BOARD == board)
    {
        if(PMIC_STATUS_CRC_INIT_VAL == gCrcTestFlag_J721E)
        {
            gCrcTestFlag_J721E = PMIC_CFG_TO_ENABLE_CRC;
        }

        pmic_device_info = J721E_LEO_PMICA_DEVICE;
        status = test_pmic_leo_pmicA_wdg_testApp();
        /* Deinit pmic handle */
        if((pPmicCoreHandle != NULL) && (PMIC_ST_SUCCESS == status))
        {
            test_pmic_appDeInit(pPmicCoreHandle);
        }

        if(PMIC_ST_SUCCESS == status)
        {
            pmic_device_info = J721E_LEO_PMICB_DEVICE;
            status = test_pmic_leo_pmicB_wdg_testApp();
            /* Deinit pmic handle */
            if((pPmicCoreHandle != NULL) && (PMIC_ST_SUCCESS == status))
            {
                test_pmic_appDeInit(pPmicCoreHandle);
            }
        }
    }
    else if(J7VCL_BOARD == board)
    {
        if(PMIC_STATUS_CRC_INIT_VAL == gCrcTestFlag_J7VCL)
        {
            gCrcTestFlag_J7VCL = PMIC_CFG_TO_ENABLE_CRC;
        }

        pmic_device_info = J7VCL_LEO_PMICA_DEVICE;
        status = test_pmic_leo_pmicA_wdg_testApp();
        /* Deinit pmic handle */
        if((pPmicCoreHandle != NULL) && (PMIC_ST_SUCCESS == status))
        {
            test_pmic_appDeInit(pPmicCoreHandle);
        }

        if(PMIC_ST_SUCCESS == status)
        {
            pmic_device_info = J7VCL_HERA_PMICB_DEVICE;
            status = test_pmic_hera_wdg_testApp();
            /* Deinit pmic handle */
            if((pPmicCoreHandle != NULL) && (PMIC_ST_SUCCESS == status))
            {
                test_pmic_appDeInit(pPmicCoreHandle);
            }
        }
    }
    else
    {
        status = PMIC_ST_ERR_INV_DEVICE;
    }
    return status;
}

static const char pmicTestMenu[] =
{
    " \r\n ================================================================="
    " \r\n Test Menu:"
    " \r\n ================================================================="
    " \r\n 0: Automatic run for all board specific WDG options"
    " \r\n 1: Manual run for WDG options"
    " \r\n 2: quit"
    " \r\n"
    " \r\n Enter option: "
};

volatile static const char pmicTestAppMenu[] =
{
    " \r\n ================================================================="
    " \r\n Menu:"
    " \r\n ================================================================="
    " \r\n 0: Pmic Leo device with Dual I2C(PMIC-A on J721E EVM)"
    " \r\n 1: Pmic Leo device with Single I2C(PMIC-A on J721E EVM)"
    " \r\n 2: Pmic Leo device with Dual I2C(PMIC-A on J7VCL EVM)"
    " \r\n 3: Pmic Hera device with Single I2C(PMIC-B on J7VCL EVM)"
    " \r\n 4: Pmic Leo device with SPI Stub Functions(PMIC-A on J721E EVM)"
    " \r\n 5: Pmic Leo device with SPI Stub Functions(PMIC-A on J7VCL EVM)"
    " \r\n 6: Pmic Leo Manual test for WDG RST INT and WDG LONGWIN TIMEOUT INT(PMIC-A on J721E EVM)"
    " \r\n 7: Pmic Leo Manual test for WDG RST INT and WDG LONGWIN TIMEOUT INT(PMIC-A on J7VCL EVM)"
    " \r\n 8: Pmic Leo Manual test for WDG Dynamic code coverage(PMIC-A on J721E EVM)"
    " \r\n 9: Pmic Leo Manual test for WDG Dynamic code coverage(PMIC-A on J7VCL EVM)"
    " \r\n 10: Back to Test Menu"
    " \r\n"
    " \r\n Enter option: "
};


static void test_pmic_wdg_testapp_run_options(int8_t option)
{
    int8_t num = -1;
    int8_t idx = 0;
#if defined(SOC_J721E)
    int8_t automatic_options[] = {0, 1, 4};
#elif defined(SOC_J7200)
    int8_t automatic_options[] = {2, 3, 5};
#endif

    while(1U)
    {
        if(idx >= (sizeof(automatic_options)/sizeof(automatic_options[0])))
        {
            pmic_printTestResult(pmic_wdg_tests, PMIC_WDG_NUM_OF_TESTCASES);
        }
        pmic_log("%s", pmicTestAppMenu);
        if(option == PMIC_UT_AUTOMATE_OPTION)
        {
            if(idx < (sizeof(automatic_options)/sizeof(automatic_options[0])))
            {
                num = automatic_options[idx++];
            }
            else
            {
                num = 10;
            }
            pmic_log("%d\n", num);
        }
        else
        {
            if(UART_scanFmt("%d", &num) != 0U)
            {
                pmic_log("Read from UART Console failed\n");
                return;
            }
        }

        switch(num)
        {
            case 0U:
#if defined(SOC_J721E)
                if(PMIC_ST_SUCCESS == setup_pmic_interrupt(J721E_BOARD))
                {
                    pmic_device_info = J721E_LEO_PMICA_DEVICE;

                    /* WDG Unity Test App wrapper Function for LEO PMIC-A */
                    if(PMIC_ST_SUCCESS == test_pmic_leo_pmicA_wdg_testApp())
                    {
                        /* Run WDG test cases for Leo PMIC-A */
                        test_pmic_run_testcases();
                    }
                    /* Deinit pmic handle */
                    if(pPmicCoreHandle != NULL)
                    {
                        test_pmic_appDeInit(pPmicCoreHandle);
                    }
                }
#else
                pmic_log("\nInvalid Board!!!\n");
#endif
                break;
            case 1U:
#if defined(SOC_J721E)
                if(PMIC_ST_SUCCESS == setup_pmic_interrupt(J721E_BOARD))
                {
                    pmic_device_info = J721E_LEO_PMICA_DEVICE;

                    /* WDG Unity Test App wrapper Function for LEO PMIC-A */
                    if(PMIC_ST_SUCCESS == test_pmic_leo_pmicA_wdg_single_i2c_testApp())
                    {
                        /* Run WDG test cases for Leo PMIC-A using Single I2C */
                        test_pmic_run_testcases();
                    }
                    /* Deinit pmic handle */
                    if(pPmicCoreHandle != NULL)
                    {
                        test_pmic_appDeInit(pPmicCoreHandle);
                    }
                }
#else
                pmic_log("\nInvalid Board!!!\n");
#endif
                break;
            case 2U:
#if defined(SOC_J7200)
                if(PMIC_ST_SUCCESS == setup_pmic_interrupt(J7VCL_BOARD))
                {
                    pmic_device_info = J7VCL_LEO_PMICA_DEVICE;

                    /* WDG Unity Test App wrapper Function for LEO PMIC-A */
                    if(PMIC_ST_SUCCESS == test_pmic_leo_pmicA_wdg_testApp())
                    {
                        /* Run WDG test cases for Leo PMIC-A */
                        test_pmic_run_testcases();
                    }
                    /* Deinit pmic handle */
                    if(pPmicCoreHandle != NULL)
                    {
                        test_pmic_appDeInit(pPmicCoreHandle);
                    }
                }
#else
                pmic_log("\nInvalid Board!!!\n");
#endif
                break;
            case 3U:
#if defined(SOC_J7200)
                if(PMIC_ST_SUCCESS == setup_pmic_interrupt(J7VCL_BOARD))
                {
                    pmic_device_info = J7VCL_HERA_PMICB_DEVICE;

                    /* WDG Unity Test App wrapper Function for HERA PMIC */
                    if(PMIC_ST_SUCCESS == test_pmic_hera_wdg_testApp())
                    {
                        /* Run wdg test cases for Hera PMIC as slave device */
                        test_pmic_run_slave_device_testcases();
                    }
                    /* Deinit pmic handle */
                    if(pPmicCoreHandle != NULL)
                    {
                        test_pmic_appDeInit(pPmicCoreHandle);
                    }
                }
#else
                pmic_log("\nInvalid Board!!!\n");
#endif
                break;
            case 4U:
#if defined(SOC_J721E)
                if(PMIC_ST_SUCCESS == setup_pmic_interrupt(J721E_BOARD))
                {
                     pmic_device_info = J721E_LEO_PMICA_DEVICE;

                    /* WDG Unity Test App wrapper Function for LEO PMIC-A
                     * using SPI stub functions */
                    if(PMIC_ST_SUCCESS ==
                          test_pmic_leo_pmicA_spiStub_wdg_testApp())
                    {
                        /* Run wdg test cases for Leo PMIC-A */
                        test_pmic_run_testcases();
                    }
                    /* Deinit pmic handle */
                    if(pPmicCoreHandle != NULL)
                    {
                        test_pmic_appDeInit(pPmicCoreHandle);
                    }
                }
#else
                pmic_log("\nInvalid Board!!!\n");
#endif
                break;
            case 5U:
#if defined(SOC_J7200)
                if(PMIC_ST_SUCCESS == setup_pmic_interrupt(J7VCL_BOARD))
                {
                    pmic_device_info = J7VCL_LEO_PMICA_DEVICE;
                    /* WDG Unity Test App wrapper Function for LEO PMIC-A
                     * using SPI stub functions */
                     if(PMIC_ST_SUCCESS ==
                            test_pmic_leo_pmicA_spiStub_wdg_testApp())
                    {
                        /* Run wdg test cases for Leo PMIC-A */
                        test_pmic_run_testcases();
                    }
                    /* Deinit pmic handle */
                    if(pPmicCoreHandle != NULL)
                    {
                        test_pmic_appDeInit(pPmicCoreHandle);
                    }
                }
#else
                pmic_log("\nInvalid Board!!!\n");
#endif
                break;
            case 6U:
#if defined(SOC_J721E)
                if(PMIC_ST_SUCCESS == setup_pmic_interrupt(J721E_BOARD))
                {
                    pmic_device_info = J721E_LEO_PMICA_DEVICE;

                    /* WDG Unity Test App wrapper Function for LEO PMIC-A */
                    if(PMIC_ST_SUCCESS == test_pmic_leo_pmicA_wdg_single_i2c_testApp())
                    {
                        /* Run WDG test cases for Leo PMIC-A using Single I2C */
                        test_pmic_wdgQaSequenceWriteAnswer_RstIntLongWinTimeoutInt();
                    }
                    /* Deinit pmic handle */
                    if(pPmicCoreHandle != NULL)
                    {
                        test_pmic_appDeInit(pPmicCoreHandle);
                    }
                }
#else
                pmic_log("\nInvalid Board!!!\n");
#endif
                break;
            case 7U:
#if defined(SOC_J7200)
                if(PMIC_ST_SUCCESS == setup_pmic_interrupt(J7VCL_BOARD))
                {
                    pmic_device_info = J7VCL_LEO_PMICA_DEVICE;

                    /* WDG Unity Test App wrapper Function for LEO PMIC-A */
                    if(PMIC_ST_SUCCESS == test_pmic_leo_pmicA_wdg_testApp())
                    {
                        /* Run WDG test cases for Leo PMIC-A */
                        test_pmic_wdgQaSequenceWriteAnswer_RstIntLongWinTimeoutInt();
                    }
                    /* Deinit pmic handle */
                    if(pPmicCoreHandle != NULL)
                    {
                        test_pmic_appDeInit(pPmicCoreHandle);
                    }
                }
#else
                pmic_log("\nInvalid Board!!!\n");
#endif
                break;
            case 8U:
#if defined(SOC_J721E)
                if(PMIC_ST_SUCCESS == setup_pmic_interrupt(J721E_BOARD))
                {
                    pmic_device_info = J721E_LEO_PMICA_DEVICE;

                    /* WDG Unity Test App wrapper Function for LEO PMIC-A */
                    if(PMIC_ST_SUCCESS == test_pmic_leo_pmicA_wdg_testApp())
                    {
                        /* Run WDG test cases for Leo PMIC-A using Single I2C */
                        test_pmic_wdg_coverageGaps();
                    }
                    /* Deinit pmic handle */
                    if(pPmicCoreHandle != NULL)
                    {
                        test_pmic_appDeInit(pPmicCoreHandle);
                    }
                }
#else
                pmic_log("\nInvalid Board!!!\n");
#endif
                return;
            case 9U:
#if defined(SOC_J7200)
                if(PMIC_ST_SUCCESS == setup_pmic_interrupt(J7VCL_BOARD))
                {
                    pmic_device_info = J7VCL_LEO_PMICA_DEVICE;

                    /* WDG Unity Test App wrapper Function for LEO PMIC-A */
                    if(PMIC_ST_SUCCESS == test_pmic_leo_pmicA_wdg_testApp())
                    {
                        /* Run WDG test cases for Leo PMIC-A */
                        test_pmic_wdg_coverageGaps();
                    }
                    /* Deinit pmic handle */
                    if(pPmicCoreHandle != NULL)
                    {
                        test_pmic_appDeInit(pPmicCoreHandle);
                    }
                }
#else
                pmic_log("\nInvalid Board!!!\n");
#endif
                return;
            case 10U:
               pmic_log(" \r\n Back to Test Menu options\n");
               return;
            default:
               pmic_log(" \r\n Invalid option... Try Again!!!\n");
               break;
        }
    }
}


/*!
 * \brief   Function to register WDG Unity Test App wrapper to Unity framework
 */
static void test_pmic_wdg_testapp_runner(void)
{
    /* @description : Test runner for wdg Test App
     *
     * @requirements: PDK-5813, PDK-5810, PDK-5805, PDK-5807, PDK-5854, PDK-5839
     *
     * @cores       : mcu1_0, mcu1_1
     */

    int8_t option = -1;

    while(1U)
    {
        pmic_log("%s", pmicTestMenu);
        if(UART_scanFmt("%d", &option) != 0U)
        {
            pmic_log("Read from UART Console failed\n");
            return;
        }

        switch(option)
        {
            case PMIC_UT_AUTOMATE_OPTION:
                test_pmic_wdg_testapp_run_options(PMIC_UT_AUTOMATE_OPTION);
               break;
            case PMIC_UT_MANUAL_OPTION:
                test_pmic_wdg_testapp_run_options(PMIC_UT_MANUAL_OPTION);
               break;
            case 2U:
                pmic_log(" \r\n Quit from application\n");
                return;
            default:
               pmic_log(" \r\n Invalid option... Try Again!!!\n");
               break;
        }
    }
}
#endif

/*!
 * \brief   TI RTOS specific WDG TEST APP main Function
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code.
 *          For valid values see \ref Pmic_ErrorCodes
 */
int main()
{
    Board_initUART();

    pmic_print_banner("PMIC Watchdog Unity Test Application");

#if defined(UNITY_INCLUDE_CONFIG_V2_H) && \
    (defined(SOC_J721E)             || \
     defined(SOC_J7200))
    test_pmic_wdg_testapp_runner();
#endif
}
