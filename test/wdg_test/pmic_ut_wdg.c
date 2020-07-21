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

/*!
 * \brief   PMIC WDG Test Cases
 */
static Pmic_Ut_Tests_t pmic_wdg_tests[] =
{
    /*! testID
     *  testDesc
     */
    {
        1001,
        "Pmic_wdgSetCfg : configure all watchdog parameters"
    },
    {
        1002,
        "Pmic_wdgSetCfg : Parameter validation for 'handle'"
    },
    {
        1003,
        "Pmic_wdgSetCfg : Parameter validation for `longWindowi_ms' min value"
    },
    {
        1004,
        "Pmic_wdgSetCfg : Parameter validation for `longWindow_ms' max value"
    },
    {
        1005,
        "Pmic_wdgSetCfg : Parameter validation for `win1Duration_us' min value"
    },
    {
        1006,
        "Pmic_wdgSetCfg : Parameter validation for `win1Duration_us' max value"
    },
    {
        1007,
        "Pmic_wdgSetCfg : Parameter validation for `win2Duration_us' min value"
    },
    {
        1008,
        "Pmic_wdgSetCfg : Parameter validation for `win2Duration_us' max value"
    },
    {
        1009,
        "Pmic_wdgSetCfg : Parameter validation for `win2Duration_us' max value"
    },
    {
        1010,
        "Pmic_wdgSetCfg : Parameter validation for `win2Duration_us' max Value"
    },
    {
        1011,
        "Pmic_wdgSetCfg : Parameter validation for `qaFdbk' max value"
    },
    {
        1012,
        "Pmic_wdgSetCfg : Parameter validation for `qaLfsr' max value"
    },
    {
        1013,
        "Pmic_wdgSetCfg : Parameter validation for `qaQuesSeed' max Value"
    },

    {
        1014,
        "Pmic_wdgGetCfg : Get all watchdog parameters"
    },
    {
        1015,
        "Pmic_wdgGetCfg : Parameter validation for 'handle'"
    },
    {
        1016,
        "Pmic_wdgGetCfg : Parameter validation for 'WdgCfg'"
    },
    {
        1017,
        "Pmic_wdgEnable : Parameter validation for 'handle'"
    },
    {
        1018,
        "Pmic_wdgDisable : Parameter validation for 'WdgCfg'"
    },

    {
        1019,
        "Pmic_wdgStartQaSequence : Parameter validation for 'handle'"
    },
    {
        1020,
        "Pmic_wdgStartQaSequence : Test wdg QA sequences"
    },
    {
        1021,
        "Pmic_wdgStartQaSequence : Test wdg QA sequences with different QA feedback values"
    },
    {
        1022,
        "Pmic_wdgStartQaSequence : Test wdg QA sequences with different QA LFSR values"
    },
    {
        1023,
        "Pmic_wdgStartQaSequence : Test wdg QA sequences with different QA Question Seed values"
    },
    {
        1024,
        "Pmic_wdgGetFailCount : Test get wdg failcount"
    },
    {
        1025,
        "Pmic_wdgGetFailCount : Parameter validation for 'handle'"
    },
    {
        1026,
        "Pmic_wdgGetFailCount : Parameter validation for 'failCount'"
    },

    {
        1027,
        "Pmic_wdgGetErrorStatus: Get all watchdog error status"
    },
    {
        1028,
        "Pmic_wdgGetErrorStatus : Parameter validation for 'handle'"
    },
    {
        1029,
        "Pmic_wdgGetErrorStatus : Parameter validation for 'errStatus'"
    },
    {
        1030,
        "Pmic_wdgStartTriggerSequence : Test wdg trigger sequence"
    },
    {
        1031,
        "Pmic_wdgStartTriggerSequence : Parameter validation for 'handle'"
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
        PMIC_WDG_PWRHOLD_DISABLE,
        PMIC_WDG_RESET_ENABLE,
        PMIC_WDG_RETLONGWIN_ENABLE,
        PMIC_WDG_QA_FEEDBACK_VALUE_2,
        PMIC_WDG_QA_LFSR_VALUE_1,
        PMIC_WDG_QA_QUES_SEED_VALUE_5,
    };

    test_pmic_print_unity_testcase_info(1001U,
                                        pmic_wdg_tests,
                                        PMIC_WDG_NUM_OF_TESTCASES);

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
}

/*!
 * \brief   Parameter validation for 'handle'
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
        PMIC_WDG_RESET_ENABLE,
        PMIC_WDG_RETLONGWIN_ENABLE,
        PMIC_WDG_QA_FEEDBACK_VALUE_2,
        PMIC_WDG_QA_LFSR_VALUE_1,
        PMIC_WDG_QA_QUES_SEED_VALUE_5,
    };

    test_pmic_print_unity_testcase_info(1002U,
                                        pmic_wdg_tests,
                                        PMIC_WDG_NUM_OF_TESTCASES);

    pmicStatus = Pmic_wdgSetCfg(NULL, wdgCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, pmicStatus);
}

/*!
 * \brief   Parameter validation for 'longWinDuration_ms' min Value
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
        PMIC_WDG_RESET_ENABLE,
        PMIC_WDG_RETLONGWIN_ENABLE,
        PMIC_WDG_QA_FEEDBACK_VALUE_2,
        PMIC_WDG_QA_LFSR_VALUE_1,
        PMIC_WDG_QA_QUES_SEED_VALUE_5,
    };

    test_pmic_print_unity_testcase_info(1003U,
                                        pmic_wdg_tests,
                                        PMIC_WDG_NUM_OF_TESTCASES);

    pmicStatus = Pmic_wdgSetCfg(pPmicCoreHandle, wdgCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_WDG_WINDOW, pmicStatus);
}

/*!
 * \brief   Parameter validation for 'longWinDuration_ms' max Value
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
        PMIC_WDG_RESET_ENABLE,
        PMIC_WDG_RETLONGWIN_ENABLE,
        PMIC_WDG_QA_FEEDBACK_VALUE_2,
        PMIC_WDG_QA_LFSR_VALUE_1,
        PMIC_WDG_QA_QUES_SEED_VALUE_5,
    };

    test_pmic_print_unity_testcase_info(1004U,
                                        pmic_wdg_tests,
                                        PMIC_WDG_NUM_OF_TESTCASES);

    pmicStatus = Pmic_wdgSetCfg(pPmicCoreHandle, wdgCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_WDG_WINDOW, pmicStatus);
}

/*!
 * \brief   Parameter validation for 'win1Duration_us' min Value
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
        PMIC_WDG_RESET_ENABLE,
        PMIC_WDG_RETLONGWIN_ENABLE,
        PMIC_WDG_QA_FEEDBACK_VALUE_2,
        PMIC_WDG_QA_LFSR_VALUE_1,
        PMIC_WDG_QA_QUES_SEED_VALUE_5,
    };

    test_pmic_print_unity_testcase_info(1005U,
                                        pmic_wdg_tests,
                                        PMIC_WDG_NUM_OF_TESTCASES);

    pmicStatus = Pmic_wdgSetCfg(pPmicCoreHandle, wdgCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_WDG_WINDOW, pmicStatus);
}

/*!
 * \brief   Parameter validation for 'win1Duration_us' max Value
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
        PMIC_WDG_RESET_ENABLE,
        PMIC_WDG_RETLONGWIN_ENABLE,
        PMIC_WDG_QA_FEEDBACK_VALUE_2,
        PMIC_WDG_QA_LFSR_VALUE_1,
        PMIC_WDG_QA_QUES_SEED_VALUE_5,
    };

    test_pmic_print_unity_testcase_info(1006U,
                                        pmic_wdg_tests,
                                        PMIC_WDG_NUM_OF_TESTCASES);

    pmicStatus = Pmic_wdgSetCfg(pPmicCoreHandle, wdgCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_WDG_WINDOW, pmicStatus);
}

/*!
 * \brief   Parameter validation for 'win2Duration_us' min Value
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
        PMIC_WDG_RESET_ENABLE,
        PMIC_WDG_RETLONGWIN_ENABLE,
        PMIC_WDG_QA_FEEDBACK_VALUE_2,
        PMIC_WDG_QA_LFSR_VALUE_1,
        PMIC_WDG_QA_QUES_SEED_VALUE_5,
    };

    test_pmic_print_unity_testcase_info(1007U,
                                        pmic_wdg_tests,
                                        PMIC_WDG_NUM_OF_TESTCASES);

    pmicStatus = Pmic_wdgSetCfg(pPmicCoreHandle, wdgCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_WDG_WINDOW, pmicStatus);
}

/*!
 * \brief   Parameter validation for 'win2Duration_us' max Value
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
        PMIC_WDG_RESET_ENABLE,
        PMIC_WDG_RETLONGWIN_ENABLE,
        PMIC_WDG_QA_FEEDBACK_VALUE_2,
        PMIC_WDG_QA_LFSR_VALUE_1,
        PMIC_WDG_QA_QUES_SEED_VALUE_5,
    };

    test_pmic_print_unity_testcase_info(1008U,
                                        pmic_wdg_tests,
                                        PMIC_WDG_NUM_OF_TESTCASES);

    pmicStatus = Pmic_wdgSetCfg(pPmicCoreHandle, wdgCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_WDG_WINDOW, pmicStatus);
}

/*!
 * \brief   Parameter validation for 'failThreshold' max Value
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
        PMIC_WDG_RESET_ENABLE,
        PMIC_WDG_RETLONGWIN_ENABLE,
        PMIC_WDG_QA_FEEDBACK_VALUE_2,
        PMIC_WDG_QA_LFSR_VALUE_1,
        PMIC_WDG_QA_QUES_SEED_VALUE_5,
    };

    test_pmic_print_unity_testcase_info(1009U,
                                        pmic_wdg_tests,
                                        PMIC_WDG_NUM_OF_TESTCASES);

    pmicStatus = Pmic_wdgSetCfg(pPmicCoreHandle, wdgCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);
}

/*!
 * \brief   Parameter validation for 'rstThreshold' max Value
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
        PMIC_WDG_RESET_ENABLE,
        PMIC_WDG_RETLONGWIN_ENABLE,
        PMIC_WDG_QA_FEEDBACK_VALUE_2,
        PMIC_WDG_QA_LFSR_VALUE_1,
        PMIC_WDG_QA_QUES_SEED_VALUE_5,
    };

    test_pmic_print_unity_testcase_info(1010U,
                                        pmic_wdg_tests,
                                        PMIC_WDG_NUM_OF_TESTCASES);

    pmicStatus = Pmic_wdgSetCfg(pPmicCoreHandle, wdgCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);
}

/*!
 * \brief   Parameter validation for 'qaFdbk' max Value
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
        PMIC_WDG_RESET_ENABLE,
        PMIC_WDG_RETLONGWIN_ENABLE,
        4U,
        PMIC_WDG_QA_LFSR_VALUE_1,
        PMIC_WDG_QA_QUES_SEED_VALUE_5,
    };

    test_pmic_print_unity_testcase_info(1011U,
                                        pmic_wdg_tests,
                                        PMIC_WDG_NUM_OF_TESTCASES);

    pmicStatus = Pmic_wdgSetCfg(pPmicCoreHandle, wdgCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);
}

/*!
 * \brief   Parameter validation for 'qaLfsr' max Value
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
        PMIC_WDG_RESET_ENABLE,
        PMIC_WDG_RETLONGWIN_ENABLE,
        PMIC_WDG_QA_FEEDBACK_VALUE_2,
        4U,
        PMIC_WDG_QA_QUES_SEED_VALUE_5,
    };

    test_pmic_print_unity_testcase_info(1012U,
                                        pmic_wdg_tests,
                                        PMIC_WDG_NUM_OF_TESTCASES);

    pmicStatus = Pmic_wdgSetCfg(pPmicCoreHandle, wdgCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);
}

/*!
 * \brief   Parameter validation for 'qaQuesSeed' max Value
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
        PMIC_WDG_RESET_ENABLE,
        PMIC_WDG_RETLONGWIN_ENABLE,
        PMIC_WDG_QA_FEEDBACK_VALUE_2,
        PMIC_WDG_QA_LFSR_VALUE_1,
        16,
    };

    test_pmic_print_unity_testcase_info(1013U,
                                        pmic_wdg_tests,
                                        PMIC_WDG_NUM_OF_TESTCASES);

    pmicStatus = Pmic_wdgSetCfg(pPmicCoreHandle, wdgCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);
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
        PMIC_WDG_RESET_ENABLE,
        PMIC_WDG_RETLONGWIN_ENABLE,
        PMIC_WDG_QA_FEEDBACK_VALUE_2,
        PMIC_WDG_QA_LFSR_VALUE_1,
        PMIC_WDG_QA_QUES_SEED_VALUE_5,
    };

    test_pmic_print_unity_testcase_info(1014U,
                                        pmic_wdg_tests,
                                        PMIC_WDG_NUM_OF_TESTCASES);

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
}

/*!
 * \brief   Parameter validation for 'handle'
 */
static void test_pmic_wdg_getCfg_prmValTest_handle(void)
{
    int32_t pmicStatus      = PMIC_ST_SUCCESS;
    Pmic_WdgCfg_t wdgCfg = {PMIC_WDG_CFG_SETPARAMS_FORALL, };

    test_pmic_print_unity_testcase_info(1015U,
                                        pmic_wdg_tests,
                                        PMIC_WDG_NUM_OF_TESTCASES);

    pmicStatus = Pmic_wdgGetCfg(NULL, &wdgCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, pmicStatus);
}

/*!
 * \brief   Parameter validation for 'wdgCfg'
 */
static void test_pmic_wdg_getCfg_prmValTest_wdgcfgParam(void)
{
    int32_t pmicStatus      = PMIC_ST_SUCCESS;

    test_pmic_print_unity_testcase_info(1016U,
                                        pmic_wdg_tests,
                                        PMIC_WDG_NUM_OF_TESTCASES);

    pmicStatus = Pmic_wdgGetCfg(pPmicCoreHandle, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, pmicStatus);
}

/*!
 * \brief   Parameter validation for 'handle'
 */
static void test_pmic_wdg_enable_prmValTest_handle(void)
{
    int32_t pmicStatus      = PMIC_ST_SUCCESS;

    test_pmic_print_unity_testcase_info(1017U,
                                        pmic_wdg_tests,
                                        PMIC_WDG_NUM_OF_TESTCASES);

    pmicStatus = Pmic_wdgEnable(NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, pmicStatus);
}

/*!
 * \brief   Parameter validation for 'handle'
 */
static void test_pmic_wdg_disable_prmValTest_handle(void)
{
    int32_t pmicStatus      = PMIC_ST_SUCCESS;

    test_pmic_print_unity_testcase_info(1018U,
                                        pmic_wdg_tests,
                                        PMIC_WDG_NUM_OF_TESTCASES);

    pmicStatus = Pmic_wdgDisable(NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, pmicStatus);
}

/*!
 * \brief   Test wdg QA sequence Parameter validation for 'handle'
 */
static void test_pmic_wdg_startQaSequence_prmValTest_handle(void)
{
    int32_t pmicStatus      = PMIC_ST_SUCCESS;
    Pmic_WdgCfg_t wdgCfg    =
    {
        PMIC_WDG_CFG_SETPARAMS_FORALL,
        24000U,
        6150U,
        4950U,
        PMIC_WDG_FAIL_THRESHOLD_COUNT_7,
        PMIC_WDG_RESET_THRESHOLD_COUNT_7,
        PMIC_WDG_QA_MODE,
        PMIC_WDG_PWRHOLD_DISABLE,
        PMIC_WDG_RESET_ENABLE,
        PMIC_WDG_RETLONGWIN_ENABLE,
        PMIC_WDG_QA_FEEDBACK_VALUE_0,
        PMIC_WDG_QA_LFSR_VALUE_0,
        PMIC_WDG_QA_QUES_SEED_VALUE_10,
    };

    test_pmic_print_unity_testcase_info(1019U,
                                        pmic_wdg_tests,
                                        PMIC_WDG_NUM_OF_TESTCASES);

    /* Enable WDG Timer */
    pmicStatus = Pmic_wdgEnable(pPmicCoreHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    /* Set QA parameters */
    pmicStatus = Pmic_wdgSetCfg(pPmicCoreHandle, wdgCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    /* Start Watchdog QA sequence */
    pmicStatus = Pmic_wdgStartQaSequence(NULL, 5U);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, pmicStatus);

    /* Disable WDG Timer */
    pmicStatus = Pmic_wdgDisable(pPmicCoreHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
}

/*!
 * \brief   Test wdg QA sequence
 */
static void test_pmic_wdg_startQaSequence(void)
{
    int32_t pmicStatus      = PMIC_ST_SUCCESS;
    Pmic_WdgCfg_t wdgCfg    =
    {
        PMIC_WDG_CFG_SETPARAMS_FORALL,
        750000U,
        6150U,
        4950U,
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

    test_pmic_print_unity_testcase_info(1020U,
                                        pmic_wdg_tests,
                                        PMIC_WDG_NUM_OF_TESTCASES);

    /* Enable WDG Timer */
    pmicStatus = Pmic_wdgEnable(pPmicCoreHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    /* Set QA parameters */
    pmicStatus = Pmic_wdgSetCfg(pPmicCoreHandle, wdgCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    /* Start Watchdog QA sequence */
    pmicStatus = Pmic_wdgStartQaSequence(pPmicCoreHandle, 5U);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    /* Disable WDG Timer */
    pmicStatus = Pmic_wdgDisable(pPmicCoreHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
}

/*!
 * \brief   Test wdg QA sequences with different QA feedback values
 */
static void test_pmic_wdg_startQaSequence_testFdbkValues(void)
{
    int32_t pmicStatus      = PMIC_ST_SUCCESS;
    uint8_t fdbk = 0U;
    Pmic_WdgCfg_t wdgCfg_rd = {0U};
    Pmic_WdgCfg_t wdgCfg    =
    {
        PMIC_WDG_CFG_SETPARAMS_FORALL,
        750000U,
        6150U,
        4950U,
        PMIC_WDG_FAIL_THRESHOLD_COUNT_7,
        PMIC_WDG_RESET_THRESHOLD_COUNT_7,
        PMIC_WDG_QA_MODE,
        PMIC_WDG_PWRHOLD_DISABLE,
        PMIC_WDG_RESET_ENABLE,
        PMIC_WDG_RETLONGWIN_ENABLE,
        PMIC_WDG_QA_FEEDBACK_VALUE_0,
        PMIC_WDG_QA_LFSR_VALUE_0,
        PMIC_WDG_QA_QUES_SEED_VALUE_10,
    };

    test_pmic_print_unity_testcase_info(1021U,
                                        pmic_wdg_tests,
                                        PMIC_WDG_NUM_OF_TESTCASES);

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
        pmicStatus = Pmic_wdgStartQaSequence(pPmicCoreHandle, 5);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        /* Disable WDG Timer */
        pmicStatus = Pmic_wdgDisable(pPmicCoreHandle);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        /* Delay is needed to sattle down watchdog to longwindow */
        Osal_delay(20U);
    }
}

/*!
 * \brief   Test wdg QA sequences with different QA LFSR values
 */
static void test_pmic_wdg_startQaSequence_testLfsrValues(void)
{
    int32_t pmicStatus      = PMIC_ST_SUCCESS;
    uint8_t lfsr = 0U;
    Pmic_WdgCfg_t wdgCfg_rd = {0U};
    Pmic_WdgCfg_t wdgCfg    =
    {
        PMIC_WDG_CFG_SETPARAMS_FORALL,
        750000U,
        6150U,
        4950U,
        PMIC_WDG_FAIL_THRESHOLD_COUNT_7,
        PMIC_WDG_RESET_THRESHOLD_COUNT_7,
        PMIC_WDG_QA_MODE,
        PMIC_WDG_PWRHOLD_DISABLE,
        PMIC_WDG_RESET_ENABLE,
        PMIC_WDG_RETLONGWIN_ENABLE,
        PMIC_WDG_QA_FEEDBACK_VALUE_0,
        PMIC_WDG_QA_LFSR_VALUE_0,
        PMIC_WDG_QA_QUES_SEED_VALUE_10,
    };

    test_pmic_print_unity_testcase_info(1022U,
                                        pmic_wdg_tests,
                                        PMIC_WDG_NUM_OF_TESTCASES);

    for(lfsr = PMIC_WDG_QA_LFSR_VALUE_0;
        lfsr <= PMIC_WDG_QA_LFSR_VALUE_3;
        lfsr++)
    {
        wdgCfg.qaLfsr = lfsr;
        /* Set QA parameters */
        pmicStatus = Pmic_wdgSetCfg(pPmicCoreHandle, wdgCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        /* Get wdg config to check lfsr value */
        wdgCfg_rd.validParams = PMIC_CFG_WDG_QA_LFSR_VALID_SHIFT;
        pmicStatus = Pmic_wdgGetCfg(pPmicCoreHandle, &wdgCfg_rd);
        TEST_ASSERT_EQUAL(wdgCfg.qaLfsr, wdgCfg_rd.qaLfsr);

        /* Start Watchdog QA sequence */
        pmicStatus = Pmic_wdgStartQaSequence(pPmicCoreHandle, 5);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        /* Disable WDG Timer */
        pmicStatus = Pmic_wdgDisable(pPmicCoreHandle);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        /* Delay is needed to sattle down watchdog to longwindow */
        Osal_delay(20U);
    }
}

/*!
 * \brief   Test wdg QA sequences with different QA Question Seed values
 */
static void test_pmic_wdg_startQaSequence_testQuesSeedValues(void)
{
    int32_t pmicStatus      = PMIC_ST_SUCCESS;
    uint8_t quesSeed = 0U;
    Pmic_WdgCfg_t wdgCfg_rd = {0U};
    Pmic_WdgCfg_t wdgCfg    =
    {
        PMIC_WDG_CFG_SETPARAMS_FORALL,
        750000U,
        5500U,
        4950U,
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

    test_pmic_print_unity_testcase_info(1023U,
                                        pmic_wdg_tests,
                                        PMIC_WDG_NUM_OF_TESTCASES);

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
        pmicStatus = Pmic_wdgStartQaSequence(pPmicCoreHandle, 5);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        /* Disable WDG Timer */
        pmicStatus = Pmic_wdgDisable(pPmicCoreHandle);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        /* Delay is needed to sattle down watchdog to longwindow */
        Osal_delay(20U);
    }
}

/*!
 * \brief   Test get wdg failcount
 */
static void test_pmic_wdg_GetFailCount(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t failCount  = 0U;

    test_pmic_print_unity_testcase_info(1024U,
                                        pmic_wdg_tests,
                                        PMIC_WDG_NUM_OF_TESTCASES);

    /* Enable WDG Timer */
    pmicStatus = Pmic_wdgEnable(pPmicCoreHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    /* Get watchdog failcount */
    pmicStatus = Pmic_wdgGetFailCount(pPmicCoreHandle, &failCount);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
}

/*!
 * \brief   Test get wdg failcount Parameter validation for 'handle'
 */
static void test_pmic_wdg_GetFailCount_prmValTest_handle(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t failCount  = 0U;

    test_pmic_print_unity_testcase_info(1025U,
                                        pmic_wdg_tests,
                                        PMIC_WDG_NUM_OF_TESTCASES);

    /* Enable WDG Timer */
    pmicStatus = Pmic_wdgEnable(pPmicCoreHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    /* Get watchdog failcount */
    pmicStatus = Pmic_wdgGetFailCount(NULL, &failCount);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, pmicStatus);
}

/*!
 * \brief   Test get wdg failcount Parameter validation for 'failCount'
 */
static void test_pmic_wdg_GetFailCount_prmValTest_invFailCountParam(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    test_pmic_print_unity_testcase_info(1026U,
                                        pmic_wdg_tests,
                                        PMIC_WDG_NUM_OF_TESTCASES);

    /* Enable WDG Timer */
    pmicStatus = Pmic_wdgEnable(pPmicCoreHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    /* Get watchdog failcount */
    pmicStatus = Pmic_wdgGetFailCount(pPmicCoreHandle, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, pmicStatus);
}

/*!
 * \brief   Test get wdg error status
 */
static void test_pmic_wdg_GetErrorStatus(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_WdgErrStatus_t errStatus = {0U};

    test_pmic_print_unity_testcase_info(1027U,
                                        pmic_wdg_tests,
                                        PMIC_WDG_NUM_OF_TESTCASES);

    /* Enable WDG Timer */
    pmicStatus = Pmic_wdgEnable(pPmicCoreHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    errStatus.validParams = PMIC_CFG_WD_ALL_ERRSTAT_VALID_PARAMS;
    /* Get watchdog error Status */
    pmicStatus = Pmic_wdgGetErrorStatus(pPmicCoreHandle, &errStatus);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
}

/*!
 * \brief   Test get wdg error status Parameter validation for 'handle'
 */
static void test_pmic_wdg_GetErrorStatus_prmValTest_handle(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_WdgErrStatus_t errStatus = {0U};

    test_pmic_print_unity_testcase_info(1028U,
                                        pmic_wdg_tests,
                                        PMIC_WDG_NUM_OF_TESTCASES);

    /* Enable WDG Timer */
    pmicStatus = Pmic_wdgEnable(pPmicCoreHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    errStatus.validParams = PMIC_CFG_WD_ALL_ERRSTAT_VALID_PARAMS;
    /* Get watchdog error Status */
    pmicStatus = Pmic_wdgGetErrorStatus(NULL, &errStatus);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, pmicStatus);
}

/*!
 * \brief   Test get wdg failcount Parameter validation for 'errStatus'
 */
static void test_pmic_wdg_GetErrorStatus_prmValTest_invErrStatParam(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    test_pmic_print_unity_testcase_info(1029U,
                                        pmic_wdg_tests,
                                        PMIC_WDG_NUM_OF_TESTCASES);

    /* Enable WDG Timer */
    pmicStatus = Pmic_wdgEnable(pPmicCoreHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    /* Get watchdog error Status */
    pmicStatus = Pmic_wdgGetErrorStatus(pPmicCoreHandle, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, pmicStatus);
}

#if 0
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
        6150U,
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

    test_pmic_print_unity_testcase_info(1030U,
                                        pmic_wdg_tests,
                                        PMIC_WDG_NUM_OF_TESTCASES);

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
}
#endif

/*!
 * \brief   Test Watchdog trigger mode Parameter validation for 'handle'
 */
static void test_pmic_wdg_StartTriggerSequence_prmValTest_handle(void)
{
    int32_t pmicStatus      = PMIC_ST_SUCCESS;
    Pmic_WdgCfg_t wdgCfg    =
    {
        PMIC_WDG_CFG_SETPARAMS_FORALL,
        750000U,
        6150U,
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

    test_pmic_print_unity_testcase_info(1031U,
                                        pmic_wdg_tests,
                                        PMIC_WDG_NUM_OF_TESTCASES);

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

#if 0
    RUN_TEST(test_pmic_wdg_StartTriggerSequence);
#endif
    RUN_TEST(test_pmic_wdg_StartTriggerSequence_prmValTest_handle);

    UNITY_END();
}

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

    pmicConfigData.slaveAddr           = LEO_PMICA_SLAVE_ADDR;
    pmicConfigData.validParams        |= PMIC_CFG_SLAVEADDR_VALID_SHIFT;

    pmicConfigData.qaSlaveAddr         = LEO_PMICA_WDG_SLAVE_ADDR;
    pmicConfigData.validParams        |= PMIC_CFG_QASLAVEADDR_VALID_SHIFT;

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
 * \brief   WDG Unity Test App wrapper Function for LEO PMIC-B
 */
static int32_t test_pmic_leo_pmicB_wdg_testApp(void)
{
    int32_t status                = PMIC_ST_SUCCESS;
    Pmic_CoreCfg_t pmicConfigData = {0U};

    /* Fill parameters to pmicConfigData */
    pmicConfigData.pmicDeviceType      = PMIC_DEV_LEO_TPS6594X;
    pmicConfigData.validParams        |= PMIC_CFG_DEVICE_TYPE_VALID_SHIFT;

    pmicConfigData.commMode            = PMIC_INTF_SINGLE_I2C;
    pmicConfigData.validParams        |= PMIC_CFG_COMM_MODE_VALID_SHIFT;

    pmicConfigData.slaveAddr           = LEO_PMICB_SLAVE_ADDR;
    pmicConfigData.validParams        |= PMIC_CFG_SLAVEADDR_VALID_SHIFT;

    pmicConfigData.qaSlaveAddr         = LEO_PMICB_WDG_SLAVE_ADDR;
    pmicConfigData.validParams        |= PMIC_CFG_QASLAVEADDR_VALID_SHIFT;

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

    pmicConfigData.slaveAddr           = HERA_PMIC_SLAVE_ADDR;
    pmicConfigData.validParams        |= PMIC_CFG_SLAVEADDR_VALID_SHIFT;

    pmicConfigData.qaSlaveAddr         = HERA_PMIC_WDG_SLAVE_ADDR;
    pmicConfigData.validParams        |= PMIC_CFG_QASLAVEADDR_VALID_SHIFT;

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

static const char pmicTestAppMenu[] =
{
    " \r\n ================================================================="
    " \r\n Menu:"
    " \r\n ================================================================="
    " \r\n 0: Pmic Leo device(PMIC A on J721E EVM)"
    " \r\n 1: Pmic Leo device(PMIC B on J721E EVM)"
    " \r\n 2: Pmic Hera device"
    " \r\n 3: quit"
    " \r\n"
    " \r\n Enter option: "
};

/*!
 * \brief   Function to register WDG Unity Test App wrapper to Unity framework
 */
static void test_pmic_wdg_testapp_runner(void)
{
    /* @description : Test runner for wdg Test App
     *
     * @requirements: XXXX
     *
     * @cores       : mcu1_0, mcu1_1
     */

    int8_t num = -1;

    while(1U)
    {
        pmic_log("%s", pmicTestAppMenu);
        if(UART_scanFmt("%d", &num) != 0U)
        {
            pmic_log("Read from UART Console failed\n");
            return;
        }

        switch(num)
        {
           case 0U:
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
               break;
           case 1U:
               /* WDG Unity Test App wrapper Function for LEO PMIC-B */
               if(PMIC_ST_SUCCESS == test_pmic_leo_pmicB_wdg_testApp())
               {
                   /* TODO : Cuasing Hang on the board */
#if 0
                   /* Run WDG test cases for Leo PMIC-B */
                   test_pmic_run_testcases();
#endif
               }

               /* Deinit pmic handle */
               if(pPmicCoreHandle != NULL)
               {
                   test_pmic_appDeInit(pPmicCoreHandle);
               }
               break;
           case 2U:
               /* WDG Unity Test App wrapper Function for HERA PMIC */
               if(PMIC_ST_SUCCESS == test_pmic_hera_wdg_testApp())
               {
                   /* Run wdg test cases for Hera PMIC */
                   test_pmic_run_testcases();
               }
               /* Deinit pmic handle */
               if(pPmicCoreHandle != NULL)
               {
                   test_pmic_appDeInit(pPmicCoreHandle);
               }
               break;
           case 3U:
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
 * \brief   TI RTOS specific GPIO TEST APP main Function
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code.
 *          For valid values see \ref Pmic_ErrorCodes
 */
int main()
{
    test_pmic_uartInit();

    pmic_log("PMIC Watchdog Unity Test Application(%s %s)\n",
                                         __TIME__, __DATE__);

#if defined(UNITY_INCLUDE_CONFIG_V2_H) && \
    (defined(SOC_J721E)             || \
     defined(SOC_J7200))
    test_pmic_wdg_testapp_runner();
#endif
}
