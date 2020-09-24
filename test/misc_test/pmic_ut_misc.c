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
 *  \file   pmic_ut_misc.c
 *
 *  \brief  PMIC Unit Test for testing PMIC MISC APIs
 *
 */

#include <pmic_ut_misc.h>

/* Pointer holds the pPmicCoreHandle */
Pmic_CoreHandle_t *pPmicCoreHandle = NULL;

static uint16_t pmic_device_info = 0U;

/*!
 * \brief   PMIC MISC Test Cases
 */
static Pmic_Ut_Tests_t pmic_misc_tests[] =
{
    /*! testID
     *  testDesc
     */
    {
        7628,
        "Pmic_SetRecoveryCntCfg : Test Set Recovery Counter Threshold."
    },
    {
        7629,
        "Pmic_SetRecoveryCntCfg : Parameter validation for thrVal."
    },
    {
        7630,
        "Pmic_SetRecoveryCntCfg : Test Set Clear Recovery Counter."
    },
    {
        7631,
        "Pmic_SetRecoveryCntCfg : Parameter validation for clrCnt."
    },
    {
        7632,
        "Pmic_SetRecoveryCntCfg : Parameter validation for handle."
    },
    {
        7633,
        "Pmic_getRecoveryCntCfg : Parameter validation for handle."
    },
    {
        7634,
        "Pmic_getRecoveryCntCfg : Parameter validation for recovCntCfg."
    },
    {
        7635,
        "Pmic_getRecoveryCnt : Parameter validation for handle."
    },
    {
        7636,
        "Pmic_getRecoveryCnt : Parameter validation for recovCntVal."
    },
    {
        7637,
        "Pmic_getRecoveryCnt : Read Recovery Count Value."
    },
    {
        7715,
        "Pmic_irqGetErrStatus : Test BIST_PASS_INT interrupt."
    },
    {
        7716,
        "Pmic_irqGetErrStatus : Test FSD_INT interrupt."
    },
    {
        7768,
        "Pmic_irqGetErrStatus : Test ENABLE_INT interrupt."
    },
};


/*!
 * \brief   Pmic_irqGetErrStatus : Test ENABLE_INT interrupt.
 */
static void test_Pmic_Enable_interrupt(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    bool intrEnable           = PMIC_IRQ_UNMASK;

    Pmic_WdgCfg_t wdgCfg    =
    {
        PMIC_WDG_CFG_SETPARAMS_FORALL,
        3000U,
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

    test_pmic_print_unity_testcase_info(7768,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    if(J721E_LEO_PMICB_DEVICE == pmic_device_info)
    {
        TEST_IGNORE();
    }

    if(startup_type == PMIC_ENABLE_STARTUP_TYPE)
    {
        pmic_log("Enable-pin Start-up is detected\n");
    }
    else
    {
        if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
        {
            pmicStatus = Pmic_irqMaskIntr(pPmicCoreHandle,
                                          PMIC_TPS6594X_ENABLE_INT,
                                          intrEnable);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
        }
        if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
        {
            pmicStatus = Pmic_irqMaskIntr(pPmicCoreHandle,
                                          PMIC_LP8764X_ENABLE_INT,
                                          intrEnable);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
        }

        /* Enable WDG Timer */
        pmicStatus = Pmic_wdgEnable(pPmicCoreHandle);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        /* Set QA parameters */
        pmicStatus = Pmic_wdgSetCfg(pPmicCoreHandle, wdgCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        /* Waiting for longwindow time interval expiring */
        Osal_delay(wdgCfg.longWinDuration_ms);
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
}

/*!
 * \brief   Pmic_SetRecoveryCntCfg : Test Set Recovery Counter Threshold.
 */
static void test_pmic_SetRecoveryCntCfg_threshold(void)
{
    int32_t status             = PMIC_ST_SUCCESS;
    Pmic_RecovCntCfg_t recovCntCfg;
    Pmic_RecovCntCfg_t recovCntCfg_rd;

    recovCntCfg.validParams    = PMIC_CFG_RECOV_CNT_THR_VAL_VALID_SHIFT;
    recovCntCfg_rd.validParams = PMIC_CFG_RECOV_CNT_THR_VAL_VALID_SHIFT;
    recovCntCfg.thrVal         = PMIC_RECOV_CNT_THR_MAX;

    test_pmic_print_unity_testcase_info(7628,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    status = Pmic_SetRecoveryCntCfg(pPmicCoreHandle, recovCntCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_getRecoveryCntCfg(pPmicCoreHandle, &recovCntCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    TEST_ASSERT_EQUAL(recovCntCfg.thrVal, recovCntCfg_rd.thrVal);

}

/*!
 * \brief   Pmic_SetRecoveryCntCfg : Parameter validation for thrVal.
 */
static void test_pmic_SetRecoveryCntCfgPrmValTest_thrVal(void)
{
    int32_t status             = PMIC_ST_SUCCESS;
    Pmic_RecovCntCfg_t recovCntCfg;

    recovCntCfg.validParams = PMIC_CFG_RECOV_CNT_THR_VAL_VALID_SHIFT;
    recovCntCfg.thrVal      = PMIC_RECOV_CNT_THR_MAX + 1U;

    test_pmic_print_unity_testcase_info(7629,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    status = Pmic_SetRecoveryCntCfg(pPmicCoreHandle, recovCntCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

}

/*!
 * \brief   Pmic_SetRecoveryCntCfg : Test Set Clear Recovery Counter.
 */
static void test_pmic_SetRecoveryCntCfg_clrCnt(void)
{
    int32_t status             = PMIC_ST_SUCCESS;
    Pmic_RecovCntCfg_t recovCntCfg;

    recovCntCfg.validParams = PMIC_CFG_RECOV_CNT_CLR_CNT_VALID_SHIFT;
    recovCntCfg.clrCnt      = 1U;

    test_pmic_print_unity_testcase_info(7630,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    status = Pmic_SetRecoveryCntCfg(pPmicCoreHandle, recovCntCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

}

/*!
 * \brief   Pmic_SetRecoveryCntCfg : Parameter validation for clrCnt.
 */
static void test_pmic_SetRecoveryCntCfgPrmValTest_clrCnt(void)
{
    int32_t status             = PMIC_ST_SUCCESS;
    Pmic_RecovCntCfg_t recovCntCfg;

    recovCntCfg.validParams = PMIC_CFG_RECOV_CNT_CLR_CNT_VALID_SHIFT;
    recovCntCfg.clrCnt      = 0U;

    test_pmic_print_unity_testcase_info(7631,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    status = Pmic_SetRecoveryCntCfg(pPmicCoreHandle, recovCntCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

}

/*!
 * \brief   Pmic_SetRecoveryCntCfg : Parameter validation for handle.
 */
static void test_pmic_SetRecoveryCntCfgPrmValTest_handle(void)
{
    int32_t status  = PMIC_ST_SUCCESS;
    Pmic_RecovCntCfg_t recovCntCfg;

    recovCntCfg.validParams = PMIC_CFG_RECOV_CNT_CLR_CNT_VALID_SHIFT;

    test_pmic_print_unity_testcase_info(7632,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    status = Pmic_SetRecoveryCntCfg(NULL, recovCntCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, status);

}

/*!
 * \brief   Pmic_getRecoveryCntCfg : Parameter validation for handle.
 */
static void test_pmic_getRecoveryCntCfgPrmValTest_handle(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_RecovCntCfg_t recovCntCfg;

    test_pmic_print_unity_testcase_info(7633,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    status = Pmic_getRecoveryCntCfg(NULL, &recovCntCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, status);

}

/*!
 * \brief   Pmic_getRecoveryCntCfg : Parameter validation for recovCntCfg.
 */
static void test_pmic_getRecoveryCntCfgPrmValTest_recovCntCfg(void)
{
    int32_t status             = PMIC_ST_SUCCESS;

    test_pmic_print_unity_testcase_info(7634,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    status = Pmic_getRecoveryCntCfg(pPmicCoreHandle, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

}

/*!
 * \brief   Pmic_getRecoveryCnt : Parameter validation for handle.
 */
static void test_Pmic_getRecoveryCntPrmValTest_handle(void)
{
    int32_t status      = PMIC_ST_SUCCESS;
    uint8_t recovCntVal = 0U;

    test_pmic_print_unity_testcase_info(7635,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    status = Pmic_getRecoveryCnt(NULL, &recovCntVal);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, status);

}

/*!
 * \brief   Pmic_getRecoveryCnt : Parameter validation for recovCntVal.
 */
static void test_Pmic_getRecoveryCntPrmValTest_recovCntVal(void)
{
    int32_t status             = PMIC_ST_SUCCESS;

    test_pmic_print_unity_testcase_info(7636,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    status = Pmic_getRecoveryCnt(pPmicCoreHandle, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

}

/*!
 * To test the following manual test-case,
 * the test-case should be executed twice continuously.
 */
/*!
 * \brief   Pmic_getRecoveryCnt : Read Recovery Count Value.
 */
static void test_Pmic_getRecoveryCnt_read_recovCntVal(void)
{
    int32_t status          = PMIC_ST_SUCCESS;
    uint8_t recovCntVal     = 0U;
    Pmic_WdgCfg_t wdgCfg    =
    {
        PMIC_WDG_CFG_SETPARAMS_FORALL,
        3000U,
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

    test_pmic_print_unity_testcase_info(7637,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    if(J721E_LEO_PMICB_DEVICE == pmic_device_info)
    {
        TEST_IGNORE();
    }

    status = Pmic_getRecoveryCnt(pPmicCoreHandle, &recovCntVal);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    pmic_log("Recovery count: %d\n", recovCntVal);
    /* Enable WDG Timer */
    status = Pmic_wdgEnable(pPmicCoreHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    /* Set QA parameters */
    status = Pmic_wdgSetCfg(pPmicCoreHandle, wdgCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

}

/*!
 * \brief   Pmic_irqGetErrStatus : Test BIST_PASS_INT interrupt.
 */
static void test_Pmic_getBistPassInterrupt(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_IrqStatus_t errStat  = {0U};
    bool clearIRQ             = false;
    bool nsleepType = 0U;
    bool maskEnable = 0U;
    int8_t timeout = 10U;

    test_pmic_print_unity_testcase_info(7715,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pmicStatus = Pmic_irqMaskIntr(pPmicCoreHandle,
                                      PMIC_TPS6594X_BIST_PASS_INT,
                                      PMIC_IRQ_UNMASK);
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pmicStatus = Pmic_irqMaskIntr(pPmicCoreHandle,
                                      PMIC_LP8764X_BIST_PASS_INT,
                                      PMIC_IRQ_UNMASK);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    }

    if(J7VCL_LEO_PMICA_DEVICE == pmic_device_info)
    {
        TEST_IGNORE();
    }

    nsleepType = PMIC_NSLEEP1_SIGNAL;
    maskEnable = PMIC_NSLEEPX_MASK;

    pmicStatus = Pmic_fsmSetNsleepSignalMask(pPmicCoreHandle,
                                             nsleepType,
                                             maskEnable);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    nsleepType = PMIC_NSLEEP2_SIGNAL;
    maskEnable = PMIC_NSLEEPX_MASK;

    pmicStatus = Pmic_fsmSetNsleepSignalMask(pPmicCoreHandle,
                                             nsleepType,
                                             maskEnable);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    /* To clear the interrupts*/
    pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, true);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_fsmRequestRuntimeBist(pPmicCoreHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        while(timeout--)
        {
            /* Delay added to avoid timeout */
            Osal_delay(1000);

            pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
            if((PMIC_ST_SUCCESS == pmicStatus) &&
               ((errStat.intStatus[PMIC_TPS6594X_BIST_PASS_INT/32U] &
                 (1U << (PMIC_TPS6594X_BIST_PASS_INT % 32U))) != 0U))
            {
                if(PMIC_ST_SUCCESS == pmicStatus)
                {
                    /* clear the interrupt */
                    pmicStatus = Pmic_irqClrErrStatus(pPmicCoreHandle,
                                                      PMIC_TPS6594X_BIST_PASS_INT);
                    break;
                }
            }
        }

        if(0 > timeout)
        {
            pmicStatus = PMIC_ST_ERR_FAIL;
        }

        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        while(timeout--)
        {
            /* Delay added to avoid timeout */
            Osal_delay(1000);

            pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
            if((PMIC_ST_SUCCESS == pmicStatus) &&
               ((errStat.intStatus[PMIC_LP8764X_BIST_PASS_INT/32U] &
                 (1U << (PMIC_LP8764X_BIST_PASS_INT % 32U))) != 0U))
            {
                if(PMIC_ST_SUCCESS == pmicStatus)
                {
                    /* clear the interrupt */
                    pmicStatus = Pmic_irqClrErrStatus(pPmicCoreHandle,
                                                      PMIC_LP8764X_BIST_PASS_INT);
                    break;
                }
            }
        }

        if(0 > timeout)
        {
            pmicStatus = PMIC_ST_ERR_FAIL;
        }

        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    }
}

/*!
 * Below test cases are not tested because of HW limitation.
 * Added below test cases as sample for reference.
 */

#if defined(ENABLE_SAMPLE_TESTCASES)
/*!
 * \brief   Pmic_irqGetErrStatus : Test FSD_INT interrupt.
 *          To test this testcase, user needs to follow below steps
 *          1. After POR reset, run below test case by setting 'startup_type' variable to
 *             PMIC_FSD_STARTUP_TYPE.
 */
static void test_Pmic_FSD_interrupt(void)
{
    int32_t status        = PMIC_ST_SUCCESS;
    bool intrEnable       = PMIC_IRQ_UNMASK;

    Pmic_GpioCfg_t gpioCfg    =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_NPWRON_PINFUNC_NONE,
        PMIC_GPIO_HIGH
    };

    Pmic_WdgCfg_t wdgCfg    =
    {
        PMIC_WDG_CFG_SETPARAMS_FORALL,
        3000U,
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

    test_pmic_print_unity_testcase_info(7716,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    /* Ignored because, it needs NVM setup */
    TEST_IGNORE();

    if(J721E_LEO_PMICB_DEVICE == pmic_device_info)
    {
        TEST_IGNORE();
    }

    if(startup_type == PMIC_FSD_STARTUP_TYPE)
    {
        pmic_log("FSD Start-up is detected\n");
    }
    else
    {
        status = Pmic_gpioSetNPwronEnablePinConfiguration(pPmicCoreHandle,
                                                              gpioCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
        {
            status = Pmic_irqMaskIntr(pPmicCoreHandle, PMIC_TPS6594X_FSD_INT, intrEnable);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        }
        if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
        {
            status = Pmic_irqMaskIntr(pPmicCoreHandle, PMIC_LP8764X_FSD_INT, intrEnable);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        }

        /* Enable WDG Timer */
        status = Pmic_wdgEnable(pPmicCoreHandle);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        /* Set QA parameters */
        status = Pmic_wdgSetCfg(pPmicCoreHandle, wdgCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        /* Waiting for longwindow time interval expiring */
        Osal_delay(wdgCfg.longWinDuration_ms);
    }
}
/*!
 * \brief   Pmic_irqGetErrStatus : Test NPWRON_INT interrupt Manual TestCase.
 *          To test this testcase, user needs to follow below steps
 *          1. After POR reset, run below test case by setting 'startup_type' variable to
 *             PMIC_NPWRON_STARTUP_TYPE.
 */
static void test_Pmic_NPWRON_interrupt(void)
{
    int32_t status        = PMIC_ST_SUCCESS;
    bool intrEnable       = PMIC_IRQ_UNMASK;

    Pmic_GpioCfg_t gpioCfg    =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_NPWRON_PINFUNC_NPWRON,
        PMIC_GPIO_HIGH
    };

    Pmic_WdgCfg_t wdgCfg    =
    {
        PMIC_WDG_CFG_SETPARAMS_FORALL,
        3000U,
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

    test_pmic_print_unity_testcase_info(7716,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    if(J721E_LEO_PMICB_DEVICE == pmic_device_info)
    {
        TEST_IGNORE();
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        TEST_IGNORE();
    }

    if(startup_type == PMIC_NPWRON_STARTUP_TYPE)
    {
        pmic_log("NPWRON Start-up is detected\n");
    }
    else
    {
        status = Pmic_gpioSetNPwronEnablePinConfiguration(pPmicCoreHandle,
                                                              gpioCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
        {
            status = Pmic_irqMaskIntr(pPmicCoreHandle,
                                      PMIC_TPS6594X_NPWRON_PINFUNC_NPWRON,
                                      intrEnable);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        }

        /* Enable WDG Timer */
        status = Pmic_wdgEnable(pPmicCoreHandle);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        /* Set QA parameters */
        status = Pmic_wdgSetCfg(pPmicCoreHandle, wdgCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        /* Waiting for longwindow time interval expiring */
        Osal_delay(wdgCfg.longWinDuration_ms);
    }
}

/*!
 * \brief   Pmic_irqGetErrStatus : Test NPWRON_LONG_INT interrupt Manual TestCase.
 *          To test this testcase, user needs to follow below steps
 *          1. User has to hold the NPWRON button or Keep low NPWON pin some duration.
 *          2. Pmic will be reset after first step.
 *          3. After reset, run below test case by setting 'startup_type' variable to
 *             PMIC_NPWRON_STARTUP_TYPE.
 */
static void test_Pmic_NPWRON_long_interrupt(void)
{
    int32_t status        = PMIC_ST_SUCCESS;
    bool intrEnable       = PMIC_IRQ_UNMASK;

    Pmic_GpioCfg_t gpioCfg    =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_NPWRON_PINFUNC_NPWRON,
        PMIC_GPIO_HIGH
    };

    test_pmic_print_unity_testcase_info(4444,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    if(J721E_LEO_PMICB_DEVICE == pmic_device_info)
    {
        TEST_IGNORE();
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        TEST_IGNORE();
    }

    if(startup_type == PMIC_NPWRON_STARTUP_TYPE)
    {
        pmic_log("NPWRON Start-up is detected\n");
    }
    else
    {
        status = Pmic_gpioSetNPwronEnablePinConfiguration(pPmicCoreHandle,
                                                              gpioCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
        {
            status = Pmic_irqMaskIntr(pPmicCoreHandle,
                                      PMIC_TPS6594X_NPWRON_LONG_INT,
                                      intrEnable);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        }

    }
}

/*!
 * \brief   Pmic_commFrmError : Test COMM_FRM_ERROR interrupt Manual TestCase.
 *          To test this testcase, user needs to follow below steps
 *          1. PMIC SPI must be enabled by NVM.
 *          2. Run below test case to get an SPI FRAME Error
 */
static void test_Pmic_commFrmErrorIntr(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_IrqStatus_t errStat  = {0U};
    bool clearIRQ = false;
    uint8_t buffLength = 1U;
    uint8_t txBuf[PMIC_IO_BUF_SIZE] = {0};
    uint8_t txData = 0x0FU;
    uint16_t regAddr = PMIC_SCRATCH_PAD_REG_4_REGADDR;
    uint16_t pmicRegAddr = regAddr;
    uint8_t instType = PMIC_MAIN_INST;
    int8_t  timeout      = 10U;

    test_pmic_print_unity_testcase_info(5555,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    if((NULL == pPmicCoreHandle) && (PMIC_INTF_SPI == pPmicCoreHandle->commMode))
    {
        TEST_IGNORE();
    }

    if(PMIC_INTF_SPI == pPmicCoreHandle->commMode)
    {
        /*
         * Frame 3 Bytes with IO header+data as per PMIC SPI IO algorithm
         * explained in PMIC TRM
         */

        buffLength = 0;
        /* Set ADDR to txbuf[0], Bits 1-8: ADDR[7:0] */
        txBuf[buffLength] = (uint8_t)(pmicRegAddr & 0xFFU);
        buffLength++;

        /* Writing wrong value to Set PAGE to txBuf[1] 2:0 bits */
        txBuf[buffLength] = (uint8_t)((pmicRegAddr >> 8U) & 0x7U);

        /* Set R/W in txBuf[1] as bit-3, Bit 12: 0 for Write Request */
        txBuf[buffLength] &= (uint8_t)(~PMIC_IO_REQ_RW);
        buffLength++;

        if(((bool)true) == pPmicCoreHandle->crcEnable)
        {
            /* Set CRC data to txBuf[3], Bits 25-32 CRC */
            txBuf[buffLength] = Pmic_getCRC8Val(txBuf, buffLength);
            /* Increment 1 more byte to store CRC8 */
            buffLength++;
        }

    }

    /* To clear the interrupts*/
    status = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, true);

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pmicStatus = Pmic_irqMaskIntr(pPmicCoreHandle,
                                      PMIC_TPS6594X_COMM_FRM_ERR_INT,
                                      PMIC_IRQ_UNMASK);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pmicStatus = Pmic_irqMaskIntr(pPmicCoreHandle,
                                      PMIC_LP8764X_COMM_FRM_ERR_INT,
                                      PMIC_IRQ_UNMASK);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    }

    pmicStatus = test_pmic_regWrite(pPmicCoreHandle,
                                    instType,
                                    pmicRegAddr,
                                    txBuf,
                                    buffLength);

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        while(timeout--)
        {
            /* Delay added to avoid timeout */
            Osal_delay(1000);

            pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

            if(((errStat.intStatus[PMIC_TPS6594X_COMM_FRM_ERR_INT/32U] &
               (1U << (PMIC_TPS6594X_COMM_FRM_ERR_INT % 32U))) != 0U))
            {
                /* clear the interrupt */
                pmicStatus = Pmic_irqClrErrStatus(pPmicCoreHandle,
                                                  PMIC_TPS6594X_COMM_FRM_ERR_INT);
                break;
            }
        }

        if(0 > timeout)
        {
            pmicStatus = PMIC_ST_ERR_FAIL;
        }

        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        while(timeout--)
        {
            /* Delay added to avoid timeout */
            Osal_delay(1000);

            pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
            if((PMIC_ST_SUCCESS == pmicStatus) &&
               ((errStat.intStatus[PMIC_LP8764X_COMM_FRM_ERR_INT/32U] &
                 (1U << (PMIC_LP8764X_COMM_FRM_ERR_INT % 32U))) != 0U))
            {
                /* clear the interrupt */
                pmicStatus = Pmic_irqClrErrStatus(pPmicCoreHandle,
                                                  PMIC_LP8764X_COMM_FRM_ERR_INT);
                break;
            }
        }

        if(0 > timeout)
        {
            pmicStatus = PMIC_ST_ERR_FAIL;
        }

        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    }
}

#endif

#if defined(UNITY_INCLUDE_CONFIG_V2_H) && \
    (defined(SOC_J721E) || defined(SOC_J7200))

/*!
 * \brief   Run misc unity test cases for Master PMIC
 */
static void test_pmic_run_testcases(void)
{
    pmic_log("\n\n%s(): %d: Begin Unity Test Cases...\n", __func__, __LINE__);
    UNITY_BEGIN();

    RUN_TEST(test_pmic_SetRecoveryCntCfg_threshold);
    RUN_TEST(test_pmic_SetRecoveryCntCfgPrmValTest_thrVal);
    RUN_TEST(test_pmic_SetRecoveryCntCfg_clrCnt);
    RUN_TEST(test_pmic_SetRecoveryCntCfgPrmValTest_clrCnt);
    RUN_TEST(test_pmic_SetRecoveryCntCfgPrmValTest_handle);
    RUN_TEST(test_pmic_getRecoveryCntCfgPrmValTest_handle);
    RUN_TEST(test_pmic_getRecoveryCntCfgPrmValTest_recovCntCfg);
    RUN_TEST(test_Pmic_getRecoveryCntPrmValTest_handle);
    RUN_TEST(test_Pmic_getRecoveryCntPrmValTest_recovCntVal);
    RUN_TEST(test_Pmic_getBistPassInterrupt);

    UNITY_END();
}

/*!
 * \brief   Run misc unity test cases for Slave PMIC
 */
static void test_pmic_run_slave_testcases(void)
{
    pmic_log("\n\n%s(): %d: Begin Unity Test Cases...\n", __func__, __LINE__);
    UNITY_BEGIN();

    RUN_TEST(test_pmic_SetRecoveryCntCfg_threshold);
    RUN_TEST(test_pmic_SetRecoveryCntCfgPrmValTest_thrVal);
    RUN_TEST(test_pmic_SetRecoveryCntCfg_clrCnt);
    RUN_TEST(test_pmic_SetRecoveryCntCfgPrmValTest_clrCnt);
    RUN_TEST(test_pmic_SetRecoveryCntCfgPrmValTest_handle);
    RUN_TEST(test_pmic_getRecoveryCntCfgPrmValTest_handle);
    RUN_TEST(test_pmic_getRecoveryCntCfgPrmValTest_recovCntCfg);
    RUN_TEST(test_Pmic_getRecoveryCntPrmValTest_handle);
    RUN_TEST(test_Pmic_getRecoveryCntPrmValTest_recovCntVal);

    UNITY_END();
}

/*!
 * \brief   Run misc manual test cases
 */
static void test_pmic_run_testcases_recoverycnt(void)
{
    pmic_log("\n\n%s(): %d: Begin Unity Test Cases...\n", __func__, __LINE__);
    UNITY_BEGIN();
    RUN_TEST(test_Pmic_getRecoveryCnt_read_recovCntVal);

    UNITY_END();
}

/*!
 * \brief   Run misc manual test cases
 */
static void test_pmic_run_testcases_EnableInterrupt(void)
{
    pmic_log("\n\n%s(): %d: Begin Unity Test Cases...\n", __func__, __LINE__);
    UNITY_BEGIN();
    RUN_TEST(test_Pmic_Enable_interrupt);

    UNITY_END();
}

/*!
 * \brief   MISC Unity Test App wrapper Function for LEO PMIC-A
 */
static int32_t test_pmic_leo_pmicA_misc_testApp(void)
{
    int32_t status                = PMIC_ST_SUCCESS;
    Pmic_CoreCfg_t pmicConfigData = {0U};

    /* Fill parameters to pmicConfigData */
    pmicConfigData.pmicDeviceType      = PMIC_DEV_LEO_TPS6594X;
    pmicConfigData.validParams        |= PMIC_CFG_DEVICE_TYPE_VALID_SHIFT;

    pmicConfigData.commMode            = PMIC_INTF_DUAL_I2C;
    pmicConfigData.validParams        |= PMIC_CFG_COMM_MODE_VALID_SHIFT;

    if(J721E_LEO_PMICA_DEVICE == pmic_device_info)
    {
        pmicConfigData.slaveAddr           = J721E_LEO_PMICA_SLAVE_ADDR;
        pmicConfigData.validParams        |= PMIC_CFG_SLAVEADDR_VALID_SHIFT;

        pmicConfigData.qaSlaveAddr         = J721E_LEO_PMICA_WDG_SLAVE_ADDR;
        pmicConfigData.validParams        |= PMIC_CFG_QASLAVEADDR_VALID_SHIFT;
    }
    if(J7VCL_LEO_PMICA_DEVICE == pmic_device_info)
    {
        pmicConfigData.slaveAddr           = J7VCL_LEO_PMICA_SLAVE_ADDR;
        pmicConfigData.validParams        |= PMIC_CFG_SLAVEADDR_VALID_SHIFT;

        pmicConfigData.qaSlaveAddr         = J7VCL_LEO_PMICA_WDG_SLAVE_ADDR;
        pmicConfigData.validParams        |= PMIC_CFG_QASLAVEADDR_VALID_SHIFT;
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

/*!
 * \brief   MISC Unity Test App wrapper Function for LEO PMIC-B
 */
static int32_t test_pmic_leo_pmicB_misc_testApp(void)
{
    int32_t status                = PMIC_ST_SUCCESS;
    Pmic_CoreCfg_t pmicConfigData = {0U};

    /* Fill parameters to pmicConfigData */
    pmicConfigData.pmicDeviceType      = PMIC_DEV_LEO_TPS6594X;
    pmicConfigData.validParams        |= PMIC_CFG_DEVICE_TYPE_VALID_SHIFT;

    pmicConfigData.commMode            = PMIC_INTF_SINGLE_I2C;
    pmicConfigData.validParams        |= PMIC_CFG_COMM_MODE_VALID_SHIFT;

    pmicConfigData.slaveAddr           = J721E_LEO_PMICB_SLAVE_ADDR;
    pmicConfigData.validParams        |= PMIC_CFG_SLAVEADDR_VALID_SHIFT;

    pmicConfigData.qaSlaveAddr         = J721E_LEO_PMICB_WDG_SLAVE_ADDR;
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
 * \brief  MISC  Unity Test App wrapper Function for HERA PMIC
 */
static int32_t test_pmic_hera_misc_testApp(void)
{
    int32_t status                = PMIC_ST_SUCCESS;
    Pmic_CoreCfg_t pmicConfigData = {0U};

    /* Fill parameters to pmicConfigData */
    pmicConfigData.pmicDeviceType      = PMIC_DEV_HERA_LP8764X;
    pmicConfigData.validParams        |= PMIC_CFG_DEVICE_TYPE_VALID_SHIFT;

    pmicConfigData.commMode            = PMIC_INTF_SINGLE_I2C;
    pmicConfigData.validParams        |= PMIC_CFG_COMM_MODE_VALID_SHIFT;

    pmicConfigData.slaveAddr           = J7VCL_HERA_PMIC_SLAVE_ADDR;
    pmicConfigData.validParams        |= PMIC_CFG_SLAVEADDR_VALID_SHIFT;

    pmicConfigData.qaSlaveAddr         = J7VCL_HERA_PMIC_WDG_SLAVE_ADDR;
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

static int32_t setup_pmic_interrupt(uint32_t board)
{
    int32_t status = PMIC_ST_SUCCESS;

    if(J721E_BOARD == board)
    {
        pmic_device_info = J721E_LEO_PMICA_DEVICE;
        status = test_pmic_leo_pmicA_misc_testApp();
        /* Deinit pmic handle */
        if((pPmicCoreHandle != NULL) && (PMIC_ST_SUCCESS == status))
        {
            test_pmic_appDeInit(pPmicCoreHandle);
        }

        if(PMIC_ST_SUCCESS == status)
        {
            pmic_device_info = J721E_LEO_PMICB_DEVICE;
            status = test_pmic_leo_pmicB_misc_testApp();
            /* Deinit pmic handle */
            if((pPmicCoreHandle != NULL) && (PMIC_ST_SUCCESS == status))
            {
                test_pmic_appDeInit(pPmicCoreHandle);
            }
        }
    }
    else if(J7VCL_BOARD == board)
    {
        pmic_device_info = J7VCL_LEO_PMICA_DEVICE;
        status = test_pmic_leo_pmicA_misc_testApp();
        /* Deinit pmic handle */
        if((pPmicCoreHandle != NULL) && (PMIC_ST_SUCCESS == status))
        {
            test_pmic_appDeInit(pPmicCoreHandle);
        }

        if(PMIC_ST_SUCCESS == status)
        {
            pmic_device_info = J7VCL_HERA_PMICB_DEVICE;
            status = test_pmic_hera_misc_testApp();
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

static const char pmicTestAppMenu[] =
{
    " \r\n ================================================================="
    " \r\n Menu:"
    " \r\n ================================================================="
    " \r\n 0: Pmic Leo device(PMIC A on J721E EVM)"
    " \r\n 1: Pmic Leo device(PMIC B on J721E EVM)"
    " \r\n 2: Pmic Leo device(PMIC A on J7VCL EVM)"
    " \r\n 3: Pmic Hera device(PMIC B on J7VCL EVM)"
    " \r\n 4: Manual test: Check recovery count on J721E EVM"
    " \r\n 5: Manual test: Check Enable Interrupt on J721E EVM"
    " \r\n 6: Manual test: Check recovery count on J7VCL EVM"
    " \r\n 7: Manual test: Check Enable Interrupt on J7VCL EVM"
    " \r\n 8: quit"
    " \r\n"
    " \r\n Enter option: "
};

/*!
 * \brief   Function to register MISC Unity Test App wrapper to Unity framework
 */
static void test_pmic_misc_testapp_runner(void)
{
    /* @description : Test runner for misc Test App
     *
     * @requirements: PDK-5827
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
#if defined(SOC_J721E)
                if(PMIC_ST_SUCCESS == setup_pmic_interrupt(J721E_BOARD))
                {
                    /* Run MISC test cases for Leo PMIC-A */
                    pmic_device_info = J721E_LEO_PMICA_DEVICE;
                    /* MISC Unity Test App wrapper Function for LEO PMIC-A */
                    if(PMIC_ST_SUCCESS == test_pmic_leo_pmicA_misc_testApp())
                    {
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
                    pmic_device_info = J721E_LEO_PMICB_DEVICE;
                    /* MISC Unity Test App wrapper Function for LEO PMIC-B */
                    if(PMIC_ST_SUCCESS == test_pmic_leo_pmicB_misc_testApp())
                    {
                        /* Run MISC test cases for Leo PMIC-B */
                        test_pmic_run_slave_testcases();
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
                    /* MISC Unity Test App wrapper Function for LEO PMIC */
                    if(PMIC_ST_SUCCESS == test_pmic_leo_pmicA_misc_testApp())
                    {
                        /* Run MISC test cases for LEO PMIC-A */
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
                    /* MISC Unity Test App wrapper Function for HERA PMIC */
                    if(PMIC_ST_SUCCESS == test_pmic_hera_misc_testApp())
                    {
                        /* Run MISC test cases for HERA PMIC-B */
                        test_pmic_run_slave_testcases();
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
                    /* Run MISC test cases for Leo PMIC-A */
                    pmic_device_info = J721E_LEO_PMICA_DEVICE;
                    /* MISC Unity Test App wrapper Function */
                    if(PMIC_ST_SUCCESS == test_pmic_leo_pmicA_misc_testApp())
                    {
                        /* Run misc manual test cases */
                        test_pmic_run_testcases_recoverycnt();
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
#if defined(SOC_J721E)
                if(PMIC_ST_SUCCESS == setup_pmic_interrupt(J721E_BOARD))
                {
                   /* Run MISC test cases for Leo PMIC-A */
                    pmic_device_info = J721E_LEO_PMICA_DEVICE;
                    /* MISC Unity Test App wrapper Function */
                    if(PMIC_ST_SUCCESS == test_pmic_leo_pmicA_misc_testApp())
                    {
                        /* Display Enable interrupt */
                        startup_type = PMIC_ENABLE_STARTUP_TYPE;
                        /* Run misc manual test cases */
                        test_pmic_run_testcases_EnableInterrupt();
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
#if defined(SOC_J7200)
                if(PMIC_ST_SUCCESS == setup_pmic_interrupt(J7VCL_BOARD))
                {
                    /* Run MISC test cases for Leo PMIC-A */
                    pmic_device_info = J7VCL_LEO_PMICA_DEVICE;
                    /* MISC Unity Test App wrapper Function */
                    if(PMIC_ST_SUCCESS == test_pmic_leo_pmicA_misc_testApp())
                    {
                        /* Run misc manual test cases */
                        test_pmic_run_testcases_recoverycnt();
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
                   /* Run MISC test cases for Leo PMIC-A */
                    pmic_device_info = J7VCL_LEO_PMICA_DEVICE;
                    /* MISC Unity Test App wrapper Function */
                    if(PMIC_ST_SUCCESS == test_pmic_leo_pmicA_misc_testApp())
                    {
                        /* Display Enable interrupt */
                        startup_type = PMIC_ENABLE_STARTUP_TYPE;
                        /* Run misc manual test cases */
                        test_pmic_run_testcases_EnableInterrupt();
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
 * \brief   TI RTOS specific MISC TEST APP main Function
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code.
 *          For valid values see \ref Pmic_ErrorCodes
 */
int main()
{
    Board_initUART();

    pmic_log("PMIC Misc Unity Test Application(%s %s)\n",
                                         __TIME__, __DATE__);

#if defined(UNITY_INCLUDE_CONFIG_V2_H) && \
    (defined(SOC_J721E)             || \
     defined(SOC_J7200))
    test_pmic_misc_testapp_runner();
#endif
}
