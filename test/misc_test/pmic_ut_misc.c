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

static uint8_t pmic_device_info = 0U;

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
        "Pmic_SetRecoveryCntCfg : Parameter validation for 'thrVal'."
    },
    {
        7630,
        "Pmic_SetRecoveryCntCfg : Test Set Clear Recovery Counter."
    },
    {
        7631,
        "Pmic_SetRecoveryCntCfg : Parameter validation for 'clrCnt'."
    },
    {
        7632,
        "Pmic_SetRecoveryCntCfg : Parameter validation for 'handle'."
    },
    {
        7633,
        "Pmic_getRecoveryCntCfg : Parameter validation for 'handle'."
    },
    {
        7634,
        "Pmic_getRecoveryCntCfg : Parameter validation for 'recovCntCfg'."
    },
    {
        7635,
        "Pmic_getRecoveryCnt : Parameter validation for 'handle'."
    },
    {
        7636,
        "Pmic_getRecoveryCnt : Parameter validation for 'recovCntVal'."
    },
    {
        7637,
        "Pmic_getRecoveryCnt : Read Recovery Count Value."
    },
    {
        37,
        "Pmic_fsmRuntimeBistRequest : Test RunTime BIST"
    },
    {
        38,
        "Pmic_fsmRuntimeBistRequest : Parameter validation for 'eventType'."
    },
    {
        39,
        "Pmic_fsmRuntimeBistRequest : Parameter validation for 'handle'."
    },
};

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
 * \brief   Pmic_SetRecoveryCntCfg : Parameter validation for 'thrVal'.
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
 * \brief   Pmic_SetRecoveryCntCfg : Parameter validation for 'clrCnt'.
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
 * \brief   Pmic_SetRecoveryCntCfg : Parameter validation for 'handle'.
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
 * \brief   Pmic_getRecoveryCntCfg : Parameter validation for 'handle'.
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
 * \brief   Pmic_getRecoveryCntCfg : Parameter validation for 'recovCntCfg'.
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
 * \brief   Pmic_getRecoveryCnt : Parameter validation for 'handle'.
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
 * \brief   Pmic_getRecoveryCnt : Parameter validation for 'recovCntVal'.
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
 * \brief   Pmic_fsmRuntimeBistRequest : Test RunTime BIST.
 */
static void test_Pmic_fsmRuntimeBistRequest(void)
{
    int32_t status     = PMIC_ST_SUCCESS;

    test_pmic_print_unity_testcase_info(37,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    status = Pmic_fsmRuntimeBistRequest(pPmicCoreHandle, PMIC_FSM_I2C_TRIGGER1_TYPE);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
}

/*!
 * \brief   Pmic_fsmRuntimeBistRequest : Parameter validation for 'eventType'.
 */
static void test_Pmic_fsmRuntimeBistRequestPrmValTest_eventType(void)
{
    int32_t status     = PMIC_ST_SUCCESS;
    uint8_t  eventType = 0U;

    eventType = PMIC_FSM_I2C_TRIGGER0_TYPE;

    test_pmic_print_unity_testcase_info(38,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    status = Pmic_fsmRuntimeBistRequest(pPmicCoreHandle, eventType);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);
}

/*!
 * \brief   Pmic_fsmRuntimeBistRequest : Parameter validation for 'handle'.
 */
static void test_Pmic_fsmRuntimeBistRequestPrmValTest_handle(void)
{
    int32_t status     = PMIC_ST_SUCCESS;
    uint8_t  eventType = 0U;

    eventType = PMIC_FSM_I2C_TRIGGER1_TYPE;

    test_pmic_print_unity_testcase_info(39,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    status = Pmic_fsmRuntimeBistRequest(NULL, eventType);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, status);
}

#if defined(UNITY_INCLUDE_CONFIG_V2_H) && \
    (defined(SOC_J721E) || defined(SOC_J7200))

/*!
 * \brief   Run misc unity test cases
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
    RUN_TEST(test_Pmic_fsmRuntimeBistRequest);
    RUN_TEST(test_Pmic_fsmRuntimeBistRequestPrmValTest_eventType);
    RUN_TEST(test_Pmic_fsmRuntimeBistRequestPrmValTest_handle);

    UNITY_END();
}

/*!
 * \brief   Run misc manual test cases
 */
static void test_pmic_run_testcases_manual(void)
{
    pmic_log("\n\n%s(): %d: Begin Unity Test Cases...\n", __func__, __LINE__);
    UNITY_BEGIN();
    RUN_TEST(test_Pmic_getRecoveryCnt_read_recovCntVal);

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

/*!
 * \brief  MISC  Unity Test App wrapper Function for manual test
 */
static int32_t test_pmic_misc_manual_testApp(void)
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

static const char pmicTestAppMenu[] =
{
    " \r\n ================================================================="
    " \r\n Menu:"
    " \r\n ================================================================="
    " \r\n 0: Pmic Leo device(PMIC A on J721E EVM)"
    " \r\n 1: Pmic Leo device(PMIC B on J721E EVM)"
    " \r\n 2: Pmic Hera device"
    " \r\n 3: Manual test: Check recovery count"
    " \r\n 4: quit"
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
                /* MISC Unity Test App wrapper Function for LEO PMIC-A */
                if(PMIC_ST_SUCCESS == test_pmic_leo_pmicA_misc_testApp())
                {
                   /* Run MISC test cases for Leo PMIC-A */
                    pmic_device_info = J721E_LEO_PMICA_DEVICE;
                    test_pmic_run_testcases();
                }
                /* Deinit pmic handle */
                if(pPmicCoreHandle != NULL)
                {
                   test_pmic_appDeInit(pPmicCoreHandle);
                }
                break;
            case 1U:
                /* MISC Unity Test App wrapper Function for LEO PMIC-B */
                if(PMIC_ST_SUCCESS == test_pmic_leo_pmicB_misc_testApp())
                {
                    pmic_device_info = J721E_LEO_PMICB_DEVICE;
                   /* Run MISC test cases for Leo PMIC-B */
                   test_pmic_run_testcases();
                }

                /* Deinit pmic handle */
                if(pPmicCoreHandle != NULL)
                {
                   test_pmic_appDeInit(pPmicCoreHandle);
                }
               break;
           case 2U:
               /* MISC Unity Test App wrapper Function for HERA PMIC */
               if(PMIC_ST_SUCCESS == test_pmic_hera_misc_testApp())
               {
                   /* Run misc test cases for Hera PMIC */
                   test_pmic_run_testcases();
               }
               /* Deinit pmic handle */
               if(pPmicCoreHandle != NULL)
               {
                   test_pmic_appDeInit(pPmicCoreHandle);
               }
               break;
           case 3U:
               /* MISC Unity Test App wrapper Function */
               if(PMIC_ST_SUCCESS == test_pmic_misc_manual_testApp())
               {
                   /* Run MISC test cases for Leo PMIC-A */
                    pmic_device_info = J721E_LEO_PMICA_DEVICE;
                   /* Run misc manual test cases */
                   test_pmic_run_testcases_manual();
               }
               /* Deinit pmic handle */
               if(pPmicCoreHandle != NULL)
               {
                   test_pmic_appDeInit(pPmicCoreHandle);
               }
               break;
           case 4U:
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
    test_pmic_misc_testapp_runner();
#endif
}
