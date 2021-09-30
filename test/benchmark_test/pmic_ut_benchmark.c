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
 *  \file   pmic_ut_benchmark.c
 *
 *  \brief  PMIC Unit Test for testing PMIC Benchmark APIs
 *
 */

#include <pmic_ut_benchmark.h>

/* Pointer holds the pPmicCoreHandle */
Pmic_CoreHandle_t *pPmicCoreHandle = NULL;

extern uint16_t pmic_device_info;
extern uint8_t enableBenchMark;
extern int32_t gCrcTestFlag_J721E;
extern int32_t gCrcTestFlag_J7VCL;

/*!
 * \brief   PMIC BenchMark Test Cases
 */
static Pmic_Ut_Tests_t pmic_benchmark_tests[] =
{
    /*! testID
     *  testDesc
     */
    {
        8234,
        "Pmic_wdgStartQaSequence : Profiling PMIC WDG QA API"
    },
    {
        8240,
        "Pmic_init : Profiling PMIC Dual I2C Init API"
    },
    {
        8233,
        "Pmic_init : Profiling PMIC Single I2C Init API"
    },
    {
        10415,
        "Pmic_wdgQaSequenceWriteAnswer : Profiling PMIC WDG Answer Computation"
    },
};

/*!
 * \brief    : Profiling PMIC WDG QA API
 */
static void test_Pmic_wdg_QA_API_profiling(void)
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
    uint64_t t1 = 0U;

    if((gCrcTestFlag_J721E == PMIC_STATUS_CRC_ENABLED)||
       (gCrcTestFlag_J7VCL == PMIC_STATUS_CRC_ENABLED))
    {
        wdgCfg.win1Duration_us = 8350U;
    }

    test_pmic_print_unity_testcase_info(8234,
                                        pmic_benchmark_tests,
                                        PMIC_BENCHMARK_NUM_OF_TESTCASES);
    pmic_log("\n");
    /* Enable WDG Timer */
    pmicStatus = Pmic_wdgEnable(pPmicCoreHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    /* WDG Set parameters */
    pmicStatus = Pmic_wdgSetCfg(pPmicCoreHandle, wdgCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    /* Profiling Start Watchdog QA sequence */
    t1 = print_timeTakenInUsecs(0U, NULL);
    pmicStatus = Pmic_wdgStartQaSequence(pPmicCoreHandle, 1U, maxCnt);
    t1 = print_timeTakenInUsecs(t1, "Pmic_wdgStartQaSequence API");
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    /*  Disable WDG Timer */
    pmicStatus = Pmic_wdgDisable(pPmicCoreHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmic_testResultUpdate_pass(8234,
                               pmic_benchmark_tests,
                               PMIC_BENCHMARK_NUM_OF_TESTCASES);
}

/*!
 * \brief    : Profiling PMIC Dual I2C Init API
 */
static void test_Pmic_init_Dual_i2c_profiling(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    test_pmic_print_unity_testcase_info(8240,
                                        pmic_benchmark_tests,
                                        PMIC_BENCHMARK_NUM_OF_TESTCASES);
    pmic_log("\n");
    /* Deinit pmic handle */
    if(pPmicCoreHandle != NULL)
    {
        test_pmic_appDeInit(pPmicCoreHandle);
    }
    enableBenchMark = true;
    pmicStatus = test_pmic_leo_pmicA_benchmark_testApp();
    enableBenchMark = false;
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmic_testResultUpdate_pass(8240,
                               pmic_benchmark_tests,
                               PMIC_BENCHMARK_NUM_OF_TESTCASES);
}

/*!
 * \brief    : Profiling PMIC Single I2C Init API
 */
static void test_Pmic_init_single_i2c_profiling(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    test_pmic_print_unity_testcase_info(8233,
                                        pmic_benchmark_tests,
                                        PMIC_BENCHMARK_NUM_OF_TESTCASES);
    pmic_log("\n");
    /* Deinit pmic handle */
    if(pPmicCoreHandle != NULL)
    {
        test_pmic_appDeInit(pPmicCoreHandle);
    }
    enableBenchMark = true;
    pmicStatus = test_pmic_leo_pmicA_benchmark_single_i2c_testApp();
    enableBenchMark = false;
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmic_testResultUpdate_pass(8233,
                               pmic_benchmark_tests,
                               PMIC_BENCHMARK_NUM_OF_TESTCASES);
}

/*!
 * \brief Pmic_wdgQaSequenceWriteAnswer : Profiling PMIC WDG Answer Computation
 */
static void test_Pmic_wdg_answer_computation(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint64_t delta = 0;
    int8_t  ansIndex   = 0;
    int32_t test_failure = PMIC_ST_SUCCESS;
    uint32_t numSequences = 1U, sequenceId;
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
        PMIC_WDG_RESET_DISABLE,
        PMIC_WDG_RETLONGWIN_DISABLE,
        PMIC_WDG_QA_FEEDBACK_VALUE_0,
        PMIC_WDG_QA_LFSR_VALUE_0,
        PMIC_WDG_QA_QUES_SEED_VALUE_10,
    };
    uint64_t t1 = 0U;

    if((gCrcTestFlag_J721E == PMIC_STATUS_CRC_ENABLED)||
       (gCrcTestFlag_J7VCL == PMIC_STATUS_CRC_ENABLED))
    {
        wdgCfg.win1Duration_us = 8350U;
    }

    test_pmic_print_unity_testcase_info(10415,
                                        pmic_benchmark_tests,
                                        PMIC_BENCHMARK_NUM_OF_TESTCASES);

    if(PMIC_SILICON_REV_ID_PG_2_0 == pPmicCoreHandle->pmicDevSiliconRev)
    {
        wdgCfg.longWinDuration_ms = 752000U;
        wdgCfg.win1Duration_us    = 7150U;
    }

     pmic_log("\n");
    /* Enable WDG Timer */
    pmicStatus = Pmic_wdgEnable(pPmicCoreHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    /* WDG Set parameters */
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
            if(ansIndex == 3)
            {
                /* Profiling WDG Answer Computation */
                t1 = print_timeTakenInUsecs(0U, NULL);
                pmicStatus = Pmic_wdgQaSequenceWriteAnswer(pPmicCoreHandle);
                delta = print_timeTakenInUsecs(t1, NULL);
                TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
            }
            else
            {
                pmicStatus = Pmic_wdgQaSequenceWriteAnswer(pPmicCoreHandle);
                TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
            }
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

    pmic_log("--------------------------------------\n");
    pmic_log("Time taken for %50s: %6d usec\n",
                "Pmic_wdgQaSequenceWriteAnswer API for single instance",
                (uint32_t)delta);
    pmic_log("--------------------------------------\n");


    pmic_testResultUpdate_pass(10415,
                               pmic_benchmark_tests,
                               PMIC_BENCHMARK_NUM_OF_TESTCASES);
}

#if defined(UNITY_INCLUDE_CONFIG_V2_H) && \
    (defined(SOC_J721E) || defined(SOC_J7200))
/*!
 * \brief   Run benchmark unity test cases
 */
static void test_pmic_run_testcases(void)
{
    pmic_log("\n\n%s(): %d: Begin Unity Test Cases...\n", __func__, __LINE__);
    UNITY_BEGIN();

    pmic_testResult_init(pmic_benchmark_tests,
                         PMIC_BENCHMARK_NUM_OF_TESTCASES);

    RUN_TEST(test_Pmic_wdg_QA_API_profiling);
    RUN_TEST(test_Pmic_init_Dual_i2c_profiling);
    RUN_TEST(test_Pmic_init_single_i2c_profiling);
    RUN_TEST(test_Pmic_wdg_answer_computation);

    pmic_updateTestResults(pmic_benchmark_tests,
                           PMIC_BENCHMARK_NUM_OF_TESTCASES);

    UNITY_END();
}

/*!
 * \brief   BenchMark Unity Test App wrapper Function for LEO PMIC-A
 */
static int32_t test_pmic_leo_pmicA_benchmark_testApp(void)
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

/*!
 * \brief   BenchMark Unity Test App wrapper Function for LEO PMIC-A
 */
static int32_t test_pmic_leo_pmicA_benchmark_single_i2c_testApp(void)
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

/*!
 * \brief   BenchMark Unity Test App wrapper Function for LEO PMIC-B
 */
static int32_t test_pmic_leo_pmicB_benchmark_testApp(void)
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

    pmicConfigData.slaveAddr           = J721E_LEO_PMICB_SLAVE_ADDR;
    pmicConfigData.validParams        |= PMIC_CFG_SLAVEADDR_VALID_SHIFT;

    pmicConfigData.qaSlaveAddr         = J721E_LEO_PMICB_WDG_SLAVE_ADDR;
    pmicConfigData.validParams        |= PMIC_CFG_QASLAVEADDR_VALID_SHIFT;

    pmicConfigData.nvmSlaveAddr        = J721E_LEO_PMICB_PAGE1_SLAVE_ADDR;
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
 * \brief  BenchMark  Unity Test App wrapper Function for HERA PMIC
 */
static int32_t test_pmic_hera_benchmark_testApp(void)
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
 * \brief   BenchMark Unity Test App wrapper Function for LEO PMIC-A
 */
static int32_t test_pmic_leo_pmicA_spiStub_benchmark_testApp(void)
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
        status = test_pmic_leo_pmicA_benchmark_testApp();
        /* Deinit pmic handle */
        if((pPmicCoreHandle != NULL) && (PMIC_ST_SUCCESS == status))
        {
            test_pmic_appDeInit(pPmicCoreHandle);
        }

        if(PMIC_ST_SUCCESS == status)
        {
            pmic_device_info = J721E_LEO_PMICB_DEVICE;
            status = test_pmic_leo_pmicB_benchmark_testApp();
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
        status = test_pmic_leo_pmicA_benchmark_testApp();
        /* Deinit pmic handle */
        if((pPmicCoreHandle != NULL) && (PMIC_ST_SUCCESS == status))
        {
            test_pmic_appDeInit(pPmicCoreHandle);
        }

        if(PMIC_ST_SUCCESS == status)
        {
            pmic_device_info = J7VCL_HERA_PMICB_DEVICE;
            status = test_pmic_hera_benchmark_testApp();
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
    " \r\n 0: Automatic run for all board specific BenchMark options"
    " \r\n 1: Manual run for BenchMark options"
    " \r\n 2: quit"
    " \r\n"
    " \r\n Enter option: "
};

volatile static const char pmicTestAppMenu[] =
{
    " \r\n ================================================================="
    " \r\n Menu:"
    " \r\n ================================================================="
    " \r\n 0: Pmic Leo device(PMIC A on J721E EVM)"
    " \r\n 1: Pmic Leo device(PMIC A on J7VCL EVM)"
    " \r\n 2: Pmic Leo device(PMIC A on J721E EVM Using SPI Stub Functions)"
    " \r\n 3: Pmic Leo device(PMIC A on J7VCL EVM Using SPI Stub Functions)"
    " \r\n 4: Back to Test Menu"
    " \r\n"
    " \r\n Enter option: "
};

volatile int8_t g_option = 0;
static void test_pmic_benchmark_testapp_run_options()
{
    int8_t num = -1;
    int8_t idx = 0;
#if defined(SOC_J721E)
    int8_t automatic_options[] = {0, 2};
#elif defined(SOC_J7200)
    int8_t automatic_options[] = {1, 3};
#endif

    while(1U)
    {
        if(idx >= (sizeof(automatic_options)/sizeof(automatic_options[0])))
        {
            pmic_printTestResult(pmic_benchmark_tests, PMIC_BENCHMARK_NUM_OF_TESTCASES);
        }
        pmic_log("%s", pmicTestAppMenu);
        if(g_option == PMIC_UT_AUTOMATE_OPTION)
        {
            if(idx < (sizeof(automatic_options)/sizeof(automatic_options[0])))
            {
                num = automatic_options[idx++];
            }
            else
            {
                num = 4;
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

                    /* BenchMark Unity Test App wrapper Function for LEO PMIC-A */
                    if(PMIC_ST_SUCCESS == test_pmic_leo_pmicA_benchmark_testApp())
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
#if defined(SOC_J7200)
                if(PMIC_ST_SUCCESS == setup_pmic_interrupt(J721E_BOARD))
                {
                    pmic_device_info = J7VCL_LEO_PMICA_DEVICE;

                    /* BenchMark Unity Test App wrapper Function for LEO PMIC-A */
                    if(PMIC_ST_SUCCESS == test_pmic_leo_pmicA_benchmark_testApp())
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
            case 2U:
#if defined(SOC_J721E)
                if(PMIC_ST_SUCCESS == setup_pmic_interrupt(J721E_BOARD))
                {
                    pmic_device_info = J721E_LEO_PMICA_DEVICE;

                    /* BenchMark Unity Test App wrapper Function for LEO PMIC-A
                     * using SPI stub functions */
                    if(PMIC_ST_SUCCESS ==
                           test_pmic_leo_pmicA_spiStub_benchmark_testApp())
                    {
                        /* Run benchmark test cases for Leo PMIC-A */
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
                    pmic_device_info = J7VCL_LEO_PMICA_DEVICE;
                    /* BenchMark Unity Test App wrapper Function for LEO PMIC-A
                     * using SPI stub functions */
                     if(PMIC_ST_SUCCESS ==
                            test_pmic_leo_pmicA_spiStub_benchmark_testApp())
                    {
                        /* Run benchmark test cases for Leo PMIC-A */
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
            case 4U:
               pmic_log(" \r\n Back to Test Menu options\n");
               return;
            default:
               pmic_log(" \r\n Invalid option... Try Again!!!\n");
               break;
        }
    }
}

/*!
 * \brief   Function to register BenchMark Unity Test App wrapper
 *          to Unity framework
 */
static void test_pmic_benchmark_testapp_runner(void)
{
    /* @description : Test runner for benchmark Test App
     *
     * @requirements: XXXX
     *
     * @cores       : mcu1_0, mcu1_1
     */

    while(1U)
    {
        pmic_log("%s", pmicTestMenu);
        if(UART_scanFmt("%d", &g_option) != 0U)
        {
            pmic_log("Read from UART Console failed\n");
            return;
        }

        switch(g_option)
        {
            case PMIC_UT_AUTOMATE_OPTION:
            case PMIC_UT_MANUAL_OPTION:
               test_pmic_benchmark_testapp_run_options();
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
 * \brief   TI RTOS specific BenchMark TEST APP main Function
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code.
 *          For valid values see \ref Pmic_ErrorCodes
 */
int main()
{
    Board_initUART();

    pmic_print_banner("PMIC BenchMark Unity Test Application");

#if defined(UNITY_INCLUDE_CONFIG_V2_H) && \
    (defined(SOC_J721E)             || \
     defined(SOC_J7200))
    test_pmic_benchmark_testapp_runner();
#endif
}
