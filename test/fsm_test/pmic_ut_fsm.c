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
 *  \file   pmic_ut_fsm.c
 *
 *  \brief  PMIC Unit Test for testing PMIC FSM APIs
 *
 */

#include <pmic_ut_fsm.h>

/* Pointer holds the pPmicCoreHandle */
Pmic_CoreHandle_t *pPmicCoreHandle = NULL;

static uint16_t pmic_device_info = 0U;

/*!
 * \brief   PMIC FSM Test Cases
 */
static Pmic_Ut_Tests_t pmic_fsm_tests[] =
{
    /*! testID
     *  testDesc
     */
    {
        7693,
        "Pmic_fsmSetNsleepSignalMask : Test Mask Nsleep1."
    },
    {
        7694,
        "Pmic_fsmSetNsleepSignalMask : Test Unmask Nsleep1."
    },
    {
        7695,
        "Pmic_fsmSetNsleepSignalMask : Test Mask Nsleep2."
    },
    {
        7696,
        "Pmic_fsmSetNsleepSignalMask : Test Unmask Nsleep2."
    },
    {
        7697,
        "Pmic_fsmSetMissionState : Test Set State to MCU."
    },
    {
        7698,
        "Pmic_fsmSetMissionState : Test Set State to Active."
    },
    {
        7699,
        "Pmic_fsmSetMissionState : Test Set State to S2R/Deep Sleep."
    },
    {
        7700,
        "Pmic_fsmSetMissionState : Manual Test Set State to LP-Standby."
    },
    {
        7701,
        "Pmic_fsmSetMissionState : Manual Test Set State to Standby."
    },
    {
        7702,
        "Pmic_fsmSetMissionState : Parameter range validation for handle."
    },
    {
        7703,
        "Pmic_fsmSetMissionState : Parameter range validation for state."
    },
    {
        7704,
        "Pmic_fsmSetNsleepSignalMask : Parameter range validation for handle."
    },
    {
        7360,
        "Pmic_fsmDeviceOffRequestCfg :  Parameter validation for handle"
    },
    {
        7361,
        "Pmic_fsmDeviceOffRequestCfg :  Parameter validation for eventType"
    },
    {
        7705,
        "Pmic_fsmDeviceOffRequestCfg :  Parameter validation for fsmState"
    },
    {
        7706,
        "Pmic_fsmRuntimeBistRequest : Parameter validation for handle."
    },
    {
        7364,
        "Pmic_fsmDeviceOnRequest : Parameter validation for handle."
    }
};

/*!
 * \brief   Pmic_fsmSetNsleepSignalMask : Test Mask Nsleep1.
 */
static void test_Pmic_fsmSetNsleepSignalMask_mask_nsleep1(void)
{
    int32_t status     = PMIC_ST_SUCCESS;
    bool    nsleepType = 0U;
    bool    maskEnable = 0U;

    nsleepType = PMIC_NSLEEP1_SIGNAL;
    maskEnable = PMIC_NSLEEPX_MASK;

    test_pmic_print_unity_testcase_info(7693,
                                        pmic_fsm_tests,
                                        PMIC_FSM_NUM_OF_TESTCASES);

    status = Pmic_fsmSetNsleepSignalMask(pPmicCoreHandle, nsleepType, maskEnable);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    pmic_testResultUpdate_pass(7693,
                               pmic_fsm_tests,
                               PMIC_FSM_NUM_OF_TESTCASES);

}

/*!
 * \brief   Pmic_fsmSetNsleepSignalMask : Test Unmask Nsleep1.
 */
static void test_Pmic_fsmSetNsleepSignalMask_unmask_nsleep1(void)
{
    int32_t status     = PMIC_ST_SUCCESS;
    bool    nsleepType = 0U;
    bool    maskEnable = 0U;;

    nsleepType = PMIC_NSLEEP1_SIGNAL;
    maskEnable = PMIC_NSLEEPX_UNMASK;

    test_pmic_print_unity_testcase_info(7694,
                                        pmic_fsm_tests,
                                        PMIC_FSM_NUM_OF_TESTCASES);

    status = Pmic_fsmSetNsleepSignalMask(pPmicCoreHandle, nsleepType, maskEnable);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    pmic_testResultUpdate_pass(7694,
                               pmic_fsm_tests,
                               PMIC_FSM_NUM_OF_TESTCASES);

}

/*!
 * \brief   Pmic_fsmSetNsleepSignalMask : Test Mask Nsleep2.
 */
static void test_Pmic_fsmSetNsleepSignalMask_mask_nsleep2(void)
{
    int32_t status     = PMIC_ST_SUCCESS;
    bool    nsleepType = 0U;
    bool    maskEnable = 0U;

    nsleepType = PMIC_NSLEEP2_SIGNAL;
    maskEnable = PMIC_NSLEEPX_MASK;

    test_pmic_print_unity_testcase_info(7695,
                                        pmic_fsm_tests,
                                        PMIC_FSM_NUM_OF_TESTCASES);

    status = Pmic_fsmSetNsleepSignalMask(pPmicCoreHandle, nsleepType, maskEnable);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    pmic_testResultUpdate_pass(7695,
                               pmic_fsm_tests,
                               PMIC_FSM_NUM_OF_TESTCASES);

}

/*!
 * \brief   Pmic_fsmSetNsleepSignalMask : Test Unmask Nsleep2.
 */
static void test_Pmic_fsmSetNsleepSignalMask_unmask_nsleep2(void)
{
    int32_t status     = PMIC_ST_SUCCESS;
    bool    nsleepType = 0U;
    bool    maskEnable = 0U;

    nsleepType = PMIC_NSLEEP2_SIGNAL;
    maskEnable = PMIC_NSLEEPX_UNMASK;

    test_pmic_print_unity_testcase_info(7696,
                                        pmic_fsm_tests,
                                        PMIC_FSM_NUM_OF_TESTCASES);

    status = Pmic_fsmSetNsleepSignalMask(pPmicCoreHandle, nsleepType, maskEnable);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    pmic_testResultUpdate_pass(7696,
                               pmic_fsm_tests,
                               PMIC_FSM_NUM_OF_TESTCASES);

}

/*!
 * \brief   Pmic_fsmSetMissionState : Test Set State to MCU.
 */
static void test_Pmic_fsmSetMissionState_mcu(void)
{
    int32_t status     = PMIC_ST_SUCCESS;
    uint8_t  pmicState = 0U;

    pmicState = PMIC_FSM_MCU_ONLY_STATE;

    test_pmic_print_unity_testcase_info(7697,
                                        pmic_fsm_tests,
                                        PMIC_FSM_NUM_OF_TESTCASES);

    status = Pmic_fsmSetNsleepSignalMask(pPmicCoreHandle,
                                         PMIC_NSLEEP1_SIGNAL,
                                         PMIC_NSLEEPX_UNMASK);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_fsmSetNsleepSignalMask(pPmicCoreHandle,
                                         PMIC_NSLEEP2_SIGNAL,
                                         PMIC_NSLEEPX_UNMASK);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
#if defined(SOC_J721E)
    pmic_log("\r\n Probe TP134 and it should change from High to Low.");
    pmic_log("\r\n Probe TP133 and it should continue to be in HIGH");
#endif
#if defined(SOC_J7200)
    pmic_log("\r\n Probe TP46 and it should change from High to Low.");
    pmic_log("\r\n Probe TP29 and it should continue to be in HIGH");
#endif
    status = Pmic_fsmSetMissionState(pPmicCoreHandle, pmicState);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
}

/*!
 * \brief   Pmic_fsmSetMissionState : Test Set State to Active.
 */
static void test_Pmic_fsmSetMissionState_active(void)
{
    int32_t status     = PMIC_ST_SUCCESS;
    uint8_t pmicState = 0U;

    pmicState = PMIC_FSM_ACTIVE_STATE;

    test_pmic_print_unity_testcase_info(7698,
                                        pmic_fsm_tests,
                                        PMIC_FSM_NUM_OF_TESTCASES);

    status = Pmic_fsmSetNsleepSignalMask(pPmicCoreHandle,
                                         PMIC_NSLEEP1_SIGNAL,
                                         PMIC_NSLEEPX_UNMASK);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_fsmSetNsleepSignalMask(pPmicCoreHandle,
                                         PMIC_NSLEEP2_SIGNAL,
                                         PMIC_NSLEEPX_UNMASK);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_fsmSetMissionState(pPmicCoreHandle, pmicState);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    pmic_testResultUpdate_pass(7698,
                               pmic_fsm_tests,
                               PMIC_FSM_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_fsmSetMissionState : Test Set State to S2R/Deep Sleep.
 */
static void test_Pmic_fsmSetMissionState_s2r(void)
{
    int32_t status     = PMIC_ST_SUCCESS;
    uint8_t  pmicState = 0U;

    pmicState = PMIC_FSM_S2R_STATE;

    test_pmic_print_unity_testcase_info(7699,
                                        pmic_fsm_tests,
                                        PMIC_FSM_NUM_OF_TESTCASES);

    status = Pmic_fsmSetNsleepSignalMask(pPmicCoreHandle,
                                         PMIC_NSLEEP1_SIGNAL,
                                         PMIC_NSLEEPX_UNMASK);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_fsmSetNsleepSignalMask(pPmicCoreHandle,
                                         PMIC_NSLEEP2_SIGNAL,
                                         PMIC_NSLEEPX_UNMASK);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);


#if defined(SOC_J721E)
     pmic_log("\r\n Probe TP134 and TP133 and it should change from High to Low.");
#endif
#if defined(SOC_J7200)
     pmic_log("\r\n Probe TP46 and TP29 and it should change from High to Low.");
#endif
    status = Pmic_fsmSetMissionState(pPmicCoreHandle, pmicState);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
}

/*!
 * \brief   Pmic_fsmSetMissionState : Manual Test Set State to LP-Standby.
 */
static void test_Pmic_fsmSetMissionState_lpstandby(void)
{
    int32_t status         = PMIC_ST_SUCCESS;
    uint8_t  pmicState     = 0U;

    pmicState = PMIC_FSM_LP_STANBY_STATE;

    test_pmic_print_unity_testcase_info(7700,
                                        pmic_fsm_tests,
                                        PMIC_FSM_NUM_OF_TESTCASES);

    status = Pmic_fsmSetNsleepSignalMask(pPmicCoreHandle,
                                         PMIC_NSLEEP1_SIGNAL,
                                         PMIC_NSLEEPX_MASK);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_fsmSetNsleepSignalMask(pPmicCoreHandle,
                                         PMIC_NSLEEP2_SIGNAL,
                                         PMIC_NSLEEPX_MASK);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

#if defined(SOC_J721E)
     pmic_log("\r\n Probe TP134 and TP133 and it should change from High to Low.");
#endif
#if defined(SOC_J7200)
     pmic_log("\r\n Probe TP46 and TP29 and it should change from High to Low.");
#endif

    status = Pmic_fsmSetMissionState(pPmicCoreHandle, pmicState);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
}

/*!
 * \brief   Pmic_fsmSetMissionState : Manual Test Set State to Standby.
 */
static void test_Pmic_fsmSetMissionState_standby(void)
{
    int32_t status         = PMIC_ST_SUCCESS;
    uint8_t  pmicState     = 0U;

    pmicState = PMIC_FSM_STANBY_STATE;

    test_pmic_print_unity_testcase_info(7701,
                                        pmic_fsm_tests,
                                        PMIC_FSM_NUM_OF_TESTCASES);

    status = Pmic_fsmSetNsleepSignalMask(pPmicCoreHandle,
                                         PMIC_NSLEEP1_SIGNAL,
                                         PMIC_NSLEEPX_MASK);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_fsmSetNsleepSignalMask(pPmicCoreHandle,
                                         PMIC_NSLEEP2_SIGNAL,
                                         PMIC_NSLEEPX_MASK);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    Osal_delay(5U);

#if defined(SOC_J721E)
     pmic_log("\r\n Probe TP134 and TP133 and it should change from High to Low.");
#endif
#if defined(SOC_J7200)
     pmic_log("\r\n Probe TP46 and TP29 and it should change from High to Low.");
#endif
    status = Pmic_fsmSetMissionState(pPmicCoreHandle, pmicState);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
}

/*!
 * \brief   Pmic_fsmSetMissionState : Parameter range validation for handle.
 */
static void test_Pmic_fsmSetMissionStatePrmValTest_handle(void)
{
    int32_t status     = PMIC_ST_SUCCESS;
    uint8_t  pmicState = 0U;

    pmicState = PMIC_FSM_S2R_STATE;

    test_pmic_print_unity_testcase_info(7702,
                                        pmic_fsm_tests,
                                        PMIC_FSM_NUM_OF_TESTCASES);

    status = Pmic_fsmSetMissionState(NULL, pmicState);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, status);

    pmic_testResultUpdate_pass(7702,
                               pmic_fsm_tests,
                               PMIC_FSM_NUM_OF_TESTCASES);

}

/*!
 * \brief   Pmic_fsmSetMissionState : Parameter range validation for state.
 */
static void test_Pmic_fsmSetMissionStatePrmValTest_state(void)
{
    int32_t status     = PMIC_ST_SUCCESS;
    uint8_t  pmicState = 0U;

    pmicState = PMIC_FSM_STATE_MAX + 1;

    test_pmic_print_unity_testcase_info(7703,
                                        pmic_fsm_tests,
                                        PMIC_FSM_NUM_OF_TESTCASES);

    status = Pmic_fsmSetMissionState(pPmicCoreHandle, pmicState);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

    pmic_testResultUpdate_pass(7703,
                               pmic_fsm_tests,
                               PMIC_FSM_NUM_OF_TESTCASES);

}

/*!
 * \brief   Pmic_fsmSetNsleepSignalMask : Parameter range validation for handle.
 */
static void test_Pmic_fsmSetNsleepSignalMaskStatePrmValTest_handle(void)
{
    int32_t status     = PMIC_ST_SUCCESS;
    bool    nsleepType = 0U;
    bool    maskEnable = 0U;

    nsleepType = PMIC_NSLEEP1_SIGNAL;
    maskEnable = PMIC_NSLEEPX_MASK;

    test_pmic_print_unity_testcase_info(7704,
                                        pmic_fsm_tests,
                                        PMIC_FSM_NUM_OF_TESTCASES);

    status = Pmic_fsmSetNsleepSignalMask(NULL, nsleepType, maskEnable);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, status);

    pmic_testResultUpdate_pass(7704,
                               pmic_fsm_tests,
                               PMIC_FSM_NUM_OF_TESTCASES);

}

/*!
 * \brief   Parameter validation for handle
 */
static void test_pmic_rtc_fsmDevOffReqCfg_PrmValTest_handle(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    bool standByState = PMIC_FSM_STANBY_STATE;

    test_pmic_print_unity_testcase_info(7360,
                                        pmic_fsm_tests,
                                        PMIC_FSM_NUM_OF_TESTCASES);

    status = Pmic_fsmDeviceOffRequestCfg(NULL,
                                         PMIC_FSM_I2C_TRIGGER0_TYPE,
                                         standByState);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, status);

    pmic_testResultUpdate_pass(7360,
                               pmic_fsm_tests,
                               PMIC_FSM_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_fsmDeviceOffRequestCfg : Parameter validation for eventType
 */
static void test_pmic_rtc_fsmDevOffReqCfg_PrmValTest_eventType(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t fsmState ;
    uint8_t evenType = PMIC_FSM_I2C_TRIGGER0_TYPE;

    fsmState = PMIC_FSM_NPWRON_PIN_TYPE + 0x1;
    test_pmic_print_unity_testcase_info(7361,
                                        pmic_fsm_tests,
                                        PMIC_FSM_NUM_OF_TESTCASES);

    status = Pmic_fsmDeviceOffRequestCfg(pPmicCoreHandle,
                                         evenType,
                                         fsmState);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

    pmic_testResultUpdate_pass(7361,
                               pmic_fsm_tests,
                               PMIC_FSM_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_fsmDeviceOffRequestCfg :  Parameter validation for fsmState
 */
static void test_pmic_rtc_fsmDevOffReqCfg_PrmValTest_fsmState(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t fsmState = 0U;
    uint8_t evenType = 1U;

    fsmState = PMIC_FSM_LP_STANBY_STATE + 1;
    test_pmic_print_unity_testcase_info(7705,
                                        pmic_fsm_tests,
                                        PMIC_FSM_NUM_OF_TESTCASES);

    status = Pmic_fsmDeviceOffRequestCfg(pPmicCoreHandle,
                                         evenType,
                                         fsmState);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

    pmic_testResultUpdate_pass(7705,
                               pmic_fsm_tests,
                               PMIC_FSM_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_fsmRuntimeBistRequest : Parameter validation for handle.
 */
static void test_Pmic_fsmRuntimeBistRequestPrmValTest_handle(void)
{
    int32_t status     = PMIC_ST_SUCCESS;

    test_pmic_print_unity_testcase_info(7706,
                                        pmic_fsm_tests,
                                        PMIC_FSM_NUM_OF_TESTCASES);

    status = Pmic_fsmRequestRuntimeBist(NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, status);

    pmic_testResultUpdate_pass(7706,
                               pmic_fsm_tests,
                               PMIC_FSM_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_fsmDeviceOnRequest : Parameter validation for handle.
 */
static void test_Pmic_Pmic_fsmDeviceOnRequestPrmValTest_handle(void)
{
    int32_t status     = PMIC_ST_SUCCESS;

    test_pmic_print_unity_testcase_info(7364,
                                        pmic_fsm_tests,
                                        PMIC_FSM_NUM_OF_TESTCASES);

    status = Pmic_fsmDeviceOnRequest(NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, status);

    pmic_testResultUpdate_pass(7364,
                               pmic_fsm_tests,
                               PMIC_FSM_NUM_OF_TESTCASES);
}

#if defined(UNITY_INCLUDE_CONFIG_V2_H) && \
    (defined(SOC_J721E) || defined(SOC_J7200))

/*!
 * \brief   Run fsm unity test cases
 */
static void test_pmic_run_testcases(void)
{
    pmic_log("\n\n%s(): %d: Begin Unity Test Cases...\n", __func__, __LINE__);
    UNITY_BEGIN();

    pmic_testResult_init(pmic_fsm_tests, PMIC_FSM_NUM_OF_TESTCASES);

    RUN_TEST(test_Pmic_fsmSetNsleepSignalMask_mask_nsleep1);
    RUN_TEST(test_Pmic_fsmSetNsleepSignalMask_unmask_nsleep1);
    RUN_TEST(test_Pmic_fsmSetNsleepSignalMask_mask_nsleep2);
    RUN_TEST(test_Pmic_fsmSetNsleepSignalMask_unmask_nsleep2);
    RUN_TEST(test_Pmic_fsmSetMissionState_active);
    RUN_TEST(test_Pmic_fsmSetMissionStatePrmValTest_handle);
    RUN_TEST(test_Pmic_fsmSetMissionStatePrmValTest_state);
    RUN_TEST(test_Pmic_fsmSetNsleepSignalMaskStatePrmValTest_handle);
    RUN_TEST(test_pmic_rtc_fsmDevOffReqCfg_PrmValTest_handle);
    RUN_TEST(test_pmic_rtc_fsmDevOffReqCfg_PrmValTest_eventType);
    RUN_TEST(test_pmic_rtc_fsmDevOffReqCfg_PrmValTest_fsmState);
    RUN_TEST(test_Pmic_fsmRuntimeBistRequestPrmValTest_handle);
    RUN_TEST(test_Pmic_Pmic_fsmDeviceOnRequestPrmValTest_handle);

    pmic_printTestResult(pmic_fsm_tests, PMIC_FSM_NUM_OF_TESTCASES);

    UNITY_END();
}

/*!
 * \brief   FSM Unity Test App wrapper Function for LEO PMIC-A
 */
static int32_t test_pmic_leo_pmicA_fsm_testApp(void)
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
 * \brief   FSM Unity Test App wrapper Function for LEO PMIC-B
 */
static int32_t test_pmic_leo_pmicB_fsm_testApp(void)
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
 * \brief  FSM  Unity Test App wrapper Function for HERA PMIC
 */
static int32_t test_pmic_hera_fsm_testApp(void)
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
        status = test_pmic_leo_pmicA_fsm_testApp();
        /* Deinit pmic handle */
        if((pPmicCoreHandle != NULL) && (PMIC_ST_SUCCESS == status))
        {
            test_pmic_appDeInit(pPmicCoreHandle);
        }

        if(PMIC_ST_SUCCESS == status)
        {
            pmic_device_info = J721E_LEO_PMICB_DEVICE;
            status = test_pmic_leo_pmicB_fsm_testApp();
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
        status = test_pmic_leo_pmicA_fsm_testApp();
        /* Deinit pmic handle */
        if((pPmicCoreHandle != NULL) && (PMIC_ST_SUCCESS == status))
        {
            test_pmic_appDeInit(pPmicCoreHandle);
        }

        if(PMIC_ST_SUCCESS == status)
        {
            pmic_device_info = J7VCL_HERA_PMICB_DEVICE;
            status = test_pmic_hera_fsm_testApp();
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
    " \r\n 1: Pmic Leo device(PMIC A on J7VCL EVM)"
    " \r\n 2: Pmic Hera device(PMIC A on J7VCL EVM)"
    " \r\n 3: Pmic Leo device(PMIC A on J721E EVM Manual Testcase for FSM states)"
    " \r\n 4: Pmic Leo device(PMIC A on J7VCL EVM Manual Testcase for FSM states)"
    " \r\n 5: quit"
    " \r\n"
    " \r\n Enter option: "
};

static void print_pmicTestAppManualTestMenu(uint32_t board)
{
    char board_name[10] = {0};

    if(J721E_BOARD == board)
    {
        strcpy(board_name, "J721E");
    }
    else if(J7VCL_BOARD == board)
    {
        strcpy(board_name, "J7VCL");
    }

    pmic_log(" \r\n =================================================================");
    pmic_log(" \r\n Manual Testcase Menu:");
    pmic_log(" \r\n =================================================================");
    pmic_log(" \r\n 0: Pmic Leo device(PMIC A on %s EVM Set FSM Mission States - MCU", board_name);
    pmic_log(" \r\n 1: Pmic Leo device(PMIC A on %s EVM Set FSM Mission States - S2R", board_name);
    pmic_log(" \r\n 2: Pmic Leo device(PMIC A on %s EVM Set FSM Mission States - lpStandby", board_name);
    pmic_log(" \r\n 3: Pmic Leo device(PMIC A on %s EVM Set FSM Mission States - Standby", board_name);
    pmic_log(" \r\n 4: Back to Main Menu");
    pmic_log(" \r\n");
    pmic_log(" \r\n Enter option: ");
};

/*!
 * \brief   Run FSM manual test cases
 */
static void test_pmic_run_testcases_manual(uint32_t board)
{
    int8_t menuOption = -1;

    while(1U)
    {
        print_pmicTestAppManualTestMenu(board);
        if(UART_scanFmt("%d", &menuOption) != 0U)
        {
            pmic_log("Read from UART Console failed\n");
            return;
        }

        if(menuOption == 4)
        {
            break;
        }   

        switch(menuOption)
        {
            case 0U:
                RUN_TEST(test_Pmic_fsmSetMissionState_mcu);
               break;
            case 1U:
                RUN_TEST(test_Pmic_fsmSetMissionState_s2r);
               break;
            case 2U:
                RUN_TEST(test_Pmic_fsmSetMissionState_lpstandby);
               break;
            case 3U:
                RUN_TEST(test_Pmic_fsmSetMissionState_standby);
               break;
            default:
               pmic_log(" \r\n Invalid option... Try Again!!!\n");
               break;
        }
    }
}

/*!
 * \brief   Function to register FSM Unity Test App wrapper to Unity framework
 */
static void test_pmic_fsm_testapp_runner(void)
{
    /* @description : Test runner for fsm Test App
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
#if defined(SOC_J721E)
                if(PMIC_ST_SUCCESS == setup_pmic_interrupt(J721E_BOARD))
                {
                    pmic_device_info = J721E_LEO_PMICA_DEVICE;
                    /* FSM Unity Test App wrapper Function for LEO PMIC-A */
                    if(PMIC_ST_SUCCESS == test_pmic_leo_pmicA_fsm_testApp())
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
                if(PMIC_ST_SUCCESS == setup_pmic_interrupt(J7VCL_BOARD))
                {
                    pmic_device_info = J7VCL_LEO_PMICA_DEVICE;
                    /* FSM Unity Test App wrapper Function for LEO PMIC-A */
                    if(PMIC_ST_SUCCESS == test_pmic_leo_pmicA_fsm_testApp())
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
#if defined(SOC_J7200)
                if(PMIC_ST_SUCCESS == setup_pmic_interrupt(J7VCL_BOARD))
                {
                    pmic_device_info = J7VCL_HERA_PMICB_DEVICE;
                    /* FSM Unity Test App wrapper Function for HERA PMIC */
                   if(PMIC_ST_SUCCESS == test_pmic_hera_fsm_testApp())
                   {
                       /* Run fsm test cases for Hera PMIC */
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
#if defined(SOC_J721E)
                if(PMIC_ST_SUCCESS == setup_pmic_interrupt(J721E_BOARD))
                {
                    pmic_device_info = J721E_LEO_PMICA_DEVICE;
                    /* FSM Manual Test App wrapper Function for LEO PMIC-A*/
                    if(PMIC_ST_SUCCESS == test_pmic_leo_pmicA_fsm_testApp())
                    {
                       /* Run fsm manual test cases */
                       test_pmic_run_testcases_manual(J721E_BOARD);
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
#if defined(SOC_J7200)
                if(PMIC_ST_SUCCESS == setup_pmic_interrupt(J7VCL_BOARD))
                {
                    pmic_device_info = J7VCL_LEO_PMICA_DEVICE;
                    /* FSM Manual Test App wrapper Function for LEO PMIC-A*/
                    if(PMIC_ST_SUCCESS == test_pmic_leo_pmicA_fsm_testApp())
                    {
                       /* Run fsm manual test cases */
                       test_pmic_run_testcases_manual(J7VCL_BOARD);
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
    Board_initUART();

    pmic_log("PMIC FSM Unity Test Application(%s %s)\n",
                                         __TIME__, __DATE__);

#if defined(UNITY_INCLUDE_CONFIG_V2_H) && \
    (defined(SOC_J721E)             || \
     defined(SOC_J7200))
    test_pmic_fsm_testapp_runner();
#endif
}
