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

extern uint16_t pmic_device_info;
extern int32_t gCrcTestFlag_J721E;
extern int32_t gCrcTestFlag_J7VCL;

extern Pmic_Ut_FaultInject_t gPmic_faultInjectCfg;
extern int8_t gMissionStateTestFlag ;

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
    },
    {
        10078,
        "Pmic_fsmGetNsleepSignalMaskStat : Parameter validation for handle."
    },
    {
        10079,
        "Pmic_fsmGetNsleepSignalMaskStat : Parameter validation for nsleepStat."
    },
    {
        10080,
        "Pmic_fsmSetConfiguration/Pmic_fsmGetConfiguration : Enable Fast BIST"
    },
    {
        10081,
        "Pmic_fsmSetConfiguration/Pmic_fsmGetConfiguration : Select LpStandby State"
    },
    {
        10082,
        "Pmic_fsmSetConfiguration/Pmic_fsmGetConfiguration : Select Standby State"
    },
    {
        10083,
        "Pmic_fsmSetConfiguration/Pmic_fsmGetConfiguration : Enable Buck/LDO regulators ILIM"
    },
    {
        10084,
        "Pmic_fsmSetConfiguration/Pmic_fsmGetConfiguration : Disable Buck/LDO regulators ILIM"
    },
    {
        10085,
        "Pmic_fsmSetConfiguration/Pmic_fsmGetConfiguration : Select FSM Startup Destination as ACTIVE"
    },
    {
        10086,
        "Pmic_fsmSetConfiguration/Pmic_fsmGetConfiguration : Select FSM Startup Destination as MCUONLY"
    },
    {
        10087,
        "Pmic_fsmSetConfiguration/Pmic_fsmGetConfiguration : Select FSM Startup Destination as STANDBY/LPSTANDBY"
    },
    {
        10088,
        "Pmic_fsmSetConfiguration : Parameter validation for handle."
    },
    {
        10089,
        "Pmic_fsmSetConfiguration : Parameter Range validation for fsmStarupDestSel Param."
    },
    {
        10090,
        "Pmic_fsmGetConfiguration : Parameter validation for handle."
    },
    {
        10091,
        "Pmic_fsmGetConfiguration : Parameter validation for fsmCfg ."
    },
    {
        10092,
        "Pmic_fsmSetPfsmDelay/Pmic_fsmGetPfsmDelay : Configure Delay for PFSM Delay1"
    },
    {
        10093,
        "Pmic_fsmSetPfsmDelay : Parameter validation for handle."
    },
    {
        10094,
        "Pmic_fsmSetPfsmDelay : Parameter Range validation for pFsmDelayType Param."
    },
    {
        10095,
        "Pmic_fsmGetPfsmDelay : Parameter validation for handle."
    },
    {
        10096,
        "Pmic_fsmGetPfsmDelay : Parameter Range validation for pFsmDelayType Param."
    },
    {
        10097,
        "Pmic_fsmGetPfsmDelay : Parameter validation for PfsmDelay Param."
    },
    {
        10098,
        "Pmic_fsmSetNsleepSignalVal/Pmic_fsmGetNsleepSignalVal : Configure NSleep1 Signal as Low"
    },
    {
        10099,
        "Pmic_fsmSetNsleepSignalVal/Pmic_fsmGetNsleepSignalVal : Configure NSleep1 Signal as High"
    },
    {
        10101,
        "Pmic_fsmSetNsleepSignalVal/Pmic_fsmGetNsleepSignalVal : Configure NSleep2 Signal as Low"
    },
    {
        10102,
        "Pmic_fsmSetNsleepSignalVal/Pmic_fsmGetNsleepSignalVal : Configure NSleep2 Signal as High"
    },
    {
        10103,
        "Pmic_fsmSetNsleepSignalVal : Parameter validation for handle."
    },
    {
        10104,
        "Pmic_fsmSetNsleepSignalVal : Parameter Range validation for nsleepVal Param."
    },
    {
        10105,
        "Pmic_fsmGetNsleepSignalVal : Parameter validation for handle."
    },
    {
        10106,
        "Pmic_fsmGetNsleepSignalVal : Parameter validation for NsleepVal."
    },
    {
        10109,
        "Pmic_fsmRecoverSocPwrErr: Configure NSleep1 Signal as Low and NSleep2 Signal to High"
    },
    {
        10110,
        "Pmic_fsmRecoverSocPwrErr: Configure NSleep1 Signal as High and NSleep2 Signal to High"
    },
    {
        10111,
        "Pmic_fsmRecoverSocPwrErr : Parameter validation for handle."
    },
    {
        10112,
        "Pmic_fsmRecoverSocPwrErr : Parameter Range validation for nsleepVal Param."
    },
    {
        10113,
        "Pmic_fsmEnableI2cTrigger: Trigger i2C4 with trigger value as '1' and '0'"
    },
    {
        10114,
        "Pmic_fsmEnableI2cTrigger: Trigger i2C5 with trigger value as '1' and '0'"
    },
    {
        10118,
        "Pmic_fsmEnableI2cTrigger: Trigger i2C6 with trigger value as '1' and '0' "
    },
    {
        10120,
        "Pmic_fsmGetI2cTriggerVal: Get FSM i2C0/i2C1/i2C2/i2c3 trigger value"
    },
    {
        10119,
        "Pmic_fsmEnableI2cTrigger: Trigger i2C7 with trigger value as '1' and '0'"
    },
    {
        10115,
        "Pmic_fsmEnableI2cTrigger : Parameter validation for handle."
    },
    {
        10116,
        "Pmic_fsmEnableI2cTrigger : Parameter Range validation for i2cTriggerVal param for i2C0/i2C1/i2C2/i2C3/i2C4/i2C5/i2C6/i2C7 Trigger type"
    },
    {
        10117,
        "Pmic_fsmEnableI2cTrigger : Parameter Range validation for i2c Trigger type"
    },
    {
        10122,
        "Pmic_fsmRequestDdrGpioRetentionMode : Parameter validation for handle."
    },
    {
        10123,
        "Pmic_fsmRequestDdrGpioRetentionMode : Parameter Range validation for retentionMode"
    },
    {
        10124,
        "Pmic_fsmRequestDdrGpioRetentionMode : Parameter Range validation for i2cTriggerVal"
    },
    {
        10125,
        "Pmic_fsmSetConfiguration/Pmic_fsmGetConfiguration : Disable Fast BIST"
    },
    {
        10740,
        "Pmic_fsmRuntimeBistRequest : Negative test for Runtime BIST on PG1.0 Silicon Revision"
    },
    {
        10741,
        "Pmic_fsmDeviceOffRequestCfg : configure eventType as PMIC_FSM_ENABLE_PIN_TYPE"
    },
    {
        10742,
        "Pmic_fsmEnableI2cTrigger: Parameter validation for i2c3 Trigger Value"
    },
    {
        10743,
        "Pmic_fsmEnableI2cTrigger: Negative test to trigger i2c2 Trigger type for FSM I2C trigger on PG1.0 Silicon Revision"
    },
    {
        10744,
        "Pmic_fsmEnableI2cTrigger: Parameter validation for i2c0 Trigger Value"
    },
    {
        10745,
        "Pmic_fsmGetI2cTriggerVal: Parameter validation for handle"
    },
    {
        10746,
        "Pmic_fsmGetI2cTriggerVal: Parameter validation for pI2cTriggerVal"
    },
    {
        10747,
        "Pmic_fsmGetI2cTriggerVal: Parameter validation for i2cTriggerType"
    },
    {
        10748,
        "Pmic_fsmRequestDdrGpioRetentionMode : Configure DDR Retention mode with trigger value as '1'"
    },
    {
        10749,
        "Pmic_fsmRecoverSocPwrErr: Negative test to configure nsleepVal as Low for PG1.0 Silicon Revision"
    },
    {
        10750,
        "Pmic_fsmTests : Fault Injection and Coverage Gaps"
    },
};

/*!
 * \brief   Pmic_fsmSetNsleepSignalMask : Test Mask Nsleep1.
 */
static void test_pmic_fsmSetNsleepSignalMask_mask_nsleep1(void)
{
    int32_t status     = PMIC_ST_SUCCESS;
    bool    nsleepType = 0U;
    bool    maskEnable = 0U;
    bool    nsleepMaskStat = 0U;

    nsleepType = PMIC_NSLEEP1_SIGNAL;
    maskEnable = PMIC_NSLEEPX_MASK;

    test_pmic_print_unity_testcase_info(7693,
                                        pmic_fsm_tests,
                                        PMIC_FSM_NUM_OF_TESTCASES);

    status = Pmic_fsmSetNsleepSignalMask(pPmicCoreHandle, nsleepType, maskEnable);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_fsmGetNsleepSignalMaskStat(pPmicCoreHandle,
                                             nsleepType,
                                             &nsleepMaskStat);
    TEST_ASSERT_EQUAL(PMIC_NSLEEPX_MASK, nsleepMaskStat);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    pmic_testResultUpdate_pass(7693,
                               pmic_fsm_tests,
                               PMIC_FSM_NUM_OF_TESTCASES);

}

/*!
 * \brief   Pmic_fsmSetNsleepSignalMask : Test Unmask Nsleep1.
 */
static void test_pmic_fsmSetNsleepSignalMask_unmask_nsleep1(void)
{
    int32_t status     = PMIC_ST_SUCCESS;
    bool    nsleepType = 0U;
    bool    maskEnable = 0U;
    bool    nsleepMaskStat = 0U;

    nsleepType = PMIC_NSLEEP1_SIGNAL;
    maskEnable = PMIC_NSLEEPX_UNMASK;

    test_pmic_print_unity_testcase_info(7694,
                                        pmic_fsm_tests,
                                        PMIC_FSM_NUM_OF_TESTCASES);

    status = Pmic_fsmSetNsleepSignalMask(pPmicCoreHandle, nsleepType, maskEnable);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_fsmGetNsleepSignalMaskStat(pPmicCoreHandle,
                                             nsleepType,
                                             &nsleepMaskStat);
    TEST_ASSERT_EQUAL(PMIC_NSLEEPX_UNMASK, nsleepMaskStat);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    pmic_testResultUpdate_pass(7694,
                               pmic_fsm_tests,
                               PMIC_FSM_NUM_OF_TESTCASES);

}

/*!
 * \brief   Pmic_fsmSetNsleepSignalMask : Test Mask Nsleep2.
 */
static void test_pmic_fsmSetNsleepSignalMask_mask_nsleep2(void)
{
    int32_t status     = PMIC_ST_SUCCESS;
    bool    nsleepType = 0U;
    bool    maskEnable = 0U;
    bool    nsleepMaskStat = 0U;

    nsleepType = PMIC_NSLEEP2_SIGNAL;
    maskEnable = PMIC_NSLEEPX_MASK;

    test_pmic_print_unity_testcase_info(7695,
                                        pmic_fsm_tests,
                                        PMIC_FSM_NUM_OF_TESTCASES);

    status = Pmic_fsmSetNsleepSignalMask(pPmicCoreHandle, nsleepType, maskEnable);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_fsmGetNsleepSignalMaskStat(pPmicCoreHandle,
                                             nsleepType,
                                             &nsleepMaskStat);
    TEST_ASSERT_EQUAL(PMIC_NSLEEPX_MASK, nsleepMaskStat);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    pmic_testResultUpdate_pass(7695,
                               pmic_fsm_tests,
                               PMIC_FSM_NUM_OF_TESTCASES);

}

/*!
 * \brief   Pmic_fsmSetNsleepSignalMask : Test Unmask Nsleep2.
 */
static void test_pmic_fsmSetNsleepSignalMask_unmask_nsleep2(void)
{
    int32_t status     = PMIC_ST_SUCCESS;
    bool    nsleepType = 0U;
    bool    maskEnable = 0U;
    bool    nsleepMaskStat = 0U;

    nsleepType = PMIC_NSLEEP2_SIGNAL;
    maskEnable = PMIC_NSLEEPX_UNMASK;

    test_pmic_print_unity_testcase_info(7696,
                                        pmic_fsm_tests,
                                        PMIC_FSM_NUM_OF_TESTCASES);

    status = Pmic_fsmSetNsleepSignalMask(pPmicCoreHandle, nsleepType, maskEnable);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_fsmGetNsleepSignalMaskStat(pPmicCoreHandle,
                                             nsleepType,
                                             &nsleepMaskStat);
    TEST_ASSERT_EQUAL(PMIC_NSLEEPX_UNMASK, nsleepMaskStat);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    pmic_testResultUpdate_pass(7696,
                               pmic_fsm_tests,
                               PMIC_FSM_NUM_OF_TESTCASES);

}

/*!
 * \brief   Pmic_fsmSetMissionState : Test Set State to Active.
 */
static void test_pmic_fsmSetMissionState_active(void)
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

#if defined(SOC_J721E)
    pmic_log("\r\n Probe TP134 and it should change from High");
    pmic_log("\r\n Probe TP133 and it should continue to be in HIGH");
#endif
#if defined(SOC_J7200)
    pmic_log("\r\n Probe TP46 and it should change from High");
    pmic_log("\r\n Probe TP29 and it should continue to be in HIGH");
#endif
    status = Pmic_fsmSetMissionState(pPmicCoreHandle, pmicState);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    pmic_testResultUpdate_pass(7698,
                               pmic_fsm_tests,
                               PMIC_FSM_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_fsmSetMissionState : Test Set State to S2R/Deep Sleep.
 */
static void test_pmic_fsmSetMissionState_s2r(void)
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
static void test_pmic_fsmSetMissionState_lpstandby(void)
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
static void test_pmic_fsmSetMissionState_standby(void)
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
static void test_pmic_fsmSetMissionStatePrmValTest_handle(void)
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
static void test_pmic_fsmSetMissionStatePrmValTest_state(void)
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
static void test_pmic_fsmSetNsleepSignalMaskStatePrmValTest_handle(void)
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
 * \brief   Pmic_fsmGetNsleepSignalMaskStat : Parameter validation for handle.
 */
static void test_pmic_fsmGetNsleepSignalMaskStatePrmValTest_handle(void)
{
    int32_t status     = PMIC_ST_SUCCESS;
    bool    nsleepType = 0U;
    bool    maskEnable = 0U;

    nsleepType = PMIC_NSLEEP1_SIGNAL;

    test_pmic_print_unity_testcase_info(10078,
                                        pmic_fsm_tests,
                                        PMIC_FSM_NUM_OF_TESTCASES);

    status = Pmic_fsmGetNsleepSignalMaskStat(NULL, nsleepType, &maskEnable);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, status);

    pmic_testResultUpdate_pass(10078,
                               pmic_fsm_tests,
                               PMIC_FSM_NUM_OF_TESTCASES);

}

/*!
 * \brief   Pmic_fsmGetNsleepSignalMaskStat : Parameter validation for nsleepStat.
 */
static void test_pmic_fsmGetNsleepSignalMaskStatePrmValTest_nsleepStat(void)
{
    int32_t status     = PMIC_ST_SUCCESS;
    bool    nsleepType = 0U;

    nsleepType = PMIC_NSLEEP1_SIGNAL;

    test_pmic_print_unity_testcase_info(10079,
                                        pmic_fsm_tests,
                                        PMIC_FSM_NUM_OF_TESTCASES);

    status = Pmic_fsmGetNsleepSignalMaskStat(pPmicCoreHandle, nsleepType, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    pmic_testResultUpdate_pass(10079,
                               pmic_fsm_tests,
                               PMIC_FSM_NUM_OF_TESTCASES);

}

/*!
 * \brief   Parameter validation for handle
 */
static void test_pmic_fsmDevOffReqCfg_PrmValTest_handle(void)
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
static void test_pmic_fsmDevOffReqCfg_PrmValTest_eventType(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t fsmState = 0;
    uint8_t eventType;

    eventType = PMIC_FSM_NPWRON_PIN_TYPE + 0x1;
    test_pmic_print_unity_testcase_info(7361,
                                        pmic_fsm_tests,
                                        PMIC_FSM_NUM_OF_TESTCASES);

    status = Pmic_fsmDeviceOffRequestCfg(pPmicCoreHandle,
                                         eventType,
                                         fsmState);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

    pmic_testResultUpdate_pass(7361,
                               pmic_fsm_tests,
                               PMIC_FSM_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_fsmDeviceOffRequestCfg :  Parameter validation for fsmState
 */
static void test_pmic_fsmDevOffReqCfg_PrmValTest_fsmState(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t fsmState = 0U;
    uint8_t eventType = 1U;

    fsmState = PMIC_FSM_LP_STANBY_STATE + 1;
    test_pmic_print_unity_testcase_info(7705,
                                        pmic_fsm_tests,
                                        PMIC_FSM_NUM_OF_TESTCASES);

    status = Pmic_fsmDeviceOffRequestCfg(pPmicCoreHandle,
                                         eventType,
                                         fsmState);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

    pmic_testResultUpdate_pass(7705,
                               pmic_fsm_tests,
                               PMIC_FSM_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_fsmRuntimeBistRequest : Parameter validation for handle.
 */
static void test_pmic_fsmRuntimeBistRequestPrmValTest_handle(void)
{
    int32_t status     = PMIC_ST_SUCCESS;

    test_pmic_print_unity_testcase_info(7706,
                                        pmic_fsm_tests,
                                        PMIC_FSM_NUM_OF_TESTCASES);

    if(PMIC_SILICON_REV_ID_PG_1_0 == pPmicCoreHandle->pmicDevSiliconRev)
    {
        pmic_testResultUpdate_ignore(7706,
                                     pmic_fsm_tests,
                                     PMIC_FSM_NUM_OF_TESTCASES);
    }

    status = Pmic_fsmRequestRuntimeBist(NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, status);

    pmic_testResultUpdate_pass(7706,
                               pmic_fsm_tests,
                               PMIC_FSM_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_fsmDeviceOnRequest : Parameter validation for handle.
 */
static void test_pmic_fsmDeviceOnRequestPrmValTest_handle(void)
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

/*!
 * \brief   Pmic_fsmSetConfiguration/Pmic_fsmGetConfiguration : Enable Fast BIST
 */
static void test_pmic_fsmSetConfiguration_fastBistEnable(void)
{
    int32_t status     = PMIC_ST_SUCCESS;

    Pmic_FsmCfg_t fsmCfg_rd = {PMIC_FSM_CFG_FAST_BIST_EN_VALID_SHIFT,};
    Pmic_FsmCfg_t fsmCfg =
    {
        PMIC_FSM_CFG_FAST_BIST_EN_VALID_SHIFT,
        PMIC_FSM_FAST_BIST_ENABLE,
        PMIC_FSM_SELECT_STANDBY_STATE,
        PMIC_FSM_ILIM_INT_FSMCTRL_DISABLE,
        PMIC_FSM_STARTUPDEST_ACTIVE
    };

    test_pmic_print_unity_testcase_info(10080,
                                        pmic_fsm_tests,
                                        PMIC_FSM_NUM_OF_TESTCASES);

    status = Pmic_fsmSetConfiguration(pPmicCoreHandle, fsmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_fsmGetConfiguration(pPmicCoreHandle, &fsmCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    TEST_ASSERT_EQUAL(fsmCfg.fastBistEn, fsmCfg_rd.fastBistEn);

    pmic_testResultUpdate_pass(10080,
                               pmic_fsm_tests,
                               PMIC_FSM_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_fsmSetConfiguration/Pmic_fsmGetConfiguration : Select LpStandby State
 */
static void test_pmic_fsmSetConfiguration_selectLpStandbyState(void)
{
    int32_t status     = PMIC_ST_SUCCESS;

    Pmic_FsmCfg_t fsmCfg_rd = {PMIC_FSM_CFG_LP_STANDBYSEL_VALID_SHIFT,};
    Pmic_FsmCfg_t fsmCfg =
    {
        PMIC_FSM_CFG_LP_STANDBYSEL_VALID_SHIFT,
        PMIC_FSM_FAST_BIST_DISABLE,
        PMIC_FSM_SELECT_LPSTANDBY_STATE,
        PMIC_FSM_ILIM_INT_FSMCTRL_DISABLE,
        PMIC_FSM_STARTUPDEST_ACTIVE
    };

    test_pmic_print_unity_testcase_info(10081,
                                        pmic_fsm_tests,
                                        PMIC_FSM_NUM_OF_TESTCASES);

    status = Pmic_fsmSetConfiguration(pPmicCoreHandle, fsmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_fsmGetConfiguration(pPmicCoreHandle, &fsmCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    TEST_ASSERT_EQUAL(fsmCfg.lpStandbySel, fsmCfg_rd.lpStandbySel);

    pmic_testResultUpdate_pass(10081,
                               pmic_fsm_tests,
                               PMIC_FSM_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_fsmSetConfiguration/Pmic_fsmGetConfiguration : Select Standby State
 */
static void test_pmic_fsmSetConfiguration_selectStandbyState(void)
{
    int32_t status     = PMIC_ST_SUCCESS;

    Pmic_FsmCfg_t fsmCfg_rd = {PMIC_FSM_CFG_LP_STANDBYSEL_VALID_SHIFT,};
    Pmic_FsmCfg_t fsmCfg =
    {
        PMIC_FSM_CFG_LP_STANDBYSEL_VALID_SHIFT,
        PMIC_FSM_FAST_BIST_DISABLE,
        PMIC_FSM_SELECT_STANDBY_STATE,
        PMIC_FSM_ILIM_INT_FSMCTRL_DISABLE,
        PMIC_FSM_STARTUPDEST_ACTIVE
    };

    test_pmic_print_unity_testcase_info(10082,
                                        pmic_fsm_tests,
                                        PMIC_FSM_NUM_OF_TESTCASES);

    status = Pmic_fsmSetConfiguration(pPmicCoreHandle, fsmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_fsmGetConfiguration(pPmicCoreHandle, &fsmCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    TEST_ASSERT_EQUAL(fsmCfg.lpStandbySel, fsmCfg_rd.lpStandbySel);

    pmic_testResultUpdate_pass(10082,
                               pmic_fsm_tests,
                               PMIC_FSM_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_fsmSetConfiguration/Pmic_fsmGetConfiguration : Enable Buck/LDO regulators ILIM
 */
static void test_pmic_fsmSetConfiguration_ilimIntfsmCtrlEnable(void)
{
    int32_t status     = PMIC_ST_SUCCESS;

    Pmic_FsmCfg_t fsmCfg_rd = {PMIC_FSM_CFG_ILIM_INT_FSMCTRL_EN_VALID_SHIFT,};
    Pmic_FsmCfg_t fsmCfg =
    {
        PMIC_FSM_CFG_ILIM_INT_FSMCTRL_EN_VALID_SHIFT,
        PMIC_FSM_FAST_BIST_ENABLE,
        PMIC_FSM_SELECT_STANDBY_STATE,
        PMIC_FSM_ILIM_INT_FSMCTRL_ENABLE,
        PMIC_FSM_STARTUPDEST_ACTIVE
    };

    test_pmic_print_unity_testcase_info(10083,
                                        pmic_fsm_tests,
                                        PMIC_FSM_NUM_OF_TESTCASES);

    status = Pmic_fsmSetConfiguration(pPmicCoreHandle, fsmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_fsmGetConfiguration(pPmicCoreHandle, &fsmCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    TEST_ASSERT_EQUAL(fsmCfg.ilimIntfsmCtrlEn, fsmCfg_rd.ilimIntfsmCtrlEn);

    pmic_testResultUpdate_pass(10083,
                               pmic_fsm_tests,
                               PMIC_FSM_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_fsmSetConfiguration/Pmic_fsmGetConfiguration : Disable Buck/LDO regulators ILIM
 */
static void test_pmic_fsmSetConfiguration_ilimIntfsmCtrlDisable(void)
{
    int32_t status     = PMIC_ST_SUCCESS;

    Pmic_FsmCfg_t fsmCfg_rd = {PMIC_FSM_CFG_ILIM_INT_FSMCTRL_EN_VALID_SHIFT,};
    Pmic_FsmCfg_t fsmCfg =
    {
        PMIC_FSM_CFG_ILIM_INT_FSMCTRL_EN_VALID_SHIFT,
        PMIC_FSM_FAST_BIST_DISABLE,
        PMIC_FSM_SELECT_STANDBY_STATE,
        PMIC_FSM_ILIM_INT_FSMCTRL_DISABLE,
        PMIC_FSM_STARTUPDEST_ACTIVE
    };

    test_pmic_print_unity_testcase_info(10084,
                                        pmic_fsm_tests,
                                        PMIC_FSM_NUM_OF_TESTCASES);

    status = Pmic_fsmSetConfiguration(pPmicCoreHandle, fsmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_fsmGetConfiguration(pPmicCoreHandle, &fsmCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    TEST_ASSERT_EQUAL(fsmCfg.ilimIntfsmCtrlEn, fsmCfg_rd.ilimIntfsmCtrlEn);

    pmic_testResultUpdate_pass(10084,
                               pmic_fsm_tests,
                               PMIC_FSM_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_fsmSetConfiguration/Pmic_fsmGetConfiguration : Select FSM Startup Destination as ACTIVE
 */
static void test_pmic_fsmSetConfiguration_selectfsmStarupDestActiveState(void)
{
    int32_t status     = PMIC_ST_SUCCESS;

    Pmic_FsmCfg_t fsmCfg_rd = {PMIC_FSM_CFG_FSM_STARTUP_DEST_SEL_VALID_SHIFT,};
    Pmic_FsmCfg_t fsmCfg =
    {
        PMIC_FSM_CFG_FSM_STARTUP_DEST_SEL_VALID_SHIFT,
        PMIC_FSM_FAST_BIST_DISABLE,
        PMIC_FSM_SELECT_STANDBY_STATE,
        PMIC_FSM_ILIM_INT_FSMCTRL_DISABLE,
        PMIC_FSM_STARTUPDEST_ACTIVE
    };

    test_pmic_print_unity_testcase_info(10085,
                                        pmic_fsm_tests,
                                        PMIC_FSM_NUM_OF_TESTCASES);

    status = Pmic_fsmSetConfiguration(pPmicCoreHandle, fsmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_fsmGetConfiguration(pPmicCoreHandle, &fsmCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    TEST_ASSERT_EQUAL(fsmCfg.fsmStarupDestSel, fsmCfg_rd.fsmStarupDestSel);

    pmic_testResultUpdate_pass(10085,
                               pmic_fsm_tests,
                               PMIC_FSM_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_fsmSetConfiguration/Pmic_fsmGetConfiguration : Select FSM Startup Destination as MCUONLY
 */
static void test_pmic_fsmSetConfiguration_selectfsmStarupDestMcuonlyState(void)
{
    int32_t status     = PMIC_ST_SUCCESS;

    Pmic_FsmCfg_t fsmCfg_rd  = {PMIC_FSM_CFG_FSM_STARTUP_DEST_SEL_VALID_SHIFT,};
    Pmic_FsmCfg_t fsmCfg_def = {PMIC_FSM_CFG_FSM_STARTUP_DEST_SEL_VALID_SHIFT,};
    Pmic_FsmCfg_t fsmCfg =
    {
        PMIC_FSM_CFG_FSM_STARTUP_DEST_SEL_VALID_SHIFT,
        PMIC_FSM_FAST_BIST_DISABLE,
        PMIC_FSM_SELECT_STANDBY_STATE,
        PMIC_FSM_ILIM_INT_FSMCTRL_DISABLE,
        PMIC_FSM_STARTUPDEST_MCUONLY
    };

    test_pmic_print_unity_testcase_info(10086,
                                        pmic_fsm_tests,
                                        PMIC_FSM_NUM_OF_TESTCASES);

    status = Pmic_fsmGetConfiguration(pPmicCoreHandle, &fsmCfg_def);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_fsmSetConfiguration(pPmicCoreHandle, fsmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_fsmGetConfiguration(pPmicCoreHandle, &fsmCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    TEST_ASSERT_EQUAL(fsmCfg.fsmStarupDestSel, fsmCfg_rd.fsmStarupDestSel);

    status = Pmic_fsmSetConfiguration(pPmicCoreHandle, fsmCfg_def);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_fsmGetConfiguration(pPmicCoreHandle, &fsmCfg_rd);

    TEST_ASSERT_EQUAL(fsmCfg_def.fsmStarupDestSel, fsmCfg_rd.fsmStarupDestSel);

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    pmic_testResultUpdate_pass(10086,
                               pmic_fsm_tests,
                               PMIC_FSM_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_fsmSetConfiguration/Pmic_fsmGetConfiguration : Select FSM Startup Destination as STANDBY/LPSTANDBY
 */
static void test_pmic_fsmSetConfiguration_selectfsmStarupDestStandbyOrLpStandbyState(void)
{
    int32_t status     = PMIC_ST_SUCCESS;

    Pmic_FsmCfg_t fsmCfg_rd  = {PMIC_FSM_CFG_FSM_STARTUP_DEST_SEL_VALID_SHIFT,};
    Pmic_FsmCfg_t fsmCfg_def = {PMIC_FSM_CFG_FSM_STARTUP_DEST_SEL_VALID_SHIFT,};
    Pmic_FsmCfg_t fsmCfg =
    {
        PMIC_FSM_CFG_FSM_STARTUP_DEST_SEL_VALID_SHIFT,
        PMIC_FSM_FAST_BIST_DISABLE,
        PMIC_FSM_SELECT_STANDBY_STATE,
        PMIC_FSM_ILIM_INT_FSMCTRL_DISABLE,
        PMIC_FSM_STARTUPDEST_STANDBY_LPSTANDBY
    };

    test_pmic_print_unity_testcase_info(10087,
                                        pmic_fsm_tests,
                                        PMIC_FSM_NUM_OF_TESTCASES);

    status = Pmic_fsmGetConfiguration(pPmicCoreHandle, &fsmCfg_def);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_fsmSetConfiguration(pPmicCoreHandle, fsmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_fsmGetConfiguration(pPmicCoreHandle, &fsmCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    TEST_ASSERT_EQUAL(fsmCfg.fsmStarupDestSel, fsmCfg_rd.fsmStarupDestSel);

    status = Pmic_fsmSetConfiguration(pPmicCoreHandle, fsmCfg_def);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_fsmGetConfiguration(pPmicCoreHandle, &fsmCfg_rd);

    TEST_ASSERT_EQUAL(fsmCfg_def.fsmStarupDestSel, fsmCfg_rd.fsmStarupDestSel);

    pmic_testResultUpdate_pass(10087,
                               pmic_fsm_tests,
                               PMIC_FSM_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_fsmSetConfiguration : Parameter validation for handle
 */
static void test_pmic_fsmSetConfiguration_prmValTest_handle(void)
{
    int32_t status     = PMIC_ST_SUCCESS;

    Pmic_FsmCfg_t fsmCfg =
    {
        PMIC_FSM_CFG_FAST_BIST_EN_VALID_SHIFT,
        PMIC_FSM_FAST_BIST_DISABLE,
        PMIC_FSM_SELECT_STANDBY_STATE,
        PMIC_FSM_ILIM_INT_FSMCTRL_DISABLE,
        PMIC_FSM_STARTUPDEST_ACTIVE
    };

    test_pmic_print_unity_testcase_info(10088,
                                        pmic_fsm_tests,
                                        PMIC_FSM_NUM_OF_TESTCASES);

    status = Pmic_fsmSetConfiguration(NULL, fsmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, status);

    pmic_testResultUpdate_pass(10088,
                               pmic_fsm_tests,
                               PMIC_FSM_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_fsmSetConfiguration : Parameter Range validation for fsmStarupDestSel Param
 */
static void test_pmic_fsmSetConfiguration_prmValTest_fsmStarupDestSel(void)
{
    int32_t status     = PMIC_ST_SUCCESS;

    Pmic_FsmCfg_t fsmCfg =
    {
        PMIC_FSM_CFG_FSM_STARTUP_DEST_SEL_VALID_SHIFT,
        PMIC_FSM_FAST_BIST_DISABLE,
        PMIC_FSM_SELECT_STANDBY_STATE,
        PMIC_FSM_ILIM_INT_FSMCTRL_DISABLE,
        4U
    };

    test_pmic_print_unity_testcase_info(10089,
                                        pmic_fsm_tests,
                                        PMIC_FSM_NUM_OF_TESTCASES);

    status = Pmic_fsmSetConfiguration(pPmicCoreHandle, fsmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

    pmic_testResultUpdate_pass(10089,
                               pmic_fsm_tests,
                               PMIC_FSM_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_fsmGetConfiguration : Parameter validation for handle
 */
static void test_pmic_fsmGetConfiguration_prmValTest_handle(void)
{
    int32_t status     = PMIC_ST_SUCCESS;

    Pmic_FsmCfg_t fsmCfg_rd;

    test_pmic_print_unity_testcase_info(10090,
                                        pmic_fsm_tests,
                                        PMIC_FSM_NUM_OF_TESTCASES);

    status = Pmic_fsmGetConfiguration(NULL, &fsmCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, status);


    pmic_testResultUpdate_pass(10090,
                               pmic_fsm_tests,
                               PMIC_FSM_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_fsmGetConfiguration : Parameter validation for fsmCfg
 */
static void test_pmic_fsmGetConfiguration_prmValTest_fsmCfg(void)
{
    int32_t status     = PMIC_ST_SUCCESS;

    test_pmic_print_unity_testcase_info(10091,
                                        pmic_fsm_tests,
                                        PMIC_FSM_NUM_OF_TESTCASES);

    status = Pmic_fsmGetConfiguration(pPmicCoreHandle, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    pmic_testResultUpdate_pass(10091,
                               pmic_fsm_tests,
                               PMIC_FSM_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_fsmSetPfsmDelay/Pmic_fsmGetPfsmDelay : Configure PFSM Delay
 */
static void test_pmic_fsmSetPfsmDelay(void)
{
    int32_t status     = PMIC_ST_SUCCESS;
    uint8_t delayType, delayVal[4], tstDelayVal[4], tstDelayVal_rd;

    test_pmic_print_unity_testcase_info(10092,
                                        pmic_fsm_tests,
                                        PMIC_FSM_NUM_OF_TESTCASES);

    for(delayType = PMIC_PFSM_DELAY1; delayType <= PMIC_PFSM_DELAY4; delayType++)
    {
        status = Pmic_fsmGetPfsmDelay(pPmicCoreHandle,
                                      delayType,
                                      &delayVal[delayType]);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        tstDelayVal[delayType] = delayVal[delayType] + 10 + delayType;

    }

    for(delayType = PMIC_PFSM_DELAY1; delayType <= PMIC_PFSM_DELAY4; delayType++)
    {
        status = Pmic_fsmSetPfsmDelay(pPmicCoreHandle,
                                      delayType,
                                      tstDelayVal[delayType]);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        status = Pmic_fsmGetPfsmDelay(pPmicCoreHandle,
                                      delayType,
                                      &tstDelayVal_rd);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        TEST_ASSERT_EQUAL(tstDelayVal_rd, tstDelayVal[delayType]);
    }

    for(delayType = PMIC_PFSM_DELAY1; delayType <= PMIC_PFSM_DELAY4; delayType++)
    {
        status = Pmic_fsmSetPfsmDelay(pPmicCoreHandle,
                                      delayType,
                                      delayVal[delayType]);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    }

    pmic_testResultUpdate_pass(10092,
                               pmic_fsm_tests,
                               PMIC_FSM_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_fsmSetPfsmDelay : Parameter validation for handle
 */
static void test_pmic_fsmSetPfsmDelay_prmValTest_handle(void)
{
    int32_t status     = PMIC_ST_SUCCESS;

    test_pmic_print_unity_testcase_info(10093,
                                        pmic_fsm_tests,
                                        PMIC_FSM_NUM_OF_TESTCASES);

    status = Pmic_fsmSetPfsmDelay(NULL, PMIC_PFSM_DELAY1, 10U);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, status);

    pmic_testResultUpdate_pass(10093,
                               pmic_fsm_tests,
                               PMIC_FSM_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_fsmSetPfsmDelay : Parameter Range validation for pFsmDelayType Param
 */
static void test_pmic_fsmSetPfsmDelay_prmValTest_fsmStarupDestSel(void)
{
    int32_t status     = PMIC_ST_SUCCESS;

    test_pmic_print_unity_testcase_info(10094,
                                        pmic_fsm_tests,
                                        PMIC_FSM_NUM_OF_TESTCASES);

    status = Pmic_fsmSetPfsmDelay(pPmicCoreHandle, 4U, 10U);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

    pmic_testResultUpdate_pass(10094,
                               pmic_fsm_tests,
                               PMIC_FSM_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_fsmGetPfsmDelay : Parameter validation for handle
 */
static void test_pmic_fsmGetPfsmDelay_prmValTest_handle(void)
{
    int32_t status     = PMIC_ST_SUCCESS;
    uint8_t delayVal;

    test_pmic_print_unity_testcase_info(10095,
                                        pmic_fsm_tests,
                                        PMIC_FSM_NUM_OF_TESTCASES);

    status = Pmic_fsmGetPfsmDelay(NULL, PMIC_PFSM_DELAY1, &delayVal);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, status);

    pmic_testResultUpdate_pass(10095,
                               pmic_fsm_tests,
                               PMIC_FSM_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_fsmGetPfsmDelay : Parameter Range validation for pFsmDelayType Param
 */
static void test_pmic_fsmGetPfsmDelay_prmValTest_fsmStarupDestSel(void)
{
    int32_t status     = PMIC_ST_SUCCESS;
    uint8_t delayVal;

    test_pmic_print_unity_testcase_info(10096,
                                        pmic_fsm_tests,
                                        PMIC_FSM_NUM_OF_TESTCASES);

    status = Pmic_fsmGetPfsmDelay(pPmicCoreHandle, 4U, &delayVal);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

    pmic_testResultUpdate_pass(10096,
                               pmic_fsm_tests,
                               PMIC_FSM_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_fsmGetPfsmDelay : Parameter validation for PfsmDelay Param
 */
static void test_pmic_fsmGetPfsmDelay_prmValTest_PfsmDelay(void)
{
    int32_t status     = PMIC_ST_SUCCESS;

    test_pmic_print_unity_testcase_info(10097,
                                        pmic_fsm_tests,
                                        PMIC_FSM_NUM_OF_TESTCASES);

    status = Pmic_fsmGetPfsmDelay(pPmicCoreHandle, PMIC_PFSM_DELAY4, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    pmic_testResultUpdate_pass(10097,
                               pmic_fsm_tests,
                               PMIC_FSM_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_fsmSetNsleepSignalVal/Pmic_fsmGetNsleepSignalVal : Configure NSleep1 Signal as Low
 */
static void test_pmic_fsmSetNsleepSignalVal_nsleep1Low(void)
{
    int32_t status     = PMIC_ST_SUCCESS;
    uint8_t nsleepVal;

    test_pmic_print_unity_testcase_info(10098,
                                        pmic_fsm_tests,
                                        PMIC_FSM_NUM_OF_TESTCASES);

#if defined(SOC_J721E)
    pmic_log("\r\n Probe TP134 and it should change from High to Low.");
    pmic_log("\r\n Probe TP133 and it should continue to be in HIGH");
#endif
#if defined(SOC_J7200)
    pmic_log("\r\n Probe TP46 and it should change from High to Low.");
    pmic_log("\r\n Probe TP29 and it should continue to be in HIGH");
#endif

    status = Pmic_fsmSetNsleepSignalVal(pPmicCoreHandle,
                                        PMIC_NSLEEP1_SIGNAL,
                                        PMIC_NSLEEP_LOW);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_fsmGetNsleepSignalVal(pPmicCoreHandle,
                                        PMIC_NSLEEP1_SIGNAL,
                                        &nsleepVal);

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    TEST_ASSERT_EQUAL(nsleepVal, PMIC_NSLEEP_LOW);

    pmic_testResultUpdate_pass(10098,
                               pmic_fsm_tests,
                               PMIC_FSM_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_fsmSetNsleepSignalVal/Pmic_fsmGetNsleepSignalVal : Configure NSleep1 Signal as High
 */
static void test_pmic_fsmSetNsleepSignalVal_nsleep1High(void)
{
    int32_t status     = PMIC_ST_SUCCESS;
    uint8_t nsleepVal;

    test_pmic_print_unity_testcase_info(10099,
                                        pmic_fsm_tests,
                                        PMIC_FSM_NUM_OF_TESTCASES);

    status = Pmic_fsmSetNsleepSignalVal(pPmicCoreHandle,
                                        PMIC_NSLEEP1_SIGNAL,
                                        PMIC_NSLEEP_HIGH);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_fsmGetNsleepSignalVal(pPmicCoreHandle,
                                        PMIC_NSLEEP1_SIGNAL,
                                        &nsleepVal);

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    TEST_ASSERT_EQUAL(nsleepVal, PMIC_NSLEEP_HIGH);

    pmic_testResultUpdate_pass(10099,
                               pmic_fsm_tests,
                               PMIC_FSM_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_fsmSetNsleepSignalVal/Pmic_fsmGetNsleepSignalVal : Configure NSleep2 Signal as Low
 */
static void test_pmic_fsmSetNsleepSignalVal_nsleep2Low(void)
{
    int32_t status     = PMIC_ST_SUCCESS;
    uint8_t nsleepVal;

    test_pmic_print_unity_testcase_info(10101,
                                        pmic_fsm_tests,
                                        PMIC_FSM_NUM_OF_TESTCASES);

#if defined(SOC_J721E)
     pmic_log("\r\n Probe TP134 and TP133 and it should change from High to Low.");
#endif
#if defined(SOC_J7200)
     pmic_log("\r\n Probe TP46 and TP29 and it should change from High to Low.");
#endif

    status = Pmic_fsmSetNsleepSignalVal(pPmicCoreHandle,
                                        PMIC_NSLEEP2_SIGNAL,
                                        PMIC_NSLEEP_LOW);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_fsmGetNsleepSignalVal(pPmicCoreHandle,
                                        PMIC_NSLEEP2_SIGNAL,
                                        &nsleepVal);

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    TEST_ASSERT_EQUAL(nsleepVal, PMIC_NSLEEP_LOW);

    pmic_testResultUpdate_pass(10101,
                               pmic_fsm_tests,
                               PMIC_FSM_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_fsmSetNsleepSignalVal/Pmic_fsmGetNsleepSignalVal : Configure NSleep2 Signal as High
 */
static void test_pmic_fsmSetNsleepSignalVal_nsleep2High(void)
{
    int32_t status     = PMIC_ST_SUCCESS;
    uint8_t nsleepVal;

    test_pmic_print_unity_testcase_info(10102,
                                        pmic_fsm_tests,
                                        PMIC_FSM_NUM_OF_TESTCASES);

    status = Pmic_fsmSetNsleepSignalVal(pPmicCoreHandle,
                                        PMIC_NSLEEP2_SIGNAL,
                                        PMIC_NSLEEP_HIGH);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_fsmGetNsleepSignalVal(pPmicCoreHandle,
                                        PMIC_NSLEEP2_SIGNAL,
                                        &nsleepVal);

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    TEST_ASSERT_EQUAL(nsleepVal, PMIC_NSLEEP_HIGH);

    pmic_testResultUpdate_pass(10102,
                               pmic_fsm_tests,
                               PMIC_FSM_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_fsmSetNsleepSignalVal : Parameter validation for handle
 */
static void test_pmic_fsmSetNsleepSignalVal_prmValTest_handle(void)
{
    int32_t status     = PMIC_ST_SUCCESS;

    test_pmic_print_unity_testcase_info(10103,
                                        pmic_fsm_tests,
                                        PMIC_FSM_NUM_OF_TESTCASES);

    status = Pmic_fsmSetNsleepSignalVal(NULL,
                                        PMIC_NSLEEP2_SIGNAL,
                                        PMIC_NSLEEP_HIGH);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, status);

    pmic_testResultUpdate_pass(10103,
                               pmic_fsm_tests,
                               PMIC_FSM_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_fsmSetNsleepSignalVal : Parameter Range validation for nsleepVal Param
 */
static void test_pmic_fsmSetNsleepSignalVal_prmValTest_nsleepVal(void)
{
    int32_t status     = PMIC_ST_SUCCESS;

    test_pmic_print_unity_testcase_info(10104,
                                        pmic_fsm_tests,
                                        PMIC_FSM_NUM_OF_TESTCASES);

    status = Pmic_fsmSetNsleepSignalVal(pPmicCoreHandle,
                                        PMIC_NSLEEP2_SIGNAL,
                                        2U);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

    pmic_testResultUpdate_pass(10104,
                               pmic_fsm_tests,
                               PMIC_FSM_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_fsmGetNsleepSignalVal : Parameter validation for handle
 */
static void test_pmic_fsmGetNsleepSignalVal_prmValTest_handle(void)
{
    int32_t status     = PMIC_ST_SUCCESS;
    uint8_t nsleepVal;

    test_pmic_print_unity_testcase_info(10105,
                                        pmic_fsm_tests,
                                        PMIC_FSM_NUM_OF_TESTCASES);

    status = Pmic_fsmGetNsleepSignalVal(NULL,
                                        PMIC_NSLEEP2_SIGNAL,
                                        &nsleepVal);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, status);

    pmic_testResultUpdate_pass(10105,
                               pmic_fsm_tests,
                               PMIC_FSM_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_fsmGetNsleepSignalVal : Parameter validation for nsleepVal Param
 */
static void test_pmic_fsmGetNsleepSignalVal_prmValTest_nsleepVal(void)
{
    int32_t status     = PMIC_ST_SUCCESS;

    test_pmic_print_unity_testcase_info(10106,
                                        pmic_fsm_tests,
                                        PMIC_FSM_NUM_OF_TESTCASES);

    status = Pmic_fsmGetNsleepSignalVal(pPmicCoreHandle,
                                        PMIC_NSLEEP2_SIGNAL,
                                        NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    pmic_testResultUpdate_pass(10106,
                               pmic_fsm_tests,
                               PMIC_FSM_NUM_OF_TESTCASES);
}

#if 0
/*!
 * \brief   Pmic_fsmRecoverSocPwrErr: Configure NSleep1 Signal as Low and
 *          NSleep2 Signal to High
 */
static void test_pmic_fsmRecoverSocPwrErr_nsleepLow(void)
{
    int32_t status     = PMIC_ST_SUCCESS;

    test_pmic_print_unity_testcase_info(10109,
                                        pmic_fsm_tests,
                                        PMIC_FSM_NUM_OF_TESTCASES);

    if(PMIC_SILICON_REV_ID_PG_1_0 == pPmicCoreHandle->pmicDevSiliconRev)
    {
        pmic_testResultUpdate_ignore(10109,
                                     pmic_fsm_tests,
                                     PMIC_FSM_NUM_OF_TESTCASES);
    }

    status = Pmic_fsmRecoverSocPwrErr(pPmicCoreHandle,
                                      PMIC_NSLEEP_LOW);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    pmic_testResultUpdate_pass(10109,
                               pmic_fsm_tests,
                               PMIC_FSM_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_fsmRecoverSocPwrErr: Configure NSleep1 Signal as High and
 *          NSleep2 Signal to High
 */
static void test_pmic_fsmRecoverSocPwrErr_nsleepHigh(void)
{
    int32_t status     = PMIC_ST_SUCCESS;

    test_pmic_print_unity_testcase_info(10110,
                                        pmic_fsm_tests,
                                        PMIC_FSM_NUM_OF_TESTCASES);

    if(PMIC_SILICON_REV_ID_PG_1_0 == pPmicCoreHandle->pmicDevSiliconRev)
    {
        pmic_testResultUpdate_ignore(10110,
                                     pmic_fsm_tests,
                                     PMIC_FSM_NUM_OF_TESTCASES);
    }

    status = Pmic_fsmRecoverSocPwrErr(pPmicCoreHandle,
                                      PMIC_NSLEEP_HIGH);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    pmic_testResultUpdate_pass(10110,
                               pmic_fsm_tests,
                               PMIC_FSM_NUM_OF_TESTCASES);
}
#endif

/*!
 * \brief   Pmic_fsmRecoverSocPwrErr: Parameter validation for handle
 */
static void test_pmic_fsmRecoverSocPwrErr_prmValTest_handle(void)
{
    int32_t status     = PMIC_ST_SUCCESS;

    test_pmic_print_unity_testcase_info(10111,
                                        pmic_fsm_tests,
                                        PMIC_FSM_NUM_OF_TESTCASES);

    if(PMIC_SILICON_REV_ID_PG_1_0 == pPmicCoreHandle->pmicDevSiliconRev)
    {
        pmic_testResultUpdate_ignore(10111,
                                     pmic_fsm_tests,
                                     PMIC_FSM_NUM_OF_TESTCASES);
    }

    status = Pmic_fsmRecoverSocPwrErr(NULL,
                                      PMIC_NSLEEP_LOW);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, status);

    pmic_testResultUpdate_pass(10111,
                               pmic_fsm_tests,
                               PMIC_FSM_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_fsmRecoverSocPwrErr: Parameter Range validation for nsleepVal Param
 */
static void test_pmic_fsmRecoverSocPwrErr_prmValTest_nsleepVal(void)
{
    int32_t status     = PMIC_ST_SUCCESS;

    test_pmic_print_unity_testcase_info(10112,
                                        pmic_fsm_tests,
                                        PMIC_FSM_NUM_OF_TESTCASES);

    if(PMIC_SILICON_REV_ID_PG_1_0 == pPmicCoreHandle->pmicDevSiliconRev)
    {
        pmic_testResultUpdate_ignore(10112,
                                     pmic_fsm_tests,
                                     PMIC_FSM_NUM_OF_TESTCASES);
    }

    status = Pmic_fsmRecoverSocPwrErr(pPmicCoreHandle, 2U);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

    pmic_testResultUpdate_pass(10112,
                               pmic_fsm_tests,
                               PMIC_FSM_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_fsmEnableI2cTrigger: Trigger i2C4 with trigger value as '1' and '0'
 */
static void test_pmic_fsmEnableI2cTrigger_i2C4(void)
{
    int32_t status     = PMIC_ST_SUCCESS;
    uint8_t i2cTriggerVal;

    test_pmic_print_unity_testcase_info(10113,
                                        pmic_fsm_tests,
                                        PMIC_FSM_NUM_OF_TESTCASES);

    status = Pmic_fsmEnableI2cTrigger(pPmicCoreHandle,
                                      PMIC_FSM_I2C_TRIGGER4,
                                      PMIC_FSM_I2C_TRIGGER_VAL_1);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_fsmGetI2cTriggerVal(pPmicCoreHandle,
                                      PMIC_FSM_I2C_TRIGGER4,
                                      &i2cTriggerVal);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(i2cTriggerVal, PMIC_FSM_I2C_TRIGGER_VAL_1);

    status = Pmic_fsmEnableI2cTrigger(pPmicCoreHandle,
                                      PMIC_FSM_I2C_TRIGGER4,
                                      PMIC_FSM_I2C_TRIGGER_VAL_0);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_fsmGetI2cTriggerVal(pPmicCoreHandle,
                                      PMIC_FSM_I2C_TRIGGER4,
                                      &i2cTriggerVal);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(i2cTriggerVal, PMIC_FSM_I2C_TRIGGER_VAL_0);

    pmic_testResultUpdate_pass(10113,
                               pmic_fsm_tests,
                               PMIC_FSM_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_fsmEnableI2cTrigger: Trigger i2C5 with trigger value as '1' and '0'
 */
static void test_pmic_fsmEnableI2cTrigger_i2C5(void)
{
    int32_t status     = PMIC_ST_SUCCESS;
    uint8_t i2cTriggerVal;

    test_pmic_print_unity_testcase_info(10114,
                                        pmic_fsm_tests,
                                        PMIC_FSM_NUM_OF_TESTCASES);

    status = Pmic_fsmEnableI2cTrigger(pPmicCoreHandle,
                                      PMIC_FSM_I2C_TRIGGER5,
                                      PMIC_FSM_I2C_TRIGGER_VAL_1);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_fsmGetI2cTriggerVal(pPmicCoreHandle,
                                      PMIC_FSM_I2C_TRIGGER5,
                                      &i2cTriggerVal);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(i2cTriggerVal, PMIC_FSM_I2C_TRIGGER_VAL_1);

    status = Pmic_fsmEnableI2cTrigger(pPmicCoreHandle,
                                      PMIC_FSM_I2C_TRIGGER5,
                                      PMIC_FSM_I2C_TRIGGER_VAL_0);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_fsmGetI2cTriggerVal(pPmicCoreHandle,
                                      PMIC_FSM_I2C_TRIGGER5,
                                      &i2cTriggerVal);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(i2cTriggerVal, PMIC_FSM_I2C_TRIGGER_VAL_0);

    pmic_testResultUpdate_pass(10114,
                               pmic_fsm_tests,
                               PMIC_FSM_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_fsmEnableI2cTrigger: Trigger i2C6 with trigger value as '1' and '0'
 */
static void test_pmic_fsmEnableI2cTrigger_i2C6(void)
{
    int32_t status     = PMIC_ST_SUCCESS;
    uint8_t i2cTriggerVal;

    test_pmic_print_unity_testcase_info(10118,
                                        pmic_fsm_tests,
                                        PMIC_FSM_NUM_OF_TESTCASES);

    status = Pmic_fsmEnableI2cTrigger(pPmicCoreHandle,
                                      PMIC_FSM_I2C_TRIGGER6,
                                      PMIC_FSM_I2C_TRIGGER_VAL_1);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_fsmGetI2cTriggerVal(pPmicCoreHandle,
                                      PMIC_FSM_I2C_TRIGGER6,
                                      &i2cTriggerVal);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(i2cTriggerVal, PMIC_FSM_I2C_TRIGGER_VAL_1);

    status = Pmic_fsmEnableI2cTrigger(pPmicCoreHandle,
                                      PMIC_FSM_I2C_TRIGGER6,
                                      PMIC_FSM_I2C_TRIGGER_VAL_0);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_fsmGetI2cTriggerVal(pPmicCoreHandle,
                                      PMIC_FSM_I2C_TRIGGER6,
                                      &i2cTriggerVal);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(i2cTriggerVal, PMIC_FSM_I2C_TRIGGER_VAL_0);

    pmic_testResultUpdate_pass(10118,
                               pmic_fsm_tests,
                               PMIC_FSM_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_fsmEnableI2cTrigger: Trigger i2C7 with trigger value as '1' and '0'
 */
static void test_pmic_fsmEnableI2cTrigger_i2C7(void)
{
    int32_t status     = PMIC_ST_SUCCESS;
    uint8_t i2cTriggerVal;

    test_pmic_print_unity_testcase_info(10119,
                                        pmic_fsm_tests,
                                        PMIC_FSM_NUM_OF_TESTCASES);

    status = Pmic_fsmEnableI2cTrigger(pPmicCoreHandle,
                                      PMIC_FSM_I2C_TRIGGER7,
                                      PMIC_FSM_I2C_TRIGGER_VAL_1);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_fsmGetI2cTriggerVal(pPmicCoreHandle,
                                      PMIC_FSM_I2C_TRIGGER7,
                                      &i2cTriggerVal);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(i2cTriggerVal, PMIC_FSM_I2C_TRIGGER_VAL_1);

    status = Pmic_fsmEnableI2cTrigger(pPmicCoreHandle,
                                      PMIC_FSM_I2C_TRIGGER7,
                                      PMIC_FSM_I2C_TRIGGER_VAL_0);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_fsmGetI2cTriggerVal(pPmicCoreHandle,
                                      PMIC_FSM_I2C_TRIGGER7,
                                      &i2cTriggerVal);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(i2cTriggerVal, PMIC_FSM_I2C_TRIGGER_VAL_0);

    pmic_testResultUpdate_pass(10119,
                               pmic_fsm_tests,
                               PMIC_FSM_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_fsmGetI2cTriggerVal: Get FSM i2C0/i2C1/i2C2/i2c3 trigger value
 */
static void test_pmic_fsmGetI2cTriggerVal_i2C0_i2c1_i2c2_i2c3(void)
{
    int32_t status     = PMIC_ST_SUCCESS;
    uint8_t i2cTriggerVal;

    test_pmic_print_unity_testcase_info(10120,
                                        pmic_fsm_tests,
                                        PMIC_FSM_NUM_OF_TESTCASES);

    status = Pmic_fsmGetI2cTriggerVal(pPmicCoreHandle,
                                      PMIC_FSM_I2C_TRIGGER0,
                                      &i2cTriggerVal);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(i2cTriggerVal, PMIC_FSM_I2C_TRIGGER_VAL_0);

    status = Pmic_fsmGetI2cTriggerVal(pPmicCoreHandle,
                                      PMIC_FSM_I2C_TRIGGER1,
                                      &i2cTriggerVal);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(i2cTriggerVal, PMIC_FSM_I2C_TRIGGER_VAL_0);

    status = Pmic_fsmGetI2cTriggerVal(pPmicCoreHandle,
                                      PMIC_FSM_I2C_TRIGGER2,
                                      &i2cTriggerVal);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(i2cTriggerVal, PMIC_FSM_I2C_TRIGGER_VAL_0);

    status = Pmic_fsmGetI2cTriggerVal(pPmicCoreHandle,
                                      PMIC_FSM_I2C_TRIGGER3,
                                      &i2cTriggerVal);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(i2cTriggerVal, PMIC_FSM_I2C_TRIGGER_VAL_0);

    pmic_testResultUpdate_pass(10120,
                               pmic_fsm_tests,
                               PMIC_FSM_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_fsmEnableI2cTrigger: Parameter validation for handle
 */
static void test_pmic_fsmEnableI2cTrigger_prmValTest_handle(void)
{
    int32_t status     = PMIC_ST_SUCCESS;

    test_pmic_print_unity_testcase_info(10115,
                                        pmic_fsm_tests,
                                        PMIC_FSM_NUM_OF_TESTCASES);

    status = Pmic_fsmEnableI2cTrigger(NULL,
                                      PMIC_FSM_I2C_TRIGGER0,
                                      PMIC_FSM_I2C_TRIGGER_VAL_1);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, status);

    pmic_testResultUpdate_pass(10115,
                               pmic_fsm_tests,
                               PMIC_FSM_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_fsmEnableI2cTrigger: Parameter Range validation for i2cTriggerVal
 *          param for i2C0/i2C1/i2C2/i2C3/i2C4/i2C5/i2C6/i2C7 Trigger type
 */
static void test_pmic_fsmEnableI2cTrigger_prmValTest_i2cTriggerVal(void)
{
    int32_t status     = PMIC_ST_SUCCESS;
    uint8_t i2cTriggerType;

    test_pmic_print_unity_testcase_info(10116,
                                        pmic_fsm_tests,
                                        PMIC_FSM_NUM_OF_TESTCASES);

    for(i2cTriggerType = PMIC_FSM_I2C_TRIGGER0; i2cTriggerType <= PMIC_FSM_I2C_TRIGGER7; i2cTriggerType++)
    {
        if((PMIC_SILICON_REV_ID_PG_1_0 == pPmicCoreHandle->pmicDevSiliconRev) &&
           ((i2cTriggerType > PMIC_FSM_I2C_TRIGGER0) &&
            (i2cTriggerType < PMIC_FSM_I2C_TRIGGER4)))
        {
            continue;
        }

        if(i2cTriggerType < PMIC_FSM_I2C_TRIGGER3)
        {
            status = Pmic_fsmEnableI2cTrigger(pPmicCoreHandle,
                                              i2cTriggerType,
                                              0U);
            TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);
        }

        if(i2cTriggerType != PMIC_FSM_I2C_TRIGGER3)
        {
            status = Pmic_fsmEnableI2cTrigger(pPmicCoreHandle,
                                              i2cTriggerType,
                                              2U);
            TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);
        }
        else
        {
            status = Pmic_fsmEnableI2cTrigger(pPmicCoreHandle,
                                              i2cTriggerType,
                                              2U);
            TEST_ASSERT_EQUAL(PMIC_ST_ERR_NOT_SUPPORTED, status);
        }
    }

    pmic_testResultUpdate_pass(10116,
                               pmic_fsm_tests,
                               PMIC_FSM_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_fsmEnableI2cTrigger: Parameter Range validation for i2c Trigger type
 */
static void test_pmic_fsmEnableI2cTrigger_prmValTest_i2cTriggerType(void)
{
    int32_t status     = PMIC_ST_SUCCESS;

    test_pmic_print_unity_testcase_info(10117,
                                        pmic_fsm_tests,
                                        PMIC_FSM_NUM_OF_TESTCASES);

    status = Pmic_fsmEnableI2cTrigger(pPmicCoreHandle,
                                      8U,
                                      PMIC_FSM_I2C_TRIGGER_VAL_1);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

    pmic_testResultUpdate_pass(10117,
                               pmic_fsm_tests,
                               PMIC_FSM_NUM_OF_TESTCASES);
}

#if defined(ENABLE_SAMPLE_TESTCASES)
/*!
 * Below test cases are not tested as the PMIC Team team has verified the
 * Voltage rails
 * Added below test cases as sample for reference.
 */

/*!
 * \brief   Pmic_fsmRequestDdrGpioRetentionMode : DDR Retention mode with trigger value as '1'
 *          Configured to MCU State using NSleep Signal
 */
static void test_pmic_fsmRequestDdrGpioRetentionMode_ddrRmi2CTriggerVal1(void)
{
    int32_t status     = PMIC_ST_SUCCESS;

    test_pmic_print_unity_testcase_info(0xAB2E,
                                        pmic_fsm_tests,
                                        PMIC_FSM_NUM_OF_TESTCASES);

    status = Pmic_fsmRequestDdrGpioRetentionMode(pPmicCoreHandle,
                                                 PMIC_FSM_DDR_RETENTION_MODE,
                                                 PMIC_FSM_I2C_TRIGGER_VAL_1);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_fsmSetNsleepSignalVal(pPmicCoreHandle,
                                        PMIC_NSLEEP1_SIGNAL,
                                        PMIC_NSLEEP_LOW);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    pmic_testResultUpdate_pass(0xAB2E,
                               pmic_fsm_tests,
                               PMIC_FSM_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_fsmRequestDdrGpioRetentionMode : DDR Retention mode with trigger value as '0'
 *          Configured to MCU State using NSleep Signal
 */
static void test_pmic_fsmRequestDdrGpioRetentionMode_ddrRmi2CTriggerVal0(void)
{
    int32_t status     = PMIC_ST_SUCCESS;

    test_pmic_print_unity_testcase_info(0xAB2F,
                                        pmic_fsm_tests,
                                        PMIC_FSM_NUM_OF_TESTCASES);

    status = Pmic_fsmRequestDdrGpioRetentionMode(pPmicCoreHandle,
                                                 PMIC_FSM_DDR_RETENTION_MODE,
                                                 PMIC_FSM_I2C_TRIGGER_VAL_0);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_fsmSetNsleepSignalVal(pPmicCoreHandle,
                                        PMIC_NSLEEP1_SIGNAL,
                                        PMIC_NSLEEP_LOW);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);


    pmic_testResultUpdate_pass(0xAB2F,
                               pmic_fsm_tests,
                               PMIC_FSM_NUM_OF_TESTCASES);
}

#if defined(SOC_J7200)
/*!
 * \brief   Pmic_fsmRequestDdrGpioRetentionMode : GPIO Retention mode with trigger value as '1'
 *          Configured to MCU State using NSleep Signal
 */
static void test_pmic_fsmRequestDdrGpioRetentionMode_gpioRmi2CTriggerVal1(void)
{
    int32_t status     = PMIC_ST_SUCCESS;

    test_pmic_print_unity_testcase_info(0xAB30,
                                        pmic_fsm_tests,
                                        PMIC_FSM_NUM_OF_TESTCASES);

    status = Pmic_fsmRequestDdrGpioRetentionMode(pPmicCoreHandle,
                                                 PMIC_FSM_GPIO_RETENTION_MODE,
                                                 PMIC_FSM_I2C_TRIGGER_VAL_1);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_fsmSetNsleepSignalVal(pPmicCoreHandle,
                                        PMIC_NSLEEP1_SIGNAL,
                                        PMIC_NSLEEP_LOW);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    pmic_testResultUpdate_pass(0xAB30,
                               pmic_fsm_tests,
                               PMIC_FSM_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_fsmRequestDdrGpioRetentionMode : GPIO Retention mode with trigger value as '0'
 *          Configured to MCU State using NSleep Signal
 */
static void test_pmic_fsmRequestDdrGpioRetentionMode_gpioRmi2CTriggerVal0(void)
{
    int32_t status     = PMIC_ST_SUCCESS;

    test_pmic_print_unity_testcase_info(0xAB31,
                                        pmic_fsm_tests,
                                        PMIC_FSM_NUM_OF_TESTCASES);

    status = Pmic_fsmRequestDdrGpioRetentionMode(pPmicCoreHandle,
                                                 PMIC_FSM_GPIO_RETENTION_MODE,
                                                 PMIC_FSM_I2C_TRIGGER_VAL_0);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_fsmSetNsleepSignalVal(pPmicCoreHandle,
                                        PMIC_NSLEEP1_SIGNAL,
                                        PMIC_NSLEEP_LOW);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    pmic_testResultUpdate_pass(0xAB31,
                               pmic_fsm_tests,
                               PMIC_FSM_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_fsmRequestDdrGpioRetentionMode : GPIO Retention mode with trigger value as '1'
 *          Configured to S2R State using NSleep Signal
 */
static void test_pmic_fsmRequestDdrGpioRetentionMode_gpioRmi2CTriggerVal1_s2R(void)
{
    int32_t status     = PMIC_ST_SUCCESS;

    test_pmic_print_unity_testcase_info(0xAB32,
                                        pmic_fsm_tests,
                                        PMIC_FSM_NUM_OF_TESTCASES);

    status = Pmic_fsmRequestDdrGpioRetentionMode(pPmicCoreHandle,
                                                 PMIC_FSM_GPIO_RETENTION_MODE,
                                                 PMIC_FSM_I2C_TRIGGER_VAL_1);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_fsmSetNsleepSignalVal(pPmicCoreHandle,
                                        PMIC_NSLEEP2_SIGNAL,
                                        PMIC_NSLEEP_LOW);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    pmic_testResultUpdate_pass(0xAB32,
                               pmic_fsm_tests,
                               PMIC_FSM_NUM_OF_TESTCASES);
}
#endif

/*!
 * \brief   Pmic_fsmRequestDdrGpioRetentionMode : DDR Retention mode with trigger value as '1'
 *          Configured to S2R State using NSleep Signal
 */
static void test_pmic_fsmRequestDdrGpioRetentionMode_ddrRmi2CTriggerVal1_s2R(void)
{
    int32_t status     = PMIC_ST_SUCCESS;

    test_pmic_print_unity_testcase_info(0xAB33,
                                        pmic_fsm_tests,
                                        PMIC_FSM_NUM_OF_TESTCASES);

    status = Pmic_fsmRequestDdrGpioRetentionMode(pPmicCoreHandle,
                                                 PMIC_FSM_DDR_RETENTION_MODE,
                                                 PMIC_FSM_I2C_TRIGGER_VAL_1);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_fsmSetNsleepSignalVal(pPmicCoreHandle,
                                        PMIC_NSLEEP2_SIGNAL,
                                        PMIC_NSLEEP_LOW);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    pmic_testResultUpdate_pass(0xAB33,
                               pmic_fsm_tests,
                               PMIC_FSM_NUM_OF_TESTCASES);
}

#endif

/*!
 * \brief   Pmic_fsmRequestDdrGpioRetentionMode : Parameter validation for handle
 */
static void test_pmic_fsmRequestDdrGpioRetentionMode_prmValTest_handle(void)
{
    int32_t status     = PMIC_ST_SUCCESS;

    test_pmic_print_unity_testcase_info(10122,
                                        pmic_fsm_tests,
                                        PMIC_FSM_NUM_OF_TESTCASES);

    status = Pmic_fsmRequestDdrGpioRetentionMode(NULL,
                                                 PMIC_FSM_GPIO_RETENTION_MODE,
                                                 PMIC_FSM_I2C_TRIGGER_VAL_0);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, status);

    pmic_testResultUpdate_pass(10122,
                               pmic_fsm_tests,
                               PMIC_FSM_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_fsmRequestDdrGpioRetentionMode : Parameter Range validation for retentionMode
 */
static void test_pmic_fsmRequestDdrGpioRetentionMode_prmValTest_retentionMode(void)
{
    int32_t status     = PMIC_ST_SUCCESS;

    test_pmic_print_unity_testcase_info(10123,
                                        pmic_fsm_tests,
                                        PMIC_FSM_NUM_OF_TESTCASES);

    status = Pmic_fsmRequestDdrGpioRetentionMode(pPmicCoreHandle,
                                                 2U,
                                                 PMIC_FSM_I2C_TRIGGER_VAL_0);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

    pmic_testResultUpdate_pass(10123,
                               pmic_fsm_tests,
                               PMIC_FSM_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_fsmRequestDdrGpioRetentionMode : Parameter Range validation for i2cTriggerVal
 */
static void test_pmic_fsmRequestDdrGpioRetentionMode_prmValTest_i2cTriggerVal(void)
{
    int32_t status     = PMIC_ST_SUCCESS;

    test_pmic_print_unity_testcase_info(10124,
                                        pmic_fsm_tests,
                                        PMIC_FSM_NUM_OF_TESTCASES);

    status = Pmic_fsmRequestDdrGpioRetentionMode(pPmicCoreHandle,
                                                 PMIC_FSM_GPIO_RETENTION_MODE,
                                                 2U);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

    pmic_testResultUpdate_pass(10124,
                               pmic_fsm_tests,
                               PMIC_FSM_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_fsmSetConfiguration/Pmic_fsmGetConfiguration : Disable Fast BIST
 */
static void test_pmic_fsmSetConfiguration_fastBistDisable(void)
{
    int32_t status     = PMIC_ST_SUCCESS;

    Pmic_FsmCfg_t fsmCfg_rd = {PMIC_FSM_CFG_FAST_BIST_EN_VALID_SHIFT,};
    Pmic_FsmCfg_t fsmCfg =
    {
        PMIC_FSM_CFG_FAST_BIST_EN_VALID_SHIFT,
        PMIC_FSM_FAST_BIST_DISABLE,
        PMIC_FSM_SELECT_STANDBY_STATE,
        PMIC_FSM_ILIM_INT_FSMCTRL_DISABLE,
        PMIC_FSM_STARTUPDEST_ACTIVE
    };

    test_pmic_print_unity_testcase_info(10125,
                                        pmic_fsm_tests,
                                        PMIC_FSM_NUM_OF_TESTCASES);

    status = Pmic_fsmSetConfiguration(pPmicCoreHandle, fsmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_fsmGetConfiguration(pPmicCoreHandle, &fsmCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    TEST_ASSERT_EQUAL(fsmCfg.fastBistEn, fsmCfg_rd.fastBistEn);

    pmic_testResultUpdate_pass(10125,
                               pmic_fsm_tests,
                               PMIC_FSM_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_fsmRuntimeBistRequest : Negative test for Runtime BIST on PG1.0 Silicon Revision
 */
static void test_pmic_fsmRuntimeBistRequest_pmicDevSiliconRev(void)
{
    int32_t status     = PMIC_ST_SUCCESS;

    test_pmic_print_unity_testcase_info(10740,
                                        pmic_fsm_tests,
                                        PMIC_FSM_NUM_OF_TESTCASES);

    if(PMIC_SILICON_REV_ID_PG_2_0 == pPmicCoreHandle->pmicDevSiliconRev)
    {
        pmic_testResultUpdate_ignore(10740,
                                     pmic_fsm_tests,
                                     PMIC_FSM_NUM_OF_TESTCASES);
    }

    status = Pmic_fsmRequestRuntimeBist(pPmicCoreHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NOT_SUPPORTED, status);

    pmic_testResultUpdate_pass(10740,
                               pmic_fsm_tests,
                               PMIC_FSM_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_fsmDeviceOffRequestCfg : configure eventType as PMIC_FSM_ENABLE_PIN_TYPE
 */
static void test_pmic_fsmDeviceOffRequestCfg_enablePinType(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t fsmState = PMIC_FSM_STANBY_STATE;
    uint8_t eventType;

    eventType = PMIC_FSM_ENABLE_PIN_TYPE;
    test_pmic_print_unity_testcase_info(10741,
                                        pmic_fsm_tests,
                                        PMIC_FSM_NUM_OF_TESTCASES);

    status = Pmic_fsmDeviceOffRequestCfg(pPmicCoreHandle,
                                         eventType,
                                         fsmState);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    pmic_testResultUpdate_pass(10741,
                               pmic_fsm_tests,
                               PMIC_FSM_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_fsmEnableI2cTrigger: Parameter validation for i2c3 Trigger Value
 */
static void test_pmic_fsmEnableI2cTriggerPrmValTest_i2c3TriggerVal(void)
{
    int32_t status     = PMIC_ST_SUCCESS;

    test_pmic_print_unity_testcase_info(10742,
                                        pmic_fsm_tests,
                                        PMIC_FSM_NUM_OF_TESTCASES);

    status = Pmic_fsmEnableI2cTrigger(pPmicCoreHandle,
                                      PMIC_FSM_I2C_TRIGGER3,
                                      PMIC_FSM_I2C_TRIGGER_VAL_1);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NOT_SUPPORTED, status);

    pmic_testResultUpdate_pass(10742,
                               pmic_fsm_tests,
                               PMIC_FSM_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_fsmEnableI2cTrigger: Negative test to trigger i2c2 Trigger type FSM I2C trigger on PG1.0 Silicon Revision
 */
static void test_pmic_fsmEnableI2cTrigger_i2c2TriggerTypePmicDevSiliconRev(void)
{
    int32_t status     = PMIC_ST_SUCCESS;

    test_pmic_print_unity_testcase_info(10743,
                                        pmic_fsm_tests,
                                        PMIC_FSM_NUM_OF_TESTCASES);

    if(PMIC_SILICON_REV_ID_PG_2_0 == pPmicCoreHandle->pmicDevSiliconRev)
    {
        pmic_testResultUpdate_ignore(10743,
                                     pmic_fsm_tests,
                                     PMIC_FSM_NUM_OF_TESTCASES);
    }

    status = Pmic_fsmEnableI2cTrigger(pPmicCoreHandle,
                                      PMIC_FSM_I2C_TRIGGER2,
                                      PMIC_FSM_I2C_TRIGGER_VAL_1);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NOT_SUPPORTED, status);

    pmic_testResultUpdate_pass(10743,
                               pmic_fsm_tests,
                               PMIC_FSM_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_fsmEnableI2cTrigger: Parameter validation for i2c0 Trigger Value
 */
static void test_pmic_fsmEnableI2cTriggerPrmValTest_i2c0TriggerVal(void)
{
    int32_t status     = PMIC_ST_SUCCESS;

    test_pmic_print_unity_testcase_info(10744,
                                        pmic_fsm_tests,
                                        PMIC_FSM_NUM_OF_TESTCASES);

    status = Pmic_fsmEnableI2cTrigger(pPmicCoreHandle,
                                      PMIC_FSM_I2C_TRIGGER0,
                                      PMIC_FSM_I2C_TRIGGER_VAL_0);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

    pmic_testResultUpdate_pass(10744,
                               pmic_fsm_tests,
                               PMIC_FSM_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_fsmGetI2cTriggerVal: Parameter validation for handle
 */
static void test_pmic_fsmGetI2cTriggerValPrmValTest_handle(void)
{
    int32_t status     = PMIC_ST_SUCCESS;
    uint8_t i2cTriggerVal;

    test_pmic_print_unity_testcase_info(10745,
                                        pmic_fsm_tests,
                                        PMIC_FSM_NUM_OF_TESTCASES);

    status = Pmic_fsmGetI2cTriggerVal(NULL,
                                      PMIC_FSM_I2C_TRIGGER0,
                                      &i2cTriggerVal);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, status);

    pmic_testResultUpdate_pass(10745,
                               pmic_fsm_tests,
                               PMIC_FSM_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_fsmGetI2cTriggerVal: Parameter validation for pI2cTriggerVal
 */
static void test_pmic_fsmGetI2cTriggerValPrmValTest_pI2cTriggerVal(void)
{
    int32_t status     = PMIC_ST_SUCCESS;

    test_pmic_print_unity_testcase_info(10746,
                                        pmic_fsm_tests,
                                        PMIC_FSM_NUM_OF_TESTCASES);

    status = Pmic_fsmGetI2cTriggerVal(pPmicCoreHandle,
                                      PMIC_FSM_I2C_TRIGGER0,
                                      NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    pmic_testResultUpdate_pass(10746,
                               pmic_fsm_tests,
                               PMIC_FSM_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_fsmGetI2cTriggerVal: Parameter validation for i2cTriggerType
 */
static void test_pmic_fsmGetI2cTriggerValPrmValTest_i2cTriggerType(void)
{
    int32_t status     = PMIC_ST_SUCCESS;
    uint8_t i2cTriggerVal;

    test_pmic_print_unity_testcase_info(10747,
                                        pmic_fsm_tests,
                                        PMIC_FSM_NUM_OF_TESTCASES);

    status = Pmic_fsmGetI2cTriggerVal(pPmicCoreHandle,
                                      8U,
                                      &i2cTriggerVal);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

    pmic_testResultUpdate_pass(10747,
                               pmic_fsm_tests,
                               PMIC_FSM_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_fsmRequestDdrGpioRetentionMode : Configure DDR Retention mode with trigger value as '1'
 *          This Test is defined only for code coverage but not for functionality test
 */
static void test_pmic_fsmRequestDdrGpioRetentionMode_ddrRmi2CTriggerVal1Cfg(void)
{
    int32_t status     = PMIC_ST_SUCCESS;
    uint8_t i2cTriggerVal;

    test_pmic_print_unity_testcase_info(10748,
                                        pmic_fsm_tests,
                                        PMIC_FSM_NUM_OF_TESTCASES);

    status = Pmic_fsmRequestDdrGpioRetentionMode(pPmicCoreHandle,
                                                 PMIC_FSM_DDR_RETENTION_MODE,
                                                 PMIC_FSM_I2C_TRIGGER_VAL_1);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_fsmGetI2cTriggerVal(pPmicCoreHandle,
                                      PMIC_FSM_I2C_TRIGGER7,
                                      &i2cTriggerVal);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(i2cTriggerVal, PMIC_FSM_I2C_TRIGGER_VAL_1);

    pmic_testResultUpdate_pass(10748,
                               pmic_fsm_tests,
                               PMIC_FSM_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_fsmRecoverSocPwrErr: Negative test to configure nsleepVal as Low for PG1.0 Silicon Revision
 */
static void test_pmic_fsmRecoverSocPwrErr_nsleepVal(void)
{
    int32_t status     = PMIC_ST_SUCCESS;

    test_pmic_print_unity_testcase_info(10749,
                                        pmic_fsm_tests,
                                        PMIC_FSM_NUM_OF_TESTCASES);

    if(PMIC_SILICON_REV_ID_PG_2_0 == pPmicCoreHandle->pmicDevSiliconRev)
    {
        pmic_testResultUpdate_ignore(10749,
                                     pmic_fsm_tests,
                                     PMIC_FSM_NUM_OF_TESTCASES);
    }

    status = Pmic_fsmRecoverSocPwrErr(pPmicCoreHandle, PMIC_NSLEEP_LOW);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NOT_SUPPORTED, status);

    pmic_testResultUpdate_pass(10749,
                               pmic_fsm_tests,
                               PMIC_FSM_NUM_OF_TESTCASES);
}

/*!
 * \brief   Added for Coverage
 */
static void test_pmic_fsm_coverageGaps(void)
{
    int32_t status     = PMIC_ST_SUCCESS;
    uint8_t fsmState, eventType, nsleepVal;
    uint8_t pmicState = 0U;
    uint8_t i2cTriggerVal;

    Pmic_FsmCfg_t fsmCfg_rd = {PMIC_FSM_CFG_FSM_STARTUP_DEST_SEL_VALID_SHIFT,};
    Pmic_FsmCfg_t fsmCfg =
    {
        PMIC_FSM_CFG_FSM_STARTUP_DEST_SEL_VALID_SHIFT,
        PMIC_FSM_FAST_BIST_DISABLE,
        PMIC_FSM_SELECT_STANDBY_STATE,
        PMIC_FSM_ILIM_INT_FSMCTRL_DISABLE,
        PMIC_FSM_STARTUPDEST_ACTIVE
    };

    test_pmic_print_unity_testcase_info(10750,
                                        pmic_fsm_tests,
                                        PMIC_FSM_NUM_OF_TESTCASES);

    gPmic_faultInjectCfg.enableFaultInjectionRead = 1U;

    //Pmic_fsmSetStartupDestState
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    status = Pmic_fsmSetConfiguration(pPmicCoreHandle, fsmCfg);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    //Pmic_fsmGetStartupDestStateCfg
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    status = Pmic_fsmGetConfiguration(pPmicCoreHandle, &fsmCfg_rd);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    //Pmic_fsmEnableBuckLdoIlimIntAffectFsm
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    fsmCfg.validParams = PMIC_FSM_CFG_ILIM_INT_FSMCTRL_EN_VALID_SHIFT;
    fsmCfg.ilimIntfsmCtrlEn = PMIC_FSM_ILIM_INT_FSMCTRL_ENABLE;
    status = Pmic_fsmSetConfiguration(pPmicCoreHandle, fsmCfg);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    //Pmic_fsmGetBuckLdoIlimIntAffectFsmCfg
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    fsmCfg_rd.validParams = PMIC_FSM_CFG_ILIM_INT_FSMCTRL_EN_VALID_SHIFT;
    status = Pmic_fsmGetConfiguration(pPmicCoreHandle, &fsmCfg_rd);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    //Pmic_fsmSetLpStandbyState
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    fsmCfg.validParams = PMIC_FSM_CFG_LP_STANDBYSEL_VALID_SHIFT;
    fsmCfg.lpStandbySel = PMIC_FSM_SELECT_LPSTANDBY_STATE;
    status = Pmic_fsmSetConfiguration(pPmicCoreHandle, fsmCfg);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    //Pmic_fsmGetLpStandbyStateCfg
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    fsmCfg_rd.validParams = PMIC_FSM_CFG_LP_STANDBYSEL_VALID_SHIFT;
    status = Pmic_fsmGetConfiguration(pPmicCoreHandle, &fsmCfg_rd);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    //Pmic_fsmEnableFastBIST
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    fsmCfg.validParams = PMIC_FSM_CFG_FAST_BIST_EN_VALID_SHIFT;
    fsmCfg.fastBistEn = PMIC_FSM_FAST_BIST_DISABLE;
    status = Pmic_fsmSetConfiguration(pPmicCoreHandle, fsmCfg);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    //Pmic_fsmGetFastBISTCfg
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    fsmCfg_rd.validParams = PMIC_FSM_CFG_FAST_BIST_EN_VALID_SHIFT;
    status = Pmic_fsmGetConfiguration(pPmicCoreHandle, &fsmCfg_rd);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    //Pmic_fsmSetNsleepSignalMask
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    status = Pmic_fsmSetNsleepSignalMask(pPmicCoreHandle,
                                         PMIC_NSLEEP1_SIGNAL,
                                         PMIC_NSLEEPX_MASK);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    //Pmic_fsmDeviceOnRequest
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    status = Pmic_fsmDeviceOnRequest(pPmicCoreHandle);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    //Pmic_fsmDeviceOffRequestCfg
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    eventType = PMIC_FSM_NPWRON_PIN_TYPE;
    fsmState = PMIC_FSM_LP_STANBY_STATE;
    status = Pmic_fsmDeviceOffRequestCfg(pPmicCoreHandle,
                                         eventType,
                                         fsmState);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    //Pmic_fsmEnableI2cTrigger
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    status = Pmic_fsmEnableI2cTrigger(pPmicCoreHandle,
                                      PMIC_FSM_I2C_TRIGGER4,
                                      PMIC_FSM_I2C_TRIGGER_VAL_1);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    //Pmic_fsmGetI2cTriggerVal
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    status = Pmic_fsmGetI2cTriggerVal(pPmicCoreHandle,
                                      PMIC_FSM_I2C_TRIGGER0,
                                      &i2cTriggerVal);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    //Pmic_fsmSetNsleepSignalVal
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    status = Pmic_fsmSetNsleepSignalVal(pPmicCoreHandle,
                                        PMIC_NSLEEP2_SIGNAL,
                                        PMIC_NSLEEP_HIGH);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    //Pmic_fsmGetNsleepSignalVal
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    status = Pmic_fsmGetNsleepSignalVal(pPmicCoreHandle,
                                        PMIC_NSLEEP2_SIGNAL,
                                        &nsleepVal);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    //Pmic_setS2RState
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    pmicState = PMIC_FSM_S2R_STATE;
    status = Pmic_fsmSetMissionState(pPmicCoreHandle, pmicState);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    //Pmic_setState
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    pmicState = PMIC_FSM_ACTIVE_STATE;
    status = Pmic_fsmSetMissionState(pPmicCoreHandle, pmicState);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    //Pmic_setState
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    pmicState = PMIC_FSM_MCU_ONLY_STATE;
    status = Pmic_fsmSetMissionState(pPmicCoreHandle, pmicState);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    gPmic_faultInjectCfg.enableFaultInjectionRead = 0U;

    gMissionStateTestFlag = 1U;

    status = Pmic_fsmSetNsleepSignalMask(pPmicCoreHandle,
                                         PMIC_NSLEEP1_SIGNAL,
                                         PMIC_NSLEEPX_UNMASK);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_fsmSetNsleepSignalMask(pPmicCoreHandle,
                                         PMIC_NSLEEP2_SIGNAL,
                                         PMIC_NSLEEPX_UNMASK);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    pmicState = PMIC_FSM_S2R_STATE;
    status = Pmic_fsmSetMissionState(pPmicCoreHandle, pmicState);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    pmicState = PMIC_FSM_MCU_ONLY_STATE;
    status = Pmic_fsmSetMissionState(pPmicCoreHandle, pmicState);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    pmicState = PMIC_FSM_STANBY_STATE;
    status = Pmic_fsmSetMissionState(pPmicCoreHandle, pmicState);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    pmicState = PMIC_FSM_LP_STANBY_STATE;
    status = Pmic_fsmSetMissionState(pPmicCoreHandle, pmicState);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    gMissionStateTestFlag = 0U;

    pmic_testResultUpdate_pass(10750,
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

    RUN_TEST(test_pmic_fsmSetNsleepSignalMask_mask_nsleep1);
    RUN_TEST(test_pmic_fsmSetNsleepSignalMask_unmask_nsleep1);
    RUN_TEST(test_pmic_fsmSetNsleepSignalMask_mask_nsleep2);
    RUN_TEST(test_pmic_fsmSetNsleepSignalMask_unmask_nsleep2);
    RUN_TEST(test_pmic_fsmSetMissionState_active);
    RUN_TEST(test_pmic_fsmSetMissionStatePrmValTest_handle);
    RUN_TEST(test_pmic_fsmSetMissionStatePrmValTest_state);
    RUN_TEST(test_pmic_fsmSetNsleepSignalMaskStatePrmValTest_handle);
    RUN_TEST(test_pmic_fsmDevOffReqCfg_PrmValTest_handle);
    RUN_TEST(test_pmic_fsmDevOffReqCfg_PrmValTest_eventType);
    RUN_TEST(test_pmic_fsmDevOffReqCfg_PrmValTest_fsmState);
    RUN_TEST(test_pmic_fsmRuntimeBistRequestPrmValTest_handle);
    RUN_TEST(test_pmic_fsmDeviceOnRequestPrmValTest_handle);
    RUN_TEST(test_pmic_fsmGetNsleepSignalMaskStatePrmValTest_handle);
    RUN_TEST(test_pmic_fsmGetNsleepSignalMaskStatePrmValTest_nsleepStat);
    RUN_TEST(test_pmic_fsmSetConfiguration_fastBistEnable);
    RUN_TEST(test_pmic_fsmSetConfiguration_selectLpStandbyState);
    RUN_TEST(test_pmic_fsmSetConfiguration_selectStandbyState);
    RUN_TEST(test_pmic_fsmSetConfiguration_ilimIntfsmCtrlEnable);
    RUN_TEST(test_pmic_fsmSetConfiguration_ilimIntfsmCtrlDisable);
    RUN_TEST(test_pmic_fsmSetConfiguration_selectfsmStarupDestActiveState);
    RUN_TEST(test_pmic_fsmSetConfiguration_selectfsmStarupDestMcuonlyState);
    RUN_TEST(test_pmic_fsmSetConfiguration_selectfsmStarupDestStandbyOrLpStandbyState);
    RUN_TEST(test_pmic_fsmSetConfiguration_prmValTest_handle);
    RUN_TEST(test_pmic_fsmSetConfiguration_prmValTest_fsmStarupDestSel);
    RUN_TEST(test_pmic_fsmGetConfiguration_prmValTest_handle);
    RUN_TEST(test_pmic_fsmGetConfiguration_prmValTest_fsmCfg);
    RUN_TEST(test_pmic_fsmSetPfsmDelay);
    RUN_TEST(test_pmic_fsmSetPfsmDelay_prmValTest_handle);
    RUN_TEST(test_pmic_fsmSetPfsmDelay_prmValTest_fsmStarupDestSel);
    RUN_TEST(test_pmic_fsmGetPfsmDelay_prmValTest_handle);
    RUN_TEST(test_pmic_fsmGetPfsmDelay_prmValTest_fsmStarupDestSel);
    RUN_TEST(test_pmic_fsmGetPfsmDelay_prmValTest_PfsmDelay);
    RUN_TEST(test_pmic_fsmSetNsleepSignalVal_nsleep1High);
    RUN_TEST(test_pmic_fsmSetNsleepSignalVal_nsleep2High);
    RUN_TEST(test_pmic_fsmSetNsleepSignalVal_prmValTest_handle);
    RUN_TEST(test_pmic_fsmSetNsleepSignalVal_prmValTest_nsleepVal);
    RUN_TEST(test_pmic_fsmGetNsleepSignalVal_prmValTest_handle);
    RUN_TEST(test_pmic_fsmGetNsleepSignalVal_prmValTest_nsleepVal);

#if 0 // TBD
    RUN_TEST(test_pmic_fsmRecoverSocPwrErr_nsleepLow);
    RUN_TEST(test_pmic_fsmRecoverSocPwrErr_nsleepHigh);
#endif

    RUN_TEST(test_pmic_fsmRecoverSocPwrErr_prmValTest_handle);
    RUN_TEST(test_pmic_fsmRecoverSocPwrErr_prmValTest_nsleepVal);

    RUN_TEST(test_pmic_fsmEnableI2cTrigger_i2C4);
    RUN_TEST(test_pmic_fsmEnableI2cTrigger_i2C5);
    RUN_TEST(test_pmic_fsmEnableI2cTrigger_i2C6);
    RUN_TEST(test_pmic_fsmEnableI2cTrigger_i2C7);
    RUN_TEST(test_pmic_fsmGetI2cTriggerVal_i2C0_i2c1_i2c2_i2c3);

    RUN_TEST(test_pmic_fsmEnableI2cTrigger_prmValTest_handle);
    RUN_TEST(test_pmic_fsmEnableI2cTrigger_prmValTest_i2cTriggerVal);
    RUN_TEST(test_pmic_fsmEnableI2cTrigger_prmValTest_i2cTriggerType);
    RUN_TEST(test_pmic_fsmRequestDdrGpioRetentionMode_prmValTest_handle);
    RUN_TEST(test_pmic_fsmRequestDdrGpioRetentionMode_prmValTest_retentionMode);
    RUN_TEST(test_pmic_fsmRequestDdrGpioRetentionMode_prmValTest_i2cTriggerVal);
    RUN_TEST(test_pmic_fsmSetConfiguration_fastBistDisable);
    RUN_TEST(test_pmic_fsmRuntimeBistRequest_pmicDevSiliconRev);
    RUN_TEST(test_pmic_fsmDeviceOffRequestCfg_enablePinType);
    RUN_TEST(test_pmic_fsmEnableI2cTriggerPrmValTest_i2c3TriggerVal);
    RUN_TEST(test_pmic_fsmEnableI2cTrigger_i2c2TriggerTypePmicDevSiliconRev);
    RUN_TEST(test_pmic_fsmEnableI2cTriggerPrmValTest_i2c0TriggerVal);
    RUN_TEST(test_pmic_fsmGetI2cTriggerValPrmValTest_handle);
    RUN_TEST(test_pmic_fsmGetI2cTriggerValPrmValTest_pI2cTriggerVal);
    RUN_TEST(test_pmic_fsmGetI2cTriggerValPrmValTest_i2cTriggerType);
    RUN_TEST(test_pmic_fsmRequestDdrGpioRetentionMode_ddrRmi2CTriggerVal1Cfg);
    RUN_TEST(test_pmic_fsmRecoverSocPwrErr_nsleepVal);
    RUN_TEST(test_pmic_fsm_coverageGaps);

    pmic_updateTestResults(pmic_fsm_tests, PMIC_FSM_NUM_OF_TESTCASES);

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
 * \brief   FSM Unity Test App wrapper Function for LEO PMIC-A
 */
static int32_t test_pmic_leo_pmicA_spiStub_fsm_testApp(void)
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
        if(PMIC_STATUS_CRC_INIT_VAL == gCrcTestFlag_J7VCL)
        {
            gCrcTestFlag_J7VCL = PMIC_CFG_TO_ENABLE_CRC;
        }

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

static const char pmicTestMenu[] =
{
    " \r\n ================================================================="
    " \r\n Test Menu:"
    " \r\n ================================================================="
    " \r\n 0: Automatic run for all board specific FSM options"
    " \r\n 1: Manual run for FSM options"
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
    " \r\n 2: Pmic Hera device(PMIC B on J7VCL EVM)"
    " \r\n 3: Pmic Leo device(PMIC A on J721E EVM Using SPI Stub Functions)"
    " \r\n 4: Pmic Leo device(PMIC A on J7VCL EVM Using SPI Stub Functions)"
    " \r\n 5: Pmic Leo device(PMIC A on J721E EVM Manual Testcase for FSM states)"
    " \r\n 6: Pmic Leo device(PMIC A on J7VCL EVM Manual Testcase for FSM states)"
    " \r\n 7: Back to Test Menu"
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
    pmic_log(" \r\n 0: Pmic Leo device(PMIC A on %s EVM Set FSM Mission States - S2R", board_name);
    pmic_log(" \r\n 1: Pmic Leo device(PMIC A on %s EVM Set FSM Mission States - lpStandby", board_name);
    pmic_log(" \r\n 2: Pmic Leo device(PMIC A on %s EVM Set FSM Mission States - Standby", board_name);
    pmic_log(" \r\n 3: Pmic Leo device(PMIC A on %s EVM Set nSleep1 Signal - Active Low", board_name);
    pmic_log(" \r\n 4: Pmic Leo device(PMIC A on %s EVM Set nSleep2 Signal - Active Low", board_name);
    pmic_log(" \r\n 5: Back to Main Menu");
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

        if(menuOption == 5)
        {
            break;
        }

        switch(menuOption)
        {
            case 0U:
                RUN_TEST(test_pmic_fsmSetMissionState_s2r);
               break;
            case 1U:
                RUN_TEST(test_pmic_fsmSetMissionState_lpstandby);
               break;
            case 2U:
                RUN_TEST(test_pmic_fsmSetMissionState_standby);
               break;
            case 3U:
                RUN_TEST(test_pmic_fsmSetNsleepSignalVal_nsleep1Low);
               break;
            case 4U:
                RUN_TEST(test_pmic_fsmSetNsleepSignalVal_nsleep2Low);
               break;
            default:
               pmic_log(" \r\n Invalid option... Try Again!!!\n");
               break;
        }
    }
}

volatile int8_t g_option = 0;
static void test_pmic_fsm_testapp_run_options()
{
    int8_t num = -1;
    int8_t idx = 0;
#if defined(SOC_J721E)
    int8_t automatic_options[] = {0, 3};
#elif defined(SOC_J7200)
    int8_t automatic_options[] = {1, 2, 4};
#endif

    while(1U)
    {
        if(idx >= (sizeof(automatic_options)/sizeof(automatic_options[0])))
        {
            pmic_printTestResult(pmic_fsm_tests, PMIC_FSM_NUM_OF_TESTCASES);
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
                num = 7;
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

                    /* FSM Unity Test App wrapper Function for LEO PMIC-A
                     * using SPI stub functions */
                    if(PMIC_ST_SUCCESS ==
                          test_pmic_leo_pmicA_spiStub_fsm_testApp())
                    {
                        /* Run fsm test cases for Leo PMIC-A */
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
#if defined(SOC_J7200)
                if(PMIC_ST_SUCCESS == setup_pmic_interrupt(J7VCL_BOARD))
                {
                    pmic_device_info = J7VCL_LEO_PMICA_DEVICE;
                    /* FSM Unity Test App wrapper Function for LEO PMIC-A
                     * using SPI stub functions */
                     if(PMIC_ST_SUCCESS ==
                            test_pmic_leo_pmicA_spiStub_fsm_testApp())
                    {
                        /* Run fsm test cases for Leo PMIC-A */
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
            case 6U:
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
            case 7U:
               pmic_log(" \r\n Back to Test Menu options\n");
               return;
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
                test_pmic_fsm_testapp_run_options();
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
 * \brief   TI RTOS specific FSM TEST APP main Function
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code.
 *          For valid values see \ref Pmic_ErrorCodes
 */
int main()
{
    Board_initUART();

    pmic_print_banner("PMIC FSM Unity Test Application");

#if defined(UNITY_INCLUDE_CONFIG_V2_H) && \
    (defined(SOC_J721E)             || \
     defined(SOC_J7200))
    test_pmic_fsm_testapp_runner();
#endif
}
