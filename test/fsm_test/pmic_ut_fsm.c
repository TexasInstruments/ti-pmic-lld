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
        "Pmic_fsmSetMissionState : Test Set State to LP-Standby."
    },
    {
        7701,
        "Pmic_fsmSetMissionState : Test Set State to Standby."
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
        "Pmic_fsmDeviceOffRequestCfg :  Parameter validation for 'handle'"
    },
    {
        7361,
        "Pmic_fsmDeviceOffRequestCfg :  Parameter validation for 'eventType'"
    },
    {
        7705,
        "Pmic_fsmDeviceOffRequestCfg :  Parameter validation for 'fsmState'"
    },
    {
        7706,
        "Pmic_fsmRuntimeBistRequest : Parameter validation for 'handle'."
    },
    {
        7364,
        "Pmic_fsmDeviceOnRequest : Parameter validation for 'handle'."
    },
    {
        1,
        "Pmic_fsmGetLpmControl : Parameter validation for 'handle'"
    },
    {
        2,
        "Pmic_fsmGetLpmControl : Parameter validation for 'lpmEnable'"
    },
    {
        3,
        "Pmic_fsmSetLpmControl : Parameter validation for 'handle'."
    },
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

    status = Pmic_fsmSetMissionState(pPmicCoreHandle, pmicState);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

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

    status = Pmic_fsmSetMissionState(pPmicCoreHandle, pmicState);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
}

/*!
 * \brief   Pmic_fsmSetMissionState : Test Set State to LP-Standby.
 */
static void test_Pmic_fsmSetMissionState_lpstandby(void)
{
    int32_t status    = PMIC_ST_SUCCESS;
    uint8_t pmicState = 0U;

    int8_t             timeout      = 10U;
    Pmic_CoreHandle_t  *pHandle     = NULL;
    Pmic_RtcTime_t     timeCfg_cr   = { 0x1F, 0U, 0U, 0U, 0U, 0U};
    Pmic_RtcDate_t     dateCfg_cr   = { 0x0F, 0U, 0U, 0U, 0U};
    Pmic_RtcTime_t     timeCfg_rd   = { 0x1F, 0U, 0U, 0U, 0U, 0U};
    Pmic_RtcDate_t     dateCfg_rd   = { 0x0F, 0U, 0U, 0U, 0U};
    Pmic_IrqStatus_t errStat        = {0U};
    bool clearIRQ                   = false;
    uint8_t  irqNum                 = 0U;
    uint8_t            timerPeriod  = 0U;
    Pmic_RtcDate_t    validDateCfg =  { 0x0F, 15U, 6U, 2055U, 1U};
    Pmic_RtcTime_t    validTimeCfg  = { 0x1F, 30U, 30U, 6U, 0U, 1U};
    uint32_t timerIntrdelayTime = 70000U;  // Timer Interrupt configured for 1 minute (60 Sec)
    uint32_t delayTime = 80000U;  // Added delay for workaround

    pHandle = pPmicCoreHandle;

    pmicState = PMIC_FSM_LP_STANBY_STATE;

    test_pmic_print_unity_testcase_info(7700,
                                        pmic_fsm_tests,
                                        PMIC_FSM_NUM_OF_TESTCASES);

    status = Pmic_rtcSetTimeDateInfo(pHandle, validTimeCfg, validDateCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_rtcGetTimeDateInfo(pHandle, &timeCfg_rd, &dateCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_rtcGetTimerPeriod(pHandle, &timerPeriod);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    irqNum = 0;

    status = Pmic_rtcSetTimerPeriod(pHandle,
                                           PMIC_RTC_MINUTE_INTR_PERIOD);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_rtcEnableTimerIntr(pHandle, PMIC_RTC_TIMER_INTR_ENABLE);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    /*To avoid DAP reset*/
    Osal_delay(delayTime);

    status =  Pmic_fsmSetMissionState(pPmicCoreHandle, pmicState);

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    Osal_delay(timerIntrdelayTime);

    while(timeout--)
    {
        status = Pmic_irqGetErrStatus(pHandle, &errStat, clearIRQ);
        if((PMIC_ST_SUCCESS == status) &&
           ((errStat.intStatus[PMIC_TPS6594X_RTC_TIMER_INT/32U] &
             (1U << (PMIC_TPS6594X_RTC_TIMER_INT % 32U))) != 0U))
        {
            while(PMIC_TPS6594X_RTC_TIMER_INT != irqNum)
            {
                status = Pmic_getNextErrorStatus(pHandle,
                                                 &errStat,
                                                 &irqNum);
            }

            if(PMIC_ST_SUCCESS == status)
            {
                /* clear the interrupt */
                status = Pmic_irqClrErrStatus(pPmicCoreHandle,
                                              PMIC_TPS6594X_RTC_TIMER_INT);
            }

            if(PMIC_ST_SUCCESS == status)
            {
                /* Disable the timer interrupt  */
                status = Pmic_rtcEnableTimerIntr(pHandle,
                                                 PMIC_RTC_TIMER_INTR_DISABLE);

                break;
            }
        }

        status = Pmic_rtcGetTimeDateInfo(pHandle, &timeCfg_cr, &dateCfg_cr);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    }

    status = Pmic_rtcSetTimerPeriod(pHandle, timerPeriod);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

}

/*!
 * \brief   Pmic_fsmSetMissionState : Test Set State to Standby.
 */
static void test_Pmic_fsmSetMissionState_standby(void)
{
    int32_t status    = PMIC_ST_SUCCESS;
    uint8_t pmicState = 0U;

    int8_t             timeout      = 10U;
    Pmic_CoreHandle_t  *pHandle     = NULL;
    Pmic_RtcTime_t     timeCfg_cr   = { 0x1F, 0U, 0U, 0U, 0U, 0U};
    Pmic_RtcDate_t     dateCfg_cr   = { 0x0F, 0U, 0U, 0U, 0U};
    Pmic_RtcTime_t     timeCfg_rd   = { 0x1F, 0U, 0U, 0U, 0U, 0U};
    Pmic_RtcDate_t     dateCfg_rd   = { 0x0F, 0U, 0U, 0U, 0U};
    Pmic_IrqStatus_t errStat        = {0U};
    bool clearIRQ                   = false;
    uint8_t  irqNum                 = 0U;
    uint8_t            timerPeriod  = 0U;
    Pmic_RtcDate_t    validDateCfg =  { 0x0F, 15U, 6U, 2055U, 1U};
    Pmic_RtcTime_t    validTimeCfg  = { 0x1F, 30U, 30U, 6U, 0U, 1U};
    uint32_t timerIntrdelayTime = 70000U;  // Timer Interrupt configured for 1 minute (60 Sec)
    uint32_t delayTime = 80000U;  // Added delay for workaround

    pHandle = pPmicCoreHandle;

    pmicState = PMIC_FSM_STANBY_STATE;

    test_pmic_print_unity_testcase_info(7701,
                                        pmic_fsm_tests,
                                        PMIC_FSM_NUM_OF_TESTCASES);

    status = Pmic_rtcSetTimeDateInfo(pHandle, validTimeCfg, validDateCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_rtcGetTimeDateInfo(pHandle, &timeCfg_rd, &dateCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_rtcGetTimerPeriod(pHandle, &timerPeriod);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    irqNum = 0;

    status = Pmic_rtcSetTimerPeriod(pHandle,
                                           PMIC_RTC_MINUTE_INTR_PERIOD);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_rtcEnableTimerIntr(pHandle, PMIC_RTC_TIMER_INTR_ENABLE);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    /*To avoid DAP reset*/
    Osal_delay(delayTime);

    status =  Pmic_fsmSetMissionState(pPmicCoreHandle, pmicState);

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    Osal_delay(timerIntrdelayTime);

    while(timeout--)
    {
        status = Pmic_irqGetErrStatus(pHandle, &errStat, clearIRQ);
        if((PMIC_ST_SUCCESS == status) &&
           ((errStat.intStatus[PMIC_TPS6594X_RTC_TIMER_INT/32U] &
             (1U << (PMIC_TPS6594X_RTC_TIMER_INT % 32U))) != 0U))
        {
            while(PMIC_TPS6594X_RTC_TIMER_INT != irqNum)
            {
                status = Pmic_getNextErrorStatus(pHandle,
                                                 &errStat,
                                                 &irqNum);
            }

            if(PMIC_ST_SUCCESS == status)
            {
                /* clear the interrupt */
                status = Pmic_irqClrErrStatus(pPmicCoreHandle,
                                              PMIC_TPS6594X_RTC_TIMER_INT);
            }

            if(PMIC_ST_SUCCESS == status)
            {
                /* Disable the timer interrupt  */
                status = Pmic_rtcEnableTimerIntr(pHandle,
                                                 PMIC_RTC_TIMER_INTR_DISABLE);

                break;
            }
        }

        status = Pmic_rtcGetTimeDateInfo(pHandle, &timeCfg_cr, &dateCfg_cr);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    }

    status = Pmic_rtcSetTimerPeriod(pHandle, timerPeriod);
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

}

/*!
 * \brief   Parameter validation for 'handle'
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
}

/*!
 * \brief   Pmic_fsmDeviceOffRequestCfg : Parameter validation for 'eventType'
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
}

/*!
 * \brief   Pmic_fsmDeviceOffRequestCfg :  Parameter validation for 'fsmState'
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
}

/*!
 * \brief   Pmic_fsmRuntimeBistRequest : Parameter validation for 'handle'.
 */
static void test_Pmic_fsmRuntimeBistRequestPrmValTest_handle(void)
{
    int32_t status     = PMIC_ST_SUCCESS;

    test_pmic_print_unity_testcase_info(7706,
                                        pmic_fsm_tests,
                                        PMIC_FSM_NUM_OF_TESTCASES);

    status = Pmic_fsmRuntimeBistRequest(NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, status);
}

/*!
 * \brief   Pmic_fsmDeviceOnRequest : Parameter validation for 'handle'.
 */
static void test_Pmic_Pmic_fsmDeviceOnRequestPrmValTest_handle(void)
{
    int32_t status     = PMIC_ST_SUCCESS;

    test_pmic_print_unity_testcase_info(7364,
                                        pmic_fsm_tests,
                                        PMIC_FSM_NUM_OF_TESTCASES);

    status = Pmic_fsmDeviceOnRequest(NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, status);
}

/*!
 * \brief   Pmic_fsmGetLpmControl : Parameter validation for 'handle'.
 */
static void test_Pmic_Pmic_fsmGetLpmControlPrmValTest_handle(void)
{
    int32_t status     = PMIC_ST_SUCCESS;
    bool lpmEnable;

    test_pmic_print_unity_testcase_info(1,
                                        pmic_fsm_tests,
                                        PMIC_FSM_NUM_OF_TESTCASES);

    status = Pmic_fsmGetLpmControl(NULL, &lpmEnable);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, status);
}

/*!
 * \brief   Pmic_fsmGetLpmControl : Parameter validation for 'lpmEnable'.
 */
static void test_Pmic_Pmic_fsmGetLpmControlPrmValTest_lpmEnable(void)
{
    int32_t status     = PMIC_ST_SUCCESS;

    test_pmic_print_unity_testcase_info(2,
                                        pmic_fsm_tests,
                                        PMIC_FSM_NUM_OF_TESTCASES);

    status = Pmic_fsmGetLpmControl(pPmicCoreHandle, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

/*!
 * \brief   Pmic_fsmSetLpmControl : Parameter validation for 'handle'.
 */
static void test_Pmic_Pmic_fsmSetLpmControlPrmValTest_handle(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    bool lpmEnable = PMIC_FSM_LPM_ENABLE;

    test_pmic_print_unity_testcase_info(3,
                                        pmic_fsm_tests,
                                        PMIC_FSM_NUM_OF_TESTCASES);

    status = Pmic_fsmSetLpmControl(NULL, lpmEnable);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, status);
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

    RUN_TEST(test_Pmic_fsmSetNsleepSignalMask_mask_nsleep1);
    RUN_TEST(test_Pmic_fsmSetNsleepSignalMask_unmask_nsleep1);
    RUN_TEST(test_Pmic_fsmSetNsleepSignalMask_mask_nsleep2);
    RUN_TEST(test_Pmic_fsmSetNsleepSignalMask_unmask_nsleep2);
    RUN_TEST(test_Pmic_fsmSetMissionState_active);
    RUN_TEST(test_Pmic_fsmSetMissionState_lpstandby);
    RUN_TEST(test_Pmic_fsmSetMissionState_standby);
    RUN_TEST(test_Pmic_fsmSetMissionStatePrmValTest_handle);
    RUN_TEST(test_Pmic_fsmSetMissionStatePrmValTest_state);
    RUN_TEST(test_Pmic_fsmSetNsleepSignalMaskStatePrmValTest_handle);
    RUN_TEST(test_pmic_rtc_fsmDevOffReqCfg_PrmValTest_handle);
    RUN_TEST(test_pmic_rtc_fsmDevOffReqCfg_PrmValTest_eventType);
    RUN_TEST(test_pmic_rtc_fsmDevOffReqCfg_PrmValTest_fsmState);
    RUN_TEST(test_Pmic_fsmRuntimeBistRequestPrmValTest_handle);
    RUN_TEST(test_Pmic_Pmic_fsmDeviceOnRequestPrmValTest_handle);
    RUN_TEST(test_Pmic_Pmic_fsmGetLpmControlPrmValTest_handle);
    RUN_TEST(test_Pmic_Pmic_fsmGetLpmControlPrmValTest_lpmEnable);
    RUN_TEST(test_Pmic_Pmic_fsmSetLpmControlPrmValTest_handle);

    UNITY_END();
}

/*!
 * \brief   Run fsm manual test cases
 */
static void test_pmic_run_testcases_manual(void)
{
    pmic_log("\n\n%s(): %d: Begin Unity Test Cases...\n", __func__, __LINE__);
    UNITY_BEGIN();

    RUN_TEST(test_Pmic_fsmSetMissionState_mcu);
    RUN_TEST(test_Pmic_fsmSetMissionState_s2r);

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

#ifdef SOC_J721E
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
#endif

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
 * \brief   FSM Manual Test App wrapper Function for LEO PMIC-A
 */
static int32_t test_pmic_fsm_manual_testApp(void)
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

static int32_t setup_pmic_interrupt()
{
    int32_t status = PMIC_ST_SUCCESS;

#ifdef SOC_J721E

    status = test_pmic_leo_pmicA_fsm_testApp();
   /* Deinit pmic handle */
    if((pPmicCoreHandle != NULL) && (PMIC_ST_SUCCESS == status))
    {
        test_pmic_appDeInit(pPmicCoreHandle);
    }

    if(PMIC_ST_SUCCESS == status)
    {
        status = test_pmic_leo_pmicB_fsm_testApp();
       /* Deinit pmic handle */
        if((pPmicCoreHandle != NULL) && (PMIC_ST_SUCCESS == status))
        {
            test_pmic_appDeInit(pPmicCoreHandle);
        }
    }
#endif
    return status;
}

static const char pmicTestAppMenu[] =
{
    " \r\n ================================================================="
    " \r\n Menu:"
    " \r\n ================================================================="
    " \r\n 0: Pmic Leo device(PMIC A on J721E EVM)"
    " \r\n 1: Pmic Hera device"
    " \r\n 2: Manual test: Set FSM Mission States"
    " \r\n 3: quit"
    " \r\n"
    " \r\n Enter option: "
};

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
                if(PMIC_ST_SUCCESS == setup_pmic_interrupt())
                {
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
               break;
           case 1U:
                if(PMIC_ST_SUCCESS == setup_pmic_interrupt())
                {
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
               break;
           case 2U:
                if(PMIC_ST_SUCCESS == setup_pmic_interrupt())
                {
                    /* FSM Manual Test App wrapper Function for LEO PMIC-A*/
                    if(PMIC_ST_SUCCESS == test_pmic_fsm_manual_testApp())
                    {
                       /* Run fsm manual test cases */
                       test_pmic_run_testcases_manual();
                    }
                    /* Deinit pmic handle */
                    if(pPmicCoreHandle != NULL)
                    {
                       test_pmic_appDeInit(pPmicCoreHandle);
                    }
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
    Board_initUART();

    pmic_log("PMIC FSM Unity Test Application(%s %s)\n",
                                         __TIME__, __DATE__);

#if defined(UNITY_INCLUDE_CONFIG_V2_H) && \
    (defined(SOC_J721E)             || \
     defined(SOC_J7200))
    test_pmic_fsm_testapp_runner();
#endif
}
