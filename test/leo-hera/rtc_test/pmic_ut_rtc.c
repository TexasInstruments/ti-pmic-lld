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
 *  \file   pmic_ut_rtc.c
 *
 *  \brief  PMIC Uint Test for testing PMIC RTC APIs
 *
 */

#include <pmic_ut_rtc.h>

/* Pointer to Pmic Core Handle */
Pmic_CoreHandle_t *pPmicCoreHandle = NULL;

extern uint16_t pmic_device_info;
extern int32_t gCrcTestFlag_J721E;
extern int32_t gCrcTestFlag_J7VCL;
volatile uint32_t pmic_intr_triggered;

extern int8_t gEnableRtcTestFlag;

extern Pmic_Ut_FaultInject_t gPmic_faultInjectCfg;

/*!
 * \brief   PMIC RTC Test Cases
 */
static Pmic_Ut_Tests_t pmic_rtc_tests[] =
{
    {
        5990,
        "Pmic_rtcSetAlarmInfo : Set RTC Alarm interrupt"
    },
    {
        5991,
        "Pmic_rtcSetAlarmInfo : Parameter validation for handle"
    },
    {
        6090,
        "Pmic_rtcSetAlarmInfo : Parameter validation for seconds"
    },
    {
        6091,
        "Pmic_rtcSetAlarmInfo : Parameter validation for minutes"
    },
    {
        6092,
        "Pmic_rtcSetAlarmInfo : Parameter validation for timeMode"
    },
    {
        6093,
        "Pmic_rtcSetAlarmInfo : Parameter validation for meridianMode"
    },
    {
        6094,
        "Pmic_rtcSetAlarmInfo : Parameter validation for hour when timeMode = 1"
    },
    {
        6095,
        "Pmic_rtcSetAlarmInfo : Parameter validation for hour when timeMode = 0"
    },
    {
        6096,
        "Pmic_rtcSetAlarmInfo : Negative test for hour  = 0, when timeMode = 1"
    },
    {
        6099,
        "Pmic_rtcSetAlarmInfo : Negative test for month  = 0"
    },
    {
        6100,
        "Pmic_rtcSetAlarmInfo : Negative test for day = 0"
    },
    {
        6101,
        "Pmic_rtcSetAlarmInfo : Parameter range validation for year"
    },
    {
        6102,
        "Pmic_rtcSetAlarmInfo : Parameter range validation for month"
    },
    {
        6103,
        "Pmic_rtcSetAlarmInfo : Parameter range validation for day for months with 30 days"
    },
    {
        6104,
        "Pmic_rtcSetAlarmInfo : Parameter range validation for day for leap year(year %4 = 0 ) and month = 2 (february)"
    },
    {
        6107,
        "Pmic_rtcSetAlarmInfo : Parameter range validation for day for Non-leap year(year %4 != 0 ) and month = 2 (february)"
    },
    {
        6108,
        "Pmic_rtcSetAlarmInfo : Parameter range validation for day, for months with 31 days)"
    },
    {
        6109,
        "Pmic_rtcGetAlarmInfo : Test Get RTC Alarm interrupt API"
    },
    {
        6110,
        "Pmic_rtcGetAlarmInfo : Parameter validation for handle"
    },
    {
        6111,
        "Pmic_rtcGetAlarmInfo : Parameter validation for timeCfg"
    },
    {
        6112,
        "Pmic_rtcGetAlarmInfo : Parameter validation for dataCfg"
    },
    {
        6113,
        "Pmic_rtcSetTimerPeriod : Test Set RTC Timer interrupt"
    },
    {
        6114,
        "Pmic_rtcSetTimerPeriod : Parameter validation for handle"
    },
    {
        6115,
        "Pmic_rtcSetTimerPeriod : Parameter validation for timerPeriod"
    },
    {
        6116,
        "Pmic_rtcSetTimerPeriod : Test Get RTC Timer interrupt"
    },
    {
        6117,
        "Pmic_rtcSetTimerPeriod : Parameter validation for handle"
    },
    {
        6118,
        "Pmic_rtcSetTimerPeriod : Parameter validation for timerPeriod"
    },
    {
        6119,
        "Pmic_rtcEnable : Test RTC Disable"
    },
    {
        6120,
        "Pmic_rtcEnable : Parameter validation for handle"
    },
    {
        6121,
        "Pmic_rtcEnable : Test RTC Enable"
    },
    {
        6122,
        "Pmic_rtcEnable : Parameter validation for handle"
    },
    {
        6158,
        "Pmic_rtcSetTimeDateInfo : Test RTC Set Time"
    },
    {
        6159,
        "Pmic_rtcSetTimeDateInfo : Parameter validation for handle"
    },
    {
        6162,
        "Pmic_rtcSetTimeDateInfo : Parameter validation for seconds"
    },
    {
        6163,
        "Pmic_rtcSetTimeDateInfo : Parameter validation for minutes"
    },
    {
        6164,
        "Pmic_rtcSetTimeDateInfo : Parameter validation for timeMode"
    },
    {
        6165,
        "Pmic_rtcSetTimeDateInfo : Parameter validation for meridianMode"
    },
    {
        6166,
        "Pmic_rtcSetTimeDateInfo : Parameter validation for hour when timeMode = 1"
    },
    {
        6167,
        "Pmic_rtcSetTimeDateInfo : Parameter validation for hour when timeMode = 0"
    },
    {
        6168,
        "Pmic_rtcSetTimeDateInfo : Negative test for hour  = 0, when timeMode = 1"
    },
    {
        6169,
        "Pmic_rtcSetTimeDateInfo : Negative test for month  = 0"
    },
    {
        6290,
        "Pmic_rtcSetTimeDateInfo : Negative test for day = 0"
    },
    {
        7021,
        "Pmic_rtcSetTimeDateInfo: Parameter range validation for year"
    },
    {
        6170,
        "Pmic_rtcSetTimeDateInfo : Parameter range validation for month"
    },
    {
        6171,
        "Pmic_rtcSetTimeDateInfo : Parameter range validation for day"
    },
    {
        6172,
       "Pmic_rtcSetTimeDateInfo : Parameter range validation for day for months with 30 days"
    },
    {
        6173,
        "Pmic_rtcSetTimeDateInfo : Parameter range validation for day for leap year(year %4 = 0 ) and month = 2 (february)"
    },
    {
        6174,
        "Pmic_rtcSetTimeDateInfo : Parameter range validation for day for Non-leap year(year %4 != 0 ) and month = 2 (february)"
    },
    {
        6175,
        "Pmic_rtcSetTimeDateInfo : Parameter range validation for day, for months with 31 days)"
    },
    {
        6176,
        "Pmic_rtcGetTimeDateInfo : Test RTC Get Time"
    },
    {
        6177,
        "Pmic_rtcGetTimeDateInfo : Parameter validation for handle"
    },
    {
        6178,
        "Pmic_rtcGetTimeDateInfo : Parameter validation for timeCfg"
    },
    {
        6179,
        "Pmic_rtcGetTimeDateInfo : Parameter validation for dataCfg"
    },
    {
        6180,
        "Pmic_rtcSetFreqComp : Test RTC set RTC frequency compensation"
    },
    {
        6181,
        "Pmic_rtcSetFreqComp : Parameter validation for handle"
    },
    {
        6182,
        "Pmic_rtcGetFreqComp : Test RTC get RTC frequency compensation"
    },
    {
        6183,
        "Pmic_rtcGetFreqComp : Parameter validation for handle"
    },
    {
        6287,
        "Pmic_rtcGetFreqComp : Parameter validation for compensation"
    },
    {
        6266,
        "test_rtc_timer_irq : Test rtc timer interrupt"
    },
    {
        6267,
        "test_rtc_alarm_irq : Test rtc alarm interrupt"
    },
    {
        6268,
        "Pmic_rtcEnableTimerIntr : Parameter validation for handle"
    },
    {
        6269,
        "Pmic_rtcEnableAlarmIntr : Parameter validation for handle"
    },
    {
        6194,
        "Pmic_rtcGetStatus : RTC Live Status Validation RTC Current state Running"
    },
    {
        6089,
        "Pmic_rtcGetStatus : RTC Live Status Validation RTC Current state Frozen"
    },
    {
        7465,
        "Pmic_rtcGetRstStatus : Parameter validation for handle"
    },
    {
        7466,
        "Pmic_rtcGetRstStatus : Parameter validation for rtcRstStatus"
    },
    {
        7467,
        "Pmic_rtcGetRstStatus : Parameter validation for ValidParams"
    },
    {
        7358,
        "Pmic_fsmSetMissionState : RTC Wakeup using Timer Interrupt using LP Standby State"
    },
    {
        7359,
        "Pmic_fsmSetMissionState :  RTC Wakeup using Alarm Interrupt using LP Standby State"
    },
    {
        7888,
        "test_rtc_timer_irq : Test rtc timer asynchronous interrupt"
    },
    {
        7889,
        "test_rtc_alarm_irq: Test rtc alarm asynchronous interrupt"
    },
    {
        7860,
        "Pmic_rtcEnable : Negative test for RTC state for HERA"
    },
    {
        7861,
        "Pmic_rtcEnableAlarmIntr : Negative test for RTC Alarm interrupt for HERA"
    },
    {
        7862,
        "Pmic_rtcEnableTimerIntr :Negative test for RTC Timer interrupt for HERA"
    },
    {
        7863,
        "Pmic_rtcGetFreqComp :Negative test for Test RTC for set RTC frequency compensation for HERA"
    },
    {
        7864,
        "Pmic_rtcSetTimeDateInfo : Negative test for Test RTC for Get Time for HERA"
    },
    {
        7865,
        "Pmic_rtcGetAlarmInfo : Negative test for Test RTC Get Alarm for HERA"
    },
    {
        7866,
        "Pmic_rtcSetAlarmInfo : Negative test for Set RTC Alarm interrupt for HERA"
    },
    {
        7867,
        "Pmic_rtcSetTimerPeriod : Negative test for Test Set RTC Timer interrupt Period for hera"
    },
    {
        7868,
        "Pmic_rtcGetTimerPeriod : Negative test for Test Get RTC Timer interrupt for hera"
    },
    {
        7869,
        "Pmic_getRtcStatus : Negative test for RTC Get Status for hera"
    },
    {
        7870,
        "Pmic_rtcGetTimeDateInfo : Negative test for Test RTC for Get Time for HERA"
    },
    {
        7871,
        "Pmic_rtcGetFreqComp : Negative test for Test RTC for get RTC frequency compensation for HERA"
    },
    {
        8015,
        "Pmic_fsmSetMissionState : RTC Wakeup using Timer Interrupt using Standby State"
    },
    {
        8016,
        "Pmic_fsmSetMissionState :  RTC Wakeup using Alarm Interrupt using Standby State"
    },
    {
        9895,
        "Pmic_rtcGetStatus : Parameter validation for handle"
    },
    {
        9896,
        "Pmic_rtcGetStatus : Parameter validation for pRtcstatus"
    },
    {
        9897,
        "Pmic_rtcGetConfiguration : Parameter validation for handle"
    },
    {
        9898,
        "Pmic_rtcGetConfiguration : Parameter validation for pRtcCfg"
    },
    {
        9899,
        "Pmic_rtcClrRstStatus : Clear RTC Reset status using powerupStatus"
    },
    {
        9900,
        "Pmic_rtcClrRstStatus : Clear RTC Reset status using rtcRstStatus"
    },
    {
        9901,
        "Pmic_rtcClrRstStatus : Parameter validation for handle"
    },
    {
        9902,
        "Pmic_rtcClrRstStatus : Parameter validation for rtcRstStatType"
    },
    {
        9903,
        "Pmic_rtcSetConfiguration : Set 32K counter with compensation values"
    },
    {
        9904,
        "Pmic_rtcSetConfiguration : Configure Enable/Disable Crystal Oscillator"
    },
    {
        9905,
        "Pmic_rtcSetConfiguration: Configure RTC time to round the time to closest minute"
    },
    {
        9907,
        "Pmic_rtcSetConfiguration : Select RTC Time and Date Register read from Dynamic Registers"
    },
    {
        9908,
        "Pmic_rtcSetConfiguration : Select RTC Time and Date Register read from Static Shadowed Registers"
    },
    {
        9909,
        "Pmic_rtcSetConfiguration : Select Crystal Oscillator type as 9PF/ 6PF/ 12.5PF"
    },
    {
        9910,
        "Pmic_rtcSetConfiguration : Parameter validation for handle"
    },
    {
        9911,
        "Pmic_rtcSetConfiguration : Parameter validation for RTC Config Param - setRtcTimeRound30s"
    },
    {
        9912,
        "Pmic_rtcSetConfiguration : Parameter validation for RTC Config Param - timeDateRegSel"
    },
    {
        9913,
        "Pmic_rtcSetConfiguration : Parameter validation for RTC Config Param - crystalOScType"
    },
    {
        9914,
        "Pmic_rtcSetConfiguration : Parameter validation for RTC Config Param - set32KCounterCompVal"
    },
    {
        10605,
        "Pmic_rtcSetAlarmInfo : Parameter Validation for RTC day min value in Feb Month"
    },
    {
        10606,
        "Pmic_rtcSetAlarmInfo : Parameter Validation for RTC day min value"
    },
    {
        10607,
        "Pmic_rtcSetAlarmInfo : Parameter Validation for RTC day max value"
    },
    {
        10608,
        "Pmic_rtcSetAlarmInfo : Parameter Validation for RTC month min value"
    },
    {
        10609,
        "Pmic_rtcSetAlarmInfo : Parameter Validation for RTC month max value"
    },
    {
        10610,
        "Pmic_rtcSetAlarmInfo : Parameter Validation for RTC year min value"
    },
    {
        10611,
        "Pmic_rtcSetAlarmInfo : Parameter Validation for RTC weekday min value"
    },
    {
        10612,
        "Pmic_rtcSetAlarmInfo : Parameter Validation for RTC Meridian Mode"
    },
    {
        10613,
        "Pmic_rtcSetAlarmInfo : Parameter Validation for RTC weekday"
    },
    {
        10614,
        "Pmic_rtcSetAlarmInfo : Parameter Validation for dateCfg and timeCfg"
    },
    {
        10615,
        "Pmic_rtcSetTimeDateInfo : Parameter Validation for dateCfg and timeCfg"
    },
    {
        10616,
        "Pmic_rtcGetAlarmInfo : Parameter Validation for pDateCfg and pTimeCfg"
    },
    {
        10617,
        "Pmic_rtcGetTimeDateInfo : Parameter Validation for pDateCfg and pTimeCfg"
    },
    {
        10618,
        "Pmic_rtcGetRstStatus : Test to get RTC Power Up status"
    },
    {
        10619,
        "Pmic_rtcSetTimeDateInfo : Parameter validation to RTC day min value in leap year"
    },
    {
        10620,
        "Pmic_rtcGetTimeDateInfo : Test to set RTC timeDateRegSel as static shadowed registers"
    },
    {
        10621,
        "Pmic_rtcSetConfiguration : Test to set rtc configuration when RTC is started"
    },
    {
        8814,
        "Pmic_rtcTests: Added for Coverage"
    },
};

/*!
 * \brief   Set RTC Alarm interrupt
 */
static void test_pmic_rtc_testSetAlarm(void)
{
    int32_t status             = PMIC_ST_SUCCESS;
    Pmic_RtcTime_t timeCfg     = {PMIC_RTC_VALID_PARAM_TIME_CFG_VAL, 30U, 30U,
                                  6U, 0U, 1U};
    Pmic_RtcDate_t dateCfg     = {PMIC_RTC_VALID_PARAM_DATE_CFG_VAL, 15U, 6U,
                                  2055U, 1U};

    Pmic_RtcTime_t timeCfg_rd;
    Pmic_RtcDate_t dateCfg_rd;

    test_pmic_print_unity_testcase_info(5990,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    timeCfg.timeMode        = PMIC_RTC_12_HOUR_MODE;

    status = Pmic_rtcSetAlarmInfo(pPmicCoreHandle, timeCfg, dateCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    timeCfg_rd.validParams = PMIC_RTC_VALID_PARAM_TIME_CFG_VAL;
    dateCfg_rd.validParams = PMIC_RTC_VALID_PARAM_DATE_CFG_VAL;
    status = Pmic_rtcGetAlarmInfo(pPmicCoreHandle, &timeCfg_rd, &dateCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    TEST_ASSERT_EQUAL(timeCfg.seconds, timeCfg_rd.seconds);
    TEST_ASSERT_EQUAL(timeCfg.minutes, timeCfg_rd.minutes);
    TEST_ASSERT_EQUAL(timeCfg.hour, timeCfg_rd.hour);
    TEST_ASSERT_EQUAL(timeCfg.timeMode, timeCfg_rd.timeMode);
    TEST_ASSERT_EQUAL(timeCfg.meridianMode, timeCfg_rd.meridianMode);

    TEST_ASSERT_EQUAL(dateCfg.day, dateCfg_rd.day);
    TEST_ASSERT_EQUAL(dateCfg.month, dateCfg_rd.month);
    TEST_ASSERT_EQUAL(dateCfg.year, dateCfg_rd.year);

    timeCfg.validParams = PMIC_RTC_VALID_PARAM_TIME_CFG_VAL;
    dateCfg.validParams = PMIC_RTC_VALID_PARAM_DATE_CFG_VAL;
    dateCfg.day = 1U;
    dateCfg.month = 2U;
    dateCfg.year = 2040U;

    status = Pmic_rtcSetAlarmInfo(pPmicCoreHandle, timeCfg, dateCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    timeCfg_rd.validParams = PMIC_RTC_VALID_PARAM_TIME_CFG_VAL;
    dateCfg_rd.validParams = PMIC_RTC_VALID_PARAM_DATE_CFG_VAL;
    status = Pmic_rtcGetAlarmInfo(pPmicCoreHandle, &timeCfg_rd, &dateCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(dateCfg.day, dateCfg_rd.day);
    TEST_ASSERT_EQUAL(dateCfg.month, dateCfg_rd.month);
    TEST_ASSERT_EQUAL(dateCfg.year, dateCfg_rd.year);

    dateCfg.month = 4U;
    status = Pmic_rtcSetAlarmInfo(pPmicCoreHandle, timeCfg, dateCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_rtcGetAlarmInfo(pPmicCoreHandle, &timeCfg_rd, &dateCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(dateCfg.month, dateCfg_rd.month);

    dateCfg.month = 9U;
    status = Pmic_rtcSetAlarmInfo(pPmicCoreHandle, timeCfg, dateCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_rtcGetAlarmInfo(pPmicCoreHandle, &timeCfg_rd, &dateCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(dateCfg.month, dateCfg_rd.month);

    dateCfg.month = 11U;
    status = Pmic_rtcSetAlarmInfo(pPmicCoreHandle, timeCfg, dateCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_rtcGetAlarmInfo(pPmicCoreHandle, &timeCfg_rd, &dateCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(dateCfg.month, dateCfg_rd.month);

    dateCfg.day = 15U;
    dateCfg.month = 1U;
    dateCfg.year = 2055U;
    status = Pmic_rtcSetAlarmInfo(pPmicCoreHandle, timeCfg, dateCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_rtcGetAlarmInfo(pPmicCoreHandle, &timeCfg_rd, &dateCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(dateCfg.day, dateCfg_rd.day);
    TEST_ASSERT_EQUAL(dateCfg.month, dateCfg_rd.month);
    TEST_ASSERT_EQUAL(dateCfg.year, dateCfg_rd.year);

    /* Checking for Valid Params PMIC_RTC_DATE_CFG_DAY_VALID and Valid
       Params not equal to PMIC_RTC_DATE_CFG_MONTH_VALID */
    dateCfg.day = 1U;
    dateCfg.validParams = PMIC_RTC_DATE_CFG_DAY_VALID_SHIFT;
    status = Pmic_rtcSetAlarmInfo(pPmicCoreHandle, timeCfg, dateCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    dateCfg_rd.validParams = PMIC_RTC_DATE_CFG_DAY_VALID_SHIFT;
    status = Pmic_rtcGetAlarmInfo(pPmicCoreHandle, &timeCfg_rd, &dateCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(dateCfg.day, dateCfg_rd.day);

    pmic_testResultUpdate_pass(5990,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Parameter validation for handle
 */
static void test_pmic_rtc_setAlarmInfoPrmValTest_handle(void)
{
    int32_t status         = PMIC_ST_SUCCESS;
    Pmic_RtcTime_t timeCfg = {PMIC_RTC_VALID_PARAM_TIME_CFG_VAL, 30U, 30U, 6U,
                              0U, 1U};
    Pmic_RtcDate_t dateCfg = {PMIC_RTC_VALID_PARAM_DATE_CFG_VAL, 15U, 6U,
                              2055U,1U};

    test_pmic_print_unity_testcase_info(5991,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    status = Pmic_rtcSetAlarmInfo(NULL, timeCfg, dateCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, status);

    pmic_testResultUpdate_pass(5991,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Parameter validation for seconds
 */
static void test_pmic_rtc_setAlarmInfoPrmValTest_seconds(void)
{
    int32_t      status       = PMIC_ST_SUCCESS;
    Pmic_RtcTime_t timeCfg    = {PMIC_RTC_TIME_CFG_SEC_VALID_SHIFT, 30U, 30U,
                                 6U, 0U, 1U};
    Pmic_RtcDate_t dateCfg    = {0x00U, 15U, 6U, 2055U, 1U};

    test_pmic_print_unity_testcase_info(6090,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    timeCfg.seconds = PMIC_RTC_INVALID_SEC_MINUTE;

    status = Pmic_rtcSetAlarmInfo(pPmicCoreHandle, timeCfg, dateCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_TIME, status);

    pmic_testResultUpdate_pass(6090,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Parameter validation for minutes
 */
static void test_pmic_rtc_setAlarmInfoPrmValTest_minutes(void)
{
    int32_t      status       = PMIC_ST_SUCCESS;
    Pmic_RtcTime_t timeCfg    = {PMIC_RTC_TIME_CFG_MIN_VALID_SHIFT, 30U, 30U,
                                 6U, 0U, 1U};
    Pmic_RtcDate_t dateCfg    = {0x00U, 15U, 6U, 2055U, 1U};

    test_pmic_print_unity_testcase_info(6091,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    timeCfg.minutes           = PMIC_RTC_INVALID_SEC_MINUTE;

    status = Pmic_rtcSetAlarmInfo(pPmicCoreHandle, timeCfg, dateCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_TIME, status);

    pmic_testResultUpdate_pass(6091,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Parameter validation for timeMode
 */
static void test_pmic_rtc_setAlarmInfoPrmValTest_timeMode(void)
{
    int32_t      status       = PMIC_ST_SUCCESS;
    Pmic_RtcTime_t timeCfg    = {PMIC_RTC_TIME_CFG_TIMEMODE_VALID_SHIFT, 30U,
                                 30U, 6U, 0U, 1U};
    Pmic_RtcDate_t dateCfg    = {0x00U, 15U, 6U, 2055U, 1U};

    test_pmic_print_unity_testcase_info(6092,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    timeCfg.timeMode          = PMIC_RTC_INVALID_TIME_MODE;

    status = Pmic_rtcSetAlarmInfo(pPmicCoreHandle, timeCfg, dateCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_TIME, status);

    pmic_testResultUpdate_pass(6092,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Parameter validation for meridianMode
 */
static void test_pmic_rtc_setAlarmInfoPrmValTest_meridianMode(void)
{
    int32_t status         = PMIC_ST_SUCCESS;
    Pmic_RtcTime_t timeCfg = {PMIC_RTC_TIME_CFG_MERIDIAN_VALID_SHIFT,  30U,
                              30U, 6U, 0U, 1U};
    Pmic_RtcDate_t dateCfg = {0x00U,  15U, 6U, 2055U, 1U};

    test_pmic_print_unity_testcase_info(6093,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    timeCfg.timeMode       = PMIC_RTC_12_HOUR_MODE;
    timeCfg.meridianMode   = PMIC_RTC_INVALID_MERIDIEN_MODE;

    status = Pmic_rtcSetAlarmInfo(pPmicCoreHandle, timeCfg, dateCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_TIME, status);

    pmic_testResultUpdate_pass(6093,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Parameter validation for hour when timeMode = 1
 */
static void test_pmic_rtc_setAlarmInfoPrmValTest_hour12(void)
{
    int32_t      status      = PMIC_ST_SUCCESS;
    Pmic_RtcTime_t timeCfg    =
                              {PMIC_RTC_VALID_PARAM_TIME_HRS_TIMEMODE_CFG_VAL,
                               30U, 30U, 6U, 0U, 1U};
    Pmic_RtcDate_t dateCfg    = {0x00U, 15U, 6U, 2055U, 1U};

    test_pmic_print_unity_testcase_info(6094,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    timeCfg.timeMode          = PMIC_RTC_12_HOUR_MODE;
    timeCfg.hour              = PMIC_RTC_INVALID_HOUR_13;

    status = Pmic_rtcSetAlarmInfo(pPmicCoreHandle, timeCfg, dateCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_TIME, status);

    pmic_testResultUpdate_pass(6094,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Parameter validation for hour when timeMode = 0
 */
static void test_pmic_rtc_setAlarmInfoPrmValTest_hour24(void)
{
    int32_t      status      = PMIC_ST_SUCCESS;
    Pmic_RtcTime_t timeCfg    =
                            {PMIC_RTC_VALID_PARAM_TIME_HRS_TIMEMODE_CFG_VAL,
                             30U, 30U, 6U, 0U, 1U};
    Pmic_RtcDate_t dateCfg    = {0x00U, 15U, 6U, 2055U, 1U};

    test_pmic_print_unity_testcase_info(6095,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    timeCfg.timeMode          = PMIC_RTC_24_HOUR_MODE;
    timeCfg.hour              = PMIC_RTC_INVALID_HOUR_25;

    status = Pmic_rtcSetAlarmInfo(pPmicCoreHandle, timeCfg, dateCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_TIME, status);

    pmic_testResultUpdate_pass(6095,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Negative test for hour  = 0U, when timeMode = 1
 */
static void test_pmic_rtc_setAlarmInfoPrmValTest_hour(void)
{
    int32_t      status       = PMIC_ST_SUCCESS;
    Pmic_RtcTime_t timeCfg    =
                            {PMIC_RTC_VALID_PARAM_TIME_HRS_TIMEMODE_CFG_VAL,
                             30U, 30U,6U, 0U, 1U};
    Pmic_RtcDate_t dateCfg    = {0x00, 15U, 6U, 2055U, 1U};

    test_pmic_print_unity_testcase_info(6096,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    timeCfg.timeMode          = PMIC_RTC_12_HOUR_MODE;
    timeCfg.hour              = PMIC_RTC_HOUR_0;

    status = Pmic_rtcSetAlarmInfo(pPmicCoreHandle, timeCfg, dateCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_TIME, status);

    pmic_testResultUpdate_pass(6096,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Negative test for month = 0
 */
static void test_pmic_rtc_setAlarmInfoPrmValTest_month(void)
{
    int32_t      status      = PMIC_ST_SUCCESS;
    Pmic_RtcTime_t timeCfg    = {0x00U, 30U, 30U, 6U, 0U, 1U};
    Pmic_RtcDate_t dateCfg    = {PMIC_RTC_DATE_CFG_MONTH_VALID_SHIFT, 15U, 6U,
                                 2055U, 1U};

    test_pmic_print_unity_testcase_info(6099,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    dateCfg.month             = PMIC_RTC_INVALID_MONTH_0;

    status = Pmic_rtcSetAlarmInfo(pPmicCoreHandle, timeCfg, dateCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_DATE, status);

    pmic_testResultUpdate_pass(6099,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Negative test for day = 0
 */
static void test_pmic_rtc_setAlarmInfoPrmValTest_day(void)
{
    int32_t      status      = PMIC_ST_SUCCESS;
    Pmic_RtcTime_t timeCfg    = {0x00U, 30U, 30U, 6U, 0U, 1U};
    Pmic_RtcDate_t dateCfg    = {PMIC_RTC_DATE_CFG_DAY_VALID_SHIFT, 15U, 6U,
                                 2055U, 1U};

    dateCfg.day               = PMIC_RTC_INVALID_DAY_0;

    test_pmic_print_unity_testcase_info(6100,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    status = Pmic_rtcSetAlarmInfo(pPmicCoreHandle, timeCfg, dateCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_DATE, status);

    pmic_testResultUpdate_pass(6100,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Parameter range validation for year
 */
static void test_pmic_rtc_setAlarmInfoPrmValTest_year(void)
{
    int32_t      status      = PMIC_ST_SUCCESS;
    Pmic_RtcTime_t timeCfg    = {0U, 30U, 30U, 6U, 0U, 1U};
    Pmic_RtcDate_t dateCfg    = {PMIC_RTC_DATE_CFG_YEAR_VALID_SHIFT, 15U, 6U,
                                 2055U, 1U};

    test_pmic_print_unity_testcase_info(6101,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    dateCfg.year              = PMIC_RTC_INVALID_YEAR;

    status = Pmic_rtcSetAlarmInfo(pPmicCoreHandle, timeCfg, dateCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_DATE, status);

    pmic_testResultUpdate_pass(6101,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Parameter range validation for month
 */
static void test_pmic_rtc_setAlarmInfoPrmValTest_month_range(void)
{
    int32_t      status      = PMIC_ST_SUCCESS;
    Pmic_RtcTime_t timeCfg    = {0U, 30U, 30U, 6U, 0U, 1U};
    Pmic_RtcDate_t dateCfg    = {PMIC_RTC_DATE_CFG_MONTH_VALID_SHIFT, 15U, 6U,
                                 2055U, 1U};

    test_pmic_print_unity_testcase_info(6102,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    dateCfg.month             = PMIC_RTC_INVALID_MONTH;

    status = Pmic_rtcSetAlarmInfo(pPmicCoreHandle, timeCfg, dateCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_DATE, status);

    pmic_testResultUpdate_pass(6102,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Parameter range validation for dayfor months with 30 days
 */
static void test_pmic_rtc_setAlarmInfoPrmValTest_day_month(void)
{
    int32_t      status      = PMIC_ST_SUCCESS;
    Pmic_RtcTime_t timeCfg    = {0U, 30U, 30U, 6U, 0U, 1U};
    Pmic_RtcDate_t dateCfg    = {PMIC_RTC_VALID_PARAM_DATE_CFG_DAY_MNTH_VAL,
                                 15U, 6U,2055U, 1U};

    test_pmic_print_unity_testcase_info(6103,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    dateCfg.month             = PMIC_RTC_MONTH_APR;
    dateCfg.day               = PMIC_RTC_INVALID_DAY_31;

    status = Pmic_rtcSetAlarmInfo(pPmicCoreHandle, timeCfg, dateCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_DATE, status);

    pmic_testResultUpdate_pass(6103,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Parameter range validation for day for leap
 *          year(year %4 = 0 ) and month = 2 (february)
 */
static void test_pmic_rtc_setAlarmInfoPrmValTest_feb_leapyear(void)
{
    int32_t      status      = PMIC_ST_SUCCESS;
    Pmic_RtcTime_t timeCfg    = {0x0U, 30U, 30U, 6U, 0U, 1U};
    Pmic_RtcDate_t dateCfg    = {PMIC_RTC_VALID_PARAM_DATE_CFG_DAY_MNTH_YR_VAL,
                                 15U, 6U,2055U, 1U};

    test_pmic_print_unity_testcase_info(6104,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    dateCfg.month             = PMIC_RTC_MONTH_FEB;
    dateCfg.year              = PMIC_RTC_YEAR_2044;
    dateCfg.day               = PMIC_RTC_INVALID_DAY_30;

    status = Pmic_rtcSetAlarmInfo(pPmicCoreHandle, timeCfg, dateCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_DATE, status);

    pmic_testResultUpdate_pass(6104,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Parameter range validation for day for Non-leap
 *          year(year %4 != 0 ) and month = 2 (february)
 */
static void test_pmic_rtc_setAlarmInfoPrmValTest_feb_nonleapyear(void)
{
    int32_t      status      = PMIC_ST_SUCCESS;
    Pmic_RtcTime_t timeCfg    = {0U, 30U, 30U, 6U, 0U, 1U};
    Pmic_RtcDate_t dateCfg    = {PMIC_RTC_VALID_PARAM_DATE_CFG_DAY_MNTH_YR_VAL,
                                 15U, 6U,2055U, 1U};

    test_pmic_print_unity_testcase_info(6107,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    dateCfg.month             = PMIC_RTC_MONTH_FEB;
    dateCfg.year              = PMIC_RTC_YEAR_2045;
    dateCfg.day               = PMIC_RTC_INVALID_DAY;

    status = Pmic_rtcSetAlarmInfo(pPmicCoreHandle, timeCfg, dateCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_DATE, status);

    pmic_testResultUpdate_pass(6107,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Parameter range validation for day, for months with 31 days
 */
static void test_pmic_rtc_setAlarmInfoPrmValTest_day_month31(void)
{
    int32_t      status      = PMIC_ST_SUCCESS;
    Pmic_RtcTime_t timeCfg    = {0U,  30U, 30U, 6U, 0U, 1U};
    Pmic_RtcDate_t dateCfg    = {PMIC_RTC_VALID_PARAM_DATE_CFG_DAY_MNTH_VAL,
                                 15U, 6U,2055U, 1U};

    test_pmic_print_unity_testcase_info(6108,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    dateCfg.month             = PMIC_RTC_MONTH_JUL;
    dateCfg.day               = PMIC_RTC_INVALID_DAY_32;

    status = Pmic_rtcSetAlarmInfo(pPmicCoreHandle, timeCfg, dateCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_DATE, status);

    pmic_testResultUpdate_pass(6108,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Get RTC Alarm interrupt
 */
static void test_pmic_rtc_testGetAlarm(void)
{
    int32_t status             = PMIC_ST_SUCCESS;
    Pmic_RtcTime_t timeCfg     = {PMIC_RTC_VALID_PARAM_TIME_CFG_VAL, 30U, 30U,
                                  6U, 0U, 1U};
    Pmic_RtcDate_t dateCfg     = {PMIC_RTC_VALID_PARAM_DATE_CFG_VAL, 15U, 6U,
                                  2055U, 1U};

    Pmic_RtcTime_t timeCfg_rd;
    Pmic_RtcDate_t dateCfg_rd;

    test_pmic_print_unity_testcase_info(6109,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    timeCfg.timeMode        = PMIC_RTC_12_HOUR_MODE;

    status = Pmic_rtcSetAlarmInfo(pPmicCoreHandle, timeCfg, dateCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    timeCfg_rd.validParams = 0x1FU;
    dateCfg_rd.validParams = 0x1FU;
    status = Pmic_rtcGetAlarmInfo(pPmicCoreHandle, &timeCfg_rd, &dateCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    TEST_ASSERT_EQUAL(timeCfg.seconds, timeCfg_rd.seconds);
    TEST_ASSERT_EQUAL(timeCfg.minutes, timeCfg_rd.minutes);
    TEST_ASSERT_EQUAL(timeCfg.hour, timeCfg_rd.hour);
    TEST_ASSERT_EQUAL(timeCfg.timeMode, timeCfg_rd.timeMode);
    TEST_ASSERT_EQUAL(timeCfg.meridianMode, timeCfg_rd.meridianMode);

    TEST_ASSERT_EQUAL(dateCfg.day, dateCfg_rd.day);
    TEST_ASSERT_EQUAL(dateCfg.month, dateCfg_rd.month);
    TEST_ASSERT_EQUAL(dateCfg.year, dateCfg_rd.year);

    pmic_testResultUpdate_pass(6109,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Parameter validation for handle
 */
static void test_pmic_rtc_getAlarmInfoPrmValTest_handle(void)
{
    int32_t      status      = PMIC_ST_SUCCESS;
    Pmic_RtcTime_t timeCfg    = {PMIC_RTC_VALID_PARAM_TIME_CFG_VAL, 30U, 30U,
                                 6U, 0U, 1U};
    Pmic_RtcDate_t dateCfg    = {PMIC_RTC_VALID_PARAM_DATE_CFG_VAL,  15U, 6U,
                                 2055U, 1U};

    test_pmic_print_unity_testcase_info(6110,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    status = Pmic_rtcGetAlarmInfo(NULL, &timeCfg, &dateCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, status);

    pmic_testResultUpdate_pass(6110,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Parameter validation for timeCfg
 */
static void test_pmic_rtc_getAlarmInfoPrmValTest_timeCfg(void)
{
    int32_t status         = PMIC_ST_SUCCESS;
    Pmic_RtcDate_t dateCfg = {PMIC_RTC_VALID_PARAM_DATE_CFG_VAL, 15U, 6U,
                              2055U, 1U};

    test_pmic_print_unity_testcase_info(6111,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    status = Pmic_rtcGetAlarmInfo(pPmicCoreHandle, NULL, &dateCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    pmic_testResultUpdate_pass(6111,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Parameter validation for dataCfg
 */
static void test_pmic_rtc_getAlarmInfoPrmValTest_dateCfg(void)
{
    int32_t      status      = PMIC_ST_SUCCESS;
    Pmic_RtcTime_t timeCfg    = {PMIC_RTC_VALID_PARAM_TIME_CFG_VAL, 30U, 30U,
                                 6U, 0U, 1U};

    test_pmic_print_unity_testcase_info(6112,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    status = Pmic_rtcGetAlarmInfo(pPmicCoreHandle, &timeCfg, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    pmic_testResultUpdate_pass(6112,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Test Set RTC Timer interrupt Period
 */
static void test_pmic_rtc_testSetTimer(void)
{
    int32_t status         = PMIC_ST_SUCCESS;
    uint8_t timerPeriod, timerPeriod_rd;

    test_pmic_print_unity_testcase_info(6113,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    timerPeriod = PMIC_RTC_HOUR_INTR_PERIOD;

    status = Pmic_rtcSetTimerPeriod(pPmicCoreHandle, timerPeriod);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_rtcGetTimerPeriod(pPmicCoreHandle, &timerPeriod_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    TEST_ASSERT_EQUAL(timerPeriod, timerPeriod_rd);

    pmic_testResultUpdate_pass(6113,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Parameter validation for handle
 */
static void test_pmic_rtc_setTimerPeriodPrmValTest_handle(void)
{
    int32_t status       = PMIC_ST_SUCCESS;
    uint8_t  timerPeriod = PMIC_RTC_HOUR_INTR_PERIOD;

    test_pmic_print_unity_testcase_info(6114,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    status = Pmic_rtcSetTimerPeriod(NULL, timerPeriod);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, status);

    pmic_testResultUpdate_pass(6114,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Parameter validation for timerPeriod
 */
static void test_pmic_rtc_setTimerPeriodPrmValTest_timerPeriod(void)
{
    int32_t status         = PMIC_ST_SUCCESS;
    uint8_t timerPeriod    = PMIC_RTC_HOUR_INTR_PERIOD;

    test_pmic_print_unity_testcase_info(6115,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    timerPeriod            = 5;

    status = Pmic_rtcSetTimerPeriod(pPmicCoreHandle, timerPeriod);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

    pmic_testResultUpdate_pass(6115,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Test Get RTC Timer interrupt
 */
static void test_pmic_rtc_testGetTimer(void)
{
    int32_t status         = PMIC_ST_SUCCESS;
    uint8_t timerPeriod, timerPeriod_rd;

    test_pmic_print_unity_testcase_info(6116,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    timerPeriod = PMIC_RTC_HOUR_INTR_PERIOD;

    status = Pmic_rtcSetTimerPeriod(pPmicCoreHandle, timerPeriod);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_rtcGetTimerPeriod(pPmicCoreHandle, &timerPeriod_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    TEST_ASSERT_EQUAL(timerPeriod, timerPeriod_rd);

    pmic_testResultUpdate_pass(6116,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Parameter validation for handle
 */
static void test_pmic_rtc_getTimerPeriodPrmValTest_handle(void)
{
    int32_t status       = PMIC_ST_SUCCESS;
    uint8_t timerPeriod  = PMIC_RTC_HOUR_INTR_PERIOD;

    test_pmic_print_unity_testcase_info(6117,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    status = Pmic_rtcGetTimerPeriod(NULL, &timerPeriod);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, status);

    pmic_testResultUpdate_pass(6117,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Parameter validation for timerPeriod
 */
static void test_pmic_rtc_getTimerPeriodPrmValTest_timerPeriod(void)
{
    int32_t status    = PMIC_ST_SUCCESS;

    test_pmic_print_unity_testcase_info(6118,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    status = Pmic_rtcGetTimerPeriod(pPmicCoreHandle, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    pmic_testResultUpdate_pass(6118,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Test RTC Disable
 */
static void test_pmic_rtc_testDisable(void)
{
    int32_t status         = PMIC_ST_SUCCESS;

    test_pmic_print_unity_testcase_info(6119,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    status = Pmic_rtcEnable(pPmicCoreHandle, PMIC_RTC_STOP);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    pmic_testResultUpdate_pass(6119,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Parameter validation for handle
 */
static void test_pmic_rtc_disablePrmValTest_handle(void)
{
    int32_t status         = PMIC_ST_SUCCESS;

    test_pmic_print_unity_testcase_info(6120,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    status = Pmic_rtcEnable(NULL, PMIC_RTC_STOP);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, status);

    pmic_testResultUpdate_pass(6120,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Test RTC Enable
 */
static void test_pmic_rtc_testEnable(void)
{
    int32_t status         = PMIC_ST_SUCCESS;

    test_pmic_print_unity_testcase_info(6121,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    status = Pmic_rtcEnable(pPmicCoreHandle, PMIC_RTC_START);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    pmic_testResultUpdate_pass(6121,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Parameter validation for handle
 */
static void test_pmic_rtc_enablePrmValTest_handle(void)
{
    int32_t status         = PMIC_ST_SUCCESS;

    test_pmic_print_unity_testcase_info(6122,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    status = Pmic_rtcEnable(NULL, PMIC_RTC_START);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, status);

    pmic_testResultUpdate_pass(6122,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Test RTC for Set Time
 */
static void test_pmic_rtc_testSetTime(void)
{
    int32_t status         = PMIC_ST_SUCCESS;

    /* To test this case user must ensure below condition:
     * timeCfg.seconds sholud be in range (0 to 50)
     *
     * So that, other than seconds of time and date parameters
     * are same as setRtc while reading back RTC parameters
     * using GetRTC
     */
    Pmic_RtcTime_t timeCfg = {PMIC_RTC_VALID_PARAM_TIME_CFG_VAL, 30U, 30U, 6U,
                              0U, 1U};
    Pmic_RtcDate_t dateCfg = {PMIC_RTC_VALID_PARAM_DATE_CFG_VAL, 15U, 6U,
                              2055U, 1U};
    Pmic_RtcTime_t timeCfg_rd;
    Pmic_RtcDate_t dateCfg_rd;

    test_pmic_print_unity_testcase_info(6158,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    timeCfg.timeMode       = PMIC_RTC_12_HOUR_MODE;

    status = Pmic_rtcSetTimeDateInfo(pPmicCoreHandle, timeCfg, dateCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    timeCfg_rd.validParams = PMIC_RTC_VALID_PARAM_TIME_CFG_VAL;
    dateCfg_rd.validParams = PMIC_RTC_VALID_PARAM_DATE_CFG_VAL;

    status = Pmic_rtcGetTimeDateInfo(pPmicCoreHandle, &timeCfg_rd, &dateCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    TEST_ASSERT_EQUAL(timeCfg.minutes, timeCfg_rd.minutes);
    TEST_ASSERT_EQUAL(timeCfg.hour, timeCfg_rd.hour);
    TEST_ASSERT_EQUAL(timeCfg.timeMode, timeCfg_rd.timeMode);
    TEST_ASSERT_EQUAL(timeCfg.meridianMode, timeCfg_rd.meridianMode);

    TEST_ASSERT_EQUAL(dateCfg.day, dateCfg_rd.day);
    TEST_ASSERT_EQUAL(dateCfg.month, dateCfg_rd.month);
    TEST_ASSERT_EQUAL(dateCfg.year, dateCfg_rd.year);
    TEST_ASSERT_EQUAL(dateCfg.weekday, dateCfg_rd.weekday);

    dateCfg.month = 2U;
    dateCfg.day   = 2U;
    status = Pmic_rtcSetTimeDateInfo(pPmicCoreHandle, timeCfg, dateCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_rtcGetTimeDateInfo(pPmicCoreHandle, &timeCfg_rd, &dateCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(dateCfg.day, dateCfg_rd.day);
    TEST_ASSERT_EQUAL(dateCfg.month, dateCfg_rd.month);

    /* Checking for Valid Params not equal to PMIC_RTC_DATE_CFG_MONTH_VALID */
    dateCfg.validParams = PMIC_RTC_DATE_CFG_MONTH_VALID_SHIFT;
    status = Pmic_rtcSetTimeDateInfo(pPmicCoreHandle, timeCfg, dateCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_rtcGetTimeDateInfo(pPmicCoreHandle, &timeCfg_rd, &dateCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(dateCfg.month, dateCfg_rd.month);

    /* Checking for Valid Params not equal to PMIC_RTC_DATE_CFG_YEAR_VALID */
    dateCfg.validParams = PMIC_RTC_DATE_CFG_YEAR_VALID_SHIFT;
    status = Pmic_rtcSetTimeDateInfo(pPmicCoreHandle, timeCfg, dateCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_rtcGetTimeDateInfo(pPmicCoreHandle, &timeCfg_rd, &dateCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(dateCfg.year, dateCfg_rd.year);

    pmic_testResultUpdate_pass(6158,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Parameter validation for handle
 */
static void test_pmic_rtc_setTimePrmValTest_handle(void)
{
    int32_t      status      = PMIC_ST_SUCCESS;
    Pmic_RtcTime_t timeCfg    = {PMIC_RTC_VALID_PARAM_TIME_CFG_VAL,  30U, 30U,
                                 6U, 0U, 1U};
    Pmic_RtcDate_t dateCfg    = {PMIC_RTC_VALID_PARAM_DATE_CFG_VAL, 15U, 6U,
                                 2055U, 1U};

    test_pmic_print_unity_testcase_info(6159,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    status = Pmic_rtcSetTimeDateInfo(NULL, timeCfg, dateCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, status);

    pmic_testResultUpdate_pass(6159,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Parameter validation for seconds
 */
static void test_pmic_rtc_setTimePrmValTest_seconds(void)
{
    int32_t      status      = PMIC_ST_SUCCESS;
    Pmic_RtcTime_t timeCfg    = {PMIC_RTC_TIME_CFG_SEC_VALID_SHIFT, 30U, 30U,
                                 6U, 0U, 1U};
    Pmic_RtcDate_t dateCfg    = {0U, 15U, 6U, 2055U, 1U};

    test_pmic_print_unity_testcase_info(6162,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    timeCfg.seconds           = PMIC_RTC_INVALID_SEC_MINUTE;

    status = Pmic_rtcSetTimeDateInfo(pPmicCoreHandle, timeCfg, dateCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_TIME, status);

    pmic_testResultUpdate_pass(6162,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Parameter validation for minutes
 */
static void test_pmic_rtc_setTimePrmValTest_minutes(void)
{
    int32_t      status      = PMIC_ST_SUCCESS;
    Pmic_RtcTime_t timeCfg    = {PMIC_RTC_TIME_CFG_MIN_VALID_SHIFT, 30U, 30U,
                                 6U, 0U, 1U};
    Pmic_RtcDate_t dateCfg    = {0U, 15U, 6U, 2055U, 1U};

    test_pmic_print_unity_testcase_info(6163,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    timeCfg.minutes           = PMIC_RTC_INVALID_SEC_MINUTE;

    status = Pmic_rtcSetTimeDateInfo(pPmicCoreHandle, timeCfg, dateCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_TIME, status);

    pmic_testResultUpdate_pass(6163,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Parameter validation for timeMode
 */
static void test_pmic_rtc_setTimePrmValTest_timeMode(void)
{
    int32_t      status      = PMIC_ST_SUCCESS;
    Pmic_RtcTime_t timeCfg    = {PMIC_RTC_TIME_CFG_TIMEMODE_VALID_SHIFT, 30U,
                                 30U, 6U, 0U, 1U};
    Pmic_RtcDate_t dateCfg    = {0x0U, 15U, 6U, 2055U, 1U};

    test_pmic_print_unity_testcase_info(6164,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    timeCfg.timeMode           = PMIC_RTC_INVALID_TIME_MODE;

    status = Pmic_rtcSetTimeDateInfo(pPmicCoreHandle, timeCfg, dateCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_TIME, status);

    pmic_testResultUpdate_pass(6164,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Parameter validation for meridianMode
 */
static void test_pmic_rtc_setTimePrmValTest_meridianMode(void)
{
    int32_t status         = PMIC_ST_SUCCESS;
    Pmic_RtcTime_t timeCfg = {PMIC_RTC_TIME_CFG_MERIDIAN_VALID_SHIFT, 30U, 30U,
                              6U, 0U, 1U};
    Pmic_RtcDate_t dateCfg = {0U,  15U, 6U, 2055U, 1U};

    test_pmic_print_unity_testcase_info(6165,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    timeCfg.timeMode       = PMIC_RTC_12_HOUR_MODE;
    timeCfg.meridianMode   = PMIC_RTC_INVALID_MERIDIEN_MODE;
    status = Pmic_rtcSetTimeDateInfo(pPmicCoreHandle, timeCfg, dateCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_TIME, status);

    pmic_testResultUpdate_pass(6165,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Parameter validation for hour when timeMode = 1
 */
static void test_pmic_rtc_setTimePrmValTest_hour12(void)
{
    int32_t      status      = PMIC_ST_SUCCESS;
    Pmic_RtcTime_t timeCfg    =
                              {PMIC_RTC_VALID_PARAM_TIME_HRS_TIMEMODE_CFG_VAL,
                               30U, 30U, 6U, 0U, 1U};
    Pmic_RtcDate_t dateCfg    = {0x0U, 15U, 6U, 2055U, 1U};

    test_pmic_print_unity_testcase_info(6166,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    timeCfg.timeMode          = PMIC_RTC_12_HOUR_MODE;
    timeCfg.hour              = PMIC_RTC_INVALID_HOUR_13;

    status = Pmic_rtcSetTimeDateInfo(pPmicCoreHandle, timeCfg, dateCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_TIME, status);

    pmic_testResultUpdate_pass(6166,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Parameter validation for hour when timeMode = 0
 */
static void test_pmic_rtc_setTimePrmValTest_hour24(void)
{
    int32_t      status      = PMIC_ST_SUCCESS;
    Pmic_RtcTime_t timeCfg    =
                              {PMIC_RTC_VALID_PARAM_TIME_HRS_TIMEMODE_CFG_VAL,
                               30U, 30U, 6U, 0U, 1U};
    Pmic_RtcDate_t dateCfg    = {0x0U, 15U, 6U, 2055U, 1U};

    test_pmic_print_unity_testcase_info(6167,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    timeCfg.timeMode          = PMIC_RTC_12_HOUR_MODE;
    timeCfg.hour              = PMIC_RTC_INVALID_HOUR_25;

    status = Pmic_rtcSetTimeDateInfo(pPmicCoreHandle, timeCfg, dateCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_TIME, status);

    pmic_testResultUpdate_pass(6167,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);

}

/*!
 * \brief   Negative test for hour  = 0U, when timeMode = 1
 */
static void test_pmic_rtc_setTimePrmValTest_hour(void)
{
    int32_t      status      = PMIC_ST_SUCCESS;
    Pmic_RtcTime_t timeCfg    =
                              {PMIC_RTC_VALID_PARAM_TIME_HRS_TIMEMODE_CFG_VAL,
                               30U, 30U, 6U, 0U, 1U};
    Pmic_RtcDate_t dateCfg    = {0U, 15U, 6U, 2055U, 1U};

    test_pmic_print_unity_testcase_info(6168,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    timeCfg.timeMode          = PMIC_RTC_12_HOUR_MODE;
    timeCfg.hour              = PMIC_RTC_INVALID_HOUR_0;

    status = Pmic_rtcSetTimeDateInfo(pPmicCoreHandle, timeCfg, dateCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_TIME, status);

    pmic_testResultUpdate_pass(6168,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Negative test for month  = 0
 */
static void test_pmic_rtc_setTimePrmValTest_month(void)
{
    int32_t      status      = PMIC_ST_SUCCESS;
    Pmic_RtcTime_t timeCfg    = {0U, 30U, 30U, 6U, 0U, 1U};
    Pmic_RtcDate_t dateCfg    = {PMIC_RTC_DATE_CFG_MONTH_VALID_SHIFT, 15U, 6U,
                                 2055U, 1U};

    test_pmic_print_unity_testcase_info(6169,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    dateCfg.month             = PMIC_RTC_INVALID_MONTH_0;

    status = Pmic_rtcSetTimeDateInfo(pPmicCoreHandle, timeCfg, dateCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_DATE, status);

    pmic_testResultUpdate_pass(6169,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Negative test for day = 0
 */
static void test_pmic_rtc_setTimePrmValTest_day(void)
{
    int32_t      status      = PMIC_ST_SUCCESS;
    Pmic_RtcTime_t timeCfg    = {0x0U, 30U, 30U, 6U, 0U, 1U};
    Pmic_RtcDate_t dateCfg    = {PMIC_RTC_DATE_CFG_DAY_VALID_SHIFT, 15U, 6U,
                                 2055U, 1U};

    test_pmic_print_unity_testcase_info(6290,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    dateCfg.day               = PMIC_RTC_INVALID_DAY_0;

    status = Pmic_rtcSetTimeDateInfo(pPmicCoreHandle, timeCfg, dateCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_DATE, status);

    pmic_testResultUpdate_pass(6290,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Parameter range validation for year
 */
static void test_pmic_rtc_setTimePrmValTest_year(void)
{
    int32_t      status      = PMIC_ST_SUCCESS;
    Pmic_RtcTime_t timeCfg    = {0x0U, 30U, 30U, 6U, 0U, 1U};
    Pmic_RtcDate_t dateCfg    = {PMIC_RTC_DATE_CFG_YEAR_VALID_SHIFT, 15U, 6U,
                                 2055U, 1U};

    test_pmic_print_unity_testcase_info(7021,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    dateCfg.year              = PMIC_RTC_INVALID_YEAR;

    status = Pmic_rtcSetTimeDateInfo(pPmicCoreHandle, timeCfg, dateCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_DATE, status);

    pmic_testResultUpdate_pass(7021,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Parameter range validation for month
 */
static void test_pmic_rtc_setTimePrmValTest_pvmonth(void)
{
    int32_t      status      = PMIC_ST_SUCCESS;
    Pmic_RtcTime_t timeCfg    = {0x0U, 30U, 30U, 6U, 0U, 1U};
    Pmic_RtcDate_t dateCfg    = {PMIC_RTC_DATE_CFG_MONTH_VALID_SHIFT, 15U, 6U,
                                 2055U, 1U};

    test_pmic_print_unity_testcase_info(6170,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    dateCfg.month              = PMIC_RTC_INVALID_MONTH;

    status = Pmic_rtcSetTimeDateInfo(pPmicCoreHandle, timeCfg, dateCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_DATE, status);

    pmic_testResultUpdate_pass(6170,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Parameter range validation for day
 */
static void test_pmic_rtc_setTimePrmValTest_month_range(void)
{
    int32_t      status      = PMIC_ST_SUCCESS;
    Pmic_RtcTime_t timeCfg    = {0U, 30U, 30U, 6U, 0U, 1U};
    Pmic_RtcDate_t dateCfg    = {PMIC_RTC_DATE_CFG_DAY_VALID_SHIFT, 15U, 6U,
                                 2055U, 1U};

    test_pmic_print_unity_testcase_info(6171,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    dateCfg.day             = PMIC_RTC_INVALID_DAY_32;

    status = Pmic_rtcSetTimeDateInfo(pPmicCoreHandle, timeCfg, dateCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_DATE, status);

    pmic_testResultUpdate_pass(6171,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Parameter range validation for dayfor months with 30 days
 */
static void test_pmic_rtc_setTimePrmValTest_day_month(void)
{
    int32_t status         = PMIC_ST_SUCCESS;
    Pmic_RtcTime_t timeCfg = {0x0, 30U, 30U, 6U, 0U, 1U};
    Pmic_RtcDate_t dateCfg = {PMIC_RTC_VALID_PARAM_DATE_CFG_DAY_MNTH_VAL, 15U,
                              6U, 2055U, 1U};

    test_pmic_print_unity_testcase_info(6172,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    dateCfg.month          = PMIC_RTC_MONTH_APR;
    dateCfg.day            = PMIC_RTC_INVALID_DAY_31;

    status = Pmic_rtcSetTimeDateInfo(pPmicCoreHandle, timeCfg, dateCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_DATE, status);

    pmic_testResultUpdate_pass(6172,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Parameter range validation for day for leap
 *          year(year %4 = 0 ) and month = 2 (february)
 */
static void test_pmic_rtc_setTimePrmValTest_feb_leapyear(void)
{
    int32_t status         = PMIC_ST_SUCCESS;
    Pmic_RtcTime_t timeCfg = {0x0U, 30U, 30U, 6U, 0U, 1U};
    Pmic_RtcDate_t dateCfg = {PMIC_RTC_VALID_PARAM_DATE_CFG_DAY_MNTH_YR_VAL,
                              15U, 6U, 2055U, 1U};

    test_pmic_print_unity_testcase_info(6173,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    dateCfg.month          = PMIC_RTC_MONTH_FEB;
    dateCfg.year           = PMIC_RTC_YEAR_2044;
    dateCfg.day            = PMIC_RTC_INVALID_DAY_30;

    status = Pmic_rtcSetTimeDateInfo(pPmicCoreHandle, timeCfg, dateCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_DATE, status);

    pmic_testResultUpdate_pass(6173,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Parameter range validation for day for Non-leap
 *          year(year %4 != 0 ) and month = 2 (february)
 */
static void test_pmic_rtc_setTimePrmValTest_feb_nonleapyear(void)
{
    int32_t      status      = PMIC_ST_SUCCESS;
    Pmic_RtcTime_t timeCfg    = {0U, 30U, 30U, 6U, 0U, 1U};
    Pmic_RtcDate_t dateCfg    = {PMIC_RTC_VALID_PARAM_DATE_CFG_DAY_MNTH_YR_VAL,
                                 15U, 6U, 2055U, 1U};

    test_pmic_print_unity_testcase_info(6174,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    dateCfg.month             = PMIC_RTC_MONTH_FEB;
    dateCfg.year              = 2045U;
    dateCfg.day               = PMIC_RTC_INVALID_DAY;

    status = Pmic_rtcSetTimeDateInfo(pPmicCoreHandle, timeCfg, dateCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_DATE, status);

    pmic_testResultUpdate_pass(6174,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Parameter range validation for day, for months with 31 days
 */
static void test_pmic_rtc_setTimePrmValTest_day_month31(void)
{
    int32_t      status      = PMIC_ST_SUCCESS;
    Pmic_RtcTime_t timeCfg    = {0U, 30U, 30U, 6U, 0U, 1U};
    Pmic_RtcDate_t dateCfg    = {PMIC_RTC_VALID_PARAM_DATE_CFG_DAY_MNTH_VAL,
                                 15U, 6U, 2055U, 1U};

    test_pmic_print_unity_testcase_info(6175,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    dateCfg.month             = PMIC_RTC_MONTH_JUL;
    dateCfg.day               = PMIC_RTC_INVALID_DAY_32;

    status = Pmic_rtcSetTimeDateInfo(pPmicCoreHandle, timeCfg, dateCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_DATE, status);

    pmic_testResultUpdate_pass(6175,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);
}
/*!
 * \brief   Test RTC for Get Time
 */
static void test_pmic_rtc_testGetTime(void)
{
    int32_t status         = PMIC_ST_SUCCESS;

    /* To test this case user must ensure below condition:
     * timeCfg.seconds sholud be in range (0 to 50)
     *
     * So that, other than seconds of time and date parameters
     * are same as setRtc while reading back RTC parameters
     * using GetRTC
     */
    Pmic_RtcTime_t timeCfg = {PMIC_RTC_VALID_PARAM_TIME_CFG_VAL, 30U, 30U, 6U,
                              0U, 1U};
    Pmic_RtcDate_t dateCfg = {PMIC_RTC_VALID_PARAM_DATE_CFG_VAL, 15U, 6U,
                              2055U, 1U};
    Pmic_RtcTime_t timeCfg_rd;
    Pmic_RtcDate_t dateCfg_rd;

    test_pmic_print_unity_testcase_info(6176,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    timeCfg.timeMode       = PMIC_RTC_12_HOUR_MODE;

    status = Pmic_rtcSetTimeDateInfo(pPmicCoreHandle, timeCfg, dateCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    timeCfg_rd.validParams = PMIC_RTC_VALID_PARAM_TIME_CFG_VAL;
    dateCfg_rd.validParams = PMIC_RTC_VALID_PARAM_DATE_CFG_VAL;

    status = Pmic_rtcGetTimeDateInfo(pPmicCoreHandle, &timeCfg_rd, &dateCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    TEST_ASSERT_EQUAL(timeCfg.minutes, timeCfg_rd.minutes);
    TEST_ASSERT_EQUAL(timeCfg.hour, timeCfg_rd.hour);
    TEST_ASSERT_EQUAL(timeCfg.timeMode, timeCfg_rd.timeMode);
    TEST_ASSERT_EQUAL(timeCfg.meridianMode, timeCfg_rd.meridianMode);

    TEST_ASSERT_EQUAL(dateCfg.day, dateCfg_rd.day);
    TEST_ASSERT_EQUAL(dateCfg.month, dateCfg_rd.month);
    TEST_ASSERT_EQUAL(dateCfg.year, dateCfg_rd.year);
    TEST_ASSERT_EQUAL(dateCfg.weekday, dateCfg_rd.weekday);

    pmic_testResultUpdate_pass(6176,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Parameter validation for handle
 */
static void test_pmic_rtc_getTimePrmValTest_handle(void)
{
    int32_t      status      = PMIC_ST_SUCCESS;
    Pmic_RtcTime_t timeCfg    = {PMIC_RTC_VALID_PARAM_TIME_CFG_VAL, 30U, 30U,
                                 6U, 0U, 1U};
    Pmic_RtcDate_t dateCfg    = {PMIC_RTC_VALID_PARAM_DATE_CFG_VAL, 15U, 6U,
                                 2055U, 1U};

    test_pmic_print_unity_testcase_info(6177,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    status = Pmic_rtcGetTimeDateInfo(NULL, &timeCfg, &dateCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, status);

    pmic_testResultUpdate_pass(6177,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Parameter validation for timeCfg
 */
static void test_pmic_rtc_getTimePrmValTest_timeCfg(void)
{
    int32_t status         = PMIC_ST_SUCCESS;
    Pmic_RtcDate_t dateCfg = {PMIC_RTC_VALID_PARAM_DATE_CFG_VAL, 15U, 6U,
                              2055U, 1U};

    test_pmic_print_unity_testcase_info(6178,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    status = Pmic_rtcGetTimeDateInfo(pPmicCoreHandle, NULL, &dateCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    pmic_testResultUpdate_pass(6178,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Parameter validation for dataCfg
 */
static void test_pmic_rtc_getTimePrmValTest_dateCfg(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_RtcTime_t timeCfg = {PMIC_RTC_VALID_PARAM_TIME_CFG_VAL, 30U, 30U, 6U,
                              0U, 1U};

    test_pmic_print_unity_testcase_info(6179,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    status = Pmic_rtcGetTimeDateInfo(pPmicCoreHandle, &timeCfg, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    pmic_testResultUpdate_pass(6179,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Test RTC for set RTC frequency compensation
 */
static void test_pmic_rtc_testSetFreqComp(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint16_t compensation = 34952;
    uint16_t compensation_rd = 0U;

    test_pmic_print_unity_testcase_info(6180,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    status = Pmic_rtcSetFreqComp(pPmicCoreHandle, compensation);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status  = Pmic_rtcGetFreqComp(pPmicCoreHandle, &compensation_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    TEST_ASSERT_EQUAL(compensation_rd, compensation);

    pmic_testResultUpdate_pass(6180,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Parameter validation for handle
 */
static void test_pmic_rtc_setFreqCompPrmValTest_handle(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint16_t compensation = 34952U;

    test_pmic_print_unity_testcase_info(6181,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    status = Pmic_rtcSetFreqComp(NULL, compensation);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, status);

    pmic_testResultUpdate_pass(6181,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Test RTC for get RTC frequency compensation
 */
static void test_pmic_rtc_testGetFreqComp(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint16_t compensation = 34952U;
    uint16_t compensation_rd;

    test_pmic_print_unity_testcase_info(6182,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    status = Pmic_rtcSetFreqComp(pPmicCoreHandle, compensation);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status  = Pmic_rtcGetFreqComp(pPmicCoreHandle, &compensation_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    TEST_ASSERT_EQUAL(compensation, compensation_rd);

    pmic_testResultUpdate_pass(6182,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Parameter validation for handle
 */
static void test_pmic_rtc_getFreqCompPrmValTest_handle(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint16_t compensation = 34952U;

    test_pmic_print_unity_testcase_info(6183,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    status = Pmic_rtcGetFreqComp(NULL, &compensation);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, status);

    pmic_testResultUpdate_pass(6183,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Parameter validation for compensation
 */
static void test_pmic_rtc_getFreqCompPrmValTest_compensation(void)
{
    int32_t status = PMIC_ST_SUCCESS;

    test_pmic_print_unity_testcase_info(6287,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    status = Pmic_rtcGetFreqComp(pPmicCoreHandle, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    pmic_testResultUpdate_pass(6287,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);
}

/*!
 * \brief   RTC time interrupt
 */
static void test_pmic_rtc_testTimerIntr(void)
{
    int32_t            status       = PMIC_ST_SUCCESS;
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

    test_pmic_print_unity_testcase_info(6266,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

#if defined(SOC_J7200)
    if(PMIC_INTF_SPI == pPmicCoreHandle->commMode)
    {
        pmic_testResultUpdate_ignore(6266,
                                     pmic_rtc_tests,
                                     PMIC_RTC_NUM_OF_TESTCASES);
    }
#endif

    pHandle                         = pPmicCoreHandle;

    status = Pmic_rtcSetTimeDateInfo(pHandle, validTimeCfg, validDateCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_rtcGetTimeDateInfo(pHandle, &timeCfg_rd, &dateCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    /* To clear the interrupts*/
    status = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, true);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_rtcGetTimerPeriod(pHandle, &timerPeriod);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_rtcSetTimerPeriod(pHandle,
                                           PMIC_RTC_SECOND_INTR_PERIOD);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_rtcEnableTimerIntr(pHandle, PMIC_RTC_TIMER_INTR_ENABLE);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    while(timeout--)
    {
        /* Delay added to avoid timeout */
        Osal_delay(1000);

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

    if(0 > timeout)
    {
        status = PMIC_ST_ERR_FAIL;
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_rtcSetTimerPeriod(pHandle, timerPeriod);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    pmic_testResultUpdate_pass(6266,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);
}

/*!
 * \brief   RTC Alarm interrupt
 */
static void test_pmic_rtc_testAlarmIntr(void)
{
    int32_t            status       = PMIC_ST_SUCCESS;
    int8_t             timeout      = 10U;
    Pmic_CoreHandle_t  *pHandle     = NULL;
    pHandle                         = pPmicCoreHandle;
    Pmic_RtcTime_t     timeCfg_cr   = { 0x1F, 0U, 0U, 0U, 0U, 0U};
    Pmic_RtcDate_t     dateCfg_cr   = { 0x0F, 0U, 0U, 0U, 0U};
    Pmic_RtcTime_t     timeCfg_rd   = { 0x1F, 0U, 0U, 0U, 0U, 0U};
    Pmic_RtcDate_t     dateCfg_rd   = { 0x0F, 0U, 0U, 0U, 0U};
    Pmic_IrqStatus_t errStat        = {0U};
    bool clearIRQ                   = false;
    uint8_t  irqNum                 = 0U;
    Pmic_RtcDate_t    validDateCfg =  { 0x0F, 15U, 6U, 2055U, 1U};
    Pmic_RtcTime_t    validTimeCfg  = { 0x1F, 30U, 30U, 6U, 0U, 1U};

    test_pmic_print_unity_testcase_info(6267,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

#if defined(SOC_J7200)
    if(PMIC_INTF_SPI == pPmicCoreHandle->commMode)
    {
        pmic_testResultUpdate_ignore(6267,
                                     pmic_rtc_tests,
                                     PMIC_RTC_NUM_OF_TESTCASES);
    }
#endif

    status = Pmic_rtcSetTimeDateInfo(pHandle, validTimeCfg, validDateCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    /* Get the current time value */
    status = Pmic_rtcGetTimeDateInfo(pHandle, &timeCfg_rd, &dateCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    /* To clear the interrupts*/
    status = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, true);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    timeCfg_rd.seconds = timeCfg_rd.seconds + 3U;
    status   = Pmic_rtcSetAlarmInfo(pHandle, timeCfg_rd, dateCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    /* Set Alarm Interupt */
    status = Pmic_rtcEnableAlarmIntr(pHandle, PMIC_RTC_ALARM_INTR_ENABLE);
    /* Get the current time for timeout */
    status = Pmic_rtcGetTimeDateInfo(pHandle, &timeCfg_cr, &dateCfg_cr);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    while(timeout--)
    {
        /* Delay added to avoid timeout */
        Osal_delay(1000);

        status = Pmic_irqGetErrStatus(pHandle, &errStat, clearIRQ);
        if((PMIC_ST_SUCCESS == status) &&
           ((errStat.intStatus[PMIC_TPS6594X_RTC_ALARM_INT/32U] &
             (1U << (PMIC_TPS6594X_RTC_ALARM_INT % 32U))) != 0U))
        {
            while(PMIC_TPS6594X_RTC_ALARM_INT != irqNum)
            {
                status = Pmic_getNextErrorStatus(pHandle,
                                                 &errStat,
                                                 &irqNum);
            }

            if(PMIC_ST_SUCCESS == status)
            {
                /* clear the interrupt */
                status = Pmic_irqClrErrStatus(pPmicCoreHandle,
                                              PMIC_TPS6594X_RTC_ALARM_INT);
            }

            if(PMIC_ST_SUCCESS == status)
            {
                /* Interrupt received */
                /* Disable the alarm interrupt */
                status = Pmic_rtcEnableAlarmIntr(pHandle,
                                                 PMIC_RTC_ALARM_INTR_DISABLE);

                /* clear the interrupt */
                if(PMIC_ST_SUCCESS == status)
                {
                    break;
                }
            }
        }

        status = Pmic_rtcGetTimeDateInfo(pHandle, &timeCfg_cr, &dateCfg_cr);
        if(PMIC_ST_SUCCESS != status)
        {
            break;
        }
    }

    if(0 > timeout)
    {
        status = PMIC_ST_ERR_FAIL;
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    pmic_testResultUpdate_pass(6267,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Parameter validation for handle
 */
static void test_pmic_rtc_enableAlarmInterrupt_PrmValTest_handle(void)
{
    int32_t status = PMIC_ST_SUCCESS;

    test_pmic_print_unity_testcase_info(6268,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    status = Pmic_rtcEnableAlarmIntr(NULL, PMIC_RTC_ALARM_INTR_DISABLE);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, status);

    pmic_testResultUpdate_pass(6268,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Parameter validation for handle
 */
static void test_pmic_rtc_enableTimerInterrupt_PrmValTest_handle(void)
{
    int32_t status = PMIC_ST_SUCCESS;

    test_pmic_print_unity_testcase_info(6269,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    status = Pmic_rtcEnableTimerIntr(NULL, PMIC_RTC_TIMER_INTR_DISABLE);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, status);

    pmic_testResultUpdate_pass(6269,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);
}

/*!
 * \brief   RTC Status Validation RTC Current state Running
 */
static void test_pmic_rtc_testGetStatus_running(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    bool    rtcStatus;

    test_pmic_print_unity_testcase_info(6194,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    status = Pmic_rtcEnable(pPmicCoreHandle, PMIC_RTC_START);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_rtcGetStatus(pPmicCoreHandle, &rtcStatus);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    TEST_ASSERT_EQUAL(PMIC_RTC_STATUS_RUNNING, rtcStatus);

    pmic_testResultUpdate_pass(6194,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);
}

/*!
 * \brief   RTC Status Validation RTC Current state Frozen
 */
static void test_pmic_rtc_testGetStatus_frozen(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    bool    rtcStatus;

    test_pmic_print_unity_testcase_info(6089,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    status = Pmic_rtcEnable(pPmicCoreHandle, PMIC_RTC_STOP);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_rtcGetStatus(pPmicCoreHandle, &rtcStatus);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    TEST_ASSERT_EQUAL(PMIC_RTC_STATUS_FROZEN, rtcStatus);

    pmic_testResultUpdate_pass(6089,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Parameter validation for handle
 */
static void test_pmic_rtc_getRstStatus_PrmValTest_handle(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_RtcRstStatus_t pRtcRstStatus = {0};

    test_pmic_print_unity_testcase_info(7465,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    status = Pmic_rtcEnable(pPmicCoreHandle, PMIC_RTC_STOP);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_rtcGetRstStatus(NULL, &pRtcRstStatus);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, status);

    pmic_testResultUpdate_pass(7465,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Parameter validation for rtcStatus
 */
static void test_pmic_rtc_getRstStatus_PrmValTest_rtcStatus(void)
{
    int32_t status = PMIC_ST_SUCCESS;

    test_pmic_print_unity_testcase_info(7466,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    status = Pmic_rtcGetRstStatus(pPmicCoreHandle, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    pmic_testResultUpdate_pass(7466,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Parameter validation for ValidParams
 */
static void test_pmic_rtc_getRstStatus_PrmValTest_validParams(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_RtcRstStatus_t pRtcRstStatus = {0};

    test_pmic_print_unity_testcase_info(7467,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    status = Pmic_rtcGetRstStatus(pPmicCoreHandle, &pRtcRstStatus);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INSUFFICIENT_CFG, status);

    pmic_testResultUpdate_pass(7467,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);
}

/*!
 * \brief   RTC wakeup using time interrupt for LP Standby State
 */
static void test_pmic_rtc_testWakeup_TimerIntr_lpStandbyState(void)
{
    int32_t            status       = PMIC_ST_SUCCESS;
    Pmic_CoreHandle_t  *pHandle     = NULL;
    uint8_t            timerPeriod  = 0U;
    int8_t num                      = 0;
    Pmic_RtcDate_t    validDateCfg =  { 0x0F, 15U, 6U, 2055U, 1U};
    Pmic_RtcTime_t    validTimeCfg  = { 0x1F, 1U, 30U, 6U, 0U, 1U};
    Pmic_IrqStatus_t errStat  = {0U};

    test_pmic_print_unity_testcase_info(7358,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    if(PMIC_SILICON_REV_ID_PG_2_0 == pPmicCoreHandle->pmicDevSiliconRev)
    {
        /*Refer the Bug PDK-10450 for more details*/
        pmic_testResultUpdate_ignore(7358,
                                     pmic_rtc_tests,
                                     PMIC_RTC_NUM_OF_TESTCASES);
    }

    pHandle                         = pPmicCoreHandle;
#if defined(SOC_J721E)
    pmic_log("\r\n Probe TP134 and TP133 and it should be High");
#endif
#if defined(SOC_J7200)
    pmic_log("\r\n Probe TP46 and TP29 and it should be High");
#endif
    pmic_log("\r\n Enter 1 to continue");
    UART_scanFmt("%d", &num);

    status = Pmic_rtcGetTimerPeriod(pHandle, &timerPeriod);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_rtcSetTimerPeriod(pHandle, PMIC_RTC_MINUTE_INTR_PERIOD);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_rtcSetTimeDateInfo(pHandle, validTimeCfg, validDateCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    /* To clear the interrupts*/
    status = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, true);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    {
        int i = 0;
        for(i=0;i<4; i++)
        {
            pmic_log("\r\nINT STAT[%d]: 0x%08x", i, errStat.intStatus[i]);
        }
    }
#if defined(SOC_J721E)
    pmic_log("\r\n Probe TP134 and TP133 and it should be low after 2 sec");

    pmic_log("\r\n Probe TP134 and TP133 and it should be High after 60 sec");
#endif
#if defined(SOC_J7200)
    pmic_log("\r\n Probe TP46 and TP29 and it should be low after 2 sec");

    pmic_log("\r\n Probe TP46 and TP29 and it should be High after 60 sec");
#endif

    pmic_log("\r\n After 60sec Rerun the application in UART Boot mode");

    pmic_log("\r\n Also check for RTC Timer interrupt in Interrupt status register");

    status = Pmic_rtcEnableTimerIntr(pHandle, PMIC_RTC_TIMER_INTR_ENABLE);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_fsmSetNsleepSignalMask(pHandle,
                                         PMIC_NSLEEP1_SIGNAL,
                                         PMIC_NSLEEPX_MASK);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_fsmSetNsleepSignalMask(pHandle,
                                         PMIC_NSLEEP2_SIGNAL,
                                         PMIC_NSLEEPX_MASK);
     TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    /* Needs Delay to mask Nsleep1B and Nsleep2B signals for LP Stand-By State */
    Osal_delay(10U);

    status =  Pmic_fsmSetMissionState(pPmicCoreHandle, PMIC_FSM_LP_STANBY_STATE);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
}

/*!
 * \brief   RTC wakeup using time interrupt for Standby State
 */
static void test_pmic_rtc_testWakeup_TimerIntr_standbyState(void)
{
    int32_t            status       = PMIC_ST_SUCCESS;
    Pmic_CoreHandle_t  *pHandle     = NULL;
    uint8_t            timerPeriod  = 0U;
    int8_t num                      = 0;
    Pmic_IrqStatus_t errStat  = {0U};

    Pmic_RtcDate_t    validDateCfg =  { 0x0F, 15U, 6U, 2055U, 1U};
    Pmic_RtcTime_t    validTimeCfg  = { 0x1F, 0U, 30U, 6U, 0U, 1U};

    test_pmic_print_unity_testcase_info(8015,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    pHandle                         = pPmicCoreHandle;
#if defined(SOC_J721E)
    pmic_log("\r\n Probe TP134 and TP133 and it should be High");
#endif
#if defined(SOC_J7200)
    pmic_log("\r\n Probe TP46 and TP29 and it should be High");
#endif
    pmic_log("\r\n Enter 1 to continue");
    UART_scanFmt("%d", &num);

    status = Pmic_rtcSetTimeDateInfo(pHandle, validTimeCfg, validDateCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_rtcGetTimerPeriod(pHandle, &timerPeriod);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    /* To clear the interrupts*/
    status = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, true);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    {
        int i = 0;
        for(i=0;i<4; i++)
        {
            pmic_log("\r\nINT STAT[%d]: 0x%08x", i, errStat.intStatus[i]);
        }
    }
#if defined(SOC_J721E)
    pmic_log("\r\n Probe TP134 and TP133 and it should be low after 2 sec");

    pmic_log("\r\n Probe TP134 and TP133 and it should be High after 60 sec");
#endif
#if defined(SOC_J7200)
    pmic_log("\r\n Probe TP46 and TP29 and it should be low after 2 sec");

    pmic_log("\r\n Probe TP46 and TP29 and it should be High after 60 sec");
#endif

    pmic_log("\r\n After 60sec Rerun the application in UART Boot mode");

    pmic_log("\r\n Also check for RTC Timer interrupt in Interrupt status register");

    status = Pmic_rtcSetTimerPeriod(pHandle,
                                           PMIC_RTC_MINUTE_INTR_PERIOD);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_rtcEnableTimerIntr(pHandle, PMIC_RTC_TIMER_INTR_ENABLE);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_fsmSetNsleepSignalMask(pHandle,
                                         PMIC_NSLEEP1_SIGNAL,
                                         PMIC_NSLEEPX_MASK);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_fsmSetNsleepSignalMask(pHandle,
                                         PMIC_NSLEEP2_SIGNAL,
                                         PMIC_NSLEEPX_MASK);
     TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    /* Needs Delay to mask Nsleep1B and Nsleep2B signals for Stand-By State */
    Osal_delay(10U);

    status =  Pmic_fsmSetMissionState(pPmicCoreHandle, PMIC_FSM_STANBY_STATE);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
}

/*!
 * \brief   RTC wakeup using Alarm interrupt for Lp Standby State
 */
static void test_pmic_rtc_testWakeup_AlarmIntr_lpStandbyState(void)
{
    int32_t            status       = PMIC_ST_SUCCESS;
    Pmic_CoreHandle_t  *pHandle     = NULL;
    pHandle                         = pPmicCoreHandle;
    Pmic_RtcTime_t     timeCfg_cr   = { 0x1F, 0U, 0U, 0U, 0U, 0U};
    Pmic_RtcDate_t     dateCfg_cr   = { 0x0F, 0U, 0U, 0U, 0U};
    Pmic_RtcTime_t     timeCfg_rd   = { 0U};
    Pmic_RtcDate_t     dateCfg_rd   = { 0U};
    Pmic_RtcDate_t    validDateCfg =  { 0x0F, 15U, 6U, 2055U, 1U};
    Pmic_RtcTime_t    validTimeCfg  = { 0x1F, 0U, 30U, 6U, 0U, 1U};
    int8_t num = 0;
    Pmic_IrqStatus_t errStat  = {0U};

    test_pmic_print_unity_testcase_info(7359,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

#if defined(SOC_J721E)
    pmic_log("\r\n Probe TP134 and TP133 and it should be High");
#endif
#if defined(SOC_J7200)
    pmic_log("\r\n Probe TP46 and TP29 and it should be High");
#endif
    pmic_log("\r\n Enter 1 to continue");
    UART_scanFmt("%d", &num);

    status = Pmic_rtcSetTimeDateInfo(pHandle, validTimeCfg, validDateCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    dateCfg_rd.validParams = 0xF;
    timeCfg_rd.validParams = 0x1F;

    /* Get the current time value */
    status = Pmic_rtcGetTimeDateInfo(pHandle, &timeCfg_rd, &dateCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    /* To clear the interrupts*/
    status = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, true);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    {
        int i = 0;
        for(i=0;i<4; i++)
        {
            pmic_log("\r\nINT STAT[%d]: 0x%08x", i, errStat.intStatus[i]);
        }
    }
#if defined(SOC_J721E)
    pmic_log("\r\n Probe TP134 and TP133 and it should be low after 2 sec");

    pmic_log("\r\n Probe TP134 and TP133 and it should be High after 60 sec");
#endif
#if defined(SOC_J7200)
    pmic_log("\r\n Probe TP46 and TP29 and it should be low after 2 sec");

    pmic_log("\r\n Probe TP46 and TP29 and it should be High after 60 sec");
#endif

    pmic_log("\r\n After 60sec Rerun the application in UART Boot mode");

    pmic_log("\r\n Also check for RTC Alarm interrupt in Interrupt status register");


    timeCfg_rd.minutes = timeCfg_rd.minutes + 1U;
    status   = Pmic_rtcSetAlarmInfo(pHandle, timeCfg_rd, dateCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    /* Set Alarm Interupt */
    status = Pmic_rtcEnableAlarmIntr(pHandle, PMIC_RTC_ALARM_INTR_ENABLE);

    /* Get the current time for timeout */
    status = Pmic_rtcGetTimeDateInfo(pHandle, &timeCfg_cr, &dateCfg_cr);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_fsmSetNsleepSignalMask(pHandle,
                                         PMIC_NSLEEP1_SIGNAL,
                                         PMIC_NSLEEPX_MASK);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_fsmSetNsleepSignalMask(pHandle,
                                         PMIC_NSLEEP2_SIGNAL,
                                         PMIC_NSLEEPX_MASK);
     TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    /* Needs Delay to mask Nsleep1B and Nsleep2B signals LP Stand-By State */
    Osal_delay(10U);

    status =  Pmic_fsmSetMissionState(pPmicCoreHandle, PMIC_FSM_STANBY_STATE);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
}

/*!
 * \brief   RTC wakeup using Alarm interrupt for Standby State
 */
static void test_pmic_rtc_testWakeup_AlarmIntr_standbyState(void)
{
    int32_t            status       = PMIC_ST_SUCCESS;
    Pmic_CoreHandle_t  *pHandle     = NULL;
    pHandle                         = pPmicCoreHandle;
    Pmic_RtcTime_t     timeCfg_cr   = { 0x1F, 0U, 0U, 0U, 0U, 0U};
    Pmic_RtcDate_t     dateCfg_cr   = { 0x0F, 0U, 0U, 0U, 0U};
    Pmic_RtcTime_t     timeCfg_rd   = { 0U};
    Pmic_RtcDate_t     dateCfg_rd   = { 0U};
    Pmic_RtcDate_t    validDateCfg =  { 0x0F, 15U, 6U, 2055U, 1U};
    Pmic_RtcTime_t    validTimeCfg  = { 0x1F, 0U, 30U, 6U, 0U, 1U};
    bool standByState = 0U;
    int8_t num = 0;
    Pmic_IrqStatus_t errStat  = {0U};

    test_pmic_print_unity_testcase_info(8016,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

#if defined(SOC_J721E)
    pmic_log("\r\n Probe TP134 and TP133 and it should be High");
#endif
#if defined(SOC_J7200)
    pmic_log("\r\n Probe TP46 and TP29 and it should be High");
#endif
    pmic_log("\r\n Enter 1 to continue");
    UART_scanFmt("%d", &num);

    status = Pmic_rtcSetTimeDateInfo(pHandle, validTimeCfg, validDateCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    dateCfg_rd.validParams = 0xF;
    timeCfg_rd.validParams = 0x1F;

    /* Get the current time value */
    status = Pmic_rtcGetTimeDateInfo(pHandle, &timeCfg_rd, &dateCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    /* To clear the interrupts*/
    status = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, true);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    {
        int i = 0;
        for(i=0;i<4; i++)
        {
            pmic_log("\r\nINT STAT[%d]: 0x%08x", i, errStat.intStatus[i]);
        }
    }
#if defined(SOC_J721E)
    pmic_log("\r\n Probe TP134 and TP133 and it should be low after 2 sec");

    pmic_log("\r\n Probe TP134 and TP133 and it should be High after 60 sec");
#endif
#if defined(SOC_J7200)
    pmic_log("\r\n Probe TP46 and TP29 and it should be low after 2 sec");

    pmic_log("\r\n Probe TP46 and TP29 and it should be High after 60 sec");
#endif

    pmic_log("\r\n After 60sec Rerun the application in UART Boot mode");

    pmic_log("\r\n Also check for RTC Alarm interrupt in Interrupt status register");

    timeCfg_rd.minutes = timeCfg_rd.minutes + 1U;
    status   = Pmic_rtcSetAlarmInfo(pHandle, timeCfg_rd, dateCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    /* Set Alarm Interupt */
    status = Pmic_rtcEnableAlarmIntr(pHandle, PMIC_RTC_ALARM_INTR_ENABLE);

    /* Get the current time for timeout */
    status = Pmic_rtcGetTimeDateInfo(pHandle, &timeCfg_cr, &dateCfg_cr);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_fsmSetNsleepSignalMask(pHandle,
                                         PMIC_NSLEEP1_SIGNAL,
                                         PMIC_NSLEEPX_MASK);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_fsmSetNsleepSignalMask(pHandle,
                                         PMIC_NSLEEP2_SIGNAL,
                                         PMIC_NSLEEPX_MASK);
     TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    /* Needs Delay to mask Nsleep1B and Nsleep2B signals for Stand-By State */
    Osal_delay(10U);

    status =  Pmic_fsmSetMissionState(pPmicCoreHandle, standByState);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
}

/*!
 * \brief   RTC timer Asynchronous Interrupt
 */
static void test_pmic_rtc_testTimerAsyncIntr(void)
{
    int32_t            status       = PMIC_ST_SUCCESS;
    int8_t             timeout      = 10U;
    Pmic_CoreHandle_t  *pHandle     = NULL;
    uint8_t            timerPeriod  = 0U;
    Pmic_IrqStatus_t errStat        = {0U};
    Pmic_RtcDate_t    validDateCfg =  { 0x0F, 15U, 6U, 2055U, 1U};
    Pmic_RtcTime_t    validTimeCfg  = { 0x1F, 30U, 30U, 6U, 0U, 1U};

    test_pmic_print_unity_testcase_info(7888,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

#if ((defined(SOC_J7200) && (defined(BUILD_MCU2_0) || defined(BUILD_MCU2_1))) || \
     (defined(SOC_J721E) && (defined(BUILD_MCU3_0) || defined(BUILD_MCU3_1))))
    /*Refer the Bug-PDK-10394 for more details*/
    pmic_testResultUpdate_ignore(7888,
                                 pmic_rtc_tests,
                                 PMIC_RTC_NUM_OF_TESTCASES);
#endif
    /* Enable GPIO interrupt on the specific gpio pin */
    GPIO_enableInt(0);

    pHandle             = pPmicCoreHandle;
    pmic_intr_triggered = 0U;

    status = Pmic_rtcSetTimeDateInfo(pHandle, validTimeCfg, validDateCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    /* To clear the interrupts*/
    status = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, true);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_rtcGetTimerPeriod(pHandle, &timerPeriod);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_rtcSetTimerPeriod(pHandle,
                                           PMIC_RTC_SECOND_INTR_PERIOD);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_rtcEnableTimerIntr(pHandle, PMIC_RTC_TIMER_INTR_ENABLE);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    while(timeout--)
    {
        /* Wait for Interrupt */
        if(pmic_intr_triggered != 1U)
        {
            status = PMIC_ST_ERR_FAIL;
        }
        else
        {
            status = PMIC_ST_SUCCESS;
            break;
        }

        /* Delay added to avoid timeout */
        Osal_delay(1000);
    }

    if(0 > timeout)
    {
        status = PMIC_ST_ERR_FAIL;
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    /* Disable the timer interrupt  */
    status = Pmic_rtcEnableTimerIntr(pHandle, PMIC_RTC_TIMER_INTR_DISABLE);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_rtcSetTimerPeriod(pHandle, timerPeriod);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    /* Disable GPIO interrupt on the specific gpio pin */
    GPIO_disableInt(0);

    pmic_testResultUpdate_pass(7888,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);
}

/*!
 * \brief   RTC Alarm Asynchronous interrupt
 */
static void test_pmic_rtc_testAlarmAsyncIntr(void)
{
    int32_t            status       = PMIC_ST_SUCCESS;
    int8_t             timeout      = 10U;
    Pmic_CoreHandle_t  *pHandle     = NULL;
    pHandle                         = pPmicCoreHandle;
    Pmic_RtcTime_t     timeCfg_rd   = { 0x1F, 0U, 0U, 0U, 0U, 0U};
    Pmic_RtcDate_t     dateCfg_rd   = { 0x0F, 0U, 0U, 0U, 0U};
    Pmic_RtcDate_t    validDateCfg =  { 0x0F, 15U, 6U, 2055U, 1U};
    Pmic_RtcTime_t    validTimeCfg  = { 0x1F, 30U, 30U, 6U, 0U, 1U};
    Pmic_IrqStatus_t errStat        = {0U};

    test_pmic_print_unity_testcase_info(7889,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

#if ((defined(SOC_J7200) && (defined(BUILD_MCU2_0) || defined(BUILD_MCU2_1))) || \
     (defined(SOC_J721E) && (defined(BUILD_MCU3_0) || defined(BUILD_MCU3_1))))
    /*Refer the Bug-PDK-10394 for more details*/
    pmic_testResultUpdate_ignore(7889,
                                 pmic_rtc_tests,
                                 PMIC_RTC_NUM_OF_TESTCASES);
#endif
    /* Enable GPIO interrupt on the specific gpio pin */
    GPIO_enableInt(0);

    pmic_intr_triggered = 0U;

    status = Pmic_rtcSetTimeDateInfo(pHandle, validTimeCfg, validDateCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    /* To clear the interrupts*/
    status = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, true);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    /* Get the current time value */
    status = Pmic_rtcGetTimeDateInfo(pHandle, &timeCfg_rd, &dateCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    timeCfg_rd.seconds = timeCfg_rd.seconds + 3U;
    status   = Pmic_rtcSetAlarmInfo(pHandle, timeCfg_rd, dateCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    /* Set Alarm Interupt */
    status = Pmic_rtcEnableAlarmIntr(pHandle, PMIC_RTC_ALARM_INTR_ENABLE);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    while(timeout--)
    {
        /* Wait for Interrupt */
        if(pmic_intr_triggered != 2U)
        {
            status = PMIC_ST_ERR_FAIL;
        }
        else
        {
            status = PMIC_ST_SUCCESS;
            break;
        }

        /* Delay added to avoid timeout */
        Osal_delay(1000);
    }

    if(0 > timeout)
    {
        status = PMIC_ST_ERR_FAIL;
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    /* Disable the timer interrupt  */
    status = Pmic_rtcEnableTimerIntr(pHandle, PMIC_RTC_TIMER_INTR_DISABLE);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    /* Disable GPIO interrupt on the specific gpio pin */
    GPIO_disableInt(0);

    pmic_testResultUpdate_pass(7889,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_rtcEnable : Negative test for RTC state for HERA
 */
static void test_Pmic_rtcEnable_hera(void)
{
    int32_t status = PMIC_ST_SUCCESS;

    test_pmic_print_unity_testcase_info(7860,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(7860,
                                     pmic_rtc_tests,
                                     PMIC_RTC_NUM_OF_TESTCASES);
    }
    status = Pmic_rtcEnable(pPmicCoreHandle, PMIC_RTC_START);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_DEVICE, status);

    pmic_testResultUpdate_pass(7860,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);

}

/*!
 * \brief   Pmic_rtcEnableAlarmIntr : Negative test for RTC Alarm interrupt for HERA
 */
static void test_Pmic_rtcEnableAlarmIntr_hera(void)
{
    int32_t status = PMIC_ST_SUCCESS;

    test_pmic_print_unity_testcase_info(7861,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(7861,
                                     pmic_rtc_tests,
                                     PMIC_RTC_NUM_OF_TESTCASES);
    }

    status = Pmic_rtcEnableAlarmIntr(pPmicCoreHandle, PMIC_RTC_ALARM_INTR_DISABLE);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_DEVICE, status);

    pmic_testResultUpdate_pass(7861,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);

}

/*!
 * \brief   Pmic_rtcEnableTimerIntr :Negative test for RTC Timer interrupt for HERA
 */
static void test_Pmic_rtcEnableTimerIntr_hera(void)
{
    int32_t status = PMIC_ST_SUCCESS;

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(7862,
                                     pmic_rtc_tests,
                                     PMIC_RTC_NUM_OF_TESTCASES);
    }

    test_pmic_print_unity_testcase_info(7862,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    status = Pmic_rtcEnableTimerIntr(pPmicCoreHandle, PMIC_RTC_TIMER_INTR_DISABLE);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_DEVICE, status);

    pmic_testResultUpdate_pass(7862,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);

}

/*!
 * \brief   Pmic_rtcGetFreqComp :Negative test for Test RTC for set RTC frequency compensation for HERA
 */
static void test_pmic_rtc_testSetFreqComp_hera(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint16_t compensation_rd = 34952;

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(7863,
                                     pmic_rtc_tests,
                                     PMIC_RTC_NUM_OF_TESTCASES);
    }

    test_pmic_print_unity_testcase_info(7863,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    status  = Pmic_rtcSetFreqComp(pPmicCoreHandle, compensation_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_DEVICE, status);

    pmic_testResultUpdate_pass(7863,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);

}

/*!
 * \brief   Pmic_rtcSetTimeDateInfo : Negative test for Test RTC for Set Time for HERA
 */
static void test_Pmic_rtcSetTimeDateInfo_hera(void)
{
    int32_t status         = PMIC_ST_SUCCESS;

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(7864,
                                     pmic_rtc_tests,
                                     PMIC_RTC_NUM_OF_TESTCASES);
    }
    Pmic_RtcTime_t timeCfg = {PMIC_RTC_VALID_PARAM_TIME_CFG_VAL, 30U, 30U, 6U,
                              0U, 1U};
    Pmic_RtcDate_t dateCfg = {PMIC_RTC_VALID_PARAM_DATE_CFG_VAL, 15U, 6U,
                              2055U, 1U};

    test_pmic_print_unity_testcase_info(7864,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    timeCfg.timeMode       = PMIC_RTC_12_HOUR_MODE;

    status = Pmic_rtcSetTimeDateInfo(pPmicCoreHandle, timeCfg, dateCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_DEVICE, status);

    pmic_testResultUpdate_pass(7864,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);

}

/*!
 * \brief   Pmic_rtcGetAlarmInfo : Negative test for Test RTC Get Alarm for HERA
 */
static void test_Pmic_rtcGetAlarmInfo_hera(void)
{
    int32_t status         = PMIC_ST_SUCCESS;

    Pmic_RtcTime_t timeCfg_rd;
    Pmic_RtcDate_t dateCfg_rd;

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(7865,
                                     pmic_rtc_tests,
                                     PMIC_RTC_NUM_OF_TESTCASES);
    }

    test_pmic_print_unity_testcase_info(7865,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    status = Pmic_rtcGetAlarmInfo(pPmicCoreHandle, &timeCfg_rd, &dateCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_DEVICE, status);

    pmic_testResultUpdate_pass(7865,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_rtcSetAlarmInfo : Negative test for Set RTC Alarm interrupt for HERA
 */
static void test_pmic_rtc_testSetAlarm_hera(void)
{
    int32_t status             = PMIC_ST_SUCCESS;

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(7866,
                                     pmic_rtc_tests,
                                     PMIC_RTC_NUM_OF_TESTCASES);
    }

    Pmic_RtcTime_t timeCfg     = {PMIC_RTC_VALID_PARAM_TIME_CFG_VAL, 30U, 30U,
                                  6U, 0U, 1U};
    Pmic_RtcDate_t dateCfg     = {PMIC_RTC_VALID_PARAM_DATE_CFG_VAL, 15U, 6U,
                                  2055U, 1U};

    test_pmic_print_unity_testcase_info(7866,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    timeCfg.timeMode        = PMIC_RTC_12_HOUR_MODE;

    status = Pmic_rtcSetAlarmInfo(pPmicCoreHandle, timeCfg, dateCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_DEVICE, status);

    pmic_testResultUpdate_pass(7866,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);
}


/*!
 * \brief   Pmic_rtcSetTimerPeriod : Negative test for Test Set RTC Timer interrupt Period for hera
 */
static void test_pmic_rtc_testSetTimer_hera(void)
{
    int32_t status         = PMIC_ST_SUCCESS;
    uint8_t timerPeriod;

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(7867,
                                     pmic_rtc_tests,
                                     PMIC_RTC_NUM_OF_TESTCASES);
    }

    test_pmic_print_unity_testcase_info(7867,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    timerPeriod = PMIC_RTC_HOUR_INTR_PERIOD;

    status = Pmic_rtcSetTimerPeriod(pPmicCoreHandle, timerPeriod);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_DEVICE, status);

    pmic_testResultUpdate_pass(7867,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_rtcGetTimerPeriod : Negative test for Test Get RTC Timer period for hera
 */
static void test_pmic_rtc_testGetTimer_hera(void)
{
    int32_t status         = PMIC_ST_SUCCESS;
    uint8_t timerPeriod_rd;

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(7868,
                                     pmic_rtc_tests,
                                     PMIC_RTC_NUM_OF_TESTCASES);
    }

    test_pmic_print_unity_testcase_info(7868,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    status = Pmic_rtcGetTimerPeriod(pPmicCoreHandle, &timerPeriod_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_DEVICE, status);

    pmic_testResultUpdate_pass(7868,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);
}


/*!
 * \brief   Pmic_rtcGetRstStatus : Negative test for RTC Get Status for hera
 */
static void test_pmic_rtc_testGetRstStatus_running_hera(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_RtcRstStatus_t pRtcRstStatus = {0};

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(7869,
                                     pmic_rtc_tests,
                                     PMIC_RTC_NUM_OF_TESTCASES);
    }

    test_pmic_print_unity_testcase_info(7869,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    pRtcRstStatus.validParams |= PMIC_RTC_RESET_STATUS_VALID_SHIFT;
    status = Pmic_rtcGetRstStatus(pPmicCoreHandle, &pRtcRstStatus);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_DEVICE, status);

    pmic_testResultUpdate_pass(7869,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);
}


/*!
 * \brief   Pmic_rtcGetTimeDateInfo : Negative test for Test RTC for Get Time for HERA
 */
static void test_pmic_rtc_testGetTime_hera(void)
{
    int32_t status         = PMIC_ST_SUCCESS;
    Pmic_RtcTime_t timeCfg_rd;
    Pmic_RtcDate_t dateCfg_rd;

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(7870,
                                     pmic_rtc_tests,
                                     PMIC_RTC_NUM_OF_TESTCASES);
    }

    test_pmic_print_unity_testcase_info(7870,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    timeCfg_rd.validParams = PMIC_RTC_VALID_PARAM_TIME_CFG_VAL;
    dateCfg_rd.validParams = PMIC_RTC_VALID_PARAM_DATE_CFG_VAL;

    status = Pmic_rtcGetTimeDateInfo(pPmicCoreHandle, &timeCfg_rd, &dateCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_DEVICE, status);

    pmic_testResultUpdate_pass(7870,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_rtcGetFreqComp : Negative test for Test RTC for get RTC frequency compensation for HERA
 */
static void test_pmic_rtc_testGetFreqComp_hera(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint16_t compensation_rd = 0U;

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(7871,
                                     pmic_rtc_tests,
                                     PMIC_RTC_NUM_OF_TESTCASES);
    }

    test_pmic_print_unity_testcase_info(7871,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    status  = Pmic_rtcGetFreqComp(pPmicCoreHandle, &compensation_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_DEVICE, status);

    pmic_testResultUpdate_pass(7871,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);
}


/*!
 * \brief   Pmic_rtcSetAlarmInfo : Parameter Validation for RTC day min value in Feb Month
 */
static void test_pmic_rtc_setAlarmInfoPrmValTest_febMonth_day(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_RtcDate_t dateCfg     = {PMIC_RTC_VALID_PARAM_DATE_CFG_VAL, 15U, 6U,
                                  2055U, 1U};
    Pmic_RtcTime_t timeCfg     = {PMIC_RTC_VALID_PARAM_TIME_CFG_VAL, 30U, 30U,
                                  6U, 1U, 1U};

    test_pmic_print_unity_testcase_info(10605,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    dateCfg.month = 2U;
    dateCfg.day = 0U;
    status = Pmic_rtcSetAlarmInfo(pPmicCoreHandle, timeCfg, dateCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_DATE, status);

    pmic_testResultUpdate_pass(10605,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_rtcSetAlarmInfo : Parameter Validation for RTC day min value
 */
static void test_pmic_rtc_setAlarmInfoPrmValTest_day_min(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_RtcDate_t dateCfg     = {PMIC_RTC_VALID_PARAM_DATE_CFG_VAL, 15U, 6U,
                                  2055U, 1U};
    Pmic_RtcTime_t timeCfg     = {PMIC_RTC_VALID_PARAM_TIME_CFG_VAL, 30U, 30U,
                                  6U, 1U, 1U};

    test_pmic_print_unity_testcase_info(10606,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    dateCfg.month = 1U;
    dateCfg.day = 0U;
    status = Pmic_rtcSetAlarmInfo(pPmicCoreHandle, timeCfg, dateCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_DATE, status);

    pmic_testResultUpdate_pass(10606,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_rtcSetAlarmInfo : Parameter Validation for RTC day max value
 */
static void test_pmic_rtc_setAlarmInfoPrmValTest_day_max(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_RtcDate_t dateCfg     = {PMIC_RTC_VALID_PARAM_DATE_CFG_VAL, 15U, 6U,
                                  2055U, 1U};
    Pmic_RtcTime_t timeCfg     = {PMIC_RTC_VALID_PARAM_TIME_CFG_VAL, 30U, 30U,
                                  6U, 1U, 1U};

    test_pmic_print_unity_testcase_info(10607,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    dateCfg.month = 1U;
    dateCfg.day = 32U;
    status = Pmic_rtcSetAlarmInfo(pPmicCoreHandle, timeCfg, dateCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_DATE, status);

    pmic_testResultUpdate_pass(10607,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_rtcSetAlarmInfo : Parameter Validation for RTC month min value
 */
static void test_pmic_rtc_setAlarmInfoPrmValTest_month_min(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_RtcDate_t dateCfg     = {PMIC_RTC_VALID_PARAM_DATE_CFG_VAL, 15U, 6U,
                                  2055U, 1U};
    Pmic_RtcTime_t timeCfg     = {PMIC_RTC_VALID_PARAM_TIME_CFG_VAL, 30U, 30U,
                                  6U, 1U, 1U};

    test_pmic_print_unity_testcase_info(10608,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    dateCfg.month = 0U;
    dateCfg.day = 1U;
    status = Pmic_rtcSetAlarmInfo(pPmicCoreHandle, timeCfg, dateCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_DATE, status);

    pmic_testResultUpdate_pass(10608,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_rtcSetAlarmInfo : Parameter Validation for RTC month max value
 */
static void test_pmic_rtc_setAlarmInfoPrmValTest_month_max(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_RtcDate_t dateCfg     = {PMIC_RTC_VALID_PARAM_DATE_CFG_VAL, 15U, 6U,
                                  2055U, 1U};
    Pmic_RtcTime_t timeCfg     = {PMIC_RTC_VALID_PARAM_TIME_CFG_VAL, 30U, 30U,
                                  6U, 1U, 1U};

    test_pmic_print_unity_testcase_info(10609,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    dateCfg.month = 13U;
    status = Pmic_rtcSetAlarmInfo(pPmicCoreHandle, timeCfg, dateCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_DATE, status);

    pmic_testResultUpdate_pass(10609,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_rtcSetAlarmInfo : Parameter Validation for RTC year min value
 */
static void test_pmic_rtc_setAlarmInfoPrmValTest_year_min(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_RtcDate_t dateCfg     = {PMIC_RTC_VALID_PARAM_DATE_CFG_VAL, 15U, 6U,
                                  2055U, 1U};
    Pmic_RtcTime_t timeCfg     = {PMIC_RTC_VALID_PARAM_TIME_CFG_VAL, 30U, 30U,
                                  6U, 1U, 1U};

    test_pmic_print_unity_testcase_info(10610,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    dateCfg.month = 12U;
    dateCfg.year = 1996U;
    status = Pmic_rtcSetAlarmInfo(pPmicCoreHandle, timeCfg, dateCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_DATE, status);

    pmic_testResultUpdate_pass(10610,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_rtcSetAlarmInfo : Parameter Validation for RTC weekday min value
 */
static void test_pmic_rtc_setAlarmInfoPrmValTest_weekday_min(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_RtcDate_t dateCfg     = {PMIC_RTC_VALID_PARAM_DATE_CFG_VAL, 15U, 6U,
                                  2055U, 1U};
    Pmic_RtcTime_t timeCfg     = {PMIC_RTC_VALID_PARAM_TIME_CFG_VAL, 30U, 30U,
                                  6U, 1U, 1U};

    test_pmic_print_unity_testcase_info(10611,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    dateCfg.year = 2055U;
    dateCfg.weekday =0U;
    status = Pmic_rtcSetAlarmInfo(pPmicCoreHandle, timeCfg, dateCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_DATE, status);

    pmic_testResultUpdate_pass(10611,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_rtcSetAlarmInfo : Parameter Validation for RTC Meridian Mode
 */
static void test_pmic_rtc_setAlarmInfoPrmValTest_meridianMode_12HrMode(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_RtcDate_t dateCfg     = {PMIC_RTC_VALID_PARAM_DATE_CFG_VAL, 15U, 6U,
                                  2055U, 1U};
    Pmic_RtcTime_t timeCfg     = {PMIC_RTC_VALID_PARAM_TIME_CFG_VAL, 30U, 30U,
                                  6U, 1U, 1U};

    test_pmic_print_unity_testcase_info(10612,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    /* Checking for valid params Hrs and Timemode for invalid meridianMode */
    timeCfg.validParams = PMIC_RTC_TIME_CFG_HRS_VALID_SHIFT |
                          PMIC_RTC_TIME_CFG_TIMEMODE_VALID_SHIFT;
    timeCfg.meridianMode = 0x2U;
    status = Pmic_rtcSetAlarmInfo(pPmicCoreHandle, timeCfg, dateCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_TIME, status);

    pmic_testResultUpdate_pass(10612,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_rtcSetAlarmInfo : Parameter Validation for RTC weekday
 */
static void test_pmic_rtc_setAlarmInfoPrmValTest_weekday(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_RtcDate_t dateCfg     = {PMIC_RTC_VALID_PARAM_DATE_CFG_VAL, 15U, 6U,
                                  2055U, 1U};
    Pmic_RtcTime_t timeCfg     = {PMIC_RTC_VALID_PARAM_TIME_CFG_VAL, 30U, 30U,
                                  6U, 1U, 1U};

    test_pmic_print_unity_testcase_info(10613,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    timeCfg.meridianMode = 1U;
    dateCfg.weekday = 8U;
    status = Pmic_rtcSetAlarmInfo(pPmicCoreHandle, timeCfg, dateCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_DATE, status);

    pmic_testResultUpdate_pass(10613,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_rtcSetAlarmInfo : Parameter Validation for dateCfg and timeCfg
 */
static void test_pmic_rtc_setAlarmInfoPrmValTest_dateTimeCfg(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_RtcDate_t dateCfg     = {PMIC_RTC_VALID_PARAM_DATE_CFG_VAL, 15U, 6U,
                                  2055U, 1U};
    Pmic_RtcTime_t timeCfg     = {PMIC_RTC_VALID_PARAM_TIME_CFG_VAL, 30U, 30U,
                                  6U, 1U, 1U};

    test_pmic_print_unity_testcase_info(10614,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    dateCfg.weekday = 1U;
    dateCfg.validParams = 0U;
    timeCfg.validParams = 0U;
    status = Pmic_rtcSetAlarmInfo(pPmicCoreHandle, timeCfg, dateCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INSUFFICIENT_CFG, status);

    pmic_testResultUpdate_pass(10614,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_rtcSetTimeDateInfo : Parameter Validation for dateCfg and timeCfg
 */
static void test_pmic_rtc_setTimeDateInfoPrmValTest_dateTimeCfg(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_RtcDate_t dateCfg     = {PMIC_RTC_VALID_PARAM_DATE_CFG_VAL, 15U, 6U,
                                  2055U, 1U};
    Pmic_RtcTime_t timeCfg     = {PMIC_RTC_VALID_PARAM_TIME_CFG_VAL, 30U, 30U,
                                  6U, 1U, 1U};

    test_pmic_print_unity_testcase_info(10615,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    dateCfg.weekday = 1U;
    dateCfg.validParams = 0U;
    timeCfg.validParams = 0U;
    status = Pmic_rtcSetTimeDateInfo(pPmicCoreHandle, timeCfg, dateCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INSUFFICIENT_CFG, status);

    pmic_testResultUpdate_pass(10615,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_rtcGetAlarmInfo : Parameter Validation for pDateCfg and pTimeCfg
 */
static void test_pmic_rtc_getAlarmInfoPrmValTest_pDatePtimeCfg(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_RtcTime_t timeCfg_rd;
    Pmic_RtcDate_t dateCfg_rd;

    test_pmic_print_unity_testcase_info(10616,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    timeCfg_rd.validParams = 0x0U;
    dateCfg_rd.validParams = 0x0U;
    status = Pmic_rtcGetAlarmInfo(pPmicCoreHandle, &timeCfg_rd, &dateCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INSUFFICIENT_CFG, status);

    pmic_testResultUpdate_pass(10616,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_rtcGetTimeDateInfo : Parameter Validation for pDateCfg and pTimeCfg
 */
static void test_pmic_rtc_getTimeDateInfoPrmValTest_pDatePtimeCfg(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_RtcTime_t timeCfg_rd;
    Pmic_RtcDate_t dateCfg_rd;

    test_pmic_print_unity_testcase_info(10617,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    timeCfg_rd.validParams = 0x0U;
    dateCfg_rd.validParams = 0x0U;
    status = Pmic_rtcGetTimeDateInfo(pPmicCoreHandle, &timeCfg_rd, &dateCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INSUFFICIENT_CFG, status);

    pmic_testResultUpdate_pass(10617,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_rtcGetRstStatus : Test to get RTC Power Up status
 */
static void test_pmic_rtc_getRstStatus_powerUpStatus(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_RtcRstStatus_t pRtcRstStatus = {0};

    test_pmic_print_unity_testcase_info(10618,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    pRtcRstStatus.validParams |= PMIC_RTC_POWERUP_STATUS_VALID_SHIFT;
    status = Pmic_rtcGetRstStatus(pPmicCoreHandle, &pRtcRstStatus);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    pmic_testResultUpdate_pass(10618,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_rtcSetTimeDateInfo : Parameter validation to day min value in leap year
 */
static void test_pmic_rtc_setTimeDateInfoPrmValTest_day_min_leapyear(void)
{
    int32_t status         = PMIC_ST_SUCCESS;

    Pmic_RtcTime_t timeCfg = {PMIC_RTC_VALID_PARAM_TIME_CFG_VAL, 30U, 30U, 6U,
                              0U, 1U};
    Pmic_RtcDate_t dateCfg = {PMIC_RTC_VALID_PARAM_DATE_CFG_VAL, 15U, 6U,
                              2055U, 1U};

    test_pmic_print_unity_testcase_info(10619,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    dateCfg.validParams = PMIC_RTC_DATE_CFG_DAY_VALID_SHIFT |
                          PMIC_RTC_DATE_CFG_YEAR_VALID_SHIFT |
                          PMIC_RTC_DATE_CFG_MONTH_VALID_SHIFT;
    dateCfg.month = 2U;
    dateCfg.year  = 2056U;
    dateCfg.day   = 0U;
    status = Pmic_rtcSetTimeDateInfo(pPmicCoreHandle, timeCfg, dateCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_DATE, status);

    pmic_testResultUpdate_pass(10619,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_rtcGetTimeDateInfo : Test to set RTC timeDateRegSel as static shadowed registers
 */
static void test_pmic_rtc_getTimeDateInfo_timeDateRegSel_staticShadow(void)
{
    int32_t status        = PMIC_ST_SUCCESS;
    Pmic_CoreHandle_t  *pHandle     = NULL;
    pHandle                         = pPmicCoreHandle;
    Pmic_RtcCfg_t rtcCfg_rd = {PMIC_RTC_CFG_TIME_DATE_REG_SEL_VALID_SHIFT,};
    Pmic_RtcCfg_t rtcCfg_default = {PMIC_RTC_CFG_TIME_DATE_REG_SEL_VALID_SHIFT,};
    Pmic_RtcCfg_t rtcCfg =
    {
        PMIC_RTC_CFG_TIME_DATE_REG_SEL_VALID_SHIFT,
        PMIC_RTC_CRYSTAL_OSC_ENABLE,
        PMIC_RTC_32K_COUNTER_COMP_VAL_SET,
        PMIC_RTC_ROUND_TIME_SET,
        PMIC_RTC_STATIC_SHADOWED_REG_SEL,
        PMIC_RTC_CRYSTAL_OSC_TYPE_9PF
    };

    Pmic_RtcTime_t timeCfg_rd;
    Pmic_RtcDate_t dateCfg_rd;

    test_pmic_print_unity_testcase_info(10620,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    status = Pmic_rtcEnable(pHandle, PMIC_RTC_STOP);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_rtcGetConfiguration(pHandle, &rtcCfg_default);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_rtcSetConfiguration(pHandle, rtcCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_rtcGetConfiguration(pHandle, &rtcCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    TEST_ASSERT_EQUAL(rtcCfg.timeDateRegSel,
                      rtcCfg_rd.timeDateRegSel);

    status = Pmic_rtcGetTimeDateInfo(pHandle, &timeCfg_rd, &dateCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_rtcSetConfiguration(pHandle, rtcCfg_default);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_rtcGetConfiguration(pHandle, &rtcCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    TEST_ASSERT_EQUAL(rtcCfg_default.timeDateRegSel,
                      rtcCfg_rd.timeDateRegSel);

    status = Pmic_rtcEnable(pHandle, PMIC_RTC_START);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    pmic_testResultUpdate_pass(10620,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_rtcSetConfiguration : Test to set rtc configuration when RTC is started
 */
static void test_pmic_rtc_setConfiguration_rtcIsStarted(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_RtcCfg_t rtcCfg =
    {
        PMIC_RTC_CFG_32K_COUNTER_COMP_VAL_SET_VALID_SHIFT,
        PMIC_RTC_CRYSTAL_OSC_ENABLE,
        PMIC_RTC_32K_COUNTER_COMP_VAL_SET,
        PMIC_RTC_ROUND_TIME_SET,
        PMIC_RTC_STATIC_SHADOWED_REG_SEL,
        PMIC_RTC_CRYSTAL_OSC_TYPE_9PF
    };

    test_pmic_print_unity_testcase_info(10621,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    status = Pmic_rtcEnable(pPmicCoreHandle, PMIC_RTC_START);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_rtcSetConfiguration(pPmicCoreHandle, rtcCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_FAIL, status);

    pmic_testResultUpdate_pass(10621,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Added for Coverage
 */
static void test_pmic_rtc_coverageGaps(void)
{
    int32_t status             = PMIC_ST_SUCCESS;
    Pmic_RtcRstStatus_t pRtcRstStatus = {0};
    Pmic_RtcRstStatus_t rtcRstStatus = {PMIC_RTC_POWERUP_STATUS_VALID_SHIFT, 0U, 0U};
    Pmic_RtcTime_t timeCfg     = {PMIC_RTC_VALID_PARAM_TIME_CFG_VAL, 30U, 30U,
                                  6U, 1U, 1U};
    Pmic_RtcDate_t dateCfg     = {PMIC_RTC_VALID_PARAM_DATE_CFG_VAL, 15U, 6U,
                                  2055U, 1U};
    Pmic_RtcCfg_t rtcCfg_rd;
    Pmic_RtcCfg_t rtcCfg_default = {PMIC_RTC_CFG_TIME_DATE_REG_SEL_VALID_SHIFT,};
    Pmic_RtcCfg_t rtcCfg =
    {
        PMIC_RTC_CFG_CRYSTAL_OSC_TYPE_VALID_SHIFT,
        PMIC_RTC_CRYSTAL_OSC_DISABLE,
        PMIC_RTC_32K_COUNTER_COMP_VAL_SET,
        PMIC_RTC_ROUND_TIME_SET,
        PMIC_RTC_STATIC_SHADOWED_REG_SEL,
        PMIC_RTC_CRYSTAL_OSC_TYPE_6PF
    };

    Pmic_RtcTime_t timeCfg_rd;
    Pmic_RtcDate_t dateCfg_rd;
    int32_t crcTestFlag = PMIC_STATUS_CRC_INIT_VAL;

#if defined(SOC_J721E)
    crcTestFlag = gCrcTestFlag_J721E;
#endif

#if defined(SOC_J7200)
    crcTestFlag = gCrcTestFlag_J7VCL;
#endif

    test_pmic_print_unity_testcase_info(8814,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    timeCfg_rd.validParams = PMIC_RTC_VALID_PARAM_TIME_CFG_VAL;
    dateCfg_rd.validParams = PMIC_RTC_VALID_PARAM_DATE_CFG_VAL;
    status = Pmic_rtcSetAlarmInfo(pPmicCoreHandle, timeCfg, dateCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    /* Fault Injection Tests for Branch Coverage */
    //Pmic_rtcGetSeconds
    gPmic_faultInjectCfg.enableFaultInjectionRead = 1U;
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    status = Pmic_rtcGetAlarmInfo(pPmicCoreHandle, &timeCfg_rd, &dateCfg_rd);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    //Pmic_rtcGetMinutes
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    timeCfg_rd.validParams = PMIC_RTC_TIME_CFG_MIN_VALID_SHIFT;
    status = Pmic_rtcGetAlarmInfo(pPmicCoreHandle, &timeCfg_rd, &dateCfg_rd);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    //Pmic_rtcGetTimeMode
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    timeCfg_rd.validParams = PMIC_RTC_TIME_CFG_TIMEMODE_VALID_SHIFT;
    status = Pmic_rtcGetAlarmInfo(pPmicCoreHandle, &timeCfg_rd, &dateCfg_rd);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    //Pmic_rtcSetTimeMode
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    timeCfg.validParams = PMIC_RTC_TIME_CFG_TIMEMODE_VALID_SHIFT;
    status = Pmic_rtcSetAlarmInfo(pPmicCoreHandle, timeCfg, dateCfg);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    //Pmic_rtcSetMeridianMode Alarm
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 2;
    timeCfg.validParams = PMIC_RTC_TIME_CFG_MERIDIAN_VALID_SHIFT;
    status = Pmic_rtcSetAlarmInfo(pPmicCoreHandle, timeCfg, dateCfg);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 3;
    timeCfg.validParams = PMIC_RTC_TIME_CFG_MERIDIAN_VALID_SHIFT;
    status = Pmic_rtcSetAlarmInfo(pPmicCoreHandle, timeCfg, dateCfg);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    //Pmic_rtcGetMeridianMode alarm
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    timeCfg_rd.validParams = PMIC_RTC_TIME_CFG_MERIDIAN_VALID_SHIFT;
    status = Pmic_rtcGetAlarmInfo(pPmicCoreHandle, &timeCfg_rd, &dateCfg_rd);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 2;
    timeCfg_rd.validParams = PMIC_RTC_TIME_CFG_MERIDIAN_VALID_SHIFT;
    status = Pmic_rtcGetAlarmInfo(pPmicCoreHandle, &timeCfg_rd, &dateCfg_rd);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    //Pmic_rtcSetMeridianMode Timer
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 5;
    timeCfg.validParams = PMIC_RTC_TIME_CFG_MERIDIAN_VALID_SHIFT;
    status = Pmic_rtcSetTimeDateInfo(pPmicCoreHandle, timeCfg, dateCfg);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    //Pmic_rtcGetMeridianMode Timer
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 3;
    timeCfg_rd.validParams = PMIC_RTC_TIME_CFG_MERIDIAN_VALID_SHIFT;
    status = Pmic_rtcGetTimeDateInfo(pPmicCoreHandle, &timeCfg_rd, &dateCfg_rd);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    //Pmic_rtcSetHours alarm
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 2;
    timeCfg.validParams = PMIC_RTC_TIME_CFG_HRS_VALID_SHIFT;
    status = Pmic_rtcSetAlarmInfo(pPmicCoreHandle, timeCfg, dateCfg);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    //Pmic_rtcGetHours alarm
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    timeCfg_rd.validParams = PMIC_RTC_TIME_CFG_HRS_VALID_SHIFT;
    status = Pmic_rtcGetAlarmInfo(pPmicCoreHandle, &timeCfg_rd, &dateCfg_rd);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    //Pmic_rtcSetHours Timer
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 4;
    timeCfg.validParams = PMIC_RTC_TIME_CFG_HRS_VALID_SHIFT;
    status = Pmic_rtcSetTimeDateInfo(pPmicCoreHandle, timeCfg, dateCfg);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    //Pmic_rtcGetHours Timer
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 2;
    timeCfg_rd.validParams = PMIC_RTC_TIME_CFG_HRS_VALID_SHIFT;
    status = Pmic_rtcGetTimeDateInfo(pPmicCoreHandle, &timeCfg_rd, &dateCfg_rd);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    //Pmic_rtcGetDay
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    timeCfg_rd.validParams = 0U;
    dateCfg_rd.validParams = PMIC_RTC_DATE_CFG_DAY_VALID_SHIFT;
    status = Pmic_rtcGetAlarmInfo(pPmicCoreHandle, &timeCfg_rd, &dateCfg_rd);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    //Pmic_rtcGetMonth
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    dateCfg_rd.validParams = PMIC_RTC_DATE_CFG_MONTH_VALID_SHIFT;
    status = Pmic_rtcGetAlarmInfo(pPmicCoreHandle, &timeCfg_rd, &dateCfg_rd);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    //Pmic_rtcGetYear
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    dateCfg_rd.validParams = PMIC_RTC_DATE_CFG_YEAR_VALID_SHIFT;
    status = Pmic_rtcGetAlarmInfo(pPmicCoreHandle, &timeCfg_rd, &dateCfg_rd);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    //Pmic_rtcGetWeekday
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 2;
    dateCfg_rd.validParams = PMIC_RTC_DATE_CFG_WEEKDAY_VALID_SHIFT;
    status = Pmic_rtcGetTimeDateInfo(pPmicCoreHandle, &timeCfg_rd, &dateCfg_rd);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    //Pmic_rtcCheckHoursMode
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    dateCfg.validParams = PMIC_RTC_VALID_PARAM_DATE_CFG_VAL;
    timeCfg.validParams = PMIC_RTC_TIME_CFG_HRS_VALID_SHIFT;
    status = Pmic_rtcSetAlarmInfo(pPmicCoreHandle, timeCfg, dateCfg);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    //Pmic_rtcCheckDate 1
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    dateCfg.validParams = PMIC_RTC_DATE_CFG_WEEKDAY_VALID_SHIFT;
    timeCfg.validParams = PMIC_RTC_TIME_CFG_TIMEMODE_VALID_SHIFT;
    status = Pmic_rtcSetAlarmInfo(pPmicCoreHandle, timeCfg, dateCfg);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    //Pmic_rtcCheckDate 2
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    dateCfg.validParams = PMIC_RTC_DATE_CFG_YEAR_VALID_SHIFT;
    status = Pmic_rtcSetAlarmInfo(pPmicCoreHandle, timeCfg, dateCfg);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    dateCfg.validParams = 0U;
    status = Pmic_rtcSetAlarmInfo(pPmicCoreHandle, timeCfg, dateCfg);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    //Pmic_alarmSetTime
    gPmic_faultInjectCfg.enableFaultInjectionRead = 0U;
    gPmic_faultInjectCfg.enableFaultInjectionWrite = 1U;
    gPmic_faultInjectCfg.writeCount = 0;
    gPmic_faultInjectCfg.skipWriteCount = 1;
    timeCfg.validParams = PMIC_RTC_TIME_CFG_MIN_VALID_SHIFT | PMIC_RTC_TIME_CFG_HRS_VALID_SHIFT | PMIC_RTC_TIME_CFG_TIMEMODE_VALID_SHIFT | PMIC_RTC_TIME_CFG_MERIDIAN_VALID_SHIFT;
    status = Pmic_rtcSetAlarmInfo(pPmicCoreHandle, timeCfg, dateCfg);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    gPmic_faultInjectCfg.writeCount = 0;
    gPmic_faultInjectCfg.skipWriteCount = 1;
    timeCfg.validParams = PMIC_RTC_VALID_PARAM_TIME_CFG_VAL;
    status = Pmic_rtcSetAlarmInfo(pPmicCoreHandle, timeCfg, dateCfg);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);
    gPmic_faultInjectCfg.enableFaultInjectionRead = 1U;
    gPmic_faultInjectCfg.enableFaultInjectionWrite = 0U;

    //Pmic_alarmSetDate
    gPmic_faultInjectCfg.enableFaultInjectionRead = 0U;
    gPmic_faultInjectCfg.enableFaultInjectionWrite = 1U;
    gPmic_faultInjectCfg.writeCount = 0;
    gPmic_faultInjectCfg.skipWriteCount = 1;
    timeCfg.validParams = 0U;
    dateCfg.validParams = PMIC_RTC_DATE_CFG_MONTH_VALID_SHIFT | PMIC_RTC_DATE_CFG_YEAR_VALID_SHIFT;
    status = Pmic_rtcSetAlarmInfo(pPmicCoreHandle, timeCfg, dateCfg);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    gPmic_faultInjectCfg.writeCount = 0;
    gPmic_faultInjectCfg.skipWriteCount = 1;
    dateCfg.validParams = PMIC_RTC_VALID_PARAM_DATE_CFG_VAL;
    status = Pmic_rtcSetAlarmInfo(pPmicCoreHandle, timeCfg, dateCfg);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);
    gPmic_faultInjectCfg.enableFaultInjectionWrite = 0U;
    gPmic_faultInjectCfg.enableFaultInjectionRead = 1U;

    //Pmic_setAlarmIntr
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    status = Pmic_rtcEnableAlarmIntr(pPmicCoreHandle, PMIC_RTC_ALARM_INTR_ENABLE);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    //Pmic_alarmGetTime
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    timeCfg_rd.validParams = PMIC_RTC_VALID_PARAM_TIME_CFG_VAL;
    dateCfg_rd.validParams = 0U;
    status = Pmic_rtcGetAlarmInfo(pPmicCoreHandle, &timeCfg_rd, &dateCfg_rd);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    //Pmic_alarmGetDate
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    timeCfg_rd.validParams = 0U;
    dateCfg_rd.validParams = PMIC_RTC_VALID_PARAM_DATE_CFG_DAY_MNTH_YR_VAL;
    status = Pmic_rtcGetAlarmInfo(pPmicCoreHandle, &timeCfg_rd, &dateCfg_rd);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    //Pmic_rtcSetTime
    gPmic_faultInjectCfg.enableFaultInjectionRead = 0U;
    gPmic_faultInjectCfg.enableFaultInjectionWrite = 1U;
    gPmic_faultInjectCfg.writeCount = 0;
    gPmic_faultInjectCfg.skipWriteCount = 2;
    timeCfg.validParams = PMIC_RTC_TIME_CFG_MIN_VALID_SHIFT | PMIC_RTC_TIME_CFG_HRS_VALID_SHIFT | PMIC_RTC_TIME_CFG_TIMEMODE_VALID_SHIFT | PMIC_RTC_TIME_CFG_MERIDIAN_VALID_SHIFT;
    status = Pmic_rtcSetTimeDateInfo(pPmicCoreHandle, timeCfg, dateCfg);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    gPmic_faultInjectCfg.writeCount = 0;
    gPmic_faultInjectCfg.skipWriteCount = 2;
    timeCfg.validParams = PMIC_RTC_VALID_PARAM_TIME_CFG_VAL;
    status = Pmic_rtcSetTimeDateInfo(pPmicCoreHandle, timeCfg, dateCfg);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);
    gPmic_faultInjectCfg.enableFaultInjectionRead = 1U;
    gPmic_faultInjectCfg.enableFaultInjectionWrite = 0U;

    //Pmic_rtcSetDate
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 4;
    timeCfg.validParams = 0U;
    dateCfg.validParams = PMIC_RTC_DATE_CFG_MONTH_VALID_SHIFT | PMIC_RTC_DATE_CFG_YEAR_VALID_SHIFT | PMIC_RTC_DATE_CFG_WEEKDAY_VALID_SHIFT;
    status = Pmic_rtcSetTimeDateInfo(pPmicCoreHandle, timeCfg, dateCfg);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    gPmic_faultInjectCfg.enableFaultInjectionRead = 0U;
    gPmic_faultInjectCfg.enableFaultInjectionWrite = 1U;
    gPmic_faultInjectCfg.writeCount = 0;
    gPmic_faultInjectCfg.skipWriteCount = 2;
    timeCfg.validParams = 0U;
    dateCfg.validParams = PMIC_RTC_DATE_CFG_DAY_VALID_SHIFT | PMIC_RTC_DATE_CFG_YEAR_VALID_SHIFT | PMIC_RTC_DATE_CFG_WEEKDAY_VALID_SHIFT;
    status = Pmic_rtcSetTimeDateInfo(pPmicCoreHandle, timeCfg, dateCfg);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);
    gPmic_faultInjectCfg.enableFaultInjectionRead = 1U;
    gPmic_faultInjectCfg.enableFaultInjectionWrite = 0U;

    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 4;
    timeCfg.validParams = 0U;
    dateCfg.validParams = PMIC_RTC_DATE_CFG_DAY_VALID_SHIFT | PMIC_RTC_DATE_CFG_MONTH_VALID_SHIFT | PMIC_RTC_DATE_CFG_WEEKDAY_VALID_SHIFT;
    status = Pmic_rtcSetTimeDateInfo(pPmicCoreHandle, timeCfg, dateCfg);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 4;
    timeCfg.validParams = 0U;
    dateCfg.validParams = PMIC_RTC_DATE_CFG_DAY_VALID_SHIFT | PMIC_RTC_DATE_CFG_MONTH_VALID_SHIFT | PMIC_RTC_DATE_CFG_YEAR_VALID_SHIFT;
    status = Pmic_rtcSetTimeDateInfo(pPmicCoreHandle, timeCfg, dateCfg);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    gPmic_faultInjectCfg.enableFaultInjectionRead = 0U;
    gPmic_faultInjectCfg.enableFaultInjectionWrite = 1U;
    gPmic_faultInjectCfg.writeCount = 0;
    gPmic_faultInjectCfg.skipWriteCount = 2;
    dateCfg.validParams = PMIC_RTC_VALID_PARAM_DATE_CFG_VAL;
    status = Pmic_rtcSetTimeDateInfo(pPmicCoreHandle, timeCfg, dateCfg);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);
    gPmic_faultInjectCfg.enableFaultInjectionRead = 1U;
    gPmic_faultInjectCfg.enableFaultInjectionWrite = 0U;

    //Pmic_rtcTriggerShadowRegisters
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    timeCfg_rd.validParams = PMIC_RTC_TIME_CFG_HRS_VALID_SHIFT;
    status = Pmic_rtcGetTimeDateInfo(pPmicCoreHandle, &timeCfg_rd, &dateCfg_rd);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    //Pmic_rtcGetTime
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 2;
    timeCfg_rd.validParams = PMIC_RTC_TIME_CFG_MIN_VALID_SHIFT | PMIC_RTC_TIME_CFG_HRS_VALID_SHIFT | PMIC_RTC_TIME_CFG_TIMEMODE_VALID_SHIFT | PMIC_RTC_TIME_CFG_MERIDIAN_VALID_SHIFT;
    status = Pmic_rtcGetTimeDateInfo(pPmicCoreHandle, &timeCfg_rd, &dateCfg_rd);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 2;
    timeCfg_rd.validParams = PMIC_RTC_TIME_CFG_SEC_VALID_SHIFT | PMIC_RTC_TIME_CFG_HRS_VALID_SHIFT | PMIC_RTC_TIME_CFG_TIMEMODE_VALID_SHIFT | PMIC_RTC_TIME_CFG_MERIDIAN_VALID_SHIFT;
    status = Pmic_rtcGetTimeDateInfo(pPmicCoreHandle, &timeCfg_rd, &dateCfg_rd);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 2;
    timeCfg_rd.validParams = PMIC_RTC_TIME_CFG_MIN_VALID_SHIFT | PMIC_RTC_TIME_CFG_SEC_VALID_SHIFT | PMIC_RTC_TIME_CFG_TIMEMODE_VALID_SHIFT | PMIC_RTC_TIME_CFG_MERIDIAN_VALID_SHIFT;
    status = Pmic_rtcGetTimeDateInfo(pPmicCoreHandle, &timeCfg_rd, &dateCfg_rd);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 2;
    timeCfg_rd.validParams = PMIC_RTC_TIME_CFG_MIN_VALID_SHIFT | PMIC_RTC_TIME_CFG_HRS_VALID_SHIFT | PMIC_RTC_TIME_CFG_SEC_VALID_SHIFT | PMIC_RTC_TIME_CFG_MERIDIAN_VALID_SHIFT;
    status = Pmic_rtcGetTimeDateInfo(pPmicCoreHandle, &timeCfg_rd, &dateCfg_rd);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 2;
    timeCfg_rd.validParams = PMIC_RTC_TIME_CFG_MIN_VALID_SHIFT | PMIC_RTC_TIME_CFG_HRS_VALID_SHIFT | PMIC_RTC_TIME_CFG_TIMEMODE_VALID_SHIFT | PMIC_RTC_TIME_CFG_SEC_VALID_SHIFT;
    status = Pmic_rtcGetTimeDateInfo(pPmicCoreHandle, &timeCfg_rd, &dateCfg_rd);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    //Pmic_rtcGetDate
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 2;
    timeCfg_rd.validParams = 0U;
    dateCfg_rd.validParams = PMIC_RTC_DATE_CFG_MONTH_VALID_SHIFT | PMIC_RTC_DATE_CFG_YEAR_VALID_SHIFT | PMIC_RTC_DATE_CFG_WEEKDAY_VALID_SHIFT;
    status = Pmic_rtcGetTimeDateInfo(pPmicCoreHandle, &timeCfg_rd, &dateCfg_rd);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 2;
    timeCfg_rd.validParams = 0U;
    dateCfg_rd.validParams = PMIC_RTC_DATE_CFG_DAY_VALID_SHIFT | PMIC_RTC_DATE_CFG_YEAR_VALID_SHIFT | PMIC_RTC_DATE_CFG_WEEKDAY_VALID_SHIFT;
    status = Pmic_rtcGetTimeDateInfo(pPmicCoreHandle, &timeCfg_rd, &dateCfg_rd);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 2;
    timeCfg_rd.validParams = 0U;
    dateCfg_rd.validParams = PMIC_RTC_DATE_CFG_MONTH_VALID_SHIFT | PMIC_RTC_DATE_CFG_DAY_VALID_SHIFT | PMIC_RTC_DATE_CFG_WEEKDAY_VALID_SHIFT;
    status = Pmic_rtcGetTimeDateInfo(pPmicCoreHandle, &timeCfg_rd, &dateCfg_rd);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 2;
    timeCfg_rd.validParams = 0U;
    dateCfg_rd.validParams = PMIC_RTC_DATE_CFG_MONTH_VALID_SHIFT | PMIC_RTC_DATE_CFG_YEAR_VALID_SHIFT | PMIC_RTC_DATE_CFG_DAY_VALID_SHIFT;
    status = Pmic_rtcGetTimeDateInfo(pPmicCoreHandle, &timeCfg_rd, &dateCfg_rd);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    //Pmic_rtcSetFreqCompensateVal
    gPmic_faultInjectCfg.enableFaultInjectionRead = 0U;
    gPmic_faultInjectCfg.enableFaultInjectionWrite = 1U;
    gPmic_faultInjectCfg.writeCount = 0;
    gPmic_faultInjectCfg.skipWriteCount = 1;
    status = Pmic_rtcSetFreqComp(pPmicCoreHandle, 34952U);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);
    gPmic_faultInjectCfg.enableFaultInjectionRead = 1U;
    gPmic_faultInjectCfg.enableFaultInjectionWrite = 0U;

    //Pmic_rtcGetFreqCompensateVal
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    uint16_t compensation = 0U;
    status = Pmic_rtcGetFreqComp(pPmicCoreHandle, &compensation);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    //Pmic_rtcSetTimerIntr
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    status = Pmic_rtcEnableTimerIntr(pPmicCoreHandle, PMIC_RTC_TIMER_INTR_DISABLE);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    //Pmic_rtcEnableRtc
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    status = Pmic_rtcEnable(pPmicCoreHandle, PMIC_RTC_STOP);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 2;
    status = Pmic_rtcEnable(pPmicCoreHandle, PMIC_RTC_START);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 2;
    status = Pmic_rtcEnable(pPmicCoreHandle, PMIC_RTC_STOP);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    //Pmic_rtcSetTimerPeriod
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    status = Pmic_rtcSetTimerPeriod(pPmicCoreHandle, PMIC_RTC_HOUR_INTR_PERIOD);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    //Pmic_rtcGetTimerPeriod
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    uint8_t timePeriod = 0U;
    status = Pmic_rtcGetTimerPeriod(pPmicCoreHandle, &timePeriod);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    //Pmic_rtcSetCrystalOscCfg
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    rtcCfg.validParams = PMIC_RTC_CFG_CRYSTAL_OSC_EN_VALID_SHIFT;
    status = Pmic_rtcSetConfiguration(pPmicCoreHandle, rtcCfg);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    //Pmic_rtcSelectCrystalOscType
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    rtcCfg.validParams = PMIC_RTC_CFG_CRYSTAL_OSC_TYPE_VALID_SHIFT;
    status = Pmic_rtcSetConfiguration(pPmicCoreHandle, rtcCfg);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    //Pmic_rtcGetCrystalOscTypeCfg
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    rtcCfg_rd.validParams = PMIC_RTC_CFG_CRYSTAL_OSC_TYPE_VALID_SHIFT;
    status = Pmic_rtcGetConfiguration(pPmicCoreHandle, &rtcCfg_rd);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    //Pmic_rtcSelectTimeDateReg
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    rtcCfg.validParams = PMIC_RTC_CFG_TIME_DATE_REG_SEL_VALID_SHIFT;
    status = Pmic_rtcSetConfiguration(pPmicCoreHandle, rtcCfg);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    //Pmic_rtcGetTimeDateRegCfg
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    rtcCfg_rd.validParams = PMIC_RTC_CFG_TIME_DATE_REG_SEL_VALID_SHIFT;
    status = Pmic_rtcGetConfiguration(pPmicCoreHandle, &rtcCfg_rd);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    //Pmic_rtcSetRtcTimeRound30s
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    rtcCfg.validParams = PMIC_RTC_CFG_RTC_TIME_ROUND_30S_SET_VALID_SHIFT;
    status = Pmic_rtcSetConfiguration(pPmicCoreHandle, rtcCfg);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    //Pmic_rtcGetRtcTimeRound30s
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    rtcCfg_rd.validParams = PMIC_RTC_CFG_RTC_TIME_ROUND_30S_SET_VALID_SHIFT;
    status = Pmic_rtcGetConfiguration(pPmicCoreHandle, &rtcCfg_rd);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    //Pmic_rtcEnableCrystalOsc
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    rtcCfg.validParams = PMIC_RTC_CFG_CRYSTAL_OSC_EN_VALID_SHIFT;
    status = Pmic_rtcSetConfiguration(pPmicCoreHandle, rtcCfg);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    //Pmic_rtcGetCrystalOscEnCfg
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    rtcCfg_rd.validParams = PMIC_RTC_CFG_CRYSTAL_OSC_EN_VALID_SHIFT;
    status = Pmic_rtcGetConfiguration(pPmicCoreHandle, &rtcCfg_rd);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    //Pmic_rtcSetHours
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 6;
    timeCfg.validParams = PMIC_RTC_TIME_CFG_HRS_VALID_SHIFT | PMIC_RTC_TIME_CFG_TIMEMODE_VALID_SHIFT;
    status = Pmic_rtcSetTimeDateInfo(pPmicCoreHandle, timeCfg, dateCfg);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    //Pmic_rtcSetMeridianMode
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 6;
    timeCfg.validParams = PMIC_RTC_TIME_CFG_MERIDIAN_VALID_SHIFT;
    status = Pmic_rtcSetTimeDateInfo(pPmicCoreHandle, timeCfg, dateCfg);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    rtcCfg_rd.validParams= PMIC_RTC_CFG_TIME_DATE_REG_SEL_VALID_SHIFT;
    rtcCfg.validParams = PMIC_RTC_CFG_TIME_DATE_REG_SEL_VALID_SHIFT;
    status = Pmic_rtcSetConfiguration(pPmicCoreHandle, rtcCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    //Pmic_rtcTriggerShadowRegisters
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 2;
    status = Pmic_rtcGetTimeDateInfo(pPmicCoreHandle, &timeCfg_rd, &dateCfg_rd);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    status = Pmic_rtcSetConfiguration(pPmicCoreHandle, rtcCfg_default);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_rtcGetConfiguration(pPmicCoreHandle, &rtcCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    TEST_ASSERT_EQUAL(rtcCfg_default.timeDateRegSel, rtcCfg_rd.timeDateRegSel);

    //Pmic_rtcClrRstStatus
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    status = Pmic_rtcClrRstStatus(pPmicCoreHandle, PMIC_RTC_RST_STATUS);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    //Pmic_rtcClrRstStatus
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    status = Pmic_rtcClrRstStatus(pPmicCoreHandle, PMIC_RTC_POWERUP_STATUS);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    //Pmic_rtcSet32KCounterCompVal
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    rtcCfg.validParams = PMIC_RTC_CFG_32K_COUNTER_COMP_VAL_SET_VALID_SHIFT;
    status = Pmic_rtcSetConfiguration(pPmicCoreHandle, rtcCfg);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    status = Pmic_rtcEnable(pPmicCoreHandle, PMIC_RTC_STOP);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    //Pmic_rtcSet32KCounterCompVal
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 2;
    rtcCfg.validParams = PMIC_RTC_CFG_32K_COUNTER_COMP_VAL_SET_VALID_SHIFT;
    status = Pmic_rtcSetConfiguration(pPmicCoreHandle, rtcCfg);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    status = Pmic_rtcEnable(pPmicCoreHandle, PMIC_RTC_START);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    //Pmic_rtcGet32KCounterCompValCfg
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    rtcCfg_rd.validParams = PMIC_RTC_CFG_32K_COUNTER_COMP_VAL_SET_VALID_SHIFT;
    status = Pmic_rtcGetConfiguration(pPmicCoreHandle, &rtcCfg_rd);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    gPmic_faultInjectCfg.enableFaultInjectionRead = 0U;
    gPmic_faultInjectCfg.enableFaultInjectionWrite = 0U;
    gPmic_faultInjectCfg.skipReadCount = 0U;
    gPmic_faultInjectCfg.skipWriteCount = 0U;
    gPmic_faultInjectCfg.readCount = 0xFFU;
    gPmic_faultInjectCfg.writeCount = 0xFFU;

    gPmic_faultInjectCfg.enableFaultInjectionRead = 1U;

    //Pmic_rtcGetPowerupStatus
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    status = Pmic_rtcGetRstStatus(pPmicCoreHandle, &rtcRstStatus);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    //Pmic_rtcGetRstStatus
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    rtcRstStatus.validParams = PMIC_RTC_RESET_STATUS_VALID_SHIFT;
    status = Pmic_rtcGetRstStatus(pPmicCoreHandle, &rtcRstStatus);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    gPmic_faultInjectCfg.enableFaultInjectionRead = 0U;

    if(PMIC_STATUS_CRC_INIT_VAL == crcTestFlag)
    {
        //Pmic_rtcEnableRtc
        gEnableRtcTestFlag = 1U;
        status = Pmic_rtcEnable(pPmicCoreHandle, PMIC_RTC_STOP);
        TEST_ASSERT_EQUAL(PMIC_ST_ERR_RTC_STOP_FAIL, status);
        gEnableRtcTestFlag = 0U;
    }

    Pmic_DevSubSysInfo_t pmicDevSubSysInfo =
    {
        .gpioEnable = (bool)true,
        .rtcEnable  = (bool)false,
        .wdgEnable  = (bool)true,
        .buckEnable = (bool)true,
        .ldoEnable  = (bool)true,
        .esmEnable  = (bool)true
    };

    //Pmic_rtcSetConfiguration
    pPmicCoreHandle->pPmic_SubSysInfo = (&pmicDevSubSysInfo);
    status = Pmic_rtcSetConfiguration(pPmicCoreHandle, rtcCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_DEVICE, status);

    //Pmic_rtcGetConfiguration
    status = Pmic_rtcGetConfiguration(pPmicCoreHandle, &rtcCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_DEVICE, status);

    //Pmic_rtcClrRstStatus
    uint8_t rstStatType = PMIC_RTC_POWERUP_STATUS;
    status = Pmic_rtcClrRstStatus(pPmicCoreHandle, rstStatType);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_DEVICE, status);

    //Pmic_rtcClrRstStatus
    bool rtcStatus;
    status = Pmic_rtcGetStatus(pPmicCoreHandle, &rtcStatus);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_DEVICE, status);

    //Pmic_rtcEnableTimerIntr
    status = Pmic_rtcEnableTimerIntr(pPmicCoreHandle, PMIC_RTC_TIMER_INTR_DISABLE);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_DEVICE, status);

    //Pmic_rtcEnableAlarmIntr
    status = Pmic_rtcEnableAlarmIntr(pPmicCoreHandle, PMIC_RTC_ALARM_INTR_DISABLE);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_DEVICE, status);

    //Pmic_rtcEnable
    status = Pmic_rtcEnable(pPmicCoreHandle, PMIC_RTC_START);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_DEVICE, status);

    //Pmic_rtcSetFreqComp
    status = Pmic_rtcSetFreqComp(pPmicCoreHandle, 34952U);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_DEVICE, status);

    //Pmic_rtcGetFreqComp
    status = Pmic_rtcGetFreqComp(pPmicCoreHandle, &compensation);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_DEVICE, status);

    //Pmic_rtcSetTimeDateInfo
    status = Pmic_rtcSetTimeDateInfo(pPmicCoreHandle, timeCfg, dateCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_DEVICE, status);

    //Pmic_rtcGetTimeDateInfo
    status = Pmic_rtcGetTimeDateInfo(pPmicCoreHandle, &timeCfg_rd, &dateCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_DEVICE, status);

    //Pmic_rtcSetTimerPeriod
    status = Pmic_rtcSetTimerPeriod(pPmicCoreHandle, PMIC_RTC_HOUR_INTR_PERIOD);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_DEVICE, status);

    //Pmic_rtcGetTimerPeriod
    status = Pmic_rtcGetTimerPeriod(pPmicCoreHandle, &timePeriod);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_DEVICE, status);

    //Pmic_rtcSetAlarmInfo
    status = Pmic_rtcSetAlarmInfo(pPmicCoreHandle, timeCfg, dateCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_DEVICE, status);

    //Pmic_rtcGetAlarmInfo
    status = Pmic_rtcGetAlarmInfo(pPmicCoreHandle, &timeCfg_rd, &dateCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_DEVICE, status);

    //Pmic_rtcGetRstStatus
    status = Pmic_rtcGetRstStatus(pPmicCoreHandle, &pRtcRstStatus);
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

    pmic_testResultUpdate_pass(8814,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_rtcGetStatus : Parameter validation for handle
 */
static void test_pmic_rtc_getStatus_PrmValTest_handle(void)
{
    int32_t            status       = PMIC_ST_SUCCESS;
    bool               rtcStatus;

    test_pmic_print_unity_testcase_info(9895,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    status = Pmic_rtcGetStatus(NULL, &rtcStatus);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, status);

    pmic_testResultUpdate_pass(9895,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_rtcGetStatus : Parameter validation for pRtcstatus
 */
static void test_pmic_rtc_getStatus_PrmValTest_pRtcstatus(void)
{
    int32_t            status       = PMIC_ST_SUCCESS;
    Pmic_CoreHandle_t  *pHandle     = NULL;
    pHandle                         = pPmicCoreHandle;

    test_pmic_print_unity_testcase_info(9896,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    status = Pmic_rtcGetStatus(pHandle, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    pmic_testResultUpdate_pass(9896,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_rtcGetConfiguration : Parameter validation for handle
 */
static void test_pmic_rtc_getCfgPrmValTest_handle(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_CoreHandle_t  *pHandle     = NULL;
    Pmic_RtcCfg_t rtcCfg;

    test_pmic_print_unity_testcase_info(9897,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    pmicStatus = Pmic_rtcGetConfiguration(pHandle, &rtcCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, pmicStatus);

    pmic_testResultUpdate_pass(9897,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_rtcGetConfiguration : Parameter validation for pRtcCfg
 */
static void test_pmic_rtc_getCfgPrmValTest_pRtcCfg(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_CoreHandle_t  *pHandle     = NULL;
    pHandle                         = pPmicCoreHandle;

    test_pmic_print_unity_testcase_info(9898,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    pmicStatus = Pmic_rtcGetConfiguration(pHandle, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(9898,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_rtcClrRstStatus : Clear RTC Reset status using powerupStatus
 */
static void test_pmic_rtc_clrRstStatus_powerupStatus(void)
{
    int32_t            status       = PMIC_ST_SUCCESS;
    Pmic_CoreHandle_t  *pHandle     = NULL;
    pHandle                         = pPmicCoreHandle;
    Pmic_RtcRstStatus_t rtcRstStatus = {0U};
    uint8_t rstStatType = PMIC_RTC_POWERUP_STATUS;
    int8_t  num         = 0;

    pmic_log("\r\n Run this test alone after POR");
    pmic_log("\r\n Enter 1 to continue");
    UART_scanFmt("%d", &num);

    rtcRstStatus.validParams = PMIC_RTC_POWERUP_STATUS_VALID_SHIFT;

    test_pmic_print_unity_testcase_info(9899,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    status = Pmic_rtcGetRstStatus(pHandle, &rtcRstStatus);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    TEST_ASSERT_EQUAL(PMIC_RTC_POWERUP_STATUS_RESET_VAL,
                      rtcRstStatus.powerupStatus);

    status = Pmic_rtcClrRstStatus(pHandle, rstStatType);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_rtcGetRstStatus(pHandle, &rtcRstStatus);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    TEST_ASSERT_EQUAL(PMIC_RTC_POWERUP_STATUS_CLR_VAL,
                      rtcRstStatus.powerupStatus);

    pmic_testResultUpdate_pass(9899,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_rtcClrRstStatus : Clear RTC Reset status using rtcRstStatus
 */
static void test_pmic_rtc_clrRstStatus_rtcRstStatus(void)
{
    int32_t            status       = PMIC_ST_SUCCESS;
    Pmic_CoreHandle_t  *pHandle     = NULL;
    pHandle                         = pPmicCoreHandle;
    Pmic_RtcRstStatus_t rtcRstStatus = {0U};
    uint8_t rstStatType = PMIC_RTC_RST_STATUS;
    int8_t  num         = 0;

    pmic_log("\r\n Run this test alone after POR");
    pmic_log("\r\n Enter 1 to continue");
    UART_scanFmt("%d", &num);

    rtcRstStatus.validParams = PMIC_RTC_RESET_STATUS_VALID_SHIFT;

    test_pmic_print_unity_testcase_info(9900,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    status = Pmic_rtcGetRstStatus(pHandle, &rtcRstStatus);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    TEST_ASSERT_EQUAL(PMIC_RTC_RST_STATUS_RESET_VAL,
                      rtcRstStatus.rtcRstStatus);

    status = Pmic_rtcClrRstStatus(pHandle, rstStatType);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_rtcGetRstStatus(pHandle, &rtcRstStatus);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    TEST_ASSERT_EQUAL(PMIC_RTC_RST_STATUS_SET_VAL,
                      rtcRstStatus.rtcRstStatus);

    pmic_testResultUpdate_pass(9900,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_rtcClrRstStatus : Parameter validation for handle
 */
static void test_pmic_rtc_clrRstStatus_PrmValTest_handle(void)
{
    int32_t status       = PMIC_ST_SUCCESS;
    uint8_t rstStatType = PMIC_RTC_RST_STATUS;

    test_pmic_print_unity_testcase_info(9901,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    status = Pmic_rtcClrRstStatus(NULL, rstStatType);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, status);

    pmic_testResultUpdate_pass(9901,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_rtcClrRstStatus : Parameter validation for rtcRstStatType
 */
static void test_pmic_rtc_clrRstStatus_PrmValTest_rtcRstStatType(void)
{
    int32_t            status       = PMIC_ST_SUCCESS;
    Pmic_CoreHandle_t  *pHandle     = NULL;
    pHandle                         = pPmicCoreHandle;
    uint8_t             rstStatType = 2U;

    test_pmic_print_unity_testcase_info(9902,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    status = Pmic_rtcClrRstStatus(pHandle, rstStatType);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

    pmic_testResultUpdate_pass(9902,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_rtcSetConfiguration : Set 32K counter with compensation values
 */
static void test_pmic_rtc_setCfg_set32KCounterCompVal(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_RtcCfg_t rtcCfg_rd = {PMIC_RTC_CFG_32K_COUNTER_COMP_VAL_SET_VALID_SHIFT,};
    Pmic_CoreHandle_t  *pHandle     = NULL;
    pHandle                         = pPmicCoreHandle;
    int8_t  num         = 0;
    Pmic_RtcCfg_t rtcCfg =
    {
        PMIC_RTC_CFG_32K_COUNTER_COMP_VAL_SET_VALID_SHIFT,
        PMIC_RTC_CRYSTAL_OSC_ENABLE,
        PMIC_RTC_32K_COUNTER_COMP_VAL_SET,
        PMIC_RTC_ROUND_TIME_SET,
        PMIC_RTC_STATIC_SHADOWED_REG_SEL,
        PMIC_RTC_CRYSTAL_OSC_TYPE_9PF
    };

    test_pmic_print_unity_testcase_info(9903,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    pmicStatus = Pmic_rtcEnable(pPmicCoreHandle, PMIC_RTC_STOP);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_rtcSetConfiguration(pHandle, rtcCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_rtcGetConfiguration(pHandle, &rtcCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(rtcCfg.set32KCounterCompVal,
                      rtcCfg_rd.set32KCounterCompVal);

    pmicStatus = Pmic_rtcEnable(pPmicCoreHandle, PMIC_RTC_START);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmic_testResultUpdate_pass(9903,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);

    pmic_log("\r\n Do the power cycle to test the other RTC tests\n");
    pmic_log("\r\n Enter 1 to continue");
    UART_scanFmt("%d", &num);
}

/*!
 * \brief   Pmic_rtcSetConfiguration : Configure Enable/Disable Crystal Oscillator
 */
static void test_pmic_rtc_setCfg_crystalOScEn(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_RtcCfg_t rtcCfg_rd = {PMIC_RTC_CFG_CRYSTAL_OSC_EN_VALID_SHIFT,};
    Pmic_RtcCfg_t rtcCfg_default = {PMIC_RTC_CFG_CRYSTAL_OSC_EN_VALID_SHIFT,};
    Pmic_CoreHandle_t  *pHandle     = NULL;
    pHandle                         = pPmicCoreHandle;
    Pmic_RtcCfg_t rtcCfg =
    {
        PMIC_RTC_CFG_CRYSTAL_OSC_EN_VALID_SHIFT,
        PMIC_RTC_CRYSTAL_OSC_DISABLE,
        PMIC_RTC_32K_COUNTER_COMP_VAL_SET,
        PMIC_RTC_ROUND_TIME_SET,
        PMIC_RTC_STATIC_SHADOWED_REG_SEL,
        PMIC_RTC_CRYSTAL_OSC_TYPE_9PF
    };

    test_pmic_print_unity_testcase_info(9904,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    pmicStatus = Pmic_rtcGetConfiguration(pHandle, &rtcCfg_default);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_rtcSetConfiguration(pHandle, rtcCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_rtcGetConfiguration(pHandle, &rtcCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(rtcCfg.crystalOScEn, rtcCfg_rd.crystalOScEn);

    rtcCfg.crystalOScEn = PMIC_RTC_CRYSTAL_OSC_ENABLE;
    pmicStatus = Pmic_rtcSetConfiguration(pHandle, rtcCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_rtcGetConfiguration(pHandle, &rtcCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(rtcCfg.crystalOScEn, rtcCfg_rd.crystalOScEn);

    pmicStatus = Pmic_rtcSetConfiguration(pHandle, rtcCfg_default);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_rtcGetConfiguration(pHandle, &rtcCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(rtcCfg_default.crystalOScEn, rtcCfg_rd.crystalOScEn);

    pmic_testResultUpdate_pass(9904,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_rtcSetConfiguration : Configure RTC time to round the time to closest minute
 */
static void test_pmic_rtc_setCfg_rtcTimeRound30s(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_RtcCfg_t rtcCfg_rd = {PMIC_RTC_CFG_RTC_TIME_ROUND_30S_SET_VALID_SHIFT,};
    Pmic_CoreHandle_t  *pHandle     = NULL;
    pHandle                         = pPmicCoreHandle;
    Pmic_RtcCfg_t rtcCfg =
    {
        PMIC_RTC_CFG_RTC_TIME_ROUND_30S_SET_VALID_SHIFT,
        PMIC_RTC_CRYSTAL_OSC_ENABLE,
        PMIC_RTC_32K_COUNTER_COMP_VAL_SET,
        PMIC_RTC_ROUND_TIME_SET,
        PMIC_RTC_STATIC_SHADOWED_REG_SEL,
        PMIC_RTC_CRYSTAL_OSC_TYPE_9PF
    };

    test_pmic_print_unity_testcase_info(9905,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    pmicStatus = Pmic_rtcSetConfiguration(pHandle, rtcCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_rtcGetConfiguration(pHandle, &rtcCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(rtcCfg.setRtcTimeRound30s, rtcCfg_rd.setRtcTimeRound30s);

    pmic_testResultUpdate_pass(9905,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_rtcSetConfiguration : Select RTC Time and Date Register read from Dynamic Registers
 */
static void test_pmic_rtc_setCfg_timeDateRegSel_dynamic(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_RtcCfg_t rtcCfg_rd = {PMIC_RTC_CFG_TIME_DATE_REG_SEL_VALID_SHIFT,};
    Pmic_RtcCfg_t rtcCfg_default = {PMIC_RTC_CFG_TIME_DATE_REG_SEL_VALID_SHIFT,};
    Pmic_CoreHandle_t  *pHandle     = NULL;
    pHandle                         = pPmicCoreHandle;
    Pmic_RtcCfg_t rtcCfg =
    {
        PMIC_RTC_CFG_TIME_DATE_REG_SEL_VALID_SHIFT,
        PMIC_RTC_CRYSTAL_OSC_ENABLE,
        PMIC_RTC_32K_COUNTER_COMP_VAL_SET,
        PMIC_RTC_ROUND_TIME_SET,
        PMIC_RTC_DYNAMIC_REG_SEL,
        PMIC_RTC_CRYSTAL_OSC_TYPE_9PF
    };

    test_pmic_print_unity_testcase_info(9907,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    pmicStatus = Pmic_rtcGetConfiguration(pHandle, &rtcCfg_default);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_rtcSetConfiguration(pHandle, rtcCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_rtcGetConfiguration(pHandle, &rtcCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(rtcCfg.timeDateRegSel, rtcCfg_rd.timeDateRegSel);

    pmicStatus = Pmic_rtcSetConfiguration(pHandle, rtcCfg_default);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_rtcGetConfiguration(pHandle, &rtcCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(rtcCfg_default.timeDateRegSel,
                      rtcCfg_rd.timeDateRegSel);

    pmic_testResultUpdate_pass(9907,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_rtcSetConfiguration : Select RTC Time and Date Register read from Static Shadowed Registers
 */
static void test_pmic_rtc_setCfg_timeDateRegSel_staticShadow(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_RtcCfg_t rtcCfg_rd = {PMIC_RTC_CFG_TIME_DATE_REG_SEL_VALID_SHIFT,};
    Pmic_RtcCfg_t rtcCfg_default = {PMIC_RTC_CFG_TIME_DATE_REG_SEL_VALID_SHIFT,};
    Pmic_CoreHandle_t  *pHandle     = NULL;
    pHandle                         = pPmicCoreHandle;
    Pmic_RtcCfg_t rtcCfg =
    {
        PMIC_RTC_CFG_TIME_DATE_REG_SEL_VALID_SHIFT,
        PMIC_RTC_CRYSTAL_OSC_ENABLE,
        PMIC_RTC_32K_COUNTER_COMP_VAL_SET,
        PMIC_RTC_ROUND_TIME_SET,
        PMIC_RTC_STATIC_SHADOWED_REG_SEL,
        PMIC_RTC_CRYSTAL_OSC_TYPE_9PF
    };

    test_pmic_print_unity_testcase_info(9908,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    pmicStatus = Pmic_rtcGetConfiguration(pHandle, &rtcCfg_default);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_rtcSetConfiguration(pHandle, rtcCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_rtcGetConfiguration(pHandle, &rtcCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(rtcCfg.timeDateRegSel, rtcCfg_rd.timeDateRegSel);

    pmicStatus = Pmic_rtcSetConfiguration(pHandle, rtcCfg_default);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_rtcGetConfiguration(pHandle, &rtcCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(rtcCfg_default.timeDateRegSel,
                      rtcCfg_rd.timeDateRegSel);

    pmic_testResultUpdate_pass(9908,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_rtcSetConfiguration : Select Crystal Oscillator type as 9PF/ 6PF/ 12.5PF
 */
static void test_pmic_rtc_setCfg_crystalOScType(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_RtcCfg_t rtcCfg_rd = {PMIC_RTC_CFG_CRYSTAL_OSC_TYPE_VALID_SHIFT,};
    Pmic_RtcCfg_t rtcCfg_default = {PMIC_RTC_CFG_CRYSTAL_OSC_TYPE_VALID_SHIFT,};
    Pmic_CoreHandle_t  *pHandle     = NULL;
    pHandle                         = pPmicCoreHandle;
    Pmic_RtcCfg_t rtcCfg =
    {
        PMIC_RTC_CFG_CRYSTAL_OSC_TYPE_VALID_SHIFT,
        PMIC_RTC_CRYSTAL_OSC_DISABLE,
        PMIC_RTC_32K_COUNTER_COMP_VAL_SET,
        PMIC_RTC_ROUND_TIME_SET,
        PMIC_RTC_STATIC_SHADOWED_REG_SEL,
        PMIC_RTC_CRYSTAL_OSC_TYPE_6PF
    };

    rtcCfg.validParams = rtcCfg.validParams |
                         PMIC_RTC_CFG_CRYSTAL_OSC_EN_VALID_SHIFT;
    rtcCfg_default.validParams = rtcCfg.validParams ;
    rtcCfg_rd.validParams = rtcCfg.validParams ;

    test_pmic_print_unity_testcase_info(9909,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    pmicStatus = Pmic_rtcGetConfiguration(pHandle, &rtcCfg_default);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_rtcSetConfiguration(pHandle, rtcCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_rtcGetConfiguration(pHandle, &rtcCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(rtcCfg.crystalOScType, rtcCfg_rd.crystalOScType);

    rtcCfg.crystalOScType = PMIC_RTC_CRYSTAL_OSC_TYPE_12_5PF;
    pmicStatus = Pmic_rtcSetConfiguration(pHandle, rtcCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_rtcGetConfiguration(pHandle, &rtcCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(rtcCfg.crystalOScType, rtcCfg_rd.crystalOScType);

    rtcCfg.crystalOScType = PMIC_RTC_CRYSTAL_OSC_TYPE_9PF;
    pmicStatus = Pmic_rtcSetConfiguration(pHandle, rtcCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_rtcGetConfiguration(pHandle, &rtcCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(rtcCfg.crystalOScType, rtcCfg_rd.crystalOScType);

    pmicStatus = Pmic_rtcSetConfiguration(pHandle, rtcCfg_default);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_rtcGetConfiguration(pHandle, &rtcCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(rtcCfg_default.timeDateRegSel,
                      rtcCfg_rd.timeDateRegSel);

    pmic_testResultUpdate_pass(9909,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_rtcSetConfiguration : Parameter validation for handle
 */
static void test_pmic_rtc_setCfgPrmValTest_handle(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_CoreHandle_t  *pHandle     = NULL;
    Pmic_RtcCfg_t rtcCfg =
    {
        PMIC_RTC_CFG_CRYSTAL_OSC_EN_VALID_SHIFT,
        PMIC_RTC_CRYSTAL_OSC_ENABLE,
        PMIC_RTC_32K_COUNTER_COMP_VAL_SET,
        PMIC_RTC_ROUND_TIME_SET,
        PMIC_RTC_STATIC_SHADOWED_REG_SEL,
        PMIC_RTC_CRYSTAL_OSC_TYPE_6PF
    };

    test_pmic_print_unity_testcase_info(9910,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    pmicStatus = Pmic_rtcSetConfiguration(pHandle, rtcCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, pmicStatus);

    pmic_testResultUpdate_pass(9910,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_rtcSetConfiguration : Parameter validation for RTC Config Param - setRtcTimeRound30s
 */
static void test_pmic_rtc_setCfgPrmValTest_setRtcTimeRound30s(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_CoreHandle_t  *pHandle     = NULL;
    pHandle                         = pPmicCoreHandle;
    Pmic_RtcCfg_t rtcCfg =
    {
        PMIC_RTC_CFG_RTC_TIME_ROUND_30S_SET_VALID_SHIFT,
        PMIC_RTC_CRYSTAL_OSC_ENABLE,
        PMIC_RTC_32K_COUNTER_COMP_VAL_SET,
        0U,
        PMIC_RTC_STATIC_SHADOWED_REG_SEL,
        PMIC_RTC_CRYSTAL_OSC_TYPE_6PF
    };

    test_pmic_print_unity_testcase_info(9911,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    pmicStatus = Pmic_rtcSetConfiguration(pHandle, rtcCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(9911,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_rtcSetConfiguration : Parameter validation for RTC Config Param - timeDateRegSel
 */
static void test_pmic_rtc_setCfgPrmValTest_timeDateRegSel(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_CoreHandle_t  *pHandle     = NULL;
    pHandle                         = pPmicCoreHandle;
    Pmic_RtcCfg_t rtcCfg =
    {
        PMIC_RTC_CFG_TIME_DATE_REG_SEL_VALID_SHIFT,
        PMIC_RTC_CRYSTAL_OSC_ENABLE,
        PMIC_RTC_32K_COUNTER_COMP_VAL_SET,
        PMIC_RTC_ROUND_TIME_SET,
        2U,
        PMIC_RTC_CRYSTAL_OSC_TYPE_6PF
    };

    test_pmic_print_unity_testcase_info(9912,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    pmicStatus = Pmic_rtcSetConfiguration(pHandle, rtcCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(9912,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_rtcSetConfiguration : Parameter validation for RTC Config Param - crystalOScType
 */
static void test_pmic_rtc_setCfgPrmValTest_crystalOScType(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_CoreHandle_t  *pHandle     = NULL;
    pHandle                         = pPmicCoreHandle;
    Pmic_RtcCfg_t rtcCfg =
    {
        PMIC_RTC_CFG_CRYSTAL_OSC_TYPE_VALID_SHIFT,
        PMIC_RTC_CRYSTAL_OSC_ENABLE,
        PMIC_RTC_32K_COUNTER_COMP_VAL_SET,
        PMIC_RTC_ROUND_TIME_SET,
        PMIC_RTC_STATIC_SHADOWED_REG_SEL,
        3U
    };

    test_pmic_print_unity_testcase_info(9913,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    pmicStatus = Pmic_rtcSetConfiguration(pHandle, rtcCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(9913,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_rtcSetConfiguration : Parameter validation for RTC Config Param - set32KCounterCompVal
 */
static void test_pmic_rtc_setCfgPrmValTest_set32KCounterCompVal(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_CoreHandle_t  *pHandle     = NULL;
    pHandle                         = pPmicCoreHandle;
    Pmic_RtcCfg_t rtcCfg =
    {
        PMIC_RTC_CFG_32K_COUNTER_COMP_VAL_SET_VALID_SHIFT,
        PMIC_RTC_CRYSTAL_OSC_ENABLE,
        0U,
        PMIC_RTC_ROUND_TIME_SET,
        PMIC_RTC_STATIC_SHADOWED_REG_SEL,
        PMIC_RTC_CRYSTAL_OSC_TYPE_6PF
    };

    test_pmic_print_unity_testcase_info(9914,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    pmicStatus = Pmic_rtcSetConfiguration(pHandle, rtcCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(9914,
                               pmic_rtc_tests,
                               PMIC_RTC_NUM_OF_TESTCASES);
}

#if defined(UNITY_INCLUDE_CONFIG_V2_H) && \
    (defined(SOC_J721E) || defined(SOC_J7200))

/*!
 * \brief   Run RTC unity test cases
 */
static void test_pmic_run_testcases(void)
{
    pmic_log("\n\n%s(): %d: Begin Unity Test Cases...\n", __func__, __LINE__);
    UNITY_BEGIN();

    pmic_testResult_init(pmic_rtc_tests, PMIC_RTC_NUM_OF_TESTCASES);

    RUN_TEST(test_pmic_rtc_testSetAlarm);
    RUN_TEST(test_pmic_rtc_setAlarmInfoPrmValTest_handle);
    RUN_TEST(test_pmic_rtc_setAlarmInfoPrmValTest_seconds);
    RUN_TEST(test_pmic_rtc_setAlarmInfoPrmValTest_minutes);
    RUN_TEST(test_pmic_rtc_setAlarmInfoPrmValTest_timeMode);
    RUN_TEST(test_pmic_rtc_setAlarmInfoPrmValTest_meridianMode);
    RUN_TEST(test_pmic_rtc_setAlarmInfoPrmValTest_hour12);
    RUN_TEST(test_pmic_rtc_setAlarmInfoPrmValTest_hour24);
    RUN_TEST(test_pmic_rtc_setAlarmInfoPrmValTest_hour);
    RUN_TEST(test_pmic_rtc_setAlarmInfoPrmValTest_month);
    RUN_TEST(test_pmic_rtc_setAlarmInfoPrmValTest_day);
    RUN_TEST(test_pmic_rtc_setAlarmInfoPrmValTest_year);
    RUN_TEST(test_pmic_rtc_setAlarmInfoPrmValTest_month_range);
    RUN_TEST(test_pmic_rtc_setAlarmInfoPrmValTest_day_month);
    RUN_TEST(test_pmic_rtc_setAlarmInfoPrmValTest_feb_leapyear);
    RUN_TEST(test_pmic_rtc_setAlarmInfoPrmValTest_feb_nonleapyear);
    RUN_TEST(test_pmic_rtc_setAlarmInfoPrmValTest_day_month31);
    RUN_TEST(test_pmic_rtc_testGetAlarm);
    RUN_TEST(test_pmic_rtc_getAlarmInfoPrmValTest_handle);
    RUN_TEST(test_pmic_rtc_getAlarmInfoPrmValTest_timeCfg);
    RUN_TEST(test_pmic_rtc_getAlarmInfoPrmValTest_dateCfg);
    RUN_TEST(test_pmic_rtc_testSetTimer);
    RUN_TEST(test_pmic_rtc_setTimerPeriodPrmValTest_handle);
    RUN_TEST(test_pmic_rtc_setTimerPeriodPrmValTest_timerPeriod);
    RUN_TEST(test_pmic_rtc_testGetTimer);
    RUN_TEST(test_pmic_rtc_getTimerPeriodPrmValTest_handle);
    RUN_TEST(test_pmic_rtc_getTimerPeriodPrmValTest_timerPeriod);
    RUN_TEST(test_pmic_rtc_testDisable);
    RUN_TEST(test_pmic_rtc_disablePrmValTest_handle);
    RUN_TEST(test_pmic_rtc_testEnable);
    RUN_TEST(test_pmic_rtc_enablePrmValTest_handle);
    RUN_TEST(test_pmic_rtc_testSetTime);
    RUN_TEST(test_pmic_rtc_setTimePrmValTest_handle);
    RUN_TEST(test_pmic_rtc_setTimePrmValTest_seconds);
    RUN_TEST(test_pmic_rtc_setTimePrmValTest_minutes);
    RUN_TEST(test_pmic_rtc_setTimePrmValTest_timeMode);
    RUN_TEST(test_pmic_rtc_setTimePrmValTest_meridianMode);
    RUN_TEST(test_pmic_rtc_setTimePrmValTest_hour12);
    RUN_TEST(test_pmic_rtc_setTimePrmValTest_hour24);
    RUN_TEST(test_pmic_rtc_setTimePrmValTest_hour);
    RUN_TEST(test_pmic_rtc_setTimePrmValTest_year);
    RUN_TEST(test_pmic_rtc_setTimePrmValTest_month);
    RUN_TEST(test_pmic_rtc_setTimePrmValTest_day);
    RUN_TEST(test_pmic_rtc_setTimePrmValTest_pvmonth);
    RUN_TEST(test_pmic_rtc_setTimePrmValTest_month_range);
    RUN_TEST(test_pmic_rtc_setTimePrmValTest_day_month);
    RUN_TEST(test_pmic_rtc_setTimePrmValTest_feb_leapyear);
    RUN_TEST(test_pmic_rtc_setTimePrmValTest_feb_nonleapyear);
    RUN_TEST(test_pmic_rtc_setTimePrmValTest_day_month31);
    RUN_TEST(test_pmic_rtc_testGetTime);
    RUN_TEST(test_pmic_rtc_getTimePrmValTest_handle);
    RUN_TEST(test_pmic_rtc_getTimePrmValTest_timeCfg);
    RUN_TEST(test_pmic_rtc_getTimePrmValTest_dateCfg);
    RUN_TEST(test_pmic_rtc_testSetFreqComp);
    RUN_TEST(test_pmic_rtc_setFreqCompPrmValTest_handle);
    RUN_TEST(test_pmic_rtc_testGetFreqComp);
    RUN_TEST(test_pmic_rtc_getFreqCompPrmValTest_handle);
    RUN_TEST(test_pmic_rtc_getFreqCompPrmValTest_compensation);
    RUN_TEST(test_pmic_rtc_testTimerIntr);
    RUN_TEST(test_pmic_rtc_testAlarmIntr);
    RUN_TEST(test_pmic_rtc_enableTimerInterrupt_PrmValTest_handle);
    RUN_TEST(test_pmic_rtc_enableAlarmInterrupt_PrmValTest_handle);
    RUN_TEST(test_pmic_rtc_testGetStatus_running);
    RUN_TEST(test_pmic_rtc_testGetStatus_frozen);
    RUN_TEST(test_pmic_rtc_getRstStatus_PrmValTest_handle);
    RUN_TEST(test_pmic_rtc_getRstStatus_PrmValTest_rtcStatus);
    RUN_TEST(test_pmic_rtc_getRstStatus_PrmValTest_validParams);
    RUN_TEST(test_pmic_rtc_testTimerAsyncIntr);
    RUN_TEST(test_pmic_rtc_testAlarmAsyncIntr);
    RUN_TEST(test_pmic_rtc_getStatus_PrmValTest_handle);
    RUN_TEST(test_pmic_rtc_getStatus_PrmValTest_pRtcstatus);
    RUN_TEST(test_pmic_rtc_getCfgPrmValTest_handle);
    RUN_TEST(test_pmic_rtc_getCfgPrmValTest_pRtcCfg);
    RUN_TEST(test_pmic_rtc_clrRstStatus_PrmValTest_handle);
    RUN_TEST(test_pmic_rtc_clrRstStatus_PrmValTest_rtcRstStatType);
    RUN_TEST(test_pmic_rtc_setCfg_crystalOScEn);
    RUN_TEST(test_pmic_rtc_setCfg_rtcTimeRound30s);
    RUN_TEST(test_pmic_rtc_setCfg_timeDateRegSel_dynamic);
    RUN_TEST(test_pmic_rtc_setCfg_timeDateRegSel_staticShadow);
    RUN_TEST(test_pmic_rtc_setCfg_crystalOScType);
    RUN_TEST(test_pmic_rtc_setCfgPrmValTest_handle);
    RUN_TEST(test_pmic_rtc_setCfgPrmValTest_setRtcTimeRound30s);
    RUN_TEST(test_pmic_rtc_setCfgPrmValTest_timeDateRegSel);
    RUN_TEST(test_pmic_rtc_setCfgPrmValTest_crystalOScType);
    RUN_TEST(test_pmic_rtc_setCfgPrmValTest_set32KCounterCompVal);

    RUN_TEST(test_pmic_rtc_setAlarmInfoPrmValTest_febMonth_day);
    RUN_TEST(test_pmic_rtc_setAlarmInfoPrmValTest_day_min);
    RUN_TEST(test_pmic_rtc_setAlarmInfoPrmValTest_day_max);
    RUN_TEST(test_pmic_rtc_setAlarmInfoPrmValTest_month_min);
    RUN_TEST(test_pmic_rtc_setAlarmInfoPrmValTest_month_max);
    RUN_TEST(test_pmic_rtc_setAlarmInfoPrmValTest_year_min);
    RUN_TEST(test_pmic_rtc_setAlarmInfoPrmValTest_weekday_min);
    RUN_TEST(test_pmic_rtc_setAlarmInfoPrmValTest_meridianMode_12HrMode);
    RUN_TEST(test_pmic_rtc_setAlarmInfoPrmValTest_weekday);
    RUN_TEST(test_pmic_rtc_setAlarmInfoPrmValTest_dateTimeCfg);
    RUN_TEST(test_pmic_rtc_setTimeDateInfoPrmValTest_dateTimeCfg);
    RUN_TEST(test_pmic_rtc_getAlarmInfoPrmValTest_pDatePtimeCfg);
    RUN_TEST(test_pmic_rtc_getTimeDateInfoPrmValTest_pDatePtimeCfg);
    RUN_TEST(test_pmic_rtc_getRstStatus_powerUpStatus);
    RUN_TEST(test_pmic_rtc_setTimeDateInfoPrmValTest_day_min_leapyear);
    RUN_TEST(test_pmic_rtc_getTimeDateInfo_timeDateRegSel_staticShadow);
    RUN_TEST(test_pmic_rtc_setConfiguration_rtcIsStarted);
    RUN_TEST(test_pmic_rtc_coverageGaps);

    pmic_updateTestResults(pmic_rtc_tests, PMIC_RTC_NUM_OF_TESTCASES);

    UNITY_END();
}

/*!
 * \brief   Run RTC unity test cases for HERA PMIC
 */
void test_pmic_hera_run_testcases(void)
{
    pmic_log("\n\n%s(): %d: Begin Unity Test Cases...\n", __func__, __LINE__);
    UNITY_BEGIN();

    pmic_testResult_init(pmic_rtc_tests, PMIC_RTC_NUM_OF_TESTCASES);

    RUN_TEST(test_Pmic_rtcEnable_hera);
    RUN_TEST(test_Pmic_rtcEnableAlarmIntr_hera);
    RUN_TEST(test_Pmic_rtcEnableTimerIntr_hera);
    RUN_TEST(test_pmic_rtc_testSetFreqComp_hera);
    RUN_TEST(test_Pmic_rtcSetTimeDateInfo_hera);
    RUN_TEST(test_Pmic_rtcGetAlarmInfo_hera);
    RUN_TEST(test_pmic_rtc_testSetAlarm_hera);
    RUN_TEST(test_pmic_rtc_testSetTimer_hera);
    RUN_TEST(test_pmic_rtc_testGetTimer_hera);
    RUN_TEST(test_pmic_rtc_testGetRstStatus_running_hera);
    RUN_TEST(test_pmic_rtc_testGetTime_hera);
    RUN_TEST(test_pmic_rtc_testGetFreqComp_hera);

    pmic_updateTestResults(pmic_rtc_tests, PMIC_RTC_NUM_OF_TESTCASES);

    UNITY_END();
}

/*!
 * \brief   RTC Unity Test App wrapper Function for LEO PMIC-A
 */
static int32_t test_pmic_leo_pmicA_rtc_testApp(void)
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
 * \brief   RTC Unity Test App wrapper Function for LEO PMIC-A
 */
static int32_t test_pmic_leo_pmicA_spiStub_rtc_testApp(void)
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

/*!
 * \brief   RTC Unity Test App wrapper Function for HERA PMIC
 */
static int32_t test_pmic_hera_rtc_testApp(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_CoreCfg_t pmicConfigData = {0U};

    /* Fill parameters to pmicConfigData */
    pmicConfigData.pmicDeviceType     = PMIC_DEV_HERA_LP8764X;
    pmicConfigData.validParams        |= PMIC_CFG_DEVICE_TYPE_VALID_SHIFT;

    pmicConfigData.commMode           = PMIC_INTF_SINGLE_I2C;
    pmicConfigData.validParams        |= PMIC_CFG_COMM_MODE_VALID_SHIFT;

    pmicConfigData.i2c1Speed            = PMIC_I2C_STANDARD_MODE;
    pmicConfigData.validParams         |= PMIC_CFG_I2C1_SPEED_VALID_SHIFT;

    pmicConfigData.slaveAddr          = J7VCL_HERA_PMIC_SLAVE_ADDR;
    pmicConfigData.validParams        |= PMIC_CFG_SLAVEADDR_VALID_SHIFT;

    pmicConfigData.qaSlaveAddr        = J7VCL_HERA_PMIC_WDG_SLAVE_ADDR;
    pmicConfigData.validParams        |= PMIC_CFG_QASLAVEADDR_VALID_SHIFT;

    pmicConfigData.nvmSlaveAddr        = J7VCL_HERA_PMIC_PAGE1_SLAVE_ADDR;
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
 * \brief   RTC Unity Test App wrapper Function for LEO PMIC-B
 */
static int32_t test_pmic_leo_pmicB_rtc_testApp(void)
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

static int32_t setup_pmic_interrupt(uint32_t board, bool enableCRC)
{
    int32_t status = PMIC_ST_SUCCESS;

    if(J721E_BOARD == board)
    {
        if(enableCRC == true)
        {
            if(PMIC_STATUS_CRC_INIT_VAL == gCrcTestFlag_J721E)
            {
                gCrcTestFlag_J721E = PMIC_CFG_TO_ENABLE_CRC;
            }
        }
        else
        {
            gCrcTestFlag_J721E = PMIC_STATUS_CRC_INIT_VAL;
        }

        pmic_device_info = J721E_LEO_PMICA_DEVICE;
        status = test_pmic_leo_pmicA_rtc_testApp();
        /* Deinit pmic handle */
        if((pPmicCoreHandle != NULL) && (PMIC_ST_SUCCESS == status))
        {
            test_pmic_appDeInit(pPmicCoreHandle);
        }

        if(PMIC_ST_SUCCESS == status)
        {
            pmic_device_info = J721E_LEO_PMICB_DEVICE;
            status = test_pmic_leo_pmicB_rtc_testApp();
            /* Deinit pmic handle */
            if((pPmicCoreHandle != NULL) && (PMIC_ST_SUCCESS == status))
            {
                test_pmic_appDeInit(pPmicCoreHandle);
            }
        }
    }
    else if(J7VCL_BOARD == board)
    {
        if(enableCRC == true)
        {
            if(PMIC_STATUS_CRC_INIT_VAL == gCrcTestFlag_J7VCL)
            {
                gCrcTestFlag_J7VCL = PMIC_CFG_TO_ENABLE_CRC;
            }
        }
        else
        {
            gCrcTestFlag_J7VCL = PMIC_STATUS_CRC_INIT_VAL;
        }

        pmic_device_info = J7VCL_LEO_PMICA_DEVICE;
        status = test_pmic_leo_pmicA_rtc_testApp();
        /* Deinit pmic handle */
        if((pPmicCoreHandle != NULL) && (PMIC_ST_SUCCESS == status))
        {
            test_pmic_appDeInit(pPmicCoreHandle);
        }

        if(PMIC_ST_SUCCESS == status)
        {
            pmic_device_info = J7VCL_HERA_PMICB_DEVICE;
            status = test_pmic_hera_rtc_testApp();
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

/*!
 * \brief   PMIC Application Callback Function
 */
void AppPmicCallbackFxn(void)
{
    int32_t status           = PMIC_ST_SUCCESS;
    Pmic_IrqStatus_t errStat = {0U};
    uint8_t irqNum           = 0U;

    status = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, false);

    if(PMIC_ST_SUCCESS == status)
    {
        status = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                         &errStat,
                                         &irqNum);
        if(PMIC_ST_SUCCESS == status)
        {
            switch(irqNum)
            {
                case PMIC_TPS6594X_RTC_TIMER_INT:
                    pmic_intr_triggered = 1U;
                    /* clear the interrupt */
                    status = Pmic_irqClrErrStatus(pPmicCoreHandle,
                                              PMIC_TPS6594X_RTC_TIMER_INT);
                   break;
                case PMIC_TPS6594X_RTC_ALARM_INT:
                    pmic_intr_triggered = 2U;
                    /* clear the interrupt */
                    status = Pmic_irqClrErrStatus(pPmicCoreHandle,
                                              PMIC_TPS6594X_RTC_ALARM_INT);
                   break;
                default:
                   break;
            }
        }
    }
}

static const char pmicTestMenu[] =
{
    " \r\n ================================================================="
    " \r\n Test Menu:"
    " \r\n ================================================================="
    " \r\n 0: Automatic run for all board specific RTC options"
    " \r\n 1: Manual run for RTC options"
    " \r\n 2: quit"
    " \r\n"
    " \r\n Enter option: "
};

volatile static const char pmicTestAppMenu[] =
{
    " \r\n ================================================================="
    " \r\n Menu Options:"
    " \r\n ================================================================="
    " \r\n 0: Pmic Leo device(PMIC A on J721E EVM Using I2C Interface)"
    " \r\n 1: Pmic Leo device(PMIC A on J721E EVM Using SPI Stub Functions)"
    " \r\n 2: Pmic Leo device(PMIC B on J721E EVM)"
    " \r\n 3: Pmic Leo device(PMIC A on J7VCL EVM Using I2C Interface)"
    " \r\n 4: Pmic Leo device(PMIC A on J7VCL EVM Using SPI Stub Functions)"
    " \r\n 5: Pmic HERA device(PMIC B on J7VCL EVM)"
    " \r\n 6: Pmic Leo device(PMIC A on J721E EVM RTC Manual Testcase)"
    " \r\n 7: Pmic Leo device(PMIC A on J7VCL EVM RTC Manual Testcase)"
    " \r\n 8: Pmic Leo device(PMIC A on J721E EVM Manual Testcase with CRC Disable on PG2.0 Silicon revision)"
    " \r\n 9: Back to Test Menu"
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
    pmic_log(" \r\n 0: Pmic Leo device(PMIC A on %s EVM for RTC WKUP using Timer Interrupt from LP Standby State)", board_name);
    pmic_log(" \r\n 1: Pmic Leo device(PMIC A on %s EVM for RTC WKUP using Timer Interrupt from Standby State)", board_name);
    pmic_log(" \r\n 2: Pmic Leo device(PMIC A on %s EVM for RTC WKUP using Alarm Interrupt from LP Standby State)", board_name);
    pmic_log(" \r\n 3: Pmic Leo device(PMIC A on %s EVM for RTC WKUP using Alarm Interrupt from Standby State)", board_name);
    pmic_log(" \r\n 4: Pmic Leo device(PMIC A on %s EVM for Get RTC Reset status and then Clear RTC Reset status using powerupStatus)", board_name);
    pmic_log(" \r\n 5: Pmic Leo device(PMIC A on %s EVM for Get RTC Reset status and then Clear RTC Reset status using rtcRstStatus)", board_name);
    pmic_log(" \r\n 6: Pmic Leo device(PMIC A on %s EVM for Set 32K counter with compensation values)", board_name);
    pmic_log(" \r\n 7: Back to Main Menu");
    pmic_log(" \r\n");
    pmic_log(" \r\n Enter option: ");
}

/*!
 * \brief   Run RTC manual test cases
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

        switch(menuOption)
        {
            case 0U:
                RUN_TEST(test_pmic_rtc_testWakeup_TimerIntr_lpStandbyState);
               break;
            case 1U:
                RUN_TEST(test_pmic_rtc_testWakeup_TimerIntr_standbyState);
               break;
            case 2U:
                RUN_TEST(test_pmic_rtc_testWakeup_AlarmIntr_lpStandbyState);
               break;
            case 3U:
                RUN_TEST(test_pmic_rtc_testWakeup_AlarmIntr_standbyState);
               break;
            case 4U:
                RUN_TEST(test_pmic_rtc_clrRstStatus_powerupStatus);
               break;
            case 5U:
                RUN_TEST(test_pmic_rtc_clrRstStatus_rtcRstStatus);
               break;
            case 6U:
                RUN_TEST(test_pmic_rtc_setCfg_set32KCounterCompVal);
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

static void test_pmic_rtc_testapp_run_options(int8_t option)
{
    int8_t num = -1;
    int8_t idx = 0;
#if defined(SOC_J721E)
    int8_t automatic_options[] = {0, 1};
    int8_t  userInput = 0;
#elif defined(SOC_J7200)
    int8_t automatic_options[] = {3, 4, 5};
#endif

    while(1U)
    {
        if(idx >= (sizeof(automatic_options)/sizeof(automatic_options[0])))
        {
            pmic_printTestResult(pmic_rtc_tests, PMIC_RTC_NUM_OF_TESTCASES);
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
                num = 9;
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
                if(PMIC_ST_SUCCESS == setup_pmic_interrupt(J721E_BOARD, true))
                {
                    pmic_device_info = J721E_LEO_PMICA_DEVICE;

                    /* RTC Unity Test App wrapper Function for LEO PMIC-A */
                    if(PMIC_ST_SUCCESS == test_pmic_leo_pmicA_rtc_testApp())
                    {
                       if(PMIC_SILICON_REV_ID_PG_2_0 ==
                          pPmicCoreHandle->pmicDevSiliconRev)
                       {
                            test_pmic_rtc_setCfg_xtalOScEnType(pPmicCoreHandle);
                       }
                        /* Run rtc test cases for Leo PMIC-A */
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
                if(PMIC_ST_SUCCESS == setup_pmic_interrupt(J721E_BOARD, true))
                {
                    pmic_device_info = J721E_LEO_PMICA_DEVICE;

                    /* RTC Unity Test App wrapper Function for LEO PMIC-A using
                     * SPI stub functions */
                    if(PMIC_ST_SUCCESS ==
                           test_pmic_leo_pmicA_spiStub_rtc_testApp())
                    {
                       if(PMIC_SILICON_REV_ID_PG_2_0 ==
                          pPmicCoreHandle->pmicDevSiliconRev)
                       {
                            test_pmic_rtc_setCfg_xtalOScEnType(pPmicCoreHandle);
                       }
                        /* Run rtc test cases for Leo PMIC-A */
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
               /* RTC Unity Test App wrapper Function for LEO PMIC-B */
                pmic_log("RTC on LEO PMIC-B is not supported due to HW limitation\n");
#else
                pmic_log("\nInvalid Board!!!\n");
#endif
                break;
            case 3U:
#if defined(SOC_J7200)
                if(PMIC_ST_SUCCESS == setup_pmic_interrupt(J7VCL_BOARD, true))
                {
                    pmic_device_info = J7VCL_LEO_PMICA_DEVICE;

                    /* RTC Unity Test App wrapper Function for LEO PMIC-A */
                    if(PMIC_ST_SUCCESS == test_pmic_leo_pmicA_rtc_testApp())
                    {
                        /* Run rtc test cases for Leo PMIC-A */
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
                if(PMIC_ST_SUCCESS == setup_pmic_interrupt(J7VCL_BOARD, true))
                {
                    pmic_device_info = J7VCL_LEO_PMICA_DEVICE;
                    /* RTC Unity Test App wrapper Function for LEO PMIC-A
                     * using SPI stub functions */
                     if(PMIC_ST_SUCCESS ==
                            test_pmic_leo_pmicA_spiStub_rtc_testApp())
                    {
                        /* Run rtc test cases for Leo PMIC-A */
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
                if(PMIC_ST_SUCCESS == setup_pmic_interrupt(J7VCL_BOARD, true))
                {
                    pmic_device_info = J7VCL_HERA_PMICB_DEVICE;

                    /* RTC Unity Test App wrapper Function for HERA */
                    if(PMIC_ST_SUCCESS == test_pmic_hera_rtc_testApp())
                    {
                        /* Run rtc test cases for Leo PMIC-A */
                        test_pmic_hera_run_testcases();
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
                if(PMIC_ST_SUCCESS == setup_pmic_interrupt(J721E_BOARD, true))
                {
                    pmic_device_info = J721E_LEO_PMICA_DEVICE;

                    /* RTC Manual Test App wrapper Function for LEO PMIC-A */
                    if(PMIC_ST_SUCCESS == test_pmic_leo_pmicA_rtc_testApp())
                    {
                       if(PMIC_SILICON_REV_ID_PG_2_0 ==
                          pPmicCoreHandle->pmicDevSiliconRev)
                       {
                            test_pmic_rtc_setCfg_xtalOScEnType(pPmicCoreHandle);
                       }

                        /* Run Rtc manual test cases */
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
                return;
           case 7U:
#if defined(SOC_J7200)
                if(PMIC_ST_SUCCESS == setup_pmic_interrupt(J7VCL_BOARD, true))
                {
                    pmic_device_info = J7VCL_LEO_PMICA_DEVICE;

                    /* RTC Manual Test App wrapper Function for LEO PMIC-A */
                    if(PMIC_ST_SUCCESS == test_pmic_leo_pmicA_rtc_testApp())
                    {
                        /* Run Rtc manual test cases */
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
                return;
            case 8U:
#if defined(SOC_J721E)
                pmic_log("\r\n Run the test only on Leo PMIC-A PG2.0 after Board Power cycle !!!\n");
                pmic_log("\r\n Enter 1 to continue");
                UART_scanFmt("%d", &userInput);

                if(PMIC_ST_SUCCESS == setup_pmic_interrupt(J721E_BOARD, false))
                {
                    pmic_device_info = J721E_LEO_PMICA_DEVICE;

                    /* RTC Unity Test App wrapper Function for LEO PMIC-A */
                    if(PMIC_ST_SUCCESS == test_pmic_leo_pmicA_rtc_testApp())
                    {
                       if(PMIC_SILICON_REV_ID_PG_2_0 ==
                          pPmicCoreHandle->pmicDevSiliconRev)
                       {
                            test_pmic_rtc_setCfg_xtalOScEnType(pPmicCoreHandle);
                       }
                        /* Run rtc test cases for Leo PMIC-A */
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
            case 9U:
                pmic_log(" \r\n Back to Test Menu options\n");
                return;
            default:
                pmic_log(" \r\n Invalid option... Try Again!!!\n");
                break;
        }
    }
}

/*!
 * \brief   Function to register RTC Unity Test App wrapper to Unity framework
 */
static void test_pmic_rtc_testapp_runner(void)
{
    /* @description : Test runner for RTC Test App
     *
     * @requirements: 5855, 5813, 5806
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
                test_pmic_rtc_testapp_run_options(PMIC_UT_AUTOMATE_OPTION);
               break;
            case PMIC_UT_MANUAL_OPTION:
                test_pmic_rtc_testapp_run_options(PMIC_UT_MANUAL_OPTION);
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
 * \brief   TI RTOS specific RTC TEST APP main Function
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code.
 *          For valid values see \ref Pmic_ErrorCodes
 */
int main()
{
    Board_initUART();

    /*
     * Initialze and Register ISR handler to J7 Interrupts
     */
    App_initGPIO(AppPmicCallbackFxn);

    pmic_print_banner("PMIC RTC Unity Test Application");

#if defined(UNITY_INCLUDE_CONFIG_V2_H) && \
    (defined(SOC_J721E)             || \
     defined(SOC_J7200))
    test_pmic_rtc_testapp_runner();
#endif
}
