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
#include <pmic_fsm.h>

/* Pointer to Pmic Core Handle */
Pmic_CoreHandle_t *pPmicCoreHandle = NULL;

extern volatile uint32_t pmic_intr_triggered;
extern Pmic_CoreHandle_t *pPmicCoreHandleISR;
extern volatile uint8_t irqNumToClr;

/*!
 * \brief   PMIC RTC Test Cases
 */
static Pmic_Ut_Tests_t pmic_rtc_tests[] =
{
    {
        5990,
        "SetRtcAlarmIntr : Set RTC Alarm interrupt"
    },
    {
        5991,
        "SetRtcAlarmIntr : Parameter validation for 'handle'"
    },
    {
        6090,
        "SetRtcAlarmIntr : Parameter validation for 'seconds'"
    },
    {
        6091,
        "SetRtcAlarmIntr : Parameter validation for 'minutes'"
    },
    {
        6092,
        "SetRtcAlarmIntr : Parameter validation for 'timeMode'"
    },
    {
        6093,
        "SetRtcAlarmIntr : Parameter validation for 'meridianMode'"
    },
    {
        6094,
        "SetRtcAlarmIntr : Parameter validation for 'hour' when 'timeMode' = 1"
    },
    {
        6095,
        "SetRtcAlarmIntr : Parameter validation for 'hour' when 'timeMode' = 0"
    },
    {
        6096,
        "SetRtcAlarmIntr : Negative test for 'hour ' = 0, when 'timeMode' = 1"
    },
    {
        6099,
        "SetRtcAlarmIntr : Negative test for 'month ' = 0"
    },
    {
        6100,
        "SetRtcAlarmIntr : Negative test for 'day' = 0"
    },
    {
        6101,
        "SetRtcAlarmIntr : Parameter range validation for 'year'"
    },
    {
        6102,
        "SetRtcAlarmIntr : Parameter range validation for 'month'"
    },
    {
        6103,
        "SetRtcAlarmIntr : Parameter range validation for 'day' for months with 30 days"
    },
    {
        6104,
        "SetRtcAlarmIntr : Parameter range validation for 'day' for leap year('year' %4 = 0 ) and 'month' = 2 (february)"
    },
    {
        6107,
        "SetRtcAlarmIntr : Parameter range validation for 'day' for Non-leap year('year' %4 != 0 ) and 'month' = 2 (february)"
    },
    {
        6108,
        "SetRtcAlarmIntr : Parameter range validation for 'day', for months with 31 days)"
    },
    {
        6109,
        "GetRtcAlarmIntr : Test Get RTC Alarm interrupt API"
    },
    {
        6110,
        "GetRtcAlarmIntr : Parameter validation for 'handle'"
    },
    {
        6111,
        "GetRtcAlarmIntr : Parameter validation for 'timeCfg'"
    },
    {
        6112,
        "GetRtcAlarmIntr : Parameter validation for 'dataCfg'"
    },
    {
        6113,
        "SetRtcTimerIntr : Test Set RTC Timer interrupt"
    },
    {
        6114,
        "SetRtcTimerIntr : Parameter validation for 'handle'"
    },
    {
        6115,
        "SetRtcTimerIntr : Parameter validation for 'timerPeriod'"
    },
    {
        6116,
        "GetRtcTimer : Test Get RTC Timer interrupt"
    },
    {
        6117,
        "GetRtcTimer : Parameter validation for 'handle'"
    },
    {
        6118,
        "GetRtcTimer : Parameter validation for 'timerPeriod'"
    },
    {
        6119,
        "DisableRtc : Test RTC Disable"
    },
    {
        6120,
        "DisableRtc : Parameter validation for 'handle'"
    },
    {
        6121,
        "EnableRtc : Test RTC Enable"
    },
    {
        6122,
        "EnableRtc : Parameter validation for 'handle'"
    },
    {
        6158,
        "SetRtc : Test RTC Set Time"
    },
    {
        6159,
        "SetRtc : Parameter validation for 'handle'"
    },
    {
        6162,
        "SetRtc : Parameter validation for 'seconds'"
    },
    {
        6163,
        "SetRtc : Parameter validation for 'minutes'"
    },
    {
        6164,
        "SetRtc : Parameter validation for 'timeMode'"
    },
    {
        6165,
        "SetRtc : Parameter validation for 'meridianMode'"
    },
    {
        6166,
        "SetRtc : Parameter validation for 'hour' when 'timeMode' = 1"
    },
    {
        6167,
        "SetRtc : Parameter validation for 'hour' when 'timeMode' = 0"
    },
    {
        6168,
        "SetRtc : Negative test for 'hour ' = 0, when 'timeMode' = 1"
    },
    {
        6169,
        "SetRtc : Negative test for 'month ' = 0"
    },
    {
        6290,
        "SetRtc : Negative test for 'day' = 0"
    },
    {
        7021,
        "SetRtc: Parameter range validation for 'year'"
    },
    {
        6170,
        "SetRtc : Parameter range validation for 'month'"
    },
    {
        6171,
        "SetRtc : Parameter range validation for 'day'"
    },
    {
       6172,
       "SetRtc : Parameter range validation for 'day' for months with 30 days"
    },
    {
        6173,
        "SetRtc : Parameter range validation for 'day' for leap year('year' %4 = 0 ) and 'month' = 2 (february)"
    },
    {
        6174,
        "SetRtc : Parameter range validation for 'day' for Non-leap year('year' %4 != 0 ) and 'month' = 2 (february)"
    },
    {
        6175,
        "SetRtc : Parameter range validation for 'day', for months with 31 days)"
    },
    {
        6176,
        "GetRtc : Test RTC Get Time"
    },
    {
        6177,
        "GetRtc : Parameter validation for 'handle'"
    },
    {
        6178,
        "GetRtc : Parameter validation for 'timeCfg'"
    },
    {
        6179,
        "GetRtc : Parameter validation for 'dataCfg'"
    },
    {
        6180,
        "SetRtcFreqCompen : Test RTC set RTC frequency compensation"
    },
    {
        6181,
        "SetRtcFreqCompen : Parameter validation for 'handle'"
    },
    {
        6182,
        "GetRtcFreqCompen : Test RTC get RTC frequency compensation"
    },
    {
        6183,
        "GetRtcFreqCompen : Parameter validation for 'handle'"
    },
    {
        6287,
        "GetRtcFreqCompen : Parameter validation for 'compensation'"
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
        "Pmic_rtcEnableTimerIntr : Parameter validation for 'handle'"
    },
    {
        6269,
        "Pmic_rtcEnableAlarmIntr : Parameter validation for 'handle'"
    },
    {
        6194,
        "Pmic_getRtcStatus : RTC Live Status Validation RTC Current state Running"
    },
    {
        6089,
        "Pmic_getRtcStatus : RTC Live Status Validation RTC Current state Frozen"
    },
    {
        7465,
        "Pmic_getRtcStatus : Parameter validation for 'handle'"
    },
    {
        7466,
        "Pmic_getRtcStatus : Parameter validation for 'RtcStatus'"
    },
    {
        7467,
        "Pmic_getRtcStatus : Parameter validation for 'ValidParams'"
    },
    {
        7358,
        "Pmic_fsmDeviceOffRequestCfg/Pmic_fsmEnableI2cTrigger : RTC Wakeup using Timer Interrupt'"
    },
    {
        7359,
        "Pmic_fsmDeviceOffRequestCfg/Pmic_fsmEnableI2cTrigger :  RTC Wakeup using Alarm Interrupt'"
    },
    {
        1,
        "test_rtc_timer_irq : Test rtc timer asynchronous interrupt"
    },
    {
        2,
        "test_rtc_alarm_irq: Test rtc alarm asynchronous interrupt"
    }
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
}

/*!
 * \brief   Parameter validation for 'handle'
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
}

/*!
 * \brief   Parameter validation for 'seconds'
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
}

/*!
 * \brief   Parameter validation for 'minutes'
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
}

/*!
 * \brief   Parameter validation for 'timeMode'
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
}

/*!
 * \brief   Parameter validation for 'meridianMode'
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
}

/*!
 * \brief   Parameter validation for 'hour' when 'timeMode' = 1
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
}

/*!
 * \brief   Parameter validation for 'hour' when 'timeMode' = 0
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
}

/*!
 * \brief   Negative test for 'hour ' = 0U, when 'timeMode' = 1
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
}

/*!
 * \brief   Negative test for 'month' = 0
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
}

/*!
 * \brief   Negative test for 'day' = 0
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
}

/*!
 * \brief   Parameter range validation for 'year'
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
}

/*!
 * \brief   Parameter range validation for 'month'
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
}

/*!
 * \brief   Parameter range validation for 'day'for months with 30 days
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
}

/*!
 * \brief   Parameter range validation for 'day' for leap
 *          year('year' %4 = 0 ) and 'month' = 2 (february)
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
}

/*!
 * \brief   Parameter range validation for 'day' for Non-leap
 *          year('year' %4 != 0 ) and 'month' = 2 (february)
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
}

/*!
 * \brief   Parameter range validation for 'day', for months with 31 days
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
}

/*!
 * \brief   Parameter validation for 'handle'
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
}

/*!
 * \brief   Parameter validation for 'timeCfg'
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
}

/*!
 * \brief   Parameter validation for 'dataCfg'
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
}

/*!
 * \brief   Parameter validation for 'handle'
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
}

/*!
 * \brief   Parameter validation for 'timerPeriod'
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
}

/*!
 * \brief   Parameter validation for 'handle'
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
}

/*!
 * \brief   Parameter validation for 'timerPeriod
 */
static void test_pmic_rtc_getTimerPeriodPrmValTest_timerPeriod(void)
{
    int32_t status    = PMIC_ST_SUCCESS;

    test_pmic_print_unity_testcase_info(6118,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    status = Pmic_rtcGetTimerPeriod(pPmicCoreHandle, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
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
}

/*!
 * \brief   Parameter validation for 'handle'
 */
static void test_pmic_rtc_disablePrmValTest_handle(void)
{
    int32_t status         = PMIC_ST_SUCCESS;

    test_pmic_print_unity_testcase_info(6120,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    status = Pmic_rtcEnable(NULL, PMIC_RTC_STOP);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, status);
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
}

/*!
 * \brief   Parameter validation for 'handle'
 */
static void test_pmic_rtc_enablePrmValTest_handle(void)
{
    int32_t status         = PMIC_ST_SUCCESS;

    test_pmic_print_unity_testcase_info(6122,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    status = Pmic_rtcEnable(NULL, PMIC_RTC_START);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, status);
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
}

/*!
 * \brief   Parameter validation for 'handle'
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
}

/*!
 * \brief   Parameter validation for 'seconds'
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
}

/*!
 * \brief   Parameter validation for 'minutes'
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
}

/*!
 * \brief   Parameter validation for 'timeMode'
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
}

/*!
 * \brief   Parameter validation for 'meridianMode'
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
}

/*!
 * \brief   Parameter validation for 'hour' when 'timeMode' = 1
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
}

/*!
 * \brief   Parameter validation for 'hour' when 'timeMode' = 0
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
}

/*!
 * \brief   Negative test for 'hour ' = 0U, when 'timeMode' = 1
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
}

/*!
 * \brief   Negative test for 'month ' = 0
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
}

/*!
 * \brief   Negative test for 'day' = 0
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
}

/*!
 * \brief   Parameter range validation for 'year'
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
}

/*!
 * \brief   Parameter range validation for 'month'
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
}

/*!
 * \brief   Parameter range validation for 'day'
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
}

/*!
 * \brief   Parameter range validation for 'day'for months with 30 days
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
}

/*!
 * \brief   Parameter range validation for 'day' for leap
 *          year('year' %4 = 0 ) and 'month' = 2 (february)
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
}

/*!
 * \brief   Parameter range validation for 'day' for Non-leap
 *          year('year' %4 != 0 ) and 'month' = 2 (february)
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
}

/*!
 * \brief   Parameter range validation for 'day', for months with 31 days
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
}

/*!
 * \brief   Parameter validation for 'handle'
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
}

/*!
 * \brief   Parameter validation for 'timeCfg'
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
}

/*!
 * \brief   Parameter validation for 'dataCfg'
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
}

/*!
 * \brief   Parameter validation for 'handle'
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
}

/*!
 * \brief   Parameter validation for 'handle'
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
}

/*!
 * \brief   Parameter validation for 'compensation'
 */
static void test_pmic_rtc_getFreqCompPrmValTest_compensation(void)
{
    int32_t status = PMIC_ST_SUCCESS;

    test_pmic_print_unity_testcase_info(6287,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    status = Pmic_rtcGetFreqComp(pPmicCoreHandle, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
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

    pHandle                         = pPmicCoreHandle;

    status = Pmic_rtcSetTimeDateInfo(pHandle, validTimeCfg, validDateCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_rtcGetTimeDateInfo(pHandle, &timeCfg_rd, &dateCfg_rd);
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

    status = Pmic_rtcSetTimeDateInfo(pHandle, validTimeCfg, validDateCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    /* Get the current time value */
    status = Pmic_rtcGetTimeDateInfo(pHandle, &timeCfg_rd, &dateCfg_rd);
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

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
}

/*!
 * \brief   Parameter validation for 'handle'
 */
static void test_pmic_rtc_enableAlarmInterrupt_PrmValTest_handle(void)
{
    int32_t status = PMIC_ST_SUCCESS;

    test_pmic_print_unity_testcase_info(6268,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    status = Pmic_rtcEnableAlarmIntr(NULL, PMIC_RTC_ALARM_INTR_DISABLE);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, status);
}

/*!
 * \brief   Parameter validation for 'handle'
 */
static void test_pmic_rtc_enableTimerInterrupt_PrmValTest_handle(void)
{
    int32_t status = PMIC_ST_SUCCESS;

    test_pmic_print_unity_testcase_info(6269,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    status = Pmic_rtcEnableTimerIntr(NULL, PMIC_RTC_TIMER_INTR_DISABLE);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, status);
}

/*!
 * \brief   RTC Status Validation RTC Current state Running
 */
static void test_pmic_rtc_testGetRtcStatus_running(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_RtcStatus_t pmicRtcStatus = {0};

    test_pmic_print_unity_testcase_info(6194,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    status = Pmic_rtcEnable(pPmicCoreHandle, PMIC_RTC_START);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    pmicRtcStatus.validParams |= PMIC_RTC_CFG_RTC_STATUS_VALID_SHIFT;
    status = Pmic_getRtcStatus(pPmicCoreHandle, &pmicRtcStatus);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    TEST_ASSERT_EQUAL(PMIC_RTC_STATUS_RUNNING, pmicRtcStatus.rtcStatus);
}

/*!
 * \brief   RTC Status Validation RTC Current state Frozen
 */
static void test_pmic_rtc_testGetRtcStatus_frozen(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_RtcStatus_t pmicRtcStatus = {0};

    test_pmic_print_unity_testcase_info(6089,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    status = Pmic_rtcEnable(pPmicCoreHandle, PMIC_RTC_STOP);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    pmicRtcStatus.validParams |= PMIC_RTC_CFG_RTC_STATUS_VALID_SHIFT;
    status = Pmic_getRtcStatus(pPmicCoreHandle, &pmicRtcStatus);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    TEST_ASSERT_EQUAL(PMIC_RTC_STATUS_FROZEN, pmicRtcStatus.rtcStatus);
}

/*!
 * \brief   Parameter validation for 'handle'
 */
static void test_pmic_rtc_getRtcStatus_PrmValTest_handle(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_RtcStatus_t pmicRtcStatus = {0};

    test_pmic_print_unity_testcase_info(7465,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    status = Pmic_rtcEnable(pPmicCoreHandle, PMIC_RTC_STOP);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_getRtcStatus(NULL, &pmicRtcStatus);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, status);
}

/*!
 * \brief   Parameter validation for 'rtcStatus'
 */
static void test_pmic_rtc_getRtcStatus_PrmValTest_rtcStatus(void)
{
    int32_t status = PMIC_ST_SUCCESS;

    test_pmic_print_unity_testcase_info(7466,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    status = Pmic_getRtcStatus(pPmicCoreHandle, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

/*!
 * \brief   Parameter validation for ValidParams
 */
static void test_pmic_rtc_getRtcStatus_PrmValTest_validParams(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_RtcStatus_t pmicRtcStatus = {0};

    test_pmic_print_unity_testcase_info(7467,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    status = Pmic_getRtcStatus(pPmicCoreHandle, &pmicRtcStatus);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INSUFFICIENT_CFG, status);
}

/*!
 * \brief   RTC  wakeup using time interrupt
 */
static void test_pmic_rtc_testWakeup_TimerIntr(void)
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
    bool standByState = 0U;
    uint32_t timerIntrdelayTime = 70000U;  // Timer Interrupt configured for 1 minute (60 Sec)
    uint32_t delayTime = 80000U;  // Added delay for workaround
    uint8_t i = 0U;

    test_pmic_print_unity_testcase_info(7358,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    pHandle                         = pPmicCoreHandle;

    for(i=0; i< 2; i++)
    {
        status = Pmic_rtcSetTimeDateInfo(pHandle, validTimeCfg, validDateCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        status = Pmic_rtcGetTimeDateInfo(pHandle, &timeCfg_rd, &dateCfg_rd);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        status = Pmic_rtcGetTimerPeriod(pHandle, &timerPeriod);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        irqNum = 0;

        if(i == 0)
        {
          standByState = PMIC_FSM_STANBY_STATE;
        }
        else
        {
          standByState = PMIC_FSM_LP_STANBY_STATE;
        }
        status = Pmic_rtcSetTimerPeriod(pHandle,
                                               PMIC_RTC_MINUTE_INTR_PERIOD);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        status = Pmic_rtcEnableTimerIntr(pHandle, PMIC_RTC_TIMER_INTR_ENABLE);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        /*To avoid DAP reset*/
        Osal_delay(delayTime);

        status = Pmic_fsmDeviceOffRequestCfg(pHandle,
                                             PMIC_FSM_I2C_TRIGGER0_TYPE,
                                             standByState);
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
    }

    status = Pmic_rtcSetTimerPeriod(pHandle, timerPeriod);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

}

/*!
 * \brief   RTC wakeup using Alarm interrupt
 */
static void test_pmic_rtc_testWakeup_AlarmIntr(void)
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
    bool standByState = 0U;
    uint32_t alarmIntrdelayTime = 70000U;  // Alarm Interrupt configured for 1 minute (60 Sec)
    uint32_t delayTimeBefore = 80000U;  // Added delay for workaround
    uint8_t i = 0U;

    test_pmic_print_unity_testcase_info(7359,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    for(i=0; i< 2; i++)
    {
        status = Pmic_rtcSetTimeDateInfo(pHandle, validTimeCfg, validDateCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        /* Get the current time value */
        status = Pmic_rtcGetTimeDateInfo(pHandle, &timeCfg_rd, &dateCfg_rd);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        irqNum = 0;

        if(i == 0)
        {
          standByState = PMIC_FSM_STANBY_STATE;
        }
        else
        {
          standByState = PMIC_FSM_LP_STANBY_STATE;
        }

        timeCfg_rd.minutes = timeCfg_rd.minutes + 1U;
        status   = Pmic_rtcSetAlarmInfo(pHandle, timeCfg_rd, dateCfg_rd);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        /* Set Alarm Interupt */
        status = Pmic_rtcEnableAlarmIntr(pHandle, PMIC_RTC_ALARM_INTR_ENABLE);
        /* Get the current time for timeout */
        status = Pmic_rtcGetTimeDateInfo(pHandle, &timeCfg_cr, &dateCfg_cr);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        /*To avoid DAP reset for FSM I2C0_Trigger*/
        Osal_delay(delayTimeBefore);

        status = Pmic_fsmDeviceOffRequestCfg(pHandle,
                                             PMIC_FSM_I2C_TRIGGER0_TYPE,
                                             standByState);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        Osal_delay(alarmIntrdelayTime);

        while(timeout--)
        {
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
    }
}

/*!
 * \brief   RTC timer Asynchronous Interrupt
 */
static void test_pmic_rtc_testTimerAsyncIntr(void)
{
    int32_t            status       = PMIC_ST_SUCCESS;
    int8_t             timeout      = 10U;
    Pmic_CoreHandle_t  *pHandle     = NULL;
    Pmic_RtcTime_t     timeCfg_cr   = { 0x1F, 0U, 0U, 0U, 0U, 0U};
    Pmic_RtcDate_t     dateCfg_cr   = { 0x0F, 0U, 0U, 0U, 0U};
    Pmic_RtcTime_t     timeCfg_rd   = { 0x1F, 0U, 0U, 0U, 0U, 0U};
    Pmic_RtcDate_t     dateCfg_rd   = { 0x0F, 0U, 0U, 0U, 0U};
    uint8_t            timerPeriod  = 0U;

    Pmic_RtcDate_t    validDateCfg =  { 0x0F, 15U, 6U, 2055U, 1U};
    Pmic_RtcTime_t    validTimeCfg  = { 0x1F, 30U, 30U, 6U, 0U, 1U};

    test_pmic_print_unity_testcase_info(1,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    /* Enable GPIO interrupt on the specific gpio pin */
    GPIO_enableInt(0);

    pHandle             = pPmicCoreHandle;
    pmic_intr_triggered = 0U;
    pPmicCoreHandleISR  = pHandle;
    irqNumToClr         = PMIC_TPS6594X_RTC_TIMER_INT;

    status = Pmic_rtcSetTimeDateInfo(pHandle, validTimeCfg, validDateCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_rtcGetTimeDateInfo(pHandle, &timeCfg_rd, &dateCfg_rd);
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

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    /* Disable the timer interrupt  */
    status = Pmic_rtcEnableTimerIntr(pHandle, PMIC_RTC_TIMER_INTR_DISABLE);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_rtcGetTimeDateInfo(pHandle, &timeCfg_cr, &dateCfg_cr);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_rtcSetTimerPeriod(pHandle, timerPeriod);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    /* Disable GPIO interrupt on the specific gpio pin */
    GPIO_disableInt(0);
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
    Pmic_RtcTime_t     timeCfg_cr   = { 0x1F, 0U, 0U, 0U, 0U, 0U};
    Pmic_RtcDate_t     dateCfg_cr   = { 0x0F, 0U, 0U, 0U, 0U};
    Pmic_RtcTime_t     timeCfg_rd   = { 0x1F, 0U, 0U, 0U, 0U, 0U};
    Pmic_RtcDate_t     dateCfg_rd   = { 0x0F, 0U, 0U, 0U, 0U};
    Pmic_RtcDate_t    validDateCfg =  { 0x0F, 15U, 6U, 2055U, 1U};
    Pmic_RtcTime_t    validTimeCfg  = { 0x1F, 30U, 30U, 6U, 0U, 1U};

    test_pmic_print_unity_testcase_info(2,
                                        pmic_rtc_tests,
                                        PMIC_RTC_NUM_OF_TESTCASES);

    /* Enable GPIO interrupt on the specific gpio pin */
    GPIO_enableInt(0);

    pmic_intr_triggered = 0U;
    pPmicCoreHandleISR  = pHandle;
    irqNumToClr         = PMIC_TPS6594X_RTC_ALARM_INT;

    status = Pmic_rtcSetTimeDateInfo(pHandle, validTimeCfg, validDateCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    /* Get the current time value */
    status = Pmic_rtcGetTimeDateInfo(pHandle, &timeCfg_rd, &dateCfg_rd);
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

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    /* Disable the timer interrupt  */
    status = Pmic_rtcEnableTimerIntr(pHandle, PMIC_RTC_TIMER_INTR_DISABLE);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_rtcGetTimeDateInfo(pHandle, &timeCfg_cr, &dateCfg_cr);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    /* Disable GPIO interrupt on the specific gpio pin */
    GPIO_disableInt(0);
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
    RUN_TEST(test_pmic_rtc_testGetRtcStatus_running);
    RUN_TEST(test_pmic_rtc_testGetRtcStatus_frozen);
    RUN_TEST(test_pmic_rtc_getRtcStatus_PrmValTest_handle);
    RUN_TEST(test_pmic_rtc_getRtcStatus_PrmValTest_rtcStatus);
    RUN_TEST(test_pmic_rtc_getRtcStatus_PrmValTest_validParams);
    RUN_TEST(test_pmic_rtc_testWakeup_TimerIntr);
    RUN_TEST(test_pmic_rtc_testWakeup_AlarmIntr);
    RUN_TEST(test_pmic_rtc_testTimerAsyncIntr);
    RUN_TEST(test_pmic_rtc_testAlarmAsyncIntr);

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
 * \brief   Unity Test App wrapper Function for LEO PMIC-B
 */
static int32_t test_pmic_leo_pmicB_testApp(void)
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
#endif
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

static int32_t setup_pmic_interrupt()
{
    int32_t status = PMIC_ST_SUCCESS;

#ifdef SOC_J721E
    status = test_pmic_leo_pmicA_rtc_testApp();
   /* Deinit pmic handle */
    if((pPmicCoreHandle != NULL) && (PMIC_ST_SUCCESS == status))
    {
        test_pmic_appDeInit(pPmicCoreHandle);
    }

    if(PMIC_ST_SUCCESS == status)
    {
        Pmic_IrqStatus_t intstat;
        Pmic_irqGetErrStatus(pPmicCoreHandle, &intstat,1);

        status = test_pmic_leo_pmicB_testApp();
       /* Deinit pmic handle */
        if((pPmicCoreHandle != NULL) && (PMIC_ST_SUCCESS == status))
        {
            Pmic_IrqStatus_t intstat;
            Pmic_irqGetErrStatus(pPmicCoreHandle, &intstat,1);
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
    " \r\n 0: Pmic Leo device(PMIC A on J721E EVM Using I2C Interface)"
    " \r\n 1: Pmic Leo device(PMIC A on J721E EVM Using SPI Stub Functions)"
    " \r\n 2: Pmic Leo device(PMIC B on J721E EVM)"
    " \r\n 3: quit"
    " \r\n"
    " \r\n Enter option: "
};

/*!
 * \brief   Function to register RTC Unity Test App wrapper to Unity framework
 */
static void test_pmic_rtc_testapp_runner(void)
{
    /* @description : Test runner for RTC Test App
     *
     * @requirements: PDK-5855
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
                   /* RTC Unity Test App wrapper Function for LEO PMIC-A */
                   test_pmic_leo_pmicA_rtc_testApp();

                   /* Run rtc test cases for Leo PMIC-A */
                   test_pmic_run_testcases();
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
                   /* RTC Unity Test App wrapper Function for LEO PMIC-A using
                    * SPI stub functions */
                   test_pmic_leo_pmicA_spiStub_rtc_testApp();
                   /* Run rtc test cases for Leo PMIC-A */
                   test_pmic_run_testcases();
                   /* Deinit pmic handle */
                   if(pPmicCoreHandle != NULL)
                   {
                       test_pmic_appDeInit(pPmicCoreHandle);
                   }
                }
               break;
           case 2U:
               /* RTC Unity Test App wrapper Function for LEO PMIC-B */
                pmic_log("RTC on LEO PMIC-B not supported due to HW limitation \n");
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
 * \brief   TI RTOS specific RTC TEST APP main Function
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code.
 *          For valid values see \ref Pmic_ErrorCodes
 */
int main()
{
    Board_initUART();

    /* GPIO Configuration
     * This API is required for Asynchronous Interrupts only
     */
    App_initGPIO();

    pmic_log("RTC Unity Test Application(%s %s)\n", __TIME__, __DATE__);
#if defined(UNITY_INCLUDE_CONFIG_V2_H) && \
    (defined(SOC_J721E)             || \
     defined(SOC_J7200))
    test_pmic_rtc_testapp_runner();
#endif
}
