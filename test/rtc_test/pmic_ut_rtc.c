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

/*!
 * \brief   Set RTC Alarm interrupt
 *
 * \retval  PMIC_UT_SUCCESS in case of success or
 *          PMIC_UT_FAILURE in case of failure.
 */
static bool test_pmic_rtc_alarmIntr(void *pPmicCoreHandle)
{
    int32_t status             = PMIC_ST_SUCCESS;
    Pmic_CoreHandle_t *pHandle = pPmicCoreHandle;
    Pmic_RtcTime_t timeCfg     = {PMIC_RTC_VALID_PARAM_TIME_CFG_VAL, 30U, 30U,
                                  6U, 0U, 1U};
    Pmic_RtcDate_t dateCfg     = {PMIC_RTC_VALID_PARAM_DATE_CFG_VAL, 15U, 6U,
                                  2055U, 1U};

    Pmic_RtcTime_t timeCfg_rd;
    Pmic_RtcDate_t dateCfg_rd;

    timeCfg.timeMode        = PMIC_RTC_12_HOUR_MODE;
    status = Pmic_rtcSetAlarmIntr(pHandle, &timeCfg, &dateCfg);
    if(status != PMIC_ST_SUCCESS)
    {
        pmic_log("Failed %s with status: %d\n\t",__func__, status);
        return PMIC_UT_FAILURE;
    }

    timeCfg_rd.validParams = 0x1FU;
    dateCfg_rd.validParams = 0x1FU;
    status = Pmic_rtcGetAlarmIntr(pHandle, &timeCfg_rd, &dateCfg_rd);
    if(status != PMIC_ST_SUCCESS)
    {
        pmic_log("Failed %s with status: %d\n\t",__func__, status);
        return PMIC_UT_FAILURE;
    }

    if((timeCfg.seconds      != timeCfg_rd.seconds ) ||
       (timeCfg.minutes      != timeCfg_rd.minutes ) ||
       (timeCfg.hour         != timeCfg_rd.hour)     ||
       (timeCfg.timeMode     != timeCfg_rd.timeMode) ||
       (timeCfg.meridianMode != timeCfg_rd.meridianMode))
    {
        pmic_log("Failed %s with status: %d\n\t",__func__, status);
        return PMIC_UT_FAILURE;
    }

    if((dateCfg.day   != dateCfg_rd.day)   ||
       (dateCfg.month != dateCfg_rd.month) ||
       (dateCfg.year  != dateCfg_rd.year))
    {
        pmic_log("Failed %s with status: %d\n\t",__func__, status);
        return PMIC_UT_FAILURE;
    }

    return PMIC_UT_SUCCESS;
}

/*!
 * \brief   Parameter validation for 'handle'
 *
 * \retval  PMIC_UT_SUCCESS in case of success or
 *          PMIC_UT_FAILURE in case of failure.
 */
static bool test_pmic_rtc_setAlarmIntrPrmValTest_handle(void *pPmicCoreHandle)
{
    int32_t status         = PMIC_ST_SUCCESS;
    Pmic_RtcTime_t timeCfg = {PMIC_RTC_VALID_PARAM_TIME_CFG_VAL, 30U, 30U, 6U,
                              0U, 1U};
    Pmic_RtcDate_t dateCfg = {PMIC_RTC_VALID_PARAM_DATE_CFG_VAL, 15U, 6U,
                              2055U,1U};

    status = Pmic_rtcSetAlarmIntr(NULL, &timeCfg, &dateCfg);
    if(PMIC_ST_ERR_INV_HANDLE != status)
    {
        pmic_log("Failed %s with status: %d\n\t",__func__, status);
        return PMIC_UT_FAILURE;
    }

    return PMIC_UT_SUCCESS;
}

/*!
 * \brief   Parameter validation for 'timeCfg'
 *
 * \retval  PMIC_UT_SUCCESS in case of success or
 *          PMIC_UT_FAILURE in case of failure.
 */
static bool test_pmic_rtc_setAlarmIntrPrmValTest_timeCfg(void *pPmicCoreHandle)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_CoreHandle_t *handle = pPmicCoreHandle;
    Pmic_RtcDate_t dateCfg = {PMIC_RTC_VALID_PARAM_DATE_CFG_VAL, 15U, 6U,
                              2055U, 1U};

    status = Pmic_rtcSetAlarmIntr(handle, NULL, &dateCfg);
    if(PMIC_ST_ERR_NULL_PARAM != status)
    {
        pmic_log("Failed %s with status: %d\n\t",__func__, status);
        return PMIC_UT_FAILURE;
    }

    return PMIC_UT_SUCCESS;
}

/*!
 * \brief   Parameter validation for 'dataCfg'
 *
 * \retval  PMIC_UT_SUCCESS in case of success or
 *          PMIC_UT_FAILURE in case of failure.
 */
static bool test_pmic_rtc_setAlarmIntrPrmValTest_dateCfg(void *pPmicCoreHandle)
{
    int32_t      status       = PMIC_ST_SUCCESS;
    Pmic_CoreHandle_t *handle = pPmicCoreHandle;
    Pmic_RtcTime_t timeCfg    = {PMIC_RTC_VALID_PARAM_TIME_CFG_VAL, 30U, 30U,
                                 6U, 0U, 1U};

    status = Pmic_rtcSetAlarmIntr(handle, &timeCfg, NULL);
    if(PMIC_ST_ERR_NULL_PARAM != status)
    {
        pmic_log("Failed %s with status: %d\n\t",__func__, status);
        return PMIC_UT_FAILURE;
    }

    return PMIC_UT_SUCCESS;
}

/*!
 * \brief   Parameter validation for 'seconds'
 *
 * \retval  PMIC_UT_SUCCESS in case of success or
 *          PMIC_UT_FAILURE in case of failure.
 */
static bool test_pmic_rtc_setAlarmIntrPrmValTest_seconds(void *pPmicCoreHandle)
{
    int32_t      status       = PMIC_ST_SUCCESS;
    Pmic_CoreHandle_t *handle = pPmicCoreHandle;
    Pmic_RtcTime_t timeCfg    = {PMIC_RTC_TIME_CFG_SEC_VALID_SHIFT, 30U, 30U,
                                 6U, 0U, 1U};
    Pmic_RtcDate_t dateCfg    = {0x00U, 15U, 6U, 2055U, 1U};

    timeCfg.seconds = PMIC_RTC_INVALID_SEC_MINUTE;

    status = Pmic_rtcSetAlarmIntr(handle, &timeCfg, &dateCfg);
    if(status != PMIC_ST_ERR_INV_TIME)
    {
        pmic_log("Failed %s with status: %d\n\t",__func__, status);
        return PMIC_UT_FAILURE;
    }

    return PMIC_UT_SUCCESS;
}

/*!
 * \brief   Parameter validation for 'minutes'
 *
 * \retval  PMIC_UT_SUCCESS in case of success or
 *          PMIC_UT_FAILURE in case of failure.
 */
static bool test_pmic_rtc_setAlarmIntrPrmValTest_minutes(void *pPmicCoreHandle)
{
    int32_t      status       = PMIC_ST_SUCCESS;
    Pmic_CoreHandle_t *handle = pPmicCoreHandle;
    Pmic_RtcTime_t timeCfg    = {PMIC_RTC_TIME_CFG_MIN_VALID_SHIFT, 30U, 30U,
                                 6U, 0U, 1U};
    Pmic_RtcDate_t dateCfg    = {0x00U, 15U, 6U, 2055U, 1U};

    timeCfg.minutes           = PMIC_RTC_INVALID_SEC_MINUTE;

    status = Pmic_rtcSetAlarmIntr(handle, &timeCfg, &dateCfg);
    if(status != PMIC_ST_ERR_INV_TIME)
    {
        pmic_log("Failed %s with status: %d\n\t",__func__, status);
        return PMIC_UT_FAILURE;
    }

    return PMIC_UT_SUCCESS;
}

/*!
 * \brief   Parameter validation for 'timeMode'
 *
 * \retval  PMIC_UT_SUCCESS in case of success or
 *          PMIC_UT_FAILURE in case of failure.
 */
static bool test_pmic_rtc_setAlarmIntrPrmValTest_timeMode(void *pPmicCoreHandle)
{
    int32_t      status       = PMIC_ST_SUCCESS;
    Pmic_CoreHandle_t *handle = pPmicCoreHandle;
    Pmic_RtcTime_t timeCfg    = {PMIC_RTC_TIME_CFG_TIMEMODE_VALID_SHIFT, 30U,
                                 30U, 6U, 0U, 1U};
    Pmic_RtcDate_t dateCfg    = {0x00U, 15U, 6U, 2055U, 1U};

    timeCfg.timeMode          = PMIC_RTC_INVALID_TIME_MODE;

    status = Pmic_rtcSetAlarmIntr(handle, &timeCfg, &dateCfg);
    if(status != PMIC_ST_ERR_INV_TIME)
    {
        pmic_log("Failed %s with status: %d\n\t",__func__, status);
        return PMIC_UT_FAILURE;
    }

    return PMIC_UT_SUCCESS;
}

/*!
 * \brief   Parameter validation for 'meridianMode'
 *
 * \retval  PMIC_UT_SUCCESS in case of success or
 *          PMIC_UT_FAILURE in case of failure.
 */
static bool test_pmic_rtc_setAlarmIntrPrmValTest_meridianMode
                                                    (void *pPmicCoreHandle)
{
    int32_t status         = PMIC_ST_SUCCESS;
    Pmic_CoreHandle_t *handle = pPmicCoreHandle;
    Pmic_RtcTime_t timeCfg = {PMIC_RTC_TIME_CFG_MERIDIAN_VALID_SHIFT,  30U,
                              30U, 6U, 0U, 1U};
    Pmic_RtcDate_t dateCfg = {0x00U,  15U, 6U, 2055U, 1U};

    timeCfg.timeMode       = PMIC_RTC_12_HOUR_MODE;
    timeCfg.meridianMode   = PMIC_RTC_INVALID_MERIDIEN_MODE;

    status = Pmic_rtcSetAlarmIntr(handle, &timeCfg, &dateCfg);
    if(status != PMIC_ST_ERR_INV_TIME)
    {
        pmic_log("Failed %s with status: %d\n\t",__func__, status);
        return PMIC_UT_FAILURE;
    }

    return PMIC_UT_SUCCESS;
}

/*!
 * \brief   Parameter validation for 'hour' when 'timeMode' = 1
 *
 * \retval  PMIC_UT_SUCCESS in case of success or
 *          PMIC_UT_FAILURE in case of failure.
 */
static bool test_pmic_rtc_setAlarmIntrPrmValTest_hour12(void *pPmicCoreHandle)
{
    int32_t      status      = PMIC_ST_SUCCESS;
    Pmic_CoreHandle_t *handle = pPmicCoreHandle;
    Pmic_RtcTime_t timeCfg    =
                              {PMIC_RTC_VALID_PARAM_TIME_HRS_TIMEMODE_CFG_VAL,
                               30U, 30U, 6U, 0U, 1U};
    Pmic_RtcDate_t dateCfg    = {0x00U, 15U, 6U, 2055U, 1U};

    timeCfg.timeMode          = PMIC_RTC_12_HOUR_MODE;
    timeCfg.hour              = PMIC_RTC_INVALID_HOUR_13;

    status = Pmic_rtcSetAlarmIntr(handle, &timeCfg, &dateCfg);
    if(status != PMIC_ST_ERR_INV_TIME)
    {
        pmic_log("Failed %s with status: %d\n\t",__func__, status);
        return PMIC_UT_FAILURE;
    }

    return PMIC_UT_SUCCESS;
}

/*!
 * \brief   Parameter validation for 'hour' when 'timeMode' = 0
 *
 * \retval  PMIC_UT_SUCCESS in case of success or
 *          PMIC_UT_FAILURE in case of failure.
 */
static bool test_pmic_rtc_setAlarmIntrPrmValTest_hour24(void *pPmicCoreHandle)
{
    int32_t      status      = PMIC_ST_SUCCESS;
    Pmic_CoreHandle_t *handle = pPmicCoreHandle;
    Pmic_RtcTime_t timeCfg    =
                            {PMIC_RTC_VALID_PARAM_TIME_HRS_TIMEMODE_CFG_VAL,
                             30U, 30U, 6U, 0U, 1U};
    Pmic_RtcDate_t dateCfg    = {0x00U, 15U, 6U, 2055U, 1U};

    timeCfg.timeMode          = PMIC_RTC_24_HOUR_MODE;
    timeCfg.hour              = PMIC_RTC_INVALID_HOUR_25;

    status = Pmic_rtcSetAlarmIntr(handle, &timeCfg, &dateCfg);
    if(status != PMIC_ST_ERR_INV_TIME)
    {
        pmic_log("Failed %s with status: %d\n\t",__func__, status);
        return PMIC_UT_FAILURE;
    }

    return PMIC_UT_SUCCESS;
}

/*!
 * \brief   Negative test for 'hour ' = 0U, when 'timeMode' = 1
 *
 * \retval  PMIC_UT_SUCCESS in case of success or
 *          PMIC_UT_FAILURE in case of failure.
 */
static bool test_pmic_rtc_setAlarmIntrPrmValTest_hour(void *pPmicCoreHandle)
{
    int32_t      status       = PMIC_ST_SUCCESS;
    Pmic_CoreHandle_t *handle = pPmicCoreHandle;
    Pmic_RtcTime_t timeCfg    =
                            {PMIC_RTC_VALID_PARAM_TIME_HRS_TIMEMODE_CFG_VAL,
                             30U, 30U,6U, 0U, 1U};
    Pmic_RtcDate_t dateCfg    = {0x00, 15U, 6U, 2055U, 1U};

    timeCfg.timeMode          = PMIC_RTC_12_HOUR_MODE;
    timeCfg.hour              = PMIC_RTC_HOUR_0;

    status = Pmic_rtcSetAlarmIntr(handle, &timeCfg, &dateCfg);
    if(status != PMIC_ST_ERR_INV_TIME)
    {
        pmic_log("Failed %s with status: %d\n\t",__func__, status);
        return PMIC_UT_FAILURE;
    }

    return PMIC_UT_SUCCESS;
}

/*!
 * \brief   Negative test for 'month' = 0
 *
 * \retval  PMIC_UT_SUCCESS in case of success or
 *          PMIC_UT_FAILURE in case of failure.
 */
static bool test_pmic_rtc_setAlarmIntrPrmValTest_month(void *pPmicCoreHandle)
{
    int32_t      status      = PMIC_ST_SUCCESS;
    Pmic_CoreHandle_t *handle = pPmicCoreHandle;
    Pmic_RtcTime_t timeCfg    = {0x00U, 30U, 30U, 6U, 0U, 1U};
    Pmic_RtcDate_t dateCfg    = {PMIC_RTC_DATE_CFG_MONTH_VALID_SHIFT, 15U, 6U,
                                 2055U, 1U};

    dateCfg.month             = PMIC_RTC_INVALID_MONTH_0;

    status = Pmic_rtcSetAlarmIntr(handle, &timeCfg, &dateCfg);
    if(PMIC_ST_ERR_INV_DATE != status)
    {
        pmic_log("Failed %s with status: %d\n\t",__func__, status);
        return PMIC_UT_FAILURE;
    }

    return PMIC_UT_SUCCESS;
}

/*!
 * \brief   Negative test for 'day' = 0
 *
 * \retval  PMIC_UT_SUCCESS in case of success or
 *          PMIC_UT_FAILURE in case of failure.
 */
static bool test_pmic_rtc_setAlarmIntrPrmValTest_day(void *pPmicCoreHandle)
{
    int32_t      status      = PMIC_ST_SUCCESS;
    Pmic_CoreHandle_t *handle = pPmicCoreHandle;
    Pmic_RtcTime_t timeCfg    = {0x00U, 30U, 30U, 6U, 0U, 1U};
    Pmic_RtcDate_t dateCfg    = {PMIC_RTC_DATE_CFG_DAY_VALID_SHIFT, 15U, 6U,
                                 2055U, 1U};

    dateCfg.day               = PMIC_RTC_INVALID_DAY_0;

    status = Pmic_rtcSetAlarmIntr(handle, &timeCfg, &dateCfg);
    if(PMIC_ST_ERR_INV_DATE != status)
    {
        pmic_log("Failed %s with status: %d\n\t",__func__, status);
        return PMIC_UT_FAILURE;
    }

    return PMIC_UT_SUCCESS;
}

/*!
 * \brief   Parameter range validation for 'year'
 *
 * \retval  PMIC_UT_SUCCESS in case of success or
 *          PMIC_UT_FAILURE in case of failure.
 */
static bool test_pmic_rtc_setAlarmIntrPrmValTest_year(void *pPmicCoreHandle)
{
    int32_t      status      = PMIC_ST_SUCCESS;
    Pmic_CoreHandle_t *handle = pPmicCoreHandle;
    Pmic_RtcTime_t timeCfg    = {0U, 30U, 30U, 6U, 0U, 1U};
    Pmic_RtcDate_t dateCfg    = {PMIC_RTC_DATE_CFG_WEEK_VALID_SHIFT, 15U, 6U,
                                 2055U, 1U};

    dateCfg.year              = PMIC_RTC_INVALID_YEAR;

    status = Pmic_rtcSetAlarmIntr(handle, &timeCfg, &dateCfg);
    if(PMIC_ST_ERR_INV_DATE != status)
    {
        pmic_log("Failed %s with status: %d\n\t",__func__, status);
        return PMIC_UT_FAILURE;
    }

    return PMIC_UT_SUCCESS;
}

/*!
 * \brief   Parameter range validation for 'month'
 *
 * \retval  PMIC_UT_SUCCESS in case of success or
 *          PMIC_UT_FAILURE in case of failure.
 */
static bool test_pmic_rtc_setAlarmIntrPrmValTest_month_range
                                                    (void *pPmicCoreHandle)
{
    int32_t      status      = PMIC_ST_SUCCESS;
    Pmic_CoreHandle_t *handle = pPmicCoreHandle;
    Pmic_RtcTime_t timeCfg    = {0U, 30U, 30U, 6U, 0U, 1U};
    Pmic_RtcDate_t dateCfg    = {PMIC_RTC_DATE_CFG_MONTH_VALID_SHIFT, 15U, 6U,
                                 2055U, 1U};

    dateCfg.month             = PMIC_RTC_INVALID_MONTH;

    status = Pmic_rtcSetAlarmIntr(handle, &timeCfg, &dateCfg);
    if(PMIC_ST_ERR_INV_DATE != status)
    {
        pmic_log("Failed %s with status: %d\n\t",__func__, status);
        return PMIC_UT_FAILURE;
    }

    return PMIC_UT_SUCCESS;
}

/*!
 * \brief   Parameter range validation for 'day'for months with 30 days
 *
 * \retval  PMIC_UT_SUCCESS in case of success or
 *          PMIC_UT_FAILURE in case of failure.
 */
static bool test_pmic_rtc_setAlarmIntrPrmValTest_day_month
                                                    (void *pPmicCoreHandle)
{
    int32_t      status      = PMIC_ST_SUCCESS;
    Pmic_CoreHandle_t *handle = pPmicCoreHandle;
    Pmic_RtcTime_t timeCfg    = {0U, 30U, 30U, 6U, 0U, 1U};
    Pmic_RtcDate_t dateCfg    = {PMIC_RTC_VALID_PARAM_DATE_CFG_DAY_MNTH_VAL,
                                 15U, 6U,2055U, 1U};
    dateCfg.month             = PMIC_RTC_MONTH_APR;
    dateCfg.day               = PMIC_RTC_INVALID_DAY_31;

    status = Pmic_rtcSetAlarmIntr(handle, &timeCfg, &dateCfg);
    if(PMIC_ST_ERR_INV_DATE != status)
    {
        pmic_log("Failed %s with status: %d\n\t",__func__, status);
        return PMIC_UT_FAILURE;
    }

    return PMIC_UT_SUCCESS;
}

/*!
 * \brief   Parameter range validation for 'day' for leap
 *          year('year' %4 = 0 ) and 'month' = 2 (february)
 *
 * \retval  PMIC_UT_SUCCESS in case of success or
 *          PMIC_UT_FAILURE in case of failure.
 */
static bool test_pmic_rtc_setAlarmIntrPrmValTest_feb_leapyear
                                                    (void *pPmicCoreHandle)
{
    int32_t      status      = PMIC_ST_SUCCESS;
    Pmic_CoreHandle_t *handle = pPmicCoreHandle;
    Pmic_RtcTime_t timeCfg    = {0x0U, 30U, 30U, 6U, 0U, 1U};
    Pmic_RtcDate_t dateCfg    = {PMIC_RTC_VALID_PARAM_DATE_CFG_DAY_MNTH_YR_VAL,
                                 15U, 6U,2055U, 1U};

    dateCfg.month             = PMIC_RTC_MONTH_FEB;
    dateCfg.year              = PMIC_RTC_YEAR_2044;
    dateCfg.day               = PMIC_RTC_INVALID_DAY_30;

    status = Pmic_rtcSetAlarmIntr(handle, &timeCfg, &dateCfg);
    if(PMIC_ST_ERR_INV_DATE != status)
    {
        pmic_log("Failed %s with status: %d\n\t",__func__, status);
        return PMIC_UT_FAILURE;
    }

    return PMIC_UT_SUCCESS;
}

/*!
 * \brief   Parameter range validation for 'day' for Non-leap
 *          year('year' %4 != 0 ) and 'month' = 2 (february)
 *
 * \retval  PMIC_UT_SUCCESS in case of success or
 *          PMIC_UT_FAILURE in case of failure.
 */
static bool test_pmic_rtc_setAlarmIntrPrmValTest_feb_nonleapyear
                                                    (void *pPmicCoreHandle)
{
    int32_t      status      = PMIC_ST_SUCCESS;
    Pmic_CoreHandle_t *handle = pPmicCoreHandle;
    Pmic_RtcTime_t timeCfg    = {0U, 30U, 30U, 6U, 0U, 1U};
    Pmic_RtcDate_t dateCfg    = {PMIC_RTC_VALID_PARAM_DATE_CFG_DAY_MNTH_YR_VAL,
                                 15U, 6U,2055U, 1U};

    dateCfg.month             = PMIC_RTC_MONTH_FEB;
    dateCfg.year              = PMIC_RTC_YEAR_2045;
    dateCfg.day               = PMIC_RTC_INVALID_DAY;

    status = Pmic_rtcSetAlarmIntr(handle, &timeCfg, &dateCfg);
    if(PMIC_ST_ERR_INV_DATE != status)
    {
        pmic_log("Failed %s with status: %d\n\t",__func__, status);
        return PMIC_UT_FAILURE;
    }

    return PMIC_UT_SUCCESS;
}

/*!
 * \brief   Parameter range validation for 'day', for months with 31 days
 *
 * \retval  PMIC_UT_SUCCESS in case of success or
 *          PMIC_UT_FAILURE in case of failure.
 */
static bool test_pmic_rtc_setAlarmIntrPrmValTest_day_month31
                                                    (void *pPmicCoreHandle)
{
    int32_t      status      = PMIC_ST_SUCCESS;
    Pmic_CoreHandle_t *handle = pPmicCoreHandle;
    Pmic_RtcTime_t timeCfg    = {0U,  30U, 30U, 6U, 0U, 1U};
    Pmic_RtcDate_t dateCfg    = {PMIC_RTC_VALID_PARAM_DATE_CFG_DAY_MNTH_VAL,
                                 15U, 6U,2055U, 1U};

    dateCfg.month             = PMIC_RTC_MONTH_JUL;
    dateCfg.day               = PMIC_RTC_INVALID_DAY_32;

    status = Pmic_rtcSetAlarmIntr(handle, &timeCfg, &dateCfg);
    if(PMIC_ST_ERR_INV_DATE != status)
    {
        pmic_log("Failed %s with status: %d\n\t",__func__, status);
        return PMIC_UT_FAILURE;
    }

    return PMIC_UT_SUCCESS;
}

/*!
 * \brief   Parameter validation for 'handle'
 *
 * \retval  PMIC_UT_SUCCESS in case of success or
 *          PMIC_UT_FAILURE in case of failure.
 */
static bool test_pmic_rtc_getAlarmIntrPrmValTest_handle(void *pPmicCoreHandle)
{
    int32_t      status      = PMIC_ST_SUCCESS;
    Pmic_RtcTime_t timeCfg    = {PMIC_RTC_VALID_PARAM_TIME_CFG_VAL, 30U, 30U,
                                 6U, 0U, 1U};
    Pmic_RtcDate_t dateCfg    = {PMIC_RTC_VALID_PARAM_DATE_CFG_VAL,  15U, 6U,
                                 2055U, 1U};

    status = Pmic_rtcGetAlarmIntr(NULL, &timeCfg, &dateCfg);
    if(PMIC_ST_ERR_INV_HANDLE != status)
    {
        pmic_log("Failed %s with status: %d\n\t",__func__, status);
        return PMIC_UT_FAILURE;
    }

    return PMIC_UT_SUCCESS;
}

/*!
 * \brief   Parameter validation for 'timeCfg'
 *
 * \retval  PMIC_UT_SUCCESS in case of success or
 *          PMIC_UT_FAILURE in case of failure.
 */
static bool test_pmic_rtc_getAlarmIntrPrmValTest_timeCfg(void *pPmicCoreHandle)
{
    int32_t status         = PMIC_ST_SUCCESS;
    Pmic_CoreHandle_t *handle = pPmicCoreHandle;
    Pmic_RtcDate_t dateCfg = {PMIC_RTC_VALID_PARAM_DATE_CFG_VAL, 15U, 6U,
                              2055U, 1U};

    status = Pmic_rtcGetAlarmIntr(handle, NULL, &dateCfg);
    if(PMIC_ST_ERR_NULL_PARAM != status)
    {
        pmic_log("Failed %s with status: %d\n\t",__func__, status);
        return PMIC_UT_FAILURE;
    }

    return PMIC_UT_SUCCESS;
}

/*!
 * \brief   Parameter validation for 'dataCfg'
 *
 * \retval  PMIC_UT_SUCCESS in case of success or
 *          PMIC_UT_FAILURE in case of failure.
 */
static bool test_pmic_rtc_getAlarmIntrPrmValTest_dateCfg(void *pPmicCoreHandle)
{
    int32_t      status      = PMIC_ST_SUCCESS;
    Pmic_CoreHandle_t *handle = pPmicCoreHandle;
    Pmic_RtcTime_t timeCfg    = {PMIC_RTC_VALID_PARAM_TIME_CFG_VAL, 30U, 30U,
                                 6U, 0U, 1U};

    status = Pmic_rtcGetAlarmIntr(handle, &timeCfg, NULL);
    if(PMIC_ST_ERR_NULL_PARAM != status)
    {
        pmic_log("Failed %s with status: %d\n\t",__func__, status);
        return PMIC_UT_FAILURE;
    }

    return PMIC_UT_SUCCESS;
}

/*!
 * \brief   Test Set RTC Timer interrupt
 *
 * \retval  PMIC_UT_SUCCESS in case of success or
 *          PMIC_UT_FAILURE in case of failure.
 */
static bool test_pmic_rtc_timer(void *pPmicCoreHandle)
{
    int32_t status         = PMIC_ST_SUCCESS;
    Pmic_CoreHandle_t *handle = pPmicCoreHandle;
    uint8_t timerPeriod, timerPeriod_rd;

    timerPeriod = PMIC_RTC_HOUR_INTR_PERIOD;

    status = Pmic_rtcSetTimerIntr(handle, timerPeriod);
    if(status != 0U)
    {
        pmic_log("Failed %s with status: %d\n\t",__func__, status);
        return PMIC_UT_FAILURE;
    }

    status = Pmic_rtcGetTimerIntr(handle, &timerPeriod_rd);
    if(status != 0U)
    {
        pmic_log("Failed %s with status: %d\n\t",__func__, status);
        return PMIC_UT_FAILURE;
    }

    if((timerPeriod != timerPeriod_rd))
    {
        pmic_log("Failed %s with status: %d\n\t",__func__, status);
        return PMIC_UT_FAILURE;
    }

    return PMIC_UT_SUCCESS;
}

/*!
 * \brief   Parameter validation for 'handle'
 *
 * \retval  PMIC_UT_SUCCESS in case of success or
 *          PMIC_UT_FAILURE in case of failure.
 */
static bool test_pmic_rtc_setTimerIntrPrmValTest_handle(void *pPmicCoreHandle)
{
    int32_t status       = PMIC_ST_SUCCESS;
    uint8_t  timerPeriod = PMIC_RTC_HOUR_INTR_PERIOD;

    status = Pmic_rtcSetTimerIntr(NULL, timerPeriod);
    if(PMIC_ST_ERR_INV_HANDLE != status)
    {
        pmic_log("Failed %s with status: %d\n\t",__func__, status);
        return PMIC_UT_FAILURE;
    }

    return PMIC_UT_SUCCESS;
}

/*!
 * \brief   Parameter validation for 'timerPeriod'
 *
 * \retval  PMIC_UT_SUCCESS in case of success or
 *          PMIC_UT_FAILURE in case of failure.
 */
static bool test_pmic_rtc_setTimerIntrPrmValTest_timerPeriod
                                                    (void *pPmicCoreHandle)
{
    int32_t status         = PMIC_ST_SUCCESS;
    Pmic_CoreHandle_t *handle = pPmicCoreHandle;
    uint8_t timerPeriod    = PMIC_RTC_HOUR_INTR_PERIOD;
    timerPeriod            = 5;

    status = Pmic_rtcSetTimerIntr(handle, timerPeriod);
    if(status != PMIC_ST_ERR_INV_PARAM)
    {
        pmic_log("Failed %s with status: %d\n\t",__func__, status);
        return PMIC_UT_FAILURE;
    }

    return PMIC_UT_SUCCESS;
}

/*!
 * \brief   Parameter validation for 'handle'
 *
 * \retval  PMIC_UT_SUCCESS in case of success or
 *          PMIC_UT_FAILURE in case of failure.
 */
static bool test_pmic_rtc_getTimerIntrPrmValTest_handle(void *pPmicCoreHandle)
{
    int32_t status       = PMIC_ST_SUCCESS;
    uint8_t timerPeriod  = PMIC_RTC_HOUR_INTR_PERIOD;

    status = Pmic_rtcGetTimerIntr(NULL, &timerPeriod);
    if(PMIC_ST_ERR_INV_HANDLE != status)
    {
        pmic_log("Failed %s with status: %d\n\t",__func__, status);
        return PMIC_UT_FAILURE;
    }

    return PMIC_UT_SUCCESS;
}

/*!
 * \brief   Parameter validation for 'timerPeriod
 *
 * \retval  PMIC_UT_SUCCESS in case of success or
 *          PMIC_UT_FAILURE in case of failure.
 */
static bool test_pmic_rtc_getTimerIntrPrmValTest_timerPeriod
                                                    (void *pPmicCoreHandle)
{
    int32_t status    = PMIC_ST_SUCCESS;
    Pmic_CoreHandle_t *handle = pPmicCoreHandle;

    status = Pmic_rtcGetTimerIntr(handle, NULL);
    if(PMIC_ST_ERR_NULL_PARAM != status)
    {
        pmic_log("Failed %s with status: %d\n\t",__func__, status);
        return PMIC_UT_FAILURE;
    }

    return PMIC_UT_SUCCESS;
}

/*!
 * \brief   Test RTC Disable
 *
 * \retval  PMIC_UT_SUCCESS in case of success or
 *          PMIC_UT_FAILURE in case of failure.
 */
static bool test_pmic_rtc_disable(void *pPmicCoreHandle)
{
    int32_t status         = PMIC_ST_SUCCESS;
    Pmic_CoreHandle_t *handle = pPmicCoreHandle;

    status = Pmic_rtcEnable(handle, PMIC_RTC_STOP);
    if(PMIC_ST_SUCCESS != status)
    {
        pmic_log("Failed %s with status: %d\n\t",__func__, status);
        return PMIC_UT_FAILURE;
    }

    return PMIC_UT_SUCCESS;
}

/*!
 * \brief   Parameter validation for 'handle'
 *
 * \retval  PMIC_UT_SUCCESS in case of success or
 *          PMIC_UT_FAILURE in case of failure.
 */
static bool test_pmic_rtc_disablePrmValTest_handle(void *pPmicCoreHandle)
{
    int32_t status         = PMIC_ST_SUCCESS;

    status = Pmic_rtcEnable(NULL, PMIC_RTC_STOP);
    if(PMIC_ST_ERR_INV_HANDLE != status)
    {
        pmic_log("Failed %s with status: %d\n\t",__func__, status);
        return PMIC_UT_FAILURE;
    }

    return PMIC_UT_SUCCESS;
}

/*!
 * \brief   Test RTC Enable
 *
 * \retval  PMIC_UT_SUCCESS in case of success or
 *          PMIC_UT_FAILURE in case of failure.
 */
static bool test_pmic_rtc_enable(void *pPmicCoreHandle)
{
    int32_t status         = PMIC_ST_SUCCESS;
    Pmic_CoreHandle_t *handle = pPmicCoreHandle;

    status = Pmic_rtcEnable(handle, PMIC_RTC_START);
    if(PMIC_ST_SUCCESS != status)
    {
        pmic_log("Failed %s with status: %d\n\t",__func__, status);
        return PMIC_UT_FAILURE;
    }

    return PMIC_UT_SUCCESS;
}

/*!
 * \brief   Parameter validation for 'handle'
 *
 * \retval  PMIC_UT_SUCCESS in case of success or
 *          PMIC_UT_FAILURE in case of failure.
 */
static bool test_pmic_rtc_enablePrmValTest_handle(void *pPmicCoreHandle)
{
    int32_t status         = PMIC_ST_SUCCESS;

    status = Pmic_rtcEnable(NULL, PMIC_RTC_START);
    if(PMIC_ST_ERR_INV_HANDLE != status)
    {
        pmic_log("Failed %s with status: %d\n\t",__func__, status);
        return PMIC_UT_FAILURE;
    }

    return PMIC_UT_SUCCESS;
}

/*!
 * \brief   Test RTC for Set Time
 *
 * \retval  PMIC_UT_SUCCESS in case of success or
 *          PMIC_UT_FAILURE in case of failure.
 */
static bool test_pmic_rtc_time(void *pPmicCoreHandle)
{
    int32_t status         = PMIC_ST_SUCCESS;
    Pmic_CoreHandle_t *handle = pPmicCoreHandle;

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

    timeCfg.timeMode       = PMIC_RTC_12_HOUR_MODE;
    status = Pmic_rtcSetTimeDateInfo(handle, &timeCfg, &dateCfg);
    if(status != 0U)
    {
        pmic_log("Failed %s with status: %d\n\t",__func__, status);
        return PMIC_UT_FAILURE;
    }

    timeCfg_rd.validParams = PMIC_RTC_VALID_PARAM_TIME_CFG_VAL;
    dateCfg_rd.validParams = PMIC_RTC_VALID_PARAM_DATE_CFG_VAL;
    status = Pmic_rtcGetTimeDateInfo(handle, &timeCfg_rd, &dateCfg_rd);
    if(status != 0U)
    {
        pmic_log("Failed %s with status: %d\n\t",__func__, status);
        return PMIC_UT_FAILURE;
    }

    if((timeCfg.minutes      != timeCfg_rd.minutes)  ||
       (timeCfg.hour         != timeCfg_rd.hour)     ||
       (timeCfg.timeMode     != timeCfg_rd.timeMode) ||
       (timeCfg.meridianMode != timeCfg_rd.meridianMode))
    {
        pmic_log("Failed %s with status: %d\n\t",__func__, status);
        return PMIC_UT_FAILURE;
    }

    if((dateCfg.day          != dateCfg_rd.day)   ||
       (dateCfg.month        != dateCfg_rd.month) ||
       ((dateCfg.year % 100) != dateCfg_rd.year)  ||
       (dateCfg.week         != dateCfg_rd.week))
    {
        pmic_log("Failed %s with status: %d\n\t",__func__, status);
        return PMIC_UT_FAILURE;
    }

    return PMIC_UT_SUCCESS;
}

/*!
 * \brief   Parameter validation for 'handle'
 *
 * \retval  PMIC_UT_SUCCESS in case of success or
 *          PMIC_UT_FAILURE in case of failure.
 */
static bool test_pmic_rtc_setTimePrmValTest_handle(void *pPmicCoreHandle)
{
    int32_t      status      = PMIC_ST_SUCCESS;
    Pmic_RtcTime_t timeCfg    = {PMIC_RTC_VALID_PARAM_TIME_CFG_VAL,  30U, 30U,
                                 6U, 0U, 1U};
    Pmic_RtcDate_t dateCfg    = {PMIC_RTC_VALID_PARAM_DATE_CFG_VAL, 15U, 6U,
                                 2055U, 1U};

    status = Pmic_rtcSetTimeDateInfo(NULL, &timeCfg, &dateCfg);
    if(PMIC_ST_ERR_INV_HANDLE != status)
    {
        pmic_log("Failed %s with status: %d\n\t",__func__, status);
        return PMIC_UT_FAILURE;
    }

    return PMIC_UT_SUCCESS;
}

/*!
 * \brief   Parameter validation for 'timeCfg'
 *
 * \retval  PMIC_UT_SUCCESS in case of success or
 *          PMIC_UT_FAILURE in case of failure.
 */
static bool test_pmic_rtc_setTimePrmValTest_timeCfg(void *pPmicCoreHandle)
{
    int32_t status         = PMIC_ST_SUCCESS;
    Pmic_CoreHandle_t *handle = pPmicCoreHandle;
    Pmic_RtcDate_t dateCfg = {PMIC_RTC_VALID_PARAM_DATE_CFG_VAL, 15U, 6U,
                              2055U, 1U};

    status = Pmic_rtcSetTimeDateInfo(handle, NULL, &dateCfg);
    if(PMIC_ST_ERR_NULL_PARAM != status)
    {
        pmic_log("Failed %s with status: %d\n\t",__func__, status);
        return PMIC_UT_FAILURE;
    }

    return PMIC_UT_SUCCESS;
}

/*!
 * \brief   Parameter validation for 'dataCfg'
 *
 * \retval  PMIC_UT_SUCCESS in case of success or
 *          PMIC_UT_FAILURE in case of failure.
 */
static bool test_pmic_rtc_setTimePrmValTest_dateCfg(void *pPmicCoreHandle)
{
    int32_t      status      = PMIC_ST_SUCCESS;
    Pmic_CoreHandle_t *handle = pPmicCoreHandle;
    Pmic_RtcTime_t timeCfg    = {PMIC_RTC_VALID_PARAM_TIME_CFG_VAL, 30U, 30U,
                                 6U, 0U, 1U};

    status = Pmic_rtcSetTimeDateInfo(handle, &timeCfg, NULL);
    if(PMIC_ST_ERR_NULL_PARAM != status)
    {
        pmic_log("Failed %s with status: %d\n\t",__func__, status);
        return PMIC_UT_FAILURE;
    }

    return PMIC_UT_SUCCESS;
}

/*!
 * \brief   Parameter validation for 'seconds'
 *
 * \retval  PMIC_UT_SUCCESS in case of success or
 *          PMIC_UT_FAILURE in case of failure.
 */
static bool test_pmic_rtc_setTimePrmValTest_seconds(void *pPmicCoreHandle)
{
    int32_t      status      = PMIC_ST_SUCCESS;
    Pmic_CoreHandle_t *handle = pPmicCoreHandle;
    Pmic_RtcTime_t timeCfg    = {PMIC_RTC_TIME_CFG_SEC_VALID_SHIFT, 30U, 30U,
                                 6U, 0U, 1U};
    Pmic_RtcDate_t dateCfg    = {0U, 15U, 6U, 2055U, 1U};

    timeCfg.seconds           = PMIC_RTC_INVALID_SEC_MINUTE;

    status = Pmic_rtcSetTimeDateInfo(handle, &timeCfg, &dateCfg);
    if(status != PMIC_ST_ERR_INV_TIME)
    {
        pmic_log("Failed %s with status: %d\n\t",__func__, status);
        return PMIC_UT_FAILURE;
    }

    return PMIC_UT_SUCCESS;
}

/*!
 * \brief   Parameter validation for 'minutes'
 *
 * \retval  PMIC_UT_SUCCESS in case of success or
 *          PMIC_UT_FAILURE in case of failure.
 */
static bool test_pmic_rtc_setTimePrmValTest_minutes(void *pPmicCoreHandle)
{
    int32_t      status      = PMIC_ST_SUCCESS;
    Pmic_CoreHandle_t *handle = pPmicCoreHandle;
    Pmic_RtcTime_t timeCfg    = {PMIC_RTC_TIME_CFG_MIN_VALID_SHIFT, 30U, 30U,
                                 6U, 0U, 1U};
    Pmic_RtcDate_t dateCfg    = {0U, 15U, 6U, 2055U, 1U};

    timeCfg.minutes           = PMIC_RTC_INVALID_SEC_MINUTE;

    status = Pmic_rtcSetTimeDateInfo(handle, &timeCfg, &dateCfg);
    if(status != PMIC_ST_ERR_INV_TIME)
    {
        pmic_log("Failed %s with status: %d\n\t",__func__, status);
        return PMIC_UT_FAILURE;
    }

    return PMIC_UT_SUCCESS;
}

/*!
 * \brief   Parameter validation for 'timeMode'
 *
 * \retval  PMIC_UT_SUCCESS in case of success or
 *          PMIC_UT_FAILURE in case of failure.
 */
static bool test_pmic_rtc_setTimePrmValTest_timeMode(void *pPmicCoreHandle)
{
    int32_t      status      = PMIC_ST_SUCCESS;
    Pmic_CoreHandle_t *handle = pPmicCoreHandle;
    Pmic_RtcTime_t timeCfg    = {PMIC_RTC_TIME_CFG_TIMEMODE_VALID_SHIFT, 30U,
                                 30U, 6U, 0U, 1U};
    Pmic_RtcDate_t dateCfg    = {0x0U, 15U, 6U, 2055U, 1U};

    timeCfg.timeMode           = PMIC_RTC_INVALID_TIME_MODE;

    status = Pmic_rtcSetTimeDateInfo(handle, &timeCfg, &dateCfg);
    if(status != PMIC_ST_ERR_INV_TIME)
    {
        pmic_log("Failed %s with status: %d\n\t",__func__, status);
        return PMIC_UT_FAILURE;
    }

    return PMIC_UT_SUCCESS;
}

/*!
 * \brief   Parameter validation for 'meridianMode'
 *
 * \retval  PMIC_UT_SUCCESS in case of success or
 *          PMIC_UT_FAILURE in case of failure.
 */
static bool test_pmic_rtc_setTimePrmValTest_meridianMode(void *pPmicCoreHandle)
{
    int32_t status         = PMIC_ST_SUCCESS;
    Pmic_CoreHandle_t *handle = pPmicCoreHandle;
    Pmic_RtcTime_t timeCfg = {PMIC_RTC_TIME_CFG_MERIDIAN_VALID_SHIFT, 30U, 30U,
                              6U, 0U, 1U};
    Pmic_RtcDate_t dateCfg = {0U,  15U, 6U, 2055U, 1U};
    timeCfg.timeMode       = PMIC_RTC_12_HOUR_MODE;
    timeCfg.meridianMode   = PMIC_RTC_INVALID_MERIDIEN_MODE;

    status = Pmic_rtcSetTimeDateInfo(handle, &timeCfg, &dateCfg);
    if(status != PMIC_ST_ERR_INV_TIME)
    {
        pmic_log("Failed %s with status: %d\n\t",__func__, status);
        return PMIC_UT_FAILURE;
    }

    return PMIC_UT_SUCCESS;
}

/*!
 * \brief   Parameter validation for 'hour' when 'timeMode' = 1
 *
 * \retval  PMIC_UT_SUCCESS in case of success or
 *          PMIC_UT_FAILURE in case of failure.
 */
static bool test_pmic_rtc_setTimePrmValTest_hour12(void *pPmicCoreHandle)
{
    int32_t      status      = PMIC_ST_SUCCESS;
    Pmic_CoreHandle_t *handle = pPmicCoreHandle;
    Pmic_RtcTime_t timeCfg    =
                              {PMIC_RTC_VALID_PARAM_TIME_HRS_TIMEMODE_CFG_VAL,
                               30U, 30U, 6U, 0U, 1U};
    Pmic_RtcDate_t dateCfg    = {0x0U, 15U, 6U, 2055U, 1U};

    timeCfg.timeMode          = PMIC_RTC_12_HOUR_MODE;
    timeCfg.hour              = PMIC_RTC_INVALID_HOUR_13;

    status = Pmic_rtcSetTimeDateInfo(handle, &timeCfg, &dateCfg);
    if(status != PMIC_ST_ERR_INV_TIME)
    {
        pmic_log("Failed %s with status: %d\n\t",__func__, status);
        return PMIC_UT_FAILURE;
    }

    return PMIC_UT_SUCCESS;
}

/*!
 * \brief   Parameter validation for 'hour' when 'timeMode' = 0
 *
 * \retval  PMIC_UT_SUCCESS in case of success or
 *          PMIC_UT_FAILURE in case of failure.
 */
static bool test_pmic_rtc_setTimePrmValTest_hour24(void *pPmicCoreHandle)
{
    int32_t      status      = PMIC_ST_SUCCESS;
    Pmic_CoreHandle_t *handle = pPmicCoreHandle;
    Pmic_RtcTime_t timeCfg    =
                              {PMIC_RTC_VALID_PARAM_TIME_HRS_TIMEMODE_CFG_VAL,
                               30U, 30U, 6U, 0U, 1U};
    Pmic_RtcDate_t dateCfg    = {0x0U, 15U, 6U, 2055U, 1U};

    timeCfg.timeMode          = PMIC_RTC_12_HOUR_MODE;
    timeCfg.hour              = PMIC_RTC_INVALID_HOUR_25;

    status = Pmic_rtcSetTimeDateInfo(handle, &timeCfg, &dateCfg);
    if(status != PMIC_ST_ERR_INV_TIME)
    {
        pmic_log("Failed %s with status: %d\n\t",__func__, status);
        return PMIC_UT_FAILURE;
    }

    return PMIC_UT_SUCCESS;
}

/*!
 * \brief   Negative test for 'hour ' = 0U, when 'timeMode' = 1
 *
 * \retval  PMIC_UT_SUCCESS in case of success or
 *          PMIC_UT_FAILURE in case of failure.
 */
static bool test_pmic_rtc_setTimePrmValTest_hour(void *pPmicCoreHandle)
{
    int32_t      status      = PMIC_ST_SUCCESS;
    Pmic_CoreHandle_t *handle = pPmicCoreHandle;
    Pmic_RtcTime_t timeCfg    =
                              {PMIC_RTC_VALID_PARAM_TIME_HRS_TIMEMODE_CFG_VAL,
                               30U, 30U, 6U, 0U, 1U};
    Pmic_RtcDate_t dateCfg    = {0U, 15U, 6U, 2055U, 1U};

    timeCfg.timeMode          = PMIC_RTC_12_HOUR_MODE;
    timeCfg.hour              = PMIC_RTC_INVALID_HOUR_0;

    status = Pmic_rtcSetTimeDateInfo(handle, &timeCfg, &dateCfg);
    if(status != PMIC_ST_ERR_INV_TIME)
    {
        pmic_log("Failed %s with status: %d\n\t",__func__, status);
        return PMIC_UT_FAILURE;
    }

    return PMIC_UT_SUCCESS;
}

/*!
 * \brief   Negative test for 'month ' = 0
 *
 * \retval  PMIC_UT_SUCCESS in case of success or
 *          PMIC_UT_FAILURE in case of failure.
 */
static bool test_pmic_rtc_setTimePrmValTest_month(void *pPmicCoreHandle)
{
    int32_t      status      = PMIC_ST_SUCCESS;
    Pmic_CoreHandle_t *handle = pPmicCoreHandle;
    Pmic_RtcTime_t timeCfg    = {0U, 30U, 30U, 6U, 0U, 1U};
    Pmic_RtcDate_t dateCfg    = {PMIC_RTC_DATE_CFG_MONTH_VALID_SHIFT, 15U, 6U,
                                 2055U, 1U};

    dateCfg.month             = PMIC_RTC_INVALID_MONTH_0;

    status = Pmic_rtcSetTimeDateInfo(handle, &timeCfg, &dateCfg);
    if(PMIC_ST_ERR_INV_DATE != status)
    {
        pmic_log("Failed %s with status: %d\n\t",__func__, status);
        return PMIC_UT_FAILURE;
    }

    return PMIC_UT_SUCCESS;
}

/*!
 * \brief   Negative test for 'day' = 0
 *
 * \retval  PMIC_UT_SUCCESS in case of success or
 *          PMIC_UT_FAILURE in case of failure.
 */
static bool test_pmic_rtc_setTimePrmValTest_day(void *pPmicCoreHandle)
{
    int32_t      status      = PMIC_ST_SUCCESS;
    Pmic_CoreHandle_t *handle = pPmicCoreHandle;
    Pmic_RtcTime_t timeCfg    = {0x0U, 30U, 30U, 6U, 0U, 1U};
    Pmic_RtcDate_t dateCfg    = {PMIC_RTC_DATE_CFG_DAY_VALID_SHIFT, 15U, 6U,
                                 2055U, 1U};

    dateCfg.day               = PMIC_RTC_INVALID_DAY_0;

    status = Pmic_rtcSetTimeDateInfo(handle, &timeCfg, &dateCfg);
    if(PMIC_ST_ERR_INV_DATE != status)
    {
        pmic_log("Failed %s with status: %d\n\t",__func__, status);
        return PMIC_UT_FAILURE;
    }

    return PMIC_UT_SUCCESS;
}

/*!
 * \brief   Parameter range validation for 'year'
 *
 * \retval  PMIC_UT_SUCCESS in case of success or
 *          PMIC_UT_FAILURE in case of failure.
 */
static bool test_pmic_rtc_setTimePrmValTest_year(void *pPmicCoreHandle)
{
    int32_t      status      = PMIC_ST_SUCCESS;
    Pmic_CoreHandle_t *handle = pPmicCoreHandle;
    Pmic_RtcTime_t timeCfg    = {0x0U, 30U, 30U, 6U, 0U, 1U};
    Pmic_RtcDate_t dateCfg    = {PMIC_RTC_DATE_CFG_WEEK_VALID_SHIFT, 15U, 6U,
                                 2055U, 1U};

    dateCfg.year              = PMIC_RTC_INVALID_YEAR;

    status = Pmic_rtcSetTimeDateInfo(handle, &timeCfg, &dateCfg);
    if(PMIC_ST_ERR_INV_DATE != status)
    {
        pmic_log("Failed %s with status: %d\n\t",__func__, status);
        return PMIC_UT_FAILURE;
    }

    return PMIC_UT_SUCCESS;
}

/*!
 * \brief   Parameter range validation for 'month'
 *
 * \retval  PMIC_UT_SUCCESS in case of success or
 *          PMIC_UT_FAILURE in case of failure.
 */
static bool test_pmic_rtc_setTimePrmValTest_month_range(void *pPmicCoreHandle)
{
    int32_t      status      = PMIC_ST_SUCCESS;
    Pmic_CoreHandle_t *handle = pPmicCoreHandle;
    Pmic_RtcTime_t timeCfg    = {0U, 30U, 30U, 6U, 0U, 1U};
    Pmic_RtcDate_t dateCfg    = {PMIC_RTC_DATE_CFG_MONTH_VALID_SHIFT, 15U, 6U,
                                 2055U, 1U};

    dateCfg.month             = PMIC_RTC_INVALID_MONTH;

    status = Pmic_rtcSetTimeDateInfo(handle, &timeCfg, &dateCfg);
    if(PMIC_ST_ERR_INV_DATE != status)
    {
        pmic_log("Failed %s with status: %d\n\t",__func__, status);
        return PMIC_UT_FAILURE;
    }

    return PMIC_UT_SUCCESS;
}

/*!
 * \brief   Parameter range validation for 'day'for months with 30 days
 *
 * \retval  PMIC_UT_SUCCESS in case of success or
 *          PMIC_UT_FAILURE in case of failure.
 */
static bool test_pmic_rtc_setTimePrmValTest_day_month(void *pPmicCoreHandle)
{
    int32_t status         = PMIC_ST_SUCCESS;
    Pmic_CoreHandle_t *handle = pPmicCoreHandle;
    Pmic_RtcTime_t timeCfg = {0x0, 30U, 30U, 6U, 0U, 1U};
    Pmic_RtcDate_t dateCfg = {PMIC_RTC_VALID_PARAM_DATE_CFG_DAY_MNTH_VAL, 15U,
                              6U, 2055U, 1U};

    dateCfg.month          = PMIC_RTC_MONTH_APR;
    dateCfg.day            = PMIC_RTC_INVALID_DAY_31;

    status = Pmic_rtcSetTimeDateInfo(handle, &timeCfg, &dateCfg);
    if(PMIC_ST_ERR_INV_DATE != status)
    {
        pmic_log("Failed %s with status: %d\n\t",__func__, status);
        return PMIC_UT_FAILURE;
    }

    return PMIC_UT_SUCCESS;
}

/*!
 * \brief   Parameter range validation for 'day' for leap
 *          year('year' %4 = 0 ) and 'month' = 2 (february)
 *
 * \retval  PMIC_UT_SUCCESS in case of success or
 *          PMIC_UT_FAILURE in case of failure.
 */
static bool test_pmic_rtc_setTimePrmValTest_feb_leapyear(void *pPmicCoreHandle)
{
    int32_t status         = PMIC_ST_SUCCESS;
    Pmic_CoreHandle_t *handle = pPmicCoreHandle;
    Pmic_RtcTime_t timeCfg = {0x0U, 30U, 30U, 6U, 0U, 1U};
    Pmic_RtcDate_t dateCfg = {PMIC_RTC_VALID_PARAM_DATE_CFG_DAY_MNTH_YR_VAL,
                              15U, 6U, 2055U, 1U};

    dateCfg.month          = PMIC_RTC_MONTH_FEB;
    dateCfg.year           = PMIC_RTC_YEAR_2044;
    dateCfg.day            = PMIC_RTC_INVALID_DAY_30;

    status = Pmic_rtcSetTimeDateInfo(handle, &timeCfg, &dateCfg);
    if(PMIC_ST_ERR_INV_DATE != status)
    {
        pmic_log("Failed %s with status: %d\n\t",__func__, status);
        return PMIC_UT_FAILURE;
    }

    return PMIC_UT_SUCCESS;
}

/*!
 * \brief   Parameter range validation for 'day' for Non-leap
 *          year('year' %4 != 0 ) and 'month' = 2 (february)
 *
 * \retval  PMIC_UT_SUCCESS in case of success or
 *          PMIC_UT_FAILURE in case of failure.
 */
static bool test_pmic_rtc_setTimePrmValTest_feb_nonleapyear
                                                    (void *pPmicCoreHandle)
{

    int32_t      status      = PMIC_ST_SUCCESS;
    Pmic_CoreHandle_t *handle = pPmicCoreHandle;
    Pmic_RtcTime_t timeCfg    = {0U, 30U, 30U, 6U, 0U, 1U};
    Pmic_RtcDate_t dateCfg    = {PMIC_RTC_VALID_PARAM_DATE_CFG_DAY_MNTH_YR_VAL,
                                 15U, 6U, 2055U, 1U};

    dateCfg.month             = PMIC_RTC_MONTH_FEB;
    dateCfg.year              = 2045U;
    dateCfg.day               = PMIC_RTC_INVALID_DAY;

    status = Pmic_rtcSetTimeDateInfo(handle, &timeCfg, &dateCfg);
    if(PMIC_ST_ERR_INV_DATE != status)
    {
        pmic_log("Failed %s with status: %d\n\t",__func__, status);
        return PMIC_UT_FAILURE;
    }

    return PMIC_UT_SUCCESS;
}

/*!
 * \brief   Parameter range validation for 'day', for months with 31 days
 *
 * \retval  PMIC_UT_SUCCESS in case of success or
 *          PMIC_UT_FAILURE in case of failure.
 */
static bool test_pmic_rtc_setTimePrmValTest_day_month31(void *pPmicCoreHandle)
{
    int32_t      status      = PMIC_ST_SUCCESS;
    Pmic_CoreHandle_t *handle = pPmicCoreHandle;
    Pmic_RtcTime_t timeCfg    = {0U, 30U, 30U, 6U, 0U, 1U};
    Pmic_RtcDate_t dateCfg    = {PMIC_RTC_VALID_PARAM_DATE_CFG_DAY_MNTH_VAL,
                                 15U, 6U, 2055U, 1U};

    dateCfg.month             = PMIC_RTC_MONTH_JUL;
    dateCfg.day               = PMIC_RTC_INVALID_DAY_32;

    status = Pmic_rtcSetTimeDateInfo(handle, &timeCfg, &dateCfg);
    if(PMIC_ST_ERR_INV_DATE != status)
    {
        pmic_log("Failed %s with status: %d\n\t",__func__, status);
        return PMIC_UT_FAILURE;
    }

    return PMIC_UT_SUCCESS;
}

/*!
 * \brief   Parameter validation for 'handle'
 *
 * \retval  PMIC_UT_SUCCESS in case of success or
 *          PMIC_UT_FAILURE in case of failure.
 */
static bool test_pmic_rtc_getTimePrmValTest_handle(void *pPmicCoreHandle)
{
    int32_t      status      = PMIC_ST_SUCCESS;
    Pmic_RtcTime_t timeCfg    = {PMIC_RTC_VALID_PARAM_TIME_CFG_VAL, 30U, 30U,
                                 6U, 0U, 1U};
    Pmic_RtcDate_t dateCfg    = {PMIC_RTC_VALID_PARAM_DATE_CFG_VAL, 15U, 6U,
                                 2055U, 1U};

    status = Pmic_rtcGetTimeDateInfo(NULL, &timeCfg, &dateCfg);
    if(PMIC_ST_ERR_INV_HANDLE != status)
    {
        pmic_log("Failed %s with status: %d\n\t",__func__, status);
        return PMIC_UT_FAILURE;
    }

    return PMIC_UT_SUCCESS;
}

/*!
 * \brief   Parameter validation for 'timeCfg'
 *
 * \retval  PMIC_UT_SUCCESS in case of success or
 *          PMIC_UT_FAILURE in case of failure.
 */
static bool test_pmic_rtc_getTimePrmValTest_timeCfg(void *pPmicCoreHandle)
{
    int32_t status         = PMIC_ST_SUCCESS;
    Pmic_CoreHandle_t *handle = pPmicCoreHandle;
    Pmic_RtcDate_t dateCfg = {PMIC_RTC_VALID_PARAM_DATE_CFG_VAL, 15U, 6U,
                              2055U, 1U};

    status = Pmic_rtcGetTimeDateInfo(handle, NULL, &dateCfg);
    if(PMIC_ST_ERR_NULL_PARAM != status)
    {
        pmic_log("Failed %s with status: %d\n\t",__func__, status);
        return PMIC_UT_FAILURE;
    }

    return PMIC_UT_SUCCESS;
}

/*!
 * \brief   Parameter validation for 'dataCfg'
 *
 * \retval  PMIC_UT_SUCCESS in case of success or
 *          PMIC_UT_FAILURE in case of failure.
 */
static bool test_pmic_rtc_getTimePrmValTest_dateCfg(void *pPmicCoreHandle)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_CoreHandle_t *handle = pPmicCoreHandle;
    Pmic_RtcTime_t timeCfg = {PMIC_RTC_VALID_PARAM_TIME_CFG_VAL, 30U, 30U, 6U,
                              0U, 1U};

    status = Pmic_rtcGetTimeDateInfo(handle, &timeCfg, NULL);
    if(PMIC_ST_ERR_NULL_PARAM != status)
    {
        pmic_log("Failed %s with status: %d\n\t",__func__, status);
        return PMIC_UT_FAILURE;
    }

    return PMIC_UT_SUCCESS;
}

/*!
 * \brief   Test RTC for set RTC frequency compensation
 *
 * \retval  PMIC_UT_SUCCESS in case of success or
 *          PMIC_UT_FAILURE in case of failure.
 */
static bool test_pmic_rtc_setFreqComp(void *pPmicCoreHandle)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_CoreHandle_t *handle = pPmicCoreHandle;
    uint16_t compensation = 34952;
    uint16_t compensation_rd = 0U;

    status = Pmic_rtcSetFreqComp(handle, compensation);
    if(PMIC_ST_SUCCESS != status)
    {
        pmic_log("Failed %s with status: %d\n\t",__func__, status);
        return PMIC_UT_FAILURE;
    }

    status  = Pmic_rtcGetFreqComp(handle, &compensation_rd);
    if(PMIC_ST_SUCCESS != status)
    {
        pmic_log("Failed %s with status: %d\n\t",__func__, status);
        return PMIC_UT_FAILURE;
    }

    if(compensation_rd != compensation)
    {
        return PMIC_UT_FAILURE;
    }

    return PMIC_UT_SUCCESS;
}

/*!
 * \brief   Parameter validation for 'handle'
 *
 * \retval  PMIC_UT_SUCCESS in case of success or
 *          PMIC_UT_FAILURE in case of failure.
 */
static bool test_pmic_rtc_setFreqCompPrmValTest_handle(void *pPmicCoreHandle)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint16_t compensation = 34952U;

    status = Pmic_rtcSetFreqComp(NULL, compensation);
    if(PMIC_ST_ERR_INV_HANDLE != status)
    {
        pmic_log("Failed %s with status: %d\n\t",__func__, status);
        return PMIC_UT_FAILURE;
    }

    return PMIC_UT_SUCCESS;
}

/*!
 * \brief   Test RTC for get RTC frequency compensation
 *
 * \retval  PMIC_UT_SUCCESS in case of success or
 *          PMIC_UT_FAILURE in case of failure.
 */
static bool test_pmic_rtc_getFreqComp(void *pPmicCoreHandle)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_CoreHandle_t *handle= pPmicCoreHandle;
    uint16_t compensation = 34952U;
    uint16_t compensation_rd;

    status = Pmic_rtcSetFreqComp(handle, compensation);
    if(PMIC_ST_SUCCESS != status)
    {
        pmic_log("Failed %s with status: %d\n\t",__func__, status);
        return PMIC_UT_FAILURE;
    }

    status  = Pmic_rtcGetFreqComp(handle, &compensation_rd);
    if(PMIC_ST_SUCCESS != status)
    {
        pmic_log("Failed %s with status: %d\n\t",__func__, status);
        return PMIC_UT_FAILURE;
    }

    if(compensation_rd != compensation)
    {
        return PMIC_UT_FAILURE;
    }

    return PMIC_UT_SUCCESS;
}

/*!
 * \brief   Parameter validation for 'handle'
 *
 * \retval  PMIC_UT_SUCCESS in case of success or
 *          PMIC_UT_FAILURE in case of failure.
 */
static bool test_pmic_rtc_getFreqCompPrmValTest_handle(void *pPmicCoreHandle)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint16_t compensation = 34952U;

    status = Pmic_rtcGetFreqComp(NULL, &compensation);
    if(PMIC_ST_ERR_INV_HANDLE != status)
    {
        pmic_log("Failed %s with status: %d\n\t",__func__, status);
        return PMIC_UT_FAILURE;
    }

    return PMIC_UT_SUCCESS;
}

/*!
 * \brief   Parameter validation for 'compensation'
 *
 * \retval  PMIC_UT_SUCCESS in case of success or
 *          PMIC_UT_FAILURE in case of failure.
 */
static bool test_pmic_rtc_getFreqCompPrmValTest_compensation
                                                    (void *pPmicCoreHandle)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_CoreHandle_t *handle = pPmicCoreHandle;

    status = Pmic_rtcGetFreqComp(handle, NULL);
    if(PMIC_ST_ERR_NULL_PARAM != status)
    {
        pmic_log("Failed %s with status: %d\n\t",__func__, status);
        return PMIC_UT_FAILURE;
    }

    return PMIC_UT_SUCCESS;
}

/*!
 * \brief   PMIC RTC Test Cases
 */
static Pmic_Ut_Tests_t pmic_rtc_tests[] =
{
    {
        test_pmic_rtc_alarmIntr,
        TID_7373_T01_01,
        "SetRtcAlarmIntr : Set RTC Alarm interrupt"
    },
    {
        test_pmic_rtc_setAlarmIntrPrmValTest_handle,
        TID_7373_T01_02,
        "SetRtcAlarmIntr : Parameter validation for 'handle'"
    },
    {
        test_pmic_rtc_setAlarmIntrPrmValTest_timeCfg,
        TID_7373_T01_03,
        "SetRtcAlarmIntr : Parameter validation for 'timeCfg'"
    },
    {
        test_pmic_rtc_setAlarmIntrPrmValTest_dateCfg,
        TID_7373_T01_04,
        "SetRtcAlarmIntr : Parameter validation for 'dataCfg'"
    },
    {
        test_pmic_rtc_setAlarmIntrPrmValTest_seconds,
        TID_7373_T01_05,
        "SetRtcAlarmIntr : Parameter validation for 'seconds'"
    },
    {
        test_pmic_rtc_setAlarmIntrPrmValTest_minutes,
        TID_7373_T01_06,
        "SetRtcAlarmIntr : Parameter validation for 'minutes'"
    },
    {
        test_pmic_rtc_setAlarmIntrPrmValTest_timeMode,
        TID_7373_T01_07,
        "SetRtcAlarmIntr : Parameter validation for 'timeMode'"
    },
    {
        test_pmic_rtc_setAlarmIntrPrmValTest_meridianMode,
        TID_7373_T01_08,
        "SetRtcAlarmIntr : Parameter validation for 'meridianMode'"
    },
    {
        test_pmic_rtc_setAlarmIntrPrmValTest_hour12,
        TID_7373_T01_09,
        "SetRtcAlarmIntr : Parameter validation for 'hour' when 'timeMode' = 1"
    },
    {
        test_pmic_rtc_setAlarmIntrPrmValTest_hour24,
        TID_7373_T01_10,
        "SetRtcAlarmIntr : Parameter validation for 'hour' when 'timeMode' = 0"
    },
    {
        test_pmic_rtc_setAlarmIntrPrmValTest_hour,
        TID_7373_T01_11,
        "SetRtcAlarmIntr : Negative test for 'hour ' = 0, when 'timeMode' = 1" },
    {
        test_pmic_rtc_setAlarmIntrPrmValTest_month,
        TID_7373_T01_12,
        "SetRtcAlarmIntr : Negative test for 'month ' = 0"
    },
    {
        test_pmic_rtc_setAlarmIntrPrmValTest_day,
        TID_7373_T01_13,
        "SetRtcAlarmIntr : Negative test for 'day' = 0"
    },
    {
        test_pmic_rtc_setAlarmIntrPrmValTest_year,
        TID_7373_T01_14,
        "SetRtcAlarmIntr : Parameter range validation for 'year'"
    },
    {
        test_pmic_rtc_setAlarmIntrPrmValTest_month_range,
        TID_7373_T01_15,
        "SetRtcAlarmIntr : Parameter range validation for 'month'"
    },
    {
        test_pmic_rtc_setAlarmIntrPrmValTest_day_month,
        TID_7373_T01_16,
        "SetRtcAlarmIntr : Parameter range validation for 'day' for months with 30 days"
    },
    {
        test_pmic_rtc_setAlarmIntrPrmValTest_feb_leapyear,
        TID_7373_T01_17,
        "SetRtcAlarmIntr : Parameter range validation for 'day' for leap year('year' %4 = 0 ) and 'month' = 2 (february)"
    },
    {
        test_pmic_rtc_setAlarmIntrPrmValTest_feb_nonleapyear,
        TID_7373_T01_18,
        "SetRtcAlarmIntr : Parameter range validation for 'day' for Non-leap year('year' %4 != 0 ) and 'month' = 2 (february)"
    },
    {
        test_pmic_rtc_setAlarmIntrPrmValTest_day_month31,
        TID_7373_T01_19,
        "SetRtcAlarmIntr : Parameter range validation for 'day', for months with 31 days)"
    },
    {
        test_pmic_rtc_alarmIntr,
        TID_7373_T01_20,
        "GetRtcAlarmIntr : Test Get RTC Alarm interrupt API"
    },
    {
        test_pmic_rtc_getAlarmIntrPrmValTest_handle,
        TID_7373_T01_21,
        "GetRtcAlarmIntr : Parameter validation for 'handle'"
    },
    {
        test_pmic_rtc_getAlarmIntrPrmValTest_timeCfg,
        TID_7373_T01_22,
        "GetRtcAlarmIntr : Parameter validation for 'timeCfg'"
    },
    {
        test_pmic_rtc_getAlarmIntrPrmValTest_dateCfg,
        TID_7373_T01_23,
        "GetRtcAlarmIntr : Parameter validation for 'dataCfg'"
    },
    {
        test_pmic_rtc_timer,
        TID_7373_T01_24,
        "SetRtcTimerIntr : Test Set RTC Timer interrupt"
    },
    {
        test_pmic_rtc_setTimerIntrPrmValTest_handle,
        TID_7373_T01_25,
        "SetRtcTimerIntr : Parameter validation for 'handle'"
    },
    {
        test_pmic_rtc_setTimerIntrPrmValTest_timerPeriod,
        TID_7373_T01_26,
        "SetRtcTimerIntr : Parameter validation for 'timerPeriod'"
    },
    {
        test_pmic_rtc_timer,
        TID_7373_T01_27,
        "GetRtcTimer : Test Get RTC Timer interrupt"
    },
    {
        test_pmic_rtc_getTimerIntrPrmValTest_handle,
        TID_7373_T01_28,
        "GetRtcTimer : Parameter validation for 'handle'"
    },
    {
        test_pmic_rtc_getTimerIntrPrmValTest_timerPeriod,
        TID_7373_T01_29,
        "GetRtcTimer : Parameter validation for 'timerPeriod'"
    },
    {
        test_pmic_rtc_disable,
        TID_7373_T01_30,
        "DisableRtc : Test RTC Disable"
    },
    {
        test_pmic_rtc_disablePrmValTest_handle,
        TID_7373_T01_31,
        "DisableRtc : Parameter validation for 'handle'"
    },
    {
        test_pmic_rtc_enable,
        TID_7373_T01_32,
        "EnableRtc : Test RTC Enable"
    },
    {
        test_pmic_rtc_enablePrmValTest_handle,
        TID_7373_T01_33,
        "EnableRtc : Parameter validation for 'handle'"
    },
    {
        test_pmic_rtc_time,
        TID_7373_T01_34,
        "SetRtc : Test RTC Set Time"
    },
    {
        test_pmic_rtc_setTimePrmValTest_handle,
        TID_7373_T01_35,
        "SetRtc : Parameter validation for 'handle'"
    },
    {
        test_pmic_rtc_setTimePrmValTest_timeCfg,
        TID_7373_T01_36,
        "SetRtc : Parameter validation for 'timeCfg'"
    },
    {
        test_pmic_rtc_setTimePrmValTest_dateCfg,
        TID_7373_T01_37,
        "SetRtc : Parameter validation for 'dataCfg'"
    },
    {
        test_pmic_rtc_setTimePrmValTest_seconds,
        TID_7373_T01_38,
        "SetRtc : Parameter validation for 'seconds'"
    },
    {
        test_pmic_rtc_setTimePrmValTest_minutes,
        TID_7373_T01_39,
        "SetRtc : Parameter validation for 'minutes'"
    },
    {
        test_pmic_rtc_setTimePrmValTest_timeMode,
        TID_7373_T01_40,
        "SetRtc : Parameter validation for 'timeMode'"
    },
    {
        test_pmic_rtc_setTimePrmValTest_meridianMode,
        TID_7373_T01_41,
        "SetRtc : Parameter validation for 'meridianMode'"
    },
    {
        test_pmic_rtc_setTimePrmValTest_hour12,
        TID_7373_T01_42,
        "SetRtc : Parameter validation for 'hour' when 'timeMode' = 1"
    },
    {
        test_pmic_rtc_setTimePrmValTest_hour24,
        TID_7373_T01_43,
        "SetRtc : Parameter validation for 'hour' when 'timeMode' = 0"
    },
    {
        test_pmic_rtc_setTimePrmValTest_hour,
        TID_7373_T01_44,
        "SetRtc : Negative test for 'hour ' = 0, when 'timeMode' = 1"
    },
    {
        test_pmic_rtc_setTimePrmValTest_month,
        TID_7373_T01_45,
        "SetRtc : Negative test for 'month ' = 0"
    },
    {
        test_pmic_rtc_setTimePrmValTest_day,
        TID_7373_T01_46,
        "SetRtc : Negative test for 'day' = 0"
    },
    {
        test_pmic_rtc_setTimePrmValTest_year,
        TID_7373_T01_47,
        "SetRtc : Parameter range validation for 'year'"
    },
    {
        test_pmic_rtc_setTimePrmValTest_month_range,
        TID_7373_T01_48,
        "SetRtc : Parameter range validation for 'month'"
    },
    {
       test_pmic_rtc_setTimePrmValTest_day_month,
       TID_7373_T01_49,
       "SetRtc : Parameter range validation for 'day' for months with 30 days"
    },
    {
        test_pmic_rtc_setTimePrmValTest_feb_leapyear,
        TID_7373_T01_50,
        "SetRtc : Parameter range validation for 'day' for leap year('year' %4 = 0 ) and 'month' = 2 (february)"
    },
    {
        test_pmic_rtc_setTimePrmValTest_feb_nonleapyear,
        TID_7373_T01_51,
        "SetRtc : Parameter range validation for 'day' for Non-leap year('year' %4 != 0 ) and 'month' = 2 (february)"
    },
    {
        test_pmic_rtc_setTimePrmValTest_day_month31,
        TID_7373_T01_52,
        "SetRtc : Parameter range validation for 'day', for months with 31 days)"
    },
    {
        test_pmic_rtc_time,
        TID_7373_T01_53,
        "GetRtc : Test RTC Get Time"
    },
    {
        test_pmic_rtc_getTimePrmValTest_handle,
        TID_7373_T01_54,
        "GetRtc : Parameter validation for 'handle'"
    },
    {
        test_pmic_rtc_getTimePrmValTest_timeCfg,
        TID_7373_T01_55,
        "GetRtc : Parameter validation for 'timeCfg'"
    },
    {
        test_pmic_rtc_getTimePrmValTest_dateCfg,
        TID_7373_T01_56,
        "GetRtc : Parameter validation for 'dataCfg'"
    },
    {
        test_pmic_rtc_setFreqComp,
        TID_7373_T01_57,
        "SetRtcFreqCompen : Test RTC set RTC frequency compensation"
    },
    {
        test_pmic_rtc_setFreqCompPrmValTest_handle,
        TID_7373_T01_58,
        "SetRtcFreqCompen : Parameter validation for 'handle'"
    },
    {
        test_pmic_rtc_getFreqComp,
        TID_7373_T01_59,
        "GetRtcFreqCompen : Test RTC get RTC frequency compensation"
    },
    {
        test_pmic_rtc_getFreqCompPrmValTest_handle,
        TID_7373_T01_60,
        "GetRtcFreqCompen : Parameter validation for 'handle'"
    },
    {
        test_pmic_rtc_getFreqCompPrmValTest_compensation,
        TID_7373_T01_61,
        "GetRtcFreqCompen : Parameter validation for 'compensation'"
    },
    {
        NULL,
    },
};

#if defined(UNITY_INCLUDE_CONFIG_H) && \
    (defined(SOC_J721E) || defined(SOC_J7200))

/*!
 * \brief   RTC Unity Test App wrapper Function
 */
static void test_pmic_rtc_testApp(void)
{
    bool testResult = PMIC_UT_FAILURE;
    Pmic_CoreCfg_t pmicConfigData = {0U};

     /* Fill parameters to pmicConfigData */
    pmicConfigData.pmicDeviceType      = PMIC_DEV_LEO_TPS6594;
    pmicConfigData.validParams        |= PMIC_CFG_DEVICE_TYPE_VALID_SHIFT;

    pmicConfigData.commMode            = PMIC_INTF_SINGLE_I2C;
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

    pmicConfigData.validParams         |= PMIC_CFG_COMM_HANDLE_VALID_SHIFT;

    pmicConfigData.validParams         |= (PMIC_CFG_COMM_HANDLE_VALID_SHIFT |
                                           PMIC_CFG_QASLAVEADDR_VALID_SHIFT);

    testResult = test_pmic_common(pmic_rtc_tests, &pmicConfigData);
    TEST_ASSERT(PMIC_UT_SUCCESS == testResult);

    pmic_log("\n All tests have passed. \n");

    while (true)
    {
    }
}

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
    pmic_log("%s(): %d: Begin Unity...\n", __func__, __LINE__);
    UNITY_BEGIN();

    RUN_TEST(test_pmic_rtc_testApp);

    UNITY_END();
    pmic_log("%s(): %d: End Unity...\n", __func__, __LINE__);
    /* Function to print results defined in unity_config.h file */
    print_unityOutputBuffer_usingUARTstdio();
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
    test_pmic_uartInit();

    pmic_log("PMIC RTC Unit Test started...\n");

    pmic_log("%s(): %d: %s(%s)\n", __func__, __LINE__, __TIME__, __DATE__);

#if defined(UNITY_INCLUDE_CONFIG_H) && \
    (defined(SOC_J721E)             || \
     defined(SOC_J7200))
    test_pmic_rtc_testapp_runner();
#endif
}
