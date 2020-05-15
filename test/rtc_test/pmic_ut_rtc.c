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
    Pmic_RtcDate_t dateCfg    = {PMIC_RTC_DATE_CFG_YEAR_VALID_SHIFT, 15U, 6U,
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
       (dateCfg.year         != dateCfg_rd.year)  ||
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
    Pmic_RtcDate_t dateCfg    = {PMIC_RTC_DATE_CFG_YEAR_VALID_SHIFT, 15U, 6U,
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
 * \brief   Mask or Unmask All interrupts
 *
 * \param   mask              [IN]    Parameter to mask/unmask all INTRs
 *                                    Valid values: \ref Pmic_IrqMaskFlag
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code.
 *          For valid values \ref Pmic_ErrorCodes
 */
static int32_t pmic_irqMaskAll(Pmic_CoreHandle_t *pHandle, bool mask)
{
    int32_t status = PMIC_ST_SUCCESS;

    /* Mask or Unmask all interrupts */
    status = Pmic_irqMaskIntr(pHandle, PMIC_IRQ_MASK_BUCK1_2_MASK, mask);
    if(PMIC_ST_SUCCESS == status)
    {
        status = Pmic_irqMaskIntr(pHandle, PMIC_IRQ_MASK_BUCK3_4_MASK, mask);
    }
    if(PMIC_ST_SUCCESS == status)
    {
        status = Pmic_irqMaskIntr(pHandle, PMIC_IRQ_MASK_BUCK5_MASK, mask);
    }
    if(PMIC_ST_SUCCESS == status)
    {
        status = Pmic_irqMaskIntr(pHandle, PMIC_IRQ_MASK_LDO1_2_MASK, mask);
    }
    if(PMIC_ST_SUCCESS == status)
    {
        status = Pmic_irqMaskIntr(pHandle, PMIC_IRQ_MASK_LDO3_4_MASK, mask);
    }
    if(PMIC_ST_SUCCESS == status)
    {
        status = Pmic_irqMaskIntr(pHandle, PMIC_IRQ_MASK_VMON_MASK, mask);
    }
    if(PMIC_ST_SUCCESS == status)
    {
        status = Pmic_irqMaskIntr(pHandle, PMIC_IRQ_MASK_GPIO1_8_FALL_MASK,
                                                                    mask);
    }
    if(PMIC_ST_SUCCESS == status)
    {
        status = Pmic_irqMaskIntr(pHandle, PMIC_IRQ_MASK_GPIO1_8_RISE_MASK,
                                                                    mask);
    }
    if(PMIC_ST_SUCCESS == status)
    {
        status = Pmic_irqMaskIntr(pHandle, PMIC_IRQ_MASK_GPIO9_11_MASK, mask);
    }
    if(PMIC_ST_SUCCESS == status)
    {
        status = Pmic_irqMaskIntr(pHandle, PMIC_IRQ_MASK_MISC_MASK, mask);
    }
    if(PMIC_ST_SUCCESS == status)
    {
        status = Pmic_irqMaskIntr(pHandle, PMIC_IRQ_MASK_MODERATE_ERR_MASK,
                                                                    mask);
    }
    if(PMIC_ST_SUCCESS == status)
    {
        status = Pmic_irqMaskIntr(pHandle, PMIC_IRQ_MASK_FSM_ERR_MASK, mask);
    }
    if(PMIC_ST_SUCCESS == status)
    {
        status = Pmic_irqMaskIntr(pHandle, PMIC_IRQ_MASK_COMM_ERR_MASK, mask);
    }
    if(PMIC_ST_SUCCESS == status)
    {
        status = Pmic_irqMaskIntr(pHandle, PMIC_IRQ_MASK_READBACK_ERR_MASK,
                                                                      mask);
    }
    if(PMIC_ST_SUCCESS == status)
    {
        status = Pmic_irqMaskIntr(pHandle, PMIC_IRQ_MASK_ESM_MASK, mask);
    }
    if(PMIC_ST_SUCCESS == status)
    {
        status = Pmic_irqMaskIntr(pHandle, PMIC_IRQ_MASK_STARTUP_MASK, mask);
    }
    if(PMIC_ST_SUCCESS == status)
    {
        if(mask)
        {
            status = Pmic_rtcEnableAlarmIntr(pHandle,
                                             PMIC_RTC_ALARM_INTR_DISABLE);
            if(PMIC_ST_SUCCESS == status)
            {
                status = Pmic_rtcEnableTimerIntr(pHandle,
                                                 PMIC_RTC_TIMER_INTR_DISABLE);
            }
        }
        else
        {
            status = Pmic_rtcEnableAlarmIntr(pHandle,
                                             PMIC_RTC_ALARM_INTR_ENABLE);
            if(PMIC_ST_SUCCESS == status)
            {
                status = Pmic_rtcEnableTimerIntr(pHandle,
                                                 PMIC_RTC_TIMER_INTR_ENABLE);
            }
       }
    }

    return status;
}

/*
 * FIXME: Below test case is causing reset to J721 EVM.
 *        TO avoid reset used dalay as workaround for the testcase
 */
/*!
 * \brief   RTC time interrupt
 *
 * \retval  PMIC_UT_SUCCESS in case of success or
 *          PMIC_UT_FAILURE in case of failure.
 */
static bool test_rtc_timer_irq(void *pPmicCoreHandle)
{
    int32_t            status       = PMIC_ST_SUCCESS;
    int8_t             timeout      = 10U;
    Pmic_CoreHandle_t  *pHandle     = NULL;
    Pmic_RtcTime_t     timeCfg_cr   = { 0x1F, 0U, 0U, 0U, 0U, 0U};
    Pmic_RtcDate_t     dateCfg_cr   = { 0x0F, 0U, 0U, 0U, 0U};
    Pmic_RtcTime_t     timeCfg_rd   = { 0x1F, 0U, 0U, 0U, 0U, 0U};
    Pmic_RtcDate_t     dateCfg_rd   = { 0x0F, 0U, 0U, 0U, 0U};
    uint8_t            clearIRQ     = 1U;
    uint32_t           pErrStat     = 0U;
    uint32_t           errBitStatus = 0U;
    bool               tstStatus    = PMIC_UT_FAILURE;
    uint8_t timerPeriod = 0U;

    Pmic_RtcDate_t    validDateCfg =  { 0x0F, 15U, 6U, 2055U, 1U};
    Pmic_RtcTime_t    validTimeCfg  = { 0x1F, 30U, 30U, 6U, 0U, 1U};

    pHandle                         = pPmicCoreHandle;

    status = Pmic_rtcSetTimeDateInfo(pHandle, &validTimeCfg, &validDateCfg);
    if(status != PMIC_ST_SUCCESS)
    {
        return PMIC_UT_FAILURE;
    }

    /* MASKING all Interrupts */
    status = pmic_irqMaskAll(pHandle, 1U);
    if(status != PMIC_ST_SUCCESS)
    {
        return PMIC_UT_FAILURE;
    }

    if(PMIC_ST_SUCCESS == status)
    {
        status = Pmic_rtcGetTimeDateInfo(pHandle, &timeCfg_rd, &dateCfg_rd);
        if(PMIC_ST_SUCCESS != status)
        {
            pmic_log("%d Failed %s with status: %d\n\t",__LINE__,__func__,
                                                        status);
            return PMIC_UT_FAILURE;
        }
    }

    if(PMIC_ST_SUCCESS == status)
    {
        status = Pmic_rtcGetTimerIntr(pHandle, &timerPeriod);
    }

    if(PMIC_ST_SUCCESS == status)
    {        status = Pmic_rtcSetTimerIntr(pHandle,
                                           PMIC_RTC_SECOND_INTR_PERIOD);
        if(PMIC_ST_SUCCESS != status)
        {
            pmic_log("%d Failed %s with status: %d\n\t",__LINE__,__func__,
                                                        status);
            return PMIC_UT_FAILURE;
        }
    }

    status = Pmic_rtcEnableTimerIntr(pHandle, PMIC_RTC_TIMER_INTR_ENABLE);

    while(timeout--)
    {
        /* Added delay as workaround to avoid reset on J721 EVM */
        Osal_delay(5000U);
        status = Pmic_irqGetErrStatus(pHandle, &pErrStat, clearIRQ);
        if(PMIC_ST_SUCCESS == status)
        {
            /* Extract offsets and error code */
            errBitStatus = PMIC_IRQID_BITMASK (pErrStat);

            if(PMIC_INT_RTC_STATUS_TIMER_MASK == errBitStatus)
            {
                /* Disable the timer interrupt  */
                status = Pmic_rtcEnableTimerIntr(pHandle,
                                                 PMIC_RTC_TIMER_INTR_DISABLE);

                tstStatus = PMIC_UT_SUCCESS;
                break;
            }
        }
        status = Pmic_rtcGetTimeDateInfo(pHandle, &timeCfg_cr, &dateCfg_cr);
        if(PMIC_ST_SUCCESS != status)
        {
            tstStatus = PMIC_UT_FAILURE;
        }
    }

    /* UNMASKING all Interrupts */
    status = pmic_irqMaskAll(pHandle, 0U);
    if(status != PMIC_ST_SUCCESS)
    {
        return PMIC_UT_FAILURE;
    }

    status = Pmic_rtcSetTimerIntr(pHandle, timerPeriod);
    return tstStatus;
}

/*
 * FIXME: Below test case is causing reset to J721 EVM.
 *        TO avoid reset used dalay as workaround for the testcase
 */
/*!
 * \brief   RTC Alarm interrupt
 *
 * \retval  PMIC_UT_SUCCESS in case of success or
 *          PMIC_UT_FAILURE in case of failure.
 */
static bool test_rtc_alarm_irq(void *pPmicCoreHandle)
{
    int32_t            status       = PMIC_ST_SUCCESS;
    Pmic_CoreHandle_t  *pHandle     = NULL;
    pHandle                         = pPmicCoreHandle;
    Pmic_RtcTime_t     timeCfg_cr   = { 0x1F, 0U, 0U, 0U, 0U, 0U};
    Pmic_RtcDate_t     dateCfg_cr   = { 0x0F, 0U, 0U, 0U, 0U};
    Pmic_RtcTime_t     timeCfg_rd   = { 0x1F, 0U, 0U, 0U, 0U, 0U};
    Pmic_RtcDate_t     dateCfg_rd   = { 0x0F, 0U, 0U, 0U, 0U};

    uint8_t            clearIRQ     = 1U;
    uint32_t           errBitStatus = 0U;
    uint32_t           pErrStat     = 0U;
    bool               tstStatus    = PMIC_UT_FAILURE;
    int8_t             timeout      = 10U;

    Pmic_RtcDate_t    validDateCfg =  { 0x0F, 15U, 6U, 2055U, 1U};
    Pmic_RtcTime_t    validTimeCfg  = { 0x1F, 30U, 30U, 6U, 0U, 1U};

    status = Pmic_rtcSetTimeDateInfo(pHandle, &validTimeCfg, &validDateCfg);
    if(status != PMIC_ST_SUCCESS)
    {
        return PMIC_UT_FAILURE;
    }

    /* MASKING all Interrupts */
    status = pmic_irqMaskAll(pHandle, 1U);
    if(status != PMIC_ST_SUCCESS)
    {
        return PMIC_UT_FAILURE;
    }

    if(PMIC_ST_SUCCESS == status)
    {
        /* Get the current time value */
        status = Pmic_rtcGetTimeDateInfo(pHandle, &timeCfg_rd, &dateCfg_rd);
    }

    if(PMIC_ST_SUCCESS == status)
    {
        timeCfg_rd.seconds = timeCfg_rd.seconds + 3U;
        status   = Pmic_rtcSetAlarmIntr(pHandle, &timeCfg_rd, &dateCfg_rd);
    }

    if(PMIC_ST_SUCCESS == status)
    {
        /* Get the current time for timeout */
        status = Pmic_rtcGetTimeDateInfo(pHandle, &timeCfg_cr, &dateCfg_cr);
    }

    if(PMIC_ST_SUCCESS == status)
    {
        while(timeout--)
        {
            /* Added delay as workaround to avoid reset on J721 EVM */
            Osal_delay(10000U);
            status = Pmic_irqGetErrStatus(pHandle, &pErrStat, clearIRQ);
            if(PMIC_ST_SUCCESS == status)
            {
                /* Extract Level 1, Level 2 register offsets and error code */
                errBitStatus   = PMIC_IRQID_BITMASK (pErrStat);

                if(PMIC_INT_RTC_STATUS_ALARM_MASK == errBitStatus)
                {
                    /* Interrupt received */
                    /* Disable the alarm interrupt */
                    status = Pmic_rtcEnableAlarmIntr(pHandle,
                                                  PMIC_RTC_ALARM_INTR_DISABLE);

                    /* clear the interrupt */
                    if(PMIC_ST_SUCCESS == status)
                    {
                        tstStatus = PMIC_UT_SUCCESS;
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

    /* UNMASKING all Interrupts */
    status = pmic_irqMaskAll(pHandle, 0U);
    if(status != PMIC_ST_SUCCESS)
    {
        return PMIC_UT_FAILURE;
    }

    if(status != PMIC_ST_SUCCESS)
    {
        return PMIC_UT_FAILURE;
    }

    return tstStatus;
}

/*!
 * \brief   Parameter validation for 'handle'
 *
 * \retval  PMIC_UT_SUCCESS in case of success or
 *          PMIC_UT_FAILURE in case of failure.
 */
static bool test_rtc_enable_alarm_interrupt_test_handle(void *pPmicCoreHandle)
{
    int32_t status = PMIC_ST_SUCCESS;

    status = Pmic_rtcEnableAlarmIntr(NULL, PMIC_RTC_ALARM_INTR_DISABLE);
    if(PMIC_ST_ERR_INV_HANDLE != status)
    {
        pmic_log("%d Failed %s with status: %d\n\t",__LINE__,__func__, status);
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
static bool test_rtc_enable_timer_interrupt_test_handle(void *pPmicCoreHandle)
{
    int32_t status = PMIC_ST_SUCCESS;

    status = Pmic_rtcEnableTimerIntr(NULL, PMIC_RTC_TIMER_INTR_DISABLE);
    if(PMIC_ST_ERR_INV_HANDLE != status)
    {
        pmic_log("%d Failed %s with status: %d\n\t",__LINE__,__func__, status);
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
        5990,
        "SetRtcAlarmIntr : Set RTC Alarm interrupt"
    },
    {
        test_pmic_rtc_setAlarmIntrPrmValTest_handle,
        5991,
        "SetRtcAlarmIntr : Parameter validation for 'handle'"
    },
    {
        test_pmic_rtc_setAlarmIntrPrmValTest_timeCfg,
        6087,
        "SetRtcAlarmIntr : Parameter validation for 'timeCfg'"
    },
    {
        test_pmic_rtc_setAlarmIntrPrmValTest_dateCfg,
        6088,
        "SetRtcAlarmIntr : Parameter validation for 'dataCfg'"
    },
    {
        test_pmic_rtc_setAlarmIntrPrmValTest_seconds,
        6090,
        "SetRtcAlarmIntr : Parameter validation for 'seconds'"
    },
    {
        test_pmic_rtc_setAlarmIntrPrmValTest_minutes,
        6091,
        "SetRtcAlarmIntr : Parameter validation for 'minutes'"
    },
    {
        test_pmic_rtc_setAlarmIntrPrmValTest_timeMode,
        6092,
        "SetRtcAlarmIntr : Parameter validation for 'timeMode'"
    },
    {
        test_pmic_rtc_setAlarmIntrPrmValTest_meridianMode,
        6093,
        "SetRtcAlarmIntr : Parameter validation for 'meridianMode'"
    },
    {
        test_pmic_rtc_setAlarmIntrPrmValTest_hour12,
        6094,
        "SetRtcAlarmIntr : Parameter validation for 'hour' when 'timeMode' = 1"
    },
    {
        test_pmic_rtc_setAlarmIntrPrmValTest_hour24,
        6095,
        "SetRtcAlarmIntr : Parameter validation for 'hour' when 'timeMode' = 0"
    },
    {
        test_pmic_rtc_setAlarmIntrPrmValTest_hour,
        6096,
        "SetRtcAlarmIntr : Negative test for 'hour ' = 0, when 'timeMode' = 1" },
    {
        test_pmic_rtc_setAlarmIntrPrmValTest_month,
        6099,
        "SetRtcAlarmIntr : Negative test for 'month ' = 0"
    },
    {
        test_pmic_rtc_setAlarmIntrPrmValTest_day,
        6100,
        "SetRtcAlarmIntr : Negative test for 'day' = 0"
    },
    {
        test_pmic_rtc_setAlarmIntrPrmValTest_year,
        6101,
        "SetRtcAlarmIntr : Parameter range validation for 'year'"
    },
    {
        test_pmic_rtc_setAlarmIntrPrmValTest_month_range,
        6102,
        "SetRtcAlarmIntr : Parameter range validation for 'month'"
    },
    {
        test_pmic_rtc_setAlarmIntrPrmValTest_day_month,
        6103,
        "SetRtcAlarmIntr : Parameter range validation for 'day' for months with 30 days"
    },
    {
        test_pmic_rtc_setAlarmIntrPrmValTest_feb_leapyear,
        6104,
        "SetRtcAlarmIntr : Parameter range validation for 'day' for leap year('year' %4 = 0 ) and 'month' = 2 (february)"
    },
    {
        test_pmic_rtc_setAlarmIntrPrmValTest_feb_nonleapyear,
        6107,
        "SetRtcAlarmIntr : Parameter range validation for 'day' for Non-leap year('year' %4 != 0 ) and 'month' = 2 (february)"
    },
    {
        test_pmic_rtc_setAlarmIntrPrmValTest_day_month31,
        6108,
        "SetRtcAlarmIntr : Parameter range validation for 'day', for months with 31 days)"
    },
    {
        test_pmic_rtc_alarmIntr,
        6109,
        "GetRtcAlarmIntr : Test Get RTC Alarm interrupt API"
    },
    {
        test_pmic_rtc_getAlarmIntrPrmValTest_handle,
        6110,
        "GetRtcAlarmIntr : Parameter validation for 'handle'"
    },
    {
        test_pmic_rtc_getAlarmIntrPrmValTest_timeCfg,
        6111,
        "GetRtcAlarmIntr : Parameter validation for 'timeCfg'"
    },
    {
        test_pmic_rtc_getAlarmIntrPrmValTest_dateCfg,
        6112,
        "GetRtcAlarmIntr : Parameter validation for 'dataCfg'"
    },
    {
        test_pmic_rtc_timer,
        6113,
        "SetRtcTimerIntr : Test Set RTC Timer interrupt"
    },
    {
        test_pmic_rtc_setTimerIntrPrmValTest_handle,
        6114,
        "SetRtcTimerIntr : Parameter validation for 'handle'"
    },
    {
        test_pmic_rtc_setTimerIntrPrmValTest_timerPeriod,
        6115,
        "SetRtcTimerIntr : Parameter validation for 'timerPeriod'"
    },
    {
        test_pmic_rtc_timer,
        6116,
        "GetRtcTimer : Test Get RTC Timer interrupt"
    },
    {
        test_pmic_rtc_getTimerIntrPrmValTest_handle,
        6117,
        "GetRtcTimer : Parameter validation for 'handle'"
    },
    {
        test_pmic_rtc_getTimerIntrPrmValTest_timerPeriod,
        6118,
        "GetRtcTimer : Parameter validation for 'timerPeriod'"
    },
    {
        test_pmic_rtc_disable,
        6119,
        "DisableRtc : Test RTC Disable"
    },
    {
        test_pmic_rtc_disablePrmValTest_handle,
        6120,
        "DisableRtc : Parameter validation for 'handle'"
    },
    {
        test_pmic_rtc_enable,
        6121,
        "EnableRtc : Test RTC Enable"
    },
    {
        test_pmic_rtc_enablePrmValTest_handle,
        6122,
        "EnableRtc : Parameter validation for 'handle'"
    },
    {
        test_pmic_rtc_time,
        6158,
        "SetRtc : Test RTC Set Time"
    },
    {
        test_pmic_rtc_setTimePrmValTest_handle,
        6159,
        "SetRtc : Parameter validation for 'handle'"
    },
    {
        test_pmic_rtc_setTimePrmValTest_timeCfg,
        6160,
        "SetRtc : Parameter validation for 'timeCfg'"
    },
    {
        test_pmic_rtc_setTimePrmValTest_dateCfg,
        6161,
        "SetRtc : Parameter validation for 'dataCfg'"
    },
    {
        test_pmic_rtc_setTimePrmValTest_seconds,
        6162,
        "SetRtc : Parameter validation for 'seconds'"
    },
    {
        test_pmic_rtc_setTimePrmValTest_minutes,
        6163,
        "SetRtc : Parameter validation for 'minutes'"
    },
    {
        test_pmic_rtc_setTimePrmValTest_timeMode,
        6164,
        "SetRtc : Parameter validation for 'timeMode'"
    },
    {
        test_pmic_rtc_setTimePrmValTest_meridianMode,
        6165,
        "SetRtc : Parameter validation for 'meridianMode'"
    },
    {
        test_pmic_rtc_setTimePrmValTest_hour12,
        6166,
        "SetRtc : Parameter validation for 'hour' when 'timeMode' = 1"
    },
    {
        test_pmic_rtc_setTimePrmValTest_hour24,
        6167,
        "SetRtc : Parameter validation for 'hour' when 'timeMode' = 0"
    },
    {
        test_pmic_rtc_setTimePrmValTest_hour,
        6168,
        "SetRtc : Negative test for 'hour ' = 0, when 'timeMode' = 1"
    },
    {
        test_pmic_rtc_setTimePrmValTest_month,
        6169,
        "SetRtc : Negative test for 'month ' = 0"
    },
    {
        test_pmic_rtc_setTimePrmValTest_day,
        6290,
        "SetRtc : Negative test for 'day' = 0"
    },
    {
        test_pmic_rtc_setTimePrmValTest_year,
        6170,
        "SetRtc : Parameter range validation for 'year'"
    },
    {
        test_pmic_rtc_setTimePrmValTest_month_range,
        6171,
        "SetRtc : Parameter range validation for 'month'"
    },
    {
       test_pmic_rtc_setTimePrmValTest_day_month,
       6172,
       "SetRtc : Parameter range validation for 'day' for months with 30 days"
    },
    {
        test_pmic_rtc_setTimePrmValTest_feb_leapyear,
        6173,
        "SetRtc : Parameter range validation for 'day' for leap year('year' %4 = 0 ) and 'month' = 2 (february)"
    },
    {
        test_pmic_rtc_setTimePrmValTest_feb_nonleapyear,
        6174,
        "SetRtc : Parameter range validation for 'day' for Non-leap year('year' %4 != 0 ) and 'month' = 2 (february)"
    },
    {
        test_pmic_rtc_setTimePrmValTest_day_month31,
        6175,
        "SetRtc : Parameter range validation for 'day', for months with 31 days)"
    },
    {
        test_pmic_rtc_time,
        6176,
        "GetRtc : Test RTC Get Time"
    },
    {
        test_pmic_rtc_getTimePrmValTest_handle,
        6177,
        "GetRtc : Parameter validation for 'handle'"
    },
    {
        test_pmic_rtc_getTimePrmValTest_timeCfg,
        6178,
        "GetRtc : Parameter validation for 'timeCfg'"
    },
    {
        test_pmic_rtc_getTimePrmValTest_dateCfg,
        6179,
        "GetRtc : Parameter validation for 'dataCfg'"
    },
    {
        test_pmic_rtc_setFreqComp,
        6180,
        "SetRtcFreqCompen : Test RTC set RTC frequency compensation"
    },
    {
        test_pmic_rtc_setFreqCompPrmValTest_handle,
        6181,
        "SetRtcFreqCompen : Parameter validation for 'handle'"
    },
    {
        test_pmic_rtc_getFreqComp,
        6182,
        "GetRtcFreqCompen : Test RTC get RTC frequency compensation"
    },
    {
        test_pmic_rtc_getFreqCompPrmValTest_handle,
        6183,
        "GetRtcFreqCompen : Parameter validation for 'handle'"
    },
    {
        test_pmic_rtc_getFreqCompPrmValTest_compensation,
        6287,
        "GetRtcFreqCompen : Parameter validation for 'compensation'"
    },
/*
 * FIXME: Below test case is causing reset to J721 EVM.
 *        TO avoid reset used dalay as workaround for the testcase
 */
    {
        test_rtc_timer_irq,
        1111,/* Dummy */
        "\r\n test_rtc_timer_irq   :    Test rtc timer interrupt"
    },
/*
 * FIXME: Below test case is causing reset to J721 EVM.
 *        TO avoid reset used dalay as workaround for the testcase
 */
    {
        test_rtc_alarm_irq,
        2222,/* Dummy */
        "\r\n test_rtc_alarm_irq   :    Test rtc alarm interrupt"
    },
    {
        test_rtc_enable_timer_interrupt_test_handle,
        3333,/* Dummy */
        "\r\n Pmic_rtcEnableTimerIntr   :    Parameter validation for 'handle'"
    },
    {
        test_rtc_enable_alarm_interrupt_test_handle,
        4444,/* Dummy */
        "\r\n Pmic_rtcEnableAlarmIntr   :    Parameter validation for 'handle'"
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
