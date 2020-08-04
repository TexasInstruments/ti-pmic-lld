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
 *  \file   pmic_rtc.c
 *
 *  \brief  This file contains APIs definitions for PMIC Real Time Clock(RTC)
 *          to set/get time calendar register, set/get alarm/timer interrupt
 *          and to enable/disable RTC.
 */
#include <pmic_rtc.h>
#include <pmic_core_priv.h>
#include <pmic_rtc_priv.h>

/*!
 * \brief   This function is used to set the RTC/Alarm seconds.
 */
static int32_t Pmic_rtcSetSeconds(Pmic_CoreHandle_t    *pPmicCoreHandle,
                                  const Pmic_RtcTime_t  timeCfg,
                                  bool                  operation_type)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData    = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);

    /* Operation for Alarm */
    if(PMIC_RTC_OPS_FOR_ALARM == operation_type)
    {
        /* Writing seconds to PMIC */
        HW_REG_SET_FIELD(regData,
                         PMIC_ALARM_SECONDS_ALR_SECOND_1,
                         (timeCfg.seconds /
                          PMIC_RTC_CONVERT_4BIT_MSB_TO_DEC));
        HW_REG_SET_FIELD(regData,
                         PMIC_ALARM_SECONDS_ALR_SECOND_0,
                         (timeCfg.seconds %
                          PMIC_RTC_CONVERT_4BIT_MSB_TO_DEC));

        pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                            PMIC_ALARM_SECONDS_REGADDR,
                                            regData);
    }
    /* Operation for RTC */
    else
    {
        /* Writing seconds to PMIC */
        HW_REG_SET_FIELD(regData,
                         PMIC_RTC_SECONDS_SECOND_1,
                         (timeCfg.seconds /
                          PMIC_RTC_CONVERT_4BIT_MSB_TO_DEC));
        HW_REG_SET_FIELD(regData,
                         PMIC_RTC_SECONDS_SECOND_0,
                         (timeCfg.seconds %
                          PMIC_RTC_CONVERT_4BIT_MSB_TO_DEC));

        pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                            PMIC_RTC_SECONDS_REGADDR,
                                            regData);
    }

    Pmic_criticalSectionStop(pPmicCoreHandle);

    return pmicStatus;
}

/*!
 * \brief   This function is used to get the RTC/Alarm Seconds.
 */
static int32_t Pmic_rtcGetSeconds(Pmic_CoreHandle_t *pPmicCoreHandle,
                                  Pmic_RtcTime_t    *pTimeCfg,
                                  bool               operation_type)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData    = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);

    /* Operation for Alarm */
    if(PMIC_RTC_OPS_FOR_ALARM == operation_type)
    {
        /* Reading the seconds */
        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                            PMIC_ALARM_SECONDS_REGADDR,
                                            &regData);
    }
    /* Operation for RTC */
    else
    {
        /* Reading the seconds */
        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                            PMIC_RTC_SECONDS_REGADDR,
                                            &regData);
    }

    Pmic_criticalSectionStop(pPmicCoreHandle);

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        /* Operation for Alarm */
        if(PMIC_RTC_OPS_FOR_ALARM == operation_type)
        {
            /* Update Seconds to pTimeCfg */
            pTimeCfg->seconds = PMIC_RTC_CONVERT_4BIT_MSB_TO_DEC *
                                (HW_REG_GET_FIELD(regData,
                                            PMIC_ALARM_SECONDS_ALR_SECOND_1));
            pTimeCfg->seconds = pTimeCfg->seconds +
                                (HW_REG_GET_FIELD(regData,
                                            PMIC_ALARM_SECONDS_ALR_SECOND_0));
        }
        else
        {
            /* Update Seconds to pTimeCfg */
            pTimeCfg->seconds = PMIC_RTC_CONVERT_4BIT_MSB_TO_DEC *
                                (HW_REG_GET_FIELD(regData,
                                                  PMIC_RTC_SECONDS_SECOND_1));
            pTimeCfg->seconds = pTimeCfg->seconds +
                                (HW_REG_GET_FIELD(regData,
                                                  PMIC_RTC_SECONDS_SECOND_0));
        }
    }

    return pmicStatus;
}

/*!
 * \brief   This function is used to set the RTC/Alarm Minutes.
 */
static int32_t Pmic_rtcSetMinutes(Pmic_CoreHandle_t    *pPmicCoreHandle,
                                  const Pmic_RtcTime_t  timeCfg,
                                  bool                  operation_type)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData    = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);

    /* Operation for Alarm */
    if(PMIC_RTC_OPS_FOR_ALARM == operation_type)
    {
        /* Writing minutes to PMIC */
        HW_REG_SET_FIELD(regData,
                         PMIC_ALARM_MINUTES_ALR_MINUTE_1,
                         (timeCfg.minutes /
                          PMIC_RTC_CONVERT_4BIT_MSB_TO_DEC));
        HW_REG_SET_FIELD(regData,
                         PMIC_ALARM_MINUTES_ALR_MINUTE_0,
                         (timeCfg.minutes %
                          PMIC_RTC_CONVERT_4BIT_MSB_TO_DEC));
        pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                            PMIC_ALARM_MINUTES_REGADDR,
                                            regData);
    }
    /* Operation for RTC */
    else
    {
        /* Writing minutes to PMIC */
        HW_REG_SET_FIELD(regData,
                         PMIC_RTC_MINUTES_MINUTE_1,
                         (timeCfg.minutes /
                          PMIC_RTC_CONVERT_4BIT_MSB_TO_DEC));
        HW_REG_SET_FIELD(regData,
                         PMIC_RTC_MINUTES_MINUTE_0,
                         (timeCfg.minutes %
                          PMIC_RTC_CONVERT_4BIT_MSB_TO_DEC));
        pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                            PMIC_RTC_MINUTES_REGADDR,
                                            regData);
    }

    Pmic_criticalSectionStop(pPmicCoreHandle);

    return pmicStatus;
}

/*!
 * \brief   This function is used to get the RTC/Alarm Minutes.
 */
static int32_t Pmic_rtcGetMinutes(Pmic_CoreHandle_t *pPmicCoreHandle,
                                  Pmic_RtcTime_t    *pTimeCfg,
                                  bool               operation_type)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData    = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);

    /* Operation for Alarm */
    if(PMIC_RTC_OPS_FOR_ALARM == operation_type)
    {
        /* Reading the minutes */
        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                            PMIC_ALARM_MINUTES_REGADDR,
                                            &regData);
    }
    /* Operation for RTC */
    else
    {
        /* Reading the minutes */
        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                            PMIC_RTC_MINUTES_REGADDR,
                                            &regData);
    }

    Pmic_criticalSectionStop(pPmicCoreHandle);

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        /* Operation for Alarm */
        if(PMIC_RTC_OPS_FOR_ALARM == operation_type)
        {
            /* Update Minutes to pTimeCfg */
            pTimeCfg->minutes = PMIC_RTC_CONVERT_4BIT_MSB_TO_DEC *
                                (HW_REG_GET_FIELD(regData,
                                            PMIC_ALARM_MINUTES_ALR_MINUTE_1));
            pTimeCfg->minutes = pTimeCfg->minutes +
                                (HW_REG_GET_FIELD(regData,
                                            PMIC_ALARM_MINUTES_ALR_MINUTE_0));
        }
        else
        {
            /* Update Minutes to pTimeCfg */
            pTimeCfg->minutes = PMIC_RTC_CONVERT_4BIT_MSB_TO_DEC *
                                (HW_REG_GET_FIELD(regData,
                                                 PMIC_RTC_MINUTES_MINUTE_1));
            pTimeCfg->minutes = pTimeCfg->minutes +
                                (HW_REG_GET_FIELD(regData,
                                                  PMIC_RTC_MINUTES_MINUTE_0));
        }
    }

    return pmicStatus;
}

/*!
 * \brief   This function is used to set the RTC/Alarm TimeMode.
 */
static int32_t Pmic_rtcSetTimeMode(Pmic_CoreHandle_t    *pPmicCoreHandle,
                                   const Pmic_RtcTime_t  timeCfg)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData    = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);

    /* Setting hour mode to PMIC */
    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        PMIC_RTC_CTRL_1_REGADDR,
                                        &regData);
    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        HW_REG_SET_FIELD(regData,
                         PMIC_RTC_CTRL_1_MODE_12_24,
                         timeCfg.timeMode);
        pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                            PMIC_RTC_CTRL_1_REGADDR,
                                            regData);
    }

    Pmic_criticalSectionStop(pPmicCoreHandle);

    return pmicStatus;
}

/*!
 * \brief   This function is used to get the RTC/Alarm TimeMode.
 */
static int32_t Pmic_rtcGetTimeMode(Pmic_CoreHandle_t *pPmicCoreHandle,
                                   Pmic_RtcTime_t    *pTimeCfg)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData    = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);

    /* Getting the hour mode */
    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        PMIC_RTC_CTRL_1_REGADDR,
                                        &regData);

    Pmic_criticalSectionStop(pPmicCoreHandle);

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        /* Update TimeMode to pTimeCfg */
        pTimeCfg->timeMode = HW_REG_GET_FIELD(regData,
                                              PMIC_RTC_CTRL_1_MODE_12_24);
    }

    return pmicStatus;
}

/*!
 * \brief   This function is used to set the RTC/Alarm Meridian Mode.
 */
static int32_t Pmic_rtcSetMeridianMode(Pmic_CoreHandle_t    *pPmicCoreHandle,
                                       const Pmic_RtcTime_t  timeCfg,
                                       bool                  operation_type)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData    = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);

    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        PMIC_RTC_CTRL_1_REGADDR,
                                        &regData);
    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (PMIC_RTC_12_HOUR_MODE == HW_REG_GET_FIELD(regData,
                                  PMIC_RTC_CTRL_1_MODE_12_24)))
    {
        /* Operation for Alarm */
        if(PMIC_RTC_OPS_FOR_ALARM == operation_type)
        {
            /* Writing Meridian Mode to PMIC */
            pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                                PMIC_ALARM_HOURS_REGADDR,
                                                &regData);
            if(PMIC_ST_SUCCESS == pmicStatus)
            {
                HW_REG_SET_FIELD(regData,
                                 PMIC_ALARM_HOURS_ALR_PM_NAM,
                                 timeCfg.meridianMode);
              pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                            PMIC_ALARM_HOURS_REGADDR,
                                            regData);
            }
        }
        /* Operation for RTC */
        else
        {
            /* Writing Meridian Mode to PMIC */
            pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                                PMIC_RTC_HOURS_REGADDR,
                                                &regData);
            if(PMIC_ST_SUCCESS == pmicStatus)
            {
                HW_REG_SET_FIELD(regData,
                                 PMIC_ALARM_HOURS_ALR_PM_NAM,
                                 timeCfg.meridianMode);
                pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                                    PMIC_RTC_HOURS_REGADDR,
                                                    regData);
            }
        }
    }

    Pmic_criticalSectionStop(pPmicCoreHandle);
    return pmicStatus;
}

/*!
 * \brief   This function is used to get the RTC/Alarm Meridian Mode.
 */
static int32_t Pmic_rtcGetMeridianMode(Pmic_CoreHandle_t *pPmicCoreHandle,
                                       Pmic_RtcTime_t    *pTimeCfg,
                                       bool               operation_type)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData    = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);

    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        PMIC_RTC_CTRL_1_REGADDR,
                                        &regData);
    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (PMIC_RTC_12_HOUR_MODE == HW_REG_GET_FIELD(regData,
                                  PMIC_RTC_CTRL_1_MODE_12_24)))
    {
        /* Operation for Alarm */
        if(PMIC_RTC_OPS_FOR_ALARM == operation_type)
        {
            /* Reading the hour */
            pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                                PMIC_ALARM_HOURS_REGADDR,
                                                &regData);
        }
        /* Operation for RTC */
        else
        {
            /* Reading the hour */
            pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                                PMIC_RTC_HOURS_REGADDR,
                                                &regData);
        }

        if(PMIC_ST_SUCCESS == pmicStatus)
        {
            /* Operation for Alarm */
            if(PMIC_RTC_OPS_FOR_ALARM == operation_type)
            {
                /* Update Time Meridian to pTimeCfg */
                pTimeCfg->meridianMode = HW_REG_GET_FIELD(regData,
                                            PMIC_ALARM_HOURS_ALR_PM_NAM);
            }
            else
            {
                /* Update Time Meridian to pTimeCfg */
                pTimeCfg->meridianMode = HW_REG_GET_FIELD(regData,
                                            PMIC_ALARM_HOURS_ALR_PM_NAM);
            }
        }
    }

    Pmic_criticalSectionStop(pPmicCoreHandle);

    return pmicStatus;
}

/*!
 * \brief   This function is used to set the RTC/Alarm Hours.
 */
static int32_t Pmic_rtcSetHours(Pmic_CoreHandle_t    *pPmicCoreHandle,
                                const Pmic_RtcTime_t  timeCfg,
                                bool                  operation_type)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData    = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);

    /* Operation for Alarm */
    if(PMIC_RTC_OPS_FOR_ALARM == operation_type)
    {
        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                            PMIC_ALARM_HOURS_REGADDR,
                                            &regData);
        if(PMIC_ST_SUCCESS == pmicStatus)
        {
            /* Writing hour to PMIC */
            HW_REG_SET_FIELD(regData,
                             PMIC_ALARM_HOURS_ALR_HOUR_1,
                             (timeCfg.hour /
                              PMIC_RTC_CONVERT_4BIT_MSB_TO_DEC));
            HW_REG_SET_FIELD(regData,
                             PMIC_ALARM_HOURS_ALR_HOUR_0,
                             (timeCfg.hour %
                              PMIC_RTC_CONVERT_4BIT_MSB_TO_DEC));

            pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                                PMIC_ALARM_HOURS_REGADDR,
                                                regData);
        }
    }
    /* Operation for RTC */
    else
    {
        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                            PMIC_RTC_HOURS_REGADDR,
                                            &regData);
        if(PMIC_ST_SUCCESS == pmicStatus)
        {
                /* Writing hour to PMIC */
            HW_REG_SET_FIELD(regData,
                             PMIC_RTC_HOURS_HOUR_1,
                             (timeCfg.hour /
                              PMIC_RTC_CONVERT_4BIT_MSB_TO_DEC));
            HW_REG_SET_FIELD(regData,
                             PMIC_RTC_HOURS_HOUR_0,
                             (timeCfg.hour %
                              PMIC_RTC_CONVERT_4BIT_MSB_TO_DEC));
            pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                                PMIC_RTC_HOURS_REGADDR,
                                                regData);
        }
    }

    Pmic_criticalSectionStop(pPmicCoreHandle);
    return pmicStatus;
}

/*!
 * \brief   This function is used to get the RTC/Alarm Hours.
 */
static int32_t Pmic_rtcGetHours(Pmic_CoreHandle_t *pPmicCoreHandle,
                                Pmic_RtcTime_t    *pTimeCfg,
                                bool               operation_type)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData    = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);

    /* Operation for Alarm */
    if(PMIC_RTC_OPS_FOR_ALARM == operation_type)
    {
        /* Reading the hour */
        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                            PMIC_ALARM_HOURS_REGADDR,
                                            &regData);
    }
    /* Operation for RTC */
    else
    {
        /* Reading the hour */
        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                            PMIC_RTC_HOURS_REGADDR,
                                            &regData);
    }

    Pmic_criticalSectionStop(pPmicCoreHandle);

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        /* Operation for Alarm */
        if(PMIC_RTC_OPS_FOR_ALARM == operation_type)
        {
            /* Update Hours to pTimeCfg */
            pTimeCfg->hour = PMIC_RTC_CONVERT_4BIT_MSB_TO_DEC *
                             (HW_REG_GET_FIELD(regData,
                                               PMIC_ALARM_HOURS_ALR_HOUR_1));
            pTimeCfg->hour = pTimeCfg->hour +
                             (HW_REG_GET_FIELD(regData,
                                               PMIC_ALARM_HOURS_ALR_HOUR_0));
        }
        else
        {
            /* Update Hours to pTimeCfg */
            pTimeCfg->hour = PMIC_RTC_CONVERT_4BIT_MSB_TO_DEC *
                             (HW_REG_GET_FIELD(regData,
                                               PMIC_RTC_HOURS_HOUR_1));
            pTimeCfg->hour = pTimeCfg->hour +
                             (HW_REG_GET_FIELD(regData,
                                               PMIC_RTC_HOURS_HOUR_0));
        }
    }

    return pmicStatus;
}

/*!
 * \brief   This function is used to set the RTC/Alarm Day.
 */
static int32_t Pmic_rtcSetDay(Pmic_CoreHandle_t    *pPmicCoreHandle,
                              const Pmic_RtcDate_t  dateCfg,
                              bool                  operation_type)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData    = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);

    /* Operation for Alarm */
    if(PMIC_RTC_OPS_FOR_ALARM == operation_type)
    {
        /* Setting the day to PMIC */
        HW_REG_SET_FIELD(regData,
                         PMIC_ALARM_DAYS_ALR_DAY_1,
                         (dateCfg.day /
                          PMIC_RTC_CONVERT_4BIT_MSB_TO_DEC));
        HW_REG_SET_FIELD(regData,
                         PMIC_ALARM_DAYS_ALR_DAY_0,
                         (dateCfg.day %
                          PMIC_RTC_CONVERT_4BIT_MSB_TO_DEC));

        pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                            PMIC_ALARM_DAYS_REGADDR,
                                            regData);
    }
    /* Operation for RTC */
    else
    {
        /* Setting the day to PMIC */
        HW_REG_SET_FIELD(regData,
                         PMIC_RTC_DAYS_DAY_1,
                         (dateCfg.day /
                          PMIC_RTC_CONVERT_4BIT_MSB_TO_DEC));
        HW_REG_SET_FIELD(regData,
                         PMIC_RTC_DAYS_DAY_0,
                         (dateCfg.day %
                          PMIC_RTC_CONVERT_4BIT_MSB_TO_DEC));

        pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                            PMIC_RTC_DAYS_REGADDR,
                                            regData);
    }

    Pmic_criticalSectionStop(pPmicCoreHandle);

    return pmicStatus;
}

/*!
 * \brief   This function is used to get the RTC/Alarm Day.
 */
static int32_t Pmic_rtcGetDay(Pmic_CoreHandle_t *pPmicCoreHandle,
                              Pmic_RtcDate_t    *pDateCfg,
                              bool               operation_type)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData    = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);

    /* Operation for Alarm */
    if(PMIC_RTC_OPS_FOR_ALARM == operation_type)
    {
        /* Reading the day */
        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                            PMIC_ALARM_DAYS_REGADDR,
                                            &regData);
    }
    /* Operation for RTC */
    else
    {
        /* Reading the day */
        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                            PMIC_RTC_DAYS_REGADDR,
                                            &regData);
    }

    Pmic_criticalSectionStop(pPmicCoreHandle);

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        /* Operation for Alarm */
        if(PMIC_RTC_OPS_FOR_ALARM == operation_type)
        {
            /* Update Day of the Date to pDateCfg */
            pDateCfg->day = PMIC_RTC_CONVERT_4BIT_MSB_TO_DEC *
                                     (HW_REG_GET_FIELD(regData,
                                         PMIC_ALARM_DAYS_ALR_DAY_1));
            pDateCfg->day = pDateCfg->day +
                                     (HW_REG_GET_FIELD(regData,
                                         PMIC_ALARM_DAYS_ALR_DAY_0));
        }
        else
        {
            /* Update Day of the Date to pDateCfg */
            pDateCfg->day = PMIC_RTC_CONVERT_4BIT_MSB_TO_DEC *
                                 (HW_REG_GET_FIELD(regData,
                                     PMIC_RTC_DAYS_DAY_1));
            pDateCfg->day = pDateCfg->day +
                                 (HW_REG_GET_FIELD(regData,
                                     PMIC_RTC_DAYS_DAY_0));
        }
    }

    return pmicStatus;
}

/*!
 * \brief   This function is used to set the RTC/Alarm Month.
 */
static int32_t Pmic_rtcSetMonth(Pmic_CoreHandle_t    *pPmicCoreHandle,
                                const Pmic_RtcDate_t  dateCfg,
                                bool                  operation_type)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData    = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);

    /* Operation for Alarm */
    if(PMIC_RTC_OPS_FOR_ALARM == operation_type)
    {
        /* Setting the month to PMIC */
        HW_REG_SET_FIELD(regData,
                         PMIC_ALARM_MONTHS_ALR_MONTH_1,
                         (dateCfg.month /
                          PMIC_RTC_CONVERT_4BIT_MSB_TO_DEC));
        HW_REG_SET_FIELD(regData,
                         PMIC_ALARM_MONTHS_ALR_MONTH_0,
                         (dateCfg.month %
                          PMIC_RTC_CONVERT_4BIT_MSB_TO_DEC));
        pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                            PMIC_ALARM_MONTHS_REGADDR,
                                            regData);
    }
    /* Operation for RTC */
    else
    {
        /* Setting the month to PMIC */
        HW_REG_SET_FIELD(regData,
                         PMIC_RTC_MONTHS_MONTH_1,
                         (dateCfg.month /
                          PMIC_RTC_CONVERT_4BIT_MSB_TO_DEC));
        HW_REG_SET_FIELD(regData,
                         PMIC_RTC_MONTHS_MONTH_0,
                         (dateCfg.month %
                          PMIC_RTC_CONVERT_4BIT_MSB_TO_DEC));
        pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                            PMIC_RTC_MONTHS_REGADDR,
                                            regData);
    }

    Pmic_criticalSectionStop(pPmicCoreHandle);

    return pmicStatus;
}

/*!
 * \brief   This function is used to get the RTC/Alarm Month.
 */
static int32_t Pmic_rtcGetMonth(Pmic_CoreHandle_t *pPmicCoreHandle,
                                Pmic_RtcDate_t    *pDateCfg,
                                bool               operation_type)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData    = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);

    /* Operation for Alarm */
    if(PMIC_RTC_OPS_FOR_ALARM == operation_type)
    {
        /* Reading the month */
        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                            PMIC_ALARM_MONTHS_REGADDR,
                                            &regData);
    }
    /* Operation for RTC */
    else
    {
        /* Reading the month */
        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                            PMIC_RTC_MONTHS_REGADDR,
                                            &regData);
    }

    Pmic_criticalSectionStop(pPmicCoreHandle);

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        /* Operation for Alarm */
        if(PMIC_RTC_OPS_FOR_ALARM == operation_type)
        {
            /* Update Month of the Date to pDateCfg */
            pDateCfg->month = PMIC_RTC_CONVERT_4BIT_MSB_TO_DEC *
                              (HW_REG_GET_FIELD(regData,
                                      PMIC_ALARM_MONTHS_ALR_MONTH_1));
            pDateCfg->month = pDateCfg->month +
                              (HW_REG_GET_FIELD(regData,
                                      PMIC_ALARM_MONTHS_ALR_MONTH_0));
        }
        else
        {
            /* Update Month of the Date to pDateCfg */
            pDateCfg->month = PMIC_RTC_CONVERT_4BIT_MSB_TO_DEC *
                              (HW_REG_GET_FIELD(regData,
                                      PMIC_RTC_MONTHS_MONTH_1));
            pDateCfg->month = pDateCfg->month +
                              (HW_REG_GET_FIELD(regData,
                                      PMIC_RTC_MONTHS_MONTH_0));
        }
    }

    return pmicStatus;
}

/*!
 * \brief   This function is used to set the RTC/Alarm Year.
 */
static int32_t Pmic_rtcSetYear(Pmic_CoreHandle_t    *pPmicCoreHandle,
                               const Pmic_RtcDate_t  dateCfg,
                               bool                  operation_type)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData    = 0U;
    uint16_t year      = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);

    year = (dateCfg.year % PMIC_RTC_EXTRACT_YEAR_DECIMAL_0_99);

    /* Operation for Alarm */
    if(PMIC_RTC_OPS_FOR_ALARM == operation_type)
    {
        /* Setting the year to PMIC */
        HW_REG_SET_FIELD(regData,
                         PMIC_ALARM_YEARS_ALR_YEAR_1,
                         (year / PMIC_RTC_CONVERT_4BIT_MSB_TO_DEC));
        HW_REG_SET_FIELD(regData,
                         PMIC_ALARM_YEARS_ALR_YEAR_0,
                         (year % PMIC_RTC_CONVERT_4BIT_MSB_TO_DEC));

        pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                            PMIC_ALARM_YEARS_REGADDR,
                                            regData);
    }
    /* Operation for RTC */
    else
    {
        /* Setting the year to PMIC */
        HW_REG_SET_FIELD(regData,
                         PMIC_RTC_YEARS_YEAR_1,
                         (year / PMIC_RTC_CONVERT_4BIT_MSB_TO_DEC));
        HW_REG_SET_FIELD(regData,
                         PMIC_RTC_YEARS_YEAR_0,
                         (year % PMIC_RTC_CONVERT_4BIT_MSB_TO_DEC));

        pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                            PMIC_RTC_YEARS_REGADDR,
                                            regData);
    }

    Pmic_criticalSectionStop(pPmicCoreHandle);

    return pmicStatus;
}

/*!
 * \brief   This function is used to get the RTC/Alarm Year.
 */
static int32_t Pmic_rtcGetYear(Pmic_CoreHandle_t *pPmicCoreHandle,
                               Pmic_RtcDate_t    *pDateCfg,
                               bool               operation_type)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData    = 0U;
    uint8_t regVal     = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);

    /* Operation for Alarm */
    if(PMIC_RTC_OPS_FOR_ALARM == operation_type)
    {
        /* Reading the year */
        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                            PMIC_ALARM_YEARS_REGADDR,
                                            &regData);
    }
    /* Operation for RTC */
    else
    {
        /* Reading the year */
        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                            PMIC_RTC_YEARS_REGADDR,
                                            &regData);
    }

    Pmic_criticalSectionStop(pPmicCoreHandle);

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        /* Operation for Alarm */
        if(PMIC_RTC_OPS_FOR_ALARM == operation_type)
        {
            regVal = HW_REG_GET_FIELD(regData,
                                      PMIC_ALARM_YEARS_ALR_YEAR_1);

            /* Update Year of the Date to pDateCfg */
            pDateCfg->year = ((uint16_t)PMIC_RTC_CONVERT_4BIT_MSB_TO_DEC)
                                      * regVal;

            regVal = HW_REG_GET_FIELD(regData,
                                      PMIC_ALARM_YEARS_ALR_YEAR_0);

            pDateCfg->year = pDateCfg->year + regVal;
        }
        else
        {
            regVal = HW_REG_GET_FIELD(regData,
                                      PMIC_RTC_YEARS_YEAR_1);
            /* Update Year of the Date to pDateCfg */
            pDateCfg->year = ((uint16_t)PMIC_RTC_CONVERT_4BIT_MSB_TO_DEC)
                                      * regVal;

            regVal = HW_REG_GET_FIELD(regData,
                                     PMIC_RTC_YEARS_YEAR_0);

            pDateCfg->year = pDateCfg->year + regVal;
        }
            pDateCfg->year += PMIC_RTC_YEAR_MIN;
    }

    return pmicStatus;
}

/*!
 * \brief   This function is used to set the RTC Weekday.
 */
static int32_t Pmic_rtcSetWeekday(Pmic_CoreHandle_t    *pPmicCoreHandle,
                                  const Pmic_RtcDate_t  dateCfg)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData    = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);

    HW_REG_SET_FIELD(regData, PMIC_RTC_WEEKS_WEEK, dateCfg.weekday);

    pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                        PMIC_RTC_WEEKS_REGADDR,
                                        regData);

    Pmic_criticalSectionStop(pPmicCoreHandle);

    return pmicStatus;
}

/*!
 * \brief   This function is used to get the RTC Weekday.
 */
static int32_t Pmic_rtcGetWeekday(Pmic_CoreHandle_t *pPmicCoreHandle,
                                  Pmic_RtcDate_t    *pDateCfg)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData    = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);

        /* Reading the weekday */
        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                            PMIC_RTC_WEEKS_REGADDR,
                                            &regData);

    Pmic_criticalSectionStop(pPmicCoreHandle);

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        /* Update Weekday of the Date to pDateCfg */
        pDateCfg->weekday = HW_REG_GET_FIELD(regData, PMIC_RTC_WEEKS_WEEK);
    }

    return pmicStatus;
}

/*!
 * \brief   Check RTC time hours mode
 */
static int32_t Pmic_rtcCheckHoursMode(Pmic_CoreHandle_t    *pPmicCoreHandle,
                                      const Pmic_RtcTime_t  timeCfg)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t timeMode = 0U;

    /* Get current TimeMode */
    if(0U == pmic_validParamCheck(timeCfg.validParams,
                            PMIC_RTC_TIME_CFG_TIMEMODE_VALID))
    {
        pmicStatus = Pmic_rtcGetTimeMode(pPmicCoreHandle,
                                         (Pmic_RtcTime_t *)&timeCfg);
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        timeMode = timeCfg.timeMode;
        if((timeCfg.timeMode == PMIC_RTC_12_HOUR_MODE) &&
           (pmic_validParamCheck(timeCfg.validParams,
                            PMIC_RTC_TIME_CFG_MERIDIAN_VALID)) &&
            (timeCfg.meridianMode > PMIC_RTC_PM_MODE))
        {
            pmicStatus = PMIC_ST_ERR_INV_TIME;
        }
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (pmic_validParamCheck(timeCfg.validParams,
                            PMIC_RTC_TIME_CFG_HRS_VALID)))
    {
        if(PMIC_RTC_12_HOUR_MODE == timeMode)
        {
            if((timeCfg.hour > PMIC_RTC_12HFMT_HR_MAX) ||
               (timeCfg.hour < PMIC_RTC_12HFMT_HR_MIN))
            {
                pmicStatus = PMIC_ST_ERR_INV_TIME;
            }

            if((PMIC_ST_SUCCESS == pmicStatus) &&
               (timeCfg.meridianMode > PMIC_RTC_PM_MODE))
            {
                pmicStatus = PMIC_ST_ERR_INV_TIME;
            }
        }
        else if (PMIC_RTC_24_HOUR_MODE == timeMode)
        {
            if(timeCfg.hour > PMIC_RTC_24HFMT_HR_MAX)
            {
                pmicStatus = PMIC_ST_ERR_INV_TIME;
            }
        }
        else
        {
            pmicStatus = PMIC_ST_ERR_INV_TIME;
        }
    }

    return pmicStatus;
}

/*!
 * \brief   This function is used to check for errors in time values
 */
static int32_t Pmic_rtcCheckTime(Pmic_CoreHandle_t    *pPmicCoreHandle,
                                 const Pmic_RtcTime_t  timeCfg)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    if(pmic_validParamCheck(timeCfg.validParams,
                            PMIC_RTC_TIME_CFG_SEC_VALID) &&
        (timeCfg.seconds > PMIC_RTC_MINUTE_SEC_MAX))
    {
        pmicStatus = PMIC_ST_ERR_INV_TIME;
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
        pmic_validParamCheck(timeCfg.validParams,
                            PMIC_RTC_TIME_CFG_MIN_VALID) &&
        (timeCfg.minutes > PMIC_RTC_MINUTE_SEC_MAX))
    {
        pmicStatus = PMIC_ST_ERR_INV_TIME;
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
        pmic_validParamCheck(timeCfg.validParams,
                            PMIC_RTC_TIME_CFG_TIMEMODE_VALID) &&
        (timeCfg.timeMode > PMIC_RTC_12_HOUR_MODE))
    {
        pmicStatus = PMIC_ST_ERR_INV_TIME;
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        pmicStatus = Pmic_rtcCheckHoursMode(pPmicCoreHandle, timeCfg);
    }

    return pmicStatus;
}

/*!
 * \brief   This function is used to check for errors in date values
 *          w.r.to month
 */
static int32_t Pmic_rtcCheckMonthDays(const Pmic_RtcDate_t dateCfg, bool leap)
{
    int32_t  pmicStatus = PMIC_ST_SUCCESS;

    /* Check months having 30 days */
    if((PMIC_RTC_MONTH_APR == dateCfg.month)  ||
       (PMIC_RTC_MONTH_JUN == dateCfg.month)  ||
       (PMIC_RTC_MONTH_SEP == dateCfg.month)  ||
       (PMIC_RTC_MONTH_NOV == dateCfg.month))
    {
        if((dateCfg.day < PMIC_RTC_DAY_MIN) ||
           (dateCfg.day > PMIC_RTC_MNTH_DAY_MAX_30))
        {
            pmicStatus = PMIC_ST_ERR_INV_DATE;
        }
    }
    /* Check February days in year */
    else if(PMIC_RTC_MONTH_FEB == dateCfg.month)
    {
        /* February days in leap year */
        if(true == leap)
        {
            if((dateCfg.day < PMIC_RTC_DAY_MIN) ||
               (dateCfg.day > PMIC_RTC_LPY_FEB_MNTH_DAY_MAX))
            {
                pmicStatus = PMIC_ST_ERR_INV_DATE;
            }
        }
        /* February days in non-leap year */
        else
        {
            if((dateCfg.day < PMIC_RTC_DAY_MIN) ||
               (dateCfg.day > PMIC_RTC_NLPY_FEB_MNTH_DAY_MAX))
            {
                pmicStatus = PMIC_ST_ERR_INV_DATE;
            }
        }
     }
     /* Check months having 31 days */
     else
     {
        if((dateCfg.day < PMIC_RTC_DAY_MIN) ||
           (dateCfg.day > PMIC_RTC_MNTH_DAY_MAX_31))
        {
            pmicStatus = PMIC_ST_ERR_INV_DATE;
        }
     }

    return pmicStatus;
}

/*!
 * \brief   This function is used to check for errors in date values.
 */
static int32_t Pmic_rtcCheckDate(Pmic_CoreHandle_t    *pPmicCoreHandle,
                                 const Pmic_RtcDate_t  dateCfg)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    bool leapyear = (bool)false;

    if(pmic_validParamCheck(dateCfg.validParams,
                            PMIC_RTC_DATE_CFG_MONTH_VALID) &&
       ((dateCfg.month > PMIC_RTC_MONTH_DEC) ||
        (dateCfg.month <= PMIC_RTC_MONTH_JAN)))
    {
        pmicStatus = PMIC_ST_ERR_INV_DATE;
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (pmic_validParamCheck(dateCfg.validParams,
                             PMIC_RTC_DATE_CFG_YEAR_VALID)) &&
       ((dateCfg.year > PMIC_RTC_YEAR_MAX) ||
        (dateCfg.year < PMIC_RTC_YEAR_MIN)))
    {
        pmicStatus = PMIC_ST_ERR_INV_DATE;
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (pmic_validParamCheck(dateCfg.validParams,
                             PMIC_RTC_DATE_CFG_WEEKDAY_VALID)) &&
       ((dateCfg.weekday > PMIC_RTC_WEEKDAY_SATURDAY) ||
        (dateCfg.weekday < PMIC_RTC_WEEKDAY_SUNDAY)))
    {
        pmicStatus = PMIC_ST_ERR_INV_DATE;
    }

    /* Get RTC Current leap Status */
    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        if(pmic_validParamCheck(dateCfg.validParams,
                                 PMIC_RTC_DATE_CFG_YEAR_VALID))
        {
            leapyear = (0U == (dateCfg.year % 4U));
        }
        else
        {
            pmicStatus = Pmic_rtcGetYear(pPmicCoreHandle,
                                         (Pmic_RtcDate_t  *)&dateCfg,
                                         PMIC_RTC_OPS_FOR_RTC);
            if(PMIC_ST_SUCCESS == pmicStatus)
            {
                leapyear = (0U == (dateCfg.year % 4U));
            }
        }
    }

    /* Get RTC Current Month */
    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        if(0U == pmic_validParamCheck(dateCfg.validParams,
                                 PMIC_RTC_DATE_CFG_MONTH_VALID))
        {
            pmicStatus = Pmic_rtcGetMonth(pPmicCoreHandle,
                                          (Pmic_RtcDate_t  *)&dateCfg,
                                          PMIC_RTC_OPS_FOR_RTC);
            /* Check the current month to check days */
            if((PMIC_ST_SUCCESS == pmicStatus) &&
               ((dateCfg.month > PMIC_RTC_MONTH_DEC) ||
                (dateCfg.month <= PMIC_RTC_MONTH_JAN)))
            {
                pmicStatus = PMIC_ST_ERR_INV_DATE;
            }
        }
    }


    if((PMIC_ST_SUCCESS == pmicStatus) &&
       pmic_validParamCheck(dateCfg.validParams,
                            PMIC_RTC_DATE_CFG_DAY_VALID))
    {
        pmicStatus = Pmic_rtcCheckMonthDays(dateCfg, leapyear);
    }

    return pmicStatus;
}

/*!
 * \brief   This function is used to check Date and Time values.
 */
static int32_t Pmic_rtcCheckDateTime(Pmic_CoreHandle_t    *pPmicCoreHandle,
                                     const Pmic_RtcTime_t  timeCfg,
                                     const Pmic_RtcDate_t  dateCfg)
{
    int32_t  pmicStatus = PMIC_ST_SUCCESS;

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        pmicStatus = Pmic_rtcCheckTime(pPmicCoreHandle, timeCfg);
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        pmicStatus = Pmic_rtcCheckDate(pPmicCoreHandle, dateCfg);
    }

    return (pmicStatus);
}

/*!
 * \brief   This function is used to set the RTC Alarm Time.
 */
static int32_t Pmic_alarmSetTime(Pmic_CoreHandle_t    *pPmicCoreHandle,
                                 const Pmic_RtcTime_t  timeCfg)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    if(pmic_validParamCheck(timeCfg.validParams,
                            PMIC_RTC_TIME_CFG_SEC_VALID))
    {
        /* Writing seconds to PMIC */
        pmicStatus = Pmic_rtcSetSeconds(pPmicCoreHandle,
                                        timeCfg,
                                        PMIC_RTC_OPS_FOR_ALARM);
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (pmic_validParamCheck(timeCfg.validParams,
                                       PMIC_RTC_TIME_CFG_MIN_VALID)))
    {
        /* Writing minutes to PMIC */
        pmicStatus = Pmic_rtcSetMinutes(pPmicCoreHandle,
                                        timeCfg,
                                        PMIC_RTC_OPS_FOR_ALARM);
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (pmic_validParamCheck(timeCfg.validParams,
                                       PMIC_RTC_TIME_CFG_TIMEMODE_VALID)))
    {
        /* Setting  Timemode to PMIC */
        pmicStatus = Pmic_rtcSetTimeMode(pPmicCoreHandle,
                                         timeCfg);
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (pmic_validParamCheck(timeCfg.validParams,
                                       PMIC_RTC_TIME_CFG_TIMEMODE_VALID)))
    {
        /* Setting Meridian mode to PMIC */
        pmicStatus = Pmic_rtcSetMeridianMode(pPmicCoreHandle,
                                             timeCfg,
                                             PMIC_RTC_OPS_FOR_ALARM);
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (pmic_validParamCheck(timeCfg.validParams,
                                       PMIC_RTC_TIME_CFG_HRS_VALID)))
    {
        /* Setting hour to PMIC */
        pmicStatus = Pmic_rtcSetHours(pPmicCoreHandle,
                                      timeCfg,
                                      PMIC_RTC_OPS_FOR_ALARM);
    }

    return pmicStatus;
}

/*!
 * \brief   This function is used to set the RTC Alarm Date.
 */
static int32_t Pmic_alarmSetDate(Pmic_CoreHandle_t *pPmicCoreHandle,
                                 Pmic_RtcDate_t     dateCfg)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    if(pmic_validParamCheck(dateCfg.validParams,
                            PMIC_RTC_DATE_CFG_DAY_VALID))
    {
        /* Setting the day to PMIC */
        pmicStatus = Pmic_rtcSetDay(pPmicCoreHandle,
                                    dateCfg,
                                    PMIC_RTC_OPS_FOR_ALARM);
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (pmic_validParamCheck(dateCfg.validParams,
                                       PMIC_RTC_DATE_CFG_MONTH_VALID)))
    {
        /* Setting the month to PMIC */
        pmicStatus = Pmic_rtcSetMonth(pPmicCoreHandle,
                                      dateCfg,
                                      PMIC_RTC_OPS_FOR_ALARM);
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (pmic_validParamCheck(dateCfg.validParams,
                                       PMIC_RTC_DATE_CFG_YEAR_VALID)))
    {
        /* Setting the year to PMIC */
        pmicStatus = Pmic_rtcSetYear(pPmicCoreHandle,
                                     dateCfg,
                                     PMIC_RTC_OPS_FOR_ALARM);
    }

    return pmicStatus;
}

/*!
 * \brief   This function is used to En/Disable the Alarm Interrupt.
 */
static int32_t Pmic_setAlarmIntr(Pmic_CoreHandle_t *pPmicCoreHandle,
                                 bool               enableIntr)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData    = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                            PMIC_RTC_INTERRUPTS_REGADDR,
                                            &regData);
        if(PMIC_ST_SUCCESS == pmicStatus)
        {
            if(PMIC_RTC_ALARM_INTR_ENABLE == enableIntr)
            {
                HW_REG_SET_FIELD(regData,
                                 PMIC_RTC_INTERRUPTS_IT_ALARM,
                                 PMIC_RTC_ALARM_INTR_ENABLE);
            }
            else
            {
                HW_REG_SET_FIELD(regData,
                                 PMIC_RTC_INTERRUPTS_IT_ALARM,
                                 PMIC_RTC_ALARM_INTR_DISABLE);
            }

            pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                                PMIC_RTC_INTERRUPTS_REGADDR,
                                                regData);
        }
    }

    Pmic_criticalSectionStop(pPmicCoreHandle);

    return pmicStatus;
}

/*!
 * \brief   This function is used to get the RTC Alarm Time.
 */
static int32_t Pmic_alarmGetTime(Pmic_CoreHandle_t *pPmicCoreHandle,
                                  Pmic_RtcTime_t    *pTimeCfg)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    if(pmic_validParamCheck(pTimeCfg->validParams,
                            PMIC_RTC_TIME_CFG_SEC_VALID))
    {
        /* Reading the seconds */
        pmicStatus = Pmic_rtcGetSeconds(pPmicCoreHandle,
                                        pTimeCfg,
                                        PMIC_RTC_OPS_FOR_ALARM);
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (pmic_validParamCheck(pTimeCfg->validParams,
                                       PMIC_RTC_TIME_CFG_MIN_VALID)))
    {
        /* Reading the minutes */
        pmicStatus = Pmic_rtcGetMinutes(pPmicCoreHandle,
                                        pTimeCfg,
                                        PMIC_RTC_OPS_FOR_ALARM);
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (pmic_validParamCheck(pTimeCfg->validParams,
                                       PMIC_RTC_TIME_CFG_TIMEMODE_VALID)))
    {
        /* Getting the TimeMode */
        pmicStatus = Pmic_rtcGetTimeMode(pPmicCoreHandle,
                                         pTimeCfg);
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (pmic_validParamCheck(pTimeCfg->validParams,
                                       PMIC_RTC_TIME_CFG_MERIDIAN_VALID)))
    {
        /* Getting the Meridian Mode */
        pmicStatus = Pmic_rtcGetMeridianMode(pPmicCoreHandle,
                                             pTimeCfg,
                                             PMIC_RTC_OPS_FOR_ALARM);
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (pmic_validParamCheck(pTimeCfg->validParams,
                                       PMIC_RTC_TIME_CFG_HRS_VALID)))
    {
        /* Reading the hour */
        pmicStatus = Pmic_rtcGetHours(pPmicCoreHandle,
                                      pTimeCfg,
                                      PMIC_RTC_OPS_FOR_ALARM);
    }

    return pmicStatus;
}

/*!
 * \brief   This function is used to get the RTC Alarm Date.
 */
static int32_t Pmic_alarmGetDate(Pmic_CoreHandle_t *pPmicCoreHandle,
                                 Pmic_RtcDate_t    *pDateCfg)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    if(pmic_validParamCheck(pDateCfg->validParams,
                            PMIC_RTC_DATE_CFG_DAY_VALID))
    {
        /* Reading the day */
        pmicStatus = Pmic_rtcGetDay(pPmicCoreHandle,
                                    pDateCfg,
                                    PMIC_RTC_OPS_FOR_ALARM);
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (pmic_validParamCheck(pDateCfg->validParams,
                                       PMIC_RTC_DATE_CFG_MONTH_VALID)))
    {
        /* Reading the month */
        pmicStatus = Pmic_rtcGetMonth(pPmicCoreHandle,
                                      pDateCfg,
                                      PMIC_RTC_OPS_FOR_ALARM);
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (pmic_validParamCheck(pDateCfg->validParams,
                                       PMIC_RTC_DATE_CFG_YEAR_VALID)))
    {
        /* Reading the year */
        pmicStatus = Pmic_rtcGetYear(pPmicCoreHandle,
                                     pDateCfg,
                                     PMIC_RTC_OPS_FOR_ALARM);
    }

    return pmicStatus;
}

/*!
 * \brief   This function is used to set the RTC Time.
 */
static int32_t Pmic_rtcSetTime(Pmic_CoreHandle_t *pPmicCoreHandle,
                               Pmic_RtcTime_t     timeCfg)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    if(pmic_validParamCheck(timeCfg.validParams,
                            PMIC_RTC_TIME_CFG_SEC_VALID))
    {
        /* Writing seconds to PMIC */
        pmicStatus = Pmic_rtcSetSeconds(pPmicCoreHandle,
                                        timeCfg,
                                        PMIC_RTC_OPS_FOR_RTC);
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (pmic_validParamCheck(timeCfg.validParams,
                                       PMIC_RTC_TIME_CFG_MIN_VALID)))
    {
        /* Writing minutes to PMIC */
        pmicStatus = Pmic_rtcSetMinutes(pPmicCoreHandle,
                                        timeCfg,
                                        PMIC_RTC_OPS_FOR_RTC);
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (pmic_validParamCheck(timeCfg.validParams,
                                       PMIC_RTC_TIME_CFG_TIMEMODE_VALID)))
    {
        /* Setting TimeMode to PMIC */
        pmicStatus = Pmic_rtcSetTimeMode(pPmicCoreHandle,
                                         timeCfg);
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (pmic_validParamCheck(timeCfg.validParams,
                                       PMIC_RTC_TIME_CFG_MERIDIAN_VALID)))
    {
        /* Setting Meridian Mode to PMIC */
        pmicStatus = Pmic_rtcSetMeridianMode(pPmicCoreHandle,
                                             timeCfg,
                                             PMIC_RTC_OPS_FOR_RTC);
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (pmic_validParamCheck(timeCfg.validParams,
                                       PMIC_RTC_TIME_CFG_HRS_VALID)))
    {
        /* Setting Hours to PMIC */
        pmicStatus = Pmic_rtcSetHours(pPmicCoreHandle,
                                      timeCfg,
                                      PMIC_RTC_OPS_FOR_RTC);
    }

    return pmicStatus;
}

/*!
 * \brief   This function is used to set the RTC Date.
 */
static int32_t Pmic_rtcSetDate(Pmic_CoreHandle_t *pPmicCoreHandle,
                               Pmic_RtcDate_t     dateCfg)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    if(pmic_validParamCheck(dateCfg.validParams,
                            PMIC_RTC_DATE_CFG_DAY_VALID))
    {
        /* Setting the day to PMIC */
        pmicStatus = Pmic_rtcSetDay(pPmicCoreHandle,
                                    dateCfg,
                                    PMIC_RTC_OPS_FOR_RTC);
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (pmic_validParamCheck(dateCfg.validParams,
                                       PMIC_RTC_DATE_CFG_MONTH_VALID)))
    {
        /* Setting the month to PMIC */
        pmicStatus = Pmic_rtcSetMonth(pPmicCoreHandle,
                                      dateCfg,
                                      PMIC_RTC_OPS_FOR_RTC);
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (pmic_validParamCheck(dateCfg.validParams,
                                       PMIC_RTC_DATE_CFG_YEAR_VALID)))
    {
        /* Setting the year to PMIC */
        pmicStatus = Pmic_rtcSetYear(pPmicCoreHandle,
                                     dateCfg,
                                     PMIC_RTC_OPS_FOR_RTC);
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (pmic_validParamCheck(dateCfg.validParams,
                                       PMIC_RTC_DATE_CFG_WEEKDAY_VALID)))
    {
        /* Setting the weekday to PMIC */
        pmicStatus = Pmic_rtcSetWeekday(pPmicCoreHandle, dateCfg);
    }

    return pmicStatus;
}


/*!
 * \brief   Set RTC dynamic registers to static shadowed registers.
 */
static int32_t Pmic_rtcTriggerShadowRegisters(
                    Pmic_CoreHandle_t *pPmicCoreHandle)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData    = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);

    /* Setting GET_TIME */
    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        PMIC_RTC_CTRL_1_REGADDR,
                                        &regData);
    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        /* RESETTING the register by writing 0 */
        HW_REG_SET_FIELD(regData, PMIC_RTC_CTRL_1_GET_TIME, 0U);

        pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                            PMIC_RTC_CTRL_1_REGADDR,
                                            regData);
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        /* Writing 1 to copy data from dynamic register to shadowed register */
        HW_REG_SET_FIELD(regData, PMIC_RTC_CTRL_1_GET_TIME, 1U);

        pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                            PMIC_RTC_CTRL_1_REGADDR,
                                            regData);
    }

    Pmic_criticalSectionStop(pPmicCoreHandle);
    return pmicStatus;
}

/*!
 * \brief   This function is used to get the RTC Time.
 */
static int32_t Pmic_rtcGetTime(Pmic_CoreHandle_t *pPmicCoreHandle,
                               Pmic_RtcTime_t    *pTimeCfg)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    if(pmic_validParamCheck(pTimeCfg->validParams,
                            PMIC_RTC_TIME_CFG_SEC_VALID))
    {
        /* Reading the seconds */
        pmicStatus = Pmic_rtcGetSeconds(pPmicCoreHandle,
                                        pTimeCfg,
                                        PMIC_RTC_OPS_FOR_RTC);
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (pmic_validParamCheck(pTimeCfg->validParams,
                                       PMIC_RTC_TIME_CFG_MIN_VALID)))
    {
        /* Reading the minutes */
        pmicStatus = Pmic_rtcGetMinutes(pPmicCoreHandle,
                                        pTimeCfg,
                                        PMIC_RTC_OPS_FOR_RTC);
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (pmic_validParamCheck(pTimeCfg->validParams,
                                       PMIC_RTC_TIME_CFG_TIMEMODE_VALID)))
    {
        /* Getting the TimeMode */
        pmicStatus = Pmic_rtcGetTimeMode(pPmicCoreHandle,
                                         pTimeCfg);
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (pmic_validParamCheck(pTimeCfg->validParams,
                                       PMIC_RTC_TIME_CFG_MERIDIAN_VALID)))
    {
        /* Getting the Meridian mode */
        pmicStatus = Pmic_rtcGetMeridianMode(pPmicCoreHandle,
                                             pTimeCfg,
                                             PMIC_RTC_OPS_FOR_RTC);
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (pmic_validParamCheck(pTimeCfg->validParams,
                                       PMIC_RTC_TIME_CFG_HRS_VALID)))
    {
        /* Reading the hour  */
        pmicStatus = Pmic_rtcGetHours(pPmicCoreHandle,
                                      pTimeCfg,
                                      PMIC_RTC_OPS_FOR_RTC);
    }

    return pmicStatus;
}

/*!
 * \brief   This function is used to get the RTC Date.
 */
static int32_t Pmic_rtcGetDate(Pmic_CoreHandle_t *pPmicCoreHandle,
                               Pmic_RtcDate_t    *pDateCfg)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    if(pmic_validParamCheck(pDateCfg->validParams,
                            PMIC_RTC_DATE_CFG_DAY_VALID))
    {
        /* Reading the day */
        pmicStatus = Pmic_rtcGetDay(pPmicCoreHandle,
                                    pDateCfg,
                                    PMIC_RTC_OPS_FOR_RTC);
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (pmic_validParamCheck(pDateCfg->validParams,
                                       PMIC_RTC_DATE_CFG_MONTH_VALID)))
    {
        /* Reading the month */
        pmicStatus = Pmic_rtcGetMonth(pPmicCoreHandle,
                                      pDateCfg,
                                      PMIC_RTC_OPS_FOR_RTC);
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (pmic_validParamCheck(pDateCfg->validParams,
                                       PMIC_RTC_DATE_CFG_YEAR_VALID)))
    {
        /* Reading the year */
        pmicStatus = Pmic_rtcGetYear(pPmicCoreHandle,
                                     pDateCfg,
                                     PMIC_RTC_OPS_FOR_RTC);
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (pmic_validParamCheck(pDateCfg->validParams,
                                       PMIC_RTC_DATE_CFG_WEEKDAY_VALID)))
    {
        /* Reading the weekday */
        pmicStatus = Pmic_rtcGetWeekday(pPmicCoreHandle, pDateCfg);
    }

    return pmicStatus;
}

/*!
 * \brief   Set the RTC frequency compensation.
 */
static int32_t Pmic_rtcSetFreqCompensateVal(Pmic_CoreHandle_t *pPmicCoreHandle,
                                            const uint16_t     compensation)
{
    int32_t pmicStatus  = PMIC_ST_SUCCESS;
    uint8_t regData     = 0U;

    Pmic_criticalSectionStart (pPmicCoreHandle);

    /* Writing the LSB */
    regData = (compensation & PMIC_RTC_COMP_LSB_COMP_LSB_RTC_MASK);
    pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                        PMIC_RTC_COMP_LSB_REGADDR,
                                        regData);

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        /* Writing the MSB register */
        regData = ((compensation & PMIC_RTC_COMP_MSB_COMP_MSB_RTC_MASK) >>
                   PMIC_RTC_COMP_MSB_COMP_MSB_RTC_SHIFT);

        pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                            PMIC_RTC_COMP_MSB_REGADDR,
                                            regData);
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        /* Enable auto compensation */
        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                    PMIC_RTC_CTRL_1_REGADDR,
                                    &regData);
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        HW_REG_SET_FIELD(regData,
                         PMIC_RTC_CTRL_1_AUTO_COMP,
                         PMIC_RTC_AUTO_COMP_ON);

        pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                            PMIC_RTC_CTRL_1_REGADDR,
                                            regData);
    }

    Pmic_criticalSectionStop(pPmicCoreHandle);

    return pmicStatus;
}

/*!
 * \brief   Get the RTC frequency compensation.
 */
static int32_t Pmic_rtcGetFreqCompensateVal(Pmic_CoreHandle_t *pPmicCoreHandle,
                                            uint16_t          *pCompensation)
{
    int32_t pmicStatus  = PMIC_ST_SUCCESS;
    uint8_t regData     = 0U;

    *pCompensation = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);

    /* Reading the MSB register */
    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        PMIC_RTC_COMP_MSB_REGADDR,
                                        &regData);

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        *pCompensation = regData;
        *pCompensation = (*pCompensation <<
                          PMIC_RTC_COMP_MSB_COMP_MSB_RTC_SHIFT);

        /* Reading the LSB Register */
        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                            PMIC_RTC_COMP_LSB_REGADDR,
                                            &regData);
    }

    Pmic_criticalSectionStop(pPmicCoreHandle);

    *pCompensation = *pCompensation | regData;
    return pmicStatus;
}

/*!
 * \brief   This function is used to en/disable the TImer Interrupt.
 */
static int32_t Pmic_setTimerIntr(Pmic_CoreHandle_t *pPmicCoreHandle,
                                    bool               enableIntr)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData    = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        /* Enabling the RTC Timer Intr */
        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                            PMIC_RTC_INTERRUPTS_REGADDR,
                                            &regData);

        if(PMIC_ST_SUCCESS == pmicStatus)
        {
            if(PMIC_RTC_TIMER_INTR_DISABLE == enableIntr)
            {
                HW_REG_SET_FIELD(regData,
                             PMIC_RTC_INTERRUPTS_IT_TIMER,
                             PMIC_RTC_TIMER_INTR_DISABLE);
            }
            else
            {
                HW_REG_SET_FIELD(regData,
                             PMIC_RTC_INTERRUPTS_IT_TIMER,
                             PMIC_RTC_TIMER_INTR_ENABLE);
            }

            pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                               PMIC_RTC_INTERRUPTS_REGADDR,
                                               regData);
        }
    }

    Pmic_criticalSectionStop(pPmicCoreHandle);

    return pmicStatus;
}

/*!
 * \brief   This function is used to start/stop the RTC present in the PMIC.
 */
int32_t  Pmic_rtcEnableRtc(Pmic_CoreHandle_t *pPmicCoreHandle,
                           bool               enableRtc)
{
    int32_t  pmicStatus  = PMIC_ST_SUCCESS;
    uint8_t  regData     = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);

    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        PMIC_RTC_CTRL_1_REGADDR,
                                        &regData);
    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        if(PMIC_RTC_STOP == enableRtc)
        {
            /* Stopping the RTC */
            regData &= (uint8_t)(~(PMIC_RTC_CTRL_1_STOP_RTC_MASK));
        }
        else
        {
            /* Start RTC */
            HW_REG_SET_FIELD(regData,
                             PMIC_RTC_CTRL_1_STOP_RTC,
                             PMIC_RTC_START);
        }

        pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                            PMIC_RTC_CTRL_1_REGADDR,
                                            regData);
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        /* Checking RTC status */
        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                            PMIC_RTC_STATUS_REGADDR,
                                            &regData);

        if((PMIC_ST_SUCCESS == pmicStatus)  &&
           (enableRtc != HW_REG_GET_FIELD(regData, PMIC_RTC_STATUS_RUN)))
        {
            /* Improper RTC status */
            pmicStatus = PMIC_ST_ERR_RTC_STOP_FAIL;
        }
    }

    Pmic_criticalSectionStop(pPmicCoreHandle);

    return pmicStatus;
}

/*!
 * \brief   API to Set the alarm Time and Date to PMIC RTC.
 *          This function is used to set the alarm Date and Time parameters
 *          depending upon the bit fields set in validParams of Time and Date
 *          structures in RTC of PMIC Device.
 *
 * \param   pPmicCoreHandle   [IN]    PMIC Interface Handle.
 * \param   timeCfg           [IN]    PMIC RTC time configuration
 * \param   dateCfg           [IN]    PMIC RTC date configuration
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code.
 *          For valid values \ref Pmic_ErrorCodes.
 */
int32_t  Pmic_rtcSetAlarmInfo(Pmic_CoreHandle_t    *pPmicCoreHandle,
                              const Pmic_RtcTime_t  timeCfg,
                              const Pmic_RtcDate_t  dateCfg)
{
    int32_t  pmicStatus  = PMIC_ST_SUCCESS;

    if(NULL == pPmicCoreHandle)
    {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (false == pPmicCoreHandle->pPmic_SubSysInfo->rtcEnable))
    {
        pmicStatus = PMIC_ST_ERR_INV_DEVICE;
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       ((0U == timeCfg.validParams)  &&
        (0U == dateCfg.validParams)))
    {
        pmicStatus = PMIC_ST_ERR_INSUFFICIENT_CFG;
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        /* Verify time and date */
        pmicStatus = Pmic_rtcCheckDateTime(pPmicCoreHandle,
                                           timeCfg,
                                           dateCfg);
    }

    /* Set Alarm Time */
    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        pmicStatus = Pmic_alarmSetTime(pPmicCoreHandle, timeCfg);
    }

    /* Set Alarmr Date */
    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        pmicStatus = Pmic_alarmSetDate(pPmicCoreHandle, dateCfg);
    }

    return (pmicStatus);
}

/*!
 * \brief   API to Get the alarm Time and Date from PMIC RTC function.
 *          This function is used to Get the alarm date and time parameters
 *          depending upon the bit fields set in validParams of Time and Date
 *          structures in RTC of the PMIC Device.
 *
 * \param   pPmicCoreHandle   [IN]     PMIC Interface Handle.
 * \param   pTimeCfg          [OUT]    PMIC RTC time configuration
 * \param   pDateCfg          [OUT]    PMIC RTC date configuration
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code.
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t  Pmic_rtcGetAlarmInfo(Pmic_CoreHandle_t *pPmicCoreHandle,
                              Pmic_RtcTime_t    *pTimeCfg,
                              Pmic_RtcDate_t    *pDateCfg)
{
    int32_t  pmicStatus  = PMIC_ST_SUCCESS;

    if(NULL == pPmicCoreHandle)
    {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (false == pPmicCoreHandle->pPmic_SubSysInfo->rtcEnable))
    {
        pmicStatus = PMIC_ST_ERR_INV_DEVICE;
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       ((NULL == pTimeCfg) || (NULL == pDateCfg)))
    {
        pmicStatus = PMIC_ST_ERR_NULL_PARAM;
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       ((0U == pTimeCfg->validParams)  &&
        (0U == pDateCfg->validParams)))
    {
        pmicStatus = PMIC_ST_ERR_INSUFFICIENT_CFG;
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
         pmicStatus = Pmic_alarmGetTime(pPmicCoreHandle, pTimeCfg);
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
         pmicStatus = Pmic_alarmGetDate(pPmicCoreHandle, pDateCfg);
    }

    return (pmicStatus);
}

/*!
 * \brief   API to Set the timer interrupt Period to PMIC RTC.
 *          This function is used to set the timer interrupt Period to
 *          the RTC present in the PMIC.
 *
 * \param   pPmicCoreHandle   [IN]    PMIC Interface Handle.
 * \param   timerPeriod       [IN]    Timer interrupt periods.
 *                                    For Valid values:
 *                                          \ref Pmic_RtcTimerIntrPeriod.
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code.
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t  Pmic_rtcSetTimerPeriod(Pmic_CoreHandle_t *pPmicCoreHandle,
                                const uint8_t      timerPeriod)
{
    int32_t  pmicStatus  = PMIC_ST_SUCCESS;
    uint8_t  regData     = 0U;

    if(NULL == pPmicCoreHandle)
    {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (false == pPmicCoreHandle->pPmic_SubSysInfo->rtcEnable))
    {
        pmicStatus = PMIC_ST_ERR_INV_DEVICE;
    }

    if(timerPeriod > PMIC_RTC_DAY_INTR_PERIOD)
    {
        pmicStatus = PMIC_ST_ERR_INV_PARAM;
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        /* Setting timer period for timer interrupt */
        Pmic_criticalSectionStart(pPmicCoreHandle);

        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                            PMIC_RTC_INTERRUPTS_REGADDR,
                                            &regData);

        if(PMIC_ST_SUCCESS == pmicStatus)
        {
            HW_REG_SET_FIELD(regData, PMIC_RTC_INTERRUPTS_EVERY, timerPeriod);
            HW_REG_SET_FIELD(regData,
                             PMIC_RTC_INTERRUPTS_IT_TIMER,
                             PMIC_RTC_TIMER_INTR_ENABLE);

            pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                                PMIC_RTC_INTERRUPTS_REGADDR,
                                                regData);
        }

        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    return pmicStatus;
}

/*!
 * \brief   API to Get the timer interrupt period from PMIC RTC.
 *          This function is used to get the timer interrupt period from RTC
 *          present in the PMIC.
 *
 * \param   pPmicCoreHandle   [IN]     PMIC Interface Handle.
 * \param   pTimerPeriod      [OUT]    Timer interrupt period
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code.
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t  Pmic_rtcGetTimerPeriod(Pmic_CoreHandle_t *pPmicCoreHandle,
                                uint8_t           *pTimerPeriod)
{
    int32_t  pmicStatus  = PMIC_ST_SUCCESS;
    uint8_t  regData     = 0U;

    if(NULL == pPmicCoreHandle)
    {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (false == pPmicCoreHandle->pPmic_SubSysInfo->rtcEnable))
    {
        pmicStatus = PMIC_ST_ERR_INV_DEVICE;
    }

    if(NULL == pTimerPeriod)
    {
        pmicStatus = PMIC_ST_ERR_NULL_PARAM;
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        /* Getiing timer period for timer interrupt */
        Pmic_criticalSectionStart(pPmicCoreHandle);

        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                            PMIC_RTC_INTERRUPTS_REGADDR,
                                            &regData);
        Pmic_criticalSectionStop(pPmicCoreHandle);

        if(PMIC_ST_SUCCESS == pmicStatus)
        {
            *pTimerPeriod = HW_REG_GET_FIELD(regData,
                                             PMIC_RTC_INTERRUPTS_EVERY);
        }
    }

    return pmicStatus;
}

/*!
 * \brief   API to Set the RTC Time and Date to PMIC RTC.
 *          This function is used to set the RTC Date and Time parameters
 *          depending upon the bit fields set in validParams of Time and Date
 *          structures in RTC of PMIC Device.
 *
 * \param   pPmicCoreHandle   [IN]    PMIC Interface Handle.
 * \param   timeCfg           [IN]    PMIC RTC time configuration
 * \param   dateCfg           [IN]    PMIC RTC date configuration
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code.
 *          For valid values \ref Pmic_ErrorCodes.
 */
int32_t Pmic_rtcSetTimeDateInfo(Pmic_CoreHandle_t    *pPmicCoreHandle,
                                const Pmic_RtcTime_t  timeCfg,
                                const Pmic_RtcDate_t  dateCfg)
{
    int32_t  pmicStatus  = PMIC_ST_SUCCESS;

    if(NULL == pPmicCoreHandle)
    {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (false == pPmicCoreHandle->pPmic_SubSysInfo->rtcEnable))
    {
        pmicStatus = PMIC_ST_ERR_INV_DEVICE;
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       ((0U == timeCfg.validParams)  &&
        (0U == dateCfg.validParams)))
    {
        pmicStatus = PMIC_ST_ERR_INSUFFICIENT_CFG;
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        /* Caliing the function to validate the time and date for errors */
        pmicStatus = Pmic_rtcCheckDateTime(pPmicCoreHandle, timeCfg, dateCfg);
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        /* Stop RTC */
        pmicStatus = Pmic_rtcEnable(pPmicCoreHandle, PMIC_RTC_STOP);
    }

    /* Set PMIC RTC Time */
    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        pmicStatus = Pmic_rtcSetTime(pPmicCoreHandle, timeCfg);
    }

    /* Set PMIC RTC Date */
    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        pmicStatus = Pmic_rtcSetDate(pPmicCoreHandle, dateCfg);
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        /* Restarting the RTC */
        pmicStatus = Pmic_rtcEnable(pPmicCoreHandle, PMIC_RTC_START);
    }

    return pmicStatus;
}

/*!
 * \brief   API to Get the RTC Time and Date from PMIC RTC function.
 *          This function is used to Get the RTC date and time parameters
 *          depending upon the bit fields set in validParams of Time and Date
 *          structures in RTC of the PMIC Device.
 *
 * \param   pPmicCoreHandle   [IN]     PMIC Interface Handle.
 * \param   pTimeCfg          [OUT]    PMIC RTC time configuration
 * \param   pDateCfg          [OUT]    PMIC RTC date configuration
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code.
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_rtcGetTimeDateInfo(Pmic_CoreHandle_t *pPmicCoreHandle,
                                Pmic_RtcTime_t    *pTimeCfg,
                                Pmic_RtcDate_t    *pDateCfg)
{
    int32_t  pmicStatus = PMIC_ST_SUCCESS;

    if(NULL == pPmicCoreHandle)
    {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (false == pPmicCoreHandle->pPmic_SubSysInfo->rtcEnable))
    {
        pmicStatus = PMIC_ST_ERR_INV_DEVICE;
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       ((NULL == pTimeCfg) || (NULL == pDateCfg)))
    {
        pmicStatus = PMIC_ST_ERR_NULL_PARAM;
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       ((0U == pTimeCfg->validParams)  &&
        (0U == pDateCfg->validParams)))
    {
        pmicStatus = PMIC_ST_ERR_INSUFFICIENT_CFG;
    }

    /* Set RTC dynamic registers to static shadowed registers */
    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        pmicStatus = Pmic_rtcTriggerShadowRegisters(pPmicCoreHandle);
    }

    /* Get RTC time */
    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        pmicStatus = Pmic_rtcGetTime(pPmicCoreHandle, pTimeCfg);
    }

    /* Get RTC Date */
    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        pmicStatus = Pmic_rtcGetDate(pPmicCoreHandle, pDateCfg);
    }

    return (pmicStatus);
}

/*!
 * \brief   API to Set the RTC frequency compensation value.
 *          This function is used to set the frequency compensation
 *          value in the RTC of the PMIC Devicec.
 *
 * \param   pPmicCoreHandle   [IN]    PMIC Interface Handle.
 * \param   compensation      [IN]    PMIC RTC frequency compensation value
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code.
 *          For valid values \ref Pmic_ErrorCodes
 */

int32_t  Pmic_rtcSetFreqComp(Pmic_CoreHandle_t *pPmicCoreHandle,
                             const uint16_t     compensation)
{
    int32_t  pmicStatus  = PMIC_ST_SUCCESS;

    if(NULL == pPmicCoreHandle)
    {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (false == pPmicCoreHandle->pPmic_SubSysInfo->rtcEnable))
    {
        pmicStatus = PMIC_ST_ERR_INV_DEVICE;
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        pmicStatus = Pmic_rtcSetFreqCompensateVal(pPmicCoreHandle,
                                                  compensation);
    }

    return pmicStatus;
}

/*!
 * \brief   API to Get the RTC frequency compensation value.
 *          This function is used to get the frequency compensation
 *          value from the RTC of the PMIC Devicec.
 *
 * \param   pPmicCoreHandle   [IN]    PMIC Interface Handle.
 * \param   pCompensation     [OUT]   Pointer to store frequency compensation
 *                                    value
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code.
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t  Pmic_rtcGetFreqComp(Pmic_CoreHandle_t *pPmicCoreHandle,
                             uint16_t          *pCompensation)
{
    int32_t pmicStatus  = PMIC_ST_SUCCESS;

    if(NULL == pPmicCoreHandle)
    {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (false == pPmicCoreHandle->pPmic_SubSysInfo->rtcEnable))
    {
        pmicStatus = PMIC_ST_ERR_INV_DEVICE;
    }

    if(NULL == pCompensation)
    {
        pmicStatus = PMIC_ST_ERR_NULL_PARAM;
    }

    /* Get RTC compensation value */
    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        pmicStatus = Pmic_rtcGetFreqCompensateVal(pPmicCoreHandle,
                                                  pCompensation);
    }

    return pmicStatus;
}

/*!
 * \brief   API to Enable/Disable the RTC.
 *          This function is used to Start/Stop the RTC present in PMIC.
 *
 * \param   pPmicCoreHandle   [IN]    PMIC Interface Handle.
 * \param   enableRtc         [IN]    Parameter to start/stop RTC.
 *                                    Valid values: \ref Pmic_RtcState
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code.
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t  Pmic_rtcEnable(Pmic_CoreHandle_t *pPmicCoreHandle,
                        bool               enableRtc)
{
    int32_t  pmicStatus  = PMIC_ST_SUCCESS;

    if(NULL == pPmicCoreHandle)
    {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (false == pPmicCoreHandle->pPmic_SubSysInfo->rtcEnable))
    {
        pmicStatus = PMIC_ST_ERR_INV_DEVICE;
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        pmicStatus = Pmic_rtcEnableRtc(pPmicCoreHandle, enableRtc);
    }

    return pmicStatus;
}

/*!
 * \brief   API to Enable/Disable the RTC Alarm Interrupt.
 *          This function is used to enable/disable the RTC alarm interrupt.
 *
 * \param   pPmicCoreHandle   [IN]    PMIC Interface Handle.
 * \param   enableIntr        [IN]    Parameter to enable/disable Alarm
 *                                    Interrupt.
 *                                    For Valid values:
 *                                        \ref Pmic_RtcAlramIntrEnable
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code.
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t  Pmic_rtcEnableAlarmIntr(Pmic_CoreHandle_t *pPmicCoreHandle,
                                 bool               enableIntr)
{
    int32_t  pmicStatus  = PMIC_ST_SUCCESS;
    /* Flag to define Critical section started or not */

    if(NULL == pPmicCoreHandle)
    {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (false == pPmicCoreHandle->pPmic_SubSysInfo->rtcEnable))
    {
        pmicStatus = PMIC_ST_ERR_INV_DEVICE;
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        /* En/Disabling the RTC Alarm Intr */
        pmicStatus = Pmic_setAlarmIntr(pPmicCoreHandle, enableIntr);
    }

    return pmicStatus;
}

/*!
 * \brief   API to Enable/Disable the RTC Timer Interrupt.
 *          This function is used to enable/disable the RTC timer interrupt.
 *
 * \param   pPmicCoreHandle   [IN]    PMIC Interface Handle.
 * \param   enableIntr        [IN]    Parameter to enable/disable Timer
 *                                    Interrupt.
 *                                    For Valid values:
 *                                        \ref Pmic_RtcTimerIntrEnable
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code.
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t  Pmic_rtcEnableTimerIntr(Pmic_CoreHandle_t *pPmicCoreHandle,
                                 bool               enableIntr)
{
    int32_t  pmicStatus  = PMIC_ST_SUCCESS;

    if(NULL == pPmicCoreHandle)
    {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (false == pPmicCoreHandle->pPmic_SubSysInfo->rtcEnable))
    {
        pmicStatus = PMIC_ST_ERR_INV_DEVICE;
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        /* En/Disabling the RTC Timer Intr */
        pmicStatus = Pmic_setTimerIntr(pPmicCoreHandle, enableIntr);
    }

    return pmicStatus;
}

/*!
 * \brief   API to Get the current status of RTC.
 *          This function is used to get the Current state of the RTC
 *          depending on the bit fields set in validParams of
 *          struct Pmic_RtcStatus_t structures.
 *
 * \param   pPmicCoreHandle   [IN]    PMIC Interface Handle.
 * \param   pPmicRtcStatus    [OUT]   Parameter to hold RTC status.
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code.
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t  Pmic_getRtcStatus(Pmic_CoreHandle_t *pPmicCoreHandle,
                           Pmic_RtcStatus_t  *pPmicRtcStatus)
{
    int32_t  pmicStatus  = PMIC_ST_SUCCESS;
    uint8_t  regData     = 0U;

    if(NULL == pPmicCoreHandle)
    {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    }

    if((PMIC_ST_SUCCESS == pmicStatus) && (NULL == pPmicRtcStatus))
    {
         pmicStatus = PMIC_ST_ERR_NULL_PARAM;
    }

    if((PMIC_ST_SUCCESS == pmicStatus) && (0U == pPmicRtcStatus->validParams))
    {
         pmicStatus = PMIC_ST_ERR_INSUFFICIENT_CFG;
    }


    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        Pmic_criticalSectionStart(pPmicCoreHandle);

        /* Checking RTC status */
        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                            PMIC_RTC_STATUS_REGADDR,
                                            &regData);

        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        /* Get RTC Status */
        if(0U != (pmic_validParamCheck(pPmicRtcStatus->validParams,
                                       PMIC_RTC_CFG_RTC_STATUS_VALID)))
        {
            if(HW_REG_GET_FIELD(regData, PMIC_RTC_STATUS_RUN) == 0U)
            {
                pPmicRtcStatus->rtcStatus = (bool)false;
            }
            else
            {
                pPmicRtcStatus->rtcStatus = (bool)true;
            }
        }

        /* Get RTC POWER-UP status */
        if(0U != (pmic_validParamCheck(pPmicRtcStatus->validParams,
                                       PMIC_RTC_CFG_POWERUP_STATUS_VALID)))
        {
            if(HW_REG_GET_FIELD(regData, PMIC_RTC_STATUS_POWER_UP) == 0U)
            {
                pPmicRtcStatus->rtcStatus = (bool)false;
            }
            else
            {
                pPmicRtcStatus->rtcStatus = (bool)true;
            }
        }
    }

    return pmicStatus;
}
