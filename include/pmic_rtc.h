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
 *  \ingroup DRV_PMIC_MODULE
 *  \defgroup DRV_PMIC_RTC_MODULE PMIC RTC Driver API
 *            These are PMIC RTC driver parameters and API's
 *
 *  @{
 */

/**
 * \file   pmic_rtc.h
 *
 * \brief  PMIC Low Level Driver API/interface file for RTC API
 */

#ifndef PMIC_RTC_H_
#define PMIC_RTC_H_

/* ========================================================================= */
/*                             Include Files                                 */
/* ========================================================================= */

#include <pmic_core.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================= */
/*                             Macros & Typedefs                             */
/* ========================================================================= */

/**
 *  \anchor Pmic_RtcTimerIntrPeriod
 *  \name PMIC RTC timer interrupt Periods
 *
 *  @{
 */
#define PMIC_RTC_SECOND_INTR_PERIOD                    (0x0U)
#define PMIC_RTC_MINUTE_INTR_PERIOD                    (0x1U)
#define PMIC_RTC_HOUR_INTR_PERIOD                      (0x2U)
#define PMIC_RTC_DAY_INTR_PERIOD                       (0x3U)
/* @} */

/**
 *  \anchor Pmic_RtcAlramIntrEnable
 *  \name PMIC RTC alarm interrupt Enable/Disable
 *
 *  @{
 */
#define PMIC_RTC_ALARM_INTR_ENABLE                     (0x1U)
#define PMIC_RTC_ALARM_INTR_DISABLE                    (0x0U)
/* @} */

/**
 *  \anchor Pmic_RtcTimerIntrEnable
 *  \name PMIC RTC timer interrupt Enable/Disable
 *
 *  @{
 */
#define PMIC_RTC_TIMER_INTR_ENABLE                     (0x1U)
#define PMIC_RTC_TIMER_INTR_DISABLE                    (0x0U)
/* @} */

/**
 *  \anchor Pmic_RtcTimeMode
 *  \name PMIC RTC time mode
 *
 *  @{
 */
#define PMIC_RTC_12_HOUR_MODE                          (0x1U)
#define PMIC_RTC_24_HOUR_MODE                          (0x0U)
/* @} */

/**
 *  \anchor Pmic_RtcOperationMode
 *  \name PMIC RTC Operations for RTC/ALARM
 *
 *  @{
 */
#define PMIC_RTC_OPS_FOR_RTC                           (0x0U)
#define PMIC_RTC_OPS_FOR_ALARM                         (0x1U)
/* @} */

/**
 *  \anchor Pmic_RtcMeridienMode
 *  \name PMIC RTC Meridien mode
 *
 *  @{
 */
#define PMIC_RTC_AM_MODE                               (0x0U)
#define PMIC_RTC_PM_MODE                               (0x1U)
/* @} */

/**
 *  \anchor Pmic_RtcState
 *  \name PMIC RTC State
 *
 *  @{
 */
#define PMIC_RTC_STOP                                  (0x0U)
#define PMIC_RTC_START                                 (0x1U)
/* @} */

/*!
 * \brief   RTC Compensation
 */
#define PMIC_RTC_AUTO_COMP_ON                          (0x1U)

/**
 *  \anchor Pmic_RtcTimeCfgLimit
 *  \name PMIC RTC time Configuration Limit Values
 *
 *  @{
 */
/*!
 * \brief   RTC max value for Minutes and Seconds
 */
#define PMIC_RTC_MINUTE_SEC_MAX                        (59U)

/*!
 * \brief   RTC 12 Hour Time values limit
 */
#define PMIC_RTC_12HFMT_HR_MIN                         (1U)
#define PMIC_RTC_12HFMT_HR_MAX                         (12U)

/*!
 * \brief   RTC 24 Hour Time values limit
 */
#define PMIC_RTC_24HFMT_HR_MAX                         (23U)
/* @} */

/**
 *  \anchor Pmic_RtcDateCfgLimit
 *  \name PMIC RTC Date Configuration Limit Values
 *
 *  @{
 */
/*!
 * \brief  RTC month min values
 */
#define PMIC_RTC_DAY_MIN                               (1U)

/*!
 * \brief   RTC years values limit
 */
#define PMIC_RTC_YEAR_MIN                              (2000U)
#define PMIC_RTC_YEAR_MAX                              (2099U)

/*!
 * \brief  RTC month max value for
 *         February month in a Non-leap year.
 */
#define PMIC_RTC_NLPY_FEB_MNTH_DAY_MAX                 (28U)

/*!
 * \brief  RTC month max value for
 *         February month in a leap year.
 */
#define PMIC_RTC_LPY_FEB_MNTH_DAY_MAX                  (29U)

/*!
 * \brief  RTC month max value for
 *         general months.
 */
#define PMIC_RTC_MNTH_DAY_MAX_30                       (30U)
#define PMIC_RTC_MNTH_DAY_MAX_31                       (31U)
/* @} */

/*!
 * \brief   Used to Extract the First and Second
 *          digit RTC Timer /Alarm decimal Values
 *
 */
#define PMIC_RTC_CONVERT_4BIT_MSB_TO_DEC               (10U)

/*!
 * \brief   Used to Extract Year decimal value as 0 to 99 from values 2000 to
 *          2099
 */
#define PMIC_RTC_EXTRACT_YEAR_DECIMAL_0_99             (100U)

/**
 *  \anchor Pmic_RtcWeekDay
 *  \name PMIC RTC Week Days
 *
 *  @{
 */
#define PMIC_RTC_WEEKDAY_SUNDAY      (1U)
#define PMIC_RTC_WEEKDAY_MONDAY      (2U)
#define PMIC_RTC_WEEKDAY_TUESDAY     (3U)
#define PMIC_RTC_WEEKDAY_WEDNESDAY   (4U)
#define PMIC_RTC_WEEKDAY_THURSDAY    (5U)
#define PMIC_RTC_WEEKDAY_FRIDAY      (6U)
#define PMIC_RTC_WEEKDAY_SATURDAY    (7U)
/* @} */

/**
 *  \anchor Pmic_RtcMonth
 *  \name PMIC RTC Months
 *
 *  List of calendar months.
 *
 *  @{
 */
#define PMIC_RTC_MONTH_JAN           (1U)
#define PMIC_RTC_MONTH_FEB           (2U)
#define PMIC_RTC_MONTH_MAR           (3U)
#define PMIC_RTC_MONTH_APR           (4U)
#define PMIC_RTC_MONTH_MAY           (5U)
#define PMIC_RTC_MONTH_JUN           (6U)
#define PMIC_RTC_MONTH_JUL           (7U)
#define PMIC_RTC_MONTH_AUG           (8U)
#define PMIC_RTC_MONTH_SEP           (9U)
#define PMIC_RTC_MONTH_OCT           (10U)
#define PMIC_RTC_MONTH_NOV           (11U)
#define PMIC_RTC_MONTH_DEC           (12U)
/* @} */

/**
 *  \anchor Pmic_RtcTimeValidParamCfg
 *  \name PMIC RTC Time strcture Param Bits
 *
 *  @{
 */
#define PMIC_RTC_TIME_CFG_SEC_VALID       (0U)
#define PMIC_RTC_TIME_CFG_MIN_VALID       (1U)
#define PMIC_RTC_TIME_CFG_HRS_VALID       (2U)
#define PMIC_RTC_TIME_CFG_TIMEMODE_VALID  (3U)
#define PMIC_RTC_TIME_CFG_MERIDIAN_VALID  (4U)
/* @} */

/**
 *  \anchor Pmic_RtcTimeValidParamBits
 *  \name   PMIC RTC Time Structure Param Bit shift values
 *
 *  Application can use below shifted values to set the validParam
 *  member defined in Pmic_RtcTime_t structure
 *
 *  @{
 */
#define PMIC_RTC_TIME_CFG_SEC_VALID_SHIFT       \
                                      (1U << PMIC_RTC_TIME_CFG_SEC_VALID)
#define PMIC_RTC_TIME_CFG_MIN_VALID_SHIFT       \
                                      (1U << PMIC_RTC_TIME_CFG_MIN_VALID)
#define PMIC_RTC_TIME_CFG_HRS_VALID_SHIFT       \
                                      (1U << PMIC_RTC_TIME_CFG_HRS_VALID)
#define PMIC_RTC_TIME_CFG_TIMEMODE_VALID_SHIFT  \
                                      (1U << PMIC_RTC_TIME_CFG_TIMEMODE_VALID)
#define PMIC_RTC_TIME_CFG_MERIDIAN_VALID_SHIFT  \
                                      (1U << PMIC_RTC_TIME_CFG_MERIDIAN_VALID)
/* @} */

/**
 *  \anchor Pmic_RtcDateValidParamCfg
 *  \name PMIC RTC Date strcture Param Bits
 *
 *  @{
 */
#define PMIC_RTC_DATE_CFG_DAY_VALID       (0U)
#define PMIC_RTC_DATE_CFG_MONTH_VALID     (1U)
#define PMIC_RTC_DATE_CFG_YEAR_VALID      (2U)
#define PMIC_RTC_DATE_CFG_WEEK_VALID      (3U)
/* @} */

/**
 *  \anchor Pmic_RtcDateValidParamBits
 *  \name   PMIC RTC Date Structure Param Bit shift values
 *
 *  Application can use below shifted values to set the validParam
 *  member defined in Pmic_RtcDate_t structure
 *
 *  @{
 */
#define PMIC_RTC_DATE_CFG_DAY_VALID_SHIFT  (1U << PMIC_RTC_DATE_CFG_DAY_VALID)
#define PMIC_RTC_DATE_CFG_MONTH_VALID_SHIFT   \
                                         (1U << PMIC_RTC_DATE_CFG_MONTH_VALID)
#define PMIC_RTC_DATE_CFG_YEAR_VALID_SHIFT (1U << PMIC_RTC_DATE_CFG_YEAR_VALID)
#define PMIC_RTC_DATE_CFG_WEEK_VALID_SHIFT (1U << PMIC_RTC_DATE_CFG_WEEK_VALID)
/* @} */

/*==========================================================================*/
/*                         Structures and Enums                             */
/*==========================================================================*/

 /*!
 *  \brief  RTC time configuration
 *          The Pmic_RtcTime_s structure contains a set of time parameters to
 *          the RTC time.
 *
 *  \param   validParams        Validate params Bits
 *                              Depending on the parameters want to get/set
 *                              corresponding bits should be set in validParam
 *                                For valid values:
 *                                \ref Pmic_RtcTimeValidParamBits
 *  \param   seconds            Time value in seconds.
 *                                Valid only when PMIC_RTC_TIME_CFG_SEC_VALID
 *                                bit of validParams is set
 *  \param   minutes            Time value in minutes
 *                                Valid only when PMIC_RTC_TIME_CFG_MIN_VALID
 *                                bit of validParams is set
 *  \param   hour               Time value in hour
 *                                Valid only when PMIC_RTC_TIME_CFG_HRS_VALID
 *                                bit of validParams is set
 *  \param   timeMode           Hour Mode
 *                                For valid values: \ref Pmic_RtcTimeMode
 *                                Valid only when
 *                                PMIC_RTC_TIME_CFG_TIMEMODE_VALID bit of
 *                                validParams is set
 *  \param   meridianMode       Meridian type
 *                                For valid values: \ref Pmic_RtcMeridienMode
 *                                Valid only when
 *                                PMIC_RTC_TIME_CFG_MERIDIAN_VALID bit of
 *                                validParams is set
 */
typedef struct Pmic_RtcTime_s
{
    uint32_t validParams;
    uint8_t seconds;
    uint8_t minutes;
    uint8_t hour;
    uint8_t timeMode;
    uint8_t meridianMode;
}Pmic_RtcTime_t;

/*!
 *  \brief   RTC Date configuration
 *           The Pmic_RtcDate_s structure contains a set of time parameters to
 *           characterize the RTC Date.
 *
 *  \param   validParams        Validate params Bits
 *                              Depending on the parameters want to get/set
 *                              corresponding bits should be set in validParam
 *  \param   day                Value to represent the day
 *                                Valid only when PMIC_RTC_DATE_CFG_DAY_VALID
 *                                bit of validParams is set
 *  \param   month              Value to represent the Month.
 *                                  For valid values
 *                                  \ref Pmic_RtcMonth
 *                                Valid only when PMIC_RTC_DATE_CFG_MONTH_VALID
 *                                bit of validParams is set
 *  \param   year               Value to represent the Year.
 *                              Supported range: \ref
 *                                  PMIC_RTC_YEAR_MIN (2000)
 *                                  PMIC_RTC_YEAR_MAX (2099)
 *                                Valid only when PMIC_RTC_DATE_CFG_YEAR_VALID
 *                                bit of validParams is set
 *  \param   week               Value to represent the week of the year
 *                                Valid only when PMIC_RTC_DATE_CFG_WEEK_VALID
 *                                bit of validParams is set
 *
 */
typedef struct Pmic_RtcDate_s
{
    uint32_t validParams;
    uint8_t  day;
    uint8_t  month;
    uint16_t year;
    uint8_t  week;
}Pmic_RtcDate_t;

/*==========================================================================*/
/*                         Function Declarations                            */
/*==========================================================================*/

/*!
 * \brief   Set the alarm interrupt configurations in PMIC RTC function.
 *          This function is used to set the alarm date and time interrupt
 *          depending upon the bit fields set in validParams of Time and Date
 *          structures in RTC present in the PMIC.
 *
 * \param   pPmicCoreHandle   [IN]    PMIC Interface Handle.
 * \param   pTimeCfg          [IN]    PMIC RTC time configuration
 * \param   pDateCfg          [IN]    PMIC RTC date configuration
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code.
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t  Pmic_rtcSetAlarmIntr(Pmic_CoreHandle_t *pPmicCoreHandle,
                              Pmic_RtcTime_t    *pTimeCfg,
                              Pmic_RtcDate_t    *pDateCfg);

/*!
 * \brief   Get the alarm interrupt configurations in PMIC RTC function.
 *          This function is used to Get the alarm date and time interrupt
 *          depending upon the bit fields set in validParams of Time and Date
 *          structures in RTC present in the PMIC.
 *
 * \param   pPmicCoreHandle   [IN]     PMIC Interface Handle.
 * \param   pTimeCfg          [OUT]    PMIC RTC time configuration
 * \param   pDateCfg          [OUT]    PMIC RTC date configuration
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code.
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t  Pmic_rtcGetAlarmIntr(Pmic_CoreHandle_t *pPmicCoreHandle,
                              Pmic_RtcTime_t    *pTimeCfg,
                              Pmic_RtcDate_t    *pDateCfg);

/*!
 * \brief   Set the timer interrupt configuration in PMIC RTC function.
 *          This function is used to set the timer interrupt in RTC present
 *          in the PMIC.
 *
 * \param   pPmicCoreHandle   [IN]    PMIC Interface Handle.
 * \param   timerPeriod       [IN]    Timer interrupt periods
 *                                      Valid values:
 *                                          \ref Pmic_RtcTimerIntrPeriod
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code.
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t  Pmic_rtcSetTimerIntr(Pmic_CoreHandle_t *pPmicCoreHandle,
                              uint8_t            timerPeriod);

/*!
 * \brief   Get the timer interrupt configuration in PMIC RTC function.
 *          This function is used to set the timer interrupt in RTC present
 *          in the PMIC.
 *
 * \param   pPmicCoreHandle   [IN]     PMIC Interface Handle.
 * \param   pTimerPeriod      [OUT]    Timer interrupt periods
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code.
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t  Pmic_rtcGetTimerIntr(Pmic_CoreHandle_t *pPmicCoreHandle,
                              uint8_t           *pTimerPeriod);

/*!
 * \brief   Set the PMIC RTC date and time function.
 *          This function is used to Set the current date and time parameters,
 *          depending upon the bit fields set in validParams of Time and Date
 *          structures in RTC present in the PMIC.
 *
 * \param   pPmicCoreHandle   [IN]   PMIC Interface Handle.
 * \param   pTimeCfg          [IN]   PMIC RTC time configuration
 * \param   pDateCfg          [IN]   PMIC RTC date configuration
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code.
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t  Pmic_rtcSetTimeDateInfo(Pmic_CoreHandle_t *pPmicCoreHandle,
                                 Pmic_RtcTime_t    *pTimeCfg,
                                 Pmic_RtcDate_t    *pDateCfg);

/*!
 * \brief   Get the PMIC RTC date and time function.
 *          This function is used to Get the current date and time parameters,
 *          depending upon the bit fields set in validParams of Time and Date
 *          structures in RTC present in the PMIC.
 *
 * \param   pPmicCoreHandle   [IN]    PMIC Interface Handle.
 * \param   pTimeCfg          [OUT]   PMIC RTC time configuration
 * \param   pDateCfg          [OUT]   PMIC RTC date configuration
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code.
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t  Pmic_rtcGetTimeDateInfo(Pmic_CoreHandle_t *pPmicCoreHandle,
                                 Pmic_RtcTime_t    *pTimeCfg,
                                 Pmic_RtcDate_t    *pDateCfg);

/*!
 * \brief   Set the RTC frequency compensation.
 *          This function is used to enable frequency compensation
 *          in RTC present in the PMIC.
 *
 * \param   pPmicCoreHandle   [IN]    PMIC Interface Handle.
 * \param   compensation      [IN]    PMIC RTC time configuration
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code.
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t  Pmic_rtcSetFreqComp(Pmic_CoreHandle_t *pPmicCoreHandle,
                             uint16_t           compensation);

/*!
 * \brief   Get the RTC frequency pCompensation.
 *          This function is used to get frequency pCompensation
 *          value in RTC present in the PMIC.
 *
 * \param   pPmicCoreHandle   [IN]    PMIC Interface Handle.
 * \param   pCompensation     [OUT]   PMIC RTC time configuration
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code.
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t  Pmic_rtcGetFreqComp(Pmic_CoreHandle_t *pPmicCoreHandle,
                             uint16_t          *pCompensation);

/*!
 * \brief   Enable/Disable the RTC.
 *          This function is used to stop the RTC present in the PMIC.
 *
 * \param   pPmicCoreHandle   [IN]    PMIC Interface Handle.
 * \param   enableRtc         [IN]    Parameter to start/stop RTC.
 *                                    Valid values: \ref Pmic_RtcState
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code.
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t  Pmic_rtcEnable(Pmic_CoreHandle_t *pPmicCoreHandle,
                        uint8_t            enableRtc);

/*!
 * \brief   Enable/Disable the RTC Timer Interrupt.
 *          This function is used to enable/disable the RTC timer interrupt.
 *
 * \param   pPmicCoreHandle   [IN]    PMIC Interface Handle.
 * \param   enableIntr        [IN]    Parameter to enable/disable Timer
 *                                    Interrupt
 *                                    Valid values: \ref Pmic_RtcTimerIntrEnable
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code.
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t  Pmic_rtcEnableTimerIntr(Pmic_CoreHandle_t *pPmicCoreHandle,
                                 bool               enableIntr);

/*!
 * \brief   Enable/Disable the RTC Alarm Interrupt.
 *          This function is used to enable/disable the RTC alarm interrupt.
 *
 * \param   pPmicCoreHandle   [IN]    PMIC Interface Handle.
 * \param   enableIntr        [IN]    Parameter to enable/disable alarm
 *                                    Interrupt
 *                                    Valid values: \ref Pmic_RtcAlramIntrEnable
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code.
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t  Pmic_rtcEnableAlarmIntr(Pmic_CoreHandle_t *pPmicCoreHandle,
                                 bool               enableIntr);


#ifdef __cplusplus
}

#endif /* __cplusplus */

#endif /* PMIC_RTC_H_ */

/* @} */
