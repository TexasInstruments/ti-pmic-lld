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
 *      This Module explains PMIC RTC APIs and their usage. This Module
 *  has APIs to cover all PMIC RTC features. like, set/get RTC time, Alarm
 *  time, RTC frequncy compensation, timer interrupt period and enable or
 *  disable RTC and RTC/Alarm interrupts.
 *
 *  Supported PMIC Devices for RTC Module:
 *  1. TPS6594x (Leo PMIC Device)
 *
 *  @{
 */

/**
 * \file   pmic_rtc.h
 *
 * \brief  PMIC Driver RTC API/interface file
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
 *  \name PMIC RTC timer interrupt Period Values.
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
#define PMIC_RTC_ALARM_INTR_ENABLE                      1U
#define PMIC_RTC_ALARM_INTR_DISABLE                     0U
/* @} */

/**
 *  \anchor Pmic_RtcTimerIntrEnable
 *  \name PMIC RTC timer interrupt Enable/Disable
 *
 *  @{
 */
#define PMIC_RTC_TIMER_INTR_ENABLE                      1U
#define PMIC_RTC_TIMER_INTR_DISABLE                     0U
/* @} */

/**
 *  \anchor Pmic_RtcTimeMode
 *  \name PMIC RTC Time Mode
 *
 *  @{
 */
#define PMIC_RTC_24_HOUR_MODE                          (0x0U)
#define PMIC_RTC_12_HOUR_MODE                          (0x1U)
/* @} */

/**
 *  \anchor Pmic_RtcMeridienMode
 *  \name PMIC RTC Meridien Mode
 *
 *  @{
 */
#define PMIC_RTC_AM_MODE                               (0x0U)
#define PMIC_RTC_PM_MODE                               (0x1U)
/* @} */

/**
 *  \anchor Pmic_RtcState
 *  \name PMIC RTC State to START/STOP RTC
 *
 *  @{
 */
#define PMIC_RTC_STOP                                   0U
#define PMIC_RTC_START                                  1U
/* @} */

/**
 *  \anchor Pmic_RtcStatus
 *  \name PMIC RTC Current Status
 *
 *  @{
 */
#define PMIC_RTC_STATUS_FROZEN                         (0x0U)
#define PMIC_RTC_STATUS_RUNNING                        (0x1U)
/* @} */

/**
 *  \anchor Pmic_RtcPowerUpStatus
 *  \name PMIC RTC power-up Status
 *
 *  @{
 */
#define PMIC_RTC_STATUS_POWERUP_UNSET                  (0x0U)
#define PMIC_RTC_STATUS_POWERUP_SET                    (0x1U)
/* @} */

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
 *  \name PMIC RTC Time strcture Param Bit Positions
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
 *  \name PMIC RTC Date strcture Param Bit Positions
 *
 *  @{
 */
#define PMIC_RTC_DATE_CFG_DAY_VALID       (0U)
#define PMIC_RTC_DATE_CFG_MONTH_VALID     (1U)
#define PMIC_RTC_DATE_CFG_YEAR_VALID      (2U)
#define PMIC_RTC_DATE_CFG_WEEKDAY_VALID   (3U)
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
#define PMIC_RTC_DATE_CFG_DAY_VALID_SHIFT     \
                                    (1U << PMIC_RTC_DATE_CFG_DAY_VALID)
#define PMIC_RTC_DATE_CFG_MONTH_VALID_SHIFT   \
                                    (1U << PMIC_RTC_DATE_CFG_MONTH_VALID)
#define PMIC_RTC_DATE_CFG_YEAR_VALID_SHIFT    \
                                    (1U << PMIC_RTC_DATE_CFG_YEAR_VALID)
#define PMIC_RTC_DATE_CFG_WEEKDAY_VALID_SHIFT \
                                    (1U << PMIC_RTC_DATE_CFG_WEEKDAY_VALID)
/* @} */

/**
 *  \anchor Pmic_RtcStatusValidParamCfg
 *  \name PMIC RTC Status strcture Param Bit Positions
 *
 *  @{
 */
#define PMIC_RTC_CFG_RTC_STATUS_VALID     (0U)
#define PMIC_RTC_CFG_POWERUP_STATUS_VALID (1U)
/* @} */

/**
 *  \anchor Pmic_RtcStatusValidParamBits
 *  \name   PMIC RTC Status Structure Param Bit shift values
 *
 *  Application can use below shifted values to set the validParam
 *  member defined in Pmic_RtcStatus_t structure
 *
 *  @{
 */
#define PMIC_RTC_CFG_RTC_STATUS_VALID_SHIFT     \
                     (1U << PMIC_RTC_CFG_RTC_STATUS_VALID)
#define PMIC_RTC_CFG_POWERUP_STATUS_VALID_SHIFT \
                     (1U << PMIC_RTC_CFG_POWERUP_STATUS_VALID)
/* @} */

/*==========================================================================*/
/*                         Structures and Enums                             */
/*==========================================================================*/

 /*!
 *  \brief  RTC time configuration.
 *          The Pmic_RtcTime_s structure contains set of time parameters to
 *          set/get the RTC time.
 *
 *  \param   validParams        Validate params Bits.
 *                              Depending on the parameters want to get/set,
 *                              corresponding bits should be set in validParam.
 *                                For valid values:
 *                                \ref Pmic_RtcTimeValidParamBits
 *  \param   seconds            Value to represent Seconds.
 *                                Valid only when PMIC_RTC_TIME_CFG_SEC_VALID
 *                                bit of validParams is set.
 *  \param   minutes            Value to represent Minutes.
 *                                Valid only when PMIC_RTC_TIME_CFG_MIN_VALID
 *                                bit of validParams is set.
 *  \param   hour               Value to represent Hours.
 *                                Valid only when PMIC_RTC_TIME_CFG_HRS_VALID
 *                                bit of validParams is set.
 *  \param   timeMode           Value to represent Time Mode.
 *                                For valid values: \ref Pmic_RtcTimeMode.
 *                                Valid only when
 *                                PMIC_RTC_TIME_CFG_TIMEMODE_VALID bit of
 *                                validParams is set.
 *  \param   meridianMode       Value to represent Maridian Mode.
 *                                For valid values: \ref Pmic_RtcMeridienMode.
 *                                Valid only when
 *                                PMIC_RTC_TIME_CFG_MERIDIAN_VALID bit of
 *                                validParams is set.
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
 *  \brief   RTC Date configuration.
 *           The Pmic_RtcDate_s structure contains set of date parameters to
 *           set/get the RTC Date.
 *
 *  \param   validParams        Validate params Bits.
 *                              Depending on the parameters want to get/set,
 *                              corresponding bits should be set in validParam.
 *  \param   day                Value to represent the day.
 *                                Valid only when PMIC_RTC_DATE_CFG_DAY_VALID
 *                                bit of validParams is set.
 *  \param   month              Value to represent the Month.
 *                                  For valid values \ref Pmic_RtcMonth.
 *                                Valid only when PMIC_RTC_DATE_CFG_MONTH_VALID
 *                                bit of validParams is set.
 *  \param   year               Value to represent the Year.
 *                                Valid only when PMIC_RTC_DATE_CFG_YEAR_VALID
 *                                bit of validParams is set.
 *  \param   weekday            Value to represent the weekday of the week.
 *                              For Valid Values: \ref Pmic_RtcWeekDay.
 *                                Valid only when
 *                                PMIC_RTC_DATE_CFG_WEEKDAY_VALID
 *                                bit of validParams is set.
 *
 */
typedef struct Pmic_RtcDate_s
{
    uint32_t validParams;
    uint8_t  day;
    uint8_t  month;
    uint16_t year;
    uint8_t  weekday;
}Pmic_RtcDate_t;

/*!
 *  \brief   RTC Live Status
 *           The Pmic_RtcStatus_s structure contains status of RTC and
 *           power-up status.
 *
 *  \param   validParams        Validate params Bits.
 *                              Depending on the parameters want to get,
 *                              corresponding bits should be set in validParam.
 *  \param   rtcStatus          Value of current status of RTC.
 *                                Valid only when PMIC_RTC_CFG_RTC_STATUS_VALID
 *                                bit of validParams is set.
 *                                  For valid values
 *                                      \ref Pmic_RtcStatus
 *  \param   powerupStatus      Value of power-up status of RTC.
 *                                Valid only when
 *                                PMIC_RTC_CFG_POWERUP_STATUS_VALID
 *                                bit of validParams is set.
 *                                  For valid values
 *                                      \ref Pmic_RtcPowerUpStatus
 */
typedef struct Pmic_RtcStatus_s
{
    uint32_t validParams;
    bool     rtcStatus;
    bool     powerupStatus;
}Pmic_RtcStatus_t;

/*==========================================================================*/
/*                         Function Declarations                            */
/*==========================================================================*/

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
                              const Pmic_RtcDate_t  dateCfg);

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
                              Pmic_RtcDate_t    *pDateCfg);

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
                                const uint8_t      timerPeriod);

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
                                uint8_t           *pTimerPeriod);

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
int32_t  Pmic_rtcSetTimeDateInfo(Pmic_CoreHandle_t    *pPmicCoreHandle,
                                 const Pmic_RtcTime_t  timeCfg,
                                 const Pmic_RtcDate_t  dateCfg);

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
int32_t  Pmic_rtcGetTimeDateInfo(Pmic_CoreHandle_t *pPmicCoreHandle,
                                 Pmic_RtcTime_t    *pTimeCfg,
                                 Pmic_RtcDate_t    *pDateCfg);

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
                             const uint16_t     compensation);

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
                             uint16_t          *pCompensation);

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
                        bool               enableRtc);

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
                                 bool               enableIntr);

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
                                 bool               enableIntr);

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
                           Pmic_RtcStatus_t  *pPmicRtcStatus);

#ifdef __cplusplus
}

#endif /* __cplusplus */

#endif /* PMIC_RTC_H_ */

/* @} */
