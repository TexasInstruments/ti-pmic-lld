/******************************************************************************
 * Copyright (c) 2023-2024 Texas Instruments Incorporated - http://www.ti.com
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
 * \file   pmic_rtc_priv.h
 *
 * \brief  The macro definitions, structures and function prototypes for
 *         configuring PMIC RTC
 */

#ifndef PMIC_RTC_PRIV_H_
#define PMIC_RTC_PRIV_H_

/* ========================================================================= */
/*                             Include Files                                 */
/* ========================================================================= */
#include "pmic_core_priv.h"

#ifdef __cplusplus
extern "C"
{
#endif

/* ========================================================================= */
/*                             Macros & Typedefs                             */
/* ========================================================================= */

/*!
 * \brief   PMIC RTC time and date Register Address
 */
#define PMIC_RTC_SECONDS_REGADDR                     (0xB5U)
#define PMIC_RTC_MINUTES_REGADDR                     (0xB6U)
#define PMIC_RTC_HOURS_REGADDR                       (0xB7U)
#define PMIC_RTC_DAYS_REGADDR                        (0xB8U)
#define PMIC_RTC_MONTHS_REGADDR                      (0xB9U)
#define PMIC_RTC_YEARS_REGADDR                       (0xBAU)
#define PMIC_RTC_WEEKS_REGADDR                       (0xBBU)

/*!
 * \brief  PMIC Alarm time and date Register Address
 */
#define PMIC_ALARM_SECONDS_REGADDR                   (0xBCU)
#define PMIC_ALARM_MINUTES_REGADDR                   (0xBDU)
#define PMIC_ALARM_HOURS_REGADDR                     (0xBEU)
#define PMIC_ALARM_DAYS_REGADDR                      (0xBFU)
#define PMIC_ALARM_MONTHS_REGADDR                    (0xC0U)
#define PMIC_ALARM_YEARS_REGADDR                     (0xC1U)

/*!
 * \brief  PMIC RTC Control, Status and Interrupt Register Address
 */
#define PMIC_RTC_CTRL_1_REGADDR                      (0xC2U)
#define PMIC_RTC_CTRL_2_REGADDR                      (0xC3U)
#define PMIC_RTC_STATUS_REGADDR                      (0xC4U)
#define PMIC_RTC_INTERRUPTS_REGADDR                  (0xC5U)

/*!
 * \brief  PMIC RTC frequency compensation register address
 */
#define PMIC_RTC_COMP_LSB_REGADDR                    (0xC6U)
#define PMIC_RTC_COMP_MSB_REGADDR                    (0xC7U)

/*!
 * \brief  PMIC RTC reset status register address
 */
#define PMIC_RTC_RESET_STATUS_REGADDR                (0xC8U)

/*!
 * \brief  PMIC ALARM_SECONDS register bit positions
 */
#define PMIC_ALR_SECOND_1_SHIFT        (0x04U)
#define PMIC_ALR_SECOND_0_SHIFT        (0x00U)
/*!
 * \brief  PMIC ALARM_MINUTES register bit positions
 */
#define PMIC_ALR_MINUTE_1_SHIFT        (0x04U)
#define PMIC_ALR_MINUTE_0_SHIFT        (0x00U)
/*!
 * \brief  PMIC ALARM_HOURS register bit positions
 */
#define PMIC_ALR_PM_NAM_SHIFT            (0x07U)
#define PMIC_ALR_HOUR_1_SHIFT            (0x04U)
#define PMIC_ALR_HOUR_0_SHIFT            (0x00U)
/*!
 * \brief  PMIC ALARM_DAYS register bit positions
 */
#define PMIC_ALR_DAY_1_SHIFT              (0x04U)
#define PMIC_ALR_DAY_0_SHIFT              (0x00U)
/*!
 * \brief  PMIC ALARM_MONTHS register bit positions
 */
#define PMIC_ALR_MONTH_1_SHIFT          (0x04U)
#define PMIC_ALR_MONTH_0_SHIFT          (0x00U)
/*!
 * \brief  PMIC ALARM_YEARS register bit positions
 */
#define PMIC_ALR_YEAR_1_SHIFT            (0x04U)
#define PMIC_ALR_YEAR_0_SHIFT            (0x00U)

/*!
 * \brief  PMIC RTC_SECONDS register bit positions
 */
#define PMIC_RTC_SECOND_1_SHIFT              (0x04U)
#define PMIC_RTC_SECOND_0_SHIFT              (0x00U)
/*!
 * \brief  PMIC RTC_MINUTES register bit positions
 */
#define PMIC_RTC_MINUTE_1_SHIFT              (0x04U)
#define PMIC_RTC_MINUTE_0_SHIFT              (0x00U)
/*!
 * \brief  PMIC RTC_HOURS register bit positions
 */
#define PMIC_RTC_PM_NAM_SHIFT                  (0x07U)
#define PMIC_RTC_HOUR_1_SHIFT                  (0x04U)
#define PMIC_RTC_HOUR_0_SHIFT                  (0x00U)
/*!
 * \brief  PMIC RTC_DAYS register bit positions
 */
#define PMIC_RTC_DAY_1_SHIFT                    (0x04U)
#define PMIC_RTC_DAY_0_SHIFT                    (0x00U)
/*!
 * \brief  PMIC RTC_MONTHS register bit positions
 */
#define PMIC_RTC_MONTH_1_SHIFT                (0x04U)
#define PMIC_RTC_MONTH_0_SHIFT                (0x00U)
/*!
 * \brief  PMICRTC_YEARS register bit positions
 */
#define PMIC_RTC_YEAR_1_SHIFT                  (0x04U)
#define PMIC_RTC_YEAR_0_SHIFT                  (0x00U)
/*!
 * \brief  PMIC RTC_WEEKS register bit positions
 */
#define PMIC_RTC_WEEK_SHIFT                    (0x00U)

/*!
 * \brief  PMIC RTC_STATUS register bit positions
 */
#define PMIC_RTC_ALARM_SHIFT       (0x06U)
#define PMIC_RTC_TIMER_SHIFT       (0x05U)
#define PMIC_RTC_RUN_SHIFT         (0x01U)
#define PMIC_RTC_POWER_UP_SHIFT    (0x07U)

/*!
 * \brief  PMIC RTC_INTERRUPTS register bit positions
 */
#define PMIC_RTC_IT_ALARM_SHIFT           (0x03U)
#define PMIC_RTC_IT_TIMER_SHIFT           (0x02U)
#define PMIC_RTC_EVERY_SHIFT              (0x00U)

/*!
 * \brief  PMIC RTC_CTRL_1 register bit positions
 */
#define PMIC_RTC_V_OPT_SHIFT              (0x07U)
#define PMIC_GET_TIME_SHIFT               (0x06U)
#define PMIC_SET_32_COUNTER_SHIFT         (0x05U)
#define PMIC_MODE_12_24_SHIFT             (0x03U)
#define PMIC_AUTO_COMP_SHIFT              (0x02U)
#define PMIC_ROUND_30S_SHIFT              (0x01U)
#define PMIC_STOP_RTC_SHIFT               (0x00U)

/*!
 * \brief  PMIC RTC_CTRL_2 register bit positions
 */
#define PMIC_XTAL_SEL_SHIFT               (0x01U)
#define PMIC_XTAL_EN_SHIFT                (0x00U)

/*!
 * \brief  PMIC RTC_RESET_STATUS register bit positions 
 */
#define PMIC_RESET_STATUS_RTC_SHIFT (0x00U)

/*!
 * \brief  PMIC RTC_COMP_LSB and RTC_COMP_MSB register shifts
 */
#define PMIC_COMP_MSB_RTC_SHIFT         (0x08U)
#define PMIC_COMP_LSB_RTC_SHIFT         (0x00U)

/*!
 * \brief   PMIC RTC_SECONDS register bit masks
 */
#define PMIC_RTC_SECOND_1_MASK               (uint8_t)(0x07U << PMIC_RTC_SECOND_1_SHIFT)
#define PMIC_RTC_SECOND_0_MASK               (uint8_t)(0x0FU << PMIC_RTC_SECOND_0_SHIFT)
/*!
 * \brief   PMIC RTC_MINUTES register bit masks
 */
#define PMIC_RTC_MINUTE_1_MASK               (uint8_t)(0x07U << PMIC_RTC_MINUTE_1_SHIFT)
#define PMIC_RTC_MINUTE_0_MASK               (uint8_t)(0x0FU << PMIC_RTC_MINUTE_0_SHIFT)
/*!
 * \brief   PMIC RTC_HOURS register bit masks
 */
#define PMIC_RTC_PM_NAM_MASK                   (uint8_t)(0x01U << PMIC_RTC_PM_NAM_SHIFT)
#define PMIC_RTC_HOUR_1_MASK                   (uint8_t)(0x03U << PMIC_RTC_HOUR_1_SHIFT)
#define PMIC_RTC_HOUR_0_MASK                   (uint8_t)(0x0FU << PMIC_RTC_HOUR_0_SHIFT)
/*!
 * \brief   PMIC RTC_DAYS register bit masks
 */
#define PMIC_RTC_DAY_1_MASK                     (uint8_t)(0x03U << PMIC_RTC_DAY_1_SHIFT)
#define PMIC_RTC_DAY_0_MASK                     (uint8_t)(0x0FU << PMIC_RTC_DAY_0_SHIFT)
/*!
 * \brief   PMIC RTC_MONTHS register bit masks
 */
#define PMIC_RTC_MONTH_1_MASK                 (uint8_t)(0x01U << PMIC_RTC_MONTH_1_SHIFT)
#define PMIC_RTC_MONTH_0_MASK                 (uint8_t)(0x0FU << PMIC_RTC_MONTH_0_SHIFT)
/*!
 * \brief   PMIC RTC_YEARS register bit masks
 */
#define PMIC_RTC_YEAR_1_MASK                   (uint8_t)(0x0FU << PMIC_RTC_YEAR_1_SHIFT)
#define PMIC_RTC_YEAR_0_MASK                   (uint8_t)(0x0FU << PMIC_RTC_YEAR_0_SHIFT)
/*!
 * \brief   PMIC ALARM_SECONDS register bit masks
 */
#define PMIC_ALR_SECOND_1_MASK         (uint8_t)(0x07U << PMIC_ALR_SECOND_1_SHIFT)
#define PMIC_ALR_SECOND_0_MASK         (uint8_t)(0x0FU << PMIC_ALR_SECOND_0_SHIFT)
/*!
 * \brief   PMIC ALARM_MINUTES register bit masks
 */
#define PMIC_ALR_MINUTE_1_MASK         (uint8_t)(0x07U << PMIC_ALR_MINUTE_1_SHIFT)
#define PMIC_ALR_MINUTE_0_MASK         (uint8_t)(0x0FU << PMIC_ALR_MINUTE_0_SHIFT)
/*!
 * \brief   PMIC ALARM_HOURS register bit masks
 */
#define PMIC_ALR_PM_NAM_MASK             (uint8_t)(0x01U << PMIC_ALR_PM_NAM_SHIFT)
#define PMIC_ALR_HOUR_1_MASK             (uint8_t)(0x03U << PMIC_ALR_HOUR_1_SHIFT)
#define PMIC_ALR_HOUR_0_MASK             (uint8_t)(0x0FU << PMIC_ALR_HOUR_0_SHIFT)
/*!
 * \brief   PMIC ALARM_DAYS register bit masks
 */
#define PMIC_ALR_DAY_1_MASK               (uint8_t)(0x03U << PMIC_ALR_DAY_1_SHIFT)
#define PMIC_ALR_DAY_0_MASK               (uint8_t)(0x0FU << PMIC_ALR_DAY_0_SHIFT)
/*!
 * \brief   PMIC ALARM_MONTHS register bit masks
 */
#define PMIC_ALR_MONTH_1_MASK           (uint8_t)(0x01U << PMIC_ALR_MONTH_1_SHIFT)
#define PMIC_ALR_MONTH_0_MASK           (uint8_t)(0x0FU << PMIC_ALR_MONTH_0_SHIFT)
/*!
 * \brief   PMIC ALARM_YEARS register bit masks
 */
#define PMIC_ALR_YEAR_1_MASK             (uint8_t)(0x0FU << PMIC_ALR_YEAR_1_SHIFT)
#define PMIC_ALR_YEAR_0_MASK             (uint8_t)(0x0FU << PMIC_ALR_YEAR_0_SHIFT)
/*!
 * \brief   PMIC RTC_WEEKS register bit masks
 */
#define PMIC_RTC_WEEK_MASK                     (uint8_t)(0x07U << PMIC_RTC_WEEK_SHIFT)

/*!
 * \brief   PMIC RTC_STATUS register bit masks
 */
#define PMIC_RTC_ALARM_MASK                   (uint8_t)(0x01U << PMIC_RTC_ALARM_SHIFT)
#define PMIC_RTC_TIMER_MASK                   (uint8_t)(0x01U << PMIC_RTC_TIMER_SHIFT)
#define PMIC_RTC_RUN_MASK                     (uint8_t)(0x01U << PMIC_RTC_RUN_SHIFT)
#define PMIC_RTC_POWER_UP_MASK                (uint8_t)(0x01U << PMIC_RTC_POWER_UP_SHIFT)

/*!
 * \brief   PMIC RTC_CTRL_1 register bit masks
 */
#define PMIC_RTC_V_OPT_MASK               (uint8_t)(0x01U << PMIC_RTC_V_OPT_SHIFT)
#define PMIC_GET_TIME_MASK                (uint8_t)(0x01U << PMIC_GET_TIME_SHIFT)
#define PMIC_SET_32_COUNTER_MASK          (uint8_t)(0x01U << PMIC_SET_32_COUNTER_SHIFT)
#define PMIC_MODE_12_24_MASK              (uint8_t)(0x01U << PMIC_MODE_12_24_SHIFT)
#define PMIC_ROUND_30S_MASK               (uint8_t)(0x01U << PMIC_ROUND_30S_SHIFT)
#define PMIC_STOP_RTC_MASK                (uint8_t)(0x01U << PMIC_STOP_RTC_SHIFT)
#define PMIC_AUTO_COMP_MASK               (uint8_t)(0x01U << PMIC_AUTO_COMP_SHIFT)
/*! 
 * \brief PMIC RTC_CTRL_2 register bit masks 
 */
#define PMIC_XTAL_SEL_MASK                (uint8_t)(0x03U << PMIC_XTAL_SEL_SHIFT)
#define PMIC_XTAL_EN_MASK                 (uint8_t)(0x01U << PMIC_XTAL_EN_SHIFT)

/*!
 * \brief   PMIC RTC_INTERRUPTS register bit masks
 */
#define PMIC_RTC_EVERY_MASK                 (uint8_t)(0x03U << PMIC_RTC_EVERY_SHIFT)
#define PMIC_RTC_IT_ALARM_MASK              (uint8_t)(0x01U << PMIC_RTC_IT_ALARM_SHIFT)
#define PMIC_RTC_IT_TIMER_MASK              (uint8_t)(0x01U << PMIC_RTC_IT_TIMER_SHIFT)

/*!
 * \brief  PMIC RTC_RESET_STATUS register bit masks 
 */
#define PMIC_RESET_STATUS_RTC_MASK          (uint8_t)(0x01U << PMIC_RESET_STATUS_RTC_SHIFT)

/*!
 * \brief  PMIC RTC_COMP_LSB and RTC_COMP_MSB bit masks
 */
#define PMIC_RTC_COMP_MSB_COMP_MSB_RTC_MASK          (uint16_t)((uint16_t)0xFFU << PMIC_COMP_MSB_RTC_SHIFT)
#define PMIC_RTC_COMP_LSB_COMP_LSB_RTC_MASK          (uint8_t)(0xFFU << PMIC_COMP_LSB_RTC_SHIFT)

/*!
 * \brief   RTC max value for Minutes and Seconds
 */
#define PMIC_RTC_MINUTE_SEC_MAX            (59U)

/*!
 * \brief   RTC 12 Hour Time values limit
 */
#define PMIC_RTC_12HFMT_HR_MIN             (1U)
#define PMIC_RTC_12HFMT_HR_MAX             (12U)

/*!
 * \brief   RTC 24 Hour Time values limit
 */
#define PMIC_RTC_24HFMT_HR_MAX             (23U)

/*!
 * \brief  RTC month min values
 */
#define PMIC_RTC_DAY_MIN                   (1U)

/*!
 * \brief   RTC years values limit
 */
#define PMIC_RTC_YEAR_MIN                  (2000U)
#define PMIC_RTC_YEAR_MAX                  (2099U)

/*!
 * \brief  RTC month max value for
 *         February month in a Non-leap year.
 */
#define PMIC_RTC_NLPY_FEB_MNTH_DAY_MAX     (28U)

/*!
 * \brief  RTC month max value for
 *         February month in a leap year.
 */
#define PMIC_RTC_LPY_FEB_MNTH_DAY_MAX      (29U)

/*!
 * \brief  RTC month max value for
 *         general months.
 */
#define PMIC_RTC_MNTH_DAY_MAX_30           (30U)
#define PMIC_RTC_MNTH_DAY_MAX_31           (31U)

/*!
 * \brief   Used to Extract the First and Second
 *          digits of RTC Timer/Alarm decimal Values
 *
 */
#define PMIC_RTC_CONVERT_4BIT_MSB_TO_DEC   (10U)

/*!
 * \brief   Used to Extract last two decimal digits of given Year.
 */
#define PMIC_RTC_EXTRACT_YEAR_DECIMAL_0_99 (100U)

/*!
 * \brief   RTC Auto Compensation ON Value
 */
#define PMIC_RTC_AUTO_COMP_ON              (0x1U)

/**
 *  \brief PMIC RTC Operations for RTC/ALARM
 */
#define PMIC_RTC_OPS_FOR_RTC               ((bool)false)
#define PMIC_RTC_OPS_FOR_ALARM             ((bool)true)

/*==========================================================================*/
/*                         Structures and Enums                             */
/*==========================================================================*/

/*==========================================================================*/
/*                         Function Declarations                            */
/*==========================================================================*/

#ifdef __cplusplus
}

#endif /* __cplusplus */

#endif /* PMIC_RTC_PRIV_H_ */
