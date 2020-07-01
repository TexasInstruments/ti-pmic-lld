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
 *  \file   pmic_ut_rtc.h
 *
 *  \brief  Header file for PMIC RTC Unit Tests
 *
 */

#include <pmic_ut_common.h>

/*!
 * \brief   RTC invalid date/time macros
 */
#define PMIC_RTC_INVALID_SEC_MINUTE     (61U)
#define PMIC_RTC_INVALID_TIME_MODE      (2U)
#define PMIC_RTC_INVALID_MERIDIEN_MODE  (2U)
#define PMIC_RTC_INVALID_DAY            (29U)
#define PMIC_RTC_INVALID_DAY_32         (32U)
#define PMIC_RTC_INVALID_HOUR_25        (25U)
#define PMIC_RTC_INVALID_DAY_31         (31U)
#define PMIC_RTC_INVALID_DAY_30         (30U)
#define PMIC_RTC_INVALID_YEAR           (3000U)
#define PMIC_RTC_INVALID_MONTH_0        (0U)
#define PMIC_RTC_INVALID_MONTH          (13U)
#define PMIC_RTC_INVALID_DAY_0          (0U)
#define PMIC_RTC_INVALID_HOUR_13        (13U)
#define PMIC_RTC_INVALID_HOUR_0         (0U)

/*!
 * \brief   RTC Date limit Macros
 */
#define PMIC_RTC_YEAR_2044              (2044U)
#define PMIC_RTC_YEAR_2045              (2045U)
#define PMIC_RTC_HOUR_0                 (0U)

/*!
 * \brief  PMIC RTC Valid Params configuration for Time and Date
 */
#define PMIC_RTC_VALID_PARAM_TIME_CFG_VAL  \
                                    (PMIC_RTC_TIME_CFG_SEC_VALID_SHIFT | \
                                     PMIC_RTC_TIME_CFG_MIN_VALID_SHIFT | \
                                     PMIC_RTC_TIME_CFG_HRS_VALID_SHIFT | \
                                     PMIC_RTC_TIME_CFG_TIMEMODE_VALID_SHIFT | \
                                     PMIC_RTC_TIME_CFG_MERIDIAN_VALID_SHIFT)

#define PMIC_RTC_VALID_PARAM_TIME_HRS_TIMEMODE_CFG_VAL  \
                                    (PMIC_RTC_TIME_CFG_HRS_VALID_SHIFT | \
                                     PMIC_RTC_TIME_CFG_TIMEMODE_VALID_SHIFT)

#define PMIC_RTC_VALID_PARAM_DATE_CFG_VAL  \
                                    (PMIC_RTC_DATE_CFG_DAY_VALID_SHIFT | \
                                     PMIC_RTC_DATE_CFG_MONTH_VALID_SHIFT | \
                                     PMIC_RTC_DATE_CFG_YEAR_VALID_SHIFT | \
                                     PMIC_RTC_DATE_CFG_WEEKDAY_VALID_SHIFT)

#define PMIC_RTC_VALID_PARAM_DATE_CFG_DAY_MNTH_VAL  \
                                    (PMIC_RTC_DATE_CFG_DAY_VALID_SHIFT | \
                                     PMIC_RTC_DATE_CFG_MONTH_VALID_SHIFT)

#define PMIC_RTC_VALID_PARAM_DATE_CFG_DAY_MNTH_YR_VAL  \
                                    (PMIC_RTC_DATE_CFG_DAY_VALID_SHIFT | \
                                     PMIC_RTC_DATE_CFG_MONTH_VALID_SHIFT | \
                                     PMIC_RTC_DATE_CFG_YEAR_VALID_SHIFT)

#define PMIC_RTC_TIMEOUT                (5U)

/*!
 * \brief   Number of RTC testcases
 */
#define PMIC_RTC_NUM_OF_TESTCASES     \
        (sizeof(pmic_rtc_tests)/sizeof(pmic_rtc_tests[0]))
