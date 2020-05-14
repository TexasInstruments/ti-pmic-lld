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
#include <pmic_rtc.h>

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
                                     PMIC_RTC_DATE_CFG_WEEK_VALID_SHIFT)

#define PMIC_RTC_VALID_PARAM_DATE_CFG_DAY_MNTH_VAL  \
                                    (PMIC_RTC_DATE_CFG_DAY_VALID_SHIFT | \
                                     PMIC_RTC_DATE_CFG_MONTH_VALID_SHIFT)

#define PMIC_RTC_VALID_PARAM_DATE_CFG_DAY_MNTH_YR_VAL  \
                                    (PMIC_RTC_DATE_CFG_DAY_VALID_SHIFT | \
                                     PMIC_RTC_DATE_CFG_MONTH_VALID_SHIFT | \
                                     PMIC_RTC_DATE_CFG_YEAR_VALID_SHIFT)


/*!
 * \brief   RTC test ID definitions
 */
#define TID_7373_T01_01                 (1U)
#define TID_7373_T01_02                 (2U)
#define TID_7373_T01_03                 (3U)
#define TID_7373_T01_04                 (4U)
#define TID_7373_T01_05                 (5U)
#define TID_7373_T01_06                 (6U)
#define TID_7373_T01_07                 (7U)
#define TID_7373_T01_08                 (8U)
#define TID_7373_T01_09                 (9U)
#define TID_7373_T01_10                 (10U)
#define TID_7373_T01_11                 (11U)
#define TID_7373_T01_12                 (12U)
#define TID_7373_T01_13                 (13U)
#define TID_7373_T01_14                 (14U)
#define TID_7373_T01_15                 (15U)
#define TID_7373_T01_16                 (16U)
#define TID_7373_T01_17                 (17U)
#define TID_7373_T01_18                 (18U)
#define TID_7373_T01_19                 (19U)
#define TID_7373_T01_20                 (20U)
#define TID_7373_T01_21                 (21U)
#define TID_7373_T01_22                 (22U)
#define TID_7373_T01_23                 (23U)
#define TID_7373_T01_24                 (24U)
#define TID_7373_T01_25                 (25U)
#define TID_7373_T01_26                 (26U)
#define TID_7373_T01_27                 (27U)
#define TID_7373_T01_28                 (28U)
#define TID_7373_T01_29                 (29U)
#define TID_7373_T01_30                 (30U)
#define TID_7373_T01_31                 (31U)
#define TID_7373_T01_32                 (32U)
#define TID_7373_T01_33                 (33U)
#define TID_7373_T01_34                 (34U)
#define TID_7373_T01_35                 (35U)
#define TID_7373_T01_36                 (36U)
#define TID_7373_T01_37                 (37U)
#define TID_7373_T01_38                 (38U)
#define TID_7373_T01_39                 (39U)
#define TID_7373_T01_40                 (40U)
#define TID_7373_T01_41                 (41U)
#define TID_7373_T01_42                 (42U)
#define TID_7373_T01_43                 (43U)
#define TID_7373_T01_44                 (44U)
#define TID_7373_T01_45                 (45U)
#define TID_7373_T01_46                 (46U)
#define TID_7373_T01_47                 (47U)
#define TID_7373_T01_48                 (48U)
#define TID_7373_T01_49                 (49U)
#define TID_7373_T01_50                 (50U)
#define TID_7373_T01_51                 (51U)
#define TID_7373_T01_52                 (52U)
#define TID_7373_T01_53                 (53U)
#define TID_7373_T01_54                 (54U)
#define TID_7373_T01_55                 (55U)
#define TID_7373_T01_56                 (56U)
#define TID_7373_T01_57                 (57U)
#define TID_7373_T01_58                 (58U)
#define TID_7373_T01_59                 (59U)
#define TID_7373_T01_60                 (60U)
#define TID_7373_T01_61                 (61U)
