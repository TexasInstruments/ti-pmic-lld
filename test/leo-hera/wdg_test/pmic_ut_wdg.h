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
 *  \file   pmic_ut_wdg.h
 *
 *  \brief  Header file for PMIC WDG Unit Tests
 *
 */

#include <pmic_ut_common.h>

/*!
 * \brief  PMIC WDG Valid Params configuration for all
 */
#define PMIC_WDG_CFG_SETPARAMS_FORALL \
        (PMIC_CFG_WDG_LONGWINDURATION_VALID_SHIFT | \
         PMIC_CFG_WDG_WIN1DURATION_VALID_SHIFT    | \
         PMIC_CFG_WDG_WIN2DURATION_VALID_SHIFT    | \
         PMIC_CFG_WDG_FAILTHRESHOLD_VALID_SHIFT   | \
         PMIC_CFG_WDG_RSTTHRESHOLD_VALID_SHIFT    | \
         PMIC_CFG_WDG_RSTENABLE_VALID_SHIFT       | \
         PMIC_CFG_WDG_WDGMODE_VALID_SHIFT         | \
         PMIC_CFG_WDG_PWRHOLD_VALID_SHIFT         | \
         PMIC_CFG_WDG_RETLONGWIN_VALID_SHIFT      | \
         PMIC_CFG_WDG_QA_FDBK_VALID_SHIFT         | \
         PMIC_CFG_WDG_QA_LFSR_VALID_SHIFT         | \
         PMIC_CFG_WDG_QA_QUES_SEED_VALID_SHIFT)

/*!
 * \brief   Numbers of WDG testcases
 */
#define PMIC_WDG_NUM_OF_TESTCASES    \
        (sizeof(pmic_wdg_tests)/sizeof(pmic_wdg_tests[0]))

/*!
 * \brief  PMIC WDG Valid Params configuration for all error status
 */
#define PMIC_CFG_WD_ALL_ERRSTAT_VALID_PARAMS \
        (PMIC_CFG_WD_LONGWIN_TIMEOUT_ERRSTAT_VALID_SHIFT | \
         PMIC_CFG_WD_TIMEOUT_ERRSTAT_VALID_SHIFT         | \
         PMIC_CFG_WD_TRIG_EARLY_ERRSTAT_VALID_SHIFT      | \
         PMIC_CFG_WD_ANSW_EARLY_ERRSTAT_VALID_SHIFT      | \
         PMIC_CFG_WD_SEQ_ERR_ERRSTAT_VALID_SHIFT         | \
         PMIC_CFG_WD_ANSW_ERR_ERRSTAT_VALID_SHIFT        | \
         PMIC_CFG_WD_FAIL_INT_ERRSTAT_VALID_SHIFT        | \
         PMIC_CFG_WD_RST_INT_ERRSTAT_VALID_SHIFT)

/*!
 * \brief  PMIC WDG Valid Params configuration for all Fail Count status
 */
#define PMIC_CFG_WD_ALL_FAILCNTSTAT_VALID_PARAMS \
        (PMIC_CFG_WD_BAD_EVENT_STAT_VALID_SHIFT     | \
         PMIC_CFG_WD_GOOD_EVENT_STAT_VALID_SHIFT    | \
         PMIC_CFG_WD_FAIL_CNT_VAL_VALID_SHIFT)
