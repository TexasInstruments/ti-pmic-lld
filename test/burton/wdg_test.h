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
 * \file wdg_test.h
 * \author John Bui (j-bui1@ti.com)
 * \brief Header file for Burton watchdog Unity tests
 * \version 1.0
 */
#ifndef WDG_TEST_H
#define WDG_TEST_H

#include "common_test.h"

#ifdef __cplusplus
extern "C"
{
#endif

/**
 *  \brief  Pmic_wdgEnable/Pmic_wdgDisable: Test whether API can enable/disable Burton Watchdog
 */
void test_wdg_enableDisable(void);

/**
 *  \brief  Pmic_wdgSetCfg: Test whether API can configure Watchdog Long Window Duration
 */
void test_wdg_setCfg_longWindowDuration(void);

/**
 *  \brief  Pmic_wdgSetCfg: Test whether API can configure Watchdog Window-1 time-interval
 */
void test_wdg_setCfg_window1Duration(void);

/**
 *  \brief  Pmic_wdgSetCfg: Test whether API can configure Watchdog Window-2 time-interval
 */
void test_wdg_setCfg_window2Duration(void);

/**
 *  \brief  Pmic_wdgSetCfg: Test whether API can configure Watchdog Fail Treshold
 */
void test_wdg_setCfg_failThreshold(void);

/**
 *  \brief  Pmic_wdgSetCfg: Test whether API can configure Watchdog Reset Threshold
 */
void test_wdg_setCfg_resetThreshold(void);

/**
 *  \brief  Pmic_wdgSetCfg: Test whether API can configure Watchdog Reset Enable
 */
void test_wdg_setCfg_resetEnable(void);

/**
 *  \brief  Pmic_wdgSetCfg: Test whether API can configure Watchdog Mode
 */
void test_wdg_setCfg_wdgMode(void);

/**
 *  \brief  Pmic_wdgSetCfg: Test whether API can configure Watchdog Power Hold
 */
void test_wdg_setCfg_powerHold(void);

/**
 *  \brief  Pmic_wdgSetCfg: Test whether API can configure Watchdog Return Long Window
 */
void test_wdg_setCfg_ReturnLongWindow(void);

/**
 *  \brief  Pmic_wdgSetCfg: Test whether API can configure Watchdog Q&A Feedback
 */
void test_wdg_setCfg_QA_feedback(void);

/**
 *  \brief  Pmic_wdgSetCfg: Test whether API can configure Watchdog Q&A Linear-Feedback Shift Register
 */
void test_wdg_setCfg_QA_LFSR(void);

/**
 *  \brief  Pmic_wdgSetCfg: Test whether API can configure Watchdog Q&A Question Seed
 */
void test_wdg_setCfg_QA_questionSeed(void);

/**
 *  \brief  Pmic_wdgSetCfg: Test whether API can configure Watchdog Fail Counter
 */
void test_wdg_setCfg_cntSel(void);

/**
 *  \brief  Pmic_wdgSetCfg: Test whether API can configure Watchdog ENDRV_SEL
 */
void test_wdg_setCfg_enDrvSel(void);

/**
 *  \brief  Pmic_wdgBeginSequences: Test for no errors during correct Q&A mode operation.
 */
void test_wdg_QaMode_noErrors(void);

/**
 *  \brief  Pmic_wdgGetErrorStatus: Test whether WD_LONGWIN_TIMEOUT_INT error can be detected.
 *                                  Pmic_wdgClrErrStatus() API is also tested
 */
void test_wdg_QaMode_detect_longWindowTimeout(void);

/**
 *  \brief  Pmic_wdgGetErrorStatus: Test whether Window WD_TIMEOUT error can be detected.
 *                                  Pmic_wdgClrErrStatus() API is also tested
 */
void test_wdg_QaMode_detect_windowTimeout(void);

/**
 *  \brief  Pmic_wdgGetErrorStatus: Test whether WD_ANSW_EARLY error can be detected.
 *                                  Pmic_wdgClrErrStatus() API is also tested
 */
void test_wdg_QaMode_detect_answerEarly(void);

/**
 *  \brief  Pmic_wdgGetErrorStatus: Test whether WD_SEQ_ERR error can be detected.
 *                                  Pmic_wdgClrErrStatus() API is also tested
 */
void test_wdg_QaMode_detect_sequenceError(void);

/**
 *  \brief  Pmic_wdgGetErrorStatus: Test whether WD_ANSW_ERR error can be detected.
 *                                  Pmic_wdgClrErrStatus() API is also tested
 */
void test_wdg_QaMode_detect_answerError(void);

/**
 *  \brief  Pmic_wdgGetErrorStatus: Test whether WD_FAIL_INT error can be detected.
 *                                  Pmic_wdgClrErrStatus() and Pmic_wdgGetFailCntStat()
 *                                  APIs are also tested
 */
void test_wdg_QaMode_detect_failError(void);

/**
 *  \brief  Pmic_wdgGetErrorStatus: Test whether WD_RST_INT error can be detected.
 *                                  Pmic_wdgClrErrStatus() API is also tested
 */
void test_wdg_QaMode_detect_resetError(void);

#ifdef __cplusplus
}

#endif /* __cplusplus */

#endif /* WDG_TEST_H */
