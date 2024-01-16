/**
 * \file wdg_test.h
 * \author John Bui (j-bui1@ti.com)
 * \brief Header file for Burton watchdog Unity tests
 * \version 1.0
 * \date 2023-12-12
 *
 * \copyright Copyright (c) 2023
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
