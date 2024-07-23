/**
 * @file wdg_test.h
 *
 * @brief Header file containing prototypes of all Watchdog module tests.
 */
#ifndef __WDG_TEST_H__
#define __WDG_TEST_H__

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                Include Files                               */
/* ========================================================================== */
#include "common_test.h"

/* ========================================================================== */
/*                              Test Declarations                             */
/* ========================================================================== */

void wdgTest(void *args);

/**
 * @brief wdgTest0 - Test Pmic_wdgGetEnableStat() API error handling for when
 *                   NULL parameters are passed as input.
 */
void test_faultHandling_wdgGetEnableStat_nullParams(void);

/**
 * @brief wdgTest1 - Test Pmic_wdgDisable() API error handling for when NULL
 *                   parameters are passed as input.
 */
void test_faultHandling_wdgDisable_nullParams(void);

/**
 * @brief wdgTest2 - Test Pmic_wdgEnable() API error handling for when NULL
 *                   parameters are passed as input.
 */
void test_faultHandling_wdgEnable_nullParams(void);

/**
 * @brief wdgTest3 - Test whether PMIC LLD can enable and disable PMIC watchdog
 *                   in addition to getting the watchdog enable/disable status.
 */
void test_functionality_wdgEnableDisable(void);

/**
 * @brief wdgTest4 - Test Pmic_wdgGetPwrHoldStat() API error handling for when
 *                   NULL parameters are passed as input.
 */
void test_faultHandling_wdgGetPwrHold_nullParams(void);

/**
 * @brief wdgTest5 - Test Pmic_wdgSetPwrHold() API error handling for when NULL
 *                   parameters are passed as input.
 */
void test_faultHandling_wdgSetPwrHold_nullParams(void);

/**
 * @brief wdgTest6 - Test whether PMIC LLD can set and clear WD_PWR_HOLD in addition
 *                   to getting the WD_PWR_HOLD status.
 */
void test_functionality_wdgSetClrPwrHold(void);

/**
 * @brief wdgTest7 - Test Pmic_wdgGetRetLongWinStat() API error handling for when
 *                   NULL parameters are passed as input.
 */
void test_faultHandling_wdgGetRetLongWin_nullParams(void);

/**
 * @brief wdgTest8 - Test Pmic_wdgSetRetLongWin() API error handling for when NULL
 *                   parameters are passed as input.
 */
void test_faultHandling_wdgSetRetLongWin_nullParams(void);

/**
 * @brief wdgTest9 - Test whether PMIC LLD can set and clear WD_RETURN_LONGWIN in
 *                   addition to getting the WD_RETURN_LONGWIN status.
 */
void test_functionality_wdgSetClrRetLongWin(void);

/**
 * @brief wdgTest10 - Test Pmic_wdgGetCfg() API error handling for when NULL parameters
 *                    are passed as input.
 */
void test_faultHandling_wdgGetCfg_nullParams(void);

/**
 * @brief wdgTest11 - Test Pmic_wdgGetCfg() API error handling for when no valid parameters
 *                    are specified.
 */
void test_faultHandling_wdgGetCfg_noValidParams(void);

/**
 * @brief wdgTest12 - Test Pmic_wdgSetCfg() API error handling for when NULL parameters
 *                    are passed as input.
 */
void test_faultHandling_wdgSetCfg_nullParams(void);

/**
 * @brief wdgTest13 - Test Pmic_wdgSetCfg() API error handling for when no valid parameters
 *                    are specified.
 */
void test_faultHandling_wdgSetCfg_noValidParams(void);

/**
 * @brief wdgTest14 - Test whether PMIC LLD can set and get watchdog Reset Enable
 *                    configuration.
 */
void test_functionality_wdgSetGetCfg_rstEn(void);

/**
 * @brief wdgTest15 - Test whether PMIC LLD can set and get watchdog mode of operation.
 */
void test_functionality_wdgSetGetCfg_mode(void);

/**
 * @brief wdgTest16 - Test whether PMIC LLD can set and get watchdog Trigger Selection.
 */
void test_functionality_wdgSetGetCfg_trigSel(void);

/**
 * @brief wdgTest17 - Test whether PMIC LLD can set and get watchdog Fail Threshold.
 */
void test_functionality_wdgSetGetCfg_failThr(void);

/**
 * @brief wdgTest18 - Test whether PMIC LLD can set and get watchdog Reset Threshold.
 */
void test_functionality_wdgSetGetCfg_rstThr(void);

/**
 * @brief wdgTest19 - Test whether PMIC LLD can set and get watchdog Long Window duration.
 */
void test_functionality_wdgSetGetCfg_longWinDuration(void);

/**
 * @brief wdgTest20 - Test whether PMIC LLD can set and get watchdog Window-1 duration.
 */
void test_functionality_wdgSetGetCfg_win1Duration(void);

/**
 * @brief wdgTest21 - Test whether PMIC LLD can set and get watchdog Window-2 duration.
 */
void test_functionality_wdgSetGetCfg_win2Duration(void);

/**
 * @brief wdgTest22 - Test whether PMIC LLD can set and get watchdog Q&A Feedback.
 */
void test_functionality_wdgSetGetCfg_qaFdbk(void);

/**
 * @brief wdgTest23 - Test whether PMIC LLD can set and get watchdog Q&A LFSR.
 */
void test_functionality_wdgSetGetCfg_qaLfsr(void);

/**
 * @brief wdgTest24 - Test whether PMIC LLD can set and get watchdog Q&A Question Seed.
 */
void test_functionality_wdgSetGetCfg_qaSeed(void);

/**
 * @brief wdgTest25 - Test Pmic_wdgGetErrStat() API error handling for when NULL
 *                    parameters are passed as input.
 */
void test_faultHandling_wdgGetErrStat_nullParams(void);

/**
 * @brief wdgTest26 - Test Pmic_wdgClrErrStat() API error handling for when NULL
 *                    parameters are passed as input.
 */
void test_faultHandling_wdgClrErrStat_nullParams(void);

/**
 * @brief wdgTest27 - Test Pmic_wdgGetFailCntStat() API error handling for when NULL
 *                    parameters are passed as input.
 */
void test_faultHandling_wdgGetFailCntStat_nullParams(void);

/**
 * @brief wdgTest28 - Test whether PMIC LLD can facilitate Watchdog operation in the
 *                    Trigger mode (SW trigger as the trigger source) with no errors.
 */
void test_functionality_wdgSwTriggerMode_noErrors(void);

/**
 * @brief wdgTest29 - Test whether PMIC LLD can facilitate Watchdog operation in the
 *                    Q&A mode with no errors.
 */
void test_functionality_wdgQaMode_noErrors(void);

/**
 * @brief wdgTest30 - Test whether PMIC LLD can detect WD_LONGWIN_TIMEOUT_INT error.
 */
void test_errorDetection_wdg_longWinTimeoutInt(void);

/**
 * @brief wdgTest31 - Test whether PMIC LLD can detect WD_TIMEOUT error.
 */
void test_errorDetection_wdg_timeout(void);

/**
 * @brief wdgTest32 - Test whether PMIC LLD can detect WD_FAIL_INT error.
 */
void test_errorDetection_wdg_failInt(void);

/**
 * @brief wdgTest33 - Test whether PMIC LLD can detect WD_RST_INT error.
 */
void test_errorDetection_wdg_RstInt(void);

#ifdef __cplusplus
}

#endif /* __cplusplus */
#endif /* __WDG_TEST_H__ */
