/**
 * \file wdg_test.h
 * \author John Bui (j-bui1@ti.com)
 * \brief Header file for Burton watchdog Unity tests
 * \version 1.0
 * \date 2023-12-12
 *
 * \copyright Copyright (c) 2023
 */
#ifndef wdg_TEST_H
#define wdg_TEST_H

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
 *  \brief  Pmic_wdgSetCfg:Test whether API can configure Watchdog Window-2 time-interval
 */
void test_wdg_setCfg_window2Duration(void);

/**
 *  \brief  Pmic_wdgSetCfg:Test whether API can configure Watchdog Fail Treshold
 */
void test_wdg_setCfg_failThreshold(void);

/**
 *  \brief  Pmic_wdgSetCfg:Test whether API can configure Watchdog Reset Threshold
 */
void test_wdg_setCfg_resetThreshold(void);

/**
 *  \brief  Pmic_wdgSetCfg:Test whether API can configure Watchdog Reset Enable
 */
void test_wdg_setCfg_resetEnable(void);

/**
 *  \brief  Pmic_wdgSetCfg:Test whether API can configure Watchdog Mode
 */
void test_wdg_setCfg_wdgMode(void);

/**
 *  \brief  Pmic_wdgSetCfg:Test whether API can configure Watchdog Power Hold
 */
void test_wdg_setCfg_powerHold(void);

/**
 *  \brief  Pmic_wdgSetCfg:Test whether API can configure Watchdog Return Long Window
 */
void test_wdg_setCfg_ReturnLongWindow(void);

/**
 *  \brief  Pmic_wdgSetCfg:Test whether API can configure Watchdog Q&A Feedback
 */
void test_wdg_setCfg_QA_feedback(void);

/**
 *  \brief  Pmic_wdgSetCfg:Test whether API can configure Watchdog Q&A Linear-Feedback Shift Register
 */
void test_wdg_setCfg_QA_LFSR(void);

/**
 *  \brief  Pmic_wdgSetCfg:Test whether API can configure Watchdog Q&A Question Seed
 */
void test_wdg_setCfg_QA_questionSeed(void);

#ifdef __cplusplus
}

#endif /* __cplusplus */

#endif /* WDG_TEST_H */
