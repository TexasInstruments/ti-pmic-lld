/**
 * @file esm_test.h
 *
 * @brief Header file containing prototypes of all ESM module tests.
 */
#ifndef __ESM_TEST_H__
#define __ESM_TEST_H__

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

void esmTest(void *args);

/**
 * @brief esmTest0: Test Pmic_esmStartStop() API error handling for when NULL
 * parameters are passed as input.
 */
void test_faultHandling_esmStartStop_nullParams(void);

/**
 * @brief esmTest1: Test Pmic_esmGetStartStop() API error handling for when NULL
 * parameters are passed as input.
 */
void test_faultHandling_esmGetStartStop_nullParams(void);

/**
 * @brief esmTest2: Test whether PMIC LLD can start and stop the ESM.
 */
void test_functionality_esmStartStop(void);

/**
 * @brief esmTest3: Test Pmic_esmSetCfg() API error handling for when NULL
 * parameters are passed as input.
 */
void test_faultHandling_esmSetCfg_nullParams(void);

/**
 * @brief esmTest4: Test Pmic_esmSetCfg() API error handling for when no valid
 * parameters are specified.
 */
void test_faultHandling_esmSetCfg_noValidParams(void);

/**
 * @brief esmTest5: Test Pmic_esmGetCfg() API error handling for when NULL
 * parameters are passed as input.
 */
void test_faultHandling_esmGetCfg_nullParams(void);

/**
 * @brief esmTest6: Test Pmic_esmGetCfg() API error handling for when no valid
 * parameters are specified.
 */
void test_faultHandling_esmGetCfg_noValidParams(void);

/**
 * @brief esmTest7: Test whether PMIC LLD can set and get ESM enable
 * configuration.
 */
void test_functionality_esmSetGetCfg_enable(void);

/**
 * @brief esmTest8: Test whether PMIC LLD can set and get ESM mode
 * configuration.
 */
void test_functionality_esmSetGetCfg_mode(void);

/**
 * @brief esmTest9: Test whether PMIC LLD can set and get ESM error count
 * threshold configuration.
 */
void test_functionality_esmSetGetCfg_errCntThr(void);

/**
 * @brief esmTest10: Test whether PMIC LLD can set and get ESM DELAY1
 * configuration.
 */
void test_functionality_esmSetGetCfg_delay1(void);

/**
 * @brief esmTest11: Test whether PMIC LLD can set and get ESM DELAY2
 * configuration.
 */
void test_functionality_esmSetGetCfg_delay2(void);

/**
 * @brief esmTest12: Test whether PMIC LLD can set and get ESM HMAX
 * configuration.
 */
void test_functionality_esmSetGetCfg_hmax(void);

/**
 * @brief esmTest13: Test whether PMIC LLD can set and get ESM HMIN
 * configuration.
 */
void test_functionality_esmSetGetCfg_hmin(void);

/**
 * @brief esmTest14: Test whether PMIC LLD can set and get ESM LMAX
 * configuration.
 */
void test_functionality_esmSetGetCfg_lmax(void);

/**
 * @brief esmTest15: Test whether PMIC LLD can set and get ESM LMIN
 * configuration.
 */
void test_functionality_esmSetGetCfg_lmin(void);

/**
 * @brief esmTest16: Test Pmic_esmGetStat() API error handling for when NULL
 * parameters are passed as input.
 */
void test_faulthandling_esmGetStat_nullParams(void);

/**
 * @brief esmTest17: Test Pmic_esmGetStat() API error handling for when no
 * valid parameters are specified.
 */
void test_faulthandling_esmGetStat_noValidParams(void);

/**
 * @brief esmTest18: Test Pmic_esmClrStat() API error handling for when NULL
 * parameters are passed as input.
 */
void test_faulthandling_esmClrStat_nullParams(void);

/**
 * @brief esmTest19: Test Pmic_esmClrStat() API error handling for when no valid
 * parameters are specified.
 */
void test_faulthandling_esmClrStat_noValidParams(void);

/**
 * @brief esmTest20: Test Pmic_esmGetErrCnt() API error handling for when NULL
 * parameters are passed as input.
 */
void test_faulthandling_esmGetErrCnt_nullParams(void);

/**
 * @brief esmTest21: Test whether PMIC LLD can facilitate ESM operation in Level
 * mode.
 */
void test_functionality_esm_levelMode(void);

/**
 * @brief esmTest22: Test whether PMIC LLD can facilitate ESM operation in PWM
 * mode.
 */
void test_functionality_esm_pwmMode(void);

/**
 * @brief esmTest23: Test whether PMIC LLD can detect ESM_MCU_RST_INT error.
 */
void test_errorDetection_esm_rstInt(void);

/**
 * @brief esmTest24: Test whether PMIC LLD can detect ESM_MCU_FAIL_INT error.
 */
void test_errorDetection_esm_failInt(void);

/**
 * @brief esmTest25: Test whether PMIC LLD can detect ESM_MCU_PIN_INT error.
 */
void test_errorDetection_esm_pinInt(void);

#ifdef __cplusplus
}

#endif /* __cplusplus */
#endif /* __ESM_TEST_H__ */
