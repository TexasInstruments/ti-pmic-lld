/**
 * @file misc_test.h
 *
 * @brief Header file containing prototypes of all miscellaneous module tests.
 */
#ifndef __MISC_TEST_H__
#define __MISC_TEST_H__

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

void miscTest(void *args);

/**
 * @brief miscTest0: Test Pmic_init() API error handling for when NULL parameters
 * are passed as input.
 */
void test_faultHandling_pmicInit_nullParams(void);

/**
 * @brief miscTest1: Test Pmic_init() API error handling for when invalid
 * configurations are passed as input.
 */
void test_faultHandling_pmicInit_invalidCfg(void);

/**
 * @brief miscTest2: Test whether LLD can initialize the PMIC handle.
 */
void test_functionality_pmicInit(void);

/**
 * @brief miscTest3: Test Pmic_deinit() API error handling for when NULL parameters
 * are passed as input.
 */
void test_faultHandling_pmicDeinit_nullParams(void);

/**
 * @brief miscTest4: Test whether LLD can deinitialize the PMIC handle.
 */
void test_functionality_pmicDeinit(void);

#ifdef __cplusplus
}

#endif /* __cplusplus */
#endif /* __MISC_TEST_H__ */
