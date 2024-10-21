/**
 * @file irq_test.h
 *
 * @brief Header file containing prototypes of all IRQ module tests.
 */
#ifndef __IRQ_TEST_H__
#define __IRQ_TEST_H__

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

void irqTest(void *args);

/**
 * @brief irqTest0: Test Pmic_irqGetStat API error handling for when null
 * parameters are passed as input.
 */
void test_faultHandling_irqGetStat_nullParams(void);

/**
 * @brief irqTest1: Test Pmic_irqGetNextFlag API error handling for when null
 * parameters are passed as input.
 */
void test_faultHandling_irqGetNextFlag_nullParams(void);

/**
 * @brief irqTest2: Test Pmic_irqGetFlag API error handling for when null
 * parameters are passed as input.
 */
void test_faultHandling_irqGetFlag_nullParams(void);

/**
 * @brief irqTest3: Test Pmic_irqGetFlag API error handling for when an invalid
 * IRQ number is passed as input.
 */
void test_faultHandling_irqGetFlag_invalidIrqNum(void);

/**
 * @brief irqTest4: Test whether LLD can get the status of all IRQs and be able
 * to indicate whether IRQ statuses are pending.
 */
void test_functionality_irqGetStat_irqGetNextFlag(void);

#ifdef __cplusplus
}

#endif /* __cplusplus */
#endif /* __IRQ_TEST_H__ */
