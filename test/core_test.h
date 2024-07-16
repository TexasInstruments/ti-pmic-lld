/**
 * @file core_test.h
 *
 * @brief Header file containing prototypes of all Core module tests.
 */
#ifndef __CORE_TEST_H__
#define __CORE_TEST_H__

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

void coreTest(void *args);

/**
 * @brief coreTest0 - Test Pmic_setScratchPadVal() API error handling for when
 * NULL parameters are passed as input.
 */
void test_faultHandling_setScratchPadVal_nullParams(void);

/**
 * @brief coreTest1 - Test Pmic_setScratchPadVal() API error handling for when
 * an invalid scratchpad register is passed as input.
 */
void test_faultHandling_setScratchPadVal_invalidScratchPadReg(void);

/**
 * @brief coreTest2 - Test Pmic_getScratchPadVal() API error handling for when
 * NULL parameters are passed as input.
 */
void test_faultHandling_getScratchPadVal_nullParams(void);

/**
 * @brief coreTest3 - Test Pmic_getScratchPadVal() API error handling for when
 * an invalid scratchpad register is passed as input.
 */
void test_faultHandling_getScratchPadVal_invalidScratchPadReg(void);

/**
 * @brief coreTest4 - Test whether PMIC LLD can set and get values of
 * SCRATCH_PAD_REG_1, SCRATCH_PAD_REG_2, SCRATCH_PAD_REG_3, and SCRATCH_PAD_REG_4.
 */
void test_functionality_setGetScratchPadVal(void);

/**
 * @brief coreTest5 - Test Pmic_enableCRC8() API error handling for when NULL
 * parameters are passed as input.
 */
void test_faultHandling_enableCRC8_nullParams(void);

/**
 * @brief coreTest6 - Test Pmic_disableCRC8() API error handling for when NULL
 * parameters are passed as input.
 */
void test_faultHandling_disableCRC8_nullParams(void);

/**
 * @brief coreTest7 - Test Pmic_getCRC8Enable() API error handling for when
 * NULL parameters are passed as input.
 */
void test_faultHandling_getCRC8Enable_nullParams(void);

/**
 * @brief coreTest8 - Test whether PMIC LLD can enable and disable CRC8.
 */
void test_functionality_enableDisableCRC8(void);

/**
 * @brief coreTest9 - Test Pmic_unlockRegs() API error handling for when NULL
 * parameters are passed as input.
 */
void test_faultHandling_unlockRegs_nullParams(void);

/**
 * @brief coreTest10 - Test Pmic_lockRegs() API error handling for when NULL
 * parameters are passed as input.
 */
void test_faultHandling_lockRegs_nullParams(void);

/**
 * @brief coreTest11 - Test Pmic_getRegLock() API error handling for when NULL
 * parameters are passed as input.
 */
void test_faultHandling_getRegLock_nullParams(void);

/**
 * @brief coreTest12 - Test whether PMIC LLD can lock and unlock configuration
 * registers.
 */
void test_functionality_LockUnlockRegs(void);

/**
 * @brief coreTest13 - Test Pmic_setPwrOn() API error handling for when NULL
 * parameters are passed as input.
 */
void test_faultHandling_setPwrOn_nullParams(void);

/**
 * @brief coreTest14 - Test Pmic_getPwrOn() API error handling for when NULL
 * parameters are passed as input.
 */
void test_faultHandling_getPwrOn_nullParams(void);

/**
 * @brief coreTest15 - Test whether PMIC LLD can set the PWR_ON bit field.
 */
void test_functionality_setPwrOn(void);

/**
 * @brief coreTest16 - Test Pmic_sendFsmCmd() API error handling for when NULL
 * parameters are passed as input.
 */
void test_faultHandling_sendFsmCmd_nullParams(void);

/**
 * @brief coreTest17 - Test Pmic_sendFsmCmd() API error handling for when an
 * invalid FSM command is passed as input.
 */
void test_faultHandling_sendFsmCmd_invalidFsmCmd(void);

/**
 * @brief coreTest18 - Test whether PMIC LLD can send FSM commands to enter LPM
 * and exit LPM.
 */
void test_functionality_sendFsmCmd_lpmEntryExit(void);

/**
 * @brief coreTest19 - Test Pmic_runABIST() API error handling for when NULL
 * parameters are passed as input.
 */
void test_faultHandling_runABIST_nullParams(void);

/**
 * @brief coreTest20 - Test whether PMIC LLD can initiate ABIST.
 */
void test_functionality_runABIST(void);

/**
 * @brief coreTest21 - Test Pmic_setLpmCfg() API error handling for when NULL
 * parameters are passed as input.
 */
void test_faultHandling_setLpmCfg_nullParams(void);

/**
 * @brief coreTest22 - Test Pmic_setLpmCfg() API error handling for when there
 * are no valid parameters specified.
 */
void test_faultHandling_setLpmCfg_noValidParams(void);

/**
 * @brief coreTest23 - Test Pmic_getLpmCfg() API error handling for when NULL
 * parameters are passed as input.
 */
void test_faultHandling_getLpmCfg_nullParams(void);

/**
 * @brief coreTest24 - Test Pmic_getLpmCfg() API error handling for when there
 * are no valid parameters specified.
 */
void test_faultHandling_getLpmCfg_noValidParams(void);

/**
 * @brief coreTest25 - Test whether PMIC LLD can set and get the LPM pin
 * detection configuration.
 */
void test_functionality_SetGetLpmCfg_pinDetection(void);

/**
 * @brief coreTest26 - Test whether PMIC LLD can set and get the LPM pin
 * detection delay.
 */
void test_functionality_SetGetLpmCfg_detectionDelay(void);

/**
 * @brief coreTest27 - Test whether PMIC LLD can set and get the LPM VMON enable.
 */
void test_functionality_SetGetLpmCfg_vmonEn(void);

/**
 * @brief coreTest28 - Test whether PMIC LLD can set and get the LPM ESM enable.
 */
void test_functionality_SetGetLpmCfg_esmEn(void);

/**
 * @brief coreTest29 - Test whether PMIC LLD can set and get the LPM watchdog
 * enable.
 */
void test_functionality_SetGetLpmCfg_wdEn(void);

#ifdef __cplusplus
}

#endif /* __cplusplus */
#endif /* __CORE_TEST_H__ */
