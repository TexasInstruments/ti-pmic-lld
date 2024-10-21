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

#include "regmap/core.h"
#include "regmap/irq.h"

/* ========================================================================== */
/*                              Test Declarations                             */
/* ========================================================================== */

void coreTest(void *args);

/**
 * @brief core_test0: Test Pmic_setRegLock() API error handling for when NULL
 * parameters are passed as input.
 */
void test_faultHandling_setRegLock_nullParams(void);

/**
 * @brief core_test1: Test Pmic_getRegLock() API error handling for when NULL
 * parameters are passed as input.
 */
void test_faultHandling_getRegLock_nullParams(void);

/**
 * @brief core_test2: Test Pmic_unlockRegs() API error handling for when NULL
 * parameters are passed as input.
 */
void test_faultHandling_unlockRegs_nullParams(void);

/**
 * @brief core_test3: Test Pmic_lockRegs() API error handling for when NULL
 * parameters are passed as input.
 */
void test_faultHandling_lockRegs_nullParams(void);

/**
 * @brief core_test4: Test whether PMIC LLD can set and get PMIC register lock.
 */
void test_functionality_setGetRegLock(void);

/**
 * @brief core_test5: Test Pmic_writeCfgCrc() API error handling for when
 * NULL parameters are passed as input.
 */
void test_faultHandling_writeCfgCrc_nullParams(void);

/**
 * @brief core_test6: Test Pmic_setCfgCrc() API error handling for when NULL
 * parameters are passed as input.
 */
void test_faultHandling_setCfgCrc_nullParams(void);

/**
 * @brief core_test7: Test Pmic_enableCfgCrc() API error handling for when
 * NULL parameters are passed as input.
 */
void test_faultHandling_enableCfgCrc_nullParams(void);

/**
 * @brief core_test8: Test Pmic_disableCfgCrc() API error handling for when
 * NULL parameters are passed as input.
 */
void test_faultHandling_disableCfgCrc_nullParams(void);

/**
 * @brief core_test9: Test Pmic_getCfgCrcErrStat() API error handling for
 * when NULL parameters are passed as input.
 */
void test_faultHandling_getCfgCrcErrStat_nullParams(void);

/**
 * @brief core_test10: Test whether PMIC LLD can write the correct configuration
 * register CRC via Pmic_writeCfgCrc() and enable the CRC check via
 * Pmic_setCfgCrc().
 */
void test_functionality_writeCfgCrc_setCfgCrc(void);

/**
 * @brief core_test11: Test Pmic_getDevState() API error handling for when NULL
 * parameters are passed as input.
 */
void test_faultHandling_getDevState_nullParams(void);

/**
 * @brief core_test12: Test whether PMIC LLD can get the current device state.
 */
void test_functionality_getDevState(void);

/**
 * @brief core_test13: Test Pmic_sendRstReq() API error handling for when NULL
 * parameters are passed as input.
 */
void test_faultHandling_sendRstReq_nullParams(void);

/**
 * @brief core_test15: Test Pmic_sendSafeExitReq() API error handling for when
 * NULL parameters are passed as input.
 */
void test_faultHandling_sendSafeExitReq_nullParams(void);

/**
 * @brief core_test16: Test Pmic_getBistStat() API error handling for when NULL
 * parameters are passed as input.
 */
void test_faultHandling_getBistStat_nullParams(void);

/**
 * @brief core_test17: Test Pmic_startLBIST() API error handling for when NULL
 * parameters are passed as input.
 */
void test_faultHandling_startLBIST_nullParams(void);

/**
 * @brief core_test18: Test Pmic_startABIST() API error handling for when NULL
 * parameters are passed as input.
 */
void test_faultHandling_startABIST_nullParams(void);

/**
 * @brief core_test19: Test Pmic_setDeviceCfg() API error handling for when NULL
 * parameters are passed as input.
 */
void test_faultHandling_setDeviceCfg_nullParams(void);

/**
 * @brief core_test20: Test Pmic_getDeviceCfg() API error handling for when NULL
 * parameters are passed as input.
 */
void test_faultHandling_getDeviceCfg_nullParams(void);

/**
 * @brief core_test21: Test whether PMIC LLD can set and get the configuration
 * of PWD_THR[3:0] bit field.
 */
void test_functionality_setGetDeviceCfg_pwrDwnThr(void);

/**
 * @brief core_test22: Test whether PMIC LLD can set and get the configuration
 * of SAFE_LOCK_THR[3:0] bit field.
 */
void test_functionality_setGetDeviceCfg_safeLockThr(void);

/**
 * @brief core_test23: Test whether PMIC LLD can set and get the configuration
 * of DEV_ERR_CNT[3:0] bit field.
 */
void test_functionality_setGetDeviceCfg_devErrCnt(void);

/**
 * @brief core_test24: Test whether PMIC LLD can set and get the configuration
 * of SAFE_TO[2:0] bit field.
 */
void test_functionality_setGetDeviceCfg_safeTimeoutDuration();

/**
 * @brief core_test25: Test whether PMIC LLD can set and get the configuration
 * of SAFE_LOCK_TO_DIS bit field.
 */
void test_functionality_setGetDeviceCfg_disableSafeLockTimeout(void);

/**
 * @brief core_test26: Test whether PMIC LLD can set and get the configuration
 * of AUTO_BIST_DIS bit field.
 */
void test_functionality_setGetDeviceCfg_disableAutoBIST(void);

/**
 * @brief core_test27: Test whether PMIC LLD can set and get the configuration
 * of ENABLE_DRV bit field.
 */
void test_functionality_setGetDeviceCfg_enableDrv(void);

/**
 * @brief core_test28: Test whether PMIC LLD can set and get the configuration
 * of DIAG_EXIT_MASK bit field.
 */
void test_functionality_setGetDeviceCfg_diagExitMask(void);

/**
 * @brief core_test29: Test whether PMIC LLD can set and get the configuration
 * of DIAG_EXIT bit field.
 */
void test_functionality_setGetDeviceCfg_diagExit(void);

#ifdef __cplusplus
}

#endif /* __cplusplus */
#endif /* __CORE_TEST_H__ */
