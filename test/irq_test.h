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
 * @brief irqTest0 - Test Pmic_irqSetMasks() API error handling for when an
 * invalid IRQ is passed as input.
 */
void test_faultHandling_irqSetMasks_invalidIrqNum(void);

/**
 * @brief irqTest1 - Test Pmic_irqSetMasks() API error handling for when an
 * invalid number of IRQ masks is passed as input.
 */
void test_faultHandling_irqSetMasks_invalidNumIrqMasks(void);

/**
 * @brief irqTest2 - Test Pmic_irqSetMasks() API error handling for when NULL
 * parameters are passed as input.
 */
void test_faultHandling_irqSetMasks_nullParams(void);

/**
 * @brief irqTest3 - Test Pmic_irqGetMasks() API error handling for when an
 * invalid IRQ is passed as input.
 */
void test_faultHandling_irqGetMasks_invalidIrqNum(void);

/**
 * @brief irqTest4 - Test Pmic_irqGetMasks() API error handling for when an
 * invalid number of IRQ masks is passed as input.
 */
void test_faultHandling_irqGetMasks_invalidNumIrqMasks(void);

/**
 * @brief irqTest5 - Test Pmic_irqGetMasks() API error handling for when NULL
 * parameters are passed as input.
 */
void test_faultHandling_irqGetMasks_nullParams(void);

/**
 * @brief irqTest6 - Test PMIC LLD error handling of setting and getting the
 * mask configuration of NMI LDO_SC_INT.
 */
void test_faultHandling_irqSetGetMask_LDO_SC_INT(void);

/**
 * @brief irqTest7 - Test PMIC LLD error handling of setting and getting the
 * mask configuration of NMI BUCK3_SC_INT.
 */
void test_faultHandling_irqSetGetMask_BUCK3_SC_INT(void);

/**
 * @brief irqTest8 - Test PMIC LLD error handling of setting and getting the
 * mask configuration of NMI BUCK2_SC_INT.
 */
void test_faultHandling_irqSetGetMask_BUCK2_SC_INT(void);

/**
 * @brief irqTest9 - Test PMIC LLD error handling of setting and getting the
 * mask configuration of NMI BUCK1_SC_INT.
 */
void test_faultHandling_irqSetGetMask_BUCK1_SC_INT(void);

/**
 * @brief irqTest10 - Test whether PMIC LLD can set and get the mask configuration
 * of IRQ BUCK2_OVP_INT.
 */
void test_functionality_irqSetGetMask_BUCK2_OVP_INT(void);

/**
 * @brief irqTest11 - Test whether PMIC LLD can set and get the mask configuration
 * of IRQ BUCK2_UV_INT.
 */
void test_functionality_irqSetGetMask_BUCK2_UV_INT(void);

/**
 * @brief irqTest12 - Test whether PMIC LLD can set and get the mask configuration
 * of IRQ BUCK2_OV_INT.
 */
void test_functionality_irqSetGetMask_BUCK2_OV_INT(void);

/**
 * @brief irqTest13 - Test whether PMIC LLD can set and get the mask configuration
 * of IRQ BUCK1_OVP_INT.
 */
void test_functionality_irqSetGetMask_BUCK1_OVP_INT(void);

/**
 * @brief irqTest14 - Test whether PMIC LLD can set and get the mask configuration
 * of IRQ BUCK1_UV_INT.
 */
void test_functionality_irqSetGetMask_BUCK1_UV_INT(void);

/**
 * @brief irqTest15 - Test whether PMIC LLD can set and get the mask configuration
 * of IRQ BUCK1_OV_INT.
 */
void test_functionality_irqSetGetMask_BUCK1_OV_INT(void);

/**
 * @brief irqTest16 - Test whether PMIC LLD can set and get the mask configuration
 * of IRQ LDO_OVP_INT.
 */
void test_functionality_irqSetGetMask_LDO_OVP_INT(void);

/**
 * @brief irqTest17 - Test whether PMIC LLD can set and get the mask configuration
 * of IRQ LDO_UV_INT.
 */
void test_functionality_irqSetGetMask_LDO_UV_INT(void);

/**
 * @brief irqTest18 - Test whether PMIC LLD can set and get the mask configuration
 * of IRQ LDO_OV_INT.
 */
void test_functionality_irqSetGetMask_LDO_OV_INT(void);

/**
 * @brief irqTest19 - Test whether PMIC LLD can set and get the mask configuration
 * of IRQ BUCK3_OVP_INT.
 */
void test_functionality_irqSetGetMask_BUCK3_OVP_INT(void);

/**
 * @brief irqTest20 - Test whether PMIC LLD can set and get the mask configuration
 * of IRQ BUCK3_OVP_INT.
 */
void test_functionality_irqSetGetMask_BUCK3_UV_INT(void);

/**
 * @brief irqTest21 - Test whether PMIC LLD can set and get the mask configuration
 * of IRQ BUCK3_OVP_INT.
 */
void test_functionality_irqSetGetMask_BUCK3_OV_INT(void);

/**
 * @brief irqTest22 - Test whether PMIC LLD can set and get the mask configuration
 * of IRQ TWARN_INT.
 */
void test_functionality_irqSetGetMask_TWARN_INT(void);

/**
 * @brief irqTest23 - Test whether PMIC LLD can set and get the mask configuration
 * of IRQ B1_PVIN_UVLO_INT.
 */
void test_functionality_irqSetGetMask_B1_PVIN_UVLO_INT(void);

/**
 * @brief irqTest24 - Test whether PMIC LLD can set and get the mask configuration
 * of IRQ BUCKS_VSET_ERR_INT.
 */
void test_functionality_irqSetGetMask_BUCKS_VSET_ERR_INT(void);

/**
 * @brief irqTest25 - Test PMIC LLD error handling of setting and getting the
 * mask configuration of NMI CFG_NVM_VERIFY_ERR.
 */
void test_faultHandling_irqSetGetMask_CFG_NVM_VERIFY_ERR(void);

/**
 * @brief irqTest26 - Test PMIC LLD error handling of setting and getting the
 * mask configuration of NMI CFG_NVM_VERIFY_DONE.
 */
void test_faultHandling_irqSetGetMask_CFG_NVM_VERIFY_DONE(void);

/**
 * @brief irqTest27 - Test PMIC LLD error handling of setting and getting the
 * mask configuration of NMI CFG_NVM_PRG_DONE.
 */
void test_faultHandling_irqSetGetMask_CFG_NVM_PRG_DONE(void);

/**
 * @brief irqTest28 - Test whether PMIC LLD can set and get the mask configuration
 * of IRQ ABIST_FAIL_INT.
 */
void test_functionality_irqSetGetMask_ABIST_FAIL_INT(void);

/**
 * @brief irqTest29 - Test whether PMIC LLD can set and get the mask configuration
 * of IRQ ABIST_DONE_INT.
 */
void test_functionality_irqSetGetMask_ABIST_DONE_INT(void);

/**
 * @brief irqTest30 - Test whether PMIC LLD can set and get the mask configuration
 * of IRQ GPO_READBACK_INT.
 */
void test_functionality_irqSetGetMask_GPO_READBACK_INT(void);

/**
 * @brief irqTest31 - Test whether PMIC LLD can set and get the mask configuration
 * of IRQ NINT_READBACK_INT.
 */
void test_functionality_irqSetGetMask_NINT_READBACK_INT(void);

/**
 * @brief irqTest32 - Test whether PMIC LLD can set and get the mask configuration
 * of IRQ CONFIG_CRC_INT.
 */
void test_functionality_irqSetGetMask_CONFIG_CRC_INT(void);

/**
 * @brief irqTest33 - Test whether PMIC LLD can set and get the mask configuration
 * of IRQ TRIM_TEST_CRC_INT.
 */
void test_functionality_irqSetGetMask_TRIM_TEST_CRC_INT(void);

/**
 * @brief irqTest34 - Test PMIC LLD error handling of setting and getting the
 * mask configuration of NMI RECOV_CNT_INT.
 */
void test_faultHandling_irqSetGetMask_RECOV_CNT_INT(void);

/**
 * @brief irqTest35 - Test PMIC LLD error handling of setting and getting the
 * mask configuration of NMI TSD_IMM_INT.
 */
void test_faultHandling_irqSetGetMask_TSD_IMM_INT(void);

/**
 * @brief irqTest36 - Test PMIC LLD error handling of setting and getting the
 * mask configuration of NMI WD_FIRST_NOK_INT.
 */
void test_faultHandling_irqSetGetMask_WD_FIRST_NOK_INT(void);

/**
 * @brief irqTest37 - Test PMIC LLD error handling of setting and getting the
 * mask configuration of NMI WAIT_FOR_PWRCYCLE_INT.
 */
void test_faultHandling_irqSetGetMask_WAIT_FOR_PWRCYCLE_INT(void);

/**
 * @brief irqTest38 - Test PMIC LLD error handling of setting and getting the
 * mask configuration of NMI WARM_RESET_INT.
 */
void test_faultHandling_irqSetGetMask_WARM_RESET_INT(void);

/**
 * @brief irqTest39 - Test PMIC LLD error handling of setting and getting the
 * mask configuration of NMI ORD_SHUTDOWN_INT.
 */
void test_faultHandling_irqSetGetMask_ORD_SHUTDOWN_INT(void);

/**
 * @brief irqTest40 - Test PMIC LLD error handling of setting and getting the
 * mask configuration of NMI IMM_SHUTDOWN_INT.
 */
void test_faultHandling_irqSetGetMask_IMM_SHUTDOWN_INT(void);

/**
 * @brief irqTest41 - Test whether PMIC LLD can set and get the mask configuration
 * of IRQ MCU_COMM_ERR_INT.
 */
void test_functionality_irqSetGetMask_MCU_COMM_ERR_INT(void);

/**
 * @brief irqTest42 - Test whether PMIC LLD can set and get the mask configuration
 * of IRQ COMM_ADR_ERR_INT.
 */
void test_functionality_irqSetGetMask_COMM_ADR_ERR_INT(void);

/**
 * @brief irqTest43 - Test whether PMIC LLD can set and get the mask configuration
 * of IRQ COMM_CRC_ERR_INT.
 */
void test_functionality_irqSetGetMask_COMM_CRC_ERR_INT(void);

/**
 * @brief irqTest44 - Test whether PMIC LLD can set and get the mask configuration
 * of IRQ ESM_MCU_RST_INT.
 */
void test_functionality_irqSetGetMask_ESM_MCU_RST_INT(void);

/**
 * @brief irqTest45 - Test whether PMIC LLD can set and get the mask configuration
 * of IRQ ESM_MCU_FAIL_INT.
 */
void test_functionality_irqSetGetMask_ESM_MCU_FAIL_INT(void);

/**
 * @brief irqTest46 - Test whether PMIC LLD can set and get the mask configuration
 * of IRQ ESM_MCU_PIN_INT.
 */
void test_functionality_irqSetGetMask_ESM_MCU_PIN_INT(void);

/**
 * @brief irqTest47 - Test PMIC LLD error handling of setting and getting the
 * mask configuration of NMI WD_RST_INT.
 */
void test_faultHandling_irqSetGetMask_WD_RST_INT(void);

/**
 * @brief irqTest48 - Test PMIC LLD error handling of setting and getting the
 * mask configuration of NMI WD_FAIL_INT.
 */
void test_faultHandling_irqSetGetMask_WD_FAIL_INT(void);

/**
 * @brief irqTest49 - Test PMIC LLD error handling of setting and getting the
 * mask configuration of NMI WD_LONGWIN_TIMEOUT_INT.
 */
void test_faultHandling_irqSetGetMask_WD_LONGWIN_TIMEOUT_INT(void);

/**
 * @brief irqTest50 - Test whether PMIC LLD can set multiple mask configurations
 * within the same register.
 */
void test_functionality_irqSetGetMask_multipleMasks_esm(void);

/**
 * @brief irqTest51 - Test whether PMIC LLD can set multiple mask configurations
 * that reside in different registers.
 */
void test_functionality_irqSetGetMask_multipleMasks_esmBuck(void);

/**
 * @brief irqTest52 - Test Pmic_irqGetStat API error handling for when NULL
 * parameters are passed as input.
 */
void test_faultHandling_irqGetStat_nullParams(void);

/**
 * @brief irqTest53 - Test Pmic_irqGetNextFlag API error handling for when NULL
 * parameters are passed as input.
 */
void test_faultHandling_irqGetNextFlag_nullParams(void);

/**
 * @brief irqTest54 - Test Pmic_irqGetFlag API error handling for when an invalid
 * IRQ is passed as input.
 */
void test_faultHandling_irqGetFlag_invalidIrqNum(void);

/**
 * @brief irqTest55 - Test Pmic_irqGetFlag API error handling for when NULL
 * parameters are passed as input.
 */
void test_faultHandling_irqGetFlag_nullParams(void);

/**
 * @brief irqTest56 - Test PMIC LLD functionality for getting IRQ flags. This tests
 * Pmic_irqGetStat(), Pmic_irqGetNextFlag(), and Pmic_irqGetFlag().
 */
void test_functionality_irqGetFlags(void);

/**
 * @brief irqTest57 - Test Pmic_irqClrFlag API error handling for when an invalid
 * IRQ is passed as input.
 */
void test_faultHandling_irqClrFlag_invalidIrqNum(void);

/**
 * @brief irqTest58 - Test Pmic_irqClrFlag API error handling for when NULL
 * parameters are passed as input.
 */
void test_faultHandling_irqClrFlag_nullParams(void);

/**
 * @brief irqTest59 - Test whether PMIC LLD can clear IRQ status flags.
 */
void test_functionality_irqClrFlags(void);

#ifdef __cplusplus
}

#endif /* __cplusplus */
#endif /* __IRQ_TEST_H__ */
