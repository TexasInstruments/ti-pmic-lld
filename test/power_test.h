/**
 * @file power_test.h
 *
 * @brief Header file containing prototypes of all Power module tests.
 */
#ifndef __POWER_TEST_H__
#define __POWER_TEST_H__

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

void powerTest(void *args);

/**
 * @brief powerTest0: Test Pmic_pwrSetBuckCfg() API error handling for when NULL
 * parameters are passed as input.
 */
void test_faultHandling_pwrSetBuckCfg_nullParams(void);

/**
 * @brief powerTest1: Test Pmic_pwrSetBuckCfg() API error handling for when no
 * valid parameters are specified.
 */
void test_faultHandling_pwrSetBuckCfg_noValidParams(void);

/**
 * @brief powerTest2: Test Pmic_pwrSetBuckCfg() API error handling for when an
 * invalid power resource is passed as input.
 */
void test_faultHandling_pwrSetBuckCfg_invalidResource(void);

/**
 * @brief powerTest3: Test Pmic_pwrGetBuckCfg() API error handling for when NULL
 * parameters are passed as input.
 */
void test_faultHandling_pwrGetBuckCfg_nullParams(void);

/**
 * @brief powerTest4: Test Pmic_pwrGetBuckCfg() API error handling for when no
 * valid parameters are specified.
 */
void test_faultHandling_pwrGetBuckCfg_noValidParams(void);

/**
 * @brief powerTest5: Test Pmic_pwrGetBuckCfg() API error handling for when an
 * invalid power resource is passed as input.
 */
void test_faultHandling_pwrGetBuckCfg_invalidResource(void);

/**
 * @brief powerTest6: Test whether PMIC LLD can set and get buck regulator
 * Enable configuration.
 */
void test_functionality_pwrSetGetBuckCfg_enable(void);

/**
 * @brief powerTest7: Test whether PMIC LLD can set and get buck regulator
 * Pulldown Resistor configuration.
 */
void test_functionality_pwrSetGetBuckCfg_pulldownResistor(void);

/**
 * @brief powerTest8: Test whether PMIC LLD can set and get buck regulator
 * FPWM configuration.
 */
void test_functionality_pwrSetGetBuckCfg_fpwm(void);

/**
 * @brief powerTest9: Test whether PMIC LLD can set and get buck regulator
 * UV Threshold configuration.
 */
void test_functionality_pwrSetGetBuckCfg_uvThr(void);

/**
 * @brief powerTest10: Test whether PMIC LLD can set and get buck regulator
 * OV Threshold configuration.
 */
void test_functionality_pwrSetGetBuckCfg_ovThr(void);

/**
 * @brief powerTest11: Test whether PMIC LLD can set and get buck regulator
 * ILIM configuration.
 */
void test_functionality_pwrSetGetBuckCfg_ilimSel(void);

/**
 * @brief powerTest12: Test whether PMIC LLD can set and get buck regulator
 * OVP Response configuration.
 */
void test_functionality_pwrSetGetBuckCfg_ovpSel(void);

/**
 * @brief powerTest13: Test whether PMIC LLD can set and get buck regulator
 * OV Response configuration.
 */

void test_functionality_pwrSetGetBuckCfg_ovSel(void);

/**
 * @brief powerTest14: Test whether PMIC LLD can set and get buck regulator
 * UV Response configuration.
 */
void test_functionality_pwrSetGetBuckCfg_uvSel(void);

/**
 * @brief powerTest15: Test whether PMIC LLD can set and get buck regulator
 * SC Response configuration.
 */
void test_functionality_pwrSetGetBuckCfg_scSel(void);

/**
 * @brief powerTest16: Test whether PMIC LLD can set and get buck regulator
 * Residual Voltage configuration.
 */
void test_functionality_pwrSetGetBuckCfg_rvConf(void);

/**
 * @brief powerTest17: Test whether PMIC LLD can set and get buck regulator
 * Slew Rate configuration.
 */
void test_functionality_pwrSetGetBuckCfg_slewRate(void);

/**
 * @brief powerTest18: Test whether PMIC LLD can set and get buck regulator
 * Deglitch Selection configuration.
 */
void test_functionality_pwrSetGetBuckCfg_deglitchSel(void);

/**
 * @brief powerTest19: Test whether PMIC LLD can set and get buck regulator
 * Discharge Selection configuration.
 */
void test_functionality_pwrSetGetBuckCfg_dischargeSel(void);

/**
 * @brief powerTest20: Test whether PMIC LLD can set and get buck regulator
 * Spread Spectrum Enable configuration.
 */
void test_functionality_pwrSetGetBuckCfg_ssEn(void);

/**
 * @brief powerTest21: Test whether PMIC LLD can set and get buck regulator
 * Spread Spectrum Modulation Selection configuration.
 */
void test_functionality_pwrSetGetBuckCfg_ssmSel(void);

/**
 * @brief powerTest22: Test whether PMIC LLD can set and get buck regulator
 * VSET configuration.
 */
void test_functionality_pwrSetGetBuckCfg_vset(void);

/**
 * @brief powerTest23: Test whether PMIC LLD can set and get buck regulator
 * UVLO Rising configuration.
 */
void test_functionality_pwrSetGetBuckCfg_uvloRising(void);

/**
 * @brief powerTest24: Test whether PMIC LLD can set and get buck regulator
 * UVLO Falling configuration.
 */
void test_functionality_pwrSetGetBuckCfg_uvloFalling(void);

/**
 * @brief powerTest25: Test whether PMIC LLD can set and get buck regulator
 * High Side Slew Rate configuration.
 */
void test_functionality_pwrSetGetBuckCfg_highSideSlewRate(void);

/**
 * @brief powerTest26: Test whether PMIC LLD can set and get buck regulator
 * Active VSET configuration.
 */
void test_functionality_pwrSetGetBuckCfg_vsetActive(void);

/**
 * @brief powerTest27: Test whether PMIC LLD can set and get buck regulator
 * Low Power VSET configuration.
 */
void test_functionality_pwrSetGetBuckCfg_vsetLPwr(void);

/**
 * @brief powerTest28: Test whether PMIC LLD can set and get buck regulator
 * VMON Only configuration.
 */
void test_functionality_pwrSetGetBuckCfg_vmonOnly(void);

/**
 * @brief powerTest29: Test Pmic_pwrSetLdoCfg() API error handling for when NULL
 * parameters are passed as input.
 */
void test_faultHandling_pwrSetLdoCfg_nullParams(void);

/**
 * @brief powerTest30: Test Pmic_pwrSetLdoCfg() API error handling for when no
 * valid parameters are specified.
 */
void test_faultHandling_pwrSetLdoCfg_noValidParams(void);

/**
 * @brief powerTest31: Test Pmic_pwrGetLdoCfg() API error handling for when NULL
 * parameters are passed as input.
 */
void test_faultHandling_pwrGetLdoCfg_nullParams(void);

/**
 * @brief powerTest32: Test Pmic_pwrGetLdoCfg() API error handling for when no
 * valid parameters are specified.
 */
void test_faultHandling_pwrGetLdoCfg_noValidParams(void);

/**
 * @brief powerTest33: Test whether PMIC LLD can set and get LDO regulator
 * Enable configuration.
 */
void test_functionality_pwrSetGetLdoCfg_enable(void);

/**
 * @brief powerTest34: Test whether PMIC LLD can set and get LDO regulator
 * Mode configuration.
 */
void test_functionality_pwrSetGetLdoCfg_mode(void);

/**
 * @brief powerTest35: Test whether PMIC LLD can set and get LDO regulator
 * VSET configuration.
 */
void test_functionality_pwrSetGetLdoCfg_vset(void);

/**
 * @brief powerTest36: Test whether PMIC LLD can set and get LDO regulator
 * VMON Only configuration.
 */
void test_functionality_pwrSetGetLdoCfg_vmonOnly(void);

/**
 * @brief powerTest37: Test whether PMIC LLD can set and get LDO regulator
 * Discharge Enable configuration.
 */
void test_functionality_pwrSetGetLdoCfg_dischargeEn(void);

/**
 * @brief powerTest38: Test whether PMIC LLD can set and get LDO regulator
 * Discharge Select configuration.
 */
void test_functionality_pwrSetGetLdoCfg_dischargeSel(void);

/**
 * @brief powerTest39: Test whether PMIC LLD can set and get LDO regulator
 * Deglitch Selection configuration.
 */
void test_functionality_pwrSetGetLdoCfg_deglitchSel(void);

/**
 * @brief powerTest40: Test whether PMIC LLD can set and get LDO regulator
 * UV Threshold configuration.
 */
void test_functionality_pwrSetGetLdoCfg_uvThr(void);

/**
 * @brief powerTest41: Test whether PMIC LLD can set and get LDO regulator
 * OV Threshold configuration.
 */
void test_functionality_pwrSetGetLdoCfg_ovThr(void);

/**
 * @brief powerTest42: Test whether PMIC LLD can set and get LDO regulator
 * ILIM configuration.
 */
void test_functionality_pwrSetGetLdoCfg_ilimSel(void);

/**
 * @brief powerTest43: Test whether PMIC LLD can set and get LDO regulator
 * OVP Response configuration.
 */
void test_functionality_pwrSetGetLdoCfg_ovpSel(void);

/**
 * @brief powerTest44: Test whether PMIC LLD can set and get LDO regulator
 * OV Response configuration.
 */
void test_functionality_pwrSetGetLdoCfg_ovSel(void);

/**
 * @brief powerTest45: Test whether PMIC LLD can set and get LDO regulator
 * UV Response configuration.
 */
void test_functionality_pwrSetGetLdoCfg_uvSel(void);

/**
 * @brief powerTest46: Test whether PMIC LLD can set and get LDO regulator
 * SC Response configuration.
 */
void test_functionality_pwrSetGetLdoCfg_scSel(void);

/**
 * @brief powerTest47: Test whether PMIC LLD can set and get LDO regulator
 * Residual Voltage configuration.
 */
void test_functionality_pwrSetGetLdoCfg_rvConf(void);

/**
 * @brief powerTest48: Test Pmic_pwrGetRsrcStat() API error handling for when
 * NULL parameters are passed as input.
 */
void test_faultHandling_pwrGetRsrcStat_nullParams(void);

/**
 * @brief powerTest49: Test Pmic_pwrGetRsrcStat() API error handling for when
 * an invalid power resource is passed as input.
 */
void test_faultHandling_pwrGetRsrcStat_invalidResource(void);

/**
 * @brief powerTest50: Test whether PMIC LLD can get the status of a power
 * resource.
 */
void test_functionality_pwrGetRsrcStat(void);

/**
 * @brief powerTest51: Test Pmic_pwrSetTsdCfg() API error handling for when NULL
 * parameters are passed as input.
 */
void test_faultHandling_pwrSetTsdCfg_nullParams(void);

/**
 * @brief powerTest52: Test Pmic_pwrSetTsdCfg() API error handling for when no
 * valid parameters are specified.
 */
void test_faultHandling_pwrSetTsdCfg_noValidParams(void);

/**
 * @brief powerTest53: Test Pmic_pwrGetTsdCfg() API error handling for when NULL
 * parameters are passed as input.
 */
void test_faultHandling_pwrGetTsdCfg_nullParams(void);

/**
 * @brief powerTest54: Test Pmic_pwrGetTsdCfg() API error handling for when no
 * valid parameters are specified.
 */
void test_faultHandling_pwrGetTsdCfg_noValidParams(void);

/**
 * @brief powerTest55: Test whether PMIC LLD can set and get the TWARN_CONFIG
 * bit field configuration.
 */
void test_functionality_pwrSetGetTsdCfg_twarnStayInSafeState(void);

/**
 * @brief powerTest56: Test whether PMIC LLD can set and get the TSD_IMM_LEVEL
 * bit field configuration.
 */
void test_functionality_pwrSetGetTsdCfg_tsdImmLevel(void);

/**
 * @brief powerTest57: Test whether PMIC LLD can set and get the TWARN_LEVEL
 * bit field configuration.
 */
void test_functionality_pwrSetGetTsdCfg_twarnLevel(void);

/**
 * @brief powerTest58: Test Pmic_pwrSetBuckLdoSeqTrig() API error handling for
 * when NULL parameters are passed as input.
 */
void test_faultHandling_pwrSetBuckLdoSeqTrig_nullParams(void);

/**
 * @brief powerTest59: Test Pmic_pwrSetBuckLdoSeqTrig() API error handling for
 * when an invalid trigger is passed as input.
 */
void test_faultHandling_pwrSetBuckLdoSeqTrig_invalidTrigger(void);

/**
 * @brief powerTest60: Test Pmic_pwrGetBuckLdoSeqTrig() API error handling for
 * when NULL parameters are passed as input.
 */
void test_faultHandling_pwrGetBuckLdoSeqTrig_nullParams(void);

/**
 * @brief powerTest61: Test Pmic_pwrGetBuckLdoSeqTrig() API error handling for
 * when an invalid trigger is passed as input.
 */
void test_faultHandling_pwrGetBuckLdoSeqTrig_invalidTrigger(void);

/**
 * @brief powerTest62: Test whether PMIC LLD can set and get BUCK1 PWR_ON_BIT
 * trigger.
 */
void test_functionality_pwrSetGetBuckLdoSeqTrig_BUCK1_TRIGGER_PWR_ON_BIT(void);

/**
 * @brief powerTest63: Test whether PMIC LLD can set and get BUCK1 LDO_PG
 * trigger.
 */
void test_functionality_pwrSetGetBuckLdoSeqTrig_BUCK1_TRIGGER_LDO_PG(void);

/**
 * @brief powerTest64: Test whether PMIC LLD can set and get BUCK1 BUCK3_PG
 * trigger.
 */
void test_functionality_pwrSetGetBuckLdoSeqTrig_BUCK1_TRIGGER_BUCK3_PG(void);

/**
 * @brief powerTest65: Test whether PMIC LLD can set and get BUCK1 BUCK2_PG
 * trigger.
 */
void test_functionality_pwrSetGetBuckLdoSeqTrig_BUCK1_TRIGGER_BUCK2_PG(void);

/**
 * @brief powerTest66: Test whether PMIC LLD can set and get BUCK1 GPIO_PIN
 * trigger.
 */
void test_functionality_pwrSetGetBuckLdoSeqTrig_BUCK1_TRIGGER_GPIO_PIN(void);

/**
 * @brief powerTest67: Test whether PMIC LLD can set and get BUCK1 SEQ_PIN
 * trigger.
 */
void test_functionality_pwrSetGetBuckLdoSeqTrig_BUCK1_TRIGGER_SEQ_PIN(void);

/**
 * @brief powerTest68: Test whether PMIC LLD can set and get BUCK2 GPIO_PIN
 * trigger.
 */
void test_functionality_pwrSetGetBuckLdoSeqTrig_BUCK2_TRIGGER_PWR_ON_BIT(void);

/**
 * @brief powerTest69: Test whether PMIC LLD can set and get BUCK2 LDO_PG
 * trigger.
 */
void test_functionality_pwrSetGetBuckLdoSeqTrig_BUCK2_TRIGGER_LDO_PG(void);

/**
 * @brief powerTest70: Test whether PMIC LLD can set and get BUCK2 BUCK3_PG
 * trigger.
 */
void test_functionality_pwrSetGetBuckLdoSeqTrig_BUCK2_TRIGGER_BUCK3_PG(void);

/**
 * @brief powerTest71: Test whether PMIC LLD can set and get BUCK2 BUCK1_PG
 * trigger.
 */
void test_functionality_pwrSetGetBuckLdoSeqTrig_BUCK2_TRIGGER_BUCK1_PG(void);

/**
 * @brief powerTest72: Test whether PMIC LLD can set and get BUCK2 GPIO_PIN
 * trigger.
 */
void test_functionality_pwrSetGetBuckLdoSeqTrig_BUCK2_TRIGGER_GPIO_PIN(void);

/**
 * @brief powerTest73: Test whether PMIC LLD can set and get BUCK2 SEQ_PIN
 * trigger.
 */
void test_functionality_pwrSetGetBuckLdoSeqTrig_BUCK2_TRIGGER_SEQ_PIN(void);

/**
 * @brief powerTest74: Test whether PMIC LLD can set and get BUCK3 PWR_ON_BIT
 * trigger.
 */
void test_functionality_pwrSetGetBuckLdoSeqTrig_BUCK3_TRIGGER_PWR_ON_BIT(void);

/**
 * @brief powerTest75: Test whether PMIC LLD can set and get BUCK3 LDO_PG
 * trigger.
 */
void test_functionality_pwrSetGetBuckLdoSeqTrig_BUCK3_TRIGGER_LDO_PG(void);

/**
 * @brief powerTest76: Test whether PMIC LLD can set and get BUCK3 BUCK2_PG
 * trigger.
 */
void test_functionality_pwrSetGetBuckLdoSeqTrig_BUCK3_TRIGGER_BUCK2_PG(void);

/**
 * @brief powerTest77: Test whether PMIC LLD can set and get BUCK3 BUCK1_PG
 * trigger.
 */
void test_functionality_pwrSetGetBuckLdoSeqTrig_BUCK3_TRIGGER_BUCK1_PG(void);

/**
 * @brief powerTest78: Test whether PMIC LLD can set and get BUCK3 GPIO_PIN
 * trigger.
 */
void test_functionality_pwrSetGetBuckLdoSeqTrig_BUCK3_TRIGGER_GPIO_PIN(void);

/**
 * @brief powerTest79: Test whether PMIC LLD can set and get BUCK3 SEQ_PIN
 * trigger.
 */
void test_functionality_pwrSetGetBuckLdoSeqTrig_BUCK3_TRIGGER_SEQ_PIN(void);

/**
 * @brief powerTest80: Test whether PMIC LLD can set and get LDO PWR_ON_BIT
 * trigger.
 */
void test_functionality_pwrSetGetBuckLdoSeqTrig_LDO_TRIGGER_PWR_ON_BIT(void);

/**
 * @brief powerTest81: Test whether PMIC LLD can set and get LDO BUCK3_PG
 * trigger.
 */
void test_functionality_pwrSetGetBuckLdoSeqTrig_LDO_TRIGGER_BUCK3_PG(void);

/**
 * @brief powerTest82: Test whether PMIC LLD can set and get LDO BUCK2_PG
 * trigger.
 */
void test_functionality_pwrSetGetBuckLdoSeqTrig_LDO_TRIGGER_BUCK2_PG(void);

/**
 * @brief powerTest83: Test whether PMIC LLD can set and get LDO BUCK1_PG
 * trigger.
 */
void test_functionality_pwrSetGetBuckLdoSeqTrig_LDO_TRIGGER_BUCK1_PG(void);

/**
 * @brief powerTest84: Test whether PMIC LLD can set and get LDO GPIO_PIN
 * trigger.
 */
void test_functionality_pwrSetGetBuckLdoSeqTrig_LDO_TRIGGER_GPIO_PIN(void);

/**
 * @brief powerTest85: Test whether PMIC LLD can set and get LDO SEQ_PIN
 * trigger.
 */
void test_functionality_pwrSetGetBuckLdoSeqTrig_LDO_TRIGGER_SEQ_PIN(void);

/**
 * @brief powerTest86: Test whether PMIC LLD can set and get multiple
 * triggers.
 */
void test_functionality_pwrSetGetBuckLdoSeqTrig_multipleTriggers(void);

/**
 * @brief powerTest87: Test Pmic_pwrSetBuckLdoSeqDly() API error handling for
 * when NULL parameters are passed as input.
 */
void test_faultHandling_pwrSetBuckLdoSeqDly_nullParams(void);

/**
 * @brief powerTest88: Test Pmic_pwrSetBuckLdoSeqDly() API error handling for
 * when an invalid resource is passed as input.
 */
void test_faultHandling_pwrSetBuckLdoSeqDly_invalidResource(void);

/**
 * @brief powerTest89: Test Pmic_pwrSetBuckLdoSeqDly() API error handling for
 * when an invalid SEQ_DLY_OFF value is passed as input.
 */
void test_faultHandling_pwrSetBuckLdoSeqDly_invalidSeqDlyOff(void);

/**
 * @brief powerTest90: Test Pmic_pwrSetBuckLdoSeqDly() API error handling for
 * when an invalid SEQ_DLY_ON value is passed as input.
 */
void test_faultHandling_pwrSetBuckLdoSeqDly_invalidSeqDlyOn(void);

/**
 * @brief powerTest91: Test pwrGetBuckLdoSeqDly() API error handling for when
 * NULL parameters are passed as input.
 */
void test_faultHandling_pwrGetBuckLdoSeqDly_nullParams(void);

/**
 * @brief powerTest92: Test pwrGetBuckLdoSeqDly() API error handling for when an
 * invalid power resource is passed as input.
 */
void test_faultHandling_pwrGetBuckLdoSeqDly_invalidResource(void);

/**
 * @brief powerTest93: Test whether PMIC LLD can set and get SEQ_DLY_ON
 * configuration for regulators.
 */
void test_functionality_pwrSetGetBuckLdoSeqDly_seqDlyOn(void);

/**
 * @brief powerTest94: Test whether PMIC LLD can set and get SEQ_DLY_OFF
 * configuration for regulators.
 */
void test_functionality_pwrSetGetBuckLdoSeqDly_seqDlyOff(void);

#ifdef __cplusplus
}

#endif /* __cplusplus */
#endif /* __POWER_TEST_H__ */
