/******************************************************************************
 * Copyright (c) 2025 Texas Instruments Incorporated - http://www.ti.com
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *****************************************************************************/



#ifndef IRQ_TEST_H
#define IRQ_TEST_H

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "../platform.h"
#include "pmic_irq.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 * @brief IRQ test suite entry point
 * @param args Test arguments (unused)
 */
void irq_test(void *args);

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/* Run all IRQ tests */
#define IRQ_TEST_RUN_ALL() \
    IRQ_TEST_RUN_POSITIVE(); \
    IRQ_TEST_RUN_NEGATIVE()

/* Run all IRQ positive tests */
#define IRQ_TEST_RUN_POSITIVE() \
    PLATFORM_RUN_TEST(test_positive_irq_setGetCfg_bb_uvErr_mask); \
    PLATFORM_RUN_TEST(test_positive_irq_setGetCfg_bb_ovErr_mask); \
    PLATFORM_RUN_TEST(test_positive_irq_setGetCfg_ldo1_uvErr_mask); \
    PLATFORM_RUN_TEST(test_positive_irq_setGetCfg_ldo1_ovErr_mask); \
    PLATFORM_RUN_TEST(test_positive_irq_setGetCfg_ldo2_uvErr_mask); \
    PLATFORM_RUN_TEST(test_positive_irq_setGetCfg_ldo2_ovErr_mask); \
    PLATFORM_RUN_TEST(test_positive_irq_setGetCfg_ldo3_uvErr_mask); \
    PLATFORM_RUN_TEST(test_positive_irq_setGetCfg_ldo3_ovErr_mask); \
    PLATFORM_RUN_TEST(test_positive_irq_setGetCfg_ldo4_uvErr_mask); \
    PLATFORM_RUN_TEST(test_positive_irq_setGetCfg_ldo4_ovErr_mask); \
    PLATFORM_RUN_TEST(test_positive_irq_setGetCfg_pldo1_uvErr_mask); \
    PLATFORM_RUN_TEST(test_positive_irq_setGetCfg_pldo1_ovErr_mask); \
    PLATFORM_RUN_TEST(test_positive_irq_setGetCfg_pldo2_uvErr_mask); \
    PLATFORM_RUN_TEST(test_positive_irq_setGetCfg_pldo2_ovErr_mask); \
    PLATFORM_RUN_TEST(test_positive_irq_setGetCfg_extVmon1_uvErr_mask); \
    PLATFORM_RUN_TEST(test_positive_irq_setGetCfg_extVmon1_ovErr_mask); \
    PLATFORM_RUN_TEST(test_positive_irq_setGetCfg_extVmon2_uvErr_mask); \
    PLATFORM_RUN_TEST(test_positive_irq_setGetCfg_extVmon2_ovErr_mask); \
    PLATFORM_RUN_TEST(test_positive_irq_setGetCfg_wdTh1Err_mask); \
    PLATFORM_RUN_TEST(test_positive_irq_setGetCfg_wdTh2Err_mask); \
    PLATFORM_RUN_TEST(test_positive_irq_setGetCfg_esmDly1Err_mask); \
    PLATFORM_RUN_TEST(test_positive_irq_setGetCfg_esmDly2Err_mask); \
    PLATFORM_RUN_TEST(test_positive_irq_setGetCfg_nrstRdbkErr_mask); \
    PLATFORM_RUN_TEST(test_positive_irq_setGetCfg_safeOut1RdbkErr_mask); \
    PLATFORM_RUN_TEST(test_positive_irq_setGetCfg_enOutRdbkErr_mask); \
    PLATFORM_RUN_TEST(test_positive_irq_setGetCfg_gpo1RdbkErr_mask); \
    PLATFORM_RUN_TEST(test_positive_irq_setGetCfg_gpo2RdbkErr_mask); \
    PLATFORM_RUN_TEST(test_positive_irq_setGetCfg_gpo3RdbkErr_mask); \
    PLATFORM_RUN_TEST(test_positive_irq_setGetCfg_gpo4RdbkErr_mask); \
    PLATFORM_RUN_TEST(test_positive_irq_setGetCfg_comp1pUvErr_mask); \
    PLATFORM_RUN_TEST(test_positive_irq_setGetCfg_comp1pOvErr_mask); \
    PLATFORM_RUN_TEST(test_positive_irq_setGetCfg_comp1nUvErr_mask); \
    PLATFORM_RUN_TEST(test_positive_irq_setGetCfg_comp1nOvErr_mask); \
    PLATFORM_RUN_TEST(test_positive_irq_setGetCfg_comp2pUvErr_mask); \
    PLATFORM_RUN_TEST(test_positive_irq_setGetCfg_comp2pOvErr_mask); \
    PLATFORM_RUN_TEST(test_positive_irq_setGetCfg_comp2nUvErr_mask); \
    PLATFORM_RUN_TEST(test_positive_irq_setGetCfg_comp2nOvErr_mask); \
    PLATFORM_RUN_TEST(test_positive_irq_setGetCfgs_multipleMasks); \
    PLATFORM_RUN_TEST(test_positive_irq_getStatus_allIrqs); \
    PLATFORM_RUN_TEST(test_positive_irq_getNextFlag); \
    PLATFORM_RUN_TEST(test_positive_irq_getFlagClrFlag); \
    PLATFORM_RUN_TEST(test_positive_irq_clrAllFlags)

/* Run all IRQ negative tests */
#define IRQ_TEST_RUN_NEGATIVE() \
    PLATFORM_RUN_TEST(test_negative_Pmic_irqSetCfg_nullParam_handle); \
    PLATFORM_RUN_TEST(test_negative_Pmic_irqSetCfg_nullParam_irqCfg); \
    PLATFORM_RUN_TEST(test_negative_Pmic_irqSetCfg_invalidParam_irqNum); \
    PLATFORM_RUN_TEST(test_negative_Pmic_irqSetCfgs_nullParam_handle); \
    PLATFORM_RUN_TEST(test_negative_Pmic_irqSetCfgs_nullParam_irqCfg); \
    PLATFORM_RUN_TEST(test_negative_Pmic_irqSetCfgs_invalidParam_numIrqs); \
    PLATFORM_RUN_TEST(test_negative_Pmic_irqGetCfg_nullParam_handle); \
    PLATFORM_RUN_TEST(test_negative_Pmic_irqGetCfg_nullParam_irqCfg); \
    PLATFORM_RUN_TEST(test_negative_Pmic_irqGetCfg_invalidParam_irqNum); \
    PLATFORM_RUN_TEST(test_negative_Pmic_irqGetCfgs_nullParam_handle); \
    PLATFORM_RUN_TEST(test_negative_Pmic_irqGetCfgs_nullParam_irqCfgs); \
    PLATFORM_RUN_TEST(test_negative_Pmic_irqGetCfgs_invalidParam_numIrqs); \
    PLATFORM_RUN_TEST(test_negative_Pmic_irqGetStatus_nullParam_handle); \
    PLATFORM_RUN_TEST(test_negative_Pmic_irqGetStatus_nullParam_irqStat); \
    PLATFORM_RUN_TEST(test_negative_Pmic_irqGetNextFlag_nullParam_irqStat); \
    PLATFORM_RUN_TEST(test_negative_Pmic_irqGetNextFlag_nullParam_irqNum); \
    PLATFORM_RUN_TEST(test_negative_Pmic_irqGetFlag_nullParam_handle); \
    PLATFORM_RUN_TEST(test_negative_Pmic_irqGetFlag_nullParam_flag); \
    PLATFORM_RUN_TEST(test_negative_Pmic_irqGetFlag_invalidParam_irqNum); \
    PLATFORM_RUN_TEST(test_negative_Pmic_irqClrFlag_nullParam_handle); \
    PLATFORM_RUN_TEST(test_negative_Pmic_irqClrFlag_invalidParam_irqNum); \
    PLATFORM_RUN_TEST(test_negative_Pmic_irqClrAllFlags_nullParam_handle); \
    PLATFORM_RUN_TEST(test_negative_irq_setMask_nonMaskableIrq); \
    PLATFORM_RUN_TEST(test_negative_irqSetCfg_config_nonConfigurableIrq_ABIST_ERR); \
    PLATFORM_RUN_TEST(test_negative_irqGetCfg_config_nonConfigurableIrq); \
    PLATFORM_RUN_TEST(test_negative_irqSetCfg_invalidConfig_cfgRegCrcErr); \
    PLATFORM_RUN_TEST(test_negative_irqSetCfg_invalidConfig_comp1pUvErr); \
    PLATFORM_RUN_TEST(test_negative_irqSetCfg_invalidConfig_comp2nOvErr); \
    PLATFORM_RUN_TEST(test_negative_irqSetCfg_invalidConfig_otherIrq); \
    PLATFORM_RUN_TEST(test_negative_irqSetCfgs_invalidConfigInBatch); \
    PLATFORM_RUN_TEST(test_negative_irqGetCfgs_nonConfigurableInBatch); \
    PLATFORM_RUN_TEST(test_positive_irqClrFlag_offStateStat1Register); \
    PLATFORM_RUN_TEST(test_positive_irqClrFlag_offStateStat2Register); \
    PLATFORM_RUN_TEST(test_positive_irqClrAllFlags_devErrStatPreservation); \
    PLATFORM_RUN_TEST(test_negative_irqSetMask_OFF_INT_EVT_ERR_INT); \
    PLATFORM_RUN_TEST(test_negative_irqSetMask_OFF_PROT_EVT_INT); \
    PLATFORM_RUN_TEST(test_negative_irqSetMask_FIRST_PWR_ON_INT); \
    PLATFORM_RUN_TEST(test_negative_irqSetMask_CLK_ERR_INT); \
    PLATFORM_RUN_TEST(test_negative_irqSetMask_INTERNAL_OV_INT); \
    PLATFORM_RUN_TEST(test_negative_irqSetMask_INIT_AN_TMO_INT); \
    PLATFORM_RUN_TEST(test_negative_irqSetMask_WD_TMO_INT); \
    PLATFORM_RUN_TEST(test_negative_irqSetMask_WD_TRIG_EARLY_INT); \
    PLATFORM_RUN_TEST(test_negative_irqSetMask_ESM_ERR_INT); \
    PLATFORM_RUN_TEST(test_positive_irqGetNextFlag_multipleFlagsSet); \
    PLATFORM_RUN_TEST(test_positive_irqSetCfg_configValue); \
    PLATFORM_RUN_TEST(test_positive_irqSetCfgs_batchConfigValues); \
    PLATFORM_RUN_TEST(test_negative_irqSetMask_invalidIrqNum); \
    PLATFORM_RUN_TEST(test_positive_irqClrAllFlags_ioFailureMidLoop); \
    PLATFORM_RUN_TEST(test_negative_irqSetCfg_invalidIrqNum_viaMask); \
    PLATFORM_RUN_TEST(test_negative_irqSetConfig_invalidIrqNum)

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* Positive test function declarations */
void test_positive_irq_setGetCfg_bb_uvErr_mask(void);
void test_positive_irq_setGetCfg_bb_ovErr_mask(void);
void test_positive_irq_setGetCfg_ldo1_uvErr_mask(void);
void test_positive_irq_setGetCfg_ldo1_ovErr_mask(void);
void test_positive_irq_setGetCfg_ldo2_uvErr_mask(void);
void test_positive_irq_setGetCfg_ldo2_ovErr_mask(void);
void test_positive_irq_setGetCfg_ldo3_uvErr_mask(void);
void test_positive_irq_setGetCfg_ldo3_ovErr_mask(void);
void test_positive_irq_setGetCfg_ldo4_uvErr_mask(void);
void test_positive_irq_setGetCfg_ldo4_ovErr_mask(void);
void test_positive_irq_setGetCfg_pldo1_uvErr_mask(void);
void test_positive_irq_setGetCfg_pldo1_ovErr_mask(void);
void test_positive_irq_setGetCfg_pldo2_uvErr_mask(void);
void test_positive_irq_setGetCfg_pldo2_ovErr_mask(void);
void test_positive_irq_setGetCfg_extVmon1_uvErr_mask(void);
void test_positive_irq_setGetCfg_extVmon1_ovErr_mask(void);
void test_positive_irq_setGetCfg_extVmon2_uvErr_mask(void);
void test_positive_irq_setGetCfg_extVmon2_ovErr_mask(void);
void test_positive_irq_setGetCfg_wdTh1Err_mask(void);
void test_positive_irq_setGetCfg_wdTh2Err_mask(void);
void test_positive_irq_setGetCfg_esmDly1Err_mask(void);
void test_positive_irq_setGetCfg_esmDly2Err_mask(void);
void test_positive_irq_setGetCfg_nrstRdbkErr_mask(void);
void test_positive_irq_setGetCfg_safeOut1RdbkErr_mask(void);
void test_positive_irq_setGetCfg_enOutRdbkErr_mask(void);
void test_positive_irq_setGetCfg_gpo1RdbkErr_mask(void);
void test_positive_irq_setGetCfg_gpo2RdbkErr_mask(void);
void test_positive_irq_setGetCfg_gpo3RdbkErr_mask(void);
void test_positive_irq_setGetCfg_gpo4RdbkErr_mask(void);
void test_positive_irq_setGetCfg_comp1pUvErr_mask(void);
void test_positive_irq_setGetCfg_comp1pOvErr_mask(void);
void test_positive_irq_setGetCfg_comp1nUvErr_mask(void);
void test_positive_irq_setGetCfg_comp1nOvErr_mask(void);
void test_positive_irq_setGetCfg_comp2pUvErr_mask(void);
void test_positive_irq_setGetCfg_comp2pOvErr_mask(void);
void test_positive_irq_setGetCfg_comp2nUvErr_mask(void);
void test_positive_irq_setGetCfg_comp2nOvErr_mask(void);
void test_positive_irq_setGetCfgs_multipleMasks(void);
void test_positive_irq_getStatus_allIrqs(void);
void test_positive_irq_getNextFlag(void);
void test_positive_irq_getFlagClrFlag(void);
void test_positive_irq_clrAllFlags(void);

/* Negative test function declarations */
void test_negative_Pmic_irqSetCfg_nullParam_handle(void);
void test_negative_Pmic_irqSetCfg_nullParam_irqCfg(void);
void test_negative_Pmic_irqSetCfg_invalidParam_irqNum(void);
void test_negative_Pmic_irqSetCfgs_nullParam_handle(void);
void test_negative_Pmic_irqSetCfgs_nullParam_irqCfg(void);
void test_negative_Pmic_irqSetCfgs_invalidParam_numIrqs(void);
void test_negative_Pmic_irqGetCfg_nullParam_handle(void);
void test_negative_Pmic_irqGetCfg_nullParam_irqCfg(void);
void test_negative_Pmic_irqGetCfg_invalidParam_irqNum(void);
void test_negative_Pmic_irqGetCfgs_nullParam_handle(void);
void test_negative_Pmic_irqGetCfgs_nullParam_irqCfgs(void);
void test_negative_Pmic_irqGetCfgs_invalidParam_numIrqs(void);
void test_negative_Pmic_irqGetStatus_nullParam_handle(void);
void test_negative_Pmic_irqGetStatus_nullParam_irqStat(void);
void test_negative_Pmic_irqGetNextFlag_nullParam_irqStat(void);
void test_negative_Pmic_irqGetNextFlag_nullParam_irqNum(void);
void test_negative_Pmic_irqGetFlag_nullParam_handle(void);
void test_negative_Pmic_irqGetFlag_nullParam_flag(void);
void test_negative_Pmic_irqGetFlag_invalidParam_irqNum(void);
void test_negative_Pmic_irqClrFlag_nullParam_handle(void);
void test_negative_Pmic_irqClrFlag_invalidParam_irqNum(void);
void test_negative_Pmic_irqClrAllFlags_nullParam_handle(void);
void test_negative_irq_setMask_nonMaskableIrq(void);

/* New test function declarations for Task 1B */
void test_negative_irqSetCfg_config_nonConfigurableIrq_ABIST_ERR(void);
void test_negative_irqGetCfg_config_nonConfigurableIrq(void);
void test_negative_irqSetCfg_invalidConfig_cfgRegCrcErr(void);
void test_negative_irqSetCfg_invalidConfig_comp1pUvErr(void);
void test_negative_irqSetCfg_invalidConfig_comp2nOvErr(void);
void test_negative_irqSetCfg_invalidConfig_otherIrq(void);
void test_negative_irqSetCfgs_invalidConfigInBatch(void);
void test_negative_irqGetCfgs_nonConfigurableInBatch(void);
void test_positive_irqClrFlag_offStateStat1Register(void);
void test_positive_irqClrFlag_offStateStat2Register(void);
void test_positive_irqClrAllFlags_devErrStatPreservation(void);
void test_negative_irqSetMask_OFF_INT_EVT_ERR_INT(void);
void test_negative_irqSetMask_OFF_PROT_EVT_INT(void);
void test_negative_irqSetMask_FIRST_PWR_ON_INT(void);
void test_negative_irqSetMask_CLK_ERR_INT(void);
void test_negative_irqSetMask_INTERNAL_OV_INT(void);
void test_negative_irqSetMask_INIT_AN_TMO_INT(void);
void test_negative_irqSetMask_WD_TMO_INT(void);
void test_negative_irqSetMask_WD_TRIG_EARLY_INT(void);
void test_negative_irqSetMask_ESM_ERR_INT(void);

/* Additional coverage tests */
void test_positive_irqGetNextFlag_multipleFlagsSet(void);
void test_positive_irqSetCfg_configValue(void);
void test_positive_irqSetCfgs_batchConfigValues(void);
void test_negative_irqSetMask_invalidIrqNum(void);
void test_positive_irqClrAllFlags_ioFailureMidLoop(void);

/* Coverage tests for pmic_irq.c validation paths */
void test_negative_irqSetCfg_invalidIrqNum_viaMask(void);
void test_negative_irqSetConfig_invalidIrqNum(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* IRQ_TEST_H */
