/******************************************************************************
 * Copyright (c) 2026 Texas Instruments Incorporated - http://www.ti.com
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

#include "platform.h"
#include "pmic_irq.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/* ======================================================================== */
/*                           Test APIs: irqSetCfg                           */
/* ======================================================================== */

#define IRQ_TEST_POS_IRQSETCFG() \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetCfg_bb_uvErr_mask); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetCfg_bb_ovErr_mask); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetCfg_ldo1_uvErr_mask); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetCfg_ldo1_ovErr_mask); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetCfg_ldo2_uvErr_mask); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetCfg_ldo2_ovErr_mask); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetCfg_ldo3_uvErr_mask); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetCfg_ldo3_ovErr_mask); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetCfg_ldo4_uvErr_mask); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetCfg_ldo4_ovErr_mask); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetCfg_pldo1_uvErr_mask); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetCfg_pldo1_ovErr_mask); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetCfg_pldo2_uvErr_mask); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetCfg_pldo2_ovErr_mask); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetCfg_extVmon1_uvErr_mask); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetCfg_extVmon1_ovErr_mask); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetCfg_extVmon2_uvErr_mask); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetCfg_extVmon2_ovErr_mask); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetCfg_wdTh1Err_mask); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetCfg_wdTh2Err_mask); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetCfg_esmDly1Err_mask); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetCfg_esmDly2Err_mask); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetCfg_nrstRdbkErr_mask); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetCfg_safeOut1RdbkErr_mask); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetCfg_enOutRdbkErr_mask); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetCfg_gpo1RdbkErr_mask); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetCfg_gpo2RdbkErr_mask); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetCfg_gpo3RdbkErr_mask); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetCfg_gpo4RdbkErr_mask); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetCfg_comp1pUvErr_mask); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetCfg_comp1pOvErr_mask); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetCfg_comp1nUvErr_mask); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetCfg_comp1nOvErr_mask); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetCfg_comp2pUvErr_mask); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetCfg_comp2pOvErr_mask); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetCfg_comp2nUvErr_mask); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetCfg_comp2nOvErr_mask); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetCfg_configValue)

#define IRQ_TEST_NEG_IRQSETCFG() \
    PLATFORM_RUN_TEST(test_neg_irq_irqSetCfg_nullParam_handle); \
    PLATFORM_RUN_TEST(test_neg_irq_irqSetCfg_nullParam_irqCfg); \
    PLATFORM_RUN_TEST(test_neg_irq_irqSetCfg_invalidParam_irqNum); \
    PLATFORM_RUN_TEST(test_neg_irq_irqSetCfg_nonMaskableIrq); \
    PLATFORM_RUN_TEST(test_neg_irq_irqSetCfg_nonConfigurableIrq_ABIST_ERR); \
    PLATFORM_RUN_TEST(test_neg_irq_irqSetCfg_invalidConfig_cfgRegCrcErr); \
    PLATFORM_RUN_TEST(test_neg_irq_irqSetCfg_invalidConfig_comp1pUvErr); \
    PLATFORM_RUN_TEST(test_neg_irq_irqSetCfg_invalidConfig_comp2nOvErr); \
    PLATFORM_RUN_TEST(test_neg_irq_irqSetCfg_invalidConfig_otherIrq); \
    PLATFORM_RUN_TEST(test_neg_irq_irqSetCfg_maskNonMaskable_OFF_INT_EVT_ERR_INT); \
    PLATFORM_RUN_TEST(test_neg_irq_irqSetCfg_maskNonMaskable_OFF_PROT_EVT_INT); \
    PLATFORM_RUN_TEST(test_neg_irq_irqSetCfg_maskNonMaskable_FIRST_PWR_ON_INT); \
    PLATFORM_RUN_TEST(test_neg_irq_irqSetCfg_maskNonMaskable_CLK_ERR_INT); \
    PLATFORM_RUN_TEST(test_neg_irq_irqSetCfg_maskNonMaskable_INTERNAL_OV_INT); \
    PLATFORM_RUN_TEST(test_neg_irq_irqSetCfg_maskNonMaskable_INIT_AN_TMO_INT); \
    PLATFORM_RUN_TEST(test_neg_irq_irqSetCfg_maskNonMaskable_WD_TMO_INT); \
    PLATFORM_RUN_TEST(test_neg_irq_irqSetCfg_maskNonMaskable_WD_TRIG_EARLY_INT); \
    PLATFORM_RUN_TEST(test_neg_irq_irqSetCfg_maskNonMaskable_ESM_ERR_INT); \
    PLATFORM_RUN_TEST(test_neg_irq_irqSetCfg_invalidIrqNum_mask); \
    PLATFORM_RUN_TEST(test_neg_irq_irqSetCfg_invalidIrqNum_viaMask); \
    PLATFORM_RUN_TEST(test_neg_irq_irqSetCfg_invalidIrqNum_config)

/* Test: TC-IRQ-0001 */
#define IRQ_TEST_IRQSETCFG() \
    IRQ_TEST_POS_IRQSETCFG(); \
    IRQ_TEST_NEG_IRQSETCFG()

/* ======================================================================== */
/*                           Test APIs: irqGetCfg                           */
/* ======================================================================== */

#define IRQ_TEST_POS_IRQGETCFG() \
    /* Positive tests for irqGetCfg are combined with irqSetCfg tests */

#define IRQ_TEST_NEG_IRQGETCFG() \
    PLATFORM_RUN_TEST(test_neg_irq_irqGetCfg_nullParam_handle); \
    PLATFORM_RUN_TEST(test_neg_irq_irqGetCfg_nullParam_irqCfg); \
    PLATFORM_RUN_TEST(test_neg_irq_irqGetCfg_invalidParam_irqNum); \
    PLATFORM_RUN_TEST(test_neg_irq_irqGetCfg_nonConfigurableIrq)

/* Test: TC-IRQ-0002 */
#define IRQ_TEST_IRQGETCFG() \
    IRQ_TEST_POS_IRQGETCFG(); \
    IRQ_TEST_NEG_IRQGETCFG()

/* ======================================================================== */
/*                          Test APIs: irqSetCfgs                           */
/* ======================================================================== */

#define IRQ_TEST_POS_IRQSETCFGS() \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetCfgs_multipleMasks); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetCfgs_batchConfigValues)

#define IRQ_TEST_NEG_IRQSETCFGS() \
    PLATFORM_RUN_TEST(test_neg_irq_irqSetCfgs_nullParam_handle); \
    PLATFORM_RUN_TEST(test_neg_irq_irqSetCfgs_nullParam_irqCfg); \
    PLATFORM_RUN_TEST(test_neg_irq_irqSetCfgs_invalidParam_numIrqs); \
    PLATFORM_RUN_TEST(test_neg_irq_irqSetCfgs_invalidConfigInBatch)

/* Test: TC-IRQ-0003 */
#define IRQ_TEST_IRQSETCFGS() \
    IRQ_TEST_POS_IRQSETCFGS(); \
    IRQ_TEST_NEG_IRQSETCFGS()

/* ======================================================================== */
/*                          Test APIs: irqGetCfgs                           */
/* ======================================================================== */

#define IRQ_TEST_POS_IRQGETCFGS() \
    /* Positive tests for irqGetCfgs are combined with irqSetCfgs tests */

#define IRQ_TEST_NEG_IRQGETCFGS() \
    PLATFORM_RUN_TEST(test_neg_irq_irqGetCfgs_nullParam_handle); \
    PLATFORM_RUN_TEST(test_neg_irq_irqGetCfgs_nullParam_irqCfgs); \
    PLATFORM_RUN_TEST(test_neg_irq_irqGetCfgs_invalidParam_numIrqs); \
    PLATFORM_RUN_TEST(test_neg_irq_irqGetCfgs_nonConfigurableInBatch)

/* Test: TC-IRQ-0004 */
#define IRQ_TEST_IRQGETCFGS() \
    IRQ_TEST_POS_IRQGETCFGS(); \
    IRQ_TEST_NEG_IRQGETCFGS()

/* ======================================================================== */
/*                         Test APIs: irqGetStatus                          */
/* ======================================================================== */

#define IRQ_TEST_POS_IRQGETSTATUS() \
    PLATFORM_RUN_TEST(test_pos_irq_irqGetStatus_allIrqs)

#define IRQ_TEST_NEG_IRQGETSTATUS() \
    PLATFORM_RUN_TEST(test_neg_irq_irqGetStatus_nullParam_handle); \
    PLATFORM_RUN_TEST(test_neg_irq_irqGetStatus_nullParam_irqStat)

/* Test: TC-IRQ-0005 */
#define IRQ_TEST_IRQGETSTATUS() \
    IRQ_TEST_POS_IRQGETSTATUS(); \
    IRQ_TEST_NEG_IRQGETSTATUS()

/* ======================================================================== */
/*                        Test APIs: irqGetNextFlag                         */
/* ======================================================================== */

#define IRQ_TEST_POS_IRQGETNEXTFLAG() \
    PLATFORM_RUN_TEST(test_pos_irq_irqGetNextFlag_singleFlag); \
    PLATFORM_RUN_TEST(test_pos_irq_irqGetNextFlag_multipleFlagsSet)

#define IRQ_TEST_NEG_IRQGETNEXTFLAG() \
    PLATFORM_RUN_TEST(test_neg_irq_irqGetNextFlag_nullParam_irqStat); \
    PLATFORM_RUN_TEST(test_neg_irq_irqGetNextFlag_nullParam_irqNum)

/* Test: TC-IRQ-0006 */
#define IRQ_TEST_IRQGETNEXTFLAG() \
    IRQ_TEST_POS_IRQGETNEXTFLAG(); \
    IRQ_TEST_NEG_IRQGETNEXTFLAG()

/* ======================================================================== */
/*                          Test APIs: irqGetFlag                           */
/* ======================================================================== */

#define IRQ_TEST_POS_IRQGETFLAG() \
    PLATFORM_RUN_TEST(test_pos_irq_irqGetFlag_andClrFlag)

#define IRQ_TEST_NEG_IRQGETFLAG() \
    PLATFORM_RUN_TEST(test_neg_irq_irqGetFlag_nullParam_handle); \
    PLATFORM_RUN_TEST(test_neg_irq_irqGetFlag_nullParam_flag); \
    PLATFORM_RUN_TEST(test_neg_irq_irqGetFlag_invalidParam_irqNum)

/* Test: TC-IRQ-0007 */
#define IRQ_TEST_IRQGETFLAG() \
    IRQ_TEST_POS_IRQGETFLAG(); \
    IRQ_TEST_NEG_IRQGETFLAG()

/* ======================================================================== */
/*                          Test APIs: irqClrFlag                           */
/* ======================================================================== */

#define IRQ_TEST_POS_IRQCLRFLAG() \
    PLATFORM_RUN_TEST(test_pos_irq_irqClrFlag_offStateStat1Register); \
    PLATFORM_RUN_TEST(test_pos_irq_irqClrFlag_offStateStat2Register)

#define IRQ_TEST_NEG_IRQCLRFLAG() \
    PLATFORM_RUN_TEST(test_neg_irq_irqClrFlag_nullParam_handle); \
    PLATFORM_RUN_TEST(test_neg_irq_irqClrFlag_invalidParam_irqNum)

/* Test: TC-IRQ-0008 */
#define IRQ_TEST_IRQCLRFLAG() \
    IRQ_TEST_POS_IRQCLRFLAG(); \
    IRQ_TEST_NEG_IRQCLRFLAG()

/* ======================================================================== */
/*                        Test APIs: irqClrAllFlags                         */
/* ======================================================================== */

#define IRQ_TEST_POS_IRQCLRALLFLAGS() \
    PLATFORM_RUN_TEST(test_pos_irq_irqClrAllFlags_basic); \
    PLATFORM_RUN_TEST(test_pos_irq_irqClrAllFlags_devErrStatPreservation); \
    PLATFORM_RUN_TEST(test_pos_irq_irqClrAllFlags_ioFailureMidLoop)

#define IRQ_TEST_NEG_IRQCLRALLFLAGS() \
    PLATFORM_RUN_TEST(test_neg_irq_irqClrAllFlags_nullParam_handle)

/* Test: TC-IRQ-0009 */
#define IRQ_TEST_IRQCLRALLFLAGS() \
    IRQ_TEST_POS_IRQCLRALLFLAGS(); \
    IRQ_TEST_NEG_IRQCLRALLFLAGS()

/* ========================================================================== */
/*                         Aggregate Test Runners                             */
/* ========================================================================== */

#define IRQ_TEST_RUN_POSITIVE() \
    IRQ_TEST_POS_IRQSETCFG(); \
    IRQ_TEST_POS_IRQGETCFG(); \
    IRQ_TEST_POS_IRQSETCFGS(); \
    IRQ_TEST_POS_IRQGETCFGS(); \
    IRQ_TEST_POS_IRQGETSTATUS(); \
    IRQ_TEST_POS_IRQGETNEXTFLAG(); \
    IRQ_TEST_POS_IRQGETFLAG(); \
    IRQ_TEST_POS_IRQCLRFLAG(); \
    IRQ_TEST_POS_IRQCLRALLFLAGS()

#define IRQ_TEST_RUN_NEGATIVE() \
    IRQ_TEST_NEG_IRQSETCFG(); \
    IRQ_TEST_NEG_IRQGETCFG(); \
    IRQ_TEST_NEG_IRQSETCFGS(); \
    IRQ_TEST_NEG_IRQGETCFGS(); \
    IRQ_TEST_NEG_IRQGETSTATUS(); \
    IRQ_TEST_NEG_IRQGETNEXTFLAG(); \
    IRQ_TEST_NEG_IRQGETFLAG(); \
    IRQ_TEST_NEG_IRQCLRFLAG(); \
    IRQ_TEST_NEG_IRQCLRALLFLAGS()

#define IRQ_TEST_RUN_ALL() \
    IRQ_TEST_RUN_POSITIVE(); \
    IRQ_TEST_RUN_NEGATIVE()

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 * @brief IRQ test suite entry point
 * @param args Test arguments (unused)
 */
void irq_test(void *args);

/* ========================================================================== */
/*                       irqSetCfg API Tests                                  */
/* ========================================================================== */

/* Positive tests */
void test_pos_irq_irqSetCfg_bb_uvErr_mask(void);
void test_pos_irq_irqSetCfg_bb_ovErr_mask(void);
void test_pos_irq_irqSetCfg_ldo1_uvErr_mask(void);
void test_pos_irq_irqSetCfg_ldo1_ovErr_mask(void);
void test_pos_irq_irqSetCfg_ldo2_uvErr_mask(void);
void test_pos_irq_irqSetCfg_ldo2_ovErr_mask(void);
void test_pos_irq_irqSetCfg_ldo3_uvErr_mask(void);
void test_pos_irq_irqSetCfg_ldo3_ovErr_mask(void);
void test_pos_irq_irqSetCfg_ldo4_uvErr_mask(void);
void test_pos_irq_irqSetCfg_ldo4_ovErr_mask(void);
void test_pos_irq_irqSetCfg_pldo1_uvErr_mask(void);
void test_pos_irq_irqSetCfg_pldo1_ovErr_mask(void);
void test_pos_irq_irqSetCfg_pldo2_uvErr_mask(void);
void test_pos_irq_irqSetCfg_pldo2_ovErr_mask(void);
void test_pos_irq_irqSetCfg_extVmon1_uvErr_mask(void);
void test_pos_irq_irqSetCfg_extVmon1_ovErr_mask(void);
void test_pos_irq_irqSetCfg_extVmon2_uvErr_mask(void);
void test_pos_irq_irqSetCfg_extVmon2_ovErr_mask(void);
void test_pos_irq_irqSetCfg_wdTh1Err_mask(void);
void test_pos_irq_irqSetCfg_wdTh2Err_mask(void);
void test_pos_irq_irqSetCfg_esmDly1Err_mask(void);
void test_pos_irq_irqSetCfg_esmDly2Err_mask(void);
void test_pos_irq_irqSetCfg_nrstRdbkErr_mask(void);
void test_pos_irq_irqSetCfg_safeOut1RdbkErr_mask(void);
void test_pos_irq_irqSetCfg_enOutRdbkErr_mask(void);
void test_pos_irq_irqSetCfg_gpo1RdbkErr_mask(void);
void test_pos_irq_irqSetCfg_gpo2RdbkErr_mask(void);
void test_pos_irq_irqSetCfg_gpo3RdbkErr_mask(void);
void test_pos_irq_irqSetCfg_gpo4RdbkErr_mask(void);
void test_pos_irq_irqSetCfg_comp1pUvErr_mask(void);
void test_pos_irq_irqSetCfg_comp1pOvErr_mask(void);
void test_pos_irq_irqSetCfg_comp1nUvErr_mask(void);
void test_pos_irq_irqSetCfg_comp1nOvErr_mask(void);
void test_pos_irq_irqSetCfg_comp2pUvErr_mask(void);
void test_pos_irq_irqSetCfg_comp2pOvErr_mask(void);
void test_pos_irq_irqSetCfg_comp2nUvErr_mask(void);
void test_pos_irq_irqSetCfg_comp2nOvErr_mask(void);
void test_pos_irq_irqSetCfg_configValue(void);

/* Negative tests */
void test_neg_irq_irqSetCfg_nullParam_handle(void);
void test_neg_irq_irqSetCfg_nullParam_irqCfg(void);
void test_neg_irq_irqSetCfg_invalidParam_irqNum(void);
void test_neg_irq_irqSetCfg_nonMaskableIrq(void);
void test_neg_irq_irqSetCfg_nonConfigurableIrq_ABIST_ERR(void);
void test_neg_irq_irqSetCfg_invalidConfig_cfgRegCrcErr(void);
void test_neg_irq_irqSetCfg_invalidConfig_comp1pUvErr(void);
void test_neg_irq_irqSetCfg_invalidConfig_comp2nOvErr(void);
void test_neg_irq_irqSetCfg_invalidConfig_otherIrq(void);
void test_neg_irq_irqSetCfg_maskNonMaskable_OFF_INT_EVT_ERR_INT(void);
void test_neg_irq_irqSetCfg_maskNonMaskable_OFF_PROT_EVT_INT(void);
void test_neg_irq_irqSetCfg_maskNonMaskable_FIRST_PWR_ON_INT(void);
void test_neg_irq_irqSetCfg_maskNonMaskable_CLK_ERR_INT(void);
void test_neg_irq_irqSetCfg_maskNonMaskable_INTERNAL_OV_INT(void);
void test_neg_irq_irqSetCfg_maskNonMaskable_INIT_AN_TMO_INT(void);
void test_neg_irq_irqSetCfg_maskNonMaskable_WD_TMO_INT(void);
void test_neg_irq_irqSetCfg_maskNonMaskable_WD_TRIG_EARLY_INT(void);
void test_neg_irq_irqSetCfg_maskNonMaskable_ESM_ERR_INT(void);
void test_neg_irq_irqSetCfg_invalidIrqNum_mask(void);
void test_neg_irq_irqSetCfg_invalidIrqNum_viaMask(void);
void test_neg_irq_irqSetCfg_invalidIrqNum_config(void);

/* ========================================================================== */
/*                       irqGetCfg API Tests                                  */
/* ========================================================================== */

/* Negative tests (positive tests combined with irqSetCfg) */
void test_neg_irq_irqGetCfg_nullParam_handle(void);
void test_neg_irq_irqGetCfg_nullParam_irqCfg(void);
void test_neg_irq_irqGetCfg_invalidParam_irqNum(void);
void test_neg_irq_irqGetCfg_nonConfigurableIrq(void);

/* ========================================================================== */
/*                      irqSetCfgs API Tests                                  */
/* ========================================================================== */

/* Positive tests */
void test_pos_irq_irqSetCfgs_multipleMasks(void);
void test_pos_irq_irqSetCfgs_batchConfigValues(void);

/* Negative tests */
void test_neg_irq_irqSetCfgs_nullParam_handle(void);
void test_neg_irq_irqSetCfgs_nullParam_irqCfg(void);
void test_neg_irq_irqSetCfgs_invalidParam_numIrqs(void);
void test_neg_irq_irqSetCfgs_invalidConfigInBatch(void);

/* ========================================================================== */
/*                      irqGetCfgs API Tests                                  */
/* ========================================================================== */

/* Negative tests (positive tests combined with irqSetCfgs) */
void test_neg_irq_irqGetCfgs_nullParam_handle(void);
void test_neg_irq_irqGetCfgs_nullParam_irqCfgs(void);
void test_neg_irq_irqGetCfgs_invalidParam_numIrqs(void);
void test_neg_irq_irqGetCfgs_nonConfigurableInBatch(void);

/* ========================================================================== */
/*                     irqGetStatus API Tests                                 */
/* ========================================================================== */

/* Positive tests */
void test_pos_irq_irqGetStatus_allIrqs(void);

/* Negative tests */
void test_neg_irq_irqGetStatus_nullParam_handle(void);
void test_neg_irq_irqGetStatus_nullParam_irqStat(void);

/* ========================================================================== */
/*                    irqGetNextFlag API Tests                                */
/* ========================================================================== */

/* Positive tests */
void test_pos_irq_irqGetNextFlag_singleFlag(void);
void test_pos_irq_irqGetNextFlag_multipleFlagsSet(void);

/* Negative tests */
void test_neg_irq_irqGetNextFlag_nullParam_irqStat(void);
void test_neg_irq_irqGetNextFlag_nullParam_irqNum(void);

/* ========================================================================== */
/*                      irqGetFlag API Tests                                  */
/* ========================================================================== */

/* Positive tests */
void test_pos_irq_irqGetFlag_andClrFlag(void);

/* Negative tests */
void test_neg_irq_irqGetFlag_nullParam_handle(void);
void test_neg_irq_irqGetFlag_nullParam_flag(void);
void test_neg_irq_irqGetFlag_invalidParam_irqNum(void);

/* ========================================================================== */
/*                      irqClrFlag API Tests                                  */
/* ========================================================================== */

/* Positive tests */
void test_pos_irq_irqClrFlag_offStateStat1Register(void);
void test_pos_irq_irqClrFlag_offStateStat2Register(void);

/* Negative tests */
void test_neg_irq_irqClrFlag_nullParam_handle(void);
void test_neg_irq_irqClrFlag_invalidParam_irqNum(void);

/* ========================================================================== */
/*                    irqClrAllFlags API Tests                                */
/* ========================================================================== */

/* Positive tests */
void test_pos_irq_irqClrAllFlags_basic(void);
void test_pos_irq_irqClrAllFlags_devErrStatPreservation(void);
void test_pos_irq_irqClrAllFlags_ioFailureMidLoop(void);

/* Negative tests */
void test_neg_irq_irqClrAllFlags_nullParam_handle(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* IRQ_TEST_H */
