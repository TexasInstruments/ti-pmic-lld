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
 *    documentation and/or other materials provided with the
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
/*                              Include Files                                 */
/* ========================================================================== */

#include "platform.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/* ======================================================================== */
/*                          Test APIs: irqSetMask                           */
/* ======================================================================== */

#define IRQ_TEST_NEG_IRQSETMASK() \
    PLATFORM_RUN_TEST(test_neg_irq_irqSetMask_nullParam_handle); \
    PLATFORM_RUN_TEST(test_neg_irq_irqSetMask_outOfBounds_irqNum)

/* Test: TC-IRQ-0015 */
#define IRQ_TEST_IRQSETMASK() \
    IRQ_TEST_NEG_IRQSETMASK()

/* ======================================================================== */
/*                          Test APIs: irqSetMasks                          */
/* ======================================================================== */

#define IRQ_TEST_NEG_IRQSETMASKS() \
    PLATFORM_RUN_TEST(test_neg_irq_irqSetMasks_nullParam_handle); \
    PLATFORM_RUN_TEST(test_neg_irq_irqSetMasks_nullParam_irqMasks); \
    PLATFORM_RUN_TEST(test_neg_irq_irqSetMasks_outOfBounds_numIrqMasks); \
    PLATFORM_RUN_TEST(test_neg_irq_irqSetMasks_outOfBounds_irqNum)

/* Test: TC-IRQ-0016 */
#define IRQ_TEST_IRQSETMASKS() \
    IRQ_TEST_NEG_IRQSETMASKS()

/* ======================================================================== */
/*                          Test APIs: irqGetMask                           */
/* ======================================================================== */

#define IRQ_TEST_NEG_IRQGETMASK() \
    PLATFORM_RUN_TEST(test_neg_irq_irqGetMask_nullParam_handle); \
    PLATFORM_RUN_TEST(test_neg_irq_irqGetMask_nullParam_irqMasks); \
    PLATFORM_RUN_TEST(test_neg_irq_irqGetMask_outOfBounds_numIrqMasks); \
    PLATFORM_RUN_TEST(test_neg_irq_irqGetMask_outOfBounds_irqNum)

/* Test: TC-IRQ-0017 */
#define IRQ_TEST_IRQGETMASK() \
    IRQ_TEST_NEG_IRQGETMASK()

/* ======================================================================== */
/*               Test APIs: Pmic_irqSetMask, Pmic_irqGetMask                */
/* ======================================================================== */

#define IRQ_TEST_POS_IRQSETGETMASK() \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetGetMask_BUCK1_OV_INT); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetGetMask_BUCK1_UV_INT); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetGetMask_BUCK1_RV_INT); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetGetMask_BUCK1_ILIM_INT); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetGetMask_BUCK2_OV_INT); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetGetMask_BUCK2_UV_INT); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetGetMask_BUCK2_RV_INT); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetGetMask_BUCK2_ILIM_INT); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetGetMask_BUCK3_OV_INT); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetGetMask_BUCK3_UV_INT); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetGetMask_BUCK3_RV_INT); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetGetMask_BUCK3_ILIM_INT); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetGetMask_LDO_LS1_VMON1_OV_INT); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetGetMask_LDO_LS1_VMON1_UV_INT); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetGetMask_LDO_LS1_VMON1_RV_INT); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetGetMask_LDO_LS1_VMON1_ILIM_INT); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetGetMask_LS2_VMON2_OV_INT); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetGetMask_LS2_VMON2_UV_INT); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetGetMask_LS2_VMON2_RV_INT); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetGetMask_LS2_VMON2_ILIM_INT); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetGetMask_VCCA_OV_INT); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetGetMask_VCCA_UV_INT); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetGetMask_STARTUP_ENABLE_INT); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetGetMask_ABIST_FAIL_INT); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetGetMask_BUCKS_VSET_ERR_INT); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetGetMask_EXT_CLK_INT); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetGetMask_TWARN_INT); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetGetMask_TRIM_TEST_CRC_INT); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetGetMask_CONFIG_CRC_INT); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetGetMask_NINT_READBACK_INT); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetGetMask_NRSTOUT_READBACK_INT); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetGetMask_COMM_FRM_ERR_INT); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetGetMask_COMM_CRC_ERR_INT); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetGetMask_COMM_ADR_ERR_INT); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetGetMask_COMM_MCU_ERR_INT); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetGetMask_ESM_MCU_PIN_INT); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetGetMask_ESM_MCU_FAIL_INT); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetGetMask_ESM_MCU_RST_INT); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetGetMask_all)

#define IRQ_TEST_NEG_IRQSETGETMASK() \
    PLATFORM_RUN_TEST(test_neg_irq_irqSetGetMask_BUCK1_SC_NMI); \
    PLATFORM_RUN_TEST(test_neg_irq_irqSetGetMask_BUCK2_SC_NMI); \
    PLATFORM_RUN_TEST(test_neg_irq_irqSetGetMask_BUCK3_SC_NMI); \
    PLATFORM_RUN_TEST(test_neg_irq_irqSetGetMask_LDO_LS1_VMON1_SC_NMI); \
    PLATFORM_RUN_TEST(test_neg_irq_irqSetGetMask_LS2_VMON2_SC_NMI); \
    PLATFORM_RUN_TEST(test_neg_irq_irqSetGetMask_TSD_ORD_NMI); \
    PLATFORM_RUN_TEST(test_neg_irq_irqSetGetMask_RECOV_CNT_NMI); \
    PLATFORM_RUN_TEST(test_neg_irq_irqSetGetMask_TSD_IMM_NMI); \
    PLATFORM_RUN_TEST(test_neg_irq_irqSetGetMask_VCCA_OVP_NMI); \
    PLATFORM_RUN_TEST(test_neg_irq_irqSetGetMask_WDG_FIRST_NOK_NMI); \
    PLATFORM_RUN_TEST(test_neg_irq_irqSetGetMask_WDG_LONGWIN_TIMEOUT_NMI); \
    PLATFORM_RUN_TEST(test_neg_irq_irqSetGetMask_WDG_TIMEOUT_NMI); \
    PLATFORM_RUN_TEST(test_neg_irq_irqSetGetMask_WDG_ANSWER_EARLY_NMI); \
    PLATFORM_RUN_TEST(test_neg_irq_irqSetGetMask_WDG_SEQ_ERR_NMI); \
    PLATFORM_RUN_TEST(test_neg_irq_irqSetGetMask_WDG_ANSWER_ERR_NMI); \
    PLATFORM_RUN_TEST(test_neg_irq_irqSetGetMask_WDG_FAIL_NMI); \
    PLATFORM_RUN_TEST(test_neg_irq_irqSetGetMask_WDG_RST_NMI); \
    PLATFORM_RUN_TEST(test_neg_irq_irqSetGetMask_REGULATOR_ERR_NMI); \
    PLATFORM_RUN_TEST(test_neg_irq_irqSetGetMask_FSM_IMM_SHUTDOWN_NMI); \
    PLATFORM_RUN_TEST(test_neg_irq_irqSetGetMask_FSM_ORD_SHUTDOWN_NMI); \
    PLATFORM_RUN_TEST(test_neg_irq_irqSetGetMask_FSM_WARM_RESET_NMI)

/* Test: TC-IRQ-0018 */
#define IRQ_TEST_IRQSETGETMASK() \
    IRQ_TEST_POS_IRQSETGETMASK(); \
    IRQ_TEST_NEG_IRQSETGETMASK()

/* ======================================================================== */
/*                         Test APIs: irqGetStatus                          */
/* ======================================================================== */

#define IRQ_TEST_POS_IRQGETSTATUS() \
    PLATFORM_RUN_TEST(test_pos_irq_irqGetStatus_noFlags); \
    PLATFORM_RUN_TEST(test_pos_irq_irqGetStatus_withFlags); \
    PLATFORM_RUN_TEST(test_pos_irq_irqGetStatus_afterClearAll); \
    PLATFORM_RUN_TEST(test_pos_irq_irqGetStatus_trigger_L1_BUCK_LDO); \
    PLATFORM_RUN_TEST(test_pos_irq_irqGetStatus_trigger_L1_LS2_VMON2); \
    PLATFORM_RUN_TEST(test_pos_irq_irqGetStatus_trigger_L1_VCCA); \
    PLATFORM_RUN_TEST(test_pos_irq_irqGetStatus_trigger_L1_STARTUP); \
    PLATFORM_RUN_TEST(test_pos_irq_irqGetStatus_trigger_L1_MISC); \
    PLATFORM_RUN_TEST(test_pos_irq_irqGetStatus_trigger_L1_MODERATE_ERR); \
    PLATFORM_RUN_TEST(test_pos_irq_irqGetStatus_trigger_L1_SEVERE_ERR); \
    PLATFORM_RUN_TEST(test_pos_irq_irqGetStatus_trigger_L1_FSM_ERR); \
    PLATFORM_RUN_TEST(test_pos_irq_irqGetStatus_trigger_L2_BUCK1_2_via_BUCK1); \
    PLATFORM_RUN_TEST(test_pos_irq_irqGetStatus_trigger_L2_BUCK1_2_via_BUCK2); \
    PLATFORM_RUN_TEST(test_pos_irq_irqGetStatus_trigger_L2_BUCK3_LDO_via_BUCK3); \
    PLATFORM_RUN_TEST(test_pos_irq_irqGetStatus_trigger_L2_BUCK3_LDO_via_LDO); \
    PLATFORM_RUN_TEST(test_pos_irq_irqGetStatus_trigger_L2_ESM); \
    PLATFORM_RUN_TEST(test_pos_irq_irqGetStatus_trigger_L2_COMM_ERR); \
    PLATFORM_RUN_TEST(test_pos_irq_irqGetStatus_trigger_L2_WD_ERR_STAT); \
    PLATFORM_RUN_TEST(test_pos_irq_irqGetStatus_full_hierarchy_cascade); \
    PLATFORM_RUN_TEST(test_pos_irq_irqGetStatus_L1_set_but_L2_empty); \
    PLATFORM_RUN_TEST(test_pos_irq_irqGetStatus_all_L0_categories_set); \
    PLATFORM_RUN_TEST(test_pos_irq_irqGetStatus_multiple_L2_same_category)

#define IRQ_TEST_NEG_IRQGETSTATUS() \
    PLATFORM_RUN_TEST(test_neg_irq_irqGetStatus_nullParam_handle); \
    PLATFORM_RUN_TEST(test_neg_irq_irqGetStatus_nullParam_irqStat)

/* Test: TC-IRQ-0019 */
#define IRQ_TEST_IRQGETSTATUS() \
    IRQ_TEST_POS_IRQGETSTATUS(); \
    IRQ_TEST_NEG_IRQGETSTATUS()

/* ======================================================================== */
/*                        Test APIs: irqGetNextFlag                         */
/* ======================================================================== */

#define IRQ_TEST_POS_IRQGETNEXTFLAG() \
    PLATFORM_RUN_TEST(test_pos_irq_irqGetNextFlag_noFlags); \
    PLATFORM_RUN_TEST(test_pos_irq_irqGetNextFlag_multipleFlags); \
    PLATFORM_RUN_TEST(test_pos_irq_irqGetNextFlag_L2_populated_intrStat); \
    PLATFORM_RUN_TEST(test_pos_irq_irqGetNextFlag_mixed_L1_L2_flags); \
    PLATFORM_RUN_TEST(test_pos_irq_irqGetNextFlag_highIndex_IRQs); \
    PLATFORM_RUN_TEST(test_pos_irq_irqGetNextFlag_noFlagsFound)

#define IRQ_TEST_NEG_IRQGETNEXTFLAG() \
    PLATFORM_RUN_TEST(test_neg_irq_irqGetNextFlag_nullParam_irqStat); \
    PLATFORM_RUN_TEST(test_neg_irq_irqGetNextFlag_nullParam_irqNum)

/* Test: TC-IRQ-0020 */
#define IRQ_TEST_IRQGETNEXTFLAG() \
    IRQ_TEST_POS_IRQGETNEXTFLAG(); \
    IRQ_TEST_NEG_IRQGETNEXTFLAG()

/* ======================================================================== */
/*                          Test APIs: irqGetFlag                           */
/* ======================================================================== */

#define IRQ_TEST_POS_IRQGETFLAG() \
    PLATFORM_RUN_TEST(test_pos_irq_irqGetFlag_variousIrqs); \
    PLATFORM_RUN_TEST(test_pos_irq_irqGetFlag_allMaskableIrqs); \
    PLATFORM_RUN_TEST(test_pos_irq_irqGetClrFlag)

#define IRQ_TEST_NEG_IRQGETFLAG() \
    PLATFORM_RUN_TEST(test_neg_irq_irqGetFlag_nullParam_handle); \
    PLATFORM_RUN_TEST(test_neg_irq_irqGetFlag_nullParam_flag); \
    PLATFORM_RUN_TEST(test_neg_irq_irqGetFlag_outOfBounds_irqNum)

/* Test: TC-IRQ-0021 */
#define IRQ_TEST_IRQGETFLAG() \
    IRQ_TEST_POS_IRQGETFLAG(); \
    IRQ_TEST_NEG_IRQGETFLAG()

/* ======================================================================== */
/*                          Test APIs: irqClrFlag                           */
/* ======================================================================== */

#define IRQ_TEST_POS_IRQCLRFLAG() \
    PLATFORM_RUN_TEST(test_pos_irq_irqClrFlag_singleFlag); \
    PLATFORM_RUN_TEST(test_pos_irq_irqClrFlag_multipleSequence); \
    PLATFORM_RUN_TEST(test_pos_irq_irqClrFlag_verifyCleared)

#define IRQ_TEST_NEG_IRQCLRFLAG() \
    PLATFORM_RUN_TEST(test_neg_irq_irqClrFlag_nullParam_handle); \
    PLATFORM_RUN_TEST(test_neg_irq_irqClrFlag_outOfBounds_irqNum)

/* Test: TC-IRQ-0022 */
#define IRQ_TEST_IRQCLRFLAG() \
    IRQ_TEST_POS_IRQCLRFLAG(); \
    IRQ_TEST_NEG_IRQCLRFLAG()

/* ======================================================================== */
/*                        Test APIs: irqClrAllFlags                         */
/* ======================================================================== */

#define IRQ_TEST_POS_IRQCLRALLFLAGS() \
    PLATFORM_RUN_TEST(test_pos_irq_irqClrAllFlags_basic)

#define IRQ_TEST_NEG_IRQCLRALLFLAGS() \
    PLATFORM_RUN_TEST(test_neg_irq_irqClrAllFlags_nullParam_handle)

/* Test: TC-IRQ-0023 */
#define IRQ_TEST_IRQCLRALLFLAGS() \
    IRQ_TEST_POS_IRQCLRALLFLAGS(); \
    IRQ_TEST_NEG_IRQCLRALLFLAGS()

/* ======================================================================== */
/*   Test APIs: Pmic_irqGetStatus, Pmic_irqGetNextFlag, Pmic_irqGetFlag,    */
/*              Pmic_irqClrFlag                                             */
/* ======================================================================== */

#define IRQ_TEST_POS_INTEGRATION() \
    PLATFORM_RUN_TEST(test_pos_irq_irqWorkflow_completeHandling); \
    PLATFORM_RUN_TEST(test_pos_irq_irqMaskedBehavior); \
    PLATFORM_RUN_TEST(test_pos_irq_irqFlagPersistence); \
    PLATFORM_RUN_TEST(test_pos_irq_irqStatusReadMultipleTimes); \
    PLATFORM_RUN_TEST(test_pos_irq_irqIterateAndClearAll)

/* Test: TC-IRQ-0024 */
#define IRQ_TEST_INTEGRATION() \
    IRQ_TEST_POS_INTEGRATION()

/* ======================================================================== */
/*                  Test APIs: Pmic_irqResponseCallback                     */
/* ======================================================================== */

#define IRQ_TEST_POS_IRQRESPONSECALLBACK() \
    PLATFORM_RUN_TEST(test_pos_irq_irqResponseCallback_callbackInvoked); \
    PLATFORM_RUN_TEST(test_pos_irq_irqResponseCallback_nullCallback); \
    PLATFORM_RUN_TEST(test_pos_irq_irqResponseCallback_nullHandle)

/* Test: TC-IRQ-0031 */
#define IRQ_TEST_IRQRESPONSECALLBACK() \
    IRQ_TEST_POS_IRQRESPONSECALLBACK()

/* ========================================================================== */
/*                         Aggregate Test Runners                             */
/* ========================================================================== */

#define IRQ_TEST_RUN_POSITIVE() \
    IRQ_TEST_POS_IRQCLRALLFLAGS(); \
    IRQ_TEST_POS_IRQSETGETMASK(); \
    IRQ_TEST_POS_IRQGETSTATUS(); \
    IRQ_TEST_POS_IRQGETNEXTFLAG(); \
    IRQ_TEST_POS_IRQGETFLAG(); \
    IRQ_TEST_POS_IRQCLRFLAG(); \
    IRQ_TEST_POS_INTEGRATION(); \
    IRQ_TEST_POS_IRQRESPONSECALLBACK()

#define IRQ_TEST_RUN_NEGATIVE() \
    IRQ_TEST_NEG_IRQSETMASK(); \
    IRQ_TEST_NEG_IRQSETMASKS(); \
    IRQ_TEST_NEG_IRQGETMASK(); \
    IRQ_TEST_NEG_IRQSETGETMASK(); \
    IRQ_TEST_NEG_IRQGETSTATUS(); \
    IRQ_TEST_NEG_IRQGETNEXTFLAG(); \
    IRQ_TEST_NEG_IRQGETFLAG(); \
    IRQ_TEST_NEG_IRQCLRFLAG(); \
    IRQ_TEST_NEG_IRQCLRALLFLAGS()

#define IRQ_TEST_RUN_ALL() \
    IRQ_TEST_RUN_POSITIVE(); \
    IRQ_TEST_RUN_NEGATIVE();

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

void irq_test(void *args);

/* ========================================================================== */
/*                       irqSetMask API Tests                                 */
/* ========================================================================== */

/* Negative tests */
void test_neg_irq_irqSetMask_nullParam_handle(void);
void test_neg_irq_irqSetMask_outOfBounds_irqNum(void);

/* ========================================================================== */
/*                       irqSetMasks API Tests                                */
/* ========================================================================== */

/* Negative tests */
void test_neg_irq_irqSetMasks_nullParam_handle(void);
void test_neg_irq_irqSetMasks_nullParam_irqMasks(void);
void test_neg_irq_irqSetMasks_outOfBounds_numIrqMasks(void);
void test_neg_irq_irqSetMasks_outOfBounds_irqNum(void);

/* ========================================================================== */
/*                       irqGetMask API Tests                                 */
/* ========================================================================== */

/* Negative tests */
void test_neg_irq_irqGetMask_nullParam_handle(void);
void test_neg_irq_irqGetMask_nullParam_irqMasks(void);
void test_neg_irq_irqGetMask_outOfBounds_numIrqMasks(void);
void test_neg_irq_irqGetMask_outOfBounds_irqNum(void);

/* ========================================================================== */
/*                  irqSetGetMask API Tests (Combined)                        */
/* ========================================================================== */

/* Positive tests */
void test_pos_irq_irqSetGetMask_BUCK1_OV_INT(void);
void test_pos_irq_irqSetGetMask_BUCK1_UV_INT(void);
void test_pos_irq_irqSetGetMask_BUCK1_RV_INT(void);
void test_pos_irq_irqSetGetMask_BUCK1_ILIM_INT(void);
void test_pos_irq_irqSetGetMask_BUCK2_OV_INT(void);
void test_pos_irq_irqSetGetMask_BUCK2_UV_INT(void);
void test_pos_irq_irqSetGetMask_BUCK2_RV_INT(void);
void test_pos_irq_irqSetGetMask_BUCK2_ILIM_INT(void);
void test_pos_irq_irqSetGetMask_BUCK3_OV_INT(void);
void test_pos_irq_irqSetGetMask_BUCK3_UV_INT(void);
void test_pos_irq_irqSetGetMask_BUCK3_RV_INT(void);
void test_pos_irq_irqSetGetMask_BUCK3_ILIM_INT(void);
void test_pos_irq_irqSetGetMask_LDO_LS1_VMON1_OV_INT(void);
void test_pos_irq_irqSetGetMask_LDO_LS1_VMON1_UV_INT(void);
void test_pos_irq_irqSetGetMask_LDO_LS1_VMON1_RV_INT(void);
void test_pos_irq_irqSetGetMask_LDO_LS1_VMON1_ILIM_INT(void);
void test_pos_irq_irqSetGetMask_LS2_VMON2_OV_INT(void);
void test_pos_irq_irqSetGetMask_LS2_VMON2_UV_INT(void);
void test_pos_irq_irqSetGetMask_LS2_VMON2_RV_INT(void);
void test_pos_irq_irqSetGetMask_LS2_VMON2_ILIM_INT(void);
void test_pos_irq_irqSetGetMask_VCCA_OV_INT(void);
void test_pos_irq_irqSetGetMask_VCCA_UV_INT(void);
void test_pos_irq_irqSetGetMask_STARTUP_ENABLE_INT(void);
void test_pos_irq_irqSetGetMask_ABIST_FAIL_INT(void);
void test_pos_irq_irqSetGetMask_BUCKS_VSET_ERR_INT(void);
void test_pos_irq_irqSetGetMask_EXT_CLK_INT(void);
void test_pos_irq_irqSetGetMask_TWARN_INT(void);
void test_pos_irq_irqSetGetMask_TRIM_TEST_CRC_INT(void);
void test_pos_irq_irqSetGetMask_CONFIG_CRC_INT(void);
void test_pos_irq_irqSetGetMask_NINT_READBACK_INT(void);
void test_pos_irq_irqSetGetMask_NRSTOUT_READBACK_INT(void);
void test_pos_irq_irqSetGetMask_COMM_FRM_ERR_INT(void);
void test_pos_irq_irqSetGetMask_COMM_CRC_ERR_INT(void);
void test_pos_irq_irqSetGetMask_COMM_ADR_ERR_INT(void);
void test_pos_irq_irqSetGetMask_COMM_MCU_ERR_INT(void);
void test_pos_irq_irqSetGetMask_ESM_MCU_PIN_INT(void);
void test_pos_irq_irqSetGetMask_ESM_MCU_FAIL_INT(void);
void test_pos_irq_irqSetGetMask_ESM_MCU_RST_INT(void);
void test_pos_irq_irqSetGetMask_all(void);

/* Negative tests - Non-maskable IRQs */
void test_neg_irq_irqSetGetMask_BUCK1_SC_NMI(void);
void test_neg_irq_irqSetGetMask_BUCK2_SC_NMI(void);
void test_neg_irq_irqSetGetMask_BUCK3_SC_NMI(void);
void test_neg_irq_irqSetGetMask_LDO_LS1_VMON1_SC_NMI(void);
void test_neg_irq_irqSetGetMask_LS2_VMON2_SC_NMI(void);
void test_neg_irq_irqSetGetMask_TSD_ORD_NMI(void);
void test_neg_irq_irqSetGetMask_RECOV_CNT_NMI(void);
void test_neg_irq_irqSetGetMask_TSD_IMM_NMI(void);
void test_neg_irq_irqSetGetMask_VCCA_OVP_NMI(void);
void test_neg_irq_irqSetGetMask_WDG_FIRST_NOK_NMI(void);
void test_neg_irq_irqSetGetMask_WDG_LONGWIN_TIMEOUT_NMI(void);
void test_neg_irq_irqSetGetMask_WDG_TIMEOUT_NMI(void);
void test_neg_irq_irqSetGetMask_WDG_ANSWER_EARLY_NMI(void);
void test_neg_irq_irqSetGetMask_WDG_SEQ_ERR_NMI(void);
void test_neg_irq_irqSetGetMask_WDG_ANSWER_ERR_NMI(void);
void test_neg_irq_irqSetGetMask_WDG_FAIL_NMI(void);
void test_neg_irq_irqSetGetMask_WDG_RST_NMI(void);
void test_neg_irq_irqSetGetMask_REGULATOR_ERR_NMI(void);
void test_neg_irq_irqSetGetMask_FSM_IMM_SHUTDOWN_NMI(void);
void test_neg_irq_irqSetGetMask_FSM_ORD_SHUTDOWN_NMI(void);
void test_neg_irq_irqSetGetMask_FSM_WARM_RESET_NMI(void);

/* ========================================================================== */
/*                      irqGetStatus API Tests                                */
/* ========================================================================== */

/* Positive tests */
void test_pos_irq_irqGetStatus_noFlags(void);
void test_pos_irq_irqGetStatus_withFlags(void);
void test_pos_irq_irqGetStatus_afterClearAll(void);
void test_pos_irq_irqGetStatus_trigger_L1_BUCK_LDO(void);
void test_pos_irq_irqGetStatus_trigger_L1_LS2_VMON2(void);
void test_pos_irq_irqGetStatus_trigger_L1_VCCA(void);
void test_pos_irq_irqGetStatus_trigger_L1_STARTUP(void);
void test_pos_irq_irqGetStatus_trigger_L1_MISC(void);
void test_pos_irq_irqGetStatus_trigger_L1_MODERATE_ERR(void);
void test_pos_irq_irqGetStatus_trigger_L1_SEVERE_ERR(void);
void test_pos_irq_irqGetStatus_trigger_L1_FSM_ERR(void);
void test_pos_irq_irqGetStatus_trigger_L2_BUCK1_2_via_BUCK1(void);
void test_pos_irq_irqGetStatus_trigger_L2_BUCK1_2_via_BUCK2(void);
void test_pos_irq_irqGetStatus_trigger_L2_BUCK3_LDO_via_BUCK3(void);
void test_pos_irq_irqGetStatus_trigger_L2_BUCK3_LDO_via_LDO(void);
void test_pos_irq_irqGetStatus_trigger_L2_ESM(void);
void test_pos_irq_irqGetStatus_trigger_L2_COMM_ERR(void);
void test_pos_irq_irqGetStatus_trigger_L2_WD_ERR_STAT(void);
void test_pos_irq_irqGetStatus_full_hierarchy_cascade(void);
void test_pos_irq_irqGetStatus_L1_set_but_L2_empty(void);
void test_pos_irq_irqGetStatus_all_L0_categories_set(void);
void test_pos_irq_irqGetStatus_multiple_L2_same_category(void);

/* Negative tests */
void test_neg_irq_irqGetStatus_nullParam_handle(void);
void test_neg_irq_irqGetStatus_nullParam_irqStat(void);

/* ========================================================================== */
/*                     irqGetNextFlag API Tests                               */
/* ========================================================================== */

/* Positive tests */
void test_pos_irq_irqGetNextFlag_noFlags(void);
void test_pos_irq_irqGetNextFlag_multipleFlags(void);
void test_pos_irq_irqGetNextFlag_L2_populated_intrStat(void);
void test_pos_irq_irqGetNextFlag_mixed_L1_L2_flags(void);
void test_pos_irq_irqGetNextFlag_highIndex_IRQs(void);
void test_pos_irq_irqGetNextFlag_noFlagsFound(void);

/* Negative tests */
void test_neg_irq_irqGetNextFlag_nullParam_irqStat(void);
void test_neg_irq_irqGetNextFlag_nullParam_irqNum(void);

/* ========================================================================== */
/*                       irqGetFlag API Tests                                 */
/* ========================================================================== */

/* Positive tests */
void test_pos_irq_irqGetFlag_variousIrqs(void);
void test_pos_irq_irqGetFlag_allMaskableIrqs(void);
void test_pos_irq_irqGetClrFlag(void);

/* Negative tests */
void test_neg_irq_irqGetFlag_nullParam_handle(void);
void test_neg_irq_irqGetFlag_nullParam_flag(void);
void test_neg_irq_irqGetFlag_outOfBounds_irqNum(void);

/* ========================================================================== */
/*                       irqClrFlag API Tests                                 */
/* ========================================================================== */

/* Positive tests */
void test_pos_irq_irqClrFlag_singleFlag(void);
void test_pos_irq_irqClrFlag_multipleSequence(void);
void test_pos_irq_irqClrFlag_verifyCleared(void);

/* Negative tests */
void test_neg_irq_irqClrFlag_nullParam_handle(void);
void test_neg_irq_irqClrFlag_outOfBounds_irqNum(void);

/* ========================================================================== */
/*                     irqClrAllFlags API Tests                               */
/* ========================================================================== */

/* Positive tests */
void test_pos_irq_irqClrAllFlags_basic(void);

/* Negative tests */
void test_neg_irq_irqClrAllFlags_nullParam_handle(void);

/* ========================================================================== */
/*                     Integration & Workflow Tests                           */
/* ========================================================================== */

/* Positive tests */
void test_pos_irq_irqWorkflow_completeHandling(void);
void test_pos_irq_irqMaskedBehavior(void);
void test_pos_irq_irqFlagPersistence(void);
void test_pos_irq_irqStatusReadMultipleTimes(void);
void test_pos_irq_irqIterateAndClearAll(void);

/* ========================================================================== */
/*                  irqResponseCallback API Tests                             */
/* ========================================================================== */

/* Positive tests */
void test_pos_irq_irqResponseCallback_callbackInvoked(void);
void test_pos_irq_irqResponseCallback_nullCallback(void);
void test_pos_irq_irqResponseCallback_nullHandle(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /*__IRQ_TEST_H__*/
