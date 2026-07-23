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
#ifndef PMIC_TEST_IRQ_H
#define PMIC_TEST_IRQ_H

/* ========================================================================== */
/*                              Include Files                                 */
/* ========================================================================== */

#include "test_utils.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/* ======================================================================== */
/*       Test APIs: irqSetMask, irqSetMasks, irqGetMask, irqGetMasks        */
/* ======================================================================== */
#define IRQ_TEST_POS_IRQSETGETMASK() \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetGetMask_BUCK2_OVP_INT); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetGetMask_BUCK2_UV_INT); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetGetMask_BUCK2_OV_INT); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetGetMask_BUCK1_OVP_INT); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetGetMask_BUCK1_UV_INT); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetGetMask_BUCK1_OV_INT); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetGetMask_LDO_OVP_INT); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetGetMask_LDO_UV_INT); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetGetMask_LDO_OV_INT); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetGetMask_BUCK3_OVP_INT); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetGetMask_BUCK3_UV_INT); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetGetMask_BUCK3_OV_INT); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetGetMask_TWARN_INT); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetGetMask_B1_PVIN_UVLO_INT); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetGetMask_BUCKS_VSET_ERR_INT); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetGetMask_ABIST_FAIL_INT); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetGetMask_ABIST_DONE_INT); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetGetMask_GPO_READBACK_INT); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetGetMask_NINT_READBACK_INT); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetGetMask_CONFIG_CRC_INT); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetGetMask_TRIM_TEST_CRC_INT); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetGetMask_MCU_COMM_ERR_INT); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetGetMask_COMM_ADR_ERR_INT); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetGetMask_COMM_CRC_ERR_INT); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetGetMask_ESM_MCU_RST_INT); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetGetMask_ESM_MCU_FAIL_INT); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetGetMask_ESM_MCU_PIN_INT); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetGetMask_all); \
    PLATFORM_RUN_TEST(test_neg_irq_irqSetMasks_earlyExitOnError); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetMasks_totalProcessedEarlyExit); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetMasks_anyMasksForReg_earlyExit); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetMasks_handleRecordsForReg_anyMasksFalse); \
    PLATFORM_RUN_TEST(test_pos_irq_irqSetMasks_handleRecordsForReg_anyMasksFalse_lateReg); \
    PLATFORM_RUN_TEST(test_pos_irq_irqHandleRecordsForReg_successfulRmw)

#define IRQ_TEST_NEG_IRQSETGETMASK() \
    PLATFORM_RUN_TEST(test_neg_irq_irqSetMask_nullParam_handle); \
    PLATFORM_RUN_TEST(test_neg_irq_irqSetMask_outOfBounds_irqNum); \
    PLATFORM_RUN_TEST(test_neg_irq_irqSetMasks_nullParam_handle); \
    PLATFORM_RUN_TEST(test_neg_irq_irqSetMasks_nullParam_irqMasks); \
    PLATFORM_RUN_TEST(test_neg_irq_irqSetMasks_outOfBounds_irqNum); \
    PLATFORM_RUN_TEST(test_neg_irq_irqGetMask_nullParam_handle); \
    PLATFORM_RUN_TEST(test_neg_irq_irqGetMask_nullParam_irqMasks); \
    PLATFORM_RUN_TEST(test_neg_irq_irqGetMask_outOfBounds_irqNum); \
    PLATFORM_RUN_TEST(test_neg_irq_irqSetGetMask_LDO_SC_NMI); \
    PLATFORM_RUN_TEST(test_neg_irq_irqSetGetMask_BUCK3_SC_NMI); \
    PLATFORM_RUN_TEST(test_neg_irq_irqSetGetMask_BUCK2_SC_NMI); \
    PLATFORM_RUN_TEST(test_neg_irq_irqSetGetMask_BUCK1_SC_NMI); \
    PLATFORM_RUN_TEST(test_neg_irq_irqSetGetMask_CFG_NVM_VERIFY_ERR_NMI); \
    PLATFORM_RUN_TEST(test_neg_irq_irqSetGetMask_CFG_NVM_VERIFY_DONE_NMI); \
    PLATFORM_RUN_TEST(test_neg_irq_irqSetGetMask_CFG_NVM_PRG_DONE_NMI); \
    PLATFORM_RUN_TEST(test_neg_irq_irqSetGetMask_RECOV_CNT_NMI); \
    PLATFORM_RUN_TEST(test_neg_irq_irqSetGetMask_TSD_IMM_NMI); \
    PLATFORM_RUN_TEST(test_neg_irq_irqSetGetMask_WD_FIRST_NOK_NMI); \
    PLATFORM_RUN_TEST(test_neg_irq_irqSetGetMask_WAIT_FOR_PWRCYCLE_NMI); \
    PLATFORM_RUN_TEST(test_neg_irq_irqSetGetMask_WARM_RESET_NMI); \
    PLATFORM_RUN_TEST(test_neg_irq_irqSetGetMask_ORD_SHUTDOWN_NMI); \
    PLATFORM_RUN_TEST(test_neg_irq_irqSetGetMask_IMM_SHUTDOWN_NMI); \
    PLATFORM_RUN_TEST(test_neg_irq_irqSetGetMask_WD_RST_NMI); \
    PLATFORM_RUN_TEST(test_neg_irq_irqSetGetMask_WD_FAIL_NMI); \
    PLATFORM_RUN_TEST(test_neg_irq_irqSetGetMask_WD_LONGWIN_TIMEOUT_NMI); \
    PLATFORM_RUN_TEST(test_neg_irq_irqSetMasks_numMasks_exceeds_max); \
    PLATFORM_RUN_TEST(test_neg_irq_irqGetMask_numMasks_exceeds_max); \
    PLATFORM_RUN_TEST(test_neg_irq_irqSetMasks_handleRecordsForReg_ioReadFail); \
    PLATFORM_RUN_TEST(test_neg_irq_irqHandleRecordsForReg_ioReadFail_zeroProcessed); \
    PLATFORM_RUN_TEST(test_pos_irq_irqGetMask_zeroNumMasks)

/* Test: TC-IRQ-0025 */
#define IRQ_TEST_IRQSETGETMASK() \
    IRQ_TEST_POS_IRQSETGETMASK(); \
    IRQ_TEST_NEG_IRQSETGETMASK()

/* ======================================================================== */
/*                         Test APIs: irqGetStatus                          */
/* ======================================================================== */
#define IRQ_TEST_POS_IRQGETSTATUS() \
    PLATFORM_RUN_TEST(test_pos_irq_irqGetStatus_noFlags); \
    PLATFORM_RUN_TEST(test_pos_irq_irqGetStatus_singleFlag_L0); \
    PLATFORM_RUN_TEST(test_pos_irq_irqGetStatus_singleFlag_L1); \
    PLATFORM_RUN_TEST(test_pos_irq_irqGetStatus_singleFlag_L2); \
    PLATFORM_RUN_TEST(test_pos_irq_irqGetStatus_multipleFlags_sameReg); \
    PLATFORM_RUN_TEST(test_pos_irq_irqGetStatus_multipleFlags_diffRegs); \
    PLATFORM_RUN_TEST(test_pos_irq_irqGetStatus_hierarchyChain); \
    PLATFORM_RUN_TEST(test_pos_irq_irqGetStatus_intrStatBitMapping); \
    PLATFORM_RUN_TEST(test_pos_irq_irqGetStatus_trigger_L1_BUCK_LDO); \
    PLATFORM_RUN_TEST(test_pos_irq_irqGetStatus_trigger_L2_BUCK1_2); \
    PLATFORM_RUN_TEST(test_pos_irq_irqGetStatus_trigger_L2_BUCK1_2_viaBuck2Int); \
    PLATFORM_RUN_TEST(test_pos_irq_irqGetStatus_trigger_L2_BUCK3_LDO); \
    PLATFORM_RUN_TEST(test_pos_irq_irqGetStatus_trigger_L1_MISC); \
    PLATFORM_RUN_TEST(test_pos_irq_irqGetStatus_trigger_L1_MODERATE_ERR); \
    PLATFORM_RUN_TEST(test_pos_irq_irqGetStatus_trigger_L1_SEVERE_ERR); \
    PLATFORM_RUN_TEST(test_pos_irq_irqGetStatus_trigger_L1_FSM_ERR); \
    PLATFORM_RUN_TEST(test_pos_irq_irqGetStatus_trigger_L2_WD_ERR_STATUS); \
    PLATFORM_RUN_TEST(test_pos_irq_irqGetStatus_trigger_L2_COMM_ERR); \
    PLATFORM_RUN_TEST(test_pos_irq_irqGetStatus_trigger_L2_ESM); \
    PLATFORM_RUN_TEST(test_pos_irq_irqGetStatus_full_hierarchy_cascade); \
    PLATFORM_RUN_TEST(test_pos_irq_irqGetStatus_all_L2_interrupts); \
    PLATFORM_RUN_TEST(test_pos_irq_irqGetStatus_buckLdo_onlyBuck1_2_noBuck3Ldo); \
    PLATFORM_RUN_TEST(test_pos_irq_irqGetStatus_buckLdo_onlyLdo_noBuck1_2); \
    PLATFORM_RUN_TEST(test_pos_irq_irqReadL1IntBuckLdo_onlyLdoInt_skipBuck1_2)

#define IRQ_TEST_NEG_IRQGETSTATUS() \
    PLATFORM_RUN_TEST(test_neg_irq_irqGetStatus_nullParam_pmicHandle); \
    PLATFORM_RUN_TEST(test_neg_irq_irqGetStatus_nullParam_irqStat); \
    PLATFORM_RUN_TEST(test_neg_irq_irqGetStatus_ioFailOnL0Read); \
    PLATFORM_RUN_TEST(test_neg_irq_irqGetStatus_ioFailOnL1BuckLdoRead); \
    PLATFORM_RUN_TEST(test_neg_irq_irqGetStatus_ioFailOnL1MiscRead); \
    PLATFORM_RUN_TEST(test_neg_irq_irqGetStatus_ioFailOnL1ModerateErrRead); \
    PLATFORM_RUN_TEST(test_neg_irq_irqGetStatus_ioFailOnL1SevereErrRead); \
    PLATFORM_RUN_TEST(test_neg_irq_irqGetStatus_ioFailOnL1FsmErrRead); \
    PLATFORM_RUN_TEST(test_neg_irq_irqGetStatus_ioFailOnL2WdRead); \
    PLATFORM_RUN_TEST(test_neg_irq_irqGetStatus_ioFailOnL2CommErrRead); \
    PLATFORM_RUN_TEST(test_neg_irq_irqGetStatus_ioFailOnL2EsmRead); \
    PLATFORM_RUN_TEST(test_neg_irq_irqGetStatus_ioFailOnL2Buck3LdoRead); \
    PLATFORM_RUN_TEST(test_neg_irq_irqGetStatus_ioFailOnL2Buck1_2Read); \
    PLATFORM_RUN_TEST(test_neg_irq_irqGetStatus_ioFailOnL1BuckLdo_skipBuck1_2); \
    PLATFORM_RUN_TEST(test_neg_irq_irqReadL1IntBuckLdo_ioReadFail); \
    PLATFORM_RUN_TEST(test_neg_irq_irqReadL1IntMisc_ioReadFail); \
    PLATFORM_RUN_TEST(test_neg_irq_irqReadL1IntModerateErr_ioReadFail); \
    PLATFORM_RUN_TEST(test_neg_irq_irqReadL1IntSevereErr_ioReadFail)

/* Test: TC-IRQ-0026 */
#define IRQ_TEST_IRQGETSTATUS() \
    IRQ_TEST_POS_IRQGETSTATUS(); \
    IRQ_TEST_NEG_IRQGETSTATUS()

/* ======================================================================== */
/*                        Test APIs: irqGetNextFlag                         */
/* ======================================================================== */
#define IRQ_TEST_POS_IRQGETNEXTFLAG() \
    PLATFORM_RUN_TEST(test_pos_irq_irqGetNextFlag_singleFlag); \
    PLATFORM_RUN_TEST(test_pos_irq_irqGetNextFlag_multipleFlags); \
    PLATFORM_RUN_TEST(test_pos_irq_irqGetNextFlag_clears_intrStat); \
    PLATFORM_RUN_TEST(test_pos_irq_irqGetNextFlag_highIndexIRQ); \
    PLATFORM_RUN_TEST(test_pos_irq_irqGetNextFlag_emptyIntrStat); \
    PLATFORM_RUN_TEST(test_pos_irq_irqGetNextFlag_mixed_L1_L2); \
    PLATFORM_RUN_TEST(test_pos_irq_IRQ_getNextFlag_outerLoopAdvancesToSecondElement); \
    PLATFORM_RUN_TEST(test_pos_irq_IRQ_getNextFlag_innerLoopIteratesMultipleBitPositions); \
    PLATFORM_RUN_TEST(test_pos_irq_IRQ_getNextFlag_foundFlagGatesOuterBreakOnSkippedElement)

#define IRQ_TEST_NEG_IRQGETNEXTFLAG() \
    PLATFORM_RUN_TEST(test_neg_irq_irqGetNextFlag_nullParam_irqStat); \
    PLATFORM_RUN_TEST(test_neg_irq_irqGetNextFlag_nullParam_irqNum)

/* Test: TC-IRQ-0027 */
#define IRQ_TEST_IRQGETNEXTFLAG() \
    IRQ_TEST_POS_IRQGETNEXTFLAG(); \
    IRQ_TEST_NEG_IRQGETNEXTFLAG()

/* ======================================================================== */
/*                          Test APIs: irqGetFlag                           */
/* ======================================================================== */
#define IRQ_TEST_POS_IRQGETFLAG() \
    PLATFORM_RUN_TEST(test_pos_irq_irqGetFlag_flagSet); \
    PLATFORM_RUN_TEST(test_pos_irq_irqGetFlag_flagClear)

#define IRQ_TEST_NEG_IRQGETFLAG() \
    PLATFORM_RUN_TEST(test_neg_irq_irqGetFlag_nullParam_pmicHandle); \
    PLATFORM_RUN_TEST(test_neg_irq_irqGetFlag_outOfBounds_irqNum); \
    PLATFORM_RUN_TEST(test_neg_irq_irqGetFlag_nullParam_flag)

/* Test: TC-IRQ-0028 */
#define IRQ_TEST_IRQGETFLAG() \
    IRQ_TEST_POS_IRQGETFLAG(); \
    IRQ_TEST_NEG_IRQGETFLAG()

/* ======================================================================== */
/*                  Test APIs: irqClrAllFlags, irqClrFlag                   */
/* ======================================================================== */
#define IRQ_TEST_POS_IRQCLRFLAG() \
    PLATFORM_RUN_TEST(test_pos_irq_irqClrAllFlags); \
    PLATFORM_RUN_TEST(test_pos_irq_irqClrFlag_singleFlag); \
    PLATFORM_RUN_TEST(test_pos_irq_irqClrFlag_preserveOthers)

#define IRQ_TEST_NEG_IRQCLRFLAG() \
    PLATFORM_RUN_TEST(test_neg_irq_irqClrFlag_nullParam_pmicHandle); \
    PLATFORM_RUN_TEST(test_neg_irq_irqClrFlag_outOfBounds_irqNum); \
    PLATFORM_RUN_TEST(test_neg_irq_irqClrAllFlags_nullParam_handle)

/* Test: TC-IRQ-0029 */
#define IRQ_TEST_IRQCLRFLAG() \
    IRQ_TEST_POS_IRQCLRFLAG(); \
    IRQ_TEST_NEG_IRQCLRFLAG()

/* ======================================================================== */
/*   Test APIs: Pmic_irqClrAllFlags, Pmic_irqSetMask, Pmic_irqGetStatus,    */
/*              Pmic_irqGetNextFlag, Pmic_irqClrFlag                        */
/* ======================================================================== */

/* Test: TC-IRQ-0030 */
#define IRQ_TEST_IRQINTEGRATION() \
    PLATFORM_RUN_TEST(test_pos_irq_irqFullCycle_setMask_getStatus_iterate_clear); \
    PLATFORM_RUN_TEST(test_pos_irq_irqMultipleSimultaneous_allRegisters)

/* ======================================================================== */
/*                  Test APIs: Pmic_irqResponseCallback                     */
/* ======================================================================== */

#define IRQ_TEST_POS_IRQRESPONSECALLBACK() \
    PLATFORM_RUN_TEST(test_pos_irq_irqResponseCallback_callbackInvoked); \
    PLATFORM_RUN_TEST(test_pos_irq_irqResponseCallback_nullCallback); \
    PLATFORM_RUN_TEST(test_pos_irq_irqResponseCallback_nullHandle)

/* Test: TC-IRQ-0033 */
#define IRQ_TEST_IRQRESPONSECALLBACK() \
    IRQ_TEST_POS_IRQRESPONSECALLBACK()

/* ========================================================================== */
/*                         Aggregate Test Macros                              */
/* ========================================================================== */

/* Run all positive IRQ tests */
#define IRQ_TEST_RUN_POSITIVE() \
    PLATFORM_RUN_TEST(test_pos_irq_irqClrAllFlags); \
    IRQ_TEST_POS_IRQSETGETMASK(); \
    IRQ_TEST_POS_IRQGETSTATUS(); \
    IRQ_TEST_POS_IRQGETNEXTFLAG(); \
    IRQ_TEST_POS_IRQGETFLAG(); \
    PLATFORM_RUN_TEST(test_pos_irq_irqClrFlag_singleFlag); \
    PLATFORM_RUN_TEST(test_pos_irq_irqClrFlag_preserveOthers); \
    IRQ_TEST_IRQINTEGRATION(); \
    IRQ_TEST_POS_IRQRESPONSECALLBACK()

/* Run all negative IRQ tests */
#define IRQ_TEST_RUN_NEGATIVE() \
    IRQ_TEST_NEG_IRQSETGETMASK(); \
    IRQ_TEST_NEG_IRQGETSTATUS(); \
    IRQ_TEST_NEG_IRQGETNEXTFLAG(); \
    IRQ_TEST_NEG_IRQGETFLAG(); \
    IRQ_TEST_NEG_IRQCLRFLAG()

/* Run all IRQ tests */
#define IRQ_TEST_RUN_ALL() \
    PLATFORM_RUN_TEST(test_pos_irq_irqClrAllFlags); \
    IRQ_TEST_IRQSETGETMASK(); \
    IRQ_TEST_IRQGETSTATUS(); \
    IRQ_TEST_IRQGETNEXTFLAG(); \
    IRQ_TEST_IRQGETFLAG(); \
    IRQ_TEST_NEG_IRQCLRFLAG(); \
    PLATFORM_RUN_TEST(test_pos_irq_irqClrFlag_singleFlag); \
    PLATFORM_RUN_TEST(test_pos_irq_irqClrFlag_preserveOthers); \
    IRQ_TEST_IRQINTEGRATION(); \
    IRQ_TEST_IRQRESPONSECALLBACK()

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

void irq_test(void *args);

/* ========================================================================== */
/*             irqSetMask / irqSetMasks / irqGetMask API Tests                */
/* ========================================================================== */

/* Positive tests */
void test_pos_irq_irqSetGetMask_BUCK2_OVP_INT(void);
void test_pos_irq_irqSetGetMask_BUCK2_UV_INT(void);
void test_pos_irq_irqSetGetMask_BUCK2_OV_INT(void);
void test_pos_irq_irqSetGetMask_BUCK1_OVP_INT(void);
void test_pos_irq_irqSetGetMask_BUCK1_UV_INT(void);
void test_pos_irq_irqSetGetMask_BUCK1_OV_INT(void);
void test_pos_irq_irqSetGetMask_LDO_OVP_INT(void);
void test_pos_irq_irqSetGetMask_LDO_UV_INT(void);
void test_pos_irq_irqSetGetMask_LDO_OV_INT(void);
void test_pos_irq_irqSetGetMask_BUCK3_OVP_INT(void);
void test_pos_irq_irqSetGetMask_BUCK3_UV_INT(void);
void test_pos_irq_irqSetGetMask_BUCK3_OV_INT(void);
void test_pos_irq_irqSetGetMask_TWARN_INT(void);
void test_pos_irq_irqSetGetMask_B1_PVIN_UVLO_INT(void);
void test_pos_irq_irqSetGetMask_BUCKS_VSET_ERR_INT(void);
void test_pos_irq_irqSetGetMask_ABIST_FAIL_INT(void);
void test_pos_irq_irqSetGetMask_ABIST_DONE_INT(void);
void test_pos_irq_irqSetGetMask_GPO_READBACK_INT(void);
void test_pos_irq_irqSetGetMask_NINT_READBACK_INT(void);
void test_pos_irq_irqSetGetMask_CONFIG_CRC_INT(void);
void test_pos_irq_irqSetGetMask_TRIM_TEST_CRC_INT(void);
void test_pos_irq_irqSetGetMask_MCU_COMM_ERR_INT(void);
void test_pos_irq_irqSetGetMask_COMM_ADR_ERR_INT(void);
void test_pos_irq_irqSetGetMask_COMM_CRC_ERR_INT(void);
void test_pos_irq_irqSetGetMask_ESM_MCU_RST_INT(void);
void test_pos_irq_irqSetGetMask_ESM_MCU_FAIL_INT(void);
void test_pos_irq_irqSetGetMask_ESM_MCU_PIN_INT(void);
void test_pos_irq_irqSetGetMask_all(void);
void test_neg_irq_irqSetMasks_earlyExitOnError(void);
void test_pos_irq_irqSetMasks_totalProcessedEarlyExit(void);
void test_pos_irq_irqSetMasks_anyMasksForReg_earlyExit(void);
void test_pos_irq_irqSetMasks_handleRecordsForReg_anyMasksFalse(void);
void test_pos_irq_irqSetMasks_handleRecordsForReg_anyMasksFalse_lateReg(void);

/* Negative tests */
void test_neg_irq_irqSetMask_nullParam_handle(void);
void test_neg_irq_irqSetMask_outOfBounds_irqNum(void);
void test_neg_irq_irqSetMasks_nullParam_handle(void);
void test_neg_irq_irqSetMasks_nullParam_irqMasks(void);
void test_neg_irq_irqSetMasks_outOfBounds_irqNum(void);
void test_neg_irq_irqGetMask_nullParam_handle(void);
void test_neg_irq_irqGetMask_nullParam_irqMasks(void);
void test_neg_irq_irqGetMask_outOfBounds_irqNum(void);
void test_neg_irq_irqSetGetMask_LDO_SC_NMI(void);
void test_neg_irq_irqSetGetMask_BUCK3_SC_NMI(void);
void test_neg_irq_irqSetGetMask_BUCK2_SC_NMI(void);
void test_neg_irq_irqSetGetMask_BUCK1_SC_NMI(void);
void test_neg_irq_irqSetGetMask_CFG_NVM_VERIFY_ERR_NMI(void);
void test_neg_irq_irqSetGetMask_CFG_NVM_VERIFY_DONE_NMI(void);
void test_neg_irq_irqSetGetMask_CFG_NVM_PRG_DONE_NMI(void);
void test_neg_irq_irqSetGetMask_RECOV_CNT_NMI(void);
void test_neg_irq_irqSetGetMask_TSD_IMM_NMI(void);
void test_neg_irq_irqSetGetMask_WD_FIRST_NOK_NMI(void);
void test_neg_irq_irqSetGetMask_WAIT_FOR_PWRCYCLE_NMI(void);
void test_neg_irq_irqSetGetMask_WARM_RESET_NMI(void);
void test_neg_irq_irqSetGetMask_ORD_SHUTDOWN_NMI(void);
void test_neg_irq_irqSetGetMask_IMM_SHUTDOWN_NMI(void);
void test_neg_irq_irqSetGetMask_WD_RST_NMI(void);
void test_neg_irq_irqSetGetMask_WD_FAIL_NMI(void);
void test_neg_irq_irqSetGetMask_WD_LONGWIN_TIMEOUT_NMI(void);
void test_neg_irq_irqSetMasks_numMasks_exceeds_max(void);
void test_neg_irq_irqGetMask_numMasks_exceeds_max(void);
void test_neg_irq_irqSetMasks_handleRecordsForReg_ioReadFail(void);
void test_neg_irq_irqHandleRecordsForReg_ioReadFail_zeroProcessed(void);
void test_pos_irq_irqHandleRecordsForReg_successfulRmw(void);
void test_pos_irq_irqGetMask_zeroNumMasks(void);

/* ========================================================================== */
/*                      irqGetStatus API Tests                                */
/* ========================================================================== */

/* Positive tests */
void test_pos_irq_irqGetStatus_noFlags(void);
void test_pos_irq_irqGetStatus_singleFlag_L0(void);
void test_pos_irq_irqGetStatus_singleFlag_L1(void);
void test_pos_irq_irqGetStatus_singleFlag_L2(void);
void test_pos_irq_irqGetStatus_multipleFlags_sameReg(void);
void test_pos_irq_irqGetStatus_multipleFlags_diffRegs(void);
void test_pos_irq_irqGetStatus_hierarchyChain(void);
void test_pos_irq_irqGetStatus_intrStatBitMapping(void);
void test_pos_irq_irqGetStatus_trigger_L1_BUCK_LDO(void);
void test_pos_irq_irqGetStatus_trigger_L2_BUCK1_2(void);
void test_pos_irq_irqGetStatus_trigger_L2_BUCK1_2_viaBuck2Int(void);
void test_pos_irq_irqGetStatus_trigger_L2_BUCK3_LDO(void);
void test_pos_irq_irqGetStatus_trigger_L1_MISC(void);
void test_pos_irq_irqGetStatus_trigger_L1_MODERATE_ERR(void);
void test_pos_irq_irqGetStatus_trigger_L1_SEVERE_ERR(void);
void test_pos_irq_irqGetStatus_trigger_L1_FSM_ERR(void);
void test_pos_irq_irqGetStatus_trigger_L2_WD_ERR_STATUS(void);
void test_pos_irq_irqGetStatus_trigger_L2_COMM_ERR(void);
void test_pos_irq_irqGetStatus_trigger_L2_ESM(void);
void test_pos_irq_irqGetStatus_full_hierarchy_cascade(void);
void test_pos_irq_irqGetStatus_all_L2_interrupts(void);
void test_pos_irq_irqGetStatus_buckLdo_onlyBuck1_2_noBuck3Ldo(void);
void test_pos_irq_irqGetStatus_buckLdo_onlyLdo_noBuck1_2(void);

/* Negative tests */
void test_neg_irq_irqGetStatus_nullParam_pmicHandle(void);
void test_neg_irq_irqGetStatus_nullParam_irqStat(void);

/* Static branch coverage tests: IO failure paths in IRQ read functions */
void test_neg_irq_irqGetStatus_ioFailOnL0Read(void);
void test_neg_irq_irqGetStatus_ioFailOnL1BuckLdoRead(void);
void test_neg_irq_irqGetStatus_ioFailOnL1MiscRead(void);
void test_neg_irq_irqGetStatus_ioFailOnL1ModerateErrRead(void);
void test_neg_irq_irqGetStatus_ioFailOnL1SevereErrRead(void);
void test_neg_irq_irqGetStatus_ioFailOnL1FsmErrRead(void);
void test_neg_irq_irqGetStatus_ioFailOnL2WdRead(void);
void test_neg_irq_irqGetStatus_ioFailOnL2CommErrRead(void);
void test_neg_irq_irqGetStatus_ioFailOnL2EsmRead(void);
void test_neg_irq_irqGetStatus_ioFailOnL2Buck3LdoRead(void);
void test_neg_irq_irqGetStatus_ioFailOnL2Buck1_2Read(void);
void test_neg_irq_irqGetStatus_ioFailOnL1BuckLdo_skipBuck1_2(void);
void test_neg_irq_irqReadL1IntBuckLdo_ioReadFail(void);
void test_pos_irq_irqReadL1IntBuckLdo_onlyLdoInt_skipBuck1_2(void);
void test_neg_irq_irqReadL1IntMisc_ioReadFail(void);
void test_neg_irq_irqReadL1IntModerateErr_ioReadFail(void);
void test_neg_irq_irqReadL1IntSevereErr_ioReadFail(void);

/* ========================================================================== */
/*                     irqGetNextFlag API Tests                               */
/* ========================================================================== */

/* Positive tests */
void test_pos_irq_irqGetNextFlag_singleFlag(void);
void test_pos_irq_irqGetNextFlag_multipleFlags(void);
void test_pos_irq_irqGetNextFlag_clears_intrStat(void);
void test_pos_irq_irqGetNextFlag_highIndexIRQ(void);
void test_pos_irq_irqGetNextFlag_emptyIntrStat(void);
void test_pos_irq_irqGetNextFlag_mixed_L1_L2(void);
void test_pos_irq_IRQ_getNextFlag_outerLoopAdvancesToSecondElement(void);
void test_pos_irq_IRQ_getNextFlag_innerLoopIteratesMultipleBitPositions(void);
void test_pos_irq_IRQ_getNextFlag_foundFlagGatesOuterBreakOnSkippedElement(void);

/* Negative tests */
void test_neg_irq_irqGetNextFlag_nullParam_irqStat(void);
void test_neg_irq_irqGetNextFlag_nullParam_irqNum(void);

/* ========================================================================== */
/*                       irqGetFlag API Tests                                 */
/* ========================================================================== */

/* Positive tests */
void test_pos_irq_irqGetFlag_flagSet(void);
void test_pos_irq_irqGetFlag_flagClear(void);

/* Negative tests */
void test_neg_irq_irqGetFlag_nullParam_pmicHandle(void);
void test_neg_irq_irqGetFlag_outOfBounds_irqNum(void);
void test_neg_irq_irqGetFlag_nullParam_flag(void);

/* ========================================================================== */
/*                 irqClrFlag / irqClrAllFlags API Tests                      */
/* ========================================================================== */

/* Positive tests */
void test_pos_irq_irqClrAllFlags(void);
void test_pos_irq_irqClrFlag_singleFlag(void);
void test_pos_irq_irqClrFlag_preserveOthers(void);

/* Negative tests */
void test_neg_irq_irqClrFlag_nullParam_pmicHandle(void);
void test_neg_irq_irqClrFlag_outOfBounds_irqNum(void);
void test_neg_irq_irqClrAllFlags_nullParam_handle(void);

/* ========================================================================== */
/*                       Integration Tests                                    */
/* ========================================================================== */

void test_pos_irq_irqFullCycle_setMask_getStatus_iterate_clear(void);
void test_pos_irq_irqMultipleSimultaneous_allRegisters(void);

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
#endif /* PMIC_TEST_IRQ_H */
