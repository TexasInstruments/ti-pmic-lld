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
/*                          Function Declarations                             */
/* ========================================================================== */

void irq_test(void *args);

void test_negative_Pmic_irqSetMask_nullParam_handle(void);
void test_negative_Pmic_irqSetMask_outOfBounds_irqNum(void);
void test_negative_Pmic_irqSetMasks_nullParam_handle(void);
void test_negative_Pmic_irqSetMasks_nullParam_irqMasks(void);
void test_negative_Pmic_irqSetMasks_outOfBounds_numIrqMasks(void);
void test_negative_Pmic_irqSetMasks_outOfBounds_irqNum(void);
void test_negative_Pmic_irqGetMask_nullParam_handle(void);
void test_negative_Pmic_irqGetMask_nullParam_irqMasks(void);
void test_negative_Pmic_irqGetMask_outOfBounds_numIrqMasks(void);
void test_negative_Pmic_irqGetMask_outOfBounds_irqNum(void);
void test_negative_Pmic_irqGetStatus_nullParam_handle(void);
void test_negative_Pmic_irqGetStatus_nullParam_irqStat(void);
void test_negative_Pmic_irqGetNextFlag_nullParam_irqStat(void);
void test_negative_Pmic_irqGetNextFlag_nullParam_irqNum(void);
void test_negative_Pmic_irqGetFlag_nullParam_handle(void);
void test_negative_Pmic_irqGetFlag_nullParam_flag(void);
void test_negative_Pmic_irqGetFlag_outOfBounds_irqNum(void);
void test_negative_Pmic_irqClrFlag_nullParam_handle(void);
void test_negative_Pmic_irqClrFlag_outOfBounds_irqNum(void);
void test_negative_Pmic_irqClrAllFlags_nullParam_handle(void);
void test_negative_irqSetGetMask_BUCK1_SC_NMI(void);
void test_negative_irqSetGetMask_BUCK2_SC_NMI(void);
void test_negative_irqSetGetMask_BUCK3_SC_NMI(void);
void test_negative_irqSetGetMask_LDO_LS1_VMON1_SC_NMI(void);
void test_negative_irqSetGetMask_LS2_VMON2_SC_NMI(void);
void test_negative_irqSetGetMask_TSD_ORD_NMI(void);
void test_negative_irqSetGetMask_RECOV_CNT_NMI(void);
void test_negative_irqSetGetMask_TSD_IMM_NMI(void);
void test_negative_irqSetGetMask_VCCA_OVP_NMI(void);
void test_negative_irqSetGetMask_WDG_FIRST_NOK_NMI(void);
void test_negative_irqSetGetMask_WDG_LONGWIN_TIMEOUT_NMI(void);
void test_negative_irqSetGetMask_WDG_TIMEOUT_NMI(void);
void test_negative_irqSetGetMask_WDG_ANSWER_EARLY_NMI(void);
void test_negative_irqSetGetMask_WDG_SEQ_ERR_NMI(void);
void test_negative_irqSetGetMask_WDG_ANSWER_ERR_NMI(void);
void test_negative_irqSetGetMask_WDG_FAIL_NMI(void);
void test_negative_irqSetGetMask_WDG_RST_NMI(void);
void test_negative_irqSetGetMask_REGULATOR_ERR_NMI(void);
void test_negative_irqSetGetMask_FSM_IMM_SHUTDOWN_NMI(void);
void test_negative_irqSetGetMask_FSM_ORD_SHUTDOWN_NMI(void);
void test_negative_irqSetGetMask_FSM_WARM_RESET_NMI(void);
void test_positive_irqClrAllFlags(void);
void test_positive_irqSetGetMask_BUCK1_OV_INT(void);
void test_positive_irqSetGetMask_BUCK1_UV_INT(void);
void test_positive_irqSetGetMask_BUCK1_RV_INT(void);
void test_positive_irqSetGetMask_BUCK1_ILIM_INT(void);
void test_positive_irqSetGetMask_BUCK2_OV_INT(void);
void test_positive_irqSetGetMask_BUCK2_UV_INT(void);
void test_positive_irqSetGetMask_BUCK2_RV_INT(void);
void test_positive_irqSetGetMask_BUCK2_ILIM_INT(void);
void test_positive_irqSetGetMask_BUCK3_OV_INT(void);
void test_positive_irqSetGetMask_BUCK3_UV_INT(void);
void test_positive_irqSetGetMask_BUCK3_RV_INT(void);
void test_positive_irqSetGetMask_BUCK3_ILIM_INT(void);
void test_positive_irqSetGetMask_LDO_LS1_VMON1_OV_INT(void);
void test_positive_irqSetGetMask_LDO_LS1_VMON1_UV_INT(void);
void test_positive_irqSetGetMask_LDO_LS1_VMON1_RV_INT(void);
void test_positive_irqSetGetMask_LDO_LS1_VMON1_ILIM_INT(void);
void test_positive_irqSetGetMask_LS2_VMON2_OV_INT(void);
void test_positive_irqSetGetMask_LS2_VMON2_UV_INT(void);
void test_positive_irqSetGetMask_LS2_VMON2_RV_INT(void);
void test_positive_irqSetGetMask_LS2_VMON2_ILIM_INT(void);
void test_positive_irqSetGetMask_VCCA_OV_INT(void);
void test_positive_irqSetGetMask_VCCA_UV_INT(void);
void test_positive_irqSetGetMask_STARTUP_ENABLE_INT(void);
void test_positive_irqSetGetMask_ABIST_FAIL_INT(void);
void test_positive_irqSetGetMask_BUCKS_VSET_ERR_INT(void);
void test_positive_irqSetGetMask_EXT_CLK_INT(void);
void test_positive_irqSetGetMask_TWARN_INT(void);
void test_positive_irqSetGetMask_TRIM_TEST_CRC_INT(void);
void test_positive_irqSetGetMask_CONFIG_CRC_INT(void);
void test_positive_irqSetGetMask_NINT_READBACK_INT(void);
void test_positive_irqSetGetMask_NRSTOUT_READBACK_INT(void);
void test_positive_irqSetGetMask_COMM_FRM_ERR_INT(void);
void test_positive_irqSetGetMask_COMM_CRC_ERR_INT(void);
void test_positive_irqSetGetMask_COMM_ADR_ERR_INT(void);
void test_positive_irqSetGetMask_COMM_MCU_ERR_INT(void);
void test_positive_irqSetGetMask_ESM_MCU_PIN_INT(void);
void test_positive_irqSetGetMask_ESM_MCU_FAIL_INT(void);
void test_positive_irqSetGetMask_ESM_MCU_RST_INT(void);
void test_positive_irqSetGetMask_all(void);
void test_positive_irqGetClrFlag(void);
void test_positive_irqGetStatus_noFlags(void);
void test_positive_irqGetStatus_withFlags(void);
void test_positive_irqGetNextFlag_noFlags(void);
void test_positive_irqGetNextFlag_multipleFlags(void);
void test_positive_irqGetFlag_variousIrqs(void);
void test_positive_irqClrFlag_singleFlag(void);
void test_positive_irqClrFlag_multipleSequence(void);
void test_positive_irqWorkflow_completeHandling(void);
void test_positive_irqMaskedBehavior(void);
void test_positive_irqFlagPersistence(void);
void test_positive_irqStatusReadMultipleTimes(void);
void test_positive_irqGetFlag_allMaskableIrqs(void);
void test_positive_irqClrFlag_verifyCleared(void);
void test_positive_irqIterateAndClearAll(void);
void test_positive_irqGetStatus_afterClearAll(void);

/* IRQ Hierarchy Navigation Tests */
void test_positive_irqGetStatus_trigger_L1_BUCK_LDO(void);
void test_positive_irqGetStatus_trigger_L1_LS2_VMON2(void);
void test_positive_irqGetStatus_trigger_L1_VCCA(void);
void test_positive_irqGetStatus_trigger_L1_STARTUP(void);
void test_positive_irqGetStatus_trigger_L1_MISC(void);
void test_positive_irqGetStatus_trigger_L1_MODERATE_ERR(void);
void test_positive_irqGetStatus_trigger_L1_SEVERE_ERR(void);
void test_positive_irqGetStatus_trigger_L1_FSM_ERR(void);
void test_positive_irqGetStatus_trigger_L2_BUCK1_2_via_BUCK1(void);
void test_positive_irqGetStatus_trigger_L2_BUCK1_2_via_BUCK2(void);
void test_positive_irqGetStatus_trigger_L2_BUCK3_LDO_via_BUCK3(void);
void test_positive_irqGetStatus_trigger_L2_BUCK3_LDO_via_LDO(void);
void test_positive_irqGetStatus_trigger_L2_ESM(void);
void test_positive_irqGetStatus_trigger_L2_COMM_ERR(void);
void test_positive_irqGetStatus_trigger_L2_WD_ERR_STAT(void);
void test_positive_irqGetStatus_full_hierarchy_cascade(void);
void test_positive_irqGetNextFlag_L2_populated_intrStat(void);
void test_positive_irqGetNextFlag_mixed_L1_L2_flags(void);
void test_positive_irqGetNextFlag_highIndex_IRQs(void);
void test_positive_irqGetStatus_L1_set_but_L2_empty(void);
void test_positive_irqGetStatus_all_L0_categories_set(void);
void test_positive_irqGetStatus_multiple_L2_same_category(void);

/* LP8772x-Q1 tests for uncovered lines in pmic_irq.c */
void test_positive_irq_getNextFlag_noFlagsFound(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /*__IRQ_TEST_H__*/
