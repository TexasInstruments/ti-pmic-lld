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
#ifndef PMIC_TEST_IRQ_H
#define PMIC_TEST_IRQ_H



/* ========================================================================== */
/*                              Include Files                                 */
/* ========================================================================== */

#include "test_common.h"

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
void test_negative_Pmic_irqSetMasks_outOfBounds_irqNum(void);
void test_negative_Pmic_irqGetMask_nullParam_handle(void);
void test_negative_Pmic_irqGetMask_nullParam_irqMasks(void);
void test_negative_Pmic_irqGetMask_outOfBounds_irqNum(void);
void test_negative_Pmic_irqGetStatus_nullParam_pmicHandle(void);
void test_negative_Pmic_irqGetStatus_nullParam_irqStat(void);
void test_negative_Pmic_irqGetNextFlag_nullParam_irqStat(void);
void test_negative_Pmic_irqGetNextFlag_nullParam_irqNum(void);
void test_negative_Pmic_irqGetFlag_nullParam_pmicHandle(void);
void test_negative_Pmic_irqGetFlag_outOfBounds_irqNum(void);
void test_negative_Pmic_irqGetFlag_nullParam_flag(void);
void test_negative_Pmic_irqClrFlag_nullParam_pmicHandle(void);
void test_negative_Pmic_irqClrFlag_outOfBounds_irqNum(void);
void test_negative_Pmic_irqClrAllFlags_nullParam_handle(void);
void test_negative_irqSetGetMask_LDO_SC_NMI(void);
void test_negative_irqSetGetMask_BUCK3_SC_NMI(void);
void test_negative_irqSetGetMask_BUCK2_SC_NMI(void);
void test_negative_irqSetGetMask_BUCK1_SC_NMI(void);
void test_negative_irqSetGetMask_CFG_NVM_VERIFY_ERR_NMI(void);
void test_negative_irqSetGetMask_CFG_NVM_VERIFY_DONE_NMI(void);
void test_negative_irqSetGetMask_CFG_NVM_PRG_DONE_NMI(void);
void test_negative_irqSetGetMask_RECOV_CNT_NMI(void);
void test_negative_irqSetGetMask_TSD_IMM_NMI(void);
void test_negative_irqSetGetMask_WD_FIRST_NOK_NMI(void);
void test_negative_irqSetGetMask_WAIT_FOR_PWRCYCLE_NMI(void);
void test_negative_irqSetGetMask_WARM_RESET_NMI(void);
void test_negative_irqSetGetMask_ORD_SHUTDOWN_NMI(void);
void test_negative_irqSetGetMask_IMM_SHUTDOWN_NMI(void);
void test_negative_irqSetGetMask_WD_RST_NMI(void);
void test_negative_irqSetGetMask_WD_FAIL_NMI(void);
void test_negative_irqSetGetMask_WD_LONGWIN_TIMEOUT_NMI(void);
void test_positive_irqClrAllFlags(void);
void test_positive_irqSetGetMask_BUCK2_OVP_INT(void);
void test_positive_irqSetGetMask_BUCK2_UV_INT(void);
void test_positive_irqSetGetMask_BUCK2_OV_INT(void);
void test_positive_irqSetGetMask_BUCK1_OVP_INT(void);
void test_positive_irqSetGetMask_BUCK1_UV_INT(void);
void test_positive_irqSetGetMask_BUCK1_OV_INT(void);
void test_positive_irqSetGetMask_LDO_OVP_INT(void);
void test_positive_irqSetGetMask_LDO_UV_INT(void);
void test_positive_irqSetGetMask_LDO_OV_INT(void);
void test_positive_irqSetGetMask_BUCK3_OVP_INT(void);
void test_positive_irqSetGetMask_BUCK3_UV_INT(void);
void test_positive_irqSetGetMask_BUCK3_OV_INT(void);
void test_positive_irqSetGetMask_TWARN_INT(void);
void test_positive_irqSetGetMask_B1_PVIN_UVLO_INT(void);
void test_positive_irqSetGetMask_BUCKS_VSET_ERR_INT(void);
void test_positive_irqSetGetMask_ABIST_FAIL_INT(void);
void test_positive_irqSetGetMask_ABIST_DONE_INT(void);
void test_positive_irqSetGetMask_GPO_READBACK_INT(void);
void test_positive_irqSetGetMask_NINT_READBACK_INT(void);
void test_positive_irqSetGetMask_CONFIG_CRC_INT(void);
void test_positive_irqSetGetMask_TRIM_TEST_CRC_INT(void);
void test_positive_irqSetGetMask_MCU_COMM_ERR_INT(void);
void test_positive_irqSetGetMask_COMM_ADR_ERR_INT(void);
void test_positive_irqSetGetMask_COMM_CRC_ERR_INT(void);
void test_positive_irqSetGetMask_ESM_MCU_RST_INT(void);
void test_positive_irqSetGetMask_ESM_MCU_FAIL_INT(void);
void test_positive_irqSetGetMask_ESM_MCU_PIN_INT(void);
void test_positive_irqSetGetMask_all(void);

/* Status Reading and Flag Iteration Tests */
void test_positive_irqGetStatus_noFlags(void);
void test_positive_irqGetStatus_singleFlag_L0(void);
void test_positive_irqGetStatus_singleFlag_L1(void);
void test_positive_irqGetStatus_singleFlag_L2(void);
void test_positive_irqGetStatus_multipleFlags_sameReg(void);
void test_positive_irqGetStatus_multipleFlags_diffRegs(void);
void test_positive_irqGetStatus_hierarchyChain(void);
void test_positive_irqGetStatus_intrStatBitMapping(void);
void test_positive_irqGetNextFlag_singleFlag(void);
void test_positive_irqGetNextFlag_multipleFlags(void);
void test_positive_irqGetNextFlag_clears_intrStat(void);
void test_positive_irqGetNextFlag_highIndexIRQ(void);
void test_positive_irqGetNextFlag_emptyIntrStat(void);
void test_positive_irqGetNextFlag_mixed_L1_L2(void);
void test_positive_irqGetFlag_flagSet(void);
void test_positive_irqGetFlag_flagClear(void);
void test_positive_irqClrFlag_singleFlag(void);
void test_positive_irqClrFlag_preserveOthers(void);
void test_positive_irqFullCycle_setMask_getStatus_iterate_clear(void);
void test_positive_irqMultipleSimultaneous_allRegisters(void);

/* Edge Case and L2 Hierarchy Tests */
void test_negative_Pmic_irqSetMasks_numMasks_exceeds_max(void);
void test_negative_Pmic_irqGetMask_numMasks_exceeds_max(void);
void test_positive_irqGetStatus_trigger_L1_BUCK_LDO(void);
void test_positive_irqGetStatus_trigger_L2_BUCK1_2(void);
void test_positive_irqGetStatus_trigger_L2_BUCK3_LDO(void);
void test_positive_irqGetStatus_trigger_L1_MISC(void);
void test_positive_irqGetStatus_trigger_L1_MODERATE_ERR(void);
void test_positive_irqGetStatus_trigger_L1_SEVERE_ERR(void);
void test_positive_irqGetStatus_trigger_L1_FSM_ERR(void);
void test_positive_irqGetStatus_trigger_L2_WD_ERR_STATUS(void);
void test_positive_irqGetStatus_trigger_L2_COMM_ERR(void);
void test_positive_irqGetStatus_trigger_L2_ESM(void);
void test_positive_irqGetStatus_full_hierarchy_cascade(void);
void test_positive_irqGetStatus_all_L2_interrupts(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* PMIC_TEST_IRQ_H */
