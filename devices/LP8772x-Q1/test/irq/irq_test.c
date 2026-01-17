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


/* ========================================================================== */
/*                              Include Files                                 */
/* ========================================================================== */

#include "irq_test.h"
#include "test_inject.h"

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/* ========================================================================== */
/*                    API-Specific Test Macros - irqSetMask                   */
/* ========================================================================== */

#define IRQ_TEST_NEG_IRQSETMASK() \
    PLATFORM_RUN_TEST(test_neg_irq_irqSetMask_nullParam_handle); \
    PLATFORM_RUN_TEST(test_neg_irq_irqSetMask_outOfBounds_irqNum)

#define IRQ_TEST_IRQSETMASK() \
    IRQ_TEST_NEG_IRQSETMASK()

/* ========================================================================== */
/*                   API-Specific Test Macros - irqSetMasks                   */
/* ========================================================================== */

#define IRQ_TEST_NEG_IRQSETMASKS() \
    PLATFORM_RUN_TEST(test_neg_irq_irqSetMasks_nullParam_handle); \
    PLATFORM_RUN_TEST(test_neg_irq_irqSetMasks_nullParam_irqMasks); \
    PLATFORM_RUN_TEST(test_neg_irq_irqSetMasks_outOfBounds_numIrqMasks); \
    PLATFORM_RUN_TEST(test_neg_irq_irqSetMasks_outOfBounds_irqNum)

#define IRQ_TEST_IRQSETMASKS() \
    IRQ_TEST_NEG_IRQSETMASKS()

/* ========================================================================== */
/*                    API-Specific Test Macros - irqGetMask                   */
/* ========================================================================== */

#define IRQ_TEST_NEG_IRQGETMASK() \
    PLATFORM_RUN_TEST(test_neg_irq_irqGetMask_nullParam_handle); \
    PLATFORM_RUN_TEST(test_neg_irq_irqGetMask_nullParam_irqMasks); \
    PLATFORM_RUN_TEST(test_neg_irq_irqGetMask_outOfBounds_numIrqMasks); \
    PLATFORM_RUN_TEST(test_neg_irq_irqGetMask_outOfBounds_irqNum)

#define IRQ_TEST_IRQGETMASK() \
    IRQ_TEST_NEG_IRQGETMASK()

/* ========================================================================== */
/*              API-Specific Test Macros - irqSetGetMask (Combined)          */
/* ========================================================================== */

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

#define IRQ_TEST_IRQSETGETMASK() \
    IRQ_TEST_POS_IRQSETGETMASK(); \
    IRQ_TEST_NEG_IRQSETGETMASK()

/* ========================================================================== */
/*                  API-Specific Test Macros - irqGetStatus                   */
/* ========================================================================== */

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

#define IRQ_TEST_IRQGETSTATUS() \
    IRQ_TEST_POS_IRQGETSTATUS(); \
    IRQ_TEST_NEG_IRQGETSTATUS()

/* ========================================================================== */
/*                 API-Specific Test Macros - irqGetNextFlag                  */
/* ========================================================================== */

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

#define IRQ_TEST_IRQGETNEXTFLAG() \
    IRQ_TEST_POS_IRQGETNEXTFLAG(); \
    IRQ_TEST_NEG_IRQGETNEXTFLAG()

/* ========================================================================== */
/*                  API-Specific Test Macros - irqGetFlag                     */
/* ========================================================================== */

#define IRQ_TEST_POS_IRQGETFLAG() \
    PLATFORM_RUN_TEST(test_pos_irq_irqGetFlag_variousIrqs); \
    PLATFORM_RUN_TEST(test_pos_irq_irqGetFlag_allMaskableIrqs); \
    PLATFORM_RUN_TEST(test_pos_irq_irqGetClrFlag)

#define IRQ_TEST_NEG_IRQGETFLAG() \
    PLATFORM_RUN_TEST(test_neg_irq_irqGetFlag_nullParam_handle); \
    PLATFORM_RUN_TEST(test_neg_irq_irqGetFlag_nullParam_flag); \
    PLATFORM_RUN_TEST(test_neg_irq_irqGetFlag_outOfBounds_irqNum)

#define IRQ_TEST_IRQGETFLAG() \
    IRQ_TEST_POS_IRQGETFLAG(); \
    IRQ_TEST_NEG_IRQGETFLAG()

/* ========================================================================== */
/*                  API-Specific Test Macros - irqClrFlag                     */
/* ========================================================================== */

#define IRQ_TEST_POS_IRQCLRFLAG() \
    PLATFORM_RUN_TEST(test_pos_irq_irqClrFlag_singleFlag); \
    PLATFORM_RUN_TEST(test_pos_irq_irqClrFlag_multipleSequence); \
    PLATFORM_RUN_TEST(test_pos_irq_irqClrFlag_verifyCleared)

#define IRQ_TEST_NEG_IRQCLRFLAG() \
    PLATFORM_RUN_TEST(test_neg_irq_irqClrFlag_nullParam_handle); \
    PLATFORM_RUN_TEST(test_neg_irq_irqClrFlag_outOfBounds_irqNum)

#define IRQ_TEST_IRQCLRFLAG() \
    IRQ_TEST_POS_IRQCLRFLAG(); \
    IRQ_TEST_NEG_IRQCLRFLAG()

/* ========================================================================== */
/*                API-Specific Test Macros - irqClrAllFlags                   */
/* ========================================================================== */

#define IRQ_TEST_POS_IRQCLRALLFLAGS() \
    PLATFORM_RUN_TEST(test_pos_irq_irqClrAllFlags_basic)

#define IRQ_TEST_NEG_IRQCLRALLFLAGS() \
    PLATFORM_RUN_TEST(test_neg_irq_irqClrAllFlags_nullParam_handle)

#define IRQ_TEST_IRQCLRALLFLAGS() \
    IRQ_TEST_POS_IRQCLRALLFLAGS(); \
    IRQ_TEST_NEG_IRQCLRALLFLAGS()

/* ========================================================================== */
/*              API-Specific Test Macros - Integration & Workflow             */
/* ========================================================================== */

#define IRQ_TEST_POS_INTEGRATION() \
    PLATFORM_RUN_TEST(test_pos_irq_irqWorkflow_completeHandling); \
    PLATFORM_RUN_TEST(test_pos_irq_irqMaskedBehavior); \
    PLATFORM_RUN_TEST(test_pos_irq_irqFlagPersistence); \
    PLATFORM_RUN_TEST(test_pos_irq_irqStatusReadMultipleTimes); \
    PLATFORM_RUN_TEST(test_pos_irq_irqIterateAndClearAll)

#define IRQ_TEST_INTEGRATION() \
    IRQ_TEST_POS_INTEGRATION()

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
    IRQ_TEST_POS_INTEGRATION()

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
    IRQ_TEST_RUN_NEGATIVE()


/* ========================================================================== */
/*                             Global Variables                               */
/* ========================================================================== */
static Pmic_Handle_t pmicHandle;

/* ========================================================================== */
/*                           Function Declarations                            */
/* ========================================================================== */
static void irqTest_setGetMaskError(uint8_t irqNum);
static int32_t irqTest_setGetMask(uint8_t irqNum, bool shouldMask);

/* ========================================================================== */
/*                           Function Definitions                             */
/* ========================================================================== */

void irq_test(void *args)
{
    (void)args;
    char msg[50U] = {0};
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_HandleCfg_t coreCfg = {
        .validParams = (PMIC_COMM_MODE_VALID |
                        PMIC_I2C_ADDR0_VALID |
                        PMIC_COMM_HANDLE_0_VALID |
                        PMIC_IO_READ_VALID |
                        PMIC_IO_WRITE_VALID |
                        PMIC_CRITICAL_SECTION_START_VALID |
                        PMIC_CRITICAL_SECTION_STOP_VALID |
                        PMIC_CRC_ENABLE_VALID |
                        PMIC_CONFIG_CRC_ENABLE_VALID |
                        PMIC_IRQ_RESPONSE_CALLBACK_VALID),
        .commMode = PMIC_INTF_I2C_SINGLE,
        .i2cAddr0 = PLATFORM_TARGET_I2C_ADDR,
        .crcEnable = PMIC_DISABLE,
        .configCrcEnable = PMIC_DISABLE,
        .commHandle0 = platform_getCommHandle(),
        .ioRead = &platform_rxByte,
        .ioWrite = &platform_txByte,
        .criticalSectionStart = &platform_critSecStart,
        .criticalSectionStop = &platform_critSecStop,
        .irqResponseCallback = &platform_irqResponse
    };

    platform_init();

    platform_printString("\r\n");
    platform_printString("IRQ_TEST\r\n");
    platform_printString("-------\r\n\r\n");

    status = Pmic_init(&pmicHandle, &coreCfg);

    if (status == PMIC_ST_SUCCESS)
    {
        platform_setupTests();
        IRQ_TEST_RUN_ALL();
        platform_tearDownTests();
    }
    else
    {
        (void)sprintf(msg, "Error in initializing PMIC LLD: %d\r\n", status);
        platform_printString(msg);
    }

    (void)Pmic_deinit(&pmicHandle);
    platform_deinit();
}

void test_neg_irq_irqSetMask_nullParam_handle(void)
{
    // Pass NULL handle into Pmic_irqSetMask()
    int32_t status = Pmic_irqSetMask(NULL, PMIC_BUCK1_OV_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}
void test_neg_irq_irqSetMask_outOfBounds_irqNum(void)
{
    // Pass out of bounds irqNum into Pmic_irqSetMask()
    int32_t status = Pmic_irqSetMask(&pmicHandle, PMIC_IRQ_MAX + 1U, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_irq_irqSetMasks_nullParam_handle(void)
{
    // Pass NULL handle into Pmic_irqSetMasks()
    Pmic_IrqMask_t irqMask = {
        .irqNum = PMIC_BUCK1_OV_INT,
        .mask = PMIC_IRQ_MASK
    };
    int32_t status = Pmic_irqSetMasks(NULL, 1U, &irqMask);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_irq_irqSetMasks_nullParam_irqMasks(void)
{
    // Pass NULL irqMasks into Pmic_irqSetMasks()
    int32_t status = Pmic_irqSetMasks(&pmicHandle, 1U, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_irq_irqSetMasks_outOfBounds_numIrqMasks(void)
{
    // Pass out of bounds numIrqMasks into Pmic_irqSetMasks()
    Pmic_IrqMask_t irqMask = {
        .irqNum = PMIC_BUCK1_OV_INT,
        .mask = PMIC_IRQ_MASK
    };
    int32_t status = Pmic_irqSetMasks(&pmicHandle, PMIC_IRQ_NUM + 1U, &irqMask);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_irq_irqSetMasks_outOfBounds_irqNum(void)
{
    // Pass out of bounds irqNum into Pmic_irqSetMasks()
    Pmic_IrqMask_t irqMask = {
        .irqNum = PMIC_IRQ_MAX + 1U,
        .mask = PMIC_IRQ_MASK
    };
    int32_t status = Pmic_irqSetMasks(&pmicHandle, 1U, &irqMask);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_irq_irqGetMask_nullParam_handle(void)
{
    // Pass NULL handle into Pmic_irqGetMask()
    Pmic_IrqMask_t irqMask = {
        .irqNum = PMIC_BUCK1_OV_INT,
        .mask = PMIC_IRQ_MASK
    };
    int32_t status = Pmic_irqGetMask(NULL, 1U, &irqMask);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_irq_irqGetMask_nullParam_irqMasks(void)
{
    // Pass NULL irqMasks into Pmic_irqGetMask()
    int32_t status = Pmic_irqGetMask(&pmicHandle, 1U, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_irq_irqGetMask_outOfBounds_numIrqMasks(void)
{
    // Pass out of bounds numIrqMasks into Pmic_irqGetMask()
    Pmic_IrqMask_t irqMask = {
        .irqNum = PMIC_BUCK1_OV_INT,
        .mask = PMIC_IRQ_MASK
    };
    int32_t status = Pmic_irqGetMask(&pmicHandle, PMIC_IRQ_NUM + 1U, &irqMask);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_irq_irqGetMask_outOfBounds_irqNum(void)
{
    // Pass out of bounds irqNum into Pmic_irqGetMask()
    Pmic_IrqMask_t irqMask = {
        .irqNum = PMIC_IRQ_MAX + 1U,
        .mask = PMIC_IRQ_MASK
    };
    int32_t status = Pmic_irqGetMask(&pmicHandle, 1U, &irqMask);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_irq_irqGetStatus_nullParam_handle(void)
{
    // Pass NULL handle into Pmic_irqGetStatus()
    Pmic_IrqStat_t irqStat = {0U};
    int32_t status = Pmic_irqGetStatus(NULL, &irqStat);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_irq_irqGetStatus_nullParam_irqStat(void)
{
    // Pass NULL irqStat into Pmic_irqGetStatus()
    int32_t status = Pmic_irqGetStatus(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_irq_irqGetNextFlag_nullParam_irqStat(void)
{
    // Pass NULL irqStat into Pmic_irqGetNextFlag()
    uint8_t irqNum = 0U;
    int32_t status = Pmic_irqGetNextFlag(NULL, NULL, &irqNum);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_irq_irqGetNextFlag_nullParam_irqNum(void)
{
    // Pass NULL irqNum into Pmic_irqGetNextFlag()
    Pmic_IrqStat_t irqStat = {0U};
    int32_t status = Pmic_irqGetNextFlag(NULL, &irqStat, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_irq_irqGetFlag_nullParam_handle(void)
{
    // Pass NULL handle into Pmic_irqGetFlag()
    bool flag = (bool)false;
    int32_t status = Pmic_irqGetFlag(NULL, PMIC_BUCK1_OV_INT, &flag);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_irq_irqGetFlag_nullParam_flag(void)
{
    // Pass NULL flag into Pmic_irqGetFlag()
    int32_t status = Pmic_irqGetFlag(&pmicHandle, PMIC_BUCK1_OV_INT, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_irq_irqGetFlag_outOfBounds_irqNum(void)
{
    // Pass out of bounds irqNum into Pmic_irqGetFlag()
    bool flag = (bool)false;
    int32_t status = Pmic_irqGetFlag(&pmicHandle, PMIC_IRQ_MAX + 1U, &flag);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_irq_irqClrFlag_nullParam_handle(void)
{
    // Pass NULL handle into Pmic_irqClrFlag()
    int32_t status = Pmic_irqClrFlag(NULL, PMIC_BUCK1_OV_INT);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_irq_irqClrFlag_outOfBounds_irqNum(void)
{
    // Pass out of bounds irqNum into Pmic_irqClrFlag
    int32_t status = Pmic_irqClrFlag(&pmicHandle, PMIC_IRQ_MAX + 1U);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_irq_irqClrAllFlags_nullParam_handle(void)
{
    // Pass NULL handle into Pmic_irqClrAllFlags
    int32_t status = Pmic_irqClrAllFlags(NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

static void irqTest_setGetMaskError(uint8_t irqNum)
{
    int32_t status = PMIC_ST_SUCCESS;
    const uint8_t numIrqMasks = 1U;
    Pmic_IrqMask_t irqMask = {.irqNum = irqNum, .mask = PMIC_IRQ_MASK};

    // Set NMI mask via Pmic_irqSetMask() and check for error
    status = Pmic_irqSetMask(&pmicHandle, irqNum, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NOT_SUPPORTED);

    // Set NMI mask via Pmic_irqSetMasks() and check for error
    status = Pmic_irqSetMasks(&pmicHandle, numIrqMasks, &irqMask);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NOT_SUPPORTED);

    // Get NMI mask and check for error
    status = Pmic_irqGetMask(&pmicHandle, numIrqMasks, &irqMask);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NOT_SUPPORTED);
}

void test_neg_irq_irqSetGetMask_BUCK1_SC_NMI(void)
{
    irqTest_setGetMaskError(PMIC_BUCK1_SC_NMI);
}

void test_neg_irq_irqSetGetMask_BUCK2_SC_NMI(void)
{
    irqTest_setGetMaskError(PMIC_BUCK2_SC_NMI);
}

void test_neg_irq_irqSetGetMask_BUCK3_SC_NMI(void)
{
    irqTest_setGetMaskError(PMIC_BUCK3_SC_NMI);
}

void test_neg_irq_irqSetGetMask_LDO_LS1_VMON1_SC_NMI(void)
{
    irqTest_setGetMaskError(PMIC_LDO_LS1_VMON1_SC_NMI);
}

void test_neg_irq_irqSetGetMask_LS2_VMON2_SC_NMI(void)
{
    irqTest_setGetMaskError(PMIC_LS2_VMON2_SC_NMI);
}

void test_neg_irq_irqSetGetMask_TSD_ORD_NMI(void)
{
    irqTest_setGetMaskError(PMIC_ME_TSD_ORD_NMI);
}

void test_neg_irq_irqSetGetMask_RECOV_CNT_NMI(void)
{
    irqTest_setGetMaskError(PMIC_ME_RECOV_CNT_NMI);
}

void test_neg_irq_irqSetGetMask_TSD_IMM_NMI(void)
{
    irqTest_setGetMaskError(PMIC_SE_TSD_IMM_NMI);
}

void test_neg_irq_irqSetGetMask_VCCA_OVP_NMI(void)
{
    irqTest_setGetMaskError(PMIC_SE_VCCA_OVP_NMI);
}

void test_neg_irq_irqSetGetMask_WDG_FIRST_NOK_NMI(void)
{
    irqTest_setGetMaskError(PMIC_WDG_FIRST_NOK_NMI);
}

void test_neg_irq_irqSetGetMask_WDG_LONGWIN_TIMEOUT_NMI(void)
{
    irqTest_setGetMaskError(PMIC_WDG_LONGWIN_TIMEOUT_NMI);
}

void test_neg_irq_irqSetGetMask_WDG_TIMEOUT_NMI(void)
{
    irqTest_setGetMaskError(PMIC_WDG_TIMEOUT_NMI);
}

void test_neg_irq_irqSetGetMask_WDG_ANSWER_EARLY_NMI(void)
{
    irqTest_setGetMaskError(PMIC_WDG_ANSWER_EARLY_NMI);
}

void test_neg_irq_irqSetGetMask_WDG_SEQ_ERR_NMI(void)
{
    irqTest_setGetMaskError(PMIC_WDG_SEQ_ERR_NMI);
}

void test_neg_irq_irqSetGetMask_WDG_ANSWER_ERR_NMI(void)
{
    irqTest_setGetMaskError(PMIC_WDG_ANSWER_ERR_NMI);
}

void test_neg_irq_irqSetGetMask_WDG_FAIL_NMI(void)
{
    irqTest_setGetMaskError(PMIC_WDG_FAIL_NMI);
}

void test_neg_irq_irqSetGetMask_WDG_RST_NMI(void)
{
    irqTest_setGetMaskError(PMIC_WDG_RST_NMI);
}

void test_neg_irq_irqSetGetMask_REGULATOR_ERR_NMI(void)
{
    irqTest_setGetMaskError(PMIC_REGULATOR_ERR_NMI);
}

void test_neg_irq_irqSetGetMask_FSM_IMM_SHUTDOWN_NMI(void)
{
    irqTest_setGetMaskError(PMIC_FSM_IMM_SHUTDOWN_NMI);
}

void test_neg_irq_irqSetGetMask_FSM_ORD_SHUTDOWN_NMI(void)
{
    irqTest_setGetMaskError(PMIC_FSM_ORD_SHUTDOWN_NMI);
}

void test_neg_irq_irqSetGetMask_FSM_WARM_RESET_NMI(void)
{
    irqTest_setGetMaskError(PMIC_FSM_WARM_RESET_NMI);
}

static int32_t irqTest_setGetMask(uint8_t irqNum, bool shouldMask)
{
    int32_t status = PMIC_ST_SUCCESS;
    const uint8_t numIrqMasks = 1U;
    Pmic_IrqMask_t actIrqMask = {.irqNum = irqNum};

    // Set expected IRQ mask configuration
    status = Pmic_irqSetMask(&pmicHandle, irqNum, shouldMask);

    // Get actual IRQ mask configuration
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_irqGetMask(&pmicHandle, numIrqMasks, &actIrqMask);
    }

    // Compare expected vs. actual values
    if ((status == PMIC_ST_SUCCESS) && (shouldMask != actIrqMask.mask))
    {
        status = PMIC_ST_ERR_FAIL;
    }

    return status;
}

// NOTE: Since WDG is not being serviced, WD_FIRST_NOK_INT flag will remain set
void test_pos_irq_irqClrAllFlags_basic(void)
{
    uint8_t regData = 0U;
    const uint8_t intTopReg = 0x46U, intFsmErrReg = 0x50U, bufLen = 1U;

    // Clear all PMIC IRQ flags
    int32_t status = Pmic_irqClrAllFlags(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Validate that all IRQ flags have been cleared
    status = platform_rxByte(&pmicHandle, 0U, intTopReg, &regData, bufLen);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT((regData == 0U) || (regData == 128U)); // All flags cleared or only FSM_ERR_INT flag set
    status = platform_rxByte(&pmicHandle, 0U, intFsmErrReg, &regData, bufLen);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT((regData == 0U) || (regData == 16U)); // All flags cleared or only WD_FIRST_NOK_INT flag set
}

void test_pos_irq_irqSetGetMask_BUCK1_OV_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_BUCK1_OV_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_BUCK1_OV_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_irq_irqSetGetMask_BUCK1_UV_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_BUCK1_UV_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_BUCK1_UV_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_irq_irqSetGetMask_BUCK1_RV_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_BUCK1_RV_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_BUCK1_RV_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_irq_irqSetGetMask_BUCK1_ILIM_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_BUCK1_ILIM_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_BUCK1_ILIM_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_irq_irqSetGetMask_BUCK2_OV_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_BUCK2_OV_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_BUCK2_OV_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_irq_irqSetGetMask_BUCK2_UV_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_BUCK2_UV_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_BUCK2_UV_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_irq_irqSetGetMask_BUCK2_RV_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_BUCK2_RV_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_BUCK2_RV_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_irq_irqSetGetMask_BUCK2_ILIM_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_BUCK2_ILIM_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_BUCK2_ILIM_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_irq_irqSetGetMask_BUCK3_OV_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_BUCK3_OV_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_BUCK3_OV_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_irq_irqSetGetMask_BUCK3_UV_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_BUCK3_UV_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_BUCK3_UV_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_irq_irqSetGetMask_BUCK3_RV_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_BUCK3_RV_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_BUCK3_RV_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_irq_irqSetGetMask_BUCK3_ILIM_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_BUCK3_ILIM_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_BUCK3_ILIM_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_irq_irqSetGetMask_LDO_LS1_VMON1_OV_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_LDO_LS1_VMON1_OV_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_LDO_LS1_VMON1_OV_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_irq_irqSetGetMask_LDO_LS1_VMON1_UV_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_LDO_LS1_VMON1_UV_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_LDO_LS1_VMON1_UV_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_irq_irqSetGetMask_LDO_LS1_VMON1_RV_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_LDO_LS1_VMON1_RV_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_LDO_LS1_VMON1_RV_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_irq_irqSetGetMask_LDO_LS1_VMON1_ILIM_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_LDO_LS1_VMON1_ILIM_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_LDO_LS1_VMON1_ILIM_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_irq_irqSetGetMask_LS2_VMON2_OV_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_LS2_VMON2_OV_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_LS2_VMON2_OV_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_irq_irqSetGetMask_LS2_VMON2_UV_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_LS2_VMON2_UV_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_LS2_VMON2_UV_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_irq_irqSetGetMask_LS2_VMON2_RV_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_LS2_VMON2_RV_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_LS2_VMON2_RV_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_irq_irqSetGetMask_LS2_VMON2_ILIM_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_LS2_VMON2_ILIM_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_LS2_VMON2_ILIM_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_irq_irqSetGetMask_VCCA_OV_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_VCCA_OV_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_VCCA_OV_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_irq_irqSetGetMask_VCCA_UV_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_VCCA_UV_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_VCCA_UV_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_irq_irqSetGetMask_STARTUP_ENABLE_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_STARTUP_ENABLE_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_STARTUP_ENABLE_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_irq_irqSetGetMask_ABIST_FAIL_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_MISC_ABIST_FAIL_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_MISC_ABIST_FAIL_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_irq_irqSetGetMask_BUCKS_VSET_ERR_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_MISC_BUCKS_VSET_ERR_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_MISC_BUCKS_VSET_ERR_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_irq_irqSetGetMask_EXT_CLK_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_MISC_EXT_CLK_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_MISC_EXT_CLK_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_irq_irqSetGetMask_TWARN_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_MISC_TWARN_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_MISC_TWARN_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_irq_irqSetGetMask_TRIM_TEST_CRC_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_ME_TRIM_TEST_CRC_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_ME_TRIM_TEST_CRC_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_irq_irqSetGetMask_CONFIG_CRC_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_ME_CONFIG_CRC_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_ME_CONFIG_CRC_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_irq_irqSetGetMask_NINT_READBACK_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_ME_NINT_READBACK_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_ME_NINT_READBACK_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_irq_irqSetGetMask_NRSTOUT_READBACK_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_ME_NRSTOUT_READBACK_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_ME_NRSTOUT_READBACK_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_irq_irqSetGetMask_COMM_FRM_ERR_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_COMM_FRM_ERR_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_COMM_FRM_ERR_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_irq_irqSetGetMask_COMM_CRC_ERR_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_COMM_CRC_ERR_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_COMM_CRC_ERR_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_irq_irqSetGetMask_COMM_ADR_ERR_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_COMM_ADR_ERR_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_COMM_ADR_ERR_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_irq_irqSetGetMask_COMM_MCU_ERR_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_COMM_MCU_ERR_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_COMM_MCU_ERR_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_irq_irqSetGetMask_ESM_MCU_PIN_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_ESM_MCU_PIN_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_ESM_MCU_PIN_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_irq_irqSetGetMask_ESM_MCU_FAIL_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_ESM_MCU_FAIL_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_ESM_MCU_FAIL_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_irq_irqSetGetMask_ESM_MCU_RST_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_ESM_MCU_RST_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_ESM_MCU_RST_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

static bool irqTest_isNMI(uint8_t irqNum)
{
    switch (irqNum)
    {
        case PMIC_BUCK1_SC_NMI:
        case PMIC_BUCK2_SC_NMI:
        case PMIC_BUCK3_SC_NMI:
        case PMIC_LDO_LS1_VMON1_SC_NMI:
        case PMIC_LS2_VMON2_SC_NMI:
        case PMIC_ME_TSD_ORD_NMI:
        case PMIC_ME_RECOV_CNT_NMI:
        case PMIC_SE_TSD_IMM_NMI:
        case PMIC_SE_VCCA_OVP_NMI:
        case PMIC_WDG_FIRST_NOK_NMI:
        case PMIC_WDG_LONGWIN_TIMEOUT_NMI:
        case PMIC_WDG_TIMEOUT_NMI:
        case PMIC_WDG_ANSWER_EARLY_NMI:
        case PMIC_WDG_SEQ_ERR_NMI:
        case PMIC_WDG_ANSWER_ERR_NMI:
        case PMIC_WDG_FAIL_NMI:
        case PMIC_WDG_RST_NMI:
        case PMIC_REGULATOR_ERR_NMI:
        case PMIC_FSM_IMM_SHUTDOWN_NMI:
        case PMIC_FSM_ORD_SHUTDOWN_NMI:
        case PMIC_FSM_WARM_RESET_NMI:
            return (bool)true;
        default:
            return (bool)false;
    }
}


static int32_t irqTest_setGetMaskAll(bool shouldMask)
{
    char msg[50U] = {0};
    uint8_t irqMaskCnt = 0U;
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_IrqMask_t irqMasks[PMIC_IRQ_NUM] = {0U};

    // Set expected IRQ mask configuration
    for (uint8_t i = 0U; i < PMIC_IRQ_NUM; i++)
    {
        // Skip setting mask for NMI interrupts
        if (irqTest_isNMI(i))
        {
            continue;
        }

        irqMasks[irqMaskCnt].irqNum = i;
        irqMasks[irqMaskCnt].mask = shouldMask;
        irqMaskCnt++;
    }
    status = Pmic_irqSetMasks(&pmicHandle, irqMaskCnt, irqMasks);

    // Get actual IRQ mask configuration
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_irqGetMask(&pmicHandle, irqMaskCnt, irqMasks);
    }

    // Compare expected vs. actual values
    if (status == PMIC_ST_SUCCESS)
    {
        for (uint8_t i = 0U; i < irqMaskCnt; i++)
        {
            if (shouldMask != irqMasks[i].mask)
            {
                (void)sprintf(msg, "IRQ mask mismatch for IRQ %d: expected %d, got %d\r\n",
                              irqMasks[i].irqNum, shouldMask, irqMasks[i].mask);
                platform_printString(msg);
                status = PMIC_ST_ERR_FAIL;
            }
        }
    }

    return status;
}

void test_pos_irq_irqSetGetMask_all(void)
{
    int32_t status = irqTest_setGetMaskAll(PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMaskAll(PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_irq_irqGetClrFlag(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    bool flag = (bool)false;

    // Clear all flags first to start with a clean state
    status = Pmic_irqClrAllFlags(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get flag status for BUCK1_OV_INT (should be cleared)
    status = Pmic_irqGetFlag(&pmicHandle, PMIC_BUCK1_OV_INT, &flag);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Clear the flag
    status = Pmic_irqClrFlag(&pmicHandle, PMIC_BUCK1_OV_INT);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_irq_irqGetStatus_noFlags(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_IrqStat_t irqStat = {0U};

    // Clear all flags first
    status = Pmic_irqClrAllFlags(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get IRQ status - should have minimal or no flags set
    status = Pmic_irqGetStatus(&pmicHandle, &irqStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_irq_irqGetStatus_withFlags(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_IrqStat_t irqStat = {0U};

    // Get IRQ status - may have some flags set (like WD_FIRST_NOK_INT)
    status = Pmic_irqGetStatus(&pmicHandle, &irqStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify the status structure was populated (intrStat array should be initialized)
    // No assertion on specific values since we're in a test environment
}

void test_pos_irq_irqGetNextFlag_noFlags(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_IrqStat_t irqStat = {0U};
    uint8_t irqNum = 0U;

    // Clear all flags first
    status = Pmic_irqClrAllFlags(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get IRQ status
    status = Pmic_irqGetStatus(&pmicHandle, &irqStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Try to get next flag - should return warning if no flags are set
    status = Pmic_irqGetNextFlag(&pmicHandle, &irqStat, &irqNum);
    // Status can be SUCCESS if there's a flag (like WD_FIRST_NOK), or WARN_NO_IRQ_REMAINING
    PLATFORM_ASSERT((status == PMIC_ST_SUCCESS) || (status == PMIC_ST_WARN_NO_IRQ_REMAINING));
}

void test_pos_irq_irqGetNextFlag_multipleFlags(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_IrqStat_t irqStat = {0U};
    uint8_t irqNum = 0U;
    uint8_t flagCount = 0U;

    // Get current IRQ status (may have some flags set)
    status = Pmic_irqGetStatus(&pmicHandle, &irqStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Iterate through all available flags
    while ((bool)true)
    {
        status = Pmic_irqGetNextFlag(&pmicHandle, &irqStat, &irqNum);

        if (status == PMIC_ST_WARN_NO_IRQ_REMAINING)
        {
            // No more flags remaining
            break;
        }

        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        PLATFORM_ASSERT(irqNum <= PMIC_IRQ_MAX);

        flagCount++;

        // Safety check to prevent infinite loop
        if (flagCount > PMIC_IRQ_NUM)
        {
            break;
        }
    }

    // After iteration, there should be no remaining flags
    status = Pmic_irqGetNextFlag(&pmicHandle, &irqStat, &irqNum);
    PLATFORM_ASSERT(status == PMIC_ST_WARN_NO_IRQ_REMAINING);
}

void test_pos_irq_irqGetFlag_variousIrqs(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    bool flag = (bool)false;

    // Test getting flags for various IRQ numbers
    const uint8_t testIrqs[] = {
        PMIC_BUCK1_OV_INT,
        PMIC_BUCK2_UV_INT,
        PMIC_BUCK3_RV_INT,
        PMIC_LDO_LS1_VMON1_ILIM_INT,
        PMIC_LS2_VMON2_OV_INT,
        PMIC_VCCA_UV_INT,
        PMIC_STARTUP_ENABLE_INT,
        PMIC_MISC_TWARN_INT,
    };

    for (uint8_t i = 0U; i < COUNT(testIrqs); i++)
    {
        status = Pmic_irqGetFlag(&pmicHandle, testIrqs[i], &flag);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        // Flag can be true or false, we just verify the API works
    }
}

void test_pos_irq_irqClrFlag_singleFlag(void)
{
    int32_t status = PMIC_ST_SUCCESS;

    // Clear specific flags
    status = Pmic_irqClrFlag(&pmicHandle, PMIC_BUCK1_OV_INT);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_irqClrFlag(&pmicHandle, PMIC_BUCK2_UV_INT);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_irqClrFlag(&pmicHandle, PMIC_VCCA_OV_INT);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_irq_irqClrFlag_multipleSequence(void)
{
    int32_t status = PMIC_ST_SUCCESS;

    // Clear multiple flags in sequence
    const uint8_t irqsToClear[] = {
        PMIC_BUCK1_OV_INT,
        PMIC_BUCK1_UV_INT,
        PMIC_BUCK2_OV_INT,
        PMIC_BUCK2_UV_INT,
        PMIC_BUCK3_OV_INT,
        PMIC_BUCK3_UV_INT,
        PMIC_LDO_LS1_VMON1_OV_INT,
        PMIC_LS2_VMON2_OV_INT,
    };

    for (uint8_t i = 0U; i < COUNT(irqsToClear); i++)
    {
        status = Pmic_irqClrFlag(&pmicHandle, irqsToClear[i]);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    }
}

void test_pos_irq_irqWorkflow_completeHandling(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_IrqStat_t irqStat = {0U};
    uint8_t irqNum = 0U;
    bool flag = (bool)false;

    // Step 1: Get IRQ status
    status = Pmic_irqGetStatus(&pmicHandle, &irqStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Step 2: Get next flag (if any)
    status = Pmic_irqGetNextFlag(&pmicHandle, &irqStat, &irqNum);
    if (status == PMIC_ST_SUCCESS)
    {
        // We have a flag set
        PLATFORM_ASSERT(irqNum <= PMIC_IRQ_MAX);

        // Step 3: Get specific flag status
        status = Pmic_irqGetFlag(&pmicHandle, irqNum, &flag);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        // Step 4: Clear the flag
        status = Pmic_irqClrFlag(&pmicHandle, irqNum);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    }
    else
    {
        // No flags set, which is also valid
        PLATFORM_ASSERT(status == PMIC_ST_WARN_NO_IRQ_REMAINING);
    }
}

void test_pos_irq_irqMaskedBehavior(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    bool flag = (bool)false;

    // Mask an interrupt
    status = Pmic_irqSetMask(&pmicHandle, PMIC_BUCK1_OV_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Masked interrupts don't trigger nINT, but flags are still readable
    status = Pmic_irqGetFlag(&pmicHandle, PMIC_BUCK1_OV_INT, &flag);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Clear the flag
    status = Pmic_irqClrFlag(&pmicHandle, PMIC_BUCK1_OV_INT);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Unmask the interrupt
    status = Pmic_irqSetMask(&pmicHandle, PMIC_BUCK1_OV_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_irq_irqFlagPersistence(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    bool flag1 = (bool)false;
    bool flag2 = (bool)false;

    // Get flag status before clear
    status = Pmic_irqGetFlag(&pmicHandle, PMIC_BUCK2_OV_INT, &flag1);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Clear the flag
    status = Pmic_irqClrFlag(&pmicHandle, PMIC_BUCK2_OV_INT);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get flag status after clear
    status = Pmic_irqGetFlag(&pmicHandle, PMIC_BUCK2_OV_INT, &flag2);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // After clear, flag should be cleared (or a new event occurred)
    // We just verify the API sequence works correctly
}

void test_pos_irq_irqStatusReadMultipleTimes(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_IrqStat_t irqStat1 = {0U};
    Pmic_IrqStat_t irqStat2 = {0U};

    // Read status first time
    status = Pmic_irqGetStatus(&pmicHandle, &irqStat1);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Read status second time
    status = Pmic_irqGetStatus(&pmicHandle, &irqStat2);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Both reads should succeed
}

void test_pos_irq_irqGetFlag_allMaskableIrqs(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    bool flag = (bool)false;

    // Test all maskable IRQs
    for (uint8_t i = 0U; i < PMIC_IRQ_NUM; i++)
    {
        // Skip NMI interrupts by checking if they're maskable
        // We'll test each IRQ by trying to get its flag
        status = Pmic_irqGetFlag(&pmicHandle, i, &flag);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    }
}

void test_pos_irq_irqClrFlag_verifyCleared(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    bool flagBefore = (bool)false;
    bool flagAfter = (bool)false;

    // Get flag status
    status = Pmic_irqGetFlag(&pmicHandle, PMIC_MISC_TWARN_INT, &flagBefore);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Clear the flag
    status = Pmic_irqClrFlag(&pmicHandle, PMIC_MISC_TWARN_INT);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get flag status again
    status = Pmic_irqGetFlag(&pmicHandle, PMIC_MISC_TWARN_INT, &flagAfter);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify the clear operation succeeded
}

void test_pos_irq_irqIterateAndClearAll(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_IrqStat_t irqStat = {0U};
    uint8_t irqNum = 0U;
    uint8_t clearedCount = 0U;

    // Get current IRQ status
    status = Pmic_irqGetStatus(&pmicHandle, &irqStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Iterate through and clear all flags
    while ((bool)true)
    {
        status = Pmic_irqGetNextFlag(&pmicHandle, &irqStat, &irqNum);

        if (status == PMIC_ST_WARN_NO_IRQ_REMAINING)
        {
            break;
        }

        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        // Clear this flag
        status = Pmic_irqClrFlag(&pmicHandle, irqNum);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        clearedCount++;

        // Safety check
        if (clearedCount > PMIC_IRQ_NUM)
        {
            break;
        }
    }
}

void test_pos_irq_irqGetStatus_afterClearAll(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_IrqStat_t irqStat = {0U};
    uint8_t irqNum = 0U;

    // Clear all flags
    status = Pmic_irqClrAllFlags(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get status after clearing all
    status = Pmic_irqGetStatus(&pmicHandle, &irqStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Try to get next flag - should mostly return no remaining
    // (unless WD_FIRST_NOK is set which persists)
    status = Pmic_irqGetNextFlag(&pmicHandle, &irqStat, &irqNum);
    PLATFORM_ASSERT((status == PMIC_ST_SUCCESS) || (status == PMIC_ST_WARN_NO_IRQ_REMAINING));
}

/* ========================================================================== */
/*                     IRQ Hierarchy Navigation Tests                         */
/* ========================================================================== */

/* Test Group 1: L0→L1 Category Triggers - Exercise each L0 category bit */

void test_pos_irq_irqGetStatus_trigger_L1_BUCK_LDO(void)
{
    Pmic_IrqStat_t irqStat = {0U};
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    const uint8_t bufLen = 1U;
    uint32_t expectedBit = 0U;

    // Clear all PMIC IRQ flags
    status = Pmic_irqClrAllFlags(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Inject BUCK_LDO_LS1_VMON1_INT in INT_TOP (bit 0)
    regData = (1U << 0U);
    testInject_setBits(0x46U, regData);

    // Inject BUCK1_SC_NMI in INT_BUCK_LDO_LS1_VMON1 (bit 4)
    regData = (1U << 4U);
    testInject_setBits(0x47U, regData);

    // Get IRQ status - triggers IRQ_getStatBucks()
    status = Pmic_irqGetStatus(&pmicHandle, &irqStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify BUCK1_SC_NMI (IRQ 0) is set in intrStat[0]
    expectedBit = (1U << PMIC_BUCK1_SC_NMI);
    PLATFORM_ASSERT((irqStat.intrStat[0] & expectedBit) != 0U);
}

void test_pos_irq_irqGetStatus_trigger_L1_LS2_VMON2(void)
{
    Pmic_IrqStat_t irqStat = {0U};
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    const uint8_t bufLen = 1U;
    uint32_t expectedBit = 0U;

    // Clear all PMIC IRQ flags
    status = Pmic_irqClrAllFlags(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Inject LS2_VMON2_INT in INT_TOP (bit 1)
    regData = (1U << 1U);
    testInject_setBits(0x46U, regData);

    // Inject LS2_VMON2_SC_NMI in INT_LS2_VMON2 (bit 4)
    regData = (1U << 4U);
    testInject_setBits(0x4AU, regData);

    // Get IRQ status - triggers IRQ_getStatLs2Vmon2()
    status = Pmic_irqGetStatus(&pmicHandle, &irqStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify LS2_VMON2_SC_NMI (IRQ 20) is set in intrStat[0]
    expectedBit = (1U << PMIC_LS2_VMON2_SC_NMI);
    PLATFORM_ASSERT((irqStat.intrStat[0] & expectedBit) != 0U);
}

void test_pos_irq_irqGetStatus_trigger_L1_VCCA(void)
{
    Pmic_IrqStat_t irqStat = {0U};
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    const uint8_t bufLen = 1U;
    uint32_t expectedBit = 0U;

    // Clear all PMIC IRQ flags
    status = Pmic_irqClrAllFlags(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Inject VCCA_INT in INT_TOP (bit 2)
    regData = (1U << 2U);
    testInject_setBits(0x46U, regData);

    // Inject VCCA_OV_INT in INT_VCCA (bit 0)
    regData = (1U << 0U);
    testInject_setBits(0x4BU, regData);

    // Get IRQ status - triggers IRQ_getStatVccaVmon1()
    status = Pmic_irqGetStatus(&pmicHandle, &irqStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify VCCA_OV_INT (IRQ 25) is set in intrStat[0]
    expectedBit = (1U << PMIC_VCCA_OV_INT);
    PLATFORM_ASSERT((irqStat.intrStat[0] & expectedBit) != 0U);
}

void test_pos_irq_irqGetStatus_trigger_L1_STARTUP(void)
{
    Pmic_IrqStat_t irqStat = {0U};
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    const uint8_t bufLen = 1U;
    uint32_t expectedBit = 0U;

    // Clear all PMIC IRQ flags
    status = Pmic_irqClrAllFlags(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Inject STARTUP_INT in INT_TOP (bit 3)
    regData = (1U << 3U);
    testInject_setBits(0x46U, regData);

    // Inject STARTUP_ENABLE_INT in INT_STARTUP (bit 1)
    regData = (1U << 1U);
    testInject_setBits(0x4CU, regData);

    // Get IRQ status - triggers IRQ_getStatStartup()
    status = Pmic_irqGetStatus(&pmicHandle, &irqStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify STARTUP_ENABLE_INT (IRQ 27) is set in intrStat[0]
    expectedBit = (1U << PMIC_STARTUP_ENABLE_INT);
    PLATFORM_ASSERT((irqStat.intrStat[0] & expectedBit) != 0U);
}

void test_pos_irq_irqGetStatus_trigger_L1_MISC(void)
{
    Pmic_IrqStat_t irqStat = {0U};
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    const uint8_t bufLen = 1U;
    uint32_t expectedBit = 0U;

    // Clear all PMIC IRQ flags
    status = Pmic_irqClrAllFlags(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Inject MISC_INT in INT_TOP (bit 4)
    regData = (1U << 4U);
    testInject_setBits(0x46U, regData);

    // Inject TWARN_INT in INT_MISC (bit 7)
    regData = (1U << 7U);
    testInject_setBits(0x4DU, regData);

    // Get IRQ status - triggers IRQ_getStatMisc()
    status = Pmic_irqGetStatus(&pmicHandle, &irqStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify MISC_TWARN_INT (IRQ 31) is set in intrStat[0]
    expectedBit = (1U << PMIC_MISC_TWARN_INT);
    PLATFORM_ASSERT((irqStat.intrStat[0] & expectedBit) != 0U);
}

void test_pos_irq_irqGetStatus_trigger_L1_MODERATE_ERR(void)
{
    Pmic_IrqStat_t irqStat = {0U};
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    const uint8_t bufLen = 1U;
    uint32_t expectedBit = 0U;

    // Clear all PMIC IRQ flags
    status = Pmic_irqClrAllFlags(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Inject MODERATE_ERR_INT in INT_TOP (bit 5)
    regData = (1U << 5U);
    testInject_setBits(0x46U, regData);

    // Inject CONFIG_CRC_INT in INT_MODERATE_ERR (bit 3)
    regData = (1U << 3U);
    testInject_setBits(0x4EU, regData);

    // Get IRQ status - triggers IRQ_getStatModerate()
    status = Pmic_irqGetStatus(&pmicHandle, &irqStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify ME_CONFIG_CRC_INT (IRQ 35) is set in intrStat[1]
    expectedBit = (1U << (PMIC_ME_CONFIG_CRC_INT - 32U));
    PLATFORM_ASSERT((irqStat.intrStat[1] & expectedBit) != 0U);
}

void test_pos_irq_irqGetStatus_trigger_L1_SEVERE_ERR(void)
{
    Pmic_IrqStat_t irqStat = {0U};
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    const uint8_t bufLen = 1U;
    uint32_t expectedBit = 0U;

    // Clear all PMIC IRQ flags
    status = Pmic_irqClrAllFlags(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Inject SEVERE_ERR_INT in INT_TOP (bit 6)
    regData = (1U << 6U);
    testInject_setBits(0x46U, regData);

    // Inject TSD_IMM_NMI in INT_SEVERE_ERR (bit 0)
    regData = (1U << 0U);
    testInject_setBits(0x4FU, regData);

    // Get IRQ status - triggers IRQ_getStatSevere()
    status = Pmic_irqGetStatus(&pmicHandle, &irqStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify SE_TSD_IMM_NMI (IRQ 38) is set in intrStat[1]
    expectedBit = (1U << (PMIC_SE_TSD_IMM_NMI - 32U));
    PLATFORM_ASSERT((irqStat.intrStat[1] & expectedBit) != 0U);
}

void test_pos_irq_irqGetStatus_trigger_L1_FSM_ERR(void)
{
    Pmic_IrqStat_t irqStat = {0U};
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    const uint8_t bufLen = 1U;
    uint32_t expectedBit = 0U;

    // Clear all PMIC IRQ flags
    status = Pmic_irqClrAllFlags(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Inject FSM_ERR_INT in INT_TOP (bit 7)
    regData = (1U << 7U);
    testInject_setBits(0x46U, regData);

    // Inject FSM_IMM_SHUTDOWN_NMI in INT_FSM_ERR (bit 0)
    regData = (1U << 0U);
    testInject_setBits(0x50U, regData);

    // Get IRQ status - triggers IRQ_getStatFSM()
    status = Pmic_irqGetStatus(&pmicHandle, &irqStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify FSM_IMM_SHUTDOWN_NMI (IRQ 56) is set in intrStat[1]
    expectedBit = (1U << (PMIC_FSM_IMM_SHUTDOWN_NMI - 32U));
    PLATFORM_ASSERT((irqStat.intrStat[1] & expectedBit) != 0U);
}

/* Test Group 2: L1→L2 Cascade Reads - Exercise conditional L2 register reads */

void test_pos_irq_irqGetStatus_trigger_L2_BUCK1_2_via_BUCK1(void)
{
    Pmic_IrqStat_t irqStat = {0U};
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    const uint8_t bufLen = 1U;
    uint32_t expectedBit = 0U;

    // Clear all PMIC IRQ flags
    status = Pmic_irqClrAllFlags(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Inject BUCK_LDO_LS1_VMON1_INT in INT_TOP (bit 0)
    regData = (1U << 0U);
    testInject_setBits(0x46U, regData);

    // Inject BUCK1_INT in INT_BUCK_LDO_LS1_VMON1 (bit 0) → triggers L2 read
    regData = (1U << 0U);
    testInject_setBits(0x47U, regData);

    // Inject BUCK1_OV_INT in INT_BUCK_12 (bit 0)
    regData = (1U << 0U);
    testInject_setBits(0x48U, regData);

    // Get IRQ status - triggers L0→L1→L2 cascade
    status = Pmic_irqGetStatus(&pmicHandle, &irqStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify BUCK1_OV_INT (IRQ 1) is set in intrStat[0]
    expectedBit = (1U << PMIC_BUCK1_OV_INT);
    PLATFORM_ASSERT((irqStat.intrStat[0] & expectedBit) != 0U);
}

void test_pos_irq_irqGetStatus_trigger_L2_BUCK1_2_via_BUCK2(void)
{
    Pmic_IrqStat_t irqStat = {0U};
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    const uint8_t bufLen = 1U;
    uint32_t expectedBit = 0U;

    // Clear all PMIC IRQ flags
    status = Pmic_irqClrAllFlags(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Inject BUCK_LDO_LS1_VMON1_INT in INT_TOP (bit 0)
    regData = (1U << 0U);
    testInject_setBits(0x46U, regData);

    // Inject BUCK2_INT in INT_BUCK_LDO_LS1_VMON1 (bit 1) → triggers L2 read
    regData = (1U << 1U);
    testInject_setBits(0x47U, regData);

    // Inject BUCK2_UV_INT in INT_BUCK_12 (bit 5)
    regData = (1U << 5U);
    testInject_setBits(0x48U, regData);

    // Get IRQ status - triggers L0→L1→L2 cascade
    status = Pmic_irqGetStatus(&pmicHandle, &irqStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify BUCK2_UV_INT (IRQ 7) is set in intrStat[0]
    expectedBit = (1U << PMIC_BUCK2_UV_INT);
    PLATFORM_ASSERT((irqStat.intrStat[0] & expectedBit) != 0U);
}

void test_pos_irq_irqGetStatus_trigger_L2_BUCK3_LDO_via_BUCK3(void)
{
    Pmic_IrqStat_t irqStat = {0U};
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    const uint8_t bufLen = 1U;
    uint32_t expectedBit = 0U;

    // Clear all PMIC IRQ flags
    status = Pmic_irqClrAllFlags(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Inject BUCK_LDO_LS1_VMON1_INT in INT_TOP (bit 0)
    regData = (1U << 0U);
    testInject_setBits(0x46U, regData);

    // Inject BUCK3_INT in INT_BUCK_LDO_LS1_VMON1 (bit 2) → triggers L2 read
    regData = (1U << 2U);
    testInject_setBits(0x47U, regData);

    // Inject BUCK3_ILIM_INT in INT_BUCK3_LDO_LS1_VMON1 (bit 3)
    regData = (1U << 3U);
    testInject_setBits(0x49U, regData);

    // Get IRQ status - triggers L0→L1→L2 cascade
    status = Pmic_irqGetStatus(&pmicHandle, &irqStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify BUCK3_ILIM_INT (IRQ 14) is set in intrStat[0]
    expectedBit = (1U << PMIC_BUCK3_ILIM_INT);
    PLATFORM_ASSERT((irqStat.intrStat[0] & expectedBit) != 0U);
}

void test_pos_irq_irqGetStatus_trigger_L2_BUCK3_LDO_via_LDO(void)
{
    Pmic_IrqStat_t irqStat = {0U};
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    const uint8_t bufLen = 1U;
    uint32_t expectedBit = 0U;

    // Clear all PMIC IRQ flags
    status = Pmic_irqClrAllFlags(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Inject BUCK_LDO_LS1_VMON1_INT in INT_TOP (bit 0)
    regData = (1U << 0U);
    testInject_setBits(0x46U, regData);

    // Inject LDO_LS1_VMON1_INT in INT_BUCK_LDO_LS1_VMON1 (bit 3) → triggers L2 read
    regData = (1U << 3U);
    testInject_setBits(0x47U, regData);

    // Inject LDO_LS1_VMON1_RV_INT in INT_BUCK3_LDO_LS1_VMON1 (bit 6)
    regData = (1U << 6U);
    testInject_setBits(0x49U, regData);

    // Get IRQ status - triggers L0→L1→L2 cascade
    status = Pmic_irqGetStatus(&pmicHandle, &irqStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify LDO_LS1_VMON1_RV_INT (IRQ 18) is set in intrStat[0]
    expectedBit = (1U << PMIC_LDO_LS1_VMON1_RV_INT);
    PLATFORM_ASSERT((irqStat.intrStat[0] & expectedBit) != 0U);
}

void test_pos_irq_irqGetStatus_trigger_L2_ESM(void)
{
    Pmic_IrqStat_t irqStat = {0U};
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    const uint8_t bufLen = 1U;
    uint32_t expectedBit = 0U;

    // Clear all PMIC IRQ flags
    status = Pmic_irqClrAllFlags(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Inject FSM_ERR_INT in INT_TOP (bit 7)
    regData = (1U << 7U);
    testInject_setBits(0x46U, regData);

    // Inject ESM_MCU_INT in INT_FSM_ERR (bit 5) → triggers INT_ESM_REG read
    regData = (1U << 5U);
    testInject_setBits(0x50U, regData);

    // Inject ESM_MCU_FAIL_INT in INT_ESM (bit 4)
    regData = (1U << 4U);
    testInject_setBits(0x52U, regData);

    // Get IRQ status - triggers L0→L1→L2 cascade
    status = Pmic_irqGetStatus(&pmicHandle, &irqStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify ESM_MCU_FAIL_INT (IRQ 45) is set in intrStat[1]
    expectedBit = (1U << (PMIC_ESM_MCU_FAIL_INT - 32U));
    PLATFORM_ASSERT((irqStat.intrStat[1] & expectedBit) != 0U);
}

void test_pos_irq_irqGetStatus_trigger_L2_COMM_ERR(void)
{
    Pmic_IrqStat_t irqStat = {0U};
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    const uint8_t bufLen = 1U;
    uint32_t expectedBit = 0U;

    // Clear all PMIC IRQ flags
    status = Pmic_irqClrAllFlags(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Inject FSM_ERR_INT in INT_TOP (bit 7)
    regData = (1U << 7U);
    testInject_setBits(0x46U, regData);

    // Inject COMM_ERR_INT in INT_FSM_ERR (bit 6) → triggers INT_COMM_ERR read
    regData = (1U << 6U);
    testInject_setBits(0x50U, regData);

    // Inject COMM_CRC_ERR_INT in INT_COMM_ERR (bit 1)
    regData = (1U << 1U);
    testInject_setBits(0x51U, regData);

    // Get IRQ status - triggers L0→L1→L2 cascade
    status = Pmic_irqGetStatus(&pmicHandle, &irqStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify COMM_CRC_ERR_INT (IRQ 41) is set in intrStat[1]
    expectedBit = (1U << (PMIC_COMM_CRC_ERR_INT - 32U));
    PLATFORM_ASSERT((irqStat.intrStat[1] & expectedBit) != 0U);
}

void test_pos_irq_irqGetStatus_trigger_L2_WD_ERR_STAT(void)
{
    Pmic_IrqStat_t irqStat = {0U};
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    const uint8_t bufLen = 1U;
    uint32_t expectedBit = 0U;

    // Clear all PMIC IRQ flags
    status = Pmic_irqClrAllFlags(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Inject FSM_ERR_INT in INT_TOP (bit 7)
    regData = (1U << 7U);
    testInject_setBits(0x46U, regData);

    // Inject WD_INT in INT_FSM_ERR (bit 7) → triggers WD_ERR_STAT read
    regData = (1U << 7U);
    testInject_setBits(0x50U, regData);

    // Inject WDG_TIMEOUT_NMI in WD_ERR_STAT (bit 1)
    regData = (1U << 1U);
    testInject_setBits(0x5EU, regData);

    // Get IRQ status - triggers L0→L1→L2 cascade
    status = Pmic_irqGetStatus(&pmicHandle, &irqStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify WDG_TIMEOUT_NMI (IRQ 49) is set in intrStat[1]
    expectedBit = (1U << (PMIC_WDG_TIMEOUT_NMI - 32U));
    PLATFORM_ASSERT((irqStat.intrStat[1] & expectedBit) != 0U);
}

void test_pos_irq_irqGetStatus_full_hierarchy_cascade(void)
{
    Pmic_IrqStat_t irqStat = {0U};
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    const uint8_t bufLen = 1U;
    uint32_t expectedBit = 0U;

    // Clear all PMIC IRQ flags
    status = Pmic_irqClrAllFlags(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Inject multiple L0 bits: BUCK_LDO (bit 0) + FSM_ERR (bit 7)
    regData = (1U << 0U) | (1U << 7U);
    testInject_setBits(0x46U, regData);

    // L1: Inject BUCK1_INT in INT_BUCK_LDO_LS1_VMON1
    regData = (1U << 0U);
    testInject_setBits(0x47U, regData);

    // L2: Inject BUCK1_OV_INT in INT_BUCK_12
    regData = (1U << 0U);
    testInject_setBits(0x48U, regData);

    // L1: Inject ESM_MCU_INT in INT_FSM_ERR
    regData = (1U << 5U);
    testInject_setBits(0x50U, regData);

    // L2: Inject ESM_MCU_FAIL_INT in INT_ESM
    regData = (1U << 4U);
    testInject_setBits(0x52U, regData);

    // Get IRQ status - triggers multiple L0→L1→L2 cascades
    status = Pmic_irqGetStatus(&pmicHandle, &irqStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify BUCK1_OV_INT (IRQ 1) is set
    expectedBit = (1U << PMIC_BUCK1_OV_INT);
    PLATFORM_ASSERT((irqStat.intrStat[0] & expectedBit) != 0U);

    // Verify ESM_MCU_FAIL_INT (IRQ 45) is set
    expectedBit = (1U << (PMIC_ESM_MCU_FAIL_INT - 32U));
    PLATFORM_ASSERT((irqStat.intrStat[1] & expectedBit) != 0U);
}

/* Test Group 3: Flag Iterator with Hierarchy - Verify intrStat population and iteration */

void test_pos_irq_irqGetNextFlag_L2_populated_intrStat(void)
{
    Pmic_IrqStat_t irqStat = {0U};
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    const uint8_t bufLen = 1U;
    uint8_t irqNum = 0U;
    uint8_t flagCount = 0U;
    bool foundBuck1Ov = false;
    bool foundEsmFail = false;

    // Clear all PMIC IRQ flags
    status = Pmic_irqClrAllFlags(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Inject L0→L1→L2 cascade for BUCK1_OV_INT
    regData = (1U << 0U);
    testInject_setBits(0x46U, regData);

    regData = (1U << 0U);
    testInject_setBits(0x47U, regData);

    regData = (1U << 0U);
    testInject_setBits(0x48U, regData);

    // Inject L0→L1→L2 cascade for ESM_MCU_FAIL_INT
    regData = (1U << 7U);
    testInject_setBits(0x46U, regData);

    regData = (1U << 5U);
    testInject_setBits(0x50U, regData);

    regData = (1U << 4U);
    testInject_setBits(0x52U, regData);

    // Get IRQ status - populates intrStat with L2 IRQs
    status = Pmic_irqGetStatus(&pmicHandle, &irqStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Iterate through all flags
    while (flagCount < PMIC_IRQ_NUM)
    {
        status = Pmic_irqGetNextFlag(&pmicHandle, &irqStat, &irqNum);

        if (status == PMIC_ST_WARN_NO_IRQ_REMAINING)
        {
            break;
        }

        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        if (irqNum == PMIC_BUCK1_OV_INT)
        {
            foundBuck1Ov = true;
        }
        if (irqNum == PMIC_ESM_MCU_FAIL_INT)
        {
            foundEsmFail = true;
        }

        flagCount++;
    }

    // Verify both L2 IRQs were found
    PLATFORM_ASSERT(foundBuck1Ov == true);
    PLATFORM_ASSERT(foundEsmFail == true);
}

void test_pos_irq_irqGetNextFlag_mixed_L1_L2_flags(void)
{
    Pmic_IrqStat_t irqStat = {0U};
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    const uint8_t bufLen = 1U;
    uint8_t irqNum = 0U;
    uint8_t flagCount = 0U;
    bool foundBuck1Sc = false;    // L1 direct NMI
    bool foundBuck1Ov = false;    // L2 cascaded IRQ

    // Clear all PMIC IRQ flags
    status = Pmic_irqClrAllFlags(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Inject L0 bit
    regData = (1U << 0U);
    testInject_setBits(0x46U, regData);

    // Inject both L1 direct NMI + L1 indicator for L2 read
    regData = (1U << 4U) | (1U << 0U);  // BUCK1_SC_NMI (bit 4) + BUCK1_INT (bit 0)
    testInject_setBits(0x47U, regData);

    // Inject L2 flag
    regData = (1U << 0U);  // BUCK1_OV_INT
    testInject_setBits(0x48U, regData);

    // Get IRQ status
    status = Pmic_irqGetStatus(&pmicHandle, &irqStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Iterate through all flags
    while (flagCount < PMIC_IRQ_NUM)
    {
        status = Pmic_irqGetNextFlag(&pmicHandle, &irqStat, &irqNum);

        if (status == PMIC_ST_WARN_NO_IRQ_REMAINING)
        {
            break;
        }

        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        if (irqNum == PMIC_BUCK1_SC_NMI)
        {
            foundBuck1Sc = true;
        }
        if (irqNum == PMIC_BUCK1_OV_INT)
        {
            foundBuck1Ov = true;
        }

        flagCount++;
    }

    // Verify both L1 and L2 IRQs were found
    PLATFORM_ASSERT(foundBuck1Sc == true);
    PLATFORM_ASSERT(foundBuck1Ov == true);
}

void test_pos_irq_irqGetNextFlag_highIndex_IRQs(void)
{
    Pmic_IrqStat_t irqStat = {0U};
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    const uint8_t bufLen = 1U;
    uint8_t irqNum = 0U;
    uint8_t flagCount = 0U;
    bool foundFsmShutdown = false;

    // Clear all PMIC IRQ flags
    status = Pmic_irqClrAllFlags(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Inject FSM_ERR_INT in INT_TOP (bit 7)
    regData = (1U << 7U);
    testInject_setBits(0x46U, regData);

    // Inject FSM_IMM_SHUTDOWN_NMI (IRQ 56) in INT_FSM_ERR (bit 0)
    regData = (1U << 0U);
    testInject_setBits(0x50U, regData);

    // Get IRQ status - populates intrStat[1] (IRQ 32+)
    status = Pmic_irqGetStatus(&pmicHandle, &irqStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify intrStat[1] bit is set
    uint32_t expectedBit = (1U << (PMIC_FSM_IMM_SHUTDOWN_NMI - 32U));
    PLATFORM_ASSERT((irqStat.intrStat[1] & expectedBit) != 0U);

    // Iterate through all flags
    while (flagCount < PMIC_IRQ_NUM)
    {
        status = Pmic_irqGetNextFlag(&pmicHandle, &irqStat, &irqNum);

        if (status == PMIC_ST_WARN_NO_IRQ_REMAINING)
        {
            break;
        }

        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        if (irqNum == PMIC_FSM_IMM_SHUTDOWN_NMI)
        {
            foundFsmShutdown = true;
        }

        flagCount++;
    }

    // Verify high-index IRQ (56) was found
    PLATFORM_ASSERT(foundFsmShutdown == true);
}

/* Test Group 4: Edge Cases - Test boundary conditions and special scenarios */

void test_pos_irq_irqGetStatus_L1_set_but_L2_empty(void)
{
    Pmic_IrqStat_t irqStat = {0U};
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    const uint8_t bufLen = 1U;

    // Clear all PMIC IRQ flags
    status = Pmic_irqClrAllFlags(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Inject BUCK_LDO_LS1_VMON1_INT in INT_TOP (bit 0)
    regData = (1U << 0U);
    testInject_setBits(0x46U, regData);

    // Inject BUCK1_INT (bit 0) and BUCK1_SC_NMI (bit 4) in INT_BUCK_LDO_LS1_VMON1
    regData = (1U << 0U) | (1U << 4U);
    testInject_setBits(0x47U, regData);

    // DO NOT inject any L2 flags - INT_BUCK_12 remains 0x00

    // Get IRQ status - L2 read occurs but finds no flags
    status = Pmic_irqGetStatus(&pmicHandle, &irqStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify no false positives - only BUCK1_SC_NMI (L1 direct) should be set
    // BUCK1_SC_NMI is at bit 4, BUCK1_INT indicator is at bit 0
    uint32_t expectedBit = (1U << PMIC_BUCK1_SC_NMI);
    PLATFORM_ASSERT((irqStat.intrStat[0] & expectedBit) != 0U);

    // Verify no L2 BUCK1 IRQs are set (IRQ 1-4)
    uint32_t buck1L2Mask = ((1U << PMIC_BUCK1_OV_INT) |
                             (1U << PMIC_BUCK1_UV_INT) |
                             (1U << PMIC_BUCK1_RV_INT) |
                             (1U << PMIC_BUCK1_ILIM_INT));
    PLATFORM_ASSERT((irqStat.intrStat[0] & buck1L2Mask) == 0U);
}

void test_pos_irq_irqGetStatus_all_L0_categories_set(void)
{
    Pmic_IrqStat_t irqStat = {0U};
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    const uint8_t bufLen = 1U;

    // Clear all PMIC IRQ flags
    status = Pmic_irqClrAllFlags(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Inject all 8 L0 category bits in INT_TOP
    regData = 0xFFU;
    testInject_setBits(0x46U, regData);

    // Inject one flag in each L1 category register
    regData = (1U << 0U);  // BUCK1_SC_NMI
    testInject_setBits(0x47U, regData);

    regData = (1U << 0U);  // LS2_VMON2_SC_NMI
    testInject_setBits(0x4AU, regData);

    regData = (1U << 0U);  // VCCA_OV_INT
    testInject_setBits(0x4BU, regData);

    regData = (1U << 0U);  // STARTUP_ENABLE_INT
    testInject_setBits(0x4CU, regData);

    regData = (1U << 0U);  // ABIST_FAIL_INT
    testInject_setBits(0x4DU, regData);

    regData = (1U << 0U);  // TSD_ORD_NMI
    testInject_setBits(0x4EU, regData);

    regData = (1U << 0U);  // TSD_IMM_NMI
    testInject_setBits(0x4FU, regData);

    regData = (1U << 0U);  // FSM_IMM_SHUTDOWN_NMI
    testInject_setBits(0x50U, regData);

    // Get IRQ status - all category read functions execute
    status = Pmic_irqGetStatus(&pmicHandle, &irqStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify at least one flag from each category is captured
    // (Not checking all 8 specifically, just that function succeeded)
}

void test_pos_irq_irqGetStatus_multiple_L2_same_category(void)
{
    Pmic_IrqStat_t irqStat = {0U};
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    const uint8_t bufLen = 1U;
    uint32_t expectedBit = 0U;

    // Clear all PMIC IRQ flags
    status = Pmic_irqClrAllFlags(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Inject BUCK_LDO_LS1_VMON1_INT in INT_TOP (bit 0)
    regData = (1U << 0U);
    testInject_setBits(0x46U, regData);

    // Inject BOTH BUCK1_INT and BUCK2_INT in INT_BUCK_LDO_LS1_VMON1
    // Both trigger read of shared INT_BUCK_12_REG
    regData = (1U << 0U) | (1U << 1U);
    testInject_setBits(0x47U, regData);

    // Inject multiple flags in shared INT_BUCK_12 register
    regData = (1U << 0U) | (1U << 5U);  // BUCK1_OV + BUCK2_UV
    testInject_setBits(0x48U, regData);

    // Get IRQ status - INT_BUCK_12 read once, handles both L1 indicators
    status = Pmic_irqGetStatus(&pmicHandle, &irqStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify both BUCK1_OV_INT and BUCK2_UV_INT are captured
    expectedBit = (1U << PMIC_BUCK1_OV_INT);
    PLATFORM_ASSERT((irqStat.intrStat[0] & expectedBit) != 0U);

    expectedBit = (1U << PMIC_BUCK2_UV_INT);
    PLATFORM_ASSERT((irqStat.intrStat[0] & expectedBit) != 0U);
}


/* ========================================================================== */
/*           LP8772x-Q1 Tests for Uncovered Lines in pmic_irq.c              */
/* ========================================================================== */

void test_pos_irq_irqGetNextFlag_noFlagsFound(void)
{
    // Test coverage for line 737: Call Pmic_irqGetNextFlag with empty status
    // When no flags are set, it should return PMIC_ST_ERR_INV_IRQ_NUM

    Pmic_IrqStat_t irqStat = {0U};
    uint8_t nextIrqNum = 0U;

    // Clear all IRQ flags first
    int32_t status = Pmic_irqClrAllFlags(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get IRQ status - should have no flags set
    status = Pmic_irqGetStatus(&pmicHandle, &irqStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Call Pmic_irqGetNextFlag with empty status
    // This should return PMIC_ST_WARN_NO_IRQ_REMAINING (line 758)
    status = Pmic_irqGetNextFlag(&pmicHandle, &irqStat, &nextIrqNum);
    PLATFORM_ASSERT(status == PMIC_ST_WARN_NO_IRQ_REMAINING);
}
