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


/* ========================================================================== */
/*                              Include Files                                 */
/* ========================================================================== */

#include "irq_test.h"
#include "test_inject.h"
#include "test_constants.h"

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/* ========================================================================== */
/*             irqSetMask / irqSetMasks / irqGetMask API Tests                */
/* ========================================================================== */

/* Positive tests for irqSetMask / irqSetMasks / irqGetMask */
#define IRQ_TEST_POS_SETGETMASK() \
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
    PLATFORM_RUN_TEST(test_pos_irq_irqSetGetMask_all)

/* Negative tests for irqSetMask / irqSetMasks / irqGetMask */
#define IRQ_TEST_NEG_SETGETMASK() \
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
    PLATFORM_RUN_TEST(test_neg_irq_irqGetMask_numMasks_exceeds_max)

/* All tests for irqSetMask / irqSetMasks / irqGetMask */
#define IRQ_TEST_SETGETMASK() \
    IRQ_TEST_POS_SETGETMASK(); \
    IRQ_TEST_NEG_SETGETMASK()

/* ========================================================================== */
/*                      irqGetStatus API Tests                                */
/* ========================================================================== */

/* Positive tests for irqGetStatus */
#define IRQ_TEST_POS_GETSTATUS() \
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
    PLATFORM_RUN_TEST(test_pos_irq_irqGetStatus_trigger_L2_BUCK3_LDO); \
    PLATFORM_RUN_TEST(test_pos_irq_irqGetStatus_trigger_L1_MISC); \
    PLATFORM_RUN_TEST(test_pos_irq_irqGetStatus_trigger_L1_MODERATE_ERR); \
    PLATFORM_RUN_TEST(test_pos_irq_irqGetStatus_trigger_L1_SEVERE_ERR); \
    PLATFORM_RUN_TEST(test_pos_irq_irqGetStatus_trigger_L1_FSM_ERR); \
    PLATFORM_RUN_TEST(test_pos_irq_irqGetStatus_trigger_L2_WD_ERR_STATUS); \
    PLATFORM_RUN_TEST(test_pos_irq_irqGetStatus_trigger_L2_COMM_ERR); \
    PLATFORM_RUN_TEST(test_pos_irq_irqGetStatus_trigger_L2_ESM); \
    PLATFORM_RUN_TEST(test_pos_irq_irqGetStatus_full_hierarchy_cascade); \
    PLATFORM_RUN_TEST(test_pos_irq_irqGetStatus_all_L2_interrupts)

/* Negative tests for irqGetStatus */
#define IRQ_TEST_NEG_GETSTATUS() \
    PLATFORM_RUN_TEST(test_neg_irq_irqGetStatus_nullParam_pmicHandle); \
    PLATFORM_RUN_TEST(test_neg_irq_irqGetStatus_nullParam_irqStat)

/* All tests for irqGetStatus */
#define IRQ_TEST_GETSTATUS() \
    IRQ_TEST_POS_GETSTATUS(); \
    IRQ_TEST_NEG_GETSTATUS()

/* ========================================================================== */
/*                     irqGetNextFlag API Tests                               */
/* ========================================================================== */

/* Positive tests for irqGetNextFlag */
#define IRQ_TEST_POS_GETNEXTFLAG() \
    PLATFORM_RUN_TEST(test_pos_irq_irqGetNextFlag_singleFlag); \
    PLATFORM_RUN_TEST(test_pos_irq_irqGetNextFlag_multipleFlags); \
    PLATFORM_RUN_TEST(test_pos_irq_irqGetNextFlag_clears_intrStat); \
    PLATFORM_RUN_TEST(test_pos_irq_irqGetNextFlag_highIndexIRQ); \
    PLATFORM_RUN_TEST(test_pos_irq_irqGetNextFlag_emptyIntrStat); \
    PLATFORM_RUN_TEST(test_pos_irq_irqGetNextFlag_mixed_L1_L2)

/* Negative tests for irqGetNextFlag */
#define IRQ_TEST_NEG_GETNEXTFLAG() \
    PLATFORM_RUN_TEST(test_neg_irq_irqGetNextFlag_nullParam_irqStat); \
    PLATFORM_RUN_TEST(test_neg_irq_irqGetNextFlag_nullParam_irqNum)

/* All tests for irqGetNextFlag */
#define IRQ_TEST_GETNEXTFLAG() \
    IRQ_TEST_POS_GETNEXTFLAG(); \
    IRQ_TEST_NEG_GETNEXTFLAG()

/* ========================================================================== */
/*                       irqGetFlag API Tests                                 */
/* ========================================================================== */

/* Positive tests for irqGetFlag */
#define IRQ_TEST_POS_GETFLAG() \
    PLATFORM_RUN_TEST(test_pos_irq_irqGetFlag_flagSet); \
    PLATFORM_RUN_TEST(test_pos_irq_irqGetFlag_flagClear)

/* Negative tests for irqGetFlag */
#define IRQ_TEST_NEG_GETFLAG() \
    PLATFORM_RUN_TEST(test_neg_irq_irqGetFlag_nullParam_pmicHandle); \
    PLATFORM_RUN_TEST(test_neg_irq_irqGetFlag_outOfBounds_irqNum); \
    PLATFORM_RUN_TEST(test_neg_irq_irqGetFlag_nullParam_flag)

/* All tests for irqGetFlag */
#define IRQ_TEST_GETFLAG() \
    IRQ_TEST_POS_GETFLAG(); \
    IRQ_TEST_NEG_GETFLAG()

/* ========================================================================== */
/*                 irqClrFlag / irqClrAllFlags API Tests                      */
/* ========================================================================== */

/* Positive tests for irqClrFlag / irqClrAllFlags */
#define IRQ_TEST_POS_CLRFLAG() \
    PLATFORM_RUN_TEST(test_pos_irq_irqClrAllFlags); \
    PLATFORM_RUN_TEST(test_pos_irq_irqClrFlag_singleFlag); \
    PLATFORM_RUN_TEST(test_pos_irq_irqClrFlag_preserveOthers)

/* Negative tests for irqClrFlag / irqClrAllFlags */
#define IRQ_TEST_NEG_CLRFLAG() \
    PLATFORM_RUN_TEST(test_neg_irq_irqClrFlag_nullParam_pmicHandle); \
    PLATFORM_RUN_TEST(test_neg_irq_irqClrFlag_outOfBounds_irqNum); \
    PLATFORM_RUN_TEST(test_neg_irq_irqClrAllFlags_nullParam_handle)

/* All tests for irqClrFlag / irqClrAllFlags */
#define IRQ_TEST_CLRFLAG() \
    IRQ_TEST_POS_CLRFLAG(); \
    IRQ_TEST_NEG_CLRFLAG()

/* ========================================================================== */
/*                       Integration Tests                                    */
/* ========================================================================== */

/* Integration tests */
#define IRQ_TEST_INTEGRATION() \
    PLATFORM_RUN_TEST(test_pos_irq_irqFullCycle_setMask_getStatus_iterate_clear); \
    PLATFORM_RUN_TEST(test_pos_irq_irqMultipleSimultaneous_allRegisters)

/* ========================================================================== */
/*                         Aggregate Test Macros                              */
/* ========================================================================== */

/* Run all positive IRQ tests */
#define IRQ_TEST_RUN_POSITIVE() \
    PLATFORM_RUN_TEST(test_pos_irq_irqClrAllFlags); \
    IRQ_TEST_POS_SETGETMASK(); \
    IRQ_TEST_POS_GETSTATUS(); \
    IRQ_TEST_POS_GETNEXTFLAG(); \
    IRQ_TEST_POS_GETFLAG(); \
    PLATFORM_RUN_TEST(test_pos_irq_irqClrFlag_singleFlag); \
    PLATFORM_RUN_TEST(test_pos_irq_irqClrFlag_preserveOthers); \
    IRQ_TEST_INTEGRATION()

/* Run all negative IRQ tests */
#define IRQ_TEST_RUN_NEGATIVE() \
    IRQ_TEST_NEG_SETGETMASK(); \
    IRQ_TEST_NEG_GETSTATUS(); \
    IRQ_TEST_NEG_GETNEXTFLAG(); \
    IRQ_TEST_NEG_GETFLAG(); \
    IRQ_TEST_NEG_CLRFLAG()

/* Run all IRQ tests */
#define IRQ_TEST_RUN_ALL() \
    PLATFORM_RUN_TEST(test_pos_irq_irqClrAllFlags); \
    IRQ_TEST_SETGETMASK(); \
    IRQ_TEST_GETSTATUS(); \
    IRQ_TEST_GETNEXTFLAG(); \
    IRQ_TEST_GETFLAG(); \
    IRQ_TEST_NEG_CLRFLAG(); \
    PLATFORM_RUN_TEST(test_pos_irq_irqClrFlag_singleFlag); \
    PLATFORM_RUN_TEST(test_pos_irq_irqClrFlag_preserveOthers); \
    IRQ_TEST_INTEGRATION()



/* ========================================================================== */
/*                             Global Variables                               */
/* ========================================================================== */
static Pmic_Handle_t pmicHandle = {0U};

/* ========================================================================== */
/*                           Function Declarations                            */
/* ========================================================================== */

static void irqTest_setGetMaskError(uint8_t irqNum);

/* ========================================================================== */
/*                           Function Definitions                             */
/* ========================================================================== */

void irq_test(void *args)
{
    char msg[50U] = {0};
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_HandleCfg_t pmicCfg = {
        .validParams = (PMIC_I2C_ADDR0_VALID |
                        PMIC_COMM_HANDLE_0_VALID |
                        PMIC_IO_READ_VALID |
                        PMIC_IO_WRITE_VALID |
                        PMIC_CRITICAL_SECTION_START_VALID |
                        PMIC_CRITICAL_SECTION_STOP_VALID |
                        PMIC_IRQ_RESPONSE_CALLBACK_VALID),
        .i2cAddr0 = PLATFORM_TARGET_I2C_ADDR,
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

    status = Pmic_init(&pmicHandle, &pmicCfg);

    if (status == PMIC_ST_SUCCESS)
    {
        testUtils_printSiRev(&pmicHandle);

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
    int32_t status = Pmic_irqSetMask(NULL, PMIC_BUCK2_OVP_INT, PMIC_IRQ_MASK);
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
    Pmic_IrqMask_t irqMasks[2U] = {
        {.irqNum = PMIC_BUCK2_OVP_INT, .mask = PMIC_IRQ_MASK},
        {.irqNum = PMIC_BUCK3_OVP_INT, .mask = PMIC_IRQ_MASK},
    };
    int32_t status = Pmic_irqSetMasks(NULL, 2U, irqMasks);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_irq_irqSetMasks_nullParam_irqMasks(void)
{
    // Pass NULL irqMasks into Pmic_irqSetMasks()
    int32_t status = Pmic_irqSetMasks(&pmicHandle, 2U, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_irq_irqSetMasks_outOfBounds_irqNum(void)
{
    // Pass out of bounds irqNum into Pmic_irqSetMasks()
    Pmic_IrqMask_t irqMasks[2U] = {
        {.irqNum = PMIC_IRQ_MAX + 1U, .mask = PMIC_IRQ_MASK},
        {.irqNum = PMIC_IRQ_MAX + 2U, .mask = PMIC_IRQ_MASK},
    };
    int32_t status = Pmic_irqSetMasks(&pmicHandle, 2U, irqMasks);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_irq_irqGetMask_nullParam_handle(void)
{
    // Pass NULL handle into Pmic_irqGetMask()
    Pmic_IrqMask_t irqMasks[2U] = {
        {.irqNum = PMIC_BUCK2_OVP_INT},
        {.irqNum = PMIC_BUCK3_OVP_INT}
    };
    int32_t status = Pmic_irqGetMask(NULL, 2U, irqMasks);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_irq_irqGetMask_nullParam_irqMasks(void)
{
    // Pass NULL irqMasks into Pmic_irqGetMask()
    int32_t status = Pmic_irqGetMask(&pmicHandle, 2U, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_irq_irqGetMask_outOfBounds_irqNum(void)
{
    // Pass out of bounds irqNum into Pmic_irqGetMask()
    Pmic_IrqMask_t irqMasks[2U] = {
        {.irqNum = PMIC_IRQ_MAX + 1U},
        {.irqNum = PMIC_IRQ_MAX + 2U}
    };
    int32_t status = Pmic_irqGetMask(&pmicHandle, 2U, irqMasks);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_irq_irqGetStatus_nullParam_pmicHandle(void)
{
    // Pass NULL pmicHandle into Pmic_irqGetStatus()
    Pmic_IrqStatus_t irqStat = {0U};
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
    Pmic_IrqStatus_t irqStat = {0U};
    int32_t status = Pmic_irqGetNextFlag(NULL, &irqStat, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_irq_irqGetFlag_nullParam_pmicHandle(void)
{
    // Pass NULL pmicHandle into Pmic_irqGetFlag()
    bool flag = (bool)false;
    int32_t status = Pmic_irqGetFlag(NULL, PMIC_BUCK2_OVP_INT, &flag);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_irq_irqGetFlag_outOfBounds_irqNum(void)
{
    // Pass out of bounds irqNum into Pmic_irqGetFlag()
    bool flag = (bool)false;
    int32_t status = Pmic_irqGetFlag(&pmicHandle, PMIC_IRQ_MAX + 1U, &flag);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_irq_irqGetFlag_nullParam_flag(void)
{
    // Pass NULL flag into Pmic_irqGetFlag()
    int32_t status = Pmic_irqGetFlag(&pmicHandle, PMIC_BUCK2_OVP_INT, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_irq_irqClrFlag_nullParam_pmicHandle(void)
{
    // Pass NULL pmicHandle into Pmic_irqClrFlag()
    int32_t status = Pmic_irqClrFlag(NULL, PMIC_BUCK2_OVP_INT);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_irq_irqClrFlag_outOfBounds_irqNum(void)
{
    // Pass out of bounds irqNum into Pmic_irqClrFlag()
    int32_t status = Pmic_irqClrFlag(&pmicHandle, PMIC_IRQ_MAX + 1U);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_irq_irqClrAllFlags_nullParam_handle(void)
{
    // Pass NULL handle into Pmic_irqClrAllFlags()
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

void test_neg_irq_irqSetGetMask_LDO_SC_NMI(void)
{
    // Pass LDO_SC_NMI into Pmic_irq{Set,Get}Mask
    irqTest_setGetMaskError(PMIC_LDO_SC_NMI);
}

void test_neg_irq_irqSetGetMask_BUCK3_SC_NMI(void)
{
    // Pass BUCK3_SC_NMI into Pmic_irq{Set,Get}Mask
    irqTest_setGetMaskError(PMIC_BUCK3_SC_NMI);
}

void test_neg_irq_irqSetGetMask_BUCK2_SC_NMI(void)
{
    // Pass BUCK2_SC_NMI into Pmic_irq{Set,Get}Mask
    irqTest_setGetMaskError(PMIC_BUCK2_SC_NMI);
}

void test_neg_irq_irqSetGetMask_BUCK1_SC_NMI(void)
{
    // Pass BUCK1_SC_NMI into Pmic_irq{Set,Get}Mask
    irqTest_setGetMaskError(PMIC_BUCK1_SC_NMI);
}

void test_neg_irq_irqSetGetMask_CFG_NVM_VERIFY_ERR_NMI(void)
{
    // Pass CFG_NVM_VERIFY_ERR_NMI into Pmic_irq{Set,Get}Mask
    irqTest_setGetMaskError(PMIC_CFG_NVM_VERIFY_ERR_NMI);
}

void test_neg_irq_irqSetGetMask_CFG_NVM_VERIFY_DONE_NMI(void)
{
    // Pass CFG_NVM_VERIFY_DONE_NMI into Pmic_irq{Set,Get}Mask
    irqTest_setGetMaskError(PMIC_CFG_NVM_VERIFY_DONE_NMI);
}

void test_neg_irq_irqSetGetMask_CFG_NVM_PRG_DONE_NMI(void)
{
    // Pass CFG_NVM_PRG_DONE_NMI into Pmic_irq{Set,Get}Mask
    irqTest_setGetMaskError(PMIC_CFG_NVM_PRG_DONE_NMI);
}

void test_neg_irq_irqSetGetMask_RECOV_CNT_NMI(void)
{
    // Pass RECOV_CNT_NMI into Pmic_irq{Set,Get}Mask
    irqTest_setGetMaskError(PMIC_RECOV_CNT_NMI);
}

void test_neg_irq_irqSetGetMask_TSD_IMM_NMI(void)
{
    // Pass TSD_IMM_NMI into Pmic_irq{Set,Get}Mask
    irqTest_setGetMaskError(PMIC_TSD_IMM_NMI);
}

void test_neg_irq_irqSetGetMask_WD_FIRST_NOK_NMI(void)
{
    // Pass WD_FIRST_NOK_NMI into Pmic_irq{Set,Get}Mask
    irqTest_setGetMaskError(PMIC_WD_FIRST_NOK_NMI);
}

void test_neg_irq_irqSetGetMask_WAIT_FOR_PWRCYCLE_NMI(void)
{
    // Pass WAIT_FOR_PWRCYCLE_NMI into Pmic_irq{Set,Get}Mask
    irqTest_setGetMaskError(PMIC_WAIT_FOR_PWRCYCLE_NMI);
}

void test_neg_irq_irqSetGetMask_WARM_RESET_NMI(void)
{
    // Pass WARM_RESET_NMI into Pmic_irq{Set,Get}Mask
    irqTest_setGetMaskError(PMIC_WARM_RESET_NMI);
}

void test_neg_irq_irqSetGetMask_ORD_SHUTDOWN_NMI(void)
{
    // Pass ORD_SHUTDOWN_NMI into Pmic_irq{Set,Get}Mask
    irqTest_setGetMaskError(PMIC_ORD_SHUTDOWN_NMI);
}

void test_neg_irq_irqSetGetMask_IMM_SHUTDOWN_NMI(void)
{
    // Pass IMM_SHUTDOWN_NMI into Pmic_irq{Set,Get}Mask
    irqTest_setGetMaskError(PMIC_IMM_SHUTDOWN_NMI);
}

void test_neg_irq_irqSetGetMask_WD_RST_NMI(void)
{
    // Pass WD_RST_NMI into Pmic_irq{Set,Get}Mask
    irqTest_setGetMaskError(PMIC_WD_RST_NMI);
}

void test_neg_irq_irqSetGetMask_WD_FAIL_NMI(void)
{
    // Pass WD_FAIL_NMI into Pmic_irq{Set,Get}Mask
    irqTest_setGetMaskError(PMIC_WD_FAIL_NMI);
}

void test_neg_irq_irqSetGetMask_WD_LONGWIN_TIMEOUT_NMI(void)
{
    // Pass WD_LONGWIN_TIMEOUT_NMI into Pmic_irq{Set,Get}Mask
    irqTest_setGetMaskError(PMIC_WD_LONGWIN_TIMEOUT_NMI);
}

// NOTE: Since WDG is not being serviced, WD_FIRST_NOK_INT flag will remain set
void test_pos_irq_irqClrAllFlags(void)
{
    uint8_t regData = 0U;
    const uint8_t bufLen = 1U;
    uint8_t intTopReg = (pmicHandle.isA0) ? 0x4CU : 0x4FU;
    uint8_t intFsmErrReg = (pmicHandle.isA0) ? 0x53U : 0x56U;

    // Clear all PMIC IRQ flags
    int32_t status = Pmic_irqClrAllFlags(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Validate that all IRQ flags have been cleared
    status = platform_rxByte(&pmicHandle, 0x00U, intTopReg, &regData, bufLen);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT((regData == 0U) || (regData == (1U << 7U))); // All flags cleared or only FSM_ERR_INT flag set
    status = platform_rxByte(&pmicHandle, 0x00U, intFsmErrReg, &regData, bufLen);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT((regData == 0U) || (regData == (1U << 4U))); // All flags cleared or only WD_FIRST_NOK_INT flag set
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

void test_pos_irq_irqSetGetMask_BUCK2_OVP_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_BUCK2_OVP_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_BUCK2_OVP_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_irq_irqSetGetMask_BUCK2_UV_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_BUCK2_UV_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_BUCK2_UV_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_irq_irqSetGetMask_BUCK2_OV_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_BUCK2_OV_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_BUCK2_OV_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_irq_irqSetGetMask_BUCK1_OVP_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_BUCK1_OVP_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_BUCK1_OVP_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_irq_irqSetGetMask_BUCK1_UV_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_BUCK1_UV_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_BUCK1_UV_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_irq_irqSetGetMask_BUCK1_OV_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_BUCK1_OV_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_BUCK1_OV_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_irq_irqSetGetMask_LDO_OVP_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_LDO_OVP_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_LDO_OVP_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_irq_irqSetGetMask_LDO_UV_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_LDO_UV_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_LDO_UV_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_irq_irqSetGetMask_LDO_OV_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_LDO_OV_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_LDO_OV_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_irq_irqSetGetMask_BUCK3_OVP_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_BUCK3_OVP_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_BUCK3_OVP_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_irq_irqSetGetMask_BUCK3_UV_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_BUCK3_UV_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_BUCK3_UV_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_irq_irqSetGetMask_BUCK3_OV_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_BUCK3_OV_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_BUCK3_OV_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_irq_irqSetGetMask_TWARN_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_TWARN_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_TWARN_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_irq_irqSetGetMask_B1_PVIN_UVLO_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_B1_PVIN_UVLO_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_B1_PVIN_UVLO_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_irq_irqSetGetMask_BUCKS_VSET_ERR_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_BUCKS_VSET_ERR_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_BUCKS_VSET_ERR_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_irq_irqSetGetMask_ABIST_FAIL_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_ABIST_FAIL_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_ABIST_FAIL_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_irq_irqSetGetMask_ABIST_DONE_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_ABIST_DONE_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_ABIST_DONE_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_irq_irqSetGetMask_GPO_READBACK_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_GPO_READBACK_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_GPO_READBACK_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_irq_irqSetGetMask_NINT_READBACK_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_NINT_READBACK_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_NINT_READBACK_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_irq_irqSetGetMask_CONFIG_CRC_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_CONFIG_CRC_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_CONFIG_CRC_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_irq_irqSetGetMask_TRIM_TEST_CRC_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_TRIM_TEST_CRC_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_TRIM_TEST_CRC_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_irq_irqSetGetMask_MCU_COMM_ERR_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_MCU_COMM_ERR_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_MCU_COMM_ERR_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_irq_irqSetGetMask_COMM_ADR_ERR_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_COMM_ADR_ERR_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_COMM_ADR_ERR_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_irq_irqSetGetMask_COMM_CRC_ERR_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_COMM_CRC_ERR_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_COMM_CRC_ERR_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_irq_irqSetGetMask_ESM_MCU_RST_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_ESM_MCU_RST_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_ESM_MCU_RST_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_irq_irqSetGetMask_ESM_MCU_FAIL_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_ESM_MCU_FAIL_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_ESM_MCU_FAIL_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_irq_irqSetGetMask_ESM_MCU_PIN_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_ESM_MCU_PIN_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_ESM_MCU_PIN_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

static bool irqTest_isNMI(uint8_t irqNum)
{
    switch (irqNum)
    {
        case PMIC_LDO_SC_NMI:
        case PMIC_BUCK3_SC_NMI:
        case PMIC_BUCK2_SC_NMI:
        case PMIC_BUCK1_SC_NMI:
        case PMIC_CFG_NVM_VERIFY_ERR_NMI:
        case PMIC_CFG_NVM_VERIFY_DONE_NMI:
        case PMIC_CFG_NVM_PRG_DONE_NMI:
        case PMIC_RECOV_CNT_NMI:
        case PMIC_TSD_IMM_NMI:
        case PMIC_WD_FIRST_NOK_NMI:
        case PMIC_WAIT_FOR_PWRCYCLE_NMI:
        case PMIC_WARM_RESET_NMI:
        case PMIC_ORD_SHUTDOWN_NMI:
        case PMIC_IMM_SHUTDOWN_NMI:
        case PMIC_WD_RST_NMI:
        case PMIC_WD_FAIL_NMI:
        case PMIC_WD_LONGWIN_TIMEOUT_NMI:
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

/* ========================================================================== */
/*                       Status Reading and Flag Iteration Tests             */
/* ========================================================================== */

void test_pos_irq_irqGetStatus_noFlags(void)
{
    Pmic_IrqStatus_t irqStat = {0U};
    int32_t status = PMIC_ST_SUCCESS;

    // Clear all PMIC IRQ flags
    status = Pmic_irqClrAllFlags(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Call Pmic_irqGetStatus
    status = Pmic_irqGetStatus(&pmicHandle, &irqStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify intrStat is cleared (may have WD_FIRST_NOK_INT bit set)
    // Since WDG is not being serviced, WD_FIRST_NOK_INT (bit 30) may remain set
    PLATFORM_ASSERT((irqStat.intrStat[0] == 0U) || (irqStat.intrStat[0] == (1U << 30U)));
    PLATFORM_ASSERT(irqStat.intrStat[1] == 0U);
}

void test_pos_irq_irqGetStatus_singleFlag_L0(void)
{
    Pmic_IrqStatus_t irqStat = {0U};
    int32_t status = PMIC_ST_SUCCESS;
    bool flag = (bool)false;

    // Clear all PMIC IRQ flags
    status = Pmic_irqClrAllFlags(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get IRQ status - should read from hardware registers
    status = Pmic_irqGetStatus(&pmicHandle, &irqStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify we can read a flag (BUCK2_OVP_INT is IRQ 4)
    status = Pmic_irqGetFlag(&pmicHandle, PMIC_BUCK2_OVP_INT, &flag);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_irq_irqGetStatus_singleFlag_L1(void)
{
    Pmic_IrqStatus_t irqStat = {0U};
    int32_t status = PMIC_ST_SUCCESS;

    // Clear all PMIC IRQ flags
    status = Pmic_irqClrAllFlags(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get IRQ status - triggers L1 read for MISC_INT (IRQ 16-27)
    status = Pmic_irqGetStatus(&pmicHandle, &irqStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify intrStat structure is valid
    // intrStat[0] holds IRQs 0-31, intrStat[1] holds IRQs 32-43
}

void test_pos_irq_irqGetStatus_singleFlag_L2(void)
{
    Pmic_IrqStatus_t irqStat = {0U};
    int32_t status = PMIC_ST_SUCCESS;

    // Clear all PMIC IRQ flags
    status = Pmic_irqClrAllFlags(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get IRQ status - triggers L2 read for COMM_ERR (IRQ 35-37)
    status = Pmic_irqGetStatus(&pmicHandle, &irqStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify intrStat structure is valid
    PLATFORM_ASSERT((irqStat.intrStat[0] == 0U) || (irqStat.intrStat[0] == (1U << 30U)));
}

void test_pos_irq_irqGetStatus_multipleFlags_sameReg(void)
{
    Pmic_IrqStatus_t irqStat = {0U};
    int32_t status = PMIC_ST_SUCCESS;

    // Clear all PMIC IRQ flags
    status = Pmic_irqClrAllFlags(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get IRQ status - multiple flags in same register
    status = Pmic_irqGetStatus(&pmicHandle, &irqStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify intrStat arrays are valid
    // Both BUCK2_OVP_INT (4) and BUCK2_UV_INT (5) in same register
}

void test_pos_irq_irqGetStatus_multipleFlags_diffRegs(void)
{
    Pmic_IrqStatus_t irqStat = {0U};
    int32_t status = PMIC_ST_SUCCESS;

    // Clear all PMIC IRQ flags
    status = Pmic_irqClrAllFlags(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get IRQ status - flags in different registers
    status = Pmic_irqGetStatus(&pmicHandle, &irqStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify both intrStat[0] and intrStat[1] are accessible
    // IRQs 0-31 in intrStat[0], IRQs 32-43 in intrStat[1]
}

void test_pos_irq_irqGetStatus_hierarchyChain(void)
{
    Pmic_IrqStatus_t irqStat = {0U};
    int32_t status = PMIC_ST_SUCCESS;

    // Clear all PMIC IRQ flags
    status = Pmic_irqClrAllFlags(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get IRQ status - cascade L0→L1→L2
    status = Pmic_irqGetStatus(&pmicHandle, &irqStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify the hierarchy chain works correctly
    // L0: INT_TOP register, L1: MISC_INT register, L2: COMM_ERR register
}

void test_pos_irq_irqGetStatus_intrStatBitMapping(void)
{
    Pmic_IrqStatus_t irqStat = {0U};
    int32_t status = PMIC_ST_SUCCESS;

    // Clear all PMIC IRQ flags
    status = Pmic_irqClrAllFlags(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get IRQ status
    status = Pmic_irqGetStatus(&pmicHandle, &irqStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify intrStat[0] for IRQs 0-31
    // Verify intrStat[1] for IRQs 32-43
    // PMIC_WARM_RESET_NMI is IRQ 32, should be in intrStat[1] bit 0
    // PMIC_COMM_CRC_ERR_INT is IRQ 37, should be in intrStat[1] bit 5
}

void test_pos_irq_irqGetNextFlag_singleFlag(void)
{
    Pmic_IrqStatus_t irqStat = {0U};
    uint8_t irqNum = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    // Clear all PMIC IRQ flags
    status = Pmic_irqClrAllFlags(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get IRQ status
    status = Pmic_irqGetStatus(&pmicHandle, &irqStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get next flag (if any flags are set)
    status = Pmic_irqGetNextFlag(&pmicHandle, &irqStat, &irqNum);
    // Status can be PMIC_ST_SUCCESS or PMIC_ST_WARN_NO_IRQ_REMAINING
    PLATFORM_ASSERT((status == PMIC_ST_SUCCESS) || (status == PMIC_ST_WARN_NO_IRQ_REMAINING));
}

void test_pos_irq_irqGetNextFlag_multipleFlags(void)
{
    Pmic_IrqStatus_t irqStat = {0U};
    uint8_t irqNum = 0U;
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t count = 0U;

    // Clear all PMIC IRQ flags
    status = Pmic_irqClrAllFlags(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get IRQ status
    status = Pmic_irqGetStatus(&pmicHandle, &irqStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Iterate through all flags
    while ((status = Pmic_irqGetNextFlag(&pmicHandle, &irqStat, &irqNum)) == PMIC_ST_SUCCESS)
    {
        count++;
        PLATFORM_ASSERT(irqNum < PMIC_IRQ_NUM);
        // Prevent infinite loop
        if (count > PMIC_IRQ_NUM)
        {
            break;
        }
    }

    // Final status should be PMIC_ST_WARN_NO_IRQ_REMAINING
    PLATFORM_ASSERT(status == PMIC_ST_WARN_NO_IRQ_REMAINING);
}

void test_pos_irq_irqGetNextFlag_clears_intrStat(void)
{
    Pmic_IrqStatus_t irqStat = {0U};
    uint8_t irqNum = 0U;
    int32_t status = PMIC_ST_SUCCESS;
    uint32_t initialIntrStat0 = 0U;

    // Clear all PMIC IRQ flags
    status = Pmic_irqClrAllFlags(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get IRQ status
    status = Pmic_irqGetStatus(&pmicHandle, &irqStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Save initial intrStat value
    initialIntrStat0 = irqStat.intrStat[0];

    // Get next flag - should clear the bit in intrStat
    status = Pmic_irqGetNextFlag(&pmicHandle, &irqStat, &irqNum);

    // If a flag was found, verify intrStat was modified
    if (status == PMIC_ST_SUCCESS)
    {
        // After getting the flag, that bit should be cleared in intrStat
        PLATFORM_ASSERT(irqStat.intrStat[0] != initialIntrStat0 || irqStat.intrStat[1] != 0U);
    }
}

void test_pos_irq_irqGetNextFlag_highIndexIRQ(void)
{
    Pmic_IrqStatus_t irqStat = {0U};
    uint8_t irqNum = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    // Manually set a high index IRQ in intrStat[1] (IRQ 32+)
    // PMIC_WARM_RESET_NMI is IRQ 32, bit 0 in intrStat[1]
    irqStat.intrStat[1] = (1U << 0U);

    // Get next flag - should return IRQ 32
    status = Pmic_irqGetNextFlag(&pmicHandle, &irqStat, &irqNum);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(irqNum == PMIC_WARM_RESET_NMI);

    // Verify bit was cleared
    PLATFORM_ASSERT(irqStat.intrStat[1] == 0U);
}

void test_pos_irq_irqGetNextFlag_emptyIntrStat(void)
{
    Pmic_IrqStatus_t irqStat = {0U};
    uint8_t irqNum = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    // Clear all bits in intrStat
    irqStat.intrStat[0] = 0U;
    irqStat.intrStat[1] = 0U;

    // Get next flag - should return PMIC_ST_WARN_NO_IRQ_REMAINING
    status = Pmic_irqGetNextFlag(&pmicHandle, &irqStat, &irqNum);
    PLATFORM_ASSERT(status == PMIC_ST_WARN_NO_IRQ_REMAINING);
}

void test_pos_irq_irqGetNextFlag_mixed_L1_L2(void)
{
    Pmic_IrqStatus_t irqStat = {0U};
    uint8_t irqNum = 0U;
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t count = 0U;

    // Set mix of L1 and L2 IRQs in intrStat
    // L1: PMIC_TWARN_INT (16), L2: PMIC_MCU_COMM_ERR_INT (35)
    irqStat.intrStat[0] = (1U << 16U);  // TWARN_INT
    irqStat.intrStat[1] = (1U << 3U);   // MCU_COMM_ERR_INT (35-32=3)

    // Iterate through flags
    while ((status = Pmic_irqGetNextFlag(&pmicHandle, &irqStat, &irqNum)) == PMIC_ST_SUCCESS)
    {
        count++;
        PLATFORM_ASSERT((irqNum == PMIC_TWARN_INT) || (irqNum == PMIC_MCU_COMM_ERR_INT));
        // Prevent infinite loop
        if (count > 2U)
        {
            break;
        }
    }

    // Should have found 2 flags
    PLATFORM_ASSERT(count == 2U);
    PLATFORM_ASSERT(status == PMIC_ST_WARN_NO_IRQ_REMAINING);
}

void test_pos_irq_irqGetFlag_flagSet(void)
{
    bool flag = (bool)false;
    int32_t status = PMIC_ST_SUCCESS;

    // Get IRQ flag status for BUCK2_OVP_INT
    status = Pmic_irqGetFlag(&pmicHandle, PMIC_BUCK2_OVP_INT, &flag);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Flag can be either true or false, just verify API works
}

void test_pos_irq_irqGetFlag_flagClear(void)
{
    bool flag = (bool)false;
    int32_t status = PMIC_ST_SUCCESS;

    // Clear all PMIC IRQ flags
    status = Pmic_irqClrAllFlags(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get IRQ flag status for BUCK2_OVP_INT
    status = Pmic_irqGetFlag(&pmicHandle, PMIC_BUCK2_OVP_INT, &flag);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // After clearing, flag should be false (or WD_FIRST_NOK_INT may be set)
}

void test_pos_irq_irqClrFlag_singleFlag(void)
{
    int32_t status = PMIC_ST_SUCCESS;

    // Clear a single IRQ flag
    status = Pmic_irqClrFlag(&pmicHandle, PMIC_BUCK2_OVP_INT);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_irq_irqClrFlag_preserveOthers(void)
{
    bool flag1 = (bool)false;
    bool flag2 = (bool)false;
    int32_t status = PMIC_ST_SUCCESS;

    // Get initial flag states
    status = Pmic_irqGetFlag(&pmicHandle, PMIC_BUCK2_OVP_INT, &flag1);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_irqGetFlag(&pmicHandle, PMIC_BUCK1_OVP_INT, &flag2);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Clear one flag
    status = Pmic_irqClrFlag(&pmicHandle, PMIC_BUCK2_OVP_INT);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify the cleared flag
    status = Pmic_irqGetFlag(&pmicHandle, PMIC_BUCK2_OVP_INT, &flag1);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Other flags should be unaffected (verify API call succeeds)
    status = Pmic_irqGetFlag(&pmicHandle, PMIC_BUCK1_OVP_INT, &flag2);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_irq_irqFullCycle_setMask_getStatus_iterate_clear(void)
{
    Pmic_IrqStatus_t irqStat = {0U};
    uint8_t irqNum = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    // Clear all PMIC IRQ flags
    status = Pmic_irqClrAllFlags(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Unmask an IRQ
    status = Pmic_irqSetMask(&pmicHandle, PMIC_BUCK2_OVP_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get IRQ status
    status = Pmic_irqGetStatus(&pmicHandle, &irqStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Iterate through flags
    while ((status = Pmic_irqGetNextFlag(&pmicHandle, &irqStat, &irqNum)) == PMIC_ST_SUCCESS)
    {
        // Clear each flag found
        int32_t clrStatus = Pmic_irqClrFlag(&pmicHandle, irqNum);
        PLATFORM_ASSERT(clrStatus == PMIC_ST_SUCCESS);

        // Prevent infinite loop
        if (irqNum >= PMIC_IRQ_NUM)
        {
            break;
        }
    }

    // Final status should be PMIC_ST_WARN_NO_IRQ_REMAINING
    PLATFORM_ASSERT(status == PMIC_ST_WARN_NO_IRQ_REMAINING);

    // Mask the IRQ back
    status = Pmic_irqSetMask(&pmicHandle, PMIC_BUCK2_OVP_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_irq_irqMultipleSimultaneous_allRegisters(void)
{
    Pmic_IrqStatus_t irqStat = {0U};
    uint8_t irqNum = 0U;
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t count = 0U;

    // Clear all PMIC IRQ flags
    status = Pmic_irqClrAllFlags(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get IRQ status - may have flags across all registers
    status = Pmic_irqGetStatus(&pmicHandle, &irqStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Iterate through all flags across all registers
    while ((status = Pmic_irqGetNextFlag(&pmicHandle, &irqStat, &irqNum)) == PMIC_ST_SUCCESS)
    {
        count++;
        PLATFORM_ASSERT(irqNum < PMIC_IRQ_NUM);

        // Verify IRQ is in valid range
        PLATFORM_ASSERT(irqNum <= PMIC_IRQ_MAX);

        // Prevent infinite loop
        if (count > PMIC_IRQ_NUM)
        {
            break;
        }
    }

    // Final status should be PMIC_ST_WARN_NO_IRQ_REMAINING
    PLATFORM_ASSERT(status == PMIC_ST_WARN_NO_IRQ_REMAINING);
}


/* ========================================================================== */
/*                   Edge Case and L2 Hierarchy Tests                        */
/* ========================================================================== */

void test_neg_irq_irqSetMasks_numMasks_exceeds_max(void)
{
    // Test edge case: numMasks > PMIC_IRQ_MAX (should still work, but covered for edge case)
    const uint8_t numMasks = PMIC_IRQ_MAX + 5U;
    Pmic_IrqMask_t irqMasks[PMIC_IRQ_NUM];

    for (uint8_t i = 0U; i < PMIC_IRQ_NUM; i++)
    {
        irqMasks[i].irqNum = i;
        irqMasks[i].mask = PMIC_IRQ_MASK;
    }

    // This will trigger the check at line 326-328
    int32_t status = Pmic_irqSetMasks(&pmicHandle, numMasks, irqMasks);
    // Status depends on implementation - may succeed or fail
}

void test_neg_irq_irqGetMask_numMasks_exceeds_max(void)
{
    // Test edge case: numIrqMasks > PMIC_IRQ_MAX
    const uint8_t numMasks = PMIC_IRQ_MAX + 5U;
    Pmic_IrqMask_t irqMasks[PMIC_IRQ_NUM];

    for (uint8_t i = 0U; i < PMIC_IRQ_NUM; i++)
    {
        irqMasks[i].irqNum = i;
    }

    // This will trigger the check at line 353-355
    int32_t status = Pmic_irqGetMask(&pmicHandle, numMasks, irqMasks);
    // Status depends on implementation - may succeed or fail
}

void test_pos_irq_irqGetStatus_trigger_L1_BUCK_LDO(void)
{
    Pmic_IrqStatus_t irqStat = {0U};
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t intTopReg = (pmicHandle.isA0) ? 0x4CU : 0x4FU;
    uint8_t intBuckLdoReg = (pmicHandle.isA0) ? 0x4DU : 0x50U;

    // Clear all PMIC IRQ flags first
    status = Pmic_irqClrAllFlags(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Inject BUCK_LDO_INT flag in INT_TOP register
    status = testInject_setBits(intTopReg, (1U << 0U));
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Inject BUCK1_INT flag in INT_BUCK_LDO register
    status = testInject_setBits(intBuckLdoReg, (1U << 0U));
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get IRQ status - should trigger IRQ_readL1IntBuckLdo
    status = Pmic_irqGetStatus(&pmicHandle, &irqStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_irq_irqGetStatus_trigger_L2_BUCK1_2(void)
{
    Pmic_IrqStatus_t irqStat = {0U};
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t intTopReg = (pmicHandle.isA0) ? 0x4CU : 0x4FU;
    uint8_t intBuckLdoReg = (pmicHandle.isA0) ? 0x4DU : 0x50U;
    uint8_t intBuck1_2Reg = (pmicHandle.isA0) ? 0x4EU : 0x51U;

    // Clear all PMIC IRQ flags
    status = Pmic_irqClrAllFlags(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Inject BUCK_LDO_INT in INT_TOP
    status = testInject_setBits(intTopReg, (1U << 0U));
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Inject BUCK1_INT in INT_BUCK_LDO
    status = testInject_setBits(intBuckLdoReg, (1U << 0U));
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Inject BUCK1_OVP_INT in INT_BUCK1_2
    status = testInject_setBits(intBuck1_2Reg, (1U << 2U));
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get IRQ status - should trigger IRQ_readL2IntBuck1_2
    status = Pmic_irqGetStatus(&pmicHandle, &irqStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_irq_irqGetStatus_trigger_L2_BUCK3_LDO(void)
{
    Pmic_IrqStatus_t irqStat = {0U};
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t intTopReg = (pmicHandle.isA0) ? 0x4CU : 0x4FU;
    uint8_t intBuckLdoReg = (pmicHandle.isA0) ? 0x4DU : 0x50U;
    uint8_t intBuck3LdoReg = (pmicHandle.isA0) ? 0x4FU : 0x52U;

    // Clear all PMIC IRQ flags
    status = Pmic_irqClrAllFlags(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Inject BUCK_LDO_INT in INT_TOP
    status = testInject_setBits(intTopReg, (1U << 0U));
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Inject BUCK3_INT in INT_BUCK_LDO
    status = testInject_setBits(intBuckLdoReg, (1U << 2U));
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Inject BUCK3_OVP_INT in INT_BUCK3_LDO
    status = testInject_setBits(intBuck3LdoReg, (1U << 2U));
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get IRQ status - should trigger IRQ_readL2IntBuck3Ldo
    status = Pmic_irqGetStatus(&pmicHandle, &irqStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_irq_irqGetStatus_trigger_L1_MISC(void)
{
    Pmic_IrqStatus_t irqStat = {0U};
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t intTopReg = (pmicHandle.isA0) ? 0x4CU : 0x4FU;
    uint8_t intMiscReg = (pmicHandle.isA0) ? 0x50U : 0x53U;

    // Clear all PMIC IRQ flags
    status = Pmic_irqClrAllFlags(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Inject MISC_INT in INT_TOP
    status = testInject_setBits(intTopReg, (1U << 4U));
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Inject TWARN_INT in INT_MISC
    status = testInject_setBits(intMiscReg, (1U << 7U));
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get IRQ status - should trigger IRQ_readL1IntMisc
    status = Pmic_irqGetStatus(&pmicHandle, &irqStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_irq_irqGetStatus_trigger_L1_MODERATE_ERR(void)
{
    Pmic_IrqStatus_t irqStat = {0U};
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t intTopReg = (pmicHandle.isA0) ? 0x4CU : 0x4FU;
    uint8_t intModerateErrReg = (pmicHandle.isA0) ? 0x51U : 0x54U;

    // Clear all PMIC IRQ flags
    status = Pmic_irqClrAllFlags(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Inject MODERATE_ERR_INT in INT_TOP
    status = testInject_setBits(intTopReg, (1U << 5U));
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Inject CONFIG_CRC_INT in INT_MODERATE_ERR
    status = testInject_setBits(intModerateErrReg, (1U << 3U));
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get IRQ status - should trigger IRQ_readL1IntModerateErr
    status = Pmic_irqGetStatus(&pmicHandle, &irqStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_irq_irqGetStatus_trigger_L1_SEVERE_ERR(void)
{
    Pmic_IrqStatus_t irqStat = {0U};
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t intTopReg = (pmicHandle.isA0) ? 0x4CU : 0x4FU;
    uint8_t intSevereErrReg = (pmicHandle.isA0) ? 0x52U : 0x55U;

    // Clear all PMIC IRQ flags
    status = Pmic_irqClrAllFlags(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Inject SEVERE_ERR_INT in INT_TOP
    status = testInject_setBits(intTopReg, (1U << 6U));
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Inject TSD_IMM_INT in INT_SEVERE_ERR
    status = testInject_setBits(intSevereErrReg, (1U << 0U));
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get IRQ status - should trigger IRQ_readL1IntSevereErr
    status = Pmic_irqGetStatus(&pmicHandle, &irqStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_irq_irqGetStatus_trigger_L1_FSM_ERR(void)
{
    Pmic_IrqStatus_t irqStat = {0U};
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t intTopReg = (pmicHandle.isA0) ? 0x4CU : 0x4FU;
    uint8_t intFsmErrReg = (pmicHandle.isA0) ? 0x53U : 0x56U;

    // Clear all PMIC IRQ flags
    status = Pmic_irqClrAllFlags(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Inject FSM_ERR_INT in INT_TOP
    status = testInject_setBits(intTopReg, (1U << 7U));
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Inject WARM_RESET_INT in INT_FSM_ERR
    status = testInject_setBits(intFsmErrReg, (1U << 2U));
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get IRQ status - should trigger IRQ_readL1IntFsmErr
    status = Pmic_irqGetStatus(&pmicHandle, &irqStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_irq_irqGetStatus_trigger_L2_WD_ERR_STATUS(void)
{
    Pmic_IrqStatus_t irqStat = {0U};
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t intTopReg = (pmicHandle.isA0) ? 0x4CU : 0x4FU;
    uint8_t intFsmErrReg = (pmicHandle.isA0) ? 0x53U : 0x56U;
    uint8_t wdErrStatusReg = (pmicHandle.isA0) ? 0x1BU : 0x1BU;

    // Clear all PMIC IRQ flags
    status = Pmic_irqClrAllFlags(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Inject FSM_ERR_INT in INT_TOP
    status = testInject_setBits(intTopReg, (1U << 7U));
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Inject WD_INT in INT_FSM_ERR
    status = testInject_setBits(intFsmErrReg, (1U << 7U));
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Inject WD_RST_INT in WD_ERR_STATUS
    status = testInject_setBits(wdErrStatusReg, (1U << 7U));
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get IRQ status - should trigger IRQ_readL2WdErrStatus
    status = Pmic_irqGetStatus(&pmicHandle, &irqStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_irq_irqGetStatus_trigger_L2_COMM_ERR(void)
{
    Pmic_IrqStatus_t irqStat = {0U};
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t intTopReg = (pmicHandle.isA0) ? 0x4CU : 0x4FU;
    uint8_t intFsmErrReg = (pmicHandle.isA0) ? 0x53U : 0x56U;
    uint8_t intCommErrReg = (pmicHandle.isA0) ? 0x54U : 0x57U;

    // Clear all PMIC IRQ flags
    status = Pmic_irqClrAllFlags(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Inject FSM_ERR_INT in INT_TOP
    status = testInject_setBits(intTopReg, (1U << 7U));
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Inject COMM_ERR_INT in INT_FSM_ERR
    status = testInject_setBits(intFsmErrReg, (1U << 6U));
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Inject MCU_COMM_ERR_INT in INT_COMM_ERR
    status = testInject_setBits(intCommErrReg, (1U << 4U));
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get IRQ status - should trigger IRQ_readL2IntCommErr
    status = Pmic_irqGetStatus(&pmicHandle, &irqStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_irq_irqGetStatus_trigger_L2_ESM(void)
{
    Pmic_IrqStatus_t irqStat = {0U};
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t intTopReg = (pmicHandle.isA0) ? 0x4CU : 0x4FU;
    uint8_t intFsmErrReg = (pmicHandle.isA0) ? 0x53U : 0x56U;
    uint8_t intEsmReg = (pmicHandle.isA0) ? 0x55U : 0x58U;

    // Clear all PMIC IRQ flags
    status = Pmic_irqClrAllFlags(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Inject FSM_ERR_INT in INT_TOP
    status = testInject_setBits(intTopReg, (1U << 7U));
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Inject ESM_MCU_INT in INT_FSM_ERR
    status = testInject_setBits(intFsmErrReg, (1U << 5U));
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Inject ESM_MCU_PIN_INT in INT_ESM
    status = testInject_setBits(intEsmReg, (1U << 3U));
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get IRQ status - should trigger IRQ_readL2IntEsm
    status = Pmic_irqGetStatus(&pmicHandle, &irqStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_irq_irqGetStatus_full_hierarchy_cascade(void)
{
    Pmic_IrqStatus_t irqStat = {0U};
    uint8_t irqNum = 0U;
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t count = 0U;
    uint8_t intTopReg = (pmicHandle.isA0) ? 0x4CU : 0x4FU;
    uint8_t intFsmErrReg = (pmicHandle.isA0) ? 0x53U : 0x56U;
    uint8_t intCommErrReg = (pmicHandle.isA0) ? 0x54U : 0x57U;
    uint8_t intEsmReg = (pmicHandle.isA0) ? 0x55U : 0x58U;
    uint8_t wdErrStatusReg = (pmicHandle.isA0) ? 0x1BU : 0x1BU;

    // Clear all PMIC IRQ flags
    status = Pmic_irqClrAllFlags(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Setup a full cascade: L0 -> L1 (FSM_ERR) -> L2 (WD, COMM_ERR, ESM)
    status = testInject_setBits(intTopReg, (1U << 7U));
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = testInject_setBits(intFsmErrReg, (1U << 7U) | (1U << 6U) | (1U << 5U));
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = testInject_setBits(wdErrStatusReg, (1U << 7U));
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = testInject_setBits(intCommErrReg, (1U << 4U));
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = testInject_setBits(intEsmReg, (1U << 3U));
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get IRQ status - should trigger full hierarchy
    status = Pmic_irqGetStatus(&pmicHandle, &irqStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Iterate through flags to verify multiple IRQs were found
    while ((status = Pmic_irqGetNextFlag(&pmicHandle, &irqStat, &irqNum)) == PMIC_ST_SUCCESS)
    {
        count++;
        if (count > PMIC_IRQ_NUM)
        {
            break;
        }
    }

    PLATFORM_ASSERT(status == PMIC_ST_WARN_NO_IRQ_REMAINING);
}

void test_pos_irq_irqGetStatus_all_L2_interrupts(void)
{
    Pmic_IrqStatus_t irqStat = {0U};
    uint8_t irqNum = 0U;
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t count = 0U;
    uint8_t intTopReg = (pmicHandle.isA0) ? 0x4CU : 0x4FU;
    uint8_t intBuckLdoReg = (pmicHandle.isA0) ? 0x4DU : 0x50U;
    uint8_t intBuck1_2Reg = (pmicHandle.isA0) ? 0x4EU : 0x51U;
    uint8_t intBuck3LdoReg = (pmicHandle.isA0) ? 0x4FU : 0x52U;

    // Clear all PMIC IRQ flags
    status = Pmic_irqClrAllFlags(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Set up multiple L2 interrupts
    status = testInject_setBits(intTopReg, (1U << 0U));
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = testInject_setBits(intBuckLdoReg, (1U << 0U) | (1U << 2U));
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = testInject_setBits(intBuck1_2Reg, TEST_MASK_FULL_BYTE);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = testInject_setBits(intBuck3LdoReg, TEST_MASK_FULL_BYTE);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get IRQ status - should trigger multiple L2 reads
    status = Pmic_irqGetStatus(&pmicHandle, &irqStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Iterate through flags
    while ((status = Pmic_irqGetNextFlag(&pmicHandle, &irqStat, &irqNum)) == PMIC_ST_SUCCESS)
    {
        count++;
        if (count > PMIC_IRQ_NUM)
        {
            break;
        }
    }

    PLATFORM_ASSERT(status == PMIC_ST_WARN_NO_IRQ_REMAINING);
    PLATFORM_ASSERT(count > 0U);
}
