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
/**
 * @file platform.c
 * @brief Source file containing definitions to PMIC IRQ tests.
 */

/* ========================================================================== */
/*                              Include Files                                 */
/* ========================================================================== */

#include "irq_test.h"

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/* Run all IRQ tests */
#define IRQ_TEST_RUN_ALL() PLATFORM_RUN_TEST(test_negative_Pmic_irqSetMask_nullParam_handle); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_irqSetMask_outOfBounds_irqNum); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_irqSetMasks_nullParam_handle); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_irqSetMasks_nullParam_irqMasks); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_irqSetMasks_outOfBounds_irqNum); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_irqGetMask_nullParam_handle); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_irqGetMask_nullParam_irqMasks); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_irqGetMask_outOfBounds_irqNum); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_irqGetStat_nullParam_pmicHandle); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_irqGetStat_nullParam_irqStat); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_irqGetNextFlag_nullParam_irqStat); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_irqGetNextFlag_nullParam_irqNum); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_irqGetFlag_nullParam_pmicHandle); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_irqGetFlag_outOfBounds_irqNum); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_irqGetFlag_nullParam_flag); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_irqClrFlag_nullParam_pmicHandle); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_irqClrFlag_outOfBounds_irqNum); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_irqClrAllFlags_nullParam_handle); \
                           PLATFORM_RUN_TEST(test_negative_irqSetGetMask_LDO_SC_NMI); \
                           PLATFORM_RUN_TEST(test_negative_irqSetGetMask_BUCK3_SC_NMI); \
                           PLATFORM_RUN_TEST(test_negative_irqSetGetMask_BUCK2_SC_NMI); \
                           PLATFORM_RUN_TEST(test_negative_irqSetGetMask_BUCK1_SC_NMI); \
                           PLATFORM_RUN_TEST(test_negative_irqSetGetMask_CFG_NVM_VERIFY_ERR_NMI); \
                           PLATFORM_RUN_TEST(test_negative_irqSetGetMask_CFG_NVM_VERIFY_DONE_NMI); \
                           PLATFORM_RUN_TEST(test_negative_irqSetGetMask_CFG_NVM_PRG_DONE_NMI); \
                           PLATFORM_RUN_TEST(test_negative_irqSetGetMask_RECOV_CNT_NMI); \
                           PLATFORM_RUN_TEST(test_negative_irqSetGetMask_TSD_IMM_NMI); \
                           PLATFORM_RUN_TEST(test_negative_irqSetGetMask_WD_FIRST_NOK_NMI); \
                           PLATFORM_RUN_TEST(test_negative_irqSetGetMask_WAIT_FOR_PWRCYCLE_NMI); \
                           PLATFORM_RUN_TEST(test_negative_irqSetGetMask_WARM_RESET_NMI); \
                           PLATFORM_RUN_TEST(test_negative_irqSetGetMask_ORD_SHUTDOWN_NMI); \
                           PLATFORM_RUN_TEST(test_negative_irqSetGetMask_IMM_SHUTDOWN_NMI); \
                           PLATFORM_RUN_TEST(test_negative_irqSetGetMask_WD_RST_NMI); \
                           PLATFORM_RUN_TEST(test_negative_irqSetGetMask_WD_FAIL_NMI); \
                           PLATFORM_RUN_TEST(test_positive_irqClrAllFlags); \
                           PLATFORM_RUN_TEST(test_positive_irqSetGetMask_BUCK2_OVP_INT); \
                           PLATFORM_RUN_TEST(test_positive_irqSetGetMask_BUCK2_UV_INT); \
                           PLATFORM_RUN_TEST(test_positive_irqSetGetMask_BUCK2_OV_INT); \
                           PLATFORM_RUN_TEST(test_positive_irqSetGetMask_BUCK1_OVP_INT); \
                           PLATFORM_RUN_TEST(test_positive_irqSetGetMask_BUCK1_UV_INT); \
                           PLATFORM_RUN_TEST(test_positive_irqSetGetMask_BUCK1_OV_INT); \
                           PLATFORM_RUN_TEST(test_positive_irqSetGetMask_LDO_OVP_INT); \
                           PLATFORM_RUN_TEST(test_positive_irqSetGetMask_LDO_UV_INT); \
                           PLATFORM_RUN_TEST(test_positive_irqSetGetMask_LDO_OV_INT); \
                           PLATFORM_RUN_TEST(test_positive_irqSetGetMask_BUCK3_OVP_INT); \
                           PLATFORM_RUN_TEST(test_positive_irqSetGetMask_BUCK3_UV_INT); \
                           PLATFORM_RUN_TEST(test_positive_irqSetGetMask_BUCK3_OV_INT); \
                           PLATFORM_RUN_TEST(test_positive_irqSetGetMask_TWARN_INT); \
                           PLATFORM_RUN_TEST(test_positive_irqSetGetMask_B1_PVIN_UVLO_INT); \
                           PLATFORM_RUN_TEST(test_positive_irqSetGetMask_BUCKS_VSET_ERR_INT); \
                           PLATFORM_RUN_TEST(test_positive_irqSetGetMask_ABIST_FAIL_INT); \
                           PLATFORM_RUN_TEST(test_positive_irqSetGetMask_ABIST_DONE_INT); \
                           PLATFORM_RUN_TEST(test_positive_irqSetGetMask_GPO_READBACK_INT); \
                           PLATFORM_RUN_TEST(test_positive_irqSetGetMask_NINT_READBACK_INT); \
                           PLATFORM_RUN_TEST(test_positive_irqSetGetMask_CONFIG_CRC_INT); \
                           PLATFORM_RUN_TEST(test_positive_irqSetGetMask_TRIM_TEST_CRC_INT); \
                           PLATFORM_RUN_TEST(test_positive_irqSetGetMask_MCU_COMM_ERR_INT); \
                           PLATFORM_RUN_TEST(test_positive_irqSetGetMask_COMM_ADR_ERR_INT); \
                           PLATFORM_RUN_TEST(test_positive_irqSetGetMask_COMM_CRC_ERR_INT); \
                           PLATFORM_RUN_TEST(test_positive_irqSetGetMask_ESM_MCU_RST_INT); \
                           PLATFORM_RUN_TEST(test_positive_irqSetGetMask_ESM_MCU_FAIL_INT); \
                           PLATFORM_RUN_TEST(test_positive_irqSetGetMask_ESM_MCU_PIN_INT); \
                           PLATFORM_RUN_TEST(test_positive_irqSetGetMask_all); \
                           PLATFORM_RUN_TEST(test_negative_irqSetGetMask_WD_LONGWIN_TIMEOUT_NMI)

/* Run all IRQ negative tests */
#define IRQ_TEST_RUN_NEGATIVE() PLATFORM_RUN_TEST(test_negative_Pmic_irqSetMask_nullParam_handle); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_irqSetMask_outOfBounds_irqNum); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_irqSetMasks_nullParam_handle); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_irqSetMasks_nullParam_irqMasks); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_irqSetMasks_outOfBounds_irqNum); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_irqGetMask_nullParam_handle); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_irqGetMask_nullParam_irqMasks); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_irqGetMask_outOfBounds_irqNum); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_irqGetStat_nullParam_pmicHandle); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_irqGetStat_nullParam_irqStat); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_irqGetNextFlag_nullParam_irqStat); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_irqGetNextFlag_nullParam_irqNum); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_irqGetFlag_nullParam_pmicHandle); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_irqGetFlag_outOfBounds_irqNum); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_irqGetFlag_nullParam_flag); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_irqClrFlag_nullParam_pmicHandle); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_irqClrFlag_outOfBounds_irqNum); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_irqClrAllFlags_nullParam_handle); \
                                PLATFORM_RUN_TEST(test_negative_irqSetGetMask_LDO_SC_NMI); \
                                PLATFORM_RUN_TEST(test_negative_irqSetGetMask_BUCK3_SC_NMI); \
                                PLATFORM_RUN_TEST(test_negative_irqSetGetMask_BUCK2_SC_NMI); \
                                PLATFORM_RUN_TEST(test_negative_irqSetGetMask_BUCK1_SC_NMI); \
                                PLATFORM_RUN_TEST(test_negative_irqSetGetMask_CFG_NVM_VERIFY_ERR_NMI); \
                                PLATFORM_RUN_TEST(test_negative_irqSetGetMask_CFG_NVM_VERIFY_DONE_NMI); \
                                PLATFORM_RUN_TEST(test_negative_irqSetGetMask_CFG_NVM_PRG_DONE_NMI); \
                                PLATFORM_RUN_TEST(test_negative_irqSetGetMask_RECOV_CNT_NMI); \
                                PLATFORM_RUN_TEST(test_negative_irqSetGetMask_TSD_IMM_NMI); \
                                PLATFORM_RUN_TEST(test_negative_irqSetGetMask_WD_FIRST_NOK_NMI); \
                                PLATFORM_RUN_TEST(test_negative_irqSetGetMask_WAIT_FOR_PWRCYCLE_NMI); \
                                PLATFORM_RUN_TEST(test_negative_irqSetGetMask_WARM_RESET_NMI); \
                                PLATFORM_RUN_TEST(test_negative_irqSetGetMask_ORD_SHUTDOWN_NMI); \
                                PLATFORM_RUN_TEST(test_negative_irqSetGetMask_IMM_SHUTDOWN_NMI); \
                                PLATFORM_RUN_TEST(test_negative_irqSetGetMask_WD_RST_NMI); \
                                PLATFORM_RUN_TEST(test_negative_irqSetGetMask_WD_FAIL_NMI); \
                                PLATFORM_RUN_TEST(test_negative_irqSetGetMask_WD_LONGWIN_TIMEOUT_NMI)

// Run all IRQ positive tests
#define IRQ_TEST_RUN_POSITIVE() PLATFORM_RUN_TEST(test_positive_irqClrAllFlags); \
                                PLATFORM_RUN_TEST(test_positive_irqSetGetMask_BUCK2_OVP_INT); \
                                PLATFORM_RUN_TEST(test_positive_irqSetGetMask_BUCK2_UV_INT); \
                                PLATFORM_RUN_TEST(test_positive_irqSetGetMask_BUCK2_OV_INT); \
                                PLATFORM_RUN_TEST(test_positive_irqSetGetMask_BUCK1_OVP_INT); \
                                PLATFORM_RUN_TEST(test_positive_irqSetGetMask_BUCK1_UV_INT); \
                                PLATFORM_RUN_TEST(test_positive_irqSetGetMask_BUCK1_OV_INT); \
                                PLATFORM_RUN_TEST(test_positive_irqSetGetMask_LDO_OVP_INT); \
                                PLATFORM_RUN_TEST(test_positive_irqSetGetMask_LDO_UV_INT); \
                                PLATFORM_RUN_TEST(test_positive_irqSetGetMask_LDO_OV_INT); \
                                PLATFORM_RUN_TEST(test_positive_irqSetGetMask_BUCK3_OVP_INT); \
                                PLATFORM_RUN_TEST(test_positive_irqSetGetMask_BUCK3_UV_INT); \
                                PLATFORM_RUN_TEST(test_positive_irqSetGetMask_BUCK3_OV_INT); \
                                PLATFORM_RUN_TEST(test_positive_irqSetGetMask_TWARN_INT); \
                                PLATFORM_RUN_TEST(test_positive_irqSetGetMask_B1_PVIN_UVLO_INT); \
                                PLATFORM_RUN_TEST(test_positive_irqSetGetMask_BUCKS_VSET_ERR_INT); \
                                PLATFORM_RUN_TEST(test_positive_irqSetGetMask_ABIST_FAIL_INT); \
                                PLATFORM_RUN_TEST(test_positive_irqSetGetMask_ABIST_DONE_INT); \
                                PLATFORM_RUN_TEST(test_positive_irqSetGetMask_GPO_READBACK_INT); \
                                PLATFORM_RUN_TEST(test_positive_irqSetGetMask_NINT_READBACK_INT); \
                                PLATFORM_RUN_TEST(test_positive_irqSetGetMask_CONFIG_CRC_INT); \
                                PLATFORM_RUN_TEST(test_positive_irqSetGetMask_TRIM_TEST_CRC_INT); \
                                PLATFORM_RUN_TEST(test_positive_irqSetGetMask_MCU_COMM_ERR_INT); \
                                PLATFORM_RUN_TEST(test_positive_irqSetGetMask_COMM_ADR_ERR_INT); \
                                PLATFORM_RUN_TEST(test_positive_irqSetGetMask_COMM_CRC_ERR_INT); \
                                PLATFORM_RUN_TEST(test_positive_irqSetGetMask_ESM_MCU_RST_INT); \
                                PLATFORM_RUN_TEST(test_positive_irqSetGetMask_ESM_MCU_FAIL_INT); \
                                PLATFORM_RUN_TEST(test_positive_irqSetGetMask_ESM_MCU_PIN_INT); \
                                PLATFORM_RUN_TEST(test_positive_irqSetGetMask_all)



/* ========================================================================== */
/*                             Global Variables                               */
/* ========================================================================== */
static Pmic_CoreHandle_t pmicHandle = {0U};

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
    Pmic_CoreCfg_t pmicCfg = {
        .i2cAddr = PLATFORM_TARGET_I2C_ADDR,
        .commHandle = platform_getCommHandle(),
        .ioRead = &platform_rxByte,
        .ioWrite = &platform_txByte,
        .critSecStart = &platform_critSecStart,
        .critSecStop = &platform_critSecStop,
        .irqResponse = &platform_irqResponse
    };

    platform_init();

    platform_printString("\r\n");
    platform_printString("IRQ_TEST\r\n");
    platform_printString("-------\r\n\r\n");

    status = Pmic_init(&pmicCfg, &pmicHandle);

    if (status == PMIC_ST_SUCCESS)
    {
        status = testCommon_unlockPmicRegs(&pmicHandle);

        if (status == PMIC_ST_SUCCESS)
        {
            platform_setupTests();
            IRQ_TEST_RUN_ALL();
            platform_tearDownTests();
        }
        else
        {
            (void)sprintf(msg, "Error in unlocking PMIC registers: %d\r\n", status);
            platform_printString(msg);
        }
    }
    else
    {
        (void)sprintf(msg, "Error in initializing PMIC LLD: %d\r\n", status);
        platform_printString(msg);
    }

    (void)Pmic_deinit(&pmicHandle);
    platform_deinit();
}

void test_negative_Pmic_irqSetMask_nullParam_handle(void)
{
    // Pass NULL handle into Pmic_irqSetMask()
    int32_t status = Pmic_irqSetMask(NULL, PMIC_BUCK2_OVP_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_irqSetMask_outOfBounds_irqNum(void)
{
    // Pass out of bounds irqNum into Pmic_irqSetMask()
    int32_t status = Pmic_irqSetMask(&pmicHandle, PMIC_IRQ_MAX + 1U, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_negative_Pmic_irqSetMasks_nullParam_handle(void)
{
    // Pass NULL handle into Pmic_irqSetMasks()
    Pmic_IrqMask_t irqMasks[2U] = {
        {.irqNum = PMIC_BUCK2_OVP_INT, .mask = PMIC_IRQ_MASK},
        {.irqNum = PMIC_BUCK3_OVP_INT, .mask = PMIC_IRQ_MASK},
    };
    int32_t status = Pmic_irqSetMasks(NULL, 2U, irqMasks);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_irqSetMasks_nullParam_irqMasks(void)
{
    // Pass NULL irqMasks into Pmic_irqSetMasks()
    int32_t status = Pmic_irqSetMasks(&pmicHandle, 2U, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_irqSetMasks_outOfBounds_irqNum(void)
{
    // Pass out of bounds irqNum into Pmic_irqSetMasks()
    Pmic_IrqMask_t irqMasks[2U] = {
        {.irqNum = PMIC_IRQ_MAX + 1U, .mask = PMIC_IRQ_MASK},
        {.irqNum = PMIC_IRQ_MAX + 2U, .mask = PMIC_IRQ_MASK},
    };
    int32_t status = Pmic_irqSetMasks(&pmicHandle, 2U, irqMasks);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_negative_Pmic_irqGetMask_nullParam_handle(void)
{
    // Pass NULL handle into Pmic_irqGetMask()
    Pmic_IrqMask_t irqMasks[2U] = {
        {.irqNum = PMIC_BUCK2_OVP_INT},
        {.irqNum = PMIC_BUCK3_OVP_INT}
    };
    int32_t status = Pmic_irqGetMask(NULL, 2U, irqMasks);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_irqGetMask_nullParam_irqMasks(void)
{
    // Pass NULL irqMasks into Pmic_irqGetMask()
    int32_t status = Pmic_irqGetMask(&pmicHandle, 2U, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_irqGetMask_outOfBounds_irqNum(void)
{
    // Pass out of bounds irqNum into Pmic_irqGetMask()
    Pmic_IrqMask_t irqMasks[2U] = {
        {.irqNum = PMIC_IRQ_MAX + 1U},
        {.irqNum = PMIC_IRQ_MAX + 2U}
    };
    int32_t status = Pmic_irqGetMask(&pmicHandle, 2U, irqMasks);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_negative_Pmic_irqGetStat_nullParam_pmicHandle(void)
{
    // Pass NULL pmicHandle into Pmic_irqGetStat()
    Pmic_IrqStat_t irqStat = {0U};
    int32_t status = Pmic_irqGetStat(NULL, &irqStat);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_irqGetStat_nullParam_irqStat(void)
{
    // Pass NULL irqStat into Pmic_irqGetStat()
    int32_t status = Pmic_irqGetStat(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_irqGetNextFlag_nullParam_irqStat(void)
{
    // Pass NULL irqStat into Pmic_irqGetNextFlag()
    uint8_t irqNum = 0U;
    int32_t status = Pmic_irqGetNextFlag(NULL, &irqNum);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_irqGetNextFlag_nullParam_irqNum(void)
{
    // Pass NULL irqNum into Pmic_irqGetNextFlag()
    Pmic_IrqStat_t irqStat = {0U};
    int32_t status = Pmic_irqGetNextFlag(&irqStat, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_irqGetFlag_nullParam_pmicHandle(void)
{
    // Pass NULL pmicHandle into Pmic_irqGetFlag()
    bool flag = (bool)false;
    int32_t status = Pmic_irqGetFlag(NULL, PMIC_BUCK2_OVP_INT, &flag);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_irqGetFlag_outOfBounds_irqNum(void)
{
    // Pass out of bounds irqNum into Pmic_irqGetFlag()
    bool flag = (bool)false;
    int32_t status = Pmic_irqGetFlag(&pmicHandle, PMIC_IRQ_MAX + 1U, &flag);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_negative_Pmic_irqGetFlag_nullParam_flag(void)
{
    // Pass NULL flag into Pmic_irqGetFlag()
    int32_t status = Pmic_irqGetFlag(&pmicHandle, PMIC_BUCK2_OVP_INT, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_irqClrFlag_nullParam_pmicHandle(void)
{
    // Pass NULL pmicHandle into Pmic_irqClrFlag()
    int32_t status = Pmic_irqClrFlag(NULL, PMIC_BUCK2_OVP_INT);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_irqClrFlag_outOfBounds_irqNum(void)
{
    // Pass out of bounds irqNum into Pmic_irqClrFlag()
    int32_t status = Pmic_irqClrFlag(&pmicHandle, PMIC_IRQ_MAX + 1U);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_negative_Pmic_irqClrAllFlags_nullParam_handle(void)
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

void test_negative_irqSetGetMask_LDO_SC_NMI(void)
{
    // Pass LDO_SC_NMI into Pmic_irq{Set,Get}Mask
    irqTest_setGetMaskError(PMIC_LDO_SC_NMI);
}

void test_negative_irqSetGetMask_BUCK3_SC_NMI(void)
{
    // Pass BUCK3_SC_NMI into Pmic_irq{Set,Get}Mask
    irqTest_setGetMaskError(PMIC_BUCK3_SC_NMI);
}

void test_negative_irqSetGetMask_BUCK2_SC_NMI(void)
{
    // Pass BUCK2_SC_NMI into Pmic_irq{Set,Get}Mask
    irqTest_setGetMaskError(PMIC_BUCK2_SC_NMI);
}

void test_negative_irqSetGetMask_BUCK1_SC_NMI(void)
{
    // Pass BUCK1_SC_NMI into Pmic_irq{Set,Get}Mask
    irqTest_setGetMaskError(PMIC_BUCK1_SC_NMI);
}

void test_negative_irqSetGetMask_CFG_NVM_VERIFY_ERR_NMI(void)
{
    // Pass CFG_NVM_VERIFY_ERR_NMI into Pmic_irq{Set,Get}Mask
    irqTest_setGetMaskError(PMIC_CFG_NVM_VERIFY_ERR_NMI);
}

void test_negative_irqSetGetMask_CFG_NVM_VERIFY_DONE_NMI(void)
{
    // Pass CFG_NVM_VERIFY_DONE_NMI into Pmic_irq{Set,Get}Mask
    irqTest_setGetMaskError(PMIC_CFG_NVM_VERIFY_DONE_NMI);
}

void test_negative_irqSetGetMask_CFG_NVM_PRG_DONE_NMI(void)
{
    // Pass CFG_NVM_PRG_DONE_NMI into Pmic_irq{Set,Get}Mask
    irqTest_setGetMaskError(PMIC_CFG_NVM_PRG_DONE_NMI);
}

void test_negative_irqSetGetMask_RECOV_CNT_NMI(void)
{
    // Pass RECOV_CNT_NMI into Pmic_irq{Set,Get}Mask
    irqTest_setGetMaskError(PMIC_RECOV_CNT_NMI);
}

void test_negative_irqSetGetMask_TSD_IMM_NMI(void)
{
    // Pass TSD_IMM_NMI into Pmic_irq{Set,Get}Mask
    irqTest_setGetMaskError(PMIC_TSD_IMM_NMI);
}

void test_negative_irqSetGetMask_WD_FIRST_NOK_NMI(void)
{
    // Pass WD_FIRST_NOK_NMI into Pmic_irq{Set,Get}Mask
    irqTest_setGetMaskError(PMIC_WD_FIRST_NOK_NMI);
}

void test_negative_irqSetGetMask_WAIT_FOR_PWRCYCLE_NMI(void)
{
    // Pass WAIT_FOR_PWRCYCLE_NMI into Pmic_irq{Set,Get}Mask
    irqTest_setGetMaskError(PMIC_WAIT_FOR_PWRCYCLE_NMI);
}

void test_negative_irqSetGetMask_WARM_RESET_NMI(void)
{
    // Pass WARM_RESET_NMI into Pmic_irq{Set,Get}Mask
    irqTest_setGetMaskError(PMIC_WARM_RESET_NMI);
}

void test_negative_irqSetGetMask_ORD_SHUTDOWN_NMI(void)
{
    // Pass ORD_SHUTDOWN_NMI into Pmic_irq{Set,Get}Mask
    irqTest_setGetMaskError(PMIC_ORD_SHUTDOWN_NMI);
}

void test_negative_irqSetGetMask_IMM_SHUTDOWN_NMI(void)
{
    // Pass IMM_SHUTDOWN_NMI into Pmic_irq{Set,Get}Mask
    irqTest_setGetMaskError(PMIC_IMM_SHUTDOWN_NMI);
}

void test_negative_irqSetGetMask_WD_RST_NMI(void)
{
    // Pass WD_RST_NMI into Pmic_irq{Set,Get}Mask
    irqTest_setGetMaskError(PMIC_WD_RST_NMI);
}

void test_negative_irqSetGetMask_WD_FAIL_NMI(void)
{
    // Pass WD_FAIL_NMI into Pmic_irq{Set,Get}Mask
    irqTest_setGetMaskError(PMIC_WD_FAIL_NMI);
}

void test_negative_irqSetGetMask_WD_LONGWIN_TIMEOUT_NMI(void)
{
    // Pass WD_LONGWIN_TIMEOUT_NMI into Pmic_irq{Set,Get}Mask
    irqTest_setGetMaskError(PMIC_WD_LONGWIN_TIMEOUT_NMI);
}

// NOTE: Since WDG is not being serviced, WD_FIRST_NOK_INT flag will remain set
void test_positive_irqClrAllFlags(void)
{
    uint8_t regData = 0U;
    const uint8_t intTopReg = 0x4FU, intFsmErrReg = 0x56U, bufLen = 1U;

    // Clear all PMIC IRQ flags
    int32_t status = Pmic_irqClrAllFlags(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Validate that all IRQ flags have been cleared
    status = platform_rxByte(&pmicHandle, intTopReg, bufLen, &regData);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT((regData == 0U) || (regData == (1U << 7U))); // All flags cleared or only FSM_ERR_INT flag set
    status = platform_rxByte(&pmicHandle, intFsmErrReg, bufLen, &regData);
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

void test_positive_irqSetGetMask_BUCK2_OVP_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_BUCK2_OVP_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_BUCK2_OVP_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_positive_irqSetGetMask_BUCK2_UV_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_BUCK2_UV_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_BUCK2_UV_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_positive_irqSetGetMask_BUCK2_OV_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_BUCK2_OV_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_BUCK2_OV_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_positive_irqSetGetMask_BUCK1_OVP_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_BUCK1_OVP_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_BUCK1_OVP_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_positive_irqSetGetMask_BUCK1_UV_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_BUCK1_UV_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_BUCK1_UV_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_positive_irqSetGetMask_BUCK1_OV_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_BUCK1_OV_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_BUCK1_OV_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_positive_irqSetGetMask_LDO_OVP_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_LDO_OVP_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_LDO_OVP_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_positive_irqSetGetMask_LDO_UV_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_LDO_UV_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_LDO_UV_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_positive_irqSetGetMask_LDO_OV_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_LDO_OV_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_LDO_OV_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_positive_irqSetGetMask_BUCK3_OVP_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_BUCK3_OVP_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_BUCK3_OVP_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_positive_irqSetGetMask_BUCK3_UV_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_BUCK3_UV_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_BUCK3_UV_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_positive_irqSetGetMask_BUCK3_OV_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_BUCK3_OV_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_BUCK3_OV_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_positive_irqSetGetMask_TWARN_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_TWARN_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_TWARN_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_positive_irqSetGetMask_B1_PVIN_UVLO_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_B1_PVIN_UVLO_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_B1_PVIN_UVLO_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_positive_irqSetGetMask_BUCKS_VSET_ERR_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_BUCKS_VSET_ERR_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_BUCKS_VSET_ERR_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_positive_irqSetGetMask_ABIST_FAIL_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_ABIST_FAIL_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_ABIST_FAIL_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_positive_irqSetGetMask_ABIST_DONE_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_ABIST_DONE_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_ABIST_DONE_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_positive_irqSetGetMask_GPO_READBACK_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_GPO_READBACK_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_GPO_READBACK_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_positive_irqSetGetMask_NINT_READBACK_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_NINT_READBACK_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_NINT_READBACK_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_positive_irqSetGetMask_CONFIG_CRC_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_CONFIG_CRC_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_CONFIG_CRC_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_positive_irqSetGetMask_TRIM_TEST_CRC_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_TRIM_TEST_CRC_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_TRIM_TEST_CRC_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_positive_irqSetGetMask_MCU_COMM_ERR_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_MCU_COMM_ERR_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_MCU_COMM_ERR_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_positive_irqSetGetMask_COMM_ADR_ERR_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_COMM_ADR_ERR_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_COMM_ADR_ERR_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_positive_irqSetGetMask_COMM_CRC_ERR_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_COMM_CRC_ERR_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_COMM_CRC_ERR_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_positive_irqSetGetMask_ESM_MCU_RST_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_ESM_MCU_RST_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_ESM_MCU_RST_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_positive_irqSetGetMask_ESM_MCU_FAIL_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_ESM_MCU_FAIL_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_ESM_MCU_FAIL_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_positive_irqSetGetMask_ESM_MCU_PIN_INT(void)
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

void test_positive_irqSetGetMask_all(void)
{
    int32_t status = irqTest_setGetMaskAll(PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMaskAll(PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

/**
 * @brief Some testing frameworks require an API to setup tests. Rename/rewrite
 * as necessary.
 */
void setUp(void)
{
}

/**
 * @brief Some testing frameworks require an API to teardown tests.
 * Rename/rewrite as necessary.
 */
void tearDown(void)
{
}
