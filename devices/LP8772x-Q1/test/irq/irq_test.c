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
                           PLATFORM_RUN_TEST(test_negative_Pmic_irqSetMasks_outOfBounds_numIrqMasks); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_irqSetMasks_outOfBounds_irqNum); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_irqGetMask_nullParam_handle); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_irqGetMask_nullParam_irqMasks); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_irqGetMask_outOfBounds_numIrqMasks); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_irqGetMask_outOfBounds_irqNum); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_irqGetStat_nullParam_handle); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_irqGetStat_nullParam_irqStat); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_irqGetNextFlag_nullParam_irqStat); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_irqGetNextFlag_nullParam_irqNum); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_irqGetFlag_nullParam_handle); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_irqGetFlag_nullParam_flag); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_irqGetFlag_outOfBounds_irqNum); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_irqClrFlag_nullParam_handle); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_irqClrFlag_outOfBounds_irqNum); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_irqClrAllFlags_nullParam_handle); \
                           PLATFORM_RUN_TEST(test_negative_irqSetGetMask_BUCK1_SC_NMI); \
                           PLATFORM_RUN_TEST(test_negative_irqSetGetMask_BUCK2_SC_NMI); \
                           PLATFORM_RUN_TEST(test_negative_irqSetGetMask_BUCK3_SC_NMI); \
                           PLATFORM_RUN_TEST(test_negative_irqSetGetMask_LDO_LS1_VMON1_SC_NMI); \
                           PLATFORM_RUN_TEST(test_negative_irqSetGetMask_LS2_VMON2_SC_NMI); \
                           PLATFORM_RUN_TEST(test_negative_irqSetGetMask_TSD_ORD_NMI); \
                           PLATFORM_RUN_TEST(test_negative_irqSetGetMask_RECOV_CNT_NMI); \
                           PLATFORM_RUN_TEST(test_negative_irqSetGetMask_TSD_IMM_NMI); \
                           PLATFORM_RUN_TEST(test_negative_irqSetGetMask_VCCA_OVP_NMI); \
                           PLATFORM_RUN_TEST(test_negative_irqSetGetMask_WDG_FIRST_NOK_NMI); \
                           PLATFORM_RUN_TEST(test_negative_irqSetGetMask_WDG_LONGWIN_TIMEOUT_NMI); \
                           PLATFORM_RUN_TEST(test_negative_irqSetGetMask_WDG_TIMEOUT_NMI); \
                           PLATFORM_RUN_TEST(test_negative_irqSetGetMask_WDG_ANSWER_EARLY_NMI); \
                           PLATFORM_RUN_TEST(test_negative_irqSetGetMask_WDG_SEQ_ERR_NMI); \
                           PLATFORM_RUN_TEST(test_negative_irqSetGetMask_WDG_ANSWER_ERR_NMI); \
                           PLATFORM_RUN_TEST(test_negative_irqSetGetMask_WDG_FAIL_NMI); \
                           PLATFORM_RUN_TEST(test_negative_irqSetGetMask_WDG_RST_NMI); \
                           PLATFORM_RUN_TEST(test_negative_irqSetGetMask_REGULATOR_ERR_NMI); \
                           PLATFORM_RUN_TEST(test_negative_irqSetGetMask_FSM_IMM_SHUTDOWN_NMI); \
                           PLATFORM_RUN_TEST(test_negative_irqSetGetMask_FSM_ORD_SHUTDOWN_NMI); \
                           PLATFORM_RUN_TEST(test_negative_irqSetGetMask_FSM_WARM_RESET_NMI); \
                           PLATFORM_RUN_TEST(test_positive_irqClrAllFlags); \
                           PLATFORM_RUN_TEST(test_positive_irqSetGetMask_BUCK1_OV_INT); \
                           PLATFORM_RUN_TEST(test_positive_irqSetGetMask_BUCK1_UV_INT); \
                           PLATFORM_RUN_TEST(test_positive_irqSetGetMask_BUCK1_RV_INT); \
                           PLATFORM_RUN_TEST(test_positive_irqSetGetMask_BUCK1_ILIM_INT); \
                           PLATFORM_RUN_TEST(test_positive_irqSetGetMask_BUCK2_OV_INT); \
                           PLATFORM_RUN_TEST(test_positive_irqSetGetMask_BUCK2_UV_INT); \
                           PLATFORM_RUN_TEST(test_positive_irqSetGetMask_BUCK2_RV_INT); \
                           PLATFORM_RUN_TEST(test_positive_irqSetGetMask_BUCK2_ILIM_INT); \
                           PLATFORM_RUN_TEST(test_positive_irqSetGetMask_BUCK3_OV_INT); \
                           PLATFORM_RUN_TEST(test_positive_irqSetGetMask_BUCK3_UV_INT); \
                           PLATFORM_RUN_TEST(test_positive_irqSetGetMask_BUCK3_RV_INT); \
                           PLATFORM_RUN_TEST(test_positive_irqSetGetMask_BUCK3_ILIM_INT); \
                           PLATFORM_RUN_TEST(test_positive_irqSetGetMask_LDO_LS1_VMON1_OV_INT); \
                           PLATFORM_RUN_TEST(test_positive_irqSetGetMask_LDO_LS1_VMON1_UV_INT); \
                           PLATFORM_RUN_TEST(test_positive_irqSetGetMask_LDO_LS1_VMON1_RV_INT); \
                           PLATFORM_RUN_TEST(test_positive_irqSetGetMask_LDO_LS1_VMON1_ILIM_INT); \
                           PLATFORM_RUN_TEST(test_positive_irqSetGetMask_LS2_VMON2_OV_INT); \
                           PLATFORM_RUN_TEST(test_positive_irqSetGetMask_LS2_VMON2_UV_INT); \
                           PLATFORM_RUN_TEST(test_positive_irqSetGetMask_LS2_VMON2_RV_INT); \
                           PLATFORM_RUN_TEST(test_positive_irqSetGetMask_LS2_VMON2_ILIM_INT); \
                           PLATFORM_RUN_TEST(test_positive_irqSetGetMask_VCCA_OV_INT); \
                           PLATFORM_RUN_TEST(test_positive_irqSetGetMask_VCCA_UV_INT); \
                           PLATFORM_RUN_TEST(test_positive_irqSetGetMask_STARTUP_ENABLE_INT); \
                           PLATFORM_RUN_TEST(test_positive_irqSetGetMask_ABIST_FAIL_INT); \
                           PLATFORM_RUN_TEST(test_positive_irqSetGetMask_BUCKS_VSET_ERR_INT); \
                           PLATFORM_RUN_TEST(test_positive_irqSetGetMask_EXT_CLK_INT); \
                           PLATFORM_RUN_TEST(test_positive_irqSetGetMask_TWARN_INT); \
                           PLATFORM_RUN_TEST(test_positive_irqSetGetMask_TRIM_TEST_CRC_INT); \
                           PLATFORM_RUN_TEST(test_positive_irqSetGetMask_CONFIG_CRC_INT); \
                           PLATFORM_RUN_TEST(test_positive_irqSetGetMask_NINT_READBACK_INT); \
                           PLATFORM_RUN_TEST(test_positive_irqSetGetMask_NRSTOUT_READBACK_INT); \
                           PLATFORM_RUN_TEST(test_positive_irqSetGetMask_COMM_FRM_ERR_INT); \
                           PLATFORM_RUN_TEST(test_positive_irqSetGetMask_COMM_CRC_ERR_INT); \
                           PLATFORM_RUN_TEST(test_positive_irqSetGetMask_COMM_ADR_ERR_INT); \
                           PLATFORM_RUN_TEST(test_positive_irqSetGetMask_COMM_MCU_ERR_INT); \
                           PLATFORM_RUN_TEST(test_positive_irqSetGetMask_ESM_MCU_PIN_INT); \
                           PLATFORM_RUN_TEST(test_positive_irqSetGetMask_ESM_MCU_FAIL_INT); \
                           PLATFORM_RUN_TEST(test_positive_irqSetGetMask_ESM_MCU_RST_INT); \
                           PLATFORM_RUN_TEST(test_positive_irqSetGetMask_all); \
                           PLATFORM_RUN_TEST(test_positive_irqGetClrFlag)

/* Run all IRQ negative tests */
#define IRQ_TEST_RUN_NEGATIVE() PLATFORM_RUN_TEST(test_negative_Pmic_irqSetMask_nullParam_handle); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_irqSetMask_outOfBounds_irqNum); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_irqSetMasks_nullParam_handle); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_irqSetMasks_nullParam_irqMasks); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_irqSetMasks_outOfBounds_numIrqMasks); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_irqSetMasks_outOfBounds_irqNum); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_irqGetMask_nullParam_handle); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_irqGetMask_nullParam_irqMasks); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_irqGetMask_outOfBounds_numIrqMasks); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_irqGetMask_outOfBounds_irqNum); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_irqGetStat_nullParam_handle); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_irqGetStat_nullParam_irqStat); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_irqGetNextFlag_nullParam_irqStat); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_irqGetNextFlag_nullParam_irqNum); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_irqGetFlag_nullParam_handle); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_irqGetFlag_nullParam_flag); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_irqGetFlag_outOfBounds_irqNum); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_irqClrFlag_nullParam_handle); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_irqClrFlag_outOfBounds_irqNum); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_irqClrAllFlags_nullParam_handle); \
                                PLATFORM_RUN_TEST(test_negative_irqSetGetMask_BUCK1_SC_NMI); \
                                PLATFORM_RUN_TEST(test_negative_irqSetGetMask_BUCK2_SC_NMI); \
                                PLATFORM_RUN_TEST(test_negative_irqSetGetMask_BUCK3_SC_NMI); \
                                PLATFORM_RUN_TEST(test_negative_irqSetGetMask_LDO_LS1_VMON1_SC_NMI); \
                                PLATFORM_RUN_TEST(test_negative_irqSetGetMask_LS2_VMON2_SC_NMI); \
                                PLATFORM_RUN_TEST(test_negative_irqSetGetMask_TSD_ORD_NMI); \
                                PLATFORM_RUN_TEST(test_negative_irqSetGetMask_RECOV_CNT_NMI); \
                                PLATFORM_RUN_TEST(test_negative_irqSetGetMask_TSD_IMM_NMI); \
                                PLATFORM_RUN_TEST(test_negative_irqSetGetMask_VCCA_OVP_NMI); \
                                PLATFORM_RUN_TEST(test_negative_irqSetGetMask_WDG_FIRST_NOK_NMI); \
                                PLATFORM_RUN_TEST(test_negative_irqSetGetMask_WDG_LONGWIN_TIMEOUT_NMI); \
                                PLATFORM_RUN_TEST(test_negative_irqSetGetMask_WDG_TIMEOUT_NMI); \
                                PLATFORM_RUN_TEST(test_negative_irqSetGetMask_WDG_ANSWER_EARLY_NMI); \
                                PLATFORM_RUN_TEST(test_negative_irqSetGetMask_WDG_SEQ_ERR_NMI); \
                                PLATFORM_RUN_TEST(test_negative_irqSetGetMask_WDG_ANSWER_ERR_NMI); \
                                PLATFORM_RUN_TEST(test_negative_irqSetGetMask_WDG_FAIL_NMI); \
                                PLATFORM_RUN_TEST(test_negative_irqSetGetMask_WDG_RST_NMI); \
                                PLATFORM_RUN_TEST(test_negative_irqSetGetMask_REGULATOR_ERR_NMI); \
                                PLATFORM_RUN_TEST(test_negative_irqSetGetMask_FSM_IMM_SHUTDOWN_NMI); \
                                PLATFORM_RUN_TEST(test_negative_irqSetGetMask_FSM_ORD_SHUTDOWN_NMI); \
                                PLATFORM_RUN_TEST(test_negative_irqSetGetMask_FSM_WARM_RESET_NMI)

// Run all IRQ positive tests
#define IRQ_TEST_RUN_POSITIVE() PLATFORM_RUN_TEST(test_positive_irqClrAllFlags); \
                                PLATFORM_RUN_TEST(test_positive_irqSetGetMask_BUCK1_OV_INT); \
                                PLATFORM_RUN_TEST(test_positive_irqSetGetMask_BUCK1_UV_INT); \
                                PLATFORM_RUN_TEST(test_positive_irqSetGetMask_BUCK1_RV_INT); \
                                PLATFORM_RUN_TEST(test_positive_irqSetGetMask_BUCK1_ILIM_INT); \
                                PLATFORM_RUN_TEST(test_positive_irqSetGetMask_BUCK2_OV_INT); \
                                PLATFORM_RUN_TEST(test_positive_irqSetGetMask_BUCK2_UV_INT); \
                                PLATFORM_RUN_TEST(test_positive_irqSetGetMask_BUCK2_RV_INT); \
                                PLATFORM_RUN_TEST(test_positive_irqSetGetMask_BUCK2_ILIM_INT); \
                                PLATFORM_RUN_TEST(test_positive_irqSetGetMask_BUCK3_OV_INT); \
                                PLATFORM_RUN_TEST(test_positive_irqSetGetMask_BUCK3_UV_INT); \
                                PLATFORM_RUN_TEST(test_positive_irqSetGetMask_BUCK3_RV_INT); \
                                PLATFORM_RUN_TEST(test_positive_irqSetGetMask_BUCK3_ILIM_INT); \
                                PLATFORM_RUN_TEST(test_positive_irqSetGetMask_LDO_LS1_VMON1_OV_INT); \
                                PLATFORM_RUN_TEST(test_positive_irqSetGetMask_LDO_LS1_VMON1_UV_INT); \
                                PLATFORM_RUN_TEST(test_positive_irqSetGetMask_LDO_LS1_VMON1_RV_INT); \
                                PLATFORM_RUN_TEST(test_positive_irqSetGetMask_LDO_LS1_VMON1_ILIM_INT); \
                                PLATFORM_RUN_TEST(test_positive_irqSetGetMask_LS2_VMON2_OV_INT); \
                                PLATFORM_RUN_TEST(test_positive_irqSetGetMask_LS2_VMON2_UV_INT); \
                                PLATFORM_RUN_TEST(test_positive_irqSetGetMask_LS2_VMON2_RV_INT); \
                                PLATFORM_RUN_TEST(test_positive_irqSetGetMask_LS2_VMON2_ILIM_INT); \
                                PLATFORM_RUN_TEST(test_positive_irqSetGetMask_VCCA_OV_INT); \
                                PLATFORM_RUN_TEST(test_positive_irqSetGetMask_VCCA_UV_INT); \
                                PLATFORM_RUN_TEST(test_positive_irqSetGetMask_STARTUP_ENABLE_INT); \
                                PLATFORM_RUN_TEST(test_positive_irqSetGetMask_ABIST_FAIL_INT); \
                                PLATFORM_RUN_TEST(test_positive_irqSetGetMask_BUCKS_VSET_ERR_INT); \
                                PLATFORM_RUN_TEST(test_positive_irqSetGetMask_EXT_CLK_INT); \
                                PLATFORM_RUN_TEST(test_positive_irqSetGetMask_TWARN_INT); \
                                PLATFORM_RUN_TEST(test_positive_irqSetGetMask_TRIM_TEST_CRC_INT); \
                                PLATFORM_RUN_TEST(test_positive_irqSetGetMask_CONFIG_CRC_INT); \
                                PLATFORM_RUN_TEST(test_positive_irqSetGetMask_NINT_READBACK_INT); \
                                PLATFORM_RUN_TEST(test_positive_irqSetGetMask_NRSTOUT_READBACK_INT); \
                                PLATFORM_RUN_TEST(test_positive_irqSetGetMask_COMM_FRM_ERR_INT); \
                                PLATFORM_RUN_TEST(test_positive_irqSetGetMask_COMM_CRC_ERR_INT); \
                                PLATFORM_RUN_TEST(test_positive_irqSetGetMask_COMM_ADR_ERR_INT); \
                                PLATFORM_RUN_TEST(test_positive_irqSetGetMask_COMM_MCU_ERR_INT); \
                                PLATFORM_RUN_TEST(test_positive_irqSetGetMask_ESM_MCU_PIN_INT); \
                                PLATFORM_RUN_TEST(test_positive_irqSetGetMask_ESM_MCU_FAIL_INT); \
                                PLATFORM_RUN_TEST(test_positive_irqSetGetMask_ESM_MCU_RST_INT); \
                                PLATFORM_RUN_TEST(test_positive_irqSetGetMask_all); \
                                PLATFORM_RUN_TEST(test_positive_irqGetClrFlag)

/* ========================================================================== */
/*                             Global Variables                               */
/* ========================================================================== */
static Pmic_Handle_t pmicHandle;

/* ========================================================================== */
/*                           Function Declarations                            */
/* ========================================================================== */
static int32_t irqTest_unlockPmicRegs(Pmic_Handle_t *pmicHandle);
static void irqTest_setGetMaskError(uint8_t irqNum);
static int32_t irqTest_setGetMask(uint8_t irqNum, bool shouldMask);

/* ========================================================================== */
/*                           Function Definitions                             */
/* ========================================================================== */

void irq_test(void *args)
{
    char msg[50U] = {0};
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_CoreCfg_t coreCfg = {
        .validParams = (PMIC_CFG_DEVICE_TYPE_VALID |
                        PMIC_CFG_COMM_MODE_VALID |
                        PMIC_CFG_SLAVEADDR_VALID |
                        PMIC_CFG_COMM_HANDLE_VALID |
                        PMIC_CFG_COMM_IO_RD_VALID |
                        PMIC_CFG_COMM_IO_WR_VALID |
                        PMIC_CFG_CRITSEC_START_VALID |
                        PMIC_CFG_CRITSEC_STOP_VALID |
                        PMIC_CFG_CRC_ENABLE_VALID |
                        PMIC_CFG_CFG_CRC_ENABLE_VALID |
                        PMIC_CFG_PSEUDO_IRQ_VALID),
        .instType = PMIC_MAIN_INST,
        .pmicDeviceType = PLATFORM_TARGET_DEV_TYPE,
        .commMode = PMIC_INTF_I2C_SINGLE,
        .slaveAddr = PLATFORM_TARGET_I2C_ADDR,
        .crcEnable = PMIC_DISABLE,
        .configCrcEnable = PMIC_DISABLE,
        .pCommHandle = platform_getCommHandle(),
        .pFnPmicCommIoRd = &platform_rxByte,
        .pFnPmicCommIoWr = &platform_txByte,
        .pFnPmicCritSecStart = &platform_critSecStart,
        .pFnPmicCritSecStop = &platform_critSecStop,
        .irqResponseCallback = &platform_irqResponse
    };

    platform_init();

    platform_printString("\r\n");
    platform_printString("IRQ_TEST\r\n");
    platform_printString("-------\r\n\r\n");

    status = Pmic_init(&pmicHandle, &coreCfg);

    if (status == PMIC_ST_SUCCESS)
    {
        status = irqTest_unlockPmicRegs(&pmicHandle);

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

static int32_t irqTest_unlockPmicRegs(Pmic_Handle_t *pmicHandle)
{
    uint8_t regData = 0x9BU;
    const uint8_t bufLen = 1U;
    const uint16_t registerLockAddr = 0x09U;

    // Check handle
    int32_t status = Pmic_checkPmicCoreHandle(pmicHandle);

    // Write key to REGISTER_LOCK
    if (status == PMIC_ST_SUCCESS)
    {
        status = platform_txByte(pmicHandle, PMIC_MAIN_INST, registerLockAddr, &regData, bufLen);
    }

    // Get register lock status
    if (status == PMIC_ST_SUCCESS)
    {
        status = platform_rxByte(pmicHandle, PMIC_MAIN_INST, registerLockAddr, &regData, bufLen);
    }

    // Validate that registers are unlocked
    if ((status == PMIC_ST_SUCCESS) && (regData != 0U))
    {
        status = PMIC_ST_ERR_I2C_COMM_FAIL;
    }

    return status;
}

void test_negative_Pmic_irqSetMask_nullParam_handle(void)
{
    // Pass NULL handle into Pmic_irqSetMask()
    int32_t status = Pmic_irqSetMask(NULL, PMIC_BUCK1_OV_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_HANDLE);
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
    Pmic_IrqMask_t irqMask = {
        .irqNum = PMIC_BUCK1_OV_INT,
        .mask = PMIC_IRQ_MASK
    };
    int32_t status = Pmic_irqSetMasks(NULL, 1U, &irqMask);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_HANDLE);
}

void test_negative_Pmic_irqSetMasks_nullParam_irqMasks(void)
{
    // Pass NULL irqMasks into Pmic_irqSetMasks()
    int32_t status = Pmic_irqSetMasks(&pmicHandle, 1U, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_irqSetMasks_outOfBounds_numIrqMasks(void)
{
    // Pass out of bounds numIrqMasks into Pmic_irqSetMasks()
    Pmic_IrqMask_t irqMask = {
        .irqNum = PMIC_BUCK1_OV_INT,
        .mask = PMIC_IRQ_MASK
    };
    int32_t status = Pmic_irqSetMasks(&pmicHandle, PMIC_IRQ_NUM + 1U, &irqMask);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_negative_Pmic_irqSetMasks_outOfBounds_irqNum(void)
{
    // Pass out of bounds irqNum into Pmic_irqSetMasks()
    Pmic_IrqMask_t irqMask = {
        .irqNum = PMIC_IRQ_MAX + 1U,
        .mask = PMIC_IRQ_MASK
    };
    int32_t status = Pmic_irqSetMasks(&pmicHandle, 1U, &irqMask);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_negative_Pmic_irqGetMask_nullParam_handle(void)
{
    // Pass NULL handle into Pmic_irqGetMask()
    Pmic_IrqMask_t irqMask = {
        .irqNum = PMIC_BUCK1_OV_INT,
        .mask = PMIC_IRQ_MASK
    };
    int32_t status = Pmic_irqGetMask(NULL, 1U, &irqMask);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_HANDLE);
}

void test_negative_Pmic_irqGetMask_nullParam_irqMasks(void)
{
    // Pass NULL irqMasks into Pmic_irqGetMask()
    int32_t status = Pmic_irqGetMask(&pmicHandle, 1U, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_irqGetMask_outOfBounds_numIrqMasks(void)
{
    // Pass out of bounds numIrqMasks into Pmic_irqGetMask()
    Pmic_IrqMask_t irqMask = {
        .irqNum = PMIC_BUCK1_OV_INT,
        .mask = PMIC_IRQ_MASK
    };
    int32_t status = Pmic_irqGetMask(&pmicHandle, PMIC_IRQ_NUM + 1U, &irqMask);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_negative_Pmic_irqGetMask_outOfBounds_irqNum(void)
{
    // Pass out of bounds irqNum into Pmic_irqGetMask()
    Pmic_IrqMask_t irqMask = {
        .irqNum = PMIC_IRQ_MAX + 1U,
        .mask = PMIC_IRQ_MASK
    };
    int32_t status = Pmic_irqGetMask(&pmicHandle, 1U, &irqMask);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_negative_Pmic_irqGetStat_nullParam_handle(void)
{
    // Pass NULL handle into Pmic_irqGetStat()
    Pmic_IrqStat_t irqStat = {0U};
    int32_t status = Pmic_irqGetStat(NULL, &irqStat);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_HANDLE);
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

void test_negative_Pmic_irqGetFlag_nullParam_handle(void)
{
    // Pass NULL handle into Pmic_irqGetFlag()
    bool flag = (bool)false;
    int32_t status = Pmic_irqGetFlag(NULL, PMIC_BUCK1_OV_INT, &flag);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_HANDLE);
}

void test_negative_Pmic_irqGetFlag_nullParam_flag(void)
{
    // Pass NULL flag into Pmic_irqGetFlag()
    int32_t status = Pmic_irqGetFlag(&pmicHandle, PMIC_BUCK1_OV_INT, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_irqGetFlag_outOfBounds_irqNum(void)
{
    // Pass out of bounds irqNum into Pmic_irqGetFlag()
    bool flag = (bool)false;
    int32_t status = Pmic_irqGetFlag(&pmicHandle, PMIC_IRQ_MAX + 1U, &flag);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_negative_Pmic_irqClrFlag_nullParam_handle(void)
{
    // Pass NULL handle into Pmic_irqClrFlag()
    int32_t status = Pmic_irqClrFlag(NULL, PMIC_BUCK1_OV_INT);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_HANDLE);
}

void test_negative_Pmic_irqClrFlag_outOfBounds_irqNum(void)
{
    // Pass out of bounds irqNum into Pmic_irqClrFlag
    int32_t status = Pmic_irqClrFlag(&pmicHandle, PMIC_IRQ_MAX + 1U);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_negative_Pmic_irqClrAllFlags_nullParam_handle(void)
{
    // Pass NULL handle into Pmic_irqClrAllFlags
    int32_t status = Pmic_irqClrAllFlags(NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_HANDLE);
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

void test_negative_irqSetGetMask_BUCK1_SC_NMI(void)
{
    irqTest_setGetMaskError(PMIC_BUCK1_SC_NMI);
}

void test_negative_irqSetGetMask_BUCK2_SC_NMI(void)
{
    irqTest_setGetMaskError(PMIC_BUCK2_SC_NMI);
}

void test_negative_irqSetGetMask_BUCK3_SC_NMI(void)
{
    irqTest_setGetMaskError(PMIC_BUCK3_SC_NMI);
}

void test_negative_irqSetGetMask_LDO_LS1_VMON1_SC_NMI(void)
{
    irqTest_setGetMaskError(PMIC_LDO_LS1_VMON1_SC_NMI);
}

void test_negative_irqSetGetMask_LS2_VMON2_SC_NMI(void)
{
    irqTest_setGetMaskError(PMIC_LS2_VMON2_SC_NMI);
}

void test_negative_irqSetGetMask_TSD_ORD_NMI(void)
{
    irqTest_setGetMaskError(PMIC_ME_TSD_ORD_NMI);
}

void test_negative_irqSetGetMask_RECOV_CNT_NMI(void)
{
    irqTest_setGetMaskError(PMIC_ME_RECOV_CNT_NMI);
}

void test_negative_irqSetGetMask_TSD_IMM_NMI(void)
{
    irqTest_setGetMaskError(PMIC_SE_TSD_IMM_NMI);
}

void test_negative_irqSetGetMask_VCCA_OVP_NMI(void)
{
    irqTest_setGetMaskError(PMIC_SE_VCCA_OVP_NMI);
}

void test_negative_irqSetGetMask_WDG_FIRST_NOK_NMI(void)
{
    irqTest_setGetMaskError(PMIC_WDG_FIRST_NOK_NMI);
}

void test_negative_irqSetGetMask_WDG_LONGWIN_TIMEOUT_NMI(void)
{
    irqTest_setGetMaskError(PMIC_WDG_LONGWIN_TIMEOUT_NMI);
}

void test_negative_irqSetGetMask_WDG_TIMEOUT_NMI(void)
{
    irqTest_setGetMaskError(PMIC_WDG_TIMEOUT_NMI);
}

void test_negative_irqSetGetMask_WDG_ANSWER_EARLY_NMI(void)
{
    irqTest_setGetMaskError(PMIC_WDG_ANSWER_EARLY_NMI);
}

void test_negative_irqSetGetMask_WDG_SEQ_ERR_NMI(void)
{
    irqTest_setGetMaskError(PMIC_WDG_SEQ_ERR_NMI);
}

void test_negative_irqSetGetMask_WDG_ANSWER_ERR_NMI(void)
{
    irqTest_setGetMaskError(PMIC_WDG_ANSWER_ERR_NMI);
}

void test_negative_irqSetGetMask_WDG_FAIL_NMI(void)
{
    irqTest_setGetMaskError(PMIC_WDG_FAIL_NMI);
}

void test_negative_irqSetGetMask_WDG_RST_NMI(void)
{
    irqTest_setGetMaskError(PMIC_WDG_RST_NMI);
}

void test_negative_irqSetGetMask_REGULATOR_ERR_NMI(void)
{
    irqTest_setGetMaskError(PMIC_REGULATOR_ERR_NMI);
}

void test_negative_irqSetGetMask_FSM_IMM_SHUTDOWN_NMI(void)
{
    irqTest_setGetMaskError(PMIC_FSM_IMM_SHUTDOWN_NMI);
}

void test_negative_irqSetGetMask_FSM_ORD_SHUTDOWN_NMI(void)
{
    irqTest_setGetMaskError(PMIC_FSM_ORD_SHUTDOWN_NMI);
}

void test_negative_irqSetGetMask_FSM_WARM_RESET_NMI(void)
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
void test_positive_irqClrAllFlags(void)
{
    uint8_t regData = 0U;
    const uint8_t intTopReg = 0x46U, intFsmErrReg = 0x50U, bufLen = 1U;

    // Clear all PMIC IRQ flags
    int32_t status = Pmic_irqClrAllFlags(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Validate that all IRQ flags have been cleared
    status = platform_rxByte(&pmicHandle, PMIC_MAIN_INST, intTopReg, &regData, bufLen);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT((regData == 0U) || (regData == 128U)); // All flags cleared or only FSM_ERR_INT flag set
    status = platform_rxByte(&pmicHandle, PMIC_MAIN_INST, intFsmErrReg, &regData, bufLen);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT((regData == 0U) || (regData == 16U)); // All flags cleared or only WD_FIRST_NOK_INT flag set
}

void test_positive_irqSetGetMask_BUCK1_OV_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_BUCK1_OV_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_BUCK1_OV_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_positive_irqSetGetMask_BUCK1_UV_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_BUCK1_UV_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_BUCK1_UV_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_positive_irqSetGetMask_BUCK1_RV_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_BUCK1_RV_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_BUCK1_RV_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_positive_irqSetGetMask_BUCK1_ILIM_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_BUCK1_ILIM_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_BUCK1_ILIM_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_positive_irqSetGetMask_BUCK2_OV_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_BUCK2_OV_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_BUCK2_OV_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_positive_irqSetGetMask_BUCK2_UV_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_BUCK2_UV_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_BUCK2_UV_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_positive_irqSetGetMask_BUCK2_RV_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_BUCK2_RV_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_BUCK2_RV_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_positive_irqSetGetMask_BUCK2_ILIM_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_BUCK2_ILIM_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_BUCK2_ILIM_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_positive_irqSetGetMask_BUCK3_OV_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_BUCK3_OV_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_BUCK3_OV_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_positive_irqSetGetMask_BUCK3_UV_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_BUCK3_UV_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_BUCK3_UV_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_positive_irqSetGetMask_BUCK3_RV_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_BUCK3_RV_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_BUCK3_RV_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_positive_irqSetGetMask_BUCK3_ILIM_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_BUCK3_ILIM_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_BUCK3_ILIM_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_positive_irqSetGetMask_LDO_LS1_VMON1_OV_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_LDO_LS1_VMON1_OV_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_LDO_LS1_VMON1_OV_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_positive_irqSetGetMask_LDO_LS1_VMON1_UV_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_LDO_LS1_VMON1_UV_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_LDO_LS1_VMON1_UV_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_positive_irqSetGetMask_LDO_LS1_VMON1_RV_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_LDO_LS1_VMON1_RV_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_LDO_LS1_VMON1_RV_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_positive_irqSetGetMask_LDO_LS1_VMON1_ILIM_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_LDO_LS1_VMON1_ILIM_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_LDO_LS1_VMON1_ILIM_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_positive_irqSetGetMask_LS2_VMON2_OV_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_LS2_VMON2_OV_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_LS2_VMON2_OV_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_positive_irqSetGetMask_LS2_VMON2_UV_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_LS2_VMON2_UV_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_LS2_VMON2_UV_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_positive_irqSetGetMask_LS2_VMON2_RV_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_LS2_VMON2_RV_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_LS2_VMON2_RV_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_positive_irqSetGetMask_LS2_VMON2_ILIM_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_LS2_VMON2_ILIM_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_LS2_VMON2_ILIM_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_positive_irqSetGetMask_VCCA_OV_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_VCCA_OV_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_VCCA_OV_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_positive_irqSetGetMask_VCCA_UV_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_VCCA_UV_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_VCCA_UV_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_positive_irqSetGetMask_STARTUP_ENABLE_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_STARTUP_ENABLE_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_STARTUP_ENABLE_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_positive_irqSetGetMask_ABIST_FAIL_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_MISC_ABIST_FAIL_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_MISC_ABIST_FAIL_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_positive_irqSetGetMask_BUCKS_VSET_ERR_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_MISC_BUCKS_VSET_ERR_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_MISC_BUCKS_VSET_ERR_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_positive_irqSetGetMask_EXT_CLK_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_MISC_EXT_CLK_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_MISC_EXT_CLK_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_positive_irqSetGetMask_TWARN_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_MISC_TWARN_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_MISC_TWARN_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_positive_irqSetGetMask_TRIM_TEST_CRC_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_ME_TRIM_TEST_CRC_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_ME_TRIM_TEST_CRC_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_positive_irqSetGetMask_CONFIG_CRC_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_ME_CONFIG_CRC_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_ME_CONFIG_CRC_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_positive_irqSetGetMask_NINT_READBACK_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_ME_NINT_READBACK_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_ME_NINT_READBACK_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_positive_irqSetGetMask_NRSTOUT_READBACK_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_ME_NRSTOUT_READBACK_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_ME_NRSTOUT_READBACK_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_positive_irqSetGetMask_COMM_FRM_ERR_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_COMM_FRM_ERR_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_COMM_FRM_ERR_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_positive_irqSetGetMask_COMM_CRC_ERR_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_COMM_CRC_ERR_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_COMM_CRC_ERR_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_positive_irqSetGetMask_COMM_ADR_ERR_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_COMM_ADR_ERR_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_COMM_ADR_ERR_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_positive_irqSetGetMask_COMM_MCU_ERR_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_COMM_MCU_ERR_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_COMM_MCU_ERR_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_positive_irqSetGetMask_ESM_MCU_PIN_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_ESM_MCU_PIN_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_ESM_MCU_PIN_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_positive_irqSetGetMask_ESM_MCU_FAIL_INT(void)
{
    int32_t status = irqTest_setGetMask(PMIC_ESM_MCU_FAIL_INT, PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMask(PMIC_ESM_MCU_FAIL_INT, PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_positive_irqSetGetMask_ESM_MCU_RST_INT(void)
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

void test_positive_irqSetGetMask_all(void)
{
    int32_t status = irqTest_setGetMaskAll(PMIC_IRQ_UNMASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = irqTest_setGetMaskAll(PMIC_IRQ_MASK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_positive_irqGetClrFlag(void)
{
    int32_t status = PMIC_ST_SUCCESS;
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
