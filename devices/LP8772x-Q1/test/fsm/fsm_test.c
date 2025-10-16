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
 * @brief Source file containing definitions to PMIC FSM tests.
 */

/* ========================================================================== */
/*                              Include Files                                 */
/* ========================================================================== */

#include "fsm_test.h"

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/* Run all FSM tests */
#define FSM_TEST_RUN_ALL() PLATFORM_RUN_TEST(test_negative_Pmic_fsmMcuCommand_nullParam_handle); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_fsmMcuCommand_invalidParam_cmd); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_fsmSetResetCntThr_nullParam_handle); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_fsmSetResetCntThr_outOfBounds_resetCntThr); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_fsmGetResetCntThr_nullParam_handle); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_fsmGetResetCntThr_nullParam_resetCntThr); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_fsmGetResetCnt_nullParam_handle); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_fsmGetResetCnt_nullParam_resetCnt); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_fsmClrResetCnt_nullParam_handle); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_fsmSetRecovCntThr_nullParam_handle); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_fsmSetRecovCntThr_outOfBounds_recovCntThr); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_fsmGetRecovCntThr_nullParam_handle); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_fsmGetRecovCntThr_nullParam_recovCntThr); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_fsmGetRecovCnt_nullParam_handle); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_fsmGetRecovCnt_nullParam_recovCnt); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_fsmClrRecovCnt_nullParam_handle); \
                           PLATFORM_RUN_TEST(test_positive_setGetResetCntThr); \
                           PLATFORM_RUN_TEST(test_positive_setGetRecovCntThr); \
                           PLATFORM_RUN_TEST(test_positive_Pmic_fsmMcuCommand_coldBootReq); \
                           PLATFORM_RUN_TEST(test_positive_Pmic_fsmMcuCommand_warmResetReq); \
                           PLATFORM_RUN_TEST(test_positive_Pmic_fsmMcuCommand_safeRecovReq); \
                           PLATFORM_RUN_TEST(test_positive_Pmic_fsmMcuCommand_offReq)

/* Run all FSM negative tests */
#define FSM_TEST_RUN_NEGATIVE() PLATFORM_RUN_TEST(test_negative_Pmic_fsmMcuCommand_nullParam_handle); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_fsmMcuCommand_invalidParam_cmd); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_fsmSetResetCntThr_nullParam_handle); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_fsmSetResetCntThr_outOfBounds_resetCntThr); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_fsmGetResetCntThr_nullParam_handle); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_fsmGetResetCntThr_nullParam_resetCntThr); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_fsmGetResetCnt_nullParam_handle); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_fsmGetResetCnt_nullParam_resetCnt); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_fsmClrResetCnt_nullParam_handle); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_fsmSetRecovCntThr_nullParam_handle); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_fsmSetRecovCntThr_outOfBounds_recovCntThr); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_fsmGetRecovCntThr_nullParam_handle); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_fsmGetRecovCntThr_nullParam_recovCntThr); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_fsmGetRecovCnt_nullParam_handle); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_fsmGetRecovCnt_nullParam_recovCnt); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_fsmClrRecovCnt_nullParam_handle)

/* Run all FSM positive tests */
#define FSM_TEST_RUN_POSITIVE() PLATFORM_RUN_TEST(test_positive_setGetResetCntThr); \
                                PLATFORM_RUN_TEST(test_positive_setGetRecovCntThr); \
                                PLATFORM_RUN_TEST(test_positive_Pmic_fsmMcuCommand_coldBootReq); \
                                PLATFORM_RUN_TEST(test_positive_Pmic_fsmMcuCommand_warmResetReq); \
                                PLATFORM_RUN_TEST(test_positive_Pmic_fsmMcuCommand_safeRecovReq); \
                                PLATFORM_RUN_TEST(test_positive_Pmic_fsmMcuCommand_offReq)

/* ========================================================================== */
/*                             Global Variables                               */
/* ========================================================================== */
static Pmic_CoreHandle_t pmicHandle;

/* ========================================================================== */
/*                           Function Declarations                            */
/* ========================================================================== */
static int32_t fsmTest_unlockPmicRegs(Pmic_CoreHandle_t *pmicHandle);
static inline void fsmTest_assertPmicRegsLocked(bool lock);

/* ========================================================================== */
/*                           Function Definitions                             */
/* ========================================================================== */

void fsm_test(void *args)
{
    char msg[50U] = {0};
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_CoreCfg_t coreCfg = {
        .validParams = (PMIC_CFG_DEVICE_TYPE_VALID_SHIFT |
                        PMIC_CFG_COMM_MODE_VALID_SHIFT |
                        PMIC_CFG_SLAVEADDR_VALID_SHIFT |
                        PMIC_CFG_COMM_HANDLE_VALID_SHIFT |
                        PMIC_CFG_COMM_IO_RD_VALID_SHIFT |
                        PMIC_CFG_COMM_IO_WR_VALID_SHIFT |
                        PMIC_CFG_CRITSEC_START_VALID_SHIFT |
                        PMIC_CFG_CRITSEC_STOP_VALID_SHIFT |
                        PMIC_CFG_CRC_ENABLE_VALID_SHIFT |
                        PMIC_CFG_CFG_CRC_ENABLE_VALID_SHIFT |
                        PMIC_CFG_PSEUDO_IRQ_VALID_SHIFT),
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
        .pFnPmicPseudoIrq = &platform_irqResponse
    };

    platform_init();

    platform_printString("\r\n");
    platform_printString("FSM_TEST\r\n");
    platform_printString("--------\r\n\r\n");

    status = Pmic_init(&pmicHandle, &coreCfg);

    if (status == PMIC_ST_SUCCESS)
    {
        status = fsmTest_unlockPmicRegs(&pmicHandle);

        if (status == PMIC_ST_SUCCESS)
        {
            platform_setupTests();
            FSM_TEST_RUN_ALL();
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

static int32_t fsmTest_unlockPmicRegs(Pmic_CoreHandle_t *pmicHandle)
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

void test_negative_Pmic_fsmMcuCommand_nullParam_handle(void)
{
    // Pass null handle into Pmic_fsmMcuCommand()
    int32_t status = Pmic_fsmMcuCommand(NULL, PMIC_FSM_COMMAND_WARM_RESET_REQ);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_HANDLE);
}

void test_negative_Pmic_fsmMcuCommand_invalidParam_cmd(void)
{
    // Pass invalid command into Pmic_fsmMcuCommand()
    for (uint16_t cmd = 0U; cmd <= UINT8_MAX; cmd++)
    {
        if ((cmd == PMIC_FSM_COMMAND_OFF_REQ) ||
            (cmd == PMIC_FSM_COMMAND_COLD_BOOT_REQ) ||
            (cmd == PMIC_FSM_COMMAND_WARM_RESET_REQ) ||
            (cmd == PMIC_FSM_COMMAND_SAFE_RECOV_REQ) ||
            (cmd == PMIC_FSM_COMMAND_OTA_FW_DOWNLOAD_REQ))
        {
            continue;
        }

        int32_t status = Pmic_fsmMcuCommand(&pmicHandle, cmd);
        PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
    }
}

void test_negative_Pmic_fsmSetResetCntThr_nullParam_handle(void)
{
    // Pass null handle into Pmic_fsmSetResetCntThr()
    int32_t status = Pmic_fsmSetResetCntThr(NULL, PMIC_FSM_RESET_RECOV_CNT_THR_MAX);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_HANDLE);
}

void test_negative_Pmic_fsmSetResetCntThr_outOfBounds_resetCntThr(void)
{
    // Pass out-of-bounds resetCntThr into Pmic_fsmSetResetCntThr()
    int32_t status = Pmic_fsmSetResetCntThr(&pmicHandle, PMIC_FSM_RESET_RECOV_CNT_THR_MAX + 1U);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_negative_Pmic_fsmGetResetCntThr_nullParam_handle(void)
{
    // Pass null handle into Pmic_fsmGetResetCntThr()
    uint8_t resetCntThr = 0U;
    int32_t status = Pmic_fsmGetResetCntThr(NULL, &resetCntThr);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_HANDLE);
}

void test_negative_Pmic_fsmGetResetCntThr_nullParam_resetCntThr(void)
{
    // Pass null resetCntThr into Pmic_fsmGetResetCntThr()
    int32_t status = Pmic_fsmGetResetCntThr(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_fsmGetResetCnt_nullParam_handle(void)
{
    // Pass null handle into Pmic_fsmGetResetCnt()
    uint8_t resetCnt = 0U;
    int32_t status = Pmic_fsmGetResetCnt(NULL, &resetCnt);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_HANDLE);
}

void test_negative_Pmic_fsmGetResetCnt_nullParam_resetCnt(void)
{
    // Pass null resetCnt into Pmic_fsmGetResetCnt()
    int32_t status = Pmic_fsmGetResetCnt(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_fsmClrResetCnt_nullParam_handle(void)
{
    // Pass null handle into Pmic_fsmClrResetCnt()
    int32_t status = Pmic_fsmClrResetCnt(NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_HANDLE);
}

void test_negative_Pmic_fsmSetRecovCntThr_nullParam_handle(void)
{
    // Pass null handle into Pmic_fsmSetRecovCntThr()
    int32_t status = Pmic_fsmSetRecovCntThr(NULL, PMIC_FSM_RESET_RECOV_CNT_THR_MAX);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_HANDLE);
}

void test_negative_Pmic_fsmSetRecovCntThr_outOfBounds_recovCntThr(void)
{
    // Pass out-of-bounds recovCntThr into Pmic_fsmSetRecovCntThr()
    int32_t status = Pmic_fsmSetRecovCntThr(&pmicHandle, PMIC_FSM_RESET_RECOV_CNT_THR_MAX + 1U);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_negative_Pmic_fsmGetRecovCntThr_nullParam_handle(void)
{
    // Pass null handle into Pmic_fsmGetRecovCntThr()
    uint8_t recovCntThr = 0U;
    int32_t status = Pmic_fsmGetRecovCntThr(NULL, &recovCntThr);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_HANDLE);
}

void test_negative_Pmic_fsmGetRecovCntThr_nullParam_recovCntThr(void)
{
    // Pass null recovCntThr into Pmic_fsmGetRecovCntThr()
    int32_t status = Pmic_fsmGetRecovCntThr(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_fsmGetRecovCnt_nullParam_handle(void)
{
    // Pass null handle into Pmic_fsmGetRecovCnt()
    uint8_t recovCnt = 0U;
    int32_t status = Pmic_fsmGetRecovCnt(NULL, &recovCnt);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_HANDLE);
}

void test_negative_Pmic_fsmGetRecovCnt_nullParam_recovCnt(void)
{
    // Pass null recovCnt into Pmic_fsmGetRecovCnt()
    int32_t status = Pmic_fsmGetRecovCnt(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_fsmClrRecovCnt_nullParam_handle(void)
{
    // Pass null handle into Pmic_fsmClrRecovCnt()
    int32_t status = Pmic_fsmClrRecovCnt(NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_HANDLE);
}

void test_positive_setGetResetCntThr(void)
{
    uint8_t actResetCntThr = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    // Clear reset counter
    status = Pmic_fsmClrResetCnt(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // For each valid resetCntThr value...
    for (uint8_t expResetCntThr = 0U; expResetCntThr <= PMIC_FSM_RESET_RECOV_CNT_THR_MAX; expResetCntThr++)
    {
        // Set reset count threshold
        status = Pmic_fsmSetResetCntThr(&pmicHandle, expResetCntThr);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        // Get actual reset count threshold and compare expected vs. actual values
        status = Pmic_fsmGetResetCntThr(&pmicHandle, &actResetCntThr);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        PLATFORM_ASSERT(expResetCntThr == actResetCntThr);
    }
}

void test_positive_setGetRecovCntThr(void)
{
    uint8_t actRecovCntThr = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    // Clear recovery counter
    status = Pmic_fsmClrRecovCnt(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // For each valid recovCntThr value...
    for (uint8_t expRecovCntThr = 0U; expRecovCntThr <= PMIC_FSM_RESET_RECOV_CNT_THR_MAX; expRecovCntThr++)
    {
        // Set recovery count threshold
        status = Pmic_fsmSetRecovCntThr(&pmicHandle, expRecovCntThr);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        // Get actual recovery count threshold and compare expected vs. actual values
        status = Pmic_fsmGetRecovCntThr(&pmicHandle, &actRecovCntThr);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        PLATFORM_ASSERT(expRecovCntThr == actRecovCntThr);
    }
}

static inline void fsmTest_assertPmicRegsLocked(bool lock)
{
    uint8_t regData = 0U;
    const uint8_t bufLen = 1U;
    const uint16_t registerLockAddr = 0x09U;

    // Read REGISTER_LOCK
    int32_t status = platform_rxByte(&pmicHandle, PMIC_MAIN_INST, registerLockAddr, &regData, bufLen);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Assert PMIC registers are locked
    if (lock)
    {
        PLATFORM_ASSERT(regData != 0U);
    }
    // Assert PMIC registers are unlocked
    else
    {
        PLATFORM_ASSERT(regData == 0U);
    }
}

void test_positive_Pmic_fsmMcuCommand_coldBootReq(void)
{
    // Assert PMIC registers are unlocked (PMIC registers should've be unlocked
    // previously via fsmTest_unlockPmicRegs()).
    fsmTest_assertPmicRegsLocked((bool)false);

    // Send Cold Boot request
    int32_t status = Pmic_fsmMcuCommand(&pmicHandle, PMIC_FSM_COMMAND_COLD_BOOT_REQ);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Wait some time for PMIC to go through Cold Boot
    platform_timerWaitMs(5U);

    // After Cold Boot, registers are automatically locked; assert PMIC registers are locked
    fsmTest_assertPmicRegsLocked((bool)true);

    // Unlock PMIC registers
    status = fsmTest_unlockPmicRegs(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_positive_Pmic_fsmMcuCommand_warmResetReq(void)
{
    uint8_t initResetCnt = 0U, actResetCnt = 0U;

    // Get initial PMIC RESET_CNT value
    int32_t status = Pmic_fsmGetResetCnt(&pmicHandle, &initResetCnt);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Send Warm Reset request. It is likely that API will return PMIC_ST_ERR_I2C_COMM_FAIL
    // because PMIC ceases communication upon entering Warm Reset, so ignore status code
    status = Pmic_fsmMcuCommand(&pmicHandle, PMIC_FSM_COMMAND_WARM_RESET_REQ);

    // Wait some time for PMIC to go through Warm Reset
    platform_timerWaitMs(5U);

    // Get actual PMIC RESET_CNT value after PMIC has undergone Warm Reset.
    // Compare expected vs. actual values
    status = Pmic_fsmGetResetCnt(&pmicHandle, &actResetCnt);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT((initResetCnt + 1U) == actResetCnt);

    // Unlock PMIC registers
    status = fsmTest_unlockPmicRegs(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_positive_Pmic_fsmMcuCommand_safeRecovReq(void)
{
    uint8_t initRecovCnt = 0U, actRecovCnt = 0U;

    // Get initial PMIC RECOV_CNT
    int32_t status = Pmic_fsmGetRecovCnt(&pmicHandle, &initRecovCnt);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Send Safe Recovery request
    status = Pmic_fsmMcuCommand(&pmicHandle, PMIC_FSM_COMMAND_SAFE_RECOV_REQ);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Wait some time for PMIC to go through Safe Recovery
    platform_timerWaitMs(5U);

    // Get actual PMIC RECOV_CNT value after PMIC has undergone Safe Recovery.
    // Compare expected vs. actual values
    status = Pmic_fsmGetRecovCnt(&pmicHandle, &actRecovCnt);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT((initRecovCnt + 1U) == actRecovCnt);

    // Unlock PMIC registers
    status = fsmTest_unlockPmicRegs(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_positive_Pmic_fsmMcuCommand_offReq(void)
{
    // Assert PMIC registers are unlocked (PMIC registers should've be unlocked
    // previously via fsmTest_unlockPmicRegs() or another test).
    fsmTest_assertPmicRegsLocked((bool)false);

    // Send Off request
    int32_t status = Pmic_fsmMcuCommand(&pmicHandle, PMIC_FSM_COMMAND_OFF_REQ);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Wait some time for PMIC to turn off (PMIC enters STANDBY state)
    platform_timerWaitMs(5U);

    // After the PMIC enters STANDBY state, registers are automatically locked; assert PMIC registers are locked
    fsmTest_assertPmicRegsLocked((bool)true);
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
