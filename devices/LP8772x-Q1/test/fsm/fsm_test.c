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

#include "fsm_test.h"

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/* ========================================================================== */
/*               API-Specific Test Macros - fsmClrRecovCnt                    */
/* ========================================================================== */

#define FSM_TEST_NEG_FSMCLRRECOVCNT() \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmClrRecovCnt_nullHandle)

#define FSM_TEST_FSMCLRRECOVCNT() \
    FSM_TEST_NEG_FSMCLRRECOVCNT()

/* ========================================================================== */
/*               API-Specific Test Macros - fsmClrResetCnt                    */
/* ========================================================================== */

#define FSM_TEST_NEG_FSMCLRRESETCNT() \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmClrResetCnt_nullHandle)

#define FSM_TEST_FSMCLRRESETCNT() \
    FSM_TEST_NEG_FSMCLRRESETCNT()

/* ========================================================================== */
/*               API-Specific Test Macros - fsmGetRecovCnt                    */
/* ========================================================================== */

#define FSM_TEST_NEG_FSMGETRECOVCNT() \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmGetRecovCnt_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmGetRecovCnt_nullRecovCnt)

#define FSM_TEST_FSMGETRECOVCNT() \
    FSM_TEST_NEG_FSMGETRECOVCNT()

/* ========================================================================== */
/*              API-Specific Test Macros - fsmGetRecovCntThr                  */
/* ========================================================================== */

#define FSM_TEST_POS_FSMGETRECOVCNTTHR() \
    PLATFORM_RUN_TEST(test_pos_fsm_setGetRecovCntThr)

#define FSM_TEST_NEG_FSMGETRECOVCNTTHR() \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmGetRecovCntThr_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmGetRecovCntThr_nullRecovCntThr)

#define FSM_TEST_FSMGETRECOVCNTTHR() \
    FSM_TEST_POS_FSMGETRECOVCNTTHR(); \
    FSM_TEST_NEG_FSMGETRECOVCNTTHR()

/* ========================================================================== */
/*               API-Specific Test Macros - fsmGetResetCnt                    */
/* ========================================================================== */

#define FSM_TEST_NEG_FSMGETRESETCNT() \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmGetResetCnt_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmGetResetCnt_nullResetCnt)

#define FSM_TEST_FSMGETRESETCNT() \
    FSM_TEST_NEG_FSMGETRESETCNT()

/* ========================================================================== */
/*              API-Specific Test Macros - fsmGetResetCntThr                  */
/* ========================================================================== */

#define FSM_TEST_POS_FSMGETRESETCNTTHR() \
    PLATFORM_RUN_TEST(test_pos_fsm_setGetResetCntThr)

#define FSM_TEST_NEG_FSMGETRESETCNTTHR() \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmGetResetCntThr_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmGetResetCntThr_nullResetCntThr)

#define FSM_TEST_FSMGETRESETCNTTHR() \
    FSM_TEST_POS_FSMGETRESETCNTTHR(); \
    FSM_TEST_NEG_FSMGETRESETCNTTHR()

/* ========================================================================== */
/*               API-Specific Test Macros - fsmSetDevState                    */
/* ========================================================================== */

#define FSM_TEST_POS_FSMSETDEVSTATE() \
    PLATFORM_RUN_TEST(test_pos_fsm_fsmSetDevState_coldBootReq); \
    PLATFORM_RUN_TEST(test_pos_fsm_fsmSetDevState_offReq); \
    PLATFORM_RUN_TEST(test_pos_fsm_fsmSetDevState_safeRecovReq); \
    PLATFORM_RUN_TEST(test_pos_fsm_fsmSetDevState_warmResetReq)

#define FSM_TEST_NEG_FSMSETDEVSTATE() \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmSetDevState_invalidCmd); \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmSetDevState_nullHandle)

#define FSM_TEST_FSMSETDEVSTATE() \
    FSM_TEST_POS_FSMSETDEVSTATE(); \
    FSM_TEST_NEG_FSMSETDEVSTATE()

/* ========================================================================== */
/*              API-Specific Test Macros - fsmSetRecovCntThr                  */
/* ========================================================================== */

#define FSM_TEST_POS_FSMSETRECOVCNTTHR() \
    PLATFORM_RUN_TEST(test_pos_fsm_setGetRecovCntThr)

#define FSM_TEST_NEG_FSMSETRECOVCNTTHR() \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmSetRecovCntThr_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmSetRecovCntThr_outOfBoundsRecovCntThr)

#define FSM_TEST_FSMSETRECOVCNTTHR() \
    FSM_TEST_POS_FSMSETRECOVCNTTHR(); \
    FSM_TEST_NEG_FSMSETRECOVCNTTHR()

/* ========================================================================== */
/*              API-Specific Test Macros - fsmSetResetCntThr                  */
/* ========================================================================== */

#define FSM_TEST_POS_FSMSETRESETCNTTHR() \
    PLATFORM_RUN_TEST(test_pos_fsm_setGetResetCntThr)

#define FSM_TEST_NEG_FSMSETRESETCNTTHR() \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmSetResetCntThr_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmSetResetCntThr_outOfBoundsResetCntThr)

#define FSM_TEST_FSMSETRESETCNTTHR() \
    FSM_TEST_POS_FSMSETRESETCNTTHR(); \
    FSM_TEST_NEG_FSMSETRESETCNTTHR()

/* ========================================================================== */
/*                         Aggregate Test Runners                             */
/* ========================================================================== */

#define FSM_TEST_RUN_POSITIVE() \
    FSM_TEST_POS_FSMGETRECOVCNTTHR(); \
    FSM_TEST_POS_FSMGETRESETCNTTHR(); \
    FSM_TEST_POS_FSMSETDEVSTATE(); \
    FSM_TEST_POS_FSMSETRECOVCNTTHR(); \
    FSM_TEST_POS_FSMSETRESETCNTTHR()

#define FSM_TEST_RUN_NEGATIVE() \
    FSM_TEST_NEG_FSMCLRRECOVCNT(); \
    FSM_TEST_NEG_FSMCLRRESETCNT(); \
    FSM_TEST_NEG_FSMGETRECOVCNT(); \
    FSM_TEST_NEG_FSMGETRECOVCNTTHR(); \
    FSM_TEST_NEG_FSMGETRESETCNT(); \
    FSM_TEST_NEG_FSMGETRESETCNTTHR(); \
    FSM_TEST_NEG_FSMSETDEVSTATE(); \
    FSM_TEST_NEG_FSMSETRECOVCNTTHR(); \
    FSM_TEST_NEG_FSMSETRESETCNTTHR()

#define FSM_TEST_RUN_ALL() \
    FSM_TEST_RUN_POSITIVE(); \
    FSM_TEST_RUN_NEGATIVE()

/* ========================================================================== */
/*                             Global Variables                               */
/* ========================================================================== */
static Pmic_Handle_t pmicHandle;

/* ========================================================================== */
/*                           Function Declarations                            */
/* ========================================================================== */
static inline void fsmTest_assertPmicRegsLocked(bool lock);

/* ========================================================================== */
/*                           Function Definitions                             */
/* ========================================================================== */

void fsm_test(void *args)
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
    platform_printString("FSM_TEST\r\n");
    platform_printString("--------\r\n\r\n");

    status = Pmic_init(&pmicHandle, &coreCfg);

    if (status == PMIC_ST_SUCCESS)
    {
        platform_setupTests();
        FSM_TEST_RUN_ALL();
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

void test_neg_fsm_fsmSetDevState_nullHandle(void)
{
    // Pass null handle into Pmic_fsmSetDevState()
    int32_t status = Pmic_fsmSetDevState(NULL, PMIC_FSM_COMMAND_WARM_RESET_REQ);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_fsm_fsmSetDevState_invalidCmd(void)
{
    // Pass invalid command into Pmic_fsmSetDevState()
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

        int32_t status = Pmic_fsmSetDevState(&pmicHandle, cmd);
        PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
    }
}

void test_neg_fsm_fsmSetResetCntThr_nullHandle(void)
{
    // Pass null handle into Pmic_fsmSetResetCntThr()
    int32_t status = Pmic_fsmSetResetCntThr(NULL, PMIC_FSM_RESET_RECOV_CNT_THR_MAX);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_fsm_fsmSetResetCntThr_outOfBoundsResetCntThr(void)
{
    // Pass out-of-bounds resetCntThr into Pmic_fsmSetResetCntThr()
    int32_t status = Pmic_fsmSetResetCntThr(&pmicHandle, PMIC_FSM_RESET_RECOV_CNT_THR_MAX + 1U);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_fsm_fsmGetResetCntThr_nullHandle(void)
{
    // Pass null handle into Pmic_fsmGetResetCntThr()
    uint8_t resetCntThr = 0U;
    int32_t status = Pmic_fsmGetResetCntThr(NULL, &resetCntThr);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_fsm_fsmGetResetCntThr_nullResetCntThr(void)
{
    // Pass null resetCntThr into Pmic_fsmGetResetCntThr()
    int32_t status = Pmic_fsmGetResetCntThr(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_fsm_fsmGetResetCnt_nullHandle(void)
{
    // Pass null handle into Pmic_fsmGetResetCnt()
    uint8_t resetCnt = 0U;
    int32_t status = Pmic_fsmGetResetCnt(NULL, &resetCnt);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_fsm_fsmGetResetCnt_nullResetCnt(void)
{
    // Pass null resetCnt into Pmic_fsmGetResetCnt()
    int32_t status = Pmic_fsmGetResetCnt(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_fsm_fsmClrResetCnt_nullHandle(void)
{
    // Pass null handle into Pmic_fsmClrResetCnt()
    int32_t status = Pmic_fsmClrResetCnt(NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_fsm_fsmSetRecovCntThr_nullHandle(void)
{
    // Pass null handle into Pmic_fsmSetRecovCntThr()
    int32_t status = Pmic_fsmSetRecovCntThr(NULL, PMIC_FSM_RESET_RECOV_CNT_THR_MAX);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_fsm_fsmSetRecovCntThr_outOfBoundsRecovCntThr(void)
{
    // Pass out-of-bounds recovCntThr into Pmic_fsmSetRecovCntThr()
    int32_t status = Pmic_fsmSetRecovCntThr(&pmicHandle, PMIC_FSM_RESET_RECOV_CNT_THR_MAX + 1U);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_fsm_fsmGetRecovCntThr_nullHandle(void)
{
    // Pass null handle into Pmic_fsmGetRecovCntThr()
    uint8_t recovCntThr = 0U;
    int32_t status = Pmic_fsmGetRecovCntThr(NULL, &recovCntThr);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_fsm_fsmGetRecovCntThr_nullRecovCntThr(void)
{
    // Pass null recovCntThr into Pmic_fsmGetRecovCntThr()
    int32_t status = Pmic_fsmGetRecovCntThr(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_fsm_fsmGetRecovCnt_nullHandle(void)
{
    // Pass null handle into Pmic_fsmGetRecovCnt()
    uint8_t recovCnt = 0U;
    int32_t status = Pmic_fsmGetRecovCnt(NULL, &recovCnt);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_fsm_fsmGetRecovCnt_nullRecovCnt(void)
{
    // Pass null recovCnt into Pmic_fsmGetRecovCnt()
    int32_t status = Pmic_fsmGetRecovCnt(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_fsm_fsmClrRecovCnt_nullHandle(void)
{
    // Pass null handle into Pmic_fsmClrRecovCnt()
    int32_t status = Pmic_fsmClrRecovCnt(NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_pos_fsm_setGetResetCntThr(void)
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

void test_pos_fsm_setGetRecovCntThr(void)
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
    int32_t status = platform_rxByte(&pmicHandle, 0, registerLockAddr, &regData, bufLen);
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

void test_pos_fsm_fsmSetDevState_coldBootReq(void)
{
    // Assert PMIC registers are unlocked (registers are unlocked in platform_setupMock)
    fsmTest_assertPmicRegsLocked((bool)false);

    // Send Cold Boot request
    int32_t status = Pmic_fsmSetDevState(&pmicHandle, PMIC_FSM_COMMAND_COLD_BOOT_REQ);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Wait some time for PMIC to go through Cold Boot
    platform_timerWaitMs(5U);

    // After Cold Boot, registers are automatically locked; assert PMIC registers are locked
    fsmTest_assertPmicRegsLocked((bool)true);
}

void test_pos_fsm_fsmSetDevState_warmResetReq(void)
{
    uint8_t initResetCnt = 0U, actResetCnt = 0U;

    // Get initial PMIC RESET_CNT value
    int32_t status = Pmic_fsmGetResetCnt(&pmicHandle, &initResetCnt);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Send Warm Reset request. It is likely that API will return PMIC_ST_ERR_I2C_COMM_FAIL
    // because PMIC ceases communication upon entering Warm Reset, so ignore status code
    status = Pmic_fsmSetDevState(&pmicHandle, PMIC_FSM_COMMAND_WARM_RESET_REQ);

    // Wait some time for PMIC to go through Warm Reset
    platform_timerWaitMs(5U);

    // Get actual PMIC RESET_CNT value after PMIC has undergone Warm Reset.
    // Compare expected vs. actual values
    status = Pmic_fsmGetResetCnt(&pmicHandle, &actResetCnt);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT((initResetCnt + 1U) == actResetCnt);
}

void test_pos_fsm_fsmSetDevState_safeRecovReq(void)
{
    uint8_t initRecovCnt = 0U, actRecovCnt = 0U;

    // Get initial PMIC RECOV_CNT
    int32_t status = Pmic_fsmGetRecovCnt(&pmicHandle, &initRecovCnt);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Send Safe Recovery request
    status = Pmic_fsmSetDevState(&pmicHandle, PMIC_FSM_COMMAND_SAFE_RECOV_REQ);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Wait some time for PMIC to go through Safe Recovery
    platform_timerWaitMs(5U);

    // Get actual PMIC RECOV_CNT value after PMIC has undergone Safe Recovery.
    // Compare expected vs. actual values
    status = Pmic_fsmGetRecovCnt(&pmicHandle, &actRecovCnt);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT((initRecovCnt + 1U) == actRecovCnt);
}

void test_pos_fsm_fsmSetDevState_offReq(void)
{
    // Assert PMIC registers are unlocked (registers are unlocked in platform_setupMock)
    fsmTest_assertPmicRegsLocked((bool)false);

    // Send Off request
    int32_t status = Pmic_fsmSetDevState(&pmicHandle, PMIC_FSM_COMMAND_OFF_REQ);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Wait some time for PMIC to turn off (PMIC enters STANDBY state)
    platform_timerWaitMs(5U);

    // After the PMIC enters STANDBY state, registers are automatically locked; assert PMIC registers are locked
    fsmTest_assertPmicRegsLocked((bool)true);
}

