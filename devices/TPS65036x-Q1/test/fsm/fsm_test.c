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
#include "test_constants.h"


/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/* Test organization macros are defined in fsm_test.h */

/* ========================================================================== */
/*                             Global Variables                               */
/* ========================================================================== */

static Pmic_Handle_t pmicHandle = {0U};

/* ========================================================================== */
/*                           Function Declarations                            */
/* ========================================================================== */

static int32_t fsmTest_clrResetCnt(void);
static int32_t fsmTest_clrRecovCnt(void);
static int32_t fsmTest_unlockPmicRegs(Pmic_Handle_t *pHandle);

/* ========================================================================== */
/*                           Function Definitions                             */
/* ========================================================================== */

void fsm_test(void *args)
{
    char msg[50U] = {0};
    int32_t status = PMIC_ST_SUCCESS;

    platform_init();

    Pmic_HandleCfg_t pmicCfg = {
        .validParams = (PMIC_I2C_ADDR0_VALID |
                        PMIC_COMM_HANDLE_0_VALID |
                        PMIC_IO_READ_VALID |
                        PMIC_IO_WRITE_VALID |
                        PMIC_CRITICAL_SECTION_START_VALID |
                        PMIC_CRITICAL_SECTION_STOP_VALID |
                        PMIC_IRQ_RESPONSE_CALLBACK_VALID |
                        PMIC_TIMER_WAIT_MS_VALID),
        .i2cAddr0 = PLATFORM_TARGET_I2C_ADDR,
        .commHandle0 = platform_getCommHandle(),
        .ioRead = &platform_rxByte,
        .ioWrite = &platform_txByte,
        .criticalSectionStart = &platform_critSecStart,
        .criticalSectionStop = &platform_critSecStop,
        .irqResponseCallback = &platform_irqResponse,
        .timerWaitMs = &testUtils_timerWaitMs
    };

    testTimer_startModule("FSM");

    platform_printString("\r\n");
    platform_printString("FSM_TEST\r\n");
    platform_printString("-------\r\n\r\n");

    status = Pmic_init(&pmicHandle, &pmicCfg);

    if (status == PMIC_ST_SUCCESS)
    {
        testUtils_printSiRev(&pmicHandle);

        /* Unlock PMIC registers for testing */
        status = fsmTest_unlockPmicRegs(&pmicHandle);
        if (status != PMIC_ST_SUCCESS)
        {
            (void)sprintf(msg, "Error unlocking PMIC registers: %d\r\n", status);
            platform_printString(msg);
        }

        platform_setupTests();
        FSM_TEST_RUN_ALL();
        platform_tearDownTests();
    }
    else
    {
        (void)sprintf(msg, "Error in initializing PMIC LLD: %d\r\n", status);
        platform_printString(msg);
    }

    testTimer_endModule();

    (void)Pmic_deinit(&pmicHandle);
    platform_deinit();
}

void test_neg_fsm_fsmSetDevState_nullParam_pmicHandle(void)
{
    // Pass NULL pmicHandle into Pmic_fsmSetDevState()
    int32_t status = Pmic_fsmSetDevState(NULL, PMIC_WARM_RESET_REQUEST);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_fsm_fsmSetDevState_invalid_fsmCmd(void)
{
    // Pass invalid fsmCmd into Pmic_sendFsmCmd
    int32_t status = Pmic_fsmSetDevState(&pmicHandle, 0x00U);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_fsm_fsmSetRecovCntThr_nullParam_pmicHandle(void)
{
    // Pass NULL pmicHandle into Pmic_fsmSetRecovCntThr()
    int32_t status = Pmic_fsmSetRecovCntThr(NULL, PMIC_RESET_RECOV_CNT_THR_MAX);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_fsm_fsmSetRecovCntThr_outOfBounds_threshold(void)
{
    // Pass out of bounds threshold into Pmic_fsmSetRecovCntThr()
    int32_t status = Pmic_fsmSetRecovCntThr(&pmicHandle, PMIC_RESET_RECOV_CNT_THR_MAX + 1U);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_fsm_fsmGetRecovCntThr_nullParam_pmicHandle(void)
{
    // Pass NULL pmicHandle into Pmic_fsmGetRecovCntThr()
    uint8_t threshold = 0U;
    int32_t status = Pmic_fsmGetRecovCntThr(NULL, &threshold);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_fsm_fsmGetRecovCntThr_nullParam_threshold(void)
{
    // Pass NULL threshold into Pmic_fsmGetRecovCntThr()
    int32_t status = Pmic_fsmGetRecovCntThr(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_fsm_fsmGetRecovCnt_nullParam_pmicHandle(void)
{
    // Pass NULL pmicHandle into Pmic_fsmGetRecovCnt()
    uint8_t recovCnt = 0U;
    int32_t status = Pmic_fsmGetRecovCnt(NULL, &recovCnt);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_fsm_fsmGetRecovCnt_nullParam_recovCnt(void)
{
    // Pass NULL recovCnt into Pmic_fsmGetRecovCnt()
    int32_t status = Pmic_fsmGetRecovCnt(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_fsm_fsmClrRecovCnt_nullParam_pmicHandle(void)
{
    // Pass NULL pmicHandle into Pmic_fsmClrRecovCnt()
    int32_t status = Pmic_fsmClrRecovCnt(NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_fsm_fsmSetResetCntThr_nullParam_pmicHandle(void)
{
    // Pass NULL pmicHandle into Pmic_fsmSetResetCntThr()
    int32_t status = Pmic_fsmSetResetCntThr(NULL, PMIC_RESET_RECOV_CNT_THR_MAX);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_fsm_fsmSetResetCntThr_outOfBounds_threshold(void)
{
    // Pass out of bounds threshold into Pmic_fsmSetResetCntThr()
    int32_t status = Pmic_fsmSetResetCntThr(&pmicHandle, PMIC_RESET_RECOV_CNT_THR_MAX + 1U);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_fsm_fsmGetResetCntThr_nullParam_pmicHandle(void)
{
    // Pass NULL pmicHandle into Pmic_fsmGetResetCntThr()
    uint8_t threshold = 0U;
    int32_t status = Pmic_fsmGetResetCntThr(NULL, &threshold);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_fsm_fsmGetResetCntThr_nullParam_threshold(void)
{
    // Pass NULL threshold into Pmic_fsmGetResetCntThr()
    int32_t status = Pmic_fsmGetResetCntThr(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_fsm_fsmGetResetCnt_nullParam_pmicHandle(void)
{
    // Pass NULL pmicHandle into Pmic_fsmGetResetCnt()
    uint8_t resetCnt = 0U;
    int32_t status = Pmic_fsmGetResetCnt(NULL, &resetCnt);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_fsm_fsmGetResetCnt_nullParam_resetCnt(void)
{
    // Pass NULL resetCnt into Pmic_fsmGetResetCnt()
    int32_t status = Pmic_fsmGetResetCnt(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_fsm_fsmClrResetCnt_nullParam_pmicHandle(void)
{
    // Pass NULL pmicHandle into Pmic_fsmClrResetCnt()
    int32_t status = Pmic_fsmClrResetCnt(NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_pos_fsm_setGetRecovCntThr(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t actThreshold = 0U;

    // clear recovery counter
    status = fsmTest_clrRecovCnt();
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // For each valid threshold value...
    for (uint8_t expThreshold = 0U; expThreshold <= PMIC_RESET_RECOV_CNT_THR_MAX; expThreshold++)
    {
        // Set expected threshold value
        status = Pmic_fsmSetRecovCntThr(&pmicHandle, expThreshold);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        // Get actual threshold value and compare expected vs. actual values
        status = Pmic_fsmGetRecovCntThr(&pmicHandle, &actThreshold);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        PLATFORM_ASSERT(expThreshold == actThreshold);
    }
}

void test_pos_fsm_setGetResetCntThr(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t actThreshold = 0U;

    // clear reset counter
    status = fsmTest_clrResetCnt();
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // For each valid threshold value...
    for (uint8_t expThreshold = 0U; expThreshold <= PMIC_RESET_RECOV_CNT_THR_MAX; expThreshold++)
    {
        // Set expected threshold value
        status = Pmic_fsmSetResetCntThr(&pmicHandle, expThreshold);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        // Get actual threshold value and compare expected vs. actual values
        status = Pmic_fsmGetResetCntThr(&pmicHandle, &actThreshold);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        PLATFORM_ASSERT(expThreshold == actThreshold);
    }
}

/*
 * NOTE: This test puts the PMIC in SAFE state, which affects I2C communication.
 * As a result, ignore all I2C communication errors after sending Safe Recovery
 * Request.
 */
void test_pos_fsm_getClrRecovCnt(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t initRecovCnt = 0U, newRecovCnt = 0U;

    // Get initial recovery count
    status = Pmic_fsmGetRecovCnt(&pmicHandle, &initRecovCnt);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Send FSM command to enter safe state
    status = Pmic_fsmSetDevState(&pmicHandle, PMIC_SAFE_RECOVERY_REQUEST);
    PLATFORM_ASSERT((status == PMIC_ST_SUCCESS) || (status == PMIC_ST_ERR_I2C_COMM_FAIL));

    // Clear all IRQs and unlock PMIC registers
    status = Pmic_irqClrAllFlags(&pmicHandle);
    PLATFORM_ASSERT((status == PMIC_ST_SUCCESS) || (status == PMIC_ST_ERR_I2C_COMM_FAIL));
    PLATFORM_ASSERT((status == PMIC_ST_SUCCESS) || (status == PMIC_ST_ERR_I2C_COMM_FAIL));

    // Get new recovery count and compare initial vs. new recovery count
    status = Pmic_fsmGetRecovCnt(&pmicHandle, &newRecovCnt);
    PLATFORM_ASSERT((status == PMIC_ST_SUCCESS) || (status == PMIC_ST_ERR_I2C_COMM_FAIL));
    PLATFORM_ASSERT(newRecovCnt = (initRecovCnt + 1U));

    // Clear the recovery counter
    status = Pmic_fsmClrRecovCnt(&pmicHandle);
    PLATFORM_ASSERT((status == PMIC_ST_SUCCESS) || (status == PMIC_ST_ERR_I2C_COMM_FAIL));

    // Get new recovery count and compare expected vs. actual value
    status = Pmic_fsmGetRecovCnt(&pmicHandle, &newRecovCnt);
    PLATFORM_ASSERT((status == PMIC_ST_SUCCESS) || (status == PMIC_ST_ERR_I2C_COMM_FAIL));
    PLATFORM_ASSERT(newRecovCnt == 0U);
}

/*
 * NOTE: This test makes the PMIC undergo WARM RESET, which affects I2C communication.
 * As a result, ignore all I2C communication errors after sending WARM RESET Request.
 */
void test_pos_fsm_getClrResetCnt(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t initResetCnt = 0U, newResetCnt = 0U;

    // Get initial reset count
    status = Pmic_fsmGetResetCnt(&pmicHandle, &initResetCnt);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Send FSM command for warm reset
    status = Pmic_fsmSetDevState(&pmicHandle, PMIC_WARM_RESET_REQUEST);
    PLATFORM_ASSERT((status == PMIC_ST_SUCCESS) || (status == PMIC_ST_ERR_I2C_COMM_FAIL));

    // Clear all IRQs and unlock PMIC registers
    status = Pmic_irqClrAllFlags(&pmicHandle);
    PLATFORM_ASSERT((status == PMIC_ST_SUCCESS) || (status == PMIC_ST_ERR_I2C_COMM_FAIL));
    PLATFORM_ASSERT((status == PMIC_ST_SUCCESS) || (status == PMIC_ST_ERR_I2C_COMM_FAIL));

    // Get new reset count and compare initial vs. new reset count
    status = Pmic_fsmGetRecovCnt(&pmicHandle, &newResetCnt);
    PLATFORM_ASSERT((status == PMIC_ST_SUCCESS) || (status == PMIC_ST_ERR_I2C_COMM_FAIL));
    PLATFORM_ASSERT(newResetCnt = (initResetCnt + 1U));

    // Clear the reset counter
    status = Pmic_fsmClrResetCnt(&pmicHandle);
    PLATFORM_ASSERT((status == PMIC_ST_SUCCESS) || (status == PMIC_ST_ERR_I2C_COMM_FAIL));

    // Get new reset count and compare expected vs. actual value
    status = Pmic_fsmGetResetCnt(&pmicHandle, &newResetCnt);
    PLATFORM_ASSERT((status == PMIC_ST_SUCCESS) || (status == PMIC_ST_ERR_I2C_COMM_FAIL));
    PLATFORM_ASSERT(newResetCnt == 0U);
}

static int32_t fsmTest_clrResetCnt(void)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;
    const uint8_t recovCntControlRegAddr = 0x07U, bufLen = 1U, resetCntClrShift = 1U, resetCntClrMask = 1UL << 1U;

    // Read RECOV_CNT_CONTROL
    status = platform_rxByte(&pmicHandle, 0U, recovCntControlRegAddr, &regData, bufLen);

    // Set RESET_CNT_CLR bit field to 1 and write RECOV_CNT_CONTROL
    if (status == PMIC_ST_SUCCESS)
    {
        Pmic_setBitField(&regData, resetCntClrShift, resetCntClrMask, 1U);
        status = platform_txByte(&pmicHandle, 0U, recovCntControlRegAddr, &regData, bufLen);
    }

    return status;
}

static int32_t fsmTest_clrRecovCnt(void)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;
    const uint8_t recovCntControlRegAddr = 0x07U, bufLen = 1U, recovCntClrShift = 0U, recovCntClrMask = 1UL << 0U;

    // Read RECOV_CNT_CONTROL
    status = platform_rxByte(&pmicHandle, 0U, recovCntControlRegAddr, &regData, bufLen);

    // Set RECOV_CNT_CLR bit field to 1 and write RECOV_CNT_CONTROL
    if (status == PMIC_ST_SUCCESS)
    {
        Pmic_setBitField(&regData, recovCntClrShift, recovCntClrMask, 1U);
        status = platform_txByte(&pmicHandle, 0U, recovCntControlRegAddr, &regData, bufLen);
    }

    return status;
}

static int32_t fsmTest_unlockPmicRegs(Pmic_Handle_t *pHandle)
{
    uint8_t regData = 0x9BU;
    const uint8_t bufLen = 1U;
    const uint16_t registerLockAddr = 0x09U;

    int32_t status = Pmic_checkHandle(pHandle);

    if (status == PMIC_ST_SUCCESS)
    {
        status = platform_txByte(pHandle, 0U, registerLockAddr, &regData, bufLen);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        status = platform_rxByte(pHandle, 0U, registerLockAddr, &regData, bufLen);
    }

    if ((status == PMIC_ST_SUCCESS) && (regData != 0U))
    {
        status = PMIC_ST_ERR_I2C_COMM_FAIL;
    }

    return status;
}
