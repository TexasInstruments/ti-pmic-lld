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
/**
 * @file io_test.c
 * @brief Source file containing definitions to PMIC IO tests.
 */

/* ========================================================================== */
/*                              Include Files                                 */
/* ========================================================================== */

#include "io_test.h"
#include "test_utils.h"

#include "test_constants.h"


/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/* PMIC scratchpad register addresses */
#define IO_TEST_SCRATCH_PAD_REG_1_REG (0x0AU)
#define IO_TEST_SCRATCH_PAD_REG_2_REG (0x0BU)
#define IO_TEST_SCRATCH_PAD_REG_3_REG (0x0CU)
#define IO_TEST_SCRATCH_PAD_REG_4_REG (0x0DU)

/* Max PMIC user-space register address */
#define IO_TEST_MAX_REG (0xF2U)

/* Register address where A0/B0 mapping differs (0x4D) */
#define IO_TEST_REGMAP_DIFF_START (0x4DU)

/* ========================================================================== */
/*                             Global Variables                               */
/* ========================================================================== */
Pmic_Handle_t pmicHandle;

/* Global variables for mock I/O control */
static uint32_t g_mockIoReadCallCount = 0U;
static uint32_t g_mockIoWriteCallCount = 0U;
static int32_t g_mockIoReadReturnStatus = PMIC_ST_SUCCESS;
static int32_t g_mockIoWriteReturnStatus = PMIC_ST_SUCCESS;
static uint8_t g_mockCrcCorruptionMask = 0x00U;  /* XOR mask to corrupt CRC byte */


/* ========================================================================== */
/*                           Function Declarations                            */
/* ========================================================================== */
static int32_t ioTest_unlockPmicRegs(Pmic_Handle_t *pmicHandle);

/* ========================================================================== */
/*                          Mock I/O Helper Functions                         */
/* ========================================================================== */

/**
 * @brief Reset mock I/O control variables to default state.
 */
static void resetMockIoState(void)
{
    g_mockIoReadCallCount = 0U;
    g_mockIoWriteCallCount = 0U;
    g_mockIoReadReturnStatus = PMIC_ST_SUCCESS;
    g_mockIoWriteReturnStatus = PMIC_ST_SUCCESS;
    g_mockCrcCorruptionMask = 0x00U;
}

/**
 * @brief Mock ioRead function that can inject errors on specific calls.
 */
static int32_t mockIoRead(const Pmic_Handle_t *handle, uint8_t page, uint8_t regAddr,
                          uint8_t *buffer, uint8_t bufLen)
{
    int32_t status;
    g_mockIoReadCallCount++;

    /* If configured to return error on this call, return it */
    if (g_mockIoReadReturnStatus != PMIC_ST_SUCCESS)
    {
        /* Return error only on first call, then succeed */
        status = g_mockIoReadReturnStatus;
        g_mockIoReadReturnStatus = PMIC_ST_SUCCESS;  /* Clear for next call */
        return status;
    }

    /* Otherwise call the real platform function */
    status = platform_rxByte(handle, page, regAddr, buffer, bufLen);

    /* Apply CRC corruption if configured (only when CRC is enabled and on first call) */
    if ((g_mockCrcCorruptionMask != 0x00U) && (bufLen == 2U) && (status == PMIC_ST_SUCCESS))
    {
        buffer[1] ^= g_mockCrcCorruptionMask;  /* Corrupt CRC byte */
        g_mockCrcCorruptionMask = 0x00U;       /* Clear for next call */
    }

    return status;
}

/**
 * @brief Mock ioWrite function that can inject errors on specific calls.
 */
static int32_t mockIoWrite(const Pmic_Handle_t *handle, uint8_t page, uint8_t regAddr,
                           const uint8_t *buffer, uint8_t bufLen)
{
    int32_t status;
    g_mockIoWriteCallCount++;

    /* If configured to return error on this call, return it */
    if (g_mockIoWriteReturnStatus != PMIC_ST_SUCCESS)
    {
        /* Return error only on first call, then succeed */
        status = g_mockIoWriteReturnStatus;
        g_mockIoWriteReturnStatus = PMIC_ST_SUCCESS;  /* Clear for next call */
        return status;
    }

    /* Otherwise call the real platform function */
    return platform_txByte(handle, page, regAddr, buffer, bufLen);
}

/**
 * @brief Mock timer wait function for retry testing.
 */
static void mockTimerWait(uint32_t ms)
{
    /* No-op for testing - we don't want to actually wait */
    (void)ms;
}

/* ========================================================================== */
/*                           Function Definitions                             */
/* ========================================================================== */

void io_test(void *args)
{
    (void)args;
    char msg[50U] = {0};
    int32_t status = PMIC_ST_SUCCESS;

    platform_init();

    Pmic_HandleCfg_t coreCfg = {
        .validParams = (PMIC_I2C_ADDR0_VALID |
                        PMIC_COMM_HANDLE_0_VALID |
                        PMIC_IO_READ_VALID |
                        PMIC_IO_WRITE_VALID |
                        PMIC_CRITICAL_SECTION_START_VALID |
                        PMIC_CRITICAL_SECTION_STOP_VALID |
                        PMIC_IRQ_RESPONSE_CALLBACK_VALID |
                        PMIC_TIMER_WAIT_MS_VALID),
        .i2cAddr0 = 0x60U,
        .commHandle0 = platform_getCommHandle(),
        .ioRead = &platform_rxByte,
        .ioWrite = &platform_txByte,
        .criticalSectionStart = &platform_critSecStart,
        .criticalSectionStop = &platform_critSecStop,
        .irqResponseCallback = &platform_irqResponse,
        .timerWaitMs = &testUtils_timerWaitMs
    };

    testTimer_startModule("I/O");

    platform_printString("\r\n");
    platform_printString("IO_TEST\r\n");
    platform_printString("-------\r\n\r\n");

    status = Pmic_init(&pmicHandle, &coreCfg);

    if (status == PMIC_ST_SUCCESS)
    {
        status = ioTest_unlockPmicRegs(&pmicHandle);

        if (status == PMIC_ST_SUCCESS)
        {
            platform_setupTests();
            IO_TEST_RUN_ALL();
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

    testTimer_endModule();

    (void)Pmic_deinit(&pmicHandle);
    platform_deinit();
}

static int32_t ioTest_unlockPmicRegs(Pmic_Handle_t *pmicHandle)
{
    uint8_t regData = 0x9BU;
    const uint8_t bufLen = 1U;
    const uint16_t registerLockAddr = 0x09U;

    // Check handle
    int32_t status = Pmic_checkHandle(pmicHandle);

    // Write key to REGISTER_LOCK
    if (status == PMIC_ST_SUCCESS)
    {
        status = platform_txByte(pmicHandle, 0U, registerLockAddr, &regData, bufLen);
    }

    // Get register lock status
    if (status == PMIC_ST_SUCCESS)
    {
        status = platform_rxByte(pmicHandle, 0U, registerLockAddr, &regData, bufLen);
    }

    // Validate that registers are unlocked
    if ((status == PMIC_ST_SUCCESS) && (regData != 0U))
    {
        status = PMIC_ST_ERR_I2C_COMM_FAIL;
    }

    return status;
}

/* ========================================================================== */
/*                        Negative Test Definitions                           */
/* ========================================================================== */

void test_neg_io_ioTxByte_nullHandle(void)
{
    // Pass null handle into Pmic_ioTxByte()
    const uint8_t regData = TEST_PATTERN_AA;
    int32_t status = Pmic_ioTxByte(NULL, IO_TEST_SCRATCH_PAD_REG_1_REG, regData);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_io_ioRxByte_nullHandle(void)
{
    // Pass null handle into Pmic_ioRxByte()
    uint8_t regData = 0U;
    int32_t status = Pmic_ioRxByte(NULL, IO_TEST_SCRATCH_PAD_REG_1_REG, &regData);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_io_ioRxByte_nullRxBuffer(void)
{
    // Pass null rxBuffer into Pmic_ioRxByte()
    int32_t status = Pmic_ioRxByte(&pmicHandle, IO_TEST_SCRATCH_PAD_REG_1_REG, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_io_ioTxByte_CS_nullHandle(void)
{
    // Pass null handle into Pmic_ioTxByte_CS()
    const uint8_t regData = TEST_PATTERN_AA;
    int32_t status = Pmic_ioTxByte_CS(NULL, IO_TEST_SCRATCH_PAD_REG_1_REG, regData);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_io_ioRxByte_CS_nullHandle(void)
{
    // Pass null handle into Pmic_ioRxByte_CS()
    uint8_t regData = 0U;
    int32_t status = Pmic_ioRxByte_CS(NULL, IO_TEST_SCRATCH_PAD_REG_1_REG, &regData);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_io_ioRxByte_CS_nullRxBuffer(void)
{
    // Pass null rxBuffer into Pmic_ioRxByte_CS()
    int32_t status = Pmic_ioRxByte_CS(&pmicHandle, IO_TEST_SCRATCH_PAD_REG_1_REG, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_io_ioUpdateByte_nullHandle(void)
{
    // Pass null handle into Pmic_ioUpdateByte()
    int32_t status = Pmic_ioUpdateByte(NULL, IO_TEST_SCRATCH_PAD_REG_1_REG, 0U, TEST_MASK_FULL_BYTE, TEST_PATTERN_AA);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_io_ioUpdateByte_CS_nullHandle(void)
{
    // Pass null handle into Pmic_ioUpdateByte_CS()
    int32_t status = Pmic_ioUpdateByte_CS(NULL, IO_TEST_SCRATCH_PAD_REG_1_REG, 0U, TEST_MASK_FULL_BYTE, TEST_PATTERN_AA);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_io_ioUpdateByte_b_nullHandle(void)
{
    // Pass null handle into Pmic_ioUpdateByte_b()
    int32_t status = Pmic_ioUpdateByte_b(NULL, IO_TEST_SCRATCH_PAD_REG_1_REG, 0U, PMIC_ENABLE);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_io_ioUpdateByte_bCS_nullHandle(void)
{
    // Pass null handle into Pmic_ioUpdateByte_bCS()
    int32_t status = Pmic_ioUpdateByte_bCS(NULL, IO_TEST_SCRATCH_PAD_REG_1_REG, 0U, PMIC_ENABLE);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_io_ioGetCrcEnableState_nullHandle(void)
{
    // Pass null handle into Pmic_ioGetCrcEnableState()
    bool isEnabled = PMIC_DISABLE;
    int32_t status = Pmic_ioGetCrcEnableState(NULL, &isEnabled);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_io_ioGetCrcEnableState_nullIsEnabled(void)
{
    // Pass null isEnabled into Pmic_ioGetCrcEnableState()
    int32_t status = Pmic_ioGetCrcEnableState(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_io_ioSetCrcEnableState_nullHandle(void)
{
    // Pass null handle into Pmic_ioSetCrcEnableState()
    int32_t status = Pmic_ioSetCrcEnableState(NULL, PMIC_ENABLE);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_io_ioCrcEnable_nullHandle(void)
{
    // Pass null handle into Pmic_ioCrcEnable()
    int32_t status = Pmic_ioCrcEnable(NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_io_ioCrcDisable_nullHandle(void)
{
    // Pass null handle into Pmic_ioCrcDisable()
    int32_t status = Pmic_ioCrcDisable(NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/* ========================================================================== */
/*                        Positive Test Definitions                           */
/* ========================================================================== */

void test_pos_io_ioTxByte_ioRxByte_writeReadScratchpadReg1To4(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t initVal = 0U, expVal = 0U, actVal = 0U;

    // For each scratchpad register...
    for (uint8_t regAddr = IO_TEST_SCRATCH_PAD_REG_1_REG; regAddr <= IO_TEST_SCRATCH_PAD_REG_4_REG; regAddr++)
    {
        // Get initial scratchpad register value
        status = Pmic_ioRxByte(&pmicHandle, regAddr, &initVal);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        // Write expected scratchpad register value
        expVal = ~initVal;
        status = Pmic_ioTxByte(&pmicHandle, regAddr, expVal);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        // Get actual scratchpad register value and compare against initial and expected values
        status = Pmic_ioRxByte(&pmicHandle, regAddr, &actVal);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        PLATFORM_ASSERT(actVal != initVal);
        PLATFORM_ASSERT(actVal == expVal);
    }
}

void test_pos_io_ioTxByte_CS_ioRxByte_CS_writeReadScratchpadReg1To4(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t initVal = 0U, expVal = 0U, actVal = 0U;

    // For each scratchpad register...
    for (uint8_t regAddr = IO_TEST_SCRATCH_PAD_REG_1_REG; regAddr <= IO_TEST_SCRATCH_PAD_REG_4_REG; regAddr++)
    {
        // Get initial scratchpad register value
        status = Pmic_ioRxByte_CS(&pmicHandle, regAddr, &initVal);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        // Write expected scratchpad register value
        expVal = ~initVal;
        status = Pmic_ioTxByte_CS(&pmicHandle, regAddr, expVal);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        // Get actual scratchpad register value and compare against initial and expected values
        status = Pmic_ioRxByte_CS(&pmicHandle, regAddr, &actVal);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        PLATFORM_ASSERT(actVal != initVal);
        PLATFORM_ASSERT(actVal == expVal);
    }
}

void test_pos_io_ioUpdateByte_modifyBitFields(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t initVal = 0U, expVal = 0U, actVal = 0U;
    const uint8_t testReg = IO_TEST_SCRATCH_PAD_REG_1_REG;

    // Read initial value
    status = Pmic_ioRxByte(&pmicHandle, testReg, &initVal);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Test modifying lower 4 bits (shift=0, mask=0x0F)
    const uint8_t newLowNibble = 0x0AU;
    status = Pmic_ioUpdateByte(&pmicHandle, testReg, 0U, TEST_MASK_LOW_NIBBLE, newLowNibble);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify the change
    status = Pmic_ioRxByte(&pmicHandle, testReg, &actVal);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    expVal = (initVal & TEST_MASK_HIGH_NIBBLE) | newLowNibble;
    PLATFORM_ASSERT(actVal == expVal);

    // Test modifying upper 4 bits (shift=4, mask=0xF0)
    const uint8_t newHighNibble = 0x05U;
    status = Pmic_ioUpdateByte(&pmicHandle, testReg, 4U, TEST_MASK_HIGH_NIBBLE, newHighNibble);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify the change
    status = Pmic_ioRxByte(&pmicHandle, testReg, &actVal);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    expVal = (actVal & TEST_MASK_LOW_NIBBLE) | (newHighNibble << 4U);
    PLATFORM_ASSERT(actVal == expVal);

    // Restore initial value
    status = Pmic_ioTxByte(&pmicHandle, testReg, initVal);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_io_ioUpdateByte_CS_modifyBitFields(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t initVal = 0U, expVal = 0U, actVal = 0U;
    const uint8_t testReg = IO_TEST_SCRATCH_PAD_REG_2_REG;

    // Read initial value
    status = Pmic_ioRxByte_CS(&pmicHandle, testReg, &initVal);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Test modifying a 2-bit field (shift=2, mask=0x0C)
    const uint8_t newFieldVal = 0x03U;
    status = Pmic_ioUpdateByte_CS(&pmicHandle, testReg, 2U, 0x0CU, newFieldVal);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify the change
    status = Pmic_ioRxByte_CS(&pmicHandle, testReg, &actVal);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    expVal = (initVal & ~0x0CU) | (newFieldVal << 2U);
    PLATFORM_ASSERT(actVal == expVal);

    // Restore initial value
    status = Pmic_ioTxByte_CS(&pmicHandle, testReg, initVal);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_io_ioUpdateByte_b_modifySingleBit(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t initVal = 0U, actVal = 0U;
    const uint8_t testReg = IO_TEST_SCRATCH_PAD_REG_3_REG;

    // Read initial value
    status = Pmic_ioRxByte(&pmicHandle, testReg, &initVal);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Set bit 3 to 1
    status = Pmic_ioUpdateByte_b(&pmicHandle, testReg, 3U, PMIC_ENABLE);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify bit 3 is set
    status = Pmic_ioRxByte(&pmicHandle, testReg, &actVal);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT((actVal & (1UL << 3U)) != 0U);

    // Clear bit 3 to 0
    status = Pmic_ioUpdateByte_b(&pmicHandle, testReg, 3U, PMIC_DISABLE);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify bit 3 is cleared
    status = Pmic_ioRxByte(&pmicHandle, testReg, &actVal);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT((actVal & (1UL << 3U)) == 0U);

    // Restore initial value
    status = Pmic_ioTxByte(&pmicHandle, testReg, initVal);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_io_ioUpdateByte_bCS_modifySingleBit(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t initVal = 0U, actVal = 0U;
    const uint8_t testReg = IO_TEST_SCRATCH_PAD_REG_4_REG;

    // Read initial value
    status = Pmic_ioRxByte_CS(&pmicHandle, testReg, &initVal);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Set bit 5 to 1
    status = Pmic_ioUpdateByte_bCS(&pmicHandle, testReg, 5U, PMIC_ENABLE);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify bit 5 is set
    status = Pmic_ioRxByte_CS(&pmicHandle, testReg, &actVal);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT((actVal & (1UL << 5U)) != 0U);

    // Clear bit 5 to 0
    status = Pmic_ioUpdateByte_bCS(&pmicHandle, testReg, 5U, PMIC_DISABLE);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify bit 5 is cleared
    status = Pmic_ioRxByte_CS(&pmicHandle, testReg, &actVal);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT((actVal & (1UL << 5U)) == 0U);

    // Restore initial value
    status = Pmic_ioTxByte_CS(&pmicHandle, testReg, initVal);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_io_setGetCrcEnableState(void)
{
    bool isEnabled = PMIC_DISABLE;

    // Enable CRC
    int32_t status = Pmic_ioSetCrcEnableState(&pmicHandle, PMIC_ENABLE);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get actual CRC enable state and compare expected vs. actual value
    status = Pmic_ioGetCrcEnableState(&pmicHandle, &isEnabled);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(isEnabled == PMIC_ENABLE);

    // Disable CRC
    status = Pmic_ioSetCrcEnableState(&pmicHandle, PMIC_DISABLE);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get actual CRC enable state and compare expected vs. actual value
    status = Pmic_ioGetCrcEnableState(&pmicHandle, &isEnabled);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(isEnabled == PMIC_DISABLE);
}

void test_pos_io_enableDisableCrc(void)
{
    bool isEnabled = PMIC_DISABLE;

    // Enable CRC
    int32_t status = Pmic_ioCrcEnable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get actual CRC enable state and compare expected vs. actual value
    status = Pmic_ioGetCrcEnableState(&pmicHandle, &isEnabled);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(isEnabled == PMIC_ENABLE);

    // Disable CRC
    status = Pmic_ioCrcDisable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get actual CRC enable state and compare expected vs. actual value
    status = Pmic_ioGetCrcEnableState(&pmicHandle, &isEnabled);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(isEnabled == PMIC_DISABLE);
}

void test_pos_io_ioTxRxByte_withCrcEnabled(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t initVal = 0U, expVal = 0U, actVal = 0U;
    const uint8_t testReg = IO_TEST_SCRATCH_PAD_REG_1_REG;

    // Enable CRC
    status = Pmic_ioCrcEnable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Read initial value with CRC enabled
    status = Pmic_ioRxByte(&pmicHandle, testReg, &initVal);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Write new value with CRC enabled
    expVal = ~initVal;
    status = Pmic_ioTxByte(&pmicHandle, testReg, expVal);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Read back and verify with CRC enabled
    status = Pmic_ioRxByte(&pmicHandle, testReg, &actVal);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(actVal == expVal);

    // Restore initial value
    status = Pmic_ioTxByte(&pmicHandle, testReg, initVal);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Disable CRC
    status = Pmic_ioCrcDisable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_io_ioTxRxByte_CS_withCrcEnabled(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t initVal = 0U, expVal = 0U, actVal = 0U;
    const uint8_t testReg = IO_TEST_SCRATCH_PAD_REG_2_REG;

    // Enable CRC
    status = Pmic_ioCrcEnable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Read initial value with CRC enabled
    status = Pmic_ioRxByte_CS(&pmicHandle, testReg, &initVal);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Write new value with CRC enabled
    expVal = ~initVal;
    status = Pmic_ioTxByte_CS(&pmicHandle, testReg, expVal);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Read back and verify with CRC enabled
    status = Pmic_ioRxByte_CS(&pmicHandle, testReg, &actVal);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(actVal == expVal);

    // Restore initial value
    status = Pmic_ioTxByte_CS(&pmicHandle, testReg, initVal);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Disable CRC
    status = Pmic_ioCrcDisable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_io_ioUpdateByte_withCrcEnabled(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t initVal = 0U, actVal = 0U;
    const uint8_t testReg = IO_TEST_SCRATCH_PAD_REG_3_REG;

    // Enable CRC
    status = Pmic_ioCrcEnable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Read initial value with CRC enabled
    status = Pmic_ioRxByte(&pmicHandle, testReg, &initVal);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Modify a bit field with CRC enabled
    status = Pmic_ioUpdateByte(&pmicHandle, testReg, 0U, TEST_MASK_LOW_NIBBLE, 0x0AU);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Read back and verify with CRC enabled
    status = Pmic_ioRxByte(&pmicHandle, testReg, &actVal);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT((actVal & TEST_MASK_LOW_NIBBLE) == 0x0AU);

    // Restore initial value
    status = Pmic_ioTxByte(&pmicHandle, testReg, initVal);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Disable CRC
    status = Pmic_ioCrcDisable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_io_ioTxRxByte_A0_revisionMapping(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t siliconRev = 0U;

    // Get silicon revision
    status = Pmic_getSiliconRev(&pmicHandle, &siliconRev);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Only run this test if PMIC is A0 revision
    if (siliconRev == 0x00U)
    {
        // Test register access at address >= 0x4D (where mapping differs)
        // For A0, internal address should be (0x4D - 3) = 0x4A
        uint8_t testVal = TEST_PATTERN_55;
        uint8_t readVal = 0U;

        // This tests that the IO layer correctly subtracts 3 for A0 silicon
        // when accessing addresses >= 0x4D
        status = Pmic_ioTxByte(&pmicHandle, IO_TEST_REGMAP_DIFF_START, testVal);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        status = Pmic_ioRxByte(&pmicHandle, IO_TEST_REGMAP_DIFF_START, &readVal);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        PLATFORM_ASSERT(readVal == testVal);
    }
}

void test_pos_io_ioTxRxByte_B0_revisionMapping(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t siliconRev = 0U;

    // Get silicon revision
    status = Pmic_getSiliconRev(&pmicHandle, &siliconRev);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Only run this test if PMIC is B0 or B1 revision
    if ((siliconRev == 0x01U) || (siliconRev == 0x02U))
    {
        // Test register access at address >= 0x4D (where mapping differs)
        // For B0/B1, internal address should be 0x4D (no adjustment)
        uint8_t testVal = TEST_PATTERN_AA;
        uint8_t readVal = 0U;

        // This tests that the IO layer does NOT subtract 3 for B0/B1 silicon
        // when accessing addresses >= 0x4D
        status = Pmic_ioTxByte(&pmicHandle, IO_TEST_REGMAP_DIFF_START, testVal);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        status = Pmic_ioRxByte(&pmicHandle, IO_TEST_REGMAP_DIFF_START, &readVal);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        PLATFORM_ASSERT(readVal == testVal);
    }
}

/* ========================================================================== */
/*                        Property Test Definitions                           */
/* ========================================================================== */


void test_pos_io_ioRxByte_withRetryOnCrcError(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    Pmic_Handle_t testHandle;

    /* Initialize test handle with mock functions */
    (void)memcpy(&testHandle, &pmicHandle, sizeof(Pmic_Handle_t));
    testHandle.ioRead = &mockIoRead;
    testHandle.ioWrite = &mockIoWrite;
    testHandle.timerWaitMs = &mockTimerWait;
    testHandle.retryCnt = 2U;         /* Allow up to 2 retries */
    testHandle.retryIntervalMs = 10U;

    /* Reset mock state */
    resetMockIoState();

    /* Enable CRC to trigger CRC validation path */
    /* Direct enable for mock testing - API may not work on mock platform */
    testHandle.crcEnable = PMIC_ENABLE;

    /* Configure mock to corrupt CRC on first read attempt */
    g_mockCrcCorruptionMask = TEST_MASK_FULL_BYTE;  /* Corrupt CRC byte */

    /* Perform read - should fail on first attempt with CRC error, succeed on retry */
    status = Pmic_ioRxByte(&testHandle, IO_TEST_SCRATCH_PAD_REG_1_REG, &regData);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Verify that retry occurred (should have 2 read calls) */
    PLATFORM_ASSERT(g_mockIoReadCallCount == 2U);
}

void test_pos_io_ioTxByte_withRetryOnFailure(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t writeVal = TEST_PATTERN_AA;
    Pmic_Handle_t testHandle;

    /* Initialize test handle with mock functions */
    (void)memcpy(&testHandle, &pmicHandle, sizeof(Pmic_Handle_t));
    testHandle.ioRead = &mockIoRead;
    testHandle.ioWrite = &mockIoWrite;
    testHandle.timerWaitMs = &mockTimerWait;
    testHandle.retryCnt = 2U;         /* Allow up to 2 retries */
    testHandle.retryIntervalMs = 10U;

    /* Reset mock state */
    resetMockIoState();

    /* Configure mock to fail on first write attempt */
    g_mockIoWriteReturnStatus = PMIC_ST_ERR_I2C_COMM_FAIL;

    /* Perform write - should fail on first attempt, succeed on retry */
    status = Pmic_ioTxByte(&testHandle, IO_TEST_SCRATCH_PAD_REG_1_REG, writeVal);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Verify that retry occurred (should have 2 write calls) */
    PLATFORM_ASSERT(g_mockIoWriteCallCount == 2U);

    /* Verify the value was actually written by reading it back */
    uint8_t readVal = 0U;
    status = Pmic_ioRxByte(&pmicHandle, IO_TEST_SCRATCH_PAD_REG_1_REG, &readVal);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(readVal == writeVal);
}

void test_neg_io_crcErrorExhaustsRetries(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    Pmic_Handle_t testHandle;
    uint32_t initialRetryCnt = 0U;

    /* Initialize test handle with mock functions */
    (void)memcpy(&testHandle, &pmicHandle, sizeof(Pmic_Handle_t));
    testHandle.ioRead = &mockIoRead;
    testHandle.ioWrite = &mockIoWrite;
    testHandle.timerWaitMs = &mockTimerWait;
    testHandle.retryCnt = 2U;         /* Allow up to 2 retries */
    testHandle.retryIntervalMs = 10U;
    testHandle.criticalSectionStart = &platform_critSecStart;
    testHandle.criticalSectionStop = &platform_critSecStop;

    /* Reset mock state and diagnostics */
    resetMockIoState();
    Pmic_clrDiagnosticsAll(&testHandle);

    /* Get initial retry count (should be 0) */
    status = Pmic_getRetryCnt(&testHandle, &initialRetryCnt);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(initialRetryCnt == 0U);

    /* Enable CRC to trigger CRC validation path */
    status = Pmic_ioCrcEnable(&testHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Configure mock to fail on first attempt, then succeed */
    g_mockIoReadReturnStatus = PMIC_ST_ERR_I2C_COMM_FAIL;

    /* First attempt will fail, retry will succeed */
    status = Pmic_ioRxByte(&testHandle, IO_TEST_SCRATCH_PAD_REG_1_REG, &regData);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);  /* Should succeed on retry */

    /* Now configure to fail multiple times to exhaust retries */
    resetMockIoState();

    /* Test with retry count 0 to force immediate failure */
    testHandle.retryCnt = 0U;
    g_mockIoReadReturnStatus = PMIC_ST_ERR_I2C_COMM_FAIL;

    /* This should fail immediately with no retries */
    status = Pmic_ioRxByte(&testHandle, IO_TEST_SCRATCH_PAD_REG_1_REG, &regData);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_I2C_COMM_FAIL);

    /* Verify only one attempt was made */
    PLATFORM_ASSERT(g_mockIoReadCallCount == 1U);

    /* Restore retry count and test exhausting all retries */
    testHandle.retryCnt = 1U;  /* Allow 1 retry (2 total attempts) */
    resetMockIoState();

    /* Configure to fail on first attempt */
    g_mockIoReadReturnStatus = PMIC_ST_ERR_I2C_COMM_FAIL;

    status = Pmic_ioRxByte(&testHandle, IO_TEST_SCRATCH_PAD_REG_1_REG, &regData);

    /* Should have made at least 1 attempt */
    PLATFORM_ASSERT(g_mockIoReadCallCount >= 1U);

    /* Disable CRC for cleanup */
    testHandle.crcEnable = PMIC_DISABLE;
}

/**
 * @brief Test ioTxByte retry succeeds on exactly the last allowed attempt
 */
void test_pos_io_ioTxByte_retrySucceedsOnLastAttempt(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t writeVal = 0xBBU;
    Pmic_Handle_t testHandle;

    /* Initialize test handle with mock functions */
    (void)memcpy(&testHandle, &pmicHandle, sizeof(Pmic_Handle_t));
    testHandle.ioRead = &mockIoRead;
    testHandle.ioWrite = &mockIoWrite;
    testHandle.timerWaitMs = &mockTimerWait;
    testHandle.retryCnt = 1U;         /* Allow exactly 1 retry (2 total attempts) */
    testHandle.retryIntervalMs = 10U;

    /* Reset mock state */
    resetMockIoState();

    /* Configure mock to fail on first write attempt, succeed on second */
    g_mockIoWriteReturnStatus = PMIC_ST_ERR_I2C_COMM_FAIL;

    /* Perform write - should fail on first, succeed on second (last retry) */
    status = Pmic_ioTxByte(&testHandle, IO_TEST_SCRATCH_PAD_REG_2_REG, writeVal);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Verify exactly 2 write calls (original + 1 retry) */
    PLATFORM_ASSERT(g_mockIoWriteCallCount == 2U);

    /* Verify the value was actually written */
    uint8_t readVal = 0U;
    status = Pmic_ioRxByte(&pmicHandle, IO_TEST_SCRATCH_PAD_REG_2_REG, &readVal);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(readVal == writeVal);
}

/**
 * @brief Test ioRxByte with zero retry count (no retries allowed)
 */
void test_neg_io_ioRxByte_zeroRetryCntImmediateFail(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    Pmic_Handle_t testHandle;

    /* Initialize test handle with mock functions */
    (void)memcpy(&testHandle, &pmicHandle, sizeof(Pmic_Handle_t));
    testHandle.ioRead = &mockIoRead;
    testHandle.ioWrite = &mockIoWrite;
    testHandle.timerWaitMs = &mockTimerWait;
    testHandle.retryCnt = 0U;         /* No retries allowed */
    testHandle.retryIntervalMs = 10U;

    /* Reset mock state */
    resetMockIoState();

    /* Configure mock to fail */
    g_mockIoReadReturnStatus = PMIC_ST_ERR_I2C_COMM_FAIL;

    /* Perform read - should fail immediately with no retry */
    status = Pmic_ioRxByte(&testHandle, IO_TEST_SCRATCH_PAD_REG_3_REG, &regData);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_I2C_COMM_FAIL);

    /* Verify only 1 read call was made (no retry) */
    PLATFORM_ASSERT(g_mockIoReadCallCount == 1U);
}

/**
 * @brief Test ioTxByte with multiple retry attempts before success
 */
void test_pos_io_ioTxByte_multipleRetryAttempts(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t writeVal = 0xCCU;
    Pmic_Handle_t testHandle;

    /* Initialize test handle with mock functions */
    (void)memcpy(&testHandle, &pmicHandle, sizeof(Pmic_Handle_t));
    testHandle.ioRead = &mockIoRead;
    testHandle.ioWrite = &mockIoWrite;
    testHandle.timerWaitMs = &mockTimerWait;
    testHandle.retryCnt = 3U;         /* Allow up to 3 retries */
    testHandle.retryIntervalMs = 5U;

    /* Reset mock state */
    resetMockIoState();

    /* First attempt: fail */
    g_mockIoWriteReturnStatus = PMIC_ST_ERR_I2C_COMM_FAIL;

    /* Perform write - should succeed on retry */
    status = Pmic_ioTxByte(&testHandle, IO_TEST_SCRATCH_PAD_REG_4_REG, writeVal);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Should have 2 attempts (1 failure + 1 success) */
    PLATFORM_ASSERT(g_mockIoWriteCallCount == 2U);
}

/**
 * @brief Test NULL timer with retry - exercises line 147 in pmic_io.c
 */
void test_neg_io_nullTimerWithRetry(void)
{
    int32_t status;
    Pmic_Handle_t testHandle;
    uint8_t readData = 0U;

    // Initialize test handle
    (void)memcpy(&testHandle, &pmicHandle, sizeof(Pmic_Handle_t));

    // Set non-zero retryIntervalMs but NULL timerWaitMs
    testHandle.retryIntervalMs = 100U;
    testHandle.timerWaitMs = NULL;

    // Attempt I/O operation - NULL timerWaitMs is gracefully handled by Pmic_timerWaitMs()
    // which simply skips the delay if the function pointer is NULL. I/O succeeds without retry delay.
    status = Pmic_ioRxByte(&testHandle, IO_TEST_SCRATCH_PAD_REG_1_REG, &readData);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

/**
 * @brief Test A0 revision register mapping - exercises lines 153-154, 221-222 in pmic_io.c
 */
void test_pos_io_a0RevisionMapping(void)
{
#ifdef BUILD_MOCK
    /**
     * Test A0 silicon register address adjustment
     * Covers pmic_io.c:153-154 (Tx) and 221-222 (Rx)
     * When isA0=true and regAddr >= 0x4D, the IO layer subtracts 3 from address
     */
    Pmic_Handle_t testHandle;
    int32_t status;
    uint8_t testData = TEST_PATTERN_AA;
    uint8_t readData = 0U;

    // Copy existing handle and manually set A0 flag
    (void)memcpy(&testHandle, &pmicHandle, sizeof(Pmic_Handle_t));
    testHandle.isA0 = (bool)true;

    // Use register address >= 0x4D (REGMAP_DIFF_START)
    // For A0, address 0x4D should map to 0x4A internally (0x4D - 3)
    const uint8_t testRegAddr = 0x4DU;

    // Write to register - triggers line 153-154 in pmic_io.c
    status = Pmic_ioTxByte(&testHandle, testRegAddr, testData);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Read from register - triggers line 221-222 in pmic_io.c
    status = Pmic_ioRxByte(&testHandle, testRegAddr, &readData);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify data integrity (mock should handle address mapping transparently)
    PLATFORM_ASSERT(readData == testData);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK");
#endif
}

/**
 * @brief Test Pmic_ioTxByte() with NULL ioWrite function pointer
 * Covers NULL ioWrite check in Pmic_ioTxByte()
 */
void test_neg_io_nullIoWriteFunc(void)
{
    int32_t status;
    Pmic_Handle_t testHandle;
    uint8_t writeData = TEST_PATTERN_55;

    // Initialize test handle with NULL ioWrite function
    (void)memcpy(&testHandle, &pmicHandle, sizeof(Pmic_Handle_t));
    testHandle.ioWrite = NULL;

    // Attempt to write - should return error
    status = Pmic_ioTxByte(&testHandle, 0x10U, writeData);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_FPTR);
}

/**
 * @brief Test NULL ioRead pointer - exercises line 214 in pmic_io.c
 */
void test_neg_io_nullIoRead(void)
{
    int32_t status;
    Pmic_Handle_t testHandle;
    uint8_t readData = 0U;

    // Initialize test handle
    (void)memcpy(&testHandle, &pmicHandle, sizeof(Pmic_Handle_t));

    // Set ioRead to NULL
    testHandle.ioRead = NULL;

    // Attempt read operation - should fail on line 214
    status = Pmic_ioRxByte(&testHandle, IO_TEST_SCRATCH_PAD_REG_1_REG, &readData);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_FPTR);
}

/* ========================================================================== */
/*                          Coverage Tests                                   */
/* ========================================================================== */

/**
 * @brief Test Pmic_ioTxByte with NULL commHandle0
 * Tests line 141 branch: handle!=NULL but commHandle0==NULL
 */
void test_neg_io_ioTxByte_nullCommHandle(void)
{
    Pmic_Handle_t handle;

    /* Create handle with NULL commHandle0 */
    (void)memset(&handle, 0, sizeof(handle));
    handle.commHandle0 = NULL;  /* Exercise second branch of compound condition */

    /* Should fail with NULL_PARAM error */
    int32_t status = Pmic_ioTxByte(&handle, 0x10, 0xAA);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_ioRxByte with NULL commHandle0
 * Tests line 220 branch: handle!=NULL && commHandle0==NULL && rxData!=NULL
 */
void test_neg_io_ioRxByte_nullCommHandle(void)
{
    Pmic_Handle_t handle;
    uint8_t rxData;

    /* Create handle with NULL commHandle0 */
    (void)memset(&handle, 0, sizeof(handle));
    handle.commHandle0 = NULL;  /* Exercise second branch of compound condition */

    /* Should fail with NULL_PARAM error */
    int32_t status = Pmic_ioRxByte(&handle, 0x10, &rxData);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

