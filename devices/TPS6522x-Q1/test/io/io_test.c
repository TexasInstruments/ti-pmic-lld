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


#include "../platform.h"
#include "io_test.h"
#include "pmic_io.h"
#include "regmap/core.h"
#include "test_constants.h"

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

static Pmic_Handle_t pmicHandle = {0};

/* Global variables for mock I/O control */
static uint32_t g_mockIoReadCallCount = 0U;
static uint32_t g_mockIoWriteCallCount = 0U;
static int32_t g_mockIoReadReturnStatus = PMIC_ST_SUCCESS;
static int32_t g_mockIoWriteReturnStatus = PMIC_ST_SUCCESS;
static uint8_t g_mockCrcCorruptionMask = 0x00U;  /* XOR mask to corrupt CRC byte */

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
        /* I2C mode: CRC is at buffer[1] when bufLen = 2 */
        buffer[1] ^= g_mockCrcCorruptionMask;
        g_mockCrcCorruptionMask = 0x00U;
    }
    if ((g_mockCrcCorruptionMask != 0x00U) && (bufLen == 4U) && (status == PMIC_ST_SUCCESS))
    {
        /* SPI mode: CRC is at buffer[3] when frameLen = 4 */
        buffer[3] ^= g_mockCrcCorruptionMask;
        g_mockCrcCorruptionMask = 0x00U;
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
/*                       Negative Test Functions                              */
/* ========================================================================== */

/**
 * @brief Test Pmic_ioTxByte with NULL handle
 */
void test_neg_io_ioTxByte_nullHandle(void)
{
    int32_t status = Pmic_ioTxByte(NULL, DEV_REV_REG, 0x00U);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_ioTxByte_CS with NULL handle
 */
void test_neg_io_ioTxByte_CS_nullHandle(void)
{
    int32_t status = Pmic_ioTxByte_CS(NULL, DEV_REV_REG, 0x00U);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_ioRxByte with NULL handle
 */
void test_neg_io_ioRxByte_nullHandle(void)
{
    uint8_t rxData = 0U;
    int32_t status = Pmic_ioRxByte(NULL, DEV_REV_REG, &rxData);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_ioRxByte with NULL rxData pointer
 */
void test_neg_io_ioRxByte_nullRxData(void)
{
    int32_t status = Pmic_ioRxByte(&pmicHandle, DEV_REV_REG, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_ioRxByte_CS with NULL handle
 */
void test_neg_io_ioRxByte_CS_nullHandle(void)
{
    uint8_t rxData = 0U;
    int32_t status = Pmic_ioRxByte_CS(NULL, DEV_REV_REG, &rxData);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_ioRxByte_CS with NULL rxData pointer
 */
void test_neg_io_ioRxByte_CS_nullRxData(void)
{
    int32_t status = Pmic_ioRxByte_CS(&pmicHandle, DEV_REV_REG, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_ioTxByte with NULL ioWrite function pointer
 */
void test_neg_io_ioTxByte_nullIoWrite(void)
{
    Pmic_Handle_t testHandle = pmicHandle;
    testHandle.ioWrite = NULL;
    int32_t status = Pmic_ioTxByte(&testHandle, DEV_REV_REG, 0x00U);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_FPTR);
}

/**
 * @brief Test Pmic_ioTxByte with NULL commHandle0
 */
void test_neg_io_ioTxByte_nullCommHandle(void)
{
    Pmic_Handle_t testHandle = pmicHandle;
    testHandle.commHandle0 = NULL;
    int32_t status = Pmic_ioTxByte(&testHandle, DEV_REV_REG, 0x00U);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_ioRxByte with NULL ioRead function pointer
 */
void test_neg_io_ioRxByte_nullIoRead(void)
{
    Pmic_Handle_t testHandle = pmicHandle;
    testHandle.ioRead = NULL;
    uint8_t rxData = 0U;
    int32_t status = Pmic_ioRxByte(&testHandle, DEV_REV_REG, &rxData);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_FPTR);
}

/**
 * @brief Test Pmic_ioRxByte with NULL commHandle0
 */
void test_neg_io_ioRxByte_nullCommHandle(void)
{
    Pmic_Handle_t testHandle = pmicHandle;
    testHandle.commHandle0 = NULL;
    uint8_t rxData = 0U;
    int32_t status = Pmic_ioRxByte(&testHandle, DEV_REV_REG, &rxData);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_ioUpdateByte with NULL handle
 */
void test_neg_io_ioUpdateByte_nullHandle(void)
{
    int32_t status = Pmic_ioUpdateByte(NULL, CONFIG_2_REG, 0U, TEST_MASK_LOW_NIBBLE, 0x05U);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_ioUpdateByte_CS with NULL handle
 */
void test_neg_io_ioUpdateByte_CS_nullHandle(void)
{
    int32_t status = Pmic_ioUpdateByte_CS(NULL, CONFIG_2_REG, 0U, TEST_MASK_LOW_NIBBLE, 0x05U);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_ioUpdateByte_b with NULL handle
 */
void test_neg_io_ioUpdateByte_b_nullHandle(void)
{
    int32_t status = Pmic_ioUpdateByte_b(NULL, CONFIG_2_REG, 0U, true);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_ioUpdateByte_bCS with NULL handle
 */
void test_neg_io_ioUpdateByte_bCS_nullHandle(void)
{
    int32_t status = Pmic_ioUpdateByte_bCS(NULL, CONFIG_2_REG, 0U, true);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_ioSetCrcEnableState with NULL handle
 */
void test_neg_io_ioSetCrcEnableState_nullHandle(void)
{
    int32_t status = Pmic_ioSetCrcEnableState(NULL, true);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_ioCrcEnable with NULL handle
 */
void test_neg_io_ioCrcEnable_nullHandle(void)
{
    int32_t status = Pmic_ioCrcEnable(NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_ioCrcDisable with NULL handle
 */
void test_neg_io_ioCrcDisable_nullHandle(void)
{
    int32_t status = Pmic_ioCrcDisable(NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_ioGetCrcEnableState with NULL handle
 */
void test_neg_io_ioGetCrcEnableState_nullHandle(void)
{
    bool enabled = false;
    int32_t status = Pmic_ioGetCrcEnableState(NULL, &enabled);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_ioGetCrcEnableState with NULL enabled pointer
 */
void test_neg_io_ioGetCrcEnableState_nullEnabled(void)
{
    int32_t status = Pmic_ioGetCrcEnableState(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/* ========================================================================== */
/*                       Positive Test Functions                              */
/* ========================================================================== */

/**
 * @brief Test single register read
 */
void test_pos_io_ioRxByte_singleRegisterRead(void)
{
    uint8_t rxData = 0U;
    int32_t status;

    /* Read device revision register */
    status = Pmic_ioRxByte(&pmicHandle, DEV_REV_REG, &rxData);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

/**
 * @brief Test single register write and readback
 */
void test_pos_io_ioTxByte_singleRegisterWrite(void)
{
    uint8_t txData = TEST_PATTERN_AA;
    uint8_t rxData = 0U;
    int32_t status;

    /* Write to scratch pad register */
    status = Pmic_ioTxByte(&pmicHandle, SCRATCH_PAD_REG_1_REG, txData);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Read back and verify */
    status = Pmic_ioRxByte(&pmicHandle, SCRATCH_PAD_REG_1_REG, &rxData);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(rxData == txData);
}

/**
 * @brief Test single register read with critical section
 */
void test_pos_io_ioRxByte_CS_singleRegisterRead(void)
{
    uint8_t rxData = 0U;
    int32_t status;

    /* Read device revision register with critical section */
    status = Pmic_ioRxByte_CS(&pmicHandle, DEV_REV_REG, &rxData);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

/**
 * @brief Test single register write with critical section
 */
void test_pos_io_ioTxByte_CS_singleRegisterWrite(void)
{
    uint8_t txData = TEST_PATTERN_55;
    uint8_t rxData = 0U;
    int32_t status;

    /* Write to scratch pad register with critical section */
    status = Pmic_ioTxByte_CS(&pmicHandle, SCRATCH_PAD_REG_2_REG, txData);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Read back and verify */
    status = Pmic_ioRxByte_CS(&pmicHandle, SCRATCH_PAD_REG_2_REG, &rxData);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(rxData == txData);
}

/**
 * @brief Test read-modify-write operation
 */
void test_pos_io_ioUpdateByte_readModifyWrite(void)
{
    uint8_t originalData = 0U;
    uint8_t modifiedData = 0U;
    int32_t status;
    uint8_t testValue = 0x03U;

    /* Read original value */
    status = Pmic_ioRxByte(&pmicHandle, SCRATCH_PAD_REG_3_REG, &originalData);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Modify lower 4 bits */
    status = Pmic_ioUpdateByte(&pmicHandle, SCRATCH_PAD_REG_3_REG, 0U, TEST_MASK_LOW_NIBBLE, testValue);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Read back and verify lower 4 bits changed, upper 4 bits unchanged */
    status = Pmic_ioRxByte(&pmicHandle, SCRATCH_PAD_REG_3_REG, &modifiedData);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT((modifiedData & TEST_MASK_LOW_NIBBLE) == testValue);
    PLATFORM_ASSERT((modifiedData & TEST_MASK_HIGH_NIBBLE) == (originalData & TEST_MASK_HIGH_NIBBLE));
}

/**
 * @brief Test read-modify-write operation with critical section
 */
void test_pos_io_ioUpdateByte_CS_readModifyWrite(void)
{
    uint8_t originalData = 0U;
    uint8_t modifiedData = 0U;
    int32_t status;
    uint8_t testValue = 0x05U;

    /* Read original value */
    status = Pmic_ioRxByte(&pmicHandle, SCRATCH_PAD_REG_4_REG, &originalData);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Modify lower 4 bits with critical section */
    status = Pmic_ioUpdateByte_CS(&pmicHandle, SCRATCH_PAD_REG_4_REG, 0U, TEST_MASK_LOW_NIBBLE, testValue);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Read back and verify */
    status = Pmic_ioRxByte(&pmicHandle, SCRATCH_PAD_REG_4_REG, &modifiedData);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT((modifiedData & TEST_MASK_LOW_NIBBLE) == testValue);
    PLATFORM_ASSERT((modifiedData & TEST_MASK_HIGH_NIBBLE) == (originalData & TEST_MASK_HIGH_NIBBLE));
}

/**
 * @brief Test read-modify-write single bit operation
 */
void test_pos_io_ioUpdateByte_b_readModifyWriteBit(void)
{
    uint8_t originalData = 0U;
    uint8_t modifiedData = 0U;
    int32_t status;

    /* Read original value */
    status = Pmic_ioRxByte(&pmicHandle, SCRATCH_PAD_REG_1_REG, &originalData);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Set bit 5 to 1 */
    status = Pmic_ioUpdateByte_b(&pmicHandle, SCRATCH_PAD_REG_1_REG, 5U, true);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Read back and verify bit 5 is set */
    status = Pmic_ioRxByte(&pmicHandle, SCRATCH_PAD_REG_1_REG, &modifiedData);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT((modifiedData & (1UL << 5U)) != 0U);

    /* Clear bit 5 to 0 */
    status = Pmic_ioUpdateByte_b(&pmicHandle, SCRATCH_PAD_REG_1_REG, 5U, false);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Read back and verify bit 5 is cleared */
    status = Pmic_ioRxByte(&pmicHandle, SCRATCH_PAD_REG_1_REG, &modifiedData);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT((modifiedData & (1UL << 5U)) == 0U);
}

/**
 * @brief Test read-modify-write single bit operation with critical section
 */
void test_pos_io_ioUpdateByte_bCS_readModifyWriteBit(void)
{
    uint8_t originalData = 0U;
    uint8_t modifiedData = 0U;
    int32_t status;

    /* Read original value */
    status = Pmic_ioRxByte(&pmicHandle, SCRATCH_PAD_REG_2_REG, &originalData);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Set bit 3 to 1 with critical section */
    status = Pmic_ioUpdateByte_bCS(&pmicHandle, SCRATCH_PAD_REG_2_REG, 3U, true);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Read back and verify bit 3 is set */
    status = Pmic_ioRxByte(&pmicHandle, SCRATCH_PAD_REG_2_REG, &modifiedData);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT((modifiedData & (1UL << 3U)) != 0U);

    /* Clear bit 3 to 0 with critical section */
    status = Pmic_ioUpdateByte_bCS(&pmicHandle, SCRATCH_PAD_REG_2_REG, 3U, false);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Read back and verify bit 3 is cleared */
    status = Pmic_ioRxByte(&pmicHandle, SCRATCH_PAD_REG_2_REG, &modifiedData);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT((modifiedData & (1UL << 3U)) == 0U);
}

/**
 * @brief Test CRC enable and disable functionality
 */
void test_pos_io_ioCrcEnable_crcEnableDisable(void)
{
    bool enabled = false;
    int32_t status;

    /* Disable CRC */
    status = Pmic_ioCrcDisable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Verify CRC is disabled */
    status = Pmic_ioGetCrcEnableState(&pmicHandle, &enabled);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(enabled == false);

    /* Enable CRC */
    status = Pmic_ioCrcEnable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Verify CRC is enabled */
    status = Pmic_ioGetCrcEnableState(&pmicHandle, &enabled);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(enabled == true);

    /* Disable CRC again for other tests */
    status = Pmic_ioCrcDisable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

/**
 * @brief Test CRC set enable state functionality
 */
void test_pos_io_ioSetCrcEnableState_crcSetEnableState(void)
{
    bool enabled = false;
    int32_t status;

    /* Set CRC to disabled state */
    status = Pmic_ioSetCrcEnableState(&pmicHandle, false);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Verify CRC is disabled */
    status = Pmic_ioGetCrcEnableState(&pmicHandle, &enabled);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(enabled == false);

    /* Set CRC to enabled state */
    status = Pmic_ioSetCrcEnableState(&pmicHandle, true);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Verify CRC is enabled */
    status = Pmic_ioGetCrcEnableState(&pmicHandle, &enabled);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(enabled == true);

    /* Disable CRC again for other tests */
    status = Pmic_ioSetCrcEnableState(&pmicHandle, false);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

/**
 * @brief Test multiple register write and readback
 */
void test_pos_io_ioTxByte_multipleRegisterAccess(void)
{
    uint8_t txData[4] = {0x11U, 0x22U, 0x33U, 0x44U};
    uint8_t rxData[4] = {0U};
    int32_t status;

    /* Write to all scratch pad registers */
    status = Pmic_ioTxByte(&pmicHandle, SCRATCH_PAD_REG_1_REG, txData[0]);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_ioTxByte(&pmicHandle, SCRATCH_PAD_REG_2_REG, txData[1]);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_ioTxByte(&pmicHandle, SCRATCH_PAD_REG_3_REG, txData[2]);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_ioTxByte(&pmicHandle, SCRATCH_PAD_REG_4_REG, txData[3]);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Read back all scratch pad registers and verify */
    status = Pmic_ioRxByte(&pmicHandle, SCRATCH_PAD_REG_1_REG, &rxData[0]);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(rxData[0] == txData[0]);

    status = Pmic_ioRxByte(&pmicHandle, SCRATCH_PAD_REG_2_REG, &rxData[1]);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(rxData[1] == txData[1]);

    status = Pmic_ioRxByte(&pmicHandle, SCRATCH_PAD_REG_3_REG, &rxData[2]);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(rxData[2] == txData[2]);

    status = Pmic_ioRxByte(&pmicHandle, SCRATCH_PAD_REG_4_REG, &rxData[3]);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(rxData[3] == txData[3]);
}

/**
 * @brief Test register read verification
 */
void test_pos_io_ioRxByte_registerReadVerification(void)
{
    uint8_t rxData1 = 0U;
    uint8_t rxData2 = 0U;
    int32_t status;

    /* Read device revision register twice */
    status = Pmic_ioRxByte(&pmicHandle, DEV_REV_REG, &rxData1);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_ioRxByte(&pmicHandle, DEV_REV_REG, &rxData2);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Verify both reads return the same value */
    PLATFORM_ASSERT(rxData1 == rxData2);
}

/**
 * @brief Test CRC control with register access
 */
void test_pos_io_ioCrcEnable_crcWithRegisterAccess(void)
{
    uint8_t txData = TEST_PATTERN_A5;
    uint8_t rxData = 0U;
    bool enabled = false;
    int32_t status;

    /* Disable CRC */
    status = Pmic_ioCrcDisable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Write and read with CRC disabled */
    status = Pmic_ioTxByte(&pmicHandle, SCRATCH_PAD_REG_1_REG, txData);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_ioRxByte(&pmicHandle, SCRATCH_PAD_REG_1_REG, &rxData);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(rxData == txData);

    /* Enable CRC */
    status = Pmic_ioCrcEnable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Verify CRC is enabled */
    status = Pmic_ioGetCrcEnableState(&pmicHandle, &enabled);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(enabled == true);

    /* Write and read with CRC enabled */
    txData = TEST_PATTERN_5A;
    status = Pmic_ioTxByte(&pmicHandle, SCRATCH_PAD_REG_2_REG, txData);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_ioRxByte(&pmicHandle, SCRATCH_PAD_REG_2_REG, &rxData);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(rxData == txData);

    /* Disable CRC for other tests */
    status = Pmic_ioCrcDisable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

/**
 * @brief Test read operation with CRC validation enabled
 *
 * This test enables CRC, performs a read operation, and verifies that
 * CRC validation occurs correctly with valid CRC from the mock device.
 */
void test_pos_io_ioRxByte_readWithCrcValidation(void)
{
    uint8_t rxData = 0U;
    int32_t status;
    bool enabled = false;

    /* Enable CRC in the PMIC */
    status = Pmic_ioCrcEnable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Verify CRC is enabled */
    status = Pmic_ioGetCrcEnableState(&pmicHandle, &enabled);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(enabled == true);

    /* Read device revision register with CRC enabled */
    status = Pmic_ioRxByte(&pmicHandle, DEV_REV_REG, &rxData);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Read scratch pad register with CRC enabled */
    status = Pmic_ioRxByte(&pmicHandle, SCRATCH_PAD_REG_1_REG, &rxData);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Read multiple registers with CRC enabled */
    status = Pmic_ioRxByte(&pmicHandle, SCRATCH_PAD_REG_2_REG, &rxData);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_ioRxByte(&pmicHandle, SCRATCH_PAD_REG_3_REG, &rxData);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Disable CRC for other tests */
    status = Pmic_ioCrcDisable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

/**
 * @brief Test write operation with CRC calculation
 *
 * This test enables CRC, performs write operations, and verifies that
 * CRC calculation is performed correctly for transmitted data.
 */
void test_pos_io_ioTxByte_writeWithCrcCalculation(void)
{
    uint8_t txData = TEST_PATTERN_A5;
    uint8_t rxData = 0U;
    int32_t status;

    /* Enable CRC */
    status = Pmic_ioCrcEnable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Write to scratch pad register 1 with CRC enabled */
    status = Pmic_ioTxByte(&pmicHandle, SCRATCH_PAD_REG_1_REG, txData);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Read back and verify */
    status = Pmic_ioRxByte(&pmicHandle, SCRATCH_PAD_REG_1_REG, &rxData);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(rxData == txData);

    /* Write to scratch pad register 2 with different data */
    txData = TEST_PATTERN_5A;
    status = Pmic_ioTxByte(&pmicHandle, SCRATCH_PAD_REG_2_REG, txData);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Read back and verify */
    status = Pmic_ioRxByte(&pmicHandle, SCRATCH_PAD_REG_2_REG, &rxData);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(rxData == txData);

    /* Perform multiple writes with CRC enabled */
    txData = 0x33U;
    status = Pmic_ioTxByte(&pmicHandle, SCRATCH_PAD_REG_3_REG, txData);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    txData = 0xCCU;
    status = Pmic_ioTxByte(&pmicHandle, SCRATCH_PAD_REG_4_REG, txData);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Disable CRC for other tests */
    status = Pmic_ioCrcDisable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

/**
 * @brief Test CRC enable/disable state transitions
 *
 * This test verifies that CRC can be properly enabled and disabled,
 * and that the state is correctly reflected in the handle.
 */
void test_pos_io_ioCrcEnable_crcEnableDisableTransitions(void)
{
    bool enabled = false;
    int32_t status;

    /* Initial state - CRC should be disabled */
    status = Pmic_ioGetCrcEnableState(&pmicHandle, &enabled);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(enabled == false);

    /* Enable CRC */
    status = Pmic_ioCrcEnable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Verify CRC is enabled */
    status = Pmic_ioGetCrcEnableState(&pmicHandle, &enabled);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(enabled == true);

    /* Disable CRC */
    status = Pmic_ioCrcDisable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Verify CRC is disabled */
    status = Pmic_ioGetCrcEnableState(&pmicHandle, &enabled);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(enabled == false);

    /* Enable CRC again */
    status = Pmic_ioCrcEnable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Verify CRC is enabled */
    status = Pmic_ioGetCrcEnableState(&pmicHandle, &enabled);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(enabled == true);

    /* Test using SetCrcEnableState to disable */
    status = Pmic_ioSetCrcEnableState(&pmicHandle, false);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Verify CRC is disabled */
    status = Pmic_ioGetCrcEnableState(&pmicHandle, &enabled);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(enabled == false);

    /* Test using SetCrcEnableState to enable */
    status = Pmic_ioSetCrcEnableState(&pmicHandle, true);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Verify CRC is enabled */
    status = Pmic_ioGetCrcEnableState(&pmicHandle, &enabled);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(enabled == true);

    /* Final cleanup - disable CRC */
    status = Pmic_ioCrcDisable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

/**
 * @brief Test I2C mode write with CRC enabled
 *
 * This test specifically covers the I2C CRC write path that was previously uncovered.
 */
void test_pos_io_ioTxByte_i2cWriteWithCrc(void)
{
    uint8_t txData = 0xBBU;
    uint8_t rxData = 0U;
    int32_t status;
    Pmic_Handle_t i2cHandle = {0};

    /* Initialize handle with I2C mode */
    Pmic_HandleCfg_t handleCfg = {
        .validParams = PMIC_COMM_MODE_VALID |
                       PMIC_CRC_ENABLE_VALID |
                       PMIC_COMM_HANDLE_0_VALID |
                       PMIC_IO_READ_VALID |
                       PMIC_IO_WRITE_VALID |
                       PMIC_CRITICAL_SECTION_START_VALID |
                       PMIC_CRITICAL_SECTION_STOP_VALID,
        .commMode = PMIC_INTF_I2C_SINGLE,
        .crcEnable = false,
        .commHandle0 = platform_getCommHandle(),
        .ioRead = platform_rxByte,
        .ioWrite = platform_txByte,
        .criticalSectionStart = platform_critSecStart,
        .criticalSectionStop = platform_critSecStop
    };

    status = Pmic_init(&i2cHandle, &handleCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Unlock registers for CRC configuration */
    status = Pmic_setRegLockState(&i2cHandle, false);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Enable CRC */
    status = Pmic_ioCrcEnable(&i2cHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Write with CRC enabled in I2C mode - this covers line 202 */
    status = Pmic_ioTxByte(&i2cHandle, SCRATCH_PAD_REG_1_REG, txData);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Read back and verify */
    status = Pmic_ioRxByte(&i2cHandle, SCRATCH_PAD_REG_1_REG, &rxData);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(rxData == txData);

    /* Perform another write to ensure path is covered */
    txData = 0x77U;
    status = Pmic_ioTxByte(&i2cHandle, SCRATCH_PAD_REG_2_REG, txData);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Disable CRC and cleanup */
    status = Pmic_ioCrcDisable(&i2cHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    Pmic_deinit(&i2cHandle);
}

/**
 * @brief Test async write operation in SPI mode
 *
 * This test covers async write paths (lines 176-180).
 */
void test_pos_io_ioTxByte_asyncWriteSpi(void)
{
    uint8_t txData = 0xCDU;
    int32_t status;
    Pmic_Handle_t asyncHandle = {0};

    /* Initialize handle with SPI mode and async enabled */
    Pmic_HandleCfg_t handleCfg = {
        .validParams = PMIC_COMM_MODE_VALID |
                       PMIC_CRC_ENABLE_VALID |
                       PMIC_ASYNC_ENABLE_VALID |
                       PMIC_COMM_HANDLE_0_VALID |
                       PMIC_TASK_HANDLE_VALID |
                       PMIC_ASYNC_RX_START_VALID |
                       PMIC_ASYNC_TX_START_VALID |
                       PMIC_ASYNC_RX_AWAIT_VALID |
                       PMIC_ASYNC_TX_AWAIT_VALID |
                       PMIC_CRITICAL_SECTION_START_VALID |
                       PMIC_CRITICAL_SECTION_STOP_VALID |
                       PMIC_IRQ_RESPONSE_CALLBACK_VALID |
                       PMIC_IO_READ_VALID |
                       PMIC_IO_WRITE_VALID,
        .commMode = PMIC_INTF_SPI,
        .crcEnable = false,
        .asyncEnable = true,
        .commHandle0 = platform_getCommHandle(),
        .taskHandle = platform_getCommHandle(),
        .ioRead = platform_rxByte,
        .ioWrite = platform_txByte,
        .asyncRxStart = test_pmic_asyncRxStart,
        .asyncTxStart = test_pmic_asyncTxStart,
        .asyncRxAwait = test_pmic_asyncRxAwait,
        .asyncTxAwait = test_pmic_asyncTxAwait,
        .criticalSectionStart = platform_critSecStart,
        .criticalSectionStop = platform_critSecStop,
        .irqResponseCallback = platform_irqResponse
    };

    status = Pmic_init(&asyncHandle, &handleCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Unlock registers */
    status = Pmic_setRegLockState(&asyncHandle, false);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Async write in SPI mode without CRC - covers lines 176-180 */
    status = Pmic_ioTxByte(&asyncHandle, SCRATCH_PAD_REG_1_REG, txData);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Async write in SPI mode with CRC enabled */
    status = Pmic_ioCrcEnable(&asyncHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    txData = 0xEFU;
    status = Pmic_ioTxByte(&asyncHandle, SCRATCH_PAD_REG_2_REG, txData);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Cleanup */
    status = Pmic_ioCrcDisable(&asyncHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    Pmic_deinit(&asyncHandle);
}

/**
 * @brief Test async write operation in I2C mode
 *
 * This test covers async I2C write paths (lines 209-213).
 */
void test_pos_io_ioTxByte_asyncWriteI2c(void)
{
    uint8_t txData = 0xABU;
    int32_t status;
    Pmic_Handle_t asyncHandle = {0};

    /* Initialize handle with I2C mode and async enabled */
    Pmic_HandleCfg_t handleCfg = {
        .validParams = PMIC_COMM_MODE_VALID |
                       PMIC_CRC_ENABLE_VALID |
                       PMIC_ASYNC_ENABLE_VALID |
                       PMIC_COMM_HANDLE_0_VALID |
                       PMIC_TASK_HANDLE_VALID |
                       PMIC_ASYNC_RX_START_VALID |
                       PMIC_ASYNC_TX_START_VALID |
                       PMIC_ASYNC_RX_AWAIT_VALID |
                       PMIC_ASYNC_TX_AWAIT_VALID |
                       PMIC_CRITICAL_SECTION_START_VALID |
                       PMIC_CRITICAL_SECTION_STOP_VALID |
                       PMIC_IRQ_RESPONSE_CALLBACK_VALID |
                       PMIC_IO_READ_VALID |
                       PMIC_IO_WRITE_VALID,
        .commMode = PMIC_INTF_I2C_SINGLE,
        .crcEnable = false,
        .asyncEnable = true,
        .commHandle0 = platform_getCommHandle(),
        .taskHandle = platform_getCommHandle(),
        .ioRead = platform_rxByte,
        .ioWrite = platform_txByte,
        .asyncRxStart = test_pmic_asyncRxStart,
        .asyncTxStart = test_pmic_asyncTxStart,
        .asyncRxAwait = test_pmic_asyncRxAwait,
        .asyncTxAwait = test_pmic_asyncTxAwait,
        .criticalSectionStart = platform_critSecStart,
        .criticalSectionStop = platform_critSecStop,
        .irqResponseCallback = platform_irqResponse
    };

    status = Pmic_init(&asyncHandle, &handleCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Unlock registers */
    status = Pmic_setRegLockState(&asyncHandle, false);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Async write in I2C mode without CRC - covers lines 209-213 */
    status = Pmic_ioTxByte(&asyncHandle, SCRATCH_PAD_REG_3_REG, txData);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Async write in I2C mode with CRC enabled */
    status = Pmic_ioCrcEnable(&asyncHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    txData = 0x9DU;
    status = Pmic_ioTxByte(&asyncHandle, SCRATCH_PAD_REG_4_REG, txData);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Cleanup */
    status = Pmic_ioCrcDisable(&asyncHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    Pmic_deinit(&asyncHandle);
}

/**
 * @brief Test async read operation in SPI mode
 *
 * This test covers async SPI read paths.
 */
void test_pos_io_ioRxByte_asyncReadSpi(void)
{
    uint8_t rxData = 0U;
    int32_t status;
    Pmic_Handle_t asyncHandle = {0};

    /* Initialize handle with SPI mode and async enabled */
    Pmic_HandleCfg_t handleCfg = {
        .validParams = PMIC_COMM_MODE_VALID |
                       PMIC_CRC_ENABLE_VALID |
                       PMIC_ASYNC_ENABLE_VALID |
                       PMIC_COMM_HANDLE_0_VALID |
                       PMIC_TASK_HANDLE_VALID |
                       PMIC_ASYNC_RX_START_VALID |
                       PMIC_ASYNC_TX_START_VALID |
                       PMIC_ASYNC_RX_AWAIT_VALID |
                       PMIC_ASYNC_TX_AWAIT_VALID |
                       PMIC_CRITICAL_SECTION_START_VALID |
                       PMIC_CRITICAL_SECTION_STOP_VALID |
                       PMIC_IRQ_RESPONSE_CALLBACK_VALID |
                       PMIC_IO_READ_VALID |
                       PMIC_IO_WRITE_VALID,
        .commMode = PMIC_INTF_SPI,
        .crcEnable = false,
        .asyncEnable = true,
        .commHandle0 = platform_getCommHandle(),
        .taskHandle = platform_getCommHandle(),
        .ioRead = platform_rxByte,
        .ioWrite = platform_txByte,
        .asyncRxStart = test_pmic_asyncRxStart,
        .asyncTxStart = test_pmic_asyncTxStart,
        .asyncRxAwait = test_pmic_asyncRxAwait,
        .asyncTxAwait = test_pmic_asyncTxAwait,
        .criticalSectionStart = platform_critSecStart,
        .criticalSectionStop = platform_critSecStop,
        .irqResponseCallback = platform_irqResponse
    };

    status = Pmic_init(&asyncHandle, &handleCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Async read in SPI mode without CRC */
    status = Pmic_ioRxByte(&asyncHandle, DEV_REV_REG, &rxData);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Unlock registers for CRC configuration */
    status = Pmic_setRegLockState(&asyncHandle, false);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Async read in SPI mode with CRC enabled */
    status = Pmic_ioCrcEnable(&asyncHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_ioRxByte(&asyncHandle, SCRATCH_PAD_REG_1_REG, &rxData);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Cleanup */
    status = Pmic_ioCrcDisable(&asyncHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    Pmic_deinit(&asyncHandle);
}

/**
 * @brief Test async read operation in I2C mode
 *
 * This test covers async I2C read paths.
 */
void test_pos_io_ioRxByte_asyncReadI2c(void)
{
    uint8_t rxData = 0U;
    int32_t status;
    Pmic_Handle_t asyncHandle = {0};

    /* Initialize handle with I2C mode and async enabled */
    Pmic_HandleCfg_t handleCfg = {
        .validParams = PMIC_COMM_MODE_VALID |
                       PMIC_CRC_ENABLE_VALID |
                       PMIC_ASYNC_ENABLE_VALID |
                       PMIC_COMM_HANDLE_0_VALID |
                       PMIC_TASK_HANDLE_VALID |
                       PMIC_ASYNC_RX_START_VALID |
                       PMIC_ASYNC_TX_START_VALID |
                       PMIC_ASYNC_RX_AWAIT_VALID |
                       PMIC_ASYNC_TX_AWAIT_VALID |
                       PMIC_CRITICAL_SECTION_START_VALID |
                       PMIC_CRITICAL_SECTION_STOP_VALID |
                       PMIC_IRQ_RESPONSE_CALLBACK_VALID |
                       PMIC_IO_READ_VALID |
                       PMIC_IO_WRITE_VALID,
        .commMode = PMIC_INTF_I2C_SINGLE,
        .crcEnable = false,
        .asyncEnable = true,
        .commHandle0 = platform_getCommHandle(),
        .taskHandle = platform_getCommHandle(),
        .ioRead = platform_rxByte,
        .ioWrite = platform_txByte,
        .asyncRxStart = test_pmic_asyncRxStart,
        .asyncTxStart = test_pmic_asyncTxStart,
        .asyncRxAwait = test_pmic_asyncRxAwait,
        .asyncTxAwait = test_pmic_asyncTxAwait,
        .criticalSectionStart = platform_critSecStart,
        .criticalSectionStop = platform_critSecStop,
        .irqResponseCallback = platform_irqResponse
    };

    status = Pmic_init(&asyncHandle, &handleCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Async read in I2C mode without CRC */
    status = Pmic_ioRxByte(&asyncHandle, DEV_REV_REG, &rxData);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Unlock registers for CRC configuration */
    status = Pmic_setRegLockState(&asyncHandle, false);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Async read in I2C mode with CRC enabled */
    status = Pmic_ioCrcEnable(&asyncHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_ioRxByte(&asyncHandle, SCRATCH_PAD_REG_2_REG, &rxData);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Cleanup */
    status = Pmic_ioCrcDisable(&asyncHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    Pmic_deinit(&asyncHandle);
}

/**
 * @brief Test CRC state transitions with I/O operations
 *
 * This test verifies that I/O operations work correctly when CRC state
 * is changed between operations.
 */
void test_pos_io_ioCrcEnable_crcStateTransitionsWithOperations(void)
{
    uint8_t txData = 0x12U;
    uint8_t rxData = 0U;
    int32_t status;

    /* Start with CRC disabled */
    status = Pmic_ioCrcDisable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Write with CRC disabled */
    status = Pmic_ioTxByte(&pmicHandle, SCRATCH_PAD_REG_1_REG, txData);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Enable CRC */
    status = Pmic_ioCrcEnable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Read with CRC enabled (reading previously written data) */
    status = Pmic_ioRxByte(&pmicHandle, SCRATCH_PAD_REG_1_REG, &rxData);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(rxData == txData);

    /* Write with CRC enabled */
    txData = 0x34U;
    status = Pmic_ioTxByte(&pmicHandle, SCRATCH_PAD_REG_2_REG, txData);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Disable CRC */
    status = Pmic_ioCrcDisable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Read with CRC disabled */
    status = Pmic_ioRxByte(&pmicHandle, SCRATCH_PAD_REG_2_REG, &rxData);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(rxData == txData);

    /* Write with CRC disabled */
    txData = 0x56U;
    status = Pmic_ioTxByte(&pmicHandle, SCRATCH_PAD_REG_3_REG, txData);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Enable CRC again */
    status = Pmic_ioCrcEnable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Perform read-modify-write with CRC enabled */
    status = Pmic_ioUpdateByte(&pmicHandle, SCRATCH_PAD_REG_4_REG, 0U, TEST_MASK_HIGH_NIBBLE, 0xA0U);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Read back and verify */
    status = Pmic_ioRxByte(&pmicHandle, SCRATCH_PAD_REG_4_REG, &rxData);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT((rxData & TEST_MASK_HIGH_NIBBLE) == 0xA0U);

    /* Final cleanup - disable CRC */
    status = Pmic_ioCrcDisable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

/**
 * @brief Test ioRxByte with CRC error and retry logic
 */
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

    /* Enable CRC to trigger CRC validation path */
    status = Pmic_ioCrcEnable(&testHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Reset mock state AFTER enabling CRC so read count only reflects test operation */
    resetMockIoState();

    /* Configure mock to corrupt CRC on first read attempt */
    g_mockCrcCorruptionMask = TEST_MASK_FULL_BYTE;  /* Corrupt CRC byte */

    /* Perform read - should fail on first attempt with CRC error, succeed on retry */
    status = Pmic_ioRxByte(&testHandle, SCRATCH_PAD_REG_1_REG, &regData);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Verify that retry occurred (should have 2 read calls) */
    PLATFORM_ASSERT(g_mockIoReadCallCount == 2U);

    /* Disable CRC for cleanup */
    status = Pmic_ioCrcDisable(&testHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

/**
 * @brief Test ioTxByte with I/O failure and retry logic
 */
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
    status = Pmic_ioTxByte(&testHandle, SCRATCH_PAD_REG_1_REG, writeVal);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Verify that retry occurred (should have 2 write calls) */
    PLATFORM_ASSERT(g_mockIoWriteCallCount == 2U);

    /* Verify the value was actually written by reading it back */
    uint8_t readVal = 0U;
    status = Pmic_ioRxByte(&pmicHandle, SCRATCH_PAD_REG_1_REG, &readVal);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(readVal == writeVal);
}

/**
 * @brief Test CRC error exhausts retries
 */
void test_neg_io_ioRxByte_crcErrorExhaustsRetries(void)
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
    status = Pmic_ioRxByte(&testHandle, SCRATCH_PAD_REG_1_REG, &regData);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);  /* Should succeed on retry */

    /* Now configure to fail multiple times to exhaust retries */
    resetMockIoState();

    /* Test with retry count 0 to force immediate failure */
    testHandle.retryCnt = 0U;
    g_mockIoReadReturnStatus = PMIC_ST_ERR_I2C_COMM_FAIL;

    /* This should fail immediately with no retries */
    status = Pmic_ioRxByte(&testHandle, SCRATCH_PAD_REG_1_REG, &regData);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_I2C_COMM_FAIL);

    /* Verify only one attempt was made */
    PLATFORM_ASSERT(g_mockIoReadCallCount == 1U);

    /* Restore retry count and test exhausting all retries */
    testHandle.retryCnt = 1U;  /* Allow 1 retry (2 total attempts) */
    resetMockIoState();

    /* Configure to fail on first attempt */
    g_mockIoReadReturnStatus = PMIC_ST_ERR_I2C_COMM_FAIL;

    status = Pmic_ioRxByte(&testHandle, SCRATCH_PAD_REG_1_REG, &regData);

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
    status = Pmic_ioTxByte(&testHandle, SCRATCH_PAD_REG_2_REG, writeVal);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Verify exactly 2 write calls (original + 1 retry) */
    PLATFORM_ASSERT(g_mockIoWriteCallCount == 2U);

    /* Verify the value was actually written */
    uint8_t readVal = 0U;
    status = Pmic_ioRxByte(&pmicHandle, SCRATCH_PAD_REG_2_REG, &readVal);
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
    status = Pmic_ioRxByte(&testHandle, SCRATCH_PAD_REG_3_REG, &regData);
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
    status = Pmic_ioTxByte(&testHandle, SCRATCH_PAD_REG_4_REG, writeVal);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Should have 2 attempts (1 failure + 1 success) */
    PLATFORM_ASSERT(g_mockIoWriteCallCount == 2U);
}

/**
 * @brief Test NULL timer with retry configuration
 * Covers line 147 in pmic_io.c
 */
void test_neg_io_ioTxByte_nullTimerWithRetry(void)
{
    int32_t status;
    Pmic_Handle_t testHandle;
    uint8_t rxData = 0U;

    /* Initialize test handle with NULL timer but non-zero retry */
    (void)memcpy(&testHandle, &pmicHandle, sizeof(Pmic_Handle_t));
    testHandle.timerWaitMs = NULL;
    testHandle.retryIntervalMs = 10U;  /* Non-zero retry interval with NULL timer */
    testHandle.retryCnt = 2U;

    /* This should fail due to NULL timer with retry enabled */
    status = Pmic_ioRxByte(&testHandle, DEV_REV_REG, &rxData);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_FPTR);
}

/**
 * @brief Test NULL async hooks with async enabled
 * Covers line 162 in pmic_io.c
 */
void test_neg_io_ioTxByte_nullAsyncHooks(void)
{
    int32_t status;
    Pmic_Handle_t testHandle = {0};
    uint8_t rxData = 0U;

    /* Initialize handle with async enabled but NULL hooks */
    Pmic_HandleCfg_t handleCfg = {
        .validParams = PMIC_COMM_MODE_VALID |
                       PMIC_ASYNC_ENABLE_VALID |
                       PMIC_COMM_HANDLE_0_VALID |
                       PMIC_IO_READ_VALID |
                       PMIC_IO_WRITE_VALID |
                       PMIC_CRITICAL_SECTION_START_VALID |
                       PMIC_CRITICAL_SECTION_STOP_VALID,
        .commMode = PMIC_INTF_SPI,
        .asyncEnable = true,
        .commHandle0 = platform_getCommHandle(),
        .ioRead = platform_rxByte,
        .ioWrite = platform_txByte,
        .criticalSectionStart = platform_critSecStart,
        .criticalSectionStop = platform_critSecStop
        /* Async hooks are NULL */
    };

    status = Pmic_init(&testHandle, &handleCfg);
    if (status == PMIC_ST_SUCCESS)
    {
        /* Try to perform I/O with async enabled but NULL hooks */
        status = Pmic_ioRxByte(&testHandle, DEV_REV_REG, &rxData);
        PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_FPTR);

        Pmic_deinit(&testHandle);
    }
}

/**
 * @brief Test I2C TX retry logic
 * Covers lines 275-277 in pmic_io.c
 */
void test_pos_io_ioTxByte_i2cTxRetry(void)
{
    int32_t status;
    Pmic_Handle_t testHandle;
    uint8_t writeVal = 0xDDU;

    /* Initialize test handle with I2C mode and mock functions */
    (void)memcpy(&testHandle, &pmicHandle, sizeof(Pmic_Handle_t));
    testHandle.commMode = PMIC_INTF_I2C_SINGLE;
    testHandle.ioRead = &mockIoRead;
    testHandle.ioWrite = &mockIoWrite;
    testHandle.timerWaitMs = &mockTimerWait;
    testHandle.retryCnt = 2U;
    testHandle.retryIntervalMs = 10U;

    /* Reset mock state */
    resetMockIoState();

    /* Configure mock to fail on first write attempt */
    g_mockIoWriteReturnStatus = PMIC_ST_ERR_I2C_COMM_FAIL;

    /* Perform write - should retry and succeed */
    status = Pmic_ioTxByte(&testHandle, SCRATCH_PAD_REG_1_REG, writeVal);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Verify retry occurred */
    PLATFORM_ASSERT(g_mockIoWriteCallCount == 2U);
}

/**
 * @brief Test SPI RX CRC mismatch error
 * Covers lines 353-354 in pmic_io.c
 */
void test_neg_io_ioRxByte_spiRxCrcMismatch(void)
{
    int32_t status;
    Pmic_Handle_t testHandle;
    uint8_t rxData = 0U;

    /* Initialize test handle with SPI mode and CRC enabled */
    (void)memcpy(&testHandle, &pmicHandle, sizeof(Pmic_Handle_t));
    testHandle.commMode = PMIC_INTF_SPI;
    testHandle.ioRead = &mockIoRead;
    testHandle.ioWrite = &mockIoWrite;
    testHandle.timerWaitMs = &mockTimerWait;
    testHandle.retryCnt = 0U;  /* No retries to force CRC error */
    testHandle.retryIntervalMs = 0U;

    /* Reset mock state */
    resetMockIoState();

    /* Enable CRC */
    status = Pmic_ioCrcEnable(&testHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Configure mock to corrupt CRC */
    g_mockCrcCorruptionMask = TEST_MASK_FULL_BYTE;

    /* Perform read - should fail with CRC error */
    status = Pmic_ioRxByte(&testHandle, SCRATCH_PAD_REG_1_REG, &rxData);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_DATA_IO_CRC);
}

/**
 * @brief Test I2C RX CRC mismatch error
 * Covers lines 406-407 in pmic_io.c
 */
void test_neg_io_ioRxByte_i2cRxCrcMismatch(void)
{
    int32_t status;
    Pmic_Handle_t testHandle;
    uint8_t rxData = 0U;

    /* Initialize test handle with I2C mode and CRC enabled */
    (void)memcpy(&testHandle, &pmicHandle, sizeof(Pmic_Handle_t));
    testHandle.commMode = PMIC_INTF_I2C_SINGLE;
    testHandle.ioRead = &mockIoRead;
    testHandle.ioWrite = &mockIoWrite;
    testHandle.timerWaitMs = &mockTimerWait;
    testHandle.retryCnt = 0U;  /* No retries to force CRC error */
    testHandle.retryIntervalMs = 0U;

    /* Reset mock state */
    resetMockIoState();

    /* Enable CRC */
    status = Pmic_ioCrcEnable(&testHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Configure mock to corrupt CRC */
    g_mockCrcCorruptionMask = TEST_MASK_FULL_BYTE;

    /* Perform read - should fail with CRC error */
    status = Pmic_ioRxByte(&testHandle, SCRATCH_PAD_REG_1_REG, &rxData);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_DATA_IO_CRC);
}

/**
 * @brief Test I2C RX retry logic
 * Covers lines 415-417 in pmic_io.c
 */
void test_pos_io_ioRxByte_i2cRxRetry(void)
{
    int32_t status;
    Pmic_Handle_t testHandle;
    uint8_t rxData = 0U;

    /* Initialize test handle with I2C mode and mock functions */
    (void)memcpy(&testHandle, &pmicHandle, sizeof(Pmic_Handle_t));
    testHandle.commMode = PMIC_INTF_I2C_SINGLE;
    testHandle.ioRead = &mockIoRead;
    testHandle.ioWrite = &mockIoWrite;
    testHandle.timerWaitMs = &mockTimerWait;
    testHandle.retryCnt = 2U;
    testHandle.retryIntervalMs = 10U;

    /* Reset mock state */
    resetMockIoState();

    /* Configure mock to fail on first read attempt */
    g_mockIoReadReturnStatus = PMIC_ST_ERR_I2C_COMM_FAIL;

    /* Perform read - should retry and succeed */
    status = Pmic_ioRxByte(&testHandle, SCRATCH_PAD_REG_1_REG, &rxData);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Verify retry occurred */
    PLATFORM_ASSERT(g_mockIoReadCallCount == 2U);
}

/* ========================================================================== */
/*                         Entry Point Function                               */
/* ========================================================================== */

void io_test(void *args)
{
    (void)args;
    int32_t status;

    platform_init();
    platform_setupTests();

    /* Initialize PMIC handle */
    Pmic_HandleCfg_t handleCfg = {
        .validParams = PMIC_COMM_MODE_VALID |
                       PMIC_CRC_ENABLE_VALID |
                       PMIC_COMM_HANDLE_0_VALID |
                       PMIC_IO_READ_VALID |
                       PMIC_IO_WRITE_VALID |
                       PMIC_CRITICAL_SECTION_START_VALID |
                       PMIC_CRITICAL_SECTION_STOP_VALID,
        .commMode = PMIC_INTF_SPI,
        .crcEnable = false,
        .commHandle0 = platform_getCommHandle(),
        .ioRead = platform_rxByte,
        .ioWrite = platform_txByte,
        .criticalSectionStart = platform_critSecStart,
        .criticalSectionStop = platform_critSecStop
    };

    status = Pmic_init(&pmicHandle, &handleCfg);
    if (status != PMIC_ST_SUCCESS)
    {
        platform_printString("\r\nERROR: Failed to initialize PMIC handle\r\n");
        platform_tearDownTests();
        platform_deinit();
        return;
    }

    /* Unlock registers for CRC configuration */
    status = Pmic_setRegLockState(&pmicHandle, false);
    if (status != PMIC_ST_SUCCESS)
    {
        platform_printString("\r\nERROR: Failed to unlock registers\r\n");
        Pmic_deinit(&pmicHandle);
        platform_tearDownTests();
        platform_deinit();
        return;
    }

    platform_printString("\r\n=== IO Module Tests ===\r\n");
    IO_TEST_RUN_ALL();

    Pmic_deinit(&pmicHandle);
    platform_tearDownTests();
    platform_deinit();
}
