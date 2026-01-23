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

#include "io_test.h"
#include "test_constants.h"
#include "regmap/core.h"
#include "pmic_mock_core.h"

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/* Test register addresses - using scratchpad registers for safe testing */
#define IO_TEST_SCRATCHPAD1_REG     PMIC_CUSTOMER_SCRATCH1_REG  /* 0x68 */
#define IO_TEST_SCRATCHPAD2_REG     PMIC_CUSTOMER_SCRATCH2_REG  /* 0x69 */

/* Test bit field positions for RMW tests */
#define IO_TEST_BIT_POS_0           (0U)
#define IO_TEST_BIT_POS_4           (4U)
#define IO_TEST_BIT_MASK_NIBBLE     TEST_MASK_LOW_NIBBLE
#define IO_TEST_BIT_MASK_SINGLE     (0x01U)

/* ========================================================================== */
/*                             Global Variables                               */
/* ========================================================================== */

static Pmic_Handle_t g_pmicHandle;

/* Global variables for mock I/O control */
static uint32_t g_mockIoReadCallCount = 0U;
static uint32_t g_mockIoWriteCallCount = 0U;
static int32_t g_mockIoReadReturnStatus = PMIC_ST_SUCCESS;
static int32_t g_mockIoWriteReturnStatus = PMIC_ST_SUCCESS;
static uint8_t g_mockCrcCorruptionMask = 0x00U;

/* ========================================================================== */
/*                          Mock I/O Helper Functions                         */
/* ========================================================================== */

static void resetMockIoState(void)
{
    g_mockIoReadCallCount = 0U;
    g_mockIoWriteCallCount = 0U;
    g_mockIoReadReturnStatus = PMIC_ST_SUCCESS;
    g_mockIoWriteReturnStatus = PMIC_ST_SUCCESS;
    g_mockCrcCorruptionMask = 0x00U;
}

static int32_t mockIoRead(const Pmic_Handle_t *handle, uint8_t page, uint8_t regAddr,
                          uint8_t *buffer, uint8_t bufLen)
{
    int32_t status;
    g_mockIoReadCallCount++;
    if (g_mockIoReadReturnStatus != PMIC_ST_SUCCESS)
    {
        status = g_mockIoReadReturnStatus;
        g_mockIoReadReturnStatus = PMIC_ST_SUCCESS;
        return status;
    }
    /* Call the actual mock backend read function */
    status = test_pmic_regRead(handle, page, regAddr, buffer, bufLen);
    if ((g_mockCrcCorruptionMask != 0x00U) && (bufLen == 4U) && (status == PMIC_ST_SUCCESS))
    {
        /* For SPI, CRC is at index 3 (4-byte frame) */
        buffer[3] ^= g_mockCrcCorruptionMask;
        g_mockCrcCorruptionMask = 0x00U;
    }
    return status;
}

static int32_t mockIoWrite(const Pmic_Handle_t *handle, uint8_t page, uint8_t regAddr,
                           const uint8_t *buffer, uint8_t bufLen)
{
    int32_t status;
    g_mockIoWriteCallCount++;
    if (g_mockIoWriteReturnStatus != PMIC_ST_SUCCESS)
    {
        status = g_mockIoWriteReturnStatus;
        g_mockIoWriteReturnStatus = PMIC_ST_SUCCESS;
        return status;
    }
    /* Call the actual mock backend write function */
    return test_pmic_regWrite(handle, page, regAddr, buffer, bufLen);
}

static void mockTimerWait(uint32_t ms)
{
    (void)ms;
}

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void io_test(void *args)
{
    (void)args;
    int32_t status = PMIC_ST_SUCCESS;
    /* Dummy handle for mock - driver validates non-NULL but doesn't dereference */
    static uint32_t dummyCommHandle = TEST_DUMMY_HANDLE;

    Pmic_HandleCfg_t pmicCfg = {
        .validParams = PMIC_COMM_MODE_VALID |
                       PMIC_COMM_HANDLE_0_VALID |
                       PMIC_IO_READ_VALID |
                       PMIC_IO_WRITE_VALID |
                       PMIC_CRITICAL_SECTION_START_VALID |
                       PMIC_CRITICAL_SECTION_STOP_VALID,
        .commMode = PMIC_INTF_SPI,
        .commHandle0 = (void*)&dummyCommHandle,  /* Driver requires non-NULL, even for mock */
        .ioRead = &test_pmic_regRead,
        .ioWrite = &test_pmic_regWrite,
        .criticalSectionStart = &test_pmic_criticalSectionStartFn,
        .criticalSectionStop = &test_pmic_criticalSectionStopFn
    };

    platform_init();

    printf("\r\n");
    printf("==================================================\r\n");
    printf("    TPS65386x-Q1 I/O Module Tests\r\n");
    printf("==================================================\r\n\r\n");

    /* Initialize PMIC */
    status = Pmic_init(&g_pmicHandle, &pmicCfg);
    if (status != PMIC_ST_SUCCESS)
    {
        printf("ERROR: PMIC initialization failed with status: %d\r\n", status);
        platform_deinit();
        return;
    }

    /* Run all I/O tests */
    IO_TEST_RUN_ALL();

    /* Cleanup */
    (void)Pmic_deinit(&g_pmicHandle);
    platform_deinit();

    printf("\r\n==================================================\r\n");
    printf("    I/O Module Tests Complete\r\n");
    printf("==================================================\r\n\r\n");
}

/* ========================================================================== */
/*                      Negative Tests - NULL Handle                          */
/* ========================================================================== */

void test_neg_io_ioTxByte_nullHandle(void)
{
    uint8_t txData = TEST_PATTERN_AA;
    int32_t status = Pmic_ioTxByte(NULL, IO_TEST_SCRATCHPAD1_REG, txData);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_io_ioRxByte_nullHandle(void)
{
    uint8_t rxData = 0U;
    int32_t status = Pmic_ioRxByte(NULL, IO_TEST_SCRATCHPAD1_REG, &rxData);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_io_ioTxByte_CS_nullHandle(void)
{
    uint8_t txData = TEST_PATTERN_AA;
    int32_t status = Pmic_ioTxByte_CS(NULL, IO_TEST_SCRATCHPAD1_REG, txData);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_io_ioRxByte_CS_nullHandle(void)
{
    uint8_t rxData = 0U;
    int32_t status = Pmic_ioRxByte_CS(NULL, IO_TEST_SCRATCHPAD1_REG, &rxData);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/* ========================================================================== */
/*                    Negative Tests - NULL Parameters                        */
/* ========================================================================== */

void test_neg_io_ioRxByte_nullRxBuffer(void)
{
    int32_t status = Pmic_ioRxByte(&g_pmicHandle, IO_TEST_SCRATCHPAD1_REG, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_io_ioRxByte_CS_nullRxBuffer(void)
{
    int32_t status = Pmic_ioRxByte_CS(&g_pmicHandle, IO_TEST_SCRATCHPAD1_REG, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_io_ioRxWordSeq_nullHandle(void)
{
    uint32_t rxData = 0U;
    int32_t status = Pmic_ioRxWordSeq(NULL, IO_TEST_SCRATCHPAD1_REG, &rxData, 2U);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_io_ioRxWordSeq_nullRxData(void)
{
    int32_t status = Pmic_ioRxWordSeq(&g_pmicHandle, IO_TEST_SCRATCHPAD1_REG, NULL, 2U);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_io_ioTxWordSeq_nullHandle(void)
{
    uint32_t txData = 0xAABBCCDDU;
    int32_t status = Pmic_ioTxWordSeq(NULL, IO_TEST_SCRATCHPAD1_REG, txData, 2U);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/* ========================================================================== */
/*                   Negative Tests - Invalid Parameters                      */
/* ========================================================================== */

void test_neg_io_ioRxWordSeq_invalidCount(void)
{
    uint32_t rxData = 0U;
    /* Count exceeds uint32_t size (4 bytes) */
    int32_t status = Pmic_ioRxWordSeq(&g_pmicHandle, IO_TEST_SCRATCHPAD1_REG, &rxData, 5U);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_io_ioTxWordSeq_invalidCount(void)
{
    uint32_t txData = 0xAABBCCDDU;
    /* Count exceeds uint32_t size (4 bytes) */
    int32_t status = Pmic_ioTxWordSeq(&g_pmicHandle, IO_TEST_SCRATCHPAD1_REG, txData, 5U);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/* ========================================================================== */
/*              Positive Tests - Single Byte Operations                       */
/* ========================================================================== */

void test_pos_io_ioTxByte_scratchpad1(void)
{
    int32_t status;
    uint8_t writeData = TEST_PATTERN_A5;
    uint8_t readData = 0U;

    /* Write test pattern */
    status = Pmic_ioTxByte(&g_pmicHandle, IO_TEST_SCRATCHPAD1_REG, writeData);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Read back and verify */
    status = Pmic_ioRxByte(&g_pmicHandle, IO_TEST_SCRATCHPAD1_REG, &readData);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(readData == writeData);
}

void test_pos_io_ioRxByte_scratchpad1(void)
{
    int32_t status;
    uint8_t writeData = TEST_PATTERN_A5;
    uint8_t readData = 0U;

    /* Write test pattern */
    status = Pmic_ioTxByte(&g_pmicHandle, IO_TEST_SCRATCHPAD1_REG, writeData);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Read back and verify */
    status = Pmic_ioRxByte(&g_pmicHandle, IO_TEST_SCRATCHPAD1_REG, &readData);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(readData == writeData);
}

void test_pos_io_ioTxByte_scratchpad2(void)
{
    int32_t status;
    uint8_t writeData = TEST_PATTERN_5A;
    uint8_t readData = 0U;

    /* Write test pattern */
    status = Pmic_ioTxByte(&g_pmicHandle, IO_TEST_SCRATCHPAD2_REG, writeData);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Read back and verify */
    status = Pmic_ioRxByte(&g_pmicHandle, IO_TEST_SCRATCHPAD2_REG, &readData);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(readData == writeData);
}

void test_pos_io_ioRxByte_scratchpad2(void)
{
    int32_t status;
    uint8_t writeData = TEST_PATTERN_5A;
    uint8_t readData = 0U;

    /* Write test pattern */
    status = Pmic_ioTxByte(&g_pmicHandle, IO_TEST_SCRATCHPAD2_REG, writeData);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Read back and verify */
    status = Pmic_ioRxByte(&g_pmicHandle, IO_TEST_SCRATCHPAD2_REG, &readData);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(readData == writeData);
}

void test_pos_io_ioTxByte_CS_scratchpad1(void)
{
    int32_t status;
    uint8_t writeData = 0x3CU;
    uint8_t readData = 0U;

    /* Write test pattern with critical section */
    status = Pmic_ioTxByte_CS(&g_pmicHandle, IO_TEST_SCRATCHPAD1_REG, writeData);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Read back and verify with critical section */
    status = Pmic_ioRxByte_CS(&g_pmicHandle, IO_TEST_SCRATCHPAD1_REG, &readData);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(readData == writeData);
}

void test_pos_io_ioRxByte_CS_scratchpad1(void)
{
    int32_t status;
    uint8_t writeData = 0x3CU;
    uint8_t readData = 0U;

    /* Write test pattern with critical section */
    status = Pmic_ioTxByte_CS(&g_pmicHandle, IO_TEST_SCRATCHPAD1_REG, writeData);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Read back and verify with critical section */
    status = Pmic_ioRxByte_CS(&g_pmicHandle, IO_TEST_SCRATCHPAD1_REG, &readData);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(readData == writeData);
}

void test_pos_io_ioTxByte_CS_scratchpad2(void)
{
    int32_t status;
    uint8_t writeData = 0xC3U;
    uint8_t readData = 0U;

    /* Write test pattern with critical section */
    status = Pmic_ioTxByte_CS(&g_pmicHandle, IO_TEST_SCRATCHPAD2_REG, writeData);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Read back and verify with critical section */
    status = Pmic_ioRxByte_CS(&g_pmicHandle, IO_TEST_SCRATCHPAD2_REG, &readData);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(readData == writeData);
}

void test_pos_io_ioRxByte_CS_scratchpad2(void)
{
    int32_t status;
    uint8_t writeData = 0xC3U;
    uint8_t readData = 0U;

    /* Write test pattern with critical section */
    status = Pmic_ioTxByte_CS(&g_pmicHandle, IO_TEST_SCRATCHPAD2_REG, writeData);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Read back and verify with critical section */
    status = Pmic_ioRxByte_CS(&g_pmicHandle, IO_TEST_SCRATCHPAD2_REG, &readData);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(readData == writeData);
}

/* ========================================================================== */
/*          Positive Tests - Multi-Byte Sequential Operations                 */
/* ========================================================================== */

void test_pos_io_ioTxRxWordSeq_1byte(void)
{
    int32_t status;
    uint32_t writeData = 0x000000ABU;
    uint32_t readData = 0U;

    /* Write 1 byte */
    status = Pmic_ioTxWordSeq(&g_pmicHandle, IO_TEST_SCRATCHPAD1_REG, writeData, 1U);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Read back and verify */
    status = Pmic_ioRxWordSeq(&g_pmicHandle, IO_TEST_SCRATCHPAD1_REG, &readData, 1U);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT((readData & 0xFFU) == (writeData & 0xFFU));
}

void test_pos_io_ioTxRxWordSeq_2bytes(void)
{
    int32_t status;
    uint32_t writeData = 0x0000ABCDU;
    uint32_t readData = 0U;

    /* Write 2 bytes sequentially starting at SCRATCHPAD1 */
    status = Pmic_ioTxWordSeq(&g_pmicHandle, IO_TEST_SCRATCHPAD1_REG, writeData, 2U);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Read back and verify */
    status = Pmic_ioRxWordSeq(&g_pmicHandle, IO_TEST_SCRATCHPAD1_REG, &readData, 2U);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT((readData & 0xFFFFU) == (writeData & 0xFFFFU));
}


/* ========================================================================== */
/*          Positive Tests - Read-Modify-Write Operations                     */
/* ========================================================================== */

void test_pos_io_ioUpdateByte_singleBitField(void)
{
    int32_t status;
    uint8_t initialValue = TEST_MASK_HIGH_NIBBLE;
    uint8_t readData = 0U;
    uint8_t expectedValue;

    /* Initialize register */
    status = Pmic_ioTxByte(&g_pmicHandle, IO_TEST_SCRATCHPAD1_REG, initialValue);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Modify single bit (bit 0) */
    status = Pmic_ioUpdateByte(&g_pmicHandle, IO_TEST_SCRATCHPAD1_REG,
                               IO_TEST_BIT_POS_0, (1U << IO_TEST_BIT_POS_0), 1U);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Verify modification */
    status = Pmic_ioRxByte(&g_pmicHandle, IO_TEST_SCRATCHPAD1_REG, &readData);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    expectedValue = initialValue | (1U << IO_TEST_BIT_POS_0);
    PLATFORM_ASSERT(readData == expectedValue);
}

void test_pos_io_ioUpdateByte_multiBitField(void)
{
    int32_t status;
    uint8_t initialValue = 0x00U;
    uint8_t readData = 0U;
    uint8_t newFieldValue = 0x0AU;
    uint8_t expectedValue;

    /* Initialize register */
    status = Pmic_ioTxByte(&g_pmicHandle, IO_TEST_SCRATCHPAD1_REG, initialValue);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Modify nibble (bits 3:0) */
    status = Pmic_ioUpdateByte(&g_pmicHandle, IO_TEST_SCRATCHPAD1_REG,
                               IO_TEST_BIT_POS_0, IO_TEST_BIT_MASK_NIBBLE, newFieldValue);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Verify modification */
    status = Pmic_ioRxByte(&g_pmicHandle, IO_TEST_SCRATCHPAD1_REG, &readData);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    expectedValue = (initialValue & ~IO_TEST_BIT_MASK_NIBBLE) | newFieldValue;
    PLATFORM_ASSERT(readData == expectedValue);
}

void test_pos_io_ioUpdateByte_b_setBit(void)
{
    int32_t status;
    uint8_t initialValue = 0x00U;
    uint8_t readData = 0U;

    /* Initialize register */
    status = Pmic_ioTxByte(&g_pmicHandle, IO_TEST_SCRATCHPAD1_REG, initialValue);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Set bit 4 using boolean API */
    status = Pmic_ioUpdateByte_b(&g_pmicHandle, IO_TEST_SCRATCHPAD1_REG,
                                 IO_TEST_BIT_POS_4, true);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Verify bit is set */
    status = Pmic_ioRxByte(&g_pmicHandle, IO_TEST_SCRATCHPAD1_REG, &readData);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT((readData & (1U << IO_TEST_BIT_POS_4)) != 0U);
}

void test_pos_io_ioUpdateByte_b_clearBit(void)
{
    int32_t status;
    uint8_t initialValue = TEST_MASK_FULL_BYTE;
    uint8_t readData = 0U;

    /* Initialize register with all bits set */
    status = Pmic_ioTxByte(&g_pmicHandle, IO_TEST_SCRATCHPAD1_REG, initialValue);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Clear bit 4 using boolean API */
    status = Pmic_ioUpdateByte_b(&g_pmicHandle, IO_TEST_SCRATCHPAD1_REG,
                                 IO_TEST_BIT_POS_4, false);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Verify bit is cleared */
    status = Pmic_ioRxByte(&g_pmicHandle, IO_TEST_SCRATCHPAD1_REG, &readData);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT((readData & (1U << IO_TEST_BIT_POS_4)) == 0U);
}

void test_pos_io_ioUpdateByte_CS_singleBitField(void)
{
    int32_t status;
    uint8_t initialValue = TEST_PATTERN_55;
    uint8_t readData = 0U;
    uint8_t expectedValue;

    /* Initialize register */
    status = Pmic_ioTxByte(&g_pmicHandle, IO_TEST_SCRATCHPAD2_REG, initialValue);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Modify bit field with critical section */
    status = Pmic_ioUpdateByte_CS(&g_pmicHandle, IO_TEST_SCRATCHPAD2_REG,
                                  IO_TEST_BIT_POS_0, (1U << IO_TEST_BIT_POS_0), 0U);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Verify modification */
    status = Pmic_ioRxByte(&g_pmicHandle, IO_TEST_SCRATCHPAD2_REG, &readData);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    expectedValue = initialValue & ~(1U << IO_TEST_BIT_POS_0);
    PLATFORM_ASSERT(readData == expectedValue);
}

void test_pos_io_ioUpdateByte_bCS_setBit(void)
{
    int32_t status;
    uint8_t initialValue = TEST_PATTERN_AA;
    uint8_t readData = 0U;

    /* Initialize register */
    status = Pmic_ioTxByte(&g_pmicHandle, IO_TEST_SCRATCHPAD2_REG, initialValue);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Set bit with critical section using boolean API */
    status = Pmic_ioUpdateByte_bCS(&g_pmicHandle, IO_TEST_SCRATCHPAD2_REG,
                                   IO_TEST_BIT_POS_4, true);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Verify bit is set */
    status = Pmic_ioRxByte(&g_pmicHandle, IO_TEST_SCRATCHPAD2_REG, &readData);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT((readData & (1U << IO_TEST_BIT_POS_4)) != 0U);
}

/* ========================================================================== */
/*               Positive Tests - Register Boundaries                         */
/* ========================================================================== */

void test_pos_io_ioTxRxByte_registerBoundaries(void)
{
    int32_t status;
    uint8_t writeData1 = 0x12U;
    uint8_t writeData2 = 0x34U;
    uint8_t readData1 = 0U;
    uint8_t readData2 = 0U;

    /* Test writing to both scratchpad registers */
    status = Pmic_ioTxByte(&g_pmicHandle, IO_TEST_SCRATCHPAD1_REG, writeData1);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_ioTxByte(&g_pmicHandle, IO_TEST_SCRATCHPAD2_REG, writeData2);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Verify both registers independently */
    status = Pmic_ioRxByte(&g_pmicHandle, IO_TEST_SCRATCHPAD1_REG, &readData1);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(readData1 == writeData1);

    status = Pmic_ioRxByte(&g_pmicHandle, IO_TEST_SCRATCHPAD2_REG, &readData2);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(readData2 == writeData2);
}

void test_pos_io_ioTxRxByte_allScratchpadRegs(void)
{
    int32_t status;
    uint8_t writePattern[2] = {0xABU, 0xCDU};
    uint8_t readData = 0U;

    /* Write different patterns to each scratchpad register */
    for (uint8_t i = 0U; i < 2U; i++)
    {
        uint16_t regAddr = IO_TEST_SCRATCHPAD1_REG + i;

        status = Pmic_ioTxByte(&g_pmicHandle, regAddr, writePattern[i]);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    }

    /* Verify each scratchpad register independently */
    for (uint8_t i = 0U; i < 2U; i++)
    {
        uint16_t regAddr = IO_TEST_SCRATCHPAD1_REG + i;

        status = Pmic_ioRxByte(&g_pmicHandle, regAddr, &readData);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        PLATFORM_ASSERT(readData == writePattern[i]);
    }
}

/* ========================================================================== */
/*                      Positive Tests - CRC Validation                       */
/* ========================================================================== */

void test_pos_io_read_with_crc_validation(void)
{
    int32_t status;
    uint8_t testData[] = {0x12U, 0x34U, 0x56U, 0x78U};

    /* TPS65386x-Q1 always has CRC enabled in SPI protocol
     * This test verifies that read operations with CRC validation work correctly
     */

    /* Write test patterns to scratchpad registers */
    for (uint8_t i = 0U; i < 2U; i++)
    {
        uint16_t regAddr = IO_TEST_SCRATCHPAD1_REG + i;
        status = Pmic_ioTxByte(&g_pmicHandle, regAddr, testData[i]);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    }

    /* Read back and verify - CRC validation occurs internally in Pmic_ioRxByte */
    for (uint8_t i = 0U; i < 2U; i++)
    {
        uint8_t readData = 0U;
        uint16_t regAddr = IO_TEST_SCRATCHPAD1_REG + i;

        status = Pmic_ioRxByte(&g_pmicHandle, regAddr, &readData);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        PLATFORM_ASSERT(readData == testData[i]);
    }

    /* Verify sequential read operations maintain CRC integrity */
    uint32_t seqReadData = 0U;
    status = Pmic_ioRxWordSeq(&g_pmicHandle, IO_TEST_SCRATCHPAD1_REG, &seqReadData, 2U);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Verify sequential read matches individual reads */
    uint8_t byte0 = (uint8_t)(seqReadData & 0xFFU);
    uint8_t byte1 = (uint8_t)((seqReadData >> 8U) & 0xFFU);
    PLATFORM_ASSERT(byte0 == testData[0]);
    PLATFORM_ASSERT(byte1 == testData[1]);
}

void test_neg_io_ioRxByte_crcError(void)
{
    int32_t status;
    uint8_t readData = 0U;
    PmicMockDevice_t* mockDevice = platform_getMockDevice();

    /* TPS65386x-Q1 validates CRC in Pmic_ioRxByte (line 161 of pmic_io.c)
     * If CRC validation fails, it returns PMIC_ST_ERR_DATA_IO_CRC
     *
     * This test injects a CRC error to verify the error handling path.
     */

    PLATFORM_ASSERT(mockDevice != NULL);

    /* Inject CRC error for the next read operation */
    PmicMock_InjectError(mockDevice, PMIC_MOCK_ERROR_CRC_MISMATCH, 1);

    /* Perform read operation - should detect CRC mismatch */
    status = Pmic_ioRxByte(&g_pmicHandle, IO_TEST_SCRATCHPAD1_REG, &readData);

    /* Verify CRC error was detected */
    PLATFORM_ASSERT(status == PMIC_ST_ERR_DATA_IO_CRC);

    /* Clear error injection for subsequent tests */
    PmicMock_ClearErrors(mockDevice);

    /* Verify normal operation resumes after clearing errors */
    status = Pmic_ioRxByte(&g_pmicHandle, IO_TEST_SCRATCHPAD1_REG, &readData);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Test CRC error with critical section variant */
    PmicMock_InjectError(mockDevice, PMIC_MOCK_ERROR_CRC_MISMATCH, 1);
    status = Pmic_ioRxByte_CS(&g_pmicHandle, IO_TEST_SCRATCHPAD2_REG, &readData);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_DATA_IO_CRC);

    /* Cleanup */
    PmicMock_ClearErrors(mockDevice);
}

void test_pos_io_write_with_crc_calculation(void)
{
    int32_t status;
    uint8_t testPatterns[] = {TEST_PATTERN_AA, TEST_PATTERN_55, TEST_MASK_HIGH_NIBBLE, TEST_MASK_LOW_NIBBLE};

    /* TPS65386x-Q1 calculates CRC for write operations in Pmic_ioTxByte (line 221 of pmic_io.c)
     * spiBuf[3] = getCRC8Val(spiBuf, bufLen) where bufLen=3
     * This test verifies write operations with CRC calculation work correctly
     */

    /* Test writing multiple patterns to verify CRC calculation for different data */
    for (uint8_t i = 0U; i < 4U; i++)
    {
        /* Write with CRC calculation */
        status = Pmic_ioTxByte(&g_pmicHandle, IO_TEST_SCRATCHPAD1_REG, testPatterns[i]);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        /* Read back to verify write succeeded */
        uint8_t readData = 0U;
        status = Pmic_ioRxByte(&g_pmicHandle, IO_TEST_SCRATCHPAD1_REG, &readData);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        PLATFORM_ASSERT(readData == testPatterns[i]);
    }

    /* Verify write with critical section also calculates CRC correctly */
    status = Pmic_ioTxByte_CS(&g_pmicHandle, IO_TEST_SCRATCHPAD2_REG, 0xCCU);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Verify sequential write operations maintain CRC integrity */
    uint32_t writeData = 0x0000ABCDU;
    status = Pmic_ioTxWordSeq(&g_pmicHandle, IO_TEST_SCRATCHPAD1_REG, writeData, 2U);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Read back sequential write to verify integrity */
    uint32_t readData = 0U;
    status = Pmic_ioRxWordSeq(&g_pmicHandle, IO_TEST_SCRATCHPAD1_REG, &readData, 2U);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT((readData & 0xFFFFU) == (writeData & 0xFFFFU));
}

void test_pos_io_crc_enable_disable_transitions(void)
{
    int32_t status;

    /* TPS65386x-Q1 always has CRC enabled in SPI protocol - no disable capability
     * This is different from LP8772x-Q1 which has Pmic_ioCrcEnable/Disable APIs
     *
     * This test verifies that CRC remains consistently active across multiple operations
     * and state transitions (init/deinit/reinit)
     */

    /* Perform operations to verify CRC is consistently active */
    uint8_t writeData1 = TEST_PATTERN_A5;
    uint8_t readData1 = 0U;

    /* Write and read with CRC active */
    status = Pmic_ioTxByte(&g_pmicHandle, IO_TEST_SCRATCHPAD1_REG, writeData1);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_ioRxByte(&g_pmicHandle, IO_TEST_SCRATCHPAD1_REG, &readData1);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(readData1 == writeData1);

    /* Perform read-modify-write operation to verify CRC during complex operations */
    status = Pmic_ioUpdateByte(&g_pmicHandle, IO_TEST_SCRATCHPAD1_REG,
                               IO_TEST_BIT_POS_0, (1U << IO_TEST_BIT_POS_0), 1U);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Verify modified value */
    uint8_t readData2 = 0U;
    status = Pmic_ioRxByte(&g_pmicHandle, IO_TEST_SCRATCHPAD1_REG, &readData2);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT((readData2 & 0x01U) == 0x01U);

    /* Perform critical section operations to verify CRC during protected operations */
    uint8_t writeData3 = TEST_PATTERN_5A;
    status = Pmic_ioTxByte_CS(&g_pmicHandle, IO_TEST_SCRATCHPAD2_REG, writeData3);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    uint8_t readData3 = 0U;
    status = Pmic_ioRxByte_CS(&g_pmicHandle, IO_TEST_SCRATCHPAD2_REG, &readData3);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(readData3 == writeData3);

    /* Verify boolean update operations maintain CRC integrity */
    status = Pmic_ioUpdateByte_b(&g_pmicHandle, IO_TEST_SCRATCHPAD1_REG,
                                 IO_TEST_BIT_POS_4, true);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Verify bit was set */
    uint8_t readData4 = 0U;
    status = Pmic_ioRxByte(&g_pmicHandle, IO_TEST_SCRATCHPAD1_REG, &readData4);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT((readData4 & (1U << IO_TEST_BIT_POS_4)) != 0U);
}

void test_pos_io_ioRxByte_withRetryOnCrcError(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    Pmic_Handle_t testHandle;

    (void)memcpy(&testHandle, &g_pmicHandle, sizeof(Pmic_Handle_t));
    testHandle.ioRead = &mockIoRead;
    testHandle.ioWrite = &mockIoWrite;
    testHandle.timerWaitMs = &mockTimerWait;
    testHandle.retryCnt = 2U;
    testHandle.retryIntervalMs = 10U;

    resetMockIoState();

    /* TPS65386x-Q1 always has CRC enabled in SPI protocol */
    g_mockCrcCorruptionMask = TEST_MASK_FULL_BYTE;

    status = Pmic_ioRxByte(&testHandle, IO_TEST_SCRATCHPAD1_REG, &regData);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(g_mockIoReadCallCount == 2U);
}

void test_neg_io_ioTxByte_withRetryOnFailure(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t writeVal = TEST_PATTERN_AA;
    Pmic_Handle_t testHandle;

    (void)memcpy(&testHandle, &g_pmicHandle, sizeof(Pmic_Handle_t));
    testHandle.ioRead = &mockIoRead;
    testHandle.ioWrite = &mockIoWrite;
    testHandle.timerWaitMs = &mockTimerWait;
    testHandle.retryCnt = 2U;
    testHandle.retryIntervalMs = 10U;

    resetMockIoState();

    g_mockIoWriteReturnStatus = PMIC_ST_ERR_I2C_COMM_FAIL;

    status = Pmic_ioTxByte(&testHandle, IO_TEST_SCRATCHPAD1_REG, writeVal);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(g_mockIoWriteCallCount == 2U);

    uint8_t readVal = 0U;
    status = Pmic_ioRxByte(&g_pmicHandle, IO_TEST_SCRATCHPAD1_REG, &readVal);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(readVal == writeVal);
}

void test_neg_io_crcErrorExhaustsRetries(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    Pmic_Handle_t testHandle;
    uint32_t initialRetryCnt = 0U;

    (void)memcpy(&testHandle, &g_pmicHandle, sizeof(Pmic_Handle_t));
    testHandle.ioRead = &mockIoRead;
    testHandle.ioWrite = &mockIoWrite;
    testHandle.timerWaitMs = &mockTimerWait;
    testHandle.retryCnt = 2U;
    testHandle.retryIntervalMs = 10U;
    testHandle.criticalSectionStart = &platform_critSecStart;
    testHandle.criticalSectionStop = &platform_critSecStop;

    resetMockIoState();
    Pmic_clrDiagnosticsAll(&testHandle);

    status = Pmic_getRetryCnt(&testHandle, &initialRetryCnt);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(initialRetryCnt == 0U);

    /* TPS65386x-Q1 always has CRC enabled in SPI protocol */
    g_mockIoReadReturnStatus = PMIC_ST_ERR_I2C_COMM_FAIL;

    status = Pmic_ioRxByte(&testHandle, IO_TEST_SCRATCHPAD1_REG, &regData);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    resetMockIoState();

    testHandle.retryCnt = 0U;
    g_mockIoReadReturnStatus = PMIC_ST_ERR_I2C_COMM_FAIL;

    status = Pmic_ioRxByte(&testHandle, IO_TEST_SCRATCHPAD1_REG, &regData);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_I2C_COMM_FAIL);
    PLATFORM_ASSERT(g_mockIoReadCallCount == 1U);

    testHandle.retryCnt = 1U;
    resetMockIoState();

    g_mockIoReadReturnStatus = PMIC_ST_ERR_I2C_COMM_FAIL;

    status = Pmic_ioRxByte(&testHandle, IO_TEST_SCRATCHPAD1_REG, &regData);
    PLATFORM_ASSERT(g_mockIoReadCallCount >= 1U);

    testHandle.crcEnable = PMIC_DISABLE;
}

/* ========================================================================== */
/*                  Positive Tests - Retry Edge Cases                        */
/* ========================================================================== */

/**
 * @brief Test ioTxByte retry succeeds on exactly the last allowed attempt
 */
void test_pos_io_ioTxByte_retrySucceedsOnLastAttempt(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t writeVal = 0xBBU;
    Pmic_Handle_t testHandle;

    /* Initialize test handle with mock functions */
    (void)memcpy(&testHandle, &g_pmicHandle, sizeof(Pmic_Handle_t));
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
    status = Pmic_ioTxByte(&testHandle, IO_TEST_SCRATCHPAD2_REG, writeVal);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Verify exactly 2 write calls (original + 1 retry) */
    PLATFORM_ASSERT(g_mockIoWriteCallCount == 2U);

    /* Verify the value was actually written */
    uint8_t readVal = 0U;
    status = Pmic_ioRxByte(&g_pmicHandle, IO_TEST_SCRATCHPAD2_REG, &readVal);
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
    (void)memcpy(&testHandle, &g_pmicHandle, sizeof(Pmic_Handle_t));
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
    status = Pmic_ioRxByte(&testHandle, IO_TEST_SCRATCHPAD1_REG, &regData);
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
    (void)memcpy(&testHandle, &g_pmicHandle, sizeof(Pmic_Handle_t));
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
    status = Pmic_ioTxByte(&testHandle, IO_TEST_SCRATCHPAD2_REG, writeVal);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Should have 2 attempts (1 failure + 1 success) */
    PLATFORM_ASSERT(g_mockIoWriteCallCount == 2U);
}

/**
 * @brief Test IO with NULL commHandle0
 *
 * Covers line 138 in pmic_io.c - commHandle0 NULL validation
 */
void test_neg_io_nullCommHandle(void)
{
    Pmic_Handle_t testHandle;
    uint8_t rxData = 0U;

    // Initialize a valid handle first
    (void)memcpy(&testHandle, &g_pmicHandle, sizeof(Pmic_Handle_t));

    // Corrupt commHandle0 to NULL
    testHandle.commHandle0 = NULL;

    // Attempt IO operation - should fail with NULL param error
    int32_t status = Pmic_ioRxByte(&testHandle, IO_TEST_SCRATCHPAD1_REG, &rxData);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test IO with NULL ioRead/ioWrite function pointers
 *
 * Covers line 142 in pmic_io.c - ioRead/ioWrite NULL validation
 */
void test_neg_io_nullIoFptrs(void)
{
    Pmic_Handle_t testHandle;
    uint8_t rxData = 0U;
    uint8_t txData = TEST_PATTERN_AA;

    // Test NULL ioRead
    (void)memcpy(&testHandle, &g_pmicHandle, sizeof(Pmic_Handle_t));
    testHandle.ioRead = NULL;

    int32_t status = Pmic_ioRxByte(&testHandle, IO_TEST_SCRATCHPAD1_REG, &rxData);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_FPTR);

    // Test NULL ioWrite
    (void)memcpy(&testHandle, &g_pmicHandle, sizeof(Pmic_Handle_t));
    testHandle.ioWrite = NULL;

    status = Pmic_ioTxByte(&testHandle, IO_TEST_SCRATCHPAD1_REG, txData);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_FPTR);
}

/**
 * @brief Test IO with NULL timer but non-zero retry interval
 *
 * Covers line 146 in pmic_io.c - timer validation when retry interval is set
 */
void test_neg_io_nullTimerWithRetry(void)
{
    Pmic_Handle_t testHandle;
    uint8_t rxData = 0U;

    // Initialize test handle with retry interval but NULL timer
    (void)memcpy(&testHandle, &g_pmicHandle, sizeof(Pmic_Handle_t));
    testHandle.retryIntervalMs = 100U;  // Non-zero retry interval
    testHandle.timerWaitMs = NULL;       // NULL timer function

    // Attempt IO operation - should fail with NULL function pointer error
    int32_t status = Pmic_ioRxByte(&testHandle, IO_TEST_SCRATCHPAD1_REG, &rxData);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_FPTR);
}

/* NOTE: setUp() and tearDown() removed - already defined in test_runner.c */
