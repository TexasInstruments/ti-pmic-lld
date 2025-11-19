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
 * @brief Source file containing definitions to PMIC IO tests.
 */

/* ========================================================================== */
/*                              Include Files                                 */
/* ========================================================================== */

#include "io_test.h"

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/* Run all IO tests */
#define IO_TEST_RUN_ALL() PLATFORM_RUN_TEST(test_negative_Pmic_ioTxByte_nullParam_handle); \
                          PLATFORM_RUN_TEST(test_negative_Pmic_ioRxByte_nullParam_handle); \
                          PLATFORM_RUN_TEST(test_negative_Pmic_ioRxByte_nullParam_rxBuffer); \
                          PLATFORM_RUN_TEST(test_negative_Pmic_ioTxByte_CS_nullParam_handle); \
                          PLATFORM_RUN_TEST(test_negative_Pmic_ioRxByte_CS_nullParam_handle); \
                          PLATFORM_RUN_TEST(test_negative_Pmic_ioRxByte_CS_nullParam_rxBuffer); \
                          PLATFORM_RUN_TEST(test_negative_Pmic_ioGetCrcEnableState_nullParam_handle); \
                          PLATFORM_RUN_TEST(test_negative_Pmic_ioGetCrcEnableState_nullParam_isEnabled); \
                          PLATFORM_RUN_TEST(test_negative_Pmic_ioSetCrcEnableState_nullParam_handle); \
                          PLATFORM_RUN_TEST(test_negative_Pmic_ioCrcEnable_nullParam_handle); \
                          PLATFORM_RUN_TEST(test_negative_Pmic_ioCrcDisable_nullParam_handle); \
                          PLATFORM_RUN_TEST(test_positive_Pmic_ioTxByte_Pmic_ioRxByte_writeReadScratchpadReg1To4); \
                          PLATFORM_RUN_TEST(test_positive_Pmic_ioTxByte_CS_Pmic_ioRxByte_CS_writeReadScratchpadReg1To4); \
                          PLATFORM_RUN_TEST(test_positive_setGetCrcEnableState); \
                          PLATFORM_RUN_TEST(test_positive_enableDisableCrc)

/* Run all IO negative tests */
#define IO_TEST_RUN_NEGATIVE() PLATFORM_RUN_TEST(test_negative_Pmic_ioTxByte_nullParam_handle); \
                               PLATFORM_RUN_TEST(test_negative_Pmic_ioRxByte_nullParam_handle); \
                               PLATFORM_RUN_TEST(test_negative_Pmic_ioRxByte_nullParam_rxBuffer); \
                               PLATFORM_RUN_TEST(test_negative_Pmic_ioTxByte_CS_nullParam_handle); \
                               PLATFORM_RUN_TEST(test_negative_Pmic_ioRxByte_CS_nullParam_handle); \
                               PLATFORM_RUN_TEST(test_negative_Pmic_ioRxByte_CS_nullParam_rxBuffer); \
                               PLATFORM_RUN_TEST(test_negative_Pmic_ioGetCrcEnableState_nullParam_handle); \
                               PLATFORM_RUN_TEST(test_negative_Pmic_ioGetCrcEnableState_nullParam_isEnabled); \
                               PLATFORM_RUN_TEST(test_negative_Pmic_ioSetCrcEnableState_nullParam_handle); \
                               PLATFORM_RUN_TEST(test_negative_Pmic_ioCrcEnable_nullParam_handle); \
                               PLATFORM_RUN_TEST(test_negative_Pmic_ioCrcDisable_nullParam_handle)

/* Run all IO positive tests */
#define IO_TEST_RUN_POSITIVE() PLATFORM_RUN_TEST(test_positive_Pmic_ioTxByte_Pmic_ioRxByte_writeReadScratchpadReg1To4); \
                               PLATFORM_RUN_TEST(test_positive_Pmic_ioTxByte_CS_Pmic_ioRxByte_CS_writeReadScratchpadReg1To4); \
                               PLATFORM_RUN_TEST(test_positive_setGetCrcEnableState); \
                               PLATFORM_RUN_TEST(test_positive_enableDisableCrc)

/* PMIC scratchpad register addresses */
#define IO_TEST_SCRATCH_PAD_REG_1_REG (0x0AU)
#define IO_TEST_SCRATCH_PAD_REG_2_REG (0x0BU)
#define IO_TEST_SCRATCH_PAD_REG_3_REG (0x0CU)
#define IO_TEST_SCRATCH_PAD_REG_4_REG (0x0DU)

/* Max PMIC user-space register address */
#define IO_TEST_MAX_REG (0x62U)

/* ========================================================================== */
/*                             Global Variables                               */
/* ========================================================================== */
Pmic_CoreHandle_t pmicHandle;

/* ========================================================================== */
/*                           Function Declarations                            */
/* ========================================================================== */
static int32_t ioTest_unlockPmicRegs(Pmic_CoreHandle_t *pmicHandle);

/* ========================================================================== */
/*                           Function Definitions                             */
/* ========================================================================== */

void io_test(void *args)
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

    (void)Pmic_deinit(&pmicHandle);
    platform_deinit();
}

static int32_t ioTest_unlockPmicRegs(Pmic_CoreHandle_t *pmicHandle)
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

void test_negative_Pmic_ioTxByte_nullParam_handle(void)
{
    // Pass null handle into Pmic_ioTxByte()
    const uint8_t regData = 0xAAU;
    int32_t status = Pmic_ioTxByte(NULL, IO_TEST_SCRATCH_PAD_REG_1_REG, regData);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_HANDLE);
}

void test_negative_Pmic_ioRxByte_nullParam_handle(void)
{
    // Pass null handle into Pmic_ioRxByte()
    uint8_t regData = 0U;
    int32_t status = Pmic_ioRxByte(NULL, IO_TEST_SCRATCH_PAD_REG_1_REG, &regData);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_HANDLE);
}

void test_negative_Pmic_ioRxByte_nullParam_rxBuffer(void)
{
    // Pass null rxBuffer into Pmic_ioRxByte()
    int32_t status = Pmic_ioRxByte(&pmicHandle, IO_TEST_SCRATCH_PAD_REG_1_REG, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_ioTxByte_CS_nullParam_handle(void)
{
    // Pass null handle into Pmic_ioTxByte_CS()
    const uint8_t regData = 0xAAU;
    int32_t status = Pmic_ioTxByte_CS(NULL, IO_TEST_SCRATCH_PAD_REG_1_REG, regData);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_HANDLE);
}

void test_negative_Pmic_ioRxByte_CS_nullParam_handle(void)
{
    // Pass null handle into Pmic_ioRxByte_CS()
    uint8_t regData = 0U;
    int32_t status = Pmic_ioRxByte_CS(NULL, IO_TEST_SCRATCH_PAD_REG_1_REG, &regData);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_HANDLE);
}

void test_negative_Pmic_ioRxByte_CS_nullParam_rxBuffer(void)
{
    // Pass null rxBuffer into Pmic_ioRxByte_CS()
    int32_t status = Pmic_ioRxByte_CS(&pmicHandle, IO_TEST_SCRATCH_PAD_REG_1_REG, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_ioGetCrcEnableState_nullParam_handle(void)
{
    // Pass null handle into Pmic_ioGetCrcEnableState()
    bool isEnabled = PMIC_DISABLE;
    int32_t status = Pmic_ioGetCrcEnableState(NULL, &isEnabled);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_HANDLE);
}

void test_negative_Pmic_ioGetCrcEnableState_nullParam_isEnabled(void)
{
    // Pass null isEnabled into Pmic_ioGetCrcEnableState()
    int32_t status = Pmic_ioGetCrcEnableState(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_ioSetCrcEnableState_nullParam_handle(void)
{
    // Pass null handle into Pmic_ioSetCrcEnableState()
    int32_t status = Pmic_ioSetCrcEnableState(NULL, PMIC_ENABLE);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_HANDLE);
}

void test_negative_Pmic_ioCrcEnable_nullParam_handle(void)
{
    // Pass null handle into Pmic_ioCrcEnable()
    int32_t status = Pmic_ioCrcEnable(NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_HANDLE);
}

void test_negative_Pmic_ioCrcDisable_nullParam_handle(void)
{
    // Pass null handle into Pmic_ioCrcDisable()
    int32_t status = Pmic_ioCrcDisable(NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_HANDLE);
}

void test_positive_Pmic_ioTxByte_Pmic_ioRxByte_writeReadScratchpadReg1To4(void)
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

        // Get actual scratchpad register value and compare against inital and expected values
        status = Pmic_ioRxByte(&pmicHandle, regAddr, &actVal);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        PLATFORM_ASSERT(actVal != initVal);
        PLATFORM_ASSERT(actVal == expVal);
    }
}

void test_positive_Pmic_ioTxByte_CS_Pmic_ioRxByte_CS_writeReadScratchpadReg1To4(void)
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

        // Get actual scratchpad register value and compare against inital and expected values
        status = Pmic_ioRxByte_CS(&pmicHandle, regAddr, &actVal);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        PLATFORM_ASSERT(actVal != initVal);
        PLATFORM_ASSERT(actVal == expVal);
    }
}

void test_positive_setGetCrcEnableState(void)
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

void test_positive_enableDisableCrc(void)
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
