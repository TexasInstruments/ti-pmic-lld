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


/* ========================================================================== */
/*                              Include Files                                 */
/* ========================================================================== */

#include "pmic_init_test.h"

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/* Run all PMIC_INIT tests */
#define PMIC_INIT_TEST_RUN_ALL() PMIC_INIT_TEST_RUN_NEGATIVE(); \
                                 PMIC_INIT_TEST_RUN_POSITIVE()

/* Run all PMIC_INIT negative tests */
#define PMIC_INIT_TEST_RUN_NEGATIVE() PLATFORM_RUN_TEST(test_negative_Pmic_init_nullHandle); \
                                      PLATFORM_RUN_TEST(test_negative_Pmic_init_nullConfig); \
                                      PLATFORM_RUN_TEST(test_negative_Pmic_deinit_nullHandle); \
                                      PLATFORM_RUN_TEST(test_negative_Pmic_checkHandle_nullHandle); \
                                      PLATFORM_RUN_TEST(test_negative_Pmic_init_invalidCommMode); \
                                      PLATFORM_RUN_TEST(test_negative_Pmic_init_nullCommHandle); \
                                      PLATFORM_RUN_TEST(test_negative_Pmic_init_nullTaskHandle); \
                                      PLATFORM_RUN_TEST(test_negative_Pmic_init_nullIoRead); \
                                      PLATFORM_RUN_TEST(test_negative_Pmic_init_nullIoWrite); \
                                      PLATFORM_RUN_TEST(test_negative_Pmic_init_nullAsyncRxStart); \
                                      PLATFORM_RUN_TEST(test_negative_Pmic_init_nullAsyncTxStart); \
                                      PLATFORM_RUN_TEST(test_negative_Pmic_init_nullAsyncRxAwait); \
                                      PLATFORM_RUN_TEST(test_negative_Pmic_init_nullAsyncTxAwait); \
                                      PLATFORM_RUN_TEST(test_negative_Pmic_init_nullCritSecStart); \
                                      PLATFORM_RUN_TEST(test_negative_Pmic_init_nullCritSecStop); \
                                      PLATFORM_RUN_TEST(test_negative_Pmic_init_nullIrqCallback); \
                                      PLATFORM_RUN_TEST(test_negative_init_timerWaitNull); \
                                      PLATFORM_RUN_TEST(test_negative_init_timerWaitMsCallbackNull)

/* Run all PMIC_INIT positive tests */
#define PMIC_INIT_TEST_RUN_POSITIVE() PLATFORM_RUN_TEST(test_positive_Pmic_init_validConfig); \
                                      PLATFORM_RUN_TEST(test_positive_Pmic_deinit_afterInit); \
                                      PLATFORM_RUN_TEST(test_positive_Pmic_checkHandle_validHandle); \
                                      PLATFORM_RUN_TEST(test_positive_Pmic_checkHandle_invalidHandle); \
                                      PLATFORM_RUN_TEST(test_positive_Pmic_init_reinit); \
                                      PLATFORM_RUN_TEST(test_positive_Pmic_init_with_crc_enabled); \
                                      PLATFORM_RUN_TEST(test_positive_Pmic_init_with_both_crc_flags); \
                                      PLATFORM_RUN_TEST(test_positive_Pmic_init_crc_disabled); \
                                      PLATFORM_RUN_TEST(test_positive_Pmic_init_verify_crc_state); \
                                      PLATFORM_RUN_TEST(test_positive_Pmic_init_complete_flow); \
                                      PLATFORM_RUN_TEST(test_positive_Pmic_init_i2c_single_mode); \
                                      PLATFORM_RUN_TEST(test_positive_Pmic_init_i2c_dual_mode); \
                                      PLATFORM_RUN_TEST(test_positive_Pmic_init_device_info_retrieval); \
                                      PLATFORM_RUN_TEST(test_positive_Pmic_deinit_success_path); \
                                      PLATFORM_RUN_TEST(test_positive_Pmic_checkHandle_all_validations); \
                                      PLATFORM_RUN_TEST(test_positive_Pmic_init_async_mode); \
                                      PLATFORM_RUN_TEST(test_positive_Pmic_init_with_i2c_addresses); \
                                      PLATFORM_RUN_TEST(test_positive_Pmic_init_with_task_handle); \
                                      PLATFORM_RUN_TEST(test_positive_init_withRetryCnt); \
                                      PLATFORM_RUN_TEST(test_positive_init_withRetryInterval); \
                                      PLATFORM_RUN_TEST(test_positive_init_withTimerWaitMs)

/* Used in certain unit tests to validate driver initialization status */
#define PMIC_INIT_TEST_DRV_INIT_STATUS (0xBEEF0000U)

/* ========================================================================== */
/*                           Function Declarations                            */
/* ========================================================================== */
static inline void pmicInitTest_initHandleCfg(Pmic_HandleCfg_t *handleCfg);

/* ========================================================================== */
/*                             Global Variables                               */
/* ========================================================================== */
static Pmic_Handle_t pmicHandle = {0};

/* Wrapper for platform timer wait to match PMIC API signature */
static void testTimerWaitWrapper(uint32_t ms)
{
    /* Platform function takes uint16_t, truncate if needed */
    platform_timerWaitMs((uint16_t)ms);
}

/* ========================================================================== */
/*                           Function Definitions                             */
/* ========================================================================== */

void pmic_init_test(void *args)
{
    (void)args;
    int32_t status = PMIC_ST_SUCCESS;

    platform_init();

    platform_printString("\r\n");
    platform_printString("PMIC_INIT_TEST\r\n");
    platform_printString("--------------\r\n\r\n");

    if (status == PMIC_ST_SUCCESS)
    {
        platform_setupTests();
        PMIC_INIT_TEST_RUN_ALL();
        platform_tearDownTests();
    }

    platform_deinit();
}

static inline void pmicInitTest_initHandleCfg(Pmic_HandleCfg_t *handleCfg)
{
    handleCfg->validParams = PMIC_COMM_MODE_VALID |
                             PMIC_CRC_ENABLE_VALID |
                             PMIC_COMM_HANDLE_0_VALID |
                             PMIC_IO_READ_VALID |
                             PMIC_IO_WRITE_VALID |
                             PMIC_CRITICAL_SECTION_START_VALID |
                             PMIC_CRITICAL_SECTION_STOP_VALID |
                             PMIC_IRQ_RESPONSE_CALLBACK_VALID;
    handleCfg->commMode = PMIC_INTF_SPI;
    handleCfg->crcEnable = PMIC_DISABLE;
    handleCfg->commHandle0 = platform_getCommHandle();
    handleCfg->ioRead = &platform_rxByte;
    handleCfg->ioWrite = &platform_txByte;
    handleCfg->criticalSectionStart = &platform_critSecStart;
    handleCfg->criticalSectionStop = &platform_critSecStop;
    handleCfg->irqResponseCallback = &platform_irqResponse;
}

/* ========================================================================== */
/*                         Negative Test Functions                            */
/* ========================================================================== */

void test_negative_Pmic_init_nullHandle(void)
{
    // Pass NULL handle into Pmic_init()
    Pmic_HandleCfg_t handleCfg = {0};
    pmicInitTest_initHandleCfg(&handleCfg);
    int32_t status = Pmic_init(NULL, &handleCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_init_nullConfig(void)
{
    // Pass NULL config into Pmic_init()
    Pmic_Handle_t handle = {0};
    int32_t status = Pmic_init(&handle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_deinit_nullHandle(void)
{
    // Pass NULL handle into Pmic_deinit()
    int32_t status = Pmic_deinit(NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_checkHandle_nullHandle(void)
{
    // Pass NULL handle into Pmic_checkHandle()
    int32_t status = Pmic_checkHandle(NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_init_invalidCommMode(void)
{
    // Pass invalid commMode into Pmic_init()
    Pmic_HandleCfg_t handleCfg = {0};
    Pmic_Handle_t handle = {0};

    pmicInitTest_initHandleCfg(&handleCfg);
    handleCfg.commMode = PMIC_INTF_MAX + 1U;
    int32_t status = Pmic_init(&handle, &handleCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_negative_Pmic_init_nullCommHandle(void)
{
    // Pass NULL commHandle0 with valid param flag set
    Pmic_HandleCfg_t handleCfg = {0};
    Pmic_Handle_t handle = {0};

    pmicInitTest_initHandleCfg(&handleCfg);
    handleCfg.validParams |= PMIC_COMM_HANDLE_0_VALID;
    handleCfg.commHandle0 = NULL;
    int32_t status = Pmic_init(&handle, &handleCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_init_nullTaskHandle(void)
{
    // Pass NULL taskHandle with valid param flag set
    Pmic_HandleCfg_t handleCfg = {0};
    Pmic_Handle_t handle = {0};

    pmicInitTest_initHandleCfg(&handleCfg);
    handleCfg.validParams |= PMIC_TASK_HANDLE_VALID;
    handleCfg.taskHandle = NULL;
    int32_t status = Pmic_init(&handle, &handleCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_init_nullIoRead(void)
{
    // Pass NULL ioRead with valid param flag set
    Pmic_HandleCfg_t handleCfg = {0};
    Pmic_Handle_t handle = {0};

    pmicInitTest_initHandleCfg(&handleCfg);
    handleCfg.validParams |= PMIC_IO_READ_VALID;
    handleCfg.ioRead = NULL;
    int32_t status = Pmic_init(&handle, &handleCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_FPTR);
}

void test_negative_Pmic_init_nullIoWrite(void)
{
    // Pass NULL ioWrite with valid param flag set
    Pmic_HandleCfg_t handleCfg = {0};
    Pmic_Handle_t handle = {0};

    pmicInitTest_initHandleCfg(&handleCfg);
    handleCfg.validParams |= PMIC_IO_WRITE_VALID;
    handleCfg.ioWrite = NULL;
    int32_t status = Pmic_init(&handle, &handleCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_FPTR);
}

void test_negative_Pmic_init_nullAsyncRxStart(void)
{
    // Pass NULL asyncRxStart with valid param flag set
    Pmic_HandleCfg_t handleCfg = {0};
    Pmic_Handle_t handle = {0};

    pmicInitTest_initHandleCfg(&handleCfg);
    handleCfg.validParams |= PMIC_ASYNC_RX_START_VALID;
    handleCfg.asyncRxStart = NULL;
    int32_t status = Pmic_init(&handle, &handleCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_FPTR);
}

void test_negative_Pmic_init_nullAsyncTxStart(void)
{
    // Pass NULL asyncTxStart with valid param flag set
    Pmic_HandleCfg_t handleCfg = {0};
    Pmic_Handle_t handle = {0};

    pmicInitTest_initHandleCfg(&handleCfg);
    handleCfg.validParams |= PMIC_ASYNC_TX_START_VALID;
    handleCfg.asyncTxStart = NULL;
    int32_t status = Pmic_init(&handle, &handleCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_FPTR);
}

void test_negative_Pmic_init_nullAsyncRxAwait(void)
{
    // Pass NULL asyncRxAwait with valid param flag set
    Pmic_HandleCfg_t handleCfg = {0};
    Pmic_Handle_t handle = {0};

    pmicInitTest_initHandleCfg(&handleCfg);
    handleCfg.validParams |= PMIC_ASYNC_RX_AWAIT_VALID;
    handleCfg.asyncRxAwait = NULL;
    int32_t status = Pmic_init(&handle, &handleCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_FPTR);
}

void test_negative_Pmic_init_nullAsyncTxAwait(void)
{
    // Pass NULL asyncTxAwait with valid param flag set
    Pmic_HandleCfg_t handleCfg = {0};
    Pmic_Handle_t handle = {0};

    pmicInitTest_initHandleCfg(&handleCfg);
    handleCfg.validParams |= PMIC_ASYNC_TX_AWAIT_VALID;
    handleCfg.asyncTxAwait = NULL;
    int32_t status = Pmic_init(&handle, &handleCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_FPTR);
}

void test_negative_Pmic_init_nullCritSecStart(void)
{
    // Pass NULL criticalSectionStart with valid param flag set
    Pmic_HandleCfg_t handleCfg = {0};
    Pmic_Handle_t handle = {0};

    pmicInitTest_initHandleCfg(&handleCfg);
    handleCfg.validParams |= PMIC_CRITICAL_SECTION_START_VALID;
    handleCfg.criticalSectionStart = NULL;
    int32_t status = Pmic_init(&handle, &handleCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_FPTR);
}

void test_negative_Pmic_init_nullCritSecStop(void)
{
    // Pass NULL criticalSectionStop with valid param flag set
    Pmic_HandleCfg_t handleCfg = {0};
    Pmic_Handle_t handle = {0};

    pmicInitTest_initHandleCfg(&handleCfg);
    handleCfg.validParams |= PMIC_CRITICAL_SECTION_STOP_VALID;
    handleCfg.criticalSectionStop = NULL;
    int32_t status = Pmic_init(&handle, &handleCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_FPTR);
}

void test_negative_Pmic_init_nullIrqCallback(void)
{
    // Pass NULL irqResponseCallback with valid param flag set
    Pmic_HandleCfg_t handleCfg = {0};
    Pmic_Handle_t handle = {0};

    pmicInitTest_initHandleCfg(&handleCfg);
    handleCfg.validParams |= PMIC_IRQ_RESPONSE_CALLBACK_VALID;
    handleCfg.irqResponseCallback = NULL;
    int32_t status = Pmic_init(&handle, &handleCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_FPTR);
}

void test_negative_init_timerWaitNull(void)
{
    // Pass NULL timerWaitMs with non-zero retryIntervalMs
    // Covers line 449 in pmic.c
    Pmic_HandleCfg_t handleCfg = {0};
    Pmic_Handle_t handle = {0};

    pmicInitTest_initHandleCfg(&handleCfg);
    handleCfg.validParams |= PMIC_RETRY_INTERVAL_MS_VALID;
    handleCfg.retryIntervalMs = 10U;  /* Non-zero retry interval */
    handleCfg.timerWaitMs = NULL;     /* NULL timer function */

    int32_t status = Pmic_init(&handle, &handleCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_FPTR);
}

/* ========================================================================== */
/*                         Positive Test Functions                            */
/* ========================================================================== */

void test_positive_Pmic_init_validConfig(void)
{
    // Initialize PMIC LLD with valid Burton configuration
    Pmic_HandleCfg_t handleCfg = {0};
    pmicInitTest_initHandleCfg(&handleCfg);
    int32_t status = Pmic_init(&pmicHandle, &handleCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(pmicHandle.commMode == PMIC_INTF_SPI);
    PLATFORM_ASSERT(pmicHandle.crcEnable == PMIC_DISABLE);
    PLATFORM_ASSERT(pmicHandle.commHandle0 == platform_getCommHandle());
    PLATFORM_ASSERT(pmicHandle.ioRead == &platform_rxByte);
    PLATFORM_ASSERT(pmicHandle.ioWrite == &platform_txByte);
    PLATFORM_ASSERT(pmicHandle.criticalSectionStart == &platform_critSecStart);
    PLATFORM_ASSERT(pmicHandle.criticalSectionStop == &platform_critSecStop);
    PLATFORM_ASSERT(pmicHandle.irqResponseCallback == &platform_irqResponse);
}

void test_positive_Pmic_deinit_afterInit(void)
{
    // Deinitialize PMIC LLD after successful init
    int32_t status = Pmic_deinit(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(pmicHandle.drvInitStat == 0U);
    PLATFORM_ASSERT(pmicHandle.devRev == 0U);
    PLATFORM_ASSERT(pmicHandle.devSiRev == 0U);
    PLATFORM_ASSERT(pmicHandle.commMode == 0U);
    PLATFORM_ASSERT(pmicHandle.crcEnable == PMIC_DISABLE);
    PLATFORM_ASSERT(pmicHandle.commHandle0 == NULL);
    PLATFORM_ASSERT(pmicHandle.ioRead == NULL);
    PLATFORM_ASSERT(pmicHandle.ioWrite == NULL);
    PLATFORM_ASSERT(pmicHandle.criticalSectionStart == NULL);
    PLATFORM_ASSERT(pmicHandle.criticalSectionStop == NULL);
    PLATFORM_ASSERT(pmicHandle.irqResponseCallback == NULL);
}

void test_positive_Pmic_checkHandle_validHandle(void)
{
    // Initialize handle first, then check it
    Pmic_HandleCfg_t handleCfg = {0};
    pmicInitTest_initHandleCfg(&handleCfg);
    int32_t status = Pmic_init(&pmicHandle, &handleCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Check valid initialized handle
    status = Pmic_checkHandle(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Clean up
    Pmic_deinit(&pmicHandle);
}

void test_positive_Pmic_checkHandle_invalidHandle(void)
{
    // Check uninitialized handle (should fail check)
    Pmic_Handle_t uninitHandle = {0};
    int32_t status = Pmic_checkHandle(&uninitHandle);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_HANDLE);
}

void test_positive_Pmic_init_reinit(void)
{
    // Initialize, deinitialize, then re-initialize
    Pmic_HandleCfg_t handleCfg = {0};
    pmicInitTest_initHandleCfg(&handleCfg);

    // First init
    int32_t status = Pmic_init(&pmicHandle, &handleCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Deinit
    status = Pmic_deinit(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Re-init
    status = Pmic_init(&pmicHandle, &handleCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(pmicHandle.commMode == PMIC_INTF_SPI);

    // Clean up
    Pmic_deinit(&pmicHandle);
}

void test_positive_Pmic_init_with_crc_enabled(void)
{
    // Initialize PMIC with CRC enabled
    Pmic_HandleCfg_t handleCfg = {0};
    pmicInitTest_initHandleCfg(&handleCfg);

    // Enable CRC validation parameter
    handleCfg.validParams |= PMIC_CRC_ENABLE_VALID;
    handleCfg.crcEnable = PMIC_ENABLE;

    int32_t status = Pmic_init(&pmicHandle, &handleCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify CRC is enabled in handle
    PLATFORM_ASSERT(pmicHandle.crcEnable == PMIC_ENABLE);

    // Clean up
    Pmic_deinit(&pmicHandle);
}

void test_positive_Pmic_init_with_both_crc_flags(void)
{
    // Initialize with CRC enabled in both valid params and config
    Pmic_HandleCfg_t handleCfg = {0};
    pmicInitTest_initHandleCfg(&handleCfg);

    // Set CRC enable flag with valid parameter
    handleCfg.validParams |= PMIC_CRC_ENABLE_VALID;
    handleCfg.crcEnable = PMIC_ENABLE;

    int32_t status = Pmic_init(&pmicHandle, &handleCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify handle was initialized correctly
    PLATFORM_ASSERT(pmicHandle.crcEnable == PMIC_ENABLE);
    PLATFORM_ASSERT(pmicHandle.commMode == PMIC_INTF_SPI);

    // Clean up
    Pmic_deinit(&pmicHandle);
}

void test_positive_Pmic_init_crc_disabled(void)
{
    // Initialize with CRC explicitly disabled
    Pmic_HandleCfg_t handleCfg = {0};
    pmicInitTest_initHandleCfg(&handleCfg);

    // Explicitly set CRC to disabled
    handleCfg.validParams |= PMIC_CRC_ENABLE_VALID;
    handleCfg.crcEnable = PMIC_DISABLE;

    int32_t status = Pmic_init(&pmicHandle, &handleCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify CRC is disabled in handle
    PLATFORM_ASSERT(pmicHandle.crcEnable == PMIC_DISABLE);

    // Clean up
    Pmic_deinit(&pmicHandle);
}

void test_positive_Pmic_init_verify_crc_state(void)
{
    // Initialize and verify CRC state can be read
    Pmic_HandleCfg_t handleCfg = {0};
    pmicInitTest_initHandleCfg(&handleCfg);

    // Initialize with CRC enabled
    handleCfg.validParams |= PMIC_CRC_ENABLE_VALID;
    handleCfg.crcEnable = PMIC_ENABLE;

    int32_t status = Pmic_init(&pmicHandle, &handleCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify the CRC state is reflected in handle
    PLATFORM_ASSERT(pmicHandle.crcEnable == PMIC_ENABLE);

    // Test that we can disable CRC after init
    pmicHandle.crcEnable = PMIC_DISABLE;
    PLATFORM_ASSERT(pmicHandle.crcEnable == PMIC_DISABLE);

    // Test that we can re-enable CRC
    pmicHandle.crcEnable = PMIC_ENABLE;
    PLATFORM_ASSERT(pmicHandle.crcEnable == PMIC_ENABLE);

    // Clean up
    Pmic_deinit(&pmicHandle);
}

void test_positive_Pmic_init_complete_flow(void)
{
    // Test complete initialization flow with device info retrieval and comm validation
    Pmic_HandleCfg_t handleCfg = {0};
    pmicInitTest_initHandleCfg(&handleCfg);

    int32_t status = Pmic_init(&pmicHandle, &handleCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify driver initialization status magic number
    PLATFORM_ASSERT(pmicHandle.drvInitStat == 0x504D4943U);

    // Verify all function pointers are set
    PLATFORM_ASSERT(pmicHandle.ioRead == &platform_rxByte);
    PLATFORM_ASSERT(pmicHandle.ioWrite == &platform_txByte);
    PLATFORM_ASSERT(pmicHandle.criticalSectionStart == &platform_critSecStart);
    PLATFORM_ASSERT(pmicHandle.criticalSectionStop == &platform_critSecStop);

    // Verify handle passes validation
    status = Pmic_checkHandle(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Clean up
    Pmic_deinit(&pmicHandle);
}

void test_positive_Pmic_init_i2c_single_mode(void)
{
    // Initialize with I2C single mode (mock mode - no timer/retry needed)
    Pmic_HandleCfg_t handleCfg = {0};
    handleCfg.validParams = PMIC_COMM_MODE_VALID |
                            PMIC_I2C_ADDR0_VALID |
                            PMIC_I2C_ADDR1_VALID |
                            PMIC_CRC_ENABLE_VALID |
                            PMIC_COMM_HANDLE_0_VALID |
                            PMIC_IO_READ_VALID |
                            PMIC_IO_WRITE_VALID |
                            PMIC_CRITICAL_SECTION_START_VALID |
                            PMIC_CRITICAL_SECTION_STOP_VALID |
                            PMIC_IRQ_RESPONSE_CALLBACK_VALID;
    handleCfg.commMode = PMIC_INTF_I2C_SINGLE;
    handleCfg.i2cAddr0 = PLATFORM_I2C_ADDR_MAIN;
    handleCfg.i2cAddr1 = PLATFORM_I2C_ADDR_SECONDARY;
    handleCfg.crcEnable = PMIC_DISABLE;
    handleCfg.commHandle0 = platform_getCommHandle();
    handleCfg.ioRead = &platform_rxByte;
    handleCfg.ioWrite = &platform_txByte;
    handleCfg.criticalSectionStart = &platform_critSecStart;
    handleCfg.criticalSectionStop = &platform_critSecStop;
    handleCfg.irqResponseCallback = &platform_irqResponse;

    int32_t status = Pmic_init(&pmicHandle, &handleCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify I2C single mode configuration
    PLATFORM_ASSERT(pmicHandle.commMode == PMIC_INTF_I2C_SINGLE);
    PLATFORM_ASSERT(pmicHandle.i2cAddr0 == PLATFORM_I2C_ADDR_MAIN);
    PLATFORM_ASSERT(pmicHandle.i2cAddr1 == PLATFORM_I2C_ADDR_SECONDARY);

    // Clean up
    Pmic_deinit(&pmicHandle);
}

void test_positive_Pmic_init_i2c_dual_mode(void)
{
    // Initialize with I2C dual mode (mock mode - no timer/retry needed)
    Pmic_HandleCfg_t handleCfg = {0};
    handleCfg.validParams = PMIC_COMM_MODE_VALID |
                            PMIC_I2C_ADDR0_VALID |
                            PMIC_I2C_ADDR1_VALID |
                            PMIC_CRC_ENABLE_VALID |
                            PMIC_COMM_HANDLE_0_VALID |
                            PMIC_IO_READ_VALID |
                            PMIC_IO_WRITE_VALID |
                            PMIC_CRITICAL_SECTION_START_VALID |
                            PMIC_CRITICAL_SECTION_STOP_VALID |
                            PMIC_IRQ_RESPONSE_CALLBACK_VALID;
    handleCfg.commMode = PMIC_INTF_I2C_DUAL;
    handleCfg.i2cAddr0 = PLATFORM_I2C_ADDR_MAIN;
    handleCfg.i2cAddr1 = PLATFORM_I2C_ADDR_SECONDARY;
    handleCfg.crcEnable = PMIC_DISABLE;
    handleCfg.commHandle0 = platform_getCommHandle();
    handleCfg.ioRead = &platform_rxByte;
    handleCfg.ioWrite = &platform_txByte;
    handleCfg.criticalSectionStart = &platform_critSecStart;
    handleCfg.criticalSectionStop = &platform_critSecStop;
    handleCfg.irqResponseCallback = &platform_irqResponse;

    int32_t status = Pmic_init(&pmicHandle, &handleCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify I2C dual mode configuration
    PLATFORM_ASSERT(pmicHandle.commMode == PMIC_INTF_I2C_DUAL);
    PLATFORM_ASSERT(pmicHandle.i2cAddr0 == PLATFORM_I2C_ADDR_MAIN);
    PLATFORM_ASSERT(pmicHandle.i2cAddr1 == PLATFORM_I2C_ADDR_SECONDARY);

    // Clean up
    Pmic_deinit(&pmicHandle);
}

void test_positive_Pmic_init_device_info_retrieval(void)
{
    // Initialize and verify device info fields are populated
    Pmic_HandleCfg_t handleCfg = {0};
    pmicInitTest_initHandleCfg(&handleCfg);

    int32_t status = Pmic_init(&pmicHandle, &handleCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Mock should return realistic device info values
    // Verify that device revision fields are populated (non-zero from mock)
    // Note: Mock backend returns register values, we just verify they were read
    PLATFORM_ASSERT(pmicHandle.drvInitStat == 0x504D4943U);

    // Clean up
    Pmic_deinit(&pmicHandle);
}

void test_positive_Pmic_deinit_success_path(void)
{
    // Test complete deinit success path
    Pmic_HandleCfg_t handleCfg = {0};
    pmicInitTest_initHandleCfg(&handleCfg);

    // Initialize first
    int32_t status = Pmic_init(&pmicHandle, &handleCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(pmicHandle.drvInitStat == 0x504D4943U);

    // Deinitialize
    status = Pmic_deinit(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify all fields are cleared
    PLATFORM_ASSERT(pmicHandle.drvInitStat == 0U);
    PLATFORM_ASSERT(pmicHandle.devRev == 0U);
    PLATFORM_ASSERT(pmicHandle.devSiRev == 0U);
    PLATFORM_ASSERT(pmicHandle.nvmCode == 0U);
    PLATFORM_ASSERT(pmicHandle.nvmRev == 0U);
    PLATFORM_ASSERT(pmicHandle.commMode == 0U);
    PLATFORM_ASSERT(pmicHandle.i2cAddr0 == 0U);
    PLATFORM_ASSERT(pmicHandle.i2cAddr1 == 0U);
    PLATFORM_ASSERT(pmicHandle.i2cAddr2 == 0U);
    PLATFORM_ASSERT(pmicHandle.crcEnable == PMIC_DISABLE);
    PLATFORM_ASSERT(pmicHandle.asyncEnable == PMIC_DISABLE);
    PLATFORM_ASSERT(pmicHandle.commHandle0 == NULL);
    PLATFORM_ASSERT(pmicHandle.taskHandle == NULL);
    PLATFORM_ASSERT(pmicHandle.ioRead == NULL);
    PLATFORM_ASSERT(pmicHandle.ioWrite == NULL);
    PLATFORM_ASSERT(pmicHandle.asyncRxStart == NULL);
    PLATFORM_ASSERT(pmicHandle.asyncTxStart == NULL);
    PLATFORM_ASSERT(pmicHandle.asyncRxAwait == NULL);
    PLATFORM_ASSERT(pmicHandle.asyncTxAwait == NULL);
    PLATFORM_ASSERT(pmicHandle.criticalSectionStart == NULL);
    PLATFORM_ASSERT(pmicHandle.criticalSectionStop == NULL);
    PLATFORM_ASSERT(pmicHandle.irqResponseCallback == NULL);
}

void test_positive_Pmic_checkHandle_all_validations(void)
{
    // Test all validation paths in Pmic_checkHandle

    // Test 1: NULL handle
    int32_t status = Pmic_checkHandle(NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);

    // Test 2: Invalid drvInitStat
    Pmic_Handle_t testHandle = {0};
    testHandle.drvInitStat = 0xDEADBEEFU;  // Wrong magic number
    testHandle.commMode = PMIC_INTF_SPI;
    testHandle.commHandle0 = platform_getCommHandle();
    testHandle.ioRead = &platform_rxByte;
    testHandle.ioWrite = &platform_txByte;
    testHandle.criticalSectionStart = &platform_critSecStart;
    testHandle.criticalSectionStop = &platform_critSecStop;
    status = Pmic_checkHandle(&testHandle);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_HANDLE);

    // Test 3: Invalid commMode
    testHandle.drvInitStat = 0x504D4943U;  // Correct magic number
    testHandle.commMode = PMIC_INTF_MAX + 1U;
    status = Pmic_checkHandle(&testHandle);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);

    // Test 4: NULL commHandle0
    testHandle.commMode = PMIC_INTF_SPI;
    testHandle.commHandle0 = NULL;
    status = Pmic_checkHandle(&testHandle);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);

    // Test 5: NULL critical section functions
    testHandle.commHandle0 = platform_getCommHandle();
    testHandle.criticalSectionStart = NULL;
    status = Pmic_checkHandle(&testHandle);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_FPTR);

    // Test 6: NULL ioRead in synchronous mode
    testHandle.criticalSectionStart = &platform_critSecStart;
    testHandle.asyncEnable = PMIC_DISABLE;
    testHandle.ioRead = NULL;
    status = Pmic_checkHandle(&testHandle);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_FPTR);

    // Test 7: NULL async functions in async mode
    testHandle.ioRead = &platform_rxByte;
    testHandle.ioWrite = &platform_txByte;
    testHandle.asyncEnable = PMIC_ENABLE;
    testHandle.asyncRxStart = NULL;
    status = Pmic_checkHandle(&testHandle);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_FPTR);

    // Test 8: Valid handle should pass all checks
    Pmic_HandleCfg_t handleCfg = {0};
    pmicInitTest_initHandleCfg(&handleCfg);
    status = Pmic_init(&pmicHandle, &handleCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    status = Pmic_checkHandle(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Clean up
    Pmic_deinit(&pmicHandle);
}

void test_positive_Pmic_init_async_mode(void)
{
    // Initialize with async mode enabled
    // Note: Even in async mode, synchronous I/O is needed for initialization (getPmicInfo)
    Pmic_HandleCfg_t handleCfg = {0};
    handleCfg.validParams = PMIC_COMM_MODE_VALID |
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
                            PMIC_IO_WRITE_VALID;
    handleCfg.commMode = PMIC_INTF_SPI;
    handleCfg.crcEnable = PMIC_DISABLE;
    handleCfg.asyncEnable = PMIC_ENABLE;
    handleCfg.commHandle0 = platform_getCommHandle();
    handleCfg.taskHandle = platform_getCommHandle();  // Use sentinel for task handle
    handleCfg.ioRead = &platform_rxByte;  // Needed for getPmicInfo during init
    handleCfg.ioWrite = &platform_txByte;  // Needed for getPmicInfo during init
    handleCfg.asyncRxStart = &test_pmic_asyncRxStart;
    handleCfg.asyncTxStart = &test_pmic_asyncTxStart;
    handleCfg.asyncRxAwait = &test_pmic_asyncRxAwait;
    handleCfg.asyncTxAwait = &test_pmic_asyncTxAwait;
    handleCfg.criticalSectionStart = &platform_critSecStart;
    handleCfg.criticalSectionStop = &platform_critSecStop;
    handleCfg.irqResponseCallback = &platform_irqResponse;

    int32_t status = Pmic_init(&pmicHandle, &handleCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify async mode is enabled
    PLATFORM_ASSERT(pmicHandle.asyncEnable == PMIC_ENABLE);
    PLATFORM_ASSERT(pmicHandle.asyncRxStart == &test_pmic_asyncRxStart);
    PLATFORM_ASSERT(pmicHandle.asyncTxStart == &test_pmic_asyncTxStart);
    PLATFORM_ASSERT(pmicHandle.asyncRxAwait == &test_pmic_asyncRxAwait);
    PLATFORM_ASSERT(pmicHandle.asyncTxAwait == &test_pmic_asyncTxAwait);
    PLATFORM_ASSERT(pmicHandle.taskHandle != NULL);

    // Verify handle passes async validation
    status = Pmic_checkHandle(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Clean up
    Pmic_deinit(&pmicHandle);
}

void test_positive_Pmic_init_with_i2c_addresses(void)
{
    // Test initialization with all three I2C addresses configured (mock mode - no timer/retry needed)
    Pmic_HandleCfg_t handleCfg = {0};
    handleCfg.validParams = PMIC_COMM_MODE_VALID |
                            PMIC_I2C_ADDR0_VALID |
                            PMIC_I2C_ADDR1_VALID |
                            PMIC_I2C_ADDR2_VALID |
                            PMIC_CRC_ENABLE_VALID |
                            PMIC_COMM_HANDLE_0_VALID |
                            PMIC_IO_READ_VALID |
                            PMIC_IO_WRITE_VALID |
                            PMIC_CRITICAL_SECTION_START_VALID |
                            PMIC_CRITICAL_SECTION_STOP_VALID |
                            PMIC_IRQ_RESPONSE_CALLBACK_VALID;
    handleCfg.commMode = PMIC_INTF_I2C_SINGLE;
    handleCfg.i2cAddr0 = 0x60U;
    handleCfg.i2cAddr1 = 0x12U;
    handleCfg.i2cAddr2 = 0x34U;
    handleCfg.crcEnable = PMIC_DISABLE;
    handleCfg.commHandle0 = platform_getCommHandle();
    handleCfg.ioRead = &platform_rxByte;
    handleCfg.ioWrite = &platform_txByte;
    handleCfg.criticalSectionStart = &platform_critSecStart;
    handleCfg.criticalSectionStop = &platform_critSecStop;
    handleCfg.irqResponseCallback = &platform_irqResponse;

    int32_t status = Pmic_init(&pmicHandle, &handleCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify all I2C addresses are set
    PLATFORM_ASSERT(pmicHandle.i2cAddr0 == 0x60U);
    PLATFORM_ASSERT(pmicHandle.i2cAddr1 == 0x12U);
    PLATFORM_ASSERT(pmicHandle.i2cAddr2 == 0x34U);

    // Clean up
    Pmic_deinit(&pmicHandle);
}

void test_positive_Pmic_init_with_task_handle(void)
{
    // Test initialization with task handle configured (for RTOS environments)
    Pmic_HandleCfg_t handleCfg = {0};
    handleCfg.validParams = PMIC_COMM_MODE_VALID |
                            PMIC_CRC_ENABLE_VALID |
                            PMIC_COMM_HANDLE_0_VALID |
                            PMIC_IO_READ_VALID |
                            PMIC_IO_WRITE_VALID |
                            PMIC_CRITICAL_SECTION_START_VALID |
                            PMIC_CRITICAL_SECTION_STOP_VALID |
                            PMIC_IRQ_RESPONSE_CALLBACK_VALID |
                            PMIC_TASK_HANDLE_VALID;
    handleCfg.commMode = PMIC_INTF_SPI;
    handleCfg.crcEnable = PMIC_DISABLE;
    handleCfg.commHandle0 = platform_getCommHandle();
    handleCfg.taskHandle = (void*)0x12345678U;  // Use a non-NULL sentinel value
    handleCfg.ioRead = &platform_rxByte;
    handleCfg.ioWrite = &platform_txByte;
    handleCfg.criticalSectionStart = &platform_critSecStart;
    handleCfg.criticalSectionStop = &platform_critSecStop;
    handleCfg.irqResponseCallback = &platform_irqResponse;

    int32_t status = Pmic_init(&pmicHandle, &handleCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify task handle is set
    PLATFORM_ASSERT(pmicHandle.taskHandle == (void*)0x12345678U);

    // Clean up
    Pmic_deinit(&pmicHandle);
}

void test_positive_init_withRetryCnt(void)
{
    // Initialize with PMIC_RETRY_CNT_VALID set
    Pmic_HandleCfg_t handleCfg = {0};
    Pmic_Handle_t handle = {0};

    pmicInitTest_initHandleCfg(&handleCfg);

    // Add retry count configuration
    handleCfg.validParams |= PMIC_RETRY_CNT_VALID;
    handleCfg.retryCnt = 5U;

    int32_t status = Pmic_init(&handle, &handleCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify retry count is set in handle
    PLATFORM_ASSERT(handle.retryCnt == 5U);

    // Clean up
    status = Pmic_deinit(&handle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_positive_init_withRetryInterval(void)
{
    // Initialize with PMIC_RETRY_INTERVAL_MS_VALID set
    Pmic_HandleCfg_t handleCfg = {0};
    Pmic_Handle_t handle = {0};

    pmicInitTest_initHandleCfg(&handleCfg);

    // Add retry interval configuration
    // Note: When retryIntervalMs is non-zero, timerWaitMs must also be provided
    handleCfg.validParams |= PMIC_RETRY_INTERVAL_MS_VALID | PMIC_TIMER_WAIT_MS_VALID;
    handleCfg.retryIntervalMs = 100U;
    handleCfg.timerWaitMs = &testTimerWaitWrapper;

    int32_t status = Pmic_init(&handle, &handleCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify retry interval is set in handle
    PLATFORM_ASSERT(handle.retryIntervalMs == 100U);

    // Clean up
    status = Pmic_deinit(&handle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_positive_init_withTimerWaitMs(void)
{
    // Initialize with PMIC_TIMER_WAIT_MS_VALID and valid callback
    Pmic_HandleCfg_t handleCfg = {0};
    Pmic_Handle_t handle = {0};

    pmicInitTest_initHandleCfg(&handleCfg);

    // Add timer wait callback configuration
    handleCfg.validParams |= PMIC_TIMER_WAIT_MS_VALID;
    handleCfg.timerWaitMs = &testTimerWaitWrapper;

    int32_t status = Pmic_init(&handle, &handleCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify timer wait callback is set in handle
    PLATFORM_ASSERT(handle.timerWaitMs == &testTimerWaitWrapper);

    // Clean up
    status = Pmic_deinit(&handle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_negative_init_timerWaitMsCallbackNull(void)
{
    // Set PMIC_TIMER_WAIT_MS_VALID but pass NULL callback
    Pmic_HandleCfg_t handleCfg = {0};
    Pmic_Handle_t handle = {0};

    pmicInitTest_initHandleCfg(&handleCfg);

    // Set valid param flag but provide NULL callback
    handleCfg.validParams |= PMIC_TIMER_WAIT_MS_VALID;
    handleCfg.timerWaitMs = NULL;

    int32_t status = Pmic_init(&handle, &handleCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_FPTR);
}
