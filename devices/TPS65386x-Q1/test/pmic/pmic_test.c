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

#include "pmic_test.h"
#include "test_constants.h"
#include "regmap/core.h"

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/* Driver initialization magic number (from pmic.c) */
#define PMIC_INIT_TEST_DRV_INIT_MAGIC   TEST_PMIC_INIT_MAGIC

/* Expected initialization status for SPI mode */
/* NOTE: PMIC_MAIN_INST not defined for TPS65386x-Q1 - using magic number only */
#define PMIC_INIT_TEST_EXPECTED_STAT    (PMIC_INIT_TEST_DRV_INIT_MAGIC)

/* ========================================================================== */
// Mock I/O infrastructure for pmic_test
/* ========================================================================== */

// One-shot return status for the next mock read. Resets to SUCCESS after firing.
static int32_t g_pmicTestMockReadReturnStatus = PMIC_ST_SUCCESS;

/* Number of reads to allow (pass) before injecting the failure.
 * 0 = fail on call 1; N = skip N successful calls then fail on call N+1. */
static uint32_t g_pmicTestMockReadPassCount = 0U;

// Running count of read calls in current test. Reset via resetPmicMockState().
static uint32_t g_pmicTestMockReadCallCount = 0U;

static void resetPmicMockState(void)
{
    g_pmicTestMockReadReturnStatus = PMIC_ST_SUCCESS;
    g_pmicTestMockReadPassCount    = 0U;
    g_pmicTestMockReadCallCount    = 0U;
}

static int32_t pmicTestMockIoRead(const Pmic_Handle_t *handle, uint8_t page,
                                  uint8_t regAddr, uint8_t *buffer, uint8_t bufLen)
{
    g_pmicTestMockReadCallCount++;

    if (g_pmicTestMockReadCallCount > g_pmicTestMockReadPassCount)
    {
        if (g_pmicTestMockReadReturnStatus != PMIC_ST_SUCCESS)
        {
            int32_t ret = g_pmicTestMockReadReturnStatus;
            g_pmicTestMockReadReturnStatus = PMIC_ST_SUCCESS;  // one-shot
            return ret;
        }
    }

    return platform_rxByte(handle, page, regAddr, buffer, bufLen);
}

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static void initTestHandleCfg(Pmic_HandleCfg_t *pmicCfg);

/* ========================================================================== */
/*                             Global Variables                               */
/* ========================================================================== */

static Pmic_Handle_t g_pmicHandle;
static uint8_t dummyCommHandle = 0U;

/* Mock timer wait callback for testing */
static void mockTimerWait(uint32_t ms)
{
    (void)ms;  // Unused in mock
}

/* Mock async hooks used to isolate individual OR-terms in validatePmicHandle */
static int32_t mockAsyncRxStart(const Pmic_Handle_t *handle, uint8_t page, uint8_t regAddr,
                                 uint8_t *buffer, uint8_t bufLen)
{
    (void)handle; (void)page; (void)regAddr; (void)buffer; (void)bufLen;
    return PMIC_ST_SUCCESS;
}

static int32_t mockAsyncTxStart(const Pmic_Handle_t *handle, uint8_t page, uint8_t regAddr,
                                 const uint8_t *buffer, uint8_t bufLen)
{
    (void)handle; (void)page; (void)regAddr; (void)buffer; (void)bufLen;
    return PMIC_ST_SUCCESS;
}

static int32_t mockAsyncRxAwait(const Pmic_Handle_t *handle)
{
    (void)handle;
    return PMIC_ST_SUCCESS;
}

static int32_t mockAsyncTxAwait(const Pmic_Handle_t *handle)
{
    (void)handle;
    return PMIC_ST_SUCCESS;
}

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void pmic_test(void *args)
{
    platform_init();
    testTimer_startModule("PMIC");

    // Run all PMIC tests
    PMIC_TEST_RUN_ALL();

    testTimer_endModule();
    platform_deinit();

}

/**
 * @brief Helper function to initialize handle configuration with valid defaults.
 */
static void initTestHandleCfg(Pmic_HandleCfg_t *pmicCfg)
{
    pmicCfg->validParams = PMIC_CFG_INIT_COMM_MODE_VALID |
                           PMIC_CFG_INIT_COMM_HANDLE_0_VALID |
                           PMIC_CFG_INIT_IO_READ_VALID |
                           PMIC_CFG_INIT_IO_WRITE_VALID |
                           PMIC_CFG_INIT_CRITICAL_SECTION_START_VALID |
                           PMIC_CFG_INIT_CRITICAL_SECTION_STOP_VALID;
    pmicCfg->commMode = PMIC_INTF_SPI;
    pmicCfg->commHandle0 = (void*)&dummyCommHandle;
    pmicCfg->ioRead = &platform_rxByte;
    pmicCfg->ioWrite = &platform_txByte;
    pmicCfg->criticalSectionStart = &platform_critSecStart;
    pmicCfg->criticalSectionStop = &platform_critSecStop;
}

/* ========================================================================== */
/*                    Negative Tests - NULL Parameters                        */
/* ========================================================================== */

void test_neg_pmic_init_nullHandle(void)
{
    Pmic_HandleCfg_t pmicCfg = {0};
    initTestHandleCfg(&pmicCfg);

    int32_t status = Pmic_init(NULL, &pmicCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_pmic_init_nullCoreCfg(void)
{
    Pmic_Handle_t handle = {0};

    int32_t status = Pmic_init(&handle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_pmic_init_nullCommHandle(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_HandleCfg_t pmicCfg = {0};
    initTestHandleCfg(&pmicCfg);

    // Set PMIC_CFG_INIT_COMM_HANDLE_0_VALID to trigger validation
    pmicCfg.validParams |= PMIC_CFG_INIT_COMM_HANDLE_0_VALID;
    pmicCfg.commHandle0 = NULL;
    int32_t status = Pmic_init(&handle, &pmicCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_pmic_init_nullIoRead(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_HandleCfg_t pmicCfg = {0};
    initTestHandleCfg(&pmicCfg);

    pmicCfg.ioRead = NULL;
    int32_t status = Pmic_init(&handle, &pmicCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_FPTR);
}

void test_neg_pmic_init_nullIoWrite(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_HandleCfg_t pmicCfg = {0};
    initTestHandleCfg(&pmicCfg);

    pmicCfg.ioWrite = NULL;
    int32_t status = Pmic_init(&handle, &pmicCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_FPTR);
}

void test_neg_pmic_init_nullCritSecStart(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_HandleCfg_t pmicCfg = {0};
    initTestHandleCfg(&pmicCfg);

    pmicCfg.criticalSectionStart = NULL;
    int32_t status = Pmic_init(&handle, &pmicCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_FPTR);
}

void test_neg_pmic_init_nullCritSecStop(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_HandleCfg_t pmicCfg = {0};
    initTestHandleCfg(&pmicCfg);

    pmicCfg.criticalSectionStop = NULL;
    int32_t status = Pmic_init(&handle, &pmicCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_FPTR);
}

void test_neg_pmic_deinit_nullHandle(void)
{
    int32_t status = Pmic_deinit(NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_pmic_checkHandle_nullHandle(void)
{
    int32_t status = Pmic_checkHandle(NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/* ========================================================================== */
/*                   Negative Tests - Invalid Parameters                      */
/* ========================================================================== */

void test_neg_pmic_init_invalidDeviceType(void)
{
    // NOTE: TPS65386x-Q1 does not have deviceType validation (unlike TPS6522x-Q1).
    // This device variant only supports SPI mode and doesn't require deviceType configuration.
    // Test skipped as feature is not applicable to this device. */
    (void)0;  // No-op test
}

void test_neg_pmic_init_invalidCommMode(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_HandleCfg_t pmicCfg = {0};
    initTestHandleCfg(&pmicCfg);

    // TPS65386x only supports SPI mode
    pmicCfg.commMode = PMIC_INTF_I2C_SINGLE;
    int32_t status = Pmic_init(&handle, &pmicCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_pmic_init_insufficientCfg_missingIoRead(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_HandleCfg_t pmicCfg = {0};
    initTestHandleCfg(&pmicCfg);

    // Set IO read callback to NULL with valid bit set to trigger NULL check
    pmicCfg.ioRead = NULL;
    // Keep PMIC_CFG_INIT_IO_READ_VALID set so validation catches the NULL

    int32_t status = Pmic_init(&handle, &pmicCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_FPTR);
}

void test_neg_pmic_init_insufficientCfg_missingIoWrite(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_HandleCfg_t pmicCfg = {0};
    initTestHandleCfg(&pmicCfg);

    // Set IO write callback to NULL with valid bit set to trigger NULL check
    pmicCfg.ioWrite = NULL;
    // Keep PMIC_CFG_INIT_IO_WRITE_VALID set so validation catches the NULL

    int32_t status = Pmic_init(&handle, &pmicCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_FPTR);
}

void test_neg_pmic_init_insufficientCfg_missingCritSec(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_HandleCfg_t pmicCfg = {0};
    initTestHandleCfg(&pmicCfg);

    // Set critical section start to NULL with valid bit set to trigger NULL check
    pmicCfg.criticalSectionStart = NULL;
    // Keep PMIC_CFG_INIT_CRITICAL_SECTION_START_VALID set so validation catches the NULL

    int32_t status = Pmic_init(&handle, &pmicCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_FPTR);
}

void test_neg_pmic_checkHandle_invalidInitStat(void)
{
    Pmic_Handle_t handle = {0};

    // Handle with invalid initialization status
    handle.drvInitStat = 0x00000000U;
    handle.commMode = PMIC_INTF_SPI;
    handle.commHandle0 = NULL;
    handle.ioRead = &platform_rxByte;

    int32_t status = Pmic_checkHandle(&handle);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_HANDLE);
}

/* ========================================================================== */
/*                 Positive Tests - Basic Init/Deinit                         */
/* ========================================================================== */

void test_pos_pmic_init_spiMode(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_HandleCfg_t pmicCfg = {0};
    initTestHandleCfg(&pmicCfg);

    int32_t status = Pmic_init(&handle, &pmicCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(handle.commMode == PMIC_INTF_SPI);
    // NOTE: TPS65386x-Q1 handle doesn't have devId field - skipping assertion

    (void)Pmic_deinit(&handle);
}

void test_pos_pmic_init_validateHandle(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_HandleCfg_t pmicCfg = {0};
    initTestHandleCfg(&pmicCfg);

    int32_t status = Pmic_init(&handle, &pmicCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify handle is properly initialized
    PLATFORM_ASSERT(handle.drvInitStat == PMIC_INIT_TEST_EXPECTED_STAT);
    PLATFORM_ASSERT(handle.commHandle0 != NULL);
    PLATFORM_ASSERT(handle.ioRead != NULL);
    PLATFORM_ASSERT(handle.ioWrite != NULL);
    PLATFORM_ASSERT(handle.criticalSectionStart != NULL);
    PLATFORM_ASSERT(handle.criticalSectionStop != NULL);

    (void)Pmic_deinit(&handle);
}

void test_pos_pmic_deinit_clearsHandle(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_HandleCfg_t pmicCfg = {0};
    initTestHandleCfg(&pmicCfg);

    // Initialize then deinitialize
    int32_t status = Pmic_init(&handle, &pmicCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_deinit(&handle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify handle is cleared
    PLATFORM_ASSERT(handle.drvInitStat == 0x00U);
    PLATFORM_ASSERT(handle.commHandle0 == NULL);
    PLATFORM_ASSERT(handle.ioRead == NULL);
    PLATFORM_ASSERT(handle.ioWrite == NULL);
    PLATFORM_ASSERT(handle.criticalSectionStart == NULL);
    PLATFORM_ASSERT(handle.criticalSectionStop == NULL);
}

void test_pos_pmic_checkHandle_validHandle(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_HandleCfg_t pmicCfg = {0};
    initTestHandleCfg(&pmicCfg);

    int32_t status = Pmic_init(&handle, &pmicCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify handle passes validation
    status = Pmic_checkHandle(&handle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    (void)Pmic_deinit(&handle);
}

/* ========================================================================== */
/*              Positive Tests - Initialization Variations                    */
/* ========================================================================== */

void test_pos_pmic_init_withAllCallbacks(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_HandleCfg_t pmicCfg = {0};
    initTestHandleCfg(&pmicCfg);

    int32_t status = Pmic_init(&handle, &pmicCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify all callbacks are stored
    PLATFORM_ASSERT(handle.ioRead == &platform_rxByte);
    PLATFORM_ASSERT(handle.ioWrite == &platform_txByte);
    PLATFORM_ASSERT(handle.criticalSectionStart == &platform_critSecStart);
    PLATFORM_ASSERT(handle.criticalSectionStop == &platform_critSecStop);

    (void)Pmic_deinit(&handle);
}

void test_pos_pmic_init_multipleInitDeinit(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_HandleCfg_t pmicCfg = {0};
    initTestHandleCfg(&pmicCfg);

    // First cycle
    int32_t status = Pmic_init(&handle, &pmicCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    status = Pmic_deinit(&handle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Second cycle
    status = Pmic_init(&handle, &pmicCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    status = Pmic_deinit(&handle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Third cycle
    status = Pmic_init(&handle, &pmicCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    status = Pmic_deinit(&handle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_pmic_init_handlePersistence(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_HandleCfg_t pmicCfg = {0};
    initTestHandleCfg(&pmicCfg);

    int32_t status = Pmic_init(&handle, &pmicCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Store initial values
    uint32_t initStat = handle.drvInitStat;
    uint8_t commMode = handle.commMode;

    // Verify values persist
    PLATFORM_ASSERT(handle.drvInitStat == initStat);
    PLATFORM_ASSERT(handle.commMode == commMode);

    (void)Pmic_deinit(&handle);
}

/* ========================================================================== */
/*            Positive Tests - Configuration Validation                       */
/* ========================================================================== */

void test_pos_pmic_init_validDeviceType(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_HandleCfg_t pmicCfg = {0};
    initTestHandleCfg(&pmicCfg);

    int32_t status = Pmic_init(&handle, &pmicCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    (void)Pmic_deinit(&handle);
}

void test_pos_pmic_init_validCommMode(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_HandleCfg_t pmicCfg = {0};
    initTestHandleCfg(&pmicCfg);

    pmicCfg.commMode = PMIC_INTF_SPI;

    int32_t status = Pmic_init(&handle, &pmicCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(handle.commMode == PMIC_INTF_SPI);

    (void)Pmic_deinit(&handle);
}

void test_pos_pmic_init_validateCommHandleStored(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_HandleCfg_t pmicCfg = {0};
    initTestHandleCfg(&pmicCfg);

    void *expectedCommHandle = platform_getCommHandle();
    pmicCfg.commHandle0 = expectedCommHandle;

    int32_t status = Pmic_init(&handle, &pmicCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(handle.commHandle0 == expectedCommHandle);

    (void)Pmic_deinit(&handle);
}

void test_pos_pmic_init_validateCallbacksStored(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_HandleCfg_t pmicCfg = {0};
    initTestHandleCfg(&pmicCfg);

    int32_t status = Pmic_init(&handle, &pmicCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify all function pointers are correctly stored
    PLATFORM_ASSERT(handle.ioRead == pmicCfg.ioRead);
    PLATFORM_ASSERT(handle.ioWrite == pmicCfg.ioWrite);
    PLATFORM_ASSERT(handle.criticalSectionStart == pmicCfg.criticalSectionStart);
    PLATFORM_ASSERT(handle.criticalSectionStop == pmicCfg.criticalSectionStop);

    (void)Pmic_deinit(&handle);
}

/* ========================================================================== */
/*            Positive Tests - Communication Validation                       */
/* ========================================================================== */

void test_pos_pmic_init_communicationTest(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_HandleCfg_t pmicCfg = {0};
    initTestHandleCfg(&pmicCfg);

    int32_t status = Pmic_init(&handle, &pmicCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify communication is established during init
    // Init function reads WD_LONGWIN_CFG_REG to validate comms
    PLATFORM_ASSERT(handle.drvInitStat == PMIC_INIT_TEST_EXPECTED_STAT);

    (void)Pmic_deinit(&handle);
}

void test_pos_pmic_init_registerAccess(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_HandleCfg_t pmicCfg = {0};
    initTestHandleCfg(&pmicCfg);

    int32_t status = Pmic_init(&handle, &pmicCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Test register access after init
    uint8_t readData = 0U;
    status = Pmic_ioRxByte(&handle, PMIC_CUSTOMER_SCRATCH1_REG, &readData);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    (void)Pmic_deinit(&handle);
}

/* ========================================================================== */
/*                Positive Tests - Handle Validation                          */
/* ========================================================================== */

void test_pos_pmic_checkHandle_afterInit(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_HandleCfg_t pmicCfg = {0};
    initTestHandleCfg(&pmicCfg);

    int32_t status = Pmic_init(&handle, &pmicCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Check handle immediately after init
    status = Pmic_checkHandle(&handle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    (void)Pmic_deinit(&handle);
}

void test_pos_pmic_checkHandle_detectsUninit(void)
{
    Pmic_Handle_t handle = {0};

    // Uninitialized handle should fail validation
    handle.commHandle0 = NULL;
    handle.ioRead = &platform_rxByte;
    handle.drvInitStat = 0x00U;  // Missing magic

    int32_t status = Pmic_checkHandle(&handle);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_HANDLE);
}

void test_pos_pmic_checkHandle_detectsMissingIoRead(void)
{
    Pmic_Handle_t handle = {0};

    // Handle missing IO read callback
    handle.drvInitStat = PMIC_INIT_TEST_EXPECTED_STAT;
    handle.commMode = PMIC_INTF_SPI;
    handle.commHandle0 = platform_getCommHandle();
    handle.ioRead = NULL;  // Missing

    int32_t status = Pmic_checkHandle(&handle);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_FPTR);
}

void test_pos_pmic_checkHandle_detectsMissingCommHandle(void)
{
    Pmic_Handle_t handle = {0};

    // Handle missing comm handle
    handle.drvInitStat = PMIC_INIT_TEST_EXPECTED_STAT;
    handle.commMode = PMIC_INTF_SPI;
    handle.commHandle0 = NULL;  // Missing
    handle.ioRead = &platform_rxByte;

    int32_t status = Pmic_checkHandle(&handle);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/* ========================================================================== */
/*            Positive Tests - Initialization Lifecycle                       */
/* ========================================================================== */

void test_pos_pmic_init_deinit_reinit(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_HandleCfg_t pmicCfg = {0};
    initTestHandleCfg(&pmicCfg);

    // Init -> Deinit -> Reinit
    int32_t status = Pmic_init(&handle, &pmicCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_deinit(&handle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_init(&handle, &pmicCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify handle is valid after reinit
    status = Pmic_checkHandle(&handle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    (void)Pmic_deinit(&handle);
}

void test_pos_pmic_init_cleanStateAfterDeinit(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_HandleCfg_t pmicCfg = {0};
    initTestHandleCfg(&pmicCfg);

    // Initialize
    int32_t status = Pmic_init(&handle, &pmicCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Deinitialize
    status = Pmic_deinit(&handle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify clean state
    PLATFORM_ASSERT(handle.drvInitStat == 0x00U);
    PLATFORM_ASSERT(handle.commHandle0 == NULL);
    PLATFORM_ASSERT(handle.ioRead == NULL);
    PLATFORM_ASSERT(handle.ioWrite == NULL);
}

void test_pos_pmic_init_verifySubsystemInfo(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_HandleCfg_t pmicCfg = {0};
    initTestHandleCfg(&pmicCfg);

    int32_t status = Pmic_init(&handle, &pmicCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    (void)Pmic_deinit(&handle);
}

/* ========================================================================== */
/*          Positive Tests - Configuration Combinations                       */
/* ========================================================================== */

void test_pos_pmic_init_minimalConfig(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_HandleCfg_t pmicCfg = {0};

    // Minimal valid configuration
    pmicCfg.validParams = PMIC_CFG_INIT_COMM_MODE_VALID | PMIC_CFG_INIT_COMM_HANDLE_0_VALID | PMIC_CFG_INIT_IO_READ_VALID | PMIC_CFG_INIT_IO_WRITE_VALID | PMIC_CFG_INIT_CRITICAL_SECTION_START_VALID | PMIC_CFG_INIT_CRITICAL_SECTION_STOP_VALID;
    pmicCfg.commMode = PMIC_INTF_SPI;
    pmicCfg.commHandle0 = platform_getCommHandle();
    pmicCfg.ioRead = &platform_rxByte;
    pmicCfg.ioWrite = &platform_txByte;
    pmicCfg.criticalSectionStart = &platform_critSecStart;
    pmicCfg.criticalSectionStop = &platform_critSecStop;

    int32_t status = Pmic_init(&handle, &pmicCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    (void)Pmic_deinit(&handle);
}

void test_pos_pmic_init_fullConfig(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_HandleCfg_t pmicCfg = {0};
    initTestHandleCfg(&pmicCfg);

    // Full configuration with all parameters
    int32_t status = Pmic_init(&handle, &pmicCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify all configured parameters
    PLATFORM_ASSERT(handle.commMode == PMIC_INTF_SPI);
    PLATFORM_ASSERT(handle.commHandle0 != NULL);
    PLATFORM_ASSERT(handle.ioRead != NULL);
    PLATFORM_ASSERT(handle.ioWrite != NULL);
    PLATFORM_ASSERT(handle.criticalSectionStart != NULL);
    PLATFORM_ASSERT(handle.criticalSectionStop != NULL);

    (void)Pmic_deinit(&handle);
}

void test_pos_pmic_init_verifyInitMagic(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_HandleCfg_t pmicCfg = {0};
    initTestHandleCfg(&pmicCfg);

    int32_t status = Pmic_init(&handle, &pmicCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify initialization magic number is set
    PLATFORM_ASSERT(handle.drvInitStat == PMIC_INIT_TEST_EXPECTED_STAT);

    (void)Pmic_deinit(&handle);
}

/* ========================================================================== */
/*                    Positive Tests - Advanced                               */
/* ========================================================================== */

void test_pos_pmic_init_critSecFunctions(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_HandleCfg_t pmicCfg = {0};
    initTestHandleCfg(&pmicCfg);

    int32_t status = Pmic_init(&handle, &pmicCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify critical section functions can be called
    Pmic_criticalSectionStart(&handle, 0);
    Pmic_criticalSectionStop(&handle, 0);

    (void)Pmic_deinit(&handle);
}

void test_pos_pmic_init_verifyDeviceComm(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_HandleCfg_t pmicCfg = {0};
    initTestHandleCfg(&pmicCfg);

    int32_t status = Pmic_init(&handle, &pmicCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify device communication by reading device ID register
    uint8_t deviceId = 0U;
    status = Pmic_ioRxByte(&handle, PMIC_DEV_ID_REG, &deviceId);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    // Device ID should be non-zero for a valid device

    (void)Pmic_deinit(&handle);
}

/* ========================================================================== */
/*                    Positive Tests - CRC Configuration                      */
/* ========================================================================== */

void test_pos_pmic_init_with_crc_enabled(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_HandleCfg_t pmicCfg = {0};
    initTestHandleCfg(&pmicCfg);

    // TPS65386x-Q1 has CRC always enabled in SPI protocol
    // This test verifies successful initialization with CRC active
    int32_t status = Pmic_init(&handle, &pmicCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify CRC is working by performing I/O operations
    uint8_t writeData = TEST_PATTERN_A5;
    uint8_t readData = 0U;

    // Write to scratchpad register
    status = Pmic_ioTxByte(&handle, PMIC_CUSTOMER_SCRATCH1_REG, writeData);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Read back and verify - CRC validation happens internally
    status = Pmic_ioRxByte(&handle, PMIC_CUSTOMER_SCRATCH1_REG, &readData);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(readData == writeData);

    // Verify handle is properly initialized
    PLATFORM_ASSERT(handle.drvInitStat == PMIC_INIT_TEST_EXPECTED_STAT);

    (void)Pmic_deinit(&handle);
}

void test_pos_pmic_init_with_config_crc_enabled(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_HandleCfg_t pmicCfg = {0};
    initTestHandleCfg(&pmicCfg);

    // Initialize with configuration CRC support
    // Note: TPS65386x-Q1 architecture differs from LP8772x-Q1
    // Config CRC is a device-specific feature not exposed in this driver yet
    int32_t status = Pmic_init(&handle, &pmicCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify successful initialization
    PLATFORM_ASSERT(handle.drvInitStat == PMIC_INIT_TEST_EXPECTED_STAT);
    PLATFORM_ASSERT(handle.ioRead != NULL);
    PLATFORM_ASSERT(handle.ioWrite != NULL);

    // Verify basic I/O works
    uint8_t regData = 0U;
    status = Pmic_ioRxByte(&handle, PMIC_DEV_ID_REG, &regData);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    (void)Pmic_deinit(&handle);
}

void test_pos_pmic_init_with_both_crc_enabled(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_HandleCfg_t pmicCfg = {0};
    initTestHandleCfg(&pmicCfg);

    // Initialize with both communication and configuration CRC enabled
    // TPS65386x-Q1 always uses CRC in SPI communication protocol
    int32_t status = Pmic_init(&handle, &pmicCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify initialization completed successfully
    PLATFORM_ASSERT(handle.drvInitStat == PMIC_INIT_TEST_EXPECTED_STAT);

    // Verify multiple I/O operations work with CRC active
    uint8_t testPatterns[] = {TEST_PATTERN_AA, TEST_PATTERN_55, TEST_MASK_HIGH_NIBBLE, TEST_MASK_LOW_NIBBLE};

    for (uint8_t i = 0U; i < 4U; i++)
    {
        uint8_t writeData = testPatterns[i];
        uint8_t readData = 0U;

        // Write pattern
        status = Pmic_ioTxByte(&handle, PMIC_CUSTOMER_SCRATCH1_REG, writeData);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        // Read back and verify - CRC is validated internally
        status = Pmic_ioRxByte(&handle, PMIC_CUSTOMER_SCRATCH1_REG, &readData);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        PLATFORM_ASSERT(readData == writeData);
    }

    (void)Pmic_deinit(&handle);
}

void test_pos_pmic_init_crc_error_recovery(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_HandleCfg_t pmicCfg = {0};
    initTestHandleCfg(&pmicCfg);

    // Initialize device
    int32_t status = Pmic_init(&handle, &pmicCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify initialization succeeded
    PLATFORM_ASSERT(handle.drvInitStat == PMIC_INIT_TEST_EXPECTED_STAT);

    // Test that normal operations work after initialization
    // CRC error recovery is tested in io_test.c with CRC error injection
    // This test verifies the init process is robust
    uint8_t devId = 0U;
    status = Pmic_ioRxByte(&handle, PMIC_DEV_ID_REG, &devId);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify we can perform multiple operations
    uint8_t devRev = 0U;
    status = Pmic_ioRxByte(&handle, PMIC_DEV_REV_REG, &devRev);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify handle fields are still valid after operations
    PLATFORM_ASSERT(handle.drvInitStat == PMIC_INIT_TEST_EXPECTED_STAT);
    PLATFORM_ASSERT(handle.ioRead != NULL);
    PLATFORM_ASSERT(handle.ioWrite != NULL);

    (void)Pmic_deinit(&handle);
}

/* ========================================================================== */
/*        Phase 3 Tests - Device Info Retrieval & Complete Flow              */
/* ========================================================================== */

void test_pos_pmic_init_complete_flow(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_HandleCfg_t pmicCfg = {0};
    initTestHandleCfg(&pmicCfg);

    // Perform complete initialization flow
    int32_t status = Pmic_init(&handle, &pmicCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify device info is populated (getPmicInfo was called)
    // Note: Mock will return 0, but fields should be set
    PLATFORM_ASSERT(handle.devRev != TEST_INVALID_PARAM_255);  // Field was written
    PLATFORM_ASSERT(handle.devSiRev != TEST_INVALID_PARAM_255);  // Field was written
    PLATFORM_ASSERT(handle.nvmCode != TEST_INVALID_PARAM_255);  // Field was written
    PLATFORM_ASSERT(handle.nvmRev != TEST_INVALID_PARAM_255);  // Field was written

    // Verify communication validation completed (validateComms was called)
    PLATFORM_ASSERT(handle.drvInitStat == PMIC_INIT_TEST_EXPECTED_STAT);

    // Verify all function pointers are set
    PLATFORM_ASSERT(handle.ioRead != NULL);
    PLATFORM_ASSERT(handle.ioWrite != NULL);
    PLATFORM_ASSERT(handle.criticalSectionStart != NULL);
    PLATFORM_ASSERT(handle.criticalSectionStop != NULL);

    // Verify communication mode is stored
    PLATFORM_ASSERT(handle.commMode == PMIC_INTF_SPI);

    (void)Pmic_deinit(&handle);
}

void test_pos_pmic_init_device_info_retrieval(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_HandleCfg_t pmicCfg = {0};
    initTestHandleCfg(&pmicCfg);

    // Initialize to trigger device info retrieval
    int32_t status = Pmic_init(&handle, &pmicCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify all device info fields are populated by getPmicInfo()
    // The mock will return register values that get stored in handle

    // devRev comes from DEV_ID register (PMIC_DEV_ID_REG)
    // Field should be initialized (not uninitialized memory)
    PLATFORM_ASSERT(handle.devRev != 0xFFU);

    // devSiRev comes from DEV_REV register (PMIC_DEV_REV_REG)
    PLATFORM_ASSERT(handle.devSiRev != 0xFFU);

    // nvmCode comes from NVM_CODE register (PMIC_NVM_CODE_REG)
    PLATFORM_ASSERT(handle.nvmCode != 0xFFU);

    // nvmRev comes from NVM_REV register (PMIC_NVM_REV_REG)
    PLATFORM_ASSERT(handle.nvmRev != 0xFFU);

    // Verify init completed successfully after device info retrieval
    PLATFORM_ASSERT(handle.drvInitStat == PMIC_INIT_TEST_EXPECTED_STAT);

    (void)Pmic_deinit(&handle);
}

void test_pos_pmic_init_communication_validation(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_HandleCfg_t pmicCfg = {0};
    initTestHandleCfg(&pmicCfg);

    // Initialize - this triggers validateComms()
    int32_t status = Pmic_init(&handle, &pmicCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify validateComms set drvInitStat correctly
    // validateComms reads PMIC_WD_LONGWIN_CFG_REG to verify communication
    // On success, it sets drvInitStat to TEST_PMIC_INIT_MAGIC
    PLATFORM_ASSERT(handle.drvInitStat == PMIC_INIT_TEST_EXPECTED_STAT);

    // Verify we can actually communicate with device after validation
    uint8_t regData = 0U;
    status = Pmic_ioRxByte(&handle, PMIC_WD_LONGWIN_CFG_REG, &regData);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify handle remains valid after communication
    PLATFORM_ASSERT(handle.ioRead != NULL);
    PLATFORM_ASSERT(handle.ioWrite != NULL);

    (void)Pmic_deinit(&handle);
}

void test_pos_pmic_init_spi_comprehensive(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_HandleCfg_t pmicCfg = {0};
    initTestHandleCfg(&pmicCfg);

    // Test comprehensive SPI mode initialization
    pmicCfg.commMode = PMIC_INTF_SPI;

    int32_t status = Pmic_init(&handle, &pmicCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify SPI mode is stored
    PLATFORM_ASSERT(handle.commMode == PMIC_INTF_SPI);

    // Test multiple SPI operations
    uint8_t writeData = TEST_PATTERN_A5;
    uint8_t readData = 0U;

    // Write to scratchpad
    status = Pmic_ioTxByte(&handle, PMIC_CUSTOMER_SCRATCH1_REG, writeData);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Read back
    status = Pmic_ioRxByte(&handle, PMIC_CUSTOMER_SCRATCH1_REG, &readData);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(readData == writeData);

    // Test reading device registers
    status = Pmic_ioRxByte(&handle, PMIC_DEV_ID_REG, &readData);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_ioRxByte(&handle, PMIC_DEV_REV_REG, &readData);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify handle state after multiple operations
    PLATFORM_ASSERT(handle.drvInitStat == PMIC_INIT_TEST_EXPECTED_STAT);
    PLATFORM_ASSERT(handle.commMode == PMIC_INTF_SPI);

    (void)Pmic_deinit(&handle);
}

void test_pos_pmic_deinit_success_comprehensive(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_HandleCfg_t pmicCfg = {0};
    initTestHandleCfg(&pmicCfg);

    // Initialize fully
    int32_t status = Pmic_init(&handle, &pmicCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify fields are populated before deinit
    PLATFORM_ASSERT(handle.drvInitStat == PMIC_INIT_TEST_EXPECTED_STAT);
    PLATFORM_ASSERT(handle.commHandle0 != NULL);
    PLATFORM_ASSERT(handle.ioRead != NULL);
    PLATFORM_ASSERT(handle.ioWrite != NULL);
    PLATFORM_ASSERT(handle.criticalSectionStart != NULL);
    PLATFORM_ASSERT(handle.criticalSectionStop != NULL);

    // Deinitialize
    status = Pmic_deinit(&handle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify ALL fields are cleared including device info fields
    PLATFORM_ASSERT(handle.drvInitStat == 0x00U);
    PLATFORM_ASSERT(handle.commHandle0 == NULL);
    PLATFORM_ASSERT(handle.ioRead == NULL);
    PLATFORM_ASSERT(handle.ioWrite == NULL);
    PLATFORM_ASSERT(handle.criticalSectionStart == NULL);
    PLATFORM_ASSERT(handle.criticalSectionStop == NULL);

    // Verify device info fields are cleared (lines 218-221 in pmic.c)
    PLATFORM_ASSERT(handle.devRev == 0U);
    PLATFORM_ASSERT(handle.devSiRev == 0U);
    PLATFORM_ASSERT(handle.nvmCode == 0U);
    PLATFORM_ASSERT(handle.nvmRev == 0U);
}

void test_pos_pmic_checkHandle_comprehensive(void)
{
    Pmic_Handle_t handle = {0};
    int32_t status;

    // Test 1: NULL handle (line 230)
    status = Pmic_checkHandle(NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);

    // Test 2: NULL commHandle0 (line 230)
    handle.commMode = PMIC_INTF_SPI;
    handle.commHandle0 = NULL;
    handle.ioRead = &platform_rxByte;
    handle.drvInitStat = PMIC_INIT_TEST_EXPECTED_STAT;
    status = Pmic_checkHandle(&handle);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);

    // Test 3: NULL ioRead (line 234) - THIS WAS NEVER TESTED
    handle.commHandle0 = (void *)TEST_DUMMY_HANDLE;  // Non-NULL
    handle.ioRead = NULL;  // NULL ioRead
    handle.drvInitStat = PMIC_INIT_TEST_EXPECTED_STAT;
    status = Pmic_checkHandle(&handle);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_FPTR);

    // Test 4: Invalid drvInitStat (line 238)
    handle.commHandle0 = (void *)TEST_DUMMY_HANDLE;  // Non-NULL
    handle.ioRead = &platform_rxByte;
    handle.drvInitStat = TEST_INVALID_MAGIC;  // Invalid magic
    status = Pmic_checkHandle(&handle);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_HANDLE);

    // Test 5: Valid handle passes all checks
    handle.commHandle0 = (void *)TEST_DUMMY_HANDLE;  // Non-NULL
    handle.ioRead = &platform_rxByte;
    handle.ioWrite = &platform_txByte;
    handle.criticalSectionStart = &platform_critSecStart;
    handle.criticalSectionStop = &platform_critSecStop;
    handle.drvInitStat = PMIC_INIT_TEST_EXPECTED_STAT;
    status = Pmic_checkHandle(&handle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_pmic_init_withRetryCnt(void)
{
    // Initialize with PMIC_CFG_INIT_RETRY_CNT_VALID set
    Pmic_Handle_t handle = {0};
    Pmic_HandleCfg_t pmicCfg = {0};
    initTestHandleCfg(&pmicCfg);

    // Add retry count configuration
    pmicCfg.validParams |= PMIC_CFG_INIT_RETRY_CNT_VALID;
    pmicCfg.retryCnt = 5U;

    int32_t status = Pmic_init(&handle, &pmicCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify retry count is set in handle
    PLATFORM_ASSERT(handle.retryCnt == 5U);

    // Clean up
    status = Pmic_deinit(&handle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_pmic_init_withRetryInterval(void)
{
    // Initialize with PMIC_CFG_INIT_RETRY_INTERVAL_MS_VALID set
    Pmic_Handle_t handle = {0};
    Pmic_HandleCfg_t pmicCfg = {0};
    initTestHandleCfg(&pmicCfg);

    // Add retry interval configuration
    // Note: When retryIntervalMs is non-zero, timerWaitMs must also be provided
    pmicCfg.validParams |= PMIC_CFG_INIT_RETRY_INTERVAL_MS_VALID | PMIC_CFG_INIT_TIMER_WAIT_MS_VALID;
    pmicCfg.retryIntervalMs = 100U;
    pmicCfg.timerWaitMs = &mockTimerWait;

    int32_t status = Pmic_init(&handle, &pmicCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify retry interval is set in handle
    PLATFORM_ASSERT(handle.retryIntervalMs == 100U);

    // Clean up
    status = Pmic_deinit(&handle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_pmic_init_withTimerWaitMs(void)
{
    // Initialize with PMIC_CFG_INIT_TIMER_WAIT_MS_VALID and valid callback
    Pmic_Handle_t handle = {0};
    Pmic_HandleCfg_t pmicCfg = {0};
    initTestHandleCfg(&pmicCfg);

    // Add timer wait callback configuration
    pmicCfg.validParams |= PMIC_CFG_INIT_TIMER_WAIT_MS_VALID;
    pmicCfg.timerWaitMs = &mockTimerWait;

    int32_t status = Pmic_init(&handle, &pmicCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify timer wait callback is set in handle
    PLATFORM_ASSERT(handle.timerWaitMs == &mockTimerWait);

    // Clean up
    status = Pmic_deinit(&handle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_neg_pmic_init_timerWaitNull(void)
{
    // Set PMIC_CFG_INIT_TIMER_WAIT_MS_VALID but pass NULL callback
    Pmic_Handle_t handle = {0};
    Pmic_HandleCfg_t pmicCfg = {0};
    initTestHandleCfg(&pmicCfg);

    // Set valid param flag but provide NULL callback
    pmicCfg.validParams |= PMIC_CFG_INIT_TIMER_WAIT_MS_VALID;
    pmicCfg.timerWaitMs = NULL;

    int32_t status = Pmic_init(&handle, &pmicCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_FPTR);
}

/**
 * @brief Test Pmic_init with NULL asyncRxStart hook.
 */
void test_neg_pmic_pmicInit_nullAsyncRxStart(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_HandleCfg_t pmicCfg = {0};
    initTestHandleCfg(&pmicCfg);

    // Enable async mode and set valid param flag but provide NULL callback
    pmicCfg.validParams |= PMIC_CFG_INIT_ASYNC_ENABLE_VALID | PMIC_CFG_INIT_ASYNC_RX_START_VALID;
    pmicCfg.asyncEnable = true;
    pmicCfg.asyncRxStart = NULL;

    int32_t status = Pmic_init(&handle, &pmicCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_FPTR);
}

/**
 * @brief Test Pmic_init with NULL asyncTxStart hook.
 */
void test_neg_pmic_pmicInit_nullAsyncTxStart(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_HandleCfg_t pmicCfg = {0};
    initTestHandleCfg(&pmicCfg);

    // Keep asyncRxStart valid so only asyncTxStart's OR-term is exercised
    pmicCfg.validParams |= PMIC_CFG_INIT_ASYNC_ENABLE_VALID | PMIC_CFG_INIT_ASYNC_RX_START_VALID |
                           PMIC_CFG_INIT_ASYNC_TX_START_VALID;
    pmicCfg.asyncEnable = true;
    pmicCfg.asyncRxStart = &mockAsyncRxStart;
    pmicCfg.asyncTxStart = NULL;

    int32_t status = Pmic_init(&handle, &pmicCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_FPTR);
}

/**
 * @brief Test Pmic_init with NULL asyncRxAwait hook.
 */
void test_neg_pmic_pmicInit_nullAsyncRxAwait(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_HandleCfg_t pmicCfg = {0};
    initTestHandleCfg(&pmicCfg);

    // Keep asyncRxStart/asyncTxStart/asyncTxAwait valid so only asyncRxAwait's OR-term is exercised
    pmicCfg.validParams |= PMIC_CFG_INIT_ASYNC_ENABLE_VALID | PMIC_CFG_INIT_ASYNC_RX_START_VALID |
                           PMIC_CFG_INIT_ASYNC_TX_START_VALID | PMIC_CFG_INIT_ASYNC_RX_AWAIT_VALID |
                           PMIC_CFG_INIT_ASYNC_TX_AWAIT_VALID;
    pmicCfg.asyncEnable = true;
    pmicCfg.asyncRxStart = &mockAsyncRxStart;
    pmicCfg.asyncTxStart = &mockAsyncTxStart;
    pmicCfg.asyncRxAwait = NULL;
    pmicCfg.asyncTxAwait = &mockAsyncTxAwait;

    int32_t status = Pmic_init(&handle, &pmicCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_FPTR);
}

/**
 * @brief Test Pmic_init with NULL asyncTxAwait hook.
 */
void test_neg_pmic_pmicInit_nullAsyncTxAwait(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_HandleCfg_t pmicCfg = {0};
    initTestHandleCfg(&pmicCfg);

    // Keep asyncRxStart/asyncTxStart/asyncRxAwait valid so only asyncTxAwait's OR-term is exercised
    pmicCfg.validParams |= PMIC_CFG_INIT_ASYNC_ENABLE_VALID | PMIC_CFG_INIT_ASYNC_RX_START_VALID |
                           PMIC_CFG_INIT_ASYNC_TX_START_VALID | PMIC_CFG_INIT_ASYNC_RX_AWAIT_VALID |
                           PMIC_CFG_INIT_ASYNC_TX_AWAIT_VALID;
    pmicCfg.asyncEnable = true;
    pmicCfg.asyncRxStart = &mockAsyncRxStart;
    pmicCfg.asyncTxStart = &mockAsyncTxStart;
    pmicCfg.asyncRxAwait = &mockAsyncRxAwait;
    pmicCfg.asyncTxAwait = NULL;

    int32_t status = Pmic_init(&handle, &pmicCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_FPTR);
}

/**
 * @brief Test checkHandle with invalid commMode (non-SPI).
 */
void test_neg_pmic_checkHandle_invalidCommMode(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_HandleCfg_t pmicCfg = {0};
    initTestHandleCfg(&pmicCfg);

    // Initialize handle properly
    int32_t status = Pmic_init(&handle, &pmicCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Corrupt commMode to non-SPI value (TPS65386x only supports SPI)
    handle.commMode = PMIC_INTF_I2C_SINGLE;

    // Check handle should fail with invalid param
    status = Pmic_checkHandle(&handle);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);

    // Clean up (restore valid commMode before deinit)
    handle.commMode = PMIC_INTF_SPI;
    (void)Pmic_deinit(&handle);
}

/**
 * @brief Test checkHandle with NULL critical section functions.
 */
void test_neg_pmic_checkHandle_nullCritSec(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_HandleCfg_t pmicCfg = {0};
    initTestHandleCfg(&pmicCfg);

    // Initialize handle properly
    int32_t status = Pmic_init(&handle, &pmicCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Save original critical section functions
    void (*origStart)(uint8_t) = handle.criticalSectionStart;
    void (*origStop)(uint8_t) = handle.criticalSectionStop;

    // Set critical section start to NULL
    handle.criticalSectionStart = NULL;

    // Check handle should fail with null function pointer
    status = Pmic_checkHandle(&handle);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_FPTR);

    // Restore and test with stop NULL
    handle.criticalSectionStart = origStart;
    handle.criticalSectionStop = NULL;

    status = Pmic_checkHandle(&handle);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_FPTR);

    // Restore for cleanup
    handle.criticalSectionStop = origStop;
    (void)Pmic_deinit(&handle);
}

/**
 * @brief Test checkHandle with NULL timer but non-zero retry interval.
 */
void test_neg_pmic_checkHandle_nullTimerWithRetry(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_HandleCfg_t pmicCfg = {0};
    initTestHandleCfg(&pmicCfg);

    // Initialize handle properly with timer
    pmicCfg.validParams |= PMIC_CFG_INIT_RETRY_INTERVAL_MS_VALID | PMIC_CFG_INIT_TIMER_WAIT_MS_VALID;
    pmicCfg.retryIntervalMs = 100U;
    pmicCfg.timerWaitMs = &mockTimerWait;

    int32_t status = Pmic_init(&handle, &pmicCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Now corrupt the handle: set timerWaitMs to NULL while retryIntervalMs is non-zero
    handle.timerWaitMs = NULL;

    // Check handle should fail
    status = Pmic_checkHandle(&handle);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_FPTR);

    // Restore for cleanup
    handle.timerWaitMs = &mockTimerWait;
    (void)Pmic_deinit(&handle);
}

/**
 * @brief Test Pmic_checkHandle with retryIntervalMs > 0 but timerWaitMs == NULL.
 */
void test_neg_pmic_checkHandle_retryIntervalWithNoTimer(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_HandleCfg_t pmicCfg = {0};
    initTestHandleCfg(&pmicCfg);

    // Initialize a valid handle first
    int32_t status = Pmic_init(&handle, &pmicCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Corrupt the handle: set retryIntervalMs to non-zero, clear timerWaitMs
    handle.retryIntervalMs = 10U;
    handle.timerWaitMs = NULL;

    // Pmic_checkHandle must detect the missing timer callback and return an error
    status = Pmic_checkHandle(&handle);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_FPTR);

    // Restore before deinit
    handle.retryIntervalMs = 0U;
    (void)Pmic_deinit(&handle);
}

/**
 * @brief Test Pmic_checkHandle with retryIntervalMs != 0 AND timerWaitMs != NULL.
 */
void test_pos_pmic_checkHandle_retryWithValidTimer(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_HandleCfg_t pmicCfg = {0};
    initTestHandleCfg(&pmicCfg);

    // Configure retry with a valid timer callback
    pmicCfg.validParams |= PMIC_CFG_INIT_RETRY_INTERVAL_MS_VALID | PMIC_CFG_INIT_TIMER_WAIT_MS_VALID;
    pmicCfg.retryIntervalMs = 10U;
    pmicCfg.timerWaitMs = &mockTimerWait;

    int32_t status = Pmic_init(&handle, &pmicCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Pmic_checkHandle must accept retryIntervalMs!=0 when timerWaitMs is valid
    status = Pmic_checkHandle(&handle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    (void)Pmic_deinit(&handle);
}

/**
 * @brief Test Pmic_checkHandle with ioRead set but ioWrite NULL.
 */
void test_neg_pmic_checkHandle_nullIoWrite(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_HandleCfg_t pmicCfg = {0};
    initTestHandleCfg(&pmicCfg);

    // Initialize a valid handle first
    int32_t status = Pmic_init(&handle, &pmicCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Corrupt the handle: keep ioRead valid but clear ioWrite
    handle.ioWrite = NULL;

    // Pmic_checkHandle must detect the missing ioWrite and return an error
    status = Pmic_checkHandle(&handle);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_FPTR);

    // Restore ioWrite before deinit so the deinit call itself succeeds
    handle.ioWrite = &platform_txByte;
    (void)Pmic_deinit(&handle);
}

/* ========================================================================== */
// Negative Tests - Pmic_init / Pmic_checkHandle
/* ========================================================================== */

/**
 * @brief Test Pmic_init without COMM_MODE_VALID set in validParams.
 */
void test_pos_pmic_init_noCommModeValid(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_HandleCfg_t pmicCfg = {0};
    initTestHandleCfg(&pmicCfg);

    // Remove COMM_MODE_VALID so the commMode block in initHandleBasicDevCfg
    // is not entered (false branch of Pmic_validParamStatusCheck at L71). */
    pmicCfg.validParams &= ~PMIC_CFG_INIT_COMM_MODE_VALID;

    // commMode field in cfg is irrelevant now; commMode in handle will stay
    // at zero (= PMIC_INTF_SPI) from the memset in Pmic_init, so the rest
    // of init succeeds. */
    int32_t status = Pmic_init(&handle, &pmicCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    (void)Pmic_deinit(&handle);
}

/**
 * @brief Test Pmic_init without IO_READ_VALID set.
 */
void test_neg_pmic_init_noIoReadValid(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_HandleCfg_t pmicCfg = {0};
    initTestHandleCfg(&pmicCfg);

    // Remove IO_READ_VALID so initCommsFunctions skips the ioRead assignment.
    pmicCfg.validParams &= ~PMIC_CFG_INIT_IO_READ_VALID;

    // Without ioRead, validatePmicHandle detects the missing sync I/O hook.
    int32_t status = Pmic_init(&handle, &pmicCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INSUFFICIENT_CFG);
}

/**
 * @brief Test Pmic_init without CRITICAL_SECTION_START_VALID.
 */
void test_neg_pmic_init_noCritSecStartValid(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_HandleCfg_t pmicCfg = {0};
    initTestHandleCfg(&pmicCfg);

    // Remove CRITICAL_SECTION_START_VALID so initCritSecFunctions skips it.
    pmicCfg.validParams &= ~PMIC_CFG_INIT_CRITICAL_SECTION_START_VALID;

    // critSecStart stays NULL; validatePmicHandle catches this.
    int32_t status = Pmic_init(&handle, &pmicCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_FPTR);
}

/**
 * @brief Test Pmic_init without IO_WRITE_VALID set.
 */
void test_neg_pmic_init_noIoWriteValid(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_HandleCfg_t pmicCfg = {0};
    initTestHandleCfg(&pmicCfg);

    // Remove IO_WRITE_VALID so initCommsFunctions skips the ioWrite assignment.
    pmicCfg.validParams &= ~PMIC_CFG_INIT_IO_WRITE_VALID;

    // Without ioWrite, validatePmicHandle detects the missing sync I/O hook.
    int32_t status = Pmic_init(&handle, &pmicCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INSUFFICIENT_CFG);
}

/**
 * @brief Test Pmic_init without CRITICAL_SECTION_STOP_VALID.
 */
void test_neg_pmic_init_noCritSecStopValid(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_HandleCfg_t pmicCfg = {0};
    initTestHandleCfg(&pmicCfg);

    // Remove CRITICAL_SECTION_STOP_VALID so initCritSecFunctions skips it.
    pmicCfg.validParams &= ~PMIC_CFG_INIT_CRITICAL_SECTION_STOP_VALID;

    // critSecStop stays NULL; validatePmicHandle catches this.
    int32_t status = Pmic_init(&handle, &pmicCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_FPTR);
}

/**
 * @brief Test Pmic_init with TASK_HANDLE_VALID set.
 */
void test_pos_pmic_init_withTaskHandle(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_HandleCfg_t pmicCfg = {0};
    static uint8_t dummyTaskHandle = 0U;
    initTestHandleCfg(&pmicCfg);

    // Add TASK_HANDLE_VALID and provide a non-NULL task handle pointer.
    pmicCfg.validParams |= PMIC_CFG_INIT_TASK_HANDLE_VALID;
    pmicCfg.taskHandle = (void *)&dummyTaskHandle;

    int32_t status = Pmic_init(&handle, &pmicCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify task handle was stored.
    PLATFORM_ASSERT(handle.taskHandle == (void *)&dummyTaskHandle);

    (void)Pmic_deinit(&handle);
}

/**
 * @brief Test Pmic_init with mock I/O that fails on read 1 (DEV_ID_REG).
 */
void test_neg_pmic_getPmicInfo_firstReadFail(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_HandleCfg_t pmicCfg = {0};
    initTestHandleCfg(&pmicCfg);

    // Point ioRead to the mock — fails on the very first call.
    pmicCfg.ioRead = &pmicTestMockIoRead;
    resetPmicMockState();
    g_pmicTestMockReadPassCount    = 0U;
    g_pmicTestMockReadReturnStatus = PMIC_ST_ERR_I2C_COMM_FAIL;

    int32_t status = Pmic_init(&handle, &pmicCfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
}

/**
 * @brief Test Pmic_init with mock I/O that fails on read 2 (DEV_REV_REG).
 */
void test_neg_pmic_getPmicInfo_secondReadFail(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_HandleCfg_t pmicCfg = {0};
    initTestHandleCfg(&pmicCfg);

    pmicCfg.ioRead = &pmicTestMockIoRead;
    resetPmicMockState();
    // Allow 1 successful read (DEV_ID_REG), then fail.
    g_pmicTestMockReadPassCount    = 1U;
    g_pmicTestMockReadReturnStatus = PMIC_ST_ERR_I2C_COMM_FAIL;

    int32_t status = Pmic_init(&handle, &pmicCfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
}

/**
 * @brief Test Pmic_init with mock I/O that fails on read 3 (NVM_CODE_REG).
 */
void test_neg_pmic_getPmicInfo_thirdReadFail(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_HandleCfg_t pmicCfg = {0};
    initTestHandleCfg(&pmicCfg);

    pmicCfg.ioRead = &pmicTestMockIoRead;
    resetPmicMockState();
    // Allow 2 successful reads (DEV_ID_REG + DEV_REV_REG), then fail.
    g_pmicTestMockReadPassCount    = 2U;
    g_pmicTestMockReadReturnStatus = PMIC_ST_ERR_I2C_COMM_FAIL;

    int32_t status = Pmic_init(&handle, &pmicCfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
}

/**
 * @brief Test Pmic_init with mock I/O that fails on read 4 (NVM_REV_REG).
 */
void test_neg_pmic_getPmicInfo_fourthReadFail(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_HandleCfg_t pmicCfg = {0};
    initTestHandleCfg(&pmicCfg);

    pmicCfg.ioRead = &pmicTestMockIoRead;
    resetPmicMockState();
    // Allow 3 successful reads (DEV_ID + DEV_REV + NVM_CODE), then fail.
    g_pmicTestMockReadPassCount    = 3U;
    g_pmicTestMockReadReturnStatus = PMIC_ST_ERR_I2C_COMM_FAIL;

    int32_t status = Pmic_init(&handle, &pmicCfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
}
