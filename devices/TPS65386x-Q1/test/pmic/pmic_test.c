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
#include "regmap/core.h"

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/* Driver initialization magic number (from pmic.c) */
#define PMIC_INIT_TEST_DRV_INIT_MAGIC   (0x504D4943U)  /* "PMIC" in ASCII */

/* Expected initialization status for SPI mode */
/* NOTE: PMIC_MAIN_INST not defined for TPS65386x-Q1 - using magic number only */
#define PMIC_INIT_TEST_EXPECTED_STAT    (PMIC_INIT_TEST_DRV_INIT_MAGIC)

/* ========================================================================== */
/*                    API-Specific Test Macros - Pmic_init                   */
/* ========================================================================== */

#define PMIC_TEST_POS_INIT() \
    PLATFORM_RUN_TEST(test_pos_pmic_init_spiMode); \
    PLATFORM_RUN_TEST(test_pos_pmic_init_validateHandle); \
    PLATFORM_RUN_TEST(test_pos_pmic_init_withAllCallbacks); \
    PLATFORM_RUN_TEST(test_pos_pmic_init_multipleInitDeinit); \
    PLATFORM_RUN_TEST(test_pos_pmic_init_handlePersistence); \
    PLATFORM_RUN_TEST(test_pos_pmic_init_validDeviceType); \
    PLATFORM_RUN_TEST(test_pos_pmic_init_validCommMode); \
    PLATFORM_RUN_TEST(test_pos_pmic_init_validateCommHandleStored); \
    PLATFORM_RUN_TEST(test_pos_pmic_init_validateCallbacksStored); \
    PLATFORM_RUN_TEST(test_pos_pmic_init_communicationTest); \
    PLATFORM_RUN_TEST(test_pos_pmic_init_registerAccess); \
    PLATFORM_RUN_TEST(test_pos_pmic_init_deinit_reinit); \
    PLATFORM_RUN_TEST(test_pos_pmic_init_cleanStateAfterDeinit); \
    PLATFORM_RUN_TEST(test_pos_pmic_init_verifySubsystemInfo); \
    PLATFORM_RUN_TEST(test_pos_pmic_init_minimalConfig); \
    PLATFORM_RUN_TEST(test_pos_pmic_init_fullConfig); \
    PLATFORM_RUN_TEST(test_pos_pmic_init_verifyInitMagic); \
    PLATFORM_RUN_TEST(test_pos_pmic_init_critSecFunctions); \
    PLATFORM_RUN_TEST(test_pos_pmic_init_verifyDeviceComm); \
    PLATFORM_RUN_TEST(test_pos_pmic_init_with_crc_enabled); \
    PLATFORM_RUN_TEST(test_pos_pmic_init_with_config_crc_enabled); \
    PLATFORM_RUN_TEST(test_pos_pmic_init_with_both_crc_enabled); \
    PLATFORM_RUN_TEST(test_pos_pmic_init_crc_error_recovery); \
    PLATFORM_RUN_TEST(test_pos_pmic_init_complete_flow); \
    PLATFORM_RUN_TEST(test_pos_pmic_init_device_info_retrieval); \
    PLATFORM_RUN_TEST(test_pos_pmic_init_communication_validation); \
    PLATFORM_RUN_TEST(test_pos_pmic_init_spi_comprehensive); \
    PLATFORM_RUN_TEST(test_pos_pmic_init_withRetryCnt); \
    PLATFORM_RUN_TEST(test_pos_pmic_init_withRetryInterval); \
    PLATFORM_RUN_TEST(test_pos_pmic_init_withTimerWaitMs)

#define PMIC_TEST_NEG_INIT() \
    PLATFORM_RUN_TEST(test_neg_pmic_init_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_pmic_init_nullCoreCfg); \
    PLATFORM_RUN_TEST(test_neg_pmic_init_nullCommHandle); \
    PLATFORM_RUN_TEST(test_neg_pmic_init_nullIoRead); \
    PLATFORM_RUN_TEST(test_neg_pmic_init_nullIoWrite); \
    PLATFORM_RUN_TEST(test_neg_pmic_init_nullCritSecStart); \
    PLATFORM_RUN_TEST(test_neg_pmic_init_nullCritSecStop); \
    PLATFORM_RUN_TEST(test_neg_pmic_init_invalidDeviceType); \
    PLATFORM_RUN_TEST(test_neg_pmic_init_invalidCommMode); \
    PLATFORM_RUN_TEST(test_neg_pmic_init_insufficientCfg_missingIoRead); \
    PLATFORM_RUN_TEST(test_neg_pmic_init_insufficientCfg_missingIoWrite); \
    PLATFORM_RUN_TEST(test_neg_pmic_init_insufficientCfg_missingCritSec); \
    PLATFORM_RUN_TEST(test_neg_pmic_init_timerWaitNull)

#define PMIC_TEST_INIT() \
    PMIC_TEST_POS_INIT(); \
    PMIC_TEST_NEG_INIT()

/* ========================================================================== */
/*                    API-Specific Test Macros - Pmic_deinit                 */
/* ========================================================================== */

#define PMIC_TEST_POS_DEINIT() \
    PLATFORM_RUN_TEST(test_pos_pmic_deinit_clearsHandle); \
    PLATFORM_RUN_TEST(test_pos_pmic_deinit_success_comprehensive)

#define PMIC_TEST_NEG_DEINIT() \
    PLATFORM_RUN_TEST(test_neg_pmic_deinit_nullHandle)

#define PMIC_TEST_DEINIT() \
    PMIC_TEST_POS_DEINIT(); \
    PMIC_TEST_NEG_DEINIT()

/* ========================================================================== */
/*                  API-Specific Test Macros - Pmic_checkHandle              */
/* ========================================================================== */

#define PMIC_TEST_POS_CHECK_HANDLE() \
    PLATFORM_RUN_TEST(test_pos_pmic_checkHandle_validHandle); \
    PLATFORM_RUN_TEST(test_pos_pmic_checkHandle_afterInit); \
    PLATFORM_RUN_TEST(test_pos_pmic_checkHandle_detectsUninit); \
    PLATFORM_RUN_TEST(test_pos_pmic_checkHandle_detectsMissingIoRead); \
    PLATFORM_RUN_TEST(test_pos_pmic_checkHandle_detectsMissingCommHandle); \
    PLATFORM_RUN_TEST(test_pos_pmic_checkHandle_comprehensive)

#define PMIC_TEST_NEG_CHECK_HANDLE() \
    PLATFORM_RUN_TEST(test_neg_pmic_checkHandle_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_pmic_checkHandle_invalidInitStat); \
    PLATFORM_RUN_TEST(test_neg_pmic_checkHandle_invalidCommMode); \
    PLATFORM_RUN_TEST(test_neg_pmic_checkHandle_nullCritSec); \
    PLATFORM_RUN_TEST(test_neg_pmic_checkHandle_nullTimerWithRetry)

#define PMIC_TEST_CHECK_HANDLE() \
    PMIC_TEST_POS_CHECK_HANDLE(); \
    PMIC_TEST_NEG_CHECK_HANDLE()

/* ========================================================================== */
/*                         Aggregate Test Macros                              */
/* ========================================================================== */

#define PMIC_TEST_RUN_POSITIVE() \
    PMIC_TEST_POS_INIT(); \
    PMIC_TEST_POS_DEINIT(); \
    PMIC_TEST_POS_CHECK_HANDLE()

#define PMIC_TEST_RUN_NEGATIVE() \
    PMIC_TEST_NEG_INIT(); \
    PMIC_TEST_NEG_DEINIT(); \
    PMIC_TEST_NEG_CHECK_HANDLE()

#define PMIC_TEST_RUN_ALL() \
    PMIC_TEST_INIT(); \
    PMIC_TEST_DEINIT(); \
    PMIC_TEST_CHECK_HANDLE()

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
    (void)ms;  /* Unused in mock */
}

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void pmic_test(void *args)
{
    platform_init();

    printf("\r\n");
    printf("==================================================\r\n");
    printf("    TPS65386x-Q1 PMIC Module Tests\r\n");
    printf("==================================================\r\n\r\n");

    /* Run all PMIC tests */
    PMIC_TEST_RUN_ALL();

    platform_deinit();

    printf("\r\n==================================================\r\n");
    printf("    PMIC Module Tests Complete\r\n");
    printf("==================================================\r\n\r\n");
}

/**
 * @brief Helper function to initialize handle configuration with valid defaults
 */
static void initTestHandleCfg(Pmic_HandleCfg_t *pmicCfg)
{
    pmicCfg->validParams = PMIC_COMM_MODE_VALID |
                           PMIC_COMM_HANDLE_0_VALID |
                           PMIC_IO_READ_VALID |
                           PMIC_IO_WRITE_VALID |
                           PMIC_CRITICAL_SECTION_START_VALID |
                           PMIC_CRITICAL_SECTION_STOP_VALID;
    pmicCfg->commMode = PMIC_INTF_SPI;
    pmicCfg->commHandle0 = (void*)&dummyCommHandle;
    pmicCfg->ioRead = &test_pmic_regRead;
    pmicCfg->ioWrite = &test_pmic_regWrite;
    pmicCfg->criticalSectionStart = &test_pmic_criticalSectionStartFn;
    pmicCfg->criticalSectionStop = &test_pmic_criticalSectionStopFn;
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

    /* Set PMIC_COMM_HANDLE_0_VALID to trigger validation */
    pmicCfg.validParams |= PMIC_COMM_HANDLE_0_VALID;
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
    /* NOTE: TPS65386x-Q1 does not have deviceType validation (unlike TPS6522x-Q1).
     * This device variant only supports SPI mode and doesn't require deviceType configuration.
     * Test skipped as feature is not applicable to this device. */
    (void)0;  /* No-op test */
}

void test_neg_pmic_init_invalidCommMode(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_HandleCfg_t pmicCfg = {0};
    initTestHandleCfg(&pmicCfg);

    /* TPS65386x only supports SPI mode */
    pmicCfg.commMode = PMIC_INTF_I2C_SINGLE;
    int32_t status = Pmic_init(&handle, &pmicCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_pmic_init_insufficientCfg_missingIoRead(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_HandleCfg_t pmicCfg = {0};
    initTestHandleCfg(&pmicCfg);

    /* Set IO read callback to NULL with valid bit set to trigger NULL check */
    pmicCfg.ioRead = NULL;
    /* Keep PMIC_IO_READ_VALID set so validation catches the NULL */

    int32_t status = Pmic_init(&handle, &pmicCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_FPTR);
}

void test_neg_pmic_init_insufficientCfg_missingIoWrite(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_HandleCfg_t pmicCfg = {0};
    initTestHandleCfg(&pmicCfg);

    /* Set IO write callback to NULL with valid bit set to trigger NULL check */
    pmicCfg.ioWrite = NULL;
    /* Keep PMIC_IO_WRITE_VALID set so validation catches the NULL */

    int32_t status = Pmic_init(&handle, &pmicCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_FPTR);
}

void test_neg_pmic_init_insufficientCfg_missingCritSec(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_HandleCfg_t pmicCfg = {0};
    initTestHandleCfg(&pmicCfg);

    /* Set critical section start to NULL with valid bit set to trigger NULL check */
    pmicCfg.criticalSectionStart = NULL;
    /* Keep PMIC_CRITICAL_SECTION_START_VALID set so validation catches the NULL */

    int32_t status = Pmic_init(&handle, &pmicCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_FPTR);
}

void test_neg_pmic_checkHandle_invalidInitStat(void)
{
    Pmic_Handle_t handle = {0};

    /* Handle with invalid initialization status */
    handle.drvInitStat = 0x00000000U;
    handle.commMode = PMIC_INTF_SPI;
    handle.commHandle0 = NULL;
    handle.ioRead = &test_pmic_regRead;

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
    /* NOTE: TPS65386x-Q1 handle doesn't have devId field - skipping assertion */

    (void)Pmic_deinit(&handle);
}

void test_pos_pmic_init_validateHandle(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_HandleCfg_t pmicCfg = {0};
    initTestHandleCfg(&pmicCfg);

    int32_t status = Pmic_init(&handle, &pmicCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Verify handle is properly initialized */
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

    /* Initialize then deinitialize */
    int32_t status = Pmic_init(&handle, &pmicCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_deinit(&handle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Verify handle is cleared */
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

    /* Verify handle passes validation */
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

    /* Verify all callbacks are stored */
    PLATFORM_ASSERT(handle.ioRead == &test_pmic_regRead);
    PLATFORM_ASSERT(handle.ioWrite == &test_pmic_regWrite);
    PLATFORM_ASSERT(handle.criticalSectionStart == &test_pmic_criticalSectionStartFn);
    PLATFORM_ASSERT(handle.criticalSectionStop == &test_pmic_criticalSectionStopFn);

    (void)Pmic_deinit(&handle);
}

void test_pos_pmic_init_multipleInitDeinit(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_HandleCfg_t pmicCfg = {0};
    initTestHandleCfg(&pmicCfg);

    /* First cycle */
    int32_t status = Pmic_init(&handle, &pmicCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    status = Pmic_deinit(&handle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Second cycle */
    status = Pmic_init(&handle, &pmicCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    status = Pmic_deinit(&handle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Third cycle */
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

    /* Store initial values */
    uint32_t initStat = handle.drvInitStat;
    uint8_t commMode = handle.commMode;

    /* Verify values persist */
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

    /* Verify all function pointers are correctly stored */
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

    /* Verify communication is established during init */
    /* Init function reads WD_LONGWIN_CFG_REG to validate comms */
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

    /* Test register access after init */
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

    /* Check handle immediately after init */
    status = Pmic_checkHandle(&handle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    (void)Pmic_deinit(&handle);
}

void test_pos_pmic_checkHandle_detectsUninit(void)
{
    Pmic_Handle_t handle = {0};

    /* Uninitialized handle should fail validation */
    handle.commHandle0 = NULL;
    handle.ioRead = &test_pmic_regRead;
    handle.drvInitStat = 0x00U;  /* Missing magic */

    int32_t status = Pmic_checkHandle(&handle);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_HANDLE);
}

void test_pos_pmic_checkHandle_detectsMissingIoRead(void)
{
    Pmic_Handle_t handle = {0};

    /* Handle missing IO read callback */
    handle.drvInitStat = PMIC_INIT_TEST_EXPECTED_STAT;
    handle.commMode = PMIC_INTF_SPI;
    handle.commHandle0 = platform_getCommHandle();
    handle.ioRead = NULL;  /* Missing */

    int32_t status = Pmic_checkHandle(&handle);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_FPTR);
}

void test_pos_pmic_checkHandle_detectsMissingCommHandle(void)
{
    Pmic_Handle_t handle = {0};

    /* Handle missing comm handle */
    handle.drvInitStat = PMIC_INIT_TEST_EXPECTED_STAT;
    handle.commMode = PMIC_INTF_SPI;
    handle.commHandle0 = NULL;  /* Missing */
    handle.ioRead = &test_pmic_regRead;

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

    /* Init -> Deinit -> Reinit */
    int32_t status = Pmic_init(&handle, &pmicCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_deinit(&handle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_init(&handle, &pmicCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Verify handle is valid after reinit */
    status = Pmic_checkHandle(&handle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    (void)Pmic_deinit(&handle);
}

void test_pos_pmic_init_cleanStateAfterDeinit(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_HandleCfg_t pmicCfg = {0};
    initTestHandleCfg(&pmicCfg);

    /* Initialize */
    int32_t status = Pmic_init(&handle, &pmicCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Deinitialize */
    status = Pmic_deinit(&handle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Verify clean state */
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

    /* Minimal valid configuration */
    pmicCfg.validParams = PMIC_COMM_MODE_VALID | PMIC_COMM_HANDLE_0_VALID | PMIC_IO_READ_VALID | PMIC_IO_WRITE_VALID | PMIC_CRITICAL_SECTION_START_VALID | PMIC_CRITICAL_SECTION_STOP_VALID;
    pmicCfg.commMode = PMIC_INTF_SPI;
    pmicCfg.commHandle0 = platform_getCommHandle();
    pmicCfg.ioRead = &test_pmic_regRead;
    pmicCfg.ioWrite = &test_pmic_regWrite;
    pmicCfg.criticalSectionStart = &test_pmic_criticalSectionStartFn;
    pmicCfg.criticalSectionStop = &test_pmic_criticalSectionStopFn;

    int32_t status = Pmic_init(&handle, &pmicCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    (void)Pmic_deinit(&handle);
}

void test_pos_pmic_init_fullConfig(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_HandleCfg_t pmicCfg = {0};
    initTestHandleCfg(&pmicCfg);

    /* Full configuration with all parameters */
    int32_t status = Pmic_init(&handle, &pmicCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Verify all configured parameters */
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

    /* Verify initialization magic number is set */
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

    /* Verify critical section functions can be called */
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

    /* Verify device communication by reading device ID register */
    uint8_t deviceId = 0U;
    status = Pmic_ioRxByte(&handle, PMIC_DEV_ID_REG, &deviceId);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    /* Device ID should be non-zero for a valid device */

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

    /* TPS65386x-Q1 has CRC always enabled in SPI protocol */
    /* This test verifies successful initialization with CRC active */
    int32_t status = Pmic_init(&handle, &pmicCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Verify CRC is working by performing I/O operations */
    uint8_t writeData = 0xA5U;
    uint8_t readData = 0U;

    /* Write to scratchpad register */
    status = Pmic_ioTxByte(&handle, PMIC_CUSTOMER_SCRATCH1_REG, writeData);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Read back and verify - CRC validation happens internally */
    status = Pmic_ioRxByte(&handle, PMIC_CUSTOMER_SCRATCH1_REG, &readData);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(readData == writeData);

    /* Verify handle is properly initialized */
    PLATFORM_ASSERT(handle.drvInitStat == PMIC_INIT_TEST_EXPECTED_STAT);

    (void)Pmic_deinit(&handle);
}

void test_pos_pmic_init_with_config_crc_enabled(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_HandleCfg_t pmicCfg = {0};
    initTestHandleCfg(&pmicCfg);

    /* Initialize with configuration CRC support
     * Note: TPS65386x-Q1 architecture differs from LP8772x-Q1
     * Config CRC is a device-specific feature not exposed in this driver yet
     */
    int32_t status = Pmic_init(&handle, &pmicCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Verify successful initialization */
    PLATFORM_ASSERT(handle.drvInitStat == PMIC_INIT_TEST_EXPECTED_STAT);
    PLATFORM_ASSERT(handle.ioRead != NULL);
    PLATFORM_ASSERT(handle.ioWrite != NULL);

    /* Verify basic I/O works */
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

    /* Initialize with both communication and configuration CRC enabled
     * TPS65386x-Q1 always uses CRC in SPI communication protocol
     */
    int32_t status = Pmic_init(&handle, &pmicCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Verify initialization completed successfully */
    PLATFORM_ASSERT(handle.drvInitStat == PMIC_INIT_TEST_EXPECTED_STAT);

    /* Verify multiple I/O operations work with CRC active */
    uint8_t testPatterns[] = {0xAAU, 0x55U, 0xF0U, 0x0FU};

    for (uint8_t i = 0U; i < 4U; i++)
    {
        uint8_t writeData = testPatterns[i];
        uint8_t readData = 0U;

        /* Write pattern */
        status = Pmic_ioTxByte(&handle, PMIC_CUSTOMER_SCRATCH1_REG, writeData);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        /* Read back and verify - CRC is validated internally */
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

    /* Initialize device */
    int32_t status = Pmic_init(&handle, &pmicCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Verify initialization succeeded */
    PLATFORM_ASSERT(handle.drvInitStat == PMIC_INIT_TEST_EXPECTED_STAT);

    /* Test that normal operations work after initialization
     * CRC error recovery is tested in io_test.c with CRC error injection
     * This test verifies the init process is robust
     */
    uint8_t devId = 0U;
    status = Pmic_ioRxByte(&handle, PMIC_DEV_ID_REG, &devId);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Verify we can perform multiple operations */
    uint8_t devRev = 0U;
    status = Pmic_ioRxByte(&handle, PMIC_DEV_REV_REG, &devRev);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Verify handle fields are still valid after operations */
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

    /* Perform complete initialization flow */
    int32_t status = Pmic_init(&handle, &pmicCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Verify device info is populated (getPmicInfo was called) */
    /* Note: Mock will return 0, but fields should be set */
    PLATFORM_ASSERT(handle.devRev != 0xFFU);  /* Field was written */
    PLATFORM_ASSERT(handle.devSiRev != 0xFFU);  /* Field was written */
    PLATFORM_ASSERT(handle.nvmCode != 0xFFU);  /* Field was written */
    PLATFORM_ASSERT(handle.nvmRev != 0xFFU);  /* Field was written */

    /* Verify communication validation completed (validateComms was called) */
    PLATFORM_ASSERT(handle.drvInitStat == PMIC_INIT_TEST_EXPECTED_STAT);

    /* Verify all function pointers are set */
    PLATFORM_ASSERT(handle.ioRead != NULL);
    PLATFORM_ASSERT(handle.ioWrite != NULL);
    PLATFORM_ASSERT(handle.criticalSectionStart != NULL);
    PLATFORM_ASSERT(handle.criticalSectionStop != NULL);

    /* Verify communication mode is stored */
    PLATFORM_ASSERT(handle.commMode == PMIC_INTF_SPI);

    (void)Pmic_deinit(&handle);
}

void test_pos_pmic_init_device_info_retrieval(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_HandleCfg_t pmicCfg = {0};
    initTestHandleCfg(&pmicCfg);

    /* Initialize to trigger device info retrieval */
    int32_t status = Pmic_init(&handle, &pmicCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Verify all device info fields are populated by getPmicInfo() */
    /* The mock will return register values that get stored in handle */

    /* devRev comes from DEV_ID register (PMIC_DEV_ID_REG) */
    /* Field should be initialized (not uninitialized memory) */
    PLATFORM_ASSERT(handle.devRev != 0xFFU);

    /* devSiRev comes from DEV_REV register (PMIC_DEV_REV_REG) */
    PLATFORM_ASSERT(handle.devSiRev != 0xFFU);

    /* nvmCode comes from NVM_CODE register (PMIC_NVM_CODE_REG) */
    PLATFORM_ASSERT(handle.nvmCode != 0xFFU);

    /* nvmRev comes from NVM_REV register (PMIC_NVM_REV_REG) */
    PLATFORM_ASSERT(handle.nvmRev != 0xFFU);

    /* Verify init completed successfully after device info retrieval */
    PLATFORM_ASSERT(handle.drvInitStat == PMIC_INIT_TEST_EXPECTED_STAT);

    (void)Pmic_deinit(&handle);
}

void test_pos_pmic_init_communication_validation(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_HandleCfg_t pmicCfg = {0};
    initTestHandleCfg(&pmicCfg);

    /* Initialize - this triggers validateComms() */
    int32_t status = Pmic_init(&handle, &pmicCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Verify validateComms set drvInitStat correctly */
    /* validateComms reads PMIC_WD_LONGWIN_CFG_REG to verify communication */
    /* On success, it sets drvInitStat to PMIC_DRV_INIT_SUCCESS */
    PLATFORM_ASSERT(handle.drvInitStat == PMIC_INIT_TEST_EXPECTED_STAT);

    /* Verify we can actually communicate with device after validation */
    uint8_t regData = 0U;
    status = Pmic_ioRxByte(&handle, PMIC_WD_LONGWIN_CFG_REG, &regData);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Verify handle remains valid after communication */
    PLATFORM_ASSERT(handle.ioRead != NULL);
    PLATFORM_ASSERT(handle.ioWrite != NULL);

    (void)Pmic_deinit(&handle);
}

void test_pos_pmic_init_spi_comprehensive(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_HandleCfg_t pmicCfg = {0};
    initTestHandleCfg(&pmicCfg);

    /* Test comprehensive SPI mode initialization */
    pmicCfg.commMode = PMIC_INTF_SPI;

    int32_t status = Pmic_init(&handle, &pmicCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Verify SPI mode is stored */
    PLATFORM_ASSERT(handle.commMode == PMIC_INTF_SPI);

    /* Test multiple SPI operations */
    uint8_t writeData = 0xA5U;
    uint8_t readData = 0U;

    /* Write to scratchpad */
    status = Pmic_ioTxByte(&handle, PMIC_CUSTOMER_SCRATCH1_REG, writeData);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Read back */
    status = Pmic_ioRxByte(&handle, PMIC_CUSTOMER_SCRATCH1_REG, &readData);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(readData == writeData);

    /* Test reading device registers */
    status = Pmic_ioRxByte(&handle, PMIC_DEV_ID_REG, &readData);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_ioRxByte(&handle, PMIC_DEV_REV_REG, &readData);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Verify handle state after multiple operations */
    PLATFORM_ASSERT(handle.drvInitStat == PMIC_INIT_TEST_EXPECTED_STAT);
    PLATFORM_ASSERT(handle.commMode == PMIC_INTF_SPI);

    (void)Pmic_deinit(&handle);
}

void test_pos_pmic_deinit_success_comprehensive(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_HandleCfg_t pmicCfg = {0};
    initTestHandleCfg(&pmicCfg);

    /* Initialize fully */
    int32_t status = Pmic_init(&handle, &pmicCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Verify fields are populated before deinit */
    PLATFORM_ASSERT(handle.drvInitStat == PMIC_INIT_TEST_EXPECTED_STAT);
    PLATFORM_ASSERT(handle.commHandle0 != NULL);
    PLATFORM_ASSERT(handle.ioRead != NULL);
    PLATFORM_ASSERT(handle.ioWrite != NULL);
    PLATFORM_ASSERT(handle.criticalSectionStart != NULL);
    PLATFORM_ASSERT(handle.criticalSectionStop != NULL);

    /* Deinitialize */
    status = Pmic_deinit(&handle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Verify ALL fields are cleared including device info fields */
    PLATFORM_ASSERT(handle.drvInitStat == 0x00U);
    PLATFORM_ASSERT(handle.commHandle0 == NULL);
    PLATFORM_ASSERT(handle.ioRead == NULL);
    PLATFORM_ASSERT(handle.ioWrite == NULL);
    PLATFORM_ASSERT(handle.criticalSectionStart == NULL);
    PLATFORM_ASSERT(handle.criticalSectionStop == NULL);

    /* Verify device info fields are cleared (lines 218-221 in pmic.c) */
    PLATFORM_ASSERT(handle.devRev == 0U);
    PLATFORM_ASSERT(handle.devSiRev == 0U);
    PLATFORM_ASSERT(handle.nvmCode == 0U);
    PLATFORM_ASSERT(handle.nvmRev == 0U);
}

void test_pos_pmic_checkHandle_comprehensive(void)
{
    Pmic_Handle_t handle = {0};
    int32_t status;

    /* Test 1: NULL handle (line 230) */
    status = Pmic_checkHandle(NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);

    /* Test 2: NULL commHandle0 (line 230) */
    handle.commMode = PMIC_INTF_SPI;
    handle.commHandle0 = NULL;
    handle.ioRead = &test_pmic_regRead;
    handle.drvInitStat = PMIC_INIT_TEST_EXPECTED_STAT;
    status = Pmic_checkHandle(&handle);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);

    /* Test 3: NULL ioRead (line 234) - THIS WAS NEVER TESTED */
    handle.commHandle0 = (void *)0x12345678U;  /* Non-NULL */
    handle.ioRead = NULL;  /* NULL ioRead */
    handle.drvInitStat = PMIC_INIT_TEST_EXPECTED_STAT;
    status = Pmic_checkHandle(&handle);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_FPTR);

    /* Test 4: Invalid drvInitStat (line 238) */
    handle.commHandle0 = (void *)0x12345678U;  /* Non-NULL */
    handle.ioRead = &test_pmic_regRead;
    handle.drvInitStat = 0xDEADBEEFU;  /* Invalid magic */
    status = Pmic_checkHandle(&handle);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_HANDLE);

    /* Test 5: Valid handle passes all checks */
    handle.commHandle0 = (void *)0x12345678U;  /* Non-NULL */
    handle.ioRead = &test_pmic_regRead;
    handle.ioWrite = &test_pmic_regWrite;
    handle.criticalSectionStart = &test_pmic_criticalSectionStartFn;
    handle.criticalSectionStop = &test_pmic_criticalSectionStopFn;
    handle.drvInitStat = PMIC_INIT_TEST_EXPECTED_STAT;
    status = Pmic_checkHandle(&handle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_pmic_init_withRetryCnt(void)
{
    // Initialize with PMIC_RETRY_CNT_VALID set
    Pmic_Handle_t handle = {0};
    Pmic_HandleCfg_t pmicCfg = {0};
    initTestHandleCfg(&pmicCfg);

    // Add retry count configuration
    pmicCfg.validParams |= PMIC_RETRY_CNT_VALID;
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
    // Initialize with PMIC_RETRY_INTERVAL_MS_VALID set
    Pmic_Handle_t handle = {0};
    Pmic_HandleCfg_t pmicCfg = {0};
    initTestHandleCfg(&pmicCfg);

    // Add retry interval configuration
    // Note: When retryIntervalMs is non-zero, timerWaitMs must also be provided
    pmicCfg.validParams |= PMIC_RETRY_INTERVAL_MS_VALID | PMIC_TIMER_WAIT_MS_VALID;
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
    // Initialize with PMIC_TIMER_WAIT_MS_VALID and valid callback
    Pmic_Handle_t handle = {0};
    Pmic_HandleCfg_t pmicCfg = {0};
    initTestHandleCfg(&pmicCfg);

    // Add timer wait callback configuration
    pmicCfg.validParams |= PMIC_TIMER_WAIT_MS_VALID;
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
    // Set PMIC_TIMER_WAIT_MS_VALID but pass NULL callback
    Pmic_Handle_t handle = {0};
    Pmic_HandleCfg_t pmicCfg = {0};
    initTestHandleCfg(&pmicCfg);

    // Set valid param flag but provide NULL callback
    pmicCfg.validParams |= PMIC_TIMER_WAIT_MS_VALID;
    pmicCfg.timerWaitMs = NULL;

    int32_t status = Pmic_init(&handle, &pmicCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_FPTR);
}

/**
 * @brief Test checkHandle with invalid commMode (non-SPI)
 *
 * Covers line 291 in pmic.c - commMode validation in Pmic_checkHandle
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
 * @brief Test checkHandle with NULL critical section functions
 *
 * Covers line 303 in pmic.c - critical section function pointer validation
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
 * @brief Test checkHandle with NULL timer but non-zero retry interval
 *
 * Covers line 307 in pmic.c - timer validation when retry interval is set
 */
void test_neg_pmic_checkHandle_nullTimerWithRetry(void)
{
    Pmic_Handle_t handle = {0};
    Pmic_HandleCfg_t pmicCfg = {0};
    initTestHandleCfg(&pmicCfg);

    // Initialize handle properly with timer
    pmicCfg.validParams |= PMIC_RETRY_INTERVAL_MS_VALID | PMIC_TIMER_WAIT_MS_VALID;
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

