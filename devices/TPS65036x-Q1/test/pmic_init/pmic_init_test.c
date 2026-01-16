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
#define PMIC_INIT_TEST_RUN_ALL() PLATFORM_RUN_TEST(test_negative_Pmic_init_nullParam_pmicHandle); \
                                 PLATFORM_RUN_TEST(test_negative_Pmic_init_nullParam_pmicCfg); \
                                 PLATFORM_RUN_TEST(test_negative_Pmic_init_nullParam_pmicCfg_commHandle); \
                                 PLATFORM_RUN_TEST(test_negative_Pmic_init_nullParam_pmicCfg_ioRead); \
                                 PLATFORM_RUN_TEST(test_negative_Pmic_init_nullParam_pmicCfg_ioWrite); \
                                 PLATFORM_RUN_TEST(test_negative_Pmic_init_nullParam_pmicCfg_critSecStart); \
                                 PLATFORM_RUN_TEST(test_negative_Pmic_init_nullParam_pmicCfg_critSecStop); \
                                 PLATFORM_RUN_TEST(test_negative_Pmic_deinit_nullParam_pmicHandle); \
                                 PLATFORM_RUN_TEST(test_negative_init_timerWaitNull); \
                                 PLATFORM_RUN_TEST(test_negative_init_nullIrqResponseCallback); \
                                 PLATFORM_RUN_TEST(test_negative_checkHandle_nullCommHandle); \
                                 PLATFORM_RUN_TEST(test_negative_checkHandle_nullFptrs); \
                                 PLATFORM_RUN_TEST(test_negative_checkHandle_nullTimerWithRetry); \
                                 PLATFORM_RUN_TEST(test_negative_checkHandle_invalidDrvInitStat); \
                                 PLATFORM_RUN_TEST(test_positive_Pmic_init); \
                                 PLATFORM_RUN_TEST(test_positive_Pmic_deinit); \
                                 PLATFORM_RUN_TEST(test_positive_init_withRetryCnt); \
                                 PLATFORM_RUN_TEST(test_positive_init_withRetryInterval); \
                                 PLATFORM_RUN_TEST(test_positive_init_withTimerWaitMs)

/* Run all PMIC_INIT negative tests */
#define PMIC_INIT_TEST_RUN_NEGATIVE() PLATFORM_RUN_TEST(test_negative_Pmic_init_nullParam_pmicHandle); \
                                      PLATFORM_RUN_TEST(test_negative_Pmic_init_nullParam_pmicCfg); \
                                      PLATFORM_RUN_TEST(test_negative_Pmic_init_nullParam_pmicCfg_commHandle); \
                                      PLATFORM_RUN_TEST(test_negative_Pmic_init_nullParam_pmicCfg_ioRead); \
                                      PLATFORM_RUN_TEST(test_negative_Pmic_init_nullParam_pmicCfg_ioWrite); \
                                      PLATFORM_RUN_TEST(test_negative_Pmic_init_nullParam_pmicCfg_critSecStart); \
                                      PLATFORM_RUN_TEST(test_negative_Pmic_init_nullParam_pmicCfg_critSecStop); \
                                      PLATFORM_RUN_TEST(test_negative_Pmic_deinit_nullParam_pmicHandle); \
                                      PLATFORM_RUN_TEST(test_negative_init_nullIrqResponseCallback); \
                                      PLATFORM_RUN_TEST(test_negative_checkHandle_nullCommHandle); \
                                      PLATFORM_RUN_TEST(test_negative_checkHandle_nullFptrs); \
                                      PLATFORM_RUN_TEST(test_negative_checkHandle_nullTimerWithRetry); \
                                      PLATFORM_RUN_TEST(test_negative_checkHandle_invalidDrvInitStat)

/* Run all PMIC_INIT positive tests */
#define PMIC_INIT_TEST_RUN_POSITIVE() PLATFORM_RUN_TEST(test_positive_Pmic_init); \
                                      PLATFORM_RUN_TEST(test_positive_Pmic_deinit)

/* ========================================================================== */
/*                           Function Declarations                            */
/* ========================================================================== */

static void pmicInitTest_initPmicCfg(Pmic_HandleCfg_t *pmicCfg);
static void pmicInitTest_nullParamPmicInit(const char *param);

/* ========================================================================== */
/*                             Global Variables                               */
/* ========================================================================== */

static Pmic_Handle_t pmicHandle = {0U};

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
    platform_init();

    platform_printString("\r\n");
    platform_printString("PMIC_INIT_TEST\r\n");
    platform_printString("--------------\r\n\r\n");

    platform_setupTests();
    PMIC_INIT_TEST_RUN_ALL();
    platform_tearDownTests();

    platform_deinit();
}

static void pmicInitTest_initPmicCfg(Pmic_HandleCfg_t *pmicCfg)
{
    pmicCfg->validParams = PMIC_I2C_ADDR0_VALID |
                           PMIC_COMM_HANDLE_0_VALID |
                           PMIC_IO_READ_VALID |
                           PMIC_IO_WRITE_VALID |
                           PMIC_CRITICAL_SECTION_START_VALID |
                           PMIC_CRITICAL_SECTION_STOP_VALID |
                           PMIC_IRQ_RESPONSE_CALLBACK_VALID;
    pmicCfg->i2cAddr0 = 0x60U;
    pmicCfg->commHandle0 = platform_getCommHandle();
    pmicCfg->ioRead = &platform_rxByte;
    pmicCfg->ioWrite = &platform_txByte;
    pmicCfg->criticalSectionStart = &platform_critSecStart;
    pmicCfg->criticalSectionStop = &platform_critSecStop;
    pmicCfg->irqResponseCallback = &platform_irqResponse;
}

void test_negative_Pmic_init_nullParam_pmicHandle(void)
{
    // Pass NULL pmicHandle into Pmic_init()
    Pmic_HandleCfg_t pmicCfg = {0U};
    pmicInitTest_initPmicCfg(&pmicCfg);
    int32_t status = Pmic_init(NULL, &pmicCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_init_nullParam_pmicCfg(void)
{
    // Pass NULL pmicCfg into Pmic_init()
    int32_t status = Pmic_init(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

static void pmicInitTest_nullParamPmicInit(const char *param)
{
    Pmic_HandleCfg_t pmicCfg = {0U};
    pmicInitTest_initPmicCfg(&pmicCfg);

    if (strcmp(param, "commHandle0") == 0U) {
        pmicCfg.commHandle0 = NULL;
    } else if (strcmp(param, "ioRead") == 0U) {
        pmicCfg.ioRead = NULL;
    } else if (strcmp(param, "ioWrite") == 0U) {
        pmicCfg.ioWrite = NULL;
    } else if (strcmp(param, "criticalSectionStart") == 0U) {
        pmicCfg.criticalSectionStart = NULL;
    } else if (strcmp(param, "criticalSectionStop") == 0U) {
        pmicCfg.criticalSectionStop = NULL;
    } else {
        PLATFORM_ASSERT(0U);
    }

    int32_t status = Pmic_init(&pmicHandle, &pmicCfg);

    /* Function pointers return PMIC_ST_ERR_NULL_FPTR, other parameters return PMIC_ST_ERR_NULL_PARAM */
    if (strcmp(param, "commHandle0") == 0U) {
        PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
    } else {
        /* ioRead, ioWrite, criticalSectionStart, criticalSectionStop are function pointers */
        PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_FPTR);
    }
}

void test_negative_Pmic_init_nullParam_pmicCfg_commHandle(void)
{
    // Pass NULL commHandle0 into Pmic_init()
    pmicInitTest_nullParamPmicInit("commHandle0");
}

void test_negative_Pmic_init_nullParam_pmicCfg_ioRead(void)
{
    // Pass NULL ioRead into Pmic_init()
    pmicInitTest_nullParamPmicInit("ioRead");
}

void test_negative_Pmic_init_nullParam_pmicCfg_ioWrite(void)
{
    // Pass NULL ioWrite into Pmic_init()
    pmicInitTest_nullParamPmicInit("ioWrite");
}

void test_negative_Pmic_init_nullParam_pmicCfg_critSecStart(void)
{
    // Pass NULL criticalSectionStart into Pmic_init()
    pmicInitTest_nullParamPmicInit("criticalSectionStart");
}

void test_negative_Pmic_init_nullParam_pmicCfg_critSecStop(void)
{
    // Pass NULL criticalSectionStop into Pmic_init()
    pmicInitTest_nullParamPmicInit("criticalSectionStop");
}

void test_negative_Pmic_deinit_nullParam_pmicHandle(void)
{
    // Pass NULL pmicHandle into Pmic_deinit()
    int32_t status = Pmic_deinit(NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_positive_Pmic_init(void)
{
    // Initialize PMIC LLD
    Pmic_HandleCfg_t pmicCfg = {0U};
    pmicInitTest_initPmicCfg(&pmicCfg);
    int32_t status = Pmic_init(&pmicHandle, &pmicCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_positive_Pmic_deinit(void)
{
    // De-initialize PMIC LLD
    int32_t status = Pmic_deinit(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(pmicHandle.drvInitStat == 0U);
    PLATFORM_ASSERT(pmicHandle.i2cAddr0 == 0U);
    PLATFORM_ASSERT(pmicHandle.devRev == 0U);
    PLATFORM_ASSERT(pmicHandle.nvmCode == 0U);
    PLATFORM_ASSERT(pmicHandle.nvmRev == 0U);
    PLATFORM_ASSERT(pmicHandle.devSiRev == 0U);
    PLATFORM_ASSERT(pmicHandle.crcEnable == PMIC_DISABLE);
    PLATFORM_ASSERT(pmicHandle.commHandle0 == NULL);
    PLATFORM_ASSERT(pmicHandle.ioRead == NULL);
    PLATFORM_ASSERT(pmicHandle.ioWrite == NULL);
    PLATFORM_ASSERT(pmicHandle.criticalSectionStart == NULL);
    PLATFORM_ASSERT(pmicHandle.criticalSectionStop == NULL);
    PLATFORM_ASSERT(pmicHandle.irqResponseCallback == NULL);
}

void test_positive_init_withRetryCnt(void)
{
    // Initialize with PMIC_RETRY_CNT_VALID set
    Pmic_HandleCfg_t pmicCfg = {0U};
    Pmic_Handle_t handle = {0U};

    pmicInitTest_initPmicCfg(&pmicCfg);

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

void test_positive_init_withRetryInterval(void)
{
    // Initialize with PMIC_RETRY_INTERVAL_MS_VALID set
    Pmic_HandleCfg_t pmicCfg = {0U};
    Pmic_Handle_t handle = {0U};

    pmicInitTest_initPmicCfg(&pmicCfg);

    // Add retry interval configuration
    // Note: When retryIntervalMs is non-zero, timerWaitMs must also be provided
    pmicCfg.validParams |= PMIC_RETRY_INTERVAL_MS_VALID | PMIC_TIMER_WAIT_MS_VALID;
    pmicCfg.retryIntervalMs = 100U;
    pmicCfg.timerWaitMs = &testTimerWaitWrapper;

    int32_t status = Pmic_init(&handle, &pmicCfg);
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
    Pmic_HandleCfg_t pmicCfg = {0U};
    Pmic_Handle_t handle = {0U};

    pmicInitTest_initPmicCfg(&pmicCfg);

    // Add timer wait callback configuration
    pmicCfg.validParams |= PMIC_TIMER_WAIT_MS_VALID;
    pmicCfg.timerWaitMs = &testTimerWaitWrapper;

    int32_t status = Pmic_init(&handle, &pmicCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify timer wait callback is set in handle
    PLATFORM_ASSERT(handle.timerWaitMs == &testTimerWaitWrapper);

    // Clean up
    status = Pmic_deinit(&handle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_negative_init_timerWaitNull(void)
{
    // Set PMIC_TIMER_WAIT_MS_VALID but pass NULL callback
    Pmic_HandleCfg_t pmicCfg = {0U};
    Pmic_Handle_t handle = {0U};

    pmicInitTest_initPmicCfg(&pmicCfg);

    // Set valid param flag but provide NULL callback
    pmicCfg.validParams |= PMIC_TIMER_WAIT_MS_VALID;
    pmicCfg.timerWaitMs = NULL;

    int32_t status = Pmic_init(&handle, &pmicCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_FPTR);
}

void test_negative_init_nullIrqResponseCallback(void)
{
    // Test NULL IRQ response callback with PMIC_IRQ_RESPONSE_CALLBACK_VALID set (lines 173-174)
    Pmic_HandleCfg_t pmicCfg = {0U};
    Pmic_Handle_t handle = {0U};

    pmicInitTest_initPmicCfg(&pmicCfg);

    // Set valid param flag but provide NULL callback
    pmicCfg.validParams |= PMIC_IRQ_RESPONSE_CALLBACK_VALID;
    pmicCfg.irqResponseCallback = NULL;

    int32_t status = Pmic_init(&handle, &pmicCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_FPTR);
}

void test_negative_checkHandle_nullCommHandle(void)
{
    // Test corrupted commHandle0 to NULL (line 411)
    Pmic_HandleCfg_t pmicCfg = {0U};
    Pmic_Handle_t handle = {0U};

    pmicInitTest_initPmicCfg(&pmicCfg);

    int32_t status = Pmic_init(&handle, &pmicCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Corrupt commHandle0 to NULL
    handle.commHandle0 = NULL;

    // Try to use the corrupted handle - should fail
    bool wdgEnabled = false;
    status = Pmic_wdgGetEnableState(&handle, &wdgEnabled);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);

    // Restore handle for deinit
    handle.commHandle0 = platform_getCommHandle();
    status = Pmic_deinit(&handle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_negative_checkHandle_nullFptrs(void)
{
    // Test corrupted function pointers to NULL (line 417)
    Pmic_HandleCfg_t pmicCfg = {0U};
    Pmic_Handle_t handle = {0U};

    pmicInitTest_initPmicCfg(&pmicCfg);

    int32_t status = Pmic_init(&handle, &pmicCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Corrupt ioRead to NULL
    handle.ioRead = NULL;

    // Try to use the corrupted handle - should fail
    bool wdgEnabled = false;
    status = Pmic_wdgGetEnableState(&handle, &wdgEnabled);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_FPTR);

    // Restore handle for deinit
    handle.ioRead = &platform_rxByte;
    status = Pmic_deinit(&handle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_negative_checkHandle_nullTimerWithRetry(void)
{
    // Test non-zero retry with NULL timer (line 422)
    Pmic_HandleCfg_t pmicCfg = {0U};
    Pmic_Handle_t handle = {0U};

    pmicInitTest_initPmicCfg(&pmicCfg);

    int32_t status = Pmic_init(&handle, &pmicCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Set retryIntervalMs to non-zero but timerWaitMs to NULL
    handle.retryIntervalMs = 100U;
    handle.timerWaitMs = NULL;

    // Try to use the corrupted handle - should fail
    bool wdgEnabled = false;
    status = Pmic_wdgGetEnableState(&handle, &wdgEnabled);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_FPTR);

    // Restore handle for deinit
    handle.retryIntervalMs = 0U;
    status = Pmic_deinit(&handle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_negative_checkHandle_invalidDrvInitStat(void)
{
    // Test corrupted drvInitStat (line 427)
    Pmic_HandleCfg_t pmicCfg = {0U};
    Pmic_Handle_t handle = {0U};

    pmicInitTest_initPmicCfg(&pmicCfg);

    int32_t status = Pmic_init(&handle, &pmicCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Corrupt drvInitStat
    handle.drvInitStat = 0U;

    // Try to use the corrupted handle - should fail
    bool wdgEnabled = false;
    status = Pmic_wdgGetEnableState(&handle, &wdgEnabled);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_HANDLE);

    // Restore handle for deinit
    handle.drvInitStat = PMIC_DRV_INIT_SUCCESS;
    status = Pmic_deinit(&handle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

