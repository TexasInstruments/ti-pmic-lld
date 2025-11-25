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
 * @brief Source file containing definitions to PMIC Init tests.
 */

/* ========================================================================== */
/*                              Include Files                                 */
/* ========================================================================== */

#include "pmic_init_test.h"

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/* Run all PMIC_INIT tests */
#define PMIC_INIT_TEST_RUN_ALL() PLATFORM_RUN_TEST(test_negative_Pmic_init_nullParam_handle); \
                                 PLATFORM_RUN_TEST(test_negative_Pmic_init_nullParam_coreCfg); \
                                 PLATFORM_RUN_TEST(test_negative_Pmic_init_nullParam_coreCfg_pCommHandle); \
                                 PLATFORM_RUN_TEST(test_negative_Pmic_init_nullParam_coreCfg_pFnPmicCommIoRd); \
                                 PLATFORM_RUN_TEST(test_negative_Pmic_init_nullParam_coreCfg_pFnPmicCommIoWr); \
                                 PLATFORM_RUN_TEST(test_negative_Pmic_init_nullParam_coreCfg_pFnPmicCritSecStart); \
                                 PLATFORM_RUN_TEST(test_negative_Pmic_init_nullParam_coreCfg_pFnPmicCritSecStop); \
                                 PLATFORM_RUN_TEST(test_negative_Pmic_init_nullParam_coreCfg_irqResponseCallback); \
                                 PLATFORM_RUN_TEST(test_negative_Pmic_init_incorrect_coreCfg_commMode); \
                                 PLATFORM_RUN_TEST(test_negative_Pmic_deinit_nullParam_handle); \
                                 PLATFORM_RUN_TEST(test_negative_Pmic_checkPmicCoreHandle_nullParam_handle); \
                                 PLATFORM_RUN_TEST(test_negative_Pmic_checkPmicCoreHandle_nullParam_commHandle0); \
                                 PLATFORM_RUN_TEST(test_negative_Pmic_checkPmicCoreHandle_nullParam_ioRead); \
                                 PLATFORM_RUN_TEST(test_negative_Pmic_checkPmicCoreHandle_incorrect_drvInitStatus); \
                                 PLATFORM_RUN_TEST(test_positive_Pmic_init); \
                                 PLATFORM_RUN_TEST(test_positive_Pmic_checkPmicCoreHandle); \
                                 PLATFORM_RUN_TEST(test_positive_Pmic_deinit)

/* Run all PMIC_INIT negative tests */
#define PMIC_INIT_TEST_RUN_NEGATIVE() PLATFORM_RUN_TEST(test_negative_Pmic_init_nullParam_handle); \
                                      PLATFORM_RUN_TEST(test_negative_Pmic_init_nullParam_coreCfg); \
                                      PLATFORM_RUN_TEST(test_negative_Pmic_init_nullParam_coreCfg_pCommHandle); \
                                           PLATFORM_RUN_TEST(test_negative_Pmic_init_nullParam_coreCfg_pFnPmicCommIoRd); \
                                      PLATFORM_RUN_TEST(test_negative_Pmic_init_nullParam_coreCfg_pFnPmicCommIoWr); \
                                      PLATFORM_RUN_TEST(test_negative_Pmic_init_nullParam_coreCfg_pFnPmicCritSecStart); \
                                      PLATFORM_RUN_TEST(test_negative_Pmic_init_nullParam_coreCfg_pFnPmicCritSecStop); \
                                      PLATFORM_RUN_TEST(test_negative_Pmic_init_nullParam_coreCfg_irqResponseCallback); \
                                      PLATFORM_RUN_TEST(test_negative_Pmic_init_incorrect_coreCfg_commMode); \
                                      PLATFORM_RUN_TEST(test_negative_Pmic_deinit_nullParam_handle); \
                                      PLATFORM_RUN_TEST(test_negative_Pmic_checkPmicCoreHandle_nullParam_handle); \
                                      PLATFORM_RUN_TEST(test_negative_Pmic_checkPmicCoreHandle_nullParam_commHandle0); \
                                      PLATFORM_RUN_TEST(test_negative_Pmic_checkPmicCoreHandle_nullParam_ioRead); \
                                      PLATFORM_RUN_TEST(test_negative_Pmic_checkPmicCoreHandle_incorrect_drvInitStatus)

/* Run all PMIC_INIT positive tests */
#define PMIC_INIT_TEST_RUN_POSITIVE() PLATFORM_RUN_TEST(test_positive_Pmic_init); \
                                      PLATFORM_RUN_TEST(test_positive_Pmic_checkPmicCoreHandle); \
                                      PLATFORM_RUN_TEST(test_positive_Pmic_deinit)

/* Used in certain unit tests to validate driver initialization status */
#define PMIC_INIT_TEST_DRV_INIT_STATUS (0x504D4943U) /* "PMIC" in ASCII */

/* Arbitrary value used for testing purposes */
#define PMIC_INIT_TEST_ARBITARY_VALUE (0xAAU)

/* ========================================================================== */
/*                           Function Declarations                            */
/* ========================================================================== */
static inline void pmicInitTest_initCoreCfg(Pmic_HandleCfg_t *coreCfg);

/* ========================================================================== */
/*                             Global Variables                               */
/* ========================================================================== */
static Pmic_Handle_t pmicHandle = {0};

/* ========================================================================== */
/*                           Function Definitions                             */
/* ========================================================================== */

void pmic_init_test(void *args)
{
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

void test_negative_Pmic_init_nullParam_handle(void)
{
    // Pass NULL handle into Pmic_init()
    Pmic_HandleCfg_t coreCfg = {0};
    int32_t status = Pmic_init(NULL, &coreCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_init_nullParam_coreCfg(void)
{
    // Pass NULL coreCfg into Pmic_init()
    Pmic_Handle_t handle = {0};
    int32_t status = Pmic_init(&handle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

static inline void pmicInitTest_initCoreCfg(Pmic_HandleCfg_t *coreCfg)
{
    coreCfg->validParams = PMIC_COMM_MODE_VALID |
                           PMIC_I2C_ADDR0_VALID |
                           PMIC_CFG_I2CADDR1_VALID |
                           PMIC_CFG_I2CADDR2_VALID |
                           PMIC_CRC_ENABLE_VALID |
                           PMIC_CONFIG_CRC_ENABLE_VALID |
                           PMIC_COMM_HANDLE_0_VALID |
                           PMIC_IO_READ_VALID |
                           PMIC_IO_WRITE_VALID |
                           PMIC_CRITICAL_SECTION_START_VALID |
                           PMIC_CRITICAL_SECTION_STOP_VALID |
                           PMIC_IRQ_RESPONSE_CALLBACK_VALID;
    coreCfg->commMode = PMIC_INTF_I2C_SINGLE;
    coreCfg->i2cAddr0 = PLATFORM_TARGET_I2C_ADDR;
    coreCfg->i2cAddr1 = PMIC_INIT_TEST_ARBITARY_VALUE;   // Not needed to be specified for Coach
    coreCfg->i2cAddr2 = PMIC_INIT_TEST_ARBITARY_VALUE;  // Not needed to be specified for Coach
    coreCfg->crcEnable = PMIC_DISABLE;
    coreCfg->configCrcEnable = PMIC_DISABLE;
    coreCfg->commHandle0 = platform_getCommHandle();
    coreCfg->ioRead = &platform_rxByte;
    coreCfg->ioWrite = &platform_txByte;
    coreCfg->criticalSectionStart = &platform_critSecStart;
    coreCfg->criticalSectionStop = &platform_critSecStop;
    coreCfg->irqResponseCallback = &platform_irqResponse;
}

void test_negative_Pmic_init_nullParam_coreCfg_commHandle0(void)
{
    Pmic_HandleCfg_t coreCfg = {0};
    Pmic_Handle_t handle = {0};

    pmicInitTest_initCoreCfg(&coreCfg);

    // Pass NULL commHandle0 into Pmic_init()
    coreCfg.commHandle0 = NULL;
    int32_t status = Pmic_init(&handle, &coreCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_init_nullParam_coreCfg_ioRead(void)
{
    Pmic_HandleCfg_t coreCfg = {0};
    Pmic_Handle_t handle = {0};

    pmicInitTest_initCoreCfg(&coreCfg);

    // Pass NULL ioRead into Pmic_init()
    coreCfg.ioRead = NULL;
    int32_t status = Pmic_init(&handle, &coreCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_FPTR);
}

void test_negative_Pmic_init_nullParam_coreCfg_ioWrite(void)
{
    Pmic_HandleCfg_t coreCfg = {0};
    Pmic_Handle_t handle = {0};

    pmicInitTest_initCoreCfg(&coreCfg);

    // Pass NULL ioWrite into Pmic_init()
    coreCfg.ioWrite = NULL;
    int32_t status = Pmic_init(&handle, &coreCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_FPTR);
}

void test_negative_Pmic_init_nullParam_coreCfg_criticalSectionStart(void)
{
    Pmic_HandleCfg_t coreCfg = {0};
    Pmic_Handle_t handle = {0};

    pmicInitTest_initCoreCfg(&coreCfg);

    // Pass NULL criticalSectionStart into Pmic_init()
    coreCfg.criticalSectionStart = NULL;
    int32_t status = Pmic_init(&handle, &coreCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_FPTR);
}

void test_negative_Pmic_init_nullParam_coreCfg_criticalSectionStop(void)
{
    Pmic_HandleCfg_t coreCfg = {0};
    Pmic_Handle_t handle = {0};

    pmicInitTest_initCoreCfg(&coreCfg);

    // Pass NULL criticalSectionStop into Pmic_init()
    coreCfg.criticalSectionStop = NULL;
    int32_t status = Pmic_init(&handle, &coreCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_FPTR);
}

void test_negative_Pmic_init_nullParam_coreCfg_irqResponseCallback(void)
{
    Pmic_HandleCfg_t coreCfg = {0};
    Pmic_Handle_t handle = {0};

    pmicInitTest_initCoreCfg(&coreCfg);

    // Pass NULL irqResponseCallback into Pmic_init()
    coreCfg.irqResponseCallback = NULL;
    int32_t status = Pmic_init(&handle, &coreCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_FPTR);
}

void test_negative_Pmic_init_incorrect_coreCfg_commMode(void)
{
    Pmic_HandleCfg_t coreCfg = {0};
    Pmic_Handle_t handle = {0};

    pmicInitTest_initCoreCfg(&coreCfg);

    // Pass incorrect commMode into Pmic_init()
    coreCfg.commMode = PMIC_INTF_MAX + 1U;
    int32_t status = Pmic_init(&handle, &coreCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_negative_Pmic_deinit_nullParam_handle(void)
{
    // Pass NULL handle into Pmic_deinit()
    int32_t status = Pmic_deinit(NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_HANDLE);
}

void test_negative_Pmic_checkPmicCoreHandle_nullParam_handle(void)
{
    // Pass NULL handle into Pmic_checkHandle()
    int32_t status = Pmic_checkHandle(NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_HANDLE);
}

void test_negative_Pmic_checkPmicCoreHandle_nullParam_commHandle0(void)
{
    Pmic_Handle_t handle = {
        .drvInitStat = (uint32_t)(PMIC_INIT_TEST_DRV_INIT_STATUS | (uint8_t)PMIC_MAIN_INST),
        .commMode = PMIC_INTF_I2C_SINGLE,
        .i2cAddr0 = PLATFORM_TARGET_I2C_ADDR,
        .crcEnable = PMIC_DISABLE,
        .configCrcEnable = PMIC_DISABLE,
        .commHandle0 = NULL,
        .ioRead = &platform_rxByte,
        .ioWrite = &platform_txByte,
        .criticalSectionStart = &platform_critSecStart,
        .criticalSectionStop = &platform_critSecStop,
        .irqResponseCallback = &platform_irqResponse
    };

    // Pass NULL commHandle0 into Pmic_checkPmicCoreHandle()
    int32_t status = Pmic_checkHandle(&handle);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_HANDLE);
}

void test_negative_Pmic_checkPmicCoreHandle_nullParam_ioRead(void)
{
    Pmic_Handle_t handle = {
        .drvInitStat = (uint32_t)(PMIC_INIT_TEST_DRV_INIT_STATUS | (uint8_t)PMIC_MAIN_INST),
        .commMode = PMIC_INTF_I2C_SINGLE,
        .i2cAddr0 = PLATFORM_TARGET_I2C_ADDR,
        .crcEnable = PMIC_DISABLE,
        .configCrcEnable = PMIC_DISABLE,
        .commHandle0 = platform_getCommHandle(),
        .ioRead = NULL,
        .ioWrite = &platform_txByte,
        .criticalSectionStart = &platform_critSecStart,
        .criticalSectionStop = &platform_critSecStop,
        .irqResponseCallback = &platform_irqResponse
    };

    // Pass NULL ioRead into Pmic_checkPmicCoreHandle()
    int32_t status = Pmic_checkHandle(&handle);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_FPTR);
}

void test_negative_Pmic_checkPmicCoreHandle_incorrect_drvInitStatus(void)
{
    // Pass incorrect/corrupted drvInitStatus into Pmic_checkPmicCoreHandle()
    Pmic_Handle_t handle = {
        .drvInitStat = 0x00U,
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
    int32_t status = Pmic_checkHandle(&handle);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_HANDLE);
}

void test_positive_Pmic_init(void)
{
    // Initialize PMIC LLD
    Pmic_HandleCfg_t coreCfg = {0};
    pmicInitTest_initCoreCfg(&coreCfg);
    int32_t status = Pmic_init(&pmicHandle, &coreCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(pmicHandle.drvInitStat == (uint32_t)(PMIC_INIT_TEST_DRV_INIT_STATUS | (uint8_t)PMIC_MAIN_INST));
    PLATFORM_ASSERT(pmicHandle.commMode == PMIC_INTF_I2C_SINGLE);
    PLATFORM_ASSERT(pmicHandle.i2cAddr0 == PLATFORM_TARGET_I2C_ADDR);
    PLATFORM_ASSERT(pmicHandle.i2cAddr1 == PMIC_INIT_TEST_ARBITARY_VALUE);
    PLATFORM_ASSERT(pmicHandle.i2cAddr2 == PMIC_INIT_TEST_ARBITARY_VALUE);
    PLATFORM_ASSERT(pmicHandle.crcEnable == PMIC_DISABLE);
    PLATFORM_ASSERT(pmicHandle.configCrcEnable == PMIC_DISABLE);
    PLATFORM_ASSERT(pmicHandle.commHandle0 == platform_getCommHandle());
    PLATFORM_ASSERT(pmicHandle.ioRead == &platform_rxByte);
    PLATFORM_ASSERT(pmicHandle.ioWrite == &platform_txByte);
    PLATFORM_ASSERT(pmicHandle.criticalSectionStart == &platform_critSecStart);
    PLATFORM_ASSERT(pmicHandle.criticalSectionStop == &platform_critSecStop);
    PLATFORM_ASSERT(pmicHandle.irqResponseCallback == &platform_irqResponse);
}

void test_positive_Pmic_checkPmicCoreHandle(void)
{
    // Check PMIC core handle
    int32_t status = Pmic_checkHandle(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_positive_Pmic_deinit(void)
{
    // Deinitialize PMIC LLD
    int32_t status = Pmic_deinit(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(pmicHandle.drvInitStat == 0U);
    PLATFORM_ASSERT(pmicHandle.devRev == 0U);
    PLATFORM_ASSERT(pmicHandle.devSiRev == 0U);
    PLATFORM_ASSERT(pmicHandle.commMode == 0U);
    PLATFORM_ASSERT(pmicHandle.i2cAddr0 == 0U);
    PLATFORM_ASSERT(pmicHandle.i2cAddr1 == 0U);
    PLATFORM_ASSERT(pmicHandle.i2cAddr2 == 0U);
    PLATFORM_ASSERT(pmicHandle.crcEnable == PMIC_DISABLE);
    PLATFORM_ASSERT(pmicHandle.configCrcEnable == PMIC_DISABLE);
    PLATFORM_ASSERT(pmicHandle.commHandle0 == NULL);
    PLATFORM_ASSERT(pmicHandle.ioRead == NULL);
    PLATFORM_ASSERT(pmicHandle.ioWrite == NULL);
    PLATFORM_ASSERT(pmicHandle.criticalSectionStart == NULL);
    PLATFORM_ASSERT(pmicHandle.criticalSectionStop == NULL);
    PLATFORM_ASSERT(pmicHandle.irqResponseCallback == NULL);
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
