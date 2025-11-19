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
                                 PLATFORM_RUN_TEST(test_negative_Pmic_init_nullParam_coreCfg_pQACommHandle); \
                                 PLATFORM_RUN_TEST(test_negative_Pmic_init_nullParam_coreCfg_pFnPmicCommIoRd); \
                                 PLATFORM_RUN_TEST(test_negative_Pmic_init_nullParam_coreCfg_pFnPmicCommIoWr); \
                                 PLATFORM_RUN_TEST(test_negative_Pmic_init_nullParam_coreCfg_pFnPmicCritSecStart); \
                                 PLATFORM_RUN_TEST(test_negative_Pmic_init_nullParam_coreCfg_pFnPmicCritSecStop); \
                                 PLATFORM_RUN_TEST(test_negative_Pmic_init_nullParam_coreCfg_irqResponseCallback); \
                                 PLATFORM_RUN_TEST(test_negative_Pmic_init_incorrect_coreCfg_instType); \
                                 PLATFORM_RUN_TEST(test_negative_Pmic_init_incorrect__coreCfg_pmicDeviceType); \
                                 PLATFORM_RUN_TEST(test_negative_Pmic_init_incorrect_coreCfg_commMode); \
                                 PLATFORM_RUN_TEST(test_negative_Pmic_deinit_nullParam_handle); \
                                 PLATFORM_RUN_TEST(test_negative_Pmic_checkPmicCoreHandle_nullParam_handle); \
                                 PLATFORM_RUN_TEST(test_negative_Pmic_checkPmicCoreHandle_nullParam_pCommHandle); \
                                 PLATFORM_RUN_TEST(test_negative_Pmic_checkPmicCoreHandle_nullParam_pFnPmicCommIoRd); \
                                 PLATFORM_RUN_TEST(test_negative_Pmic_checkPmicCoreHandle_incorrect_drvInitStatus); \
                                 PLATFORM_RUN_TEST(test_positive_Pmic_init); \
                                 PLATFORM_RUN_TEST(test_positive_Pmic_checkPmicCoreHandle); \
                                 PLATFORM_RUN_TEST(test_positive_Pmic_deinit)

/* Run all PMIC_INIT negative tests */
#define PMIC_INIT_TEST_RUN_NEGATIVE() PLATFORM_RUN_TEST(test_negative_Pmic_init_nullParam_handle); \
                                      PLATFORM_RUN_TEST(test_negative_Pmic_init_nullParam_coreCfg); \
                                      PLATFORM_RUN_TEST(test_negative_Pmic_init_nullParam_coreCfg_pCommHandle); \
                                      PLATFORM_RUN_TEST(test_negative_Pmic_init_nullParam_coreCfg_pQACommHandle); \
                                      PLATFORM_RUN_TEST(test_negative_Pmic_init_nullParam_coreCfg_pFnPmicCommIoRd); \
                                      PLATFORM_RUN_TEST(test_negative_Pmic_init_nullParam_coreCfg_pFnPmicCommIoWr); \
                                      PLATFORM_RUN_TEST(test_negative_Pmic_init_nullParam_coreCfg_pFnPmicCritSecStart); \
                                      PLATFORM_RUN_TEST(test_negative_Pmic_init_nullParam_coreCfg_pFnPmicCritSecStop); \
                                      PLATFORM_RUN_TEST(test_negative_Pmic_init_nullParam_coreCfg_irqResponseCallback); \
                                      PLATFORM_RUN_TEST(test_negative_Pmic_init_incorrect_coreCfg_instType); \
                                      PLATFORM_RUN_TEST(test_negative_Pmic_init_incorrect__coreCfg_pmicDeviceType); \
                                      PLATFORM_RUN_TEST(test_negative_Pmic_init_incorrect_coreCfg_commMode); \
                                      PLATFORM_RUN_TEST(test_negative_Pmic_deinit_nullParam_handle); \
                                      PLATFORM_RUN_TEST(test_negative_Pmic_checkPmicCoreHandle_nullParam_handle); \
                                      PLATFORM_RUN_TEST(test_negative_Pmic_checkPmicCoreHandle_nullParam_pCommHandle); \
                                      PLATFORM_RUN_TEST(test_negative_Pmic_checkPmicCoreHandle_nullParam_pFnPmicCommIoRd); \
                                      PLATFORM_RUN_TEST(test_negative_Pmic_checkPmicCoreHandle_incorrect_drvInitStatus)

/* Run all PMIC_INIT positive tests */
#define PMIC_INIT_TEST_RUN_POSITIVE() PLATFORM_RUN_TEST(test_positive_Pmic_init); \
                                      PLATFORM_RUN_TEST(test_positive_Pmic_checkPmicCoreHandle); \
                                      PLATFORM_RUN_TEST(test_positive_Pmic_deinit)

/* Used in certain unit tests to validate driver initialization status */
#define PMIC_INIT_TEST_DRV_INIT_STATUS (0xBEEF0000U)

/* Arbitrary value used for testing purposes */
#define PMIC_INIT_TEST_ARBITARY_VALUE (0xAAU)

/* ========================================================================== */
/*                           Function Declarations                            */
/* ========================================================================== */
static inline void pmicInitTest_initCoreCfg(Pmic_CoreCfg_t *coreCfg);

/* ========================================================================== */
/*                             Global Variables                               */
/* ========================================================================== */
static Pmic_CoreHandle_t pmicHandle = {0};

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
    Pmic_CoreCfg_t coreCfg = {0};
    int32_t status = Pmic_init(NULL, &coreCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_init_nullParam_coreCfg(void)
{
    // Pass NULL coreCfg into Pmic_init()
    Pmic_CoreHandle_t handle = {0};
    int32_t status = Pmic_init(&handle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

static inline void pmicInitTest_initCoreCfg(Pmic_CoreCfg_t *coreCfg)
{
    coreCfg->validParams = PMIC_CFG_DEVICE_TYPE_VALID_SHIFT |
                           PMIC_CFG_COMM_MODE_VALID_SHIFT |
                           PMIC_CFG_SLAVEADDR_VALID_SHIFT |
                           PMIC_CFG_QASLAVEADDR_VALID_SHIFT |
                           PMIC_CFG_NVMSLAVEADDR_VALID_SHIFT |
                           PMIC_CFG_I2C1_SPEED_VALID_SHIFT |
                           PMIC_CFG_I2C2_SPEED_VALID_SHIFT |
                           PMIC_CFG_CRC_ENABLE_VALID_SHIFT |
                           PMIC_CFG_CFG_CRC_ENABLE_VALID_SHIFT |
                           PMIC_CFG_COMM_HANDLE_VALID_SHIFT |
                           PMIC_CFG_QACOMM_HANDLE_VALID_SHIFT |
                           PMIC_CFG_COMM_IO_RD_VALID_SHIFT |
                           PMIC_CFG_COMM_IO_WR_VALID_SHIFT |
                           PMIC_CFG_CRITSEC_START_VALID_SHIFT |
                           PMIC_CFG_CRITSEC_STOP_VALID_SHIFT |
                           PMIC_CFG_PSEUDO_IRQ_VALID_SHIFT;
    coreCfg->instType = PMIC_MAIN_INST;
    coreCfg->pmicDeviceType = PMIC_DEV_COACH_LP8772X;
    coreCfg->commMode = PMIC_INTF_I2C_SINGLE;
    coreCfg->slaveAddr = PLATFORM_TARGET_I2C_ADDR;
    coreCfg->qaSlaveAddr = PMIC_INIT_TEST_ARBITARY_VALUE;   // Not needed to be specified for Coach
    coreCfg->nvmSlaveAddr = PMIC_INIT_TEST_ARBITARY_VALUE;  // Not needed to be specified for Coach
    coreCfg->i2c1Speed = PMIC_INIT_TEST_ARBITARY_VALUE;     // Not needed to be specified for Coach
    coreCfg->i2c2Speed = PMIC_INIT_TEST_ARBITARY_VALUE;     // Not needed to be specified for Coach
    coreCfg->crcEnable = PMIC_DISABLE;
    coreCfg->configCrcEnable = PMIC_DISABLE;
    coreCfg->pCommHandle = platform_getCommHandle();
    coreCfg->pQACommHandle = platform_getCommHandle();      // Not needed to be specified for Coach
    coreCfg->pFnPmicCommIoRd = &platform_rxByte;
    coreCfg->pFnPmicCommIoWr = &platform_txByte;
    coreCfg->pFnPmicCritSecStart = &platform_critSecStart;
    coreCfg->pFnPmicCritSecStop = &platform_critSecStop;
    coreCfg->irqResponseCallback = &platform_irqResponse;
}

void test_negative_Pmic_init_nullParam_coreCfg_pCommHandle(void)
{
    Pmic_CoreCfg_t coreCfg = {0};
    Pmic_CoreHandle_t handle = {0};

    pmicInitTest_initCoreCfg(&coreCfg);

    // Pass NULL pCommHandle into Pmic_init()
    coreCfg.pCommHandle = NULL;
    int32_t status = Pmic_init(&handle, &coreCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_init_nullParam_coreCfg_pQACommHandle(void)
{
    Pmic_CoreCfg_t coreCfg = {0};
    Pmic_CoreHandle_t handle = {0};

    pmicInitTest_initCoreCfg(&coreCfg);

    // Pass NULL pQACommHandle into Pmic_init()
    coreCfg.pQACommHandle = NULL;
    int32_t status = Pmic_init(&handle, &coreCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_init_nullParam_coreCfg_pFnPmicCommIoRd(void)
{
    Pmic_CoreCfg_t coreCfg = {0};
    Pmic_CoreHandle_t handle = {0};

    pmicInitTest_initCoreCfg(&coreCfg);

    // Pass NULL pFnPmicCommIoRd into Pmic_init()
    coreCfg.pFnPmicCommIoRd = NULL;
    int32_t status = Pmic_init(&handle, &coreCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_FPTR);
}

void test_negative_Pmic_init_nullParam_coreCfg_pFnPmicCommIoWr(void)
{
    Pmic_CoreCfg_t coreCfg = {0};
    Pmic_CoreHandle_t handle = {0};

    pmicInitTest_initCoreCfg(&coreCfg);

    // Pass NULL pFnPmicCommIoWr into Pmic_init()
    coreCfg.pFnPmicCommIoWr = NULL;
    int32_t status = Pmic_init(&handle, &coreCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_FPTR);
}

void test_negative_Pmic_init_nullParam_coreCfg_pFnPmicCritSecStart(void)
{
    Pmic_CoreCfg_t coreCfg = {0};
    Pmic_CoreHandle_t handle = {0};

    pmicInitTest_initCoreCfg(&coreCfg);

    // Pass NULL pFnPmicCritSecStart into Pmic_init()
    coreCfg.pFnPmicCritSecStart = NULL;
    int32_t status = Pmic_init(&handle, &coreCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_FPTR);
}

void test_negative_Pmic_init_nullParam_coreCfg_pFnPmicCritSecStop(void)
{
    Pmic_CoreCfg_t coreCfg = {0};
    Pmic_CoreHandle_t handle = {0};

    pmicInitTest_initCoreCfg(&coreCfg);

    // Pass NULL pFnPmicCritSecStop into Pmic_init()
    coreCfg.pFnPmicCritSecStop = NULL;
    int32_t status = Pmic_init(&handle, &coreCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_FPTR);
}

void test_negative_Pmic_init_nullParam_coreCfg_irqResponseCallback(void)
{
    Pmic_CoreCfg_t coreCfg = {0};
    Pmic_CoreHandle_t handle = {0};

    pmicInitTest_initCoreCfg(&coreCfg);

    // Pass NULL irqResponseCallback into Pmic_init()
    coreCfg.irqResponseCallback = NULL;
    int32_t status = Pmic_init(&handle, &coreCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_FPTR);
}

void test_negative_Pmic_init_incorrect_coreCfg_instType(void)
{
    Pmic_CoreCfg_t coreCfg = {0};
    Pmic_CoreHandle_t handle = {0};

    pmicInitTest_initCoreCfg(&coreCfg);

    // Pass incorrect instType into Pmic_init()
    coreCfg.instType = PMIC_INST_TYPE_MAX + 1U;
    int32_t status = Pmic_init(&handle, &coreCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_HANDLE);
}

void test_negative_Pmic_init_incorrect__coreCfg_pmicDeviceType(void)
{
    Pmic_CoreCfg_t coreCfg = {0};
    Pmic_CoreHandle_t handle = {0};

    pmicInitTest_initCoreCfg(&coreCfg);

    // Pass incorrect pmicDeviceType into Pmic_init()
    coreCfg.pmicDeviceType = PMIC_DEV_COACH_LP8772X + 1U;
    int32_t status = Pmic_init(&handle, &coreCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_negative_Pmic_init_incorrect_coreCfg_commMode(void)
{
    Pmic_CoreCfg_t coreCfg = {0};
    Pmic_CoreHandle_t handle = {0};

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
    // Pass NULL handle into Pmic_checkPmicCoreHandle()
    int32_t status = Pmic_checkPmicCoreHandle(NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_HANDLE);
}

void test_negative_Pmic_checkPmicCoreHandle_nullParam_pCommHandle(void)
{
    Pmic_CoreHandle_t handle = {
        .drvInitStatus = (uint32_t)(PMIC_INIT_TEST_DRV_INIT_STATUS | (uint8_t)PMIC_MAIN_INST),
        .pmicDeviceType = PMIC_DEV_COACH_LP8772X,
        .commMode = PMIC_INTF_I2C_SINGLE,
        .slaveAddr = PLATFORM_TARGET_I2C_ADDR,
        .crcEnable = PMIC_DISABLE,
        .configCrcEnable = PMIC_DISABLE,
        .pCommHandle = NULL,
        .pFnPmicCommIoRd = &platform_rxByte,
        .pFnPmicCommIoWr = &platform_txByte,
        .pFnPmicCritSecStart = &platform_critSecStart,
        .pFnPmicCritSecStop = &platform_critSecStop,
        .irqResponseCallback = &platform_irqResponse
    };

    // Pass NULL pCommHandle into Pmic_checkPmicCoreHandle()
    int32_t status = Pmic_checkPmicCoreHandle(&handle);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_HANDLE);
}

void test_negative_Pmic_checkPmicCoreHandle_nullParam_pFnPmicCommIoRd(void)
{
    Pmic_CoreHandle_t handle = {
        .drvInitStatus = (uint32_t)(PMIC_INIT_TEST_DRV_INIT_STATUS | (uint8_t)PMIC_MAIN_INST),
        .pmicDeviceType = PMIC_DEV_COACH_LP8772X,
        .commMode = PMIC_INTF_I2C_SINGLE,
        .slaveAddr = PLATFORM_TARGET_I2C_ADDR,
        .crcEnable = PMIC_DISABLE,
        .configCrcEnable = PMIC_DISABLE,
        .pCommHandle = platform_getCommHandle(),
        .pFnPmicCommIoRd = NULL,
        .pFnPmicCommIoWr = &platform_txByte,
        .pFnPmicCritSecStart = &platform_critSecStart,
        .pFnPmicCritSecStop = &platform_critSecStop,
        .irqResponseCallback = &platform_irqResponse
    };

    // Pass NULL pFnPmicCommIoRd into Pmic_checkPmicCoreHandle()
    int32_t status = Pmic_checkPmicCoreHandle(&handle);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_FPTR);
}

void test_negative_Pmic_checkPmicCoreHandle_incorrect_drvInitStatus(void)
{
    // Pass incorrect/corrupted drvInitStatus into Pmic_checkPmicCoreHandle()
    Pmic_CoreHandle_t handle = {
        .drvInitStatus = 0x00U,
        .pmicDeviceType = PMIC_DEV_COACH_LP8772X,
        .commMode = PMIC_INTF_I2C_SINGLE,
        .slaveAddr = PLATFORM_TARGET_I2C_ADDR,
        .crcEnable = PMIC_DISABLE,
        .configCrcEnable = PMIC_DISABLE,
        .pCommHandle = platform_getCommHandle(),
        .pFnPmicCommIoRd = &platform_rxByte,
        .pFnPmicCommIoWr = &platform_txByte,
        .pFnPmicCritSecStart = &platform_critSecStart,
        .pFnPmicCritSecStop = &platform_critSecStop,
        .irqResponseCallback = &platform_irqResponse
    };
    int32_t status = Pmic_checkPmicCoreHandle(&handle);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_HANDLE);
}

void test_positive_Pmic_init(void)
{
    // Initialize PMIC LLD
    Pmic_CoreCfg_t coreCfg = {0};
    pmicInitTest_initCoreCfg(&coreCfg);
    int32_t status = Pmic_init(&pmicHandle, &coreCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(pmicHandle.drvInitStatus == (uint32_t)(PMIC_INIT_TEST_DRV_INIT_STATUS | (uint8_t)PMIC_MAIN_INST));
    PLATFORM_ASSERT(pmicHandle.pmicDeviceType == PMIC_DEV_COACH_LP8772X);
    PLATFORM_ASSERT(pmicHandle.commMode == PMIC_INTF_I2C_SINGLE);
    PLATFORM_ASSERT(pmicHandle.slaveAddr == PLATFORM_TARGET_I2C_ADDR);
    PLATFORM_ASSERT(pmicHandle.qaSlaveAddr == PMIC_INIT_TEST_ARBITARY_VALUE);
    PLATFORM_ASSERT(pmicHandle.nvmSlaveAddr == PMIC_INIT_TEST_ARBITARY_VALUE);
    PLATFORM_ASSERT(pmicHandle.i2c1Speed == PMIC_INIT_TEST_ARBITARY_VALUE);
    PLATFORM_ASSERT(pmicHandle.i2c2Speed == PMIC_INIT_TEST_ARBITARY_VALUE);
    PLATFORM_ASSERT(pmicHandle.crcEnable == PMIC_DISABLE);
    PLATFORM_ASSERT(pmicHandle.configCrcEnable == PMIC_DISABLE);
    PLATFORM_ASSERT(pmicHandle.pCommHandle == platform_getCommHandle());
    PLATFORM_ASSERT(pmicHandle.pQACommHandle == platform_getCommHandle());
    PLATFORM_ASSERT(pmicHandle.pFnPmicCommIoRd == &platform_rxByte);
    PLATFORM_ASSERT(pmicHandle.pFnPmicCommIoWr == &platform_txByte);
    PLATFORM_ASSERT(pmicHandle.pFnPmicCritSecStart == &platform_critSecStart);
    PLATFORM_ASSERT(pmicHandle.pFnPmicCritSecStop == &platform_critSecStop);
    PLATFORM_ASSERT(pmicHandle.irqResponseCallback == &platform_irqResponse);
}

void test_positive_Pmic_checkPmicCoreHandle(void)
{
    // Check PMIC core handle
    int32_t status = Pmic_checkPmicCoreHandle(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_positive_Pmic_deinit(void)
{
    // Deinitialize PMIC LLD
    int32_t status = Pmic_deinit(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(pmicHandle.pPmic_SubSysInfo == NULL);
    PLATFORM_ASSERT(pmicHandle.drvInitStatus == 0U);
    PLATFORM_ASSERT(pmicHandle.pmicDeviceType == 0U);
    PLATFORM_ASSERT(pmicHandle.pmicDevRev == 0U);
    PLATFORM_ASSERT(pmicHandle.pmicDevSiliconRev == 0U);
    PLATFORM_ASSERT(pmicHandle.commMode == 0U);
    PLATFORM_ASSERT(pmicHandle.slaveAddr == 0U);
    PLATFORM_ASSERT(pmicHandle.qaSlaveAddr == 0U);
    PLATFORM_ASSERT(pmicHandle.nvmSlaveAddr == 0U);
    PLATFORM_ASSERT(pmicHandle.i2c1Speed == 0U);
    PLATFORM_ASSERT(pmicHandle.i2c2Speed == 0U);
    PLATFORM_ASSERT(pmicHandle.crcEnable == PMIC_DISABLE);
    PLATFORM_ASSERT(pmicHandle.configCrcEnable == PMIC_DISABLE);
    PLATFORM_ASSERT(pmicHandle.pCommHandle == NULL);
    PLATFORM_ASSERT(pmicHandle.pQACommHandle == NULL);
    PLATFORM_ASSERT(pmicHandle.pFnPmicCommIoRd == NULL);
    PLATFORM_ASSERT(pmicHandle.pFnPmicCommIoWr == NULL);
    PLATFORM_ASSERT(pmicHandle.pFnPmicCritSecStart == NULL);
    PLATFORM_ASSERT(pmicHandle.pFnPmicCritSecStop == NULL);
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
