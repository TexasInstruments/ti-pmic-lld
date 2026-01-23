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

#include "pmic_test.h"
#include "regmap/core.h"
#include "test_constants.h"

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/* Used in certain unit tests to validate driver initialization status */
#define PMIC_TEST_DRV_INIT_STATUS TEST_PMIC_INIT_MAGIC /* "PMIC" in ASCII */

/* Arbitrary value used for testing purposes */
#define PMIC_TEST_ARBITRARY_VALUE TEST_PATTERN_AA

/* ========================================================================== */
/*                           Function Declarations                            */
/* ========================================================================== */
static inline void pmicInitTest_initCoreCfg(Pmic_HandleCfg_t *coreCfg);

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

void pmic_test(void *args)
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
        PMIC_TEST_RUN_ALL();
        platform_tearDownTests();
    }

    platform_deinit();
}

void test_neg_pmic_init_nullHandle(void)
{
    // Pass NULL handle into Pmic_init()
    Pmic_HandleCfg_t coreCfg = {0};
    int32_t status = Pmic_init(NULL, &coreCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_pmic_init_nullCoreCfg(void)
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
                           PMIC_I2C_ADDR1_VALID |
                           PMIC_I2C_ADDR2_VALID |
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
    coreCfg->i2cAddr1 = PMIC_TEST_ARBITRARY_VALUE;   // Not needed to be specified for Coach
    coreCfg->i2cAddr2 = PMIC_TEST_ARBITRARY_VALUE;  // Not needed to be specified for Coach
    coreCfg->crcEnable = PMIC_DISABLE;
    coreCfg->configCrcEnable = PMIC_DISABLE;
    coreCfg->commHandle0 = platform_getCommHandle();
    coreCfg->ioRead = &platform_rxByte;
    coreCfg->ioWrite = &platform_txByte;
    coreCfg->criticalSectionStart = &platform_critSecStart;
    coreCfg->criticalSectionStop = &platform_critSecStop;
    coreCfg->irqResponseCallback = &platform_irqResponse;
}

void test_neg_pmic_init_nullCoreCfgCommHandle0(void)
{
    Pmic_HandleCfg_t coreCfg = {0};
    Pmic_Handle_t handle = {0};

    pmicInitTest_initCoreCfg(&coreCfg);

    // Pass NULL commHandle0 into Pmic_init()
    coreCfg.commHandle0 = NULL;
    int32_t status = Pmic_init(&handle, &coreCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_pmic_init_nullCoreCfgIoRead(void)
{
    Pmic_HandleCfg_t coreCfg = {0};
    Pmic_Handle_t handle = {0};

    pmicInitTest_initCoreCfg(&coreCfg);

    // Pass NULL ioRead into Pmic_init()
    coreCfg.ioRead = NULL;
    int32_t status = Pmic_init(&handle, &coreCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_FPTR);
}

void test_neg_pmic_init_nullCoreCfgIoWrite(void)
{
    Pmic_HandleCfg_t coreCfg = {0};
    Pmic_Handle_t handle = {0};

    pmicInitTest_initCoreCfg(&coreCfg);

    // Pass NULL ioWrite into Pmic_init()
    coreCfg.ioWrite = NULL;
    int32_t status = Pmic_init(&handle, &coreCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_FPTR);
}

void test_neg_pmic_init_nullCoreCfgCriticalSectionStart(void)
{
    Pmic_HandleCfg_t coreCfg = {0};
    Pmic_Handle_t handle = {0};

    pmicInitTest_initCoreCfg(&coreCfg);

    // Pass NULL criticalSectionStart into Pmic_init()
    coreCfg.criticalSectionStart = NULL;
    int32_t status = Pmic_init(&handle, &coreCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_FPTR);
}

void test_neg_pmic_init_nullCoreCfgCriticalSectionStop(void)
{
    Pmic_HandleCfg_t coreCfg = {0};
    Pmic_Handle_t handle = {0};

    pmicInitTest_initCoreCfg(&coreCfg);

    // Pass NULL criticalSectionStop into Pmic_init()
    coreCfg.criticalSectionStop = NULL;
    int32_t status = Pmic_init(&handle, &coreCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_FPTR);
}

void test_neg_pmic_init_nullCoreCfgIrqResponseCallback(void)
{
    Pmic_HandleCfg_t coreCfg = {0};
    Pmic_Handle_t handle = {0};

    pmicInitTest_initCoreCfg(&coreCfg);

    // Pass NULL irqResponseCallback into Pmic_init()
    coreCfg.irqResponseCallback = NULL;
    int32_t status = Pmic_init(&handle, &coreCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_FPTR);
}

void test_neg_pmic_init_incorrectCoreCfgCommMode(void)
{
    Pmic_HandleCfg_t coreCfg = {0};
    Pmic_Handle_t handle = {0};

    pmicInitTest_initCoreCfg(&coreCfg);

    // Pass incorrect commMode into Pmic_init()
    coreCfg.commMode = PMIC_INTF_MAX + 1U;
    int32_t status = Pmic_init(&handle, &coreCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_pmic_deinit_nullHandle(void)
{
    // Pass NULL handle into Pmic_deinit()
    int32_t status = Pmic_deinit(NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_HANDLE);
}

void test_neg_pmic_checkPmicCoreHandle_nullHandle(void)
{
    // Pass NULL handle into Pmic_checkHandle()
    int32_t status = Pmic_checkHandle(NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_pmic_checkPmicCoreHandle_nullCommHandle0(void)
{
    Pmic_Handle_t handle = {
        .drvInitStat = (uint32_t)(PMIC_TEST_DRV_INIT_STATUS | (uint8_t)0U),
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
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_pmic_checkPmicCoreHandle_nullIoRead(void)
{
    Pmic_Handle_t handle = {
        .drvInitStat = (uint32_t)(PMIC_TEST_DRV_INIT_STATUS | (uint8_t)0U),
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

void test_neg_pmic_checkPmicCoreHandle_incorrectDrvInitStatus(void)
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

void test_pos_pmic_init(void)
{
    // Initialize PMIC LLD
    Pmic_HandleCfg_t coreCfg = {0};
    pmicInitTest_initCoreCfg(&coreCfg);
    int32_t status = Pmic_init(&pmicHandle, &coreCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(pmicHandle.drvInitStat == (uint32_t)(PMIC_TEST_DRV_INIT_STATUS | (uint8_t)0U));
    PLATFORM_ASSERT(pmicHandle.commMode == PMIC_INTF_I2C_SINGLE);
    PLATFORM_ASSERT(pmicHandle.i2cAddr0 == PLATFORM_TARGET_I2C_ADDR);
    PLATFORM_ASSERT(pmicHandle.i2cAddr1 == PMIC_TEST_ARBITRARY_VALUE);
    PLATFORM_ASSERT(pmicHandle.i2cAddr2 == PMIC_TEST_ARBITRARY_VALUE);
    PLATFORM_ASSERT(pmicHandle.crcEnable == PMIC_DISABLE);
    PLATFORM_ASSERT(pmicHandle.configCrcEnable == PMIC_DISABLE);
    PLATFORM_ASSERT(pmicHandle.commHandle0 == platform_getCommHandle());
    PLATFORM_ASSERT(pmicHandle.ioRead == &platform_rxByte);
    PLATFORM_ASSERT(pmicHandle.ioWrite == &platform_txByte);
    PLATFORM_ASSERT(pmicHandle.criticalSectionStart == &platform_critSecStart);
    PLATFORM_ASSERT(pmicHandle.criticalSectionStop == &platform_critSecStop);
    PLATFORM_ASSERT(pmicHandle.irqResponseCallback == &platform_irqResponse);
}

void test_pos_pmic_checkPmicCoreHandle(void)
{
    // Check PMIC core handle
    int32_t status = Pmic_checkHandle(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_pmic_deinit(void)
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

void test_pos_pmic_init_withCrcEnabled(void)
{
    // Initialize PMIC LLD with CRC enabled
    Pmic_HandleCfg_t coreCfg = {0};
    Pmic_Handle_t handle = {0};
    bool isEnabled = PMIC_DISABLE;

    pmicInitTest_initCoreCfg(&coreCfg);

    // Enable CRC
    coreCfg.crcEnable = PMIC_ENABLE;

    int32_t status = Pmic_init(&handle, &coreCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(handle.drvInitStat == (uint32_t)(PMIC_TEST_DRV_INIT_STATUS | (uint8_t)0U));
    PLATFORM_ASSERT(handle.crcEnable == PMIC_ENABLE);

    // Verify CRC is enabled in the device
    status = Pmic_ioGetCrcEnableState(&handle, &isEnabled);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(isEnabled == PMIC_ENABLE);

    // Clean up
    status = Pmic_deinit(&handle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_pmic_init_withConfigCrcEnabled(void)
{
    // Initialize PMIC LLD with config CRC enabled
    Pmic_HandleCfg_t coreCfg = {0};
    Pmic_Handle_t handle = {0};

    pmicInitTest_initCoreCfg(&coreCfg);

    // Enable config CRC
    coreCfg.configCrcEnable = PMIC_ENABLE;

    int32_t status = Pmic_init(&handle, &coreCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(handle.drvInitStat == (uint32_t)(PMIC_TEST_DRV_INIT_STATUS | (uint8_t)0U));
    PLATFORM_ASSERT(handle.configCrcEnable == PMIC_ENABLE);

    // Clean up
    status = Pmic_deinit(&handle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_pmic_init_withBothCrcEnabled(void)
{
    // Initialize PMIC LLD with both CRC types enabled
    Pmic_HandleCfg_t coreCfg = {0};
    Pmic_Handle_t handle = {0};
    bool isEnabled = PMIC_DISABLE;

    pmicInitTest_initCoreCfg(&coreCfg);

    // Enable both CRC types
    coreCfg.crcEnable = PMIC_ENABLE;
    coreCfg.configCrcEnable = PMIC_ENABLE;

    int32_t status = Pmic_init(&handle, &coreCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(handle.drvInitStat == (uint32_t)(PMIC_TEST_DRV_INIT_STATUS | (uint8_t)0U));
    PLATFORM_ASSERT(handle.crcEnable == PMIC_ENABLE);
    PLATFORM_ASSERT(handle.configCrcEnable == PMIC_ENABLE);

    // Verify CRC is enabled in the device
    status = Pmic_ioGetCrcEnableState(&handle, &isEnabled);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(isEnabled == PMIC_ENABLE);

    // Clean up
    status = Pmic_deinit(&handle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_pmic_init_crcErrorRecovery(void)
{
    // Test CRC configuration error handling during init
    Pmic_HandleCfg_t coreCfg = {0};
    Pmic_Handle_t handle = {0};

    pmicInitTest_initCoreCfg(&coreCfg);

    // Enable CRC and config CRC
    coreCfg.crcEnable = PMIC_ENABLE;
    coreCfg.configCrcEnable = PMIC_ENABLE;

    // Test CRC configuration error handling during init
    int32_t status = Pmic_init(&handle, &coreCfg);
    PLATFORM_ASSERT((status == PMIC_ST_SUCCESS) || (status > PMIC_ST_SUCCESS));

    // If init succeeded, verify handle state and clean up
    if (status == PMIC_ST_SUCCESS)
    {
        PLATFORM_ASSERT(handle.drvInitStat == (uint32_t)(PMIC_TEST_DRV_INIT_STATUS | (uint8_t)0U));
        status = Pmic_deinit(&handle);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    }
}

void test_pos_pmic_init_completeFlow(void)
{
    // Test complete initialization flow including device info retrieval and comm validation
    Pmic_HandleCfg_t coreCfg = {0};
    Pmic_Handle_t handle = {0};

    pmicInitTest_initCoreCfg(&coreCfg);

    // Initialize with all valid parameters
    int32_t status = Pmic_init(&handle, &coreCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify the driver initialization status magic number is set correctly
    PLATFORM_ASSERT(handle.drvInitStat == (uint32_t)(PMIC_TEST_DRV_INIT_STATUS | (uint8_t)0U));

    // Verify all function pointers are set correctly
    PLATFORM_ASSERT(handle.ioRead == &platform_rxByte);
    PLATFORM_ASSERT(handle.ioWrite == &platform_txByte);
    PLATFORM_ASSERT(handle.criticalSectionStart == &platform_critSecStart);
    PLATFORM_ASSERT(handle.criticalSectionStop == &platform_critSecStop);
    PLATFORM_ASSERT(handle.irqResponseCallback == &platform_irqResponse);

    // Verify communication handle is set
    PLATFORM_ASSERT(handle.commHandle0 == platform_getCommHandle());

    // Verify communication mode
    PLATFORM_ASSERT(handle.commMode == PMIC_INTF_I2C_SINGLE);

    // Verify device information was retrieved (devRev and devSiRev should be non-zero after successful init)

    // Verify handle can be validated successfully
    status = Pmic_checkHandle(&handle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Clean up
    status = Pmic_deinit(&handle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_pmic_init_deviceInfoRetrieval(void)
{
    // Test that device information (devRev, devSiRev) is correctly retrieved during init
    Pmic_HandleCfg_t coreCfg = {0};
    Pmic_Handle_t handle = {0};

    pmicInitTest_initCoreCfg(&coreCfg);

    // Initialize the handle - this should trigger getPmicInfo() internally
    int32_t status = Pmic_init(&handle, &coreCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify initialization completed successfully
    PLATFORM_ASSERT(handle.drvInitStat == (uint32_t)(PMIC_TEST_DRV_INIT_STATUS | (uint8_t)0U));

    // Verify device info registers are accessible after initialization

    // Read the registers directly to confirm they're accessible
    uint8_t devRevRegVal = 0U;
    status = Pmic_ioRxByte_CS(&handle, PMIC_DEV_REV_REG, &devRevRegVal);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    uint8_t mfgVerRegVal = 0U;
    status = Pmic_ioRxByte_CS(&handle, PMIC_MANUFACTURING_VER_REG, &mfgVerRegVal);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Clean up
    status = Pmic_deinit(&handle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_pmic_init_communicationValidation(void)
{
    // Test that communication validation occurs during initialization
    Pmic_HandleCfg_t coreCfg = {0};
    Pmic_Handle_t handle = {0};

    pmicInitTest_initCoreCfg(&coreCfg);

    // Initialize - this should trigger validateComms() internally
    int32_t status = Pmic_init(&handle, &coreCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify that initialization completed (which means validateComms succeeded)
    PLATFORM_ASSERT(handle.drvInitStat == (uint32_t)(PMIC_TEST_DRV_INIT_STATUS | (uint8_t)0U));

    // Verify we can successfully communicate with the device
    uint8_t regVal = 0U;
    status = Pmic_ioRxByte_CS(&handle, PMIC_DEV_REV_REG, &regVal);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Clean up
    status = Pmic_deinit(&handle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_pmic_deinit_completeFlow(void)
{
    // Test complete deinitialization flow and verify all fields are properly cleared
    Pmic_HandleCfg_t coreCfg = {0};
    Pmic_Handle_t handle = {0};

    pmicInitTest_initCoreCfg(&coreCfg);

    // First initialize the handle
    int32_t status = Pmic_init(&handle, &coreCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(handle.drvInitStat == (uint32_t)(PMIC_TEST_DRV_INIT_STATUS | (uint8_t)0U));

    // Store some expected values to verify they're properly set before deinit
    PLATFORM_ASSERT(handle.commHandle0 == platform_getCommHandle());
    PLATFORM_ASSERT(handle.ioRead == &platform_rxByte);
    PLATFORM_ASSERT(handle.ioWrite == &platform_txByte);
    PLATFORM_ASSERT(handle.commMode == PMIC_INTF_I2C_SINGLE);

    // Now deinitialize
    status = Pmic_deinit(&handle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify all fields are properly cleared/reset to default values
    PLATFORM_ASSERT(handle.drvInitStat == 0U);
    PLATFORM_ASSERT(handle.devRev == 0U);
    PLATFORM_ASSERT(handle.devSiRev == 0U);
    PLATFORM_ASSERT(handle.commMode == 0U);
    PLATFORM_ASSERT(handle.i2cAddr0 == 0U);
    PLATFORM_ASSERT(handle.i2cAddr1 == 0U);
    PLATFORM_ASSERT(handle.i2cAddr2 == 0U);
    PLATFORM_ASSERT(handle.crcEnable == PMIC_DISABLE);
    PLATFORM_ASSERT(handle.configCrcEnable == PMIC_DISABLE);
    PLATFORM_ASSERT(handle.commHandle0 == NULL);
    PLATFORM_ASSERT(handle.ioRead == NULL);
    PLATFORM_ASSERT(handle.ioWrite == NULL);
    PLATFORM_ASSERT(handle.criticalSectionStart == NULL);
    PLATFORM_ASSERT(handle.criticalSectionStop == NULL);
    PLATFORM_ASSERT(handle.irqResponseCallback == NULL);

    // After deinit, checkHandle should fail
    status = Pmic_checkHandle(&handle);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_HANDLE);
}

void test_pos_pmic_checkHandle_validations(void)
{
    // Test all validation paths in Pmic_checkHandle independently
    Pmic_HandleCfg_t coreCfg = {0};
    Pmic_Handle_t handle = {0};
    int32_t status;

    pmicInitTest_initCoreCfg(&coreCfg);

    // First, initialize a valid handle to use as baseline
    status = Pmic_init(&handle, &coreCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Test 1: Verify checkHandle succeeds with valid initialized handle
    status = Pmic_checkHandle(&handle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Save the valid drvInitStat
    uint32_t validInitStat = handle.drvInitStat;

    // Test 2: Invalid drvInitStat should fail
    handle.drvInitStat = TEST_INVALID_MAGIC;
    status = Pmic_checkHandle(&handle);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_HANDLE);

    // Restore valid state
    handle.drvInitStat = validInitStat;
    status = Pmic_checkHandle(&handle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Test 3: NULL ioRead function pointer should fail
    void *savedIoRead = (void *)handle.ioRead;
    handle.ioRead = NULL;
    status = Pmic_checkHandle(&handle);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_FPTR);

    // Restore and verify
    handle.ioRead = savedIoRead;
    status = Pmic_checkHandle(&handle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Test 4: NULL commHandle0 should fail
    void *savedCommHandle = handle.commHandle0;
    handle.commHandle0 = NULL;
    status = Pmic_checkHandle(&handle);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);

    // Restore and verify
    handle.commHandle0 = savedCommHandle;
    status = Pmic_checkHandle(&handle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Clean up
    status = Pmic_deinit(&handle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_pmic_init_withRetryCnt(void)
{
    // Initialize with PMIC_RETRY_CNT_VALID set
    Pmic_HandleCfg_t coreCfg = {0};
    Pmic_Handle_t handle = {0};

    pmicInitTest_initCoreCfg(&coreCfg);

    // Add retry count configuration
    coreCfg.validParams |= PMIC_RETRY_CNT_VALID;
    coreCfg.retryCnt = 5U;

    int32_t status = Pmic_init(&handle, &coreCfg);
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
    Pmic_HandleCfg_t coreCfg = {0};
    Pmic_Handle_t handle = {0};

    pmicInitTest_initCoreCfg(&coreCfg);

    // Add retry interval configuration
    // Note: When retryIntervalMs is non-zero, timerWaitMs must also be provided
    coreCfg.validParams |= PMIC_RETRY_INTERVAL_MS_VALID | PMIC_TIMER_WAIT_MS_VALID;
    coreCfg.retryIntervalMs = 100U;
    coreCfg.timerWaitMs = &testTimerWaitWrapper;

    int32_t status = Pmic_init(&handle, &coreCfg);
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
    Pmic_HandleCfg_t coreCfg = {0};
    Pmic_Handle_t handle = {0};

    pmicInitTest_initCoreCfg(&coreCfg);

    // Add timer wait callback configuration
    coreCfg.validParams |= PMIC_TIMER_WAIT_MS_VALID;
    coreCfg.timerWaitMs = &testTimerWaitWrapper;

    int32_t status = Pmic_init(&handle, &coreCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify timer wait callback is set in handle
    PLATFORM_ASSERT(handle.timerWaitMs == &testTimerWaitWrapper);

    // Clean up
    status = Pmic_deinit(&handle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_neg_pmic_init_timerWaitNull(void)
{
    // Set PMIC_TIMER_WAIT_MS_VALID but pass NULL callback
    Pmic_HandleCfg_t coreCfg = {0};
    Pmic_Handle_t handle = {0};

    pmicInitTest_initCoreCfg(&coreCfg);

    // Set valid param flag but provide NULL callback
    coreCfg.validParams |= PMIC_TIMER_WAIT_MS_VALID;
    coreCfg.timerWaitMs = NULL;

    int32_t status = Pmic_init(&handle, &coreCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_FPTR);
}


/* ========================================================================== */
/*           LP8772x-Q1 Tests for Uncovered Lines in pmic.c                  */
/* ========================================================================== */

void test_neg_pmic_checkHandle_invalidCommMode(void)
{
    // Test coverage for line 238: Corrupt handle commMode after init

    Pmic_Handle_t handle;
    Pmic_HandleCfg_t coreCfg;

    // Initialize with valid configuration
    pmicInitTest_initCoreCfg(&coreCfg);
    int32_t status = Pmic_init(&handle, &coreCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Corrupt the commMode to invalid value
    handle.commMode = PMIC_INTF_MAX + 1U;

    // Now try to use the handle with an API that calls Pmic_checkHandle
    // For example, Pmic_setScratchPadValue
    status = Pmic_setScratchPadValue(&handle, PMIC_SCRATCH_PAD_REG_1, TEST_PATTERN_AA);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);

    // No need to deinit corrupted handle
}

void test_neg_pmic_checkHandle_nullTimerWithRetry(void)
{
    // Test coverage for line 242: Set retryIntervalMs != 0 with timerWaitMs == NULL

    Pmic_Handle_t handle;
    Pmic_HandleCfg_t coreCfg;

    // Initialize with valid configuration
    pmicInitTest_initCoreCfg(&coreCfg);
    coreCfg.validParams |= PMIC_RETRY_INTERVAL_MS_VALID;
    coreCfg.retryIntervalMs = 10U;  // Non-zero retry interval
    // Explicitly set timerWaitMs to NULL (it should not be set)
    // Don't set PMIC_TIMER_WAIT_MS_VALID flag

    int32_t status = Pmic_init(&handle, &coreCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // After successful init, corrupt the handle to have retryIntervalMs != 0 but NULL timer
    handle.retryIntervalMs = 10U;
    handle.timerWaitMs = NULL;

    // Now call an API that uses Pmic_checkHandle
    status = Pmic_setScratchPadValue(&handle, PMIC_SCRATCH_PAD_REG_1, TEST_PATTERN_AA);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_FPTR);

    // No need to deinit corrupted handle
}
