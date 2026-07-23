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

#include "core_test.h"
#include "pmic.h"
#include "pmic_core.h"
#include "test_constants.h"

/* ========================================================================== */
/*                             Global Variables                               */
/* ========================================================================== */

static Pmic_Handle_t pmicHandle = {0};

/* ========================================================================== */
/*                           Function Definitions                             */
/* ========================================================================== */

void core_test(void *args)
{
    (void)args;
    char msg[50U] = {0};
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_HandleCfg_t handleCfg = {
        .validParams = (PMIC_CFG_INIT_COMM_MODE_VALID |
                        PMIC_CFG_INIT_I2C_ADDR0_VALID |
                        PMIC_CFG_INIT_I2C_ADDR1_VALID |
                        PMIC_CRC_ENABLE_0_VALID |
                        PMIC_CFG_INIT_COMM_HANDLE_0_VALID |
                        PMIC_CFG_INIT_IO_READ_VALID |
                        PMIC_CFG_INIT_IO_WRITE_VALID |
                        PMIC_CFG_INIT_CRITICAL_SECTION_START_VALID |
                        PMIC_CFG_INIT_CRITICAL_SECTION_STOP_VALID |
                        PMIC_CFG_INIT_IRQ_RESPONSE_CALLBACK_VALID),
        .commMode = PMIC_INTF_I2C_SINGLE,
        .i2cAddr0 = PLATFORM_I2C_ADDR_MAIN,
        .i2cAddr1 = PLATFORM_I2C_ADDR_SECONDARY,
        .crcEnable0 = PMIC_DISABLE,
        .commHandle0 = platform_getCommHandle0(),
        .ioRead = &platform_rxByte,
        .ioWrite = &platform_txByte,
        .criticalSectionStart = &platform_critSecStart,
        .criticalSectionStop = &platform_critSecStop,
        .irqResponseCallback = &platform_irqResponse
    };

    platform_init();
    testTimer_startModule("Core");

    platform_printString("\r\n");
    platform_printString("CORE_TEST\r\n");
    platform_printString("---------\r\n\r\n");

    status = Pmic_init(&pmicHandle, &handleCfg);

    if (status == PMIC_ST_SUCCESS)
    {
        /* Unlock registers for scratchpad configuration */
        int32_t unlockStatus = Pmic_setRegLockState(&pmicHandle, false);
        if (unlockStatus != PMIC_ST_SUCCESS)
        {
            (void)sprintf(msg, "WARNING: Failed to unlock registers: %d\r\n", unlockStatus);
            platform_printString(msg);
            /* Continue with tests anyway - some may still pass */
        }

        platform_setupTests();
        CORE_TEST_RUN_ALL();
        platform_tearDownTests();
    }
    else
    {
        (void)sprintf(msg, "Error in initializing PMIC LLD: %d\r\n", status);
        platform_printString(msg);
    }

    testTimer_endModule();
    (void)Pmic_deinit(&pmicHandle);
    platform_deinit();
}

/* ========================================================================== */
/*                         Negative Test Cases                                */
/* ========================================================================== */

void test_neg_core_coreGetSilRev_nullParam_handle(void)
{
    uint8_t siliconRev = 0U;
    int32_t status = Pmic_getSiliconRev(NULL, &siliconRev);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_core_coreGetSilRev_nullParam_siliconRev(void)
{
    int32_t status = Pmic_getSiliconRev(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_core_coreGetNvmRev_nullParam_handle(void)
{
    uint8_t nvmRev = 0U;
    int32_t status = Pmic_getNvmRev(NULL, &nvmRev);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_core_coreGetNvmRev_nullParam_nvmRev(void)
{
    int32_t status = Pmic_getNvmRev(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_core_coreGetRegLockState_nullParam_handle(void)
{
    bool lockState = PMIC_DISABLE;
    int32_t status = Pmic_getRegLockState(NULL, &lockState);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_core_coreGetRegLockState_nullParam_lockState(void)
{
    int32_t status = Pmic_getRegLockState(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_core_coreSetScratchPadValue_nullParam_handle(void)
{
    int32_t status = Pmic_setScratchPadValue(NULL, PMIC_SCRATCH_PAD_REG_1, TEST_PATTERN_AA);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_core_coreSetScratchPadValue_outOfBounds_scratchPadRegNum(void)
{
    int32_t status = Pmic_setScratchPadValue(&pmicHandle, PMIC_SCRATCH_PAD_REG_MAX + 1U, TEST_PATTERN_AA);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_core_coreGetScratchPadValue_outOfBounds_scratchPadRegNum(void)
{
    uint8_t value = 0U;
    int32_t status = Pmic_getScratchPadValue(&pmicHandle, PMIC_SCRATCH_PAD_REG_MAX + 1U, &value);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_core_coreGetScratchPadValue_nullValue(void)
{
    int32_t status = Pmic_getScratchPadValue(&pmicHandle, PMIC_SCRATCH_PAD_REG_1, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_core_setConfigCrcVal_nullHandle(void)
{
    int32_t status = Pmic_setConfigCrc(NULL, 0xA55AU);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test validatePmicHandle with NULL criticalSectionStart
 *
 * This test validates MCDC coverage for line 422 in pmic.c::validatePmicHandle()
 * Condition: (criticalSectionStart == NULL) || (criticalSectionStop == NULL)
 * Test case: First condition TRUE, second condition FALSE
 */
void test_neg_core_validatePmicHandle_nullCritSecStart(void)
{
    Pmic_Handle_t testHandle = pmicHandle;
    uint8_t siliconRev = 0U;
    testHandle.criticalSectionStart = NULL;  // Set first condition to TRUE
    // criticalSectionStop remains valid (second condition FALSE)

    int32_t status = Pmic_getSiliconRev(&testHandle, &siliconRev);  // Will call validatePmicHandle internally
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_FPTR);
}

/**
 * @brief Test validatePmicHandle with NULL criticalSectionStop
 *
 * This test validates MCDC coverage for line 422 in pmic.c::validatePmicHandle()
 * Condition: (criticalSectionStart == NULL) || (criticalSectionStop == NULL)
 * Test case: First condition FALSE, second condition TRUE
 */
void test_neg_core_validatePmicHandle_nullCritSecStop(void)
{
    Pmic_Handle_t testHandle = pmicHandle;
    uint8_t siliconRev = 0U;
    // criticalSectionStart remains valid (first condition FALSE)
    testHandle.criticalSectionStop = NULL;  // Set second condition to TRUE

    int32_t status = Pmic_getSiliconRev(&testHandle, &siliconRev);  // Will call validatePmicHandle internally
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_FPTR);
}

void test_neg_core_getConfigCrcVal_nullHandle(void)
{
    uint16_t value = 0U;
    int32_t status = Pmic_getConfigCrc(NULL, &value);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_core_getConfigCrcVal_nullValue(void)
{
    int32_t status = Pmic_getConfigCrc(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/* ========================================================================== */
/*                         Positive Test Cases                                */
/* ========================================================================== */

void test_pos_core_coreGetSilRev(void)
{
    uint8_t siliconRev = 0U;
    int32_t status = Pmic_getSiliconRev(&pmicHandle, &siliconRev);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_core_coreGetNvmRev(void)
{
    uint8_t nvmRev = 0U;
    int32_t status = Pmic_getNvmRev(&pmicHandle, &nvmRev);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_core_coreGetRegLockState(void)
{
    bool lockState = PMIC_DISABLE;
    int32_t status = Pmic_getRegLockState(&pmicHandle, &lockState);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_core_coreSetRegLockState_lock(void)
{
    int32_t status = Pmic_setRegLockState(&pmicHandle, true);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_core_coreSetRegLockState_unlock(void)
{
    int32_t status = Pmic_setRegLockState(&pmicHandle, false);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_neg_core_coreSetRegLockState_nullHandle(void)
{
    int32_t status = Pmic_setRegLockState(NULL, true);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_pos_core_coreSetScratchPadValue_reg1(void)
{
    uint8_t writeVal = TEST_PATTERN_A5;
    uint8_t readVal = 0U;

    // Write value to scratchpad register 1
    int32_t status = Pmic_setScratchPadValue(&pmicHandle, PMIC_SCRATCH_PAD_REG_1, writeVal);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Read back and verify
    status = Pmic_getScratchPadValue(&pmicHandle, PMIC_SCRATCH_PAD_REG_1, &readVal);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(readVal == writeVal);
}

void test_pos_core_coreGetScratchPadValue_reg2(void)
{
    uint8_t writeVal = TEST_PATTERN_5A;
    uint8_t readVal = 0U;

    // Write value to scratchpad register 2
    int32_t status = Pmic_setScratchPadValue(&pmicHandle, PMIC_SCRATCH_PAD_REG_2, writeVal);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Read back and verify
    status = Pmic_getScratchPadValue(&pmicHandle, PMIC_SCRATCH_PAD_REG_2, &readVal);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(readVal == writeVal);
}

void test_pos_core_coreSetScratchPadValue_reg3(void)
{
    uint8_t writeVal = TEST_MASK_HIGH_NIBBLE;
    uint8_t readVal = 0U;

    // Write value to scratchpad register 3
    int32_t status = Pmic_setScratchPadValue(&pmicHandle, PMIC_SCRATCH_PAD_REG_3, writeVal);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Read back and verify
    status = Pmic_getScratchPadValue(&pmicHandle, PMIC_SCRATCH_PAD_REG_3, &readVal);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(readVal == writeVal);
}

void test_pos_core_coreGetScratchPadValue_reg4(void)
{
    uint8_t writeVal = TEST_MASK_LOW_NIBBLE;
    uint8_t readVal = 0U;

    // Write value to scratchpad register 4
    int32_t status = Pmic_setScratchPadValue(&pmicHandle, PMIC_SCRATCH_PAD_REG_4, writeVal);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Read back and verify
    status = Pmic_getScratchPadValue(&pmicHandle, PMIC_SCRATCH_PAD_REG_4, &readVal);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(readVal == writeVal);
}

void test_pos_core_scratchPadValue_boundary(void)
{
    uint8_t writeVal = TEST_MASK_FULL_BYTE;
    uint8_t readVal = 0U;

    // Test minimum valid register (REG_1)
    int32_t status = Pmic_setScratchPadValue(&pmicHandle, PMIC_SCRATCH_PAD_REG_MIN, writeVal);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_getScratchPadValue(&pmicHandle, PMIC_SCRATCH_PAD_REG_MIN, &readVal);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(readVal == writeVal);

    // Test maximum valid register (REG_4)
    writeVal = 0x00U;
    status = Pmic_setScratchPadValue(&pmicHandle, PMIC_SCRATCH_PAD_REG_MAX, writeVal);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_getScratchPadValue(&pmicHandle, PMIC_SCRATCH_PAD_REG_MAX, &readVal);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(readVal == writeVal);
}

/**
 * @brief Test validatePmicHandle with valid critical section pointers
 *
 * This test validates MCDC coverage for line 422 in pmic.c::validatePmicHandle()
 * Condition: (criticalSectionStart == NULL) || (criticalSectionStop == NULL)
 * Test case: Both conditions FALSE (positive case)
 */
void test_pos_core_validatePmicHandle_validCriticalSection(void)
{
    // Using the global pmicHandle which has valid critical section pointers
    // This test verifies the positive path through line 422
    uint8_t siliconRev = 0U;
    int32_t status = Pmic_getSiliconRev(&pmicHandle, &siliconRev);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_core_setConfigCrcVal_writeAndVerify(void)
{
    int32_t status;
    uint16_t readBack = 0U;

    status = Pmic_setConfigCrc(&pmicHandle, 0xA55AU);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_getConfigCrc(&pmicHandle, &readBack);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(readBack == 0xA55AU);
}

void test_pos_core_getConfigCrcVal_readValue(void)
{
    int32_t status;
    uint16_t value = 0U;

    status = Pmic_setConfigCrc(&pmicHandle, 0x5AA5U);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_getConfigCrc(&pmicHandle, &value);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(value == 0x5AA5U);
}

/* Note: setUp/tearDown removed - provided by test_runner.c for Unity */
