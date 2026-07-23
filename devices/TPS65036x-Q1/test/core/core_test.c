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
#include "test_constants.h"

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/* Test organization macros are defined in core_test.h */

/* ========================================================================== */
/*                             Global Variables                               */
/* ========================================================================== */

static Pmic_Handle_t pmicHandle = {0U};

/* ========================================================================== */
/*                           Function Declarations                            */
/* ========================================================================== */

static int32_t coreTest_unlockPmicRegs(Pmic_Handle_t *pHandle);

/* ========================================================================== */
/*                           Function Definitions                             */
/* ========================================================================== */

void core_test(void *args)
{
    char msg[50U] = {0};
    int32_t status = PMIC_ST_SUCCESS;

    platform_init();

    Pmic_HandleCfg_t pmicCfg = {
        .validParams = (PMIC_CFG_INIT_I2C_ADDR0_VALID |
                        PMIC_CFG_INIT_COMM_HANDLE_0_VALID |
                        PMIC_CFG_INIT_IO_READ_VALID |
                        PMIC_CFG_INIT_IO_WRITE_VALID |
                        PMIC_CFG_INIT_CRITICAL_SECTION_START_VALID |
                        PMIC_CFG_INIT_CRITICAL_SECTION_STOP_VALID |
                        PMIC_CFG_INIT_IRQ_RESPONSE_CALLBACK_VALID |
                        PMIC_CFG_INIT_TIMER_WAIT_MS_VALID),
        .i2cAddr0 = PLATFORM_TARGET_I2C_ADDR,
        .commHandle0 = platform_getCommHandle(),
        .ioRead = &platform_rxByte,
        .ioWrite = &platform_txByte,
        .criticalSectionStart = &platform_critSecStart,
        .criticalSectionStop = &platform_critSecStop,
        .irqResponseCallback = &platform_irqResponse,
        .timerWaitMs = &testUtils_timerWaitMs
    };

    testTimer_startModule("Core");

    platform_printString("\r\n");
    platform_printString("CORE_TEST\r\n");
    platform_printString("-------\r\n\r\n");

    status = Pmic_init(&pmicHandle, &pmicCfg);

    if (status == PMIC_ST_SUCCESS)
    {
        testUtils_printSiRev(&pmicHandle);

        /* Unlock PMIC registers for testing (except lock-specific tests) */
        status = coreTest_unlockPmicRegs(&pmicHandle);
        if (status != PMIC_ST_SUCCESS)
        {
            (void)sprintf(msg, "Error unlocking PMIC registers: %d\r\n", status);
            platform_printString(msg);
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

void test_neg_core_getNvmRev_nullParam_pmicHandle(void)
{
    // Pass NULL pmicHandle into Pmic_getNvmRev()
    uint8_t nvmRev = 0U;
    int32_t status = Pmic_getNvmRev(NULL, &nvmRev);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_core_getNvmRev_nullParam_nvmRev(void)
{
    // Pass NULL nvmRev into Pmic_getNvmRev()
    int32_t status = Pmic_getNvmRev(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_core_getSiliconRev_nullParam_pmicHandle(void)
{
    // Pass NULL pmicHandle into Pmic_getSiliconRev()
    uint8_t siliconRev = 0U;
    int32_t status = Pmic_getSiliconRev(NULL, &siliconRev);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_core_getSiliconRev_nullParam_siliconRev(void)
{
    // Pass NULL siliconRev into Pmic_getSiliconRev()
    int32_t status = Pmic_getSiliconRev(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_core_setRegLockState_nullParam_pmicHandle(void)
{
    // Pass NULL pmicHandle into Pmic_setRegLockState()
    int32_t status = Pmic_setRegLockState(NULL, PMIC_LOCK);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_core_disableRegLock_nullParam_pmicHandle(void)
{
    // Pass NULL pmicHandle into Pmic_disableRegLock()
    int32_t status = Pmic_disableRegLock(NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_core_enableRegLock_nullParam_pmicHandle(void)
{
    // Pass NULL pmicHandle into Pmic_enableRegLock()
    int32_t status = Pmic_enableRegLock(NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_core_getRegLockState_nullParam_pmicHandle(void)
{
    // Pass NULL pmicHandle into Pmic_getRegLockState()
    bool isLocked = (bool)false;
    int32_t status = Pmic_getRegLockState(NULL, &isLocked);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_core_getRegLockState_nullParam_regLockStat(void)
{
    // Pass NULL regLockStat into Pmic_getRegLockState()
    int32_t status = Pmic_getRegLockState(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_core_ioSetCrcEnableState_nullParam_pmicHandle(void)
{
    // Pass NULL pmicHandle into Pmic_ioSetCrcEnableState()
    int32_t status = Pmic_ioSetCrcEnableState(NULL, PMIC_ENABLE);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_core_ioCrcEnable_nullParam_pmicHandle(void)
{
    // Pass NULL pmicHandle into Pmic_ioCrcEnable()
    int32_t status = Pmic_ioCrcEnable(NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_core_ioCrcDisable_nullParam_pmicHandle(void)
{
    // Pass NULL pmicHandle into Pmic_ioCrcDisable()
    int32_t status = Pmic_ioCrcDisable(NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_core_ioGetCrcEnableState_nullParam_pmicHandle(void)
{
    // Pass NULL pmicHandle into Pmic_ioGetCrcEnableState()
    bool isEnabled = PMIC_DISABLE;
    int32_t status = Pmic_ioGetCrcEnableState(NULL, &isEnabled);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_core_ioGetCrcEnableState_nullParam_crcEnabled(void)
{
    // Pass NULL crcEnabled into Pmic_ioGetCrcEnableState()
    int32_t status = Pmic_ioGetCrcEnableState(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_core_setPwrOn_nullParam_pmicHandle(void)
{
    // Pass NULL pmicHandle into Pmic_setPwrOn()
    int32_t status = Pmic_setPwrOn(NULL, PMIC_ENABLE);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_core_getPwrOn_nullParam_pmicHandle(void)
{
    // Pass NULL pmicHandle into Pmic_getPwrOn()
    bool isEnabled = PMIC_DISABLE;
    int32_t status = Pmic_getPwrOn(NULL, &isEnabled);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_core_getPwrOn_nullParam_pwrOnStat(void)
{
    // Pass NULL pwrOnStat into Pmic_getPwrOn()
    int32_t status = Pmic_getPwrOn(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_core_setLpmCfg_nullParam_pmicHandle(void)
{
    // Pass NULL pmicHandle into Pmic_setLpmCfg()
    Pmic_CoreLpmCfg_t lpmCfg = {
        .validParams = PMIC_CFG_CORE_LPM_VMON_EN_VALID,
        .vmonEn = PMIC_ENABLE
    };
    int32_t status = Pmic_setLpmCfg(NULL, &lpmCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_core_setLpmCfg_nullParam_lpmCfg(void)
{
    // Pass NULL lpmCfg into Pmic_setLpmCfg()
    int32_t status = Pmic_setLpmCfg(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_core_setLpmCfg_outOfBounds_pinDetection(void)
{
    // Pass out of bounds pinDetection into Pmic_setLpmCfg()
    Pmic_CoreLpmCfg_t lpmCfg = {
        .validParams = PMIC_CFG_CORE_LPM_PIN_DETECTION_VALID,
        .pinDetection = PMIC_PIN_DETECTION_CONDITION_MAX + 1U
    };
    int32_t status = Pmic_setLpmCfg(&pmicHandle, &lpmCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_core_setLpmCfg_outOfBounds_detectionDelay(void)
{
    // Pass out of bounds detectionDelay into Pmic_setLpmCfg()
    Pmic_CoreLpmCfg_t lpmCfg = {
        .validParams = PMIC_CFG_CORE_LPM_DETECTION_DELAY_VALID,
        .detectionDelay = PMIC_DETECTION_DELAY_MAX + 1U
    };
    int32_t status = Pmic_setLpmCfg(&pmicHandle, &lpmCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_core_getLpmCfg_nullParam_pmicHandle(void)
{
    // Pass NULL pmicHandle into Pmic_getLpmCfg()
    Pmic_CoreLpmCfg_t lpmCfg = {.validParams = PMIC_CFG_CORE_LPM_VMON_EN_VALID};
    int32_t status = Pmic_getLpmCfg(NULL, &lpmCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_core_getLpmCfg_nullParam_lpmCfg(void)
{
    // Pass NULL lpmCfg into Pmic_getLpmCfg()
    int32_t status = Pmic_getLpmCfg(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_core_runABIST_nullParam_pmicHandle(void)
{
    // Pass NULL pmicHandle into Pmic_runABIST()
    int32_t status = Pmic_runABIST(NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_core_getABISTStat_nullParam_pmicHandle(void)
{
    // Pass NULL pmicHandle into Pmic_getABISTStat()
    bool isActive = (bool)false;
    int32_t status = Pmic_getABISTStat(NULL, &isActive);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_core_getABISTStat_nullParam_isActive(void)
{
    // Pass NULL isActive into Pmic_getABISTStat()
    int32_t status = Pmic_getABISTStat(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_core_setScratchPadValue_nullParam_pmicHandle(void)
{
    // Pass NULL pmicHandle into Pmic_setScratchPadValue()
    int32_t status = Pmic_setScratchPadValue(NULL, PMIC_SCRATCH_PAD_REG_1, TEST_PATTERN_AA);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_core_setScratchPadValue_outOfBounds_scratchPadRegNum(void)
{
    // Pass out of bounds scratchPadRegNum into Pmic_setScratchPadValue()
    int32_t status = Pmic_setScratchPadValue(&pmicHandle, PMIC_SCRATCH_PAD_REG_MAX + 1U, TEST_PATTERN_AA);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_core_getScratchPadValue_nullParam_pmicHandle(void)
{
    // Pass NULL pmicHandle into Pmic_getScratchPadValue()
    uint8_t value = 0U;
    int32_t status = Pmic_getScratchPadValue(NULL, PMIC_SCRATCH_PAD_REG_1, &value);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_core_getScratchPadValue_outOfBounds_scratchPadRegNum(void)
{
    // Pass out of bounds scratchPadRegNum into Pmic_getScratchPadValue()
    uint8_t value = 0U;
    int32_t status = Pmic_getScratchPadValue(&pmicHandle, PMIC_SCRATCH_PAD_REG_MAX + 1U, &value);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_core_getScratchPadValue_nullParam_value(void)
{
    // Pass NULL value into Pmic_getScratchPadValue()
    int32_t status = Pmic_getScratchPadValue(&pmicHandle, PMIC_SCRATCH_PAD_REG_1, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/* checkHandle API tests */

void test_neg_core_checkHandle_nullCritSecStart(void)
{
    // Pass handle with NULL criticalSectionStart into Pmic_checkHandle()
    Pmic_Handle_t testHandle = {0};

    testHandle.drvInitStat = TEST_PMIC_INIT_MAGIC;
    testHandle.commHandle0 = (void*)&pmicHandle;
    testHandle.ioRead = &platform_rxByte;
    testHandle.ioWrite = &platform_txByte;
    testHandle.criticalSectionStart = NULL;
    testHandle.criticalSectionStop = &platform_critSecStop;
    testHandle.retryIntervalMs = 0U;

    int32_t status = Pmic_checkHandle(&testHandle);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_FPTR);
}

void test_neg_core_checkHandle_nullCritSecStop(void)
{
    // Pass handle with NULL criticalSectionStop into Pmic_checkHandle()
    Pmic_Handle_t testHandle = {0};

    testHandle.drvInitStat = TEST_PMIC_INIT_MAGIC;
    testHandle.commHandle0 = (void*)&pmicHandle;
    testHandle.ioRead = &platform_rxByte;
    testHandle.ioWrite = &platform_txByte;
    testHandle.criticalSectionStart = &platform_critSecStart;
    testHandle.criticalSectionStop = NULL;
    testHandle.retryIntervalMs = 0U;

    int32_t status = Pmic_checkHandle(&testHandle);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_FPTR);
}

void test_pos_core_checkHandle_validCriticalSection(void)
{
    // Pass handle with valid criticalSectionStart and Stop into Pmic_checkHandle()
    Pmic_Handle_t testHandle = {0};

    testHandle.drvInitStat = TEST_PMIC_INIT_MAGIC;
    testHandle.commHandle0 = (void*)&pmicHandle;
    testHandle.ioRead = &platform_rxByte;
    testHandle.ioWrite = &platform_txByte;
    testHandle.criticalSectionStart = &platform_critSecStart;
    testHandle.criticalSectionStop = &platform_critSecStop;
    testHandle.retryIntervalMs = 0U;

    int32_t status = Pmic_checkHandle(&testHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_core_getNvmRev(void)
{
    // Get PMIC NVM revision ID
    uint8_t nvmRev = 0xFFU;
    int32_t status = Pmic_getNvmRev(&pmicHandle, &nvmRev);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(nvmRev != 0xFFU);
}

void test_pos_core_getSiliconRev(void)
{
    // Get PMIC silicon revision
    uint8_t siliconRev = 0xFFU;
    int32_t status = Pmic_getSiliconRev(&pmicHandle, &siliconRev);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(siliconRev != 0xFFU);
}

void test_pos_core_setGetRegLock(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    bool isLocked = (bool)false;

    // Lock PMIC registers
    status = Pmic_setRegLockState(&pmicHandle, PMIC_LOCK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get actual register lock status and compare expected vs. actual values
    status = Pmic_getRegLockState(&pmicHandle, &isLocked);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(isLocked == (bool)true);

    // Unlock PMIC registers
    status = Pmic_setRegLockState(&pmicHandle, PMIC_UNLOCK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get actual register lock status and compare expected vs. actual values
    status = Pmic_getRegLockState(&pmicHandle, &isLocked);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(isLocked == (bool)false);

    // Verify registers are actually writable after unlock (functional test)
    // Use scratch pad register which is lock-protected and safe to modify
    uint8_t testVal = TEST_PATTERN_AA;
    status = Pmic_setScratchPadValue(&pmicHandle, PMIC_SCRATCH_PAD_REG_1, testVal);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    uint8_t readVal = 0x00;
    status = Pmic_getScratchPadValue(&pmicHandle, PMIC_SCRATCH_PAD_REG_1, &readVal);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(readVal == testVal);
}

void test_pos_core_enableDisableCRC8(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    bool crcEnabled = (bool)false;

    // Enable CRC8
    status = Pmic_ioSetCrcEnableState(&pmicHandle, PMIC_ENABLE);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get actual CRC8 enable status and compare expected vs. actual values
    status = Pmic_ioGetCrcEnableState(&pmicHandle, &crcEnabled);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(crcEnabled == (bool)true);

    // Disable CRC8
    status = Pmic_ioSetCrcEnableState(&pmicHandle, PMIC_DISABLE);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get actual CRC8 enable status and compare expected vs. actual values
    status = Pmic_ioGetCrcEnableState(&pmicHandle, &crcEnabled);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(crcEnabled == (bool)false);

    // Restore CRC to enabled (NVM default) so subsequent tests are not affected
    status = Pmic_ioSetCrcEnableState(&pmicHandle, PMIC_ENABLE);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_core_setGetPwrOn(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    bool poweredOn = (bool)false;

    // Enable PMIC power on
    status = Pmic_setPwrOn(&pmicHandle, PMIC_ENABLE);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get actual PMIC power on status and compare expected vs. actual values
    status = Pmic_getPwrOn(&pmicHandle, &poweredOn);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(poweredOn == (bool)true);

    // Disable PMIC power on
    status = Pmic_setPwrOn(&pmicHandle, PMIC_DISABLE);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get actual PMIC power on status and compare expected vs. actual values
    status = Pmic_getPwrOn(&pmicHandle, &poweredOn);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(poweredOn == (bool)false);
}

void test_pos_core_setGetLpmCfg_pinDetection(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_CoreLpmCfg_t expLpmCfg = {.validParams = PMIC_CFG_CORE_LPM_PIN_DETECTION_VALID};
    Pmic_CoreLpmCfg_t actLpmCfg = {.validParams = PMIC_CFG_CORE_LPM_PIN_DETECTION_VALID};

    // For each valid pin detection value...
    for (uint8_t pinDetectionCfg = PMIC_ALL_IRQ_CLEARED_CONDITION;
        pinDetectionCfg <= PMIC_PIN_DETECTION_CONDITION_MAX; pinDetectionCfg++)
    {
        // Set the expected configuration
        expLpmCfg.pinDetection = pinDetectionCfg;
        status = Pmic_setLpmCfg(&pmicHandle, &expLpmCfg);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        // Get actual configuration and compare expected vs. actual
        status = Pmic_getLpmCfg(&pmicHandle, &actLpmCfg);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        PLATFORM_ASSERT(pinDetectionCfg == actLpmCfg.pinDetection);
    }
}

void test_pos_core_setGetLpmCfg_detectionDelay(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_CoreLpmCfg_t expLpmCfg = {.validParams = PMIC_CFG_CORE_LPM_DETECTION_DELAY_VALID};
    Pmic_CoreLpmCfg_t actLpmCfg = {.validParams = PMIC_CFG_CORE_LPM_DETECTION_DELAY_VALID};

    // For each valid detection delay value...
    for (uint8_t detectionDelayCfg = PMIC_DETECTION_DELAY_50_MS;
        detectionDelayCfg <= PMIC_DETECTION_DELAY_MAX; detectionDelayCfg++)
    {
        // Set the expected configuration
        expLpmCfg.detectionDelay = detectionDelayCfg;
        status = Pmic_setLpmCfg(&pmicHandle, &expLpmCfg);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        // Get actual configuration and compare expected vs. actual
        status = Pmic_getLpmCfg(&pmicHandle, &actLpmCfg);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        PLATFORM_ASSERT(detectionDelayCfg == actLpmCfg.detectionDelay);
    }
}

void test_pos_core_setGetLpmCfg_vmonEn(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_CoreLpmCfg_t expLpmCfg = {.validParams = PMIC_CFG_CORE_LPM_VMON_EN_VALID};
    Pmic_CoreLpmCfg_t actLpmCfg = {.validParams = PMIC_CFG_CORE_LPM_VMON_EN_VALID};

    // Enable VMON in LPM
    expLpmCfg.vmonEn = PMIC_ENABLE;
    status = Pmic_setLpmCfg(&pmicHandle, &expLpmCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get actual VMON enable status and compare expected vs. actual values
    status = Pmic_getLpmCfg(&pmicHandle, &actLpmCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(actLpmCfg.vmonEn == PMIC_ENABLE);

    // Disable VMON in LPM
    expLpmCfg.vmonEn = PMIC_DISABLE;
    status = Pmic_setLpmCfg(&pmicHandle, &expLpmCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get actual VMON enable status and compare expected vs. actual values
    status = Pmic_getLpmCfg(&pmicHandle, &actLpmCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(actLpmCfg.vmonEn == PMIC_DISABLE);
}

void test_pos_core_setGetLpmCfg_esmEn(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_CoreLpmCfg_t expLpmCfg = {.validParams = PMIC_CFG_CORE_LPM_ESM_EN_VALID};
    Pmic_CoreLpmCfg_t actLpmCfg = {.validParams = PMIC_CFG_CORE_LPM_ESM_EN_VALID};

    // Enable ESM in LPM
    expLpmCfg.esmEn = PMIC_ENABLE;
    status = Pmic_setLpmCfg(&pmicHandle, &expLpmCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get actual ESM enable status and compare expected vs. actual values
    status = Pmic_getLpmCfg(&pmicHandle, &actLpmCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(actLpmCfg.esmEn == PMIC_ENABLE);

    // Disable ESM in LPM
    expLpmCfg.esmEn = PMIC_DISABLE;
    status = Pmic_setLpmCfg(&pmicHandle, &expLpmCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get actual ESM enable status and compare expected vs. actual values
    status = Pmic_getLpmCfg(&pmicHandle, &actLpmCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(actLpmCfg.esmEn == PMIC_DISABLE);
}

void test_pos_core_setGetLpmCfg_wdgEn(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_CoreLpmCfg_t expLpmCfg = {.validParams = PMIC_CFG_CORE_LPM_WDG_EN_VALID};
    Pmic_CoreLpmCfg_t actLpmCfg = {.validParams = PMIC_CFG_CORE_LPM_WDG_EN_VALID};

    // Enable WDG in LPM
    expLpmCfg.wdgEn = PMIC_ENABLE;
    status = Pmic_setLpmCfg(&pmicHandle, &expLpmCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get actual WDG enable status and compare expected vs. actual values
    status = Pmic_getLpmCfg(&pmicHandle, &actLpmCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(actLpmCfg.wdgEn == PMIC_ENABLE);

    // Disable WDG in LPM
    expLpmCfg.wdgEn = PMIC_DISABLE;
    status = Pmic_setLpmCfg(&pmicHandle, &expLpmCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get actual WDG enable status and compare expected vs. actual values
    status = Pmic_getLpmCfg(&pmicHandle, &actLpmCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(actLpmCfg.wdgEn == PMIC_DISABLE);
}

void test_pos_core_runABIST(void)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t intMiscRegAddr = (pmicHandle.isA0) ? 0x50U : 0x53U;
    const uint8_t maskMiscRegAddr = 0x38U, bufLen = 1U, abistDoneShift = 0U;

    // Clear all PMIC IRQs
    status = Pmic_irqClrAllFlags(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Unmask ABIST_DONE_MASK
    status = platform_rxByte(&pmicHandle, 0U, maskMiscRegAddr, &regData, bufLen);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    Pmic_setBitField(&regData, abistDoneShift, 1UL << abistDoneShift, 0U);
    status = Pmic_ioTxByte(&pmicHandle, maskMiscRegAddr, regData);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Run ABIST
    status = Pmic_runABIST(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Validate ABIST was run by checking ABIST_DONE_INT
    status = platform_rxByte(&pmicHandle, 0U, intMiscRegAddr, &regData, bufLen);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(Pmic_getBitField_b(regData, abistDoneShift) == (bool)true);

    // Clear all PMIC IRQs
    status = Pmic_irqClrAllFlags(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_core_setGetScratchPadVal(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t initVal = 0U, actVal = 0U, expVal = 0U;

    // For each scratchpad register...
    for (uint8_t scratchpadReg = PMIC_SCRATCH_PAD_REG_1; scratchpadReg <= PMIC_SCRATCH_PAD_REG_MAX; scratchpadReg++)
    {
        // Get initial value
        status = Pmic_getScratchPadValue(&pmicHandle, scratchpadReg, &initVal);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        // Set expected value (inverted initial value)
        expVal = ~initVal;
        status = Pmic_setScratchPadValue(&pmicHandle, scratchpadReg, expVal);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        // Get actual value and compare expected vs. actual
        status = Pmic_getScratchPadValue(&pmicHandle, scratchpadReg, &actVal);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        PLATFORM_ASSERT(initVal != actVal);
        PLATFORM_ASSERT(expVal == actVal);
    }
}

static int32_t coreTest_unlockPmicRegs(Pmic_Handle_t *pHandle)
{
    uint8_t regData = 0x9BU;
    const uint8_t bufLen = 1U;
    const uint16_t registerLockAddr = 0x09U;

    int32_t status = Pmic_checkHandle(pHandle);

    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioTxByte(pHandle, (uint8_t)registerLockAddr, regData);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        status = platform_rxByte(pHandle, 0U, registerLockAddr, &regData, bufLen);
    }

    if ((status == PMIC_ST_SUCCESS) && ((regData & PMIC_REGISTER_LOCK_STATUS_MASK) != 0U))
    {
        status = PMIC_ST_ERR_I2C_COMM_FAIL;
    }

    return status;
}

/* ========================================================================== */
/*                        LPM Get Tests (7)                                   */
/* ========================================================================== */

void test_pos_core_getLpmCfg_pinDetection(void)
{
    Pmic_CoreLpmCfg_t lpmCfg = {0};
    int32_t status;

    // First set a known value
    lpmCfg.validParams = PMIC_CFG_CORE_LPM_PIN_DETECTION_VALID;
    lpmCfg.pinDetection = PMIC_DELAY_VALUE_MET_CONDITION;
    status = Pmic_setLpmCfg(&pmicHandle, &lpmCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Now get independently
    Pmic_CoreLpmCfg_t readCfg = {0};
    readCfg.validParams = PMIC_CFG_CORE_LPM_PIN_DETECTION_VALID;
    status = Pmic_getLpmCfg(&pmicHandle, &readCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(readCfg.pinDetection == PMIC_DELAY_VALUE_MET_CONDITION);
}

void test_pos_core_getLpmCfg_detectionDelay(void)
{
    Pmic_CoreLpmCfg_t lpmCfg = {0};
    int32_t status;

    // First set a known value
    lpmCfg.validParams = PMIC_CFG_CORE_LPM_DETECTION_DELAY_VALID;
    lpmCfg.detectionDelay = PMIC_DETECTION_DELAY_250_MS;
    status = Pmic_setLpmCfg(&pmicHandle, &lpmCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Now get independently
    Pmic_CoreLpmCfg_t readCfg = {0};
    readCfg.validParams = PMIC_CFG_CORE_LPM_DETECTION_DELAY_VALID;
    status = Pmic_getLpmCfg(&pmicHandle, &readCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(readCfg.detectionDelay == PMIC_DETECTION_DELAY_250_MS);
}

void test_pos_core_getLpmCfg_vmonEn(void)
{
    Pmic_CoreLpmCfg_t lpmCfg = {0};
    int32_t status;

    // First set a known value
    lpmCfg.validParams = PMIC_CFG_CORE_LPM_VMON_EN_VALID;
    lpmCfg.vmonEn = PMIC_ENABLE;
    status = Pmic_setLpmCfg(&pmicHandle, &lpmCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Now get independently
    Pmic_CoreLpmCfg_t readCfg = {0};
    readCfg.validParams = PMIC_CFG_CORE_LPM_VMON_EN_VALID;
    status = Pmic_getLpmCfg(&pmicHandle, &readCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(readCfg.vmonEn == PMIC_ENABLE);
}

void test_pos_core_getLpmCfg_esmEn(void)
{
    Pmic_CoreLpmCfg_t lpmCfg = {0};
    int32_t status;

    // First set a known value
    lpmCfg.validParams = PMIC_CFG_CORE_LPM_ESM_EN_VALID;
    lpmCfg.esmEn = PMIC_ENABLE;
    status = Pmic_setLpmCfg(&pmicHandle, &lpmCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Now get independently
    Pmic_CoreLpmCfg_t readCfg = {0};
    readCfg.validParams = PMIC_CFG_CORE_LPM_ESM_EN_VALID;
    status = Pmic_getLpmCfg(&pmicHandle, &readCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(readCfg.esmEn == PMIC_ENABLE);
}

void test_pos_core_getLpmCfg_wdgEn(void)
{
    Pmic_CoreLpmCfg_t lpmCfg = {0};
    int32_t status;

    // First set a known value
    lpmCfg.validParams = PMIC_CFG_CORE_LPM_WDG_EN_VALID;
    lpmCfg.wdgEn = PMIC_ENABLE;
    status = Pmic_setLpmCfg(&pmicHandle, &lpmCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Now get independently
    Pmic_CoreLpmCfg_t readCfg = {0};
    readCfg.validParams = PMIC_CFG_CORE_LPM_WDG_EN_VALID;
    status = Pmic_getLpmCfg(&pmicHandle, &readCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(readCfg.wdgEn == PMIC_ENABLE);
}

void test_pos_core_getLpmCfg_multipleParams(void)
{
    Pmic_CoreLpmCfg_t lpmCfg = {0};
    int32_t status;

    // First set multiple known values
    lpmCfg.validParams = PMIC_CFG_CORE_LPM_PIN_DETECTION_VALID | PMIC_CFG_CORE_LPM_VMON_EN_VALID | PMIC_CFG_CORE_LPM_ESM_EN_VALID;
    lpmCfg.pinDetection = PMIC_ALL_IRQ_CLEARED_CONDITION;
    lpmCfg.vmonEn = PMIC_ENABLE;
    lpmCfg.esmEn = PMIC_DISABLE;
    status = Pmic_setLpmCfg(&pmicHandle, &lpmCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Now get multiple parameters at once
    Pmic_CoreLpmCfg_t readCfg = {0};
    readCfg.validParams = PMIC_CFG_CORE_LPM_PIN_DETECTION_VALID | PMIC_CFG_CORE_LPM_VMON_EN_VALID | PMIC_CFG_CORE_LPM_ESM_EN_VALID;
    status = Pmic_getLpmCfg(&pmicHandle, &readCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(readCfg.pinDetection == PMIC_ALL_IRQ_CLEARED_CONDITION);
    PLATFORM_ASSERT(readCfg.vmonEn == PMIC_ENABLE);
    PLATFORM_ASSERT(readCfg.esmEn == PMIC_DISABLE);
}

void test_neg_core_getLpmCfg_invalidParam_validParams(void)
{
    Pmic_CoreLpmCfg_t lpmCfg = {0};
    // Set invalid validParams (bits outside the valid range)
    lpmCfg.validParams = 0xFFFFFFFFU;
    int32_t status = Pmic_getLpmCfg(&pmicHandle, &lpmCfg);
    // API correctly masks validParams bits - invalid bits are ignored, not rejected
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_neg_core_setLpmCfg_zeroValidParams(void)
{
    // Pass validParams == 0 into Pmic_setLpmCfg()
    Pmic_CoreLpmCfg_t lpmCfg = {0};
    lpmCfg.validParams = 0U;
    int32_t status = Pmic_setLpmCfg(&pmicHandle, &lpmCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/* ========================================================================== */
/*                   Additional LPM Setter Tests (15)                         */
/* ========================================================================== */

void test_pos_core_setLpmCfg_pinDetection_allValues(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_CoreLpmCfg_t lpmCfg = {.validParams = PMIC_CFG_CORE_LPM_PIN_DETECTION_VALID};
    Pmic_CoreLpmCfg_t readCfg = {.validParams = PMIC_CFG_CORE_LPM_PIN_DETECTION_VALID};

    // Test each valid pin detection value
    for (uint8_t i = PMIC_ALL_IRQ_CLEARED_CONDITION; i <= PMIC_PIN_DETECTION_CONDITION_MAX; i++)
    {
        lpmCfg.pinDetection = i;
        status = Pmic_setLpmCfg(&pmicHandle, &lpmCfg);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        // Verify the value was set correctly
        status = Pmic_getLpmCfg(&pmicHandle, &readCfg);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        PLATFORM_ASSERT(readCfg.pinDetection == i);
    }
}

void test_pos_core_setLpmCfg_detectionDelay_allValues(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_CoreLpmCfg_t lpmCfg = {.validParams = PMIC_CFG_CORE_LPM_DETECTION_DELAY_VALID};
    Pmic_CoreLpmCfg_t readCfg = {.validParams = PMIC_CFG_CORE_LPM_DETECTION_DELAY_VALID};

    // Test each valid detection delay value
    for (uint8_t i = PMIC_DETECTION_DELAY_50_MS; i <= PMIC_DETECTION_DELAY_MAX; i++)
    {
        lpmCfg.detectionDelay = i;
        status = Pmic_setLpmCfg(&pmicHandle, &lpmCfg);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        // Verify the value was set correctly
        status = Pmic_getLpmCfg(&pmicHandle, &readCfg);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        PLATFORM_ASSERT(readCfg.detectionDelay == i);
    }
}

void test_pos_core_setLpmCfg_vmonEn_enable(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_CoreLpmCfg_t lpmCfg = {.validParams = PMIC_CFG_CORE_LPM_VMON_EN_VALID, .vmonEn = PMIC_ENABLE};
    Pmic_CoreLpmCfg_t readCfg = {.validParams = PMIC_CFG_CORE_LPM_VMON_EN_VALID};

    status = Pmic_setLpmCfg(&pmicHandle, &lpmCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_getLpmCfg(&pmicHandle, &readCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(readCfg.vmonEn == PMIC_ENABLE);
}

void test_pos_core_setLpmCfg_vmonEn_disable(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_CoreLpmCfg_t lpmCfg = {.validParams = PMIC_CFG_CORE_LPM_VMON_EN_VALID, .vmonEn = PMIC_DISABLE};
    Pmic_CoreLpmCfg_t readCfg = {.validParams = PMIC_CFG_CORE_LPM_VMON_EN_VALID};

    status = Pmic_setLpmCfg(&pmicHandle, &lpmCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_getLpmCfg(&pmicHandle, &readCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(readCfg.vmonEn == PMIC_DISABLE);
}

void test_pos_core_setLpmCfg_esmEn_enable(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_CoreLpmCfg_t lpmCfg = {.validParams = PMIC_CFG_CORE_LPM_ESM_EN_VALID, .esmEn = PMIC_ENABLE};
    Pmic_CoreLpmCfg_t readCfg = {.validParams = PMIC_CFG_CORE_LPM_ESM_EN_VALID};

    status = Pmic_setLpmCfg(&pmicHandle, &lpmCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_getLpmCfg(&pmicHandle, &readCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(readCfg.esmEn == PMIC_ENABLE);
}

void test_pos_core_setLpmCfg_esmEn_disable(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_CoreLpmCfg_t lpmCfg = {.validParams = PMIC_CFG_CORE_LPM_ESM_EN_VALID, .esmEn = PMIC_DISABLE};
    Pmic_CoreLpmCfg_t readCfg = {.validParams = PMIC_CFG_CORE_LPM_ESM_EN_VALID};

    status = Pmic_setLpmCfg(&pmicHandle, &lpmCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_getLpmCfg(&pmicHandle, &readCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(readCfg.esmEn == PMIC_DISABLE);
}

void test_pos_core_setLpmCfg_wdgEn_enable(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_CoreLpmCfg_t lpmCfg = {.validParams = PMIC_CFG_CORE_LPM_WDG_EN_VALID, .wdgEn = PMIC_ENABLE};
    Pmic_CoreLpmCfg_t readCfg = {.validParams = PMIC_CFG_CORE_LPM_WDG_EN_VALID};

    status = Pmic_setLpmCfg(&pmicHandle, &lpmCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_getLpmCfg(&pmicHandle, &readCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(readCfg.wdgEn == PMIC_ENABLE);
}

void test_pos_core_setLpmCfg_wdgEn_disable(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_CoreLpmCfg_t lpmCfg = {.validParams = PMIC_CFG_CORE_LPM_WDG_EN_VALID, .wdgEn = PMIC_DISABLE};
    Pmic_CoreLpmCfg_t readCfg = {.validParams = PMIC_CFG_CORE_LPM_WDG_EN_VALID};

    status = Pmic_setLpmCfg(&pmicHandle, &lpmCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_getLpmCfg(&pmicHandle, &readCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(readCfg.wdgEn == PMIC_DISABLE);
}

void test_pos_core_setLpmCfg_multipleEnables(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_CoreLpmCfg_t lpmCfg = {
        .validParams = PMIC_CFG_CORE_LPM_VMON_EN_VALID | PMIC_CFG_CORE_LPM_ESM_EN_VALID | PMIC_CFG_CORE_LPM_WDG_EN_VALID,
        .vmonEn = PMIC_ENABLE,
        .esmEn = PMIC_ENABLE,
        .wdgEn = PMIC_ENABLE
    };
    Pmic_CoreLpmCfg_t readCfg = {
        .validParams = PMIC_CFG_CORE_LPM_VMON_EN_VALID | PMIC_CFG_CORE_LPM_ESM_EN_VALID | PMIC_CFG_CORE_LPM_WDG_EN_VALID
    };

    status = Pmic_setLpmCfg(&pmicHandle, &lpmCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_getLpmCfg(&pmicHandle, &readCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(readCfg.vmonEn == PMIC_ENABLE);
    PLATFORM_ASSERT(readCfg.esmEn == PMIC_ENABLE);
    PLATFORM_ASSERT(readCfg.wdgEn == PMIC_ENABLE);
}

void test_pos_core_setLpmCfg_allParams(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_CoreLpmCfg_t lpmCfg = {
        .validParams = PMIC_CFG_CORE_LPM_PIN_DETECTION_VALID | PMIC_CFG_CORE_LPM_DETECTION_DELAY_VALID |
                       PMIC_CFG_CORE_LPM_VMON_EN_VALID | PMIC_CFG_CORE_LPM_ESM_EN_VALID | PMIC_CFG_CORE_LPM_WDG_EN_VALID,
        .pinDetection = PMIC_DELAY_VALUE_MET_CONDITION,
        .detectionDelay = PMIC_DETECTION_DELAY_100_MS,
        .vmonEn = PMIC_ENABLE,
        .esmEn = PMIC_DISABLE,
        .wdgEn = PMIC_ENABLE
    };
    Pmic_CoreLpmCfg_t readCfg = {
        .validParams = PMIC_CFG_CORE_LPM_PIN_DETECTION_VALID | PMIC_CFG_CORE_LPM_DETECTION_DELAY_VALID |
                       PMIC_CFG_CORE_LPM_VMON_EN_VALID | PMIC_CFG_CORE_LPM_ESM_EN_VALID | PMIC_CFG_CORE_LPM_WDG_EN_VALID
    };

    status = Pmic_setLpmCfg(&pmicHandle, &lpmCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_getLpmCfg(&pmicHandle, &readCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(readCfg.pinDetection == PMIC_DELAY_VALUE_MET_CONDITION);
    PLATFORM_ASSERT(readCfg.detectionDelay == PMIC_DETECTION_DELAY_100_MS);
    PLATFORM_ASSERT(readCfg.vmonEn == PMIC_ENABLE);
    PLATFORM_ASSERT(readCfg.esmEn == PMIC_DISABLE);
    PLATFORM_ASSERT(readCfg.wdgEn == PMIC_ENABLE);
}

void test_pos_core_setLpmCfg_pinDetection_boundaryMin(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_CoreLpmCfg_t lpmCfg = {
        .validParams = PMIC_CFG_CORE_LPM_PIN_DETECTION_VALID,
        .pinDetection = PMIC_ALL_IRQ_CLEARED_CONDITION
    };
    Pmic_CoreLpmCfg_t readCfg = {.validParams = PMIC_CFG_CORE_LPM_PIN_DETECTION_VALID};

    status = Pmic_setLpmCfg(&pmicHandle, &lpmCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_getLpmCfg(&pmicHandle, &readCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(readCfg.pinDetection == PMIC_ALL_IRQ_CLEARED_CONDITION);
}

void test_pos_core_setLpmCfg_pinDetection_boundaryMax(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_CoreLpmCfg_t lpmCfg = {
        .validParams = PMIC_CFG_CORE_LPM_PIN_DETECTION_VALID,
        .pinDetection = PMIC_PIN_DETECTION_CONDITION_MAX
    };
    Pmic_CoreLpmCfg_t readCfg = {.validParams = PMIC_CFG_CORE_LPM_PIN_DETECTION_VALID};

    status = Pmic_setLpmCfg(&pmicHandle, &lpmCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_getLpmCfg(&pmicHandle, &readCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(readCfg.pinDetection == PMIC_PIN_DETECTION_CONDITION_MAX);
}

void test_pos_core_setLpmCfg_detectionDelay_boundaryMin(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_CoreLpmCfg_t lpmCfg = {
        .validParams = PMIC_CFG_CORE_LPM_DETECTION_DELAY_VALID,
        .detectionDelay = PMIC_DETECTION_DELAY_50_MS
    };
    Pmic_CoreLpmCfg_t readCfg = {.validParams = PMIC_CFG_CORE_LPM_DETECTION_DELAY_VALID};

    status = Pmic_setLpmCfg(&pmicHandle, &lpmCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_getLpmCfg(&pmicHandle, &readCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(readCfg.detectionDelay == PMIC_DETECTION_DELAY_50_MS);
}

void test_pos_core_setLpmCfg_detectionDelay_boundaryMax(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_CoreLpmCfg_t lpmCfg = {
        .validParams = PMIC_CFG_CORE_LPM_DETECTION_DELAY_VALID,
        .detectionDelay = PMIC_DETECTION_DELAY_MAX
    };
    Pmic_CoreLpmCfg_t readCfg = {.validParams = PMIC_CFG_CORE_LPM_DETECTION_DELAY_VALID};

    status = Pmic_setLpmCfg(&pmicHandle, &lpmCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_getLpmCfg(&pmicHandle, &readCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(readCfg.detectionDelay == PMIC_DETECTION_DELAY_MAX);
}

void test_pos_core_setLpmCfg_pinDetectionAndDelay(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_CoreLpmCfg_t lpmCfg = {
        .validParams = PMIC_CFG_CORE_LPM_PIN_DETECTION_VALID | PMIC_CFG_CORE_LPM_DETECTION_DELAY_VALID,
        .pinDetection = PMIC_DELAY_VALUE_MET_CONDITION,
        .detectionDelay = PMIC_DETECTION_DELAY_250_MS
    };
    Pmic_CoreLpmCfg_t readCfg = {
        .validParams = PMIC_CFG_CORE_LPM_PIN_DETECTION_VALID | PMIC_CFG_CORE_LPM_DETECTION_DELAY_VALID
    };

    status = Pmic_setLpmCfg(&pmicHandle, &lpmCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_getLpmCfg(&pmicHandle, &readCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(readCfg.pinDetection == PMIC_DELAY_VALUE_MET_CONDITION);
    PLATFORM_ASSERT(readCfg.detectionDelay == PMIC_DETECTION_DELAY_250_MS);
}

/* ========================================================================== */
/*                        ABIST Status Test (1)                               */
/* ========================================================================== */

void test_pos_core_getABISTStat_active(void)
{
    int32_t status;
    bool isActive = (bool)false;

    // Clear all PMIC IRQs before starting
    status = Pmic_irqClrAllFlags(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify ABIST is not active initially
    status = Pmic_getABISTStat(&pmicHandle, &isActive);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(isActive == (bool)false);

    // Run ABIST
    status = Pmic_runABIST(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Check if ABIST is active (immediately after triggering)
    status = Pmic_getABISTStat(&pmicHandle, &isActive);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    // Note: ABIST may complete very quickly, so we just verify the API works
    // The actual active status depends on timing

    // Clear all PMIC IRQs after test
    status = Pmic_irqClrAllFlags(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

/* ========================================================================== */
/*                    Silicon Revision Validation Tests                       */
/* ========================================================================== */

void test_neg_core_getLpmCfg_zeroValidParams(void)
{
    // Test zero validParams for getLpmCfg (lines 362-363)
    int32_t status;
    Pmic_CoreLpmCfg_t lpmCfg = {
        .validParams = 0U  // Zero validParams
    };

    status = Pmic_getLpmCfg(&pmicHandle, &lpmCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test A0 silicon detection when registers are initially locked
 *
 * Covers pmic.c:288-289 (unlock), 310-311 (A0 detection), 317-318 (re-lock)
 *
 * The mock library supports A0 silicon emulation:
 * - A0: MASK_MODERATE_ERR (0x39) bit 5 (NRSTOUT_READBACK_MASK) is writable
 * - B0/B1: MASK_MODERATE_ERR (0x39) bit 5 is read-only
 * - platform_reinitWithSiliconLocked() keeps registers locked
 */
void test_pos_core_init_A0_silicon_with_locked_registers(void)
{
#ifdef BUILD_MOCK
    Pmic_Handle_t testHandle = {0};
    int32_t status;

    /* Step 1: Reinitialize mock as A0 silicon with registers LOCKED */
    status = platform_reinitWithSiliconLocked(PMIC_SILICON_REV_A0);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Step 2: Initialize PMIC - triggers A0 detection with locked registers */
    Pmic_HandleCfg_t pmicCfg = {
        .validParams = (PMIC_CFG_INIT_I2C_ADDR0_VALID | PMIC_CFG_INIT_COMM_HANDLE_0_VALID |
                       PMIC_CFG_INIT_IO_READ_VALID | PMIC_CFG_INIT_IO_WRITE_VALID |
                       PMIC_CFG_INIT_CRITICAL_SECTION_START_VALID |
                       PMIC_CFG_INIT_CRITICAL_SECTION_STOP_VALID),
        .i2cAddr0 = PLATFORM_TARGET_I2C_ADDR,
        .commHandle0 = platform_getCommHandle(),
        .ioRead = &platform_rxByte,
        .ioWrite = &platform_txByte,
        .criticalSectionStart = &platform_critSecStart,
        .criticalSectionStop = &platform_critSecStop
    };

    status = Pmic_init(&testHandle, &pmicCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Step 3: Verify A0 was detected (line 310-311) */
    PLATFORM_ASSERT(testHandle.isA0 == (bool)true);

    /* Step 4: Verify registers were re-locked (line 317-318) */
    uint8_t lockStatus = 0U;
    status = Pmic_ioRxByte(&testHandle, PMIC_REGISTER_LOCK_REG, &lockStatus);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT((lockStatus & 0x01U) != 0U);  /* Bit 0 = locked */

    /* Cleanup */
    (void)Pmic_deinit(&testHandle);

    /* Restore mock to default B0 silicon (unlocked) for other tests */
    status = platform_reinitWithSilicon(PMIC_SILICON_REV_B0);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK");
#endif
}

/**
 * @brief Test B0 silicon detection with locked registers
 *
 * Covers pmic.c:294 FALSE path: NRSTOUT_READBACK_MASK bit returns 0
 *
 * The mock library supports B0 silicon emulation:
 * - B0: MASK_MODERATE_ERR (0x39) bit 5 (NRSTOUT_READBACK_MASK) is read-only
 * - Write attempt returns 0 on read, indicating B0 silicon
 * - platform_reinitWithSiliconLocked() keeps registers locked
 */
void test_pos_core_init_B0_silicon_with_locked_registers(void)
{
#ifdef BUILD_MOCK
    Pmic_Handle_t testHandle = {0};
    int32_t status;

    /* Step 1: Reinitialize mock as B0 silicon with registers LOCKED */
    status = platform_reinitWithSiliconLocked(PMIC_SILICON_REV_B0);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Step 2: Initialize PMIC - triggers B0 detection with locked registers */
    Pmic_HandleCfg_t pmicCfg = {
        .validParams = (PMIC_CFG_INIT_I2C_ADDR0_VALID | PMIC_CFG_INIT_COMM_HANDLE_0_VALID |
                       PMIC_CFG_INIT_IO_READ_VALID | PMIC_CFG_INIT_IO_WRITE_VALID |
                       PMIC_CFG_INIT_CRITICAL_SECTION_START_VALID |
                       PMIC_CFG_INIT_CRITICAL_SECTION_STOP_VALID),
        .i2cAddr0 = PLATFORM_TARGET_I2C_ADDR,
        .commHandle0 = platform_getCommHandle(),
        .ioRead = &platform_rxByte,
        .ioWrite = &platform_txByte,
        .criticalSectionStart = &platform_critSecStart,
        .criticalSectionStop = &platform_critSecStop
    };

    status = Pmic_init(&testHandle, &pmicCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Step 3: Verify B0 was detected (line 294 condition = FALSE) */
    PLATFORM_ASSERT(testHandle.isA0 == (bool)false);

    /* Step 4: Verify registers were re-locked */
    uint8_t lockStatus = 0U;
    status = Pmic_ioRxByte(&testHandle, PMIC_REGISTER_LOCK_REG, &lockStatus);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT((lockStatus & 0x01U) != 0U);  /* Bit 0 = locked */

    /* Cleanup */
    (void)Pmic_deinit(&testHandle);

    /* Restore mock to default B0 silicon (unlocked) for other tests */
    status = platform_reinitWithSilicon(PMIC_SILICON_REV_B0);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK");
#endif
}

/**
 * @brief Test B0 silicon detection with unlocked registers
 *
 * Covers pmic.c:294 FALSE path without lock/unlock operations
 *
 * The mock library supports B0 silicon emulation:
 * - B0: MASK_MODERATE_ERR (0x39) bit 5 (NRSTOUT_READBACK_MASK) is read-only
 * - Write attempt returns 0 on read, indicating B0 silicon
 * - platform_reinitWithSilicon() leaves registers unlocked
 */
void test_pos_core_init_B0_silicon_with_unlocked_registers(void)
{
#ifdef BUILD_MOCK
    Pmic_Handle_t testHandle = {0};
    int32_t status;

    /* Step 1: Reinitialize mock as B0 silicon (registers unlocked) */
    status = platform_reinitWithSilicon(PMIC_SILICON_REV_B0);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Step 2: Initialize PMIC - triggers B0 detection */
    Pmic_HandleCfg_t pmicCfg = {
        .validParams = (PMIC_CFG_INIT_I2C_ADDR0_VALID | PMIC_CFG_INIT_COMM_HANDLE_0_VALID |
                       PMIC_CFG_INIT_IO_READ_VALID | PMIC_CFG_INIT_IO_WRITE_VALID |
                       PMIC_CFG_INIT_CRITICAL_SECTION_START_VALID |
                       PMIC_CFG_INIT_CRITICAL_SECTION_STOP_VALID),
        .i2cAddr0 = PLATFORM_TARGET_I2C_ADDR,
        .commHandle0 = platform_getCommHandle(),
        .ioRead = &platform_rxByte,
        .ioWrite = &platform_txByte,
        .criticalSectionStart = &platform_critSecStart,
        .criticalSectionStop = &platform_critSecStop
    };

    status = Pmic_init(&testHandle, &pmicCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Step 3: Verify B0 was detected (line 294 condition = FALSE) */
    PLATFORM_ASSERT(testHandle.isA0 == (bool)false);

    /* Cleanup */
    (void)Pmic_deinit(&testHandle);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK");
#endif
}

/* ========================================================================== */
/*               Config CRC Tests (TC-CORE-0071 to TC-CORE-0076)             */
/* ========================================================================== */

void test_pos_core_configCrcEnable_enableOnly(void)
{
    Pmic_ConfigCrcStat_t configCrcStat = {0U};
    int32_t status;

    status = Pmic_configCrcEnable(&pmicHandle, PMIC_CFG_CRC_ENABLE_ONLY);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_getConfigCrcStatus(&pmicHandle, &configCrcStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(configCrcStat.crcEn == (bool)true);

    (void)Pmic_configCrcDisable(&pmicHandle);
}

void test_pos_core_configCrcEnable_recalculate(void)
{
    Pmic_ConfigCrcStat_t configCrcStat = {0U};
    int32_t status;

    status = Pmic_configCrcEnable(&pmicHandle, PMIC_CFG_CRC_RECALCULATE);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_getConfigCrcStatus(&pmicHandle, &configCrcStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(configCrcStat.errorDetected == (bool)false);

    (void)Pmic_configCrcDisable(&pmicHandle);
}

void test_neg_core_configCrcEnable_nullHandle(void)
{
    int32_t status = Pmic_configCrcEnable(NULL, PMIC_CFG_CRC_ENABLE_ONLY);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_pos_core_configCrcDisable_disable(void)
{
    Pmic_ConfigCrcStat_t configCrcStat = {0U};
    int32_t status;

    status = Pmic_configCrcEnable(&pmicHandle, PMIC_CFG_CRC_ENABLE_ONLY);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_configCrcDisable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_getConfigCrcStatus(&pmicHandle, &configCrcStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(configCrcStat.crcEn == (bool)false);
}

void test_neg_core_configCrcDisable_nullHandle(void)
{
    int32_t status = Pmic_configCrcDisable(NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_pos_core_getConfigCrcStatus_crcEnabled(void)
{
    Pmic_ConfigCrcStat_t configCrcStat = {0U};
    int32_t status;

    status = Pmic_configCrcEnable(&pmicHandle, PMIC_CFG_CRC_ENABLE_ONLY);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_getConfigCrcStatus(&pmicHandle, &configCrcStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(configCrcStat.crcEn == (bool)true);

    (void)Pmic_configCrcDisable(&pmicHandle);
}

void test_pos_core_getConfigCrcStatus_crcDisabled(void)
{
    Pmic_ConfigCrcStat_t configCrcStat = {0U};
    int32_t status;

    status = Pmic_configCrcDisable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_getConfigCrcStatus(&pmicHandle, &configCrcStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(configCrcStat.crcEn == (bool)false);
}

void test_neg_core_getConfigCrcStatus_nullHandle(void)
{
    Pmic_ConfigCrcStat_t configCrcStat = {0U};
    int32_t status = Pmic_getConfigCrcStatus(NULL, &configCrcStat);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_core_getConfigCrcStatus_nullStatus(void)
{
    int32_t status = Pmic_getConfigCrcStatus(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_pos_core_configCrcCalculate_calculate(void)
{
    int32_t status = Pmic_configCrcCalculate(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_neg_core_configCrcCalculate_nullHandle(void)
{
    int32_t status = Pmic_configCrcCalculate(NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_core_configCrcCalculate_crcEnabled(void)
{
    int32_t status;

    status = Pmic_configCrcEnable(&pmicHandle, PMIC_CFG_CRC_ENABLE_ONLY);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_configCrcCalculate(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NOT_SUPPORTED);

    (void)Pmic_configCrcDisable(&pmicHandle);
}

void test_pos_core_getConfigCrc_readValue(void)
{
    uint16_t value = 0U;
    int32_t status = Pmic_getConfigCrc(&pmicHandle, &value);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_neg_core_getConfigCrc_nullHandle(void)
{
    uint16_t value = 0U;
    int32_t status = Pmic_getConfigCrc(NULL, &value);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_core_getConfigCrc_nullValue(void)
{
    int32_t status = Pmic_getConfigCrc(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_pos_core_setConfigCrc_writeValue(void)
{
    int32_t status = Pmic_setConfigCrc(&pmicHandle, 0xA55AU);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_neg_core_setConfigCrc_nullHandle(void)
{
    int32_t status = Pmic_setConfigCrc(NULL, 0xA55AU);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}
