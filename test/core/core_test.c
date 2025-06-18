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
 * @brief Source file containing definitions to PMIC Core tests.
 */

/* ========================================================================== */
/*                              Include Files                                 */
/* ========================================================================== */

#include "core_test.h"

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/* Run all Core tests */
#define CORE_TEST_RUN_ALL() PLATFORM_RUN_TEST(test_negative_Pmic_getDevId_nullParam_pmicHandle); \
                            PLATFORM_RUN_TEST(test_negative_Pmic_getDevId_nullParam_devId); \
                            PLATFORM_RUN_TEST(test_negative_Pmic_getNvmId_nullParam_pmicHandle); \
                            PLATFORM_RUN_TEST(test_negative_Pmic_getNvmId_nullParam_nvmId); \
                            PLATFORM_RUN_TEST(test_negative_Pmic_getNvmRev_nullParam_pmicHandle); \
                            PLATFORM_RUN_TEST(test_negative_Pmic_getNvmRev_nullParam_nvmRev); \
                            PLATFORM_RUN_TEST(test_negative_Pmic_getSiliconRev_nullParam_pmicHandle); \
                            PLATFORM_RUN_TEST(test_negative_Pmic_getSiliconRev_nullParam_siliconRev); \
                            PLATFORM_RUN_TEST(test_negative_Pmic_setRegLock_nullParam_pmicHandle); \
                            PLATFORM_RUN_TEST(test_negative_Pmic_unlockRegs_nullParam_pmicHandle); \
                            PLATFORM_RUN_TEST(test_negative_Pmic_lockRegs_nullParam_pmicHandle); \
                            PLATFORM_RUN_TEST(test_negative_Pmic_getRegLock_nullParam_pmicHandle); \
                            PLATFORM_RUN_TEST(test_negative_Pmic_getRegLock_nullParam_regLockStat); \
                            PLATFORM_RUN_TEST(test_negative_Pmic_enableDisableCRC8_nullParam_pmicHandle); \
                            PLATFORM_RUN_TEST(test_negative_Pmic_enableCRC8_nullParam_pmicHandle); \
                            PLATFORM_RUN_TEST(test_negative_Pmic_disableCRC8_nullParam_pmicHandle); \
                            PLATFORM_RUN_TEST(test_negative_Pmic_getCRC8Enable_nullParam_pmicHandle); \
                            PLATFORM_RUN_TEST(test_negative_Pmic_getCRC8Enable_nullParam_crcEnabled); \
                            PLATFORM_RUN_TEST(test_negative_Pmic_sendFsmCmd_nullParam_pmicHandle); \
                            PLATFORM_RUN_TEST(test_negative_Pmic_sendFsmCmd_invalid_fsmCmd); \
                            PLATFORM_RUN_TEST(test_negative_Pmic_setPwrOn_nullParam_pmicHandle); \
                            PLATFORM_RUN_TEST(test_negative_Pmic_getPwrOn_nullParam_pmicHandle); \
                            PLATFORM_RUN_TEST(test_negative_Pmic_getPwrOn_nullParam_pwrOnStat); \
                            PLATFORM_RUN_TEST(test_negative_Pmic_setLpmCfg_nullParam_pmicHandle); \
                            PLATFORM_RUN_TEST(test_negative_Pmic_setLpmCfg_nullParam_lpmCfg); \
                            PLATFORM_RUN_TEST(test_negative_Pmic_setLpmCfg_outOfBounds_pinDetection); \
                            PLATFORM_RUN_TEST(test_negative_Pmic_setLpmCfg_outOfBounds_detectionDelay); \
                            PLATFORM_RUN_TEST(test_negative_Pmic_getLpmCfg_nullParam_pmicHandle); \
                            PLATFORM_RUN_TEST(test_negative_Pmic_getLpmCfg_nullParam_lpmCfg); \
                            PLATFORM_RUN_TEST(test_negative_Pmic_runABIST_nullParam_pmicHandle); \
                            PLATFORM_RUN_TEST(test_negative_Pmic_getABISTStat_nullParam_pmicHandle); \
                            PLATFORM_RUN_TEST(test_negative_Pmic_getABISTStat_nullParam_isActive); \
                            PLATFORM_RUN_TEST(test_negative_Pmic_setScratchPadVal_nullParam_pmicHandle); \
                            PLATFORM_RUN_TEST(test_negative_Pmic_setScratchPadVal_outOfBounds_scratchPadRegNum); \
                            PLATFORM_RUN_TEST(test_negative_Pmic_getScratchPadVal_nullParam_pmicHandle); \
                            PLATFORM_RUN_TEST(test_negative_Pmic_getScratchPadVal_outOfBounds_scratchPadRegNum); \
                            PLATFORM_RUN_TEST(test_negative_Pmic_getScratchPadVal_nullParam_value); \
                            PLATFORM_RUN_TEST(test_negative_Pmic_setRecovCntThr_nullParam_pmicHandle); \
                            PLATFORM_RUN_TEST(test_negative_Pmic_setRecovCntThr_outOfBounds_threshold); \
                            PLATFORM_RUN_TEST(test_negative_Pmic_getRecovCntThr_nullParam_pmicHandle); \
                            PLATFORM_RUN_TEST(test_negative_Pmic_getRecovCntThr_nullParam_threshold); \
                            PLATFORM_RUN_TEST(test_negative_Pmic_getRecovCnt_nullParam_pmicHandle); \
                            PLATFORM_RUN_TEST(test_negative_Pmic_getRecovCnt_nullParam_recovCnt); \
                            PLATFORM_RUN_TEST(test_negative_Pmic_clrRecovCnt_nullParam_pmicHandle); \
                            PLATFORM_RUN_TEST(test_negative_Pmic_setResetCntThr_nullParam_pmicHandle); \
                            PLATFORM_RUN_TEST(test_negative_Pmic_setResetCntThr_outOfBounds_threshold); \
                            PLATFORM_RUN_TEST(test_negative_Pmic_getResetCntThr_nullParam_pmicHandle); \
                            PLATFORM_RUN_TEST(test_negative_Pmic_getResetCntThr_nullParam_threshold); \
                            PLATFORM_RUN_TEST(test_negative_Pmic_getResetCnt_nullParam_pmicHandle); \
                            PLATFORM_RUN_TEST(test_negative_Pmic_getResetCnt_nullParam_resetCnt); \
                            PLATFORM_RUN_TEST(test_negative_Pmic_clrResetCnt_nullParam_pmicHandle); \
                            PLATFORM_RUN_TEST(test_positive_Pmic_getDevId); \
                            PLATFORM_RUN_TEST(test_positive_Pmic_getNvmId); \
                            PLATFORM_RUN_TEST(test_positive_Pmic_getNvmRev); \
                            PLATFORM_RUN_TEST(test_positive_Pmic_getSiliconRev); \
                            PLATFORM_RUN_TEST(test_positive_setGetRegLock); \
                            PLATFORM_RUN_TEST(test_positive_enableDisableCRC8); \
                            PLATFORM_RUN_TEST(test_positive_setGetPwrOn); \
                            PLATFORM_RUN_TEST(test_positive_setGetLpmCfg_pinDetection); \
                            PLATFORM_RUN_TEST(test_positive_setGetLpmCfg_detectionDelay); \
                            PLATFORM_RUN_TEST(test_positive_setGetLpmCfg_vmonEn); \
                            PLATFORM_RUN_TEST(test_positive_setGetLpmCfg_esmEn); \
                            PLATFORM_RUN_TEST(test_positive_setGetLpmCfg_wdgEn); \
                            PLATFORM_RUN_TEST(test_positive_Pmic_runABIST); \
                            PLATFORM_RUN_TEST(test_positive_setGetScratchPadVal); \
                            PLATFORM_RUN_TEST(test_positive_setGetRecovCntThr); \
                            PLATFORM_RUN_TEST(test_positive_setGetResetCntThr); \
                            PLATFORM_RUN_TEST(test_positive_getClrRecovCnt); \
                            PLATFORM_RUN_TEST(test_positive_getClrResetCnt)

/* Run all Core negative tests */
#define CORE_TEST_RUN_NEGATIVE() PLATFORM_RUN_TEST(test_negative_Pmic_getDevId_nullParam_pmicHandle); \
                                 PLATFORM_RUN_TEST(test_negative_Pmic_getDevId_nullParam_devId); \
                                 PLATFORM_RUN_TEST(test_negative_Pmic_getNvmId_nullParam_pmicHandle); \
                                 PLATFORM_RUN_TEST(test_negative_Pmic_getNvmId_nullParam_nvmId); \
                                 PLATFORM_RUN_TEST(test_negative_Pmic_getNvmRev_nullParam_pmicHandle); \
                                 PLATFORM_RUN_TEST(test_negative_Pmic_getNvmRev_nullParam_nvmRev); \
                                 PLATFORM_RUN_TEST(test_negative_Pmic_getSiliconRev_nullParam_pmicHandle); \
                                 PLATFORM_RUN_TEST(test_negative_Pmic_getSiliconRev_nullParam_siliconRev); \
                                 PLATFORM_RUN_TEST(test_negative_Pmic_setRegLock_nullParam_pmicHandle); \
                                 PLATFORM_RUN_TEST(test_negative_Pmic_unlockRegs_nullParam_pmicHandle); \
                                 PLATFORM_RUN_TEST(test_negative_Pmic_lockRegs_nullParam_pmicHandle); \
                                 PLATFORM_RUN_TEST(test_negative_Pmic_getRegLock_nullParam_pmicHandle); \
                                 PLATFORM_RUN_TEST(test_negative_Pmic_getRegLock_nullParam_regLockStat); \
                                 PLATFORM_RUN_TEST(test_negative_Pmic_enableDisableCRC8_nullParam_pmicHandle); \
                                 PLATFORM_RUN_TEST(test_negative_Pmic_enableCRC8_nullParam_pmicHandle); \
                                 PLATFORM_RUN_TEST(test_negative_Pmic_disableCRC8_nullParam_pmicHandle); \
                                 PLATFORM_RUN_TEST(test_negative_Pmic_getCRC8Enable_nullParam_pmicHandle); \
                                 PLATFORM_RUN_TEST(test_negative_Pmic_getCRC8Enable_nullParam_crcEnabled); \
                                 PLATFORM_RUN_TEST(test_negative_Pmic_sendFsmCmd_nullParam_pmicHandle); \
                                 PLATFORM_RUN_TEST(test_negative_Pmic_sendFsmCmd_invalid_fsmCmd); \
                                 PLATFORM_RUN_TEST(test_negative_Pmic_setPwrOn_nullParam_pmicHandle); \
                                 PLATFORM_RUN_TEST(test_negative_Pmic_getPwrOn_nullParam_pmicHandle); \
                                 PLATFORM_RUN_TEST(test_negative_Pmic_getPwrOn_nullParam_pwrOnStat); \
                                 PLATFORM_RUN_TEST(test_negative_Pmic_setLpmCfg_nullParam_pmicHandle); \
                                 PLATFORM_RUN_TEST(test_negative_Pmic_setLpmCfg_nullParam_lpmCfg); \
                                 PLATFORM_RUN_TEST(test_negative_Pmic_setLpmCfg_outOfBounds_pinDetection); \
                                 PLATFORM_RUN_TEST(test_negative_Pmic_setLpmCfg_outOfBounds_detectionDelay); \
                                 PLATFORM_RUN_TEST(test_negative_Pmic_getLpmCfg_nullParam_pmicHandle); \
                                 PLATFORM_RUN_TEST(test_negative_Pmic_getLpmCfg_nullParam_lpmCfg); \
                                 PLATFORM_RUN_TEST(test_negative_Pmic_runABIST_nullParam_pmicHandle); \
                                 PLATFORM_RUN_TEST(test_negative_Pmic_getABISTStat_nullParam_pmicHandle); \
                                 PLATFORM_RUN_TEST(test_negative_Pmic_getABISTStat_nullParam_isActive); \
                                 PLATFORM_RUN_TEST(test_negative_Pmic_setScratchPadVal_nullParam_pmicHandle); \
                                 PLATFORM_RUN_TEST(test_negative_Pmic_setScratchPadVal_outOfBounds_scratchPadRegNum); \
                                 PLATFORM_RUN_TEST(test_negative_Pmic_getScratchPadVal_nullParam_pmicHandle); \
                                 PLATFORM_RUN_TEST(test_negative_Pmic_getScratchPadVal_outOfBounds_scratchPadRegNum); \
                                 PLATFORM_RUN_TEST(test_negative_Pmic_getScratchPadVal_nullParam_value); \
                                 PLATFORM_RUN_TEST(test_negative_Pmic_setRecovCntThr_nullParam_pmicHandle); \
                                 PLATFORM_RUN_TEST(test_negative_Pmic_setRecovCntThr_outOfBounds_threshold); \
                                 PLATFORM_RUN_TEST(test_negative_Pmic_getRecovCntThr_nullParam_pmicHandle); \
                                 PLATFORM_RUN_TEST(test_negative_Pmic_getRecovCntThr_nullParam_threshold); \
                                 PLATFORM_RUN_TEST(test_negative_Pmic_clrRecovCnt_nullParam_pmicHandle); \
                                 PLATFORM_RUN_TEST(test_negative_Pmic_setResetCntThr_nullParam_pmicHandle); \
                                 PLATFORM_RUN_TEST(test_negative_Pmic_setResetCntThr_outOfBounds_threshold); \
                                 PLATFORM_RUN_TEST(test_negative_Pmic_getResetCntThr_nullParam_pmicHandle); \
                                 PLATFORM_RUN_TEST(test_negative_Pmic_getResetCntThr_nullParam_threshold); \
                                 PLATFORM_RUN_TEST(test_negative_Pmic_clrResetCnt_nullParam_pmicHandle)

/* Run all Core positive tests */
#define CORE_TEST_RUN_POSITIVE() PLATFORM_RUN_TEST(test_positive_Pmic_getDevId); \
                                 PLATFORM_RUN_TEST(test_positive_Pmic_getNvmId); \
                                 PLATFORM_RUN_TEST(test_positive_Pmic_getNvmRev); \
                                 PLATFORM_RUN_TEST(test_positive_Pmic_getSiliconRev); \
                                 PLATFORM_RUN_TEST(test_positive_setGetRegLock); \
                                 PLATFORM_RUN_TEST(test_positive_enableDisableCRC8); \
                                 PLATFORM_RUN_TEST(test_positive_setGetPwrOn); \
                                 PLATFORM_RUN_TEST(test_positive_setGetLpmCfg_pinDetection); \
                                 PLATFORM_RUN_TEST(test_positive_setGetLpmCfg_detectionDelay); \
                                 PLATFORM_RUN_TEST(test_positive_setGetLpmCfg_vmonEn); \
                                 PLATFORM_RUN_TEST(test_positive_setGetLpmCfg_esmEn); \
                                 PLATFORM_RUN_TEST(test_positive_setGetLpmCfg_wdgEn); \
                                 PLATFORM_RUN_TEST(test_positive_Pmic_runABIST); \
                                 PLATFORM_RUN_TEST(test_positive_setGetScratchPadVal); \
                                 PLATFORM_RUN_TEST(test_positive_setGetRecovCntThr); \
                                 PLATFORM_RUN_TEST(test_positive_setGetResetCntThr); \
                                 PLATFORM_RUN_TEST(test_positive_getClrRecovCnt); \
                                 PLATFORM_RUN_TEST(test_positive_getClrResetCnt)

/* ========================================================================== */
/*                             Global Variables                               */
/* ========================================================================== */

static Pmic_CoreHandle_t pmicHandle = {0U};

/* ========================================================================== */
/*                           Function Declarations                            */
/* ========================================================================== */

static int32_t coreTest_clrResetCnt(void);
static int32_t coreTest_clrRecovCnt(void);

/* ========================================================================== */
/*                           Function Definitions                             */
/* ========================================================================== */

void core_test(void *args)
{
    char msg[50U] = {0};
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_CoreCfg_t pmicCfg = {
        .i2cAddr = PLATFORM_TARGET_I2C_ADDR,
        .commHandle = platform_getCommHandle(),
        .ioRead = &platform_rxByte,
        .ioWrite = &platform_txByte,
        .critSecStart = &platform_critSecStart,
        .critSecStop = &platform_critSecStop,
        .irqResponse = &platform_irqResponse
    };

    platform_init();

    platform_printString("\r\n");
    platform_printString("CORE_TEST\r\n");
    platform_printString("-------\r\n\r\n");

    status = Pmic_init(&pmicCfg, &pmicHandle);

    if (status == PMIC_ST_SUCCESS)
    {
        platform_setupTests();
        CORE_TEST_RUN_ALL();
        platform_tearDownTests();
    }
    else
    {
        (void)sprintf(msg, "Error in initializing PMIC LLD: %d\r\n", status);
        platform_printString(msg);
    }

    (void)Pmic_deinit(&pmicHandle);
    platform_deinit();
}

void test_negative_Pmic_getDevId_nullParam_pmicHandle(void)
{
    // Pass NULL pmicHandle into Pmic_getDevId()
    uint8_t devId = 0U;
    int32_t status = Pmic_getDevId(NULL, &devId);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_getDevId_nullParam_devId(void)
{
    // Pass NULL devId into Pmic_getDevId()
    int32_t status = Pmic_getDevId(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_getNvmId_nullParam_pmicHandle(void)
{
    // Pass NULL pmicHandle into Pmic_getNvmId()
    uint8_t nvmId = 0U;
    int32_t status = Pmic_getNvmId(NULL, &nvmId);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_getNvmId_nullParam_nvmId(void)
{
    // Pass NULL nvmId into Pmic_getNvmId()
    int32_t status = Pmic_getNvmId(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_getNvmRev_nullParam_pmicHandle(void)
{
    // Pass NULL pmicHandle into Pmic_getNvmRev()
    uint8_t nvmRev = 0U;
    int32_t status = Pmic_getNvmRev(NULL, &nvmRev);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_getNvmRev_nullParam_nvmRev(void)
{
    // Pass NULL nvmRev into Pmic_getNvmRev()
    int32_t status = Pmic_getNvmRev(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_getSiliconRev_nullParam_pmicHandle(void)
{
    // Pass NULL pmicHandle into Pmic_getSiliconRev()
    uint8_t siliconRev = 0U;
    int32_t status = Pmic_getSiliconRev(NULL, &siliconRev);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_getSiliconRev_nullParam_siliconRev(void)
{
    // Pass NULL siliconRev into Pmic_getSiliconRev()
    int32_t status = Pmic_getSiliconRev(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_setRegLock_nullParam_pmicHandle(void)
{
    // Pass NULL pmicHandle into Pmic_setRegLock()
    int32_t status = Pmic_setRegLock(NULL, PMIC_LOCK);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_unlockRegs_nullParam_pmicHandle(void)
{
    // Pass NULL pmicHandle into Pmic_unlockRegs()
    int32_t status = Pmic_unlockRegs(NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_lockRegs_nullParam_pmicHandle(void)
{
    // Pass NULL pmicHandle into Pmic_lockRegs()
    int32_t status = Pmic_lockRegs(NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_getRegLock_nullParam_pmicHandle(void)
{
    // Pass NULL pmicHandle into Pmic_getRegLock()
    bool isLocked = (bool)false;
    int32_t status = Pmic_getRegLock(NULL, &isLocked);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_getRegLock_nullParam_regLockStat(void)
{
    // Pass NULL regLockStat into Pmic_getRegLock()
    int32_t status = Pmic_getRegLock(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_enableDisableCRC8_nullParam_pmicHandle(void)
{
    // Pass NULL pmicHandle into Pmic_enableDisableCRC8()
    int32_t status = Pmic_enableDisableCRC8(NULL, PMIC_ENABLE);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_enableCRC8_nullParam_pmicHandle(void)
{
    // Pass NULL pmicHandle into Pmic_enableCRC8()
    int32_t status = Pmic_enableCRC8(NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_disableCRC8_nullParam_pmicHandle(void)
{
    // Pass NULL pmicHandle into Pmic_disableCRC8()
    int32_t status = Pmic_disableCRC8(NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_getCRC8Enable_nullParam_pmicHandle(void)
{
    // Pass NULL pmicHandle into Pmic_getCRC8Enable()
    bool isEnabled = PMIC_DISABLE;
    int32_t status = Pmic_getCRC8Enable(NULL, &isEnabled);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_getCRC8Enable_nullParam_crcEnabled(void)
{
    // Pass NULL crcEnabled into Pmic_getCRC8Enable()
    int32_t status = Pmic_getCRC8Enable(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_sendFsmCmd_nullParam_pmicHandle(void)
{
    // Pass NULL pmicHandle into Pmic_sendFsmCmd()
    int32_t status = Pmic_sendFsmCmd(NULL, PMIC_WARM_RESET_REQUEST);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_sendFsmCmd_invalid_fsmCmd(void)
{
    // Pass invalid fsmCmd into Pmic_sendFsmCmd
    int32_t status = Pmic_sendFsmCmd(&pmicHandle, 0x00U);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_negative_Pmic_setPwrOn_nullParam_pmicHandle(void)
{
    // Pass NULL pmicHandle into Pmic_setPwrOn()
    int32_t status = Pmic_setPwrOn(NULL, PMIC_ENABLE);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_getPwrOn_nullParam_pmicHandle(void)
{
    // Pass NULL pmicHandle into Pmic_getPwrOn()
    bool isEnabled = PMIC_DISABLE;
    int32_t status = Pmic_getPwrOn(NULL, &isEnabled);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_getPwrOn_nullParam_pwrOnStat(void)
{
    // Pass NULL pwrOnStat into Pmic_getPwrOn()
    int32_t status = Pmic_getPwrOn(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_setLpmCfg_nullParam_pmicHandle(void)
{
    // Pass NULL pmicHandle into Pmic_setLpmCfg()
    Pmic_CoreLpmCfg_t lpmCfg = {
        .validParams = PMIC_LPM_VMON_EN_VALID,
        .vmonEn = PMIC_ENABLE
    };
    int32_t status = Pmic_setLpmCfg(NULL, &lpmCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_setLpmCfg_nullParam_lpmCfg(void)
{
    // Pass NULL lpmCfg into Pmic_setLpmCfg()
    int32_t status = Pmic_setLpmCfg(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_setLpmCfg_outOfBounds_pinDetection(void)
{
    // Pass out of bounds pinDetection into Pmic_setLpmCfg()
    Pmic_CoreLpmCfg_t lpmCfg = {
        .validParams = PMIC_LPM_PIN_DETECTION_VALID,
        .pinDetection = PMIC_PIN_DETECTION_CONDITION_MAX + 1U
    };
    int32_t status = Pmic_setLpmCfg(&pmicHandle, &lpmCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_negative_Pmic_setLpmCfg_outOfBounds_detectionDelay(void)
{
    // Pass out of bounds detectionDelay into Pmic_setLpmCfg()
    Pmic_CoreLpmCfg_t lpmCfg = {
        .validParams = PMIC_LPM_DETECTION_DELAY_VALID,
        .detectionDelay = PMIC_DETECTION_DELAY_MAX + 1U
    };
    int32_t status = Pmic_setLpmCfg(&pmicHandle, &lpmCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_negative_Pmic_getLpmCfg_nullParam_pmicHandle(void)
{
    // Pass NULL pmicHandle into Pmic_getLpmCfg()
    Pmic_CoreLpmCfg_t lpmCfg = {.validParams = PMIC_LPM_VMON_EN_VALID};
    int32_t status = Pmic_getLpmCfg(NULL, &lpmCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_getLpmCfg_nullParam_lpmCfg(void)
{
    // Pass NULL lpmCfg into Pmic_getLpmCfg()
    int32_t status = Pmic_getLpmCfg(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_runABIST_nullParam_pmicHandle(void)
{
    // Pass NULL pmicHandle into Pmic_runABIST()
    int32_t status = Pmic_runABIST(NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_getABISTStat_nullParam_pmicHandle(void)
{
    // Pass NULL pmicHandle into Pmic_getABISTStat()
    bool isActive = (bool)false;
    int32_t status = Pmic_getABISTStat(NULL, &isActive);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_getABISTStat_nullParam_isActive(void)
{
    // Pass NULL isActive into Pmic_getABISTStat()
    int32_t status = Pmic_getABISTStat(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_setScratchPadVal_nullParam_pmicHandle(void)
{
    // Pass NULL pmicHandle into Pmic_setScratchPadVal()
    int32_t status = Pmic_setScratchPadVal(NULL, PMIC_SCRATCH_PAD_REG_1, 0xAAU);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_setScratchPadVal_outOfBounds_scratchPadRegNum(void)
{
    // Pass out of bounds scratchPadRegNum into Pmic_setScratchPadVal()
    int32_t status = Pmic_setScratchPadVal(&pmicHandle, PMIC_SCRATCH_PAD_REG_MAX + 1U, 0xAAU);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_negative_Pmic_getScratchPadVal_nullParam_pmicHandle(void)
{
    // Pass NULL pmicHandle into Pmic_getScratchPadVal()
    uint8_t value = 0U;
    int32_t status = Pmic_getScratchPadVal(NULL, PMIC_SCRATCH_PAD_REG_1, &value);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_getScratchPadVal_outOfBounds_scratchPadRegNum(void)
{
    // Pass out of bounds scratchPadRegNum into Pmic_getScratchPadVal()
    uint8_t value = 0U;
    int32_t status = Pmic_getScratchPadVal(&pmicHandle, PMIC_SCRATCH_PAD_REG_MAX + 1U, &value);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_negative_Pmic_getScratchPadVal_nullParam_value(void)
{
    // Pass NULL value into Pmic_getScratchPadVal()
    int32_t status = Pmic_getScratchPadVal(&pmicHandle, PMIC_SCRATCH_PAD_REG_1, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_setRecovCntThr_nullParam_pmicHandle(void)
{
    // Pass NULL pmicHandle into Pmic_setRecovCntThr()
    int32_t status = Pmic_setRecovCntThr(NULL, PMIC_RESET_RECOV_CNT_THR_MAX);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_setRecovCntThr_outOfBounds_threshold(void)
{
    // Pass out of bounds threshold into Pmic_setRecovCntThr()
    int32_t status = Pmic_setRecovCntThr(&pmicHandle, PMIC_RESET_RECOV_CNT_THR_MAX + 1U);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_negative_Pmic_getRecovCntThr_nullParam_pmicHandle(void)
{
    // Pass NULL pmicHandle into Pmic_getRecovCntThr()
    uint8_t threshold = 0U;
    int32_t status = Pmic_getRecovCntThr(NULL, &threshold);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_getRecovCntThr_nullParam_threshold(void)
{
    // Pass NULL threshold into Pmic_getRecovCntThr()
    int32_t status = Pmic_getRecovCntThr(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_getRecovCnt_nullParam_pmicHandle(void)
{
    // Pass NULL pmicHandle into Pmic_getRecovCnt()
    uint8_t recovCnt = 0U;
    int32_t status = Pmic_getRecovCnt(NULL, &recovCnt);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_getRecovCnt_nullParam_recovCnt(void)
{
    // Pass NULL recovCnt into Pmic_getRecovCnt()
    int32_t status = Pmic_getRecovCnt(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_clrRecovCnt_nullParam_pmicHandle(void)
{
    // Pass NULL pmicHandle into Pmic_clrRecovCnt()
    int32_t status = Pmic_clrRecovCnt(NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_setResetCntThr_nullParam_pmicHandle(void)
{
    // Pass NULL pmicHandle into Pmic_setResetCntThr()
    int32_t status = Pmic_setResetCntThr(NULL, PMIC_RESET_RECOV_CNT_THR_MAX);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_setResetCntThr_outOfBounds_threshold(void)
{
    // Pass out of bounds threshold into Pmic_setResetCntThr()
    int32_t status = Pmic_setResetCntThr(&pmicHandle, PMIC_RESET_RECOV_CNT_THR_MAX + 1U);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_negative_Pmic_getResetCntThr_nullParam_pmicHandle(void)
{
    // Pass NULL pmicHandle into Pmic_getResetCntThr()
    uint8_t threshold = 0U;
    int32_t status = Pmic_getResetCntThr(NULL, &threshold);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_getResetCntThr_nullParam_threshold(void)
{
    // Pass NULL threshold into Pmic_getResetCntThr()
    int32_t status = Pmic_getResetCntThr(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_getResetCnt_nullParam_pmicHandle(void)
{
    // Pass NULL pmicHandle into Pmic_getResetCnt()
    uint8_t resetCnt = 0U;
    int32_t status = Pmic_getResetCnt(NULL, &resetCnt);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_getResetCnt_nullParam_resetCnt(void)
{
    // Pass NULL resetCnt into Pmic_getResetCnt()
    int32_t status = Pmic_getResetCnt(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_clrResetCnt_nullParam_pmicHandle(void)
{
    // Pass NULL pmicHandle into Pmic_clrResetCnt()
    int32_t status = Pmic_clrResetCnt(NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_positive_Pmic_getDevId(void)
{
    // Get PMIC device ID
    uint8_t devId = 0xFFU;
    int32_t status = Pmic_getDevId(&pmicHandle, &devId);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(devId != 0xFFU);
}

void test_positive_Pmic_getNvmId(void)
{
    // Get PMIC NVM ID
    uint8_t nvmId = 0xFFU;
    int32_t status = Pmic_getNvmId(&pmicHandle, &nvmId);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(nvmId != 0xFFU);
}

void test_positive_Pmic_getNvmRev(void)
{
    // Get PMIC NVM revision ID
    uint8_t nvmRev = 0xFFU;
    int32_t status = Pmic_getNvmRev(&pmicHandle, &nvmRev);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(nvmRev != 0xFFU);
}

void test_positive_Pmic_getSiliconRev(void)
{
    // Get PMIC silicon revision
    uint8_t siliconRev = 0xFFU;
    int32_t status = Pmic_getSiliconRev(&pmicHandle, &siliconRev);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(siliconRev != 0xFFU);
}

void test_positive_setGetRegLock(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    bool isLocked = (bool)false;

    // Lock PMIC registers
    status = Pmic_setRegLock(&pmicHandle, PMIC_LOCK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get actual register lock status and compare expected vs. actual values
    status = Pmic_getRegLock(&pmicHandle, &isLocked);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(isLocked == (bool)true);

    // Unlock PMIC registers
    status = Pmic_setRegLock(&pmicHandle, PMIC_UNLOCK);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get actual register lock status and compare expected vs. actual values
    status = Pmic_getRegLock(&pmicHandle, &isLocked);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(isLocked == (bool)false);
}

void test_positive_enableDisableCRC8(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    bool crcEnabled = (bool)false;

    // Enable CRC8
    status = Pmic_enableDisableCRC8(&pmicHandle, PMIC_ENABLE);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get actual CRC8 enable status and compare expected vs. actual values
    status = Pmic_getCRC8Enable(&pmicHandle, &crcEnabled);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(crcEnabled == (bool)true);

    // Disable CRC8
    status = Pmic_enableDisableCRC8(&pmicHandle, PMIC_DISABLE);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get actual CRC8 enable status and compare expected vs. actual values
    status = Pmic_getCRC8Enable(&pmicHandle, &crcEnabled);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(crcEnabled == (bool)false);
}

void test_positive_setGetPwrOn(void)
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

void test_positive_setGetLpmCfg_pinDetection(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_CoreLpmCfg_t expLpmCfg = {.validParams = PMIC_LPM_PIN_DETECTION_VALID};
    Pmic_CoreLpmCfg_t actLpmCfg = {.validParams = PMIC_LPM_PIN_DETECTION_VALID};

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

void test_positive_setGetLpmCfg_detectionDelay(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_CoreLpmCfg_t expLpmCfg = {.validParams = PMIC_LPM_DETECTION_DELAY_VALID};
    Pmic_CoreLpmCfg_t actLpmCfg = {.validParams = PMIC_LPM_DETECTION_DELAY_VALID};

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

void test_positive_setGetLpmCfg_vmonEn(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_CoreLpmCfg_t expLpmCfg = {.validParams = PMIC_LPM_VMON_EN_VALID};
    Pmic_CoreLpmCfg_t actLpmCfg = {.validParams = PMIC_LPM_VMON_EN_VALID};

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

void test_positive_setGetLpmCfg_esmEn(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_CoreLpmCfg_t expLpmCfg = {.validParams = PMIC_LPM_ESM_EN_VALID};
    Pmic_CoreLpmCfg_t actLpmCfg = {.validParams = PMIC_LPM_ESM_EN_VALID};

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

void test_positive_setGetLpmCfg_wdgEn(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_CoreLpmCfg_t expLpmCfg = {.validParams = PMIC_LPM_WDG_EN_VALID};
    Pmic_CoreLpmCfg_t actLpmCfg = {.validParams = PMIC_LPM_WDG_EN_VALID};

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

void test_positive_Pmic_runABIST(void)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;
    const uint8_t maskMiscRegAddr = 0x38U, intMiscRegAddr = 0x53U, bufLen = 1U, abistDoneShift = 0U;

    // Clear all PMIC IRQs
    status = testCommon_clrAllPmicIrq(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Unmask ABIST_DONE_MASK
    status = platform_rxByte(&pmicHandle, maskMiscRegAddr, bufLen, &regData);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    Pmic_setBitField(&regData, abistDoneShift, 1U << abistDoneShift, 0U);
    status = platform_txByte(&pmicHandle, maskMiscRegAddr, bufLen, &regData);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Run ABIST
    status = Pmic_runABIST(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Validate ABIST was run by checking ABIST_DONE_INT
    status = platform_rxByte(&pmicHandle, intMiscRegAddr, bufLen, &regData);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(Pmic_getBitField_b(regData, abistDoneShift) == (bool)true);

    // Clear all PMIC IRQs
    status = testCommon_clrAllPmicIrq(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_positive_setGetScratchPadVal(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t initVal = 0U, actVal = 0U, expVal = 0U;

    // For each scratchpad register...
    for (uint8_t scratchpadReg = PMIC_SCRATCH_PAD_REG_1; scratchpadReg <= PMIC_SCRATCH_PAD_REG_MAX; scratchpadReg++)
    {
        // Get initial value
        status = Pmic_getScratchPadVal(&pmicHandle, scratchpadReg, &initVal);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        // Set expected value (inverted initial value)
        expVal = ~initVal;
        status = Pmic_setScratchPadVal(&pmicHandle, scratchpadReg, expVal);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        // Get actual value and compare expected vs. actual
        status = Pmic_getScratchPadVal(&pmicHandle, scratchpadReg, &actVal);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        PLATFORM_ASSERT(initVal != actVal);
        PLATFORM_ASSERT(expVal == actVal);
    }
}

static int32_t coreTest_clrResetCnt(void)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;
    const uint8_t recovCntControlRegAddr = 0x07U, bufLen = 1U, resetCntClrShift = 1U, resetCntClrMask = 1U << 1U;

    // Read RECOV_CNT_CONTROL
    status = platform_rxByte(&pmicHandle, recovCntControlRegAddr, bufLen, &regData);

    // Set RESET_CNT_CLR bit field to 1 and write RECOV_CNT_CONTROL
    if (status == PMIC_ST_SUCCESS)
    {
        Pmic_setBitField(&regData, resetCntClrShift, resetCntClrMask, 1U);
        status = platform_txByte(&pmicHandle, recovCntControlRegAddr, bufLen, &regData);
    }

    return status;
}

static int32_t coreTest_clrRecovCnt(void)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;
    const uint8_t recovCntControlRegAddr = 0x07U, bufLen = 1U, recovCntClrShift = 0U, recovCntClrMask = 1U << 0U;

    // Read RECOV_CNT_CONTROL
    status = platform_rxByte(&pmicHandle, recovCntControlRegAddr, bufLen, &regData);

    // Set RECOV_CNT_CLR bit field to 1 and write RECOV_CNT_CONTROL
    if (status == PMIC_ST_SUCCESS)
    {
        Pmic_setBitField(&regData, recovCntClrShift, recovCntClrMask, 1U);
        status = platform_txByte(&pmicHandle, recovCntControlRegAddr, bufLen, &regData);
    }

    return status;
}

void test_positive_setGetRecovCntThr(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t actThreshold = 0U;

    // clear recovery counter
    status = coreTest_clrRecovCnt();
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // For each valid threshold value...
    for (uint8_t expThreshold = 0U; expThreshold <= PMIC_RESET_RECOV_CNT_THR_MAX; expThreshold++)
    {
        // Set expected threshold value
        status = Pmic_setRecovCntThr(&pmicHandle, expThreshold);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        // Get actual threshold value and compare expected vs. actual values
        status = Pmic_getRecovCntThr(&pmicHandle, &actThreshold);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        PLATFORM_ASSERT(expThreshold == actThreshold);
    }
}

void test_positive_setGetResetCntThr(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t actThreshold = 0U;

    // clear reset counter
    status = coreTest_clrResetCnt();
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // For each valid threshold value...
    for (uint8_t expThreshold = 0U; expThreshold <= PMIC_RESET_RECOV_CNT_THR_MAX; expThreshold++)
    {
        // Set expected threshold value
        status = Pmic_setResetCntThr(&pmicHandle, expThreshold);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        // Get actual threshold value and compare expected vs. actual values
        status = Pmic_getResetCntThr(&pmicHandle, &actThreshold);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        PLATFORM_ASSERT(expThreshold == actThreshold);
    }
}

/*
 * NOTE: This test puts the PMIC in SAFE state, which affects I2C communication.
 * As a result, ignore all I2C communication errors after sending Safe Recovery
 * Request.
 */
void test_positive_getClrRecovCnt(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t initRecovCnt = 0U, newRecovCnt = 0U;

    // Get initial recovery count
    status = Pmic_getRecovCnt(&pmicHandle, &initRecovCnt);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Send FSM command to enter safe state
    status = Pmic_sendFsmCmd(&pmicHandle, PMIC_SAFE_RECOVERY_REQUEST);
    PLATFORM_ASSERT((status == PMIC_ST_SUCCESS) || (status == PMIC_ST_ERR_I2C_COMM_FAIL));

    // Clear all IRQs and unlock PMIC registers
    status = testCommon_clrAllPmicIrq(&pmicHandle);
    PLATFORM_ASSERT((status == PMIC_ST_SUCCESS) || (status == PMIC_ST_ERR_I2C_COMM_FAIL));
    status = testCommon_unlockPmicRegs(&pmicHandle);
    PLATFORM_ASSERT((status == PMIC_ST_SUCCESS) || (status == PMIC_ST_ERR_I2C_COMM_FAIL));

    // Get new recovery count and compare initial vs. new recovery count
    status = Pmic_getRecovCnt(&pmicHandle, &newRecovCnt);
    PLATFORM_ASSERT((status == PMIC_ST_SUCCESS) || (status == PMIC_ST_ERR_I2C_COMM_FAIL));
    PLATFORM_ASSERT(newRecovCnt = (initRecovCnt + 1U));

    // Clear the recovery counter
    status = Pmic_clrRecovCnt(&pmicHandle);
    PLATFORM_ASSERT((status == PMIC_ST_SUCCESS) || (status == PMIC_ST_ERR_I2C_COMM_FAIL));

    // Get new recovery count and compare expected vs. actual value
    status = Pmic_getRecovCnt(&pmicHandle, &newRecovCnt);
    PLATFORM_ASSERT((status == PMIC_ST_SUCCESS) || (status == PMIC_ST_ERR_I2C_COMM_FAIL));
    PLATFORM_ASSERT(newRecovCnt == 0U);
}

/*
 * NOTE: This test makes the PMIC undergo WARM RESET, which affects I2C communication.
 * As a result, ignore all I2C communication errors after sending WARM RESET Request.
 */
void test_positive_getClrResetCnt(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t initResetCnt = 0U, newResetCnt = 0U;

    // Get initial reset count
    status = Pmic_getResetCnt(&pmicHandle, &initResetCnt);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Send FSM command for warm reset
    status = Pmic_sendFsmCmd(&pmicHandle, PMIC_WARM_RESET_REQUEST);
    PLATFORM_ASSERT((status == PMIC_ST_SUCCESS) || (status == PMIC_ST_ERR_I2C_COMM_FAIL));

    // Clear all IRQs and unlock PMIC registers
    status = testCommon_clrAllPmicIrq(&pmicHandle);
    PLATFORM_ASSERT((status == PMIC_ST_SUCCESS) || (status == PMIC_ST_ERR_I2C_COMM_FAIL));
    status = testCommon_unlockPmicRegs(&pmicHandle);
    PLATFORM_ASSERT((status == PMIC_ST_SUCCESS) || (status == PMIC_ST_ERR_I2C_COMM_FAIL));

    // Get new reset count and compare initial vs. new reset count
    status = Pmic_getRecovCnt(&pmicHandle, &newResetCnt);
    PLATFORM_ASSERT((status == PMIC_ST_SUCCESS) || (status == PMIC_ST_ERR_I2C_COMM_FAIL));
    PLATFORM_ASSERT(newResetCnt = (initResetCnt + 1U));

    // Clear the reset counter
    status = Pmic_clrResetCnt(&pmicHandle);
    PLATFORM_ASSERT((status == PMIC_ST_SUCCESS) || (status == PMIC_ST_ERR_I2C_COMM_FAIL));

    // Get new reset count and compare expected vs. actual value
    status = Pmic_getResetCnt(&pmicHandle, &newResetCnt);
    PLATFORM_ASSERT((status == PMIC_ST_SUCCESS) || (status == PMIC_ST_ERR_I2C_COMM_FAIL));
    PLATFORM_ASSERT(newResetCnt == 0U);
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
