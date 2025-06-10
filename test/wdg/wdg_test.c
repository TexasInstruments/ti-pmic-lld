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
 * @brief Source file containing definitions to PMIC WDG tests.
 */

/* ========================================================================== */
/*                              Include Files                                 */
/* ========================================================================== */

#include "wdg_test.h"

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/* Run all WDG tests */
#define WDG_TEST_RUN_ALL() PLATFORM_RUN_TEST(test_negative_Pmic_wdgEnable_nullParam_handle); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_wdgDisable_nullParam_handle); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_wdgSetEnableState_nullParam_handle); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_wdgGetEnableState_nullParam_handle); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_wdgGetEnableState_nullParam_isEnabled); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_wdgSetCfg_nullParam_handle); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_wdgSetCfg_nullParam_wdgCfg); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_wdgSetCfg_outOfBounds_thresholdReset); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_wdgSetCfg_outOfBounds_thresholdFail); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_wdgSetCfg_outOfBounds_win1Code); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_wdgSetCfg_outOfBounds_win2Code); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_wdgSetCfg_outOfBounds_qaFdbk); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_wdgSetCfg_outOfBounds_qaLfsr); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_wdgSetCfg_outOfBounds_qaQuesSeed); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_wdgGetCfg_nullParam_handle); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_wdgGetCfg_nullParam_wdgCfg); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_wdgSetPowerHold_nullParam_handle); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_wdgGetPowerHold_nullParam_handle); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_wdgGetPowerHold_nullParam_isEnabled); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_wdgSetReturnToLongWindow_nullParam_handle); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_wdgGetReturnToLongWindow_nullParam_handle); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_wdgGetReturnToLongWindow_nullParam_isEnabled); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_wdgGetErrorStatus_nullParam_handle); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_wdgGetErrorStatus_nullParam_errors); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_wdgClrErrStatus_nullParam_handle); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_wdgClrErrStatus_nullParam_errors); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_wdgClrErrStatusAll_nullParam_handle); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_wdgGetFailCntStat_nullParam_handle); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_wdgGetFailCntStat_nullParam_failCount); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_wdgQaSequenceWriteAnswer_nullParam_handle); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_wdgGetFdbkRegData_nullParam_handle); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_wdgGetFdbkRegData_nullParam_regData); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_wdgExtractFdbk_nullParam_wdgAnsInfo); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_wdgGetAnsCntAndQuesRegData_nullParam_handle); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_wdgGetAnsCntAndQuesRegData_nullParam_regData); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_wdgExtractAnsCntAndQues_nullParam_handle); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_wdgExtractAnsCntAndQues_nullParam_wdgAnsInfo); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_wdgWriteAnswer_nullParam_handle); \
                           PLATFORM_RUN_TEST(test_negative_Pmic_wdgWriteAnswer_nullParam_wdgAnsInfo); \
                           PLATFORM_RUN_TEST(test_positive_wdgEnableDisable); \
                           PLATFORM_RUN_TEST(test_positive_wdgEnableDisablePowerHold); \
                           PLATFORM_RUN_TEST(test_positive_wdgEnableDisableReturnToLongWindow); \
                           PLATFORM_RUN_TEST(test_positive_wdgSetGetCfg_rstEn); \
                           PLATFORM_RUN_TEST(test_positive_wdgSetGetCfg_thresholdReset); \
                           PLATFORM_RUN_TEST(test_positive_wdgSetGetCfg_thresholdFail); \
                           PLATFORM_RUN_TEST(test_positive_wdgSetGetCfg_longWinCode); \
                           PLATFORM_RUN_TEST(test_positive_wdgSetGetCfg_win1Code); \
                           PLATFORM_RUN_TEST(test_positive_wdgSetGetCfg_win2Code); \
                           PLATFORM_RUN_TEST(test_positive_wdgSetGetCfg_qaFdbk); \
                           PLATFORM_RUN_TEST(test_positive_wdgSetGetCfg_qaLfsr); \
                           PLATFORM_RUN_TEST(test_positive_wdgSetGetCfg_qaQuesSeed); \
                           PLATFORM_RUN_TEST(test_positive_wdgQaSequence_detectNoErrors); \
                           PLATFORM_RUN_TEST(test_positive_wdgQaSequence_detectTimeout); \
                           PLATFORM_RUN_TEST(test_positive_wdgQaSequence_detectLongWindowTimeout); \
                           PLATFORM_RUN_TEST(test_positive_wdgQaSequence_detectAnswerEarlyError); \
                           PLATFORM_RUN_TEST(test_positive_wdgQaSequence_detectSequenceError); \
                           PLATFORM_RUN_TEST(test_positive_wdgQaSequence_detectAnswerError); \
                           PLATFORM_RUN_TEST(test_positive_wdgQaSequence_detectFailInt); \
                           PLATFORM_RUN_TEST(test_positive_wdgQaSequence_detectResetInt)

/* Run all WDG negative tests */
#define WDG_TEST_RUN_NEGATIVE() PLATFORM_RUN_TEST(test_negative_Pmic_wdgEnable_nullParam_handle); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_wdgDisable_nullParam_handle); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_wdgSetEnableState_nullParam_handle); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_wdgGetEnableState_nullParam_handle); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_wdgGetEnableState_nullParam_isEnabled); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_wdgSetCfg_nullParam_handle); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_wdgSetCfg_nullParam_wdgCfg); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_wdgSetCfg_outOfBounds_thresholdReset); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_wdgSetCfg_outOfBounds_thresholdFail); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_wdgSetCfg_outOfBounds_win1Code); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_wdgSetCfg_outOfBounds_win2Code); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_wdgSetCfg_outOfBounds_qaFdbk); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_wdgSetCfg_outOfBounds_qaLfsr); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_wdgSetCfg_outOfBounds_qaQuesSeed); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_wdgGetCfg_nullParam_handle); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_wdgGetCfg_nullParam_wdgCfg); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_wdgSetPowerHold_nullParam_handle); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_wdgGetPowerHold_nullParam_handle); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_wdgGetPowerHold_nullParam_isEnabled); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_wdgSetReturnToLongWindow_nullParam_handle); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_wdgGetReturnToLongWindow_nullParam_handle); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_wdgGetReturnToLongWindow_nullParam_isEnabled); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_wdgGetErrorStatus_nullParam_handle); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_wdgGetErrorStatus_nullParam_errors); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_wdgClrErrStatus_nullParam_handle); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_wdgClrErrStatus_nullParam_errors); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_wdgClrErrStatusAll_nullParam_handle); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_wdgGetFailCntStat_nullParam_handle); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_wdgGetFailCntStat_nullParam_failCount); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_wdgQaSequenceWriteAnswer_nullParam_handle); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_wdgGetFdbkRegData_nullParam_handle); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_wdgGetFdbkRegData_nullParam_regData); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_wdgExtractFdbk_nullParam_wdgAnsInfo); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_wdgGetAnsCntAndQuesRegData_nullParam_handle); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_wdgGetAnsCntAndQuesRegData_nullParam_regData); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_wdgExtractAnsCntAndQues_nullParam_handle); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_wdgExtractAnsCntAndQues_nullParam_wdgAnsInfo); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_wdgWriteAnswer_nullParam_handle); \
                                PLATFORM_RUN_TEST(test_negative_Pmic_wdgWriteAnswer_nullParam_wdgAnsInfo)

/* Run all WDG positive tests */
#define WDG_TEST_RUN_POSITIVE() PLATFORM_RUN_TEST(test_positive_wdgEnableDisable); \
                                PLATFORM_RUN_TEST(test_positive_wdgEnableDisablePowerHold); \
                                PLATFORM_RUN_TEST(test_positive_wdgEnableDisableReturnToLongWindow); \
                                PLATFORM_RUN_TEST(test_positive_wdgSetGetCfg_rstEn); \
                                PLATFORM_RUN_TEST(test_positive_wdgSetGetCfg_thresholdReset); \
                                PLATFORM_RUN_TEST(test_positive_wdgSetGetCfg_thresholdFail); \
                                PLATFORM_RUN_TEST(test_positive_wdgSetGetCfg_longWinCode); \
                                PLATFORM_RUN_TEST(test_positive_wdgSetGetCfg_win1Code); \
                                PLATFORM_RUN_TEST(test_positive_wdgSetGetCfg_win2Code); \
                                PLATFORM_RUN_TEST(test_positive_wdgSetGetCfg_qaFdbk); \
                                PLATFORM_RUN_TEST(test_positive_wdgSetGetCfg_qaLfsr); \
                                PLATFORM_RUN_TEST(test_positive_wdgSetGetCfg_qaQuesSeed); \
                                PLATFORM_RUN_TEST(test_positive_wdgQaSequence_detectNoErrors); \
                                PLATFORM_RUN_TEST(test_positive_wdgQaSequence_detectTimeout); \
                                PLATFORM_RUN_TEST(test_positive_wdgQaSequence_detectLongWindowTimeout); \
                                PLATFORM_RUN_TEST(test_positive_wdgQaSequence_detectAnswerEarlyError); \
                                PLATFORM_RUN_TEST(test_positive_wdgQaSequence_detectSequenceError); \
                                PLATFORM_RUN_TEST(test_positive_wdgQaSequence_detectAnswerError); \
                                PLATFORM_RUN_TEST(test_positive_wdgQaSequence_detectFailInt); \
                                PLATFORM_RUN_TEST(test_positive_wdgQaSequence_detectResetInt)

#define WDG_TEST_LONG_WINDOW_CODE_MAX (0xFFU)

/* ========================================================================== */
/*                             Global Variables                               */
/* ========================================================================== */
Pmic_CoreHandle_t pmicHandle;

/* ========================================================================== */
/*                           Function Declarations                            */
/* ========================================================================== */
static int32_t wdgTest_clrAllPmicIrq(void);
static int32_t wdgTest_unlockPmicRegs(void);
static void wdgTest_checkForWdgErrors(void);

/* ========================================================================== */
/*                           Function Definitions                             */
/* ========================================================================== */

void wdg_test(void *args)
{
    char msg[50U] = {0};
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_CoreCfg_t coreCfg = {
        .validParams = (PMIC_CFG_DEVICE_TYPE_VALID_SHIFT |
                        PMIC_CFG_COMM_MODE_VALID_SHIFT |
                        PMIC_CFG_SLAVEADDR_VALID_SHIFT |
                        PMIC_CFG_COMM_HANDLE_VALID_SHIFT |
                        PMIC_CFG_COMM_IO_RD_VALID_SHIFT |
                        PMIC_CFG_COMM_IO_WR_VALID_SHIFT |
                        PMIC_CFG_CRITSEC_START_VALID_SHIFT |
                        PMIC_CFG_CRITSEC_STOP_VALID_SHIFT |
                        PMIC_CFG_CRC_ENABLE_VALID_SHIFT |
                        PMIC_CFG_CFG_CRC_ENABLE_VALID_SHIFT |
                        PMIC_CFG_PSEUDO_IRQ_VALID_SHIFT),
        .instType = PMIC_MAIN_INST,
        .pmicDeviceType = PLATFORM_TARGET_DEV_TYPE,
        .commMode = PMIC_INTF_I2C_SINGLE,
        .slaveAddr = PLATFORM_TARGET_I2C_ADDR,
        .crcEnable = PMIC_DISABLE,
        .configCrcEnable = PMIC_DISABLE,
        .pCommHandle = platform_getCommHandle(),
        .pFnPmicCommIoRd = &platform_rxByte,
        .pFnPmicCommIoWr = &platform_txByte,
        .pFnPmicCritSecStart = &platform_critSecStart,
        .pFnPmicCritSecStop = &platform_critSecStop,
        .pFnPmicPseudoIrq = &platform_irqResponse
    };

    platform_init();

    platform_printString("\r\n");
    platform_printString("WDG_TEST\r\n");
    platform_printString("-------\r\n\r\n");

    status = Pmic_init(&pmicHandle, &coreCfg);

    if (status == PMIC_ST_SUCCESS)
    {
        status = wdgTest_unlockPmicRegs();

        if (status == PMIC_ST_SUCCESS)
        {
            status = wdgTest_clrAllPmicIrq();

            if (status == PMIC_ST_SUCCESS)
            {
                platform_setupTests();
                WDG_TEST_RUN_ALL();
                platform_tearDownTests();
            }
            else
            {
                (void)sprintf(msg, "Error in clearing all PMIC IRQs: %d\r\n", status);
                platform_printString(msg);
            }
        }
        else
        {
            (void)sprintf(msg, "Error in unlocking PMIC registers: %d\r\n", status);
            platform_printString(msg);
        }
    }
    else
    {
        (void)sprintf(msg, "Error in initializing PMIC LLD: %d\r\n", status);
        platform_printString(msg);
    }

    (void)Pmic_deinit(&pmicHandle);
    platform_deinit();
}

static int32_t wdgTest_unlockPmicRegs(void)
{
    uint8_t regData = 0x9BU;
    const uint8_t bufLen = 1U;
    const uint16_t registerLockAddr = 0x09U;

    // Check handle
    int32_t status = Pmic_checkPmicCoreHandle(&pmicHandle);

    // Write key to REGISTER_LOCK
    if (status == PMIC_ST_SUCCESS)
    {
        status = platform_txByte(&pmicHandle, PMIC_MAIN_INST, registerLockAddr, &regData, bufLen);
    }

    // Get register lock status
    if (status == PMIC_ST_SUCCESS)
    {
        status = platform_rxByte(&pmicHandle, PMIC_MAIN_INST, registerLockAddr, &regData, bufLen);
    }

    // Validate that registers are unlocked
    if ((status == PMIC_ST_SUCCESS) && (regData != 0U))
    {
        status = PMIC_ST_ERR_I2C_COMM_FAIL;
    }

    return status;
}

static int32_t wdgTest_clrAllPmicIrq(void)
{
    uint8_t regData = 0xFFU;
    int32_t status = PMIC_ST_SUCCESS;
    const uint8_t irqRegStart = 0x47U, irqRegEnd = 0x52U, bufLen = 1U;

    for (uint8_t regAddr = irqRegStart; regAddr <= irqRegEnd; regAddr++)
    {
        status = platform_txByte(&pmicHandle, PMIC_MAIN_INST, regAddr, &regData, bufLen);

        if (status != PMIC_ST_SUCCESS)
        {
            break;
        }
    }

    return status;
}

void test_negative_Pmic_wdgEnable_nullParam_handle(void)
{
    // Pass NULL handle into Pmic_wdgEnable()
    int32_t status = Pmic_wdgEnable(NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_HANDLE);
}

void test_negative_Pmic_wdgDisable_nullParam_handle(void)
{
    // Pass NULL handle into Pmic_wdgDisable()
    int32_t status = Pmic_wdgDisable(NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_HANDLE);
}

void test_negative_Pmic_wdgSetEnableState_nullParam_handle(void)
{
    // Pass NULL handle into Pmic_wdgSetEnableState()
    int32_t status = Pmic_wdgSetEnableState(NULL, PMIC_ENABLE);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_HANDLE);
}

void test_negative_Pmic_wdgGetEnableState_nullParam_handle(void)
{
    // Pass NULL handle into Pmic_wdgGetEnableState()
    bool isEnabled = PMIC_DISABLE;
    int32_t status = Pmic_wdgGetEnableState(NULL, &isEnabled);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_HANDLE);
}

void test_negative_Pmic_wdgGetEnableState_nullParam_isEnabled(void)
{
    // Pass null isEnabled into Pmic_wdgGetEnableState()
    int32_t status = Pmic_wdgGetEnableState(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_wdgSetCfg_nullParam_handle(void)
{
    // Pass NULL handle into Pmic_wdgSetCfg()
    Pmic_WdgCfg_t wdgCfg = {0U};
    int32_t status = Pmic_wdgSetCfg(NULL, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_HANDLE);
}

void test_negative_Pmic_wdgSetCfg_nullParam_wdgCfg(void)
{
    // Pass NULL wdgCfg into Pmic_wdgSetCfg()
    int32_t status = Pmic_wdgSetCfg(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_wdgSetCfg_outOfBounds_thresholdReset(void)
{
    // Pass out of bounds thresholdReset value into Pmic_wdgSetCfg()
    Pmic_WdgCfg_t wdgCfg = {
        .validParams = PMIC_CFG_WDG_THRESHOLD_RESET_VALID_SHIFT,
        .thresholdReset = PMIC_WDG_THRESHOLD_COUNT_MAX + 1U
    };
    int32_t status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_negative_Pmic_wdgSetCfg_outOfBounds_thresholdFail(void)
{
    // Pass out of bounds thresholdFail value into Pmic_wdgSetCfg()
    Pmic_WdgCfg_t wdgCfg = {
        .validParams = PMIC_CFG_WDG_THRESHOLD_FAIL_VALID_SHIFT,
        .thresholdFail = PMIC_WDG_THRESHOLD_COUNT_MAX + 1U
    };
    int32_t status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_negative_Pmic_wdgSetCfg_outOfBounds_win1Code(void)
{
    // Pass out of bounds win1Code value into Pmic_wdgSetCfg()
    Pmic_WdgCfg_t wdgCfg = {
        .validParams = PMIC_CFG_WDG_WIN1DURATION_VALID_SHIFT,
        .win1Code = PMIC_WDG_WIN_CODE_MAX + 1U
    };
    int32_t status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_negative_Pmic_wdgSetCfg_outOfBounds_win2Code(void)
{
    // Pass out of bounds win2Code value into Pmic_wdgSetCfg()
    Pmic_WdgCfg_t wdgCfg = {
        .validParams = PMIC_CFG_WDG_WIN2DURATION_VALID_SHIFT,
        .win2Code = PMIC_WDG_WIN_CODE_MAX + 1U
    };
    int32_t status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_negative_Pmic_wdgSetCfg_outOfBounds_qaFdbk(void)
{
    // Pass out of bounds qaFdbk value into Pmic_wdgSetCfg()
    Pmic_WdgCfg_t wdgCfg = {
        .validParams = PMIC_CFG_WDG_QA_FDBK_VALID_SHIFT,
        .qaFdbk = PMIC_WDG_QA_FEEDBACK_VALUE_MAX + 1U
    };
    int32_t status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_negative_Pmic_wdgSetCfg_outOfBounds_qaLfsr(void)
{
    // Pass out of bounds qaLfsr value into Pmic_wdgSetCfg()
    Pmic_WdgCfg_t wdgCfg = {
        .validParams = PMIC_CFG_WDG_QA_LFSR_VALID_SHIFT,
        .qaLfsr = PMIC_WDG_QA_LFSR_VALUE_MAX + 1U
    };
    int32_t status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_negative_Pmic_wdgSetCfg_outOfBounds_qaQuesSeed(void)
{
    // Pass out of bounds qaQuesSeed value into Pmic_wdgSetCfg()
    Pmic_WdgCfg_t wdgCfg = {
        .validParams = PMIC_CFG_WDG_QA_QUES_SEED_VALID_SHIFT,
        .qaQuesSeed = PMIC_WDG_QA_QUES_SEED_VALUE_MAX + 1U
    };
    int32_t status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_negative_Pmic_wdgGetCfg_nullParam_handle(void)
{
    // Pass NULL handle into Pmic_wdgGetCfg()
    Pmic_WdgCfg_t wdgCfg = {0U};
    int32_t status = Pmic_wdgGetCfg(NULL, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_HANDLE);
}

void test_negative_Pmic_wdgGetCfg_nullParam_wdgCfg(void)
{
    // Pass NULL wdgCfg into Pmic_wdgGetCfg()
    int32_t status = Pmic_wdgGetCfg(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_wdgSetPowerHold_nullParam_handle(void)
{
    // Pass NULL handle into Pmic_wdgSetPowerHold()
    int32_t status = Pmic_wdgSetPowerHold(NULL, PMIC_DISABLE);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_HANDLE);
}

void test_negative_Pmic_wdgGetPowerHold_nullParam_handle(void)
{
    // Pass NULL handle into Pmic_wdgGetPowerHold()
    bool isEnabled = PMIC_DISABLE;
    int32_t status = Pmic_wdgGetPowerHold(NULL, &isEnabled);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_HANDLE);
}

void test_negative_Pmic_wdgGetPowerHold_nullParam_isEnabled(void)
{
    // Pass NULL isEnabled into Pmic_wdgGetPowerHold()
    int32_t status = Pmic_wdgGetPowerHold(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_wdgSetReturnToLongWindow_nullParam_handle(void)
{
    // Pass NULL handle into Pmic_wdgSetReturnToLongWindow()
    int32_t status = Pmic_wdgSetReturnToLongWindow(NULL, PMIC_DISABLE);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_HANDLE);
}

void test_negative_Pmic_wdgGetReturnToLongWindow_nullParam_handle(void)
{
    // Pass NULL handle into Pmic_wdgGetReturnToLongWindow()
    bool isEnabled = PMIC_DISABLE;
    int32_t status = Pmic_wdgGetReturnToLongWindow(NULL, &isEnabled);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_HANDLE);
}

void test_negative_Pmic_wdgGetReturnToLongWindow_nullParam_isEnabled(void)
{
    // Pass NULL isEnabled into Pmic_wdgGetReturnToLongWindow()
    int32_t status = Pmic_wdgGetReturnToLongWindow(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_wdgGetErrorStatus_nullParam_handle(void)
{
    // Pass NULL handle into Pmic_wdgGetErrorStatus()
    Pmic_WdgError_t wdgErrStat = {0U};
    int32_t status = Pmic_wdgGetErrorStatus(NULL, &wdgErrStat);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_HANDLE);
}

void test_negative_Pmic_wdgGetErrorStatus_nullParam_errors(void)
{
    // Pass NULL errors into Pmic_wdgGetErrorStatus()
    int32_t status = Pmic_wdgGetErrorStatus(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_wdgClrErrStatus_nullParam_handle(void)
{
    // Pass NULL handle into Pmic_wdgClrErrStatus()
    Pmic_WdgError_t wdgErrStat = {0U};
    int32_t status = Pmic_wdgClrErrStatus(NULL, &wdgErrStat);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_HANDLE);
}

void test_negative_Pmic_wdgClrErrStatus_nullParam_errors(void)
{
    // Pass NULL errors into Pmic_wdgClrErrStatus()
    int32_t status = Pmic_wdgClrErrStatus(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_wdgClrErrStatusAll_nullParam_handle(void)
{
    // Pass NULL handle into Pmic_wdgClrErrStatusAll()
    int32_t status = Pmic_wdgClrErrStatusAll(NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_HANDLE);
}

void test_negative_Pmic_wdgGetFailCntStat_nullParam_handle(void)
{
    // Pass NULL handle into Pmic_wdgGetFailCntStat()
    Pmic_WdgFailCntStat_t failCnt = {0U};
    int32_t status = Pmic_wdgGetFailCntStat(NULL, &failCnt);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_HANDLE);
}

void test_negative_Pmic_wdgGetFailCntStat_nullParam_failCount(void)
{
    // Pass NULL failCount into Pmic_wdgGetFailCntStat()
    int32_t status = Pmic_wdgGetFailCntStat(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_wdgQaSequenceWriteAnswer_nullParam_handle(void)
{
    // Pass NULL handle into Pmic_wdgQaSequenceWriteAnswer()
    int32_t status = Pmic_wdgQaSequenceWriteAnswer(NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_HANDLE);
}

void test_negative_Pmic_wdgGetFdbkRegData_nullParam_handle(void)
{
    // Pass NULL handle into Pmic_wdgGetFdbkRegData()
    uint8_t regData = 0U;
    int32_t status = Pmic_wdgGetFdbkRegData(NULL, &regData);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_HANDLE);
}

void test_negative_Pmic_wdgGetFdbkRegData_nullParam_regData(void)
{
    // Pass NULL regData into Pmic_wdgGetFdbkRegData()
    int32_t status = Pmic_wdgGetFdbkRegData(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_wdgExtractFdbk_nullParam_wdgAnsInfo(void)
{
    // Pass NULL wdgAnsInfo into Pmic_wdgExtractFdbk
    const uint8_t regData = 0xAAU;
    int32_t status = Pmic_wdgExtractFdbk(regData, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_wdgGetAnsCntAndQuesRegData_nullParam_handle(void)
{
    // Pass NULL handle into Pmic_wdgGetAnsCntAndQuesRegData()
    uint8_t regData = 0U;
    int32_t status = Pmic_wdgGetAnsCntAndQuesRegData(NULL, &regData);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_HANDLE);
}

void test_negative_Pmic_wdgGetAnsCntAndQuesRegData_nullParam_regData(void)
{
    // Pass NULL regData into Pmic_wdgGetAnsCntAndQuesRegData()
    int32_t status = Pmic_wdgGetAnsCntAndQuesRegData(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_wdgExtractAnsCntAndQues_nullParam_handle(void)
{
    Pmic_WdgAnsInfo_t wdgAnsInfo = {0U};

    // Pass NULL handle into Pmic_wdgExtractAnsCntAndQues()
    const uint8_t regData = 0xAAU;
    int32_t status = Pmic_wdgExtractAnsCntAndQues(NULL, regData, &wdgAnsInfo);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_HANDLE);
}

void test_negative_Pmic_wdgExtractAnsCntAndQues_nullParam_wdgAnsInfo(void)
{
    // Pass NULL wdgAnsInfo into Pmic_wdgExtractAnsCntAndQues()
    const uint8_t regData = 0xAAU;
    int32_t status = Pmic_wdgExtractAnsCntAndQues(&pmicHandle, regData, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_wdgWriteAnswer_nullParam_handle(void)
{
    // Pass NULL handle into Pmic_wdgWriteAnswer()
    Pmic_WdgAnsInfo_t wdgAnsInfo = {0U};
    int32_t status = Pmic_wdgWriteAnswer(NULL, &wdgAnsInfo);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_HANDLE);
}

void test_negative_Pmic_wdgWriteAnswer_nullParam_wdgAnsInfo(void)
{
    // Pass NULL wdgAnsInfo into Pmic_wdgWriteAnswer()
    int32_t status = Pmic_wdgWriteAnswer(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_positive_wdgEnableDisable(void)
{
    bool isEnabled = PMIC_ENABLE;

    // Disable WDG
    int32_t status = Pmic_wdgDisable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get actual WDG enable state and compare expected vs. actual values
    status = Pmic_wdgGetEnableState(&pmicHandle, &isEnabled);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(isEnabled == PMIC_DISABLE);

    // Enable WDG
    status = Pmic_wdgEnable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get actual WDG enable state and compare expected vs. actual values
    status = Pmic_wdgGetEnableState(&pmicHandle, &isEnabled);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(isEnabled == PMIC_ENABLE);
}

void test_positive_wdgEnableDisablePowerHold(void)
{
    bool isEnabled = PMIC_ENABLE;

    // Disable power hold
    int32_t status = Pmic_wdgSetPowerHold(&pmicHandle, PMIC_DISABLE);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get actual power hold enable state and compare expected vs. actual values
    status = Pmic_wdgGetPowerHold(&pmicHandle, &isEnabled);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(isEnabled == PMIC_DISABLE);

    // Enable power hold
    status = Pmic_wdgSetPowerHold(&pmicHandle, PMIC_ENABLE);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get actual power hold enable state and compare expected vs. actual values
    status = Pmic_wdgGetPowerHold(&pmicHandle, &isEnabled);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(isEnabled == PMIC_ENABLE);
}

void test_positive_wdgEnableDisableReturnToLongWindow(void)
{
    bool isEnabled = PMIC_ENABLE;

    // Disable return to long window
    int32_t status = Pmic_wdgSetReturnToLongWindow(&pmicHandle, PMIC_DISABLE);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get actual return to long window state and compare expected vs. actual values
    status = Pmic_wdgGetReturnToLongWindow(&pmicHandle, &isEnabled);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(isEnabled == PMIC_DISABLE);

    // Enable return to long window
    status = Pmic_wdgSetReturnToLongWindow(&pmicHandle, PMIC_ENABLE);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get actual return to long window state and compare expected vs. actual values
    status = Pmic_wdgGetReturnToLongWindow(&pmicHandle, &isEnabled);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(isEnabled == PMIC_ENABLE);
}

void test_positive_wdgSetGetCfg_rstEn(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_WdgCfg_t expWdgCfg = {.validParams = PMIC_CFG_WDG_RST_EN_VALID_SHIFT};
    Pmic_WdgCfg_t actWdgCfg = {.validParams = PMIC_CFG_WDG_RST_EN_VALID_SHIFT};

    // Set watchdog reset enable to true
    expWdgCfg.rstEn = PMIC_ENABLE;
    status = Pmic_wdgSetCfg(&pmicHandle, &expWdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get actual WDG configuration and compare expected vs. actual values
    status = Pmic_wdgGetCfg(&pmicHandle, &actWdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(expWdgCfg.rstEn == actWdgCfg.rstEn);

    // Set watchdog reset enable to false
    expWdgCfg.rstEn = PMIC_DISABLE;
    status = Pmic_wdgSetCfg(&pmicHandle, &expWdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get actual WDG configuration and compare expected vs. actual values
    status = Pmic_wdgGetCfg(&pmicHandle, &actWdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(expWdgCfg.rstEn == actWdgCfg.rstEn);
}

void test_positive_wdgSetGetCfg_thresholdReset(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_WdgCfg_t expWdgCfg = {.validParams = PMIC_CFG_WDG_THRESHOLD_RESET_VALID_SHIFT};
    Pmic_WdgCfg_t actWdgCfg = {.validParams = PMIC_CFG_WDG_THRESHOLD_RESET_VALID_SHIFT};

    // For each thresholdReset value...
    for (uint8_t expVal = PMIC_WDG_THRESHOLD_COUNT_0; expVal <= PMIC_WDG_THRESHOLD_COUNT_MAX; expVal++)
    {
        // Set expected value
        expWdgCfg.thresholdReset = expVal;
        status = Pmic_wdgSetCfg(&pmicHandle, &expWdgCfg);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        // Get actual WDG configuration and compare expected vs. actual values
        status = Pmic_wdgGetCfg(&pmicHandle, &actWdgCfg);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        PLATFORM_ASSERT(expWdgCfg.thresholdReset == actWdgCfg.thresholdReset);
    }
}

void test_positive_wdgSetGetCfg_thresholdFail(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_WdgCfg_t expWdgCfg = {.validParams = PMIC_CFG_WDG_THRESHOLD_FAIL_VALID_SHIFT};
    Pmic_WdgCfg_t actWdgCfg = {.validParams = PMIC_CFG_WDG_THRESHOLD_FAIL_VALID_SHIFT};

    // For each thresholdFail value...
    for (uint8_t expVal = PMIC_WDG_THRESHOLD_COUNT_0; expVal <= PMIC_WDG_THRESHOLD_COUNT_MAX; expVal++)
    {
        // Set expected value
        expWdgCfg.thresholdFail = expVal;
        status = Pmic_wdgSetCfg(&pmicHandle, &expWdgCfg);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        // Get actual WDG configuration and compare expected vs. actual values
        status = Pmic_wdgGetCfg(&pmicHandle, &actWdgCfg);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        PLATFORM_ASSERT(expWdgCfg.thresholdFail == actWdgCfg.thresholdFail);
    }
}

void test_positive_wdgSetGetCfg_longWinCode(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_WdgCfg_t expWdgCfg = {.validParams = PMIC_CFG_WDG_LONGWINDURATION_VALID_SHIFT};
    Pmic_WdgCfg_t actWdgCfg = {.validParams = PMIC_CFG_WDG_LONGWINDURATION_VALID_SHIFT};

    // For each longWinCode value...
    for (uint16_t expVal = 0U; expVal <= WDG_TEST_LONG_WINDOW_CODE_MAX; expVal++)
    {
        // Set expected value
        expWdgCfg.longWinCode = (uint8_t)expVal;
        status = Pmic_wdgSetCfg(&pmicHandle, &expWdgCfg);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        // Get actual WDG configuration and compare expected vs. actual values
        status = Pmic_wdgGetCfg(&pmicHandle, &actWdgCfg);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        PLATFORM_ASSERT(expWdgCfg.longWinCode == actWdgCfg.longWinCode);
    }
}

void test_positive_wdgSetGetCfg_win1Code(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_WdgCfg_t expWdgCfg = {.validParams = PMIC_CFG_WDG_WIN1DURATION_VALID_SHIFT};
    Pmic_WdgCfg_t actWdgCfg = {.validParams = PMIC_CFG_WDG_WIN1DURATION_VALID_SHIFT};

    // For each win1Code value...
    for (uint16_t expVal = 0U; expVal <= PMIC_WDG_WIN_CODE_MAX; expVal++)
    {
        // Set expected value
        expWdgCfg.win1Code = (uint8_t)expVal;
        status = Pmic_wdgSetCfg(&pmicHandle, &expWdgCfg);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        // Get actual WDG configuration and compare expected vs. actual values
        status = Pmic_wdgGetCfg(&pmicHandle, &actWdgCfg);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        PLATFORM_ASSERT(expWdgCfg.win1Code == actWdgCfg.win1Code);
    }
}

void test_positive_wdgSetGetCfg_win2Code(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_WdgCfg_t expWdgCfg = {.validParams = PMIC_CFG_WDG_WIN2DURATION_VALID_SHIFT};
    Pmic_WdgCfg_t actWdgCfg = {.validParams = PMIC_CFG_WDG_WIN2DURATION_VALID_SHIFT};

    // For each win2Code value...
    for (uint16_t expVal = 0U; expVal <= PMIC_WDG_WIN_CODE_MAX; expVal++)
    {
        // Set expected value
        expWdgCfg.win2Code = (uint8_t)expVal;
        status = Pmic_wdgSetCfg(&pmicHandle, &expWdgCfg);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        // Get actual WDG configuration and compare expected vs. actual values
        status = Pmic_wdgGetCfg(&pmicHandle, &actWdgCfg);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        PLATFORM_ASSERT(expWdgCfg.win2Code == actWdgCfg.win2Code);
    }
}

void test_positive_wdgSetGetCfg_qaFdbk(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_WdgCfg_t expWdgCfg = {.validParams = PMIC_CFG_WDG_QA_FDBK_VALID_SHIFT};
    Pmic_WdgCfg_t actWdgCfg = {.validParams = PMIC_CFG_WDG_QA_FDBK_VALID_SHIFT};

    // For each qaFdbk value...
    for (uint16_t expVal = 0U; expVal <= PMIC_WDG_QA_FEEDBACK_VALUE_MAX; expVal++)
    {
        // Set expected value
        expWdgCfg.qaFdbk = (uint8_t)expVal;
        status = Pmic_wdgSetCfg(&pmicHandle, &expWdgCfg);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        // Get actual WDG configuration and compare expected vs. actual values
        status = Pmic_wdgGetCfg(&pmicHandle, &actWdgCfg);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        PLATFORM_ASSERT(expWdgCfg.qaFdbk == actWdgCfg.qaFdbk);
    }
}

void test_positive_wdgSetGetCfg_qaLfsr(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_WdgCfg_t expWdgCfg = {.validParams = PMIC_CFG_WDG_QA_LFSR_VALID_SHIFT};
    Pmic_WdgCfg_t actWdgCfg = {.validParams = PMIC_CFG_WDG_QA_LFSR_VALID_SHIFT};

    // For each qaLfsr value...
    for (uint16_t expVal = 0U; expVal <= PMIC_WDG_QA_LFSR_VALUE_MAX; expVal++)
    {
        // Set expected value
        expWdgCfg.qaLfsr = (uint8_t)expVal;
        status = Pmic_wdgSetCfg(&pmicHandle, &expWdgCfg);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        // Get actual WDG configuration and compare expected vs. actual values
        status = Pmic_wdgGetCfg(&pmicHandle, &actWdgCfg);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        PLATFORM_ASSERT(expWdgCfg.qaLfsr == actWdgCfg.qaLfsr);
    }
}

void test_positive_wdgSetGetCfg_qaQuesSeed(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_WdgCfg_t expWdgCfg = {.validParams = PMIC_CFG_WDG_QA_QUES_SEED_VALID_SHIFT};
    Pmic_WdgCfg_t actWdgCfg = {.validParams = PMIC_CFG_WDG_QA_QUES_SEED_VALID_SHIFT};

    // For each qaQuesSeed value...
    for (uint16_t expVal = 0U; expVal <= PMIC_WDG_QA_QUES_SEED_VALUE_MAX; expVal++)
    {
        // Set expected value
        expWdgCfg.qaQuesSeed = (uint8_t)expVal;
        status = Pmic_wdgSetCfg(&pmicHandle, &expWdgCfg);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        // Get actual WDG configuration and compare expected vs. actual values
        status = Pmic_wdgGetCfg(&pmicHandle, &actWdgCfg);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        PLATFORM_ASSERT(expWdgCfg.qaQuesSeed == actWdgCfg.qaQuesSeed);
    }
}

static void wdgTest_checkForWdgErrors(void)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;
    const uint8_t wdErrStatusRegAddr = 0x5EU, bufLen = 1U;

    status = platform_rxByte(&pmicHandle, PMIC_MAIN_INST, wdErrStatusRegAddr, &regData, bufLen);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(regData == 0U);
}

void test_positive_wdgQaSequence_detectNoErrors(void)
{
    uint8_t answerCnt = 0U;
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_WdgCfg_t wdgCfg = {
        .validParams = (PMIC_CFG_WDG_THRESHOLD_RESET_VALID_SHIFT |
                        PMIC_CFG_WDG_THRESHOLD_FAIL_VALID_SHIFT |
                        PMIC_CFG_WDG_LONGWINDURATION_VALID_SHIFT |
                        PMIC_CFG_WDG_WIN1DURATION_VALID_SHIFT |
                        PMIC_CFG_WDG_WIN2DURATION_VALID_SHIFT |
                        PMIC_CFG_WDG_QA_FDBK_VALID_SHIFT |
                        PMIC_CFG_WDG_QA_LFSR_VALID_SHIFT |
                        PMIC_CFG_WDG_QA_QUES_SEED_VALID_SHIFT),
        .thresholdReset = 0U,
        .thresholdFail = 0U,
        .longWinCode = 0xFFU, // ~13 minutes
        .win1Code = PMIC_WDG_WIN_CODE_MAX, // 70.4 ms
        .win2Code = PMIC_WDG_WIN_CODE_MAX, // 70.4 ms
        .qaFdbk = 0U,
        .qaLfsr = 1U,
        .qaQuesSeed = 2U
    };

    // Enable Watchdog and clear all watchdog statuses
    status = Pmic_wdgEnable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    status = Pmic_wdgClrErrStatusAll(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Configure watchdog
    status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Disable power hold and disable return to long window to begin Q&A sequences
    status = Pmic_wdgSetPowerHold(&pmicHandle, PMIC_DISABLE);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    status = Pmic_wdgSetReturnToLongWindow(&pmicHandle, PMIC_DISABLE);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Exit Long Window by sending all 4 answer bytes
    for (answerCnt = 4U; answerCnt != 0U; answerCnt--)
    {
        status = Pmic_wdgQaSequenceWriteAnswer(&pmicHandle);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        wdgTest_checkForWdgErrors();
    }

    // Undergo Q&A sequences
    for (uint16_t numSeqeunces = 20U; numSeqeunces != 0U; numSeqeunces--)
    {
        // Upon last iteration, indicate that we want to return to Long Window
        if (numSeqeunces == 1U)
        {
            status = Pmic_wdgSetReturnToLongWindow(&pmicHandle, PMIC_ENABLE);
            PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        }

        // Enter Window-1; calculate and send answer bytes Answer-3, Answer-2,
        // and Answer-1; check for any WDG errors
        for (answerCnt = 3U; answerCnt >= 1U; answerCnt--)
        {
            status = Pmic_wdgQaSequenceWriteAnswer(&pmicHandle);
            PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
            wdgTest_checkForWdgErrors();
        }

        // Wait until Window-1 time elapses
        platform_timerWaitMs(71U);

        // Enter Window-2; calculate and send last answer byte; check for any WDG errors
        status = Pmic_wdgQaSequenceWriteAnswer(&pmicHandle);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        wdgTest_checkForWdgErrors();

        // End of Q&A sequence; next question will be
        // generated and the next sequence will begin
    }

    // WDG has returned to Long Window; set WD_PWRHOLD so that WDG remains in Long Window
    status = Pmic_wdgSetPowerHold(&pmicHandle, PMIC_ENABLE);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_positive_wdgQaSequence_detectTimeout(void)
{
    uint8_t answerCnt = 0U;
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_WdgCfg_t wdgCfg = {
        .validParams = (PMIC_CFG_WDG_THRESHOLD_RESET_VALID_SHIFT |
                        PMIC_CFG_WDG_THRESHOLD_FAIL_VALID_SHIFT |
                        PMIC_CFG_WDG_LONGWINDURATION_VALID_SHIFT |
                        PMIC_CFG_WDG_WIN1DURATION_VALID_SHIFT |
                        PMIC_CFG_WDG_WIN2DURATION_VALID_SHIFT |
                        PMIC_CFG_WDG_QA_FDBK_VALID_SHIFT |
                        PMIC_CFG_WDG_QA_LFSR_VALID_SHIFT |
                        PMIC_CFG_WDG_QA_QUES_SEED_VALID_SHIFT),
        .thresholdReset = PMIC_WDG_THRESHOLD_COUNT_MAX,
        .thresholdFail = PMIC_WDG_THRESHOLD_COUNT_MAX,
        .longWinCode = 0xFFU, // ~13 minutes
        .win1Code = PMIC_WDG_WIN_CODE_MAX, // 70.4 ms
        .win2Code = PMIC_WDG_WIN_CODE_MAX, // 70.4 ms
        .qaFdbk = 0U,
        .qaLfsr = 1U,
        .qaQuesSeed = 2U
    };
    Pmic_WdgFailCntStat_t wdgFailCntStat = {
        .validParams = (PMIC_CFG_WD_BAD_EVENT_STAT_VALID_SHIFT | PMIC_CFG_WD_FAIL_CNT_VAL_VALID_SHIFT),
        .badEvent = (bool)false,
        .wdFailCnt = 0U
    };
    Pmic_WdgError_t wdgErrStat = {
        .validParams = PMIC_CFG_WD_TIMEOUT_ERR_VALID_SHIFT,
        .timeout = (bool)false
    };

    // Enable Watchdog and clear all watchdog statuses
    status = Pmic_wdgEnable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    status = Pmic_wdgClrErrStatusAll(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Configure watchdog
    status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Disable power hold and disable return to long window to begin Q&A sequences
    status = Pmic_wdgSetPowerHold(&pmicHandle, PMIC_DISABLE);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    status = Pmic_wdgSetReturnToLongWindow(&pmicHandle, PMIC_DISABLE);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Exit Long Window by sending all 4 answer bytes
    for (answerCnt = 4U; answerCnt != 0U; answerCnt--)
    {
        status = Pmic_wdgQaSequenceWriteAnswer(&pmicHandle);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        wdgTest_checkForWdgErrors();
    }

    // Enter Window-1; wait duration of Window-1 to enter Window-2
    platform_timerWaitMs(71U);

    // Validate bad event (no answers sent in Window-1)
    status = Pmic_wdgGetFailCntStat(&pmicHandle, &wdgFailCntStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(wdgFailCntStat.badEvent == (bool)true);

    // Enter Window-2; wait duration of Window-2 to end the sequence
    platform_timerWaitMs(71U);

    // End of sequence and start of new sequence; validate fail count
    status = Pmic_wdgGetFailCntStat(&pmicHandle, &wdgFailCntStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(wdgFailCntStat.wdFailCnt != 0U);

    // Enable return to long window and wait until PMIC returns to long window
    status = Pmic_wdgSetReturnToLongWindow(&pmicHandle, PMIC_ENABLE);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    platform_timerWaitMs(142U);

    // PMIC has entered long window; enable power hold
    status = Pmic_wdgSetPowerHold(&pmicHandle, PMIC_ENABLE);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Validate and clear WD_TIMEOUT flag
    status = Pmic_wdgGetErrorStatus(&pmicHandle, &wdgErrStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(wdgErrStat.timeout == (bool)true);
    status = Pmic_wdgClrErrStatus(&pmicHandle, &wdgErrStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_positive_wdgQaSequence_detectLongWindowTimeout(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_WdgCfg_t wdgCfg = {
        .validParams = (PMIC_CFG_WDG_THRESHOLD_RESET_VALID_SHIFT |
                        PMIC_CFG_WDG_THRESHOLD_FAIL_VALID_SHIFT |
                        PMIC_CFG_WDG_LONGWINDURATION_VALID_SHIFT |
                        PMIC_CFG_WDG_WIN1DURATION_VALID_SHIFT |
                        PMIC_CFG_WDG_WIN2DURATION_VALID_SHIFT |
                        PMIC_CFG_WDG_QA_FDBK_VALID_SHIFT |
                        PMIC_CFG_WDG_QA_LFSR_VALID_SHIFT |
                        PMIC_CFG_WDG_QA_QUES_SEED_VALID_SHIFT),
        .thresholdReset = PMIC_WDG_THRESHOLD_COUNT_MAX,
        .thresholdFail = PMIC_WDG_THRESHOLD_COUNT_MAX,
        .longWinCode = 2U, // 252 ms
        .win1Code = PMIC_WDG_WIN_CODE_MAX, // 70.4 ms
        .win2Code = PMIC_WDG_WIN_CODE_MAX, // 70.4 ms
        .qaFdbk = 0U,
        .qaLfsr = 1U,
        .qaQuesSeed = 2U
    };
    Pmic_WdgError_t wdgErrStat = {
        .validParams = PMIC_CFG_WD_LONGWIN_TIMEOUT_ERR_VALID_SHIFT,
        .longWindowTimeout = (bool)false
    };

    // Enable Watchdog and clear all watchdog statuses
    status = Pmic_wdgEnable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    status = Pmic_wdgClrErrStatusAll(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Configure watchdog
    status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Disable power hold and disable return to long window to begin Q&A sequences
    status = Pmic_wdgSetPowerHold(&pmicHandle, PMIC_DISABLE);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    status = Pmic_wdgSetReturnToLongWindow(&pmicHandle, PMIC_DISABLE);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Wait entire long window duration to incur long window timeout
    platform_timerWaitMs(253U);

    // PMIC has undergone warm reset; unlock PMIC registers and clear all PMIC IRQs
    status = wdgTest_unlockPmicRegs();
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    status = wdgTest_clrAllPmicIrq();
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Validate and clear WD_LONGWIN_TIMEOUT_INT flag
    status = Pmic_wdgGetErrorStatus(&pmicHandle, &wdgErrStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(wdgErrStat.longWindowTimeout == (bool)true);
    status = Pmic_wdgClrErrStatus(&pmicHandle, &wdgErrStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Enable power hold and enable return to long window
    status = Pmic_wdgSetPowerHold(&pmicHandle, PMIC_ENABLE);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    status = Pmic_wdgSetReturnToLongWindow(&pmicHandle, PMIC_ENABLE);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_positive_wdgQaSequence_detectAnswerEarlyError(void)
{
    uint8_t answerCnt = 0U;
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_WdgCfg_t wdgCfg = {
        .validParams = (PMIC_CFG_WDG_THRESHOLD_RESET_VALID_SHIFT |
                        PMIC_CFG_WDG_THRESHOLD_FAIL_VALID_SHIFT |
                        PMIC_CFG_WDG_LONGWINDURATION_VALID_SHIFT |
                        PMIC_CFG_WDG_WIN1DURATION_VALID_SHIFT |
                        PMIC_CFG_WDG_WIN2DURATION_VALID_SHIFT |
                        PMIC_CFG_WDG_QA_FDBK_VALID_SHIFT |
                        PMIC_CFG_WDG_QA_LFSR_VALID_SHIFT |
                        PMIC_CFG_WDG_QA_QUES_SEED_VALID_SHIFT),
        .thresholdReset = PMIC_WDG_THRESHOLD_COUNT_MAX,
        .thresholdFail = PMIC_WDG_THRESHOLD_COUNT_MAX,
        .longWinCode = 0xFFU, // ~13 minutes
        .win1Code = PMIC_WDG_WIN_CODE_MAX, // 70.4 ms
        .win2Code = PMIC_WDG_WIN_CODE_MAX, // 70.4 ms
        .qaFdbk = 0U,
        .qaLfsr = 1U,
        .qaQuesSeed = 2U
    };
    Pmic_WdgError_t wdgErrStat = {
        .validParams = PMIC_CFG_WD_ANSW_EARLY_ERR_VALID_SHIFT,
        .answerEarlyError = (bool)false
    };

    // Enable Watchdog and clear all watchdog statuses
    status = Pmic_wdgEnable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    status = Pmic_wdgClrErrStatusAll(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Configure watchdog
    status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Disable power hold and disable return to long window to begin Q&A sequences
    status = Pmic_wdgSetPowerHold(&pmicHandle, PMIC_DISABLE);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    status = Pmic_wdgSetReturnToLongWindow(&pmicHandle, PMIC_DISABLE);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Exit Long Window by sending all 4 answer bytes
    for (answerCnt = 4U; answerCnt != 0U; answerCnt--)
    {
        status = Pmic_wdgQaSequenceWriteAnswer(&pmicHandle);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        wdgTest_checkForWdgErrors();
    }

    // Enter Window-1; enable return to long window
    status = Pmic_wdgSetReturnToLongWindow(&pmicHandle, PMIC_ENABLE);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Send all four answer bytes to incur WD_ANSW_EARLY error
    for (answerCnt = 4U; answerCnt != 0U; answerCnt--)
    {
        status = Pmic_wdgQaSequenceWriteAnswer(&pmicHandle);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    }

    // Wait until Window-1 duration is elapsed to enter Window-2
    platform_timerWaitMs(71U);

    // Enter Window-2; wait until Window-2 duration is elapsed to end sequence
    platform_timerWaitMs(71U);

    // PMIC has returned to long window; enable power hold
    status = Pmic_wdgSetPowerHold(&pmicHandle, PMIC_ENABLE);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Validate and clear WD_ANSW_EARLY flag
    status = Pmic_wdgGetErrorStatus(&pmicHandle, &wdgErrStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(wdgErrStat.answerEarlyError == (bool)true);
    status = Pmic_wdgClrErrStatus(&pmicHandle, &wdgErrStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_positive_wdgQaSequence_detectSequenceError(void)
{
    uint8_t answerCnt = 0U;
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_WdgCfg_t wdgCfg = {
        .validParams = (PMIC_CFG_WDG_THRESHOLD_RESET_VALID_SHIFT |
                        PMIC_CFG_WDG_THRESHOLD_FAIL_VALID_SHIFT |
                        PMIC_CFG_WDG_LONGWINDURATION_VALID_SHIFT |
                        PMIC_CFG_WDG_WIN1DURATION_VALID_SHIFT |
                        PMIC_CFG_WDG_WIN2DURATION_VALID_SHIFT |
                        PMIC_CFG_WDG_QA_FDBK_VALID_SHIFT |
                        PMIC_CFG_WDG_QA_LFSR_VALID_SHIFT |
                        PMIC_CFG_WDG_QA_QUES_SEED_VALID_SHIFT),
        .thresholdReset = PMIC_WDG_THRESHOLD_COUNT_MAX,
        .thresholdFail = PMIC_WDG_THRESHOLD_COUNT_MAX,
        .longWinCode = 0xFFU, // ~13 minutes
        .win1Code = PMIC_WDG_WIN_CODE_MAX, // 70.4 ms
        .win2Code = PMIC_WDG_WIN_CODE_MAX, // 70.4 ms
        .qaFdbk = 0U,
        .qaLfsr = 1U,
        .qaQuesSeed = 2U
    };
    Pmic_WdgError_t wdgErrStat = {
        .validParams = PMIC_CFG_WD_SEQ_ERR_ERR_VALID_SHIFT,
        .sequenceError = (bool)false
    };

    // Enable Watchdog and clear all watchdog statuses
    status = Pmic_wdgEnable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    status = Pmic_wdgClrErrStatusAll(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Configure watchdog
    status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Disable power hold and disable return to long window to begin Q&A sequences
    status = Pmic_wdgSetPowerHold(&pmicHandle, PMIC_DISABLE);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    status = Pmic_wdgSetReturnToLongWindow(&pmicHandle, PMIC_DISABLE);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Exit Long Window by sending all 4 answer bytes
    for (answerCnt = 4U; answerCnt != 0U; answerCnt--)
    {
        status = Pmic_wdgQaSequenceWriteAnswer(&pmicHandle);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        wdgTest_checkForWdgErrors();
    }

    // Enter Window-1; enable return to long window
    status = Pmic_wdgSetReturnToLongWindow(&pmicHandle, PMIC_ENABLE);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Send only answer bytes Answer-3 and Answer-2 to incur WD_SEQ_ERR
    status = Pmic_wdgQaSequenceWriteAnswer(&pmicHandle); // Answer-3
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    status = Pmic_wdgQaSequenceWriteAnswer(&pmicHandle); // Answer-2
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Wait until Window-1 duration is elapsed to enter Window-2
    platform_timerWaitMs(71U);

    // Enter Window-2; send answer bytes Answer-1 and Answer-0
    status = Pmic_wdgQaSequenceWriteAnswer(&pmicHandle); // Answer-1
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    status = Pmic_wdgQaSequenceWriteAnswer(&pmicHandle); // Answer-0
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // PMIC has returned to long window after end of sequence; enable power hold
    status = Pmic_wdgSetPowerHold(&pmicHandle, PMIC_ENABLE);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Validate and clear WD_SEQ_ERR flag
    status = Pmic_wdgGetErrorStatus(&pmicHandle, &wdgErrStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(wdgErrStat.sequenceError == (bool)true);
    status = Pmic_wdgClrErrStatus(&pmicHandle, &wdgErrStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_positive_wdgQaSequence_detectAnswerError(void)
{
    const uint8_t bufLen = 1U;
    const uint16_t wdAnswerReg = 0x0EU;
    uint8_t answerCnt = 0U, regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_WdgCfg_t wdgCfg = {
        .validParams = (PMIC_CFG_WDG_THRESHOLD_RESET_VALID_SHIFT |
                        PMIC_CFG_WDG_THRESHOLD_FAIL_VALID_SHIFT |
                        PMIC_CFG_WDG_LONGWINDURATION_VALID_SHIFT |
                        PMIC_CFG_WDG_WIN1DURATION_VALID_SHIFT |
                        PMIC_CFG_WDG_WIN2DURATION_VALID_SHIFT |
                        PMIC_CFG_WDG_QA_FDBK_VALID_SHIFT |
                        PMIC_CFG_WDG_QA_LFSR_VALID_SHIFT |
                        PMIC_CFG_WDG_QA_QUES_SEED_VALID_SHIFT),
        .thresholdReset = PMIC_WDG_THRESHOLD_COUNT_MAX,
        .thresholdFail = PMIC_WDG_THRESHOLD_COUNT_MAX,
        .longWinCode = 0xFFU, // ~13 minutes
        .win1Code = PMIC_WDG_WIN_CODE_MAX, // 70.4 ms
        .win2Code = PMIC_WDG_WIN_CODE_MAX, // 70.4 ms
        .qaFdbk = 0U,
        .qaLfsr = 1U,
        .qaQuesSeed = 2U
    };
    Pmic_WdgError_t wdgErrStat = {
        .validParams = PMIC_CFG_WD_ANSW_ERR_ERR_VALID_SHIFT,
        .answerError = (bool)false
    };

    // Enable Watchdog and clear all watchdog statuses
    status = Pmic_wdgEnable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    status = Pmic_wdgClrErrStatusAll(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Configure watchdog
    status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Disable power hold and disable return to long window to begin Q&A sequences
    status = Pmic_wdgSetPowerHold(&pmicHandle, PMIC_DISABLE);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    status = Pmic_wdgSetReturnToLongWindow(&pmicHandle, PMIC_DISABLE);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Exit Long Window by sending all 4 answer bytes
    for (answerCnt = 4U; answerCnt != 0U; answerCnt--)
    {
        status = Pmic_wdgQaSequenceWriteAnswer(&pmicHandle);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        wdgTest_checkForWdgErrors();
    }

    // Enter Window-1; enable return to long window
    status = Pmic_wdgSetReturnToLongWindow(&pmicHandle, PMIC_ENABLE);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Send incorrect Answer-3 but correct Answer-2 and Answer-1 to incur WD_ANSW_ERR
    platform_txByte(&pmicHandle, PMIC_MAIN_INST, wdAnswerReg, &regData, bufLen); // Answer-3
    for (answerCnt = 2U; answerCnt >= 1U; answerCnt--)
    {
        status = Pmic_wdgQaSequenceWriteAnswer(&pmicHandle);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    }

    // Wait until Window-1 duration is elapsed
    platform_timerWaitMs(71U);

    // Enter Window-2; send last answer byte
    status = Pmic_wdgQaSequenceWriteAnswer(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // PMIC has returned to long window after end of sequence; enable power hold
    status = Pmic_wdgSetPowerHold(&pmicHandle, PMIC_ENABLE);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Validate and clear WD_ANSW_ERR flag
    status = Pmic_wdgGetErrorStatus(&pmicHandle, &wdgErrStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(wdgErrStat.answerError == (bool)true);
    status = Pmic_wdgClrErrStatus(&pmicHandle, &wdgErrStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_positive_wdgQaSequence_detectFailInt(void)
{
    uint8_t answerCnt = 0U, expFailCnt = 0U;
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_WdgCfg_t wdgCfg = {
        .validParams = (PMIC_CFG_WDG_THRESHOLD_RESET_VALID_SHIFT |
                        PMIC_CFG_WDG_THRESHOLD_FAIL_VALID_SHIFT |
                        PMIC_CFG_WDG_LONGWINDURATION_VALID_SHIFT |
                        PMIC_CFG_WDG_WIN1DURATION_VALID_SHIFT |
                        PMIC_CFG_WDG_WIN2DURATION_VALID_SHIFT |
                        PMIC_CFG_WDG_QA_FDBK_VALID_SHIFT |
                        PMIC_CFG_WDG_QA_LFSR_VALID_SHIFT |
                        PMIC_CFG_WDG_QA_QUES_SEED_VALID_SHIFT),
        .thresholdReset = PMIC_WDG_THRESHOLD_COUNT_MAX,
        .thresholdFail = 3U,
        .longWinCode = 0xFFU, // ~13 minutes
        .win1Code = PMIC_WDG_WIN_CODE_MAX, // 70.4 ms
        .win2Code = PMIC_WDG_WIN_CODE_MAX, // 70.4 ms
        .qaFdbk = 0U,
        .qaLfsr = 1U,
        .qaQuesSeed = 2U
    };
    Pmic_WdgFailCntStat_t wdgFailCntStat = {
        .validParams = (PMIC_CFG_WD_FAIL_CNT_VAL_VALID_SHIFT),
        .wdFailCnt = 0U
    };
    Pmic_WdgError_t wdgErrStat = {
        .validParams = PMIC_CFG_WD_FAIL_INT_ERR_VALID_SHIFT,
        .failInt = (bool)false
    };

    // Enable Watchdog and clear all watchdog statuses
    status = Pmic_wdgEnable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    status = Pmic_wdgClrErrStatusAll(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Configure watchdog
    status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Disable power hold and disable return to long window to begin Q&A sequences
    status = Pmic_wdgSetPowerHold(&pmicHandle, PMIC_DISABLE);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    status = Pmic_wdgSetReturnToLongWindow(&pmicHandle, PMIC_DISABLE);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Exit Long Window by sending all 4 answer bytes
    for (answerCnt = 4U; answerCnt != 0U; answerCnt--)
    {
        status = Pmic_wdgQaSequenceWriteAnswer(&pmicHandle);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        wdgTest_checkForWdgErrors();
    }

    // Incur WD_FAIL_INT by causing WD_FAIL_CNT to be greater than WD_FAIL_TH
    for (uint8_t numSequences = (wdgCfg.thresholdFail + 2U); numSequences != 0U; numSequences--)
    {
        expFailCnt++;

        // Enable return to long window upon last iteration
        if (numSequences == 1U)
        {
            status = Pmic_wdgSetReturnToLongWindow(&pmicHandle, PMIC_ENABLE);
            PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        }

        // Enter Window-1; wait duration of Window-1 to enter Window-2
        platform_timerWaitMs(71U);

        // Enter Window-2; wait duration of Window-2 to end the sequence
        platform_timerWaitMs(71U);

        // End of sequence and start of new sequence; validate fail count.
        // NOTE: the fail counter resets to zero upon entering long window
        if (numSequences != 1U)
        {
            status = Pmic_wdgGetFailCntStat(&pmicHandle, &wdgFailCntStat);
            PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
            PLATFORM_ASSERT(wdgFailCntStat.wdFailCnt == expFailCnt);
        }
        else
        {
            status = Pmic_wdgGetFailCntStat(&pmicHandle, &wdgFailCntStat);
            PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
            PLATFORM_ASSERT(wdgFailCntStat.wdFailCnt == 0U);
        }
    }

    // PMIC has entered long window; enable power hold
    status = Pmic_wdgSetPowerHold(&pmicHandle, PMIC_ENABLE);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Validate and clear WD_FAIL_INT flag
    status = Pmic_wdgGetErrorStatus(&pmicHandle, &wdgErrStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(wdgErrStat.failInt == (bool)true);
    status = Pmic_wdgClrErrStatus(&pmicHandle, &wdgErrStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_positive_wdgQaSequence_detectResetInt(void)
{
    uint8_t answerCnt = 0U, expFailCnt = 0U;
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_WdgCfg_t wdgCfg = {
        .validParams = (PMIC_CFG_WDG_RST_EN_VALID_SHIFT |
                        PMIC_CFG_WDG_THRESHOLD_RESET_VALID_SHIFT |
                        PMIC_CFG_WDG_THRESHOLD_FAIL_VALID_SHIFT |
                        PMIC_CFG_WDG_LONGWINDURATION_VALID_SHIFT |
                        PMIC_CFG_WDG_WIN1DURATION_VALID_SHIFT |
                        PMIC_CFG_WDG_WIN2DURATION_VALID_SHIFT |
                        PMIC_CFG_WDG_QA_FDBK_VALID_SHIFT |
                        PMIC_CFG_WDG_QA_LFSR_VALID_SHIFT |
                        PMIC_CFG_WDG_QA_QUES_SEED_VALID_SHIFT),
        .rstEn = PMIC_ENABLE,
        .thresholdReset = 3U,
        .thresholdFail = 3U,
        .longWinCode = 0xFFU, // ~13 minutes
        .win1Code = PMIC_WDG_WIN_CODE_MAX, // 70.4 ms
        .win2Code = PMIC_WDG_WIN_CODE_MAX, // 70.4 ms
        .qaFdbk = 0U,
        .qaLfsr = 1U,
        .qaQuesSeed = 2U
    };
    Pmic_WdgFailCntStat_t wdgFailCntStat = {
        .validParams = (PMIC_CFG_WD_FAIL_CNT_VAL_VALID_SHIFT),
        .wdFailCnt = 0U
    };
    Pmic_WdgError_t wdgErrStat = {
        .validParams = PMIC_CFG_WD_RST_INT_ERR_VALID_SHIFT,
        .resetInt = (bool)false
    };

    // Enable Watchdog and clear all watchdog statuses
    status = Pmic_wdgEnable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    status = Pmic_wdgClrErrStatusAll(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Configure watchdog
    status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Disable power hold and disable return to long window to begin Q&A sequences
    status = Pmic_wdgSetPowerHold(&pmicHandle, PMIC_DISABLE);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    status = Pmic_wdgSetReturnToLongWindow(&pmicHandle, PMIC_DISABLE);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Exit Long Window by sending all 4 answer bytes
    for (answerCnt = 4U; answerCnt != 0U; answerCnt--)
    {
        status = Pmic_wdgQaSequenceWriteAnswer(&pmicHandle);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        wdgTest_checkForWdgErrors();
    }

    // Incur WD_RST_INT by causing WD_FAIL_CNT to be greater than WD_FAIL_TH + WD_RST_INT
    const uint8_t threshold = wdgCfg.thresholdFail + wdgCfg.thresholdReset + 1U;
    for (uint8_t numSequences = threshold; numSequences != 0U; numSequences--)
    {
        expFailCnt++;

        // Enter Window-1; wait duration of Window-1 to enter Window-2
        platform_timerWaitMs(71U);

        // Enter Window-2; wait duration of Window-2 to end the sequence
        platform_timerWaitMs(71U);

        // End of sequence and start of new sequence; validate fail count.
        // NOTE: PMIC will warm reset at end of last sequence
        if (numSequences != 1U)
        {
            status = Pmic_wdgGetFailCntStat(&pmicHandle, &wdgFailCntStat);
            PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
            PLATFORM_ASSERT(wdgFailCntStat.wdFailCnt == expFailCnt);
        }
    }

    // PMIC has undergone warm reset; unlock PMIC registers and clear all PMIC IRQs
    status = wdgTest_unlockPmicRegs();
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    status = wdgTest_clrAllPmicIrq();
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // PMIC is in long window; enable power hold and enable return to long window
    status = Pmic_wdgSetPowerHold(&pmicHandle, PMIC_ENABLE);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    status = Pmic_wdgSetReturnToLongWindow(&pmicHandle, PMIC_ENABLE);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Validate and clear WD_RST_EN flag
    status = Pmic_wdgGetErrorStatus(&pmicHandle, &wdgErrStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(wdgErrStat.resetInt == (bool)true);
    status = Pmic_wdgClrErrStatus(&pmicHandle, &wdgErrStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
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
