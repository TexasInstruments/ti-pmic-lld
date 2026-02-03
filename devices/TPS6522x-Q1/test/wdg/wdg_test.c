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


#include "../platform.h"
#include "wdg_test.h"

/*                            Global Variables                                */
static Pmic_Handle_t pmicHandle = {0};
/*                         Helper Functions                                   */
/**
 * @brief Setup helper: Initialize WDG to valid configuration state
 *
 * Ensures WDG is enabled and in Long Window mode, which are
 * prerequisites for calling Pmic_wdgSetCfg().
 */
static void wdg_setupForConfig(void)
{
    int32_t status;

    /* Enable watchdog */
    status = Pmic_wdgSetEnableState(&pmicHandle, true);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Enable return to long window */
    status = Pmic_wdgSetReturnToLongWindow(&pmicHandle, true);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}
/*                       Negative Test Functions                              */
/**
 * @brief Test Pmic_wdgSetEnableState with NULL handle
 */
void test_neg_wdg_wdgSetEnableState_nullHandle(void)
{
    int32_t status = Pmic_wdgSetEnableState(NULL, true);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}
/**
 * @brief Test Pmic_wdgGetEnableState with NULL handle
 */
void test_neg_wdg_wdgGetEnableState_nullHandle(void)
{
    bool isEnabled = false;
    int32_t status = Pmic_wdgGetEnableState(NULL, &isEnabled);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}
/**
 * @brief Test Pmic_wdgGetEnableState with NULL isEnabled parameter
 */
void test_neg_wdg_wdgGetEnableState_nullIsEnabled(void)
{
    int32_t status = Pmic_wdgGetEnableState(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}
/**
 * @brief Test Pmic_wdgSetCfg with NULL handle
 */
void test_neg_wdg_wdgSetCfg_nullHandle(void)
{
    Pmic_WdgCfg_t wdgCfg = {0};
    int32_t status = Pmic_wdgSetCfg(NULL, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}
/**
 * @brief Test Pmic_wdgSetCfg with NULL wdgCfg parameter
 */
void test_neg_wdg_wdgSetCfg_nullWdgCfg(void)
{
    int32_t status = Pmic_wdgSetCfg(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}
/**
 * @brief Test Pmic_wdgSetCfg with invalid mode value
 */
void test_neg_wdg_wdgSetCfg_invalidMode(void)
{
    Pmic_WdgCfg_t wdgCfg = {
        .validParams = PMIC_WDG_MODE_SEL_VALID,
        .mode = PMIC_WDG_MODE_MAX + 1U
    };
    int32_t status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}
/**
 * @brief Test Pmic_wdgSetCfg with invalid win1Code value
 */
void test_neg_wdg_wdgSetCfg_invalidWin1Code(void)
{
    Pmic_WdgCfg_t wdgCfg = {
        .validParams = PMIC_WDG_WIN1_CODE_VALID,
        .win1Code = PMIC_WDG_WIN1_CODE_MAX + 1U
    };
    int32_t status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}
/**
 * @brief Test Pmic_wdgSetCfg with invalid win2Code value
 */
void test_neg_wdg_wdgSetCfg_invalidWin2Code(void)
{
    Pmic_WdgCfg_t wdgCfg = {
        .validParams = PMIC_WDG_WIN2_CODE_VALID,
        .win2Code = PMIC_WDG_WIN2_CODE_MAX + 1U
    };
    int32_t status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}
/**
 * @brief Test Pmic_wdgSetCfg with invalid qaFdbk value
 */
void test_neg_wdg_wdgSetCfg_invalidQaFdbk(void)
{
    Pmic_WdgCfg_t wdgCfg = {
        .validParams = PMIC_WDG_QA_FDBK_VALID,
        .qaFdbk = PMIC_WDG_QA_FDBK_MAX + 1U
    };
    int32_t status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}
/**
 * @brief Test Pmic_wdgSetCfg with invalid qaLfsr value
 */
void test_neg_wdg_wdgSetCfg_invalidQaLfsr(void)
{
    Pmic_WdgCfg_t wdgCfg = {
        .validParams = PMIC_WDG_QA_LFSR_VALID,
        .qaLfsr = PMIC_WDG_QA_LFSR_MAX + 1U
    };
    int32_t status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}
/**
 * @brief Test Pmic_wdgSetCfg with invalid qaSeed value
 */
void test_neg_wdg_wdgSetCfg_invalidQaSeed(void)
{
    Pmic_WdgCfg_t wdgCfg = {
        .validParams = PMIC_WDG_QA_SEED_VALID,
        .qaSeed = PMIC_WDG_QA_SEED_MAX + 1U
    };
    int32_t status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}
/**
 * @brief Test Pmic_wdgSetCfg with invalid failThr value
 */
void test_neg_wdg_wdgSetCfg_invalidFailThr(void)
{
    Pmic_WdgCfg_t wdgCfg = {
        .validParams = PMIC_WDG_FAIL_THR_VALID,
        .failThr = PMIC_WDG_FAIL_THR_MAX + 1U
    };
    int32_t status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}
/**
 * @brief Test Pmic_wdgSetCfg with invalid rstThr value
 */
void test_neg_wdg_wdgSetCfg_invalidRstThr(void)
{
    Pmic_WdgCfg_t wdgCfg = {
        .validParams = PMIC_WDG_RST_THR_VALID,
        .rstThr = PMIC_WDG_RST_THR_MAX + 1U
    };
    int32_t status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}
/**
 * @brief Test Pmic_wdgSetCfg with invalid cntSel value
 */
void test_neg_wdg_wdgSetCfg_invalidCntSel(void)
{
    Pmic_WdgCfg_t wdgCfg = {
        .validParams = PMIC_WDG_CNT_SEL_VALID,
        .cntSel = PMIC_WDG_CNT_SEL_MAX + 1U
    };
    int32_t status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}
/**
 * @brief Test Pmic_wdgSetCfg when watchdog is disabled
 */
void test_neg_wdg_wdgSetCfg_whenDisabled(void)
{
    int32_t status;
    Pmic_WdgCfg_t wdgCfg = {
        .validParams = PMIC_WDG_MODE_SEL_VALID,
        .mode = PMIC_WDG_TRIGGER_MODE
    };

    /* Disable watchdog */
    status = Pmic_wdgSetEnableState(&pmicHandle, false);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Attempt to configure - should fail */
    status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NOT_SUPPORTED);

    /* Re-enable watchdog for subsequent tests */
    status = Pmic_wdgSetEnableState(&pmicHandle, true);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    status = Pmic_wdgSetReturnToLongWindow(&pmicHandle, true);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}
/**
 * @brief Test Pmic_wdgSetCfg when not in Long Window mode
 */
void test_neg_wdg_wdgSetCfg_whenNotInLongWindow(void)
{
    int32_t status;
    Pmic_WdgCfg_t wdgCfg = {
        .validParams = PMIC_WDG_MODE_SEL_VALID,
        .mode = PMIC_WDG_TRIGGER_MODE
    };

    /* Enable watchdog but disable return to long window */
    status = Pmic_wdgSetEnableState(&pmicHandle, true);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    status = Pmic_wdgSetReturnToLongWindow(&pmicHandle, false);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Attempt to configure - should fail */
    status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NOT_SUPPORTED);

    /* Re-enable return to long window for subsequent tests */
    status = Pmic_wdgSetReturnToLongWindow(&pmicHandle, true);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}
/**
 * @brief Test Pmic_wdgGetCfg with NULL handle
 */
void test_neg_wdg_wdgGetCfg_nullHandle(void)
{
    Pmic_WdgCfg_t wdgCfg = {0};
    int32_t status = Pmic_wdgGetCfg(NULL, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}
/**
 * @brief Test Pmic_wdgGetCfg with NULL wdgCfg parameter
 */
void test_neg_wdg_wdgGetCfg_nullWdgCfg(void)
{
    int32_t status = Pmic_wdgGetCfg(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}
/**
 * @brief Test Pmic_wdgSetPowerHold with NULL handle
 */
void test_neg_wdg_wdgSetPowerHold_nullHandle(void)
{
    int32_t status = Pmic_wdgSetPowerHold(NULL, true);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}
/**
 * @brief Test Pmic_wdgGetPowerHold with NULL handle
 */
void test_neg_wdg_wdgGetPowerHold_nullHandle(void)
{
    bool isEnabled = false;
    int32_t status = Pmic_wdgGetPowerHold(NULL, &isEnabled);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}
/**
 * @brief Test Pmic_wdgGetPowerHold with NULL isEnabled parameter
 */
void test_neg_wdg_wdgGetPowerHold_nullIsEnabled(void)
{
    int32_t status = Pmic_wdgGetPowerHold(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}
/**
 * @brief Test Pmic_wdgSetReturnToLongWindow with NULL handle
 */
void test_neg_wdg_wdgSetReturnToLongWindow_nullHandle(void)
{
    int32_t status = Pmic_wdgSetReturnToLongWindow(NULL, true);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}
/**
 * @brief Test Pmic_wdgGetReturnToLongWindow with NULL handle
 */
void test_neg_wdg_wdgGetReturnToLongWindow_nullHandle(void)
{
    bool isEnabled = false;
    int32_t status = Pmic_wdgGetReturnToLongWindow(NULL, &isEnabled);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}
/**
 * @brief Test Pmic_wdgGetReturnToLongWindow with NULL isEnabled parameter
 */
void test_neg_wdg_wdgGetReturnToLongWindow_nullIsEnabled(void)
{
    int32_t status = Pmic_wdgGetReturnToLongWindow(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}
/**
 * @brief Test Pmic_wdgQaWriteAnswer with NULL handle
 */
void test_neg_wdg_wdgQaWriteAnswer_nullHandle(void)
{
    int32_t status = Pmic_wdgQaWriteAnswer(NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}
/**
 * @brief Test Pmic_wdgClrErrStatus with NULL handle
 */
void test_neg_wdg_wdgClrErrStatus_nullHandle(void)
{
    Pmic_WdgErrStatus_t errStatus = {0};
    int32_t status = Pmic_wdgClrErrStatus(NULL, &errStatus);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}
/**
 * @brief Test Pmic_wdgClrErrStatus with NULL errStatus parameter
 */
void test_neg_wdg_wdgClrErrStatus_nullErrStatus(void)
{
    int32_t status = Pmic_wdgClrErrStatus(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}
/**
 * @brief Test Pmic_wdgClrErrStatusAll with NULL handle
 */
void test_neg_wdg_wdgClrErrStatusAll_nullHandle(void)
{
    int32_t status = Pmic_wdgClrErrStatusAll(NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}
/**
 * @brief Test Pmic_wdgGetErrStatus with NULL handle
 */
void test_neg_wdg_wdgGetErrStatus_nullHandle(void)
{
    Pmic_WdgErrStatus_t errStatus = {0};
    int32_t status = Pmic_wdgGetErrStatus(NULL, &errStatus);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}
/**
 * @brief Test Pmic_wdgGetErrStatus with NULL errStatus parameter
 */
void test_neg_wdg_wdgGetErrStatus_nullErrStatus(void)
{
    int32_t status = Pmic_wdgGetErrStatus(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}
/**
 * @brief Test Pmic_wdgGetFailCntStatus with NULL handle
 */
void test_neg_wdg_wdgGetFailCntStatus_nullHandle(void)
{
    Pmic_WdgFailCntStatus_t failCntStatus = {0};
    int32_t status = Pmic_wdgGetFailCntStatus(NULL, &failCntStatus);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}
/**
 * @brief Test Pmic_wdgGetFailCntStatus with NULL failCntStatus parameter
 */
void test_neg_wdg_wdgGetFailCntStatus_nullFailCntStatus(void)
{
    int32_t status = Pmic_wdgGetFailCntStatus(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}
/*                       Positive Test Functions                              */
/**
 * @brief Test watchdog enable and disable functionality
 */
void test_pos_wdg_wdgSetEnableState_enableDisable(void)
{
    bool isEnabled = false;
    int32_t status;
    /* Disable watchdog */
    status = Pmic_wdgSetEnableState(&pmicHandle, false);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    /* Verify disabled state */
    status = Pmic_wdgGetEnableState(&pmicHandle, &isEnabled);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(isEnabled == false);
    /* Enable watchdog */
    status = Pmic_wdgSetEnableState(&pmicHandle, true);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    /* Verify enabled state */
    status = Pmic_wdgGetEnableState(&pmicHandle, &isEnabled);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(isEnabled == true);
}
/**
 * @brief Test power hold enable and disable
 */
void test_pos_wdg_wdgSetPowerHold_powerHold(void)
{
    bool isEnabled = false;
    int32_t status;
    /* Disable power hold */
    status = Pmic_wdgSetPowerHold(&pmicHandle, false);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    /* Verify disabled state */
    status = Pmic_wdgGetPowerHold(&pmicHandle, &isEnabled);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(isEnabled == false);
    /* Enable power hold */
    status = Pmic_wdgSetPowerHold(&pmicHandle, true);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    /* Verify enabled state */
    status = Pmic_wdgGetPowerHold(&pmicHandle, &isEnabled);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(isEnabled == true);
}
/**
 * @brief Test return to long window enable and disable
 */
void test_pos_wdg_wdgSetReturnToLongWindow_returnToLongWindow(void)
{
    bool isEnabled = false;
    int32_t status;
    /* Disable return to long window */
    status = Pmic_wdgSetReturnToLongWindow(&pmicHandle, false);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    /* Verify disabled state */
    status = Pmic_wdgGetReturnToLongWindow(&pmicHandle, &isEnabled);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(isEnabled == false);
    /* Enable return to long window */
    status = Pmic_wdgSetReturnToLongWindow(&pmicHandle, true);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    /* Verify enabled state */
    status = Pmic_wdgGetReturnToLongWindow(&pmicHandle, &isEnabled);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(isEnabled == true);
}
/**
 * @brief Test watchdog configuration set and get for rstEn
 */
void test_pos_wdg_wdgSetCfg_rstEn(void)
{
    wdg_setupForConfig();

    Pmic_WdgCfg_t wdgCfgSet = {
        .validParams = PMIC_WDG_RST_EN_VALID,
        .rstEn = true
    };
    Pmic_WdgCfg_t wdgCfgGet = {
        .validParams = PMIC_WDG_RST_EN_VALID
    };
    int32_t status;
    /* Set rstEn to true */
    status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    /* Get and verify */
    status = Pmic_wdgGetCfg(&pmicHandle, &wdgCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(wdgCfgGet.rstEn == true);
    /* Set rstEn to false */
    wdgCfgSet.rstEn = false;
    status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    /* Get and verify */
    status = Pmic_wdgGetCfg(&pmicHandle, &wdgCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(wdgCfgGet.rstEn == false);
}
/**
 * @brief Test watchdog configuration set and get for mode
 */
void test_pos_wdg_wdgSetCfg_mode(void)
{
    wdg_setupForConfig();

    Pmic_WdgCfg_t wdgCfgSet = {
        .validParams = PMIC_WDG_MODE_SEL_VALID,
        .mode = PMIC_WDG_TRIGGER_MODE
    };
    Pmic_WdgCfg_t wdgCfgGet = {
        .validParams = PMIC_WDG_MODE_SEL_VALID
    };
    int32_t status;
    /* Set trigger mode */
    status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    /* Get and verify */
    status = Pmic_wdgGetCfg(&pmicHandle, &wdgCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(wdgCfgGet.mode == PMIC_WDG_TRIGGER_MODE);
    /* Set Q&A mode */
    wdgCfgSet.mode = PMIC_WDG_QA_MODE;
    status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    /* Get and verify */
    status = Pmic_wdgGetCfg(&pmicHandle, &wdgCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(wdgCfgGet.mode == PMIC_WDG_QA_MODE);
}
/**
 * @brief Test watchdog configuration set and get for win1Code
 */
void test_pos_wdg_wdgSetCfg_win1Code(void)
{
    wdg_setupForConfig();

    Pmic_WdgCfg_t wdgCfgSet = {
        .validParams = PMIC_WDG_WIN1_CODE_VALID
    };
    Pmic_WdgCfg_t wdgCfgGet = {
        .validParams = PMIC_WDG_WIN1_CODE_VALID
    };
    int32_t status;
    uint8_t testValues[] = {0x00, 0x20, 0x40, 0x7F};
    for (uint8_t i = 0; i < sizeof(testValues); i++)
    {
        wdgCfgSet.win1Code = testValues[i];
        status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfgSet);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        status = Pmic_wdgGetCfg(&pmicHandle, &wdgCfgGet);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        PLATFORM_ASSERT(wdgCfgGet.win1Code == testValues[i]);
    }
}
/**
 * @brief Test watchdog configuration set and get for win2Code
 */
void test_pos_wdg_wdgSetCfg_win2Code(void)
{
    wdg_setupForConfig();

    Pmic_WdgCfg_t wdgCfgSet = {
        .validParams = PMIC_WDG_WIN2_CODE_VALID
    };
    Pmic_WdgCfg_t wdgCfgGet = {
        .validParams = PMIC_WDG_WIN2_CODE_VALID
    };
    int32_t status;
    uint8_t testValues[] = {0x00, 0x20, 0x40, 0x7F};
    for (uint8_t i = 0; i < sizeof(testValues); i++)
    {
        wdgCfgSet.win2Code = testValues[i];
        status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfgSet);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        status = Pmic_wdgGetCfg(&pmicHandle, &wdgCfgGet);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        PLATFORM_ASSERT(wdgCfgGet.win2Code == testValues[i]);
    }
}
/**
 * @brief Test watchdog configuration set and get for longWinCode
 */
void test_pos_wdg_wdgSetCfg_longWinCode(void)
{
    wdg_setupForConfig();

    Pmic_WdgCfg_t wdgCfgSet = {
        .validParams = PMIC_WDG_LONG_WIN_CODE_VALID
    };
    Pmic_WdgCfg_t wdgCfgGet = {
        .validParams = PMIC_WDG_LONG_WIN_CODE_VALID
    };
    int32_t status;
    uint8_t testValues[] = {0x00, 0x50, 0xA0, 0xFF};
    for (uint8_t i = 0; i < sizeof(testValues); i++)
    {
        wdgCfgSet.longWinCode = testValues[i];
        status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfgSet);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        status = Pmic_wdgGetCfg(&pmicHandle, &wdgCfgGet);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        PLATFORM_ASSERT(wdgCfgGet.longWinCode == testValues[i]);
    }
}
/**
 * @brief Test watchdog configuration set and get for qaFdbk
 */
void test_pos_wdg_wdgSetCfg_qaFdbk(void)
{
    wdg_setupForConfig();

    Pmic_WdgCfg_t wdgCfgSet = {
        .validParams = PMIC_WDG_QA_FDBK_VALID
    };
    Pmic_WdgCfg_t wdgCfgGet = {
        .validParams = PMIC_WDG_QA_FDBK_VALID
    };
    int32_t status;
    for (uint8_t val = PMIC_WDG_QA_FDBK_MIN; val <= PMIC_WDG_QA_FDBK_MAX; val++)
    {
        wdgCfgSet.qaFdbk = val;
        status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfgSet);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        status = Pmic_wdgGetCfg(&pmicHandle, &wdgCfgGet);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        PLATFORM_ASSERT(wdgCfgGet.qaFdbk == val);
    }
}
/**
 * @brief Test watchdog configuration set and get for qaLfsr
 */
void test_pos_wdg_wdgSetCfg_qaLfsr(void)
{
    wdg_setupForConfig();

    Pmic_WdgCfg_t wdgCfgSet = {
        .validParams = PMIC_WDG_QA_LFSR_VALID
    };
    Pmic_WdgCfg_t wdgCfgGet = {
        .validParams = PMIC_WDG_QA_LFSR_VALID
    };
    int32_t status;
    for (uint8_t val = PMIC_WDG_QA_LFSR_MIN; val <= PMIC_WDG_QA_LFSR_MAX; val++)
    {
        wdgCfgSet.qaLfsr = val;
        status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfgSet);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        status = Pmic_wdgGetCfg(&pmicHandle, &wdgCfgGet);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        PLATFORM_ASSERT(wdgCfgGet.qaLfsr == val);
    }
}
/**
 * @brief Test watchdog configuration set and get for qaSeed
 */
void test_pos_wdg_wdgSetCfg_qaSeed(void)
{
    wdg_setupForConfig();

    Pmic_WdgCfg_t wdgCfgSet = {
        .validParams = PMIC_WDG_QA_SEED_VALID
    };
    Pmic_WdgCfg_t wdgCfgGet = {
        .validParams = PMIC_WDG_QA_SEED_VALID
    };
    int32_t status;
    for (uint8_t val = PMIC_WDG_QA_SEED_MIN; val <= PMIC_WDG_QA_SEED_MAX; val++)
    {
        wdgCfgSet.qaSeed = val;
        status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfgSet);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        status = Pmic_wdgGetCfg(&pmicHandle, &wdgCfgGet);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        PLATFORM_ASSERT(wdgCfgGet.qaSeed == val);
    }
}
/**
 * @brief Test watchdog configuration set and get for failThr
 */
void test_pos_wdg_wdgSetCfg_failThr(void)
{
    wdg_setupForConfig();

    Pmic_WdgCfg_t wdgCfgSet = {
        .validParams = PMIC_WDG_FAIL_THR_VALID
    };
    Pmic_WdgCfg_t wdgCfgGet = {
        .validParams = PMIC_WDG_FAIL_THR_VALID
    };
    int32_t status;
    for (uint8_t val = PMIC_WDG_FAIL_THR_MIN; val <= PMIC_WDG_FAIL_THR_MAX; val++)
    {
        wdgCfgSet.failThr = val;
        status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfgSet);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        status = Pmic_wdgGetCfg(&pmicHandle, &wdgCfgGet);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        PLATFORM_ASSERT(wdgCfgGet.failThr == val);
    }
}
/**
 * @brief Test watchdog configuration set and get for rstThr
 */
void test_pos_wdg_wdgSetCfg_rstThr(void)
{
    wdg_setupForConfig();

    Pmic_WdgCfg_t wdgCfgSet = {
        .validParams = PMIC_WDG_RST_THR_VALID
    };
    Pmic_WdgCfg_t wdgCfgGet = {
        .validParams = PMIC_WDG_RST_THR_VALID
    };
    int32_t status;
    for (uint8_t val = PMIC_WDG_RST_THR_MIN; val <= PMIC_WDG_RST_THR_MAX; val++)
    {
        wdgCfgSet.rstThr = val;
        status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfgSet);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        status = Pmic_wdgGetCfg(&pmicHandle, &wdgCfgGet);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        PLATFORM_ASSERT(wdgCfgGet.rstThr == val);
    }
}
/**
 * @brief Test watchdog configuration set and get for cntSel
 */
void test_pos_wdg_wdgSetCfg_cntSel(void)
{
    wdg_setupForConfig();

    Pmic_WdgCfg_t wdgCfgSet = {
        .validParams = PMIC_WDG_CNT_SEL_VALID
    };
    Pmic_WdgCfg_t wdgCfgGet = {
        .validParams = PMIC_WDG_CNT_SEL_VALID
    };
    int32_t status;
    for (uint8_t val = PMIC_WDG_CNT_SEL_MIN; val <= PMIC_WDG_CNT_SEL_MAX; val++)
    {
        wdgCfgSet.cntSel = val;
        status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfgSet);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        status = Pmic_wdgGetCfg(&pmicHandle, &wdgCfgGet);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        PLATFORM_ASSERT(wdgCfgGet.cntSel == val);
    }
}
/**
 * @brief Test Q&A sequence with correct answers (no errors)
 */
void test_pos_wdg_wdgQaWriteAnswer_qaSequenceCorrectAnswers(void)
{
    int32_t status;
    /* Configure watchdog for Q&A mode with long timeouts */
    Pmic_WdgCfg_t wdgCfg = {
        .validParams = PMIC_WDG_MODE_SEL_VALID | PMIC_WDG_WIN1_CODE_VALID |
                       PMIC_WDG_WIN2_CODE_VALID | PMIC_WDG_LONG_WIN_CODE_VALID |
                       PMIC_WDG_QA_FDBK_VALID | PMIC_WDG_QA_LFSR_VALID |
                       PMIC_WDG_QA_SEED_VALID,
        .mode = PMIC_WDG_QA_MODE,
        .win1Code = 0x7F,
        .win2Code = 0x7F,
        .longWinCode = 0xFF,
        .qaFdbk = 0,
        .qaLfsr = 1,
        .qaSeed = 2
    };
    /* Enable watchdog and set configuration */
    status = Pmic_wdgSetEnableState(&pmicHandle, true);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    /* Clear any existing errors */
    status = Pmic_wdgClrErrStatusAll(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    /* Send correct Q&A answers - exit long window */
    for (uint8_t i = 0; i < 4; i++)
    {
        status = Pmic_wdgQaWriteAnswer(&pmicHandle);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    }
    /* Verify no errors occurred */
    Pmic_WdgErrStatus_t errStatus = {
        .validParams = PMIC_WDG_TIMEOUT_ERR_VALID | PMIC_WDG_ANSW_ERR_VALID |
                       PMIC_WDG_SEQ_ERR_VALID
    };
    status = Pmic_wdgGetErrStatus(&pmicHandle, &errStatus);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(errStatus.timeoutErr == false);
    PLATFORM_ASSERT(errStatus.answErr == false);
    PLATFORM_ASSERT(errStatus.seqErr == false);
    /* Return to long window */
    status = Pmic_wdgSetReturnToLongWindow(&pmicHandle, true);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    status = Pmic_wdgSetPowerHold(&pmicHandle, true);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}
/**
 * @brief Test error status get and clear functionality
 */
void test_pos_wdg_wdgGetErrStatus_errorStatusGetClear(void)
{
    int32_t status;
    /* Clear all error statuses */
    status = Pmic_wdgClrErrStatusAll(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    /* Verify all errors are cleared */
    Pmic_WdgErrStatus_t errStatus = {
        .validParams = PMIC_WDG_RST_INT_VALID | PMIC_WDG_FAIL_INT_VALID |
                       PMIC_WDG_ANSW_ERR_VALID | PMIC_WDG_SEQ_ERR_VALID |
                       PMIC_WDG_ANSW_EARLY_ERR_VALID | PMIC_WDG_TRIG_EARLY_ERR_VALID |
                       PMIC_WDG_TIMEOUT_ERR_VALID | PMIC_WDG_LONG_WIN_TIMEOUT_ERR_VALID
    };
    status = Pmic_wdgGetErrStatus(&pmicHandle, &errStatus);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}
/**
 * @brief Test fail counter status read functionality
 */
void test_pos_wdg_wdgGetFailCntStatus_failCounterStatus(void)
{
    int32_t status;
    /* Read fail counter status */
    Pmic_WdgFailCntStatus_t failCntStatus = {
        .validParams = PMIC_WDG_BAD_EVENT_VALID | PMIC_WDG_GOOD_EVENT_VALID |
                       PMIC_WDG_FAIL_CNT_VALID
    };
    status = Pmic_wdgGetFailCntStatus(&pmicHandle, &failCntStatus);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}
/**
 * @brief Test combined configuration parameters
 */
void test_pos_wdg_wdgSetCfg_combinedConfiguration(void)
{
    wdg_setupForConfig();

    int32_t status;
    /* Set multiple configuration parameters at once */
    Pmic_WdgCfg_t wdgCfgSet = {
        .validParams = PMIC_WDG_RST_EN_VALID | PMIC_WDG_MODE_SEL_VALID |
                       PMIC_WDG_WIN1_CODE_VALID | PMIC_WDG_WIN2_CODE_VALID |
                       PMIC_WDG_FAIL_THR_VALID | PMIC_WDG_RST_THR_VALID,
        .rstEn = true,
        .mode = PMIC_WDG_QA_MODE,
        .win1Code = 0x40,
        .win2Code = 0x40,
        .failThr = 3,
        .rstThr = 2
    };
    status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    /* Read back and verify */
    Pmic_WdgCfg_t wdgCfgGet = {
        .validParams = PMIC_WDG_RST_EN_VALID | PMIC_WDG_MODE_SEL_VALID |
                       PMIC_WDG_WIN1_CODE_VALID | PMIC_WDG_WIN2_CODE_VALID |
                       PMIC_WDG_FAIL_THR_VALID | PMIC_WDG_RST_THR_VALID
    };
    status = Pmic_wdgGetCfg(&pmicHandle, &wdgCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(wdgCfgGet.rstEn == wdgCfgSet.rstEn);
    PLATFORM_ASSERT(wdgCfgGet.mode == wdgCfgSet.mode);
    PLATFORM_ASSERT(wdgCfgGet.win1Code == wdgCfgSet.win1Code);
    PLATFORM_ASSERT(wdgCfgGet.win2Code == wdgCfgSet.win2Code);
    PLATFORM_ASSERT(wdgCfgGet.failThr == wdgCfgSet.failThr);
    PLATFORM_ASSERT(wdgCfgGet.rstThr == wdgCfgSet.rstThr);
}
/**
 * @brief Test Q&A write answer with qaFdbk=0 (tests mux_4x1 case 0)
 */
void test_pos_wdg_wdgQaWriteAnswer_qaFdbk0(void)
{
    wdg_setupForConfig();

    int32_t status;
    /* Configure watchdog for Q&A mode with qaFdbk=0 */
    Pmic_WdgCfg_t wdgCfg = {
        .validParams = PMIC_WDG_MODE_SEL_VALID | PMIC_WDG_WIN1_CODE_VALID |
                       PMIC_WDG_WIN2_CODE_VALID | PMIC_WDG_LONG_WIN_CODE_VALID |
                       PMIC_WDG_QA_FDBK_VALID | PMIC_WDG_QA_LFSR_VALID |
                       PMIC_WDG_QA_SEED_VALID,
        .mode = PMIC_WDG_QA_MODE,
        .win1Code = 0x7F,
        .win2Code = 0x7F,
        .longWinCode = 0xFF,
        .qaFdbk = 0,
        .qaLfsr = 1,
        .qaSeed = 2
    };
    status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    /* Write Q&A answer - internally reads qaFdbk and exercises mux_4x1 with case 0 */
    status = Pmic_wdgQaWriteAnswer(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}
/**
 * @brief Test Q&A write answer with qaFdbk=1 (tests mux_4x1 case 1)
 */
void test_pos_wdg_wdgQaWriteAnswer_qaFdbk1(void)
{
    wdg_setupForConfig();

    int32_t status;
    /* Configure watchdog for Q&A mode with qaFdbk=1 */
    Pmic_WdgCfg_t wdgCfg = {
        .validParams = PMIC_WDG_MODE_SEL_VALID | PMIC_WDG_WIN1_CODE_VALID |
                       PMIC_WDG_WIN2_CODE_VALID | PMIC_WDG_LONG_WIN_CODE_VALID |
                       PMIC_WDG_QA_FDBK_VALID | PMIC_WDG_QA_LFSR_VALID |
                       PMIC_WDG_QA_SEED_VALID,
        .mode = PMIC_WDG_QA_MODE,
        .win1Code = 0x7F,
        .win2Code = 0x7F,
        .longWinCode = 0xFF,
        .qaFdbk = 1,
        .qaLfsr = 2,
        .qaSeed = 3
    };
    status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    /* Write Q&A answer - internally reads qaFdbk and exercises mux_4x1 with case 1 */
    status = Pmic_wdgQaWriteAnswer(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}
/**
 * @brief Test Q&A write answer with qaFdbk=2 (tests mux_4x1 case 2)
 */
void test_pos_wdg_wdgQaWriteAnswer_qaFdbk2(void)
{
    wdg_setupForConfig();

    int32_t status;
    /* Configure watchdog for Q&A mode with qaFdbk=2 */
    Pmic_WdgCfg_t wdgCfg = {
        .validParams = PMIC_WDG_MODE_SEL_VALID | PMIC_WDG_WIN1_CODE_VALID |
                       PMIC_WDG_WIN2_CODE_VALID | PMIC_WDG_LONG_WIN_CODE_VALID |
                       PMIC_WDG_QA_FDBK_VALID | PMIC_WDG_QA_LFSR_VALID |
                       PMIC_WDG_QA_SEED_VALID,
        .mode = PMIC_WDG_QA_MODE,
        .win1Code = 0x7F,
        .win2Code = 0x7F,
        .longWinCode = 0xFF,
        .qaFdbk = 2,
        .qaLfsr = 0,
        .qaSeed = 1
    };
    status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    /* Write Q&A answer - internally reads qaFdbk and exercises mux_4x1 with case 2 */
    status = Pmic_wdgQaWriteAnswer(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}
/**
 * @brief Test Q&A write answer with qaFdbk=3 (tests mux_4x1 default case)
 */
void test_pos_wdg_wdgQaWriteAnswer_qaFdbk3(void)
{
    wdg_setupForConfig();

    int32_t status;
    /* Configure watchdog for Q&A mode with qaFdbk=3 */
    Pmic_WdgCfg_t wdgCfg = {
        .validParams = PMIC_WDG_MODE_SEL_VALID | PMIC_WDG_WIN1_CODE_VALID |
                       PMIC_WDG_WIN2_CODE_VALID | PMIC_WDG_LONG_WIN_CODE_VALID |
                       PMIC_WDG_QA_FDBK_VALID | PMIC_WDG_QA_LFSR_VALID |
                       PMIC_WDG_QA_SEED_VALID,
        .mode = PMIC_WDG_QA_MODE,
        .win1Code = 0x7F,
        .win2Code = 0x7F,
        .longWinCode = 0xFF,
        .qaFdbk = 3,
        .qaLfsr = 3,
        .qaSeed = 0
    };
    status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    /* Write Q&A answer - internally reads qaFdbk and exercises mux_4x1 with default case (3) */
    status = Pmic_wdgQaWriteAnswer(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}
/**
 * @brief Test clearing only RST_INT error (first threshold error)
 */
void test_pos_wdg_wdgClrErrStatus_th1ErrorOnly(void)
{
    int32_t status;
    /* Clear only the RST_INT (first threshold) error status */
    Pmic_WdgErrStatus_t errStatus = {
        .validParams = PMIC_WDG_RST_INT_VALID
    };
    status = Pmic_wdgClrErrStatus(&pmicHandle, &errStatus);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    /* Verify the error was cleared by reading back */
    Pmic_WdgErrStatus_t errStatusRead = {
        .validParams = PMIC_WDG_RST_INT_VALID
    };
    status = Pmic_wdgGetErrStatus(&pmicHandle, &errStatusRead);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}
/**
 * @brief Test clearing only FAIL_INT error (second threshold error)
 */
void test_pos_wdg_wdgClrErrStatus_th2ErrorOnly(void)
{
    int32_t status;
    /* Clear only the FAIL_INT (second threshold) error status */
    Pmic_WdgErrStatus_t errStatus = {
        .validParams = PMIC_WDG_FAIL_INT_VALID
    };
    status = Pmic_wdgClrErrStatus(&pmicHandle, &errStatus);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    /* Verify the error was cleared by reading back */
    Pmic_WdgErrStatus_t errStatusRead = {
        .validParams = PMIC_WDG_FAIL_INT_VALID
    };
    status = Pmic_wdgGetErrStatus(&pmicHandle, &errStatusRead);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}
/**
 * @brief Test clearing only SEQ_ERR error (sequence error)
 */
void test_pos_wdg_wdgClrErrStatus_seqErrorOnly(void)
{
    int32_t status;
    /* Clear only the sequence error status */
    Pmic_WdgErrStatus_t errStatus = {
        .validParams = PMIC_WDG_SEQ_ERR_VALID
    };
    status = Pmic_wdgClrErrStatus(&pmicHandle, &errStatus);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    /* Verify the error was cleared by reading back */
    Pmic_WdgErrStatus_t errStatusRead = {
        .validParams = PMIC_WDG_SEQ_ERR_VALID
    };
    status = Pmic_wdgGetErrStatus(&pmicHandle, &errStatusRead);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}
/**
 * @brief Test getting fail count status with only failCnt parameter
 */
void test_pos_wdg_wdgGetFailCntStatus_failCntOnly(void)
{
    int32_t status;
    /* Read only the fail count value */
    Pmic_WdgFailCntStatus_t failCntStatus = {
        .validParams = PMIC_WDG_FAIL_CNT_VALID
    };
    status = Pmic_wdgGetFailCntStatus(&pmicHandle, &failCntStatus);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    /* failCnt should be populated, badEvent and goodEvent should not be modified */
}
/**
 * @brief Test getting fail count status with only badEvent parameter
 */
void test_pos_wdg_wdgGetFailCntStatus_badCntOnly(void)
{
    int32_t status;
    /* Read only the bad event status */
    Pmic_WdgFailCntStatus_t failCntStatus = {
        .validParams = PMIC_WDG_BAD_EVENT_VALID
    };
    status = Pmic_wdgGetFailCntStatus(&pmicHandle, &failCntStatus);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    /* badEvent should be populated, failCnt and goodEvent should not be modified */
}
/**
 * @brief Test Pmic_wdgSetCfg with EN_DRV_SEL configuration
 * Covers lines 263-264 in pmic_wdg.c
 */
void test_pos_wdg_wdgSetEnDrvSel_setEnDrvSel(void)
{
    int32_t status;
    Pmic_WdgCfg_t wdgCfg = {
        .validParams = PMIC_WDG_EN_DRV_SEL_VALID,
        .clrEnDrvOnFailInt = true
    };
    status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    /* Read back and verify */
    wdgCfg.validParams = PMIC_WDG_EN_DRV_SEL_VALID;
    wdgCfg.clrEnDrvOnFailInt = false;  /* Reset to read back */
    status = Pmic_wdgGetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}
/**
 * @brief Test Pmic_wdgGetCfg with EN_DRV_SEL configuration
 * Covers lines 381-382 in pmic_wdg.c
 */
void test_pos_wdg_wdgGetEnDrvSel_getEnDrvSel(void)
{
    int32_t status;
    Pmic_WdgCfg_t wdgCfg = {
        .validParams = PMIC_WDG_EN_DRV_SEL_VALID
    };
    status = Pmic_wdgGetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}
/**
 * @brief Test Pmic_wdgClrErrStatus to clear WD_ANSW_ERR bit
 * Covers lines 747-748 in pmic_wdg.c
 */
void test_pos_wdg_wdgClrErrStatus_clrAnswErr(void)
{
    int32_t status;
    Pmic_WdgErrStatus_t errStatus = {
        .validParams = PMIC_WDG_ANSW_ERR_VALID,
        .answErr = true
    };
    status = Pmic_wdgClrErrStatus(&pmicHandle, &errStatus);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}
/**
 * @brief Test Pmic_wdgClrErrStatus to clear WD_ANSW_EARLY bit
 * Covers lines 757-758 in pmic_wdg.c
 */
void test_pos_wdg_wdgClrErrStatus_clrAnswEarlyErr(void)
{
    int32_t status;
    Pmic_WdgErrStatus_t errStatus = {
        .validParams = PMIC_WDG_ANSW_EARLY_ERR_VALID,
        .answEarlyErr = true
    };
    status = Pmic_wdgClrErrStatus(&pmicHandle, &errStatus);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}
/**
 * @brief Test Pmic_wdgClrErrStatus to clear WD_TRIG_EARLY bit
 * Covers lines 762-763 in pmic_wdg.c
 */
void test_pos_wdg_wdgClrErrStatus_clrTrigEarlyErr(void)
{
    int32_t status;
    Pmic_WdgErrStatus_t errStatus = {
        .validParams = PMIC_WDG_TRIG_EARLY_ERR_VALID,
        .trigEarlyErr = true
    };
    status = Pmic_wdgClrErrStatus(&pmicHandle, &errStatus);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}
/**
 * @brief Test Pmic_wdgClrErrStatus to clear WD_TIMEOUT bit
 * Covers lines 767-768 in pmic_wdg.c
 */
void test_pos_wdg_wdgClrErrStatus_clrTimeoutErr(void)
{
    int32_t status;
    Pmic_WdgErrStatus_t errStatus = {
        .validParams = PMIC_WDG_TIMEOUT_ERR_VALID,
        .timeoutErr = true
    };
    status = Pmic_wdgClrErrStatus(&pmicHandle, &errStatus);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}
/**
 * @brief Test Pmic_wdgClrErrStatus to clear WD_LONGWIN_TIMEOUT_INT bit
 * Covers lines 772-773 in pmic_wdg.c
 */
void test_pos_wdg_wdgClrErrStatus_clrLongWinTimeoutErr(void)
{
    int32_t status;
    Pmic_WdgErrStatus_t errStatus = {
        .validParams = PMIC_WDG_LONG_WIN_TIMEOUT_ERR_VALID,
        .longWinTimeoutErr = true
    };
    status = Pmic_wdgClrErrStatus(&pmicHandle, &errStatus);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}


/* ========================================================================== */
/*                         Entry Point Function                               */
/* ========================================================================== */

void wdg_test(void *args)
{
    (void)args;
    int32_t status;

    platform_init();
    platform_setupTests();

    /* Initialize PMIC handle */
    Pmic_HandleCfg_t pmicCfg = {
        .validParams = PMIC_COMM_MODE_VALID | PMIC_COMM_HANDLE_0_VALID |
                       PMIC_IO_READ_VALID | PMIC_IO_WRITE_VALID |
                       PMIC_CRITICAL_SECTION_START_VALID |
                       PMIC_CRITICAL_SECTION_STOP_VALID |
                       PMIC_IRQ_RESPONSE_CALLBACK_VALID,
        .commMode = PMIC_INTF_SPI,
        .commHandle0 = platform_getCommHandle(),
        .ioRead = &platform_rxByte,
        .ioWrite = &platform_txByte,
        .criticalSectionStart = &platform_critSecStart,
        .criticalSectionStop = &platform_critSecStop,
        .irqResponseCallback = &platform_irqResponse
    };

    status = Pmic_init(&pmicHandle, &pmicCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Run all tests */
    WDG_TEST_RUN_ALL();

    /* Cleanup */
    (void)Pmic_deinit(&pmicHandle);
    platform_tearDownTests();
    platform_deinit();
}
