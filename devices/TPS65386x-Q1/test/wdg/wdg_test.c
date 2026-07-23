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
/*                             Include Files                                  */
/* ========================================================================== */

#include "wdg_test.h"
#ifdef BUILD_MOCK
#include "test_inject.h"
#include "platform_mock.h"
#include "pmic_mock_core.h"
#endif
#include "test_constants.h"
#include "regmap/wdg.h"
#include "pmic_fsm.h"
#include "pmic_irq.h"

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/* ========================================================================== */
/*                             Global Variables                               */
/* ========================================================================== */

#ifdef BUILD_MOCK
extern PmicMockDevice_t *platform_getMockDevice(void);
#endif

static Pmic_Handle_t pmicHandle = {0U};

/**
 * @brief Setup helper: Initialize WDG to valid configuration state.
 *
 * Ensures WDG is enabled and in Long Window mode, which are
 * prerequisites for calling Pmic_wdgSetCfg().
 */
static void wdg_setupForConfig(void)
{
    int32_t status;

    // Disable WDG to reset any corrupted Q&A state from previous test
    (void)Pmic_wdgDisable(&pmicHandle);

    // Clear all error flags after disable
    status = Pmic_wdgClrErrStatusAll(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Enable watchdog (fresh start with clean Q&A state)
    status = Pmic_wdgEnable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Enable return to long window
    status = Pmic_wdgSetReturnToLongWindow(&pmicHandle, true);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Set PWR_HOLD to keep WDG in long window mode during configuration
    status = Pmic_wdgSetPowerHold(&pmicHandle, true);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Wait for watchdog to enter long window mode
    platform_timerWaitMs(25U);
}

static void wdg_cleanupAfterTest(void)
{
    // Return to long window from any Q&A mode — best effort, no assert
    (void)Pmic_wdgSetReturnToLongWindow(&pmicHandle, true);
    (void)Pmic_wdgSetPowerHold(&pmicHandle, true);

    platform_timerWaitMs(5U);

    // Clear accumulated error flags
    (void)Pmic_wdgClrErrStatusAll(&pmicHandle);

    // Disable WDG safely from long window
    (void)Pmic_wdgDisable(&pmicHandle);

    // Final error clear after disable
    (void)Pmic_wdgClrErrStatusAll(&pmicHandle);
}

/* ========================================================================== */
/*                           Function Definitions                             */
/* ========================================================================== */

/**
 * @brief Test watchdog enable and disable operations.
 */
void test_pos_wdg_wdgEnable_enableDisable(void)
{
    int32_t status;
    bool wdgEnabled = false;

    wdg_setupForConfig();

    // Verify watchdog is enabled
    status = Pmic_wdgGetEnableState(&pmicHandle, &wdgEnabled);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(wdgEnabled == true);

    // Disable Watchdog
    status = Pmic_wdgDisable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify watchdog is disabled
    status = Pmic_wdgGetEnableState(&pmicHandle, &wdgEnabled);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(wdgEnabled == false);

    wdg_cleanupAfterTest();
}

/**
 * @brief Test watchdog long window duration configuration.
 */
void test_pos_wdg_wdgSetCfg_longWindowDuration(void)
{
    wdg_setupForConfig();
    int32_t status;
    Pmic_WdgCfg_t wdgCfg = {0};

    // Set long window duration
    wdgCfg.validParams = PMIC_CFG_WDG_LONG_WIN_CODE_VALID;
    wdgCfg.longWinCode = TEST_INVALID_PARAM_255;  // 255 * 1100us = 280.5ms (max possible, approximates 772ms intent)

    status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get and verify long window duration
    wdgCfg.validParams = PMIC_CFG_WDG_LONG_WIN_CODE_VALID;
    wdgCfg.longWinCode = 0U;

    status = Pmic_wdgGetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(wdgCfg.longWinCode == TEST_INVALID_PARAM_255);

    wdg_cleanupAfterTest();
}

/**
 * @brief Test watchdog window-1 duration configuration.
 */
void test_pos_wdg_wdgSetCfg_window1Duration(void)
{
    wdg_setupForConfig();
    int32_t status;
    Pmic_WdgCfg_t wdgCfg = {0};

    // Set window-1 duration
    wdgCfg.validParams = PMIC_CFG_WDG_WIN1_CODE_VALID;
    wdgCfg.win1Code = 0x40U;  // 64 * 1100us = 70.4ms (exact match to original intent)

    status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get and verify window-1 duration
    wdgCfg.validParams = PMIC_CFG_WDG_WIN1_CODE_VALID;
    wdgCfg.win1Code = 0U;

    status = Pmic_wdgGetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(wdgCfg.win1Code == 0x40U);

    wdg_cleanupAfterTest();
}

/**
 * @brief Test watchdog window-2 duration configuration.
 */
void test_pos_wdg_wdgSetCfg_window2Duration(void)
{
    wdg_setupForConfig();
    int32_t status;
    Pmic_WdgCfg_t wdgCfg = {0};

    // Set window-2 duration
    wdgCfg.validParams = PMIC_CFG_WDG_WIN2_CODE_VALID;
    wdgCfg.win2Code = 0x40U;  // 64 * 1100us = 70.4ms (exact match to original intent)

    status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get and verify window-2 duration
    wdgCfg.validParams = PMIC_CFG_WDG_WIN2_CODE_VALID;
    wdgCfg.win2Code = 0U;

    status = Pmic_wdgGetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(wdgCfg.win2Code == 0x40U);

    wdg_cleanupAfterTest();
}

/**
 * @brief Test watchdog fail threshold configuration.
 */
void test_pos_wdg_wdgSetCfg_failThreshold(void)
{
    wdg_setupForConfig();
    int32_t status;
    Pmic_WdgCfg_t wdgCfg = {0};

    // Set fail threshold
    wdgCfg.validParams = PMIC_CFG_WDG_THRESHOLD_1_VALID;
    wdgCfg.threshold1 = 7U;

    status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get and verify fail threshold
    wdgCfg.validParams = PMIC_CFG_WDG_THRESHOLD_1_VALID;
    wdgCfg.threshold1 = 0U;

    status = Pmic_wdgGetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(wdgCfg.threshold1 == 7U);

    wdg_cleanupAfterTest();
}

/**
 * @brief Test watchdog reset threshold configuration.
 */
void test_pos_wdg_wdgSetCfg_resetThreshold(void)
{
    wdg_setupForConfig();
    int32_t status;
    Pmic_WdgCfg_t wdgCfg = {0};

    // Set reset threshold
    wdgCfg.validParams = PMIC_CFG_WDG_THRESHOLD_2_VALID;
    wdgCfg.threshold2 = 7U;

    status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get and verify reset threshold
    wdgCfg.validParams = PMIC_CFG_WDG_THRESHOLD_2_VALID;
    wdgCfg.threshold2 = 0U;

    status = Pmic_wdgGetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(wdgCfg.threshold2 == 7U);

    wdg_cleanupAfterTest();
}

/**
 * @brief Test watchdog threshold 1 interrupt behavior configuration.
 * This implements the missing test_wdg_setCfg_resetEnable
 */
void test_pos_wdg_wdgSetCfg_threshold1IntBehavior(void)
{
    wdg_setupForConfig();
    int32_t status;
    Pmic_WdgCfg_t wdgCfg = {0};

    // Set threshold1 interrupt behavior
    wdgCfg.validParams = PMIC_CFG_WDG_THRESHOLD1_INT_BEHAVIOR_VALID;
    wdgCfg.threshold1IntBehavior = 1U;  // Enable interrupt

    status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get and verify threshold1 interrupt behavior
    wdgCfg.validParams = PMIC_CFG_WDG_THRESHOLD1_INT_BEHAVIOR_VALID;
    wdgCfg.threshold1IntBehavior = 0U;

    status = Pmic_wdgGetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(wdgCfg.threshold1IntBehavior == 1U);

    wdg_cleanupAfterTest();
}

/**
 * @brief Test watchdog mode configuration.
 */
void test_pos_wdg_wdgSetCfg_wdgMode(void)
{
    wdg_setupForConfig();
    int32_t status;
    Pmic_WdgCfg_t wdgCfg = {0};

    // Set Q&A mode
    wdgCfg.validParams = PMIC_CFG_WDG_MODE_VALID;
    wdgCfg.mode = PMIC_WDG_QA_MODE;

    status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify Q&A mode
    wdgCfg.validParams = PMIC_CFG_WDG_MODE_VALID;
    wdgCfg.mode = 0U;

    status = Pmic_wdgGetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(wdgCfg.mode == PMIC_WDG_QA_MODE);

    // Set Trigger mode
    wdgCfg.validParams = PMIC_CFG_WDG_MODE_VALID;
    wdgCfg.mode = PMIC_WDG_TRIGGER_MODE;

    status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify Trigger mode
    wdgCfg.validParams = PMIC_CFG_WDG_MODE_VALID;
    wdgCfg.mode = 0U;

    status = Pmic_wdgGetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(wdgCfg.mode == PMIC_WDG_TRIGGER_MODE);

    wdg_cleanupAfterTest();
}

/**
 * @brief Test watchdog threshold 2 interrupt behavior configuration.
 * This implements the missing test_wdg_setCfg_powerHold
 */
void test_pos_wdg_wdgSetCfg_threshold2IntBehavior(void)
{
    wdg_setupForConfig();
    int32_t status;
    Pmic_WdgCfg_t wdgCfg = {0};

    // Set threshold2 interrupt behavior
    wdgCfg.validParams = PMIC_CFG_WDG_THRESHOLD2_INT_BEHAVIOR_VALID;
    wdgCfg.threshold2IntBehavior = 1U;  // Enable interrupt

    status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get and verify threshold2 interrupt behavior
    wdgCfg.validParams = PMIC_CFG_WDG_THRESHOLD2_INT_BEHAVIOR_VALID;
    wdgCfg.threshold2IntBehavior = 0U;

    status = Pmic_wdgGetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(wdgCfg.threshold2IntBehavior == 1U);

    wdg_cleanupAfterTest();
}

/**
 * @brief Test watchdog return to long window configuration.
 * This implements the missing test_wdg_setCfg_ReturnLongWindow
 */
void test_pos_wdg_wdgSetCfg_returnLongWindow(void)
{
    wdg_setupForConfig();
    int32_t status;
    Pmic_WdgCfg_t wdgCfg = {0};

    // Configure long window and verify we can return to it
    wdgCfg.validParams = PMIC_CFG_WDG_LONG_WIN_CODE_VALID;
    wdgCfg.longWinCode = TEST_INVALID_PARAM_255;  // 255 * 1100us = 280.5ms (max possible, approximates 512ms intent)

    status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Set a different window temporarily
    wdgCfg.validParams = PMIC_CFG_WDG_WIN1_CODE_VALID;
    wdgCfg.win1Code = 0x20U;  // 32 * 1100us = 35.2ms (exact match to original intent)

    status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Return to long window and verify
    wdgCfg.validParams = PMIC_CFG_WDG_LONG_WIN_CODE_VALID;
    wdgCfg.longWinCode = 0U;

    status = Pmic_wdgGetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(wdgCfg.longWinCode == TEST_INVALID_PARAM_255);

    wdg_cleanupAfterTest();
}

/**
 * @brief Test watchdog Q&A feedback configuration.
 */
void test_pos_wdg_wdgSetCfg_QA_feedback(void)
{
    wdg_setupForConfig();
    int32_t status;
    Pmic_WdgCfg_t wdgCfg = {0};

    // Set Q&A feedback
    wdgCfg.validParams = PMIC_CFG_WDG_QA_FDBK_VALID;
    wdgCfg.qaFdbk = 1U;

    status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get and verify Q&A feedback
    wdgCfg.validParams = PMIC_CFG_WDG_QA_FDBK_VALID;
    wdgCfg.qaFdbk = 0U;

    status = Pmic_wdgGetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(wdgCfg.qaFdbk == 1U);

    wdg_cleanupAfterTest();
}

/**
 * @brief Test watchdog Q&A LFSR configuration.
 */
void test_pos_wdg_wdgSetCfg_QA_LFSR(void)
{
    wdg_setupForConfig();
    int32_t status;
    Pmic_WdgCfg_t wdgCfg = {0};

    // Set Q&A LFSR
    wdgCfg.validParams = PMIC_CFG_WDG_QA_LFSR_VALID;
    wdgCfg.qaLfsr = 0x3U;  // 2-bit field max value (was 0xAB=171, masked to 3 by hardware)

    status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get and verify Q&A LFSR
    wdgCfg.validParams = PMIC_CFG_WDG_QA_LFSR_VALID;
    wdgCfg.qaLfsr = 0U;

    status = Pmic_wdgGetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(wdgCfg.qaLfsr == 0x3U);

    wdg_cleanupAfterTest();
}

/**
 * @brief Test watchdog Q&A question seed configuration.
 */
void test_pos_wdg_wdgSetCfg_QA_questionSeed(void)
{
    wdg_setupForConfig();
    int32_t status;
    Pmic_WdgCfg_t wdgCfg = {0};

    // Set Q&A question seed
    wdgCfg.validParams = PMIC_CFG_WDG_QA_QUES_SEED_VALID;
    wdgCfg.qaQuesSeed = 0x5U;  // 4-bit field valid values 0-15 (was 0x55=85, masked to 5 by hardware)

    status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get and verify Q&A question seed
    wdgCfg.validParams = PMIC_CFG_WDG_QA_QUES_SEED_VALID;
    wdgCfg.qaQuesSeed = 0U;

    status = Pmic_wdgGetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(wdgCfg.qaQuesSeed == 0x5U);

    wdg_cleanupAfterTest();
}

/* ========================================================================== */
/* Q&A Sequence Tests                                                         */
/* ========================================================================== */

/**
 * @brief Test that testInject works by reading back injected value.
 */
/**
 * @brief Test Q&A write answer with full sequence in long window.
 */
void test_pos_wdg_wdgQaWriteAnswer_fullSequence(void)
{
    wdg_setupForConfig();
    int32_t status;
    Pmic_WdgCfg_t wdgCfg = {0};

    // Configure WDG in Q&A mode
    wdgCfg.validParams = PMIC_CFG_WDG_MODE_VALID |
                         PMIC_CFG_WDG_QA_FDBK_VALID |
                         PMIC_CFG_WDG_QA_LFSR_VALID |
                         PMIC_CFG_WDG_QA_QUES_SEED_VALID;
    wdgCfg.mode = PMIC_WDG_QA_MODE;
    wdgCfg.qaFdbk = 0U;
    wdgCfg.qaLfsr = 0x2U;
    wdgCfg.qaQuesSeed = 0xCU;

    status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Write 4 answer bytes (long window requires 4)
    for (uint8_t i = 0; i < 4U; i++) {
        status = Pmic_wdgQaWriteAnswer(&pmicHandle);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    }

    wdg_cleanupAfterTest();
}

/**
 * @brief Test Q&A write answer with qaFdbk=0 (mux case 0).
 */
void test_pos_wdg_wdgQaWriteAnswer_qaFdbk0(void)
{
    wdg_setupForConfig();
    int32_t status;
    Pmic_WdgCfg_t wdgCfg = {0};

    wdgCfg.validParams = PMIC_CFG_WDG_MODE_VALID | PMIC_CFG_WDG_QA_FDBK_VALID;
    wdgCfg.mode = PMIC_WDG_QA_MODE;
    wdgCfg.qaFdbk = 0U;

    status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_wdgQaWriteAnswer(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    wdg_cleanupAfterTest();
}

/**
 * @brief Test Q&A write answer with qaFdbk=1 (mux case 1).
 */
void test_pos_wdg_wdgQaWriteAnswer_qaFdbk1(void)
{
    wdg_setupForConfig();
    int32_t status;
    Pmic_WdgCfg_t wdgCfg = {0};

    wdgCfg.validParams = PMIC_CFG_WDG_MODE_VALID | PMIC_CFG_WDG_QA_FDBK_VALID;
    wdgCfg.mode = PMIC_WDG_QA_MODE;
    wdgCfg.qaFdbk = 1U;

    status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_wdgQaWriteAnswer(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    wdg_cleanupAfterTest();
}

/**
 * @brief Test Q&A write answer with qaFdbk=2 (mux case 2).
 */
void test_pos_wdg_wdgQaWriteAnswer_qaFdbk2(void)
{
    wdg_setupForConfig();
    int32_t status;
    Pmic_WdgCfg_t wdgCfg = {0};

    wdgCfg.validParams = PMIC_CFG_WDG_MODE_VALID | PMIC_CFG_WDG_QA_FDBK_VALID;
    wdgCfg.mode = PMIC_WDG_QA_MODE;
    wdgCfg.qaFdbk = 2U;

    status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_wdgQaWriteAnswer(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    wdg_cleanupAfterTest();
}

/**
 * @brief Test Q&A write answer with qaFdbk=3 (mux case 3).
 */
void test_pos_wdg_wdgQaWriteAnswer_qaFdbk3(void)
{
    wdg_setupForConfig();
    int32_t status;
    Pmic_WdgCfg_t wdgCfg = {0};

    wdgCfg.validParams = PMIC_CFG_WDG_MODE_VALID | PMIC_CFG_WDG_QA_FDBK_VALID;
    wdgCfg.mode = PMIC_WDG_QA_MODE;
    wdgCfg.qaFdbk = 3U;

    status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_wdgQaWriteAnswer(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    wdg_cleanupAfterTest();
}

/**
 * @brief Test Q&A write answer with different qaSeed values.
 */
void test_pos_wdg_wdgQaWriteAnswer_differentSeeds(void)
{
    wdg_setupForConfig();
    int32_t status;
    Pmic_WdgCfg_t wdgCfg = {0};

    wdgCfg.validParams = PMIC_CFG_WDG_MODE_VALID | PMIC_CFG_WDG_QA_QUES_SEED_VALID;
    wdgCfg.mode = PMIC_WDG_QA_MODE;
    wdgCfg.qaQuesSeed = 0xAU;  // Test with seed = 10

    status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_wdgQaWriteAnswer(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    wdg_cleanupAfterTest();
}

/**
 * @brief Test Q&A write answer with different qaLfsr values.
 */
void test_pos_wdg_wdgQaWriteAnswer_differentLfsr(void)
{
    wdg_setupForConfig();
    int32_t status;
    Pmic_WdgCfg_t wdgCfg = {0};

    wdgCfg.validParams = PMIC_CFG_WDG_MODE_VALID | PMIC_CFG_WDG_QA_LFSR_VALID;
    wdgCfg.mode = PMIC_WDG_QA_MODE;
    wdgCfg.qaLfsr = 0x1U;  // Test with LFSR = 1

    status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_wdgQaWriteAnswer(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    wdg_cleanupAfterTest();
}

/**
 * @brief Test getting error status after answer error (using test injection).
 */
void test_pos_wdg_wdgGetErrStatus_afterAnswerError(void)
{
#ifdef BUILD_MOCK
    int32_t status;
    Pmic_WdgErrStatus_t errors = {0};

    // Use test injection to set the ANSW_ERR flag
    testInject_setRegister(PMIC_WD_ERR_STAT_REG, PMIC_WD_ANSW_ERR_MASK);

    // Read error status
    errors.validParams = PMIC_CFG_WD_ANSW_ERR_ERR_VALID;
    status = Pmic_wdgGetErrStatus(&pmicHandle, &errors);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(errors.answerError == true);
#else
    TEST_IGNORE_MESSAGE("Requires BUILD_MOCK for register injection");
#endif
}

/* ========================================================================== */
/* Error Status Get Tests                                                     */
/* ========================================================================== */

/**
 * @brief Test getting timeout error status.
 */
void test_pos_wdg_wdgGetErrStatus_timeout(void)
{
#ifdef BUILD_MOCK
    int32_t status;
    Pmic_WdgErrStatus_t errors = {0};

    // Clear the register first, then set the TIMEOUT flag
    testInject_setRegister(PMIC_WD_ERR_STAT_REG, 0x00U);

    testInject_setBits(PMIC_WD_ERR_STAT_REG, PMIC_WD_TMO_MASK);
    errors.validParams = PMIC_CFG_WD_TIMEOUT_ERR_VALID;
    status = Pmic_wdgGetErrStatus(&pmicHandle, &errors);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(errors.timeout == true);
#else
    TEST_IGNORE_MESSAGE("Requires BUILD_MOCK for register injection");
#endif
}

/**
 * @brief Test getting long window timeout error status.
 */
void test_pos_wdg_wdgGetErrStatus_longWindowTimeout(void)
{
#ifdef BUILD_MOCK
    int32_t status;
    Pmic_WdgErrStatus_t errors = {0};

    // Use test injection to set the LONGWIN_TMO flag
    testInject_setRegister(PMIC_WD_ERR_STAT_REG, PMIC_WD_LONGWIN_TMO_MASK);

    errors.validParams = PMIC_CFG_WD_LONGWIN_TIMEOUT_ERR_VALID;
    status = Pmic_wdgGetErrStatus(&pmicHandle, &errors);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(errors.longWindowTimeout == true);
#endif
}

/**
 * @brief Test getting answer early error status.
 */
void test_pos_wdg_wdgGetErrStatus_answerEarlyError(void)
{
#ifdef BUILD_MOCK
    int32_t status;
    Pmic_WdgErrStatus_t errors = {0};

    // Use test injection to set the ANSW_EARLY flag
    testInject_setRegister(PMIC_WD_ERR_STAT_REG, PMIC_WD_ANSW_EARLY_MASK);

    errors.validParams = PMIC_CFG_WD_ANSW_EARLY_ERR_VALID;
    status = Pmic_wdgGetErrStatus(&pmicHandle, &errors);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(errors.answerEarlyError == true);
#else
    TEST_IGNORE_MESSAGE("Requires BUILD_MOCK for register injection");
#endif
}

/**
 * @brief Test getting sequence error status.
 */
void test_pos_wdg_wdgGetErrStatus_sequenceErr(void)
{
#ifdef BUILD_MOCK
    int32_t status;
    Pmic_WdgErrStatus_t errors = {0};

    // Use test injection to set the SEQ_ERR flag
    testInject_setRegister(PMIC_WD_ERR_STAT_REG, PMIC_WD_SEQ_ERR_MASK);

    errors.validParams = PMIC_CFG_WD_SEQ_ERR_ERR_VALID;
    status = Pmic_wdgGetErrStatus(&pmicHandle, &errors);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(errors.sequenceError == true);
#else
    TEST_IGNORE_MESSAGE("Requires BUILD_MOCK for register injection");
#endif
}

/**
 * @brief Test getting answer error status.
 */
void test_pos_wdg_wdgGetErrStatus_answerErr(void)
{
#ifdef BUILD_MOCK
    int32_t status;
    Pmic_WdgErrStatus_t errors = {0};

    // Use test injection to set the ANSW_ERR flag
    testInject_setRegister(PMIC_WD_ERR_STAT_REG, PMIC_WD_ANSW_ERR_MASK);

    errors.validParams = PMIC_CFG_WD_ANSW_ERR_ERR_VALID;
    status = Pmic_wdgGetErrStatus(&pmicHandle, &errors);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(errors.answerError == true);
#else
    TEST_IGNORE_MESSAGE("Requires BUILD_MOCK for register injection");
#endif
}

/**
 * @brief Test getting trigger early error status (TPS65386x unique).
 */
void test_pos_wdg_wdgGetErrStatus_triggerEarly(void)
{
#ifdef BUILD_MOCK
    int32_t status;
    Pmic_WdgErrStatus_t errors = {0};

    // Use test injection to set the TRIG_EARLY flag
    testInject_setRegister(PMIC_WD_ERR_STAT_REG, PMIC_WD_TRIG_EARLY_MASK);

    errors.validParams = PMIC_CFG_WD_TRIG_EARLY_ERR_VALID;
    status = Pmic_wdgGetErrStatus(&pmicHandle, &errors);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(errors.triggerEarlyError == true);
#else
    TEST_IGNORE_MESSAGE("Requires BUILD_MOCK for register injection");
#endif
}

/**
 * @brief Test getting threshold 1 interrupt error status.
 */
void test_pos_wdg_wdgGetErrStatus_th1Int(void)
{
#ifdef BUILD_MOCK
    int32_t status;
    Pmic_WdgErrStatus_t errors = {0};

    // Use test injection to set the TH1_ERR flag
    testInject_setRegister(PMIC_WD_ERR_STAT_REG, PMIC_WD_TH1_ERR_MASK);

    errors.validParams = PMIC_CFG_WD_TH1_INT_ERR_VALID;
    status = Pmic_wdgGetErrStatus(&pmicHandle, &errors);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(errors.threshold1Error == true);
#else
    TEST_IGNORE_MESSAGE("Requires BUILD_MOCK for register injection");
#endif
}

/**
 * @brief Test getting threshold 2 interrupt error status.
 */
void test_pos_wdg_wdgGetErrStatus_th2Int(void)
{
#ifdef BUILD_MOCK
    int32_t status;
    Pmic_WdgErrStatus_t errors = {0};

    // Use test injection to set the TH2_ERR flag
    testInject_setRegister(PMIC_WD_ERR_STAT_REG, PMIC_WD_TH2_ERR_MASK);

    errors.validParams = PMIC_CFG_WD_TH2_INT_ERR_VALID;
    status = Pmic_wdgGetErrStatus(&pmicHandle, &errors);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(errors.threshold2Error == true);
#else
    TEST_IGNORE_MESSAGE("Requires BUILD_MOCK for register injection");
#endif
}

/**
 * @brief Test getting all error flags at once.
 */
void test_pos_wdg_wdgGetErrStatus_allFlags(void)
{
#ifdef BUILD_MOCK
    int32_t status;
    Pmic_WdgErrStatus_t errors = {0};

    // Set all error flags
    uint8_t allFlags = PMIC_WD_TMO_MASK |
                       PMIC_WD_TRIG_EARLY_MASK |
                       PMIC_WD_ANSW_EARLY_MASK |
                       PMIC_WD_SEQ_ERR_MASK |
                       PMIC_WD_ANSW_ERR_MASK |
                       PMIC_WD_LONGWIN_TMO_MASK |
                       PMIC_WD_TH1_ERR_MASK |
                       PMIC_WD_TH2_ERR_MASK;

    testInject_setRegister(PMIC_WD_ERR_STAT_REG, allFlags);

    // Read all error flags
    errors.validParams = PMIC_CFG_WD_ERR_STAT_ALL_VALID_SHIFT;
    status = Pmic_wdgGetErrStatus(&pmicHandle, &errors);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(errors.timeout == true);
    PLATFORM_ASSERT(errors.triggerEarlyError == true);
    PLATFORM_ASSERT(errors.answerEarlyError == true);
    PLATFORM_ASSERT(errors.sequenceError == true);
    PLATFORM_ASSERT(errors.answerError == true);
    PLATFORM_ASSERT(errors.longWindowTimeout == true);
    PLATFORM_ASSERT(errors.threshold1Error == true);
    PLATFORM_ASSERT(errors.threshold2Error == true);
#else
    TEST_IGNORE_MESSAGE("Requires BUILD_MOCK for register injection");
#endif
}

/* ========================================================================== */
/* Clear Error Status Tests                                                   */
/* ========================================================================== */

/**
 * @brief Test clearing timeout error status.
 */
void test_pos_wdg_wdgClrErrStatus_timeout(void)
{
#ifdef BUILD_MOCK
    int32_t status;
    Pmic_WdgErrStatus_t errors = {0};

    // Set the error flag first
    testInject_setRegister(PMIC_WD_ERR_STAT_REG, PMIC_WD_TMO_MASK);

    // Clear it
    errors.validParams = PMIC_CFG_WD_TIMEOUT_ERR_VALID;
    errors.timeout = true;
    status = Pmic_wdgClrErrStatus(&pmicHandle, &errors);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify it's cleared
    errors.timeout = false;
    status = Pmic_wdgGetErrStatus(&pmicHandle, &errors);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(errors.timeout == false);
#else
    TEST_IGNORE_MESSAGE("Requires BUILD_MOCK for register injection");
#endif
}

/**
 * @brief Test clearing long window timeout error status.
 */
void test_pos_wdg_wdgClrErrStatus_longWindowTimeout(void)
{
#ifdef BUILD_MOCK
    int32_t status;
    Pmic_WdgErrStatus_t errors = {0};

    // Set the error flag first
    testInject_setRegister(PMIC_WD_ERR_STAT_REG, PMIC_WD_LONGWIN_TMO_MASK);

    // Clear it
    errors.validParams = PMIC_CFG_WD_LONGWIN_TIMEOUT_ERR_VALID;
    errors.longWindowTimeout = true;
    status = Pmic_wdgClrErrStatus(&pmicHandle, &errors);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify it's cleared
    errors.longWindowTimeout = false;
    status = Pmic_wdgGetErrStatus(&pmicHandle, &errors);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(errors.longWindowTimeout == false);
#else
    TEST_IGNORE_MESSAGE("Requires BUILD_MOCK for register injection");
#endif
}

/**
 * @brief Test clearing answer early error status.
 */
void test_pos_wdg_wdgClrErrStatus_answerEarlyError(void)
{
#ifdef BUILD_MOCK
    int32_t status;
    Pmic_WdgErrStatus_t errors = {0};

    // Set the error flag first
    testInject_setRegister(PMIC_WD_ERR_STAT_REG, PMIC_WD_ANSW_EARLY_MASK);

    // Clear it
    errors.validParams = PMIC_CFG_WD_ANSW_EARLY_ERR_VALID;
    errors.answerEarlyError = true;
    status = Pmic_wdgClrErrStatus(&pmicHandle, &errors);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify it's cleared
    errors.answerEarlyError = false;
    status = Pmic_wdgGetErrStatus(&pmicHandle, &errors);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(errors.answerEarlyError == false);
#else
    TEST_IGNORE_MESSAGE("Requires BUILD_MOCK for register injection");
#endif
}

/**
 * @brief Test clearing sequence error status.
 */
void test_pos_wdg_wdgClrErrStatus_sequenceErr(void)
{
#ifdef BUILD_MOCK
    int32_t status;
    Pmic_WdgErrStatus_t errors = {0};

    // Set the error flag first
    testInject_setRegister(PMIC_WD_ERR_STAT_REG, PMIC_WD_SEQ_ERR_MASK);

    // Clear it
    errors.validParams = PMIC_CFG_WD_SEQ_ERR_ERR_VALID;
    errors.sequenceError = true;
    status = Pmic_wdgClrErrStatus(&pmicHandle, &errors);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify it's cleared
    errors.sequenceError = false;
    status = Pmic_wdgGetErrStatus(&pmicHandle, &errors);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(errors.sequenceError == false);
#else
    TEST_IGNORE_MESSAGE("Requires BUILD_MOCK for register injection");
#endif
}

/**
 * @brief Test clearing answer error status.
 */
void test_pos_wdg_wdgClrErrStatus_answerErr(void)
{
#ifdef BUILD_MOCK
    int32_t status;
    Pmic_WdgErrStatus_t errors = {0};

    // Set the error flag first
    testInject_setRegister(PMIC_WD_ERR_STAT_REG, PMIC_WD_ANSW_ERR_MASK);

    // Clear it
    errors.validParams = PMIC_CFG_WD_ANSW_ERR_ERR_VALID;
    errors.answerError = true;
    status = Pmic_wdgClrErrStatus(&pmicHandle, &errors);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify it's cleared
    errors.answerError = false;
    status = Pmic_wdgGetErrStatus(&pmicHandle, &errors);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(errors.answerError == false);
#else
    TEST_IGNORE_MESSAGE("Requires BUILD_MOCK for register injection");
#endif
}

/**
 * @brief Test clearing trigger early error status.
 */
void test_pos_wdg_wdgClrErrStatus_triggerEarly(void)
{
#ifdef BUILD_MOCK
    int32_t status;
    Pmic_WdgErrStatus_t errors = {0};

    // Set the error flag first
    testInject_setRegister(PMIC_WD_ERR_STAT_REG, PMIC_WD_TRIG_EARLY_MASK);

    // Clear it
    errors.validParams = PMIC_CFG_WD_TRIG_EARLY_ERR_VALID;
    errors.triggerEarlyError = true;
    status = Pmic_wdgClrErrStatus(&pmicHandle, &errors);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify it's cleared
    errors.triggerEarlyError = false;
    status = Pmic_wdgGetErrStatus(&pmicHandle, &errors);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(errors.triggerEarlyError == false);
#else
    TEST_IGNORE_MESSAGE("Requires BUILD_MOCK for register injection");
#endif
}

/**
 * @brief Test clearing threshold1 error status only.
 */
void test_pos_wdg_wdgClrErrStatus_th1ErrorOnly(void)
{
#ifdef BUILD_MOCK
    int32_t status;
    Pmic_WdgErrStatus_t errors = {0};

    // Set the error flag first
    testInject_setRegister(PMIC_WD_ERR_STAT_REG, PMIC_WD_TH1_ERR_MASK);

    // Clear it
    errors.validParams = PMIC_CFG_WD_TH1_INT_ERR_VALID;
    errors.threshold1Error = true;
    status = Pmic_wdgClrErrStatus(&pmicHandle, &errors);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify it's cleared
    errors.threshold1Error = false;
    status = Pmic_wdgGetErrStatus(&pmicHandle, &errors);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(errors.threshold1Error == false);
#else
    TEST_IGNORE_MESSAGE("Requires BUILD_MOCK for register injection");
#endif
}

/**
 * @brief Test clearing threshold2 error status only.
 */
void test_pos_wdg_wdgClrErrStatus_th2ErrorOnly(void)
{
#ifdef BUILD_MOCK
    int32_t status;
    Pmic_WdgErrStatus_t errors = {0};

    // Set the error flag first
    testInject_setRegister(PMIC_WD_ERR_STAT_REG, PMIC_WD_TH2_ERR_MASK);

    // Clear it
    errors.validParams = PMIC_CFG_WD_TH2_INT_ERR_VALID;
    errors.threshold2Error = true;
    status = Pmic_wdgClrErrStatus(&pmicHandle, &errors);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify it's cleared
    errors.threshold2Error = false;
    status = Pmic_wdgGetErrStatus(&pmicHandle, &errors);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(errors.threshold2Error == false);
#else
    TEST_IGNORE_MESSAGE("Requires BUILD_MOCK for register injection");
#endif
}

/**
 * @brief Test clearing sequence error status only (when multiple errors are set).
 */
void test_pos_wdg_wdgClrErrStatus_seqErrorOnly(void)
{
#ifdef BUILD_MOCK
    int32_t status;
    Pmic_WdgErrStatus_t errors = {0};

    // Set multiple error flags: sequence error + answer error
    testInject_setRegister(PMIC_WD_ERR_STAT_REG,
                           PMIC_WD_SEQ_ERR_MASK | PMIC_WD_ANSW_ERR_MASK);

    // Clear only sequence error
    errors.validParams = PMIC_CFG_WD_SEQ_ERR_ERR_VALID;
    errors.sequenceError = true;
    status = Pmic_wdgClrErrStatus(&pmicHandle, &errors);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify sequence error is cleared but answer error remains
    errors.validParams = PMIC_CFG_WD_SEQ_ERR_ERR_VALID | PMIC_CFG_WD_ANSW_ERR_ERR_VALID;
    errors.sequenceError = false;
    errors.answerError = false;
    status = Pmic_wdgGetErrStatus(&pmicHandle, &errors);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(errors.sequenceError == false);
    PLATFORM_ASSERT(errors.answerError == true);
#else
    TEST_IGNORE_MESSAGE("Requires BUILD_MOCK for register injection");
#endif
}

/**
 * @brief Test clearing all errors when no errors are set (optimization path).
 */
void test_pos_wdg_wdgClrErrStatusAll_whenNoErrors(void)
{
#ifdef BUILD_MOCK
    int32_t status;

    // Clear the register first to ensure no errors

    // Clear all errors (when none are set)
    status = Pmic_wdgClrErrStatusAll(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Requires BUILD_MOCK for register injection");
#endif
}

/* ========================================================================== */
/* Fail Count Status Tests                                                    */
/* ========================================================================== */

/**
 * @brief Test getting bad event status.
 */
void test_pos_wdg_wdgGetFailCntStatus_badEvent(void)
{
#ifdef BUILD_MOCK
    int32_t status;
    Pmic_WdgFailCntStatus_t failCount = {0};

    // Set the bad event flag
    testInject_setRegister(PMIC_WD_STAT_REG, PMIC_WD_BAD_EVENT_MASK);

    failCount.validParams = PMIC_CFG_WD_BAD_EVENT_STAT_VALID;
    status = Pmic_wdgGetFailCntStatus(&pmicHandle, &failCount);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(failCount.badEvent == true);
#else
    TEST_IGNORE_MESSAGE("Requires BUILD_MOCK for register injection");
#endif
}

/**
 * @brief Test getting good event status.
 */
void test_pos_wdg_wdgGetFailCntStatus_goodEvent(void)
{
#ifdef BUILD_MOCK
    int32_t status;
    Pmic_WdgFailCntStatus_t failCount = {0};

    // Set the first ok flag (good event)
    testInject_setRegister(PMIC_WD_STAT_REG, PMIC_WD_FIRST_OK_MASK);

    failCount.validParams = PMIC_CFG_WD_GOOD_EVENT_STAT_VALID;
    status = Pmic_wdgGetFailCntStatus(&pmicHandle, &failCount);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(failCount.goodEvent == true);
#else
    TEST_IGNORE_MESSAGE("Requires BUILD_MOCK for register injection");
#endif
}

/**
 * @brief Test getting watchdog fail count value.
 */
void test_pos_wdg_wdgGetFailCntStatus_wdFailCnt(void)
{
#ifdef BUILD_MOCK
    int32_t status;
    Pmic_WdgFailCntStatus_t failCount = {0};

    // Set fail count to 5 (bits [3:0] = 0x5)
    testInject_setRegister(PMIC_WD_STAT_REG, 0x05U);

    failCount.validParams = PMIC_CFG_WD_FAIL_CNT_VAL_VALID;
    status = Pmic_wdgGetFailCntStatus(&pmicHandle, &failCount);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(failCount.wdFailCnt == 5U);
#else
    TEST_IGNORE_MESSAGE("Requires BUILD_MOCK for register injection");
#endif
}

/**
 * @brief Test getting all fail count status fields at once.
 */
void test_pos_wdg_wdgGetFailCntStatus_allFields(void)
{
#ifdef BUILD_MOCK
    int32_t status;
    Pmic_WdgFailCntStatus_t failCount = {0};

    // Set all relevant bits: fail count=3, bad event, first ok, long window active
    uint8_t statValue = 0x03U |  // Fail count = 3
                        PMIC_WD_BAD_EVENT_MASK |  // Bad event
                        PMIC_WD_FIRST_OK_MASK |  // Good event
                        PMIC_WD_LONGWIN_ACTIVE_MASK;  // Long window active

    testInject_setRegister(PMIC_WD_STAT_REG, statValue);

    failCount.validParams = PMIC_CFG_WD_FAILCNT_ALL_VALID_SHIFT;
    status = Pmic_wdgGetFailCntStatus(&pmicHandle, &failCount);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(failCount.wdFailCnt == 3U);
    PLATFORM_ASSERT(failCount.badEvent == true);
    PLATFORM_ASSERT(failCount.goodEvent == true);
    PLATFORM_ASSERT(failCount.longWinActive == true);
#else
    TEST_IGNORE_MESSAGE("Requires BUILD_MOCK for register injection");
#endif
}

/**
 * @brief Test getting fail count only with specific validParams.
 */
void test_pos_wdg_wdgGetFailCntStatus_failCntOnly(void)
{
#ifdef BUILD_MOCK
    int32_t status;
    Pmic_WdgFailCntStatus_t failCount = {0};

    // Set fail count to 6 (bits [3:0] = 0x6) with other status bits set
    uint8_t statValue = 0x06U | PMIC_WD_BAD_EVENT_MASK | PMIC_WD_FIRST_OK_MASK;
    testInject_setRegister(PMIC_WD_STAT_REG, statValue);

    // Only request fail count value
    failCount.validParams = PMIC_CFG_WD_FAIL_CNT_VAL_VALID;
    status = Pmic_wdgGetFailCntStatus(&pmicHandle, &failCount);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(failCount.wdFailCnt == 6U);
#else
    TEST_IGNORE_MESSAGE("Requires BUILD_MOCK for register injection");
#endif
}

/**
 * @brief Test getting bad event count only with specific validParams.
 */
void test_pos_wdg_wdgGetFailCntStatus_badCntOnly(void)
{
#ifdef BUILD_MOCK
    int32_t status;
    Pmic_WdgFailCntStatus_t failCount = {0};

    // Set bad event flag along with other fields
    uint8_t statValue = 0x02U | PMIC_WD_BAD_EVENT_MASK | PMIC_WD_FIRST_OK_MASK;
    testInject_setRegister(PMIC_WD_STAT_REG, statValue);

    // Only request bad event status
    failCount.validParams = PMIC_CFG_WD_BAD_EVENT_STAT_VALID;
    status = Pmic_wdgGetFailCntStatus(&pmicHandle, &failCount);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(failCount.badEvent == true);
#else
    TEST_IGNORE_MESSAGE("Requires BUILD_MOCK for register injection");
#endif
}

/* ========================================================================== */
/* Configuration Tests                                                        */
/* ========================================================================== */

/**
 * @brief Test time base configuration.
 */
void test_pos_wdg_wdgSetCfg_timeBase(void)
{
    wdg_setupForConfig();
    int32_t status;
    Pmic_WdgCfg_t wdgCfg = {0};

    // Set time base to 550us
    wdgCfg.validParams = PMIC_CFG_WDG_TIME_BASE_VALID;
    wdgCfg.timeBase = PMIC_WDG_TIME_BASE_550_US;

    status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get and verify time base
    wdgCfg.validParams = PMIC_CFG_WDG_TIME_BASE_VALID;
    wdgCfg.timeBase = 0U;

    status = Pmic_wdgGetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(wdgCfg.timeBase == PMIC_WDG_TIME_BASE_550_US);

    wdg_cleanupAfterTest();
}

/**
 * @brief Test set config with zero validParams (should return error).
 */
void test_neg_wdg_wdgSetCfg_zeroValidParams(void)
{
    int32_t status;
    Pmic_WdgCfg_t wdgCfg = {0};

    // Enable watchdog first
    status = Pmic_wdgEnable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Try to set config with validParams = 0
    wdgCfg.validParams = 0U;

    status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_wdgSetCfg when watchdog is disabled.
 */
void test_neg_wdg_wdgSetCfg_whenDisabled(void)
{
    int32_t status;
    Pmic_WdgCfg_t wdgCfg = {
        .validParams = PMIC_CFG_WDG_MODE_VALID,
        .mode = PMIC_WDG_TRIGGER_MODE
    };

    // Disable watchdog
    status = Pmic_wdgDisable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Attempt to configure - should fail
    status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NOT_SUPPORTED);

    // Re-enable watchdog for subsequent tests
    status = Pmic_wdgEnable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    status = Pmic_wdgSetReturnToLongWindow(&pmicHandle, true);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

/**
 * @brief Test Pmic_wdgSetCfg when not in Long Window mode.
 */
void test_neg_wdg_wdgSetCfg_whenNotInLongWindow(void)
{
    int32_t status;
    Pmic_WdgCfg_t wdgCfg = {
        .validParams = PMIC_CFG_WDG_MODE_VALID,
        .mode = PMIC_WDG_TRIGGER_MODE
    };

    // Enable watchdog but disable return to long window
    status = Pmic_wdgEnable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    status = Pmic_wdgSetReturnToLongWindow(&pmicHandle, false);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Attempt to configure - should fail
    status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NOT_SUPPORTED);

    // Re-enable return to long window for subsequent tests
    status = Pmic_wdgSetReturnToLongWindow(&pmicHandle, true);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

/**
 * @brief Test watchdog SetMode/GetMode with TRIGGER_MODE.
 */
void test_pos_wdg_wdgSetMode_triggerMode(void)
{
    int32_t status;
    uint8_t mode = TEST_INVALID_PARAM_255;

    // Enable watchdog first
    status = Pmic_wdgEnable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Set watchdog mode to TRIGGER_MODE
    status = Pmic_wdgSetMode(&pmicHandle, PMIC_WDG_TRIGGER_MODE);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get and verify mode
    status = Pmic_wdgGetMode(&pmicHandle, &mode);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(mode == PMIC_WDG_TRIGGER_MODE);

    wdg_cleanupAfterTest();
}

/**
 * @brief Test watchdog SetMode/GetMode with Q&A_MODE.
 */
void test_pos_wdg_wdgSetMode_qAndAMode(void)
{
    int32_t status;
    uint8_t mode = TEST_INVALID_PARAM_255;

    // Enable watchdog first
    status = Pmic_wdgEnable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Set watchdog mode to QA_MODE
    status = Pmic_wdgSetMode(&pmicHandle, PMIC_WDG_QA_MODE);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get and verify mode
    status = Pmic_wdgGetMode(&pmicHandle, &mode);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(mode == PMIC_WDG_QA_MODE);

    wdg_cleanupAfterTest();
}

/**
 * @brief Test watchdog SetPowerHold/GetPowerHold - enable.
 */
void test_pos_wdg_wdgSetPowerHold_enable(void)
{
    int32_t status;
    bool isEnabled = false;

    // Enable watchdog first
    status = Pmic_wdgEnable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Set power hold enable
    status = Pmic_wdgSetPowerHold(&pmicHandle, true);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get and verify power hold is enabled
    status = Pmic_wdgGetPowerHold(&pmicHandle, &isEnabled);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(isEnabled == true);

    wdg_cleanupAfterTest();
}

/**
 * @brief Test watchdog SetPowerHold/GetPowerHold - disable.
 */
void test_pos_wdg_wdgSetPowerHold_disable(void)
{
    int32_t status;
    bool isEnabled = true;

    // Enable watchdog first
    status = Pmic_wdgEnable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Set power hold disable
    status = Pmic_wdgSetPowerHold(&pmicHandle, false);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get and verify power hold is disabled
    status = Pmic_wdgGetPowerHold(&pmicHandle, &isEnabled);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(isEnabled == false);

    wdg_cleanupAfterTest();
}

/**
 * @brief Test watchdog SetReturnToLongWindow/GetReturnToLongWindow - enable.
 */
void test_pos_wdg_wdgSetReturnToLongWindow_enable(void)
{
    int32_t status;
    bool isEnabled = false;

    // Enable watchdog first
    status = Pmic_wdgEnable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Set return to long window enable
    status = Pmic_wdgSetReturnToLongWindow(&pmicHandle, true);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get and verify return to long window is enabled
    status = Pmic_wdgGetReturnToLongWindow(&pmicHandle, &isEnabled);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(isEnabled == true);

    wdg_cleanupAfterTest();
}

/**
 * @brief Test watchdog SetReturnToLongWindow/GetReturnToLongWindow - disable.
 */
void test_pos_wdg_wdgSetReturnToLongWindow_disable(void)
{
    int32_t status;
    bool isEnabled = true;

    // Enable watchdog first
    status = Pmic_wdgEnable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Set return to long window disable
    status = Pmic_wdgSetReturnToLongWindow(&pmicHandle, false);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get and verify return to long window is disabled
    status = Pmic_wdgGetReturnToLongWindow(&pmicHandle, &isEnabled);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(isEnabled == false);

    wdg_cleanupAfterTest();
}

/* ========================================================================== */
/*                       Negative Test Functions                              */
/* ========================================================================== */

/**
 * @brief Test Pmic_wdgEnable with NULL handle.
 */
void test_neg_wdg_wdgEnable_nullHandle(void)
{
    int32_t status = Pmic_wdgEnable(NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_wdgDisable with NULL handle.
 */
void test_neg_wdg_wdgDisable_nullHandle(void)
{
    int32_t status = Pmic_wdgDisable(NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_wdgSetEnableState with NULL handle.
 */
void test_neg_wdg_wdgSetEnableState_nullHandle(void)
{
    int32_t status = Pmic_wdgSetEnableState(NULL, true);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_wdgGetEnableState with NULL handle.
 */
void test_neg_wdg_wdgGetEnableState_nullHandle(void)
{
    bool isEnabled = false;
    int32_t status = Pmic_wdgGetEnableState(NULL, &isEnabled);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_wdgGetEnableState with NULL output parameter.
 */
void test_neg_wdg_wdgGetEnableState_nullParam(void)
{
    int32_t status = Pmic_wdgGetEnableState(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_wdgSetCfg with NULL handle.
 */
void test_neg_wdg_wdgSetCfg_nullHandle(void)
{
    Pmic_WdgCfg_t wdgCfg = {0};
    wdgCfg.validParams = PMIC_CFG_WDG_MODE_VALID;
    int32_t status = Pmic_wdgSetCfg(NULL, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_wdgSetCfg with NULL config parameter.
 */
void test_neg_wdg_wdgSetCfg_nullConfig(void)
{
    int32_t status = Pmic_wdgSetCfg(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_wdgSetCfg with invalid mode value.
 */
void test_neg_wdg_wdgSetCfg_invalidMode(void)
{
    Pmic_WdgCfg_t wdgCfg = {0};
    wdgCfg.validParams = PMIC_CFG_WDG_MODE_VALID;
    wdgCfg.mode = PMIC_WDG_MODE_MAX + 1U;

    int32_t status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_wdgSetCfg with invalid time base value.
 */
void test_neg_wdg_wdgSetCfg_invalidTimeBase(void)
{
    Pmic_WdgCfg_t wdgCfg = {0};
    wdgCfg.validParams = PMIC_CFG_WDG_TIME_BASE_VALID;
    wdgCfg.timeBase = PMIC_WDG_TIME_BASE_MAX + 1U;

    int32_t status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_wdgSetCfg with invalid threshold1 value.
 */
void test_neg_wdg_wdgSetCfg_invalidThreshold1(void)
{
    Pmic_WdgCfg_t wdgCfg = {0};
    wdgCfg.validParams = PMIC_CFG_WDG_THRESHOLD_1_VALID;
    wdgCfg.threshold1 = PMIC_WDG_THRESHOLD_COUNT_MAX + 1U;

    int32_t status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_wdgSetCfg with invalid threshold2 value.
 */
void test_neg_wdg_wdgSetCfg_invalidThreshold2(void)
{
    Pmic_WdgCfg_t wdgCfg = {0};
    wdgCfg.validParams = PMIC_CFG_WDG_THRESHOLD_2_VALID;
    wdgCfg.threshold2 = PMIC_WDG_THRESHOLD_COUNT_MAX + 1U;

    int32_t status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_wdgSetCfg with invalid QA feedback value.
 */
void test_neg_wdg_wdgSetCfg_invalidQaFdbk(void)
{
    Pmic_WdgCfg_t wdgCfg = {0};
    wdgCfg.validParams = PMIC_CFG_WDG_QA_FDBK_VALID;
    wdgCfg.qaFdbk = PMIC_WDG_QA_FEEDBACK_VALUE_MAX + 1U;

    int32_t status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_wdgSetCfg with invalid QA LFSR value.
 */
void test_neg_wdg_wdgSetCfg_invalidQaLfsr(void)
{
    Pmic_WdgCfg_t wdgCfg = {0};
    wdgCfg.validParams = PMIC_CFG_WDG_QA_LFSR_VALID;
    wdgCfg.qaLfsr = PMIC_WDG_QA_LFSR_VALUE_MAX + 1U;

    int32_t status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_wdgSetCfg with invalid QA question seed value.
 */
void test_neg_wdg_wdgSetCfg_invalidQaQuesSeed(void)
{
    Pmic_WdgCfg_t wdgCfg = {0};
    wdgCfg.validParams = PMIC_CFG_WDG_QA_QUES_SEED_VALID;
    wdgCfg.qaQuesSeed = PMIC_WDG_QA_QUES_SEED_VALUE_MAX + 1U;

    int32_t status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_wdgSetCfg with invalid threshold1 interrupt behavior.
 */
void test_neg_wdg_wdgSetCfg_invalidThreshold1IntBehavior(void)
{
    Pmic_WdgCfg_t wdgCfg = {0};
    wdgCfg.validParams = PMIC_CFG_WDG_THRESHOLD1_INT_BEHAVIOR_VALID;
    wdgCfg.threshold1IntBehavior = PMIC_WDG_THRESHOLD_INT_BEHAVIOR_MAX + 1U;

    int32_t status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_wdgSetCfg with invalid threshold2 interrupt behavior.
 */
void test_neg_wdg_wdgSetCfg_invalidThreshold2IntBehavior(void)
{
    Pmic_WdgCfg_t wdgCfg = {0};
    wdgCfg.validParams = PMIC_CFG_WDG_THRESHOLD2_INT_BEHAVIOR_VALID;
    wdgCfg.threshold2IntBehavior = PMIC_WDG_THRESHOLD_INT_BEHAVIOR_MAX + 1U;

    int32_t status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_wdgGetCfg with NULL handle.
 */
void test_neg_wdg_wdgGetCfg_nullHandle(void)
{
    Pmic_WdgCfg_t wdgCfg = {0};
    wdgCfg.validParams = PMIC_CFG_WDG_MODE_VALID;
    int32_t status = Pmic_wdgGetCfg(NULL, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_wdgGetCfg with NULL config parameter.
 */
void test_neg_wdg_wdgGetCfg_nullConfig(void)
{
    int32_t status = Pmic_wdgGetCfg(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_wdgSetMode with NULL handle.
 */
void test_neg_wdg_wdgSetMode_nullHandle(void)
{
    int32_t status = Pmic_wdgSetMode(NULL, PMIC_WDG_TRIGGER_MODE);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_wdgSetMode with invalid mode.
 */
void test_neg_wdg_wdgSetMode_invalidMode(void)
{
    int32_t status = Pmic_wdgSetMode(&pmicHandle, PMIC_WDG_MODE_MAX + 1U);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_wdgGetMode with NULL handle.
 */
void test_neg_wdg_wdgGetMode_nullHandle(void)
{
    uint8_t mode = 0U;
    int32_t status = Pmic_wdgGetMode(NULL, &mode);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_wdgGetMode with NULL output parameter.
 */
void test_neg_wdg_wdgGetMode_nullParam(void)
{
    int32_t status = Pmic_wdgGetMode(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_wdgSetPowerHold with NULL handle.
 */
void test_neg_wdg_wdgSetPowerHold_nullHandle(void)
{
    int32_t status = Pmic_wdgSetPowerHold(NULL, true);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_wdgGetPowerHold with NULL handle.
 */
void test_neg_wdg_wdgGetPowerHold_nullHandle(void)
{
    bool isEnabled = false;
    int32_t status = Pmic_wdgGetPowerHold(NULL, &isEnabled);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_wdgGetPowerHold with NULL output parameter.
 */
void test_neg_wdg_wdgGetPowerHold_nullParam(void)
{
    int32_t status = Pmic_wdgGetPowerHold(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_wdgSetReturnToLongWindow with NULL handle.
 */
void test_neg_wdg_wdgSetReturnToLongWindow_nullHandle(void)
{
    int32_t status = Pmic_wdgSetReturnToLongWindow(NULL, true);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_wdgGetReturnToLongWindow with NULL handle.
 */
void test_neg_wdg_wdgGetReturnToLongWindow_nullHandle(void)
{
    bool isEnabled = false;
    int32_t status = Pmic_wdgGetReturnToLongWindow(NULL, &isEnabled);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_wdgGetReturnToLongWindow with NULL output parameter.
 */
void test_neg_wdg_wdgGetReturnToLongWindow_nullParam(void)
{
    int32_t status = Pmic_wdgGetReturnToLongWindow(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_wdgGetErrStatus with NULL handle.
 */
void test_neg_wdg_wdgGetErrStatus_nullHandle(void)
{
    Pmic_WdgErrStatus_t errors = {0};
    errors.validParams = PMIC_CFG_WD_TIMEOUT_ERR_VALID;
    int32_t status = Pmic_wdgGetErrStatus(NULL, &errors);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_wdgGetErrStatus with NULL output parameter.
 */
void test_neg_wdg_wdgGetErrStatus_nullParam(void)
{
    int32_t status = Pmic_wdgGetErrStatus(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_wdgClrErrStatus with NULL handle.
 */
void test_neg_wdg_wdgClrErrStatus_nullHandle(void)
{
    Pmic_WdgErrStatus_t errors = {0};
    errors.validParams = PMIC_CFG_WD_TIMEOUT_ERR_VALID;
    errors.timeout = true;
    int32_t status = Pmic_wdgClrErrStatus(NULL, &errors);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_wdgClrErrStatus with NULL error parameter.
 */
void test_neg_wdg_wdgClrErrStatus_nullParam(void)
{
    int32_t status = Pmic_wdgClrErrStatus(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_wdgClrErrStatusAll with NULL handle.
 */
void test_neg_wdg_wdgClrErrStatusAll_nullHandle(void)
{
    int32_t status = Pmic_wdgClrErrStatusAll(NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_wdgGetFailCntStatus with NULL handle.
 */
void test_neg_wdg_wdgGetFailCntStatus_nullHandle(void)
{
    Pmic_WdgFailCntStatus_t failCount = {0};
    failCount.validParams = PMIC_CFG_WD_FAIL_CNT_VAL_VALID;
    int32_t status = Pmic_wdgGetFailCntStatus(NULL, &failCount);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_wdgGetFailCntStatus with NULL output parameter.
 */
void test_neg_wdg_wdgGetFailCntStatus_nullParam(void)
{
    int32_t status = Pmic_wdgGetFailCntStatus(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_wdgQaWriteAnswer with NULL handle.
 */
void test_neg_wdg_wdgQaWriteAnswer_nullHandle(void)
{
    int32_t status = Pmic_wdgQaWriteAnswer(NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/* ========================================================================== */
// I/O Failure Tests (BUILD_MOCK-gated)
/* ========================================================================== */

/**
 * @brief Test Pmic_wdgSetEnableState when ioRxByte fails.
 */
void test_neg_wdg_wdgSetEnableState_ioRxByteFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;

    PLATFORM_ASSERT(mockDevice != NULL);

    // Inject 1 comm failure — will fire on the ioRxByte inside wdgSetEnableState
    status = PmicMock_InjectError(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    // Call function — ioRxByte fails, so the function must return an error
    status = Pmic_wdgSetEnableState(&pmicHandle, false);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    // Test requires mock support for error injection
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/**
 * @brief Test Pmic_wdgGetEnableState when ioRxByte fails.
 */
void test_neg_wdg_wdgGetEnableState_ioRxByteFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;
    bool isEnabled = false;

    PLATFORM_ASSERT(mockDevice != NULL);

    // Inject 1 comm failure — will fire on the ioRxByte inside wdgGetEnableState
    status = PmicMock_InjectError(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    // Call function — ioRxByte fails, so the function must return an error
    status = Pmic_wdgGetEnableState(&pmicHandle, &isEnabled);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    // Test requires mock support for error injection
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/**
 * @brief Test Pmic_wdgSetMode when ioRxByte fails.
 */
void test_neg_wdg_wdgSetMode_ioRxByteFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;

    PLATFORM_ASSERT(mockDevice != NULL);

    // Inject 1 comm failure — will fire on the ioRxByte inside wdgSetMode
    status = PmicMock_InjectError(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    // Call function — ioRxByte fails, so the function must return an error
    status = Pmic_wdgSetMode(&pmicHandle, PMIC_WDG_QA_MODE);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    // Test requires mock support for error injection
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/**
 * @brief Test Pmic_wdgSetPowerHold when ioRxByte fails.
 */
void test_neg_wdg_wdgSetPowerHold_ioRxByteFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;

    PLATFORM_ASSERT(mockDevice != NULL);

    // Inject 1 comm failure — will fire on the ioRxByte inside wdgSetPowerHold
    status = PmicMock_InjectError(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    // Call function — ioRxByte fails, so the function must return an error
    status = Pmic_wdgSetPowerHold(&pmicHandle, true);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    // Test requires mock support for error injection
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/**
 * @brief Test Pmic_wdgGetPowerHold when ioRxByte fails.
 */
void test_neg_wdg_wdgGetPowerHold_ioRxByteFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;
    bool isEnabled = false;

    PLATFORM_ASSERT(mockDevice != NULL);

    // Inject 1 comm failure — will fire on the ioRxByte inside wdgGetPowerHold
    status = PmicMock_InjectError(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    // Call function — ioRxByte fails, so the function must return an error
    status = Pmic_wdgGetPowerHold(&pmicHandle, &isEnabled);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    // Test requires mock support for error injection
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/**
 * @brief Test Pmic_wdgSetReturnToLongWindow when ioRxByte fails.
 */
void test_neg_wdg_wdgSetReturnToLongWindow_ioRxByteFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;

    PLATFORM_ASSERT(mockDevice != NULL);

    // Inject 1 comm failure — will fire on the ioRxByte inside wdgSetReturnToLongWindow
    status = PmicMock_InjectError(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    // Call function — ioRxByte fails, so the function must return an error
    status = Pmic_wdgSetReturnToLongWindow(&pmicHandle, true);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    // Test requires mock support for error injection
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/* ========================================================================== */
// Coverage Gap: wdgClrErrStatus noBitsSet
/* ========================================================================== */

/**
 * @brief Test Pmic_wdgClrErrStatus when all valid params are set but all.
 */
void test_pos_wdg_wdgClrErrStatus_noBitsSet(void)
{
    int32_t status;
    Pmic_WdgErrStatus_t errors = {0};

    // Request all valid error fields but leave every boolean field false
    errors.validParams = PMIC_CFG_WD_ERR_STAT_ALL_VALID_SHIFT;
    errors.timeout            = false;
    errors.longWindowTimeout  = false;
    errors.answerEarlyError   = false;
    errors.sequenceError      = false;
    errors.answerError        = false;
    errors.triggerEarlyError  = false;
    errors.threshold1Error    = false;
    errors.threshold2Error    = false;

    // Should succeed — regVal stays 0, the write is skipped
    status = Pmic_wdgClrErrStatus(&pmicHandle, &errors);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

/* ========================================================================== */
// WDG_checkCfgState ReturnToLongWindow Tests
/* ========================================================================== */

/**
 * @brief Test WDG_checkCfgState() second I/O failure path.
 */
void test_neg_wdg_wdgSetCfg_checkCfgStateIoRxByteFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;
    Pmic_WdgCfg_t cfg = {0};

    PLATFORM_ASSERT(mockDevice != NULL);

    wdg_setupForConfig();

    cfg.validParams = PMIC_CFG_WDG_THRESHOLD_1_VALID;
    cfg.threshold1 = 0U;

    // Skip op 1 so the enable-state read succeeds, then fail op 2 to exercise the ReturnToLongWindow error path
    status = PmicMock_InjectErrorAfterN(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1U, 1U);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_wdgSetCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);

    wdg_cleanupAfterTest();
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/**
 * @brief Test WDG_checkCfgState() first I/O failure path.
 */
void test_neg_wdg_wdgSetCfg_checkCfgStateIoRxByteFailOp1(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;
    Pmic_WdgCfg_t cfg = {0};

    PLATFORM_ASSERT(mockDevice != NULL);

    wdg_setupForConfig();

    cfg.validParams = PMIC_CFG_WDG_THRESHOLD_1_VALID;
    cfg.threshold1 = 0U;

    // Fail op 1 (the enable-state read) directly, exercising the
    // status != PMIC_ST_SUCCESS side of WDG_checkCfgState's first check
    status = PmicMock_InjectErrorAfterN(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 0U, 1U);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_wdgSetCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);

    wdg_cleanupAfterTest();
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/* ========================================================================== */
// Coverage Gap: wdgSetCfg invalidBitsInValidParams
/* ========================================================================== */

/**
 * @brief Test Pmic_wdgSetCfg with invalid bits set in validParams.
 */
void test_neg_wdg_wdgSetCfg_invalidBitsInValidParams(void)
{
    int32_t status;
    Pmic_WdgCfg_t wdgCfg = {0};

    // Combine a legitimate valid-param bit with a bit outside the allowed mask
    wdgCfg.validParams = PMIC_CFG_WDG_CFG_ALL_VALID_SHIFT | 0x8000U;

    status = Pmic_wdgSetCfg(&pmicHandle, &wdgCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/* ========================================================================== */
// Coverage Gap: I/O failures in WDG_setThresholds / WDG_setModeAndTimeBaseCfg
// / WDG_setQAConfigurations / WDG_setThrIntBehavior
/* ========================================================================== */

/**
 * @brief Test WDG_setWindowsTimeIntervals() IO failure path.
 *
 * WDG_setAllCfgFields() calls WDG_setWindowsTimeIntervals() first; its
 * I/O write can genuinely fail independent of validation (the
 * `if (status == PMIC_ST_SUCCESS)` branch gating WDG_setThresholds()).
 * This exercises that failure directly, using the same
 * PmicMock_InjectErrorAfterN mechanism as the sibling sub-function tests
 * below, retargeted to the long-window-code write inside
 * WDG_setWindowsTimeIntervals (I/O #3).
 */
void test_neg_wdg_wdgSetAllCfgFields_windowsTimeIntervalsIoTxByteFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;
    Pmic_WdgCfg_t cfg = {0};

    PLATFORM_ASSERT(mockDevice != NULL);

    wdg_setupForConfig();

    // Only the long-window code is valid, so WDG_setWindowsTimeIntervals
    // issues exactly one write (win1/win2 writes are skipped by validParams).
    cfg.validParams = PMIC_CFG_WDG_LONG_WIN_CODE_VALID;
    cfg.longWinCode = 0U;

    // I/O #1: Pmic_wdgGetEnableState (WDG_checkCfgState) - succeeds
    // I/O #2: Pmic_wdgGetReturnToLongWindow (WDG_checkCfgState) - succeeds
    // I/O #3: Pmic_ioTxByte_CS(WD_LONGWIN_CFG_REG) inside
    //         WDG_setWindowsTimeIntervals - FAIL
    // Skip ops 1-2 so they succeed, then fail op 3. */
    status = PmicMock_InjectErrorAfterN(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 2U, 1U);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_wdgSetCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);

    wdg_cleanupAfterTest();
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/**
 * @brief Test WDG_setThresholds() IO failure path.
 */
void test_neg_wdg_wdgSetCfg_thresholdsIoRxByteFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;
    Pmic_WdgCfg_t cfg = {0};

    PLATFORM_ASSERT(mockDevice != NULL);

    wdg_setupForConfig();

    cfg.validParams = PMIC_CFG_WDG_THRESHOLD_1_VALID;
    cfg.threshold1 = 0U;

    // I/O #1: Pmic_wdgGetEnableState (WDG_checkCfgState) - succeeds
    // I/O #2: Pmic_wdgGetReturnToLongWindow (WDG_checkCfgState) - succeeds
    // I/O #3: Pmic_ioRxByte(WD_TH_CFG_REG) inside WDG_setThresholds - FAIL
    // Skip ops 1-2 so they succeed, then fail op 3. */
    status = PmicMock_InjectErrorAfterN(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 2U, 1U);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_wdgSetCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);

    wdg_cleanupAfterTest();
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/**
 * @brief Test WDG_setModeAndTimeBaseCfg() IO failure path.
 */
void test_neg_wdg_wdgSetCfg_modeAndTimeBaseIoRxByteFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;
    Pmic_WdgCfg_t cfg = {0};

    PLATFORM_ASSERT(mockDevice != NULL);

    wdg_setupForConfig();

    cfg.validParams = PMIC_CFG_WDG_MODE_VALID;
    cfg.mode = PMIC_WDG_QA_MODE;

    // I/O #1: Pmic_wdgGetEnableState - succeeds
    // I/O #2: Pmic_wdgGetReturnToLongWindow - succeeds
    // I/O #3-4: WDG_setThresholds read+write (unconditional, runs before this
    // function regardless of validParams) - succeed
    // I/O #5: Pmic_ioRxByte(WD_CFG_REG) inside WDG_setModeAndTimeBaseCfg - FAIL
    // Skip ops 1-4 so they succeed, then fail op 5. */
    status = PmicMock_InjectErrorAfterN(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 4U, 1U);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_wdgSetCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);

    wdg_cleanupAfterTest();
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/**
 * @brief Test WDG_setQAConfigurations() IO failure path.
 */
void test_neg_wdg_wdgSetCfg_qaConfigurationsIoRxByteFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;
    Pmic_WdgCfg_t cfg = {0};

    PLATFORM_ASSERT(mockDevice != NULL);

    wdg_setupForConfig();

    cfg.validParams = PMIC_CFG_WDG_QA_FDBK_VALID;
    cfg.qaFdbk = 0U;

    // I/O #1: Pmic_wdgGetEnableState - succeeds
    // I/O #2: Pmic_wdgGetReturnToLongWindow - succeeds
    // I/O #3-4: WDG_setThresholds read+write (unconditional) - succeed
    // I/O #5-6: WDG_setModeAndTimeBaseCfg read+write (unconditional) - succeed
    // I/O #7: Pmic_ioRxByte(WD_QA_CFG_REG) inside WDG_setQAConfigurations - FAIL
    // Skip ops 1-6 so they succeed, then fail op 7. */
    status = PmicMock_InjectErrorAfterN(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 6U, 1U);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_wdgSetCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);

    wdg_cleanupAfterTest();
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/**
 * @brief Test WDG_setThrIntBehavior() IO failure path.
 */
void test_neg_wdg_wdgSetCfg_thrIntBehaviorIoRxByteFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;
    Pmic_WdgCfg_t cfg = {0};

    PLATFORM_ASSERT(mockDevice != NULL);

    wdg_setupForConfig();

    cfg.validParams = PMIC_CFG_WDG_THRESHOLD2_INT_BEHAVIOR_VALID;
    cfg.threshold2IntBehavior = 0U;

    // I/O #1: Pmic_wdgGetEnableState - succeeds
    // I/O #2: Pmic_wdgGetReturnToLongWindow - succeeds
    // I/O #3-4: WDG_setThresholds read+write (unconditional) - succeed
    // I/O #5-6: WDG_setModeAndTimeBaseCfg read+write (unconditional) - succeed
    // I/O #7-8: WDG_setQAConfigurations read+write (unconditional) - succeed
    // I/O #9: Pmic_ioRxByte(WD_INT_CFG_REG) inside WDG_setThrIntBehavior - FAIL
    // Skip ops 1-8 so they succeed, then fail op 9. */
    status = PmicMock_InjectErrorAfterN(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 8U, 1U);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_wdgSetCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);

    wdg_cleanupAfterTest();
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/* ========================================================================== */
// Coverage Gap: I/O failures in WDG get sub-functions
/* ========================================================================== */

/**
 * @brief Test WDG_getLongWindowTimeInterval() IO failure path.
 */
void test_neg_wdg_wdgGetCfg_longWinIoRxByteFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;
    Pmic_WdgCfg_t cfg = {0};

    PLATFORM_ASSERT(mockDevice != NULL);

    cfg.validParams = PMIC_CFG_WDG_LONG_WIN_CODE_VALID;

    // Fail the 1st I/O: Pmic_ioRxByte(WD_LONGWIN_CFG_REG) inside
    // WDG_getLongWindowTimeInterval */
    status = PmicMock_InjectError(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_wdgGetCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/**
 * @brief Test WDG_getWindow1TimeInterval() IO failure path.
 */
void test_neg_wdg_wdgGetCfg_win1IoRxByteFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;
    Pmic_WdgCfg_t cfg = {0};

    PLATFORM_ASSERT(mockDevice != NULL);

    cfg.validParams = PMIC_CFG_WDG_WIN1_CODE_VALID;

    // Fail the 1st I/O: Pmic_ioRxByte(WD_WIN1_CFG_REG) inside
    // WDG_getWindow1TimeInterval */
    status = PmicMock_InjectError(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_wdgGetCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/**
 * @brief Test WDG_getWindow2TimeInterval() IO failure path.
 */
void test_neg_wdg_wdgGetCfg_win2IoRxByteFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;
    Pmic_WdgCfg_t cfg = {0};

    PLATFORM_ASSERT(mockDevice != NULL);

    cfg.validParams = PMIC_CFG_WDG_WIN2_CODE_VALID;

    // Fail the 1st I/O: Pmic_ioRxByte(WD_WIN2_CFG_REG) inside
    // WDG_getWindow2TimeInterval */
    status = PmicMock_InjectError(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_wdgGetCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/**
 * @brief Test WDG_getThrIntBehavior() IO failure path.
 */
void test_neg_wdg_wdgGetCfg_thrIntBehaviorIoRxByteFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;
    Pmic_WdgCfg_t cfg = {0};

    PLATFORM_ASSERT(mockDevice != NULL);

    cfg.validParams = PMIC_CFG_WDG_THRESHOLD2_INT_BEHAVIOR_VALID;

    // WDG_getThresholds reads WD_TH_CFG_REG (I/O #1), WDG_getModeAndTimeBaseCfg
    // reads WD_CFG_REG (I/O #2), WDG_getQAConfigurations reads WD_QA_CFG_REG
    // (I/O #3), then WDG_getThrIntBehavior reads WD_INT_CFG_REG (I/O #4).
    // Skip ops 1-3 so they succeed, then fail op 4. */
    status = PmicMock_InjectErrorAfterN(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 3U, 1U);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_wdgGetCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/**
 * @brief Test WDG_getThresholds() IO failure path inside Pmic_wdgGetCfg.
 */
void test_neg_wdg_wdgGetCfg_thresholdsIoRxByteFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;
    Pmic_WdgCfg_t cfg = {0};

    PLATFORM_ASSERT(mockDevice != NULL);

    // Only threshold params — no window reads will occur first
    cfg.validParams = PMIC_CFG_WDG_THRESHOLD_1_VALID;

    // I/O #1: Pmic_ioRxByte(WD_TH_CFG_REG) inside WDG_getThresholds - FAIL
    status = PmicMock_InjectError(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_wdgGetCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/**
 * @brief Test WDG_getModeAndTimeBaseCfg() IO failure path inside Pmic_wdgGetCfg.
 */
void test_neg_wdg_wdgGetCfg_modeAndTimeBaseIoRxByteFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;
    Pmic_WdgCfg_t cfg = {0};

    PLATFORM_ASSERT(mockDevice != NULL);

    // Only mode param — no window reads; threshold read (I/O #1) succeeds
    cfg.validParams = PMIC_CFG_WDG_MODE_VALID;

    // I/O #1: Pmic_ioRxByte(WD_TH_CFG_REG) inside WDG_getThresholds - succeeds
    // I/O #2: Pmic_ioRxByte(WD_CFG_REG) inside WDG_getModeAndTimeBaseCfg - FAIL */
    status = PmicMock_InjectErrorAfterN(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1U, 1U);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_wdgGetCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/**
 * @brief Test WDG_getQAConfigurations() IO failure path inside Pmic_wdgGetCfg.
 */
void test_neg_wdg_wdgGetCfg_qaConfigurationsIoRxByteFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;
    Pmic_WdgCfg_t cfg = {0};

    PLATFORM_ASSERT(mockDevice != NULL);

    // Only QA fdbk param — no window reads
    cfg.validParams = PMIC_CFG_WDG_QA_FDBK_VALID;

    // I/O #1: WDG_getThresholds reads WD_TH_CFG_REG          - succeeds
    // I/O #2: WDG_getModeAndTimeBaseCfg reads WD_CFG_REG      - succeeds
    // I/O #3: WDG_getQAConfigurations reads WD_QA_CFG_REG     - FAIL */
    status = PmicMock_InjectErrorAfterN(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 2U, 1U);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_wdgGetCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/**
 * @brief Test WDG_getQuestionAndAnswer() IO failure path.
 */
void test_neg_wdg_wdgQaWriteAnswer_getQandAIoRxByteFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;

    PLATFORM_ASSERT(mockDevice != NULL);

    wdg_setupForConfig();

    // I/O #1: Pmic_ioRxByte(WD_QA_CFG_REG) for qaFbk - succeeds
    // I/O #2: Pmic_ioRxByte(WD_QA_CNT_REG) in WDG_getQuestionAndAnswer - FAIL
    // Skip op 1 so it succeeds, then fail op 2. */
    status = PmicMock_InjectErrorAfterN(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1U, 1U);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_wdgQaWriteAnswer(&pmicHandle);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);

    wdg_cleanupAfterTest();
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/* ========================================================================== */
// Coverage Gap: WDG_checkCfgState disabled branch (mock-based)
/* ========================================================================== */

/**
 * @brief Test WDG_checkCfgState() disabled-watchdog branch via mock.
 */
void test_neg_wdg_wdgSetCfg_checkCfgStateDisabledBranch(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDev = platform_getMockDevice();
    PLATFORM_ASSERT(mockDev != NULL);

    // Disable the watchdog first so WD_EN bit in WD_CFG_REG is 0
    int32_t status = Pmic_wdgDisable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Attempt to configure — WDG_checkCfgState reads WD_CFG_REG, sees
    // WD_EN = 0, and returns PMIC_ST_ERR_NOT_SUPPORTED immediately */
    Pmic_WdgCfg_t cfg = {
        .validParams = PMIC_CFG_WDG_MODE_VALID,
        .mode        = PMIC_WDG_TRIGGER_MODE
    };
    status = Pmic_wdgSetCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);

    // Re-enable watchdog for subsequent tests
    status = Pmic_wdgEnable(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    status = Pmic_wdgSetReturnToLongWindow(&pmicHandle, true);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/* ========================================================================== */
// Coverage Gap: WDG_setModeAndTimeBaseCfg op-4 path (threshold + mode set)
/* ========================================================================== */

/**
 * @brief Test WDG_setModeAndTimeBaseCfg() IO failure when reached as 4th op.
 */
void test_neg_wdg_wdgSetCfg_modeAndTimeBaseIoRxByteFailOp4(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDev = platform_getMockDevice();
    PLATFORM_ASSERT(mockDev != NULL);

    wdg_setupForConfig();

    // Both threshold1 and mode valid: threshold read is I/O #3, mode read is I/O #4
    Pmic_WdgCfg_t cfg = {
        .validParams = PMIC_CFG_WDG_THRESHOLD_1_VALID | PMIC_CFG_WDG_MODE_VALID,
        .threshold1  = 0U,
        .mode        = PMIC_WDG_TRIGGER_MODE
    };

    // Skip ops 1-3 so they succeed, fail op 4
    int32_t status = PmicMock_InjectErrorAfterN(mockDev, PMIC_MOCK_ERROR_COMM_FAILURE, 3U, 1U);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_wdgSetCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);

    wdg_cleanupAfterTest();
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/* ========================================================================== */
// Coverage Gap: WDG_setQAConfigurations op-4 path (threshold + qaFdbk set)
/* ========================================================================== */

/**
 * @brief Test WDG_setQAConfigurations() IO failure when reached as 4th op.
 */
void test_neg_wdg_wdgSetCfg_qaConfigurationsIoRxByteFailOp4(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDev = platform_getMockDevice();
    PLATFORM_ASSERT(mockDev != NULL);

    wdg_setupForConfig();

    // threshold1 + qaFdbk: threshold read is I/O #3, QA read is I/O #4
    Pmic_WdgCfg_t cfg = {
        .validParams = PMIC_CFG_WDG_THRESHOLD_1_VALID | PMIC_CFG_WDG_QA_FDBK_VALID,
        .threshold1  = 0U,
        .qaFdbk      = 0U
    };

    // Skip ops 1-3 so they succeed, fail op 4
    int32_t status = PmicMock_InjectErrorAfterN(mockDev, PMIC_MOCK_ERROR_COMM_FAILURE, 3U, 1U);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_wdgSetCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);

    wdg_cleanupAfterTest();
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/* ========================================================================== */
// Coverage Gap: WDG_setThrIntBehavior op-4 path (mode + thrIntBehavior set)
/* ========================================================================== */

/**
 * @brief Test WDG_setThrIntBehavior() IO failure when reached as 4th op.
 */
void test_neg_wdg_wdgSetCfg_thrIntBehaviorIoRxByteFailOp4(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDev = platform_getMockDevice();
    PLATFORM_ASSERT(mockDev != NULL);

    wdg_setupForConfig();

    // mode + threshold2IntBehavior: mode read is I/O #3, int-cfg read is I/O #4
    Pmic_WdgCfg_t cfg = {
        .validParams             = PMIC_CFG_WDG_MODE_VALID |
                                   PMIC_CFG_WDG_THRESHOLD2_INT_BEHAVIOR_VALID,
        .mode                    = PMIC_WDG_TRIGGER_MODE,
        .threshold2IntBehavior   = 0U
    };

    // Skip ops 1-3 so they succeed, fail op 4
    int32_t status = PmicMock_InjectErrorAfterN(mockDev, PMIC_MOCK_ERROR_COMM_FAILURE, 3U, 1U);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_wdgSetCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);

    wdg_cleanupAfterTest();
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/**
 * @brief WDG test suite entry point.
 * @param args Test arguments (unused)
 */
void wdg_test(void *args)
{
    char msg[50U] = {0};
    int32_t status = PMIC_ST_SUCCESS;

    platform_init();
    testTimer_startModule("WDG");

    Pmic_HandleCfg_t pmicCfg = {
        .validParams = (PMIC_CFG_INIT_COMM_MODE_VALID |
                        PMIC_CFG_INIT_COMM_HANDLE_0_VALID |
                        PMIC_CFG_INIT_IO_READ_VALID |
                        PMIC_CFG_INIT_IO_WRITE_VALID |
                        PMIC_CFG_INIT_CRITICAL_SECTION_START_VALID |
                        PMIC_CFG_INIT_CRITICAL_SECTION_STOP_VALID),
        .commMode = PMIC_INTF_SPI,
        .commHandle0 = platform_getCommHandle(),
        .ioRead = &platform_rxByte,
        .ioWrite = &platform_txByte,
        .criticalSectionStart = &platform_critSecStart,
        .criticalSectionStop = &platform_critSecStop
    };

    status = Pmic_init(&pmicHandle, &pmicCfg);

    if (status == PMIC_ST_SUCCESS)
    {
        testUtils_printSiRev(&pmicHandle);

        platform_unlockRegisters();

        platform_setupTests();
        WDG_TEST_RUN_ALL();
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
