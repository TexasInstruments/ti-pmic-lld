/******************************************************************************
 * Copyright (c) 2024 Texas Instruments Incorporated - http://www.ti.com
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
 *  @file   pmic_wdg_test.c
 *
 *  @brief: This file contains all the testing related files APIs for the PMIC
 *          Watchdog timer servicing tests.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include "pmic_wdg_test.h"

/* ========================================================================== */
/*                            Function Definitions                            */
/* ========================================================================== */

/* Pointer to Pmic Core Handle */
Pmic_CoreHandle_t *pPmicCoreHandle_wdg = NULL;

/*!
 * @brief   Initialize the PMIC WDG configuration for testing.
 * This function initializes the PMIC WDG configuration for testing purposes.
 * It sets up various parameters required for WDG configuration and initializes
 * the PMIC core handle.
 *
 * @param   void
 * @return  Returns the status of the initialization process.
 */
int32_t test_pmic_wdg_config_init(void) {
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_CoreCfg_t pmicConfigData = {0U};

    /* Fill parameters to pmicConfigData */
    pmicConfigData.pmicDeviceType = PMIC_DEV_BB_TPS65386X;
    pmicConfigData.validParams |= PMIC_CFG_DEVICE_TYPE_VALID_SHIFT;

    pmicConfigData.commMode = PMIC_INTF_SPI;
    pmicConfigData.validParams |= PMIC_CFG_COMM_MODE_VALID_SHIFT;

    pmicConfigData.pFnPmicCommIoRead = test_pmic_regRead;
    pmicConfigData.validParams |= PMIC_CFG_COMM_IO_RD_VALID_SHIFT;

    pmicConfigData.pFnPmicCommIoWrite = test_pmic_regWrite;
    pmicConfigData.validParams |= PMIC_CFG_COMM_IO_WR_VALID_SHIFT;

    pmicConfigData.pFnPmicCritSecStart = test_pmic_criticalSectionStartFn;
    pmicConfigData.validParams |= PMIC_CFG_CRITSEC_START_VALID_SHIFT;

    pmicConfigData.pFnPmicCritSecStop = test_pmic_criticalSectionStopFn;
    pmicConfigData.validParams |= PMIC_CFG_CRITSEC_STOP_VALID_SHIFT;

    status = test_pmic_appInit(&pPmicCoreHandle_wdg, &pmicConfigData);
    if (PMIC_ST_SUCCESS != status) {
        DebugP_log("%s(): %d: FAILED with status: %d\r\n", __func__, __LINE__,
                   status);
    }
    return status;
}

/*!
 * @brief   Deinitialize the PMIC WDG configuration after testing.
 * This function deinitializes the PMIC WDG configuration after testing is
 * completed. It deinitializes the PMIC core handle and frees up any allocated
 * memory.
 *
 * @param   void
 * @return  Returns the status of the deinitialization process.
 */
int32_t test_pmic_wdg_config_deinit(void) {
    int32_t status = PMIC_ST_SUCCESS;

    status = Pmic_deinit(pPmicCoreHandle_wdg);
    free(pPmicCoreHandle_wdg);
    SemaphoreP_destruct(&gpmicCoreObj);
    if (PMIC_ST_SUCCESS != status) {
        DebugP_log("%s(): %d: FAILED with status: %d\r\n", __func__, __LINE__,
                   status);
    }
    return status;
}

/**
 * @brief Test enabling and disabling the watchdog feature.
 * This function tests the enabling and disabling of the watchdog feature.
 * It enables the watchdog, checks if it's enabled, disables it, and checks if
 * it's disabled.
 *
 * @param  void
 * @return NULL
 */
void test_wdg_enableDisable(void) {
    int32_t status = PMIC_ST_SUCCESS;
    bool wdgEnabled = false;

    /* Enable Watchdog */
    status = Pmic_wdgEnable(pPmicCoreHandle_wdg);
    if (PMIC_ST_SUCCESS == status) {
        /* Get actual Watchdog enable/disable status and compare expected vs.
         * actual value */
        status = Pmic_wdgGetEnableState(pPmicCoreHandle_wdg, &wdgEnabled);
        if (wdgEnabled == PMIC_WDG_ENABLE) {
            DebugP_log("Test Passed: Watchdog Enable Test successful!\r\n");
        } else {
            DebugP_log("Test Failed: Watchdog Enable Test failed!\r\n");
        }
    }

    /* Disable Watchdog */
    status = Pmic_wdgDisable(pPmicCoreHandle_wdg);
    if (PMIC_ST_SUCCESS == status) {
        /* Get actual Watchdog enable/disable status and compare expected vs.
         * actual value */
        status = Pmic_wdgGetEnableState(pPmicCoreHandle_wdg, &wdgEnabled);
        if (wdgEnabled == PMIC_WDG_DISABLE) {
            DebugP_log("Test Passed: Watchdog Disable Test successful!\r\n");
        } else {
            DebugP_log("Test Failed: Watchdog Disable Test failed!\r\n");
        }
    }
}

/**
 * @brief Check if the watchdog is enabled.
 * This function checks if the watchdog is enabled and logs the status.
 *
 * @param void
 * @return NULL
 */
static void checkWdgEnabled(void) {
    int32_t status = PMIC_ST_SUCCESS;
    bool wdgEnabled = false;

    status = Pmic_wdgGetEnableState(pPmicCoreHandle_wdg, &wdgEnabled);
    if (PMIC_ST_SUCCESS == status) {
        status = Pmic_wdgEnable(pPmicCoreHandle_wdg);
        if (PMIC_ST_SUCCESS == status) {
            if (wdgEnabled != PMIC_WDG_ENABLE) {
                DebugP_log("Watchdog Enabled\r\n");
            } else {
                DebugP_log("Watchdog Disabled\r\n");
            }
        }
    }
}

/**
 * @brief Test setting the long window duration for the watchdog.
 * This function tests setting and getting the long window duration
 * configuration for the watchdog.
 *
 * @param void
 * @return NULL
 */
void test_wdg_setCfg_longWindowDuration(void) {
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_WdgCfg_t wdgCfg_expected, wdgCfg_actual;

    checkWdgEnabled();

    wdgCfg_expected.validParams = PMIC_CFG_WDG_LONGWINDURATION_VALID_SHIFT;
    wdgCfg_actual.validParams = PMIC_CFG_WDG_LONGWINDURATION_VALID_SHIFT;

    /* Set Watchdog Long Window duration to be maximum duration */
    wdgCfg_expected.longWinDuration_ms = 772000;
    status = Pmic_wdgSetCfg(pPmicCoreHandle_wdg, wdgCfg_expected);
    if (PMIC_ST_SUCCESS == status) {
        status = Pmic_wdgGetCfg(pPmicCoreHandle_wdg, &wdgCfg_actual);
        if (wdgCfg_actual.longWinDuration_ms ==
            wdgCfg_expected.longWinDuration_ms) {
            DebugP_log("Test Passed: Watchdog Long Window duration set/get "
                       "configuration passed!\r\n");
        } else {
            DebugP_log("Test Failed: Watchdog Long Window duration set/get "
                       "configuration failed!\r\n");
        }
    }
}

/**
 * @brief Test setting the Window-1 duration for the watchdog.
 * This function tests setting and getting the Window-1 duration configuration
 * for the watchdog.
 *
 * @param void
 * @return NULL
 */
void test_wdg_setCfg_window1Duration(void) {
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_WdgCfg_t wdgCfg_expected, wdgCfg_actual;

    checkWdgEnabled();

    wdgCfg_expected.validParams = PMIC_CFG_WDG_WIN1DURATION_VALID_SHIFT;
    wdgCfg_actual.validParams = PMIC_CFG_WDG_WIN1DURATION_VALID_SHIFT;

    /* Set Watchdog Window-1 duration to be maximum duration */
    wdgCfg_expected.win1Duration_us = 70400;
    status = Pmic_wdgSetCfg(pPmicCoreHandle_wdg, wdgCfg_expected);
    if (PMIC_ST_SUCCESS == status) {
        /* Get actual Window-1 duration and compare expected vs. actual value */
        status = Pmic_wdgGetCfg(pPmicCoreHandle_wdg, &wdgCfg_actual);
        if (wdgCfg_actual.win1Duration_us == wdgCfg_expected.win1Duration_us) {
            DebugP_log("Test Passed: Watchdog Window-1 duration set/get "
                       "configuration passed!\r\n");
        } else {
            DebugP_log("Test Failed: Watchdog Window-1 duration set/get "
                       "configuration failed!\r\n");
        }
    }
}

/**
 * @brief Test setting the Window-2 duration for the watchdog.
 * This function tests setting and getting the Window-2 duration configuration
 * for the watchdog.
 *
 * @param void
 * @return NULL
 */
void test_wdg_setCfg_window2Duration(void) {
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_WdgCfg_t wdgCfg_expected, wdgCfg_actual;

    checkWdgEnabled();

    wdgCfg_expected.validParams = PMIC_CFG_WDG_WIN2DURATION_VALID_SHIFT;
    wdgCfg_actual.validParams = PMIC_CFG_WDG_WIN2DURATION_VALID_SHIFT;

    /* Set Watchdog Window-2 duration to be maximum duration */
    wdgCfg_expected.win2Duration_us = 70400;
    status = Pmic_wdgSetCfg(pPmicCoreHandle_wdg, wdgCfg_expected);
    if (PMIC_ST_SUCCESS == status) {
        status = Pmic_wdgGetCfg(pPmicCoreHandle_wdg, &wdgCfg_actual);
        if (wdgCfg_actual.win2Duration_us == wdgCfg_expected.win2Duration_us) {
            DebugP_log("Test Passed: Watchdog Window-2 duration set/get "
                       "configuration passed!\r\n");
        } else {
            DebugP_log("Test Failed: Watchdog Window-2 duration set/get "
                       "configuration failed!\r\n");
        }
    }
}

/**
 * @brief Test setting the fail threshold for the watchdog.
 * This function tests setting and getting the fail threshold configuration for
 * the watchdog.
 *
 * @param void
 * @return NULL
 */
void test_wdg_setCfg_failThreshold(void) {
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_WdgCfg_t wdgCfg_expected, wdgCfg_actual;

    checkWdgEnabled();

    wdgCfg_expected.validParams = PMIC_CFG_WDG_FAILTHRESHOLD_VALID_SHIFT;
    wdgCfg_actual.validParams = PMIC_CFG_WDG_FAILTHRESHOLD_VALID_SHIFT;

    /* Set fail threshold to maximum value */
    wdgCfg_expected.failThreshold = 7;
    status = Pmic_wdgSetCfg(pPmicCoreHandle_wdg, wdgCfg_expected);
    if (PMIC_ST_SUCCESS == status) {
        status = Pmic_wdgGetCfg(pPmicCoreHandle_wdg, &wdgCfg_actual);
        if (wdgCfg_actual.failThreshold == wdgCfg_expected.failThreshold) {
            DebugP_log("Test Passed: Watchdog Fail Threshold set/get "
                       "configuration passed!\r\n");
        } else {
            DebugP_log("Test Failed: Watchdog Fail Threshold set/get "
                       "configuration failed!\r\n");
        }
    }
}

/**
 * @brief Test setting the reset threshold for the watchdog.
 * This function tests setting and getting the reset threshold configuration for
 * the watchdog.
 *
 * @param void
 * @return NULL
 */
void test_wdg_setCfg_resetThreshold(void) {
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_WdgCfg_t wdgCfg_expected, wdgCfg_actual;

    checkWdgEnabled();

    wdgCfg_expected.validParams = PMIC_CFG_WDG_RSTTHRESHOLD_VALID_SHIFT;
    wdgCfg_actual.validParams = PMIC_CFG_WDG_RSTTHRESHOLD_VALID_SHIFT;

    /* Set fail threshold to maximum value */
    wdgCfg_expected.rstThreshold = 7;
    status = Pmic_wdgSetCfg(pPmicCoreHandle_wdg, wdgCfg_expected);
    if (PMIC_ST_SUCCESS == status) {
        status = Pmic_wdgGetCfg(pPmicCoreHandle_wdg, &wdgCfg_actual);
        if (wdgCfg_actual.rstThreshold == wdgCfg_expected.rstThreshold) {
            DebugP_log("Test Passed: Watchdog Reset Threshold set/get "
                       "configuration passed!\r\n");
        } else {
            DebugP_log("Test Failed: Watchdog Reset Threshold set/get "
                       "configuration failed!\r\n");
        }
    }
}

/**
 * @brief Test setting the reset enable/disable for the watchdog.
 * This function tests setting and getting the reset enable/disable
 * configuration for the watchdog.
 *
 * @param void
 * @return NULL
 */
void test_wdg_setCfg_resetEnable(void) {
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_WdgCfg_t wdgCfg_expected, wdgCfg_actual;

    checkWdgEnabled();

    wdgCfg_expected.validParams = PMIC_CFG_WDG_RSTENABLE_VALID_SHIFT;
    wdgCfg_actual.validParams = PMIC_CFG_WDG_RSTENABLE_VALID_SHIFT;

    /* Enable Watchdog Reset */
    wdgCfg_expected.rstEnable = PMIC_WDG_RESET_ENABLE;
    status = Pmic_wdgSetCfg(pPmicCoreHandle_wdg, wdgCfg_expected);
    if (PMIC_ST_SUCCESS == status) {
        status = Pmic_wdgGetCfg(pPmicCoreHandle_wdg, &wdgCfg_actual);
        if (wdgCfg_actual.rstEnable == wdgCfg_expected.rstEnable) {
            DebugP_log("Test Passed: Watchdog Reset Enable passed!\r\n");
        } else {
            DebugP_log("Test Failed: Watchdog Reset Enable failed!\r\n");
        }
    }

    /* Disable Watchdog Reset */
    wdgCfg_expected.rstEnable = PMIC_WDG_RESET_DISABLE;
    status = Pmic_wdgSetCfg(pPmicCoreHandle_wdg, wdgCfg_expected);
    if (PMIC_ST_SUCCESS == status) {
        status = Pmic_wdgGetCfg(pPmicCoreHandle_wdg, &wdgCfg_actual);
        if (wdgCfg_actual.rstEnable == wdgCfg_expected.rstEnable) {
            DebugP_log("Test Passed: Watchdog Reset Disable passed!\r\n");
        } else {
            DebugP_log("Test Failed: Watchdog Reset Disable failed!\r\n");
        }
    }
}

/**
 * @brief Test setting the watchdog mode for the watchdog.
 * This function tests setting and getting the watchdog mode configuration for
 * the watchdog.
 *
 * @param void
 * @return NULL
 */
void test_wdg_setCfg_wdgMode(void) {
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_WdgCfg_t wdgCfg_expected, wdgCfg_actual;

    checkWdgEnabled();

    wdgCfg_expected.validParams = PMIC_CFG_WDG_WDGMODE_VALID_SHIFT;
    wdgCfg_actual.validParams = PMIC_CFG_WDG_WDGMODE_VALID_SHIFT;

    /* Set Watchdog mode to be Q&A mode */
    wdgCfg_expected.wdgMode = PMIC_WDG_QA_MODE;
    status = Pmic_wdgSetCfg(pPmicCoreHandle_wdg, wdgCfg_expected);
    if (PMIC_ST_SUCCESS == status) {
        status = Pmic_wdgGetCfg(pPmicCoreHandle_wdg, &wdgCfg_actual);
        if (wdgCfg_actual.wdgMode == wdgCfg_expected.wdgMode) {
            DebugP_log("Test Passed: Watchdog set to Q&A mode passed!\r\n");
        } else {
            DebugP_log("Test Failed: Watchdog set to Q&A mode failed!\r\n");
        }
    }

    /* Set Watchdog mode to be trigger mode */
    wdgCfg_expected.wdgMode = PMIC_WDG_TRIGGER_MODE;
    status = Pmic_wdgSetCfg(pPmicCoreHandle_wdg, wdgCfg_expected);
    if (PMIC_ST_SUCCESS == status) {
        status = Pmic_wdgGetCfg(pPmicCoreHandle_wdg, &wdgCfg_actual);
        if (wdgCfg_actual.wdgMode == wdgCfg_expected.wdgMode) {
            DebugP_log("Test Passed: Watchdog set to Trigger mode passed!\r\n");
        } else {
            DebugP_log("Test Failed: Watchdog set to Trigger mode failed!\r\n");
        }
    }
}

/**
 * @brief Test setting the power hold enable/disable for the watchdog.
 * This function tests setting and getting the power hold enable/disable
 * configuration for the watchdog.
 *
 * @param void
 * @return NULL
 */
void test_wdg_setCfg_powerHold(void) {
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_WdgCfg_t wdgCfg_expected, wdgCfg_actual;

    checkWdgEnabled();

    wdgCfg_expected.validParams = PMIC_CFG_WDG_PWRHOLD_VALID_SHIFT;
    wdgCfg_actual.validParams = PMIC_CFG_WDG_PWRHOLD_VALID_SHIFT;

    /* Enable Watchdog PWRHOLD */
    wdgCfg_expected.pwrHold = PMIC_WDG_PWRHOLD_ENABLE;
    status = Pmic_wdgSetCfg(pPmicCoreHandle_wdg, wdgCfg_expected);
    if (PMIC_ST_SUCCESS == status) {
        status = Pmic_wdgGetCfg(pPmicCoreHandle_wdg, &wdgCfg_actual);
        if (wdgCfg_actual.pwrHold == wdgCfg_expected.pwrHold) {
            DebugP_log("Test Passed: Watchdog Power Hold enable passed!\r\n");
        } else {
            DebugP_log("Test Failed: Watchdog Power Hold enable failed!\r\n");
        }
    }

    /* Disable Watchdog PWRHOLD */
    wdgCfg_expected.wdgMode = PMIC_WDG_PWRHOLD_DISABLE;
    status = Pmic_wdgSetCfg(pPmicCoreHandle_wdg, wdgCfg_expected);
    if (PMIC_ST_SUCCESS == status) {
        status = Pmic_wdgGetCfg(pPmicCoreHandle_wdg, &wdgCfg_actual);
        if (wdgCfg_actual.pwrHold == wdgCfg_expected.pwrHold) {
            DebugP_log("Test Passed: Watchdog Power Hold disable passed!\r\n");
        } else {
            DebugP_log("Test Failed: Watchdog Power Hold disable failed!\r\n");
        }
    }
}

/**
 * @brief Test setting the return to Long Window enable/disable for the
 * watchdog. This function tests setting and getting the return to Long Window
 * enable/disable configuration for the watchdog.
 *
 * @param void
 * @return NULL
 */
void test_wdg_setCfg_ReturnLongWindow(void) {
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_WdgCfg_t wdgCfg_expected, wdgCfg_actual;

    checkWdgEnabled();

    wdgCfg_expected.validParams = PMIC_CFG_WDG_RETLONGWIN_VALID_SHIFT;
    wdgCfg_actual.validParams = PMIC_CFG_WDG_RETLONGWIN_VALID_SHIFT;

    /* Enable Watchdog return to Long Window */
    wdgCfg_expected.retLongWin = PMIC_WDG_RETLONGWIN_ENABLE;
    status = Pmic_wdgSetCfg(pPmicCoreHandle_wdg, wdgCfg_expected);
    if (PMIC_ST_SUCCESS == status) {
        status = Pmic_wdgGetCfg(pPmicCoreHandle_wdg, &wdgCfg_actual);
        if (wdgCfg_actual.retLongWin == wdgCfg_expected.retLongWin) {
            DebugP_log("Test Passed: Watchdog return to Long Window enable "
                       "passed!\r\n");
        } else {
            DebugP_log("Test Failed: Watchdog return to Long Window enable "
                       "failed!\r\n");
        }
    }

    /* Disable Watchdog return to Long Window */
    wdgCfg_expected.retLongWin = PMIC_WDG_RETLONGWIN_DISABLE;
    status = Pmic_wdgSetCfg(pPmicCoreHandle_wdg, wdgCfg_expected);
    if (PMIC_ST_SUCCESS == status) {
        status = Pmic_wdgGetCfg(pPmicCoreHandle_wdg, &wdgCfg_actual);
        if (wdgCfg_actual.retLongWin == wdgCfg_expected.retLongWin) {
            DebugP_log("Test Passed: Watchdog return to Long Window disable "
                       "passed!\r\n");
        } else {
            DebugP_log("Test Failed: Watchdog return to Long Window disable "
                       "failed!\r\n");
        }
    }
}

/**
 * @brief Test setting the QA feedback configuration for the watchdog.
 * This function tests setting and getting the QA feedback configuration for the
 * watchdog.
 *
 * @param void
 * @return NULL
 */
void test_wdg_setCfg_QA_feedback(void) {
    uint8_t i = 0;
    int32_t status = PMIC_ST_SUCCESS;

    Pmic_WdgCfg_t wdgCfg_expected, wdgCfg_actual;

    checkWdgEnabled();

    wdgCfg_expected.validParams = PMIC_CFG_WDG_QA_FDBK_VALID_SHIFT;
    wdgCfg_actual.validParams = PMIC_CFG_WDG_QA_FDBK_VALID_SHIFT;

    /* For each possible feedback value */
    for (i = PMIC_WDG_QA_FEEDBACK_VALUE_3; i != PMIC_WDG_QA_FEEDBACK_VALUE_0;
         i--) {
        /* Set the feedback configuration */
        wdgCfg_expected.qaFdbk = i;
        status = Pmic_wdgSetCfg(pPmicCoreHandle_wdg, wdgCfg_expected);
        if (PMIC_ST_SUCCESS == status) {
            status = Pmic_wdgGetCfg(pPmicCoreHandle_wdg, &wdgCfg_actual);
            if (wdgCfg_actual.qaFdbk == wdgCfg_expected.qaFdbk) {
                DebugP_log("Test Passed: Watchdog set feedback configuration "
                           "for QA-%u passed!\r\n",
                           i);
            } else {
                DebugP_log("Test Failed: Watchdog set feedback configuration "
                           "for QA-%u failed!\r\n",
                           i);
            }
        }
    }
}

/**
 * @brief Test setting the QA LFSR configuration for the watchdog.
 * This function tests setting and getting the QA LFSR configuration for the
 * watchdog.
 *
 * @param void
 * @return NULL
 */
void test_wdg_setCfg_QA_LFSR(void) {
    uint8_t i = 0;
    int32_t status = PMIC_ST_SUCCESS;

    Pmic_WdgCfg_t wdgCfg_expected, wdgCfg_actual;

    checkWdgEnabled();

    wdgCfg_expected.validParams = PMIC_CFG_WDG_QA_LFSR_VALID_SHIFT;
    wdgCfg_actual.validParams = PMIC_CFG_WDG_QA_LFSR_VALID_SHIFT;

    /* For each possible LFSR value */
    for (i = PMIC_WDG_QA_LFSR_VALUE_3; i != PMIC_WDG_QA_LFSR_VALUE_0; i--) {
        /* Set the LFSR configuration */
        wdgCfg_expected.qaLfsr = i;
        status = Pmic_wdgSetCfg(pPmicCoreHandle_wdg, wdgCfg_expected);
        if (PMIC_ST_SUCCESS == status) {
            status = Pmic_wdgGetCfg(pPmicCoreHandle_wdg, &wdgCfg_actual);
            if (wdgCfg_actual.qaLfsr == wdgCfg_expected.qaLfsr) {
                DebugP_log("Test Passed: Watchdog set LFSR configuration for "
                           "LFSR-%u passed!\r\n",
                           i);
            } else {
                DebugP_log("Test Failed: Watchdog set LFSR configuration for "
                           "LFSR-%u failed!\r\n",
                           i);
            }
        }
    }
}

/**
 * @brief Test setting the QA question seed configuration for the watchdog.
 * This function tests setting and getting the QA question seed configuration
 * for the watchdog.
 *
 * @param void
 * @return NULL
 */
void test_wdg_setCfg_QA_questionSeed(void) {
    uint8_t i = 0;
    int32_t status = PMIC_ST_SUCCESS;

    Pmic_WdgCfg_t wdgCfg_expected, wdgCfg_actual;

    checkWdgEnabled();

    wdgCfg_expected.validParams = PMIC_CFG_WDG_QA_QUES_SEED_VALID_SHIFT;
    wdgCfg_actual.validParams = PMIC_CFG_WDG_QA_QUES_SEED_VALID_SHIFT;

    /* For each possible question seed value */
    for (i = PMIC_WDG_QA_QUES_SEED_VALUE_15; i != PMIC_WDG_QA_QUES_SEED_VALUE_0;
         i--) {
        /* Set the question seed */
        wdgCfg_expected.qaQuesSeed = i;
        status = Pmic_wdgSetCfg(pPmicCoreHandle_wdg, wdgCfg_expected);
        if (PMIC_ST_SUCCESS == status) {
            status = Pmic_wdgGetCfg(pPmicCoreHandle_wdg, &wdgCfg_actual);
            if (wdgCfg_actual.qaQuesSeed == wdgCfg_expected.qaQuesSeed) {
                DebugP_log("Test Passed: Watchdog set Question Seed "
                           "configuration for Ques-%u passed!\r\n",
                           i);
            } else {
                DebugP_log("Test Failed: Watchdog set Question Seed "
                           "configuration for Ques-%u failed!\r\n",
                           i);
            }
        }
    }
}

/**
 * @brief Test setting the count selection bit for the watchdog.
 * This function tests setting and getting the count selection bit configuration
 * for the watchdog.
 *
 * @param void
 * @return NULL
 */
void test_wdg_setCfg_cntSel(void) {
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_WdgCfg_t wdgCfg_expected = {.validParams =
                                         PMIC_CFG_WDG_CNT_SEL_VALID_SHIFT};
    Pmic_WdgCfg_t wdgCfg_actual = {.validParams =
                                       PMIC_CFG_WDG_CNT_SEL_VALID_SHIFT};

    /* Set WD_CNT_SEL bit to 1 */
    wdgCfg_expected.cntSel = PMIC_WDG_CNT_SEL_2_1_SCHEME;
    status = Pmic_wdgSetCfg(pPmicCoreHandle_wdg, wdgCfg_expected);
    if (PMIC_ST_SUCCESS == status) {
        status = Pmic_wdgGetCfg(pPmicCoreHandle_wdg, &wdgCfg_actual);
        if (wdgCfg_actual.cntSel == wdgCfg_expected.cntSel) {
            DebugP_log("Test Passed: Watchdog set count selection bit to 1 "
                       "passed!\r\n");
        } else {
            DebugP_log("Test Failed: Watchdog set count selection bit to 1 "
                       "failed!\r\n");
        }
    }

    /* Set WD_CNT_SEL bit to 0 */
    wdgCfg_expected.cntSel = PMIC_WDG_CNT_SEL_1_1_SCHEME;
    status = Pmic_wdgSetCfg(pPmicCoreHandle_wdg, wdgCfg_expected);
    if (PMIC_ST_SUCCESS == status) {
        status = Pmic_wdgGetCfg(pPmicCoreHandle_wdg, &wdgCfg_actual);
        if (wdgCfg_actual.cntSel == wdgCfg_expected.cntSel) {
            DebugP_log("Test Passed: Watchdog set count selection bit to 0 "
                       "passed!\r\n");
        } else {
            DebugP_log("Test Failed: Watchdog set count selection bit to 0 "
                       "failed!\r\n");
        }
    }
}

/**
 * @brief Test setting the ENDRV bit for the watchdog.
 * This function tests setting and getting the ENDRV bit configuration for the
 * watchdog.
 *
 * @param void
 * @return NULL
 */
void test_wdg_setCfg_enDrvSel(void) {
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_WdgCfg_t wdgCfg_expected = {.validParams =
                                         PMIC_CFG_WDG_ENDRV_SEL_VALID_SHIFT};
    Pmic_WdgCfg_t wdgCfg_actual = {.validParams =
                                       PMIC_CFG_WDG_ENDRV_SEL_VALID_SHIFT};

    /* Set WD_ENDRV_SEL bit to 1 */
    wdgCfg_expected.enDrvSel = PMIC_WDG_ENDRV_SEL_CLR;
    status = Pmic_wdgSetCfg(pPmicCoreHandle_wdg, wdgCfg_expected);
    if (PMIC_ST_SUCCESS == status) {
        status = Pmic_wdgGetCfg(pPmicCoreHandle_wdg, &wdgCfg_actual);
        if (wdgCfg_actual.enDrvSel == wdgCfg_expected.enDrvSel) {
            DebugP_log("Test Passed: Watchdog set ENDRV bit to 1 passed!\r\n");
        } else {
            DebugP_log("Test Failed: Watchdog set ENDRV bit to 1 failed!\r\n");
        }
    }

    /* Set WD_ENDRV_SEL bit to 0 */
    wdgCfg_expected.enDrvSel = PMIC_WDG_ENDRV_SEL_NO_CLR;
    status = Pmic_wdgSetCfg(pPmicCoreHandle_wdg, wdgCfg_expected);
    if (PMIC_ST_SUCCESS == status) {
        status = Pmic_wdgGetCfg(pPmicCoreHandle_wdg, &wdgCfg_actual);
        if (wdgCfg_actual.enDrvSel == wdgCfg_expected.enDrvSel) {
            DebugP_log("Test Passed: Watchdog set ENDRV bit to 0 passed!\r\n");
        } else {
            DebugP_log("Test Failed: Watchdog set ENDRV bit to 0 failed!\r\n");
        }
    }
}

void test_wdg_QaMode_noErrors(void) {
    uint8_t answerCnt = 0;
    uint16_t numSeqeunces = 0;
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_WdgCfg_t wdgCfg = {.validParams =
                                (PMIC_CFG_WDG_LONGWINDURATION_VALID_SHIFT | PMIC_CFG_WDG_WIN1DURATION_VALID_SHIFT |
                                 PMIC_CFG_WDG_WIN2DURATION_VALID_SHIFT | PMIC_CFG_WDG_FAILTHRESHOLD_VALID_SHIFT |
                                 PMIC_CFG_WDG_RSTTHRESHOLD_VALID_SHIFT | PMIC_CFG_WDG_PWRHOLD_VALID_SHIFT |
                                 PMIC_CFG_WDG_RETLONGWIN_VALID_SHIFT | PMIC_CFG_WDG_QA_FDBK_VALID_SHIFT |
                                 PMIC_CFG_WDG_QA_LFSR_VALID_SHIFT | PMIC_CFG_WDG_QA_QUES_SEED_VALID_SHIFT),
                            .longWinDuration_ms = 5000,
                            .win1Duration_us = 70400,
                            .win2Duration_us = 70400,
                            .failThreshold = 7,
                            .rstThreshold = 7,
                            .pwrHold = true,
                            .retLongWin = true,
                            .qaFdbk = 0,
                            .qaLfsr = 0,
                            .qaQuesSeed = 0};

    checkWdgEnabled();

    /* Configure Watchdog */
    if(PMIC_ST_SUCCESS == Pmic_wdgSetCfg(pPmicCoreHandle_wdg, wdgCfg)) {
        DebugP_log("WDG Configured successfully\r\n");

        /* Begin Q&A sequences */
        if (PMIC_ST_SUCCESS == Pmic_wdgBeginSequences(pPmicCoreHandle_wdg, PMIC_WDG_QA_MODE)) {

            /* Exit Long Window by sending all 4 answer bytes */
            for (answerCnt = 4; answerCnt != 0; answerCnt--)
            {
                status += Pmic_wdgQaSequenceWriteAnswer(pPmicCoreHandle_wdg);
            }

            /* Undergo Q&A sequences */
            for (numSeqeunces = 20; numSeqeunces != 0; numSeqeunces--)
            {
                if (numSeqeunces == 1)
                {
                    wdgCfg.validParams = PMIC_CFG_WDG_RETLONGWIN_VALID_SHIFT;
                    wdgCfg.retLongWin = true;
                    status += Pmic_wdgSetCfg(pPmicCoreHandle_wdg, wdgCfg);
                }

                /* Enter Window-1; for Answer-3, Answer-2, and Answer-1,
                 calculate and send answer byte; */
                for (answerCnt = 3; answerCnt >= 1; answerCnt--)
                {
                    status += Pmic_wdgQaSequenceWriteAnswer(pPmicCoreHandle_wdg);
                    if(status == PMIC_ST_SUCCESS){
                        DebugP_log("Test Passed: Watchdog window-1 test successfull!\r\n");
                    }
                    else {
                        DebugP_log("Test Failed: Watchdog window-1 test failed!\r\n");
                    }
                }

                /* Enter Window-2; calculate and send last answer byte; check for any WDG errors */
                status = Pmic_wdgQaSequenceWriteAnswer(pPmicCoreHandle_wdg);
                if(status == PMIC_ST_SUCCESS){
                    DebugP_log("Test Passed: Watchdog window-2 test successfull!\r\n");
                }
                else {
                    DebugP_log("Test Failed: Watchdog window-2 test failed!\r\n");
                }
            }
            /* WDG has returned to Long Window; set WD_PWRHOLD so that WDG remains in Long Window */
            wdgCfg.validParams = PMIC_CFG_WDG_PWRHOLD_VALID_SHIFT;
            wdgCfg.pwrHold = true;
            if (PMIC_ST_SUCCESS == Pmic_wdgSetCfg(pPmicCoreHandle_wdg, wdgCfg)) {
                if(PMIC_ST_SUCCESS == Pmic_wdgGetCfg(pPmicCoreHandle_wdg, &wdgCfg)) {
                        if(wdgCfg.pwrHold == true) {
                            DebugP_log("Test Passed: Watchdog QA Mode test passed!\r\n");
                        } else {
                            DebugP_log("Test Failed: Watchdog QA Mode test failed!\r\n");
                        }
                }
            }
        }
    } else {
        DebugP_log("WDG Configuration failed\r\n");
    }
}

/**
 * @brief Main test function for Watchdog timer service.
 * This function serves as the main test case for watchdog timer service
 * functionality.
 *
 * @param args Pointer to the arguments (not used in this function).
 * @return NULL
 */
void *test_pmic_WDG(void *args) {
    Drivers_open();
    Board_driversOpen();

    mcspi_mux_pmic();

    /* Initialization */
    DebugP_log("Initializing...\r\n");
    test_pmic_wdg_config_init();
    DebugP_log("Initialization Completed!\r\n\n");

    /* Lock config register initially */
    DebugP_log("[INIT] Configuration Register Lock Sequence:\r\n");
    test_pmic_LockUnlock(pPmicCoreHandle_wdg, 0);

    /*API to unlock configuration register */
    DebugP_log("[INIT] Configuration Register Unlock Sequence:\r\n");
    test_pmic_LockUnlock(pPmicCoreHandle_wdg, 1);

    test_wdg_enableDisable();
    delay(1000);

    test_wdg_setCfg_longWindowDuration();
    delay(1000);

    test_wdg_setCfg_window1Duration();
    delay(1000);

    test_wdg_setCfg_window2Duration();
    delay(1000);

    test_wdg_setCfg_failThreshold();
    delay(1000);

    test_wdg_setCfg_resetThreshold();
    delay(1000);

    test_wdg_setCfg_resetEnable();
    delay(1000);

    test_wdg_setCfg_wdgMode();
    delay(1000);

    test_wdg_setCfg_powerHold();
    delay(1000);

    test_wdg_setCfg_ReturnLongWindow();
    delay(1000);

    test_wdg_setCfg_QA_feedback();
    delay(1000);

    test_wdg_setCfg_QA_LFSR();
    delay(1000);

    test_wdg_setCfg_QA_questionSeed();
    delay(1000);

    test_wdg_setCfg_cntSel();
    delay(1000);

    test_wdg_setCfg_enDrvSel();
    delay(1000);

    test_wdg_QaMode_noErrors();

    /* De-initialization */
    test_pmic_wdg_config_deinit();
    DebugP_log("\n[DE-INIT] De-initialization Completed\r\n");

    Board_driversClose();
    Drivers_close();

    return NULL;
}
