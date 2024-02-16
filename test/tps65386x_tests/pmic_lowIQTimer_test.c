/****************************************************************************
 *  Copyright (C) 2024 Texas Instruments Incorporated
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
 ****************************************************************************/

/**
 *  @file  pmic_lowIQTimer_test.c
 *
 *  @brief  This file contains all the testing related files APIs for the PMIC core
 *          and basic register lock/unlock and other miscellaneous tests.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include "pmic_lowIQTimer_test.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Definitions                             */
/* ========================================================================== */

/* Pointer to Pmic Core Handle */
Pmic_CoreHandle_t *pPmicCoreHandle_lowiqtimer = NULL;

/*!
 * @brief   Initialize the PMIC Low IQ Timer configuration for testing.
 * This function initializes the PMIC Low IQ Timer configuration for testing purposes.
 * It sets up various parameters required for Low IQ Timer configuration and initializes
 * the PMIC core handle.
 *
 * @param   void
 * @return  Returns the status of the initialization process.
 */

int32_t test_pmic_low_iq_timer_config_init(void)
{
    int32_t status                = PMIC_ST_SUCCESS;
    Pmic_CoreCfg_t pmicConfigData = {0U};

    /* Fill parameters to pmicConfigData */
    pmicConfigData.pmicDeviceType      = PMIC_DEV_BB_TPS65386X;
    pmicConfigData.validParams        |= PMIC_CFG_DEVICE_TYPE_VALID_SHIFT;

    pmicConfigData.commMode            = PMIC_INTF_SPI;
    pmicConfigData.validParams        |= PMIC_CFG_COMM_MODE_VALID_SHIFT;

    pmicConfigData.pFnPmicCommIoRead    = test_pmic_regRead;
    pmicConfigData.validParams         |= PMIC_CFG_COMM_IO_RD_VALID_SHIFT;

    pmicConfigData.pFnPmicCommIoWrite   = test_pmic_regWrite;
    pmicConfigData.validParams         |= PMIC_CFG_COMM_IO_WR_VALID_SHIFT;

    pmicConfigData.pFnPmicCritSecStart  = test_pmic_criticalSectionStartFn;
    pmicConfigData.validParams         |= PMIC_CFG_CRITSEC_START_VALID_SHIFT;

    pmicConfigData.pFnPmicCritSecStop   = test_pmic_criticalSectionStopFn;
    pmicConfigData.validParams         |= PMIC_CFG_CRITSEC_STOP_VALID_SHIFT;

    status = test_pmic_appInit(&pPmicCoreHandle_lowiqtimer, &pmicConfigData);
    if (PMIC_ST_SUCCESS != status)
    {
        DebugP_log("%s(): %d: FAILED with status: %d\r\n",
                 __func__, __LINE__, status);
    }
    return status;
}

/*!
 * @brief   Deinitialize the PMIC Low IQ Timer configuration after testing.
 * This function deinitializes the PMIC Low IQ Timer configuration after testing is completed.
 * It deinitializes the PMIC core handle and frees up any allocated memory.
 *
 * @param   void
 * @return  Returns the status of the deinitialization process.
 */
int32_t test_pmic_low_iq_timer_config_deinit(void)
{
    int32_t status = PMIC_ST_SUCCESS;

    status = Pmic_deinit(pPmicCoreHandle_lowiqtimer);
    free(pPmicCoreHandle_lowiqtimer);
    SemaphoreP_destruct(&gpmicCoreObj);
    if (PMIC_ST_SUCCESS != status)
    {
        DebugP_log("%s(): %d: FAILED with status: %d\r\n",
                 __func__, __LINE__,  status);
    }
    return status;
}

/**
 * @brief Test function to set timer configuration.
 * This function sets the configuration for the low IQ timer and logs
 * the configured value for verification.
 *
 * @param tmr_cfg The configuration value for the timer.
 * @return NULL
 */
void test_pmic_set_tmrconfig(uint32_t tmr_cfg)
{
    int32_t pmicStatus  = PMIC_ST_SUCCESS;
    pmicStatus = Pmic_SetTimerConfig(pPmicCoreHandle_lowiqtimer, tmr_cfg);
    if (PMIC_ST_SUCCESS != pmicStatus)
    {
        DebugP_log("%s(): %d: FAILED with status: %d\n",
                 __func__, __LINE__,  pmicStatus);
    }
    DebugP_log("Timer Config : %d successfully!!!\r\n",tmr_cfg);
}

/**
 * @brief Test function to retrieve timer configuration.
 * This function retrieves the configured timer configuration and logs
 * the value for verification.
 *
 * @param void
 * @return NULL
 */
void test_pmic_get_tmrconfig()
{
    int32_t pmicStatus  = PMIC_ST_SUCCESS;
    uint8_t *tmrConfigData = 0;
    pmicStatus = Pmic_GetTimerConfig(pPmicCoreHandle_lowiqtimer, tmrConfigData);
    if (PMIC_ST_SUCCESS != pmicStatus)
    {
        DebugP_log("%s(): %d: FAILED with status: %d\n",
                 __func__, __LINE__,  pmicStatus);
    }
    DebugP_log("Timer Config : %d retrieved!!!\r\n",tmrConfigData);
}

/**
 * @brief Test function to set timer prescale.
 * This function sets the prescale value for the low IQ timer.
 *
 * @param void
 * @return NULL
 */
void test_pmic_set_timerprescale()
{
    int32_t pmicStatus  = PMIC_ST_SUCCESS;
    uint8_t tmrData =  0;
    pmicStatus = Pmic_SetTimerPrescale(pPmicCoreHandle_lowiqtimer, tmrData);
    if (PMIC_ST_SUCCESS != pmicStatus)
    {
        DebugP_log("%s(): %d: FAILED with status: %d\n",
                 __func__, __LINE__,  pmicStatus);
    }
}

/**
 * @brief Test function to retrieve timer prescale.
 * This function retrieves the configured timer prescale value.
 *
 * @param void
 * @return NULL
 */
void test_pmic_get_timerprescale()
{
    int32_t pmicStatus  = PMIC_ST_SUCCESS;
    uint8_t *tmrConfigData = 0;
    pmicStatus = Pmic_GetTimerPrescale(pPmicCoreHandle_lowiqtimer, tmrConfigData);
    if (PMIC_ST_SUCCESS != pmicStatus)
    {
        DebugP_log("%s(): %d: FAILED with status: %d\n",
                 __func__, __LINE__,  pmicStatus);
    }
}

/**
 * @brief Test function to clear the timer.
 * This function clears the low IQ timer.
 *
 * @param void
 * @return NULL
 */
void test_pmic_timer_clear()
{
    int32_t pmicStatus  = PMIC_ST_SUCCESS;
    pmicStatus = Pmic_TimerClear(pPmicCoreHandle_lowiqtimer);
    if (PMIC_ST_SUCCESS != pmicStatus)
    {
        DebugP_log("%s(): %d: FAILED with status: %d\n",
                 __func__, __LINE__,  pmicStatus);
    }
}

/**
 * @brief Test function to set timer counter.
 * This function sets the counter value for the low IQ timer and logs
 * the configured value for verification.
 *
 * @param void
 * @return NULL
 */
void test_pmic_set_timer_cnt()
{
    int32_t pmicStatus  = PMIC_ST_SUCCESS;
    uint8_t tmrCntData = 25;
    pmicStatus = Pmic_SetTimerCounter0(pPmicCoreHandle_lowiqtimer, tmrCntData);
    DebugP_log("Timer Counter0 : %d\r\n", tmrCntData);
    if (PMIC_ST_SUCCESS != pmicStatus)
    {
        DebugP_log("%s(): %d: FAILED with status: %d\n",
                 __func__, __LINE__,  pmicStatus);
    }
}

/**
 * @brief Test function to retrieve timer counter.
 * This function retrieves the configured timer counter value and logs
 * the value for verification.
 *
 * @param void
 * @return NULL
 */
void test_pmic_get_timer_cnt()
{
    int32_t pmicStatus  = PMIC_ST_SUCCESS;
    uint8_t *tmrCntData = 0;
    pmicStatus = Pmic_GetTimerCounter0(pPmicCoreHandle_lowiqtimer, tmrCntData);
    DebugP_log("Timer Counter0 : %d\r\n", tmrCntData);
    if (PMIC_ST_SUCCESS != pmicStatus)
    {
        DebugP_log("%s(): %d: FAILED with status: %d\n",
                 __func__, __LINE__,  pmicStatus);
    }
}

/**
 * @brief Test function for low IQ timer.
 * This function serves as the main test case for the low IQ timer functionality.
 * It executes a sequence of operations including retrieving timer counter and configuration,
 * setting timer counter and configuration, and again retrieving timer counter and configuration
 * for verification purposes.
 *
 * @param void
 * @return NULL
 */
void test_pmic_low_iq_timer()
{
    test_pmic_get_timer_cnt();
    test_pmic_get_tmrconfig();

    test_pmic_set_timer_cnt();
    test_pmic_set_tmrconfig(PMIC_TMR_CFG4);

    test_pmic_get_timer_cnt();
    test_pmic_get_tmrconfig();
}

/**
 * @brief Main test function for low IQ timer.
 * This function initializes the necessary drivers and configurations for
 * testing the low IQ timer functionality. It performs various tests related
 * to timer configuration, setting timer counter, and retrieving timer values.
 *
 * @param args Unused argument
 * @return NULL
 */
void *test_pmic_lowIQTimer(void *args)
{
    Drivers_open();
    Board_driversOpen();

    mcspi_mux_pmic();

    /* Initialization */
    DebugP_log("Initializing...\r\n");
    test_pmic_low_iq_timer_config_init();
    DebugP_log("Initialization Completed!\r\n\n");

    /* Lock config register initially */
    DebugP_log("[INIT] Configuration Register Lock Sequence:\r\n");
    test_pmic_LockUnlock(pPmicCoreHandle_lowiqtimer, 0);

    /*API to unlock configuration register */
    DebugP_log("[INIT] Configuration Register Unlock Sequence:\r\n");
    test_pmic_LockUnlock(pPmicCoreHandle_lowiqtimer, 1);

    /* Lock counter register initially */
    DebugP_log("[INIT] TC and RC Lock Sequence:\r\n");
    test_pmic_CNT_LockUnlock(pPmicCoreHandle_lowiqtimer, 0);

    /* Unlock counter register initially */
    DebugP_log("[INIT] TC and RC Unlock Sequence:\r\n");
    test_pmic_CNT_LockUnlock(pPmicCoreHandle_lowiqtimer, 1);

    /* Low IQ Timer Test Cases */
    DebugP_log("[TEST] :\r\n");
    test_pmic_low_iq_timer();

    /* De-initialization */
    test_pmic_low_iq_timer_config_deinit();
    DebugP_log("\n[DE-INIT] De-initialization Completed\r\n");

    Board_driversClose();
    Drivers_close();

    return NULL;
}
