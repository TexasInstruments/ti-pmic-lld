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
 *  @file pmic_esm_test.c
 *
 *  @brief This file contains PMIC ESM specific test cases and related test
 *  APIs
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "pmic_esm_test.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

/* Pointer to Pmic Core Handle */
Pmic_CoreHandle_t *pPmicCoreHandle_esm = NULL;

/*!
 * @brief   Initialize the PMIC ESM configuration for testing.
 * This function initializes the PMIC ESM configuration for testing purposes.
 * It sets up various parameters required for ESM configuration and initializes
 * the PMIC core handle.
 *
 * @param   void
 * @return  Returns the status of the initialization process.
 */
int32_t test_pmic_esm_config_init(void) {
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

    status = test_pmic_appInit(&pPmicCoreHandle_esm, &pmicConfigData);
    if (PMIC_ST_SUCCESS != status) {
        DebugP_log("%s(): %d: FAILED with status: %d\r\n", __func__, __LINE__,
                   status);
    }
    return status;
}

/*!
 * @brief   Deinitialize the PMIC ESM configuration after testing.
 * This function deinitializes the PMIC ESM configuration after testing is
 * completed. It deinitializes the PMIC core handle and frees up any allocated
 * memory.
 *
 * @param   void
 * @return  Returns the status of the deinitialization process.
 */
int32_t test_pmic_esm_config_deinit(void) {
    int32_t status = PMIC_ST_SUCCESS;

    status = Pmic_deinit(pPmicCoreHandle_esm);
    free(pPmicCoreHandle_esm);
    SemaphoreP_destruct(&gpmicCoreObj);
    if (PMIC_ST_SUCCESS != status) {
        DebugP_log("%s(): %d: FAILED with status: %d\r\n", __func__, __LINE__,
                   status);
    }
    return status;
}

/*!
 * @brief   Test the functionality of starting and stopping ESM for MCU mode.
 * This function tests the functionality of starting and stopping the ESM for
 * MCU mode. It verifies the behavior of starting and stopping ESM and
 * enabling/disabling it.
 *
 * @param   void
 * @return  NULL
 */
static void test_pmic_esm_startEsm_esmMcuStart(void) {
    uint32_t pmicStatus = PMIC_ST_SUCCESS;
    bool esmState = PMIC_ESM_START;

    pmicStatus = Pmic_esmStart(pPmicCoreHandle_esm, esmState);
    if (PMIC_ST_SUCCESS == pmicStatus) {
        DebugP_log("Test Passed \r\n");
    } else {
        DebugP_log("Test Failed \r\n");
    }

    pmicStatus = Pmic_esmEnable(pPmicCoreHandle_esm, PMIC_ESM_ENABLE);

    if (PMIC_ST_ERR_ESM_STARTED == pmicStatus) {
        DebugP_log("Expected Result \n");
    } else {
        DebugP_log("Unexpected Result\n");
    }

    esmState = PMIC_ESM_STOP;
    pmicStatus = Pmic_esmStart(pPmicCoreHandle_esm, esmState);
    if (PMIC_ST_SUCCESS == pmicStatus) {
        DebugP_log("Test Passed \r\n");
    } else {
        DebugP_log("Test Failed \r\n");
    }

    pmicStatus = Pmic_esmEnable(pPmicCoreHandle_esm, PMIC_ESM_ENABLE);

    if (PMIC_ST_SUCCESS == pmicStatus) {
        DebugP_log("Test Passed \r\n");
    } else {
        DebugP_log("Test Failed \r\n");
    }
}

/*!
 * @brief   Parameter validation test for enabling ESM.
 * This function performs parameter validation tests for enabling ESM.
 * It checks for invalid handle and toggle parameters and ensures appropriate
 * error codes are returned.
 *
 * @param   void
 * @return  NULL
 */
static void test_pmic_esm_enableEsmPrmValTest_handle(void) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    bool esmToggle = PMIC_ESM_ENABLE;

    pmicStatus = Pmic_esmEnable(pPmicCoreHandle_esm, esmToggle);
    if (PMIC_ST_ERR_INV_HANDLE == pmicStatus) {
        DebugP_log("Expected Result \n");
    } else {
        DebugP_log("Unexpected Result\n");
    }
}

/*!
 * @brief   Parameter validation test for setting ESM HMAX value.
 * This function performs parameter validation tests for setting ESM HMAX value.
 * It checks for invalid handle and HMAX value and ensures appropriate
 * error codes are returned.
 *
 * @param   void
 * @return  NULL
 */
static void test_pmic_esm_setConfigurationPrmValTest_hmaxValue(void) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_EsmCfg_t esmCfg = {PMIC_ESM_CFG_DELAY1_VALID_SHIFT,
                            4096U,
                            2048U,
                            30U,
                            30U,
                            30U,
                            30U,
                            4U,
                            PMIC_ESM_ERR_EN_DRV_CLEAR_ENABLE,
                            PMIC_ESM_LEVEL_MODE};

    esmCfg.validParams = PMIC_ESM_CFG_HMAX_VALID_SHIFT;
    esmCfg.esmHmax_us = 3850U;
    pmicStatus = Pmic_esmSetConfiguration(pPmicCoreHandle_esm, esmCfg);
    if (PMIC_ST_ERR_INV_ESM_VAL == pmicStatus) {
        DebugP_log("Expected Result \n");
    } else {
        DebugP_log("Unexpected Result\n");
    }
}

/*!
 * @brief   Parameter validation test for setting ESM HMIN value.
 * This function performs parameter validation tests for setting ESM HMIN value.
 * It checks for invalid handle and HMIN value and ensures appropriate
 * error codes are returned.
 *
 * @param   void
 * @return  NULL
 */
static void test_pmic_esm_setConfigurationPrmValTest_hminValue(void) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_EsmCfg_t esmCfg = {PMIC_ESM_CFG_DELAY1_VALID_SHIFT,
                            4096U,
                            2048U,
                            30U,
                            30U,
                            30U,
                            30U,
                            4U,
                            PMIC_ESM_ERR_EN_DRV_CLEAR_ENABLE,
                            PMIC_ESM_LEVEL_MODE};

    esmCfg.validParams = PMIC_ESM_CFG_HMIN_VALID_SHIFT;
    esmCfg.esmHmin_us = 3850U;
    pmicStatus = Pmic_esmSetConfiguration(pPmicCoreHandle_esm, esmCfg);
    if (PMIC_ST_ERR_INV_ESM_VAL == pmicStatus) {
        DebugP_log("Expected Result \n");
    } else {
        DebugP_log("Unexpected Result\n");
    }
}

/*!
 * @brief   Parameter validation test for setting ESM LMAX value.
 * This function performs parameter validation tests for setting ESM LMAX value.
 * It checks for invalid handle and LMAX value and ensures appropriate
 * error codes are returned.
 *
 * @param   void
 * @return  NULL
 */
static void test_pmic_esm_setConfigurationPrmValTest_lmaxValue(void) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_EsmCfg_t esmCfg = {PMIC_ESM_CFG_DELAY1_VALID_SHIFT,
                            4096U,
                            2048U,
                            30U,
                            30U,
                            30U,
                            30U,
                            4U,
                            PMIC_ESM_ERR_EN_DRV_CLEAR_ENABLE,
                            PMIC_ESM_LEVEL_MODE};

    esmCfg.validParams = PMIC_ESM_CFG_LMAX_VALID_SHIFT;
    esmCfg.esmLmax_us = 3850U;
    pmicStatus = Pmic_esmSetConfiguration(pPmicCoreHandle_esm, esmCfg);
    if (PMIC_ST_ERR_INV_ESM_VAL == pmicStatus) {
        DebugP_log("Expected Result \n");
    } else {
        DebugP_log("Unexpected Result\n");
    }
}

/*!
 * @brief   Parameter validation test for setting ESM LMIN value.
 * This function performs parameter validation tests for setting ESM LMIN value.
 * It checks for invalid handle and LMIN value and ensures appropriate
 * error codes are returned.
 *
 * @param   void
 * @return  NULL
 */
static void test_pmic_esm_setConfigurationPrmValTest_lminValue(void) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_EsmCfg_t esmCfg = {PMIC_ESM_CFG_DELAY1_VALID_SHIFT,
                            4096U,
                            2048U,
                            30U,
                            30U,
                            30U,
                            30U,
                            4U,
                            PMIC_ESM_ERR_EN_DRV_CLEAR_ENABLE,
                            PMIC_ESM_LEVEL_MODE};

    esmCfg.validParams = PMIC_ESM_CFG_LMIN_VALID_SHIFT;
    esmCfg.esmLmin_us = 3850U;
    pmicStatus = Pmic_esmSetConfiguration(pPmicCoreHandle_esm, esmCfg);
    if (PMIC_ST_ERR_INV_ESM_VAL == pmicStatus) {
        DebugP_log("Expected Result \n");
    } else {
        DebugP_log("Unexpected Result\n");
    }
}

/*!
 * @brief   Parameter validation test for getting ESM configuration with valid
 * parameters. This function performs parameter validation tests for getting ESM
 * configuration with valid parameters. It checks for insufficient
 * configurations and ensures the API behavior is as expected.
 *
 * @param   void
 * @return  NULL
 */
static void test_pmic_esm_getconfiguration_PrmValTest_validParams(void) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_EsmCfg_t esmCfg_rd = {0U};

    pmicStatus =
        Pmic_esmGetConfiguration(pPmicCoreHandle_esm, &esmCfg_rd);
    if (pmicStatus == PMIC_ST_ERR_INSUFFICIENT_CFG) {
        DebugP_log("Insufficient Configurations \r\n");
    } else {
        void test_pmic_set_scratchpad();
        void test_pmic_get_scratchpad();
        void test_pmic_set_spreadSpectrum();
        void test_pmic_clear_spreadSpectrum();
        void test_pmic_get_spreadSpectrum();
        void test_pmic_deviceonbus();
        void test_pmic_get_common_stat();
        void test_pmic_get_diagout();
        void test_pmic_enable_diagout();
        void test_pmic_get_safeout_cfg();
        void test_pmic_enable_safeout_cfg();

        DebugP_log("Sufficient Configurations \r\n");
    }
}

/*!
 * @brief   Test the functionality of getting ESM error count.
 * This function tests the functionality of getting ESM error count.
 * It verifies the behavior of getting error count for SOC mode.
 *
 * @param   void
 * @return  NULL
 */
static void test_Pmic_esmGetErrCnt(void) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    ;
    uint8_t esmErrCnt = 0U;

    pmicStatus = Pmic_esmGetErrCnt(pPmicCoreHandle_esm, &esmErrCnt);

    if (PMIC_ST_ERR_INV_DEVICE == pmicStatus) {
        DebugP_log("Expected Result \n");
    } else {
        DebugP_log("Unexpected Result\n");
    }
}

/*!
 * @brief   Test the functionality of getting ESM status for MCU mode.
 * This function tests the functionality of getting ESM status for MCU mode.
 * It verifies the behavior of starting and stopping ESM and checking its
 * status.
 *
 * @param   void
 * @return  NULL
 */
static void test_pmic_esm_getStatusEsmMcu(void) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t esmState_rd;

    pmicStatus = Pmic_esmStart(pPmicCoreHandle_esm, PMIC_ESM_START);

    if (PMIC_ST_SUCCESS == pmicStatus) {
        DebugP_log("Expected Result \n");
    } else {
        DebugP_log("Unexpected Result\n");
    }

    pmicStatus = Pmic_esmEnable(pPmicCoreHandle_esm, PMIC_ESM_ENABLE);
    if (PMIC_ST_ERR_ESM_STARTED == pmicStatus) {
        DebugP_log("Expected Result \n");
    } else {
        DebugP_log("Unexpected Result\n");
    }

    pmicStatus = Pmic_esmGetStatus(pPmicCoreHandle_esm, &esmState_rd);

    if (PMIC_ST_SUCCESS == pmicStatus) {
        DebugP_log("Expected Result \n");
    } else {
        DebugP_log("Unexpected Result\n");
    }

    if (PMIC_ESM_START == esmState_rd) {
        DebugP_log("Expected Result \n");
    } else {
        DebugP_log("Unexpected Result\n");
    }

    pmicStatus = Pmic_esmStart(pPmicCoreHandle_esm, PMIC_ESM_STOP);

    if (PMIC_ST_SUCCESS == pmicStatus) {
        DebugP_log("Expected Result \n");
    } else {
        DebugP_log("Unexpected Result\n");
    }

    pmicStatus = Pmic_esmEnable(pPmicCoreHandle_esm, PMIC_ESM_ENABLE);
    if (PMIC_ST_SUCCESS == pmicStatus) {
        DebugP_log("Expected Result \n");
    } else {
        DebugP_log("Unexpected Result\n");
    }

    pmicStatus = Pmic_esmGetStatus(pPmicCoreHandle_esm, &esmState_rd);
    if (PMIC_ST_SUCCESS == pmicStatus) {
        DebugP_log("Expected Result \n");
    } else {
        DebugP_log("Unexpected Result\n");
    }

    if (PMIC_ESM_STOP == esmState_rd) {
        DebugP_log("Expected Result \n");
    } else {
        DebugP_log("Unexpected Result\n");
    }
}

/*!
 * @brief   Main function for testing PMIC ESM.
 * This function serves as the main entry point for testing PMIC ESM
 * functionality. It initializes the necessary drivers and hardware, performs
 * various tests, and then deinitializes the configuration.
 *
 * @param args Arguments passed to the function (not used).
 * @return void* Returns NULL when testing is completed.
 */
void *test_pmic_esm(void *args) {
    Drivers_open();
    Board_driversOpen();

    mcspi_mux_pmic();

    /* Initialization */
    DebugP_log("Initializing...\r\n");
    test_pmic_esm_config_init();
    DebugP_log("Initialization Completed!\r\n\n");

    /* Lock config register initially */
    DebugP_log("[INIT] Configuration Register Lock Sequence:\r\n");
    test_pmic_LockUnlock(pPmicCoreHandle_esm, 0);

    /*API to unlock configuration register */
    DebugP_log("[INIT] Configuration Register Unlock Sequence:\r\n");
    test_pmic_LockUnlock(pPmicCoreHandle_esm, 1);

    test_pmic_esm_startEsm_esmMcuStart();
    delay(1000);

    test_pmic_esm_enableEsmPrmValTest_handle();
    delay(1000);

    test_pmic_esm_setConfigurationPrmValTest_hmaxValue();
    delay(1000);

    test_pmic_esm_setConfigurationPrmValTest_hminValue();
    delay(1000);

    test_pmic_esm_setConfigurationPrmValTest_lmaxValue();
    delay(1000);

    test_pmic_esm_setConfigurationPrmValTest_lminValue();
    delay(1000);

    test_pmic_esm_getconfiguration_PrmValTest_validParams();
    delay(1000);

    test_Pmic_esmGetErrCnt();
    delay(1000);

    test_pmic_esm_getStatusEsmMcu();
    delay(1000);

    /* Deinitialization */
    test_pmic_esm_config_deinit();
    DebugP_log("Deinitialization Completed\r\n");

    Board_driversClose();
    Drivers_close();

    return NULL;
}
