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
 *  @file pmic_core_test.c
 *
 *  @brief This file contains PMIC Core specific test cases and related test
 *  APIs
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "pmic_core_test.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

/* Pointer to Pmic Core Handle */
Pmic_CoreHandle_t *pPmicCoreHandle_core = NULL;

/**
 * @brief Initializes PMIC Core configuration.
 * This function initializes the PMIC Core configuration by filling in the
 * required parameters such as PMIC device type, communication mode,
 * communication I/O functions, and critical section functions. It calls
 * test_pmic_appInit() to perform the initialization and logs any failure status
 * using DebugP_log().
 *
 * @param   void
 * @return int32_t Returns PMIC_ST_SUCCESS upon success, error code otherwise.
 */
int32_t test_pmic_core_config_init(void) {
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

    status = test_pmic_appInit(&pPmicCoreHandle_core, &pmicConfigData);
    if (PMIC_ST_SUCCESS != status) {
        DebugP_log("%s(): %d: FAILED with status: %d\r\n", __func__, __LINE__,
                   status);
    }
    return status;
}

/**
 * @brief Deinitializes PMIC Core configuration.
 * This function deinitializes the PMIC Core configuration by calling
 * Pmic_deinit() to deinitialize the PMIC and freeing the memory allocated for
 * PMIC Core handle. It also destructs the semaphore. Any failure status is
 * logged using DebugP_log().
 *
 * @param   void
 * @return int32_t Returns PMIC_ST_SUCCESS upon success, error code otherwise.
 */
int32_t test_pmic_core_config_deinit(void) {
    int32_t status = PMIC_ST_SUCCESS;

    status = Pmic_deinit(pPmicCoreHandle_core);
    free(pPmicCoreHandle_core);
    SemaphoreP_destruct(&gpmicCoreObj);
    if (PMIC_ST_SUCCESS != status) {
        DebugP_log("%s(): %d: FAILED with status: %d\r\n", __func__, __LINE__,
                   status);
    }
    return status;
}

/**
 * @brief Sets data to scratchpad registers.
 * This function sets data to the scratchpad registers using
 * Pmic_setScratchPadValue(). It logs any failure status using DebugP_log().
 *
 * @param   void
 * @return  NULL
 */
void test_pmic_set_scratchpad() {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t wdata1, wdata2, scratchPadRegId1, scratchPadRegId2;

    wdata1 = 0x38;
    wdata2 = 0x69;
    DebugP_log("Writing Data1:%x, Data2:%x\r\n", wdata1, wdata2);
    scratchPadRegId1 = PMIC_SCRATCH_PAD_REG_1;
    scratchPadRegId2 = PMIC_SCRATCH_PAD_REG_2;
    pmicStatus =
        Pmic_setScratchPadValue(pPmicCoreHandle_core, scratchPadRegId1, wdata1);
    pmicStatus =
        Pmic_setScratchPadValue(pPmicCoreHandle_core, scratchPadRegId2, wdata2);
    if (PMIC_ST_SUCCESS != pmicStatus) {
        DebugP_log("%s(): %d: FAILED with status: %d\n", __func__, __LINE__,
                   pmicStatus);
    }
}

/**
 * @brief Reads data from scratchpad registers.
 * This function reads data from the scratchpad registers using
 * Pmic_getScratchPadValue(). It logs any failure status using DebugP_log().
 *
 * @param   void
 * @return  NULL
 */
void test_pmic_get_scratchpad() {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t wdata1, wdata2, scratchPadRegId1, scratchPadRegId2;

    scratchPadRegId1 = PMIC_SCRATCH_PAD_REG_1;
    scratchPadRegId2 = PMIC_SCRATCH_PAD_REG_2;
    pmicStatus = Pmic_getScratchPadValue(pPmicCoreHandle_core, scratchPadRegId1,
                                         &wdata1);
    pmicStatus = Pmic_getScratchPadValue(pPmicCoreHandle_core, scratchPadRegId2,
                                         &wdata2);
    if (PMIC_ST_SUCCESS != pmicStatus) {
        DebugP_log("%s(): %d: FAILED with status: %d\n", __func__, __LINE__,
                   pmicStatus);
    }
    DebugP_log("Reading Data1:%x, Data2:%x\r\n\n", wdata1, wdata2);
}

/**
 * @brief Enables spread spectrum.
 * This function enables spread spectrum by setting the spread spectrum enable
 * bit in the common control configuration and calls
 * Pmic_spreadSpectrumEnable(). It logs any failure status using DebugP_log().
 *
 * @param   void
 * @return  NULL
 */
void test_pmic_set_spreadSpectrum() {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_CommonCtrlCfg_t commonCtrlCfg;
    commonCtrlCfg.spreadSpectrumEn = 1;
    DebugP_log("Enabling DRSS...\r\n");
    pmicStatus = Pmic_spreadSpectrumEnable(pPmicCoreHandle_core, commonCtrlCfg);
    if (PMIC_ST_SUCCESS != pmicStatus) {
        DebugP_log("%s(): %d: FAILED with status: %d\n", __func__, __LINE__,
                   pmicStatus);
    }
}

/**
 * @brief Disables spread spectrum.
 * This function disables spread spectrum by clearing the spread spectrum enable
 * bit in the common control configuration and calls
 * Pmic_spreadSpectrumEnable(). It logs any failure status using DebugP_log().
 *
 * @param   void
 * @return  NULL
 */
void test_pmic_clear_spreadSpectrum() {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_CommonCtrlCfg_t commonCtrlCfg;
    commonCtrlCfg.spreadSpectrumEn = 0;
    DebugP_log("Disabling DRSS...\r\n");
    pmicStatus = Pmic_spreadSpectrumEnable(pPmicCoreHandle_core, commonCtrlCfg);
    if (PMIC_ST_SUCCESS != pmicStatus) {
        DebugP_log("%s(): %d: FAILED with status: %d\n", __func__, __LINE__,
                   pmicStatus);
    }
}

/**
 * @brief Checks if spread spectrum is enabled.
 * This function checks if spread spectrum is enabled by calling
 * Pmic_getSpreadSpectrumEnable() and logs the result using DebugP_log().
 *
 * @param   void
 * @return  NULL
 */
void test_pmic_get_spreadSpectrum() {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_CommonCtrlCfg_t *commonCtrlCfg = NULL;
    DebugP_log("Checking DRSS....\r\n");
    pmicStatus =
        Pmic_getSpreadSpectrumEnable(pPmicCoreHandle_core, commonCtrlCfg);
    if (PMIC_ST_SUCCESS != pmicStatus) {
        DebugP_log("%s(): %d: FAILED with status: %d\n", __func__, __LINE__,
                   pmicStatus);
    }
    if ((commonCtrlCfg->spreadSpectrumEn) == 1) {
        DebugP_log("DRSS modulation is enabled!\r\n\n");
    } else {
        DebugP_log("DRSS modulation is disabled!\r\n");
    }
}

/**
 * @brief Checks if device is present on the bus.
 * This function checks if the device is present on the bus by calling
 * Pmic_validateDevOnBus() and logs any failure status using DebugP_log().
 *
 * @param   void
 * @return  NULL
 */
void test_pmic_deviceonbus() {
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    pmicStatus = Pmic_validateDevOnBus(pPmicCoreHandle_core);
    if (PMIC_ST_SUCCESS != pmicStatus) {
        DebugP_log("%s(): %d: FAILED with status: %d\n", __func__, __LINE__,
                   pmicStatus);
    }
}

/**
 * @brief Gets common control status.
 * This function gets the common control status using Pmic_getCommonStat() and
 * logs the status of various pins and configuration registers using
 * DebugP_log().
 *
 * @param   void
 * @return  NULL
 */
void test_pmic_get_common_stat() {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_CommonCtrlStat_t *pCommonCtrlStat = NULL;
    pmicStatus = Pmic_getCommonStat(pPmicCoreHandle_core, pCommonCtrlStat);
    if (PMIC_ST_SUCCESS != pmicStatus) {
        DebugP_log("%s(): %d: FAILED with status: %d\n", __func__, __LINE__,
                   pmicStatus);
    }
    if ((pCommonCtrlStat->nRstPin) == 1) {
        DebugP_log("NRST pin is HIGH!\r\n");
    } else {
        DebugP_log("NRST pin is LOW!\r\n");
    }
    if ((pCommonCtrlStat->safeOut1Pin) == 1) {
        DebugP_log("SAFE_OUT1 pin is HIGH!\r\n");
    } else {
        DebugP_log("SAFE_OUT1 pin is LOW!\r\n");
    }
    if ((pCommonCtrlStat->enOutPin) == 1) {
        DebugP_log("EN_OUT pin is HIGH!\r\n");
    } else {
        DebugP_log("EN_OUT pin is LOW!\r\n");
    }
    if ((pCommonCtrlStat->cfgregLockStat) == 1) {
        DebugP_log("Configuration registers are Locked!\r\n");
    } else {
        DebugP_log("Configuration registers are Unlocked!\r\n\n");
    }
}

/**
 * @brief Gets diagnostic output configuration.
 * This function gets the diagnostic output configuration using
 * Pmic_getDiagOutCtrlConfig() and logs the status of AMUX/DMUX enable/disable
 * using DebugP_log().
 *
 * @param   void
 * @return  NULL
 */
void test_pmic_get_diagout() {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_DiagOutCfgCtrl_t *pDiagOutCfgCtrl = NULL;
    DebugP_log("Checking AMUX/DMUX Enable/Disable...\r\n");
    pmicStatus =
        Pmic_getDiagOutCtrlConfig(pPmicCoreHandle_core, pDiagOutCfgCtrl);
    if (PMIC_ST_SUCCESS != pmicStatus) {
        DebugP_log("%s(): %d: FAILED with status: %d\n", __func__, __LINE__,
                   pmicStatus);
    }
    if ((pDiagOutCfgCtrl->diagOutCtrl) == 1) {
        DebugP_log("AMUX Enabled!\r\n\n");
    } else if ((pDiagOutCfgCtrl->diagOutCtrl) == 2) {
        DebugP_log("DMUX Enabled!\r\n\n");
    } else {
        DebugP_log("AMUX/DMUX not Enabled!\r\n\n");
    }
}

/**
 * @brief Enables diagnostic output.
 * This function enables diagnostic output by setting the corresponding
 * configuration in Pmic_DiagOutCfgCtrl_t structure and calls
 * Pmic_setDiagOutCtrlConfig(). It also calls test_pmic_get_diagout() to log the
 * updated configuration.
 *
 * @param   void
 * @return  NULL
 */
void test_pmic_enable_diagout() {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_DiagOutCfgCtrl_t DiagOutCfgCtrl;
    DiagOutCfgCtrl.diagOutCtrl_AMUXEn = 1;
    DiagOutCfgCtrl.diagOutCtrl_DMUXEn = 0;
    DebugP_log("Enabling AMUX...\r\n");
    pmicStatus =
        Pmic_setDiagOutCtrlConfig(pPmicCoreHandle_core, DiagOutCfgCtrl);
    if (PMIC_ST_SUCCESS != pmicStatus) {
        DebugP_log("%s(): %d: FAILED with status: %d\n", __func__, __LINE__,
                   pmicStatus);
    }
    test_pmic_get_diagout();
}

/**
 * @brief Gets SAFEOUT configuration.
 * This function gets the SAFEOUT pin configuration using
 * Pmic_getSafeOutPinCfg() and logs the status of SAFEOUT1/SAFEOUT2
 * enable/disable using DebugP_log().
 *
 * @param   void
 * @return  NULL
 */
void test_pmic_get_safeout_cfg() {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_CommonCtrlCfg_t *pCommonCtrlCfg = NULL;
    DebugP_log("Checking SAFEOUT1/SAFEOUT2 Enable/Disable...\r\n");
    pmicStatus = Pmic_getSafeOutPinCfg(pPmicCoreHandle_core, pCommonCtrlCfg);
    if (PMIC_ST_SUCCESS != pmicStatus) {
        DebugP_log("%s(): %d: FAILED with status: %d\n", __func__, __LINE__,
                   pmicStatus);
    }
    if (pCommonCtrlCfg->eNsafeOut1 == 1) {
        DebugP_log("SAFEOUT1 Enabled!\r\n");
    }
    if (pCommonCtrlCfg->eNsafeOut2 == 1) {
        DebugP_log("SAFEOUT2 Enabled!\r\n\n");
    }
}

/**
 * @brief Enables SAFEOUT configuration.
 * This function enables the SAFEOUT pin configuration by setting the
 * corresponding enable bits in Pmic_CommonCtrlCfg_t structure and calls
 * Pmic_setEnableSafeOutCfg(). It also calls test_pmic_get_safeout_cfg() to log
 * the updated configuration.
 *
 * @param   void
 * @return  NULL
 */
void test_pmic_enable_safeout_cfg() {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_CommonCtrlCfg_t commonCtrlCfg;
    commonCtrlCfg.eNsafeOut1 = 1;
    commonCtrlCfg.eNsafeOut2 = 1;
    DebugP_log("Enabling SAFEOUT1 and SAFEOUT2...\r\n");
    pmicStatus = Pmic_setEnableSafeOutCfg(pPmicCoreHandle_core, commonCtrlCfg);
    if (PMIC_ST_SUCCESS != pmicStatus) {
        DebugP_log("%s(): %d: FAILED with status: %d\n", __func__, __LINE__,
                   pmicStatus);
    }
    test_pmic_get_safeout_cfg();
}

void test_pmicDiagControl_AMUX() {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_AMUXFeatures setfeature, getFeature;

    DebugP_log("Testing Diagnostic Control AMUX...\r\n");

    /* Testing AMUX configuration - 1 */
    setfeature = FEATURE_LDO3_OUTPUT_VOLTAGE;
    pmicStatus = Pmic_setDiagAMUXFeatureCfg(pPmicCoreHandle_core, setfeature);
    if (PMIC_ST_SUCCESS != pmicStatus) {
        DebugP_log("%s(): %d: FAILED with status: %d\n", __func__, __LINE__, pmicStatus);
    }
    else
    {
        DebugP_log("Set Diagnostic Control AMUX Feature %d\r\n", setfeature);
    }
    pmicStatus = Pmic_getDiagAMUXFeatureCfg(pPmicCoreHandle_core, &getFeature);
    if (PMIC_ST_SUCCESS != pmicStatus) {
        DebugP_log("%s(): %d: FAILED with status: %d\n", __func__, __LINE__, pmicStatus);
    }
    else
    {
        DebugP_log("Retrieved Diagnostic Control AMUX Feature %d\r\n", getFeature);
        if(getFeature == setfeature) {
            DebugP_log("Test Passed: Diagnostic Control AMUX -> Expected feature: %d,\r\n Actual feature: %d\r\n", setfeature, getFeature);
        }
        else
        {
            DebugP_log("Test Failed: Diagnostic Control AMUX -> Expected feature: %d,\r\n Actual feature: %d\r\n", setfeature, getFeature);
        }
    }

    /* Testing AMUX configuration - 2 */
    setfeature = FEATURE_TEMP_SENSOR_LDO2;
    pmicStatus = Pmic_setDiagAMUXFeatureCfg(pPmicCoreHandle_core, setfeature);
    if (PMIC_ST_SUCCESS != pmicStatus) {
        DebugP_log("%s(): %d: FAILED with status: %d\n", __func__, __LINE__, pmicStatus);
    }
    else
    {
        DebugP_log("Set Diagnostic Control AMUX Feature %d\r\n", setfeature);
    }
    pmicStatus = Pmic_getDiagAMUXFeatureCfg(pPmicCoreHandle_core, &getFeature);
    if (PMIC_ST_SUCCESS != pmicStatus) {
        DebugP_log("%s(): %d: FAILED with status: %d\n", __func__, __LINE__, pmicStatus);
    }
    else
    {
        DebugP_log("Retrieved Diagnostic Control AMUX Feature %d\r\n", getFeature);
        if(getFeature == setfeature) {
            DebugP_log("Test Passed: Diagnostic Control AMUX -> Expected feature: %d,\r\n Actual feature: %d\r\n", setfeature, getFeature);
        }
        else
        {
            DebugP_log("Test Failed: Diagnostic Control AMUX -> Expected feature: %d,\r\n Actual feature: %d\r\n", setfeature, getFeature);
        }
    }
}

void test_pmicDiagControl_DMUX() {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_DMUXFeatures setfeature, getFeature;

    DebugP_log("Testing Diagnostic Control DMUX...\r\n");

    /* Testing DMUX configuration - 1 */
    setfeature = FEATURE_LDO1_DEGLITCHED_UV;
    pmicStatus = Pmic_setDiagDMUXFeatureCfg(pPmicCoreHandle_core, setfeature);
    if (PMIC_ST_SUCCESS != pmicStatus) {
        DebugP_log("%s(): %d: FAILED with status: %d\n", __func__, __LINE__, pmicStatus);
    }
    else
    {
        DebugP_log("Set Diagnostic Control DMUX Feature %d\r\n", setfeature);
    }
    pmicStatus = Pmic_getDiagDMUXFeatureCfg(pPmicCoreHandle_core, &getFeature);
    if (PMIC_ST_SUCCESS != pmicStatus) {
        DebugP_log("%s(): %d: FAILED with status: %d\n", __func__, __LINE__, pmicStatus);
    }
    else
    {
        DebugP_log("Retrieved Diagnostic Control DMUX Feature %d\r\n", getFeature);
        if(getFeature == setfeature) {
            DebugP_log("Test Passed: Diagnostic Control DMUX -> Expected feature: %d,\r\n Actual feature: %d\r\n", setfeature, getFeature);
        }
        else
        {
            DebugP_log("Test Failed: Diagnostic Control DMUX -> Expected feature: %d,\r\n Actual feature: %d\r\n", setfeature, getFeature);
        }
    }

    /* Testing DMUX configuration - 2 */
    setfeature = FEATURE_LDO1_BYPASS_ENABLE;
    pmicStatus = Pmic_setDiagDMUXFeatureCfg(pPmicCoreHandle_core, setfeature);
    if (PMIC_ST_SUCCESS != pmicStatus) {
        DebugP_log("%s(): %d: FAILED with status: %d\n", __func__, __LINE__, pmicStatus);
    }
    else
    {
        DebugP_log("Set Diagnostic Control DMUX Feature %d\r\n", setfeature);
    }
    pmicStatus = Pmic_getDiagDMUXFeatureCfg(pPmicCoreHandle_core, &getFeature);
    if (PMIC_ST_SUCCESS != pmicStatus) {
        DebugP_log("%s(): %d: FAILED with status: %d\n", __func__, __LINE__, pmicStatus);
    }
    else
    {
        DebugP_log("Retrieved Diagnostic Control DMUX Feature %d\r\n", getFeature);
        if(getFeature == setfeature) {
            DebugP_log("Test Passed: Diagnostic Control DMUX -> Expected feature: %d,\r\n Actual feature: %d\r\n", setfeature, getFeature);
        }
        else
        {
            DebugP_log("Test Failed: Diagnostic Control DMUX -> Expected feature: %d,\r\n Actual feature: %d\r\n", setfeature, getFeature);
        }
    }
}

/**
 * @brief Main function for PMIC Core test.
 * This function serves as the main entry point for testing PMIC Core
 * functionalities. It initializes the necessary drivers, performs PMIC Core
 * configuration initialization, executes various test functions, and then
 * deinitializes the PMIC Core configuration before closing the drivers. Debug
 * logs are provided at each step to indicate the progress and any failures
 * encountered.
 *
 * @param args Arguments to the function.
 * @return void* Returns NULL upon completion.
 */
void *test_pmic_core(void *args) {
    Drivers_open();
    Board_driversOpen();

    mcspi_mux_pmic();

    /* Initialization */
    DebugP_log("Initializing...\r\n");
    test_pmic_core_config_init();
    DebugP_log("Initialization Completed!\r\n\n");

    /* Lock config register initially */
    DebugP_log("[INIT] Configuration Register Lock Sequence:\r\n");
    test_pmic_LockUnlock(pPmicCoreHandle_core, 0);

    /*API to unlock configuration register */
    DebugP_log("[INIT] Configuration Register Unlock Sequence:\r\n");
    test_pmic_LockUnlock(pPmicCoreHandle_core, 1);

    /* Write to scratch pad */
    DebugP_log("Testing Scratch Pad:\r\n");
    test_pmic_set_scratchpad();

    /*Read from scratch pad */
    test_pmic_get_scratchpad();

    DebugP_log("Testing DRSS:\r\n");
    test_pmic_clear_spreadSpectrum();

    test_pmic_get_spreadSpectrum();

    test_pmic_set_spreadSpectrum();

    test_pmic_get_spreadSpectrum();

    test_pmic_deviceonbus();

    test_pmic_get_common_stat();

    DebugP_log("Testing AMUX/DMUX:\r\n");
    test_pmic_enable_diagout();

    DebugP_log("Testing SAFEOUT_EN\r\n");
    test_pmic_enable_safeout_cfg();

    test_pmicDiagControl_AMUX();

    test_pmicDiagControl_DMUX();

    /* Deinitialization */
    test_pmic_core_config_deinit();
    DebugP_log("Deinitialization Completed\r\n");

    Board_driversClose();
    Drivers_close();

    return NULL;
}
