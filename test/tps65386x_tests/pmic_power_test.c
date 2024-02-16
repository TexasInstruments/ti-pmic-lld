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
 *  @file   pmic_power_test.c
 *
 *  @brief  PMIC Unit Test for testing PMIC POWER APIs
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "pmic_power_test.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

/* Pointer to Pmic Core Handle */
Pmic_CoreHandle_t *pPmicCoreHandle_power = NULL;

/*!
 * @brief   Initialize the PMIC Power configuration for testing.
 * This function initializes the PMIC Power configuration for testing purposes.
 * It sets up various parameters required for Power configuration and initializes
 * the PMIC core handle.
 *
 * @param   void
 * @return  Returns the status of the initialization process.
 */
int32_t test_pmic_power_config_init(void)
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

    status = test_pmic_appInit(&pPmicCoreHandle_power, &pmicConfigData);
    if (PMIC_ST_SUCCESS != status)
    {
        DebugP_log("%s(): %d: FAILED with status: %d\r\n",
                 __func__, __LINE__, status);
    }
    return status;
}

/*!
 * @brief   Deinitialize the PMIC Power configuration after testing.
 * This function deinitializes the PMIC Power configuration after testing is completed.
 * It deinitializes the PMIC core handle and frees up any allocated memory.
 *
 * @param   void
 * @return  Returns the status of the deinitialization process.
 */
int32_t test_pmic_power_config_deinit(void)
{
    int32_t status = PMIC_ST_SUCCESS;

    status = Pmic_deinit(pPmicCoreHandle_power);
    free(pPmicCoreHandle_power);
    SemaphoreP_destruct(&gpmicCoreObj);
    if (PMIC_ST_SUCCESS != status)
    {
        DebugP_log("%s(): %d: FAILED with status: %d\r\n",
                 __func__, __LINE__,  status);
    }
    return status;
}

/**
 * @brief Test function for setting and getting Buck-Boost PGOOD configuration.
 * This function tests the functionality of setting and getting the Buck-Boost PGOOD configuration.
 * It sets the Buck-Boost OV and UV status to be included and not included in PGOOD, respectively,
 * and verifies the configuration by retrieving and comparing the actual configuration with the expected one.
 *
 * @param void
 * @return void
 */
void test_power_setBuckBst_PGood_Cfg(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_powerBuckBoostCfgReg_t buckBstCfg_expected, buckBstCfg_actual;
    Pmic_powerRsrcCfg_t pwrRsrcCfg;
    Pmic_powerRsrcRegCfg_t pwrRsrcRegCfg;

    pwrRsrcRegCfg.buckConfigRegAddr = PMIC_BUCK_BST_CFG_REGADDR;

    pwrRsrcCfg.pmicConfigShiftVal = BB_PGOOD_CFG_SHIFT;
    pwrRsrcCfg.pmicConfigMaskVal = BB_PGOOD_CFG_MASK;

    /* Set Buck Boost OV and UV status be included in PGOOD */
    buckBstCfg_expected.bbPgoodCfg = PMIC_TPS65386X_BB_CFG_STATUS_INCLUDED;
    status = Pmic_powerSetBuckBstCfg(pPmicCoreHandle_power, &buckBstCfg_expected, &pwrRsrcRegCfg, &pwrRsrcCfg);
    if(PMIC_ST_SUCCESS == status)
    {
        status = Pmic_powerGetBuckBstCfg(pPmicCoreHandle_power, &buckBstCfg_actual, &pwrRsrcRegCfg, &pwrRsrcCfg);
        if(buckBstCfg_expected.bbPgoodCfg == buckBstCfg_actual.bbPgoodCfg)
        {
            DebugP_log("Test Passed: Buck-Boost OV and UV status included in PGOOD set/get configuration passed!\r\n");
        }
        else
        {
            DebugP_log("Test Failed: Buck-Boost OV and UV status included in PGOOD set/get configuration failed!\r\n");
        }
    }

    /* Set Buck Boost OV and UV status be not included in PGOOD */
    buckBstCfg_expected.bbPgoodCfg = PMIC_TPS65386X_BB_CFG_STATUS_NOT_INCLUDED;
    status = Pmic_powerSetBuckBstCfg(pPmicCoreHandle_power, &buckBstCfg_expected, &pwrRsrcRegCfg, &pwrRsrcCfg);
    if(PMIC_ST_SUCCESS == status)
    {
        status = Pmic_powerGetBuckBstCfg(pPmicCoreHandle_power, &buckBstCfg_actual, &pwrRsrcRegCfg, &pwrRsrcCfg);
        if(buckBstCfg_expected.bbPgoodCfg == buckBstCfg_actual.bbPgoodCfg)
        {
            DebugP_log("Test Passed: Buck-Boost OV and UV status not included in PGOOD set/get configuration passed!\r\n");
        }
        else
        {
            DebugP_log("Test Failed: Buck-Boost OV and UV status not included in PGOOD set/get configuration failed!\r\n");
        }
    }
}

/**
 * @brief Test function for setting and getting Buck-Boost SS Enable configuration.
 * This function tests the functionality of setting and getting the Buck-Boost SS Enable configuration.
 * It sets the Buck-Boost DRSS to be enabled and disabled, respectively,
 * and verifies the configuration by retrieving and comparing the actual configuration with the expected one.
 *
 * @param void
 * @return void
 */
void test_power_setBuckBst_SSEn_Cfg(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_powerBuckBoostCfgReg_t buckBstCfg_expected, buckBstCfg_actual;
    Pmic_powerRsrcCfg_t pwrRsrcCfg;
    Pmic_powerRsrcRegCfg_t pwrRsrcRegCfg;

    pwrRsrcRegCfg.buckConfigRegAddr = PMIC_BUCK_BST_CFG_REGADDR;

    pwrRsrcCfg.pmicConfigShiftVal = BB_SS_EN_SHIFT;
    pwrRsrcCfg.pmicConfigMaskVal = BB_SS_EN_MASK;

    /* Set Buck Boost DRSS Enabled */
    buckBstCfg_expected.bbSsEn = PMIC_TPS65386X_BB_CFG_DRSS_ENABLED;
    status = Pmic_powerSetBuckBstCfg(pPmicCoreHandle_power, &buckBstCfg_expected, &pwrRsrcRegCfg, &pwrRsrcCfg);
    if(PMIC_ST_SUCCESS == status)
    {
        status = Pmic_powerGetBuckBstCfg(pPmicCoreHandle_power, &buckBstCfg_actual, &pwrRsrcRegCfg, &pwrRsrcCfg);
        if(buckBstCfg_expected.bbSsEn == buckBstCfg_actual.bbSsEn)
        {
            DebugP_log("Test Passed: Buck-Boost DRSS Enable passed!\r\n");
        }
        else
        {
            DebugP_log("Test Failed: Buck-Boost DRSS Enable failed!\r\n");
        }
    }

    /* Set Buck Boost DRSS Disabled */
    buckBstCfg_expected.bbSsEn = PMIC_TPS65386X_BB_CFG_DRSS_DISABLED;
    status = Pmic_powerSetBuckBstCfg(pPmicCoreHandle_power, &buckBstCfg_expected, &pwrRsrcRegCfg, &pwrRsrcCfg);
    if(PMIC_ST_SUCCESS == status)
    {
        status = Pmic_powerGetBuckBstCfg(pPmicCoreHandle_power, &buckBstCfg_actual, &pwrRsrcRegCfg, &pwrRsrcCfg);
        if(buckBstCfg_expected.bbSsEn == buckBstCfg_actual.bbSsEn)
        {
            DebugP_log("Test Passed: Buck-Boost DRSS Disable passed!\r\n");
        }
        else
        {
            DebugP_log("Test Failed: Buck-Boost DRSS Disable failed!\r\n");
        }
    }
}

/**
 * @brief Test function for setting and getting Buck-Boost Standby Level configuration.
 * This function tests the functionality of setting and getting the Buck-Boost Standby Level configuration.
 * It sets the Buck-Boost Standby Level to different voltage levels and same as the level config, respectively,
 * and verifies the configuration by retrieving and comparing the actual configuration with the expected one.
 *
 * @param void
 * @return void
 */
void test_power_setBuckBst_StbyLvl_Cfg(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_powerBuckBoostCfgReg_t buckBstCfg_expected, buckBstCfg_actual;
    Pmic_powerRsrcCfg_t pwrRsrcCfg;
    Pmic_powerRsrcRegCfg_t pwrRsrcRegCfg;

    pwrRsrcRegCfg.buckConfigRegAddr = PMIC_BUCK_BST_CFG_REGADDR;

    pwrRsrcCfg.pmicConfigShiftVal = BB_STBY_LVL_CFG_SHIFT;
    pwrRsrcCfg.pmicConfigMaskVal = BB_STBY_LVL_CFG_MASK;

    /* Set Buck Boost Standby Level to 4V */
    buckBstCfg_expected.bbPgoodCfg = PMIC_TPS65386X_BB_CFG_4V;
    status = Pmic_powerSetBuckBstCfg(pPmicCoreHandle_power, &buckBstCfg_expected, &pwrRsrcRegCfg, &pwrRsrcCfg);
    if(PMIC_ST_SUCCESS == status)
    {
        status = Pmic_powerGetBuckBstCfg(pPmicCoreHandle_power, &buckBstCfg_actual, &pwrRsrcRegCfg, &pwrRsrcCfg);
        if(buckBstCfg_expected.bbPgoodCfg == buckBstCfg_actual.bbPgoodCfg)
        {
            DebugP_log("Test Passed: Buck-Boost Standby level to 4V passed!\r\n");
        }
        else
        {
            DebugP_log("Test Failed: Buck-Boost Standby level to 4V failed!\r\n");
        }
    }

    /* Set Buck Boost Standby Level to same as level config */
    buckBstCfg_expected.bbPgoodCfg = PMIC_TPS65386X_BB_CFG_SAME_BBCFG;
    status = Pmic_powerSetBuckBstCfg(pPmicCoreHandle_power, &buckBstCfg_expected, &pwrRsrcRegCfg, &pwrRsrcCfg);
    if(PMIC_ST_SUCCESS == status)
    {
        status = Pmic_powerGetBuckBstCfg(pPmicCoreHandle_power, &buckBstCfg_actual, &pwrRsrcRegCfg, &pwrRsrcCfg);
        if(buckBstCfg_expected.bbPgoodCfg == buckBstCfg_actual.bbPgoodCfg)
        {
            DebugP_log("Test Passed: Buck-Boost Standby level to same as level config passed!\r\n");
        }
        else
        {
            DebugP_log("Test Failed: Buck-Boost Standby level to same as level config failed!\r\n");
        }
    }
}

/**
 * @brief Test function for setting and getting Buck-Boost Level configuration.
 * This function tests the functionality of setting and getting the Buck-Boost Level configuration.
 * It sets the Buck-Boost Level config to different voltage levels, such as 4.3V, 5V, and 6V, respectively,
 * and verifies the configuration by retrieving and comparing the actual configuration with the expected one.
 *
 * @param void
 * @return void
 */
void test_power_setBuckBst_Level_Cfg(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_powerBuckBoostCfgReg_t buckBstCfg_expected, buckBstCfg_actual;
    Pmic_powerRsrcCfg_t pwrRsrcCfg;
    Pmic_powerRsrcRegCfg_t pwrRsrcRegCfg;

    pwrRsrcRegCfg.buckConfigRegAddr = PMIC_BUCK_BST_CFG_REGADDR;

    pwrRsrcCfg.pmicConfigShiftVal = BB_LVL_CFG_SHIFT;
    pwrRsrcCfg.pmicConfigMaskVal = BB_LVL_CFG_MASK;

    /* Set Buck Boost Level config to 4.3V */
    buckBstCfg_expected.bbPgoodCfg = PMIC_TPS65386X_BB_CFG_BB_LVL_CFG_4_3V;
    status = Pmic_powerSetBuckBstCfg(pPmicCoreHandle_power, &buckBstCfg_expected, &pwrRsrcRegCfg, &pwrRsrcCfg);
    if(PMIC_ST_SUCCESS == status)
    {
        status = Pmic_powerGetBuckBstCfg(pPmicCoreHandle_power, &buckBstCfg_actual, &pwrRsrcRegCfg, &pwrRsrcCfg);
        if(buckBstCfg_expected.bbPgoodCfg == buckBstCfg_actual.bbPgoodCfg)
        {
            DebugP_log("Test Passed: Buck-Boost level config set to 4.3V passed!\r\n");
        }
        else
        {
            DebugP_log("Test Failed: Buck-Boost level config set to 4.3V failed!\r\n");
        }
    }

    /* Set Buck Boost Level config to 5V */
    buckBstCfg_expected.bbPgoodCfg = PMIC_TPS65386X_BB_CFG_BB_LVL_CFG_5V;
    status = Pmic_powerSetBuckBstCfg(pPmicCoreHandle_power, &buckBstCfg_expected, &pwrRsrcRegCfg, &pwrRsrcCfg);
    if(PMIC_ST_SUCCESS == status)
    {
        status = Pmic_powerGetBuckBstCfg(pPmicCoreHandle_power, &buckBstCfg_actual, &pwrRsrcRegCfg, &pwrRsrcCfg);
        if(buckBstCfg_expected.bbPgoodCfg == buckBstCfg_actual.bbPgoodCfg)
        {
            DebugP_log("Test Passed: Buck-Boost level config set to 5V passed!\r\n");
        }
        else
        {
            DebugP_log("Test Failed: Buck-Boost level config set to 5V failed!\r\n");
        }
    }

    /* Set Buck Boost Level config to 6V */
    buckBstCfg_expected.bbPgoodCfg = PMIC_TPS65386X_BB_CFG_BB_LVL_CFG_6V;
    status = Pmic_powerSetBuckBstCfg(pPmicCoreHandle_power, &buckBstCfg_expected, &pwrRsrcRegCfg, &pwrRsrcCfg);
    if(PMIC_ST_SUCCESS == status)
    {
        status = Pmic_powerGetBuckBstCfg(pPmicCoreHandle_power, &buckBstCfg_actual, &pwrRsrcRegCfg, &pwrRsrcCfg);
        if(buckBstCfg_expected.bbPgoodCfg == buckBstCfg_actual.bbPgoodCfg)
        {
            DebugP_log("Test Passed: Buck-Boost level config set to 6V passed!\r\n");
        }
        else
        {
            DebugP_log("Test Failed: Buck-Boost level config set to 6V failed!\r\n");
        }
    }
}

/**
 * @brief Main test function for power management.
 * This function serves as the main test case for power management functionality.
 * It executes a sequence of tests including setting and getting various Buck-Boost configurations,
 * and performs initialization and de-initialization procedures before and after the tests.
 *
 * @param args Pointer to the arguments (not used in this function).
 * @return NULL
 */
void *test_pmic_power(void *args)
{
  Drivers_open();
  Board_driversOpen();

  mcspi_mux_pmic();

  /* Initialization */
  DebugP_log("Initializing...\r\n");
  test_pmic_power_config_init();
  DebugP_log("Initialization Completed!\r\n\n");

  /* Lock config register initially */
  DebugP_log("[INIT] Configuration Register Lock Sequence:\r\n");
  test_pmic_LockUnlock(pPmicCoreHandle_power, 0);

  /*API to unlock configuration register */
  DebugP_log("[INIT] Configuration Register Unlock Sequence:\r\n");
  test_pmic_LockUnlock(pPmicCoreHandle_power, 1);

  test_power_setBuckBst_PGood_Cfg();
  delay(1000);

  test_power_setBuckBst_SSEn_Cfg();
  delay(1000);

  test_power_setBuckBst_StbyLvl_Cfg();
  delay(1000);

  test_power_setBuckBst_Level_Cfg();
  delay(1000);

  /* De-initialization */
  test_pmic_power_config_deinit();
  DebugP_log("\n[DE-INIT] De-initialization Completed\r\n");

  Board_driversClose();
  Drivers_close();

  return NULL;
}
