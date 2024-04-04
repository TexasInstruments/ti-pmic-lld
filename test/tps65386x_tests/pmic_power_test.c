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
 * It sets up various parameters required for Power configuration and
 * initializes the PMIC core handle.
 *
 * @param   void
 * @return  Returns the status of the initialization process.
 */
int32_t test_pmic_power_config_init(void) {
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

  status = test_pmic_appInit(&pPmicCoreHandle_power, &pmicConfigData);
  if (PMIC_ST_SUCCESS != status) {
    DebugP_log("%s(): %d: FAILED with status: %d\r\n", __func__, __LINE__,
               status);
  }
  return status;
}

/*!
 * @brief   Deinitialize the PMIC Power configuration after testing.
 * This function deinitializes the PMIC Power configuration after testing is
 * completed. It deinitializes the PMIC core handle and frees up any allocated
 * memory.
 *
 * @param   void
 * @return  Returns the status of the deinitialization process.
 */
int32_t test_pmic_power_config_deinit(void) {
  int32_t status = PMIC_ST_SUCCESS;

  status = Pmic_deinit(pPmicCoreHandle_power);
  free(pPmicCoreHandle_power);
  SemaphoreP_destruct(&gpmicCoreObj);
  if (PMIC_ST_SUCCESS != status) {
    DebugP_log("%s(): %d: FAILED with status: %d\r\n", __func__, __LINE__,
               status);
  }
  return status;
}

/**
 * @brief Test function for setting and getting Buck-Boost PGOOD configuration.
 * This function tests the functionality of setting and getting the Buck-Boost
 * PGOOD configuration. It sets the Buck-Boost OV and UV status to be included
 * and not included in PGOOD, respectively, and verifies the configuration by
 * retrieving and comparing the actual configuration with the expected one.
 *
 * @param void
 * @return void
 */
void test_power_setBuckBst_PGood_Cfg(void) {
  int32_t status = PMIC_ST_SUCCESS;
  Pmic_powerBuckBoostCfgReg_t buckBstCfg_expected, buckBstCfg_actual;
  Pmic_powerRsrcCfg_t pwrRsrcCfg;
  Pmic_powerRsrcRegCfg_t pwrRsrcRegCfg;

  pwrRsrcRegCfg.buckConfigRegAddr = PMIC_POWER_BUCK_CFG_REGADDR;

  pwrRsrcCfg.pmicConfigShiftVal = BB_PGOOD_CFG_SHIFT;
  pwrRsrcCfg.pmicConfigMaskVal = BB_PGOOD_CFG_MASK;

  /* Set Buck Boost OV and UV status be included in PGOOD */
  buckBstCfg_expected.bbPgoodCfg = PMIC_BB_CFG_STATUS_INCLUDED;
  status = Pmic_powerSetBuckBstCfg(pPmicCoreHandle_power, &buckBstCfg_expected,
                                   &pwrRsrcRegCfg, &pwrRsrcCfg);
  if (PMIC_ST_SUCCESS == status) {
    status = Pmic_powerGetBuckBstCfg(pPmicCoreHandle_power, &buckBstCfg_actual,
                                     &pwrRsrcRegCfg, &pwrRsrcCfg);
    if (buckBstCfg_expected.bbPgoodCfg == buckBstCfg_actual.bbPgoodCfg) {
      DebugP_log("Test Passed: Buck-Boost OV and UV status included in "
                 "PGOOD set/get configuration passed!\r\n");
    } else {
      DebugP_log("Test Failed: Buck-Boost OV and UV status included in "
                 "PGOOD set/get configuration failed!\r\n");
    }
  }

  /* Set Buck Boost OV and UV status be not included in PGOOD */
  buckBstCfg_expected.bbPgoodCfg = PMIC_BB_CFG_STATUS_NOT_INCLUDED;
  status = Pmic_powerSetBuckBstCfg(pPmicCoreHandle_power, &buckBstCfg_expected,
                                   &pwrRsrcRegCfg, &pwrRsrcCfg);
  if (PMIC_ST_SUCCESS == status) {
    status = Pmic_powerGetBuckBstCfg(pPmicCoreHandle_power, &buckBstCfg_actual,
                                     &pwrRsrcRegCfg, &pwrRsrcCfg);
    if (buckBstCfg_expected.bbPgoodCfg == buckBstCfg_actual.bbPgoodCfg) {
      DebugP_log("Test Passed: Buck-Boost OV and UV status not included "
                 "in PGOOD set/get configuration passed!\r\n");
    } else {
      DebugP_log("Test Failed: Buck-Boost OV and UV status not included "
                 "in PGOOD set/get configuration failed!\r\n");
    }
  }
}

/**
 * @brief Test function for setting and getting Buck-Boost SS Enable
 * configuration. This function tests the functionality of setting and getting
 * the Buck-Boost SS Enable configuration. It sets the Buck-Boost DRSS to be
 * enabled and disabled, respectively, and verifies the configuration by
 * retrieving and comparing the actual configuration with the expected one.
 *
 * @param void
 * @return void
 */
void test_power_setBuckBst_SSEn_Cfg(void) {
  int32_t status = PMIC_ST_SUCCESS;
  Pmic_powerBuckBoostCfgReg_t buckBstCfg_expected, buckBstCfg_actual;
  Pmic_powerRsrcCfg_t pwrRsrcCfg;
  Pmic_powerRsrcRegCfg_t pwrRsrcRegCfg;

  pwrRsrcRegCfg.buckConfigRegAddr = PMIC_POWER_BUCK_CFG_REGADDR;

  pwrRsrcCfg.pmicConfigShiftVal = BB_SS_EN_SHIFT;
  pwrRsrcCfg.pmicConfigMaskVal = BB_SS_EN_MASK;

  /* Set Buck Boost DRSS Enabled */
  buckBstCfg_expected.bbSsEn = PMIC_BB_CFG_DRSS_ENABLED;
  status = Pmic_powerSetBuckBstCfg(pPmicCoreHandle_power, &buckBstCfg_expected,
                                   &pwrRsrcRegCfg, &pwrRsrcCfg);
  if (PMIC_ST_SUCCESS == status) {
    status = Pmic_powerGetBuckBstCfg(pPmicCoreHandle_power, &buckBstCfg_actual,
                                     &pwrRsrcRegCfg, &pwrRsrcCfg);
    if (buckBstCfg_expected.bbSsEn == buckBstCfg_actual.bbSsEn) {
      DebugP_log("Test Passed: Buck-Boost DRSS Enable passed!\r\n");
    } else {
      DebugP_log("Test Failed: Buck-Boost DRSS Enable failed!\r\n");
    }
  }

  /* Set Buck Boost DRSS Disabled */
  buckBstCfg_expected.bbSsEn = PMIC_BB_CFG_DRSS_DISABLED;
  status = Pmic_powerSetBuckBstCfg(pPmicCoreHandle_power, &buckBstCfg_expected,
                                   &pwrRsrcRegCfg, &pwrRsrcCfg);
  if (PMIC_ST_SUCCESS == status) {
    status = Pmic_powerGetBuckBstCfg(pPmicCoreHandle_power, &buckBstCfg_actual,
                                     &pwrRsrcRegCfg, &pwrRsrcCfg);
    if (buckBstCfg_expected.bbSsEn == buckBstCfg_actual.bbSsEn) {
      DebugP_log("Test Passed: Buck-Boost DRSS Disable passed!\r\n");
    } else {
      DebugP_log("Test Failed: Buck-Boost DRSS Disable failed!\r\n");
    }
  }
}

/**
 * @brief Test function for setting and getting Buck-Boost Standby Level
 * configuration. This function tests the functionality of setting and getting
 * the Buck-Boost Standby Level configuration. It sets the Buck-Boost Standby
 * Level to different voltage levels and same as the level config, respectively,
 * and verifies the configuration by retrieving and comparing the actual
 * configuration with the expected one.
 *
 * @param void
 * @return void
 */
void test_power_setBuckBst_StbyLvl_Cfg(void) {
  int32_t status = PMIC_ST_SUCCESS;
  Pmic_powerBuckBoostCfgReg_t buckBstCfg_expected, buckBstCfg_actual;
  Pmic_powerRsrcCfg_t pwrRsrcCfg;
  Pmic_powerRsrcRegCfg_t pwrRsrcRegCfg;

  pwrRsrcRegCfg.buckConfigRegAddr = PMIC_POWER_BUCK_CFG_REGADDR;

  pwrRsrcCfg.pmicConfigShiftVal = BB_STBY_LVL_CFG_SHIFT;
  pwrRsrcCfg.pmicConfigMaskVal = BB_STBY_LVL_CFG_MASK;

  /* Set Buck Boost Standby Level to 4V */
  buckBstCfg_expected.bbPgoodCfg = PMIC_BB_CFG_4V;
  status = Pmic_powerSetBuckBstCfg(pPmicCoreHandle_power, &buckBstCfg_expected,
                                   &pwrRsrcRegCfg, &pwrRsrcCfg);
  if (PMIC_ST_SUCCESS == status) {
    status = Pmic_powerGetBuckBstCfg(pPmicCoreHandle_power, &buckBstCfg_actual,
                                     &pwrRsrcRegCfg, &pwrRsrcCfg);
    if (buckBstCfg_expected.bbPgoodCfg == buckBstCfg_actual.bbPgoodCfg) {
      DebugP_log("Test Passed: Buck-Boost Standby level to 4V passed!\r\n");
    } else {
      DebugP_log("Test Failed: Buck-Boost Standby level to 4V failed!\r\n");
    }
  }

  /* Set Buck Boost Standby Level to same as level config */
  buckBstCfg_expected.bbPgoodCfg = PMIC_BB_CFG_SAME_BBCFG;
  status = Pmic_powerSetBuckBstCfg(pPmicCoreHandle_power, &buckBstCfg_expected,
                                   &pwrRsrcRegCfg, &pwrRsrcCfg);
  if (PMIC_ST_SUCCESS == status) {
    status = Pmic_powerGetBuckBstCfg(pPmicCoreHandle_power, &buckBstCfg_actual,
                                     &pwrRsrcRegCfg, &pwrRsrcCfg);
    if (buckBstCfg_expected.bbPgoodCfg == buckBstCfg_actual.bbPgoodCfg) {
      DebugP_log("Test Passed: Buck-Boost Standby level to same as level "
                 "config passed!\r\n");
    } else {
      DebugP_log("Test Failed: Buck-Boost Standby level to same as level "
                 "config failed!\r\n");
    }
  }
}

/**
 * @brief Test function for setting and getting Buck-Boost Level configuration.
 * This function tests the functionality of setting and getting the Buck-Boost
 * Level configuration. It sets the Buck-Boost Level config to different voltage
 * levels, such as 4.3V, 5V, and 6V, respectively, and verifies the
 * configuration by retrieving and comparing the actual configuration with the
 * expected one.
 *
 * @param void
 * @return void
 */
void test_power_setBuckBst_Level_Cfg(void) {
  int32_t status = PMIC_ST_SUCCESS;
  Pmic_powerBuckBoostCfgReg_t buckBstCfg_expected, buckBstCfg_actual;
  Pmic_powerRsrcCfg_t pwrRsrcCfg;
  Pmic_powerRsrcRegCfg_t pwrRsrcRegCfg;

  pwrRsrcRegCfg.buckConfigRegAddr = PMIC_POWER_BUCK_CFG_REGADDR;

  pwrRsrcCfg.pmicConfigShiftVal = BB_LVL_CFG_SHIFT;
  pwrRsrcCfg.pmicConfigMaskVal = BB_LVL_CFG_MASK;

  /* Set Buck Boost Level config to 4.3V */
  buckBstCfg_expected.bbPgoodCfg = PMIC_BB_CFG_BB_LVL_CFG_4_3V;
  status = Pmic_powerSetBuckBstCfg(pPmicCoreHandle_power, &buckBstCfg_expected,
                                   &pwrRsrcRegCfg, &pwrRsrcCfg);
  if (PMIC_ST_SUCCESS == status) {
    status = Pmic_powerGetBuckBstCfg(pPmicCoreHandle_power, &buckBstCfg_actual,
                                     &pwrRsrcRegCfg, &pwrRsrcCfg);
    if (buckBstCfg_expected.bbPgoodCfg == buckBstCfg_actual.bbPgoodCfg) {
      DebugP_log(
          "Test Passed: Buck-Boost level config set to 4.3V passed!\r\n");
    } else {
      DebugP_log(
          "Test Failed: Buck-Boost level config set to 4.3V failed!\r\n");
    }
  }

  /* Set Buck Boost Level config to 5V */
  buckBstCfg_expected.bbPgoodCfg = PMIC_BB_CFG_BB_LVL_CFG_5V;
  status = Pmic_powerSetBuckBstCfg(pPmicCoreHandle_power, &buckBstCfg_expected,
                                   &pwrRsrcRegCfg, &pwrRsrcCfg);
  if (PMIC_ST_SUCCESS == status) {
    status = Pmic_powerGetBuckBstCfg(pPmicCoreHandle_power, &buckBstCfg_actual,
                                     &pwrRsrcRegCfg, &pwrRsrcCfg);
    if (buckBstCfg_expected.bbPgoodCfg == buckBstCfg_actual.bbPgoodCfg) {
      DebugP_log("Test Passed: Buck-Boost level config set to 5V passed!\r\n");
    } else {
      DebugP_log("Test Failed: Buck-Boost level config set to 5V failed!\r\n");
    }
  }

  /* Set Buck Boost Level config to 6V */
  buckBstCfg_expected.bbPgoodCfg = PMIC_BB_CFG_BB_LVL_CFG_6V;
  status = Pmic_powerSetBuckBstCfg(pPmicCoreHandle_power, &buckBstCfg_expected,
                                   &pwrRsrcRegCfg, &pwrRsrcCfg);
  if (PMIC_ST_SUCCESS == status) {
    status = Pmic_powerGetBuckBstCfg(pPmicCoreHandle_power, &buckBstCfg_actual,
                                     &pwrRsrcRegCfg, &pwrRsrcCfg);
    if (buckBstCfg_expected.bbPgoodCfg == buckBstCfg_actual.bbPgoodCfg) {
      DebugP_log("Test Passed: Buck-Boost level config set to 6V passed!\r\n");
    } else {
      DebugP_log("Test Failed: Buck-Boost level config set to 6V failed!\r\n");
    }
  }
}

void test_pmic_set_ldoVmonCtrl() {
  int32_t pmicStatus = PMIC_ST_SUCCESS;
  Pmic_lpVMonCtrlReg_t *lpVMonCtrlCfg = NULL;
  lpVMonCtrlCfg->lpldo1VMonCtrl = LP_VMON_CTRL_DATA1;
  lpVMonCtrlCfg->lpldo2VMonCtrl = LP_VMON_CTRL_DATA2;
  lpVMonCtrlCfg->lpldo3VMonCtrl = LP_VMON_CTRL_DATA2;
  lpVMonCtrlCfg->lpldo4VMonCtrl = LP_VMON_CTRL_DATA1;
  pmicStatus = Pmic_SetLPLDOVMonCtrl(pPmicCoreHandle_power, lpVMonCtrlCfg);
  if (PMIC_ST_SUCCESS != pmicStatus) {
    DebugP_log("%s(): %d: FAILED with status: %d\n", __func__, __LINE__,
               pmicStatus);
  }
}

void test_pmic_get_ldoVmonCtrl() {
  int32_t pmicStatus = PMIC_ST_SUCCESS;
  Pmic_lpVMonCtrlReg_t *lpVMonCtrlCfg = NULL;
  pmicStatus = Pmic_GetLPLDOVMonCtrl(pPmicCoreHandle_power, lpVMonCtrlCfg);
  if (PMIC_ST_SUCCESS != pmicStatus) {
    DebugP_log("%s(): %d: FAILED with status: %d\n", __func__, __LINE__,
               pmicStatus);
  }
}

void test_pmic_set_pldoVmonCtrl() {
  int32_t pmicStatus = PMIC_ST_SUCCESS;
  Pmic_lpVMonCtrlReg_t *lpVMonCtrlCfg = NULL;
  lpVMonCtrlCfg->lppldo1VMonCtrl = LP_VMON_CTRL_DATA2;
  lpVMonCtrlCfg->lppldo2VMonCtrl = LP_VMON_CTRL_DATA1;
  pmicStatus = Pmic_SetLPPLDOVMonCtrl(pPmicCoreHandle_power, lpVMonCtrlCfg);
  if (PMIC_ST_SUCCESS != pmicStatus) {
    DebugP_log("%s(): %d: FAILED with status: %d\n", __func__, __LINE__,
               pmicStatus);
  }
}

void test_pmic_get_pldoVmonCtrl() {
  int32_t pmicStatus = PMIC_ST_SUCCESS;
  Pmic_lpVMonCtrlReg_t *lpVMonCtrlCfg = NULL;
  pmicStatus = Pmic_GetLPPLDOVMonCtrl(pPmicCoreHandle_power, lpVMonCtrlCfg);
  if (PMIC_ST_SUCCESS != pmicStatus) {
    DebugP_log("%s(): %d: FAILED with status: %d\n", __func__, __LINE__,
               pmicStatus);
  }
}

void test_pmic_set_extVmonCtrl() {
  int32_t pmicStatus = PMIC_ST_SUCCESS;
  Pmic_lpVMonCtrlReg_t *lpVMonCtrlCfg = NULL;
  lpVMonCtrlCfg->lpExtVMon2Ctrl = LP_VMON_CTRL_DATA1;
  lpVMonCtrlCfg->lpExtVMon1Ctrl = LP_VMON_CTRL_DATA2;
  pmicStatus = Pmic_SetLPExtVMonCtrl(pPmicCoreHandle_power, lpVMonCtrlCfg);
  if (PMIC_ST_SUCCESS != pmicStatus) {
    DebugP_log("%s(): %d: FAILED with status: %d\n", __func__, __LINE__,
               pmicStatus);
  }
}

void test_pmic_get_extVmonCtrl() {
  int32_t pmicStatus = PMIC_ST_SUCCESS;
  Pmic_lpVMonCtrlReg_t *lpVMonCtrlCfg = NULL;
  pmicStatus = Pmic_GetLPExtVMonCtrl(pPmicCoreHandle_power, lpVMonCtrlCfg);
  if (PMIC_ST_SUCCESS != pmicStatus) {
    DebugP_log("%s(): %d: FAILED with status: %d\n", __func__, __LINE__,
               pmicStatus);
  }
}

void test_pmic_set_bb_ovp_ctrl() {
  int32_t pmicStatus = PMIC_ST_SUCCESS;
  Pmic_lpConfigCtrlReg_t *lpConfig = NULL;
  lpConfig->lpBBOVPCtrl = LP_VMON_CTRL_DATA1;
  pmicStatus = Pmic_SetlpBBOVPCtrl(pPmicCoreHandle_power, lpConfig);
  if (PMIC_ST_SUCCESS != pmicStatus) {
    DebugP_log("%s(): %d: FAILED with status: %d\n", __func__, __LINE__,
               pmicStatus);
  }
}

void test_pmic_get_bb_ovp_ctrl() {
  int32_t pmicStatus = PMIC_ST_SUCCESS;
  Pmic_lpConfigCtrlReg_t *lpConfig = NULL;
  pmicStatus = Pmic_GetlpBBOVPCtrl(pPmicCoreHandle_power, lpConfig);
  if (PMIC_ST_SUCCESS != pmicStatus) {
    DebugP_log("%s(): %d: FAILED with status: %d\n", __func__, __LINE__,
               pmicStatus);
  }
}

void test_pmic_set_bb_vmon_ctrl() {
  int32_t pmicStatus = PMIC_ST_SUCCESS;
  Pmic_lpConfigCtrlReg_t *lpConfig = NULL;
  lpConfig->lpBBVMonCtrl = LP_VMON_CTRL_DATA1;
  pmicStatus = Pmic_SetlpBBVmonCtrl(pPmicCoreHandle_power, lpConfig);
  if (PMIC_ST_SUCCESS != pmicStatus) {
    DebugP_log("%s(): %d: FAILED with status: %d\n", __func__, __LINE__,
               pmicStatus);
  }
}

void test_pmic_get_bb_vmon_ctrl() {
  int32_t pmicStatus = PMIC_ST_SUCCESS;
  Pmic_lpConfigCtrlReg_t *lpConfig = NULL;
  pmicStatus = Pmic_GetlpBBVmonCtrl(pPmicCoreHandle_power, lpConfig);
  if (PMIC_ST_SUCCESS != pmicStatus) {
    DebugP_log("%s(): %d: FAILED with status: %d\n", __func__, __LINE__,
               pmicStatus);
  }
}

void test_pmic_set_tsd_per_cfg() {
  int32_t pmicStatus = PMIC_ST_SUCCESS;
  Pmic_lpConfigCtrlReg_t *lpConfig = NULL;
  lpConfig->lpTSDperConfig = LP_TSD_PER_CFG_DATA1;
  pmicStatus = Pmic_SetlpTSDperCfg(pPmicCoreHandle_power, lpConfig);
  if (PMIC_ST_SUCCESS != pmicStatus) {
    DebugP_log("%s(): %d: FAILED with status: %d\n", __func__, __LINE__,
               pmicStatus);
  }
}

void test_pmic_get_tsd_per_cfg() {
  int32_t pmicStatus = PMIC_ST_SUCCESS;
  Pmic_lpConfigCtrlReg_t *lpConfig = NULL;
  pmicStatus = Pmic_GetlpTSDperCfg(pPmicCoreHandle_power, lpConfig);
  if (PMIC_ST_SUCCESS != pmicStatus) {
    DebugP_log("%s(): %d: FAILED with status: %d\n", __func__, __LINE__,
               pmicStatus);
  }
}

void test_pmic_set_vmon_per_cfg() {
  int32_t pmicStatus = PMIC_ST_SUCCESS;
  Pmic_lpConfigCtrlReg_t *lpConfig = NULL;
  lpConfig->lpVMonperConfig = LP_VMON_PER_CFG_DATA2;
  pmicStatus = Pmic_SetlpVmonperCfg(pPmicCoreHandle_power, lpConfig);
  if (PMIC_ST_SUCCESS != pmicStatus) {
    DebugP_log("%s(): %d: FAILED with status: %d\n", __func__, __LINE__,
               pmicStatus);
  }
}

void test_pmic_get_vmon_per_cfg() {
  int32_t pmicStatus = PMIC_ST_SUCCESS;
  Pmic_lpConfigCtrlReg_t *lpConfig = NULL;
  pmicStatus = Pmic_GetlpVmonperCfg(pPmicCoreHandle_power, lpConfig);
  if (PMIC_ST_SUCCESS != pmicStatus) {
    DebugP_log("%s(): %d: FAILED with status: %d\n", __func__, __LINE__,
               pmicStatus);
  }
}

void test_pmic_ldoVmon_threshold() {
  int32_t pmicStatus = PMIC_ST_SUCCESS;
  int32_t ldoNumber = 0U;
  Pmic_ldoVMONThresholdReg_t vmonTestBitConfig_exp, vmonTestBitConfig_act;
  Pmic_powerRsrcRegCfg_t *vmonTestRegConfig = NULL;

  vmonTestRegConfig->vmonTHCfg1RegAddr = PMIC_VMON_TH_CFG1_REGADDR;

  /*SET LDO1 VMON Threshold*/
  ldoNumber = PMIC_LDO1;
  vmonTestBitConfig_exp.ldo1vmonthresh = PMIC_LDO_VMON_TH_5;
  vmonTestRegConfig->bitPosVal = PMIC_VMON_CFG1_LDO1_VMON_SHIFT;
  vmonTestRegConfig->bitMaskVal = PMIC_VMON_CFG1_LDO1_VMON_MASK;
  pmicStatus =
      Pmic_setldoVmonThresholdConfig(pPmicCoreHandle_power, ldoNumber,
                                     &vmonTestBitConfig_exp, vmonTestRegConfig);
  if (PMIC_ST_SUCCESS != pmicStatus) {
    pmicStatus = Pmic_getldoVmonThresholdConfig(
        pPmicCoreHandle_power, ldoNumber, &vmonTestBitConfig_act,
        vmonTestRegConfig);
    if (vmonTestBitConfig_exp.ldo1vmonthresh ==
        vmonTestBitConfig_act.ldo1vmonthresh) {
      DebugP_log("Test Passed: LDO1 Set VMON Threshold passed!\r\n");
    } else {
      DebugP_log("Test Failed: LDO1 Set VMON Threshold failed!\r\n");
    }
  }

  /*SET LDO2 VMON Threshold*/
  ldoNumber = PMIC_LDO2;
  vmonTestBitConfig_exp.ldo2vmonthresh = PMIC_LDO_VMON_TH_5;
  vmonTestRegConfig->bitPosVal = PMIC_VMON_CFG1_LDO2_VMON_SHIFT;
  vmonTestRegConfig->bitMaskVal = PMIC_VMON_CFG1_LDO2_VMON_MASK;
  pmicStatus =
      Pmic_setldoVmonThresholdConfig(pPmicCoreHandle_power, ldoNumber,
                                     &vmonTestBitConfig_exp, vmonTestRegConfig);
  if (PMIC_ST_SUCCESS != pmicStatus) {
    pmicStatus = Pmic_getldoVmonThresholdConfig(
        pPmicCoreHandle_power, ldoNumber, &vmonTestBitConfig_act,
        vmonTestRegConfig);
    if (vmonTestBitConfig_exp.ldo2vmonthresh ==
        vmonTestBitConfig_act.ldo2vmonthresh) {
      DebugP_log("Test Passed: LDO2 Set VMON Threshold passed!\r\n");
    } else {
      DebugP_log("Test Failed: LDO2 Set VMON Threshold failed!\r\n");
    }
  }

  /*SET LDO3 VMON Threshold*/
  ldoNumber = PMIC_LDO3;
  vmonTestBitConfig_exp.ldo3vmonthresh = PMIC_LDO_VMON_TH_5;
  vmonTestRegConfig->bitPosVal = PMIC_VMON_CFG1_LDO3_VMON_SHIFT;
  vmonTestRegConfig->bitMaskVal = PMIC_VMON_CFG1_LDO3_VMON_MASK;
  pmicStatus =
      Pmic_setldoVmonThresholdConfig(pPmicCoreHandle_power, ldoNumber,
                                     &vmonTestBitConfig_exp, vmonTestRegConfig);
  if (PMIC_ST_SUCCESS != pmicStatus) {
    pmicStatus = Pmic_getldoVmonThresholdConfig(
        pPmicCoreHandle_power, ldoNumber, &vmonTestBitConfig_act,
        vmonTestRegConfig);
    if (vmonTestBitConfig_exp.ldo3vmonthresh ==
        vmonTestBitConfig_act.ldo3vmonthresh) {
      DebugP_log("Test Passed: LDO3 Set VMON Threshold passed!\r\n");
    } else {
      DebugP_log("Test Failed: LDO3 Set VMON Threshold failed!\r\n");
    }
  }

  /*SET LDO4 VMON Threshold*/
  ldoNumber = PMIC_LDO4;
  vmonTestBitConfig_exp.ldo4vmonthresh = PMIC_LDO_VMON_TH_5;
  vmonTestRegConfig->bitPosVal = PMIC_VMON_CFG1_LDO4_VMON_SHIFT;
  vmonTestRegConfig->bitMaskVal = PMIC_VMON_CFG1_LDO4_VMON_MASK;
  pmicStatus =
      Pmic_setldoVmonThresholdConfig(pPmicCoreHandle_power, ldoNumber,
                                     &vmonTestBitConfig_exp, vmonTestRegConfig);
  if (PMIC_ST_SUCCESS != pmicStatus) {
    pmicStatus = Pmic_getldoVmonThresholdConfig(
        pPmicCoreHandle_power, ldoNumber, &vmonTestBitConfig_act,
        vmonTestRegConfig);
    if (vmonTestBitConfig_exp.ldo4vmonthresh ==
        vmonTestBitConfig_act.ldo4vmonthresh) {
      DebugP_log("Test Passed: LDO4 Set VMON Threshold passed!\r\n");
    } else {
      DebugP_log("Test Failed: LDO4 Set VMON Threshold failed!\r\n");
    }
  }
}

void test_pmic_ldoVmon_deglitch() {
  int32_t pmicStatus = PMIC_ST_SUCCESS;
  int32_t ldoNumber = 0U;
  Pmic_ldoVMONDglReg_t vmonTestBitConfig_exp, vmonTestBitConfig_act;
  Pmic_powerRsrcRegCfg_t *vmonTestRegConfig = NULL;

  vmonTestRegConfig->vmonTHCfg1RegAddr = PMIC_VMON_DGL_CFG2_REGADDR;

  /*SET LDO1 VMON De-Glitch*/
  ldoNumber = PMIC_LDO1;
  vmonTestBitConfig_exp.ldo1vmondgl = PMIC_LDO_VMON_DGL_8US;
  vmonTestRegConfig->bitPosVal = PMIC_VMON_DGL_CFG2_LDO1_SHIFT;
  vmonTestRegConfig->bitMaskVal = PMIC_VMON_DGL_CFG2_LDO1_MASK;
  pmicStatus =
      Pmic_setldoVMONDeGlitchConfig(pPmicCoreHandle_power, ldoNumber,
                                    &vmonTestBitConfig_exp, vmonTestRegConfig);
  if (PMIC_ST_SUCCESS != pmicStatus) {
    pmicStatus = Pmic_getldoVMONDeGlitchConfig(pPmicCoreHandle_power, ldoNumber,
                                               &vmonTestBitConfig_act,
                                               vmonTestRegConfig);
    if (vmonTestBitConfig_exp.ldo1vmondgl ==
        vmonTestBitConfig_act.ldo1vmondgl) {
      DebugP_log("Test Passed: LDO1 Set VMON Threshold passed!\r\n");
    } else {
      DebugP_log("Test Failed: LDO1 Set VMON Threshold failed!\r\n");
    }
  }

  /*SET LDO2 VMON De-Gltich*/
  ldoNumber = PMIC_LDO2;
  vmonTestBitConfig_exp.ldo2vmondgl = PMIC_LDO_VMON_DGL_16US;
  vmonTestRegConfig->bitPosVal = PMIC_VMON_DGL_CFG2_LDO2_SHIFT;
  vmonTestRegConfig->bitMaskVal = PMIC_VMON_DGL_CFG2_LDO2_MASK;
  pmicStatus =
      Pmic_setldoVMONDeGlitchConfig(pPmicCoreHandle_power, ldoNumber,
                                    &vmonTestBitConfig_exp, vmonTestRegConfig);
  if (PMIC_ST_SUCCESS != pmicStatus) {
    pmicStatus = Pmic_getldoVMONDeGlitchConfig(pPmicCoreHandle_power, ldoNumber,
                                               &vmonTestBitConfig_act,
                                               vmonTestRegConfig);
    if (vmonTestBitConfig_exp.ldo2vmondgl ==
        vmonTestBitConfig_act.ldo2vmondgl) {
      DebugP_log("Test Passed: LDO2 Set VMON Threshold passed!\r\n");
    } else {
      DebugP_log("Test Failed: LDO2 Set VMON Threshold failed!\r\n");
    }
  }

  /*SET LDO3 VMON De-Gltich*/
  ldoNumber = PMIC_LDO3;
  vmonTestBitConfig_exp.ldo3vmondgl = PMIC_LDO_VMON_DGL_24US;
  vmonTestRegConfig->bitPosVal = PMIC_VMON_DGL_CFG2_LDO3_SHIFT;
  vmonTestRegConfig->bitMaskVal = PMIC_VMON_DGL_CFG2_LDO3_MASK;
  pmicStatus =
      Pmic_setldoVMONDeGlitchConfig(pPmicCoreHandle_power, ldoNumber,
                                    &vmonTestBitConfig_exp, vmonTestRegConfig);
  if (PMIC_ST_SUCCESS != pmicStatus) {
    pmicStatus = Pmic_getldoVMONDeGlitchConfig(pPmicCoreHandle_power, ldoNumber,
                                               &vmonTestBitConfig_act,
                                               vmonTestRegConfig);
    if (vmonTestBitConfig_exp.ldo3vmondgl ==
        vmonTestBitConfig_act.ldo3vmondgl) {
      DebugP_log("Test Passed: LDO3 Set VMON De-Gltich passed!\r\n");
    } else {
      DebugP_log("Test Failed: LDO3 Set VMON De-Gltich failed!\r\n");
    }
  }

  /*SET LDO4 VMON De-Gltich*/
  ldoNumber = PMIC_LDO4;
  vmonTestBitConfig_exp.ldo4vmondgl = PMIC_LDO_VMON_DGL_32US;
  vmonTestRegConfig->bitPosVal = PMIC_VMON_DGL_CFG2_LDO4_SHIFT;
  vmonTestRegConfig->bitMaskVal = PMIC_VMON_DGL_CFG2_LDO4_MASK;
  pmicStatus =
      Pmic_setldoVMONDeGlitchConfig(pPmicCoreHandle_power, ldoNumber,
                                    &vmonTestBitConfig_exp, vmonTestRegConfig);
  if (PMIC_ST_SUCCESS != pmicStatus) {
    pmicStatus = Pmic_getldoVMONDeGlitchConfig(pPmicCoreHandle_power, ldoNumber,
                                               &vmonTestBitConfig_act,
                                               vmonTestRegConfig);
    if (vmonTestBitConfig_exp.ldo4vmondgl ==
        vmonTestBitConfig_act.ldo4vmondgl) {
      DebugP_log("Test Passed: LDO4 Set VMON De-Gltich passed!\r\n");
    } else {
      DebugP_log("Test Failed: LDO4 Set VMON De-Gltich failed!\r\n");
    }
  }
}

void test_pmic_setLdo() {
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    uint8_t ldoNumber = 0U;
    uint8_t ldoCtrlFeature = PMIC_LDO_ENABLED_LDO_MODE;
    Pmic_ldoCfgReg_t ldoCfg_exp, ldoCfg_act;
    Pmic_powerRsrcCfg_t *pwrRscCfg = NULL;
    Pmic_powerRsrcRegCfg_t *setLDOpwrRscRegCfg = NULL;

    setLDOpwrRscRegCfg->ldo1ConfigRegAddr = PMIC_LDO1_CFG_REGADDR;
    setLDOpwrRscRegCfg->ldo2ConfigRegAddr = PMIC_LDO2_CFG_REGADDR;
    setLDOpwrRscRegCfg->ldo3ConfigRegAddr = PMIC_LDO3_CFG_REGADDR;
    setLDOpwrRscRegCfg->ldo4ConfigRegAddr = PMIC_LDO4_CFG_REGADDR;
    setLDOpwrRscRegCfg->ldoCtrlRegAddr = PMIC_LDO_CTRL_REGADDR;

    ldoCfg_exp.ldoRegShift = LDO_LVL_CFG_SHIFT;
    ldoCfg_exp.ldoRegMask = LDO_LVL_CFG_MASK;

    /* Set LDO 3.3V configurations */
    ldoNumber = PMIC_LDO2;
    ldoCfg_exp.ldoLvlCfg = PMIC_LDO_PLDO_LVL_CFG_VOLT_3_3V;
    setLDOpwrRscRegCfg->bitPosVal = LDO1_CTRL_SHIFT;
    setLDOpwrRscRegCfg->bitMaskVal = LDO1_CTRL_MASK;
    pmicStatus += Pmic_powerSetLdoConfigRegister(
            pPmicCoreHandle_power, ldoNumber, &ldoCfg_exp, pwrRscCfg, setLDOpwrRscRegCfg);
    if (PMIC_ST_SUCCESS == pmicStatus) {
        pmicStatus = Pmic_powerGetLdoConfigRegister(
                pPmicCoreHandle_power, ldoNumber, &ldoCfg_act, pwrRscCfg, setLDOpwrRscRegCfg);
    }

    if(ldoCfg_exp.ldoLvlCfg == ldoCfg_act.ldoLvlCfg) {
        pmicStatus = Pmic_setLdoCtrl(pPmicCoreHandle_power, ldoCtrlFeature, setLDOpwrRscRegCfg);
        if(PMIC_ST_SUCCESS == pmicStatus)
        {
            DebugP_log("Test Passed: Set LDO Configuration passed!\r\n");
        } else {
            DebugP_log("Test Failed: Set LDO Configuration failed!\r\n");
        }
    }
}

void test_low_powerMode_pmic() {
  test_pmic_get_ldoVmonCtrl();
  test_pmic_set_ldoVmonCtrl();

  test_pmic_get_pldoVmonCtrl();
  test_pmic_set_pldoVmonCtrl();

  test_pmic_get_extVmonCtrl();
  test_pmic_set_extVmonCtrl();

  test_pmic_get_bb_ovp_ctrl();
  test_pmic_set_bb_ovp_ctrl();

  test_pmic_get_bb_vmon_ctrl();
  test_pmic_set_bb_vmon_ctrl();

  test_pmic_get_tsd_per_cfg();
  test_pmic_set_tsd_per_cfg();

  test_pmic_get_vmon_per_cfg();
  test_pmic_set_vmon_per_cfg();
}

/**
 * @brief Main test function for power management.
 * This function serves as the main test case for power management
 * functionality. It executes a sequence of tests including setting and getting
 * various Buck-Boost configurations, and performs initialization and
 * de-initialization procedures before and after the tests.
 *
 * @param args Pointer to the arguments (not used in this function).
 * @return NULL
 */
void *test_pmic_power(void *args) {
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

  /* Commenting for quick test
  test_power_setBuckBst_PGood_Cfg();
  delay(1000);

  test_power_setBuckBst_SSEn_Cfg();
  delay(1000);

  test_power_setBuckBst_StbyLvl_Cfg();
  delay(1000);

  test_power_setBuckBst_Level_Cfg();
  delay(1000);

  test_low_powerMode_pmic();
  delay(1000);

  test_pmic_ldoVmon_threshold();
  delay(1000);

  test_pmic_ldoVmon_deglitch();
  delay(1000);
  */

  DebugP_log("[TEST] Set LDO Configuration :\r\n");
  test_pmic_setLdo();

  /* De-initialization */
  test_pmic_power_config_deinit();
  DebugP_log("\n[DE-INIT] De-initialization Completed\r\n");

  Board_driversClose();
  Drivers_close();

  return NULL;
}
