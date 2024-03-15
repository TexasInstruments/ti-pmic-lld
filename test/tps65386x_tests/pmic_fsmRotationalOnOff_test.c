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
 *  @file  pmic_fsmRotationalOnOff_test.c
 *
 *  @brief  This file contains all the testing related files APIs for the PMIC
 *  core and basic register lock/unlock and other miscellaneous tests.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "pmic_fsmRotationalOnOff_test.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

/* Pointer to Pmic Core Handle */
Pmic_CoreHandle_t *pPmicCoreHandle_fsm = NULL;

/*!
 * @brief   Initialize the PMIC FSM configuration for testing.
 * This function initializes the PMIC FSM configuration for testing purposes.
 * It sets up various parameters required for FSM configuration and initializes
 * the PMIC core handle.
 *
 * @return  Returns the status of the initialization process.
 */
int32_t test_pmic_fsm_config_init(void) {
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

  status = test_pmic_appInit(&pPmicCoreHandle_fsm, &pmicConfigData);
  if (PMIC_ST_SUCCESS != status) {
    DebugP_log("%s(): %d: FAILED with status: %d\r\n", __func__, __LINE__,
               status);
  }
  return status;
}

/*!
 * @brief   Deinitialize the PMIC FSM configuration after testing.
 * This function deinitializes the PMIC FSM configuration after testing is
 * completed. It deinitializes the PMIC core handle and frees up any allocated
 * memory.
 *
 * @return  Returns the status of the deinitialization process.
 */
int32_t test_pmic_fsm_config_deinit(void) {
  int32_t status = PMIC_ST_SUCCESS;

  status = Pmic_deinit(pPmicCoreHandle_fsm);
  free(pPmicCoreHandle_fsm);
  SemaphoreP_destruct(&gpmicCoreObj);
  if (PMIC_ST_SUCCESS != status) {
    DebugP_log("%s(): %d: FAILED with status: %d\r\n", __func__, __LINE__,
               status);
  }
  return status;
}

/**
 * @brief   Gets the current state of the PMIC Finite State Machine (FSM).
 * This function retrieves the current state of the PMIC FSM by calling the
 * `Pmic_fsmGetDeviceStateCfg` function. The retrieved state is then logged for
 * debugging purposes.
 *
 * @param   void
 * @return  NULL
 */
void test_pmic_fsm_state_get() {
  int32_t pmicStatus = PMIC_ST_SUCCESS;
  uint8_t deviceState = 0;

  pmicStatus = Pmic_fsmGetDeviceStateCfg(pPmicCoreHandle_fsm, &deviceState);
  if (pmicStatus != PMIC_ST_SUCCESS) {
    DebugP_log("Failed to retrieve current FSM state.\r\n");
  } else {
    DebugP_log("PMIC_FSM Current State: %u\r\n", deviceState);
  }

  delay(2000);
}

/**
 * @brief   Tests setting the FSM Mission State to ACTIVE.
 * This function tests setting the FSM Mission State to ACTIVE using
 * `Pmic_fsmSetMissionState` and validates the result. It logs whether
 * the test passed or failed along with the expected and actual status.
 *
 * @param   void
 * @return  NULL
 */
void test_pmic_fsmSetMissionState_active(void) {
  int32_t expectedStatus = PMIC_FSM_STAT_ACTIVE;
  int32_t status = PMIC_ST_SUCCESS;
  uint8_t pmicState = 0U;

  pmicState = PMIC_FSM_ACTIVE_STATE;

  status = Pmic_fsmSetMissionState(pPmicCoreHandle_fsm, pmicState);
  if (status != expectedStatus) {
    DebugP_log("Test failed: Setting FSM Mission State to ACTIVE failed.\r\n");
    DebugP_log("Expected status: %ld, Actual status: %ld\r\n", expectedStatus,
               status);
  } else {
    DebugP_log("Test passed: Setting FSM Mission State to ACTIVE passed.\r\n");
  }
}

/**
 * @brief:  Tests setting the FSM Mission State to SAFE.
 * This function tests setting the FSM Mission State to SAFE using
 * `Pmic_fsmSetMissionState` and validates the result. It logs whether
 * the test passed or failed along with the expected and actual status.
 *
 * @param   void
 * @return  NULL
 */
void test_pmic_fsmSetMissionState_safe(void) {
  int32_t expectedStatus = PMIC_FSM_STAT_SAFE;
  int32_t status = PMIC_ST_SUCCESS;
  uint8_t pmicState = 0U;

  pmicState = PMIC_FSM_SAFE_STATE;

  status = Pmic_fsmSetMissionState(pPmicCoreHandle_fsm, pmicState);
  if (status != expectedStatus) {
    DebugP_log("Test failed: Setting FSM Mission State to SAFE failed.\r\n");
    DebugP_log("Expected status: %ld, Actual status: %ld\r\n", expectedStatus,
               status);
  } else {
    DebugP_log("Test passed: Setting FSM Mission State to SAFE passed.\r\n");
  }
}

/**
 * @brief:  Tests setting the FSM Mission State to STANDBY.
 * This function tests setting the FSM Mission State to STANDBY using
 * `Pmic_fsmSetMissionState` and validates the result. It logs whether
 * the test passed or failed along with the expected and actual status.
 *
 * @param   void
 * @return  NULL
 */
void test_pmic_fsmSetMissionState_standby(void) {
  int32_t expectedStatus = PMIC_FSM_STAT_STANDBY;
  int32_t status = PMIC_ST_SUCCESS;
  uint8_t pmicState = 0U;

  pmicState = PMIC_FSM_STANBY_STATE;

  status = Pmic_fsmSetMissionState(pPmicCoreHandle_fsm, pmicState);
  if (status != expectedStatus) {
    DebugP_log("Test failed: Setting FSM Mission State to STANDBY failed.\r\n");
    DebugP_log("Expected status: %ld, Actual status: %ld\r\n", expectedStatus,
               status);
  } else {
    DebugP_log("Test passed: Setting FSM Mission State to STANDBY passed.\r\n");
  }
}

/**
 * @brief:  Tests setting the FSM Mission State to RESET.
 * This function tests setting the FSM Mission State to RESET using
 * `Pmic_fsmSetMissionState` and validates the result. It logs whether
 * the test passed or failed along with the expected and actual status.
 *
 * @param   void
 * @return  NULL
 */
void test_pmic_fsmSetMissionState_reset(void) {
  int32_t expectedStatus = PMIC_FSM_STAT_RESET_MCU;
  int32_t status = PMIC_ST_SUCCESS;
  uint8_t pmicState = 0U;

  pmicState = PMIC_FSM_MCU_ONLY_STATE;

  status = Pmic_fsmSetMissionState(pPmicCoreHandle_fsm, pmicState);
  if (status != expectedStatus) {
    DebugP_log("Test failed: Setting FSM Mission State to RESET failed.\r\n");
    DebugP_log("Expected status: %ld, Actual status: %ld\r\n", expectedStatus,
               status);
  } else {
    DebugP_log("Test passed: Setting FSM Mission State to RESET passed.\r\n");
  }
}


/**
 * @brief:  Tests setting FSM Configuration to Enable FAST BIST.
 * This function tests setting FSM Configuration to enable FAST BIST
 * using `Pmic_fsmSetConfiguration` and validates the result. It
 * logs whether the test passed or failed along with the expected
 * and actual status.
 *
 * @param   void
 * @return  NULL
 */
void test_pmic_fsmSetConfiguration_fastBistEnable(void) {
  int32_t expectedStatus = PMIC_ST_SUCCESS;
  int32_t status = PMIC_ST_SUCCESS;

  Pmic_FsmCfg_t fsmCfg_rd = {
      PMIC_FAST_BIST_EN_VALID_SHIFT,
  };
  Pmic_FsmCfg_t fsmCfg = {
      PMIC_FAST_BIST_EN_VALID_SHIFT, PMIC_FSM_FAST_BIST_ENABLE,
      PMIC_FSM_SELECT_STANDBY_STATE, PMIC_FSM_ILIM_INT_DISABLE,
      PMIC_FSM_STARTUPDEST_ACTIVE};

  status = Pmic_fsmSetConfiguration(pPmicCoreHandle_fsm, fsmCfg);
  if (status != expectedStatus) {
    DebugP_log("Test failed: Setting FSM Configuration to Enable FAST BIST "
               "failed.\r\n");
    DebugP_log("Expected status: %ld, Actual status: %ld\r\n", expectedStatus,
               status);
  } else {
    DebugP_log("Test passed: Setting FSM Configuration to Enable FAST BIST "
               "passed.\r\n");
  }

  status = Pmic_fsmGetConfiguration(pPmicCoreHandle_fsm, &fsmCfg_rd);
  if (status == expectedStatus) {
    DebugP_log("Test failed: Getting FSM Configuration to Enable FAST BIST "
               "failed.\r\n");
    DebugP_log("Expected status: %ld, Actual status: %ld\r\n", expectedStatus,
               status);
  } else {
    DebugP_log("Test passed: Getting FSM Configuration to Enable FAST BIST "
               "passed.\r\n");
  }
}

/**
 * @brief:  Tests setting FSM Configuration to Disable FAST BIST.
 * This function tests setting FSM Configuration to disable FAST BIST
 * using `Pmic_fsmSetConfiguration` and validates the result. It logs
 * whether the test passed or failed along with the expected and
 * actual status.
 *
 * @param   void
 * @return  NULL
 */
void test_pmic_fsmSetConfiguration_fastBistDisable(void) {
  int32_t expectedStatus = PMIC_ST_SUCCESS;
  int32_t status = PMIC_ST_SUCCESS;

  Pmic_FsmCfg_t fsmCfg_rd = {
      PMIC_FAST_BIST_EN_VALID_SHIFT,
  };
  Pmic_FsmCfg_t fsmCfg = {
      PMIC_FAST_BIST_EN_VALID_SHIFT, PMIC_FSM_FAST_BIST_DISABLE,
      PMIC_FSM_SELECT_STANDBY_STATE, PMIC_FSM_ILIM_INT_DISABLE,
      PMIC_FSM_STARTUPDEST_ACTIVE};

  status = Pmic_fsmSetConfiguration(pPmicCoreHandle_fsm, fsmCfg);
  if (status != expectedStatus) {
    DebugP_log("Test failed: Setting FSM Configuration to Disable FAST BIST "
               "failed.\r\n");
    DebugP_log("Expected status: %ld, Actual status: %ld\r\n", expectedStatus,
               status);
  } else {
    DebugP_log("Test passed: Setting FSM Configuration to Disable FAST BIST "
               "passed.\r\n");
  }

  status = Pmic_fsmGetConfiguration(pPmicCoreHandle_fsm, &fsmCfg_rd);
  if (status == expectedStatus) {
    DebugP_log("Test failed: Getting FSM Configuration to Disable FAST BIST "
               "failed.\r\n");
    DebugP_log("Expected status: %ld, Actual status: %ld\r\n", expectedStatus,
               status);
  } else {
    DebugP_log("Test passed: Getting FSM Configuration to Disable FAST BIST "
               "passed.\r\n");
  }
}

/**
 * @brief:  Tests setting FSM Configuration to Enable ILIM.
 * This function tests setting FSM Configuration to enable ILIM using
 * `Pmic_fsmSetConfiguration` and validates the result. It logs whether
 * the test passed or failed along with the expected and actual status.
 *
 * @param   void
 * @return  NULL
 */
void test_pmic_fsmSetConfiguration_ilimIntfsmCtrlEnable(void) {
  int32_t expectedStatus = PMIC_ST_SUCCESS;
  int32_t status = PMIC_ST_SUCCESS;

  Pmic_FsmCfg_t fsmCfg_rd = {
      PMIC_ILIM_INT_EN_VALID_SHIFT,
  };
  Pmic_FsmCfg_t fsmCfg = {
      PMIC_ILIM_INT_EN_VALID_SHIFT, PMIC_FSM_FAST_BIST_ENABLE,
      PMIC_FSM_SELECT_STANDBY_STATE, PMIC_FSM_ILIM_INT_ENABLE,
      PMIC_FSM_STARTUPDEST_ACTIVE};

  status = Pmic_fsmSetConfiguration(pPmicCoreHandle_fsm, fsmCfg);
  if (status != expectedStatus) {
    DebugP_log("Test failed: Setting FSM Configuration to Enable ILIM "
               "failed.\r\n");
    DebugP_log("Expected status: %ld, Actual status: %ld\r\n", expectedStatus,
               status);
  } else {
    DebugP_log("Test passed: Setting FSM Configuration to Enable ILIM "
               "passed.\r\n");
  }

  status = Pmic_fsmGetConfiguration(pPmicCoreHandle_fsm, &fsmCfg_rd);
  if (status != expectedStatus) {
    DebugP_log("Test failed: Getting FSM Configuration to Enable ILIM "
               "failed.\r\n");
    DebugP_log("Expected status: %ld, Actual status: %ld\r\n", expectedStatus,
               status);
  } else {
    DebugP_log("Test passed: Getting FSM Configuration to Enable ILIM "
               "passed.\r\n");
  }
}

/**
 * @brief:  Tests setting FSM Configuration to Disable ILIM.
 * This function tests setting FSM Configuration to disable ILIM using
 * `Pmic_fsmSetConfiguration` and validates the result. It logs whether
 * the test passed or failed along with the expected and actual status.
 *
 * @param   void
 * @return  NULL
 */
void test_pmic_fsmSetConfiguration_ilimIntfsmCtrlDisable(void) {
  int32_t expectedStatus = PMIC_ST_SUCCESS;
  int32_t status = PMIC_ST_SUCCESS;

  Pmic_FsmCfg_t fsmCfg_rd = {
      PMIC_ILIM_INT_EN_VALID_SHIFT,
  };
  Pmic_FsmCfg_t fsmCfg = {
      PMIC_ILIM_INT_EN_VALID_SHIFT, PMIC_FSM_FAST_BIST_DISABLE,
      PMIC_FSM_SELECT_STANDBY_STATE, PMIC_FSM_ILIM_INT_DISABLE,
      PMIC_FSM_STARTUPDEST_ACTIVE};

  status = Pmic_fsmSetConfiguration(pPmicCoreHandle_fsm, fsmCfg);
  if (status != expectedStatus) {
    DebugP_log("Test failed: Setting FSM Configuration to Disable ILIM "
               "failed.\r\n");
    DebugP_log("Expected status: %ld, Actual status: %ld\r\n", expectedStatus,
               status);
  } else {
    DebugP_log("Test passed: Setting FSM Configuration to Disable ILIM "
               "passed.\r\n");
  }

  status = Pmic_fsmGetConfiguration(pPmicCoreHandle_fsm, &fsmCfg_rd);
  if (status != expectedStatus) {
    DebugP_log("Test failed: Getting FSM Configuration to Disable ILIM "
               "failed.\r\n");
    DebugP_log("Expected status: %ld, Actual status: %ld\r\n", expectedStatus,
               status);
  } else {
    DebugP_log("Test passed: Getting FSM Configuration to Disable ILIM "
               "passed.\r\n");
  }
}

/**
 * @brief:  Tests requesting Runtime BIST with a negative scenario.
 * This function tests requesting Runtime BIST using
 * `Pmic_fsmRequestRuntimeBist` with a negative scenario and validates the
 * result. It logs whether the test passed or failed along with the expected and
 * actual status.
 *
 * @param   void
 * @return  NULL
 */
void test_pmic_negativefsmRuntimeBistRequest(void) {
  int32_t expectedStatus = PMIC_ST_SUCCESS;
  int32_t status = PMIC_ST_SUCCESS;

  status = Pmic_fsmRequestRuntimeBist(pPmicCoreHandle_fsm);
  if (status != expectedStatus) {
    DebugP_log("Test failed: Request for Runtime BIST failed.\r\n");
    DebugP_log("Expected status: %ld, Actual status: %ld\r\n", expectedStatus,
               status);
  } else {
    DebugP_log("Test passed: Getting FSM Configuration to Disable ILIM "
               "passed.\r\n");
  }
}

/*!
 * @brief   Main function for testing PMIC FSM.
 * This function serves as the main entry point for testing PMIC FSM
 * functionality. It initializes the necessary drivers and hardware, performs
 * various tests, and then deinitializes the configuration.
 *
 * @param args Arguments passed to the function (not used).
 * @return void* Returns NULL when testing is completed.
 */
void *test_pmic_FSM(void *args) {
  Drivers_open();
  Board_driversOpen();

  mcspi_mux_pmic();

  /* Initialization */
  DebugP_log("Initializing...\r\n");
  test_pmic_fsm_config_init();
  DebugP_log("Initialization Completed!\r\n\n");

  /* Lock config register initially */
  DebugP_log("[INIT] Configuration Register Lock Sequence:\r\n");
  test_pmic_LockUnlock(pPmicCoreHandle_fsm, 0); // LOCK

  /*API to unlock configuration register */
  DebugP_log("[INIT] Configuration Register Unlock Sequence:\r\n");
  test_pmic_LockUnlock(pPmicCoreHandle_fsm, 1); // UNLOCK

  /* Lock counter register initially */
  DebugP_log("[INIT] TC and RC Lock Sequence:\r\n");
  test_pmic_CNT_LockUnlock(pPmicCoreHandle_fsm, 0); // LOCK

  /* Unlock counter register initially */
  DebugP_log("[INIT] TC and RC Unlock Sequence:\r\n");
  test_pmic_CNT_LockUnlock(pPmicCoreHandle_fsm, 1); // UNLOCK

  DebugP_log("[TEST] INITIAL PMIC FSM GET STATE:\r\n");
  test_pmic_fsm_state_get();
  delay(10000);

  DebugP_log("\n[TEST] SET FSM STATE - ACTIVE\r\n");
  test_pmic_fsmSetMissionState_active();
  delay(10000);

  DebugP_log("\n[TEST] SET FSM STATE - RESET\r\n");
  test_pmic_fsmSetMissionState_reset();

  DebugP_log("\n[TEST] ENABLE FAST BIST\r\n");
  test_pmic_fsmSetConfiguration_fastBistEnable();
  delay(10000);

  DebugP_log("\n[TEST] DISABLE FAST BIST\r\n");
  test_pmic_fsmSetConfiguration_fastBistDisable();
  delay(10000);

  DebugP_log("\n[TEST] BUCK/LDO ILIM ENABLE\r\n");
  test_pmic_fsmSetConfiguration_ilimIntfsmCtrlEnable();
  delay(10000);

  DebugP_log("\n[TEST] BUCK/LDO ILIM DISABLE\r\n");
  test_pmic_fsmSetConfiguration_ilimIntfsmCtrlDisable();
  delay(10000);

  DebugP_log("\n[TEST] RUNTIME BIST REQUEST - NEGATIVE TESTCASE\r\n");
  test_pmic_negativefsmRuntimeBistRequest();
  delay(10000);

  DebugP_log("\n[TEST] FINAL PMIC FSM GET STATE:\r\n");
  test_pmic_fsm_state_get();

  /* De-initialization */
  test_pmic_fsm_config_deinit();
  DebugP_log("\n[DE-INIT] De-initialization Completed\r\n");

  Board_driversClose();
  Drivers_close();

  return NULL;
}
