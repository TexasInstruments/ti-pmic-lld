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
 *  @file  pmic_gpio_test.c
 *
 *  @brief  This file contains all the testing related APIs for the PMIC
 *  GPI/GPO.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include "pmic_gpio_test.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

/* Pointer to Pmic Core Handle */
Pmic_CoreHandle_t *pPmicCoreHandle_gpio = NULL;

/*!
 * @brief   Initialize the PMIC GPIO configuration for testing.
 * This function initializes the PMIC GPIO configuration for testing purposes.
 * It sets up various parameters required for GPIO configuration and initializes
 * the PMIC core handle.
 *
 * @param   void
 * @return  Returns the status of the initialization process.
 */
int32_t test_pmic_gpio_config_init(void) {
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

  status = test_pmic_appInit(&pPmicCoreHandle_gpio, &pmicConfigData);
  if (PMIC_ST_SUCCESS != status) {
    DebugP_log("%s(): %d: FAILED with status: %d\r\n", __func__, __LINE__,
               status);
  }
  return status;
}

/*!
 * @brief   Deinitialize the PMIC GPIO configuration after testing.
 * This function deinitializes the PMIC GPIO configuration after testing is
 * completed. It deinitializes the PMIC core handle and frees up any allocated
 * memory.
 *
 * @param   void
 * @return  Returns the status of the deinitialization process.
 */
int32_t test_pmic_gpio_config_deinit(void) {
  int32_t status = PMIC_ST_SUCCESS;

  status = Pmic_deinit(pPmicCoreHandle_gpio);
  free(pPmicCoreHandle_gpio);
  SemaphoreP_destruct(&gpmicCoreObj);
  if (PMIC_ST_SUCCESS != status) {
    DebugP_log("%s(): %d: FAILED with status: %d\r\n", __func__, __LINE__,
               status);
  }
  return status;
}

/**
 * @brief Test function to set deglitch time for PMIC_BB_GPO3 and PMIC_BB_GPO4.
 * This function configures the deglitch time for PMIC_BB_GPO3 and PMIC_BB_GPO4
 * and then retrieves the configured values for verification.
 *
 * @param  void
 * @return NULL
 */
void test_pmic_gpo34setdgl() {
  int32_t pmicStatus = PMIC_ST_SUCCESS;
  Pmic_GpioRdbkDglCfg_t GpioRdbkDglCfg;
  GpioRdbkDglCfg.gpo3RDglConfig = 1;
  GpioRdbkDglCfg.gpo3RDglData = 2;
  GpioRdbkDglCfg.gpo4RDglConfig = 1;
  GpioRdbkDglCfg.gpo4RDglData = 3;
  pmicStatus = Pmic_gpo34SetDeglitchTime(pPmicCoreHandle_gpio, &GpioRdbkDglCfg);
  if (PMIC_ST_SUCCESS != pmicStatus) {
    DebugP_log("%s(): %d: FAILED with status: %d\n", __func__, __LINE__,
               pmicStatus);
  }
  test_get_gpo34_deglitch_time();
}

/**
 * @brief Test function to retrieve deglitch time for PMIC_BB_GPO3 and
 * PMIC_BB_GPO4. This function retrieves the configured deglitch time for
 * PMIC_BB_GPO3 and PMIC_BB_GPO4 and logs the values for verification.
 *
 * @param  void
 * @return NULL
 */
void test_get_gpo34_deglitch_time() {
  int32_t status = PMIC_ST_SUCCESS;
  Pmic_GpioRdbkDglCfg_t GpioRdbkDglCfg;

  DebugP_log(
      "Testing PMIC_BB_GPO3 and PMIC_BB_GPO4 deglitch time retrieval...\r\n");

  status = Pmic_gpo34GetDeglitchTime(pPmicCoreHandle_gpio, &GpioRdbkDglCfg);

  if (PMIC_ST_SUCCESS != status) {
    DebugP_log("Failed to retrieve PMIC_BB_GPO3 and PMIC_BB_GPO4 deglitch "
               "time: %d\r\n",
               status);
  } else {
    DebugP_log("GPO3RDglData: %x\r\nGPO4RDglData: %x\r\n",
               GpioRdbkDglCfg.gpo3RDglData, GpioRdbkDglCfg.gpo4RDglData);
  }
}

/**
 * @brief Test function to set deglitch time for PMIC_BB_GPO1 and PMIC_BB_GPO2.
 * This function configures the deglitch time for PMIC_BB_GPO1 and PMIC_BB_GPO2
 * and then retrieves the configured values for verification.
 *
 * @param  void
 * @return NULL
 */
void test_pmic_gpo12setdgl() {
  int32_t pmicStatus = PMIC_ST_SUCCESS;
  Pmic_GpioRdbkDglCfg_t GpioRdbkDglCfg;
  GpioRdbkDglCfg.gpo1RDglConfig = 1;
  GpioRdbkDglCfg.gpo1RDglData = 0;
  GpioRdbkDglCfg.gpo2RDglConfig = 1;
  GpioRdbkDglCfg.gpo2RDglData = 1;
  pmicStatus = Pmic_gpo12SetDeglitchTime(pPmicCoreHandle_gpio, &GpioRdbkDglCfg);
  if (PMIC_ST_SUCCESS != pmicStatus) {
    DebugP_log("%s(): %d: FAILED with status: %d\n", __func__, __LINE__,
               pmicStatus);
  }
  test_get_gpo12_deglitch_time();
}

/**
 * @brief Test function to retrieve deglitch time for PMIC_BB_GPO1 and
 * PMIC_BB_GPO2. This function retrieves the configured deglitch time for
 * PMIC_BB_GPO1 and PMIC_BB_GPO2 and logs the values for verification.
 *
 * @param  void
 * @return NULL
 */
void test_get_gpo12_deglitch_time() {
  int32_t status = PMIC_ST_SUCCESS;
  Pmic_GpioRdbkDglCfg_t GpioRdbkDglCfg;

  DebugP_log(
      "Testing PMIC_BB_GPO1 and PMIC_BB_GPO2 deglitch time retrieval...\r\n");

  status = Pmic_gpo12GetDeglitchTime(pPmicCoreHandle_gpio, &GpioRdbkDglCfg);

  if (PMIC_ST_SUCCESS != status) {
    DebugP_log("Failed to retrieve PMIC_BB_GPO1 and PMIC_BB_GPO2 deglitch "
               "time: %d\r\n",
               status);
  } else {
    DebugP_log("GPO1RDglData: %x\r\nGPO2RDglData: %x\r\n",
               GpioRdbkDglCfg.gpo1RDglData, GpioRdbkDglCfg.gpo2RDglData);
  }
}

/**
 * @brief Test function to retrieve GPO configuration.
 * This function retrieves the configuration of a specified GPO pin and
 * logs the values for verification.
 *
 * @param  void
 * @return NULL
 */
void test_gpoGetConfiguration() {
  int32_t status = PMIC_ST_SUCCESS;
  Pmic_GpioRdbkDglCfg_t *GpioRdbkDglCfg = NULL;
  Pmic_GpioCfg_t *pGpioCfg = NULL;
  uint8_t pin = PMIC_GPO3;

  status = Pmic_gpioGetConfiguration(pPmicCoreHandle_gpio, pin, pGpioCfg,
                                     GpioRdbkDglCfg);
  if (PMIC_ST_SUCCESS != status) {
    DebugP_log("2nd Failed set GPO value %d\r\n", status);
  }
  DebugP_log("PMIC_BB_GPO1:%x\r\n", pGpioCfg->gpo1Cfg);
  DebugP_log("PMIC_BB_GPO2:%x\r\n", pGpioCfg->gpo2Cfg);
  DebugP_log("PMIC_BB_GPO3:%x\r\n", pGpioCfg->gpo3Cfg);
  DebugP_log("PMIC_BB_GPO4:%x\r\n", pGpioCfg->gpo4Cfg);
  DebugP_log("GPO pin state:%x\r\n", pGpioCfg->pinDir);
}

/**
 * @brief Test function to retrieve GPI configuration.
 * This function retrieves the configuration of a specified GPI pin and
 * logs the values for verification.
 *
 * @param  void
 * @return NULL
 */
void test_gpiGetConfiguration() {
  int32_t status = PMIC_ST_SUCCESS;
  // Pmic_GpioRdbkDglCfg_t *GpioRdbkDglCfg = NULL;
  Pmic_GpioCfg_t *pGpioCfg = NULL;
  uint8_t pin = PMIC_GPI4;

  status = Pmic_gpiGetConfiguration(pPmicCoreHandle_gpio, pin, pGpioCfg);
  if (PMIC_ST_SUCCESS != status) {
    DebugP_log("2nd Failed set GPO value %d\r\n", status);
  }
  DebugP_log("GPI1:%x\r\n", pGpioCfg->gpi1Cfg);
  DebugP_log("GPI4:%x\r\n", pGpioCfg->gpi4Cfg);
}

/**
 * @brief Test function to set GPO configuration.
 * This function sets the configuration for a specified GPO pin and then
 * retrieves the configured values for verification.
 *
 * @param  void
 * @return NULL
 */
void test_gpoSetConfiguration() {
  int32_t status = PMIC_ST_SUCCESS;
  Pmic_GpioCfg_t pGpioCfg;
  uint8_t pin = PMIC_GPO3;
  pGpioCfg.gpo3Cfg = HIGH_LEVEL;
  pGpioCfg.pinDir = INTL_PULL_UP;

  status = Pmic_gpioSetConfiguration(pPmicCoreHandle_gpio, pin, pGpioCfg);
  if (PMIC_ST_SUCCESS != status) {
    DebugP_log("Failed set GPO value %d\r\n", status);
  }

  test_gpoGetConfiguration();
}

/**
 * @brief Test function to set GPI configuration.
 * This function sets the configuration for a specified GPI pin and then
 * retrieves the configured values for verification.
 *
 * @param  void
 * @return NULL
 */
void test_gpiSetConfiguration() {
  int32_t status = PMIC_ST_SUCCESS;
  Pmic_GpioCfg_t pGpioCfg;
  uint8_t pin = PMIC_GPI4;
  pGpioCfg.gpi4Cfg = WD_IN;

  status = Pmic_gpiSetConfiguration(pPmicCoreHandle_gpio, pin, pGpioCfg);
  if (PMIC_ST_SUCCESS != status) {
    DebugP_log("Failed set GPO value %d\r\n", status);
  }

  test_gpiGetConfiguration();
}

/**
 * @brief Main test function for PMIC GPIO module.
 * This function initializes the necessary drivers and configurations for
 * testing the PMIC GPIO module. It performs various tests related to
 * GPIO configuration and deglitch time settings.
 *
 * @param args Unused argument
 * @return NULL
 */
void *test_pmic_gpio(void *args) {
  Drivers_open();
  Board_driversOpen();

  mcspi_mux_pmic();

  /* Initialization */
  DebugP_log("Initializing...\r\n");
  test_pmic_gpio_config_init();
  DebugP_log("Initialization Completed!\r\n\n");

  /* Lock config register initially */
  DebugP_log("[INIT] Configuration Register Lock Sequence:\r\n");
  test_pmic_LockUnlock(pPmicCoreHandle_gpio, 0);

  /*API to unlock configuration register */
  DebugP_log("[INIT] Configuration Register Unlock Sequence:\r\n");
  test_pmic_LockUnlock(pPmicCoreHandle_gpio, 1);

  test_pmic_gpo12setdgl();
  test_pmic_gpo34setdgl();
  test_gpiSetConfiguration();

  /* Deinitialization */
  test_pmic_gpio_config_deinit();
  DebugP_log("Deinitialization Completed\r\n");

  Board_driversClose();
  Drivers_close();

  return NULL;
}
