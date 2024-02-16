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
 *  @file  pmic_core.c
 *
 *  @brief This file contains PMIC generic driver APIs
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include "pmic_core.h"
#include "pmic_core_priv.h"
#include "pmic_core_tps65386x.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */
static const Pmic_DevSubSysInfo_t pmicSubSysInfo[] = {
    /* PMIC_DEV_BB_TPS65386x */
    {.gpioEnable = (bool)true,
     .rtcEnable = (bool)true,
     .wdgEnable = (bool)true,
     .buckEnable = (bool)true,
     .ldoEnable = (bool)true,
     .esmEnable = (bool)true}};

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

/**
 * @brief Check if a specific bit position in a parameter validity value is set.
 * This function checks whether a specific bit position in a parameter validity
 * value is set.
 *
 * @param validParamVal Validity parameter value to check.
 * @param bitPos Bit position to check.
 * @return bool True if the specified bit is set, false otherwise.
 */
bool pmic_validParamCheck(uint32_t validParamVal, uint8_t bitPos) {
  bool retVal = (bool)false;

  if (((validParamVal >> bitPos) & 0x01U) != 0U) {
    retVal = (bool)true;
  }

  return retVal;
}

/**
 * @brief Start a critical section for PMIC operations.
 * This function starts a critical section for PMIC operations, if the critical
 * section start function pointer is not NULL.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle structure.
 * @return void No return value.
 */
void Pmic_criticalSectionStart(const Pmic_CoreHandle_t *pPmicCoreHandle) {
  if (NULL != pPmicCoreHandle->pFnPmicCritSecStart) {
    pPmicCoreHandle->pFnPmicCritSecStart();
  }
}

/**
 * @brief Stop a critical section for PMIC operations.
 * This function stops a critical section for PMIC operations, if the critical
 * section stop function pointer is not NULL.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle structure.
 * @return void No return value.
 */
void Pmic_criticalSectionStop(const Pmic_CoreHandle_t *pPmicCoreHandle) {
  if (NULL != pPmicCoreHandle->pFnPmicCritSecStop) {
    pPmicCoreHandle->pFnPmicCritSecStop();
  }
}

/**
 * @brief Set register lock/unlock configuration.
 * This function sets the register lock/unlock configuration based on the
 * provided parameters.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle structure.
 * @param commonCtrlCfg Common control configuration structure.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
int32_t Pmic_setRegisterLockUnlock(Pmic_CoreHandle_t *pPmicCoreHandle,
                                   const Pmic_CommonCtrlCfg_t commonCtrlCfg) {
  int32_t pmicStatus = PMIC_ST_SUCCESS;

  if (PMIC_ST_SUCCESS == pmicStatus) {
    Pmic_criticalSectionStart(pPmicCoreHandle);

    pmicStatus = Pmic_commIntf_sendByte(
        pPmicCoreHandle, PMIC_REGISTER_UNLOCK_REGADDR, commonCtrlCfg.regLock_1);

    pmicStatus = Pmic_commIntf_sendByte(
        pPmicCoreHandle, PMIC_REGISTER_UNLOCK_REGADDR, commonCtrlCfg.regLock_2);

    Pmic_criticalSectionStop(pPmicCoreHandle);
  }

  return pmicStatus;
}

/**
 * @brief Set counter lock/unlock configuration.
 * This function sets the counter lock/unlock configuration based on the
 * provided parameters.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle structure.
 * @param commonCtrlCfg Common control configuration structure.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
int32_t Pmic_setCounterLockUnlock(Pmic_CoreHandle_t *pPmicCoreHandle,
                                  const Pmic_CommonCtrlCfg_t commonCtrlCfg) {
  int32_t pmicStatus = PMIC_ST_SUCCESS;

  if (PMIC_ST_SUCCESS == pmicStatus) {
    Pmic_criticalSectionStart(pPmicCoreHandle);

    pmicStatus =
        Pmic_commIntf_sendByte(pPmicCoreHandle, PMIC_TMR_COUNTER_UNLOCK_REGADDR,
                               commonCtrlCfg.cntLock_1);

    pmicStatus =
        Pmic_commIntf_sendByte(pPmicCoreHandle, PMIC_TMR_COUNTER_UNLOCK_REGADDR,
                               commonCtrlCfg.cntLock_2);

    Pmic_criticalSectionStop(pPmicCoreHandle);
  }

  return pmicStatus;
}

/**
 * @brief Get register lock status.
 * This function retrieves the register lock status.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle structure.
 * @param pCommonCtrlStat Pointer to the common control status structure to
 * store the retrieved status.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
int32_t Pmic_getRegLockStat(Pmic_CoreHandle_t *pPmicCoreHandle,
                            Pmic_CommonCtrlStat_t *pCommonCtrlStat) {
  int32_t pmicStatus = PMIC_ST_SUCCESS;

  Pmic_criticalSectionStart(pPmicCoreHandle);
  pmicStatus =
      Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_REGISTER_LOCK_STATUS_REGADDR,
                             &pCommonCtrlStat->cfgregLockStat);
  Pmic_criticalSectionStop(pPmicCoreHandle);

  if (PMIC_ST_SUCCESS == pmicStatus) {
    pCommonCtrlStat->cfgregLockStat =
        Pmic_getBitField(pCommonCtrlStat->cfgregLockStat,
                         PMIC_REGISTER_CFG_REG_LOCKED_STATUS_SHIFT,
                         PMIC_REGISTER_LOCK_CFG_REG_LOCKED_STATUS_READ_MASK);
  }

  return pmicStatus;
}

/**
 * @brief Get timer counter lock status.
 * This function retrieves the timer counter lock status.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle structure.
 * @param pCommonCtrlStat Pointer to the common control status structure to
 * store the retrieved status.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
int32_t Pmic_getTmrCntLockStat(Pmic_CoreHandle_t *pPmicCoreHandle,
                               Pmic_CommonCtrlStat_t *pCommonCtrlStat) {
  int32_t pmicStatus = PMIC_ST_SUCCESS;

  Pmic_criticalSectionStart(pPmicCoreHandle);
  pmicStatus =
      Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_REGISTER_LOCK_STATUS_REGADDR,
                             &pCommonCtrlStat->cntregLockStat);
  Pmic_criticalSectionStop(pPmicCoreHandle);

  if (PMIC_ST_SUCCESS == pmicStatus) {
    pCommonCtrlStat->cntregLockStat =
        Pmic_getBitField(pCommonCtrlStat->cntregLockStat,
                         PMIC_REGISTER_CNT_REG_LOCKED_STATUS_SHIFT,
                         PMIC_REGISTER_LOCK_CNT_REG_LOCKED_STATUS_READ_MASK);
  }

  return pmicStatus;
}

/**
 * @brief Get the MCU reset counter value.
 * This function retrieves the MCU reset counter value.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle structure.
 * @param pRecovCntVal Pointer to store the recovered counter value.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
int32_t Pmic_getRstmcuCnt(Pmic_CoreHandle_t *pPmicCoreHandle,
                          uint8_t *pRecovCntVal) {
  int32_t pmicStatus = PMIC_ST_SUCCESS;
  uint8_t regVal = 0U;

  if (NULL == pPmicCoreHandle) {
    pmicStatus = PMIC_ST_ERR_INV_HANDLE;
  }

  if ((PMIC_ST_SUCCESS == pmicStatus) && (NULL == pRecovCntVal)) {
    pmicStatus = PMIC_ST_ERR_NULL_PARAM;
  }

  if (PMIC_ST_SUCCESS == pmicStatus) {
    /* Start Critical Section */
    Pmic_criticalSectionStart(pPmicCoreHandle);

    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        PMIC_STATE_STAT_REGADDR, &regVal);

    /* Stop Critical Section */
    Pmic_criticalSectionStop(pPmicCoreHandle);
  }

  if (PMIC_ST_SUCCESS == pmicStatus) {
    *pRecovCntVal = Pmic_getBitField(regVal, PMIC_STATE_STAT_RST_MCU_CNT_SHIFT,
                                     PMIC_STATE_STAT_RST_MCU_CNT_MASK);
  }

  return pmicStatus;
}

/**
 * @brief Set state status register.
 * This function sets the state status register based on the provided
 * parameters.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle structure.
 * @param pCommonCtrlStat Pointer to the common control status structure
 * containing the data to set.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
int32_t Pmic_setStateStatReg(Pmic_CoreHandle_t *pPmicCoreHandle,
                             Pmic_CommonStateStat_t *pCommonCtrlStat) {
  int32_t pmicStatus = PMIC_ST_SUCCESS;

  if (pCommonCtrlStat != NULL) {
    uint8_t regData = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);
    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        PMIC_STATE_STAT_REGADDR, &regData);

    if (PMIC_ST_SUCCESS == pmicStatus) {
      /* Set RST_MCU_CNT bits (7-6) if pCommonCtrlStat->rstMcuCnt is not NULL */
      if (pCommonCtrlStat->rstMcuCnt != NULL) {
        Pmic_setBitField(&regData, PMIC_RST_MCU_CNT_SHIFT,
                         PMIC_RST_MCU_CNT_MASK, *(pCommonCtrlStat->rstMcuCnt));
      }

      /* Set RST_MCU_RQ_FLAG bit (5) if pCommonCtrlStat->rstMcuRqFlag is not
       * NULL */
      if (pCommonCtrlStat->rstMcuRqFlag != NULL) {
        Pmic_setBitField(&regData, PMIC_RST_MCU_RQ_FLAG_SHIFT,
                         PMIC_RST_MCU_RQ_FLAG_MASK,
                         *(pCommonCtrlStat->rstMcuRqFlag));
      }
    }

    /* Update the modified STATE_STAT register */
    pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                        PMIC_STATE_STAT_REGADDR, regData);

    Pmic_criticalSectionStop(pPmicCoreHandle);
  } else {
    /* User did not provide necessary data */
    pmicStatus = PMIC_ST_ERR_FAIL;
  }
  return pmicStatus;
}

/**
 * @brief Set state control register.
 * This function sets the state control register based on the provided
 * parameters.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle structure.
 * @param pCommonStateCtrl Pointer to the common state control structure
 * containing the data to set.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
int32_t Pmic_setStateCtrlReg(Pmic_CoreHandle_t *pPmicCoreHandle,
                             Pmic_CommonStateCtrl_t *pCommonStateCtrl) {
  int32_t pmicStatus = PMIC_ST_SUCCESS;
  if (pCommonStateCtrl != NULL) {
    uint8_t regData = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);
    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        PMIC_STATE_CTRL_REGADDR, &regData);
    Pmic_criticalSectionStop(pPmicCoreHandle);

    if (PMIC_ST_SUCCESS == pmicStatus) {
      if (pCommonStateCtrl->state_req != NULL) {
        Pmic_setBitField(&regData, PMIC_STATE_CTRL_STATE_REQ_SHIFT,
                         PMIC_STATE_CTRL_STATE_REQ_MASK,
                         pCommonStateCtrl->state_req);
      }
    } else {
      pmicStatus = PMIC_ST_ERR_FAIL;
    }
  } else {
    pmicStatus = PMIC_ST_ERR_INV_PARAM;
  }
  return pmicStatus;
}

/**
 * @brief Get state control register.
 * This function retrieves the state control register value.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle structure.
 * @param pCommonStateCtrl Pointer to the common state control structure to
 * store the retrieved value.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
int32_t Pmic_getStateCtrlReg(Pmic_CoreHandle_t *pPmicCoreHandle,
                             Pmic_CommonStateCtrl_t *pCommonStateCtrl) {
  int32_t pmicStatus = PMIC_ST_SUCCESS;
  if (pCommonStateCtrl != NULL) {
    uint8_t regData = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);
    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        PMIC_STATE_CTRL_REGADDR, &regData);
    Pmic_criticalSectionStop(pPmicCoreHandle);

    if (PMIC_ST_SUCCESS == pmicStatus) {
      if (pCommonStateCtrl->state_req != NULL) {
        pCommonStateCtrl->state_req =
            Pmic_getBitField(regData, PMIC_STATE_CTRL_STATE_REQ_SHIFT,
                             PMIC_STATE_CTRL_STATE_REQ_MASK);
      }
    } else {
      pmicStatus = PMIC_ST_ERR_FAIL;
    }
  } else {
    pmicStatus = PMIC_ST_ERR_INV_PARAM;
  }
  return pmicStatus;
}

/**
 * @brief Get state status register.
 * This function retrieves the state status register value.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle structure.
 * @param pCommonCtrlStat Pointer to the common control status structure to
 * store the retrieved value.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
int32_t Pmic_getStateStatReg(Pmic_CoreHandle_t *pPmicCoreHandle,
                             Pmic_CommonStateStat_t *pCommonCtrlStat) {
  int32_t pmicStatus = PMIC_ST_SUCCESS;

  /* Check if user provides storage for the STATE_STAT data */
  if (pCommonCtrlStat != NULL) {
    uint8_t regData = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);
    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        PMIC_STATE_STAT_REGADDR, &regData);
    Pmic_criticalSectionStop(pPmicCoreHandle);

    if (PMIC_ST_SUCCESS == pmicStatus) {
      /* Extract RST_MCU_CNT bits (7-6) into pCommonCtrlStat->rstMcuCnt if not
       * NULL */
      if (pCommonCtrlStat->rstMcuCnt != NULL) {
        *(pCommonCtrlStat->rstMcuCnt) = Pmic_getBitField(
            regData, PMIC_RST_MCU_CNT_SHIFT, PMIC_RST_MCU_CNT_MASK);
      }

      /* Extract RST_MCU_RQ_FLAG bit (5) into pCommonCtrlStat->rstMcuRqFlag if
       * not NULL */
      if (pCommonCtrlStat->rstMcuRqFlag != NULL) {
        *(pCommonCtrlStat->rstMcuRqFlag) = Pmic_getBitField(
            regData, PMIC_RST_MCU_RQ_FLAG_SHIFT, PMIC_RST_MCU_RQ_FLAG_MASK);
      }

      /* Extract PWRD_DLY_ACTV bit (4) into pCommonCtrlStat->pwrdDlyActv if not
       * NULL */
      if (pCommonCtrlStat->pwrdDlyActv != NULL) {
        *(pCommonCtrlStat->pwrdDlyActv) = Pmic_getBitField(
            regData, PMIC_PWRD_DLY_ACTV_SHIFT, PMIC_PWRD_DLY_ACTV_MASK);
      }

      /* Extract STATE bits (3-0) into pCommonCtrlStat->state if not NULL */
      if (pCommonCtrlStat->state != NULL) {
        *(pCommonCtrlStat->state) =
            Pmic_getBitField(regData, PMIC_STATE_SHIFT, PMIC_STATE_MASK);
      }
    }
  } else {
    /* User did not provide storage for the retrieved data */
    pmicStatus = PMIC_ST_ERR_INV_PARAM;
  }
  return pmicStatus;
}

/**
 * @brief Initialize the PMIC core handle with basic device configuration
 * parameters. This function initializes the PMIC core handle with basic device
 * configuration parameters.
 *
 * @param pPmicConfigData Pointer to the PMIC core configuration data structure.
 * @param pPmicCoreHandle Pointer to the PMIC core handle structure to be
 * initialized.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
static int32_t
Pmic_initCoreHandleBasicDevCfgParams(const Pmic_CoreCfg_t *pPmicConfigData,
                                     Pmic_CoreHandle_t *pPmicCoreHandle) {
  int32_t pmicStatus = PMIC_ST_SUCCESS;

  /* Check and update PMIC Handle device type */
  if (((bool)true) == pmic_validParamCheck(pPmicConfigData->validParams,
                                           PMIC_CFG_DEVICE_TYPE_VALID)) {
    if (PMIC_DEV_BB_TPS65386X != pPmicConfigData->pmicDeviceType) {
      pmicStatus = PMIC_ST_ERR_INV_PARAM;
    }

    if (PMIC_ST_SUCCESS == pmicStatus) {
      pPmicCoreHandle->pmicDeviceType = pPmicConfigData->pmicDeviceType;
    }
  }

  /* Check and update PMIC Handle Comm Mode */
  if ((PMIC_ST_SUCCESS == pmicStatus) &&
      (((bool)true) == pmic_validParamCheck(pPmicConfigData->validParams,
                                            PMIC_CFG_COMM_MODE_VALID))) {
    if (PMIC_INTF_SPI != pPmicConfigData->commMode) {
      pmicStatus = PMIC_ST_ERR_INV_PARAM;
    } else {
      pPmicCoreHandle->commMode = pPmicConfigData->commMode;
    }
    if (PMIC_ST_SUCCESS != pmicStatus) {
      pmicStatus = PMIC_ST_ERR_FAIL;
    }
  }
  return pmicStatus;
}

/**
 * @brief Initialize the PMIC core handle with communication I/O critical
 * section function pointers. This function initializes the PMIC core handle
 * with communication I/O critical section function pointers.
 *
 * @param pPmicConfigData Pointer to the PMIC core configuration data structure.
 * @param pPmicCoreHandle Pointer to the PMIC core handle structure to be
 * initialized.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
static int32_t Pmic_initCoreHandleCommIOCriticalSectionFns(
    const Pmic_CoreCfg_t *pPmicConfigData, Pmic_CoreHandle_t *pPmicCoreHandle) {
  int32_t pmicStatus = PMIC_ST_SUCCESS;

  /* Check and update PMIC Handle Comm IO RD Fn */
  if (((bool)true) == pmic_validParamCheck(pPmicConfigData->validParams,
                                           PMIC_CFG_COMM_IO_RD_VALID)) {
    if (NULL == pPmicConfigData->pFnPmicCommIoRead) {
      pmicStatus = PMIC_ST_ERR_NULL_FPTR;
    } else {
      pPmicCoreHandle->pFnPmicCommIoRead = pPmicConfigData->pFnPmicCommIoRead;
    }
  }

  /* Check and update PMIC Handle Comm IO WR Fn */
  if ((PMIC_ST_SUCCESS == pmicStatus) &&
      (((bool)true) == pmic_validParamCheck(pPmicConfigData->validParams,
                                            PMIC_CFG_COMM_IO_WR_VALID))) {
    if (NULL == pPmicConfigData->pFnPmicCommIoWrite) {
      pmicStatus = PMIC_ST_ERR_NULL_FPTR;
    } else {
      pPmicCoreHandle->pFnPmicCommIoWrite = pPmicConfigData->pFnPmicCommIoWrite;
    }
  }

  /* Check and update PMIC Handle Critical Section Start Fn */
  if ((PMIC_ST_SUCCESS == pmicStatus) &&
      (((bool)true) == pmic_validParamCheck(pPmicConfigData->validParams,
                                            PMIC_CFG_CRITSEC_START_VALID))) {
    if (NULL == pPmicConfigData->pFnPmicCritSecStart) {
      pmicStatus = PMIC_ST_ERR_NULL_FPTR;
    } else {
      pPmicCoreHandle->pFnPmicCritSecStart =
          pPmicConfigData->pFnPmicCritSecStart;
    }
  }

  /* Check and update PMIC Handle Critical Section Stop Fn */
  if ((PMIC_ST_SUCCESS == pmicStatus) &&
      (((bool)true) == pmic_validParamCheck(pPmicConfigData->validParams,
                                            PMIC_CFG_CRITSEC_STOP_VALID))) {
    if (NULL == pPmicConfigData->pFnPmicCritSecStop) {
      pmicStatus = PMIC_ST_ERR_NULL_FPTR;
    } else {
      pPmicCoreHandle->pFnPmicCritSecStop = pPmicConfigData->pFnPmicCritSecStop;
    }
  }
  return pmicStatus;
}

/**
 * @brief Validate the presence of the device on the bus.
 * This function validates the presence of the device on the bus.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle structure.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
int32_t Pmic_validateDevOnBus(Pmic_CoreHandle_t *pPmicCoreHandle) {
  int32_t pmicStatus = PMIC_ST_SUCCESS;
  uint8_t regVal = 0U;
  if (PMIC_ST_SUCCESS != pmicStatus) {
    pmicStatus = PMIC_ST_ERR_FAIL;
  }

  /* Start Critical Section */
  Pmic_criticalSectionStart(pPmicCoreHandle);

  pmicStatus =
      Pmic_commIntf_sendByte(pPmicCoreHandle, PMIC_DEV_ID_REGADDR, regVal);

  if (PMIC_ST_SUCCESS != pmicStatus) {
    pmicStatus = PMIC_ST_ERR_FAIL;
  }

  pmicStatus =
      Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_DEV_ID_REGADDR, &regVal);

  Pmic_criticalSectionStop(pPmicCoreHandle);
  if (PMIC_ST_SUCCESS != pmicStatus) {
    pmicStatus = PMIC_ST_ERR_FAIL;
  }
  if (PMIC_ST_SUCCESS == pmicStatus) {
    pPmicCoreHandle->pmicDevRev =
        Pmic_getBitField(regVal, PMIC_DEV_ID_SHIFT, PMIC_DEV_ID_MASK);

    /* Validate if the device requested is the one on the bus */
    if (PMIC_DEV_BB_TPS65386X == (pPmicCoreHandle->pmicDeviceType)) {
      if (PMIC_TPS65386X_DEV_ID != pPmicCoreHandle->pmicDevRev) {
        pmicStatus = PMIC_ST_WARN_INV_DEVICE_ID;
      }
    }
  }
  if (PMIC_ST_SUCCESS != pmicStatus) {
    pmicStatus = PMIC_ST_ERR_FAIL;
  }
  return pmicStatus;
}

/**
 * @brief Update subsystem information and validate main Q&A communication
 * interface read/write.
 * This function updates subsystem information and validates the main Q&A
 * communication interface read/write.
 *
 * @param pPmicConfigData Pointer to the PMIC core configuration data structure.
 * @param pPmicCoreHandle Pointer to the PMIC core handle structure to be
 * updated.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
static int32_t Pmic_updateSubSysInfoValidateMainQaCommIFRdWr(
    const Pmic_CoreCfg_t *pPmicConfigData, Pmic_CoreHandle_t *pPmicCoreHandle) {
  int32_t pmicStatus = PMIC_ST_SUCCESS;
  uint8_t regVal = 0U;

  /* Update PMIC subsystem info to PMIC handle */
  pPmicCoreHandle->pPmic_SubSysInfo =
      (&pmicSubSysInfo[pPmicCoreHandle->pmicDeviceType]);

  if (PMIC_ST_SUCCESS != pmicStatus) {
    pmicStatus = PMIC_ST_ERR_FAIL;
  }

  if (PMIC_ST_SUCCESS == pmicStatus) {
    /* Start Critical Section */
    Pmic_criticalSectionStart(pPmicCoreHandle);

    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        PMIC_WD_LONGWIN_CFG_REGADDR, &regVal);

    Pmic_criticalSectionStop(pPmicCoreHandle);
    if (PMIC_ST_SUCCESS != pmicStatus) {
      pmicStatus = PMIC_ST_ERR_FAIL;
    }
    if (PMIC_ST_SUCCESS == pmicStatus) {
      pPmicCoreHandle->drvInitStatus |= pPmicConfigData->instType;
    }
  }
  if (PMIC_ST_SUCCESS != pmicStatus) {
    pmicStatus = PMIC_ST_ERR_FAIL;
  }
  return pmicStatus;
}

/**
 * @brief Initialize the PMIC core.
 * This function initializes the PMIC core based on the provided configuration
 * data.
 *
 * @param pPmicConfigData Pointer to the PMIC core configuration data structure.
 * @param pPmicCoreHandle Pointer to the PMIC core handle structure to be
 * initialized.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
int32_t Pmic_init(const Pmic_CoreCfg_t *pPmicConfigData,
                  Pmic_CoreHandle_t *pPmicCoreHandle) {
  int32_t pmicStatus = PMIC_ST_SUCCESS;

  if ((NULL == pPmicCoreHandle) || (NULL == pPmicConfigData)) {
    pmicStatus = PMIC_ST_ERR_NULL_PARAM;
  }
  if (PMIC_ST_SUCCESS != pmicStatus) {
    pmicStatus = PMIC_ST_ERR_FAIL;
  }

  /* Check and update PMIC Handle for device type, Comm Mode,
   * Main Slave Address and NVM Slave Address */
  if (PMIC_ST_SUCCESS == pmicStatus) {
    pmicStatus =
        Pmic_initCoreHandleBasicDevCfgParams(pPmicConfigData, pPmicCoreHandle);
  }
  if (PMIC_ST_SUCCESS != pmicStatus) {
    pmicStatus = PMIC_ST_ERR_FAIL;
  }
  /* Check and update PMIC Handle for Comm IO RD Fn, Comm IO Wr Fn,
   * Critical Section Start Fn and Critical Section Stop Fn */
  if (PMIC_ST_SUCCESS == pmicStatus) {
    pmicStatus = Pmic_initCoreHandleCommIOCriticalSectionFns(pPmicConfigData,
                                                             pPmicCoreHandle);
  }
  if (PMIC_ST_SUCCESS != pmicStatus) {
    pmicStatus = PMIC_ST_ERR_FAIL;
  }

  /* Check for required members for I2C/SPI Main handle comm */
  if ((PMIC_ST_SUCCESS == pmicStatus) &&
      ((NULL == pPmicCoreHandle->pFnPmicCritSecStart) ||
       (NULL == pPmicCoreHandle->pFnPmicCritSecStop) ||
       (NULL == pPmicCoreHandle->pFnPmicCommIoRead) ||
       (NULL == pPmicCoreHandle->pFnPmicCommIoWrite))) {
    pmicStatus = PMIC_ST_ERR_INSUFFICIENT_CFG;
  }

  return pmicStatus;
}

/**
 * @brief Deinitialize the PMIC core.
 * This function deinitializes the PMIC core.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle structure.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
int32_t Pmic_deinit(Pmic_CoreHandle_t *pPmicCoreHandle) {
  int32_t pmicStatus = PMIC_ST_SUCCESS;

  if (NULL == pPmicCoreHandle) {
    pmicStatus = PMIC_ST_ERR_INV_HANDLE;
  }

  if (PMIC_ST_SUCCESS == pmicStatus) {
    pPmicCoreHandle->pCommHandle = NULL;
    pPmicCoreHandle->pQACommHandle = NULL;
    pPmicCoreHandle->pFnPmicCritSecStart = NULL;
    pPmicCoreHandle->pFnPmicCritSecStop = NULL;
    pPmicCoreHandle->pFnPmicCommIoRead = NULL;
    pPmicCoreHandle->pFnPmicCommIoWrite = NULL;
    pPmicCoreHandle->pPmic_SubSysInfo = NULL;
    pPmicCoreHandle->drvInitStatus = 0x00U;
  }

  return pmicStatus;
}

/**
 * @brief Get the address of the scratch pad register.
 * This function retrieves the address of the scratch pad register based on the
 * specified register ID.
 *
 * @param scratchPadRegId Scratch pad register ID.
 * @param pRegAddr Pointer to store the address of the scratch pad register.
 * @return void
 */
static void Pmic_getScratchPadRegAddr(uint8_t scratchPadRegId,
                                      uint8_t *pRegAddr) {
  switch (scratchPadRegId) {
  case PMIC_SCRATCH_PAD_REG_1:
    *pRegAddr = PMIC_CUSTOMER_SCRATCH1_REGADDR;
    break;
  case PMIC_SCRATCH_PAD_REG_2:
    *pRegAddr = PMIC_CUSTOMER_SCRATCH2_REGADDR;
    break;
  default:
    *pRegAddr = PMIC_INVALID_REGADDR;
    break;
  }
}

/**
 * @brief Set value to the scratch pad register.
 * This function sets a value to the scratch pad register.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle structure.
 * @param scratchPadRegId Scratch pad register ID.
 * @param data Data to be written to the scratch pad register.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
int32_t Pmic_setScratchPadValue(Pmic_CoreHandle_t *pPmicCoreHandle,
                                const uint8_t scratchPadRegId,
                                const uint8_t data) {
  int32_t pmicStatus = PMIC_ST_SUCCESS;
  uint8_t regAddr;

  if (NULL == pPmicCoreHandle) {
    pmicStatus = PMIC_ST_ERR_INV_HANDLE;
  }

  if ((PMIC_ST_SUCCESS == pmicStatus) &&
      (scratchPadRegId > PMIC_SCRATCH_PAD_REG_2)) {
    pmicStatus = PMIC_ST_ERR_INV_PARAM;
  }
  if (PMIC_ST_SUCCESS == pmicStatus) {
    Pmic_getScratchPadRegAddr(scratchPadRegId, &regAddr);
    Pmic_criticalSectionStart(pPmicCoreHandle);
    if (regAddr != PMIC_INVALID_REGADDR) {
      pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle, regAddr, data);
    }
    Pmic_criticalSectionStop(pPmicCoreHandle);
  }

  return pmicStatus;
}

/**
 * @brief Get value from the scratch pad register.
 * This function retrieves a value from the scratch pad register.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle structure.
 * @param scratchPadRegId Scratch pad register ID.
 * @param pData Pointer to store the retrieved data from the scratch pad
 * register.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
int32_t Pmic_getScratchPadValue(Pmic_CoreHandle_t *pPmicCoreHandle,
                                const uint8_t scratchPadRegId, uint8_t *pData) {
  int32_t pmicStatus = PMIC_ST_SUCCESS;
  uint8_t regAddr;

  if (NULL == pPmicCoreHandle) {
    pmicStatus = PMIC_ST_ERR_INV_HANDLE;
  }

  if ((PMIC_ST_SUCCESS == pmicStatus) &&
      (scratchPadRegId > PMIC_SCRATCH_PAD_REG_2)) {
    pmicStatus = PMIC_ST_ERR_INV_PARAM;
  }

  if ((PMIC_ST_SUCCESS == pmicStatus) && (NULL == pData)) {
    pmicStatus = PMIC_ST_ERR_NULL_PARAM;
  }

  if (PMIC_ST_SUCCESS == pmicStatus) {
    Pmic_getScratchPadRegAddr(scratchPadRegId, &regAddr);

    Pmic_criticalSectionStart(pPmicCoreHandle);
    if (regAddr != PMIC_INVALID_REGADDR) {
      pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, regAddr, pData);
    }
    Pmic_criticalSectionStop(pPmicCoreHandle);
  }

  return pmicStatus;
}

/**
 * @brief Enable or disable spread spectrum.
 * This function enables or disables spread spectrum based on the provided
 * configuration.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param commonCtrlCfg Spread spectrum configuration.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
int32_t Pmic_spreadSpectrumEnable(Pmic_CoreHandle_t *pPmicCoreHandle,
                                  const Pmic_CommonCtrlCfg_t commonCtrlCfg) {
  int32_t pmicStatus = PMIC_ST_SUCCESS;
  uint8_t regData = 0U;

  Pmic_criticalSectionStart(pPmicCoreHandle);

  pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                      PMIC_BUCK_BST_CFG_REGADDR, &regData);

  if (PMIC_ST_SUCCESS == pmicStatus) {
    if (((bool)PMIC_SPREAD_SPECTRUM_CFG_ENABLE) ==
        commonCtrlCfg.sreadSpectrumEn) {
      Pmic_setBitField(&regData, PMIC_DRSS_SS_EN_SHIFT, PMIC_DRSS_SS_EN_MASK,
                       PMIC_SPREAD_SPECTRUM_CFG_ENABLE);
    } else {
      Pmic_setBitField(&regData, PMIC_DRSS_SS_EN_SHIFT, PMIC_DRSS_SS_EN_MASK,
                       PMIC_SPREAD_SPECTRUM_CFG_DISABLE);
    }

    pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                        PMIC_BUCK_BST_CFG_REGADDR, regData);
  }

  Pmic_criticalSectionStop(pPmicCoreHandle);

  return pmicStatus;
}

/**
 * @brief Get the spread spectrum enable status.
 * This function retrieves the status of spread spectrum.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param pCommonCtrlCfg Pointer to store the spread spectrum configuration.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
int32_t Pmic_getSpreadSpectrumEnable(Pmic_CoreHandle_t *pPmicCoreHandle,
                                     Pmic_CommonCtrlCfg_t *pCommonCtrlCfg) {
  int32_t pmicStatus = PMIC_ST_SUCCESS;
  uint8_t regData = 0U;

  Pmic_criticalSectionStart(pPmicCoreHandle);
  pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                      PMIC_BUCK_BST_CFG_REGADDR, &regData);
  Pmic_criticalSectionStop(pPmicCoreHandle);

  if (PMIC_ST_SUCCESS == pmicStatus) {
    pCommonCtrlCfg->sreadSpectrumEn = (bool)false;

    if (Pmic_getBitField(regData, PMIC_DRSS_SS_EN_SHIFT,
                         PMIC_DRSS_SS_EN_MASK) == 1U) {
      pCommonCtrlCfg->sreadSpectrumEn = (bool)true;
    }
  }

  return pmicStatus;
}

/**
 * @brief Set the safe output pin configuration.
 * This function sets the configuration for safe output pin.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param commonCtrlCfg Safe output pin configuration.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
int32_t Pmic_setEnableSafeOutCfg(Pmic_CoreHandle_t *pPmicCoreHandle,
                                 const Pmic_CommonCtrlCfg_t commonCtrlCfg) {
  int32_t pmicStatus = PMIC_ST_SUCCESS;
  uint8_t regData = 0U;

  if ((commonCtrlCfg.eNsafeOut1 > PMIC_PIN_SIGNAL_LEVEL_HIGH) ||
      (commonCtrlCfg.eNsafeOut2 > PMIC_PIN_SIGNAL_LEVEL_HIGH)) {
    pmicStatus = PMIC_ST_ERR_INV_PARAM;
  }

  if (PMIC_ST_SUCCESS == pmicStatus) {
    Pmic_criticalSectionStart(pPmicCoreHandle);
    pmicStatus = Pmic_commIntf_recvByte(
        pPmicCoreHandle, PMIC_SAFE_OUT_CFG_CTRL_REGADDR, &regData);
    if (PMIC_ST_SUCCESS == pmicStatus) {
      Pmic_setBitField(&regData, PMIC_ENABLE_SAFE_OUTEN1_SHIFT,
                       PMIC_ENABLE_SAFE_OUTEN1_MASK, commonCtrlCfg.eNsafeOut1);

      Pmic_setBitField(&regData, PMIC_ENABLE_SAFE_OUTEN2_SHIFT,
                       PMIC_ENABLE_SAFE_OUTEN2_MASK, commonCtrlCfg.eNsafeOut2);

      pmicStatus = Pmic_commIntf_sendByte(
          pPmicCoreHandle, PMIC_SAFE_OUT_CFG_CTRL_REGADDR, regData);
    }
    Pmic_criticalSectionStop(pPmicCoreHandle);
  }

  return pmicStatus;
}

/**
 * @brief Get the safe output pin configuration.
 * This function retrieves the configuration of safe output pin.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param pCommonCtrlCfg Pointer to store the safe output pin configuration.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
int32_t Pmic_getSafeOutPinCfg(Pmic_CoreHandle_t *pPmicCoreHandle,
                              Pmic_CommonCtrlCfg_t *pCommonCtrlCfg) {
  int32_t pmicStatus = PMIC_ST_SUCCESS;
  uint8_t regData = 0U;

  Pmic_criticalSectionStart(pPmicCoreHandle);
  pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                      PMIC_SAFE_OUT_CFG_CTRL_REGADDR, &regData);
  Pmic_criticalSectionStop(pPmicCoreHandle);

  if (PMIC_ST_SUCCESS == pmicStatus) {
    pCommonCtrlCfg->eNsafeOut1 = Pmic_getBitField(
        regData, PMIC_ENABLE_SAFE_OUTEN1_SHIFT, PMIC_ENABLE_SAFE_OUTEN1_MASK);

    pCommonCtrlCfg->eNsafeOut2 = Pmic_getBitField(
        regData, PMIC_ENABLE_SAFE_OUTEN2_SHIFT, PMIC_ENABLE_SAFE_OUTEN2_MASK);
  }

  return pmicStatus;
}

/**
 * @brief Set the safe state timeout configuration.
 * This function sets the safe state timeout configuration.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param safeCfg Safe state timeout configuration.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
int32_t Pmic_setSafeStateTimeoutCfg(Pmic_CoreHandle_t *pPmicCoreHandle,
                                    Pmic_SafeStateCfg_t safeCfg) {
  int32_t pmicStatus = PMIC_ST_SUCCESS;
  uint8_t regData = 0U;

  if (PMIC_ST_SUCCESS == pmicStatus) {
    Pmic_criticalSectionStart(pPmicCoreHandle);
    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        PMIC_SAFE_TMO_CFG_REGADDR, &regData);
    if (PMIC_ST_SUCCESS == pmicStatus) {
      Pmic_getBitField(regData, PMIC_SAFE_TMO_SHIFT, PMIC_SAFE_TMO_MASK);

      Pmic_setBitField(&regData, PMIC_SAFE_TMO_SHIFT, PMIC_SAFE_TMO_MASK,
                       safeCfg.safeStateTMO);

      pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                          PMIC_SAFE_TMO_CFG_REGADDR, regData);
    }
    Pmic_criticalSectionStop(pPmicCoreHandle);
  }

  return pmicStatus;
}

/**
 * @brief Get the safe state timeout configuration.
 * This function retrieves the safe state timeout configuration.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param safeCfg Pointer to store the safe state timeout configuration.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
int32_t Pmic_getSafeStateTimeoutCfg(Pmic_CoreHandle_t *pPmicCoreHandle,
                                    Pmic_SafeStateCfg_t safeCfg) {
  int32_t pmicStatus = PMIC_ST_SUCCESS;
  uint8_t regData = 0U;

  if (PMIC_ST_SUCCESS == pmicStatus) {
    Pmic_criticalSectionStart(pPmicCoreHandle);
    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        PMIC_SAFE_TMO_CFG_REGADDR, &regData);
    if (PMIC_ST_SUCCESS == pmicStatus) {
      safeCfg.safeStateTMO =
          Pmic_getBitField(regData, PMIC_SAFE_TMO_SHIFT, PMIC_SAFE_TMO_MASK);
    }
    Pmic_criticalSectionStop(pPmicCoreHandle);
  }

  return pmicStatus;
}

/**
 * @brief Set the safe state threshold configuration.
 * This function sets the safe state threshold configuration.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param safeCfg Safe state threshold configuration.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
int32_t Pmic_setSafeStateThresholdCfg(Pmic_CoreHandle_t *pPmicCoreHandle,
                                      Pmic_SafeStateCfg_t safeCfg) {
  int32_t pmicStatus = PMIC_ST_SUCCESS;
  uint8_t regData = 0U;

  if (PMIC_ST_SUCCESS == pmicStatus) {
    Pmic_criticalSectionStart(pPmicCoreHandle);
    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        PMIC_SAFE_TMO_CFG_REGADDR, &regData);
    if (PMIC_ST_SUCCESS == pmicStatus) {
      Pmic_getBitField(regData, PMIC_SAFE_LOCK_TH_SHIFT,
                       PMIC_SAFE_LOCK_TH_MASK);

      Pmic_setBitField(&regData, PMIC_SAFE_LOCK_TH_SHIFT,
                       PMIC_SAFE_LOCK_TH_MASK, safeCfg.safeLockThreshold);

      pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                          PMIC_SAFE_TMO_CFG_REGADDR, regData);
    }
    Pmic_criticalSectionStop(pPmicCoreHandle);
  }

  return pmicStatus;
}

/**
 * @brief Get the safe state threshold configuration.
 * This function retrieves the safe state threshold configuration.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param safeCfg Pointer to store the safe state threshold configuration.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
int32_t Pmic_getSafeStateThresholdCfg(Pmic_CoreHandle_t *pPmicCoreHandle,
                                      Pmic_SafeStateCfg_t safeCfg) {
  int32_t pmicStatus = PMIC_ST_SUCCESS;
  uint8_t regData = 0U;

  if (PMIC_ST_SUCCESS == pmicStatus) {
    Pmic_criticalSectionStart(pPmicCoreHandle);
    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        PMIC_SAFE_TMO_CFG_REGADDR, &regData);
    if (PMIC_ST_SUCCESS == pmicStatus) {
      safeCfg.safeLockThreshold = Pmic_getBitField(
          regData, PMIC_SAFE_LOCK_TH_SHIFT, PMIC_SAFE_LOCK_TH_MASK);
    }
    Pmic_criticalSectionStop(pPmicCoreHandle);
  }

  return pmicStatus;
}

/**
 * @brief Set the common control configuration.
 * This function sets the common control configuration.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param CommonCtrlCfg Common control configuration.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
int32_t Pmic_setCommonCtrlConfig(Pmic_CoreHandle_t *pPmicCoreHandle,
                                 const Pmic_CommonCtrlCfg_t CommonCtrlCfg) {
  int32_t pmicStatus = PMIC_ST_SUCCESS;

  if (NULL == pPmicCoreHandle) {
    pmicStatus = PMIC_ST_ERR_INV_HANDLE;
  }

  if ((PMIC_ST_SUCCESS == pmicStatus) &&
      ((bool)true == pmic_validParamCheck(CommonCtrlCfg.sreadSpectrumEn,
                                          PMIC_CFG_SPREAD_SPECTRUM_EN_VALID))) {
    /* Enable/Disable Spread Spectrum */
    pmicStatus = Pmic_spreadSpectrumEnable(pPmicCoreHandle, CommonCtrlCfg);
  }

  if ((PMIC_ST_SUCCESS == pmicStatus) &&
      ((bool)true == (pmic_validParamCheck(CommonCtrlCfg.eNsafeOut1,
                                           PMIC_CFG_ENABLE_SAFEOUT_VALID)) ||
       (pmic_validParamCheck(CommonCtrlCfg.eNsafeOut2,
                             PMIC_CFG_ENABLE_SAFEOUT_VALID)))) {
    /* Set ENABLE_DRV Pin Configuration */
    pmicStatus = Pmic_setEnableSafeOutCfg(pPmicCoreHandle, CommonCtrlCfg);
  }

  if ((PMIC_ST_SUCCESS == pmicStatus) &&
      ((bool)true == (pmic_validParamCheck(CommonCtrlCfg.regLock_1,
                                           PMIC_REGISTER_UNLOCK_DATA1)) ||
       (pmic_validParamCheck(CommonCtrlCfg.regLock_2,
                             PMIC_REGISTER_UNLOCK_DATA2)))) {
    /* Set Register Lock/UnLock Configuration */
    pmicStatus = Pmic_setRegisterLockUnlock(pPmicCoreHandle, CommonCtrlCfg);
  }
  return pmicStatus;
}

/**
 * @brief Get the common control configuration.
 * This function retrieves the common control configuration.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param pCommonCtrlCfg Pointer to store the common control configuration.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
int32_t Pmic_getCommonCtrlConfig(Pmic_CoreHandle_t *pPmicCoreHandle,
                                 Pmic_CommonCtrlCfg_t *pCommonCtrlCfg) {
  int32_t pmicStatus = PMIC_ST_SUCCESS;

  if (NULL == pPmicCoreHandle) {
    pmicStatus = PMIC_ST_ERR_INV_HANDLE;
  }

  if ((PMIC_ST_SUCCESS == pmicStatus) && (NULL == pCommonCtrlCfg)) {
    pmicStatus = PMIC_ST_ERR_NULL_PARAM;
  }

  if ((PMIC_ST_SUCCESS == pmicStatus) &&
      ((bool)true == pmic_validParamCheck(pCommonCtrlCfg->validParams,
                                          PMIC_CFG_SPREAD_SPECTRUM_EN_VALID))) {
    /* Get the status of Spread Spectrum is Enabled/Disabled  */
    pmicStatus = Pmic_getSpreadSpectrumEnable(pPmicCoreHandle, pCommonCtrlCfg);
  }
  return pmicStatus;
}

int32_t Pmic_setAmuxDmuxPinCfg(Pmic_CoreHandle_t *pPmicCoreHandle,
                               const Pmic_DiagOutCfgCtrl_t diagoutCfgCtrl) {
  int32_t pmicStatus = PMIC_ST_SUCCESS;
  uint8_t regData = 0U;
  bool diagoutAMUXEn = diagoutCfgCtrl.DiagOutCtrl_AMUXEn;
  bool diagoutDMUXEn = diagoutCfgCtrl.DiagOutCtrl_DMUXEn;

  Pmic_criticalSectionStart(pPmicCoreHandle);

  pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                      PMIC_DIAG_OUT_CFG_CTR_REGADDR, &regData);

  if ((PMIC_ST_SUCCESS == pmicStatus) &&
      (PMIC_DEV_BB_TPS65386X == pPmicCoreHandle->pmicDeviceType)) {
    if (((bool)PMIC_TPS65386X_DIAG_OUT_AMUX_ENABLE) == diagoutAMUXEn) {
      Pmic_setBitField(&regData, PMIC_DIAG_OUT_CTRL_SHIFT,
                       PMIC_DIAG_OUT_CTRL_MASK,
                       PMIC_TPS65386X_DIAG_OUT_AMUX_ENABLE);
    } else if (((bool)PMIC_TPS65386X_DIAG_OUT_DMUX_ENABLE) == diagoutDMUXEn) {
      Pmic_setBitField(&regData, PMIC_DIAG_OUT_CTRL_SHIFT,
                       PMIC_DIAG_OUT_CTRL_MASK,
                       PMIC_TPS65386X_DIAG_OUT_DMUX_ENABLE);
    } else {
      Pmic_setBitField(&regData, PMIC_DIAG_OUT_CTRL_SHIFT,
                       PMIC_DIAG_OUT_CTRL_MASK,
                       PMIC_TPS65386X_DIAG_OUT_DISABLE);
    }
    pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                        PMIC_DIAG_OUT_CFG_CTR_REGADDR, regData);
  }

  Pmic_criticalSectionStop(pPmicCoreHandle);

  return pmicStatus;
}

int32_t Pmic_getAmuxDmuxPinCfg(Pmic_CoreHandle_t *pPmicCoreHandle,
                               Pmic_DiagOutCfgCtrl_t *pDiagOutCfgCtrl) {
  int32_t pmicStatus = PMIC_ST_SUCCESS;
  uint8_t regData = 0U;

  Pmic_criticalSectionStart(pPmicCoreHandle);
  pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                      PMIC_DIAG_OUT_CFG_CTR_REGADDR, &regData);
  Pmic_criticalSectionStop(pPmicCoreHandle);

  if (PMIC_ST_SUCCESS == pmicStatus) {
    pDiagOutCfgCtrl->DiagOutCtrl = Pmic_getBitField(
        regData, PMIC_DIAG_OUT_CTRL_SHIFT, PMIC_DIAG_OUT_CTRL_MASK);
  }

  return pmicStatus;
}

/**
 * @brief Set the diagnostics output pin control configuration.
 * This function sets the diagnostics output pin control configuration.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param DiagOutCfgCtrl Diagnostics output pin control configuration.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
int32_t Pmic_setDiagOutCtrlConfig(Pmic_CoreHandle_t *pPmicCoreHandle,
                                  const Pmic_DiagOutCfgCtrl_t DiagOutCfgCtrl) {
  int32_t pmicStatus = PMIC_ST_SUCCESS;

  if (NULL == pPmicCoreHandle) {
    pmicStatus = PMIC_ST_ERR_INV_HANDLE;
  }

  if (PMIC_ST_SUCCESS == pmicStatus) {
    /* Set AMUX_OUT/REF_OUT Pin Control Configuration */
    pmicStatus = Pmic_setAmuxDmuxPinCfg(pPmicCoreHandle, DiagOutCfgCtrl);
  }

  return pmicStatus;
}

/**
 * @brief Get the diagnostics output pin control configuration.
 * This function retrieves the diagnostics output pin control configuration.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param pDiagOutCfgCtrl Pointer to store the diagnostics output pin control
 * configuration.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
int32_t Pmic_getDiagOutCtrlConfig(Pmic_CoreHandle_t *pPmicCoreHandle,
                                  Pmic_DiagOutCfgCtrl_t *pDiagOutCfgCtrl) {
  int32_t pmicStatus = PMIC_ST_SUCCESS;

  if (NULL == pPmicCoreHandle) {
    pmicStatus = PMIC_ST_ERR_INV_HANDLE;
  }

  if (PMIC_ST_SUCCESS == pmicStatus) {
    /* Get AMUX/DMUX Pin Control Configuration */
    pmicStatus = Pmic_getAmuxDmuxPinCfg(pPmicCoreHandle, pDiagOutCfgCtrl);
  }

  return pmicStatus;
}

static void Pmic_getPinTypeRegBitFields(const uint8_t pinType,
                                        uint8_t *pBitShift, uint8_t *pBitMask) {
  switch (pinType) {
  case PMIC_PIN_TYPE_NRST_RDBK_LVL:
    *pBitShift = PMIC_STAT_READBACK_ERR_NRST_RDBK_LVL_SHIFT;
    *pBitMask = PMIC_STAT_READBACK_ERR_NRST_RDBK_LVL_MASK;
    break;
  case PMIC_PIN_TYPE_SAFE_OUT1_RDBK_LVL:
    *pBitShift = PMIC_STAT_READBACK_ERR_SAFE_OUT1_RDBK_LVL_SHIFT;
    *pBitMask = PMIC_STAT_READBACK_ERR_SAFE_OUT1_RDBK_LVL_MASK;
    break;
  default:
    *pBitShift = PMIC_STAT_READBACK_ERR_EN_OUT_RDBK_LVL_SHIFT;
    *pBitMask = PMIC_STAT_READBACK_ERR_EN_OUT_RDBK_LVL_MASK;
    break;
  }
}

/**
 * @brief Get the pin value.
 * This function retrieves the value of the specified pin.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param pinType Type of the pin.
 * @param pPinValue Pointer to store the pin value.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
int32_t Pmic_getPinValue(Pmic_CoreHandle_t *pPmicCoreHandle,
                         const uint8_t pinType, uint8_t *pPinValue) {
  int32_t pmicStatus = PMIC_ST_SUCCESS;
  uint8_t regVal;
  uint8_t bitShift, bitMask;

  if (NULL == pPmicCoreHandle) {
    pmicStatus = PMIC_ST_ERR_INV_HANDLE;
  }

  if ((PMIC_ST_SUCCESS == pmicStatus) &&
      (pinType > PMIC_PIN_TYPE_GPO4_RDBK_LVL)) {
    pmicStatus = PMIC_ST_ERR_INV_PARAM;
  }

  if ((PMIC_ST_SUCCESS == pmicStatus) && (NULL == pPinValue)) {
    pmicStatus = PMIC_ST_ERR_NULL_PARAM;
  }

  if (PMIC_ST_SUCCESS == pmicStatus) {
    Pmic_criticalSectionStart(pPmicCoreHandle);
    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        PMIC_RDBK_ERR_STAT_REGADDR, &regVal);
    Pmic_criticalSectionStop(pPmicCoreHandle);
  }

  if (PMIC_ST_SUCCESS == pmicStatus) {
    Pmic_getPinTypeRegBitFields(pinType, &bitShift, &bitMask);

    *pPinValue = Pmic_getBitField(regVal, bitShift, bitMask);
  }

  return pmicStatus;
}

/**
 * @brief Get the status of the safe output 1 pin.
 * This function retrieves the status of the safe output 1 pin.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param pCommonCtrlStat Pointer to store the common control status.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
static int32_t Pmic_getSafeOut1Stat(Pmic_CoreHandle_t *pPmicCoreHandle,
                                    Pmic_CommonCtrlStat_t *pCommonCtrlStat) {
  int32_t pmicStatus = PMIC_ST_SUCCESS;
  uint8_t regData = 0U;

  Pmic_criticalSectionStart(pPmicCoreHandle);
  pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                      PMIC_RDBK_ERR_STAT_REGADDR, &regData);
  Pmic_criticalSectionStop(pPmicCoreHandle);

  if (PMIC_ST_SUCCESS == pmicStatus) {
    pCommonCtrlStat->safeOut1Pin = Pmic_getBitField(
        regData, PMIC_STAT_READBACK_ERR_SAFE_OUT1_RDBK_LVL_SHIFT,
        PMIC_STAT_READBACK_ERR_SAFE_OUT1_RDBK_LVL_MASK);
  }

  return pmicStatus;
}

/**
 * @brief Get the status of the nRST pin.
 * This function retrieves the status of the nRST pin.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param pCommonCtrlStat Pointer to store the common control status.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
static int32_t Pmic_getNRstPinStat(Pmic_CoreHandle_t *pPmicCoreHandle,
                                   Pmic_CommonCtrlStat_t *pCommonCtrlStat) {
  int32_t pmicStatus = PMIC_ST_SUCCESS;
  uint8_t regData = 0U;

  Pmic_criticalSectionStart(pPmicCoreHandle);
  pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                      PMIC_RDBK_ERR_STAT_REGADDR, &regData);
  Pmic_criticalSectionStop(pPmicCoreHandle);

  if (PMIC_ST_SUCCESS == pmicStatus) {
    pCommonCtrlStat->nRstPin =
        Pmic_getBitField(regData, PMIC_STAT_READBACK_ERR_NRST_RDBK_LVL_SHIFT,
                         PMIC_STAT_READBACK_ERR_NRST_RDBK_LVL_MASK);
  }

  return pmicStatus;
}

/**
 * @brief Get the status of the EN_OUT pin.
 * This function retrieves the status of the EN_OUT pin.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param pCommonCtrlStat Pointer to store the common control status.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
static int32_t Pmic_getEnOutPinStat(Pmic_CoreHandle_t *pPmicCoreHandle,
                                    Pmic_CommonCtrlStat_t *pCommonCtrlStat) {
  int32_t pmicStatus = PMIC_ST_SUCCESS;
  uint8_t regData = 0U;

  Pmic_criticalSectionStart(pPmicCoreHandle);
  pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                      PMIC_RDBK_ERR_STAT_REGADDR, &regData);
  Pmic_criticalSectionStop(pPmicCoreHandle);

  if (PMIC_ST_SUCCESS == pmicStatus) {
    pCommonCtrlStat->enOutPin =
        Pmic_getBitField(regData, PMIC_STAT_READBACK_ERR_EN_OUT_RDBK_LVL_SHIFT,
                         PMIC_STAT_READBACK_ERR_EN_OUT_RDBK_LVL_MASK);
  }

  return pmicStatus;
}

/**
 * @brief Get the status of the EN_OUT, nRST, and safe output 1 pins.
 * This function retrieves the status of the EN_OUT, nRST, and safe output 1
 * pins.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param pCommonCtrlStat Pointer to store the common control status.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
static int32_t
Pmic_getEnOutNrstSafeOut1PinStat(Pmic_CoreHandle_t *pPmicCoreHandle,
                                 Pmic_CommonCtrlStat_t *pCommonCtrlStat) {
  int32_t pmicStatus = PMIC_ST_SUCCESS;

  if ((PMIC_ST_SUCCESS == pmicStatus) &&
      ((bool)true == pmic_validParamCheck(pCommonCtrlStat->validParams,
                                          PMIC_CFG_SAFE_OUT1_PIN_STAT_VALID))) {
    /* Get SAFE_OUT1 Pin Status*/
    pmicStatus = Pmic_getSafeOut1Stat(pPmicCoreHandle, pCommonCtrlStat);
  }

  if ((PMIC_ST_SUCCESS == pmicStatus) &&
      ((bool)true == pmic_validParamCheck(pCommonCtrlStat->validParams,
                                          PMIC_CFG_NRST_PIN_STAT_VALID))) {
    /* Get NRST Pin Status*/
    pmicStatus = Pmic_getNRstPinStat(pPmicCoreHandle, pCommonCtrlStat);
  }
  if ((PMIC_ST_SUCCESS == pmicStatus) &&
      ((bool)true == pmic_validParamCheck(pCommonCtrlStat->validParams,
                                          PMIC_CFG_EN_OUT_PIN_STAT_VALID))) {
    /* Get EN_OUT Pin Status*/
    pmicStatus = Pmic_getEnOutPinStat(pPmicCoreHandle, pCommonCtrlStat);
  }

  return pmicStatus;
}

/**
 * @brief Get the common status.
 * This function retrieves the common control status.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param pCommonCtrlStat Pointer to store the common control status.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
int32_t Pmic_getCommonStat(Pmic_CoreHandle_t *pPmicCoreHandle,
                           Pmic_CommonCtrlStat_t *pCommonCtrlStat) {
  int32_t pmicStatus = PMIC_ST_SUCCESS;

  if (NULL == pPmicCoreHandle) {
    pmicStatus = PMIC_ST_ERR_INV_HANDLE;
  }

  if ((PMIC_ST_SUCCESS == pmicStatus) &&
      ((bool)true == pmic_validParamCheck(pCommonCtrlStat->validParams,
                                          PMIC_CFG_REGISTER_LOCK_STAT_VALID))) {
    /* Get Register Lock Status*/
    pmicStatus = Pmic_getRegLockStat(pPmicCoreHandle, pCommonCtrlStat);
  }

  if (PMIC_ST_SUCCESS == pmicStatus) {
    /* Get EN_Out Pin, NRST Pin, and Safe_Out1 Pin Status */
    pmicStatus =
        Pmic_getEnOutNrstSafeOut1PinStat(pPmicCoreHandle, pCommonCtrlStat);
  }

  return pmicStatus;
}
