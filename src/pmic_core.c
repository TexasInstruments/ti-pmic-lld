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

bool pmic_validParamCheck(uint32_t validParamVal, uint8_t bitPos) {
  bool retVal = (bool)false;

  if (((validParamVal >> bitPos) & 0x01U) != 0U) {
    retVal = (bool)true;
  }

  return retVal;
}

void Pmic_criticalSectionStart(const Pmic_CoreHandle_t *pPmicCoreHandle) {
  if (NULL != pPmicCoreHandle->pFnPmicCritSecStart) {
    pPmicCoreHandle->pFnPmicCritSecStart();
  }
}

void Pmic_criticalSectionStop(const Pmic_CoreHandle_t *pPmicCoreHandle) {
  if (NULL != pPmicCoreHandle->pFnPmicCritSecStop) {
    pPmicCoreHandle->pFnPmicCritSecStop();
  }
}

/**
 * @brief:  Set or clear the register lock/unlock configuration for PMIC.
 *          This function is used to either lock or unlock the configuration
 *          registers of the PMIC. It sends the appropriate unlock data to the
 *          PMIC based on the provided configuration. The function first
 *          checks for valid unlock data and then sends the unlock sequence.
 *          The critical section is used to protect the communication interface
 *          during the operation.
 *
 * @param   pPmicCoreHandle [IN]    PMIC Core Handle.
 * @param   commonCtrlCfg   [IN]    Configuration structure containing register
 * lock/unlock data.
 *
 * @return Status of the API call.
 *         - PMIC_ST_SUCCESS if the operation is successful.
 *         - PMIC_ST_ERR_INV_PARAM if the provided unlock data is invalid.
 *         - Error codes from underlying functions on communication failure.
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

/*!
 * @brief:  Set or clear the counter register lock/unlock configuration for
 * PMIC. This function is used to either lock or unlock the timer and counter
 *          registers of the PMIC. It sends the appropriate unlock data to the
 *          PMIC based on the provided configuration. The function first checks
 *          for valid unlock data and then sends the unlock sequence. The
 * critical section is used to protect the communication interface during the
 * operation.
 *
 * @param   pPmicCoreHandle [IN]    PMIC Core Handle.
 * @param   commonCtrlCfg   [IN]    Configuration structure containing counter
 * register lock/unlock data.
 *
 * @return  Status of the API call.
 *         - PMIC_ST_SUCCESS if the operation is successful.
 *         - PMIC_ST_ERR_INV_PARAM if the provided unlock data is invalid.
 *         - Error codes from underlying functions on communication failure.
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

/*!
 * @brief  API to get Register Lock Status
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

/*!
 * @brief  API to get Timer and Counter Lock Status
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

/*!
 * @brief  API to Read RST-MCU Count Value.
 *         This function reads out the RST-MCU count value.
 *
 * @param   pPmicCoreHandle       [IN]    PMIC Interface Handle.
 * @param   pRecovCntVal          [OUT]   Pointer to store recovery count
 *                                        value
 *
 * @return  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
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

/*!
 * @brief  API to Initialize pPmicCoreHandle for pmicDeviceType, Comm Mode,
 *         Main Slave Address, and NVM Slave Address
 *
 *         This function gets device configuration from pPmicConfigData and
 *         initializes pPmicCoreHandle after validation of given params depends
 *         on validParams bit fields
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

/*!
 * @brief  API to Initialize pPmicCoreHandle for Comm IO RD Fn, Comm IO Wr Fn,
 *         Critical Section Start Fn and Critical Section Stop Fn
 *
 *         This function gets device configuration from pPmicConfigData and
 *         initializes pPmicCoreHandle after validation of given params depends
 *         on validParams bit fields
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

/*!
 * @brief  API to check if the device requested is the one on the bus
 *
 *         Note: In this API, the default PMIC device is assumed as TPS6594x
 *               LEO PMIC. While adding support for New PMIC device, developer
 *               need to update the API functionality for New PMIC device
 *               accordingly.
 *
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

/*!
 * @brief  API to update PMIC subsystem info to PMIC handle and Check the Main
 *         and QA communication interface if PMIC handle is ready for rw
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

/*!
 * @brief  API to Initialize pmic core handle for PMIC LLD.
 *
 * Requirement: REQ_TAG(PDK-5814), REQ_TAG(PDK-5810), REQ_TAG(PDK-5813),
 *              REQ_TAG(PDK-5843), REQ_TAG(PDK-5811), REQ_TAG(PDK-5853),
 *              REQ_TAG(PDK-9129), REQ_TAG(PDK-9329), REQ_TAG(PDK-9159),
 *              REQ_TAG(PDK-5816), REQ_TAG(PDK-5817), REQ_TAG(PDK-5818),
 *              REQ_TAG(PDK-5819), REQ_TAG(PDK-5820), REQ_TAG(PDK-5821),
 *              REQ_TAG(PDK-5822), REQ_TAG(PDK-5823), REQ_TAG(PDK-5824),
 *              REQ_TAG(PDK-5825), REQ_TAG(PDK-5826), REQ_TAG(PDK-5827),
 *              REQ_TAG(PDK-5856), REQ_TAG(PDK-5857), REQ_TAG(PDK-5858),
 *              REQ_TAG(PDK-5859), REQ_TAG(PDK-5860)
 * Design: did_pmic_comm_intf_cfg, did_pmic_comm_single_i2c_cfg,
 *         did_pmic_comm_dual_i2c_cfg, did_pmic_comm_spi_cfg,
 *         did_pmic_tps6594x_j721e_support, did_pmic_lp8764x_j7200_support,
 *         did_pmic_validation_feature_support, did_pmic_performance_support,
 *         did_pmic_generic_feature_support, did_pmic_safety_feature_support,
 *         did_pmic_pre_emption_support, did_pmic_stateless_reentrant_support,
 *         did_pmic_dynamic_alloc_mem_not_supported, did_pmic_build_infra_cfg,
 *         did_pmic_debug_release_profile_support, did_pmic_standalone_support,
 *         did_pmic_multiple_pmic_support, did_pmic_baremetal_support
 * Architecture: aid_pmic_tps6594x_lp8764x_support, aid_pmic_standalone_support,
 *               aid_pmic_multiple_pmic_support, aid_pmic_pre_emption_support,
 *               aid_pmic_stateless_reentrant_support, aid_pmic_generic_support,
 *               aid_pmic_baremetal_support, aid_pmic_comm_intf_i2c_spi_cfg,
 *               aid_pmic_dynamic_alloc_mem_not_supported,
 *               aid_pmic_build_infra_cfg,
 *               aid_pmic_debug_release_profile_support,
 *               aid_pmic_performance_support, aid_pmic_test_support
 *
 *         This function gets device configuration from pPmicConfigData and
 *         initializes device specific information in pPmicCoreHandle after
 *         validation of given params depends on validParams bit fields
 *         and does some basic validation on PMIC interface I2C/SPI,
 *         confirming that PMIC is accessible for PMIC configuration and
 *         monitor features.
 *         Note:  Application has to ensure to avoid access to write protection
 *                registers using PMIC Driver APIs when register lock status is
 *                locked. API returns an erroe when application access to write
 *                protection registers using PMIC Driver APIs when register lock
 *                status is locked
 *
 *  @param   pPmicConfigData [IN]   PMIC Configuration data
 *  @param   pPmicCoreHandle [OUT]  PMIC Interface Handle.
 *
 *  @return  PMIC_ST_SUCCESS in case of success or appropriate error code
 *           For valid values \ref Pmic_ErrorCodes
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

/*!
 * @brief  API to DeInitilizes an existing PMIC Instance.
 *
 * Requirement: REQ_TAG(PDK-5814)
 * Design: did_pmic_comm_intf_cfg
 * Architecture: aid_pmic_tps6594x_lp8764x_support
 *
 *         This function takes an existing Instance pPmicCoreHandle and
 *         closes the LLD being used for this Instance. It should be called
 *         only once per valid pPmicCoreHandle. Should not be called
 *         if Pmic_init() is not called.
 *
 *  @param   pPmicCoreHandle  [IN] PMIC Interface Handle.
 *
 *  @return  PMIC_ST_SUCCESS in case of success or appropriate error code
 *           For valid values \ref Pmic_ErrorCodes
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

/*!
 * @brief   API to get Scratch pad Register Address
 *          Note: In this API, the default scratchPadRegId is assumed as
 *                PMIC_SCRATCH_PAD_REG_1. While adding support for New PMIC
 *                device, developer need to update the API functionality for
 *                New PMIC device accordingly.
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

/*!
 * @brief   API to set/write value in/to scratchpad register.
 *
 * Requirement: REQ_TAG(PDK-5810), REQ_TAG(PDK-5843)
 * Design: did_pmic_comm_single_i2c_cfg, did_pmic_comm_spi_cfg
 * Architecture: aid_pmic_core_misc_cfg
 *
 *          This function is used write data to scratchpad register of PMIC
 *
 * @param   pPmicCoreHandle    [IN]    PMIC Interface Handle.
 * @param   scratchPadRegNum   [IN]    ScratchPad register number
 *                                     \ref Pmic_ScratchPad_Sel
 * @param   data               [IN]    Data/Value to be written to scratchpad.
 *
 * @return  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
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

/*!
 * @brief   API to get/read data from scratchpad register.
 *
 * Requirement: REQ_TAG(PDK-5810), REQ_TAG(PDK-5843)
 * Design: did_pmic_comm_single_i2c_cfg, did_pmic_comm_spi_cfg
 * Architecture: aid_pmic_core_misc_cfg
 *
 *          This function is used read data from scratchpad register of PMIC
 *
 * @param   pPmicCoreHandle    [IN]    PMIC Interface Handle.
 * @param   scratchPadRegNum   [IN]    ScratchPad register number
 *                                     \ref Pmic_ScratchPad_Sel
 * @param   data               [OUT]   Parameter to hold the Data/Value read
 *                                     from scratchpad.
 *
 * @return  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
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

/*!
 * @brief  API to Enable/Disable Spread Spectrum
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

/*!
 * @brief  API to get the status of  Spread Spectrum is Enabled/Disabled
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

/*!
 * @brief  API to Enable SAFE_OUT Pin Configuration
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

/*!
 * @brief  API to get SAFE_OUT Pin Configuration
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


int32_t Pmic_setSafeStateTimeoutCfg(Pmic_CoreHandle_t *pPmicCoreHandle,
                                    Pmic_SafeStateCfg_t safeCfg) {
  int32_t pmicStatus = PMIC_ST_SUCCESS;
  uint8_t regData = 0U;

  if (PMIC_ST_SUCCESS == pmicStatus) {
    Pmic_criticalSectionStart(pPmicCoreHandle);
    pmicStatus = Pmic_commIntf_recvByte(
        pPmicCoreHandle, PMIC_SAFE_TMO_CFG_REGADDR, &regData);
    if (PMIC_ST_SUCCESS == pmicStatus) {
        Pmic_getBitField(regData, PMIC_SAFE_TMO_SHIFT, PMIC_SAFE_TMO_MASK);

        Pmic_setBitField(&regData, PMIC_SAFE_TMO_SHIFT,
                         PMIC_SAFE_TMO_MASK, safeCfg.safeStateTMO);

      pmicStatus = Pmic_commIntf_sendByte(
          pPmicCoreHandle, PMIC_SAFE_TMO_CFG_REGADDR, regData);
    }
    Pmic_criticalSectionStop(pPmicCoreHandle);
  }

  return pmicStatus;
}


int32_t Pmic_getSafeStateTimeoutCfg(Pmic_CoreHandle_t *pPmicCoreHandle,
                                    Pmic_SafeStateCfg_t safeCfg) {
  int32_t pmicStatus = PMIC_ST_SUCCESS;
  uint8_t regData = 0U;

  if (PMIC_ST_SUCCESS == pmicStatus) {
    Pmic_criticalSectionStart(pPmicCoreHandle);
    pmicStatus = Pmic_commIntf_recvByte(
        pPmicCoreHandle, PMIC_SAFE_TMO_CFG_REGADDR, &regData);
    if (PMIC_ST_SUCCESS == pmicStatus) {
        safeCfg.safeStateTMO = Pmic_getBitField(regData,
                                PMIC_SAFE_TMO_SHIFT,
                                PMIC_SAFE_TMO_MASK);
    }
    Pmic_criticalSectionStop(pPmicCoreHandle);
  }

  return pmicStatus;
}


int32_t Pmic_setSafeStateThresholdCfg(Pmic_CoreHandle_t *pPmicCoreHandle,
                                      Pmic_SafeStateCfg_t safeCfg) {
  int32_t pmicStatus = PMIC_ST_SUCCESS;
  uint8_t regData = 0U;

  if (PMIC_ST_SUCCESS == pmicStatus) {
    Pmic_criticalSectionStart(pPmicCoreHandle);
    pmicStatus = Pmic_commIntf_recvByte(
        pPmicCoreHandle, PMIC_SAFE_TMO_CFG_REGADDR, &regData);
    if (PMIC_ST_SUCCESS == pmicStatus) {
      Pmic_getBitField(regData,
                       PMIC_SAFE_LOCK_TH_SHIFT,
                       PMIC_SAFE_LOCK_TH_MASK);

      Pmic_setBitField(&regData,
                       PMIC_SAFE_LOCK_TH_SHIFT,
                       PMIC_SAFE_LOCK_TH_MASK,
                       safeCfg.safeLockThreshold);

      pmicStatus = Pmic_commIntf_sendByte(
          pPmicCoreHandle, PMIC_SAFE_TMO_CFG_REGADDR, regData);
    }
    Pmic_criticalSectionStop(pPmicCoreHandle);
  }

  return pmicStatus;
}


int32_t Pmic_getSafeStateThresholdCfg(Pmic_CoreHandle_t *pPmicCoreHandle,
                                      Pmic_SafeStateCfg_t safeCfg) {
  int32_t pmicStatus = PMIC_ST_SUCCESS;
  uint8_t regData = 0U;

  if (PMIC_ST_SUCCESS == pmicStatus) {
    Pmic_criticalSectionStart(pPmicCoreHandle);
    pmicStatus = Pmic_commIntf_recvByte(
        pPmicCoreHandle, PMIC_SAFE_TMO_CFG_REGADDR, &regData);
    if (PMIC_ST_SUCCESS == pmicStatus) {
        safeCfg.safeLockThreshold = Pmic_getBitField(regData,
                                     PMIC_SAFE_LOCK_TH_SHIFT,
                                     PMIC_SAFE_LOCK_TH_MASK);
    }
    Pmic_criticalSectionStop(pPmicCoreHandle);
  }

  return pmicStatus;
}

/*!
 * @brief   API to set PMIC common control parameter configuration.
 *
 * Requirement: REQ_TAG(PDK-9112), REQ_TAG(PDK-9114), REQ_TAG(PDK-9131),
 *              REQ_TAG(PDK-9143)
 * Design: did_pmic_common_ctrl_cfg_readback
 * Architecture: aid_pmic_core_misc_cfg
 *
 *          This function is used to set the required common control parameter
 *          configuration when corresponding validParam bit field is set in
 *          the Pmic_CommonCtrlCfg_t
 *          For more information \ref Pmic_CommonCtrlCfg_t
 *
 * @param   pPmicCoreHandle [IN]    PMIC Interface Handle.
 * @param   commonCtrlCfg   [IN]    Set PMIC required common control parameter
 *                                  configuration.
 *
 * @return  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
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

/*!
 * @brief   API to get PMIC common control parameter configuration.
 *
 * Requirement: REQ_TAG(PDK-9112), REQ_TAG(PDK-9114), REQ_TAG(PDK-9131),
 *              REQ_TAG(PDK-9143)
 * Design: did_pmic_common_ctrl_cfg_readback
 * Architecture: aid_pmic_core_misc_cfg
 *
 *          This function is used to get the required common control parameter
 *          configuration when corresponding validParam bit field is set in
 *          the Pmic_CommonCtrlCfg_t
 *          For more information \ref Pmic_CommonCtrlCfg_t
 *
 * @param   pPmicCoreHandle [IN]        PMIC Interface Handle.
 * @param   pCommonCtrlCfg  [IN/OUT]    Pointer to store PMIC required common
 *                                      control parameter configuration.
 *
 * @return  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
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

/*!
 * @brief  API to set AMUX/DMUX Pin Control Configuration
 */
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

/*!
 * @brief  API to get AMUX/DMUX Pin Control Configuration
 */
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

/*!
 * @brief   API to set PMIC Miscellaneous control parameter configuration.
 *
 * Requirement: REQ_TAG(PDK-9132), REQ_TAG(PDK-9127), REQ_TAG(PDK-9111)
 * Design: did_pmic_misc_ctrl_cfg_readback
 * Architecture: aid_pmic_core_misc_cfg
 *
 *          This function is used to set the required miscellaneous control
 *          parameter configuration when corresponding validParam bit field is
 *          set in the Pmic_MiscCtrlCfg_t
 *          For more information \ref Pmic_MiscCtrlCfg_t
 *
 * @param   pPmicCoreHandle [IN]    PMIC Interface Handle.
 * @param   miscCtrlCfg     [IN]    Set PMIC required miscellaneous control
 *                                  parameter configuration.
 *
 * @return  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
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

/*!
 * @brief   API to get PMIC Miscellaneous control parameter configuration
 *
 * Requirement: REQ_TAG(PDK-9132), REQ_TAG(PDK-9127)
 * Design: did_pmic_misc_ctrl_cfg_readback
 * Architecture: aid_pmic_core_misc_cfg
 *
 *          This function is used to get the required miscellaneous control
 *          parameter configuration when corresponding validParam bit field is
 *          set in the Pmic_MiscCtrlCfg_t
 *          For more information \ref Pmic_MiscCtrlCfg_t
 *
 * @param   pPmicCoreHandle [IN]       PMIC Interface Handle.
 * @param   pMiscCtrlCfg    [IN/OUT]   Pointer to store PMIC required
 *                                     miscellaneous control parameter
 *                                     configuration.
 *
 * @return  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
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

/*!
 * @brief   API to get NRST/SAFE_OUT1/EN_OUT Register Bit fields
 *          Note: In this API, the default pinType is assumed as EN_OUT. While
 *          adding support for New PMIC device, developer need to update the API
 *          functionality for New PMIC device accordingly.
 */
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

/*!
 * @brief   API to get PMIC GPIO NRST/SAFE_OUT1/EN_OUT Pin
 *
 * Requirement: REQ_TAG(PDK-9137), REQ_TAG(PDK-9131)
 * Design: did_pmic_pin_readback
 * Architecture: aid_pmic_core_misc_cfg
 *
 *          This function is used to read the signal level of the NRSTOUT_SOC/
 *          NRSTOUT/ EN_DRV Pin
 *
 * @param   pPmicCoreHandle [IN]    PMIC Interface Handle
 * @param   pinType         [IN]    PMIC pin type.
 *                                   Valid values of pin type
 *                                   \ref Pmic_PinType_Sel
 * @param   pPinValue       [OUT]   Pointer to store the status of pin type
 *                                    Valid values \ref Pmic_SignalLvl
 *
 * @return  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
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

/*!
 * @brief  API to get SAFE_OUT1 Pin status
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

/*!
 * @brief  API to get NRST Pin Status
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

/*!
 * @brief  API to get EN_OUT Pin Status
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

/*!
 * @brief   API to get EN_Out Pin, NRST Pin, and Safe_Out1 Pin Status
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

/*!
 * @brief   API to get PMIC common control parameter status.
 *          This function is used to get the required common control parameter
 *          status when corresponding validParam bit field is set in
 *          the Pmic_CommonCtrlStat_t
 *          For more information \ref Pmic_CommonCtrlStat_t
 *
 * @param   pPmicCoreHandle  [IN]       PMIC Interface Handle.
 * @param   pCommonCtrlStat  [IN/OUT]   Pointer to store PMIC required common
 *                                      control parameter status.
 *
 * @return  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
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
