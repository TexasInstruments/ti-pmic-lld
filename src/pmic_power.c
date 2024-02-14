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
 *   \file    pmic_power_tps65386x.c
 *
 *   \brief   This file contains the TPS65386x BB PMIC power Specific
 *            configuration API's and structures
 */

#include <pmic_power.h>

/* ========================================================================= */
/*                           RegAddr for BlackBird                           */
/* ========================================================================= */


void pmic_get_tps65386x_pwrRsrceRegCfg(
    Pmic_powerRsrcRegCfg_t **pPwrRsrcRegCfg) {
  static Pmic_powerRsrcRegCfg_t tps65386x_pwrRsrcRegCfg = {
      .buckConfigRegAddr = PMIC_BUCK_BST_CFG_REGADDR,
      .ldo1ConfigRegAddr = PMIC_LDO1_CFG_REGADDR,
      .ldo2ConfigRegAddr = PMIC_LDO2_CFG_REGADDR,
      .ldo3ConfigRegAddr = PMIC_LDO3_CFG_REGADDR,
      .ldo4ConfigRegAddr = PMIC_LDO4_CFG_REGADDR,
      .pldo1ConfigRegAddr = PMIC_PLDO1_CFG_REGADDR,
      .pldo2ConfigRegAddr = PMIC_PLDO2_CFG_REGADDR,
      .pldoConfigRegAddr = PMIC_PLDO_CFG_REGADDR,
      .dscgConfigRegAddr = PMIC_LDO_DSCG_CFG_REGADDR,
      .pgoodConfigRegAddr = PMIC_LDO_PGOOD_CFG_REGADDR,
      .ldoCtrlRegAddr = PMIC_LDO_CTRL_REGADDR,
      .enoutCtrlRegAddr = PMIC_PLDO_EN_OUT_CTRL_REGADDR};

  *pPwrRsrcRegCfg = &tps65386x_pwrRsrcRegCfg;
}


int32_t Pmic_powerSetBuckBstPgoodCfg(Pmic_CoreHandle_t *pPmicCoreHandle,
                                     Pmic_powerBuckBoostCfgReg_t *buckBstCfg) {
  uint8_t pmicStatus = PMIC_ST_SUCCESS;
  uint8_t regAddr = buckBstCfg->bbCfgRegAddr;
  uint8_t regData = 0U;

  Pmic_criticalSectionStart(pPmicCoreHandle);
  pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, regAddr, &regData);

  if (PMIC_ST_SUCCESS == pmicStatus) {
    Pmic_setBitField(&regData, buckBstCfg->bbCfgRegShift,
                     buckBstCfg->bbCfgRegMask, buckBstCfg->bbPgoodCfg);

    pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle, regAddr, regData);
  }

  Pmic_criticalSectionStop(pPmicCoreHandle);

  return pmicStatus;
}


int32_t Pmic_powerGetBuckBstPgoodCfg(Pmic_CoreHandle_t *pPmicCoreHandle,
                                     Pmic_powerBuckBoostCfgReg_t *buckBstCfg) {
  uint8_t pmicStatus = PMIC_ST_SUCCESS;
  uint8_t regAddr = buckBstCfg->bbCfgRegAddr;
  uint8_t regData = 0U;

  Pmic_criticalSectionStart(pPmicCoreHandle);
  pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, regAddr, &regData);

  if (PMIC_ST_SUCCESS == pmicStatus) {
      buckBstCfg->bbPgoodCfg = Pmic_getBitField(regData,
                                                buckBstCfg->bbCfgRegShift,
                                                buckBstCfg->bbCfgRegMask);
  }

  Pmic_criticalSectionStop(pPmicCoreHandle);

  return pmicStatus;
}


int32_t Pmic_powerSetBuckBstSsEn(Pmic_CoreHandle_t *pPmicCoreHandle,
                                 Pmic_powerBuckBoostCfgReg_t *buckBstCfg) {
  uint8_t pmicStatus = PMIC_ST_SUCCESS;
  uint8_t regAddr = buckBstCfg->bbCfgRegAddr;
  uint8_t regData = 0U;

  Pmic_criticalSectionStart(pPmicCoreHandle);
  pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, regAddr, &regData);

  if (PMIC_ST_SUCCESS == pmicStatus) {
    Pmic_setBitField(&regData, buckBstCfg->bbCfgRegShift,
                     buckBstCfg->bbCfgRegMask, buckBstCfg->bbSsEn);

    pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle, regAddr, regData);
  }

  Pmic_criticalSectionStop(pPmicCoreHandle);

  return pmicStatus;
}


int32_t Pmic_powerGetBuckBstSsEn(Pmic_CoreHandle_t *pPmicCoreHandle,
                                 Pmic_powerBuckBoostCfgReg_t *buckBstCfg) {
  uint8_t pmicStatus = PMIC_ST_SUCCESS;
  uint8_t regAddr = buckBstCfg->bbCfgRegAddr;
  uint8_t regData = 0U;

  Pmic_criticalSectionStart(pPmicCoreHandle);
  pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, regAddr, &regData);

  if (PMIC_ST_SUCCESS == pmicStatus) {
      buckBstCfg->bbSsEn = Pmic_getBitField(regData,
                            buckBstCfg->bbCfgRegShift,
                            buckBstCfg->bbCfgRegMask);
  }

  Pmic_criticalSectionStop(pPmicCoreHandle);

  return pmicStatus;
}


int32_t Pmic_powerSetBuckBstStbyLvlCfg(Pmic_CoreHandle_t *pPmicCoreHandle,
                                       Pmic_powerBuckBoostCfgReg_t *buckBstCfg) {
  uint8_t pmicStatus = PMIC_ST_SUCCESS;
  uint8_t regAddr = buckBstCfg->bbCfgRegAddr;
  uint8_t regData = 0U;

  Pmic_criticalSectionStart(pPmicCoreHandle);
  pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, regAddr, &regData);

  if (PMIC_ST_SUCCESS == pmicStatus) {
    Pmic_setBitField(&regData, buckBstCfg->bbCfgRegShift,
                     buckBstCfg->bbCfgRegMask, buckBstCfg->bbStbyLvlCfg);

    pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle, regAddr, regData);
  }

  Pmic_criticalSectionStop(pPmicCoreHandle);

  return pmicStatus;
}


int32_t Pmic_powerGetBuckBstStbyLvlCfg(Pmic_CoreHandle_t *pPmicCoreHandle,
                                       Pmic_powerBuckBoostCfgReg_t *buckBstCfg) {
  uint8_t pmicStatus = PMIC_ST_SUCCESS;
  uint8_t regAddr = buckBstCfg->bbCfgRegAddr;
  uint8_t regData = 0U;

  Pmic_criticalSectionStart(pPmicCoreHandle);
  pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, regAddr, &regData);

  if (PMIC_ST_SUCCESS == pmicStatus) {
      buckBstCfg->bbStbyLvlCfg = Pmic_getBitField(regData,
                                  buckBstCfg->bbCfgRegShift,
                                  buckBstCfg->bbCfgRegMask);
  }

  Pmic_criticalSectionStop(pPmicCoreHandle);

  return pmicStatus;
}


int32_t Pmic_powerSetBuckBstLvlCfg(Pmic_CoreHandle_t *pPmicCoreHandle,
                                   Pmic_powerBuckBoostCfgReg_t *buckBstCfg) {
  uint8_t pmicStatus = PMIC_ST_SUCCESS;
  uint8_t regAddr = buckBstCfg->bbCfgRegAddr;
  uint8_t regData = 0U;

  Pmic_criticalSectionStart(pPmicCoreHandle);
  pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, regAddr, &regData);

  if (PMIC_ST_SUCCESS == pmicStatus) {
    Pmic_setBitField(&regData, buckBstCfg->bbCfgRegShift,
                     buckBstCfg->bbCfgRegMask, buckBstCfg->bbLvlCfg);

    pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle, regAddr, regData);
  }

  Pmic_criticalSectionStop(pPmicCoreHandle);

  return pmicStatus;
}


int32_t Pmic_powerGetBuckBstLvlCfg(Pmic_CoreHandle_t *pPmicCoreHandle,
                                   Pmic_powerBuckBoostCfgReg_t *buckBstCfg) {
  uint8_t pmicStatus = PMIC_ST_SUCCESS;
  uint8_t regAddr = buckBstCfg->bbCfgRegAddr;
  uint8_t regData = 0U;

  Pmic_criticalSectionStart(pPmicCoreHandle);
  pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, regAddr, &regData);

  if (PMIC_ST_SUCCESS == pmicStatus) {
      buckBstCfg->bbLvlCfg = Pmic_getBitField(regData,
                              buckBstCfg->bbCfgRegShift,
                              buckBstCfg->bbCfgRegMask);
  }

  Pmic_criticalSectionStop(pPmicCoreHandle);

  return pmicStatus;
}


int32_t Pmic_powerSetBuckBstCfg(Pmic_CoreHandle_t *pPmicCoreHandle,
                                Pmic_powerBuckBoostCfgReg_t *buckBstCfg,
                                Pmic_powerRsrcRegCfg_t *pwrRsrcRegCfg,
                                Pmic_powerRsrcCfg_t *pwrRsrcCfg) {

  uint8_t pmicStatus = PMIC_ST_SUCCESS;

  if (pPmicCoreHandle == NULL) {
    pmicStatus = PMIC_ST_ERR_INV_HANDLE;
  } else {
    buckBstCfg->bbCfgRegAddr = pwrRsrcRegCfg->buckConfigRegAddr;
    buckBstCfg->bbCfgRegShift = pwrRsrcCfg->pmicConfigShiftVal;
    buckBstCfg->bbCfgRegMask = pwrRsrcCfg->pmicConfigMaskVal;

    if (buckBstCfg->bbPgoodCfg != PMIC_ST_DEFAULT_DATA) {
      pmicStatus = Pmic_powerSetBuckBstPgoodCfg(pPmicCoreHandle, buckBstCfg);
    }

    if (buckBstCfg->bbSsEn != PMIC_ST_DEFAULT_DATA) {
      pmicStatus = Pmic_powerSetBuckBstSsEn(pPmicCoreHandle, buckBstCfg);
    }

    if (buckBstCfg->bbStbyLvlCfg != PMIC_ST_DEFAULT_DATA) {
      pmicStatus = Pmic_powerSetBuckBstStbyLvlCfg(pPmicCoreHandle, buckBstCfg);
    }

    if (buckBstCfg->bbLvlCfg != PMIC_ST_DEFAULT_DATA) {
      pmicStatus = Pmic_powerSetBuckBstLvlCfg(pPmicCoreHandle, buckBstCfg);
    }
  }

  initializeBuckBoostCfgReg(buckBstCfg);

  return pmicStatus;
}


int32_t Pmic_powerGetBuckBstCfg(Pmic_CoreHandle_t *pPmicCoreHandle,
                                Pmic_powerBuckBoostCfgReg_t *buckBstCfg,
                                Pmic_powerRsrcRegCfg_t *pwrRsrcRegCfg,
                                Pmic_powerRsrcCfg_t *pwrRsrcCfg) {

  uint8_t pmicStatus = PMIC_ST_SUCCESS;

  if (pPmicCoreHandle == NULL) {
    pmicStatus = PMIC_ST_ERR_INV_HANDLE;
  } else {
    buckBstCfg->bbCfgRegAddr = pwrRsrcRegCfg->buckConfigRegAddr;
    buckBstCfg->bbCfgRegShift = pwrRsrcCfg->pmicConfigShiftVal;
    buckBstCfg->bbCfgRegMask = pwrRsrcCfg->pmicConfigMaskVal;

    if (buckBstCfg->bbPgoodCfg != PMIC_ST_DEFAULT_DATA) {
      pmicStatus = Pmic_powerGetBuckBstPgoodCfg(pPmicCoreHandle, buckBstCfg);
    }

    if (buckBstCfg->bbSsEn != PMIC_ST_DEFAULT_DATA) {
      pmicStatus = Pmic_powerGetBuckBstSsEn(pPmicCoreHandle, buckBstCfg);
    }

    if (buckBstCfg->bbStbyLvlCfg != PMIC_ST_DEFAULT_DATA) {
      pmicStatus = Pmic_powerGetBuckBstStbyLvlCfg(pPmicCoreHandle, buckBstCfg);
    }

    if (buckBstCfg->bbLvlCfg != PMIC_ST_DEFAULT_DATA) {
      pmicStatus = Pmic_powerGetBuckBstLvlCfg(pPmicCoreHandle, buckBstCfg);
    }
  }

  initializeBuckBoostCfgReg(buckBstCfg);

  return pmicStatus;
}


void Pmic_getLDOCfgFields(uint8_t ldoNumber, Pmic_ldoCfgReg_t *ldoCfg,
                          Pmic_powerRsrcCfg_t *pwrRsrcCfg,
                          Pmic_powerRsrcRegCfg_t *pwrRsrcRegCfg) {
  ldoCfg->ldoRegShift = pwrRsrcCfg->pmicConfigShiftVal;
  ldoCfg->ldoRegMask = pwrRsrcCfg->pmicConfigMaskVal;

  switch (ldoNumber) {
  case PMIC_LDO1:
    ldoCfg->ldoRegAddr = pwrRsrcRegCfg->ldo1ConfigRegAddr;
    break;
  case PMIC_LDO2:
    ldoCfg->ldoRegAddr = pwrRsrcRegCfg->ldo2ConfigRegAddr;
    break;
  case PMIC_LDO3:
    ldoCfg->ldoRegAddr = pwrRsrcRegCfg->ldo3ConfigRegAddr;
    break;
  case PMIC_LDO4:
    ldoCfg->ldoRegAddr = pwrRsrcRegCfg->ldo4ConfigRegAddr;
    break;
  default:
    ldoCfg->ldoRegAddr = PMIC_ST_DEFAULT_DATA;
    break;
  }
}


int32_t Pmic_powerSetLDORtCfg(Pmic_CoreHandle_t *pPmicCoreHandle,
                              Pmic_ldoCfgReg_t *ldoCfg) {
  uint8_t pmicStatus = PMIC_ST_SUCCESS;
  uint8_t regAddr = ldoCfg->ldoRegAddr;
  uint8_t regData = 0U;

  Pmic_criticalSectionStart(pPmicCoreHandle);
  pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, regAddr, &regData);

  if (PMIC_ST_SUCCESS == pmicStatus) {
    Pmic_setBitField(&regData, ldoCfg->ldoRegShift, ldoCfg->ldoRegMask,
                     ldoCfg->ldoRtCfg);

    pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle, regAddr, regData);
  }

  Pmic_criticalSectionStop(pPmicCoreHandle);

  return pmicStatus;
}


int32_t Pmic_powerGetLDORtCfg(Pmic_CoreHandle_t *pPmicCoreHandle,
                              Pmic_ldoCfgReg_t *ldoCfg) {
  uint8_t pmicStatus = PMIC_ST_SUCCESS;
  uint8_t regAddr = ldoCfg->ldoRegAddr;
  uint8_t regData = 0U;

  Pmic_criticalSectionStart(pPmicCoreHandle);
  pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, regAddr, &regData);

  if (PMIC_ST_SUCCESS == pmicStatus) {
      ldoCfg->ldoRtCfg = Pmic_getBitField(regData,
                          ldoCfg->ldoRegShift,
                          ldoCfg->ldoRegMask);
  }
  Pmic_criticalSectionStop(pPmicCoreHandle);

  return pmicStatus;
}


int32_t Pmic_powerSetLDOIlimLvlCfg(Pmic_CoreHandle_t *pPmicCoreHandle,
                                   Pmic_ldoCfgReg_t *ldoCfg) {
  uint8_t pmicStatus = PMIC_ST_SUCCESS;
  uint8_t regAddr = ldoCfg->ldoRegAddr;
  uint8_t regData = 0U;

  Pmic_criticalSectionStart(pPmicCoreHandle);
  pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, regAddr, &regData);

  if (PMIC_ST_SUCCESS == pmicStatus) {
    Pmic_setBitField(&regData, ldoCfg->ldoRegShift, ldoCfg->ldoRegMask,
                     ldoCfg->ldoIlimLvlCfg);

    pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle, regAddr, regData);
  }

  Pmic_criticalSectionStop(pPmicCoreHandle);

  return pmicStatus;
}


int32_t Pmic_powerGetLDOIlimLvlCfg(Pmic_CoreHandle_t *pPmicCoreHandle,
                                   Pmic_ldoCfgReg_t *ldoCfg) {
  uint8_t pmicStatus = PMIC_ST_SUCCESS;
  uint8_t regAddr = ldoCfg->ldoRegAddr;
  uint8_t regData = 0U;

  Pmic_criticalSectionStart(pPmicCoreHandle);
  pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, regAddr, &regData);

  if (PMIC_ST_SUCCESS == pmicStatus) {
      ldoCfg->ldoIlimLvlCfg = Pmic_getBitField(regData,
                               ldoCfg->ldoRegShift,
                               ldoCfg->ldoRegMask);
  }
  Pmic_criticalSectionStop(pPmicCoreHandle);

  return pmicStatus;
}


int32_t Pmic_powerSetLDOLvlCfg(Pmic_CoreHandle_t *pPmicCoreHandle,
                               Pmic_ldoCfgReg_t *ldoCfg) {
  uint8_t pmicStatus = PMIC_ST_SUCCESS;
  uint8_t regAddr = ldoCfg->ldoRegAddr;
  uint8_t regData = 0U;

  Pmic_criticalSectionStart(pPmicCoreHandle);
  pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, regAddr, &regData);

  if (PMIC_ST_SUCCESS == pmicStatus) {
    Pmic_setBitField(&regData, ldoCfg->ldoRegShift, ldoCfg->ldoRegMask,
                     ldoCfg->ldoLvlCfg);

    pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle, regAddr, regData);
  }

  Pmic_criticalSectionStop(pPmicCoreHandle);

  return pmicStatus;
}


int32_t Pmic_powerGetLDOLvlCfg(Pmic_CoreHandle_t *pPmicCoreHandle,
                               Pmic_ldoCfgReg_t *ldoCfg) {
  uint8_t pmicStatus = PMIC_ST_SUCCESS;
  uint8_t regAddr = ldoCfg->ldoRegAddr;
  uint8_t regData = 0U;

  Pmic_criticalSectionStart(pPmicCoreHandle);
  pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, regAddr, &regData);

  if (PMIC_ST_SUCCESS == pmicStatus) {
      ldoCfg->ldoLvlCfg = Pmic_getBitField(regData,
                           ldoCfg->ldoRegShift,
                           ldoCfg->ldoRegMask);
  }
  Pmic_criticalSectionStop(pPmicCoreHandle);

  return pmicStatus;
}


int32_t Pmic_powerSetLdoConfigRegister(void *pPmicCoreHandle, uint8_t ldoNumber,
                                       Pmic_ldoCfgReg_t *ldoConfig,
                                       Pmic_powerRsrcCfg_t *pwrRsrcCfg,
                                       Pmic_powerRsrcRegCfg_t *pwrRsrcRegCfg) {
  uint8_t pmicStatus = PMIC_ST_SUCCESS;

  if (pPmicCoreHandle == NULL) {
    pmicStatus = PMIC_ST_ERR_INV_HANDLE;
  } else {
    if (ldoNumber < PMIC_POWER_LDO_MIN || ldoNumber > PMIC_POWER_LDO_MAX) {
      pmicStatus = PMIC_ST_ERR_INV_PARAM;
    } else {
      Pmic_getLDOCfgFields(ldoNumber, ldoConfig, pwrRsrcCfg, pwrRsrcRegCfg);

      if (ldoConfig->ldoRtCfg != PMIC_ST_DEFAULT_DATA) {
        pmicStatus = Pmic_powerSetLDORtCfg(pPmicCoreHandle, ldoConfig);
      }

      if (ldoConfig->ldoIlimLvlCfg != PMIC_ST_DEFAULT_DATA) {
        pmicStatus = Pmic_powerSetLDOIlimLvlCfg(pPmicCoreHandle, ldoConfig);
      }

      if (ldoConfig->ldoLvlCfg != PMIC_ST_DEFAULT_DATA) {
        pmicStatus = Pmic_powerSetLDOLvlCfg(pPmicCoreHandle, ldoConfig);
      }

      initializeLDOCfgReg(ldoConfig);
    }
  }

  return pmicStatus;
}


int32_t Pmic_powerGetLdoConfigRegister(void *pPmicCoreHandle, uint8_t ldoNumber,
                                       Pmic_ldoCfgReg_t *ldoConfig,
                                       Pmic_powerRsrcCfg_t *pwrRsrcCfg,
                                       Pmic_powerRsrcRegCfg_t *pwrRsrcRegCfg) {
  uint8_t pmicStatus = PMIC_ST_SUCCESS;

  if (pPmicCoreHandle == NULL) {
    pmicStatus = PMIC_ST_ERR_INV_HANDLE;
  } else {
    if (ldoNumber < PMIC_POWER_LDO_MIN || ldoNumber > PMIC_POWER_LDO_MAX) {
      pmicStatus = PMIC_ST_ERR_INV_PARAM;
    } else {
      Pmic_getLDOCfgFields(ldoNumber, ldoConfig, pwrRsrcCfg, pwrRsrcRegCfg);

      if (ldoConfig->ldoRtCfg != PMIC_ST_DEFAULT_DATA) {
        pmicStatus = Pmic_powerGetLDORtCfg(pPmicCoreHandle, ldoConfig);
      }

      if (ldoConfig->ldoIlimLvlCfg != PMIC_ST_DEFAULT_DATA) {
        pmicStatus = Pmic_powerGetLDOIlimLvlCfg(pPmicCoreHandle, ldoConfig);
      }

      if (ldoConfig->ldoLvlCfg != PMIC_ST_DEFAULT_DATA) {
        pmicStatus = Pmic_powerGetLDOLvlCfg(pPmicCoreHandle, ldoConfig);
      }

      initializeLDOCfgReg(ldoConfig);
    }
  }

  return pmicStatus;
}


void Pmic_getPLDOCfgFields(uint8_t pldoNumber, Pmic_pldoCfgReg_t *pldoCfg,
                           Pmic_powerRsrcCfg_t *pwrRsrcCfg,
                           Pmic_powerRsrcRegCfg_t *pwrRsrcRegCfg) {
  pldoCfg->pldoRegShift = pwrRsrcCfg->pmicConfigShiftVal;
  pldoCfg->pldoRegMask = pwrRsrcCfg->pmicConfigMaskVal;

  switch (pldoNumber) {
  case PMIC_PLDO1:
    pldoCfg->pldoRegAddr = pwrRsrcRegCfg->pldo1ConfigRegAddr;
    break;
  case PMIC_PLDO2:
    pldoCfg->pldoRegAddr = pwrRsrcRegCfg->pldo2ConfigRegAddr;
    break;
  default:
    pldoCfg->pldoRegAddr = PMIC_ST_DEFAULT_DATA;
    break;
  }
}


int32_t Pmic_powerSetPLDOModeSel(Pmic_CoreHandle_t *pPmicCoreHandle,
                                 Pmic_pldoCfgReg_t *pldoCfg) {
  uint8_t pmicStatus = PMIC_ST_SUCCESS;
  uint8_t regAddr = pldoCfg->pldoRegAddr;
  uint8_t regData = 0U;

  Pmic_criticalSectionStart(pPmicCoreHandle);
  pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, regAddr, &regData);

  if (PMIC_ST_SUCCESS == pmicStatus) {
    Pmic_setBitField(&regData, pldoCfg->pldoRegShift, pldoCfg->pldoRegMask,
                     pldoCfg->pldoModeSel);

    pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle, regAddr, regData);
  }

  Pmic_criticalSectionStop(pPmicCoreHandle);

  return pmicStatus;
}


int32_t Pmic_powerGetPLDOModeSel(Pmic_CoreHandle_t *pPmicCoreHandle,
                                 Pmic_pldoCfgReg_t *pldoCfg) {
  uint8_t pmicStatus = PMIC_ST_SUCCESS;
  uint8_t regAddr = pldoCfg->pldoRegAddr;
  uint8_t regData = 0U;

  Pmic_criticalSectionStart(pPmicCoreHandle);
  pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, regAddr, &regData);

  if (PMIC_ST_SUCCESS == pmicStatus) {
      pldoCfg->pldoModeSel = Pmic_getBitField(regData,
                              pldoCfg->pldoRegShift,
                              pldoCfg->pldoRegMask);
  }
  Pmic_criticalSectionStop(pPmicCoreHandle);

  return pmicStatus;
}


int32_t Pmic_powerSetPLDOIlimLvlCfg(Pmic_CoreHandle_t *pPmicCoreHandle,
                                    Pmic_pldoCfgReg_t *pldoCfg) {
  uint8_t pmicStatus = PMIC_ST_SUCCESS;
  uint8_t regAddr = pldoCfg->pldoRegAddr;
  uint8_t regData = 0U;

  Pmic_criticalSectionStart(pPmicCoreHandle);
  pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, regAddr, &regData);

  if (PMIC_ST_SUCCESS == pmicStatus) {
    Pmic_setBitField(&regData, pldoCfg->pldoRegShift, pldoCfg->pldoRegMask,
                     pldoCfg->pldoIlimLvlCfg);

    pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle, regAddr, regData);
  }

  Pmic_criticalSectionStop(pPmicCoreHandle);

  return pmicStatus;
}


int32_t Pmic_powerGetPLDOIlimLvlCfg(Pmic_CoreHandle_t *pPmicCoreHandle,
                                    Pmic_pldoCfgReg_t *pldoCfg) {
  uint8_t pmicStatus = PMIC_ST_SUCCESS;
  uint8_t regAddr = pldoCfg->pldoRegAddr;
  uint8_t regData = 0U;

  Pmic_criticalSectionStart(pPmicCoreHandle);
  pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, regAddr, &regData);

  if (PMIC_ST_SUCCESS == pmicStatus) {
      pldoCfg->pldoIlimLvlCfg = Pmic_getBitField(regData,
                                 pldoCfg->pldoRegShift,
                                 pldoCfg->pldoRegMask);
  }
  Pmic_criticalSectionStop(pPmicCoreHandle);

  return pmicStatus;
}


int32_t Pmic_powerSetPLDOLvlCfg(Pmic_CoreHandle_t *pPmicCoreHandle,
                                Pmic_pldoCfgReg_t *pldoCfg) {
  uint8_t pmicStatus = PMIC_ST_SUCCESS;
  uint8_t regAddr = pldoCfg->pldoRegAddr;
  uint8_t regData = 0U;

  Pmic_criticalSectionStart(pPmicCoreHandle);
  pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, regAddr, &regData);

  if (PMIC_ST_SUCCESS == pmicStatus) {
    Pmic_setBitField(&regData, pldoCfg->pldoRegShift, pldoCfg->pldoRegMask,
                     pldoCfg->pldoLvlCfg);

    pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle, regAddr, regData);
  }

  Pmic_criticalSectionStop(pPmicCoreHandle);

  return pmicStatus;
}


int32_t Pmic_powerGetPLDOLvlCfg(Pmic_CoreHandle_t *pPmicCoreHandle,
                                Pmic_pldoCfgReg_t *pldoCfg) {
  uint8_t pmicStatus = PMIC_ST_SUCCESS;
  uint8_t regAddr = pldoCfg->pldoRegAddr;
  uint8_t regData = 0U;

  Pmic_criticalSectionStart(pPmicCoreHandle);
  pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, regAddr, &regData);

  if (PMIC_ST_SUCCESS == pmicStatus) {
      pldoCfg->pldoLvlCfg = Pmic_getBitField(regData,
                             pldoCfg->pldoRegShift,
                             pldoCfg->pldoRegMask);
  }
  Pmic_criticalSectionStop(pPmicCoreHandle);

  return pmicStatus;
}


int32_t Pmic_powerSetPLDOConfigRegister(void *pPmicCoreHandle,
                                        uint8_t pldoNumber,
                                        Pmic_pldoCfgReg_t *pldoConfig,
                                        Pmic_powerRsrcCfg_t *pwrRsrcCfg,
                                        Pmic_powerRsrcRegCfg_t *pwrRsrcRegCfg) {
  uint8_t pmicStatus = PMIC_ST_SUCCESS;

  if (pPmicCoreHandle == NULL) {
    pmicStatus = PMIC_ST_ERR_INV_HANDLE;
  } else {
    if (pldoNumber < PMIC_POWER_PLDO_MIN || pldoNumber > PMIC_POWER_PLDO_MAX) {
      pmicStatus = PMIC_ST_ERR_INV_PARAM;
    } else {
      Pmic_getPLDOCfgFields(pldoNumber, pldoConfig, pwrRsrcCfg, pwrRsrcRegCfg);

      if (pldoConfig->pldoModeSel != PMIC_ST_DEFAULT_DATA) {
        pmicStatus = Pmic_powerSetPLDOModeSel(pPmicCoreHandle, pldoConfig);
      }

      if (pldoConfig->pldoIlimLvlCfg != PMIC_ST_DEFAULT_DATA) {
        pmicStatus = Pmic_powerSetPLDOIlimLvlCfg(pPmicCoreHandle, pldoConfig);
      }

      if (pldoConfig->pldoLvlCfg != PMIC_ST_DEFAULT_DATA) {
        pmicStatus = Pmic_powerSetPLDOLvlCfg(pPmicCoreHandle, pldoConfig);
      }

      initializePLDOCfgReg(pldoConfig);
    }
  }

  return pmicStatus;
}


int32_t Pmic_powerGetPLDOConfigRegister(void *pPmicCoreHandle,
                                        uint8_t pldoNumber,
                                        Pmic_pldoCfgReg_t *pldoConfig,
                                        Pmic_powerRsrcCfg_t *pwrRsrcCfg,
                                        Pmic_powerRsrcRegCfg_t *pwrRsrcRegCfg) {
  uint8_t pmicStatus = PMIC_ST_SUCCESS;

  if (pPmicCoreHandle == NULL) {
    pmicStatus = PMIC_ST_ERR_INV_HANDLE;
  } else {
    if (pldoNumber < PMIC_POWER_PLDO_MIN || pldoNumber > PMIC_POWER_PLDO_MAX) {
      pmicStatus = PMIC_ST_ERR_INV_PARAM;
    } else {
      Pmic_getPLDOCfgFields(pldoNumber, pldoConfig, pwrRsrcCfg, pwrRsrcRegCfg);

      if (pldoConfig->pldoModeSel != PMIC_ST_DEFAULT_DATA) {
        pmicStatus = Pmic_powerGetPLDOModeSel(pPmicCoreHandle, pldoConfig);
      }

      if (pldoConfig->pldoIlimLvlCfg != PMIC_ST_DEFAULT_DATA) {
        pmicStatus = Pmic_powerGetPLDOIlimLvlCfg(pPmicCoreHandle, pldoConfig);
      }

      if (pldoConfig->pldoLvlCfg != PMIC_ST_DEFAULT_DATA) {
        pmicStatus = Pmic_powerGetPLDOLvlCfg(pPmicCoreHandle, pldoConfig);
      }

      initializePLDOCfgReg(pldoConfig);
    }
  }

  return pmicStatus;
}


int32_t Pmic_setPLDOVTrackRng(Pmic_CoreHandle_t *pPmicCoreHandle,
                              Pmic_pldoVTrackRtReg_t *pldoCfg) {
  uint8_t pmicStatus = PMIC_ST_SUCCESS;
  uint8_t regAddr = pldoCfg->pldoVTrackRTRegAddr;
  uint8_t regData = 0U;

  Pmic_criticalSectionStart(pPmicCoreHandle);
  pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, regAddr, &regData);

  if (PMIC_ST_SUCCESS == pmicStatus) {
    Pmic_setBitField(&regData, pldoCfg->pldoVTrackRTRegShift,
                     pldoCfg->pldoVTrackRTRegMask, pldoCfg->pldoVTrackRng);

    pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle, regAddr, regData);
  }

  Pmic_criticalSectionStop(pPmicCoreHandle);

  return pmicStatus;
}


int32_t Pmic_getPLDOVTrackRng(Pmic_CoreHandle_t *pPmicCoreHandle,
                              Pmic_pldoVTrackRtReg_t *pldoCfg) {
  uint8_t pmicStatus = PMIC_ST_SUCCESS;
  uint8_t regAddr = pldoCfg->pldoVTrackRTRegAddr;
  uint8_t regData = 0U;

  Pmic_criticalSectionStart(pPmicCoreHandle);
  pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, regAddr, &regData);

  if (PMIC_ST_SUCCESS == pmicStatus) {
      pldoCfg->pldoVTrackRng = Pmic_getBitField(regData,
                                pldoCfg->pldoVTrackRTRegShift,
                                pldoCfg->pldoVTrackRTRegMask);
  }
  Pmic_criticalSectionStop(pPmicCoreHandle);

  return pmicStatus;
}


int32_t Pmic_setPLDO1RTCfg(Pmic_CoreHandle_t *pPmicCoreHandle,
                           Pmic_pldoVTrackRtReg_t *pldoCfg) {
  uint8_t pmicStatus = PMIC_ST_SUCCESS;
  uint8_t regAddr = pldoCfg->pldoVTrackRTRegAddr;
  uint8_t regData = 0U;

  Pmic_criticalSectionStart(pPmicCoreHandle);
  pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, regAddr, &regData);

  if (PMIC_ST_SUCCESS == pmicStatus) {
    Pmic_setBitField(&regData, pldoCfg->pldoVTrackRTRegShift,
                     pldoCfg->pldoVTrackRTRegMask, pldoCfg->pldo1RTCfgVal);

    pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle, regAddr, regData);
  }

  Pmic_criticalSectionStop(pPmicCoreHandle);

  return pmicStatus;
}


int32_t Pmic_getPLDO1RTCfg(Pmic_CoreHandle_t *pPmicCoreHandle,
                           Pmic_pldoVTrackRtReg_t *pldoCfg) {
  uint8_t pmicStatus = PMIC_ST_SUCCESS;
  uint8_t regAddr = pldoCfg->pldoVTrackRTRegAddr;
  uint8_t regData = 0U;

  Pmic_criticalSectionStart(pPmicCoreHandle);
  pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, regAddr, &regData);

  if (PMIC_ST_SUCCESS == pmicStatus) {
      pldoCfg->pldo1RTCfgVal = Pmic_getBitField(regData,
                                pldoCfg->pldoVTrackRTRegShift,
                                pldoCfg->pldoVTrackRTRegMask);
  }
  Pmic_criticalSectionStop(pPmicCoreHandle);

  return pmicStatus;
}


int32_t Pmic_setPLDO2RTCfg(Pmic_CoreHandle_t *pPmicCoreHandle,
                           Pmic_pldoVTrackRtReg_t *pldoCfg) {
  uint8_t pmicStatus = PMIC_ST_SUCCESS;
  uint8_t regAddr = pldoCfg->pldoVTrackRTRegAddr;
  uint8_t regData = 0U;

  Pmic_criticalSectionStart(pPmicCoreHandle);
  pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, regAddr, &regData);

  if (PMIC_ST_SUCCESS == pmicStatus) {
    Pmic_setBitField(&regData, pldoCfg->pldoVTrackRTRegShift,
                     pldoCfg->pldoVTrackRTRegMask, pldoCfg->pldo2RTCfgVal);

    pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle, regAddr, regData);
  }

  Pmic_criticalSectionStop(pPmicCoreHandle);

  return pmicStatus;
}


int32_t Pmic_getPLDO2RTCfg(Pmic_CoreHandle_t *pPmicCoreHandle,
                           Pmic_pldoVTrackRtReg_t *pldoCfg) {
  uint8_t pmicStatus = PMIC_ST_SUCCESS;
  uint8_t regAddr = pldoCfg->pldoVTrackRTRegAddr;
  uint8_t regData = 0U;

  Pmic_criticalSectionStart(pPmicCoreHandle);
  pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, regAddr, &regData);

  if (PMIC_ST_SUCCESS == pmicStatus) {
      pldoCfg->pldo2RTCfgVal = Pmic_getBitField(regData,
                                pldoCfg->pldoVTrackRTRegShift,
                                pldoCfg->pldoVTrackRTRegMask);
  }
  Pmic_criticalSectionStop(pPmicCoreHandle);

  return pmicStatus;
}


int32_t Pmic_SetPLDOVTrackRTRegCfg(void *pPmicCoreHandle, uint8_t pldoNumber,
                                   Pmic_pldoVTrackRtReg_t *pldoConfig,
                                   Pmic_powerRsrcCfg_t *pwrRsrcCfg,
                                   Pmic_powerRsrcRegCfg_t *pwrRsrcRegCfg) {
  uint8_t pmicStatus = PMIC_ST_SUCCESS;

  if (pPmicCoreHandle == NULL) {
    pmicStatus = PMIC_ST_ERR_INV_HANDLE;
  } else {
    if (pldoNumber < PMIC_POWER_PLDO_MIN || pldoNumber > PMIC_POWER_PLDO_MAX) {
      pmicStatus = PMIC_ST_ERR_INV_PARAM;
    } else {
      pldoConfig->pldoVTrackRTRegAddr = pwrRsrcRegCfg->pldoConfigRegAddr;
      pldoConfig->pldoVTrackRTRegShift = pwrRsrcCfg->pmicConfigShiftVal;
      pldoConfig->pldoVTrackRTRegMask = pwrRsrcCfg->pmicConfigMaskVal;

      if (pldoConfig->pldoVTrackRng != PMIC_ST_DEFAULT_DATA) {
        pmicStatus = Pmic_setPLDOVTrackRng(pPmicCoreHandle, pldoConfig);
      }

      if (pldoConfig->pldo1RTCfgVal != PMIC_ST_DEFAULT_DATA) {
        pmicStatus = Pmic_setPLDO1RTCfg(pPmicCoreHandle, pldoConfig);
      }

      if (pldoConfig->pldo2RTCfgVal != PMIC_ST_DEFAULT_DATA) {
        pmicStatus = Pmic_setPLDO2RTCfg(pPmicCoreHandle, pldoConfig);
      }

      initializePLDOVTrackRTReg(pldoConfig);
    }
  }

  return pmicStatus;
}


int32_t Pmic_GetPLDOVTrackRTRegCfg(void *pPmicCoreHandle, uint8_t pldoNumber,
                                   Pmic_pldoVTrackRtReg_t *pldoConfig,
                                   Pmic_powerRsrcCfg_t *pwrRsrcCfg,
                                   Pmic_powerRsrcRegCfg_t *pwrRsrcRegCfg) {
  uint8_t pmicStatus = PMIC_ST_SUCCESS;

  if (pPmicCoreHandle == NULL) {
    pmicStatus = PMIC_ST_ERR_INV_HANDLE;
  } else {
    if (pldoNumber < PMIC_POWER_PLDO_MIN || pldoNumber > PMIC_POWER_PLDO_MAX) {
      pmicStatus = PMIC_ST_ERR_INV_PARAM;
    } else {
      pldoConfig->pldoVTrackRTRegAddr = pwrRsrcRegCfg->pldoConfigRegAddr;
      pldoConfig->pldoVTrackRTRegShift = pwrRsrcCfg->pmicConfigShiftVal;
      pldoConfig->pldoVTrackRTRegMask = pwrRsrcCfg->pmicConfigMaskVal;

      if (pldoConfig->pldoVTrackRng != PMIC_ST_DEFAULT_DATA) {
        pmicStatus = Pmic_getPLDOVTrackRng(pPmicCoreHandle, pldoConfig);
      }

      if (pldoConfig->pldo1RTCfgVal != PMIC_ST_DEFAULT_DATA) {
        pmicStatus = Pmic_getPLDO1RTCfg(pPmicCoreHandle, pldoConfig);
      }

      if (pldoConfig->pldo2RTCfgVal != PMIC_ST_DEFAULT_DATA) {
        pmicStatus = Pmic_getPLDO2RTCfg(pPmicCoreHandle, pldoConfig);
      }

      initializePLDOVTrackRTReg(pldoConfig);
    }
  }

  return pmicStatus;
}


int32_t Pmic_setPldoPgoodCfg(Pmic_CoreHandle_t *pPmicCoreHandle,
                             uint8_t pldoNumber,
                             Pmic_pgoodCfgReg_t *pldoPgoodCfg,
                             Pmic_powerRsrcCfg_t *pwrRsrcCfg,
                             Pmic_powerRsrcRegCfg_t *pwrRsrcRegCfg) {
  uint8_t pmicStatus = PMIC_ST_SUCCESS;
  uint8_t regAddr = pldoPgoodCfg->pgoodRegAddr;
  uint8_t regData = 0U;

  if (pPmicCoreHandle == NULL) {
    pmicStatus = PMIC_ST_ERR_INV_PARAM;
  } else {
    Pmic_criticalSectionStart(pPmicCoreHandle);
    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, regAddr, &regData);

    switch (pldoNumber) {
    case PMIC_PLDO1:
      Pmic_setBitField(&regData, pldoPgoodCfg->pgoodRegShift,
                       pldoPgoodCfg->pgoodRegMask, pldoPgoodCfg->pldo1PgoodCfg);
      pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle, regAddr, regData);
      break;
    case PMIC_PLDO2:
      Pmic_setBitField(&regData, pldoPgoodCfg->pgoodRegShift,
                       pldoPgoodCfg->pgoodRegMask, pldoPgoodCfg->pldo2PgoodCfg);
      pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle, regAddr, regData);
      break;
    // Add more cases if additional PLDOs are present
    default:
      pmicStatus = PMIC_ST_ERR_INV_PARAM;
      break;
    }

    Pmic_criticalSectionStop(pPmicCoreHandle);
  }

  return pmicStatus;
}


int32_t Pmic_getPldoPgoodCfg(Pmic_CoreHandle_t *pPmicCoreHandle,
                             uint8_t pldoNumber,
                             Pmic_pgoodCfgReg_t *pldoPgoodCfg,
                             Pmic_powerRsrcCfg_t *pwrRsrcCfg,
                             Pmic_powerRsrcRegCfg_t *pwrRsrcRegCfg) {
  uint8_t pmicStatus = PMIC_ST_SUCCESS;
  uint8_t regAddr = pldoPgoodCfg->pgoodRegAddr;
  uint8_t regData = 0U;

  if (pPmicCoreHandle == NULL) {
    pmicStatus = PMIC_ST_ERR_INV_PARAM;
  } else {
    Pmic_criticalSectionStart(pPmicCoreHandle);
    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, regAddr, &regData);

    switch (pldoNumber) {
    case PMIC_PLDO1:
        pldoPgoodCfg->pldo1PgoodCfg = Pmic_getBitField(regData,
                                       pldoPgoodCfg->pgoodRegShift,
                                       pldoPgoodCfg->pgoodRegMask);
      break;
    case PMIC_PLDO2:
        pldoPgoodCfg->pldo2PgoodCfg = Pmic_getBitField(regData,
                                       pldoPgoodCfg->pgoodRegShift,
                                       pldoPgoodCfg->pgoodRegMask);
      break;
    default:
      pmicStatus = PMIC_ST_ERR_INV_PARAM;
      break;
    }
    Pmic_criticalSectionStop(pPmicCoreHandle);
  }

  return pmicStatus;
}


int32_t Pmic_setLdoPgoodCfg(Pmic_CoreHandle_t *pPmicCoreHandle,
                            uint8_t ldoNumber, Pmic_pgoodCfgReg_t *ldoPgoodCfg,
                            Pmic_powerRsrcCfg_t *pwrRsrcCfg,
                            Pmic_powerRsrcRegCfg_t *pwrRsrcRegCfg) {
  uint8_t pmicStatus = PMIC_ST_SUCCESS;
  uint8_t regAddr = ldoPgoodCfg->pgoodRegAddr;
  uint8_t regData = 0U;

  if (pPmicCoreHandle == NULL) {
    return PMIC_ST_ERR_INV_PARAM;
  } else {
    Pmic_criticalSectionStart(pPmicCoreHandle);
    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, regAddr, &regData);

    switch (ldoNumber) {
    case PMIC_LDO1:
      Pmic_setBitField(&regData, ldoPgoodCfg->pgoodRegShift,
                       ldoPgoodCfg->pgoodRegMask, ldoPgoodCfg->ldo1PgoodCfg);
      pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle, regAddr, regData);
      break;
    case PMIC_LDO2:
      Pmic_setBitField(&regData, ldoPgoodCfg->pgoodRegShift,
                       ldoPgoodCfg->pgoodRegMask, ldoPgoodCfg->ldo2PgoodCfg);
      pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle, regAddr, regData);
      break;
    case PMIC_LDO3:
      Pmic_setBitField(&regData, ldoPgoodCfg->pgoodRegShift,
                       ldoPgoodCfg->pgoodRegMask, ldoPgoodCfg->ldo3PgoodCfg);
      pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle, regAddr, regData);
      break;
    case PMIC_LDO4:
      Pmic_setBitField(&regData, ldoPgoodCfg->pgoodRegShift,
                       ldoPgoodCfg->pgoodRegMask, ldoPgoodCfg->ldo4PgoodCfg);
      pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle, regAddr, regData);
      break;
    // Add more cases if additional LDOs are present
    default:
      pmicStatus = PMIC_ST_ERR_INV_PARAM;
      break;
    }

    Pmic_criticalSectionStop(pPmicCoreHandle);
  }

  return pmicStatus;
}


int32_t Pmic_getLdoPgoodCfg(Pmic_CoreHandle_t *pPmicCoreHandle,
                            uint8_t ldoNumber, Pmic_pgoodCfgReg_t *ldoPgoodCfg,
                            Pmic_powerRsrcCfg_t *pwrRsrcCfg,
                            Pmic_powerRsrcRegCfg_t *pwrRsrcRegCfg) {
  uint8_t pmicStatus = PMIC_ST_SUCCESS;
  uint8_t regAddr = ldoPgoodCfg->pgoodRegAddr;
  uint8_t regData = 0U;

  if (pPmicCoreHandle == NULL) {
    return PMIC_ST_ERR_INV_PARAM;
  } else {
    Pmic_criticalSectionStart(pPmicCoreHandle);
    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, regAddr, &regData);

    switch (ldoNumber) {
    case PMIC_LDO1:
        ldoPgoodCfg->ldo1PgoodCfg = Pmic_getBitField(regData,
                                     ldoPgoodCfg->pgoodRegShift,
                                     ldoPgoodCfg->pgoodRegMask);
        break;
    case PMIC_LDO2:
        ldoPgoodCfg->ldo2PgoodCfg = Pmic_getBitField(regData,
                                     ldoPgoodCfg->pgoodRegShift,
                                     ldoPgoodCfg->pgoodRegMask);
        break;
    case PMIC_LDO3:
        ldoPgoodCfg->ldo3PgoodCfg = Pmic_getBitField(regData,
                                     ldoPgoodCfg->pgoodRegShift,
                                     ldoPgoodCfg->pgoodRegMask);
        break;
    case PMIC_LDO4:
        ldoPgoodCfg->ldo4PgoodCfg = Pmic_getBitField(regData,
                                     ldoPgoodCfg->pgoodRegShift,
                                     ldoPgoodCfg->pgoodRegMask);
      break;
    default:
      pmicStatus = PMIC_ST_ERR_INV_PARAM;
      break;
    }
    Pmic_criticalSectionStop(pPmicCoreHandle);
  }

  return pmicStatus;
}


void Pmic_getLDOCtrlFields(uint8_t ldoNumber, uint8_t *pBitPos,
                           uint8_t *pBitMask) {
  switch (ldoNumber) {
  case PMIC_LDO1:
    *pBitPos = LDO1_CTRL_SHIFT;
    *pBitMask = LDO1_CTRL_MASK;
    break;
  case PMIC_LDO2:
    *pBitPos = LDO2_CTRL_SHIFT;
    *pBitMask = LDO2_CTRL_MASK;
    break;
  case PMIC_LDO3:
    *pBitPos = LDO3_CTRL_SHIFT;
    *pBitMask = LDO3_CTRL_MASK;
    break;
  case PMIC_LDO4:
    *pBitPos = LDO4_CTRL_SHIFT;
    *pBitMask = LDO4_CTRL_MASK;
    break;
  default:
    *pBitPos = PMIC_ST_DEFAULT_DATA;
    *pBitMask = PMIC_ST_DEFAULT_DATA;
    break;
  }
}


int32_t Pmic_setLdoCtrl(Pmic_CoreHandle_t *pPmicCoreHandle, uint8_t ldoNumber,
                        uint8_t ldoCtrlFeature,
                        Pmic_powerRsrcRegCfg_t *pwrRsrcRegCfg) {
  uint8_t pmicStatus = PMIC_ST_SUCCESS;
  uint8_t regAddr = pwrRsrcRegCfg->ldoCtrlRegAddr;
  uint8_t regData = 0U;
  uint8_t bitPos = 0U;
  uint8_t bitMask = 0U;

  if (pPmicCoreHandle == NULL) {
    pmicStatus = PMIC_ST_ERR_INV_PARAM;
  } else {
    Pmic_getLDOCtrlFields(ldoNumber, &bitPos, &bitMask);

    if (bitPos == PMIC_ST_DEFAULT_DATA && bitMask == PMIC_ST_DEFAULT_DATA) {
      pmicStatus = PMIC_ST_ERR_INV_PARAM;
    } else {
      Pmic_criticalSectionStart(pPmicCoreHandle);
      pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, regAddr, &regData);

      if (pmicStatus != PMIC_ST_SUCCESS) {
        pmicStatus = PMIC_ST_ERR_FAIL;
      } else {
        Pmic_setBitField(&regData, bitPos, bitMask, ldoCtrlFeature);

        pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle, regAddr, regData);

        Pmic_criticalSectionStop(pPmicCoreHandle);
      }
    }
  }

  return pmicStatus;
}


int32_t Pmic_getLdoCtrl(Pmic_CoreHandle_t *pPmicCoreHandle,
                        uint8_t ldoNumber,
                        uint8_t ldoCtrlFeature,
                        Pmic_powerRsrcRegCfg_t *pwrRsrcRegCfg) {
  uint8_t pmicStatus = PMIC_ST_SUCCESS;
  uint8_t regAddr = pwrRsrcRegCfg->ldoCtrlRegAddr;
  uint8_t regData = 0U;
  uint8_t bitPos = 0U;
  uint8_t bitMask = 0U;

  if (pPmicCoreHandle == NULL) {
    pmicStatus = PMIC_ST_ERR_INV_PARAM;
  } else {
    Pmic_getLDOCtrlFields(ldoNumber, &bitPos, &bitMask);

    if (bitPos == PMIC_ST_DEFAULT_DATA && bitMask == PMIC_ST_DEFAULT_DATA) {
      pmicStatus = PMIC_ST_ERR_INV_PARAM;
    } else {
      Pmic_criticalSectionStart(pPmicCoreHandle);
      pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, regAddr, &regData);
      if (pmicStatus != PMIC_ST_SUCCESS) {
        pmicStatus = PMIC_ST_ERR_FAIL;
      } else {
          ldoCtrlFeature = Pmic_getBitField(regData, bitPos, bitMask);

        Pmic_criticalSectionStop(pPmicCoreHandle);
      }
    }
  }

  return ldoCtrlFeature;
}


void Pmic_getEnOutCtrlFields(uint8_t enableNumber, uint8_t *pBitPos,
                             uint8_t *pBitMask) {
  switch (enableNumber) {
  case PMIC_EN_OUT_ALL:
    *pBitPos = EN_OUTALL_ENABLE_SHIFT;
    *pBitMask = EN_OUTALL_ENABLE_MASK;
    break;
  case PMIC_EN_OUT2:
    *pBitPos = EN_OUT2_ENABLE_SHIFT;
    *pBitMask = EN_OUT2_ENABLE_MASK;
    break;
  case PMIC_EN_OUT1:
    *pBitPos = EN_OUT1_ENABLE_SHIFT;
    *pBitMask = EN_OUT1_ENABLE_MASK;
    break;
  default:
    *pBitPos = PMIC_ST_DEFAULT_DATA;
    *pBitMask = PMIC_ST_DEFAULT_DATA;
    break;
  }
}

void Pmic_getPLDOCtrlFields(uint8_t pldoNumber, uint8_t *pBitPos,
                            uint8_t *pBitMask) {
  switch (pldoNumber) {
  case PMIC_PLDO1:
    *pBitPos = PLDO1_CTRL_SHIFT;
    *pBitMask = PLDO1_CTRL_MASK;
    break;
  case PMIC_PLDO2:
    *pBitPos = PLDO2_CTRL_SHIFT;
    *pBitMask = PLDO2_CTRL_MASK;
    break;
  default:
    *pBitPos = PMIC_ST_DEFAULT_DATA;
    *pBitMask = PMIC_ST_DEFAULT_DATA;
    break;
  }
}


int32_t Pmic_setPowerEnOutCtrlReg(Pmic_CoreHandle_t *pPmicCoreHandle,
                               Pmic_pldoEnOutCtrlReg_t *pldoEnOutCfg) {
  int32_t pmicStatus = PMIC_ST_SUCCESS;
  uint8_t regAddr = pldoEnOutCfg->pldoEnOutRegAddr;
  uint8_t regData = 0U;
  uint8_t bitPos = 0U;
  uint8_t bitMask = 0U;
  uint8_t bitVal = 0U;

  if (pPmicCoreHandle == NULL) {
    pmicStatus = PMIC_ST_ERR_INV_PARAM;
  } else {
    uint8_t combinedValue =
        (pldoEnOutCfg->enOut2Enable << 1) | pldoEnOutCfg->enOut1Enable;
    switch (combinedValue) {
    case 0b01: /* enOut2Enable = 0, enOut1Enable = 1 */
      bitVal = 1U;
      Pmic_getEnOutCtrlFields(PMIC_EN_OUT1, &bitPos, &bitMask);
      break;
    case 0b10: /* enOut2Enable = 1, enOut1Enable = 0 */
      bitVal = 1U;
      Pmic_getEnOutCtrlFields(PMIC_EN_OUT2, &bitPos, &bitMask);
      break;
    case 0b11: /* enOut2Enable = 1, enOut1Enable = 1 */
      bitVal = 3U;
      Pmic_getEnOutCtrlFields(PMIC_EN_OUT_ALL, &bitPos, &bitMask);
      break;
    default:
      pmicStatus = PMIC_ST_ERR_FAIL;
      break;
    }

    Pmic_criticalSectionStart(pPmicCoreHandle);

    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, regAddr, &regData);
    if (PMIC_ST_SUCCESS != pmicStatus) {
      pmicStatus = PMIC_ST_ERR_FAIL;
    } else {
      Pmic_setBitField(&regData, bitPos, bitMask, bitVal);

      pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle, regAddr, regData);

      Pmic_criticalSectionStop(pPmicCoreHandle);
    }
  }

  return pmicStatus;
}


int32_t Pmic_setPowerPLDOCtrlReg(Pmic_CoreHandle_t *pPmicCoreHandle,
                              Pmic_pldoEnOutCtrlReg_t *pldoEnOutCfg) {
  int32_t pmicStatus = PMIC_ST_SUCCESS;
  uint8_t regAddr = pldoEnOutCfg->pldoEnOutRegAddr;
  uint8_t regData = 0U;
  uint8_t bitPos = 0U;
  uint8_t bitMask = 0U;
  uint8_t bitVal = 0U;

  if (pPmicCoreHandle == NULL) {
    pmicStatus = PMIC_ST_ERR_INV_PARAM;
  } else {
    if (pldoEnOutCfg->pldo2Ctrl != PMIC_ST_DEFAULT_DATA) {
      bitVal = pldoEnOutCfg->pldo2Ctrl;
      Pmic_getPLDOCtrlFields(PMIC_PLDO2, &bitPos, &bitMask);
    }

    if (pldoEnOutCfg->pldo1Ctrl != PMIC_ST_DEFAULT_DATA) {
      bitVal = pldoEnOutCfg->pldo1Ctrl;
      Pmic_getPLDOCtrlFields(PMIC_PLDO1, &bitPos, &bitMask);
    }

    Pmic_criticalSectionStart(pPmicCoreHandle);

    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, regAddr, &regData);
    if (PMIC_ST_SUCCESS != pmicStatus) {
      pmicStatus = PMIC_ST_ERR_FAIL;
    } else {
      Pmic_setBitField(&regData, bitPos, bitMask, bitVal);

      pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle, regAddr, regData);

      Pmic_criticalSectionStop(pPmicCoreHandle);
    }
  }

  return pmicStatus;
}


int32_t Pmic_setPowerPLDOEnOutControl(Pmic_CoreHandle_t *pPmicCoreHandle,
                                   Pmic_pldoEnOutCtrlReg_t *pldoEnOutCfg,
                                   Pmic_powerRsrcRegCfg_t *pwrRsrcRegCfg) {
  int32_t pmicStatus = PMIC_ST_SUCCESS;

  if ((pldoEnOutCfg->pldo2Ctrl != PMIC_ST_DEFAULT_DATA) ||
      (pldoEnOutCfg->pldo1Ctrl != PMIC_ST_DEFAULT_DATA)) {
    pldoEnOutCfg->pldoEnOutRegAddr = pwrRsrcRegCfg->enoutCtrlRegAddr;
    pmicStatus = Pmic_setPowerPLDOCtrlReg(pPmicCoreHandle, pldoEnOutCfg);
    if (PMIC_ST_SUCCESS != pmicStatus) {
      pmicStatus = PMIC_ST_ERR_FAIL;
    }
  }

  if ((pldoEnOutCfg->enOut2Enable != PMIC_ST_DEFAULT_DATA) ||
      (pldoEnOutCfg->enOut1Enable != PMIC_ST_DEFAULT_DATA)) {
    pldoEnOutCfg->pldoEnOutRegAddr = pwrRsrcRegCfg->enoutCtrlRegAddr;
    pmicStatus = Pmic_setPowerEnOutCtrlReg(pPmicCoreHandle, pldoEnOutCfg);
    if (PMIC_ST_SUCCESS != pmicStatus) {
      pmicStatus = PMIC_ST_ERR_FAIL;
    }
  }

  return pmicStatus;
}


int32_t Pmic_setPLDODscgDisCtrl(Pmic_CoreHandle_t *pPmicCoreHandle,
                                Pmic_powerRsrcCfg_t *dscgConfig,
                                Pmic_DscgDisCtrlReg_t *dscgDisCtrlCfg) {
  int32_t pmicStatus = PMIC_ST_SUCCESS;
  uint8_t regAddr = dscgDisCtrlCfg->dscgDisCtrlRegAddr;
  uint8_t regData = 0U;
  uint8_t bitPos = 0U;
  uint8_t bitMask = 0U;

  if (pPmicCoreHandle == NULL) {
    pmicStatus = PMIC_ST_ERR_INV_PARAM;
  } else {
    bitPos = dscgConfig->pmicConfigShiftVal;
    bitMask = dscgConfig->pmicConfigMaskVal;

    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, regAddr, &regData);

    if (PMIC_ST_SUCCESS == pmicStatus) {
      if (dscgDisCtrlCfg->pldo2DscgDis != PMIC_ST_DEFAULT_DATA) {
        Pmic_setBitField(&regData, bitPos, bitMask,
                         dscgDisCtrlCfg->pldo2DscgDis);
      }

      if (dscgDisCtrlCfg->pldo1DscgDis != PMIC_ST_DEFAULT_DATA) {
        Pmic_setBitField(&regData, bitPos, bitMask,
                         dscgDisCtrlCfg->pldo1DscgDis);
      }

      pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle, regAddr, regData);
    }
  }

  return pmicStatus;
}


int32_t Pmic_setLDODscgDisCtrl(Pmic_CoreHandle_t *pPmicCoreHandle,
                               Pmic_powerRsrcCfg_t *dscgConfig,
                               Pmic_DscgDisCtrlReg_t *dscgDisCtrlCfg) {
  int32_t pmicStatus = PMIC_ST_SUCCESS;
  uint8_t regAddr = dscgDisCtrlCfg->dscgDisCtrlRegAddr;
  uint8_t regData = 0U;
  uint8_t bitPos = 0U;
  uint8_t bitMask = 0U;

  if (pPmicCoreHandle == NULL) {
    pmicStatus = PMIC_ST_ERR_INV_PARAM;
  } else {
    bitPos = dscgConfig->pmicConfigShiftVal;
    bitMask = dscgConfig->pmicConfigMaskVal;

    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, regAddr, &regData);

    if (PMIC_ST_SUCCESS == pmicStatus) {
      if (dscgDisCtrlCfg->ldo4DscgDis != PMIC_ST_DEFAULT_DATA) {
        Pmic_setBitField(&regData, bitPos, bitMask,
                         dscgDisCtrlCfg->ldo4DscgDis);
      }

      if (dscgDisCtrlCfg->ldo3DscgDis != PMIC_ST_DEFAULT_DATA) {
        Pmic_setBitField(&regData, bitPos, bitMask,
                         dscgDisCtrlCfg->ldo3DscgDis);
      }

      if (dscgDisCtrlCfg->ldo2DscgDis != PMIC_ST_DEFAULT_DATA) {
        Pmic_setBitField(&regData, bitPos, bitMask,
                         dscgDisCtrlCfg->ldo2DscgDis);
      }

      if (dscgDisCtrlCfg->ldo1DscgDis != PMIC_ST_DEFAULT_DATA) {
        Pmic_setBitField(&regData, bitPos, bitMask,
                         dscgDisCtrlCfg->ldo1DscgDis);
      }

      pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle, regAddr, regData);
    } else {
      pmicStatus = PMIC_ST_ERR_FAIL;
    }
  }

  return pmicStatus;
}


int32_t Pmic_setPowerDscgDisControl(Pmic_CoreHandle_t *pPmicCoreHandle,
                                 Pmic_DscgDisCtrlReg_t *dscgDisCtrlCfg,
                                 Pmic_powerRsrcCfg_t *dscgConfig,
                                 Pmic_powerRsrcRegCfg_t *pwrRsrcRegCfg) {
  int32_t pmicStatus = PMIC_ST_SUCCESS;

  if (pPmicCoreHandle == NULL) {
    pmicStatus = PMIC_ST_ERR_INV_PARAM;
  } else {

    dscgDisCtrlCfg->dscgDisCtrlRegAddr = pwrRsrcRegCfg->dscgConfigRegAddr;

    if ((dscgDisCtrlCfg->pldo1DscgDis != PMIC_ST_DEFAULT_DATA) ||
        (dscgDisCtrlCfg->pldo2DscgDis != PMIC_ST_DEFAULT_DATA)) {
      pmicStatus =
          Pmic_setPLDODscgDisCtrl(pPmicCoreHandle, dscgConfig, dscgDisCtrlCfg);
      if (PMIC_ST_SUCCESS != pmicStatus) {
        pmicStatus = PMIC_ST_ERR_FAIL;
      }
    } else {
      pmicStatus =
          Pmic_setLDODscgDisCtrl(pPmicCoreHandle, dscgConfig, dscgDisCtrlCfg);
      if (PMIC_ST_SUCCESS != pmicStatus) {
        pmicStatus = PMIC_ST_ERR_FAIL;
      }
    }
  }

  return pmicStatus;
}


