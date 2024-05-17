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
 * @file pmic_power.h
 *
 * @brief PMIC LLD Power module header file
 * 
 * @details This module contains declarations/definitions of macros, data 
 * structures, and APIs used to interact with PMIC regulators and related
 * components. Some components of the PMIC Power module are as follows:
 * setting/getting regulator configurations, setting/getting thermal
 * shutdown configurations, and getting regulator and TSD statuses
 */
#ifndef __PMIC_POWER_H__
#define __PMIC_POWER_H__

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                Include Files                               */
/* ========================================================================== */
#include "pmic_common.h"

/* ========================================================================== */
/*                               Macros & Typedefs                            */
/* ========================================================================== */

/* ========================================================================== */
/*                             Structures and Enums                           */
/* ========================================================================== */



typedef struct Pmic_PwrBuckCfg_s
{
    uint32_t validParams;

    /* BUCK1 only */
    uint8_t vset;
    uint8_t uvloRising;
    uint8_t uvloFalling;
    uint8_t highSideSlewRate;

    /* BUCK2 and BUCK3 only  */
    uint8_t vsetActive;
    uint8_t vsetLPwr;
    bool vmonOnly;

    /* All bucks */
    bool enable;
    bool pldnEn;
    uint8_t fpwm;
    uint8_t slewRate;
    uint8_t deglitchSel;
    uint8_t dischargeSel;
    uint8_t uvThr;
    uint8_t ovThr;
    uint8_t ilimSel;
    uint8_t ovpSel;
    uint8_t ovSel;
    uint8_t uvSel;
    uint8_t scSel;
    uint8_t rvConf;
    bool ssEn;
    uint8_t ssmSel;
} Pmic_PwrBuckCfg_t;

typedef struct Pmic_PwrLdoCfg_s
{
    uint16_t validParams;
    bool enable;
    uint8_t mode;
    uint8_t vset;
    bool vmonOnly;
    uint8_t deglitchSel;
    uint8_t dischargeSel;
    bool dischargeEn;
    uint8_t uvThr;
    uint8_t ovThr;
    uint8_t ilimSel;
    uint8_t ovpSel;
    uint8_t ovSel;
    uint8_t uvSel;
    uint8_t scSel;
    uint8_t rvConf;
} Pmic_PwrLdoCfg_t;

typedef struct Pmic_PwrBuckLdoStat_s
{
    uint16_t validParams;
    bool buck1Active;
    bool buck1OV;
    bool buck1UV;
    bool buck1OVP;
    bool buck2Active;
    bool buck2OV;
    bool buck2UV;
    bool buck2OVP;
    bool buck3Active;
    bool buck3OV;
    bool buck3UV;
    bool buck3OVP;
    bool ldoActive;
    bool ldoOV;
    bool ldoUV;
    bool ldoOVP;
} Pmic_PwrBuckLdoStat_t;

typedef struct Pmic_PwrTsdCfg_s
{
    uint8_t validParams;
    uint8_t twarnCfg;
    uint8_t tsdImmLvl;
    uint8_t twarnLvl;
} Pmic_PwrTsdCfg_t;

/* ========================================================================== */
/*                             Function Declarations                          */
/* ========================================================================== */

int32_t Pmic_pwrSetBuckCfg(const Pmic_CoreHandle_t *pmicHandle, const Pmic_PwrBuckCfg_t *buckCfg);

int32_t Pmic_pwrGetBuckCfg(const Pmic_CoreHandle_t *pmicHandle, Pmic_PwrBuckCfg_t *buckCfg);

int32_t Pmic_pwrSetLdoCfg(const Pmic_CoreHandle_t *pmicHandle, const Pmic_PwrLdoCfg_t *ldoCfg);

int32_t Pmic_pwrGetLdoCfg(const Pmic_CoreHandle_t *pmicHandle, Pmic_PwrLdoCfg_t *ldoCfg);

int32_t Pmic_pwrGetBuckLdoStat(const Pmic_CoreHandle_t *pmicHandle, Pmic_PwrBuckLdoStat_t *buckLdoStat);

int32_t Pmic_pwrSetTsdCfg(const Pmic_CoreHandle_t *pmicHandle, const Pmic_PwrTsdCfg_t *tsdCfg);

int32_t Pmic_pwrGetTsdCfg(const Pmic_CoreHandle_t *pmicHandle, Pmic_PwrTsdCfg_t *tsdCfg);

int32_t Pmic_pwrGetTsdImmStat(const Pmic_CoreHandle_t *pmicHandle, bool *tsdImmStat);

int32_t Pmic_pwrSetBuckLdoSeqTrig(const Pmic_CoreHandle_t *pmicHandle, uint8_t seqTrig, bool set);

int32_t Pmic_pwrGetBuckLdoSeqTrig(const Pmic_CoreHandle_t *pmicHandle, uint8_t seqTrig, bool *set);

int32_t Pmic_pwrSetBuckLdoSeqDly(const Pmic_CoreHandle_t *pmicHandle, uint8_t seqDly, uint8_t delay);

int32_t Pmic_pwrGetBuckLdoSeqDly(const Pmic_CoreHandle_t *pmicHandle, uint8_t seqDly, uint8_t *delay);

#ifdef __cplusplus
}

#endif /* __cplusplus */
#endif /* __PMIC_POWER_H__ */
