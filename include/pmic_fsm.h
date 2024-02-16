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
 *   @file    pmic_fsm.h
 *
 *   @brief   This file contains the default MACRO's and function definitions
 * for PMIC FSM state configuration
 *
 */

#ifndef PMIC_FSM_H_
#define PMIC_FSM_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "pmic_core.h"
#include "pmic_core_priv.h"
#include "pmic_io_priv.h"
#include "pmic_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/*==========================================================================*/
/*                         Structures and Enums                             */
/*==========================================================================*/

typedef struct Pmic_FsmCfg_s {
  uint8_t validParams;
  bool fastBistEn;
  bool lpStandbySel;
  bool ilimIntfsmCtrlEn;
  uint8_t fsmStarupDestSel;
} Pmic_FsmCfg_t;

/*==========================================================================*/
/*                         Function Declarations                            */
/*==========================================================================*/

int32_t Pmic_setS2RState(Pmic_CoreHandle_t *pPmicCoreHandle);

void Pmic_fsmGetstandByCfgRegFields(uint8_t pmicNextState, uint8_t *pRegAddr,
                                    uint8_t *pBitPos, uint8_t *pBitMask,
                                    uint8_t *pBitVal, uint8_t *pDeviceState);

void Pmic_fsmGetNsleepMaskBitField(bool nsleepType, uint8_t *pBitPos,
                                   uint8_t *pBitMask);

int32_t Pmic_fsmGetDeviceStateCfg(Pmic_CoreHandle_t *pPmicCoreHandle);

int32_t Pmic_fsmDeviceRequestCfg(Pmic_CoreHandle_t *pPmicCoreHandle,
                                 uint8_t fsmState);

int32_t Pmic_setState(Pmic_CoreHandle_t *pPmicCoreHandle,
                      uint8_t pmicNextState);

int32_t Pmic_fsmSetMissionState(Pmic_CoreHandle_t *pPmicCoreHandle,
                                const uint8_t pmicState);

int32_t Pmic_fsmSetConfiguration(Pmic_CoreHandle_t *pPmicCoreHandle,
                                 const Pmic_FsmCfg_t fsmCfg);

int32_t Pmic_fsmGetConfiguration(Pmic_CoreHandle_t *pPmicCoreHandle,
                                 Pmic_FsmCfg_t *pFsmCfg);

int32_t Pmic_fsmRequestRuntimeBist(Pmic_CoreHandle_t *pPmicCoreHandle);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* PMIC_FSM_H_ */
