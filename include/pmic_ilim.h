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
*   @file    pmic_ilim.h
*
*   @brief   This file contains the default MACRO's and function definitions for
*            PMIC ILIM configuration
*
*/

#ifndef PMIC_INC_ILIM_H_
#define PMIC_INC_ILIM_H_

#include "pmic.h"
#include "pmic_core.h"
#include "pmic_types.h"
#include "pmic_io_priv.h"
#include "pmic_core_priv.h"
#include "pmic_ilim_priv.h"

/*==========================================================================*/
/*                         Structures and Enums                             */
/*==========================================================================*/

typedef struct Pmic_ilimCfgReg_s {
  uint8_t pldo2ILIMCfg; /**< Bit field for PLDO2 ILIM configuration. */
  uint8_t pldo1ILIMCfg; /**< Bit field for PLDO1 ILIM configuration. */
  uint8_t ldo4ILIMCfg;  /**< Bit field for LDO4 ILIM configuration. */
  uint8_t ldo3ILIMCfg;  /**< Bit field for LDO3 ILIM configuration. */
  uint8_t ldo2ILIMCfg;  /**< Bit field for LDO2 ILIM configuration. */
  uint8_t ldo1ILIMCfg;  /**< Bit field for LDO1 ILIM configuration. */
} Pmic_ilimCfgReg_t;

typedef struct Pmic_ilimDglCfgReg_s {
  uint8_t pldo2ILIMdglCfg; /**< Bit field for PLDO2 ILIM DGL configuration. */
  uint8_t pldo1ILIMdglCfg; /**< Bit field for PLDO1 ILIM DGL configuration. */
  uint8_t ldo4ILIMdglCfg;  /**< Bit field for LDO4 ILIM DGL configuration. */
  uint8_t ldo3ILIMdglCfg;  /**< Bit field for LDO3 ILIM DGL configuration. */
  uint8_t ldo2ILIMdglCfg;  /**< Bit field for LDO2 ILIM DGL configuration. */
  uint8_t ldo1ILIMdglCfg;  /**< Bit field for LDO1 ILIM DGL configuration. */
} Pmic_ilimDglCfgReg_t;

typedef struct Pmic_ilimStatReg_s {
  uint8_t bbavgILIMErr; /**< Bit field for BB ILIM ERR configuration. */
  uint8_t pldo2ILIMErr; /**< Bit field for PLDO2 ILIM ERR configuration. */
  uint8_t pldo1ILIMErr; /**< Bit field for PLDO1 ILIM ERR configuration. */
  uint8_t ldo4ILIMErr;  /**< Bit field for LDO4 ILIM ERR configuration. */
  uint8_t ldo3ILIMErr;  /**< Bit field for LDO3 ILIM ERR configuration. */
  uint8_t ldo2ILIMErr;  /**< Bit field for LDO2 ILIM ERR configuration. */
  uint8_t ldo1ILIMErr;  /**< Bit field for LDO1 ILIM ERR configuration. */
} Pmic_ilimStatReg_t;

/* ========================================================================== */
/*                          Function Prototypes                               */
/* ========================================================================== */

static void initializeILIMeRRReg(Pmic_ilimStatReg_t *config);
static void initializeILIMCfgReg(Pmic_ilimCfgReg_t *config);
static void initializeILIMDglCfgReg(Pmic_ilimDglCfgReg_t *config);
int32_t Pmic_SetILIMConfig(Pmic_CoreHandle_t *pPmicCoreHandle,Pmic_ilimCfgReg_t *pPmicILIMConfig);
int32_t Pmic_GetILIMErrStat(Pmic_CoreHandle_t *pPmicCoreHandle, Pmic_ilimStatReg_t  *pPmicILIMStat);
int32_t Pmic_GetILIMDglConfig(Pmic_CoreHandle_t *pPmicCoreHandle, Pmic_ilimDglCfgReg_t  *pPmicILIMdglConfig);
int32_t Pmic_GetILIMConfig(Pmic_CoreHandle_t *pPmicCoreHandle, Pmic_ilimCfgReg_t  *pPmicILIMConfig);
int32_t Pmic_ClearILIMErrStat(Pmic_CoreHandle_t *pPmicCoreHandle, Pmic_ilimStatReg_t  *pPmicILIMStat);
int32_t Pmic_SetILIMDglConfig(Pmic_CoreHandle_t *pPmicCoreHandle, Pmic_ilimDglCfgReg_t  *pPmicILIMdglConfig);

#endif /* PMIC_INC_ILIM_H_ */
