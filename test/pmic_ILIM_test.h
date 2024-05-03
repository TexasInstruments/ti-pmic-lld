/****************************************************************************
 *  Copyright (C) 2024 Texas Instruments Incorporated
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
 ****************************************************************************/

/**
 *  @file  pmic_ilim_test.h
 *
 *  @brief  This file contains all the testing related files APIs for the ILIM.
 */
#ifndef __PMIC_ILIM_TEST_H__
#define __PMIC_ILIM_TEST_H__

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include "pmic_test_common.h"
#include "pmic_ilim.h"
#include "regmap/ilim.h"


/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/*==========================================================================*/
/*                         Structures and Enums                             */
/*==========================================================================*/

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

int32_t Pmic_SetILIMConfig(Pmic_CoreHandle_t *pPmicCoreHandle, Pmic_ilimCfgReg_t  *pPmicILIMConfig);
int32_t Pmic_GetILIMConfig(Pmic_CoreHandle_t *pPmicCoreHandle, Pmic_ilimCfgReg_t  *pPmicILIMConfig);

int32_t Pmic_SetILIMDglConfig(Pmic_CoreHandle_t *pPmicCoreHandle, Pmic_ilimDglCfgReg_t  *pPmicILIMDglConfig);
int32_t Pmic_GetILIMDglConfig(Pmic_CoreHandle_t *pPmicCoreHandle, Pmic_ilimDglCfgReg_t  *pPmicILIMDglConfig);


int32_t Pmic_ClearILIMErrStat(Pmic_CoreHandle_t *pPmicCoreHandle, Pmic_ilimStatReg_t  *pPmicILIMStat);
int32_t Pmic_GetILIMErrStat(Pmic_CoreHandle_t *pPmicCoreHandle, Pmic_ilimStatReg_t  *pPmicILIMStat);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __PMIC_LOWIQTIMER_TEST_H__ */
