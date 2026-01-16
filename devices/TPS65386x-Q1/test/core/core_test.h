/******************************************************************************
 * Copyright (c) 2025 Texas Instruments Incorporated - http://www.ti.com
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


#ifndef CORE_TEST_H
#define CORE_TEST_H

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "unity.h"
#include "platform.h"
#include "pmic.h"
#include "pmic_core.h"
#include "test_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Function Declarations                            */
/* ========================================================================== */

/**
 * @brief Run all core tests
 * @param args Test arguments (unused)
 */
void core_test(void *args);

/* ========================================================================== */
/* Negative Tests - Scratchpad APIs                                          */
/* ========================================================================== */
extern void test_negative_Pmic_setScratchPadValue_nullParam_handle(void);
extern void test_negative_Pmic_setScratchPadValue_invalidParam_regNum(void);
extern void test_negative_Pmic_getScratchPadValue_nullParam_handle(void);
extern void test_negative_Pmic_getScratchPadValue_nullParam_value(void);
extern void test_negative_Pmic_getScratchPadValue_invalidParam_regNum(void);

/* ========================================================================== */
/* Negative Tests - Lock Control APIs                                        */
/* ========================================================================== */
extern void test_negative_Pmic_setRegLockState_nullParam_handle(void);
extern void test_negative_Pmic_getRegLockState_nullParam_handle(void);
extern void test_negative_Pmic_getRegLockState_nullParam_lockState(void);
extern void test_negative_Pmic_setCntLockState_nullParam_handle(void);
extern void test_negative_Pmic_setCntLockState_invalidParam_lockState(void);
extern void test_negative_Pmic_getCntLockState_nullParam_handle(void);
extern void test_negative_Pmic_getCntLockState_nullParam_lockState(void);
extern void test_negative_Pmic_setLockCfg_nullParam_config(void);
extern void test_negative_Pmic_setLockCfg_invalidParam_validParams(void);
extern void test_negative_Pmic_getLockCfg_nullParam_handle(void);
extern void test_negative_Pmic_getLockCfg_nullParam_config(void);

/* ========================================================================== */
/* Negative Tests - Device Revision APIs                                     */
/* ========================================================================== */
extern void test_negative_Pmic_getNvmRev_nullParam_handle(void);
extern void test_negative_Pmic_getNvmRev_nullParam_nvmRev(void);
extern void test_negative_Pmic_getSiliconRev_nullParam_handle(void);
extern void test_negative_Pmic_getSiliconRev_nullParam_siliconRev(void);

/* ========================================================================== */
/* Negative Tests - Spread Spectrum APIs                                     */
/* ========================================================================== */
extern void test_negative_Pmic_spreadSpectrumEnable_nullParam_handle(void);
extern void test_negative_Pmic_spreadSpectrumEnable_invalidParam_validParams(void);
extern void test_negative_Pmic_getSpreadSpectrumEnable_nullParam_handle(void);
extern void test_negative_Pmic_getSpreadSpectrumEnable_nullParam_config(void);

/* ========================================================================== */
/* Negative Tests - Safe Output APIs                                         */
/* ========================================================================== */
extern void test_negative_Pmic_setEnableSafeOutCfg_nullParam_handle(void);
extern void test_negative_Pmic_getSafeOutPinCfg_nullParam_handle(void);
extern void test_negative_Pmic_getSafeOutPinCfg_nullParam_config(void);

/* ========================================================================== */
/* Negative Tests - Common Status APIs                                       */
/* ========================================================================== */
extern void test_negative_Pmic_getCommonStat_nullParam_handle(void);
extern void test_negative_Pmic_getCommonStat_nullParam_stat(void);

/* ========================================================================== */
/* Negative Tests - Diagnostic Output Control APIs                           */
/* ========================================================================== */
extern void test_negative_Pmic_diagSetOutCtrlCfg_nullParam_handle(void);
extern void test_negative_Pmic_diagGetOutCtrlCfg_nullParam_handle(void);
extern void test_negative_Pmic_diagGetOutCtrlCfg_nullParam_config(void);

/* ========================================================================== */
/* Negative Tests - Diagnostic AMUX APIs                                     */
/* ========================================================================== */
extern void test_negative_Pmic_diagSetAmuxCfg_nullParam_handle(void);
extern void test_negative_Pmic_diagSetAmuxCfg_invalidParam_channel(void);
extern void test_negative_Pmic_diagGetAmuxCfg_nullParam_handle(void);
extern void test_negative_Pmic_diagGetAmuxCfg_nullParam_channel(void);

/* ========================================================================== */
/* Negative Tests - Diagnostic DMUX APIs                                     */
/* ========================================================================== */
extern void test_negative_Pmic_diagSetDmuxCfg_nullParam_handle(void);
extern void test_negative_Pmic_diagSetDmuxCfg_invalidParam_group(void);
extern void test_negative_Pmic_diagGetDmuxCfg_nullParam_handle(void);
extern void test_negative_Pmic_diagGetDmuxCfg_nullParam_group(void);

/* ========================================================================== */
/* Positive Tests                                                             */
/* ========================================================================== */
extern void test_positive_scratchPadSetGet(void);
extern void test_positive_regLockSetGet(void);
extern void test_positive_cntLockSetGet(void);
extern void test_positive_lockCfgSetGet(void);
extern void test_positive_deviceIdRevision(void);
extern void test_positive_spreadSpectrumSetGet(void);
extern void test_positive_safeOutSetGet(void);
extern void test_positive_commonStat(void);
extern void test_positive_diagOutCtrlSetGet(void);
extern void test_positive_diagAMUXSetGet(void);
extern void test_positive_diagDMUXSetGet(void);

#ifdef __cplusplus
}
#endif

#endif /* CORE_TEST_H */
