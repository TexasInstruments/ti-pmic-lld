/******************************************************************************
 * Copyright (c) 2026 Texas Instruments Incorporated - http://www.ti.com
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
#include "test_utils.h"

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
/*                Negative Tests - setScratchPadValue                         */
/* ========================================================================== */
void test_neg_core_setScratchPadValue_nullHandle(void);
void test_neg_core_setScratchPadValue_invalidRegNum(void);

/* ========================================================================== */
/*                Negative Tests - getScratchPadValue                         */
/* ========================================================================== */
void test_neg_core_getScratchPadValue_nullHandle(void);
void test_neg_core_getScratchPadValue_nullValue(void);
void test_neg_core_getScratchPadValue_invalidRegNum(void);

/* ========================================================================== */
/*                Negative Tests - setRegLockState                            */
/* ========================================================================== */
void test_neg_core_setRegLockState_nullHandle(void);

/* ========================================================================== */
/*                Negative Tests - getRegLockState                            */
/* ========================================================================== */
void test_neg_core_getRegLockState_nullHandle(void);
void test_neg_core_getRegLockState_nullLockState(void);

/* ========================================================================== */
/*                Negative Tests - setCntLockState                            */
/* ========================================================================== */
void test_neg_core_setCntLockState_nullHandle(void);
void test_neg_core_setCntLockState_invalidLockState(void);

/* ========================================================================== */
/*                Negative Tests - getCntLockState                            */
/* ========================================================================== */
void test_neg_core_getCntLockState_nullHandle(void);
void test_neg_core_getCntLockState_nullLockState(void);

/* ========================================================================== */
/*                Negative Tests - setLockCfg                                 */
/* ========================================================================== */
void test_neg_core_setLockCfg_nullConfig(void);
void test_neg_core_setLockCfg_invalidValidParams(void);

/* ========================================================================== */
/*                Negative Tests - getLockCfg                                 */
/* ========================================================================== */
void test_neg_core_getLockCfg_nullHandle(void);
void test_neg_core_getLockCfg_nullConfig(void);

/* ========================================================================== */
/*                Negative Tests - getNvmRev                                  */
/* ========================================================================== */
void test_neg_core_getNvmRev_nullHandle(void);
void test_neg_core_getNvmRev_nullNvmRev(void);

/* ========================================================================== */
/*                Negative Tests - getSiliconRev                              */
/* ========================================================================== */
void test_neg_core_getSiliconRev_nullHandle(void);
void test_neg_core_getSiliconRev_nullSiliconRev(void);

/* ========================================================================== */
/*                Negative Tests - getCommonStat                              */
/* ========================================================================== */
void test_neg_core_getCommonStat_nullHandle(void);
void test_neg_core_getCommonStat_nullStat(void);

/* ========================================================================== */
/*                Negative Tests - diagSetOutCtrlCfg                          */
/* ========================================================================== */
void test_neg_core_diagSetOutCtrlCfg_nullHandle(void);

/* ========================================================================== */
/*                Negative Tests - diagGetOutCtrlCfg                          */
/* ========================================================================== */
void test_neg_core_diagGetOutCtrlCfg_nullHandle(void);
void test_neg_core_diagGetOutCtrlCfg_nullConfig(void);

/* ========================================================================== */
/*                Negative Tests - diagSetAmuxCfg                             */
/* ========================================================================== */
void test_neg_core_diagSetAmuxCfg_nullHandle(void);
void test_neg_core_diagSetAmuxCfg_invalidChannel(void);

/* ========================================================================== */
/*                Negative Tests - diagGetAmuxCfg                             */
/* ========================================================================== */
void test_neg_core_diagGetAmuxCfg_nullHandle(void);
void test_neg_core_diagGetAmuxCfg_nullChannel(void);

/* ========================================================================== */
/*                Negative Tests - diagSetDmuxCfg                             */
/* ========================================================================== */
void test_neg_core_diagSetDmuxCfg_nullHandle(void);
void test_neg_core_diagSetDmuxCfg_invalidGroup(void);

/* ========================================================================== */
/*                Negative Tests - diagGetDmuxCfg                             */
/* ========================================================================== */
void test_neg_core_diagGetDmuxCfg_nullHandle(void);
void test_neg_core_diagGetDmuxCfg_nullGroup(void);

/* ========================================================================== */
/*                Positive Tests - Scratchpad                                 */
/* ========================================================================== */
void test_pos_core_scratchPad_setGet(void);

/* ========================================================================== */
/*                Positive Tests - Register Lock                              */
/* ========================================================================== */
void test_pos_core_regLock_setGet(void);

/* ========================================================================== */
/*                Positive Tests - Counter Lock                               */
/* ========================================================================== */
void test_pos_core_cntLock_setGet(void);

/* ========================================================================== */
/*                Positive Tests - Lock Configuration                         */
/* ========================================================================== */
void test_pos_core_lockCfg_setGet(void);

/* ========================================================================== */
/*                Positive Tests - Device ID & Revision                       */
/* ========================================================================== */
void test_pos_core_deviceId_revision(void);

/* ========================================================================== */
/*                Positive Tests - Common Status                              */
/* ========================================================================== */
void test_pos_core_commonStat_get(void);

/* ========================================================================== */
/*                Positive Tests - Diagnostic Output Control                  */
/* ========================================================================== */
void test_pos_core_diagOutCtrl_setGet(void);

/* ========================================================================== */
/*                Positive Tests - Diagnostic AMUX                            */
/* ========================================================================== */
void test_pos_core_diagAMUX_setGet(void);

/* ========================================================================== */
/*                Positive Tests - Diagnostic DMUX                            */
/* ========================================================================== */
void test_pos_core_diagDMUX_setGet(void);

#ifdef __cplusplus
}
#endif

#endif /* CORE_TEST_H */
