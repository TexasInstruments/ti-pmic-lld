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
/*        API-Specific Test Macros - setScratchPadValue/getScratchPadValue   */
/* ========================================================================== */

#define CORE_TEST_POS_SETSCRATCHPADVALUE() \
    PLATFORM_RUN_TEST(test_pos_core_scratchPad_setGet)

#define CORE_TEST_NEG_SETSCRATCHPADVALUE() \
    PLATFORM_RUN_TEST(test_neg_core_setScratchPadValue_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_core_setScratchPadValue_invalidRegNum)

#define CORE_TEST_SETSCRATCHPADVALUE() \
    CORE_TEST_POS_SETSCRATCHPADVALUE(); \
    CORE_TEST_NEG_SETSCRATCHPADVALUE()

#define CORE_TEST_POS_GETSCRATCHPADVALUE() \
    /* Positive tests for getScratchPadValue are combined with setScratchPadValue tests */

#define CORE_TEST_NEG_GETSCRATCHPADVALUE() \
    PLATFORM_RUN_TEST(test_neg_core_getScratchPadValue_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_core_getScratchPadValue_nullValue); \
    PLATFORM_RUN_TEST(test_neg_core_getScratchPadValue_invalidRegNum)

#define CORE_TEST_GETSCRATCHPADVALUE() \
    CORE_TEST_POS_GETSCRATCHPADVALUE(); \
    CORE_TEST_NEG_GETSCRATCHPADVALUE()

/* ========================================================================== */
/*          API-Specific Test Macros - setRegLockState/getRegLockState       */
/* ========================================================================== */

#define CORE_TEST_POS_SETREGLOCKSTATE() \
    PLATFORM_RUN_TEST(test_pos_core_regLock_setGet)

#define CORE_TEST_NEG_SETREGLOCKSTATE() \
    PLATFORM_RUN_TEST(test_neg_core_setRegLockState_nullHandle)

#define CORE_TEST_SETREGLOCKSTATE() \
    CORE_TEST_POS_SETREGLOCKSTATE(); \
    CORE_TEST_NEG_SETREGLOCKSTATE()

#define CORE_TEST_POS_GETREGLOCKSTATE() \
    /* Positive tests for getRegLockState are combined with setRegLockState tests */

#define CORE_TEST_NEG_GETREGLOCKSTATE() \
    PLATFORM_RUN_TEST(test_neg_core_getRegLockState_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_core_getRegLockState_nullLockState)

#define CORE_TEST_GETREGLOCKSTATE() \
    CORE_TEST_POS_GETREGLOCKSTATE(); \
    CORE_TEST_NEG_GETREGLOCKSTATE()

/* ========================================================================== */
/*          API-Specific Test Macros - setCntLockState/getCntLockState       */
/* ========================================================================== */

#define CORE_TEST_POS_SETCNTLOCKSTATE() \
    PLATFORM_RUN_TEST(test_pos_core_cntLock_setGet)

#define CORE_TEST_NEG_SETCNTLOCKSTATE() \
    PLATFORM_RUN_TEST(test_neg_core_setCntLockState_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_core_setCntLockState_invalidLockState)

#define CORE_TEST_SETCNTLOCKSTATE() \
    CORE_TEST_POS_SETCNTLOCKSTATE(); \
    CORE_TEST_NEG_SETCNTLOCKSTATE()

#define CORE_TEST_POS_GETCNTLOCKSTATE() \
    /* Positive tests for getCntLockState are combined with setCntLockState tests */

#define CORE_TEST_NEG_GETCNTLOCKSTATE() \
    PLATFORM_RUN_TEST(test_neg_core_getCntLockState_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_core_getCntLockState_nullLockState)

#define CORE_TEST_GETCNTLOCKSTATE() \
    CORE_TEST_POS_GETCNTLOCKSTATE(); \
    CORE_TEST_NEG_GETCNTLOCKSTATE()

/* ========================================================================== */
/*               API-Specific Test Macros - setLockCfg/getLockCfg            */
/* ========================================================================== */

#define CORE_TEST_POS_SETLOCKCFG() \
    PLATFORM_RUN_TEST(test_pos_core_lockCfg_setGet)

#define CORE_TEST_NEG_SETLOCKCFG() \
    PLATFORM_RUN_TEST(test_neg_core_setLockCfg_nullConfig); \
    PLATFORM_RUN_TEST(test_neg_core_setLockCfg_invalidValidParams)

#define CORE_TEST_SETLOCKCFG() \
    CORE_TEST_POS_SETLOCKCFG(); \
    CORE_TEST_NEG_SETLOCKCFG()

#define CORE_TEST_POS_GETLOCKCFG() \
    /* Positive tests for getLockCfg are combined with setLockCfg tests */

#define CORE_TEST_NEG_GETLOCKCFG() \
    PLATFORM_RUN_TEST(test_neg_core_getLockCfg_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_core_getLockCfg_nullConfig)

#define CORE_TEST_GETLOCKCFG() \
    CORE_TEST_POS_GETLOCKCFG(); \
    CORE_TEST_NEG_GETLOCKCFG()

/* ========================================================================== */
/*                 API-Specific Test Macros - getNvmRev                       */
/* ========================================================================== */

#define CORE_TEST_POS_GETNVMREV() \
    PLATFORM_RUN_TEST(test_pos_core_deviceId_revision)

#define CORE_TEST_NEG_GETNVMREV() \
    PLATFORM_RUN_TEST(test_neg_core_getNvmRev_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_core_getNvmRev_nullNvmRev)

#define CORE_TEST_GETNVMREV() \
    CORE_TEST_POS_GETNVMREV(); \
    CORE_TEST_NEG_GETNVMREV()

/* ========================================================================== */
/*                 API-Specific Test Macros - getSiliconRev                   */
/* ========================================================================== */

#define CORE_TEST_POS_GETSILICONREV() \
    /* Positive tests for getSiliconRev are combined with getNvmRev tests */

#define CORE_TEST_NEG_GETSILICONREV() \
    PLATFORM_RUN_TEST(test_neg_core_getSiliconRev_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_core_getSiliconRev_nullSiliconRev)

#define CORE_TEST_GETSILICONREV() \
    CORE_TEST_POS_GETSILICONREV(); \
    CORE_TEST_NEG_GETSILICONREV()

/* ========================================================================== */
/*                 API-Specific Test Macros - getCommonStat                   */
/* ========================================================================== */

#define CORE_TEST_POS_GETCOMMONSTAT() \
    PLATFORM_RUN_TEST(test_pos_core_commonStat_get)

#define CORE_TEST_NEG_GETCOMMONSTAT() \
    PLATFORM_RUN_TEST(test_neg_core_getCommonStat_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_core_getCommonStat_nullStat)

#define CORE_TEST_GETCOMMONSTAT() \
    CORE_TEST_POS_GETCOMMONSTAT(); \
    CORE_TEST_NEG_GETCOMMONSTAT()

/* ========================================================================== */
/*         API-Specific Test Macros - diagSetOutCtrlCfg/diagGetOutCtrlCfg    */
/* ========================================================================== */

#define CORE_TEST_POS_DIAGSETOUTCTRLCFG() \
    PLATFORM_RUN_TEST(test_pos_core_diagOutCtrl_setGet)

#define CORE_TEST_NEG_DIAGSETOUTCTRLCFG() \
    PLATFORM_RUN_TEST(test_neg_core_diagSetOutCtrlCfg_nullHandle)

#define CORE_TEST_DIAGSETOUTCTRLCFG() \
    CORE_TEST_POS_DIAGSETOUTCTRLCFG(); \
    CORE_TEST_NEG_DIAGSETOUTCTRLCFG()

#define CORE_TEST_POS_DIAGGETOUTCTRLCFG() \
    /* Positive tests for diagGetOutCtrlCfg are combined with diagSetOutCtrlCfg tests */

#define CORE_TEST_NEG_DIAGGETOUTCTRLCFG() \
    PLATFORM_RUN_TEST(test_neg_core_diagGetOutCtrlCfg_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_core_diagGetOutCtrlCfg_nullConfig)

#define CORE_TEST_DIAGGETOUTCTRLCFG() \
    CORE_TEST_POS_DIAGGETOUTCTRLCFG(); \
    CORE_TEST_NEG_DIAGGETOUTCTRLCFG()

/* ========================================================================== */
/*            API-Specific Test Macros - diagSetAmuxCfg/diagGetAmuxCfg       */
/* ========================================================================== */

#define CORE_TEST_POS_DIAGSETAMUXCFG() \
    PLATFORM_RUN_TEST(test_pos_core_diagAMUX_setGet)

#define CORE_TEST_NEG_DIAGSETAMUXCFG() \
    PLATFORM_RUN_TEST(test_neg_core_diagSetAmuxCfg_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_core_diagSetAmuxCfg_invalidChannel)

#define CORE_TEST_DIAGSETAMUXCFG() \
    CORE_TEST_POS_DIAGSETAMUXCFG(); \
    CORE_TEST_NEG_DIAGSETAMUXCFG()

#define CORE_TEST_POS_DIAGGETAMUXCFG() \
    /* Positive tests for diagGetAmuxCfg are combined with diagSetAmuxCfg tests */

#define CORE_TEST_NEG_DIAGGETAMUXCFG() \
    PLATFORM_RUN_TEST(test_neg_core_diagGetAmuxCfg_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_core_diagGetAmuxCfg_nullChannel)

#define CORE_TEST_DIAGGETAMUXCFG() \
    CORE_TEST_POS_DIAGGETAMUXCFG(); \
    CORE_TEST_NEG_DIAGGETAMUXCFG()

/* ========================================================================== */
/*            API-Specific Test Macros - diagSetDmuxCfg/diagGetDmuxCfg       */
/* ========================================================================== */

#define CORE_TEST_POS_DIAGSETDMUXCFG() \
    PLATFORM_RUN_TEST(test_pos_core_diagDMUX_setGet)

#define CORE_TEST_NEG_DIAGSETDMUXCFG() \
    PLATFORM_RUN_TEST(test_neg_core_diagSetDmuxCfg_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_core_diagSetDmuxCfg_invalidGroup)

#define CORE_TEST_DIAGSETDMUXCFG() \
    CORE_TEST_POS_DIAGSETDMUXCFG(); \
    CORE_TEST_NEG_DIAGSETDMUXCFG()

#define CORE_TEST_POS_DIAGGETDMUXCFG() \
    /* Positive tests for diagGetDmuxCfg are combined with diagSetDmuxCfg tests */

#define CORE_TEST_NEG_DIAGGETDMUXCFG() \
    PLATFORM_RUN_TEST(test_neg_core_diagGetDmuxCfg_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_core_diagGetDmuxCfg_nullGroup)

#define CORE_TEST_DIAGGETDMUXCFG() \
    CORE_TEST_POS_DIAGGETDMUXCFG(); \
    CORE_TEST_NEG_DIAGGETDMUXCFG()

/* ========================================================================== */
/*                         Aggregate Test Runners                             */
/* ========================================================================== */

#define CORE_TEST_RUN_POSITIVE() \
    CORE_TEST_POS_SETSCRATCHPADVALUE(); \
    CORE_TEST_POS_GETSCRATCHPADVALUE(); \
    CORE_TEST_POS_SETREGLOCKSTATE(); \
    CORE_TEST_POS_GETREGLOCKSTATE(); \
    CORE_TEST_POS_SETCNTLOCKSTATE(); \
    CORE_TEST_POS_GETCNTLOCKSTATE(); \
    CORE_TEST_POS_SETLOCKCFG(); \
    CORE_TEST_POS_GETLOCKCFG(); \
    CORE_TEST_POS_GETNVMREV(); \
    CORE_TEST_POS_GETSILICONREV(); \
    CORE_TEST_POS_GETCOMMONSTAT(); \
    CORE_TEST_POS_DIAGSETOUTCTRLCFG(); \
    CORE_TEST_POS_DIAGGETOUTCTRLCFG(); \
    CORE_TEST_POS_DIAGSETAMUXCFG(); \
    CORE_TEST_POS_DIAGGETAMUXCFG(); \
    CORE_TEST_POS_DIAGSETDMUXCFG(); \
    CORE_TEST_POS_DIAGGETDMUXCFG()

#define CORE_TEST_RUN_NEGATIVE() \
    CORE_TEST_NEG_SETSCRATCHPADVALUE(); \
    CORE_TEST_NEG_GETSCRATCHPADVALUE(); \
    CORE_TEST_NEG_SETREGLOCKSTATE(); \
    CORE_TEST_NEG_GETREGLOCKSTATE(); \
    CORE_TEST_NEG_SETCNTLOCKSTATE(); \
    CORE_TEST_NEG_GETCNTLOCKSTATE(); \
    CORE_TEST_NEG_SETLOCKCFG(); \
    CORE_TEST_NEG_GETLOCKCFG(); \
    CORE_TEST_NEG_GETNVMREV(); \
    CORE_TEST_NEG_GETSILICONREV(); \
    CORE_TEST_NEG_GETCOMMONSTAT(); \
    CORE_TEST_NEG_DIAGSETOUTCTRLCFG(); \
    CORE_TEST_NEG_DIAGGETOUTCTRLCFG(); \
    CORE_TEST_NEG_DIAGSETAMUXCFG(); \
    CORE_TEST_NEG_DIAGGETAMUXCFG(); \
    CORE_TEST_NEG_DIAGSETDMUXCFG(); \
    CORE_TEST_NEG_DIAGGETDMUXCFG()

#define CORE_TEST_RUN_ALL() \
    CORE_TEST_RUN_POSITIVE(); \
    CORE_TEST_RUN_NEGATIVE()

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
