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
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/* ======================================================================== */
/*                      Test APIs: setScratchPadValue                       */
/* ======================================================================== */

#define CORE_TEST_POS_SETSCRATCHPADVALUE() \
    PLATFORM_RUN_TEST(test_pos_core_scratchPad_setGet)

#define CORE_TEST_NEG_SETSCRATCHPADVALUE() \
    PLATFORM_RUN_TEST(test_neg_core_setScratchPadValue_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_core_setScratchPadValue_invalidRegNum)

/* Test: TC-CORE-0001 */
#define CORE_TEST_SETSCRATCHPADVALUE() \
    CORE_TEST_POS_SETSCRATCHPADVALUE(); \
    CORE_TEST_NEG_SETSCRATCHPADVALUE()

/* ======================================================================== */
/*                      Test APIs: getScratchPadValue                       */
/* ======================================================================== */

#define CORE_TEST_POS_GETSCRATCHPADVALUE() \
    /* Positive tests for getScratchPadValue are combined with setScratchPadValue tests */

#define CORE_TEST_NEG_GETSCRATCHPADVALUE() \
    PLATFORM_RUN_TEST(test_neg_core_getScratchPadValue_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_core_getScratchPadValue_nullValue); \
    PLATFORM_RUN_TEST(test_neg_core_getScratchPadValue_invalidRegNum)

/* Test: TC-CORE-0002 */
#define CORE_TEST_GETSCRATCHPADVALUE() \
    CORE_TEST_POS_GETSCRATCHPADVALUE(); \
    CORE_TEST_NEG_GETSCRATCHPADVALUE()

/* ======================================================================== */
/*                        Test APIs: setRegLockState                        */
/* ======================================================================== */

#define CORE_TEST_POS_SETREGLOCKSTATE() \
    PLATFORM_RUN_TEST(test_pos_core_regLock_setGet)

#define CORE_TEST_NEG_SETREGLOCKSTATE() \
    PLATFORM_RUN_TEST(test_neg_core_setRegLockState_nullHandle)

/* Test: TC-CORE-0003 */
#define CORE_TEST_SETREGLOCKSTATE() \
    CORE_TEST_POS_SETREGLOCKSTATE(); \
    CORE_TEST_NEG_SETREGLOCKSTATE()

/* ======================================================================== */
/*                        Test APIs: getRegLockState                        */
/* ======================================================================== */

#define CORE_TEST_POS_GETREGLOCKSTATE() \
    /* Positive tests for getRegLockState are combined with setRegLockState tests */

#define CORE_TEST_NEG_GETREGLOCKSTATE() \
    PLATFORM_RUN_TEST(test_neg_core_getRegLockState_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_core_getRegLockState_nullLockState)

/* Test: TC-CORE-0004 */
#define CORE_TEST_GETREGLOCKSTATE() \
    CORE_TEST_POS_GETREGLOCKSTATE(); \
    CORE_TEST_NEG_GETREGLOCKSTATE()

/* ======================================================================== */
/*                        Test APIs: setCntLockState                        */
/* ======================================================================== */

#define CORE_TEST_POS_SETCNTLOCKSTATE() \
    PLATFORM_RUN_TEST(test_pos_core_cntLock_setGet)

#define CORE_TEST_NEG_SETCNTLOCKSTATE() \
    PLATFORM_RUN_TEST(test_neg_core_setCntLockState_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_core_setCntLockState_invalidLockState)

/* Test: TC-CORE-0005 */
#define CORE_TEST_SETCNTLOCKSTATE() \
    CORE_TEST_POS_SETCNTLOCKSTATE(); \
    CORE_TEST_NEG_SETCNTLOCKSTATE()

/* ======================================================================== */
/*                        Test APIs: getCntLockState                        */
/* ======================================================================== */

#define CORE_TEST_POS_GETCNTLOCKSTATE() \
    /* Positive tests for getCntLockState are combined with setCntLockState tests */

#define CORE_TEST_NEG_GETCNTLOCKSTATE() \
    PLATFORM_RUN_TEST(test_neg_core_getCntLockState_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_core_getCntLockState_nullLockState)

/* Test: TC-CORE-0006 */
#define CORE_TEST_GETCNTLOCKSTATE() \
    CORE_TEST_POS_GETCNTLOCKSTATE(); \
    CORE_TEST_NEG_GETCNTLOCKSTATE()

/* ======================================================================== */
/*                          Test APIs: setLockCfg                           */
/* ======================================================================== */

#define CORE_TEST_POS_SETLOCKCFG() \
    PLATFORM_RUN_TEST(test_pos_core_lockCfg_setGet); \
    PLATFORM_RUN_TEST(test_pos_core_setLockCfg_lockCntValidOnly); \
    PLATFORM_RUN_TEST(test_pos_core_setLockCfg_lockRegValidOnly)

#define CORE_TEST_NEG_SETLOCKCFG() \
    PLATFORM_RUN_TEST(test_neg_core_setLockCfg_nullConfig); \
    PLATFORM_RUN_TEST(test_neg_core_setLockCfg_invalidValidParams); \
    PLATFORM_RUN_TEST(test_neg_core_setLockCfg_invalidBitsInValidParams)

/* Test: TC-CORE-0007 */
#define CORE_TEST_SETLOCKCFG() \
    CORE_TEST_POS_SETLOCKCFG(); \
    CORE_TEST_NEG_SETLOCKCFG()

/* ======================================================================== */
/*                          Test APIs: getLockCfg                           */
/* ======================================================================== */

#define CORE_TEST_POS_GETLOCKCFG() \
    /* Positive tests for getLockCfg are combined with setLockCfg tests */

#define CORE_TEST_NEG_GETLOCKCFG() \
    PLATFORM_RUN_TEST(test_neg_core_getLockCfg_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_core_getLockCfg_nullConfig)

/* Test: TC-CORE-0008 */
#define CORE_TEST_GETLOCKCFG() \
    CORE_TEST_POS_GETLOCKCFG(); \
    CORE_TEST_NEG_GETLOCKCFG()

/* ======================================================================== */
/*                           Test APIs: getNvmRev                           */
/* ======================================================================== */

#define CORE_TEST_POS_GETNVMREV() \
    PLATFORM_RUN_TEST(test_pos_core_deviceId_revision)

#define CORE_TEST_NEG_GETNVMREV() \
    PLATFORM_RUN_TEST(test_neg_core_getNvmRev_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_core_getNvmRev_nullNvmRev)

/* Test: TC-CORE-0009 */
#define CORE_TEST_GETNVMREV() \
    CORE_TEST_POS_GETNVMREV(); \
    CORE_TEST_NEG_GETNVMREV()

/* ======================================================================== */
/*                         Test APIs: getSiliconRev                         */
/* ======================================================================== */

#define CORE_TEST_POS_GETSILICONREV() \
    /* Positive tests for getSiliconRev are combined with getNvmRev tests */

#define CORE_TEST_NEG_GETSILICONREV() \
    PLATFORM_RUN_TEST(test_neg_core_getSiliconRev_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_core_getSiliconRev_nullSiliconRev)

/* Test: TC-CORE-0010 */
#define CORE_TEST_GETSILICONREV() \
    CORE_TEST_POS_GETSILICONREV(); \
    CORE_TEST_NEG_GETSILICONREV()

/* ======================================================================== */
/*                      Test APIs: setMuxCfg, getMuxCfg                     */
/* ======================================================================== */

#define CORE_TEST_POS_DIAGSETOUTCTRLCFG() \
    PLATFORM_RUN_TEST(test_pos_core_diagOutCtrl_setGet); \
    PLATFORM_RUN_TEST(test_pos_core_setMuxCfg_amuxEnableDisabled); \
    PLATFORM_RUN_TEST(test_pos_core_setMuxCfg_dmuxEnableDisabled); \
    PLATFORM_RUN_TEST(test_pos_core_setMuxCfg_amuxChannelOnly); \
    PLATFORM_RUN_TEST(test_pos_core_setMuxCfg_updateCtrlSuccess)

#define CORE_TEST_NEG_DIAGSETOUTCTRLCFG() \
    PLATFORM_RUN_TEST(test_neg_core_diagSetOutCtrlCfg_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_core_setMuxCfg_nullConfig); \
    PLATFORM_RUN_TEST(test_neg_core_setMuxCfg_readFail); \
    PLATFORM_RUN_TEST(test_neg_core_setMuxCfg_cfgRegReadFail); \
    PLATFORM_RUN_TEST(test_neg_core_setMuxCfg_writeCtrlTxFail)

/* Test: TC-CORE-0012 */
#define CORE_TEST_DIAGSETOUTCTRLCFG() \
    CORE_TEST_POS_DIAGSETOUTCTRLCFG(); \
    CORE_TEST_NEG_DIAGSETOUTCTRLCFG()

/* ======================================================================== */
/*                           Test APIs: getMuxCfg                           */
/* ======================================================================== */

#define CORE_TEST_POS_DIAGGETOUTCTRLCFG() \
    /* Positive tests for diagGetOutCtrlCfg are combined with diagSetOutCtrlCfg tests */ \
    PLATFORM_RUN_TEST(test_pos_core_getMuxCfg_amuxChannelValid)

#define CORE_TEST_NEG_DIAGGETOUTCTRLCFG() \
    PLATFORM_RUN_TEST(test_neg_core_diagGetOutCtrlCfg_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_core_diagGetOutCtrlCfg_nullConfig); \
    PLATFORM_RUN_TEST(test_neg_core_getMuxCfg_readFail); \
    PLATFORM_RUN_TEST(test_neg_core_getMuxCfg_cfgRegReadFail)

/* Test: TC-CORE-0013 */
#define CORE_TEST_DIAGGETOUTCTRLCFG() \
    CORE_TEST_POS_DIAGGETOUTCTRLCFG(); \
    CORE_TEST_NEG_DIAGGETOUTCTRLCFG()

/* ======================================================================== */
/*                       Test APIs: setMuxCfg, getMuxCfg                    */
/* ======================================================================== */

#define CORE_TEST_POS_DIAGSETAMUXCFG() \
    PLATFORM_RUN_TEST(test_pos_core_diagAMUX_setGet)

#define CORE_TEST_NEG_DIAGSETAMUXCFG() \
    PLATFORM_RUN_TEST(test_neg_core_diagSetAmuxCfg_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_core_diagSetAmuxCfg_invalidChannel)

/* Test: TC-CORE-0014 */
#define CORE_TEST_DIAGSETAMUXCFG() \
    CORE_TEST_POS_DIAGSETAMUXCFG(); \
    CORE_TEST_NEG_DIAGSETAMUXCFG()

/* ======================================================================== */
/*                             Test APIs: getMuxCfg                         */
/* ======================================================================== */

#define CORE_TEST_POS_DIAGGETAMUXCFG() \
    /* Positive tests for diagGetAmuxCfg are combined with diagSetAmuxCfg tests */

#define CORE_TEST_NEG_DIAGGETAMUXCFG() \
    PLATFORM_RUN_TEST(test_neg_core_diagGetAmuxCfg_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_core_diagGetAmuxCfg_nullChannel)

/* Test: TC-CORE-0015 */
#define CORE_TEST_DIAGGETAMUXCFG() \
    CORE_TEST_POS_DIAGGETAMUXCFG(); \
    CORE_TEST_NEG_DIAGGETAMUXCFG()

/* ======================================================================== */
/*                           Test APIs: setMuxCfg                           */
/* ======================================================================== */

#define CORE_TEST_POS_DIAGSETDMUXCFG() \
    PLATFORM_RUN_TEST(test_pos_core_diagDMUX_setGet)

#define CORE_TEST_NEG_DIAGSETDMUXCFG() \
    PLATFORM_RUN_TEST(test_neg_core_diagSetDmuxCfg_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_core_diagSetDmuxCfg_invalidGroup)

/* Test: TC-CORE-0016 */
#define CORE_TEST_DIAGSETDMUXCFG() \
    CORE_TEST_POS_DIAGSETDMUXCFG(); \
    CORE_TEST_NEG_DIAGSETDMUXCFG()

/* ======================================================================== */
/*                           Test APIs: getMuxCfg                           */
/* ======================================================================== */

#define CORE_TEST_POS_DIAGGETDMUXCFG() \
    /* Positive tests for diagGetDmuxCfg are combined with diagSetDmuxCfg tests */

#define CORE_TEST_NEG_DIAGGETDMUXCFG() \
    PLATFORM_RUN_TEST(test_neg_core_diagGetDmuxCfg_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_core_diagGetDmuxCfg_nullGroup)

/* Test: TC-CORE-0017 */
#define CORE_TEST_DIAGGETDMUXCFG() \
    CORE_TEST_POS_DIAGGETDMUXCFG(); \
    CORE_TEST_NEG_DIAGGETDMUXCFG()

/* ======================================================================== */
/*                      Test APIs: configCrcEnable                          */
/* ======================================================================== */

#define CORE_TEST_POS_CONFIGCRCENABLE() \
    PLATFORM_RUN_TEST(test_pos_core_configCrcEnable_enableOnly); \
    PLATFORM_RUN_TEST(test_pos_core_configCrcEnable_recalculate)

#define CORE_TEST_NEG_CONFIGCRCENABLE() \
    PLATFORM_RUN_TEST(test_neg_core_configCrcEnable_nullHandle)

/* Test: TC-CORE-0132 */
#define CORE_TEST_CONFIGCRCENABLE() \
    CORE_TEST_POS_CONFIGCRCENABLE(); \
    CORE_TEST_NEG_CONFIGCRCENABLE()

/* ======================================================================== */
/*                      Test APIs: configCrcDisable                         */
/* ======================================================================== */

#define CORE_TEST_POS_CONFIGCRCDISABLE() \
    PLATFORM_RUN_TEST(test_pos_core_configCrcDisable_disable)

#define CORE_TEST_NEG_CONFIGCRCDISABLE() \
    PLATFORM_RUN_TEST(test_neg_core_configCrcDisable_nullHandle)

/* Test: TC-CORE-0133 */
#define CORE_TEST_CONFIGCRCDISABLE() \
    CORE_TEST_POS_CONFIGCRCDISABLE(); \
    CORE_TEST_NEG_CONFIGCRCDISABLE()

/* ======================================================================== */
/*                  Test APIs: getConfigCrcEnableState                      */
/* ======================================================================== */

#define CORE_TEST_POS_GETCONFIGCRCENABLESTATE() \
    PLATFORM_RUN_TEST(test_pos_core_getConfigCrcEnableState_enabled); \
    PLATFORM_RUN_TEST(test_pos_core_getConfigCrcEnableState_disabled)

#define CORE_TEST_NEG_GETCONFIGCRCENABLESTATE() \
    PLATFORM_RUN_TEST(test_neg_core_getConfigCrcEnableState_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_core_getConfigCrcEnableState_nullIsEnabled)

/* Test: TC-CORE-0134 */
#define CORE_TEST_GETCONFIGCRCENABLESTATE() \
    CORE_TEST_POS_GETCONFIGCRCENABLESTATE(); \
    CORE_TEST_NEG_GETCONFIGCRCENABLESTATE()

/* ======================================================================== */
/*                    Test APIs: getConfigCrcStatus                         */
/* ======================================================================== */

#define CORE_TEST_POS_GETCONFIGCRCSTATUS() \
    PLATFORM_RUN_TEST(test_pos_core_getConfigCrcStatus_calcDone); \
    PLATFORM_RUN_TEST(test_pos_core_getConfigCrcStatus_error)

#define CORE_TEST_NEG_GETCONFIGCRCSTATUS() \
    PLATFORM_RUN_TEST(test_neg_core_getConfigCrcStatus_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_core_getConfigCrcStatus_nullStatus); \
    PLATFORM_RUN_TEST(test_neg_core_getConfigCrcStatus_zeroValidParams); \
    PLATFORM_RUN_TEST(test_neg_core_getConfigCrcStatus_firstReadFail); \
    PLATFORM_RUN_TEST(test_neg_core_getConfigCrcStatus_secondReadFail)

/* Test: TC-CORE-0135 */
#define CORE_TEST_GETCONFIGCRCSTATUS() \
    CORE_TEST_POS_GETCONFIGCRCSTATUS(); \
    CORE_TEST_NEG_GETCONFIGCRCSTATUS()

/* ======================================================================== */
/*                    Test APIs: clrConfigCrcStatus                         */
/* ======================================================================== */

#define CORE_TEST_POS_CLRCONFIGCRCSTATUS() \
    PLATFORM_RUN_TEST(test_pos_core_clrConfigCrcStatus_clearCalcDone); \
    PLATFORM_RUN_TEST(test_pos_core_clrConfigCrcStatus_clearError)

#define CORE_TEST_NEG_CLRCONFIGCRCSTATUS() \
    PLATFORM_RUN_TEST(test_neg_core_clrConfigCrcStatus_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_core_clrConfigCrcStatus_nullStatus); \
    PLATFORM_RUN_TEST(test_neg_core_clrConfigCrcStatus_zeroValidParams)

/* Test: TC-CORE-0136 */
#define CORE_TEST_CLRCONFIGCRCSTATUS() \
    CORE_TEST_POS_CLRCONFIGCRCSTATUS(); \
    CORE_TEST_NEG_CLRCONFIGCRCSTATUS()

/* ======================================================================== */
/*                      Test APIs: setConfigCrc                             */
/* ======================================================================== */

#define CORE_TEST_POS_SETCONFIGCRC() \
    PLATFORM_RUN_TEST(test_pos_core_setConfigCrc_writeAndVerify)

#define CORE_TEST_NEG_SETCONFIGCRC() \
    PLATFORM_RUN_TEST(test_neg_core_setConfigCrc_nullHandle)

/* Test: TC-CORE-0137 */
#define CORE_TEST_SETCONFIGCRC() \
    CORE_TEST_POS_SETCONFIGCRC(); \
    CORE_TEST_NEG_SETCONFIGCRC()

/* ======================================================================== */
/*                      Test APIs: getConfigCrc                             */
/* ======================================================================== */

#define CORE_TEST_POS_GETCONFIGCRC() \
    PLATFORM_RUN_TEST(test_pos_core_getConfigCrc_readValue)

#define CORE_TEST_NEG_GETCONFIGCRC() \
    PLATFORM_RUN_TEST(test_neg_core_getConfigCrc_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_core_getConfigCrc_nullValue)

/* Test: TC-CORE-0138 */
#define CORE_TEST_GETCONFIGCRC() \
    CORE_TEST_POS_GETCONFIGCRC(); \
    CORE_TEST_NEG_GETCONFIGCRC()

/* ======================================================================== */
/*                    Test APIs: configCrcCalculate                         */
/* ======================================================================== */

#define CORE_TEST_POS_CONFIGCRCCALCULATE() \
    PLATFORM_RUN_TEST(test_pos_core_configCrcCalculate_calculate)

#define CORE_TEST_NEG_CONFIGCRCCALCULATE() \
    PLATFORM_RUN_TEST(test_neg_core_configCrcCalculate_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_core_calculateCrc_midLoopReadFail); \
    PLATFORM_RUN_TEST(test_neg_core_configCrcValidate_safetyCtrlReadFail); \
    PLATFORM_RUN_TEST(test_neg_core_configCrcValidate_crcAlreadyEnabled); \
    PLATFORM_RUN_TEST(test_neg_core_configCrcValidate_calcBitAlreadySet); \
    PLATFORM_RUN_TEST(test_neg_core_configCrcValidate_crcErrorDetected); \
    PLATFORM_RUN_TEST(test_neg_core_configCrcValidate_calcAssertWriteFail); \
    PLATFORM_RUN_TEST(test_neg_core_configCrcValidate_calcAssertGateSkippedOnCleanEdgeFail)

/* Test: TC-CORE-0067 */
#define CORE_TEST_CONFIGCRCCALCULATE() \
    CORE_TEST_POS_CONFIGCRCCALCULATE(); \
    CORE_TEST_NEG_CONFIGCRCCALCULATE()

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
    CORE_TEST_POS_DIAGSETOUTCTRLCFG(); \
    CORE_TEST_POS_DIAGGETOUTCTRLCFG(); \
    CORE_TEST_POS_DIAGSETAMUXCFG(); \
    CORE_TEST_POS_DIAGGETAMUXCFG(); \
    CORE_TEST_POS_DIAGSETDMUXCFG(); \
    CORE_TEST_POS_DIAGGETDMUXCFG(); \
    CORE_TEST_POS_CONFIGCRCENABLE(); \
    CORE_TEST_POS_CONFIGCRCDISABLE(); \
    CORE_TEST_POS_GETCONFIGCRCENABLESTATE(); \
    CORE_TEST_POS_GETCONFIGCRCSTATUS(); \
    CORE_TEST_POS_CLRCONFIGCRCSTATUS(); \
    CORE_TEST_POS_SETCONFIGCRC(); \
    CORE_TEST_POS_GETCONFIGCRC(); \
    CORE_TEST_POS_CONFIGCRCCALCULATE()

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
    CORE_TEST_NEG_DIAGSETOUTCTRLCFG(); \
    CORE_TEST_NEG_DIAGGETOUTCTRLCFG(); \
    CORE_TEST_NEG_DIAGSETAMUXCFG(); \
    CORE_TEST_NEG_DIAGGETAMUXCFG(); \
    CORE_TEST_NEG_DIAGSETDMUXCFG(); \
    CORE_TEST_NEG_DIAGGETDMUXCFG(); \
    CORE_TEST_NEG_CONFIGCRCENABLE(); \
    CORE_TEST_NEG_CONFIGCRCDISABLE(); \
    CORE_TEST_NEG_GETCONFIGCRCENABLESTATE(); \
    CORE_TEST_NEG_GETCONFIGCRCSTATUS(); \
    CORE_TEST_NEG_CLRCONFIGCRCSTATUS(); \
    CORE_TEST_NEG_SETCONFIGCRC(); \
    CORE_TEST_NEG_GETCONFIGCRC(); \
    CORE_TEST_NEG_CONFIGCRCCALCULATE()

#define CORE_TEST_RUN_ALL() \
    CORE_TEST_RUN_POSITIVE(); \
    CORE_TEST_RUN_NEGATIVE()

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
void test_neg_core_setLockCfg_invalidBitsInValidParams(void);

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
/*                Negative Tests - diagSetOutCtrlCfg                          */
/* ========================================================================== */
void test_neg_core_diagSetOutCtrlCfg_nullHandle(void);

/* ========================================================================== */
/*                Negative Tests - diagGetOutCtrlCfg                          */
/* ========================================================================== */
void test_neg_core_diagGetOutCtrlCfg_nullHandle(void);
void test_neg_core_diagGetOutCtrlCfg_nullConfig(void);
void test_neg_core_setMuxCfg_nullConfig(void);

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
void test_pos_core_setLockCfg_lockCntValidOnly(void);
void test_pos_core_setLockCfg_lockRegValidOnly(void);

/* ========================================================================== */
/*                Positive Tests - Device ID & Revision                       */
/* ========================================================================== */
void test_pos_core_deviceId_revision(void);

/* ========================================================================== */
/*                Positive Tests - Diagnostic Output Control                  */
/* ========================================================================== */
void test_pos_core_diagOutCtrl_setGet(void);
void test_pos_core_getMuxCfg_amuxChannelValid(void);
void test_pos_core_setMuxCfg_amuxEnableDisabled(void);
void test_pos_core_setMuxCfg_dmuxEnableDisabled(void);
void test_pos_core_setMuxCfg_amuxChannelOnly(void);
void test_pos_core_setMuxCfg_updateCtrlSuccess(void);

/* ========================================================================== */
/*                   Negative Tests - getMuxCfg / setMuxCfg                   */
/* ========================================================================== */
void test_neg_core_getMuxCfg_readFail(void);
void test_neg_core_getMuxCfg_cfgRegReadFail(void);
void test_neg_core_setMuxCfg_readFail(void);
void test_neg_core_setMuxCfg_cfgRegReadFail(void);
void test_neg_core_setMuxCfg_writeCtrlTxFail(void);

/* ========================================================================== */
/*                Positive Tests - Diagnostic AMUX                            */
/* ========================================================================== */
void test_pos_core_diagAMUX_setGet(void);

/* ========================================================================== */
/*                Positive Tests - Diagnostic DMUX                            */
/* ========================================================================== */
void test_pos_core_diagDMUX_setGet(void);

/* ========================================================================== */
/*                configCrcEnable API Tests                                   */
/* ========================================================================== */
void test_pos_core_configCrcEnable_enableOnly(void);
void test_pos_core_configCrcEnable_recalculate(void);
void test_neg_core_configCrcEnable_nullHandle(void);

/* ========================================================================== */
/*                configCrcDisable API Tests                                  */
/* ========================================================================== */
void test_pos_core_configCrcDisable_disable(void);
void test_neg_core_configCrcDisable_nullHandle(void);

/* ========================================================================== */
/*                getConfigCrcEnableState API Tests                           */
/* ========================================================================== */
void test_pos_core_getConfigCrcEnableState_enabled(void);
void test_pos_core_getConfigCrcEnableState_disabled(void);
void test_neg_core_getConfigCrcEnableState_nullHandle(void);
void test_neg_core_getConfigCrcEnableState_nullIsEnabled(void);

/* ========================================================================== */
/*                getConfigCrcStatus API Tests                                */
/* ========================================================================== */
void test_pos_core_getConfigCrcStatus_calcDone(void);
void test_pos_core_getConfigCrcStatus_error(void);
void test_neg_core_getConfigCrcStatus_nullHandle(void);
void test_neg_core_getConfigCrcStatus_nullStatus(void);
void test_neg_core_getConfigCrcStatus_zeroValidParams(void);
void test_neg_core_getConfigCrcStatus_firstReadFail(void);
void test_neg_core_getConfigCrcStatus_secondReadFail(void);

/* ========================================================================== */
/*                clrConfigCrcStatus API Tests                                */
/* ========================================================================== */
void test_pos_core_clrConfigCrcStatus_clearCalcDone(void);
void test_pos_core_clrConfigCrcStatus_clearError(void);
void test_neg_core_clrConfigCrcStatus_nullHandle(void);
void test_neg_core_clrConfigCrcStatus_nullStatus(void);
void test_neg_core_clrConfigCrcStatus_zeroValidParams(void);

/* ========================================================================== */
/*                setConfigCrc API Tests                                      */
/* ========================================================================== */
void test_pos_core_setConfigCrc_writeAndVerify(void);
void test_neg_core_setConfigCrc_nullHandle(void);

/* ========================================================================== */
/*                getConfigCrc API Tests                                      */
/* ========================================================================== */
void test_pos_core_getConfigCrc_readValue(void);
void test_neg_core_getConfigCrc_nullHandle(void);
void test_neg_core_getConfigCrc_nullValue(void);

/* ========================================================================== */
/*                configCrcCalculate API Tests                                */
/* ========================================================================== */
void test_pos_core_configCrcCalculate_calculate(void);
void test_neg_core_configCrcCalculate_nullHandle(void);
void test_neg_core_calculateCrc_midLoopReadFail(void);
void test_neg_core_configCrcValidate_safetyCtrlReadFail(void);
void test_neg_core_configCrcValidate_crcAlreadyEnabled(void);
void test_neg_core_configCrcValidate_calcBitAlreadySet(void);
void test_neg_core_configCrcValidate_crcErrorDetected(void);
void test_neg_core_configCrcValidate_calcAssertWriteFail(void);
void test_neg_core_configCrcValidate_calcAssertGateSkippedOnCleanEdgeFail(void);

#ifdef __cplusplus
}
#endif

#endif /* CORE_TEST_H */
