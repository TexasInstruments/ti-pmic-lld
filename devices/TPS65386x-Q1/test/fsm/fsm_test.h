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
#ifndef FSM_TEST_H
#define FSM_TEST_H



/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include "../platform.h"
#include "pmic_fsm.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/**
 * @brief Convenience macro for common FSM configuration parameters
 */
#define PMIC_FSM_CFG_VALID \
    (PMIC_CFG_STBY_EN_VALID | \
     PMIC_CFG_AUTO_BIST_EN_VALID | \
     PMIC_CFG_NRST_ACTIVE_IN_STBY_SEQ_VALID | \
     PMIC_CFG_PWD_THR_VALID | \
     PMIC_CFG_NRST_EXT_VALID | \
     PMIC_CFG_RST_MCU_TMO_VALID | \
     PMIC_CFG_SAFE_TMO_VALID | \
     PMIC_CFG_SAFE_LOCK_THR_VALID | \
     PMIC_CFG_VBAT_STBY_ENTRY_THR_VALID)

/**
 * @brief Convenience macros for FSM wakeup/latch valid params (aliases for non-_SHIFT versions)
 */
#define PMIC_CFG_WAKE1_EVENT_VALID_SHIFT PMIC_CFG_WAKE1_EVENT_VALID
#define PMIC_CFG_WAKE2_EVENT_VALID_SHIFT PMIC_CFG_WAKE2_EVENT_VALID
#define PMIC_CFG_WAKE1_DGL_VALID_SHIFT PMIC_CFG_WAKE1_DGL_VALID
#define PMIC_CFG_WAKE2_DGL_VALID_SHIFT PMIC_CFG_WAKE2_DGL_VALID
#define PMIC_CFG_WAKE1_EVENT_PWRL_EN_VALID_SHIFT PMIC_CFG_WAKE1_EVENT_PWRL_EN_VALID
#define PMIC_CFG_WAKE2_EVENT_PWRL_EN_VALID_SHIFT PMIC_CFG_WAKE2_EVENT_PWRL_EN_VALID
#define PMIC_CFG_WAKE1_LATCH_VALID_SHIFT PMIC_CFG_WAKE1_LATCH_VALID
#define PMIC_CFG_WAKE2_LATCH_VALID_SHIFT PMIC_CFG_WAKE2_LATCH_VALID

/* ======================================================================== */
/*                        Test APIs: fsmSetDevState                         */
/* ======================================================================== */

#define FSM_TEST_POS_FSMSETDEVSTATE() \
    PLATFORM_RUN_TEST(test_pos_fsm_fsmSetDevState_validStates)

#define FSM_TEST_NEG_FSMSETDEVSTATE() \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmSetDevState_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmSetDevState_invalidState); \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmSetDevState_invalidStateBoundary)

/* Test: TC-FSM-0032 */
#define FSM_TEST_FSMSETDEVSTATE() \
    FSM_TEST_POS_FSMSETDEVSTATE(); \
    FSM_TEST_NEG_FSMSETDEVSTATE()

/* ======================================================================== */
/*                        Test APIs: fsmGetDevState                         */
/* ======================================================================== */

#define FSM_TEST_POS_FSMGETDEVSTATE() \
    PLATFORM_RUN_TEST(test_pos_fsm_fsmGetDevState_validRange); \
    PLATFORM_RUN_TEST(test_pos_fsm_fsmGetDevState_validRead); \
    PLATFORM_RUN_TEST(test_pos_fsm_fsmGetDevState_initStateMapping); \
    PLATFORM_RUN_TEST(test_pos_fsm_fsmGetDevState_offStateRepeated); \
    PLATFORM_RUN_TEST(test_pos_fsm_fsmGetDevState_initState); \
    PLATFORM_RUN_TEST(test_pos_fsm_fsmGetDevState_offState)

#define FSM_TEST_NEG_FSMGETDEVSTATE() \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmGetDevState_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmGetDevState_nullPointer)

/* Test: TC-FSM-0002 */
#define FSM_TEST_FSMGETDEVSTATE() \
    FSM_TEST_POS_FSMGETDEVSTATE(); \
    FSM_TEST_NEG_FSMGETDEVSTATE()

/* ======================================================================== */
/*                           Test APIs: fsmSetCfg                           */
/* ======================================================================== */

#define FSM_TEST_POS_FSMSETCFG() \
    PLATFORM_RUN_TEST(test_pos_fsm_fsmSetCfg_stbyEn); \
    PLATFORM_RUN_TEST(test_pos_fsm_fsmSetCfg_autoBistEn); \
    PLATFORM_RUN_TEST(test_pos_fsm_fsmSetCfg_nrstActiveInStbySeq); \
    PLATFORM_RUN_TEST(test_pos_fsm_fsmSetCfg_pwdThr); \
    PLATFORM_RUN_TEST(test_pos_fsm_fsmSetCfg_nrstExt); \
    PLATFORM_RUN_TEST(test_pos_fsm_fsmSetCfg_rstMcuTmo); \
    PLATFORM_RUN_TEST(test_pos_fsm_fsmSetCfg_safeTmo); \
    PLATFORM_RUN_TEST(test_pos_fsm_fsmSetCfg_safeLockThr); \
    PLATFORM_RUN_TEST(test_pos_fsm_fsmSetCfg_vbatStbyEntryThr); \
    PLATFORM_RUN_TEST(test_pos_fsm_fsmSetCfg_multiple); \
    PLATFORM_RUN_TEST(test_pos_fsm_fsmSetCfg_higherVbatStbyExitThr)

#define FSM_TEST_NEG_FSMSETCFG() \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmSetCfg_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmSetCfg_nullPointer); \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmSetCfg_invalidParams); \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmSetCfg_invalidVbatStbyEntryThr); \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmSetCfg_invalidPwdThr); \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmSetCfg_invalidRstMcuTmo); \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmSetCfg_invalidNrstExt); \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmSetCfg_invalidSafeTmo); \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmSetCfg_invalidSafeLockThr)

/* Test: TC-FSM-0003 */
#define FSM_TEST_FSMSETCFG() \
    FSM_TEST_POS_FSMSETCFG(); \
    FSM_TEST_NEG_FSMSETCFG()

/* ======================================================================== */
/*                           Test APIs: fsmGetCfg                           */
/* ======================================================================== */

#define FSM_TEST_POS_FSMGETCFG() \
    /* Positive tests for fsmGetCfg are combined with fsmSetCfg tests */ \
    PLATFORM_RUN_TEST(test_pos_fsm_fsmGetCfg_vbatStbyEntryThr)

#define FSM_TEST_NEG_FSMGETCFG() \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmGetCfg_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmGetCfg_nullPointer); \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmGetCfg_invalidParams)

/* Test: TC-FSM-0004 */
#define FSM_TEST_FSMGETCFG() \
    FSM_TEST_POS_FSMGETCFG(); \
    FSM_TEST_NEG_FSMGETCFG()

/* ======================================================================== */
/*                        Test APIs: fsmSetDevErrCnt                        */
/* ======================================================================== */

#define FSM_TEST_POS_FSMSETDEVERRCNT() \
    PLATFORM_RUN_TEST(test_pos_fsm_fsmSetDevErrCnt_basic)

#define FSM_TEST_NEG_FSMSETDEVERRCNT() \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmSetDevErrCnt_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmSetDevErrCnt_outOfBounds)

/* Test: TC-FSM-0005 */
#define FSM_TEST_FSMSETDEVERRCNT() \
    FSM_TEST_POS_FSMSETDEVERRCNT(); \
    FSM_TEST_NEG_FSMSETDEVERRCNT()

/* ======================================================================== */
/*                        Test APIs: fsmGetDevErrCnt                        */
/* ======================================================================== */

#define FSM_TEST_POS_FSMGETDEVERRCNT() \
    /* Positive tests for fsmGetDevErrCnt are combined with fsmSetDevErrCnt tests */

#define FSM_TEST_NEG_FSMGETDEVERRCNT() \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmGetDevErrCnt_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmGetDevErrCnt_nullPointer)

/* Test: TC-FSM-0006 */
#define FSM_TEST_FSMGETDEVERRCNT() \
    FSM_TEST_POS_FSMGETDEVERRCNT(); \
    FSM_TEST_NEG_FSMGETDEVERRCNT()

/* ======================================================================== */
/*                        Test APIs: fsmSetWakeupCfg                        */
/* ======================================================================== */

#define FSM_TEST_POS_FSMSETWAKEUPCFG() \
    PLATFORM_RUN_TEST(test_pos_fsm_fsmSetWakeupCfg_basic); \
    PLATFORM_RUN_TEST(test_pos_fsm_fsmSetWakeupCfg_withAllValidParams)

#define FSM_TEST_NEG_FSMSETWAKEUPCFG() \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmSetWakeupCfg_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmSetWakeupCfg_nullPointer); \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmSetWakeupCfg_zeroValidParams); \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmSetWakeupCfg_invalidWake1Event); \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmSetWakeupCfg_invalidWake2Event); \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmSetWakeupCfg_invalidWake1Dgl); \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmSetWakeupCfg_invalidWake2Dgl)

/* Test: TC-FSM-0007 */
#define FSM_TEST_FSMSETWAKEUPCFG() \
    FSM_TEST_POS_FSMSETWAKEUPCFG(); \
    FSM_TEST_NEG_FSMSETWAKEUPCFG()

/* ======================================================================== */
/*                        Test APIs: fsmGetWakeupCfg                        */
/* ======================================================================== */

#define FSM_TEST_POS_FSMGETWAKEUPCFG() \
    /* Positive tests for fsmGetWakeupCfg are combined with fsmSetWakeupCfg tests */ \
    PLATFORM_RUN_TEST(test_pos_fsm_fsmGetWakeupCfg_individualParams)

#define FSM_TEST_NEG_FSMGETWAKEUPCFG() \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmGetWakeupCfg_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmGetWakeupCfg_nullPointer); \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmGetWakeupCfg_zeroValidParams)

/* Test: TC-FSM-0008 */
#define FSM_TEST_FSMGETWAKEUPCFG() \
    FSM_TEST_POS_FSMGETWAKEUPCFG(); \
    FSM_TEST_NEG_FSMGETWAKEUPCFG()

/* ======================================================================== */
/*                       Test APIs: fsmGetWakeStatus                        */
/* ======================================================================== */

#define FSM_TEST_POS_FSMGETWAKESTATUS() \
    PLATFORM_RUN_TEST(test_pos_fsm_fsmGetWakeStatus_basic)

#define FSM_TEST_NEG_FSMGETWAKESTATUS() \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmGetWakeStatus_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmGetWakeStatus_nullPointer)

/* Test: TC-FSM-0009 */
#define FSM_TEST_FSMGETWAKESTATUS() \
    FSM_TEST_POS_FSMGETWAKESTATUS(); \
    FSM_TEST_NEG_FSMGETWAKESTATUS()

/* ======================================================================== */
/*                      Test APIs: fsmSetPowerLatchCfg                      */
/* ======================================================================== */

#define FSM_TEST_POS_FSMSETPOWERLATCHCFG() \
    PLATFORM_RUN_TEST(test_pos_fsm_fsmSetPowerLatchCfg_basic); \
    PLATFORM_RUN_TEST(test_pos_fsm_fsmSetPowerLatchCfg_stbyErrWakeEvent)

#define FSM_TEST_NEG_FSMSETPOWERLATCHCFG() \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmSetPowerLatchCfg_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmSetPowerLatchCfg_nullPointer); \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmSetPowerLatchCfg_zeroValidParams); \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmSetPowerLatchCfg_invalidPwdDly)

/* Test: TC-FSM-0010 */
#define FSM_TEST_FSMSETPOWERLATCHCFG() \
    FSM_TEST_POS_FSMSETPOWERLATCHCFG(); \
    FSM_TEST_NEG_FSMSETPOWERLATCHCFG()

/* ======================================================================== */
/*                      Test APIs: fsmGetPowerLatchCfg                      */
/* ======================================================================== */

#define FSM_TEST_POS_FSMGETPOWERLATCHCFG() \
    /* Positive tests for fsmGetPowerLatchCfg are combined with fsmSetPowerLatchCfg tests */ \
    PLATFORM_RUN_TEST(test_pos_fsm_fsmGetPowerLatchCfg_individualParams)

#define FSM_TEST_NEG_FSMGETPOWERLATCHCFG() \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmGetPowerLatchCfg_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmGetPowerLatchCfg_nullPointer); \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmGetPowerLatchCfg_zeroValidParams)

/* Test: TC-FSM-0011 */
#define FSM_TEST_FSMGETPOWERLATCHCFG() \
    FSM_TEST_POS_FSMGETPOWERLATCHCFG(); \
    FSM_TEST_NEG_FSMGETPOWERLATCHCFG()

/* ======================================================================== */
/*                       Test APIs: fsmSetPowerLatch                        */
/* ======================================================================== */

#define FSM_TEST_POS_FSMSETPOWERLATCH() \
    PLATFORM_RUN_TEST(test_pos_fsm_fsmSetPowerLatch_basic)

#define FSM_TEST_NEG_FSMSETPOWERLATCH() \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmSetPowerLatch_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmSetPowerLatch_nullPointer); \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmSetPowerLatch_zeroValidParams)

/* Test: TC-FSM-0012 */
#define FSM_TEST_FSMSETPOWERLATCH() \
    FSM_TEST_POS_FSMSETPOWERLATCH(); \
    FSM_TEST_NEG_FSMSETPOWERLATCH()

/* ======================================================================== */
/*                       Test APIs: fsmGetPowerLatch                        */
/* ======================================================================== */

#define FSM_TEST_POS_FSMGETPOWERLATCH() \
    /* Positive tests for fsmGetPowerLatch are combined with fsmSetPowerLatch tests */ \
    PLATFORM_RUN_TEST(test_pos_fsm_fsmGetPowerLatch_individualParams)

#define FSM_TEST_NEG_FSMGETPOWERLATCH() \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmGetPowerLatch_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmGetPowerLatch_nullPointer); \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmGetPowerLatch_zeroValidParams)

/* Test: TC-FSM-0013 */
#define FSM_TEST_FSMGETPOWERLATCH() \
    FSM_TEST_POS_FSMGETPOWERLATCH(); \
    FSM_TEST_NEG_FSMGETPOWERLATCH()

/* ======================================================================== */
/*                Test APIs: fsmGetLastResetMcuStateDuration                */
/* ======================================================================== */

#define FSM_TEST_POS_FSMGETLASTRESETMCUSTATEDURATION() \
    PLATFORM_RUN_TEST(test_pos_fsm_fsmGetLastResetMcuStateDuration_basic)

#define FSM_TEST_NEG_FSMGETLASTRESETMCUSTATEDURATION() \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmGetLastResetMcuStateDuration_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_fsm_fsmGetLastResetMcuStateDuration_nullPointer)

/* Test: TC-FSM-0014 */
#define FSM_TEST_FSMGETLASTRESETMCUSTATEDURATION() \
    FSM_TEST_POS_FSMGETLASTRESETMCUSTATEDURATION(); \
    FSM_TEST_NEG_FSMGETLASTRESETMCUSTATEDURATION()

/* ========================================================================== */
/*                         Aggregate Test Runners                             */
/* ========================================================================== */

/**
 * @brief Run all FSM tests (positive and negative).
 */
#define FSM_TEST_RUN_ALL() \
    FSM_TEST_RUN_POSITIVE(); \
    FSM_TEST_RUN_NEGATIVE()

/**
 * @brief Run all positive FSM tests.
 */
#define FSM_TEST_RUN_POSITIVE() \
    FSM_TEST_POS_FSMSETDEVSTATE(); \
    FSM_TEST_POS_FSMGETDEVSTATE(); \
    FSM_TEST_POS_FSMSETCFG(); \
    FSM_TEST_POS_FSMGETCFG(); \
    FSM_TEST_POS_FSMSETDEVERRCNT(); \
    FSM_TEST_POS_FSMGETDEVERRCNT(); \
    FSM_TEST_POS_FSMSETWAKEUPCFG(); \
    FSM_TEST_POS_FSMGETWAKEUPCFG(); \
    FSM_TEST_POS_FSMGETWAKESTATUS(); \
    FSM_TEST_POS_FSMSETPOWERLATCHCFG(); \
    FSM_TEST_POS_FSMGETPOWERLATCHCFG(); \
    FSM_TEST_POS_FSMSETPOWERLATCH(); \
    FSM_TEST_POS_FSMGETPOWERLATCH(); \
    FSM_TEST_POS_FSMGETLASTRESETMCUSTATEDURATION()

/**
 * @brief Run all negative FSM tests.
 */
#define FSM_TEST_RUN_NEGATIVE() \
    FSM_TEST_NEG_FSMSETDEVSTATE(); \
    FSM_TEST_NEG_FSMGETDEVSTATE(); \
    FSM_TEST_NEG_FSMSETCFG(); \
    FSM_TEST_NEG_FSMGETCFG(); \
    FSM_TEST_NEG_FSMSETDEVERRCNT(); \
    FSM_TEST_NEG_FSMGETDEVERRCNT(); \
    FSM_TEST_NEG_FSMSETWAKEUPCFG(); \
    FSM_TEST_NEG_FSMGETWAKEUPCFG(); \
    FSM_TEST_NEG_FSMGETWAKESTATUS(); \
    FSM_TEST_NEG_FSMSETPOWERLATCHCFG(); \
    FSM_TEST_NEG_FSMGETPOWERLATCHCFG(); \
    FSM_TEST_NEG_FSMSETPOWERLATCH(); \
    FSM_TEST_NEG_FSMGETPOWERLATCH(); \
    FSM_TEST_NEG_FSMGETLASTRESETMCUSTATEDURATION()

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* ========================================================================== */
/*                       Positive Test Declarations                           */
/* ========================================================================== */

/**
 * @brief Test: Set and get FSM device state.
 */
void test_pos_fsm_fsmSetDevState_validStates(void);

/**
 * @brief Test: Get FSM device state.
 */
void test_pos_fsm_fsmGetDevState_validRange(void);

/**
 * @brief Test: Set and get FSM standby enable configuration.
 */
void test_pos_fsm_fsmSetCfg_stbyEn(void);

/**
 * @brief Test: Set and get FSM auto BIST enable configuration.
 */
void test_pos_fsm_fsmSetCfg_autoBistEn(void);

/**
 * @brief Test: Set and get FSM nRST active in standby sequence configuration.
 */
void test_pos_fsm_fsmSetCfg_nrstActiveInStbySeq(void);

/**
 * @brief Test: Set and get FSM power-down threshold configuration.
 */
void test_pos_fsm_fsmSetCfg_pwdThr(void);

/**
 * @brief Test: Set and get FSM nRST extension configuration.
 */
void test_pos_fsm_fsmSetCfg_nrstExt(void);

/**
 * @brief Test: Set and get FSM RESET-MCU timeout configuration.
 */
void test_pos_fsm_fsmSetCfg_rstMcuTmo(void);

/**
 * @brief Test: Set and get FSM SAFE timeout configuration.
 */
void test_pos_fsm_fsmSetCfg_safeTmo(void);

/**
 * @brief Test: Set and get FSM SAFE lock threshold configuration.
 */
void test_pos_fsm_fsmSetCfg_safeLockThr(void);

/**
 * @brief Test: Set and get FSM VBAT standby entry threshold configuration.
 */
void test_pos_fsm_fsmSetCfg_vbatStbyEntryThr(void);

/**
 * @brief Test: Set and get multiple FSM configuration parameters simultaneously.
 */
void test_pos_fsm_fsmSetCfg_multiple(void);

/**
 * @brief Test: Set and get FSM device error count.
 */
void test_pos_fsm_fsmSetDevErrCnt_basic(void);

/**
 * @brief Test: Set and get FSM wakeup configuration.
 */
void test_pos_fsm_fsmSetWakeupCfg_basic(void);

/**
 * @brief Test: Get FSM wakeup status.
 */
void test_pos_fsm_fsmGetWakeStatus_basic(void);

/**
 * @brief Test: Set and get FSM power latch configuration.
 */
void test_pos_fsm_fsmSetPowerLatchCfg_basic(void);

/**
 * @brief Test: Set and get FSM power latch status.
 */
void test_pos_fsm_fsmSetPowerLatch_basic(void);

/**
 * @brief Test: Get last RESET-MCU state duration.
 */
void test_pos_fsm_fsmGetLastResetMcuStateDuration_basic(void);

/**
 * @brief Test: Set and get FSM higher VBAT standby exit threshold configuration.
 */
void test_pos_fsm_fsmSetCfg_higherVbatStbyExitThr(void);

/**
 * @brief Test: Get FSM state when in INIT state (covers INIT state path).
 */
void test_pos_fsm_fsmGetDevState_initState(void);

/**
 * @brief Test: Get FSM state when in OFF state (covers OFF_STATE_REPEATED path).
 */
void test_pos_fsm_fsmGetDevState_offState(void);

/**
 * @brief Test: Set and get FSM wakeup configuration with all valid parameters.
 */
void test_pos_fsm_fsmSetWakeupCfg_withAllValidParams(void);

/**
 * @brief Test: Get FSM vbatStbyEntryThr configuration individually.
 */
void test_pos_fsm_fsmGetCfg_vbatStbyEntryThr(void);

/**
 * @brief Test: Get FSM wakeup configuration parameters individually.
 */
void test_pos_fsm_fsmGetWakeupCfg_individualParams(void);

/**
 * @brief Test: Get FSM power latch configuration parameters individually.
 */
void test_pos_fsm_fsmGetPowerLatchCfg_individualParams(void);

/**
 * @brief Test: Get FSM power latch status parameters individually.
 */
void test_pos_fsm_fsmGetPowerLatch_individualParams(void);

/**
 * @brief Test: Get FSM device state reads current state correctly after state requests.
 */
void test_pos_fsm_fsmGetDevState_validRead(void);

/* Coverage tests for pmic_fsm.c */
void test_pos_fsm_fsmGetDevState_validRange_initStateMapping(void);
void test_pos_fsm_fsmGetDevState_validRange_offStateRepeated(void);
void test_pos_fsm_fsmSetPowerLatchCfg_stbyErrWakeEvent(void);

/* ========================================================================== */
/*                       Negative Test Declarations                           */
/* ========================================================================== */

/**
 * @brief Test: Pmic_fsmSetDevState() with NULL handle.
 */
void test_neg_fsm_fsmSetDevState_nullHandle(void);

/**
 * @brief Test: Pmic_fsmSetDevState() with invalid state value.
 */
void test_neg_fsm_fsmSetDevState_invalidState(void);

/**
 * @brief Test: Pmic_fsmGetDevState() with NULL handle.
 */
void test_neg_fsm_fsmGetDevState_nullHandle(void);

/**
 * @brief Test: Pmic_fsmGetDevState() with NULL state pointer.
 */
void test_neg_fsm_fsmGetDevState_nullPointer(void);

/**
 * @brief Test: Pmic_fsmSetCfg() with NULL handle.
 */
void test_neg_fsm_fsmSetCfg_nullHandle(void);

/**
 * @brief Test: Pmic_fsmSetCfg() with NULL configuration pointer.
 */
void test_neg_fsm_fsmSetCfg_nullPointer(void);

/**
 * @brief Test: Pmic_fsmSetCfg() with invalid validParams (0).
 */
void test_neg_fsm_fsmSetCfg_invalidParams(void);

/**
 * @brief Test: Pmic_fsmGetCfg() with NULL handle.
 */
void test_neg_fsm_fsmGetCfg_nullHandle(void);

/**
 * @brief Test: Pmic_fsmGetCfg() with NULL configuration pointer.
 */
void test_neg_fsm_fsmGetCfg_nullPointer(void);

/**
 * @brief Test: Pmic_fsmGetCfg() with invalid validParams (0).
 */
void test_neg_fsm_fsmGetCfg_invalidParams(void);

/**
 * @brief Test: Pmic_fsmSetDevErrCnt() with NULL handle.
 */
void test_neg_fsm_fsmSetDevErrCnt_nullHandle(void);

/**
 * @brief Test: Pmic_fsmSetDevErrCnt() with out-of-bounds error count.
 */
void test_neg_fsm_fsmSetDevErrCnt_outOfBounds(void);

/**
 * @brief Test: Pmic_fsmGetDevErrCnt() with NULL handle.
 */
void test_neg_fsm_fsmGetDevErrCnt_nullHandle(void);

/**
 * @brief Test: Pmic_fsmGetDevErrCnt() with NULL error count pointer.
 */
void test_neg_fsm_fsmGetDevErrCnt_nullPointer(void);

/**
 * @brief Test: Pmic_fsmSetWakeupCfg() with NULL handle.
 */
void test_neg_fsm_fsmSetWakeupCfg_nullHandle(void);

/**
 * @brief Test: Pmic_fsmSetWakeupCfg() with NULL configuration pointer.
 */
void test_neg_fsm_fsmSetWakeupCfg_nullPointer(void);

/**
 * @brief Test: Pmic_fsmGetWakeStatus() with NULL handle.
 */
void test_neg_fsm_fsmGetWakeStatus_nullHandle(void);

/**
 * @brief Test: Pmic_fsmGetWakeStatus() with NULL status pointer.
 */
void test_neg_fsm_fsmGetWakeStatus_nullPointer(void);

/**
 * @brief Test: Pmic_fsmSetPowerLatchCfg() with NULL handle.
 */
void test_neg_fsm_fsmSetPowerLatchCfg_nullHandle(void);

/**
 * @brief Test: Pmic_fsmSetPowerLatchCfg() with NULL configuration pointer.
 */
void test_neg_fsm_fsmSetPowerLatchCfg_nullPointer(void);

/**
 * @brief Test: Pmic_fsmGetPowerLatch() with NULL handle.
 */
void test_neg_fsm_fsmGetPowerLatch_nullHandle(void);

/**
 * @brief Test: Pmic_fsmGetPowerLatch() with NULL latch pointer.
 */
void test_neg_fsm_fsmGetPowerLatch_nullPointer(void);

/**
 * @brief Test: Pmic_fsmSetCfg() with invalid vbatStbyEntryThr value.
 */
void test_neg_fsm_fsmSetCfg_invalidVbatStbyEntryThr(void);

/**
 * @brief Test: Pmic_fsmSetCfg() with invalid pwdThr value.
 */
void test_neg_fsm_fsmSetCfg_invalidPwdThr(void);

/**
 * @brief Test: Pmic_fsmSetCfg() with invalid rstMcuTmo value.
 */
void test_neg_fsm_fsmSetCfg_invalidRstMcuTmo(void);

/**
 * @brief Test: Pmic_fsmSetCfg() with invalid nrstExt value.
 */
void test_neg_fsm_fsmSetCfg_invalidNrstExt(void);

/**
 * @brief Test: Pmic_fsmSetCfg() with invalid safeTmo value.
 */
void test_neg_fsm_fsmSetCfg_invalidSafeTmo(void);

/**
 * @brief Test: Pmic_fsmSetCfg() with invalid safeLockThr value.
 */
void test_neg_fsm_fsmSetCfg_invalidSafeLockThr(void);

/**
 * @brief Test: Pmic_fsmSetWakeupCfg() with zero validParams.
 */
void test_neg_fsm_fsmSetWakeupCfg_zeroValidParams(void);

/**
 * @brief Test: Pmic_fsmSetWakeupCfg() with invalid wake1Event value.
 */
void test_neg_fsm_fsmSetWakeupCfg_invalidWake1Event(void);

/**
 * @brief Test: Pmic_fsmSetWakeupCfg() with invalid wake2Event value.
 */
void test_neg_fsm_fsmSetWakeupCfg_invalidWake2Event(void);

/**
 * @brief Test: Pmic_fsmSetWakeupCfg() with invalid wake1Dgl value.
 */
void test_neg_fsm_fsmSetWakeupCfg_invalidWake1Dgl(void);

/**
 * @brief Test: Pmic_fsmSetWakeupCfg() with invalid wake2Dgl value.
 */
void test_neg_fsm_fsmSetWakeupCfg_invalidWake2Dgl(void);

/**
 * @brief Test: Pmic_fsmGetWakeupCfg() with NULL handle.
 */
void test_neg_fsm_fsmGetWakeupCfg_nullHandle(void);

/**
 * @brief Test: Pmic_fsmGetWakeupCfg() with NULL configuration pointer.
 */
void test_neg_fsm_fsmGetWakeupCfg_nullPointer(void);

/**
 * @brief Test: Pmic_fsmGetWakeupCfg() with zero validParams.
 */
void test_neg_fsm_fsmGetWakeupCfg_zeroValidParams(void);

/**
 * @brief Test: Pmic_fsmSetPowerLatchCfg() with zero validParams.
 */
void test_neg_fsm_fsmSetPowerLatchCfg_zeroValidParams(void);

/**
 * @brief Test: Pmic_fsmSetPowerLatchCfg() with invalid pwdDly value.
 */
void test_neg_fsm_fsmSetPowerLatchCfg_invalidPwdDly(void);

/**
 * @brief Test: Pmic_fsmGetPowerLatchCfg() with NULL handle.
 */
void test_neg_fsm_fsmGetPowerLatchCfg_nullHandle(void);

/**
 * @brief Test: Pmic_fsmGetPowerLatchCfg() with NULL configuration pointer.
 */
void test_neg_fsm_fsmGetPowerLatchCfg_nullPointer(void);

/**
 * @brief Test: Pmic_fsmGetPowerLatchCfg() with zero validParams.
 */
void test_neg_fsm_fsmGetPowerLatchCfg_zeroValidParams(void);

/**
 * @brief Test: Pmic_fsmSetPowerLatch() with NULL handle.
 */
void test_neg_fsm_fsmSetPowerLatch_nullHandle(void);

/**
 * @brief Test: Pmic_fsmSetPowerLatch() with NULL latch pointer.
 */
void test_neg_fsm_fsmSetPowerLatch_nullPointer(void);

/**
 * @brief Test: Pmic_fsmSetPowerLatch() with zero validParams.
 */
void test_neg_fsm_fsmSetPowerLatch_zeroValidParams(void);

/**
 * @brief Test: Pmic_fsmGetPowerLatch() with zero validParams.
 */
void test_neg_fsm_fsmGetPowerLatch_zeroValidParams(void);

/**
 * @brief Test: Pmic_fsmGetLastResetMcuStateDuration() with NULL handle.
 */
void test_neg_fsm_fsmGetLastResetMcuStateDuration_nullHandle(void);

/**
 * @brief Test: Pmic_fsmGetLastResetMcuStateDuration() with NULL duration pointer.
 */
void test_neg_fsm_fsmGetLastResetMcuStateDuration_nullPointer(void);

/* ========================================================================== */
/*                         Unity Framework Functions                          */
/* ========================================================================== */

/* setUp() and tearDown() are defined in test_runner.c, not here */

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* FSM_TEST_H */
