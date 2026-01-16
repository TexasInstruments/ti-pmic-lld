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
    PLATFORM_RUN_TEST(test_positive_fsm_setGetDevState); \
    PLATFORM_RUN_TEST(test_positive_fsm_getDevState); \
    PLATFORM_RUN_TEST(test_positive_fsm_setGetCfg_stbyEn); \
    PLATFORM_RUN_TEST(test_positive_fsm_setGetCfg_autoBistEn); \
    PLATFORM_RUN_TEST(test_positive_fsm_setGetCfg_nrstActiveInStbySeq); \
    PLATFORM_RUN_TEST(test_positive_fsm_setGetCfg_pwdThr); \
    PLATFORM_RUN_TEST(test_positive_fsm_setGetCfg_nrstExt); \
    PLATFORM_RUN_TEST(test_positive_fsm_setGetCfg_rstMcuTmo); \
    PLATFORM_RUN_TEST(test_positive_fsm_setGetCfg_safeTmo); \
    PLATFORM_RUN_TEST(test_positive_fsm_setGetCfg_safeLockThr); \
    PLATFORM_RUN_TEST(test_positive_fsm_setGetCfg_vbatStbyEntryThr); \
    PLATFORM_RUN_TEST(test_positive_fsm_setGetCfg_multiple); \
    PLATFORM_RUN_TEST(test_positive_fsm_setGetDevErrCnt); \
    PLATFORM_RUN_TEST(test_positive_fsm_setGetWakeupCfg); \
    PLATFORM_RUN_TEST(test_positive_fsm_getWakeStatus); \
    PLATFORM_RUN_TEST(test_positive_fsm_setGetPowerLatchCfg); \
    PLATFORM_RUN_TEST(test_positive_fsm_setGetPowerLatch); \
    PLATFORM_RUN_TEST(test_positive_fsm_getLastResetMcuStateDuration); \
    PLATFORM_RUN_TEST(test_positive_fsm_setGetCfg_higherVbatStbyExitThr); \
    PLATFORM_RUN_TEST(test_positive_fsm_getState_initState); \
    PLATFORM_RUN_TEST(test_positive_fsm_getState_offState); \
    PLATFORM_RUN_TEST(test_positive_fsm_setWakeupCfg_withAllValidParams); \
    PLATFORM_RUN_TEST(test_positive_fsm_getCfg_vbatStbyEntryThr); \
    PLATFORM_RUN_TEST(test_positive_fsm_getWakeupCfg_individualParams); \
    PLATFORM_RUN_TEST(test_positive_fsm_getPowerLatchCfg_individualParams); \
    PLATFORM_RUN_TEST(test_positive_fsm_getPowerLatch_individualParams); \
    PLATFORM_RUN_TEST(test_positive_fsmGetDevState_validRead); \
    PLATFORM_RUN_TEST(test_positive_fsm_getDevState_initStateMapping); \
    PLATFORM_RUN_TEST(test_positive_fsm_getDevState_offStateRepeated); \
    PLATFORM_RUN_TEST(test_positive_fsm_setPwrLatchCfg_stbyErrWakeEvent)

/**
 * @brief Run all negative FSM tests.
 */
#define FSM_TEST_RUN_NEGATIVE() \
    PLATFORM_RUN_TEST(test_negative_fsm_setDevState_nullHandle); \
    PLATFORM_RUN_TEST(test_negative_fsm_setDevState_invalidState); \
    PLATFORM_RUN_TEST(test_negative_fsm_getDevState_nullHandle); \
    PLATFORM_RUN_TEST(test_negative_fsm_getDevState_nullPointer); \
    PLATFORM_RUN_TEST(test_negative_fsm_setCfg_nullHandle); \
    PLATFORM_RUN_TEST(test_negative_fsm_setCfg_nullPointer); \
    PLATFORM_RUN_TEST(test_negative_fsm_setCfg_invalidParams); \
    PLATFORM_RUN_TEST(test_negative_fsm_getCfg_nullHandle); \
    PLATFORM_RUN_TEST(test_negative_fsm_getCfg_nullPointer); \
    PLATFORM_RUN_TEST(test_negative_fsm_getCfg_invalidParams); \
    PLATFORM_RUN_TEST(test_negative_fsm_setDevErrCnt_nullHandle); \
    PLATFORM_RUN_TEST(test_negative_fsm_setDevErrCnt_outOfBounds); \
    PLATFORM_RUN_TEST(test_negative_fsm_getDevErrCnt_nullHandle); \
    PLATFORM_RUN_TEST(test_negative_fsm_getDevErrCnt_nullPointer); \
    PLATFORM_RUN_TEST(test_negative_fsm_setWakeupCfg_nullHandle); \
    PLATFORM_RUN_TEST(test_negative_fsm_setWakeupCfg_nullPointer); \
    PLATFORM_RUN_TEST(test_negative_fsm_getWakeStatus_nullHandle); \
    PLATFORM_RUN_TEST(test_negative_fsm_getWakeStatus_nullPointer); \
    PLATFORM_RUN_TEST(test_negative_fsm_setPowerLatchCfg_nullHandle); \
    PLATFORM_RUN_TEST(test_negative_fsm_setPowerLatchCfg_nullPointer); \
    PLATFORM_RUN_TEST(test_negative_fsm_getPowerLatch_nullHandle); \
    PLATFORM_RUN_TEST(test_negative_fsm_getPowerLatch_nullPointer); \
    PLATFORM_RUN_TEST(test_negative_fsm_setCfg_invalidVbatStbyEntryThr); \
    PLATFORM_RUN_TEST(test_negative_fsm_setCfg_invalidPwdThr); \
    PLATFORM_RUN_TEST(test_negative_fsm_setCfg_invalidRstMcuTmo); \
    PLATFORM_RUN_TEST(test_negative_fsm_setCfg_invalidNrstExt); \
    PLATFORM_RUN_TEST(test_negative_fsm_setCfg_invalidSafeTmo); \
    PLATFORM_RUN_TEST(test_negative_fsm_setCfg_invalidSafeLockThr); \
    PLATFORM_RUN_TEST(test_negative_fsm_setWakeupCfg_zeroValidParams); \
    PLATFORM_RUN_TEST(test_negative_fsm_setWakeupCfg_invalidWake1Event); \
    PLATFORM_RUN_TEST(test_negative_fsm_setWakeupCfg_invalidWake2Event); \
    PLATFORM_RUN_TEST(test_negative_fsm_setWakeupCfg_invalidWake1Dgl); \
    PLATFORM_RUN_TEST(test_negative_fsm_setWakeupCfg_invalidWake2Dgl); \
    PLATFORM_RUN_TEST(test_negative_fsm_getWakeupCfg_nullHandle); \
    PLATFORM_RUN_TEST(test_negative_fsm_getWakeupCfg_nullPointer); \
    PLATFORM_RUN_TEST(test_negative_fsm_getWakeupCfg_zeroValidParams); \
    PLATFORM_RUN_TEST(test_negative_fsm_setPowerLatchCfg_zeroValidParams); \
    PLATFORM_RUN_TEST(test_negative_fsm_setPowerLatchCfg_invalidPwdDly); \
    PLATFORM_RUN_TEST(test_negative_fsm_getPowerLatchCfg_nullHandle); \
    PLATFORM_RUN_TEST(test_negative_fsm_getPowerLatchCfg_nullPointer); \
    PLATFORM_RUN_TEST(test_negative_fsm_getPowerLatchCfg_zeroValidParams); \
    PLATFORM_RUN_TEST(test_negative_fsm_setPowerLatch_nullHandle); \
    PLATFORM_RUN_TEST(test_negative_fsm_setPowerLatch_nullPointer); \
    PLATFORM_RUN_TEST(test_negative_fsm_setPowerLatch_zeroValidParams); \
    PLATFORM_RUN_TEST(test_negative_fsm_getPowerLatch_zeroValidParams); \
    PLATFORM_RUN_TEST(test_negative_fsm_getLastResetMcuStateDuration_nullHandle); \
    PLATFORM_RUN_TEST(test_negative_fsm_getLastResetMcuStateDuration_nullPointer); \
    PLATFORM_RUN_TEST(test_negative_fsmSetDevState_invalidState)

/* ========================================================================== */
/*                       Positive Test Declarations                           */
/* ========================================================================== */

/**
 * @brief Test: Set and get FSM device state.
 */
void test_positive_fsm_setGetDevState(void);

/**
 * @brief Test: Get FSM device state.
 */
void test_positive_fsm_getDevState(void);

/**
 * @brief Test: Set and get FSM standby enable configuration.
 */
void test_positive_fsm_setGetCfg_stbyEn(void);

/**
 * @brief Test: Set and get FSM auto BIST enable configuration.
 */
void test_positive_fsm_setGetCfg_autoBistEn(void);

/**
 * @brief Test: Set and get FSM nRST active in standby sequence configuration.
 */
void test_positive_fsm_setGetCfg_nrstActiveInStbySeq(void);

/**
 * @brief Test: Set and get FSM power-down threshold configuration.
 */
void test_positive_fsm_setGetCfg_pwdThr(void);

/**
 * @brief Test: Set and get FSM nRST extension configuration.
 */
void test_positive_fsm_setGetCfg_nrstExt(void);

/**
 * @brief Test: Set and get FSM RESET-MCU timeout configuration.
 */
void test_positive_fsm_setGetCfg_rstMcuTmo(void);

/**
 * @brief Test: Set and get FSM SAFE timeout configuration.
 */
void test_positive_fsm_setGetCfg_safeTmo(void);

/**
 * @brief Test: Set and get FSM SAFE lock threshold configuration.
 */
void test_positive_fsm_setGetCfg_safeLockThr(void);

/**
 * @brief Test: Set and get FSM VBAT standby entry threshold configuration.
 */
void test_positive_fsm_setGetCfg_vbatStbyEntryThr(void);

/**
 * @brief Test: Set and get multiple FSM configuration parameters simultaneously.
 */
void test_positive_fsm_setGetCfg_multiple(void);

/**
 * @brief Test: Set and get FSM device error count.
 */
void test_positive_fsm_setGetDevErrCnt(void);

/**
 * @brief Test: Set and get FSM wakeup configuration.
 */
void test_positive_fsm_setGetWakeupCfg(void);

/**
 * @brief Test: Get FSM wakeup status.
 */
void test_positive_fsm_getWakeStatus(void);

/**
 * @brief Test: Set and get FSM power latch configuration.
 */
void test_positive_fsm_setGetPowerLatchCfg(void);

/**
 * @brief Test: Set and get FSM power latch status.
 */
void test_positive_fsm_setGetPowerLatch(void);

/**
 * @brief Test: Get last RESET-MCU state duration.
 */
void test_positive_fsm_getLastResetMcuStateDuration(void);

/**
 * @brief Test: Set and get FSM higher VBAT standby exit threshold configuration.
 */
void test_positive_fsm_setGetCfg_higherVbatStbyExitThr(void);

/**
 * @brief Test: Get FSM state when in INIT state (covers INIT state path).
 */
void test_positive_fsm_getState_initState(void);

/**
 * @brief Test: Get FSM state when in OFF state (covers OFF_STATE_REPEATED path).
 */
void test_positive_fsm_getState_offState(void);

/**
 * @brief Test: Set and get FSM wakeup configuration with all valid parameters.
 */
void test_positive_fsm_setWakeupCfg_withAllValidParams(void);

/**
 * @brief Test: Get FSM vbatStbyEntryThr configuration individually.
 */
void test_positive_fsm_getCfg_vbatStbyEntryThr(void);

/**
 * @brief Test: Get FSM wakeup configuration parameters individually.
 */
void test_positive_fsm_getWakeupCfg_individualParams(void);

/**
 * @brief Test: Get FSM power latch configuration parameters individually.
 */
void test_positive_fsm_getPowerLatchCfg_individualParams(void);

/**
 * @brief Test: Get FSM power latch status parameters individually.
 */
void test_positive_fsm_getPowerLatch_individualParams(void);

/**
 * @brief Test: Get FSM device state reads current state correctly after state requests.
 */
void test_positive_fsmGetDevState_validRead(void);

/* Coverage tests for pmic_fsm.c */
void test_positive_fsm_getDevState_initStateMapping(void);
void test_positive_fsm_getDevState_offStateRepeated(void);
void test_positive_fsm_setPwrLatchCfg_stbyErrWakeEvent(void);

/* ========================================================================== */
/*                       Negative Test Declarations                           */
/* ========================================================================== */

/**
 * @brief Test: Pmic_fsmSetDevState() with NULL handle.
 */
void test_negative_fsm_setDevState_nullHandle(void);

/**
 * @brief Test: Pmic_fsmSetDevState() with invalid state value.
 */
void test_negative_fsm_setDevState_invalidState(void);

/**
 * @brief Test: Pmic_fsmGetDevState() with NULL handle.
 */
void test_negative_fsm_getDevState_nullHandle(void);

/**
 * @brief Test: Pmic_fsmGetDevState() with NULL state pointer.
 */
void test_negative_fsm_getDevState_nullPointer(void);

/**
 * @brief Test: Pmic_fsmSetCfg() with NULL handle.
 */
void test_negative_fsm_setCfg_nullHandle(void);

/**
 * @brief Test: Pmic_fsmSetCfg() with NULL configuration pointer.
 */
void test_negative_fsm_setCfg_nullPointer(void);

/**
 * @brief Test: Pmic_fsmSetCfg() with invalid validParams (0).
 */
void test_negative_fsm_setCfg_invalidParams(void);

/**
 * @brief Test: Pmic_fsmGetCfg() with NULL handle.
 */
void test_negative_fsm_getCfg_nullHandle(void);

/**
 * @brief Test: Pmic_fsmGetCfg() with NULL configuration pointer.
 */
void test_negative_fsm_getCfg_nullPointer(void);

/**
 * @brief Test: Pmic_fsmGetCfg() with invalid validParams (0).
 */
void test_negative_fsm_getCfg_invalidParams(void);

/**
 * @brief Test: Pmic_fsmSetDevErrCnt() with NULL handle.
 */
void test_negative_fsm_setDevErrCnt_nullHandle(void);

/**
 * @brief Test: Pmic_fsmSetDevErrCnt() with out-of-bounds error count.
 */
void test_negative_fsm_setDevErrCnt_outOfBounds(void);

/**
 * @brief Test: Pmic_fsmGetDevErrCnt() with NULL handle.
 */
void test_negative_fsm_getDevErrCnt_nullHandle(void);

/**
 * @brief Test: Pmic_fsmGetDevErrCnt() with NULL error count pointer.
 */
void test_negative_fsm_getDevErrCnt_nullPointer(void);

/**
 * @brief Test: Pmic_fsmSetWakeupCfg() with NULL handle.
 */
void test_negative_fsm_setWakeupCfg_nullHandle(void);

/**
 * @brief Test: Pmic_fsmSetWakeupCfg() with NULL configuration pointer.
 */
void test_negative_fsm_setWakeupCfg_nullPointer(void);

/**
 * @brief Test: Pmic_fsmGetWakeStatus() with NULL handle.
 */
void test_negative_fsm_getWakeStatus_nullHandle(void);

/**
 * @brief Test: Pmic_fsmGetWakeStatus() with NULL status pointer.
 */
void test_negative_fsm_getWakeStatus_nullPointer(void);

/**
 * @brief Test: Pmic_fsmSetPowerLatchCfg() with NULL handle.
 */
void test_negative_fsm_setPowerLatchCfg_nullHandle(void);

/**
 * @brief Test: Pmic_fsmSetPowerLatchCfg() with NULL configuration pointer.
 */
void test_negative_fsm_setPowerLatchCfg_nullPointer(void);

/**
 * @brief Test: Pmic_fsmGetPowerLatch() with NULL handle.
 */
void test_negative_fsm_getPowerLatch_nullHandle(void);

/**
 * @brief Test: Pmic_fsmGetPowerLatch() with NULL latch pointer.
 */
void test_negative_fsm_getPowerLatch_nullPointer(void);

/**
 * @brief Test: Pmic_fsmSetCfg() with invalid vbatStbyEntryThr value.
 */
void test_negative_fsm_setCfg_invalidVbatStbyEntryThr(void);

/**
 * @brief Test: Pmic_fsmSetCfg() with invalid pwdThr value.
 */
void test_negative_fsm_setCfg_invalidPwdThr(void);

/**
 * @brief Test: Pmic_fsmSetCfg() with invalid rstMcuTmo value.
 */
void test_negative_fsm_setCfg_invalidRstMcuTmo(void);

/**
 * @brief Test: Pmic_fsmSetCfg() with invalid nrstExt value.
 */
void test_negative_fsm_setCfg_invalidNrstExt(void);

/**
 * @brief Test: Pmic_fsmSetCfg() with invalid safeTmo value.
 */
void test_negative_fsm_setCfg_invalidSafeTmo(void);

/**
 * @brief Test: Pmic_fsmSetCfg() with invalid safeLockThr value.
 */
void test_negative_fsm_setCfg_invalidSafeLockThr(void);

/**
 * @brief Test: Pmic_fsmSetWakeupCfg() with zero validParams.
 */
void test_negative_fsm_setWakeupCfg_zeroValidParams(void);

/**
 * @brief Test: Pmic_fsmSetWakeupCfg() with invalid wake1Event value.
 */
void test_negative_fsm_setWakeupCfg_invalidWake1Event(void);

/**
 * @brief Test: Pmic_fsmSetWakeupCfg() with invalid wake2Event value.
 */
void test_negative_fsm_setWakeupCfg_invalidWake2Event(void);

/**
 * @brief Test: Pmic_fsmSetWakeupCfg() with invalid wake1Dgl value.
 */
void test_negative_fsm_setWakeupCfg_invalidWake1Dgl(void);

/**
 * @brief Test: Pmic_fsmSetWakeupCfg() with invalid wake2Dgl value.
 */
void test_negative_fsm_setWakeupCfg_invalidWake2Dgl(void);

/**
 * @brief Test: Pmic_fsmGetWakeupCfg() with NULL handle.
 */
void test_negative_fsm_getWakeupCfg_nullHandle(void);

/**
 * @brief Test: Pmic_fsmGetWakeupCfg() with NULL configuration pointer.
 */
void test_negative_fsm_getWakeupCfg_nullPointer(void);

/**
 * @brief Test: Pmic_fsmGetWakeupCfg() with zero validParams.
 */
void test_negative_fsm_getWakeupCfg_zeroValidParams(void);

/**
 * @brief Test: Pmic_fsmSetPowerLatchCfg() with zero validParams.
 */
void test_negative_fsm_setPowerLatchCfg_zeroValidParams(void);

/**
 * @brief Test: Pmic_fsmSetPowerLatchCfg() with invalid pwdDly value.
 */
void test_negative_fsm_setPowerLatchCfg_invalidPwdDly(void);

/**
 * @brief Test: Pmic_fsmGetPowerLatchCfg() with NULL handle.
 */
void test_negative_fsm_getPowerLatchCfg_nullHandle(void);

/**
 * @brief Test: Pmic_fsmGetPowerLatchCfg() with NULL configuration pointer.
 */
void test_negative_fsm_getPowerLatchCfg_nullPointer(void);

/**
 * @brief Test: Pmic_fsmGetPowerLatchCfg() with zero validParams.
 */
void test_negative_fsm_getPowerLatchCfg_zeroValidParams(void);

/**
 * @brief Test: Pmic_fsmSetPowerLatch() with NULL handle.
 */
void test_negative_fsm_setPowerLatch_nullHandle(void);

/**
 * @brief Test: Pmic_fsmSetPowerLatch() with NULL latch pointer.
 */
void test_negative_fsm_setPowerLatch_nullPointer(void);

/**
 * @brief Test: Pmic_fsmSetPowerLatch() with zero validParams.
 */
void test_negative_fsm_setPowerLatch_zeroValidParams(void);

/**
 * @brief Test: Pmic_fsmGetPowerLatch() with zero validParams.
 */
void test_negative_fsm_getPowerLatch_zeroValidParams(void);

/**
 * @brief Test: Pmic_fsmGetLastResetMcuStateDuration() with NULL handle.
 */
void test_negative_fsm_getLastResetMcuStateDuration_nullHandle(void);

/**
 * @brief Test: Pmic_fsmGetLastResetMcuStateDuration() with NULL duration pointer.
 */
void test_negative_fsm_getLastResetMcuStateDuration_nullPointer(void);

/* ========================================================================== */
/*                         Unity Framework Functions                          */
/* ========================================================================== */

/* setUp() and tearDown() are defined in test_runner.c, not here */

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* FSM_TEST_H */
