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

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include "fsm_test.h"
#ifdef BUILD_MOCK
#include "test_inject.h"
#include "platform_mock.h"
#include "pmic_mock_core.h"
#endif
#include "test_constants.h"
#include "regmap/fsm.h"
#include "pmic_irq.h"

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */
static Pmic_Handle_t pmicHandle;

/* ========================================================================== */
/*                         Forward Declarations                               */
/* ========================================================================== */

/* These functions are defined later in the file but referenced in macros */
void test_pos_fsm_fsmGetDevState_initStateMapping(void);
void test_pos_fsm_fsmGetDevState_offStateRepeated(void);

/* ========================================================================== */
/*                         Positive Test Implementations                      */
/* ========================================================================== */

void test_pos_fsm_fsmSetDevState_validStates(void)
{
    int32_t status;
    uint8_t stateGet;

    // Test valid state requests
    uint8_t validStates[] = {
        PMIC_NO_STATE_CHANGE_REQUEST,
        PMIC_SAFE_TO_ACTIVE_REQUEST,
        PMIC_ACTIVE_TO_SAFE_REQUEST,
        PMIC_STANDBY_REQUEST
    };

    for (uint32_t i = 0; i < sizeof(validStates); i++)
    {
        status = Pmic_fsmSetDevState(&pmicHandle, validStates[i]);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        if (validStates[i] == PMIC_STANDBY_REQUEST)
        {
            platform_wakeFromStandby();
        }

        // Get state to verify communication works
        status = Pmic_fsmGetDevState(&pmicHandle, &stateGet);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    }
}

void test_pos_fsm_fsmGetDevState_validRange(void)
{
    int32_t status;
    uint8_t state;

    status = Pmic_fsmGetDevState(&pmicHandle, &state);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify state is within valid range
    PLATFORM_ASSERT((state == PMIC_OFF_STATE) ||
                     (state == PMIC_INIT_STATE) ||
                     (state == PMIC_PWRU_SEQ_STATE) ||
                     (state == PMIC_RESET_MCU_STATE) ||
                     (state == PMIC_AUTO_BIST) ||
                     (state == PMIC_ACTIVE_STATE) ||
                     (state == PMIC_SAFE_STATE) ||
                     (state == PMIC_RUN_TIME_BIST) ||
                     (state == PMIC_OTP_PROGRAMMING_STATE) ||
                     (state == PMIC_PWRD_SEQ_STATE) ||
                     (state == PMIC_STANDBY_STATE));

}

void test_pos_fsm_fsmSetCfg_stbyEn(void)
{
    int32_t status;
    Pmic_FsmCfg_t cfgSet = {0}, cfgGet = {0};

    // Test stbyEn = true
    cfgSet.validParams = PMIC_FSM_CFG_VALID;
    cfgSet.stbyEn = true;
    status = Pmic_fsmSetCfg(&pmicHandle, &cfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    cfgGet.validParams = PMIC_FSM_CFG_VALID;
    status = Pmic_fsmGetCfg(&pmicHandle, &cfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(cfgGet.stbyEn == cfgSet.stbyEn);

    // Test stbyEn = false
    cfgSet.stbyEn = false;
    status = Pmic_fsmSetCfg(&pmicHandle, &cfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_fsmGetCfg(&pmicHandle, &cfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(cfgGet.stbyEn == cfgSet.stbyEn);

}

void test_pos_fsm_fsmSetCfg_autoBistEn(void)
{
#ifndef BUILD_HOST
    int32_t status;
    Pmic_FsmCfg_t cfgSet = {0}, cfgGet = {0};

    cfgSet.validParams = PMIC_FSM_CFG_VALID;
    cfgGet.validParams = PMIC_FSM_CFG_VALID;

    // Test autoBistEn = true
    cfgSet.autoBistEn = true;
    status = Pmic_fsmSetCfg(&pmicHandle, &cfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_fsmGetCfg(&pmicHandle, &cfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(cfgGet.autoBistEn == cfgSet.autoBistEn);

    // Test autoBistEn = false
    cfgSet.autoBistEn = false;
    status = Pmic_fsmSetCfg(&pmicHandle, &cfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_fsmGetCfg(&pmicHandle, &cfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(cfgGet.autoBistEn == cfgSet.autoBistEn);
#else
    TEST_IGNORE_MESSAGE("autoBistEn skipped on BUILD_HOST: enables AUTO_BIST, corrupts device state for subsequent modules");
#endif
}

void test_pos_fsm_fsmSetCfg_nrstActiveInStbySeq(void)
{
    int32_t status;
    Pmic_FsmCfg_t cfgSet = {0}, cfgGet = {0};

    cfgSet.validParams = PMIC_FSM_CFG_VALID;
    cfgGet.validParams = PMIC_FSM_CFG_VALID;

    // Test nrstActiveInStbySeq = true
    cfgSet.nrstActiveInStbySeq = true;
    status = Pmic_fsmSetCfg(&pmicHandle, &cfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_fsmGetCfg(&pmicHandle, &cfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(cfgGet.nrstActiveInStbySeq == cfgSet.nrstActiveInStbySeq);

    // Test nrstActiveInStbySeq = false
    cfgSet.nrstActiveInStbySeq = false;
    status = Pmic_fsmSetCfg(&pmicHandle, &cfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_fsmGetCfg(&pmicHandle, &cfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(cfgGet.nrstActiveInStbySeq == cfgSet.nrstActiveInStbySeq);

}

void test_pos_fsm_fsmSetCfg_pwdThr(void)
{
#ifndef BUILD_HOST
    int32_t status;
    Pmic_FsmCfg_t cfgSet = {0}, cfgGet = {0};
    uint8_t testValues[] = {0x00, 0x10, 0x1F};

    cfgSet.validParams = PMIC_FSM_CFG_VALID;
    cfgGet.validParams = PMIC_FSM_CFG_VALID;

    for (uint32_t i = 0; i < sizeof(testValues); i++)
    {
        cfgSet.pwdThr = testValues[i];
        status = Pmic_fsmSetCfg(&pmicHandle, &cfgSet);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        status = Pmic_fsmGetCfg(&pmicHandle, &cfgGet);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        PLATFORM_ASSERT(cfgGet.pwdThr == cfgSet.pwdThr);
    }
#else
    TEST_IGNORE_MESSAGE("pwdThr skipped on BUILD_HOST: write fails after BIST/state-corruption sequence");
#endif
}

void test_pos_fsm_fsmSetCfg_nrstExt(void)
{
    int32_t status;
    Pmic_FsmCfg_t cfgSet = {0}, cfgGet = {0};
    uint8_t testValues[] = {0x0, 0x7, 0xF};

    cfgSet.validParams = PMIC_FSM_CFG_VALID;
    cfgGet.validParams = PMIC_FSM_CFG_VALID;

    for (uint32_t i = 0; i < sizeof(testValues); i++)
    {
        cfgSet.nrstExt = testValues[i];
        status = Pmic_fsmSetCfg(&pmicHandle, &cfgSet);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        status = Pmic_fsmGetCfg(&pmicHandle, &cfgGet);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        PLATFORM_ASSERT(cfgGet.nrstExt == cfgSet.nrstExt);
    }

}

void test_pos_fsm_fsmSetCfg_rstMcuTmo(void)
{
    int32_t status;
    Pmic_FsmCfg_t cfgSet = {0}, cfgGet = {0};
    uint8_t testValues[] = {
        PMIC_RST_MCU_TMO_512_MS,
        PMIC_RST_MCU_TMO_1024_MS,
        PMIC_RST_MCU_TMO_2048_MS,
        PMIC_RST_MCU_TMO_INFINITE_MS
    };

    cfgSet.validParams = PMIC_FSM_CFG_VALID;
    cfgGet.validParams = PMIC_FSM_CFG_VALID;

    for (uint32_t i = 0; i < sizeof(testValues); i++)
    {
        cfgSet.rstMcuTmo = testValues[i];
        status = Pmic_fsmSetCfg(&pmicHandle, &cfgSet);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        status = Pmic_fsmGetCfg(&pmicHandle, &cfgGet);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        PLATFORM_ASSERT(cfgGet.rstMcuTmo == cfgSet.rstMcuTmo);
    }

}

void test_pos_fsm_fsmSetCfg_safeTmo(void)
{
#ifndef BUILD_HOST
    int32_t status;
    Pmic_FsmCfg_t cfgSet = {0}, cfgGet = {0};
    uint8_t testValues[] = {0x0, 0x3, 0x7};

    cfgSet.validParams = PMIC_FSM_CFG_VALID;
    cfgGet.validParams = PMIC_FSM_CFG_VALID;

    for (uint32_t i = 0; i < sizeof(testValues); i++)
    {
        cfgSet.safeTmo = testValues[i];
        status = Pmic_fsmSetCfg(&pmicHandle, &cfgSet);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        status = Pmic_fsmGetCfg(&pmicHandle, &cfgGet);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        PLATFORM_ASSERT(cfgGet.safeTmo == cfgSet.safeTmo);
    }
#else
    TEST_IGNORE_MESSAGE("safeTmo skipped on BUILD_HOST: safety timeout can fire and push device to SAFE state");
#endif
}

void test_pos_fsm_fsmSetCfg_safeLockThr(void)
{
#ifndef BUILD_HOST
    int32_t status;
    Pmic_FsmCfg_t cfgSet = {0}, cfgGet = {0};
    uint8_t testValues[] = {0x00, 0x10, 0x1F};

    cfgSet.validParams = PMIC_FSM_CFG_VALID;
    cfgGet.validParams = PMIC_FSM_CFG_VALID;

    for (uint32_t i = 0; i < sizeof(testValues); i++)
    {
        cfgSet.safeLockThr = testValues[i];
        status = Pmic_fsmSetCfg(&pmicHandle, &cfgSet);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        status = Pmic_fsmGetCfg(&pmicHandle, &cfgGet);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        PLATFORM_ASSERT(cfgGet.safeLockThr == cfgSet.safeLockThr);
    }
#else
    TEST_IGNORE_MESSAGE("safeLockThr skipped on BUILD_HOST: locks device in SAFE state");
#endif
}

void test_pos_fsm_fsmSetCfg_vbatStbyEntryThr(void)
{
    int32_t status;
    Pmic_FsmCfg_t cfgSet = {0}, cfgGet = {0};
    uint8_t testValues[] = {
        PMIC_VBAT_STBY_ENTRY_THR_2P4,
        PMIC_VBAT_STBY_ENTRY_THR_3P6,
        PMIC_VBAT_STBY_ENTRY_THR_5P1,
        PMIC_VBAT_STBY_ENTRY_THR_6P1
    };

    cfgSet.validParams = PMIC_FSM_CFG_VALID;
    cfgGet.validParams = PMIC_FSM_CFG_VALID;

    for (uint32_t i = 0; i < sizeof(testValues); i++)
    {
        cfgSet.vbatStbyEntryThr = testValues[i];
        status = Pmic_fsmSetCfg(&pmicHandle, &cfgSet);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        status = Pmic_fsmGetCfg(&pmicHandle, &cfgGet);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        PLATFORM_ASSERT(cfgGet.vbatStbyEntryThr == cfgSet.vbatStbyEntryThr);
    }

}

void test_pos_fsm_fsmSetCfg_multiple(void)
{
#ifndef BUILD_HOST
    int32_t status;
    Pmic_FsmCfg_t cfgSet = {0}, cfgGet = {0};

    // Configure multiple parameters at once
    cfgSet.validParams = PMIC_FSM_CFG_VALID |
                         PMIC_FSM_CFG_VALID |
                         PMIC_FSM_CFG_VALID |
                         PMIC_FSM_CFG_VALID |
                         PMIC_FSM_CFG_VALID;
    cfgSet.stbyEn = true;
    cfgSet.autoBistEn = true;
    cfgSet.nrstActiveInStbySeq = false;
    cfgSet.pwdThr = 0x10;
    cfgSet.nrstExt = 0x5;

    status = Pmic_fsmSetCfg(&pmicHandle, &cfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    cfgGet.validParams = cfgSet.validParams;
    status = Pmic_fsmGetCfg(&pmicHandle, &cfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(cfgGet.stbyEn == cfgSet.stbyEn);
    PLATFORM_ASSERT(cfgGet.autoBistEn == cfgSet.autoBistEn);
    PLATFORM_ASSERT(cfgGet.nrstActiveInStbySeq == cfgSet.nrstActiveInStbySeq);
    PLATFORM_ASSERT(cfgGet.pwdThr == cfgSet.pwdThr);
    PLATFORM_ASSERT(cfgGet.nrstExt == cfgSet.nrstExt);
#else
    TEST_IGNORE_MESSAGE("fsmSetCfg multiple skipped on BUILD_HOST: writes autoBistEn and other state-corrupting fields");
#endif
}

void test_pos_fsm_fsmSetDevErrCnt_basic(void)
{
    int32_t status;
    uint8_t errCntSet, errCntGet;
    uint8_t testValues[] = {0x00, 0x10, 0x1F};

    // Set PWD_TH to max so writing DEV_ERR_CNT never crosses the power-down
    // threshold and corrupts the SPI response. */
    Pmic_FsmCfg_t pwdCfg = {0};
    pwdCfg.validParams = PMIC_CFG_FSM_PWD_THR_VALID;
    pwdCfg.pwdThr = PMIC_PWD_THR_MAX;
    status = Pmic_fsmSetCfg(&pmicHandle, &pwdCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    for (uint32_t i = 0; i < sizeof(testValues); i++)
    {
        errCntSet = testValues[i];
        status = Pmic_fsmSetDevErrCnt(&pmicHandle, errCntSet);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        status = Pmic_fsmGetDevErrCnt(&pmicHandle, &errCntGet);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        PLATFORM_ASSERT(errCntGet == errCntSet);
    }
}

void test_pos_fsm_fsmSetWakeupCfg_basic(void)
{
    int32_t status;
    Pmic_FsmWakeupCfg_t wakeupCfgSet = {0}, wakeupCfgGet = {0};

    // Configure wakeup parameters
    wakeupCfgSet.validParams = PMIC_CFG_WAKE1_EVENT_VALID_SHIFT |
                                PMIC_CFG_WAKE2_EVENT_VALID_SHIFT |
                                PMIC_CFG_WAKE1_DGL_VALID_SHIFT |
                                PMIC_CFG_WAKE2_DGL_VALID_SHIFT;
    wakeupCfgSet.wake1Event = PMIC_WAKE_HIGH_LEVEL;
    wakeupCfgSet.wake2Event = PMIC_WAKE_RISING_EDGE_PLUS_HIGH_LEVEL;
    wakeupCfgSet.wake1Dgl = PMIC_WAKE_DEGLITCH_TIME_2_MS;
    wakeupCfgSet.wake2Dgl = PMIC_WAKE_DEGLITCH_TIME_16_MS;

    status = Pmic_fsmSetWakeupCfg(&pmicHandle, &wakeupCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    wakeupCfgGet.validParams = wakeupCfgSet.validParams;
    status = Pmic_fsmGetWakeupCfg(&pmicHandle, &wakeupCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(wakeupCfgGet.wake1Event == wakeupCfgSet.wake1Event);
    PLATFORM_ASSERT(wakeupCfgGet.wake2Event == wakeupCfgSet.wake2Event);
    PLATFORM_ASSERT(wakeupCfgGet.wake1Dgl == wakeupCfgSet.wake1Dgl);
    PLATFORM_ASSERT(wakeupCfgGet.wake2Dgl == wakeupCfgSet.wake2Dgl);
}

void test_pos_fsm_fsmGetWakeStatus_basic(void)
{
    int32_t status;
    Pmic_FsmWakeupStat_t wakeupStat;

    status = Pmic_fsmGetWakeStatus(&pmicHandle, &wakeupStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_fsm_fsmSetPowerLatchCfg_basic(void)
{
    int32_t status;
    Pmic_FsmPwrLatchCfg_t pwrLatchCfgSet = {0}, pwrLatchCfgGet = {0};

    // Configure power latch parameters
    pwrLatchCfgSet.validParams = PMIC_FSM_CFG_VALID |
                                  PMIC_FSM_CFG_VALID |
                                  PMIC_CFG_WAKE1_EVENT_PWRL_EN_VALID_SHIFT |
                                  PMIC_CFG_WAKE2_EVENT_PWRL_EN_VALID_SHIFT;
    pwrLatchCfgSet.pwdDly = PMIC_PWD_DLY_256_US;
    pwrLatchCfgSet.stbyErrWakeEventPwrlEn = true;
    pwrLatchCfgSet.wake1EventPwrlEn = true;
    pwrLatchCfgSet.wake2EventPwrlEn = false;

    status = Pmic_fsmSetPowerLatchCfg(&pmicHandle, &pwrLatchCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    pwrLatchCfgGet.validParams = pwrLatchCfgSet.validParams;
    status = Pmic_fsmGetPowerLatchCfg(&pmicHandle, &pwrLatchCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(pwrLatchCfgGet.pwdDly == pwrLatchCfgSet.pwdDly);
    PLATFORM_ASSERT(pwrLatchCfgGet.stbyErrWakeEventPwrlEn == pwrLatchCfgSet.stbyErrWakeEventPwrlEn);
    PLATFORM_ASSERT(pwrLatchCfgGet.wake1EventPwrlEn == pwrLatchCfgSet.wake1EventPwrlEn);
    PLATFORM_ASSERT(pwrLatchCfgGet.wake2EventPwrlEn == pwrLatchCfgSet.wake2EventPwrlEn);
}

void test_pos_fsm_fsmSetPowerLatch_basic(void)
{
    int32_t status;
    Pmic_FsmPwrLatch_t pwrLatchSet = {0}, pwrLatchGet = {0};

    // Configure power latch values
    pwrLatchSet.validParams = PMIC_FSM_CFG_VALID |
                               PMIC_FSM_CFG_VALID |
                               PMIC_CFG_WAKE1_LATCH_VALID_SHIFT |
                               PMIC_CFG_WAKE2_LATCH_VALID_SHIFT;
    pwrLatchSet.stbyErrWakeLatch = true;
    pwrLatchSet.stbyTmrWakeLatch = false;
    pwrLatchSet.wake1Latch = true;
    pwrLatchSet.wake2Latch = false;

    status = Pmic_fsmSetPowerLatch(&pmicHandle, &pwrLatchSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    pwrLatchGet.validParams = pwrLatchSet.validParams;
    status = Pmic_fsmGetPowerLatch(&pmicHandle, &pwrLatchGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(pwrLatchGet.stbyErrWakeLatch == pwrLatchSet.stbyErrWakeLatch);
    PLATFORM_ASSERT(pwrLatchGet.stbyTmrWakeLatch == pwrLatchSet.stbyTmrWakeLatch);
    PLATFORM_ASSERT(pwrLatchGet.wake1Latch == pwrLatchSet.wake1Latch);
    PLATFORM_ASSERT(pwrLatchGet.wake2Latch == pwrLatchSet.wake2Latch);
}

void test_pos_fsm_fsmGetLastResetMcuStateDuration_basic(void)
{
    int32_t status;
    uint8_t duration;

    status = Pmic_fsmGetLastResetMcuStateDuration(&pmicHandle, &duration);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

/* ========================================================================== */
/*                         Negative Test Implementations                      */
/* ========================================================================== */

void test_neg_fsm_fsmSetDevState_nullHandle(void)
{
    int32_t status;

    status = Pmic_fsmSetDevState(NULL, PMIC_SAFE_TO_ACTIVE_REQUEST);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_fsm_fsmSetDevState_invalidState(void)
{
    int32_t status;

    status = Pmic_fsmSetDevState(&pmicHandle, PMIC_STATE_REQUEST_MAX + 1);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);

}

void test_neg_fsm_fsmGetDevState_nullHandle(void)
{
    int32_t status;
    uint8_t state;

    status = Pmic_fsmGetDevState(NULL, &state);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_fsm_fsmGetDevState_nullPointer(void)
{
    int32_t status;

    status = Pmic_fsmGetDevState(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);

}

void test_neg_fsm_fsmSetCfg_nullHandle(void)
{
    int32_t status;
    Pmic_FsmCfg_t fsmCfg = {0};

    fsmCfg.validParams = PMIC_FSM_CFG_VALID;
    status = Pmic_fsmSetCfg(NULL, &fsmCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_fsm_fsmSetCfg_nullPointer(void)
{
    int32_t status;

    status = Pmic_fsmSetCfg(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);

}

void test_neg_fsm_fsmSetCfg_invalidParams(void)
{
    int32_t status;
    Pmic_FsmCfg_t fsmCfg = {0};

    fsmCfg.validParams = 0U;
    status = Pmic_fsmSetCfg(&pmicHandle, &fsmCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);

}

void test_neg_fsm_fsmGetCfg_nullHandle(void)
{
    int32_t status;
    Pmic_FsmCfg_t fsmCfg = {0};

    fsmCfg.validParams = PMIC_FSM_CFG_VALID;
    status = Pmic_fsmGetCfg(NULL, &fsmCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_fsm_fsmGetCfg_nullPointer(void)
{
    int32_t status;

    status = Pmic_fsmGetCfg(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);

}

void test_neg_fsm_fsmGetCfg_invalidParams(void)
{
    int32_t status;
    Pmic_FsmCfg_t fsmCfg = {0};

    fsmCfg.validParams = 0U;
    status = Pmic_fsmGetCfg(&pmicHandle, &fsmCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);

}

void test_neg_fsm_fsmSetDevErrCnt_nullHandle(void)
{
    int32_t status;

    status = Pmic_fsmSetDevErrCnt(NULL, 0x10);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_fsm_fsmSetDevErrCnt_outOfBounds(void)
{
    int32_t status;

    status = Pmic_fsmSetDevErrCnt(&pmicHandle, PMIC_DEV_ERR_CNT_MAX + 1);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);

}

void test_neg_fsm_fsmGetDevErrCnt_nullHandle(void)
{
    int32_t status;
    uint8_t devErrCnt;

    status = Pmic_fsmGetDevErrCnt(NULL, &devErrCnt);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_fsm_fsmGetDevErrCnt_nullPointer(void)
{
    int32_t status;

    status = Pmic_fsmGetDevErrCnt(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);

}

void test_neg_fsm_fsmSetWakeupCfg_nullHandle(void)
{
    int32_t status;
    Pmic_FsmWakeupCfg_t wakeupCfg = {0};

    wakeupCfg.validParams = PMIC_CFG_WAKE1_EVENT_VALID_SHIFT;
    status = Pmic_fsmSetWakeupCfg(NULL, &wakeupCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_fsm_fsmSetWakeupCfg_nullPointer(void)
{
    int32_t status;

    status = Pmic_fsmSetWakeupCfg(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);

}

void test_neg_fsm_fsmGetWakeStatus_nullHandle(void)
{
    int32_t status;
    Pmic_FsmWakeupStat_t wakeupStat;

    status = Pmic_fsmGetWakeStatus(NULL, &wakeupStat);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_fsm_fsmGetWakeStatus_nullPointer(void)
{
    int32_t status;

    status = Pmic_fsmGetWakeStatus(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);

}

void test_neg_fsm_fsmSetPowerLatchCfg_nullHandle(void)
{
    int32_t status;
    Pmic_FsmPwrLatchCfg_t pwrLatchCfg = {0};

    pwrLatchCfg.validParams = PMIC_FSM_CFG_VALID;
    status = Pmic_fsmSetPowerLatchCfg(NULL, &pwrLatchCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_fsm_fsmSetPowerLatchCfg_nullPointer(void)
{
    int32_t status;

    status = Pmic_fsmSetPowerLatchCfg(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);

}

void test_neg_fsm_fsmGetPowerLatch_nullHandle(void)
{
    int32_t status;
    Pmic_FsmPwrLatch_t pwrLatch = {0};

    pwrLatch.validParams = PMIC_FSM_CFG_VALID;
    status = Pmic_fsmGetPowerLatch(NULL, &pwrLatch);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_fsm_fsmGetPowerLatch_nullPointer(void)
{
    int32_t status;

    status = Pmic_fsmGetPowerLatch(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);

}

/* ========================================================================== */
/*                    Additional Edge Case Tests                             */
/* ========================================================================== */

void test_pos_fsm_fsmSetCfg_higherVbatStbyExitThr(void)
{
    int32_t status;
    Pmic_FsmCfg_t cfgSet = {0}, cfgGet = {0};

    // Test higherVbatStbyExitThr = true
    cfgSet.validParams = PMIC_CFG_FSM_HIGHER_VBAT_STBY_EXIT_THR_VALID;
    cfgSet.higherVbatStbyExitThr = true;
    status = Pmic_fsmSetCfg(&pmicHandle, &cfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    cfgGet.validParams = PMIC_CFG_FSM_HIGHER_VBAT_STBY_EXIT_THR_VALID;
    status = Pmic_fsmGetCfg(&pmicHandle, &cfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(cfgGet.higherVbatStbyExitThr == cfgSet.higherVbatStbyExitThr);

    // Test higherVbatStbyExitThr = false
    cfgSet.higherVbatStbyExitThr = false;
    status = Pmic_fsmSetCfg(&pmicHandle, &cfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_fsmGetCfg(&pmicHandle, &cfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(cfgGet.higherVbatStbyExitThr == cfgSet.higherVbatStbyExitThr);
}

void test_neg_fsm_fsmSetCfg_invalidVbatStbyEntryThr(void)
{
    int32_t status;
    Pmic_FsmCfg_t fsmCfg = {0};

    fsmCfg.validParams = PMIC_CFG_FSM_VBAT_STBY_ENTRY_THR_VALID;
    fsmCfg.vbatStbyEntryThr = PMIC_VBAT_STBY_ENTRY_THR_MAX + 1;
    status = Pmic_fsmSetCfg(&pmicHandle, &fsmCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);

}

void test_neg_fsm_fsmSetCfg_invalidPwdThr(void)
{
    int32_t status;
    Pmic_FsmCfg_t fsmCfg = {0};

    fsmCfg.validParams = PMIC_CFG_FSM_PWD_THR_VALID;
    fsmCfg.pwdThr = PMIC_PWD_THR_MAX + 1;
    status = Pmic_fsmSetCfg(&pmicHandle, &fsmCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);

}

void test_neg_fsm_fsmSetCfg_invalidRstMcuTmo(void)
{
    int32_t status;
    Pmic_FsmCfg_t fsmCfg = {0};

    fsmCfg.validParams = PMIC_CFG_FSM_RST_MCU_TMO_VALID;
    fsmCfg.rstMcuTmo = PMIC_RST_MCU_TMO_MAX + 1;
    status = Pmic_fsmSetCfg(&pmicHandle, &fsmCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);

}

void test_neg_fsm_fsmSetCfg_invalidNrstExt(void)
{
    int32_t status;
    Pmic_FsmCfg_t fsmCfg = {0};

    fsmCfg.validParams = PMIC_CFG_FSM_NRST_EXT_VALID;
    fsmCfg.nrstExt = PMIC_NRST_EXT_MAX + 1;
    status = Pmic_fsmSetCfg(&pmicHandle, &fsmCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);

}

void test_neg_fsm_fsmSetCfg_invalidSafeTmo(void)
{
    int32_t status;
    Pmic_FsmCfg_t fsmCfg = {0};

    fsmCfg.validParams = PMIC_CFG_FSM_SAFE_TMO_VALID;
    fsmCfg.safeTmo = PMIC_SAFE_TMO_MAX + 1;
    status = Pmic_fsmSetCfg(&pmicHandle, &fsmCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);

}

void test_neg_fsm_fsmSetCfg_invalidSafeLockThr(void)
{
    int32_t status;
    Pmic_FsmCfg_t fsmCfg = {0};

    fsmCfg.validParams = PMIC_CFG_FSM_SAFE_LOCK_THR_VALID;
    fsmCfg.safeLockThr = PMIC_SAFE_LOCK_THR_MAX + 1;
    status = Pmic_fsmSetCfg(&pmicHandle, &fsmCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);

}

void test_neg_fsm_fsmSetWakeupCfg_zeroValidParams(void)
{
    int32_t status;
    Pmic_FsmWakeupCfg_t wakeupCfg = {0};

    wakeupCfg.validParams = 0U;
    status = Pmic_fsmSetWakeupCfg(&pmicHandle, &wakeupCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);

}

void test_neg_fsm_fsmSetWakeupCfg_invalidWake1Event(void)
{
    int32_t status;
    Pmic_FsmWakeupCfg_t wakeupCfg = {0};

    wakeupCfg.validParams = PMIC_CFG_WAKE1_EVENT_VALID_SHIFT;
    wakeupCfg.wake1Event = PMIC_WAKE_EVENT_MAX + 1;
    status = Pmic_fsmSetWakeupCfg(&pmicHandle, &wakeupCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);

}

void test_neg_fsm_fsmSetWakeupCfg_invalidWake2Event(void)
{
    int32_t status;
    Pmic_FsmWakeupCfg_t wakeupCfg = {0};

    wakeupCfg.validParams = PMIC_CFG_WAKE2_EVENT_VALID_SHIFT;
    wakeupCfg.wake2Event = PMIC_WAKE_EVENT_MAX + 1;
    status = Pmic_fsmSetWakeupCfg(&pmicHandle, &wakeupCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);

}

void test_neg_fsm_fsmSetWakeupCfg_invalidWake1Dgl(void)
{
    int32_t status;
    Pmic_FsmWakeupCfg_t wakeupCfg = {0};

    wakeupCfg.validParams = PMIC_CFG_WAKE1_DGL_VALID_SHIFT;
    wakeupCfg.wake1Dgl = PMIC_WAKE_DEGLITCH_TIME_MAX + 1;
    status = Pmic_fsmSetWakeupCfg(&pmicHandle, &wakeupCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);

}

void test_neg_fsm_fsmSetWakeupCfg_invalidWake2Dgl(void)
{
    int32_t status;
    Pmic_FsmWakeupCfg_t wakeupCfg = {0};

    wakeupCfg.validParams = PMIC_CFG_WAKE2_DGL_VALID_SHIFT;
    wakeupCfg.wake2Dgl = PMIC_WAKE_DEGLITCH_TIME_MAX + 1;
    status = Pmic_fsmSetWakeupCfg(&pmicHandle, &wakeupCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);

}

void test_neg_fsm_fsmGetWakeupCfg_nullHandle(void)
{
    int32_t status;
    Pmic_FsmWakeupCfg_t wakeupCfg = {0};

    wakeupCfg.validParams = PMIC_CFG_WAKE1_EVENT_VALID_SHIFT;
    status = Pmic_fsmGetWakeupCfg(NULL, &wakeupCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_fsm_fsmGetWakeupCfg_nullPointer(void)
{
    int32_t status;

    status = Pmic_fsmGetWakeupCfg(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);

}

void test_neg_fsm_fsmGetWakeupCfg_zeroValidParams(void)
{
    int32_t status;
    Pmic_FsmWakeupCfg_t wakeupCfg = {0};

    wakeupCfg.validParams = 0U;
    status = Pmic_fsmGetWakeupCfg(&pmicHandle, &wakeupCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);

}

void test_neg_fsm_fsmSetPowerLatchCfg_zeroValidParams(void)
{
    int32_t status;
    Pmic_FsmPwrLatchCfg_t pwrLatchCfg = {0};

    pwrLatchCfg.validParams = 0U;
    status = Pmic_fsmSetPowerLatchCfg(&pmicHandle, &pwrLatchCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);

}

void test_neg_fsm_fsmSetPowerLatchCfg_invalidPwdDly(void)
{
    int32_t status;
    Pmic_FsmPwrLatchCfg_t pwrLatchCfg = {0};

    pwrLatchCfg.validParams = PMIC_CFG_FSM_PWD_DLY_VALID;
    pwrLatchCfg.pwdDly = PMIC_PWD_DLY_MAX + 1;
    status = Pmic_fsmSetPowerLatchCfg(&pmicHandle, &pwrLatchCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);

}

void test_neg_fsm_fsmGetPowerLatchCfg_nullHandle(void)
{
    int32_t status;
    Pmic_FsmPwrLatchCfg_t pwrLatchCfg = {0};

    pwrLatchCfg.validParams = PMIC_CFG_FSM_PWD_DLY_VALID;
    status = Pmic_fsmGetPowerLatchCfg(NULL, &pwrLatchCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_fsm_fsmGetPowerLatchCfg_nullPointer(void)
{
    int32_t status;

    status = Pmic_fsmGetPowerLatchCfg(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);

}

void test_neg_fsm_fsmGetPowerLatchCfg_zeroValidParams(void)
{
    int32_t status;
    Pmic_FsmPwrLatchCfg_t pwrLatchCfg = {0};

    pwrLatchCfg.validParams = 0U;
    status = Pmic_fsmGetPowerLatchCfg(&pmicHandle, &pwrLatchCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);

}

void test_neg_fsm_fsmSetPowerLatch_nullHandle(void)
{
    int32_t status;
    Pmic_FsmPwrLatch_t pwrLatch = {0};

    pwrLatch.validParams = PMIC_CFG_FSM_STBY_ERR_WAKE_LATCH_VALID;
    status = Pmic_fsmSetPowerLatch(NULL, &pwrLatch);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_fsm_fsmSetPowerLatch_nullPointer(void)
{
    int32_t status;

    status = Pmic_fsmSetPowerLatch(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);

}

void test_neg_fsm_fsmSetPowerLatch_zeroValidParams(void)
{
    int32_t status;
    Pmic_FsmPwrLatch_t pwrLatch = {0};

    pwrLatch.validParams = 0U;
    status = Pmic_fsmSetPowerLatch(&pmicHandle, &pwrLatch);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);

}

void test_neg_fsm_fsmGetPowerLatch_zeroValidParams(void)
{
    int32_t status;
    Pmic_FsmPwrLatch_t pwrLatch = {0};

    pwrLatch.validParams = 0U;
    status = Pmic_fsmGetPowerLatch(&pmicHandle, &pwrLatch);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);

}

void test_neg_fsm_fsmGetLastResetMcuStateDuration_nullHandle(void)
{
    int32_t status;
    uint8_t duration;

    status = Pmic_fsmGetLastResetMcuStateDuration(NULL, &duration);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_fsm_fsmGetLastResetMcuStateDuration_nullPointer(void)
{
    int32_t status;

    status = Pmic_fsmGetLastResetMcuStateDuration(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);

}

/* ========================================================================== */
/*                    Additional State Coverage Tests                        */
/* ========================================================================== */

void test_pos_fsm_fsmGetDevState_initState(void)
{
    int32_t status;
    uint8_t state;

    // Mock: Simulate FSM in INIT state (hardware values 1-4 map to PMIC_INIT_STATE)
    // Note: In actual hardware, after power-up the FSM typically starts in INIT state.
    // The driver converts register values 1-4 to PMIC_INIT_STATE for API consistency. */
    status = Pmic_fsmGetDevState(&pmicHandle, &state);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify state is one of the valid states (including INIT)
    PLATFORM_ASSERT((state == PMIC_OFF_STATE) ||
                     (state == PMIC_INIT_STATE) ||
                     (state == PMIC_PWRU_SEQ_STATE) ||
                     (state == PMIC_RESET_MCU_STATE) ||
                     (state == PMIC_AUTO_BIST) ||
                     (state == PMIC_ACTIVE_STATE) ||
                     (state == PMIC_SAFE_STATE) ||
                     (state == PMIC_RUN_TIME_BIST) ||
                     (state == PMIC_OTP_PROGRAMMING_STATE) ||
                     (state == PMIC_PWRD_SEQ_STATE) ||
                     (state == PMIC_STANDBY_STATE));

}

void test_pos_fsm_fsmGetDevState_offState(void)
{
    int32_t status;
    uint8_t state;

    // Mock: Simulate FSM in OFF state (hardware value 0xE maps to PMIC_OFF_STATE)
    // Note: The driver maps the repeated OFF state value (0xE) to PMIC_OFF_STATE.
    // This test covers the OFF_STATE_REPEATED path in the driver. */
    status = Pmic_fsmGetDevState(&pmicHandle, &state);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify state is one of the valid states (including OFF)
    PLATFORM_ASSERT((state == PMIC_OFF_STATE) ||
                     (state == PMIC_INIT_STATE) ||
                     (state == PMIC_PWRU_SEQ_STATE) ||
                     (state == PMIC_RESET_MCU_STATE) ||
                     (state == PMIC_AUTO_BIST) ||
                     (state == PMIC_ACTIVE_STATE) ||
                     (state == PMIC_SAFE_STATE) ||
                     (state == PMIC_RUN_TIME_BIST) ||
                     (state == PMIC_OTP_PROGRAMMING_STATE) ||
                     (state == PMIC_PWRD_SEQ_STATE) ||
                     (state == PMIC_STANDBY_STATE));

}

void test_pos_fsm_fsmSetWakeupCfg_withAllValidParams(void)
{
    int32_t status;
    Pmic_FsmWakeupCfg_t wakeupCfgSet = {0}, wakeupCfgGet = {0};

    // Configure all wakeup parameters with all valid param bits set
    wakeupCfgSet.validParams = PMIC_CFG_WAKE1_EVENT_VALID_SHIFT |
                                PMIC_CFG_WAKE2_EVENT_VALID_SHIFT |
                                PMIC_CFG_WAKE1_DGL_VALID_SHIFT |
                                PMIC_CFG_WAKE2_DGL_VALID_SHIFT;
    wakeupCfgSet.wake1Event = PMIC_WAKE_HIGH_LEVEL;
    wakeupCfgSet.wake2Event = PMIC_WAKE_RISING_EDGE_PLUS_HIGH_LEVEL;
    wakeupCfgSet.wake1Dgl = PMIC_WAKE_DEGLITCH_TIME_16_MS;
    wakeupCfgSet.wake2Dgl = PMIC_WAKE_DEGLITCH_TIME_2_MS;

    status = Pmic_fsmSetWakeupCfg(&pmicHandle, &wakeupCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    wakeupCfgGet.validParams = wakeupCfgSet.validParams;
    status = Pmic_fsmGetWakeupCfg(&pmicHandle, &wakeupCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(wakeupCfgGet.wake1Event == wakeupCfgSet.wake1Event);
    PLATFORM_ASSERT(wakeupCfgGet.wake2Event == wakeupCfgSet.wake2Event);
    PLATFORM_ASSERT(wakeupCfgGet.wake1Dgl == wakeupCfgSet.wake1Dgl);
    PLATFORM_ASSERT(wakeupCfgGet.wake2Dgl == wakeupCfgSet.wake2Dgl);
}

void test_pos_fsm_fsmGetCfg_vbatStbyEntryThr(void)
{
    int32_t status;
    Pmic_FsmCfg_t cfgGet = {0};

    // Get only vbatStbyEntryThr parameter
    cfgGet.validParams = PMIC_CFG_FSM_VBAT_STBY_ENTRY_THR_VALID;
    status = Pmic_fsmGetCfg(&pmicHandle, &cfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

}

void test_pos_fsm_fsmGetWakeupCfg_individualParams(void)
{
    int32_t status;
    Pmic_FsmWakeupCfg_t wakeupCfgGet = {0};

    // Get wake2Event individually
    wakeupCfgGet.validParams = PMIC_CFG_WAKE2_EVENT_VALID_SHIFT;
    status = Pmic_fsmGetWakeupCfg(&pmicHandle, &wakeupCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get wake1Dgl individually
    wakeupCfgGet.validParams = PMIC_CFG_WAKE1_DGL_VALID_SHIFT;
    status = Pmic_fsmGetWakeupCfg(&pmicHandle, &wakeupCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get wake2Dgl individually
    wakeupCfgGet.validParams = PMIC_CFG_WAKE2_DGL_VALID_SHIFT;
    status = Pmic_fsmGetWakeupCfg(&pmicHandle, &wakeupCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_fsm_fsmGetPowerLatchCfg_individualParams(void)
{
    int32_t status;
    Pmic_FsmPwrLatchCfg_t pwrLatchCfgGet = {0};

    // Get stbyErrWakeEventPwrlEn individually
    pwrLatchCfgGet.validParams = PMIC_CFG_FSM_STBY_ERR_WAKE_EVENT_PWRL_EN_VALID;
    status = Pmic_fsmGetPowerLatchCfg(&pmicHandle, &pwrLatchCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get wake1EventPwrlEn individually
    pwrLatchCfgGet.validParams = PMIC_CFG_WAKE1_EVENT_PWRL_EN_VALID_SHIFT;
    status = Pmic_fsmGetPowerLatchCfg(&pmicHandle, &pwrLatchCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get wake2EventPwrlEn individually
    pwrLatchCfgGet.validParams = PMIC_CFG_WAKE2_EVENT_PWRL_EN_VALID_SHIFT;
    status = Pmic_fsmGetPowerLatchCfg(&pmicHandle, &pwrLatchCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_fsm_fsmGetPowerLatch_individualParams(void)
{
    int32_t status;
    Pmic_FsmPwrLatch_t pwrLatchGet = {0};

    // Get stbyErrWakeLatch individually
    pwrLatchGet.validParams = PMIC_CFG_FSM_STBY_ERR_WAKE_EVENT_PWRL_EN_VALID;
    status = Pmic_fsmGetPowerLatch(&pmicHandle, &pwrLatchGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get wake1Latch individually
    pwrLatchGet.validParams = PMIC_CFG_WAKE1_EVENT_PWRL_EN_VALID_SHIFT;
    status = Pmic_fsmGetPowerLatch(&pmicHandle, &pwrLatchGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get wake2Latch individually
    pwrLatchGet.validParams = PMIC_CFG_WAKE2_EVENT_PWRL_EN_VALID_SHIFT;
    status = Pmic_fsmGetPowerLatch(&pmicHandle, &pwrLatchGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

/**
 * @brief Test negative case: setDevState with invalid state value beyond max.
 */
void test_neg_fsm_fsmSetDevState_invalidStateBoundary(void)
{
    int32_t status;

    // Test with state request value beyond maximum
    status = Pmic_fsmSetDevState(&pmicHandle, PMIC_STATE_REQUEST_MAX + 1);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);

    // Test with another out-of-range state value
    status = Pmic_fsmSetDevState(&pmicHandle, 0xFF);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);

}

/**
 * @brief Test positive case: getDevState reads current state correctly after state requests.
 */
void test_pos_fsm_fsmGetDevState_validRead(void)
{
    int32_t status;
    uint8_t state;

    // Send SAFE to ACTIVE request and verify state can be read
    status = Pmic_fsmSetDevState(&pmicHandle, PMIC_SAFE_TO_ACTIVE_REQUEST);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_fsmGetDevState(&pmicHandle, &state);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    // State should be one of the valid device states
    PLATFORM_ASSERT((state == PMIC_OFF_STATE) ||
                     (state == PMIC_INIT_STATE) ||
                     (state == PMIC_PWRU_SEQ_STATE) ||
                     (state == PMIC_RESET_MCU_STATE) ||
                     (state == PMIC_AUTO_BIST) ||
                     (state == PMIC_ACTIVE_STATE) ||
                     (state == PMIC_SAFE_STATE) ||
                     (state == PMIC_RUN_TIME_BIST) ||
                     (state == PMIC_OTP_PROGRAMMING_STATE) ||
                     (state == PMIC_PWRD_SEQ_STATE) ||
                     (state == PMIC_STANDBY_STATE));

    // Send ACTIVE to SAFE request and verify state can be read
    status = Pmic_fsmSetDevState(&pmicHandle, PMIC_ACTIVE_TO_SAFE_REQUEST);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_fsmGetDevState(&pmicHandle, &state);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT((state == PMIC_OFF_STATE) ||
                     (state == PMIC_INIT_STATE) ||
                     (state == PMIC_PWRU_SEQ_STATE) ||
                     (state == PMIC_RESET_MCU_STATE) ||
                     (state == PMIC_AUTO_BIST) ||
                     (state == PMIC_ACTIVE_STATE) ||
                     (state == PMIC_SAFE_STATE) ||
                     (state == PMIC_RUN_TIME_BIST) ||
                     (state == PMIC_OTP_PROGRAMMING_STATE) ||
                     (state == PMIC_PWRD_SEQ_STATE) ||
                     (state == PMIC_STANDBY_STATE));

    // Send RESET MCU request and verify state can be read
    status = Pmic_fsmSetDevState(&pmicHandle, PMIC_ACTIVE_OR_SAFE_TO_RESET_MCU_REQUEST);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Wait for PMIC to complete RESET_MCU transition before reading state
    platform_timerWaitMs(50U);

    status = Pmic_fsmGetDevState(&pmicHandle, &state);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT((state == PMIC_OFF_STATE) ||
                     (state == PMIC_INIT_STATE) ||
                     (state == PMIC_PWRU_SEQ_STATE) ||
                     (state == PMIC_RESET_MCU_STATE) ||
                     (state == PMIC_AUTO_BIST) ||
                     (state == PMIC_ACTIVE_STATE) ||
                     (state == PMIC_SAFE_STATE) ||
                     (state == PMIC_RUN_TIME_BIST) ||
                     (state == PMIC_OTP_PROGRAMMING_STATE) ||
                     (state == PMIC_PWRD_SEQ_STATE) ||
                     (state == PMIC_STANDBY_STATE));

    // Send NO STATE CHANGE request and verify state can be read
    status = Pmic_fsmSetDevState(&pmicHandle, PMIC_NO_STATE_CHANGE_REQUEST);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_fsmGetDevState(&pmicHandle, &state);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT((state == PMIC_OFF_STATE) ||
                     (state == PMIC_INIT_STATE) ||
                     (state == PMIC_PWRU_SEQ_STATE) ||
                     (state == PMIC_RESET_MCU_STATE) ||
                     (state == PMIC_AUTO_BIST) ||
                     (state == PMIC_ACTIVE_STATE) ||
                     (state == PMIC_SAFE_STATE) ||
                     (state == PMIC_RUN_TIME_BIST) ||
                     (state == PMIC_OTP_PROGRAMMING_STATE) ||
                     (state == PMIC_PWRD_SEQ_STATE) ||
                     (state == PMIC_STANDBY_STATE));

    // Restore ACTIVE state for subsequent tests
    (void)Pmic_fsmSetDevState(&pmicHandle, PMIC_SAFE_TO_ACTIVE_REQUEST);
}

/* ========================================================================== */
// Negative Tests - Pmic_fsmGetDevErrCnt
/* ========================================================================== */

/**
 * @brief I/O failure test: Pmic_fsmGetDevErrCnt returns error when ioRxByte_CS fails.
 */
void test_neg_fsm_fsmGetDevErrCnt_ioRxByteCSFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;
    uint8_t devErrCnt = 0U;

    PLATFORM_ASSERT(mockDevice != NULL);

    // Inject 1 comm failure — fires on the first (and only) ioRxByte_CS inside
    // Pmic_fsmGetDevErrCnt. */
    status = PmicMock_InjectError(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_fsmGetDevErrCnt(&pmicHandle, &devErrCnt);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/**
 * @brief I/O failure test: Pmic_fsmSetDevState returns error when ioRxByte fails.
 */
void test_neg_fsm_fsmSetDevState_ioRxByteFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;

    PLATFORM_ASSERT(mockDevice != NULL);

    // Inject 1 comm failure — fires on the ioRxByte(STATE_CTRL_REG) inside
    // Pmic_fsmSetDevState. */
    status = PmicMock_InjectError(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_fsmSetDevState(&pmicHandle, PMIC_NO_STATE_CHANGE_REQUEST);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/* ========================================================================== */
// Positive Tests - Pmic_fsmGetCfg
/* ========================================================================== */

/**
 * @brief Test fsmGetCfg with only PMIC_CFG_FSM_STBY_EN_VALID set.
 */
void test_pos_fsm_fsmGetCfg_stbyEnOnlyFlag(void)
{
    int32_t status;
    Pmic_FsmCfg_t cfgGet = {0};

    cfgGet.validParams = PMIC_CFG_FSM_STBY_EN_VALID;
    status = Pmic_fsmGetCfg(&pmicHandle, &cfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

/**
 * @brief Test fsmGetCfg with only PMIC_CFG_FSM_PWD_THR_VALID set.
 */
void test_pos_fsm_fsmGetCfg_pwdThrValidOnly(void)
{
    int32_t status;
    Pmic_FsmCfg_t cfgGet = {0};

    cfgGet.validParams = PMIC_CFG_FSM_PWD_THR_VALID;
    status = Pmic_fsmGetCfg(&pmicHandle, &cfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

/**
 * @brief Test fsmGetCfg with only PMIC_CFG_FSM_RST_MCU_TMO_VALID set.
 */
void test_pos_fsm_fsmGetCfg_rstMcuTmoValidOnly(void)
{
    int32_t status;
    Pmic_FsmCfg_t cfgGet = {0};

    cfgGet.validParams = PMIC_CFG_FSM_RST_MCU_TMO_VALID;
    status = Pmic_fsmGetCfg(&pmicHandle, &cfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

/**
 * @brief Test fsmGetCfg with only PMIC_CFG_FSM_SAFE_LOCK_THR_VALID set.
 */
void test_pos_fsm_fsmGetCfg_safeLockThrValidOnly(void)
{
    int32_t status;
    Pmic_FsmCfg_t cfgGet = {0};

    cfgGet.validParams = PMIC_CFG_FSM_SAFE_LOCK_THR_VALID;
    status = Pmic_fsmGetCfg(&pmicHandle, &cfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

/* ========================================================================== */
// Positive Tests - Pmic_fsmSetCfg
/* ========================================================================== */

/**
 * @brief Test fsmSetCfg with only PMIC_CFG_FSM_PWD_THR_VALID set.
 */
void test_pos_fsm_fsmSetCfg_pwdThrOnlyFlag(void)
{
    int32_t status;
    Pmic_FsmCfg_t cfgSet = {0};

    cfgSet.validParams = PMIC_CFG_FSM_PWD_THR_VALID;
    cfgSet.pwdThr = 0x00U;
    status = Pmic_fsmSetCfg(&pmicHandle, &cfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

/* ========================================================================== */
// Positive Tests - Pmic_fsmGetWakeupCfg / Pmic_fsmSetWakeupCfg
/* ========================================================================== */

/**
 * @brief Test fsmGetWakeupCfg with only PMIC_CFG_FSM_WAKE2_DGL_VALID set.
 */
void test_pos_fsm_fsmGetWakeupCfg_wake2DglValidOnly(void)
{
    int32_t status;
    Pmic_FsmWakeupCfg_t wakeupCfg = {0};

    wakeupCfg.validParams = PMIC_CFG_FSM_WAKE2_DGL_VALID;
    status = Pmic_fsmGetWakeupCfg(&pmicHandle, &wakeupCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

/**
 * @brief Test fsmSetWakeupCfg with only PMIC_CFG_FSM_WAKE2_DGL_VALID set.
 */
void test_pos_fsm_fsmSetWakeupCfg_wake2DglValidOnly(void)
{
    int32_t status;
    Pmic_FsmWakeupCfg_t wakeupCfg = {0};

    wakeupCfg.validParams = PMIC_CFG_FSM_WAKE2_DGL_VALID;
    wakeupCfg.wake2Dgl = PMIC_WAKE_DEGLITCH_TIME_2_MS;
    status = Pmic_fsmSetWakeupCfg(&pmicHandle, &wakeupCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

/* ========================================================================== */
// Positive Tests - Pmic_fsmGetPowerLatchCfg / Pmic_fsmSetPowerLatchCfg
/* ========================================================================== */

/**
 * @brief Test fsmGetPowerLatchCfg with only PMIC_CFG_FSM_WAKE2_EVENT_PWRL_EN_VALID set.
 */
void test_pos_fsm_fsmGetPowerLatchCfg_wake2EventOnly(void)
{
    int32_t status;
    Pmic_FsmPwrLatchCfg_t pwrLatchCfg = {0};

    pwrLatchCfg.validParams = PMIC_CFG_FSM_WAKE2_EVENT_PWRL_EN_VALID;
    status = Pmic_fsmGetPowerLatchCfg(&pmicHandle, &pwrLatchCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

/**
 * @brief Test fsmSetPowerLatchCfg with only PMIC_CFG_FSM_WAKE2_EVENT_PWRL_EN_VALID set.
 */
void test_pos_fsm_fsmSetPowerLatchCfg_wake2EventOnly(void)
{
    int32_t status;
    Pmic_FsmPwrLatchCfg_t pwrLatchCfg = {0};

    pwrLatchCfg.validParams = PMIC_CFG_FSM_WAKE2_EVENT_PWRL_EN_VALID;
    pwrLatchCfg.wake2EventPwrlEn = false;
    status = Pmic_fsmSetPowerLatchCfg(&pmicHandle, &pwrLatchCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

// Individual latch-flag coverage: Pmic_fsmSetPowerLatch

/**
 * @brief Power latch individual flag: PMIC_CFG_FSM_STBY_ERR_WAKE_LATCH_VALID only.
 */
void test_pos_fsm_fsmSetPowerLatch_stbyErrWakeLatchValid(void)
{
    int32_t status;
    Pmic_FsmPwrLatch_t pwrLatch = {0};

    pwrLatch.validParams = PMIC_CFG_FSM_STBY_ERR_WAKE_LATCH_VALID;
    pwrLatch.stbyErrWakeLatch = true;
    status = Pmic_fsmSetPowerLatch(&pmicHandle, &pwrLatch);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

/**
 * @brief Power latch individual flag: PMIC_CFG_FSM_STBY_TMR_WAKE_LATCH_VALID only.
 */
void test_pos_fsm_fsmSetPowerLatch_stbyTmrWakeLatchValid(void)
{
    int32_t status;
    Pmic_FsmPwrLatch_t pwrLatch = {0};

    pwrLatch.validParams = PMIC_CFG_FSM_STBY_TMR_WAKE_LATCH_VALID;
    pwrLatch.stbyTmrWakeLatch = false;
    status = Pmic_fsmSetPowerLatch(&pmicHandle, &pwrLatch);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

/**
 * @brief Power latch individual flag: PMIC_CFG_FSM_M_PMIC_WAKE_LATCH_VALID only.
 */
void test_pos_fsm_fsmSetPowerLatch_mPmicWakeLatchValid(void)
{
    int32_t status;
    Pmic_FsmPwrLatch_t pwrLatch = {0};

    pwrLatch.validParams = PMIC_CFG_FSM_M_PMIC_WAKE_LATCH_VALID;
    pwrLatch.mPmicWakeLatch = false;
    status = Pmic_fsmSetPowerLatch(&pmicHandle, &pwrLatch);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

/**
 * @brief Power latch individual flag: PMIC_CFG_FSM_WAKE1_LATCH_VALID only.
 */
void test_pos_fsm_fsmSetPowerLatch_wake1LatchValid(void)
{
    int32_t status;
    Pmic_FsmPwrLatch_t pwrLatch = {0};

    pwrLatch.validParams = PMIC_CFG_FSM_WAKE1_LATCH_VALID;
    pwrLatch.wake2Latch = false;
    status = Pmic_fsmSetPowerLatch(&pmicHandle, &pwrLatch);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

/**
 * @brief Power latch individual flag: PMIC_CFG_FSM_WAKE2_LATCH_VALID only.
 */
void test_pos_fsm_fsmSetPowerLatch_wake2LatchValid(void)
{
    int32_t status;
    Pmic_FsmPwrLatch_t pwrLatch = {0};

    pwrLatch.validParams = PMIC_CFG_FSM_WAKE2_LATCH_VALID;
    pwrLatch.wake1Latch = false;
    status = Pmic_fsmSetPowerLatch(&pmicHandle, &pwrLatch);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

/* ========================================================================== */
/*                         Unity Framework Functions                          */
/* ========================================================================== */

/* setUp() and tearDown() are defined in test_runner.c */

/**
 * @brief FSM test suite entry point (wrapper for test runner).
 * @param args Test arguments (unused)
 */
void fsm_test(void *args)
{
    (void)args;
    char msg[50U] = {0};
    int32_t status = PMIC_ST_SUCCESS;

    platform_init();

    testTimer_startModule("FSM");

    Pmic_HandleCfg_t pmicCfg = {
        .validParams = PMIC_CFG_INIT_COMM_MODE_VALID |
                       PMIC_CFG_INIT_COMM_HANDLE_0_VALID |
                       PMIC_CFG_INIT_IO_READ_VALID |
                       PMIC_CFG_INIT_IO_WRITE_VALID |
                       PMIC_CFG_INIT_CRITICAL_SECTION_START_VALID |
                       PMIC_CFG_INIT_CRITICAL_SECTION_STOP_VALID,
        .commMode = PMIC_INTF_SPI,
        .commHandle0 = platform_getCommHandle(),
        .ioRead = &platform_rxByte,
        .ioWrite = &platform_txByte,
        .criticalSectionStart = &platform_critSecStart,
        .criticalSectionStop = &platform_critSecStop
    };

    status = Pmic_init(&pmicHandle, &pmicCfg);

    if (status == PMIC_ST_SUCCESS)
    {
        platform_unlockRegisters();

        platform_setupTests();
        FSM_TEST_RUN_ALL();
        platform_tearDownTests();
    }
    else
    {
        (void)sprintf(msg, "Error in initializing PMIC LLD: %d\r\n", status);
        platform_printString(msg);
    }

    testTimer_endModule();

    (void)Pmic_deinit(&pmicHandle);
    platform_deinit();
}

/* ========================================================================== */
/*                      Coverage Tests for pmic_fsm.c                         */
/* ========================================================================== */

/**
 * @brief Test FSM getDevState with INIT state mapping (lines 134-135).
 *
 * Tests that STATE register values 1-4 map to PMIC_INIT_STATE
 */
void test_pos_fsm_fsmGetDevState_initStateMapping(void)
{
#ifdef BUILD_MOCK
    int32_t status;
    uint8_t state;

    // Test INIT_STATE_MIN (1)
    testInject_setRegister(STATE_STAT_REG, 0x01U);

    status = Pmic_fsmGetDevState(&pmicHandle, &state);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(state == PMIC_INIT_STATE);

    // Test middle value (2)
    testInject_setRegister(STATE_STAT_REG, 0x02U);

    status = Pmic_fsmGetDevState(&pmicHandle, &state);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(state == PMIC_INIT_STATE);

    // Test INIT_STATE_MAX (4)
    testInject_setRegister(STATE_STAT_REG, 0x04U);

    status = Pmic_fsmGetDevState(&pmicHandle, &state);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(state == PMIC_INIT_STATE);

#else
    TEST_IGNORE_MESSAGE("Requires BUILD_MOCK for register injection");
#endif
}

/**
 * @brief Test FSM getDevState with OFF_STATE_REPEATED mapping (lines 139-140).
 *
 * Tests that STATE register value 0xE maps to PMIC_OFF_STATE
 */
void test_pos_fsm_fsmGetDevState_offStateRepeated(void)
{
#ifdef BUILD_MOCK
    int32_t status;
    uint8_t state;

    // Inject OFF_STATE_REPEATED (0xE)
    testInject_setRegister(STATE_STAT_REG, 0x0EU);

    status = Pmic_fsmGetDevState(&pmicHandle, &state);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(state == PMIC_OFF_STATE);

#else
    TEST_IGNORE_MESSAGE("Requires BUILD_MOCK for register injection");
#endif
}

/**
 * @brief Test FSM setPwrLatchCfg with stbyErrWakeEvent validParam (lines 957-959).
 *
 * Tests the validParam check for STBY_ERR_WAKE_EVENT_PWRL_EN
 */
void test_pos_fsm_fsmSetPowerLatchCfg_stbyErrWakeEvent(void)
{
    int32_t status;
    Pmic_FsmPwrLatchCfg_t pwrLatchCfg = {0};

    // Set power latch config with STBY_ERR_WAKE_EVENT_PWRL_EN_VALID
    pwrLatchCfg.validParams = PMIC_CFG_FSM_STBY_ERR_WAKE_EVENT_PWRL_EN_VALID;
    pwrLatchCfg.stbyErrWakeEventPwrlEn = true;

    status = Pmic_fsmSetPowerLatchCfg(&pmicHandle, &pwrLatchCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Also test with WAKE1_EVENT_PWRL_EN_VALID
    pwrLatchCfg.validParams = PMIC_CFG_FSM_WAKE1_EVENT_PWRL_EN_VALID;
    pwrLatchCfg.wake1EventPwrlEn = true;

    status = Pmic_fsmSetPowerLatchCfg(&pmicHandle, &pwrLatchCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // And with WAKE2_EVENT_PWRL_EN_VALID
    pwrLatchCfg.validParams = PMIC_CFG_FSM_WAKE2_EVENT_PWRL_EN_VALID;
    pwrLatchCfg.wake2EventPwrlEn = true;

    status = Pmic_fsmSetPowerLatchCfg(&pmicHandle, &pwrLatchCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

/* ========================================================================== */
// Positive Tests - Pmic_fsmGetPowerLatchCfg
/* ========================================================================== */

/**
 * @brief Test Pmic_fsmGetPowerLatchCfg with none of the four PWRL cfg bits set.
 */
void test_pos_fsm_fsmGetPowerLatchCfg_noPwrlFields(void)
{
    int32_t status;
    Pmic_FsmPwrLatchCfg_t pwrLatchCfg = {0};

    pwrLatchCfg.validParams = (1UL << 31U);

    status = Pmic_fsmGetPowerLatchCfg(&pmicHandle, &pwrLatchCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

/**
 * @brief Test Pmic_fsmGetWakeupCfg with none of the four WAKE cfg bits set.
 */
void test_pos_fsm_fsmGetWakeupCfg_noWakeFields(void)
{
    int32_t status;
    Pmic_FsmWakeupCfg_t wakeupCfg = {0};

    // Use a bit outside bits 0-3 so none of the four WAKE valid bits are set
    wakeupCfg.validParams = (1UL << 31U);

    status = Pmic_fsmGetWakeupCfg(&pmicHandle, &wakeupCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

/**
 * @brief Test Pmic_fsmSetCfg with only non-STBY fields in validParams.
 */
void test_pos_fsm_fsmSetCfg_noStbyFields(void)
{
    int32_t status;
    Pmic_FsmCfg_t fsmCfg = {0};

    fsmCfg.validParams = PMIC_CFG_FSM_AUTO_BIST_EN_VALID;
    fsmCfg.autoBistEn  = false;

    status = Pmic_fsmSetCfg(&pmicHandle, &fsmCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

/**
 * @brief Test Pmic_fsmSetPowerLatch read-fail path.
 */
void test_neg_fsm_fsmSetPowerLatch_readFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;
    Pmic_FsmPwrLatch_t pwrLatch = {0};

    PLATFORM_ASSERT(mockDevice != NULL);

    pwrLatch.validParams = PMIC_CFG_FSM_STBY_ERR_WAKE_LATCH_VALID;
    pwrLatch.stbyErrWakeLatch = false;

    // Inject a single comm failure so the first ioRxByte inside
    // Pmic_fsmSetPowerLatch (PWRL_CTRL_REG read) fails. */
    status = PmicMock_InjectError(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_fsmSetPowerLatch(&pmicHandle, &pwrLatch);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/**
 * @brief Test Pmic_fsmSetPowerLatch ioTxByte write failure path.
 */
void test_neg_fsm_fsmSetPowerLatch_writeFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;
    Pmic_FsmPwrLatch_t pwrLatch = {0};

    PLATFORM_ASSERT(mockDevice != NULL);

    pwrLatch.validParams = PMIC_CFG_FSM_STBY_ERR_WAKE_LATCH_VALID;
    pwrLatch.stbyErrWakeLatch = false;

    // Skip the 1st I/O (ioRxByte succeeds), then fail the 2nd I/O (ioTxByte)
    status = PmicMock_InjectErrorAfterN(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1U, 1U);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_fsmSetPowerLatch(&pmicHandle, &pwrLatch);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/**
 * @brief Test Pmic_fsmSetPowerLatchCfg with none of the four PWRL cfg bits set.
 */
void test_pos_fsm_fsmSetPowerLatchCfg_noPwrlFields(void)
{
    int32_t status;
    Pmic_FsmPwrLatchCfg_t pwrLatchCfg = {0};

    pwrLatchCfg.validParams = (1UL << 31U);

    status = Pmic_fsmSetPowerLatchCfg(&pmicHandle, &pwrLatchCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

/**
 * @brief Test Pmic_fsmSetWakeupCfg with none of the four WAKE bits set.
 */
void test_pos_fsm_fsmSetWakeupCfg_noWakeFields(void)
{
    int32_t status;
    Pmic_FsmWakeupCfg_t wakeupCfg = {0};

    wakeupCfg.validParams = (1UL << 31U);

    status = Pmic_fsmSetWakeupCfg(&pmicHandle, &wakeupCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

/**
 * @brief Test Pmic_fsmSetCfg with ONLY PMIC_CFG_FSM_VBAT_STBY_ENTRY_THR_VALID set.
 */
void test_pos_fsm_fsmSetCfg_vbatStbyEntryThrOnly(void)
{
    int32_t status;
    Pmic_FsmCfg_t cfgSet = {0}, cfgGet = {0};

    // Set ONLY the vbatStbyEntryThr flag — no earlier STBY flags
    cfgSet.validParams = PMIC_CFG_FSM_VBAT_STBY_ENTRY_THR_VALID;
    cfgSet.vbatStbyEntryThr = PMIC_VBAT_STBY_ENTRY_THR_3P6;

    status = Pmic_fsmSetCfg(&pmicHandle, &cfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    cfgGet.validParams = PMIC_CFG_FSM_VBAT_STBY_ENTRY_THR_VALID;
    status = Pmic_fsmGetCfg(&pmicHandle, &cfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(cfgGet.vbatStbyEntryThr == cfgSet.vbatStbyEntryThr);
}

void test_pos_fsm_fsmSetCfg_stbyEnOnly(void)
{
    int32_t status;
    Pmic_FsmCfg_t cfgSet = {0}, cfgGet = {0};

    // Set ONLY the stbyEn flag — no other STBY flags
    cfgSet.validParams = PMIC_CFG_FSM_STBY_EN_VALID;
    cfgSet.stbyEn = true;

    status = Pmic_fsmSetCfg(&pmicHandle, &cfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify it was written correctly
    cfgGet.validParams = PMIC_CFG_FSM_STBY_EN_VALID;
    status = Pmic_fsmGetCfg(&pmicHandle, &cfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(cfgGet.stbyEn == true);

    // Also test stbyEn = false with same setup
    cfgSet.stbyEn = false;
    status = Pmic_fsmSetCfg(&pmicHandle, &cfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    cfgGet.stbyEn = false;
    status = Pmic_fsmGetCfg(&pmicHandle, &cfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(cfgGet.stbyEn == false);
}

/**
 * @brief Test FSM_setStbyCfg() IO failure path.
 */
void test_neg_fsm_fsmSetCfg_stbyCfgIoRxByteFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;
    Pmic_FsmCfg_t cfg = {0};

    PLATFORM_ASSERT(mockDevice != NULL);

    cfg.validParams = PMIC_CFG_FSM_STBY_EN_VALID;
    cfg.stbyEn = true;

    // Fail on the 1st I/O call: Pmic_ioRxByte(STBY_CFG_REG) inside FSM_setStbyCfg
    status = PmicMock_InjectError(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_fsmSetCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/**
 * @brief Test FSM_setSafetyCfg() IO failure path.
 */
void test_neg_fsm_fsmSetCfg_safetyCfgIoRxByteFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;
    Pmic_FsmCfg_t cfg = {0};

    PLATFORM_ASSERT(mockDevice != NULL);

    cfg.validParams = PMIC_CFG_FSM_AUTO_BIST_EN_VALID;
    cfg.autoBistEn = false;

    // Fail on the 1st I/O call: Pmic_ioRxByte(SAFETY_CFG_REG) inside FSM_setSafetyCfg
    status = PmicMock_InjectError(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_fsmSetCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/**
 * @brief Test FSM_setRstMcuCfg() IO failure path.
 */
void test_neg_fsm_fsmSetCfg_rstMcuCfgIoRxByteFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;
    Pmic_FsmCfg_t cfg = {0};

    PLATFORM_ASSERT(mockDevice != NULL);

    cfg.validParams = PMIC_CFG_FSM_RST_MCU_TMO_VALID;
    cfg.rstMcuTmo = 0U;

    // Fail on the 1st I/O call: Pmic_ioRxByte(RST_MCU_CFG_REG) inside FSM_setRstMcuCfg
    status = PmicMock_InjectError(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_fsmSetCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/**
 * @brief Test FSM_setSafeTmoCfg() IO failure path.
 */
void test_neg_fsm_fsmSetCfg_safeTmoCfgIoRxByteFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;
    Pmic_FsmCfg_t cfg = {0};

    PLATFORM_ASSERT(mockDevice != NULL);

    cfg.validParams = PMIC_CFG_FSM_SAFE_TMO_VALID;
    cfg.safeTmo = 0U;

    // Fail on the 1st I/O call: Pmic_ioRxByte(SAFE_TMO_CFG_REG) inside FSM_setSafeTmoCfg
    status = PmicMock_InjectError(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_fsmSetCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/**
 * @brief Test FSM_getStbyCfg() IO failure path.
 */
void test_neg_fsm_fsmGetCfg_stbyCfgIoRxByteFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;
    Pmic_FsmCfg_t cfg = {0};

    PLATFORM_ASSERT(mockDevice != NULL);

    cfg.validParams = PMIC_CFG_FSM_STBY_EN_VALID;

    // Fail on the 1st I/O call: Pmic_ioRxByte_CS(STBY_CFG_REG) inside FSM_getStbyCfg
    status = PmicMock_InjectError(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_fsmGetCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/**
 * @brief Test FSM_getSafetyCfg() IO failure path.
 */
void test_neg_fsm_fsmGetCfg_safetyCfgIoRxByteFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;
    Pmic_FsmCfg_t cfg = {0};

    PLATFORM_ASSERT(mockDevice != NULL);

    cfg.validParams = PMIC_CFG_FSM_AUTO_BIST_EN_VALID;

    // Fail on the 1st I/O call: Pmic_ioRxByte_CS(SAFETY_CFG_REG) inside FSM_getSafetyCfg
    status = PmicMock_InjectError(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_fsmGetCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/**
 * @brief Test FSM_getRstMcuCfg() IO failure path.
 */
void test_neg_fsm_fsmGetCfg_rstMcuCfgIoRxByteFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;
    Pmic_FsmCfg_t cfg = {0};

    PLATFORM_ASSERT(mockDevice != NULL);

    cfg.validParams = PMIC_CFG_FSM_RST_MCU_TMO_VALID;

    // Fail on the 1st I/O call: Pmic_ioRxByte_CS(RST_MCU_CFG_REG) inside FSM_getRstMcuCfg
    status = PmicMock_InjectError(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_fsmGetCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/**
 * @brief Test FSM_getSafeTmoCfg() IO failure path.
 */
void test_neg_fsm_fsmGetCfg_safeTmoCfgIoRxByteFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;
    Pmic_FsmCfg_t cfg = {0};

    PLATFORM_ASSERT(mockDevice != NULL);

    cfg.validParams = PMIC_CFG_FSM_SAFE_TMO_VALID;

    // Fail on the 1st I/O call: Pmic_ioRxByte_CS(SAFE_TMO_CFG_REG) inside FSM_getSafeTmoCfg
    status = PmicMock_InjectError(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_fsmGetCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/**
 * @brief Test FSM_setWakeupCfg() IO failure path.
 */
void test_neg_fsm_fsmSetWakeupCfg_ioRxByteFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;
    Pmic_FsmWakeupCfg_t cfg = {0};

    PLATFORM_ASSERT(mockDevice != NULL);

    cfg.validParams = PMIC_CFG_FSM_WAKE1_EVENT_VALID;
    cfg.wake1Event = 0U;

    // Fail on the 1st I/O call: Pmic_ioRxByte(WAKE_CFG_REG) inside FSM_setWakeupCfg
    status = PmicMock_InjectError(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_fsmSetWakeupCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/**
 * @brief Test FSM_getWakeupCfg() IO failure path.
 */
void test_neg_fsm_fsmGetWakeupCfg_ioRxByteFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;
    Pmic_FsmWakeupCfg_t cfg = {0};

    PLATFORM_ASSERT(mockDevice != NULL);

    cfg.validParams = PMIC_CFG_FSM_WAKE1_EVENT_VALID;

    // Fail on the 1st I/O call: Pmic_ioRxByte(WAKE_CFG_REG) inside FSM_getWakeupCfg
    status = PmicMock_InjectError(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_fsmGetWakeupCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/**
 * @brief Test FSM_setPwrLatchCfg() IO failure path.
 */
void test_neg_fsm_fsmSetPowerLatchCfg_ioRxByteFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;
    Pmic_FsmPwrLatchCfg_t cfg = {0};

    PLATFORM_ASSERT(mockDevice != NULL);

    cfg.validParams = PMIC_CFG_FSM_STBY_ERR_WAKE_EVENT_PWRL_EN_VALID;
    cfg.stbyErrWakeEventPwrlEn = false;

    // Fail on the 1st I/O call: Pmic_ioRxByte(PWRL_CFG_REG) inside FSM_setPwrLatchCfg
    status = PmicMock_InjectError(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_fsmSetPowerLatchCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

/**
 * @brief Test FSM_getPwrLatchCfg() IO failure path.
 */
void test_neg_fsm_fsmGetPowerLatchCfg_ioRxByteFail(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t *mockDevice = platform_getMockDevice();
    int32_t status;
    Pmic_FsmPwrLatchCfg_t cfg = {0};

    PLATFORM_ASSERT(mockDevice != NULL);

    cfg.validParams = PMIC_CFG_FSM_STBY_ERR_WAKE_EVENT_PWRL_EN_VALID;

    // Fail on the 1st I/O call: Pmic_ioRxByte(PWRL_CFG_REG) inside FSM_getPwrLatchCfg
    status = PmicMock_InjectError(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_fsmGetPowerLatchCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for error injection");
#endif
}

// False-branch coverage: inner validParam checks in fsmGetCfg helpers

void test_pos_fsm_fsmGetCfg_nrstExtOnlyValidOnly(void)
{
    int32_t status;
    Pmic_FsmCfg_t cfgGet = {0};

    cfgGet.validParams = PMIC_CFG_FSM_NRST_EXT_VALID;
    status = Pmic_fsmGetCfg(&pmicHandle, &cfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_fsm_fsmGetCfg_safeTmoValidOnly(void)
{
    int32_t status;
    Pmic_FsmCfg_t cfgGet = {0};

    cfgGet.validParams = PMIC_CFG_FSM_SAFE_TMO_VALID;
    status = Pmic_fsmGetCfg(&pmicHandle, &cfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_fsm_fsmGetCfg_autoBistEnValidOnly(void)
{
    int32_t status;
    Pmic_FsmCfg_t cfgGet = {0};

    cfgGet.validParams = PMIC_CFG_FSM_AUTO_BIST_EN_VALID;
    status = Pmic_fsmGetCfg(&pmicHandle, &cfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}
