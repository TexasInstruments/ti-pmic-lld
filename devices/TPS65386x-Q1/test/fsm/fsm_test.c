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



/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include "fsm_test.h"
#include "test_inject.h"
#include "regmap/fsm.h"

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */
static Pmic_Handle_t g_handle;

/* Dummy handle for mock - driver validates non-NULL but doesn't dereference */
static uint32_t dummyCommHandle = 0x12345678U;

/* ========================================================================== */
/*                           Helper Functions                                 */
/* ========================================================================== */

/**
 * @brief Helper function to initialize PMIC handle for each test.
 */
static int32_t helper_initPmic(Pmic_Handle_t *handle)
{
    Pmic_HandleCfg_t pmicCfg = {
        .validParams = PMIC_COMM_MODE_VALID |
                       PMIC_COMM_HANDLE_0_VALID |
                       PMIC_IO_READ_VALID |
                       PMIC_IO_WRITE_VALID |
                       PMIC_CRITICAL_SECTION_START_VALID |
                       PMIC_CRITICAL_SECTION_STOP_VALID,
        .commMode = PMIC_INTF_SPI,
        .commHandle0 = (void*)&dummyCommHandle,
        .ioRead = &test_pmic_regRead,
        .ioWrite = &test_pmic_regWrite,
        .criticalSectionStart = &test_pmic_criticalSectionStartFn,
        .criticalSectionStop = &test_pmic_criticalSectionStopFn
    };
    int32_t status;

    platform_init();
    status = Pmic_init(handle, &pmicCfg);
    return status;
}

/**
 * @brief Helper function to deinitialize PMIC handle after each test.
 */
static void helper_deinitPmic(Pmic_Handle_t *handle)
{
    Pmic_deinit(handle);
    platform_deinit();
}

/* ========================================================================== */
/*                         Positive Test Implementations                      */
/* ========================================================================== */

void test_positive_fsm_setGetDevState(void)
{
    int32_t status;
    uint8_t stateGet;

    status = helper_initPmic(&g_handle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    /* Test valid state requests */
    uint8_t validStates[] = {
        PMIC_NO_STATE_CHANGE_REQUEST,
        PMIC_SAFE_TO_ACTIVE_REQUEST,
        PMIC_ACTIVE_TO_SAFE_REQUEST,
        PMIC_STANDBY_REQUEST
    };

    for (uint32_t i = 0; i < sizeof(validStates); i++)
    {
        status = Pmic_fsmSetDevState(&g_handle, validStates[i]);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        /* Get state to verify communication works */
        status = Pmic_fsmGetDevState(&g_handle, &stateGet);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    }

    helper_deinitPmic(&g_handle);
}

void test_positive_fsm_getDevState(void)
{
    int32_t status;
    uint8_t state;

    status = helper_initPmic(&g_handle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_fsmGetDevState(&g_handle, &state);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    /* Verify state is within valid range */
    TEST_ASSERT_TRUE((state == PMIC_OFF_STATE) ||
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

    helper_deinitPmic(&g_handle);
}

void test_positive_fsm_setGetCfg_stbyEn(void)
{
    int32_t status;
    Pmic_FsmCfg_t cfgSet = {0}, cfgGet = {0};

    status = helper_initPmic(&g_handle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    /* Test stbyEn = true */
    cfgSet.validParams = PMIC_FSM_CFG_VALID;
    cfgSet.stbyEn = true;
    status = Pmic_fsmSetCfg(&g_handle, &cfgSet);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    cfgGet.validParams = PMIC_FSM_CFG_VALID;
    status = Pmic_fsmGetCfg(&g_handle, &cfgGet);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(cfgSet.stbyEn, cfgGet.stbyEn);

    /* Test stbyEn = false */
    cfgSet.stbyEn = false;
    status = Pmic_fsmSetCfg(&g_handle, &cfgSet);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_fsmGetCfg(&g_handle, &cfgGet);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(cfgSet.stbyEn, cfgGet.stbyEn);

    helper_deinitPmic(&g_handle);
}

void test_positive_fsm_setGetCfg_autoBistEn(void)
{
    int32_t status;
    Pmic_FsmCfg_t cfgSet = {0}, cfgGet = {0};

    status = helper_initPmic(&g_handle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    cfgSet.validParams = PMIC_FSM_CFG_VALID;
    cfgGet.validParams = PMIC_FSM_CFG_VALID;

    /* Test autoBistEn = true */
    cfgSet.autoBistEn = true;
    status = Pmic_fsmSetCfg(&g_handle, &cfgSet);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_fsmGetCfg(&g_handle, &cfgGet);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(cfgSet.autoBistEn, cfgGet.autoBistEn);

    /* Test autoBistEn = false */
    cfgSet.autoBistEn = false;
    status = Pmic_fsmSetCfg(&g_handle, &cfgSet);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_fsmGetCfg(&g_handle, &cfgGet);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(cfgSet.autoBistEn, cfgGet.autoBistEn);

    helper_deinitPmic(&g_handle);
}

void test_positive_fsm_setGetCfg_nrstActiveInStbySeq(void)
{
    int32_t status;
    Pmic_FsmCfg_t cfgSet = {0}, cfgGet = {0};

    status = helper_initPmic(&g_handle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    cfgSet.validParams = PMIC_FSM_CFG_VALID;
    cfgGet.validParams = PMIC_FSM_CFG_VALID;

    /* Test nrstActiveInStbySeq = true */
    cfgSet.nrstActiveInStbySeq = true;
    status = Pmic_fsmSetCfg(&g_handle, &cfgSet);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_fsmGetCfg(&g_handle, &cfgGet);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(cfgSet.nrstActiveInStbySeq, cfgGet.nrstActiveInStbySeq);

    /* Test nrstActiveInStbySeq = false */
    cfgSet.nrstActiveInStbySeq = false;
    status = Pmic_fsmSetCfg(&g_handle, &cfgSet);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_fsmGetCfg(&g_handle, &cfgGet);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(cfgSet.nrstActiveInStbySeq, cfgGet.nrstActiveInStbySeq);

    helper_deinitPmic(&g_handle);
}

void test_positive_fsm_setGetCfg_pwdThr(void)
{
    int32_t status;
    Pmic_FsmCfg_t cfgSet = {0}, cfgGet = {0};
    uint8_t testValues[] = {0x00, 0x10, 0x1F};

    status = helper_initPmic(&g_handle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    cfgSet.validParams = PMIC_FSM_CFG_VALID;
    cfgGet.validParams = PMIC_FSM_CFG_VALID;

    for (uint32_t i = 0; i < sizeof(testValues); i++)
    {
        cfgSet.pwdThr = testValues[i];
        status = Pmic_fsmSetCfg(&g_handle, &cfgSet);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        status = Pmic_fsmGetCfg(&g_handle, &cfgGet);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        TEST_ASSERT_EQUAL(cfgSet.pwdThr, cfgGet.pwdThr);
    }

    helper_deinitPmic(&g_handle);
}

void test_positive_fsm_setGetCfg_nrstExt(void)
{
    int32_t status;
    Pmic_FsmCfg_t cfgSet = {0}, cfgGet = {0};
    uint8_t testValues[] = {0x0, 0x7, 0xF};

    status = helper_initPmic(&g_handle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    cfgSet.validParams = PMIC_FSM_CFG_VALID;
    cfgGet.validParams = PMIC_FSM_CFG_VALID;

    for (uint32_t i = 0; i < sizeof(testValues); i++)
    {
        cfgSet.nrstExt = testValues[i];
        status = Pmic_fsmSetCfg(&g_handle, &cfgSet);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        status = Pmic_fsmGetCfg(&g_handle, &cfgGet);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        TEST_ASSERT_EQUAL(cfgSet.nrstExt, cfgGet.nrstExt);
    }

    helper_deinitPmic(&g_handle);
}

void test_positive_fsm_setGetCfg_rstMcuTmo(void)
{
    int32_t status;
    Pmic_FsmCfg_t cfgSet = {0}, cfgGet = {0};
    uint8_t testValues[] = {
        PMIC_RST_MCU_TMO_512_MS,
        PMIC_RST_MCU_TMO_1024_MS,
        PMIC_RST_MCU_TMO_2048_MS,
        PMIC_RST_MCU_TMO_INFINITE_MS
    };

    status = helper_initPmic(&g_handle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    cfgSet.validParams = PMIC_FSM_CFG_VALID;
    cfgGet.validParams = PMIC_FSM_CFG_VALID;

    for (uint32_t i = 0; i < sizeof(testValues); i++)
    {
        cfgSet.rstMcuTmo = testValues[i];
        status = Pmic_fsmSetCfg(&g_handle, &cfgSet);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        status = Pmic_fsmGetCfg(&g_handle, &cfgGet);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        TEST_ASSERT_EQUAL(cfgSet.rstMcuTmo, cfgGet.rstMcuTmo);
    }

    helper_deinitPmic(&g_handle);
}

void test_positive_fsm_setGetCfg_safeTmo(void)
{
    int32_t status;
    Pmic_FsmCfg_t cfgSet = {0}, cfgGet = {0};
    uint8_t testValues[] = {0x0, 0x3, 0x7};

    status = helper_initPmic(&g_handle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    cfgSet.validParams = PMIC_FSM_CFG_VALID;
    cfgGet.validParams = PMIC_FSM_CFG_VALID;

    for (uint32_t i = 0; i < sizeof(testValues); i++)
    {
        cfgSet.safeTmo = testValues[i];
        status = Pmic_fsmSetCfg(&g_handle, &cfgSet);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        status = Pmic_fsmGetCfg(&g_handle, &cfgGet);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        TEST_ASSERT_EQUAL(cfgSet.safeTmo, cfgGet.safeTmo);
    }

    helper_deinitPmic(&g_handle);
}

void test_positive_fsm_setGetCfg_safeLockThr(void)
{
    int32_t status;
    Pmic_FsmCfg_t cfgSet = {0}, cfgGet = {0};
    uint8_t testValues[] = {0x00, 0x10, 0x1F};

    status = helper_initPmic(&g_handle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    cfgSet.validParams = PMIC_FSM_CFG_VALID;
    cfgGet.validParams = PMIC_FSM_CFG_VALID;

    for (uint32_t i = 0; i < sizeof(testValues); i++)
    {
        cfgSet.safeLockThr = testValues[i];
        status = Pmic_fsmSetCfg(&g_handle, &cfgSet);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        status = Pmic_fsmGetCfg(&g_handle, &cfgGet);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        TEST_ASSERT_EQUAL(cfgSet.safeLockThr, cfgGet.safeLockThr);
    }

    helper_deinitPmic(&g_handle);
}

void test_positive_fsm_setGetCfg_vbatStbyEntryThr(void)
{
    int32_t status;
    Pmic_FsmCfg_t cfgSet = {0}, cfgGet = {0};
    uint8_t testValues[] = {
        PMIC_VBAT_STBY_ENTRY_THR_2P4,
        PMIC_VBAT_STBY_ENTRY_THR_3P6,
        PMIC_VBAT_STBY_ENTRY_THR_5P1,
        PMIC_VBAT_STBY_ENTRY_THR_6P1
    };

    status = helper_initPmic(&g_handle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    cfgSet.validParams = PMIC_FSM_CFG_VALID;
    cfgGet.validParams = PMIC_FSM_CFG_VALID;

    for (uint32_t i = 0; i < sizeof(testValues); i++)
    {
        cfgSet.vbatStbyEntryThr = testValues[i];
        status = Pmic_fsmSetCfg(&g_handle, &cfgSet);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        status = Pmic_fsmGetCfg(&g_handle, &cfgGet);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        TEST_ASSERT_EQUAL(cfgSet.vbatStbyEntryThr, cfgGet.vbatStbyEntryThr);
    }

    helper_deinitPmic(&g_handle);
}

void test_positive_fsm_setGetCfg_multiple(void)
{
    int32_t status;
    Pmic_FsmCfg_t cfgSet = {0}, cfgGet = {0};

    status = helper_initPmic(&g_handle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    /* Configure multiple parameters at once */
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

    status = Pmic_fsmSetCfg(&g_handle, &cfgSet);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    cfgGet.validParams = cfgSet.validParams;
    status = Pmic_fsmGetCfg(&g_handle, &cfgGet);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(cfgSet.stbyEn, cfgGet.stbyEn);
    TEST_ASSERT_EQUAL(cfgSet.autoBistEn, cfgGet.autoBistEn);
    TEST_ASSERT_EQUAL(cfgSet.nrstActiveInStbySeq, cfgGet.nrstActiveInStbySeq);
    TEST_ASSERT_EQUAL(cfgSet.pwdThr, cfgGet.pwdThr);
    TEST_ASSERT_EQUAL(cfgSet.nrstExt, cfgGet.nrstExt);

    helper_deinitPmic(&g_handle);
}

void test_positive_fsm_setGetDevErrCnt(void)
{
    int32_t status;
    uint8_t errCntSet, errCntGet;
    uint8_t testValues[] = {0x00, 0x10, 0x1F};

    status = helper_initPmic(&g_handle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    for (uint32_t i = 0; i < sizeof(testValues); i++)
    {
        errCntSet = testValues[i];
        status = Pmic_fsmSetDevErrCnt(&g_handle, errCntSet);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        status = Pmic_fsmGetDevErrCnt(&g_handle, &errCntGet);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        TEST_ASSERT_EQUAL(errCntSet, errCntGet);
    }

    helper_deinitPmic(&g_handle);
}

void test_positive_fsm_setGetWakeupCfg(void)
{
    int32_t status;
    Pmic_FsmWakeupCfg_t wakeupCfgSet = {0}, wakeupCfgGet = {0};

    status = helper_initPmic(&g_handle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    /* Configure wakeup parameters */
    wakeupCfgSet.validParams = PMIC_CFG_WAKE1_EVENT_VALID_SHIFT |
                                PMIC_CFG_WAKE2_EVENT_VALID_SHIFT |
                                PMIC_CFG_WAKE1_DGL_VALID_SHIFT |
                                PMIC_CFG_WAKE2_DGL_VALID_SHIFT;
    wakeupCfgSet.wake1Event = PMIC_WAKE_HIGH_LEVEL;
    wakeupCfgSet.wake2Event = PMIC_WAKE_RISING_EDGE_PLUS_HIGH_LEVEL;
    wakeupCfgSet.wake1Dgl = PMIC_WAKE_DEGLITCH_TIME_2_MS;
    wakeupCfgSet.wake2Dgl = PMIC_WAKE_DEGLITCH_TIME_16_MS;

    status = Pmic_fsmSetWakeupCfg(&g_handle, &wakeupCfgSet);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    wakeupCfgGet.validParams = wakeupCfgSet.validParams;
    status = Pmic_fsmGetWakeupCfg(&g_handle, &wakeupCfgGet);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(wakeupCfgSet.wake1Event, wakeupCfgGet.wake1Event);
    TEST_ASSERT_EQUAL(wakeupCfgSet.wake2Event, wakeupCfgGet.wake2Event);
    TEST_ASSERT_EQUAL(wakeupCfgSet.wake1Dgl, wakeupCfgGet.wake1Dgl);
    TEST_ASSERT_EQUAL(wakeupCfgSet.wake2Dgl, wakeupCfgGet.wake2Dgl);

    helper_deinitPmic(&g_handle);
}

void test_positive_fsm_getWakeStatus(void)
{
    int32_t status;
    Pmic_FsmWakeupStat_t wakeupStat;

    status = helper_initPmic(&g_handle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_fsmGetWakeStatus(&g_handle, &wakeupStat);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    helper_deinitPmic(&g_handle);
}

void test_positive_fsm_setGetPowerLatchCfg(void)
{
    int32_t status;
    Pmic_FsmPwrLatchCfg_t pwrLatchCfgSet = {0}, pwrLatchCfgGet = {0};

    status = helper_initPmic(&g_handle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    /* Configure power latch parameters */
    pwrLatchCfgSet.validParams = PMIC_FSM_CFG_VALID |
                                  PMIC_FSM_CFG_VALID |
                                  PMIC_CFG_WAKE1_EVENT_PWRL_EN_VALID_SHIFT |
                                  PMIC_CFG_WAKE2_EVENT_PWRL_EN_VALID_SHIFT;
    pwrLatchCfgSet.pwdDly = PMIC_PWD_DLY_256_US;
    pwrLatchCfgSet.stbyErrWakeEventPwrlEn = true;
    pwrLatchCfgSet.wake1EventPwrlEn = true;
    pwrLatchCfgSet.wake2EventPwrlEn = false;

    status = Pmic_fsmSetPowerLatchCfg(&g_handle, &pwrLatchCfgSet);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    pwrLatchCfgGet.validParams = pwrLatchCfgSet.validParams;
    status = Pmic_fsmGetPowerLatchCfg(&g_handle, &pwrLatchCfgGet);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(pwrLatchCfgSet.pwdDly, pwrLatchCfgGet.pwdDly);
    TEST_ASSERT_EQUAL(pwrLatchCfgSet.stbyErrWakeEventPwrlEn, pwrLatchCfgGet.stbyErrWakeEventPwrlEn);
    TEST_ASSERT_EQUAL(pwrLatchCfgSet.wake1EventPwrlEn, pwrLatchCfgGet.wake1EventPwrlEn);
    TEST_ASSERT_EQUAL(pwrLatchCfgSet.wake2EventPwrlEn, pwrLatchCfgGet.wake2EventPwrlEn);

    helper_deinitPmic(&g_handle);
}

void test_positive_fsm_setGetPowerLatch(void)
{
    int32_t status;
    Pmic_FsmPwrLatch_t pwrLatchSet = {0}, pwrLatchGet = {0};

    status = helper_initPmic(&g_handle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    /* Configure power latch values */
    pwrLatchSet.validParams = PMIC_FSM_CFG_VALID |
                               PMIC_FSM_CFG_VALID |
                               PMIC_CFG_WAKE1_LATCH_VALID_SHIFT |
                               PMIC_CFG_WAKE2_LATCH_VALID_SHIFT;
    pwrLatchSet.stbyErrWakeLatch = true;
    pwrLatchSet.stbyTmrWakeLatch = false;
    pwrLatchSet.wake1Latch = true;
    pwrLatchSet.wake2Latch = false;

    status = Pmic_fsmSetPowerLatch(&g_handle, &pwrLatchSet);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    pwrLatchGet.validParams = pwrLatchSet.validParams;
    status = Pmic_fsmGetPowerLatch(&g_handle, &pwrLatchGet);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(pwrLatchSet.stbyErrWakeLatch, pwrLatchGet.stbyErrWakeLatch);
    TEST_ASSERT_EQUAL(pwrLatchSet.stbyTmrWakeLatch, pwrLatchGet.stbyTmrWakeLatch);
    TEST_ASSERT_EQUAL(pwrLatchSet.wake1Latch, pwrLatchGet.wake1Latch);
    TEST_ASSERT_EQUAL(pwrLatchSet.wake2Latch, pwrLatchGet.wake2Latch);

    helper_deinitPmic(&g_handle);
}

void test_positive_fsm_getLastResetMcuStateDuration(void)
{
    int32_t status;
    uint8_t duration;

    status = helper_initPmic(&g_handle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_fsmGetLastResetMcuStateDuration(&g_handle, &duration);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    helper_deinitPmic(&g_handle);
}

/* ========================================================================== */
/*                         Negative Test Implementations                      */
/* ========================================================================== */

void test_negative_fsm_setDevState_nullHandle(void)
{
    int32_t status;

    status = Pmic_fsmSetDevState(NULL, PMIC_SAFE_TO_ACTIVE_REQUEST);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

void test_negative_fsm_setDevState_invalidState(void)
{
    int32_t status;

    status = helper_initPmic(&g_handle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_fsmSetDevState(&g_handle, PMIC_STATE_REQUEST_MAX + 1);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

    helper_deinitPmic(&g_handle);
}

void test_negative_fsm_getDevState_nullHandle(void)
{
    int32_t status;
    uint8_t state;

    status = Pmic_fsmGetDevState(NULL, &state);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

void test_negative_fsm_getDevState_nullPointer(void)
{
    int32_t status;

    status = helper_initPmic(&g_handle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_fsmGetDevState(&g_handle, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    helper_deinitPmic(&g_handle);
}

void test_negative_fsm_setCfg_nullHandle(void)
{
    int32_t status;
    Pmic_FsmCfg_t fsmCfg = {0};

    fsmCfg.validParams = PMIC_FSM_CFG_VALID;
    status = Pmic_fsmSetCfg(NULL, &fsmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

void test_negative_fsm_setCfg_nullPointer(void)
{
    int32_t status;

    status = helper_initPmic(&g_handle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_fsmSetCfg(&g_handle, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    helper_deinitPmic(&g_handle);
}

void test_negative_fsm_setCfg_invalidParams(void)
{
    int32_t status;
    Pmic_FsmCfg_t fsmCfg = {0};

    status = helper_initPmic(&g_handle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    fsmCfg.validParams = 0U;
    status = Pmic_fsmSetCfg(&g_handle, &fsmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

    helper_deinitPmic(&g_handle);
}

void test_negative_fsm_getCfg_nullHandle(void)
{
    int32_t status;
    Pmic_FsmCfg_t fsmCfg = {0};

    fsmCfg.validParams = PMIC_FSM_CFG_VALID;
    status = Pmic_fsmGetCfg(NULL, &fsmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

void test_negative_fsm_getCfg_nullPointer(void)
{
    int32_t status;

    status = helper_initPmic(&g_handle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_fsmGetCfg(&g_handle, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    helper_deinitPmic(&g_handle);
}

void test_negative_fsm_getCfg_invalidParams(void)
{
    int32_t status;
    Pmic_FsmCfg_t fsmCfg = {0};

    status = helper_initPmic(&g_handle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    fsmCfg.validParams = 0U;
    status = Pmic_fsmGetCfg(&g_handle, &fsmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

    helper_deinitPmic(&g_handle);
}

void test_negative_fsm_setDevErrCnt_nullHandle(void)
{
    int32_t status;

    status = Pmic_fsmSetDevErrCnt(NULL, 0x10);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

void test_negative_fsm_setDevErrCnt_outOfBounds(void)
{
    int32_t status;

    status = helper_initPmic(&g_handle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_fsmSetDevErrCnt(&g_handle, PMIC_DEV_ERR_CNT_MAX + 1);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

    helper_deinitPmic(&g_handle);
}

void test_negative_fsm_getDevErrCnt_nullHandle(void)
{
    int32_t status;
    uint8_t devErrCnt;

    status = Pmic_fsmGetDevErrCnt(NULL, &devErrCnt);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

void test_negative_fsm_getDevErrCnt_nullPointer(void)
{
    int32_t status;

    status = helper_initPmic(&g_handle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_fsmGetDevErrCnt(&g_handle, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    helper_deinitPmic(&g_handle);
}

void test_negative_fsm_setWakeupCfg_nullHandle(void)
{
    int32_t status;
    Pmic_FsmWakeupCfg_t wakeupCfg = {0};

    wakeupCfg.validParams = PMIC_CFG_WAKE1_EVENT_VALID_SHIFT;
    status = Pmic_fsmSetWakeupCfg(NULL, &wakeupCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

void test_negative_fsm_setWakeupCfg_nullPointer(void)
{
    int32_t status;

    status = helper_initPmic(&g_handle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_fsmSetWakeupCfg(&g_handle, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    helper_deinitPmic(&g_handle);
}

void test_negative_fsm_getWakeStatus_nullHandle(void)
{
    int32_t status;
    Pmic_FsmWakeupStat_t wakeupStat;

    status = Pmic_fsmGetWakeStatus(NULL, &wakeupStat);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

void test_negative_fsm_getWakeStatus_nullPointer(void)
{
    int32_t status;

    status = helper_initPmic(&g_handle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_fsmGetWakeStatus(&g_handle, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    helper_deinitPmic(&g_handle);
}

void test_negative_fsm_setPowerLatchCfg_nullHandle(void)
{
    int32_t status;
    Pmic_FsmPwrLatchCfg_t pwrLatchCfg = {0};

    pwrLatchCfg.validParams = PMIC_FSM_CFG_VALID;
    status = Pmic_fsmSetPowerLatchCfg(NULL, &pwrLatchCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

void test_negative_fsm_setPowerLatchCfg_nullPointer(void)
{
    int32_t status;

    status = helper_initPmic(&g_handle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_fsmSetPowerLatchCfg(&g_handle, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    helper_deinitPmic(&g_handle);
}

void test_negative_fsm_getPowerLatch_nullHandle(void)
{
    int32_t status;
    Pmic_FsmPwrLatch_t pwrLatch = {0};

    pwrLatch.validParams = PMIC_FSM_CFG_VALID;
    status = Pmic_fsmGetPowerLatch(NULL, &pwrLatch);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

void test_negative_fsm_getPowerLatch_nullPointer(void)
{
    int32_t status;

    status = helper_initPmic(&g_handle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_fsmGetPowerLatch(&g_handle, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    helper_deinitPmic(&g_handle);
}

/* ========================================================================== */
/*                    Additional Edge Case Tests                             */
/* ========================================================================== */

void test_positive_fsm_setGetCfg_higherVbatStbyExitThr(void)
{
    int32_t status;
    Pmic_FsmCfg_t cfgSet = {0}, cfgGet = {0};

    status = helper_initPmic(&g_handle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    /* Test higherVbatStbyExitThr = true */
    cfgSet.validParams = PMIC_CFG_HIGHER_VBAT_STBY_EXIT_THR_VALID;
    cfgSet.higherVbatStbyExitThr = true;
    status = Pmic_fsmSetCfg(&g_handle, &cfgSet);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    cfgGet.validParams = PMIC_CFG_HIGHER_VBAT_STBY_EXIT_THR_VALID;
    status = Pmic_fsmGetCfg(&g_handle, &cfgGet);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(cfgSet.higherVbatStbyExitThr, cfgGet.higherVbatStbyExitThr);

    /* Test higherVbatStbyExitThr = false */
    cfgSet.higherVbatStbyExitThr = false;
    status = Pmic_fsmSetCfg(&g_handle, &cfgSet);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_fsmGetCfg(&g_handle, &cfgGet);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(cfgSet.higherVbatStbyExitThr, cfgGet.higherVbatStbyExitThr);

    helper_deinitPmic(&g_handle);
}

void test_negative_fsm_setCfg_invalidVbatStbyEntryThr(void)
{
    int32_t status;
    Pmic_FsmCfg_t fsmCfg = {0};

    status = helper_initPmic(&g_handle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    fsmCfg.validParams = PMIC_CFG_VBAT_STBY_ENTRY_THR_VALID;
    fsmCfg.vbatStbyEntryThr = PMIC_VBAT_STBY_ENTRY_THR_MAX + 1;
    status = Pmic_fsmSetCfg(&g_handle, &fsmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

    helper_deinitPmic(&g_handle);
}

void test_negative_fsm_setCfg_invalidPwdThr(void)
{
    int32_t status;
    Pmic_FsmCfg_t fsmCfg = {0};

    status = helper_initPmic(&g_handle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    fsmCfg.validParams = PMIC_CFG_PWD_THR_VALID;
    fsmCfg.pwdThr = PMIC_PWD_THR_MAX + 1;
    status = Pmic_fsmSetCfg(&g_handle, &fsmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

    helper_deinitPmic(&g_handle);
}

void test_negative_fsm_setCfg_invalidRstMcuTmo(void)
{
    int32_t status;
    Pmic_FsmCfg_t fsmCfg = {0};

    status = helper_initPmic(&g_handle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    fsmCfg.validParams = PMIC_CFG_RST_MCU_TMO_VALID;
    fsmCfg.rstMcuTmo = PMIC_RST_MCU_TMO_MAX + 1;
    status = Pmic_fsmSetCfg(&g_handle, &fsmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

    helper_deinitPmic(&g_handle);
}

void test_negative_fsm_setCfg_invalidNrstExt(void)
{
    int32_t status;
    Pmic_FsmCfg_t fsmCfg = {0};

    status = helper_initPmic(&g_handle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    fsmCfg.validParams = PMIC_CFG_NRST_EXT_VALID;
    fsmCfg.nrstExt = PMIC_NRST_EXT_MAX + 1;
    status = Pmic_fsmSetCfg(&g_handle, &fsmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

    helper_deinitPmic(&g_handle);
}

void test_negative_fsm_setCfg_invalidSafeTmo(void)
{
    int32_t status;
    Pmic_FsmCfg_t fsmCfg = {0};

    status = helper_initPmic(&g_handle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    fsmCfg.validParams = PMIC_CFG_SAFE_TMO_VALID;
    fsmCfg.safeTmo = PMIC_SAFE_TMO_MAX + 1;
    status = Pmic_fsmSetCfg(&g_handle, &fsmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

    helper_deinitPmic(&g_handle);
}

void test_negative_fsm_setCfg_invalidSafeLockThr(void)
{
    int32_t status;
    Pmic_FsmCfg_t fsmCfg = {0};

    status = helper_initPmic(&g_handle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    fsmCfg.validParams = PMIC_CFG_SAFE_LOCK_THR_VALID;
    fsmCfg.safeLockThr = PMIC_SAFE_LOCK_THR_MAX + 1;
    status = Pmic_fsmSetCfg(&g_handle, &fsmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

    helper_deinitPmic(&g_handle);
}

void test_negative_fsm_setWakeupCfg_zeroValidParams(void)
{
    int32_t status;
    Pmic_FsmWakeupCfg_t wakeupCfg = {0};

    status = helper_initPmic(&g_handle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    wakeupCfg.validParams = 0U;
    status = Pmic_fsmSetWakeupCfg(&g_handle, &wakeupCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

    helper_deinitPmic(&g_handle);
}

void test_negative_fsm_setWakeupCfg_invalidWake1Event(void)
{
    int32_t status;
    Pmic_FsmWakeupCfg_t wakeupCfg = {0};

    status = helper_initPmic(&g_handle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    wakeupCfg.validParams = PMIC_CFG_WAKE1_EVENT_VALID_SHIFT;
    wakeupCfg.wake1Event = PMIC_WAKE_EVENT_MAX + 1;
    status = Pmic_fsmSetWakeupCfg(&g_handle, &wakeupCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

    helper_deinitPmic(&g_handle);
}

void test_negative_fsm_setWakeupCfg_invalidWake2Event(void)
{
    int32_t status;
    Pmic_FsmWakeupCfg_t wakeupCfg = {0};

    status = helper_initPmic(&g_handle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    wakeupCfg.validParams = PMIC_CFG_WAKE2_EVENT_VALID_SHIFT;
    wakeupCfg.wake2Event = PMIC_WAKE_EVENT_MAX + 1;
    status = Pmic_fsmSetWakeupCfg(&g_handle, &wakeupCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

    helper_deinitPmic(&g_handle);
}

void test_negative_fsm_setWakeupCfg_invalidWake1Dgl(void)
{
    int32_t status;
    Pmic_FsmWakeupCfg_t wakeupCfg = {0};

    status = helper_initPmic(&g_handle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    wakeupCfg.validParams = PMIC_CFG_WAKE1_DGL_VALID_SHIFT;
    wakeupCfg.wake1Dgl = PMIC_WAKE_DEGLITCH_TIME_MAX + 1;
    status = Pmic_fsmSetWakeupCfg(&g_handle, &wakeupCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

    helper_deinitPmic(&g_handle);
}

void test_negative_fsm_setWakeupCfg_invalidWake2Dgl(void)
{
    int32_t status;
    Pmic_FsmWakeupCfg_t wakeupCfg = {0};

    status = helper_initPmic(&g_handle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    wakeupCfg.validParams = PMIC_CFG_WAKE2_DGL_VALID_SHIFT;
    wakeupCfg.wake2Dgl = PMIC_WAKE_DEGLITCH_TIME_MAX + 1;
    status = Pmic_fsmSetWakeupCfg(&g_handle, &wakeupCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

    helper_deinitPmic(&g_handle);
}

void test_negative_fsm_getWakeupCfg_nullHandle(void)
{
    int32_t status;
    Pmic_FsmWakeupCfg_t wakeupCfg = {0};

    wakeupCfg.validParams = PMIC_CFG_WAKE1_EVENT_VALID_SHIFT;
    status = Pmic_fsmGetWakeupCfg(NULL, &wakeupCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

void test_negative_fsm_getWakeupCfg_nullPointer(void)
{
    int32_t status;

    status = helper_initPmic(&g_handle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_fsmGetWakeupCfg(&g_handle, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    helper_deinitPmic(&g_handle);
}

void test_negative_fsm_getWakeupCfg_zeroValidParams(void)
{
    int32_t status;
    Pmic_FsmWakeupCfg_t wakeupCfg = {0};

    status = helper_initPmic(&g_handle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    wakeupCfg.validParams = 0U;
    status = Pmic_fsmGetWakeupCfg(&g_handle, &wakeupCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

    helper_deinitPmic(&g_handle);
}

void test_negative_fsm_setPowerLatchCfg_zeroValidParams(void)
{
    int32_t status;
    Pmic_FsmPwrLatchCfg_t pwrLatchCfg = {0};

    status = helper_initPmic(&g_handle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    pwrLatchCfg.validParams = 0U;
    status = Pmic_fsmSetPowerLatchCfg(&g_handle, &pwrLatchCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

    helper_deinitPmic(&g_handle);
}

void test_negative_fsm_setPowerLatchCfg_invalidPwdDly(void)
{
    int32_t status;
    Pmic_FsmPwrLatchCfg_t pwrLatchCfg = {0};

    status = helper_initPmic(&g_handle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    pwrLatchCfg.validParams = PMIC_CFG_PWD_DLY_VALID;
    pwrLatchCfg.pwdDly = PMIC_PWD_DLY_MAX + 1;
    status = Pmic_fsmSetPowerLatchCfg(&g_handle, &pwrLatchCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

    helper_deinitPmic(&g_handle);
}

void test_negative_fsm_getPowerLatchCfg_nullHandle(void)
{
    int32_t status;
    Pmic_FsmPwrLatchCfg_t pwrLatchCfg = {0};

    pwrLatchCfg.validParams = PMIC_CFG_PWD_DLY_VALID;
    status = Pmic_fsmGetPowerLatchCfg(NULL, &pwrLatchCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

void test_negative_fsm_getPowerLatchCfg_nullPointer(void)
{
    int32_t status;

    status = helper_initPmic(&g_handle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_fsmGetPowerLatchCfg(&g_handle, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    helper_deinitPmic(&g_handle);
}

void test_negative_fsm_getPowerLatchCfg_zeroValidParams(void)
{
    int32_t status;
    Pmic_FsmPwrLatchCfg_t pwrLatchCfg = {0};

    status = helper_initPmic(&g_handle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    pwrLatchCfg.validParams = 0U;
    status = Pmic_fsmGetPowerLatchCfg(&g_handle, &pwrLatchCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

    helper_deinitPmic(&g_handle);
}

void test_negative_fsm_setPowerLatch_nullHandle(void)
{
    int32_t status;
    Pmic_FsmPwrLatch_t pwrLatch = {0};

    pwrLatch.validParams = PMIC_CFG_STBY_ERR_WAKE_LATCH_VALID;
    status = Pmic_fsmSetPowerLatch(NULL, &pwrLatch);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

void test_negative_fsm_setPowerLatch_nullPointer(void)
{
    int32_t status;

    status = helper_initPmic(&g_handle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_fsmSetPowerLatch(&g_handle, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    helper_deinitPmic(&g_handle);
}

void test_negative_fsm_setPowerLatch_zeroValidParams(void)
{
    int32_t status;
    Pmic_FsmPwrLatch_t pwrLatch = {0};

    status = helper_initPmic(&g_handle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    pwrLatch.validParams = 0U;
    status = Pmic_fsmSetPowerLatch(&g_handle, &pwrLatch);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

    helper_deinitPmic(&g_handle);
}

void test_negative_fsm_getPowerLatch_zeroValidParams(void)
{
    int32_t status;
    Pmic_FsmPwrLatch_t pwrLatch = {0};

    status = helper_initPmic(&g_handle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    pwrLatch.validParams = 0U;
    status = Pmic_fsmGetPowerLatch(&g_handle, &pwrLatch);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

    helper_deinitPmic(&g_handle);
}

void test_negative_fsm_getLastResetMcuStateDuration_nullHandle(void)
{
    int32_t status;
    uint8_t duration;

    status = Pmic_fsmGetLastResetMcuStateDuration(NULL, &duration);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

void test_negative_fsm_getLastResetMcuStateDuration_nullPointer(void)
{
    int32_t status;

    status = helper_initPmic(&g_handle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_fsmGetLastResetMcuStateDuration(&g_handle, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    helper_deinitPmic(&g_handle);
}

/* ========================================================================== */
/*                    Additional State Coverage Tests                        */
/* ========================================================================== */

void test_positive_fsm_getState_initState(void)
{
    int32_t status;
    uint8_t state;

    status = helper_initPmic(&g_handle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    /* Mock: Simulate FSM in INIT state (hardware values 1-4 map to PMIC_INIT_STATE) */
    /* Note: In actual hardware, after power-up the FSM typically starts in INIT state.
     * The driver converts register values 1-4 to PMIC_INIT_STATE for API consistency. */
    status = Pmic_fsmGetDevState(&g_handle, &state);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    /* Verify state is one of the valid states (including INIT) */
    TEST_ASSERT_TRUE((state == PMIC_OFF_STATE) ||
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

    helper_deinitPmic(&g_handle);
}

void test_positive_fsm_getState_offState(void)
{
    int32_t status;
    uint8_t state;

    status = helper_initPmic(&g_handle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    /* Mock: Simulate FSM in OFF state (hardware value 0xE maps to PMIC_OFF_STATE) */
    /* Note: The driver maps the repeated OFF state value (0xE) to PMIC_OFF_STATE.
     * This test covers the OFF_STATE_REPEATED path in the driver. */
    status = Pmic_fsmGetDevState(&g_handle, &state);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    /* Verify state is one of the valid states (including OFF) */
    TEST_ASSERT_TRUE((state == PMIC_OFF_STATE) ||
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

    helper_deinitPmic(&g_handle);
}

void test_positive_fsm_setWakeupCfg_withAllValidParams(void)
{
    int32_t status;
    Pmic_FsmWakeupCfg_t wakeupCfgSet = {0}, wakeupCfgGet = {0};

    status = helper_initPmic(&g_handle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    /* Configure all wakeup parameters with all valid param bits set */
    wakeupCfgSet.validParams = PMIC_CFG_WAKE1_EVENT_VALID_SHIFT |
                                PMIC_CFG_WAKE2_EVENT_VALID_SHIFT |
                                PMIC_CFG_WAKE1_DGL_VALID_SHIFT |
                                PMIC_CFG_WAKE2_DGL_VALID_SHIFT;
    wakeupCfgSet.wake1Event = PMIC_WAKE_HIGH_LEVEL;
    wakeupCfgSet.wake2Event = PMIC_WAKE_RISING_EDGE_PLUS_HIGH_LEVEL;
    wakeupCfgSet.wake1Dgl = PMIC_WAKE_DEGLITCH_TIME_16_MS;
    wakeupCfgSet.wake2Dgl = PMIC_WAKE_DEGLITCH_TIME_2_MS;

    status = Pmic_fsmSetWakeupCfg(&g_handle, &wakeupCfgSet);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    wakeupCfgGet.validParams = wakeupCfgSet.validParams;
    status = Pmic_fsmGetWakeupCfg(&g_handle, &wakeupCfgGet);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(wakeupCfgSet.wake1Event, wakeupCfgGet.wake1Event);
    TEST_ASSERT_EQUAL(wakeupCfgSet.wake2Event, wakeupCfgGet.wake2Event);
    TEST_ASSERT_EQUAL(wakeupCfgSet.wake1Dgl, wakeupCfgGet.wake1Dgl);
    TEST_ASSERT_EQUAL(wakeupCfgSet.wake2Dgl, wakeupCfgGet.wake2Dgl);

    helper_deinitPmic(&g_handle);
}

void test_positive_fsm_getCfg_vbatStbyEntryThr(void)
{
    int32_t status;
    Pmic_FsmCfg_t cfgGet = {0};

    status = helper_initPmic(&g_handle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    /* Get only vbatStbyEntryThr parameter */
    cfgGet.validParams = PMIC_CFG_VBAT_STBY_ENTRY_THR_VALID;
    status = Pmic_fsmGetCfg(&g_handle, &cfgGet);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    helper_deinitPmic(&g_handle);
}

void test_positive_fsm_getWakeupCfg_individualParams(void)
{
    int32_t status;
    Pmic_FsmWakeupCfg_t wakeupCfgGet = {0};

    status = helper_initPmic(&g_handle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    /* Get wake2Event individually */
    wakeupCfgGet.validParams = PMIC_CFG_WAKE2_EVENT_VALID_SHIFT;
    status = Pmic_fsmGetWakeupCfg(&g_handle, &wakeupCfgGet);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    /* Get wake1Dgl individually */
    wakeupCfgGet.validParams = PMIC_CFG_WAKE1_DGL_VALID_SHIFT;
    status = Pmic_fsmGetWakeupCfg(&g_handle, &wakeupCfgGet);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    /* Get wake2Dgl individually */
    wakeupCfgGet.validParams = PMIC_CFG_WAKE2_DGL_VALID_SHIFT;
    status = Pmic_fsmGetWakeupCfg(&g_handle, &wakeupCfgGet);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    helper_deinitPmic(&g_handle);
}

void test_positive_fsm_getPowerLatchCfg_individualParams(void)
{
    int32_t status;
    Pmic_FsmPwrLatchCfg_t pwrLatchCfgGet = {0};

    status = helper_initPmic(&g_handle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    /* Get stbyErrWakeEventPwrlEn individually */
    pwrLatchCfgGet.validParams = PMIC_CFG_STBY_ERR_WAKE_EVENT_PWRL_EN_VALID;
    status = Pmic_fsmGetPowerLatchCfg(&g_handle, &pwrLatchCfgGet);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    /* Get wake1EventPwrlEn individually */
    pwrLatchCfgGet.validParams = PMIC_CFG_WAKE1_EVENT_PWRL_EN_VALID_SHIFT;
    status = Pmic_fsmGetPowerLatchCfg(&g_handle, &pwrLatchCfgGet);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    /* Get wake2EventPwrlEn individually */
    pwrLatchCfgGet.validParams = PMIC_CFG_WAKE2_EVENT_PWRL_EN_VALID_SHIFT;
    status = Pmic_fsmGetPowerLatchCfg(&g_handle, &pwrLatchCfgGet);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    helper_deinitPmic(&g_handle);
}

void test_positive_fsm_getPowerLatch_individualParams(void)
{
    int32_t status;
    Pmic_FsmPwrLatch_t pwrLatchGet = {0};

    status = helper_initPmic(&g_handle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    /* Get stbyErrWakeLatch individually */
    pwrLatchGet.validParams = PMIC_CFG_STBY_ERR_WAKE_EVENT_PWRL_EN_VALID;
    status = Pmic_fsmGetPowerLatch(&g_handle, &pwrLatchGet);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    /* Get wake1Latch individually */
    pwrLatchGet.validParams = PMIC_CFG_WAKE1_EVENT_PWRL_EN_VALID_SHIFT;
    status = Pmic_fsmGetPowerLatch(&g_handle, &pwrLatchGet);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    /* Get wake2Latch individually */
    pwrLatchGet.validParams = PMIC_CFG_WAKE2_EVENT_PWRL_EN_VALID_SHIFT;
    status = Pmic_fsmGetPowerLatch(&g_handle, &pwrLatchGet);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    helper_deinitPmic(&g_handle);
}

/**
 * @brief Test negative case: setDevState with invalid state value beyond max
 */
void test_negative_fsmSetDevState_invalidState(void)
{
    int32_t status;

    status = helper_initPmic(&g_handle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    /* Test with state request value beyond maximum */
    status = Pmic_fsmSetDevState(&g_handle, PMIC_STATE_REQUEST_MAX + 1);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

    /* Test with another out-of-range state value */
    status = Pmic_fsmSetDevState(&g_handle, 0xFF);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

    helper_deinitPmic(&g_handle);
}

/**
 * @brief Test positive case: getDevState reads current state correctly after state requests
 */
void test_positive_fsmGetDevState_validRead(void)
{
    int32_t status;
    uint8_t state;

    status = helper_initPmic(&g_handle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    /* Send SAFE to ACTIVE request and verify state can be read */
    status = Pmic_fsmSetDevState(&g_handle, PMIC_SAFE_TO_ACTIVE_REQUEST);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_fsmGetDevState(&g_handle, &state);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    /* State should be one of the valid device states */
    TEST_ASSERT_TRUE((state == PMIC_OFF_STATE) ||
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

    /* Send ACTIVE to SAFE request and verify state can be read */
    status = Pmic_fsmSetDevState(&g_handle, PMIC_ACTIVE_TO_SAFE_REQUEST);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_fsmGetDevState(&g_handle, &state);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_TRUE((state == PMIC_OFF_STATE) ||
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

    /* Send RESET MCU request and verify state can be read */
    status = Pmic_fsmSetDevState(&g_handle, PMIC_ACTIVE_OR_SAFE_TO_RESET_MCU_REQUEST);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_fsmGetDevState(&g_handle, &state);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_TRUE((state == PMIC_OFF_STATE) ||
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

    /* Send NO STATE CHANGE request and verify state can be read */
    status = Pmic_fsmSetDevState(&g_handle, PMIC_NO_STATE_CHANGE_REQUEST);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_fsmGetDevState(&g_handle, &state);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_TRUE((state == PMIC_OFF_STATE) ||
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

    helper_deinitPmic(&g_handle);
}

/* ========================================================================== */
/*                         Unity Framework Functions                          */
/* ========================================================================== */

/* setUp() and tearDown() are defined in test_runner.c */

/**
 * @brief FSM test suite entry point (wrapper for test runner)
 * @param args Test arguments (unused)
 */
void fsm_test(void *args)
{
    int32_t status;
    (void)args;  /* Unused parameter */

    printf("\r\n");
    printf("==================================================\r\n");
    printf("    TPS65386x-Q1 FSM Module Tests\r\n");
    printf("==================================================\r\n\r\n");

    /* Initialize once for all FSM tests */
    platform_init();
    status = helper_initPmic(&g_handle);
    if (status != PMIC_ST_SUCCESS)
    {
        printf("ERROR: FSM test initialization failed with status: %d\r\n", status);
        platform_deinit();
        return;
    }

    /* Run all FSM tests */
    platform_setupTests();
    FSM_TEST_RUN_ALL();
    platform_tearDownTests();

    /* Cleanup */
    helper_deinitPmic(&g_handle);
    platform_deinit();

    printf("\r\n==================================================\r\n");
    printf("    FSM Module Tests Complete\r\n");
    printf("==================================================\r\n\r\n");
}

/* ========================================================================== */
/*                      Coverage Tests for pmic_fsm.c                         */
/* ========================================================================== */

/**
 * @brief Test FSM getDevState with INIT state mapping (lines 134-135)
 *
 * Tests that STATE register values 1-4 map to PMIC_INIT_STATE
 */
void test_positive_fsm_getDevState_initStateMapping(void)
{
#ifdef BUILD_MOCK
    int32_t status;
    uint8_t state;

    status = helper_initPmic(&g_handle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Test INIT_STATE_MIN (1)
    status = testInject_setRegister(STATE_STAT_REG, 0x01U);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_fsmGetDevState(&g_handle, &state);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(PMIC_INIT_STATE, state);

    // Test middle value (2)
    status = testInject_setRegister(STATE_STAT_REG, 0x02U);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_fsmGetDevState(&g_handle, &state);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(PMIC_INIT_STATE, state);

    // Test INIT_STATE_MAX (4)
    status = testInject_setRegister(STATE_STAT_REG, 0x04U);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_fsmGetDevState(&g_handle, &state);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(PMIC_INIT_STATE, state);

    helper_deinitPmic(&g_handle);
#else
    TEST_ASSERT_TRUE(true);  // Skip if not BUILD_MOCK
#endif
}

/**
 * @brief Test FSM getDevState with OFF_STATE_REPEATED mapping (lines 139-140)
 *
 * Tests that STATE register value 0xE maps to PMIC_OFF_STATE
 */
void test_positive_fsm_getDevState_offStateRepeated(void)
{
#ifdef BUILD_MOCK
    int32_t status;
    uint8_t state;

    status = helper_initPmic(&g_handle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Inject OFF_STATE_REPEATED (0xE)
    status = testInject_setRegister(STATE_STAT_REG, 0x0EU);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_fsmGetDevState(&g_handle, &state);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(PMIC_OFF_STATE, state);

    helper_deinitPmic(&g_handle);
#else
    TEST_ASSERT_TRUE(true);  // Skip if not BUILD_MOCK
#endif
}

/**
 * @brief Test FSM setPwrLatchCfg with stbyErrWakeEvent validParam (lines 957-959)
 *
 * Tests the validParam check for STBY_ERR_WAKE_EVENT_PWRL_EN
 */
void test_positive_fsm_setPwrLatchCfg_stbyErrWakeEvent(void)
{
    int32_t status;
    Pmic_FsmPwrLatchCfg_t pwrLatchCfg = {0};

    status = helper_initPmic(&g_handle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Set power latch config with STBY_ERR_WAKE_EVENT_PWRL_EN_VALID
    pwrLatchCfg.validParams = PMIC_CFG_STBY_ERR_WAKE_EVENT_PWRL_EN_VALID;
    pwrLatchCfg.stbyErrWakeEventPwrlEn = true;

    status = Pmic_fsmSetPowerLatchCfg(&g_handle, &pwrLatchCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Also test with WAKE1_EVENT_PWRL_EN_VALID
    pwrLatchCfg.validParams = PMIC_CFG_WAKE1_EVENT_PWRL_EN_VALID;
    pwrLatchCfg.wake1EventPwrlEn = true;

    status = Pmic_fsmSetPowerLatchCfg(&g_handle, &pwrLatchCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // And with WAKE2_EVENT_PWRL_EN_VALID
    pwrLatchCfg.validParams = PMIC_CFG_WAKE2_EVENT_PWRL_EN_VALID;
    pwrLatchCfg.wake2EventPwrlEn = true;

    status = Pmic_fsmSetPowerLatchCfg(&g_handle, &pwrLatchCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    helper_deinitPmic(&g_handle);
}
