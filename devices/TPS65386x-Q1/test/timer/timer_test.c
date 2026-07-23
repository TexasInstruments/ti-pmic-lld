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

#include "timer_test.h"
#include "test_constants.h"

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/* ========================================================================== */

/*                             Global Variables                               */
/* ========================================================================== */

static Pmic_Handle_t pmicHandle = {0U};

/* ========================================================================== */
/*                           Function Definitions                             */
/* ========================================================================== */

/**
 * @brief Helper function to unlock CNT registers before accessing timer/counter.
 *
 * The TPS65386x-Q1 has CNT register write-access lock enabled by default.
 * Timer counter registers (TMR_CNT0/1/2) require this lock to be disabled
 * before they can be written (including hardware auto-clear via TMR_CLR).
 */
static void unlockCntRegisters(void)
{
    int32_t status = Pmic_setCntLockState(&pmicHandle, PMIC_LOCK_DISABLE);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

/* ========================================================================== */
/*                         POSITIVE TEST CASES                                */
/* ========================================================================== */

/* Configuration Tests - Positive (11 tests) */

/**
 * @brief Test setting and getting prescale to 64.64 microseconds.
 */
void test_pos_timer_timerSetCfg_prescale64us(void)
{
    int32_t status;
    Pmic_TimerCfg_t setCfg, getCfg;

    /* Set prescale to 64.64us */
    setCfg.validParams = PMIC_CFG_TMR_PRESCALE_VALID;
    setCfg.prescale = PMIC_TMR_PRESCALE_64P64_US;
    status = Pmic_timerSetCfg(&pmicHandle, &setCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Get and verify */
    getCfg.validParams = PMIC_CFG_TMR_PRESCALE_VALID;
    status = Pmic_timerGetCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(getCfg.prescale == PMIC_TMR_PRESCALE_64P64_US);
}

/**
 * @brief Test setting and getting prescale to 16.384 milliseconds.
 */
void test_pos_timer_timerSetCfg_prescale16ms(void)
{
    int32_t status;
    Pmic_TimerCfg_t setCfg, getCfg;

    /* Set prescale to 16.384ms */
    setCfg.validParams = PMIC_CFG_TMR_PRESCALE_VALID;
    setCfg.prescale = PMIC_TMR_PRESCALE_16P384_MS;
    status = Pmic_timerSetCfg(&pmicHandle, &setCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Get and verify */
    getCfg.validParams = PMIC_CFG_TMR_PRESCALE_VALID;
    status = Pmic_timerGetCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(getCfg.prescale == PMIC_TMR_PRESCALE_16P384_MS);
}

/**
 * @brief Test setting and getting prescale to 131.072 milliseconds.
 */
void test_pos_timer_timerSetCfg_prescale131ms(void)
{
    int32_t status;
    Pmic_TimerCfg_t setCfg, getCfg;

    /* Set prescale to 131.072ms */
    setCfg.validParams = PMIC_CFG_TMR_PRESCALE_VALID;
    setCfg.prescale = PMIC_TMR_PRESCALE_131P072_MS;
    status = Pmic_timerSetCfg(&pmicHandle, &setCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Get and verify */
    getCfg.validParams = PMIC_CFG_TMR_PRESCALE_VALID;
    status = Pmic_timerGetCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(getCfg.prescale == PMIC_TMR_PRESCALE_131P072_MS);
}

/**
 * @brief Test setting and getting prescale to 1049 milliseconds.
 */
void test_pos_timer_timerSetCfg_prescale1049ms(void)
{
    int32_t status;
    Pmic_TimerCfg_t setCfg, getCfg;

    /* Set prescale to 1049ms */
    setCfg.validParams = PMIC_CFG_TMR_PRESCALE_VALID;
    setCfg.prescale = PMIC_TMR_PRESCALE_1049_MS;
    status = Pmic_timerSetCfg(&pmicHandle, &setCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Get and verify */
    getCfg.validParams = PMIC_CFG_TMR_PRESCALE_VALID;
    status = Pmic_timerGetCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(getCfg.prescale == PMIC_TMR_PRESCALE_1049_MS);
}

/**
 * @brief Test setting and getting timer mode to stopped.
 */
void test_pos_timer_timerSetCfg_modeStopped(void)
{
    int32_t status;
    Pmic_TimerCfg_t setCfg, getCfg;

    /* Set mode to stopped */
    setCfg.validParams = PMIC_CFG_TMR_MODE_VALID;
    setCfg.mode = PMIC_TMR_MODE_STOPPED;
    status = Pmic_timerSetCfg(&pmicHandle, &setCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Get and verify */
    getCfg.validParams = PMIC_CFG_TMR_MODE_VALID;
    status = Pmic_timerGetCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(getCfg.mode == PMIC_TMR_MODE_STOPPED);
}

/**
 * @brief Test setting and getting timer mode to operating and sequencing.
 */
void test_pos_timer_timerSetCfg_modeOperSeq(void)
{
    int32_t status;
    Pmic_TimerCfg_t setCfg, getCfg;

    /* Set mode to OPER_SEQ */
    setCfg.validParams = PMIC_CFG_TMR_MODE_VALID;
    setCfg.mode = PMIC_TMR_MODE_OPER_SEQ;
    status = Pmic_timerSetCfg(&pmicHandle, &setCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Get and verify */
    getCfg.validParams = PMIC_CFG_TMR_MODE_VALID;
    status = Pmic_timerGetCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(getCfg.mode == PMIC_TMR_MODE_OPER_SEQ);
}

/**
 * @brief Test setting and getting timer mode to STANDBY only.
 */
void test_pos_timer_timerSetCfg_modeStdby(void)
{
    int32_t status;
    Pmic_TimerCfg_t setCfg, getCfg;

    /* Set mode to STDBY */
    setCfg.validParams = PMIC_CFG_TMR_MODE_VALID;
    setCfg.mode = PMIC_TMR_MODE_STDBY;
    status = Pmic_timerSetCfg(&pmicHandle, &setCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Get and verify */
    getCfg.validParams = PMIC_CFG_TMR_MODE_VALID;
    status = Pmic_timerGetCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(getCfg.mode == PMIC_TMR_MODE_STDBY);
}

/**
 * @brief Test setting and getting timer mode to STANDBY with wakeup.
 */
void test_pos_timer_timerSetCfg_modeStdbyWu(void)
{
    int32_t status;
    Pmic_TimerCfg_t setCfg, getCfg;

    /* Set mode to STDBY_WU */
    setCfg.validParams = PMIC_CFG_TMR_MODE_VALID;
    setCfg.mode = PMIC_TMR_MODE_STDBY_WU;
    status = Pmic_timerSetCfg(&pmicHandle, &setCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Get and verify */
    getCfg.validParams = PMIC_CFG_TMR_MODE_VALID;
    status = Pmic_timerGetCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(getCfg.mode == PMIC_TMR_MODE_STDBY_WU);
}

/**
 * @brief Test setting and getting timer mode to operating, sequencing, and STANDBY.
 */
void test_pos_timer_timerSetCfg_modeOperSeqStdby(void)
{
    int32_t status;
    Pmic_TimerCfg_t setCfg, getCfg;

    /* Set mode to OPER_SEQ_STDBY */
    setCfg.validParams = PMIC_CFG_TMR_MODE_VALID;
    setCfg.mode = PMIC_TMR_MODE_OPER_SEQ_STDBY;
    status = Pmic_timerSetCfg(&pmicHandle, &setCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Get and verify */
    getCfg.validParams = PMIC_CFG_TMR_MODE_VALID;
    status = Pmic_timerGetCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(getCfg.mode == PMIC_TMR_MODE_OPER_SEQ_STDBY);
}

/**
 * @brief Test setting and getting timer mode to all states with wakeup.
 */
void test_pos_timer_timerSetCfg_modeOperSeqStdbyWu(void)
{
    int32_t status;
    Pmic_TimerCfg_t setCfg, getCfg;

    /* Set mode to OPER_SEQ_STDBY_WU */
    setCfg.validParams = PMIC_CFG_TMR_MODE_VALID;
    setCfg.mode = PMIC_TMR_MODE_OPER_SEQ_STDBY_WU;
    status = Pmic_timerSetCfg(&pmicHandle, &setCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Get and verify */
    getCfg.validParams = PMIC_CFG_TMR_MODE_VALID;
    status = Pmic_timerGetCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(getCfg.mode == PMIC_TMR_MODE_OPER_SEQ_STDBY_WU);
}

/**
 * @brief Test setting and getting both prescale and mode simultaneously.
 */
void test_pos_timer_timerSetCfg_prescaleAndModeVerify(void)
{
    int32_t status;
    Pmic_TimerCfg_t setCfg, getCfg;

    /* Ensure timer is stopped before configuring prescale */
    status = Pmic_timerStop(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Set both prescale and mode */
    setCfg.validParams = PMIC_CFG_TMR_PRESCALE_VALID | PMIC_CFG_TMR_MODE_VALID;
    setCfg.prescale = PMIC_TMR_PRESCALE_131P072_MS;
    setCfg.mode = PMIC_TMR_MODE_OPER_SEQ_STDBY;
    status = Pmic_timerSetCfg(&pmicHandle, &setCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Get and verify both */
    getCfg.validParams = PMIC_CFG_TMR_PRESCALE_VALID | PMIC_CFG_TMR_MODE_VALID;
    status = Pmic_timerGetCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(getCfg.prescale == PMIC_TMR_PRESCALE_131P072_MS);
    PLATFORM_ASSERT(getCfg.mode == PMIC_TMR_MODE_OPER_SEQ_STDBY);
}

/* Counter Tests - Positive (9 tests) */

/**
 * @brief Test setting and getting minimum counter value (0x000000).
 */
void test_pos_timer_timerSetCnt_minValueVerify(void)
{
    int32_t status;
    uint32_t setCnt = 0x000000U;
    uint32_t getCnt;

    /* Unlock CNT registers before timer operations */
    unlockCntRegisters();

    /* Set counter to minimum value */
    status = Pmic_timerSetCnt(&pmicHandle, setCnt);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Get and verify */
    status = Pmic_timerGetCnt(&pmicHandle, &getCnt);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(setCnt == getCnt);
}

/**
 * @brief Test setting and getting maximum counter value (0xFFFFFF).
 */
void test_pos_timer_timerSetCnt_maxValueVerify(void)
{
    int32_t status;
    uint32_t setCnt = 0xFFFFFFU;
    uint32_t getCnt;

    /* Unlock CNT registers before timer operations */
    unlockCntRegisters();

    /* Set counter to maximum value */
    status = Pmic_timerSetCnt(&pmicHandle, setCnt);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Get and verify */
    status = Pmic_timerGetCnt(&pmicHandle, &getCnt);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(setCnt == getCnt);
}

/**
 * @brief Test setting and getting mid-range counter value (0x800000).
 */
void test_pos_timer_timerSetCnt_midValueVerify(void)
{
    int32_t status;
    uint32_t setCnt = 0x800000U;
    uint32_t getCnt;

    /* Unlock CNT registers before timer operations */
    unlockCntRegisters();

    /* Set counter to mid-range value */
    status = Pmic_timerSetCnt(&pmicHandle, setCnt);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Get and verify */
    status = Pmic_timerGetCnt(&pmicHandle, &getCnt);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(setCnt == getCnt);
}

/**
 * @brief Test setting and getting boundary counter value (0x000001).
 */
void test_pos_timer_timerSetCnt_boundary1Verify(void)
{
    int32_t status;
    uint32_t setCnt = 0x000001U;
    uint32_t getCnt;

    /* Unlock CNT registers before timer operations */
    unlockCntRegisters();

    /* Set counter to boundary value */
    status = Pmic_timerSetCnt(&pmicHandle, setCnt);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Get and verify */
    status = Pmic_timerGetCnt(&pmicHandle, &getCnt);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(setCnt == getCnt);
}

/**
 * @brief Test setting and getting boundary counter value (0xFFFFFE).
 */
void test_pos_timer_timerSetCnt_boundary2Verify(void)
{
    int32_t status;
    uint32_t setCnt = 0xFFFFFEU;
    uint32_t getCnt;

    /* Unlock CNT registers before timer operations */
    unlockCntRegisters();

    /* Set counter to boundary value */
    status = Pmic_timerSetCnt(&pmicHandle, setCnt);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Get and verify */
    status = Pmic_timerGetCnt(&pmicHandle, &getCnt);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(setCnt == getCnt);
}

/**
 * @brief Test clearing timer counter.
 */
void test_pos_timer_timerClr_resetCounter(void)
{
    int32_t status;

    /* Unlock CNT registers before timer operations */
    unlockCntRegisters();

    /* Set counter to non-zero value first */
    status = Pmic_timerSetCnt(&pmicHandle, 0x123456U);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Clear the counter */
    status = Pmic_timerClr(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

/**
 * @brief Test that counter is zero after clearing.
 */
void test_pos_timer_timerClr_verifyZero(void)
{
    int32_t status;
    uint32_t getCnt;

    /* Unlock CNT registers before timer operations */
    unlockCntRegisters();

    /* Set counter to non-zero value */
    status = Pmic_timerSetCnt(&pmicHandle, 0xABCDEFU);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Clear the counter */
    status = Pmic_timerClr(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Verify counter is zero */
    status = Pmic_timerGetCnt(&pmicHandle, &getCnt);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(0x000000U == getCnt);
}

/**
 * @brief Test sequence: Set counter, clear, then get to verify zero.
 */
void test_pos_timer_timerClr_sequenceVerify(void)
{
    int32_t status;
    uint32_t getCnt;

    /* Unlock CNT registers before timer operations */
    unlockCntRegisters();

    /* Set counter */
    status = Pmic_timerSetCnt(&pmicHandle, 0x555555U);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Clear counter */
    status = Pmic_timerClr(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Get and verify zero */
    status = Pmic_timerGetCnt(&pmicHandle, &getCnt);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(0x000000U == getCnt);
}

/* Wakeup Tests - Positive (6 tests) */

/**
 * @brief Test setting and getting minimum wakeup value (0x000000).
 */
void test_pos_timer_timerSetWakeupValue_minValueVerify(void)
{
    int32_t status;
    uint32_t setWakeup = 0x000000U;
    uint32_t getWakeup;

    /* Set wakeup to minimum value */
    status = Pmic_timerSetWakeupValue(&pmicHandle, setWakeup);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Get and verify */
    status = Pmic_timerGetWakeupValue(&pmicHandle, &getWakeup);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(setWakeup == getWakeup);
}

/**
 * @brief Test setting and getting maximum wakeup value (0xFFFFFF).
 */
void test_pos_timer_timerSetWakeupValue_maxValueVerify(void)
{
    int32_t status;
    uint32_t setWakeup = 0xFFFFFFU;
    uint32_t getWakeup;

    /* Set wakeup to maximum value */
    status = Pmic_timerSetWakeupValue(&pmicHandle, setWakeup);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Get and verify */
    status = Pmic_timerGetWakeupValue(&pmicHandle, &getWakeup);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(setWakeup == getWakeup);
}

/**
 * @brief Test setting and getting mid-range wakeup value (0x800000).
 */
void test_pos_timer_timerSetWakeupValue_midValueVerify(void)
{
    int32_t status;
    uint32_t setWakeup = 0x800000U;
    uint32_t getWakeup;

    /* Set wakeup to mid-range value */
    status = Pmic_timerSetWakeupValue(&pmicHandle, setWakeup);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Get and verify */
    status = Pmic_timerGetWakeupValue(&pmicHandle, &getWakeup);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(setWakeup == getWakeup);
}

/**
 * @brief Test setting and getting boundary wakeup value (0x000001).
 */
void test_pos_timer_timerSetWakeupValue_boundary1Verify(void)
{
    int32_t status;
    uint32_t setWakeup = 0x000001U;
    uint32_t getWakeup;

    /* Set wakeup to boundary value */
    status = Pmic_timerSetWakeupValue(&pmicHandle, setWakeup);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Get and verify */
    status = Pmic_timerGetWakeupValue(&pmicHandle, &getWakeup);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(setWakeup == getWakeup);
}

/**
 * @brief Test setting and getting boundary wakeup value (0xFFFFFE).
 */
void test_pos_timer_timerSetWakeupValue_boundary2Verify(void)
{
    int32_t status;
    uint32_t setWakeup = 0xFFFFFEU;
    uint32_t getWakeup;

    /* Set wakeup to boundary value */
    status = Pmic_timerSetWakeupValue(&pmicHandle, setWakeup);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Get and verify */
    status = Pmic_timerGetWakeupValue(&pmicHandle, &getWakeup);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(setWakeup == getWakeup);
}

/**
 * @brief Test that wakeup value persists across mode changes.
 */
void test_pos_timer_timerSetWakeupValue_persistenceVerify(void)
{
    int32_t status;
    uint32_t setWakeup = 0x123456U;
    uint32_t getWakeup;
    Pmic_TimerCfg_t cfg;

    /* Set wakeup value */
    status = Pmic_timerSetWakeupValue(&pmicHandle, setWakeup);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Change mode to STDBY_WU */
    cfg.validParams = PMIC_CFG_TMR_MODE_VALID;
    cfg.mode = PMIC_TMR_MODE_STDBY_WU;
    status = Pmic_timerSetCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Change mode to OPER_SEQ_STDBY_WU */
    cfg.mode = PMIC_TMR_MODE_OPER_SEQ_STDBY_WU;
    status = Pmic_timerSetCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Verify wakeup value persisted */
    status = Pmic_timerGetWakeupValue(&pmicHandle, &getWakeup);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(setWakeup == getWakeup);
}

/* Control Tests - Positive (6 tests) */

/**
 * @brief Test stopping timer from mode 1 (OPER_SEQ).
 */
void test_pos_timer_timerStop_fromMode1(void)
{
    int32_t status;
    Pmic_TimerCfg_t cfg;

    /* Set mode to OPER_SEQ */
    cfg.validParams = PMIC_CFG_TMR_MODE_VALID;
    cfg.mode = PMIC_TMR_MODE_OPER_SEQ;
    status = Pmic_timerSetCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Stop the timer */
    status = Pmic_timerStop(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

/**
 * @brief Test stopping timer from mode 2 (STDBY).
 */
void test_pos_timer_timerStop_fromMode2(void)
{
    int32_t status;
    Pmic_TimerCfg_t cfg;

    /* Set mode to STDBY */
    cfg.validParams = PMIC_CFG_TMR_MODE_VALID;
    cfg.mode = PMIC_TMR_MODE_STDBY;
    status = Pmic_timerSetCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Stop the timer */
    status = Pmic_timerStop(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

/**
 * @brief Test stopping timer from mode 3 (STDBY_WU).
 */
void test_pos_timer_timerStop_fromMode3(void)
{
    int32_t status;
    Pmic_TimerCfg_t cfg;

    /* Set mode to STDBY_WU */
    cfg.validParams = PMIC_CFG_TMR_MODE_VALID;
    cfg.mode = PMIC_TMR_MODE_STDBY_WU;
    status = Pmic_timerSetCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Stop the timer */
    status = Pmic_timerStop(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

/**
 * @brief Test stopping timer from mode 4 (OPER_SEQ_STDBY).
 */
void test_pos_timer_timerStop_fromMode4(void)
{
    int32_t status;
    Pmic_TimerCfg_t cfg;

    /* Set mode to OPER_SEQ_STDBY */
    cfg.validParams = PMIC_CFG_TMR_MODE_VALID;
    cfg.mode = PMIC_TMR_MODE_OPER_SEQ_STDBY;
    status = Pmic_timerSetCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Stop the timer */
    status = Pmic_timerStop(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

/**
 * @brief Test stopping timer from mode 5 (OPER_SEQ_STDBY_WU).
 */
void test_pos_timer_timerStop_fromMode5(void)
{
    int32_t status;
    Pmic_TimerCfg_t cfg;

    /* Set mode to OPER_SEQ_STDBY_WU */
    cfg.validParams = PMIC_CFG_TMR_MODE_VALID;
    cfg.mode = PMIC_TMR_MODE_OPER_SEQ_STDBY_WU;
    status = Pmic_timerSetCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Stop the timer */
    status = Pmic_timerStop(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

/**
 * @brief Test verifying timer is in stopped mode after stop.
 */
void test_pos_timer_timerStop_verifyStopped(void)
{
    int32_t status;
    Pmic_TimerCfg_t setCfg, getCfg;

    /* Set mode to running state */
    setCfg.validParams = PMIC_CFG_TMR_MODE_VALID;
    setCfg.mode = PMIC_TMR_MODE_OPER_SEQ_STDBY;
    status = Pmic_timerSetCfg(&pmicHandle, &setCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Stop the timer */
    status = Pmic_timerStop(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Verify mode is STOPPED */
    getCfg.validParams = PMIC_CFG_TMR_MODE_VALID;
    status = Pmic_timerGetCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(getCfg.mode == PMIC_TMR_MODE_STOPPED);
}

/* ========================================================================== */
/*                         NEGATIVE TEST CASES                                */
/* ========================================================================== */

/* Configuration Tests - Negative (4 tests) */

/**
 * @brief Test Pmic_timerSetCfg with invalid prescale parameter.
 */
void test_neg_timer_timerSetCfg_invalidPrescale(void)
{
    /* Ensure timer is stopped to isolate test from previous state */
    int32_t status = Pmic_timerStop(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    Pmic_TimerCfg_t cfg = {
        .validParams = PMIC_CFG_TMR_PRESCALE_VALID,
        .prescale = PMIC_TMR_PRESCALE_MAX + 1
    };
    status = Pmic_timerSetCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_timerSetCfg with NULL config pointer.
 */
void test_neg_timer_timerSetCfg_nullCfg(void)
{
    int32_t status = Pmic_timerSetCfg(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_timerSetCfg with invalid mode parameter.
 */
void test_neg_timer_timerSetCfg_invalidMode(void)
{
    Pmic_TimerCfg_t cfg = {
        .validParams = PMIC_CFG_TMR_MODE_VALID,
        .mode = PMIC_TMR_MODE_MAX + 1
    };
    int32_t status = Pmic_timerSetCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_timerGetCfg with NULL config pointer.
 */
void test_neg_timer_timerGetCfg_nullCfg(void)
{
    int32_t status = Pmic_timerGetCfg(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/* Counter Tests - Negative (2 tests) */

/**
 * @brief Test Pmic_timerSetCnt with out-of-bounds counter value.
 */
void test_neg_timer_timerSetCnt_outOfBounds(void)
{
    /* Unlock CNT registers before timer operations */
    unlockCntRegisters();

    int32_t status = Pmic_timerSetCnt(&pmicHandle, PMIC_TMR_CNT_MAX + 1);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_timerGetCnt with NULL counter pointer.
 */
void test_neg_timer_timerGetCnt_nullCnt(void)
{
    int32_t status = Pmic_timerGetCnt(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/* Wakeup Tests - Negative (2 tests) */

/**
 * @brief Test Pmic_timerSetWakeupValue with out-of-bounds wakeup value.
 */
void test_neg_timer_timerSetWakeupValue_outOfBounds(void)
{
    int32_t status = Pmic_timerSetWakeupValue(&pmicHandle, PMIC_TMR_WAKEUP_VAL_MAX + 1);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_timerGetWakeupValue with NULL wakeup pointer.
 */
void test_neg_timer_timerGetWakeupValue_nullWakeup(void)
{
    int32_t status = Pmic_timerGetWakeupValue(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/* Handle Validation Tests - Negative (6 tests) */

/**
 * @brief Test Pmic_timerSetCfg with NULL handle.
 */
void test_neg_timer_timerSetCfg_nullHandle(void)
{
    Pmic_TimerCfg_t cfg = {0};
    int32_t status = Pmic_timerSetCfg(NULL, &cfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_timerGetCfg with NULL handle.
 */
void test_neg_timer_timerGetCfg_nullHandle(void)
{
    Pmic_TimerCfg_t cfg = {0};
    int32_t status = Pmic_timerGetCfg(NULL, &cfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_timerSetCnt with NULL handle.
 */
void test_neg_timer_timerSetCnt_nullHandle(void)
{
    int32_t status = Pmic_timerSetCnt(NULL, 0x123456U);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_timerGetCnt with NULL handle.
 */
void test_neg_timer_timerGetCnt_nullHandle(void)
{
    uint32_t cnt;
    int32_t status = Pmic_timerGetCnt(NULL, &cnt);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_timerSetWakeupValue with NULL handle.
 */
void test_neg_timer_timerSetWakeupValue_nullHandle(void)
{
    int32_t status = Pmic_timerSetWakeupValue(NULL, 0x123456U);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_timerGetWakeupValue with NULL handle.
 */
void test_neg_timer_timerGetWakeupValue_nullHandle(void)
{
    uint32_t wakeup;
    int32_t status = Pmic_timerGetWakeupValue(NULL, &wakeup);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/* Edge Case Tests (2 tests) */

/**
 * @brief Test Pmic_timerSetCnt with overflow value beyond maximum.
 */
void test_neg_timer_timerSetCnt_overflowValue(void)
{
    int32_t status;
    uint32_t cnt = PMIC_TMR_CNT_MAX + 1;  /* 0x1000000 - overflow */

    status = Pmic_timerSetCnt(&pmicHandle, cnt);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_timerSetCfg with validParams set to zero.
 */
void test_neg_timer_timerSetCfg_validParamsZero(void)
{
    Pmic_TimerCfg_t cfg = {
        .validParams = 0,
        .prescale = PMIC_TMR_PRESCALE_64P64_US
    };
    int32_t status = Pmic_timerSetCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_timerSetCfg prescale configuration while timer is running
 */
void test_neg_timer_timerSetCfg_prescaleWhileRunning(void)
{
    int32_t status;
    Pmic_TimerCfg_t cfg;

    /* Start timer in operational sequence mode */
    cfg.validParams = PMIC_CFG_TMR_MODE_VALID;
    cfg.mode = PMIC_TMR_MODE_OPER_SEQ;
    status = Pmic_timerSetCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Attempt to configure prescale while running - should fail */
    cfg.validParams = PMIC_CFG_TMR_PRESCALE_VALID;
    cfg.prescale = PMIC_TMR_PRESCALE_131P072_MS;
    status = Pmic_timerSetCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NOT_SUPPORTED);

    /* Stop timer for subsequent tests */
    status = Pmic_timerStop(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

/**
 * @brief Test Pmic_timerGetCfg with validParams set to zero.
 */
void test_neg_timer_timerGetCfg_validParamsZero(void)
{
    Pmic_TimerCfg_t cfg = {
        .validParams = 0
    };
    int32_t status = Pmic_timerGetCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/* Completeness Tests (2 tests) */

/**
 * @brief Test Pmic_timerStop with NULL handle.
 */
void test_neg_timer_timerStop_nullHandle(void)
{
    int32_t status = Pmic_timerStop(NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_timerClr with NULL handle.
 */
void test_neg_timer_timerClr_nullHandle(void)
{
    int32_t status = Pmic_timerClr(NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/* ========================================================================== */
/*                          Test Entry Point                                  */
/* ========================================================================== */

/**
 * @brief Timer test suite entry point
 * @param args Test arguments (unused)
 */
void timer_test(void *args)
{
    (void)args;
    char msg[50U] = {0};
    int32_t status = PMIC_ST_SUCCESS;
    /* Dummy handle for mock - driver validates non-NULL but doesn't dereference */
    static uint32_t dummyCommHandle = TEST_DUMMY_HANDLE;

    Pmic_HandleCfg_t pmicCfg = {
        .validParams = PMIC_COMM_MODE_VALID |
                       PMIC_COMM_HANDLE_0_VALID |
                       PMIC_IO_READ_VALID |
                       PMIC_IO_WRITE_VALID |
                       PMIC_CRITICAL_SECTION_START_VALID |
                       PMIC_CRITICAL_SECTION_STOP_VALID,
        .commMode = PMIC_INTF_SPI,
        .commHandle0 = (void*)&dummyCommHandle,  /* Driver requires non-NULL, even for mock */
        .ioRead = &platform_rxByte,
        .ioWrite = &platform_txByte,
        .criticalSectionStart = &platform_critSecStart,
        .criticalSectionStop = &platform_critSecStop
    };

    platform_init();
    testTimer_startModule("Timer");

    platform_printString("\r\n");
    platform_printString("TIMER_TEST\r\n");
    platform_printString("----------\r\n\r\n");

    status = Pmic_init(&pmicHandle, &pmicCfg);

    if (status == PMIC_ST_SUCCESS)
    {
        testUtils_printSiRev(&pmicHandle);

        platform_setupTests();
        TIMER_TEST_RUN_ALL();
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
