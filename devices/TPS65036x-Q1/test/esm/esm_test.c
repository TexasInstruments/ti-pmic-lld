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
/**
 * @file esm_test.c
 * @brief Source file containing definitions to PMIC ESM tests.
 */

/* ========================================================================== */
/*                              Include Files                                 */
/* ========================================================================== */

#include "esm_test.h"

/* ========================================================================== */
/*                             Global Variables                               */
/* ========================================================================== */

static Pmic_Handle_t pmicHandle = {0U};

/* ========================================================================== */
/*                           Function Definitions                             */
/* ========================================================================== */

void esm_test(void *args)
{
    char msg[50U] = {0};
    int32_t status = PMIC_ST_SUCCESS;

    platform_init();

    Pmic_HandleCfg_t pmicCfg = {
        .validParams = (PMIC_CFG_INIT_I2C_ADDR0_VALID |
                        PMIC_CFG_INIT_COMM_HANDLE_0_VALID |
                        PMIC_CFG_INIT_IO_READ_VALID |
                        PMIC_CFG_INIT_IO_WRITE_VALID |
                        PMIC_CFG_INIT_CRITICAL_SECTION_START_VALID |
                        PMIC_CFG_INIT_CRITICAL_SECTION_STOP_VALID |
                        PMIC_CFG_INIT_IRQ_RESPONSE_CALLBACK_VALID |
                        PMIC_CFG_INIT_TIMER_WAIT_MS_VALID),
        .i2cAddr0 = PLATFORM_TARGET_I2C_ADDR,
        .commHandle0 = platform_getCommHandle(),
        .ioRead = &platform_rxByte,
        .ioWrite = &platform_txByte,
        .criticalSectionStart = &platform_critSecStart,
        .criticalSectionStop = &platform_critSecStop,
        .irqResponseCallback = &platform_irqResponse,
        .timerWaitMs = &testUtils_timerWaitMs
    };

    testTimer_startModule("ESM");

    platform_printString("\r\n");
    platform_printString("ESM_TEST\r\n");
    platform_printString("-------\r\n\r\n");

    status = Pmic_init(&pmicHandle, &pmicCfg);

    if (status == PMIC_ST_SUCCESS)
    {
        testUtils_printSiRev(&pmicHandle);

        /* Unlock PMIC registers for testing */
        if (status != PMIC_ST_SUCCESS)
        {
            (void)sprintf(msg, "Error unlocking PMIC registers: %d\r\n", status);
            platform_printString(msg);
        }

        platform_setupTests();
        ESM_TEST_RUN_ALL();
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
/*                    Negative Tests - Pmic_esmSetCfg                         */
/* ========================================================================== */

void test_neg_esm_esmSetCfg_nullHandle(void)
{
    // Pass NULL pmicHandle into Pmic_esmSetCfg()
    Pmic_EsmCfg_t esmCfg = {
        .validParams = PMIC_CFG_ESM_ENABLE_VALID,
        .enable = PMIC_ENABLE
    };
    int32_t status = Pmic_esmSetCfg(NULL, &esmCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_esm_esmSetCfg_nullEsmCfg(void)
{
    // Pass NULL esmCfg into Pmic_esmSetCfg()
    int32_t status = Pmic_esmSetCfg(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_esm_esmSetCfg_invalidValidParams(void)
{
    // Pass invalid validParams (0) into Pmic_esmSetCfg()
    Pmic_EsmCfg_t esmCfg = {
        .validParams = 0U,
        .enable = PMIC_ENABLE
    };
    int32_t status = Pmic_esmSetCfg(&pmicHandle, &esmCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_esm_esmSetCfg_outOfBounds_mode(void)
{
    // Pass out of bounds mode into Pmic_esmSetCfg()
    Pmic_EsmCfg_t esmCfg = {
        .validParams = PMIC_CFG_ESM_MODE_VALID,
        .mode = ESM_MODE_MAX + 1U
    };
    int32_t status = Pmic_esmSetCfg(&pmicHandle, &esmCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_esm_esmSetCfg_outOfBounds_errCntThr(void)
{
    // Pass out of bounds errCntThr into Pmic_esmSetCfg()
    Pmic_EsmCfg_t esmCfg = {
        .validParams = PMIC_CFG_ESM_ERR_CNT_THR_VALID,
        .errCntThr = ESM_ERR_CNT_THR_MAX + 1U
    };
    int32_t status = Pmic_esmSetCfg(&pmicHandle, &esmCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/* ========================================================================== */
/*                    Negative Tests - Pmic_esmGetCfg                         */
/* ========================================================================== */

void test_neg_esm_esmGetCfg_nullHandle(void)
{
    // Pass NULL pmicHandle into Pmic_esmGetCfg()
    Pmic_EsmCfg_t esmCfg = {.validParams = PMIC_CFG_ESM_ENABLE_VALID};
    int32_t status = Pmic_esmGetCfg(NULL, &esmCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_esm_esmGetCfg_nullEsmCfg(void)
{
    // Pass NULL esmCfg into Pmic_esmGetCfg()
    int32_t status = Pmic_esmGetCfg(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_esm_esmGetCfg_invalidValidParams(void)
{
    // Pass invalid validParams (0) into Pmic_esmGetCfg()
    Pmic_EsmCfg_t esmCfg = {.validParams = 0U};
    int32_t status = Pmic_esmGetCfg(&pmicHandle, &esmCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/* ========================================================================== */
/*                  Negative Tests - Pmic_esmSetStartState                    */
/* ========================================================================== */

void test_neg_esm_esmSetStartState_nullHandle(void)
{
    // Pass NULL pmicHandle into Pmic_esmSetStartState()
    int32_t status = Pmic_esmSetStartState(NULL, PMIC_ESM_START);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/* ========================================================================== */
/*                     Negative Tests - Pmic_esmStart                         */
/* ========================================================================== */

void test_neg_esm_esmStart_nullHandle(void)
{
    // Pass NULL pmicHandle into Pmic_esmStart()
    int32_t status = Pmic_esmStart(NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/* ========================================================================== */
/*                     Negative Tests - Pmic_esmStop                          */
/* ========================================================================== */

void test_neg_esm_esmStop_nullHandle(void)
{
    // Pass NULL pmicHandle into Pmic_esmStop()
    int32_t status = Pmic_esmStop(NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/* ========================================================================== */
/*                  Negative Tests - Pmic_esmGetStartState                    */
/* ========================================================================== */

void test_neg_esm_esmGetStartState_nullHandle(void)
{
    // Pass NULL pmicHandle into Pmic_esmGetStartState()
    bool start = (bool)false;
    int32_t status = Pmic_esmGetStartState(NULL, &start);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_esm_esmGetStartState_nullStarted(void)
{
    // Pass NULL start into Pmic_esmGetStartState()
    int32_t status = Pmic_esmGetStartState(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/* ========================================================================== */
/*                   Negative Tests - Pmic_esmGetStatus                       */
/* ========================================================================== */

void test_neg_esm_esmGetStatus_nullHandle(void)
{
    // Pass NULL pmicHandle into Pmic_esmGetStatus()
    Pmic_EsmStatus_t esmStat = {.validParams = PMIC_ESM_RST_INT_VALID};
    int32_t status = Pmic_esmGetStatus(NULL, &esmStat);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_esm_esmGetStatus_nullEsmStat(void)
{
    // Pass NULL esmStat into Pmic_esmGetStatus()
    int32_t status = Pmic_esmGetStatus(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_esm_esmGetStatus_invalidValidParams_zero(void)
{
    // Pass invalid validParams (0) into Pmic_esmGetStatus()
    Pmic_EsmStatus_t esmStat = {.validParams = 0U};
    int32_t status = Pmic_esmGetStatus(&pmicHandle, &esmStat);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_esm_esmGetStatus_invalidValidParams_outOfBounds(void)
{
    // Pass out of bounds validParams into Pmic_esmGetStatus()
    Pmic_EsmStatus_t esmStat = {.validParams = PMIC_ESM_STATUS_ALL_VALID + 1U};
    int32_t status = Pmic_esmGetStatus(&pmicHandle, &esmStat);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/* ========================================================================== */
/*                   Negative Tests - Pmic_esmClrStatus                       */
/* ========================================================================== */

void test_neg_esm_esmClrStatus_nullHandle(void)
{
    // Pass NULL pmicHandle into Pmic_esmClrStatus()
    Pmic_EsmStatus_t esmStat = {.validParams = PMIC_ESM_RST_INT_VALID};
    int32_t status = Pmic_esmClrStatus(NULL, &esmStat);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_esm_esmClrStatus_nullEsmStat(void)
{
    // Pass NULL esmStat into Pmic_esmClrStatus()
    int32_t status = Pmic_esmClrStatus(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_esm_esmClrStatus_invalidValidParams_zero(void)
{
    // Pass invalid validParams (0) into Pmic_esmClrStatus()
    Pmic_EsmStatus_t esmStat = {.validParams = 0U};
    int32_t status = Pmic_esmClrStatus(&pmicHandle, &esmStat);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_esm_esmClrStatus_invalidValidParams_outOfBounds(void)
{
    // Pass out of bounds validParams into Pmic_esmClrStatus()
    Pmic_EsmStatus_t esmStat = {.validParams = PMIC_ESM_STATUS_ALL_VALID + 1U};
    int32_t status = Pmic_esmClrStatus(&pmicHandle, &esmStat);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/* ========================================================================== */
/*                   Negative Tests - Pmic_esmGetErrCnt                       */
/* ========================================================================== */

void test_neg_esm_esmGetErrCnt_nullHandle(void)
{
    // Pass NULL pmicHandle into Pmic_esmGetErrCnt()
    uint8_t errCnt = 0U;
    int32_t status = Pmic_esmGetErrCnt(NULL, &errCnt);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_esm_esmGetErrCnt_nullErrCnt(void)
{
    // Pass NULL errCnt into Pmic_esmGetErrCnt()
    int32_t status = Pmic_esmGetErrCnt(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/* ========================================================================== */
/*                 Positive Tests - Set/Get ESM Configuration                 */
/* ========================================================================== */

void test_pos_esm_esmSetGetCfg_enable(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_EsmCfg_t expCfg = {.validParams = PMIC_CFG_ESM_ENABLE_VALID};
    Pmic_EsmCfg_t actCfg = {.validParams = PMIC_CFG_ESM_ENABLE_VALID};

    // Stop ESM before configuration
    status = Pmic_esmStop(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Enable ESM
    expCfg.enable = PMIC_ENABLE;
    status = Pmic_esmSetCfg(&pmicHandle, &expCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get actual enable status and compare expected vs. actual values
    status = Pmic_esmGetCfg(&pmicHandle, &actCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(actCfg.enable == PMIC_ENABLE);

    // Disable ESM
    expCfg.enable = PMIC_DISABLE;
    status = Pmic_esmSetCfg(&pmicHandle, &expCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get actual enable status and compare expected vs. actual values
    status = Pmic_esmGetCfg(&pmicHandle, &actCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(actCfg.enable == PMIC_DISABLE);
}

void test_pos_esm_esmSetGetCfg_mode_level(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_EsmCfg_t expCfg = {.validParams = PMIC_CFG_ESM_MODE_VALID};
    Pmic_EsmCfg_t actCfg = {.validParams = PMIC_CFG_ESM_MODE_VALID};

    // Stop ESM before configuration
    status = Pmic_esmStop(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Set ESM mode to Level mode
    expCfg.mode = ESM_LEVEL_MODE;
    status = Pmic_esmSetCfg(&pmicHandle, &expCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get actual mode and compare expected vs. actual values
    status = Pmic_esmGetCfg(&pmicHandle, &actCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(actCfg.mode == ESM_LEVEL_MODE);
}

void test_pos_esm_esmSetGetCfg_mode_pwm(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_EsmCfg_t expCfg = {.validParams = PMIC_CFG_ESM_MODE_VALID};
    Pmic_EsmCfg_t actCfg = {.validParams = PMIC_CFG_ESM_MODE_VALID};

    // Stop ESM before configuration
    status = Pmic_esmStop(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Set ESM mode to PWM mode
    expCfg.mode = ESM_PWM_MODE;
    status = Pmic_esmSetCfg(&pmicHandle, &expCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get actual mode and compare expected vs. actual values
    status = Pmic_esmGetCfg(&pmicHandle, &actCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(actCfg.mode == ESM_PWM_MODE);
}

void test_pos_esm_esmSetGetCfg_errCntThr(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_EsmCfg_t expCfg = {.validParams = PMIC_CFG_ESM_ERR_CNT_THR_VALID};
    Pmic_EsmCfg_t actCfg = {.validParams = PMIC_CFG_ESM_ERR_CNT_THR_VALID};

    // Stop ESM before configuration
    status = Pmic_esmStop(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Test boundary values
    expCfg.errCntThr = 0U;
    status = Pmic_esmSetCfg(&pmicHandle, &expCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    status = Pmic_esmGetCfg(&pmicHandle, &actCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(actCfg.errCntThr == 0U);

    expCfg.errCntThr = ESM_ERR_CNT_THR_MAX;
    status = Pmic_esmSetCfg(&pmicHandle, &expCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    status = Pmic_esmGetCfg(&pmicHandle, &actCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(actCfg.errCntThr == ESM_ERR_CNT_THR_MAX);
}

void test_pos_esm_esmSetGetCfg_delay1(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_EsmCfg_t expCfg = {.validParams = PMIC_CFG_ESM_DELAY1_VALID};
    Pmic_EsmCfg_t actCfg = {.validParams = PMIC_CFG_ESM_DELAY1_VALID};

    // Stop ESM before configuration
    status = Pmic_esmStop(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Test boundary values
    expCfg.delay1 = 0U;
    status = Pmic_esmSetCfg(&pmicHandle, &expCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    status = Pmic_esmGetCfg(&pmicHandle, &actCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(actCfg.delay1 == 0U);

    expCfg.delay1 = 0xFFU;
    status = Pmic_esmSetCfg(&pmicHandle, &expCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    status = Pmic_esmGetCfg(&pmicHandle, &actCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(actCfg.delay1 == 0xFFU);
}

void test_pos_esm_esmSetGetCfg_delay2(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_EsmCfg_t expCfg = {.validParams = PMIC_CFG_ESM_DELAY2_VALID};
    Pmic_EsmCfg_t actCfg = {.validParams = PMIC_CFG_ESM_DELAY2_VALID};

    // Stop ESM before configuration
    status = Pmic_esmStop(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Test boundary values
    expCfg.delay2 = 0U;
    status = Pmic_esmSetCfg(&pmicHandle, &expCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    status = Pmic_esmGetCfg(&pmicHandle, &actCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(actCfg.delay2 == 0U);

    expCfg.delay2 = 0xFFU;
    status = Pmic_esmSetCfg(&pmicHandle, &expCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    status = Pmic_esmGetCfg(&pmicHandle, &actCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(actCfg.delay2 == 0xFFU);
}

void test_pos_esm_esmSetGetCfg_hmax(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_EsmCfg_t expCfg = {.validParams = PMIC_CFG_ESM_HMAX_VALID};
    Pmic_EsmCfg_t actCfg = {.validParams = PMIC_CFG_ESM_HMAX_VALID};

    // Stop ESM before configuration
    status = Pmic_esmStop(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Test boundary values
    expCfg.hmax = 0U;
    status = Pmic_esmSetCfg(&pmicHandle, &expCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    status = Pmic_esmGetCfg(&pmicHandle, &actCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(actCfg.hmax == 0U);

    expCfg.hmax = 0xFFU;
    status = Pmic_esmSetCfg(&pmicHandle, &expCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    status = Pmic_esmGetCfg(&pmicHandle, &actCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(actCfg.hmax == 0xFFU);
}

void test_pos_esm_esmSetGetCfg_hmin(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_EsmCfg_t expCfg = {.validParams = PMIC_CFG_ESM_HMIN_VALID};
    Pmic_EsmCfg_t actCfg = {.validParams = PMIC_CFG_ESM_HMIN_VALID};

    // Stop ESM before configuration
    status = Pmic_esmStop(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Test boundary values
    expCfg.hmin = 0U;
    status = Pmic_esmSetCfg(&pmicHandle, &expCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    status = Pmic_esmGetCfg(&pmicHandle, &actCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(actCfg.hmin == 0U);

    expCfg.hmin = 0xFFU;
    status = Pmic_esmSetCfg(&pmicHandle, &expCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    status = Pmic_esmGetCfg(&pmicHandle, &actCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(actCfg.hmin == 0xFFU);
}

void test_pos_esm_esmSetGetCfg_lmax(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_EsmCfg_t expCfg = {.validParams = PMIC_CFG_ESM_LMAX_VALID};
    Pmic_EsmCfg_t actCfg = {.validParams = PMIC_CFG_ESM_LMAX_VALID};

    // Stop ESM before configuration
    status = Pmic_esmStop(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Test boundary values
    expCfg.lmax = 0U;
    status = Pmic_esmSetCfg(&pmicHandle, &expCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    status = Pmic_esmGetCfg(&pmicHandle, &actCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(actCfg.lmax == 0U);

    expCfg.lmax = 0xFFU;
    status = Pmic_esmSetCfg(&pmicHandle, &expCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    status = Pmic_esmGetCfg(&pmicHandle, &actCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(actCfg.lmax == 0xFFU);
}

void test_pos_esm_esmSetGetCfg_lmin(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_EsmCfg_t expCfg = {.validParams = PMIC_CFG_ESM_LMIN_VALID};
    Pmic_EsmCfg_t actCfg = {.validParams = PMIC_CFG_ESM_LMIN_VALID};

    // Stop ESM before configuration
    status = Pmic_esmStop(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Test boundary values
    expCfg.lmin = 0U;
    status = Pmic_esmSetCfg(&pmicHandle, &expCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    status = Pmic_esmGetCfg(&pmicHandle, &actCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(actCfg.lmin == 0U);

    expCfg.lmin = 0xFFU;
    status = Pmic_esmSetCfg(&pmicHandle, &expCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    status = Pmic_esmGetCfg(&pmicHandle, &actCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(actCfg.lmin == 0xFFU);
}

/* ========================================================================== */
/*                    Positive Tests - Start/Stop ESM                         */
/* ========================================================================== */

void test_pos_esm_esmStartStop(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    bool startState = (bool)false;

    // Start ESM using Pmic_esmSetStartState()
    status = Pmic_esmSetStartState(&pmicHandle, PMIC_ESM_START);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get actual start state and compare expected vs. actual values
    status = Pmic_esmGetStartState(&pmicHandle, &startState);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(startState == (bool)true);

    // Stop ESM using Pmic_esmSetStartState()
    status = Pmic_esmSetStartState(&pmicHandle, PMIC_ESM_STOP);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get actual start state and compare expected vs. actual values
    status = Pmic_esmGetStartState(&pmicHandle, &startState);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(startState == (bool)false);
}

void test_pos_esm_esmStart(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    bool startState = (bool)false;

    // Start ESM using Pmic_esmStart()
    status = Pmic_esmStart(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get actual start state and compare expected vs. actual values
    status = Pmic_esmGetStartState(&pmicHandle, &startState);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(startState == (bool)true);
}

void test_pos_esm_esmStop(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    bool startState = (bool)false;

    // Start ESM first
    status = Pmic_esmStart(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Stop ESM using Pmic_esmStop()
    status = Pmic_esmStop(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get actual start state and compare expected vs. actual values
    status = Pmic_esmGetStartState(&pmicHandle, &startState);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(startState == (bool)false);
}

void test_pos_esm_esmGetStartState(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    bool startState = (bool)false;

    // Stop ESM
    status = Pmic_esmStop(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get start state and verify it is stopped
    status = Pmic_esmGetStartState(&pmicHandle, &startState);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(startState == (bool)false);

    // Start ESM
    status = Pmic_esmStart(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get start state and verify it is started
    status = Pmic_esmGetStartState(&pmicHandle, &startState);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(startState == (bool)true);

    // Stop ESM
    status = Pmic_esmStop(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

/* ========================================================================== */
/*                      Positive Tests - ESM Status                           */
/* ========================================================================== */

void test_pos_esm_esmGetStatus(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_EsmStatus_t esmStat = {.validParams = PMIC_ESM_STATUS_ALL_VALID};

    // Get ESM status (all status bits)
    status = Pmic_esmGetStatus(&pmicHandle, &esmStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get individual ESM status bits
    esmStat.validParams = PMIC_ESM_RST_INT_VALID;
    status = Pmic_esmGetStatus(&pmicHandle, &esmStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    esmStat.validParams = PMIC_ESM_FAIL_INT_VALID;
    status = Pmic_esmGetStatus(&pmicHandle, &esmStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    esmStat.validParams = PMIC_ESM_PIN_INT_VALID;
    status = Pmic_esmGetStatus(&pmicHandle, &esmStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_esm_esmClrStatus(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_EsmStatus_t esmStat = {0U};

    // Clear all ESM status bits
    esmStat.validParams = PMIC_ESM_STATUS_ALL_VALID;
    status = Pmic_esmClrStatus(&pmicHandle, &esmStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Clear individual ESM status bits
    esmStat.validParams = PMIC_ESM_RST_INT_VALID;
    status = Pmic_esmClrStatus(&pmicHandle, &esmStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    esmStat.validParams = PMIC_ESM_FAIL_INT_VALID;
    status = Pmic_esmClrStatus(&pmicHandle, &esmStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    esmStat.validParams = PMIC_ESM_PIN_INT_VALID;
    status = Pmic_esmClrStatus(&pmicHandle, &esmStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

/* ========================================================================== */
/*                   Positive Tests - ESM Error Count                         */
/* ========================================================================== */

void test_pos_esm_esmGetErrCnt(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t errCnt = 0U;

    // Get ESM error count
    status = Pmic_esmGetErrCnt(&pmicHandle, &errCnt);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

