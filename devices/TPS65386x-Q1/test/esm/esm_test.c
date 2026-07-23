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
#include "esm_test.h"
#include "test_constants.h"

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */
static Pmic_Handle_t g_handle;

/* Dummy handle for mock - driver validates non-NULL but doesn't dereference */
static uint32_t dummyCommHandle = TEST_DUMMY_HANDLE;

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
        .ioRead = &platform_rxByte,
        .ioWrite = &platform_txByte,
        .criticalSectionStart = &platform_critSecStart,
        .criticalSectionStop = &platform_critSecStop
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

void test_pos_esm_esmSetGetStartState(void)
{
    int32_t status;
    bool startSet, startGet;

    status = helper_initPmic(&g_handle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    /* Test: Start ESM */
    startSet = true;
    status = Pmic_esmSetStartState(&g_handle, startSet);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_esmGetStartState(&g_handle, &startGet);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(startSet, startGet);

    /* Test: Stop ESM */
    startSet = false;
    status = Pmic_esmSetStartState(&g_handle, startSet);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_esmGetStartState(&g_handle, &startGet);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(startSet, startGet);

    helper_deinitPmic(&g_handle);
}

void test_pos_esm_esmStart(void)
{
    int32_t status;
    bool startState;

    status = helper_initPmic(&g_handle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_esmStart(&g_handle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_esmGetStartState(&g_handle, &startState);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(true, startState);

    helper_deinitPmic(&g_handle);
}

void test_pos_esm_esmStop(void)
{
    int32_t status;
    bool startState;

    status = helper_initPmic(&g_handle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    /* Start ESM first */
    status = Pmic_esmStart(&g_handle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    /* Then stop ESM */
    status = Pmic_esmStop(&g_handle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_esmGetStartState(&g_handle, &startState);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(false, startState);

    helper_deinitPmic(&g_handle);
}

void test_pos_esm_esmSetCfg_enable(void)
{
    int32_t status;
    Pmic_EsmCfg_t cfgSet = {0}, cfgGet = {0};

    status = helper_initPmic(&g_handle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    /* Test enable = true */
    cfgSet.validParams = PMIC_ESM_CFG_VALID;
    cfgSet.enable = true;
    status = Pmic_esmSetCfg(&g_handle, &cfgSet);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    cfgGet.validParams = PMIC_ESM_CFG_VALID;
    status = Pmic_esmGetCfg(&g_handle, &cfgGet);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(cfgSet.enable, cfgGet.enable);

    /* Test enable = false */
    cfgSet.enable = false;
    status = Pmic_esmSetCfg(&g_handle, &cfgSet);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_esmGetCfg(&g_handle, &cfgGet);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(cfgSet.enable, cfgGet.enable);

    helper_deinitPmic(&g_handle);
}

void test_pos_esm_esmSetCfg_mode(void)
{
    int32_t status;
    Pmic_EsmCfg_t cfgSet = {0}, cfgGet = {0};

    status = helper_initPmic(&g_handle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    /* Test LEVEL_MODE */
    cfgSet.validParams = PMIC_ESM_CFG_VALID;
    cfgSet.mode = PMIC_ESM_LEVEL_MODE;
    status = Pmic_esmSetCfg(&g_handle, &cfgSet);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    cfgGet.validParams = PMIC_ESM_CFG_VALID;
    status = Pmic_esmGetCfg(&g_handle, &cfgGet);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(cfgSet.mode, cfgGet.mode);

    /* Test PWM_MODE */
    cfgSet.mode = PMIC_ESM_PWM_MODE;
    status = Pmic_esmSetCfg(&g_handle, &cfgSet);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_esmGetCfg(&g_handle, &cfgGet);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(cfgSet.mode, cfgGet.mode);

    helper_deinitPmic(&g_handle);
}

void test_pos_esm_esmSetCfg_errThr(void)
{
    int32_t status;
    Pmic_EsmCfg_t cfgSet = {0}, cfgGet = {0};
    uint8_t testValues[] = {0x0, 0x7, 0xF};

    status = helper_initPmic(&g_handle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    cfgSet.validParams = PMIC_ESM_CFG_VALID;
    cfgGet.validParams = PMIC_ESM_CFG_VALID;

    for (uint32_t i = 0; i < sizeof(testValues); i++)
    {
        cfgSet.errThr = testValues[i];
        status = Pmic_esmSetCfg(&g_handle, &cfgSet);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        status = Pmic_esmGetCfg(&g_handle, &cfgGet);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        TEST_ASSERT_EQUAL(cfgSet.errThr, cfgGet.errThr);
    }

    helper_deinitPmic(&g_handle);
}

void test_pos_esm_esmSetCfg_polarity(void)
{
    int32_t status;
    Pmic_EsmCfg_t cfgSet = {0}, cfgGet = {0};

    status = helper_initPmic(&g_handle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    cfgSet.validParams = PMIC_ESM_CFG_VALID;
    cfgGet.validParams = PMIC_ESM_CFG_VALID;

    /* Test POLARITY_LOW_GOOD */
    cfgSet.polarity = PMIC_ESM_POLARITY_LOW_GOOD;
    status = Pmic_esmSetCfg(&g_handle, &cfgSet);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_esmGetCfg(&g_handle, &cfgGet);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(cfgSet.polarity, cfgGet.polarity);

    /* Test POLARITY_HIGH_GOOD */
    cfgSet.polarity = PMIC_ESM_POLARITY_HIGH_GOOD;
    status = Pmic_esmSetCfg(&g_handle, &cfgSet);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_esmGetCfg(&g_handle, &cfgGet);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(cfgSet.polarity, cfgGet.polarity);

    helper_deinitPmic(&g_handle);
}

void test_pos_esm_esmSetCfg_deglitch(void)
{
    int32_t status;
    Pmic_EsmCfg_t cfgSet = {0}, cfgGet = {0};

    status = helper_initPmic(&g_handle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    cfgSet.validParams = PMIC_ESM_CFG_VALID;
    cfgGet.validParams = PMIC_ESM_CFG_VALID;

    /* Test DEGLITCH_1_US */
    cfgSet.deglitch = PMIC_ESM_DEGLITCH_1_US;
    status = Pmic_esmSetCfg(&g_handle, &cfgSet);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_esmGetCfg(&g_handle, &cfgGet);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(cfgSet.deglitch, cfgGet.deglitch);

    /* Test DEGLITCH_4_US */
    cfgSet.deglitch = PMIC_ESM_DEGLITCH_4_US;
    status = Pmic_esmSetCfg(&g_handle, &cfgSet);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_esmGetCfg(&g_handle, &cfgGet);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(cfgSet.deglitch, cfgGet.deglitch);

    helper_deinitPmic(&g_handle);
}

void test_pos_esm_esmSetCfg_timeBase(void)
{
    int32_t status;
    Pmic_EsmCfg_t cfgSet = {0}, cfgGet = {0};
    uint8_t testValues[] = {
        PMIC_ESM_TIME_BASE_2_US,
        PMIC_ESM_TIME_BASE_16_US,
        PMIC_ESM_TIME_BASE_64_US,
        PMIC_ESM_TIME_BASE_96_US
    };

    status = helper_initPmic(&g_handle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    cfgSet.validParams = PMIC_ESM_CFG_VALID;
    cfgGet.validParams = PMIC_ESM_CFG_VALID;

    for (uint32_t i = 0; i < sizeof(testValues); i++)
    {
        cfgSet.timeBase = testValues[i];
        status = Pmic_esmSetCfg(&g_handle, &cfgSet);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        status = Pmic_esmGetCfg(&g_handle, &cfgGet);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        TEST_ASSERT_EQUAL(cfgSet.timeBase, cfgGet.timeBase);
    }

    helper_deinitPmic(&g_handle);
}

void test_pos_esm_esmSetCfg_delay1(void)
{
    int32_t status;
    Pmic_EsmCfg_t cfgSet = {0}, cfgGet = {0};
    uint8_t testValues[] = {0x00, 0x55, 0xAA, 0xFF};

    status = helper_initPmic(&g_handle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    cfgSet.validParams = PMIC_CFG_ESM_DELAY1_VALID_SHIFT;
    cfgGet.validParams = PMIC_CFG_ESM_DELAY1_VALID_SHIFT;

    for (uint32_t i = 0; i < sizeof(testValues); i++)
    {
        cfgSet.delay1 = testValues[i];
        status = Pmic_esmSetCfg(&g_handle, &cfgSet);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        status = Pmic_esmGetCfg(&g_handle, &cfgGet);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        TEST_ASSERT_EQUAL(cfgSet.delay1, cfgGet.delay1);
    }

    helper_deinitPmic(&g_handle);
}

void test_pos_esm_esmSetCfg_delay2(void)
{
    int32_t status;
    Pmic_EsmCfg_t cfgSet = {0}, cfgGet = {0};
    uint8_t testValues[] = {0x00, 0x55, 0xAA, 0xFF};

    status = helper_initPmic(&g_handle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    cfgSet.validParams = PMIC_CFG_ESM_DELAY2_VALID_SHIFT;
    cfgGet.validParams = PMIC_CFG_ESM_DELAY2_VALID_SHIFT;

    for (uint32_t i = 0; i < sizeof(testValues); i++)
    {
        cfgSet.delay2 = testValues[i];
        status = Pmic_esmSetCfg(&g_handle, &cfgSet);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        status = Pmic_esmGetCfg(&g_handle, &cfgGet);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        TEST_ASSERT_EQUAL(cfgSet.delay2, cfgGet.delay2);
    }

    helper_deinitPmic(&g_handle);
}

void test_pos_esm_esmSetCfg_hmax(void)
{
    int32_t status;
    Pmic_EsmCfg_t cfgSet = {0}, cfgGet = {0};
    uint8_t testValues[] = {0x00, 0x7F, 0xFF};

    status = helper_initPmic(&g_handle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    cfgSet.validParams = PMIC_ESM_CFG_VALID;
    cfgGet.validParams = PMIC_ESM_CFG_VALID;

    for (uint32_t i = 0; i < sizeof(testValues); i++)
    {
        cfgSet.hmax = testValues[i];
        status = Pmic_esmSetCfg(&g_handle, &cfgSet);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        status = Pmic_esmGetCfg(&g_handle, &cfgGet);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        TEST_ASSERT_EQUAL(cfgSet.hmax, cfgGet.hmax);
    }

    helper_deinitPmic(&g_handle);
}

void test_pos_esm_esmSetCfg_hmin(void)
{
    int32_t status;
    Pmic_EsmCfg_t cfgSet = {0}, cfgGet = {0};
    uint8_t testValues[] = {0x00, 0x7F, 0xFF};

    status = helper_initPmic(&g_handle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    cfgSet.validParams = PMIC_ESM_CFG_VALID;
    cfgGet.validParams = PMIC_ESM_CFG_VALID;

    for (uint32_t i = 0; i < sizeof(testValues); i++)
    {
        cfgSet.hmin = testValues[i];
        status = Pmic_esmSetCfg(&g_handle, &cfgSet);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        status = Pmic_esmGetCfg(&g_handle, &cfgGet);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        TEST_ASSERT_EQUAL(cfgSet.hmin, cfgGet.hmin);
    }

    helper_deinitPmic(&g_handle);
}

void test_pos_esm_esmSetCfg_lmax(void)
{
    int32_t status;
    Pmic_EsmCfg_t cfgSet = {0}, cfgGet = {0};
    uint8_t testValues[] = {0x00, 0x7F, 0xFF};

    status = helper_initPmic(&g_handle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    cfgSet.validParams = PMIC_ESM_CFG_VALID;
    cfgGet.validParams = PMIC_ESM_CFG_VALID;

    for (uint32_t i = 0; i < sizeof(testValues); i++)
    {
        cfgSet.lmax = testValues[i];
        status = Pmic_esmSetCfg(&g_handle, &cfgSet);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        status = Pmic_esmGetCfg(&g_handle, &cfgGet);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        TEST_ASSERT_EQUAL(cfgSet.lmax, cfgGet.lmax);
    }

    helper_deinitPmic(&g_handle);
}

void test_pos_esm_esmSetCfg_lmin(void)
{
    int32_t status;
    Pmic_EsmCfg_t cfgSet = {0}, cfgGet = {0};
    uint8_t testValues[] = {0x00, 0x7F, 0xFF};

    status = helper_initPmic(&g_handle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    cfgSet.validParams = PMIC_ESM_CFG_VALID;
    cfgGet.validParams = PMIC_ESM_CFG_VALID;

    for (uint32_t i = 0; i < sizeof(testValues); i++)
    {
        cfgSet.lmin = testValues[i];
        status = Pmic_esmSetCfg(&g_handle, &cfgSet);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        status = Pmic_esmGetCfg(&g_handle, &cfgGet);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        TEST_ASSERT_EQUAL(cfgSet.lmin, cfgGet.lmin);
    }

    helper_deinitPmic(&g_handle);
}

void test_pos_esm_esmSetCfg_multiple(void)
{
    int32_t status;
    Pmic_EsmCfg_t cfgSet = {0}, cfgGet = {0};

    status = helper_initPmic(&g_handle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    /* Configure multiple parameters at once */
    cfgSet.validParams = PMIC_ESM_CFG_VALID |
                         PMIC_ESM_CFG_VALID |
                         PMIC_ESM_CFG_VALID |
                         PMIC_ESM_CFG_VALID |
                         PMIC_ESM_CFG_VALID |
                         PMIC_ESM_CFG_VALID;
    cfgSet.enable = true;
    cfgSet.mode = PMIC_ESM_PWM_MODE;
    cfgSet.errThr = 0x7;
    cfgSet.polarity = PMIC_ESM_POLARITY_HIGH_GOOD;
    cfgSet.deglitch = PMIC_ESM_DEGLITCH_4_US;
    cfgSet.timeBase = PMIC_ESM_TIME_BASE_32_US;

    status = Pmic_esmSetCfg(&g_handle, &cfgSet);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    cfgGet.validParams = cfgSet.validParams;
    status = Pmic_esmGetCfg(&g_handle, &cfgGet);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(cfgSet.enable, cfgGet.enable);
    TEST_ASSERT_EQUAL(cfgSet.mode, cfgGet.mode);
    TEST_ASSERT_EQUAL(cfgSet.errThr, cfgGet.errThr);
    TEST_ASSERT_EQUAL(cfgSet.polarity, cfgGet.polarity);
    TEST_ASSERT_EQUAL(cfgSet.deglitch, cfgGet.deglitch);
    TEST_ASSERT_EQUAL(cfgSet.timeBase, cfgGet.timeBase);

    helper_deinitPmic(&g_handle);
}

void test_pos_esm_esmGetStatus_esmErr(void)
{
    int32_t status;
    Pmic_EsmStatus_t esmStatus = {0};

    status = helper_initPmic(&g_handle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    esmStatus.validParams = PMIC_ESM_ERR_VALID_SHIFT;
    status = Pmic_esmGetStatus(&g_handle, &esmStatus);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    helper_deinitPmic(&g_handle);
}

void test_pos_esm_esmGetStatus_delay1Err(void)
{
    int32_t status;
    Pmic_EsmStatus_t esmStatus = {0};

    status = helper_initPmic(&g_handle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    esmStatus.validParams = PMIC_ESM_DELAY1_ERR_VALID_SHIFT;
    status = Pmic_esmGetStatus(&g_handle, &esmStatus);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    helper_deinitPmic(&g_handle);
}

void test_pos_esm_esmGetStatus_delay2Err(void)
{
    int32_t status;
    Pmic_EsmStatus_t esmStatus = {0};

    status = helper_initPmic(&g_handle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    esmStatus.validParams = PMIC_ESM_DELAY2_ERR_VALID_SHIFT;
    status = Pmic_esmGetStatus(&g_handle, &esmStatus);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    helper_deinitPmic(&g_handle);
}

void test_pos_esm_esmGetStatus_errCnt(void)
{
    int32_t status;
    Pmic_EsmStatus_t esmStatus = {0};

    status = helper_initPmic(&g_handle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    esmStatus.validParams = PMIC_ESM_ERR_CNT_VALID_SHIFT;
    status = Pmic_esmGetStatus(&g_handle, &esmStatus);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    helper_deinitPmic(&g_handle);
}

void test_pos_esm_esmClrStatus(void)
{
    int32_t status;
    Pmic_EsmStatus_t esmStatus = {0};

    status = helper_initPmic(&g_handle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    /* Clear all clearable status flags */
    esmStatus.validParams = PMIC_ESM_ERR_VALID_SHIFT |
                            PMIC_ESM_DELAY1_ERR_VALID_SHIFT |
                            PMIC_ESM_DELAY2_ERR_VALID_SHIFT;
    status = Pmic_esmClrStatus(&g_handle, &esmStatus);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    helper_deinitPmic(&g_handle);
}

/* ========================================================================== */
/*                         Negative Test Implementations                      */
/* ========================================================================== */

void test_neg_esm_esmSetStartState_nullHandle(void)
{
    int32_t status;

    status = Pmic_esmSetStartState(NULL, true);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

void test_neg_esm_esmGetStartState_nullHandle(void)
{
    int32_t status;
    bool startState;

    status = Pmic_esmGetStartState(NULL, &startState);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

void test_neg_esm_esmGetStartState_nullPointer(void)
{
    int32_t status;

    status = helper_initPmic(&g_handle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_esmGetStartState(&g_handle, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    helper_deinitPmic(&g_handle);
}

void test_neg_esm_esmStart_nullHandle(void)
{
    int32_t status;

    status = Pmic_esmStart(NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

void test_neg_esm_esmStop_nullHandle(void)
{
    int32_t status;

    status = Pmic_esmStop(NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

void test_neg_esm_esmSetCfg_nullHandle(void)
{
    int32_t status;
    Pmic_EsmCfg_t esmCfg = {0};

    esmCfg.validParams = PMIC_ESM_CFG_VALID;
    status = Pmic_esmSetCfg(NULL, &esmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

void test_neg_esm_esmSetCfg_nullPointer(void)
{
    int32_t status;

    status = helper_initPmic(&g_handle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_esmSetCfg(&g_handle, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    helper_deinitPmic(&g_handle);
}

void test_neg_esm_esmSetCfg_invalidParams(void)
{
    int32_t status;
    Pmic_EsmCfg_t esmCfg = {0};

    status = helper_initPmic(&g_handle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    esmCfg.validParams = 0U;
    status = Pmic_esmSetCfg(&g_handle, &esmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

    helper_deinitPmic(&g_handle);
}

void test_neg_esm_esmSetCfg_invalidMode(void)
{
    int32_t status;
    Pmic_EsmCfg_t esmCfg = {0};

    status = helper_initPmic(&g_handle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    esmCfg.validParams = PMIC_ESM_CFG_VALID;
    esmCfg.mode = PMIC_ESM_MODE_MAX + 1;
    status = Pmic_esmSetCfg(&g_handle, &esmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

    helper_deinitPmic(&g_handle);
}

void test_neg_esm_esmSetCfg_invalidErrThr(void)
{
    int32_t status;
    Pmic_EsmCfg_t esmCfg = {0};

    status = helper_initPmic(&g_handle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    esmCfg.validParams = PMIC_ESM_CFG_VALID;
    esmCfg.errThr = PMIC_ESM_ERR_THR_MAX + 1;
    status = Pmic_esmSetCfg(&g_handle, &esmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

    helper_deinitPmic(&g_handle);
}

void test_neg_esm_esmSetCfg_invalidPolarity(void)
{
    int32_t status;
    Pmic_EsmCfg_t esmCfg = {0};

    status = helper_initPmic(&g_handle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    esmCfg.validParams = PMIC_ESM_CFG_VALID;
    esmCfg.polarity = PMIC_ESM_POLARITY_MAX + 1;
    status = Pmic_esmSetCfg(&g_handle, &esmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

    helper_deinitPmic(&g_handle);
}

void test_neg_esm_esmSetCfg_invalidDeglitch(void)
{
    int32_t status;
    Pmic_EsmCfg_t esmCfg = {0};

    status = helper_initPmic(&g_handle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    esmCfg.validParams = PMIC_ESM_CFG_VALID;
    esmCfg.deglitch = PMIC_ESM_DEGLITCH_MAX + 1;
    status = Pmic_esmSetCfg(&g_handle, &esmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

    helper_deinitPmic(&g_handle);
}

void test_neg_esm_esmSetCfg_invalidTimeBase(void)
{
    int32_t status;
    Pmic_EsmCfg_t esmCfg = {0};

    status = helper_initPmic(&g_handle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    esmCfg.validParams = PMIC_ESM_CFG_VALID;
    esmCfg.timeBase = PMIC_ESM_TIME_BASE_MAX + 1;
    status = Pmic_esmSetCfg(&g_handle, &esmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

    helper_deinitPmic(&g_handle);
}

void test_neg_esm_esmGetCfg_nullHandle(void)
{
    int32_t status;
    Pmic_EsmCfg_t esmCfg = {0};

    esmCfg.validParams = PMIC_ESM_CFG_VALID;
    status = Pmic_esmGetCfg(NULL, &esmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

void test_neg_esm_esmGetCfg_nullPointer(void)
{
    int32_t status;

    status = helper_initPmic(&g_handle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_esmGetCfg(&g_handle, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    helper_deinitPmic(&g_handle);
}

void test_neg_esm_esmGetCfg_invalidParams(void)
{
    int32_t status;
    Pmic_EsmCfg_t esmCfg = {0};

    status = helper_initPmic(&g_handle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    esmCfg.validParams = 0U;
    status = Pmic_esmGetCfg(&g_handle, &esmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

    helper_deinitPmic(&g_handle);
}

void test_neg_esm_esmGetStatus_nullHandle(void)
{
    int32_t status;
    Pmic_EsmStatus_t esmStatus = {0};

    esmStatus.validParams = PMIC_ESM_ERR_VALID_SHIFT;
    status = Pmic_esmGetStatus(NULL, &esmStatus);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

void test_neg_esm_esmGetStatus_nullPointer(void)
{
    int32_t status;

    status = helper_initPmic(&g_handle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_esmGetStatus(&g_handle, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    helper_deinitPmic(&g_handle);
}

void test_neg_esm_esmGetStatus_invalidParams(void)
{
    int32_t status;
    Pmic_EsmStatus_t esmStatus = {0};

    status = helper_initPmic(&g_handle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    esmStatus.validParams = 0U;
    status = Pmic_esmGetStatus(&g_handle, &esmStatus);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

    helper_deinitPmic(&g_handle);
}

void test_neg_esm_esmClrStatus_nullHandle(void)
{
    int32_t status;
    Pmic_EsmStatus_t esmStatus = {0};

    esmStatus.validParams = PMIC_ESM_ERR_VALID_SHIFT;
    status = Pmic_esmClrStatus(NULL, &esmStatus);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

void test_neg_esm_esmClrStatus_nullPointer(void)
{
    int32_t status;

    status = helper_initPmic(&g_handle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_esmClrStatus(&g_handle, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    helper_deinitPmic(&g_handle);
}

void test_neg_esm_esmClrStatus_invalidParams(void)
{
    int32_t status;
    Pmic_EsmStatus_t esmStatus = {0};

    status = helper_initPmic(&g_handle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    esmStatus.validParams = 0U;
    status = Pmic_esmClrStatus(&g_handle, &esmStatus);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

    helper_deinitPmic(&g_handle);
}

void test_neg_esm_esmClrStatus_unsupportedErrCnt(void)
{
    int32_t status;
    Pmic_EsmStatus_t esmStatus = {0};

    status = helper_initPmic(&g_handle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    /* Attempting to clear ERR_CNT is not supported on TPS65386x-Q1 */
    esmStatus.validParams = PMIC_ESM_ERR_CNT_VALID_SHIFT;
    status = Pmic_esmClrStatus(&g_handle, &esmStatus);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NOT_SUPPORTED, status);

    helper_deinitPmic(&g_handle);
}

/* ========================================================================== */
/*                         Unity Framework Functions                          */
/* ========================================================================== */

/* setUp() and tearDown() are defined in test_runner.c */

/**
 * @brief ESM test suite entry point (wrapper for test runner)
 * @param args Test arguments (unused)
 */
void esm_test(void *args)
{
    int32_t status;
    (void)args;  /* Unused parameter */

    /* Initialize once for all ESM tests */
    platform_init();
    testTimer_startModule("ESM");
    status = helper_initPmic(&g_handle);
    if (status != PMIC_ST_SUCCESS)
    {
        printf("ERROR: ESM test initialization failed with status: %d\r\n", status);
        platform_deinit();
        return;
    }

    /* Run all ESM tests */
    platform_setupTests();
    ESM_TEST_RUN_ALL();
    platform_tearDownTests();

    /* Cleanup */
    testTimer_endModule();
    helper_deinitPmic(&g_handle);
    platform_deinit();

}
