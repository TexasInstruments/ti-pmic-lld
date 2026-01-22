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


#include "../platform.h"
#include "esm_test.h"
#include "test_constants.h"

#ifdef BUILD_MOCK
#include "pmic_mock_types.h"
#include "pmic_mock_core.h"
#endif

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

static Pmic_Handle_t pmicHandle = {0};

/* ========================================================================== */
/*                       Negative Test Functions                              */
/* ========================================================================== */

/**
 * @brief Test Pmic_esmSetEnableState with NULL handle
 */
void test_neg_esm_esmSetEnableState_nullHandle(void)
{
    int32_t status = Pmic_esmSetEnableState(NULL, true);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_esmGetEnableState with NULL handle
 */
void test_neg_esm_esmGetEnableState_nullHandle(void)
{
    bool isEnabled = false;
    int32_t status = Pmic_esmGetEnableState(NULL, &isEnabled);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_esmGetEnableState with NULL isEnabled parameter
 */
void test_neg_esm_esmGetEnableState_nullIsEnabled(void)
{
    int32_t status = Pmic_esmGetEnableState(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_esmSetStartState with NULL handle
 */
void test_neg_esm_esmSetStartState_nullHandle(void)
{
    int32_t status = Pmic_esmSetStartState(NULL, true);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_esmGetStartState with NULL handle
 */
void test_neg_esm_esmGetStartState_nullHandle(void)
{
    bool started = false;
    int32_t status = Pmic_esmGetStartState(NULL, &started);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_esmGetStartState with NULL started parameter
 */
void test_neg_esm_esmGetStartState_nullStarted(void)
{
    int32_t status = Pmic_esmGetStartState(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_esmSetCfg with NULL handle
 */
void test_neg_esm_esmSetCfg_nullHandle(void)
{
    Pmic_EsmCfg_t esmCfg = {0};
    int32_t status = Pmic_esmSetCfg(NULL, &esmCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_esmSetCfg with NULL esmCfg parameter
 */
void test_neg_esm_esmSetCfg_nullEsmCfg(void)
{
    int32_t status = Pmic_esmSetCfg(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_esmSetCfg with invalid validParams (zero)
 */
void test_neg_esm_esmSetCfg_invalidValidParams(void)
{
    Pmic_EsmCfg_t esmCfg = {
        .validParams = 0U
    };
    int32_t status = Pmic_esmSetCfg(&pmicHandle, &esmCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_esmSetCfg with invalid mode value
 */
void test_neg_esm_esmSetCfg_invalidMode(void)
{
    Pmic_EsmCfg_t esmCfg = {
        .validParams = PMIC_ESM_MODE_VALID,
        .mode = PMIC_ESM_MODE_MAX + 1U
    };
    int32_t status = Pmic_esmSetCfg(&pmicHandle, &esmCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_esmSetCfg with invalid errCntThr value
 */
void test_neg_esm_esmSetCfg_invalidErrCntThr(void)
{
    Pmic_EsmCfg_t esmCfg = {
        .validParams = PMIC_ESM_ERR_CNT_THR_VALID,
        .errCntThr = PMIC_ESM_ERR_CNT_THR_MAX + 1U
    };
    int32_t status = Pmic_esmSetCfg(&pmicHandle, &esmCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_esmGetCfg with NULL handle
 */
void test_neg_esm_esmGetCfg_nullHandle(void)
{
    Pmic_EsmCfg_t esmCfg = {0};
    int32_t status = Pmic_esmGetCfg(NULL, &esmCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_esmGetCfg with NULL esmCfg parameter
 */
void test_neg_esm_esmGetCfg_nullEsmCfg(void)
{
    int32_t status = Pmic_esmGetCfg(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_esmGetCfg with invalid validParams (zero)
 */
void test_neg_esm_esmGetCfg_invalidValidParams(void)
{
    Pmic_EsmCfg_t esmCfg = {
        .validParams = 0U
    };
    int32_t status = Pmic_esmGetCfg(&pmicHandle, &esmCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_esmGetErrCnt with NULL handle
 */
void test_neg_esm_esmGetErrCnt_nullHandle(void)
{
    uint8_t errCnt = 0U;
    int32_t status = Pmic_esmGetErrCnt(NULL, &errCnt);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_esmGetErrCnt with NULL esmErrCnt parameter
 */
void test_neg_esm_esmGetErrCnt_nullEsmErrCnt(void)
{
    int32_t status = Pmic_esmGetErrCnt(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/* ========================================================================== */
/*                       Positive Test Functions                              */
/* ========================================================================== */

/**
 * @brief Test ESM enable and disable functionality
 */
void test_pos_esm_esmGetEnableState_enableDisable(void)
{
    bool isEnabled = false;
    int32_t status;

    /* Disable ESM */
    status = Pmic_esmSetEnableState(&pmicHandle, false);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Verify disabled state */
    status = Pmic_esmGetEnableState(&pmicHandle, &isEnabled);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(isEnabled == false);

    /* Enable ESM */
    status = Pmic_esmSetEnableState(&pmicHandle, true);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Verify enabled state */
    status = Pmic_esmGetEnableState(&pmicHandle, &isEnabled);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(isEnabled == true);
}

/**
 * @brief Test ESM start and stop functionality
 */
void test_pos_esm_esmGetStartState_startStop(void)
{
    bool started = false;
    int32_t status;

    /* Stop ESM */
    status = Pmic_esmSetStartState(&pmicHandle, false);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Verify stopped state */
    status = Pmic_esmGetStartState(&pmicHandle, &started);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(started == false);

    /* Start ESM */
    status = Pmic_esmSetStartState(&pmicHandle, true);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Verify started state */
    status = Pmic_esmGetStartState(&pmicHandle, &started);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(started == true);
}

/**
 * @brief Test ESM configuration set and get for mode
 */
void test_pos_esm_esmSetCfg_mode(void)
{
    Pmic_EsmCfg_t esmCfgSet = {
        .validParams = PMIC_ESM_MODE_VALID,
        .mode = PMIC_ESM_MODE_LEVEL
    };
    Pmic_EsmCfg_t esmCfgGet = {
        .validParams = PMIC_ESM_MODE_VALID
    };
    int32_t status;

    /* Set level mode */
    status = Pmic_esmSetCfg(&pmicHandle, &esmCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Get and verify */
    status = Pmic_esmGetCfg(&pmicHandle, &esmCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(esmCfgGet.mode == PMIC_ESM_MODE_LEVEL);

    /* Set PWM mode */
    esmCfgSet.mode = PMIC_ESM_MODE_PWM;
    status = Pmic_esmSetCfg(&pmicHandle, &esmCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Get and verify */
    status = Pmic_esmGetCfg(&pmicHandle, &esmCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(esmCfgGet.mode == PMIC_ESM_MODE_PWM);
}

/**
 * @brief Test ESM configuration set and get for error count threshold
 */
void test_pos_esm_esmSetCfg_errCntThr(void)
{
    Pmic_EsmCfg_t esmCfgSet = {
        .validParams = PMIC_ESM_ERR_CNT_THR_VALID
    };
    Pmic_EsmCfg_t esmCfgGet = {
        .validParams = PMIC_ESM_ERR_CNT_THR_VALID
    };
    int32_t status;
    uint8_t testValues[] = {0x0, 0x5, 0xA, 0xF};

    for (uint8_t i = 0; i < sizeof(testValues); i++)
    {
        esmCfgSet.errCntThr = testValues[i];
        status = Pmic_esmSetCfg(&pmicHandle, &esmCfgSet);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        status = Pmic_esmGetCfg(&pmicHandle, &esmCfgGet);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        PLATFORM_ASSERT(esmCfgGet.errCntThr == testValues[i]);
    }
}

/**
 * @brief Test ESM configuration set and get for delay1
 */
void test_pos_esm_esmSetCfg_delay1(void)
{
    Pmic_EsmCfg_t esmCfgSet = {
        .validParams = PMIC_ESM_DELAY1_VALID
    };
    Pmic_EsmCfg_t esmCfgGet = {
        .validParams = PMIC_ESM_DELAY1_VALID
    };
    int32_t status;
    uint8_t testValues[] = {0x00, 0x40, 0x80, 0xFF};

    for (uint8_t i = 0; i < sizeof(testValues); i++)
    {
        esmCfgSet.delay1 = testValues[i];
        status = Pmic_esmSetCfg(&pmicHandle, &esmCfgSet);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        status = Pmic_esmGetCfg(&pmicHandle, &esmCfgGet);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        PLATFORM_ASSERT(esmCfgGet.delay1 == testValues[i]);
    }
}

/**
 * @brief Test ESM configuration set and get for delay2
 */
void test_pos_esm_esmSetCfg_delay2(void)
{
    Pmic_EsmCfg_t esmCfgSet = {
        .validParams = PMIC_ESM_DELAY2_VALID
    };
    Pmic_EsmCfg_t esmCfgGet = {
        .validParams = PMIC_ESM_DELAY2_VALID
    };
    int32_t status;
    uint8_t testValues[] = {0x00, 0x40, 0x80, 0xFF};

    for (uint8_t i = 0; i < sizeof(testValues); i++)
    {
        esmCfgSet.delay2 = testValues[i];
        status = Pmic_esmSetCfg(&pmicHandle, &esmCfgSet);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        status = Pmic_esmGetCfg(&pmicHandle, &esmCfgGet);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        PLATFORM_ASSERT(esmCfgGet.delay2 == testValues[i]);
    }
}

/**
 * @brief Test ESM configuration set and get for hmax
 */
void test_pos_esm_esmSetCfg_hmax(void)
{
    Pmic_EsmCfg_t esmCfgSet = {
        .validParams = PMIC_ESM_HMAX_VALID
    };
    Pmic_EsmCfg_t esmCfgGet = {
        .validParams = PMIC_ESM_HMAX_VALID
    };
    int32_t status;
    uint8_t testValues[] = {0x00, 0x40, 0x80, 0xFF};

    for (uint8_t i = 0; i < sizeof(testValues); i++)
    {
        esmCfgSet.hmax = testValues[i];
        status = Pmic_esmSetCfg(&pmicHandle, &esmCfgSet);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        status = Pmic_esmGetCfg(&pmicHandle, &esmCfgGet);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        PLATFORM_ASSERT(esmCfgGet.hmax == testValues[i]);
    }
}

/**
 * @brief Test ESM configuration set and get for hmin
 */
void test_pos_esm_esmSetCfg_hmin(void)
{
    Pmic_EsmCfg_t esmCfgSet = {
        .validParams = PMIC_ESM_HMIN_VALID
    };
    Pmic_EsmCfg_t esmCfgGet = {
        .validParams = PMIC_ESM_HMIN_VALID
    };
    int32_t status;
    uint8_t testValues[] = {0x00, 0x40, 0x80, 0xFF};

    for (uint8_t i = 0; i < sizeof(testValues); i++)
    {
        esmCfgSet.hmin = testValues[i];
        status = Pmic_esmSetCfg(&pmicHandle, &esmCfgSet);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        status = Pmic_esmGetCfg(&pmicHandle, &esmCfgGet);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        PLATFORM_ASSERT(esmCfgGet.hmin == testValues[i]);
    }
}

/**
 * @brief Test ESM configuration set and get for lmax
 */
void test_pos_esm_esmSetCfg_lmax(void)
{
    Pmic_EsmCfg_t esmCfgSet = {
        .validParams = PMIC_ESM_LMAX_VALID
    };
    Pmic_EsmCfg_t esmCfgGet = {
        .validParams = PMIC_ESM_LMAX_VALID
    };
    int32_t status;
    uint8_t testValues[] = {0x00, 0x40, 0x80, 0xFF};

    for (uint8_t i = 0; i < sizeof(testValues); i++)
    {
        esmCfgSet.lmax = testValues[i];
        status = Pmic_esmSetCfg(&pmicHandle, &esmCfgSet);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        status = Pmic_esmGetCfg(&pmicHandle, &esmCfgGet);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        PLATFORM_ASSERT(esmCfgGet.lmax == testValues[i]);
    }
}

/**
 * @brief Test ESM configuration set and get for lmin
 */
void test_pos_esm_esmSetCfg_lmin(void)
{
    Pmic_EsmCfg_t esmCfgSet = {
        .validParams = PMIC_ESM_LMIN_VALID
    };
    Pmic_EsmCfg_t esmCfgGet = {
        .validParams = PMIC_ESM_LMIN_VALID
    };
    int32_t status;
    uint8_t testValues[] = {0x00, 0x40, 0x80, 0xFF};

    for (uint8_t i = 0; i < sizeof(testValues); i++)
    {
        esmCfgSet.lmin = testValues[i];
        status = Pmic_esmSetCfg(&pmicHandle, &esmCfgSet);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        status = Pmic_esmGetCfg(&pmicHandle, &esmCfgGet);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        PLATFORM_ASSERT(esmCfgGet.lmin == testValues[i]);
    }
}

/**
 * @brief Test ESM error count read functionality
 */
void test_pos_esm_esmGetErrCnt_getCount(void)
{
    uint8_t errCnt = 0U;
    int32_t status;

    /* Read error count */
    status = Pmic_esmGetErrCnt(&pmicHandle, &errCnt);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

/**
 * @brief Test combined configuration parameters
 */
void test_pos_esm_esmSetCfg_combinedConfiguration(void)
{
    int32_t status;

    /* Set multiple configuration parameters at once */
    Pmic_EsmCfg_t esmCfgSet = {
        .validParams = PMIC_ESM_MODE_VALID | PMIC_ESM_ERR_CNT_THR_VALID |
                       PMIC_ESM_DELAY1_VALID | PMIC_ESM_DELAY2_VALID,
        .mode = PMIC_ESM_MODE_LEVEL,
        .errCntThr = 0x5,
        .delay1 = 0x80,
        .delay2 = 0x40
    };

    status = Pmic_esmSetCfg(&pmicHandle, &esmCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Read back and verify */
    Pmic_EsmCfg_t esmCfgGet = {
        .validParams = PMIC_ESM_MODE_VALID | PMIC_ESM_ERR_CNT_THR_VALID |
                       PMIC_ESM_DELAY1_VALID | PMIC_ESM_DELAY2_VALID
    };

    status = Pmic_esmGetCfg(&pmicHandle, &esmCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(esmCfgGet.mode == esmCfgSet.mode);
    PLATFORM_ASSERT(esmCfgGet.errCntThr == esmCfgSet.errCntThr);
    PLATFORM_ASSERT(esmCfgGet.delay1 == esmCfgSet.delay1);
    PLATFORM_ASSERT(esmCfgGet.delay2 == esmCfgSet.delay2);
}

/**
 * @brief Test PWM mode configuration with timing parameters
 */
void test_pos_esm_esmSetCfg_pwmModeConfiguration(void)
{
    int32_t status;

    /* Configure ESM for PWM mode with all timing parameters */
    Pmic_EsmCfg_t esmCfgSet = {
        .validParams = PMIC_ESM_MODE_VALID | PMIC_ESM_HMIN_VALID |
                       PMIC_ESM_HMAX_VALID | PMIC_ESM_LMIN_VALID |
                       PMIC_ESM_LMAX_VALID,
        .mode = PMIC_ESM_MODE_PWM,
        .hmin = 0x20,
        .hmax = 0x80,
        .lmin = 0x10,
        .lmax = 0x60
    };

    status = Pmic_esmSetCfg(&pmicHandle, &esmCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Read back and verify */
    Pmic_EsmCfg_t esmCfgGet = {
        .validParams = PMIC_ESM_MODE_VALID | PMIC_ESM_HMIN_VALID |
                       PMIC_ESM_HMAX_VALID | PMIC_ESM_LMIN_VALID |
                       PMIC_ESM_LMAX_VALID
    };

    status = Pmic_esmGetCfg(&pmicHandle, &esmCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(esmCfgGet.mode == PMIC_ESM_MODE_PWM);
    PLATFORM_ASSERT(esmCfgGet.hmin == esmCfgSet.hmin);
    PLATFORM_ASSERT(esmCfgGet.hmax == esmCfgSet.hmax);
    PLATFORM_ASSERT(esmCfgGet.lmin == esmCfgSet.lmin);
    PLATFORM_ASSERT(esmCfgGet.lmax == esmCfgSet.lmax);
}

/**
 * @brief Test complete ESM configuration and enable sequence
 */
void test_pos_esm_integration_completeConfigurationSequence(void)
{
    int32_t status;
    bool isEnabled = false;
    bool started = false;

    /* Configure ESM with all parameters */
    Pmic_EsmCfg_t esmCfg = {
        .validParams = PMIC_ESM_MODE_VALID | PMIC_ESM_ERR_CNT_THR_VALID |
                       PMIC_ESM_DELAY1_VALID | PMIC_ESM_DELAY2_VALID |
                       PMIC_ESM_HMIN_VALID | PMIC_ESM_HMAX_VALID |
                       PMIC_ESM_LMIN_VALID | PMIC_ESM_LMAX_VALID,
        .mode = PMIC_ESM_MODE_PWM,
        .errCntThr = 0x3,
        .delay1 = 0x50,
        .delay2 = 0x30,
        .hmin = 0x20,
        .hmax = 0x80,
        .lmin = 0x10,
        .lmax = 0x60
    };

    /* Set configuration */
    status = Pmic_esmSetCfg(&pmicHandle, &esmCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Enable ESM */
    status = Pmic_esmSetEnableState(&pmicHandle, true);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Verify enabled */
    status = Pmic_esmGetEnableState(&pmicHandle, &isEnabled);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(isEnabled == true);

    /* Start ESM */
    status = Pmic_esmSetStartState(&pmicHandle, true);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Verify started */
    status = Pmic_esmGetStartState(&pmicHandle, &started);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(started == true);

    /* Stop ESM */
    status = Pmic_esmSetStartState(&pmicHandle, false);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Disable ESM */
    status = Pmic_esmSetEnableState(&pmicHandle, false);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

/**
 * @brief Test ESM configuration readback verification
 */
void test_pos_esm_esmSetCfg_configurationReadbackVerification(void)
{
    int32_t status;

    /* Set specific configuration */
    Pmic_EsmCfg_t esmCfgSet = {
        .validParams = PMIC_ESM_MODE_VALID | PMIC_ESM_ERR_CNT_THR_VALID |
                       PMIC_ESM_DELAY1_VALID | PMIC_ESM_DELAY2_VALID,
        .mode = PMIC_ESM_MODE_LEVEL,
        .errCntThr = 0xA,
        .delay1 = TEST_PATTERN_AA,
        .delay2 = TEST_PATTERN_55
    };

    status = Pmic_esmSetCfg(&pmicHandle, &esmCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Read back and verify each parameter */
    Pmic_EsmCfg_t esmCfgGet = {0};

    /* Verify mode */
    esmCfgGet.validParams = PMIC_ESM_MODE_VALID;
    status = Pmic_esmGetCfg(&pmicHandle, &esmCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(esmCfgGet.mode == PMIC_ESM_MODE_LEVEL);

    /* Verify error count threshold */
    esmCfgGet.validParams = PMIC_ESM_ERR_CNT_THR_VALID;
    status = Pmic_esmGetCfg(&pmicHandle, &esmCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(esmCfgGet.errCntThr == 0xA);

    /* Verify delay1 */
    esmCfgGet.validParams = PMIC_ESM_DELAY1_VALID;
    status = Pmic_esmGetCfg(&pmicHandle, &esmCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(esmCfgGet.delay1 == TEST_PATTERN_AA);

    /* Verify delay2 */
    esmCfgGet.validParams = PMIC_ESM_DELAY2_VALID;
    status = Pmic_esmGetCfg(&pmicHandle, &esmCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(esmCfgGet.delay2 == TEST_PATTERN_55);
}

/**
 * @brief Test ESM enable, configure, and start combined sequence
 */
void test_pos_esm_integration_enableConfigureStartSequence(void)
{
    int32_t status;
    bool isEnabled = false;
    bool started = false;

    /* Step 1: Enable ESM */
    status = Pmic_esmSetEnableState(&pmicHandle, true);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_esmGetEnableState(&pmicHandle, &isEnabled);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(isEnabled == true);

    /* Step 2: Configure ESM */
    Pmic_EsmCfg_t esmCfg = {
        .validParams = PMIC_ESM_MODE_VALID | PMIC_ESM_ERR_CNT_THR_VALID,
        .mode = PMIC_ESM_MODE_LEVEL,
        .errCntThr = 0x7
    };

    status = Pmic_esmSetCfg(&pmicHandle, &esmCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Step 3: Start ESM */
    status = Pmic_esmSetStartState(&pmicHandle, true);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_esmGetStartState(&pmicHandle, &started);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(started == true);

    /* Cleanup: Stop and disable */
    status = Pmic_esmSetStartState(&pmicHandle, false);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_esmSetEnableState(&pmicHandle, false);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

/**
 * @brief Test Pmic_esmStart wrapper function
 */
void test_pos_esm_esmStart_start(void)
{
    int32_t status;
    bool started = false;

    /* Call Pmic_esmStart wrapper */
    status = Pmic_esmStart(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Verify ESM is started */
    status = Pmic_esmGetStartState(&pmicHandle, &started);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(started == true);

    /* Cleanup */
    status = Pmic_esmSetStartState(&pmicHandle, false);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

/**
 * @brief Test Pmic_esmStop wrapper function
 */
void test_pos_esm_esmStop_stop(void)
{
    int32_t status;
    bool started = false;

    /* Start ESM first */
    status = Pmic_esmSetStartState(&pmicHandle, true);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Call Pmic_esmStop wrapper */
    status = Pmic_esmStop(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Verify ESM is stopped */
    status = Pmic_esmGetStartState(&pmicHandle, &started);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(started == false);
}

/**
 * @brief Test Pmic_esmStart with NULL handle
 */
void test_neg_esm_esmStart_nullHandle(void)
{
    int32_t status = Pmic_esmStart(NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_esmStop with NULL handle
 */
void test_neg_esm_esmStop_nullHandle(void)
{
    int32_t status = Pmic_esmStop(NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_esmGetStatus with NULL handle
 */
void test_neg_esm_esmGetStatus_nullHandle(void)
{
    Pmic_EsmStatus_t esmStat = {0};
    int32_t status = Pmic_esmGetStatus(NULL, &esmStat);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_esmGetStatus with NULL esmStat parameter
 */
void test_neg_esm_esmGetStatus_nullEsmStat(void)
{
    int32_t status = Pmic_esmGetStatus(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_esmGetStatus with zero validParams
 */
void test_neg_esm_esmGetStatus_zeroValidParams(void)
{
    Pmic_EsmStatus_t esmStat = {
        .validParams = 0U
    };
    int32_t status = Pmic_esmGetStatus(&pmicHandle, &esmStat);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_esmGetStatus with invalid validParams
 */
void test_neg_esm_esmGetStatus_invalidValidParams(void)
{
    Pmic_EsmStatus_t esmStat = {
        .validParams = PMIC_ESM_STATUS_ALL_VALID + 1U
    };
    int32_t status = Pmic_esmGetStatus(&pmicHandle, &esmStat);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_esmGetStatus retrieval of all status fields
 */
void test_pos_esm_esmGetStatus_allFields(void)
{
    Pmic_EsmStatus_t esmStat = {
        .validParams = PMIC_ESM_STATUS_ALL_VALID
    };
    int32_t status = Pmic_esmGetStatus(&pmicHandle, &esmStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

/**
 * @brief Test Pmic_esmGetStatus retrieval of rstInt field
 */
void test_pos_esm_esmGetStatus_rstInt(void)
{
    Pmic_EsmStatus_t esmStat = {
        .validParams = PMIC_ESM_RST_INT_VALID
    };
    int32_t status = Pmic_esmGetStatus(&pmicHandle, &esmStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

/**
 * @brief Test Pmic_esmGetStatus retrieval of failInt field
 */
void test_pos_esm_esmGetStatus_failInt(void)
{
    Pmic_EsmStatus_t esmStat = {
        .validParams = PMIC_ESM_FAIL_INT_VALID
    };
    int32_t status = Pmic_esmGetStatus(&pmicHandle, &esmStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

/**
 * @brief Test Pmic_esmGetStatus retrieval of pinInt field
 */
void test_pos_esm_esmGetStatus_pinInt(void)
{
    Pmic_EsmStatus_t esmStat = {
        .validParams = PMIC_ESM_PIN_INT_VALID
    };
    int32_t status = Pmic_esmGetStatus(&pmicHandle, &esmStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

/**
 * @brief Test Pmic_esmClrStatus with NULL handle
 */
void test_neg_esm_esmClrStatus_nullHandle(void)
{
    Pmic_EsmStatus_t esmStat = {0};
    int32_t status = Pmic_esmClrStatus(NULL, &esmStat);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_esmClrStatus with NULL esmStat parameter
 */
void test_neg_esm_esmClrStatus_nullEsmStat(void)
{
    int32_t status = Pmic_esmClrStatus(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_esmClrStatus with zero validParams
 */
void test_neg_esm_esmClrStatus_zeroValidParams(void)
{
    Pmic_EsmStatus_t esmStat = {
        .validParams = 0U
    };
    int32_t status = Pmic_esmClrStatus(&pmicHandle, &esmStat);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_esmClrStatus with invalid validParams
 */
void test_neg_esm_esmClrStatus_invalidValidParams(void)
{
    Pmic_EsmStatus_t esmStat = {
        .validParams = PMIC_ESM_STATUS_ALL_VALID + 1U
    };
    int32_t status = Pmic_esmClrStatus(&pmicHandle, &esmStat);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_esmClrStatus clearing all status fields
 */
void test_pos_esm_esmClrStatus_allFields(void)
{
    Pmic_EsmStatus_t esmStat = {
        .validParams = PMIC_ESM_STATUS_ALL_VALID
    };
    int32_t status = Pmic_esmClrStatus(&pmicHandle, &esmStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

/**
 * @brief Test Pmic_esmClrStatus clearing rstInt field
 */
void test_pos_esm_esmClrStatus_rstInt(void)
{
    Pmic_EsmStatus_t esmStat = {
        .validParams = PMIC_ESM_RST_INT_VALID
    };
    int32_t status = Pmic_esmClrStatus(&pmicHandle, &esmStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

/**
 * @brief Test Pmic_esmClrStatus clearing failInt field
 */
void test_pos_esm_esmClrStatus_failInt(void)
{
    Pmic_EsmStatus_t esmStat = {
        .validParams = PMIC_ESM_FAIL_INT_VALID
    };
    int32_t status = Pmic_esmClrStatus(&pmicHandle, &esmStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

/**
 * @brief Test Pmic_esmClrStatus clearing pinInt field
 */
void test_pos_esm_esmClrStatus_pinInt(void)
{
    Pmic_EsmStatus_t esmStat = {
        .validParams = PMIC_ESM_PIN_INT_VALID
    };
    int32_t status = Pmic_esmClrStatus(&pmicHandle, &esmStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

/* ========================================================================== */
/*                      I/O Error Injection Tests                             */
/* ========================================================================== */

/**
 * @brief Test Pmic_esmSetEnableState I/O read failure
 * Covers pmic_esm.c:231 - status = Pmic_ioRxByte(handle, ESM_MCU_MODE_CFG_REG, &regData);
 */
void test_neg_esm_esmSetEnableState_ioReadFailure(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t* mockDevice = platform_getMockDevice();
    int32_t status;

    PLATFORM_ASSERT(mockDevice != NULL);

    status = PmicMock_InjectError(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_esmSetEnableState(&pmicHandle, true);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    PLATFORM_ASSERT(true);
#endif
}

/**
 * @brief Test Pmic_esmGetEnableState I/O read failure
 * Covers pmic_esm.c:260 - status = Pmic_ioRxByte_CS(handle, ESM_MCU_MODE_CFG_REG, &regData);
 */
void test_neg_esm_esmGetEnableState_ioReadFailure(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t* mockDevice = platform_getMockDevice();
    int32_t status;
    bool isEnabled;

    PLATFORM_ASSERT(mockDevice != NULL);

    status = PmicMock_InjectError(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_esmGetEnableState(&pmicHandle, &isEnabled);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    PLATFORM_ASSERT(true);
#endif
}

/**
 * @brief Test Pmic_esmSetStartState I/O read failure
 * Covers pmic_esm.c:281 - status = Pmic_ioRxByte(handle, ESM_MCU_START_REG_REG, &regData);
 */
void test_neg_esm_esmSetStartState_ioReadFailure(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t* mockDevice = platform_getMockDevice();
    int32_t status;

    PLATFORM_ASSERT(mockDevice != NULL);

    status = PmicMock_InjectError(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_esmSetStartState(&pmicHandle, true);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    PLATFORM_ASSERT(true);
#endif
}

/**
 * @brief Test Pmic_esmGetStartState I/O read failure
 * Covers pmic_esm.c:310 - status = Pmic_ioRxByte_CS(handle, ESM_MCU_START_REG_REG, &regData);
 */
void test_neg_esm_esmGetStartState_ioReadFailure(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t* mockDevice = platform_getMockDevice();
    int32_t status;
    bool started;

    PLATFORM_ASSERT(mockDevice != NULL);

    status = PmicMock_InjectError(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_esmGetStartState(&pmicHandle, &started);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    PLATFORM_ASSERT(true);
#endif
}

/**
 * @brief Test Pmic_esmSetCfg I/O read failure in ESM_setDelayRegs (delay1)
 * Covers pmic_esm.c:105 - status = Pmic_ioRxByte(handle, ESM_MCU_DELAY1_REG_REG, &regData);
 */
void test_neg_esm_esmSetCfg_delay1ReadFailure(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t* mockDevice = platform_getMockDevice();
    int32_t status;
    Pmic_EsmCfg_t esmCfg = {
        .validParams = PMIC_ESM_DELAY1_VALID,
        .delay1 = 0x50
    };

    PLATFORM_ASSERT(mockDevice != NULL);

    status = PmicMock_InjectError(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_esmSetCfg(&pmicHandle, &esmCfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    PLATFORM_ASSERT(true);
#endif
}

/**
 * @brief Test Pmic_esmSetCfg I/O read failure in ESM_setHmaxHminRegs (hmax)
 * Covers pmic_esm.c:147 - status = Pmic_ioRxByte(handle, ESM_MCU_HMAX_REG_REG, &regData);
 */
void test_neg_esm_esmSetCfg_hmaxReadFailure(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t* mockDevice = platform_getMockDevice();
    int32_t status;
    Pmic_EsmCfg_t esmCfg = {
        .validParams = PMIC_ESM_HMAX_VALID,
        .hmax = 0x80
    };

    PLATFORM_ASSERT(mockDevice != NULL);

    status = PmicMock_InjectError(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_esmSetCfg(&pmicHandle, &esmCfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    PLATFORM_ASSERT(true);
#endif
}

/**
 * @brief Test Pmic_esmSetCfg I/O read failure in ESM_setLmaxLminRegs (lmax)
 * Covers pmic_esm.c:189 - status = Pmic_ioRxByte(handle, ESM_MCU_LMAX_REG_REG, &regData);
 */
void test_neg_esm_esmSetCfg_lmaxReadFailure(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t* mockDevice = platform_getMockDevice();
    int32_t status;
    Pmic_EsmCfg_t esmCfg = {
        .validParams = PMIC_ESM_LMAX_VALID,
        .lmax = 0x90
    };

    PLATFORM_ASSERT(mockDevice != NULL);

    status = PmicMock_InjectError(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_esmSetCfg(&pmicHandle, &esmCfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    PLATFORM_ASSERT(true);
#endif
}

/**
 * @brief Test Pmic_esmSetCfg I/O read failure in ESM_setModeCfg
 * Covers pmic_esm.c:57 - status = Pmic_ioRxByte(handle, ESM_MCU_MODE_CFG_REG, &regData);
 */
void test_neg_esm_esmSetCfg_modeCfgReadFailure(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t* mockDevice = platform_getMockDevice();
    int32_t status;
    Pmic_EsmCfg_t esmCfg = {
        .validParams = PMIC_ESM_MODE_VALID,
        .mode = PMIC_ESM_MODE_LEVEL
    };

    PLATFORM_ASSERT(mockDevice != NULL);

    status = PmicMock_InjectError(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_esmSetCfg(&pmicHandle, &esmCfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    PLATFORM_ASSERT(true);
#endif
}

/**
 * @brief Test Pmic_esmGetCfg I/O read failure in ESM_readModeCfg
 * Covers pmic_esm.c:373 - status = Pmic_ioRxByte_CS(handle, ESM_MCU_MODE_CFG_REG, &regData);
 */
void test_neg_esm_esmGetCfg_modeCfgReadFailure(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t* mockDevice = platform_getMockDevice();
    int32_t status;
    Pmic_EsmCfg_t esmCfg = {
        .validParams = PMIC_ESM_MODE_VALID
    };

    PLATFORM_ASSERT(mockDevice != NULL);

    status = PmicMock_InjectError(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_esmGetCfg(&pmicHandle, &esmCfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    PLATFORM_ASSERT(true);
#endif
}

/**
 * @brief Test Pmic_esmGetCfg I/O read failure in ESM_readDelayRegs (delay1)
 * Covers pmic_esm.c:402 - status = Pmic_ioRxByte_CS(handle, ESM_MCU_DELAY1_REG_REG, &regData);
 */
void test_neg_esm_esmGetCfg_delay1ReadFailure(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t* mockDevice = platform_getMockDevice();
    int32_t status;
    Pmic_EsmCfg_t esmCfg = {
        .validParams = PMIC_ESM_DELAY1_VALID
    };

    PLATFORM_ASSERT(mockDevice != NULL);

    status = PmicMock_InjectError(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_esmGetCfg(&pmicHandle, &esmCfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    PLATFORM_ASSERT(true);
#endif
}

/**
 * @brief Test Pmic_esmGetCfg I/O read failure in ESM_readHmaxHminRegs (hmax)
 * Covers pmic_esm.c:434 - status = Pmic_ioRxByte_CS(handle, ESM_MCU_HMAX_REG_REG, &regData);
 */
void test_neg_esm_esmGetCfg_hmaxReadFailure(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t* mockDevice = platform_getMockDevice();
    int32_t status;
    Pmic_EsmCfg_t esmCfg = {
        .validParams = PMIC_ESM_HMAX_VALID
    };

    PLATFORM_ASSERT(mockDevice != NULL);

    status = PmicMock_InjectError(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_esmGetCfg(&pmicHandle, &esmCfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    PLATFORM_ASSERT(true);
#endif
}

/**
 * @brief Test Pmic_esmGetCfg I/O read failure in ESM_readLmaxLminRegs (lmax)
 * Covers pmic_esm.c:466 - status = Pmic_ioRxByte_CS(handle, ESM_MCU_LMAX_REG_REG, &regData);
 */
void test_neg_esm_esmGetCfg_lmaxReadFailure(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t* mockDevice = platform_getMockDevice();
    int32_t status;
    Pmic_EsmCfg_t esmCfg = {
        .validParams = PMIC_ESM_LMAX_VALID
    };

    PLATFORM_ASSERT(mockDevice != NULL);

    status = PmicMock_InjectError(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_esmGetCfg(&pmicHandle, &esmCfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    PLATFORM_ASSERT(true);
#endif
}

/**
 * @brief Test Pmic_esmGetErrCnt I/O read failure
 * Covers pmic_esm.c:546 - status = Pmic_ioRxByte_CS(handle, ESM_MCU_ERR_CNT_REG_REG, &regData);
 */
void test_neg_esm_esmGetErrCnt_ioReadFailure(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t* mockDevice = platform_getMockDevice();
    int32_t status;
    uint8_t esmErrCnt;

    PLATFORM_ASSERT(mockDevice != NULL);

    status = PmicMock_InjectError(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_esmGetErrCnt(&pmicHandle, &esmErrCnt);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    PLATFORM_ASSERT(true);
#endif
}

/**
 * @brief Test Pmic_esmGetStatus I/O read failure
 * Covers pmic_esm.c:586 - status = Pmic_ioRxByte_CS(handle, INT_ESM_REG, &regData);
 */
void test_neg_esm_esmGetStatus_ioReadFailure(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t* mockDevice = platform_getMockDevice();
    int32_t status;
    Pmic_EsmStatus_t esmStat = {
        .validParams = PMIC_ESM_RST_INT_VALID
    };

    PLATFORM_ASSERT(mockDevice != NULL);

    status = PmicMock_InjectError(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_esmGetStatus(&pmicHandle, &esmStat);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    PLATFORM_ASSERT(true);
#endif
}

/**
 * @brief Test Pmic_esmClrStatus I/O write failure
 * Covers pmic_esm.c:639 - status = Pmic_ioTxByte_CS(handle, INT_ESM_REG, regData);
 */
void test_neg_esm_esmClrStatus_ioWriteFailure(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t* mockDevice = platform_getMockDevice();
    int32_t status;
    Pmic_EsmStatus_t esmStat = {
        .validParams = PMIC_ESM_RST_INT_VALID
    };

    PLATFORM_ASSERT(mockDevice != NULL);

    status = PmicMock_InjectError(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_esmClrStatus(&pmicHandle, &esmStat);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    PLATFORM_ASSERT(true);
#endif
}

/* ========================================================================== */
/*                    Cascading Failure Tests                                 */
/* ========================================================================== */

/**
 * @brief Test Pmic_esmSetCfg cascading failure from ESM_setDelayRegs to ESM_setHmaxHminRegs
 * Covers pmic_esm.c:345-347 - if (status == PMIC_ST_SUCCESS) { status = ESM_setHmaxHminRegs(...) }
 */
void test_neg_esm_esmSetCfg_cascadeFailure_delayToHmax(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t* mockDevice = platform_getMockDevice();
    int32_t status;
    Pmic_EsmCfg_t esmCfg = {
        .validParams = PMIC_ESM_DELAY1_VALID | PMIC_ESM_HMAX_VALID,
        .delay1 = 0x50,
        .hmax = 0x80
    };

    PLATFORM_ASSERT(mockDevice != NULL);

    /* Inject error to fail during delay1 operation */
    status = PmicMock_InjectError(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_esmSetCfg(&pmicHandle, &esmCfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    PLATFORM_ASSERT(true);
#endif
}

/**
 * @brief Test Pmic_esmSetCfg cascading failure from ESM_setHmaxHminRegs to ESM_setLmaxLminRegs
 * Covers pmic_esm.c:350-353 - if (status == PMIC_ST_SUCCESS) { status = ESM_setLmaxLminRegs(...) }
 */
void test_neg_esm_esmSetCfg_cascadeFailure_hmaxToLmax(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t* mockDevice = platform_getMockDevice();
    int32_t status;
    Pmic_EsmCfg_t esmCfg = {
        .validParams = PMIC_ESM_HMAX_VALID | PMIC_ESM_LMAX_VALID,
        .hmax = 0x80,
        .lmax = 0x90
    };

    PLATFORM_ASSERT(mockDevice != NULL);

    /* Inject error to fail during hmax operation */
    status = PmicMock_InjectError(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_esmSetCfg(&pmicHandle, &esmCfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    PLATFORM_ASSERT(true);
#endif
}

/**
 * @brief Test Pmic_esmGetCfg cascading failure from ESM_readModeCfg to ESM_readDelayRegs
 * Covers pmic_esm.c:513-515 - if (status == PMIC_ST_SUCCESS) { status = ESM_readDelayRegs(...) }
 */
void test_neg_esm_esmGetCfg_cascadeFailure_modeToDelay(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t* mockDevice = platform_getMockDevice();
    int32_t status;
    Pmic_EsmCfg_t esmCfg = {
        .validParams = PMIC_ESM_MODE_VALID | PMIC_ESM_DELAY1_VALID
    };

    PLATFORM_ASSERT(mockDevice != NULL);

    /* Inject error to fail during mode read */
    status = PmicMock_InjectError(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_esmGetCfg(&pmicHandle, &esmCfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    PLATFORM_ASSERT(true);
#endif
}

/**
 * @brief Test Pmic_esmGetCfg cascading failure from ESM_readDelayRegs to ESM_readHmaxHminRegs
 * Covers pmic_esm.c:518-521 - if (status == PMIC_ST_SUCCESS) { status = ESM_readHmaxHminRegs(...) }
 */
void test_neg_esm_esmGetCfg_cascadeFailure_delayToHmax(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t* mockDevice = platform_getMockDevice();
    int32_t status;
    Pmic_EsmCfg_t esmCfg = {
        .validParams = PMIC_ESM_DELAY1_VALID | PMIC_ESM_HMAX_VALID
    };

    PLATFORM_ASSERT(mockDevice != NULL);

    /* Inject error to fail during delay1 read */
    status = PmicMock_InjectError(mockDevice, PMIC_MOCK_ERROR_COMM_FAILURE, 1);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    status = Pmic_esmGetCfg(&pmicHandle, &esmCfg);
    PLATFORM_ASSERT(status != PMIC_ST_SUCCESS);
#else
    PLATFORM_ASSERT(true);
#endif
}

/* ========================================================================== */
/*                  Partial ValidParams Tests                                 */
/* ========================================================================== */

/**
 * @brief Test setting only delay2 (without delay1)
 * Covers pmic_esm.c:118 - second param branch in ESM_setDelayRegs
 */
void test_pos_esm_esmSetCfg_delay2Only(void)
{
    Pmic_EsmCfg_t esmCfgSet = {
        .validParams = PMIC_ESM_DELAY2_VALID,
        .delay2 = 0x75
    };
    Pmic_EsmCfg_t esmCfgGet = {
        .validParams = PMIC_ESM_DELAY2_VALID
    };
    int32_t status;

    status = Pmic_esmSetCfg(&pmicHandle, &esmCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_esmGetCfg(&pmicHandle, &esmCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(esmCfgGet.delay2 == 0x75);
}

/**
 * @brief Test setting only hmin (without hmax)
 * Covers pmic_esm.c:160 - second param branch in ESM_setHmaxHminRegs
 */
void test_pos_esm_esmSetCfg_hminOnly(void)
{
    Pmic_EsmCfg_t esmCfgSet = {
        .validParams = PMIC_ESM_HMIN_VALID,
        .hmin = 0x30
    };
    Pmic_EsmCfg_t esmCfgGet = {
        .validParams = PMIC_ESM_HMIN_VALID
    };
    int32_t status;

    status = Pmic_esmSetCfg(&pmicHandle, &esmCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_esmGetCfg(&pmicHandle, &esmCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(esmCfgGet.hmin == 0x30);
}

/**
 * @brief Test setting only lmin (without lmax)
 * Covers pmic_esm.c:202 - second param branch in ESM_setLmaxLminRegs
 */
void test_pos_esm_esmSetCfg_lminOnly(void)
{
    Pmic_EsmCfg_t esmCfgSet = {
        .validParams = PMIC_ESM_LMIN_VALID,
        .lmin = 0x15
    };
    Pmic_EsmCfg_t esmCfgGet = {
        .validParams = PMIC_ESM_LMIN_VALID
    };
    int32_t status;

    status = Pmic_esmSetCfg(&pmicHandle, &esmCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_esmGetCfg(&pmicHandle, &esmCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(esmCfgGet.lmin == 0x15);
}

/**
 * @brief Test getting only delay2 (without delay1)
 * Covers pmic_esm.c:411 - second param branch in ESM_readDelayRegs
 */
void test_pos_esm_esmGetCfg_delay2Only(void)
{
    Pmic_EsmCfg_t esmCfgSet = {
        .validParams = PMIC_ESM_DELAY2_VALID,
        .delay2 = 0x88
    };
    Pmic_EsmCfg_t esmCfgGet = {
        .validParams = PMIC_ESM_DELAY2_VALID
    };
    int32_t status;

    /* First set delay2 */
    status = Pmic_esmSetCfg(&pmicHandle, &esmCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Now get only delay2 */
    status = Pmic_esmGetCfg(&pmicHandle, &esmCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(esmCfgGet.delay2 == 0x88);
}

/**
 * @brief Test getting only hmin (without hmax)
 * Covers pmic_esm.c:443 - second param branch in ESM_readHmaxHminRegs
 */
void test_pos_esm_esmGetCfg_hminOnly(void)
{
    Pmic_EsmCfg_t esmCfgSet = {
        .validParams = PMIC_ESM_HMIN_VALID,
        .hmin = 0x22
    };
    Pmic_EsmCfg_t esmCfgGet = {
        .validParams = PMIC_ESM_HMIN_VALID
    };
    int32_t status;

    /* First set hmin */
    status = Pmic_esmSetCfg(&pmicHandle, &esmCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Now get only hmin */
    status = Pmic_esmGetCfg(&pmicHandle, &esmCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(esmCfgGet.hmin == 0x22);
}

/**
 * @brief Test getting only lmin (without lmax)
 * Covers pmic_esm.c:475 - second param branch in ESM_readLmaxLminRegs
 */
void test_pos_esm_esmGetCfg_lminOnly(void)
{
    Pmic_EsmCfg_t esmCfgSet = {
        .validParams = PMIC_ESM_LMIN_VALID,
        .lmin = 0x11
    };
    Pmic_EsmCfg_t esmCfgGet = {
        .validParams = PMIC_ESM_LMIN_VALID
    };
    int32_t status;

    /* First set lmin */
    status = Pmic_esmSetCfg(&pmicHandle, &esmCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Now get only lmin */
    status = Pmic_esmGetCfg(&pmicHandle, &esmCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(esmCfgGet.lmin == 0x11);
}

/* ========================================================================== */
/*                    API-Specific Test Macros - esmClrStatus                 */
/* ========================================================================== */

#define ESM_TEST_POS_ESMCLRSTATUS() \
    PLATFORM_RUN_TEST(test_pos_esm_esmClrStatus_allFields); \
    PLATFORM_RUN_TEST(test_pos_esm_esmClrStatus_failInt); \
    PLATFORM_RUN_TEST(test_pos_esm_esmClrStatus_pinInt); \
    PLATFORM_RUN_TEST(test_pos_esm_esmClrStatus_rstInt)

#define ESM_TEST_NEG_ESMCLRSTATUS() \
    PLATFORM_RUN_TEST(test_neg_esm_esmClrStatus_invalidValidParams); \
    PLATFORM_RUN_TEST(test_neg_esm_esmClrStatus_ioWriteFailure); \
    PLATFORM_RUN_TEST(test_neg_esm_esmClrStatus_nullEsmStat); \
    PLATFORM_RUN_TEST(test_neg_esm_esmClrStatus_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_esm_esmClrStatus_zeroValidParams)

#define ESM_TEST_ESMCLRSTATUS() \
    ESM_TEST_POS_ESMCLRSTATUS(); \
    ESM_TEST_NEG_ESMCLRSTATUS()

/* ========================================================================== */
/*                    API-Specific Test Macros - esmGetCfg                    */
/* ========================================================================== */

#define ESM_TEST_POS_ESMGETCFG() \
    PLATFORM_RUN_TEST(test_pos_esm_esmGetCfg_delay1); \
    PLATFORM_RUN_TEST(test_pos_esm_esmGetCfg_delay2); \
    PLATFORM_RUN_TEST(test_pos_esm_esmGetCfg_delay2Only); \
    PLATFORM_RUN_TEST(test_pos_esm_esmGetCfg_errCntThr); \
    PLATFORM_RUN_TEST(test_pos_esm_esmGetCfg_hmax); \
    PLATFORM_RUN_TEST(test_pos_esm_esmGetCfg_hmin); \
    PLATFORM_RUN_TEST(test_pos_esm_esmGetCfg_hminOnly); \
    PLATFORM_RUN_TEST(test_pos_esm_esmGetCfg_lmax); \
    PLATFORM_RUN_TEST(test_pos_esm_esmGetCfg_lmin); \
    PLATFORM_RUN_TEST(test_pos_esm_esmGetCfg_lminOnly); \
    PLATFORM_RUN_TEST(test_pos_esm_esmGetCfg_mode)

#define ESM_TEST_NEG_ESMGETCFG() \
    PLATFORM_RUN_TEST(test_neg_esm_esmGetCfg_cascadeFailure_delayToHmax); \
    PLATFORM_RUN_TEST(test_neg_esm_esmGetCfg_cascadeFailure_modeToDelay); \
    PLATFORM_RUN_TEST(test_neg_esm_esmGetCfg_delay1ReadFailure); \
    PLATFORM_RUN_TEST(test_neg_esm_esmGetCfg_hmaxReadFailure); \
    PLATFORM_RUN_TEST(test_neg_esm_esmGetCfg_invalidValidParams); \
    PLATFORM_RUN_TEST(test_neg_esm_esmGetCfg_lmaxReadFailure); \
    PLATFORM_RUN_TEST(test_neg_esm_esmGetCfg_modeCfgReadFailure); \
    PLATFORM_RUN_TEST(test_neg_esm_esmGetCfg_nullEsmCfg); \
    PLATFORM_RUN_TEST(test_neg_esm_esmGetCfg_nullHandle)

#define ESM_TEST_ESMGETCFG() \
    ESM_TEST_POS_ESMGETCFG(); \
    ESM_TEST_NEG_ESMGETCFG()

/* ========================================================================== */
/*                    API-Specific Test Macros - esmGetEnableState            */
/* ========================================================================== */

#define ESM_TEST_POS_ESMGETENABLESTATE() \
    PLATFORM_RUN_TEST(test_pos_esm_esmGetEnableState_enableDisable)

#define ESM_TEST_NEG_ESMGETENABLESTATE() \
    PLATFORM_RUN_TEST(test_neg_esm_esmGetEnableState_ioReadFailure); \
    PLATFORM_RUN_TEST(test_neg_esm_esmGetEnableState_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_esm_esmGetEnableState_nullIsEnabled)

#define ESM_TEST_ESMGETENABLESTATE() \
    ESM_TEST_POS_ESMGETENABLESTATE(); \
    ESM_TEST_NEG_ESMGETENABLESTATE()

/* ========================================================================== */
/*                    API-Specific Test Macros - esmGetErrCnt                 */
/* ========================================================================== */

#define ESM_TEST_POS_ESMGETERRCNT() \
    PLATFORM_RUN_TEST(test_pos_esm_esmGetErrCnt_getCount)

#define ESM_TEST_NEG_ESMGETERRCNT() \
    PLATFORM_RUN_TEST(test_neg_esm_esmGetErrCnt_ioReadFailure); \
    PLATFORM_RUN_TEST(test_neg_esm_esmGetErrCnt_nullEsmErrCnt); \
    PLATFORM_RUN_TEST(test_neg_esm_esmGetErrCnt_nullHandle)

#define ESM_TEST_ESMGETERRCNT() \
    ESM_TEST_POS_ESMGETERRCNT(); \
    ESM_TEST_NEG_ESMGETERRCNT()

/* ========================================================================== */
/*                    API-Specific Test Macros - esmGetStartState             */
/* ========================================================================== */

#define ESM_TEST_POS_ESMGETSTARTSTATE() \
    PLATFORM_RUN_TEST(test_pos_esm_esmGetStartState_startStop)

#define ESM_TEST_NEG_ESMGETSTARTSTATE() \
    PLATFORM_RUN_TEST(test_neg_esm_esmGetStartState_ioReadFailure); \
    PLATFORM_RUN_TEST(test_neg_esm_esmGetStartState_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_esm_esmGetStartState_nullStarted)

#define ESM_TEST_ESMGETSTARTSTATE() \
    ESM_TEST_POS_ESMGETSTARTSTATE(); \
    ESM_TEST_NEG_ESMGETSTARTSTATE()

/* ========================================================================== */
/*                    API-Specific Test Macros - esmGetStatus                 */
/* ========================================================================== */

#define ESM_TEST_POS_ESMGETSTATUS() \
    PLATFORM_RUN_TEST(test_pos_esm_esmGetStatus_allFields); \
    PLATFORM_RUN_TEST(test_pos_esm_esmGetStatus_failInt); \
    PLATFORM_RUN_TEST(test_pos_esm_esmGetStatus_pinInt); \
    PLATFORM_RUN_TEST(test_pos_esm_esmGetStatus_rstInt)

#define ESM_TEST_NEG_ESMGETSTATUS() \
    PLATFORM_RUN_TEST(test_neg_esm_esmGetStatus_invalidValidParams); \
    PLATFORM_RUN_TEST(test_neg_esm_esmGetStatus_ioReadFailure); \
    PLATFORM_RUN_TEST(test_neg_esm_esmGetStatus_nullEsmStat); \
    PLATFORM_RUN_TEST(test_neg_esm_esmGetStatus_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_esm_esmGetStatus_zeroValidParams)

#define ESM_TEST_ESMGETSTATUS() \
    ESM_TEST_POS_ESMGETSTATUS(); \
    ESM_TEST_NEG_ESMGETSTATUS()

/* ========================================================================== */
/*                    API-Specific Test Macros - esmSetCfg                    */
/* ========================================================================== */

#define ESM_TEST_POS_ESMSETCFG() \
    PLATFORM_RUN_TEST(test_pos_esm_esmSetCfg_combinedConfiguration); \
    PLATFORM_RUN_TEST(test_pos_esm_esmSetCfg_configurationReadbackVerification); \
    PLATFORM_RUN_TEST(test_pos_esm_esmSetCfg_delay1); \
    PLATFORM_RUN_TEST(test_pos_esm_esmSetCfg_delay2); \
    PLATFORM_RUN_TEST(test_pos_esm_esmSetCfg_delay2Only); \
    PLATFORM_RUN_TEST(test_pos_esm_esmSetCfg_errCntThr); \
    PLATFORM_RUN_TEST(test_pos_esm_esmSetCfg_hmax); \
    PLATFORM_RUN_TEST(test_pos_esm_esmSetCfg_hmin); \
    PLATFORM_RUN_TEST(test_pos_esm_esmSetCfg_hminOnly); \
    PLATFORM_RUN_TEST(test_pos_esm_esmSetCfg_lmax); \
    PLATFORM_RUN_TEST(test_pos_esm_esmSetCfg_lmin); \
    PLATFORM_RUN_TEST(test_pos_esm_esmSetCfg_lminOnly); \
    PLATFORM_RUN_TEST(test_pos_esm_esmSetCfg_mode); \
    PLATFORM_RUN_TEST(test_pos_esm_esmSetCfg_pwmModeConfiguration)

#define ESM_TEST_NEG_ESMSETCFG() \
    PLATFORM_RUN_TEST(test_neg_esm_esmSetCfg_cascadeFailure_delayToHmax); \
    PLATFORM_RUN_TEST(test_neg_esm_esmSetCfg_cascadeFailure_hmaxToLmax); \
    PLATFORM_RUN_TEST(test_neg_esm_esmSetCfg_delay1ReadFailure); \
    PLATFORM_RUN_TEST(test_neg_esm_esmSetCfg_hmaxReadFailure); \
    PLATFORM_RUN_TEST(test_neg_esm_esmSetCfg_invalidErrCntThr); \
    PLATFORM_RUN_TEST(test_neg_esm_esmSetCfg_invalidMode); \
    PLATFORM_RUN_TEST(test_neg_esm_esmSetCfg_invalidValidParams); \
    PLATFORM_RUN_TEST(test_neg_esm_esmSetCfg_lmaxReadFailure); \
    PLATFORM_RUN_TEST(test_neg_esm_esmSetCfg_modeCfgReadFailure); \
    PLATFORM_RUN_TEST(test_neg_esm_esmSetCfg_nullEsmCfg); \
    PLATFORM_RUN_TEST(test_neg_esm_esmSetCfg_nullHandle)

#define ESM_TEST_ESMSETCFG() \
    ESM_TEST_POS_ESMSETCFG(); \
    ESM_TEST_NEG_ESMSETCFG()

/* ========================================================================== */
/*                    API-Specific Test Macros - esmSetEnableState            */
/* ========================================================================== */

#define ESM_TEST_POS_ESMSETENABLESTATE() \
    /* Positive tests combined with esmGetEnableState */

#define ESM_TEST_NEG_ESMSETENABLESTATE() \
    PLATFORM_RUN_TEST(test_neg_esm_esmSetEnableState_ioReadFailure); \
    PLATFORM_RUN_TEST(test_neg_esm_esmSetEnableState_nullHandle)

#define ESM_TEST_ESMSETENABLESTATE() \
    ESM_TEST_POS_ESMSETENABLESTATE(); \
    ESM_TEST_NEG_ESMSETENABLESTATE()

/* ========================================================================== */
/*                    API-Specific Test Macros - esmSetStartState             */
/* ========================================================================== */

#define ESM_TEST_POS_ESMSETSTARTSTATE() \
    /* Positive tests combined with esmGetStartState */

#define ESM_TEST_NEG_ESMSETSTARTSTATE() \
    PLATFORM_RUN_TEST(test_neg_esm_esmSetStartState_ioReadFailure); \
    PLATFORM_RUN_TEST(test_neg_esm_esmSetStartState_nullHandle)

#define ESM_TEST_ESMSETSTARTSTATE() \
    ESM_TEST_POS_ESMSETSTARTSTATE(); \
    ESM_TEST_NEG_ESMSETSTARTSTATE()

/* ========================================================================== */
/*                    API-Specific Test Macros - esmStart                     */
/* ========================================================================== */

#define ESM_TEST_POS_ESMSTART() \
    PLATFORM_RUN_TEST(test_pos_esm_esmStart_start)

#define ESM_TEST_NEG_ESMSTART() \
    PLATFORM_RUN_TEST(test_neg_esm_esmStart_nullHandle)

#define ESM_TEST_ESMSTART() \
    ESM_TEST_POS_ESMSTART(); \
    ESM_TEST_NEG_ESMSTART()

/* ========================================================================== */
/*                    API-Specific Test Macros - esmStop                      */
/* ========================================================================== */

#define ESM_TEST_POS_ESMSTOP() \
    PLATFORM_RUN_TEST(test_pos_esm_esmStop_stop)

#define ESM_TEST_NEG_ESMSTOP() \
    PLATFORM_RUN_TEST(test_neg_esm_esmStop_nullHandle)

#define ESM_TEST_ESMSTOP() \
    ESM_TEST_POS_ESMSTOP(); \
    ESM_TEST_NEG_ESMSTOP()

/* ========================================================================== */
/*                         Aggregate Test Runners                             */
/* ========================================================================== */

#define ESM_TEST_RUN_POSITIVE() \
    ESM_TEST_POS_ESMCLRSTATUS(); \
    ESM_TEST_POS_ESMGETCFG(); \
    ESM_TEST_POS_ESMGETENABLESTATE(); \
    ESM_TEST_POS_ESMGETERRCNT(); \
    ESM_TEST_POS_ESMGETSTARTSTATE(); \
    ESM_TEST_POS_ESMGETSTATUS(); \
    ESM_TEST_POS_ESMSETCFG(); \
    ESM_TEST_POS_ESMSETENABLESTATE(); \
    ESM_TEST_POS_ESMSETSTARTSTATE(); \
    ESM_TEST_POS_ESMSTART(); \
    ESM_TEST_POS_ESMSTOP(); \
    PLATFORM_RUN_TEST(test_pos_esm_integration_completeConfigurationSequence); \
    PLATFORM_RUN_TEST(test_pos_esm_integration_enableConfigureStartSequence)

#define ESM_TEST_RUN_NEGATIVE() \
    ESM_TEST_NEG_ESMCLRSTATUS(); \
    ESM_TEST_NEG_ESMGETCFG(); \
    ESM_TEST_NEG_ESMGETENABLESTATE(); \
    ESM_TEST_NEG_ESMGETERRCNT(); \
    ESM_TEST_NEG_ESMGETSTARTSTATE(); \
    ESM_TEST_NEG_ESMGETSTATUS(); \
    ESM_TEST_NEG_ESMSETCFG(); \
    ESM_TEST_NEG_ESMSETENABLESTATE(); \
    ESM_TEST_NEG_ESMSETSTARTSTATE(); \
    ESM_TEST_NEG_ESMSTART(); \
    ESM_TEST_NEG_ESMSTOP()

#define ESM_TEST_RUN_ALL() \
    ESM_TEST_RUN_POSITIVE(); \
    ESM_TEST_RUN_NEGATIVE()

/* ========================================================================== */
/*                         Entry Point Function                               */
/* ========================================================================== */

void esm_test(void *args)
{
    (void)args;
    int32_t status;

    platform_init();
    platform_setupTests();

    /* Initialize PMIC handle */
    Pmic_HandleCfg_t handleCfg = {
        .validParams = (PMIC_COMM_MODE_VALID |
                        PMIC_I2C_ADDR0_VALID |
                        PMIC_COMM_HANDLE_0_VALID |
                        PMIC_IO_READ_VALID |
                        PMIC_IO_WRITE_VALID |
                        PMIC_CRITICAL_SECTION_START_VALID |
                        PMIC_CRITICAL_SECTION_STOP_VALID |
                        PMIC_CRC_ENABLE_VALID |
                        PMIC_CONFIG_CRC_ENABLE_VALID |
                        PMIC_IRQ_RESPONSE_CALLBACK_VALID),
        .commMode = PMIC_INTF_I2C_SINGLE,
        .i2cAddr0 = PLATFORM_TARGET_I2C_ADDR,
        .crcEnable = PMIC_DISABLE,
        .configCrcEnable = PMIC_DISABLE,
        .commHandle0 = platform_getCommHandle(),
        .ioRead = &platform_rxByte,
        .ioWrite = &platform_txByte,
        .criticalSectionStart = &platform_critSecStart,
        .criticalSectionStop = &platform_critSecStop,
        .irqResponseCallback = &platform_irqResponse
    };

    status = Pmic_init(&pmicHandle, &handleCfg);
    if (status != PMIC_ST_SUCCESS)
    {
        platform_printString("\r\nERROR: Failed to initialize PMIC handle\r\n");
        platform_tearDownTests();
        platform_deinit();
        return;
    }

    platform_printString("\r\n=== ESM Module Tests ===\r\n");
    ESM_TEST_RUN_ALL();

    Pmic_deinit(&pmicHandle);
    platform_tearDownTests();
    platform_deinit();
}

/* ========================================================================== */
/*      esmGetCfg Test Aliases (Get operations combined with Set tests)      */
/* ========================================================================== */

void test_pos_esm_esmGetCfg_delay1(void) { test_pos_esm_esmSetCfg_delay1(); }
void test_pos_esm_esmGetCfg_delay2(void) { test_pos_esm_esmSetCfg_delay2(); }
void test_pos_esm_esmGetCfg_errCntThr(void) { test_pos_esm_esmSetCfg_errCntThr(); }
void test_pos_esm_esmGetCfg_hmax(void) { test_pos_esm_esmSetCfg_hmax(); }
void test_pos_esm_esmGetCfg_hmin(void) { test_pos_esm_esmSetCfg_hmin(); }
void test_pos_esm_esmGetCfg_lmax(void) { test_pos_esm_esmSetCfg_lmax(); }
void test_pos_esm_esmGetCfg_lmin(void) { test_pos_esm_esmSetCfg_lmin(); }
void test_pos_esm_esmGetCfg_mode(void) { test_pos_esm_esmSetCfg_mode(); }
void test_pos_esm_esmGetCfg_hmaxOnly(void) { test_pos_esm_esmSetCfg_hmax(); }
