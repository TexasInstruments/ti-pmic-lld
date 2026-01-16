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


#include "../platform.h"
#include "esm_test.h"

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
static void test_esm_setEnableState_nullHandle(void)
{
    int32_t status = Pmic_esmSetEnableState(NULL, true);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_esmGetEnableState with NULL handle
 */
static void test_esm_getEnableState_nullHandle(void)
{
    bool isEnabled = false;
    int32_t status = Pmic_esmGetEnableState(NULL, &isEnabled);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_esmGetEnableState with NULL isEnabled parameter
 */
static void test_esm_getEnableState_nullIsEnabled(void)
{
    int32_t status = Pmic_esmGetEnableState(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_esmSetStartState with NULL handle
 */
static void test_esm_setStartState_nullHandle(void)
{
    int32_t status = Pmic_esmSetStartState(NULL, true);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_esmGetStartState with NULL handle
 */
static void test_esm_getStartState_nullHandle(void)
{
    bool started = false;
    int32_t status = Pmic_esmGetStartState(NULL, &started);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_esmGetStartState with NULL started parameter
 */
static void test_esm_getStartState_nullStarted(void)
{
    int32_t status = Pmic_esmGetStartState(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_esmSetCfg with NULL handle
 */
static void test_esm_setCfg_nullHandle(void)
{
    Pmic_EsmCfg_t esmCfg = {0};
    int32_t status = Pmic_esmSetCfg(NULL, &esmCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_esmSetCfg with NULL esmCfg parameter
 */
static void test_esm_setCfg_nullEsmCfg(void)
{
    int32_t status = Pmic_esmSetCfg(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_esmSetCfg with invalid validParams (zero)
 */
static void test_esm_setCfg_invalidValidParams(void)
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
static void test_esm_setCfg_invalidMode(void)
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
static void test_esm_setCfg_invalidErrCntThr(void)
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
static void test_esm_getCfg_nullHandle(void)
{
    Pmic_EsmCfg_t esmCfg = {0};
    int32_t status = Pmic_esmGetCfg(NULL, &esmCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_esmGetCfg with NULL esmCfg parameter
 */
static void test_esm_getCfg_nullEsmCfg(void)
{
    int32_t status = Pmic_esmGetCfg(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_esmGetCfg with invalid validParams (zero)
 */
static void test_esm_getCfg_invalidValidParams(void)
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
static void test_esm_getErrCnt_nullHandle(void)
{
    uint8_t errCnt = 0U;
    int32_t status = Pmic_esmGetErrCnt(NULL, &errCnt);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_esmGetErrCnt with NULL esmErrCnt parameter
 */
static void test_esm_getErrCnt_nullEsmErrCnt(void)
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
static void test_esm_enableDisable(void)
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
static void test_esm_startStop(void)
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
static void test_esm_cfg_mode(void)
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
static void test_esm_cfg_errCntThr(void)
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
static void test_esm_cfg_delay1(void)
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
static void test_esm_cfg_delay2(void)
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
static void test_esm_cfg_hmax(void)
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
static void test_esm_cfg_hmin(void)
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
static void test_esm_cfg_lmax(void)
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
static void test_esm_cfg_lmin(void)
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
static void test_esm_getErrCnt(void)
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
static void test_esm_combinedConfiguration(void)
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
static void test_esm_pwmModeConfiguration(void)
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
static void test_esm_completeConfigurationSequence(void)
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
static void test_esm_configurationReadbackVerification(void)
{
    int32_t status;

    /* Set specific configuration */
    Pmic_EsmCfg_t esmCfgSet = {
        .validParams = PMIC_ESM_MODE_VALID | PMIC_ESM_ERR_CNT_THR_VALID |
                       PMIC_ESM_DELAY1_VALID | PMIC_ESM_DELAY2_VALID,
        .mode = PMIC_ESM_MODE_LEVEL,
        .errCntThr = 0xA,
        .delay1 = 0xAA,
        .delay2 = 0x55
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
    PLATFORM_ASSERT(esmCfgGet.delay1 == 0xAA);

    /* Verify delay2 */
    esmCfgGet.validParams = PMIC_ESM_DELAY2_VALID;
    status = Pmic_esmGetCfg(&pmicHandle, &esmCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(esmCfgGet.delay2 == 0x55);
}

/**
 * @brief Test ESM enable, configure, and start combined sequence
 */
static void test_esm_enableConfigureStartSequence(void)
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
static void test_esmStart(void)
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
static void test_esmStop(void)
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
static void test_esmStart_nullHandle(void)
{
    int32_t status = Pmic_esmStart(NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_esmStop with NULL handle
 */
static void test_esmStop_nullHandle(void)
{
    int32_t status = Pmic_esmStop(NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_esmGetStatus with NULL handle
 */
static void test_esmGetStatus_nullHandle(void)
{
    Pmic_EsmStat_t esmStat = {0};
    int32_t status = Pmic_esmGetStatus(NULL, &esmStat);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_esmGetStatus with NULL esmStat parameter
 */
static void test_esmGetStatus_nullEsmStat(void)
{
    int32_t status = Pmic_esmGetStatus(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_esmGetStatus with zero validParams
 */
static void test_esmGetStatus_zeroValidParams(void)
{
    Pmic_EsmStat_t esmStat = {
        .validParams = 0U
    };
    int32_t status = Pmic_esmGetStatus(&pmicHandle, &esmStat);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_esmGetStatus with invalid validParams
 */
static void test_esmGetStatus_invalidValidParams(void)
{
    Pmic_EsmStat_t esmStat = {
        .validParams = PMIC_ESM_STATUS_ALL_VALID + 1U
    };
    int32_t status = Pmic_esmGetStatus(&pmicHandle, &esmStat);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_esmGetStatus retrieval of all status fields
 */
static void test_esmGetStatus_allFields(void)
{
    Pmic_EsmStat_t esmStat = {
        .validParams = PMIC_ESM_STATUS_ALL_VALID
    };
    int32_t status = Pmic_esmGetStatus(&pmicHandle, &esmStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

/**
 * @brief Test Pmic_esmGetStatus retrieval of rstInt field
 */
static void test_esmGetStatus_rstInt(void)
{
    Pmic_EsmStat_t esmStat = {
        .validParams = PMIC_ESM_RST_INT_VALID
    };
    int32_t status = Pmic_esmGetStatus(&pmicHandle, &esmStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

/**
 * @brief Test Pmic_esmGetStatus retrieval of failInt field
 */
static void test_esmGetStatus_failInt(void)
{
    Pmic_EsmStat_t esmStat = {
        .validParams = PMIC_ESM_FAIL_INT_VALID
    };
    int32_t status = Pmic_esmGetStatus(&pmicHandle, &esmStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

/**
 * @brief Test Pmic_esmGetStatus retrieval of pinInt field
 */
static void test_esmGetStatus_pinInt(void)
{
    Pmic_EsmStat_t esmStat = {
        .validParams = PMIC_ESM_PIN_INT_VALID
    };
    int32_t status = Pmic_esmGetStatus(&pmicHandle, &esmStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

/**
 * @brief Test Pmic_esmClrStatus with NULL handle
 */
static void test_esmClrStatus_nullHandle(void)
{
    Pmic_EsmStat_t esmStat = {0};
    int32_t status = Pmic_esmClrStatus(NULL, &esmStat);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_esmClrStatus with NULL esmStat parameter
 */
static void test_esmClrStatus_nullEsmStat(void)
{
    int32_t status = Pmic_esmClrStatus(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_esmClrStatus with zero validParams
 */
static void test_esmClrStatus_zeroValidParams(void)
{
    Pmic_EsmStat_t esmStat = {
        .validParams = 0U
    };
    int32_t status = Pmic_esmClrStatus(&pmicHandle, &esmStat);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_esmClrStatus with invalid validParams
 */
static void test_esmClrStatus_invalidValidParams(void)
{
    Pmic_EsmStat_t esmStat = {
        .validParams = PMIC_ESM_STATUS_ALL_VALID + 1U
    };
    int32_t status = Pmic_esmClrStatus(&pmicHandle, &esmStat);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_esmClrStatus clearing all status fields
 */
static void test_esmClrStatus_allFields(void)
{
    Pmic_EsmStat_t esmStat = {
        .validParams = PMIC_ESM_STATUS_ALL_VALID
    };
    int32_t status = Pmic_esmClrStatus(&pmicHandle, &esmStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

/**
 * @brief Test Pmic_esmClrStatus clearing rstInt field
 */
static void test_esmClrStatus_rstInt(void)
{
    Pmic_EsmStat_t esmStat = {
        .validParams = PMIC_ESM_RST_INT_VALID
    };
    int32_t status = Pmic_esmClrStatus(&pmicHandle, &esmStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

/**
 * @brief Test Pmic_esmClrStatus clearing failInt field
 */
static void test_esmClrStatus_failInt(void)
{
    Pmic_EsmStat_t esmStat = {
        .validParams = PMIC_ESM_FAIL_INT_VALID
    };
    int32_t status = Pmic_esmClrStatus(&pmicHandle, &esmStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

/**
 * @brief Test Pmic_esmClrStatus clearing pinInt field
 */
static void test_esmClrStatus_pinInt(void)
{
    Pmic_EsmStat_t esmStat = {
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
static void test_esmSetEnableState_ioReadFailure(void)
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
static void test_esmGetEnableState_ioReadFailure(void)
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
static void test_esmSetStartState_ioReadFailure(void)
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
static void test_esmGetStartState_ioReadFailure(void)
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
static void test_esmSetCfg_delay1ReadFailure(void)
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
static void test_esmSetCfg_hmaxReadFailure(void)
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
static void test_esmSetCfg_lmaxReadFailure(void)
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
static void test_esmSetCfg_modeCfgReadFailure(void)
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
static void test_esmGetCfg_modeCfgReadFailure(void)
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
static void test_esmGetCfg_delay1ReadFailure(void)
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
static void test_esmGetCfg_hmaxReadFailure(void)
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
static void test_esmGetCfg_lmaxReadFailure(void)
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
static void test_esmGetErrCnt_ioReadFailure(void)
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
static void test_esmGetStatus_ioReadFailure(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t* mockDevice = platform_getMockDevice();
    int32_t status;
    Pmic_EsmStat_t esmStat = {
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
static void test_esmClrStatus_ioWriteFailure(void)
{
#ifdef BUILD_MOCK
    PmicMockDevice_t* mockDevice = platform_getMockDevice();
    int32_t status;
    Pmic_EsmStat_t esmStat = {
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
static void test_esmSetCfg_cascadeFailure_delayToHmax(void)
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
static void test_esmSetCfg_cascadeFailure_hmaxToLmax(void)
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
static void test_esmGetCfg_cascadeFailure_modeToDelay(void)
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
static void test_esmGetCfg_cascadeFailure_delayToHmax(void)
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
static void test_esmSetCfg_delay2Only(void)
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
static void test_esmSetCfg_hminOnly(void)
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
static void test_esmSetCfg_lminOnly(void)
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
static void test_esmGetCfg_delay2Only(void)
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
static void test_esmGetCfg_hminOnly(void)
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
static void test_esmGetCfg_lminOnly(void)
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
/*                         Test Execution Macros                              */
/* ========================================================================== */

#define ESM_TEST_RUN_NEGATIVE() \
    do { \
        RUN_TEST(test_esm_setEnableState_nullHandle); \
        RUN_TEST(test_esm_getEnableState_nullHandle); \
        RUN_TEST(test_esm_getEnableState_nullIsEnabled); \
        RUN_TEST(test_esm_setStartState_nullHandle); \
        RUN_TEST(test_esm_getStartState_nullHandle); \
        RUN_TEST(test_esm_getStartState_nullStarted); \
        RUN_TEST(test_esm_setCfg_nullHandle); \
        RUN_TEST(test_esm_setCfg_nullEsmCfg); \
        RUN_TEST(test_esm_setCfg_invalidValidParams); \
        RUN_TEST(test_esm_setCfg_invalidMode); \
        RUN_TEST(test_esm_setCfg_invalidErrCntThr); \
        RUN_TEST(test_esm_getCfg_nullHandle); \
        RUN_TEST(test_esm_getCfg_nullEsmCfg); \
        RUN_TEST(test_esm_getCfg_invalidValidParams); \
        RUN_TEST(test_esm_getErrCnt_nullHandle); \
        RUN_TEST(test_esm_getErrCnt_nullEsmErrCnt); \
        RUN_TEST(test_esmStart_nullHandle); \
        RUN_TEST(test_esmStop_nullHandle); \
        RUN_TEST(test_esmGetStatus_nullHandle); \
        RUN_TEST(test_esmGetStatus_nullEsmStat); \
        RUN_TEST(test_esmGetStatus_zeroValidParams); \
        RUN_TEST(test_esmGetStatus_invalidValidParams); \
        RUN_TEST(test_esmClrStatus_nullHandle); \
        RUN_TEST(test_esmClrStatus_nullEsmStat); \
        RUN_TEST(test_esmClrStatus_zeroValidParams); \
        RUN_TEST(test_esmClrStatus_invalidValidParams); \
    } while(0)

#define ESM_TEST_RUN_POSITIVE() \
    do { \
        RUN_TEST(test_esm_enableDisable); \
        RUN_TEST(test_esm_startStop); \
        RUN_TEST(test_esm_cfg_mode); \
        RUN_TEST(test_esm_cfg_errCntThr); \
        RUN_TEST(test_esm_cfg_delay1); \
        RUN_TEST(test_esm_cfg_delay2); \
        RUN_TEST(test_esm_cfg_hmax); \
        RUN_TEST(test_esm_cfg_hmin); \
        RUN_TEST(test_esm_cfg_lmax); \
        RUN_TEST(test_esm_cfg_lmin); \
        RUN_TEST(test_esm_getErrCnt); \
        RUN_TEST(test_esm_combinedConfiguration); \
        RUN_TEST(test_esm_pwmModeConfiguration); \
        RUN_TEST(test_esm_completeConfigurationSequence); \
        RUN_TEST(test_esm_configurationReadbackVerification); \
        RUN_TEST(test_esm_enableConfigureStartSequence); \
        RUN_TEST(test_esmStart); \
        RUN_TEST(test_esmStop); \
        RUN_TEST(test_esmGetStatus_allFields); \
        RUN_TEST(test_esmGetStatus_rstInt); \
        RUN_TEST(test_esmGetStatus_failInt); \
        RUN_TEST(test_esmGetStatus_pinInt); \
        RUN_TEST(test_esmClrStatus_allFields); \
        RUN_TEST(test_esmClrStatus_rstInt); \
        RUN_TEST(test_esmClrStatus_failInt); \
        RUN_TEST(test_esmClrStatus_pinInt); \
    } while(0)

#define ESM_TEST_RUN_IO_ERROR() \
    do { \
        RUN_TEST(test_esmSetEnableState_ioReadFailure); \
        RUN_TEST(test_esmGetEnableState_ioReadFailure); \
        RUN_TEST(test_esmSetStartState_ioReadFailure); \
        RUN_TEST(test_esmGetStartState_ioReadFailure); \
        RUN_TEST(test_esmSetCfg_delay1ReadFailure); \
        RUN_TEST(test_esmSetCfg_hmaxReadFailure); \
        RUN_TEST(test_esmSetCfg_lmaxReadFailure); \
        RUN_TEST(test_esmSetCfg_modeCfgReadFailure); \
        RUN_TEST(test_esmGetCfg_modeCfgReadFailure); \
        RUN_TEST(test_esmGetCfg_delay1ReadFailure); \
        RUN_TEST(test_esmGetCfg_hmaxReadFailure); \
        RUN_TEST(test_esmGetCfg_lmaxReadFailure); \
        RUN_TEST(test_esmGetErrCnt_ioReadFailure); \
        RUN_TEST(test_esmGetStatus_ioReadFailure); \
        RUN_TEST(test_esmClrStatus_ioWriteFailure); \
    } while(0)

#define ESM_TEST_RUN_CASCADE() \
    do { \
        RUN_TEST(test_esmSetCfg_cascadeFailure_delayToHmax); \
        RUN_TEST(test_esmSetCfg_cascadeFailure_hmaxToLmax); \
        RUN_TEST(test_esmGetCfg_cascadeFailure_modeToDelay); \
        RUN_TEST(test_esmGetCfg_cascadeFailure_delayToHmax); \
    } while(0)

#define ESM_TEST_RUN_PARTIAL_PARAMS() \
    do { \
        RUN_TEST(test_esmSetCfg_delay2Only); \
        RUN_TEST(test_esmSetCfg_hminOnly); \
        RUN_TEST(test_esmSetCfg_lminOnly); \
        RUN_TEST(test_esmGetCfg_delay2Only); \
        RUN_TEST(test_esmGetCfg_hminOnly); \
        RUN_TEST(test_esmGetCfg_lminOnly); \
    } while(0)

#define ESM_TEST_RUN_ALL() \
    do { \
        ESM_TEST_RUN_NEGATIVE(); \
        ESM_TEST_RUN_POSITIVE(); \
        ESM_TEST_RUN_IO_ERROR(); \
        ESM_TEST_RUN_CASCADE(); \
        ESM_TEST_RUN_PARTIAL_PARAMS(); \
    } while(0)

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
