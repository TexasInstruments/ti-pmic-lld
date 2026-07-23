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
void test_neg_esm_setEnableState_nullHandle(void)
{
    int32_t status = Pmic_esmSetEnableState(NULL, true);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_esmGetEnableState with NULL handle
 */
void test_neg_esm_getEnableState_nullHandle(void)
{
    bool isEnabled = false;
    int32_t status = Pmic_esmGetEnableState(NULL, &isEnabled);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_esmGetEnableState with NULL isEnabled parameter
 */
void test_neg_esm_getEnableState_nullIsEnabled(void)
{
    int32_t status = Pmic_esmGetEnableState(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_esmSetStartState with NULL handle
 */
void test_neg_esm_setStartState_nullHandle(void)
{
    int32_t status = Pmic_esmSetStartState(NULL, true);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_esmGetStartState with NULL handle
 */
void test_neg_esm_getStartState_nullHandle(void)
{
    bool started = false;
    int32_t status = Pmic_esmGetStartState(NULL, &started);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_esmGetStartState with NULL started parameter
 */
void test_neg_esm_getStartState_nullStarted(void)
{
    int32_t status = Pmic_esmGetStartState(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_esmSetCfg with NULL handle
 */
void test_neg_esm_setCfg_nullHandle(void)
{
    Pmic_EsmCfg_t esmCfg = {0};
    int32_t status = Pmic_esmSetCfg(NULL, &esmCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_esmSetCfg with NULL esmCfg parameter
 */
void test_neg_esm_setCfg_nullEsmCfg(void)
{
    int32_t status = Pmic_esmSetCfg(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_esmSetCfg with invalid validParams (zero)
 */
void test_neg_esm_setCfg_invalidValidParams(void)
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
void test_neg_esm_setCfg_invalidMode(void)
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
void test_neg_esm_setCfg_invalidErrCntThr(void)
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
void test_neg_esm_getCfg_nullHandle(void)
{
    Pmic_EsmCfg_t esmCfg = {0};
    int32_t status = Pmic_esmGetCfg(NULL, &esmCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_esmGetCfg with NULL esmCfg parameter
 */
void test_neg_esm_getCfg_nullEsmCfg(void)
{
    int32_t status = Pmic_esmGetCfg(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_esmGetCfg with invalid validParams (zero)
 */
void test_neg_esm_getCfg_invalidValidParams(void)
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
void test_neg_esm_getErrCnt_nullHandle(void)
{
    uint8_t errCnt = 0U;
    int32_t status = Pmic_esmGetErrCnt(NULL, &errCnt);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_esmGetErrCnt with NULL esmErrCnt parameter
 */
void test_neg_esm_getErrCnt_nullEsmErrCnt(void)
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
void test_pos_esm_setGetEnableState(void)
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
void test_pos_esm_setGetStartState(void)
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
void test_pos_esm_setCfg_mode(void)
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
void test_pos_esm_setCfg_errCntThr(void)
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
void test_pos_esm_setCfg_delay1(void)
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
void test_pos_esm_setCfg_delay2(void)
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
void test_pos_esm_setCfg_hmax(void)
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
void test_pos_esm_setCfg_hmin(void)
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
void test_pos_esm_setCfg_lmax(void)
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
void test_pos_esm_setCfg_lmin(void)
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
 * @brief Test ESM configuration set and get for clrEnDrvOnFailInt
 */
void test_pos_esm_setCfg_clrEnDrvOnFailInt(void)
{
    Pmic_EsmCfg_t esmCfgSet = {
        .validParams = PMIC_ESM_CLR_EN_DRV_ON_FAIL_INT_VALID,
        .clrEnDrvOnFailInt = true
    };
    Pmic_EsmCfg_t esmCfgGet = {
        .validParams = PMIC_ESM_CLR_EN_DRV_ON_FAIL_INT_VALID
    };
    int32_t status;

    /* Set to true */
    status = Pmic_esmSetCfg(&pmicHandle, &esmCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Get and verify */
    status = Pmic_esmGetCfg(&pmicHandle, &esmCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(esmCfgGet.clrEnDrvOnFailInt == true);

    /* Set to false */
    esmCfgSet.clrEnDrvOnFailInt = false;
    status = Pmic_esmSetCfg(&pmicHandle, &esmCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Get and verify */
    status = Pmic_esmGetCfg(&pmicHandle, &esmCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(esmCfgGet.clrEnDrvOnFailInt == false);
}

/**
 * @brief Test ESM error count read functionality
 */
void test_pos_esm_getErrCnt(void)
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
void test_pos_esm_setCfg_combined(void)
{
    int32_t status;

    /* Set multiple configuration parameters at once */
    Pmic_EsmCfg_t esmCfgSet = {
        .validParams = PMIC_ESM_MODE_VALID | PMIC_ESM_ERR_CNT_THR_VALID |
                       PMIC_ESM_DELAY1_VALID | PMIC_ESM_DELAY2_VALID |
                       PMIC_ESM_CLR_EN_DRV_ON_FAIL_INT_VALID,
        .mode = PMIC_ESM_MODE_LEVEL,
        .errCntThr = 0x5,
        .delay1 = 0x80,
        .delay2 = 0x40,
        .clrEnDrvOnFailInt = true
    };

    status = Pmic_esmSetCfg(&pmicHandle, &esmCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Read back and verify */
    Pmic_EsmCfg_t esmCfgGet = {
        .validParams = PMIC_ESM_MODE_VALID | PMIC_ESM_ERR_CNT_THR_VALID |
                       PMIC_ESM_DELAY1_VALID | PMIC_ESM_DELAY2_VALID |
                       PMIC_ESM_CLR_EN_DRV_ON_FAIL_INT_VALID
    };

    status = Pmic_esmGetCfg(&pmicHandle, &esmCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(esmCfgGet.mode == esmCfgSet.mode);
    PLATFORM_ASSERT(esmCfgGet.errCntThr == esmCfgSet.errCntThr);
    PLATFORM_ASSERT(esmCfgGet.delay1 == esmCfgSet.delay1);
    PLATFORM_ASSERT(esmCfgGet.delay2 == esmCfgSet.delay2);
    PLATFORM_ASSERT(esmCfgGet.clrEnDrvOnFailInt == esmCfgSet.clrEnDrvOnFailInt);
}

/**
 * @brief Test PWM mode configuration with timing parameters
 */
void test_pos_esm_setCfg_pwmMode(void)
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
void test_pos_esm_completeSequence(void)
{
    int32_t status;
    bool isEnabled = false;
    bool started = false;

    /* Configure ESM with all parameters */
    Pmic_EsmCfg_t esmCfg = {
        .validParams = PMIC_ESM_MODE_VALID | PMIC_ESM_ERR_CNT_THR_VALID |
                       PMIC_ESM_DELAY1_VALID | PMIC_ESM_DELAY2_VALID |
                       PMIC_ESM_HMIN_VALID | PMIC_ESM_HMAX_VALID |
                       PMIC_ESM_LMIN_VALID | PMIC_ESM_LMAX_VALID |
                       PMIC_ESM_CLR_EN_DRV_ON_FAIL_INT_VALID,
        .mode = PMIC_ESM_MODE_PWM,
        .errCntThr = 0x3,
        .delay1 = 0x50,
        .delay2 = 0x30,
        .hmin = 0x20,
        .hmax = 0x80,
        .lmin = 0x10,
        .lmax = 0x60,
        .clrEnDrvOnFailInt = false
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
void test_pos_esm_getCfg_readback(void)
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
void test_pos_esm_enableCfgStartSequence(void)
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

/* ========================================================================== */
/*                         Entry Point Function                               */
/* ========================================================================== */

void esm_test(void *args)
{
    (void)args;
    int32_t status;

    platform_init();
    testTimer_startModule("ESM");
    platform_setupTests();

    /* Initialize PMIC handle */
    Pmic_HandleCfg_t handleCfg = {
        .validParams = PMIC_COMM_MODE_VALID |
                       PMIC_COMM_HANDLE_0_VALID |
                       PMIC_IO_READ_VALID |
                       PMIC_IO_WRITE_VALID |
                       PMIC_CRITICAL_SECTION_START_VALID |
                       PMIC_CRITICAL_SECTION_STOP_VALID,
        .commMode = PMIC_INTF_SPI,
        .commHandle0 = platform_getCommHandle(),
        .ioRead = &platform_rxByte,
        .ioWrite = &platform_txByte,
        .criticalSectionStart = &platform_critSecStart,
        .criticalSectionStop = &platform_critSecStop
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

    testTimer_endModule();
    Pmic_deinit(&pmicHandle);
    platform_tearDownTests();
    platform_deinit();
}
