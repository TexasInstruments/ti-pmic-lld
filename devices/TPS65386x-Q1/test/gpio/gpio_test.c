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

#include "gpio_test.h"
#include "pmic_mock_core.h"

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

static Pmic_Handle_t g_pmicHandle;

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

/* ========================================================================== */
/*                            Helper Functions                                */
/* ========================================================================== */

/**
 * @brief Initialize PMIC handle for GPIO tests
 */
static int32_t gpioTest_initHandle(void)
{
    /* Dummy handle for mock - driver validates non-NULL but doesn't dereference */
    static uint32_t dummyCommHandle = 0x12345678U;

    Pmic_HandleCfg_t pmicCfg = {
        .validParams = PMIC_COMM_MODE_VALID |
                       PMIC_COMM_HANDLE_0_VALID |
                       PMIC_IO_READ_VALID |
                       PMIC_IO_WRITE_VALID |
                       PMIC_CRITICAL_SECTION_START_VALID |
                       PMIC_CRITICAL_SECTION_STOP_VALID,
        .commMode = PMIC_INTF_SPI,
        .commHandle0 = (void*)&dummyCommHandle,  /* Driver requires non-NULL, even for mock */
        .ioRead = &test_pmic_regRead,
        .ioWrite = &test_pmic_regWrite,
        .criticalSectionStart = &test_pmic_criticalSectionStartFn,
        .criticalSectionStop = &test_pmic_criticalSectionStopFn
    };
    int32_t status;

    status = Pmic_init(&g_pmicHandle, &pmicCfg);

    return status;
}

/**
 * @brief Helper function to test GPIO set/get configuration
 *
 * @param validParam  Valid param bit for the GPIO to test
 * @param setValue    Value to set
 * @param verifyFunc  Function pointer to verify the value
 */
static void gpioTest_setGetCfg(uint32_t validParam, uint8_t setValue,
                                void (*verifyFunc)(const Pmic_GpioCfg_t *cfg, uint8_t expected))
{
    Pmic_GpioCfg_t setCfg, getCfg;
    int32_t status;

    memset(&setCfg, 0, sizeof(setCfg));
    memset(&getCfg, 0, sizeof(getCfg));

    setCfg.validParams = validParam;
    getCfg.validParams = validParam;

    /* Set the appropriate field based on validParam */
    if (validParam == PMIC_CFG_GPI1_VALID)
    {
        setCfg.gpi1 = setValue;
    }
    else if (validParam == PMIC_CFG_GPI4_VALID)
    {
        setCfg.gpi4 = setValue;
    }
    else if (validParam == PMIC_CFG_GPO1_VALID)
    {
        setCfg.gpo1 = setValue;
    }
    else if (validParam == PMIC_CFG_GPO2_VALID)
    {
        setCfg.gpo2 = setValue;
    }
    else if (validParam == PMIC_CFG_GPO3_VALID)
    {
        setCfg.gpo3 = setValue;
    }
    else if (validParam == PMIC_CFG_GPO4_VALID)
    {
        setCfg.gpo4 = setValue;
    }

    /* Set configuration */
    status = Pmic_gpioSetCfg(&g_pmicHandle, &setCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Get configuration */
    status = Pmic_gpioGetCfg(&g_pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Verify configuration */
    if (verifyFunc != NULL)
    {
        verifyFunc(&getCfg, setValue);
    }
}

/**
 * @brief Verify GPI1 configuration
 */
static void gpioTest_verifyGpi1(const Pmic_GpioCfg_t *cfg, uint8_t expected)
{
    PLATFORM_ASSERT(cfg->gpi1 == expected);
}

/**
 * @brief Verify GPI4 configuration
 */
static void gpioTest_verifyGpi4(const Pmic_GpioCfg_t *cfg, uint8_t expected)
{
    PLATFORM_ASSERT(cfg->gpi4 == expected);
}

/**
 * @brief Verify GPO1 configuration
 */
static void gpioTest_verifyGpo1(const Pmic_GpioCfg_t *cfg, uint8_t expected)
{
    PLATFORM_ASSERT(cfg->gpo1 == expected);
}

/**
 * @brief Verify GPO2 configuration
 */
static void gpioTest_verifyGpo2(const Pmic_GpioCfg_t *cfg, uint8_t expected)
{
    PLATFORM_ASSERT(cfg->gpo2 == expected);
}

/**
 * @brief Verify GPO3 configuration
 */
static void gpioTest_verifyGpo3(const Pmic_GpioCfg_t *cfg, uint8_t expected)
{
    PLATFORM_ASSERT(cfg->gpo3 == expected);
}

/**
 * @brief Verify GPO4 configuration
 */
static void gpioTest_verifyGpo4(const Pmic_GpioCfg_t *cfg, uint8_t expected)
{
    PLATFORM_ASSERT(cfg->gpo4 == expected);
}

/* ========================================================================== */
/*                         POSITIVE TEST CASES                                */
/* ========================================================================== */

/* GPI1 Tests */
void test_positive_gpio_setGetCfg_gpi1_esmIn(void)
{
    gpioTest_setGetCfg(PMIC_CFG_GPI1_VALID, PMIC_GPI1_ESM_IN, gpioTest_verifyGpi1);
}

void test_positive_gpio_setGetCfg_gpi1_wdIn(void)
{
    gpioTest_setGetCfg(PMIC_CFG_GPI1_VALID, PMIC_GPI1_WD_IN, gpioTest_verifyGpi1);
}

/* GPI4 Tests */
void test_positive_gpio_setGetCfg_gpi4_comparator(void)
{
    gpioTest_setGetCfg(PMIC_CFG_GPI4_VALID, PMIC_GPI4_COMPARATOR, gpioTest_verifyGpi4);
}

void test_positive_gpio_setGetCfg_gpi4_wdIn(void)
{
    gpioTest_setGetCfg(PMIC_CFG_GPI4_VALID, PMIC_GPI4_WD_IN, gpioTest_verifyGpi4);
}

void test_positive_gpio_setGetCfg_gpi4_cosN(void)
{
    gpioTest_setGetCfg(PMIC_CFG_GPI4_VALID, PMIC_GPI4_COS_N, gpioTest_verifyGpi4);
}

/* GPO1 Tests */
void test_positive_gpio_setGetCfg_gpo1_lowLvl(void)
{
    gpioTest_setGetCfg(PMIC_CFG_GPO1_VALID, PMIC_GPO1_LOW_LVL, gpioTest_verifyGpo1);
}

void test_positive_gpio_setGetCfg_gpo1_highLvl(void)
{
    gpioTest_setGetCfg(PMIC_CFG_GPO1_VALID, PMIC_GPO1_HIGH_LVL, gpioTest_verifyGpo1);
}

void test_positive_gpio_setGetCfg_gpo1_hiz(void)
{
    gpioTest_setGetCfg(PMIC_CFG_GPO1_VALID, PMIC_GPO1_HIZ, gpioTest_verifyGpo1);
}

void test_positive_gpio_setGetCfg_gpo1_nint(void)
{
    gpioTest_setGetCfg(PMIC_CFG_GPO1_VALID, PMIC_GPO1_NINT, gpioTest_verifyGpo1);
}

void test_positive_gpio_setGetCfg_gpo1_enOut(void)
{
    gpioTest_setGetCfg(PMIC_CFG_GPO1_VALID, PMIC_GPO1_EN_OUT, gpioTest_verifyGpo1);
}

void test_positive_gpio_setGetCfg_gpo1_enOut2(void)
{
    gpioTest_setGetCfg(PMIC_CFG_GPO1_VALID, PMIC_GPO1_EN_OUT2, gpioTest_verifyGpo1);
}

void test_positive_gpio_setGetCfg_gpo1_sinNO(void)
{
    gpioTest_setGetCfg(PMIC_CFG_GPO1_VALID, PMIC_GPO1_SIN_N_O, gpioTest_verifyGpo1);
}

/* GPO2 Tests */
void test_positive_gpio_setGetCfg_gpo2_lowLvl(void)
{
    gpioTest_setGetCfg(PMIC_CFG_GPO2_VALID, PMIC_GPO2_LOW_LVL, gpioTest_verifyGpo2);
}

void test_positive_gpio_setGetCfg_gpo2_highLvl(void)
{
    gpioTest_setGetCfg(PMIC_CFG_GPO2_VALID, PMIC_GPO2_HIGH_LVL, gpioTest_verifyGpo2);
}

void test_positive_gpio_setGetCfg_gpo2_hiz(void)
{
    gpioTest_setGetCfg(PMIC_CFG_GPO2_VALID, PMIC_GPO2_HIZ, gpioTest_verifyGpo2);
}

void test_positive_gpio_setGetCfg_gpo2_comp1Out(void)
{
    gpioTest_setGetCfg(PMIC_CFG_GPO2_VALID, PMIC_GPO2_COMP1_OUT, gpioTest_verifyGpo2);
}

void test_positive_gpio_setGetCfg_gpo2_enOut2(void)
{
    gpioTest_setGetCfg(PMIC_CFG_GPO2_VALID, PMIC_GPO2_EN_OUT2, gpioTest_verifyGpo2);
}

void test_positive_gpio_setGetCfg_gpo2_syncClkOut(void)
{
    gpioTest_setGetCfg(PMIC_CFG_GPO2_VALID, PMIC_GPO2_SYNCCLKOUT, gpioTest_verifyGpo2);
}

void test_positive_gpio_setGetCfg_gpo2_pgood(void)
{
    gpioTest_setGetCfg(PMIC_CFG_GPO2_VALID, PMIC_GPO2_PGOOD, gpioTest_verifyGpo2);
}

void test_positive_gpio_setGetCfg_gpo2_sinPO(void)
{
    gpioTest_setGetCfg(PMIC_CFG_GPO2_VALID, PMIC_GPO2_SIN_P_O, gpioTest_verifyGpo2);
}

/* GPO3 Tests */
void test_positive_gpio_setGetCfg_gpo3_lowLvl(void)
{
    gpioTest_setGetCfg(PMIC_CFG_GPO3_VALID, PMIC_GPO3_LOW_LVL, gpioTest_verifyGpo3);
}

void test_positive_gpio_setGetCfg_gpo3_highLvl(void)
{
    gpioTest_setGetCfg(PMIC_CFG_GPO3_VALID, PMIC_GPO3_HIGH_LVL, gpioTest_verifyGpo3);
}

void test_positive_gpio_setGetCfg_gpo3_hiz(void)
{
    gpioTest_setGetCfg(PMIC_CFG_GPO3_VALID, PMIC_GPO3_HIZ, gpioTest_verifyGpo3);
}

void test_positive_gpio_setGetCfg_gpo3_pgood(void)
{
    gpioTest_setGetCfg(PMIC_CFG_GPO3_VALID, PMIC_GPO3_PGOOD, gpioTest_verifyGpo3);
}

void test_positive_gpio_setGetCfg_gpo3_comp2Out(void)
{
    gpioTest_setGetCfg(PMIC_CFG_GPO3_VALID, PMIC_GPO3_COMP2_OUT, gpioTest_verifyGpo3);
}

void test_positive_gpio_setGetCfg_gpo3_enOut2(void)
{
    gpioTest_setGetCfg(PMIC_CFG_GPO3_VALID, PMIC_GPO3_EN_OUT2, gpioTest_verifyGpo3);
}

void test_positive_gpio_setGetCfg_gpo3_safeOut2(void)
{
    gpioTest_setGetCfg(PMIC_CFG_GPO3_VALID, PMIC_GPO3_SAFE_OUT2, gpioTest_verifyGpo3);
}

void test_positive_gpio_setGetCfg_gpo3_cosPO(void)
{
    gpioTest_setGetCfg(PMIC_CFG_GPO3_VALID, PMIC_GPO3_COS_P_O, gpioTest_verifyGpo3);
}

/* GPO4 Tests */
void test_positive_gpio_setGetCfg_gpo4_lowLvl(void)
{
    gpioTest_setGetCfg(PMIC_CFG_GPO4_VALID, PMIC_GPO4_LOW_LVL, gpioTest_verifyGpo4);
}

void test_positive_gpio_setGetCfg_gpo4_highLvl(void)
{
    gpioTest_setGetCfg(PMIC_CFG_GPO4_VALID, PMIC_GPO4_HIGH_LVL, gpioTest_verifyGpo4);
}

void test_positive_gpio_setGetCfg_gpo4_hiz(void)
{
    gpioTest_setGetCfg(PMIC_CFG_GPO4_VALID, PMIC_GPO4_HIZ, gpioTest_verifyGpo4);
}

void test_positive_gpio_setGetCfg_gpo4_safeOut2(void)
{
    gpioTest_setGetCfg(PMIC_CFG_GPO4_VALID, PMIC_GPO4_SAFE_OUT2, gpioTest_verifyGpo4);
}

void test_positive_gpio_setGetCfg_gpo4_enOut(void)
{
    gpioTest_setGetCfg(PMIC_CFG_GPO4_VALID, PMIC_GPO4_EN_OUT, gpioTest_verifyGpo4);
}

void test_positive_gpio_setGetCfg_gpo4_nint(void)
{
    gpioTest_setGetCfg(PMIC_CFG_GPO4_VALID, PMIC_GPO4_NINT, gpioTest_verifyGpo4);
}

void test_positive_gpio_setGetCfg_gpo4_pgood(void)
{
    gpioTest_setGetCfg(PMIC_CFG_GPO4_VALID, PMIC_GPO4_PGOOD, gpioTest_verifyGpo4);
}

void test_positive_gpio_setGetCfg_gpo4_cosNO(void)
{
    gpioTest_setGetCfg(PMIC_CFG_GPO4_VALID, PMIC_GPO4_COS_N_O, gpioTest_verifyGpo4);
}

/* GPO Output Value Tests */
void test_positive_gpio_getOutputValue_allGpos(void)
{
    int32_t status;
    bool high;
    uint8_t gpo;

    /* Test all 4 GPO pins */
    for (gpo = PMIC_GPO1; gpo <= PMIC_GPO4; gpo++)
    {
        status = Pmic_gpioGetOutputValue(&g_pmicHandle, gpo, &high);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    }
}

/**
 * @brief Test GPO1_HIZ_DUPLICATE handling
 *
 * This test verifies that when GPO1_CONF register returns value 6 (duplicate HIZ),
 * the driver correctly converts it to PMIC_GPO1_HIZ (value 2).
 *
 * Coverage target: pmic_gpio.c lines 281-284
 */
void test_positive_gpio_gpo1_hiz_duplicate(void)
{
    int32_t status;
    Pmic_GpioCfg_t getCfg;
    PmicMockDevice_t* mockDevice = platform_getMockDevice();

    PLATFORM_ASSERT(mockDevice != NULL);

    memset(&getCfg, 0, sizeof(getCfg));
    getCfg.validParams = PMIC_CFG_GPO1_VALID;

    /* Directly set GPO_CFG1 register to have GPO1_CONF = 6 (duplicate HIZ value)
     * Register layout: GPO_CFG1_REG (0x7C)
     *   Bits [2:0] = GPO1_CFG (we set to 6)
     *   Bits [5:3] = GPO2_CFG (we leave as 0)
     */
    status = PmicMock_WriteRegister(mockDevice, 0x7CU, 0x06U);
    PLATFORM_ASSERT(status == PMIC_MOCK_SUCCESS);

    /* Read configuration - driver should convert value 6 to PMIC_GPO1_HIZ (2) */
    status = Pmic_gpioGetCfg(&g_pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Verify that GPO1 was converted from 6 to 2 (PMIC_GPO1_HIZ) */
    PLATFORM_ASSERT(getCfg.gpo1 == PMIC_GPO1_HIZ);
}

/* ========================================================================== */
/*                    POSITIVE TESTS - SAFEOUT                                */
/* ========================================================================== */

void test_positive_gpio_safeOutSetGet(void)
{
    int32_t status;
    Pmic_GpioSafeOutCfg_t setCfg = {0U};
    Pmic_GpioSafeOutCfg_t getCfg = {0U};

    /* Enable both SAFEOUT1 and SAFEOUT2 */
    setCfg.validParams = PMIC_GPIO_SAFEOUT1_EN_VALID | PMIC_GPIO_SAFEOUT2_EN_VALID;
    setCfg.safeOut1En = true;
    setCfg.safeOut2En = true;
    status = Pmic_gpioSetSafeOutCfg(&g_pmicHandle, &setCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Verify both are enabled */
    status = Pmic_gpioGetSafeOutCfg(&g_pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(getCfg.safeOut1En == true);
    PLATFORM_ASSERT(getCfg.safeOut2En == true);

    /* Disable both SAFEOUT1 and SAFEOUT2 */
    setCfg.safeOut1En = false;
    setCfg.safeOut2En = false;
    status = Pmic_gpioSetSafeOutCfg(&g_pmicHandle, &setCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Verify both are disabled */
    status = Pmic_gpioGetSafeOutCfg(&g_pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(getCfg.safeOut1En == false);
    PLATFORM_ASSERT(getCfg.safeOut2En == false);
}

void test_positive_gpio_safeOut_individual(void)
{
    int32_t status;
    Pmic_GpioSafeOutCfg_t setCfg = {0U};
    Pmic_GpioSafeOutCfg_t getCfg = {0U};

    /* Enable only SAFEOUT1 */
    setCfg.validParams = PMIC_GPIO_SAFEOUT1_EN_VALID;
    setCfg.safeOut1En = true;
    status = Pmic_gpioSetSafeOutCfg(&g_pmicHandle, &setCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Verify SAFEOUT1 is set */
    status = Pmic_gpioGetSafeOutCfg(&g_pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(getCfg.safeOut1En == true);

    /* Enable only SAFEOUT2 */
    setCfg.validParams = PMIC_GPIO_SAFEOUT2_EN_VALID;
    setCfg.safeOut1En = false;
    setCfg.safeOut2En = true;
    status = Pmic_gpioSetSafeOutCfg(&g_pmicHandle, &setCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Verify SAFEOUT2 is set */
    status = Pmic_gpioGetSafeOutCfg(&g_pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(getCfg.safeOut2En == true);
}

/* ========================================================================== */
/*                         NEGATIVE TEST CASES                                */
/* ========================================================================== */

/* NULL Parameter Tests */
void test_negative_Pmic_gpioSetCfg_nullParam_handle(void)
{
    Pmic_GpioCfg_t gpioCfg;
    int32_t status;

    memset(&gpioCfg, 0, sizeof(gpioCfg));
    gpioCfg.validParams = PMIC_CFG_GPI1_VALID;

    status = Pmic_gpioSetCfg(NULL, &gpioCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_gpioSetCfg_nullParam_gpioCfg(void)
{
    int32_t status;

    status = Pmic_gpioSetCfg(&g_pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_gpioSetCfg_invalidParam_validParams(void)
{
    Pmic_GpioCfg_t gpioCfg;
    int32_t status;

    memset(&gpioCfg, 0, sizeof(gpioCfg));
    gpioCfg.validParams = 0U;  /* No valid params set */

    status = Pmic_gpioSetCfg(&g_pmicHandle, &gpioCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_negative_Pmic_gpioGetCfg_nullParam_handle(void)
{
    Pmic_GpioCfg_t gpioCfg;
    int32_t status;

    memset(&gpioCfg, 0, sizeof(gpioCfg));
    gpioCfg.validParams = PMIC_CFG_GPI1_VALID;

    status = Pmic_gpioGetCfg(NULL, &gpioCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_gpioGetCfg_nullParam_gpioCfg(void)
{
    int32_t status;

    status = Pmic_gpioGetCfg(&g_pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_gpioGetCfg_invalidParam_validParams(void)
{
    Pmic_GpioCfg_t gpioCfg;
    int32_t status;

    memset(&gpioCfg, 0, sizeof(gpioCfg));
    gpioCfg.validParams = 0U;  /* No valid params set */

    status = Pmic_gpioGetCfg(&g_pmicHandle, &gpioCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_negative_Pmic_gpioGetOutputValue_nullParam_handle(void)
{
    bool high;
    int32_t status;

    status = Pmic_gpioGetOutputValue(NULL, PMIC_GPO1, &high);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_gpioGetOutputValue_nullParam_high(void)
{
    int32_t status;

    status = Pmic_gpioGetOutputValue(&g_pmicHandle, PMIC_GPO1, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_gpioGetOutputValue_invalidParam_gpo(void)
{
    bool high;
    int32_t status;

    /* Test below minimum */
    status = Pmic_gpioGetOutputValue(&g_pmicHandle, PMIC_GPO_MIN - 1U, &high);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);

    /* Test above maximum */
    status = Pmic_gpioGetOutputValue(&g_pmicHandle, PMIC_GPO_MAX + 1U, &high);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/* Invalid Value Tests */
void test_negative_Pmic_gpioSetCfg_invalidValue_gpi1(void)
{
    Pmic_GpioCfg_t gpioCfg;
    int32_t status;

    memset(&gpioCfg, 0, sizeof(gpioCfg));
    gpioCfg.validParams = PMIC_CFG_GPI1_VALID;
    gpioCfg.gpi1 = PMIC_GPI1_CFG_MAX + 1U;  /* Invalid value */

    status = Pmic_gpioSetCfg(&g_pmicHandle, &gpioCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_negative_Pmic_gpioSetCfg_invalidValue_gpi4(void)
{
    Pmic_GpioCfg_t gpioCfg;
    int32_t status;

    memset(&gpioCfg, 0, sizeof(gpioCfg));
    gpioCfg.validParams = PMIC_CFG_GPI4_VALID;
    gpioCfg.gpi4 = PMIC_GPI4_CFG_MAX + 1U;  /* Invalid value */

    status = Pmic_gpioSetCfg(&g_pmicHandle, &gpioCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_negative_Pmic_gpioSetCfg_invalidValue_gpo1(void)
{
    Pmic_GpioCfg_t gpioCfg;
    int32_t status;

    memset(&gpioCfg, 0, sizeof(gpioCfg));
    gpioCfg.validParams = PMIC_CFG_GPO1_VALID;
    gpioCfg.gpo1 = PMIC_GPO1_CFG_MAX + 1U;  /* Invalid value */

    status = Pmic_gpioSetCfg(&g_pmicHandle, &gpioCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_negative_Pmic_gpioSetCfg_invalidValue_gpo2(void)
{
    Pmic_GpioCfg_t gpioCfg;
    int32_t status;

    memset(&gpioCfg, 0, sizeof(gpioCfg));
    gpioCfg.validParams = PMIC_CFG_GPO2_VALID;
    gpioCfg.gpo2 = PMIC_GPO2_CFG_MAX + 1U;  /* Invalid value */

    status = Pmic_gpioSetCfg(&g_pmicHandle, &gpioCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_negative_Pmic_gpioSetCfg_invalidValue_gpo3(void)
{
    Pmic_GpioCfg_t gpioCfg;
    int32_t status;

    memset(&gpioCfg, 0, sizeof(gpioCfg));
    gpioCfg.validParams = PMIC_CFG_GPO3_VALID;
    gpioCfg.gpo3 = PMIC_GPO3_CFG_MAX + 1U;  /* Invalid value */

    status = Pmic_gpioSetCfg(&g_pmicHandle, &gpioCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_negative_Pmic_gpioSetCfg_invalidValue_gpo4(void)
{
    Pmic_GpioCfg_t gpioCfg;
    int32_t status;

    memset(&gpioCfg, 0, sizeof(gpioCfg));
    gpioCfg.validParams = PMIC_CFG_GPO4_VALID;
    gpioCfg.gpo4 = PMIC_GPO4_CFG_MAX + 1U;  /* Invalid value */

    status = Pmic_gpioSetCfg(&g_pmicHandle, &gpioCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/* ========================================================================== */
/*                       NEGATIVE TESTS - SAFEOUT                             */
/* ========================================================================== */

void test_negative_Pmic_gpioSetSafeOutCfg_nullParam_handle(void)
{
    Pmic_GpioSafeOutCfg_t config = {
        .validParams = PMIC_GPIO_SAFEOUT1_EN_VALID,
        .safeOut1En = true
    };
    int32_t status = Pmic_gpioSetSafeOutCfg(NULL, &config);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_gpioGetSafeOutCfg_nullParam_handle(void)
{
    Pmic_GpioSafeOutCfg_t config = {0U};
    int32_t status = Pmic_gpioGetSafeOutCfg(NULL, &config);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_gpioGetSafeOutCfg_nullParam_config(void)
{
    int32_t status = Pmic_gpioGetSafeOutCfg(&g_pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_negative_Pmic_gpioSetSafeOutCfg_invalidParam_validParams(void)
{
    Pmic_GpioSafeOutCfg_t config = {
        .validParams = 0U,  /* No valid params set */
        .safeOut1En = true
    };
    int32_t status = Pmic_gpioSetSafeOutCfg(&g_pmicHandle, &config);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/* ========================================================================== */
/*                            Test Setup/Teardown                             */
/* ========================================================================== */

/**
 * @brief GPIO test suite entry point (wrapper for test runner)
 * @param args Test arguments (unused)
 *
 * Note: This module doesn't define setUp/tearDown at global scope to avoid
 * conflicts with other modules. Instead, it handles initialization internally.
 */
void gpio_test(void *args)
{
    int32_t status;
    (void)args;  /* Unused parameter */

    printf("\r\n");
    printf("==================================================\r\n");
    printf("    TPS65386x-Q1 GPIO Module Tests\r\n");
    printf("==================================================\r\n\r\n");

    /* Initialize once for all GPIO tests */
    platform_init();
    status = gpioTest_initHandle();
    if (status != PMIC_ST_SUCCESS)
    {
        printf("ERROR: GPIO test initialization failed with status: %d\r\n", status);
        platform_deinit();
        return;
    }

    /* Run all GPIO tests */
    platform_setupTests();
    GPIO_TEST_RUN_ALL();
    platform_tearDownTests();

    /* Cleanup */
    Pmic_deinit(&g_pmicHandle);
    platform_deinit();

    printf("\r\n==================================================\r\n");
    printf("    GPIO Module Tests Complete\r\n");
    printf("==================================================\r\n\r\n");
}
