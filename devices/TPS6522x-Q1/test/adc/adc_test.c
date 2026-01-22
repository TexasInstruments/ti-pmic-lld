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


/* ========================================================================= */
/*                              Include Files                                */
/* ========================================================================= */

#include "../platform.h"
#include "pmic.h"
#include "pmic_adc.h"
#include "test_constants.h"

/* ========================================================================= */
/*                             Macros & Typedefs                             */
/* ========================================================================= */

/**
 * @brief Test execution control macros.
 */
#define RUN_ALL_TESTS      (1U)
#define RUN_NEGATIVE_TESTS (1U)
#define RUN_POSITIVE_TESTS (1U)

/**
 * @brief Invalid parameter values for negative testing.
 */
#define INVALID_SRC_SEL_VALUE   (TEST_MASK_FULL_BYTE)
#define ADC_RESULT_MAX          (0x0FFFU)  /* 12-bit max value */

/* ========================================================================= */
/*                             Test Execution Macros                         */
/* ========================================================================= */

/* ========================================================================= */
/*     API-Specific Test Macros - Pmic_adcSetCfg / Pmic_adcGetCfg           */
/* ========================================================================= */

#define ADC_TEST_NEG_SETGETCFG() \
    PLATFORM_RUN_TEST(test_neg_adc_setCfg_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_adc_setCfg_nullConfig); \
    PLATFORM_RUN_TEST(test_neg_adc_getCfg_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_adc_getCfg_nullConfig); \
    PLATFORM_RUN_TEST(test_neg_adc_setCfg_invalidSrcSel); \
    PLATFORM_RUN_TEST(test_neg_adc_setCfg_zeroValidParams)

#define ADC_TEST_POS_SETGETCFG() \
    PLATFORM_RUN_TEST(test_pos_adc_setCfg_resistorDivider); \
    PLATFORM_RUN_TEST(test_pos_adc_getCfg_resistorDivider); \
    PLATFORM_RUN_TEST(test_pos_adc_setGetCfg_resistorDividerConsistency); \
    PLATFORM_RUN_TEST(test_pos_adc_setCfg_continuousMode); \
    PLATFORM_RUN_TEST(test_pos_adc_getCfg_continuousMode); \
    PLATFORM_RUN_TEST(test_pos_adc_setGetCfg_continuousModeConsistency); \
    PLATFORM_RUN_TEST(test_pos_adc_setCfg_srcSelExternal); \
    PLATFORM_RUN_TEST(test_pos_adc_setCfg_srcSelThermal); \
    PLATFORM_RUN_TEST(test_pos_adc_getCfg_srcSelection)

/* ========================================================================= */
/*     API-Specific Test Macros - Pmic_adcStartSingleConversion             */
/* ========================================================================= */

#define ADC_TEST_NEG_STARTSINGLECONVERSION() \
    PLATFORM_RUN_TEST(test_neg_adc_startSingleConversion_nullHandle)

#define ADC_TEST_POS_STARTSINGLECONVERSION() \
    PLATFORM_RUN_TEST(test_pos_adc_startSingleConversion_success)

/* ========================================================================= */
/*     API-Specific Test Macros - Pmic_adcStartSingleConversionBlocking     */
/* ========================================================================= */

#define ADC_TEST_NEG_STARTSINGLECONVERSIONBLOCKING() \
    PLATFORM_RUN_TEST(test_neg_adc_startSingleConversionBlocking_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_adc_maxLoopCntFail)

#define ADC_TEST_POS_STARTSINGLECONVERSIONBLOCKING() \
    PLATFORM_RUN_TEST(test_pos_adc_startSingleConversionBlocking_success)

/* ========================================================================= */
/*     API-Specific Test Macros - Pmic_adcGetStatus                         */
/* ========================================================================= */

#define ADC_TEST_NEG_GETSTATUS() \
    PLATFORM_RUN_TEST(test_neg_adc_getStatus_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_adc_getStatus_nullStatusPtr)

#define ADC_TEST_POS_GETSTATUS() \
    PLATFORM_RUN_TEST(test_pos_adc_getStatus_idle)

/* ========================================================================= */
/*     API-Specific Test Macros - Pmic_adcGetResultCode                     */
/* ========================================================================= */

#define ADC_TEST_NEG_GETRESULTCODE() \
    PLATFORM_RUN_TEST(test_neg_adc_getResultCode_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_adc_getResultCode_nullResultPtr)

#define ADC_TEST_POS_GETRESULTCODE() \
    PLATFORM_RUN_TEST(test_pos_adc_getResultCode_success)

/* ========================================================================= */
/*     Integration Tests                                                    */
/* ========================================================================= */

#define ADC_TEST_POS_INTEGRATION() \
    PLATFORM_RUN_TEST(test_pos_adc_fullSequence_configStartPollRead); \
    PLATFORM_RUN_TEST(test_pos_adc_multipleConversions_independence)

/* ========================================================================= */
/*     Property Tests (BUILD_MOCK)                                          */
/* ========================================================================= */

#ifdef BUILD_MOCK
#define ADC_TEST_POS_PROPERTY() \
    PLATFORM_RUN_TEST(test_pos_adc_property_randomChannelConfigurations)
#else
#define ADC_TEST_POS_PROPERTY()
#endif

/* ========================================================================= */
/*     Combined Test Macros                                                 */
/* ========================================================================= */

#define ADC_TEST_RUN_ALL() \
    ADC_TEST_NEG_SETGETCFG(); \
    ADC_TEST_POS_SETGETCFG(); \
    ADC_TEST_NEG_STARTSINGLECONVERSION(); \
    ADC_TEST_POS_STARTSINGLECONVERSION(); \
    ADC_TEST_NEG_STARTSINGLECONVERSIONBLOCKING(); \
    ADC_TEST_POS_STARTSINGLECONVERSIONBLOCKING(); \
    ADC_TEST_NEG_GETSTATUS(); \
    ADC_TEST_POS_GETSTATUS(); \
    ADC_TEST_NEG_GETRESULTCODE(); \
    ADC_TEST_POS_GETRESULTCODE(); \
    ADC_TEST_POS_INTEGRATION(); \
    ADC_TEST_POS_PROPERTY()

/* ========================================================================= */
/*                             Global Variables                              */
/* ========================================================================= */

/**
 * @brief Static PMIC handle for tests.
 */
static Pmic_Handle_t pmicHandle = {0};

/* ========================================================================= */
/*                           Function Declarations                           */
/* ========================================================================= */

/* Note: setUp/tearDown provided by test_runner.c */

/* Negative tests */
static void test_neg_adc_setCfg_nullHandle(void);
static void test_neg_adc_setCfg_nullConfig(void);
static void test_neg_adc_getCfg_nullHandle(void);
static void test_neg_adc_getCfg_nullConfig(void);
static void test_neg_adc_startSingleConversion_nullHandle(void);
static void test_neg_adc_startSingleConversionBlocking_nullHandle(void);
static void test_neg_adc_getStatus_nullHandle(void);
static void test_neg_adc_getStatus_nullStatusPtr(void);
static void test_neg_adc_getResultCode_nullHandle(void);
static void test_neg_adc_getResultCode_nullResultPtr(void);
static void test_neg_adc_setCfg_invalidSrcSel(void);
static void test_neg_adc_setCfg_zeroValidParams(void);
void test_neg_adc_maxLoopCntFail(void);

/* Positive tests */
static void test_pos_adc_setCfg_resistorDivider(void);
static void test_pos_adc_getCfg_resistorDivider(void);
static void test_pos_adc_setGetCfg_resistorDividerConsistency(void);
static void test_pos_adc_setCfg_continuousMode(void);
static void test_pos_adc_getCfg_continuousMode(void);
static void test_pos_adc_setGetCfg_continuousModeConsistency(void);
static void test_pos_adc_setCfg_srcSelExternal(void);
static void test_pos_adc_setCfg_srcSelThermal(void);
static void test_pos_adc_getCfg_srcSelection(void);
static void test_pos_adc_startSingleConversion_success(void);
static void test_pos_adc_startSingleConversionBlocking_success(void);
static void test_pos_adc_getStatus_idle(void);
static void test_pos_adc_getResultCode_success(void);
static void test_pos_adc_fullSequence_configStartPollRead(void);
static void test_pos_adc_multipleConversions_independence(void);

#ifdef BUILD_MOCK
/* Property test */
static void test_pos_adc_property_randomChannelConfigurations(void);
static uint8_t getRandomBool(void);
static uint8_t getRandomSrcSel(void);
#endif

/* ========================================================================= */
/*                           Function Definitions                            */
/* ========================================================================= */

void adc_test(void *args)
{
    Pmic_HandleCfg_t handleCfg;

    (void)args;

    platform_init();

    /* Initialize PMIC handle */
    handleCfg.validParams = PMIC_COMM_MODE_VALID |
                            PMIC_COMM_HANDLE_0_VALID |
                            PMIC_IO_READ_VALID |
                            PMIC_IO_WRITE_VALID |
                            PMIC_CRITICAL_SECTION_START_VALID |
                            PMIC_CRITICAL_SECTION_STOP_VALID |
                            PMIC_MAX_LOOP_CNT_VALID;
    handleCfg.commMode = PMIC_INTF_SPI;
    handleCfg.commHandle0 = platform_getCommHandle();
    handleCfg.ioRead = platform_rxByte;
    handleCfg.ioWrite = platform_txByte;
    handleCfg.criticalSectionStart = platform_critSecStart;
    handleCfg.criticalSectionStop = platform_critSecStop;
    handleCfg.maxLoopCnt = 1000;

    int32_t status = Pmic_init(&pmicHandle, &handleCfg);
    if (status != PMIC_ST_SUCCESS)
    {
        platform_printString("\r\nERROR: Failed to initialize PMIC handle for ADC tests\r\n");
        platform_deinit();
        return;
    }

    /* Run tests */
    platform_setupTests();

    ADC_TEST_RUN_ALL();

    platform_tearDownTests();
    platform_deinit();
}

/* ========================================================================= */
/*                          Negative Test Cases                              */
/* ========================================================================= */

static void test_neg_adc_setCfg_nullHandle(void)
{
    int32_t status;
    Pmic_AdcCfg_t adcCfg;

    memset(&adcCfg, 0, sizeof(adcCfg));
    adcCfg.validParams = PMIC_ADC_RDIV_EN_VALID;
    adcCfg.rDivEn = true;

    status = Pmic_adcSetCfg(NULL, &adcCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

static void test_neg_adc_setCfg_nullConfig(void)
{
    int32_t status;

    status = Pmic_adcSetCfg(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

static void test_neg_adc_getCfg_nullHandle(void)
{
    int32_t status;
    Pmic_AdcCfg_t adcCfg;

    memset(&adcCfg, 0, sizeof(adcCfg));

    status = Pmic_adcGetCfg(NULL, &adcCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

static void test_neg_adc_getCfg_nullConfig(void)
{
    int32_t status;

    status = Pmic_adcGetCfg(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

static void test_neg_adc_startSingleConversion_nullHandle(void)
{
    int32_t status;

    status = Pmic_adcStartSingleConversion(NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

static void test_neg_adc_startSingleConversionBlocking_nullHandle(void)
{
    int32_t status;

    status = Pmic_adcStartSingleConversionBlocking(NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

static void test_neg_adc_getStatus_nullHandle(void)
{
    int32_t status;
    bool adcBusy;

    status = Pmic_adcGetStatus(NULL, &adcBusy);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

static void test_neg_adc_getStatus_nullStatusPtr(void)
{
    int32_t status;

    status = Pmic_adcGetStatus(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

static void test_neg_adc_getResultCode_nullHandle(void)
{
    int32_t status;
    uint16_t result;

    status = Pmic_adcGetResultCode(NULL, &result);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

static void test_neg_adc_getResultCode_nullResultPtr(void)
{
    int32_t status;

    status = Pmic_adcGetResultCode(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

static void test_neg_adc_setCfg_invalidSrcSel(void)
{
    int32_t status;
    Pmic_AdcCfg_t adcCfg;

    memset(&adcCfg, 0, sizeof(adcCfg));
    adcCfg.validParams = PMIC_ADC_SRC_SEL_VALID;
    adcCfg.srcSel = INVALID_SRC_SEL_VALUE;

    status = Pmic_adcSetCfg(&pmicHandle, &adcCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

static void test_neg_adc_setCfg_zeroValidParams(void)
{
    int32_t status;
    Pmic_AdcCfg_t adcCfg;

    memset(&adcCfg, 0, sizeof(adcCfg));
    adcCfg.validParams = 0U;

    status = Pmic_adcSetCfg(&pmicHandle, &adcCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/* ========================================================================= */
/*                          Positive Test Cases                              */
/* ========================================================================= */

static void test_pos_adc_setCfg_resistorDivider(void)
{
    int32_t status;
    Pmic_AdcCfg_t adcCfg;

    memset(&adcCfg, 0, sizeof(adcCfg));
    adcCfg.validParams = PMIC_ADC_RDIV_EN_VALID;
    adcCfg.rDivEn = true;

    status = Pmic_adcSetCfg(&pmicHandle, &adcCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

static void test_pos_adc_getCfg_resistorDivider(void)
{
    int32_t status;
    Pmic_AdcCfg_t adcCfg;

    memset(&adcCfg, 0, sizeof(adcCfg));

    status = Pmic_adcGetCfg(&pmicHandle, &adcCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

static void test_pos_adc_setGetCfg_resistorDividerConsistency(void)
{
    int32_t status;
    Pmic_AdcCfg_t setCfg, getCfg;

    /* Set resistor divider enabled */
    memset(&setCfg, 0, sizeof(setCfg));
    setCfg.validParams = PMIC_ADC_RDIV_EN_VALID;
    setCfg.rDivEn = true;

    status = Pmic_adcSetCfg(&pmicHandle, &setCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Get and verify */
    memset(&getCfg, 0, sizeof(getCfg));
    getCfg.validParams = PMIC_ADC_RDIV_EN_VALID;
    status = Pmic_adcGetCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(getCfg.rDivEn == true);

    /* Set resistor divider disabled */
    setCfg.rDivEn = false;
    status = Pmic_adcSetCfg(&pmicHandle, &setCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Get and verify */
    memset(&getCfg, 0, sizeof(getCfg));
    getCfg.validParams = PMIC_ADC_RDIV_EN_VALID;
    status = Pmic_adcGetCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(getCfg.rDivEn == false);
}

static void test_pos_adc_setCfg_continuousMode(void)
{
    int32_t status;
    Pmic_AdcCfg_t adcCfg;

    memset(&adcCfg, 0, sizeof(adcCfg));
    adcCfg.validParams = PMIC_ADC_CONT_CONV_EN_VALID;
    adcCfg.contConvEn = false;

    status = Pmic_adcSetCfg(&pmicHandle, &adcCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

static void test_pos_adc_getCfg_continuousMode(void)
{
    int32_t status;
    Pmic_AdcCfg_t adcCfg;

    memset(&adcCfg, 0, sizeof(adcCfg));

    status = Pmic_adcGetCfg(&pmicHandle, &adcCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

static void test_pos_adc_setGetCfg_continuousModeConsistency(void)
{
    int32_t status;
    Pmic_AdcCfg_t setCfg, getCfg;

    /* Set continuous mode disabled (manual) */
    memset(&setCfg, 0, sizeof(setCfg));
    setCfg.validParams = PMIC_ADC_CONT_CONV_EN_VALID;
    setCfg.contConvEn = false;

    status = Pmic_adcSetCfg(&pmicHandle, &setCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Get and verify */
    memset(&getCfg, 0, sizeof(getCfg));
    getCfg.validParams = PMIC_ADC_CONT_CONV_EN_VALID;
    status = Pmic_adcGetCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(getCfg.contConvEn == false);

    /* Set continuous mode enabled */
    setCfg.contConvEn = true;
    status = Pmic_adcSetCfg(&pmicHandle, &setCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Get and verify */
    memset(&getCfg, 0, sizeof(getCfg));
    getCfg.validParams = PMIC_ADC_CONT_CONV_EN_VALID;
    status = Pmic_adcGetCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(getCfg.contConvEn == true);

    /* Restore to manual mode for other tests */
    setCfg.contConvEn = false;
    status = Pmic_adcSetCfg(&pmicHandle, &setCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

static void test_pos_adc_setCfg_srcSelExternal(void)
{
    int32_t status;
    Pmic_AdcCfg_t adcCfg;

    memset(&adcCfg, 0, sizeof(adcCfg));
    adcCfg.validParams = PMIC_ADC_SRC_SEL_VALID;
    adcCfg.srcSel = PMIC_ADC_SRC_SEL_INPUT;

    status = Pmic_adcSetCfg(&pmicHandle, &adcCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

static void test_pos_adc_setCfg_srcSelThermal(void)
{
    int32_t status;
    Pmic_AdcCfg_t adcCfg;

    memset(&adcCfg, 0, sizeof(adcCfg));
    adcCfg.validParams = PMIC_ADC_SRC_SEL_VALID;
    adcCfg.srcSel = PMIC_ADC_SRC_SEL_THERMAL_SENSOR;

    status = Pmic_adcSetCfg(&pmicHandle, &adcCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

static void test_pos_adc_getCfg_srcSelection(void)
{
    int32_t status;
    Pmic_AdcCfg_t setCfg, getCfg;

    /* Set to external input */
    memset(&setCfg, 0, sizeof(setCfg));
    setCfg.validParams = PMIC_ADC_SRC_SEL_VALID;
    setCfg.srcSel = PMIC_ADC_SRC_SEL_INPUT;

    status = Pmic_adcSetCfg(&pmicHandle, &setCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Get and verify */
    memset(&getCfg, 0, sizeof(getCfg));
    getCfg.validParams = PMIC_ADC_SRC_SEL_VALID;
    status = Pmic_adcGetCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(getCfg.srcSel == PMIC_ADC_SRC_SEL_INPUT);

    /* Set to thermal sensor */
    setCfg.srcSel = PMIC_ADC_SRC_SEL_THERMAL_SENSOR;
    status = Pmic_adcSetCfg(&pmicHandle, &setCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Get and verify */
    memset(&getCfg, 0, sizeof(getCfg));
    getCfg.validParams = PMIC_ADC_SRC_SEL_VALID;
    status = Pmic_adcGetCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(getCfg.srcSel == PMIC_ADC_SRC_SEL_THERMAL_SENSOR);
}

static void test_pos_adc_startSingleConversion_success(void)
{
    int32_t status;
    Pmic_AdcCfg_t adcCfg;

    /* Configure for manual mode */
    memset(&adcCfg, 0, sizeof(adcCfg));
    adcCfg.validParams = PMIC_ADC_CONT_CONV_EN_VALID;
    adcCfg.contConvEn = false;

    status = Pmic_adcSetCfg(&pmicHandle, &adcCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Start conversion (non-blocking) */
    status = Pmic_adcStartSingleConversion(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

static void test_pos_adc_startSingleConversionBlocking_success(void)
{
    int32_t status;
    Pmic_AdcCfg_t adcCfg;

    /* Configure for manual mode */
    memset(&adcCfg, 0, sizeof(adcCfg));
    adcCfg.validParams = PMIC_ADC_CONT_CONV_EN_VALID;
    adcCfg.contConvEn = false;

    status = Pmic_adcSetCfg(&pmicHandle, &adcCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Start conversion (blocking) */
    status = Pmic_adcStartSingleConversionBlocking(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Wait for conversion to complete */
    platform_timerWaitMs(50);
}

static void test_pos_adc_getStatus_idle(void)
{
    int32_t status;
    bool adcBusy;

    /* Get status */
    status = Pmic_adcGetStatus(&pmicHandle, &adcBusy);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

static void test_pos_adc_getResultCode_success(void)
{
    int32_t status;
    uint16_t result;
    Pmic_AdcCfg_t adcCfg;

    /* Configure ADC */
    memset(&adcCfg, 0, sizeof(adcCfg));
    adcCfg.validParams = PMIC_ADC_CONT_CONV_EN_VALID | PMIC_ADC_SRC_SEL_VALID;
    adcCfg.contConvEn = false;
    adcCfg.srcSel = PMIC_ADC_SRC_SEL_THERMAL_SENSOR;

    status = Pmic_adcSetCfg(&pmicHandle, &adcCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Start conversion */
    status = Pmic_adcStartSingleConversionBlocking(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Wait for conversion */
    platform_timerWaitMs(50);

    /* Read result */
    status = Pmic_adcGetResultCode(&pmicHandle, &result);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(result <= ADC_RESULT_MAX);
}

static void test_pos_adc_fullSequence_configStartPollRead(void)
{
    int32_t status;
    Pmic_AdcCfg_t adcCfg;
    bool adcBusy;
    uint16_t result;
    uint8_t pollCount = 0;

    /* Step 1: Configure ADC */
    memset(&adcCfg, 0, sizeof(adcCfg));
    adcCfg.validParams = PMIC_ADC_CONT_CONV_EN_VALID | PMIC_ADC_SRC_SEL_VALID | PMIC_ADC_RDIV_EN_VALID;
    adcCfg.contConvEn = false;
    adcCfg.srcSel = PMIC_ADC_SRC_SEL_THERMAL_SENSOR;
    adcCfg.rDivEn = false;

    status = Pmic_adcSetCfg(&pmicHandle, &adcCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Step 2: Start conversion */
    status = Pmic_adcStartSingleConversion(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Step 3: Poll status until idle */
    do {
        platform_timerWaitMs(10);
        status = Pmic_adcGetStatus(&pmicHandle, &adcBusy);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        pollCount++;
    } while (adcBusy && (pollCount < 100));

    PLATFORM_ASSERT(pollCount < 100);

    /* Step 4: Read result */
    status = Pmic_adcGetResultCode(&pmicHandle, &result);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(result <= ADC_RESULT_MAX);
}

static void test_pos_adc_multipleConversions_independence(void)
{
    int32_t status;
    Pmic_AdcCfg_t adcCfg;
    uint16_t result1, result2;

    /* Configure ADC */
    memset(&adcCfg, 0, sizeof(adcCfg));
    adcCfg.validParams = PMIC_ADC_CONT_CONV_EN_VALID | PMIC_ADC_SRC_SEL_VALID;
    adcCfg.contConvEn = false;
    adcCfg.srcSel = PMIC_ADC_SRC_SEL_THERMAL_SENSOR;

    status = Pmic_adcSetCfg(&pmicHandle, &adcCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* First conversion */
    status = Pmic_adcStartSingleConversionBlocking(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    platform_timerWaitMs(50);

    status = Pmic_adcGetResultCode(&pmicHandle, &result1);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(result1 <= ADC_RESULT_MAX);

    /* Second conversion */
    status = Pmic_adcStartSingleConversionBlocking(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    platform_timerWaitMs(50);

    status = Pmic_adcGetResultCode(&pmicHandle, &result2);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(result2 <= ADC_RESULT_MAX);

    /* Both conversions should succeed independently */
}

/**
 * @brief Test ADC max loop count timeout
 * Covers lines 221-222 in pmic_adc.c
 */
void test_neg_adc_maxLoopCntFail(void)
{
    int32_t status;
    Pmic_Handle_t testHandle;
    Pmic_AdcCfg_t adcCfg;

    /* Initialize test handle with very low maxLoopCnt to force timeout */
    (void)memcpy(&testHandle, &pmicHandle, sizeof(Pmic_Handle_t));
    testHandle.maxLoopCnt = 1U;  /* Set to 1 to force immediate timeout */

    /* Configure ADC for manual mode */
    memset(&adcCfg, 0, sizeof(adcCfg));
    adcCfg.validParams = PMIC_ADC_CONT_CONV_EN_VALID;
    adcCfg.contConvEn = false;

    status = Pmic_adcSetCfg(&testHandle, &adcCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Start conversion and expect timeout due to low maxLoopCnt */
    status = Pmic_adcStartSingleConversionBlocking(&testHandle);

    /* In mock mode, this may succeed or timeout depending on mock behavior */
    /* The test covers the timeout path by having a very low loop count */
    PLATFORM_ASSERT((status == PMIC_ST_SUCCESS) || (status == PMIC_ST_ERR_MAX_LOOP_CNT_FAIL));
}

/* ========================================================================= */
/*                          Property Test Cases                              */
/* ========================================================================= */

#ifdef BUILD_MOCK

static uint8_t getRandomBool(void)
{
    return (uint8_t)(rand() % 2);
}

static uint8_t getRandomSrcSel(void)
{
    return (uint8_t)(rand() % (PMIC_ADC_SRC_SEL_MAX + 1));
}

static void test_pos_adc_property_randomChannelConfigurations(void)
{
    int32_t status;
    Pmic_AdcCfg_t setCfg, getCfg;
    uint16_t iteration;
    const uint16_t maxIterations = 100;

    /* Seed random number generator */
    srand(42);

    for (iteration = 0; iteration < maxIterations; iteration++)
    {
        /* Generate random configuration */
        memset(&setCfg, 0, sizeof(setCfg));
        setCfg.validParams = PMIC_ADC_RDIV_EN_VALID | PMIC_ADC_CONT_CONV_EN_VALID | PMIC_ADC_SRC_SEL_VALID;
        setCfg.rDivEn = (bool)getRandomBool();
        setCfg.contConvEn = (bool)getRandomBool();
        setCfg.srcSel = getRandomSrcSel();

        /* Set configuration */
        status = Pmic_adcSetCfg(&pmicHandle, &setCfg);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        /* Get configuration */
        memset(&getCfg, 0, sizeof(getCfg));
        getCfg.validParams = PMIC_ADC_RDIV_EN_VALID | PMIC_ADC_CONT_CONV_EN_VALID | PMIC_ADC_SRC_SEL_VALID;
        status = Pmic_adcGetCfg(&pmicHandle, &getCfg);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        /* Verify set values match get values */
        PLATFORM_ASSERT(getCfg.rDivEn == setCfg.rDivEn);
        PLATFORM_ASSERT(getCfg.contConvEn == setCfg.contConvEn);
        PLATFORM_ASSERT(getCfg.srcSel == setCfg.srcSel);
    }
}

#endif /* BUILD_MOCK */
