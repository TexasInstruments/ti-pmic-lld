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
#ifndef ADC_TEST_H
#define ADC_TEST_H



/* ========================================================================= */
/*                              Include Files                                */
/* ========================================================================= */

#include "platform.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* ======================================================================== */
/*                           Test APIs: adcSetCfg                           */
/* ======================================================================== */
#define ADC_TEST_POS_ADCSETCFG() \
    PLATFORM_RUN_TEST(test_pos_adc_adcSetCfg_resistorDivider); \
    PLATFORM_RUN_TEST(test_pos_adc_adcSetCfg_continuousMode); \
    PLATFORM_RUN_TEST(test_pos_adc_adcSetCfg_srcSelExternal); \
    PLATFORM_RUN_TEST(test_pos_adc_adcSetCfg_srcSelThermal); \
    PLATFORM_RUN_TEST(test_pos_adc_adcSetGetCfg_resistorDividerConsistency); \
    PLATFORM_RUN_TEST(test_pos_adc_adcSetGetCfg_continuousModeConsistency)

#define ADC_TEST_NEG_ADCSETCFG() \
    PLATFORM_RUN_TEST(test_neg_adc_adcSetCfg_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_adc_adcSetCfg_nullConfig); \
    PLATFORM_RUN_TEST(test_neg_adc_adcSetCfg_invalidSrcSel); \
    PLATFORM_RUN_TEST(test_neg_adc_adcSetCfg_zeroValidParams)

/* Test: TC-ADC-0001 */
#define ADC_TEST_ADCSETCFG() \
    ADC_TEST_POS_ADCSETCFG(); \
    ADC_TEST_NEG_ADCSETCFG()

/* ======================================================================== */
/*                           Test APIs: adcGetCfg                           */
/* ======================================================================== */
#define ADC_TEST_POS_ADCGETCFG() \
    PLATFORM_RUN_TEST(test_pos_adc_adcGetCfg_resistorDivider); \
    PLATFORM_RUN_TEST(test_pos_adc_adcGetCfg_continuousMode); \
    PLATFORM_RUN_TEST(test_pos_adc_adcGetCfg_srcSelection)

#define ADC_TEST_NEG_ADCGETCFG() \
    PLATFORM_RUN_TEST(test_neg_adc_adcGetCfg_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_adc_adcGetCfg_nullConfig)

/* Test: TC-ADC-0002 */
#define ADC_TEST_ADCGETCFG() \
    ADC_TEST_POS_ADCGETCFG(); \
    ADC_TEST_NEG_ADCGETCFG()

/* ======================================================================== */
/*                   Test APIs: adcStartSingleConversion                    */
/* ======================================================================== */
#define ADC_TEST_POS_ADCSTARTSINGLECONVERSION() \
    PLATFORM_RUN_TEST(test_pos_adc_adcStartSingleConversion_success)

#define ADC_TEST_NEG_ADCSTARTSINGLECONVERSION() \
    PLATFORM_RUN_TEST(test_neg_adc_adcStartSingleConversion_nullHandle)

/* Test: TC-ADC-0003 */
#define ADC_TEST_ADCSTARTSINGLECONVERSION() \
    ADC_TEST_POS_ADCSTARTSINGLECONVERSION(); \
    ADC_TEST_NEG_ADCSTARTSINGLECONVERSION()

/* ======================================================================== */
/*               Test APIs: adcStartSingleConversionBlocking                */
/* ======================================================================== */
#define ADC_TEST_POS_ADCSTARTSINGLECONVERSIONBLOCKING() \
    PLATFORM_RUN_TEST(test_pos_adc_adcStartSingleConversionBlocking_success)

#define ADC_TEST_NEG_ADCSTARTSINGLECONVERSIONBLOCKING() \
    PLATFORM_RUN_TEST(test_neg_adc_adcStartSingleConversionBlocking_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_adc_maxLoopCntFail)

/* Test: TC-ADC-0004 */
#define ADC_TEST_ADCSTARTSINGLECONVERSIONBLOCKING() \
    ADC_TEST_POS_ADCSTARTSINGLECONVERSIONBLOCKING(); \
    ADC_TEST_NEG_ADCSTARTSINGLECONVERSIONBLOCKING()

/* ======================================================================== */
/*                         Test APIs: adcGetStatus                          */
/* ======================================================================== */
#define ADC_TEST_POS_ADCGETSTATUS() \
    PLATFORM_RUN_TEST(test_pos_adc_adcGetStatus_idle)

#define ADC_TEST_NEG_ADCGETSTATUS() \
    PLATFORM_RUN_TEST(test_neg_adc_adcGetStatus_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_adc_adcGetStatus_nullStatusPtr)

/* Test: TC-ADC-0005 */
#define ADC_TEST_ADCGETSTATUS() \
    ADC_TEST_POS_ADCGETSTATUS(); \
    ADC_TEST_NEG_ADCGETSTATUS()

/* ======================================================================== */
/*                       Test APIs: adcGetResultCode                        */
/* ======================================================================== */
#define ADC_TEST_POS_ADCGETRESULTCODE() \
    PLATFORM_RUN_TEST(test_pos_adc_adcGetResultCode_success)

#define ADC_TEST_NEG_ADCGETRESULTCODE() \
    PLATFORM_RUN_TEST(test_neg_adc_adcGetResultCode_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_adc_adcGetResultCode_nullResultPtr)

/* Test: TC-ADC-0006 */
#define ADC_TEST_ADCGETRESULTCODE() \
    ADC_TEST_POS_ADCGETRESULTCODE(); \
    ADC_TEST_NEG_ADCGETRESULTCODE()

/* ======================================================================== */
/* Test APIs: adcSetCfg, adcStartSingleConversionBlocking, adcGetResultCode */
/* ======================================================================== */
/* Test: TC-ADC-0007 */
#define ADC_TEST_PROPERTY() \
    PLATFORM_RUN_TEST(test_pos_adc_fullSequence_configStartPollRead); \
    PLATFORM_RUN_TEST(test_pos_adc_multipleConversions_independence)

/* ======================================================================== */
/* Test APIs: adcSetCfg, adcStartSingleConversionBlocking, adcGetResultCode */
/* ======================================================================== */
#ifdef BUILD_MOCK
/* Test: TC-ADC-0008 */
#define ADC_TEST_PROPERTY_MOCK_ONLY() \
    PLATFORM_RUN_TEST(test_pos_adc_property_randomChannelConfigurations)
#else
#define ADC_TEST_PROPERTY_MOCK_ONLY()
#endif

/* ========================================================================= */
/*                        Aggregate Test Macros                              */
/* ========================================================================= */

#define ADC_TEST_RUN_POSITIVE() \
    ADC_TEST_POS_ADCSETCFG(); \
    ADC_TEST_POS_ADCGETCFG(); \
    ADC_TEST_POS_ADCSTARTSINGLECONVERSION(); \
    ADC_TEST_POS_ADCSTARTSINGLECONVERSIONBLOCKING(); \
    ADC_TEST_POS_ADCGETSTATUS(); \
    ADC_TEST_POS_ADCGETRESULTCODE(); \
    ADC_TEST_PROPERTY(); \
    ADC_TEST_PROPERTY_MOCK_ONLY()

#define ADC_TEST_RUN_NEGATIVE() \
    ADC_TEST_NEG_ADCSETCFG(); \
    ADC_TEST_NEG_ADCGETCFG(); \
    ADC_TEST_NEG_ADCSTARTSINGLECONVERSION(); \
    ADC_TEST_NEG_ADCSTARTSINGLECONVERSIONBLOCKING(); \
    ADC_TEST_NEG_ADCGETSTATUS(); \
    ADC_TEST_NEG_ADCGETRESULTCODE()

#define ADC_TEST_RUN_ALL() \
    ADC_TEST_RUN_POSITIVE(); \
    ADC_TEST_RUN_NEGATIVE()

/* ========================================================================= */
/*                          Function Declarations                            */
/* ========================================================================= */

/**
 * @brief Entry point for ADC module tests.
 *
 * @param args [IN] Test arguments (unused).
 */
void adc_test(void *args);

/* Negative test functions */
void test_neg_adc_adcSetCfg_nullHandle(void);
void test_neg_adc_adcSetCfg_nullConfig(void);
void test_neg_adc_adcGetCfg_nullHandle(void);
void test_neg_adc_adcGetCfg_nullConfig(void);
void test_neg_adc_adcStartSingleConversion_nullHandle(void);
void test_neg_adc_adcStartSingleConversionBlocking_nullHandle(void);
void test_neg_adc_adcGetStatus_nullHandle(void);
void test_neg_adc_adcGetStatus_nullStatusPtr(void);
void test_neg_adc_adcGetResultCode_nullHandle(void);
void test_neg_adc_adcGetResultCode_nullResultPtr(void);
void test_neg_adc_adcSetCfg_invalidSrcSel(void);
void test_neg_adc_adcSetCfg_zeroValidParams(void);

/* Positive test functions */
void test_pos_adc_adcSetCfg_resistorDivider(void);
void test_pos_adc_adcGetCfg_resistorDivider(void);
void test_pos_adc_adcSetGetCfg_resistorDividerConsistency(void);
void test_pos_adc_adcSetCfg_continuousMode(void);
void test_pos_adc_adcGetCfg_continuousMode(void);
void test_pos_adc_adcSetGetCfg_continuousModeConsistency(void);
void test_pos_adc_adcSetCfg_srcSelExternal(void);
void test_pos_adc_adcSetCfg_srcSelThermal(void);
void test_pos_adc_adcGetCfg_srcSelection(void);
void test_pos_adc_adcStartSingleConversion_success(void);
void test_pos_adc_adcStartSingleConversionBlocking_success(void);
void test_pos_adc_adcGetStatus_idle(void);
void test_pos_adc_adcGetResultCode_success(void);
void test_pos_adc_fullSequence_configStartPollRead(void);
void test_pos_adc_multipleConversions_independence(void);
void test_pos_adc_property_randomChannelConfigurations(void);

#ifdef __cplusplus
}
#endif

#endif /* ADC_TEST_H */
