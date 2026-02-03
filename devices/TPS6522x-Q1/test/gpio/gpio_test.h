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
#ifndef GPIO_TEST_H
#define GPIO_TEST_H



/* ========================================================================== */
/*                              Include Files                                 */
/* ========================================================================== */

#include "platform.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/* ======================================================================== */
/*                         Test APIs: gpioSetPinCfg                         */
/* ======================================================================== */
#define GPIO_TEST_POS_SETPINCFG() \
    PLATFORM_RUN_TEST(test_pos_gpio_gpioSetGetPinCfg_gpio1_configOutput); \
    PLATFORM_RUN_TEST(test_pos_gpio_gpioSetGetPinCfg_gpio1_funcSdoSpi); \
    PLATFORM_RUN_TEST(test_pos_gpio_gpioSetGetPinCfg_gpio3_configInputPullUp); \
    PLATFORM_RUN_TEST(test_pos_gpio_gpioSetGetPinCfg_gpio3_funcPb); \
    PLATFORM_RUN_TEST(test_pos_gpio_gpioSetGetPinCfg_gpio4_configInputPullDown); \
    PLATFORM_RUN_TEST(test_pos_gpio_gpioSetGetPinCfg_gpio5_configOutputOpenDrain); \
    PLATFORM_RUN_TEST(test_pos_gpio_gpioSetGetPinCfg_gpio5_funcWkup); \
    PLATFORM_RUN_TEST(test_pos_gpio_gpioSetGetPinCfg_gpio6_configInputDeglitch); \
    PLATFORM_RUN_TEST(test_pos_gpio_gpioSetGetPinCfg_gpio6_funcSyncClkIn); \
    PLATFORM_RUN_TEST(test_pos_gpio_gpioSetPinCfg_pushPull)

#define GPIO_TEST_NEG_SETPINCFG() \
    PLATFORM_RUN_TEST(test_neg_gpio_gpioSetPinCfg_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_gpio_gpioSetPinCfg_nullConfig); \
    PLATFORM_RUN_TEST(test_neg_gpio_gpioSetPinCfg_zeroValidParams); \
    PLATFORM_RUN_TEST(test_neg_gpio_gpioSetPinCfg_invalidPinBelowMin); \
    PLATFORM_RUN_TEST(test_neg_gpio_gpioSetPinCfg_invalidPinAboveMax); \
    PLATFORM_RUN_TEST(test_neg_gpio_gpioSetPinCfg_invalidDirection); \
    PLATFORM_RUN_TEST(test_neg_gpio_gpioSetPinCfg_invalidFunctionGpio1); \
    PLATFORM_RUN_TEST(test_neg_gpio_gpioSetPinCfg_invalidPullSelect); \
    PLATFORM_RUN_TEST(test_neg_gpio_gpioSetPinCfg_invalidType); \
    PLATFORM_RUN_TEST(test_neg_gpio_gpiosetFunction_invalidFunctionality); \
    PLATFORM_RUN_TEST(test_neg_gpio_setPinCfg_invalidGpioPin)

/* Test: TC-GPIO-0006 */
#define GPIO_TEST_SETPINCFG() \
    GPIO_TEST_POS_SETPINCFG(); \
    GPIO_TEST_NEG_SETPINCFG()

/* ======================================================================== */
/*                         Test APIs: gpioGetPinCfg                         */
/* ======================================================================== */
#define GPIO_TEST_POS_GETPINCFG() \
    PLATFORM_RUN_TEST(test_pos_gpio_gpioSetGetPinCfg_gpio1_getConfig); \
    PLATFORM_RUN_TEST(test_pos_gpio_gpioSetGetPinCfg_gpio3_getConfig); \
    PLATFORM_RUN_TEST(test_pos_gpio_gpioSetGetPinCfg_gpio5_getConfig); \
    PLATFORM_RUN_TEST(test_pos_gpio_gpioSetGetPinCfg_gpio6_getConfig)

#define GPIO_TEST_NEG_GETPINCFG() \
    PLATFORM_RUN_TEST(test_neg_gpio_gpioGetPinCfg_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_gpio_gpioGetPinCfg_nullConfig); \
    PLATFORM_RUN_TEST(test_neg_gpio_gpioGetPinCfg_invalidPin); \
    PLATFORM_RUN_TEST(test_neg_gpio_gpiogetPinCfg_zeroValidParams); \
    PLATFORM_RUN_TEST(test_neg_gpio_getPinCfg_invalidGpioPin)

/* Test: TC-GPIO-0007 */
#define GPIO_TEST_GETPINCFG() \
    GPIO_TEST_POS_GETPINCFG(); \
    GPIO_TEST_NEG_GETPINCFG()

/* ======================================================================== */
/*                         Test APIs: gpioSetPinVal                         */
/* ======================================================================== */
#define GPIO_TEST_POS_SETPINVAL() \
    PLATFORM_RUN_TEST(test_pos_gpio_gpioSetGetPinCfg_gpio1_setHigh); \
    PLATFORM_RUN_TEST(test_pos_gpio_gpioSetGetPinCfg_gpio1_setLow); \
    PLATFORM_RUN_TEST(test_pos_gpio_gpioSetGetPinVal_gpio2_setGetSequence); \
    PLATFORM_RUN_TEST(test_pos_gpio_gpioSetValue_validPin)

#define GPIO_TEST_NEG_SETPINVAL() \
    PLATFORM_RUN_TEST(test_neg_gpio_gpioSetPinVal_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_gpio_gpioSetPinVal_invalidPin)

/* Test: TC-GPIO-0008 */
#define GPIO_TEST_SETPINVAL() \
    GPIO_TEST_POS_SETPINVAL(); \
    GPIO_TEST_NEG_SETPINVAL()

/* ======================================================================== */
/*                         Test APIs: gpioGetPinVal                         */
/* ======================================================================== */
#define GPIO_TEST_POS_GETPINVAL() \
    PLATFORM_RUN_TEST(test_pos_gpio_gpioSetGetPinCfg_gpio1_getValue); \
    PLATFORM_RUN_TEST(test_pos_gpio_gpioGetPinVal_allPins_getValue); \
    PLATFORM_RUN_TEST(test_pos_gpio_gpioGetValue_validPin)

#define GPIO_TEST_NEG_GETPINVAL() \
    PLATFORM_RUN_TEST(test_neg_gpio_gpioGetPinVal_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_gpio_gpioGetPinVal_nullValue); \
    PLATFORM_RUN_TEST(test_neg_gpio_gpioGetPinVal_invalidPin)

/* Test: TC-GPIO-0009 */
#define GPIO_TEST_GETPINVAL() \
    GPIO_TEST_POS_GETPINVAL(); \
    GPIO_TEST_NEG_GETPINVAL()

/* ======================================================================== */
/*                      Test APIs: gpioSetNIntEnDrvCfg                      */
/* ======================================================================== */
#define GPIO_TEST_POS_SETNINTENDDRVCFG() \
    PLATFORM_RUN_TEST(test_pos_gpio_gpioSetGetNIntEnDrvCfg_configNInt); \
    PLATFORM_RUN_TEST(test_pos_gpio_gpioSetGetNIntEnDrvCfg_configEnDrv); \
    PLATFORM_RUN_TEST(test_pos_gpio_gpioSetGetNIntEnDrvCfg_enablePullUp)

#define GPIO_TEST_NEG_SETNINTENDDRVCFG() \
    PLATFORM_RUN_TEST(test_neg_gpio_gpioSetNIntEnDrvCfg_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_gpio_gpioSetNIntEnDrvCfg_nullConfig); \
    PLATFORM_RUN_TEST(test_neg_gpio_gpioSetNIntEnDrvCfg_invalidFunction); \
    PLATFORM_RUN_TEST(test_neg_gpio_gpiosetNIntEnDrvCfg_zeroValidParams)

/* Test: TC-GPIO-0010 */
#define GPIO_TEST_SETNINTENDDRVCFG() \
    GPIO_TEST_POS_SETNINTENDDRVCFG(); \
    GPIO_TEST_NEG_SETNINTENDDRVCFG()

/* ======================================================================== */
/*                      Test APIs: gpioGetNIntEnDrvCfg                      */
/* ======================================================================== */
#define GPIO_TEST_POS_GETNINTENDDRVCFG() \
    PLATFORM_RUN_TEST(test_pos_gpio_gpioSetGetNIntEnDrvCfg_getConfig); \
    PLATFORM_RUN_TEST(test_pos_gpio_getNIntEnDrvCfg_enPuResistor)

#define GPIO_TEST_NEG_GETNINTENDDRVCFG() \
    PLATFORM_RUN_TEST(test_neg_gpio_gpioGetNIntEnDrvCfg_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_gpio_gpioGetNIntEnDrvCfg_nullConfig); \
    PLATFORM_RUN_TEST(test_neg_gpio_gpiogetNIntEnDrvCfg_invalidParam)

/* Test: TC-GPIO-0011 */
#define GPIO_TEST_GETNINTENDDRVCFG() \
    GPIO_TEST_POS_GETNINTENDDRVCFG(); \
    GPIO_TEST_NEG_GETNINTENDDRVCFG()

/* ======================================================================== */
/*                      Test APIs: gpioGetNIntEnDrvVal                      */
/* ======================================================================== */
#define GPIO_TEST_POS_GETNINTENDRVVAL() \
    PLATFORM_RUN_TEST(test_pos_gpio_gpioSetGetNIntEnDrvCfg_getValue)

#define GPIO_TEST_NEG_GETNINTENDRVVAL() \
    PLATFORM_RUN_TEST(test_neg_gpio_gpioGetNIntEnDrvVal_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_gpio_gpioGetNIntEnDrvVal_nullValue)

/* Test: TC-GPIO-0012 */
#define GPIO_TEST_GETNINTENDRVVAL() \
    GPIO_TEST_POS_GETNINTENDRVVAL(); \
    GPIO_TEST_NEG_GETNINTENDRVVAL()

/* ======================================================================== */
/*                     Test APIs: gpioSetEnPbVSenseCfg                      */
/* ======================================================================== */
#define GPIO_TEST_POS_SETENPBVSENSECFG() \
    PLATFORM_RUN_TEST(test_pos_gpio_gpioSetGetEnPbVSenseCfg_configEnable); \
    PLATFORM_RUN_TEST(test_pos_gpio_gpioSetGetEnPbVSenseCfg_configPb); \
    PLATFORM_RUN_TEST(test_pos_gpio_gpioSetGetEnPbVSenseCfg_configVSense); \
    PLATFORM_RUN_TEST(test_pos_gpio_gpioSetGetEnPbVSenseCfg_deglitchEnable); \
    PLATFORM_RUN_TEST(test_pos_gpio_gpioSetGetEnPbVSenseCfg_deglitchPb)

#define GPIO_TEST_NEG_SETENPBVSENSECFG() \
    PLATFORM_RUN_TEST(test_neg_gpio_gpioSetEnPbVSenseCfg_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_gpio_gpioSetEnPbVSenseCfg_nullConfig); \
    PLATFORM_RUN_TEST(test_neg_gpio_gpioSetEnPbVSenseCfg_invalidFunction); \
    PLATFORM_RUN_TEST(test_neg_gpio_gpiosetEnPbVSenseCfg_zeroValidParams); \
    PLATFORM_RUN_TEST(test_neg_gpio_gpiosetEnPbVSenseCfg_invalidDeglitch)

/* Test: TC-GPIO-0013 */
#define GPIO_TEST_SETENPBVSENSECFG() \
    GPIO_TEST_POS_SETENPBVSENSECFG(); \
    GPIO_TEST_NEG_SETENPBVSENSECFG()

/* ======================================================================== */
/*                     Test APIs: gpioGetEnPbVSenseCfg                      */
/* ======================================================================== */
#define GPIO_TEST_POS_GETENPBVSENSECFG() \
    PLATFORM_RUN_TEST(test_pos_gpio_gpioSetGetEnPbVSenseCfg_getConfig); \
    PLATFORM_RUN_TEST(test_pos_gpio_getEnPbDegl)

#define GPIO_TEST_NEG_GETENPBVSENSECFG() \
    PLATFORM_RUN_TEST(test_neg_gpio_gpioGetEnPbVSenseCfg_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_gpio_gpioGetEnPbVSenseCfg_nullConfig); \
    PLATFORM_RUN_TEST(test_neg_gpio_gpiogetEnPbVSenseCfg_invalidParam)

/* Test: TC-GPIO-0014 */
#define GPIO_TEST_GETENPBVSENSECFG() \
    GPIO_TEST_POS_GETENPBVSENSECFG(); \
    GPIO_TEST_NEG_GETENPBVSENSECFG()

/* ======================================================================== */
/*                    Test APIs: gpioGetEnPbVSenseStatus                    */
/* ======================================================================== */
#define GPIO_TEST_POS_GETENPBVSENSESTATUS() \
    PLATFORM_RUN_TEST(test_pos_gpio_gpioSetGetEnPbVSenseCfg_getStatus)

#define GPIO_TEST_NEG_GETENPBVSENSESTATUS() \
    PLATFORM_RUN_TEST(test_neg_gpio_gpioGetEnPbVSenseStatus_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_gpio_gpioGetEnPbVSenseStatus_nullStatus); \
    PLATFORM_RUN_TEST(test_neg_gpio_gpiogetEnPbVSenseStatus_zeroValidParams)

/* Test: TC-GPIO-0015 */
#define GPIO_TEST_GETENPBVSENSESTATUS() \
    GPIO_TEST_POS_GETENPBVSENSESTATUS(); \
    GPIO_TEST_NEG_GETENPBVSENSESTATUS()

/* ======================================================================== */
/*                       Test APIs: gpioGetNRstOutVal                       */
/* ======================================================================== */
#define GPIO_TEST_POS_GETNRSTOUTVAL() \
    PLATFORM_RUN_TEST(test_pos_gpio_gpioGetNRstOutVal_getValue)

#define GPIO_TEST_NEG_GETNRSTOUTVAL() \
    PLATFORM_RUN_TEST(test_neg_gpio_gpioGetNRstOutVal_nullHandle); \
    PLATFORM_RUN_TEST(test_neg_gpio_gpioGetNRstOutVal_nullValue)

/* Test: TC-GPIO-0016 */
#define GPIO_TEST_GETNRSTOUTVAL() \
    GPIO_TEST_POS_GETNRSTOUTVAL(); \
    GPIO_TEST_NEG_GETNRSTOUTVAL()

/* ========================================================================== */
/*                   Property tests (BUILD_MOCK only)                         */
/* ========================================================================== */
#ifdef BUILD_MOCK
#define GPIO_TEST_PROPERTY() \
    PLATFORM_RUN_TEST(test_pos_gpio_property_pinConfigurations)
#else
#define GPIO_TEST_PROPERTY()
#endif

/* ========================================================================== */
/*                        Aggregate Test Macros                               */
/* ========================================================================== */

#define GPIO_TEST_RUN_POSITIVE() \
    GPIO_TEST_POS_SETPINCFG(); \
    GPIO_TEST_POS_GETPINCFG(); \
    GPIO_TEST_POS_SETPINVAL(); \
    GPIO_TEST_POS_GETPINVAL(); \
    GPIO_TEST_POS_SETNINTENDDRVCFG(); \
    GPIO_TEST_POS_GETNINTENDDRVCFG(); \
    GPIO_TEST_POS_GETNINTENDRVVAL(); \
    GPIO_TEST_POS_SETENPBVSENSECFG(); \
    GPIO_TEST_POS_GETENPBVSENSECFG(); \
    GPIO_TEST_POS_GETENPBVSENSESTATUS(); \
    GPIO_TEST_POS_GETNRSTOUTVAL(); \
    GPIO_TEST_PROPERTY()

#define GPIO_TEST_RUN_NEGATIVE() \
    GPIO_TEST_NEG_SETPINCFG(); \
    GPIO_TEST_NEG_GETPINCFG(); \
    GPIO_TEST_NEG_SETPINVAL(); \
    GPIO_TEST_NEG_GETPINVAL(); \
    GPIO_TEST_NEG_SETNINTENDDRVCFG(); \
    GPIO_TEST_NEG_GETNINTENDDRVCFG(); \
    GPIO_TEST_NEG_GETNINTENDRVVAL(); \
    GPIO_TEST_NEG_SETENPBVSENSECFG(); \
    GPIO_TEST_NEG_GETENPBVSENSECFG(); \
    GPIO_TEST_NEG_GETENPBVSENSESTATUS(); \
    GPIO_TEST_NEG_GETNRSTOUTVAL()

#define GPIO_TEST_RUN_ALL() \
    GPIO_TEST_RUN_POSITIVE(); \
    GPIO_TEST_RUN_NEGATIVE()

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 * @brief Main entry point for GPIO module tests
 *
 * @param args Optional arguments (unused)
 */
void gpio_test(void *args);

/* Negative test functions */
void test_neg_gpio_gpioSetPinCfg_nullHandle(void);
void test_neg_gpio_gpioSetPinCfg_nullConfig(void);
void test_neg_gpio_gpioSetPinCfg_zeroValidParams(void);
void test_neg_gpio_gpioSetPinCfg_invalidPinBelowMin(void);
void test_neg_gpio_gpioSetPinCfg_invalidPinAboveMax(void);
void test_neg_gpio_gpioSetPinCfg_invalidDirection(void);
void test_neg_gpio_gpioSetPinCfg_invalidFunctionGpio1(void);
void test_neg_gpio_gpioSetPinCfg_invalidPullSelect(void);
void test_neg_gpio_gpioSetPinCfg_invalidType(void);
void test_neg_gpio_gpioGetPinCfg_nullHandle(void);
void test_neg_gpio_gpioGetPinCfg_nullConfig(void);
void test_neg_gpio_gpioGetPinCfg_invalidPin(void);
void test_neg_gpio_setPinCfg_invalidGpioPin(void);
void test_neg_gpio_getPinCfg_invalidGpioPin(void);
void test_neg_gpio_gpioSetPinVal_nullHandle(void);
void test_neg_gpio_gpioSetPinVal_invalidPin(void);
void test_neg_gpio_gpioGetPinVal_nullHandle(void);
void test_neg_gpio_gpioGetPinVal_nullValue(void);
void test_neg_gpio_gpioGetPinVal_invalidPin(void);
void test_neg_gpio_gpioSetNIntEnDrvCfg_nullHandle(void);
void test_neg_gpio_gpioSetNIntEnDrvCfg_nullConfig(void);
void test_neg_gpio_gpioSetNIntEnDrvCfg_invalidFunction(void);
void test_neg_gpio_gpioGetNIntEnDrvCfg_nullHandle(void);
void test_neg_gpio_gpioGetNIntEnDrvCfg_nullConfig(void);
void test_neg_gpio_gpioGetNIntEnDrvVal_nullHandle(void);
void test_neg_gpio_gpioGetNIntEnDrvVal_nullValue(void);
void test_neg_gpio_gpioSetEnPbVSenseCfg_nullHandle(void);
void test_neg_gpio_gpioSetEnPbVSenseCfg_nullConfig(void);
void test_neg_gpio_gpioSetEnPbVSenseCfg_invalidFunction(void);
void test_neg_gpio_gpioGetEnPbVSenseCfg_nullHandle(void);
void test_neg_gpio_gpioGetEnPbVSenseCfg_nullConfig(void);
void test_neg_gpio_gpioGetEnPbVSenseStatus_nullHandle(void);
void test_neg_gpio_gpioGetEnPbVSenseStatus_nullStatus(void);
void test_neg_gpio_gpioGetNRstOutVal_nullHandle(void);
void test_neg_gpio_gpioGetNRstOutVal_nullValue(void);
void test_neg_gpio_gpiosetFunction_invalidFunctionality(void);
void test_neg_gpio_gpiogetNIntEnDrvCfg_invalidParam(void);
void test_neg_gpio_gpiogetEnPbVSenseCfg_invalidParam(void);
void test_neg_gpio_gpiogetPinCfg_zeroValidParams(void);
void test_neg_gpio_gpiosetNIntEnDrvCfg_zeroValidParams(void);
void test_neg_gpio_gpiosetEnPbVSenseCfg_zeroValidParams(void);
void test_neg_gpio_gpiosetEnPbVSenseCfg_invalidDeglitch(void);
void test_neg_gpio_gpiogetEnPbVSenseStatus_zeroValidParams(void);

/* Positive test functions */
void test_pos_gpio_getNIntEnDrvCfg_enPuResistor(void);
void test_pos_gpio_gpioSetGetPinCfg_gpio1_configOutput(void);
void test_pos_gpio_gpioSetGetPinCfg_gpio1_getConfig(void);
void test_pos_gpio_gpioSetGetPinCfg_gpio1_setHigh(void);
void test_pos_gpio_gpioSetGetPinCfg_gpio1_setLow(void);
void test_pos_gpio_gpioSetGetPinCfg_gpio1_getValue(void);
void test_pos_gpio_gpioSetGetPinCfg_gpio3_configInputPullUp(void);
void test_pos_gpio_gpioSetGetPinCfg_gpio3_getConfig(void);
void test_pos_gpio_gpioSetGetPinCfg_gpio5_configOutputOpenDrain(void);
void test_pos_gpio_gpioSetGetPinCfg_gpio5_getConfig(void);
void test_pos_gpio_gpioSetGetPinCfg_gpio6_configInputDeglitch(void);
void test_pos_gpio_gpioSetGetPinCfg_gpio6_getConfig(void);
void test_pos_gpio_gpioSetGetPinVal_gpio2_setGetSequence(void);
void test_pos_gpio_gpioSetGetPinCfg_gpio4_configInputPullDown(void);
void test_pos_gpio_gpioSetGetPinCfg_gpio1_funcSdoSpi(void);
void test_pos_gpio_gpioSetGetPinVal_gpio2_funcNsleep1(void);
void test_pos_gpio_gpioSetGetPinCfg_gpio3_funcPb(void);
void test_pos_gpio_gpioSetGetPinCfg_gpio5_funcWkup(void);
void test_pos_gpio_gpioSetGetPinCfg_gpio6_funcSyncClkIn(void);
void test_pos_gpio_gpioSetGetNIntEnDrvCfg_configNInt(void);
void test_pos_gpio_gpioSetGetNIntEnDrvCfg_configEnDrv(void);
void test_pos_gpio_gpioSetGetNIntEnDrvCfg_getConfig(void);
void test_pos_gpio_gpioSetGetNIntEnDrvCfg_enablePullUp(void);
void test_pos_gpio_gpioSetGetNIntEnDrvCfg_getValue(void);
void test_pos_gpio_gpioSetGetEnPbVSenseCfg_configEnable(void);
void test_pos_gpio_gpioSetGetEnPbVSenseCfg_configPb(void);
void test_pos_gpio_gpioSetGetEnPbVSenseCfg_configVSense(void);
void test_pos_gpio_gpioSetGetEnPbVSenseCfg_getConfig(void);
void test_pos_gpio_gpioSetGetEnPbVSenseCfg_deglitchEnable(void);
void test_pos_gpio_gpioSetGetEnPbVSenseCfg_deglitchPb(void);
void test_pos_gpio_gpioSetGetEnPbVSenseCfg_getStatus(void);
void test_pos_gpio_gpioGetNRstOutVal_getValue(void);
void test_pos_gpio_gpioSetPinCfg_pushPull(void);
void test_pos_gpio_gpioGetPinVal_allPins_getValue(void);
void test_pos_gpio_gpioGetValue_validPin(void);
void test_pos_gpio_gpioSetValue_validPin(void);
void test_pos_gpio_property_pinConfigurations(void);
void test_pos_gpio_getEnPbDegl(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* GPIO_TEST_H */
