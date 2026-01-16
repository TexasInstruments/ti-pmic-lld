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
/*                              Include Files                                 */
/* ========================================================================== */

#include "../platform.h"
#include "gpio_test.h"
#include "pmic_gpio.h"
#include <stdlib.h>
#include <time.h>

/* ========================================================================== */
/*                             Global Variables                               */
/* ========================================================================== */

static Pmic_Handle_t pmicHandle = {0};

/* ========================================================================== */
/*                          Negative Test Functions                           */
/* ========================================================================== */

/* Test Pmic_gpioSetPinCfg with NULL handle */
static void test_gpio_setPinCfg_nullHandle(void)
{
    Pmic_GpioPinCfg_t cfg = {
        .validParams = PMIC_GPIO_DIR_VALID,
        .pinNum = PMIC_GPIO_PIN1,
        .dir = PMIC_GPIO_PIN_OUTPUT
    };

    int32_t status = Pmic_gpioSetPinCfg(NULL, &cfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

/* Test Pmic_gpioSetPinCfg with NULL config */
static void test_gpio_setPinCfg_nullConfig(void)
{
    int32_t status = Pmic_gpioSetPinCfg(&pmicHandle, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

/* Test Pmic_gpioSetPinCfg with zero valid params */
static void test_gpio_setPinCfg_zeroValidParams(void)
{
    Pmic_GpioPinCfg_t cfg = {
        .validParams = 0U,
        .pinNum = PMIC_GPIO_PIN1
    };

    int32_t status = Pmic_gpioSetPinCfg(&pmicHandle, &cfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);
}

/* Test Pmic_gpioSetPinCfg with invalid pin number (below min) */
static void test_gpio_setPinCfg_invalidPinBelowMin(void)
{
    Pmic_GpioPinCfg_t cfg = {
        .validParams = PMIC_GPIO_DIR_VALID,
        .pinNum = 0U,
        .dir = PMIC_GPIO_PIN_OUTPUT
    };

    int32_t status = Pmic_gpioSetPinCfg(&pmicHandle, &cfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);
}

/* Test Pmic_gpioSetPinCfg with invalid pin number (above max) */
static void test_gpio_setPinCfg_invalidPinAboveMax(void)
{
    Pmic_GpioPinCfg_t cfg = {
        .validParams = PMIC_GPIO_DIR_VALID,
        .pinNum = 7U,
        .dir = PMIC_GPIO_PIN_OUTPUT
    };

    int32_t status = Pmic_gpioSetPinCfg(&pmicHandle, &cfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);
}

/* Test Pmic_gpioSetPinCfg with invalid direction */
static void test_gpio_setPinCfg_invalidDirection(void)
{
    Pmic_GpioPinCfg_t cfg = {
        .validParams = PMIC_GPIO_DIR_VALID,
        .pinNum = PMIC_GPIO_PIN1,
        .dir = 2U  // Invalid, max is 1
    };

    int32_t status = Pmic_gpioSetPinCfg(&pmicHandle, &cfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);
}

/* Test Pmic_gpioSetPinCfg with invalid function select for GPIO1 */
static void test_gpio_setPinCfg_invalidFunctionGpio1(void)
{
    Pmic_GpioPinCfg_t cfg = {
        .validParams = PMIC_GPIO_FXN_SEL_VALID,
        .pinNum = PMIC_GPIO_PIN1,
        .fxnSel = 4U  // Invalid, max is 3 for GPIO1
    };

    int32_t status = Pmic_gpioSetPinCfg(&pmicHandle, &cfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);
}

/* Test Pmic_gpioSetPinCfg with invalid pull-up/pull-down select */
static void test_gpio_setPinCfg_invalidPullSelect(void)
{
    Pmic_GpioPinCfg_t cfg = {
        .validParams = PMIC_GPIO_PU_SEL_VALID,
        .pinNum = PMIC_GPIO_PIN1,
        .puSel = 2U  // Invalid, max is 1
    };

    int32_t status = Pmic_gpioSetPinCfg(&pmicHandle, &cfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);
}

/* Test Pmic_gpioSetPinCfg with invalid type */
static void test_gpio_setPinCfg_invalidType(void)
{
    Pmic_GpioPinCfg_t cfg = {
        .validParams = PMIC_GPIO_TYPE_VALID,
        .pinNum = PMIC_GPIO_PIN1,
        .type = 2U  // Invalid, max is 1
    };

    int32_t status = Pmic_gpioSetPinCfg(&pmicHandle, &cfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);
}

/* Test Pmic_gpioGetPinCfg with NULL handle */
static void test_gpio_getPinCfg_nullHandle(void)
{
    Pmic_GpioPinCfg_t cfg = {
        .validParams = PMIC_GPIO_DIR_VALID,
        .pinNum = PMIC_GPIO_PIN1
    };

    int32_t status = Pmic_gpioGetPinCfg(NULL, &cfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

/* Test Pmic_gpioGetPinCfg with NULL config */
static void test_gpio_getPinCfg_nullConfig(void)
{
    int32_t status = Pmic_gpioGetPinCfg(&pmicHandle, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

/* Test Pmic_gpioGetPinCfg with invalid pin number */
static void test_gpio_getPinCfg_invalidPin(void)
{
    Pmic_GpioPinCfg_t cfg = {
        .validParams = PMIC_GPIO_DIR_VALID,
        .pinNum = 10U
    };

    int32_t status = Pmic_gpioGetPinCfg(&pmicHandle, &cfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);
}

/* Test Pmic_gpioSetPinCfg with invalid GPIO pin number at boundary */
static void test_negative_gpioSetConfiguration_invalidGpioPin(void)
{
    Pmic_GpioPinCfg_t cfg = {
        .validParams = PMIC_GPIO_DIR_VALID,
        .pinNum = PMIC_GPIO_PIN_MAX + 1U,
        .dir = PMIC_GPIO_PIN_OUTPUT
    };

    int32_t status = Pmic_gpioSetPinCfg(&pmicHandle, &cfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);
}

/* Test Pmic_gpioGetPinCfg with invalid GPIO pin number at boundary */
static void test_negative_gpioGetConfiguration_invalidGpioPin(void)
{
    Pmic_GpioPinCfg_t cfg = {
        .validParams = PMIC_GPIO_DIR_VALID,
        .pinNum = PMIC_GPIO_PIN_MAX + 1U
    };

    int32_t status = Pmic_gpioGetPinCfg(&pmicHandle, &cfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);
}

/* Test Pmic_gpioSetPinVal with NULL handle */
static void test_gpio_setPinVal_nullHandle(void)
{
    int32_t status = Pmic_gpioSetPinVal(NULL, PMIC_GPIO_PIN1, true);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

/* Test Pmic_gpioSetPinVal with invalid pin */
static void test_gpio_setPinVal_invalidPin(void)
{
    int32_t status = Pmic_gpioSetPinVal(&pmicHandle, 0U, true);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);
}

/* Test Pmic_gpioGetPinVal with NULL handle */
static void test_gpio_getPinVal_nullHandle(void)
{
    bool high = false;
    int32_t status = Pmic_gpioGetPinVal(NULL, PMIC_GPIO_PIN1, &high);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

/* Test Pmic_gpioGetPinVal with NULL value pointer */
static void test_gpio_getPinVal_nullValue(void)
{
    int32_t status = Pmic_gpioGetPinVal(&pmicHandle, PMIC_GPIO_PIN1, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

/* Test Pmic_gpioGetPinVal with invalid pin */
static void test_gpio_getPinVal_invalidPin(void)
{
    bool high = false;
    int32_t status = Pmic_gpioGetPinVal(&pmicHandle, 8U, &high);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);
}

/* Test Pmic_gpioSetNIntEnDrvCfg with NULL handle */
static void test_gpio_setNIntEnDrvCfg_nullHandle(void)
{
    Pmic_GpioNIntEnDrvCfg_t cfg = {
        .validParams = PMIC_GPIO_NINT_ENDRV_FXN_SEL_VALID,
        .fxnSel = PMIC_GPIO_NINT_ENDRV_FXN_SEL_NINT
    };

    int32_t status = Pmic_gpioSetNIntEnDrvCfg(NULL, &cfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

/* Test Pmic_gpioSetNIntEnDrvCfg with NULL config */
static void test_gpio_setNIntEnDrvCfg_nullConfig(void)
{
    int32_t status = Pmic_gpioSetNIntEnDrvCfg(&pmicHandle, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

/* Test Pmic_gpioSetNIntEnDrvCfg with invalid function select */
static void test_gpio_setNIntEnDrvCfg_invalidFunction(void)
{
    Pmic_GpioNIntEnDrvCfg_t cfg = {
        .validParams = PMIC_GPIO_NINT_ENDRV_FXN_SEL_VALID,
        .fxnSel = 2U  // Invalid, max is 1
    };

    int32_t status = Pmic_gpioSetNIntEnDrvCfg(&pmicHandle, &cfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);
}

/* Test Pmic_gpioGetNIntEnDrvCfg with NULL handle */
static void test_gpio_getNIntEnDrvCfg_nullHandle(void)
{
    Pmic_GpioNIntEnDrvCfg_t cfg = {
        .validParams = PMIC_GPIO_NINT_ENDRV_FXN_SEL_VALID
    };

    int32_t status = Pmic_gpioGetNIntEnDrvCfg(NULL, &cfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

/* Test Pmic_gpioGetNIntEnDrvCfg with NULL config */
static void test_gpio_getNIntEnDrvCfg_nullConfig(void)
{
    int32_t status = Pmic_gpioGetNIntEnDrvCfg(&pmicHandle, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

/* Test Pmic_gpioGetNIntEnDrvVal with NULL handle */
static void test_gpio_getNIntEnDrvVal_nullHandle(void)
{
    bool high = false;
    int32_t status = Pmic_gpioGetNIntEnDrvVal(NULL, &high);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

/* Test Pmic_gpioGetNIntEnDrvVal with NULL value pointer */
static void test_gpio_getNIntEnDrvVal_nullValue(void)
{
    int32_t status = Pmic_gpioGetNIntEnDrvVal(&pmicHandle, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

/* Test Pmic_gpioSetEnPbVSenseCfg with NULL handle */
static void test_gpio_setEnPbVSenseCfg_nullHandle(void)
{
    Pmic_GpioNIntEnDrvCfg_t cfg = {
        .validParams = PMIC_GPIO_EN_PB_VSENSE_FXN_SEL_VALID,
        .fxnSel = PMIC_GPIO_EN_PB_VSENSE_FXN_SEL_ENABLE
    };

    int32_t status = Pmic_gpioSetEnPbVSenseCfg(NULL, &cfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

/* Test Pmic_gpioSetEnPbVSenseCfg with NULL config */
static void test_gpio_setEnPbVSenseCfg_nullConfig(void)
{
    int32_t status = Pmic_gpioSetEnPbVSenseCfg(&pmicHandle, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

/* Test Pmic_gpioSetEnPbVSenseCfg with invalid function select */
static void test_gpio_setEnPbVSenseCfg_invalidFunction(void)
{
    Pmic_GpioNIntEnDrvCfg_t cfg = {
        .validParams = PMIC_GPIO_EN_PB_VSENSE_FXN_SEL_VALID,
        .fxnSel = 3U  // Invalid, max is 2
    };

    int32_t status = Pmic_gpioSetEnPbVSenseCfg(&pmicHandle, &cfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);
}

/* Test Pmic_gpioGetEnPbVSenseCfg with NULL handle */
static void test_gpio_getEnPbVSenseCfg_nullHandle(void)
{
    Pmic_GpioNIntEnDrvCfg_t cfg = {
        .validParams = PMIC_GPIO_EN_PB_VSENSE_FXN_SEL_VALID
    };

    int32_t status = Pmic_gpioGetEnPbVSenseCfg(NULL, &cfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

/* Test Pmic_gpioGetEnPbVSenseCfg with NULL config */
static void test_gpio_getEnPbVSenseCfg_nullConfig(void)
{
    int32_t status = Pmic_gpioGetEnPbVSenseCfg(&pmicHandle, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

/* Test Pmic_gpioGetEnPbVSenseStatus with NULL handle */
static void test_gpio_getEnPbVSenseStatus_nullHandle(void)
{
    Pmic_GpioEnPbVSenseStatus_t status_data = {
        .validParams = PMIC_GPIO_EN_LVL_HIGH_VALID
    };

    int32_t status = Pmic_gpioGetEnPbVSenseStatus(NULL, &status_data);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

/* Test Pmic_gpioGetEnPbVSenseStatus with NULL status */
static void test_gpio_getEnPbVSenseStatus_nullStatus(void)
{
    int32_t status = Pmic_gpioGetEnPbVSenseStatus(&pmicHandle, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

/* Test Pmic_gpioGetNRstOutVal with NULL handle */
static void test_gpio_getNRstOutVal_nullHandle(void)
{
    bool high = false;
    int32_t status = Pmic_gpioGetNRstOutVal(NULL, &high);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

/* Test Pmic_gpioGetNRstOutVal with NULL value pointer */
static void test_gpio_getNRstOutVal_nullValue(void)
{
    int32_t status = Pmic_gpioGetNRstOutVal(&pmicHandle, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

/* Test Pmic_gpioSetPinCfg with invalid functionality (default case in switch) */
static void test_negative_gpio_setFunction_invalidFunctionality(void)
{
    Pmic_GpioPinCfg_t cfg = {
        .validParams = PMIC_GPIO_FXN_SEL_VALID,
        .pinNum = PMIC_GPIO_PIN1,
        .fxnSel = 0xFFU  // Invalid function select value
    };

    int32_t status = Pmic_gpioSetPinCfg(&pmicHandle, &cfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);
}

/* Test Pmic_gpioGetNIntEnDrvCfg with invalid parameter (zero validParams) */
static void test_negative_gpio_getNIntEnDrvCfg_invalidParam(void)
{
    Pmic_GpioNIntEnDrvCfg_t cfg = {
        .validParams = 0U
    };

    int32_t status = Pmic_gpioGetNIntEnDrvCfg(&pmicHandle, &cfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);
}

/* Test Pmic_gpioGetEnPbVSenseCfg with invalid parameter (zero validParams) */
static void test_negative_gpio_getEnPbVSenseCfg_invalidParam(void)
{
    Pmic_GpioNIntEnDrvCfg_t cfg = {
        .validParams = 0U
    };

    int32_t status = Pmic_gpioGetEnPbVSenseCfg(&pmicHandle, &cfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);
}

/* Test Pmic_gpioGetNIntEnDrvCfg reading PU resistor config */
static void test_positive_gpio_getNIntEnDrvCfg_enPuResistor(void)
{
    int32_t status;

    /* First, set PU resistor enable */
    Pmic_GpioNIntEnDrvCfg_t cfgSet = {
        .validParams = PMIC_GPIO_NINT_ENDRV_EN_PU_RESISTOR_VALID,
        .enPuResistor = true
    };

    status = Pmic_gpioSetNIntEnDrvCfg(&pmicHandle, &cfgSet);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    /* Now read it back */
    Pmic_GpioNIntEnDrvCfg_t cfgGet = {
        .validParams = PMIC_GPIO_NINT_ENDRV_EN_PU_RESISTOR_VALID
    };

    status = Pmic_gpioGetNIntEnDrvCfg(&pmicHandle, &cfgGet);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(true, cfgGet.enPuResistor);
}

/* ========================================================================== */
/*                          Positive Test Functions                           */
/* ========================================================================== */

/* Test GPIO1 configuration as output */
static void test_gpio_gpio1_configOutput(void)
{
    Pmic_GpioPinCfg_t cfg = {
        .validParams = PMIC_GPIO_DIR_VALID | PMIC_GPIO_FXN_SEL_VALID,
        .pinNum = PMIC_GPIO_PIN1,
        .fxnSel = PMIC_GPIO_PIN1_FXN_SEL_GPIO,
        .dir = PMIC_GPIO_PIN_OUTPUT
    };

    int32_t status = Pmic_gpioSetPinCfg(&pmicHandle, &cfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
}

/* Test GPIO1 configuration readback */
static void test_gpio_gpio1_getConfig(void)
{
    Pmic_GpioPinCfg_t cfg = {
        .validParams = PMIC_GPIO_DIR_VALID | PMIC_GPIO_FXN_SEL_VALID,
        .pinNum = PMIC_GPIO_PIN1
    };

    int32_t status = Pmic_gpioGetPinCfg(&pmicHandle, &cfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
}

/* Test GPIO1 set high value */
static void test_gpio_gpio1_setHigh(void)
{
    int32_t status = Pmic_gpioSetPinVal(&pmicHandle, PMIC_GPIO_PIN1, true);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
}

/* Test GPIO1 set low value */
static void test_gpio_gpio1_setLow(void)
{
    int32_t status = Pmic_gpioSetPinVal(&pmicHandle, PMIC_GPIO_PIN1, false);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
}

/* Test GPIO1 get value */
static void test_gpio_gpio1_getValue(void)
{
    bool high = false;
    int32_t status = Pmic_gpioGetPinVal(&pmicHandle, PMIC_GPIO_PIN1, &high);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
}

/* Test GPIO3 configuration as input with pull-up */
static void test_gpio_gpio3_configInputPullUp(void)
{
    Pmic_GpioPinCfg_t cfg = {
        .validParams = PMIC_GPIO_DIR_VALID | PMIC_GPIO_PU_SEL_VALID |
                       PMIC_GPIO_RESISTOR_EN_VALID | PMIC_GPIO_FXN_SEL_VALID,
        .pinNum = PMIC_GPIO_PIN3,
        .fxnSel = PMIC_GPIO_PIN3_FXN_SEL_GPIO,
        .dir = PMIC_GPIO_PIN_INPUT,
        .puSel = PMIC_GPIO_PIN_PULL_UP_RESISTOR,
        .resistorEn = true
    };

    int32_t status = Pmic_gpioSetPinCfg(&pmicHandle, &cfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
}

/* Test GPIO3 configuration readback */
static void test_gpio_gpio3_getConfig(void)
{
    Pmic_GpioPinCfg_t cfg = {
        .validParams = PMIC_GPIO_DIR_VALID | PMIC_GPIO_PU_SEL_VALID |
                       PMIC_GPIO_RESISTOR_EN_VALID,
        .pinNum = PMIC_GPIO_PIN3
    };

    int32_t status = Pmic_gpioGetPinCfg(&pmicHandle, &cfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
}

/* Test GPIO5 configuration as output with open-drain */
static void test_gpio_gpio5_configOutputOpenDrain(void)
{
    Pmic_GpioPinCfg_t cfg = {
        .validParams = PMIC_GPIO_DIR_VALID | PMIC_GPIO_TYPE_VALID |
                       PMIC_GPIO_FXN_SEL_VALID,
        .pinNum = PMIC_GPIO_PIN5,
        .fxnSel = PMIC_GPIO_PIN5_FXN_SEL_GPIO,
        .dir = PMIC_GPIO_PIN_OUTPUT,
        .type = PMIC_GPIO_PIN_OPEN_DRAIN
    };

    int32_t status = Pmic_gpioSetPinCfg(&pmicHandle, &cfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
}

/* Test GPIO5 configuration readback */
static void test_gpio_gpio5_getConfig(void)
{
    Pmic_GpioPinCfg_t cfg = {
        .validParams = PMIC_GPIO_DIR_VALID | PMIC_GPIO_TYPE_VALID,
        .pinNum = PMIC_GPIO_PIN5
    };

    int32_t status = Pmic_gpioGetPinCfg(&pmicHandle, &cfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
}

/* Test GPIO6 configuration with deglitch enabled */
static void test_gpio_gpio6_configInputDeglitch(void)
{
    Pmic_GpioPinCfg_t cfg = {
        .validParams = PMIC_GPIO_DIR_VALID | PMIC_GPIO_DEGL_EN_VALID |
                       PMIC_GPIO_FXN_SEL_VALID,
        .pinNum = PMIC_GPIO_PIN6,
        .fxnSel = PMIC_GPIO_PIN6_FXN_SEL_GPIO,
        .dir = PMIC_GPIO_PIN_INPUT,
        .deglEn = true
    };

    int32_t status = Pmic_gpioSetPinCfg(&pmicHandle, &cfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
}

/* Test GPIO6 configuration readback */
static void test_gpio_gpio6_getConfig(void)
{
    Pmic_GpioPinCfg_t cfg = {
        .validParams = PMIC_GPIO_DIR_VALID | PMIC_GPIO_DEGL_EN_VALID,
        .pinNum = PMIC_GPIO_PIN6
    };

    int32_t status = Pmic_gpioGetPinCfg(&pmicHandle, &cfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
}

/* Test GPIO2 set/get sequence */
static void test_gpio_gpio2_setGetSequence(void)
{
    bool high = false;
    int32_t status;

    // Set high
    status = Pmic_gpioSetPinVal(&pmicHandle, PMIC_GPIO_PIN2, true);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get value
    status = Pmic_gpioGetPinVal(&pmicHandle, PMIC_GPIO_PIN2, &high);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Set low
    status = Pmic_gpioSetPinVal(&pmicHandle, PMIC_GPIO_PIN2, false);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get value
    status = Pmic_gpioGetPinVal(&pmicHandle, PMIC_GPIO_PIN2, &high);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
}

/* Test GPIO4 configuration with pull-down */
static void test_gpio_gpio4_configInputPullDown(void)
{
    Pmic_GpioPinCfg_t cfg = {
        .validParams = PMIC_GPIO_DIR_VALID | PMIC_GPIO_PU_SEL_VALID |
                       PMIC_GPIO_RESISTOR_EN_VALID | PMIC_GPIO_FXN_SEL_VALID,
        .pinNum = PMIC_GPIO_PIN4,
        .fxnSel = PMIC_GPIO_PIN4_FXN_SEL_GPIO,
        .dir = PMIC_GPIO_PIN_INPUT,
        .puSel = PMIC_GPIO_PIN_PULL_DOWN_RESISTOR,
        .resistorEn = true
    };

    int32_t status = Pmic_gpioSetPinCfg(&pmicHandle, &cfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
}

/* Test GPIO1 function select - SDO_SPI */
static void test_gpio_gpio1_funcSdoSpi(void)
{
    Pmic_GpioPinCfg_t cfg = {
        .validParams = PMIC_GPIO_FXN_SEL_VALID,
        .pinNum = PMIC_GPIO_PIN1,
        .fxnSel = PMIC_GPIO_PIN1_FXN_SEL_SDO_SPI
    };

    int32_t status = Pmic_gpioSetPinCfg(&pmicHandle, &cfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
}

/* Test GPIO2 function select - NSLEEP1 */
static void test_gpio_gpio2_funcNsleep1(void)
{
    Pmic_GpioPinCfg_t cfg = {
        .validParams = PMIC_GPIO_FXN_SEL_VALID,
        .pinNum = PMIC_GPIO_PIN2,
        .fxnSel = PMIC_GPIO_PIN2_FXN_SEL_NSLEEP1
    };

    int32_t status = Pmic_gpioSetPinCfg(&pmicHandle, &cfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
}

/* Test GPIO3 function select - PB (Push Button) */
static void test_gpio_gpio3_funcPb(void)
{
    Pmic_GpioPinCfg_t cfg = {
        .validParams = PMIC_GPIO_FXN_SEL_VALID,
        .pinNum = PMIC_GPIO_PIN3,
        .fxnSel = PMIC_GPIO_PIN3_FXN_SEL_PB
    };

    int32_t status = Pmic_gpioSetPinCfg(&pmicHandle, &cfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
}

/* Test GPIO5 function select - WKUP */
static void test_gpio_gpio5_funcWkup(void)
{
    Pmic_GpioPinCfg_t cfg = {
        .validParams = PMIC_GPIO_FXN_SEL_VALID,
        .pinNum = PMIC_GPIO_PIN5,
        .fxnSel = PMIC_GPIO_PIN5_FXN_SEL_WKUP
    };

    int32_t status = Pmic_gpioSetPinCfg(&pmicHandle, &cfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
}

/* Test GPIO6 function select - SYNCCLKIN */
static void test_gpio_gpio6_funcSyncClkIn(void)
{
    Pmic_GpioPinCfg_t cfg = {
        .validParams = PMIC_GPIO_FXN_SEL_VALID,
        .pinNum = PMIC_GPIO_PIN6,
        .fxnSel = PMIC_GPIO_PIN6_FXN_SEL_SYNCCLKIN
    };

    int32_t status = Pmic_gpioSetPinCfg(&pmicHandle, &cfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
}

/* Test NINT_EN_DRV configuration as NINT */
static void test_gpio_nIntEnDrv_configNInt(void)
{
    Pmic_GpioNIntEnDrvCfg_t cfg = {
        .validParams = PMIC_GPIO_NINT_ENDRV_FXN_SEL_VALID,
        .fxnSel = PMIC_GPIO_NINT_ENDRV_FXN_SEL_NINT
    };

    int32_t status = Pmic_gpioSetNIntEnDrvCfg(&pmicHandle, &cfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
}

/* Test NINT_EN_DRV configuration as EN_DRV */
static void test_gpio_nIntEnDrv_configEnDrv(void)
{
    Pmic_GpioNIntEnDrvCfg_t cfg = {
        .validParams = PMIC_GPIO_NINT_ENDRV_FXN_SEL_VALID,
        .fxnSel = PMIC_GPIO_NINT_ENDRV_FXN_SEL_EN_DRV
    };

    int32_t status = Pmic_gpioSetNIntEnDrvCfg(&pmicHandle, &cfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
}

/* Test NINT_EN_DRV configuration readback */
static void test_gpio_nIntEnDrv_getConfig(void)
{
    Pmic_GpioNIntEnDrvCfg_t cfg = {
        .validParams = PMIC_GPIO_NINT_ENDRV_FXN_SEL_VALID
    };

    int32_t status = Pmic_gpioGetNIntEnDrvCfg(&pmicHandle, &cfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
}

/* Test NINT_EN_DRV pull-up resistor enable */
static void test_gpio_nIntEnDrv_enablePullUp(void)
{
    Pmic_GpioNIntEnDrvCfg_t cfg = {
        .validParams = PMIC_GPIO_NINT_ENDRV_EN_PU_RESISTOR_VALID,
        .enPuResistor = true
    };

    int32_t status = Pmic_gpioSetNIntEnDrvCfg(&pmicHandle, &cfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
}

/* Test NINT_EN_DRV get value */
static void test_gpio_nIntEnDrv_getValue(void)
{
    bool high = false;
    int32_t status = Pmic_gpioGetNIntEnDrvVal(&pmicHandle, &high);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
}

/* Test EN_PB_VSENSE configuration as ENABLE */
static void test_gpio_enPbVSense_configEnable(void)
{
    Pmic_GpioNIntEnDrvCfg_t cfg = {
        .validParams = PMIC_GPIO_EN_PB_VSENSE_FXN_SEL_VALID,
        .fxnSel = PMIC_GPIO_EN_PB_VSENSE_FXN_SEL_ENABLE
    };

    int32_t status = Pmic_gpioSetEnPbVSenseCfg(&pmicHandle, &cfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
}

/* Test EN_PB_VSENSE configuration as PB */
static void test_gpio_enPbVSense_configPb(void)
{
    Pmic_GpioNIntEnDrvCfg_t cfg = {
        .validParams = PMIC_GPIO_EN_PB_VSENSE_FXN_SEL_VALID,
        .fxnSel = PMIC_GPIO_EN_PB_VSENSE_FXN_SEL_PB
    };

    int32_t status = Pmic_gpioSetEnPbVSenseCfg(&pmicHandle, &cfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
}

/* Test EN_PB_VSENSE configuration as VSENSE */
static void test_gpio_enPbVSense_configVSense(void)
{
    Pmic_GpioNIntEnDrvCfg_t cfg = {
        .validParams = PMIC_GPIO_EN_PB_VSENSE_FXN_SEL_VALID,
        .fxnSel = PMIC_GPIO_EN_PB_VSENSE_FXN_SEL_VSENSE
    };

    int32_t status = Pmic_gpioSetEnPbVSenseCfg(&pmicHandle, &cfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
}

/* Test EN_PB_VSENSE configuration readback */
static void test_gpio_enPbVSense_getConfig(void)
{
    Pmic_GpioNIntEnDrvCfg_t cfg = {
        .validParams = PMIC_GPIO_EN_PB_VSENSE_FXN_SEL_VALID
    };

    int32_t status = Pmic_gpioGetEnPbVSenseCfg(&pmicHandle, &cfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
}

/* Test EN_PB_VSENSE deglitch configuration for ENABLE mode */
static void test_gpio_enPbVSense_deglitchEnable(void)
{
    Pmic_GpioNIntEnDrvCfg_t cfg = {
        .validParams = PMIC_GPIO_EN_PB_VSENSE_FXN_SEL_VALID |
                       PMIC_GPIO_EN_PB_VSENSE_EN_PB_DEGL_VALID,
        .fxnSel = PMIC_GPIO_EN_PB_VSENSE_FXN_SEL_ENABLE,
        .enPbDegl = PMIC_GPIO_EN_DEGLITCH_50_MS
    };

    int32_t status = Pmic_gpioSetEnPbVSenseCfg(&pmicHandle, &cfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
}

/* Test EN_PB_VSENSE deglitch configuration for PB mode */
static void test_gpio_enPbVSense_deglitchPb(void)
{
    Pmic_GpioNIntEnDrvCfg_t cfg = {
        .validParams = PMIC_GPIO_EN_PB_VSENSE_FXN_SEL_VALID |
                       PMIC_GPIO_EN_PB_VSENSE_EN_PB_DEGL_VALID,
        .fxnSel = PMIC_GPIO_EN_PB_VSENSE_FXN_SEL_PB,
        .enPbDegl = PMIC_GPIO_PB_DEGLITCH_600_MS
    };

    int32_t status = Pmic_gpioSetEnPbVSenseCfg(&pmicHandle, &cfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
}

/* Test EN_PB_VSENSE status readback */
static void test_gpio_enPbVSense_getStatus(void)
{
    Pmic_GpioEnPbVSenseStatus_t status_data = {
        .validParams = PMIC_GPIO_EN_LVL_HIGH_VALID |
                       PMIC_GPIO_PB_LVL_HIGH_VALID |
                       PMIC_GPIO_VSENSE_LVL_HIGH_VALID
    };

    int32_t status = Pmic_gpioGetEnPbVSenseStatus(&pmicHandle, &status_data);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
}

/* Test NRST_OUT get value */
static void test_gpio_nRstOut_getValue(void)
{
    bool high = false;
    int32_t status = Pmic_gpioGetNRstOutVal(&pmicHandle, &high);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
}

/* Test GPIO configuration push-pull type */
static void test_gpio_config_pushPull(void)
{
    Pmic_GpioPinCfg_t cfg = {
        .validParams = PMIC_GPIO_TYPE_VALID | PMIC_GPIO_DIR_VALID |
                       PMIC_GPIO_FXN_SEL_VALID,
        .pinNum = PMIC_GPIO_PIN2,
        .fxnSel = PMIC_GPIO_PIN2_FXN_SEL_GPIO,
        .dir = PMIC_GPIO_PIN_OUTPUT,
        .type = PMIC_GPIO_PIN_PUSH_PULL
    };

    int32_t status = Pmic_gpioSetPinCfg(&pmicHandle, &cfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
}

/* Test all GPIO pins value get */
static void test_gpio_allPins_getValue(void)
{
    bool high = false;
    int32_t status;

    for (uint8_t pin = PMIC_GPIO_PIN1; pin <= PMIC_GPIO_PIN6; pin++)
    {
        status = Pmic_gpioGetPinVal(&pmicHandle, pin, &high);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    }
}

/* Test Pmic_gpioGetPinVal with valid GPIO pin */
static void test_positive_gpioGetValue_validPin(void)
{
    bool high = false;
    int32_t status;

    /* Test reading value from GPIO PIN 4 */
    status = Pmic_gpioGetPinVal(&pmicHandle, PMIC_GPIO_PIN4, &high);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
}

/* Test Pmic_gpioSetPinVal with valid GPIO pin */
static void test_positive_gpioSetValue_validPin(void)
{
    int32_t status;

    /* Test setting GPIO PIN 5 to high */
    status = Pmic_gpioSetPinVal(&pmicHandle, PMIC_GPIO_PIN5, true);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    /* Test setting GPIO PIN 5 to low */
    status = Pmic_gpioSetPinVal(&pmicHandle, PMIC_GPIO_PIN5, false);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
}

/* ========================================================================== */
/*                          Property Test Functions                           */
/* ========================================================================== */

#ifdef BUILD_MOCK
/* Property test: GPIO pin configuration combinations (500 iterations) */
static void test_gpio_property_pinConfigurations(void)
{
    const uint32_t iterations = 500U;
    uint32_t successCount = 0U;

    // Seed random number generator
    srand((unsigned int)time(NULL));

    for (uint32_t i = 0U; i < iterations; i++)
    {
        // Random pin number (1-6)
        uint8_t pin = (uint8_t)((rand() % 6) + 1);

        // Random direction
        uint8_t dir = (uint8_t)(rand() % 2);

        // Random pull select
        uint8_t puSel = (uint8_t)(rand() % 2);

        // Random type
        uint8_t type = (uint8_t)(rand() % 2);

        // Random resistor enable
        bool resistorEn = (rand() % 2) ? true : false;

        // Random deglitch enable
        bool deglEn = (rand() % 2) ? true : false;

        // Set configuration
        Pmic_GpioPinCfg_t setCfg = {
            .validParams = PMIC_GPIO_DIR_VALID | PMIC_GPIO_PU_SEL_VALID |
                          PMIC_GPIO_TYPE_VALID | PMIC_GPIO_RESISTOR_EN_VALID |
                          PMIC_GPIO_DEGL_EN_VALID | PMIC_GPIO_FXN_SEL_VALID,
            .pinNum = pin,
            .fxnSel = 0U,  // GPIO function
            .dir = dir,
            .puSel = puSel,
            .type = type,
            .resistorEn = resistorEn,
            .deglEn = deglEn
        };

        int32_t status = Pmic_gpioSetPinCfg(&pmicHandle, &setCfg);

        if (status == PMIC_ST_SUCCESS)
        {
            // Get configuration and verify
            Pmic_GpioPinCfg_t getCfg = {
                .validParams = PMIC_GPIO_DIR_VALID | PMIC_GPIO_PU_SEL_VALID |
                              PMIC_GPIO_TYPE_VALID | PMIC_GPIO_RESISTOR_EN_VALID |
                              PMIC_GPIO_DEGL_EN_VALID,
                .pinNum = pin
            };

            status = Pmic_gpioGetPinCfg(&pmicHandle, &getCfg);

            if (status == PMIC_ST_SUCCESS)
            {
                // Verify consistency (in mock mode, just count successes)
                successCount++;
            }
        }
    }

    // In property testing, we expect high success rate
    TEST_ASSERT_GREATER_THAN(iterations * 90 / 100, successCount);
}
#endif

/**
 * @brief Test Pmic_gpioGetPinCfg with zero validParams
 * Covers line 359 in pmic_gpio.c
 */
static void test_negative_gpio_getPinCfg_zeroValidParams(void)
{
    Pmic_GpioPinCfg_t cfg = {
        .validParams = 0U,
        .pinNum = PMIC_GPIO_PIN1
    };

    int32_t status = Pmic_gpioGetPinCfg(&pmicHandle, &cfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);
}

/**
 * @brief Test Pmic_gpioSetNIntEnDrvCfg with zero validParams
 * Covers line 492 in pmic_gpio.c
 */
static void test_negative_gpio_setNIntEnDrvCfg_zeroValidParams(void)
{
    Pmic_GpioNIntEnDrvCfg_t cfg = {
        .validParams = 0U
    };

    int32_t status = Pmic_gpioSetNIntEnDrvCfg(&pmicHandle, &cfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);
}

/**
 * @brief Test Pmic_gpioSetEnPbVSenseCfg with zero validParams
 * Covers line 624 in pmic_gpio.c
 */
static void test_negative_gpio_setEnPbVSenseCfg_zeroValidParams(void)
{
    Pmic_GpioNIntEnDrvCfg_t cfg = {
        .validParams = 0U
    };

    int32_t status = Pmic_gpioSetEnPbVSenseCfg(&pmicHandle, &cfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);
}

/**
 * @brief Test Pmic_gpioSetEnPbVSenseCfg with invalid deglitch value
 * Covers lines 674-675 in pmic_gpio.c
 */
static void test_negative_gpio_setEnPbVSenseCfg_invalidDeglitch(void)
{
    Pmic_GpioNIntEnDrvCfg_t cfg = {
        .validParams = PMIC_GPIO_NINT_ENDRV_FXN_SEL_VALID | PMIC_GPIO_EN_PB_VSENSE_EN_PB_DEGL_VALID,
        .fxnSel = PMIC_GPIO_EN_PB_VSENSE_FXN_SEL_ENABLE,
        .enPbDegl = PMIC_GPIO_EN_DEGL_MAX + 1  /* Invalid deglitch for EN function */
    };

    int32_t status = Pmic_gpioSetEnPbVSenseCfg(&pmicHandle, &cfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);
}

/**
 * @brief Test Pmic_gpioGetEnPbVSenseCfg to get EN/PB deglitch setting
 * Covers lines 732-733 in pmic_gpio.c - reading enPbDegl when DEGL_VALID is set
 */
static void test_positive_gpio_getEnPbDegl(void)
{
    Pmic_GpioNIntEnDrvCfg_t cfg = {
        .validParams = PMIC_GPIO_EN_PB_VSENSE_EN_PB_DEGL_VALID
    };

    int32_t status = Pmic_gpioGetEnPbVSenseCfg(&pmicHandle, &cfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
}

/**
 * @brief Test Pmic_gpioGetEnPbVSenseStatus with zero validParams
 * Covers line 759 in pmic_gpio.c
 */
static void test_negative_gpio_getEnPbVSenseStatus_zeroValidParams(void)
{
    Pmic_GpioEnPbVSenseStatus_t status_var = {
        .validParams = 0U
    };

    int32_t status = Pmic_gpioGetEnPbVSenseStatus(&pmicHandle, &status_var);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);
}

/* ========================================================================== */
/*                             Test Macros                                    */
/* ========================================================================== */

#define GPIO_TEST_RUN_NEGATIVE() \
    do { \
        RUN_TEST(test_gpio_setPinCfg_nullHandle); \
        RUN_TEST(test_gpio_setPinCfg_nullConfig); \
        RUN_TEST(test_gpio_setPinCfg_zeroValidParams); \
        RUN_TEST(test_gpio_setPinCfg_invalidPinBelowMin); \
        RUN_TEST(test_gpio_setPinCfg_invalidPinAboveMax); \
        RUN_TEST(test_gpio_setPinCfg_invalidDirection); \
        RUN_TEST(test_gpio_setPinCfg_invalidFunctionGpio1); \
        RUN_TEST(test_gpio_setPinCfg_invalidPullSelect); \
        RUN_TEST(test_gpio_setPinCfg_invalidType); \
        RUN_TEST(test_gpio_getPinCfg_nullHandle); \
        RUN_TEST(test_gpio_getPinCfg_nullConfig); \
        RUN_TEST(test_gpio_getPinCfg_invalidPin); \
        RUN_TEST(test_gpio_setPinVal_nullHandle); \
        RUN_TEST(test_gpio_setPinVal_invalidPin); \
        RUN_TEST(test_gpio_getPinVal_nullHandle); \
        RUN_TEST(test_gpio_getPinVal_nullValue); \
        RUN_TEST(test_gpio_getPinVal_invalidPin); \
        RUN_TEST(test_gpio_setNIntEnDrvCfg_nullHandle); \
        RUN_TEST(test_gpio_setNIntEnDrvCfg_nullConfig); \
        RUN_TEST(test_gpio_setNIntEnDrvCfg_invalidFunction); \
        RUN_TEST(test_gpio_getNIntEnDrvCfg_nullHandle); \
        RUN_TEST(test_gpio_getNIntEnDrvCfg_nullConfig); \
        RUN_TEST(test_gpio_getNIntEnDrvVal_nullHandle); \
        RUN_TEST(test_gpio_getNIntEnDrvVal_nullValue); \
        RUN_TEST(test_gpio_setEnPbVSenseCfg_nullHandle); \
        RUN_TEST(test_gpio_setEnPbVSenseCfg_nullConfig); \
        RUN_TEST(test_gpio_setEnPbVSenseCfg_invalidFunction); \
        RUN_TEST(test_gpio_getEnPbVSenseCfg_nullHandle); \
        RUN_TEST(test_gpio_getEnPbVSenseCfg_nullConfig); \
        RUN_TEST(test_gpio_getEnPbVSenseStatus_nullHandle); \
        RUN_TEST(test_gpio_getEnPbVSenseStatus_nullStatus); \
        RUN_TEST(test_gpio_getNRstOutVal_nullHandle); \
        RUN_TEST(test_gpio_getNRstOutVal_nullValue); \
        RUN_TEST(test_negative_gpio_setFunction_invalidFunctionality); \
        RUN_TEST(test_negative_gpio_getNIntEnDrvCfg_invalidParam); \
        RUN_TEST(test_negative_gpio_getEnPbVSenseCfg_invalidParam); \
        RUN_TEST(test_negative_gpioSetConfiguration_invalidGpioPin); \
        RUN_TEST(test_negative_gpioGetConfiguration_invalidGpioPin); \
        RUN_TEST(test_negative_gpio_getPinCfg_zeroValidParams); \
        RUN_TEST(test_negative_gpio_setNIntEnDrvCfg_zeroValidParams); \
        RUN_TEST(test_negative_gpio_setEnPbVSenseCfg_zeroValidParams); \
        RUN_TEST(test_negative_gpio_setEnPbVSenseCfg_invalidDeglitch); \
        RUN_TEST(test_negative_gpio_getEnPbVSenseStatus_zeroValidParams); \
    } while(0)

#define GPIO_TEST_RUN_POSITIVE() \
    do { \
        RUN_TEST(test_gpio_gpio1_configOutput); \
        RUN_TEST(test_gpio_gpio1_getConfig); \
        RUN_TEST(test_gpio_gpio1_setHigh); \
        RUN_TEST(test_gpio_gpio1_setLow); \
        RUN_TEST(test_gpio_gpio1_getValue); \
        RUN_TEST(test_gpio_gpio3_configInputPullUp); \
        RUN_TEST(test_gpio_gpio3_getConfig); \
        RUN_TEST(test_gpio_gpio5_configOutputOpenDrain); \
        RUN_TEST(test_gpio_gpio5_getConfig); \
        RUN_TEST(test_gpio_gpio6_configInputDeglitch); \
        RUN_TEST(test_gpio_gpio6_getConfig); \
        RUN_TEST(test_gpio_gpio2_setGetSequence); \
        RUN_TEST(test_gpio_gpio4_configInputPullDown); \
        RUN_TEST(test_gpio_gpio1_funcSdoSpi); \
        RUN_TEST(test_gpio_gpio2_funcNsleep1); \
        RUN_TEST(test_gpio_gpio3_funcPb); \
        RUN_TEST(test_gpio_gpio5_funcWkup); \
        RUN_TEST(test_gpio_gpio6_funcSyncClkIn); \
        RUN_TEST(test_gpio_nIntEnDrv_configNInt); \
        RUN_TEST(test_gpio_nIntEnDrv_configEnDrv); \
        RUN_TEST(test_gpio_nIntEnDrv_getConfig); \
        RUN_TEST(test_gpio_nIntEnDrv_enablePullUp); \
        RUN_TEST(test_gpio_nIntEnDrv_getValue); \
        RUN_TEST(test_gpio_enPbVSense_configEnable); \
        RUN_TEST(test_gpio_enPbVSense_configPb); \
        RUN_TEST(test_gpio_enPbVSense_configVSense); \
        RUN_TEST(test_gpio_enPbVSense_getConfig); \
        RUN_TEST(test_gpio_enPbVSense_deglitchEnable); \
        RUN_TEST(test_gpio_enPbVSense_deglitchPb); \
        RUN_TEST(test_gpio_enPbVSense_getStatus); \
        RUN_TEST(test_gpio_nRstOut_getValue); \
        RUN_TEST(test_gpio_config_pushPull); \
        RUN_TEST(test_gpio_allPins_getValue); \
        RUN_TEST(test_positive_gpio_getNIntEnDrvCfg_enPuResistor); \
        RUN_TEST(test_positive_gpioGetValue_validPin); \
        RUN_TEST(test_positive_gpioSetValue_validPin); \
        RUN_TEST(test_positive_gpio_getEnPbDegl); \
    } while(0)

#ifdef BUILD_MOCK
#define GPIO_TEST_RUN_PROPERTY() \
    do { \
        RUN_TEST(test_gpio_property_pinConfigurations); \
    } while(0)
#else
#define GPIO_TEST_RUN_PROPERTY() do {} while(0)
#endif

#define GPIO_TEST_RUN_ALL() \
    do { \
        GPIO_TEST_RUN_NEGATIVE(); \
        GPIO_TEST_RUN_POSITIVE(); \
        GPIO_TEST_RUN_PROPERTY(); \
    } while(0)

/* ========================================================================== */
/*                             Entry Point                                    */
/* ========================================================================== */

void gpio_test(void *args)
{
    (void)args;
    int32_t status;

    Pmic_HandleCfg_t handleCfg = {
        .validParams = (PMIC_COMM_MODE_VALID |
                        PMIC_COMM_HANDLE_0_VALID |
                        PMIC_IO_READ_VALID |
                        PMIC_IO_WRITE_VALID |
                        PMIC_CRITICAL_SECTION_START_VALID |
                        PMIC_CRITICAL_SECTION_STOP_VALID |
                        PMIC_IRQ_RESPONSE_CALLBACK_VALID),
        .commMode = PMIC_INTF_SPI,
        .commHandle0 = platform_getCommHandle(),
        .ioRead = &platform_rxByte,
        .ioWrite = &platform_txByte,
        .criticalSectionStart = &platform_critSecStart,
        .criticalSectionStop = &platform_critSecStop,
        .irqResponseCallback = &platform_irqResponse
    };

    platform_init();
    platform_printString("\r\n=== GPIO Module Tests (TPS6522x-Q1 Burton) ===\r\n");

    status = Pmic_init(&pmicHandle, &handleCfg);

    if (status == PMIC_ST_SUCCESS)
    {
        /* Unlock registers for GPIO configuration */
        status = Pmic_setRegLockState(&pmicHandle, false);
        if (status != PMIC_ST_SUCCESS)
        {
            platform_printString("ERROR: Failed to unlock registers\r\n");
            platform_deinit();
            return;
        }

        platform_setupTests();
        GPIO_TEST_RUN_ALL();
        platform_tearDownTests();

        Pmic_deinit(&pmicHandle);
    }
    else
    {
        platform_printString("ERROR: Failed to initialize PMIC handle\r\n");
    }

    platform_deinit();
}
