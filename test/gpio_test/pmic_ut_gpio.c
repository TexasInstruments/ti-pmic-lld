/******************************************************************************
 * Copyright (c) 2020 Texas Instruments Incorporated - http://www.ti.com
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
 *  \file   pmic_ut_gpio.c
 *
 *  \brief  PMIC Unit Test for testing PMIC GPIO APIs
 *
 */

#include <pmic_ut_gpio.h>

/* Pointer holds the pPmicCoreHandle */
Pmic_CoreHandle_t *pPmicCoreHandle = NULL;

static uint8_t pmic_device_info = 0U;

/*!
 * \brief   PMIC GPIO Test Cases
 */
static Pmic_Ut_Tests_t pmic_gpio_tests[] =
{
    /*! testID
     *  testDesc
     */
    {
        6185,
        "Pmic_gpioSetConfiguration : configure a gpio pin as NSLEEP1 function"
    },
    {
        6186,
        "Pmic_gpioSetConfiguration : configure a gpio pin as NSLEEP2 function"
    },
    {
        6187,
        "Pmic_gpioSetConfiguration : configure gpio pin as NRSTOUT_SOC function"
    },
    {
        6189,
        "Pmic_gpioSetConfiguration : configure a gpio pin as WAKEUP1 function"
    },
    {
        6190,
        "Pmic_gpioSetConfiguration : configure a gpio pin as WAKEUP2 function"
    },
    {
        6191,
        "Pmic_gpioSetConfiguration : configure a gpio pin as general purpose input/output function"
    },
    {
        6192,
        "Pmic_gpioSetConfiguration : configure gpio pin as I2C2 SCLK function"
    },
    {
        6193,
        "Pmic_gpioSetConfiguration : configure gpio pin as I2C2 SDA function"
    },
    {
        6195,
        "Pmic_gpioSetConfiguration : configure gpio pin as SPI CS function"
    },
    {
        6196,
        "Pmic_gpioSetConfiguration : configure gpio pin as SPI SDO function"
    },
    {
        6197,
        "Pmic_gpioSetConfiguration : configure gpio pin as watchdog trigger function"
    },
    {
        6198,
        "Pmic_gpioSetConfiguration : configure gpio pin 3 as ESM Error Pins for SOC"
    },
    {
        6199,
        "Pmic_gpioSetConfiguration : configure gpio pin as ESM Error Pins for MCU"
    },
    {
        6201,
        "Pmic_gpioSetConfiguration : configure gpio pin as SPMI SDATA function"
    },
    {
        6202,
        "Pmic_gpioSetConfiguration : configure gpio pin as SYNCCLKOUT function"
    },
    {
        6204,
        "Pmic_gpioSetConfiguration : configure gpio pin 10 as SYNCLKIN function"
    },
    {
        6205,
        "Pmic_gpioSetConfiguration : configure gpio pin as CLK32KOUT function"
    },
    {
      6206,
      "Pmic_gpioSetConfiguration : configure gpio pin 10 as CLK32KOUT function"
    },
    {
        6207,
        "Pmic_gpioSetConfiguration : configure gpio pin as Watchdog disable function"
    },
    {
        6208,
        "Pmic_gpioSetConfiguration : configure gpio pin as Watchdog disable function"
    },
    {
        6209,
        "Pmic_gpioSetConfiguration : configure gpio pin as Power Good Indication line function"
    },
    {
        6210,
        "Pmic_gpioSetConfiguration : Parameter validation for handle"
    },
    {
        6211,
        "Pmic_gpioSetConfiguration : Parameter validation for pin "
    },
    {
        6213,
        "Pmic_gpioSetConfiguration : Gpio pin configuration validation for pinDir "
    },
    {
        6214,
        "Pmic_gpioSetConfiguration : Gpio pin configuration validation for outputSignalType"
    },
    {
        6215,
        "Pmic_gpioSetConfiguration : Gpio pin configuration validation for deglitchEnable "
    },
    {
        6216,
        "Pmic_gpioSetConfiguration : Gpio pin configuration validation for pinFunc"
    },
    {
        6217,
        "Pmic_gpioSetNPwronEnablePinConfiguration : nPWRON pin configuration validation for pinFunc "
    },
    {
        6218,
        "Pmic_gpioSetNPwronEnablePinConfiguration : nPWRON pin configuration validation for pinPolarity"
    },
    {
        6219,
        "Pmic_gpioGetConfiguration : Get required gpio pin configuration"
    },
    {
        6220,
        "Pmic_gpioGetConfiguration : Parameter validation for handle"
    },
    {
        6221,
        "Pmic_gpioGetConfiguration : Parameter validation for pin"
    },
    {
        6222,
        "Pmic_gpioGetConfiguration : Parameter validation for GpioCfg"
    },
    {
        6223,
        "Pmic_gpioGetValue : Get GPIO signal level "
    },
    {
        6224,
        "Pmic_gpioGetValue : Parameter validation for handle"
    },
    {
        6225,
        "Pmic_gpioGetValue : Parameter validation for pin"
    },
    {
        6226,
        "Pmic_gpioGetValue : Parameter validation for pinValue "
    },
    {
        6227,
        "Pmic_gpioSetValue : Set GPIO signal level "
    },
    {
        6228,
        "Pmic_gpioSetValue : Parameter validation for handle"
    },
    {
        6229,
        "Pmic_gpioSetValue : Parameter validation for pin "
    },
    {
        6230,
        "Pmic_gpioSetValue : Parameter validation for pinValue "
    },
    {
        6231,
        "Pmic_gpioSetValue : Set GPIO signal level for an input GPIO pin"
    },
    {
        6234,
        "Pmic_gpioSetIntr : GPIO1 Fall Interrupt Test"
    },
    {
        6235,
        "Pmic_gpioSetIntr : GPIO1 Rise Interrupt Test"
    },
    {
        6236,
        "Pmic_gpioSetIntr : GPIO2 Fall Interrupt Test"
    },
    {
        6237,
        "Pmic_gpioSetIntr : GPIO2 Rise Interrupt Test"
    },
    {
        6238,
        "Pmic_gpioSetIntr : GPIO3 Fall Interrupt Test"
    },
    {
        6239,
        "Pmic_gpioSetIntr : GPIO3 Rise Interrupt Test"
    },
    {
        6240,
        "Pmic_gpioSetIntr : GPIO4 Fall Interrupt Test"
    },
    {
        6241,
        "Pmic_gpioSetIntr : GPIO4 Rise Interrupt Test"
    },
    {
        6242,
        "Pmic_gpioSetIntr : GPIO5 Fall Interrupt Test"
    },
    {
        6243,
        "Pmic_gpioSetIntr : GPIO5 Rise Interrupt Test"
    },
    {
        6244,
        "Pmic_gpioSetIntr : GPIO6 Fall Interrupt Test"
    },
    {
        6245,
        "Pmic_gpioSetIntr : GPIO6 Rise Interrupt Test"
    },
    {
        6246,
        "Pmic_gpioSetIntr : GPIO7 Fall Interrupt Test"
    },
    {
        6247,
        "Pmic_gpioSetIntr : GPIO7 Rise Interrupt Test"
    },
    {
        6248,
        "Pmic_gpioSetIntr : GPIO8 Fall Interrupt Test"
    },
    {
        6249,
        "Pmic_gpioSetIntr : GPIO8 Rise Interrupt Test"
    },
    {
        6250,
        "Pmic_gpioSetIntr : GPIO9 Fall Interrupt Test"
    },
    {
        6251,
        "Pmic_gpioSetIntr : GPIO9 Rise Interrupt Test"
    },
    {
        6252,
        "Pmic_gpioSetIntr : GPIO10 Fall Interrupt Test"
    },
    {
        6253,
        "Pmic_gpioSetIntr : GPIO10 Rise Interrupt Test"
    },
    {
        6254,
        "Pmic_gpioSetIntr : GPIO11 Fall Interrupt Test"
    },
    {
        6255,
        "Pmic_gpioSetIntr : GPIO11 Rise Interrupt Test"
    },
    {
        6256,
        "Pmic_gpioSetIntr : Parameter validation for handle"
    },
    {
        6257,
        "Pmic_gpioSetIntr : Parameter validation for pin"
    },
    {
        6258,
        "Pmic_gpioSetIntr : Parameter validation for intrType"
    },
    {
        6259,
        "Pmic_gpioSetIntr : Parameter validation for maskPol"
    },
    {
        6200,
        "Pmic_gpioSetConfiguration : configure gpio pin as SPMI SCLK function"
    },
    {
        7374,
        "Pmic_irqMaskIntr/Pmic_irqGpioMaskIntr : Masking all interrupts test"
    },
    {
        7375,
        "Pmic_irqMaskIntr/Pmic_irqGpioMaskIntr : UnMasking all interrupts test"
    },
    {
        1000,
        "Pmic_gpioTps6594xNPwronPinGetValue : Gpio Tps6594x NPwron Get PinValue Test"
    },
    {
        1001,
        "Pmic_gpioTps6594xNPwronPinGetValue : Parameter validation for handle"
    },
    {
        1002,
        "Pmic_gpioTps6594xNPwronPinGetValue : Parameter validation for PinValue"
    },
    {
        7879,
        "Pmic_gpioGetConfiguration : NEgative test for Pin 11 for HERA."
    },
    {
        7880,
        "Pmic_gpioTps6594xNPwronPinGetValue : Negative test for nPWRON Pin for hera"
    },
};

/*!
 * \brief   configure a gpio pin as NSLEEP1 function
 */
static void test_pmic_gpio_setCfgGpioPin_nSLEEP1(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_GpioCfg_t gpioCfg_rd = {PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,};
    int pin                   = 0U;
    uint8_t pins[]            = { 1U, 2U, 3U, 4U, 5U, 6U, 7U, 8U, 9U, 10U, 11U};
    uint8_t pinMax = 0;
    Pmic_GpioCfg_t gpioCfg =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_GPIO_PINFUNC_NSLEEP1,
        PMIC_GPIO_HIGH
    };

    test_pmic_print_unity_testcase_info(6185,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        if(J721E_LEO_PMICA_DEVICE == pmic_device_info)
        {
            pinMax = PMIC_TPS6594X_GPIO11_PIN;
            gpioCfg.pinFunc = PMIC_TPS6594X_GPIO_PINFUNC_NSLEEP1;
        }

        if(J721E_LEO_PMICB_DEVICE == pmic_device_info)
        {
            /*
             * GPIO11 pin of PMIC-B is connected to 'EN_3V3IO_LDSW'
             * on J721EVM board causing hang, when programming NSLEEP1 signal.
             */
            pinMax = PMIC_TPS6594X_GPIO10_PIN;
            gpioCfg.pinFunc = PMIC_TPS6594X_GPIO_PINFUNC_NSLEEP1;
        }
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pinMax = PMIC_LP8764X_GPIO10_PIN;
        gpioCfg.pinFunc = PMIC_LP8764X_GPIO_PINFUNC_NSLEEP1;
    }

    for(pin = 0U; pin < pinMax; pin++)
    {
        /* PMIC-A GPIO3 pin Causing Reset on J721 EVM */
        if((3U == pins[pin]) &&
           (J721E_LEO_PMICA_DEVICE == pmic_device_info))
        {
            continue;
        }

        /* GPIO Pins 1, 2 and 10 are supported on PMIC-B on J721EVM.
         * Refer to 7457 for more details
         */
        if(((1U != pins[pin]) || (2U != pins[pin]) || (10U != pins[pin])) &&
           (J721E_LEO_PMICB_DEVICE == pmic_device_info))
        {
            continue;
        }

        pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle,
                                               pins[pin],
                                               gpioCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        pmicStatus = Pmic_gpioGetConfiguration(pPmicCoreHandle,
                                               pins[pin],
                                               &gpioCfg_rd);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        TEST_ASSERT_EQUAL(gpioCfg.pinFunc, gpioCfg_rd.pinFunc);
    }
}

/*!
 * \brief   configure a gpio pin as NSLEEP2 function
 */
static void test_pmic_gpio_setCfgGpioPin_nSLEEP2(void)
{
    int32_t pmicStatus       = PMIC_ST_SUCCESS;
    Pmic_GpioCfg_t gpioCfg_rd = {PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,};
    int pin                  = 0U;
    uint8_t pins[]           = { 1U, 2U, 3U, 4U, 5U, 6U, 7U, 8U, 9U, 10U, 11U};
    Pmic_GpioCfg_t gpioCfg   =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_GPIO_PINFUNC_NSLEEP2,
        PMIC_GPIO_HIGH
    };
    uint8_t pinMax = 0;

    test_pmic_print_unity_testcase_info(6186,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        if(J721E_LEO_PMICA_DEVICE == pmic_device_info)
        {
            pinMax = PMIC_TPS6594X_GPIO11_PIN;
            gpioCfg.pinFunc = PMIC_TPS6594X_GPIO_PINFUNC_NSLEEP1;
        }
        if(J721E_LEO_PMICB_DEVICE == pmic_device_info)
        {
            /*
             * GPIO11 pin of PMIC-B is connected to 'EN_3V3IO_LDSW'
             * on J721EVM board causing hang, when programming NSLEEP2 signal.
             */
            pinMax = PMIC_TPS6594X_GPIO10_PIN;
            gpioCfg.pinFunc = PMIC_TPS6594X_GPIO_PINFUNC_NSLEEP1;
        }
    }
    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pinMax = PMIC_LP8764X_GPIO10_PIN;
        gpioCfg.pinFunc = PMIC_LP8764X_GPIO_PINFUNC_NSLEEP2;
    }

    for(pin = 0U; pin < pinMax; pin++)
    {
        /* PMICA GPIO3 pin Causing Reset on J721 EVM */
        if((3U == pins[pin]) &&
           (pmic_device_info == J721E_LEO_PMICA_DEVICE))
        {
            continue;
        }

        /* GPIO Pins 1, 2 and 10 are supported on PMIC-B on J721EVM.
         * Refer to 7457 for more details
         */
        if(((1U != pins[pin]) || (2U != pins[pin]) || (10U != pins[pin])) &&
           (J721E_LEO_PMICB_DEVICE == pmic_device_info))
        {
            continue;
        }

        pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle,
                                               pins[pin],
                                               gpioCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        pmicStatus = Pmic_gpioGetConfiguration(pPmicCoreHandle,
                                               pins[pin],
                                               &gpioCfg_rd);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        TEST_ASSERT_EQUAL(gpioCfg.pinFunc, gpioCfg_rd.pinFunc);
    }
}

/*!
 * \brief   configure gpio pin as NRSTOUT_SOC function
 */
static void test_pmic_gpio_setCfgGpioPin_nRstOut_soc(void)
{
    int32_t pmicStatus       = PMIC_ST_SUCCESS;
    uint8_t pins[]           = { 1U, 11U};
    int pin                  = 0U;
    Pmic_GpioCfg_t gpioCfg_rd = {PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,};
    Pmic_GpioCfg_t gpioCfg   =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_GPIO_PINFUNC_GPIO1_GPIO11_NRSTOUT_SOC,
        PMIC_GPIO_HIGH
    };

    test_pmic_print_unity_testcase_info(6187,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        gpioCfg.pinFunc = PMIC_TPS6594X_GPIO_PINFUNC_GPIO1_GPIO11_NRSTOUT_SOC;
    }
    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pins[0U] = 1U;
        pins[1U] = 10U;
        gpioCfg.pinFunc = PMIC_LP8764X_GPIO_PINFUNC_GPIO1_GPIO10_NRSTOUT_SOC;
    }

    for(pin = 0U; pin < (sizeof(pins)/sizeof(pins[0U])); pin++)
    {
        if((J721E_LEO_PMICB_DEVICE == pmic_device_info) && (11U == pins[pin]))
        {
            /*
             * GPIO11 pin of PMIC-B is connected to 'EN_3V3IO_LDSW'
             * on J721EVM board causing hang, when programming NRSTOUT signal.
             */
            continue;
        }

        pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle,
                                               pins[pin],
                                               gpioCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        pmicStatus = Pmic_gpioGetConfiguration(pPmicCoreHandle,
                                               pins[pin],
                                               &gpioCfg_rd);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        TEST_ASSERT_EQUAL(gpioCfg.pinFunc, gpioCfg_rd.pinFunc);
    }
}

/*!
 * \brief   configure a gpio pin as WAKEUP1 function
 */
static void test_pmic_gpio_setCfgGpioPin_wakeup1(void)
{
    int32_t pmicStatus       = PMIC_ST_SUCCESS;
    Pmic_GpioCfg_t gpioCfg_rd = {PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,};
    int pin                  = 0U;
    uint8_t pins[]           = { 1U, 2U, 3U, 4U, 5U, 6U, 7U, 8U, 9U, 10U, 11U};
    Pmic_GpioCfg_t gpioCfg   =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_GPIO_PINFUNC_WKUP1,
        PMIC_GPIO_HIGH
    };
    uint8_t pinMax = 0;

    test_pmic_print_unity_testcase_info(6189,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        if(J721E_LEO_PMICA_DEVICE == pmic_device_info)
        {
            pinMax = PMIC_TPS6594X_GPIO11_PIN;
            gpioCfg.pinFunc = PMIC_TPS6594X_GPIO_PINFUNC_WKUP1;
        }

        if(J721E_LEO_PMICB_DEVICE == pmic_device_info)
        {
            /*
             * GPIO11 pin of PMIC-B is connected to 'EN_3V3IO_LDSW'
             * on J721EVM board causing hang, when programming WKUP1 signal.
             */
            pinMax = PMIC_TPS6594X_GPIO10_PIN;
            gpioCfg.pinFunc = PMIC_TPS6594X_GPIO_PINFUNC_WKUP1;
        }
    }
    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pinMax = PMIC_LP8764X_GPIO10_PIN;
        gpioCfg.pinFunc = PMIC_LP8764X_GPIO_PINFUNC_WKUP1;
    }

    for(pin = 0U; pin < pinMax; pin++)
    {
        /* PMICA GPIO3 pin Causing Reset on J721 EVM */
        if((3U == pins[pin]) &&
           (pmic_device_info == J721E_LEO_PMICA_DEVICE))
        {
            continue;
        }

        /* GPIO Pins 1, 2 and 10 are supported on PMIC-B on J721EVM.
         * Refer to 7457 for more details
         */
        if(((1U != pins[pin]) || (2U != pins[pin]) || (10U != pins[pin])) &&
           (J721E_LEO_PMICB_DEVICE == pmic_device_info))
        {
            continue;
        }

        pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle,
                                               pins[pin],
                                               gpioCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        pmicStatus = Pmic_gpioGetConfiguration(pPmicCoreHandle,
                                               pins[pin],
                                               &gpioCfg_rd);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        TEST_ASSERT_EQUAL(gpioCfg.pinFunc, gpioCfg_rd.pinFunc);
    }
}

/*!
 * \brief   configure a gpio pin as WAKEUP2 function
 */
static void test_pmic_gpio_setCfgGpioPin_wakeup2(void)
{
    int32_t pmicStatus       = PMIC_ST_SUCCESS;
    Pmic_GpioCfg_t gpioCfg_rd = {PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,};
    int pin                  = 0U;
    uint8_t pins[]           = { 1U, 2U, 3U, 4U, 5U, 6U, 7U, 8U, 9U, 10U, 11U};
    Pmic_GpioCfg_t gpioCfg   =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_GPIO_PINFUNC_WKUP2,
        PMIC_GPIO_HIGH
    };
    uint8_t pinMax = 0;

    test_pmic_print_unity_testcase_info(6190,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        if(J721E_LEO_PMICA_DEVICE == pmic_device_info)
        {
            pinMax = PMIC_TPS6594X_GPIO11_PIN;
            gpioCfg.pinFunc = PMIC_TPS6594X_GPIO_PINFUNC_WKUP2;
        }

        if(J721E_LEO_PMICB_DEVICE == pmic_device_info)
        {
            /*
             * GPIO11 pin of PMIC-B is connected to 'EN_3V3IO_LDSW'
             * on J721EVM board causing hang, when programming WKUP1 signal.
             */
            pinMax = PMIC_TPS6594X_GPIO10_PIN;
            gpioCfg.pinFunc = PMIC_TPS6594X_GPIO_PINFUNC_WKUP2;
        }
    }
    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pinMax = PMIC_LP8764X_GPIO10_PIN;
        gpioCfg.pinFunc = PMIC_LP8764X_GPIO_PINFUNC_WKUP2;
    }

    for(pin = 0U; pin < pinMax; pin++)
    {
        /* PMICA GPIO3 pin Causing Reset on J721 EVM */
        if((3U == pins[pin]) &&
           (pmic_device_info == J721E_LEO_PMICA_DEVICE))
        {
            continue;
        }

        /* GPIO Pins 1, 2 and 10 are supported on PMIC-B on J721EVM.
         * Refer to 7457 for more details
         */
        if(((1U != pins[pin]) || (2U != pins[pin]) || (10U != pins[pin])) &&
           (J721E_LEO_PMICB_DEVICE == pmic_device_info))
        {
            continue;
        }

        pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle,
                                               pins[pin],
                                               gpioCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        pmicStatus = Pmic_gpioGetConfiguration(pPmicCoreHandle,
                                               pins[pin],
                                               &gpioCfg_rd);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        TEST_ASSERT_EQUAL(gpioCfg.pinFunc, gpioCfg_rd.pinFunc);
    }
}

/*!
 * \brief   configure a gpio pin as general purpose I/O function
 */
static void test_pmic_gpio_setCfgGpioPin_gpio(void)
{
    int32_t pmicStatus       = PMIC_ST_SUCCESS;
    Pmic_GpioCfg_t gpioCfg_rd = {PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,};
    int pin                  = 0U;
    uint8_t pins[]           = { 1U, 2U, 3U, 4U, 5U, 6U, 7U, 8U, 9U, 10U, 11U};
    Pmic_GpioCfg_t gpioCfg   =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_GPIO_PINFUNC_GPIO,
        PMIC_GPIO_HIGH
    };
    uint8_t pinMax = 0;

    test_pmic_print_unity_testcase_info(6191,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pinMax = PMIC_TPS6594X_GPIO11_PIN;
        gpioCfg.pinFunc = PMIC_TPS6594X_GPIO_PINFUNC_GPIO;
    }
    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pinMax = PMIC_LP8764X_GPIO10_PIN;
        gpioCfg.pinFunc = PMIC_LP8764X_GPIO_PINFUNC_GPIO;
    }

    for(pin = 0U; pin < pinMax; pin++)
    {
        /* On J721E, PMIC-A GPIO11 is connected to H_SOC_PORz,
         * which resets entire SOC.
         */
        if((11U == pins[pin]) &&
           (pmic_device_info == J721E_LEO_PMICA_DEVICE))
        {
            continue;
        }

        /* GPIO Pins 1, 2 and 10 are supported on PMIC-B on J721EVM.
         * Refer to 7457 for more details
         */
        if(((1U != pins[pin]) || (2U != pins[pin]) || (10U != pins[pin])) &&
           (J721E_LEO_PMICB_DEVICE == pmic_device_info))
        {
            continue;
        }

        pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle,
                                               pins[pin],
                                               gpioCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        pmicStatus = Pmic_gpioGetConfiguration(pPmicCoreHandle,
                                               pins[pin],
                                               &gpioCfg_rd);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        TEST_ASSERT_EQUAL(gpioCfg.pinFunc, gpioCfg_rd.pinFunc);
    }
}

/*!
 * \brief   configure gpio pin as I2C2 SCLK function
 */
static void test_pmic_gpio_setCfgGpioPin_i2c2_sclk(void)
{
    int32_t pmicStatus       = PMIC_ST_SUCCESS;
    int pin                  = 1U;
    Pmic_GpioCfg_t gpioCfg_rd = {PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,};
    Pmic_GpioCfg_t gpioCfg   =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_GPIO_PINFUNC_GPIO1_SCL_I2C2_CS_SPI,
        PMIC_GPIO_HIGH
    };

    test_pmic_print_unity_testcase_info(6192,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pin = 1U;
        gpioCfg.pinFunc = PMIC_TPS6594X_GPIO_PINFUNC_GPIO1_SCL_I2C2_CS_SPI;
    }
    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pin = 2U;
        gpioCfg.pinFunc = PMIC_LP8764X_GPIO_PINFUNC_GPIO2_SCL_I2C2;
    }

    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioGetConfiguration(pPmicCoreHandle, pin, &gpioCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(gpioCfg.pinFunc, gpioCfg_rd.pinFunc);
}

/*!
 * \brief   configure gpio pin as I2C2 SDA function
 */
static void test_pmic_gpio_setCfgGpioPin_i2c2_sda(void)
{
    int32_t pmicStatus       = PMIC_ST_SUCCESS;
    int pin                  = 2U;
    Pmic_GpioCfg_t gpioCfg_rd = {PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,};
    Pmic_GpioCfg_t gpioCfg   =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_GPIO_PINFUNC_GPIO2_SDA_I2C2_SDO_SPI,
        PMIC_GPIO_HIGH
    };

    test_pmic_print_unity_testcase_info(6193,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pin = 2U;
        gpioCfg.pinFunc = PMIC_TPS6594X_GPIO_PINFUNC_GPIO2_SDA_I2C2_SDO_SPI;
    }
    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pin = 3U;
        gpioCfg.pinFunc = PMIC_LP8764X_GPIO_PINFUNC_GPIO3_SDA_I2C2;
    }

    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioGetConfiguration(pPmicCoreHandle, pin, &gpioCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(gpioCfg.pinFunc, gpioCfg_rd.pinFunc);
}

/*!
 * \brief   configure gpio pin as SPI CS function
 */
static void test_pmic_gpio_setCfgGpioPin_spi_cs(void)
{
    int32_t pmicStatus       = PMIC_ST_SUCCESS;
    int pin                  = 1U;
    Pmic_GpioCfg_t gpioCfg_rd = {PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,};
    Pmic_GpioCfg_t gpioCfg   =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_GPIO_PINFUNC_GPIO1_SCL_I2C2_CS_SPI,
        PMIC_GPIO_HIGH
    };

    test_pmic_print_unity_testcase_info(6195,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pin = 1U;
        gpioCfg.pinFunc = PMIC_TPS6594X_GPIO_PINFUNC_GPIO1_SCL_I2C2_CS_SPI;
    }
    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pin = 2U;
        gpioCfg.pinFunc = PMIC_LP8764X_GPIO_PINFUNC_GPIO2_CS_SPI;
    }

    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioGetConfiguration(pPmicCoreHandle, pin, &gpioCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(gpioCfg.pinFunc, gpioCfg_rd.pinFunc);
}

/*!
 * \brief   configure gpio pin as SPI SDO function
 */
static void test_pmic_gpio_setCfgGpioPin_spi_sdo(void)
{
    int32_t pmicStatus       = PMIC_ST_SUCCESS;
    int pin                  = 2U;
    Pmic_GpioCfg_t gpioCfg_rd = {PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,};
    Pmic_GpioCfg_t gpioCfg   =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_GPIO_PINFUNC_GPIO2_SDA_I2C2_SDO_SPI,
        PMIC_GPIO_HIGH
    };

    test_pmic_print_unity_testcase_info(6196,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pin = 2U;
        gpioCfg.pinFunc = PMIC_TPS6594X_GPIO_PINFUNC_GPIO2_SDA_I2C2_SDO_SPI;
    }
    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pin = 3U;
        gpioCfg.pinFunc = PMIC_LP8764X_GPIO_PINFUNC_GPIO3_SDO_SPI;
    }

    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioGetConfiguration(pPmicCoreHandle, pin, &gpioCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(gpioCfg.pinFunc, gpioCfg_rd.pinFunc);
}

/*!
 * \brief   configure a gpio pin as Watchdog Trigger function
 */
static void test_pmic_gpio_setCfgGpioPin_wdt(void)
{
    int32_t pmicStatus       = PMIC_ST_SUCCESS;
    Pmic_GpioCfg_t gpioCfg_rd = {PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,};
    int pin                  = 0U;
    uint8_t pins[]           = {2U, 11U};
    Pmic_GpioCfg_t gpioCfg   =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_GPIO_PINFUNC_GPIO2_GPIO11_TRIG_WDOG,
        PMIC_GPIO_HIGH
    };
    uint8_t pinMax = 0;

    test_pmic_print_unity_testcase_info(6197,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    pinMax = sizeof(pins)/sizeof(pins[0U]);
    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        gpioCfg.pinFunc = PMIC_TPS6594X_GPIO_PINFUNC_GPIO2_GPIO11_TRIG_WDOG;
    }
    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pins[1U] = 4U;
        gpioCfg.pinFunc = PMIC_LP8764X_GPIO_PINFUNC_GPIO2_TRIG_WDOG;
    }

    for(pin = 0U; pin < pinMax; pin++)
    {
        if((J721E_LEO_PMICB_DEVICE == pmic_device_info) && (11U == pins[pin]))
        {
            /*
             * GPIO11 pin of PMIC-B is connected to 'EN_3V3IO_LDSW'
             * on J721EVM board causing hang, when programming TRIG_WDOG signal
             */
            continue;
        }

        pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle,
                                               pins[pin],
                                               gpioCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);


        pmicStatus = Pmic_gpioGetConfiguration(pPmicCoreHandle,
                                               pins[pin],
                                               &gpioCfg_rd);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        TEST_ASSERT_EQUAL(gpioCfg.pinFunc, gpioCfg_rd.pinFunc);
    }
}

/*!
 * \brief   configure gpio pin3 as ESM Error pin for SOC
 */
static void test_pmic_gpio_setCfgGpioPin3_esm_soc(void)
{
    int32_t pmicStatus       = PMIC_ST_SUCCESS;
    int pin                  = 3U;
    Pmic_GpioCfg_t gpioCfg_rd = {PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,};
    Pmic_GpioCfg_t gpioCfg   =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_GPIO_PINFUNC_GPIO3_NERR_SOC,
        PMIC_GPIO_HIGH
    };

    test_pmic_print_unity_testcase_info(6198,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        if(J721E_LEO_PMICA_DEVICE == pmic_device_info)
        {
            /*
             * GPIO3 pin of PMIC-A is connected to 'EN_MCU3V3_LDSW'
             * on J721EVM board causing hang, when programming NERR_SOC signal.
             */
            TEST_IGNORE();
        }
    }

    if(J721E_LEO_PMICB_DEVICE == pmic_device_info)
    {
        TEST_IGNORE();
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        TEST_IGNORE();
    }
    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioGetConfiguration(pPmicCoreHandle, pin, &gpioCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(gpioCfg.pinFunc, gpioCfg_rd.pinFunc);
}

/*!
 * \brief   configure gpio pin as ESM Error pin for MCU
 */
static void test_pmic_gpio_setCfgGpioPin_esm_mcu(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    int pin                   = 7U;
    Pmic_GpioCfg_t gpioCfg_rd = {PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,};
    Pmic_GpioCfg_t gpioCfg    =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_GPIO_PINFUNC_GPIO7_NERR_MCU,
        PMIC_GPIO_HIGH
    };

    test_pmic_print_unity_testcase_info(6199,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    if(J721E_LEO_PMICB_DEVICE == pmic_device_info)
    {
        TEST_IGNORE();
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        TEST_IGNORE();
    }

    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioGetConfiguration(pPmicCoreHandle, pin, &gpioCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(gpioCfg.pinFunc, gpioCfg_rd.pinFunc);
}

/*!
 * \brief   configure gpio pin as SPMI SCLK function
 */
static void test_pmic_gpio_setCfgGpioPin_spmi_sclk(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    int pin                   = 5U;
    Pmic_GpioCfg_t gpioCfg_rd = {PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,};
    Pmic_GpioCfg_t gpioCfg    =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_GPIO_PINFUNC_GPIO5_SCLK_SPMI,
        PMIC_GPIO_HIGH
    };

    test_pmic_print_unity_testcase_info(6200,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    if(J721E_LEO_PMICB_DEVICE == pmic_device_info)
    {
        TEST_IGNORE();
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        gpioCfg.pinFunc = PMIC_TPS6594X_GPIO_PINFUNC_GPIO5_SCLK_SPMI;
    }
    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pin = 8U;
        gpioCfg.pinFunc = PMIC_LP8764X_GPIO_PINFUNC_GPIO8_SCLK_SPMI;
    }

    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioGetConfiguration(pPmicCoreHandle, pin, &gpioCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(gpioCfg.pinFunc, gpioCfg_rd.pinFunc);
}

/*!
 * \brief   configure gpio pin as SPMI SDATA function
 */
static void test_pmic_gpio_setCfgGpioPin_spmi_sdata(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    int pin                   = 6U;
    Pmic_GpioCfg_t gpioCfg_rd = {PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,};
    Pmic_GpioCfg_t gpioCfg    =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_GPIO_PINFUNC_GPIO6_SDATA_SPMI,
        PMIC_GPIO_HIGH
    };

    test_pmic_print_unity_testcase_info(6201,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    if(J721E_LEO_PMICB_DEVICE == pmic_device_info)
    {
        TEST_IGNORE();
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        gpioCfg.pinFunc = PMIC_TPS6594X_GPIO_PINFUNC_GPIO6_SDATA_SPMI;
    }
    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pin = 9U;
        gpioCfg.pinFunc = PMIC_LP8764X_GPIO_PINFUNC_GPIO9_SDATA_SPMI;
    }
    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioGetConfiguration(pPmicCoreHandle, pin, &gpioCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(gpioCfg.pinFunc, gpioCfg_rd.pinFunc);
}

/*!
 * \brief   configure a gpio pin as SYNCCLKOUT function
 */
static void test_pmic_gpio_setCfgGpioPin_syncCLKOUT(void)
{
    int32_t pmicStatus         = PMIC_ST_SUCCESS;
    Pmic_GpioCfg_t gpioCfg_rd = {PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,};
    int pin                    = 0U;
    uint8_t pins[]             = {8U, 10U, 9U};
    uint8_t pinMax             = 3;
    Pmic_GpioCfg_t gpioCfg =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_GPIO_PINFUNC_GPIO8_GPIO10_SYNCCLKOUT,
        PMIC_GPIO_HIGH
    };

    test_pmic_print_unity_testcase_info(6202,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pinMax = 3U;
        gpioCfg.pinFunc = PMIC_TPS6594X_GPIO_PINFUNC_GPIO8_GPIO10_SYNCCLKOUT;
    }
    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pins[0U] = 5U;
        pins[1U] = 6U;
        pinMax = 2U;
        gpioCfg.pinFunc = PMIC_LP8764X_GPIO_PINFUNC_GPIO5_GPIO6_SYNCCLKOUT;
    }

    for(pin = 0U; pin < pinMax; pin++)
    {
        /*!
         * On J721 EVM, PMICA GPIO10 SYNCCLKOUT functionality
         * is not supported
         */
        if((PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType) &&
           (10U == pins[pin]))
        {
            continue;
        }
        if((PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType) &&
           (9U == pins[pin]))
        {
            gpioCfg.pinFunc = PMIC_TPS6594X_GPIO_PINFUNC_GPIO9_SYNCCLKOUT;
        }

        /* GPIO Pins 1, 2 and 10 are supported on PMIC-B on J721EVM.
         * Refer to 7457 for more details
         */
        if((10U != pins[pin]) && (J721E_LEO_PMICB_DEVICE == pmic_device_info))
        {
            continue;
        }

        pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pins[pin], gpioCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        pmicStatus = Pmic_gpioGetConfiguration(pPmicCoreHandle,
                                               pins[pin],
                                               &gpioCfg_rd);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        TEST_ASSERT_EQUAL(gpioCfg.pinFunc, gpioCfg_rd.pinFunc);
    }
}

/*!
 * \brief   configure gpio pin as SYNCCLKIN function
 */
static void test_pmic_gpio_setCfgGpioPin_synCLKIN(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    int pin                   = 10U;
    Pmic_GpioCfg_t gpioCfg_rd = {PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,};
    Pmic_GpioCfg_t gpioCfg    =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_GPIO_PINFUNC_GPIO10_SYNCCLKIN,
        PMIC_GPIO_HIGH
    };

    test_pmic_print_unity_testcase_info(6204,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        gpioCfg.pinFunc = PMIC_TPS6594X_GPIO_PINFUNC_GPIO10_SYNCCLKIN;
    }
    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pin = 5U;
        gpioCfg.pinFunc = PMIC_LP8764X_GPIO_PINFUNC_GPIO5_SYNCCLKIN;
    }

    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioGetConfiguration(pPmicCoreHandle, pin, &gpioCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(gpioCfg.pinFunc, gpioCfg_rd.pinFunc);
}

/*!
 * \brief   configure a gpio pin as CLK32KOUT function
 */
static void test_pmic_gpio_setCfgGpioPin_clk32KOUT(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_GpioCfg_t gpioCfg_rd = {PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,};
    int pin                   = 0U;
    uint8_t pins[]            = {3U, 4U, 8U};
    Pmic_GpioCfg_t gpioCfg    =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_GPIO_PINFUNC_GPIO3_GPIO4_GPIO8_CLK32KOUT,
        PMIC_GPIO_HIGH
    };

    test_pmic_print_unity_testcase_info(6205,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    if(J721E_LEO_PMICB_DEVICE == pmic_device_info)
    {
        TEST_IGNORE();
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        TEST_IGNORE();
    }

    for(pin = 0U; pin < sizeof(pins)/sizeof(pins[0U]); pin++)
    {
        /* PMICA GPIO3 pin Causing Reset on J721 EVM */
        if((3U == pins[pin]) &&
           (pmic_device_info == J721E_LEO_PMICA_DEVICE))
        {
            continue;
        }

        pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle,
                                               pins[pin],
                                               gpioCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        pmicStatus = Pmic_gpioGetConfiguration(pPmicCoreHandle,
                                               pins[pin],
                                               &gpioCfg_rd);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        TEST_ASSERT_EQUAL(gpioCfg.pinFunc, gpioCfg_rd.pinFunc);
    }
}

/*!
 * \brief   configure gpio pin10 as CLK32KOUT function
 */
static void test_pmic_gpio_setCfgGpioPin10_clk32KOUT(void)
{
    int32_t pmicStatus       = PMIC_ST_SUCCESS;
    int pin                  = 10U;
    Pmic_GpioCfg_t gpioCfg_rd = {PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,};
    Pmic_GpioCfg_t gpioCfg   =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_GPIO_PINFUNC_GPIO10_CLK32KOUT,
        PMIC_GPIO_HIGH
    };

    test_pmic_print_unity_testcase_info(6206,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        if(pmic_device_info == J721E_LEO_PMICA_DEVICE)
        {
            TEST_IGNORE();
        }
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        TEST_IGNORE();
    }

    /* On J721 EVM, PMICA GPIO10 CLK32KOUT functionality is not supported */
    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioGetConfiguration(pPmicCoreHandle, pin, &gpioCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(gpioCfg.pinFunc, gpioCfg_rd.pinFunc);
}

/*!
 * \brief   configure gpio pin as Watchdog disable function
 */
static void test_pmic_gpio_setCfgGpioPin_wdg_disable(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    int pin                   = 0U;
    int pins[]                = {8U, 9U};
    Pmic_GpioCfg_t gpioCfg_rd = {PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,};
    Pmic_GpioCfg_t gpioCfg    =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_GPIO_PINFUNC_GPIO8_DISABLE_WDOG,
        PMIC_GPIO_HIGH
    };

    test_pmic_print_unity_testcase_info(6207,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    if(J721E_LEO_PMICB_DEVICE == pmic_device_info)
    {
        TEST_IGNORE();
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        TEST_IGNORE();
    }

    for(pin = 0U; pin < (sizeof(pins)/sizeof(pins[0])); pin++)
    {
        if((PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType) &&
           (pins[pin] == 9U))
        {
            gpioCfg.pinFunc = PMIC_TPS6594X_GPIO_PINFUNC_GPIO9_DISABLE_WDOG;
        }

        pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle,
                                               pins[pin],
                                               gpioCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        pmicStatus = Pmic_gpioGetConfiguration(pPmicCoreHandle,
                                               pins[pin],
                                               &gpioCfg_rd);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        TEST_ASSERT_EQUAL(gpioCfg.pinFunc, gpioCfg_rd.pinFunc);
    }
}

/*!
 * \brief   configure gpio pin as Power Good Indication line function
 */
static void test_pmic_gpio_setCfgGpioPin_good_power(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    int pin                   = 0U;
    int pins[3U]              = {9U};
    int pinMax = 0U;
    Pmic_GpioCfg_t gpioCfg_rd = {PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,};
    Pmic_GpioCfg_t gpioCfg    =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_GPIO_PINFUNC_GPIO9_PGOOD,
        PMIC_GPIO_HIGH
    };

    test_pmic_print_unity_testcase_info(6209,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    if(J721E_LEO_PMICB_DEVICE == pmic_device_info)
    {
        TEST_IGNORE();
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pinMax = 1U;
        gpioCfg.pinFunc = PMIC_TPS6594X_GPIO_PINFUNC_GPIO9_PGOOD;
    }
    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pins[0] = 9U;
        pins[1] = 1U;
        pins[2] = 6U;
        pinMax = 3U;
        gpioCfg.pinFunc = PMIC_LP8764X_GPIO_PINFUNC_GPIO9_PGOOD;
    }
    for(pin = 0U; pin < pinMax; pin++)
    {
        if((PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType) && (pin > 0))
        {
            gpioCfg.pinFunc = PMIC_LP8764X_GPIO_PINFUNC_GPIO1_GPIO6_PGOOD;
        }
        pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pins[pin], gpioCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        pmicStatus = Pmic_gpioGetConfiguration(pPmicCoreHandle, pins[pin], &gpioCfg_rd);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        TEST_ASSERT_EQUAL(gpioCfg.pinFunc, gpioCfg_rd.pinFunc);
    }
}

/*!
 * \brief   Parameter validation for handle
 */
static void test_pmic_gpio_setCfgPrmValTest_handle(void)
{
    int32_t pmicStatus     = PMIC_ST_SUCCESS;
    uint8_t pin            = 1U;
    Pmic_GpioCfg_t gpioCfg =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_GPIO_PINFUNC_NSLEEP1,
        PMIC_GPIO_HIGH
    };

    test_pmic_print_unity_testcase_info(6210,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    pmicStatus = Pmic_gpioSetConfiguration(NULL, pin, gpioCfg);
    TEST_ASSERT_EQUAL(pmicStatus, PMIC_ST_ERR_INV_HANDLE);
}

/*!
 * \brief   Parameter validation for pin
 */
static void test_pmic_gpio_setCfgPrmValTest_pin(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    uint8_t pin               = 12U;
    Pmic_GpioCfg_t gpioCfg    =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_GPIO_PINFUNC_NSLEEP1,
        PMIC_GPIO_HIGH
    };

    test_pmic_print_unity_testcase_info(6211,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(pmicStatus, PMIC_ST_ERR_INV_PARAM);
}

/*!
 * \brief   Parameter validation for pinDir
 */
static void test_pmic_gpio_setCfgPrmValTest_pinDir(void)
{
    int32_t pmicStatus       = PMIC_ST_SUCCESS;
    uint8_t pin              = 1U;
    Pmic_GpioCfg_t gpioCfg   =
    {
        PMIC_GPIO_CFG_DIR_VALID_SHIFT,
        0x02,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_GPIO_PINFUNC_NSLEEP1,
        PMIC_GPIO_HIGH
    };

    test_pmic_print_unity_testcase_info(6213,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(pmicStatus, PMIC_ST_ERR_INV_PARAM);
}

/*!
 * \brief   Parameter validation for outputSignalType
 */
static void test_pmic_gpio_setCfgPrmValTest_outputSignalType(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    uint8_t pin               = 1U;
    Pmic_GpioCfg_t gpioCfg    =
    {
        PMIC_GPIO_CFG_OD_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        0x02,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_GPIO_PINFUNC_NSLEEP1,
        PMIC_GPIO_HIGH
    };

    test_pmic_print_unity_testcase_info(6214,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(pmicStatus, PMIC_ST_ERR_INV_PARAM);
}

/*!
 * \brief   Parameter validation for deglitchEnable
 */
static void test_pmic_gpio_setCfgPrmValTest_deglitchEnable(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    uint8_t pin               = 1U;
    Pmic_GpioCfg_t gpioCfg    =
    {
        PMIC_GPIO_CFG_DEGLITCH_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        0x02,
        PMIC_TPS6594X_GPIO_PINFUNC_NSLEEP1,
        PMIC_GPIO_HIGH
    };

    test_pmic_print_unity_testcase_info(6215,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(pmicStatus, PMIC_ST_ERR_INV_PARAM);
}

/*!
 * \brief   Parameter validation for pinFunc
 */
static void test_pmic_gpio_setCfgPrmValTest_pinFunc_case1(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    uint8_t pin               = 1U;
    Pmic_GpioCfg_t gpioCfg    =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        8U,
        PMIC_GPIO_HIGH
    };

    test_pmic_print_unity_testcase_info(6216,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(pmicStatus, PMIC_ST_ERR_INV_GPIO_FUNC);
}

/*!
 * \brief   Parameter validation for nPWRON pinFunc
 */
static void test_pmic_nPWRON_setCfgPrmValTest_pinFunc(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_GpioCfg_t gpioCfg    =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        3U,
        PMIC_GPIO_HIGH
    };

    test_pmic_print_unity_testcase_info(6217,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        TEST_IGNORE();
    }

    pmicStatus = Pmic_gpioSetNPwronEnablePinConfiguration(pPmicCoreHandle,
                                                          gpioCfg);
    TEST_ASSERT_EQUAL(pmicStatus, PMIC_ST_ERR_INV_GPIO_FUNC);
}

/*!
 * \brief   Parameter validation for nPWRON pinPolarity
 */
static void test_pmic_nPWRON_setCfgPrmValTest_pinPolarity(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_GpioCfg_t gpioCfg    =
    {
        PMIC_NPWRON_CFG_POLARITY_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_GPIO_PINFUNC_NSLEEP1,
        0x02U
    };

    test_pmic_print_unity_testcase_info(6218,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    pmicStatus = Pmic_gpioSetNPwronEnablePinConfiguration(pPmicCoreHandle,
                                                          gpioCfg);
    TEST_ASSERT_EQUAL(pmicStatus, PMIC_ST_ERR_INV_PARAM);
}

/*!
 * \brief   Get required GPIO pin configuration
 */
static void test_pmic_gpio_getCfgGpioPin(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    int pin                   = 1U;
    Pmic_GpioCfg_t gpioCfg_rd = {PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,};
    Pmic_GpioCfg_t gpioCfg    =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_GPIO_PINFUNC_NSLEEP1,
        PMIC_GPIO_HIGH
    };

    test_pmic_print_unity_testcase_info(6219,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        gpioCfg.pinFunc = PMIC_TPS6594X_GPIO_PINFUNC_NSLEEP1;
    }
    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        gpioCfg.pinFunc = PMIC_LP8764X_GPIO_PINFUNC_NSLEEP1;
    }

    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioGetConfiguration(pPmicCoreHandle, pin, &gpioCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(gpioCfg.pinFunc, gpioCfg_rd.pinFunc);
}

/*!
 * \brief   Parameter validation for handle
 */
static void test_pmic_gpio_getCfgPrmValTest_handle(void)
{
    int32_t pmicStatus     = PMIC_ST_SUCCESS;
    uint8_t pin            = 1U;
    Pmic_GpioCfg_t gpioCfg =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_GPIO_PINFUNC_NSLEEP1,
        PMIC_GPIO_HIGH
    };

    test_pmic_print_unity_testcase_info(6220,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    pmicStatus = Pmic_gpioGetConfiguration(NULL, pin, &gpioCfg);
    TEST_ASSERT_EQUAL(pmicStatus, PMIC_ST_ERR_INV_HANDLE);
}

/*!
 * \brief   Parameter validation for pin
 */
static void test_pmic_gpio_getCfgPrmValTest_pin(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    uint8_t pin               = 12U;
    Pmic_GpioCfg_t gpioCfg    =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_GPIO_PINFUNC_NSLEEP1,
        PMIC_GPIO_HIGH
    };

    test_pmic_print_unity_testcase_info(6221,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    pmicStatus = Pmic_gpioGetConfiguration(pPmicCoreHandle, pin, &gpioCfg);
    TEST_ASSERT_EQUAL(pmicStatus, PMIC_ST_ERR_INV_PARAM);
}

/*!
 * \brief   Parameter validation for GpioCfg
 */
static void test_pmic_gpio_getCfgPrmValTest_gpioCfg(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    uint8_t pin               = 1U;

    test_pmic_print_unity_testcase_info(6222,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    pmicStatus = Pmic_gpioGetConfiguration(pPmicCoreHandle, pin, NULL);
    TEST_ASSERT_EQUAL(pmicStatus, PMIC_ST_ERR_NULL_PARAM);
}

/*!
 * \brief   Get GPIO signal level
 */
static void test_pmic_gpio_getValueGpioPin1_signalLevel(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    uint8_t pin               = 1U;
    uint8_t pinValue          = PMIC_GPIO_HIGH;
    uint8_t pinValue_rd;
    Pmic_GpioCfg_t gpioCfg    =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT | PMIC_GPIO_CFG_DIR_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_GPIO_PINFUNC_GPIO,
        PMIC_GPIO_HIGH
    };

    test_pmic_print_unity_testcase_info(6223,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        gpioCfg.pinFunc = PMIC_TPS6594X_GPIO_PINFUNC_GPIO;
    }
    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        gpioCfg.pinFunc = PMIC_LP8764X_GPIO_PINFUNC_GPIO;
    }
    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioSetValue(pPmicCoreHandle, pin, pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioGetValue(pPmicCoreHandle, pin, &pinValue_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(pinValue, pinValue_rd);
}

/*!
 * \brief   Parameter validation for handle
 */
static void test_pmic_gpio_getValuePrmValTest_handle(void)
{
    int32_t pmicStatus   = PMIC_ST_SUCCESS;
    uint8_t pin          = 1U;
    uint8_t pinValue;

    test_pmic_print_unity_testcase_info(6224,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    pmicStatus = Pmic_gpioGetValue(NULL, pin, &pinValue);
    TEST_ASSERT_EQUAL(pmicStatus, PMIC_ST_ERR_INV_HANDLE);
}

/*!
 * \brief   Parameter validation for pin
 */
static void test_pmic_gpio_getValuePrmValTest_pin(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    uint8_t pin               = 12U;
    uint8_t pinValue;

    test_pmic_print_unity_testcase_info(6225,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    pmicStatus = Pmic_gpioGetValue(pPmicCoreHandle,pin,&pinValue);
    TEST_ASSERT_EQUAL(pmicStatus, PMIC_ST_ERR_INV_PARAM);
}

/*!
 * \brief   Parameter validation for pinValue
 */
static void test_pmic_gpio_getValuePrmValTest_pinValue(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    uint8_t pin               = 1U;

    test_pmic_print_unity_testcase_info(6226,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    pmicStatus = Pmic_gpioGetValue(pPmicCoreHandle, pin, NULL);
    TEST_ASSERT_EQUAL(pmicStatus, PMIC_ST_ERR_NULL_PARAM);
}

/*!
 * \brief   Set GPIO signal level
 */
static void test_pmic_gpio_setValueGpioPin1_signalLevel(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    uint8_t pin               = 1U;
    uint8_t pinValue          = PMIC_GPIO_HIGH;
    uint8_t pinValue_rd;
    Pmic_GpioCfg_t gpioCfg    =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT | PMIC_GPIO_CFG_DIR_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_GPIO_PINFUNC_GPIO,
        PMIC_GPIO_HIGH
    };

    test_pmic_print_unity_testcase_info(6227,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        gpioCfg.pinFunc = PMIC_TPS6594X_GPIO_PINFUNC_GPIO;
    }
    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        gpioCfg.pinFunc = PMIC_LP8764X_GPIO_PINFUNC_GPIO;
    }
    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioSetValue(pPmicCoreHandle, pin, pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioGetValue(pPmicCoreHandle, pin, &pinValue_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(pinValue, pinValue_rd);
}

/*!
 * \brief   Parameter validation for handle
 */
static void test_pmic_gpio_setValuePrmValTest_handle(void)
{
    int32_t pmicStatus    = PMIC_ST_SUCCESS;
    uint8_t pin           = 1U;
    uint8_t pinValue      = PMIC_GPIO_HIGH;

    test_pmic_print_unity_testcase_info(6228,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    pmicStatus = Pmic_gpioSetValue(NULL, pin, pinValue);
    TEST_ASSERT_EQUAL(pmicStatus, PMIC_ST_ERR_INV_HANDLE);
}

/*!
 * \brief   Parameter validation for pin
 */
static void test_pmic_gpio_setValuePrmValTest_pin(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    uint8_t pin               = 12U;
    uint8_t pinValue          = PMIC_GPIO_HIGH;

    test_pmic_print_unity_testcase_info(6229,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    pmicStatus = Pmic_gpioSetValue(pPmicCoreHandle, pin, pinValue);
    TEST_ASSERT_EQUAL(pmicStatus, PMIC_ST_ERR_INV_PARAM);
}

/*!
 * \brief   Parameter validation for pinValue
 */
static void test_pmic_gpio_setValuePrmValTest_pinValue(void)
{
    int32_t pmicStatus       = PMIC_ST_SUCCESS;
    uint8_t pin              = 1U;
    uint8_t pinValue         = 2U;

    test_pmic_print_unity_testcase_info(6230,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    pmicStatus = Pmic_gpioSetValue(pPmicCoreHandle, pin, pinValue);
    TEST_ASSERT_EQUAL(pmicStatus, PMIC_ST_ERR_INV_PARAM);
}

/*!
 * \brief   Set GPIO signal level for an input GPIO pin
 */
static void test_pmic_gpio_setValueGpioPin1_input(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    uint8_t pinValue          = PMIC_GPIO_HIGH;
    uint8_t pin               = 1U;
    Pmic_GpioCfg_t gpioCfg    =
    {
        PMIC_GPIO_CFG_DIR_VALID_SHIFT,
        PMIC_GPIO_INPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_GPIO_PINFUNC_NSLEEP1,
        PMIC_GPIO_HIGH
    };
    Pmic_GpioCfg_t gpioCfg_rd = {PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,};

    test_pmic_print_unity_testcase_info(6231,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        gpioCfg.pinFunc = PMIC_TPS6594X_GPIO_PINFUNC_GPIO;
    }
    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        gpioCfg.pinFunc = PMIC_LP8764X_GPIO_PINFUNC_GPIO;
    }
    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioGetConfiguration(pPmicCoreHandle, pin, &gpioCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(gpioCfg.pinDir, gpioCfg_rd.pinDir);

    pmicStatus = Pmic_gpioSetValue(pPmicCoreHandle, pin, pinValue);
    TEST_ASSERT_EQUAL(pmicStatus, PMIC_ST_ERR_INV_PARAM);
}

/*!
 * \brief   Test to verify GPIO1 fall interrupt
 */
static void test_pmic_gpio1_testFall_interrupt(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    uint8_t pin               = 1U;
    uint8_t pinValue          = PMIC_GPIO_HIGH;
    uint8_t intrType          = PMIC_GPIO_FALL_INTERRUPT;
    uint8_t maskPol           = PMIC_GPIO_POL_LOW;
    Pmic_IrqStatus_t errStat  = {0U};
    bool clearIRQ             = false;
    uint8_t  irqNum           = 0U;
    Pmic_GpioCfg_t gpioCfg    =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT | PMIC_GPIO_CFG_DIR_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_GPIO_PINFUNC_GPIO,
        PMIC_GPIO_HIGH
    };

    if(J721E_LEO_PMICB_DEVICE == pmic_device_info)
    {
        gpioCfg.validParams = PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT |
                              PMIC_GPIO_CFG_DIR_VALID_SHIFT |
                              PMIC_GPIO_CFG_OD_VALID_SHIFT |
                              PMIC_GPIO_CFG_PULL_VALID_SHIFT;
        gpioCfg.outputSignalType = PMIC_GPIO_PUSH_PULL_OUTPUT;
        gpioCfg.pullCtrl = PMIC_GPIO_PULL_UP;
    }

    test_pmic_print_unity_testcase_info(6234,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        TEST_IGNORE();
    }

    /* Un Masking GPIO 1 FALL Interrupt */
    Pmic_irqGpioMaskIntr(pPmicCoreHandle,
                         PMIC_TPS6594X_IRQ_GPIO_1_INT_MASK_NUM,
                         PMIC_IRQ_UNMASK,
                         PMIC_IRQ_GPIO_FALL_INT_TYPE);

    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioSetValue(pPmicCoreHandle, pin, pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioSetIntr(pPmicCoreHandle, pin , intrType, maskPol);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pinValue = PMIC_GPIO_LOW;
    pmicStatus = Pmic_gpioSetValue(pPmicCoreHandle, pin, pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    while(1U)
    {
        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        if((PMIC_ST_SUCCESS == pmicStatus) &&
           ((errStat.intStatus[PMIC_TPS6594X_GPIO1_INT/32U] & 
             (1U << (PMIC_TPS6594X_GPIO1_INT % 32U))) != 0U))
        {
            while(PMIC_TPS6594X_GPIO1_INT != irqNum)
            {
                pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                                     &errStat,
                                                     &irqNum);
            }

            if(PMIC_ST_SUCCESS == pmicStatus)
            {
                /* clear the interrupt */
                pmicStatus = Pmic_irqClrErrStatus(pPmicCoreHandle,
                                                  PMIC_TPS6594X_GPIO1_INT);
                break;
            }
        }
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
}

/*!
 * \brief   Test to verify GPIO1 rise interrupt
 */
static void test_pmic_gpio1_testRise_interrupt(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    uint8_t pin               = 1U;
    uint8_t pinValue          = PMIC_GPIO_LOW;
    uint8_t intrType          = PMIC_GPIO_RISE_INTERRUPT;
    uint8_t maskPol           = PMIC_GPIO_POL_HIGH;
    Pmic_IrqStatus_t errStat  = {0U};
    bool clearIRQ             = false;
    uint8_t  irqNum           = 0U;
    Pmic_GpioCfg_t gpioCfg    =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT | PMIC_GPIO_CFG_DIR_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_GPIO_PINFUNC_GPIO,
        PMIC_GPIO_HIGH
    };

    if(J721E_LEO_PMICB_DEVICE == pmic_device_info)
    {
        gpioCfg.validParams = PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT |
                              PMIC_GPIO_CFG_DIR_VALID_SHIFT |
                              PMIC_GPIO_CFG_OD_VALID_SHIFT |
                              PMIC_GPIO_CFG_PULL_VALID_SHIFT;
        gpioCfg.outputSignalType = PMIC_GPIO_PUSH_PULL_OUTPUT;
        gpioCfg.pullCtrl = PMIC_GPIO_PULL_UP;
    }

    test_pmic_print_unity_testcase_info(6235,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        TEST_IGNORE();
    }

    /* Un Masking GPIO 1 RISE Interrupt */
    Pmic_irqGpioMaskIntr(pPmicCoreHandle,
                         PMIC_TPS6594X_IRQ_GPIO_1_INT_MASK_NUM,
                         PMIC_IRQ_UNMASK,
                         PMIC_IRQ_GPIO_RISE_INT_TYPE);

    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioSetValue(pPmicCoreHandle, pin, pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioSetIntr(pPmicCoreHandle, pin , intrType, maskPol);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pinValue = PMIC_GPIO_HIGH;
    pmicStatus = Pmic_gpioSetValue(pPmicCoreHandle, pin, pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    while(1U)
    {
        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        if((PMIC_ST_SUCCESS == pmicStatus) &&
           ((errStat.intStatus[PMIC_TPS6594X_GPIO1_INT/32U] & 
             (1U << (PMIC_TPS6594X_GPIO1_INT % 32U))) != 0U))
        {
            while(PMIC_TPS6594X_GPIO1_INT != irqNum)
            {
                pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                                     &errStat,
                                                     &irqNum);
            }

            if(PMIC_ST_SUCCESS == pmicStatus)
            {
                /* clear the interrupt */
                pmicStatus = Pmic_irqClrErrStatus(pPmicCoreHandle,
                                                  PMIC_TPS6594X_GPIO1_INT);
                break;
            }
        }
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
}

/*!
 * \brief   Test to verify GPIO2 fall interrupt
 */
static void test_pmic_gpio2_testFall_interrupt(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    uint8_t pin               = 2U;
    uint8_t pinValue          = PMIC_GPIO_HIGH;
    uint8_t intrType          = PMIC_GPIO_FALL_INTERRUPT;
    uint8_t maskPol           = PMIC_GPIO_POL_LOW;
    Pmic_IrqStatus_t errStat  = {0U};
    bool clearIRQ             = false;
    uint8_t  irqNum           = 0U;
    Pmic_GpioCfg_t gpioCfg    =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT | PMIC_GPIO_CFG_DIR_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_GPIO_PINFUNC_GPIO,
        PMIC_GPIO_HIGH
    };

    if(J721E_LEO_PMICB_DEVICE == pmic_device_info)
    {
        gpioCfg.validParams = PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT |
                              PMIC_GPIO_CFG_DIR_VALID_SHIFT |
                              PMIC_GPIO_CFG_OD_VALID_SHIFT |
                              PMIC_GPIO_CFG_PULL_VALID_SHIFT;
        gpioCfg.outputSignalType = PMIC_GPIO_PUSH_PULL_OUTPUT;
        gpioCfg.pullCtrl = PMIC_GPIO_PULL_UP;
    }

    test_pmic_print_unity_testcase_info(6236,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        TEST_IGNORE();
    }

    /* Un Masking GPIO 2 FALL Interrupt */
    Pmic_irqGpioMaskIntr(pPmicCoreHandle,
                         PMIC_TPS6594X_IRQ_GPIO_2_INT_MASK_NUM,
                         PMIC_IRQ_UNMASK,
                         PMIC_IRQ_GPIO_FALL_INT_TYPE);

    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioSetValue(pPmicCoreHandle, pin, pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioSetIntr(pPmicCoreHandle, pin , intrType, maskPol);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pinValue = PMIC_GPIO_LOW;
    pmicStatus = Pmic_gpioSetValue(pPmicCoreHandle, pin, pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    while(1U)
    {
        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        if((PMIC_ST_SUCCESS == pmicStatus) &&
           ((errStat.intStatus[PMIC_TPS6594X_GPIO2_INT/32U] & 
             (1U << (PMIC_TPS6594X_GPIO2_INT % 32U))) != 0U))
        {
            while(PMIC_TPS6594X_GPIO2_INT != irqNum)
            {
                pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                                     &errStat,
                                                     &irqNum);
            }

            if(PMIC_ST_SUCCESS == pmicStatus)
            {
                /* clear the interrupt */
                pmicStatus = Pmic_irqClrErrStatus(pPmicCoreHandle,
                                                  PMIC_TPS6594X_GPIO2_INT);
                break;
            }
        }
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
}

/*!
 * \brief   Test to verify GPIO2 rise interrupt
 */
static void test_pmic_gpio2_testRise_interrupt(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    uint8_t pin               = 2U;
    uint8_t pinValue          = PMIC_GPIO_LOW;
    uint8_t intrType          = PMIC_GPIO_RISE_INTERRUPT;
    uint8_t maskPol           = PMIC_GPIO_POL_HIGH;
    Pmic_IrqStatus_t errStat  = {0U};
    bool clearIRQ             = false;
    uint8_t  irqNum           = 0U;
    Pmic_GpioCfg_t gpioCfg    =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT | PMIC_GPIO_CFG_DIR_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_GPIO_PINFUNC_GPIO,
        PMIC_GPIO_HIGH
    };

    if(J721E_LEO_PMICB_DEVICE == pmic_device_info)
    {
        gpioCfg.validParams = PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT |
                              PMIC_GPIO_CFG_DIR_VALID_SHIFT |
                              PMIC_GPIO_CFG_OD_VALID_SHIFT |
                              PMIC_GPIO_CFG_PULL_VALID_SHIFT;
        gpioCfg.outputSignalType = PMIC_GPIO_PUSH_PULL_OUTPUT;
        gpioCfg.pullCtrl = PMIC_GPIO_PULL_UP;
    }

    test_pmic_print_unity_testcase_info(6237,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        TEST_IGNORE();
    }

    /* Un Masking GPIO 2 RISE Interrupt */
    Pmic_irqGpioMaskIntr(pPmicCoreHandle,
                         PMIC_TPS6594X_IRQ_GPIO_2_INT_MASK_NUM,
                         PMIC_IRQ_UNMASK,
                         PMIC_IRQ_GPIO_RISE_INT_TYPE);

    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioSetValue(pPmicCoreHandle, pin, pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioSetIntr(pPmicCoreHandle, pin , intrType, maskPol);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pinValue = PMIC_GPIO_HIGH;
    pmicStatus = Pmic_gpioSetValue(pPmicCoreHandle, pin, pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    while(1U)
    {
        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        if((PMIC_ST_SUCCESS == pmicStatus) &&
           ((errStat.intStatus[PMIC_TPS6594X_GPIO2_INT/32U] & 
             (1U << (PMIC_TPS6594X_GPIO2_INT % 32U))) != 0U))
        {
            while(PMIC_TPS6594X_GPIO2_INT != irqNum)
            {
                pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                                     &errStat,
                                                     &irqNum);
            }

            if(PMIC_ST_SUCCESS == pmicStatus)
            {
                /* clear the interrupt */
                pmicStatus = Pmic_irqClrErrStatus(pPmicCoreHandle,
                                                  PMIC_TPS6594X_GPIO2_INT);
                break;
            }
        }
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
}

/*!
 * \brief   Test to verify GPIO3 fall interrupt
 */
static void test_pmic_gpio3_testFall_interrupt(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    uint8_t pin               = 3U;
    uint8_t pinValue          = PMIC_GPIO_HIGH;
    uint8_t intrType          = PMIC_GPIO_FALL_INTERRUPT;
    uint8_t maskPol           = PMIC_GPIO_POL_LOW;
    Pmic_IrqStatus_t errStat  = {0U};
    bool clearIRQ             = false;
    uint8_t  irqNum           = 0U;
    Pmic_GpioCfg_t gpioCfg    =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT | PMIC_GPIO_CFG_DIR_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_GPIO_PINFUNC_GPIO,
        PMIC_GPIO_HIGH
    };

    test_pmic_print_unity_testcase_info(6238,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        if(J721E_LEO_PMICA_DEVICE == pmic_device_info)
        {
            /*
             * GPIO3 pin of PMIC-A is connected to 'EN_MCU3V3_LDSW'
             * on J721EVM board causing hang, when programming fall interrupt.
             */
             TEST_IGNORE();
        }
        if(J721E_LEO_PMICB_DEVICE == pmic_device_info)
        {
            /*
             * GPIO3 pin of PMIC-B is connected to 'EN_DDR0V6_BUCK'
             * on J721EVM board causing hang, when programming fall interrupt.
             */
            TEST_IGNORE();
        }
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        TEST_IGNORE();
    }

    /* Un Masking GPIO 3 FALL Interrupt */
    Pmic_irqGpioMaskIntr(pPmicCoreHandle,
                         PMIC_TPS6594X_IRQ_GPIO_3_INT_MASK_NUM,
                         PMIC_IRQ_UNMASK,
                         PMIC_IRQ_GPIO_FALL_INT_TYPE);

    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioSetValue(pPmicCoreHandle, pin, pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioSetIntr(pPmicCoreHandle, pin , intrType, maskPol);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pinValue = PMIC_GPIO_LOW;
    pmicStatus = Pmic_gpioSetValue(pPmicCoreHandle, pin, pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    while(1U)
    {
        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        if((PMIC_ST_SUCCESS == pmicStatus) &&
           ((errStat.intStatus[PMIC_TPS6594X_GPIO3_INT/32U] & 
             (1U << (PMIC_TPS6594X_GPIO3_INT % 32U))) != 0U))
        {
            while(PMIC_TPS6594X_GPIO3_INT != irqNum)
            {
                pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                                     &errStat,
                                                     &irqNum);
            }

            if(PMIC_ST_SUCCESS == pmicStatus)
            {
                /* clear the interrupt */
                pmicStatus = Pmic_irqClrErrStatus(pPmicCoreHandle,
                                                  PMIC_TPS6594X_GPIO3_INT);
                break;
            }
        }
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
}

/*!
 * \brief   Test to verify GPIO3 rise interrupt
 */
static void test_pmic_gpio3_testRise_interrupt(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    uint8_t pin               = 3U;
    uint8_t pinValue          = PMIC_GPIO_LOW;
    uint8_t intrType          = PMIC_GPIO_RISE_INTERRUPT;
    uint8_t maskPol           = PMIC_GPIO_POL_HIGH;
    Pmic_IrqStatus_t errStat  = {0U};
    bool clearIRQ             = false;
    uint8_t  irqNum           = 0U;
    Pmic_GpioCfg_t gpioCfg    =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT | PMIC_GPIO_CFG_DIR_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_GPIO_PINFUNC_GPIO,
        PMIC_GPIO_HIGH
    };

    test_pmic_print_unity_testcase_info(6239,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        if(J721E_LEO_PMICA_DEVICE == pmic_device_info)
        {
            /*
             * GPIO3 pin of PMIC-A is connected to 'EN_MCU3V3_LDSW'
             * on J721EVM board causing hang, when programming rise interrupt.
             */
             TEST_IGNORE();
        }
        if(J721E_LEO_PMICB_DEVICE == pmic_device_info)
        {
            /*
             * GPIO3 pin of PMIC-B is connected to 'EN_DDR0V6_BUCK'
             * on J721EVM board causing hang, when programming rise interrupt.
             */
            TEST_IGNORE();
        }
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        TEST_IGNORE();
    }

    /* Un Masking GPIO 3 RISE Interrupt */
    Pmic_irqGpioMaskIntr(pPmicCoreHandle,
                         PMIC_TPS6594X_IRQ_GPIO_3_INT_MASK_NUM,
                         PMIC_IRQ_UNMASK,
                         PMIC_IRQ_GPIO_RISE_INT_TYPE);

    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioSetValue(pPmicCoreHandle, pin, pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioSetIntr(pPmicCoreHandle, pin , intrType, maskPol);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pinValue = PMIC_GPIO_HIGH;
    pmicStatus = Pmic_gpioSetValue(pPmicCoreHandle, pin, pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    while(1U)
    {
        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        if((PMIC_ST_SUCCESS == pmicStatus) &&
           ((errStat.intStatus[PMIC_TPS6594X_GPIO3_INT/32U] & 
             (1U << (PMIC_TPS6594X_GPIO3_INT % 32U))) != 0U))
        {
            while(PMIC_TPS6594X_GPIO3_INT != irqNum)
            {
                pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                                     &errStat,
                                                     &irqNum);
            }

            if(PMIC_ST_SUCCESS == pmicStatus)
            {
                /* clear the interrupt */
                pmicStatus = Pmic_irqClrErrStatus(pPmicCoreHandle,
                                                  PMIC_TPS6594X_GPIO3_INT);
                break;
            }
        }
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
}

/*!
 * \brief   Test to verify GPIO4 fall interrupt
 */
static void test_pmic_gpio4_testFall_interrupt(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    uint8_t pin               = 4U;
    uint8_t pinValue          = PMIC_GPIO_HIGH;
    uint8_t intrType          = PMIC_GPIO_FALL_INTERRUPT;
    uint8_t maskPol           = PMIC_GPIO_POL_LOW;
    Pmic_IrqStatus_t errStat  = {0U};
    bool clearIRQ             = false;
    uint8_t  irqNum           = 0U;
    Pmic_GpioCfg_t gpioCfg    =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT | PMIC_GPIO_CFG_DIR_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_GPIO_PINFUNC_GPIO,
        PMIC_GPIO_HIGH
    };

    test_pmic_print_unity_testcase_info(6240,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        if(J721E_LEO_PMICB_DEVICE == pmic_device_info)
        {
            /*
             * GPIO4 pin of PMIC-B is connected to 'H_DDR_RET_1V1'
             * on J721EVM board causing hang, when programming fall interrupt.
             */
            TEST_IGNORE();
        }
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        TEST_IGNORE();
    }

    /* Un Masking GPIO 4 FALL Interrupt */
    Pmic_irqGpioMaskIntr(pPmicCoreHandle,
                         PMIC_TPS6594X_IRQ_GPIO_4_INT_MASK_NUM,
                         PMIC_IRQ_UNMASK,
                         PMIC_IRQ_GPIO_FALL_INT_TYPE);

    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioSetValue(pPmicCoreHandle, pin, pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioSetIntr(pPmicCoreHandle, pin , intrType, maskPol);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pinValue = PMIC_GPIO_LOW;
    pmicStatus = Pmic_gpioSetValue(pPmicCoreHandle, pin, pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    while(1U)
    {
        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        if((PMIC_ST_SUCCESS == pmicStatus) &&
           ((errStat.intStatus[PMIC_TPS6594X_GPIO4_INT/32U] & 
             (1U << (PMIC_TPS6594X_GPIO4_INT % 32U))) != 0U))
        {
            while(PMIC_TPS6594X_GPIO4_INT != irqNum)
            {
                pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                                     &errStat,
                                                     &irqNum);
            }

            if(PMIC_ST_SUCCESS == pmicStatus)
            {
                /* clear the interrupt */
                pmicStatus = Pmic_irqClrErrStatus(pPmicCoreHandle,
                                                  PMIC_TPS6594X_GPIO4_INT);
                break;
            }
        }
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
}

/*!
 * \brief   Test to verify GPIO4 rise interrupt
 */
static void test_pmic_gpio4_testRise_interrupt(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    uint8_t pin               = 4U;
    uint8_t pinValue          = PMIC_GPIO_LOW;
    uint8_t intrType          = PMIC_GPIO_RISE_INTERRUPT;
    uint8_t maskPol           = PMIC_GPIO_POL_HIGH;
    Pmic_IrqStatus_t errStat  = {0U};
    bool clearIRQ             = false;
    uint8_t  irqNum           = 0U;
    Pmic_GpioCfg_t gpioCfg    =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT | PMIC_GPIO_CFG_DIR_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_GPIO_PINFUNC_GPIO,
        PMIC_GPIO_HIGH
    };

    test_pmic_print_unity_testcase_info(6241,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        if(J721E_LEO_PMICB_DEVICE == pmic_device_info)
        {
            /*
             * GPIO4 pin of PMIC-B is connected to 'H_DDR_RET_1V1'
             * on J721EVM board causing hang, when programming rise interrupt.
             */
            TEST_IGNORE();
        }
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        TEST_IGNORE();
    }

    /* Un Masking GPIO 4 RISE Interrupt */
    Pmic_irqGpioMaskIntr(pPmicCoreHandle,
                         PMIC_TPS6594X_IRQ_GPIO_4_INT_MASK_NUM,
                         PMIC_IRQ_UNMASK,
                         PMIC_IRQ_GPIO_RISE_INT_TYPE);

    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioSetValue(pPmicCoreHandle, pin, pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioSetIntr(pPmicCoreHandle, pin , intrType, maskPol);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pinValue = PMIC_GPIO_HIGH;
    pmicStatus = Pmic_gpioSetValue(pPmicCoreHandle, pin, pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    while(1U)
    {
        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        if((PMIC_ST_SUCCESS == pmicStatus) &&
           ((errStat.intStatus[PMIC_TPS6594X_GPIO4_INT/32U] & 
             (1U << (PMIC_TPS6594X_GPIO4_INT % 32U))) != 0U))
        {
            while(PMIC_TPS6594X_GPIO4_INT != irqNum)
            {
                pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                                     &errStat,
                                                     &irqNum);
            }

            if(PMIC_ST_SUCCESS == pmicStatus)
            {
                /* clear the interrupt */
                pmicStatus = Pmic_irqClrErrStatus(pPmicCoreHandle,
                                                  PMIC_TPS6594X_GPIO4_INT);
                break;
            }
        }
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
}

/*!
 * \brief   Test to verify GPIO5 fall interrupt
 */
static void test_pmic_gpio5_testFall_interrupt(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    uint8_t pin               = 5U;
    uint8_t pinValue          = PMIC_GPIO_HIGH;
    uint8_t intrType          = PMIC_GPIO_FALL_INTERRUPT;
    uint8_t maskPol           = PMIC_GPIO_POL_LOW;
    Pmic_IrqStatus_t errStat  = {0U};
    bool clearIRQ             = false;
    uint8_t  irqNum           = 0U;
    Pmic_GpioCfg_t gpioCfg    =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT | PMIC_GPIO_CFG_DIR_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_GPIO_PINFUNC_GPIO,
        PMIC_GPIO_HIGH
    };

    test_pmic_print_unity_testcase_info(6242,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        if(J721E_LEO_PMICB_DEVICE == pmic_device_info)
        {
            /*
             * GPIO5 pin of PMIC-B is connected to 'LEOA_SCLK'
             * on J721EVM causing block, when programming fall interrupt signal.
             */
            TEST_IGNORE();
        }
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        TEST_IGNORE();
    }

    /* Un Masking GPIO 5 FALL Interrupt */
    Pmic_irqGpioMaskIntr(pPmicCoreHandle,
                         PMIC_TPS6594X_IRQ_GPIO_5_INT_MASK_NUM,
                         PMIC_IRQ_UNMASK,
                         PMIC_IRQ_GPIO_FALL_INT_TYPE);

    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioSetValue(pPmicCoreHandle, pin, pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioSetIntr(pPmicCoreHandle, pin , intrType, maskPol);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pinValue = PMIC_GPIO_LOW;
    pmicStatus = Pmic_gpioSetValue(pPmicCoreHandle, pin, pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    while(1U)
    {
        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        if((PMIC_ST_SUCCESS == pmicStatus) &&
           ((errStat.intStatus[PMIC_TPS6594X_GPIO5_INT/32U] & 
             (1U << (PMIC_TPS6594X_GPIO5_INT % 32U))) != 0U))
        {
            while(PMIC_TPS6594X_GPIO5_INT != irqNum)
            {
                pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                                     &errStat,
                                                     &irqNum);
            }

            if(PMIC_ST_SUCCESS == pmicStatus)
            {
                /* clear the interrupt */
                pmicStatus = Pmic_irqClrErrStatus(pPmicCoreHandle,
                                                  PMIC_TPS6594X_GPIO5_INT);
                break;
            }
        }
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
}

/*!
 * \brief   Test to verify GPIO5 rise interrupt
 */
static void test_pmic_gpio5_testRise_interrupt(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    uint8_t pin               = 5U;
    uint8_t pinValue          = PMIC_GPIO_LOW;
    uint8_t intrType          = PMIC_GPIO_RISE_INTERRUPT;
    uint8_t maskPol           = PMIC_GPIO_POL_HIGH;
    Pmic_IrqStatus_t errStat  = {0U};
    bool clearIRQ             = false;
    uint8_t  irqNum           = 0U;
    Pmic_GpioCfg_t gpioCfg    =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT | PMIC_GPIO_CFG_DIR_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_GPIO_PINFUNC_GPIO,
        PMIC_GPIO_HIGH
    };

    test_pmic_print_unity_testcase_info(6243,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        if(J721E_LEO_PMICB_DEVICE == pmic_device_info)
        {
            /*
             * GPIO5 pin of PMIC-B is connected to 'LEOA_SCLK'
             * on J721EVM causing block, when programming rise` interrupt signal.
             */
            TEST_IGNORE();
        }
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        TEST_IGNORE();
    }

    /* Un Masking GPIO 5 RISE Interrupt */
    Pmic_irqGpioMaskIntr(pPmicCoreHandle,
                         PMIC_TPS6594X_IRQ_GPIO_5_INT_MASK_NUM,
                         PMIC_IRQ_UNMASK,
                         PMIC_IRQ_GPIO_RISE_INT_TYPE);

    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioSetValue(pPmicCoreHandle, pin, pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioSetIntr(pPmicCoreHandle, pin , intrType, maskPol);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pinValue = PMIC_GPIO_HIGH;
    pmicStatus = Pmic_gpioSetValue(pPmicCoreHandle, pin, pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    while(1U)
    {
        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        if((PMIC_ST_SUCCESS == pmicStatus) &&
           ((errStat.intStatus[PMIC_TPS6594X_GPIO5_INT/32U] & 
             (1U << (PMIC_TPS6594X_GPIO5_INT % 32U))) != 0U))
        {
            while(PMIC_TPS6594X_GPIO5_INT != irqNum)
            {
                pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                                     &errStat,
                                                     &irqNum);
            }

            if(PMIC_ST_SUCCESS == pmicStatus)
            {
                /* clear the interrupt */
                pmicStatus = Pmic_irqClrErrStatus(pPmicCoreHandle,
                                                  PMIC_TPS6594X_GPIO5_INT);
                break;
            }
        }
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
}

/*!
 * \brief   Test to verify GPIO6 fall interrupt
 */
static void test_pmic_gpio6_testFall_interrupt(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    uint8_t pin               = 6U;
    uint8_t pinValue          = PMIC_GPIO_HIGH;
    uint8_t intrType          = PMIC_GPIO_FALL_INTERRUPT;
    uint8_t maskPol           = PMIC_GPIO_POL_LOW;
    Pmic_IrqStatus_t errStat  = {0U};
    bool clearIRQ             = false;
    uint8_t  irqNum           = 0U;
    Pmic_GpioCfg_t gpioCfg    =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT | PMIC_GPIO_CFG_DIR_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_GPIO_PINFUNC_GPIO,
        PMIC_GPIO_HIGH
    };

    test_pmic_print_unity_testcase_info(6244,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        if(J721E_LEO_PMICB_DEVICE == pmic_device_info)
        {
            /*
             * GPIO6 pin of PMIC-B is connected to 'LEOA_SDATA' and receives
             * on J721EVM causing hang, when programming fall interrupt signal.
             */
            TEST_IGNORE();
        }
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        TEST_IGNORE();
    }

    /* Un Masking GPIO 6 FALL Interrupt */
    Pmic_irqGpioMaskIntr(pPmicCoreHandle,
                         PMIC_TPS6594X_IRQ_GPIO_6_INT_MASK_NUM,
                         PMIC_IRQ_UNMASK,
                         PMIC_IRQ_GPIO_FALL_INT_TYPE);

    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioSetValue(pPmicCoreHandle, pin, pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioSetIntr(pPmicCoreHandle, pin , intrType, maskPol);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pinValue = PMIC_GPIO_LOW;
    pmicStatus = Pmic_gpioSetValue(pPmicCoreHandle, pin, pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    while(1U)
    {
        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        if((PMIC_ST_SUCCESS == pmicStatus) &&
           ((errStat.intStatus[PMIC_TPS6594X_GPIO6_INT/32U] & 
             (1U << (PMIC_TPS6594X_GPIO6_INT % 32U))) != 0U))
        {
            while(PMIC_TPS6594X_GPIO6_INT != irqNum)
            {
                pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                                     &errStat,
                                                     &irqNum);
            }

            if(PMIC_ST_SUCCESS == pmicStatus)
            {
                /* clear the interrupt */
                pmicStatus = Pmic_irqClrErrStatus(pPmicCoreHandle,
                                                  PMIC_TPS6594X_GPIO6_INT);
                break;
            }
        }
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
}

/*!
 * \brief   Test to verify GPIO6 rise interrupt
 */
static void test_pmic_gpio6_testRise_interrupt(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    uint8_t pin               = 6U;
    uint8_t pinValue          = PMIC_GPIO_LOW;
    uint8_t intrType          = PMIC_GPIO_RISE_INTERRUPT;
    uint8_t maskPol           = PMIC_GPIO_POL_HIGH;
    Pmic_IrqStatus_t errStat  = {0U};
    bool clearIRQ             = false;
    uint8_t  irqNum           = 0U;
    Pmic_GpioCfg_t gpioCfg    =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT | PMIC_GPIO_CFG_DIR_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_GPIO_PINFUNC_GPIO,
        PMIC_GPIO_HIGH
    };

    test_pmic_print_unity_testcase_info(6245,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        if(J721E_LEO_PMICB_DEVICE == pmic_device_info)
        {
            /*
             * GPIO6 pin of PMIC-B is connected to 'LEOA_SDATA' and receives
             * on J721EVM causing hang, when programming rise interrupt signal.
             */
            TEST_IGNORE();
        }
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        TEST_IGNORE();
    }

    /* Un Masking GPIO 6 RISE Interrupt */
    Pmic_irqGpioMaskIntr(pPmicCoreHandle,
                         PMIC_TPS6594X_IRQ_GPIO_6_INT_MASK_NUM,
                         PMIC_IRQ_UNMASK,
                         PMIC_IRQ_GPIO_RISE_INT_TYPE);

    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioSetValue(pPmicCoreHandle, pin, pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioSetIntr(pPmicCoreHandle, pin , intrType, maskPol);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pinValue = PMIC_GPIO_HIGH;
    pmicStatus = Pmic_gpioSetValue(pPmicCoreHandle, pin, pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    while(1U)
    {
        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        if((PMIC_ST_SUCCESS == pmicStatus) &&
           ((errStat.intStatus[PMIC_TPS6594X_GPIO6_INT/32U] & 
             (1U << (PMIC_TPS6594X_GPIO6_INT % 32U))) != 0U))
        {
            while(PMIC_TPS6594X_GPIO6_INT != irqNum)
            {
                pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                                     &errStat,
                                                     &irqNum);
            }

            if(PMIC_ST_SUCCESS == pmicStatus)
            {
                /* clear the interrupt */
                pmicStatus = Pmic_irqClrErrStatus(pPmicCoreHandle,
                                                  PMIC_TPS6594X_GPIO6_INT);
                break;
            }
        }
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
}

/*!
 * \brief   Test to verify GPIO7 fall interrupt
 */
static void test_pmic_gpio7_testFall_interrupt(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    uint8_t pin               = 7U;
    uint8_t pinValue          = PMIC_GPIO_HIGH;
    uint8_t intrType          = PMIC_GPIO_FALL_INTERRUPT;
    uint8_t maskPol           = PMIC_GPIO_POL_LOW;
    Pmic_IrqStatus_t errStat  = {0U};
    bool clearIRQ             = false;
    uint8_t  irqNum           = 0U;
    Pmic_GpioCfg_t gpioCfg    =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT | PMIC_GPIO_CFG_DIR_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_GPIO_PINFUNC_GPIO,
        PMIC_GPIO_HIGH
    };

    test_pmic_print_unity_testcase_info(6246,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        if(J721E_LEO_PMICA_DEVICE == pmic_device_info)
        {
            /*
             * GPIO7 pin of PMIC-A is connected to 'SOC_SAFETY_ERRZ'
             * on J721EVM board causing hang, when programming fall interrupt.
             */
            TEST_IGNORE();
        }
        if(J721E_LEO_PMICB_DEVICE == pmic_device_info)
        {
            /*
             * GPIO7 pin of PMIC-B is connected to 'EN_RAM0V85_LDO'
             * on J721EVM board causing hang, when programming fall interrupt.
             */
            TEST_IGNORE();
        }
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        TEST_IGNORE();
    }

    /* Un Masking GPIO 7 FALL Interrupt */
    Pmic_irqGpioMaskIntr(pPmicCoreHandle,
                         PMIC_TPS6594X_IRQ_GPIO_7_INT_MASK_NUM,
                         PMIC_IRQ_UNMASK,
                         PMIC_IRQ_GPIO_FALL_INT_TYPE);

    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioSetValue(pPmicCoreHandle, pin, pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioSetIntr(pPmicCoreHandle, pin , intrType, maskPol);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pinValue = PMIC_GPIO_LOW;
    pmicStatus = Pmic_gpioSetValue(pPmicCoreHandle, pin, pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    while(1U)
    {
        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        if((PMIC_ST_SUCCESS == pmicStatus) &&
           ((errStat.intStatus[PMIC_TPS6594X_GPIO7_INT/32U] & 
             (1U << (PMIC_TPS6594X_GPIO7_INT % 32U))) != 0U))
        {
            while(PMIC_TPS6594X_GPIO7_INT != irqNum)
            {
                pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                                     &errStat,
                                                     &irqNum);
            }

            if(PMIC_ST_SUCCESS == pmicStatus)
            {
                /* clear the interrupt */
                pmicStatus = Pmic_irqClrErrStatus(pPmicCoreHandle,
                                                  PMIC_TPS6594X_GPIO7_INT);
                break;
            }
        }
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
}

/*!
 * \brief   Test to verify GPIO7 rise interrupt
 */
static void test_pmic_gpio7_testRise_interrupt(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    uint8_t pin               = 7U;
    uint8_t pinValue          = PMIC_GPIO_LOW;
    uint8_t intrType          = PMIC_GPIO_RISE_INTERRUPT;
    uint8_t maskPol           = PMIC_GPIO_POL_HIGH;
    Pmic_IrqStatus_t errStat  = {0U};
    bool clearIRQ             = false;
    uint8_t  irqNum           = 0U;
    Pmic_GpioCfg_t gpioCfg    =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT | PMIC_GPIO_CFG_DIR_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_GPIO_PINFUNC_GPIO,
        PMIC_GPIO_HIGH
    };

    test_pmic_print_unity_testcase_info(6247,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        if(J721E_LEO_PMICA_DEVICE == pmic_device_info)
        {
            /*
             * GPIO7 pin of PMIC-A is connected to 'SOC_SAFETY_ERRZ'
             * on J721EVM board causing hang, when programming rise interrupt.
             */
            TEST_IGNORE();
        }
        if(J721E_LEO_PMICB_DEVICE == pmic_device_info)
        {
            /*
             * GPIO7 pin of PMIC-B is connected to 'EN_RAM0V85_LDO'
             * on J721EVM board causing hang, when programming rise interrupt.
             */
            TEST_IGNORE();
        }
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        TEST_IGNORE();
    }

    /* Un Masking GPIO 7 RISE Interrupt */
    Pmic_irqGpioMaskIntr(pPmicCoreHandle,
                         PMIC_TPS6594X_IRQ_GPIO_7_INT_MASK_NUM,
                         PMIC_IRQ_UNMASK,
                         PMIC_IRQ_GPIO_RISE_INT_TYPE);

    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioSetValue(pPmicCoreHandle, pin, pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioSetIntr(pPmicCoreHandle, pin , intrType, maskPol);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pinValue = PMIC_GPIO_HIGH;
    pmicStatus = Pmic_gpioSetValue(pPmicCoreHandle, pin, pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    while(1U)
    {
        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        if((PMIC_ST_SUCCESS == pmicStatus) &&
           ((errStat.intStatus[PMIC_TPS6594X_GPIO7_INT/32U] & 
             (1U << (PMIC_TPS6594X_GPIO7_INT % 32U))) != 0U))
        {
            while(PMIC_TPS6594X_GPIO7_INT != irqNum)
            {
                pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                                     &errStat,
                                                     &irqNum);
            }

            if(PMIC_ST_SUCCESS == pmicStatus)
            {
                /* clear the interrupt */
                pmicStatus = Pmic_irqClrErrStatus(pPmicCoreHandle,
                                                  PMIC_TPS6594X_GPIO7_INT);
                break;
            }
        }
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
}

/*!
 * \brief   Test to verify GPIO8 fall interrupt
 */
static void test_pmic_gpio8_testFall_interrupt(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    uint8_t pin               = 8U;
    uint8_t pinValue          = PMIC_GPIO_HIGH;
    uint8_t intrType          = PMIC_GPIO_FALL_INTERRUPT;
    uint8_t maskPol           = PMIC_GPIO_POL_LOW;
    Pmic_IrqStatus_t errStat  = {0U};
    bool clearIRQ             = false;
    uint8_t  irqNum           = 0U;
    Pmic_GpioCfg_t gpioCfg    =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT | PMIC_GPIO_CFG_DIR_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_GPIO_PINFUNC_GPIO,
        PMIC_GPIO_HIGH
    };

    test_pmic_print_unity_testcase_info(6248,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        if(J721E_LEO_PMICB_DEVICE == pmic_device_info)
        {
            /*
             * GPIO8 pin of PMIC-B is connected to 'EN_PHYCORE_LDO'
             * on J721EVM causing hang, when programming fall interrupt signal.
             */
            TEST_IGNORE();
        }
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        TEST_IGNORE();
    }

    /* Un Masking GPIO 8 FALL Interrupt */
    Pmic_irqGpioMaskIntr(pPmicCoreHandle,
                         PMIC_TPS6594X_IRQ_GPIO_8_INT_MASK_NUM,
                         PMIC_IRQ_UNMASK,
                         PMIC_IRQ_GPIO_FALL_INT_TYPE);

    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioSetValue(pPmicCoreHandle, pin, pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioSetIntr(pPmicCoreHandle, pin , intrType, maskPol);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pinValue = PMIC_GPIO_LOW;
    pmicStatus = Pmic_gpioSetValue(pPmicCoreHandle, pin, pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    while(1U)
    {
        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        if((PMIC_ST_SUCCESS == pmicStatus) &&
           ((errStat.intStatus[PMIC_TPS6594X_GPIO8_INT/32U] & 
             (1U << (PMIC_TPS6594X_GPIO8_INT % 32U))) != 0U))
        {
            while(PMIC_TPS6594X_GPIO8_INT != irqNum)
            {
                pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                                     &errStat,
                                                     &irqNum);
            }

            if(PMIC_ST_SUCCESS == pmicStatus)
            {
                /* clear the interrupt */
                pmicStatus = Pmic_irqClrErrStatus(pPmicCoreHandle,
                                                  PMIC_TPS6594X_GPIO8_INT);
                break;
            }
        }
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
}

/*!
 * \brief   Test to verify GPIO8 rise interrupt
 */
static void test_pmic_gpio8_testRise_interrupt(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    uint8_t pin               = 8U;
    uint8_t pinValue          = PMIC_GPIO_LOW;
    uint8_t intrType          = PMIC_GPIO_RISE_INTERRUPT;
    uint8_t maskPol           = PMIC_GPIO_POL_HIGH;
    Pmic_IrqStatus_t errStat  = {0U};
    bool clearIRQ             = false;
    uint8_t  irqNum           = 0U;
    Pmic_GpioCfg_t gpioCfg    =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT | PMIC_GPIO_CFG_DIR_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_GPIO_PINFUNC_GPIO,
        PMIC_GPIO_HIGH
    };

    test_pmic_print_unity_testcase_info(6249,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        if(J721E_LEO_PMICB_DEVICE == pmic_device_info)
        {
            /*
             * GPIO8 pin of PMIC-B is connected to 'EN_PHYCORE_LDO'
             * on J721EVM causing hang, when programming fall interrupt signal.
             */
            TEST_IGNORE();
        }
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        TEST_IGNORE();
    }

    /* Un Masking GPIO 8 RISE Interrupt */
    Pmic_irqGpioMaskIntr(pPmicCoreHandle,
                         PMIC_TPS6594X_IRQ_GPIO_8_INT_MASK_NUM,
                         PMIC_IRQ_UNMASK,
                         PMIC_IRQ_GPIO_RISE_INT_TYPE);

    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioSetValue(pPmicCoreHandle, pin, pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioSetIntr(pPmicCoreHandle, pin , intrType, maskPol);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pinValue = PMIC_GPIO_HIGH;
    pmicStatus = Pmic_gpioSetValue(pPmicCoreHandle, pin, pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    while(1U)
    {
        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        if((PMIC_ST_SUCCESS == pmicStatus) &&
           ((errStat.intStatus[PMIC_TPS6594X_GPIO8_INT/32U] & 
             (1U << (PMIC_TPS6594X_GPIO8_INT % 32U))) != 0U))
        {
            while(PMIC_TPS6594X_GPIO8_INT != irqNum)
            {
                pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                                     &errStat,
                                                     &irqNum);
            }

            if(PMIC_ST_SUCCESS == pmicStatus)
            {
                /* clear the interrupt */
                pmicStatus = Pmic_irqClrErrStatus(pPmicCoreHandle,
                                                  PMIC_TPS6594X_GPIO8_INT);
                break;
            }
        }
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
}

/*!
 * \brief   Test to verify GPIO9 fall interrupt
 */
static void test_pmic_gpio9_testFall_interrupt(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    uint8_t pin               = 9U;
    uint8_t pinValue          = PMIC_GPIO_HIGH;
    uint8_t intrType          = PMIC_GPIO_FALL_INTERRUPT;
    uint8_t maskPol           = PMIC_GPIO_POL_LOW;
    Pmic_IrqStatus_t errStat  = {0U};
    bool clearIRQ             = false;
    uint8_t  irqNum           = 0U;
    Pmic_GpioCfg_t gpioCfg    =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT | PMIC_GPIO_CFG_DIR_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_GPIO_PINFUNC_GPIO,
        PMIC_GPIO_HIGH
    };

    test_pmic_print_unity_testcase_info(6250,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        if(J721E_LEO_PMICA_DEVICE == pmic_device_info)
        {
            /*
             * GPIO9 pin of PMIC-A is connected to 'GND'
             * on J721EVM board causing hang, when programming fall interrupt.
             */
            TEST_IGNORE();
        }
        if(J721E_LEO_PMICB_DEVICE == pmic_device_info)
        {
            /*
             * GPIO9 pin of PMIC-B is connected to 'EN_VPP1V8_LDO'
             * on J721EVM board causing hang, when programming fall interrupt.
             */
            TEST_IGNORE();
        }
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        TEST_IGNORE();
    }

    /* Un Masking GPIO 9 FALL Interrupt */
    Pmic_irqGpioMaskIntr(pPmicCoreHandle,
                         PMIC_TPS6594X_IRQ_GPIO_9_INT_MASK_NUM,
                         PMIC_IRQ_UNMASK,
                         PMIC_IRQ_GPIO_FALL_INT_TYPE);

    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioSetValue(pPmicCoreHandle, pin, pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioSetIntr(pPmicCoreHandle, pin , intrType, maskPol);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pinValue = PMIC_GPIO_LOW;
    pmicStatus = Pmic_gpioSetValue(pPmicCoreHandle, pin, pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    while(1U)
    {
        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        if((PMIC_ST_SUCCESS == pmicStatus) &&
           ((errStat.intStatus[PMIC_TPS6594X_GPIO9_INT/32U] & 
             (1U << (PMIC_TPS6594X_GPIO9_INT % 32U))) != 0U))
        {
            while(PMIC_TPS6594X_GPIO9_INT != irqNum)
            {
                pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                                     &errStat,
                                                     &irqNum);
            }

            if(PMIC_ST_SUCCESS == pmicStatus)
            {
                /* clear the interrupt */
                pmicStatus = Pmic_irqClrErrStatus(pPmicCoreHandle,
                                                  PMIC_TPS6594X_GPIO9_INT);
                break;
            }
        }
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
}

/*!
 * \brief   Test to verify GPIO9 rise interrupt
 */
static void test_pmic_gpio9_testRise_interrupt(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    uint8_t pin               = 9U;
    uint8_t pinValue          = PMIC_GPIO_LOW;
    uint8_t intrType          = PMIC_GPIO_RISE_INTERRUPT;
    uint8_t maskPol           = PMIC_GPIO_POL_HIGH;
    Pmic_IrqStatus_t errStat  = {0U};
    bool clearIRQ             = false;
    uint8_t  irqNum           = 0U;
    Pmic_GpioCfg_t gpioCfg    =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT | PMIC_GPIO_CFG_DIR_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_GPIO_PINFUNC_GPIO,
        PMIC_GPIO_HIGH
    };

    test_pmic_print_unity_testcase_info(6251,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        if(J721E_LEO_PMICA_DEVICE == pmic_device_info)
        {
            /*
             * GPIO9 pin of PMIC-A is connected to 'GND'
             * on J721EVM board causing hang, when programming rise interrupt.
             */
            TEST_IGNORE();
        }
        if(J721E_LEO_PMICB_DEVICE == pmic_device_info)
        {
            /*
             * GPIO9 pin of PMIC-B is connected to 'EN_VPP1V8_LDO'
             * on J721EVM board causing hang, when programming rise interrupt.
             */
            TEST_IGNORE();
        }
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        TEST_IGNORE();
    }

    /* Un Masking GPIO 9 RISE Interrupt */
    Pmic_irqGpioMaskIntr(pPmicCoreHandle,
                         PMIC_TPS6594X_IRQ_GPIO_9_INT_MASK_NUM,
                         PMIC_IRQ_UNMASK,
                         PMIC_IRQ_GPIO_RISE_INT_TYPE);

    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioSetValue(pPmicCoreHandle, pin, pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioSetIntr(pPmicCoreHandle, pin , intrType, maskPol);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pinValue = PMIC_GPIO_HIGH;
    pmicStatus = Pmic_gpioSetValue(pPmicCoreHandle, pin, pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    while(1U)
    {
        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        if((PMIC_ST_SUCCESS == pmicStatus) &&
           ((errStat.intStatus[PMIC_TPS6594X_GPIO9_INT/32U] & 
             (1U << (PMIC_TPS6594X_GPIO9_INT % 32U))) != 0U))
        {
            while(PMIC_TPS6594X_GPIO9_INT != irqNum)
            {
                pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                                     &errStat,
                                                     &irqNum);
            }

            if(PMIC_ST_SUCCESS == pmicStatus)
            {
                /* clear the interrupt */
                pmicStatus = Pmic_irqClrErrStatus(pPmicCoreHandle,
                                                  PMIC_TPS6594X_GPIO9_INT);
                break;
            }
        }
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
}

/*!
 * \brief   Test to verify GPIO10 fall interrupt
 */
static void test_pmic_gpio10_testFall_interrupt(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    uint8_t pin               = 10U;
    uint8_t pinValue          = PMIC_GPIO_HIGH;
    uint8_t intrType          = PMIC_GPIO_FALL_INTERRUPT;
    uint8_t maskPol           = PMIC_GPIO_POL_LOW;
    Pmic_IrqStatus_t errStat  = {0U};
    bool clearIRQ             = false;
    uint8_t  irqNum           = 0U;
    Pmic_GpioCfg_t gpioCfg    =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT | PMIC_GPIO_CFG_DIR_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_GPIO_PINFUNC_GPIO,
        PMIC_GPIO_HIGH
    };

    if(J721E_LEO_PMICB_DEVICE == pmic_device_info)
    {
        gpioCfg.validParams = PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT |
                              PMIC_GPIO_CFG_DIR_VALID_SHIFT |
                              PMIC_GPIO_CFG_OD_VALID_SHIFT |
                              PMIC_GPIO_CFG_PULL_VALID_SHIFT;
        gpioCfg.outputSignalType = PMIC_GPIO_PUSH_PULL_OUTPUT;
        gpioCfg.pullCtrl = PMIC_GPIO_PULL_UP;
    }

    test_pmic_print_unity_testcase_info(6252,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        if(J721E_LEO_PMICA_DEVICE == pmic_device_info)
        {
            /*
             * GPIO10 pin of PMIC-A is connected to 'PMIC_POWER_EN1'
             * on J721EVM board causing hang, when programming fall interrupt.
             */
            TEST_IGNORE();
        }
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        TEST_IGNORE();
    }

    /* Un Masking GPIO 10 FALL Interrupt */
    Pmic_irqGpioMaskIntr(pPmicCoreHandle,
                         PMIC_TPS6594X_IRQ_GPIO_10_INT_MASK_NUM,
                         PMIC_IRQ_UNMASK,
                         PMIC_IRQ_GPIO_FALL_INT_TYPE);

    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioSetValue(pPmicCoreHandle, pin, pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioSetIntr(pPmicCoreHandle, pin , intrType, maskPol);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pinValue = PMIC_GPIO_LOW;
    pmicStatus = Pmic_gpioSetValue(pPmicCoreHandle, pin, pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    while(1U)
    {
        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        if((PMIC_ST_SUCCESS == pmicStatus) &&
           ((errStat.intStatus[PMIC_TPS6594X_GPIO10_INT/32U] & 
             (1U << (PMIC_TPS6594X_GPIO10_INT % 32U))) != 0U))
        {
            while(PMIC_TPS6594X_GPIO10_INT != irqNum)
            {
                pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                                     &errStat,
                                                     &irqNum);
            }

            if(PMIC_ST_SUCCESS == pmicStatus)
            {
                /* clear the interrupt */
                pmicStatus = Pmic_irqClrErrStatus(pPmicCoreHandle,
                                                  PMIC_TPS6594X_GPIO10_INT);
                break;
            }
        }
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
}

/*!
 * \brief   Test to verify GPIO10 rise interrupt
 */
static void test_pmic_gpio10_testRise_interrupt(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    uint8_t pin               = 10U;
    uint8_t pinValue          = PMIC_GPIO_LOW;
    uint8_t intrType          = PMIC_GPIO_RISE_INTERRUPT;
    uint8_t maskPol           = PMIC_GPIO_POL_HIGH;
    Pmic_IrqStatus_t errStat  = {0U};
    bool clearIRQ             = false;
    uint8_t  irqNum           = 0U;
    Pmic_GpioCfg_t gpioCfg    =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT | PMIC_GPIO_CFG_DIR_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_GPIO_PINFUNC_GPIO,
        PMIC_GPIO_HIGH
    };

    if(J721E_LEO_PMICB_DEVICE == pmic_device_info)
    {
        gpioCfg.validParams = PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT |
                              PMIC_GPIO_CFG_DIR_VALID_SHIFT |
                              PMIC_GPIO_CFG_OD_VALID_SHIFT |
                              PMIC_GPIO_CFG_PULL_VALID_SHIFT;
        gpioCfg.outputSignalType = PMIC_GPIO_PUSH_PULL_OUTPUT;
        gpioCfg.pullCtrl = PMIC_GPIO_PULL_UP;
    }

    test_pmic_print_unity_testcase_info(6253,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        if(J721E_LEO_PMICA_DEVICE == pmic_device_info)
        {
            /*
             * GPIO10 pin of PMIC-A is connected to 'PMIC_POWER_EN1'
             * on J721EVM board causing hang, when programming rise interrupt.
             */
            TEST_IGNORE();
        }
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        TEST_IGNORE();
    }

    /* Un Masking GPIO 10 RISE Interrupt */
    Pmic_irqGpioMaskIntr(pPmicCoreHandle,
                         PMIC_TPS6594X_IRQ_GPIO_10_INT_MASK_NUM,
                         PMIC_IRQ_UNMASK,
                         PMIC_IRQ_GPIO_RISE_INT_TYPE);

    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioSetValue(pPmicCoreHandle, pin, pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioSetIntr(pPmicCoreHandle, pin , intrType, maskPol);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pinValue = PMIC_GPIO_HIGH;
    pmicStatus = Pmic_gpioSetValue(pPmicCoreHandle, pin, pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    while(1U)
    {
        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        if((PMIC_ST_SUCCESS == pmicStatus) &&
           ((errStat.intStatus[PMIC_TPS6594X_GPIO10_INT/32U] & 
             (1U << (PMIC_TPS6594X_GPIO10_INT % 32U))) != 0U))
        {
            while(PMIC_TPS6594X_GPIO10_INT != irqNum)
            {
                pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                                     &errStat,
                                                     &irqNum);
            }

            if(PMIC_ST_SUCCESS == pmicStatus)
            {
                /* clear the interrupt */
                pmicStatus = Pmic_irqClrErrStatus(pPmicCoreHandle,
                                                  PMIC_TPS6594X_GPIO10_INT);
                break;
            }
        }
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
}

/*!
 * \brief   Test to verify GPIO11 fall interrupt
 */
static void test_pmic_gpio11_testFall_interrupt(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    uint8_t pin               = 11U;
    uint8_t pinValue          = PMIC_GPIO_HIGH;
    uint8_t intrType          = PMIC_GPIO_FALL_INTERRUPT;
    uint8_t maskPol           = PMIC_GPIO_POL_LOW;
    Pmic_IrqStatus_t errStat  = {0U};
    bool clearIRQ             = false;
    uint8_t  irqNum           = 0U;
    Pmic_GpioCfg_t gpioCfg    =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT | PMIC_GPIO_CFG_DIR_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_GPIO_PINFUNC_GPIO,
        PMIC_GPIO_HIGH
    };

    test_pmic_print_unity_testcase_info(6254,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        if(J721E_LEO_PMICA_DEVICE == pmic_device_info)
        {
            /*
             * GPIO11 pin of PMIC-A is connected to 'H_SOC_PORz'
             * on J721EVM board causing hang, when programming fall interrupt.
             */
            TEST_IGNORE();
        }
        if(J721E_LEO_PMICB_DEVICE == pmic_device_info)
        {
            /*
             * GPIO11 pin of PMIC-B is connected to 'EN_3V3IO_LDSW'
             * on J721EVM board causing hang, when programming fall interrupt.
             */
            TEST_IGNORE();
        }
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        TEST_IGNORE();
    }

    /* Un Masking GPIO 11 FALL Interrupt */
    Pmic_irqGpioMaskIntr(pPmicCoreHandle,
                         PMIC_TPS6594X_IRQ_GPIO_11_INT_MASK_NUM,
                         PMIC_IRQ_UNMASK,
                         PMIC_IRQ_GPIO_FALL_INT_TYPE);

    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioSetValue(pPmicCoreHandle, pin, pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioSetIntr(pPmicCoreHandle, pin , intrType, maskPol);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pinValue = PMIC_GPIO_LOW;
    pmicStatus = Pmic_gpioSetValue(pPmicCoreHandle, pin, pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    while(1U)
    {
        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        if((PMIC_ST_SUCCESS == pmicStatus) &&
           ((errStat.intStatus[PMIC_TPS6594X_GPIO11_INT/32U] & 
             (1U << (PMIC_TPS6594X_GPIO11_INT % 32U))) != 0U))
        {
            while(PMIC_TPS6594X_GPIO11_INT != irqNum)
            {
                pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                                     &errStat,
                                                     &irqNum);
            }

            if(PMIC_ST_SUCCESS == pmicStatus)
            {
                /* clear the interrupt */
                pmicStatus = Pmic_irqClrErrStatus(pPmicCoreHandle,
                                                  PMIC_TPS6594X_GPIO11_INT);
                break;
            }
        }
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
}

/*!
 * \brief   Test to verify GPIO11 rise interrupt
 */
static void test_pmic_gpio11_testRise_interrupt(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    uint8_t pin               = 11U;
    uint8_t pinValue          = PMIC_GPIO_LOW;
    uint8_t intrType          = PMIC_GPIO_RISE_INTERRUPT;
    uint8_t maskPol           = PMIC_GPIO_POL_HIGH;
    Pmic_IrqStatus_t errStat  = {0U};
    bool clearIRQ             = false;
    uint8_t  irqNum           = 0U;
    Pmic_GpioCfg_t gpioCfg    =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT | PMIC_GPIO_CFG_DIR_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_GPIO_PINFUNC_GPIO,
        PMIC_GPIO_HIGH
    };

    test_pmic_print_unity_testcase_info(6255,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        if(J721E_LEO_PMICA_DEVICE == pmic_device_info)
        {
            /*
             * GPIO11 pin of PMIC-A is connected to 'H_SOC_PORz'
             * on J721EVM board causing hang, when programming rise interrupt.
             */
            TEST_IGNORE();
        }
        if(J721E_LEO_PMICB_DEVICE == pmic_device_info)
        {
            /*
             * GPIO11 pin of PMIC-B is connected to 'EN_3V3IO_LDSW'
             * on J721EVM board causing hang, when programming rise interrupt.
             */
            TEST_IGNORE();
        }
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        TEST_IGNORE();
    }

    /* Un Masking GPIO 11 RISE Interrupt */
    Pmic_irqGpioMaskIntr(pPmicCoreHandle,
                         PMIC_TPS6594X_IRQ_GPIO_11_INT_MASK_NUM,
                         PMIC_IRQ_UNMASK,
                         PMIC_IRQ_GPIO_RISE_INT_TYPE);

    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioSetValue(pPmicCoreHandle, pin, pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioSetIntr(pPmicCoreHandle, pin , intrType, maskPol);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pinValue = PMIC_GPIO_HIGH;
    pmicStatus = Pmic_gpioSetValue(pPmicCoreHandle, pin, pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    while(1U)
    {
        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        if((PMIC_ST_SUCCESS == pmicStatus) &&
           ((errStat.intStatus[PMIC_TPS6594X_GPIO11_INT/32U] & 
             (1U << (PMIC_TPS6594X_GPIO11_INT % 32U))) != 0U))
        {
            while(PMIC_TPS6594X_GPIO11_INT != irqNum)
            {
                pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                                     &errStat,
                                                     &irqNum);
            }

            if(PMIC_ST_SUCCESS == pmicStatus)
            {
                /* clear the interrupt */
                pmicStatus = Pmic_irqClrErrStatus(pPmicCoreHandle,
                                                  PMIC_TPS6594X_GPIO11_INT);
                break;
            }
        }
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
}

/*!
 * \brief   Parameter validation for handle
 */
static void test_pmic_gpio_intr_prmValTest_handle(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    uint8_t pin               = 3U;
    uint8_t pinValue          = PMIC_GPIO_HIGH;
    uint8_t intrType          = PMIC_GPIO_FALL_INTERRUPT;
    uint8_t maskPol           = PMIC_GPIO_POL_LOW;
    Pmic_GpioCfg_t gpioCfg    =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT | PMIC_GPIO_CFG_DIR_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_GPIO_PINFUNC_GPIO,
        PMIC_GPIO_HIGH
    };

    test_pmic_print_unity_testcase_info(6256,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioSetValue(pPmicCoreHandle, pin, pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioSetIntr(NULL, pin , intrType, maskPol);
    TEST_ASSERT_EQUAL(pmicStatus, PMIC_ST_ERR_INV_HANDLE);
}

/*!
 * \brief   Parameter validation for pin
 */
static void test_pmic_gpio_intr_prmValTest_pin(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    uint8_t pin               = 12U;
    uint8_t pinValue          = PMIC_GPIO_HIGH;
    uint8_t intrType          = PMIC_GPIO_FALL_INTERRUPT;
    uint8_t maskPol           = PMIC_GPIO_POL_LOW;
    Pmic_GpioCfg_t gpioCfg    =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT | PMIC_GPIO_CFG_DIR_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_GPIO_PINFUNC_GPIO,
        PMIC_GPIO_HIGH
    };

    test_pmic_print_unity_testcase_info(6257,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, 1U, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioSetValue(pPmicCoreHandle, 1U, pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioSetIntr(pPmicCoreHandle, pin, intrType, maskPol);
    TEST_ASSERT_EQUAL(pmicStatus, PMIC_ST_ERR_INV_PARAM);
}

/*!
 * \brief   Parameter validation for intrType
 */
static void test_pmic_gpio_intr_prmValTest_intrType(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    uint8_t pin               = 1U;
    uint8_t pinValue          = PMIC_GPIO_HIGH;
    uint8_t intrType          = 0x4U;
    uint8_t maskPol           = PMIC_GPIO_POL_LOW;
    Pmic_GpioCfg_t gpioCfg    =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT | PMIC_GPIO_CFG_DIR_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_GPIO_PINFUNC_GPIO,
        PMIC_GPIO_HIGH
    };

    test_pmic_print_unity_testcase_info(6258,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioSetValue(pPmicCoreHandle, pin, pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioSetIntr(pPmicCoreHandle, pin, intrType, maskPol);
    TEST_ASSERT_EQUAL(pmicStatus, PMIC_ST_ERR_INV_PARAM);
}

/*!
 * \brief   Parameter validation for maskPol
 */
static void test_pmic_gpio_intr_prmValTest_maskPol(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    uint8_t pin               = 2U;
    uint8_t pinValue          = PMIC_GPIO_HIGH;
    uint8_t intrType          = PMIC_GPIO_RISE_INTERRUPT;
    uint8_t maskPol           = 0x2U;
    Pmic_GpioCfg_t gpioCfg    =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT | PMIC_GPIO_CFG_DIR_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_GPIO_PINFUNC_GPIO,
        PMIC_GPIO_HIGH
    };

    test_pmic_print_unity_testcase_info(6259,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioSetValue(pPmicCoreHandle, pin, pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioSetIntr(pPmicCoreHandle, pin , intrType, maskPol);
    TEST_ASSERT_EQUAL(pmicStatus, PMIC_ST_ERR_INV_PARAM);
}

/*!
 * \brief   Masking all interrupts test
 */
static void test_pmic_gpio_intr_irqMaskAll_interrupt(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    uint8_t pin               = 1U;
    uint8_t pinValue          = PMIC_GPIO_HIGH;
    uint8_t intrType          = PMIC_GPIO_FALL_INTERRUPT;
    uint8_t maskPol           = PMIC_GPIO_POL_LOW;
    Pmic_IrqStatus_t errStat  = {0U};
    bool clearIRQ             = false;
    Pmic_GpioCfg_t gpioCfg    =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT | PMIC_GPIO_CFG_DIR_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_GPIO_PINFUNC_GPIO,
        PMIC_GPIO_HIGH
    };

    if(J721E_LEO_PMICB_DEVICE == pmic_device_info)
    {
        gpioCfg.validParams = PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT |
                              PMIC_GPIO_CFG_DIR_VALID_SHIFT |
                              PMIC_GPIO_CFG_OD_VALID_SHIFT |
                              PMIC_GPIO_CFG_PULL_VALID_SHIFT;
        gpioCfg.outputSignalType = PMIC_GPIO_PUSH_PULL_OUTPUT;
        gpioCfg.pullCtrl = PMIC_GPIO_PULL_UP;
    }

    test_pmic_print_unity_testcase_info(7374,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        TEST_IGNORE();
    }

    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioSetValue(pPmicCoreHandle, pin, pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioSetIntr(pPmicCoreHandle, pin , intrType, maskPol);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    /* Masking All GPIO Interrupts */
    pmicStatus = Pmic_irqGpioMaskIntr(pPmicCoreHandle,
                                      PMIC_IRQ_GPIO_ALL_INT_MASK_NUM,
                                      PMIC_IRQ_MASK,
                                      PMIC_IRQ_GPIO_RISE_FALL_INT_TYPE);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    /* Masking All Interrupts */
    pmicStatus = Pmic_irqMaskIntr(pPmicCoreHandle,
                                  PMIC_IRQ_ALL,
                                  PMIC_IRQ_MASK);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pinValue = PMIC_GPIO_LOW;
    pmicStatus = Pmic_gpioSetValue(pPmicCoreHandle, pin, pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    if((0U == errStat.intStatus[0U]) &&
       (0U == errStat.intStatus[1U]) &&
       (0U == errStat.intStatus[2U]) &&
       (0U == errStat.intStatus[3U]))
    {
        pmicStatus = PMIC_ST_SUCCESS;
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
}

/*!
 * \brief   UnMasking all interrupts test
 */
static void test_pmic_gpio_intr_irqUnMaskAll_interrupt(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    uint8_t pin               = 1U;
    uint8_t pinValue          = PMIC_GPIO_HIGH;
    uint8_t intrType          = PMIC_GPIO_FALL_INTERRUPT;
    uint8_t maskPol           = PMIC_GPIO_POL_LOW;
    Pmic_IrqStatus_t errStat  = {0U};
    bool clearIRQ             = false;
    uint8_t  irqNum           = 0U;
    Pmic_GpioCfg_t gpioCfg    =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT | PMIC_GPIO_CFG_DIR_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_GPIO_PINFUNC_GPIO,
        PMIC_GPIO_HIGH
    };

    test_pmic_print_unity_testcase_info(7375,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    if(J721E_LEO_PMICB_DEVICE == pmic_device_info)
    {
        TEST_IGNORE();
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        TEST_IGNORE();
    }

    /* Un Masking all GPIO Interrupts */
    pmicStatus = Pmic_irqGpioMaskIntr(pPmicCoreHandle,
                                      PMIC_IRQ_GPIO_ALL_INT_MASK_NUM,
                                      PMIC_IRQ_UNMASK,
                                      PMIC_IRQ_GPIO_RISE_FALL_INT_TYPE);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    /* Un Masking all Interrupts */
    pmicStatus = Pmic_irqMaskIntr(pPmicCoreHandle,
                                  PMIC_IRQ_ALL,
                                  PMIC_IRQ_UNMASK);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioSetValue(pPmicCoreHandle, pin, pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioSetIntr(pPmicCoreHandle, pin , intrType, maskPol);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pinValue = PMIC_GPIO_LOW;
    pmicStatus = Pmic_gpioSetValue(pPmicCoreHandle, pin, pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    while(1U)
    {
        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        if((PMIC_ST_SUCCESS == pmicStatus) &&
           ((errStat.intStatus[PMIC_TPS6594X_GPIO1_INT/32U] & 
             (1U << (PMIC_TPS6594X_GPIO1_INT % 32U))) != 0U))
        {
            while(PMIC_TPS6594X_GPIO1_INT != irqNum)
            {
                pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                                     &errStat,
                                                     &irqNum);
            }

            if(PMIC_ST_SUCCESS == pmicStatus)
            {
                /* clear the interrupt */
                pmicStatus = Pmic_irqClrErrStatus(pPmicCoreHandle,
                                                  PMIC_TPS6594X_GPIO1_INT);
                break;
            }
        }
    }

    /* Masking All GPIO Interrupts */
    pmicStatus = Pmic_irqGpioMaskIntr(pPmicCoreHandle,
                                      PMIC_IRQ_GPIO_ALL_INT_MASK_NUM,
                                      PMIC_IRQ_MASK,
                                      PMIC_IRQ_GPIO_RISE_FALL_INT_TYPE);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    /* Masking All Interrupts */
    pmicStatus = Pmic_irqMaskIntr(pPmicCoreHandle,
                                  PMIC_IRQ_ALL,
                                  PMIC_IRQ_MASK);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
}

/*!
 * \brief   Gpio Tps6594x NPwron Get PinValue Test
 */
static void test_pmic_gpio_testTps6594xNPwronPinGetValue(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    uint8_t pinValue          = PMIC_GPIO_HIGH;

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        TEST_IGNORE();
    }

    test_pmic_print_unity_testcase_info(1000,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    pmicStatus = Pmic_gpioTps6594xNPwronPinGetValue(pPmicCoreHandle, &pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
}

/*!
 * \brief   Parameter validation for handle
 */
static void test_pmic_gpio_tps6594xNPwronPinGetValuePrmValTest_handle(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    uint8_t pinValue          = PMIC_GPIO_HIGH;

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        TEST_IGNORE();
    }

    test_pmic_print_unity_testcase_info(1001,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    pmicStatus = Pmic_gpioTps6594xNPwronPinGetValue(NULL, &pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, pmicStatus);
}

/*!
 * \brief   Parameter validation for Pin Value
 */
static void test_pmic_gpio_tps6594xNPwronPinGetValuePrmValTest_pinValue(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        TEST_IGNORE();
    }

    test_pmic_print_unity_testcase_info(1002,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    pmicStatus = Pmic_gpioTps6594xNPwronPinGetValue(pPmicCoreHandle, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, pmicStatus);
}

/*!
 * \brief   Pmic_gpioGetConfiguration : NEgative test for Pin 11 for HERA.
 */
static void test_pmic_gpio_getCfgPrmValTest_pin_hera(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    uint8_t pin               = PMIC_TPS6594X_GPIO11_PIN;
    Pmic_GpioCfg_t gpioCfg    =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_GPIO_PINFUNC_NSLEEP1,
        PMIC_GPIO_HIGH
    };

    test_pmic_print_unity_testcase_info(7879,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        TEST_IGNORE();
    }

    pmicStatus = Pmic_gpioGetConfiguration(pPmicCoreHandle, pin, &gpioCfg);
    TEST_ASSERT_EQUAL(pmicStatus, PMIC_ST_ERR_INV_PARAM);
}

/*!
 * \brief   Pmic_gpioTps6594xNPwronPinGetValue : Negative test for nPWRON Pin for hera
 */
static void test_pmic_gpio_testTps6594xNPwronPinGetValue_hera(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    uint8_t pinValue          = PMIC_GPIO_HIGH;

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        TEST_IGNORE();
    }

    test_pmic_print_unity_testcase_info(7880,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    pmicStatus = Pmic_gpioTps6594xNPwronPinGetValue(pPmicCoreHandle, &pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_DEVICE, pmicStatus);
}

#if defined(UNITY_INCLUDE_CONFIG_V2_H) && \
    (defined(SOC_J721E) || defined(SOC_J7200))

/*!
 * \brief   Run gpio unity test cases
 */
static void test_pmic_run_testcases(void)
{
    pmic_log("\n\n%s(): %d: Begin Unity Test Cases...\n", __func__, __LINE__);
    UNITY_BEGIN();

    RUN_TEST(test_pmic_gpio_setCfgGpioPin_nSLEEP1);
    RUN_TEST(test_pmic_gpio_setCfgGpioPin_nSLEEP2);
    RUN_TEST(test_pmic_gpio_setCfgGpioPin_nRstOut_soc);
    RUN_TEST(test_pmic_gpio_setCfgGpioPin_wakeup1);
    RUN_TEST(test_pmic_gpio_setCfgGpioPin_wakeup2);
    RUN_TEST(test_pmic_gpio_setCfgGpioPin_gpio);
    RUN_TEST(test_pmic_gpio_setCfgGpioPin_i2c2_sclk);
    RUN_TEST(test_pmic_gpio_setCfgGpioPin_i2c2_sda);
    RUN_TEST(test_pmic_gpio_setCfgGpioPin_spi_cs);
    RUN_TEST(test_pmic_gpio_setCfgGpioPin_spi_sdo);
    RUN_TEST(test_pmic_gpio_setCfgGpioPin_wdt);
    RUN_TEST(test_pmic_gpio_setCfgGpioPin3_esm_soc);
    RUN_TEST(test_pmic_gpio_setCfgGpioPin_esm_mcu);
    RUN_TEST(test_pmic_gpio_setCfgGpioPin_spmi_sclk);
    RUN_TEST(test_pmic_gpio_setCfgGpioPin_spmi_sdata);
    RUN_TEST(test_pmic_gpio_setCfgGpioPin_syncCLKOUT);
    RUN_TEST(test_pmic_gpio_setCfgGpioPin_synCLKIN);
    RUN_TEST(test_pmic_gpio_setCfgGpioPin_clk32KOUT);
    RUN_TEST(test_pmic_gpio_setCfgGpioPin10_clk32KOUT);
    RUN_TEST(test_pmic_gpio_setCfgGpioPin_wdg_disable);
    RUN_TEST(test_pmic_gpio_setCfgGpioPin_good_power);
    RUN_TEST(test_pmic_gpio_setCfgPrmValTest_handle);
    RUN_TEST(test_pmic_gpio_setCfgPrmValTest_pin);
    RUN_TEST(test_pmic_gpio_setCfgPrmValTest_pinDir);
    RUN_TEST(test_pmic_gpio_setCfgPrmValTest_outputSignalType);
    RUN_TEST(test_pmic_gpio_setCfgPrmValTest_deglitchEnable);
    RUN_TEST(test_pmic_gpio_setCfgPrmValTest_pinFunc_case1);
    RUN_TEST(test_pmic_nPWRON_setCfgPrmValTest_pinPolarity);
    RUN_TEST(test_pmic_gpio_getCfgGpioPin);
    RUN_TEST(test_pmic_gpio_getCfgPrmValTest_handle);
    RUN_TEST(test_pmic_gpio_getCfgPrmValTest_pin);
    RUN_TEST(test_pmic_gpio_getCfgPrmValTest_gpioCfg);
    RUN_TEST(test_pmic_gpio_getValueGpioPin1_signalLevel);
    RUN_TEST(test_pmic_gpio_getValuePrmValTest_handle);
    RUN_TEST(test_pmic_gpio_getValuePrmValTest_pin);
    RUN_TEST(test_pmic_gpio_getValuePrmValTest_pinValue);
    RUN_TEST(test_pmic_gpio_setValueGpioPin1_signalLevel);
    RUN_TEST(test_pmic_gpio_setValuePrmValTest_handle);
    RUN_TEST(test_pmic_gpio_setValuePrmValTest_pin);
    RUN_TEST(test_pmic_gpio_setValuePrmValTest_pinValue);
    RUN_TEST(test_pmic_gpio_setValueGpioPin1_input);
    RUN_TEST(test_pmic_gpio1_testFall_interrupt);
    RUN_TEST(test_pmic_gpio1_testRise_interrupt);
    RUN_TEST(test_pmic_gpio2_testFall_interrupt);
    RUN_TEST(test_pmic_gpio2_testRise_interrupt);
    RUN_TEST(test_pmic_nPWRON_setCfgPrmValTest_pinFunc);
    RUN_TEST(test_pmic_gpio3_testFall_interrupt);
    RUN_TEST(test_pmic_gpio3_testRise_interrupt);
    RUN_TEST(test_pmic_gpio4_testFall_interrupt);
    RUN_TEST(test_pmic_gpio4_testRise_interrupt);
    RUN_TEST(test_pmic_gpio5_testFall_interrupt);
    RUN_TEST(test_pmic_gpio5_testRise_interrupt);
    RUN_TEST(test_pmic_gpio6_testFall_interrupt);
    RUN_TEST(test_pmic_gpio6_testRise_interrupt);
    RUN_TEST(test_pmic_gpio7_testFall_interrupt);
    RUN_TEST(test_pmic_gpio7_testRise_interrupt);
    RUN_TEST(test_pmic_gpio8_testFall_interrupt);
    RUN_TEST(test_pmic_gpio8_testRise_interrupt);
    RUN_TEST(test_pmic_gpio9_testFall_interrupt);
    RUN_TEST(test_pmic_gpio9_testRise_interrupt);
    RUN_TEST(test_pmic_gpio10_testFall_interrupt);
    RUN_TEST(test_pmic_gpio10_testRise_interrupt);
    RUN_TEST(test_pmic_gpio11_testFall_interrupt);
    RUN_TEST(test_pmic_gpio11_testRise_interrupt);
    RUN_TEST(test_pmic_gpio_intr_prmValTest_handle);
    RUN_TEST(test_pmic_gpio_intr_prmValTest_pin);
    RUN_TEST(test_pmic_gpio_intr_prmValTest_intrType);
    RUN_TEST(test_pmic_gpio_intr_prmValTest_maskPol);
    RUN_TEST(test_pmic_gpio_intr_irqMaskAll_interrupt);
    RUN_TEST(test_pmic_gpio_intr_irqUnMaskAll_interrupt);
    RUN_TEST(test_pmic_gpio_testTps6594xNPwronPinGetValue);
    RUN_TEST(test_pmic_gpio_tps6594xNPwronPinGetValuePrmValTest_handle);
    RUN_TEST(test_pmic_gpio_tps6594xNPwronPinGetValuePrmValTest_pinValue);
    RUN_TEST(test_pmic_gpio_testTps6594xNPwronPinGetValue_hera);
    RUN_TEST(test_pmic_gpio_getCfgPrmValTest_pin_hera);

    UNITY_END();
}

/*!
 * \brief   GPIO Unity Test App wrapper Function for LEO PMIC-A
 */
static int32_t test_pmic_leo_pmicA_gpio_testApp(void)
{
    int32_t status                = PMIC_ST_SUCCESS;
    Pmic_CoreCfg_t pmicConfigData = {0U};

    /* Fill parameters to pmicConfigData */
    pmicConfigData.pmicDeviceType      = PMIC_DEV_LEO_TPS6594X;
    pmicConfigData.validParams        |= PMIC_CFG_DEVICE_TYPE_VALID_SHIFT;

    pmicConfigData.commMode            = PMIC_INTF_DUAL_I2C;
    pmicConfigData.validParams        |= PMIC_CFG_COMM_MODE_VALID_SHIFT;

    pmicConfigData.slaveAddr           = LEO_PMICA_SLAVE_ADDR;
    pmicConfigData.validParams        |= PMIC_CFG_SLAVEADDR_VALID_SHIFT;

    pmicConfigData.qaSlaveAddr         = LEO_PMICA_WDG_SLAVE_ADDR;
    pmicConfigData.validParams        |= PMIC_CFG_QASLAVEADDR_VALID_SHIFT;

    pmicConfigData.pFnPmicCommIoRead    = test_pmic_regRead;
    pmicConfigData.validParams         |= PMIC_CFG_COMM_IO_RD_VALID_SHIFT;

    pmicConfigData.pFnPmicCommIoWrite   = test_pmic_regWrite;
    pmicConfigData.validParams         |= PMIC_CFG_COMM_IO_WR_VALID_SHIFT;

    pmicConfigData.pFnPmicCritSecStart  = test_pmic_criticalSectionStartFn;
    pmicConfigData.validParams         |= PMIC_CFG_CRITSEC_START_VALID_SHIFT;

    pmicConfigData.pFnPmicCritSecStop   = test_pmic_criticalSectionStopFn;
    pmicConfigData.validParams         |= PMIC_CFG_CRITSEC_STOP_VALID_SHIFT;

    status = test_pmic_appInit(&pPmicCoreHandle, &pmicConfigData);
    return status;
}

/*!
 * \brief   GPIO Unity Test App wrapper Function for LEO PMIC-B
 */
static int32_t test_pmic_leo_pmicB_gpio_testApp(void)
{
    int32_t status                = PMIC_ST_SUCCESS;
    Pmic_CoreCfg_t pmicConfigData = {0U};

    /* Fill parameters to pmicConfigData */
    pmicConfigData.pmicDeviceType      = PMIC_DEV_LEO_TPS6594X;
    pmicConfigData.validParams        |= PMIC_CFG_DEVICE_TYPE_VALID_SHIFT;

    pmicConfigData.commMode            = PMIC_INTF_SINGLE_I2C;
    pmicConfigData.validParams        |= PMIC_CFG_COMM_MODE_VALID_SHIFT;

    pmicConfigData.slaveAddr           = LEO_PMICB_SLAVE_ADDR;
    pmicConfigData.validParams        |= PMIC_CFG_SLAVEADDR_VALID_SHIFT;

    pmicConfigData.qaSlaveAddr         = LEO_PMICB_WDG_SLAVE_ADDR;
    pmicConfigData.validParams        |= PMIC_CFG_QASLAVEADDR_VALID_SHIFT;

    pmicConfigData.pFnPmicCommIoRead    = test_pmic_regRead;
    pmicConfigData.validParams         |= PMIC_CFG_COMM_IO_RD_VALID_SHIFT;

    pmicConfigData.pFnPmicCommIoWrite   = test_pmic_regWrite;
    pmicConfigData.validParams         |= PMIC_CFG_COMM_IO_WR_VALID_SHIFT;

    pmicConfigData.pFnPmicCritSecStart  = test_pmic_criticalSectionStartFn;
    pmicConfigData.validParams         |= PMIC_CFG_CRITSEC_START_VALID_SHIFT;

    pmicConfigData.pFnPmicCritSecStop   = test_pmic_criticalSectionStopFn;
    pmicConfigData.validParams         |= PMIC_CFG_CRITSEC_STOP_VALID_SHIFT;

    status = test_pmic_appInit(&pPmicCoreHandle, &pmicConfigData);
    return status;

}

/*!
 * \brief   GPIO Unity Test App wrapper Function for LEO PMIC-A
 */
static int32_t test_pmic_leo_pmicA_spiStub_gpio_testApp(void)
{
    int32_t status                = PMIC_ST_SUCCESS;
    Pmic_CoreCfg_t pmicConfigData = {0U};

    /* Fill parameters to pmicConfigData */
    pmicConfigData.pmicDeviceType      = PMIC_DEV_LEO_TPS6594X;
    pmicConfigData.validParams        |= PMIC_CFG_DEVICE_TYPE_VALID_SHIFT;

    pmicConfigData.commMode            = PMIC_INTF_SPI;
    pmicConfigData.validParams        |= PMIC_CFG_COMM_MODE_VALID_SHIFT;

    pmicConfigData.pFnPmicCommIoRead    = test_pmic_regRead;
    pmicConfigData.validParams         |= PMIC_CFG_COMM_IO_RD_VALID_SHIFT;

    pmicConfigData.pFnPmicCommIoWrite   = test_pmic_regWrite;
    pmicConfigData.validParams         |= PMIC_CFG_COMM_IO_WR_VALID_SHIFT;

    pmicConfigData.pFnPmicCritSecStart  = test_pmic_criticalSectionStartFn;
    pmicConfigData.validParams         |= PMIC_CFG_CRITSEC_START_VALID_SHIFT;

    pmicConfigData.pFnPmicCritSecStop   = test_pmic_criticalSectionStopFn;
    pmicConfigData.validParams         |= PMIC_CFG_CRITSEC_STOP_VALID_SHIFT;

    status = test_pmic_appInit(&pPmicCoreHandle, &pmicConfigData);
    return status;
}

/*!
 * \brief   GPIO Unity Test App wrapper Function for HERA PMIC
 */
static int32_t test_pmic_hera_gpio_testApp(void)
{
    int32_t status                = PMIC_ST_SUCCESS;
    Pmic_CoreCfg_t pmicConfigData = {0U};

    /* Fill parameters to pmicConfigData */
    pmicConfigData.pmicDeviceType      = PMIC_DEV_HERA_LP8764X;
    pmicConfigData.validParams        |= PMIC_CFG_DEVICE_TYPE_VALID_SHIFT;

    pmicConfigData.commMode            = PMIC_INTF_SINGLE_I2C;
    pmicConfigData.validParams        |= PMIC_CFG_COMM_MODE_VALID_SHIFT;

    pmicConfigData.slaveAddr           = HERA_PMIC_SLAVE_ADDR;
    pmicConfigData.validParams        |= PMIC_CFG_SLAVEADDR_VALID_SHIFT;

    pmicConfigData.qaSlaveAddr         = HERA_PMIC_WDG_SLAVE_ADDR;
    pmicConfigData.validParams        |= PMIC_CFG_QASLAVEADDR_VALID_SHIFT;

    pmicConfigData.pFnPmicCommIoRead    = test_pmic_regRead;
    pmicConfigData.validParams         |= PMIC_CFG_COMM_IO_RD_VALID_SHIFT;

    pmicConfigData.pFnPmicCommIoWrite   = test_pmic_regWrite;
    pmicConfigData.validParams         |= PMIC_CFG_COMM_IO_WR_VALID_SHIFT;

    pmicConfigData.pFnPmicCritSecStart  = test_pmic_criticalSectionStartFn;
    pmicConfigData.validParams         |= PMIC_CFG_CRITSEC_START_VALID_SHIFT;

    pmicConfigData.pFnPmicCritSecStop   = test_pmic_criticalSectionStopFn;
    pmicConfigData.validParams         |= PMIC_CFG_CRITSEC_STOP_VALID_SHIFT;

    status = test_pmic_appInit(&pPmicCoreHandle, &pmicConfigData);
    return status;

}

static const char pmicTestAppMenu[] =
{
    " \r\n ================================================================="
    " \r\n Menu:"
    " \r\n ================================================================="
    " \r\n 0: Pmic Leo device(PMIC A on J721E EVM Using I2C Interface)"
    " \r\n 1: Pmic Leo device(PMIC B on J721E EVM Using I2C Interface)"
    " \r\n 2: Pmic Hera device"
    " \r\n 3: Pmic Leo device(PMIC A on J721E EVM Using SPI Stub Functions)"
    " \r\n 4: quit"
    " \r\n"
    " \r\n Enter option: "
};

/*!
 * \brief   Function to register GPIO Unity Test App wrapper to Unity framework
 */
static void test_pmic_gpio_testapp_runner(void)
{
    /* @description : Test runner for Gpio Test App
     *
     * @requirements: 5808, 5812, 5814, 5810, 5813,
     *                5843, 5853
     *
     * @cores       : mcu1_0, mcu1_1
     */

    int8_t num = -1;

    while(1U)
    {
        pmic_log("%s", pmicTestAppMenu);
        if(UART_scanFmt("%d", &num) != 0U)
        {
            pmic_log("Read from UART Console failed\n");
            return;
        }

        switch(num)
        {
           case 0U:
               /* GPIO Unity Test App wrapper Function for LEO PMIC-A */
               test_pmic_leo_pmicA_gpio_testApp();
               pmic_device_info = J721E_LEO_PMICA_DEVICE;
               /* Run gpio test cases for Leo PMIC-A */
               test_pmic_run_testcases();
               /* Deinit pmic handle */
               if(pPmicCoreHandle != NULL)
               {
                   test_pmic_appDeInit(pPmicCoreHandle);
               }
               break;
           case 1U:
               /* GPIO Unity Test App wrapper Function for LEO PMIC-B */
               test_pmic_leo_pmicB_gpio_testApp();
               pmic_device_info = J721E_LEO_PMICB_DEVICE;
               /* Run gpio test cases for Leo PMIC-B */
               test_pmic_run_testcases();

               /* Deinit pmic handle */
               if(pPmicCoreHandle != NULL)
               {
                   test_pmic_appDeInit(pPmicCoreHandle);
               }
               break;
           case 2U:
               /* GPIO Unity Test App wrapper Function for HERA PMIC */
               test_pmic_hera_gpio_testApp();
               /* Run gpio test cases for Hera PMIC */
               test_pmic_run_testcases();
               /* Deinit pmic handle */
               if(pPmicCoreHandle != NULL)
               {
                   test_pmic_appDeInit(pPmicCoreHandle);
               }
               break;
           case 3U:
               /* GPIO Unity Test App wrapper Function for LEO PMIC-A using
                * SPI stub functions */
               test_pmic_leo_pmicA_spiStub_gpio_testApp();
               pmic_device_info = J721E_LEO_PMICA_DEVICE;
               /* Run gpio test cases for Leo PMIC-A */
               test_pmic_run_testcases();
               /* Deinit pmic handle */
               if(pPmicCoreHandle != NULL)
               {
                   test_pmic_appDeInit(pPmicCoreHandle);
               }
               break;
           case 4U:
               pmic_log(" \r\n Quit from application\n");
               return;
           default:
               pmic_log(" \r\n Invalid option... Try Again!!!\n");
               break;
        }
    }
}
#endif

/*!
 * \brief   TI RTOS specific GPIO TEST APP main Function
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code.
 *          For valid values see \ref Pmic_ErrorCodes
 */
int main()
{
    Board_initUART();

    pmic_log("GPIO Unity Test Application(%s %s)\n", __TIME__, __DATE__);
#if defined(UNITY_INCLUDE_CONFIG_V2_H) && \
    (defined(SOC_J721E)             || \
     defined(SOC_J7200))
    test_pmic_gpio_testapp_runner();
#endif
}
