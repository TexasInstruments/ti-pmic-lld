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

extern uint16_t pmic_device_info;
extern int32_t gCrcTestFlag_J721E;
extern int32_t gCrcTestFlag_J7VCL;

volatile uint32_t pmic_intr_triggered;
extern Pmic_Ut_FaultInject_t gPmic_faultInjectCfg;

extern int8_t gIntrTopRegTestFlag;
extern int8_t girqGetL1RegTestFlag_Leo;
extern int8_t girqGetL2RegTestFlag_Leo;
extern int8_t girqGetL1RegTestFlag_Leo_PMICB;
extern int8_t girq1L1RegTestFlag;
extern int8_t girq2L1RegTestFlag;
extern int8_t girq1L2RegTestFlag;
extern int8_t girq2L2RegTestFlag;
extern int8_t girqGetL1RegTestFlag_Hera;
extern int8_t girqGetL2RegTestFlag_Hera;

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
        0xAB1E,
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
        "Pmic_gpioSetConfiguration : Parameter validation for pin for Max Value"
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
        0xAB19,
        "Pmic_gpioSetIntr : GPIO3 Fall Interrupt Test"
    },
    {
        0xAB18,
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
        0xAB1D,
        "Pmic_gpioSetIntr : GPIO9 Fall Interrupt Test"
    },
    {
        0xAB1C,
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
        0xAB1B,
        "Pmic_gpioSetIntr : GPIO11 Fall Interrupt Test"
    },
    {
        0xAB1A,
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
        7904,
        "Pmic_gpioTps6594xNPwronPinGetValue : Gpio Tps6594x NPwron Get PinValue Test"
    },
    {
        7905,
        "Pmic_gpioTps6594xNPwronPinGetValue : Parameter validation for handle"
    },
    {
        7906,
        "Pmic_gpioTps6594xNPwronPinGetValue : Parameter validation for PinValue"
    },
    {
        7879,
        "Pmic_gpioGetConfiguration : Negative test for Pin 11 for HERA."
    },
    {
        7880,
        "Pmic_gpioTps6594xNPwronPinGetValue : Negative test for nPWRON Pin for hera"
    },
    {
        7950,
        "Pmic_gpioSetIntr : Test to verify GPIO7 Fall Asynchronous interrupt"
    },
    {
        7951,
        "Pmic_gpioSetIntr : Test to verify GPIO7 RISE Asynchronous interrupt"
    },
    {
        8012,
        "Pmic_fsmSetMissionState: Test to verify CAN WKUP"
    },
    {
        0xAB21,
        "Pmic_fsmSetMissionState: Test to verify GPIO WKUP1"
    },
    {
        0xAB10,
        "Pmic_fsmSetMissionState: Test to verify GPIO WKUP2"
    },
    {
        8041,
        "Pmic_irqGetErrStatus: Test to Clear all interrupts"
    },
    {
        9887,
        "Pmic_irqGetGpioMaskIntr/Pmic_irqMaskIntr: Test to Get Mask status of Gpio and Irq Interrupts"
    },
    {
        9888,
        "Pmic_irqGetGpioMaskIntr/Pmic_irqMaskIntr: Test to Get UnMask status of Gpio and Irq Interrupts"
    },
    {
        9889,
        "Pmic_irqGetGpioMaskIntr: Parameter validation for handle"
    },
    {
        9890,
        "Pmic_irqGetGpioMaskIntr: Parameter validation for pFallIntrMaskStat"
    },
    {
        9891,
        "Pmic_irqGetGpioMaskIntr: Parameter validation for pRiseIntrMaskStat"
    },
    {
        9892,
        "Pmic_irqGetMaskIntrStatus: Parameter validation for handle"
    },
    {
        9893,
        "Pmic_irqGetMaskIntrStatus: Parameter validation for pMaskStatus"
    },
    {
        10558,
        "Pmic_gpioGetNPwronEnablePinConfiguration : Parameter validation for pGpioCfg"
    },
    {
        10559,
        "Pmic_gpioSetNPwronEnablePinConfiguration : Test to set/get Pin function as PMIC_TPS6594X_NPWRON_PINFUNC_NONE"
    },
    {
        10560,
        "Pmic_gpioSetNPwronEnablePinConfiguration : Test to set/get Pin polarity for Enable Pin"
    },
    {
        10561,
        "Pmic_gpioSetConfiguration : Test to set/get GPIO pullup/pull down control configuration"
    },
    {
        10562,
        "Pmic_gpioSetConfiguration : Test to set/get signal deglitch time enable configuration"
    },
    {
        10563,
        "Pmic_gpioSetIntr : Test to verify GPIO1 disable Interrupt"
    },
    {
        10564,
        "Pmic_gpioSetConfiguration : Parameter validation for pullCtrl"
    },
    {
        10565,
        "Pmic_gpioSetNPwronEnablePinConfiguration : Test to set/get nPWRON/Enable pull up control configuration"
    },
    {
        10566,
        "Pmic_gpioSetNPwronEnablePinConfiguration : Parameter validation for pPmicCoreHandle"
    },
    {
        10567,
        "Pmic_gpioGetNPwronEnablePinConfiguration : Parameter validation for pPmicCoreHandle"
    },
    {
        10568,
        "Pmic_gpioSetNPwronEnablePinConfiguration : Test to set/get signal deglitch time enable configuration"
    },
    {
        10569,
        "Pmic_gpioSetNPwronEnablePinConfiguration : Parameter Validation for deglitchEnable for NPwronEnablePin"
    },
    {
        10570,
        "Pmic_gpioSetNPwronEnablePinConfiguration : Parameter Validation for pullCtrlfor NPwronEnablePin"
    },
    {
        10571,
        "Pmic_gpioSetConfiguration : Parameter validation for pin for Min Value"
    },
    {
        10572,
        "Pmic_gpioSetNPwronEnablePinConfiguration : Test to set/get Invalid validParams"
    },
    {
        10753,
        "Pmic_irqGetMaskIntrStatus: Parameter validation for irqNum"
    },
    {
        10754,
        "Pmic_irqClrErrStatus: Parameter validation for handle"
    },
    {
        10755,
        "Pmic_irqClrErrStatus: Parameter validation for irqNum"
    },
    {
        10756,
        "Pmic_irqGetErrStatus: Parameter validation for handle"
    },
    {
        10757,
        "Pmic_irqGetErrStatus: Parameter validation for pErrStat"
    },
    {
        10758,
        "Pmic_irqMaskIntr: Parameter validation for handle"
    },
    {
        10759,
        "Pmic_getNextErrorStatus: Parameter validation for handle"
    },
    {
        10760,
        "Pmic_getNextErrorStatus: Parameter validation for pErrStat"
    },
    {
        10761,
        "Pmic_getNextErrorStatus: Parameter validation for pIrqNum"
    },
    {
        10762,
        "Pmic_getNextErrorStatus: Test to check for error Status intStatus[0], intStatus[2]and intStatus[3] not equal to zero"
    },
    {
        10763,
        "Pmic_irqGpioMaskIntr: Parameter validation for handle"
    },
    {
        10764,
        "Pmic_irqGpioMaskIntr: Parameter validation for irqGpioNum"
    },
    {
        10765,
        "Pmic_irqGpioMaskIntr: Parameter validation for gpioIntrType"
    },
    {
        10766,
        "Pmic_irqGpioMaskIntr: Parameter validation for irqGpioNum on Hera device type"
    },
    {
        10767,
        "Pmic_irqGetGpioMaskIntr: Parameter validation for irqGpioNum"
    },
    {
        10768,
        "Pmic_irqGetGpioMaskIntr: Parameter validation for gpioIntrType"
    },
    {
        10769,
        "Pmic_irqGetGpioMaskIntr: Parameter validation for irqGpioNum on Hera device type"
    },
    {
        10770,
        "Pmic_irqGpioMaskIntr: Test for Masking All GPIO Interrupts for Leo device type check - Code covrage"
    },
    {
        8852,
        "Pmic_gpioTests: Dynamic Coverage Gaps and Fault Injection Tests"
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
        pinMax = PMIC_TPS6594X_GPIO11_PIN;
        gpioCfg.pinFunc = PMIC_TPS6594X_GPIO_PINFUNC_NSLEEP1;
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

        if(PMIC_SILICON_REV_ID_PG_2_0 == pPmicCoreHandle->pmicDevSiliconRev)
        {
            /* PMIC-A GPIO5/ GPIO6 pin Causing Reset on J721 EVM */
            if(((5U == pins[pin]) || (6U == pins[pin])) &&
               (J721E_LEO_PMICA_DEVICE == pmic_device_info))
            {
                continue;
            }
        }

        if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
        {
            /* Should not break SPI Comm Lines */
            if((PMIC_INTF_SPI == pPmicCoreHandle->commMode) &&
               ((1U == pins[pin]) || (2U == pins[pin])))
            {
                continue;
            }
        }
        if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
        {
            /* Should not break SPI Comm Lines */
            if((PMIC_INTF_SPI == pPmicCoreHandle->commMode) &&
               ((2U == pins[pin]) || (3U == pins[pin])))
            {
                continue;
            }
        }

        /* GPIO Pins 1, 2 and 10 are supported on PMIC-B on J721EVM.
         * Refer to 7457 for more details
         */
        if((!((1U == pins[pin]) || (2U == pins[pin]) || (10U == pins[pin]))) &&
           (J721E_LEO_PMICB_DEVICE == pmic_device_info))
        {
            continue;
        }

        if(((3U == pins[pin]) || (4U == pins[pin]) ||
            (5U == pins[pin]) || (6U == pins[pin])) &&
           (J7VCL_LEO_PMICA_DEVICE == pmic_device_info))
        {
            if(4U == pins[pin])
            {
                Pmic_irqGpioMaskIntr(pPmicCoreHandle,
                                     PMIC_TPS6594X_IRQ_GPIO_4_INT_MASK_NUM,
                                     PMIC_IRQ_MASK,
                                     PMIC_IRQ_GPIO_RISE_INT_TYPE);
            }
            else
            {
                continue;
            }
        }

        if(((1U == pins[pin]) || (8U == pins[pin]) ||
            (9U == pins[pin]) || (10U == pins[pin])) &&
           (J7VCL_HERA_PMICB_DEVICE == pmic_device_info))
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
    pmic_testResultUpdate_pass(6185,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
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
        pinMax = PMIC_TPS6594X_GPIO11_PIN;
        gpioCfg.pinFunc = PMIC_TPS6594X_GPIO_PINFUNC_NSLEEP1;
    }
    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pinMax = PMIC_LP8764X_GPIO10_PIN;
        gpioCfg.pinFunc = PMIC_LP8764X_GPIO_PINFUNC_NSLEEP2;
    }

    for(pin = 0U; pin < pinMax; pin++)
    {
        /* PMIC-A GPIO3 pin Causing Reset on J721 EVM */
        if((3U == pins[pin]) &&
           (J721E_LEO_PMICA_DEVICE == pmic_device_info))
        {
            continue;
        }

        if(PMIC_SILICON_REV_ID_PG_2_0 == pPmicCoreHandle->pmicDevSiliconRev)
        {
            /* PMIC-A GPIO5/ GPIO6 pin Causing Reset on J721 EVM */
            if(((5U == pins[pin]) || (6U == pins[pin])) &&
               (J721E_LEO_PMICA_DEVICE == pmic_device_info))
            {
                continue;
            }
        }

        if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
        {
            /* Should not break SPI Comm Lines */
            if((PMIC_INTF_SPI == pPmicCoreHandle->commMode) &&
               ((1U == pins[pin]) || (2U == pins[pin])))
            {
                continue;
            }
        }
        if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
        {
            /* Should not break SPI Comm Lines */
            if((PMIC_INTF_SPI == pPmicCoreHandle->commMode) &&
               ((2U == pins[pin]) || (3U == pins[pin])))
            {
                continue;
            }
        }

        /* GPIO Pins 1, 2 and 10 are supported on PMIC-B on J721EVM.
         * Refer to 7457 for more details
         */
        if((!((1U == pins[pin]) || (2U == pins[pin]) || (10U == pins[pin]))) &&
           (J721E_LEO_PMICB_DEVICE == pmic_device_info))
        {
            continue;
        }

        if(((3U == pins[pin]) || (4U == pins[pin]) ||
            (5U == pins[pin]) || (6U == pins[pin])) &&
           (J7VCL_LEO_PMICA_DEVICE == pmic_device_info))
        {
            if(4U == pins[pin])
            {
                Pmic_irqGpioMaskIntr(pPmicCoreHandle,
                                     PMIC_TPS6594X_IRQ_GPIO_4_INT_MASK_NUM,
                                     PMIC_IRQ_MASK,
                                     PMIC_IRQ_GPIO_RISE_INT_TYPE);
            }
            else
            {
                continue;
            }
        }

        if(((1U == pins[pin]) || (8U == pins[pin]) ||
            (9U == pins[pin]) || (10U == pins[pin])) &&
           (J7VCL_HERA_PMICB_DEVICE == pmic_device_info))
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
    pmic_testResultUpdate_pass(6186,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
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
        if((J721E_LEO_PMICA_DEVICE == pmic_device_info) &&
           ((5U == pins[pin]) || (6U == pins[pin])))
        {
            /*
             * GPIO5/ GPIO6 pin of PMIC-A is connected to LEOA_SCLK and LEOA_SDATA
             * on J721EVM board causing hang, when programming NRSTOUT signal.
             */
            continue;
        }

        if((J721E_LEO_PMICB_DEVICE == pmic_device_info) && (11U == pins[pin]))
        {
            /*
             * GPIO11 pin of PMIC-B is connected to 'EN_3V3IO_LDSW'
             * on J721EVM board causing hang, when programming NRSTOUT signal.
             */
            continue;
        }

        if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
        {
            /* Should not break SPI Comm Lines */
            if((PMIC_INTF_SPI == pPmicCoreHandle->commMode) &&
               (1U == pins[pin]))
            {
                continue;
            }
        }


        if(((1U == pins[pin]) || (10U == pins[pin])) &&
           (J7VCL_HERA_PMICB_DEVICE == pmic_device_info))
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
    pmic_testResultUpdate_pass(6187,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
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
        pinMax = PMIC_TPS6594X_GPIO11_PIN;
        gpioCfg.pinFunc = PMIC_TPS6594X_GPIO_PINFUNC_WKUP1;
    }
    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pinMax = PMIC_LP8764X_GPIO10_PIN;
        gpioCfg.pinFunc = PMIC_LP8764X_GPIO_PINFUNC_WKUP1;
    }

    for(pin = 0U; pin < pinMax; pin++)
    {
        /* PMIC-A GPIO3 pin Causing Reset on J721 EVM */
        if((3U == pins[pin]) &&
           (J721E_LEO_PMICA_DEVICE == pmic_device_info))
        {
            continue;
        }

        if(PMIC_SILICON_REV_ID_PG_2_0 == pPmicCoreHandle->pmicDevSiliconRev)
        {
            /* PMIC-A GPIO5/ GPIO6 pin Causing Reset on J721 EVM */
            if(((5U == pins[pin]) || (6U == pins[pin])) &&
               (J721E_LEO_PMICA_DEVICE == pmic_device_info))
            {
                continue;
            }
        }

        if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
        {
            /* Should not break SPI Comm Lines */
            if((PMIC_INTF_SPI == pPmicCoreHandle->commMode) &&
               ((1U == pins[pin]) || (2U == pins[pin])))
            {
                continue;
            }
        }
        if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
        {
            /* Should not break SPI Comm Lines */
            if((PMIC_INTF_SPI == pPmicCoreHandle->commMode) &&
               ((2U == pins[pin]) || (3U == pins[pin])))
            {
                continue;
            }
        }

        /* GPIO Pins 1, 2 and 10 are supported on PMIC-B on J721EVM.
         * Refer to 7457 for more details
         */
        if((!((1U == pins[pin]) || (2U == pins[pin]) || (10U == pins[pin]))) &&
           (J721E_LEO_PMICB_DEVICE == pmic_device_info))
        {
            continue;
        }

        if(((3U == pins[pin]) || (4U == pins[pin]) ||
            (5U == pins[pin]) || (6U == pins[pin])) &&
           (J7VCL_LEO_PMICA_DEVICE == pmic_device_info))
        {
            if(4U == pins[pin])
            {
                Pmic_irqGpioMaskIntr(pPmicCoreHandle,
                                     PMIC_TPS6594X_IRQ_GPIO_4_INT_MASK_NUM,
                                     PMIC_IRQ_MASK,
                                     PMIC_IRQ_GPIO_RISE_INT_TYPE);
            }
            else
            {
                continue;
            }
        }

        if(((1U == pins[pin]) || (8U == pins[pin]) ||
            (9U == pins[pin]) || (10U == pins[pin])) &&
           (J7VCL_HERA_PMICB_DEVICE == pmic_device_info))
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
    pmic_testResultUpdate_pass(6189,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
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
        pinMax = PMIC_TPS6594X_GPIO11_PIN;
        gpioCfg.pinFunc = PMIC_TPS6594X_GPIO_PINFUNC_WKUP2;
    }
    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pinMax = PMIC_LP8764X_GPIO10_PIN;
        gpioCfg.pinFunc = PMIC_LP8764X_GPIO_PINFUNC_WKUP2;
    }

    for(pin = 0U; pin < pinMax; pin++)
    {
        /* PMIC-A GPIO3 pin Causing Reset on J721 EVM */
        if((3U == pins[pin]) &&
           (J721E_LEO_PMICA_DEVICE == pmic_device_info))
        {
            continue;
        }

        if(PMIC_SILICON_REV_ID_PG_2_0 == pPmicCoreHandle->pmicDevSiliconRev)
        {
            /* PMIC-A GPIO5/ GPIO6 pin Causing Reset on J721 EVM */
            if(((5U == pins[pin]) || (6U == pins[pin])) &&
               (J721E_LEO_PMICA_DEVICE == pmic_device_info))
            {
                continue;
            }
        }

        if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
        {
            /* Should not break SPI Comm Lines */
            if((PMIC_INTF_SPI == pPmicCoreHandle->commMode) &&
               ((1U == pins[pin]) || (2U == pins[pin])))
            {
                continue;
            }
        }
        if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
        {
            /* Should not break SPI Comm Lines */
            if((PMIC_INTF_SPI == pPmicCoreHandle->commMode) &&
               ((2U == pins[pin]) || (3U == pins[pin])))
            {
                continue;
            }
        }

        /* GPIO Pins 1, 2 and 10 are supported on PMIC-B on J721EVM.
         * Refer to 7457 for more details
         */
        if((!((1U == pins[pin]) || (2U == pins[pin]) || (10U == pins[pin]))) &&
           (J721E_LEO_PMICB_DEVICE == pmic_device_info))
        {
            continue;
        }

        if(((3U == pins[pin]) || (4U == pins[pin]) ||
            (5U == pins[pin]) || (6U == pins[pin])) &&
           (J7VCL_LEO_PMICA_DEVICE == pmic_device_info))
        {
            if(4U == pins[pin])
            {
                Pmic_irqGpioMaskIntr(pPmicCoreHandle,
                                     PMIC_TPS6594X_IRQ_GPIO_4_INT_MASK_NUM,
                                     PMIC_IRQ_MASK,
                                     PMIC_IRQ_GPIO_RISE_INT_TYPE);
            }
            else
            {
                continue;
            }
        }

        if(((1U == pins[pin]) || (8U == pins[pin]) ||
            (9U == pins[pin]) || (10U == pins[pin])) &&
           (J7VCL_HERA_PMICB_DEVICE == pmic_device_info))
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
    pmic_testResultUpdate_pass(6190,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
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
         * GPIO5/ GPIO6 pin of PMIC-A is connected to LEOA_SCLK and LEOA_SDATA
         * which resets entire SOC.
         */
        if(((11U == pins[pin]) || (5U == pins[pin]) || (6U == pins[pin])) &&
           (pmic_device_info == J721E_LEO_PMICA_DEVICE))
        {
            continue;
        }

        if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
        {
            /* Should not break SPI Comm Lines */
            if((PMIC_INTF_SPI == pPmicCoreHandle->commMode) &&
               ((1U == pins[pin]) || (2U == pins[pin])))
            {
                continue;
            }
        }
        if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
        {
            /* Should not break SPI Comm Lines */
            if((PMIC_INTF_SPI == pPmicCoreHandle->commMode) &&
               ((2U == pins[pin]) || (3U == pins[pin])))
            {
                continue;
            }
        }

        /* GPIO Pins 1, 2 and 10 are supported on PMIC-B on J721EVM.
         * Refer to 7457 for more details
         */
        if((!((1U == pins[pin]) || (2U == pins[pin]) || (10U == pins[pin]))) &&
           (J721E_LEO_PMICB_DEVICE == pmic_device_info))
        {
            continue;
        }

        if((J7VCL_LEO_PMICA_DEVICE == pmic_device_info) &&
           ((5U == pins[pin]) || (6U == pins[pin]) || (11U == pins[pin])))
        {
            continue;
        }

        if(((8U == pins[pin]) || (9U == pins[pin]) || (10U == pins[pin])) &&
           (J7VCL_HERA_PMICB_DEVICE == pmic_device_info))
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
    pmic_testResultUpdate_pass(6191,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
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
    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        /* Should not break SPI Comm Lines */
        if((PMIC_INTF_SPI == pPmicCoreHandle->commMode) &&
           ((1U == pin) || (2U == pin)))
        {
            pmic_testResultUpdate_ignore(6192,
                                         pmic_gpio_tests,
                                         PMIC_GPIO_NUM_OF_TESTCASES);
        }
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        /* Should not break SPI Comm Lines */
        if((PMIC_INTF_SPI == pPmicCoreHandle->commMode) &&
           ((2U == pin) || (3U == pin)))
        {
            pmic_testResultUpdate_ignore(6192,
                                         pmic_gpio_tests,
                                         PMIC_GPIO_NUM_OF_TESTCASES);
        }
    }

    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioGetConfiguration(pPmicCoreHandle, pin, &gpioCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(gpioCfg.pinFunc, gpioCfg_rd.pinFunc);

    pmic_testResultUpdate_pass(6192,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
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

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        /* Should not break SPI Comm Lines */
        if((PMIC_INTF_SPI == pPmicCoreHandle->commMode) &&
           ((1U == pin) || (2U == pin)))
        {
            pmic_testResultUpdate_ignore(6193,
                                         pmic_gpio_tests,
                                         PMIC_GPIO_NUM_OF_TESTCASES);
        }
    }
    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        /* Should not break SPI Comm Lines */
        if((PMIC_INTF_SPI == pPmicCoreHandle->commMode) &&
           ((2U == pin) || (3U == pin)))
        {
            pmic_testResultUpdate_ignore(6193,
                                         pmic_gpio_tests,
                                         PMIC_GPIO_NUM_OF_TESTCASES);
        }
    }

    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioGetConfiguration(pPmicCoreHandle, pin, &gpioCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(gpioCfg.pinFunc, gpioCfg_rd.pinFunc);

    pmic_testResultUpdate_pass(6193,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
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

    pmic_testResultUpdate_pass(6195,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
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

    pmic_testResultUpdate_pass(6196,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
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
        if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
        {
            /* Should not break SPI Comm Lines */
            if((PMIC_INTF_SPI == pPmicCoreHandle->commMode) &&
               (2U == pins[pin]))
            {
                continue;
            }
        }
        if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
        {
            /* Should not break SPI Comm Lines */
            if((PMIC_INTF_SPI == pPmicCoreHandle->commMode) &&
               (2U == pins[pin]))
            {
                continue;
            }
        }

        if((J721E_LEO_PMICA_DEVICE == pmic_device_info) &&
           ((5U == pins[pin]) || (6U == pins[pin])))
        {
            /*
             * GPIO5/ GPIO6 pin of PMIC-A is connected to LEOA_SCLK and LEOA_SDATA
             * on J721EVM board causing hang, when programming NRSTOUT signal.
             */
            continue;
        }

        if((J721E_LEO_PMICB_DEVICE == pmic_device_info) && (11U == pins[pin]))
        {
            /*
             * GPIO11 pin of PMIC-B is connected to 'EN_3V3IO_LDSW'
             * on J721EVM board causing hang, when programming TRIG_WDOG signal
             */
            continue;
        }

        if((PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType) &&
           (4U == pins[pin]))
        {
            gpioCfg.pinFunc = PMIC_LP8764X_GPIO_PINFUNC_GPIO4_TRIG_WDOG;
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

    pmic_testResultUpdate_pass(6197,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
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
        pmic_testResultUpdate_ignore(6199,
                                         pmic_gpio_tests,
                                         PMIC_GPIO_NUM_OF_TESTCASES);
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        gpioCfg.pinFunc = PMIC_LP8764X_GPIO_PINFUNC_GPIO6_GPIO7_NERR_MCU;
    }

    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioGetConfiguration(pPmicCoreHandle, pin, &gpioCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(gpioCfg.pinFunc, gpioCfg_rd.pinFunc);

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pin = 6U;
        pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        pmicStatus = Pmic_gpioGetConfiguration(pPmicCoreHandle,
                                               pin,
                                               &gpioCfg_rd);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        TEST_ASSERT_EQUAL(gpioCfg.pinFunc, gpioCfg_rd.pinFunc);

    }
    pmic_testResultUpdate_pass(6199,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
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
        pmic_testResultUpdate_ignore(6200,
                                         pmic_gpio_tests,
                                         PMIC_GPIO_NUM_OF_TESTCASES);
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
    pmic_testResultUpdate_pass(6200,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
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
        pmic_testResultUpdate_ignore(6201,
                                         pmic_gpio_tests,
                                         PMIC_GPIO_NUM_OF_TESTCASES);
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
    pmic_testResultUpdate_pass(6201,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
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
         * GPIO5/ GPIO6 pin of PMIC-A is connected to LEOA_SCLK and LEOA_SDATA
         * which resets SOC
         */
         if(PMIC_SILICON_REV_ID_PG_2_0 == pPmicCoreHandle->pmicDevSiliconRev)
        {
            if((J721E_LEO_PMICA_DEVICE == pmic_device_info) &&
               ((5U == pins[pin]) || (6U == pins[pin])))
            {
                continue;
            }
        }

        /*!
        * On PG1.0 J721 EVM, PMICA GPIO10 SYNCCLKOUT functionality
        * is not supported
        */
        if(PMIC_SILICON_REV_ID_PG_1_0 == pPmicCoreHandle->pmicDevSiliconRev)
        {
            if((J721E_LEO_PMICA_DEVICE == pmic_device_info) &&
               (10U == pins[pin]))
            {
                continue;
            }
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
    pmic_testResultUpdate_pass(6202,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
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
    pmic_testResultUpdate_pass(6204,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
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
        pmic_testResultUpdate_ignore(6205,
                                         pmic_gpio_tests,
                                         PMIC_GPIO_NUM_OF_TESTCASES);
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(6205,
                                         pmic_gpio_tests,
                                         PMIC_GPIO_NUM_OF_TESTCASES);
    }

    for(pin = 0U; pin < sizeof(pins)/sizeof(pins[0U]); pin++)
    {
        /* PMICA GPIO3 pin Causing Reset on J721 EVM */
        if((3U == pins[pin]) &&
           (pmic_device_info == J721E_LEO_PMICA_DEVICE))
        {
            continue;
        }

        if((J7VCL_LEO_PMICA_DEVICE == pmic_device_info) &&
           ((3U == pins[pin]) || (5U == pins[pin]) || (6U == pins[pin])))
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
    pmic_testResultUpdate_pass(6205,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
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

    /*!
    * On PG1.0 J721 EVM, PMICA GPIO10 CLK32KOUT functionality
    * is not supported
    */
    if(PMIC_SILICON_REV_ID_PG_1_0 == pPmicCoreHandle->pmicDevSiliconRev)
    {
        if(pmic_device_info == J721E_LEO_PMICA_DEVICE)
        {
            pmic_testResultUpdate_ignore(6206,
                                         pmic_gpio_tests,
                                         PMIC_GPIO_NUM_OF_TESTCASES);
        }
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(6206,
                                         pmic_gpio_tests,
                                         PMIC_GPIO_NUM_OF_TESTCASES);
    }


    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioGetConfiguration(pPmicCoreHandle, pin, &gpioCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(gpioCfg.pinFunc, gpioCfg_rd.pinFunc);
    pmic_testResultUpdate_pass(6206,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
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
        pmic_testResultUpdate_ignore(6207,
                                         pmic_gpio_tests,
                                         PMIC_GPIO_NUM_OF_TESTCASES);
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(6207,
                                         pmic_gpio_tests,
                                         PMIC_GPIO_NUM_OF_TESTCASES);
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
    pmic_testResultUpdate_pass(6207,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
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
        pmic_testResultUpdate_ignore(6209,
                                         pmic_gpio_tests,
                                         PMIC_GPIO_NUM_OF_TESTCASES);
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

        if((J7VCL_HERA_PMICB_DEVICE == pmic_device_info) &&
           (9U == pins[pin]))
        {
            continue;
        }

        pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pins[pin], gpioCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        pmicStatus = Pmic_gpioGetConfiguration(pPmicCoreHandle, pins[pin], &gpioCfg_rd);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        TEST_ASSERT_EQUAL(gpioCfg.pinFunc, gpioCfg_rd.pinFunc);
    }
    pmic_testResultUpdate_pass(6209,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
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

    pmic_testResultUpdate_pass(6210,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
}

/*!
 * \brief   Parameter validation for pin for Max Value
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

    pmic_testResultUpdate_pass(6211,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
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

    pmic_testResultUpdate_pass(6213,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
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

    pmic_testResultUpdate_pass(6214,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
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

    pmic_testResultUpdate_pass(6215,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
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

    pmic_testResultUpdate_pass(6216,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
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
        pmic_testResultUpdate_ignore(6217,
                                     pmic_gpio_tests,
                                     PMIC_GPIO_NUM_OF_TESTCASES);
    }

    pmicStatus = Pmic_gpioSetNPwronEnablePinConfiguration(pPmicCoreHandle,
                                                          gpioCfg);
    TEST_ASSERT_EQUAL(pmicStatus, PMIC_ST_ERR_INV_GPIO_FUNC);

    pmic_testResultUpdate_pass(6217,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
}

/*!
 * \brief   Parameter validation for Enable-pin Polarity
 */
static void test_pmic_enable_setCfgPrmValTest_pinPolarity(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_GpioCfg_t gpioCfg    =
    {
        PMIC_ENABLE_CFG_POLARITY_VALID_SHIFT,
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

    pmic_testResultUpdate_pass(6218,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
}

/*!
 * \brief   Get required GPIO pin configuration
 */
static void test_pmic_gpio_getCfgGpioPin(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    int pin                   = 1U;
    Pmic_GpioCfg_t gpioCfg_rd = {PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT |
                                 PMIC_GPIO_CFG_DIR_VALID_SHIFT,};
    Pmic_GpioCfg_t gpioCfg    =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT | PMIC_GPIO_CFG_DIR_VALID_SHIFT,
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
        /* Should not break SPI Comm Lines */
        if((PMIC_INTF_SPI == pPmicCoreHandle->commMode) &&
           ((1U == pin) || (2U == pin)))
        {
            pmic_testResultUpdate_ignore(6219,
                                         pmic_gpio_tests,
                                         PMIC_GPIO_NUM_OF_TESTCASES);
        }
    }
    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        gpioCfg.pinFunc = PMIC_TPS6594X_GPIO_PINFUNC_NSLEEP1;
    }
    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pin = 2U;
        gpioCfg.pinFunc = PMIC_LP8764X_GPIO_PINFUNC_NSLEEP1;
    }

    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioGetConfiguration(pPmicCoreHandle, pin, &gpioCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(gpioCfg.pinFunc, gpioCfg_rd.pinFunc);
    TEST_ASSERT_EQUAL(gpioCfg.pinDir, gpioCfg_rd.pinDir);

    pmic_testResultUpdate_pass(6219,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
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

    pmic_testResultUpdate_pass(6220,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
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

    pmic_testResultUpdate_pass(6221,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
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

    pmic_testResultUpdate_pass(6222,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
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
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT |
        PMIC_GPIO_CFG_DIR_VALID_SHIFT |
        PMIC_GPIO_CFG_PULL_VALID_SHIFT |
        PMIC_GPIO_CFG_OD_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_PUSH_PULL_OUTPUT,
        PMIC_GPIO_PULL_UP,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_GPIO_PINFUNC_GPIO,
        PMIC_GPIO_HIGH
    };

    test_pmic_print_unity_testcase_info(6223,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        /* Should not break SPI Comm Lines */
        if((PMIC_INTF_SPI == pPmicCoreHandle->commMode) &&
           ((1U == pin) || (2U == pin)))
        {
            pmic_testResultUpdate_ignore(6223,
                                         pmic_gpio_tests,
                                         PMIC_GPIO_NUM_OF_TESTCASES);
        }
    }
    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        gpioCfg.pinFunc = PMIC_TPS6594X_GPIO_PINFUNC_GPIO;
    }
    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        gpioCfg.pinFunc = PMIC_LP8764X_GPIO_PINFUNC_GPIO;
    }

    if(J721E_LEO_PMICB_DEVICE == pmic_device_info)
    {
        pmic_testResultUpdate_ignore(6223,
                                         pmic_gpio_tests,
                                         PMIC_GPIO_NUM_OF_TESTCASES);
    }

    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioSetValue(pPmicCoreHandle, pin, pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioGetValue(pPmicCoreHandle, pin, &pinValue_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(pinValue, pinValue_rd);

    pmic_testResultUpdate_pass(6223,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
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

    pmic_testResultUpdate_pass(6224,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
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

    pmic_testResultUpdate_pass(6225,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
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

    pmic_testResultUpdate_pass(6226,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
}

/*!
 * \brief   Set GPIO signal level
 */
static void test_pmic_gpio_setValueGpioPin1_signalLevel(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    uint8_t pin               = 1U;
    uint8_t pinValue          = PMIC_GPIO_LOW;
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
        /* Should not break SPI Comm Lines */
        if((PMIC_INTF_SPI == pPmicCoreHandle->commMode) &&
           ((1U == pin) || (2U == pin)))
        {
            pmic_testResultUpdate_ignore(6227,
                                         pmic_gpio_tests,
                                         PMIC_GPIO_NUM_OF_TESTCASES);
        }
    }
    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        gpioCfg.pinFunc = PMIC_TPS6594X_GPIO_PINFUNC_GPIO;
    }
    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        gpioCfg.pinFunc = PMIC_LP8764X_GPIO_PINFUNC_GPIO;
    }
    if(J721E_LEO_PMICB_DEVICE == pmic_device_info)
    {
        pmic_testResultUpdate_ignore(6227,
                                         pmic_gpio_tests,
                                         PMIC_GPIO_NUM_OF_TESTCASES);
    }
    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    if(J7VCL_HERA_PMICB_DEVICE != pmic_device_info)
    {
        pmicStatus = Pmic_gpioSetValue(pPmicCoreHandle, pin, pinValue);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        pmicStatus = Pmic_gpioGetValue(pPmicCoreHandle, pin, &pinValue_rd);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        TEST_ASSERT_EQUAL(pinValue, pinValue_rd);
    }

    pinValue   = PMIC_GPIO_HIGH;
    pmicStatus = Pmic_gpioSetValue(pPmicCoreHandle, pin, pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioGetValue(pPmicCoreHandle, pin, &pinValue_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(pinValue, pinValue_rd);

    pmic_testResultUpdate_pass(6227,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
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

    pmic_testResultUpdate_pass(6228,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
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

    pmic_testResultUpdate_pass(6229,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
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

    pmic_testResultUpdate_pass(6230,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
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
        pin = 2U;
        gpioCfg.pinFunc = PMIC_LP8764X_GPIO_PINFUNC_GPIO;
    }
    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioGetConfiguration(pPmicCoreHandle, pin, &gpioCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(gpioCfg.pinDir, gpioCfg_rd.pinDir);

    pmicStatus = Pmic_gpioSetValue(pPmicCoreHandle, pin, pinValue);
    TEST_ASSERT_EQUAL(pmicStatus, PMIC_ST_ERR_INV_PARAM);

    pmic_testResultUpdate_pass(6231,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
}

/*!
 * \brief   Function to extract and clear interrupts
 */
static void pmic_gpioTest_intClr(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_IrqStatus_t errStat  = {0U};
    uint8_t irqNum = 0U;

    pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, false);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    while((errStat.intStatus[0] != 0U) || (errStat.intStatus[1] != 0U) ||
          (errStat.intStatus[2] != 0U) || (errStat.intStatus[3] != 0U))
    {
        pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                             &errStat,
                                             &irqNum);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        pmicStatus = Pmic_irqClrErrStatus(pPmicCoreHandle, irqNum);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    }
}

/*!
 * \brief   Test to verify GPIO1 fall interrupt
 */
static void test_pmic_gpio1_testFall_interrupt(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    int8_t timeout            = 10U;
    uint8_t pin               = 1U;
    uint8_t pinValue          = PMIC_GPIO_HIGH;
    uint8_t intrType          = PMIC_GPIO_FALL_INTERRUPT;
    uint8_t maskPol           = PMIC_GPIO_POL_LOW;
    Pmic_IrqStatus_t errStat  = {0U};
    bool clearIRQ             = false;
    uint8_t  irqNum           = 0U;
    uint8_t  nextedIrq        = 0U;
    uint8_t  intMask          = 0U;
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

    if(J7VCL_HERA_PMICB_DEVICE == pmic_device_info)
    {
        gpioCfg.validParams = PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT |
                              PMIC_GPIO_CFG_DIR_VALID_SHIFT |
                              PMIC_GPIO_CFG_OD_VALID_SHIFT |
                              PMIC_GPIO_CFG_PULL_VALID_SHIFT;
        gpioCfg.outputSignalType = PMIC_GPIO_PUSH_PULL_OUTPUT;
        gpioCfg.pullCtrl = PMIC_GPIO_PULL_UP;
        gpioCfg.pinFunc = PMIC_LP8764X_GPIO_PINFUNC_GPIO;
    }

    test_pmic_print_unity_testcase_info(6234,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        /* Should not break SPI Comm Lines */
        if(PMIC_INTF_SPI == pPmicCoreHandle->commMode)
        {
            pmic_testResultUpdate_ignore(6234,
                                         pmic_gpio_tests,
                                         PMIC_GPIO_NUM_OF_TESTCASES);
        }
    }
    if(J721E_LEO_PMICB_DEVICE == pmic_device_info)
    {
        pmic_testResultUpdate_ignore(6234,
                                         pmic_gpio_tests,
                                         PMIC_GPIO_NUM_OF_TESTCASES);
    }
    if(J7VCL_HERA_PMICB_DEVICE == pmic_device_info)
    {
        pmic_testResultUpdate_ignore(6234,
                                         pmic_gpio_tests,
                                         PMIC_GPIO_NUM_OF_TESTCASES);
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        intMask = PMIC_TPS6594X_IRQ_GPIO_1_INT_MASK_NUM;
        irqNum = PMIC_TPS6594X_GPIO1_INT;
    }
    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        intMask = PMIC_LP8764X_IRQ_GPIO_1_INT_MASK_NUM;
        irqNum = PMIC_LP8764X_GPIO1_INT;
    }
    /* Un Masking GPIO 1 FALL Interrupt */
    Pmic_irqGpioMaskIntr(pPmicCoreHandle,
                         intMask,
                         PMIC_IRQ_UNMASK,
                         PMIC_IRQ_GPIO_FALL_INT_TYPE);

    /* To clear the interrupts*/
    pmic_gpioTest_intClr();

    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioSetValue(pPmicCoreHandle, pin, pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioSetIntr(pPmicCoreHandle, pin , intrType, maskPol);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pinValue = PMIC_GPIO_LOW;
    pmicStatus = Pmic_gpioSetValue(pPmicCoreHandle, pin, pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    while(timeout--)
    {
        /* Delay added to avoid timeout */
        Osal_delay(1000);
        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        if((PMIC_ST_SUCCESS == pmicStatus) &&
           ((errStat.intStatus[irqNum/32U] & (1U << (irqNum % 32U))) != 0U))
        {
            while(irqNum != nextedIrq)
            {
                pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                                     &errStat,
                                                     &nextedIrq);
            }

            if(PMIC_ST_SUCCESS == pmicStatus)
            {
                /* clear the interrupt */
                pmicStatus = Pmic_irqClrErrStatus(pPmicCoreHandle,
                                                  irqNum);
                break;
            }
        }
    }

    if(0 > timeout)
    {
        pmicStatus = PMIC_ST_ERR_FAIL;
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmic_testResultUpdate_pass(6234,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
}

/*!
 * \brief   Test to verify GPIO1 rise interrupt
 */
static void test_pmic_gpio1_testRise_interrupt(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    int8_t timeout            = 10U;
    uint8_t pin               = 1U;
    uint8_t pinValue          = PMIC_GPIO_LOW;
    uint8_t intrType          = PMIC_GPIO_RISE_INTERRUPT;
    uint8_t maskPol           = PMIC_GPIO_POL_HIGH;
    Pmic_IrqStatus_t errStat  = {0U};
    bool clearIRQ             = false;
    uint8_t  irqNum           = 0U;
    uint8_t  nextedIrq        = 0U;
    uint8_t  intMask          = 0U;
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

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        /* Should not break SPI Comm Lines */
        if(PMIC_INTF_SPI == pPmicCoreHandle->commMode)
        {
            pmic_testResultUpdate_ignore(6235,
                                         pmic_gpio_tests,
                                         PMIC_GPIO_NUM_OF_TESTCASES);
        }
    }
    if(J721E_LEO_PMICB_DEVICE == pmic_device_info)
    {
        pmic_testResultUpdate_ignore(6235,
                                         pmic_gpio_tests,
                                         PMIC_GPIO_NUM_OF_TESTCASES);
    }
    if(J7VCL_HERA_PMICB_DEVICE == pmic_device_info)
    {
        pmic_testResultUpdate_ignore(6235,
                                         pmic_gpio_tests,
                                         PMIC_GPIO_NUM_OF_TESTCASES);
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        intMask = PMIC_TPS6594X_IRQ_GPIO_1_INT_MASK_NUM;
        irqNum = PMIC_TPS6594X_GPIO1_INT;
    }
    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        intMask = PMIC_LP8764X_IRQ_GPIO_1_INT_MASK_NUM;
        irqNum = PMIC_LP8764X_GPIO1_INT;
    }
    /* Un Masking GPIO 1 RISE Interrupt */
    Pmic_irqGpioMaskIntr(pPmicCoreHandle,
                         intMask,
                         PMIC_IRQ_UNMASK,
                         PMIC_IRQ_GPIO_RISE_INT_TYPE);

    /* To clear the interrupts*/
    pmic_gpioTest_intClr();

    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioSetValue(pPmicCoreHandle, pin, pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioSetIntr(pPmicCoreHandle, pin , intrType, maskPol);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pinValue = PMIC_GPIO_HIGH;
    pmicStatus = Pmic_gpioSetValue(pPmicCoreHandle, pin, pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    while(timeout--)
    {
        /* Delay added to avoid timeout */
        Osal_delay(1000);
        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        if((PMIC_ST_SUCCESS == pmicStatus) &&
           ((errStat.intStatus[irqNum/32U] & (1U << (irqNum % 32U))) != 0U))
        {
            while(irqNum != nextedIrq)
            {
                pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                                     &errStat,
                                                     &nextedIrq);
            }

            if(PMIC_ST_SUCCESS == pmicStatus)
            {
                /* clear the interrupt */
                pmicStatus = Pmic_irqClrErrStatus(pPmicCoreHandle,
                                                  irqNum);
                break;
            }
        }
    }

    if(0 > timeout)
    {
        pmicStatus = PMIC_ST_ERR_FAIL;
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmic_testResultUpdate_pass(6235,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
}

/*!
 * \brief   Test to verify GPIO2 fall interrupt
 */
static void test_pmic_gpio2_testFall_interrupt(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    int8_t timeout            = 10U;
    uint8_t pin               = 2U;
    uint8_t pinValue          = PMIC_GPIO_HIGH;
    uint8_t intrType          = PMIC_GPIO_FALL_INTERRUPT;
    uint8_t maskPol           = PMIC_GPIO_POL_LOW;
    Pmic_IrqStatus_t errStat  = {0U};
    bool clearIRQ             = false;
    uint8_t  irqNum           = 0U;
    uint8_t  nextedIrq        = 0U;
    uint8_t  intMask          = 0U;
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

    if(J7VCL_HERA_PMICB_DEVICE == pmic_device_info)
    {
        gpioCfg.validParams = PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT |
                              PMIC_GPIO_CFG_DIR_VALID_SHIFT |
                              PMIC_GPIO_CFG_OD_VALID_SHIFT |
                              PMIC_GPIO_CFG_PULL_VALID_SHIFT;
        gpioCfg.outputSignalType = PMIC_GPIO_PUSH_PULL_OUTPUT;
        gpioCfg.pullCtrl = PMIC_GPIO_PULL_UP;
        gpioCfg.pinFunc = PMIC_LP8764X_GPIO_PINFUNC_GPIO;
    }

    test_pmic_print_unity_testcase_info(6236,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        /* Should not break SPI Comm Lines */
        if(PMIC_INTF_SPI == pPmicCoreHandle->commMode)
        {
            pmic_testResultUpdate_ignore(6236,
                                         pmic_gpio_tests,
                                         PMIC_GPIO_NUM_OF_TESTCASES);
        }
    }
    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        /* Should not break SPI Comm Lines */
        if(PMIC_INTF_SPI == pPmicCoreHandle->commMode)
        {
            pmic_testResultUpdate_ignore(6236,
                                         pmic_gpio_tests,
                                         PMIC_GPIO_NUM_OF_TESTCASES);
        }
    }
    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        intMask = PMIC_TPS6594X_IRQ_GPIO_2_INT_MASK_NUM;
        irqNum = PMIC_TPS6594X_GPIO2_INT;
    }
    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        intMask = PMIC_LP8764X_IRQ_GPIO_2_INT_MASK_NUM;
        irqNum = PMIC_LP8764X_GPIO2_INT;
    }

    /* Un Masking GPIO 2 FALL Interrupt */
    Pmic_irqGpioMaskIntr(pPmicCoreHandle,
                         intMask,
                         PMIC_IRQ_UNMASK,
                         PMIC_IRQ_GPIO_FALL_INT_TYPE);

    /* To clear the interrupts*/
    pmic_gpioTest_intClr();

    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioSetValue(pPmicCoreHandle, pin, pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioSetIntr(pPmicCoreHandle, pin , intrType, maskPol);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pinValue = PMIC_GPIO_LOW;
    pmicStatus = Pmic_gpioSetValue(pPmicCoreHandle, pin, pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    while(timeout--)
    {
        /* Delay added to avoid timeout */
        Osal_delay(1000);
        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        if((PMIC_ST_SUCCESS == pmicStatus) &&
           ((errStat.intStatus[irqNum/32U] & (1U << (irqNum % 32U))) != 0U))
        {
            while(irqNum != nextedIrq)
            {
                pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                                     &errStat,
                                                     &nextedIrq);
            }

            if(PMIC_ST_SUCCESS == pmicStatus)
            {
                /* clear the interrupt */
                pmicStatus = Pmic_irqClrErrStatus(pPmicCoreHandle,
                                                  irqNum);
                break;
            }
        }
    }

    if(0 > timeout)
    {
        pmicStatus = PMIC_ST_ERR_FAIL;
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmic_testResultUpdate_pass(6236,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
}

/*!
 * \brief   Test to verify GPIO2 rise interrupt
 */
static void test_pmic_gpio2_testRise_interrupt(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    int8_t timeout            = 10U;
    uint8_t pin               = 2U;
    uint8_t pinValue          = PMIC_GPIO_LOW;
    uint8_t intrType          = PMIC_GPIO_RISE_INTERRUPT;
    uint8_t maskPol           = PMIC_GPIO_POL_HIGH;
    Pmic_IrqStatus_t errStat  = {0U};
    bool clearIRQ             = false;
    uint8_t  irqNum           = 0U;
    uint8_t  nextedIrq        = 0U;
    uint8_t  intMask          = 0U;
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

    if(J7VCL_HERA_PMICB_DEVICE == pmic_device_info)
    {
        gpioCfg.validParams = PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT |
                              PMIC_GPIO_CFG_DIR_VALID_SHIFT |
                              PMIC_GPIO_CFG_OD_VALID_SHIFT |
                              PMIC_GPIO_CFG_PULL_VALID_SHIFT;
        gpioCfg.outputSignalType = PMIC_GPIO_PUSH_PULL_OUTPUT;
        gpioCfg.pullCtrl = PMIC_GPIO_PULL_UP;
        gpioCfg.pinFunc = PMIC_LP8764X_GPIO_PINFUNC_GPIO;
    }

    test_pmic_print_unity_testcase_info(6237,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        /* Should not break SPI Comm Lines */
        if(PMIC_INTF_SPI == pPmicCoreHandle->commMode)
        {
            pmic_testResultUpdate_ignore(6237,
                                         pmic_gpio_tests,
                                         PMIC_GPIO_NUM_OF_TESTCASES);
        }
    }
    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        /* Should not break SPI Comm Lines */
        if(PMIC_INTF_SPI == pPmicCoreHandle->commMode)
        {
            pmic_testResultUpdate_ignore(6237,
                                         pmic_gpio_tests,
                                         PMIC_GPIO_NUM_OF_TESTCASES);
        }
    }
    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        intMask = PMIC_TPS6594X_IRQ_GPIO_2_INT_MASK_NUM;
        irqNum = PMIC_TPS6594X_GPIO2_INT;
    }
    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        intMask = PMIC_LP8764X_IRQ_GPIO_2_INT_MASK_NUM;
        irqNum = PMIC_LP8764X_GPIO2_INT;
    }

    /* Un Masking GPIO 2 RISE Interrupt */
    Pmic_irqGpioMaskIntr(pPmicCoreHandle,
                         intMask,
                         PMIC_IRQ_UNMASK,
                         PMIC_IRQ_GPIO_RISE_INT_TYPE);

    /* To clear the interrupts*/
    pmic_gpioTest_intClr();

    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioSetValue(pPmicCoreHandle, pin, pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioSetIntr(pPmicCoreHandle, pin , intrType, maskPol);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pinValue = PMIC_GPIO_HIGH;
    pmicStatus = Pmic_gpioSetValue(pPmicCoreHandle, pin, pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    while(timeout--)
    {
        /* Delay added to avoid timeout */
        Osal_delay(1000);
        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        if((PMIC_ST_SUCCESS == pmicStatus) &&
           ((errStat.intStatus[irqNum/32U] & (1U << (irqNum % 32U))) != 0U))
        {
            while(irqNum != nextedIrq)
            {
                pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                                     &errStat,
                                                     &nextedIrq);
            }

            if(PMIC_ST_SUCCESS == pmicStatus)
            {
                /* clear the interrupt */
                pmicStatus = Pmic_irqClrErrStatus(pPmicCoreHandle,
                                                  irqNum);
                break;
            }
        }
    }

    if(0 > timeout)
    {
        pmicStatus = PMIC_ST_ERR_FAIL;
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmic_testResultUpdate_pass(6237,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
}

/*!
 * \brief   Test to verify GPIO4 fall interrupt
 */
static void test_pmic_gpio4_testFall_interrupt(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    int8_t timeout            = 10U;
    uint8_t pin               = 4U;
    uint8_t pinValue          = PMIC_GPIO_HIGH;
    uint8_t intrType          = PMIC_GPIO_FALL_INTERRUPT;
    uint8_t maskPol           = PMIC_GPIO_POL_LOW;
    Pmic_IrqStatus_t errStat  = {0U};
    bool clearIRQ             = false;
    uint8_t  irqNum           = 0U;
    uint8_t  nextedIrq        = 0U;
    uint8_t  intMask          = 0U;
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

    if(J721E_LEO_PMICB_DEVICE == pmic_device_info)
    {
        /*
         * GPIO4 pin of PMIC-B is connected to 'H_DDR_RET_1V1'
         * on J721EVM board causing hang, when programming fall interrupt.
         */
        pmic_testResultUpdate_ignore(6240,
                                         pmic_gpio_tests,
                                         PMIC_GPIO_NUM_OF_TESTCASES);
    }

    if(J7VCL_LEO_PMICA_DEVICE == pmic_device_info)
    {
        pmic_testResultUpdate_ignore(6240,
                                         pmic_gpio_tests,
                                         PMIC_GPIO_NUM_OF_TESTCASES);
    }

    if(J7VCL_HERA_PMICB_DEVICE == pmic_device_info)
    {
        gpioCfg.validParams = PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT |
                              PMIC_GPIO_CFG_DIR_VALID_SHIFT |
                              PMIC_GPIO_CFG_OD_VALID_SHIFT |
                              PMIC_GPIO_CFG_PULL_VALID_SHIFT;
        gpioCfg.outputSignalType = PMIC_GPIO_PUSH_PULL_OUTPUT;
        gpioCfg.pullCtrl = PMIC_GPIO_PULL_UP;
        gpioCfg.pinFunc = PMIC_LP8764X_GPIO_PINFUNC_GPIO;
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        intMask = PMIC_TPS6594X_IRQ_GPIO_4_INT_MASK_NUM;
        irqNum = PMIC_TPS6594X_GPIO4_INT;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        intMask = PMIC_LP8764X_IRQ_GPIO_4_INT_MASK_NUM;
        irqNum = PMIC_LP8764X_GPIO4_INT;
    }

    /* Un Masking GPIO 4 FALL Interrupt */
    Pmic_irqGpioMaskIntr(pPmicCoreHandle,
                         intMask,
                         PMIC_IRQ_UNMASK,
                         PMIC_IRQ_GPIO_FALL_INT_TYPE);

    /* To clear the interrupts*/
    pmic_gpioTest_intClr();

    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioSetValue(pPmicCoreHandle, pin, pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioSetIntr(pPmicCoreHandle, pin , intrType, maskPol);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pinValue = PMIC_GPIO_LOW;
    pmicStatus = Pmic_gpioSetValue(pPmicCoreHandle, pin, pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    while(timeout--)
    {
        /* Delay added to avoid timeout */
        Osal_delay(1000);
        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        if((PMIC_ST_SUCCESS == pmicStatus) &&
           ((errStat.intStatus[irqNum/32U] & (1U << (irqNum % 32U))) != 0U))
        {
            while(irqNum != nextedIrq)
            {
                pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                                     &errStat,
                                                     &nextedIrq);
            }

            if(PMIC_ST_SUCCESS == pmicStatus)
            {
                /* clear the interrupt */
                pmicStatus = Pmic_irqClrErrStatus(pPmicCoreHandle,
                                                  irqNum);
                break;
            }
        }
    }

    if(0 > timeout)
    {
        pmicStatus = PMIC_ST_ERR_FAIL;
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmic_testResultUpdate_pass(6240,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
}

/*!
 * \brief   Test to verify GPIO4 rise interrupt
 */
static void test_pmic_gpio4_testRise_interrupt(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    int8_t timeout            = 10U;
    uint8_t pin               = 4U;
    uint8_t pinValue          = PMIC_GPIO_LOW;
    uint8_t intrType          = PMIC_GPIO_RISE_INTERRUPT;
    uint8_t maskPol           = PMIC_GPIO_POL_HIGH;
    Pmic_IrqStatus_t errStat  = {0U};
    bool clearIRQ             = false;
    uint8_t  irqNum           = 0U;
    uint8_t  nextedIrq        = 0U;
    uint8_t  intMask          = 0U;
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

    if(J721E_LEO_PMICB_DEVICE == pmic_device_info)
    {
        /*
         * GPIO4 pin of PMIC-B is connected to 'H_DDR_RET_1V1'
         * on J721EVM board causing hang, when programming rise interrupt.
         */
        pmic_testResultUpdate_ignore(6241,
                                         pmic_gpio_tests,
                                         PMIC_GPIO_NUM_OF_TESTCASES);
    }

    if(J7VCL_LEO_PMICA_DEVICE == pmic_device_info)
    {
        pmic_testResultUpdate_ignore(6241,
                                         pmic_gpio_tests,
                                         PMIC_GPIO_NUM_OF_TESTCASES);
    }

    if(J7VCL_HERA_PMICB_DEVICE == pmic_device_info)
    {
        gpioCfg.validParams = PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT |
                              PMIC_GPIO_CFG_DIR_VALID_SHIFT |
                              PMIC_GPIO_CFG_OD_VALID_SHIFT |
                              PMIC_GPIO_CFG_PULL_VALID_SHIFT;
        gpioCfg.outputSignalType = PMIC_GPIO_PUSH_PULL_OUTPUT;
        gpioCfg.pullCtrl = PMIC_GPIO_PULL_UP;
        gpioCfg.pinFunc = PMIC_LP8764X_GPIO_PINFUNC_GPIO;
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        intMask = PMIC_TPS6594X_IRQ_GPIO_4_INT_MASK_NUM;
        irqNum = PMIC_TPS6594X_GPIO4_INT;
    }
    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        intMask = PMIC_LP8764X_IRQ_GPIO_4_INT_MASK_NUM;
        irqNum = PMIC_LP8764X_GPIO4_INT;
    }

    /* Un Masking GPIO 4 RISE Interrupt */
    Pmic_irqGpioMaskIntr(pPmicCoreHandle,
                         intMask,
                         PMIC_IRQ_UNMASK,
                         PMIC_IRQ_GPIO_RISE_INT_TYPE);

    /* To clear the interrupts*/
    pmic_gpioTest_intClr();

    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioSetValue(pPmicCoreHandle, pin, pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioSetIntr(pPmicCoreHandle, pin , intrType, maskPol);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pinValue = PMIC_GPIO_HIGH;
    pmicStatus = Pmic_gpioSetValue(pPmicCoreHandle, pin, pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    while(timeout--)
    {
        /* Delay added to avoid timeout */
        Osal_delay(1000);
        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        if((PMIC_ST_SUCCESS == pmicStatus) &&
           ((errStat.intStatus[irqNum/32U] & (1U << (irqNum % 32U))) != 0U))
        {
            while(irqNum != nextedIrq)
            {
                pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                                     &errStat,
                                                     &nextedIrq);
            }

            if(PMIC_ST_SUCCESS == pmicStatus)
            {
                /* clear the interrupt */
                pmicStatus = Pmic_irqClrErrStatus(pPmicCoreHandle,
                                                  irqNum);
                break;
            }
        }
    }

    if(0 > timeout)
    {
        pmicStatus = PMIC_ST_ERR_FAIL;
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmic_testResultUpdate_pass(6241,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
}

/*!
 * \brief   Test to verify GPIO5 fall interrupt
 */
static void test_pmic_gpio5_testFall_interrupt(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    int8_t timeout            = 10U;
    uint8_t pin               = 5U;
    uint8_t pinValue          = PMIC_GPIO_HIGH;
    uint8_t intrType          = PMIC_GPIO_FALL_INTERRUPT;
    uint8_t maskPol           = PMIC_GPIO_POL_LOW;
    Pmic_IrqStatus_t errStat  = {0U};
    bool clearIRQ             = false;
    uint8_t  irqNum           = 0U;
    uint8_t  nextedIrq        = 0U;
    uint8_t  intMask          = 0U;
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

    if((J721E_LEO_PMICA_DEVICE == pmic_device_info) ||
       (J721E_LEO_PMICB_DEVICE == pmic_device_info))
    {
        /*
         * GPIO5 pin of PMIC-A and PMIC-B is connected to 'LEOA_SCLK'
         * on J721EVM causing block, when programming fall interrupt signal.
         */
        pmic_testResultUpdate_ignore(6242,
                                         pmic_gpio_tests,
                                         PMIC_GPIO_NUM_OF_TESTCASES);
    }

    if(J7VCL_LEO_PMICA_DEVICE == pmic_device_info)
    {
        pmic_testResultUpdate_ignore(6242,
                                         pmic_gpio_tests,
                                         PMIC_GPIO_NUM_OF_TESTCASES);
    }

    if(J7VCL_HERA_PMICB_DEVICE == pmic_device_info)
    {
        gpioCfg.validParams = PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT |
                              PMIC_GPIO_CFG_DIR_VALID_SHIFT |
                              PMIC_GPIO_CFG_OD_VALID_SHIFT |
                              PMIC_GPIO_CFG_PULL_VALID_SHIFT;
        gpioCfg.outputSignalType = PMIC_GPIO_PUSH_PULL_OUTPUT;
        gpioCfg.pullCtrl = PMIC_GPIO_PULL_UP;
        gpioCfg.pinFunc = PMIC_LP8764X_GPIO_PINFUNC_GPIO;
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        intMask = PMIC_TPS6594X_IRQ_GPIO_5_INT_MASK_NUM;
        irqNum = PMIC_TPS6594X_GPIO5_INT;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        intMask = PMIC_LP8764X_IRQ_GPIO_5_INT_MASK_NUM;
        irqNum = PMIC_LP8764X_GPIO5_INT;
    }

    /* Un Masking GPIO 5 FALL Interrupt */
    Pmic_irqGpioMaskIntr(pPmicCoreHandle,
                         intMask,
                         PMIC_IRQ_UNMASK,
                         PMIC_IRQ_GPIO_FALL_INT_TYPE);

    /* To clear the interrupts*/
    pmic_gpioTest_intClr();

    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioSetValue(pPmicCoreHandle, pin, pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioSetIntr(pPmicCoreHandle, pin , intrType, maskPol);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pinValue = PMIC_GPIO_LOW;
    pmicStatus = Pmic_gpioSetValue(pPmicCoreHandle, pin, pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    while(timeout--)
    {
        /* Delay added to avoid timeout */
        Osal_delay(1000);
        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        if((PMIC_ST_SUCCESS == pmicStatus) &&
           ((errStat.intStatus[irqNum/32U] & (1U << (irqNum % 32U))) != 0U))
        {
            while(irqNum != nextedIrq)
            {
                pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                                     &errStat,
                                                     &nextedIrq);
            }

            if(PMIC_ST_SUCCESS == pmicStatus)
            {
                /* clear the interrupt */
                pmicStatus = Pmic_irqClrErrStatus(pPmicCoreHandle,
                                                  irqNum);
                break;
            }
        }
    }

    if(0 > timeout)
    {
        pmicStatus = PMIC_ST_ERR_FAIL;
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmic_testResultUpdate_pass(6242,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
}

/*!
 * \brief   Test to verify GPIO5 rise interrupt
 */
static void test_pmic_gpio5_testRise_interrupt(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    int8_t timeout            = 10U;
    uint8_t pin               = 5U;
    uint8_t pinValue          = PMIC_GPIO_LOW;
    uint8_t intrType          = PMIC_GPIO_RISE_INTERRUPT;
    uint8_t maskPol           = PMIC_GPIO_POL_HIGH;
    Pmic_IrqStatus_t errStat  = {0U};
    bool clearIRQ             = false;
    uint8_t  irqNum           = 0U;
    uint8_t  nextedIrq        = 0U;
    uint8_t  intMask          = 0U;
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

    if((J721E_LEO_PMICA_DEVICE == pmic_device_info) ||
       (J721E_LEO_PMICB_DEVICE == pmic_device_info))
    {
        /*
         * GPIO5 pin of PMIC-A and PMIC-B is connected to 'LEOA_SCLK'
         * on J721EVM causing block, when programming rise` interrupt signal.
         */
        pmic_testResultUpdate_ignore(6243,
                                         pmic_gpio_tests,
                                         PMIC_GPIO_NUM_OF_TESTCASES);
    }

    if(J7VCL_LEO_PMICA_DEVICE == pmic_device_info)
    {
        pmic_testResultUpdate_ignore(6243,
                                         pmic_gpio_tests,
                                         PMIC_GPIO_NUM_OF_TESTCASES);
    }

    if(J7VCL_HERA_PMICB_DEVICE == pmic_device_info)
    {
        gpioCfg.validParams = PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT |
                              PMIC_GPIO_CFG_DIR_VALID_SHIFT |
                              PMIC_GPIO_CFG_OD_VALID_SHIFT |
                              PMIC_GPIO_CFG_PULL_VALID_SHIFT;
        gpioCfg.outputSignalType = PMIC_GPIO_PUSH_PULL_OUTPUT;
        gpioCfg.pullCtrl = PMIC_GPIO_PULL_UP;
        gpioCfg.pinFunc = PMIC_LP8764X_GPIO_PINFUNC_GPIO;
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        intMask = PMIC_TPS6594X_IRQ_GPIO_5_INT_MASK_NUM;
        irqNum = PMIC_TPS6594X_GPIO5_INT;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        intMask = PMIC_LP8764X_IRQ_GPIO_5_INT_MASK_NUM;
        irqNum = PMIC_LP8764X_GPIO5_INT;
    }

    /* Un Masking GPIO 5 RISE Interrupt */
    Pmic_irqGpioMaskIntr(pPmicCoreHandle,
                         intMask,
                         PMIC_IRQ_UNMASK,
                         PMIC_IRQ_GPIO_RISE_INT_TYPE);

    /* To clear the interrupts*/
    pmic_gpioTest_intClr();

    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioSetValue(pPmicCoreHandle, pin, pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioSetIntr(pPmicCoreHandle, pin , intrType, maskPol);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pinValue = PMIC_GPIO_HIGH;
    pmicStatus = Pmic_gpioSetValue(pPmicCoreHandle, pin, pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    while(timeout--)
    {
        /* Delay added to avoid timeout */
        Osal_delay(1000);
        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        if((PMIC_ST_SUCCESS == pmicStatus) &&
           ((errStat.intStatus[irqNum/32U] & (1U << (irqNum % 32U))) != 0U))
        {
            while(irqNum != nextedIrq)
            {
                pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                                     &errStat,
                                                     &nextedIrq);
            }

            if(PMIC_ST_SUCCESS == pmicStatus)
            {
                /* clear the interrupt */
                pmicStatus = Pmic_irqClrErrStatus(pPmicCoreHandle,
                                                  irqNum);
                break;
            }
        }
    }

    if(0 > timeout)
    {
        pmicStatus = PMIC_ST_ERR_FAIL;
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmic_testResultUpdate_pass(6243,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
}

/*!
 * \brief   Test to verify GPIO6 fall interrupt
 */
static void test_pmic_gpio6_testFall_interrupt(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    int8_t timeout            = 10U;
    uint8_t pin               = 6U;
    uint8_t pinValue          = PMIC_GPIO_HIGH;
    uint8_t intrType          = PMIC_GPIO_FALL_INTERRUPT;
    uint8_t maskPol           = PMIC_GPIO_POL_LOW;
    Pmic_IrqStatus_t errStat  = {0U};
    bool clearIRQ             = false;
    uint8_t  irqNum           = 0U;
    uint8_t  nextedIrq        = 0U;
    uint8_t  intMask          = 0U;
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

    if((J721E_LEO_PMICA_DEVICE == pmic_device_info) ||
       (J721E_LEO_PMICB_DEVICE == pmic_device_info))
    {
        /*
         * GPIO6 pin of PMIC-A and PMIC-B is connected to 'LEOA_SDATA' and receives
         * on J721EVM causing hang, when programming fall interrupt signal.
         */
        pmic_testResultUpdate_ignore(6244,
                                         pmic_gpio_tests,
                                         PMIC_GPIO_NUM_OF_TESTCASES);
    }

    if(J7VCL_LEO_PMICA_DEVICE == pmic_device_info)
    {
        pmic_testResultUpdate_ignore(6244,
                                         pmic_gpio_tests,
                                         PMIC_GPIO_NUM_OF_TESTCASES);
    }

    if(J7VCL_HERA_PMICB_DEVICE == pmic_device_info)
    {
        gpioCfg.validParams = PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT |
                              PMIC_GPIO_CFG_DIR_VALID_SHIFT |
                              PMIC_GPIO_CFG_OD_VALID_SHIFT |
                              PMIC_GPIO_CFG_PULL_VALID_SHIFT;
        gpioCfg.outputSignalType = PMIC_GPIO_PUSH_PULL_OUTPUT;
        gpioCfg.pullCtrl = PMIC_GPIO_PULL_UP;
        gpioCfg.pinFunc = PMIC_LP8764X_GPIO_PINFUNC_GPIO;
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        intMask = PMIC_TPS6594X_IRQ_GPIO_6_INT_MASK_NUM;
        irqNum = PMIC_TPS6594X_GPIO6_INT;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        intMask = PMIC_LP8764X_IRQ_GPIO_6_INT_MASK_NUM;
        irqNum = PMIC_LP8764X_GPIO6_INT;
    }

    /* Un Masking GPIO 6 FALL Interrupt */
    Pmic_irqGpioMaskIntr(pPmicCoreHandle,
                         intMask,
                         PMIC_IRQ_UNMASK,
                         PMIC_IRQ_GPIO_FALL_INT_TYPE);

    /* To clear the interrupts*/
    pmic_gpioTest_intClr();

    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioSetValue(pPmicCoreHandle, pin, pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioSetIntr(pPmicCoreHandle, pin , intrType, maskPol);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pinValue = PMIC_GPIO_LOW;
    pmicStatus = Pmic_gpioSetValue(pPmicCoreHandle, pin, pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    while(timeout--)
    {
        /* Delay added to avoid timeout */
        Osal_delay(1000);
        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        if((PMIC_ST_SUCCESS == pmicStatus) &&
           ((errStat.intStatus[irqNum/32U] & (1U << (irqNum % 32U))) != 0U))
        {
            while(irqNum != nextedIrq)
            {
                pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                                     &errStat,
                                                     &nextedIrq);
            }

            if(PMIC_ST_SUCCESS == pmicStatus)
            {
                /* clear the interrupt */
                pmicStatus = Pmic_irqClrErrStatus(pPmicCoreHandle,
                                                  irqNum);
                break;
            }
        }
    }

    if(0 > timeout)
    {
        pmicStatus = PMIC_ST_ERR_FAIL;
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmic_testResultUpdate_pass(6244,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
}

/*!
 * \brief   Test to verify GPIO6 rise interrupt
 */
static void test_pmic_gpio6_testRise_interrupt(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    int8_t timeout            = 10U;
    uint8_t pin               = 6U;
    uint8_t pinValue          = PMIC_GPIO_LOW;
    uint8_t intrType          = PMIC_GPIO_RISE_INTERRUPT;
    uint8_t maskPol           = PMIC_GPIO_POL_HIGH;
    Pmic_IrqStatus_t errStat  = {0U};
    bool clearIRQ             = false;
    uint8_t  irqNum           = 0U;
    uint8_t  nextedIrq        = 0U;
    uint8_t  intMask          = 0U;
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

    if((J721E_LEO_PMICA_DEVICE == pmic_device_info) ||
       (J721E_LEO_PMICB_DEVICE == pmic_device_info))
    {
        /*
         * GPIO6 pin of PMIC-A and PMIC-B is connected to 'LEOA_SDATA' and receives
         * on J721EVM causing hang, when programming rise interrupt signal.
         */
        pmic_testResultUpdate_ignore(6245,
                                         pmic_gpio_tests,
                                         PMIC_GPIO_NUM_OF_TESTCASES);
    }

    if(J7VCL_LEO_PMICA_DEVICE == pmic_device_info)
    {
        pmic_testResultUpdate_ignore(6245,
                                         pmic_gpio_tests,
                                         PMIC_GPIO_NUM_OF_TESTCASES);
    }

    if(J7VCL_HERA_PMICB_DEVICE == pmic_device_info)
    {
        gpioCfg.validParams = PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT |
                              PMIC_GPIO_CFG_DIR_VALID_SHIFT |
                              PMIC_GPIO_CFG_OD_VALID_SHIFT |
                              PMIC_GPIO_CFG_PULL_VALID_SHIFT;
        gpioCfg.outputSignalType = PMIC_GPIO_PUSH_PULL_OUTPUT;
        gpioCfg.pullCtrl = PMIC_GPIO_PULL_UP;
        gpioCfg.pinFunc = PMIC_LP8764X_GPIO_PINFUNC_GPIO;
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        intMask = PMIC_TPS6594X_IRQ_GPIO_6_INT_MASK_NUM;
        irqNum = PMIC_TPS6594X_GPIO6_INT;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        intMask = PMIC_LP8764X_IRQ_GPIO_6_INT_MASK_NUM;
        irqNum = PMIC_LP8764X_GPIO6_INT;
    }

    /* Un Masking GPIO 6 RISE Interrupt */
    Pmic_irqGpioMaskIntr(pPmicCoreHandle,
                         intMask,
                         PMIC_IRQ_UNMASK,
                         PMIC_IRQ_GPIO_RISE_INT_TYPE);

    /* To clear the interrupts*/
    pmic_gpioTest_intClr();

    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioSetValue(pPmicCoreHandle, pin, pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioSetIntr(pPmicCoreHandle, pin , intrType, maskPol);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pinValue = PMIC_GPIO_HIGH;
    pmicStatus = Pmic_gpioSetValue(pPmicCoreHandle, pin, pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    while(timeout--)
    {
        /* Delay added to avoid timeout */
        Osal_delay(1000);
        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        if((PMIC_ST_SUCCESS == pmicStatus) &&
           ((errStat.intStatus[irqNum/32U] & (1U << (irqNum % 32U))) != 0U))
        {
            while(irqNum != nextedIrq)
            {
                pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                                     &errStat,
                                                     &nextedIrq);
            }

            if(PMIC_ST_SUCCESS == pmicStatus)
            {
                /* clear the interrupt */
                pmicStatus = Pmic_irqClrErrStatus(pPmicCoreHandle,
                                                  irqNum);
                break;
            }
        }
    }

    if(0 > timeout)
    {
        pmicStatus = PMIC_ST_ERR_FAIL;
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmic_testResultUpdate_pass(6245,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
}

/*!
 * \brief   Test to verify GPIO7 fall interrupt
 */
static void test_pmic_gpio7_testFall_interrupt(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    int8_t timeout            = 10U;
    uint8_t pin               = 7U;
    uint8_t pinValue          = PMIC_GPIO_HIGH;
    uint8_t intrType          = PMIC_GPIO_FALL_INTERRUPT;
    uint8_t maskPol           = PMIC_GPIO_POL_LOW;
    Pmic_IrqStatus_t errStat  = {0U};
    bool clearIRQ             = false;
    uint8_t  irqNum           = 0U;
    uint8_t  nextedIrq        = 0U;
    uint8_t  intMask          = 0U;
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

    if(J721E_LEO_PMICA_DEVICE == pmic_device_info)
    {
        /*
         * GPIO7 pin of PMIC-A is connected to 'SOC_SAFETY_ERRZ'
         * on J721EVM board causing hang, when programming fall interrupt.
         */
        pmic_testResultUpdate_ignore(6246,
                                         pmic_gpio_tests,
                                         PMIC_GPIO_NUM_OF_TESTCASES);
    }
    if(J721E_LEO_PMICB_DEVICE == pmic_device_info)
    {
        /*
         * GPIO7 pin of PMIC-B is connected to 'EN_RAM0V85_LDO'
         * on J721EVM board causing hang, when programming fall interrupt.
         */
        pmic_testResultUpdate_ignore(6246,
                                         pmic_gpio_tests,
                                         PMIC_GPIO_NUM_OF_TESTCASES);
    }

    if(J7VCL_LEO_PMICA_DEVICE == pmic_device_info)
    {
        pmic_testResultUpdate_ignore(6246,
                                         pmic_gpio_tests,
                                         PMIC_GPIO_NUM_OF_TESTCASES);
    }

    if(J7VCL_HERA_PMICB_DEVICE == pmic_device_info)
    {
        gpioCfg.validParams = PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT |
                              PMIC_GPIO_CFG_DIR_VALID_SHIFT |
                              PMIC_GPIO_CFG_OD_VALID_SHIFT |
                              PMIC_GPIO_CFG_PULL_VALID_SHIFT;
        gpioCfg.outputSignalType = PMIC_GPIO_PUSH_PULL_OUTPUT;
        gpioCfg.pullCtrl = PMIC_GPIO_PULL_UP;
        gpioCfg.pinFunc = PMIC_LP8764X_GPIO_PINFUNC_GPIO;
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        intMask = PMIC_TPS6594X_IRQ_GPIO_7_INT_MASK_NUM;
        irqNum = PMIC_TPS6594X_GPIO7_INT;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        intMask = PMIC_LP8764X_IRQ_GPIO_7_INT_MASK_NUM;
        irqNum = PMIC_LP8764X_GPIO7_INT;
    }

    /* Un Masking GPIO 7 FALL Interrupt */
    Pmic_irqGpioMaskIntr(pPmicCoreHandle,
                         intMask,
                         PMIC_IRQ_UNMASK,
                         PMIC_IRQ_GPIO_FALL_INT_TYPE);

    /* To clear the interrupts*/
    pmic_gpioTest_intClr();

    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioSetValue(pPmicCoreHandle, pin, pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioSetIntr(pPmicCoreHandle, pin , intrType, maskPol);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pinValue = PMIC_GPIO_LOW;
    pmicStatus = Pmic_gpioSetValue(pPmicCoreHandle, pin, pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    while(timeout--)
    {
        /* Delay added to avoid timeout */
        Osal_delay(1000);
        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        if((PMIC_ST_SUCCESS == pmicStatus) &&
           ((errStat.intStatus[irqNum/32U] & (1U << (irqNum % 32U))) != 0U))
        {
            while(irqNum != nextedIrq)
            {
                pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                                     &errStat,
                                                     &nextedIrq);
            }

            if(PMIC_ST_SUCCESS == pmicStatus)
            {
                /* clear the interrupt */
                pmicStatus = Pmic_irqClrErrStatus(pPmicCoreHandle,
                                                  irqNum);
                break;
            }
        }
    }

    if(0 > timeout)
    {
        pmicStatus = PMIC_ST_ERR_FAIL;
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmic_testResultUpdate_pass(6246,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
}

/*!
 * \brief   Test to verify GPIO7 rise interrupt
 */
static void test_pmic_gpio7_testRise_interrupt(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    int8_t timeout            = 10U;
    uint8_t pin               = 7U;
    uint8_t pinValue          = PMIC_GPIO_LOW;
    uint8_t intrType          = PMIC_GPIO_RISE_INTERRUPT;
    uint8_t maskPol           = PMIC_GPIO_POL_HIGH;
    Pmic_IrqStatus_t errStat  = {0U};
    bool clearIRQ             = false;
    uint8_t  irqNum           = 0U;
    uint8_t  nextedIrq        = 0U;
    uint8_t  intMask          = 0U;
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

    if(J721E_LEO_PMICA_DEVICE == pmic_device_info)
    {
        /*
         * GPIO7 pin of PMIC-A is connected to 'SOC_SAFETY_ERRZ'
         * on J721EVM board causing hang, when programming rise interrupt.
         */
        pmic_testResultUpdate_ignore(6247,
                                         pmic_gpio_tests,
                                         PMIC_GPIO_NUM_OF_TESTCASES);
    }
    if(J721E_LEO_PMICB_DEVICE == pmic_device_info)
    {
        /*
         * GPIO7 pin of PMIC-B is connected to 'EN_RAM0V85_LDO'
         * on J721EVM board causing hang, when programming rise interrupt.
         */
        pmic_testResultUpdate_ignore(6247,
                                         pmic_gpio_tests,
                                         PMIC_GPIO_NUM_OF_TESTCASES);
    }

    if(J7VCL_LEO_PMICA_DEVICE == pmic_device_info)
    {
         pmic_testResultUpdate_ignore(6247,
                                         pmic_gpio_tests,
                                         PMIC_GPIO_NUM_OF_TESTCASES);
    }

    if(J7VCL_HERA_PMICB_DEVICE == pmic_device_info)
    {
        gpioCfg.validParams = PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT |
                              PMIC_GPIO_CFG_DIR_VALID_SHIFT |
                              PMIC_GPIO_CFG_OD_VALID_SHIFT |
                              PMIC_GPIO_CFG_PULL_VALID_SHIFT;
        gpioCfg.outputSignalType = PMIC_GPIO_PUSH_PULL_OUTPUT;
        gpioCfg.pullCtrl = PMIC_GPIO_PULL_UP;
        gpioCfg.pinFunc = PMIC_LP8764X_GPIO_PINFUNC_GPIO;
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        intMask = PMIC_TPS6594X_IRQ_GPIO_7_INT_MASK_NUM;
        irqNum = PMIC_TPS6594X_GPIO7_INT;
    }
    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        intMask = PMIC_LP8764X_IRQ_GPIO_7_INT_MASK_NUM;
        irqNum = PMIC_LP8764X_GPIO7_INT;
    }

    /* Un Masking GPIO 7 RISE Interrupt */
    Pmic_irqGpioMaskIntr(pPmicCoreHandle,
                         intMask,
                         PMIC_IRQ_UNMASK,
                         PMIC_IRQ_GPIO_RISE_INT_TYPE);

    /* To clear the interrupts*/
    pmic_gpioTest_intClr();

    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioSetValue(pPmicCoreHandle, pin, pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioSetIntr(pPmicCoreHandle, pin , intrType, maskPol);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pinValue = PMIC_GPIO_HIGH;
    pmicStatus = Pmic_gpioSetValue(pPmicCoreHandle, pin, pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    while(timeout--)
    {
        /* Delay added to avoid timeout */
        Osal_delay(1000);
        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        if((PMIC_ST_SUCCESS == pmicStatus) &&
           ((errStat.intStatus[irqNum/32U] & (1U << (irqNum % 32U))) != 0U))
        {
            while(irqNum != nextedIrq)
            {
                pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                                     &errStat,
                                                     &nextedIrq);
            }

            if(PMIC_ST_SUCCESS == pmicStatus)
            {
                /* clear the interrupt */
                pmicStatus = Pmic_irqClrErrStatus(pPmicCoreHandle,
                                                  irqNum);
                break;
            }
        }
    }

    if(0 > timeout)
    {
        pmicStatus = PMIC_ST_ERR_FAIL;
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmic_testResultUpdate_pass(6247,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
}

/*!
 * \brief   Test to verify GPIO8 fall interrupt
 */
static void test_pmic_gpio8_testFall_interrupt(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    int8_t timeout            = 10U;
    uint8_t pin               = 8U;
    uint8_t pinValue          = PMIC_GPIO_HIGH;
    uint8_t intrType          = PMIC_GPIO_FALL_INTERRUPT;
    uint8_t maskPol           = PMIC_GPIO_POL_LOW;
    Pmic_IrqStatus_t errStat  = {0U};
    bool clearIRQ             = false;
    uint8_t  irqNum           = 0U;
    uint8_t  nextedIrq        = 0U;
    uint8_t  intMask          = 0U;
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

    if(J721E_LEO_PMICB_DEVICE == pmic_device_info)
    {
        /*
         * GPIO8 pin of PMIC-B is connected to 'EN_PHYCORE_LDO'
         * on J721EVM causing hang, when programming fall interrupt signal.
         */
        pmic_testResultUpdate_ignore(6248,
                                         pmic_gpio_tests,
                                         PMIC_GPIO_NUM_OF_TESTCASES);
    }

    if(J7VCL_HERA_PMICB_DEVICE == pmic_device_info)
    {
        gpioCfg.validParams = PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT |
                              PMIC_GPIO_CFG_DIR_VALID_SHIFT |
                              PMIC_GPIO_CFG_OD_VALID_SHIFT |
                              PMIC_GPIO_CFG_PULL_VALID_SHIFT;
        gpioCfg.outputSignalType = PMIC_GPIO_PUSH_PULL_OUTPUT;
        gpioCfg.pullCtrl = PMIC_GPIO_PULL_UP;
        gpioCfg.pinFunc = PMIC_LP8764X_GPIO_PINFUNC_GPIO;
    }

    if(J7VCL_HERA_PMICB_DEVICE == pmic_device_info)
    {
        pmic_testResultUpdate_ignore(6248,
                                         pmic_gpio_tests,
                                         PMIC_GPIO_NUM_OF_TESTCASES);
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        intMask = PMIC_TPS6594X_IRQ_GPIO_8_INT_MASK_NUM;
        irqNum = PMIC_TPS6594X_GPIO8_INT;
    }
    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        intMask = PMIC_LP8764X_IRQ_GPIO_8_INT_MASK_NUM;
        irqNum = PMIC_LP8764X_GPIO8_INT;
    }
    /* Un Masking GPIO 8 FALL Interrupt */
    Pmic_irqGpioMaskIntr(pPmicCoreHandle,
                         intMask,
                         PMIC_IRQ_UNMASK,
                         PMIC_IRQ_GPIO_FALL_INT_TYPE);

    /* To clear the interrupts*/
    pmic_gpioTest_intClr();

    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioSetValue(pPmicCoreHandle, pin, pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioSetIntr(pPmicCoreHandle, pin , intrType, maskPol);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pinValue = PMIC_GPIO_LOW;
    pmicStatus = Pmic_gpioSetValue(pPmicCoreHandle, pin, pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    while(timeout--)
    {
        /* Delay added to avoid timeout */
        Osal_delay(1000);
        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        if((PMIC_ST_SUCCESS == pmicStatus) &&
           ((errStat.intStatus[irqNum/32U] & (1U << (irqNum % 32U))) != 0U))
        {
            while(irqNum != nextedIrq)
            {
                pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                                     &errStat,
                                                     &nextedIrq);
            }

            if(PMIC_ST_SUCCESS == pmicStatus)
            {
                /* clear the interrupt */
                pmicStatus = Pmic_irqClrErrStatus(pPmicCoreHandle,
                                                  irqNum);
                break;
            }
        }
    }

    if(0 > timeout)
    {
        pmicStatus = PMIC_ST_ERR_FAIL;
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmic_testResultUpdate_pass(6248,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
}

/*!
 * \brief   Test to verify GPIO8 rise interrupt
 */
static void test_pmic_gpio8_testRise_interrupt(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    int8_t timeout            = 10U;
    uint8_t pin               = 8U;
    uint8_t pinValue          = PMIC_GPIO_LOW;
    uint8_t intrType          = PMIC_GPIO_RISE_INTERRUPT;
    uint8_t maskPol           = PMIC_GPIO_POL_HIGH;
    Pmic_IrqStatus_t errStat  = {0U};
    bool clearIRQ             = false;
    uint8_t  irqNum           = 0U;
    uint8_t  nextedIrq        = 0U;
    uint8_t  intMask          = 0U;
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

    if(J721E_LEO_PMICB_DEVICE == pmic_device_info)
    {
        /*
         * GPIO8 pin of PMIC-B is connected to 'EN_PHYCORE_LDO'
         * on J721EVM causing hang, when programming fall interrupt signal.
         */
        pmic_testResultUpdate_ignore(6249,
                                         pmic_gpio_tests,
                                         PMIC_GPIO_NUM_OF_TESTCASES);
    }

    if(J7VCL_HERA_PMICB_DEVICE == pmic_device_info)
    {
        gpioCfg.validParams = PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT |
                              PMIC_GPIO_CFG_DIR_VALID_SHIFT |
                              PMIC_GPIO_CFG_OD_VALID_SHIFT |
                              PMIC_GPIO_CFG_PULL_VALID_SHIFT;
        gpioCfg.outputSignalType = PMIC_GPIO_PUSH_PULL_OUTPUT;
        gpioCfg.pullCtrl = PMIC_GPIO_PULL_UP;
        gpioCfg.pinFunc = PMIC_LP8764X_GPIO_PINFUNC_GPIO;
    }

    if(J7VCL_HERA_PMICB_DEVICE == pmic_device_info)
    {
        pmic_testResultUpdate_ignore(6249,
                                         pmic_gpio_tests,
                                         PMIC_GPIO_NUM_OF_TESTCASES);
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        intMask = PMIC_TPS6594X_IRQ_GPIO_8_INT_MASK_NUM;
        irqNum = PMIC_TPS6594X_GPIO8_INT;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        intMask = PMIC_LP8764X_IRQ_GPIO_8_INT_MASK_NUM;
        irqNum = PMIC_LP8764X_GPIO8_INT;
    }
    /* Un Masking GPIO 8 RISE Interrupt */
    Pmic_irqGpioMaskIntr(pPmicCoreHandle,
                         intMask,
                         PMIC_IRQ_UNMASK,
                         PMIC_IRQ_GPIO_RISE_INT_TYPE);

    /* To clear the interrupts*/
    pmic_gpioTest_intClr();

    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioSetValue(pPmicCoreHandle, pin, pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioSetIntr(pPmicCoreHandle, pin , intrType, maskPol);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pinValue = PMIC_GPIO_HIGH;
    pmicStatus = Pmic_gpioSetValue(pPmicCoreHandle, pin, pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    while(timeout--)
    {
        /* Delay added to avoid timeout */
        Osal_delay(1000);
        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        if((PMIC_ST_SUCCESS == pmicStatus) &&
           ((errStat.intStatus[irqNum/32U] & (1U << (irqNum % 32U))) != 0U))
        {
            while(irqNum != nextedIrq)
            {
                pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                                     &errStat,
                                                     &nextedIrq);
            }

            if(PMIC_ST_SUCCESS == pmicStatus)
            {
                /* clear the interrupt */
                pmicStatus = Pmic_irqClrErrStatus(pPmicCoreHandle,
                                                  irqNum);
                break;
            }
        }
    }

    if(0 > timeout)
    {
        pmicStatus = PMIC_ST_ERR_FAIL;
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmic_testResultUpdate_pass(6249,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
}

/*!
 * \brief   Test to verify GPIO10 fall interrupt
 */
static void test_pmic_gpio10_testFall_interrupt(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    int8_t timeout            = 10U;
    uint8_t pin               = 10U;
    uint8_t pinValue          = PMIC_GPIO_HIGH;
    uint8_t intrType          = PMIC_GPIO_FALL_INTERRUPT;
    uint8_t maskPol           = PMIC_GPIO_POL_LOW;
    Pmic_IrqStatus_t errStat  = {0U};
    bool clearIRQ             = false;
    uint8_t  irqNum           = 0U;
    uint8_t  nextedIrq        = 0U;
    uint8_t  intMask          = 0U;
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

    if(J721E_LEO_PMICA_DEVICE == pmic_device_info)
    {
        /*
         * GPIO10 pin of PMIC-A is connected to 'PMIC_POWER_EN1'
         * on J721EVM board causing hang, when programming fall interrupt.
         */
        pmic_testResultUpdate_ignore(6252,
                                         pmic_gpio_tests,
                                         PMIC_GPIO_NUM_OF_TESTCASES);
    }

    if(J7VCL_HERA_PMICB_DEVICE == pmic_device_info)
    {
        gpioCfg.validParams = PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT |
                              PMIC_GPIO_CFG_DIR_VALID_SHIFT |
                              PMIC_GPIO_CFG_OD_VALID_SHIFT |
                              PMIC_GPIO_CFG_PULL_VALID_SHIFT;
        gpioCfg.outputSignalType = PMIC_GPIO_PUSH_PULL_OUTPUT;
        gpioCfg.pullCtrl = PMIC_GPIO_PULL_UP;
        gpioCfg.pinFunc = PMIC_LP8764X_GPIO_PINFUNC_GPIO;
    }

    if(J7VCL_HERA_PMICB_DEVICE == pmic_device_info)
    {
        pmic_testResultUpdate_ignore(6252,
                                         pmic_gpio_tests,
                                         PMIC_GPIO_NUM_OF_TESTCASES);
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        intMask = PMIC_TPS6594X_IRQ_GPIO_10_INT_MASK_NUM;
        irqNum = PMIC_TPS6594X_GPIO10_INT;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        intMask = PMIC_LP8764X_IRQ_GPIO_10_INT_MASK_NUM;
        irqNum = PMIC_LP8764X_GPIO10_INT;
    }

    /* Un Masking GPIO 10 FALL Interrupt */
    Pmic_irqGpioMaskIntr(pPmicCoreHandle,
                         intMask,
                         PMIC_IRQ_UNMASK,
                         PMIC_IRQ_GPIO_FALL_INT_TYPE);

    /* To clear the interrupts*/
    pmic_gpioTest_intClr();

    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioSetValue(pPmicCoreHandle, pin, pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioSetIntr(pPmicCoreHandle, pin , intrType, maskPol);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pinValue = PMIC_GPIO_LOW;
    pmicStatus = Pmic_gpioSetValue(pPmicCoreHandle, pin, pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    while(timeout--)
    {
        /* Delay added to avoid timeout */
        Osal_delay(1000);
        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        if((PMIC_ST_SUCCESS == pmicStatus) &&
           ((errStat.intStatus[irqNum/32U] & (1U << (irqNum % 32U))) != 0U))
        {
            while(irqNum != nextedIrq)
            {
                pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                                     &errStat,
                                                     &nextedIrq);
            }

            if(PMIC_ST_SUCCESS == pmicStatus)
            {
                /* clear the interrupt */
                pmicStatus = Pmic_irqClrErrStatus(pPmicCoreHandle,
                                                  irqNum);
                break;
            }
        }
    }

    if(0 > timeout)
    {
        pmicStatus = PMIC_ST_ERR_FAIL;
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmic_testResultUpdate_pass(6252,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
}

/*!
 * \brief   Test to verify GPIO10 rise interrupt
 */
static void test_pmic_gpio10_testRise_interrupt(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    int8_t timeout            = 10U;
    uint8_t pin               = 10U;
    uint8_t pinValue          = PMIC_GPIO_LOW;
    uint8_t intrType          = PMIC_GPIO_RISE_INTERRUPT;
    uint8_t maskPol           = PMIC_GPIO_POL_HIGH;
    Pmic_IrqStatus_t errStat  = {0U};
    bool clearIRQ             = false;
    uint8_t  irqNum           = 0U;
    uint8_t  nextedIrq        = 0U;
    uint8_t  intMask          = 0U;
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

    if(J721E_LEO_PMICA_DEVICE == pmic_device_info)
    {
        /*
         * GPIO10 pin of PMIC-A is connected to 'PMIC_POWER_EN1'
         * on J721EVM board causing hang, when programming rise interrupt.
         */
        pmic_testResultUpdate_ignore(6253,
                                         pmic_gpio_tests,
                                         PMIC_GPIO_NUM_OF_TESTCASES);
    }

    if(J7VCL_HERA_PMICB_DEVICE == pmic_device_info)
    {
        gpioCfg.validParams = PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT |
                              PMIC_GPIO_CFG_DIR_VALID_SHIFT |
                              PMIC_GPIO_CFG_OD_VALID_SHIFT |
                              PMIC_GPIO_CFG_PULL_VALID_SHIFT;
        gpioCfg.outputSignalType = PMIC_GPIO_PUSH_PULL_OUTPUT;
        gpioCfg.pullCtrl = PMIC_GPIO_PULL_UP;
        gpioCfg.pinFunc = PMIC_LP8764X_GPIO_PINFUNC_GPIO;
    }

    if(J7VCL_HERA_PMICB_DEVICE == pmic_device_info)
    {
        pmic_testResultUpdate_ignore(6253,
                                         pmic_gpio_tests,
                                         PMIC_GPIO_NUM_OF_TESTCASES);
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        intMask = PMIC_TPS6594X_IRQ_GPIO_10_INT_MASK_NUM;
        irqNum = PMIC_TPS6594X_GPIO10_INT;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        intMask = PMIC_LP8764X_IRQ_GPIO_10_INT_MASK_NUM;
        irqNum = PMIC_LP8764X_GPIO10_INT;
    }

    /* Un Masking GPIO 10 RISE Interrupt */
    Pmic_irqGpioMaskIntr(pPmicCoreHandle,
                         intMask,
                         PMIC_IRQ_UNMASK,
                         PMIC_IRQ_GPIO_RISE_INT_TYPE);

    /* To clear the interrupts*/
    pmic_gpioTest_intClr();

    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioSetValue(pPmicCoreHandle, pin, pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioSetIntr(pPmicCoreHandle, pin , intrType, maskPol);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pinValue = PMIC_GPIO_HIGH;
    pmicStatus = Pmic_gpioSetValue(pPmicCoreHandle, pin, pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    while(timeout--)
    {
        /* Delay added to avoid timeout */
        Osal_delay(1000);
        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        if((PMIC_ST_SUCCESS == pmicStatus) &&
           ((errStat.intStatus[irqNum/32U] & (1U << (irqNum % 32U))) != 0U))
        {
            while(irqNum != nextedIrq)
            {
                pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                                     &errStat,
                                                     &nextedIrq);
            }

            if(PMIC_ST_SUCCESS == pmicStatus)
            {
                /* clear the interrupt */
                pmicStatus = Pmic_irqClrErrStatus(pPmicCoreHandle,
                                                  irqNum);
                break;
            }
        }
    }

    if(0 > timeout)
    {
        pmicStatus = PMIC_ST_ERR_FAIL;
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmic_testResultUpdate_pass(6253,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
}

/*!
 * \brief   Parameter validation for handle
 */
static void test_pmic_gpio_intr_prmValTest_handle(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    uint8_t pin               = 3U;
    uint8_t intrType          = PMIC_GPIO_FALL_INTERRUPT;
    uint8_t maskPol           = PMIC_GPIO_POL_LOW;

    test_pmic_print_unity_testcase_info(6256,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    pmicStatus = Pmic_gpioSetIntr(NULL, pin , intrType, maskPol);
    TEST_ASSERT_EQUAL(pmicStatus, PMIC_ST_ERR_INV_HANDLE);

    pmic_testResultUpdate_pass(6256,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
}

/*!
 * \brief   Parameter validation for pin
 */
static void test_pmic_gpio_intr_prmValTest_pin(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    uint8_t pin               = 12U;
    uint8_t intrType          = PMIC_GPIO_FALL_INTERRUPT;
    uint8_t maskPol           = PMIC_GPIO_POL_LOW;

    test_pmic_print_unity_testcase_info(6257,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    pmicStatus = Pmic_gpioSetIntr(pPmicCoreHandle, pin, intrType, maskPol);
    TEST_ASSERT_EQUAL(pmicStatus, PMIC_ST_ERR_INV_PARAM);

    pmic_testResultUpdate_pass(6257,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
}

/*!
 * \brief   Parameter validation for intrType
 */
static void test_pmic_gpio_intr_prmValTest_intrType(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    uint8_t pin               = 1U;
    uint8_t intrType          = 0x4U;
    uint8_t maskPol           = PMIC_GPIO_POL_LOW;

    test_pmic_print_unity_testcase_info(6258,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    pmicStatus = Pmic_gpioSetIntr(pPmicCoreHandle, pin, intrType, maskPol);
    TEST_ASSERT_EQUAL(pmicStatus, PMIC_ST_ERR_INV_PARAM);

    pmic_testResultUpdate_pass(6258,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
}

/*!
 * \brief   Parameter validation for maskPol
 */
static void test_pmic_gpio_intr_prmValTest_maskPol(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    uint8_t pin               = 2U;
    uint8_t intrType          = PMIC_GPIO_RISE_INTERRUPT;
    uint8_t maskPol           = 0x2U;

    test_pmic_print_unity_testcase_info(6259,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    pmicStatus = Pmic_gpioSetIntr(pPmicCoreHandle, pin , intrType, maskPol);
    TEST_ASSERT_EQUAL(pmicStatus, PMIC_ST_ERR_INV_PARAM);

    pmic_testResultUpdate_pass(6259,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
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
        gpioCfg.pinFunc = PMIC_LP8764X_GPIO_PINFUNC_GPIO;
    }

    if(J7VCL_HERA_PMICB_DEVICE == pmic_device_info)
    {
        pin = 2U;
        gpioCfg.validParams = PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT |
                              PMIC_GPIO_CFG_DIR_VALID_SHIFT |
                              PMIC_GPIO_CFG_OD_VALID_SHIFT |
                              PMIC_GPIO_CFG_PULL_VALID_SHIFT;
        gpioCfg.outputSignalType = PMIC_GPIO_PUSH_PULL_OUTPUT;
        gpioCfg.pullCtrl = PMIC_GPIO_PULL_UP;
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

    pmic_testResultUpdate_pass(7374,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
}

/*!
 * \brief   UnMasking all interrupts test
 */
static void test_pmic_gpio_intr_irqUnMaskAll_interrupt(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    int8_t timeout            = 10U;
    uint8_t pin               = 1U;
    uint8_t pinValue          = PMIC_GPIO_HIGH;
    uint8_t intrType          = PMIC_GPIO_FALL_INTERRUPT;
    uint8_t maskPol           = PMIC_GPIO_POL_LOW;
    Pmic_IrqStatus_t errStat  = {0U};
    bool clearIRQ             = false;
    uint8_t  irqNum           = 0U;
    uint8_t  nextedIrq        = 0U;
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
        pmic_testResultUpdate_ignore(7375,
                                         pmic_gpio_tests,
                                         PMIC_GPIO_NUM_OF_TESTCASES);
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        gpioCfg.pinFunc = PMIC_LP8764X_GPIO_PINFUNC_GPIO;
    }

    if(J7VCL_HERA_PMICB_DEVICE == pmic_device_info)
    {
        pin = 2U;
        gpioCfg.validParams = PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT |
                              PMIC_GPIO_CFG_DIR_VALID_SHIFT |
                              PMIC_GPIO_CFG_OD_VALID_SHIFT |
                              PMIC_GPIO_CFG_PULL_VALID_SHIFT;
        gpioCfg.outputSignalType = PMIC_GPIO_PUSH_PULL_OUTPUT;
        gpioCfg.pullCtrl = PMIC_GPIO_PULL_UP;
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        irqNum = PMIC_TPS6594X_GPIO1_INT;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        if(J7VCL_HERA_PMICB_DEVICE == pmic_device_info)
        {
            irqNum = PMIC_LP8764X_GPIO2_INT;
        }
        else
        {
            irqNum = PMIC_LP8764X_GPIO1_INT;
        }
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

    /* To clear the interrupts*/
    pmic_gpioTest_intClr();

    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioSetValue(pPmicCoreHandle, pin, pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioSetIntr(pPmicCoreHandle, pin , intrType, maskPol);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pinValue = PMIC_GPIO_LOW;
    pmicStatus = Pmic_gpioSetValue(pPmicCoreHandle, pin, pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    while(timeout--)
    {
        /* Delay added to avoid timeout */
        Osal_delay(1000);
        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        if((PMIC_ST_SUCCESS == pmicStatus) &&
           ((errStat.intStatus[irqNum/32U] & (1U << (irqNum % 32U))) != 0U))
        {
            while(irqNum != nextedIrq)
            {
                pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                                     &errStat,
                                                     &nextedIrq);
            }

            if(PMIC_ST_SUCCESS == pmicStatus)
            {
                /* clear the interrupt */
                pmicStatus = Pmic_irqClrErrStatus(pPmicCoreHandle,
                                                  irqNum);
                break;
            }
        }
    }

    if(0 > timeout)
    {
        pmicStatus = PMIC_ST_ERR_FAIL;
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

    pmic_testResultUpdate_pass(7375,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
}

/*!
 * \brief   Gpio Tps6594x NPwron Get PinValue Test
 */
static void test_pmic_gpio_testTps6594xNPwronPinGetValue(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    uint8_t pinValue          = PMIC_GPIO_HIGH;

    test_pmic_print_unity_testcase_info(7904,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);
    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(7904,
                                         pmic_gpio_tests,
                                         PMIC_GPIO_NUM_OF_TESTCASES);
    }

    pmicStatus = Pmic_gpioTps6594xNPwronPinGetValue(pPmicCoreHandle, &pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmic_testResultUpdate_pass(7904,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
}

/*!
 * \brief   Parameter validation for handle
 */
static void test_pmic_gpio_tps6594xNPwronPinGetValuePrmValTest_handle(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    uint8_t pinValue          = PMIC_GPIO_HIGH;

    test_pmic_print_unity_testcase_info(7905,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(7905,
                                         pmic_gpio_tests,
                                         PMIC_GPIO_NUM_OF_TESTCASES);
    }

    pmicStatus = Pmic_gpioTps6594xNPwronPinGetValue(NULL, &pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, pmicStatus);

    pmic_testResultUpdate_pass(7905,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
}

/*!
 * \brief   Parameter validation for Pin Value
 */
static void test_pmic_gpio_tps6594xNPwronPinGetValuePrmValTest_pinValue(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;

    test_pmic_print_unity_testcase_info(7906,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(7906,
                                         pmic_gpio_tests,
                                         PMIC_GPIO_NUM_OF_TESTCASES);
    }

    pmicStatus = Pmic_gpioTps6594xNPwronPinGetValue(pPmicCoreHandle, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(7906,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
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
        pmic_testResultUpdate_ignore(7879,
                                         pmic_gpio_tests,
                                         PMIC_GPIO_NUM_OF_TESTCASES);
    }

    pmicStatus = Pmic_gpioGetConfiguration(pPmicCoreHandle, pin, &gpioCfg);
    TEST_ASSERT_EQUAL(pmicStatus, PMIC_ST_ERR_INV_PARAM);

    pmic_testResultUpdate_pass(7879,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_gpioTps6594xNPwronPinGetValue : Negative test for nPWRON Pin for hera
 */
static void test_pmic_gpio_testTps6594xNPwronPinGetValue_hera(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    uint8_t pinValue          = PMIC_GPIO_HIGH;

    test_pmic_print_unity_testcase_info(7880,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(7880,
                                         pmic_gpio_tests,
                                         PMIC_GPIO_NUM_OF_TESTCASES);
    }

    pmicStatus = Pmic_gpioTps6594xNPwronPinGetValue(pPmicCoreHandle, &pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_DEVICE, pmicStatus);

    pmic_testResultUpdate_pass(7880,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
}

/*!
 * \brief   Test to verify GPIO7 fall Asynchronous interrupt
 */
void test_pmic_gpio7_testFallAsynchronous_interrupt(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    uint8_t pin               = 7U;
    uint8_t intrType          = PMIC_GPIO_FALL_INTERRUPT;
    uint8_t maskPol           = PMIC_GPIO_POL_HIGH;
    int8_t  timeout           = 0xFFU;
    Pmic_GpioCfg_t gpioCfg    =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT | PMIC_GPIO_CFG_DIR_VALID_SHIFT,
        PMIC_GPIO_INPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_UP,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_GPIO_PINFUNC_GPIO,
        PMIC_GPIO_HIGH
    };

    test_pmic_print_unity_testcase_info(7950,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

#if ((defined(SOC_J7200) && (defined(BUILD_MCU2_0) || defined(BUILD_MCU2_1))) || \
     (defined(SOC_J721E) && (defined(BUILD_MCU3_0) || defined(BUILD_MCU3_1))))
    /*Refer the Bug-PDK-10394 for more details*/
    pmic_testResultUpdate_ignore(7950,
                                 pmic_gpio_tests,
                                 PMIC_GPIO_NUM_OF_TESTCASES);
#endif

#if defined(SOC_J721E)
    pmic_log("\r\n To check Fall interrupt applying SOM board Ground signal over TP24");
#endif
#if defined(SOC_J7200)
    pmic_log("\r\n To check Fall interrupt applying SOM board Ground signal over TP56");
#endif

    pmic_intr_triggered = 0U;

    /* Un Masking GPIO_7 RISE and FALL Interrupts */
    Pmic_irqGpioMaskIntr(pPmicCoreHandle,
                         PMIC_TPS6594X_IRQ_GPIO_7_INT_MASK_NUM,
                         PMIC_IRQ_UNMASK,
                         PMIC_IRQ_GPIO_FALL_INT_TYPE);

    /* To clear the interrupts*/
    pmic_gpioTest_intClr();

    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioSetIntr(pPmicCoreHandle, pin, intrType, maskPol);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    GPIO_enableInt(0);

    while((pmic_intr_triggered == 0) && (timeout--))
    {
        /* Delay added to avoid timeout */
        Osal_delay(1000);
    }

    GPIO_disableInt(0);

    /* Check for interrupt */
    if(pmic_intr_triggered > 0)
    {
        pmicStatus = PMIC_ST_SUCCESS;
    }
    else
    {
        pmicStatus = PMIC_ST_ERR_FAIL;
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
}

/*!
 * \brief   Test to verify GPIO7 rise Asynchronous interrupt
 */
void test_pmic_gpio7_testRiseAsynchronous_interrupt(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    uint8_t pin               = 7U;
    uint8_t intrType          = PMIC_GPIO_RISE_INTERRUPT;
    uint8_t maskPol           = PMIC_GPIO_POL_HIGH;
    int8_t  timeout           = 0xFFU;
    Pmic_GpioCfg_t gpioCfg    =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT | PMIC_GPIO_CFG_DIR_VALID_SHIFT,
        PMIC_GPIO_INPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_UP,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_GPIO_PINFUNC_GPIO,
        PMIC_GPIO_HIGH
    };

    test_pmic_print_unity_testcase_info(7951,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

#if ((defined(SOC_J7200) && (defined(BUILD_MCU2_0) || defined(BUILD_MCU2_1))) || \
     (defined(SOC_J721E) && (defined(BUILD_MCU3_0) || defined(BUILD_MCU3_1))))
    /*Refer the Bug-PDK-10394 for more details*/
    pmic_testResultUpdate_ignore(7951,
                                 pmic_gpio_tests,
                                 PMIC_GPIO_NUM_OF_TESTCASES);
#endif

#if defined(SOC_J721E)
    pmic_log("\r\n To check Fall interrupt applying SOM board Ground signal over TP24");
#endif
#if defined(SOC_J7200)
    pmic_log("\r\n To check Fall interrupt applying SOM board Ground signal over TP56");
#endif

    pmic_intr_triggered = 0U;

    /* Un Masking GPIO_7 RISE and FALL Interrupts */
    Pmic_irqGpioMaskIntr(pPmicCoreHandle,
                         PMIC_TPS6594X_IRQ_GPIO_7_INT_MASK_NUM,
                         PMIC_IRQ_UNMASK,
                         PMIC_IRQ_GPIO_RISE_INT_TYPE);

    /* To clear the interrupts*/
    pmic_gpioTest_intClr();

    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioSetIntr(pPmicCoreHandle, pin, intrType, maskPol);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    GPIO_enableInt(0);

    while((pmic_intr_triggered == 0) && (timeout--))
    {
        /* Delay added to avoid timeout */
        Osal_delay(1000);
    }

    GPIO_disableInt(0);

    /* Check for interrupt */
    if(pmic_intr_triggered > 0)
    {
        pmicStatus = PMIC_ST_SUCCESS;
    }
    else
    {
        pmicStatus = PMIC_ST_ERR_FAIL;
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
}

/*!
 * \brief   Test to verify CAN WKUP
 */
void test_pmic_canWkup_test(void)
{
    int32_t status         = PMIC_ST_SUCCESS;
    uint8_t  pmicState     = 0U;
    Pmic_GpioCfg_t gpioCfg = {0U};

    gpioCfg.validParams      = PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT;
    gpioCfg.pinFunc          = PMIC_TPS6594X_GPIO_PINFUNC_GPIO3_GPIO4_LP_WKUP1;

    status = Pmic_gpioSetConfiguration(pPmicCoreHandle,
                                       PMIC_TPS6594X_GPIO4_PIN,
                                       gpioCfg);
    pmicState = PMIC_FSM_LP_STANBY_STATE;

    test_pmic_print_unity_testcase_info(8012,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

#if defined(SOC_J721E)
    pmic_testResultUpdate_ignore(8012,
                                 pmic_gpio_tests,
                                 PMIC_GPIO_NUM_OF_TESTCASES);
#endif

    if(J721E_LEO_PMICA_DEVICE == pmic_device_info)
    {
         TEST_IGNORE();
    }

    if(J721E_LEO_PMICB_DEVICE == pmic_device_info)
    {
         TEST_IGNORE();
    }

    if(J7VCL_HERA_PMICB_DEVICE == pmic_device_info)
    {
        TEST_IGNORE();
    }

    Pmic_irqGpioMaskIntr(pPmicCoreHandle,
                         PMIC_TPS6594X_IRQ_GPIO_4_INT_MASK_NUM,
                         PMIC_IRQ_UNMASK,
                         PMIC_IRQ_GPIO_RISE_INT_TYPE);
    Pmic_irqGpioMaskIntr(pPmicCoreHandle,
                         PMIC_TPS6594X_IRQ_GPIO_4_INT_MASK_NUM,
                         PMIC_IRQ_MASK,
                         PMIC_IRQ_GPIO_FALL_INT_TYPE);

    /*Mask Nsleep1B and Nsleep2B signals for FSM state transitions*/
    status = Pmic_fsmSetNsleepSignalMask(pPmicCoreHandle,
                                         PMIC_NSLEEP1_SIGNAL,
                                         PMIC_NSLEEPX_MASK);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_fsmSetNsleepSignalMask(pPmicCoreHandle,
                                         PMIC_NSLEEP2_SIGNAL,
                                         PMIC_NSLEEPX_MASK);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    Osal_delay(10U);

    pmic_log("\r\n Input LOW to HIGH signal to GPIO4 to wakeup from low power mode\n");

    status = Pmic_fsmSetMissionState(pPmicCoreHandle, pmicState);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
}

/*!
 * \brief   Test to verify clearing all Interrupts using IRQ API
 */
static void test_gpio4_fallInterrupt_clrAllIrqTest(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    int8_t timeout            = 10U;
    uint8_t pin               = 4U;
    uint8_t pinValue          = PMIC_GPIO_HIGH;
    uint8_t intrType          = PMIC_GPIO_FALL_INTERRUPT;
    uint8_t maskPol           = PMIC_GPIO_POL_LOW;
    Pmic_IrqStatus_t errStat  = {0U};
    bool clearIRQ             = false;
    uint8_t  irqNum           = 0U;
    uint8_t  nextedIrq        = 0U;
    uint8_t  intMask          = 0U;
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

    test_pmic_print_unity_testcase_info(8041,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    if(J721E_LEO_PMICB_DEVICE == pmic_device_info)
    {
        /*
         * GPIO4 pin of PMIC-B is connected to 'H_DDR_RET_1V1'
         * on J721EVM board causing hang, when programming fall interrupt.
         */
        TEST_IGNORE();
    }

    if(J7VCL_HERA_PMICB_DEVICE == pmic_device_info)
    {
        gpioCfg.validParams = PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT |
                              PMIC_GPIO_CFG_DIR_VALID_SHIFT |
                              PMIC_GPIO_CFG_OD_VALID_SHIFT |
                              PMIC_GPIO_CFG_PULL_VALID_SHIFT;
        gpioCfg.outputSignalType = PMIC_GPIO_PUSH_PULL_OUTPUT;
        gpioCfg.pullCtrl = PMIC_GPIO_PULL_UP;
        gpioCfg.pinFunc = PMIC_LP8764X_GPIO_PINFUNC_GPIO;
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        intMask = PMIC_TPS6594X_IRQ_GPIO_4_INT_MASK_NUM;
        irqNum = PMIC_TPS6594X_GPIO4_INT;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        intMask = PMIC_LP8764X_IRQ_GPIO_4_INT_MASK_NUM;
        irqNum = PMIC_LP8764X_GPIO4_INT;
    }

    /* Un Masking GPIO 4 FALL Interrupt */
    Pmic_irqGpioMaskIntr(pPmicCoreHandle,
                         intMask,
                         PMIC_IRQ_UNMASK,
                         PMIC_IRQ_GPIO_FALL_INT_TYPE);

    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioSetValue(pPmicCoreHandle, pin, pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    /* To clear the interrupts*/
    pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, true);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioSetIntr(pPmicCoreHandle, pin , intrType, maskPol);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pinValue = PMIC_GPIO_LOW;
    pmicStatus = Pmic_gpioSetValue(pPmicCoreHandle, pin, pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    while(timeout--)
    {
        /* Delay added to avoid timeout */
        Osal_delay(1000);
        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        if((PMIC_ST_SUCCESS == pmicStatus) &&
           ((errStat.intStatus[irqNum/32U] & (1U << (irqNum % 32U))) != 0U))
        {
            while(irqNum != nextedIrq)
            {
                pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                                     &errStat,
                                                     &nextedIrq);
            }

            if(PMIC_ST_SUCCESS == pmicStatus)
            {
                /* clear the interrupt */
                pmicStatus = Pmic_irqClrErrStatus(pPmicCoreHandle,
                                                  irqNum);
                break;
            }
        }
    }

    if(0 > timeout)
    {
        pmicStatus = PMIC_ST_SUCCESS;
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
}

/*!
 * \brief   Test to Get Mask status of Gpio and Irq Interrupts
 */
static void test_pmic_gpio_irq_getMaskIntrStat(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    uint8_t  irqGpioNumMaxCnt, irqNumMaxCnt, irqCnt;
    bool     riseIntrMaskStat, fallIntrMaskStat, irqMaskStatus;

    test_pmic_print_unity_testcase_info(9887,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        irqGpioNumMaxCnt = PMIC_TPS6594X_IRQ_GPIO_11_INT_MASK_NUM;

        if(PMIC_SILICON_REV_ID_PG_1_0 == pPmicCoreHandle->pmicDevSiliconRev)
        {
            /* SOFT REBOOT is not valid for PG 1.0*/
            irqNumMaxCnt = PMIC_TPS6594X_IRQ_MAX_NUM_PG_1_0;
        }
        else
        {
            irqNumMaxCnt = PMIC_TPS6594X_IRQ_MAX_NUM_PG_2_0;
        }
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        irqGpioNumMaxCnt = PMIC_LP8764X_IRQ_GPIO_10_INT_MASK_NUM;

        if(PMIC_SILICON_REV_ID_PG_1_0 == pPmicCoreHandle->pmicDevSiliconRev)
        {
            /* SOFT REBOOT is not valid for PG 1.0*/
            irqNumMaxCnt = PMIC_LP8764X_IRQ_MAX_NUM_PG_1_0;
        }
        else
        {
            irqNumMaxCnt = PMIC_LP8764X_IRQ_MAX_NUM_PG_2_0;
        }
    }

    /* Masking All GPIO Interrupts */
    pmicStatus = Pmic_irqGpioMaskIntr(pPmicCoreHandle,
                                      PMIC_IRQ_GPIO_ALL_INT_MASK_NUM,
                                      PMIC_IRQ_MASK,
                                      PMIC_IRQ_GPIO_RISE_FALL_INT_TYPE);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);


     for(irqCnt = 0; irqCnt <= irqGpioNumMaxCnt; irqCnt++)
     {
        /* Get Mask status of Gpio Interrupt */
        pmicStatus = Pmic_irqGetGpioMaskIntr(pPmicCoreHandle,
                                             irqCnt,
                                             PMIC_IRQ_GPIO_RISE_FALL_INT_TYPE,
                                             &riseIntrMaskStat,
                                             &fallIntrMaskStat);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
        TEST_ASSERT_EQUAL(PMIC_IRQ_MASK, riseIntrMaskStat);
        TEST_ASSERT_EQUAL(PMIC_IRQ_MASK, fallIntrMaskStat);
     }

    /* Masking All Interrupts */
    pmicStatus = Pmic_irqMaskIntr(pPmicCoreHandle,
                                  PMIC_IRQ_ALL,
                                  PMIC_IRQ_MASK);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    /* Leo device Mask status */
    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        for(irqCnt = 0; irqCnt < irqNumMaxCnt; irqCnt++)
        {
            /* PMIC Mask registers are not available for these interrupts*/
            if((irqCnt == 0) ||  (irqCnt == 1) ||  (irqCnt == 2)  ||
               (irqCnt == 20) || (irqCnt == 21) || (irqCnt == 22) ||
               (irqCnt == 27) || (irqCnt == 30) || (irqCnt == 53) ||
               (irqCnt == 57) || (irqCnt == 61) || (irqCnt == 65) ||
               (irqCnt == 69) || (irqCnt == 73) || (irqCnt == 77) ||
               (irqCnt == 81) || (irqCnt == 85) )
            {
                continue;

            }

            /* PMIC GPIO Mask registers are tested as part of Pmic_irqGpioMaskIntr().
             * So NA for the Pmic_irqMaskIntr()  */
            if((irqCnt >= 39 ) && (irqCnt <= 49))
            {
             continue;

            }

        /* Get Mask status of Gpio Interrupt */
        pmicStatus = Pmic_irqGetMaskIntrStatus(pPmicCoreHandle,
                                             irqCnt,
                                             &irqMaskStatus);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
        TEST_ASSERT_EQUAL(PMIC_IRQ_MASK, irqMaskStatus);
        }
    }

    /* Hera device Mask status */
    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        for(irqCnt = 0; irqCnt < irqNumMaxCnt; irqCnt++)
        {
            /* PMIC Mask registers are not available for these interrupts*/
            if((irqCnt == 0)  || (irqCnt == 1)  || (irqCnt == 2)  ||
               (irqCnt == 17) || (irqCnt == 18) || (irqCnt == 19) ||
               (irqCnt == 23) || (irqCnt == 26) || (irqCnt == 42) ||
               (irqCnt == 45) || (irqCnt == 51) || (irqCnt == 55) ||
               (irqCnt == 59) || (irqCnt == 63))
            {
                continue;

            }

            /* PMIC GPIO Mask registers are tested as part of Pmic_irqGpioMaskIntr().
             * So NA for the Pmic_irqMaskIntr()  */
            if((irqCnt >= 32 ) && (irqCnt <= 41))
            {
             continue;

            }
        /* Get Mask status of Gpio Interrupt */
        pmicStatus = Pmic_irqGetMaskIntrStatus(pPmicCoreHandle,
                                             irqCnt,
                                             &irqMaskStatus);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
        TEST_ASSERT_EQUAL(PMIC_IRQ_MASK, irqMaskStatus);
        }
    }


    pmic_testResultUpdate_pass(9887,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
}

/*!
 * \brief   Test to Get UnMask status of Gpio and Irq Interrupts
 */
static void test_pmic_gpio_irq_getUnMaskIntrStat(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    uint8_t  irqGpioNumMaxCnt, irqNumMaxCnt, irqCnt;
    bool     riseIntrMaskStat, fallIntrMaskStat, irqMaskStatus;

    test_pmic_print_unity_testcase_info(9888,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    /*!
    * On J721E PG1.0, Application hangs only when the test software sets
    * ORD_SHUTDOWN_MASK and REG_CRC_ERR_MASK to '0' and other mask bits
    * is set to '1'.
    * As suggested by PMIC Team, Ignore the test for J721E PG1.0.
    * Not planning to debug this issue as the same test is working fine
    * for J721E PG2.0.
    */
    if(PMIC_SILICON_REV_ID_PG_1_0 == pPmicCoreHandle->pmicDevSiliconRev)
    {
        if(pmic_device_info == J721E_LEO_PMICB_DEVICE)
        {
            pmic_testResultUpdate_ignore(9888,
                                         pmic_gpio_tests,
                                         PMIC_GPIO_NUM_OF_TESTCASES);
        }
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        irqGpioNumMaxCnt = PMIC_TPS6594X_IRQ_GPIO_11_INT_MASK_NUM;

        if(PMIC_SILICON_REV_ID_PG_1_0 == pPmicCoreHandle->pmicDevSiliconRev)
        {
            /* SOFT REBOOT is not valid for PG 1.0*/
            irqNumMaxCnt = PMIC_TPS6594X_IRQ_MAX_NUM_PG_1_0;
        }
        else
        {
            irqNumMaxCnt = PMIC_TPS6594X_IRQ_MAX_NUM_PG_2_0;
        }
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        irqGpioNumMaxCnt = PMIC_LP8764X_IRQ_GPIO_10_INT_MASK_NUM;

        if(PMIC_SILICON_REV_ID_PG_1_0 == pPmicCoreHandle->pmicDevSiliconRev)
        {
            /* SOFT REBOOT is not valid for PG 1.0*/
            irqNumMaxCnt = PMIC_LP8764X_IRQ_MAX_NUM_PG_1_0;
        }
        else
        {
            irqNumMaxCnt = PMIC_LP8764X_IRQ_MAX_NUM_PG_2_0;
        }
    }

    /* Unmasking All GPIO Interrupts */
    pmicStatus = Pmic_irqGpioMaskIntr(pPmicCoreHandle,
                                      PMIC_IRQ_GPIO_ALL_INT_MASK_NUM,
                                      PMIC_IRQ_UNMASK,
                                      PMIC_IRQ_GPIO_RISE_FALL_INT_TYPE);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

     for(irqCnt = 0; irqCnt <= irqGpioNumMaxCnt; irqCnt++)
     {
        /* Get Mask status of Gpio Interrupt */
        pmicStatus = Pmic_irqGetGpioMaskIntr(pPmicCoreHandle,
                                             irqCnt,
                                             PMIC_IRQ_GPIO_RISE_FALL_INT_TYPE,
                                             &riseIntrMaskStat,
                                             &fallIntrMaskStat);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
        TEST_ASSERT_EQUAL(PMIC_IRQ_UNMASK, riseIntrMaskStat);
        TEST_ASSERT_EQUAL(PMIC_IRQ_UNMASK, fallIntrMaskStat);
     }

    /* Unmasking All Interrupts */
    pmicStatus = Pmic_irqMaskIntr(pPmicCoreHandle,
                                  PMIC_IRQ_ALL,
                                  PMIC_IRQ_UNMASK);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    /* Leo device Mask status */
    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        for(irqCnt = 0; irqCnt < irqNumMaxCnt; irqCnt++)
        {
            /* PMIC Mask registers are not available for these interrupts*/
            if((irqCnt == 0)  || (irqCnt == 1)  || (irqCnt == 2)  ||
               (irqCnt == 20) || (irqCnt == 21) || (irqCnt == 22) ||
               (irqCnt == 27) || (irqCnt == 30) || (irqCnt == 53) ||
               (irqCnt == 57) || (irqCnt == 61) || (irqCnt == 65) ||
               (irqCnt == 69) || (irqCnt == 73) || (irqCnt == 77) ||
               (irqCnt == 81) || (irqCnt == 85) )
            {
                continue;

            }

            /* PMIC GPIO Mask registers are tested as part of Pmic_irqGpioMaskIntr().
             * So NA for the Pmic_irqMaskIntr()  */
            if((irqCnt >= 39 ) && (irqCnt <= 49))
            {
             continue;

            }

        /* Get Mask status of Gpio Interrupt */
        pmicStatus = Pmic_irqGetMaskIntrStatus(pPmicCoreHandle,
                                             irqCnt,
                                             &irqMaskStatus);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
        TEST_ASSERT_EQUAL(PMIC_IRQ_UNMASK, irqMaskStatus);
        }
    }

    /* Hera device Mask status */
    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        for(irqCnt = 0; irqCnt < irqNumMaxCnt; irqCnt++)
        {
            /* PMIC UnMask registers are not available for these interrupts*/
            if((irqCnt == 0)  || (irqCnt == 1)  || (irqCnt == 2)  ||
               (irqCnt == 17) || (irqCnt == 18) || (irqCnt == 19) ||
               (irqCnt == 23) || (irqCnt == 26) || (irqCnt == 42) ||
               (irqCnt == 45) || (irqCnt == 51) || (irqCnt == 55) ||
               (irqCnt == 59) || (irqCnt == 63))
            {
                continue;

            }

            /* PMIC GPIO UnMask registers are tested as part of Pmic_irqGpioMaskIntr().
             * So NA for the Pmic_irqMaskIntr()  */
            if((irqCnt >= 32 ) && (irqCnt <= 41))
            {
             continue;

            }
        /* Get Mask status of Gpio Interrupt */
        pmicStatus = Pmic_irqGetMaskIntrStatus(pPmicCoreHandle,
                                             irqCnt,
                                             &irqMaskStatus);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
        TEST_ASSERT_EQUAL(PMIC_IRQ_UNMASK, irqMaskStatus);
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

    pmic_testResultUpdate_pass(9888,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_irqGetGpioMaskIntr: Parameter validation for handle
 */
static void test_pmic_gpio_getMaskIntrStatPrmValTst_handle(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    bool     riseIntrMaskStat, fallIntrMaskStat;

    test_pmic_print_unity_testcase_info(9889,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    /* Masking All GPIO Interrupts */
    pmicStatus = Pmic_irqGpioMaskIntr(pPmicCoreHandle,
                                      PMIC_IRQ_GPIO_ALL_INT_MASK_NUM,
                                      PMIC_IRQ_MASK,
                                      PMIC_IRQ_GPIO_RISE_FALL_INT_TYPE);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);


    /* Get Mask status of Gpio Interrupt */
    pmicStatus = Pmic_irqGetGpioMaskIntr(NULL,
                                         0U,
                                         PMIC_IRQ_GPIO_RISE_FALL_INT_TYPE,
                                         &riseIntrMaskStat,
                                         &fallIntrMaskStat);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, pmicStatus);

    pmic_testResultUpdate_pass(9889,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_irqGetGpioMaskIntr: Parameter validation for pFallIntrMaskStat
 */
static void test_pmic_gpio_getMaskIntrStatPrmValTst_pFallIntrMaskStat(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    bool     riseIntrMaskStat;

    test_pmic_print_unity_testcase_info(9890,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    /* Masking All GPIO Interrupts */
    pmicStatus = Pmic_irqGpioMaskIntr(pPmicCoreHandle,
                                      PMIC_IRQ_GPIO_ALL_INT_MASK_NUM,
                                      PMIC_IRQ_MASK,
                                      PMIC_IRQ_GPIO_RISE_FALL_INT_TYPE);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);


    /* Get Mask status of Gpio Interrupt */
    pmicStatus = Pmic_irqGetGpioMaskIntr(pPmicCoreHandle,
                                         0U,
                                         PMIC_IRQ_GPIO_RISE_FALL_INT_TYPE,
                                         &riseIntrMaskStat,
                                         NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(9890,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_irqGetGpioMaskIntr: Parameter validation for pRiseIntrMaskStat
 */
static void test_pmic_gpio_getMaskIntrStatPrmValTst_pRiseIntrMaskStat(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    bool  fallIntrMaskStat;

    test_pmic_print_unity_testcase_info(9891,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    /* Masking All GPIO Interrupts */
    pmicStatus = Pmic_irqGpioMaskIntr(pPmicCoreHandle,
                                      PMIC_IRQ_GPIO_ALL_INT_MASK_NUM,
                                      PMIC_IRQ_MASK,
                                      PMIC_IRQ_GPIO_RISE_FALL_INT_TYPE);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);


    /* Get Mask status of Gpio Interrupt */
    pmicStatus = Pmic_irqGetGpioMaskIntr(pPmicCoreHandle,
                                         0U,
                                         PMIC_IRQ_GPIO_RISE_FALL_INT_TYPE,
                                         NULL,
                                         &fallIntrMaskStat);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(9891,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_irqGetMaskIntrStatus: Parameter validation for handle
 */
static void test_pmic_gpio_irq_getMaskIntrStatPrmValTst_handle(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    bool  irqMaskStatus;

    test_pmic_print_unity_testcase_info(9892,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    /* Masking All Interrupts */
    pmicStatus = Pmic_irqMaskIntr(pPmicCoreHandle,
                                  PMIC_IRQ_ALL,
                                  PMIC_IRQ_MASK);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    /* Get Mask status of IRQ Interrupt */
    pmicStatus = Pmic_irqGetMaskIntrStatus(NULL,
                                           0U,
                                           &irqMaskStatus);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, pmicStatus);

    pmic_testResultUpdate_pass(9892,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_irqGetMaskIntrStatus: Parameter validation for pMaskStatus
 */
static void test_pmic_gpio_irq_getMaskIntrStatPrmValTst_pMaskStatus(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;

    test_pmic_print_unity_testcase_info(9893,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    /* Masking All Interrupts */
    pmicStatus = Pmic_irqMaskIntr(pPmicCoreHandle,
                                  PMIC_IRQ_ALL,
                                  PMIC_IRQ_MASK);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    /* Get Mask status of IRQ Interrupt */
    pmicStatus = Pmic_irqGetMaskIntrStatus(pPmicCoreHandle,
                                           0U,
                                           NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(9893,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
}

/*!
 * \brief   Parameter validation for pGpioCfg
 */
static void test_pmic_gpio_nPWRON_getCfgPrmValTest_pGpioCfg(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;

    test_pmic_print_unity_testcase_info(10558,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    pmicStatus = Pmic_gpioGetNPwronEnablePinConfiguration(pPmicCoreHandle,
                                                          NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(10558,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
}

/*!
 * \brief   Test to set/get Pin function as PMIC_TPS6594X_NPWRON_PINFUNC_NONE
 */
static void test_pmic_gpio_nPWRON_setCfg_pinFunc(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_GpioCfg_t gpioCfg    =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_NPWRON_PINFUNC_NONE,
        PMIC_GPIO_HIGH
    };
    Pmic_GpioCfg_t gpioCfg_rd = {PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT};

    test_pmic_print_unity_testcase_info(10559,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    if(J7VCL_HERA_PMICB_DEVICE == pmic_device_info)
    {
        pmic_testResultUpdate_ignore(10559,
                                     pmic_gpio_tests,
                                     PMIC_GPIO_NUM_OF_TESTCASES);
    }

    pmicStatus = Pmic_gpioSetNPwronEnablePinConfiguration(pPmicCoreHandle,
                                                          gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioGetNPwronEnablePinConfiguration(pPmicCoreHandle,
                                                          &gpioCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    TEST_ASSERT_EQUAL(gpioCfg.pinFunc, gpioCfg_rd.pinFunc);

    pmic_testResultUpdate_pass(10559,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
}

/*!
 * \brief   Test to set/get Pin polarity for Enable Pin
 */
static void test_pmic_gpio_enable_setCfg_pinPolarity(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_GpioCfg_t gpioCfg_rd = {PMIC_ENABLE_CFG_POLARITY_VALID_SHIFT};
    Pmic_GpioCfg_t gpioCfg    =
    {
        PMIC_ENABLE_CFG_POLARITY_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_GPIO_PINFUNC_NSLEEP1,
        PMIC_GPIO_HIGH
    };

    test_pmic_print_unity_testcase_info(10560,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    pmicStatus = Pmic_gpioSetNPwronEnablePinConfiguration(pPmicCoreHandle,
                                                          gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioGetNPwronEnablePinConfiguration(pPmicCoreHandle,
                                                          &gpioCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    TEST_ASSERT_EQUAL(gpioCfg.pinPolarity, gpioCfg_rd.pinPolarity);

    pmic_testResultUpdate_pass(10560,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
}

/*!
 * \brief   Test to set/get GPIO pullup/pull down control configuration
 */
static void test_pmic_gpio_setCfg_pullCtrl(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_GpioCfg_t gpioCfg_rd = {PMIC_GPIO_CFG_PULL_VALID_SHIFT};
    Pmic_GpioCfg_t gpioCfg    =
    {
        PMIC_GPIO_CFG_PULL_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_GPIO_PINFUNC_NSLEEP1,
        PMIC_GPIO_HIGH
    };

    test_pmic_print_unity_testcase_info(10561,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    gpioCfg.pullCtrl = PMIC_GPIO_PULL_DISABLED;
    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, 1U, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioGetConfiguration(pPmicCoreHandle, 1U, &gpioCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    TEST_ASSERT_EQUAL(gpioCfg.pullCtrl, gpioCfg_rd.pullCtrl);

    gpioCfg.pullCtrl = PMIC_GPIO_PULL_DOWN;
    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, 1U, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioGetConfiguration(pPmicCoreHandle, 1U, &gpioCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    TEST_ASSERT_EQUAL(gpioCfg.pullCtrl, gpioCfg_rd.pullCtrl);

    gpioCfg.pullCtrl = PMIC_GPIO_PULL_UP;
    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, 1U, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioGetConfiguration(pPmicCoreHandle, 1U, &gpioCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    TEST_ASSERT_EQUAL(gpioCfg.pullCtrl, gpioCfg_rd.pullCtrl);

    pmic_testResultUpdate_pass(10561,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
}

/*!
 * \brief   Test to set/get signal deglitch time enable configuration
 */
static void test_pmic_gpio_setCfg_deglitchTime(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_GpioCfg_t gpioCfg_rd = {PMIC_GPIO_CFG_DEGLITCH_VALID_SHIFT};
    Pmic_GpioCfg_t gpioCfg    =
    {
        PMIC_GPIO_CFG_DEGLITCH_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_GPIO_PINFUNC_NSLEEP1,
        PMIC_GPIO_HIGH
    };

    test_pmic_print_unity_testcase_info(10562,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, 1U, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioGetConfiguration(pPmicCoreHandle, 1U, &gpioCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    TEST_ASSERT_EQUAL(gpioCfg.deglitchEnable, gpioCfg_rd.deglitchEnable);

    pmic_testResultUpdate_pass(10562,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
}

/*!
 * \brief   Test to verify GPIO1 disable Interrupt
 */
static void test_pmic_gpio1_testDisable_interrupt(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    bool riseIntrMaskStat, fallIntrMaskStat;
    uint8_t      irqGpioNum;

    test_pmic_print_unity_testcase_info(10563,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    pmicStatus = Pmic_gpioSetIntr(pPmicCoreHandle,
                                  1U,
                                  PMIC_GPIO_DISABLE_INTERRUPT,
                                  PMIC_GPIO_POL_HIGH);

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        irqGpioNum = PMIC_TPS6594X_IRQ_GPIO_1_INT_MASK_NUM;

    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        irqGpioNum = PMIC_LP8764X_IRQ_GPIO_1_INT_MASK_NUM;

    }

    pmicStatus = Pmic_irqGetGpioMaskIntr(pPmicCoreHandle,
                                         irqGpioNum,
                                         PMIC_IRQ_GPIO_RISE_FALL_INT_TYPE,
                                         &riseIntrMaskStat,
                                         &fallIntrMaskStat);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    TEST_ASSERT_EQUAL(riseIntrMaskStat, PMIC_IRQ_MASK);
    TEST_ASSERT_EQUAL(fallIntrMaskStat, PMIC_IRQ_MASK);

    pmic_testResultUpdate_pass(10563,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
}

/*!
 * \brief   Parameter validation for pullCtrl
 */
static void test_pmic_gpio_setCfgPrmValTest_pullCtrl(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_GpioCfg_t gpioCfg    =
    {
        PMIC_GPIO_CFG_PULL_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        3U,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_GPIO_PINFUNC_NSLEEP1,
        PMIC_GPIO_HIGH
    };

    test_pmic_print_unity_testcase_info(10564,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, 1U, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(10564,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
}

/*!
 * \brief   Test to set/get nPWRON/Enable pull up control configuration
 */
static void test_pmic_gpio_nPWRON_setCfg_pullCtrl(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_GpioCfg_t gpioCfg    =
    {
        PMIC_GPIO_CFG_PULL_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_UP,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_GPIO_PINFUNC_NSLEEP1,
        PMIC_GPIO_HIGH
    };
    Pmic_GpioCfg_t gpioCfg_rd = {PMIC_GPIO_CFG_PULL_VALID_SHIFT};

    test_pmic_print_unity_testcase_info(10565,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    if(J7VCL_HERA_PMICB_DEVICE == pmic_device_info)
    {
        pmic_testResultUpdate_ignore(10565,
                                     pmic_gpio_tests,
                                     PMIC_GPIO_NUM_OF_TESTCASES);
    }

    pmicStatus = Pmic_gpioSetNPwronEnablePinConfiguration(pPmicCoreHandle,
                                                          gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioGetNPwronEnablePinConfiguration(pPmicCoreHandle,
                                                          &gpioCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    TEST_ASSERT_EQUAL(gpioCfg.pullCtrl, gpioCfg_rd.pullCtrl);

    pmic_testResultUpdate_pass(10565,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
}

/*!
 * \brief   Parameter validation for pPmicCoreHandle
 */
static void test_pmic_gpio_nPWRON_setCfgPrmValTest_handle(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
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

    test_pmic_print_unity_testcase_info(10566,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    pmicStatus = Pmic_gpioSetNPwronEnablePinConfiguration(NULL, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, pmicStatus);

    pmic_testResultUpdate_pass(10566,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
}

/*!
 * \brief   Parameter validation for pPmicCoreHandle
 */
static void test_pmic_gpio_nPWRON_getCfgPrmValTest_handle(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_GpioCfg_t gpioCfg_rd = {0U};

    test_pmic_print_unity_testcase_info(10567,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    pmicStatus = Pmic_gpioGetNPwronEnablePinConfiguration(NULL, &gpioCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, pmicStatus);

    pmic_testResultUpdate_pass(10567,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_gpioSetNPwronEnablePinConfiguration : Test to set/get
 *          signal deglitch time enable configuration
 */
static void test_pmic_gpio_nPWRON_setCfg_deglitchTime(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_GpioCfg_t gpioCfg_rd = {PMIC_GPIO_CFG_DEGLITCH_VALID_SHIFT};
    Pmic_GpioCfg_t gpioCfg =
    {
        PMIC_GPIO_CFG_DEGLITCH_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_GPIO_PINFUNC_NSLEEP1,
        PMIC_GPIO_HIGH
    };

    test_pmic_print_unity_testcase_info(10568,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    if(J7VCL_HERA_PMICB_DEVICE == pmic_device_info)
    {
        pmic_testResultUpdate_ignore(10568,
                                     pmic_gpio_tests,
                                     PMIC_GPIO_NUM_OF_TESTCASES);
    }

    pmicStatus = Pmic_gpioSetNPwronEnablePinConfiguration(pPmicCoreHandle,
                                                          gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioGetNPwronEnablePinConfiguration(pPmicCoreHandle,
                                                          &gpioCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    TEST_ASSERT_EQUAL(gpioCfg.deglitchEnable, gpioCfg_rd.deglitchEnable);

    pmic_testResultUpdate_pass(10568,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
}

/*!
 * \brief   Parameter Validation for deglitchEnable for NPwronEnablePin
 */
static void test_pmic_gpio_nPWRON_setCfgPrmValTest_deglitchEnable(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_GpioCfg_t gpioCfg =
    {
        PMIC_GPIO_CFG_DEGLITCH_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        2U,
        PMIC_TPS6594X_GPIO_PINFUNC_NSLEEP1,
        PMIC_GPIO_HIGH
    };

    test_pmic_print_unity_testcase_info(10569,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    if(J7VCL_HERA_PMICB_DEVICE == pmic_device_info)
    {
        pmic_testResultUpdate_ignore(10569,
                                     pmic_gpio_tests,
                                     PMIC_GPIO_NUM_OF_TESTCASES);
    }

    pmicStatus = Pmic_gpioSetNPwronEnablePinConfiguration(pPmicCoreHandle,
                                                          gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(10569,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
}

/*!
 * \brief   Parameter Validation for pullCtrl for NPwronEnablePin
 */
static void test_pmic_gpio_nPWRON_setCfgPrmValTest_pullCtrl(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_GpioCfg_t gpioCfg =
    {
        PMIC_GPIO_CFG_PULL_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        3U,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_GPIO_PINFUNC_NSLEEP1,
        PMIC_GPIO_HIGH
    };

    test_pmic_print_unity_testcase_info(10570,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    if(J7VCL_HERA_PMICB_DEVICE == pmic_device_info)
    {
        pmic_testResultUpdate_ignore(10570,
                                     pmic_gpio_tests,
                                     PMIC_GPIO_NUM_OF_TESTCASES);
    }

    pmicStatus = Pmic_gpioSetNPwronEnablePinConfiguration(pPmicCoreHandle, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(10570,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
}

/*!
 * \brief   Parameter validation for pin for Min Value
 */
static void test_pmic_gpio_setCfgPrmValTest_pin_minVal(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    uint8_t pin               = 0U;
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

    test_pmic_print_unity_testcase_info(10571,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(pmicStatus, PMIC_ST_ERR_INV_PARAM);

    pmic_testResultUpdate_pass(10571,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_gpioSetNPwronEnablePinConfiguration : Test to set/get
 *          Invalid validParams
 */
static void test_pmic_gpio_nPWRON_setCfg_validParams(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_GpioCfg_t gpioCfg    =
    {
        0x40,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_UP,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_GPIO_PINFUNC_NSLEEP1,
        PMIC_GPIO_HIGH
    };
    Pmic_GpioCfg_t gpioCfg_rd = {0x40};

    test_pmic_print_unity_testcase_info(10572,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    pmicStatus = Pmic_gpioSetNPwronEnablePinConfiguration(pPmicCoreHandle,
                                                          gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioGetNPwronEnablePinConfiguration(pPmicCoreHandle,
                                                          &gpioCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmic_testResultUpdate_pass(10572,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_irqGetMaskIntrStatus: Parameter validation for irqNum
 */
static void test_pmic_irqGetMaskIntrStatusPrmValTst_irqNum(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    bool  irqMaskStatus;

    test_pmic_print_unity_testcase_info(10753,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    /* Masking All Interrupts */
    pmicStatus = Pmic_irqMaskIntr(pPmicCoreHandle,
                                  PMIC_IRQ_ALL,
                                  PMIC_IRQ_MASK);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    /* Get Mask status of IRQ Interrupt */
    pmicStatus = Pmic_irqGetMaskIntrStatus(pPmicCoreHandle,
                                           90U,
                                           &irqMaskStatus);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(10753,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_irqClrErrStatus: Parameter validation for handle
 */
static void test_pmic_irqClrErrStatusPrmValTst_handle(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    uint8_t  irqNum           = 0U;

    test_pmic_print_unity_testcase_info(10754,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    /* Clear the interrupt */
    pmicStatus = Pmic_irqClrErrStatus(NULL, irqNum);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, pmicStatus);

    pmic_testResultUpdate_pass(10754,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_irqClrErrStatus: Parameter validation for irqNum
 */
static void test_pmic_irqClrErrStatusPrmValTst_irqNum(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;

    test_pmic_print_unity_testcase_info(10755,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    /* Clear the interrupt */
    pmicStatus = Pmic_irqClrErrStatus(pPmicCoreHandle, 90U);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(10755,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_irqGetErrStatus: Parameter validation for handle
 */
static void test_pmic_irqGetErrStatusPrmValTst_handle(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    bool clearIRQ             = false;
    Pmic_IrqStatus_t errStat  = {0U};

    test_pmic_print_unity_testcase_info(10756,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    /* To clear the interrupts*/
    pmicStatus = Pmic_irqGetErrStatus(NULL, &errStat, clearIRQ);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, pmicStatus);

    pmic_testResultUpdate_pass(10756,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_irqGetErrStatus: Parameter validation for pErrStat
 */
static void test_pmic_irqGetErrStatusPrmValTst_pErrStat(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    bool clearIRQ             = false;

    test_pmic_print_unity_testcase_info(10757,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    /* To clear the interrupts*/
    pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, NULL, clearIRQ);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(10757,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_irqMaskIntr: Parameter validation for handle
 */
static void test_pmic_irqMaskIntrPrmValTst_handle(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;

    test_pmic_print_unity_testcase_info(10758,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    /* Masking All Interrupts */
    pmicStatus = Pmic_irqMaskIntr(NULL, PMIC_IRQ_ALL, PMIC_IRQ_MASK);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, pmicStatus);

    pmic_testResultUpdate_pass(10758,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_getNextErrorStatus: Parameter validation for handle
 */
static void test_pmic_getNextErrorStatusPrmValTst_handle(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_IrqStatus_t errStat  = {0U};
    uint8_t  irqNum           = 0U;

    test_pmic_print_unity_testcase_info(10759,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    pmicStatus = Pmic_getNextErrorStatus(NULL, &errStat, &irqNum);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, pmicStatus);

    pmic_testResultUpdate_pass(10759,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_getNextErrorStatus: Parameter validation for pErrStat
 */
static void test_pmic_getNextErrorStatusPrmValTst_pErrStat(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    uint8_t  irqNum           = 0U;

    test_pmic_print_unity_testcase_info(10760,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle, NULL, &irqNum);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(10760,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_getNextErrorStatus: Parameter validation for pIrqNum
 */
static void test_pmic_getNextErrorStatusPrmValTst_pIrqNum(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_IrqStatus_t errStat  = {0U};

    test_pmic_print_unity_testcase_info(10761,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle, &errStat, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(10761,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_getNextErrorStatus: Test to check for error Status
 *          intStatus[0], intStatus[2] and intStatus[3] not equal to zero
 */
static void test_pmic_getNextErrorStatus_intStat0_intStat3(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_IrqStatus_t errStat  = {0U};
    uint8_t  irqNum           = 0U;

    test_pmic_print_unity_testcase_info(10762,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    errStat.intStatus[0] = 1U;
    pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle, &errStat, &irqNum);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    errStat.intStatus[2] = 1U;
    pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle, &errStat, &irqNum);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    errStat.intStatus[3] = 1U;
    pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle, &errStat, &irqNum);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmic_testResultUpdate_pass(10762,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_irqGpioMaskIntr: Parameter validation for handle
 */
static void test_pmic_irqGpioMaskIntrPrmValTst_handle(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;

    test_pmic_print_unity_testcase_info(10763,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    /* Masking All GPIO Interrupts */
    pmicStatus = Pmic_irqGpioMaskIntr(NULL,
                                      PMIC_IRQ_GPIO_ALL_INT_MASK_NUM,
                                      PMIC_IRQ_MASK,
                                      PMIC_IRQ_GPIO_RISE_FALL_INT_TYPE);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, pmicStatus);

    pmic_testResultUpdate_pass(10763,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_irqGpioMaskIntr: Parameter validation for irqGpioNum
 */
static void test_pmic_irqGpioMaskIntrPrmValTst_irqGpioNum(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;

    test_pmic_print_unity_testcase_info(10764,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    /* Masking All GPIO Interrupts */
    pmicStatus = Pmic_irqGpioMaskIntr(pPmicCoreHandle,
                                      13U,
                                      PMIC_IRQ_MASK,
                                      PMIC_IRQ_GPIO_RISE_FALL_INT_TYPE);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(10764,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_irqGpioMaskIntr: Parameter validation for gpioIntrType
 */
static void test_pmic_irqGpioMaskIntrPrmValTst_gpioIntrType(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;

    test_pmic_print_unity_testcase_info(10765,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    /* Masking All GPIO Interrupts */
    pmicStatus = Pmic_irqGpioMaskIntr(pPmicCoreHandle,
                                      PMIC_IRQ_GPIO_ALL_INT_MASK_NUM,
                                      PMIC_IRQ_MASK,
                                      0x3U);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(10765,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_irqGpioMaskIntr: Parameter validation for irqGpioNum on Hera device type
 */
static void test_pmic_irqGpioMaskIntrPrmValTst_irqGpioNum_hera(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;

    test_pmic_print_unity_testcase_info(10766,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(10766,
                                     pmic_gpio_tests,
                                     PMIC_GPIO_NUM_OF_TESTCASES);
    }

    /* Masking All GPIO Interrupts */
    pmicStatus = Pmic_irqGpioMaskIntr(pPmicCoreHandle,
                                      PMIC_TPS6594X_IRQ_GPIO_11_INT_MASK_NUM,
                                      PMIC_IRQ_MASK,
                                      PMIC_IRQ_GPIO_RISE_FALL_INT_TYPE);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(10766,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_irqGetGpioMaskIntr: Parameter validation for irqGpioNum
 */
static void test_pmic_irqGetGpioMaskIntrPrmValTst_irqGpioNum(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    bool     riseIntrMaskStat, fallIntrMaskStat;

    test_pmic_print_unity_testcase_info(10767,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    /* Masking All GPIO Interrupts */
    pmicStatus = Pmic_irqGpioMaskIntr(pPmicCoreHandle,
                                      PMIC_IRQ_GPIO_ALL_INT_MASK_NUM,
                                      PMIC_IRQ_MASK,
                                      PMIC_IRQ_GPIO_RISE_FALL_INT_TYPE);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    /* Get Mask status of Gpio Interrupt */
    pmicStatus = Pmic_irqGetGpioMaskIntr(pPmicCoreHandle,
                                         13U,
                                         PMIC_IRQ_GPIO_RISE_FALL_INT_TYPE,
                                         &riseIntrMaskStat,
                                         &fallIntrMaskStat);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(10767,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_irqGetGpioMaskIntr: Parameter validation for gpioIntrType
 */
static void test_pmic_irqGetGpioMaskIntrPrmValTst_gpioIntrType(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    bool     riseIntrMaskStat, fallIntrMaskStat;

    test_pmic_print_unity_testcase_info(10768,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    /* Masking All GPIO Interrupts */
    pmicStatus = Pmic_irqGpioMaskIntr(pPmicCoreHandle,
                                      PMIC_IRQ_GPIO_ALL_INT_MASK_NUM,
                                      PMIC_IRQ_MASK,
                                      PMIC_IRQ_GPIO_RISE_FALL_INT_TYPE);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    /* Get Mask status of Gpio Interrupt */
    pmicStatus = Pmic_irqGetGpioMaskIntr(pPmicCoreHandle,
                                         PMIC_IRQ_GPIO_ALL_INT_MASK_NUM,
                                         0x3U,
                                         &riseIntrMaskStat,
                                         &fallIntrMaskStat);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(10768,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_irqGetGpioMaskIntr: Parameter validation for irqGpioNum on Hera device type
 */
static void test_pmic_irqGetGpioMaskIntrPrmValTst_irqGpioNum_hera(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    bool     riseIntrMaskStat, fallIntrMaskStat;

    test_pmic_print_unity_testcase_info(10769,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(10769,
                                     pmic_gpio_tests,
                                     PMIC_GPIO_NUM_OF_TESTCASES);
    }

    /* Masking All GPIO Interrupts */
    pmicStatus = Pmic_irqGpioMaskIntr(pPmicCoreHandle,
                                      PMIC_IRQ_GPIO_ALL_INT_MASK_NUM,
                                      PMIC_IRQ_MASK,
                                      PMIC_IRQ_GPIO_RISE_FALL_INT_TYPE);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    /* Get Mask status of Gpio Interrupt */
    pmicStatus = Pmic_irqGetGpioMaskIntr(pPmicCoreHandle,
                                         PMIC_TPS6594X_IRQ_GPIO_11_INT_MASK_NUM,
                                         PMIC_IRQ_GPIO_RISE_FALL_INT_TYPE,
                                         &riseIntrMaskStat,
                                         &fallIntrMaskStat);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(10769,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_irqGpioMaskIntr: Test for Masking All GPIO Interrupts for
 *          Leo device type check - Code covrage
 */
static void test_pmic_irqGpioMaskIntr_deviceType_leo(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;

    test_pmic_print_unity_testcase_info(10770,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(10770,
                                     pmic_gpio_tests,
                                     PMIC_GPIO_NUM_OF_TESTCASES);
    }

    /* Masking All GPIO Interrupts */
    pmicStatus = Pmic_irqGpioMaskIntr(pPmicCoreHandle,
                                      PMIC_TPS6594X_IRQ_GPIO_11_INT_MASK_NUM,
                                      PMIC_IRQ_MASK,
                                      PMIC_IRQ_GPIO_RISE_FALL_INT_TYPE);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmic_testResultUpdate_pass(10770,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
}

/*!
 * \brief   Added for Coverage
 */
static void test_pmic_gpio_coverageGaps(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t pinValue = 0U;
    uint8_t pmicDeviceType;
    uint8_t  irqNum           = 0U;
    bool clearIRQ             = false;
    bool  irqMaskStatus;
    bool  riseIntrMaskStat, fallIntrMaskStat;
    Pmic_IrqStatus_t errStat  = {0U};
    Pmic_GpioCfg_t gpioCfg_rd = {0U};
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
    int32_t crcTestFlag = PMIC_STATUS_CRC_INIT_VAL;

    test_pmic_print_unity_testcase_info(8852,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    if(PMIC_INTF_SPI == pPmicCoreHandle->commMode)
    {
        pmic_testResultUpdate_ignore(8852,
                                     pmic_gpio_tests,
                                     PMIC_GPIO_NUM_OF_TESTCASES);
    }

#if defined(SOC_J721E)
    crcTestFlag = gCrcTestFlag_J721E;
#endif

#if defined(SOC_J7200)
    crcTestFlag = gCrcTestFlag_J7VCL;
#endif

    //Fault Injection Tests:
    Pmic_GpioCfg_t gpioCfg_ft =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_GPIO_PINFUNC_NSLEEP1,
        PMIC_GPIO_HIGH
    };

    if(J7VCL_HERA_PMICB_DEVICE != pmic_device_info)
    {
        gPmic_faultInjectCfg.enableFaultInjectionRead = 1U;
        gPmic_faultInjectCfg.readCount = 0;
        gPmic_faultInjectCfg.skipReadCount = 1;
        pmicStatus = Pmic_gpioTps6594xNPwronPinGetValue(pPmicCoreHandle, &pinValue);
        TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, pmicStatus);

        //Pmic_gpioGetPinFunc
        gPmic_faultInjectCfg.readCount = 0;
        gPmic_faultInjectCfg.skipReadCount = 1;
        gpioCfg_rd.validParams = PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT;
        pmicStatus =Pmic_gpioGetNPwronEnablePinConfiguration(pPmicCoreHandle, &gpioCfg_rd);
        TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, pmicStatus);

        //Pmic_gpioTps6594xGetNPwronPinConfiguration
        gPmic_faultInjectCfg.readCount = 0;
        gPmic_faultInjectCfg.skipReadCount = 1;
        gpioCfg_rd.validParams = PMIC_GPIO_CFG_DEGLITCH_VALID_SHIFT;
        pmicStatus = Pmic_gpioGetNPwronEnablePinConfiguration(pPmicCoreHandle, &gpioCfg_rd);
        TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, pmicStatus);

        gPmic_faultInjectCfg.enableFaultInjectionRead = 0U;
    }

    //Pmic_gpioSetPinFunc
    gpioCfg_ft.validParams = PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT;
    gPmic_faultInjectCfg.enableFaultInjectionRead = 1U;
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, 1U, gpioCfg_ft);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, pmicStatus);

    //Pmic_gpioSetPinPolarity
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;

    gpioCfg_ft.validParams = PMIC_ENABLE_CFG_POLARITY_VALID_SHIFT;
    pmicStatus = Pmic_gpioSetNPwronEnablePinConfiguration(pPmicCoreHandle, gpioCfg_ft);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, pmicStatus);

    //Pmic_gpioGetPinPolarity
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;

    gpioCfg_rd.validParams = PMIC_ENABLE_CFG_POLARITY_VALID_SHIFT;
    pmicStatus = Pmic_gpioGetNPwronEnablePinConfiguration(pPmicCoreHandle, &gpioCfg_rd);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, pmicStatus);

    //Pmic_gpioSetPullCtrl
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    gpioCfg_ft.validParams = PMIC_GPIO_CFG_PULL_VALID_SHIFT;
    gpioCfg_ft.pullCtrl = PMIC_GPIO_PULL_DISABLED;
    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, 1U, gpioCfg_ft);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, pmicStatus);

    //Pmic_gpioGetPullCtrl
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    gpioCfg_rd.validParams = PMIC_GPIO_CFG_PULL_VALID_SHIFT;
    pmicStatus = Pmic_gpioGetConfiguration(pPmicCoreHandle, 1U, &gpioCfg_rd);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, pmicStatus);

    //Pmic_gpioSetPinDir
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    gpioCfg_ft.validParams = PMIC_GPIO_CFG_DIR_VALID_SHIFT;
    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, 1U, gpioCfg_ft);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, pmicStatus);

    //Pmic_gpioGetPinDir
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    gpioCfg_rd.validParams = PMIC_GPIO_CFG_DIR_VALID_SHIFT;
    pmicStatus = Pmic_gpioGetConfiguration(pPmicCoreHandle, 1U, &gpioCfg_rd);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, pmicStatus);

    //Pmic_gpioSetDeglitchTime and Pmic_gpioGetDeglitchTime
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    gpioCfg_ft.validParams = PMIC_GPIO_CFG_DEGLITCH_VALID_SHIFT;
    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, 1U, gpioCfg_ft);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, pmicStatus);

    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    gpioCfg_rd.validParams = PMIC_GPIO_CFG_DEGLITCH_VALID_SHIFT;
    pmicStatus = Pmic_gpioGetConfiguration(pPmicCoreHandle, 1U, &gpioCfg_rd);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, pmicStatus);

    //Pmic_gpioIntrEnable
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    pmicStatus = Pmic_gpioSetIntr(pPmicCoreHandle, 1U, PMIC_GPIO_FALL_INTERRUPT, PMIC_GPIO_POL_HIGH);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, pmicStatus);

    //Pmic_gpioIntrDisable
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    pmicStatus = Pmic_gpioSetIntr(pPmicCoreHandle, 1U, PMIC_GPIO_DISABLE_INTERRUPT, PMIC_GPIO_POL_HIGH);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, pmicStatus);

    //Pmic_gpioSetValue
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    pmicStatus = Pmic_gpioSetValue(pPmicCoreHandle, 1U, PMIC_GPIO_HIGH);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, pmicStatus);

    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 2;
    pmicStatus = Pmic_gpioSetValue(pPmicCoreHandle, 1U, PMIC_GPIO_HIGH);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, pmicStatus);

    //Pmic_gpioGetValue
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    pmicStatus = Pmic_gpioGetValue(pPmicCoreHandle, 1U, &pinValue);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, pmicStatus);

    //Pmic_gpioSetOutputSignalType
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    gpioCfg_ft.validParams = PMIC_GPIO_CFG_OD_VALID_SHIFT;
    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, 1U, gpioCfg_ft);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, pmicStatus);

    //Pmic_gpioGetOutputSignalType
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    gpioCfg_rd.validParams = PMIC_GPIO_CFG_OD_VALID_SHIFT;
    pmicStatus = Pmic_gpioGetConfiguration(pPmicCoreHandle, 1U, &gpioCfg_rd);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, pmicStatus);

    //Pmic_irqGpioMask
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    pmicStatus = Pmic_irqGpioMaskIntr(pPmicCoreHandle,
                                      11U,
                                      PMIC_IRQ_MASK,
                                      PMIC_IRQ_GPIO_RISE_INT_TYPE);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, pmicStatus);

    //Pmic_irqGpioMask
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    pmicStatus = Pmic_irqGpioMaskIntr(pPmicCoreHandle,
                                      11U,
                                      PMIC_IRQ_MASK,
                                      PMIC_IRQ_GPIO_FALL_INT_TYPE);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, pmicStatus);

    //Pmic_irqClear
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        irqNum = PMIC_TPS6594X_GPIO1_INT;
    }
    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        irqNum = PMIC_LP8764X_GPIO1_INT;
    }
    pmicStatus = Pmic_irqClrErrStatus(pPmicCoreHandle, irqNum);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, pmicStatus);

    //Pmic_irqGetErrStatus
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, pmicStatus);

    if((PMIC_STATUS_CRC_INIT_VAL == crcTestFlag) &&
       (J721E_LEO_PMICB_DEVICE != pmic_device_info))
    {
        gIntrTopRegTestFlag  = 1U;
        girqGetL1RegTestFlag_Leo = 1U;
        girqGetL2RegTestFlag_Leo = 1U;
        //Pmic_irqGetErrStatus
        gPmic_faultInjectCfg.readCount = 0;
        gPmic_faultInjectCfg.skipReadCount = 2;
        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, pmicStatus);
        gIntrTopRegTestFlag  = 0U;
        girqGetL1RegTestFlag_Leo = 0U;
        girqGetL2RegTestFlag_Leo = 0U;
    }

    //Pmic_getIrqGpioMaskStatus
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    pmicStatus = Pmic_irqGetGpioMaskIntr(pPmicCoreHandle,
                                         8U,
                                         PMIC_IRQ_GPIO_RISE_INT_TYPE,
                                         &riseIntrMaskStat,
                                         &fallIntrMaskStat);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, pmicStatus);

    //Pmic_getIrqGpioMaskStatus
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    pmicStatus = Pmic_irqGetGpioMaskIntr(pPmicCoreHandle,
                                         8U,
                                         PMIC_IRQ_GPIO_FALL_INT_TYPE,
                                         &riseIntrMaskStat,
                                         &fallIntrMaskStat);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, pmicStatus);

    //Pmic_getIrqMaskStatus
    pmicStatus = Pmic_irqMaskIntr(pPmicCoreHandle,
                                  24U,
                                  PMIC_IRQ_MASK);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    pmicStatus = Pmic_irqGetMaskIntrStatus(pPmicCoreHandle, 24U, &irqMaskStatus);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, pmicStatus);

    //Pmic_irqMask
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    pmicStatus = Pmic_irqMaskIntr(pPmicCoreHandle,
                                  24U,
                                  PMIC_IRQ_MASK);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, pmicStatus);

    if((PMIC_STATUS_CRC_INIT_VAL == crcTestFlag) &&
       (PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType))
    {
        //Pmic_tps6594x_getBuck1Buck2Err
        gPmic_faultInjectCfg.readCount = 0;
        gPmic_faultInjectCfg.skipReadCount = 20;
        gIntrTopRegTestFlag  = 1U;
        girqGetL1RegTestFlag_Leo = 1U;
        girqGetL2RegTestFlag_Leo = 1U;
        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, pmicStatus);

        //Pmic_tps6594x_getBuck3Buck4Err
        gPmic_faultInjectCfg.readCount = 0;
        gPmic_faultInjectCfg.skipReadCount = 19;
        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        //Pmic_tps6594x_getBuck5Err
        gPmic_faultInjectCfg.readCount = 0;
        gPmic_faultInjectCfg.skipReadCount = 18;
        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        //Pmic_tps6594x_getLdoVmonErr
        gPmic_faultInjectCfg.readCount = 0;
        gPmic_faultInjectCfg.skipReadCount = 16;
        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, pmicStatus);

        //Pmic_tps6594x_getLdo3Ldo4Err
        gPmic_faultInjectCfg.readCount = 0;
        gPmic_faultInjectCfg.skipReadCount = 15;
        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        //Pmic_tps6594x_getLdo1Ldo2Err
        gPmic_faultInjectCfg.readCount = 0;
        gPmic_faultInjectCfg.skipReadCount = 14;
        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        //Pmic_tps6594x_getGpioErr
        gPmic_faultInjectCfg.readCount = 0;
        gPmic_faultInjectCfg.skipReadCount = 12;
        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, pmicStatus);

        //Pmic_tps6594x_getStartupErr
        gPmic_faultInjectCfg.readCount = 0;
        gPmic_faultInjectCfg.skipReadCount = 10;
        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, pmicStatus);

        //Pmic_tps6594x_getFsmEsmErr
        gPmic_faultInjectCfg.readCount = 0;
        gPmic_faultInjectCfg.skipReadCount = 5;
        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        //Pmic_tps6594x_getFsmReadbackErr
        gPmic_faultInjectCfg.readCount = 0;
        gPmic_faultInjectCfg.skipReadCount = 4;
        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        //Pmic_tps6594x_getFsmCommErr
        gPmic_faultInjectCfg.readCount = 0;
        gPmic_faultInjectCfg.skipReadCount = 3;
        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
        gIntrTopRegTestFlag  = 0U;
        girqGetL1RegTestFlag_Leo = 0U;
        girqGetL2RegTestFlag_Leo = 0U;
    }

    if((PMIC_STATUS_CRC_INIT_VAL == crcTestFlag) &&
       (J721E_LEO_PMICB_DEVICE == pmic_device_info))
    {
        //Pmic_lp8764x_getFSMErr
        gIntrTopRegTestFlag      = 1U;
        girqGetL1RegTestFlag_Leo_PMICB = 1U;
        girqGetL2RegTestFlag_Leo = 1U;
        gPmic_faultInjectCfg.readCount = 0;
        gPmic_faultInjectCfg.skipReadCount = 6;
        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, pmicStatus);
        gIntrTopRegTestFlag      = 0U;
        girqGetL1RegTestFlag_Leo_PMICB = 0U;
        girqGetL2RegTestFlag_Leo = 0U;
    }

    if((PMIC_STATUS_CRC_INIT_VAL == crcTestFlag) &&
       (PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType))
    {
        gIntrTopRegTestFlag  = 1U;
        girqGetL1RegTestFlag_Hera = 1U;
        girqGetL2RegTestFlag_Hera = 1U;
        //Pmic_lp8764x_getFSMErr
        gPmic_faultInjectCfg.readCount = 0;
        gPmic_faultInjectCfg.skipReadCount = 6;
        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, pmicStatus);

        //Pmic_lp8764x_getFsmReadbackEsmErr
        gPmic_faultInjectCfg.readCount = 0;
        gPmic_faultInjectCfg.skipReadCount = 5;
        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        //Pmic_lp8764x_getFsmReadbackEsmErr
        gPmic_faultInjectCfg.readCount = 0;
        gPmic_faultInjectCfg.skipReadCount = 4;
        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        //Pmic_lp8764x_getFsmCommErr
        gPmic_faultInjectCfg.readCount = 0;
        gPmic_faultInjectCfg.skipReadCount = 3;
        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        //Pmic_lp8764x_getGpioErr
        gPmic_faultInjectCfg.readCount = 0;
        gPmic_faultInjectCfg.skipReadCount = 12;
        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, pmicStatus);

        //Pmic_lp8764x_getBuck3Buck4Err
        gPmic_faultInjectCfg.readCount = 0;
        gPmic_faultInjectCfg.skipReadCount = 15;
        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        //Pmic_lp8764x_getBuck1Buck2Err
        gPmic_faultInjectCfg.readCount = 0;
        gPmic_faultInjectCfg.skipReadCount = 16;
        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, pmicStatus);

        gIntrTopRegTestFlag  = 0U;
        girqGetL1RegTestFlag_Hera = 0U;
        girqGetL2RegTestFlag_Hera = 0U;
    }

        gPmic_faultInjectCfg.enableFaultInjectionRead = 0U;

    //Pmic_maskIntr
    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        irqNum = PMIC_TPS6594X_GPIO8_INT;
    }
    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        irqNum = PMIC_LP8764X_GPIO8_INT;
    }
    pmicStatus = Pmic_irqMaskIntr(pPmicCoreHandle, irqNum, PMIC_IRQ_MASK);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_FAIL, pmicStatus);

    //Pmic_getMaskIntrStatus
    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        irqNum = PMIC_TPS6594X_GPIO8_INT;
    }
    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        irqNum = PMIC_LP8764X_GPIO8_INT;
    }
    pmicStatus = Pmic_irqGetMaskIntrStatus(pPmicCoreHandle, irqNum, &irqMaskStatus);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_FAIL, pmicStatus);

    if((PMIC_STATUS_CRC_INIT_VAL == crcTestFlag) &&
       (PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType))
    {
        /* Enable top register, L1 Register and L2 Register */
        gIntrTopRegTestFlag      = 1U;
        girqGetL1RegTestFlag_Leo = 1U;
        girqGetL2RegTestFlag_Leo = 1U;
        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
        gIntrTopRegTestFlag      = 0U;
        girqGetL1RegTestFlag_Leo = 0U;
        girqGetL2RegTestFlag_Leo = 0U;

        /* Enable top register and L1 Register */
        gIntrTopRegTestFlag      = 1U;
        girqGetL1RegTestFlag_Leo = 1U;
        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
        gIntrTopRegTestFlag      = 0U;
        girqGetL1RegTestFlag_Leo = 0U;

        /* Enable top register, L1 Register and 1st bit of L2 register */
        gIntrTopRegTestFlag      = 1U;
        girqGetL1RegTestFlag_Leo = 1U;
        girq1L2RegTestFlag         = 1U;
        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
        gIntrTopRegTestFlag      = 0U;
        girqGetL1RegTestFlag_Leo = 0U;
        girq1L2RegTestFlag         = 0U;

        /* Enable top register, L1 Register and 2nd bit of L2 register */
        gIntrTopRegTestFlag      = 1U;
        girqGetL1RegTestFlag_Leo = 1U;
        girq2L2RegTestFlag         = 1U;
        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
        gIntrTopRegTestFlag      = 0U;
        girqGetL1RegTestFlag_Leo = 0U;
        girq2L2RegTestFlag         = 0U;

        /* Enable top register and 1st bit of L1 register */
        gIntrTopRegTestFlag      = 1U;
        girq1L1RegTestFlag         = 1U;
        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
        gIntrTopRegTestFlag      = 0U;
        girq1L1RegTestFlag         = 0U;

        /* Enable top register and 2nd bit of L1 register */
        gIntrTopRegTestFlag      = 1U;
        girq2L1RegTestFlag         = 1U;
        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
        gIntrTopRegTestFlag      = 0U;
        girq2L1RegTestFlag         = 0U;

        /* Enable top register, L1 register and 2nd bit of L2 register */
        gIntrTopRegTestFlag      = 1U;
        girqGetL1RegTestFlag_Leo = 1U;
        girq1L2RegTestFlag         = 1U;
        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
        gIntrTopRegTestFlag      = 0U;
        girqGetL1RegTestFlag_Leo = 0U;
        girq1L2RegTestFlag         = 0U;

        /* Enable top register, L1 register and 2nd bit of L2 register */
        gIntrTopRegTestFlag      = 1U;
        girqGetL1RegTestFlag_Leo = 1U;
        girq2L2RegTestFlag         = 1U;
        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
        gIntrTopRegTestFlag      = 0U;
        girqGetL1RegTestFlag_Leo = 0U;
        girq2L2RegTestFlag         = 0U;
    }

    if((PMIC_STATUS_CRC_INIT_VAL == crcTestFlag) &&
       (J721E_LEO_PMICB_DEVICE == pmic_device_info))
    {
        /* Enable top register L1 Register and all bit of wdg status Register */
        gIntrTopRegTestFlag      = 1U;
        girqGetL1RegTestFlag_Leo_PMICB = 1U;
        girqGetL2RegTestFlag_Leo = 1U;
        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
        gIntrTopRegTestFlag      = 0U;
        girqGetL1RegTestFlag_Leo_PMICB = 0U;
        girqGetL2RegTestFlag_Leo = 0U;

        /* Enable top register, L1 Register and 1st bit of wdg status Register */
        gIntrTopRegTestFlag      = 1U;
        girqGetL1RegTestFlag_Leo_PMICB = 1U;
        girq1L2RegTestFlag = 1U;
        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
        gIntrTopRegTestFlag      = 0U;
        girqGetL1RegTestFlag_Leo_PMICB = 0U;
        girq1L2RegTestFlag = 0U;

        /* Enable top register, L1 Register and 6th bit of wdg status Register */
        gIntrTopRegTestFlag      = 1U;
        girqGetL1RegTestFlag_Leo_PMICB = 1U;
        girq2L2RegTestFlag = 1U;
        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
        gIntrTopRegTestFlag      = 0U;
        girqGetL1RegTestFlag_Leo_PMICB = 0U;
        girq2L2RegTestFlag = 0U;

        /* Enable top register and L1 Register */
        gIntrTopRegTestFlag      = 1U;
        girqGetL1RegTestFlag_Leo_PMICB = 1U;
        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
        gIntrTopRegTestFlag      = 0U;
        girqGetL1RegTestFlag_Leo_PMICB = 0U;
    }

    if((PMIC_STATUS_CRC_INIT_VAL == crcTestFlag) &&
       (PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType))
    {
        /* Enable top register, L1 Register and L2 Register */
        gIntrTopRegTestFlag       = 1U;
        girqGetL1RegTestFlag_Hera = 1U;
        girqGetL2RegTestFlag_Hera = 1U;
        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
        gIntrTopRegTestFlag       = 0U;
        girqGetL1RegTestFlag_Hera = 0U;
        girqGetL2RegTestFlag_Hera = 0U;

        /* Enable top register and L1 Register */
        gIntrTopRegTestFlag       = 1U;
        girqGetL1RegTestFlag_Hera = 1U;
        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
        gIntrTopRegTestFlag       = 0U;
        girqGetL1RegTestFlag_Hera = 0U;

        /* Enable top register, L1 Register and 1st bit of L2 register */
        gIntrTopRegTestFlag       = 1U;
        girqGetL1RegTestFlag_Hera = 1U;
        girq1L2RegTestFlag = 1U;
        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
        gIntrTopRegTestFlag       = 0U;
        girqGetL1RegTestFlag_Hera = 0U;
        girq1L2RegTestFlag = 0U;

        /* Enable top register, L1 Register and 2nd bit of L2 register */
        gIntrTopRegTestFlag      = 1U;
        girqGetL1RegTestFlag_Hera = 1U;
        girq2L2RegTestFlag         = 1U;
        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
        gIntrTopRegTestFlag      = 0U;
        girqGetL1RegTestFlag_Hera = 0U;
        girq2L2RegTestFlag         = 0U;

        /* Enable top register and 1st bit of L1 register */
        gIntrTopRegTestFlag      = 1U;
        girq1L1RegTestFlag         = 1U;
        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
        gIntrTopRegTestFlag      = 0U;
        girq1L1RegTestFlag         = 0U;

        /* Enable top register,and 2nd bit of L1 register */
        gIntrTopRegTestFlag      = 1U;
        girq2L1RegTestFlag         = 1U;
        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
        gIntrTopRegTestFlag      = 0U;
        girq2L1RegTestFlag         = 0U;
    }

    Pmic_DevSubSysInfo_t pmicDevSubSysInfo =
    {
        .gpioEnable = (bool)false,
        .rtcEnable  = (bool)true,
        .wdgEnable  = (bool)true,
        .buckEnable = (bool)true,
        .ldoEnable  = (bool)true,
        .esmEnable  = (bool)true
    };

    pPmicCoreHandle->pPmic_SubSysInfo = (&pmicDevSubSysInfo);
    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, 1U, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_DEVICE, pmicStatus);

    pmicStatus = Pmic_gpioTps6594xNPwronPinGetValue(pPmicCoreHandle, &pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_DEVICE, pmicStatus);

    pmicDeviceType = pPmicCoreHandle->pmicDeviceType;
    pPmicCoreHandle->pmicDeviceType = 3U;
    pmicStatus = Pmic_gpioTps6594xNPwronPinGetValue(pPmicCoreHandle, &pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_DEVICE, pmicStatus);
    pPmicCoreHandle->pmicDeviceType = pmicDeviceType;

    pmic_testResultUpdate_pass(8852,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
}

#if defined(ENABLE_SAMPLE_TESTCASES)
/*!
 * Below test cases are not tested because of HW limitation.
 * Added below test cases as sample for reference.
 */

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

    test_pmic_print_unity_testcase_info(0xAB1E,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    if(J721E_LEO_PMICA_DEVICE == pmic_device_info)
    {
        /*
         * GPIO3 pin of PMIC-A is connected to 'EN_MCU3V3_LDSW'
         * on J721EVM board causing hang, when programming NERR_SOC signal.
         */
        pmic_testResultUpdate_ignore(0xAB1E,
                                         pmic_gpio_tests,
                                         PMIC_GPIO_NUM_OF_TESTCASES);
    }

    if(J721E_LEO_PMICB_DEVICE == pmic_device_info)
    {
        pmic_testResultUpdate_ignore(0xAB1E,
                                         pmic_gpio_tests,
                                         PMIC_GPIO_NUM_OF_TESTCASES);
    }

    if(J7VCL_LEO_PMICA_DEVICE == pmic_device_info)
    {
        pmic_testResultUpdate_ignore(0xAB1E,
                                         pmic_gpio_tests,
                                         PMIC_GPIO_NUM_OF_TESTCASES);
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(0xAB1E,
                                         pmic_gpio_tests,
                                         PMIC_GPIO_NUM_OF_TESTCASES);
    }

    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioGetConfiguration(pPmicCoreHandle, pin, &gpioCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(gpioCfg.pinFunc, gpioCfg_rd.pinFunc);

    pmic_testResultUpdate_pass(0xAB1E,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
}

/*!
 * Below test cases are not tested because of HW limitation.
 * Added below test cases as sample for reference.
 */

/*!
 * \brief   Test to verify GPIO3 fall interrupt
 */
static void test_pmic_gpio3_testFall_interrupt(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    int8_t timeout            = 10U;
    uint8_t pin               = 3U;
    uint8_t pinValue          = PMIC_GPIO_HIGH;
    uint8_t intrType          = PMIC_GPIO_FALL_INTERRUPT;
    uint8_t maskPol           = PMIC_GPIO_POL_LOW;
    Pmic_IrqStatus_t errStat  = {0U};
    bool clearIRQ             = false;
    uint8_t  irqNum           = 0U;
    uint8_t  nextedIrq        = 0U;
    uint8_t  intMask          = 0U;
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

    test_pmic_print_unity_testcase_info(0xAB19,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        /* Should not break SPI Comm Lines */
        if(PMIC_INTF_SPI == pPmicCoreHandle->commMode)
        {
            pmic_testResultUpdate_ignore(0xAB19,
                                         pmic_gpio_tests,
                                         PMIC_GPIO_NUM_OF_TESTCASES);
        }
    }
    if(J721E_LEO_PMICA_DEVICE == pmic_device_info)
    {
        /*
         * GPIO3 pin of PMIC-A is connected to 'EN_MCU3V3_LDSW'
         * on J721EVM board causing hang, when programming fall interrupt.
         */
         pmic_testResultUpdate_ignore(0xAB19,
                                         pmic_gpio_tests,
                                         PMIC_GPIO_NUM_OF_TESTCASES);
    }
    if(J721E_LEO_PMICB_DEVICE == pmic_device_info)
    {
        /*
         * GPIO3 pin of PMIC-B is connected to 'EN_DDR0V6_BUCK'
         * on J721EVM board causing hang, when programming fall interrupt.
         */
        pmic_testResultUpdate_ignore(0xAB19,
                                         pmic_gpio_tests,
                                         PMIC_GPIO_NUM_OF_TESTCASES);
    }

    if(J7VCL_LEO_PMICA_DEVICE == pmic_device_info)
    {
        pmic_testResultUpdate_ignore(0xAB19,
                                         pmic_gpio_tests,
                                         PMIC_GPIO_NUM_OF_TESTCASES);
    }

    if(J7VCL_HERA_PMICB_DEVICE == pmic_device_info)
    {
        gpioCfg.validParams = PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT |
                              PMIC_GPIO_CFG_DIR_VALID_SHIFT |
                              PMIC_GPIO_CFG_OD_VALID_SHIFT |
                              PMIC_GPIO_CFG_PULL_VALID_SHIFT;
        gpioCfg.outputSignalType = PMIC_GPIO_PUSH_PULL_OUTPUT;
        gpioCfg.pullCtrl = PMIC_GPIO_PULL_UP;
        gpioCfg.pinFunc = PMIC_LP8764X_GPIO_PINFUNC_GPIO;
    }

    if(J7VCL_HERA_PMICB_DEVICE == pmic_device_info)
    {
        pmic_testResultUpdate_ignore(0xAB19,
                                     pmic_gpio_tests,
                                     PMIC_GPIO_NUM_OF_TESTCASES);
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        intMask = PMIC_TPS6594X_IRQ_GPIO_3_INT_MASK_NUM;
        irqNum = PMIC_TPS6594X_GPIO3_INT;
    }
    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        intMask = PMIC_LP8764X_IRQ_GPIO_3_INT_MASK_NUM;
        irqNum = PMIC_LP8764X_GPIO3_INT;
    }
    /* Un Masking GPIO 3 FALL Interrupt */
    Pmic_irqGpioMaskIntr(pPmicCoreHandle,
                         intMask,
                         PMIC_IRQ_UNMASK,
                         PMIC_IRQ_GPIO_FALL_INT_TYPE);

    /* To clear the interrupts*/
    pmic_gpioTest_intClr();

    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioSetValue(pPmicCoreHandle, pin, pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioSetIntr(pPmicCoreHandle, pin , intrType, maskPol);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pinValue = PMIC_GPIO_LOW;
    pmicStatus = Pmic_gpioSetValue(pPmicCoreHandle, pin, pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    while(timeout--)
    {
        /* Delay added to avoid timeout */
        Osal_delay(1000);
        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        if((PMIC_ST_SUCCESS == pmicStatus) &&
           ((errStat.intStatus[irqNum/32U] & (1U << (irqNum % 32U))) != 0U))
        {
            while(irqNum != nextedIrq)
            {
                pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                                     &errStat,
                                                     &nextedIrq);
            }

            if(PMIC_ST_SUCCESS == pmicStatus)
            {
                /* clear the interrupt */
                pmicStatus = Pmic_irqClrErrStatus(pPmicCoreHandle,
                                                  irqNum);
                break;
            }
        }
    }

    if(0 > timeout)
    {
        pmicStatus = PMIC_ST_ERR_FAIL;
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmic_testResultUpdate_pass(0xAB19,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
}

/*!
 * Below test cases are not tested because of HW limitation.
 * Added below test cases as sample for reference.
 */

/*!
 * \brief   Test to verify GPIO3 rise interrupt
 */
static void test_pmic_gpio3_testRise_interrupt(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    int8_t timeout            = 10U;
    uint8_t pin               = 3U;
    uint8_t pinValue          = PMIC_GPIO_LOW;
    uint8_t intrType          = PMIC_GPIO_RISE_INTERRUPT;
    uint8_t maskPol           = PMIC_GPIO_POL_HIGH;
    Pmic_IrqStatus_t errStat  = {0U};
    bool clearIRQ             = false;
    uint8_t  irqNum           = 0U;
    uint8_t  nextedIrq        = 0U;
    uint8_t  intMask          = 0U;
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

    test_pmic_print_unity_testcase_info(0xAB18,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        /* Should not break SPI Comm Lines */
        if(PMIC_INTF_SPI == pPmicCoreHandle->commMode)
        {
            pmic_testResultUpdate_ignore(0xAB18,
                                         pmic_gpio_tests,
                                         PMIC_GPIO_NUM_OF_TESTCASES);
        }
    }
    if(J721E_LEO_PMICA_DEVICE == pmic_device_info)
    {
        /*
         * GPIO3 pin of PMIC-A is connected to 'EN_MCU3V3_LDSW'
         * on J721EVM board causing hang, when programming rise interrupt.
         */
         pmic_testResultUpdate_ignore(0xAB18,
                                         pmic_gpio_tests,
                                         PMIC_GPIO_NUM_OF_TESTCASES);
    }

    if(J721E_LEO_PMICB_DEVICE == pmic_device_info)
    {
        /*
         * GPIO3 pin of PMIC-B is connected to 'EN_DDR0V6_BUCK'
         * on J721EVM board causing hang, when programming rise interrupt.
         */
        pmic_testResultUpdate_ignore(0xAB18,
                                         pmic_gpio_tests,
                                         PMIC_GPIO_NUM_OF_TESTCASES);
    }

    if(J7VCL_LEO_PMICA_DEVICE == pmic_device_info)
    {
        pmic_testResultUpdate_ignore(0xAB18,
                                         pmic_gpio_tests,
                                         PMIC_GPIO_NUM_OF_TESTCASES);
    }

    if(J7VCL_HERA_PMICB_DEVICE == pmic_device_info)
    {
        gpioCfg.validParams = PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT |
                              PMIC_GPIO_CFG_DIR_VALID_SHIFT |
                              PMIC_GPIO_CFG_OD_VALID_SHIFT |
                              PMIC_GPIO_CFG_PULL_VALID_SHIFT;
        gpioCfg.outputSignalType = PMIC_GPIO_PUSH_PULL_OUTPUT;
        gpioCfg.pullCtrl = PMIC_GPIO_PULL_UP;
        gpioCfg.pinFunc = PMIC_LP8764X_GPIO_PINFUNC_GPIO;
    }

    if(J7VCL_HERA_PMICB_DEVICE == pmic_device_info)
    {
        pmic_testResultUpdate_ignore(0xAB18,
                                         pmic_gpio_tests,
                                         PMIC_GPIO_NUM_OF_TESTCASES);
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        intMask = PMIC_TPS6594X_IRQ_GPIO_3_INT_MASK_NUM;
        irqNum = PMIC_TPS6594X_GPIO3_INT;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        intMask = PMIC_LP8764X_IRQ_GPIO_3_INT_MASK_NUM;
        irqNum = PMIC_LP8764X_GPIO3_INT;
    }

    /* Un Masking GPIO 3 RISE Interrupt */
    Pmic_irqGpioMaskIntr(pPmicCoreHandle,
                         intMask,
                         PMIC_IRQ_UNMASK,
                         PMIC_IRQ_GPIO_RISE_INT_TYPE);

    /* To clear the interrupts*/
    pmic_gpioTest_intClr();

    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioSetValue(pPmicCoreHandle, pin, pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioSetIntr(pPmicCoreHandle, pin , intrType, maskPol);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pinValue = PMIC_GPIO_HIGH;
    pmicStatus = Pmic_gpioSetValue(pPmicCoreHandle, pin, pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    while(timeout--)
    {
        /* Delay added to avoid timeout */
        Osal_delay(1000);
        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        if((PMIC_ST_SUCCESS == pmicStatus) &&
           ((errStat.intStatus[irqNum/32U] & (1U << (irqNum % 32U))) != 0U))
        {
            while(irqNum != nextedIrq)
            {
                pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                                     &errStat,
                                                     &nextedIrq);
            }

            if(PMIC_ST_SUCCESS == pmicStatus)
            {
                /* clear the interrupt */
                pmicStatus = Pmic_irqClrErrStatus(pPmicCoreHandle,
                                                  irqNum);
                break;
            }
        }
    }

    if(0 > timeout)
    {
        pmicStatus = PMIC_ST_ERR_FAIL;
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmic_testResultUpdate_pass(0xAB18,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
}

/*!
 * Below test cases are not tested because of HW limitation.
 * Added below test cases as sample for reference.
 */

/*!
 * \brief   Test to verify GPIO9 fall interrupt
 */
static void test_pmic_gpio9_testFall_interrupt(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    int8_t timeout            = 10U;
    uint8_t pin               = 9U;
    uint8_t pinValue          = PMIC_GPIO_HIGH;
    uint8_t intrType          = PMIC_GPIO_FALL_INTERRUPT;
    uint8_t maskPol           = PMIC_GPIO_POL_LOW;
    Pmic_IrqStatus_t errStat  = {0U};
    bool clearIRQ             = false;
    uint8_t  irqNum           = 0U;
    uint8_t  nextedIrq        = 0U;
    uint8_t  intMask          = 0U;
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

    test_pmic_print_unity_testcase_info(0xAB1D,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    if(J721E_LEO_PMICA_DEVICE == pmic_device_info)
    {
        /*
         * GPIO9 pin of PMIC-A is connected to 'GND'
         * on J721EVM board causing hang, when programming fall interrupt.
         */
        pmic_testResultUpdate_ignore(0xAB1D,
                                         pmic_gpio_tests,
                                         PMIC_GPIO_NUM_OF_TESTCASES);
    }
    if(J721E_LEO_PMICB_DEVICE == pmic_device_info)
    {
        /*
         * GPIO9 pin of PMIC-B is connected to 'EN_VPP1V8_LDO'
         * on J721EVM board causing hang, when programming fall interrupt.
         */
        pmic_testResultUpdate_ignore(0xAB1D,
                                         pmic_gpio_tests,
                                         PMIC_GPIO_NUM_OF_TESTCASES);
    }

    if(J7VCL_LEO_PMICA_DEVICE == pmic_device_info)
    {
        pmic_testResultUpdate_ignore(0xAB1D,
                                         pmic_gpio_tests,
                                         PMIC_GPIO_NUM_OF_TESTCASES);
    }

    if(J7VCL_HERA_PMICB_DEVICE == pmic_device_info)
    {
        gpioCfg.validParams = PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT |
                              PMIC_GPIO_CFG_DIR_VALID_SHIFT |
                              PMIC_GPIO_CFG_OD_VALID_SHIFT |
                              PMIC_GPIO_CFG_PULL_VALID_SHIFT;
        gpioCfg.outputSignalType = PMIC_GPIO_PUSH_PULL_OUTPUT;
        gpioCfg.pullCtrl = PMIC_GPIO_PULL_UP;
        gpioCfg.pinFunc = PMIC_LP8764X_GPIO_PINFUNC_GPIO;
    }

    if(J7VCL_HERA_PMICB_DEVICE == pmic_device_info)
    {
        pmic_testResultUpdate_ignore(0xAB1D,
                                         pmic_gpio_tests,
                                         PMIC_GPIO_NUM_OF_TESTCASES);
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        intMask = PMIC_TPS6594X_IRQ_GPIO_9_INT_MASK_NUM;
        irqNum = PMIC_TPS6594X_GPIO9_INT;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        intMask = PMIC_LP8764X_IRQ_GPIO_9_INT_MASK_NUM;
        irqNum = PMIC_LP8764X_GPIO9_INT;
    }

    /* Un Masking GPIO 9 FALL Interrupt */
    Pmic_irqGpioMaskIntr(pPmicCoreHandle,
                         intMask,
                         PMIC_IRQ_UNMASK,
                         PMIC_IRQ_GPIO_FALL_INT_TYPE);

    /* To clear the interrupts*/
    pmic_gpioTest_intClr();

    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioSetValue(pPmicCoreHandle, pin, pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioSetIntr(pPmicCoreHandle, pin , intrType, maskPol);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pinValue = PMIC_GPIO_LOW;
    pmicStatus = Pmic_gpioSetValue(pPmicCoreHandle, pin, pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    while(timeout--)
    {
        /* Delay added to avoid timeout */
        Osal_delay(1000);
        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        if((PMIC_ST_SUCCESS == pmicStatus) &&
           ((errStat.intStatus[irqNum/32U] & (1U << (irqNum % 32U))) != 0U))
        {
            while(irqNum != nextedIrq)
            {
                pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                                     &errStat,
                                                     &nextedIrq);
            }

            if(PMIC_ST_SUCCESS == pmicStatus)
            {
                /* clear the interrupt */
                pmicStatus = Pmic_irqClrErrStatus(pPmicCoreHandle,
                                                  irqNum);
                break;
            }
        }
    }

    if(0 > timeout)
    {
        pmicStatus = PMIC_ST_ERR_FAIL;
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmic_testResultUpdate_pass(0xAB1D,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
}

/*!
 * Below test cases are not tested because of HW limitation.
 * Added below test cases as sample for reference.
 */

/*!
 * \brief   Test to verify GPIO9 rise interrupt
 */
static void test_pmic_gpio9_testRise_interrupt(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    int8_t timeout            = 10U;
    uint8_t pin               = 9U;
    uint8_t pinValue          = PMIC_GPIO_LOW;
    uint8_t intrType          = PMIC_GPIO_RISE_INTERRUPT;
    uint8_t maskPol           = PMIC_GPIO_POL_HIGH;
    Pmic_IrqStatus_t errStat  = {0U};
    bool clearIRQ             = false;
    uint8_t  irqNum           = 0U;
    uint8_t  nextedIrq        = 0U;
    uint8_t  intMask          = 0U;
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

    test_pmic_print_unity_testcase_info(0xAB1C,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    if(J721E_LEO_PMICA_DEVICE == pmic_device_info)
    {
        /*
         * GPIO9 pin of PMIC-A is connected to 'GND'
         * on J721EVM board causing hang, when programming rise interrupt.
         */
        pmic_testResultUpdate_ignore(0xAB1C,
                                         pmic_gpio_tests,
                                         PMIC_GPIO_NUM_OF_TESTCASES);
    }
    if(J721E_LEO_PMICB_DEVICE == pmic_device_info)
    {
        /*
         * GPIO9 pin of PMIC-B is connected to 'EN_VPP1V8_LDO'
         * on J721EVM board causing hang, when programming rise interrupt.
         */
        pmic_testResultUpdate_ignore(0xAB1C,
                                         pmic_gpio_tests,
                                         PMIC_GPIO_NUM_OF_TESTCASES);
    }

    if(J7VCL_LEO_PMICA_DEVICE == pmic_device_info)
    {
         pmic_testResultUpdate_ignore(0xAB1C,
                                         pmic_gpio_tests,
                                         PMIC_GPIO_NUM_OF_TESTCASES);
    }

    if(J7VCL_HERA_PMICB_DEVICE == pmic_device_info)
    {
        gpioCfg.validParams = PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT |
                              PMIC_GPIO_CFG_DIR_VALID_SHIFT |
                              PMIC_GPIO_CFG_OD_VALID_SHIFT |
                              PMIC_GPIO_CFG_PULL_VALID_SHIFT;
        gpioCfg.outputSignalType = PMIC_GPIO_PUSH_PULL_OUTPUT;
        gpioCfg.pullCtrl = PMIC_GPIO_PULL_UP;
        gpioCfg.pinFunc = PMIC_LP8764X_GPIO_PINFUNC_GPIO;
    }

    if(J7VCL_HERA_PMICB_DEVICE == pmic_device_info)
    {
        pmic_testResultUpdate_ignore(0xAB1C,
                                         pmic_gpio_tests,
                                         PMIC_GPIO_NUM_OF_TESTCASES);
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        intMask = PMIC_TPS6594X_IRQ_GPIO_9_INT_MASK_NUM;
        irqNum = PMIC_TPS6594X_GPIO9_INT;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        intMask = PMIC_LP8764X_IRQ_GPIO_9_INT_MASK_NUM;
        irqNum = PMIC_LP8764X_GPIO9_INT;
    }

    /* Un Masking GPIO 9 RISE Interrupt */
    Pmic_irqGpioMaskIntr(pPmicCoreHandle,
                         intMask,
                         PMIC_IRQ_UNMASK,
                         PMIC_IRQ_GPIO_RISE_INT_TYPE);

    /* To clear the interrupts*/
    pmic_gpioTest_intClr();

    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioSetValue(pPmicCoreHandle, pin, pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioSetIntr(pPmicCoreHandle, pin , intrType, maskPol);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pinValue = PMIC_GPIO_HIGH;
    pmicStatus = Pmic_gpioSetValue(pPmicCoreHandle, pin, pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    while(timeout--)
    {
        /* Delay added to avoid timeout */
        Osal_delay(1000);
        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        if((PMIC_ST_SUCCESS == pmicStatus) &&
           ((errStat.intStatus[irqNum/32U] & (1U << (irqNum % 32U))) != 0U))
        {
            while(irqNum != nextedIrq)
            {
                pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                                     &errStat,
                                                     &nextedIrq);
            }

            if(PMIC_ST_SUCCESS == pmicStatus)
            {
                /* clear the interrupt */
                pmicStatus = Pmic_irqClrErrStatus(pPmicCoreHandle,
                                                  irqNum);
                break;
            }
        }
    }

    if(0 > timeout)
    {
        pmicStatus = PMIC_ST_ERR_FAIL;
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmic_testResultUpdate_pass(0xAB1C,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
}

/*!
 * Below test cases are not tested because of HW limitation.
 * Added below test cases as sample for reference.
 */

/*!
 * \brief   Test to verify GPIO11 fall interrupt
 */
static void test_pmic_gpio11_testFall_interrupt(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    int8_t timeout            = 10U;
    uint8_t pin               = 11U;
    uint8_t pinValue          = PMIC_GPIO_HIGH;
    uint8_t intrType          = PMIC_GPIO_FALL_INTERRUPT;
    uint8_t maskPol           = PMIC_GPIO_POL_LOW;
    Pmic_IrqStatus_t errStat  = {0U};
    bool clearIRQ             = false;
    uint8_t  irqNum           = 0U;
    uint8_t  nextedIrq        = 0U;
    uint8_t  intMask          = 0U;
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

    test_pmic_print_unity_testcase_info(0xAB1B,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    if(J721E_LEO_PMICA_DEVICE == pmic_device_info)
    {
        /*
         * GPIO11 pin of PMIC-A is connected to 'H_SOC_PORz'
         * on J721EVM board causing hang, when programming fall interrupt.
         */
        pmic_testResultUpdate_ignore(0xAB1B,
                                         pmic_gpio_tests,
                                         PMIC_GPIO_NUM_OF_TESTCASES);
    }
    if(J721E_LEO_PMICB_DEVICE == pmic_device_info)
    {
        /*
         * GPIO11 pin of PMIC-B is connected to 'EN_3V3IO_LDSW'
         * on J721EVM board causing hang, when programming fall interrupt.
         */
        pmic_testResultUpdate_ignore(0xAB1B,
                                         pmic_gpio_tests,
                                         PMIC_GPIO_NUM_OF_TESTCASES);
    }

    if(J7VCL_LEO_PMICA_DEVICE == pmic_device_info)
    {
         pmic_testResultUpdate_ignore(0xAB1B,
                                         pmic_gpio_tests,
                                         PMIC_GPIO_NUM_OF_TESTCASES);
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(0xAB1B,
                                         pmic_gpio_tests,
                                         PMIC_GPIO_NUM_OF_TESTCASES);
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        intMask = PMIC_TPS6594X_IRQ_GPIO_11_INT_MASK_NUM;
        irqNum = PMIC_TPS6594X_GPIO11_INT;
    }

    /* Un Masking GPIO 11 FALL Interrupt */
    Pmic_irqGpioMaskIntr(pPmicCoreHandle,
                         intMask,
                         PMIC_IRQ_UNMASK,
                         PMIC_IRQ_GPIO_FALL_INT_TYPE);

    /* To clear the interrupts*/
    pmic_gpioTest_intClr();

    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioSetValue(pPmicCoreHandle, pin, pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioSetIntr(pPmicCoreHandle, pin , intrType, maskPol);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pinValue = PMIC_GPIO_LOW;
    pmicStatus = Pmic_gpioSetValue(pPmicCoreHandle, pin, pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    while(timeout--)
    {
        /* Delay added to avoid timeout */
        Osal_delay(1000);
        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        if((PMIC_ST_SUCCESS == pmicStatus) &&
           ((errStat.intStatus[irqNum/32U] & (1U << (irqNum % 32U))) != 0U))
        {
            while(irqNum != nextedIrq)
            {
                pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                                     &errStat,
                                                     &nextedIrq);
            }

            if(PMIC_ST_SUCCESS == pmicStatus)
            {
                /* clear the interrupt */
                pmicStatus = Pmic_irqClrErrStatus(pPmicCoreHandle,
                                                  irqNum);
                break;
            }
        }
    }

    if(0 > timeout)
    {
        pmicStatus = PMIC_ST_ERR_FAIL;
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmic_testResultUpdate_pass(0xAB1B,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
}

/*!
 * Below test cases are not tested because of HW limitation.
 * Added below test cases as sample for reference.
 */

/*!
 * \brief   Test to verify GPIO11 rise interrupt
 */
static void test_pmic_gpio11_testRise_interrupt(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    int8_t timeout            = 10U;
    uint8_t pin               = 11U;
    uint8_t pinValue          = PMIC_GPIO_LOW;
    uint8_t intrType          = PMIC_GPIO_RISE_INTERRUPT;
    uint8_t maskPol           = PMIC_GPIO_POL_HIGH;
    Pmic_IrqStatus_t errStat  = {0U};
    bool clearIRQ             = false;
    uint8_t  irqNum           = 0U;
    uint8_t  nextedIrq        = 0U;
    uint8_t  intMask          = 0U;
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

    test_pmic_print_unity_testcase_info(0xAB1A,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    if(J721E_LEO_PMICA_DEVICE == pmic_device_info)
    {
        /*
         * GPIO11 pin of PMIC-A is connected to 'H_SOC_PORz'
         * on J721EVM board causing hang, when programming rise interrupt.
         */
        pmic_testResultUpdate_ignore(0xAB1A,
                                         pmic_gpio_tests,
                                         PMIC_GPIO_NUM_OF_TESTCASES);
    }

    if(J721E_LEO_PMICB_DEVICE == pmic_device_info)
    {
        /*
         * GPIO11 pin of PMIC-B is connected to 'EN_3V3IO_LDSW'
         * on J721EVM board causing hang, when programming rise interrupt.
         */
        pmic_testResultUpdate_ignore(0xAB1A,
                                         pmic_gpio_tests,
                                         PMIC_GPIO_NUM_OF_TESTCASES);
    }

    if(J7VCL_LEO_PMICA_DEVICE == pmic_device_info)
    {
         pmic_testResultUpdate_ignore(0xAB1A,
                                         pmic_gpio_tests,
                                         PMIC_GPIO_NUM_OF_TESTCASES);
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(0xAB1A,
                                         pmic_gpio_tests,
                                         PMIC_GPIO_NUM_OF_TESTCASES);
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        intMask = PMIC_TPS6594X_IRQ_GPIO_11_INT_MASK_NUM;
        irqNum = PMIC_TPS6594X_GPIO11_INT;
    }

    /* Un Masking GPIO 11 RISE Interrupt */
    Pmic_irqGpioMaskIntr(pPmicCoreHandle,
                         intMask,
                         PMIC_IRQ_UNMASK,
                         PMIC_IRQ_GPIO_RISE_INT_TYPE);

    /* To clear the interrupts*/
    pmic_gpioTest_intClr();

    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioSetValue(pPmicCoreHandle, pin, pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioSetIntr(pPmicCoreHandle, pin , intrType, maskPol);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pinValue = PMIC_GPIO_HIGH;
    pmicStatus = Pmic_gpioSetValue(pPmicCoreHandle, pin, pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    while(timeout--)
    {
        /* Delay added to avoid timeout */
        Osal_delay(1000);
        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        if((PMIC_ST_SUCCESS == pmicStatus) &&
           ((errStat.intStatus[irqNum/32U] & (1U << (irqNum % 32U))) != 0U))
        {
            while(irqNum != nextedIrq)
            {
                pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                                     &errStat,
                                                     &nextedIrq);
            }

            if(PMIC_ST_SUCCESS == pmicStatus)
            {
                /* clear the interrupt */
                pmicStatus = Pmic_irqClrErrStatus(pPmicCoreHandle,
                                                  irqNum);
                break;
            }
        }
    }

    if(0 > timeout)
    {
        pmicStatus = PMIC_ST_ERR_FAIL;
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmic_testResultUpdate_pass(0xAB1A,
                               pmic_gpio_tests,
                               PMIC_GPIO_NUM_OF_TESTCASES);
}

/*!
 * Below test cases are not tested because of HW limitation.
 * Added below test cases as sample for reference.
 */

/*!
 * \brief   Test to verify GPIO WKUP 1
 */
static void test_pmic_gpioWkup1_test(void)
{
    int32_t status         = PMIC_ST_SUCCESS;
    uint8_t  pmicState     = 0U;
    Pmic_GpioCfg_t gpioCfg = {0U};

    gpioCfg.validParams      = PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT;
    gpioCfg.pinFunc          = PMIC_TPS6594X_GPIO_PINFUNC_WKUP1;

    status = Pmic_gpioSetConfiguration(pPmicCoreHandle,
                                       PMIC_TPS6594X_GPIO7_PIN,
                                       gpioCfg);
    pmicState = PMIC_FSM_STANBY_STATE;

    test_pmic_print_unity_testcase_info(0xAB21,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    TEST_IGNORE();

    Pmic_irqGpioMaskIntr(pPmicCoreHandle,
                         PMIC_TPS6594X_IRQ_GPIO_7_INT_MASK_NUM,
                         PMIC_IRQ_UNMASK,
                         PMIC_IRQ_GPIO_RISE_INT_TYPE);
    Pmic_irqGpioMaskIntr(pPmicCoreHandle,
                         PMIC_TPS6594X_IRQ_GPIO_7_INT_MASK_NUM,
                         PMIC_IRQ_MASK,
                         PMIC_IRQ_GPIO_FALL_INT_TYPE);

    /*Mask Nsleep1B and Nsleep2B signals for FSM state transitions*/
    status = Pmic_fsmSetNsleepSignalMask(pPmicCoreHandle,
                                         PMIC_NSLEEP1_SIGNAL,
                                         PMIC_NSLEEPX_MASK);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_fsmSetNsleepSignalMask(pPmicCoreHandle,
                                         PMIC_NSLEEP2_SIGNAL,
                                         PMIC_NSLEEPX_MASK);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    Osal_delay(10U);

    pmic_log("\r\n Input LOW to HIGH signal to GPIO7 to wakeup from low power mode\n");

    status = Pmic_fsmSetMissionState(pPmicCoreHandle, pmicState);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
}

/*!
 * Below test cases are not tested because of HW limitation.
 * Added below test cases as sample for reference.
 */

/*!
 * \brief   Test to verify GPIO WKUP 2
 */
static void test_pmic_gpioWkup2_test(void)
{
    int32_t status         = PMIC_ST_SUCCESS;
    uint8_t  pmicState     = 0U;
    Pmic_GpioCfg_t gpioCfg = {0U};

    gpioCfg.validParams      = PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT;
    gpioCfg.pinFunc          = PMIC_TPS6594X_GPIO_PINFUNC_WKUP2;

    status = Pmic_gpioSetConfiguration(pPmicCoreHandle,
                                       PMIC_TPS6594X_GPIO7_PIN,
                                       gpioCfg);
    pmicState = PMIC_FSM_STANBY_STATE;

    test_pmic_print_unity_testcase_info(0xAB10,
                                        pmic_gpio_tests,
                                        PMIC_GPIO_NUM_OF_TESTCASES);

    TEST_IGNORE();

    Pmic_irqGpioMaskIntr(pPmicCoreHandle,
                         PMIC_TPS6594X_IRQ_GPIO_7_INT_MASK_NUM,
                         PMIC_IRQ_UNMASK,
                         PMIC_IRQ_GPIO_RISE_INT_TYPE);
    Pmic_irqGpioMaskIntr(pPmicCoreHandle,
                         PMIC_TPS6594X_IRQ_GPIO_7_INT_MASK_NUM,
                         PMIC_IRQ_MASK,
                         PMIC_IRQ_GPIO_FALL_INT_TYPE);

    /*Mask Nsleep1B and Nsleep2B signals for FSM state transitions*/
    status = Pmic_fsmSetNsleepSignalMask(pPmicCoreHandle,
                                         PMIC_NSLEEP1_SIGNAL,
                                         PMIC_NSLEEPX_MASK);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_fsmSetNsleepSignalMask(pPmicCoreHandle,
                                         PMIC_NSLEEP2_SIGNAL,
                                         PMIC_NSLEEPX_MASK);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    Osal_delay(10U);

    pmic_log("\r\n Input LOW to HIGH signal to GPIO7 to wakeup from low power mode\n");

    status = Pmic_fsmSetMissionState(pPmicCoreHandle, pmicState);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
}
#endif

#if defined(UNITY_INCLUDE_CONFIG_V2_H) && \
    (defined(SOC_J721E) || defined(SOC_J7200))

/*!
 * \brief   Run gpio unity test cases
 */
static void test_pmic_run_testcases(void)
{
    pmic_log("\n\n%s(): %d: Begin Unity Test Cases...\n", __func__, __LINE__);
    UNITY_BEGIN();

    pmic_testResult_init(pmic_gpio_tests, PMIC_GPIO_NUM_OF_TESTCASES);

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
    RUN_TEST(test_pmic_enable_setCfgPrmValTest_pinPolarity);
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
    RUN_TEST(test_pmic_nPWRON_setCfgPrmValTest_pinFunc);
    RUN_TEST(test_pmic_gpio1_testFall_interrupt);
    RUN_TEST(test_pmic_gpio1_testRise_interrupt);
    RUN_TEST(test_pmic_gpio2_testFall_interrupt);
    RUN_TEST(test_pmic_gpio2_testRise_interrupt);
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
    RUN_TEST(test_pmic_gpio10_testFall_interrupt);
    RUN_TEST(test_pmic_gpio10_testRise_interrupt);
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
    RUN_TEST(test_pmic_gpio_irq_getMaskIntrStat);
    RUN_TEST(test_pmic_gpio_irq_getUnMaskIntrStat);
    RUN_TEST(test_pmic_gpio_getMaskIntrStatPrmValTst_handle);
    RUN_TEST(test_pmic_gpio_getMaskIntrStatPrmValTst_pFallIntrMaskStat);
    RUN_TEST(test_pmic_gpio_getMaskIntrStatPrmValTst_pRiseIntrMaskStat);
    RUN_TEST(test_pmic_gpio_irq_getMaskIntrStatPrmValTst_handle);
    RUN_TEST(test_pmic_gpio_irq_getMaskIntrStatPrmValTst_pMaskStatus);

    RUN_TEST(test_pmic_gpio_nPWRON_getCfgPrmValTest_pGpioCfg);
    RUN_TEST(test_pmic_gpio_nPWRON_setCfg_pinFunc);
    RUN_TEST(test_pmic_gpio_enable_setCfg_pinPolarity);
    RUN_TEST(test_pmic_gpio_setCfg_pullCtrl);
    RUN_TEST(test_pmic_gpio_setCfg_deglitchTime);
    RUN_TEST(test_pmic_gpio1_testDisable_interrupt);
    RUN_TEST(test_pmic_gpio_setCfgPrmValTest_pullCtrl);
    RUN_TEST(test_pmic_gpio_nPWRON_setCfg_pullCtrl);
    RUN_TEST(test_pmic_gpio_nPWRON_setCfgPrmValTest_handle);
    RUN_TEST(test_pmic_gpio_nPWRON_getCfgPrmValTest_handle);
    RUN_TEST(test_pmic_gpio_nPWRON_setCfg_deglitchTime);
    RUN_TEST(test_pmic_gpio_nPWRON_setCfgPrmValTest_deglitchEnable);
    RUN_TEST(test_pmic_gpio_nPWRON_setCfgPrmValTest_pullCtrl);
    RUN_TEST(test_pmic_gpio_setCfgPrmValTest_pin_minVal);
    RUN_TEST(test_pmic_gpio_nPWRON_setCfg_validParams);
    RUN_TEST(test_pmic_irqGetMaskIntrStatusPrmValTst_irqNum);
    RUN_TEST(test_pmic_irqClrErrStatusPrmValTst_handle);
    RUN_TEST(test_pmic_irqClrErrStatusPrmValTst_irqNum);
    RUN_TEST(test_pmic_irqGetErrStatusPrmValTst_handle);
    RUN_TEST(test_pmic_irqGetErrStatusPrmValTst_pErrStat);
    RUN_TEST(test_pmic_irqMaskIntrPrmValTst_handle);
    RUN_TEST(test_pmic_getNextErrorStatusPrmValTst_handle);
    RUN_TEST(test_pmic_getNextErrorStatusPrmValTst_pErrStat);
    RUN_TEST(test_pmic_getNextErrorStatusPrmValTst_pIrqNum);
    RUN_TEST(test_pmic_getNextErrorStatus_intStat0_intStat3);
    RUN_TEST(test_pmic_irqGpioMaskIntrPrmValTst_handle);
    RUN_TEST(test_pmic_irqGpioMaskIntrPrmValTst_irqGpioNum);
    RUN_TEST(test_pmic_irqGpioMaskIntrPrmValTst_gpioIntrType);
    RUN_TEST(test_pmic_irqGpioMaskIntrPrmValTst_irqGpioNum_hera);
    RUN_TEST(test_pmic_irqGetGpioMaskIntrPrmValTst_irqGpioNum);
    RUN_TEST(test_pmic_irqGetGpioMaskIntrPrmValTst_gpioIntrType);
    RUN_TEST(test_pmic_irqGetGpioMaskIntrPrmValTst_irqGpioNum_hera);
    RUN_TEST(test_pmic_irqGpioMaskIntr_deviceType_leo);
    RUN_TEST(test_pmic_gpio_coverageGaps);

    pmic_updateTestResults(pmic_gpio_tests, PMIC_GPIO_NUM_OF_TESTCASES);

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

    pmicConfigData.i2c1Speed            = PMIC_I2C_STANDARD_MODE;
    pmicConfigData.validParams         |= PMIC_CFG_I2C1_SPEED_VALID_SHIFT;

    pmicConfigData.i2c2Speed            = PMIC_I2C_STANDARD_MODE;
    pmicConfigData.validParams         |= PMIC_CFG_I2C2_SPEED_VALID_SHIFT;

    if(J721E_LEO_PMICA_DEVICE == pmic_device_info)
    {
        pmicConfigData.slaveAddr           = J721E_LEO_PMICA_SLAVE_ADDR;
        pmicConfigData.validParams        |= PMIC_CFG_SLAVEADDR_VALID_SHIFT;

        pmicConfigData.qaSlaveAddr         = J721E_LEO_PMICA_WDG_SLAVE_ADDR;
        pmicConfigData.validParams        |= PMIC_CFG_QASLAVEADDR_VALID_SHIFT;

        pmicConfigData.nvmSlaveAddr        = J721E_LEO_PMICA_PAGE1_SLAVE_ADDR;
        pmicConfigData.validParams        |= PMIC_CFG_NVMSLAVEADDR_VALID_SHIFT;
    }

    if(J7VCL_LEO_PMICA_DEVICE == pmic_device_info)
    {
        pmicConfigData.slaveAddr           = J7VCL_LEO_PMICA_SLAVE_ADDR;
        pmicConfigData.validParams        |= PMIC_CFG_SLAVEADDR_VALID_SHIFT;

        pmicConfigData.qaSlaveAddr         = J7VCL_LEO_PMICA_WDG_SLAVE_ADDR;
        pmicConfigData.validParams        |= PMIC_CFG_QASLAVEADDR_VALID_SHIFT;

        pmicConfigData.nvmSlaveAddr        = J7VCL_LEO_PMICA_PAGE1_SLAVE_ADDR;
        pmicConfigData.validParams        |= PMIC_CFG_NVMSLAVEADDR_VALID_SHIFT;
    }

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

    pmicConfigData.i2c1Speed            = PMIC_I2C_STANDARD_MODE;
    pmicConfigData.validParams         |= PMIC_CFG_I2C1_SPEED_VALID_SHIFT;

    pmicConfigData.slaveAddr           = J721E_LEO_PMICB_SLAVE_ADDR;
    pmicConfigData.validParams        |= PMIC_CFG_SLAVEADDR_VALID_SHIFT;

    pmicConfigData.qaSlaveAddr         = J721E_LEO_PMICB_WDG_SLAVE_ADDR;
    pmicConfigData.validParams        |= PMIC_CFG_QASLAVEADDR_VALID_SHIFT;

    pmicConfigData.nvmSlaveAddr        = J721E_LEO_PMICB_PAGE1_SLAVE_ADDR;
    pmicConfigData.validParams        |= PMIC_CFG_NVMSLAVEADDR_VALID_SHIFT;

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

    pmicConfigData.i2c1Speed            = PMIC_I2C_STANDARD_MODE;
    pmicConfigData.validParams         |= PMIC_CFG_I2C1_SPEED_VALID_SHIFT;

    pmicConfigData.slaveAddr           = J7VCL_HERA_PMIC_SLAVE_ADDR;
    pmicConfigData.validParams        |= PMIC_CFG_SLAVEADDR_VALID_SHIFT;

    pmicConfigData.qaSlaveAddr         = J7VCL_HERA_PMIC_WDG_SLAVE_ADDR;
    pmicConfigData.validParams        |= PMIC_CFG_QASLAVEADDR_VALID_SHIFT;

    pmicConfigData.nvmSlaveAddr        = J7VCL_HERA_PMIC_PAGE1_SLAVE_ADDR;
    pmicConfigData.validParams        |= PMIC_CFG_NVMSLAVEADDR_VALID_SHIFT;

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

static int32_t setup_pmic_interrupt(uint32_t board, bool enableCRC)
{
    int32_t status = PMIC_ST_SUCCESS;

    if(J721E_BOARD == board)
    {
        if(enableCRC == true)
        {
            if(PMIC_STATUS_CRC_INIT_VAL == gCrcTestFlag_J721E)
            {
                gCrcTestFlag_J721E = PMIC_CFG_TO_ENABLE_CRC;
            }
        }
        else
        {
            gCrcTestFlag_J721E = PMIC_STATUS_CRC_INIT_VAL;
        }

        pmic_device_info = J721E_LEO_PMICA_DEVICE;
        status = test_pmic_leo_pmicA_gpio_testApp();

        /* Deinit pmic handle */
        if((pPmicCoreHandle != NULL) && (PMIC_ST_SUCCESS == status))
        {
            test_pmic_appDeInit(pPmicCoreHandle);
        }

        if(PMIC_ST_SUCCESS == status)
        {
            pmic_device_info = J721E_LEO_PMICB_DEVICE;
            status = test_pmic_leo_pmicB_gpio_testApp();
            /* Deinit pmic handle */
            if((pPmicCoreHandle != NULL) && (PMIC_ST_SUCCESS == status))
            {
                test_pmic_appDeInit(pPmicCoreHandle);
            }
        }
    }
    else if(J7VCL_BOARD == board)
    {
        if(enableCRC == true)
        {
            if(PMIC_STATUS_CRC_INIT_VAL == gCrcTestFlag_J7VCL)
            {
                gCrcTestFlag_J7VCL = PMIC_CFG_TO_ENABLE_CRC;
            }
        }
        else
        {
            gCrcTestFlag_J7VCL = PMIC_STATUS_CRC_INIT_VAL;
        }

        pmic_device_info = J7VCL_LEO_PMICA_DEVICE;
        status = test_pmic_leo_pmicA_gpio_testApp();
        /* Deinit pmic handle */
        if((pPmicCoreHandle != NULL) && (PMIC_ST_SUCCESS == status))
        {
            test_pmic_appDeInit(pPmicCoreHandle);
        }

        if(PMIC_ST_SUCCESS == status)
        {
            pmic_device_info = J7VCL_HERA_PMICB_DEVICE;
            status = test_pmic_hera_gpio_testApp();
            /* Deinit pmic handle */
            if((pPmicCoreHandle != NULL) && (PMIC_ST_SUCCESS == status))
            {
                test_pmic_appDeInit(pPmicCoreHandle);
            }
        }
    }
    else
    {
        status = PMIC_ST_ERR_INV_DEVICE;
    }
    return status;
}

static const char pmicTestMenu[] =
{
    " \r\n ================================================================="
    " \r\n Test Menu:"
    " \r\n ================================================================="
    " \r\n 0: Automatic run for all board specific GPIO options"
    " \r\n 1: Manual run for GPIO options"
    " \r\n 2: quit"
    " \r\n"
    " \r\n Enter option: "
};

volatile static const char pmicTestAppMenu[] =
{
    " \r\n ================================================================="
    " \r\n Menu:"
    " \r\n ================================================================="
    " \r\n 0: Pmic Leo device(PMIC A on J721E EVM Using I2C Interface)"
    " \r\n 1: Pmic Leo device(PMIC B on J721E EVM Using I2C Interface)"
    " \r\n 2: Pmic Leo device(PMIC A on J7VCL EVM Using I2C Interface)"
    " \r\n 3: Pmic Hera device(PMIC B on J7VCL EVM Using I2C Interface)"
    " \r\n 4: Pmic Leo device(PMIC A on J721E EVM Using SPI Stub Functions)"
    " \r\n 5: Pmic Leo device(PMIC A on J7VCL EVM Using SPI Stub Functions)"
    " \r\n 6: Pmic Leo device(PMIC A on J721E EVM Manual Testcase)"
    " \r\n 7: Pmic Leo device(PMIC A on J7VCL EVM Manual Testcase)"
    " \r\n 8: Pmic Leo device(PMIC A on J721E EVM Manual Testcase with CRC Disable on PG2.0 Silicon revision)"
    " \r\n 9: Pmic Leo device(PMIC B on J721E EVM Manual Testcase with CRC Disable on PG2.0 Silicon revision)"
    " \r\n 10: Back to Test Menu"
    " \r\n"
    " \r\n Enter option: "
};

static void print_pmicTestAppManualTestMenu(uint32_t board)
{
    char board_name[10] = {0};

    if(J721E_BOARD == board)
    {
        strcpy(board_name, "J721E");
    }
    else if(J7VCL_BOARD == board)
    {
        strcpy(board_name, "J7VCL");
    }

    pmic_log(" \r\n =================================================================");
    pmic_log(" \r\n Manual Testcase Menu:");
    pmic_log(" \r\n =================================================================");
#if !(defined(SOC_J7200) && (defined(BUILD_MCU2_0) || defined(BUILD_MCU2_1)))
    pmic_log(" \r\n 0: Pmic Leo device(PMIC A on %s EVM Manual Testcase for GPIO-7 Fall Asynchronous Interrupt)", board_name);
    pmic_log(" \r\n 1: Pmic Leo device(PMIC A on %s EVM Manual Testcase for GPIO-7 Rise Asynchronous Interrupt)", board_name);
#endif
#if defined(SOC_J7200)
    pmic_log(" \r\n 2: Pmic Leo device(PMIC A on %s EVM Manual Testcase for CAN WKUP)", board_name);
#endif
    pmic_log(" \r\n 3: Pmic Leo device(PMIC A on %s EVM Manual Testcase for clearing all interrupts using IRQ API)", board_name);
    pmic_log(" \r\n 4: Back to Main Menu");
    pmic_log(" \r\n");
    pmic_log(" \r\n Enter option: ");
}

/*!
 * \brief   Run GPIO manual test cases
 */
static void test_pmic_run_testcases_manual(uint32_t board)
{
    int8_t menuOption = -1;

    while(1U)
    {
        print_pmicTestAppManualTestMenu(board);
        if(UART_scanFmt("%d", &menuOption) != 0U)
        {
            pmic_log("Read from UART Console failed\n");
            return;
        }

        switch(menuOption)
        {
#if !(defined(SOC_J7200) && (defined(BUILD_MCU2_0) || defined(BUILD_MCU2_1)))
            case 0U:
                RUN_TEST(test_pmic_gpio7_testFallAsynchronous_interrupt);
               break;
            case 1U:
                RUN_TEST(test_pmic_gpio7_testRiseAsynchronous_interrupt);
               break;
#endif
#if defined(SOC_J7200)
            case 2U:
                RUN_TEST(test_pmic_canWkup_test);
               break;
#endif
            case 3U :
                RUN_TEST(test_gpio4_fallInterrupt_clrAllIrqTest);
                break;
            case 4U:
                pmic_log(" \r\n Back to Test Menu options\n");
                return;
            default:
               pmic_log(" \r\n Invalid option... Try Again!!!\n");
               break;
        }
    }
}

static void test_pmic_gpio_testapp_run_options(int8_t option)
{
    int8_t num = -1;
    int8_t idx = 0;
#if defined(SOC_J721E)
    int8_t automatic_options[] = {0, 1, 4};
    int8_t  userInput = 0;
#elif defined(SOC_J7200)
    int8_t automatic_options[] = {2, 3, 5};
#endif

    while(1U)
    {
        if(idx >= (sizeof(automatic_options)/sizeof(automatic_options[0])))
        {
            pmic_printTestResult(pmic_gpio_tests, PMIC_GPIO_NUM_OF_TESTCASES);
        }
        pmic_log("%s", pmicTestAppMenu);
        if(option == PMIC_UT_AUTOMATE_OPTION)
        {
            if(idx < (sizeof(automatic_options)/sizeof(automatic_options[0])))
            {
                num = automatic_options[idx++];
            }
            else
            {
                num = 10;
            }
            pmic_log("%d\n", num);
        }
        else
        {
            if(UART_scanFmt("%d", &num) != 0U)
            {
                pmic_log("Read from UART Console failed\n");
                return;
            }
        }

        switch(num)
        {
            case 0U:
#if defined(SOC_J721E)
                if(PMIC_ST_SUCCESS == setup_pmic_interrupt(J721E_BOARD, true))
                {
                    pmic_device_info = J721E_LEO_PMICA_DEVICE;

                    /* GPIO Unity Test App wrapper Function for LEO PMIC-A */
                    if(PMIC_ST_SUCCESS == test_pmic_leo_pmicA_gpio_testApp())
                    {
                        /* Run gpio test cases for Leo PMIC-A */
                        test_pmic_run_testcases();
                    }
                    /* Deinit pmic handle */
                    if(pPmicCoreHandle != NULL)
                    {
                        test_pmic_appDeInit(pPmicCoreHandle);
                    }
                }
#else
                pmic_log("\nInvalid Board!!!\n");
#endif
                break;
            case 1U:
#if defined(SOC_J721E)
                if(PMIC_ST_SUCCESS == setup_pmic_interrupt(J721E_BOARD, true))
                {
                    pmic_device_info = J721E_LEO_PMICB_DEVICE;

                    /* GPIO Unity Test App wrapper Function for LEO PMIC-B */
                    if(PMIC_ST_SUCCESS == test_pmic_leo_pmicB_gpio_testApp())
                    {
                        /* Run gpio test cases for Leo PMIC-B */
                        test_pmic_run_testcases();
                    }
                    /* Deinit pmic handle */
                    if(pPmicCoreHandle != NULL)
                    {
                        test_pmic_appDeInit(pPmicCoreHandle);
                    }
                }
#else
                pmic_log("\nInvalid Board!!!\n");
#endif
                break;
            case 2U:
#if defined(SOC_J7200)
                if(PMIC_ST_SUCCESS == setup_pmic_interrupt(J7VCL_BOARD, true))
                {
                    pmic_device_info = J7VCL_LEO_PMICA_DEVICE;

                    /* GPIO Unity Test App wrapper Function for LEO PMIC-A */
                    if(PMIC_ST_SUCCESS == test_pmic_leo_pmicA_gpio_testApp())
                    {
                        /* Run gpio test cases for Leo PMIC-A */
                        test_pmic_run_testcases();
                    }
                    /* Deinit pmic handle */
                    if(pPmicCoreHandle != NULL)
                    {
                        test_pmic_appDeInit(pPmicCoreHandle);
                    }
                }
#else
                pmic_log("\nInvalid Board!!!\n");
#endif
                break;
            case 3U:
#if defined(SOC_J7200)
                if(PMIC_ST_SUCCESS == setup_pmic_interrupt(J7VCL_BOARD, true))
                {
                    pmic_device_info = J7VCL_HERA_PMICB_DEVICE;

                    /* GPIO Unity Test App wrapper Function for HERA PMIC */
                    if(PMIC_ST_SUCCESS == test_pmic_hera_gpio_testApp())
                    {
                        /* Run gpio test cases for Hera PMIC */
                        test_pmic_run_testcases();
                    }
                    /* Deinit pmic handle */
                    if(pPmicCoreHandle != NULL)
                    {
                        test_pmic_appDeInit(pPmicCoreHandle);
                    }
                }
#else
                pmic_log("\nInvalid Board!!!\n");
#endif
                break;
            case 4U:
#if defined(SOC_J721E)
                if(PMIC_ST_SUCCESS == setup_pmic_interrupt(J721E_BOARD, true))
                {
                     pmic_device_info = J721E_LEO_PMICA_DEVICE;

                    /* GPIO Unity Test App wrapper Function for LEO PMIC-A
                     * using SPI stub functions */
                    if(PMIC_ST_SUCCESS ==
                          test_pmic_leo_pmicA_spiStub_gpio_testApp())
                    {
                        /* Run gpio test cases for Leo PMIC-A */
                        test_pmic_run_testcases();
                    }
                    /* Deinit pmic handle */
                    if(pPmicCoreHandle != NULL)
                    {
                        test_pmic_appDeInit(pPmicCoreHandle);
                    }
                }
#else
                pmic_log("\nInvalid Board!!!\n");
#endif
                break;
            case 5U:
#if defined(SOC_J7200)
                if(PMIC_ST_SUCCESS == setup_pmic_interrupt(J7VCL_BOARD, true))
                {
                    pmic_device_info = J7VCL_LEO_PMICA_DEVICE;
                    /* GPIO Unity Test App wrapper Function for LEO PMIC-A
                     * using SPI stub functions */
                     if(PMIC_ST_SUCCESS ==
                           test_pmic_leo_pmicA_spiStub_gpio_testApp())
                    {
                        /* Run gpio test cases for Leo PMIC-A */
                        test_pmic_run_testcases();
                    }
                    /* Deinit pmic handle */
                    if(pPmicCoreHandle != NULL)
                    {
                        test_pmic_appDeInit(pPmicCoreHandle);
                    }
                }
#else
                pmic_log("\nInvalid Board!!!\n");
#endif
                break;
            case 6U:
#if defined(SOC_J721E)
                if(PMIC_ST_SUCCESS == setup_pmic_interrupt(J721E_BOARD, true))
                {
                     pmic_device_info = J721E_LEO_PMICA_DEVICE;

                     /* GPIO Manual Tests App wrapper Func for LEO PMIC-A */
                     if(PMIC_ST_SUCCESS ==
                                        test_pmic_leo_pmicA_gpio_testApp())
                     {
                         /* Run GPIO manual test cases */
                         test_pmic_run_testcases_manual(J721E_BOARD);
                     }
                     /* Deinit pmic handle */
                     if(pPmicCoreHandle != NULL)
                     {
                        test_pmic_appDeInit(pPmicCoreHandle);
                     }
                }
#else
                pmic_log("\nInvalid Board!!!\n");
#endif
                return;
            case 7U:
#if defined(SOC_J7200)
                if(PMIC_ST_SUCCESS == setup_pmic_interrupt(J7VCL_BOARD, true))
                {
                    pmic_device_info = J7VCL_LEO_PMICA_DEVICE;

                    /* GPIO Unity Test App wrapper Function for HERA PMIC */
                    if(PMIC_ST_SUCCESS == test_pmic_leo_pmicA_gpio_testApp())
                    {
                        /* Run GPIO manual test cases */
                        test_pmic_run_testcases_manual(J7VCL_BOARD);
                    }
                    /* Deinit pmic handle */
                    if(pPmicCoreHandle != NULL)
                    {
                        test_pmic_appDeInit(pPmicCoreHandle);
                    }
                }
#else
                pmic_log("\nInvalid Board!!!\n");
#endif
                return;
            case 8U:
#if defined(SOC_J721E)
                pmic_log("\r\n Run the test only on Leo PMIC-A PG2.0 after Board Power cycle !!!\n");
                pmic_log("\r\n Enter 1 to continue");
                UART_scanFmt("%d", &userInput);

                if(PMIC_ST_SUCCESS == setup_pmic_interrupt(J721E_BOARD, false))
                {
                    pmic_device_info = J721E_LEO_PMICA_DEVICE;

                    /* GPIO Unity Test App wrapper Function for LEO PMIC-A */
                    if(PMIC_ST_SUCCESS == test_pmic_leo_pmicA_gpio_testApp())
                    {
                        /* Run gpio test cases for Leo PMIC-A */
                        test_pmic_run_testcases();
                    }
                    /* Deinit pmic handle */
                    if(pPmicCoreHandle != NULL)
                    {
                        test_pmic_appDeInit(pPmicCoreHandle);
                    }
                }
#else
                pmic_log("\nInvalid Board!!!\n");
#endif
                break;
            case 9U:
#if defined(SOC_J721E)
                pmic_log("\r\n Run the test only on Leo PMIC-B PG2.0 after Board Power cycle !!!\n");
                pmic_log("\r\n Enter 1 to continue");
                UART_scanFmt("%d", &userInput);
                if(PMIC_ST_SUCCESS == setup_pmic_interrupt(J721E_BOARD, false))
                {
                    pmic_device_info = J721E_LEO_PMICB_DEVICE;

                    /* GPIO Unity Test App wrapper Function for LEO PMIC-B */
                    if(PMIC_ST_SUCCESS == test_pmic_leo_pmicB_gpio_testApp())
                    {
                        /* Run gpio test cases for Leo PMIC-B */
                        test_pmic_run_testcases();
                    }
                    /* Deinit pmic handle */
                    if(pPmicCoreHandle != NULL)
                    {
                        test_pmic_appDeInit(pPmicCoreHandle);
                    }
                }
#else
                pmic_log("\nInvalid Board!!!\n");
#endif
                break;

            case 10U:
                pmic_log(" \r\n Back to Test Menu options\n");
                return;
            default:
               pmic_log(" \r\n Invalid option... Try Again!!!\n");
               break;
        }
    }
}

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

    int8_t option = -1;

    while(1U)
    {
        pmic_log("%s", pmicTestMenu);
        if(UART_scanFmt("%d", &option) != 0U)
        {
            pmic_log("Read from UART Console failed\n");
            return;
        }

        switch(option)
        {
            case PMIC_UT_AUTOMATE_OPTION:
                test_pmic_gpio_testapp_run_options(PMIC_UT_AUTOMATE_OPTION);
               break;
            case PMIC_UT_MANUAL_OPTION:
                test_pmic_gpio_testapp_run_options(PMIC_UT_MANUAL_OPTION);
               break;
            case 2U:
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
 * \brief   PMIC Application Callback Function
 */
void AppPmicCallbackFxn(void)
{
    int32_t status           = PMIC_ST_SUCCESS;
    Pmic_IrqStatus_t errStat = {0U};
    uint8_t irqNum           = 0U;

    status = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, false);

    if(PMIC_ST_SUCCESS == status)
    {
        status = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                         &errStat,
                                         &irqNum);
        if((PMIC_ST_SUCCESS == status) &&
           (PMIC_TPS6594X_GPIO7_INT == irqNum))
        {
            pmic_intr_triggered = 1U;
            /* clear the interrupt */
            status = Pmic_irqClrErrStatus(pPmicCoreHandle,
                                          PMIC_TPS6594X_GPIO7_INT);
        }
    }
}

/*!
 * \brief   TI RTOS specific GPIO TEST APP main Function
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code.
 *          For valid values see \ref Pmic_ErrorCodes
 */
int main()
{
    Board_initUART();

    /* GPIO Configuration
     * This API is required for Asynchronous Interrupts only
     */
    App_initGPIO(AppPmicCallbackFxn);

    pmic_print_banner("PMIC GPIO Unity Test Application");
#if defined(UNITY_INCLUDE_CONFIG_V2_H) && \
    (defined(SOC_J721E)             || \
     defined(SOC_J7200))
    test_pmic_gpio_testapp_runner();
#endif
}
