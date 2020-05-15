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

/*!
 * \brief   configure a gpio pin as NSLEEP1 function
 *
 * \param   pmicHandle    [IN]     PMIC Interface Handle.
 *
 * \retval  PMIC_UT_SUCCESS in case of success or
 *          PMIC_UT_FAILURE in case of failure.
 */
static bool test_pmic_gpio_setCfgGpioPin_nSLEEP1(void *pmicHandle)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_CoreHandle_t handle  = *(Pmic_CoreHandle_t *)pmicHandle;
    Pmic_GpioCfg_t gpioCfg_rd = {PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,};
    int pin                   = 0U;
    uint8_t pins[]           = { 1U, 2U, 3U, 4U, 5U, 6U, 7U, 8U, 9U, 10U, 11U};
    Pmic_GpioCfg_t gpioCfg   =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,
        PMIC_GPIO_HIGH,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_GPIO_PINFUNC_NSLEEP1,
        PMIC_GPIO_HIGH
    };

    for(pin = 0U; pin < sizeof(pins)/sizeof(pins[0U]); pin++)
    {
        pmicStatus = Pmic_gpioSetConfiguration(&handle, pins[pin], &gpioCfg);
        if(PMIC_ST_SUCCESS != pmicStatus)
        {
            pmic_log("Failed %s with status: %d\n\t", __func__, pmicStatus);
            return PMIC_UT_FAILURE;
        }

        Osal_delay(50U);
        pmicStatus = Pmic_gpioGetConfiguration(&handle,
                                               pins[pin],
                                               &gpioCfg_rd);
        if(PMIC_ST_SUCCESS != pmicStatus)
        {
            pmic_log("Failed %s with status: %d\n\t", __func__, pmicStatus);
            return PMIC_UT_FAILURE;
        }

        if(gpioCfg.pinFunc != gpioCfg_rd.pinFunc)
        {
            pmic_log("Failed %s for GPIO-%d\n\t",__func__, pin);
            return PMIC_UT_FAILURE;
        }
    }

    return PMIC_UT_SUCCESS;
}

/*!
 * \brief   configure a gpio pin as NSLEEP2 function
 *
 * \param   pmicHandle    [IN]     PMIC Interface Handle.
 *
 * \retval  PMIC_UT_SUCCESS in case of success or
 *          PMIC_UT_FAILURE in case of failure.
 */
static bool test_pmic_gpio_setCfgGpioPin_nSLEEP2(void *pmicHandle)
{
    int32_t pmicStatus       = PMIC_ST_SUCCESS;
    Pmic_CoreHandle_t handle = *(Pmic_CoreHandle_t *)pmicHandle;
    Pmic_GpioCfg_t gpioCfg_rd = {PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,};
    int pin                  = 0U;
    uint8_t pins[]           = { 1U, 2U, 3U, 4U, 5U, 6U, 7U, 8U, 9U, 10U, 11U};
    Pmic_GpioCfg_t gpioCfg   =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,
        PMIC_GPIO_HIGH,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_GPIO_PINFUNC_NSLEEP2,
        PMIC_GPIO_HIGH
    };

    for(pin = 0U; pin < sizeof(pins)/sizeof(pins[0U]); pin++)
    {
        pmicStatus = Pmic_gpioSetConfiguration(&handle, pins[pin], &gpioCfg);
        if(PMIC_ST_SUCCESS != pmicStatus)
        {
            pmic_log("Failed %s with status: %d\n\t", __func__, pmicStatus);
            return PMIC_UT_FAILURE;
        }

        Osal_delay(50U);
        pmicStatus = Pmic_gpioGetConfiguration(&handle,
                                               pins[pin],
                                               &gpioCfg_rd);
        if(PMIC_ST_SUCCESS != pmicStatus)
        {
            pmic_log("Failed %s with status: %d\n\t", __func__, pmicStatus);
            return PMIC_UT_FAILURE;
        }

        if(gpioCfg.pinFunc != gpioCfg_rd.pinFunc)
        {
            pmic_log("Failed %s\n\t",__func__);
            return PMIC_UT_FAILURE;
        }
    }

    return PMIC_UT_SUCCESS;
}

/*!
 * \brief   configure gpio pin1 as NRSTOUT_SOC function
 *
 * \param   pmicHandle    [IN]     PMIC Interface Handle.
 *
 * \retval  PMIC_UT_SUCCESS in case of success or
 *          PMIC_UT_FAILURE in case of failure.
 */
static bool test_pmic_gpio_setCfgGpioPin1_nRstOut_soc(void *pmicHandle)
{
    int32_t pmicStatus       = PMIC_ST_SUCCESS;
    Pmic_CoreHandle_t handle = *(Pmic_CoreHandle_t *)pmicHandle;
    int pin                  = 1U;
    Pmic_GpioCfg_t gpioCfg_rd = {PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,};
    Pmic_GpioCfg_t gpioCfg   =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,
        PMIC_GPIO_HIGH,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_GPIO_PINFUNC_NRSTOUT_SOC,
        PMIC_GPIO_HIGH
    };

    pmicStatus = Pmic_gpioSetConfiguration(&handle, pin, &gpioCfg);
    if(PMIC_ST_SUCCESS != pmicStatus)
    {
        pmic_log("Failed %s with status: %d\n\t", __func__, pmicStatus);
        return PMIC_UT_FAILURE;
    }

    Osal_delay(50U);
    pmicStatus = Pmic_gpioGetConfiguration(&handle, pin, &gpioCfg_rd);
    if(PMIC_ST_SUCCESS != pmicStatus)
    {
        pmic_log("Failed %s with status: %d\n\t", __func__, pmicStatus);
        return PMIC_UT_FAILURE;
    }

    if(gpioCfg.pinFunc != gpioCfg_rd.pinFunc)
    {
        pmic_log("Failed %s\n\t",__func__);
        return PMIC_UT_FAILURE;
    }

    return PMIC_UT_SUCCESS;
}

/*!
 * \brief   configure gpio pin11 as NRSTOUT_SOC function
 *
 * \param   pmicHandle    [IN]     PMIC Interface Handle.
 *
 * \retval  PMIC_UT_SUCCESS in case of success or
 *          PMIC_UT_FAILURE in case of failure.
 */
static bool test_pmic_gpio_setCfgGpioPin11_nRstOut_soc(void *pmicHandle)
{
    int32_t pmicStatus       = PMIC_ST_SUCCESS;
    Pmic_CoreHandle_t handle = *(Pmic_CoreHandle_t *)pmicHandle;
    int pin                  = 11U;
    Pmic_GpioCfg_t gpioCfg_rd = {PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,};
    Pmic_GpioCfg_t gpioCfg   =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,
        PMIC_GPIO_HIGH,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_GPIO_PINFUNC_NRSTOUT_SOC,
        PMIC_GPIO_HIGH
    };

    pmicStatus = Pmic_gpioSetConfiguration(&handle, pin, &gpioCfg);
    if(PMIC_ST_SUCCESS != pmicStatus)
    {
        pmic_log("Failed %s with status: %d\n\t", __func__, pmicStatus);
        return PMIC_UT_FAILURE;
    }

    Osal_delay(50U);
    pmicStatus = Pmic_gpioGetConfiguration(&handle, pin, &gpioCfg_rd);
    if(PMIC_ST_SUCCESS != pmicStatus)
    {
        pmic_log("Failed %s with status: %d\n\t", __func__, pmicStatus);
        return PMIC_UT_FAILURE;
    }

    if(gpioCfg.pinFunc != gpioCfg_rd.pinFunc)
    {
        pmic_log("Failed %s\n\t",__func__);
        return PMIC_UT_FAILURE;
    }

    return PMIC_UT_SUCCESS;
}

/*!
 * \brief   configure a gpio pin as WAKEUP1 function
 *
 * \param   pmicHandle    [IN]     PMIC Interface Handle.
 *
 * \retval  PMIC_UT_SUCCESS in case of success or
 *          PMIC_UT_FAILURE in case of failure.
 */
static bool test_pmic_gpio_setCfgGpioPin_wakeup1(void *pmicHandle)
{
    int32_t pmicStatus       = PMIC_ST_SUCCESS;
    Pmic_CoreHandle_t handle = *(Pmic_CoreHandle_t *)pmicHandle;
    Pmic_GpioCfg_t gpioCfg_rd = {PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,};
    int pin                  = 0U;
    uint8_t pins[]           = { 1U, 2U, 3U, 4U, 5U, 6U, 7U, 8U, 9U, 10U, 11U};
    Pmic_GpioCfg_t gpioCfg   =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,
        PMIC_GPIO_HIGH,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_GPIO_PINFUNC_WKUP1,
        PMIC_GPIO_HIGH
    };

    for(pin = 0U; pin < sizeof(pins)/sizeof(pins[0U]); pin++)
    {
        pmicStatus = Pmic_gpioSetConfiguration(&handle, pins[pin], &gpioCfg);
        if(PMIC_ST_SUCCESS != pmicStatus)
        {
            pmic_log("Failed %s with status: %d\n\t", __func__, pmicStatus);
            return PMIC_UT_FAILURE;
        }

        Osal_delay(50U);
        pmicStatus = Pmic_gpioGetConfiguration(&handle,
                                               pins[pin],
                                               &gpioCfg_rd);
        if(PMIC_ST_SUCCESS != pmicStatus)
        {
            pmic_log("Failed %s with status: %d\n\t", __func__, pmicStatus);
            return PMIC_UT_FAILURE;
        }

        if(gpioCfg.pinFunc != gpioCfg_rd.pinFunc)
        {
            pmic_log("Failed %s\n\t",__func__);
            return PMIC_UT_FAILURE;
        }
    }

    return PMIC_UT_SUCCESS;
}

/*!
 * \brief   configure a gpio pin as WAKEUP2 function
 *
 * \param   pmicHandle    [IN]     PMIC Interface Handle.
 *
 * \retval  PMIC_UT_SUCCESS in case of success or
 *          PMIC_UT_FAILURE in case of failure.
 */
static bool test_pmic_gpio_setCfgGpioPin_wakeup2(void *pmicHandle)
{
    int32_t pmicStatus       = PMIC_ST_SUCCESS;
    Pmic_CoreHandle_t handle = *(Pmic_CoreHandle_t *)pmicHandle;
    Pmic_GpioCfg_t gpioCfg_rd = {PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,};
    int pin                  = 0U;
    uint8_t pins[]           = { 1U, 2U, 3U, 4U, 5U, 6U, 7U, 8U, 9U, 10U, 11U};
    Pmic_GpioCfg_t gpioCfg   =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,
        PMIC_GPIO_HIGH,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_GPIO_PINFUNC_WKUP2,
        PMIC_GPIO_HIGH
    };

    for(pin = 0U; pin < sizeof(pins)/sizeof(pins[0U]); pin++)
    {
        pmicStatus = Pmic_gpioSetConfiguration(&handle, pins[pin], &gpioCfg);
        if(PMIC_ST_SUCCESS != pmicStatus)
        {
            pmic_log("Failed %s with status: %d\n\t", __func__, pmicStatus);
            return PMIC_UT_FAILURE;
        }

        Osal_delay(50U);
        pmicStatus = Pmic_gpioGetConfiguration(&handle,
                                               pins[pin],
                                               &gpioCfg_rd);
        if(PMIC_ST_SUCCESS != pmicStatus)
        {
            pmic_log("Failed %s with status: %d\n\t", __func__, pmicStatus);
            return PMIC_UT_FAILURE;
        }

        if(gpioCfg.pinFunc != gpioCfg_rd.pinFunc)
        {
            pmic_log("Failed %s\n\t",__func__);
            return PMIC_UT_FAILURE;
        }
    }

    return PMIC_UT_SUCCESS;
}

/*!
 * \brief   configure a gpio pin as general purpose I/O function
 *
 * \param   pmicHandle    [IN]     PMIC Interface Handle.
 *
 * \retval  PMIC_UT_SUCCESS in case of success or
 *          PMIC_UT_FAILURE in case of failure.
 */
static bool test_pmic_gpio_setCfgGpioPin_gpio(void *pmicHandle)
{
    int32_t pmicStatus       = PMIC_ST_SUCCESS;
    Pmic_CoreHandle_t handle = *(Pmic_CoreHandle_t *)pmicHandle;
    Pmic_GpioCfg_t gpioCfg_rd = {PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,};
    int pin                  = 0U;
    uint8_t pins[]           = { 1U, 2U, 3U, 4U, 5U, 6U, 7U, 8U, 9U, 10U,
    /* On J721, PMIC-A GPIO11 connected to H_SOC_PORz, which resets entire SOC
     * because of this reason disabling GPIOCFG test on PMIC GPIO11 pin.
     */
                     };
    Pmic_GpioCfg_t gpioCfg   =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,
        PMIC_GPIO_HIGH,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_GPIO_PINFUNC_GPIO,
        PMIC_GPIO_HIGH
    };

    for(pin = 0U; pin < sizeof(pins)/sizeof(pins[0U]); pin++)
    {
        pmicStatus = Pmic_gpioSetConfiguration(&handle, pins[pin], &gpioCfg);
        if(PMIC_ST_SUCCESS != pmicStatus)
        {
            pmic_log("Failed %s with status: %d\n\t", __func__, pmicStatus);
            return PMIC_UT_FAILURE;
        }

        Osal_delay(50U);
        pmicStatus = Pmic_gpioGetConfiguration(&handle,
                                               pins[pin],
                                               &gpioCfg_rd);
        if(PMIC_ST_SUCCESS != pmicStatus)
        {
            pmic_log("Failed %s with status: %d\n\t", __func__, pmicStatus);
            return PMIC_UT_FAILURE;
        }

        if(gpioCfg.pinFunc != gpioCfg_rd.pinFunc)
        {
            pmic_log("Failed %s\n\t",__func__);
            return PMIC_UT_FAILURE;
        }
    }

    return PMIC_UT_SUCCESS;
}

/*!
 * \brief   configure gpio pin1 as I2C2 SCLK function
 *
 * \param   pmicHandle    [IN]     PMIC Interface Handle.
 *
 * \retval  PMIC_UT_SUCCESS in case of success or
 *          PMIC_UT_FAILURE in case of failure.
 */
static bool test_pmic_gpio_setCfgGpioPin1_i2c2_sclk(void *pmicHandle)
{
    int32_t pmicStatus       = PMIC_ST_SUCCESS;
    Pmic_CoreHandle_t handle = *(Pmic_CoreHandle_t *)pmicHandle;
    int pin                  = 1U;
    Pmic_GpioCfg_t gpioCfg_rd = {PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,};
    Pmic_GpioCfg_t gpioCfg   =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,
        PMIC_GPIO_HIGH,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_GPIO_PINFUNC_SCL_I2C2_CS_SPI,
        PMIC_GPIO_HIGH
    };

    pmicStatus = Pmic_gpioSetConfiguration(&handle, pin, &gpioCfg);
    if(PMIC_ST_SUCCESS != pmicStatus)
    {
        pmic_log("Failed %s with status: %d\n\t", __func__, pmicStatus);
        return PMIC_UT_FAILURE;
    }

    Osal_delay(50U);
    pmicStatus = Pmic_gpioGetConfiguration(&handle, pin, &gpioCfg_rd);
    if(PMIC_ST_SUCCESS != pmicStatus)
    {
        pmic_log("Failed %s with status: %d\n\t", __func__, pmicStatus);
        return PMIC_UT_FAILURE;
    }

    if(gpioCfg.pinFunc != gpioCfg_rd.pinFunc)
    {
        pmic_log("Failed %s\n\t",__func__);
        return PMIC_UT_FAILURE;
    }

    return PMIC_UT_SUCCESS;
}

/*!
 * \brief   configure gpio pin2 as I2C2 SDA function
 *
 * \param   pmicHandle    [IN]     PMIC Interface Handle.
 *
 * \retval  PMIC_UT_SUCCESS in case of success or
 *          PMIC_UT_FAILURE in case of failure.
 */
static bool test_pmic_gpio_setCfgGpioPin2_i2c2_sda(void *pmicHandle)
{
    int32_t pmicStatus       = PMIC_ST_SUCCESS;
    Pmic_CoreHandle_t handle = *(Pmic_CoreHandle_t *)pmicHandle;
    int pin                  = 2U;
    Pmic_GpioCfg_t gpioCfg_rd = {PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,};
    Pmic_GpioCfg_t gpioCfg   =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,
        PMIC_GPIO_HIGH,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_GPIO_PINFUNC_SDA_I2C2_SDO_SPI,
        PMIC_GPIO_HIGH
    };

    pmicStatus = Pmic_gpioSetConfiguration(&handle, pin, &gpioCfg);
    if(PMIC_ST_SUCCESS != pmicStatus)
    {
        pmic_log("Failed %s with status: %d\n\t", __func__, pmicStatus);
        return PMIC_UT_FAILURE;
    }

    Osal_delay(50U);
    pmicStatus = Pmic_gpioGetConfiguration(&handle, pin, &gpioCfg_rd);
    if(PMIC_ST_SUCCESS != pmicStatus)
    {
        pmic_log("Failed %s with status: %d\n\t", __func__, pmicStatus);
        return PMIC_UT_FAILURE;
    }

    if(gpioCfg.pinFunc != gpioCfg_rd.pinFunc)
    {
        pmic_log("Failed %s\n\t",__func__);
        return PMIC_UT_FAILURE;
    }

    return PMIC_UT_SUCCESS;
}

/*!
 * \brief   configure gpio pin1 as SPI CS function
 *
 * \param   pmicHandle    [IN]     PMIC Interface Handle.
 *
 * \retval  PMIC_UT_SUCCESS in case of success or
 *          PMIC_UT_FAILURE in case of failure.
 */
static bool test_pmic_gpio_setCfgGpioPin1_spi_cs(void *pmicHandle)
{
    int32_t pmicStatus       = PMIC_ST_SUCCESS;
    Pmic_CoreHandle_t handle = *(Pmic_CoreHandle_t *)pmicHandle;
    int pin                  = 1U;
    Pmic_GpioCfg_t gpioCfg_rd = {PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,};
    Pmic_GpioCfg_t gpioCfg   =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,
        PMIC_GPIO_HIGH,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_GPIO_PINFUNC_SCL_I2C2_CS_SPI,
        PMIC_GPIO_HIGH
    };

    pmicStatus=Pmic_gpioSetConfiguration(&handle, pin, &gpioCfg);
    if(PMIC_ST_SUCCESS != pmicStatus)
    {
        pmic_log("Failed %s with status: %d\n\t", __func__, pmicStatus);
        return PMIC_UT_FAILURE;
    }

    Osal_delay(50U);
    pmicStatus = Pmic_gpioGetConfiguration(&handle, pin, &gpioCfg_rd);
    if(PMIC_ST_SUCCESS != pmicStatus)
    {
        pmic_log("Failed %s with status: %d\n\t", __func__, pmicStatus);
        return PMIC_UT_FAILURE;
    }

    if(gpioCfg.pinFunc != gpioCfg_rd.pinFunc)
    {
        pmic_log("Failed %s\n\t",__func__);
        return PMIC_UT_FAILURE;
    }

    return PMIC_UT_SUCCESS;
}

/*!
 * \brief   configure gpio pin2 as SPI SDO function
 *
 * \param   pmicHandle    [IN]     PMIC Interface Handle.
 *
 * \retval  PMIC_UT_SUCCESS in case of success or
 *          PMIC_UT_FAILURE in case of failure.
 */
static bool test_pmic_gpio_setCfgGpioPin2_spi_sdo(void *pmicHandle)
{
    int32_t pmicStatus       = PMIC_ST_SUCCESS;
    Pmic_CoreHandle_t handle = *(Pmic_CoreHandle_t *)pmicHandle;
    int pin                  = 2U;
    Pmic_GpioCfg_t gpioCfg_rd = {PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,};
    Pmic_GpioCfg_t gpioCfg   =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,
        PMIC_GPIO_HIGH,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_GPIO_PINFUNC_SDA_I2C2_SDO_SPI,
        PMIC_GPIO_HIGH
    };

    pmicStatus = Pmic_gpioSetConfiguration(&handle, pin, &gpioCfg);
    if(PMIC_ST_SUCCESS != pmicStatus)
    {
        pmic_log("Failed %s with status: %d\n\t", __func__, pmicStatus);
        return PMIC_UT_FAILURE;
    }

    Osal_delay(50U);
    pmicStatus = Pmic_gpioGetConfiguration(&handle, pin, &gpioCfg_rd);
    if(PMIC_ST_SUCCESS != pmicStatus)
    {
        pmic_log("Failed %s with status: %d\n\t", __func__, pmicStatus);
        return PMIC_UT_FAILURE;
    }

    if(gpioCfg.pinFunc != gpioCfg_rd.pinFunc)
    {
        pmic_log("Failed %s\n\t",__func__);
        return PMIC_UT_FAILURE;
    }

    return PMIC_UT_SUCCESS;
}

/*!
 * \brief   configure a gpio pin as Watchdog Trigger function
 *
 * \param   pmicHandle    [IN]     PMIC Interface Handle.
 *
 * \retval  PMIC_UT_SUCCESS in case of success or
 *          PMIC_UT_FAILURE in case of failure.
 */
static bool test_pmic_gpio_setCfgGpioPin_wdt(void *pmicHandle)
{
    int32_t pmicStatus       = PMIC_ST_SUCCESS;
    Pmic_CoreHandle_t handle = *(Pmic_CoreHandle_t *)pmicHandle;
    Pmic_GpioCfg_t gpioCfg_rd = {PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,};
    int pin                  = 0U;
    uint8_t pins[]           = {2U, 11U};
    Pmic_GpioCfg_t gpioCfg   =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,
        PMIC_GPIO_HIGH,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_GPIO_PINFUNC_TRIG_WDOG,
        PMIC_GPIO_HIGH
    };

    for(pin = 0U; pin < sizeof(pins)/sizeof(pins[0U]); pin++)
    {
        pmicStatus = Pmic_gpioSetConfiguration(&handle, pins[pin], &gpioCfg);
        if(PMIC_ST_SUCCESS != pmicStatus)
        {
            pmic_log("Failed %s with status: %d\n\t", __func__, pmicStatus);
            return PMIC_UT_FAILURE;
        }

        Osal_delay(50U);
        pmicStatus = Pmic_gpioGetConfiguration(&handle,
                                               pins[pin],
                                               &gpioCfg_rd);
        if(PMIC_ST_SUCCESS != pmicStatus)
        {
            pmic_log("Failed %s with status: %d\n\t", __func__, pmicStatus);
            return PMIC_UT_FAILURE;
        }

        if(gpioCfg.pinFunc != gpioCfg_rd.pinFunc)
        {
            pmic_log("Failed %s\n\t",__func__);
            return PMIC_UT_FAILURE;
        }
    }

    return PMIC_UT_SUCCESS;
}

/*!
 * \brief   configure gpio pin3 as ESM Error pin for SOC
 *
 * \param   pmicHandle    [IN]     PMIC Interface Handle.
 *
 * \retval  PMIC_UT_SUCCESS in case of success or
 *          PMIC_UT_FAILURE in case of failure.
 */
static bool test_pmic_gpio_setCfgGpioPin3_esm_soc(void *pmicHandle)
{
    int32_t pmicStatus       = PMIC_ST_SUCCESS;
    Pmic_CoreHandle_t handle = *(Pmic_CoreHandle_t *)pmicHandle;
    int pin                  = 3U;
    Pmic_GpioCfg_t gpioCfg_rd = {PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,};
    Pmic_GpioCfg_t gpioCfg   =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,
        PMIC_GPIO_HIGH,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_GPIO_PINFUNC_NERR_SOC,
        PMIC_GPIO_HIGH
    };

    pmicStatus=Pmic_gpioSetConfiguration(&handle, pin, &gpioCfg);
    if(PMIC_ST_SUCCESS != pmicStatus)
    {
        pmic_log("Failed %s with status: %d\n\t", __func__, pmicStatus);
        return PMIC_UT_FAILURE;
    }

    Osal_delay(50U);
    pmicStatus = Pmic_gpioGetConfiguration(&handle, pin, &gpioCfg_rd);
    if(PMIC_ST_SUCCESS != pmicStatus)
    {
        pmic_log("Failed %s with status: %d\n\t", __func__, pmicStatus);
        return PMIC_UT_FAILURE;
    }

    if(gpioCfg.pinFunc != gpioCfg_rd.pinFunc)
    {
        pmic_log("Failed %s\n\t",__func__);
        return PMIC_UT_FAILURE;
    }

    return PMIC_UT_SUCCESS;
}

/*!
 * \brief   configure gpio pin7 as ESM Error pin for MCU
 *
 * \param   pmicHandle    [IN]     PMIC Interface Handle.
 *
 * \retval  PMIC_UT_SUCCESS in case of success or
 *          PMIC_UT_FAILURE in case of failure.
 */
static bool test_pmic_gpio_setCfgGpioPin7_esm_mcu(void *pmicHandle)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_CoreHandle_t handle  = *(Pmic_CoreHandle_t *)pmicHandle;
    int pin                   = 7U;
    Pmic_GpioCfg_t gpioCfg_rd = {PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,};
    Pmic_GpioCfg_t gpioCfg    =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,
        PMIC_GPIO_HIGH,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_GPIO_PINFUNC_NERR_MCU,
        PMIC_GPIO_HIGH
    };

    pmicStatus = Pmic_gpioSetConfiguration(&handle, pin, &gpioCfg);
    if(PMIC_ST_SUCCESS != pmicStatus)
    {
        pmic_log("Failed %s with status: %d\n\t", __func__, pmicStatus);
        return PMIC_UT_FAILURE;
    }

    Osal_delay(50U);
    pmicStatus = Pmic_gpioGetConfiguration(&handle, pin, &gpioCfg_rd);
    if(PMIC_ST_SUCCESS != pmicStatus)
    {
        pmic_log("Failed %s with status: %d\n\t", __func__, pmicStatus);
        return PMIC_UT_FAILURE;
    }

    if(gpioCfg.pinFunc != gpioCfg_rd.pinFunc)
    {
        pmic_log("Failed %s\n\t",__func__);
        return PMIC_UT_FAILURE;
    }

    return PMIC_UT_SUCCESS;
}

/*!
 * \brief   configure gpio pin5 as SPMI SCLK function
 *
 * \param   pmicHandle    [IN]     PMIC Interface Handle.
 *
 * \retval  PMIC_UT_SUCCESS in case of success or
 *          PMIC_UT_FAILURE in case of failure.
 */
static bool test_pmic_gpio_setCfgGpioPin5_spmi_sclk(void *pmicHandle)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_CoreHandle_t handle  = *(Pmic_CoreHandle_t *)pmicHandle;
    int pin                   = 5U;
    Pmic_GpioCfg_t gpioCfg_rd = {PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,};
    Pmic_GpioCfg_t gpioCfg    =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,
        PMIC_GPIO_HIGH,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_GPIO_PINFUNC_SCLK_SPMI,
        PMIC_GPIO_HIGH
    };

    pmicStatus = Pmic_gpioSetConfiguration(&handle, pin, &gpioCfg);
    if(PMIC_ST_SUCCESS != pmicStatus)
    {
        pmic_log("Failed %s with status: %d\n\t", __func__, pmicStatus);
        return PMIC_UT_FAILURE;
    }

    Osal_delay(50U);
    pmicStatus = Pmic_gpioGetConfiguration(&handle, pin, &gpioCfg_rd);
    if(PMIC_ST_SUCCESS != pmicStatus)
    {
        pmic_log("Failed %s with status: %d\n\t", __func__, pmicStatus);
        return PMIC_UT_FAILURE;
    }

    if(gpioCfg.pinFunc != gpioCfg_rd.pinFunc)
    {
        pmic_log("Failed %s\n\t",__func__);
        return PMIC_UT_FAILURE;
    }

    return PMIC_UT_SUCCESS;
}

/*!
 * \brief   configure gpio pin6 as SPMI SDATA function
 *
 * \param   pmicHandle    [IN]     PMIC Interface Handle.
 *
 * \retval  PMIC_UT_SUCCESS in case of success or
 *          PMIC_UT_FAILURE in case of failure.
 */
static bool test_pmic_gpio_setCfgGpioPin6_spmi_sdata(void *pmicHandle)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_CoreHandle_t handle  = *(Pmic_CoreHandle_t *)pmicHandle;
    int pin                   = 6U;
    Pmic_GpioCfg_t gpioCfg_rd = {PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,};
    Pmic_GpioCfg_t gpioCfg    =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,
        PMIC_GPIO_HIGH,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_GPIO_PINFUNC_SDATA_SPMI,
        PMIC_GPIO_HIGH
    };

    pmicStatus = Pmic_gpioSetConfiguration(&handle, pin, &gpioCfg);
    if(PMIC_ST_SUCCESS != pmicStatus)
    {
        pmic_log("Failed %s with status: %d\n\t", __func__, pmicStatus);
        return PMIC_UT_FAILURE;
    }

    Osal_delay(50U);
    pmicStatus = Pmic_gpioGetConfiguration(&handle, pin, &gpioCfg_rd);
    if(PMIC_ST_SUCCESS != pmicStatus)
    {
        pmic_log("Failed %s with status: %d\n\t", __func__, pmicStatus);
        return PMIC_UT_FAILURE;
    }

    if(gpioCfg.pinFunc != gpioCfg_rd.pinFunc)
    {
        pmic_log("Failed %s\n\t",__func__);
        return PMIC_UT_FAILURE;
    }

    return PMIC_UT_SUCCESS;
}

/*!
 * \brief   configure a gpio pin as SYNCCLKOUT function
 *
 * \param   pmicHandle    [IN]     PMIC Interface Handle.
 *
 * \retval  PMIC_UT_SUCCESS in case of success or
 *          PMIC_UT_FAILURE in case of failure.
 */
static bool test_pmic_gpio_setCfgGpioPin_syncCLKOUT(void *pmicHandle)
{
    int32_t pmicStatus         = PMIC_ST_SUCCESS;
    Pmic_CoreHandle_t handle   = *(Pmic_CoreHandle_t *)pmicHandle;
    Pmic_GpioCfg_t gpioCfg_rd = {PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,};
    int pin                    = 0U;
    uint8_t pins[]             = {8U,
    /*! On J721 EVM, PMICA GPIO10 SYNCCLKOUT functionality is not supported
     *  10U */
    };
    Pmic_GpioCfg_t gpioCfg =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,
        PMIC_GPIO_HIGH,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_GPIO_PINFUNC_GPIO8_GPIO10_SYNCCLKOUT,
        PMIC_GPIO_HIGH
    };

    for(pin = 0U; pin < sizeof(pins)/sizeof(pins[0U]); pin++)
    {
        pmicStatus = Pmic_gpioSetConfiguration(&handle, pins[pin], &gpioCfg);
        if(PMIC_ST_SUCCESS != pmicStatus)
        {
            pmic_log("Failed %s with status: %d\n\t", __func__, pmicStatus);
            return PMIC_UT_FAILURE;
        }

        Osal_delay(50U);
        pmicStatus = Pmic_gpioGetConfiguration(&handle,
                                               pins[pin],
                                               &gpioCfg_rd);
        if(PMIC_ST_SUCCESS != pmicStatus)
        {
            pmic_log("Failed %s with status: %d\n\t", __func__, pmicStatus);
            return PMIC_UT_FAILURE;
        }

        if(gpioCfg.pinFunc != gpioCfg_rd.pinFunc)
        {
            pmic_log("Failed %s\n\t",__func__);
            return PMIC_UT_FAILURE;
        }
    }

    return PMIC_UT_SUCCESS;
}

/*!
 * \brief   configure gpio pin9 as SYNCCLKOUT function
 *
 * \param   pmicHandle    [IN]     PMIC Interface Handle.
 *
 * \retval  PMIC_UT_SUCCESS in case of success or
 *          PMIC_UT_FAILURE in case of failure.
 */
static bool test_pmic_gpio_setCfgGpioPin9_syncCLKOUT(void *pmicHandle)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_CoreHandle_t handle  = *(Pmic_CoreHandle_t *)pmicHandle;
    int pin                   = 9U;
    Pmic_GpioCfg_t gpioCfg_rd = {PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,};
    Pmic_GpioCfg_t gpioCfg    =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,
        PMIC_GPIO_HIGH,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_GPIO_PINFUNC_GPIO9_SYNCCLKOUT,
        PMIC_GPIO_HIGH
    };

    pmicStatus = Pmic_gpioSetConfiguration(&handle, pin, &gpioCfg);
    if(PMIC_ST_SUCCESS != pmicStatus)
    {
        pmic_log("Failed %s with status: %d\n\t", __func__, pmicStatus);
        return PMIC_UT_FAILURE;
    }

    Osal_delay(50U);
    pmicStatus = Pmic_gpioGetConfiguration(&handle, pin, &gpioCfg_rd);
    if(PMIC_ST_SUCCESS != pmicStatus)
    {
        pmic_log("Failed %s with status: %d\n\t", __func__, pmicStatus);
        return PMIC_UT_FAILURE;
    }

    if(gpioCfg.pinFunc != gpioCfg_rd.pinFunc)
    {
        pmic_log("Failed %s\n\t",__func__);
        return PMIC_UT_FAILURE;
    }

    return PMIC_UT_SUCCESS;
}

/*!
 * \brief   configure gpio pin10 as SYNCCLKIN function
 *
 * \param   pmicHandle    [IN]     PMIC Interface Handle.
 *
 * \retval  PMIC_UT_SUCCESS in case of success or
 *          PMIC_UT_FAILURE in case of failure.
 */
static bool test_pmic_gpio_setCfgGpioPin10_synCLKIN(void *pmicHandle)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_CoreHandle_t handle  = *(Pmic_CoreHandle_t *)pmicHandle;
    int pin                   = 10U;
    Pmic_GpioCfg_t gpioCfg_rd = {PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,};
    Pmic_GpioCfg_t gpioCfg    =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,
        PMIC_GPIO_HIGH,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_GPIO_PINFUNC_SYNCCLKIN,
        PMIC_GPIO_HIGH
    };

    pmicStatus = Pmic_gpioSetConfiguration(&handle, pin, &gpioCfg);
    if(PMIC_ST_SUCCESS != pmicStatus)
    {
        pmic_log("Failed %s with status: %d\n\t", __func__, pmicStatus);
        return PMIC_UT_FAILURE;
    }

    Osal_delay(50U);
    pmicStatus = Pmic_gpioGetConfiguration(&handle, pin, &gpioCfg_rd);
    if(PMIC_ST_SUCCESS != pmicStatus)
    {
        pmic_log("Failed %s with status: %d\n\t", __func__, pmicStatus);
        return PMIC_UT_FAILURE;
    }

    if(gpioCfg.pinFunc != gpioCfg_rd.pinFunc)
    {
        pmic_log("Failed %s\n\t",__func__);
        return PMIC_UT_FAILURE;
    }

    return PMIC_UT_SUCCESS;
}

/*!
 * \brief   configure a gpio pin as CLK32KOUT function
 *
 * \param   pmicHandle    [IN]     PMIC Interface Handle.
 *
 * \retval  PMIC_UT_SUCCESS in case of success or
 *          PMIC_UT_FAILURE in case of failure.
 */
static bool test_pmic_gpio_setCfgGpioPin_clk32KOUT(void *pmicHandle)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_CoreHandle_t handle  = *(Pmic_CoreHandle_t *)pmicHandle;
    Pmic_GpioCfg_t gpioCfg_rd = {PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,};
    int pin                   = 0U;
    uint8_t pins[]            = {3U, 4U, 8U};
    Pmic_GpioCfg_t gpioCfg    =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,
        PMIC_GPIO_HIGH,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_GPIO_PINFUNC_CLK32KOUT,
        PMIC_GPIO_HIGH
    };

    for(pin = 0U; pin < sizeof(pins)/sizeof(pins[0U]); pin++)
    {
        pmicStatus = Pmic_gpioSetConfiguration(&handle, pins[pin], &gpioCfg);
        if(PMIC_ST_SUCCESS != pmicStatus)
        {
            pmic_log("Failed %s with status: %d\n\t", __func__, pmicStatus);
            return PMIC_UT_FAILURE;
        }

        Osal_delay(50U);
        pmicStatus = Pmic_gpioGetConfiguration(&handle,
                                               pins[pin],
                                               &gpioCfg_rd);
        if(PMIC_ST_SUCCESS != pmicStatus)
        {
            pmic_log("Failed %s with status: %d\n\t", __func__, pmicStatus);
            return PMIC_UT_FAILURE;
        }

        if(gpioCfg.pinFunc != gpioCfg_rd.pinFunc)
        {
            pmic_log("Failed %s\n\t",__func__);
            return PMIC_UT_FAILURE;
        }
    }

    return PMIC_UT_SUCCESS;
}

/* TODO: Need to check feasible or not */
#if 0
/*!
 * \brief   configure gpio pin10 as CLK32KOUT function
 *
 * \param   pmicHandle    [IN]     PMIC Interface Handle.
 *
 * \retval  PMIC_UT_SUCCESS in case of success or
 *          PMIC_UT_FAILURE in case of failure
 */
static bool test_pmic_gpio_setCfgGpioPin10_clk32KOUT(void *pmicHandle)
{
    int32_t pmicStatus       = PMIC_ST_SUCCESS;
    Pmic_CoreHandle_t handle = *(Pmic_CoreHandle_t *)pmicHandle;
    /* On J721 EVM, PMICA GPIO10 CLK32KOUT functionality is not supported */
    int pin                  = 10U;
    Pmic_GpioCfg_t gpioCfg_rd = {PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,};
    Pmic_GpioCfg_t gpioCfg   =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,
        PMIC_GPIO_HIGH,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_GPIO_PINFUNC_GPIO10_CLK32KOUT,
        PMIC_GPIO_HIGH
    };

    /* On J721 EVM, PMICA GPIO10 CLK32KOUT functionality is not supported */
    pmicStatus = Pmic_gpioSetConfiguration(&handle, pin, &gpioCfg);
    if(PMIC_ST_SUCCESS != pmicStatus)
    {
        pmic_log("Failed %s with status: %d\n\t", __func__, pmicStatus);
        return PMIC_UT_FAILURE;
    }

    Osal_delay(50U);
    pmicStatus = Pmic_gpioGetConfiguration(&handle, pin, &gpioCfg_rd);
    if(PMIC_ST_SUCCESS != pmicStatus)
    {
        pmic_log("Failed %s with status: %d\n\t", __func__, pmicStatus);
        return PMIC_UT_FAILURE;
    }

    if(gpioCfg.pinFunc != gpioCfg_rd.pinFunc)
    {
        pmic_log("Failed %s\n\t",__func__);
        return PMIC_UT_FAILURE;
    }

    return PMIC_UT_SUCCESS;
}
#endif

/*!
 * \brief   configure gpio pin8 as Watchdog disable function
 *
 * \param   pmicHandle    [IN]     PMIC Interface Handle.
 *
 * \retval  PMIC_UT_SUCCESS in case of success or
 *          PMIC_UT_FAILURE in case of failure.
 */
static bool test_pmic_gpio_setCfgGpioPin8_wdg_disable(void *pmicHandle)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_CoreHandle_t handle  = *(Pmic_CoreHandle_t *)pmicHandle;
    int pin                   = 8U;
    Pmic_GpioCfg_t gpioCfg_rd = {PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,};
    Pmic_GpioCfg_t gpioCfg    =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,
        PMIC_GPIO_HIGH,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_GPIO_PINFUNC_GPIO8_DISABLE_WDOG,
        PMIC_GPIO_HIGH
    };

    pmicStatus = Pmic_gpioSetConfiguration(&handle, pin, &gpioCfg);
    if(PMIC_ST_SUCCESS != pmicStatus)
    {
        pmic_log("Failed %s with status: %d\n\t", __func__, pmicStatus);
        return PMIC_UT_FAILURE;
    }

    Osal_delay(50U);
    pmicStatus = Pmic_gpioGetConfiguration(&handle, pin, &gpioCfg_rd);
    if(PMIC_ST_SUCCESS != pmicStatus)
    {
        pmic_log("Failed %s with status: %d\n\t", __func__, pmicStatus);
        return PMIC_UT_FAILURE;
    }

    if(gpioCfg.pinFunc != gpioCfg_rd.pinFunc)
    {
        pmic_log("Failed %s\n\t",__func__);
        return PMIC_UT_FAILURE;
    }

    return PMIC_UT_SUCCESS;
}

/*!
 * \brief   configure gpio pin9 as Watchdog disable function
 *
 * \param   pmicHandle    [IN]     PMIC Interface Handle.
 *
 * \retval  PMIC_UT_SUCCESS in case of success or
 *          PMIC_UT_FAILURE in case of failure.
 */
static bool test_pmic_gpio_setCfgGpioPin9_wdg_disable(void *pmicHandle)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_CoreHandle_t handle  = *(Pmic_CoreHandle_t *)pmicHandle;
    int pin                   = 9U;
    Pmic_GpioCfg_t gpioCfg_rd = {PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,};
    Pmic_GpioCfg_t gpioCfg    =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,
        PMIC_GPIO_HIGH,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_GPIO_PINFUNC_GPIO9_DISABLE_WDOG,
        PMIC_GPIO_HIGH
    };

    pmicStatus = Pmic_gpioSetConfiguration(&handle, pin, &gpioCfg);
    if(PMIC_ST_SUCCESS != pmicStatus)
    {
        pmic_log("Failed %s with status: %d\n\t", __func__, pmicStatus);
        return PMIC_UT_FAILURE;
    }

    Osal_delay(50U);
    pmicStatus = Pmic_gpioGetConfiguration(&handle, pin, &gpioCfg_rd);
    if(PMIC_ST_SUCCESS != pmicStatus)
    {
        pmic_log("Failed %s with status: %d\n\t", __func__, pmicStatus);
        return PMIC_UT_FAILURE;
    }

    if(gpioCfg.pinFunc != gpioCfg_rd.pinFunc)
    {
        pmic_log("Failed %s\n\t",__func__);
        return PMIC_UT_FAILURE;
    }

    return PMIC_UT_SUCCESS;
}

/*!
 * \brief   configure gpio pin9 as Power Good Indication line function
 *
 * \param   pmicHandle    [IN]     PMIC Interface Handle.
 *
 * \retval  PMIC_UT_SUCCESS in case of success or
 *          PMIC_UT_FAILURE in case of failure.
 */
static bool test_pmic_gpio_setCfgGpioPin9_good_power(void *pmicHandle)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_CoreHandle_t handle  = *(Pmic_CoreHandle_t *)pmicHandle;
    int pin                   = 9U;
    Pmic_GpioCfg_t gpioCfg_rd = {PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,};
    Pmic_GpioCfg_t gpioCfg    =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,
        PMIC_GPIO_HIGH,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_GPIO_PINFUNC_PGOOD,
        PMIC_GPIO_HIGH
    };

    pmicStatus = Pmic_gpioSetConfiguration(&handle, pin, &gpioCfg);
    if(PMIC_ST_SUCCESS != pmicStatus)
    {
        pmic_log("Failed %s with status: %d\n\t", __func__, pmicStatus);
        return PMIC_UT_FAILURE;
    }

    Osal_delay(50U);
    pmicStatus = Pmic_gpioGetConfiguration(&handle, pin, &gpioCfg_rd);
    if(PMIC_ST_SUCCESS != pmicStatus)
    {
        pmic_log("Failed %s with status: %d\n\t", __func__, pmicStatus);
        return PMIC_UT_FAILURE;
    }

    if(gpioCfg.pinFunc != gpioCfg_rd.pinFunc)
    {
        pmic_log("Failed %s\n\t",__func__);
        return PMIC_UT_FAILURE;
    }

    return PMIC_UT_SUCCESS;
}

/*!
 * \brief   Parameter validation for handle
 *
 * \param   pmicHandle    [IN]     PMIC Interface Handle.
 *
 * \retval  PMIC_UT_SUCCESS in case of success or
 *          PMIC_UT_FAILURE in case of failure.
 */
static bool test_pmic_gpio_setCfgPrmValTest_handle(void *pmicHandle)
{
    int32_t pmicStatus     = PMIC_ST_SUCCESS;
    uint8_t pin            = 1U;
    Pmic_GpioCfg_t gpioCfg =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,
        PMIC_GPIO_HIGH,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_GPIO_PINFUNC_NSLEEP1,
        PMIC_GPIO_HIGH
    };

    pmicStatus = Pmic_gpioSetConfiguration(NULL, pin, &gpioCfg);
    if(pmicStatus != PMIC_ST_ERR_INV_HANDLE)
    {
        pmic_log("Failed %s with status: %d\n\t", __func__, pmicStatus);
        return PMIC_UT_FAILURE;
    }

    return PMIC_UT_SUCCESS;
}

/*!
 * \brief   Parameter validation for pin
 *
 * \param   pmicHandle    [IN]     PMIC Interface Handle.
 *
 * \retval  PMIC_UT_SUCCESS in case of success or
 *          PMIC_UT_FAILURE in case of failure.
 */
static bool test_pmic_gpio_setCfgPrmValTest_pin(void *pmicHandle)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_CoreHandle_t handle  = *(Pmic_CoreHandle_t *)pmicHandle;
    uint8_t pin               = 12U;
    Pmic_GpioCfg_t gpioCfg    =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,
        PMIC_GPIO_HIGH,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_GPIO_PINFUNC_NSLEEP1,
        PMIC_GPIO_HIGH
    };

    pmicStatus = Pmic_gpioSetConfiguration(&handle, pin, &gpioCfg);
    if(pmicStatus != PMIC_ST_ERR_INV_PARAM)
    {
        pmic_log("Failed %s with status: %d\n\t", __func__, pmicStatus);
        return PMIC_UT_FAILURE;
    }

    return PMIC_UT_SUCCESS;
}

/*!
 * \brief   Parameter validation for GpioCfg
 *
 * \param   pmicHandle    [IN]     PMIC Interface Handle.
 *
 * \retval  PMIC_UT_SUCCESS in case of success or
 *          PMIC_UT_FAILURE in case of failure.
 */
static bool test_pmic_gpio_setCfgPrmValTest_gpioCfg(void *pmicHandle)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_CoreHandle_t handle  = *(Pmic_CoreHandle_t *)pmicHandle;
    uint8_t pin               = 1U;

    pmicStatus = Pmic_gpioSetConfiguration(&handle, pin, NULL);
    if(pmicStatus != PMIC_ST_ERR_NULL_PARAM)
    {
        pmic_log("Failed %s with status: %d\n\t", __func__, pmicStatus);
        return PMIC_UT_FAILURE;
    }

    return PMIC_UT_SUCCESS;
}

/*!
 * \brief   Parameter validation for pinDir
 *
 * \param   pmicHandle    [IN]     PMIC Interface Handle.
 *
 * \retval  PMIC_UT_SUCCESS in case of success or
 *          PMIC_UT_FAILURE in case of failure.
 */
static bool test_pmic_gpio_setCfgPrmValTest_pinDir(void *pmicHandle)
{
    int32_t pmicStatus       = PMIC_ST_SUCCESS;
    Pmic_CoreHandle_t handle = *(Pmic_CoreHandle_t *)pmicHandle;
    uint8_t pin              = 1U;
    Pmic_GpioCfg_t gpioCfg   =
    {
        PMIC_GPIO_CFG_DIR_VALID_SHIFT,
        0x02,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_GPIO_PINFUNC_NSLEEP1,
        PMIC_GPIO_HIGH
    };

    pmicStatus = Pmic_gpioSetConfiguration(&handle, pin, &gpioCfg);
    if(pmicStatus != PMIC_ST_ERR_INV_PARAM)
    {
        pmic_log("Failed %s with status: %d\n\t", __func__, pmicStatus);
        return PMIC_UT_FAILURE;
    }

    return PMIC_UT_SUCCESS;
}

/*!
 * \brief   Parameter validation for outputSignalType
 *
 * \param   pmicHandle    [IN]     PMIC Interface Handle.
 *
 * \retval  PMIC_UT_SUCCESS in case of success or
 *          PMIC_UT_FAILURE in case of failure.
 */
static bool test_pmic_gpio_setCfgPrmValTest_outputSignalType(void *pmicHandle)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_CoreHandle_t handle  = *(Pmic_CoreHandle_t *)pmicHandle;
    uint8_t pin               = 1U;
    Pmic_GpioCfg_t gpioCfg    =
    {
        PMIC_GPIO_CFG_OD_VALID_SHIFT,
        PMIC_GPIO_HIGH,
        0x02,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_GPIO_PINFUNC_NSLEEP1,
        PMIC_GPIO_HIGH
    };

    pmicStatus = Pmic_gpioSetConfiguration(&handle, pin, &gpioCfg);
    if(pmicStatus != PMIC_ST_ERR_INV_PARAM)
    {
        pmic_log("Failed %s with status: %d\n\t", __func__, pmicStatus);
        return PMIC_UT_FAILURE;
    }

    return PMIC_UT_SUCCESS;
}

/*!
 * \brief   Parameter validation for deglitchEnable
 *
 * \param   pmicHandle    [IN]     PMIC Interface Handle.
 *
 * \retval  PMIC_UT_SUCCESS in case of success or
 *          PMIC_UT_FAILURE in case of failure.
 */
static bool test_pmic_gpio_setCfgPrmValTest_deglitchEnable(void *pmicHandle)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_CoreHandle_t handle  = *(Pmic_CoreHandle_t *)pmicHandle;
    uint8_t pin               = 1U;
    Pmic_GpioCfg_t gpioCfg    =
    {
        PMIC_GPIO_CFG_DEGLITCH_VALID_SHIFT,
        PMIC_GPIO_HIGH,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        0x02,
        PMIC_GPIO_PINFUNC_NSLEEP1,
        PMIC_GPIO_HIGH
    };

    pmicStatus = Pmic_gpioSetConfiguration(&handle, pin, &gpioCfg);
    if(pmicStatus != PMIC_ST_ERR_INV_PARAM)
    {
        pmic_log("Failed %s with status: %d\n\t", __func__, pmicStatus);
        return PMIC_UT_FAILURE;
    }

    return PMIC_UT_SUCCESS;
}

/*!
 * \brief   Parameter validation for pinFunc
 *
 * \param   pmicHandle    [IN]     PMIC Interface Handle.
 *
 * \retval  PMIC_UT_SUCCESS in case of success or
 *          PMIC_UT_FAILURE in case of failure.
 */
static bool test_pmic_gpio_setCfgPrmValTest_pinFunc_case1(void *pmicHandle)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_CoreHandle_t handle  = *(Pmic_CoreHandle_t *)pmicHandle;
    uint8_t pin               = 1U;
    Pmic_GpioCfg_t gpioCfg    =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,
        PMIC_GPIO_HIGH,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_GPIO_PINFUNC_MAX,
        PMIC_GPIO_HIGH
    };

    pmicStatus = Pmic_gpioSetConfiguration(&handle, pin, &gpioCfg);
    if(pmicStatus != PMIC_ST_ERR_INV_GPIO_FUNC)
    {
        pmic_log("Failed %s with status: %d\n\t", __func__, pmicStatus);
        return PMIC_UT_FAILURE;
    }

    return PMIC_UT_SUCCESS;
}

/*!
 * \brief   Parameter validation for pinFunc
 *
 * \param   pmicHandle    [IN]     PMIC Interface Handle.
 *
 * \retval  PMIC_UT_SUCCESS in case of success or
 *          PMIC_UT_FAILURE in case of failure.
 */
static bool test_pmic_gpio_setCfgPrmValTest_pinFunc_case2(void *pmicHandle)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_CoreHandle_t handle  = *(Pmic_CoreHandle_t *)pmicHandle;
    uint8_t pin               = PMIC_NPWRON_PIN;
    Pmic_GpioCfg_t gpioCfg    =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,
        PMIC_GPIO_HIGH,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        3U,
        PMIC_GPIO_HIGH
    };

    pmicStatus = Pmic_gpioSetConfiguration(&handle, pin, &gpioCfg);
    if(pmicStatus != PMIC_ST_ERR_INV_GPIO_FUNC)
    {
        pmic_log("Failed %s with status: %d\n\t", __func__, pmicStatus);
        return PMIC_UT_FAILURE;
    }

    return PMIC_UT_SUCCESS;
}

/*!
 * \brief   Parameter validation for pinPolarity
 *
 * \param   pmicHandle    [IN]     PMIC Interface Handle.
 *
 * \retval  PMIC_UT_SUCCESS in case of success or
 *          PMIC_UT_FAILURE in case of failure.
 */
static bool test_pmic_gpio_setCfgPrmValTest_pinPolarity(void *pmicHandle)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_CoreHandle_t handle  = *(Pmic_CoreHandle_t *)pmicHandle;
    uint8_t pin               = PMIC_NPWRON_PIN;
    Pmic_GpioCfg_t gpioCfg    =
    {
        PMIC_NPWRON_CFG_POLARITY_VALID_SHIFT,
        PMIC_GPIO_HIGH,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_GPIO_PINFUNC_NSLEEP1,
        0x02
    };

    pmicStatus = Pmic_gpioSetConfiguration(&handle, pin, &gpioCfg);
    if(pmicStatus != PMIC_ST_ERR_INV_PARAM)
    {
        pmic_log("Failed %s with status: %d\n\t", __func__, pmicStatus);
        return PMIC_UT_FAILURE;
    }

    return PMIC_UT_SUCCESS;
}

/*!
 * \brief   Get required GPIO pin configuration
 *
 * \param   pmicHandle    [IN]     PMIC Interface Handle.
 *
 * \retval  PMIC_UT_SUCCESS in case of success or
 *          PMIC_UT_FAILURE in case of failure.
 */
static bool test_pmic_gpio_getCfgGpioPin(void *pmicHandle)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_CoreHandle_t handle  = *(Pmic_CoreHandle_t *)pmicHandle;
    int pin                   = 1U;
    Pmic_GpioCfg_t gpioCfg_rd = {PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,};
    Pmic_GpioCfg_t gpioCfg    =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,
        PMIC_GPIO_HIGH,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_GPIO_PINFUNC_NSLEEP1,
        PMIC_GPIO_HIGH
    };

    pmicStatus = Pmic_gpioSetConfiguration(&handle, pin, &gpioCfg);
    if(PMIC_ST_SUCCESS != pmicStatus)
    {
        pmic_log("Failed %s with status: %d\n\t", __func__, pmicStatus);
        return PMIC_UT_FAILURE;
    }

    Osal_delay(50U);
    pmicStatus = Pmic_gpioGetConfiguration(&handle, pin, &gpioCfg_rd);
    if(PMIC_ST_SUCCESS != pmicStatus)
    {
        pmic_log("Failed %s with status: %d\n\t", __func__, pmicStatus);
        return PMIC_UT_FAILURE;
    }

    if(gpioCfg.pinFunc != gpioCfg_rd.pinFunc)
    {
        pmic_log("Failed %s\n\t",__func__);
        return PMIC_UT_FAILURE;
    }

    return PMIC_UT_SUCCESS;
}

/*!
 * \brief   Parameter validation for handle
 *
 * \param   pmicHandle    [IN]     PMIC Interface Handle.
 *
 * \retval  PMIC_UT_SUCCESS in case of success or
 *          PMIC_UT_FAILURE in case of failure.
 */
static bool test_pmic_gpio_getCfgPrmValTest_handle(void *pmicHandle)
{
    int32_t pmicStatus     = PMIC_ST_SUCCESS;
    uint8_t pin            = 1U;
    Pmic_GpioCfg_t gpioCfg =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,
        PMIC_GPIO_HIGH,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_GPIO_PINFUNC_NSLEEP1,
        PMIC_GPIO_HIGH
    };

    pmicStatus = Pmic_gpioGetConfiguration(NULL, pin, &gpioCfg);
    if(pmicStatus != PMIC_ST_ERR_INV_HANDLE)
    {
        pmic_log("Failed %s with status: %d\n\t", __func__, pmicStatus);
        return PMIC_UT_FAILURE;
    }

    return PMIC_UT_SUCCESS;
}

/*!
 * \brief   Parameter validation for pin
 *
 * \param   pmicHandle    [IN]     PMIC Interface Handle.
 *
 * \retval  PMIC_UT_SUCCESS in case of success or
 *          PMIC_UT_FAILURE in case of failure.
 */
static bool test_pmic_gpio_getCfgPrmValTest_pin(void *pmicHandle)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_CoreHandle_t handle  = *(Pmic_CoreHandle_t *)pmicHandle;
    uint8_t pin               = 12U;
    Pmic_GpioCfg_t gpioCfg    =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,
        PMIC_GPIO_HIGH,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_GPIO_PINFUNC_NSLEEP1,
        PMIC_GPIO_HIGH
    };

    pmicStatus = Pmic_gpioGetConfiguration(&handle, pin, &gpioCfg);
    if(pmicStatus != PMIC_ST_ERR_INV_PARAM)
    {
        pmic_log("Failed %s with status: %d\n\t", __func__, pmicStatus);
        return PMIC_UT_FAILURE;
    }

    return PMIC_UT_SUCCESS;
}

/*!
 * \brief   Parameter validation for GpioCfg
 *
 * \param   pmicHandle    [IN]     PMIC Interface Handle.
 *
 * \retval  PMIC_UT_SUCCESS in case of success or
 *          PMIC_UT_FAILURE in case of failure.
 */
static bool test_pmic_gpio_getCfgPrmValTest_gpioCfg(void *pmicHandle)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_CoreHandle_t handle  = *(Pmic_CoreHandle_t *)pmicHandle;
    uint8_t pin               = 1U;

    pmicStatus = Pmic_gpioGetConfiguration(&handle, pin, NULL);
    if(pmicStatus != PMIC_ST_ERR_NULL_PARAM)
    {
        pmic_log("Failed %s with status: %d\n\t", __func__, pmicStatus);
        return PMIC_UT_FAILURE;
    }

    return PMIC_UT_SUCCESS;
}

/*!
 * \brief   Get GPIO signal level
 *
 * \param   pmicHandle    [IN]     PMIC Interface Handle.
 *
 * \retval  PMIC_UT_SUCCESS in case of success or
 *          PMIC_UT_FAILURE in case of failure.
 */
static bool test_pmic_gpio_getValueGpioPin1_signalLevel(void *pmicHandle)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_CoreHandle_t handle  = *(Pmic_CoreHandle_t *)pmicHandle;
    uint8_t pin               = 1U;
    uint8_t pinValue          = PMIC_GPIO_HIGH;
    uint8_t pinValue_rd;
    Pmic_GpioCfg_t gpioCfg    =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT | PMIC_GPIO_CFG_DIR_VALID_SHIFT,
        PMIC_GPIO_HIGH,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_GPIO_PINFUNC_GPIO,
        PMIC_GPIO_HIGH
    };

    pmicStatus = Pmic_gpioSetConfiguration(&handle, pin, &gpioCfg);
    if(PMIC_ST_SUCCESS != pmicStatus)
    {
        pmic_log("Failed %s with status: %d\n\t", __func__, pmicStatus);
        return PMIC_UT_FAILURE;
    }

    Osal_delay(50U);
    pmicStatus = Pmic_gpioSetValue(&handle, pin, pinValue);
    if(PMIC_ST_SUCCESS != pmicStatus)
    {
        pmic_log("Failed %s with status: %d\n\t", __func__, pmicStatus);
        return PMIC_UT_FAILURE;
    }

    pmicStatus = Pmic_gpioGetValue(&handle, pin, &pinValue_rd);
    if(PMIC_ST_SUCCESS != pmicStatus)
    {
        pmic_log("Failed %s with status: %d\n\t", __func__, pmicStatus);
        return PMIC_UT_FAILURE;
    }

    if(pinValue != pinValue_rd)
    {
        pmic_log("Failed %s\n\t",__func__);
        return PMIC_UT_FAILURE;
    }

    return PMIC_UT_SUCCESS;
}

/*!
 * \brief   Parameter validation for handle
 *
 * \param   pmicHandle    [IN]     PMIC Interface Handle.
 *
 * \retval  PMIC_UT_SUCCESS in case of success or
 *          PMIC_UT_FAILURE in case of failure.
 */
static bool test_pmic_gpio_getValuePrmValTest_handle(void *pmicHandle)
{
    int32_t pmicStatus   = PMIC_ST_SUCCESS;
    uint8_t pin          = 1U;
    uint8_t pinValue;

    pmicStatus = Pmic_gpioGetValue(NULL, pin, &pinValue);
    if(pmicStatus != PMIC_ST_ERR_INV_HANDLE)
    {
        pmic_log("Failed %s with status: %d\n\t", __func__, pmicStatus);
        return PMIC_UT_FAILURE;
    }

    return PMIC_UT_SUCCESS;
}

/*!
 * \brief   Parameter validation for pin
 *
 * \param   pmicHandle    [IN]     PMIC Interface Handle.
 *
 * \retval  PMIC_UT_SUCCESS in case of success or
 *          PMIC_UT_FAILURE in case of failure.
 */
static bool test_pmic_gpio_getValuePrmValTest_pin(void *pmicHandle)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_CoreHandle_t handle  = *(Pmic_CoreHandle_t *)pmicHandle;
    uint8_t pin               = 12U;
    uint8_t pinValue;

    pmicStatus = Pmic_gpioGetValue(&handle,pin,&pinValue);
    if(pmicStatus != PMIC_ST_ERR_INV_PARAM)
    {
        pmic_log("Failed %s with status: %d\n\t", __func__, pmicStatus);
        return PMIC_UT_FAILURE;
    }

    return PMIC_UT_SUCCESS;
}

/*!
 * \brief   Parameter validation for pinValue
 *
 * \param   pmicHandle    [IN]     PMIC Interface Handle.
 *
 * \retval  PMIC_UT_SUCCESS in case of success or
 *          PMIC_UT_FAILURE in case of failure.
 */
static bool test_pmic_gpio_getValuePrmValTest_pinValue(void *pmicHandle)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_CoreHandle_t handle  = *(Pmic_CoreHandle_t *)pmicHandle;
    uint8_t pin               = 1U;

    pmicStatus = Pmic_gpioGetValue(&handle, pin, NULL);
    if(pmicStatus != PMIC_ST_ERR_NULL_PARAM)
    {
        pmic_log("Failed %s with status: %d\n\t", __func__, pmicStatus);
        return PMIC_UT_FAILURE;
    }

    return PMIC_UT_SUCCESS;
}

/*!
 * \brief   Set GPIO signal level
 *
 * \param   pmicHandle    [IN]     PMIC Interface Handle.
 *
 * \retval  PMIC_UT_SUCCESS in case of success or
 *          PMIC_UT_FAILURE in case of failure.
 */
static bool test_pmic_gpio_setValueGpioPin1_signalLevel(void *pmicHandle)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_CoreHandle_t handle  = *(Pmic_CoreHandle_t *)pmicHandle;
    uint8_t pin               = 1U;
    uint8_t pinValue          = PMIC_GPIO_HIGH;
    uint8_t pinValue_rd;
    Pmic_GpioCfg_t gpioCfg    =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT | PMIC_GPIO_CFG_DIR_VALID_SHIFT,
        PMIC_GPIO_HIGH,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_GPIO_PINFUNC_GPIO,
        PMIC_GPIO_HIGH
    };

    pmicStatus = Pmic_gpioSetConfiguration(&handle, pin, &gpioCfg);
    if(PMIC_ST_SUCCESS != pmicStatus)
    {
        pmic_log("Failed %s with status: %d\n\t", __func__, pmicStatus);
        return PMIC_UT_FAILURE;
    }

    Osal_delay(50U);
    pmicStatus = Pmic_gpioSetValue(&handle, pin, pinValue);
    if(PMIC_ST_SUCCESS != pmicStatus)
    {
        pmic_log("Failed %s with status: %d\n\t", __func__, pmicStatus);
        return PMIC_UT_FAILURE;
    }

    pmicStatus = Pmic_gpioGetValue(&handle, pin, &pinValue_rd);
    if(PMIC_ST_SUCCESS != pmicStatus)
    {
        pmic_log("Failed %s with status: %d\n\t", __func__, pmicStatus);
        return PMIC_UT_FAILURE;
    }

    if(pinValue != pinValue_rd)
    {
        pmic_log("Failed %s\n\t",__func__);
        return PMIC_UT_FAILURE;
    }

    return PMIC_UT_SUCCESS;
}

/*!
 * \brief   Parameter validation for handle
 *
 * \param   pmicHandle    [IN]     PMIC Interface Handle.
 *
 * \retval  PMIC_UT_SUCCESS in case of success or
 *          PMIC_UT_FAILURE in case of failure.
 */
static bool test_pmic_gpio_setValuePrmValTest_handle(void *pmicHandle)
{
    int32_t pmicStatus    = PMIC_ST_SUCCESS;
    uint8_t pin           = 1U;
    uint8_t pinValue      = PMIC_GPIO_HIGH;

    pmicStatus = Pmic_gpioSetValue(NULL, pin, pinValue);
    if(pmicStatus != PMIC_ST_ERR_INV_HANDLE)
    {
        pmic_log("Failed %s with status: %d\n\t", __func__, pmicStatus);
        return PMIC_UT_FAILURE;
    }

    return PMIC_UT_SUCCESS;
}

/*!
 * \brief   Parameter validation for pin
 *
 * \param   pmicHandle    [IN]     PMIC Interface Handle.
 *
 * \retval  PMIC_UT_SUCCESS in case of success or
 *          PMIC_UT_FAILURE in case of failure.
 */
static bool test_pmic_gpio_setValuePrmValTest_pin(void *pmicHandle)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_CoreHandle_t handle  = *(Pmic_CoreHandle_t *)pmicHandle;
    uint8_t pin               = 12U;
    uint8_t pinValue          = PMIC_GPIO_HIGH;

    pmicStatus = Pmic_gpioSetValue(&handle, pin, pinValue);
    if(pmicStatus != PMIC_ST_ERR_INV_PARAM)
    {
        pmic_log("Failed %s with status: %d\n\t", __func__, pmicStatus);
        return PMIC_UT_FAILURE;
    }

    return PMIC_UT_SUCCESS;
}

/*!
 * \brief   Parameter validation for pinValue
 *
 * \param   pmicHandle    [IN]     PMIC Interface Handle.
 *
 * \retval  PMIC_UT_SUCCESS in case of success or
 *          PMIC_UT_FAILURE in case of failure.
 */
static bool test_pmic_gpio_setValuePrmValTest_pinValue(void *pmicHandle)
{
    int32_t pmicStatus       = PMIC_ST_SUCCESS;
    Pmic_CoreHandle_t handle = *(Pmic_CoreHandle_t *)pmicHandle;
    uint8_t pin              = 1U;
    uint8_t pinValue         = 2U;

    pmicStatus = Pmic_gpioSetValue(&handle, pin, pinValue);
    if(pmicStatus != PMIC_ST_ERR_INV_PARAM)
    {
        pmic_log("Failed %s with status: %d\n\t", __func__, pmicStatus);
        return PMIC_UT_FAILURE;
    }

    return PMIC_UT_SUCCESS;
}

/*!
 * \brief   Set GPIO signal level for an input GPIO pin
 *
 * \param   pmicHandle    [IN]     PMIC Interface Handle.
 *
 * \retval  PMIC_UT_SUCCESS in case of success or
 *          PMIC_UT_FAILURE in case of failure.
 */
static bool test_pmic_gpio_setValueGpioPin5_input(void *pmicHandle)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_CoreHandle_t handle  = *(Pmic_CoreHandle_t *)pmicHandle;
    uint8_t pinValue          = PMIC_GPIO_HIGH;
    uint8_t pin               = 5U;
    Pmic_GpioCfg_t gpioCfg    =
    {
        PMIC_GPIO_CFG_DIR_VALID_SHIFT,
        PMIC_GPIO_LOW,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_GPIO_PINFUNC_NSLEEP1,
        PMIC_GPIO_HIGH
    };
    Pmic_GpioCfg_t gpioCfg_rd = {PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,};

    pmicStatus = Pmic_gpioSetConfiguration(&handle, pin, &gpioCfg);
    if(PMIC_ST_SUCCESS != pmicStatus)
    {
        pmic_log("Failed %s with status: %d\n\t", __func__, pmicStatus);
        return PMIC_UT_FAILURE;
    }

    Osal_delay(50U);
    pmicStatus = Pmic_gpioGetConfiguration(&handle, pin, &gpioCfg_rd);
    if(PMIC_ST_SUCCESS != pmicStatus)
    {
        pmic_log("Failed %s with status: %d\n\t", __func__, pmicStatus);
        return PMIC_UT_FAILURE;
    }

    if(gpioCfg.pinDir != gpioCfg_rd.pinDir)
    {
        pmic_log("Failed %s\n\t",__func__);
        return PMIC_UT_FAILURE;
    }

    pmicStatus = Pmic_gpioSetValue(&handle, pin, pinValue);
    if(pmicStatus != PMIC_ST_ERR_INV_PARAM)
    {
        pmic_log("Failed %s with status: %d\n\t", __func__, pmicStatus);
        return PMIC_UT_FAILURE;
    }

    return PMIC_UT_SUCCESS;
}

/*!
 * \brief   PMIC GPIO Test Cases
 */
static Pmic_Ut_Tests_t pmic_gpio_tests[] =
{
    /*! testFunc
     *  testID
     *  testDesc
     */
    {
        test_pmic_gpio_setCfgGpioPin_nSLEEP1,
        6185,
        "Pmic_gpioSetConfiguration : configure a gpio pin as NSLEEP1 function"
    },
    {
        test_pmic_gpio_setCfgGpioPin_nSLEEP2,
        6186,
        "Pmic_gpioSetConfiguration : configure a gpio pin as NSLEEP2 function"
    },
    {
        test_pmic_gpio_setCfgGpioPin1_nRstOut_soc,
        6187,
        "Pmic_gpioSetConfiguration : configure gpio pin 1 as NRSTOUT_SOC function"
    },
    {
        test_pmic_gpio_setCfgGpioPin11_nRstOut_soc,
        6188,
        "Pmic_gpioSetConfiguration : configure gpio pin 11 as NRSTOUT_SOC function"
    },
    {
        test_pmic_gpio_setCfgGpioPin_wakeup1,
        6189,
        "Pmic_gpioSetConfiguration : configure a gpio pin as WAKEUP1 function"
    },
    {
        test_pmic_gpio_setCfgGpioPin_wakeup2,
        6190,
        "Pmic_gpioSetConfiguration : configure a gpio pin as WAKEUP2 function"
    },
    {
        test_pmic_gpio_setCfgGpioPin_gpio,
        6191,
        "Pmic_gpioSetConfiguration : configure a gpio pin as general purpose input/output function"
    },
    {
        test_pmic_gpio_setCfgGpioPin1_i2c2_sclk ,
        6192,
        "Pmic_gpioSetConfiguration : configure gpio pin 1 as I2C2 SCLK function"
    },
    {
        test_pmic_gpio_setCfgGpioPin2_i2c2_sda,
        6193,
        "Pmic_gpioSetConfiguration : configure gpio pin 2 as I2C2 SDA function"
    },
    {
        test_pmic_gpio_setCfgGpioPin1_spi_cs,
        6195,
        "Pmic_gpioSetConfiguration : configure gpio pin 1 as SPI CS function"
    },
    {
        test_pmic_gpio_setCfgGpioPin2_spi_sdo,
        6196,
        "Pmic_gpioSetConfiguration : configure gpio pin 2 as SPI SDO function"
    },
    {
        test_pmic_gpio_setCfgGpioPin_wdt,
        6197,
        "Pmic_gpioSetConfiguration : configure gpio pin as watchdog trigger function"
    },
    {
        test_pmic_gpio_setCfgGpioPin3_esm_soc,
        6198,
        "Pmic_gpioSetConfiguration : configure gpio pin 3 as ESM Error Pins for SOC"
    },
    {
        test_pmic_gpio_setCfgGpioPin7_esm_mcu,
        6199,
        "Pmic_gpioSetConfiguration : configure gpio pin 7 as ESM Error Pins for MCU"
    },
    {
        test_pmic_gpio_setCfgGpioPin5_spmi_sclk,
        6200,
        "Pmic_gpioSetConfiguration : configure gpio pin 5 as SPMI SCLK function"
    },
    {
        test_pmic_gpio_setCfgGpioPin6_spmi_sdata,
        6201,
        "Pmic_gpioSetConfiguration : configure gpio pin 6 as SPMI SDATA function"
    },
    {
        test_pmic_gpio_setCfgGpioPin_syncCLKOUT,
        6202,
        "Pmic_gpioSetConfiguration : configure gpio pin as SYNCCLKOUT function"
    },
    {
        test_pmic_gpio_setCfgGpioPin9_syncCLKOUT,
        6203,
        "Pmic_gpioSetConfiguration : configure gpio pin 9 as SYNCCLKOUT function"
    },
    {
        test_pmic_gpio_setCfgGpioPin10_synCLKIN,
        6204,
        "Pmic_gpioSetConfiguration : configure gpio pin 10 as SYNCLKIN function"
    },
    {
        test_pmic_gpio_setCfgGpioPin_clk32KOUT,
        6205,
        "Pmic_gpioSetConfiguration : configure gpio pin as CLK32KOUT function"
    },
/* TODO: Need to check feasible or not */
/*  {
      test_pmic_gpio_setCfgGpioPin10_clk32KOUT,
      TID_7348_T10_21,
      "Pmic_gpioSetConfiguration : configure gpio pin 10 as CLK32KOUT function"
    },
*/
    {
        test_pmic_gpio_setCfgGpioPin8_wdg_disable,
        6207,
        "Pmic_gpioSetConfiguration : configure gpio pin 8 as Watchdog disable function"
    },
    {
        test_pmic_gpio_setCfgGpioPin9_wdg_disable,
        6208,
        "Pmic_gpioSetConfiguration : configure gpio pin 9 as Watchdog disable function"
    },
    {
        test_pmic_gpio_setCfgGpioPin9_good_power,
        6209,
        "Pmic_gpioSetConfiguration : configure gpio pin 9 as Power Good Indication line function"
    },
    {
        test_pmic_gpio_setCfgPrmValTest_handle,
        6210,
        "Pmic_gpioSetConfiguration : Parameter validation for handle"
    },
    {
        test_pmic_gpio_setCfgPrmValTest_pin,
        6211,
        "Pmic_gpioSetConfiguration : Parameter validation for pin "
    },
    {
        test_pmic_gpio_setCfgPrmValTest_gpioCfg,
        6212,
        "Pmic_gpioSetConfiguration : Parameter validation for GpioCfg"
    },
    {
        test_pmic_gpio_setCfgPrmValTest_pinDir,
        6213,
        "Pmic_gpioSetConfiguration : Gpio pin configuration validation for pinDir "
    },
    {
        test_pmic_gpio_setCfgPrmValTest_outputSignalType,
        6214,
        "Pmic_gpioSetConfiguration : Gpio pin configuration validation for outputSignalType"
    },
    {
        test_pmic_gpio_setCfgPrmValTest_deglitchEnable,
        6215,
        "Pmic_gpioSetConfiguration : Gpio pin configuration validation for deglitchEnable "
    },
    {
        test_pmic_gpio_setCfgPrmValTest_pinFunc_case1,
        6216,
        "Pmic_gpioSetConfiguration : Gpio pin configuration validation for pinFunc"
    },
    {
        test_pmic_gpio_setCfgPrmValTest_pinFunc_case2,
        6217,
        "Pmic_gpioSetConfiguration : Gpio pin configuration validation for pinFunc "
    },
    {
        test_pmic_gpio_setCfgPrmValTest_pinPolarity,
        6218,
        "Pmic_gpioSetConfiguration : Gpio pin configuration validation for pinPolarity"
    },
    {
        test_pmic_gpio_getCfgGpioPin,
        6219,
        "Pmic_gpioGetConfiguration : Get required gpio pin configuration"
    },
    {
        test_pmic_gpio_getCfgPrmValTest_handle,
        6220,
        "Pmic_gpioGetConfiguration : Parameter validation for handle"
    },
    {
        test_pmic_gpio_getCfgPrmValTest_pin,
        6221,
        "Pmic_gpioGetConfiguration : Parameter validation for pin"
    },
    {
        test_pmic_gpio_getCfgPrmValTest_gpioCfg,
        6222,
        "Pmic_gpioGetConfiguration : Parameter validation for GpioCfg"
    },
    {
        test_pmic_gpio_getValueGpioPin1_signalLevel,
        6223,
        "Pmic_gpioGetValue : Get GPIO signal level "
    },
    {
        test_pmic_gpio_getValuePrmValTest_handle,
        6224,
        "Pmic_gpioGetValue : Parameter validation for handle"
    },
    {
        test_pmic_gpio_getValuePrmValTest_pin,
        6225,
        "Pmic_gpioGetValue : Parameter validation for pin"
    },
    {
        test_pmic_gpio_getValuePrmValTest_pinValue,
        6226,
        "Pmic_gpioGetValue : Parameter validation for pinValue "
    },
    {
        test_pmic_gpio_setValueGpioPin1_signalLevel,
        6227,
        "Pmic_gpioSetValue : Set GPIO signal level "
    },
    {
        test_pmic_gpio_setValuePrmValTest_handle,
        6228,
        "Pmic_gpioSetValue : Parameter validation for handle"
    },
    {
        test_pmic_gpio_setValuePrmValTest_pin,
        6229,
        "Pmic_gpioSetValue : Parameter validation for pin "
    },
    {
        test_pmic_gpio_setValuePrmValTest_pinValue,
        6230,
        "Pmic_gpioSetValue : Parameter validation for pinValue "
    },
    {
        test_pmic_gpio_setValueGpioPin5_input,
        6231,
        "Pmic_gpioSetValue : Set GPIO signal level for an input GPIO pin"
    },
    {
        NULL,
    }
};

#if defined(UNITY_INCLUDE_CONFIG_H) && \
    (defined(SOC_J721E) || defined(SOC_J7200))

/*!
 * \brief   GPIO Unity Test App wrapper Function
 */
static void test_pmic_gpio_testApp(void)
{
    bool testResult = PMIC_UT_FAILURE;
    Pmic_CoreCfg_t pmicConfigData = {0U};

     /* Fill parameters to pmicConfigData */
    pmicConfigData.pmicDeviceType      = PMIC_DEV_LEO_TPS6594;
    pmicConfigData.validParams        |= PMIC_CFG_DEVICE_TYPE_VALID_SHIFT;

    pmicConfigData.commMode            = PMIC_INTF_SINGLE_I2C;
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

    pmicConfigData.validParams         |= PMIC_CFG_COMM_HANDLE_VALID_SHIFT;

    pmicConfigData.validParams         |= (PMIC_CFG_COMM_HANDLE_VALID_SHIFT |
                                           PMIC_CFG_QASLAVEADDR_VALID_SHIFT);

    testResult = test_pmic_common(pmic_gpio_tests, &pmicConfigData);
    TEST_ASSERT(PMIC_UT_SUCCESS == testResult);

    pmic_log("\n All tests have passed. \n");

}

/*!
 * \brief   Function to register GPIO Unity Test App wrapper to Unity framework
 */
static void test_pmic_gpio_testapp_runner(void)
{
    /* @description : Test runner for Gpio Test App
     *
     * @requirements: PDK-5808
     *
     * @cores       : mcu1_0, mcu1_1
     */

    pmic_log("%s(): %d: Begin Unity...\n", __func__, __LINE__);
    UNITY_BEGIN();

    RUN_TEST(test_pmic_gpio_testApp);

    UNITY_END();
    pmic_log("%s(): %d: End Unity...\n", __func__, __LINE__);
    /* Function to print results defined in unity_config.h file */
    print_unityOutputBuffer_usingUARTstdio();
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
    test_pmic_uartInit();

    pmic_log("PMIC GPIO Unit Test started...\n");

    pmic_log("%s(): %d: %s(%s)\n", __func__, __LINE__, __TIME__, __DATE__);

#if defined(UNITY_INCLUDE_CONFIG_H) && \
    (defined(SOC_J721E)             || \
     defined(SOC_J7200))
    test_pmic_gpio_testapp_runner();
#endif
}