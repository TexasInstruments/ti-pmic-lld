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
/**
 * @file gpio_test.c
 * @brief Source file containing definitions to PMIC GPIO tests.
 */

/* ========================================================================== */
/*                              Include Files                                 */
/* ========================================================================== */

#include "gpio_test.h"
#ifdef BUILD_MOCK
#include "test_inject.h"
#endif

/* ========================================================================== */
/*                             Global Variables                               */
/* ========================================================================== */

static Pmic_Handle_t pmicHandle = {0U};

/* ========================================================================== */
/*                           Function Definitions                             */
/* ========================================================================== */

void gpio_test(void *args)
{
    char msg[50U] = {0};
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_HandleCfg_t pmicCfg = {
        .validParams = (PMIC_I2C_ADDR0_VALID |
                        PMIC_COMM_HANDLE_0_VALID |
                        PMIC_IO_READ_VALID |
                        PMIC_IO_WRITE_VALID |
                        PMIC_CRITICAL_SECTION_START_VALID |
                        PMIC_CRITICAL_SECTION_STOP_VALID |
                        PMIC_IRQ_RESPONSE_CALLBACK_VALID),
        .i2cAddr0 = PLATFORM_TARGET_I2C_ADDR,
        .commHandle0 = platform_getCommHandle(),
        .ioRead = &platform_rxByte,
        .ioWrite = &platform_txByte,
        .criticalSectionStart = &platform_critSecStart,
        .criticalSectionStop = &platform_critSecStop,
        .irqResponseCallback = &platform_irqResponse
    };

    platform_init();

    platform_printString("\r\n");
    platform_printString("GPIO_TEST\r\n");
    platform_printString("---------\r\n\r\n");

    status = Pmic_init(&pmicHandle, &pmicCfg);

    if (status == PMIC_ST_SUCCESS)
    {
        testUtils_printSiRev(&pmicHandle);


        if (status == PMIC_ST_SUCCESS)
        {
            status = Pmic_irqClrAllFlags(&pmicHandle);

            if (status == PMIC_ST_SUCCESS)
            {
                platform_setupTests();
                GPIO_TEST_RUN_ALL();
                platform_tearDownTests();
            }
            else
            {
                (void)sprintf(msg, "Error in clearing all PMIC IRQs: %d\r\n", status);
                platform_printString(msg);
            }
        }
        else
        {
            (void)sprintf(msg, "Error in unlocking PMIC registers: %d\r\n", status);
            platform_printString(msg);
        }
    }
    else
    {
        (void)sprintf(msg, "Error in initializing PMIC LLD: %d\r\n", status);
        platform_printString(msg);
    }

    (void)Pmic_deinit(&pmicHandle);
    platform_deinit();
}

/* ========================================================================== */
/*                    Negative Tests - Pmic_gpioSetCfg                        */
/* ========================================================================== */

void test_neg_gpio_gpioSetCfg_nullHandle(void)
{
    // Pass NULL pmicHandle into Pmic_gpioSetCfg()
    Pmic_GpioCfg_t gpioCfg = {
        .validParams = PMIC_FUNCTIONALITY_VALID,
        .functionality = PMIC_GPIO_INPUT
    };
    int32_t status = Pmic_gpioSetCfg(NULL, PMIC_GPIO, &gpioCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_gpio_gpioSetCfg_nullGpioCfg(void)
{
    // Pass NULL gpioCfg into Pmic_gpioSetCfg()
    int32_t status = Pmic_gpioSetCfg(&pmicHandle, PMIC_GPIO, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_gpio_gpioSetCfg_invalidGpioPin(void)
{
    // Pass invalid gpioPin into Pmic_gpioSetCfg()
    Pmic_GpioCfg_t gpioCfg = {
        .validParams = PMIC_FUNCTIONALITY_VALID,
        .functionality = PMIC_GPIO_INPUT
    };
    int32_t status = Pmic_gpioSetCfg(&pmicHandle, PMIC_GPIO_PIN_MAX + 1U, &gpioCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_gpio_gpioSetCfg_zeroValidParams(void)
{
    // Pass validParams = 0 into Pmic_gpioSetCfg()
    Pmic_GpioCfg_t gpioCfg = {
        .validParams = 0U,
        .functionality = PMIC_GPIO_INPUT
    };
    int32_t status = Pmic_gpioSetCfg(&pmicHandle, PMIC_GPIO, &gpioCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_gpio_gpioSetCfg_gpio_invalidValidParams(void)
{
    // Pass invalid validParams for GPIO pin (PU_PD_CFG_VALID not supported for GPIO)
    Pmic_GpioCfg_t gpioCfg = {
        .validParams = PMIC_PU_PD_CFG_VALID,
        .puPdCfg = PMIC_PU_RESISTOR_ACTIVATED
    };
    int32_t status = Pmic_gpioSetCfg(&pmicHandle, PMIC_GPIO, &gpioCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_FAIL);
}

void test_neg_gpio_gpioSetCfg_nIntGpi_invalidValidParams(void)
{
    // Pass invalid validParams with no valid bits set (nothing to configure)
    Pmic_GpioCfg_t gpioCfg = {
        .validParams = 0x00000000U,  // No valid params - should fail
        .functionality = PMIC_NINT_GPI_NINT
    };
    int32_t status = Pmic_gpioSetCfg(&pmicHandle, PMIC_NINT_GPI, &gpioCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_gpio_gpioSetCfg_gpio_outOfBounds_functionality(void)
{
    // Pass out of bounds functionality for GPIO pin
    Pmic_GpioCfg_t gpioCfg = {
        .validParams = PMIC_FUNCTIONALITY_VALID,
        .functionality = PMIC_GPIO_FUNCTIONALITY_MAX + 1U
    };
    int32_t status = Pmic_gpioSetCfg(&pmicHandle, PMIC_GPIO, &gpioCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_gpio_gpioSetCfg_gpio_outOfBounds_polarity(void)
{
    // Pass out of bounds polarity for GPIO pin
    Pmic_GpioCfg_t gpioCfg = {
        .validParams = PMIC_POLARITY_VALID,
        .polarity = PMIC_POLARITY_MAX + 1U
    };
    int32_t status = Pmic_gpioSetCfg(&pmicHandle, PMIC_GPIO, &gpioCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_gpio_gpioSetCfg_nIntGpi_outOfBounds_functionality(void)
{
    // Pass out of bounds functionality for NINT_GPI pin
    Pmic_GpioCfg_t gpioCfg = {
        .validParams = PMIC_FUNCTIONALITY_VALID,
        .functionality = PMIC_NINT_GPI_FUNCTIONALITY_MAX + 1U
    };
    int32_t status = Pmic_gpioSetCfg(&pmicHandle, PMIC_NINT_GPI, &gpioCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_gpio_gpioSetCfg_nIntGpi_outOfBounds_polarity(void)
{
    // Pass out of bounds polarity for NINT_GPI pin
    Pmic_GpioCfg_t gpioCfg = {
        .validParams = PMIC_POLARITY_VALID,
        .polarity = PMIC_POLARITY_MAX + 1U
    };
    int32_t status = Pmic_gpioSetCfg(&pmicHandle, PMIC_NINT_GPI, &gpioCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_gpio_gpioSetCfg_nIntGpi_outOfBounds_puPdCfg(void)
{
    // Pass out of bounds puPdCfg for NINT_GPI pin
    Pmic_GpioCfg_t gpioCfg = {
        .validParams = PMIC_PU_PD_CFG_VALID,
        .puPdCfg = PMIC_PU_PD_CFG_MAX + 1U
    };
    int32_t status = Pmic_gpioSetCfg(&pmicHandle, PMIC_NINT_GPI, &gpioCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_gpio_gpioSetCfg_nIntGpi_outOfBounds_odPpCfg(void)
{
    // Pass out of bounds odPpCfg for NINT_GPI pin
    Pmic_GpioCfg_t gpioCfg = {
        .validParams = PMIC_OD_PP_CFG_VALID,
        .odPpCfg = PMIC_OD_PP_CFG_MAX + 1U
    };
    int32_t status = Pmic_gpioSetCfg(&pmicHandle, PMIC_NINT_GPI, &gpioCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/* ========================================================================== */
/*                    Negative Tests - Pmic_gpioGetCfg                        */
/* ========================================================================== */

void test_neg_gpio_gpioGetCfg_nullHandle(void)
{
    // Pass NULL pmicHandle into Pmic_gpioGetCfg()
    Pmic_GpioCfg_t gpioCfg = {
        .validParams = PMIC_FUNCTIONALITY_VALID
    };
    int32_t status = Pmic_gpioGetCfg(NULL, PMIC_GPIO, &gpioCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_gpio_gpioGetCfg_nullGpioCfg(void)
{
    // Pass NULL gpioCfg into Pmic_gpioGetCfg()
    int32_t status = Pmic_gpioGetCfg(&pmicHandle, PMIC_GPIO, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_gpio_gpioGetCfg_invalidGpioPin(void)
{
    // Pass invalid gpioPin into Pmic_gpioGetCfg()
    Pmic_GpioCfg_t gpioCfg = {
        .validParams = PMIC_FUNCTIONALITY_VALID
    };
    int32_t status = Pmic_gpioGetCfg(&pmicHandle, PMIC_GPIO_PIN_MAX + 1U, &gpioCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_gpio_gpioGetCfg_zeroValidParams(void)
{
    // Pass validParams = 0 into Pmic_gpioGetCfg()
    Pmic_GpioCfg_t gpioCfg = {
        .validParams = 0U
    };
    int32_t status = Pmic_gpioGetCfg(&pmicHandle, PMIC_GPIO, &gpioCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_gpio_gpioGetCfg_gpio_invalidValidParams(void)
{
    // Pass invalid validParams for GPIO pin (PU_PD_CFG_VALID not supported for GPIO)
    Pmic_GpioCfg_t gpioCfg = {
        .validParams = PMIC_PU_PD_CFG_VALID
    };
    int32_t status = Pmic_gpioGetCfg(&pmicHandle, PMIC_GPIO, &gpioCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_FAIL);
}

void test_neg_gpio_gpioGetCfg_nIntGpi_invalidValidParams(void)
{
    // Pass invalid validParams with no valid bits set (nothing to get)
    Pmic_GpioCfg_t gpioCfg = {
        .validParams = 0x00000000U  // No valid params - should fail
    };
    int32_t status = Pmic_gpioGetCfg(&pmicHandle, PMIC_NINT_GPI, &gpioCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/* ========================================================================== */
/*              Negative Tests - Activation State APIs                        */
/* ========================================================================== */

void test_neg_gpio_gpioSetActivationState_nullHandle(void)
{
    // Pass NULL pmicHandle into Pmic_gpioSetActivationState()
    int32_t status = Pmic_gpioSetActivationState(NULL, (bool)true);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_gpio_gpioActivate_nullHandle(void)
{
    // Pass NULL pmicHandle into Pmic_gpioActivate()
    int32_t status = Pmic_gpioActivate(NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_gpio_gpioDeactivate_nullHandle(void)
{
    // Pass NULL pmicHandle into Pmic_gpioDeactivate()
    int32_t status = Pmic_gpioDeactivate(NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_gpio_gpioGetActivationState_nullHandle(void)
{
    // Pass NULL pmicHandle into Pmic_gpioGetActivationState()
    bool activated = (bool)false;
    int32_t status = Pmic_gpioGetActivationState(NULL, &activated);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_gpio_gpioGetActivationState_nullActivated(void)
{
    // Pass NULL activated into Pmic_gpioGetActivationState()
    int32_t status = Pmic_gpioGetActivationState(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/* ========================================================================== */
/*              Positive Tests - PMIC_GPIO Pin Configuration                  */
/* ========================================================================== */

void test_pos_gpio_gpioSetGetCfg_gpio_functionality(void)
{
    /* Test all valid GPIO functionalities */
    Pmic_GpioCfg_t expCfg = {.validParams = PMIC_FUNCTIONALITY_VALID};
    Pmic_GpioCfg_t actCfg = {.validParams = PMIC_FUNCTIONALITY_VALID};
    int32_t status = PMIC_ST_SUCCESS;

    for (uint8_t func = PMIC_GPIO_INPUT; func <= PMIC_GPIO_FUNCTIONALITY_MAX; func++)
    {
        expCfg.functionality = func;
        status = Pmic_gpioSetCfg(&pmicHandle, PMIC_GPIO, &expCfg);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        status = Pmic_gpioGetCfg(&pmicHandle, PMIC_GPIO, &actCfg);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        PLATFORM_ASSERT(actCfg.functionality == func);
    }
}

void test_pos_gpio_gpioSetGetCfg_gpio_polarity(void)
{
    /* Test all valid GPIO polarities */
    Pmic_GpioCfg_t expCfg = {.validParams = PMIC_POLARITY_VALID};
    Pmic_GpioCfg_t actCfg = {.validParams = PMIC_POLARITY_VALID};
    int32_t status = PMIC_ST_SUCCESS;

    for (uint8_t polarity = PMIC_NORMAL_POLARITY; polarity <= PMIC_POLARITY_MAX; polarity++)
    {
        expCfg.polarity = polarity;
        status = Pmic_gpioSetCfg(&pmicHandle, PMIC_GPIO, &expCfg);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        status = Pmic_gpioGetCfg(&pmicHandle, PMIC_GPIO, &actCfg);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        PLATFORM_ASSERT(actCfg.polarity == polarity);
    }
}

void test_pos_gpio_gpioSetGetCfg_gpio_all_params(void)
{
    /* Test GPIO with all valid parameters simultaneously */
    Pmic_GpioCfg_t expCfg = {
        .validParams = (PMIC_FUNCTIONALITY_VALID | PMIC_POLARITY_VALID),
        .functionality = PMIC_GPIO_OUTPUT,
        .polarity = PMIC_INVERTED_POLARITY
    };
    Pmic_GpioCfg_t actCfg = {
        .validParams = (PMIC_FUNCTIONALITY_VALID | PMIC_POLARITY_VALID)
    };
    int32_t status = PMIC_ST_SUCCESS;

    status = Pmic_gpioSetCfg(&pmicHandle, PMIC_GPIO, &expCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_gpioGetCfg(&pmicHandle, PMIC_GPIO, &actCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(actCfg.functionality == PMIC_GPIO_OUTPUT);
    PLATFORM_ASSERT(actCfg.polarity == PMIC_INVERTED_POLARITY);
}

/* ========================================================================== */
/*            Positive Tests - PMIC_NINT_GPI Pin Configuration                */
/* ========================================================================== */

void test_pos_gpio_gpioSetGetCfg_nIntGpi_functionality(void)
{
    /* Test all valid NINT_GPI functionalities */
    Pmic_GpioCfg_t expCfg = {.validParams = PMIC_FUNCTIONALITY_VALID};
    Pmic_GpioCfg_t actCfg = {.validParams = PMIC_FUNCTIONALITY_VALID};
    int32_t status = PMIC_ST_SUCCESS;

    for (uint8_t func = PMIC_NINT_GPI_NINT; func <= PMIC_NINT_GPI_FUNCTIONALITY_MAX; func++)
    {
        expCfg.functionality = func;
        status = Pmic_gpioSetCfg(&pmicHandle, PMIC_NINT_GPI, &expCfg);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        status = Pmic_gpioGetCfg(&pmicHandle, PMIC_NINT_GPI, &actCfg);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        PLATFORM_ASSERT(actCfg.functionality == func);
    }
}

void test_pos_gpio_gpioSetGetCfg_nIntGpi_polarity(void)
{
    /* Test all valid NINT_GPI polarities */
    Pmic_GpioCfg_t expCfg = {.validParams = PMIC_POLARITY_VALID};
    Pmic_GpioCfg_t actCfg = {.validParams = PMIC_POLARITY_VALID};
    int32_t status = PMIC_ST_SUCCESS;

    for (uint8_t polarity = PMIC_NORMAL_POLARITY; polarity <= PMIC_POLARITY_MAX; polarity++)
    {
        expCfg.polarity = polarity;
        status = Pmic_gpioSetCfg(&pmicHandle, PMIC_NINT_GPI, &expCfg);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        status = Pmic_gpioGetCfg(&pmicHandle, PMIC_NINT_GPI, &actCfg);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        PLATFORM_ASSERT(actCfg.polarity == polarity);
    }
}

void test_pos_gpio_gpioSetGetCfg_nIntGpi_puPdCfg(void)
{
    /* Test all valid NINT_GPI pullup/pulldown configurations */
    Pmic_GpioCfg_t expCfg = {.validParams = PMIC_PU_PD_CFG_VALID};
    Pmic_GpioCfg_t actCfg = {.validParams = PMIC_PU_PD_CFG_VALID};
    int32_t status = PMIC_ST_SUCCESS;

    for (uint8_t puPdCfg = PMIC_PU_RESISTOR_ACTIVATED; puPdCfg <= PMIC_PU_PD_CFG_MAX; puPdCfg++)
    {
        expCfg.puPdCfg = puPdCfg;
        status = Pmic_gpioSetCfg(&pmicHandle, PMIC_NINT_GPI, &expCfg);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        status = Pmic_gpioGetCfg(&pmicHandle, PMIC_NINT_GPI, &actCfg);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        PLATFORM_ASSERT(actCfg.puPdCfg == puPdCfg);
    }
}

void test_pos_gpio_gpioSetGetCfg_nIntGpi_odPpCfg(void)
{
    /* Test all valid NINT_GPI open-drain/push-pull configurations */
    Pmic_GpioCfg_t expCfg = {.validParams = PMIC_OD_PP_CFG_VALID};
    Pmic_GpioCfg_t actCfg = {.validParams = PMIC_OD_PP_CFG_VALID};
    int32_t status = PMIC_ST_SUCCESS;

    for (uint8_t odPpCfg = PMIC_PUSH_PULL; odPpCfg <= PMIC_OD_PP_CFG_MAX; odPpCfg++)
    {
        expCfg.odPpCfg = odPpCfg;
        status = Pmic_gpioSetCfg(&pmicHandle, PMIC_NINT_GPI, &expCfg);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        status = Pmic_gpioGetCfg(&pmicHandle, PMIC_NINT_GPI, &actCfg);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        PLATFORM_ASSERT(actCfg.odPpCfg == odPpCfg);
    }
}

void test_pos_gpio_gpioSetGetCfg_nIntGpi_all_params(void)
{
    /* Test NINT_GPI with all valid parameters simultaneously */
    Pmic_GpioCfg_t expCfg = {
        .validParams = (PMIC_FUNCTIONALITY_VALID | PMIC_POLARITY_VALID |
                        PMIC_PU_PD_CFG_VALID | PMIC_OD_PP_CFG_VALID),
        .functionality = PMIC_NINT_GPI_WDG_TRIG_MODE_INPUT,
        .polarity = PMIC_INVERTED_POLARITY,
        .puPdCfg = PMIC_PD_RESISTOR_ACTIVATED,
        .odPpCfg = PMIC_OPEN_DRAIN
    };
    Pmic_GpioCfg_t actCfg = {
        .validParams = (PMIC_FUNCTIONALITY_VALID | PMIC_POLARITY_VALID |
                        PMIC_PU_PD_CFG_VALID | PMIC_OD_PP_CFG_VALID)
    };
    int32_t status = PMIC_ST_SUCCESS;

    status = Pmic_gpioSetCfg(&pmicHandle, PMIC_NINT_GPI, &expCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_gpioGetCfg(&pmicHandle, PMIC_NINT_GPI, &actCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(actCfg.functionality == PMIC_NINT_GPI_WDG_TRIG_MODE_INPUT);
    PLATFORM_ASSERT(actCfg.polarity == PMIC_INVERTED_POLARITY);
    PLATFORM_ASSERT(actCfg.puPdCfg == PMIC_PD_RESISTOR_ACTIVATED);
    PLATFORM_ASSERT(actCfg.odPpCfg == PMIC_OPEN_DRAIN);
}

/* ========================================================================== */
/*              Positive Tests - GPIO Activation State                        */
/* ========================================================================== */

void test_pos_gpio_gpioActivateDeactivate(void)
{
    /* Test GPIO activate/deactivate */
    bool isActivated = (bool)false;
    int32_t status = PMIC_ST_SUCCESS;

    status = Pmic_gpioDeactivate(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_gpioGetActivationState(&pmicHandle, &isActivated);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(isActivated == (bool)false);

    status = Pmic_gpioActivate(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_gpioGetActivationState(&pmicHandle, &isActivated);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(isActivated == (bool)true);
}

void test_pos_gpio_gpioSetActivationState(void)
{
    /* Test Pmic_gpioSetActivationState() with both activate and deactivate */
    bool isActivated = (bool)false;
    int32_t status = PMIC_ST_SUCCESS;

    status = Pmic_gpioSetActivationState(&pmicHandle, (bool)false);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_gpioGetActivationState(&pmicHandle, &isActivated);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(isActivated == (bool)false);

    status = Pmic_gpioSetActivationState(&pmicHandle, (bool)true);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_gpioGetActivationState(&pmicHandle, &isActivated);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(isActivated == (bool)true);
}

void test_pos_gpio_gpio_nIntGpi_repeatedFunctionality(void)
{
#ifdef BUILD_MOCK
    // Test NINT_GPI functionality value 3 (repeated value - lines 281-282)
    // Mock NINT_GPI_SEL register to return value 3
    int32_t status;
    Pmic_GpioCfg_t gpioCfg = {
        .validParams = PMIC_FUNCTIONALITY_VALID
    };

    // Inject NINT_GPI_SEL = 3 (PMIC_NINT_GPI_LPM_INPUT_REPEATED)
    // This should be converted to PMIC_NINT_GPI_LPM_INPUT
    testInject_setRegister(0x25U, 0x03U);  // FUNC_CONF reg (0x25), NINT_GPI_SEL is bits [1:0]

    status = Pmic_gpioGetCfg(&pmicHandle, PMIC_NINT_GPI, &gpioCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(gpioCfg.functionality == PMIC_NINT_GPI_LPM_CTRL_MODE_INPUT);
#else
    // Cannot test mock injection on hardware
    return;
#endif
}

