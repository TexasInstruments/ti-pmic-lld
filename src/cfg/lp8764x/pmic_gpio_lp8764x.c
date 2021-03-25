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
*   \file    pmic_gpio_lp8764x.c
*
*   \brief   This file contains the LP8764x Hera PMIC GPIO Specific
*            configuration API's and structures
*
*/

#include <pmic_types.h>
#include <pmic_gpio_lp8764x_priv.h>

/* PMIC GPIO Pins with Input Ouput Configuration */
static Pmic_GpioInOutCfg_t gLp8764x_gpioInOutCfg[] =
{
    {
        PMIC_GPIO1_CONF_REGADDR,
        PMIC_GPIO_OUT_1_REGADDR,
        PMIC_GPIO_IN_1_REGADDR,
        PMIC_GPIO_IN_1_GPIO1_IN_SHIFT,
        PMIC_GPIO_OUT_1_GPIO1_OUT_SHIFT
    },
    {
        PMIC_GPIO2_CONF_REGADDR,
        PMIC_GPIO_OUT_1_REGADDR,
        PMIC_GPIO_IN_1_REGADDR,
        PMIC_GPIO_IN_1_GPIO2_IN_SHIFT,
        PMIC_GPIO_OUT_1_GPIO2_OUT_SHIFT
    },
    {
        PMIC_GPIO3_CONF_REGADDR,
        PMIC_GPIO_OUT_1_REGADDR,
        PMIC_GPIO_IN_1_REGADDR,
        PMIC_GPIO_IN_1_GPIO3_IN_SHIFT,
        PMIC_GPIO_OUT_1_GPIO3_OUT_SHIFT
    },
    {
        PMIC_GPIO4_CONF_REGADDR,
        PMIC_GPIO_OUT_1_REGADDR,
        PMIC_GPIO_IN_1_REGADDR,
        PMIC_GPIO_IN_1_GPIO4_IN_SHIFT,
        PMIC_GPIO_OUT_1_GPIO4_OUT_SHIFT
    },
    {
        PMIC_GPIO5_CONF_REGADDR,
        PMIC_GPIO_OUT_1_REGADDR,
        PMIC_GPIO_IN_1_REGADDR,
        PMIC_GPIO_IN_1_GPIO5_IN_SHIFT,
        PMIC_GPIO_OUT_1_GPIO5_OUT_SHIFT
    },
    {
        PMIC_GPIO6_CONF_REGADDR,
        PMIC_GPIO_OUT_1_REGADDR,
        PMIC_GPIO_IN_1_REGADDR,
        PMIC_GPIO_IN_1_GPIO6_IN_SHIFT,
        PMIC_GPIO_OUT_1_GPIO6_OUT_SHIFT
    },
    {
        PMIC_GPIO7_CONF_REGADDR,
        PMIC_GPIO_OUT_1_REGADDR,
        PMIC_GPIO_IN_1_REGADDR,
        PMIC_GPIO_IN_1_GPIO7_IN_SHIFT,
        PMIC_GPIO_OUT_1_GPIO7_OUT_SHIFT
    },
    {
        PMIC_GPIO8_CONF_REGADDR,
        PMIC_GPIO_OUT_1_REGADDR,
        PMIC_GPIO_IN_1_REGADDR,
        PMIC_GPIO_IN_1_GPIO8_IN_SHIFT,
        PMIC_GPIO_OUT_1_GPIO8_OUT_SHIFT
    },
    {
        PMIC_GPIO9_CONF_REGADDR,
        PMIC_GPIO_OUT_2_REGADDR,
        PMIC_GPIO_IN_2_REGADDR,
        PMIC_GPIO_IN_2_GPIO9_IN_SHIFT,
        PMIC_GPIO_OUT_2_GPIO9_OUT_SHIFT
    },
    {
        PMIC_GPIO10_CONF_REGADDR,
        PMIC_GPIO_OUT_2_REGADDR,
        PMIC_GPIO_IN_2_REGADDR,
        PMIC_GPIO_IN_2_GPIO10_IN_SHIFT,
        PMIC_GPIO_OUT_2_GPIO10_OUT_SHIFT
    },
};

/* PMIC GPIO Interrupt Register array */
static Pmic_GpioIntRegCfg_t lp8764x_gpioIntRegCfg[] =
{
    {
        PMIC_FSM_TRIG_MASK_1_REGADDR,
        PMIC_FSM_TRIG_MASK_1_GPIO1_FSM_MASK_SHIFT,
        PMIC_FSM_TRIG_MASK_1_GPIO1_FSM_MASK_POL_SHIFT
    },
    {
        PMIC_FSM_TRIG_MASK_1_REGADDR,
        PMIC_FSM_TRIG_MASK_1_GPIO2_FSM_MASK_SHIFT,
        PMIC_FSM_TRIG_MASK_1_GPIO2_FSM_MASK_POL_SHIFT
    },
    {
        PMIC_FSM_TRIG_MASK_1_REGADDR,
        PMIC_FSM_TRIG_MASK_1_GPIO3_FSM_MASK_SHIFT,
        PMIC_FSM_TRIG_MASK_1_GPIO3_FSM_MASK_POL_SHIFT
    },
    {
        PMIC_FSM_TRIG_MASK_1_REGADDR,
        PMIC_FSM_TRIG_MASK_1_GPIO4_FSM_MASK_SHIFT,
        PMIC_FSM_TRIG_MASK_1_GPIO4_FSM_MASK_POL_SHIFT
    },
    {
        PMIC_FSM_TRIG_MASK_2_REGADDR,
        PMIC_FSM_TRIG_MASK_2_GPIO5_FSM_MASK_SHIFT,
        PMIC_FSM_TRIG_MASK_2_GPIO5_FSM_MASK_POL_SHIFT
    },
    {
        PMIC_FSM_TRIG_MASK_2_REGADDR,
        PMIC_FSM_TRIG_MASK_2_GPIO6_FSM_MASK_SHIFT,
        PMIC_FSM_TRIG_MASK_2_GPIO6_FSM_MASK_POL_SHIFT
    },
    {
        PMIC_FSM_TRIG_MASK_2_REGADDR,
        PMIC_FSM_TRIG_MASK_2_GPIO7_FSM_MASK_SHIFT,
        PMIC_FSM_TRIG_MASK_2_GPIO7_FSM_MASK_POL_SHIFT
    },
    {
        PMIC_FSM_TRIG_MASK_2_REGADDR,
        PMIC_FSM_TRIG_MASK_2_GPIO8_FSM_MASK_SHIFT,
        PMIC_FSM_TRIG_MASK_2_GPIO8_FSM_MASK_POL_SHIFT
    },
    {
        PMIC_FSM_TRIG_MASK_3_REGADDR,
        PMIC_FSM_TRIG_MASK_3_GPIO9_FSM_MASK_SHIFT,
        PMIC_FSM_TRIG_MASK_3_GPIO9_FSM_MASK_POL_SHIFT
    },
    {
        PMIC_FSM_TRIG_MASK_3_REGADDR,
        PMIC_FSM_TRIG_MASK_3_GPIO10_FSM_MASK_SHIFT,
        PMIC_FSM_TRIG_MASK_3_GPIO10_FSM_MASK_POL_SHIFT
    }
};

/*
 * \brief   Get LP8764x GPIO config
 *          This function is used to get LP8764x GPIO configuration
 *
 * \param   pGpioInOutCfg   [OUT]  to store lp8764x gpio configuration
 */
void pmic_get_lp8764x_gpioInOutCfg(Pmic_GpioInOutCfg_t **pGpioInOutCfg)
{
    *pGpioInOutCfg = gLp8764x_gpioInOutCfg;
}

/*
 * \brief   Get LP8764x GPIO Interrupt Register config
 *          This function is used to get LP8764x GPIO Interrupt register
 *          configuration
 *
 * \param   pGpioIntRegCfg   [OUT]  to store lp8764x gpio register configuration
 */
void pmic_get_lp8764x_gpioIntRegCfg(Pmic_GpioIntRegCfg_t **pGpioIntRegCfg)
{
    *pGpioIntRegCfg = lp8764x_gpioIntRegCfg;
}
