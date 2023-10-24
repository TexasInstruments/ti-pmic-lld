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
 *   \file    pmic_gpio_tps6522x.c
 *
 *   \brief   This file contains the TPS6522x BURTON PMIC GPIO Specific
 *            configuration API's and structures
 *
 */

#include "../../../include/pmic_types.h"
#include "../../../include/pmic_gpio.h"
#include "../../pmic_core_priv.h"
#include "../../pmic_io_priv.h"
#include "pmic_gpio_tps6522x_priv.h"

// clang-format off
/* PMIC GPIO Pins with Input Ouput Configuration */
static Pmic_GpioInOutCfg_t gTps6522x_gpioInOutCfg[] =
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
};

/* PMIC GPIO Interrupt Register array */
static Pmic_GpioIntRegCfg_t tps6522x_gpioIntRegCfg[] =
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
    }
};
// clang-format on

/**
 * \brief   Get TPS6522x GPIO config
 *          This function is used to get TPS6522x GPIO configuration
 *
 * \param   pGpioInOutCfg   [OUT]  to store TPS6522x gpio configuration
 */
void pmic_get_tps6522x_gpioInOutCfg(Pmic_GpioInOutCfg_t **pGpioInOutCfg)
{
    *pGpioInOutCfg = gTps6522x_gpioInOutCfg;
}

/**
 * \brief   Get TPS6522x GPIO Interrupt Register config
 *          This function is used to get TPS6522x GPIO Interrupt register
 *          configuration
 *
 * \param   pGpioIntRegCfg   [OUT]  to store tps6522x gpio register configuration
 */
void pmic_get_tps6522x_gpioIntRegCfg(Pmic_GpioIntRegCfg_t **pGpioIntRegCfg)
{
    *pGpioIntRegCfg = tps6522x_gpioIntRegCfg;
}
