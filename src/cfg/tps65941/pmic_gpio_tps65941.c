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
*   \file    pmic_gpio_tps65941.c
*
*   \brief   This file contains the TPS65941 Leo PMIC GPIO Specific
*            configuration API's and structures
*
*/

#include <pmic_types.h>
#include <pmic_gpio_tps65941_priv.h>

/* PMIC GPIO Pins with Input Ouput Configuration */
static Pmic_GpioInOutCfg_t tps65941_gpioInOutCfg[] =
{
    {
        PMIC_NPWRON_CONF_REGADDR,
        PMIC_NPWRON_OUT_REG_UNAVAILABLE,
        PMIC_GPIO_IN_2_REGADDR,
        PMIC_GPIO_IN_2_NPWRON_IN_SHIFT,
        PMIC_NPWRON_OUT_REG_UNAVAILABLE
    },
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
    {
        PMIC_GPIO11_CONF_REGADDR,
        PMIC_GPIO_OUT_2_REGADDR,
        PMIC_GPIO_IN_2_REGADDR,
        PMIC_GPIO_IN_2_GPIO11_IN_SHIFT,
        PMIC_GPIO_OUT_2_GPIO11_OUT_SHIFT
    }
};

/*
 * \brief   Get TPS65941 GPIO config
 *          This function is used to get TPS65941 GPIO configuration
 *
 * \param   pGpioInOutCfg   [OUT]  to store tps65941 gpio configuration
 */
void pmic_get_tps65941_gpioInOutCfg(Pmic_GpioInOutCfg_t **pGpioInOutCfg)
{
    *pGpioInOutCfg = tps65941_gpioInOutCfg;
}
