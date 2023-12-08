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
 *            configuration APIs and structures
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

/**
 * \brief   Set TPS6522x EN/PB/VSENSE Pin Functionality
 *          This function is used to set the pin functionality of the EN/PB/VSENSE pin
 *          on TPS6522x
 *
 * \param   pPmicCoreHandle   [IN]    PMIC Interface Handle
 * \param   enPbVsenseCfg     [IN]    EN/PB/VSENSE configuration struct to set
 *                                    pin functionality of EN/PB/VSENSE pin
 *
 * \return  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
static int32_t Pmic_gpioTps6522xSetEnPbVsensePinFunc(Pmic_CoreHandle_t         *pPmicCoreHandle,
                                                     const Pmic_EnPbVsenseCfg_t enPbVsenseCfg)
{
    uint8_t regData = 0;
    int32_t status = PMIC_ST_SUCCESS;

    // Parameter validation
    if (enPbVsenseCfg.pinFuncSel > PMIC_EN_PB_VSENSE_FUNCTIONALITY_SELECT_VSENSE)
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    // Start critical section
    Pmic_criticalSectionStart(pPmicCoreHandle);

    // Read POWER_ON_CONFIG register
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_commIntf_recvByte(pPmicCoreHandle, POWER_ON_CONFIG_REGADDR, &regData);
    }

    // Modify register value and write back
    if (status == PMIC_ST_SUCCESS)
    {
        Pmic_setBitField(&regData,
                         PMIC_POWER_ON_CONFIG_EN_PB_VSENSE_CONFIG_SHIFT,
                         PMIC_POWER_ON_CONFIG_EN_PB_VSENSE_CONFIG_MASK,
                         enPbVsenseCfg.pinFuncSel);
        status = Pmic_commIntf_sendByte(pPmicCoreHandle, POWER_ON_CONFIG_REGADDR, regData);
    }

    // Stop critical section
    Pmic_criticalSectionStop(pPmicCoreHandle);

    return status;
}

/**
 * \brief   Set TPS6522x EN/PB/VSENSE deglitch configuration
 *          This function is used to set the deglitch configuration of the EN/PB/VSENSE pin
 *          on TPS6522x
 *
 * \param   pPmicCoreHandle   [IN]    PMIC Interface Handle
 * \param   enPbVsenseCfg     [IN]    EN/PB/VSENSE configuration struct to set
 *                                    deglitch configuration of EN/PB/VSENSE pin
 *
 * \return  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
static int32_t Pmic_gpioTps6522xSetEnPbVsenseDeglitch(Pmic_CoreHandle_t         *pPmicCoreHandle,
                                                      const Pmic_EnPbVsenseCfg_t enPbVsenseCfg)
{
    uint8_t regData = 0;
    int32_t status = PMIC_ST_SUCCESS;

    // Start critical section
    Pmic_criticalSectionStart(pPmicCoreHandle);

    // Read POWER_ON_CONFIG register
    status = Pmic_commIntf_recvByte(pPmicCoreHandle, POWER_ON_CONFIG_REGADDR, &regData);

    // Modify register value and write back
    if (status == PMIC_ST_SUCCESS)
    {
        Pmic_setBitField(&regData,
                         PMIC_POWER_ON_CONFIG_EN_PB_DEGL_SHIFT,
                         PMIC_POWER_ON_CONFIG_EN_PB_DEGL_MASK,
                         enPbVsenseCfg.deglitchSel);
        status = Pmic_commIntf_sendByte(pPmicCoreHandle, POWER_ON_CONFIG_REGADDR, regData);
    }

    // Stop critical section
    Pmic_criticalSectionStop(pPmicCoreHandle);

    return status;
}

/**
 * \brief   Function to get the pin functionality of the EN/PB/VSENSE pin
 *
 * \param   pPmicCoreHandle [IN]       PMIC Interface Handle
 * \param   pEnPbVsenseCfg  [OUT]      Pointer to store EN/PB/VSENSE
 *                                     pin functionality
 *
 * \return  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
static int32_t Pmic_gpioTps6522xGetEnPbVsensePinFunc(Pmic_CoreHandle_t    *pPmicCoreHandle,
                                                     Pmic_EnPbVsenseCfg_t *pEnPbVsenseCfg)
{
    uint8_t regData = 0;
    int32_t status = PMIC_ST_SUCCESS;

    // Start critical section
    Pmic_criticalSectionStart(pPmicCoreHandle);

    // Read POWER_ON_CONFIG register
    status = Pmic_commIntf_recvByte(pPmicCoreHandle, POWER_ON_CONFIG_REGADDR, &regData);

    // Extract functionality of EN/PB/VSENSE pin
    if (status == PMIC_ST_SUCCESS)
    {
        pEnPbVsenseCfg->pinFuncSel = Pmic_getBitField(
            regData, PMIC_POWER_ON_CONFIG_EN_PB_VSENSE_CONFIG_SHIFT, PMIC_POWER_ON_CONFIG_EN_PB_VSENSE_CONFIG_MASK);
    }

    // Stop critical section
    Pmic_criticalSectionStop(pPmicCoreHandle);

    return status;
}

/**
 * \brief   Function to get the deglitch configuration of the EN/PB/VSENSE pin
 *
 * \param   pPmicCoreHandle [IN]       PMIC Interface Handle
 * \param   pEnPbVsenseCfg  [OUT]      Pointer to store EN/PB/VSENSE
 *                                     deglitch configuration
 *
 * \return  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
static int32_t Pmic_gpioTps6522xGetEnPbVsenseDeglitch(Pmic_CoreHandle_t    *pPmicCoreHandle,
                                                      Pmic_EnPbVsenseCfg_t *pEnPbVsenseCfg)
{
    uint8_t regData = 0;
    int32_t status = PMIC_ST_SUCCESS;

    // Start critical section
    Pmic_criticalSectionStart(pPmicCoreHandle);

    // Read POWER_ON_CONFIG register
    status = Pmic_commIntf_recvByte(pPmicCoreHandle, POWER_ON_CONFIG_REGADDR, &regData);

    // Extract functionality of EN/PB/VSENSE pin
    if (status == PMIC_ST_SUCCESS)
    {
        pEnPbVsenseCfg->deglitchSel = (bool)Pmic_getBitField(
            regData, PMIC_POWER_ON_CONFIG_EN_PB_DEGL_SHIFT, PMIC_POWER_ON_CONFIG_EN_PB_DEGL_MASK);
    }

    // Stop critical section
    Pmic_criticalSectionStop(pPmicCoreHandle);

    return status;
}

/**
 * \brief   Function to set the configuration of EN/PB/VSENSE pin for TPS6522x
 *          BURTON PMIC
 *
 * \param   pPmicCoreHandle   [IN]    PMIC Interface Handle
 * \param   enPbVsenseCfg     [IN]    EN/PB/VSENSE configuration struct to set
 *                                    configuration of EN/PB/VSENSE pin
 *
 * \return  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_gpioTps6522xSetEnPbVsensePinConfiguration(Pmic_CoreHandle_t         *pPmicCoreHandle,
                                                       const Pmic_EnPbVsenseCfg_t enPbVsenseCfg)
{
    int32_t status = PMIC_ST_SUCCESS;

    // Set functionality of the EN/PB/VSENSE pin
    if (pmic_validParamCheck(enPbVsenseCfg.validParams, PMIC_EN_PB_VSENSE_CFG_FUNC_SEL_VALID) == true)
    {
        status = Pmic_gpioTps6522xSetEnPbVsensePinFunc(pPmicCoreHandle, enPbVsenseCfg);
    }

    // Set deglitch of the EN/PB/VSENSE pin
    if ((status == PMIC_ST_SUCCESS) &&
        pmic_validParamCheck(enPbVsenseCfg.validParams, PMIC_EN_PB_VSENSE_CFG_DEGLITCH_SEL_VALID) == true)
    {
        status = Pmic_gpioTps6522xSetEnPbVsenseDeglitch(pPmicCoreHandle, enPbVsenseCfg);
    }

    return status;
}

/**
 * \brief   Function to get the configuration of EN/PB/VSENSE pin for TPS6522x
 *          BURTON PMIC
 *
 * \param   pPmicCoreHandle [IN]       PMIC Interface Handle
 * \param   pEnPbVsenseCfg  [IN/OUT]   Pointer to store EN/PB/VSENSE
 *                                     pin configuration
 *
 * \return  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_gpioTps6522xGetEnPbVsensePinConfiguration(Pmic_CoreHandle_t    *pPmicCoreHandle,
                                                       Pmic_EnPbVsenseCfg_t *pEnPbVsenseCfg)
{
    int32_t status = PMIC_ST_SUCCESS;

    // Get functionality of the EN/PB/VSENSE pin
    if (pmic_validParamCheck(pEnPbVsenseCfg->validParams, PMIC_EN_PB_VSENSE_CFG_FUNC_SEL_VALID) == true)
    {
        status = Pmic_gpioTps6522xGetEnPbVsensePinFunc(pPmicCoreHandle, pEnPbVsenseCfg);
    }

    // Get deglitch of the EN/PB/VSENSE pin
    if ((status == PMIC_ST_SUCCESS) &&
        pmic_validParamCheck(pEnPbVsenseCfg->validParams, PMIC_EN_PB_VSENSE_CFG_DEGLITCH_SEL_VALID) == true)
    {
        status = Pmic_gpioTps6522xGetEnPbVsenseDeglitch(pPmicCoreHandle, pEnPbVsenseCfg);
    }

    return status;
}

int32_t Pmic_tps6522xGpioPinTypeADC(Pmic_CoreHandle_t *pPmicCoreHandle, const uint8_t gpioPin)
{
    int32_t status = PMIC_ST_SUCCESS;
    // fix
    const Pmic_GpioCfg_t adcGpioCfg = {.validParams = PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,
                                       .pinFunc = ((gpioPin == PMIC_TPS6522X_GPIO4_PIN) ?
                                                       PMIC_TPS6522X_GPIO_PINFUNC_GPIO4_ADC_IN :
                                                       PMIC_TPS6522X_GPIO_PINFUNC_GPIO5_ADC_IN)};

    // Parameter check
    if (pPmicCoreHandle == NULL)
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }
    if ((status == PMIC_ST_SUCCESS) && (pPmicCoreHandle->pPmic_SubSysInfo->adcEnable == false))
    {
        status = PMIC_ST_ERR_INV_DEVICE;
    }
    if ((status == PMIC_ST_SUCCESS) && (gpioPin != PMIC_TPS6522X_GPIO4_PIN) && (gpioPin != PMIC_TPS6522X_GPIO5_PIN))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    // Configure GPIO type to be ADC
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_gpioSetConfiguration(pPmicCoreHandle, gpioPin, adcGpioCfg);
    }

    return status;
}
