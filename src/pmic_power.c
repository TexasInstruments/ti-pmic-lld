/******************************************************************************
 * Copyright (c) 2024 Texas Instruments Incorporated - http://www.ti.com
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
 * @file pmic_power.c
 *
 * @brief This file contains definitions to APIs that interact with PMIC
 * regulators and other related components.
 */
#include <stdint.h>

#include "pmic.h"
#include "pmic_io.h"
#include "pmic_power.h"

#include "regmap/power.h"
#include "regmap/core.h"

static int32_t PWR_getSpreadSpectrumCfg(const Pmic_CoreHandle_t *pmicHandle, Pmic_PwrBuckCfg_t *buckCfg)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    // Read CLK_CONF register
    if (Pmic_validParamCheck(buckCfg->validParams, PMIC_BUCK_SS_EN_VALID | PMIC_BUCK_SSM_SEL_VALID))
    {
        status = Pmic_ioRxByte_CS(pmicHandle, PMIC_CLK_CONF_REGADDR, &regData);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Extract SS_EN bit field
        if (Pmic_validParamCheck(buckCfg->validParams, PMIC_BUCK_SS_EN_VALID))
        {
            buckCfg->ssEn = Pmic_getBitField_b(regData, PMIC_SS_EN_SHIFT);
        }

        // Extract SSM_SEL bit field
        if (Pmic_validParamCheck(buckCfg->validParams, PMIC_BUCK_SSM_SEL_VALID))
        {
            buckCfg->ssmSel = Pmic_getBitField(regData, PMIC_SSM_SEL_SHIFT, PMIC_SSM_SEL_MASK);
        }
    }

    return status;
}

static int32_t PWR_getBuckVSET(const Pmic_CoreHandle_t *pmicHandle, Pmic_PwrBuckCfg_t *buckCfg)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    if (Pmic_validParamCheck(buckCfg->validParams, PMIC_BUCK_VSET_VALID))
    {
        // Read BUCK1_VOUT register
        status = Pmic_ioRxByte_CS(pmicHandle, PMIC_BUCK1_VOUT_REGADDR, &regData);

        // Extract BUCK1_VSET bit field
        if (status == PMIC_ST_SUCCESS)
        {
            buckCfg->vset = Pmic_getBitField(regData, PMIC_BUCK1_VSET_SHIFT, PMIC_BUCK1_VSET_MASK);
        }
    }

    return status;
}

static int32_t PWR_getBuckActiveLPwrVSET(const Pmic_CoreHandle_t *pmicHandle, Pmic_PwrBuckCfg_t *buckCfg)
{
    uint8_t regData = 0U, regAddr = 0U, bitShift = 0U, bitMask = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    if (Pmic_validParamCheck(buckCfg->validParams, PMIC_BUCK_VSET_ACTIVE_VALID))
    {
        if (buckCfg->resource == PMIC_BUCK2)
        {
            regAddr = PMIC_BUCK2_VOUT_ACTIVE_REGADDR;
            bitShift = PMIC_BUCK2_VSET_ACT_SHIFT;
            bitMask = PMIC_BUCK2_VSET_ACT_MASK;
        }
        else
        {
            regAddr = PMIC_BUCK3_VOUT_ACTIVE_REGADDR;
            bitShift = PMIC_BUCK3_VSET_ACT_SHIFT;
            bitMask = PMIC_BUCK3_VSET_ACT_MASK;
        }

        // Read BUCKx_VOUT_ACTIVE register
        status = Pmic_ioRxByte_CS(pmicHandle, regAddr, &regData);

        // Extract BUCKx_VSET_ACT bit field
        if (status == PMIC_ST_SUCCESS)
        {
            buckCfg->vsetActive = Pmic_getBitField(regData, bitShift, bitMask);
        }
    }

    if (Pmic_validParamCheck(buckCfg->validParams, PMIC_BUCK_VSET_LPWR_VALID))
    {
        if (buckCfg->resource == PMIC_BUCK2)
        {
            regAddr = PMIC_BUCK2_VOUT_LOWPWR_REGADDR;
            bitShift = PMIC_BUCK2_VSET_LPWR_SHIFT;
            bitMask = PMIC_BUCK2_VSET_LPWR_MASK;
        }
        else
        {
            regAddr = PMIC_BUCK3_VOUT_LOWPWR_REGADDR;
            bitShift = PMIC_BUCK3_VSET_LPWR_SHIFT;
            bitMask = PMIC_BUCK3_VSET_LPWR_MASK;
        }

        // Read BUCKx_VOUT_LOWPWR register
        status = Pmic_ioRxByte_CS(pmicHandle, regAddr, &regData);

        // Extract BUCKx_VSET_LPWR bit field
        if (status == PMIC_ST_SUCCESS)
        {
            buckCfg->vsetLPwr = Pmic_getBitField(regData, bitShift, bitMask);
        }
    }

    return status;
}

static int32_t PWR_getBuckVout(const Pmic_CoreHandle_t *pmicHandle, Pmic_PwrBuckCfg_t *buckCfg)
{
    int32_t status = PMIC_ST_SUCCESS;

    // BUCK1 does not support active VSET or LP VSET configurations
    const bool buck1InvalidParam = (buckCfg->resource == PMIC_BUCK1) &&
        Pmic_validParamCheck(buckCfg->validParams, PMIC_BUCK_VSET_ACTIVE_VALID | PMIC_BUCK_VSET_LPWR_VALID);

    // BUCK2 and BUCK3 do not support non-active and non-LP VSET configurations
    const bool buck2_3InvalidParam = (buckCfg->resource != PMIC_BUCK1) &&
        Pmic_validParamCheck(buckCfg->validParams, PMIC_BUCK_VSET_VALID);

    if (buck1InvalidParam || buck2_3InvalidParam)
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        if (buckCfg->resource == PMIC_BUCK1)
        {
            status = PWR_getBuckVSET(pmicHandle, buckCfg);
        }
        else
        {
            status = PWR_getBuckActiveLPwrVSET(pmicHandle, buckCfg);
        }
    }

    return status;
}

static int32_t PWR_getBuck1Ctrl(const Pmic_CoreHandle_t *pmicHandle, Pmic_PwrBuckCfg_t *buckCfg)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;
    const uint32_t buck1CtrlValidParams = PMIC_BUCK_HIGH_SIDE_SLEW_RATE_VALID | PMIC_BUCK_DISCHARGE_SEL_VALID |
        PMIC_BUCK_PLDN_EN_VALID | PMIC_BUCK_SLEW_RATE_VALID | PMIC_BUCK_FPWM_EN_VALID | PMIC_BUCK_ENABLE_VALID;

    if (Pmic_validParamCheck(buckCfg->validParams, buck1CtrlValidParams))
    {
        // Read BUCK1_CTRL register
        status = Pmic_ioRxByte_CS(pmicHandle ,PMIC_BUCK1_CTRL_REGADDR, &regData);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Extract BUCK1_EN_HS_ON_SR bit field
        if (Pmic_validParamCheck(buckCfg->validParams, PMIC_BUCK_HIGH_SIDE_SLEW_RATE_VALID))
        {
            buckCfg->highSideSlewRate = Pmic_getBitField(
                regData, PMIC_BUCK1_EN_HS_ON_SR_SHIFT, PMIC_BUCK1_EN_HS_ON_SR_MASK);
        }

        // Extract BUCK1_DISCHARGE_SEL bit field
        if (Pmic_validParamCheck(buckCfg->validParams, PMIC_BUCK_DISCHARGE_SEL_VALID))
        {
            buckCfg->dischargeSel = Pmic_getBitField(
                regData, PMIC_BUCK1_DISCHARGE_SEL_SHIFT, PMIC_BUCK1_DISCHARGE_SEL_MASK);
        }

        // Extract BUCK1_PLDN bit field
        if (Pmic_validParamCheck(buckCfg->validParams, PMIC_BUCK_PLDN_EN_VALID))
        {
            buckCfg->pldnEn = Pmic_getBitField_b(regData, PMIC_BUCK1_PLDN_SHIFT);
        }

        // Extract BUCK1_SLEW_RATE bit field
        if (Pmic_validParamCheck(buckCfg->validParams, PMIC_BUCK_SLEW_RATE_VALID))
        {
            buckCfg->slewRate = Pmic_getBitField(regData, PMIC_BUCK1_SLEW_RATE_SHIFT, PMIC_BUCK1_SLEW_RATE_MASK);
        }

        // Extract BUCK1_FPWM bit field
        if (Pmic_validParamCheck(buckCfg->validParams, PMIC_BUCK_FPWM_EN_VALID))
        {
            buckCfg->fpwmEn = Pmic_getBitField_b(regData, PMIC_BUCK1_FPWM_SHIFT);
        }

        // Extract BUCK1_EN bit field
        if (Pmic_validParamCheck(buckCfg->validParams, PMIC_BUCK_ENABLE_VALID))
        {
            buckCfg->enable = Pmic_getBitField_b(regData, PMIC_BUCK1_EN_SHIFT);
        }
    }

    return status;
}

static int32_t PWR_getBuck2_3Ctrl(const Pmic_CoreHandle_t *pmicHandle, Pmic_PwrBuckCfg_t *buckCfg)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;
    const uint32_t buck2_3CtrlValidParams = PMIC_BUCK_VMON_ONLY_VALID | PMIC_BUCK_DISCHARGE_SEL_VALID |
        PMIC_BUCK_PLDN_EN_VALID | PMIC_BUCK_SLEW_RATE_VALID | PMIC_BUCK_FPWM_EN_VALID | PMIC_BUCK_ENABLE_VALID;

    // Read BUCKx_CTRL (x=2 or x=3)
    if (Pmic_validParamCheck(buckCfg->validParams, buck2_3CtrlValidParams))
    {
        const uint8_t regAddr = (buckCfg->resource == PMIC_BUCK2) ? PMIC_BUCK2_CTRL_REGADDR : PMIC_BUCK3_CTRL_REGADDR;

        status = Pmic_ioRxByte_CS(pmicHandle, regAddr, &regData);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Extract BUCKx_VMON_ONLY (x=2 or x=3)
        if (Pmic_validParamCheck(buckCfg->validParams, PMIC_BUCK_VMON_ONLY_VALID))
        {
            buckCfg->vmonOnly = Pmic_getBitField_b(regData, PMIC_BUCK2_3_VMON_ONLY_SHIFT);
        }

        // Extract BUCKx_DISCHARGE_SEL (x=2 or x=3)
        if (Pmic_validParamCheck(buckCfg->validParams, PMIC_BUCK_DISCHARGE_SEL_VALID))
        {
            buckCfg->dischargeSel = Pmic_getBitField(regData, PMIC_BUCK2_3_DISCHARGE_SEL_SHIFT, PMIC_BUCK2_3_DISCHARGE_SEL_MASK);
        }

        // Extract BUCKx_PLDN (x=2 or x=3)
        if (Pmic_validParamCheck(buckCfg->validParams, PMIC_BUCK_PLDN_EN_VALID))
        {
            buckCfg->pldnEn = Pmic_getBitField_b(regData, PMIC_BUCK2_3_PLDN_SHIFT);
        }

        // Extract BUCKx_SLEW_RATE (x=2 or x=3)
        if (Pmic_validParamCheck(buckCfg->validParams, PMIC_BUCK_SLEW_RATE_VALID))
        {
            buckCfg->slewRate = Pmic_getBitField(regData, PMIC_BUCK2_3_SLEW_RATE_SHIFT, PMIC_BUCK2_3_SLEW_RATE_MASK);
        }

        // Extract BUCKx_FPWM (x=2 or x=3)
        if (Pmic_validParamCheck(buckCfg->validParams, PMIC_BUCK_FPWM_EN_VALID))
        {
            buckCfg->fpwmEn = Pmic_getBitField_b(regData, PMIC_BUCK2_3_FPWM_SHIFT);
        }

        // Extract BUCKx_EN (x=2 or x=3)
        if (Pmic_validParamCheck(buckCfg->validParams, PMIC_BUCK_ENABLE_VALID))
        {
            buckCfg->enable = Pmic_getBitField_b(regData, PMIC_BUCK2_3_EN_SHIFT);
        }
    }

    return status;
}

static int32_t PWR_getBuckCtrl(const Pmic_CoreHandle_t *pmicHandle, Pmic_PwrBuckCfg_t *buckCfg)
{
    int32_t status = PMIC_ST_SUCCESS;

    // BUCK1 does not have the VMON_ONLY configuration
    const bool buck1InvalidParam = (buckCfg->resource == PMIC_BUCK1) &&
        Pmic_validParamCheck(buckCfg->validParams, PMIC_BUCK_VMON_ONLY_VALID);

    // BUCK2 and BUCK3 do not have the EN_HS_ON_SR configuration
    const bool buck2_3InvalidParam = (buckCfg->resource != PMIC_BUCK1) &&
        Pmic_validParamCheck(buckCfg->validParams, PMIC_BUCK_HIGH_SIDE_SLEW_RATE_VALID);

    if (buck1InvalidParam || buck2_3InvalidParam)
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        if (buckCfg->resource == PMIC_BUCK1)
        {
            status = PWR_getBuck1Ctrl(pmicHandle, buckCfg);
        }
        else
        {
            status = PWR_getBuck2_3Ctrl(pmicHandle, buckCfg);
        }
    }

    return status;
}

static int32_t PWR_getBuckMonConf(const Pmic_CoreHandle_t *pmicHandle, Pmic_PwrBuckCfg_t *buckCfg)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U, regAddr = 0U;
    uint32_t buckMonConfValidParams = PMIC_BUCK_UV_THR_VALID | PMIC_BUCK_OV_THR_VALID |
        PMIC_BUCK_RV_CONF_VALID | PMIC_BUCK_ILIM_SEL_VALID | PMIC_BUCK_DEGLITCH_SEL_VALID;

    // Read BUCKx_MON_CONF (x=1, x=2, or x=3)
    if (Pmic_validParamCheck(buckCfg->validParams, buckMonConfValidParams))
    {
        if (buckCfg->resource == PMIC_BUCK1)
        {
            regAddr = PMIC_BUCK1_MON_CONF_REGADDR;
        }
        else if (buckCfg->resource == PMIC_BUCK2)
        {
            regAddr = PMIC_BUCK2_MON_CONF_REGADDR;
        }
        else
        {
            regAddr = PMIC_BUCK3_MON_CONF_REGADDR;
        }

        status = Pmic_ioRxByte_CS(pmicHandle, regAddr, &regData);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Extract BUCKx_UV_THR bit field (x=1, x=2, or x=3)
        if (Pmic_validParamCheck(buckCfg->validParams, PMIC_BUCK_UV_THR_VALID))
        {
            buckCfg->uvThr = Pmic_getBitField(regData, PMIC_BUCK_UV_THR_SHIFT, PMIC_BUCK_UV_THR_MASK);
        }

        // Extract BUCKx_OV_THR bit field (x=1, x=2, or x=3)
        if (Pmic_validParamCheck(buckCfg->validParams, PMIC_BUCK_OV_THR_VALID))
        {
            buckCfg->ovThr = Pmic_getBitField(regData, PMIC_BUCK_OV_THR_SHIFT, PMIC_BUCK_OV_THR_MASK);
        }

        // Extract BUCKx_RV_CONF bit field (x=1, x=2, or x=3)
        if (Pmic_validParamCheck(buckCfg->validParams, PMIC_BUCK_RV_CONF_VALID))
        {
            buckCfg->rvConf = Pmic_getBitField(regData, PMIC_BUCK_RV_CONF_SHIFT, PMIC_BUCK_RV_CONF_MASK);
        }

        // Extract BUCKx_ILIM_SEL bit field (x=1, x=2, or x=3)
        if (Pmic_validParamCheck(buckCfg->validParams, PMIC_BUCK_ILIM_SEL_VALID))
        {
            buckCfg->ilimSel = Pmic_getBitField(regData, PMIC_BUCK_ILIM_SEL_SHIFT, PMIC_BUCK_ILIM_SEL_MASK);
        }

        // Extract BUCKx_DEGLITCH_SEL bit field (x=1, x=2, or x=3)
        if (Pmic_validParamCheck(buckCfg->validParams, PMIC_BUCK_DEGLITCH_SEL_VALID))
        {
            buckCfg->deglitchSel = Pmic_getBitField(regData, PMIC_BUCK_DEGLITCH_SEL_SHIFT, PMIC_BUCK_DEGLITCH_SEL_MASK);
        }
    }

    return status;
}

static int32_t PWR_getBuckOvpResponse(const Pmic_CoreHandle_t *pmicHandle, Pmic_PwrBuckCfg_t *buckCfg)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U, bitShift = 0U, bitMask = 0U;

    if (Pmic_validParamCheck(buckCfg->validParams, PMIC_BUCK_OVP_SEL_VALID))
    {
        if (buckCfg->resource == PMIC_BUCK1)
        {
            bitShift = PMIC_BUCK1_OVP_SEL_SHIFT;
            bitMask = PMIC_BUCK1_OVP_SEL_MASK;
        }
        else if (buckCfg->resource == PMIC_BUCK2)
        {
            bitShift = PMIC_BUCK2_OVP_SEL_SHIFT;
            bitMask = PMIC_BUCK2_OVP_SEL_MASK;
        }
        else
        {
            bitShift = PMIC_BUCK3_OVP_SEL_SHIFT;
            bitMask = PMIC_BUCK3_OVP_SEL_MASK;
        }

        // Read REG_OVP_CONF register
        status = Pmic_ioRxByte_CS(pmicHandle, PMIC_REG_OVP_CONF_REGADDR, &regData);

        // Extract buck OVP response
        if (status == PMIC_ST_SUCCESS)
        {
            buckCfg->ovpSel = Pmic_getBitField(regData, bitShift, bitMask);
        }
    }

    return status;
}

static int32_t PWR_getBuckOvResponse(const Pmic_CoreHandle_t *pmicHandle, Pmic_PwrBuckCfg_t *buckCfg)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U, bitShift = 0U, bitMask = 0U;

    if (Pmic_validParamCheck(buckCfg->validParams, PMIC_BUCK_OV_SEL_VALID))
    {
        if (buckCfg->resource == PMIC_BUCK1)
        {
            bitShift = PMIC_BUCK1_OV_SEL_SHIFT;
            bitMask = PMIC_BUCK1_OV_SEL_MASK;
        }
        else if (buckCfg->resource == PMIC_BUCK2)
        {
            bitShift = PMIC_BUCK2_OV_SEL_SHIFT;
            bitMask = PMIC_BUCK2_OV_SEL_MASK;
        }
        else
        {
            bitShift = PMIC_BUCK3_OV_SEL_SHIFT;
            bitMask = PMIC_BUCK3_OV_SEL_MASK;
        }

        // Read REG_OV_CONF register
        status = Pmic_ioRxByte_CS(pmicHandle, PMIC_REG_OV_CONF_REGADDR, &regData);

        // Extract buck OV response
        if (status == PMIC_ST_SUCCESS)
        {
            buckCfg->ovSel = Pmic_getBitField(regData, bitShift, bitMask);
        }
    }

    return status;
}

static int32_t PWR_getBuckUvResponse(const Pmic_CoreHandle_t *pmicHandle, Pmic_PwrBuckCfg_t *buckCfg)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U, bitShift = 0U, bitMask = 0U;

    if (Pmic_validParamCheck(buckCfg->validParams, PMIC_BUCK_UV_SEL_VALID))
    {
        if (buckCfg->resource == PMIC_BUCK1)
        {
            bitShift = PMIC_BUCK1_UV_SEL_SHIFT;
            bitMask = PMIC_BUCK1_UV_SEL_MASK;
        }
        else if (buckCfg->resource == PMIC_BUCK2)
        {
            bitShift = PMIC_BUCK2_UV_SEL_SHIFT;
            bitMask = PMIC_BUCK2_UV_SEL_MASK;
        }
        else
        {
            bitShift = PMIC_BUCK3_UV_SEL_SHIFT;
            bitMask = PMIC_BUCK3_UV_SEL_MASK;
        }

        // Read REG_UV_CONF register
        status = Pmic_ioRxByte_CS(pmicHandle, PMIC_REG_UV_CONF_REGADDR, &regData);

        // Extract buck UV response
        if (status == PMIC_ST_SUCCESS)
        {
            buckCfg->uvSel = Pmic_getBitField(regData, bitShift, bitMask);
        }
    }

    return status;
}

static int32_t PWR_getBuckScResponse(const Pmic_CoreHandle_t *pmicHandle, Pmic_PwrBuckCfg_t *buckCfg)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U, bitShift = 0U, bitMask = 0U;

    if (Pmic_validParamCheck(buckCfg->validParams, PMIC_BUCK_SC_SEL_VALID))
    {
        if (buckCfg->resource == PMIC_BUCK1)
        {
            bitShift = PMIC_BUCK1_SC_SEL_SHIFT;
            bitMask = PMIC_BUCK1_SC_SEL_MASK;
        }
        else if (buckCfg->resource == PMIC_BUCK2)
        {
            bitShift = PMIC_BUCK2_SC_SEL_SHIFT;
            bitMask = PMIC_BUCK2_SC_SEL_MASK;
        }
        else
        {
            bitShift = PMIC_BUCK3_SC_SEL_SHIFT;
            bitMask = PMIC_BUCK3_SC_SEL_MASK;
        }

        // Read REG_SC_CONF register
        status = Pmic_ioRxByte_CS(pmicHandle, PMIC_REG_SC_CONF_REGADDR, &regData);

        // Extract buck SC response
        if (status == PMIC_ST_SUCCESS)
        {
            buckCfg->scSel = Pmic_getBitField(regData, bitShift, bitMask);
        }
    }

    return status;
}

static int32_t PWR_getBuckFaultResponses(const Pmic_CoreHandle_t *pmicHandle, Pmic_PwrBuckCfg_t *buckCfg)
{
    // Get buck OVP fault response
    int32_t status = PWR_getBuckOvpResponse(pmicHandle, buckCfg);

    // Get buck OV fault response
    if (status == PMIC_ST_SUCCESS)
    {
        status = PWR_getBuckOvResponse(pmicHandle, buckCfg);
    }

    // Get buck UV fault response
    if (status == PMIC_ST_SUCCESS)
    {
        status = PWR_getBuckUvResponse(pmicHandle, buckCfg);
    }

    // Get buck SC fault response
    if (status == PMIC_ST_SUCCESS)
    {
        status = PWR_getBuckScResponse(pmicHandle, buckCfg);
    }

    return status;
}

static int32_t PWR_getBuck1Uvlo(const Pmic_CoreHandle_t *pmicHandle, Pmic_PwrBuckCfg_t *buckCfg)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    // Read BUCK1_UVLO register
    if (Pmic_validParamCheck(buckCfg->validParams, PMIC_BUCK_UVLO_FALLING_VALID | PMIC_BUCK_UVLO_RISING_VALID))
    {
        if (buckCfg->resource == PMIC_BUCK1)
        {
            status = Pmic_ioRxByte_CS(pmicHandle, PMIC_BUCK1_UVLO_REGADDR, &regData);
        }
        else
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Extract BUCK1_UVLO_FALLING bit field
        if (Pmic_validParamCheck(buckCfg->validParams, PMIC_BUCK_UVLO_FALLING_VALID))
        {
            buckCfg->uvloFalling = Pmic_getBitField(
                regData, PMIC_BUCK1_UVLO_FALLING_SHIFT, PMIC_BUCK1_UVLO_FALLING_MASK);
        }

        // Extract BUCK1_UVLO_RISING bit field
        if (Pmic_validParamCheck(buckCfg->validParams, PMIC_BUCK_UVLO_RISING_VALID))
        {
            buckCfg->uvloRising = Pmic_getBitField(
                regData, PMIC_BUCK1_UVLO_RISING_SHIFT, PMIC_BUCK1_UVLO_RISING_MASK);
        }
    }

    return status;
}

int32_t Pmic_pwrGetBuckCfg(const Pmic_CoreHandle_t *pmicHandle, Pmic_PwrBuckCfg_t *buckCfg)
{
    int32_t status = Pmic_checkPmicCoreHandle(pmicHandle);

    if ((status == PMIC_ST_SUCCESS) && (buckCfg == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) && ((buckCfg->resource > PMIC_BUCK_MAX) || (buckCfg->validParams == 0U)))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    // Get spread spectrum configuration
    if (status == PMIC_ST_SUCCESS)
    {
        status = PWR_getSpreadSpectrumCfg(pmicHandle, buckCfg);
    }

    // Read buck VOUT register
    if (status == PMIC_ST_SUCCESS)
    {
        status = PWR_getBuckVout(pmicHandle, buckCfg);
    }

    // Read BUCKx_CTRL register
    if (status == PMIC_ST_SUCCESS)
    {
        status = PWR_getBuckCtrl(pmicHandle, buckCfg);
    }

    // Read BUCKx_MON_CONF register
    if (status == PMIC_ST_SUCCESS)
    {
        status = PWR_getBuckMonConf(pmicHandle, buckCfg);
    }

    // Get buck fault responses
    if (status == PMIC_ST_SUCCESS)
    {
        status = PWR_getBuckFaultResponses(pmicHandle, buckCfg);
    }

    // Get BUCK1_UVLO
    if (status == PMIC_ST_SUCCESS)
    {
        status = PWR_getBuck1Uvlo(pmicHandle, buckCfg);
    }

    return status;
}

static int32_t PWR_setSpreadSpectrumCfg(const Pmic_CoreHandle_t *pmicHandle, const Pmic_PwrBuckCfg_t *buckCfg)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    // Read CLK_CONF register
    Pmic_criticalSectionStart(pmicHandle);
    if (Pmic_validParamCheck(buckCfg->validParams, PMIC_BUCK_SS_EN_VALID | PMIC_BUCK_SSM_SEL_VALID))
    {
        status = Pmic_ioRxByte(pmicHandle, PMIC_CLK_CONF_REGADDR, &regData);
    }

    // Modify SSM_SEL bit field
    if (Pmic_validParamStatusCheck(buckCfg->validParams, PMIC_BUCK_SSM_SEL_VALID, status))
    {
        if (buckCfg->ssmSel > PMIC_SSM_SEL_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
        else
        {
            Pmic_setBitField(&regData, PMIC_SSM_SEL_SHIFT, PMIC_SSM_SEL_MASK, buckCfg->ssmSel);
        }
    }

    // Modify SS_EN bit field
    if (Pmic_validParamStatusCheck(buckCfg->validParams, PMIC_BUCK_SS_EN_VALID, status))
    {
        Pmic_setBitField_b(&regData, PMIC_SS_EN_SHIFT, PMIC_SS_EN_MASK, buckCfg->ssEn);
    }

    // Write new register value back to PMIC
    if (Pmic_validParamStatusCheck(buckCfg->validParams, PMIC_BUCK_SS_EN_VALID | PMIC_BUCK_SSM_SEL_VALID, status))
    {
        status = Pmic_ioTxByte(pmicHandle, PMIC_CLK_CONF_REGADDR, regData);
    }
    Pmic_criticalSectionStop(pmicHandle);

    return status;
}

static int32_t PWR_setBuckVSET(const Pmic_CoreHandle_t *pmicHandle, const Pmic_PwrBuckCfg_t *buckCfg)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    if (Pmic_validParamCheck(buckCfg->validParams, PMIC_BUCK_VSET_VALID))
    {
        if (buckCfg->vset > PMIC_BUCK1_VSET_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }

        // Read BUCK1_VOUT register
        Pmic_criticalSectionStart(pmicHandle);
        if (status == PMIC_ST_SUCCESS)
        {
            status = Pmic_ioRxByte(pmicHandle, PMIC_BUCK1_VOUT_REGADDR, &regData);
        }

        if (status == PMIC_ST_SUCCESS)
        {
            // Modify BUCK1_VSET bit field
            Pmic_setBitField(&regData, PMIC_BUCK1_VSET_SHIFT, PMIC_BUCK1_VSET_MASK, buckCfg->vset);

            // Write new register value back to PMIC
            status = Pmic_ioTxByte(pmicHandle, PMIC_BUCK1_VOUT_REGADDR, regData);
        }
        Pmic_criticalSectionStop(pmicHandle);
    }

    return status;
}

static int32_t PWR_setBuckActiveVSET(const Pmic_CoreHandle_t *pmicHandle, const Pmic_PwrBuckCfg_t *buckCfg)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U, regAddr = 0U, bitShift = 0U, bitMask = 0U;

    if (buckCfg->resource == PMIC_BUCK2)
    {
        regAddr = PMIC_BUCK2_VOUT_ACTIVE_REGADDR;
        bitShift = PMIC_BUCK2_VSET_ACT_SHIFT;
        bitMask = PMIC_BUCK2_VSET_ACT_MASK;
    }
    else
    {
        regAddr = PMIC_BUCK3_VOUT_ACTIVE_REGADDR;
        bitShift = PMIC_BUCK3_VSET_ACT_SHIFT;
        bitMask = PMIC_BUCK3_VSET_ACT_MASK;
    }

    // read BUCKx_VOUT_ACTIVE register (x=2 or x=3)
    Pmic_criticalSectionStart(pmicHandle);
    status = Pmic_ioRxByte(pmicHandle, regAddr, &regData);

    // Modify BUCKx_VSET_ACT bit field (x=2 or x=3)
    if (status == PMIC_ST_SUCCESS)
    {
        if ((buckCfg->vsetActive < PMIC_BUCK2_3_VSET_MIN) || (buckCfg->vsetActive > PMIC_BUCK2_3_VSET_MAX))
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
        else
        {
            Pmic_setBitField(&regData, bitShift, bitMask, buckCfg->vsetActive);
        }
    }

    // Write new register value back to PMIC
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioTxByte(pmicHandle, regAddr, regData);
    }
    Pmic_criticalSectionStop(pmicHandle);

    return status;
}

static int32_t PWR_setBuckLPwrVSET(const Pmic_CoreHandle_t *pmicHandle, const Pmic_PwrBuckCfg_t *buckCfg)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U, regAddr = 0U, bitShift = 0U, bitMask = 0U;

    if (buckCfg->resource == PMIC_BUCK2)
    {
        regAddr = PMIC_BUCK2_VOUT_LOWPWR_REGADDR;
        bitShift = PMIC_BUCK2_VSET_LPWR_SHIFT;
        bitMask = PMIC_BUCK2_VSET_LPWR_MASK;
    }
    else
    {
        regAddr = PMIC_BUCK3_VOUT_LOWPWR_REGADDR;
        bitShift = PMIC_BUCK3_VSET_LPWR_SHIFT;
        bitMask = PMIC_BUCK3_VSET_LPWR_MASK;
    }

    // read BUCKx_VOUT_LOWPWR register (x=2 or x=3)
    Pmic_criticalSectionStart(pmicHandle);
    status = Pmic_ioRxByte(pmicHandle, regAddr, &regData);

    // Modify BUCKx_VSET_LPWR bit field (x=2 or x=3)
    if (status == PMIC_ST_SUCCESS)
    {
        if ((buckCfg->vsetLPwr < PMIC_BUCK2_3_VSET_MIN) || (buckCfg->vsetLPwr > PMIC_BUCK2_3_VSET_MAX))
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
        else
        {
            Pmic_setBitField(&regData, bitShift, bitMask, buckCfg->vsetLPwr);
        }
    }

    // Write new register value back to PMIC
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioTxByte(pmicHandle, regAddr, regData);
    }
    Pmic_criticalSectionStop(pmicHandle);

    return status;
}

static int32_t PWR_setBuckActiveLPwrVSET(const Pmic_CoreHandle_t *pmicHandle, const Pmic_PwrBuckCfg_t *buckCfg)
{
    int32_t status = PMIC_ST_SUCCESS;

    if (Pmic_validParamCheck(buckCfg->validParams, PMIC_BUCK_VSET_ACTIVE_VALID))
    {
        status = PWR_setBuckActiveVSET(pmicHandle, buckCfg);
    }

    if (Pmic_validParamStatusCheck(buckCfg->validParams, PMIC_BUCK_VSET_LPWR_VALID, status))
    {
        status = PWR_setBuckLPwrVSET(pmicHandle, buckCfg);
    }

    return status;
}

static int32_t PWR_setBuckVout(const Pmic_CoreHandle_t *pmicHandle, const Pmic_PwrBuckCfg_t *buckCfg)
{
    int32_t status = PMIC_ST_SUCCESS;

    // BUCK1 does not support active VSET or LP VSET configurations
    const bool buck1InvalidParam = (buckCfg->resource == PMIC_BUCK1) &&
        Pmic_validParamCheck(buckCfg->validParams, PMIC_BUCK_VSET_ACTIVE_VALID | PMIC_BUCK_VSET_LPWR_VALID);

    // BUCK2 and BUCK3 do not support non-active and non-LP VSET configurations
    const bool buck2_3InvalidParam = (buckCfg->resource != PMIC_BUCK1) &&
        Pmic_validParamCheck(buckCfg->validParams, PMIC_BUCK_VSET_VALID);

    if (buck1InvalidParam || buck2_3InvalidParam)
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        if (buckCfg->resource == PMIC_BUCK1)
        {
            status = PWR_setBuckVSET(pmicHandle, buckCfg);
        }
        else
        {
            status = PWR_setBuckActiveLPwrVSET(pmicHandle, buckCfg);
        }
    }

    return status;
}

static int32_t PWR_setBuck1Ctrl(const Pmic_CoreHandle_t *pmicHandle, const Pmic_PwrBuckCfg_t *buckCfg)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;
    const uint32_t buck1CtrlValidParams = PMIC_BUCK_HIGH_SIDE_SLEW_RATE_VALID | PMIC_BUCK_DISCHARGE_SEL_VALID |
        PMIC_BUCK_PLDN_EN_VALID | PMIC_BUCK_SLEW_RATE_VALID | PMIC_BUCK_FPWM_EN_VALID | PMIC_BUCK_ENABLE_VALID;

    // Read BUCK1_CTRL register
    Pmic_criticalSectionStart(pmicHandle);
    if (Pmic_validParamCheck(buckCfg->validParams, buck1CtrlValidParams))
    {
        status = Pmic_ioRxByte(pmicHandle, PMIC_BUCK1_CTRL_REGADDR, &regData);
    }

    // Modify BUCK1_EN_HS_ON_SR bit field
    if (Pmic_validParamStatusCheck(buckCfg->validParams, PMIC_BUCK_HIGH_SIDE_SLEW_RATE_VALID, status))
    {
        if (buckCfg->highSideSlewRate > PMIC_BUCK1_EN_HS_ON_SR_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
        else
        {
            Pmic_setBitField(
                &regData, PMIC_BUCK1_EN_HS_ON_SR_SHIFT, PMIC_BUCK1_EN_HS_ON_SR_MASK, buckCfg->highSideSlewRate);
        }
    }

    // Modify BUCK1_DISCHARGE_SEL bit field
    if (Pmic_validParamStatusCheck(buckCfg->validParams, PMIC_BUCK_DISCHARGE_SEL_VALID, status))
    {
        if (buckCfg->dischargeSel > PMIC_BUCK_DISCHARGE_SEL_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
        else
        {
            Pmic_setBitField(
                &regData, PMIC_BUCK1_DISCHARGE_SEL_SHIFT, PMIC_BUCK1_DISCHARGE_SEL_MASK, buckCfg->dischargeSel);
        }
    }

    // Modify BUCK1_PLDN bit field
    if (Pmic_validParamStatusCheck(buckCfg->validParams, PMIC_BUCK_PLDN_EN_VALID, status))
    {
        Pmic_setBitField_b(&regData, PMIC_BUCK1_PLDN_SHIFT, PMIC_BUCK1_PLDN_MASK, buckCfg->pldnEn);
    }

    // Modify BUCK1_SLEW_RATE bit field
    if (Pmic_validParamStatusCheck(buckCfg->validParams, PMIC_BUCK_SLEW_RATE_VALID, status))
    {
        if (buckCfg->slewRate > PMIC_BUCK_SLEW_RATE_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
        else
        {
            Pmic_setBitField(
                &regData, PMIC_BUCK1_SLEW_RATE_SHIFT, PMIC_BUCK1_SLEW_RATE_MASK, buckCfg->slewRate);
        }
    }

    // Modify BUCK1_FPWM bit field
    if (Pmic_validParamStatusCheck(buckCfg->validParams, PMIC_BUCK_FPWM_EN_VALID, status))
    {
        Pmic_setBitField_b(&regData, PMIC_BUCK1_FPWM_SHIFT, PMIC_BUCK1_FPWM_MASK, buckCfg->fpwmEn);
    }

    // Modify BUCK1_EN bit field
    if (Pmic_validParamStatusCheck(buckCfg->validParams, PMIC_BUCK_ENABLE_VALID, status))
    {
        Pmic_setBitField_b(&regData, PMIC_BUCK1_EN_SHIFT, PMIC_BUCK1_EN_MASK, buckCfg->enable);
    }

    // Write new register value back to PMIC
    if (Pmic_validParamStatusCheck(buckCfg->validParams, buck1CtrlValidParams, status))
    {
        status = Pmic_ioTxByte(pmicHandle, PMIC_BUCK1_CTRL_REGADDR, regData);
    }
    Pmic_criticalSectionStop(pmicHandle);

    return status;
}

static int32_t PWR_setBuck2_3Ctrl(const Pmic_CoreHandle_t *pmicHandle, const Pmic_PwrBuckCfg_t *buckCfg)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U, regAddr = 0U;
    const uint32_t buck2_3CtrlValidParams = PMIC_BUCK_VMON_ONLY_VALID | PMIC_BUCK_DISCHARGE_SEL_VALID |
        PMIC_BUCK_PLDN_EN_VALID | PMIC_BUCK_SLEW_RATE_VALID | PMIC_BUCK_FPWM_EN_VALID | PMIC_BUCK_ENABLE_VALID;

    // Read BUCKx_CTRL register (x=2 or x=3)
    Pmic_criticalSectionStart(pmicHandle);
    if (Pmic_validParamCheck(buckCfg->validParams, buck2_3CtrlValidParams))
    {
        regAddr = (buckCfg->resource == PMIC_BUCK2) ? PMIC_BUCK2_CTRL_REGADDR : PMIC_BUCK3_CTRL_REGADDR;
        status = Pmic_ioRxByte(pmicHandle, regAddr, &regData);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Modify BUCKx_VMON_ONLY bit field (x=2 or x=3)
        if (Pmic_validParamCheck(buckCfg->validParams, PMIC_BUCK_VMON_ONLY_VALID))
        {
            Pmic_setBitField_b(
                &regData, PMIC_BUCK2_3_VMON_ONLY_SHIFT, PMIC_BUCK2_3_VMON_ONLY_MASK, buckCfg->vmonOnly);
        }

        // Modify BUCKx_DISCHARGE_SEL bit field (x=2 or x=3)
        if (Pmic_validParamStatusCheck(buckCfg->validParams, PMIC_BUCK_DISCHARGE_SEL_VALID, status))
        {
            if (buckCfg->dischargeSel > PMIC_BUCK_DISCHARGE_SEL_MAX)
            {
                status = PMIC_ST_ERR_INV_PARAM;
            }
            else
            {
                Pmic_setBitField(
                    &regData, PMIC_BUCK2_3_DISCHARGE_SEL_SHIFT, PMIC_BUCK2_3_DISCHARGE_SEL_MASK, buckCfg->dischargeSel);
            }
        }

        // Modify BUCKx_PLDN bit field (x=2 or x=3)
        if (Pmic_validParamStatusCheck(buckCfg->validParams, PMIC_BUCK_PLDN_EN_VALID, status))
        {
            Pmic_setBitField_b(&regData, PMIC_BUCK2_3_PLDN_SHIFT, PMIC_BUCK2_3_PLDN_MASK, buckCfg->pldnEn);
        }

        // Modify BUCKx_SLEW_RATE bit field (x=2 or x=3)
        if (Pmic_validParamStatusCheck(buckCfg->validParams, PMIC_BUCK_SLEW_RATE_VALID, status))
        {
            if (buckCfg->slewRate > PMIC_BUCK_SLEW_RATE_MAX)
            {
                status = PMIC_ST_ERR_INV_PARAM;
            }
            else
            {
                Pmic_setBitField(
                    &regData, PMIC_BUCK2_3_SLEW_RATE_SHIFT, PMIC_BUCK2_3_SLEW_RATE_MASK, buckCfg->slewRate);
            }
        }

        // Modify BUCKx_FPWM bit field (x=2 or x=3)
        if (Pmic_validParamStatusCheck(buckCfg->validParams, PMIC_BUCK_FPWM_EN_VALID, status))
        {
            Pmic_setBitField_b(&regData, PMIC_BUCK2_3_FPWM_SHIFT, PMIC_BUCK2_3_FPWM_MASK, buckCfg->fpwmEn);
        }

        // Modify BUCKx_EN bit field (x=2 or x=3)
        if (Pmic_validParamStatusCheck(buckCfg->validParams, PMIC_BUCK_ENABLE_VALID, status))
        {
            Pmic_setBitField_b(&regData, PMIC_BUCK2_3_EN_SHIFT, PMIC_BUCK2_3_EN_MASK, buckCfg->enable);
        }
    }

    // Write new register value back to PMIC
    if (Pmic_validParamStatusCheck(buckCfg->validParams, buck2_3CtrlValidParams, status))
    {
        status = Pmic_ioTxByte(pmicHandle, regAddr, regData);
    }
    Pmic_criticalSectionStop(pmicHandle);

    return status;
}

static int32_t PWR_setBuckCtrl(const Pmic_CoreHandle_t *pmicHandle, const Pmic_PwrBuckCfg_t *buckCfg)
{
    int32_t status = PMIC_ST_SUCCESS;

    // BUCK1 does not have the VMON_ONLY configuration
    const bool buck1InvalidParam = (buckCfg->resource == PMIC_BUCK1) &&
        Pmic_validParamCheck(buckCfg->validParams, PMIC_BUCK_VMON_ONLY_VALID);

    // BUCK2 and BUCK3 do not have the EN_HS_ON_SR configuration
    const bool buck2_3InvalidParam = (buckCfg->resource != PMIC_BUCK1) &&
        Pmic_validParamCheck(buckCfg->validParams, PMIC_BUCK_HIGH_SIDE_SLEW_RATE_VALID);

    if (buck1InvalidParam || buck2_3InvalidParam)
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        if (buckCfg->resource == PMIC_BUCK1)
        {
            status = PWR_setBuck1Ctrl(pmicHandle, buckCfg);
        }
        else
        {
            status = PWR_setBuck2_3Ctrl(pmicHandle, buckCfg);
        }
    }

    return status;
}

static int32_t PWR_setBuckMonConf(const Pmic_CoreHandle_t *pmicHandle, const Pmic_PwrBuckCfg_t *buckCfg)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U, regAddr = 0U;
    const uint32_t buckMonConfValidParam = PMIC_BUCK_UV_THR_VALID | PMIC_BUCK_OV_THR_VALID |
        PMIC_BUCK_RV_CONF_VALID | PMIC_BUCK_ILIM_SEL_VALID | PMIC_BUCK_DEGLITCH_SEL_VALID;

    // Read BUCKx_MON_CONF register (x=1, x=2, or x=3)
    Pmic_criticalSectionStart(pmicHandle);
    if (Pmic_validParamCheck(buckCfg->validParams, buckMonConfValidParam))
    {
        if (buckCfg->resource == PMIC_BUCK1)
        {
            regAddr = PMIC_BUCK1_MON_CONF_REGADDR;
        }
        else if (buckCfg->resource == PMIC_BUCK2)
        {
            regAddr = PMIC_BUCK2_MON_CONF_REGADDR;
        }
        else
        {
            regAddr = PMIC_BUCK3_MON_CONF_REGADDR;
        }

        status = Pmic_ioRxByte(pmicHandle, regAddr, &regData);
    }

    // Modify BUCKx_UV_THR bit field (x=1, x=2, or x=3)
    if (Pmic_validParamStatusCheck(buckCfg->validParams, PMIC_BUCK_UV_THR_VALID, status))
    {
        if (buckCfg->uvThr > PMIC_BUCK_UV_THR_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
        else
        {
            Pmic_setBitField(&regData, PMIC_BUCK_UV_THR_SHIFT, PMIC_BUCK_UV_THR_MASK, buckCfg->uvThr);
        }
    }

    // Modify BUCKx_OV_THR bit field (x=1, x=2, or x=3)
    if (Pmic_validParamStatusCheck(buckCfg->validParams, PMIC_BUCK_OV_THR_VALID, status))
    {
        if (buckCfg->ovThr > PMIC_BUCK_OV_THR_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
        else
        {
            Pmic_setBitField(&regData, PMIC_BUCK_OV_THR_SHIFT, PMIC_BUCK_OV_THR_MASK, buckCfg->ovThr);
        }
    }

    // Modify BUCKx_RV_CONF bit field (x=1, x=2, or x=3)
    if (Pmic_validParamStatusCheck(buckCfg->validParams, PMIC_BUCK_RV_CONF_VALID, status))
    {
        if (buckCfg->rvConf > PMIC_BUCK_RV_CONF_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
        else
        {
            Pmic_setBitField(&regData, PMIC_BUCK_RV_CONF_SHIFT, PMIC_BUCK_RV_CONF_MASK, buckCfg->rvConf);
        }
    }

    // Modify BUCKx_ILIM_SEL bit field (x=1, x=2, or x=3)
    if (Pmic_validParamStatusCheck(buckCfg->validParams, PMIC_BUCK_ILIM_SEL_VALID, status))
    {
        if (((buckCfg->resource == PMIC_BUCK1) && (buckCfg->ilimSel > PMIC_BUCK1_ILIM_MAX)) ||
            ((buckCfg->resource != PMIC_BUCK1) && (buckCfg->ilimSel > PMIC_BUCK2_3_ILIM_MAX)))
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
        else
        {
            Pmic_setBitField(&regData, PMIC_BUCK_ILIM_SEL_SHIFT, PMIC_BUCK_ILIM_SEL_MASK, buckCfg->ilimSel);
        }
    }

    // Modify BUCKx_DEGLITCH_SEL bit field (x=1, x=2, or x=3)
    if (Pmic_validParamStatusCheck(buckCfg->validParams, PMIC_BUCK_DEGLITCH_SEL_VALID, status))
    {
        if (buckCfg->deglitchSel > PMIC_BUCK_DEGLITCH_SEL_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
        else
        {
            Pmic_setBitField(
                &regData, PMIC_BUCK_DEGLITCH_SEL_SHIFT, PMIC_BUCK_DEGLITCH_SEL_MASK, buckCfg->deglitchSel);
        }
    }

    // Write new register value back to PMIC
    if (Pmic_validParamStatusCheck(buckCfg->validParams, buckMonConfValidParam, status))
    {
        status = Pmic_ioTxByte(pmicHandle, regAddr, regData);
    }
    Pmic_criticalSectionStop(pmicHandle);

    return status;
}

static int32_t PWR_setBuckOvpResponse(const Pmic_CoreHandle_t *pmicHandle, const Pmic_PwrBuckCfg_t *buckCfg)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    if (Pmic_validParamCheck(buckCfg->validParams, PMIC_BUCK_OVP_SEL_VALID))
    {
        // Read REG_OVP_CONF register
        Pmic_criticalSectionStart(pmicHandle);
        status = Pmic_ioRxByte(pmicHandle, PMIC_REG_OVP_CONF_REGADDR, &regData);

        if (status == PMIC_ST_SUCCESS)
        {
            uint8_t bitShift = 0U, bitMask = 0U;

            if (buckCfg->resource == PMIC_BUCK1)
            {
                bitShift = PMIC_BUCK1_OVP_SEL_SHIFT;
                bitMask = PMIC_BUCK1_OVP_SEL_MASK;
            }
            else if (buckCfg->resource == PMIC_BUCK2)
            {
                bitShift = PMIC_BUCK2_OVP_SEL_SHIFT;
                bitMask = PMIC_BUCK2_OVP_SEL_MASK;
            }
            else
            {
                bitShift = PMIC_BUCK3_OVP_SEL_SHIFT;
                bitMask = PMIC_BUCK3_OVP_SEL_MASK;
            }

            // Modify BUCKx_OVP_SEL bit field (x=1, x=2, or x=3)
            if (buckCfg->ovpSel > PMIC_REGULATOR_FAULT_RESPONSE_MAX)
            {
                status = PMIC_ST_ERR_INV_PARAM;
            }
            else
            {
                Pmic_setBitField(&regData, bitShift, bitMask, buckCfg->ovpSel);
            }

            // Write new register value back to PMIC
            if (status == PMIC_ST_SUCCESS)
            {
                status = Pmic_ioTxByte(pmicHandle, PMIC_REG_OVP_CONF_REGADDR, regData);
            }
        }
        Pmic_criticalSectionStop(pmicHandle);
    }

    return status;
}

static int32_t PWR_setBuckOvResponse(const Pmic_CoreHandle_t *pmicHandle, const Pmic_PwrBuckCfg_t *buckCfg)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    if (Pmic_validParamCheck(buckCfg->validParams, PMIC_BUCK_OV_SEL_VALID))
    {
        // Read REG_OV_CONF register
        Pmic_criticalSectionStart(pmicHandle);
        status = Pmic_ioRxByte(pmicHandle, PMIC_REG_OV_CONF_REGADDR, &regData);

        if (status == PMIC_ST_SUCCESS)
        {
            uint8_t bitShift = 0U, bitMask = 0U;

            if (buckCfg->resource == PMIC_BUCK1)
            {
                bitShift = PMIC_BUCK1_OV_SEL_SHIFT;
                bitMask = PMIC_BUCK1_OV_SEL_MASK;
            }
            else if (buckCfg->resource == PMIC_BUCK2)
            {
                bitShift = PMIC_BUCK2_OV_SEL_SHIFT;
                bitMask = PMIC_BUCK2_OV_SEL_MASK;
            }
            else
            {
                bitShift = PMIC_BUCK3_OV_SEL_SHIFT;
                bitMask = PMIC_BUCK3_OV_SEL_MASK;
            }

            // Modify BUCKx_OV_SEL bit field (x=1, x=2, or x=3)
            if (buckCfg->ovSel > PMIC_REGULATOR_FAULT_RESPONSE_MAX)
            {
                status = PMIC_ST_ERR_INV_PARAM;
            }
            else
            {
                Pmic_setBitField(&regData, bitShift, bitMask, buckCfg->ovSel);
            }

            // Write new register value back to PMIC
            if (status == PMIC_ST_SUCCESS)
            {
                status = Pmic_ioTxByte(pmicHandle, PMIC_REG_OV_CONF_REGADDR, regData);
            }
        }
        Pmic_criticalSectionStop(pmicHandle);
    }

    return status;
}

static int32_t PWR_setBuckUvResponse(const Pmic_CoreHandle_t *pmicHandle, const Pmic_PwrBuckCfg_t *buckCfg)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    if (Pmic_validParamCheck(buckCfg->validParams, PMIC_BUCK_UV_SEL_VALID))
    {
        // Read REG_UV_CONF register
        Pmic_criticalSectionStart(pmicHandle);
        status = Pmic_ioRxByte(pmicHandle, PMIC_REG_UV_CONF_REGADDR, &regData);

        if (status == PMIC_ST_SUCCESS)
        {
            uint8_t bitShift = 0U, bitMask = 0U;

            if (buckCfg->resource == PMIC_BUCK1)
            {
                bitShift = PMIC_BUCK1_UV_SEL_SHIFT;
                bitMask = PMIC_BUCK1_UV_SEL_MASK;
            }
            else if (buckCfg->resource == PMIC_BUCK2)
            {
                bitShift = PMIC_BUCK2_UV_SEL_SHIFT;
                bitMask = PMIC_BUCK2_UV_SEL_MASK;
            }
            else
            {
                bitShift = PMIC_BUCK3_UV_SEL_SHIFT;
                bitMask = PMIC_BUCK3_UV_SEL_MASK;
            }

            // Modify BUCKx_UV_SEL bit field (x=1, x=2, or x=3)
            if (buckCfg->uvSel > PMIC_REGULATOR_FAULT_RESPONSE_MAX)
            {
                status = PMIC_ST_ERR_INV_PARAM;
            }
            else
            {
                Pmic_setBitField(&regData, bitShift, bitMask, buckCfg->uvSel);
            }

            // Write new register value back to PMIC
            if (status == PMIC_ST_SUCCESS)
            {
                status = Pmic_ioTxByte(pmicHandle, PMIC_REG_UV_CONF_REGADDR, regData);
            }
        }
        Pmic_criticalSectionStop(pmicHandle);
    }

    return status;
}

static int32_t PWR_setBuckScResponse(const Pmic_CoreHandle_t *pmicHandle, const Pmic_PwrBuckCfg_t *buckCfg)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    if (Pmic_validParamCheck(buckCfg->validParams, PMIC_BUCK_SC_SEL_VALID))
    {
        // Read REG_SC_CONF register
        Pmic_criticalSectionStart(pmicHandle);
        status = Pmic_ioRxByte(pmicHandle, PMIC_REG_SC_CONF_REGADDR, &regData);

        if (status == PMIC_ST_SUCCESS)
        {
            uint8_t bitShift = 0U, bitMask = 0U;

            if (buckCfg->resource == PMIC_BUCK1)
            {
                bitShift = PMIC_BUCK1_SC_SEL_SHIFT;
                bitMask = PMIC_BUCK1_SC_SEL_MASK;
            }
            else if (buckCfg->resource == PMIC_BUCK2)
            {
                bitShift = PMIC_BUCK2_SC_SEL_SHIFT;
                bitMask = PMIC_BUCK2_SC_SEL_MASK;
            }
            else
            {
                bitShift = PMIC_BUCK3_SC_SEL_SHIFT;
                bitMask = PMIC_BUCK3_SC_SEL_MASK;
            }

            // Modify BUCKx_SC_SEL bit field (x=1, x=2, or x=3)
            if (buckCfg->scSel > PMIC_REGULATOR_FAULT_RESPONSE_MAX)
            {
                status = PMIC_ST_ERR_INV_PARAM;
            }
            else
            {
                Pmic_setBitField(&regData, bitShift, bitMask, buckCfg->scSel);
            }

            // Write new register value back to PMIC
            if (status == PMIC_ST_SUCCESS)
            {
                status = Pmic_ioTxByte(pmicHandle, PMIC_REG_SC_CONF_REGADDR, regData);
            }
        }
        Pmic_criticalSectionStop(pmicHandle);
    }

    return status;
}

static int32_t PWR_setBuckFaultResponses(const Pmic_CoreHandle_t *pmicHandle, const Pmic_PwrBuckCfg_t *buckCfg)
{
    // Set buck OVP fault response
     int32_t status = PWR_setBuckOvpResponse(pmicHandle, buckCfg);

    // Set buck OV fault response
    if (status == PMIC_ST_SUCCESS)
    {
        status = PWR_setBuckOvResponse(pmicHandle, buckCfg);
    }

    // Set buck UV fault response
    if (status == PMIC_ST_SUCCESS)
    {
        status = PWR_setBuckUvResponse(pmicHandle, buckCfg);
    }

    // Set buck SC fault response
    if (status == PMIC_ST_SUCCESS)
    {
        status = PWR_setBuckScResponse(pmicHandle, buckCfg);
    }

    return status;
}

static int32_t PWR_setBuck1Uvlo(const Pmic_CoreHandle_t *pmicHandle, const Pmic_PwrBuckCfg_t *buckCfg)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;
    const uint32_t buck1UvloValidParams = PMIC_BUCK_UVLO_FALLING_VALID | PMIC_BUCK_UVLO_RISING_VALID;

    // Read BUCK1_UVLO register
    Pmic_criticalSectionStart(pmicHandle);
    if (Pmic_validParamCheck(buckCfg->validParams, buck1UvloValidParams))
    {
        if (buckCfg->resource == PMIC_BUCK1)
        {
            status = Pmic_ioRxByte(pmicHandle, PMIC_BUCK1_UVLO_REGADDR, &regData);
        }
        else
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
    }

    // Modify BUCK1_UVLO_FALLING bit field
    if (Pmic_validParamStatusCheck(buckCfg->validParams, PMIC_BUCK_UVLO_FALLING_VALID, status))
    {
        if (buckCfg->uvloFalling > PMIC_BUCK1_UVLO_FALLING_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
        else
        {
            Pmic_setBitField(&regData, PMIC_BUCK1_UVLO_FALLING_SHIFT, PMIC_BUCK1_UVLO_FALLING_MASK, buckCfg->uvloFalling);
        }
    }

    // Modify BUCK1_UVLO_RISING bit field
    if (Pmic_validParamStatusCheck(buckCfg->validParams, PMIC_BUCK_UVLO_RISING_VALID, status))
    {
        if (buckCfg->uvloRising > PMIC_BUCK1_UVLO_RISING_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
        else
        {
            Pmic_setBitField(&regData, PMIC_BUCK1_UVLO_RISING_SHIFT, PMIC_BUCK1_UVLO_RISING_MASK, buckCfg->uvloRising);
        }
    }

    // Write new register value
    if (Pmic_validParamStatusCheck(buckCfg->validParams, buck1UvloValidParams, status))
    {
        status = Pmic_ioTxByte(pmicHandle, PMIC_BUCK1_UVLO_REGADDR, regData);
    }
    Pmic_criticalSectionStop(pmicHandle);

    return status;
}

int32_t Pmic_pwrSetBuckCfg(const Pmic_CoreHandle_t *pmicHandle, const Pmic_PwrBuckCfg_t *buckCfg)
{
    int32_t status = Pmic_checkPmicCoreHandle(pmicHandle);

    if ((status == PMIC_ST_SUCCESS) && (buckCfg == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) && ((buckCfg->resource > PMIC_BUCK_MAX) || (buckCfg->validParams == 0U)))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    // Set spread spectrum configuration
    if (status == PMIC_ST_SUCCESS)
    {
        status = PWR_setSpreadSpectrumCfg(pmicHandle, buckCfg);
    }

    // Set buck VOUT register
    if (status == PMIC_ST_SUCCESS)
    {
        status = PWR_setBuckVout(pmicHandle, buckCfg);
    }

    // Set BUCKx_MON_CONF register
    if (status == PMIC_ST_SUCCESS)
    {
        status = PWR_setBuckMonConf(pmicHandle, buckCfg);
    }

    // Set buck fault responses
    if (status == PMIC_ST_SUCCESS)
    {
        status = PWR_setBuckFaultResponses(pmicHandle, buckCfg);
    }

    // Set BUCK1_UVLO
    if (status == PMIC_ST_SUCCESS)
    {
        status = PWR_setBuck1Uvlo(pmicHandle, buckCfg);
    }

    // Set BUCKx_CTRL register
    if (status == PMIC_ST_SUCCESS)
    {
        status = PWR_setBuckCtrl(pmicHandle, buckCfg);
    }

    return status;
}

static int32_t PWR_getLdoConf(const Pmic_CoreHandle_t *pmicHandle, Pmic_PwrLdoCfg_t *ldoCfg)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    // Read LDO_CONF register
    if (Pmic_validParamCheck(ldoCfg->validParams, PMIC_LDO_MODE_VALID | PMIC_LDO_VSET_VALID))
    {
        status = Pmic_ioRxByte_CS(pmicHandle, PMIC_LDO_CONF_REGADDR, &regData);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Extract LDO_BYP_CONFIG bit field
        if (Pmic_validParamCheck(ldoCfg->validParams, PMIC_LDO_MODE_VALID))
        {
            ldoCfg->mode = Pmic_getBitField(regData, PMIC_LDO_BYP_CONFIG_SHIFT, PMIC_LDO_BYP_CONFIG_MASK);
        }

        // Extract LDO_VSET bit field
        if (Pmic_validParamCheck(ldoCfg->validParams, PMIC_LDO_VSET_VALID))
        {
            ldoCfg->vset = Pmic_getBitField(regData, PMIC_LDO_VSET_SHIFT, PMIC_LDO_VSET_MASK);
        }
    }

    return status;
}

static int32_t PWR_getLdoCtrl(const Pmic_CoreHandle_t *pmicHandle, Pmic_PwrLdoCfg_t *ldoCfg)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;
    const uint32_t ldoCtrlValidParams = PMIC_LDO_VMON_ONLY_VALID | PMIC_LDO_DISCHARGE_SEL_VALID |
        PMIC_LDO_DISCHARGE_EN_VALID | PMIC_LDO_ENABLE_VALID;

    // Read LDO_CTRL register
    if (Pmic_validParamCheck(ldoCfg->validParams, ldoCtrlValidParams))
    {
        status = Pmic_ioRxByte_CS(pmicHandle, PMIC_LDO_CTRL_REGADDR, &regData);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Extract LDO_VMON_ONLY bit field
        if (Pmic_validParamCheck(ldoCfg->validParams, PMIC_LDO_VMON_ONLY_VALID))
        {
            ldoCfg->vmonOnly = Pmic_getBitField_b(regData, PMIC_LDO_VMON_ONLY_SHIFT);
        }

        // Extract LDO_DISCHARGE_SEL bit field
        if (Pmic_validParamCheck(ldoCfg->validParams, PMIC_LDO_DISCHARGE_SEL_VALID))
        {
            ldoCfg->dischargeSel = Pmic_getBitField(regData, PMIC_LDO_DISCHARGE_SEL_SHIFT, PMIC_LDO_DISCHARGE_SEL_MASK);
        }

        // Extract LDO_DISCHARGE_EN bit field
        if (Pmic_validParamCheck(ldoCfg->validParams, PMIC_LDO_DISCHARGE_EN_VALID))
        {
            ldoCfg->dischargeEn = Pmic_getBitField_b(regData, PMIC_LDO_DISCHARGE_EN_SHIFT);
        }

        // Extract LDO_EN bit field
        if (Pmic_validParamCheck(ldoCfg->validParams, PMIC_LDO_ENABLE_VALID))
        {
            ldoCfg->enable = Pmic_getBitField_b(regData, PMIC_LDO_EN_SHIFT);
        }
    }

    return status;
}

static int32_t PWR_getLdoMonConf(const Pmic_CoreHandle_t *pmicHandle, Pmic_PwrLdoCfg_t *ldoCfg)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;
    const uint32_t ldoMonConfValidParams = PMIC_LDO_UV_THR_VALID | PMIC_LDO_OV_THR_VALID |
        PMIC_LDO_RV_CONF_VALID | PMIC_LDO_ILIM_SEL_VALID | PMIC_LDO_DEGLITCH_SEL_VALID;

    // Read LDO_MON_CONF register
    if (Pmic_validParamCheck(ldoCfg->validParams, ldoMonConfValidParams))
    {
        status = Pmic_ioRxByte_CS(pmicHandle, PMIC_LDO_MON_CONF_REGADDR, &regData);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Extract LDO_UV_THR bit field
        if (Pmic_validParamCheck(ldoCfg->validParams, PMIC_LDO_UV_THR_VALID))
        {
            ldoCfg->uvThr = Pmic_getBitField(regData, PMIC_LDO_UV_THR_SHIFT, PMIC_LDO_UV_THR_MASK);
        }

        // Extract LDO_OV_THR bit field
        if (Pmic_validParamCheck(ldoCfg->validParams, PMIC_LDO_OV_THR_VALID))
        {
            ldoCfg->ovThr = Pmic_getBitField(regData, PMIC_LDO_OV_THR_SHIFT, PMIC_LDO_OV_THR_MASK);
        }

        // Extract LDO_RV_CONF bit field
        if (Pmic_validParamCheck(ldoCfg->validParams, PMIC_LDO_RV_CONF_VALID))
        {
            ldoCfg->rvConf = Pmic_getBitField(regData, PMIC_LDO_RV_CONF_SHIFT, PMIC_LDO_RV_CONF_MASK);
        }

        // Extract LDO_ILIM_SEL bit field
        if (Pmic_validParamCheck(ldoCfg->validParams, PMIC_LDO_ILIM_SEL_VALID))
        {
            ldoCfg->ilimSel = Pmic_getBitField(regData, PMIC_LDO_ILIM_SEL_SHIFT, PMIC_LDO_ILIM_SEL_MASK);
        }

        // Extract LDO_DEGLITCH_SEL bit field
        if (Pmic_validParamCheck(ldoCfg->validParams, PMIC_LDO_DEGLITCH_SEL_VALID))
        {
            ldoCfg->deglitchSel = Pmic_getBitField(regData, PMIC_LDO_DEGLITCH_SEL_SHIFT, PMIC_LDO_DEGLITCH_SEL_MASK);
        }
    }

    return status;
}

static int32_t PWR_getLdoOvpResponse(const Pmic_CoreHandle_t *pmicHandle, Pmic_PwrLdoCfg_t *ldoCfg)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    if (Pmic_validParamCheck(ldoCfg->validParams, PMIC_LDO_OVP_SEL_VALID))
    {
        // Read REG_OVP_CONF register
        status = Pmic_ioRxByte_CS(pmicHandle, PMIC_REG_OVP_CONF_REGADDR, &regData);

        // Extract LDO_OVP_SEL bit field
        if (status == PMIC_ST_SUCCESS)
        {
            ldoCfg->ovpSel = Pmic_getBitField(regData, PMIC_LDO_OVP_SEL_SHIFT, PMIC_LDO_OVP_SEL_MASK);
        }
    }

    return status;
}

static int32_t PWR_getLdoOvResponse(const Pmic_CoreHandle_t *pmicHandle, Pmic_PwrLdoCfg_t *ldoCfg)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    if (Pmic_validParamCheck(ldoCfg->validParams, PMIC_LDO_OV_SEL_VALID))
    {
        // Read REG_OV_CONF register
        status = Pmic_ioRxByte_CS(pmicHandle, PMIC_REG_OV_CONF_REGADDR, &regData);

        // Extract LDO_OV_SEL bit field
        if (status == PMIC_ST_SUCCESS)
        {
            ldoCfg->ovSel = Pmic_getBitField(regData, PMIC_LDO_OV_SEL_SHIFT, PMIC_LDO_OV_SEL_MASK);
        }
    }

    return status;
}

static int32_t PWR_getLdoUvResponse(const Pmic_CoreHandle_t *pmicHandle, Pmic_PwrLdoCfg_t *ldoCfg)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    if (Pmic_validParamCheck(ldoCfg->validParams, PMIC_LDO_UV_SEL_VALID))
    {
        // Read REG_UV_CONF register
        status = Pmic_ioRxByte_CS(pmicHandle, PMIC_REG_UV_CONF_REGADDR, &regData);

        // Extract LDO_UV_SEL bit field
        if (status == PMIC_ST_SUCCESS)
        {
            ldoCfg->uvSel = Pmic_getBitField(regData, PMIC_LDO_UV_SEL_SHIFT, PMIC_LDO_UV_SEL_MASK);
        }
    }

    return status;
}

static int32_t PWR_getLdoScResponse(const Pmic_CoreHandle_t *pmicHandle, Pmic_PwrLdoCfg_t *ldoCfg)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    if (Pmic_validParamCheck(ldoCfg->validParams, PMIC_LDO_SC_SEL_VALID))
    {
        // Read REG_SC_CONF register
        status = Pmic_ioRxByte_CS(pmicHandle, PMIC_REG_SC_CONF_REGADDR, &regData);

        // Extract LDO_SC_SEL bit field
        if (status == PMIC_ST_SUCCESS)
        {
            ldoCfg->scSel = Pmic_getBitField(regData, PMIC_LDO_SC_SEL_SHIFT, PMIC_LDO_SC_SEL_MASK);
        }
    }

    return status;
}

static int32_t PWR_getLdoFaultResponses(const Pmic_CoreHandle_t *pmicHandle, Pmic_PwrLdoCfg_t *ldoCfg)
{
    // Get LDO OVP response
    int32_t status = PWR_getLdoOvpResponse(pmicHandle, ldoCfg);

    // Get LDO OV response
    if (status == PMIC_ST_SUCCESS)
    {
        status = PWR_getLdoOvResponse(pmicHandle, ldoCfg);
    }

    // Get LDO UV response
    if (status == PMIC_ST_SUCCESS)
    {
        status = PWR_getLdoUvResponse(pmicHandle, ldoCfg);
    }

    // Get LDO SC response
    if (status == PMIC_ST_SUCCESS)
    {
        status = PWR_getLdoScResponse(pmicHandle, ldoCfg);
    }

    return status;
}

int32_t Pmic_pwrGetLdoCfg(const Pmic_CoreHandle_t *pmicHandle, Pmic_PwrLdoCfg_t *ldoCfg)
{
    int32_t status = Pmic_checkPmicCoreHandle(pmicHandle);

    if ((status == PMIC_ST_SUCCESS) && (ldoCfg == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) && (ldoCfg->validParams == 0U))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    // Read LDO_CONF register
    if (status == PMIC_ST_SUCCESS)
    {
        status = PWR_getLdoConf(pmicHandle, ldoCfg);
    }

    // read LDO_CTRL register
    if (status == PMIC_ST_SUCCESS)
    {
        status = PWR_getLdoCtrl(pmicHandle, ldoCfg);
    }

    // Read LDO_MON_CONF register
    if (status == PMIC_ST_SUCCESS)
    {
        status = PWR_getLdoMonConf(pmicHandle, ldoCfg);
    }

    // Get LDO fault responses
    if (status == PMIC_ST_SUCCESS)
    {
        status = PWR_getLdoFaultResponses(pmicHandle, ldoCfg);
    }

    return status;
}

static int32_t PWR_setLdoConf(const Pmic_CoreHandle_t *pmicHandle, const Pmic_PwrLdoCfg_t *ldoCfg)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    // Read LDO_CONF register
    Pmic_criticalSectionStart(pmicHandle);
    if (Pmic_validParamCheck(ldoCfg->validParams, PMIC_LDO_MODE_VALID | PMIC_LDO_VSET_VALID))
    {
        status = Pmic_ioRxByte(pmicHandle, PMIC_LDO_CONF_REGADDR, &regData);
    }


    // Modify LDO_BYP_CONFIG bit field
    if (Pmic_validParamStatusCheck(ldoCfg->validParams, PMIC_LDO_MODE_VALID, status))
    {
        if (ldoCfg->mode > PMIC_LDO_BYP_CONFIG_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
        else
        {
            Pmic_setBitField(&regData, PMIC_LDO_BYP_CONFIG_SHIFT, PMIC_LDO_BYP_CONFIG_MASK, ldoCfg->mode);
        }
    }

    // Modify LDO_VSET bit field
    if (Pmic_validParamStatusCheck(ldoCfg->validParams, PMIC_LDO_VSET_VALID, status))
    {
        if ((ldoCfg->vset > PMIC_LDO_VSET_MAX) || (ldoCfg->vset < PMIC_LDO_VSET_MIN))
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
        else
        {
            Pmic_setBitField(&regData, PMIC_LDO_VSET_SHIFT, PMIC_LDO_VSET_MASK, ldoCfg->vset);
        }
    }

    // Write new register value back to PMIC
    if (Pmic_validParamStatusCheck(ldoCfg->validParams, PMIC_LDO_MODE_VALID | PMIC_LDO_VSET_VALID, status))
    {
        status = Pmic_ioTxByte(pmicHandle, PMIC_LDO_CONF_REGADDR, regData);
    }
    Pmic_criticalSectionStop(pmicHandle);

    return status;
}

static int32_t PWR_setLdoCtrl(const Pmic_CoreHandle_t *pmicHandle, const Pmic_PwrLdoCfg_t *ldoCfg)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;
    const uint32_t ldoCtrlValidParams = PMIC_LDO_VMON_ONLY_VALID | PMIC_LDO_DISCHARGE_SEL_VALID |
        PMIC_LDO_DISCHARGE_EN_VALID | PMIC_LDO_ENABLE_VALID;

    // Read LDO_CTRL register
    Pmic_criticalSectionStart(pmicHandle);
    if (Pmic_validParamCheck(ldoCfg->validParams, ldoCtrlValidParams))
    {
        status = Pmic_ioRxByte(pmicHandle, PMIC_LDO_CTRL_REGADDR, &regData);
    }

    // Modify LDO_VMON_ONLY bit field
    if (Pmic_validParamStatusCheck(ldoCfg->validParams, PMIC_LDO_VMON_ONLY_VALID, status))
    {
        Pmic_setBitField_b(&regData, PMIC_LDO_VMON_ONLY_SHIFT, PMIC_LDO_VMON_ONLY_MASK, ldoCfg->vmonOnly);
    }

    // Modify LDO_DISCHARGE_SEL bit field
    if (Pmic_validParamStatusCheck(ldoCfg->validParams, PMIC_LDO_DISCHARGE_SEL_VALID, status))
    {
        if (ldoCfg->dischargeSel > PMIC_LDO_DISCHARGE_SEL_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
        else
        {
            Pmic_setBitField(&regData, PMIC_LDO_DISCHARGE_SEL_SHIFT, PMIC_LDO_DISCHARGE_SEL_MASK, ldoCfg->dischargeSel);
        }
    }

    // Modify LDO_DISCHARGE_EN bit field
    if (Pmic_validParamStatusCheck(ldoCfg->validParams, PMIC_LDO_DISCHARGE_EN_VALID, status))
    {
        Pmic_setBitField_b(&regData, PMIC_LDO_DISCHARGE_EN_SHIFT, PMIC_LDO_DISCHARGE_EN_MASK, ldoCfg->dischargeEn);
    }

    // Modify LDO_EN bit field
    if (Pmic_validParamStatusCheck(ldoCfg->validParams, PMIC_LDO_ENABLE_VALID, status))
    {
        Pmic_setBitField_b(&regData, PMIC_LDO_EN_SHIFT, PMIC_LDO_EN_MASK, ldoCfg->enable);
    }

    // Write new register value back to PMIC
    if (Pmic_validParamStatusCheck(ldoCfg->validParams, ldoCtrlValidParams, status))
    {
        status = Pmic_ioTxByte(pmicHandle, PMIC_LDO_CTRL_REGADDR, regData);
    }
    Pmic_criticalSectionStop(pmicHandle);

    return status;
}

static int32_t PWR_setLdoMonConf(const Pmic_CoreHandle_t *pmicHandle, const Pmic_PwrLdoCfg_t *ldoCfg)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;
    const uint32_t ldoMonConfValidParams = PMIC_LDO_UV_THR_VALID | PMIC_LDO_OV_THR_VALID |
        PMIC_LDO_RV_CONF_VALID | PMIC_LDO_ILIM_SEL_VALID | PMIC_LDO_DEGLITCH_SEL_VALID;

    // Read LDO_MON_CONF register
    Pmic_criticalSectionStart(pmicHandle);
    if (Pmic_validParamCheck(ldoCfg->validParams, ldoMonConfValidParams))
    {
        status = Pmic_ioRxByte(pmicHandle, PMIC_LDO_MON_CONF_REGADDR, &regData);
    }

    // Modify LDO_UV_THR bit field
    if (Pmic_validParamStatusCheck(ldoCfg->validParams, PMIC_LDO_UV_THR_VALID, status))
    {
        if (ldoCfg->uvThr > PMIC_LDO_UV_THR_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
        else
        {
            Pmic_setBitField(&regData, PMIC_LDO_UV_THR_SHIFT, PMIC_LDO_UV_THR_MASK, ldoCfg->uvThr);
        }
    }

    // Modify LDO_OV_THR bit field
    if (Pmic_validParamStatusCheck(ldoCfg->validParams, PMIC_LDO_OV_THR_VALID, status))
    {
        if (ldoCfg->ovThr > PMIC_LDO_OV_THR_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
        else
        {
            Pmic_setBitField(&regData, PMIC_LDO_OV_THR_SHIFT, PMIC_LDO_OV_THR_MASK, ldoCfg->ovThr);
        }
    }

    // Modify LDO_RV_CONF bit field
    if (Pmic_validParamStatusCheck(ldoCfg->validParams, PMIC_LDO_RV_CONF_VALID, status))
    {
        if (ldoCfg->rvConf > PMIC_LDO_RV_CONF_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
        else
        {
            Pmic_setBitField(&regData, PMIC_LDO_RV_CONF_SHIFT, PMIC_LDO_RV_CONF_MASK, ldoCfg->rvConf);
        }
    }

    // Modify LDO_ILIM_SEL bit field
    if (Pmic_validParamStatusCheck(ldoCfg->validParams, PMIC_LDO_ILIM_SEL_VALID, status))
    {
        if (ldoCfg->ilimSel > PMIC_LDO_ILIM_SEL_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
        else
        {
            Pmic_setBitField(&regData, PMIC_LDO_ILIM_SEL_SHIFT, PMIC_LDO_ILIM_SEL_MASK, ldoCfg->ilimSel);
        }
    }

    // Modify LDO_DEGLITCH_SEL bit field
    if (Pmic_validParamStatusCheck(ldoCfg->validParams, PMIC_LDO_DEGLITCH_SEL_VALID, status))
    {
        if (ldoCfg->deglitchSel > PMIC_LDO_DEGLITCH_SEL_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
        else
        {
            Pmic_setBitField(&regData, PMIC_LDO_DEGLITCH_SEL_SHIFT, PMIC_LDO_DEGLITCH_SEL_MASK, ldoCfg->deglitchSel);
        }
    }

    // Write new register value back to PMIC
    if (Pmic_validParamStatusCheck(ldoCfg->validParams, ldoMonConfValidParams, status))
    {
        status = Pmic_ioTxByte(pmicHandle, PMIC_LDO_MON_CONF_REGADDR, regData);
    }
    Pmic_criticalSectionStop(pmicHandle);

    return status;
}

static int32_t PWR_setLdoOvpResponse(const Pmic_CoreHandle_t *pmicHandle, const Pmic_PwrLdoCfg_t *ldoCfg)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    if (Pmic_validParamCheck(ldoCfg->validParams, PMIC_LDO_OVP_SEL_VALID))
    {
        // Read REG_OVP_CONF register
        Pmic_criticalSectionStart(pmicHandle);
        status = Pmic_ioRxByte(pmicHandle, PMIC_REG_OVP_CONF_REGADDR, &regData);

        // Modify LDO_OVP_SEL bit field
        if (status == PMIC_ST_SUCCESS)
        {
            if (ldoCfg->ovpSel > PMIC_REGULATOR_FAULT_RESPONSE_MAX)
            {
                status = PMIC_ST_ERR_INV_PARAM;
            }
            else
            {
                Pmic_setBitField(&regData, PMIC_LDO_OVP_SEL_SHIFT, PMIC_LDO_OVP_SEL_MASK, ldoCfg->ovpSel);
            }
        }

        // Write new register value back to PMIC
        if (status == PMIC_ST_SUCCESS)
        {
            status = Pmic_ioTxByte(pmicHandle, PMIC_REG_OVP_CONF_REGADDR, regData);
        }
        Pmic_criticalSectionStop(pmicHandle);
    }

    return status;
}

static int32_t PWR_setLdoOvResponse(const Pmic_CoreHandle_t *pmicHandle, const Pmic_PwrLdoCfg_t *ldoCfg)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    if (Pmic_validParamCheck(ldoCfg->validParams, PMIC_LDO_OV_SEL_VALID))
    {
        // Read REG_OV_CONF register
        Pmic_criticalSectionStart(pmicHandle);
        status = Pmic_ioRxByte(pmicHandle, PMIC_REG_OV_CONF_REGADDR, &regData);

        // Modify LDO_OV_SEL bit field
        if (status == PMIC_ST_SUCCESS)
        {
            if (ldoCfg->ovSel > PMIC_REGULATOR_FAULT_RESPONSE_MAX)
            {
                status = PMIC_ST_ERR_INV_PARAM;
            }
            else
            {
                Pmic_setBitField(&regData, PMIC_LDO_OV_SEL_SHIFT, PMIC_LDO_OV_SEL_MASK, ldoCfg->ovSel);
            }
        }

        // Write new register value back to PMIC
        if (status == PMIC_ST_SUCCESS)
        {
            status = Pmic_ioTxByte(pmicHandle, PMIC_REG_OV_CONF_REGADDR, regData);
        }
        Pmic_criticalSectionStop(pmicHandle);
    }

    return status;
}

static int32_t PWR_setLdoUvResponse(const Pmic_CoreHandle_t *pmicHandle, const Pmic_PwrLdoCfg_t *ldoCfg)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    if (Pmic_validParamCheck(ldoCfg->validParams, PMIC_LDO_UV_SEL_VALID))
    {
        // Read REG_UV_CONF register
        Pmic_criticalSectionStart(pmicHandle);
        status = Pmic_ioRxByte(pmicHandle, PMIC_REG_UV_CONF_REGADDR, &regData);

        // Modify LDO_UV_SEL bit field
        if (status == PMIC_ST_SUCCESS)
        {
            if (ldoCfg->uvSel > PMIC_REGULATOR_FAULT_RESPONSE_MAX)
            {
                status = PMIC_ST_ERR_INV_PARAM;
            }
            else
            {
                Pmic_setBitField(&regData, PMIC_LDO_UV_SEL_SHIFT, PMIC_LDO_UV_SEL_MASK, ldoCfg->uvSel);
            }
        }

        // Write new register value back to PMIC
        if (status == PMIC_ST_SUCCESS)
        {
            status = Pmic_ioTxByte(pmicHandle, PMIC_REG_UV_CONF_REGADDR, regData);
        }
        Pmic_criticalSectionStop(pmicHandle);
    }

    return status;
}

static int32_t PWR_setLdoScResponse(const Pmic_CoreHandle_t *pmicHandle, const Pmic_PwrLdoCfg_t *ldoCfg)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    if (Pmic_validParamCheck(ldoCfg->validParams, PMIC_LDO_SC_SEL_VALID))
    {
        // Read REG_SC_CONF register
        Pmic_criticalSectionStart(pmicHandle);
        status = Pmic_ioRxByte(pmicHandle, PMIC_REG_SC_CONF_REGADDR, &regData);

        // Modify LDO_SC_SEL bit field
        if (status == PMIC_ST_SUCCESS)
        {
            if (ldoCfg->scSel > PMIC_REGULATOR_FAULT_RESPONSE_MAX)
            {
                status = PMIC_ST_ERR_INV_PARAM;
            }
            else
            {
                Pmic_setBitField(&regData, PMIC_LDO_SC_SEL_SHIFT, PMIC_LDO_SC_SEL_MASK, ldoCfg->scSel);
            }
        }

        // Write new register value back to PMIC
        if (status == PMIC_ST_SUCCESS)
        {
            status = Pmic_ioTxByte(pmicHandle, PMIC_REG_SC_CONF_REGADDR, regData);
        }
        Pmic_criticalSectionStop(pmicHandle);
    }

    return status;
}

static int32_t PWR_setLdoFaultResponses(const Pmic_CoreHandle_t *pmicHandle, const Pmic_PwrLdoCfg_t *ldoCfg)
{
    // Set LDO OVP response
    int32_t status = PWR_setLdoOvpResponse(pmicHandle, ldoCfg);

    // Set LDO OV response
    if (status == PMIC_ST_SUCCESS)
    {
        status = PWR_setLdoOvResponse(pmicHandle, ldoCfg);
    }

    // Set LDO UV response
    if (status == PMIC_ST_SUCCESS)
    {
        status = PWR_setLdoUvResponse(pmicHandle, ldoCfg);
    }

    // Set LDO SC response
    if (status == PMIC_ST_SUCCESS)
    {
        status = PWR_setLdoScResponse(pmicHandle, ldoCfg);
    }

    return status;
}

int32_t Pmic_pwrSetLdoCfg(const Pmic_CoreHandle_t *pmicHandle, const Pmic_PwrLdoCfg_t *ldoCfg)
{
    int32_t status = Pmic_checkPmicCoreHandle(pmicHandle);

    if ((status == PMIC_ST_SUCCESS) && (ldoCfg == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) && (ldoCfg->validParams == 0U))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    // Set LDO_CONF register configurations
    if (status == PMIC_ST_SUCCESS)
    {
        status = PWR_setLdoConf(pmicHandle, ldoCfg);
    }

    // Set LDO_MON_CONF register configurations
    if (status == PMIC_ST_SUCCESS)
    {
        status = PWR_setLdoMonConf(pmicHandle, ldoCfg);
    }

    // Set LDO fault responses
    if (status == PMIC_ST_SUCCESS)
    {
        status = PWR_setLdoFaultResponses(pmicHandle, ldoCfg);
    }

    // Set LDO_CTRL register configurations
    if (status == PMIC_ST_SUCCESS)
    {
        status = PWR_setLdoCtrl(pmicHandle, ldoCfg);
    }

    return status;
}

static int32_t PWR_getStatStartup(const Pmic_CoreHandle_t *pmicHandle, Pmic_PwrRsrcStat_t *pwrRsrcStat)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    // Read STAT_STARTUP register
    status = Pmic_ioRxByte_CS(pmicHandle, PMIC_STAT_STARTUP_REGADDR, &regData);

    if (status == PMIC_ST_SUCCESS)
    {
        switch (pwrRsrcStat->resource)
        {
            // Extract BUCK1_ACTIVE bit field
            case PMIC_BUCK1:
                pwrRsrcStat->active = Pmic_getBitField_b(regData, PMIC_BUCK1_ACTIVE_SHIFT);
                break;
            // Extract BUCK2_ACTIVE bit field
            case PMIC_BUCK2:
                pwrRsrcStat->active = Pmic_getBitField_b(regData, PMIC_BUCK2_ACTIVE_SHIFT);
                break;
            // Extract BUCK3_ACTIVE bit field
            case PMIC_BUCK3:
                pwrRsrcStat->active = Pmic_getBitField_b(regData, PMIC_BUCK3_ACTIVE_SHIFT);
                break;
            // Extract LDO_ACTIVE bit field
            default:
                pwrRsrcStat->active = Pmic_getBitField_b(regData, PMIC_LDO_ACTIVE_SHIFT);
                break;
        }
    }

    return status;
}

static int32_t PWR_getStatBuck1_2(const Pmic_CoreHandle_t *pmicHandle, Pmic_PwrRsrcStat_t *pwrRsrcStat)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    // Read STAT_BUCK1_2 register
    status = Pmic_ioRxByte_CS(pmicHandle, PMIC_STAT_BUCK1_2_REGADDR, &regData);

    if (status == PMIC_ST_SUCCESS)
    {
        if (pwrRsrcStat->resource == PMIC_BUCK1)
        {
            // Extract BUCK1_OVP_STAT bit field
            pwrRsrcStat->ovp = Pmic_getBitField_b(regData, PMIC_BUCK1_OVP_STAT_SHIFT);

            // Extract BUCK1_UV_STAT bit field
            pwrRsrcStat->uv = Pmic_getBitField_b(regData, PMIC_BUCK1_UV_STAT_SHIFT);

            // Extract BUCK1_OV_STAT bit field
            pwrRsrcStat->ov = Pmic_getBitField_b(regData, PMIC_BUCK1_OV_STAT_SHIFT);
        }
        else
        {
            // Extract BUCK2_OVP_STAT bit field
            pwrRsrcStat->ovp = Pmic_getBitField_b(regData, PMIC_BUCK2_OVP_STAT_SHIFT);

            // Extract BUCK2_UV_STAT bit field
            pwrRsrcStat->uv = Pmic_getBitField_b(regData, PMIC_BUCK2_UV_STAT_SHIFT);

            // Extract BUCK2_OV_STAT bit field
            pwrRsrcStat->ov = Pmic_getBitField_b(regData, PMIC_BUCK2_OV_STAT_SHIFT);
        }
    }

    return status;
}

static int32_t PWR_getStatBuck3Ldo(const Pmic_CoreHandle_t *pmicHandle, Pmic_PwrRsrcStat_t *pwrRsrcStat)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    // read STAT_BUCK3_LDO register
    status = Pmic_ioRxByte_CS(pmicHandle, PMIC_STAT_BUCK3_LDO_REGADDR, &regData);

    if (status == PMIC_ST_SUCCESS)
    {
        if (pwrRsrcStat->resource == PMIC_BUCK3)
        {
            // Extract BUCK3_OVP_STAT bit field
            pwrRsrcStat->ovp = Pmic_getBitField_b(regData, PMIC_BUCK3_OVP_STAT_SHIFT);

            // Extract BUCK3_UV_STAT bit field
            pwrRsrcStat->uv = Pmic_getBitField_b(regData, PMIC_BUCK3_UV_STAT_SHIFT);

            // Extract BUCK3_OV_STAT bit field
            pwrRsrcStat->ov = Pmic_getBitField_b(regData, PMIC_BUCK3_OV_STAT_SHIFT);
        }
        else
        {
            // Extract LDO_OVP_STAT bit field
            pwrRsrcStat->ovp = Pmic_getBitField_b(regData, PMIC_LDO_OVP_STAT_SHIFT);

            // Extract LDO_UV_STAT bit field
            pwrRsrcStat->uv = Pmic_getBitField_b(regData, PMIC_LDO_UV_STAT_SHIFT);

            // Extract LDO_OV_STAT bit field
            pwrRsrcStat->ov = Pmic_getBitField_b(regData, PMIC_LDO_OV_STAT_SHIFT);
        }
    }

    return status;
}

int32_t Pmic_pwrGetRsrcStat(const Pmic_CoreHandle_t *pmicHandle, Pmic_PwrRsrcStat_t *pwrRsrcStat)
{
    int32_t status = Pmic_checkPmicCoreHandle(pmicHandle);

    if ((status == PMIC_ST_SUCCESS) && (pwrRsrcStat == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) && (pwrRsrcStat->resource > PMIC_POWER_RESOURCE_MAX))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    // Get resource active status bit from STAT_STARTUP register
    if (status == PMIC_ST_SUCCESS)
    {
        status = PWR_getStatStartup(pmicHandle, pwrRsrcStat);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // If power resource is BUCK1 or BUCK2, get status bits from STAT_BUCK1_2 register
        if ((pwrRsrcStat->resource == PMIC_BUCK1) || (pwrRsrcStat->resource == PMIC_BUCK2))
        {
            status = PWR_getStatBuck1_2(pmicHandle, pwrRsrcStat);
        }
        // If power resource is BUCK3 or LDO, get status bits from STAT_BUCK3_LDO register
        else
        {
            status = PWR_getStatBuck3Ldo(pmicHandle, pwrRsrcStat);
        }
    }

    return status;
}

int32_t Pmic_pwrSetTsdCfg(const Pmic_CoreHandle_t *pmicHandle, const Pmic_PwrTsdCfg_t *tsdCfg)
{
    uint8_t regData = 0U;
    int32_t status = Pmic_checkPmicCoreHandle(pmicHandle);
    const uint32_t config1ValidParam = PMIC_TWARN_STAY_IN_SAFE_STATE_VALID |
        PMIC_TSD_IMM_LEVEL_VALID | PMIC_TWARN_LEVEL_VALID;

    if ((status == PMIC_ST_SUCCESS) && (tsdCfg == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) && (tsdCfg->validParams == 0U))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    // Read CONFIG_1 register
    Pmic_criticalSectionStart(pmicHandle);
    if (Pmic_validParamStatusCheck(tsdCfg->validParams, config1ValidParam, status))
    {
        status = Pmic_ioRxByte(pmicHandle, PMIC_CONFIG_1_REGADDR, &regData);
    }

    // Modify TWARN_CONFIG bit field
    if (Pmic_validParamStatusCheck(tsdCfg->validParams, PMIC_TWARN_STAY_IN_SAFE_STATE_VALID, status))
    {
        Pmic_setBitField_b(&regData, PMIC_TWARN_CONFIG_SHIFT, PMIC_TWARN_CONFIG_MASK, tsdCfg->twarnStayInSafeState);
    }

    // Modify TSD_IMM_LEVEL bit field
    if (Pmic_validParamStatusCheck(tsdCfg->validParams, PMIC_TSD_IMM_LEVEL_VALID, status))
    {
        if (tsdCfg->tsdImmLevel > PMIC_TSD_IMM_LEVEL_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
        else
        {
            Pmic_setBitField_b(&regData, PMIC_TSD_IMM_LEVEL_SHIFT, PMIC_TSD_IMM_LEVEL_MASK, tsdCfg->tsdImmLevel);
        }
    }

    // Modify TWARN_LEVEL bit field
    if (Pmic_validParamStatusCheck(tsdCfg->validParams, PMIC_TWARN_LEVEL_VALID, status))
    {
        if (tsdCfg->twarnLevel > PMIC_TWARN_LEVEL_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
        else
        {
            Pmic_setBitField_b(&regData, PMIC_TWARN_LEVEL_SHIFT, PMIC_TWARN_LEVEL_MASK, tsdCfg->twarnLevel);
        }
    }

    // Write new register value back to PMIC
    if (Pmic_validParamStatusCheck(tsdCfg->validParams, config1ValidParam, status))
    {
        status = Pmic_ioTxByte(pmicHandle, PMIC_CONFIG_1_REGADDR, regData);
    }
    Pmic_criticalSectionStop(pmicHandle);

    return status;
}

int32_t Pmic_pwrGetTsdCfg(const Pmic_CoreHandle_t *pmicHandle, Pmic_PwrTsdCfg_t *tsdCfg)
{
    uint8_t regData = 0U;
    int32_t status = Pmic_checkPmicCoreHandle(pmicHandle);
    const uint32_t config1ValidParam = PMIC_TWARN_STAY_IN_SAFE_STATE_VALID |
        PMIC_TSD_IMM_LEVEL_VALID | PMIC_TWARN_LEVEL_VALID;

    if ((status == PMIC_ST_SUCCESS) && (tsdCfg == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) && (tsdCfg->validParams == 0U))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    // Read CONFIG_1 register
    if (Pmic_validParamStatusCheck(tsdCfg->validParams, config1ValidParam, status))
    {
        status = Pmic_ioRxByte_CS(pmicHandle, PMIC_CONFIG_1_REGADDR, &regData);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Extract TWARN_CONFIG bit field
        if (Pmic_validParamCheck(tsdCfg->validParams, PMIC_TWARN_STAY_IN_SAFE_STATE_VALID))
        {
            tsdCfg->twarnStayInSafeState = Pmic_getBitField_b(regData, PMIC_TWARN_CONFIG_SHIFT);
        }

        // Extract TSD_IMM_LEVEL bit field
        if (Pmic_validParamCheck(tsdCfg->validParams, PMIC_TSD_IMM_LEVEL_VALID))
        {
            tsdCfg->tsdImmLevel = Pmic_getBitField(regData, PMIC_TSD_IMM_LEVEL_SHIFT, PMIC_TSD_IMM_LEVEL_MASK);
        }

        // Extract TWARN_LEVEL bit field
        if (Pmic_validParamCheck(tsdCfg->validParams, PMIC_TWARN_LEVEL_VALID))
        {
            tsdCfg->twarnLevel = Pmic_getBitField(regData, PMIC_TWARN_LEVEL_SHIFT, PMIC_TWARN_LEVEL_MASK);
        }
    }

    return status;
}

int32_t Pmic_pwrGetTsdImmStat(const Pmic_CoreHandle_t *pmicHandle, bool *tsdImmStat)
{
    uint8_t regData = 0U;
    int32_t status = Pmic_checkPmicCoreHandle(pmicHandle);

    if ((status == PMIC_ST_SUCCESS) && (tsdImmStat == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    // Read STAT_SEVERE_ERR register
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioRxByte_CS(pmicHandle, PMIC_STAT_SEVERE_ERR_REGADDR, &regData);
    }

    // Extract TSD_IMM_STAT bit field
    if (status == PMIC_ST_SUCCESS)
    {
        *tsdImmStat = Pmic_getBitField_b(regData, PMIC_TSD_IMM_STAT_SHIFT);
    }

    return status;
}

static inline uint8_t PWR_getSeqTrigPwrRsrc(uint16_t seqTrig)
{
    return (seqTrig >> 8U);
}

static inline uint8_t PWR_getSeqTrigBitPos(uint16_t seqTrig)
{
    return (seqTrig & 0xFFU);
}

static inline void PWR_getSeqTrigReg(uint8_t pwrRsrc, uint8_t *regAddr)
{
    switch (pwrRsrc)
    {
        case PMIC_BUCK1:
        {
            *regAddr = PMIC_SEQ_TRIG_BUCK1_REGADDR;
            break;
        }
        case PMIC_BUCK2:
        {
            *regAddr = PMIC_SEQ_TRIG_BUCK2_REGADDR;
            break;
        }
        case PMIC_BUCK3:
        {
            *regAddr = PMIC_SEQ_TRIG_BUCK3_REGADDR;
            break;
        }
        default:
        {
            *regAddr = PMIC_SEQ_TRIG_LDO_REGADDR;
            break;
        }
    }
}

int32_t Pmic_pwrGetBuckLdoSeqTrig(const Pmic_CoreHandle_t *pmicHandle, Pmic_PwrBuckLdoSeqTrig_t seqTrigCfg[], uint8_t len)
{
    const uint8_t maxSeqTrigBitPos = 5U;
    uint8_t regData = 0U, pwrRsrc = 0U, bitPos = 0U, regAddr = 0U;
    int32_t status = Pmic_checkPmicCoreHandle(pmicHandle);

    if ((status == PMIC_ST_SUCCESS) && (seqTrigCfg == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) && (len == 0U))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // For each sequence trigger...
        for (uint8_t i = 0U; i < len; i++)
        {
            // Get the power resource of the trigger and check whether power resource is valid
            pwrRsrc = PWR_getSeqTrigPwrRsrc(seqTrigCfg[i].trigger);
            if (pwrRsrc > PMIC_POWER_RESOURCE_MAX)
            {
                status = PMIC_ST_ERR_INV_PARAM;
            }

            // Get the sequence trigger bit position and check whether bit position is valid
            if (status == PMIC_ST_SUCCESS)
            {
                bitPos = PWR_getSeqTrigBitPos(seqTrigCfg[i].trigger);
                if (bitPos > maxSeqTrigBitPos)
                {
                    status = PMIC_ST_ERR_INV_PARAM;
                }
            }

            // Read the associated sequence trigger register
            if (status == PMIC_ST_SUCCESS)
            {
                PWR_getSeqTrigReg(pwrRsrc, &regAddr);

                status = Pmic_ioRxByte_CS(pmicHandle, regAddr, &regData);
            }

            // Extract sequence trigger exclude/include bit
            if (status == PMIC_ST_SUCCESS)
            {
                seqTrigCfg[i].exclude = Pmic_getBitField_b(regData, bitPos);
            }
            else
            {
                break;
            }
        }
    }

    return status;
}

int32_t Pmic_pwrSetBuckLdoSeqTrig(const Pmic_CoreHandle_t *pmicHandle, const Pmic_PwrBuckLdoSeqTrig_t seqTrigCfg[], uint8_t len)
{
    const uint8_t maxSeqTrigBitPos = 5U;
    uint8_t regData = 0U, pwrRsrc = 0U, bitPos = 0U, regAddr = 0U;
    int32_t status = Pmic_checkPmicCoreHandle(pmicHandle);

    if ((status == PMIC_ST_SUCCESS) && (seqTrigCfg == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) && (len == 0U))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // For each sequence trigger...
        for (uint8_t i = 0U; i < len; i++)
        {
            // Get the power resource of the trigger and check whether power resource is valid
            pwrRsrc = PWR_getSeqTrigPwrRsrc(seqTrigCfg[i].trigger);
            if (pwrRsrc > PMIC_POWER_RESOURCE_MAX)
            {
                status = PMIC_ST_ERR_INV_PARAM;
            }

            // Get the sequence trigger bit position and check whether bit position is valid
            if (status == PMIC_ST_SUCCESS)
            {
                bitPos = PWR_getSeqTrigBitPos(seqTrigCfg[i].trigger);
                if (bitPos > maxSeqTrigBitPos)
                {
                    status = PMIC_ST_ERR_INV_PARAM;
                }
            }

            // Read the associated sequence trigger register
            Pmic_criticalSectionStart(pmicHandle);
            if (status == PMIC_ST_SUCCESS)
            {
                PWR_getSeqTrigReg(pwrRsrc, &regAddr);

                status = Pmic_ioRxByte(pmicHandle, regAddr, &regData);
            }

            if (status == PMIC_ST_SUCCESS)
            {
                // Modify sequence trigger exclude/include bit
                Pmic_setBitField_b(&regData, bitPos, (uint8_t)(1U << bitPos), seqTrigCfg[i].exclude);

                // Write new register value back to PMIC
                status = Pmic_ioTxByte(pmicHandle, regAddr, regData);
            }
            Pmic_criticalSectionStop(pmicHandle);

            if (status != PMIC_ST_SUCCESS)
            {
                break;
            }
        }
    }

    return status;
}

static inline void PWR_getSeqDlyReg(uint8_t pwrRsrc, uint8_t *regAddr)
{
    switch (pwrRsrc)
    {
        case PMIC_BUCK1:
        {
            *regAddr = PMIC_BUCK1_SEQ_DLY_REGADDR;
            break;
        }
        case PMIC_BUCK2:
        {
            *regAddr = PMIC_BUCK2_SEQ_DLY_REGADDR;
            break;
        }
        case PMIC_BUCK3:
        {
            *regAddr = PMIC_BUCK3_SEQ_DLY_REGADDR;
            break;
        }
        default:
        {
            *regAddr = PMIC_LDO_SEQ_DLY_REGADDR;
            break;
        }
    }
}

int32_t Pmic_pwrGetBuckLdoSeqDly(const Pmic_CoreHandle_t *pmicHandle, Pmic_PwrBuckLdoSeqDly_t seqDlyCfg[], uint8_t len)
{
    uint8_t regData = 0U, regAddr = 0U;
    int32_t status = Pmic_checkPmicCoreHandle(pmicHandle);

    if ((status == PMIC_ST_SUCCESS) && (seqDlyCfg == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) && (len == 0U))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // For each sequence delay configuration...
        for (uint8_t i = 0U; i < len; i++)
        {
            // Check whether power resource is valid and whether validParams is set
            if ((seqDlyCfg->resource > PMIC_POWER_RESOURCE_MAX) || (seqDlyCfg->validParams == 0U))
            {
                status = PMIC_ST_ERR_INV_PARAM;
            }

            // Read from associated sequence delay register
            if (status == PMIC_ST_SUCCESS)
            {
                PWR_getSeqDlyReg(seqDlyCfg->resource, &regAddr);

                status = Pmic_ioRxByte_CS(pmicHandle, regAddr, &regData);
            }

            if (status == PMIC_ST_SUCCESS)
            {
                // Extract SEQ_DLY_OFF bit field
                if (Pmic_validParamCheck(seqDlyCfg->validParams, PMIC_SEQ_DLY_OFF_VALID))
                {
                    seqDlyCfg->seqDlyOff = Pmic_getBitField(
                        regData, PMIC_REGULATOR_SEQ_DLY_OFF_SHIFT, PMIC_REGULATOR_SEQ_DLY_OFF_MASK);
                }

                // Extract SEQ_DLY_ON bit field
                if (Pmic_validParamCheck(seqDlyCfg->validParams, PMIC_SEQ_DLY_ON_VALID))
                {
                    seqDlyCfg->seqDlyOn = Pmic_getBitField(
                        regData, PMIC_REGULATOR_SEQ_DLY_ON_SHIFT, PMIC_REGULATOR_SEQ_DLY_ON_MASK);
                }
            }
        }
    }

    return status;
}

int32_t Pmic_pwrSetBuckLdoSeqDly(const Pmic_CoreHandle_t *pmicHandle, const Pmic_PwrBuckLdoSeqDly_t seqDlyCfg[], uint8_t len)
{
    uint8_t regData = 0U, regAddr = 0U;
    int32_t status = Pmic_checkPmicCoreHandle(pmicHandle);

    if ((status == PMIC_ST_SUCCESS) && (seqDlyCfg == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) && (len == 0U))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // For each sequence delay configuration...
        for (uint8_t i = 0U; i < len; i++)
        {
            // Check whether power resource is valid and whether validParams is set
            if ((seqDlyCfg->resource > PMIC_POWER_RESOURCE_MAX) || (seqDlyCfg->validParams == 0U))
            {
                status = PMIC_ST_ERR_INV_PARAM;
            }

            // Read from associated sequence delay register
            Pmic_criticalSectionStart(pmicHandle);
            if (status == PMIC_ST_SUCCESS)
            {
                PWR_getSeqDlyReg(seqDlyCfg->resource, &regAddr);

                status = Pmic_ioRxByte(pmicHandle, regAddr, &regData);
            }

            // Modify SEQ_DLY_OFF bit field
            if (Pmic_validParamStatusCheck(seqDlyCfg->validParams, PMIC_SEQ_DLY_OFF_VALID, status))
            {
                if (seqDlyCfg->seqDlyOff > PMIC_SEQ_DLY_MAX)
                {
                    status = PMIC_ST_ERR_INV_PARAM;
                }
                else
                {
                    Pmic_setBitField(
                        &regData, PMIC_REGULATOR_SEQ_DLY_OFF_SHIFT, PMIC_REGULATOR_SEQ_DLY_OFF_MASK, seqDlyCfg->seqDlyOff);
                }
            }

            // Modify SEQ_DLY_ON bit field
            if (Pmic_validParamStatusCheck(seqDlyCfg->validParams, PMIC_SEQ_DLY_ON_VALID, status))
            {
                if (seqDlyCfg->seqDlyOn > PMIC_SEQ_DLY_MAX)
                {
                    status = PMIC_ST_ERR_INV_PARAM;
                }
                else
                {
                    Pmic_setBitField(
                        &regData, PMIC_REGULATOR_SEQ_DLY_ON_SHIFT, PMIC_REGULATOR_SEQ_DLY_ON_MASK, seqDlyCfg->seqDlyOn);
                }
            }

            // Write new register value back to PMIC
            if (status == PMIC_ST_SUCCESS)
            {
                status = Pmic_ioTxByte(pmicHandle, regAddr, regData);
            }
            Pmic_criticalSectionStop(pmicHandle);

            if (status != PMIC_ST_SUCCESS)
            {
                break;
            }
        }
    }

    return status;
}
