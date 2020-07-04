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
*   \file    pmic_power_tps6594x.c
*
*   \brief   This file contains the TPS6594x Leo PMIC power Specific
*            configuration API's and structures
*
*/

#include <pmic_types.h>
#include <pmic_power.h>
#include <pmic_core_priv.h>
#include <pmic_io_priv.h>
#include <pmic_power_tps6594x_priv.h>

static Pmic_powerRsrcRegCfg_t tps6594x_pwrRsrcRegCfg[] =
{
    {
        PMIC_VCCA_VMON_CTRL_REGADDR,
        PMIC_POWER_INVALID_REGADDR,
        PMIC_POWER_INVALID_REGADDR,
        PMIC_POWER_INVALID_REGADDR,
        PMIC_VCCA_PG_WINDOW_REGADDR,
        PMIC_MASK_VMON_REGADDR,
        PMIC_STAT_VMON_REGADDR,
        PMIC_RAIL_SEL_3_REGADDR,
        PMIC_RAIL_SEL_3_VCCA_GRP_SEL_SHIFT,
        PMIC_POWER_INVALID_REGADDR,
        PMIC_POWER_INVALID_REGADDR,
        PMIC_POWER_INVALID_REGADDR
    },
    {
        PMIC_BUCK1_CTRL_REGADDR,
        PMIC_BUCK1_CONF_REGADDR,
        PMIC_BUCK1_VOUT_1_REGADDR,
        PMIC_BUCK1_VOUT_2_REGADDR,
        PMIC_BUCK1_PG_WIN_REGADDR,
        PMIC_MASK_BUCK1_2_REGADDR,
        PMIC_STAT_BUCK1_2_REGADDR,
        PMIC_RAIL_SEL_1_REGADDR,
        PMIC_RAIL_SEL_1_BUCK1_GRP_SEL_SHIFT,
        PMIC_POWER_INVALID_REGADDR,
        PMIC_POWER_INVALID_REGADDR,
        PMIC_TPS6594X_BUCK1_FREQ_SEL_SHIFT
    },
    {
        PMIC_BUCK2_CTRL_REGADDR,
        PMIC_BUCK2_CONF_REGADDR,
        PMIC_BUCK2_VOUT_1_REGADDR,
        PMIC_BUCK2_VOUT_2_REGADDR,
        PMIC_BUCK2_PG_WIN_REGADDR,
        PMIC_MASK_BUCK1_2_REGADDR,
        PMIC_RAIL_SEL_1_REGADDR,
        PMIC_STAT_BUCK1_2_REGADDR,
        PMIC_RAIL_SEL_1_BUCK2_GRP_SEL_SHIFT,
        PMIC_POWER_INVALID_REGADDR,
        PMIC_POWER_INVALID_REGADDR,
        PMIC_TPS6594X_BUCK2_FREQ_SEL_SHIFT
    },
    {
        PMIC_BUCK3_CTRL_REGADDR,
        PMIC_BUCK3_CONF_REGADDR,
        PMIC_BUCK3_VOUT_1_REGADDR,
        PMIC_BUCK3_VOUT_2_REGADDR,
        PMIC_BUCK3_PG_WIN_REGADDR,
        PMIC_MASK_BUCK3_4_REGADDR,
        PMIC_STAT_BUCK3_4_REGADDR,
        PMIC_RAIL_SEL_1_REGADDR,
        PMIC_RAIL_SEL_1_BUCK3_GRP_SEL_SHIFT,
        PMIC_POWER_INVALID_REGADDR,
        PMIC_POWER_INVALID_REGADDR,
        PMIC_TPS6594X_BUCK3_FREQ_SEL_SHIFT
    },
    {
        PMIC_BUCK4_CTRL_REGADDR,
        PMIC_BUCK4_CONF_REGADDR,
        PMIC_BUCK4_VOUT_1_REGADDR,
        PMIC_BUCK4_VOUT_2_REGADDR,
        PMIC_BUCK4_PG_WIN_REGADDR,
        PMIC_MASK_BUCK3_4_REGADDR,
        PMIC_STAT_BUCK3_4_REGADDR,
        PMIC_RAIL_SEL_1_REGADDR,
        PMIC_RAIL_SEL_1_BUCK4_GRP_SEL_SHIFT,
        PMIC_POWER_INVALID_REGADDR,
        PMIC_POWER_INVALID_REGADDR,
        PMIC_TPS6594X_BUCK4_FREQ_SEL_SHIFT
    },
    {
        PMIC_BUCK5_CTRL_REGADDR,
        PMIC_BUCK5_CONF_REGADDR,
        PMIC_BUCK5_VOUT_1_REGADDR,
        PMIC_BUCK5_VOUT_2_REGADDR,
        PMIC_BUCK5_PG_WIN_REGADDR,
        PMIC_MASK_BUCK5_REGADDR,
        PMIC_STAT_BUCK5_REGADDR,
        PMIC_RAIL_SEL_2_REGADDR,
        PMIC_RAIL_SEL_2_BUCK5_GRP_SEL_SHIFT,
        PMIC_POWER_INVALID_REGADDR,
        PMIC_POWER_INVALID_REGADDR,
        PMIC_TPS6594X_BUCK5_FREQ_SEL_SHIFT
    },
    {
        PMIC_LDO1_CTRL_REGADDR,
        PMIC_POWER_INVALID_REGADDR,
        PMIC_LDO1_VOUT_REGADDR,
        PMIC_POWER_INVALID_REGADDR,
        PMIC_LDO1_PG_WIN_REGADDR,
        PMIC_MASK_LDO1_2_REGADDR,
        PMIC_STAT_LDO1_2_REGADDR,
        PMIC_RAIL_SEL_2_REGADDR,
        PMIC_RAIL_SEL_2_LDO1_GRP_SEL_SHIFT,
        PMIC_LDO_RV_TIMEOUT_REG_1_REGADDR,
        PMIC_LDO_RV_TIMEOUT_REG_LDO1_3_RV_TIMEOUT_SHIFT,
        PMIC_POWER_INVALID_REGADDR
    },
    {
        PMIC_LDO2_CTRL_REGADDR,
        PMIC_POWER_INVALID_REGADDR,
        PMIC_LDO2_VOUT_REGADDR,
        PMIC_POWER_INVALID_REGADDR,
        PMIC_LDO2_PG_WIN_REGADDR,
        PMIC_MASK_LDO1_2_REGADDR,
        PMIC_STAT_LDO1_2_REGADDR,
        PMIC_RAIL_SEL_2_REGADDR,
        PMIC_RAIL_SEL_2_LDO2_GRP_SEL_SHIFT,
        PMIC_LDO_RV_TIMEOUT_REG_1_REGADDR,
        PMIC_LDO_RV_TIMEOUT_REG_LDO2_4_RV_TIMEOUT_SHIFT,
        PMIC_POWER_INVALID_REGADDR
    },
    {
        PMIC_LDO3_CTRL_REGADDR,
        PMIC_POWER_INVALID_REGADDR,
        PMIC_LDO3_VOUT_REGADDR,
        PMIC_POWER_INVALID_REGADDR,
        PMIC_LDO3_PG_WIN_REGADDR,
        PMIC_MASK_LDO3_4_REGADDR,
        PMIC_STAT_LDO3_4_REGADDR,
        PMIC_RAIL_SEL_2_REGADDR,
        PMIC_RAIL_SEL_2_LDO3_GRP_SEL_SHIFT,
        PMIC_LDO_RV_TIMEOUT_REG_2_REGADDR,
        PMIC_LDO_RV_TIMEOUT_REG_LDO1_3_RV_TIMEOUT_SHIFT,
        PMIC_POWER_INVALID_REGADDR
    },
    {
        PMIC_LDO4_CTRL_REGADDR,
        PMIC_POWER_INVALID_REGADDR,
        PMIC_LDO4_VOUT_REGADDR,
        PMIC_POWER_INVALID_REGADDR,
        PMIC_LDO4_PG_WIN_REGADDR,
        PMIC_MASK_LDO3_4_REGADDR,
        PMIC_STAT_LDO3_4_REGADDR,
        PMIC_RAIL_SEL_3_REGADDR,
        PMIC_RAIL_SEL_3_LDO4_GRP_SEL_SHIFT,
        PMIC_LDO_RV_TIMEOUT_REG_2_REGADDR,
        PMIC_LDO_RV_TIMEOUT_REG_LDO2_4_RV_TIMEOUT_SHIFT,
        PMIC_POWER_INVALID_REGADDR
    },

};

/*!
 * \brief  PMIC power resources get Configuration function
 *         This function is used to read the PMIC POWER resources register
 *         Configuration
 *
 * \param  pwrRsrcRegCfg   [OUT]  Pointer to store power resource register
 *                                configuration
 */
void pmic_get_tps6594x_pwrRsrceRegCfg(Pmic_powerRsrcRegCfg_t **pPwrRsrcRegCfg)
{
    *pPwrRsrcRegCfg = tps6594x_pwrRsrcRegCfg;
}

/*!
 * \brief   This function is used to convert the millivolt value to vset value
 *          for LEO TPS6594x PMIC
 */
int32_t Pmic_powerTPS6594xConvertVoltage2VSetVal(uint16_t  millivolt,
                                                 uint16_t  pwrRsrc,
                                                 uint16_t *pBaseMillivolt,
                                                 uint8_t  *pMillivoltStep,
                                                 uint8_t  *pBaseVoutCode)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t pwrRsrcType;

    pwrRsrcType  = Pmic_powerGetPwrRsrcType(pwrRsrc);

    if(PMIC_TPS6594X_POWER_RESOURCE_TYPE_BUCK == pwrRsrcType)
    {
        status = Pmic_powerBuckVmonConvertVoltage2VSetVal(millivolt,
                                                          pBaseMillivolt,
                                                          pMillivoltStep,
                                                          pBaseVoutCode);

    }
    else if(PMIC_TPS6594X_POWER_RESOURCE_TYPE_LDO == pwrRsrcType)
    {
        status = Pmic_powerLdoConvertVoltage2VSetVal(millivolt,
                                                     pwrRsrc,
                                                     pBaseMillivolt,
                                                     pMillivoltStep,
                                                     pBaseVoutCode);
    }
    else
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    return status;
}

/*!
 * \brief   This function is used to convert the vset value to voltage in mv
 *          for PMIC LEO TPS6594x
 */
int32_t Pmic_powerTPS6594xConvertVSet2Voltage(uint8_t  *pVSetVal,
                                              uint16_t  pwrRsrc,
                                              uint16_t *pBaseMillivolt,
                                              uint8_t  *pMillivoltStep,
                                              uint8_t  *pBaseVoutCode)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t pwrRsrcType;

    pwrRsrcType  = Pmic_powerGetPwrRsrcType(pwrRsrc);
    if(PMIC_TPS6594X_POWER_RESOURCE_TYPE_BUCK == pwrRsrcType)
    {
        Pmic_powerBuckVmonConvertVSetVal2Voltage(pVSetVal,
                                                 pBaseMillivolt,
                                                 pMillivoltStep,
                                                 pBaseVoutCode);
    }
    else if(PMIC_TPS6594X_POWER_RESOURCE_TYPE_LDO == pwrRsrcType)
    {
        status = Pmic_powerLdoConvertVSetVal2Voltage(pVSetVal,
                                                     pwrRsrc,
                                                     pBaseMillivolt,
                                                     pMillivoltStep,
                                                     pBaseVoutCode);
    }
    else
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    return status;
}
