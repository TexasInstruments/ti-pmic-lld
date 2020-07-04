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
*   \file    pmic_power_lp8764x.c
*
*   \brief   This file contains the LP8764X Leo PMIC power Specific
*            configuration API's and structures
*
*/

#include <pmic_types.h>
#include <pmic_power.h>
#include <pmic_core_priv.h>
#include <pmic_io_priv.h>
#include <pmic_power_lp8764x_priv.h>

static Pmic_powerRsrcRegCfg_t lp8764x_pwrRsrcRegCfg[] =
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
        PMIC_LP8764X_BUCK1_FREQ_SEL_SHIFT
    },
    {
        PMIC_BUCK2_CTRL_REGADDR,
        PMIC_BUCK2_CONF_REGADDR,
        PMIC_BUCK2_VOUT_1_REGADDR,
        PMIC_BUCK2_VOUT_2_REGADDR,
        PMIC_BUCK2_PG_WIN_REGADDR,
        PMIC_MASK_BUCK1_2_REGADDR,
        PMIC_STAT_BUCK1_2_REGADDR,
        PMIC_RAIL_SEL_1_REGADDR,
        PMIC_RAIL_SEL_1_BUCK2_GRP_SEL_SHIFT,
        PMIC_POWER_INVALID_REGADDR,
        PMIC_POWER_INVALID_REGADDR,
        PMIC_LP8764X_BUCK2_FREQ_SEL_SHIFT
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
        PMIC_LP8764X_BUCK3_FREQ_SEL_SHIFT
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
        PMIC_LP8764X_BUCK1_FREQ_SEL_SHIFT
    },
    {
        PMIC_VCCA_VMON_CTRL_REGADDR,
        PMIC_VMON_CONF_REGADDR,
        PMIC_POWER_INVALID_REGADDR,
        PMIC_POWER_INVALID_REGADDR,
        PMIC_VMON1_PG_WINDOW_REGADDR,
        PMIC_MASK_VMON_REGADDR,
        PMIC_STAT_VMON_REGADDR,
        PMIC_RAIL_SEL_3_REGADDR,
        PMIC_RAIL_SEL_3_VMON1_GRP_SEL_SHIFT,
        PMIC_POWER_INVALID_REGADDR,
        PMIC_POWER_INVALID_REGADDR,
        PMIC_POWER_INVALID_REGADDR
    },
    {
        PMIC_VCCA_VMON_CTRL_REGADDR,
        PMIC_VMON_CONF_REGADDR,
        PMIC_POWER_INVALID_REGADDR,
        PMIC_POWER_INVALID_REGADDR,
        PMIC_VMON2_PG_WINDOW_REGADDR,
        PMIC_MASK_VMON_REGADDR,
        PMIC_STAT_VMON_REGADDR,
        PMIC_RAIL_SEL_3_REGADDR,
        PMIC_RAIL_SEL_3_VMON2_GRP_SEL_SHIFT,
        PMIC_POWER_INVALID_REGADDR,
        PMIC_POWER_INVALID_REGADDR,
        PMIC_POWER_INVALID_REGADDR
    }
};

/*!
 * \brief  PMIC power resources get Configuration function
 *         This function is used to read the PMIC POWER resources register
 *         Configuration
 *
 * \param  pwrRsrcRegCfg   [OUT]  Pointer to store power resource register
 *                                configuration
 */
void pmic_get_lp8764x_pwrRsrceRegCfg(Pmic_powerRsrcRegCfg_t **pPwrRsrcRegCfg)
{
    *pPwrRsrcRegCfg = lp8764x_pwrRsrcRegCfg;
}

/*!
 * \brief   This function is used to convert the millivolt value to vset value
 *          for HERA LP8764x PMIC
 */
int32_t Pmic_powerLP8764xConvertVoltage2VSetVal(
                                            Pmic_CoreHandle_t *pPmicCoreHandle,
                                            uint16_t           millivolt,
                                            uint16_t           pwrRsrc,
                                            uint16_t          *pBaseMillivolt,
                                            uint8_t           *pMillivoltStep,
                                            uint8_t           *pBaseVoutCode)
{
    int32_t status = PMIC_ST_SUCCESS;
    bool    vmonRange;
    uint8_t pwrRsrcType;

    pwrRsrcType  = Pmic_powerGetPwrRsrcType(pwrRsrc);

    switch(pwrRsrcType)
    {
        case PMIC_LP8764X_POWER_RESOURCE_TYPE_BUCK:

            status = Pmic_powerBuckVmonConvertVoltage2VSetVal(millivolt,
                                                              pBaseMillivolt,
                                                              pMillivoltStep,
                                                              pBaseVoutCode);

        break;
        case PMIC_LP8764X_POWER_RESOURCE_TYPE_VMON:
            status = Pmic_powerGetVmonRange(pPmicCoreHandle,
                                            pwrRsrc,
                                            &(vmonRange));

            if((PMIC_ST_SUCCESS == status) &&
               (PMIC_LP8764X_VMON_RANGE_0V3_3V34 == vmonRange))
            {
                status = Pmic_powerBuckVmonConvertVoltage2VSetVal(
                                                                millivolt,
                                                                pBaseMillivolt,
                                                                pMillivoltStep,
                                                                pBaseVoutCode);
            }
            else if((PMIC_ST_SUCCESS == status) &&
                    (PMIC_LP8764X_VMON_RANGE_3V35_5V == vmonRange))
            {
                status = Pmic_powerVmonRange1ConvertVoltage2VSetVal(
                                                                millivolt,
                                                                pBaseMillivolt,
                                                                pMillivoltStep,
                                                                pBaseVoutCode);
            }

            break;
        default:
            status = PMIC_ST_ERR_INV_PARAM;
            break;
    }

    return status;
}

/*!
 * \brief   This function is used to convert the vsetvalue to voltage in mv
 *          for PMIC HERA LP8764x
 */
int32_t Pmic_powerLP8764xConvertVSetVal2Voltage(
                                            Pmic_CoreHandle_t *pPmicCoreHandle,
                                            uint8_t           *pVSetVal,
                                            uint16_t           pwrRsrc,
                                            uint16_t          *pBaseMillivolt,
                                            uint8_t           *pMillivoltStep,
                                            uint8_t           *pBaseVoutCode)
{
    int32_t status = PMIC_ST_SUCCESS;
    bool    vmonRange;
    uint8_t pwrRsrcType;

    pwrRsrcType  = Pmic_powerGetPwrRsrcType(pwrRsrc);

    switch(pwrRsrcType)
    {
        case PMIC_LP8764X_POWER_RESOURCE_TYPE_BUCK:
            status = Pmic_powerBuckVmonConvertVSetVal2Voltage(pVSetVal,
                                                              pBaseMillivolt,
                                                              pMillivoltStep,
                                                              pBaseVoutCode);
            break;
        case PMIC_LP8764X_POWER_RESOURCE_TYPE_VMON:
            status = Pmic_powerGetVmonRange(pPmicCoreHandle,
                                            pwrRsrc,
                                            &(vmonRange));

            if((PMIC_ST_SUCCESS == status) &&
               (PMIC_LP8764X_VMON_RANGE_0V3_3V34 == vmonRange))
            {
                status = Pmic_powerBuckVmonConvertVSetVal2Voltage(
                                                                 pVSetVal,
                                                                 pBaseMillivolt,
                                                                 pMillivoltStep,
                                                                 pBaseVoutCode);
            }
            else if((PMIC_ST_SUCCESS == status) &&
                    (PMIC_LP8764X_VMON_RANGE_3V35_5V == vmonRange))
            {
                status = Pmic_powerVmonRange1ConvertVSetVal2Voltage(
                                                                 pVSetVal,
                                                                 pBaseMillivolt,
                                                                 pMillivoltStep,
                                                                 pBaseVoutCode);
            }

            break;
        default:
            status = PMIC_ST_ERR_INV_PARAM;
            break;
    }

    return status;
}
