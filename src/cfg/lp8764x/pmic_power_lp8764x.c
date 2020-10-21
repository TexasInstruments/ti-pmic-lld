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
        PMIC_INVALID_REGADDR,
        PMIC_INVALID_REGADDR,
        PMIC_INVALID_REGADDR,
        PMIC_VCCA_PG_WINDOW_REGADDR,
        PMIC_LP8764X_VCCA_OV_INT,
        PMIC_STAT_VMON_REGADDR,
        PMIC_INVALID_BIT_SHIFT,
        PMIC_RAIL_SEL_3_VCCA_GRP_SEL_SHIFT,
        PMIC_INVALID_BIT_SHIFT,
        PMIC_INVALID_BIT_SHIFT
    },
    {
        PMIC_BUCK1_CTRL_REGADDR,
        PMIC_BUCK1_CONF_REGADDR,
        PMIC_BUCK1_VOUT_1_REGADDR,
        PMIC_BUCK1_VOUT_2_REGADDR,
        PMIC_BUCK1_PG_WIN_REGADDR,
        PMIC_LP8764X_BUCK1_OV_INT,
        PMIC_STAT_BUCK1_2_REGADDR,
        PMIC_RAIL_SEL_1_REGADDR,
        PMIC_RAIL_SEL_1_BUCK1_GRP_SEL_SHIFT,
        PMIC_INVALID_REGADDR,
        PMIC_INVALID_BIT_SHIFT
    },
    {
        PMIC_BUCK2_CTRL_REGADDR,
        PMIC_BUCK2_CONF_REGADDR,
        PMIC_BUCK2_VOUT_1_REGADDR,
        PMIC_BUCK2_VOUT_2_REGADDR,
        PMIC_BUCK2_PG_WIN_REGADDR,
        PMIC_LP8764X_BUCK2_OV_INT,
        PMIC_STAT_BUCK1_2_REGADDR,
        PMIC_RAIL_SEL_1_REGADDR,
        PMIC_RAIL_SEL_1_BUCK2_GRP_SEL_SHIFT,
        PMIC_INVALID_REGADDR,
        PMIC_INVALID_BIT_SHIFT
    },
    {
        PMIC_BUCK3_CTRL_REGADDR,
        PMIC_BUCK3_CONF_REGADDR,
        PMIC_BUCK3_VOUT_1_REGADDR,
        PMIC_BUCK3_VOUT_2_REGADDR,
        PMIC_BUCK3_PG_WIN_REGADDR,
        PMIC_LP8764X_BUCK3_OV_INT,
        PMIC_STAT_BUCK3_4_REGADDR,
        PMIC_RAIL_SEL_1_REGADDR,
        PMIC_RAIL_SEL_1_BUCK3_GRP_SEL_SHIFT,
        PMIC_INVALID_REGADDR,
        PMIC_INVALID_BIT_SHIFT
    },
    {
        PMIC_BUCK4_CTRL_REGADDR,
        PMIC_BUCK4_CONF_REGADDR,
        PMIC_BUCK4_VOUT_1_REGADDR,
        PMIC_BUCK4_VOUT_2_REGADDR,
        PMIC_BUCK4_PG_WIN_REGADDR,
        PMIC_LP8764X_BUCK4_OV_INT,
        PMIC_STAT_BUCK3_4_REGADDR,
        PMIC_RAIL_SEL_1_REGADDR,
        PMIC_RAIL_SEL_1_BUCK4_GRP_SEL_SHIFT,
        PMIC_INVALID_REGADDR,
        PMIC_INVALID_BIT_SHIFT
    },
    {
        PMIC_VCCA_VMON_CTRL_REGADDR,
        PMIC_VMON_CONF_REGADDR,
        PMIC_INVALID_REGADDR,
        PMIC_INVALID_REGADDR,
        PMIC_VMON1_PG_WINDOW_REGADDR,
        PMIC_LP8764X_VMON1_OV_INT,
        PMIC_STAT_VMON_REGADDR,
        PMIC_RAIL_SEL_3_REGADDR,
        PMIC_RAIL_SEL_3_VMON1_GRP_SEL_SHIFT,
        PMIC_INVALID_REGADDR,
        PMIC_INVALID_BIT_SHIFT
    },
    {
        PMIC_VCCA_VMON_CTRL_REGADDR,
        PMIC_VMON_CONF_REGADDR,
        PMIC_INVALID_REGADDR,
        PMIC_INVALID_REGADDR,
        PMIC_VMON2_PG_WINDOW_REGADDR,
        PMIC_LP8764X_VMON2_OV_INT,
        PMIC_STAT_VMON_REGADDR,
        PMIC_RAIL_SEL_3_REGADDR,
        PMIC_RAIL_SEL_3_VMON2_GRP_SEL_SHIFT,
        PMIC_INVALID_REGADDR,
        PMIC_INVALID_BIT_SHIFT
    }
};

static Pmic_powerPgoodSrcRegCfg_t lp8764x_pgoodSrcRegCfg[] =
{
    {
        PMIC_PGOOD_SEL_4_REGADDR,
        PMIC_PGOOD_SEL_4_PGOOD_SEL_VCCA_SHIFT,
    },
    {
        PMIC_PGOOD_SEL_1_REGADDR,
        PMIC_PGOOD_SEL_1_PGOOD_SEL_BUCK1_SHIFT,
    },
    {
        PMIC_PGOOD_SEL_1_REGADDR,
        PMIC_PGOOD_SEL_1_PGOOD_SEL_BUCK2_SHIFT,
    },
    {
        PMIC_PGOOD_SEL_1_REGADDR,
        PMIC_PGOOD_SEL_1_PGOOD_SEL_BUCK3_SHIFT,
    },
    {
        PMIC_PGOOD_SEL_1_REGADDR,
        PMIC_PGOOD_SEL_1_PGOOD_SEL_BUCK4_SHIFT,
    },
    {
        PMIC_PGOOD_SEL_4_REGADDR,
        PMIC_PGOOD_SEL_4_PGOOD_SEL_NRSTOUT_SHIFT,
    },
    {
        PMIC_PGOOD_SEL_4_REGADDR,
        PMIC_PGOOD_SEL_4_PGOOD_SEL_NRSTOUT_SOC_SHIFT,
    },
    {
        PMIC_PGOOD_SEL_4_REGADDR,
        PMIC_PGOOD_SEL_4_PGOOD_SEL_TDIE_WARN_SHIFT,
    },
    {
        PMIC_PGOOD_SEL_4_REGADDR,
        PMIC_PGOOD_SEL_4_PGOOD_SEL_VMON1_SHIFT,
    },
    {
        PMIC_PGOOD_SEL_4_REGADDR,
        PMIC_PGOOD_SEL_4_PGOOD_SEL_VMON2_SHIFT,
    },
};

static Pmic_powerIntCfg_t lp8764x_pwrIntCfg[] =
{
    {
        PMIC_LP8764X_TWARN_INT,
    },
    {
        PMIC_LP8764X_NRSTOUT_READBACK_INT,
    },
    {
        PMIC_LP8764X_SOC_PWR_ERR_INT,
    },
    {
        PMIC_LP8764X_MCU_PWR_ERR_INT,
    },
    {
        PMIC_LP8764X_ORD_SHUTDOWN_INT,
    },
    {
        PMIC_LP8764X_IMM_SHUTOWN_INT,
    },
    {
        PMIC_LP8764X_NRSTOUT_SOC_READBACK_INT,
    },
    {
        PMIC_LP8764X_EN_DRV_READBACK_INT,
    },

};

/*!
 * \brief  PMIC power common interrupt get Configuration function
 *         This function is used to read the interrupt
 *         Configuration
 *
 * \param  pwrRsrcRegCfg   [OUT]  Pointer to store power interrupt
 *                                number.
 */
void pmic_get_lp8764x_pwrCommonIntCfg(Pmic_powerIntCfg_t **pPwrCommonIntCfg)
{
    *pPwrCommonIntCfg = lp8764x_pwrIntCfg;
}

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
 * \brief  PMIC power get Configuration function
 *         This function is used to read the PMIC pgood sources register
 *         Configuration
 *
 * \param  pPgoodSrcRegCfg   [OUT]  Pointer to store power-good source register
 *                                  configuration
 */
void pmic_get_lp8764x_pwrPgoodSrcRegCfg(
                                   Pmic_powerPgoodSrcRegCfg_t **pPgoodSrcRegCfg)
{
    *pPgoodSrcRegCfg = lp8764x_pgoodSrcRegCfg;
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

    pwrRsrcType = Pmic_powerGetPwrRsrcType(pwrRsrc);

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
            else
            {
                if((PMIC_ST_SUCCESS == status) &&
                    (PMIC_LP8764X_VMON_RANGE_3V35_5V == vmonRange))
                {
                    status = Pmic_powerVmonRange1ConvertVoltage2VSetVal(
                                                                pBaseMillivolt,
                                                                pMillivoltStep,
                                                                pBaseVoutCode);
                }
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
                                            const uint8_t     *pVSetVal,
                                            uint16_t           pwrRsrc,
                                            uint16_t          *pBaseMillivolt,
                                            uint8_t           *pMillivoltStep,
                                            uint8_t           *pBaseVoutCode)
{
    int32_t status = PMIC_ST_SUCCESS;
    bool    vmonRange = 0U;
    uint8_t pwrRsrcType = 0U;

    pwrRsrcType = Pmic_powerGetPwrRsrcType(pwrRsrc);

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
            else
            {
                if((PMIC_ST_SUCCESS == status) &&
                    (PMIC_LP8764X_VMON_RANGE_3V35_5V == vmonRange))
                {
                    status = Pmic_powerVmonRange1ConvertVSetVal2Voltage(
                                                                 pBaseMillivolt,
                                                                 pMillivoltStep,
                                                                 pBaseVoutCode);
                }
            }

            break;
        default:
            status = PMIC_ST_ERR_INV_PARAM;
            break;
    }

    return status;
}

/*!
 * \brief   This function is to validate the power good source limit for the
 *          specific PMIC device.
 */
int32_t Pmic_validate_lp8764x_pGoodSrcType(uint16_t pgoodSrc)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t  pGoodSrcType = 0U;

    pGoodSrcType = Pmic_powerGetPwrRsrcType(pgoodSrc);

    if(PMIC_LP8764X_PGOOD_SOURCE_TYPE_VCCA == pGoodSrcType)
    {
        if(pgoodSrc != PMIC_LP8764X_PGOOD_SOURCE_VCCA)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
    }
    else if(PMIC_LP8764X_PGOOD_SOURCE_TYPE_NRSTOUT == pGoodSrcType)
    {
        if(pgoodSrc != PMIC_LP8764X_PGOOD_SOURCE_NRSTOUT)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
    }
    else if(PMIC_LP8764X_PGOOD_SOURCE_TYPE_NRSTOUT_SOC == pGoodSrcType)
    {
        if(pgoodSrc != PMIC_LP8764X_PGOOD_SOURCE_NRSTOUT_SOC)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
    }
    else if(PMIC_LP8764X_PGOOD_SOURCE_TYPE_TDIE == pGoodSrcType)
    {
        if(pgoodSrc != PMIC_LP8764X_PGOOD_SOURCE_TDIE)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
    }
    else if(PMIC_LP8764X_PGOOD_SOURCE_TYPE_BUCK == pGoodSrcType)
    {
        if((pgoodSrc > PMIC_LP8764X_PGOOD_BUCK_MAX) ||
           (pgoodSrc < PMIC_LP8764X_PGOOD_BUCK_MIN))
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
    }
    else if(PMIC_LP8764X_PGOOD_SOURCE_TYPE_VMON == pGoodSrcType)
    {
        if((pgoodSrc > PMIC_LP8764X_PGOOD_VMON_MAX) ||
           (pgoodSrc < PMIC_LP8764X_PGOOD_VMON_MIN))
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
    }
    else
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    return status;
}

/*!
 * \brief   This function is to validate the power good signal source selection
 *          limit for the specific PMIC device.
 */
int32_t Pmic_validate_lp8764x_pGoodSelType(uint16_t pgoodSrc,
                                           uint8_t pgoodSelType)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t  pGoodSrcType = 0U;

    pGoodSrcType = Pmic_powerGetPwrRsrcType(pgoodSrc);
    if((PMIC_LP8764X_PGOOD_SOURCE_TYPE_VCCA == pGoodSrcType) ||
       (PMIC_LP8764X_PGOOD_SOURCE_TYPE_VMON == pGoodSrcType))
    {
        if(pgoodSelType > PMIC_LP8764X_POWER_PGOOD_SEL_VCCA_VMON_ENABLE)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
    }
    else if(PMIC_LP8764X_PGOOD_SOURCE_TYPE_NRSTOUT == pGoodSrcType)
    {
        if(pgoodSelType > PMIC_LP8764X_POWER_PGOOD_SEL_NRSTOUT)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
    }
    else if(PMIC_LP8764X_PGOOD_SOURCE_TYPE_NRSTOUT_SOC == pGoodSrcType)
    {
        if(pgoodSelType > PMIC_LP8764X_POWER_PGOOD_SEL_NRSTOUT_SOC)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
    }
    else if(PMIC_LP8764X_PGOOD_SOURCE_TYPE_TDIE == pGoodSrcType)
    {
        if(pgoodSelType > PMIC_LP8764X_POWER_PGOOD_SEL_TDIE_WARN)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
    }
    else if(PMIC_LP8764X_PGOOD_SOURCE_TYPE_BUCK == pGoodSrcType)
    {
        if(pgoodSelType > PMIC_LP8764X_POWER_PGOOD_SEL_SRC_VOLTAGE_CURRENT)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
    }
    else
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    return status;
}
