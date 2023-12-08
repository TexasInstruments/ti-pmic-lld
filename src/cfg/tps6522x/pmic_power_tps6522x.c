/******************************************************************************
 * Copyright (c) 2023 Texas Instruments Incorporated - http://www.ti.com
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
 *  \file   pmic_power.c
 *
 *  \brief  This source file contains TPS6522x Burton specific power API definitions
 *          and data structures.
 */

#include "../../../include/pmic_types.h"
#include "../../../include/pmic_power.h"
#include "../../pmic_core_priv.h"
#include "../../pmic_io_priv.h"
#include "pmic_power_tps6522x_priv.h"

// clang-format off
static const Pmic_powerTps6522xBuckRegisters_t gTps6522xBuckRegisters[] =
{
    {
        PMIC_BUCK1_CTRL_REGADDR,
        PMIC_BUCK1_CONF_REGADDR,
        PMIC_BUCK1_VOUT_1_REGADDR,
        PMIC_BUCK1_PG_WIN_REGADDR,
        PMIC_RAIL_SEL_1_REGADDR
    },
    {
        PMIC_BUCK2_CTRL_REGADDR,
        PMIC_BUCK2_CONF_REGADDR,
        PMIC_BUCK2_VOUT_1_REGADDR,
        PMIC_BUCK2_PG_WIN_REGADDR,
        PMIC_RAIL_SEL_1_REGADDR
    },
    {
        PMIC_BUCK3_CTRL_REGADDR,
        PMIC_BUCK3_CONF_REGADDR,
        PMIC_BUCK3_VOUT_1_REGADDR,
        PMIC_BUCK3_PG_WIN_REGADDR,
        PMIC_RAIL_SEL_1_REGADDR
    },
    {
        PMIC_BUCK4_CTRL_REGADDR,
        PMIC_BUCK4_CONF_REGADDR,
        PMIC_BUCK4_VOUT_1_REGADDR,
        PMIC_BUCK4_PG_WIN_REGADDR,
        PMIC_RAIL_SEL_1_REGADDR
    }
};

static const Pmic_powerTps6522xLdoRegisters_t gTps6522xLdoRegisters[] =
{
    {
        PMIC_LDO1_CTRL_REGADDR, 
        PMIC_LDO1_VOUT_REGADDR,
        PMIC_LDO1_PG_WIN_REGADDR, 
        PMIC_RAIL_SEL_2_REGADDR
    },
    {
        PMIC_LDO2_CTRL_REGADDR, 
        PMIC_LDO2_VOUT_REGADDR,
        PMIC_LDO2_PG_WIN_REGADDR, 
        PMIC_RAIL_SEL_2_REGADDR
    },
    {
        PMIC_LDO3_CTRL_REGADDR, 
        PMIC_LDO3_VOUT_REGADDR,
        PMIC_LDO3_PG_WIN_REGADDR, 
        PMIC_RAIL_SEL_2_REGADDR
    }
};

static const Pmic_powerTps6522xVccaVmonRegisters_t gTps6522xVccaVmonRegisters[] =
{
    {
        PMIC_TPS6522X_VCCA_VMON_CTRL_REGADDR, 
        PMIC_TPS6522X_INVALID_REGADDR,
        PMIC_TPS6522X_VMON1_PG_WINDOW_REGADDR,
        PMIC_TPS6522X_VMON1_PG_LEVEL_REGADDR, 
        PMIC_TPS6522X_RAIL_SEL_3_REGADDR
    },
    {
        PMIC_TPS6522X_VCCA_VMON_CTRL_REGADDR, 
        PMIC_TPS6522X_INVALID_REGADDR,
        PMIC_TPS6522X_VMON2_PG_WINDOW_REGADDR,
        PMIC_TPS6522X_VMON2_PG_LEVEL_REGADDR, 
        PMIC_TPS6522X_RAIL_SEL_3_REGADDR
    },
    {
        PMIC_TPS6522X_VCCA_VMON_CTRL_REGADDR, 
        PMIC_TPS6522X_VCCA_PG_WINDOW_REGADDR,
        PMIC_TPS6522X_INVALID_REGADDR,
        PMIC_TPS6522X_INVALID_REGADDR,
        PMIC_TPS6522X_RAIL_SEL_3_REGADDR
    }
};
// clang-format on

void Pmic_get_tps6522x_pwrBuckRegs(const Pmic_powerTps6522xBuckRegisters_t **pBuckRegisters)
{
    *pBuckRegisters = gTps6522xBuckRegisters;
}

void Pmic_get_tps6522x_pwrLdoRegs(const Pmic_powerTps6522xLdoRegisters_t **pLdoRegisters)
{
    *pLdoRegisters = gTps6522xLdoRegisters;
}

void Pmic_get_tps6522x_PwrVccaVmonRegisters(const Pmic_powerTps6522xVccaVmonRegisters_t **pVccaVmonRegisters)
{
    *pVccaVmonRegisters = gTps6522xVccaVmonRegisters;
}

static int32_t Pmic_powerTps6522xParamCheck_pPwrRsrcCfg(Pmic_CoreHandle_t                    *pPmicCoreHandle,
                                                        Pmic_powerTps6522xPowerResourceCfg_t *pPwrResourceCfg)
{
    int32_t status = PMIC_ST_SUCCESS;

    // NULL parameter check
    if ((pPmicCoreHandle == NULL) || (pPwrResourceCfg == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    // TPS6522x device type check
    if ((status == PMIC_ST_SUCCESS) && (pPmicCoreHandle->pmicDeviceType != PMIC_DEV_BURTON_TPS6522X))
    {
        status = PMIC_ST_ERR_INV_DEVICE;
    }

    // TPS6522x power subsystem check
    if ((status == PMIC_ST_SUCCESS) && ((pPmicCoreHandle->pPmic_SubSysInfo->buckEnable == false) ||
                                        (pPmicCoreHandle->pPmic_SubSysInfo->ldoEnable == false)))
    {
        status = PMIC_ST_ERR_INV_SUBSYSTEM;
    }

    // Power Resource CFG validParam check
    if ((status == PMIC_ST_SUCCESS) && (pPwrResourceCfg->validParams == 0))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    return status;
}

static int32_t Pmic_powerTps6522xParamCheck_constPwrRsrcCfg(Pmic_CoreHandle_t                         *pPmicCoreHandle,
                                                            const Pmic_powerTps6522xPowerResourceCfg_t pPwrResourceCfg)
{
    int32_t status = PMIC_ST_SUCCESS;

    // NULL parameter check
    if (pPmicCoreHandle == NULL)
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    // TPS6522x device type check
    if ((status == PMIC_ST_SUCCESS) && (pPmicCoreHandle->pmicDeviceType != PMIC_DEV_BURTON_TPS6522X))
    {
        status = PMIC_ST_ERR_INV_DEVICE;
    }

    // TPS6522x power subsystem check
    if ((status == PMIC_ST_SUCCESS) && ((pPmicCoreHandle->pPmic_SubSysInfo->buckEnable == false) ||
                                        (pPmicCoreHandle->pPmic_SubSysInfo->ldoEnable == false)))
    {
        status = PMIC_ST_ERR_INV_SUBSYSTEM;
    }

    // Power Resource CFG validParam check
    if ((status == PMIC_ST_SUCCESS) && (pPwrResourceCfg.validParams == 0))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    return status;
}

static int32_t
Pmic_powerTps6522xBuckVoltageWithinRangeCheck(const Pmic_powerTps6522xBuckPowerResourceCfg_t *buckPwrRsrcCfg,
                                              const uint8_t                                   buckNum)
{
    int32_t status = PMIC_ST_SUCCESS;

    switch (buckNum)
    {
        case PMIC_POWER_TPS6522X_REGULATOR_BUCK1:
            if ((buckPwrRsrcCfg[buckNum].buckVoltage_mv < PMIC_POWER_TPS6522X_BUCK1_VOLTAGE_MIN_500_MV) ||
                (buckPwrRsrcCfg[buckNum].buckVoltage_mv > PMIC_POWER_TPS6522X_BUCK1_VOLTAGE_MAX_3300_MV))
            {
                status = PMIC_ST_ERR_INV_VOLTAGE;
            }

            break;
        case PMIC_POWER_TPS6522X_REGULATOR_BUCK2:
        case PMIC_POWER_TPS6522X_REGULATOR_BUCK3:
        case PMIC_POWER_TPS6522X_REGULATOR_BUCK4:
            if ((buckPwrRsrcCfg[buckNum].buckVoltage_mv < PMIC_POWER_TPS6522X_BUCK2_3_4_VOLTAGE_MIN_500_MV) ||
                (buckPwrRsrcCfg[buckNum].buckVoltage_mv > PMIC_POWER_TPS6522X_BUCK2_3_4_VOLTAGE_MAX_3300_MV))
            {
                status = PMIC_ST_ERR_INV_VOLTAGE;
            }

            break;
    }

    return status;
}

static int32_t
Pmic_powerTps6522xLdoVoltageWithinRangeCheck(const Pmic_powerTps6522xLdoPowerResourceCfg_t *ldoPwrRsrcCfg,
                                             const uint8_t                                  ldoNum)
{
    int32_t status = PMIC_ST_SUCCESS;

    switch (ldoNum)
    {
        case PMIC_POWER_TPS6522X_REGULATOR_LDO1:
            if ((ldoPwrRsrcCfg[ldoNum].ldoVoltage_mv < PMIC_POWER_TPS6522X_LDO1_VOLTAGE_MIN_1200_MV) ||
                (ldoPwrRsrcCfg[ldoNum].ldoVoltage_mv > PMIC_POWER_TPS6522X_LDO1_VOLTAGE_MAX_3300_MV))
            {
                status = PMIC_ST_ERR_INV_VOLTAGE;
            }

            break;
        case PMIC_POWER_TPS6522X_REGULATOR_LDO2:
        case PMIC_POWER_TPS6522X_REGULATOR_LDO3:
            if ((ldoPwrRsrcCfg[ldoNum].ldoVoltage_mv < PMIC_POWER_TPS6522X_LDO2_3_VOLTAGE_MIN_600_MV) ||
                (ldoPwrRsrcCfg[ldoNum].ldoVoltage_mv > PMIC_POWER_TPS6522X_LDO2_3_VOLTAGE_MAX_3400_MV))
            {
                status = PMIC_ST_ERR_INV_VOLTAGE;
            }

            break;
    }

    return status;
}

static int32_t
Pmic_powerTps6522xVmonVoltageWithinRangeCheck(const Pmic_powerTps6522xVccaVmonPowerResourceCfg_t vccaVmonPwrRsrcCfg,
                                              const uint8_t                                      vmonNum)
{
    int32_t status = PMIC_ST_SUCCESS;

    switch (vmonNum)
    {
        case PMIC_POWER_TPS6522X_VOLTAGE_MONITOR_VMON1:
            if ((vccaVmonPwrRsrcCfg.vmon1PgLevel_mv < PMIC_POWER_TPS6522X_VMON1_VOLTAGE_MIN_500_MV) ||
                (vccaVmonPwrRsrcCfg.vmon1PgLevel_mv > PMIC_POWER_TPS6522X_VMON1_VOLTAGE_MAX_3340_MV))
            {
                status = PMIC_ST_ERR_INV_VOLTAGE;
            }

            break;
        case PMIC_POWER_TPS6522X_VOLTAGE_MONITOR_VMON2:
            if ((vccaVmonPwrRsrcCfg.vmon2PgLevel_mv < PMIC_POWER_TPS6522X_VMON2_VOLTAGE_MIN_500_MV) ||
                (vccaVmonPwrRsrcCfg.vmon2PgLevel_mv > PMIC_POWER_TPS6522X_VMON2_VOLTAGE_MAX_3300_MV))
            {
                status = PMIC_ST_ERR_INV_VOLTAGE;
            }

            break;
    }

    return status;
}

static int32_t Pmic_powerTps6522xVoltageWithinRangeCheck(const Pmic_powerTps6522xPowerResourceCfg_t pwrResourceCfg)
{
    uint8_t  iter = 0;
    uint16_t validParam = 0;
    int32_t  status = PMIC_ST_SUCCESS;

    // For each BUCK...
    for (iter = 0; iter < PMIC_POWER_TPS6522X_MAX_BUCK_NUM; iter++)
    {
        // Calculate BUCK validParam
        validParam = PMIC_POWER_TPS6522X_CFG_BUCK1_VALID + iter;

        // If BUCK validParam and its voltage_mv validParam are set...
        if (pmic_validParamCheck(pwrResourceCfg.validParams, validParam) &&
            pmic_validParamCheck(pwrResourceCfg.buckPwrRsrcCfg[iter].validParams,
                                 PMIC_POWER_TPS6522X_CFG_BUCK_VOLTAGE_MV_VALID))
        {
            // Check that BUCK voltage is within range
            status = Pmic_powerTps6522xBuckVoltageWithinRangeCheck(pwrResourceCfg.buckPwrRsrcCfg, iter);
        }

        // If BUCK voltage not within range, break
        if (status != PMIC_ST_SUCCESS)
        {
            break;
        }
    }

    // For each LDO...
    for (iter = 0; iter < PMIC_POWER_TPS6522X_MAX_LDO_NUM; iter++)
    {
        // Calculate LDO validParam
        validParam = PMIC_POWER_TPS6522X_CFG_LDO1_VALID + iter;

        // If LDO validParam and its voltage_mv validParam are set...
        if (pmic_validParamCheck(pwrResourceCfg.validParams, validParam) &&
            pmic_validParamCheck(pwrResourceCfg.ldoPwrRsrcCfg[iter].validParams,
                                 PMIC_POWER_TPS6522X_CFG_LDO_VOLTAGE_MV_VALID))
        {
            // Check that LDO voltage is within range
            status = Pmic_powerTps6522xLdoVoltageWithinRangeCheck(pwrResourceCfg.ldoPwrRsrcCfg, iter);
        }

        // If LDO voltage not within range, break
        if (status != PMIC_ST_SUCCESS)
        {
            break;
        }
    }

    // For each VMONx...
    for (iter = 0; iter < PMIC_POWER_TPS6522X_MAX_LDO_NUM - 1; iter++)
    {
        // Calculate VMONx validParam
        validParam = PMIC_POWER_TPS6522X_CFG_VMON1_VALID + iter;

        // If VMONx validParam is set...
        if (pmic_validParamCheck(pwrResourceCfg.validParams, validParam))
        {
            // Calculate VMONx PG_LEVEL_MV validParam
            validParam = (iter == PMIC_POWER_TPS6522X_VOLTAGE_MONITOR_VMON1) ?
                             PMIC_POWER_TPS6522X_CFG_VMON1_PG_LEVEL_MV_VALID :
                             PMIC_POWER_TPS6522X_CFG_VMON2_PG_LEVEL_MV_VALID;

            // If VMONx PG_LEVEL_MV validParam is set...
            if (pmic_validParamCheck(pwrResourceCfg.vccaVmonPwrRsrcCfg.validParams, validParam))
            {
                // Check that VMONx voltage is within range
                status = Pmic_powerTps6522xVmonVoltageWithinRangeCheck(pwrResourceCfg.vccaVmonPwrRsrcCfg, iter);
            }
        }

        // If VMONx voltage not within range, break
        if (status != PMIC_ST_SUCCESS)
        {
            break;
        }
    }

    return status;
}

static void Pmic_powerTps6522xGetBuckCtrlRegBitFields(Pmic_powerTps6522xBuckPowerResourceCfg_t *pBuckPowerResourceCfg,
                                                      const uint8_t                             buckCtrlRegData)
{
    // Get bitfields from the BUCK_CTRL register
    if (pmic_validParamCheck(pBuckPowerResourceCfg->validParams, PMIC_POWER_TPS6522X_CFG_BUCK_PLDN_VALID))
    {
        pBuckPowerResourceCfg->buckPldn = (Pmic_powerTps6522xBuckPldnEn_t)Pmic_getBitField(
            buckCtrlRegData, PMIC_POWER_TPS6522X_BUCK_CTRL_PLDN_SHIFT, PMIC_POWER_TPS6522X_BUCK_CTRL_PLDN_MASK);
    }
    if (pmic_validParamCheck(pBuckPowerResourceCfg->validParams, PMIC_POWER_TPS6522X_CFG_BUCK_VMON_EN_VALID))
    {
        pBuckPowerResourceCfg->buckVmonEn = (Pmic_powerTps6522xBuckVmonEn_t)Pmic_getBitField(
            buckCtrlRegData, PMIC_POWER_TPS6522X_BUCK_CTRL_VMON_EN_SHIFT, PMIC_POWER_TPS6522X_BUCK_CTRL_VMON_EN_MASK);
    }
    if (pmic_validParamCheck(pBuckPowerResourceCfg->validParams, PMIC_POWER_TPS6522X_CFG_BUCK_PWM_OPTION_VALID))
    {
        pBuckPowerResourceCfg->buckPwmOption =
            (Pmic_powerTps6522xBuckPwmOption_t)Pmic_getBitField(buckCtrlRegData,
                                                                PMIC_POWER_TPS6522X_BUCK_CTRL_PWM_OPTION_SHIFT,
                                                                PMIC_POWER_TPS6522X_BUCK_CTRL_PWM_OPTION_MASK);
    }
    if (pmic_validParamCheck(pBuckPowerResourceCfg->validParams, PMIC_POWER_TPS6522X_CFG_BUCK_EN_VALID))
    {
        pBuckPowerResourceCfg->buckEn = (Pmic_powerTps6522xBuckEn_t)Pmic_getBitField(
            buckCtrlRegData, PMIC_POWER_TPS6522X_BUCK_CTRL_EN_SHIFT, PMIC_POWER_TPS6522X_BUCK_CTRL_EN_MASK);
    }
}

static void Pmic_powerTps6522xGetBuckConfRegBitFields(Pmic_powerTps6522xBuckPowerResourceCfg_t *pBuckPowerResourceCfg,
                                                      const uint8_t                             buckConfRegData)
{
    if (pmic_validParamCheck(pBuckPowerResourceCfg->validParams, PMIC_POWER_TPS6522X_CFG_BUCK_SLEW_RATE_VALID))
    {
        pBuckPowerResourceCfg->buckSlewRate =
            (Pmic_powerTps6522xBuckSlewRate_t)Pmic_getBitField(buckConfRegData,
                                                               PMIC_POWER_TPS6522X_BUCK_CONF_SLEW_RATE_SHIFT,
                                                               PMIC_POWER_TPS6522X_BUCK_CONF_SLEW_RATE_MASK);
    }
}

static void Pmic_powerTps6522xBuckConvertVsetVal2Voltage(uint16_t     *buckVoltage_mv,
                                                         const uint8_t buckVoltage_vset,
                                                         const uint8_t buckNum)
{
    uint16_t baseVoltage_mv = 0, baseVoltage_vset = 0, voltageStep = 0;

    switch (buckNum)
    {
        case PMIC_POWER_TPS6522X_REGULATOR_BUCK1:
            if ((buckVoltage_vset >= PMIC_POWER_TPS6522X_BUCK1_VOLTAGE_RANGE_1_MIN_VSET) &&
                (buckVoltage_vset <= PMIC_POWER_TPS6522X_BUCK1_VOLTAGE_RANGE_1_MAX_VSET))
            {
                baseVoltage_mv = PMIC_POWER_TPS6522X_BUCK1_VOLTAGE_RANGE_1_MIN_VOLTAGE;
                baseVoltage_vset = PMIC_POWER_TPS6522X_BUCK1_VOLTAGE_RANGE_1_MIN_VSET;
                voltageStep = PMIC_POWER_TPS6522X_VOLTAGE_STEP_20_MV;
            }
            else if ((buckVoltage_vset >= PMIC_POWER_TPS6522X_BUCK1_VOLTAGE_RANGE_2_MIN_VSET) &&
                     (buckVoltage_vset <= PMIC_POWER_TPS6522X_BUCK1_VOLTAGE_RANGE_2_MAX_VSET))
            {
                baseVoltage_mv = PMIC_POWER_TPS6522X_BUCK1_VOLTAGE_RANGE_2_MIN_VOLTAGE;
                baseVoltage_vset = PMIC_POWER_TPS6522X_BUCK1_VOLTAGE_RANGE_2_MIN_VSET;
                voltageStep = PMIC_POWER_TPS6522X_VOLTAGE_STEP_5_MV;
            }
            else if ((buckVoltage_vset >= PMIC_POWER_TPS6522X_BUCK1_VOLTAGE_RANGE_3_MIN_VSET) &&
                     (buckVoltage_vset <= PMIC_POWER_TPS6522X_BUCK1_VOLTAGE_RANGE_3_MAX_VSET))
            {
                baseVoltage_mv = PMIC_POWER_TPS6522X_BUCK1_VOLTAGE_RANGE_3_MIN_VOLTAGE;
                baseVoltage_vset = PMIC_POWER_TPS6522X_BUCK1_VOLTAGE_RANGE_3_MIN_VSET;
                voltageStep = PMIC_POWER_TPS6522X_VOLTAGE_STEP_10_MV;
            }
            else if ((buckVoltage_vset >= PMIC_POWER_TPS6522X_BUCK1_VOLTAGE_RANGE_4_MIN_VSET) &&
                     (buckVoltage_vset <= PMIC_POWER_TPS6522X_BUCK1_VOLTAGE_RANGE_4_MAX_VSET))
            {
                baseVoltage_mv = PMIC_POWER_TPS6522X_BUCK1_VOLTAGE_RANGE_4_MIN_VOLTAGE;
                baseVoltage_vset = PMIC_POWER_TPS6522X_BUCK1_VOLTAGE_RANGE_4_MIN_VSET;
                voltageStep = PMIC_POWER_TPS6522X_VOLTAGE_STEP_20_MV;
            }

            break;
        case PMIC_POWER_TPS6522X_REGULATOR_BUCK2:
        case PMIC_POWER_TPS6522X_REGULATOR_BUCK3:
        case PMIC_POWER_TPS6522X_REGULATOR_BUCK4:
            if (buckVoltage_vset <= PMIC_POWER_TPS6522X_BUCK2_3_4_VOLTAGE_RANGE_1_MAX_VSET)
            {
                baseVoltage_mv = PMIC_POWER_TPS6522X_BUCK2_3_4_VOLTAGE_RANGE_1_MIN_VOLTAGE;
                baseVoltage_vset = PMIC_POWER_TPS6522X_BUCK2_3_4_VOLTAGE_RANGE_1_MIN_VSET;
                voltageStep = PMIC_POWER_TPS6522X_VOLTAGE_STEP_25_MV;
            }
            else if ((buckVoltage_vset >= PMIC_POWER_TPS6522X_BUCK2_3_4_VOLTAGE_RANGE_2_MIN_VSET) &&
                     (buckVoltage_vset <= PMIC_POWER_TPS6522X_BUCK2_3_4_VOLTAGE_RANGE_2_MAX_VSET))
            {
                baseVoltage_mv = PMIC_POWER_TPS6522X_BUCK2_3_4_VOLTAGE_RANGE_2_MIN_VOLTAGE;
                baseVoltage_vset = PMIC_POWER_TPS6522X_BUCK2_3_4_VOLTAGE_RANGE_2_MIN_VSET;
                voltageStep = PMIC_POWER_TPS6522X_VOLTAGE_STEP_50_MV;
            }

            break;
    }

    if ((baseVoltage_mv != 0) && (voltageStep != 0))
    {
        *buckVoltage_mv = baseVoltage_mv + (voltageStep * (buckVoltage_vset - baseVoltage_vset));
    }
}

static void Pmic_powerTps6522xGetBuckVoutRegBitFields(Pmic_powerTps6522xBuckPowerResourceCfg_t *pBuckPowerResourceCfg,
                                                      const uint8_t                             buckVoutRegData,
                                                      const uint8_t                             buckNum)
{
    if (pmic_validParamCheck(pBuckPowerResourceCfg->validParams, PMIC_POWER_TPS6522X_CFG_BUCK_VOLTAGE_MV_VALID))
    {
        Pmic_powerTps6522xBuckConvertVsetVal2Voltage(
            &(pBuckPowerResourceCfg->buckVoltage_mv), buckVoutRegData, buckNum);
    }
}

static void
Pmic_powerTps6522xGetBuckPgWindowRegBitFields(Pmic_powerTps6522xBuckPowerResourceCfg_t *pBuckPowerResourceCfg,
                                              const uint8_t                             buckPgWindowRegData)
{
    if (pmic_validParamCheck(pBuckPowerResourceCfg->validParams, PMIC_POWER_TPS6522X_CFG_BUCK_VMON_THR_VALID))
    {
        pBuckPowerResourceCfg->buckVmonThr =
            (Pmic_powerTps6522xBuckVmonThr_t)Pmic_getBitField(buckPgWindowRegData,
                                                              PMIC_POWER_TPS6522X_BUCK_PG_WINDOW_VMON_THR_SHIFT,
                                                              PMIC_POWER_TPS6522X_BUCK_PG_WINDOW_VMON_THR_MASK);
    }
}

static void Pmic_PowerTps6522xGetRailSel1RegBitFields(Pmic_powerTps6522xBuckPowerResourceCfg_t *pBuckPowerResourceCfg,
                                                      const uint8_t                             buckRailSelRegData,
                                                      const uint8_t                             buckNum)
{
    if (pmic_validParamCheck(pBuckPowerResourceCfg->validParams, PMIC_POWER_TPS6522X_CFG_BUCK_RAIL_GRP_SEL_VALID))
    {
        switch (buckNum)
        {
            case PMIC_POWER_TPS6522X_REGULATOR_BUCK1:
                pBuckPowerResourceCfg->buckRailGrpSel = (Pmic_powerTps6522xBuckRailSel_t)Pmic_getBitField(
                    buckRailSelRegData,
                    PMIC_POWER_TPS6522X_CFG_RAIL_SEL_1_BUCK1_GRP_SEL_SHIFT,
                    PMIC_POWER_TPS6522X_CFG_RAIL_SEL_1_BUCK1_GRP_SEL_MASK);

                break;
            case PMIC_POWER_TPS6522X_REGULATOR_BUCK2:
                pBuckPowerResourceCfg->buckRailGrpSel = (Pmic_powerTps6522xBuckRailSel_t)Pmic_getBitField(
                    buckRailSelRegData,
                    PMIC_POWER_TPS6522X_CFG_RAIL_SEL_1_BUCK2_GRP_SEL_SHIFT,
                    PMIC_POWER_TPS6522X_CFG_RAIL_SEL_1_BUCK2_GRP_SEL_MASK);

                break;
            case PMIC_POWER_TPS6522X_REGULATOR_BUCK3:
                pBuckPowerResourceCfg->buckRailGrpSel = (Pmic_powerTps6522xBuckRailSel_t)Pmic_getBitField(
                    buckRailSelRegData,
                    PMIC_POWER_TPS6522X_CFG_RAIL_SEL_1_BUCK3_GRP_SEL_SHIFT,
                    PMIC_POWER_TPS6522X_CFG_RAIL_SEL_1_BUCK3_GRP_SEL_MASK);

                break;
            case PMIC_POWER_TPS6522X_REGULATOR_BUCK4:
                pBuckPowerResourceCfg->buckRailGrpSel = (Pmic_powerTps6522xBuckRailSel_t)Pmic_getBitField(
                    buckRailSelRegData,
                    PMIC_POWER_TPS6522X_CFG_RAIL_SEL_1_BUCK4_GRP_SEL_SHIFT,
                    PMIC_POWER_TPS6522X_CFG_RAIL_SEL_1_BUCK4_GRP_SEL_MASK);

                break;
        }
    }
}

static int32_t Pmic_powerTps6522xGetBuckPwrResourceCfg(Pmic_CoreHandle_t                        *pPmicCoreHandle,
                                                       Pmic_powerTps6522xBuckPowerResourceCfg_t *pBuckPowerResourceCfg,
                                                       const uint8_t                             buckNum)
{
    // clang-format off
    uint8_t buckCtrlRegData     = 0;
    uint8_t buckConfRegData     = 0;
    uint8_t buckVoutRegData     = 0;
    uint8_t buckPgWindowRegData = 0;
    uint8_t buckRailSelRegData  = 0;
    int32_t status              = PMIC_ST_SUCCESS;
    const Pmic_powerTps6522xBuckRegisters_t *pBuckRegisters = NULL;
    // clang-format on

    // Parameter check
    if ((pBuckPowerResourceCfg + buckNum) == NULL)
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }
    if ((status == PMIC_ST_SUCCESS) && (pBuckPowerResourceCfg[buckNum].validParams == 0))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    // Get all BUCK resource registers
    Pmic_get_tps6522x_pwrBuckRegs(&pBuckRegisters);

    // As we are about to read, start critical section
    Pmic_criticalSectionStart(pPmicCoreHandle);

    // Read all registers pertaining to the BUCK resource
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_commIntf_recvByte(pPmicCoreHandle, pBuckRegisters[buckNum].buckCtrlRegAddr, &buckCtrlRegData);
    }
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_commIntf_recvByte(pPmicCoreHandle, pBuckRegisters[buckNum].buckConfRegAddr, &buckConfRegData);
    }
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_commIntf_recvByte(pPmicCoreHandle, pBuckRegisters[buckNum].buckVoutRegAddr, &buckVoutRegData);
    }
    if (status == PMIC_ST_SUCCESS)
    {
        status =
            Pmic_commIntf_recvByte(pPmicCoreHandle, pBuckRegisters[buckNum].buckPgWindowRegAddr, &buckPgWindowRegData);
    }
    if (status == PMIC_ST_SUCCESS)
    {
        status =
            Pmic_commIntf_recvByte(pPmicCoreHandle, pBuckRegisters[buckNum].buckRailSelRegAddr, &buckRailSelRegData);
    }

    // After read, stop critical section
    Pmic_criticalSectionStop(pPmicCoreHandle);

    // For each register pertaining to the BUCK resource, get the relevant bit fields of the register
    // (relevant bit fields are governed by validParams)
    if (status == PMIC_ST_SUCCESS)
    {
        // BUCK_CTRL register
        Pmic_powerTps6522xGetBuckCtrlRegBitFields(pBuckPowerResourceCfg + buckNum, buckCtrlRegData);

        // BUCK_CONF register
        Pmic_powerTps6522xGetBuckConfRegBitFields(pBuckPowerResourceCfg + buckNum, buckConfRegData);

        // BUCK_VOUT register
        Pmic_powerTps6522xGetBuckVoutRegBitFields(pBuckPowerResourceCfg + buckNum, buckVoutRegData, buckNum);

        // BUCK_PG_WINDOW register
        Pmic_powerTps6522xGetBuckPgWindowRegBitFields(pBuckPowerResourceCfg + buckNum, buckPgWindowRegData);

        // RAIL_SEL_1 register
        Pmic_PowerTps6522xGetRailSel1RegBitFields(pBuckPowerResourceCfg + buckNum, buckRailSelRegData, buckNum);
    }

    return status;
}

static void Pmic_powerTps6522xGetLdoCtrlRegBitFields(Pmic_powerTps6522xLdoPowerResourceCfg_t *pLdoPowerResourceCfg,
                                                     const uint8_t                            ldoCtrlRegData)
{
    if (pmic_validParamCheck(pLdoPowerResourceCfg->validParams, PMIC_POWER_TPS6522X_CFG_LDO_DISCHARGE_EN_VALID))
    {
        pLdoPowerResourceCfg->ldoDischargeEn =
            (Pmic_powerTps6522xLdoDischargeEn_t)Pmic_getBitField(ldoCtrlRegData,
                                                                 PMIC_POWER_TPS6522X_LDO_CTRL_DISCHARGE_EN_SHIFT,
                                                                 PMIC_POWER_TPS6522X_LDO_CTRL_DISCHARGE_EN_MASK);
    }
    if (pmic_validParamCheck(pLdoPowerResourceCfg->validParams, PMIC_POWER_TPS6522X_CFG_LDO_VMON_EN_VALID))
    {
        pLdoPowerResourceCfg->ldoVmonEn = (Pmic_powerTps6522xLdoVmonEn_t)Pmic_getBitField(
            ldoCtrlRegData, PMIC_POWER_TPS6522X_LDO_CTRL_VMON_EN_SHIFT, PMIC_POWER_TPS6522X_LDO_CTRL_VMON_EN_MASK);
    }
    if (pmic_validParamCheck(pLdoPowerResourceCfg->validParams, PMIC_POWER_TPS6522X_CFG_LDO_EN_VALID))
    {
        pLdoPowerResourceCfg->ldoEn = (Pmic_powerTps6522xLdoEn_t)Pmic_getBitField(
            ldoCtrlRegData, PMIC_POWER_TPS6522X_LDO_CTRL_EN_SHIFT, PMIC_POWER_TPS6522X_LDO_CTRL_EN_MASK);
    }
}

static void Pmic_powerTps6522xLdoConvertVsetVal2Voltage(uint16_t     *ldoVoltage_mv,
                                                        const uint8_t ldoVoltage_vset,
                                                        const uint8_t ldoNum)
{
    uint16_t baseVoltage_mv = 0, baseVoltage_vset = 0, voltageStep = 0;

    switch (ldoNum)
    {
        case PMIC_POWER_TPS6522X_REGULATOR_LDO1:
            if (ldoVoltage_vset <= PMIC_POWER_TPS6522X_LDO1_VOLTAGE_RANGE_1_MAX_VSET)
            {
                baseVoltage_mv = PMIC_POWER_TPS6522X_LDO1_VOLTAGE_RANGE_1_MIN_VOLTAGE;
                baseVoltage_vset = PMIC_POWER_TPS6522X_LDO1_VOLTAGE_RANGE_1_MIN_VSET;
                voltageStep = PMIC_POWER_TPS6522X_VOLTAGE_STEP_0_MV;
            }
            else if ((ldoVoltage_vset >= PMIC_POWER_TPS6522X_LDO1_VOLTAGE_RANGE_2_MIN_VSET) &&
                     (ldoVoltage_vset <= PMIC_POWER_TPS6522X_LDO1_VOLTAGE_RANGE_2_MAX_VSET))
            {
                baseVoltage_mv = PMIC_POWER_TPS6522X_LDO1_VOLTAGE_RANGE_2_MIN_VOLTAGE;
                baseVoltage_vset = PMIC_POWER_TPS6522X_LDO1_VOLTAGE_RANGE_2_MIN_VSET;
                voltageStep = PMIC_POWER_TPS6522X_VOLTAGE_STEP_50_MV;
            }
            else if ((ldoVoltage_vset >= PMIC_POWER_TPS6522X_LDO1_VOLTAGE_RANGE_3_MIN_VSET) &&
                     (ldoVoltage_vset <= PMIC_POWER_TPS6522X_LDO1_VOLTAGE_RANGE_3_MAX_VSET))
            {
                baseVoltage_mv = PMIC_POWER_TPS6522X_LDO1_VOLTAGE_RANGE_3_MIN_VOLTAGE;
                baseVoltage_vset = PMIC_POWER_TPS6522X_LDO1_VOLTAGE_RANGE_3_MIN_VSET;
                voltageStep = PMIC_POWER_TPS6522X_VOLTAGE_STEP_0_MV;
            }

            break;
        case PMIC_POWER_TPS6522X_REGULATOR_LDO2:
        case PMIC_POWER_TPS6522X_REGULATOR_LDO3:
            if (ldoVoltage_vset <= PMIC_POWER_TPS6522X_LDO2_3_VOLTAGE_RANGE_1_MAX_VSET)
            {
                baseVoltage_mv = PMIC_POWER_TPS6522X_LDO2_3_VOLTAGE_RANGE_1_MIN_VOLTAGE;
                baseVoltage_vset = PMIC_POWER_TPS6522X_LDO2_3_VOLTAGE_RANGE_1_MIN_VSET;
                voltageStep = PMIC_POWER_TPS6522X_VOLTAGE_STEP_50_MV;
            }
            else if ((ldoVoltage_vset >= PMIC_POWER_TPS6522X_LDO2_3_VOLTAGE_RANGE_2_MIN_VSET) &&
                     (ldoVoltage_vset <= PMIC_POWER_TPS6522X_LDO2_3_VOLTAGE_RANGE_2_MAX_VSET))
            {
                baseVoltage_mv = PMIC_POWER_TPS6522X_LDO2_3_VOLTAGE_RANGE_2_MIN_VOLTAGE;
                baseVoltage_vset = PMIC_POWER_TPS6522X_LDO2_3_VOLTAGE_RANGE_2_MIN_VSET;
                voltageStep = PMIC_POWER_TPS6522X_VOLTAGE_STEP_0_MV;
            }

            break;
    }

    if (baseVoltage_mv != 0)
    {
        *ldoVoltage_mv = baseVoltage_mv + (voltageStep * (ldoVoltage_vset - baseVoltage_vset));
    }
}

static void Pmic_powerTps6522xGetLdoVoutRegBitFields(Pmic_powerTps6522xLdoPowerResourceCfg_t *pLdoPowerResourceCfg,
                                                     const uint8_t                            ldoVoutRegData,
                                                     const uint8_t                            ldoNum)
{
    uint8_t ldoVoltage_vset = 0;

    if (pmic_validParamCheck(pLdoPowerResourceCfg->validParams, PMIC_POWER_TPS6522X_CFG_LDO_MODE_VALID))
    {
        pLdoPowerResourceCfg->ldoMode = (Pmic_powerTps6522xLdoBypassConfig_t)Pmic_getBitField(
            ldoVoutRegData, PMIC_POWER_TPS6522X_LDO_VOUT_LDO_MODE_SHIFT, PMIC_POWER_TPS6522X_LDO_VOUT_LDO_MODE_MASK);
    }
    if (pmic_validParamCheck(pLdoPowerResourceCfg->validParams, PMIC_POWER_TPS6522X_CFG_LDO_VOLTAGE_MV_VALID))
    {
        ldoVoltage_vset = Pmic_getBitField(
            ldoVoutRegData, PMIC_POWER_TPS6522X_LDO_VOUT_LDO_VSET_SHIFT, PMIC_POWER_TPS6522X_LDO_VOUT_LDO_VSET_MASK);

        Pmic_powerTps6522xLdoConvertVsetVal2Voltage(&(pLdoPowerResourceCfg->ldoVoltage_mv), ldoVoltage_vset, ldoNum);
    }
}

static void Pmic_powerTps6522xGetLdoPgWindowRegBitFields(Pmic_powerTps6522xLdoPowerResourceCfg_t *pLdoPowerResourceCfg,
                                                         const uint8_t                            ldoPgWindowRegData)
{
    if (pmic_validParamCheck(pLdoPowerResourceCfg->validParams, PMIC_POWER_TPS6522X_CFG_LDO_VMON_THR_VALID))
    {
        pLdoPowerResourceCfg->ldoVmonThr =
            (Pmic_powerTps6522xLdoVmonThr_t)Pmic_getBitField(ldoPgWindowRegData,
                                                             PMIC_POWER_TPS6522X_LDO_PG_WINDOW_LDO_VMON_THR_SHIFT,
                                                             PMIC_POWER_TPS6522X_LDO_PG_WINDOW_LDO_VMON_THR_MASK);
    }
}

void Pmic_powerTps6522xGetRailSel2RegBitFields(Pmic_powerTps6522xLdoPowerResourceCfg_t *pLdoPowerResourceCfg,
                                               const uint8_t                            ldoRailSelRegData,
                                               const uint8_t                            ldoNum)
{
    if (pmic_validParamCheck(pLdoPowerResourceCfg->validParams, PMIC_POWER_TPS6522X_CFG_LDO_RAIL_GRP_SEL_VALID))
    {
        switch (ldoNum)
        {
            case PMIC_POWER_TPS6522X_REGULATOR_LDO1:
                pLdoPowerResourceCfg->ldoRailGrpSel =
                    (Pmic_powerTps6522xLdoRailSel_t)Pmic_getBitField(ldoRailSelRegData,
                                                                     PMIC_POWER_TPS6522X_RAIL_SEL_2_LDO1_GRP_SEL_SHIFT,
                                                                     PMIC_POWER_TPS6522X_RAIL_SEL_2_LDO1_GRP_SEL_MASK);

                break;
            case PMIC_POWER_TPS6522X_REGULATOR_LDO2:
                pLdoPowerResourceCfg->ldoRailGrpSel =
                    (Pmic_powerTps6522xLdoRailSel_t)Pmic_getBitField(ldoRailSelRegData,
                                                                     PMIC_POWER_TPS6522X_RAIL_SEL_2_LDO2_GRP_SEL_SHIFT,
                                                                     PMIC_POWER_TPS6522X_RAIL_SEL_2_LDO2_GRP_SEL_MASK);

                break;
            case PMIC_POWER_TPS6522X_REGULATOR_LDO3:
                pLdoPowerResourceCfg->ldoRailGrpSel =
                    (Pmic_powerTps6522xLdoRailSel_t)Pmic_getBitField(ldoRailSelRegData,
                                                                     PMIC_POWER_TPS6522X_RAIL_SEL_2_LDO3_GRP_SEL_SHIFT,
                                                                     PMIC_POWER_TPS6522X_RAIL_SEL_2_LDO3_GRP_SEL_MASK);

                break;
        }
    }
}

int32_t Pmic_powerTps6522xGetLdoPwrResourceCfg(Pmic_CoreHandle_t                       *pPmicCoreHandle,
                                               Pmic_powerTps6522xLdoPowerResourceCfg_t *pLdoPowerResourceCfg,
                                               const uint8_t                            ldoNum)
{
    // clang-format off
    uint8_t ldoCtrlRegData      = 0;
    uint8_t ldoVoutRegData      = 0;
    uint8_t ldoPgWindowRegData  = 0;
    uint8_t ldoRailSelRegData   = 0;
    int32_t status              = PMIC_ST_SUCCESS;
    const Pmic_powerTps6522xLdoRegisters_t *pLdoRegisters = NULL;
    // clang-format on

    // Parameter check
    if ((pLdoPowerResourceCfg + ldoNum) == NULL)
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }
    if ((status == PMIC_ST_SUCCESS) && (pLdoPowerResourceCfg[ldoNum].validParams == 0))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    // Get all LDO resource registers
    Pmic_get_tps6522x_pwrLdoRegs(&pLdoRegisters);

    // As we are about to read, start critical section
    Pmic_criticalSectionStart(pPmicCoreHandle);

    // Read all registers pertaining to the LDO resource
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_commIntf_recvByte(pPmicCoreHandle, pLdoRegisters[ldoNum].ldoCtrlRegAddr, &ldoCtrlRegData);
    }
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_commIntf_recvByte(pPmicCoreHandle, pLdoRegisters[ldoNum].ldoVoutRegAddr, &ldoVoutRegData);
    }
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_commIntf_recvByte(pPmicCoreHandle, pLdoRegisters[ldoNum].ldoPgWindowRegAddr, &ldoPgWindowRegData);
    }
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_commIntf_recvByte(pPmicCoreHandle, pLdoRegisters[ldoNum].ldoRailSelRegAddr, &ldoRailSelRegData);
    }

    // After read, stop critical section
    Pmic_criticalSectionStop(pPmicCoreHandle);

    // For each register pertaining to the LDO resource, get the relevant bit fields of the register
    // (relevant bit fields are governed by validParams)
    if (status == PMIC_ST_SUCCESS)
    {
        // LDO_CTRL register
        Pmic_powerTps6522xGetLdoCtrlRegBitFields(pLdoPowerResourceCfg + ldoNum, ldoCtrlRegData);

        // LDO_VOUT register
        Pmic_powerTps6522xGetLdoVoutRegBitFields(pLdoPowerResourceCfg + ldoNum, ldoVoutRegData, ldoNum);

        // LDO_PG_WINDOW register
        Pmic_powerTps6522xGetLdoPgWindowRegBitFields(pLdoPowerResourceCfg + ldoNum, ldoPgWindowRegData);

        // RAIL_SEL_2 register
        Pmic_powerTps6522xGetRailSel2RegBitFields(pLdoPowerResourceCfg + ldoNum, ldoRailSelRegData, ldoNum);
    }

    return status;
}

static void Pmic_powerTps6522xGetVccaVmonCtrlBitFields(Pmic_powerTps6522xVccaVmonPowerResourceCfg_t *vccaVmonPwrRsrcCfg,
                                                       uint8_t       vccaVmonCtrlRegData,
                                                       const uint8_t vmonNum)
{
    if ((vmonNum == PMIC_POWER_TPS6522X_VOLTAGE_MONITOR_VMON1) &&
        pmic_validParamCheck(vccaVmonPwrRsrcCfg->validParams, PMIC_POWER_TPS6522X_CFG_VMON1_EN_VALID))
    {
        vccaVmonPwrRsrcCfg->vmon1En =
            (Pmic_powerTps6522xVmon1En_t)Pmic_getBitField(vccaVmonCtrlRegData,
                                                          PMIC_POWER_TPS6522X_VCCA_VMON_CTRL_VMON1_EN_SHIFT,
                                                          PMIC_POWER_TPS6522X_VCCA_VMON_CTRL_VMON1_EN_MASK);
    }
    else if ((vmonNum == PMIC_POWER_TPS6522X_VOLTAGE_MONITOR_VMON2) &&
             pmic_validParamCheck(vccaVmonPwrRsrcCfg->validParams, PMIC_POWER_TPS6522X_CFG_VMON2_EN_VALID))
    {
        vccaVmonPwrRsrcCfg->vmon2En =
            (Pmic_powerTps6522xVmon2En_t)Pmic_getBitField(vccaVmonCtrlRegData,
                                                          PMIC_POWER_TPS6522X_VCCA_VMON_CTRL_VMON2_EN_SHIFT,
                                                          PMIC_POWER_TPS6522X_VCCA_VMON_CTRL_VMON2_EN_MASK);
    }
    else if ((vmonNum == PMIC_POWER_TPS6522X_VOLTAGE_MONITOR_VCCA_VMON) &&
             pmic_validParamCheck(vccaVmonPwrRsrcCfg->validParams, PMIC_POWER_TPS6522X_CFG_VCCA_VMON_EN_VALID))
    {
        vccaVmonPwrRsrcCfg->vccaVmonEn =
            (Pmic_powerTps6522xVccaVmonEn_t)Pmic_getBitField(vccaVmonCtrlRegData,
                                                             PMIC_POWER_TPS6522X_VCCA_VMON_CTRL_VCCA_VMON_EN_SHIFT,
                                                             PMIC_POWER_TPS6522X_VCCA_VMON_CTRL_VCCA_VMON_EN_MASK);
    }

    if (pmic_validParamCheck(vccaVmonPwrRsrcCfg->validParams, PMIC_POWER_TPS6522X_CFG_VMON_DEGLITCH_SEL_VALID))
    {
        vccaVmonPwrRsrcCfg->vmonDeglitchSel =
            (Pmic_powerTps6522xVmonDeglitchSel_t)Pmic_getBitField(vccaVmonCtrlRegData,
                                                                  PMIC_POWER_TPS6522X_VCCA_VMON_CTRL_DEGLITCH_SEL_SHIFT,
                                                                  PMIC_POWER_TPS6522X_VCCA_VMON_CTRL_DEGLITCH_SEL_MASK);
    }
}

static Pmic_powerTps6522xGetVccaPgWindowBitFields(Pmic_powerTps6522xVccaVmonPowerResourceCfg_t *vccaVmonPwrRsrcCfg,
                                                  uint8_t                                       vccaPgWindowRegData)
{
    if (pmic_validParamCheck(vccaVmonPwrRsrcCfg->validParams, PMIC_POWER_TPS6522X_CFG_VCCA_PG_LEVEL_VALID))
    {
        vccaVmonPwrRsrcCfg->vccaPgLevel =
            (Pmic_powerTps6522xVccaPgLevel_t)Pmic_getBitField(vccaPgWindowRegData,
                                                              PMIC_POWER_TPS6522X_VCCA_PG_WINDOW_VCCA_PG_SET_SHIFT,
                                                              PMIC_POWER_TPS6522X_VCCA_PG_WINDOW_VCCA_PG_SET_MASK);
    }
    if (pmic_validParamCheck(vccaVmonPwrRsrcCfg->validParams, PMIC_POWER_TPS6522X_CFG_VCCA_VMON_THR_VALID))
    {
        vccaVmonPwrRsrcCfg->vccaVmonThr =
            (Pmic_powerTps6522xVccaVmonThr_t)Pmic_getBitField(vccaPgWindowRegData,
                                                              PMIC_POWER_TPS6522X_VCCA_PG_WINDOW_VCCA_VMON_THR_SHIFT,
                                                              PMIC_POWER_TPS6522X_VCCA_PG_WINDOW_VCCA_VMON_THR_MASK);
    }
}

static Pmic_powerTps6522xGetVmonPgWindowBitFields(Pmic_powerTps6522xVccaVmonPowerResourceCfg_t *vccaVmonPwrRsrcCfg,
                                                  uint8_t                                       vmonPgWindowRegData,
                                                  const uint8_t                                 vmonNum)
{
    if ((vmonNum == PMIC_POWER_TPS6522X_VOLTAGE_MONITOR_VMON1) &&
        pmic_validParamCheck(vccaVmonPwrRsrcCfg->validParams, PMIC_POWER_TPS6522X_CFG_VMON1_THR_VALID))
    {
        vccaVmonPwrRsrcCfg->vmon1Thr =
            (Pmic_powerTps6522xVmon1Thr_t)Pmic_getBitField(vmonPgWindowRegData,
                                                           PMIC_POWER_TPS6522X_VMON1_PG_WINDOW_VMON1_THR_SHIFT,
                                                           PMIC_POWER_TPS6522X_VMON1_PG_WINDOW_VMON1_THR_MASK);
    }
    else if ((vmonNum == PMIC_POWER_TPS6522X_VOLTAGE_MONITOR_VMON2) &&
             pmic_validParamCheck(vccaVmonPwrRsrcCfg->validParams, PMIC_POWER_TPS6522X_CFG_VMON2_THR_VALID))
    {
        vccaVmonPwrRsrcCfg->vmon2Thr =
            (Pmic_powerTps6522xVmon2Thr_t)Pmic_getBitField(vmonPgWindowRegData,
                                                           PMIC_POWER_TPS6522X_VMON2_PG_WINDOW_VMON2_THR_SHIFT,
                                                           PMIC_POWER_TPS6522X_VMON2_PG_WINDOW_VMON2_THR_MASK);
    }
}

static void Pmic_powerTps6522xVmonConvertPgSet2Voltage(uint16_t     *vmonPgLevel_mv,
                                                       const uint8_t vmonPgLevel_pgSet,
                                                       const uint8_t vmonNum)
{
    uint16_t baseVoltage_mv = 0, baseVoltage_pgSet = 0, voltageStep = 0;

    switch (vmonNum)
    {
        case PMIC_POWER_TPS6522X_VOLTAGE_MONITOR_VMON1:
            if ((vmonPgLevel_pgSet >= PMIC_POWER_TPS6522X_VMON1_VOLTAGE_RANGE_1_MIN_VSET) &&
                (vmonPgLevel_pgSet <= PMIC_POWER_TPS6522X_VMON1_VOLTAGE_RANGE_1_MAX_VSET))
            {
                baseVoltage_mv = PMIC_POWER_TPS6522X_VMON1_VOLTAGE_RANGE_1_MIN_VOLTAGE;
                baseVoltage_pgSet = PMIC_POWER_TPS6522X_VMON1_VOLTAGE_RANGE_1_MIN_VSET;
                voltageStep = PMIC_POWER_TPS6522X_VOLTAGE_STEP_20_MV;
            }
            else if ((vmonPgLevel_pgSet >= PMIC_POWER_TPS6522X_VMON1_VOLTAGE_RANGE_2_MIN_VSET) &&
                     (vmonPgLevel_pgSet <= PMIC_POWER_TPS6522X_VMON1_VOLTAGE_RANGE_2_MAX_VSET))
            {
                baseVoltage_mv = PMIC_POWER_TPS6522X_VMON1_VOLTAGE_RANGE_2_MIN_VOLTAGE;
                baseVoltage_pgSet = PMIC_POWER_TPS6522X_VMON1_VOLTAGE_RANGE_2_MIN_VSET;
                voltageStep = PMIC_POWER_TPS6522X_VOLTAGE_STEP_5_MV;
            }
            else if ((vmonPgLevel_pgSet >= PMIC_POWER_TPS6522X_VMON1_VOLTAGE_RANGE_3_MIN_VSET) &&
                     (vmonPgLevel_pgSet <= PMIC_POWER_TPS6522X_VMON1_VOLTAGE_RANGE_3_MAX_VSET))
            {
                baseVoltage_mv = PMIC_POWER_TPS6522X_VMON1_VOLTAGE_RANGE_3_MIN_VOLTAGE;
                baseVoltage_pgSet = PMIC_POWER_TPS6522X_VMON1_VOLTAGE_RANGE_3_MIN_VSET;
                voltageStep = PMIC_POWER_TPS6522X_VOLTAGE_STEP_10_MV;
            }
            else if ((vmonPgLevel_pgSet >= PMIC_POWER_TPS6522X_VMON1_VOLTAGE_RANGE_4_MIN_VSET) &&
                     (vmonPgLevel_pgSet <= PMIC_POWER_TPS6522X_VMON1_VOLTAGE_RANGE_4_MAX_VSET))
            {
                baseVoltage_mv = PMIC_POWER_TPS6522X_VMON1_VOLTAGE_RANGE_4_MIN_VOLTAGE;
                baseVoltage_pgSet = PMIC_POWER_TPS6522X_VMON1_VOLTAGE_RANGE_4_MIN_VSET;
                voltageStep = PMIC_POWER_TPS6522X_VOLTAGE_STEP_20_MV;
            }

            break;
        case PMIC_POWER_TPS6522X_VOLTAGE_MONITOR_VMON2:
            if (vmonPgLevel_pgSet <= PMIC_POWER_TPS6522X_VMON2_VOLTAGE_RANGE_1_MAX_VSET)
            {
                baseVoltage_mv = PMIC_POWER_TPS6522X_VMON2_VOLTAGE_RANGE_1_MIN_VOLTAGE;
                baseVoltage_pgSet = PMIC_POWER_TPS6522X_VMON2_VOLTAGE_RANGE_1_MIN_VSET;
                voltageStep = PMIC_POWER_TPS6522X_VOLTAGE_STEP_25_MV;
            }
            else if ((vmonPgLevel_pgSet >= PMIC_POWER_TPS6522X_VMON2_VOLTAGE_RANGE_2_MIN_VSET) &&
                     (vmonPgLevel_pgSet <= PMIC_POWER_TPS6522X_VMON2_VOLTAGE_RANGE_2_MAX_VSET))
            {
                baseVoltage_mv = PMIC_POWER_TPS6522X_VMON2_VOLTAGE_RANGE_2_MIN_VOLTAGE;
                baseVoltage_pgSet = PMIC_POWER_TPS6522X_VMON2_VOLTAGE_RANGE_2_MIN_VSET;
                voltageStep = PMIC_POWER_TPS6522X_VOLTAGE_STEP_50_MV;
            }

            break;
    }

    if ((baseVoltage_mv != 0) && (voltageStep != 0))
    {
        *vmonPgLevel_mv = baseVoltage_mv + (voltageStep * (vmonPgLevel_pgSet - baseVoltage_pgSet));
    }
}

static Pmic_powerTps6522xGetVmonPgLevelBitFields(Pmic_powerTps6522xVccaVmonPowerResourceCfg_t *vccaVmonPwrRsrcCfg,
                                                 uint8_t                                       vmonPgLevelRegData,
                                                 const uint8_t                                 vmonNum)
{
    if ((vmonNum == PMIC_POWER_TPS6522X_VOLTAGE_MONITOR_VMON1) &&
        pmic_validParamCheck(vccaVmonPwrRsrcCfg->validParams, PMIC_POWER_TPS6522X_CFG_VMON1_PG_LEVEL_MV_VALID))
    {
        Pmic_powerTps6522xVmonConvertPgSet2Voltage(&(vccaVmonPwrRsrcCfg->vmon1PgLevel_mv), vmonPgLevelRegData, vmonNum);
    }
    else if ((vmonNum == PMIC_POWER_TPS6522X_VOLTAGE_MONITOR_VMON2) &&
             pmic_validParamCheck(vccaVmonPwrRsrcCfg->validParams, PMIC_POWER_TPS6522X_CFG_VMON2_PG_LEVEL_MV_VALID))
    {
        Pmic_powerTps6522xVmonConvertPgSet2Voltage(&(vccaVmonPwrRsrcCfg->vmon2PgLevel_mv), vmonPgLevelRegData, vmonNum);
    }
}

static Pmic_powerTps6522xGetRailSel3RegBitFields(Pmic_powerTps6522xVccaVmonPowerResourceCfg_t *vccaVmonPwrRsrcCfg,
                                                 uint8_t                                       vccaVmonRailSelRegData,
                                                 const uint8_t                                 vmonNum)
{
    if ((vmonNum == PMIC_POWER_TPS6522X_VOLTAGE_MONITOR_VMON1) &&
        pmic_validParamCheck(vccaVmonPwrRsrcCfg->validParams, PMIC_POWER_TPS6522X_CFG_VMON1_RAIL_GRP_SEL_VALID))
    {
        vccaVmonPwrRsrcCfg->vmon1RailGrpSel =
            (Pmic_powerTps6522xVmon1RailSel_t)Pmic_getBitField(vccaVmonRailSelRegData,
                                                               PMIC_POWER_TPS6522X_RAIL_SEL_3_VMON1_GRP_SEL_SHIFT,
                                                               PMIC_POWER_TPS6522X_RAIL_SEL_3_VMON1_GRP_SEL_MASK);
    }
    else if ((vmonNum == PMIC_POWER_TPS6522X_VOLTAGE_MONITOR_VMON2) &&
             pmic_validParamCheck(vccaVmonPwrRsrcCfg->validParams, PMIC_POWER_TPS6522X_CFG_VMON2_RAIL_GRP_SEL_VALID))
    {
        vccaVmonPwrRsrcCfg->vmon2RailGrpSel =
            (Pmic_powerTps6522xVmon2RailSel_t)Pmic_getBitField(vccaVmonRailSelRegData,
                                                               PMIC_POWER_TPS6522X_RAIL_SEL_3_VMON2_GRP_SEL_SHIFT,
                                                               PMIC_POWER_TPS6522X_RAIL_SEL_3_VMON2_GRP_SEL_MASK);
    }
    else if ((vmonNum == PMIC_POWER_TPS6522X_VOLTAGE_MONITOR_VCCA_VMON) &&
             pmic_validParamCheck(vccaVmonPwrRsrcCfg->validParams, PMIC_POWER_TPS6522X_CFG_VCCA_RAIL_GRP_SEL_VALID))
    {
        vccaVmonPwrRsrcCfg->vccaRailGrpSel =
            (Pmic_powerTps6522xVccaRailSel_t)Pmic_getBitField(vccaVmonRailSelRegData,
                                                              PMIC_POWER_TPS6522X_RAIL_SEL_3_VCCA_GRP_SEL_SHIFT,
                                                              PMIC_POWER_TPS6522X_RAIL_SEL_3_VCCA_GRP_SEL_MASK);
    }
}

static int32_t
Pmic_powerTps6522xGetVccaVmonPwrResourceCfg(Pmic_CoreHandle_t                            *pPmicCoreHandle,
                                            Pmic_powerTps6522xVccaVmonPowerResourceCfg_t *vccaVmonPwrRsrcCfg,
                                            const uint8_t                                 vmonNum)
{
    // clang-format off
    uint8_t vccaVmonCtrlRegData     = 0;
    uint8_t vccaPgWindowRegData     = 0;
    uint8_t vmonPgWindowRegData     = 0;
    uint8_t vmonPgLevelRegData      = 0;
    uint8_t vccaVmonRailSelRegData  = 0;
    int32_t status                  = PMIC_ST_SUCCESS;
    const Pmic_powerTps6522xVccaVmonRegisters_t *pVccaVmonRegisters = NULL;
    // clang-format on

    // Parameter check
    if (vccaVmonPwrRsrcCfg == NULL)
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }
    if ((status == PMIC_ST_SUCCESS) && (vccaVmonPwrRsrcCfg->validParams == 0))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    // Get VCCA_VMON and VMONx registers
    Pmic_get_tps6522x_PwrVccaVmonRegisters(&pVccaVmonRegisters);

    // As we are about to read, start critical section
    Pmic_criticalSectionStart(pPmicCoreHandle);

    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_commIntf_recvByte(
            pPmicCoreHandle, pVccaVmonRegisters[vmonNum].vccaVmonCtrlRegAddr, &vccaVmonCtrlRegData);
    }
    if ((status == PMIC_ST_SUCCESS) && (vmonNum == PMIC_POWER_TPS6522X_VOLTAGE_MONITOR_VCCA_VMON))
    {
        status = Pmic_commIntf_recvByte(
            pPmicCoreHandle, pVccaVmonRegisters[vmonNum].vccaPgWindowRegAddr, &vccaPgWindowRegData);
    }
    if ((status == PMIC_ST_SUCCESS) && ((vmonNum == PMIC_POWER_TPS6522X_VOLTAGE_MONITOR_VMON1) ||
                                        (vmonNum == PMIC_POWER_TPS6522X_VOLTAGE_MONITOR_VMON2)))
    {
        status = Pmic_commIntf_recvByte(
            pPmicCoreHandle, pVccaVmonRegisters[vmonNum].vmonPgWindowRegAddr, &vmonPgWindowRegData);
    }
    if ((status == PMIC_ST_SUCCESS) && ((vmonNum == PMIC_POWER_TPS6522X_VOLTAGE_MONITOR_VMON1) ||
                                        (vmonNum == PMIC_POWER_TPS6522X_VOLTAGE_MONITOR_VMON2)))
    {
        status = Pmic_commIntf_recvByte(
            pPmicCoreHandle, pVccaVmonRegisters[vmonNum].vmonPgLevelRegAddr, &vmonPgLevelRegData);
    }
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_commIntf_recvByte(
            pPmicCoreHandle, pVccaVmonRegisters[vmonNum].vccaVmonRailSelRegAddr, &vccaVmonRailSelRegData);
    }

    // After read, stop critical section
    Pmic_criticalSectionStop(pPmicCoreHandle);

    // For each register pertaining to the VCCA/VMON resource, get the relevant bit fields of the register
    // (relevant bit fields are governed by validParams)
    if (status == PMIC_ST_SUCCESS)
    {
        // VCCA_VMON_CTRL
        Pmic_powerTps6522xGetVccaVmonCtrlBitFields(vccaVmonPwrRsrcCfg, vccaVmonCtrlRegData, vmonNum);

        // VCCA_PG_WINDOW
        Pmic_powerTps6522xGetVccaPgWindowBitFields(vccaVmonPwrRsrcCfg, vccaPgWindowRegData);

        // VMON_PG_WINDOW
        Pmic_powerTps6522xGetVmonPgWindowBitFields(vccaVmonPwrRsrcCfg, vmonPgWindowRegData, vmonNum);

        // VMON_PG_LEVEL
        Pmic_powerTps6522xGetVmonPgLevelBitFields(vccaVmonPwrRsrcCfg, vmonPgLevelRegData, vmonNum);

        // RAIL_SEL_3
        Pmic_powerTps6522xGetRailSel3RegBitFields(vccaVmonPwrRsrcCfg, vccaVmonRailSelRegData, vmonNum);
    }

    // After read, stop critical section
    Pmic_criticalSectionStop(pPmicCoreHandle);

    return status;
}

int32_t Pmic_powerTps6522xGetPwrResourceCfg(Pmic_CoreHandle_t                    *pPmicCoreHandle,
                                            Pmic_powerTps6522xPowerResourceCfg_t *pPwrResourceCfg)
{
    uint8_t  iter = 0;
    uint16_t validParam = 0;
    int32_t  status = PMIC_ST_SUCCESS;

    // Parameter check
    status = Pmic_powerTps6522xParamCheck_pPwrRsrcCfg(pPmicCoreHandle, pPwrResourceCfg);

    // If parameters are valid...
    if (status == PMIC_ST_SUCCESS)
    {
        // For each BUCK...
        for (iter = 0; iter < PMIC_POWER_TPS6522X_MAX_BUCK_NUM; iter++)
        {
            // Calculate BUCK validParam
            validParam = PMIC_POWER_TPS6522X_CFG_BUCK1_VALID + iter;

            // If BUCK validParam is set...
            if (pmic_validParamCheck(pPwrResourceCfg->validParams, validParam))
            {
                // Get BUCK's power resource configuration
                status =
                    Pmic_powerTps6522xGetBuckPwrResourceCfg(pPmicCoreHandle, pPwrResourceCfg->buckPwrRsrcCfg, iter);
            }

            // Upon error, break out of loop
            if (status != PMIC_ST_SUCCESS)
            {
                break;
            }
        }
    }

    // If read of BUCK power resource configuration was successful...
    if (status == PMIC_ST_SUCCESS)
    {
        // For each LDO...
        for (iter = 0; iter < PMIC_POWER_TPS6522X_MAX_LDO_NUM; iter++)
        {
            // Calculate LDO validParam
            validParam = PMIC_POWER_TPS6522X_CFG_LDO1_VALID + iter;

            // If LDO validParam is set...
            if (pmic_validParamCheck(pPwrResourceCfg->validParams, validParam))
            {
                // Get LDO's power resource configuration
                status = Pmic_powerTps6522xGetLdoPwrResourceCfg(pPmicCoreHandle, pPwrResourceCfg->ldoPwrRsrcCfg, iter);
            }

            // Upon error, break out of loop
            if (status != PMIC_ST_SUCCESS)
            {
                break;
            }
        }
    }

    // If read of LDO power resource configuration was successful...
    if (status == PMIC_ST_SUCCESS)
    {
        // For each VMON...
        for (iter = 0; iter < PMIC_POWER_TPS6522X_MAX_VOLTAGE_MONITOR_NUM; iter++)
        {
            // Calculate VCCA/VMON validParam
            validParam = PMIC_POWER_TPS6522X_CFG_VMON1_VALID + iter;

            // If VMON validParam is set...
            if (pmic_validParamCheck(pPwrResourceCfg->validParams, validParam))
            {
                // Get VMON's power resource configuration
                status = Pmic_powerTps6522xGetVccaVmonPwrResourceCfg(
                    pPmicCoreHandle, &(pPwrResourceCfg->vccaVmonPwrRsrcCfg), iter);
            }

            // Upon error, break out of loop
            if (status != PMIC_ST_SUCCESS)
            {
                break;
            }
        }
    }

    return status;
}

static void
Pmic_powerTps6522xSetBuckCtrlRegBitFields(const Pmic_powerTps6522xBuckPowerResourceCfg_t buckPowerResourceCfg,
                                          uint8_t                                       *buckCtrlRegData)
{
    if (pmic_validParamCheck(buckPowerResourceCfg.validParams, PMIC_POWER_TPS6522X_CFG_BUCK_PLDN_VALID))
    {
        Pmic_setBitField(buckCtrlRegData,
                         PMIC_POWER_TPS6522X_BUCK_CTRL_PLDN_SHIFT,
                         PMIC_POWER_TPS6522X_BUCK_CTRL_PLDN_MASK,
                         (uint8_t)buckPowerResourceCfg.buckPldn);
    }
    if (pmic_validParamCheck(buckPowerResourceCfg.validParams, PMIC_POWER_TPS6522X_CFG_BUCK_VMON_EN_VALID))
    {
        Pmic_setBitField(buckCtrlRegData,
                         PMIC_POWER_TPS6522X_BUCK_CTRL_VMON_EN_SHIFT,
                         PMIC_POWER_TPS6522X_BUCK_CTRL_VMON_EN_MASK,
                         (uint8_t)buckPowerResourceCfg.buckVmonEn);
    }
    if (pmic_validParamCheck(buckPowerResourceCfg.validParams, PMIC_POWER_TPS6522X_CFG_BUCK_PWM_OPTION_VALID))
    {
        Pmic_setBitField(buckCtrlRegData,
                         PMIC_POWER_TPS6522X_BUCK_CTRL_PWM_OPTION_SHIFT,
                         PMIC_POWER_TPS6522X_BUCK_CTRL_PWM_OPTION_MASK,
                         (uint8_t)buckPowerResourceCfg.buckPwmOption);
    }
    if (pmic_validParamCheck(buckPowerResourceCfg.validParams, PMIC_POWER_TPS6522X_CFG_BUCK_EN_VALID))
    {
        Pmic_setBitField(buckCtrlRegData,
                         PMIC_POWER_TPS6522X_BUCK_CTRL_EN_SHIFT,
                         PMIC_POWER_TPS6522X_BUCK_CTRL_EN_MASK,
                         (uint8_t)buckPowerResourceCfg.buckEn);
    }
}

static void
Pmic_powerTps6522xSetBuckConfRegBitFields(const Pmic_powerTps6522xBuckPowerResourceCfg_t buckPowerResourceCfg,
                                          uint8_t                                       *buckConfRegData)
{
    if (pmic_validParamCheck(buckPowerResourceCfg.validParams, PMIC_POWER_TPS6522X_CFG_BUCK_SLEW_RATE_VALID))
    {
        Pmic_setBitField(buckConfRegData,
                         PMIC_POWER_TPS6522X_BUCK_CONF_SLEW_RATE_SHIFT,
                         PMIC_POWER_TPS6522X_BUCK_CONF_SLEW_RATE_MASK,
                         (uint8_t)buckPowerResourceCfg.buckSlewRate);
    }
}

static void Pmic_powerTps6522xBuckConvertVoltage2VsetVal(uint8_t       *buckVoltage_vset,
                                                         const uint16_t buckVoltage_mv,
                                                         const uint8_t  buckNum)
{
    uint16_t baseVoltage_vset = 0, baseVoltage_mv = 0, voltageStep = 0;

    switch (buckNum)
    {
        case PMIC_POWER_TPS6522X_REGULATOR_BUCK1:
            if ((buckVoltage_mv >= PMIC_POWER_TPS6522X_BUCK1_VOLTAGE_RANGE_1_MIN_VOLTAGE) &&
                (buckVoltage_mv <= PMIC_POWER_TPS6522X_BUCK1_VOLTAGE_RANGE_1_MAX_VOLTAGE))
            {
                baseVoltage_vset = PMIC_POWER_TPS6522X_BUCK1_VOLTAGE_RANGE_1_MIN_VSET;
                baseVoltage_mv = PMIC_POWER_TPS6522X_BUCK1_VOLTAGE_RANGE_1_MIN_VOLTAGE;
                voltageStep = PMIC_POWER_TPS6522X_VOLTAGE_STEP_20_MV;
            }
            else if ((buckVoltage_mv >= PMIC_POWER_TPS6522X_BUCK1_VOLTAGE_RANGE_2_MIN_VOLTAGE) &&
                     (buckVoltage_mv <= PMIC_POWER_TPS6522X_BUCK1_VOLTAGE_RANGE_2_MAX_VOLTAGE))
            {
                baseVoltage_vset = PMIC_POWER_TPS6522X_BUCK1_VOLTAGE_RANGE_2_MIN_VSET;
                baseVoltage_mv = PMIC_POWER_TPS6522X_BUCK1_VOLTAGE_RANGE_2_MIN_VOLTAGE;
                voltageStep = PMIC_POWER_TPS6522X_VOLTAGE_STEP_5_MV;
            }
            else if ((buckVoltage_mv >= PMIC_POWER_TPS6522X_BUCK1_VOLTAGE_RANGE_3_MIN_VOLTAGE) &&
                     (buckVoltage_mv <= PMIC_POWER_TPS6522X_BUCK1_VOLTAGE_RANGE_3_MAX_VOLTAGE))
            {
                baseVoltage_vset = PMIC_POWER_TPS6522X_BUCK1_VOLTAGE_RANGE_3_MIN_VSET;
                baseVoltage_mv = PMIC_POWER_TPS6522X_BUCK1_VOLTAGE_RANGE_3_MIN_VOLTAGE;
                voltageStep = PMIC_POWER_TPS6522X_VOLTAGE_STEP_10_MV;
            }
            else if ((buckVoltage_mv >= PMIC_POWER_TPS6522X_BUCK1_VOLTAGE_RANGE_4_MIN_VOLTAGE) &&
                     (buckVoltage_mv <= PMIC_POWER_TPS6522X_BUCK1_VOLTAGE_RANGE_4_MAX_VOLTAGE))
            {
                baseVoltage_vset = PMIC_POWER_TPS6522X_BUCK1_VOLTAGE_RANGE_4_MIN_VSET;
                baseVoltage_mv = PMIC_POWER_TPS6522X_BUCK1_VOLTAGE_RANGE_4_MIN_VOLTAGE;
                voltageStep = PMIC_POWER_TPS6522X_VOLTAGE_STEP_20_MV;
            }

            break;
        case PMIC_POWER_TPS6522X_REGULATOR_BUCK2:
        case PMIC_POWER_TPS6522X_REGULATOR_BUCK3:
        case PMIC_POWER_TPS6522X_REGULATOR_BUCK4:
            if ((buckVoltage_mv >= PMIC_POWER_TPS6522X_BUCK2_3_4_VOLTAGE_RANGE_1_MIN_VOLTAGE) &&
                (buckVoltage_mv <= PMIC_POWER_TPS6522X_BUCK2_3_4_VOLTAGE_RANGE_1_MAX_VOLTAGE))
            {
                baseVoltage_vset = PMIC_POWER_TPS6522X_BUCK2_3_4_VOLTAGE_RANGE_1_MIN_VSET;
                baseVoltage_mv = PMIC_POWER_TPS6522X_BUCK2_3_4_VOLTAGE_RANGE_1_MIN_VOLTAGE;
                voltageStep = PMIC_POWER_TPS6522X_VOLTAGE_STEP_25_MV;
            }
            else if ((buckVoltage_mv >= PMIC_POWER_TPS6522X_BUCK2_3_4_VOLTAGE_RANGE_2_MIN_VOLTAGE) &&
                     (buckVoltage_mv <= PMIC_POWER_TPS6522X_BUCK2_3_4_VOLTAGE_RANGE_2_MAX_VOLTAGE))
            {
                baseVoltage_vset = PMIC_POWER_TPS6522X_BUCK2_3_4_VOLTAGE_RANGE_2_MIN_VSET;
                baseVoltage_mv = PMIC_POWER_TPS6522X_BUCK2_3_4_VOLTAGE_RANGE_2_MIN_VOLTAGE;
                voltageStep = PMIC_POWER_TPS6522X_VOLTAGE_STEP_50_MV;
            }

            break;
    }

    if ((baseVoltage_mv != 0) && (voltageStep != 0))
    {
        *buckVoltage_vset = baseVoltage_vset + ((buckVoltage_mv - baseVoltage_mv) / voltageStep);
    }
}

static void
Pmic_powerTps6522xSetBuckVoutRegBitFields(const Pmic_powerTps6522xBuckPowerResourceCfg_t *buckPowerResourceCfg,
                                          uint8_t                                        *buckVoutRegData,
                                          const uint8_t                                   buckNum)
{
    if (pmic_validParamCheck(buckPowerResourceCfg[buckNum].validParams, PMIC_POWER_TPS6522X_CFG_BUCK_VOLTAGE_MV_VALID))
    {
        Pmic_powerTps6522xBuckConvertVoltage2VsetVal(
            buckVoutRegData, buckPowerResourceCfg[buckNum].buckVoltage_mv, buckNum);
    }
}

static void
Pmic_powerTps6522xSetBuckPgWindowRegBitFields(const Pmic_powerTps6522xBuckPowerResourceCfg_t buckPowerResourceCfg,
                                              uint8_t                                       *buckPgWindowRegData)
{
    if (pmic_validParamCheck(buckPowerResourceCfg.validParams, PMIC_POWER_TPS6522X_CFG_BUCK_VMON_THR_VALID))
    {
        Pmic_setBitField(buckPgWindowRegData,
                         PMIC_POWER_TPS6522X_BUCK_PG_WINDOW_VMON_THR_SHIFT,
                         PMIC_POWER_TPS6522X_BUCK_PG_WINDOW_VMON_THR_MASK,
                         (uint8_t)buckPowerResourceCfg.buckVmonThr);
    }
}

static void
Pmic_powerTps6522xSetRailSel1RegBitFields(const Pmic_powerTps6522xBuckPowerResourceCfg_t *buckPowerResourceCfg,
                                          uint8_t                                        *buckRailSelRegData,
                                          const uint8_t                                   buckNum)
{
    if (pmic_validParamCheck(buckPowerResourceCfg[buckNum].validParams,
                             PMIC_POWER_TPS6522X_CFG_BUCK_RAIL_GRP_SEL_VALID))
    {
        switch (buckNum)
        {
            case PMIC_POWER_TPS6522X_REGULATOR_BUCK1:
                Pmic_setBitField(buckRailSelRegData,
                                 PMIC_POWER_TPS6522X_CFG_RAIL_SEL_1_BUCK1_GRP_SEL_SHIFT,
                                 PMIC_POWER_TPS6522X_CFG_RAIL_SEL_1_BUCK1_GRP_SEL_MASK,
                                 (uint8_t)buckPowerResourceCfg[buckNum].buckRailGrpSel);

                break;
            case PMIC_POWER_TPS6522X_REGULATOR_BUCK2:
                Pmic_setBitField(buckRailSelRegData,
                                 PMIC_POWER_TPS6522X_CFG_RAIL_SEL_1_BUCK2_GRP_SEL_SHIFT,
                                 PMIC_POWER_TPS6522X_CFG_RAIL_SEL_1_BUCK2_GRP_SEL_MASK,
                                 (uint8_t)buckPowerResourceCfg[buckNum].buckRailGrpSel);

                break;
            case PMIC_POWER_TPS6522X_REGULATOR_BUCK3:
                Pmic_setBitField(buckRailSelRegData,
                                 PMIC_POWER_TPS6522X_CFG_RAIL_SEL_1_BUCK3_GRP_SEL_SHIFT,
                                 PMIC_POWER_TPS6522X_CFG_RAIL_SEL_1_BUCK3_GRP_SEL_MASK,
                                 (uint8_t)buckPowerResourceCfg[buckNum].buckRailGrpSel);

                break;
            case PMIC_POWER_TPS6522X_REGULATOR_BUCK4:
                Pmic_setBitField(buckRailSelRegData,
                                 PMIC_POWER_TPS6522X_CFG_RAIL_SEL_1_BUCK4_GRP_SEL_SHIFT,
                                 PMIC_POWER_TPS6522X_CFG_RAIL_SEL_1_BUCK4_GRP_SEL_MASK,
                                 (uint8_t)buckPowerResourceCfg[buckNum].buckRailGrpSel);

                break;
        }
    }
}

int32_t Pmic_powerTps6522xSetBuckPwrResourceCfg(Pmic_CoreHandle_t                              *pPmicCoreHandle,
                                                const Pmic_powerTps6522xBuckPowerResourceCfg_t *buckPowerResourceCfg,
                                                const uint8_t                                   buckNum)
{
    // clang-format off
    uint8_t buckCtrlRegData     = 0;
    uint8_t buckConfRegData     = 0;
    uint8_t buckVoutRegData     = 0;
    uint8_t buckPgWindowRegData = 0;
    uint8_t buckRailSelRegData  = 0;
    int32_t status              = PMIC_ST_SUCCESS;
    const Pmic_powerTps6522xBuckRegisters_t *pBuckRegisters = NULL;
    // clang-format on

    // Parameter check
    if (buckPowerResourceCfg[buckNum].validParams == 0)
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    // Get all BUCK resource registers
    Pmic_get_tps6522x_pwrBuckRegs(&pBuckRegisters);

    // As we are about to read, start critical section
    Pmic_criticalSectionStart(pPmicCoreHandle);

    // Read all registers pertaining to the BUCK resource
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_commIntf_recvByte(pPmicCoreHandle, pBuckRegisters[buckNum].buckCtrlRegAddr, &buckCtrlRegData);
    }
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_commIntf_recvByte(pPmicCoreHandle, pBuckRegisters[buckNum].buckConfRegAddr, &buckConfRegData);
    }
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_commIntf_recvByte(pPmicCoreHandle, pBuckRegisters[buckNum].buckVoutRegAddr, &buckVoutRegData);
    }
    if (status == PMIC_ST_SUCCESS)
    {
        status =
            Pmic_commIntf_recvByte(pPmicCoreHandle, pBuckRegisters[buckNum].buckPgWindowRegAddr, &buckPgWindowRegData);
    }
    if (status == PMIC_ST_SUCCESS)
    {
        status =
            Pmic_commIntf_recvByte(pPmicCoreHandle, pBuckRegisters[buckNum].buckRailSelRegAddr, &buckRailSelRegData);
    }

    // For each register pertaining to the BUCK resource, modify the relevant bit fields of the register
    // (relevant bit fields are governed by validParams)
    if (status == PMIC_ST_SUCCESS)
    {
        // BUCK_CTRL register
        Pmic_powerTps6522xSetBuckCtrlRegBitFields(buckPowerResourceCfg[buckNum], &buckCtrlRegData);

        // BUCK_CONF register
        Pmic_powerTps6522xSetBuckConfRegBitFields(buckPowerResourceCfg[buckNum], &buckConfRegData);

        // BUCK_VOUT register
        Pmic_powerTps6522xSetBuckVoutRegBitFields(buckPowerResourceCfg, &buckVoutRegData, buckNum);

        // BUCK_PG_WINDOW register
        Pmic_powerTps6522xSetBuckPgWindowRegBitFields(buckPowerResourceCfg[buckNum], &buckPgWindowRegData);

        // RAIL_SEL_1 register
        Pmic_powerTps6522xSetRailSel1RegBitFields(buckPowerResourceCfg, &buckRailSelRegData, buckNum);
    }

    // After modification of register bit fields, write values back to PMIC
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_commIntf_sendByte(pPmicCoreHandle, pBuckRegisters[buckNum].buckConfRegAddr, buckConfRegData);
    }
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_commIntf_sendByte(pPmicCoreHandle, pBuckRegisters[buckNum].buckVoutRegAddr, buckVoutRegData);
    }
    if (status == PMIC_ST_SUCCESS)
    {
        status =
            Pmic_commIntf_sendByte(pPmicCoreHandle, pBuckRegisters[buckNum].buckPgWindowRegAddr, buckPgWindowRegData);
    }
    if (status == PMIC_ST_SUCCESS)
    {
        status =
            Pmic_commIntf_sendByte(pPmicCoreHandle, pBuckRegisters[buckNum].buckRailSelRegAddr, buckRailSelRegData);
    }
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_commIntf_sendByte(pPmicCoreHandle, pBuckRegisters[buckNum].buckCtrlRegAddr, buckCtrlRegData);
    }

    // After read-modify-write, stop critical section
    Pmic_criticalSectionStop(pPmicCoreHandle);

    return status;
}

static void Pmic_powerTps6522xSetLdoCtrlRegBitFields(const Pmic_powerTps6522xLdoPowerResourceCfg_t ldoPowerResourceCfg,
                                                     uint8_t                                      *ldoCtrlRegData)
{
    if (pmic_validParamCheck(ldoPowerResourceCfg.validParams, PMIC_POWER_TPS6522X_CFG_LDO_DISCHARGE_EN_VALID))
    {
        Pmic_setBitField(ldoCtrlRegData,
                         PMIC_POWER_TPS6522X_LDO_CTRL_DISCHARGE_EN_SHIFT,
                         PMIC_POWER_TPS6522X_LDO_CTRL_DISCHARGE_EN_MASK,
                         (uint8_t)ldoPowerResourceCfg.ldoDischargeEn);
    }
    if (pmic_validParamCheck(ldoPowerResourceCfg.validParams, PMIC_POWER_TPS6522X_CFG_LDO_VMON_EN_VALID))
    {
        Pmic_setBitField(ldoCtrlRegData,
                         PMIC_POWER_TPS6522X_LDO_CTRL_VMON_EN_SHIFT,
                         PMIC_POWER_TPS6522X_LDO_CTRL_VMON_EN_MASK,
                         (uint8_t)ldoPowerResourceCfg.ldoVmonEn);
    }
    if (pmic_validParamCheck(ldoPowerResourceCfg.validParams, PMIC_POWER_TPS6522X_CFG_LDO_EN_VALID))
    {
        Pmic_setBitField(ldoCtrlRegData,
                         PMIC_POWER_TPS6522X_LDO_CTRL_EN_SHIFT,
                         PMIC_POWER_TPS6522X_LDO_CTRL_EN_MASK,
                         (uint8_t)ldoPowerResourceCfg.ldoEn);
    }
}

static void Pmic_powerTps6522xLdoConvertVoltage2VsetVal(uint8_t       *ldoVoltage_vset,
                                                        const uint16_t ldoVoltage_mv,
                                                        const uint8_t  ldoNum)
{
    uint16_t baseVoltage_vset = 0, baseVoltage_mv = 0, voltageStep = 0;

    switch (ldoNum)
    {
        case PMIC_POWER_TPS6522X_REGULATOR_LDO1:
            if ((ldoVoltage_mv >= PMIC_POWER_TPS6522X_LDO1_VOLTAGE_RANGE_1_MIN_VOLTAGE) &&
                (ldoVoltage_mv <= PMIC_POWER_TPS6522X_LDO1_VOLTAGE_RANGE_1_MAX_VOLTAGE))
            {
                baseVoltage_vset = PMIC_POWER_TPS6522X_LDO1_VOLTAGE_RANGE_1_MIN_VSET;
                baseVoltage_mv = PMIC_POWER_TPS6522X_LDO1_VOLTAGE_RANGE_1_MIN_VOLTAGE;
                voltageStep = PMIC_POWER_TPS6522X_VOLTAGE_STEP_0_MV;
            }
            else if ((ldoVoltage_mv >= PMIC_POWER_TPS6522X_LDO1_VOLTAGE_RANGE_2_MIN_VOLTAGE) &&
                     (ldoVoltage_mv <= PMIC_POWER_TPS6522X_LDO1_VOLTAGE_RANGE_2_MAX_VOLTAGE))
            {
                baseVoltage_vset = PMIC_POWER_TPS6522X_LDO1_VOLTAGE_RANGE_2_MIN_VSET;
                baseVoltage_mv = PMIC_POWER_TPS6522X_LDO1_VOLTAGE_RANGE_2_MIN_VOLTAGE;
                voltageStep = PMIC_POWER_TPS6522X_VOLTAGE_STEP_50_MV;
            }
            else if ((ldoVoltage_mv >= PMIC_POWER_TPS6522X_LDO1_VOLTAGE_RANGE_3_MIN_VOLTAGE) &&
                     (ldoVoltage_mv <= PMIC_POWER_TPS6522X_LDO1_VOLTAGE_RANGE_3_MAX_VOLTAGE))
            {
                baseVoltage_vset = PMIC_POWER_TPS6522X_LDO1_VOLTAGE_RANGE_3_MIN_VSET;
                baseVoltage_mv = PMIC_POWER_TPS6522X_LDO1_VOLTAGE_RANGE_3_MIN_VOLTAGE;
                voltageStep = PMIC_POWER_TPS6522X_VOLTAGE_STEP_0_MV;
            }

            break;
        case PMIC_POWER_TPS6522X_REGULATOR_LDO2:
        case PMIC_POWER_TPS6522X_REGULATOR_LDO3:
            if ((ldoVoltage_mv >= PMIC_POWER_TPS6522X_LDO2_3_VOLTAGE_RANGE_1_MIN_VOLTAGE) &&
                (ldoVoltage_mv <= PMIC_POWER_TPS6522X_LDO2_3_VOLTAGE_RANGE_1_MAX_VOLTAGE))
            {
                baseVoltage_vset = PMIC_POWER_TPS6522X_LDO2_3_VOLTAGE_RANGE_1_MIN_VSET;
                baseVoltage_mv = PMIC_POWER_TPS6522X_LDO2_3_VOLTAGE_RANGE_1_MIN_VOLTAGE;
                voltageStep = PMIC_POWER_TPS6522X_VOLTAGE_STEP_50_MV;
            }
            else if ((ldoVoltage_mv >= PMIC_POWER_TPS6522X_LDO2_3_VOLTAGE_RANGE_2_MIN_VOLTAGE) &&
                     (ldoVoltage_mv <= PMIC_POWER_TPS6522X_LDO2_3_VOLTAGE_RANGE_2_MAX_VOLTAGE))
            {
                baseVoltage_vset = PMIC_POWER_TPS6522X_LDO2_3_VOLTAGE_RANGE_2_MIN_VSET;
                baseVoltage_mv = PMIC_POWER_TPS6522X_LDO2_3_VOLTAGE_RANGE_2_MIN_VOLTAGE;
                voltageStep = PMIC_POWER_TPS6522X_VOLTAGE_STEP_0_MV;
            }

            break;
    }

    if (baseVoltage_mv != 0)
    {
        *ldoVoltage_vset = baseVoltage_vset + ((ldoVoltage_mv - baseVoltage_mv) / voltageStep);
    }
}

static void Pmic_powerTps6522xSetLdoVoutRegBitFields(const Pmic_powerTps6522xLdoPowerResourceCfg_t *ldoPowerResourceCfg,
                                                     uint8_t                                       *ldoVoutRegData,
                                                     const uint8_t                                  ldoNum)
{
    uint8_t ldoVoltage_vset = 0;

    if (pmic_validParamCheck(ldoPowerResourceCfg[ldoNum].validParams, PMIC_POWER_TPS6522X_CFG_LDO_MODE_VALID))
    {
        Pmic_setBitField(ldoVoutRegData,
                         PMIC_POWER_TPS6522X_LDO_VOUT_LDO_MODE_SHIFT,
                         PMIC_POWER_TPS6522X_LDO_VOUT_LDO_MODE_MASK,
                         (uint8_t)ldoPowerResourceCfg[ldoNum].ldoMode);
    }
    if (pmic_validParamCheck(ldoPowerResourceCfg[ldoNum].validParams, PMIC_POWER_TPS6522X_CFG_LDO_VOLTAGE_MV_VALID))
    {
        Pmic_powerTps6522xLdoConvertVoltage2VsetVal(
            &ldoVoltage_vset, ldoPowerResourceCfg[ldoNum].ldoVoltage_mv, ldoNum);

        Pmic_setBitField(ldoVoutRegData,
                         PMIC_POWER_TPS6522X_LDO_VOUT_LDO_VSET_SHIFT,
                         PMIC_POWER_TPS6522X_LDO_VOUT_LDO_VSET_MASK,
                         ldoVoltage_vset);
    }
}

static void
Pmic_powerTps6522xSetLdoPgWindowRegBitFields(const Pmic_powerTps6522xLdoPowerResourceCfg_t ldoPowerResourceCfg,
                                             uint8_t                                      *ldoPgWindowRegData)
{
    if (pmic_validParamCheck(ldoPowerResourceCfg.validParams, PMIC_POWER_TPS6522X_CFG_LDO_VMON_THR_VALID))
    {
        Pmic_setBitField(ldoPgWindowRegData,
                         PMIC_POWER_TPS6522X_LDO_PG_WINDOW_LDO_VMON_THR_SHIFT,
                         PMIC_POWER_TPS6522X_LDO_PG_WINDOW_LDO_VMON_THR_MASK,
                         (uint8_t)ldoPowerResourceCfg.ldoVmonThr);
    }
}

static void Pmic_powerTps6522xSetRailSel2RegBitFields(
    const Pmic_powerTps6522xLdoPowerResourceCfg_t *ldoPowerResourceCfg, uint8_t *ldoRailSelRegData, uint8_t ldoNum)
{
    if (pmic_validParamCheck(ldoPowerResourceCfg[ldoNum].validParams, PMIC_POWER_TPS6522X_CFG_LDO_RAIL_GRP_SEL_VALID))
    {
        switch (ldoNum)
        {
            case PMIC_POWER_TPS6522X_REGULATOR_LDO1:
                Pmic_setBitField(ldoRailSelRegData,
                                 PMIC_POWER_TPS6522X_RAIL_SEL_2_LDO1_GRP_SEL_SHIFT,
                                 PMIC_POWER_TPS6522X_RAIL_SEL_2_LDO1_GRP_SEL_MASK,
                                 (uint8_t)ldoPowerResourceCfg[ldoNum].ldoRailGrpSel);

                break;
            case PMIC_POWER_TPS6522X_REGULATOR_LDO2:
                Pmic_setBitField(ldoRailSelRegData,
                                 PMIC_POWER_TPS6522X_RAIL_SEL_2_LDO2_GRP_SEL_SHIFT,
                                 PMIC_POWER_TPS6522X_RAIL_SEL_2_LDO2_GRP_SEL_MASK,
                                 (uint8_t)ldoPowerResourceCfg[ldoNum].ldoRailGrpSel);

                break;
            case PMIC_POWER_TPS6522X_REGULATOR_LDO3:
                Pmic_setBitField(ldoRailSelRegData,
                                 PMIC_POWER_TPS6522X_RAIL_SEL_2_LDO3_GRP_SEL_SHIFT,
                                 PMIC_POWER_TPS6522X_RAIL_SEL_2_LDO3_GRP_SEL_MASK,
                                 (uint8_t)ldoPowerResourceCfg[ldoNum].ldoRailGrpSel);

                break;
        }
    }
}

int32_t Pmic_powerTps6522xSetLdoPwrResourceCfg(Pmic_CoreHandle_t                             *pPmicCoreHandle,
                                               const Pmic_powerTps6522xLdoPowerResourceCfg_t *ldoPowerResourceCfg,
                                               const uint8_t                                  ldoNum)
{
    // clang-format off
    uint8_t ldoCtrlRegData      = 0;
    uint8_t ldoVoutRegData      = 0;
    uint8_t ldoPgWindowRegData  = 0;
    uint8_t ldoRailSelRegData   = 0;
    int32_t status              = PMIC_ST_SUCCESS;
    const Pmic_powerTps6522xLdoRegisters_t *pLdoRegisters = NULL;
    // clang-format on

    // Parameter check
    if (ldoPowerResourceCfg[ldoNum].validParams == 0)
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    // Get all LDO resource registers
    Pmic_get_tps6522x_pwrLdoRegs(&pLdoRegisters);

    // As we are about to read, start critical section
    Pmic_criticalSectionStart(pPmicCoreHandle);

    // Read all registers pertaining to the LDO resource
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_commIntf_recvByte(pPmicCoreHandle, pLdoRegisters[ldoNum].ldoCtrlRegAddr, &ldoCtrlRegData);
    }
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_commIntf_recvByte(pPmicCoreHandle, pLdoRegisters[ldoNum].ldoVoutRegAddr, &ldoVoutRegData);
    }
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_commIntf_recvByte(pPmicCoreHandle, pLdoRegisters[ldoNum].ldoPgWindowRegAddr, &ldoPgWindowRegData);
    }
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_commIntf_recvByte(pPmicCoreHandle, pLdoRegisters[ldoNum].ldoRailSelRegAddr, &ldoRailSelRegData);
    }

    // For each register pertaining to the LDO resource, modify the relevant bit fields of the register
    // (relevant bit fields are governed by validParams)
    if (status == PMIC_ST_SUCCESS)
    {
        // LDO_CTRL register
        Pmic_powerTps6522xSetLdoCtrlRegBitFields(ldoPowerResourceCfg[ldoNum], &ldoCtrlRegData);

        // LDO_VOUT register
        Pmic_powerTps6522xSetLdoVoutRegBitFields(ldoPowerResourceCfg, &ldoVoutRegData, ldoNum);

        // LDO_PG_WINDOW register
        Pmic_powerTps6522xSetLdoPgWindowRegBitFields(ldoPowerResourceCfg[ldoNum], &ldoPgWindowRegData);

        // RAIL_SEL_2 register
        Pmic_powerTps6522xSetRailSel2RegBitFields(ldoPowerResourceCfg, &ldoRailSelRegData, ldoNum);
    }

    // After modification of register bit fields, write values back to PMIC
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_commIntf_sendByte(pPmicCoreHandle, pLdoRegisters[ldoNum].ldoVoutRegAddr, ldoVoutRegData);
    }
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_commIntf_sendByte(pPmicCoreHandle, pLdoRegisters[ldoNum].ldoPgWindowRegAddr, ldoPgWindowRegData);
    }
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_commIntf_sendByte(pPmicCoreHandle, pLdoRegisters[ldoNum].ldoRailSelRegAddr, ldoRailSelRegData);
    }
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_commIntf_sendByte(pPmicCoreHandle, pLdoRegisters[ldoNum].ldoCtrlRegAddr, ldoCtrlRegData);
    }

    // After read-modify-write, stop critical section
    Pmic_criticalSectionStop(pPmicCoreHandle);

    return status;
}

static void
Pmic_powerTps6522xSetVccaVmonCtrlBitFields(const Pmic_powerTps6522xVccaVmonPowerResourceCfg_t vccaVmonPwrRsrcCfg,
                                           uint8_t                                           *vccaVmonCtrlRegData,
                                           const uint8_t                                      vmonNum)
{
    if (pmic_validParamCheck(vccaVmonPwrRsrcCfg.validParams, PMIC_POWER_TPS6522X_CFG_VMON_DEGLITCH_SEL_VALID))
    {
        Pmic_setBitField(vccaVmonCtrlRegData,
                         PMIC_POWER_TPS6522X_VCCA_VMON_CTRL_DEGLITCH_SEL_SHIFT,
                         PMIC_POWER_TPS6522X_VCCA_VMON_CTRL_DEGLITCH_SEL_MASK,
                         (uint8_t)vccaVmonPwrRsrcCfg.vmonDeglitchSel);
    }
    if (pmic_validParamCheck(vccaVmonPwrRsrcCfg.validParams, PMIC_POWER_TPS6522X_CFG_VMON2_EN_VALID))
    {
        Pmic_setBitField(vccaVmonCtrlRegData,
                         PMIC_POWER_TPS6522X_VCCA_VMON_CTRL_VMON2_EN_SHIFT,
                         PMIC_POWER_TPS6522X_VCCA_VMON_CTRL_VMON2_EN_MASK,
                         (uint8_t)vccaVmonPwrRsrcCfg.vmon2En);
    }
    if (pmic_validParamCheck(vccaVmonPwrRsrcCfg.validParams, PMIC_POWER_TPS6522X_CFG_VMON1_EN_VALID))
    {
        Pmic_setBitField(vccaVmonCtrlRegData,
                         PMIC_POWER_TPS6522X_VCCA_VMON_CTRL_VMON1_EN_SHIFT,
                         PMIC_POWER_TPS6522X_VCCA_VMON_CTRL_VMON1_EN_MASK,
                         (uint8_t)vccaVmonPwrRsrcCfg.vmon1En);
    }
    if (pmic_validParamCheck(vccaVmonPwrRsrcCfg.validParams, PMIC_POWER_TPS6522X_CFG_VCCA_VMON_EN_VALID))
    {
        Pmic_setBitField(vccaVmonCtrlRegData,
                         PMIC_POWER_TPS6522X_VCCA_VMON_CTRL_VCCA_VMON_EN_SHIFT,
                         PMIC_POWER_TPS6522X_VCCA_VMON_CTRL_VCCA_VMON_EN_MASK,
                         (uint8_t)vccaVmonPwrRsrcCfg.vccaVmonEn);
    }
}

static void
Pmic_powerTps6522xSetVccaPgWindowBitFields(const Pmic_powerTps6522xVccaVmonPowerResourceCfg_t vccaVmonPwrRsrcCfg,
                                           uint8_t                                           *vccaPgWindowRegData)
{
    if (pmic_validParamCheck(vccaVmonPwrRsrcCfg.validParams, PMIC_POWER_TPS6522X_CFG_VCCA_PG_LEVEL_VALID))
    {
        Pmic_setBitField(vccaPgWindowRegData,
                         PMIC_POWER_TPS6522X_VCCA_PG_WINDOW_VCCA_PG_SET_SHIFT,
                         PMIC_POWER_TPS6522X_VCCA_PG_WINDOW_VCCA_PG_SET_MASK,
                         (uint8_t)vccaVmonPwrRsrcCfg.vccaPgLevel);
    }
    if (pmic_validParamCheck(vccaVmonPwrRsrcCfg.validParams, PMIC_POWER_TPS6522X_CFG_VCCA_VMON_THR_VALID))
    {
        Pmic_setBitField(vccaPgWindowRegData,
                         PMIC_POWER_TPS6522X_VCCA_PG_WINDOW_VCCA_VMON_THR_SHIFT,
                         PMIC_POWER_TPS6522X_VCCA_PG_WINDOW_VCCA_VMON_THR_MASK,
                         (uint8_t)vccaVmonPwrRsrcCfg.vccaVmonThr);
    }
}

static void
Pmic_powerTps6522xSetVmonPgWindowBitFields(const Pmic_powerTps6522xVccaVmonPowerResourceCfg_t vccaVmonPwrRsrcCfg,
                                           uint8_t                                           *vmonPgWindowRegData,
                                           const uint8_t                                      vmonNum)
{
    if ((vmonNum == PMIC_POWER_TPS6522X_VOLTAGE_MONITOR_VMON1) &&
        pmic_validParamCheck(vccaVmonPwrRsrcCfg.validParams, PMIC_POWER_TPS6522X_CFG_VMON1_THR_VALID))
    {
        Pmic_setBitField(vmonPgWindowRegData,
                         PMIC_POWER_TPS6522X_VMON1_PG_WINDOW_VMON1_THR_SHIFT,
                         PMIC_POWER_TPS6522X_VMON1_PG_WINDOW_VMON1_THR_MASK,
                         (uint8_t)vccaVmonPwrRsrcCfg.vmon1Thr);
    }
    else if ((vmonNum == PMIC_POWER_TPS6522X_VOLTAGE_MONITOR_VMON2) &&
             pmic_validParamCheck(vccaVmonPwrRsrcCfg.validParams, PMIC_POWER_TPS6522X_CFG_VMON2_THR_VALID))
    {
        Pmic_setBitField(vmonPgWindowRegData,
                         PMIC_POWER_TPS6522X_VMON2_PG_WINDOW_VMON2_THR_SHIFT,
                         PMIC_POWER_TPS6522X_VMON2_PG_WINDOW_VMON2_THR_MASK,
                         (uint8_t)vccaVmonPwrRsrcCfg.vmon2Thr);
    }
}

static void Pmic_powerTps6522xVmonConvertVoltage2PgSet(uint8_t       *vmonPgLevel_pgSet,
                                                       const uint16_t vmonPgLevel_mv,
                                                       const uint8_t  vmonNum)
{
    uint16_t baseVoltage_vset = 0, baseVoltage_mv = 0, voltageStep = 0;

    switch (vmonNum)
    {
        case PMIC_POWER_TPS6522X_VOLTAGE_MONITOR_VMON1:
            if ((vmonPgLevel_mv >= PMIC_POWER_TPS6522X_VMON1_VOLTAGE_RANGE_1_MIN_VOLTAGE) &&
                (vmonPgLevel_mv <= PMIC_POWER_TPS6522X_VMON1_VOLTAGE_RANGE_1_MAX_VOLTAGE))
            {
                baseVoltage_vset = PMIC_POWER_TPS6522X_VMON1_VOLTAGE_RANGE_1_MIN_VSET;
                baseVoltage_mv = PMIC_POWER_TPS6522X_VMON1_VOLTAGE_RANGE_1_MIN_VOLTAGE;
                voltageStep = PMIC_POWER_TPS6522X_VOLTAGE_STEP_20_MV;
            }
            else if ((vmonPgLevel_mv >= PMIC_POWER_TPS6522X_VMON1_VOLTAGE_RANGE_2_MIN_VOLTAGE) &&
                     (vmonPgLevel_mv <= PMIC_POWER_TPS6522X_VMON1_VOLTAGE_RANGE_2_MAX_VOLTAGE))
            {
                baseVoltage_vset = PMIC_POWER_TPS6522X_VMON1_VOLTAGE_RANGE_2_MIN_VSET;
                baseVoltage_mv = PMIC_POWER_TPS6522X_VMON1_VOLTAGE_RANGE_2_MIN_VOLTAGE;
                voltageStep = PMIC_POWER_TPS6522X_VOLTAGE_STEP_5_MV;
            }
            else if ((vmonPgLevel_mv >= PMIC_POWER_TPS6522X_VMON1_VOLTAGE_RANGE_3_MIN_VOLTAGE) &&
                     (vmonPgLevel_mv <= PMIC_POWER_TPS6522X_VMON1_VOLTAGE_RANGE_3_MAX_VOLTAGE))
            {
                baseVoltage_vset = PMIC_POWER_TPS6522X_VMON1_VOLTAGE_RANGE_3_MIN_VSET;
                baseVoltage_mv = PMIC_POWER_TPS6522X_VMON1_VOLTAGE_RANGE_3_MIN_VOLTAGE;
                voltageStep = PMIC_POWER_TPS6522X_VOLTAGE_STEP_10_MV;
            }
            else if ((vmonPgLevel_mv >= PMIC_POWER_TPS6522X_VMON1_VOLTAGE_RANGE_4_MIN_VOLTAGE) &&
                     (vmonPgLevel_mv <= PMIC_POWER_TPS6522X_VMON1_VOLTAGE_RANGE_4_MAX_VOLTAGE))
            {
                baseVoltage_vset = PMIC_POWER_TPS6522X_VMON1_VOLTAGE_RANGE_4_MIN_VSET;
                baseVoltage_mv = PMIC_POWER_TPS6522X_VMON1_VOLTAGE_RANGE_4_MIN_VOLTAGE;
                voltageStep = PMIC_POWER_TPS6522X_VOLTAGE_STEP_20_MV;
            }

            break;
        case PMIC_POWER_TPS6522X_VOLTAGE_MONITOR_VMON2:
            if ((vmonPgLevel_mv >= PMIC_POWER_TPS6522X_VMON2_VOLTAGE_RANGE_1_MIN_VOLTAGE) &&
                (vmonPgLevel_mv <= PMIC_POWER_TPS6522X_VMON2_VOLTAGE_RANGE_1_MAX_VOLTAGE))
            {
                baseVoltage_vset = PMIC_POWER_TPS6522X_VMON2_VOLTAGE_RANGE_1_MIN_VSET;
                baseVoltage_mv = PMIC_POWER_TPS6522X_VMON2_VOLTAGE_RANGE_1_MIN_VOLTAGE;
                voltageStep = PMIC_POWER_TPS6522X_VOLTAGE_STEP_25_MV;
            }
            else if ((vmonPgLevel_mv >= PMIC_POWER_TPS6522X_VMON2_VOLTAGE_RANGE_2_MIN_VOLTAGE) &&
                     (vmonPgLevel_mv <= PMIC_POWER_TPS6522X_VMON2_VOLTAGE_RANGE_2_MAX_VOLTAGE))
            {
                baseVoltage_vset = PMIC_POWER_TPS6522X_VMON2_VOLTAGE_RANGE_2_MIN_VSET;
                baseVoltage_mv = PMIC_POWER_TPS6522X_VMON2_VOLTAGE_RANGE_2_MIN_VOLTAGE;
                voltageStep = PMIC_POWER_TPS6522X_VOLTAGE_STEP_50_MV;
            }

            break;
    }

    if ((baseVoltage_mv != 0) && (voltageStep != 0))
    {
        *vmonPgLevel_pgSet = baseVoltage_vset + ((vmonPgLevel_mv - baseVoltage_mv) / voltageStep);
    }
}

static void
Pmic_powerTps6522xSetVmonPgLevelBitFields(const Pmic_powerTps6522xVccaVmonPowerResourceCfg_t vccaVmonPwrRsrcCfg,
                                          uint8_t                                           *vmonPgLevelRegData,
                                          const uint8_t                                      vmonNum)
{
    if ((vmonNum == PMIC_POWER_TPS6522X_VOLTAGE_MONITOR_VMON1) &&
        pmic_validParamCheck(vccaVmonPwrRsrcCfg.validParams, PMIC_POWER_TPS6522X_CFG_VMON1_PG_LEVEL_MV_VALID))
    {
        Pmic_powerTps6522xVmonConvertVoltage2PgSet(vmonPgLevelRegData, vccaVmonPwrRsrcCfg.vmon1PgLevel_mv, vmonNum);
    }
    else if ((vmonNum == PMIC_POWER_TPS6522X_VOLTAGE_MONITOR_VMON2) &&
             pmic_validParamCheck(vccaVmonPwrRsrcCfg.validParams, PMIC_POWER_TPS6522X_CFG_VMON2_PG_LEVEL_MV_VALID))
    {
        Pmic_powerTps6522xVmonConvertVoltage2PgSet(vmonPgLevelRegData, vccaVmonPwrRsrcCfg.vmon2PgLevel_mv, vmonNum);
    }
}

static void
Pmic_powerTps6522xSetRailSel3RegBitFields(const Pmic_powerTps6522xVccaVmonPowerResourceCfg_t vccaVmonPwrRsrcCfg,
                                          uint8_t                                           *vccaVmonRailSelRegData,
                                          const uint8_t                                      vmonNum)
{
    if ((vmonNum == PMIC_POWER_TPS6522X_VOLTAGE_MONITOR_VMON1) &&
        pmic_validParamCheck(vccaVmonPwrRsrcCfg.validParams, PMIC_POWER_TPS6522X_CFG_VMON1_RAIL_GRP_SEL_VALID))
    {
        Pmic_setBitField(vccaVmonRailSelRegData,
                         PMIC_POWER_TPS6522X_RAIL_SEL_3_VMON1_GRP_SEL_SHIFT,
                         PMIC_POWER_TPS6522X_RAIL_SEL_3_VMON1_GRP_SEL_MASK,
                         (uint8_t)vccaVmonPwrRsrcCfg.vmon1RailGrpSel);
    }
    else if ((vmonNum == PMIC_POWER_TPS6522X_VOLTAGE_MONITOR_VMON2) &&
             pmic_validParamCheck(vccaVmonPwrRsrcCfg.validParams, PMIC_POWER_TPS6522X_CFG_VMON2_RAIL_GRP_SEL_VALID))
    {
        Pmic_setBitField(vccaVmonRailSelRegData,
                         PMIC_POWER_TPS6522X_RAIL_SEL_3_VMON2_GRP_SEL_SHIFT,
                         PMIC_POWER_TPS6522X_RAIL_SEL_3_VMON2_GRP_SEL_MASK,
                         (uint8_t)vccaVmonPwrRsrcCfg.vmon2RailGrpSel);
    }
    else if ((vmonNum == PMIC_POWER_TPS6522X_VOLTAGE_MONITOR_VCCA_VMON) &&
             pmic_validParamCheck(vccaVmonPwrRsrcCfg.validParams, PMIC_POWER_TPS6522X_CFG_VCCA_RAIL_GRP_SEL_VALID))
    {
        Pmic_setBitField(vccaVmonRailSelRegData,
                         PMIC_POWER_TPS6522X_RAIL_SEL_3_VCCA_GRP_SEL_SHIFT,
                         PMIC_POWER_TPS6522X_RAIL_SEL_3_VCCA_GRP_SEL_MASK,
                         (uint8_t)vccaVmonPwrRsrcCfg.vccaRailGrpSel);
    }
}

int32_t
Pmic_powerTps6522xSetVccaVmonPwrResourceCfg(Pmic_CoreHandle_t                                 *pPmicCoreHandle,
                                            const Pmic_powerTps6522xVccaVmonPowerResourceCfg_t vccaVmonPwrRsrcCfg,
                                            const uint8_t                                      vmonNum)
{
    // clang-format off
    uint8_t vccaVmonCtrlRegData     = 0;
    uint8_t vccaPgWindowRegData     = 0;
    uint8_t vmonPgWindowRegData     = 0;
    uint8_t vmonPgLevelRegData      = 0;
    uint8_t vccaVmonRailSelRegData  = 0;
    int32_t status                  = PMIC_ST_SUCCESS;
    const Pmic_powerTps6522xVccaVmonRegisters_t *pVccaVmonRegisters = NULL;
    // clang-format on

    // Parameter check
    if (vccaVmonPwrRsrcCfg.validParams == 0)
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    // Get VCCA_VMON and VMONx registers
    Pmic_get_tps6522x_PwrVccaVmonRegisters(&pVccaVmonRegisters);

    // As we are about to read, start critical section
    Pmic_criticalSectionStart(pPmicCoreHandle);

    // Read all registers pertaining to the VCCA_VMON/VMONx resource
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_commIntf_recvByte(
            pPmicCoreHandle, pVccaVmonRegisters[vmonNum].vccaVmonCtrlRegAddr, &vccaVmonCtrlRegData);
    }
    if ((status == PMIC_ST_SUCCESS) && (vmonNum == PMIC_POWER_TPS6522X_VOLTAGE_MONITOR_VCCA_VMON))
    {
        status = Pmic_commIntf_recvByte(
            pPmicCoreHandle, pVccaVmonRegisters[vmonNum].vccaPgWindowRegAddr, &vccaPgWindowRegData);
    }
    if ((status == PMIC_ST_SUCCESS) && ((vmonNum == PMIC_POWER_TPS6522X_VOLTAGE_MONITOR_VMON1) ||
                                        (vmonNum == PMIC_POWER_TPS6522X_VOLTAGE_MONITOR_VMON2)))
    {
        status = Pmic_commIntf_recvByte(
            pPmicCoreHandle, pVccaVmonRegisters[vmonNum].vmonPgWindowRegAddr, &vmonPgWindowRegData);
    }
    if ((status == PMIC_ST_SUCCESS) && ((vmonNum == PMIC_POWER_TPS6522X_VOLTAGE_MONITOR_VMON1) ||
                                        (vmonNum == PMIC_POWER_TPS6522X_VOLTAGE_MONITOR_VMON2)))
    {
        status = Pmic_commIntf_recvByte(
            pPmicCoreHandle, pVccaVmonRegisters[vmonNum].vmonPgLevelRegAddr, &vmonPgLevelRegData);
    }
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_commIntf_recvByte(
            pPmicCoreHandle, pVccaVmonRegisters[vmonNum].vccaVmonRailSelRegAddr, &vccaVmonRailSelRegData);
    }

    // For each register pertaining to the VCCA_VMON/VMONx resource, modify the relevant bit fields
    // of the register (relevant bit fields are governed by validParams)
    if (status == PMIC_ST_SUCCESS)
    {
        // VCCA_VMON_CTRL
        Pmic_powerTps6522xSetVccaVmonCtrlBitFields(vccaVmonPwrRsrcCfg, &vccaVmonCtrlRegData, vmonNum);

        // VCCA_PG_WINDOW
        Pmic_powerTps6522xSetVccaPgWindowBitFields(vccaVmonPwrRsrcCfg, &vccaPgWindowRegData);

        // VMON_PG_WINDOW
        Pmic_powerTps6522xSetVmonPgWindowBitFields(vccaVmonPwrRsrcCfg, &vmonPgWindowRegData, vmonNum);

        // VMON_PG_LEVEL
        Pmic_powerTps6522xSetVmonPgLevelBitFields(vccaVmonPwrRsrcCfg, &vmonPgLevelRegData, vmonNum);

        // RAIL_SEL_3
        Pmic_powerTps6522xSetRailSel3RegBitFields(vccaVmonPwrRsrcCfg, &vccaVmonRailSelRegData, vmonNum);
    }

    // After modification of register bit fields, write values back to PMIC
    if ((status == PMIC_ST_SUCCESS) && (vmonNum == PMIC_POWER_TPS6522X_VOLTAGE_MONITOR_VCCA_VMON))
    {
        status = Pmic_commIntf_sendByte(
            pPmicCoreHandle, pVccaVmonRegisters[vmonNum].vccaPgWindowRegAddr, vccaPgWindowRegData);
    }
    if ((status == PMIC_ST_SUCCESS) && ((vmonNum == PMIC_POWER_TPS6522X_VOLTAGE_MONITOR_VMON1) ||
                                        (vmonNum == PMIC_POWER_TPS6522X_VOLTAGE_MONITOR_VMON2)))
    {
        status = Pmic_commIntf_sendByte(
            pPmicCoreHandle, pVccaVmonRegisters[vmonNum].vmonPgWindowRegAddr, vmonPgWindowRegData);
    }
    if ((status == PMIC_ST_SUCCESS) && ((vmonNum == PMIC_POWER_TPS6522X_VOLTAGE_MONITOR_VMON1) ||
                                        (vmonNum == PMIC_POWER_TPS6522X_VOLTAGE_MONITOR_VMON2)))
    {
        status =
            Pmic_commIntf_sendByte(pPmicCoreHandle, pVccaVmonRegisters[vmonNum].vmonPgLevelRegAddr, vmonPgLevelRegData);
    }
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_commIntf_sendByte(
            pPmicCoreHandle, pVccaVmonRegisters[vmonNum].vccaVmonRailSelRegAddr, vccaVmonRailSelRegData);
    }
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_commIntf_sendByte(
            pPmicCoreHandle, pVccaVmonRegisters[vmonNum].vccaVmonCtrlRegAddr, vccaVmonCtrlRegData);
    }

    // After read-modify-write, stop critical section
    Pmic_criticalSectionStop(pPmicCoreHandle);

    return status;
}

int32_t Pmic_powerTps6522xSetPwrResourceCfg(Pmic_CoreHandle_t                         *pPmicCoreHandle,
                                            const Pmic_powerTps6522xPowerResourceCfg_t pwrResourceCfg)
{
    int32_t  status = PMIC_ST_SUCCESS;
    uint8_t  iter = 0;
    uint16_t validParam = 0;

    // Parameter check
    status = Pmic_powerTps6522xParamCheck_constPwrRsrcCfg(pPmicCoreHandle, pwrResourceCfg);
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_powerTps6522xVoltageWithinRangeCheck(pwrResourceCfg);
    }

    // If parameters are valid...
    if (status == PMIC_ST_SUCCESS)
    {
        // For each BUCK...
        for (iter = 0; iter < PMIC_POWER_TPS6522X_MAX_BUCK_NUM; iter++)
        {
            // Calculate BUCK validParam
            validParam = PMIC_POWER_TPS6522X_CFG_BUCK1_VALID + iter;

            // If BUCK validParam is set...
            if (pmic_validParamCheck(pwrResourceCfg.validParams, validParam))
            {
                // Set BUCK's power resource configuration
                status = Pmic_powerTps6522xSetBuckPwrResourceCfg(pPmicCoreHandle, pwrResourceCfg.buckPwrRsrcCfg, iter);
            }

            // Upon error, break out of loop
            if (status != PMIC_ST_SUCCESS)
            {
                break;
            }
        }
    }

    // If configuration of BUCK power resource was successful...
    if (status == PMIC_ST_SUCCESS)
    {
        // For each LDO...
        for (iter = 0; iter < PMIC_POWER_TPS6522X_MAX_LDO_NUM; iter++)
        {
            // Calculate LDO validParam
            validParam = PMIC_POWER_TPS6522X_CFG_LDO1_VALID + iter;

            // If LDO validParam is set...
            if (pmic_validParamCheck(pwrResourceCfg.validParams, validParam))
            {
                // Set LDO's power resource configuration
                status = Pmic_powerTps6522xSetLdoPwrResourceCfg(pPmicCoreHandle, pwrResourceCfg.ldoPwrRsrcCfg, iter);
            }

            // Upon error, break out of loop
            if (status != PMIC_ST_SUCCESS)
            {
                break;
            }
        }
    }

    // If configuration of LDO power resource was successful...
    if (status == PMIC_ST_SUCCESS)
    {
        // For each VMON...
        for (iter = 0; iter < PMIC_POWER_TPS6522X_MAX_VOLTAGE_MONITOR_NUM; iter++)
        {
            // Calculate VCCA_VMON/VMON validParam
            validParam = PMIC_POWER_TPS6522X_CFG_VMON1_VALID + iter;

            // If VCCA_VMON/VMON validParam is set...
            if (pmic_validParamCheck(pwrResourceCfg.validParams, validParam))
            {
                // Set VCCA_VMON/VMON's power resource configuration
                status = Pmic_powerTps6522xSetVccaVmonPwrResourceCfg(
                    pPmicCoreHandle, pwrResourceCfg.vccaVmonPwrRsrcCfg, iter);
            }

            // Upon error, break out of loop
            if (status != PMIC_ST_SUCCESS)
            {
                break;
            }
        }
    }

    return status;
}

static int32_t Pmic_powerTps6522xParamCheck_pmicHandle(Pmic_CoreHandle_t *pPmicCoreHandle)
{
    int32_t status = PMIC_ST_SUCCESS;

    // NULL handle
    if (pPmicCoreHandle == NULL)
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    // Invalid device type
    if ((status == PMIC_ST_SUCCESS) && (pPmicCoreHandle->pmicDeviceType != PMIC_DEV_BURTON_TPS6522X))
    {
        status = PMIC_ST_ERR_INV_DEVICE;
    }

    // Invalid subsystem
    if ((status == PMIC_ST_SUCCESS) && ((pPmicCoreHandle->pPmic_SubSysInfo->buckEnable == false) ||
                                        (pPmicCoreHandle->pPmic_SubSysInfo->ldoEnable == false)))
    {
        status = PMIC_ST_ERR_INV_SUBSYSTEM;
    }

    return status;
}

int32_t Pmic_powerTps6522xGetBuckStat(Pmic_CoreHandle_t        *pPmicCoreHandle,
                                      const pwrRsrcUVOVStatus_t pwrRsrcUVOVStatus,
                                      bool                     *pUnderOverVoltStat)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t statBuckRegData = 0;

    // Start critical section before reading
    Pmic_criticalSectionStart(pPmicCoreHandle);

    // Read STAT_BUCK register
    status = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_TPS6522X_STAT_BUCK_REGADDR, &statBuckRegData);

    // Extract desired UVOV status
    if (status == PMIC_ST_SUCCESS)
    {
        switch (pwrRsrcUVOVStatus)
        {
            case PMIC_POWER_TPS6522X_BUCK4_UVOV_STAT:
                *pUnderOverVoltStat = (bool)Pmic_getBitField(statBuckRegData,
                                                             PMIC_POWER_TPS6522X_STAT_BUCK_BUCK4_UVOV_STAT_SHIFT,
                                                             PMIC_POWER_TPS6522X_STAT_BUCK_BUCK4_UVOV_STAT_MASK);

                break;
            case PMIC_POWER_TPS6522X_BUCK3_UVOV_STAT:
                *pUnderOverVoltStat = (bool)Pmic_getBitField(statBuckRegData,
                                                             PMIC_POWER_TPS6522X_STAT_BUCK_BUCK3_UVOV_STAT_SHIFT,
                                                             PMIC_POWER_TPS6522X_STAT_BUCK_BUCK3_UVOV_STAT_MASK);

                break;
            case PMIC_POWER_TPS6522X_BUCK2_UVOV_STAT:
                *pUnderOverVoltStat = (bool)Pmic_getBitField(statBuckRegData,
                                                             PMIC_POWER_TPS6522X_STAT_BUCK_BUCK2_UVOV_STAT_SHIFT,
                                                             PMIC_POWER_TPS6522X_STAT_BUCK_BUCK2_UVOV_STAT_MASK);

                break;
            case PMIC_POWER_TPS6522X_BUCK1_UVOV_STAT:
                *pUnderOverVoltStat = (bool)Pmic_getBitField(statBuckRegData,
                                                             PMIC_POWER_TPS6522X_STAT_BUCK_BUCK1_UVOV_STAT_SHIFT,
                                                             PMIC_POWER_TPS6522X_STAT_BUCK_BUCK1_UVOV_STAT_MASK);

                break;
        }
    }

    // Stop critical section after reading
    Pmic_criticalSectionStop(pPmicCoreHandle);

    return status;
}

int32_t Pmic_powerTps6522xGetLdoVccaVmonStat(Pmic_CoreHandle_t        *pPmicCoreHandle,
                                             const pwrRsrcUVOVStatus_t pwrRsrcUVOVStatus,
                                             bool                     *pUnderOverVoltStat)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t statLdoVmonRegData = 0;

    // Start critical section before reading
    Pmic_criticalSectionStart(pPmicCoreHandle);

    // Read STAT_LDO_VMON register
    status = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_TPS6522X_STAT_LDO_VMON_REGADDR, &statLdoVmonRegData);

    // Extract desired UVOV status
    if (status == PMIC_ST_SUCCESS)
    {
        switch (pwrRsrcUVOVStatus)
        {
            case PMIC_POWER_TPS6522X_VMON2_UVOV_STAT:
                *pUnderOverVoltStat = (bool)Pmic_getBitField(statLdoVmonRegData,
                                                             PMIC_POWER_TPS6522X_STAT_LDO_VMON_VMON2_UVOV_STAT_SHIFT,
                                                             PMIC_POWER_TPS6522X_STAT_LDO_VMON_VMON2_UVOV_STAT_MASK);

                break;
            case PMIC_POWER_TPS6522X_VMON1_UVOV_STAT:
                *pUnderOverVoltStat = (bool)Pmic_getBitField(statLdoVmonRegData,
                                                             PMIC_POWER_TPS6522X_STAT_LDO_VMON_VMON1_UVOV_STAT_SHIFT,
                                                             PMIC_POWER_TPS6522X_STAT_LDO_VMON_VMON1_UVOV_STAT_MASK);

                break;
            case PMIC_POWER_TPS6522X_VCCA_UVOV_STAT:
                *pUnderOverVoltStat = (bool)Pmic_getBitField(statLdoVmonRegData,
                                                             PMIC_POWER_TPS6522X_STAT_LDO_VMON_VCCA_UVOV_STAT_SHIFT,
                                                             PMIC_POWER_TPS6522X_STAT_LDO_VMON_VCCA_UVOV_STAT_MASK);

                break;
            case PMIC_POWER_TPS6522X_LDO3_UVOV_STAT:
                *pUnderOverVoltStat = (bool)Pmic_getBitField(statLdoVmonRegData,
                                                             PMIC_POWER_TPS6522X_STAT_LDO_VMON_LDO3_UVOV_STAT_SHIFT,
                                                             PMIC_POWER_TPS6522X_STAT_LDO_VMON_LDO3_UVOV_STAT_MASK);

                break;
            case PMIC_POWER_TPS6522X_LDO2_UVOV_STAT:
                *pUnderOverVoltStat = (bool)Pmic_getBitField(statLdoVmonRegData,
                                                             PMIC_POWER_TPS6522X_STAT_LDO_VMON_LDO2_UVOV_STAT_SHIFT,
                                                             PMIC_POWER_TPS6522X_STAT_LDO_VMON_LDO2_UVOV_STAT_MASK);

                break;
            case PMIC_POWER_TPS6522X_LDO1_UVOV_STAT:
                *pUnderOverVoltStat = (bool)Pmic_getBitField(statLdoVmonRegData,
                                                             PMIC_POWER_TPS6522X_STAT_LDO_VMON_LDO1_UVOV_STAT_SHIFT,
                                                             PMIC_POWER_TPS6522X_STAT_LDO_VMON_LDO1_UVOV_STAT_MASK);

                break;
        }
    }

    // Stop critical section after reading
    Pmic_criticalSectionStop(pPmicCoreHandle);

    return status;
}

int32_t Pmic_powerTps6522xGetPwrRsrcStat(Pmic_CoreHandle_t        *pPmicCoreHandle,
                                         const pwrRsrcUVOVStatus_t pwrRsrcUVOVStatus,
                                         bool                     *pUnderOverVoltStat)
{
    int32_t status = PMIC_ST_SUCCESS;

    // Parameter check
    status = Pmic_powerTps6522xParamCheck_pmicHandle(pPmicCoreHandle);
    if ((status == PMIC_ST_SUCCESS) && (pUnderOverVoltStat == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }
    if ((status == PMIC_ST_SUCCESS) && ((uint8_t)pwrRsrcUVOVStatus > (uint8_t)PMIC_POWER_TPS6522X_VCCA_UVOV_STAT))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Get Buck UVOV status if user specified Buck UVOV status
        if (pwrRsrcUVOVStatus <= PMIC_POWER_TPS6522X_BUCK4_UVOV_STAT)
        {
            status = Pmic_powerTps6522xGetBuckStat(pPmicCoreHandle, pwrRsrcUVOVStatus, pUnderOverVoltStat);
        }
        // Get LDO or VCCA_VMON/VMONx UVOV status if user specified LDO or VCCA_VMON/VMONx UVOV status
        else
        {
            status = Pmic_powerTps6522xGetLdoVccaVmonStat(pPmicCoreHandle, pwrRsrcUVOVStatus, pUnderOverVoltStat);
        }
    }

    return status;
}

int32_t Pmic_powerTps6522xGetThermalStat(Pmic_CoreHandle_t               *pPmicCoreHandle,
                                         Pmic_powerTps6522xThermalStat_t *pThermalStat)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0;

    // Parameter check
    status = Pmic_powerTps6522xParamCheck_pmicHandle(pPmicCoreHandle);
    if ((status == PMIC_ST_SUCCESS) && (pThermalStat == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }
    if ((status == PMIC_ST_SUCCESS) && (pThermalStat->validParams == 0))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    // Start critical section before reading
    Pmic_criticalSectionStart(pPmicCoreHandle);

    // If TWARN_STAT validParam set, read STAT_MISC register and extract the TWARN_STAT bit
    if ((status == PMIC_ST_SUCCESS) && pmic_validParamCheck(pThermalStat->validParams, PMIC_THERMAL_STAT_WARN_VALID))
    {
        status = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_STAT_MISC_REGADDR, &regData);

        if (status == PMIC_ST_SUCCESS)
        {
            pThermalStat->twarnStat =
                Pmic_getBitField(regData, PMIC_POWER_TPS6522X_TWARN_STAT_SHIFT, PMIC_POWER_TPS6522X_TWARN_STAT_MASK);
        }
    }

    // If TWARN_ORD_STAT validParam set, read STAT_MODERATE_ERR register and extract the TSD_ORD_STAT bit
    if ((status == PMIC_ST_SUCCESS) &&
        pmic_validParamCheck(pThermalStat->validParams, PMIC_THERMAL_STAT_ORD_SHTDWN_VALID))
    {
        status = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_STAT_MODERATE_ERR_REGADDR, &regData);

        if (status == PMIC_ST_SUCCESS)
        {
            pThermalStat->tsdOrdStat = Pmic_getBitField(
                regData, PMIC_POWER_TPS6522X_TSD_ORD_STAT_SHIFT, PMIC_POWER_TPS6522X_TSD_ORD_STAT_MASK);
        }
    }

    // If TSD_IMM_STAT validParam set, read STAT_SEVERE_ERR register and extract the TSD_IMM_STAT bit
    if ((status == PMIC_ST_SUCCESS) &&
        pmic_validParamCheck(pThermalStat->validParams, PMIC_THERMAL_STAT_IMM_SHTDWN_VALID))
    {
        status = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_STAT_SEVERE_ERR_REGADDR, &regData);

        if (status == PMIC_ST_SUCCESS)
        {
            pThermalStat->tsdImmStat = Pmic_getBitField(
                regData, PMIC_POWER_TPS6522X_TSD_IMM_STAT_SHIFT, PMIC_POWER_TPS6522X_TSD_IMM_STAT_MASK);
        }
    }

    // Stop critical section after reading
    Pmic_criticalSectionStop(pPmicCoreHandle);

    return status;
}

int32_t Pmic_powerTps6522xGetThermalCfg(Pmic_CoreHandle_t *pPmicCoreHandle, Pmic_powerTps6522xThermalCfg_t *pThermalCfg)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0;

    // Parameter check
    status = Pmic_powerTps6522xParamCheck_pmicHandle(pPmicCoreHandle);
    if ((status == PMIC_ST_SUCCESS) && (pThermalCfg == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }
    if ((status == PMIC_ST_SUCCESS) && (pThermalCfg->validParams == 0))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    // Start critical section before reading
    Pmic_criticalSectionStart(pPmicCoreHandle);

    if (status == PMIC_ST_SUCCESS)
    {
        // Read CONFIG_1 register
        status = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_CONFIG_1_REGADDR, &regData);
    }

    // If TSD_ORD_LEVEL validParam is set, extract TSD_ORD_LEVEL bit
    if ((status == PMIC_ST_SUCCESS) &&
        pmic_validParamCheck(pThermalCfg->validParams, PMIC_POWER_TPS6522X_TSD_ORD_LEVEL_VALID))
    {
        pThermalCfg->tsdOrdLvl = (Pmic_powerTps6522xTsdOrdLvl_t)Pmic_getBitField(
            regData, PMIC_POWER_TPS6522X_TSD_ORD_LEVEL_SHIFT, PMIC_POWER_TPS6522X_TSD_ORD_LEVEL_MASK);
    }
    // If TWARN_LEVEL validParam is set, extract TWARN_LEVEL bit
    if ((status == PMIC_ST_SUCCESS) &&
        pmic_validParamCheck(pThermalCfg->validParams, PMIC_POWER_TPS6522X_TWARN_LEVEL_VALID))
    {
        pThermalCfg->twarnLvl = (Pmic_powerTps6522xTwarnLvl_t)Pmic_getBitField(
            regData, PMIC_POWER_TPS6522X_TWARN_LEVEL_SHIFT, PMIC_POWER_TPS6522X_TWARN_LEVEL_MASK);
    }

    // Stop critical section after reading
    Pmic_criticalSectionStop(pPmicCoreHandle);

    return status;
}

int32_t Pmic_powerTps6522xSetThermalCfg(Pmic_CoreHandle_t                   *pPmicCoreHandle,
                                        const Pmic_powerTps6522xThermalCfg_t thermalCfg)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0;

    // Parameter check
    status = Pmic_powerTps6522xParamCheck_pmicHandle(pPmicCoreHandle);
    if ((status == PMIC_ST_SUCCESS) && (thermalCfg.validParams == 0))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    // Start critical section before read-modify-write
    Pmic_criticalSectionStart(pPmicCoreHandle);

    // Read CONFIG_1 register
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_CONFIG_1_REGADDR, &regData);
    }

    // If TSD_ORD_LEVEL validParam is set, modify TSD_ORD_LEVEL bit field
    if ((status == PMIC_ST_SUCCESS) &&
        pmic_validParamCheck(thermalCfg.validParams, PMIC_POWER_TPS6522X_TSD_ORD_LEVEL_VALID))
    {
        Pmic_setBitField(&regData,
                         PMIC_POWER_TPS6522X_TSD_ORD_LEVEL_SHIFT,
                         PMIC_POWER_TPS6522X_TSD_ORD_LEVEL_MASK,
                         (uint8_t)thermalCfg.tsdOrdLvl);
    }

    // If TWARN_LEVEL validParam is set, modify TWARN_LEVEL bit field
    if ((status == PMIC_ST_SUCCESS) &&
        pmic_validParamCheck(thermalCfg.validParams, PMIC_POWER_TPS6522X_TWARN_LEVEL_VALID))
    {
        Pmic_setBitField(&regData,
                         PMIC_POWER_TPS6522X_TWARN_LEVEL_SHIFT,
                         PMIC_POWER_TPS6522X_TWARN_LEVEL_MASK,
                         (uint8_t)thermalCfg.twarnLvl);
    }

    // Write new CONFIG_1 register value back to PMIC
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_commIntf_sendByte(pPmicCoreHandle, PMIC_CONFIG_1_REGADDR, regData);
    }

    // Stop critical section after read-modify-write
    Pmic_criticalSectionStop(pPmicCoreHandle);

    return status;
}
