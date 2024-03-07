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
 *  \file   pmic_power_tps6522x.c
 *
 *  \brief  This source file contains TPS6522x Burton specific power API definitions
 *          and data structures.
 */

#include "pmic_types.h"
#include "pmic_power.h"
#include "pmic_core_priv.h"
#include "pmic_io_priv.h"
#include "pmic_power_tps6522x_priv.h"

const Pmic_powerTps6522xBuckRegisters_t gTps6522xBuckRegisters[] =
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

const Pmic_powerTps6522xLdoRegisters_t gTps6522xLdoRegisters[] =
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

const Pmic_powerTps6522xVccaVmonRegisters_t gTps6522xVccaVmonRegisters[] =
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

/**
 *  \brief      This function is used to verify the PMIC handle and pointer to TPS6522x power resource
 *              configuration struct.
 *
 *  \param      pPmicCoreHandle     [IN]        PMIC interface handle
 *  \param      pPwrResourceCfg     [IN]        Pointer to power resource configuration struct
 *
 *  \return     Success code if PMIC handle and power resource references are valid, error code otherwise.
 *              For valid success/error codes, refer to \ref Pmic_ErrorCodes
 */
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

/**
 *  \brief      This function is used to verify the PMIC handle and constant TPS6522x power resource
 *              configuration struct.
 *
 *  \param      pPmicCoreHandle     [IN]        PMIC interface handle
 *  \param      pwrResourceCfg      [IN]        Power resource configuration struct
 *
 *  \return     Success code if PMIC handle and constant power resource CFG are valid, error code otherwise.
 *              For valid success/error codes, refer to \ref Pmic_ErrorCodes
 */
static int32_t Pmic_powerTps6522xParamCheck_constPwrRsrcCfg(Pmic_CoreHandle_t                         *pPmicCoreHandle,
                                                            const Pmic_powerTps6522xPowerResourceCfg_t pwrResourceCfg)
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
    if ((status == PMIC_ST_SUCCESS) && (pwrResourceCfg.validParams == 0))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    return status;
}

/**
 *  \brief      This function is used to check whether a voltage is valid for a Buck.
 *
 *  \param      pBuckPwrRsrcCfg     [IN]        Array of Buck power resource configuration structs
 *  \param      buckNum             [IN]        Indicates which BUCK the API is working with
 *
 *  \return     Success code if Buck voltage is within range, error code otherwise.
 *              For valid success/error codes, refer to \ref Pmic_ErrorCodes
 */
static int32_t
Pmic_powerTps6522xBuckVoltageWithinRangeCheck(const Pmic_powerTps6522xBuckPowerResourceCfg_t *pBuckPwrRsrcCfg,
                                              const uint8_t                                   buckNum)
{
    int32_t status = PMIC_ST_SUCCESS;

    switch (buckNum)
    {
        case PMIC_POWER_TPS6522X_REGULATOR_BUCK1:
            if ((pBuckPwrRsrcCfg[buckNum].buckVoltage_mv < PMIC_POWER_TPS6522X_BUCK1_VOLTAGE_MIN_500_MV) ||
                (pBuckPwrRsrcCfg[buckNum].buckVoltage_mv > PMIC_POWER_TPS6522X_BUCK1_VOLTAGE_MAX_3300_MV))
            {
                status = PMIC_ST_ERR_INV_VOLTAGE;
            }

            break;
        case PMIC_POWER_TPS6522X_REGULATOR_BUCK2:
        case PMIC_POWER_TPS6522X_REGULATOR_BUCK3:
        case PMIC_POWER_TPS6522X_REGULATOR_BUCK4:
            if ((pBuckPwrRsrcCfg[buckNum].buckVoltage_mv < PMIC_POWER_TPS6522X_BUCK2_3_4_VOLTAGE_MIN_500_MV) ||
                (pBuckPwrRsrcCfg[buckNum].buckVoltage_mv > PMIC_POWER_TPS6522X_BUCK2_3_4_VOLTAGE_MAX_3300_MV))
            {
                status = PMIC_ST_ERR_INV_VOLTAGE;
            }

            break;
    }

    return status;
}

/**
 *  \brief      This function is used to check whether a voltage is valid for a LDO.
 *
 *  \param      pLdoPwrRsrcCfg      [IN]        Array of LDO power resource configuration structs
 *  \param      ldoNum              [IN]        Indicates which LDO the API is working with
 *
 *  \return     Success code if LDO voltage is within range, error code otherwise.
 *              For valid success/error codes, refer to \ref Pmic_ErrorCodes
 */
static int32_t
Pmic_powerTps6522xLdoVoltageWithinRangeCheck(const Pmic_powerTps6522xLdoPowerResourceCfg_t *pLdoPwrRsrcCfg,
                                             const uint8_t                                  ldoNum)
{
    int32_t status = PMIC_ST_SUCCESS;

    switch (ldoNum)
    {
        case PMIC_POWER_TPS6522X_REGULATOR_LDO1:
            if ((pLdoPwrRsrcCfg[ldoNum].ldoVoltage_mv < PMIC_POWER_TPS6522X_LDO1_VOLTAGE_MIN_1200_MV) ||
                (pLdoPwrRsrcCfg[ldoNum].ldoVoltage_mv > PMIC_POWER_TPS6522X_LDO1_VOLTAGE_MAX_3300_MV))
            {
                status = PMIC_ST_ERR_INV_VOLTAGE;
            }

            break;
        case PMIC_POWER_TPS6522X_REGULATOR_LDO2:
        case PMIC_POWER_TPS6522X_REGULATOR_LDO3:
            if ((pLdoPwrRsrcCfg[ldoNum].ldoVoltage_mv < PMIC_POWER_TPS6522X_LDO2_3_VOLTAGE_MIN_600_MV) ||
                (pLdoPwrRsrcCfg[ldoNum].ldoVoltage_mv > PMIC_POWER_TPS6522X_LDO2_3_VOLTAGE_MAX_3400_MV))
            {
                status = PMIC_ST_ERR_INV_VOLTAGE;
            }

            break;
    }

    return status;
}

/**
 *  \brief      This function is used to check whether a voltage is valid for a VMON.
 *
 *  \param      vccaVmonPwrRsrcCfg      [IN]        VCCA_VMON/VMONx power resource configuration struct
 *  \param      vmonNum                 [IN]        Indicates which VCCA/VMONx the API is working with
 *
 *  \return     Success code if VMON voltage is within range, error code otherwise.
 *              For valid success/error codes, refer to \ref Pmic_ErrorCodes
 */
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

/**
 *  \brief      This function is used to check whether voltages are within range for all valid power resources
 *              (valid power resources are governed by validParams).
 *
 *  \param      pwrResourceCfg      [IN]        TPS6522x power resource configuration struct
 *
 *  \return     Success code if the voltages of all valid power resources are within range, error code otherwise.
 *              For valid success/error codes, refer to \ref Pmic_ErrorCodes
 */
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

    // If BUCK voltages within range...
    if (status == PMIC_ST_SUCCESS)
    {
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
    }

    // If LDO voltages within range...
    if (status == PMIC_ST_SUCCESS)
    {
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
    }

    return status;
}

/**
 *  \brief      This function is used to store desired bit fields of the BUCK_CTRL register into the BUCK
 *              power resource CFG (desired bit fields are governed by validParams).
 *
 *  \param      pBuckPowerResourceCfg       [IN/OUT]    Pointer to BUCK power resource configuration struct
 *  \param      buckCtrlRegData             [IN]        BUCK_CTRL register data obtained from a prior read of the PMIC
 */
static void Pmic_powerTps6522xGetBuckCtrlRegBitFields(Pmic_powerTps6522xBuckPowerResourceCfg_t *pBuckPowerResourceCfg,
                                                      const uint8_t                             buckCtrlRegData)
{
    if (pmic_validParamCheck(pBuckPowerResourceCfg->validParams, PMIC_POWER_TPS6522X_CFG_BUCK_PLDN_VALID))
    {
        pBuckPowerResourceCfg->buckPldn = (uint8_t)Pmic_getBitField(
            buckCtrlRegData, PMIC_POWER_TPS6522X_BUCK_CTRL_PLDN_SHIFT, PMIC_POWER_TPS6522X_BUCK_CTRL_PLDN_MASK);
    }
    if (pmic_validParamCheck(pBuckPowerResourceCfg->validParams, PMIC_POWER_TPS6522X_CFG_BUCK_VMON_EN_VALID))
    {
        pBuckPowerResourceCfg->buckVmonEn = (uint8_t)Pmic_getBitField(
            buckCtrlRegData, PMIC_POWER_TPS6522X_BUCK_CTRL_VMON_EN_SHIFT, PMIC_POWER_TPS6522X_BUCK_CTRL_VMON_EN_MASK);
    }
    if (pmic_validParamCheck(pBuckPowerResourceCfg->validParams, PMIC_POWER_TPS6522X_CFG_BUCK_PWM_OPTION_VALID))
    {
        pBuckPowerResourceCfg->buckPwmOption =
            (uint8_t)Pmic_getBitField(buckCtrlRegData,
                                                                PMIC_POWER_TPS6522X_BUCK_CTRL_PWM_OPTION_SHIFT,
                                                                PMIC_POWER_TPS6522X_BUCK_CTRL_PWM_OPTION_MASK);
    }
    if (pmic_validParamCheck(pBuckPowerResourceCfg->validParams, PMIC_POWER_TPS6522X_CFG_BUCK_EN_VALID))
    {
        pBuckPowerResourceCfg->buckEn = (uint8_t)Pmic_getBitField(
            buckCtrlRegData, PMIC_POWER_TPS6522X_BUCK_CTRL_EN_SHIFT, PMIC_POWER_TPS6522X_BUCK_CTRL_EN_MASK);
    }
}

/**
 *  \brief      This function is used to store desired bit fields of the BUCK_CONF register into the BUCK
 *              power resource CFG (desired bit fields are governed by validParams).
 *
 *  \param      pBuckPowerResourceCfg       [IN/OUT]    Pointer to BUCK power resource configuration struct
 *  \param      buckConfRegData             [IN]        BUCK_CONF register data obtained from a prior read of the PMIC
 */
static void Pmic_powerTps6522xGetBuckConfRegBitFields(Pmic_powerTps6522xBuckPowerResourceCfg_t *pBuckPowerResourceCfg,
                                                      const uint8_t                             buckConfRegData)
{
    if (pmic_validParamCheck(pBuckPowerResourceCfg->validParams, PMIC_POWER_TPS6522X_CFG_BUCK_SLEW_RATE_VALID))
    {
        pBuckPowerResourceCfg->buckSlewRate =
            (uint8_t)Pmic_getBitField(buckConfRegData,
                                                               PMIC_POWER_TPS6522X_BUCK_CONF_SLEW_RATE_SHIFT,
                                                               PMIC_POWER_TPS6522X_BUCK_CONF_SLEW_RATE_MASK);
    }
}

/**
 *  \brief      This function is used to convert a BUCK VSET value to a BUCK voltage value.
 *
 *  \param      pBuckVoltage_mv     [OUT]       BUCK voltage value in millivolts
 *  \param      buckVoltage_vset    [IN]        BUCK voltage value in VSET form
 *  \param      buckNum             [IN]        Indicates which BUCK the API is working with
 */
static void Pmic_powerTps6522xBuckConvertVsetVal2Voltage(uint16_t     *pBuckVoltage_mv,
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
        *pBuckVoltage_mv = baseVoltage_mv + (voltageStep * (buckVoltage_vset - baseVoltage_vset));
    }
}

/**
 *  \brief      This function is used to convert BUCK_VOUT register data from VSET to voltage (mV)
 *              and store it into the BUCK power resource CFG struct (if validParam is set).
 *
 *  \param      pBuckPowerResourceCfg       [IN/OUT]    Pointer to BUCK power resource configuration struct
 *  \param      buckVoutRegData             [IN]        BUCK_VOUT register data obtained from a prior read of the PMIC
 *  \param      buckNum                     [IN]        Indicates which BUCK the API is working with
 */
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

/**
 *  \brief      This function is used to store desired bit fields of the BUCK_PG_WINDOW register into the BUCK
 *              power resource CFG struct (desired bit fields are governed by validParams).
 *
 *  \param      pBuckPowerResourceCfg       [IN/OUT]    Pointer to BUCK power resource configuration struct
 *  \param      buckPgWindowRegData         [IN]        BUCK_PG_WINDOW register data obtained from a prior read
 *                                                      of the PMIC
 */
static void
Pmic_powerTps6522xGetBuckPgWindowRegBitFields(Pmic_powerTps6522xBuckPowerResourceCfg_t *pBuckPowerResourceCfg,
                                              const uint8_t                             buckPgWindowRegData)
{
    if (pmic_validParamCheck(pBuckPowerResourceCfg->validParams, PMIC_POWER_TPS6522X_CFG_BUCK_VMON_THR_VALID))
    {
        pBuckPowerResourceCfg->buckVmonThr =
            (uint8_t)Pmic_getBitField(buckPgWindowRegData,
                                                              PMIC_POWER_TPS6522X_BUCK_PG_WINDOW_VMON_THR_SHIFT,
                                                              PMIC_POWER_TPS6522X_BUCK_PG_WINDOW_VMON_THR_MASK);
    }
}

/**
 *  \brief      This function is used to store desired bit fields of the RAIL_SEL_1 register into the BUCK
 *              power resource CFG struct (desired bit fields are governed by validParams).
 *
 *  \param      pBuckPowerResourceCfg       [IN/OUT]    Pointer to BUCK power resource configuration struct
 *  \param      buckRailSelRegData          [IN]        RAIL_SEL_1 register data obtained from a prior read of the PMIC
 *  \param      buckNum                     [IN]        Indicates which BUCK the API is working with
 */
static void Pmic_PowerTps6522xGetRailSel1RegBitFields(Pmic_powerTps6522xBuckPowerResourceCfg_t *pBuckPowerResourceCfg,
                                                      const uint8_t                             buckRailSelRegData,
                                                      const uint8_t                             buckNum)
{
    if (pmic_validParamCheck(pBuckPowerResourceCfg->validParams, PMIC_POWER_TPS6522X_CFG_BUCK_RAIL_GRP_SEL_VALID))
    {
        switch (buckNum)
        {
            case PMIC_POWER_TPS6522X_REGULATOR_BUCK1:
                pBuckPowerResourceCfg->buckRailGrpSel = (uint8_t)Pmic_getBitField(
                    buckRailSelRegData,
                    PMIC_POWER_TPS6522X_CFG_RAIL_SEL_1_BUCK1_GRP_SEL_SHIFT,
                    PMIC_POWER_TPS6522X_CFG_RAIL_SEL_1_BUCK1_GRP_SEL_MASK);

                break;
            case PMIC_POWER_TPS6522X_REGULATOR_BUCK2:
                pBuckPowerResourceCfg->buckRailGrpSel = (uint8_t)Pmic_getBitField(
                    buckRailSelRegData,
                    PMIC_POWER_TPS6522X_CFG_RAIL_SEL_1_BUCK2_GRP_SEL_SHIFT,
                    PMIC_POWER_TPS6522X_CFG_RAIL_SEL_1_BUCK2_GRP_SEL_MASK);

                break;
            case PMIC_POWER_TPS6522X_REGULATOR_BUCK3:
                pBuckPowerResourceCfg->buckRailGrpSel = (uint8_t)Pmic_getBitField(
                    buckRailSelRegData,
                    PMIC_POWER_TPS6522X_CFG_RAIL_SEL_1_BUCK3_GRP_SEL_SHIFT,
                    PMIC_POWER_TPS6522X_CFG_RAIL_SEL_1_BUCK3_GRP_SEL_MASK);

                break;
            case PMIC_POWER_TPS6522X_REGULATOR_BUCK4:
                pBuckPowerResourceCfg->buckRailGrpSel = (uint8_t)Pmic_getBitField(
                    buckRailSelRegData,
                    PMIC_POWER_TPS6522X_CFG_RAIL_SEL_1_BUCK4_GRP_SEL_SHIFT,
                    PMIC_POWER_TPS6522X_CFG_RAIL_SEL_1_BUCK4_GRP_SEL_MASK);

                break;
        }
    }
}

int32_t Pmic_powerTps6522xGetBuckPwrResourceCfg(Pmic_CoreHandle_t                        *pPmicCoreHandle,
                                                Pmic_powerTps6522xBuckPowerResourceCfg_t *pBuckPowerResourceCfg,
                                                const uint8_t                             buckNum)
{
    uint8_t buckCtrlRegData     = 0;
    uint8_t buckConfRegData     = 0;
    uint8_t buckVoutRegData     = 0;
    uint8_t buckPgWindowRegData = 0;
    uint8_t buckRailSelRegData  = 0;
    int32_t status              = PMIC_ST_SUCCESS;

    // Parameter check
    if (pBuckPowerResourceCfg == NULL)
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }
    if ((status == PMIC_ST_SUCCESS) && (pBuckPowerResourceCfg->validParams == 0))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    // As we are about to read, start critical section
    Pmic_criticalSectionStart(pPmicCoreHandle);

    // Read all registers pertaining to the BUCK resource
    if (status == PMIC_ST_SUCCESS)
    {
        status =
            Pmic_commIntf_recvByte(pPmicCoreHandle, gTps6522xBuckRegisters[buckNum].buckCtrlRegAddr, &buckCtrlRegData);
    }
    if (status == PMIC_ST_SUCCESS)
    {
        status =
            Pmic_commIntf_recvByte(pPmicCoreHandle, gTps6522xBuckRegisters[buckNum].buckConfRegAddr, &buckConfRegData);
    }
    if (status == PMIC_ST_SUCCESS)
    {
        status =
            Pmic_commIntf_recvByte(pPmicCoreHandle, gTps6522xBuckRegisters[buckNum].buckVoutRegAddr, &buckVoutRegData);
    }
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_commIntf_recvByte(
            pPmicCoreHandle, gTps6522xBuckRegisters[buckNum].buckPgWindowRegAddr, &buckPgWindowRegData);
    }
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_commIntf_recvByte(
            pPmicCoreHandle, gTps6522xBuckRegisters[buckNum].buckRailSelRegAddr, &buckRailSelRegData);
    }

    // After read, stop critical section
    Pmic_criticalSectionStop(pPmicCoreHandle);

    // Get the desired bit fields from the registers that were read
    // (desired bit fields are governed by validParams)
    if (status == PMIC_ST_SUCCESS)
    {
        // BUCK_CTRL register
        Pmic_powerTps6522xGetBuckCtrlRegBitFields(pBuckPowerResourceCfg, buckCtrlRegData);

        // BUCK_CONF register
        Pmic_powerTps6522xGetBuckConfRegBitFields(pBuckPowerResourceCfg, buckConfRegData);

        // BUCK_VOUT register
        Pmic_powerTps6522xGetBuckVoutRegBitFields(pBuckPowerResourceCfg, buckVoutRegData, buckNum);

        // BUCK_PG_WINDOW register
        Pmic_powerTps6522xGetBuckPgWindowRegBitFields(pBuckPowerResourceCfg, buckPgWindowRegData);

        // RAIL_SEL_1 register
        Pmic_PowerTps6522xGetRailSel1RegBitFields(pBuckPowerResourceCfg, buckRailSelRegData, buckNum);
    }

    return status;
}

/**
 *  \brief      This function is used to store desired bit fields of the LDO_CTRL register into the LDO
 *              power resource CFG struct (desired bit fields are governed by validParams).
 *
 *  \param      pLdoPowerResourceCfg        [IN/OUT]    Pointer to LDO power resource configuration struct
 *  \param      ldoCtrlRegData              [IN]        LDO_CTRL register data obtained from a prior read of the PMIC
 */
static void Pmic_powerTps6522xGetLdoCtrlRegBitFields(Pmic_powerTps6522xLdoPowerResourceCfg_t *pLdoPowerResourceCfg,
                                                     const uint8_t                            ldoCtrlRegData)
{
    if (pmic_validParamCheck(pLdoPowerResourceCfg->validParams, PMIC_POWER_TPS6522X_CFG_LDO_DISCHARGE_EN_VALID))
    {
        pLdoPowerResourceCfg->ldoDischargeEn =
            (uint8_t)Pmic_getBitField(ldoCtrlRegData,
                                                                 PMIC_POWER_TPS6522X_LDO_CTRL_DISCHARGE_EN_SHIFT,
                                                                 PMIC_POWER_TPS6522X_LDO_CTRL_DISCHARGE_EN_MASK);
    }
    if (pmic_validParamCheck(pLdoPowerResourceCfg->validParams, PMIC_POWER_TPS6522X_CFG_LDO_VMON_EN_VALID))
    {
        pLdoPowerResourceCfg->ldoVmonEn = (uint8_t)Pmic_getBitField(
            ldoCtrlRegData, PMIC_POWER_TPS6522X_LDO_CTRL_VMON_EN_SHIFT, PMIC_POWER_TPS6522X_LDO_CTRL_VMON_EN_MASK);
    }
    if (pmic_validParamCheck(pLdoPowerResourceCfg->validParams, PMIC_POWER_TPS6522X_CFG_LDO_EN_VALID))
    {
        pLdoPowerResourceCfg->ldoEn = (uint8_t)Pmic_getBitField(
            ldoCtrlRegData, PMIC_POWER_TPS6522X_LDO_CTRL_EN_SHIFT, PMIC_POWER_TPS6522X_LDO_CTRL_EN_MASK);
    }
}

/**
 *  \brief      This function is used to convert a LDO VSET value to LDO voltage value (mV).
 *
 *  \param      pLdoVoltage_mv      [OUT]       LDO voltage value in millivolts
 *  \param      ldoVoltage_vset     [IN]        LDO voltage value in VSET form
 *  \param      ldoNum              [IN]        Indicates which LDO the API is working with
 */
static void Pmic_powerTps6522xLdoConvertVsetVal2Voltage(uint16_t     *pLdoVoltage_mv,
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
        *pLdoVoltage_mv = baseVoltage_mv + (voltageStep * (ldoVoltage_vset - baseVoltage_vset));
    }
}

/**
 *  \brief      This function is used to get the desired bit fields of the LDO_VOUT register
 *              (desired bit fields are governed by validParams).
 *
 *  \param      pLdoPowerResourceCfg        [IN/OUT]    Pointer to LDO power resource configuration struct
 *  \param      ldoVoutRegData              [IN]        LDO_VOUT register data obtained from a prior read of the PMIC
 *  \param      ldoNum                      [IN]        Indicates which LDO the API is working with
 */
static void Pmic_powerTps6522xGetLdoVoutRegBitFields(Pmic_powerTps6522xLdoPowerResourceCfg_t *pLdoPowerResourceCfg,
                                                     const uint8_t                            ldoVoutRegData,
                                                     const uint8_t                            ldoNum)
{
    uint8_t ldoVoltage_vset = 0;

    if (pmic_validParamCheck(pLdoPowerResourceCfg->validParams, PMIC_POWER_TPS6522X_CFG_LDO_MODE_VALID))
    {
        pLdoPowerResourceCfg->ldoMode = (uint8_t)Pmic_getBitField(
            ldoVoutRegData, PMIC_POWER_TPS6522X_LDO_VOUT_LDO_MODE_SHIFT, PMIC_POWER_TPS6522X_LDO_VOUT_LDO_MODE_MASK);
    }
    if (pmic_validParamCheck(pLdoPowerResourceCfg->validParams, PMIC_POWER_TPS6522X_CFG_LDO_VOLTAGE_MV_VALID))
    {
        ldoVoltage_vset = Pmic_getBitField(
            ldoVoutRegData, PMIC_POWER_TPS6522X_LDO_VOUT_LDO_VSET_SHIFT, PMIC_POWER_TPS6522X_LDO_VOUT_LDO_VSET_MASK);

        Pmic_powerTps6522xLdoConvertVsetVal2Voltage(&(pLdoPowerResourceCfg->ldoVoltage_mv), ldoVoltage_vset, ldoNum);
    }
}

/**
 *  \brief      This function is used to store desired bit fields of the LDO_PG_WINDOW register into the LDO
 *              power resource CFG struct (desired bit fields are governed by validParams).
 *
 *  \param      pLdoPowerResourceCfg        [IN/OUT]    Pointer to LDO power resource configuration struct
 *  \param      ldoPgWindowRegData          [IN]        LDO_PG_WINDOW register data obtained from a prior read
 *                                                      of the PMIC
 */
static void Pmic_powerTps6522xGetLdoPgWindowRegBitFields(Pmic_powerTps6522xLdoPowerResourceCfg_t *pLdoPowerResourceCfg,
                                                         const uint8_t                            ldoPgWindowRegData)
{
    if (pmic_validParamCheck(pLdoPowerResourceCfg->validParams, PMIC_POWER_TPS6522X_CFG_LDO_VMON_THR_VALID))
    {
        pLdoPowerResourceCfg->ldoVmonThr =
            (uint8_t)Pmic_getBitField(ldoPgWindowRegData,
                                                             PMIC_POWER_TPS6522X_LDO_PG_WINDOW_LDO_VMON_THR_SHIFT,
                                                             PMIC_POWER_TPS6522X_LDO_PG_WINDOW_LDO_VMON_THR_MASK);
    }
}

/**
 *  \brief      This function is used to store desired bit fields of the RAIL_SEL_2 register into the LDO
 *              power resource CFG struct (desired bit fields are governed by validParams).
 *
 *  \param      pLdoPowerResourceCfg        [IN/OUT]    Pointer to LDO power resource configuration struct
 *  \param      ldoRailSelRegData           [IN]        RAIL_SEL_2 register data obtained from a prior read of the PMIC
 *  \param      ldoNum                      [IN]        Indicates which LDO the API is working with
 */
static void Pmic_powerTps6522xGetRailSel2RegBitFields(Pmic_powerTps6522xLdoPowerResourceCfg_t *pLdoPowerResourceCfg,
                                                      const uint8_t                            ldoRailSelRegData,
                                                      const uint8_t                            ldoNum)
{
    if (pmic_validParamCheck(pLdoPowerResourceCfg->validParams, PMIC_POWER_TPS6522X_CFG_LDO_RAIL_GRP_SEL_VALID))
    {
        switch (ldoNum)
        {
            case PMIC_POWER_TPS6522X_REGULATOR_LDO1:
                pLdoPowerResourceCfg->ldoRailGrpSel =
                    (uint8_t)Pmic_getBitField(ldoRailSelRegData,
                                                                     PMIC_POWER_TPS6522X_RAIL_SEL_2_LDO1_GRP_SEL_SHIFT,
                                                                     PMIC_POWER_TPS6522X_RAIL_SEL_2_LDO1_GRP_SEL_MASK);

                break;
            case PMIC_POWER_TPS6522X_REGULATOR_LDO2:
                pLdoPowerResourceCfg->ldoRailGrpSel =
                    (uint8_t)Pmic_getBitField(ldoRailSelRegData,
                                                                     PMIC_POWER_TPS6522X_RAIL_SEL_2_LDO2_GRP_SEL_SHIFT,
                                                                     PMIC_POWER_TPS6522X_RAIL_SEL_2_LDO2_GRP_SEL_MASK);

                break;
            case PMIC_POWER_TPS6522X_REGULATOR_LDO3:
                pLdoPowerResourceCfg->ldoRailGrpSel =
                    (uint8_t)Pmic_getBitField(ldoRailSelRegData,
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
    uint8_t ldoCtrlRegData      = 0;
    uint8_t ldoVoutRegData      = 0;
    uint8_t ldoPgWindowRegData  = 0;
    uint8_t ldoRailSelRegData   = 0;
    int32_t status              = PMIC_ST_SUCCESS;

    // Parameter check
    if (pLdoPowerResourceCfg == NULL)
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }
    if ((status == PMIC_ST_SUCCESS) && (pLdoPowerResourceCfg->validParams == 0))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    // As we are about to read, start critical section
    Pmic_criticalSectionStart(pPmicCoreHandle);

    // Read all registers pertaining to the LDO resource
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_commIntf_recvByte(pPmicCoreHandle, gTps6522xLdoRegisters[ldoNum].ldoCtrlRegAddr, &ldoCtrlRegData);
    }
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_commIntf_recvByte(pPmicCoreHandle, gTps6522xLdoRegisters[ldoNum].ldoVoutRegAddr, &ldoVoutRegData);
    }
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_commIntf_recvByte(
            pPmicCoreHandle, gTps6522xLdoRegisters[ldoNum].ldoPgWindowRegAddr, &ldoPgWindowRegData);
    }
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_commIntf_recvByte(
            pPmicCoreHandle, gTps6522xLdoRegisters[ldoNum].ldoRailSelRegAddr, &ldoRailSelRegData);
    }

    // After read, stop critical section
    Pmic_criticalSectionStop(pPmicCoreHandle);

    // Get the desired bit fields from the registers that were read
    // (desired bit fields are governed by validParams)
    if (status == PMIC_ST_SUCCESS)
    {
        // LDO_CTRL register
        Pmic_powerTps6522xGetLdoCtrlRegBitFields(pLdoPowerResourceCfg, ldoCtrlRegData);

        // LDO_VOUT register
        Pmic_powerTps6522xGetLdoVoutRegBitFields(pLdoPowerResourceCfg, ldoVoutRegData, ldoNum);

        // LDO_PG_WINDOW register
        Pmic_powerTps6522xGetLdoPgWindowRegBitFields(pLdoPowerResourceCfg, ldoPgWindowRegData);

        // RAIL_SEL_2 register
        Pmic_powerTps6522xGetRailSel2RegBitFields(pLdoPowerResourceCfg, ldoRailSelRegData, ldoNum);
    }

    return status;
}

/**
 *  \brief      This function is used to store desired bit fields of the VCCA_VMON_CTRL register into the VCCA/VMON
 *              power resource CFG struct (desired bit fields are governed by validParams).
 *
 *  \param      pVccaVmonPwrRsrcCfg     [IN/OUT]        Pointer to VCCA/VMON power resource configuration struct
 *  \param      vccaVmonCtrlRegData     [IN]            VCCA_VMON_CTRL register data obtained from a prior read of
 *                                                      the PMIC
 *  \param      vmonNum                 [IN]            Indicates which VCCA/VMONx the API is working with
 */
static void
Pmic_powerTps6522xGetVccaVmonCtrlBitFields(Pmic_powerTps6522xVccaVmonPowerResourceCfg_t *pVccaVmonPwrRsrcCfg,
                                           uint8_t                                       vccaVmonCtrlRegData,
                                           const uint8_t                                 vmonNum)
{
    if ((vmonNum == PMIC_POWER_TPS6522X_VOLTAGE_MONITOR_VMON1) &&
        pmic_validParamCheck(pVccaVmonPwrRsrcCfg->validParams, PMIC_POWER_TPS6522X_CFG_VMON1_EN_VALID))
    {
        pVccaVmonPwrRsrcCfg->vmon1En =
            (uint8_t)Pmic_getBitField(vccaVmonCtrlRegData,
                                                          PMIC_POWER_TPS6522X_VCCA_VMON_CTRL_VMON1_EN_SHIFT,
                                                          PMIC_POWER_TPS6522X_VCCA_VMON_CTRL_VMON1_EN_MASK);
    }
    else if ((vmonNum == PMIC_POWER_TPS6522X_VOLTAGE_MONITOR_VMON2) &&
             pmic_validParamCheck(pVccaVmonPwrRsrcCfg->validParams, PMIC_POWER_TPS6522X_CFG_VMON2_EN_VALID))
    {
        pVccaVmonPwrRsrcCfg->vmon2En =
            (uint8_t)Pmic_getBitField(vccaVmonCtrlRegData,
                                                          PMIC_POWER_TPS6522X_VCCA_VMON_CTRL_VMON2_EN_SHIFT,
                                                          PMIC_POWER_TPS6522X_VCCA_VMON_CTRL_VMON2_EN_MASK);
    }
    else if ((vmonNum == PMIC_POWER_TPS6522X_VOLTAGE_MONITOR_VCCA_VMON) &&
             pmic_validParamCheck(pVccaVmonPwrRsrcCfg->validParams, PMIC_POWER_TPS6522X_CFG_VCCA_VMON_EN_VALID))
    {
        pVccaVmonPwrRsrcCfg->vccaVmonEn =
            (uint8_t)Pmic_getBitField(vccaVmonCtrlRegData,
                                                             PMIC_POWER_TPS6522X_VCCA_VMON_CTRL_VCCA_VMON_EN_SHIFT,
                                                             PMIC_POWER_TPS6522X_VCCA_VMON_CTRL_VCCA_VMON_EN_MASK);
    }

    if (pmic_validParamCheck(pVccaVmonPwrRsrcCfg->validParams, PMIC_POWER_TPS6522X_CFG_VMON_DEGLITCH_SEL_VALID))
    {
        pVccaVmonPwrRsrcCfg->vmonDeglitchSel =
            (uint8_t)Pmic_getBitField(vccaVmonCtrlRegData,
                                                                  PMIC_POWER_TPS6522X_VCCA_VMON_CTRL_DEGLITCH_SEL_SHIFT,
                                                                  PMIC_POWER_TPS6522X_VCCA_VMON_CTRL_DEGLITCH_SEL_MASK);
    }
}

/**
 *  \brief      This function is used to store desired bit fields of the VCCA_PG_WINDOW register into the VCCA/VMON
 *              power resource CFG struct (desired bit fields are governed by validParams).
 *
 *  \param      pVccaVmonPwrRsrcCfg     [IN/OUT]        Pointer to VCCA/VMON power resource configuration struct
 *  \param      vccaPgWindowRegData     [IN]            VCCA_PG_WINDOW register data
 */
static Pmic_powerTps6522xGetVccaPgWindowBitFields(Pmic_powerTps6522xVccaVmonPowerResourceCfg_t *pVccaVmonPwrRsrcCfg,
                                                  uint8_t                                       vccaPgWindowRegData)
{
    if (pmic_validParamCheck(pVccaVmonPwrRsrcCfg->validParams, PMIC_POWER_TPS6522X_CFG_VCCA_PG_LEVEL_VALID))
    {
        pVccaVmonPwrRsrcCfg->vccaPgLevel =
            (uint8_t)Pmic_getBitField(vccaPgWindowRegData,
                                                              PMIC_POWER_TPS6522X_VCCA_PG_WINDOW_VCCA_PG_SET_SHIFT,
                                                              PMIC_POWER_TPS6522X_VCCA_PG_WINDOW_VCCA_PG_SET_MASK);
    }
    if (pmic_validParamCheck(pVccaVmonPwrRsrcCfg->validParams, PMIC_POWER_TPS6522X_CFG_VCCA_VMON_THR_VALID))
    {
        pVccaVmonPwrRsrcCfg->vccaVmonThr =
            (uint8_t)Pmic_getBitField(vccaPgWindowRegData,
                                                              PMIC_POWER_TPS6522X_VCCA_PG_WINDOW_VCCA_VMON_THR_SHIFT,
                                                              PMIC_POWER_TPS6522X_VCCA_PG_WINDOW_VCCA_VMON_THR_MASK);
    }
}

/**
 *  \brief      This function is used to store desired bit fields of the VMON_PG_WINDOW register into the VCCA/VMON
 *              power resource CFG struct (desired bit fields are governed by validParams).
 *
 *  \param      pVccaVmonPwrRsrcCfg     [IN/OUT]        Pointer to VCCA/VMON power resource configuration struct
 *  \param      vmonPgWindowRegData     [IN]            VMON_PG_WINDOW register data obtained from a prior read of
 *                                                      the PMIC
 *  \param      vmonNum                 [IN]            Indicates which VCCA/VMONx the API is working with
 */
static Pmic_powerTps6522xGetVmonPgWindowBitFields(Pmic_powerTps6522xVccaVmonPowerResourceCfg_t *pVccaVmonPwrRsrcCfg,
                                                  uint8_t                                       vmonPgWindowRegData,
                                                  const uint8_t                                 vmonNum)
{
    if ((vmonNum == PMIC_POWER_TPS6522X_VOLTAGE_MONITOR_VMON1) &&
        pmic_validParamCheck(pVccaVmonPwrRsrcCfg->validParams, PMIC_POWER_TPS6522X_CFG_VMON1_THR_VALID))
    {
        pVccaVmonPwrRsrcCfg->vmon1Thr =
            (uint8_t)Pmic_getBitField(vmonPgWindowRegData,
                                                           PMIC_POWER_TPS6522X_VMON1_PG_WINDOW_VMON1_THR_SHIFT,
                                                           PMIC_POWER_TPS6522X_VMON1_PG_WINDOW_VMON1_THR_MASK);
    }
    else if ((vmonNum == PMIC_POWER_TPS6522X_VOLTAGE_MONITOR_VMON2) &&
             pmic_validParamCheck(pVccaVmonPwrRsrcCfg->validParams, PMIC_POWER_TPS6522X_CFG_VMON2_THR_VALID))
    {
        pVccaVmonPwrRsrcCfg->vmon2Thr =
            (uint8_t)Pmic_getBitField(vmonPgWindowRegData,
                                                           PMIC_POWER_TPS6522X_VMON2_PG_WINDOW_VMON2_THR_SHIFT,
                                                           PMIC_POWER_TPS6522X_VMON2_PG_WINDOW_VMON2_THR_MASK);
    }
}

/**
 *  \brief      This function is used to convert a VMON PG_SET value to VMON voltage value (mV).
 *
 *  \param      pVmonPgLevel_mv         [OUT]       VMON voltage value in millivolts
 *  \param      vmonPgLevel_pgSet       [IN]        VMON voltage value in PG_SET form
 *  \param      vmonNum                 [IN]        Indicates which VCCA/VMONx the API is working with
 */
static void Pmic_powerTps6522xVmonConvertPgSet2Voltage(uint16_t     *pVmonPgLevel_mv,
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
        *pVmonPgLevel_mv = baseVoltage_mv + (voltageStep * (vmonPgLevel_pgSet - baseVoltage_pgSet));
    }
}

/**
 *  \brief      This function is used to convert VMON_PG_LEVEL register data from PG_SET to voltage (mV)
 *              and store it into the VCCA/VMON power resource CFG struct (if validParam is set).
 *
 *  \param      pVccaVmonPwrRsrcCfg     [IN/OUT]    Pointer to VCCA/VMON power resource configuration struct
 *  \param      vmonPgLevelRegData      [IN]        VMON_PG_LEVEL register data obtained from a prior read of the PMIC
 *  \param      vmonNum                 [IN]        Indicates which VCCA/VMONx the API is working with
 */
static Pmic_powerTps6522xGetVmonPgLevelBitFields(Pmic_powerTps6522xVccaVmonPowerResourceCfg_t *pVccaVmonPwrRsrcCfg,
                                                 uint8_t                                       vmonPgLevelRegData,
                                                 const uint8_t                                 vmonNum)
{
    if ((vmonNum == PMIC_POWER_TPS6522X_VOLTAGE_MONITOR_VMON1) &&
        pmic_validParamCheck(pVccaVmonPwrRsrcCfg->validParams, PMIC_POWER_TPS6522X_CFG_VMON1_PG_LEVEL_MV_VALID))
    {
        Pmic_powerTps6522xVmonConvertPgSet2Voltage(
            &(pVccaVmonPwrRsrcCfg->vmon1PgLevel_mv), vmonPgLevelRegData, vmonNum);
    }
    else if ((vmonNum == PMIC_POWER_TPS6522X_VOLTAGE_MONITOR_VMON2) &&
             pmic_validParamCheck(pVccaVmonPwrRsrcCfg->validParams, PMIC_POWER_TPS6522X_CFG_VMON2_PG_LEVEL_MV_VALID))
    {
        Pmic_powerTps6522xVmonConvertPgSet2Voltage(
            &(pVccaVmonPwrRsrcCfg->vmon2PgLevel_mv), vmonPgLevelRegData, vmonNum);
    }
}

/**
 *  \brief      This function is used to store desired bit fields of the RAIL_SEL_3 register into the VCCA/VMON
 *              power resource CFG struct (desired bit fields are governed by validParams).
 *
 *  \param      pVccaVmonPwrRsrcCfg         [IN/OUT]    Pointer to VCCA/VMON power resource configuration struct
 *  \param      vccaVmonRailSelRegData      [IN]        RAIL_SEL_3 register data obtained from a prior read of the PMIC
 *  \param      vmonNum                     [IN]        Indicates which VCCA/VMONx the API is working with
 */
static Pmic_powerTps6522xGetRailSel3RegBitFields(Pmic_powerTps6522xVccaVmonPowerResourceCfg_t *pVccaVmonPwrRsrcCfg,
                                                 uint8_t                                       vccaVmonRailSelRegData,
                                                 const uint8_t                                 vmonNum)
{
    if ((vmonNum == PMIC_POWER_TPS6522X_VOLTAGE_MONITOR_VMON1) &&
        pmic_validParamCheck(pVccaVmonPwrRsrcCfg->validParams, PMIC_POWER_TPS6522X_CFG_VMON1_RAIL_GRP_SEL_VALID))
    {
        pVccaVmonPwrRsrcCfg->vmon1RailGrpSel =
            (uint8_t)Pmic_getBitField(vccaVmonRailSelRegData,
                                                               PMIC_POWER_TPS6522X_RAIL_SEL_3_VMON1_GRP_SEL_SHIFT,
                                                               PMIC_POWER_TPS6522X_RAIL_SEL_3_VMON1_GRP_SEL_MASK);
    }
    else if ((vmonNum == PMIC_POWER_TPS6522X_VOLTAGE_MONITOR_VMON2) &&
             pmic_validParamCheck(pVccaVmonPwrRsrcCfg->validParams, PMIC_POWER_TPS6522X_CFG_VMON2_RAIL_GRP_SEL_VALID))
    {
        pVccaVmonPwrRsrcCfg->vmon2RailGrpSel =
            (uint8_t)Pmic_getBitField(vccaVmonRailSelRegData,
                                                               PMIC_POWER_TPS6522X_RAIL_SEL_3_VMON2_GRP_SEL_SHIFT,
                                                               PMIC_POWER_TPS6522X_RAIL_SEL_3_VMON2_GRP_SEL_MASK);
    }
    else if ((vmonNum == PMIC_POWER_TPS6522X_VOLTAGE_MONITOR_VCCA_VMON) &&
             pmic_validParamCheck(pVccaVmonPwrRsrcCfg->validParams, PMIC_POWER_TPS6522X_CFG_VCCA_RAIL_GRP_SEL_VALID))
    {
        pVccaVmonPwrRsrcCfg->vccaRailGrpSel =
            (uint8_t)Pmic_getBitField(vccaVmonRailSelRegData,
                                                              PMIC_POWER_TPS6522X_RAIL_SEL_3_VCCA_GRP_SEL_SHIFT,
                                                              PMIC_POWER_TPS6522X_RAIL_SEL_3_VCCA_GRP_SEL_MASK);
    }
}

int32_t Pmic_powerTps6522xGetVccaVmonPwrResourceCfg(Pmic_CoreHandle_t                            *pPmicCoreHandle,
                                                    Pmic_powerTps6522xVccaVmonPowerResourceCfg_t *pVccaVmonPwrRsrcCfg,
                                                    const uint8_t                                 vmonNum)
{
    uint8_t vccaVmonCtrlRegData     = 0;
    uint8_t vccaPgWindowRegData     = 0;
    uint8_t vmonPgWindowRegData     = 0;
    uint8_t vmonPgLevelRegData      = 0;
    uint8_t vccaVmonRailSelRegData  = 0;
    int32_t status                  = PMIC_ST_SUCCESS;

    // Parameter check
    if (pVccaVmonPwrRsrcCfg == NULL)
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }
    if ((status == PMIC_ST_SUCCESS) && (pVccaVmonPwrRsrcCfg->validParams == 0))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    // As we are about to read, start critical section
    Pmic_criticalSectionStart(pPmicCoreHandle);

    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_commIntf_recvByte(
            pPmicCoreHandle, gTps6522xVccaVmonRegisters[vmonNum].vccaVmonCtrlRegAddr, &vccaVmonCtrlRegData);
    }
    if ((status == PMIC_ST_SUCCESS) && (vmonNum == PMIC_POWER_TPS6522X_VOLTAGE_MONITOR_VCCA_VMON))
    {
        status = Pmic_commIntf_recvByte(
            pPmicCoreHandle, gTps6522xVccaVmonRegisters[vmonNum].vccaPgWindowRegAddr, &vccaPgWindowRegData);
    }
    if ((status == PMIC_ST_SUCCESS) && ((vmonNum == PMIC_POWER_TPS6522X_VOLTAGE_MONITOR_VMON1) ||
                                        (vmonNum == PMIC_POWER_TPS6522X_VOLTAGE_MONITOR_VMON2)))
    {
        status = Pmic_commIntf_recvByte(
            pPmicCoreHandle, gTps6522xVccaVmonRegisters[vmonNum].vmonPgWindowRegAddr, &vmonPgWindowRegData);
    }
    if ((status == PMIC_ST_SUCCESS) && ((vmonNum == PMIC_POWER_TPS6522X_VOLTAGE_MONITOR_VMON1) ||
                                        (vmonNum == PMIC_POWER_TPS6522X_VOLTAGE_MONITOR_VMON2)))
    {
        status = Pmic_commIntf_recvByte(
            pPmicCoreHandle, gTps6522xVccaVmonRegisters[vmonNum].vmonPgLevelRegAddr, &vmonPgLevelRegData);
    }
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_commIntf_recvByte(
            pPmicCoreHandle, gTps6522xVccaVmonRegisters[vmonNum].vccaVmonRailSelRegAddr, &vccaVmonRailSelRegData);
    }

    // After read, stop critical section
    Pmic_criticalSectionStop(pPmicCoreHandle);

    // Get the desired bit fields from the registers that were read
    // (desired bit fields are governed by validParams)
    if (status == PMIC_ST_SUCCESS)
    {
        // VCCA_VMON_CTRL
        Pmic_powerTps6522xGetVccaVmonCtrlBitFields(pVccaVmonPwrRsrcCfg, vccaVmonCtrlRegData, vmonNum);

        // VCCA_PG_WINDOW
        Pmic_powerTps6522xGetVccaPgWindowBitFields(pVccaVmonPwrRsrcCfg, vccaPgWindowRegData);

        // VMON_PG_WINDOW
        Pmic_powerTps6522xGetVmonPgWindowBitFields(pVccaVmonPwrRsrcCfg, vmonPgWindowRegData, vmonNum);

        // VMON_PG_LEVEL
        Pmic_powerTps6522xGetVmonPgLevelBitFields(pVccaVmonPwrRsrcCfg, vmonPgLevelRegData, vmonNum);

        // RAIL_SEL_3
        Pmic_powerTps6522xGetRailSel3RegBitFields(pVccaVmonPwrRsrcCfg, vccaVmonRailSelRegData, vmonNum);
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
                status = Pmic_powerTps6522xGetBuckPwrResourceCfg(
                    pPmicCoreHandle, pPwrResourceCfg->buckPwrRsrcCfg + iter, iter);
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
                status = Pmic_powerTps6522xGetLdoPwrResourceCfg(
                    pPmicCoreHandle, pPwrResourceCfg->ldoPwrRsrcCfg + iter, iter);
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

/**
 *  \brief      This function is used to set desired bit fields of the BUCK_CTRL register (desired bit fields
 *              are governed by validParams).
 *
 *  \param      buckPowerResourceCfg    [IN]        BUCK power resource configuration struct
 *  \param      pBuckCtrlRegData        [OUT]       Pointer to BUCK_CTRL register data
 */
static void
Pmic_powerTps6522xSetBuckCtrlRegBitFields(const Pmic_powerTps6522xBuckPowerResourceCfg_t buckPowerResourceCfg,
                                          uint8_t                                       *pBuckCtrlRegData)
{
    if (pmic_validParamCheck(buckPowerResourceCfg.validParams, PMIC_POWER_TPS6522X_CFG_BUCK_PLDN_VALID))
    {
        Pmic_setBitField(pBuckCtrlRegData,
                         PMIC_POWER_TPS6522X_BUCK_CTRL_PLDN_SHIFT,
                         PMIC_POWER_TPS6522X_BUCK_CTRL_PLDN_MASK,
                         (uint8_t)buckPowerResourceCfg.buckPldn);
    }
    if (pmic_validParamCheck(buckPowerResourceCfg.validParams, PMIC_POWER_TPS6522X_CFG_BUCK_VMON_EN_VALID))
    {
        Pmic_setBitField(pBuckCtrlRegData,
                         PMIC_POWER_TPS6522X_BUCK_CTRL_VMON_EN_SHIFT,
                         PMIC_POWER_TPS6522X_BUCK_CTRL_VMON_EN_MASK,
                         (uint8_t)buckPowerResourceCfg.buckVmonEn);
    }
    if (pmic_validParamCheck(buckPowerResourceCfg.validParams, PMIC_POWER_TPS6522X_CFG_BUCK_PWM_OPTION_VALID))
    {
        Pmic_setBitField(pBuckCtrlRegData,
                         PMIC_POWER_TPS6522X_BUCK_CTRL_PWM_OPTION_SHIFT,
                         PMIC_POWER_TPS6522X_BUCK_CTRL_PWM_OPTION_MASK,
                         (uint8_t)buckPowerResourceCfg.buckPwmOption);
    }
    if (pmic_validParamCheck(buckPowerResourceCfg.validParams, PMIC_POWER_TPS6522X_CFG_BUCK_EN_VALID))
    {
        Pmic_setBitField(pBuckCtrlRegData,
                         PMIC_POWER_TPS6522X_BUCK_CTRL_EN_SHIFT,
                         PMIC_POWER_TPS6522X_BUCK_CTRL_EN_MASK,
                         (uint8_t)buckPowerResourceCfg.buckEn);
    }
}

/**
 *  \brief      This function is used to set desired bit fields of the BUCK_CONF register (desired bit fields
 *              are governed by validParams).
 *
 *  \param      buckPowerResourceCfg    [IN]        BUCK power resource configuration struct
 *  \param      pBuckConfRegData        [OUT]       Pointer to BUCK_CONF register data
 */
static void
Pmic_powerTps6522xSetBuckConfRegBitFields(const Pmic_powerTps6522xBuckPowerResourceCfg_t buckPowerResourceCfg,
                                          uint8_t                                       *pBuckConfRegData)
{
    if (pmic_validParamCheck(buckPowerResourceCfg.validParams, PMIC_POWER_TPS6522X_CFG_BUCK_SLEW_RATE_VALID))
    {
        Pmic_setBitField(pBuckConfRegData,
                         PMIC_POWER_TPS6522X_BUCK_CONF_SLEW_RATE_SHIFT,
                         PMIC_POWER_TPS6522X_BUCK_CONF_SLEW_RATE_MASK,
                         (uint8_t)buckPowerResourceCfg.buckSlewRate);
    }
}

/**
 *  \brief      This function is used to convert a BUCK voltage value (mV) to BUCK VSET value.
 *
 *  \param      pBuckVoltage_vset   [OUT]       BUCK voltage in VSET form
 *  \param      buckVoltage_mv      [IN]        BUCK voltage in millivolts
 *  \param      buckNum             [IN]        Indicates which BUCK the API is working with
 */
static void Pmic_powerTps6522xBuckConvertVoltage2VsetVal(uint8_t       *pBuckVoltage_vset,
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
        *pBuckVoltage_vset = baseVoltage_vset + ((buckVoltage_mv - baseVoltage_mv) / voltageStep);
    }
}

/**
 *  \brief      This function calculates a VSET value given a milivolt value (obtained from BUCK power resource CFG)
 *              and sets the BUCK_VOUT register data equal to the VSET value (if validParam is set).
 *
 *  \param      buckPowerResourceCfg        [IN]        BUCK power resource configuration struct
 *  \param      pBuckVoutRegData            [OUT]       Pointer to BUCK_VOUT register data
 *  \param      buckNum                     [IN]        Indicates which BUCK the API is working with
 */
static void
Pmic_powerTps6522xSetBuckVoutRegBitFields(const Pmic_powerTps6522xBuckPowerResourceCfg_t buckPowerResourceCfg,
                                          uint8_t                                       *pBuckVoutRegData,
                                          const uint8_t                                  buckNum)
{
    if (pmic_validParamCheck(buckPowerResourceCfg.validParams, PMIC_POWER_TPS6522X_CFG_BUCK_VOLTAGE_MV_VALID))
    {
        Pmic_powerTps6522xBuckConvertVoltage2VsetVal(pBuckVoutRegData, buckPowerResourceCfg.buckVoltage_mv, buckNum);
    }
}

/**
 *  \brief      This function is used to set desired bit fields of the BUCK_PG_WINDOW register (desired
 *              bit fields are governed by validParams).
 *
 *  \param      buckPowerResourceCfg    [IN]        BUCK power resource configuration struct
 *  \param      pBuckPgWindowRegData    [OUT]       Pointer to BUCK_PG_WINDOW register data
 */
static void
Pmic_powerTps6522xSetBuckPgWindowRegBitFields(const Pmic_powerTps6522xBuckPowerResourceCfg_t buckPowerResourceCfg,
                                              uint8_t                                       *pBuckPgWindowRegData)
{
    if (pmic_validParamCheck(buckPowerResourceCfg.validParams, PMIC_POWER_TPS6522X_CFG_BUCK_VMON_THR_VALID))
    {
        Pmic_setBitField(pBuckPgWindowRegData,
                         PMIC_POWER_TPS6522X_BUCK_PG_WINDOW_VMON_THR_SHIFT,
                         PMIC_POWER_TPS6522X_BUCK_PG_WINDOW_VMON_THR_MASK,
                         (uint8_t)buckPowerResourceCfg.buckVmonThr);
    }
}

/**
 *  \brief      This function is used to set the desired bit fields of the RAIL_SEL_1 register (desired
 *              bit fields are governed by validParams).
 *
 *  \param      buckPowerResourceCfg    [IN]        BUCK power resource configuration struct
 *  \param      pBuckRailSelRegData     [OUT]       Pointer to RAIL_SEL_1 register data
 *  \param      buckNum                 [IN]        Indicates which BUCK the API is working with
 */
static void
Pmic_powerTps6522xSetRailSel1RegBitFields(const Pmic_powerTps6522xBuckPowerResourceCfg_t buckPowerResourceCfg,
                                          uint8_t                                       *pBuckRailSelRegData,
                                          const uint8_t                                  buckNum)
{
    if (pmic_validParamCheck(buckPowerResourceCfg.validParams, PMIC_POWER_TPS6522X_CFG_BUCK_RAIL_GRP_SEL_VALID))
    {
        switch (buckNum)
        {
            case PMIC_POWER_TPS6522X_REGULATOR_BUCK1:
                Pmic_setBitField(pBuckRailSelRegData,
                                 PMIC_POWER_TPS6522X_CFG_RAIL_SEL_1_BUCK1_GRP_SEL_SHIFT,
                                 PMIC_POWER_TPS6522X_CFG_RAIL_SEL_1_BUCK1_GRP_SEL_MASK,
                                 (uint8_t)buckPowerResourceCfg.buckRailGrpSel);

                break;
            case PMIC_POWER_TPS6522X_REGULATOR_BUCK2:
                Pmic_setBitField(pBuckRailSelRegData,
                                 PMIC_POWER_TPS6522X_CFG_RAIL_SEL_1_BUCK2_GRP_SEL_SHIFT,
                                 PMIC_POWER_TPS6522X_CFG_RAIL_SEL_1_BUCK2_GRP_SEL_MASK,
                                 (uint8_t)buckPowerResourceCfg.buckRailGrpSel);

                break;
            case PMIC_POWER_TPS6522X_REGULATOR_BUCK3:
                Pmic_setBitField(pBuckRailSelRegData,
                                 PMIC_POWER_TPS6522X_CFG_RAIL_SEL_1_BUCK3_GRP_SEL_SHIFT,
                                 PMIC_POWER_TPS6522X_CFG_RAIL_SEL_1_BUCK3_GRP_SEL_MASK,
                                 (uint8_t)buckPowerResourceCfg.buckRailGrpSel);

                break;
            case PMIC_POWER_TPS6522X_REGULATOR_BUCK4:
                Pmic_setBitField(pBuckRailSelRegData,
                                 PMIC_POWER_TPS6522X_CFG_RAIL_SEL_1_BUCK4_GRP_SEL_SHIFT,
                                 PMIC_POWER_TPS6522X_CFG_RAIL_SEL_1_BUCK4_GRP_SEL_MASK,
                                 (uint8_t)buckPowerResourceCfg.buckRailGrpSel);

                break;
        }
    }
}

int32_t Pmic_powerTps6522xSetBuckPwrResourceCfg(Pmic_CoreHandle_t                             *pPmicCoreHandle,
                                                const Pmic_powerTps6522xBuckPowerResourceCfg_t buckPowerResourceCfg,
                                                const uint8_t                                  buckNum)
{
    uint8_t buckCtrlRegData     = 0;
    uint8_t buckConfRegData     = 0;
    uint8_t buckVoutRegData     = 0;
    uint8_t buckPgWindowRegData = 0;
    uint8_t buckRailSelRegData  = 0;
    int32_t status              = PMIC_ST_SUCCESS;

    // Parameter check
    if (buckPowerResourceCfg.validParams == 0)
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    // As we are about to read, start critical section
    Pmic_criticalSectionStart(pPmicCoreHandle);

    // Read all registers pertaining to the BUCK resource
    if (status == PMIC_ST_SUCCESS)
    {
        status =
            Pmic_commIntf_recvByte(pPmicCoreHandle, gTps6522xBuckRegisters[buckNum].buckCtrlRegAddr, &buckCtrlRegData);
    }
    if (status == PMIC_ST_SUCCESS)
    {
        status =
            Pmic_commIntf_recvByte(pPmicCoreHandle, gTps6522xBuckRegisters[buckNum].buckConfRegAddr, &buckConfRegData);
    }
    if (status == PMIC_ST_SUCCESS)
    {
        status =
            Pmic_commIntf_recvByte(pPmicCoreHandle, gTps6522xBuckRegisters[buckNum].buckVoutRegAddr, &buckVoutRegData);
    }
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_commIntf_recvByte(
            pPmicCoreHandle, gTps6522xBuckRegisters[buckNum].buckPgWindowRegAddr, &buckPgWindowRegData);
    }
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_commIntf_recvByte(
            pPmicCoreHandle, gTps6522xBuckRegisters[buckNum].buckRailSelRegAddr, &buckRailSelRegData);
    }

    // modify the desired bit fields of the registers that were read
    // (desired bit fields are governed by validParams)
    if (status == PMIC_ST_SUCCESS)
    {
        // BUCK_CTRL register
        Pmic_powerTps6522xSetBuckCtrlRegBitFields(buckPowerResourceCfg, &buckCtrlRegData);

        // BUCK_CONF register
        Pmic_powerTps6522xSetBuckConfRegBitFields(buckPowerResourceCfg, &buckConfRegData);

        // BUCK_VOUT register
        Pmic_powerTps6522xSetBuckVoutRegBitFields(buckPowerResourceCfg, &buckVoutRegData, buckNum);

        // BUCK_PG_WINDOW register
        Pmic_powerTps6522xSetBuckPgWindowRegBitFields(buckPowerResourceCfg, &buckPgWindowRegData);

        // RAIL_SEL_1 register
        Pmic_powerTps6522xSetRailSel1RegBitFields(buckPowerResourceCfg, &buckRailSelRegData, buckNum);
    }

    // After modification of register bit fields, write values back to PMIC
    if (status == PMIC_ST_SUCCESS)
    {
        status =
            Pmic_commIntf_sendByte(pPmicCoreHandle, gTps6522xBuckRegisters[buckNum].buckConfRegAddr, buckConfRegData);
    }
    if (status == PMIC_ST_SUCCESS)
    {
        status =
            Pmic_commIntf_sendByte(pPmicCoreHandle, gTps6522xBuckRegisters[buckNum].buckVoutRegAddr, buckVoutRegData);
    }
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_commIntf_sendByte(
            pPmicCoreHandle, gTps6522xBuckRegisters[buckNum].buckPgWindowRegAddr, buckPgWindowRegData);
    }
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_commIntf_sendByte(
            pPmicCoreHandle, gTps6522xBuckRegisters[buckNum].buckRailSelRegAddr, buckRailSelRegData);
    }
    if (status == PMIC_ST_SUCCESS)
    {
        status =
            Pmic_commIntf_sendByte(pPmicCoreHandle, gTps6522xBuckRegisters[buckNum].buckCtrlRegAddr, buckCtrlRegData);
    }

    // After read-modify-write, stop critical section
    Pmic_criticalSectionStop(pPmicCoreHandle);

    return status;
}

/**
 *  \brief      This function is used to set the desired bit fields of the LDO_CTRL register
 *              (desired bit fields are governed by validParams).
 *
 *  \param      ldoPowerResourceCfg     [IN]        LDO power resource configuration struct
 *  \param      pLdoCtrlRegData         [OUT]       Pointer to LDO_CTRL register data
 */
static void Pmic_powerTps6522xSetLdoCtrlRegBitFields(const Pmic_powerTps6522xLdoPowerResourceCfg_t ldoPowerResourceCfg,
                                                     uint8_t                                      *pLdoCtrlRegData)
{
    if (pmic_validParamCheck(ldoPowerResourceCfg.validParams, PMIC_POWER_TPS6522X_CFG_LDO_DISCHARGE_EN_VALID))
    {
        Pmic_setBitField(pLdoCtrlRegData,
                         PMIC_POWER_TPS6522X_LDO_CTRL_DISCHARGE_EN_SHIFT,
                         PMIC_POWER_TPS6522X_LDO_CTRL_DISCHARGE_EN_MASK,
                         (uint8_t)ldoPowerResourceCfg.ldoDischargeEn);
    }
    if (pmic_validParamCheck(ldoPowerResourceCfg.validParams, PMIC_POWER_TPS6522X_CFG_LDO_VMON_EN_VALID))
    {
        Pmic_setBitField(pLdoCtrlRegData,
                         PMIC_POWER_TPS6522X_LDO_CTRL_VMON_EN_SHIFT,
                         PMIC_POWER_TPS6522X_LDO_CTRL_VMON_EN_MASK,
                         (uint8_t)ldoPowerResourceCfg.ldoVmonEn);
    }
    if (pmic_validParamCheck(ldoPowerResourceCfg.validParams, PMIC_POWER_TPS6522X_CFG_LDO_EN_VALID))
    {
        Pmic_setBitField(pLdoCtrlRegData,
                         PMIC_POWER_TPS6522X_LDO_CTRL_EN_SHIFT,
                         PMIC_POWER_TPS6522X_LDO_CTRL_EN_MASK,
                         (uint8_t)ldoPowerResourceCfg.ldoEn);
    }
}

/**
 *  \brief      This function is used to convert LDO voltage (mV) to LDO VSET value.
 *
 *  \param      pLdoVoltage_vset    [OUT]       LDO voltage value in VSET form
 *  \param      ldoVoltage_mv       [IN]        LDO voltage value in millivolts
 *  \param      ldoNum              [IN]        Indicates which LDO the API is working with
 */
static void Pmic_powerTps6522xLdoConvertVoltage2VsetVal(uint8_t       *pLdoVoltage_vset,
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
        *pLdoVoltage_vset = baseVoltage_vset + ((ldoVoltage_mv - baseVoltage_mv) / voltageStep);
    }
}

/**
 *  \brief      This function is used to set the desired bit fields of the LDO_VOUT register
 *              (desired bit fields are governed by validParams).
 *
 *  \param      ldoPowerResourceCfg     [IN]        LDO power resource configuration struct
 *  \param      pLdoVoutRegData         [OUT]       Pointer to LDO_VOUT register data
 *  \param      ldoNum                  [IN]        Indicates which LDO the API is working with
 */
static void Pmic_powerTps6522xSetLdoVoutRegBitFields(const Pmic_powerTps6522xLdoPowerResourceCfg_t ldoPowerResourceCfg,
                                                     uint8_t                                      *pLdoVoutRegData,
                                                     const uint8_t                                 ldoNum)
{
    uint8_t ldoVoltage_vset = 0;

    if (pmic_validParamCheck(ldoPowerResourceCfg.validParams, PMIC_POWER_TPS6522X_CFG_LDO_MODE_VALID))
    {
        Pmic_setBitField(pLdoVoutRegData,
                         PMIC_POWER_TPS6522X_LDO_VOUT_LDO_MODE_SHIFT,
                         PMIC_POWER_TPS6522X_LDO_VOUT_LDO_MODE_MASK,
                         (uint8_t)ldoPowerResourceCfg.ldoMode);
    }
    if (pmic_validParamCheck(ldoPowerResourceCfg.validParams, PMIC_POWER_TPS6522X_CFG_LDO_VOLTAGE_MV_VALID))
    {
        Pmic_powerTps6522xLdoConvertVoltage2VsetVal(&ldoVoltage_vset, ldoPowerResourceCfg.ldoVoltage_mv, ldoNum);

        Pmic_setBitField(pLdoVoutRegData,
                         PMIC_POWER_TPS6522X_LDO_VOUT_LDO_VSET_SHIFT,
                         PMIC_POWER_TPS6522X_LDO_VOUT_LDO_VSET_MASK,
                         ldoVoltage_vset);
    }
}

/**
 *  \brief      This function is used to set the desired bit fields of the LDO_PG_WINDOW register
 *              (desired bit fields are governed by validParams).
 *
 *  \param      ldoPowerResourceCfg     [IN]        LDO power resource configuration struct
 *  \param      pLdoPgWindowRegData     [OUT]       Pointer to LDO_PG_WINDOW register data
 */
static void
Pmic_powerTps6522xSetLdoPgWindowRegBitFields(const Pmic_powerTps6522xLdoPowerResourceCfg_t ldoPowerResourceCfg,
                                             uint8_t                                      *pLdoPgWindowRegData)
{
    if (pmic_validParamCheck(ldoPowerResourceCfg.validParams, PMIC_POWER_TPS6522X_CFG_LDO_VMON_THR_VALID))
    {
        Pmic_setBitField(pLdoPgWindowRegData,
                         PMIC_POWER_TPS6522X_LDO_PG_WINDOW_LDO_VMON_THR_SHIFT,
                         PMIC_POWER_TPS6522X_LDO_PG_WINDOW_LDO_VMON_THR_MASK,
                         (uint8_t)ldoPowerResourceCfg.ldoVmonThr);
    }
}

/**
 *  \brief      This function is used to set the desired bit fields of the RAIL_SEL_2 register
 *              (desired bit fields are governed by validParams).
 *
 *  \param      ldoPowerResourceCfg     [IN]        LDO power resource configuration struct
 *  \param      pLdoRailSelRegData      [OUT]       Pointer to RAIL_SEL_2 register data
 *  \param      ldoNum                  [IN]        Indicates which LDO the API is working with
 */
static void Pmic_powerTps6522xSetRailSel2RegBitFields(const Pmic_powerTps6522xLdoPowerResourceCfg_t ldoPowerResourceCfg,
                                                      uint8_t                                      *pLdoRailSelRegData,
                                                      uint8_t                                       ldoNum)
{
    if (pmic_validParamCheck(ldoPowerResourceCfg.validParams, PMIC_POWER_TPS6522X_CFG_LDO_RAIL_GRP_SEL_VALID))
    {
        switch (ldoNum)
        {
            case PMIC_POWER_TPS6522X_REGULATOR_LDO1:
                Pmic_setBitField(pLdoRailSelRegData,
                                 PMIC_POWER_TPS6522X_RAIL_SEL_2_LDO1_GRP_SEL_SHIFT,
                                 PMIC_POWER_TPS6522X_RAIL_SEL_2_LDO1_GRP_SEL_MASK,
                                 (uint8_t)ldoPowerResourceCfg.ldoRailGrpSel);

                break;
            case PMIC_POWER_TPS6522X_REGULATOR_LDO2:
                Pmic_setBitField(pLdoRailSelRegData,
                                 PMIC_POWER_TPS6522X_RAIL_SEL_2_LDO2_GRP_SEL_SHIFT,
                                 PMIC_POWER_TPS6522X_RAIL_SEL_2_LDO2_GRP_SEL_MASK,
                                 (uint8_t)ldoPowerResourceCfg.ldoRailGrpSel);

                break;
            case PMIC_POWER_TPS6522X_REGULATOR_LDO3:
                Pmic_setBitField(pLdoRailSelRegData,
                                 PMIC_POWER_TPS6522X_RAIL_SEL_2_LDO3_GRP_SEL_SHIFT,
                                 PMIC_POWER_TPS6522X_RAIL_SEL_2_LDO3_GRP_SEL_MASK,
                                 (uint8_t)ldoPowerResourceCfg.ldoRailGrpSel);

                break;
        }
    }
}

int32_t Pmic_powerTps6522xSetLdoPwrResourceCfg(Pmic_CoreHandle_t                            *pPmicCoreHandle,
                                               const Pmic_powerTps6522xLdoPowerResourceCfg_t ldoPowerResourceCfg,
                                               const uint8_t                                 ldoNum)
{
    uint8_t ldoCtrlRegData      = 0;
    uint8_t ldoVoutRegData      = 0;
    uint8_t ldoPgWindowRegData  = 0;
    uint8_t ldoRailSelRegData   = 0;
    int32_t status              = PMIC_ST_SUCCESS;

    // Parameter check
    if (ldoPowerResourceCfg.validParams == 0)
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    // As we are about to read, start critical section
    Pmic_criticalSectionStart(pPmicCoreHandle);

    // Read all registers pertaining to the LDO resource
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_commIntf_recvByte(pPmicCoreHandle, gTps6522xLdoRegisters[ldoNum].ldoCtrlRegAddr, &ldoCtrlRegData);
    }
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_commIntf_recvByte(pPmicCoreHandle, gTps6522xLdoRegisters[ldoNum].ldoVoutRegAddr, &ldoVoutRegData);
    }
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_commIntf_recvByte(
            pPmicCoreHandle, gTps6522xLdoRegisters[ldoNum].ldoPgWindowRegAddr, &ldoPgWindowRegData);
    }
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_commIntf_recvByte(
            pPmicCoreHandle, gTps6522xLdoRegisters[ldoNum].ldoRailSelRegAddr, &ldoRailSelRegData);
    }

    // modify the desired bit fields of the registers that were read
    // (desired bit fields are governed by validParams)
    if (status == PMIC_ST_SUCCESS)
    {
        // LDO_CTRL register
        Pmic_powerTps6522xSetLdoCtrlRegBitFields(ldoPowerResourceCfg, &ldoCtrlRegData);

        // LDO_VOUT register
        Pmic_powerTps6522xSetLdoVoutRegBitFields(ldoPowerResourceCfg, &ldoVoutRegData, ldoNum);

        // LDO_PG_WINDOW register
        Pmic_powerTps6522xSetLdoPgWindowRegBitFields(ldoPowerResourceCfg, &ldoPgWindowRegData);

        // RAIL_SEL_2 register
        Pmic_powerTps6522xSetRailSel2RegBitFields(ldoPowerResourceCfg, &ldoRailSelRegData, ldoNum);
    }

    // After modification of register bit fields, write values back to PMIC
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_commIntf_sendByte(pPmicCoreHandle, gTps6522xLdoRegisters[ldoNum].ldoVoutRegAddr, ldoVoutRegData);
    }
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_commIntf_sendByte(
            pPmicCoreHandle, gTps6522xLdoRegisters[ldoNum].ldoPgWindowRegAddr, ldoPgWindowRegData);
    }
    if (status == PMIC_ST_SUCCESS)
    {
        status =
            Pmic_commIntf_sendByte(pPmicCoreHandle, gTps6522xLdoRegisters[ldoNum].ldoRailSelRegAddr, ldoRailSelRegData);
    }
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_commIntf_sendByte(pPmicCoreHandle, gTps6522xLdoRegisters[ldoNum].ldoCtrlRegAddr, ldoCtrlRegData);
    }

    // After read-modify-write, stop critical section
    Pmic_criticalSectionStop(pPmicCoreHandle);

    return status;
}

/**
 *  \brief      This function is used to set the desired bit fields of the VCCA_VMON_CTRL register
 *              (desired bit fields are governed by validParams).
 *
 *  \param      vccaVmonPwrRsrcCfg      [IN]        VCCA/VMON power resource configuration struct
 *  \param      pVccaVmonCtrlRegData    [OUT]       Pointer to VCCA_VMON_CTRL register data
 *  \param      vmonNum                 [IN]        Indicates which VCCA/VMONx the API is working with
 */
static void
Pmic_powerTps6522xSetVccaVmonCtrlBitFields(const Pmic_powerTps6522xVccaVmonPowerResourceCfg_t vccaVmonPwrRsrcCfg,
                                           uint8_t                                           *pVccaVmonCtrlRegData,
                                           const uint8_t                                      vmonNum)
{
    if (pmic_validParamCheck(vccaVmonPwrRsrcCfg.validParams, PMIC_POWER_TPS6522X_CFG_VMON_DEGLITCH_SEL_VALID))
    {
        Pmic_setBitField(pVccaVmonCtrlRegData,
                         PMIC_POWER_TPS6522X_VCCA_VMON_CTRL_DEGLITCH_SEL_SHIFT,
                         PMIC_POWER_TPS6522X_VCCA_VMON_CTRL_DEGLITCH_SEL_MASK,
                         (uint8_t)vccaVmonPwrRsrcCfg.vmonDeglitchSel);
    }
    if (pmic_validParamCheck(vccaVmonPwrRsrcCfg.validParams, PMIC_POWER_TPS6522X_CFG_VMON2_EN_VALID))
    {
        Pmic_setBitField(pVccaVmonCtrlRegData,
                         PMIC_POWER_TPS6522X_VCCA_VMON_CTRL_VMON2_EN_SHIFT,
                         PMIC_POWER_TPS6522X_VCCA_VMON_CTRL_VMON2_EN_MASK,
                         (uint8_t)vccaVmonPwrRsrcCfg.vmon2En);
    }
    if (pmic_validParamCheck(vccaVmonPwrRsrcCfg.validParams, PMIC_POWER_TPS6522X_CFG_VMON1_EN_VALID))
    {
        Pmic_setBitField(pVccaVmonCtrlRegData,
                         PMIC_POWER_TPS6522X_VCCA_VMON_CTRL_VMON1_EN_SHIFT,
                         PMIC_POWER_TPS6522X_VCCA_VMON_CTRL_VMON1_EN_MASK,
                         (uint8_t)vccaVmonPwrRsrcCfg.vmon1En);
    }
    if (pmic_validParamCheck(vccaVmonPwrRsrcCfg.validParams, PMIC_POWER_TPS6522X_CFG_VCCA_VMON_EN_VALID))
    {
        Pmic_setBitField(pVccaVmonCtrlRegData,
                         PMIC_POWER_TPS6522X_VCCA_VMON_CTRL_VCCA_VMON_EN_SHIFT,
                         PMIC_POWER_TPS6522X_VCCA_VMON_CTRL_VCCA_VMON_EN_MASK,
                         (uint8_t)vccaVmonPwrRsrcCfg.vccaVmonEn);
    }
}

/**
 *  \brief      This function is used to set desired bit fields of the VCCA_PG_WINDOW register
 *              (desired bit fields are governed by validParams).
 *
 *  \param      vccaVmonPwrRsrcCfg      [IN]        VCCA/VMON power resource configuration struct
 *  \param      pVccaPgWindowRegData    [OUT]       Pointer to VCCA_PG_WINDOW register data
 */
static void
Pmic_powerTps6522xSetVccaPgWindowBitFields(const Pmic_powerTps6522xVccaVmonPowerResourceCfg_t vccaVmonPwrRsrcCfg,
                                           uint8_t                                           *pVccaPgWindowRegData)
{
    if (pmic_validParamCheck(vccaVmonPwrRsrcCfg.validParams, PMIC_POWER_TPS6522X_CFG_VCCA_PG_LEVEL_VALID))
    {
        Pmic_setBitField(pVccaPgWindowRegData,
                         PMIC_POWER_TPS6522X_VCCA_PG_WINDOW_VCCA_PG_SET_SHIFT,
                         PMIC_POWER_TPS6522X_VCCA_PG_WINDOW_VCCA_PG_SET_MASK,
                         (uint8_t)vccaVmonPwrRsrcCfg.vccaPgLevel);
    }
    if (pmic_validParamCheck(vccaVmonPwrRsrcCfg.validParams, PMIC_POWER_TPS6522X_CFG_VCCA_VMON_THR_VALID))
    {
        Pmic_setBitField(pVccaPgWindowRegData,
                         PMIC_POWER_TPS6522X_VCCA_PG_WINDOW_VCCA_VMON_THR_SHIFT,
                         PMIC_POWER_TPS6522X_VCCA_PG_WINDOW_VCCA_VMON_THR_MASK,
                         (uint8_t)vccaVmonPwrRsrcCfg.vccaVmonThr);
    }
}

/**
 *  \brief      This function is used to set desired bit fields of the VMON_PG_WINDOW register
 *              (desired bit fields are governed by validParams).
 *
 *  \param      vccaVmonPwrRsrcCfg      [IN]        VCCA/VMON power resource configuration struct
 *  \param      pVmonPgWindowRegData    [OUT]       Pointer to VMON_PG_WINDOW register data
 *  \param      vmonNum                 [IN]        Indicates which VCCA/VMONx the API is working with
 */
static void
Pmic_powerTps6522xSetVmonPgWindowBitFields(const Pmic_powerTps6522xVccaVmonPowerResourceCfg_t vccaVmonPwrRsrcCfg,
                                           uint8_t                                           *pVmonPgWindowRegData,
                                           const uint8_t                                      vmonNum)
{
    if ((vmonNum == PMIC_POWER_TPS6522X_VOLTAGE_MONITOR_VMON1) &&
        pmic_validParamCheck(vccaVmonPwrRsrcCfg.validParams, PMIC_POWER_TPS6522X_CFG_VMON1_THR_VALID))
    {
        Pmic_setBitField(pVmonPgWindowRegData,
                         PMIC_POWER_TPS6522X_VMON1_PG_WINDOW_VMON1_THR_SHIFT,
                         PMIC_POWER_TPS6522X_VMON1_PG_WINDOW_VMON1_THR_MASK,
                         (uint8_t)vccaVmonPwrRsrcCfg.vmon1Thr);
    }
    else if ((vmonNum == PMIC_POWER_TPS6522X_VOLTAGE_MONITOR_VMON2) &&
             pmic_validParamCheck(vccaVmonPwrRsrcCfg.validParams, PMIC_POWER_TPS6522X_CFG_VMON2_THR_VALID))
    {
        Pmic_setBitField(pVmonPgWindowRegData,
                         PMIC_POWER_TPS6522X_VMON2_PG_WINDOW_VMON2_THR_SHIFT,
                         PMIC_POWER_TPS6522X_VMON2_PG_WINDOW_VMON2_THR_MASK,
                         (uint8_t)vccaVmonPwrRsrcCfg.vmon2Thr);
    }
}

/**
 *  \brief      This function is used to convert a VMON voltage value (mV) to VMON PG_SET value.
 *
 *  \param      pVmonPgLevel_pgSet      [OUT]       VMON PG level in PG_SET form
 *  \param      vmonPgLevel_mv          [IN]        VMON PG level in millivolts
 *  \param      vmonNum                 [IN]        Indicates which VCCA/VMONx the API is working with
 */
static void Pmic_powerTps6522xVmonConvertVoltage2PgSet(uint8_t       *pVmonPgLevel_pgSet,
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
        *pVmonPgLevel_pgSet = baseVoltage_vset + ((vmonPgLevel_mv - baseVoltage_mv) / voltageStep);
    }
}

/**
 *  \brief      This function is used to set the desired bit fields of the VMON_PG_LEVEL register.
 *              It converts a millivolt value obtained from the VCCA/VMON power resource CFG struct
 *              to PG_SET and sets the VMON_PG_LEVEL register equal to the PG_SET value.
 *
 *  \param      vccaVmonPwrRsrcCfg      [IN]        VCCA/VMON power resource configuration register
 *  \param      pVmonPgLevelRegData     [OUT]       Pointer to VMON_PG_LEVEL register data
 *  \param      vmonNum                 [IN]        Indicates which VCCA/VMONx the API is working with
 */
static void
Pmic_powerTps6522xSetVmonPgLevelBitFields(const Pmic_powerTps6522xVccaVmonPowerResourceCfg_t vccaVmonPwrRsrcCfg,
                                          uint8_t                                           *pVmonPgLevelRegData,
                                          const uint8_t                                      vmonNum)
{
    if ((vmonNum == PMIC_POWER_TPS6522X_VOLTAGE_MONITOR_VMON1) &&
        pmic_validParamCheck(vccaVmonPwrRsrcCfg.validParams, PMIC_POWER_TPS6522X_CFG_VMON1_PG_LEVEL_MV_VALID))
    {
        Pmic_powerTps6522xVmonConvertVoltage2PgSet(pVmonPgLevelRegData, vccaVmonPwrRsrcCfg.vmon1PgLevel_mv, vmonNum);
    }
    else if ((vmonNum == PMIC_POWER_TPS6522X_VOLTAGE_MONITOR_VMON2) &&
             pmic_validParamCheck(vccaVmonPwrRsrcCfg.validParams, PMIC_POWER_TPS6522X_CFG_VMON2_PG_LEVEL_MV_VALID))
    {
        Pmic_powerTps6522xVmonConvertVoltage2PgSet(pVmonPgLevelRegData, vccaVmonPwrRsrcCfg.vmon2PgLevel_mv, vmonNum);
    }
}

/**
 *  \brief      This function is used to set the desired bit fields of the RAIL_SEL_3 register
 *              (desired bit fields are governed by validParams).
 *
 *  \param      vccaVmonPwrRsrcCfg          [IN]        VCCA/VMON power resource configuration struct
 *  \param      pVccaVmonRailSelRegData     [OUT]       Pointer to RAIL_SEL_3 register
 *  \param      vmonNum                     [IN]        Indicates which VCCA/VMONx the API is working with
 */
static void
Pmic_powerTps6522xSetRailSel3RegBitFields(const Pmic_powerTps6522xVccaVmonPowerResourceCfg_t vccaVmonPwrRsrcCfg,
                                          uint8_t                                           *pVccaVmonRailSelRegData,
                                          const uint8_t                                      vmonNum)
{
    if ((vmonNum == PMIC_POWER_TPS6522X_VOLTAGE_MONITOR_VMON1) &&
        pmic_validParamCheck(vccaVmonPwrRsrcCfg.validParams, PMIC_POWER_TPS6522X_CFG_VMON1_RAIL_GRP_SEL_VALID))
    {
        Pmic_setBitField(pVccaVmonRailSelRegData,
                         PMIC_POWER_TPS6522X_RAIL_SEL_3_VMON1_GRP_SEL_SHIFT,
                         PMIC_POWER_TPS6522X_RAIL_SEL_3_VMON1_GRP_SEL_MASK,
                         (uint8_t)vccaVmonPwrRsrcCfg.vmon1RailGrpSel);
    }
    else if ((vmonNum == PMIC_POWER_TPS6522X_VOLTAGE_MONITOR_VMON2) &&
             pmic_validParamCheck(vccaVmonPwrRsrcCfg.validParams, PMIC_POWER_TPS6522X_CFG_VMON2_RAIL_GRP_SEL_VALID))
    {
        Pmic_setBitField(pVccaVmonRailSelRegData,
                         PMIC_POWER_TPS6522X_RAIL_SEL_3_VMON2_GRP_SEL_SHIFT,
                         PMIC_POWER_TPS6522X_RAIL_SEL_3_VMON2_GRP_SEL_MASK,
                         (uint8_t)vccaVmonPwrRsrcCfg.vmon2RailGrpSel);
    }
    else if ((vmonNum == PMIC_POWER_TPS6522X_VOLTAGE_MONITOR_VCCA_VMON) &&
             pmic_validParamCheck(vccaVmonPwrRsrcCfg.validParams, PMIC_POWER_TPS6522X_CFG_VCCA_RAIL_GRP_SEL_VALID))
    {
        Pmic_setBitField(pVccaVmonRailSelRegData,
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
    uint8_t vccaVmonCtrlRegData     = 0;
    uint8_t vccaPgWindowRegData     = 0;
    uint8_t vmonPgWindowRegData     = 0;
    uint8_t vmonPgLevelRegData      = 0;
    uint8_t vccaVmonRailSelRegData  = 0;
    int32_t status                  = PMIC_ST_SUCCESS;

    // Parameter check
    if (vccaVmonPwrRsrcCfg.validParams == 0)
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    // As we are about to read, start critical section
    Pmic_criticalSectionStart(pPmicCoreHandle);

    // Read all registers pertaining to the VCCA_VMON/VMONx resource
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_commIntf_recvByte(
            pPmicCoreHandle, gTps6522xVccaVmonRegisters[vmonNum].vccaVmonCtrlRegAddr, &vccaVmonCtrlRegData);
    }
    if ((status == PMIC_ST_SUCCESS) && (vmonNum == PMIC_POWER_TPS6522X_VOLTAGE_MONITOR_VCCA_VMON))
    {
        status = Pmic_commIntf_recvByte(
            pPmicCoreHandle, gTps6522xVccaVmonRegisters[vmonNum].vccaPgWindowRegAddr, &vccaPgWindowRegData);
    }
    if ((status == PMIC_ST_SUCCESS) && ((vmonNum == PMIC_POWER_TPS6522X_VOLTAGE_MONITOR_VMON1) ||
                                        (vmonNum == PMIC_POWER_TPS6522X_VOLTAGE_MONITOR_VMON2)))
    {
        status = Pmic_commIntf_recvByte(
            pPmicCoreHandle, gTps6522xVccaVmonRegisters[vmonNum].vmonPgWindowRegAddr, &vmonPgWindowRegData);
    }
    if ((status == PMIC_ST_SUCCESS) && ((vmonNum == PMIC_POWER_TPS6522X_VOLTAGE_MONITOR_VMON1) ||
                                        (vmonNum == PMIC_POWER_TPS6522X_VOLTAGE_MONITOR_VMON2)))
    {
        status = Pmic_commIntf_recvByte(
            pPmicCoreHandle, gTps6522xVccaVmonRegisters[vmonNum].vmonPgLevelRegAddr, &vmonPgLevelRegData);
    }
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_commIntf_recvByte(
            pPmicCoreHandle, gTps6522xVccaVmonRegisters[vmonNum].vccaVmonRailSelRegAddr, &vccaVmonRailSelRegData);
    }

    // modify the desired bit fields of the registers that were read
    // of the register (desired bit fields are governed by validParams)
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
            pPmicCoreHandle, gTps6522xVccaVmonRegisters[vmonNum].vccaPgWindowRegAddr, vccaPgWindowRegData);
    }
    if ((status == PMIC_ST_SUCCESS) && ((vmonNum == PMIC_POWER_TPS6522X_VOLTAGE_MONITOR_VMON1) ||
                                        (vmonNum == PMIC_POWER_TPS6522X_VOLTAGE_MONITOR_VMON2)))
    {
        status = Pmic_commIntf_sendByte(
            pPmicCoreHandle, gTps6522xVccaVmonRegisters[vmonNum].vmonPgWindowRegAddr, vmonPgWindowRegData);
    }
    if ((status == PMIC_ST_SUCCESS) && ((vmonNum == PMIC_POWER_TPS6522X_VOLTAGE_MONITOR_VMON1) ||
                                        (vmonNum == PMIC_POWER_TPS6522X_VOLTAGE_MONITOR_VMON2)))
    {
        status = Pmic_commIntf_sendByte(
            pPmicCoreHandle, gTps6522xVccaVmonRegisters[vmonNum].vmonPgLevelRegAddr, vmonPgLevelRegData);
    }
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_commIntf_sendByte(
            pPmicCoreHandle, gTps6522xVccaVmonRegisters[vmonNum].vccaVmonRailSelRegAddr, vccaVmonRailSelRegData);
    }
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_commIntf_sendByte(
            pPmicCoreHandle, gTps6522xVccaVmonRegisters[vmonNum].vccaVmonCtrlRegAddr, vccaVmonCtrlRegData);
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
                status =
                    Pmic_powerTps6522xSetBuckPwrResourceCfg(pPmicCoreHandle, pwrResourceCfg.buckPwrRsrcCfg[iter], iter);
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
                status =
                    Pmic_powerTps6522xSetLdoPwrResourceCfg(pPmicCoreHandle, pwrResourceCfg.ldoPwrRsrcCfg[iter], iter);
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

/**
 * \brief       This function is used to check whether the PMIC handle is valid.
 *
 * \param       pPmicCoreHandle     [IN]    PMIC interface handle
 *
 * \return      Success code if PMIC handle is valid, error code otherwise.
 *              For valid success/error codes, refer to \ref Pmic_ErrorCodes
 */
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

/**
 *  \brief      This function is used to get the desired BUCK UVOV status.
 *
 *  \param      pPmicCoreHandle         [IN]        PMIC interface handle
 *  \param      pwrRsrcUVOVStatus       [IN]        Indicates which UVOV status to look for
 *  \param      pUnderOverVoltStat      [OUT]       Pointer to UVOV status variable in which API
 *                                                  will use to store the UVOV status
 *
 *  \return     Success code if BUCK UVOV status is obtained, error code otherwise.
 *              For valid success/error codes, refer to \ref Pmic_ErrorCodes
 */
static int32_t Pmic_powerTps6522xGetBuckStat(Pmic_CoreHandle_t        *pPmicCoreHandle,
                                             const uint8_t pwrRsrcUVOVStatus,
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

/**
 *  \brief      This function is used to get the desired LDO UVOV status or VCCA/VMON UVOV status.
 *
 *  \param      pPmicCoreHandle         [IN]        PMIC interface handle
 *  \param      pwrRsrcUVOVStatus       [IN]        Indicates which UVOV status to look for
 *  \param      pUnderOverVoltStat      [OUT]       Pointer to UVOV status variable in which API
 *                                                  will use to store the UVOV status
 *
 *  \return     Success code if LDO UVOV status or VCCA/VMON UVOV status is obtained, error code otherwise.
 *              For valid success/error codes, refer to \ref Pmic_ErrorCodes
 */
static int32_t Pmic_powerTps6522xGetLdoVccaVmonStat(Pmic_CoreHandle_t        *pPmicCoreHandle,
                                                    const uint8_t pwrRsrcUVOVStatus,
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
                                         const uint8_t pwrRsrcUVOVStatus,
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
        pThermalCfg->tsdOrdLvl = (uint8_t)Pmic_getBitField(
            regData, PMIC_POWER_TPS6522X_TSD_ORD_LEVEL_SHIFT, PMIC_POWER_TPS6522X_TSD_ORD_LEVEL_MASK);
    }
    // If TWARN_LEVEL validParam is set, extract TWARN_LEVEL bit
    if ((status == PMIC_ST_SUCCESS) &&
        pmic_validParamCheck(pThermalCfg->validParams, PMIC_POWER_TPS6522X_TWARN_LEVEL_VALID))
    {
        pThermalCfg->twarnLvl = (uint8_t)Pmic_getBitField(
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
