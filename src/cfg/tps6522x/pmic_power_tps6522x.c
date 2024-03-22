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

const tps6522xBuckRegs_t gTps6522xBuckRegisters[] =
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

const tps6522xLdoRegs_t gTps6522xLdoRegisters[] =
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

const tps6522xVccaVmonRegs_t gTps6522xVccaVmonRegisters[] =
{
    {
        TPS6522X_VCCA_VMON_CTRL_REGADDR, 
        TPS6522X_INVALID_REGADDR,
        TPS6522X_VMON1_PG_WINDOW_REGADDR,
        TPS6522X_VMON1_PG_LEVEL_REGADDR, 
        TPS6522X_RAIL_SEL_3_REGADDR
    },
    {
        TPS6522X_VCCA_VMON_CTRL_REGADDR, 
        TPS6522X_INVALID_REGADDR,
        TPS6522X_VMON2_PG_WINDOW_REGADDR,
        TPS6522X_VMON2_PG_LEVEL_REGADDR, 
        TPS6522X_RAIL_SEL_3_REGADDR
    },
    {
        TPS6522X_VCCA_VMON_CTRL_REGADDR, 
        TPS6522X_VCCA_PG_WINDOW_REGADDR,
        TPS6522X_INVALID_REGADDR,
        TPS6522X_INVALID_REGADDR,
        TPS6522X_RAIL_SEL_3_REGADDR
    }
};

/**
 *  \brief      This function is used to verify the PMIC handle and pointer to TPS6522x power resource
 *              configuration struct.
 *
 *  \param      pPmicCoreHandle     [IN]    PMIC interface handle
 *  \param      pPwrRsrcCfg         [IN]    Pointer to power resource configuration struct
 *
 *  \return     Success code if PMIC handle and power resource references are valid, error code otherwise.
 *              For valid success/error codes, refer to \ref Pmic_ErrorCodes
 */
static int32_t tps6522xParamCheck_pPwrRsrcCfg(const Pmic_CoreHandle_t *pPmicCoreHandle, const tps6522xPwrRsrcCfg_t *pPwrRsrcCfg)
{
    int32_t status = PMIC_ST_SUCCESS;

    // NULL parameter check
    if ((pPmicCoreHandle == NULL) || (pPwrRsrcCfg == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    // TPS6522x device type check
    if ((status == PMIC_ST_SUCCESS) && (pPmicCoreHandle->pmicDeviceType != PMIC_DEV_BURTON_TPS6522X))
    {
        status = PMIC_ST_ERR_INV_DEVICE;
    }

    // TPS6522x power subsystem check
    if ((status == PMIC_ST_SUCCESS) && ((pPmicCoreHandle->pPmic_SubSysInfo->buckEnable == (bool)false) ||
                                        (pPmicCoreHandle->pPmic_SubSysInfo->ldoEnable == (bool)false)))
    {
        status = PMIC_ST_ERR_INV_SUBSYSTEM;
    }

    // Power Resource CFG validParam check
    if ((status == PMIC_ST_SUCCESS) && (pPwrRsrcCfg->validParams == 0U))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    return status;
}

/**
 *  \brief      This function is used to verify the PMIC handle and constant TPS6522x power resource
 *              configuration struct.
 *
 *  \param      pPmicCoreHandle     [IN]    PMIC interface handle
 *  \param      pwrRsrcCfg          [IN]    Power resource configuration struct
 *
 *  \return     Success code if PMIC handle and constant power resource CFG are valid, error code otherwise.
 *              For valid success/error codes, refer to \ref Pmic_ErrorCodes
 */
static int32_t tps6522xParamCheck_constPwrRsrcCfg(const Pmic_CoreHandle_t *pPmicCoreHandle,
                                                  const tps6522xPwrRsrcCfg_t pwrRsrcCfg)
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
    if ((status == PMIC_ST_SUCCESS) && ((pPmicCoreHandle->pPmic_SubSysInfo->buckEnable == (bool)false) ||
                                        (pPmicCoreHandle->pPmic_SubSysInfo->ldoEnable == (bool)false)))
    {
        status = PMIC_ST_ERR_INV_SUBSYSTEM;
    }

    // Power Resource CFG validParam check
    if ((status == PMIC_ST_SUCCESS) && (pwrRsrcCfg.validParams == 0U))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    return status;
}

/**
 *  \brief      This function is used to check whether a voltage is valid for a Buck.
 *
 *  \param      buckCfg     [IN]    Buck power resource configuration struct
 *  \param      buckNum     [IN]    Indicates which BUCK the API is working with
 *
 *  \return     Success code if Buck voltage is within range, error code otherwise.
 *              For valid success/error codes, refer to \ref Pmic_ErrorCodes
 */
static int32_t tps6522xBuckVoltageWithinRangeCheck(const tps6522xBuckCfg_t buckCfg, const uint8_t buckNum)
{
    int32_t status = PMIC_ST_SUCCESS;

    if (pmic_validParamCheck(buckCfg.validParams, TPS6522X_BUCK_VOLTAGE_MV_VALID))
    {
        switch (buckNum)
        {
            case TPS6522X_REGULATOR_BUCK1:
                if ((buckCfg.buckVoltage_mv < 500U) || (buckCfg.buckVoltage_mv > 3300U))
                {
                    status = PMIC_ST_ERR_INV_VOLTAGE;
                }
                break;
            case TPS6522X_REGULATOR_BUCK2:
            case TPS6522X_REGULATOR_BUCK3:
            case TPS6522X_REGULATOR_BUCK4:
                if ((buckCfg.buckVoltage_mv < 500U) || (buckCfg.buckVoltage_mv > 3300U))
                {
                    status = PMIC_ST_ERR_INV_VOLTAGE;
                }
                break;
            // Invalid Buck number
            default:
                break;
        }
    }

    return status;
}

/**
 *  \brief      This function is used to check whether a voltage is valid for a LDO.
 *
 *  \param      ldoCfg      [IN]    LDO power resource configuration struct
 *  \param      ldoNum      [IN]    Indicates which LDO the API is working with
 *
 *  \return     Success code if LDO voltage is within range, error code otherwise.
 *              For valid success/error codes, refer to \ref Pmic_ErrorCodes
 */
static int32_t tps6522xLdoVoltageWithinRangeCheck(const tps6522xLdoCfg_t ldoCfg, const uint8_t ldoNum)
{
    int32_t status = PMIC_ST_SUCCESS;

    if (pmic_validParamCheck(ldoCfg.validParams, TPS6522X_LDO_VOLTAGE_MV_VALID))
    {
        switch (ldoNum)
        {
            case TPS6522X_REGULATOR_LDO1:
                if ((ldoCfg.ldoVoltage_mv < 1200U) || (ldoCfg.ldoVoltage_mv > 3300U))
                {
                    status = PMIC_ST_ERR_INV_VOLTAGE;
                }
                break;
            case TPS6522X_REGULATOR_LDO2:
            case TPS6522X_REGULATOR_LDO3:
                if ((ldoCfg.ldoVoltage_mv < 600U) || (ldoCfg.ldoVoltage_mv > 3400U))
                {
                    status = PMIC_ST_ERR_INV_VOLTAGE;
                }
                break;
            // Invalid LDO number
            default:
                break;
        }
    }

    return status;
}

/**
 *  \brief      This function is used as a helper function to check whether the 
 *              user-specified voltage for VMON1 or VMON2 is valid.
 * 
 *  \param      vccaVmonCfg     VCCA/VMON power resource configuration struct
 *  \param      vmonNum         VMON number (either VMON1 or VMON2)
 * 
 *  \return     Success code if voltage is valid for the VMON specified by \p vmonNum 
 *              error code otherwise. For valid success/error codes, refer to 
 *              \ref Pmic_ErrorCodes
 */
static int32_t tps6522xVmonVoltageWithinRangeCheck(const tps6522xVccaVmonCfg_t vccaVmonCfg, const uint8_t vmonNum)
{
    int32_t status = PMIC_ST_SUCCESS;

    if ((vmonNum == TPS6522X_VOLTAGE_MONITOR_VMON1) && 
        pmic_validParamCheck(vccaVmonCfg.validParams, TPS6522X_VMON1_PG_LEVEL_MV_VALID))
    {
        if ((vccaVmonCfg.vmon1PgLevel_mv < 500U) || (vccaVmonCfg.vmon1PgLevel_mv > 3340U))
        {
            status = PMIC_ST_ERR_INV_VOLTAGE;
        }
    }
    else if ((vmonNum == TPS6522X_VOLTAGE_MONITOR_VMON2) && 
            pmic_validParamCheck(vccaVmonCfg.validParams, TPS6522X_VMON2_PG_LEVEL_MV_VALID))
    {
        if ((vccaVmonCfg.vmon2PgLevel_mv < 500U) || (vccaVmonCfg.vmon2PgLevel_mv > 3300U))
        {
            status = PMIC_ST_ERR_INV_VOLTAGE;
        }
    }
    else
    {
        /* Invalid VMON number */
    }

    return status;
}

/**
 *  \brief      This function is used to store desired bit fields of the BUCK_CTRL register into the BUCK
 *              power resource CFG (desired bit fields are governed by validParams).
 *
 *  \param      pBuckCfg            [IN/OUT]    Pointer to BUCK power resource configuration struct
 *  \param      buckCtrlRegData     [IN]        BUCK_CTRL register data obtained from a prior read of the PMIC
 */
static void tps6522xGetBuckCtrlRegBitFields(tps6522xBuckCfg_t *pBuckCfg, const uint8_t buckCtrlRegData)
{
    if (pmic_validParamCheck(pBuckCfg->validParams, TPS6522X_BUCK_PLDN_VALID))
    {
        pBuckCfg->buckPldn = Pmic_getBitField(
            buckCtrlRegData, TPS6522X_BUCK_PLDN_SHIFT, TPS6522X_BUCK_PLDN_MASK);
    }
    if (pmic_validParamCheck(pBuckCfg->validParams, TPS6522X_BUCK_VMON_EN_VALID))
    {
        pBuckCfg->buckVmonEn = Pmic_getBitField(
            buckCtrlRegData, TPS6522X_BUCK_VMON_EN_SHIFT, TPS6522X_BUCK_VMON_EN_MASK);
    }
    if (pmic_validParamCheck(pBuckCfg->validParams, TPS6522X_BUCK_PWM_OPTION_VALID))
    {
        pBuckCfg->buckPwmOption =
            Pmic_getBitField(buckCtrlRegData,
                                                                TPS6522X_BUCK_PWM_OPTION_SHIFT,
                                                                TPS6522X_BUCK_PWM_OPTION_MASK);
    }
    if (pmic_validParamCheck(pBuckCfg->validParams, TPS6522X_BUCK_EN_VALID))
    {
        pBuckCfg->buckEn = Pmic_getBitField(
            buckCtrlRegData, TPS6522X_BUCK_EN_SHIFT, TPS6522X_BUCK_EN_MASK);
    }
}

/**
 *  \brief      This function is used to store desired bit fields of the BUCK_CONF register into the BUCK
 *              power resource CFG (desired bit fields are governed by validParams).
 *
 *  \param      pBuckCfg            [IN/OUT]    Pointer to BUCK power resource configuration struct
 *  \param      buckConfRegData     [IN]        BUCK_CONF register data obtained from a prior read of the PMIC
 */
static void tps6522xGetBuckConfRegBitFields(tps6522xBuckCfg_t *pBuckCfg, const uint8_t buckConfRegData)
{
    if (pmic_validParamCheck(pBuckCfg->validParams, TPS6522X_BUCK_SLEW_RATE_VALID))
    {
        pBuckCfg->buckSlewRate = Pmic_getBitField(buckConfRegData, 
                                                  TPS6522X_BUCK_SLEW_RATE_SHIFT, 
                                                  TPS6522X_BUCK_SLEW_RATE_MASK);
    }
}

/**
 *  \brief      This function is used to convert a BUCK VSET value to a BUCK voltage value.
 *
 *  \param      pBuckVoltage_mv     [OUT]   BUCK voltage value in millivolts
 *  \param      buckVoltage_vset    [IN]    BUCK voltage value in VSET form
 *  \param      buckNum             [IN]    Indicates which BUCK the API is working with
 */
static void tps6522xBuckConvertVsetVal2Voltage(uint16_t *pBuckVoltage_mv,
                                               const uint8_t buckVoltage_vset,
                                               const uint8_t buckNum)
{
    uint16_t baseVoltage_mv = 0, baseVoltage_vset = 0, voltageStep = 0;

    switch (buckNum)
    {
        case TPS6522X_REGULATOR_BUCK1:
            // BUCK1 voltage range 1: 500 mV to 580 mV in 20 mV steps (VSET: 0xA to 0xE)
            if ((buckVoltage_vset >= 0xAU) && (buckVoltage_vset <= 0xEU))
            {
                baseVoltage_mv = 500U;
                baseVoltage_vset = 0xAU;
                voltageStep = 20U;
            }
            // BUCK1 voltage range 2: 600 mV to 1095 mV in 5 mV steps (VSET: 0xF to 0x72)
            else if (buckVoltage_vset <= 0x72U)
            {
                baseVoltage_mv = 600U;
                baseVoltage_vset = 0xFU;
                voltageStep = 5U;
            }
            // BUCK1 voltage range 3: 1100 mV to 1650 mV in 10 mV steps (VSET: 0x73 to 0xAA)
            else if (buckVoltage_vset <= 0xAAU)
            {
                baseVoltage_mv = 1100U;
                baseVoltage_vset = 0x73U;
                voltageStep = 10U;
            }
            // BUCK1 voltage range 4: 1660 mV to 3300 mV in 20 mV steps (VSET: 0xAB to 0xFD)
            else if (buckVoltage_vset <= 0xFDU)
            {
                baseVoltage_mv = 1660U;
                baseVoltage_vset = 0xABU;
                voltageStep = 20U;
            }
            else
            {
                /* Invalid VSET value */
            }

            break;
        case TPS6522X_REGULATOR_BUCK2:
        case TPS6522X_REGULATOR_BUCK3:
        case TPS6522X_REGULATOR_BUCK4:
            // BUCK2, BUCK3, BUCK4 voltage range 1: 500 mV to 1150 mV in 25 mV steps (VSET: 0x0 to 0x1A)
            if (buckVoltage_vset <= 0x1AU)
            {
                baseVoltage_mv = 500U;
                baseVoltage_vset = 0x0U;
                voltageStep = 25U;
            }
            // BUCK2, BUCK3, BUCK4 voltage range 2: 1200 mV to 3300 mV in 50 mV steps (VSET: 0x1B to 0x45)
            else if (buckVoltage_vset <= 0x45U)
            {
                baseVoltage_mv = 1200U;
                baseVoltage_vset = 0x1BU;
                voltageStep = 50U;
            }
            else 
            {
                /* Invalid VSET value */ 
            }

            break;
        // Invalid Buck number
        default:
            break;
    }

    if ((baseVoltage_mv != 0U) && (voltageStep != 0U))
    {
        *pBuckVoltage_mv = baseVoltage_mv + (voltageStep * (buckVoltage_vset - baseVoltage_vset));
    }
}

/**
 *  \brief      This function is used to convert BUCK_VOUT register data from VSET to voltage (mV)
 *              and store it into the BUCK power resource CFG struct (if validParam is set).
 *
 *  \param      pBuckCfg            [IN/OUT]    Pointer to BUCK power resource configuration struct
 *  \param      buckVoutRegData     [IN]        BUCK_VOUT register data obtained from a prior read of the PMIC
 *  \param      buckNum             [IN]        Indicates which BUCK the API is working with
 */
static void tps6522xGetBuckVoutRegBitFields(tps6522xBuckCfg_t *pBuckCfg,
                                            const uint8_t buckVoutRegData,
                                            const uint8_t buckNum)
{
    if (pmic_validParamCheck(pBuckCfg->validParams, TPS6522X_BUCK_VOLTAGE_MV_VALID))
    {
        tps6522xBuckConvertVsetVal2Voltage(
            &(pBuckCfg->buckVoltage_mv), buckVoutRegData, buckNum);
    }
}

/**
 *  \brief      This function is used to store desired bit fields of the BUCK_PG_WINDOW register into the BUCK
 *              power resource CFG struct (desired bit fields are governed by validParams).
 *
 *  \param      pBuckCfg                [IN/OUT]    Pointer to BUCK power resource configuration struct
 *  \param      buckPgWindowRegData     [IN]        BUCK_PG_WINDOW register data obtained from a prior read
 *                                                  of the PMIC
 */
static void tps6522xGetBuckPgWindowRegBitFields(tps6522xBuckCfg_t *pBuckCfg, const uint8_t buckPgWindowRegData)
{
    if (pmic_validParamCheck(pBuckCfg->validParams, TPS6522X_BUCK_VMON_THR_VALID))
    {
        pBuckCfg->buckVmonThr =
            Pmic_getBitField(buckPgWindowRegData, 
                                      TPS6522X_BUCK_VMON_THR_SHIFT,
                                      TPS6522X_BUCK_VMON_THR_MASK);
    }
}

/**
 *  \brief      This function is used to store desired bit fields of the RAIL_SEL_1 register into the BUCK
 *              power resource CFG struct (desired bit fields are governed by validParams).
 *
 *  \param      pBuckCfg                [IN/OUT]    Pointer to BUCK power resource configuration struct
 *  \param      buckRailSelRegData      [IN]        RAIL_SEL_1 register data obtained from a prior read of the PMIC
 *  \param      buckNum                 [IN]        Indicates which BUCK the API is working with
 */
static void Pmic_PowerTps6522xGetRailSel1RegBitFields(tps6522xBuckCfg_t *pBuckCfg,
                                                      const uint8_t buckRailSelRegData,
                                                      const uint8_t buckNum)
{
    if (pmic_validParamCheck(pBuckCfg->validParams, TPS6522X_BUCK_RAIL_GRP_SEL_VALID))
    {
        switch (buckNum)
        {
            case TPS6522X_REGULATOR_BUCK1:
                pBuckCfg->buckRailGrpSel = Pmic_getBitField(
                    buckRailSelRegData,
                    TPS6522X_BUCK1_GRP_SEL_SHIFT,
                    TPS6522X_BUCK1_GRP_SEL_MASK);

                break;
            case TPS6522X_REGULATOR_BUCK2:
                pBuckCfg->buckRailGrpSel = Pmic_getBitField(
                    buckRailSelRegData,
                    TPS6522X_BUCK2_GRP_SEL_SHIFT,
                    TPS6522X_BUCK2_GRP_SEL_MASK);

                break;
            case TPS6522X_REGULATOR_BUCK3:
                pBuckCfg->buckRailGrpSel = Pmic_getBitField(
                    buckRailSelRegData,
                    TPS6522X_BUCK3_GRP_SEL_SHIFT,
                    TPS6522X_BUCK3_GRP_SEL_MASK);

                break;
            case TPS6522X_REGULATOR_BUCK4:
                pBuckCfg->buckRailGrpSel = Pmic_getBitField(
                    buckRailSelRegData,
                    TPS6522X_BUCK4_GRP_SEL_SHIFT,
                    TPS6522X_BUCK4_GRP_SEL_MASK);

                break;
            // Invalid Buck number
            default:
                break;
        }
    }
}

int32_t tps6522xGetBuckCfg(Pmic_CoreHandle_t *pPmicCoreHandle, tps6522xBuckCfg_t *pBuckCfg, const uint8_t buckNum)
{
    uint8_t buckCtrlRegData     = 0;
    uint8_t buckConfRegData     = 0;
    uint8_t buckVoutRegData     = 0;
    uint8_t buckPgWindowRegData = 0;
    uint8_t buckRailSelRegData  = 0;
    int32_t status              = PMIC_ST_SUCCESS;

    // Parameter check
    if (pBuckCfg == NULL)
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }
    if ((status == PMIC_ST_SUCCESS) && (pBuckCfg->validParams == 0U))
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
        tps6522xGetBuckCtrlRegBitFields(pBuckCfg, buckCtrlRegData);

        // BUCK_CONF register
        tps6522xGetBuckConfRegBitFields(pBuckCfg, buckConfRegData);

        // BUCK_VOUT register
        tps6522xGetBuckVoutRegBitFields(pBuckCfg, buckVoutRegData, buckNum);

        // BUCK_PG_WINDOW register
        tps6522xGetBuckPgWindowRegBitFields(pBuckCfg, buckPgWindowRegData);

        // RAIL_SEL_1 register
        Pmic_PowerTps6522xGetRailSel1RegBitFields(pBuckCfg, buckRailSelRegData, buckNum);
    }

    return status;
}

/**
 *  \brief      This function is used to store desired bit fields of the LDO_CTRL register into the LDO
 *              power resource CFG struct (desired bit fields are governed by validParams).
 *
 *  \param      pLdoCfg             [IN/OUT]    Pointer to LDO power resource configuration struct
 *  \param      ldoCtrlRegData      [IN]        LDO_CTRL register data obtained from a prior read of the PMIC
 */
static void tps6522xGetLdoCtrlRegBitFields(tps6522xLdoCfg_t *pLdoCfg, const uint8_t ldoCtrlRegData)
{
    if (pmic_validParamCheck(pLdoCfg->validParams, TPS6522X_LDO_DISCHARGE_EN_VALID))
    {
        pLdoCfg->ldoDischargeEn = Pmic_getBitField(
                ldoCtrlRegData, TPS6522X_LDO_DISCHARGE_EN_SHIFT, TPS6522X_LDO_DISCHARGE_EN_MASK);
    }
    if (pmic_validParamCheck(pLdoCfg->validParams, TPS6522X_LDO_VMON_EN_VALID))
    {
        pLdoCfg->ldoVmonEn = Pmic_getBitField(
            ldoCtrlRegData, TPS6522X_LDO_VMON_EN_SHIFT, TPS6522X_LDO_VMON_EN_MASK);
    }
    if (pmic_validParamCheck(pLdoCfg->validParams, TPS6522X_LDO_EN_VALID))
    {
        pLdoCfg->ldoEn = Pmic_getBitField(
            ldoCtrlRegData, TPS6522X_LDO_EN_SHIFT, TPS6522X_LDO_EN_MASK);
    }
}

/**
 *  \brief      This function is used to convert a LDO VSET value to LDO voltage value (mV).
 *
 *  \param      pLdoVoltage_mv      [OUT]       LDO voltage value in millivolts
 *  \param      ldoVoltage_vset     [IN]        LDO voltage value in VSET form
 *  \param      ldoNum              [IN]        Indicates which LDO the API is working with
 */
static void tps6522xLdoConvertVsetVal2Voltage(uint16_t *pLdoVoltage_mv,
                                              const uint8_t ldoVoltage_vset,
                                              const uint8_t ldoNum)
{
    uint16_t baseVoltage_mv = 0, baseVoltage_vset = 0, voltageStep = 0;

    switch (ldoNum)
    {
        case TPS6522X_REGULATOR_LDO1:
            // LDO1 voltage range 1: 1200 mV to 1200 mV in 0 mV steps (VSET: 0x0 to 0xC)
            if (ldoVoltage_vset <= 0xCU)
            {
                baseVoltage_mv = 1200U;
                baseVoltage_vset = 0x0U;
                voltageStep = 0U;
            }
            // LDO1 voltage range 2: 1250 mV to 3250 mV in 50 mV steps (VSET: 0xD to 0x35)
            else if (ldoVoltage_vset <= 0x35U)
            {
                baseVoltage_mv = 1250U;
                baseVoltage_vset = 0xDU;
                voltageStep = 50U;
            }
            // LDO1 voltage range 3: 3300 mV to 3300 mV in 0 mV steps (VSET: 0x36 to 0x3F)
            else if (ldoVoltage_vset <= 0x3FU)
            {
                baseVoltage_mv = 3300U;
                baseVoltage_vset = 0x36U;
                voltageStep = 0U;
            }
            else 
            {
                /* Invalid VSET value */
            }

            break;
        case TPS6522X_REGULATOR_LDO2:
        case TPS6522X_REGULATOR_LDO3:
            // LDO2 and LDO3 voltage range 1: 600 mV to 3350 mV in 50 mV steps (VSET: 0x0 to 0x37)
            if (ldoVoltage_vset <= 0x37U)
            {
                baseVoltage_mv = 600U;
                baseVoltage_vset = 0x0U;
                voltageStep = 50U;
            }
            // LDO2 and LDO3 voltage range 2: 3400 mV to 3400 mV in 0 mV steps (VSET: 0x38 to 0x3F)
            else if (ldoVoltage_vset <= 0x3FU)
            {
                baseVoltage_mv = 3400U;
                baseVoltage_vset = 0x38U;
                voltageStep = 0U;
            }
            else
            {
                /* Invalid VSET value */
            }

            break;
        // Invalid LDO number
        default:
            break;
    }

    if (baseVoltage_mv != 0U)
    {
        *pLdoVoltage_mv = baseVoltage_mv + (voltageStep * (ldoVoltage_vset - baseVoltage_vset));
    }
}

/**
 *  \brief      This function is used to get the desired bit fields of the LDO_VOUT register
 *              (desired bit fields are governed by validParams).
 *
 *  \param      pLdoCfg             [IN/OUT]    Pointer to LDO power resource configuration struct
 *  \param      ldoVoutRegData      [IN]        LDO_VOUT register data obtained from a prior read of the PMIC
 *  \param      ldoNum              [IN]        Indicates which LDO the API is working with
 */
static void tps6522xGetLdoVoutRegBitFields(tps6522xLdoCfg_t *pLdoCfg, 
                                           const uint8_t ldoVoutRegData, 
                                           const uint8_t ldoNum)
{
    uint8_t ldoVoltage_vset = 0;

    if (pmic_validParamCheck(pLdoCfg->validParams, TPS6522X_LDO_MODE_VALID))
    {
        pLdoCfg->ldoMode = Pmic_getBitField(
            ldoVoutRegData, TPS6522X_LDO_MODE_SHIFT, TPS6522X_LDO_MODE_MASK);
    }
    if (pmic_validParamCheck(pLdoCfg->validParams, TPS6522X_LDO_VOLTAGE_MV_VALID))
    {
        ldoVoltage_vset = Pmic_getBitField(
            ldoVoutRegData, TPS6522X_LDO_VSET_SHIFT, TPS6522X_LDO_VSET_MASK);

        tps6522xLdoConvertVsetVal2Voltage(&(pLdoCfg->ldoVoltage_mv), ldoVoltage_vset, ldoNum);
    }
}

/**
 *  \brief      This function is used to store desired bit fields of the LDO_PG_WINDOW register into the LDO
 *              power resource CFG struct (desired bit fields are governed by validParams).
 *
 *  \param      pLdoCfg                 [IN/OUT]    Pointer to LDO power resource configuration struct
 *  \param      ldoPgWindowRegData      [IN]        LDO_PG_WINDOW register data obtained from a prior read
 *                                                  of the PMIC
 */
static void tps6522xGetLdoPgWindowRegBitFields(tps6522xLdoCfg_t *pLdoCfg, const uint8_t ldoPgWindowRegData)
{
    if (pmic_validParamCheck(pLdoCfg->validParams, TPS6522X_LDO_VMON_THR_VALID))
    {
        pLdoCfg->ldoVmonThr = Pmic_getBitField(
            ldoPgWindowRegData, TPS6522X_LDO_VMON_THR_SHIFT, TPS6522X_LDO_VMON_THR_MASK);
    }
}

/**
 *  \brief      This function is used to store desired bit fields of the RAIL_SEL_2 register into the LDO
 *              power resource CFG struct (desired bit fields are governed by validParams).
 *
 *  \param      pLdoCfg                 [IN/OUT]    Pointer to LDO power resource configuration struct
 *  \param      ldoRailSelRegData       [IN]        RAIL_SEL_2 register data obtained from a prior read of the PMIC
 *  \param      ldoNum                  [IN]        Indicates which LDO the API is working with
 */
static void tps6522xGetRailSel2RegBitFields(tps6522xLdoCfg_t *pLdoCfg, 
                                            const uint8_t ldoRailSelRegData,
                                            const uint8_t ldoNum)
{
    if (pmic_validParamCheck(pLdoCfg->validParams, TPS6522X_LDO_RAIL_GRP_SEL_VALID))
    {
        switch (ldoNum)
        {
            case TPS6522X_REGULATOR_LDO1:
                pLdoCfg->ldoRailGrpSel = Pmic_getBitField(
                    ldoRailSelRegData, TPS6522X_LDO1_GRP_SEL_SHIFT, TPS6522X_LDO1_GRP_SEL_MASK);
                break;
            case TPS6522X_REGULATOR_LDO2:
                pLdoCfg->ldoRailGrpSel = Pmic_getBitField(
                    ldoRailSelRegData, TPS6522X_LDO2_GRP_SEL_SHIFT, TPS6522X_LDO2_GRP_SEL_MASK);
                break;
            case TPS6522X_REGULATOR_LDO3:
                pLdoCfg->ldoRailGrpSel =
                    Pmic_getBitField(
                        ldoRailSelRegData, TPS6522X_LDO3_GRP_SEL_SHIFT, TPS6522X_LDO3_GRP_SEL_MASK);
                break;
            // Invalid LDO number
            default: 
                break;
        }
    }
}

int32_t tps6522xGetLdoCfg(Pmic_CoreHandle_t *pPmicCoreHandle, tps6522xLdoCfg_t *pLdoCfg, const uint8_t ldoNum)
{
    uint8_t ldoCtrlRegData      = 0;
    uint8_t ldoVoutRegData      = 0;
    uint8_t ldoPgWindowRegData  = 0;
    uint8_t ldoRailSelRegData   = 0;
    int32_t status              = PMIC_ST_SUCCESS;

    // Parameter check
    if (pLdoCfg == NULL)
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }
    if ((status == PMIC_ST_SUCCESS) && (pLdoCfg->validParams == 0U))
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
        tps6522xGetLdoCtrlRegBitFields(pLdoCfg, ldoCtrlRegData);

        // LDO_VOUT register
        tps6522xGetLdoVoutRegBitFields(pLdoCfg, ldoVoutRegData, ldoNum);

        // LDO_PG_WINDOW register
        tps6522xGetLdoPgWindowRegBitFields(pLdoCfg, ldoPgWindowRegData);

        // RAIL_SEL_2 register
        tps6522xGetRailSel2RegBitFields(pLdoCfg, ldoRailSelRegData, ldoNum);
    }

    return status;
}

/**
 *  \brief      This function is used to store desired bit fields of the VCCA_VMON_CTRL register into the VCCA/VMON
 *              power resource CFG struct (desired bit fields are governed by validParams).
 *
 *  \param      pVccaVmonCfg            [IN/OUT]        Pointer to VCCA/VMON power resource configuration struct
 *  \param      vccaVmonCtrlRegData     [IN]            VCCA_VMON_CTRL register data obtained from a prior read of
 *                                                      the PMIC
 *  \param      vmonNum                 [IN]            Indicates which VCCA/VMONx the API is working with
 */
static void tps6522xGetVccaVmonCtrlBitFields(tps6522xVccaVmonCfg_t *pVccaVmonCfg, 
                                             uint8_t vccaVmonCtrlRegData,
                                             const uint8_t vmonNum)
{
    if (pmic_validParamCheck(pVccaVmonCfg->validParams, TPS6522X_VMON_DEGLITCH_SEL_VALID))
    {
        pVccaVmonCfg->vmonDeglitchSel =
            Pmic_getBitField(
                vccaVmonCtrlRegData, TPS6522X_VCCA_VMON_DEGL_SEL_SHIFT, TPS6522X_VCCA_VMON_DEGL_SEL_MASK);
    }

    if ((vmonNum == TPS6522X_VOLTAGE_MONITOR_VMON1) &&
        pmic_validParamCheck(pVccaVmonCfg->validParams, TPS6522X_VMON1_EN_VALID))
    {
        pVccaVmonCfg->vmon1En = Pmic_getBitField(
            vccaVmonCtrlRegData, TPS6522X_VMON1_EN_SHIFT, TPS6522X_VMON1_EN_MASK);
    }
    else if ((vmonNum == TPS6522X_VOLTAGE_MONITOR_VMON2) &&
             pmic_validParamCheck(pVccaVmonCfg->validParams, TPS6522X_VMON2_EN_VALID))
    {
        pVccaVmonCfg->vmon2En =
            Pmic_getBitField(
                vccaVmonCtrlRegData, TPS6522X_VMON2_EN_SHIFT, TPS6522X_VMON2_EN_MASK);
    }
    else if ((vmonNum == TPS6522X_VOLTAGE_MONITOR_VCCA_VMON) &&
             pmic_validParamCheck(pVccaVmonCfg->validParams, TPS6522X_VCCA_VMON_EN_VALID))
    {
        pVccaVmonCfg->vccaVmonEn =
            Pmic_getBitField(
                vccaVmonCtrlRegData, TPS6522X_VCCA_VMON_EN_SHIFT, TPS6522X_VCCA_VMON_EN_MASK);
    }
    else
    {
        /* Invalid VMON number or validParam is not set */
    }
}

/**
 *  \brief      This function is used to store desired bit fields of the VCCA_PG_WINDOW register into the VCCA/VMON
 *              power resource CFG struct (desired bit fields are governed by validParams).
 *
 *  \param      pVccaVmonPwrRsrcCfg     [IN/OUT]    Pointer to VCCA/VMON power resource configuration struct
 *  \param      vccaPgWindowRegData     [IN]        VCCA_PG_WINDOW register data
 */
static void tps6522xGetVccaPgWindowBitFields(tps6522xVccaVmonCfg_t *pVccaVmonPwrRsrcCfg, uint8_t vccaPgWindowRegData)
{
    if (pmic_validParamCheck(pVccaVmonPwrRsrcCfg->validParams, TPS6522X_VCCA_PG_LEVEL_VALID))
    {
        pVccaVmonPwrRsrcCfg->vccaPgLevel = Pmic_getBitField(
            vccaPgWindowRegData, TPS6522X_VCCA_PG_SET_SHIFT, TPS6522X_VCCA_PG_SET_MASK);
    }

    if (pmic_validParamCheck(pVccaVmonPwrRsrcCfg->validParams, TPS6522X_VCCA_VMON_THR_VALID))
    {
        pVccaVmonPwrRsrcCfg->vccaVmonThr = Pmic_getBitField(
            vccaPgWindowRegData, TPS6522X_VCCA_VMON_THR_SHIFT, TPS6522X_VCCA_VMON_THR_MASK);
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
static void tps6522xGetVmonPgWindowBitFields(tps6522xVccaVmonCfg_t *pVccaVmonPwrRsrcCfg,
                                            uint8_t vmonPgWindowRegData,
                                            const uint8_t vmonNum)
{
    if ((vmonNum == TPS6522X_VOLTAGE_MONITOR_VMON1) &&
        pmic_validParamCheck(pVccaVmonPwrRsrcCfg->validParams, TPS6522X_VMON1_THR_VALID))
    {
        pVccaVmonPwrRsrcCfg->vmon1Thr = Pmic_getBitField(
            vmonPgWindowRegData, TPS6522X_VMON1_THR_SHIFT, TPS6522X_VMON1_THR_MASK);
    }
    else if ((vmonNum == TPS6522X_VOLTAGE_MONITOR_VMON2) &&
             pmic_validParamCheck(pVccaVmonPwrRsrcCfg->validParams, TPS6522X_VMON2_THR_VALID))
    {
        pVccaVmonPwrRsrcCfg->vmon2Thr = Pmic_getBitField(
            vmonPgWindowRegData, TPS6522X_VMON2_THR_SHIFT, TPS6522X_VMON2_THR_MASK);
    }
    else
    {
        /* Invalid VMON number and/or validParam not set */
    }
}

/**
 *  \brief      This function is used to convert a VMON PG_SET value to VMON voltage value (mV).
 *
 *  \param      pVmonPgLevel_mv         [OUT]       VMON voltage value in millivolts
 *  \param      vmonPgLevel_pgSet       [IN]        VMON voltage value in PG_SET form
 *  \param      vmonNum                 [IN]        Indicates which VCCA/VMONx the API is working with
 */
static void tps6522xVmonConvertPgSet2Voltage(uint16_t *pVmonPgLevel_mv,
                                             const uint8_t vmonPgLevel_pgSet,
                                             const uint8_t vmonNum)
{
    uint16_t baseVoltage_mv = 0, baseVoltage_pgSet = 0, voltageStep = 0;

    switch (vmonNum)
    {
        case TPS6522X_VOLTAGE_MONITOR_VMON1:
            // VMON1 voltage range 1: 500 mV to 580 mV in 20 mV steps (PG_SET: 0xA to 0xE)
            if ((vmonPgLevel_pgSet >= 0xAU) && (vmonPgLevel_pgSet <= 0xEU))
            {
                baseVoltage_mv = 500U;
                baseVoltage_pgSet = 0xAU;
                voltageStep = 20U;
            }
            // VMON1 voltage range 2: 600 mV to 1095 mV in 5 mV steps (PG_SET: 0xF to 0x72)
            else if ((vmonPgLevel_pgSet >= 0xFU) && (vmonPgLevel_pgSet <= 0x72U))
            {
                baseVoltage_mv = 600U;
                baseVoltage_pgSet = 0xFU;
                voltageStep = 5U;
            }
            // VMON1 voltage range 3: 1100 mV to 1650 mV in 10 mV steps (PG_SET: 0x73 to 0xAA)
            else if ((vmonPgLevel_pgSet >= 0x73U) && (vmonPgLevel_pgSet <= 0xAAU))
            {
                baseVoltage_mv = 1100U;
                baseVoltage_pgSet = 0x73U;
                voltageStep = 10U;
            }
            // VMON1 voltage range 4: 1660 mV to 3340 mV in 20 mV steps (PG_SET: 0xAB to 0xFF)
            else if (vmonPgLevel_pgSet >= 0xABU)
            {
                baseVoltage_mv = 1660U;
                baseVoltage_pgSet = 0xABU;
                voltageStep = 20U;
            }
            else
            {
                /* Invalid PG_SET value */
            }
            break;
        case TPS6522X_VOLTAGE_MONITOR_VMON2:
            // VMON2 voltage range 1: 500 mV to 1150 mV in 25 mV steps (PG_SET: 0x0 to 0x1A)
            if (vmonPgLevel_pgSet <= 0x1AU)
            {
                baseVoltage_mv = 500U;
                baseVoltage_pgSet = 0x0U;
                voltageStep = 25U;
            }
            // VMON2 voltage range 2: 1200 mV to 3300 mV in 50 mV steps (PG_SET: 0x1B to 0x45)
            else if (vmonPgLevel_pgSet <= 0x45U)
            {
                baseVoltage_mv = 1200U;
                baseVoltage_pgSet = 0x1BU;
                voltageStep = 50U;
            }
            else
            {
                /* Invalid PG_SET value */
            }
            break;
        // Invalid VMON number
        default:
            break;
    }

    if ((baseVoltage_mv != 0U) && (voltageStep != 0U))
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
static void tps6522xGetVmonPgLevelBitFields(tps6522xVccaVmonCfg_t *pVccaVmonPwrRsrcCfg,
                                            uint8_t vmonPgLevelRegData,
                                            const uint8_t vmonNum)
{
    if ((vmonNum == TPS6522X_VOLTAGE_MONITOR_VMON1) &&
        pmic_validParamCheck(pVccaVmonPwrRsrcCfg->validParams, TPS6522X_VMON1_PG_LEVEL_MV_VALID))
    {
        tps6522xVmonConvertPgSet2Voltage(
            &(pVccaVmonPwrRsrcCfg->vmon1PgLevel_mv), vmonPgLevelRegData, vmonNum);
    }
    else if ((vmonNum == TPS6522X_VOLTAGE_MONITOR_VMON2) &&
             pmic_validParamCheck(pVccaVmonPwrRsrcCfg->validParams, TPS6522X_VMON2_PG_LEVEL_MV_VALID))
    {
        tps6522xVmonConvertPgSet2Voltage(
            &(pVccaVmonPwrRsrcCfg->vmon2PgLevel_mv), vmonPgLevelRegData, vmonNum);
    }
    else
    {
        /* Invalid VMON number or validParam not set */
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
static void tps6522xGetRailSel3RegBitFields(tps6522xVccaVmonCfg_t *pVccaVmonPwrRsrcCfg,
                                       uint8_t vccaVmonRailSelRegData,
                                       const uint8_t vmonNum)
{
    if ((vmonNum == TPS6522X_VOLTAGE_MONITOR_VMON1) &&
        pmic_validParamCheck(pVccaVmonPwrRsrcCfg->validParams, TPS6522X_VMON1_RAIL_GRP_SEL_VALID))
    {
        pVccaVmonPwrRsrcCfg->vmon1RailGrpSel = Pmic_getBitField(
                vccaVmonRailSelRegData, TPS6522X_VMON1_GRP_SEL_SHIFT, TPS6522X_VMON1_GRP_SEL_MASK);
    }
    else if ((vmonNum == TPS6522X_VOLTAGE_MONITOR_VMON2) &&
             pmic_validParamCheck(pVccaVmonPwrRsrcCfg->validParams, TPS6522X_VMON2_RAIL_GRP_SEL_VALID))
    {
        pVccaVmonPwrRsrcCfg->vmon2RailGrpSel = Pmic_getBitField(
                vccaVmonRailSelRegData, TPS6522X_VMON2_GRP_SEL_SHIFT, TPS6522X_VMON2_GRP_SEL_MASK);
    }
    else if ((vmonNum == TPS6522X_VOLTAGE_MONITOR_VCCA_VMON) &&
             pmic_validParamCheck(pVccaVmonPwrRsrcCfg->validParams, TPS6522X_VCCA_RAIL_GRP_SEL_VALID))
    {
        pVccaVmonPwrRsrcCfg->vccaRailGrpSel = Pmic_getBitField(
                vccaVmonRailSelRegData, TPS6522X_VCCA_GRP_SEL_SHIFT, TPS6522X_VCCA_GRP_SEL_MASK);
    }
    else
    {
        /* Invalid VMON number and/or validParam not set */
    }
}

int32_t tps6522xGetVccaVmonCfg(Pmic_CoreHandle_t *pPmicCoreHandle,
                               tps6522xVccaVmonCfg_t *pVccaVmonPwrRsrcCfg,
                               const uint8_t vmonNum)
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
    if ((status == PMIC_ST_SUCCESS) && (pVccaVmonPwrRsrcCfg->validParams == 0U))
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
    if ((status == PMIC_ST_SUCCESS) && (vmonNum == TPS6522X_VOLTAGE_MONITOR_VCCA_VMON))
    {
        status = Pmic_commIntf_recvByte(
            pPmicCoreHandle, gTps6522xVccaVmonRegisters[vmonNum].vccaPgWindowRegAddr, &vccaPgWindowRegData);
    }
    if ((status == PMIC_ST_SUCCESS) && ((vmonNum == TPS6522X_VOLTAGE_MONITOR_VMON1) ||
                                        (vmonNum == TPS6522X_VOLTAGE_MONITOR_VMON2)))
    {
        status = Pmic_commIntf_recvByte(
            pPmicCoreHandle, gTps6522xVccaVmonRegisters[vmonNum].vmonPgWindowRegAddr, &vmonPgWindowRegData);
    }
    if ((status == PMIC_ST_SUCCESS) && ((vmonNum == TPS6522X_VOLTAGE_MONITOR_VMON1) ||
                                        (vmonNum == TPS6522X_VOLTAGE_MONITOR_VMON2)))
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
        tps6522xGetVccaVmonCtrlBitFields(pVccaVmonPwrRsrcCfg, vccaVmonCtrlRegData, vmonNum);

        // VCCA_PG_WINDOW
        tps6522xGetVccaPgWindowBitFields(pVccaVmonPwrRsrcCfg, vccaPgWindowRegData);

        // VMON_PG_WINDOW
        tps6522xGetVmonPgWindowBitFields(pVccaVmonPwrRsrcCfg, vmonPgWindowRegData, vmonNum);

        // VMON_PG_LEVEL
        tps6522xGetVmonPgLevelBitFields(pVccaVmonPwrRsrcCfg, vmonPgLevelRegData, vmonNum);

        // RAIL_SEL_3
        tps6522xGetRailSel3RegBitFields(pVccaVmonPwrRsrcCfg, vccaVmonRailSelRegData, vmonNum);
    }

    // After read, stop critical section
    Pmic_criticalSectionStop(pPmicCoreHandle);

    return status;
}

int32_t tps6522xGetPwrRsrcCfg(Pmic_CoreHandle_t *pPmicCoreHandle, tps6522xPwrRsrcCfg_t *pPwrRsrcCfg)
{
    uint8_t  iter = 0;
    uint8_t validParam = 0;
    int32_t  status = PMIC_ST_SUCCESS;

    // Parameter check
    status = tps6522xParamCheck_pPwrRsrcCfg(pPmicCoreHandle, pPwrRsrcCfg);

    if (status == PMIC_ST_SUCCESS)
    {
        // For each BUCK, if the validParam for it is set, get the power resource configuration
        for (iter = 0; iter < TPS6522X_MAX_BUCK_NUM; iter++)
        {
            validParam = TPS6522X_BUCK1_VALID + iter;
            if (pmic_validParamCheck(pPwrRsrcCfg->validParams, validParam))
            {
                status = tps6522xGetBuckCfg(pPmicCoreHandle, &(pPwrRsrcCfg->buckCfg[iter]), iter);
            }

            // Upon error, break out of loop
            if (status != PMIC_ST_SUCCESS)
            {
                break;
            }
        }
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // For each LDO, if the validParam for it is set, get the power resource configuration
        for (iter = 0; iter < TPS6522X_MAX_LDO_NUM; iter++)
        {
            validParam = TPS6522X_LDO1_VALID + iter;
            if (pmic_validParamCheck(pPwrRsrcCfg->validParams, validParam))
            {
                status = tps6522xGetLdoCfg(pPmicCoreHandle, &(pPwrRsrcCfg->ldoCfg[iter]), iter);
            }

            // Upon error, break out of loop
            if (status != PMIC_ST_SUCCESS)
            {
                break;
            }
        }
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // For each VMON, if the validParam for it is set, get the power resource configuration
        for (iter = 0; iter < TPS6522X_MAX_VOLTAGE_MONITOR_NUM; iter++)
        {
            validParam = TPS6522X_VMON1_VALID + iter;
            if (pmic_validParamCheck(pPwrRsrcCfg->validParams, validParam))
            {
                status = tps6522xGetVccaVmonCfg(pPmicCoreHandle, &(pPwrRsrcCfg->vccaVmonCfg), iter);
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
 *  \param      buckCfg                 [IN]        BUCK power resource configuration struct
 *  \param      pBuckCtrlRegData        [OUT]       Pointer to BUCK_CTRL register data
 */
static void tps6522xSetBuckCtrlRegBitFields(const tps6522xBuckCfg_t buckCfg,
                                            uint8_t *pBuckCtrlRegData)
{
    if (pmic_validParamCheck(buckCfg.validParams, TPS6522X_BUCK_PLDN_VALID))
    {
        Pmic_setBitField(pBuckCtrlRegData,
                         TPS6522X_BUCK_PLDN_SHIFT,
                         TPS6522X_BUCK_PLDN_MASK,
                         buckCfg.buckPldn);
    }
    if (pmic_validParamCheck(buckCfg.validParams, TPS6522X_BUCK_VMON_EN_VALID))
    {
        Pmic_setBitField(pBuckCtrlRegData,
                         TPS6522X_BUCK_VMON_EN_SHIFT,
                         TPS6522X_BUCK_VMON_EN_MASK,
                         buckCfg.buckVmonEn);
    }
    if (pmic_validParamCheck(buckCfg.validParams, TPS6522X_BUCK_PWM_OPTION_VALID))
    {
        Pmic_setBitField(pBuckCtrlRegData,
                         TPS6522X_BUCK_PWM_OPTION_SHIFT,
                         TPS6522X_BUCK_PWM_OPTION_MASK,
                         buckCfg.buckPwmOption);
    }
    if (pmic_validParamCheck(buckCfg.validParams, TPS6522X_BUCK_EN_VALID))
    {
        Pmic_setBitField(pBuckCtrlRegData,
                         TPS6522X_BUCK_EN_SHIFT,
                         TPS6522X_BUCK_EN_MASK,
                         buckCfg.buckEn);
    }
}

/**
 *  \brief      This function is used to set desired bit fields of the BUCK_CONF register (desired bit fields
 *              are governed by validParams).
 *
 *  \param      buckCfg                 [IN]        BUCK power resource configuration struct
 *  \param      pBuckConfRegData        [OUT]       Pointer to BUCK_CONF register data
 */
static void tps6522xSetBuckConfRegBitFields(const tps6522xBuckCfg_t buckCfg, uint8_t *pBuckConfRegData)
{
    if (pmic_validParamCheck(buckCfg.validParams, TPS6522X_BUCK_SLEW_RATE_VALID))
    {
        Pmic_setBitField(pBuckConfRegData,
                         TPS6522X_BUCK_SLEW_RATE_SHIFT,
                         TPS6522X_BUCK_SLEW_RATE_MASK,
                         buckCfg.buckSlewRate);
    }
}

/**
 *  \brief      This function is used to convert a BUCK voltage value (mV) to BUCK VSET value.
 *
 *  \param      pBuckVoltage_vset       [OUT]       BUCK voltage in VSET form
 *  \param      buckVoltage_mv          [IN]        BUCK voltage in millivolts
 *  \param      buckNum                 [IN]        Indicates which BUCK the API is working with
 */
static void tps6522xBuckConvertVoltage2VsetVal(uint8_t *pBuckVoltage_vset,
                                               const uint16_t buckVoltage_mv,
                                               const uint8_t buckNum)
{
    uint16_t baseVoltage_vset = 0, baseVoltage_mv = 0, voltageStep = 0;

    switch (buckNum)
    {
        case TPS6522X_REGULATOR_BUCK1:
            // BUCK1 voltage range 1: 500 mV to 580 mV in 20 mV steps (VSET: 0xA to 0xE)
            if ((buckVoltage_mv >= 500U) && (buckVoltage_mv <= 580U))
            {
                baseVoltage_vset = 0xAU;
                baseVoltage_mv = 500U;
                voltageStep = 20U;
            }
            // BUCK1 voltage range 2: 600 mV to 1095 mV in 5 mV steps (VSET: 0xF to 0x72)
            else if ((buckVoltage_mv >= 600U) && (buckVoltage_mv <= 1095U))
            {
                baseVoltage_vset = 0xFU;
                baseVoltage_mv = 600U;
                voltageStep = 5U;
            }
            // BUCK1 voltage range 3: 1100 mV to 1650 mV in 10 mV steps (VSET: 0x73 to 0xAA)
            else if ((buckVoltage_mv >= 1100U) && (buckVoltage_mv <= 1650U))
            {
                baseVoltage_vset = 0x73U;
                baseVoltage_mv = 1100U;
                voltageStep = 10U;
            }
            // BUCK1 voltage range 4: 1660 mV to 3300 mV in 20 mV steps (VSET: 0xAB to 0xFD)
            else if ((buckVoltage_mv >= 1660U) && (buckVoltage_mv <= 3300U))
            {
                baseVoltage_vset = 0xABU;
                baseVoltage_mv = 1660U;
                voltageStep = 20U;
            }
            else
            {
                /* Invalid voltage value */
            }
            break;
        case TPS6522X_REGULATOR_BUCK2:
        case TPS6522X_REGULATOR_BUCK3:
        case TPS6522X_REGULATOR_BUCK4:
        // BUCK2, BUCK3, BUCK4 voltage range 1: 500 mV to 1150 mV in 25 mV steps (VSET: 0x0 to 0x1A)
            if ((buckVoltage_mv >= 500U) && (buckVoltage_mv <= 1150U))
            {
                baseVoltage_vset = 0x0U;
                baseVoltage_mv = 500U;
                voltageStep = 25U;
            }
            // BUCK2, BUCK3, BUCK4 voltage range 2: 1200 mV to 3300 mV in 50 mV steps (VSET: 0x1B to 0x45)
            else if ((buckVoltage_mv >= 1200U) && (buckVoltage_mv <= 3300U))
            {
                baseVoltage_vset = 0x1BU;
                baseVoltage_mv = 1200U;
                voltageStep = 50U;
            }
            else
            {
                /* Invalid voltage value */
            }
            break;
        // Invalid Buck number
        default:
            break;
    }

    if (voltageStep != 0U)
    {
        *pBuckVoltage_vset = (uint8_t)(baseVoltage_vset + ((buckVoltage_mv - baseVoltage_mv) / voltageStep));
    }
}

/**
 *  \brief      This function calculates a VSET value given a milivolt value (obtained from BUCK power resource CFG)
 *              and sets the BUCK_VOUT register data equal to the VSET value (if validParam is set).
 *
 *  \param      buckCfg                 [IN]        BUCK power resource configuration struct
 *  \param      pBuckVoutRegData        [OUT]       Pointer to BUCK_VOUT register data
 *  \param      buckNum                 [IN]        Indicates which BUCK the API is working with
 */
static void tps6522xSetBuckVoutRegBitFields(const tps6522xBuckCfg_t buckCfg,
                                            uint8_t *pBuckVoutRegData,
                                            const uint8_t buckNum)
{
    if (pmic_validParamCheck(buckCfg.validParams, TPS6522X_BUCK_VOLTAGE_MV_VALID))
    {
        tps6522xBuckConvertVoltage2VsetVal(pBuckVoutRegData, buckCfg.buckVoltage_mv, buckNum);
    }
}

/**
 *  \brief      This function is used to set desired bit fields of the BUCK_PG_WINDOW register (desired
 *              bit fields are governed by validParams).
 *
 *  \param      buckCfg    [IN]        BUCK power resource configuration struct
 *  \param      pBuckPgWindowRegData    [OUT]       Pointer to BUCK_PG_WINDOW register data
 */
static void tps6522xSetBuckPgWindowRegBitFields(const tps6522xBuckCfg_t buckCfg, uint8_t *pBuckPgWindowRegData)
{
    if (pmic_validParamCheck(buckCfg.validParams, TPS6522X_BUCK_VMON_THR_VALID))
    {
        Pmic_setBitField(pBuckPgWindowRegData,
                         TPS6522X_BUCK_VMON_THR_SHIFT,
                         TPS6522X_BUCK_VMON_THR_MASK,
                         buckCfg.buckVmonThr);
    }
}

/**
 *  \brief      This function is used to set the desired bit fields of the RAIL_SEL_1 register (desired
 *              bit fields are governed by validParams).
 *
 *  \param      buckCfg                 [IN]        BUCK power resource configuration struct
 *  \param      pBuckRailSelRegData     [OUT]       Pointer to RAIL_SEL_1 register data
 *  \param      buckNum                 [IN]        Indicates which BUCK the API is working with
 */
static void tps6522xSetRailSel1RegBitFields(const tps6522xBuckCfg_t buckCfg,
                                            uint8_t *pBuckRailSelRegData,
                                            const uint8_t buckNum)
{
    if (pmic_validParamCheck(buckCfg.validParams, TPS6522X_BUCK_RAIL_GRP_SEL_VALID))
    {
        switch (buckNum)
        {
            case TPS6522X_REGULATOR_BUCK1:
                Pmic_setBitField(pBuckRailSelRegData,
                                 TPS6522X_BUCK1_GRP_SEL_SHIFT,
                                 TPS6522X_BUCK1_GRP_SEL_MASK,
                                 buckCfg.buckRailGrpSel);
                break;
            case TPS6522X_REGULATOR_BUCK2:
                Pmic_setBitField(pBuckRailSelRegData,
                                 TPS6522X_BUCK2_GRP_SEL_SHIFT,
                                 TPS6522X_BUCK2_GRP_SEL_MASK,
                                 buckCfg.buckRailGrpSel);
                break;
            case TPS6522X_REGULATOR_BUCK3:
                Pmic_setBitField(pBuckRailSelRegData,
                                 TPS6522X_BUCK3_GRP_SEL_SHIFT,
                                 TPS6522X_BUCK3_GRP_SEL_MASK,
                                 buckCfg.buckRailGrpSel);
                break;
            case TPS6522X_REGULATOR_BUCK4:
                Pmic_setBitField(pBuckRailSelRegData,
                                 TPS6522X_BUCK4_GRP_SEL_SHIFT,
                                 TPS6522X_BUCK4_GRP_SEL_MASK,
                                 buckCfg.buckRailGrpSel);
                break;
            // Invalid Buck number
            default:
                break;
        }
    }
}

int32_t tps6522xSetBuckCfg(Pmic_CoreHandle_t *pPmicCoreHandle, const tps6522xBuckCfg_t buckCfg, const uint8_t buckNum)
{
    uint8_t buckCtrlRegData     = 0;
    uint8_t buckConfRegData     = 0;
    uint8_t buckVoutRegData     = 0;
    uint8_t buckPgWindowRegData = 0;
    uint8_t buckRailSelRegData  = 0;
    int32_t status              = PMIC_ST_SUCCESS;

    // Parameter check
    if (buckCfg.validParams == 0U)
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }
    if (status == PMIC_ST_SUCCESS)
    {
        status = tps6522xBuckVoltageWithinRangeCheck(buckCfg, buckNum);
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
        tps6522xSetBuckCtrlRegBitFields(buckCfg, &buckCtrlRegData);

        // BUCK_CONF register
        tps6522xSetBuckConfRegBitFields(buckCfg, &buckConfRegData);

        // BUCK_VOUT register
        tps6522xSetBuckVoutRegBitFields(buckCfg, &buckVoutRegData, buckNum);

        // BUCK_PG_WINDOW register
        tps6522xSetBuckPgWindowRegBitFields(buckCfg, &buckPgWindowRegData);

        // RAIL_SEL_1 register
        tps6522xSetRailSel1RegBitFields(buckCfg, &buckRailSelRegData, buckNum);
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
 *  \param      ldoCfg              [IN]        LDO power resource configuration struct
 *  \param      pLdoCtrlRegData     [OUT]       Pointer to LDO_CTRL register data
 */
static void tps6522xSetLdoCtrlRegBitFields(const tps6522xLdoCfg_t ldoCfg, uint8_t  *pLdoCtrlRegData)
{
    if (pmic_validParamCheck(ldoCfg.validParams, TPS6522X_LDO_DISCHARGE_EN_VALID))
    {
        Pmic_setBitField(pLdoCtrlRegData,
                         TPS6522X_LDO_DISCHARGE_EN_SHIFT,
                         TPS6522X_LDO_DISCHARGE_EN_MASK,
                         ldoCfg.ldoDischargeEn);
    }
    if (pmic_validParamCheck(ldoCfg.validParams, TPS6522X_LDO_VMON_EN_VALID))
    {
        Pmic_setBitField(pLdoCtrlRegData,
                         TPS6522X_LDO_VMON_EN_SHIFT,
                         TPS6522X_LDO_VMON_EN_MASK,
                         ldoCfg.ldoVmonEn);
    }
    if (pmic_validParamCheck(ldoCfg.validParams, TPS6522X_LDO_EN_VALID))
    {
        Pmic_setBitField(pLdoCtrlRegData,
                         TPS6522X_LDO_EN_SHIFT,
                         TPS6522X_LDO_EN_MASK,
                         ldoCfg.ldoEn);
    }
}

/**
 *  \brief      This function is used to convert LDO voltage (mV) to LDO VSET value.
 *
 *  \param      pLdoVoltage_vset    [OUT]       LDO voltage value in VSET form
 *  \param      ldoVoltage_mv       [IN]        LDO voltage value in millivolts
 *  \param      ldoNum              [IN]        Indicates which LDO the API is working with
 */
static void tps6522xLdoConvertVoltage2VsetVal(uint8_t *pLdoVoltage_vset,
                                              const uint16_t ldoVoltage_mv,
                                              const uint8_t  ldoNum)
{
    uint16_t baseVoltage_vset = 0, baseVoltage_mv = 0, voltageStep = 0;

    switch (ldoNum)
    {
        case TPS6522X_REGULATOR_LDO1:
        // LDO1 voltage range 1: 1200 mV to 1200 mV in 0 mV steps (VSET: 0x0 to 0xC)
            if (ldoVoltage_mv == 1200U)
            {
                *pLdoVoltage_vset = 0x0U;
            }
            // LDO1 voltage range 2: 1250 mV to 3250 mV in 50 mV steps (VSET: 0xD to 0x35)
            else if ((ldoVoltage_mv >= 1250U) && (ldoVoltage_mv <= 3250U))
            {
                baseVoltage_vset = 0xDU;
                baseVoltage_mv = 1250U;
                voltageStep = 50U;
            }
            // LDO1 voltage range 3: 3300 mV to 3300 mV in 0 mV steps (VSET: 0x36 to 0x3F)
            else if (ldoVoltage_mv == 3300U)
            {
                *pLdoVoltage_vset = 0x36U;
            }
            else
            {
                /* Invalid voltage number */
            }

            break;
        case TPS6522X_REGULATOR_LDO2:
        case TPS6522X_REGULATOR_LDO3:
        // LDO2 and LDO3 voltage range 1: 600 mV to 3350 mV in 50 mV steps (VSET: 0x0 to 0x37)
            if ((ldoVoltage_mv >= 600U) && (ldoVoltage_mv <= 3350U))
            {
                baseVoltage_vset = 0x0U;
                baseVoltage_mv = 600U;
                voltageStep = 50U;
            }
            // LDO2 and LDO3 voltage range 2: 3400 mV to 3400 mV in 0 mV steps (VSET: 0x38 to 0x3F)
            else if (ldoVoltage_mv == 3400U)
            {
                *pLdoVoltage_vset = 0x38U;
            }
            else
            {
                /* Invalid voltage number */
            }

            break;
        // Invalid LDO number
        default:
            break;
    }

    if (voltageStep != 0U)
    {
        *pLdoVoltage_vset = (uint8_t)(baseVoltage_vset + ((ldoVoltage_mv - baseVoltage_mv) / voltageStep));
    }
}

/**
 *  \brief      This function is used to set the desired bit fields of the LDO_VOUT register
 *              (desired bit fields are governed by validParams).
 *
 *  \param      ldoCfg                  [IN]        LDO power resource configuration struct
 *  \param      pLdoVoutRegData         [OUT]       Pointer to LDO_VOUT register data
 *  \param      ldoNum                  [IN]        Indicates which LDO the API is working with
 */
static void tps6522xSetLdoVoutRegBitFields(const tps6522xLdoCfg_t ldoCfg,
                                           uint8_t *pLdoVoutRegData,
                                           const uint8_t ldoNum)
{
    uint8_t ldoVoltage_vset = 0;

    if (pmic_validParamCheck(ldoCfg.validParams, TPS6522X_LDO_MODE_VALID))
    {
        Pmic_setBitField(pLdoVoutRegData,
                         TPS6522X_LDO_MODE_SHIFT,
                         TPS6522X_LDO_MODE_MASK,
                         ldoCfg.ldoMode);
    }

    if (pmic_validParamCheck(ldoCfg.validParams, TPS6522X_LDO_VOLTAGE_MV_VALID))
    {
        tps6522xLdoConvertVoltage2VsetVal(&ldoVoltage_vset, ldoCfg.ldoVoltage_mv, ldoNum);

        Pmic_setBitField(pLdoVoutRegData,
                         TPS6522X_LDO_VSET_SHIFT,
                         TPS6522X_LDO_VSET_MASK,
                         ldoVoltage_vset);
    }
}

/**
 *  \brief      This function is used to set the desired bit fields of the LDO_PG_WINDOW register
 *              (desired bit fields are governed by validParams).
 *
 *  \param      ldoCfg                  [IN]        LDO power resource configuration struct
 *  \param      pLdoPgWindowRegData     [OUT]       Pointer to LDO_PG_WINDOW register data
 */
static void tps6522xSetLdoPgWindowRegBitFields(const tps6522xLdoCfg_t ldoCfg,
                                               uint8_t *pLdoPgWindowRegData)
{
    if (pmic_validParamCheck(ldoCfg.validParams, TPS6522X_LDO_VMON_THR_VALID))
    {
        Pmic_setBitField(pLdoPgWindowRegData,
                         TPS6522X_LDO_VMON_THR_SHIFT,
                         TPS6522X_LDO_VMON_THR_MASK,
                         ldoCfg.ldoVmonThr);
    }
}

/**
 *  \brief      This function is used to set the desired bit fields of the RAIL_SEL_2 register
 *              (desired bit fields are governed by validParams).
 *
 *  \param      ldoCfg                  [IN]        LDO power resource configuration struct
 *  \param      pLdoRailSelRegData      [OUT]       Pointer to RAIL_SEL_2 register data
 *  \param      ldoNum                  [IN]        Indicates which LDO the API is working with
 */
static void tps6522xSetRailSel2RegBitFields(const tps6522xLdoCfg_t ldoCfg, uint8_t *pLdoRailSelRegData, uint8_t ldoNum)
{
    if (pmic_validParamCheck(ldoCfg.validParams, TPS6522X_LDO_RAIL_GRP_SEL_VALID))
    {
        switch (ldoNum)
        {
            case TPS6522X_REGULATOR_LDO1:
                Pmic_setBitField(pLdoRailSelRegData,
                                 TPS6522X_LDO1_GRP_SEL_SHIFT,
                                 TPS6522X_LDO1_GRP_SEL_MASK,
                                 ldoCfg.ldoRailGrpSel);
                break;
            case TPS6522X_REGULATOR_LDO2:
                Pmic_setBitField(pLdoRailSelRegData,
                                 TPS6522X_LDO2_GRP_SEL_SHIFT,
                                 TPS6522X_LDO2_GRP_SEL_MASK,
                                 ldoCfg.ldoRailGrpSel);
                break;
            case TPS6522X_REGULATOR_LDO3:
                Pmic_setBitField(pLdoRailSelRegData,
                                 TPS6522X_LDO3_GRP_SEL_SHIFT,
                                 TPS6522X_LDO3_GRP_SEL_MASK,
                                 ldoCfg.ldoRailGrpSel);
                break;
            // Invalid LDO number
            default:
                break;
        }
    }
}

int32_t tps6522xSetLdoCfg(Pmic_CoreHandle_t *pPmicCoreHandle, const tps6522xLdoCfg_t ldoCfg, const uint8_t ldoNum)
{
    uint8_t ldoCtrlRegData      = 0;
    uint8_t ldoVoutRegData      = 0;
    uint8_t ldoPgWindowRegData  = 0;
    uint8_t ldoRailSelRegData   = 0;
    int32_t status              = PMIC_ST_SUCCESS;

    // Parameter check
    if (ldoCfg.validParams == 0U)
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }
    if (status == PMIC_ST_SUCCESS)
    {
        status = tps6522xLdoVoltageWithinRangeCheck(ldoCfg, ldoNum);
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
        tps6522xSetLdoCtrlRegBitFields(ldoCfg, &ldoCtrlRegData);

        // LDO_VOUT register
        tps6522xSetLdoVoutRegBitFields(ldoCfg, &ldoVoutRegData, ldoNum);

        // LDO_PG_WINDOW register
        tps6522xSetLdoPgWindowRegBitFields(ldoCfg, &ldoPgWindowRegData);

        // RAIL_SEL_2 register
        tps6522xSetRailSel2RegBitFields(ldoCfg, &ldoRailSelRegData, ldoNum);
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
 *  \param      vccaVmonCfg             [IN]        VCCA/VMON power resource configuration struct
 *  \param      pVccaVmonCtrlRegData    [OUT]       Pointer to VCCA_VMON_CTRL register data
 *  \param      vmonNum                 [IN]        Indicates which VCCA/VMONx the API is working with
 */
static void tps6522xSetVccaVmonCtrlBitFields(const tps6522xVccaVmonCfg_t vccaVmonCfg,
                                             uint8_t *pVccaVmonCtrlRegData)
{
    if (pmic_validParamCheck(vccaVmonCfg.validParams, TPS6522X_VMON_DEGLITCH_SEL_VALID))
    {
        Pmic_setBitField(pVccaVmonCtrlRegData,
                         TPS6522X_VCCA_VMON_DEGL_SEL_SHIFT,
                         TPS6522X_VCCA_VMON_DEGL_SEL_MASK,
                         vccaVmonCfg.vmonDeglitchSel);
    }
    if (pmic_validParamCheck(vccaVmonCfg.validParams, TPS6522X_VMON2_EN_VALID))
    {
        Pmic_setBitField(pVccaVmonCtrlRegData,
                         TPS6522X_VMON2_EN_SHIFT,
                         TPS6522X_VMON2_EN_MASK,
                         vccaVmonCfg.vmon2En);
    }
    if (pmic_validParamCheck(vccaVmonCfg.validParams, TPS6522X_VMON1_EN_VALID))
    {
        Pmic_setBitField(pVccaVmonCtrlRegData,
                         TPS6522X_VMON1_EN_SHIFT,
                         TPS6522X_VMON1_EN_MASK,
                         vccaVmonCfg.vmon1En);
    }
    if (pmic_validParamCheck(vccaVmonCfg.validParams, TPS6522X_VCCA_VMON_EN_VALID))
    {
        Pmic_setBitField(pVccaVmonCtrlRegData,
                         TPS6522X_VCCA_VMON_EN_SHIFT,
                         TPS6522X_VCCA_VMON_EN_MASK,
                         vccaVmonCfg.vccaVmonEn);
    }
}

/**
 *  \brief      This function is used to set desired bit fields of the VCCA_PG_WINDOW register
 *              (desired bit fields are governed by validParams).
 *
 *  \param      vccaVmonCfg             [IN]        VCCA/VMON power resource configuration struct
 *  \param      pVccaPgWindowRegData    [OUT]       Pointer to VCCA_PG_WINDOW register data
 */
static void tps6522xSetVccaPgWindowBitFields(const tps6522xVccaVmonCfg_t vccaVmonCfg, uint8_t *pVccaPgWindowRegData)
{
    if (pmic_validParamCheck(vccaVmonCfg.validParams, TPS6522X_VCCA_PG_LEVEL_VALID))
    {
        Pmic_setBitField(pVccaPgWindowRegData,
                         TPS6522X_VCCA_PG_SET_SHIFT,
                         TPS6522X_VCCA_PG_SET_MASK,
                         vccaVmonCfg.vccaPgLevel);
    }

    if (pmic_validParamCheck(vccaVmonCfg.validParams, TPS6522X_VCCA_VMON_THR_VALID))
    {
        Pmic_setBitField(pVccaPgWindowRegData,
                         TPS6522X_VCCA_VMON_THR_SHIFT,
                         TPS6522X_VCCA_VMON_THR_MASK,
                         vccaVmonCfg.vccaVmonThr);
    }
}

/**
 *  \brief      This function is used to set desired bit fields of the VMON_PG_WINDOW register
 *              (desired bit fields are governed by validParams).
 *
 *  \param      vccaVmonCfg             [IN]        VCCA/VMON power resource configuration struct
 *  \param      pVmonPgWindowRegData    [OUT]       Pointer to VMON_PG_WINDOW register data
 *  \param      vmonNum                 [IN]        Indicates which VCCA/VMONx the API is working with
 */
static void tps6522xSetVmonPgWindowBitFields(const tps6522xVccaVmonCfg_t vccaVmonCfg,
                                             uint8_t *pVmonPgWindowRegData,
                                             const uint8_t vmonNum)
{
    if ((vmonNum == TPS6522X_VOLTAGE_MONITOR_VMON1) &&
        pmic_validParamCheck(vccaVmonCfg.validParams, TPS6522X_VMON1_THR_VALID))
    {
        Pmic_setBitField(pVmonPgWindowRegData,
                         TPS6522X_VMON1_THR_SHIFT,
                         TPS6522X_VMON1_THR_MASK,
                         vccaVmonCfg.vmon1Thr);
    }
    else if ((vmonNum == TPS6522X_VOLTAGE_MONITOR_VMON2) &&
             pmic_validParamCheck(vccaVmonCfg.validParams, TPS6522X_VMON2_THR_VALID))
    {
        Pmic_setBitField(pVmonPgWindowRegData,
                         TPS6522X_VMON2_THR_SHIFT,
                         TPS6522X_VMON2_THR_MASK,
                         vccaVmonCfg.vmon2Thr);
    }
    else
    {
        /* Invalid VMON number or validParam not set */
    }
}

/**
 *  \brief      This function is used to convert a VMON voltage value (mV) to VMON PG_SET value.
 *
 *  \param      pVmonPgLevel_pgSet      [OUT]       VMON PG level in PG_SET form
 *  \param      vmonPgLevel_mv          [IN]        VMON PG level in millivolts
 *  \param      vmonNum                 [IN]        Indicates which VCCA/VMONx the API is working with
 */
static void tps6522xVmonConvertVoltage2PgSet(uint8_t *pVmonPgLevel_pgSet,
                                             const uint16_t vmonPgLevel_mv,
                                             const uint8_t vmonNum)
{
    uint16_t baseVoltage_vset = 0, baseVoltage_mv = 0, voltageStep = 0;

    switch (vmonNum)
    {
        case TPS6522X_VOLTAGE_MONITOR_VMON1:
            // VMON1 voltage range 1: 500 mV to 580 mV in 20 mV steps (PG_SET: 0xA to 0xE)
            if ((vmonPgLevel_mv >= 500U) && (vmonPgLevel_mv <= 580U))
            {
                baseVoltage_vset = 0xAU;
                baseVoltage_mv = 500U;
                voltageStep = 20U;
            }
            // VMON1 voltage range 2: 600 mV to 1095 mV in 5 mV steps (PG_SET: 0xF to 0x72)
            else if ((vmonPgLevel_mv >= 600U) && (vmonPgLevel_mv <= 1095U))
            {
                baseVoltage_vset = 0xFU;
                baseVoltage_mv = 600U;
                voltageStep = 5U;
            }
            // VMON1 voltage range 3: 1100 mV to 1650 mV in 10 mV steps (PG_SET: 0x73 to 0xAA)
            else if ((vmonPgLevel_mv >= 1100U) && (vmonPgLevel_mv <= 1650U))
            {
                baseVoltage_vset = 0x73U;
                baseVoltage_mv = 1100U;
                voltageStep = 10U;
            }
            // VMON1 voltage range 4: 1660 mV to 3340 mV in 20 mV steps (PG_SET: 0xAB to 0xFD)
            else if ((vmonPgLevel_mv >= 1660U) && (vmonPgLevel_mv <= 3340U))
            {
                baseVoltage_vset = 0xABU;
                baseVoltage_mv = 1660U;
                voltageStep = 20U;
            }
            else
            {
                /* Invalid voltage value */
            }
            break;
        case TPS6522X_VOLTAGE_MONITOR_VMON2:
            // VMON2 voltage range 1: 500 mV to 1150 mV in 25 mV steps (PG_SET: 0x0 to 0x1A)
            if ((vmonPgLevel_mv >= 500U) && (vmonPgLevel_mv <= 1150U))
            {
                baseVoltage_vset = 0x0U;
                baseVoltage_mv = 500U;
                voltageStep = 25U;
            }
            // VMON2 voltage range 2: 1200 mV to 3300 mV in 50 mV steps (PG_SET: 0x1B to 0x45)
            else if ((vmonPgLevel_mv >= 1200U) && (vmonPgLevel_mv <= 3300U))
            {
                baseVoltage_vset = 0x1BU;
                baseVoltage_mv = 1200U;
                voltageStep = 50U;
            }
            else
            {
                /* Invalid voltage value */
            }
            break;
        // Invalid VMON number
        default:
            break;
    }

    if (voltageStep != 0U)
    {
        *pVmonPgLevel_pgSet = (uint8_t)(baseVoltage_vset + ((vmonPgLevel_mv - baseVoltage_mv) / voltageStep));
    }
}

/**
 *  \brief      This function is used to set the desired bit fields of the VMON_PG_LEVEL register.
 *              It converts a millivolt value obtained from the VCCA/VMON power resource CFG struct
 *              to PG_SET and sets the VMON_PG_LEVEL register equal to the PG_SET value.
 *
 *  \param      vccaVmonCfg      [IN]        VCCA/VMON power resource configuration register
 *  \param      pVmonPgLevelRegData     [OUT]       Pointer to VMON_PG_LEVEL register data
 *  \param      vmonNum                 [IN]        Indicates which VCCA/VMONx the API is working with
 */
static void tps6522xSetVmonPgLevelBitFields(const tps6522xVccaVmonCfg_t vccaVmonCfg,
                                            uint8_t *pVmonPgLevelRegData,
                                            const uint8_t vmonNum)
{
    if ((vmonNum == TPS6522X_VOLTAGE_MONITOR_VMON1) &&
        pmic_validParamCheck(vccaVmonCfg.validParams, TPS6522X_VMON1_PG_LEVEL_MV_VALID))
    {
        tps6522xVmonConvertVoltage2PgSet(pVmonPgLevelRegData, vccaVmonCfg.vmon1PgLevel_mv, vmonNum);
    }
    else if ((vmonNum == TPS6522X_VOLTAGE_MONITOR_VMON2) &&
             pmic_validParamCheck(vccaVmonCfg.validParams, TPS6522X_VMON2_PG_LEVEL_MV_VALID))
    {
        tps6522xVmonConvertVoltage2PgSet(pVmonPgLevelRegData, vccaVmonCfg.vmon2PgLevel_mv, vmonNum);
    }
    else
    {
        /* Invalid VMON number or validParam not set */
    }
}

/**
 *  \brief      This function is used to set the desired bit fields of the RAIL_SEL_3 register
 *              (desired bit fields are governed by validParams).
 *
 *  \param      vccaVmonCfg          [IN]        VCCA/VMON power resource configuration struct
 *  \param      pVccaVmonRailSelRegData     [OUT]       Pointer to RAIL_SEL_3 register
 *  \param      vmonNum                     [IN]        Indicates which VCCA/VMONx the API is working with
 */
static void tps6522xSetRailSel3RegBitFields(const tps6522xVccaVmonCfg_t vccaVmonCfg,
                                            uint8_t *pVccaVmonRailSelRegData,
                                            const uint8_t vmonNum)
{
    if ((vmonNum == TPS6522X_VOLTAGE_MONITOR_VMON1) &&
        pmic_validParamCheck(vccaVmonCfg.validParams, TPS6522X_VMON1_RAIL_GRP_SEL_VALID))
    {
        Pmic_setBitField(pVccaVmonRailSelRegData,
                         TPS6522X_VMON1_GRP_SEL_SHIFT,
                         TPS6522X_VMON1_GRP_SEL_MASK,
                         vccaVmonCfg.vmon1RailGrpSel);
    }
    else if ((vmonNum == TPS6522X_VOLTAGE_MONITOR_VMON2) &&
             pmic_validParamCheck(vccaVmonCfg.validParams, TPS6522X_VMON2_RAIL_GRP_SEL_VALID))
    {
        Pmic_setBitField(pVccaVmonRailSelRegData,
                         TPS6522X_VMON2_GRP_SEL_SHIFT,
                         TPS6522X_VMON2_GRP_SEL_MASK,
                         vccaVmonCfg.vmon2RailGrpSel);
    }
    else if ((vmonNum == TPS6522X_VOLTAGE_MONITOR_VCCA_VMON) &&
             pmic_validParamCheck(vccaVmonCfg.validParams, TPS6522X_VCCA_RAIL_GRP_SEL_VALID))
    {
        Pmic_setBitField(pVccaVmonRailSelRegData,
                         TPS6522X_VCCA_GRP_SEL_SHIFT,
                         TPS6522X_VCCA_GRP_SEL_MASK,
                         vccaVmonCfg.vccaRailGrpSel);
    }
    else
    {
        /* Invalid VMON number or validParam not set */
    }
}

int32_t tps6522xSetVccaVmonCfg(Pmic_CoreHandle_t *pPmicCoreHandle, 
                               const tps6522xVccaVmonCfg_t vccaVmonCfg,
                               const uint8_t vmonNum)
{
    uint8_t vccaVmonCtrlRegData     = 0;
    uint8_t vccaPgWindowRegData     = 0;
    uint8_t vmonPgWindowRegData     = 0;
    uint8_t vmonPgLevelRegData      = 0;
    uint8_t vccaVmonRailSelRegData  = 0;
    int32_t status                  = PMIC_ST_SUCCESS;

    // Parameter check
    if (vccaVmonCfg.validParams == 0U)
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }
    if (status == PMIC_ST_SUCCESS)
    {
        status = tps6522xVmonVoltageWithinRangeCheck(vccaVmonCfg, vmonNum);
    }
    
    // As we are about to read, start critical section
    Pmic_criticalSectionStart(pPmicCoreHandle);

    // Read all registers pertaining to the VCCA_VMON/VMONx resource
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_commIntf_recvByte(
            pPmicCoreHandle, gTps6522xVccaVmonRegisters[vmonNum].vccaVmonCtrlRegAddr, &vccaVmonCtrlRegData);
    }
    if ((status == PMIC_ST_SUCCESS) && (vmonNum == TPS6522X_VOLTAGE_MONITOR_VCCA_VMON))
    {
        status = Pmic_commIntf_recvByte(
            pPmicCoreHandle, gTps6522xVccaVmonRegisters[vmonNum].vccaPgWindowRegAddr, &vccaPgWindowRegData);
    }
    if ((status == PMIC_ST_SUCCESS) && ((vmonNum == TPS6522X_VOLTAGE_MONITOR_VMON1) ||
                                        (vmonNum == TPS6522X_VOLTAGE_MONITOR_VMON2)))
    {
        status = Pmic_commIntf_recvByte(
            pPmicCoreHandle, gTps6522xVccaVmonRegisters[vmonNum].vmonPgWindowRegAddr, &vmonPgWindowRegData);
    }
    if ((status == PMIC_ST_SUCCESS) && ((vmonNum == TPS6522X_VOLTAGE_MONITOR_VMON1) ||
                                        (vmonNum == TPS6522X_VOLTAGE_MONITOR_VMON2)))
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
        tps6522xSetVccaVmonCtrlBitFields(vccaVmonCfg, &vccaVmonCtrlRegData);

        // VCCA_PG_WINDOW
        tps6522xSetVccaPgWindowBitFields(vccaVmonCfg, &vccaPgWindowRegData);

        // VMON_PG_WINDOW
        tps6522xSetVmonPgWindowBitFields(vccaVmonCfg, &vmonPgWindowRegData, vmonNum);

        // VMON_PG_LEVEL
        tps6522xSetVmonPgLevelBitFields(vccaVmonCfg, &vmonPgLevelRegData, vmonNum);

        // RAIL_SEL_3
        tps6522xSetRailSel3RegBitFields(vccaVmonCfg, &vccaVmonRailSelRegData, vmonNum);
    }

    // After modification of register bit fields, write values back to PMIC
    if ((status == PMIC_ST_SUCCESS) && (vmonNum == TPS6522X_VOLTAGE_MONITOR_VCCA_VMON))
    {
        status = Pmic_commIntf_sendByte(
            pPmicCoreHandle, gTps6522xVccaVmonRegisters[vmonNum].vccaPgWindowRegAddr, vccaPgWindowRegData);
    }
    if ((status == PMIC_ST_SUCCESS) && ((vmonNum == TPS6522X_VOLTAGE_MONITOR_VMON1) ||
                                        (vmonNum == TPS6522X_VOLTAGE_MONITOR_VMON2)))
    {
        status = Pmic_commIntf_sendByte(
            pPmicCoreHandle, gTps6522xVccaVmonRegisters[vmonNum].vmonPgWindowRegAddr, vmonPgWindowRegData);
    }
    if ((status == PMIC_ST_SUCCESS) && ((vmonNum == TPS6522X_VOLTAGE_MONITOR_VMON1) ||
                                        (vmonNum == TPS6522X_VOLTAGE_MONITOR_VMON2)))
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

int32_t tps6522xSetPwrRsrcCfg(Pmic_CoreHandle_t *pPmicCoreHandle,
                              const tps6522xPwrRsrcCfg_t pwrRsrcCfg)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t iter = 0;
    uint8_t validParam = 0;

    // Parameter check
    status = tps6522xParamCheck_constPwrRsrcCfg(pPmicCoreHandle, pwrRsrcCfg);

    if (status == PMIC_ST_SUCCESS)
    {
        // For each BUCK, if the validParam for it is set, set the power resource configuration
        for (iter = 0; iter < TPS6522X_MAX_BUCK_NUM; iter++)
        {
            validParam = TPS6522X_BUCK1_VALID + iter;
            if (pmic_validParamCheck(pwrRsrcCfg.validParams, validParam))
            {
                status = tps6522xSetBuckCfg(pPmicCoreHandle, pwrRsrcCfg.buckCfg[iter], iter);
            }

            // Upon error, break out of loop
            if (status != PMIC_ST_SUCCESS)
            {
                break;
            }
        }
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // For each LDO, if the validParam for it is set, set the power resource configuration
        for (iter = 0; iter < TPS6522X_MAX_LDO_NUM; iter++)
        {
            validParam = TPS6522X_LDO1_VALID + iter;
            if (pmic_validParamCheck(pwrRsrcCfg.validParams, validParam))
            {
                status = tps6522xSetLdoCfg(pPmicCoreHandle, pwrRsrcCfg.ldoCfg[iter], iter);
            }

            // Upon error, break out of loop
            if (status != PMIC_ST_SUCCESS)
            {
                break;
            }
        }
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // For each VMON, if the validParam for it is set, set the power resource configuration
        for (iter = 0; iter < TPS6522X_MAX_VOLTAGE_MONITOR_NUM; iter++)
        {
            validParam = TPS6522X_VMON1_VALID + iter;
            if (pmic_validParamCheck(pwrRsrcCfg.validParams, validParam))
            {
                status = tps6522xSetVccaVmonCfg(pPmicCoreHandle, pwrRsrcCfg.vccaVmonCfg, iter);
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
static int32_t tps6522xParamCheck_pmicHandle(const Pmic_CoreHandle_t *pPmicCoreHandle)
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
    if ((status == PMIC_ST_SUCCESS) && ((pPmicCoreHandle->pPmic_SubSysInfo->buckEnable == (bool)false) ||
                                        (pPmicCoreHandle->pPmic_SubSysInfo->ldoEnable == (bool)false)))
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
static int32_t tps6522xGetBuckStat(Pmic_CoreHandle_t *pPmicCoreHandle,
                                   const uint8_t pwrRsrcUVOVStatus,
                                   bool *pUnderOverVoltStat)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t statBuckRegData = 0;

    // Start critical section before reading
    Pmic_criticalSectionStart(pPmicCoreHandle);

    // Read STAT_BUCK register
    status = Pmic_commIntf_recvByte(pPmicCoreHandle, TPS6522X_STAT_BUCK_REGADDR, &statBuckRegData);

    // Extract desired UVOV status
    if (status == PMIC_ST_SUCCESS)
    {
        switch (pwrRsrcUVOVStatus)
        {
            case TPS6522X_BUCK4_UVOV_STAT:
                *pUnderOverVoltStat = Pmic_getBitField_b(statBuckRegData,
                                                         TPS6522X_BUCK4_UVOV_STAT_SHIFT);
                break;
            case TPS6522X_BUCK3_UVOV_STAT:
                *pUnderOverVoltStat = Pmic_getBitField_b(statBuckRegData,
                                                         TPS6522X_BUCK3_UVOV_STAT_SHIFT);
                break;
            case TPS6522X_BUCK2_UVOV_STAT:
                *pUnderOverVoltStat = Pmic_getBitField_b(statBuckRegData,
                                                         TPS6522X_BUCK2_UVOV_STAT_SHIFT);
                break;
            case TPS6522X_BUCK1_UVOV_STAT:
                *pUnderOverVoltStat = Pmic_getBitField_b(statBuckRegData,
                                                         TPS6522X_BUCK1_UVOV_STAT_SHIFT);
                break;
            // Invalid UVOV status
            default:
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
static int32_t tps6522xGetLdoVccaVmonStat(Pmic_CoreHandle_t *pPmicCoreHandle,
                                          const uint8_t pwrRsrcUVOVStatus,
                                          bool *pUnderOverVoltStat)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t statLdoVmonRegData = 0;

    // Start critical section before reading
    Pmic_criticalSectionStart(pPmicCoreHandle);

    // Read STAT_LDO_VMON register
    status = Pmic_commIntf_recvByte(pPmicCoreHandle, TPS6522X_STAT_LDO_VMON_REGADDR, &statLdoVmonRegData);

    // Extract desired UVOV status
    if (status == PMIC_ST_SUCCESS)
    {
        switch (pwrRsrcUVOVStatus)
        {
            case TPS6522X_VMON2_UVOV_STAT:
                *pUnderOverVoltStat = Pmic_getBitField_b(statLdoVmonRegData,
                                                       TPS6522X_VMON2_UVOV_STAT_SHIFT);
                break;
            case TPS6522X_VMON1_UVOV_STAT:
                *pUnderOverVoltStat = Pmic_getBitField_b(statLdoVmonRegData,
                                                       TPS6522X_VMON1_UVOV_STAT_SHIFT);
                break;
            case TPS6522X_VCCA_UVOV_STAT:
                *pUnderOverVoltStat = Pmic_getBitField_b(statLdoVmonRegData,
                                                       TPS6522X_VCCA_UVOV_STAT_SHIFT);
                break;
            case TPS6522X_LDO3_UVOV_STAT:
                *pUnderOverVoltStat = Pmic_getBitField_b(statLdoVmonRegData,
                                                       TPS6522X_LDO3_UVOV_STAT_SHIFT);
                break;
            case TPS6522X_LDO2_UVOV_STAT:
                *pUnderOverVoltStat = Pmic_getBitField_b(statLdoVmonRegData,
                                                       TPS6522X_LDO2_UVOV_STAT_SHIFT);
                break;
            case TPS6522X_LDO1_UVOV_STAT:
                *pUnderOverVoltStat = Pmic_getBitField_b(statLdoVmonRegData,
                                                       TPS6522X_LDO1_UVOV_STAT_SHIFT);
                break;
            // Invalid UVOV status
            default:
                break;
        }
    }

    // Stop critical section after reading
    Pmic_criticalSectionStop(pPmicCoreHandle);

    return status;
}

int32_t tps6522xGetPwrRsrcStat(Pmic_CoreHandle_t *pPmicCoreHandle,
                               const uint8_t pwrRsrcUVOVStatus,
                               bool *pUnderOverVoltStat)
{
    int32_t status = PMIC_ST_SUCCESS;

    // Parameter check
    status = tps6522xParamCheck_pmicHandle(pPmicCoreHandle);
    if ((status == PMIC_ST_SUCCESS) && (pUnderOverVoltStat == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }
    if ((status == PMIC_ST_SUCCESS) && (pwrRsrcUVOVStatus > TPS6522X_VCCA_UVOV_STAT))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Get Buck UVOV status if user specified Buck UVOV status
        if (pwrRsrcUVOVStatus <= TPS6522X_BUCK4_UVOV_STAT)
        {
            status = tps6522xGetBuckStat(pPmicCoreHandle, pwrRsrcUVOVStatus, pUnderOverVoltStat);
        }
        // Get LDO or VCCA_VMON/VMONx UVOV status if user specified LDO or VCCA_VMON/VMONx UVOV status
        else
        {
            status = tps6522xGetLdoVccaVmonStat(pPmicCoreHandle, pwrRsrcUVOVStatus, pUnderOverVoltStat);
        }
    }

    return status;
}

int32_t tps6522xGetThermalStat(Pmic_CoreHandle_t *pPmicCoreHandle,
                               tps6522xThermalStat_t *pThermalStat)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0;

    // Parameter check
    status = tps6522xParamCheck_pmicHandle(pPmicCoreHandle);
    if ((status == PMIC_ST_SUCCESS) && (pThermalStat == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }
    if ((status == PMIC_ST_SUCCESS) && (pThermalStat->validParams == 0U))
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
            pThermalStat->twarnStat = Pmic_getBitField_b(regData, TPS6522X_TWARN_STAT_SHIFT);
        }
    }

    // If TWARN_ORD_STAT validParam set, read STAT_MODERATE_ERR register and extract the TSD_ORD_STAT bit
    if ((status == PMIC_ST_SUCCESS) &&
        pmic_validParamCheck(pThermalStat->validParams, PMIC_THERMAL_STAT_ORD_SHTDWN_VALID))
    {
        status = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_STAT_MODERATE_ERR_REGADDR, &regData);

        if (status == PMIC_ST_SUCCESS)
        {
            pThermalStat->tsdOrdStat = Pmic_getBitField_b(regData, TPS6522X_TSD_ORD_STAT_SHIFT);
        }
    }

    // If TSD_IMM_STAT validParam set, read STAT_SEVERE_ERR register and extract the TSD_IMM_STAT bit
    if ((status == PMIC_ST_SUCCESS) &&
        pmic_validParamCheck(pThermalStat->validParams, PMIC_THERMAL_STAT_IMM_SHTDWN_VALID))
    {
        status = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_STAT_SEVERE_ERR_REGADDR, &regData);

        if (status == PMIC_ST_SUCCESS)
        {
            pThermalStat->tsdImmStat = Pmic_getBitField_b(regData, TPS6522X_TSD_IMM_STAT_SHIFT);
        }
    }

    // Stop critical section after reading
    Pmic_criticalSectionStop(pPmicCoreHandle);

    return status;
}

int32_t tps6522xGetThermalCfg(Pmic_CoreHandle_t *pPmicCoreHandle, tps6522xThermalCfg_t *pThermalCfg)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0;

    // Parameter check
    status = tps6522xParamCheck_pmicHandle(pPmicCoreHandle);
    if ((status == PMIC_ST_SUCCESS) && (pThermalCfg == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }
    if ((status == PMIC_ST_SUCCESS) && (pThermalCfg->validParams == 0U))
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
        pmic_validParamCheck(pThermalCfg->validParams, TPS6522X_TSD_ORD_LEVEL_VALID))
    {
        pThermalCfg->tsdOrdLvl = Pmic_getBitField(
            regData, TPS6522X_TSD_ORD_LEVEL_SHIFT, TPS6522X_TSD_ORD_LEVEL_MASK);
    }
    // If TWARN_LEVEL validParam is set, extract TWARN_LEVEL bit
    if ((status == PMIC_ST_SUCCESS) &&
        pmic_validParamCheck(pThermalCfg->validParams, TPS6522X_TWARN_LEVEL_VALID))
    {
        pThermalCfg->twarnLvl = Pmic_getBitField(
            regData, TPS6522X_TWARN_LEVEL_SHIFT, TPS6522X_TWARN_LEVEL_MASK);
    }

    // Stop critical section after reading
    Pmic_criticalSectionStop(pPmicCoreHandle);

    return status;
}

int32_t tps6522xSetThermalCfg(Pmic_CoreHandle_t *pPmicCoreHandle,
                              const tps6522xThermalCfg_t thermalCfg)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0;

    // Parameter check
    status = tps6522xParamCheck_pmicHandle(pPmicCoreHandle);
    if ((status == PMIC_ST_SUCCESS) && (thermalCfg.validParams == 0U))
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
        pmic_validParamCheck(thermalCfg.validParams, TPS6522X_TSD_ORD_LEVEL_VALID))
    {
        Pmic_setBitField(&regData, TPS6522X_TSD_ORD_LEVEL_SHIFT, TPS6522X_TSD_ORD_LEVEL_MASK, thermalCfg.tsdOrdLvl);
    }

    // If TWARN_LEVEL validParam is set, modify TWARN_LEVEL bit field
    if ((status == PMIC_ST_SUCCESS) &&
        pmic_validParamCheck(thermalCfg.validParams, TPS6522X_TWARN_LEVEL_VALID))
    {
        Pmic_setBitField(&regData, TPS6522X_TWARN_LEVEL_SHIFT, TPS6522X_TWARN_LEVEL_MASK, thermalCfg.twarnLvl);
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
