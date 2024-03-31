/******************************************************************************
 * Copyright (c) 2023-2024 Texas Instruments Incorporated - http://www.ti.com
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
 *  \brief      This function is used to check whether a VSET value is valid for a Buck.
 *
 *  \param      buckCfg     [IN]    Buck power resource configuration struct
 *  \param      buckNum     [IN]    Indicates which BUCK the API is working with
 *
 *  \return     Success code if Buck VSET is within range, error code otherwise.
 *              For valid success/error codes, refer to \ref Pmic_ErrorCodes
 */
static int32_t tps6522xBuckVsetWithinRangeCheck(const tps6522xBuckCfg_t buckCfg, const uint8_t buckNum)
{
    int32_t status = PMIC_ST_SUCCESS;

    if (pmic_validParamCheck(buckCfg.validParams, TPS6522X_BUCK_VSET_VALID))
    {
        switch (buckNum)
        {
            case TPS6522X_REGULATOR_BUCK1:
                if ((buckCfg.buckVset < TPS6522X_BUCK1_MIN_VSET) || 
                    (buckCfg.buckVset > TPS6522X_BUCK1_MAX_VSET))
                {
                    status = PMIC_ST_ERR_INV_VOLTAGE;
                }
                break;
            case TPS6522X_REGULATOR_BUCK2:
            case TPS6522X_REGULATOR_BUCK3:
            case TPS6522X_REGULATOR_BUCK4:
                // The min BUCK2, BUCK3, and BUCK4 VSET is 0, 
                // hence we do not need to check for a uint8_t < 0
                if (buckCfg.buckVset > TPS6522X_BUCK2_3_4_MAX_VSET)
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
 *  \brief      This function is used to check whether a VSET value is valid for a LDO.
 *
 *  \param      ldoCfg      [IN]    LDO power resource configuration struct
 *
 *  \return     Success code if LDO VSET is within range, error code otherwise.
 *              For valid success/error codes, refer to \ref Pmic_ErrorCodes
 */
static int32_t tps6522xLdoVsetWithinRangeCheck(const tps6522xLdoCfg_t ldoCfg)
{
    int32_t status = PMIC_ST_SUCCESS;

    // The min LDO1, LDO2, and LDO3 VSET is 0, hence 
    // we do not need to check for a uint8_t < 0
    if (pmic_validParamCheck(ldoCfg.validParams, TPS6522X_LDO_VSET_VALID) && 
        (ldoCfg.ldoVset > TPS6522X_LDO1_2_3_MAX_VSET))
    {
        status = PMIC_ST_ERR_INV_VOLTAGE;
    }

    return status;
}

/**
 *  \brief      This function is used to check whether a PG_SET value is valid for VMON1
 *              or VMON2 (depending on the \p vmonNum parameter).
 * 
 *  \param      vccaVmonCfg     VCCA/VMON power resource configuration struct
 *  \param      vmonNum         VMON number (either VMON1 or VMON2)
 * 
 *  \return     Success code if the PG_SET is within range for the VMON (specified by 
 *              \p vmonNum parameter), error code otherwise. For valid success/error 
 *              codes, refer to \ref Pmic_ErrorCodes
 */
static int32_t tps6522xVmonPgSetWithinRangeCheck(const tps6522xVccaVmonCfg_t vccaVmonCfg, const uint8_t vmonNum)
{
    int32_t status = PMIC_ST_SUCCESS;

    // The max VMON1 PG_SET is 0xFF, hence we do not need to check for a uint8_t > 0xFF
    if ((vmonNum == TPS6522X_VOLTAGE_MONITOR_VMON1) && 
        pmic_validParamCheck(vccaVmonCfg.validParams, TPS6522X_VMON1_PG_SET_VALID) &&
        (vccaVmonCfg.vmon1PgSet < TPS6522X_VMON1_MIN_PG_SET))
    {
        status = PMIC_ST_ERR_INV_VOLTAGE;
    }
    // The min VMON2 PG_SET is 0, hence we do not need to check for a uint8_t < 0
    else if ((vmonNum == TPS6522X_VOLTAGE_MONITOR_VMON2) && 
            pmic_validParamCheck(vccaVmonCfg.validParams, TPS6522X_VMON2_PG_SET_VALID) &&
            (vccaVmonCfg.vmon2PgSet > TPS6522X_VMON2_MAX_PG_SET))
    {
        status = PMIC_ST_ERR_INV_VOLTAGE;
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
 *  \brief      This function is used to convert BUCK_VOUT register data from VSET to voltage (mV)
 *              and store it into the BUCK power resource CFG struct (if validParam is set).
 *
 *  \param      pBuckCfg            [IN/OUT]    Pointer to BUCK power resource configuration struct
 *  \param      buckVoutRegData     [IN]        BUCK_VOUT register data obtained from a prior read of the PMIC
 *  \param      buckNum             [IN]        Indicates which BUCK the API is working with
 */
static void tps6522xGetBuckVoutRegBitFields(
    tps6522xBuckCfg_t *pBuckCfg, const uint8_t buckVoutRegData, const uint8_t buckNum)
{
    if (pmic_validParamCheck(pBuckCfg->validParams, TPS6522X_BUCK_VSET_VALID))
    {
        switch(buckNum)
        {
            case TPS6522X_REGULATOR_BUCK1: 
                pBuckCfg->buckVset = Pmic_getBitField(
                    buckVoutRegData, TPS6522X_BUCK_VSET_SHIFT, TPS6522X_BUCK1_VSET_MASK);
                break;
            case TPS6522X_REGULATOR_BUCK2:
            case TPS6522X_REGULATOR_BUCK3:
            case TPS6522X_REGULATOR_BUCK4:
                pBuckCfg->buckVset = Pmic_getBitField(
                    buckVoutRegData, TPS6522X_BUCK_VSET_SHIFT, TPS6522X_BUCK2_3_4_VSET_MASK);
                break;
            default:
                /* Invalid Buck number */
                break;
        }
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
        pBuckCfg->buckVmonThr = Pmic_getBitField(
            buckPgWindowRegData, TPS6522X_BUCK_VMON_THR_SHIFT, TPS6522X_BUCK_VMON_THR_MASK);
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

/**
 *  \brief      This function is used to read all PMIC registers relevant to a Buck
 *              power resource and stores the data into the \p regData parameter. 
 * 
 *  \param      pPmicCoreHandle     [IN]    PMIC interface handle
 *  \param      regData             [OUT]   Array of PMIC register data. For valid indices
 *                                          of the array, see \ref Tps6522x_buckRegDataIndex
 *  \param      buckNum             [IN]    Indicates which Buck the API is working with
 * 
 *  \return     Success code if all Buck PMIC registers were read and the data has been
 *              stored in \p regData parameter, error code otherwise. For valid success/error
 *              codes, refer to \ref Pmic_ErrorCodes
 */
static int32_t tps6522xReadBuckRegs(Pmic_CoreHandle_t *pPmicCoreHandle, uint8_t *regData, const uint8_t buckNum)
{
    int32_t status = PMIC_ST_SUCCESS;

    // As we are about to read, start critical section
    Pmic_criticalSectionStart(pPmicCoreHandle);

    // Read BUCK_CTRL
    status = Pmic_commIntf_recvByte(pPmicCoreHandle, 
                                    gTps6522xBuckRegisters[buckNum].buckCtrlRegAddr, 
                                    &(regData[TPS6522X_BUCK_CTRL_REGDATA_INDEX]));

    // Read BUCK_CONF
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_commIntf_recvByte(pPmicCoreHandle, 
                                        gTps6522xBuckRegisters[buckNum].buckConfRegAddr, 
                                        &(regData[TPS6522X_BUCK_CONF_REGDATA_INDEX]));
    }

    // Read BUCK_VOUT
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_commIntf_recvByte(pPmicCoreHandle, 
                                        gTps6522xBuckRegisters[buckNum].buckVoutRegAddr, 
                                        &(regData[TPS6522X_BUCK_VOUT_REGDATA_INDEX]));
    }
    
    // Read BUCK_PG_WINDOW
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_commIntf_recvByte(pPmicCoreHandle, 
                                        gTps6522xBuckRegisters[buckNum].buckPgWindowRegAddr, 
                                        &(regData[TPS6522X_BUCK_PG_WINDOW_REGDATA_INDEX]));
    }

    // Read RAIL_SEL_1
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_commIntf_recvByte(pPmicCoreHandle, 
                                        gTps6522xBuckRegisters[buckNum].buckRailSelRegAddr, 
                                        &(regData[TPS6522X_BUCK_RAIL_SEL_REGDATA_INDEX]));
    }

    // After read, stop critical section
    Pmic_criticalSectionStop(pPmicCoreHandle);

    return status;
}

/**
 *  \brief      This function is used to extract relevant Buck information from
 *              the \p regData parameter - relevant information is governed by 
 *              the validParams of the \p pBuckCfg parameter. 
 * 
 *  \param      pBuckCfg    [IN/OUT]    Buck power resource configuration. The Buck 
 *                                      information will be stored in this struct
 *  \param      regData     [IN]        Array of PMIC register data. For valid indices
 *                                      of the array, see \ref Tps6522x_buckRegDataIndex
 *  \param      buckNum     [IN]        Indicates which Buck the API is working with
 */
static inline void tps6522xGetBuckInfo(tps6522xBuckCfg_t *pBuckCfg, const uint8_t *regData, const uint8_t buckNum)
{
    // BUCK_CTRL register
    tps6522xGetBuckCtrlRegBitFields(pBuckCfg, regData[TPS6522X_BUCK_CTRL_REGDATA_INDEX]);

    // BUCK_CONF register
    tps6522xGetBuckConfRegBitFields(pBuckCfg, regData[TPS6522X_BUCK_CONF_REGDATA_INDEX]);

    // BUCK_VOUT register
    tps6522xGetBuckVoutRegBitFields(pBuckCfg, regData[TPS6522X_BUCK_VOUT_REGDATA_INDEX], buckNum);

    // BUCK_PG_WINDOW register
    tps6522xGetBuckPgWindowRegBitFields(pBuckCfg, regData[TPS6522X_BUCK_PG_WINDOW_REGDATA_INDEX]);

    // RAIL_SEL_1 register
    Pmic_PowerTps6522xGetRailSel1RegBitFields(pBuckCfg, regData[TPS6522X_BUCK_RAIL_SEL_REGDATA_INDEX], buckNum);
}

int32_t tps6522xGetBuckCfg(Pmic_CoreHandle_t *pPmicCoreHandle, tps6522xBuckCfg_t *pBuckCfg, const uint8_t buckNum)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData[TPS6522X_BUCK_REGDATA_NUM_INDICES] = {0U};

    // Parameter check
    if (pBuckCfg == NULL)
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }
    if ((status == PMIC_ST_SUCCESS) && (pBuckCfg->validParams == 0U))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    // Read all registers that are relevant to the BUCK power resource
    if (status == PMIC_ST_SUCCESS)
    {
        status = tps6522xReadBuckRegs(pPmicCoreHandle, regData, buckNum);
    }

    // Get the desired information (bit fields) from the registers 
    // that were read (desired bit fields are governed by validParams)
    if (status == PMIC_ST_SUCCESS)
    {
        tps6522xGetBuckInfo(pBuckCfg, regData, buckNum);
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
 *  \brief      This function is used to get the desired bit fields of the LDO_VOUT register
 *              (desired bit fields are governed by validParams).
 *
 *  \param      pLdoCfg             [IN/OUT]    Pointer to LDO power resource configuration struct
 *  \param      ldoVoutRegData      [IN]        LDO_VOUT register data obtained from a prior read of the PMIC
 */
static void tps6522xGetLdoVoutRegBitFields(tps6522xLdoCfg_t *pLdoCfg, 
                                           const uint8_t ldoVoutRegData)
{
    if (pmic_validParamCheck(pLdoCfg->validParams, TPS6522X_LDO_MODE_VALID))
    {
        pLdoCfg->ldoMode = Pmic_getBitField(
            ldoVoutRegData, TPS6522X_LDO_MODE_SHIFT, TPS6522X_LDO_MODE_MASK);
    }
    if (pmic_validParamCheck(pLdoCfg->validParams, TPS6522X_LDO_VSET_VALID))
    {
        pLdoCfg->ldoVset = Pmic_getBitField(
            ldoVoutRegData, TPS6522X_LDO_VSET_SHIFT, TPS6522X_LDO_VSET_MASK);
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

/**
 *  \brief      This function is used to read all PMIC registers relevant to a LDO
 *              power resource and stores the data into the \p regData parameter. 
 *
 *  \param      pPmicCoreHandle     [IN]        PMIC interface handle
 *  \param      regData             [OUT]       Array of PMIC register data. For valid indices
 *                                              of the array, see \ref Tps6522x_ldoRegDataIndex
 *  \param      ldoNum              [IN]        Indicates which LDO the API is working with 
 * 
 *  \return     Success code if all LDO PMIC registers were read and the data has been
 *              stored in \p regData parameter, error code otherwise. For valid success/error
 *              codes, refer to \ref Pmic_ErrorCodes
 */
static int32_t tps6522xReadLdoRegs(Pmic_CoreHandle_t *pPmicCoreHandle, uint8_t *regData, const uint8_t ldoNum)
{
    int32_t status = PMIC_ST_SUCCESS;

    // As we are about to read, start critical section
    Pmic_criticalSectionStart(pPmicCoreHandle);

    // Read LDO_CTRL
    status = Pmic_commIntf_recvByte(pPmicCoreHandle, 
                                    gTps6522xLdoRegisters[ldoNum].ldoCtrlRegAddr, 
                                    &(regData[TPS6522X_LDO_CTRL_REGDATA_INDEX]));

    // Read LDO_VOUT
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_commIntf_recvByte(pPmicCoreHandle, 
                                        gTps6522xLdoRegisters[ldoNum].ldoVoutRegAddr, 
                                        &(regData[TPS6522X_LDO_VOUT_REGDATA_INDEX]));
    }

    // Read LDO_PG_WINDOW
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_commIntf_recvByte(pPmicCoreHandle, 
                                        gTps6522xLdoRegisters[ldoNum].ldoPgWindowRegAddr, 
                                        &(regData[TPS6522X_LDO_PG_WINDOW_REGDATA_INDEX]));
    }
    
    // Read RAIL_SEL_2
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_commIntf_recvByte(pPmicCoreHandle, 
                                        gTps6522xLdoRegisters[ldoNum].ldoRailSelRegAddr, 
                                        &(regData[TPS6522X_LDO_RAIL_SEL_REGDATA_INDEX]));
    }

    // After read, stop critical section
    Pmic_criticalSectionStop(pPmicCoreHandle);

    return status;
}

/**
 *  \brief      This function is used to extract relevant LDO information from
 *              the \p regData parameter - relevant information is governed by 
 *              the validParams of the \p pLdoCfg parameter. 
 * 
 *  \param      pLdoCfg     [IN/OUT]    LDO power resource configuration. The LDO
 *                                      information will be stored in this struct
 *  \param      regData     [IN]        Array of PMIC register data. For valid indices
 *                                      of the array, see \ref Tps6522x_LdoRegDataIndex
 *  \param      ldoNum      [IN]        Indicates which Buck the API is working with 
 */
static inline void tps6522xGetLdoInfo(tps6522xLdoCfg_t *pLdoCfg, const uint8_t *regData, const uint8_t ldoNum)
{
    // LDO_CTRL register
    tps6522xGetLdoCtrlRegBitFields(pLdoCfg, regData[TPS6522X_LDO_CTRL_REGDATA_INDEX]);

    // LDO_VOUT register
    tps6522xGetLdoVoutRegBitFields(pLdoCfg, regData[TPS6522X_LDO_VOUT_REGDATA_INDEX]);

    // LDO_PG_WINDOW register
    tps6522xGetLdoPgWindowRegBitFields(pLdoCfg, regData[TPS6522X_LDO_PG_WINDOW_REGDATA_INDEX]);

    // RAIL_SEL_2 register
    tps6522xGetRailSel2RegBitFields(pLdoCfg, regData[TPS6522X_LDO_RAIL_SEL_REGDATA_INDEX], ldoNum);
}

int32_t tps6522xGetLdoCfg(Pmic_CoreHandle_t *pPmicCoreHandle, tps6522xLdoCfg_t *pLdoCfg, const uint8_t ldoNum)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData[TPS6522X_LDO_REGDATA_NUM_INDICES] = {0};

    // Parameter check
    if (pLdoCfg == NULL)
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }
    if ((status == PMIC_ST_SUCCESS) && (pLdoCfg->validParams == 0U))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    // Read all registers that are relevant to the LDO power resource
    if (status == PMIC_ST_SUCCESS)
    {
        status = tps6522xReadLdoRegs(pPmicCoreHandle, regData, ldoNum);
    }

    // Get the desired information (bit fields) from the registers 
    // that were read (desired bit fields are governed by validParams)
    if (status == PMIC_ST_SUCCESS)
    {
        tps6522xGetLdoInfo(pLdoCfg, regData, ldoNum);
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
    if (pmic_validParamCheck(pVccaVmonPwrRsrcCfg->validParams, TPS6522X_VCCA_PG_SET_VALID))
    {
        pVccaVmonPwrRsrcCfg->vccaPgSet = Pmic_getBitField(
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
 *  \brief      This function is used to store desired bit fields of the VMON_PG_LEVEL register into the
 *              VCCA/VMON power resource CFG struct (desired bit fields are governed by validParams).
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
        pmic_validParamCheck(pVccaVmonPwrRsrcCfg->validParams, TPS6522X_VMON1_PG_SET_VALID))
    {
        pVccaVmonPwrRsrcCfg->vmon1PgSet = Pmic_getBitField(
            vmonPgLevelRegData, TPS6522X_VMON1_PG_SET_SHIFT, TPS6522X_VMON1_PG_SET_MASK);
    }
    else if ((vmonNum == TPS6522X_VOLTAGE_MONITOR_VMON2) &&
             pmic_validParamCheck(pVccaVmonPwrRsrcCfg->validParams, TPS6522X_VMON2_PG_SET_VALID))
    {
        pVccaVmonPwrRsrcCfg->vmon2PgSet = Pmic_getBitField(
            vmonPgLevelRegData, TPS6522X_VMON2_PG_SET_SHIFT, TPS6522X_VMON2_PG_SET_MASK);
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

/**
 *  \brief      This function is used to read all PMIC registers relevant to a 
 *              VCCA_VMON/VMONx power resource and stores the data into the 
 *              \p regData parameter. 
 * 
 *  \param      pPmicCoreHandle     [IN]    PMIC interface handle
 *  \param      regData             [OUT]   Array of PMIC register data. For valid indices
 *                                          of the array, see \ref Tps6522x_vccaVmonRegDataIndex
 *  \param      vmonNum             [IN]    Indicates which VMON the API is working with
 * 
 *  \return     Success code if all VCCA_VMON/VMONx PMIC registers were read and the data 
 *              has been stored in \p regData parameter, error code otherwise. For valid 
 *              success/error codes, refer to \ref Pmic_ErrorCodes
 */
static int32_t tps6522xReadVccaVmonRegs(Pmic_CoreHandle_t *pPmicCoreHandle, uint8_t *regData, const uint8_t vmonNum)
{
    int32_t status = PMIC_ST_SUCCESS;

    Pmic_criticalSectionStart(pPmicCoreHandle);

    // Read VCCA_VMON_CTRL
    status = Pmic_commIntf_recvByte(pPmicCoreHandle, 
                                    gTps6522xVccaVmonRegisters[vmonNum].vccaVmonCtrlRegAddr, 
                                    &(regData[TPS6522X_VCCA_VMON_CTRL_REGDATA_INDEX]));

    // Read VCCA_PG_WINDOW 
    if ((status == PMIC_ST_SUCCESS) && (vmonNum == TPS6522X_VOLTAGE_MONITOR_VCCA_VMON))
    {
        status = Pmic_commIntf_recvByte(pPmicCoreHandle, 
                                        gTps6522xVccaVmonRegisters[vmonNum].vccaPgWindowRegAddr, 
                                        &(regData[TPS6522X_VCCA_PG_WINDOW_REGDATA_INDEX]));
    }

    // Read VMON_PG_WINDOW
    if ((status == PMIC_ST_SUCCESS) && ((vmonNum == TPS6522X_VOLTAGE_MONITOR_VMON1) ||
                                        (vmonNum == TPS6522X_VOLTAGE_MONITOR_VMON2)))
    {
        status = Pmic_commIntf_recvByte(pPmicCoreHandle, 
                                        gTps6522xVccaVmonRegisters[vmonNum].vmonPgWindowRegAddr,
                                        &(regData[TPS6522X_VMON_PG_WINDOW_REGDATA_INDEX]));
    }

    // Read VMON_PG_LEVEL
    if ((status == PMIC_ST_SUCCESS) && ((vmonNum == TPS6522X_VOLTAGE_MONITOR_VMON1) ||
                                        (vmonNum == TPS6522X_VOLTAGE_MONITOR_VMON2)))
    {
        status = Pmic_commIntf_recvByte(pPmicCoreHandle, 
                                        gTps6522xVccaVmonRegisters[vmonNum].vmonPgLevelRegAddr,
                                        &(regData[TPS6522X_VMON_PG_LEVEL_REGDATA_INDEX]));
    }
    
    // Read RAIL_SEL_3
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_commIntf_recvByte(pPmicCoreHandle, 
                                        gTps6522xVccaVmonRegisters[vmonNum].vccaVmonRailSelRegAddr, 
                                        &(regData[TPS6522X_VCCA_VMON_RAIL_SEL_REGDATA_INDEX]));
    }

    Pmic_criticalSectionStop(pPmicCoreHandle);

    return status;
}

/**
 *  \brief      This function is used to extract relevant VCCA_VMON/VMONx information 
 *              from the \p regData parameter - relevant information is governed by 
 *              the validParams of the \p pVccaVmonPwrRsrcCfg parameter. 
 * 
 *  \param      pVccaVmonPwrRsrcCfg     [IN/OUT]        VCCA_VMON/VMONx power resource configuration. The  
 *                                                      VMON information will be stored in this struct
 *  \param      regData                 [IN]            Array of PMIC register data. For valid indices
 *                                                      of the array, see \ref Tps6522x_vccaVmonRegDataIndex
 *  \param      vmonNum                 [IN]            Indicates which VMON the API is working with
 */
static inline void tps6522xGetVccaVmonInfo(
    tps6522xVccaVmonCfg_t *pVccaVmonPwrRsrcCfg, const uint8_t *regData, const uint8_t vmonNum)
{
    // VCCA_VMON_CTRL
    tps6522xGetVccaVmonCtrlBitFields(pVccaVmonPwrRsrcCfg, regData[TPS6522X_VCCA_VMON_CTRL_REGDATA_INDEX], vmonNum);

    // VCCA_PG_WINDOW
    tps6522xGetVccaPgWindowBitFields(pVccaVmonPwrRsrcCfg, regData[TPS6522X_VCCA_PG_WINDOW_REGDATA_INDEX]);

    // VMON_PG_WINDOW
    tps6522xGetVmonPgWindowBitFields(pVccaVmonPwrRsrcCfg, regData[TPS6522X_VMON_PG_WINDOW_REGDATA_INDEX], vmonNum);

    // VMON_PG_LEVEL
    tps6522xGetVmonPgLevelBitFields(pVccaVmonPwrRsrcCfg, regData[TPS6522X_VMON_PG_LEVEL_REGDATA_INDEX], vmonNum);

    // RAIL_SEL_3
    tps6522xGetRailSel3RegBitFields(pVccaVmonPwrRsrcCfg, regData[TPS6522X_VCCA_VMON_RAIL_SEL_REGDATA_INDEX], vmonNum);
}

int32_t tps6522xGetVccaVmonCfg(Pmic_CoreHandle_t *pPmicCoreHandle,
                               tps6522xVccaVmonCfg_t *pVccaVmonPwrRsrcCfg,
                               const uint8_t vmonNum)
{
    uint8_t regData[TPS6522X_VCCA_VMON_REGDATA_NUM_INDICES] = {0};
    int32_t status = PMIC_ST_SUCCESS;

    // Parameter check
    if (pVccaVmonPwrRsrcCfg == NULL)
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }
    if ((status == PMIC_ST_SUCCESS) && (pVccaVmonPwrRsrcCfg->validParams == 0U))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    // Read all registers relevant to VCCA_VMON, VMON1, and VMON2
    if (status == PMIC_ST_SUCCESS)
    {
        status = tps6522xReadVccaVmonRegs(pPmicCoreHandle, regData, vmonNum);
    }

    // Get the desired information (bit fields) from the registers 
    // that were read (desired bit fields are governed by validParams)
    if (status == PMIC_ST_SUCCESS)
    {
        tps6522xGetVccaVmonInfo(pVccaVmonPwrRsrcCfg, regData, vmonNum);
    }

    return status;
}

/**
 *  \brief      This function is used to get the configurations of valid Bucks. 
 *              Valid Bucks are specified by the end-user via the validParams 
 *              of the \p pPwrRsrcCfg parameter.
 * 
 *  \param      pPmicCoreHandle     [IN]        PMIC interface handle
 *  \param      pPwrRsrcCfg         [OUT]       Power resource configuration struct
 *                                              in which all valid Buck configurations
 *                                              will be stored in
 * 
 *  \return     Success code if the configurations of valid Bucks were obtained
 *              and stored in the \p pPwrRsrcCfg parameter, error code otherwise. 
 *              For valid success/error codes, refer to \ref Pmic_ErrorCodes
 */
static int32_t tps6522xGetAllValidBuckCfg(Pmic_CoreHandle_t *pPmicCoreHandle, tps6522xPwrRsrcCfg_t *pPwrRsrcCfg)
{
    uint8_t iter = 0;
    uint8_t validParam = 0;
    int32_t status = PMIC_ST_SUCCESS;

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

    return status;
}

/**
 *  \brief      This function is used to get the configurations of valid LDOs. 
 *              Valid LDOs are specified by the end-user via the validParams 
 *              of the \p pPwrRsrcCfg parameter.
 * 
 *  \param      pPmicCoreHandle     [IN]        PMIC interface handle
 *  \param      pPwrRsrcCfg         [OUT]       Power resource configuration struct
 *                                              in which all valid LDO configurations
 *                                              will be stored in
 * 
 *  \return     Success code if the configurations of valid LDOs were obtained
 *              and stored in the \p pPwrRsrcCfg parameter, error code otherwise. 
 *              For valid success/error codes, refer to \ref Pmic_ErrorCodes
 */
static int32_t tps6522xGetAllValidLdoCfg(Pmic_CoreHandle_t *pPmicCoreHandle, tps6522xPwrRsrcCfg_t *pPwrRsrcCfg)
{
    uint8_t iter = 0;
    uint8_t validParam = 0;
    int32_t status = PMIC_ST_SUCCESS;

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

    return status;
}

/**
 *  \brief      This function is used to get the configurations of valid VMONs
 *              (VCCA_VMON/VMONx). Valid VMONs are specified by the end-user via
 *              the validParams of the \p pPwrRsrcCfg parameter.
 * 
 *  \param      pPmicCoreHandle     [IN]        PMIC interface handle
 *  \param      pPwrRsrcCfg         [OUT]       Power resource configuration struct
 *                                              in which all valid VMON configurations
 *                                              will be stored in
 * 
 *  \return     Success code if the configurations of valid VMONs were obtained
 *              and stored in the \p pPwrRsrcCfg parameter, error code otherwise. 
 *              For valid success/error codes, refer to \ref Pmic_ErrorCodes
 */
static int32_t tps6522xGetAllValidVccaVmonCfg(Pmic_CoreHandle_t *pPmicCoreHandle, 
                                                    tps6522xPwrRsrcCfg_t *pPwrRsrcCfg)
{
    uint8_t iter = 0;
    uint8_t validParam = 0;
    int32_t status = PMIC_ST_SUCCESS;

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

    return status;
}

int32_t tps6522xGetPwrRsrcCfg(Pmic_CoreHandle_t *pPmicCoreHandle, tps6522xPwrRsrcCfg_t *pPwrRsrcCfg)
{
    int32_t status = PMIC_ST_SUCCESS;

    status = tps6522xParamCheck_pPwrRsrcCfg(pPmicCoreHandle, pPwrRsrcCfg);

    if (status == PMIC_ST_SUCCESS)
    {
        status = tps6522xGetAllValidBuckCfg(pPmicCoreHandle, pPwrRsrcCfg);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        status = tps6522xGetAllValidLdoCfg(pPmicCoreHandle, pPwrRsrcCfg);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        status = tps6522xGetAllValidVccaVmonCfg(pPmicCoreHandle, pPwrRsrcCfg);
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
 *  \brief      This function is used to set desired bit fields of the BUCK_VOUT register (desired bit fields
 *              are governed by validParams).
 *
 *  \param      buckCfg                 [IN]        BUCK power resource configuration struct
 *  \param      pBuckVoutRegData        [OUT]       Pointer to BUCK_VOUT register data
 *  \param      buckNum                 [IN]        Indicates which BUCK the API is working with
 */
static void tps6522xSetBuckVoutRegBitFields(const tps6522xBuckCfg_t buckCfg,
                                            uint8_t *pBuckVoutRegData,
                                            const uint8_t buckNum)
{
    if (pmic_validParamCheck(buckCfg.validParams, TPS6522X_BUCK_VSET_VALID))
    {
        switch (buckNum)
        {
            case TPS6522X_REGULATOR_BUCK1:
                if ((buckCfg.buckVset >= TPS6522X_BUCK1_MIN_VSET) && 
                    (buckCfg.buckVset <= TPS6522X_BUCK1_MAX_VSET)) 
                {
                    Pmic_setBitField(pBuckVoutRegData, 
                                    TPS6522X_BUCK_VSET_SHIFT, 
                                    TPS6522X_BUCK1_VSET_MASK, 
                                    buckCfg.buckVset);
                }
                break;
            case TPS6522X_REGULATOR_BUCK2:
            case TPS6522X_REGULATOR_BUCK3:
            case TPS6522X_REGULATOR_BUCK4:
                // The min BUCK2, BUCK3, and BUCK4 VSET is 0, 
                // hence we do not need to check for a uint8_t >= 0
                if (buckCfg.buckVset <= TPS6522X_BUCK2_3_4_MAX_VSET)
                {
                    Pmic_setBitField(pBuckVoutRegData, 
                                    TPS6522X_BUCK_VSET_SHIFT, 
                                    TPS6522X_BUCK2_3_4_VSET_MASK, 
                                    buckCfg.buckVset);
                }
                break;
            default:
                /* Invalid regulator number */
                break;
        }
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

/**
 *  \brief      This function is used to modify all Buck data supplied by the 
 *              \p regData parameter. \p regData will be modified in accordance
 *              to the struct members of the \p buckCfg parameter. 
 * 
 *  \param      buckCfg     [IN]    Buck power resource configuration struct
 *  \param      regData     [OUT]   Array of modified Buck data. For valid indices,
 *                                  refer to \ref Tps6522x_buckRegDataIndex
 *  \param      buckNum     [IN]    Indicates which Buck the API is working with 
 */
static inline void tps6522xModifyBuckData(const tps6522xBuckCfg_t buckCfg, uint8_t *regData, const uint8_t buckNum)
{
    // BUCK_CTRL register
    tps6522xSetBuckCtrlRegBitFields(buckCfg, &(regData[TPS6522X_BUCK_CTRL_REGDATA_INDEX]));

    // BUCK_CONF register
    tps6522xSetBuckConfRegBitFields(buckCfg, &(regData[TPS6522X_BUCK_CONF_REGDATA_INDEX]));

    // BUCK_VOUT register
    tps6522xSetBuckVoutRegBitFields(buckCfg, &(regData[TPS6522X_BUCK_VOUT_REGDATA_INDEX]), buckNum);

    // BUCK_PG_WINDOW register
    tps6522xSetBuckPgWindowRegBitFields(buckCfg, &(regData[TPS6522X_BUCK_PG_WINDOW_REGDATA_INDEX]));

    // RAIL_SEL_1 register
    tps6522xSetRailSel1RegBitFields(buckCfg, &(regData[TPS6522X_BUCK_RAIL_SEL_REGDATA_INDEX]), buckNum);
}

/**
 *  \brief      This function is used to write Buck register data \p regData to the PMIC. 
 * 
 *  \param      pPmicCoreHandle     [IN]    PMIC interface handle
 *  \param      regData             [IN]    Register data to be written to the PMIC. 
 *                                          For valid indices, refer to \ref Tps6522x_buckRegDataIndex
 *  \param      buckNum             [IN]    Indicates which Buck the API is working with
 * 
 *  \return     Success code if the data was written to the PMIC, error code otherwise.
 *              For valid success/error codes, refer to \ref Pmic_ErrorCodes
 */
static int32_t tps6522xWriteBuckData(Pmic_CoreHandle_t *pPmicCoreHandle, const uint8_t *regData, const uint8_t buckNum)
{
    int32_t status = PMIC_ST_SUCCESS;

    Pmic_criticalSectionStart(pPmicCoreHandle);

    // Write to BUCK_CONF
    status = Pmic_commIntf_sendByte(pPmicCoreHandle, 
                                    gTps6522xBuckRegisters[buckNum].buckConfRegAddr, 
                                    regData[TPS6522X_BUCK_CONF_REGDATA_INDEX]);

    // Write to BUCK_VOUT
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_commIntf_sendByte(pPmicCoreHandle, 
                                        gTps6522xBuckRegisters[buckNum].buckVoutRegAddr, 
                                        regData[TPS6522X_BUCK_VOUT_REGDATA_INDEX]);
    }

    // Write to BUCK_PG_WINDOW
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_commIntf_sendByte(pPmicCoreHandle, 
                                        gTps6522xBuckRegisters[buckNum].buckPgWindowRegAddr, 
                                        regData[TPS6522X_BUCK_PG_WINDOW_REGDATA_INDEX]);
    }

    // Write to RAIL_SEL_1
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_commIntf_sendByte(pPmicCoreHandle, 
                                        gTps6522xBuckRegisters[buckNum].buckRailSelRegAddr, 
                                        regData[TPS6522X_BUCK_RAIL_SEL_REGDATA_INDEX]);
    }

    // Write to BUCK_CTRL
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_commIntf_sendByte(pPmicCoreHandle, 
                                        gTps6522xBuckRegisters[buckNum].buckCtrlRegAddr, 
                                        regData[TPS6522X_BUCK_CTRL_REGDATA_INDEX]);
    }

    Pmic_criticalSectionStop(pPmicCoreHandle);

    return status;
}

int32_t tps6522xSetBuckCfg(Pmic_CoreHandle_t *pPmicCoreHandle, const tps6522xBuckCfg_t buckCfg, const uint8_t buckNum)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData[TPS6522X_BUCK_REGDATA_NUM_INDICES] = {0U};

    // Parameter check
    if (buckCfg.validParams == 0U)
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }
    if (status == PMIC_ST_SUCCESS)
    {
        status = tps6522xBuckVsetWithinRangeCheck(buckCfg, buckNum);
    }

    // Read all registers pertaining to the BUCK resource
    if (status == PMIC_ST_SUCCESS)
    {
        status = tps6522xReadBuckRegs(pPmicCoreHandle, regData, buckNum);
    }

    // modify the desired bit fields of the registers that were 
    // read (desired bit fields are governed by validParams)
    if (status == PMIC_ST_SUCCESS)
    {
        tps6522xModifyBuckData(buckCfg, regData, buckNum);
    }

    // After modification of register bit fields, write values back to PMIC
    if (status == PMIC_ST_SUCCESS)
    {
        status = tps6522xWriteBuckData(pPmicCoreHandle, regData, buckNum);
    }

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
 *  \brief      This function is used to set the desired bit fields of the LDO_VOUT register
 *              (desired bit fields are governed by validParams).
 *
 *  \param      ldoCfg                  [IN]        LDO power resource configuration struct
 *  \param      pLdoVoutRegData         [OUT]       Pointer to LDO_VOUT register data
 *  \param      ldoNum                  [IN]        Indicates which LDO the API is working with
 */
static void tps6522xSetLdoVoutRegBitFields(const tps6522xLdoCfg_t ldoCfg, uint8_t *pLdoVoutRegData)
{
    if (pmic_validParamCheck(ldoCfg.validParams, TPS6522X_LDO_MODE_VALID))
    {
        Pmic_setBitField(pLdoVoutRegData, TPS6522X_LDO_MODE_SHIFT, TPS6522X_LDO_MODE_MASK, ldoCfg.ldoMode);
    }

    // The min LDO1, LDO2, and LDO3 VSET is 0, hence we do not need to check for a uint8_t >= 0
    if (pmic_validParamCheck(ldoCfg.validParams, TPS6522X_LDO_VSET_VALID) && 
        (ldoCfg.ldoVset <= TPS6522X_LDO1_2_3_MAX_VSET))
    {
        Pmic_setBitField(pLdoVoutRegData, TPS6522X_LDO_VSET_SHIFT, TPS6522X_LDO_VSET_MASK, ldoCfg.ldoVset);
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

/**
 *  \brief      This function is used to modify all LDO data supplied by the 
 *              \p regData parameter. \p regData will be modified in accordance
 *              to the struct members of the \p ldoCfg parameter. 
 * 
 *  \param      ldoCfg      [IN]    LDO power resource configuration struct
 *  \param      regData     [OUT]   Array of modified LDO data. For valid indices,
 *                                  refer to \ref Tps6522x_ldoRegDataIndex
 *  \param      ldoNum      [IN]    Indicates which LDO the API is working with 
 */
static inline void tps6522xModifyLdoData(const tps6522xLdoCfg_t ldoCfg, uint8_t *regData, const uint8_t ldoNum)
{
    // LDO_CTRL register
    tps6522xSetLdoCtrlRegBitFields(ldoCfg, &(regData[TPS6522X_LDO_CTRL_REGDATA_INDEX]));

    // LDO_VOUT register
    tps6522xSetLdoVoutRegBitFields(ldoCfg, &(regData[TPS6522X_LDO_VOUT_REGDATA_INDEX]));

    // LDO_PG_WINDOW register
    tps6522xSetLdoPgWindowRegBitFields(ldoCfg, &(regData[TPS6522X_LDO_PG_WINDOW_REGDATA_INDEX]));

    // RAIL_SEL_2 register
    tps6522xSetRailSel2RegBitFields(ldoCfg, &(regData[TPS6522X_LDO_RAIL_SEL_REGDATA_INDEX]), ldoNum);
}

/**
 *  \brief      This function is used to write LDO register data \p regData to the PMIC. 
 * 
 *  \param      pPmicCoreHandle     [IN]    PMIC interface handle
 *  \param      regData             [IN]    Register data to be written to the PMIC 
 *                                          For valid indices, refer to \ref Tps6522x_ldoRegDataIndex
 *  \param      ldoNum              [IN]    Indicates which LDO the API is working with
 * 
 *  \return     Success code if the data was written to the PMIC, error code otherwise.
 *              For valid success/error codes, refer to \ref Pmic_ErrorCodes
 */
static int32_t tps6522xWriteLdoData(Pmic_CoreHandle_t *pPmicCoreHandle, const uint8_t *regData, const uint8_t ldoNum)
{
    int32_t status = PMIC_ST_SUCCESS;

    Pmic_criticalSectionStart(pPmicCoreHandle);

    // Write to LDO_VOUT
    status = Pmic_commIntf_sendByte(pPmicCoreHandle, 
                                    gTps6522xLdoRegisters[ldoNum].ldoVoutRegAddr, 
                                    regData[TPS6522X_LDO_VOUT_REGDATA_INDEX]);

    // Write to LDO_PG_WINDOW
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_commIntf_sendByte(pPmicCoreHandle, 
                                        gTps6522xLdoRegisters[ldoNum].ldoPgWindowRegAddr, 
                                        regData[TPS6522X_LDO_PG_WINDOW_REGDATA_INDEX]);
    }

    // Write to RAIL_SEL_2
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_commIntf_sendByte(pPmicCoreHandle, 
                                        gTps6522xLdoRegisters[ldoNum].ldoRailSelRegAddr, 
                                        regData[TPS6522X_LDO_RAIL_SEL_REGDATA_INDEX]);
    }

    // Write to LDO_CTRL
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_commIntf_sendByte(pPmicCoreHandle, 
                                        gTps6522xLdoRegisters[ldoNum].ldoCtrlRegAddr, 
                                        regData[TPS6522X_LDO_CTRL_REGDATA_INDEX]);
    }

    Pmic_criticalSectionStop(pPmicCoreHandle);

    return status;
}

int32_t tps6522xSetLdoCfg(Pmic_CoreHandle_t *pPmicCoreHandle, const tps6522xLdoCfg_t ldoCfg, const uint8_t ldoNum)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData[TPS6522X_LDO_REGDATA_NUM_INDICES] = {0};

    // Parameter check
    if (ldoCfg.validParams == 0U)
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }
    if (status == PMIC_ST_SUCCESS)
    {
        status = tps6522xLdoVsetWithinRangeCheck(ldoCfg);
    }

    // As we are about to read, start critical section
    Pmic_criticalSectionStart(pPmicCoreHandle);

    // Read all registers pertaining to the LDO resource
    if (status == PMIC_ST_SUCCESS)
    {
        status = tps6522xReadLdoRegs(pPmicCoreHandle, regData, ldoNum);
    }

    // modify the desired bit fields of the registers that were 
    // read (desired bit fields are governed by validParams)
    if (status == PMIC_ST_SUCCESS)
    {
        tps6522xModifyLdoData(ldoCfg, regData, ldoNum);
    }

    // After modification of register bit fields, write values back to PMIC
    if (status == PMIC_ST_SUCCESS)
    {
        status = tps6522xWriteLdoData(pPmicCoreHandle, regData, ldoNum);
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
    if (pmic_validParamCheck(vccaVmonCfg.validParams, TPS6522X_VCCA_PG_SET_VALID))
    {
        Pmic_setBitField(pVccaPgWindowRegData,
                         TPS6522X_VCCA_PG_SET_SHIFT,
                         TPS6522X_VCCA_PG_SET_MASK,
                         vccaVmonCfg.vccaPgSet);
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
 *  \brief      This function is used to set the desired bit fields of the VMON_PG_LEVEL register
 *              (desired bit fields are governed by validParams).
 *
 *  \param      vccaVmonCfg             [IN]        VCCA/VMON power resource configuration register
 *  \param      pVmonPgLevelRegData     [OUT]       Pointer to VMON_PG_LEVEL register data
 *  \param      vmonNum                 [IN]        Indicates which VCCA/VMONx the API is working with
 */
static void tps6522xSetVmonPgLevelBitFields(const tps6522xVccaVmonCfg_t vccaVmonCfg,
                                            uint8_t *pVmonPgLevelRegData,
                                            const uint8_t vmonNum)
{
    // The max VMON1 PG_SET is 0xFF, hence we do not need to check for a uint8_t <= 0xFF
    if ((vmonNum == TPS6522X_VOLTAGE_MONITOR_VMON1) &&
        pmic_validParamCheck(vccaVmonCfg.validParams, TPS6522X_VMON1_PG_SET_VALID) &&
        (vccaVmonCfg.vmon1PgSet >= TPS6522X_VMON1_MIN_PG_SET))
    {
        Pmic_setBitField(pVmonPgLevelRegData, 
                        TPS6522X_VMON1_PG_SET_SHIFT, 
                        TPS6522X_VMON1_PG_SET_MASK, 
                        vccaVmonCfg.vmon1PgSet);
    }
    // The min VMON2 VSET is 0, hence we do not need to check for a uint8_t >= 0
    else if ((vmonNum == TPS6522X_VOLTAGE_MONITOR_VMON2) &&
             pmic_validParamCheck(vccaVmonCfg.validParams, TPS6522X_VMON2_PG_SET_VALID) && 
             (vccaVmonCfg.vmon2PgSet <= TPS6522X_VMON2_MAX_PG_SET))
    {
        Pmic_setBitField(pVmonPgLevelRegData, 
                        TPS6522X_VMON2_PG_SET_SHIFT, 
                        TPS6522X_VMON2_PG_SET_MASK, 
                        vccaVmonCfg.vmon2PgSet);
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

/**
 *  \brief      This function is used to modify all VMON (VCCA_VMON/VMONx) data supplied 
 *              by the \p regData parameter. \p regData will be modified in accordance
 *              to the struct members of the \p vccaVmonCfg parameter. 
 * 
 *  \param      vccaVmonCfg     [IN]    VMON power resource configuration struct
 *  \param      regData         [OUT]   Array of modified VMON data. For valid indices,
 *                                      refer to \ref Tps6522x_vccaVmonRegDataIndex
 *  \param      vmonNum         [IN]    Indicates which VMON the API is working with 
 */
static inline void tps6522xModifyVccaVmonData(
    const tps6522xVccaVmonCfg_t vccaVmonCfg, uint8_t *regData, const uint8_t vmonNum)
{
    // VCCA_VMON_CTRL
    tps6522xSetVccaVmonCtrlBitFields(vccaVmonCfg, &(regData[TPS6522X_VCCA_VMON_CTRL_REGDATA_INDEX]));

    // VCCA_PG_WINDOW
    tps6522xSetVccaPgWindowBitFields(vccaVmonCfg, &(regData[TPS6522X_VCCA_PG_WINDOW_REGDATA_INDEX]));

    // VMON_PG_WINDOW
    tps6522xSetVmonPgWindowBitFields(vccaVmonCfg, &(regData[TPS6522X_VMON_PG_WINDOW_REGDATA_INDEX]), vmonNum);

    // VMON_PG_LEVEL
    tps6522xSetVmonPgLevelBitFields(vccaVmonCfg, &(regData[TPS6522X_VMON_PG_LEVEL_REGDATA_INDEX]), vmonNum);

    // RAIL_SEL_3
    tps6522xSetRailSel3RegBitFields(vccaVmonCfg, &(regData[TPS6522X_VCCA_VMON_RAIL_SEL_REGDATA_INDEX]), vmonNum);
}

/**
 *  \brief      This function is used to write VMON (VCCA_VMON/VMONx) register data \p regData to the PMIC. 
 * 
 *  \param      pPmicCoreHandle     [IN]    PMIC interface handle
 *  \param      regData             [IN]    Register data to be written to the PMIC. 
 *                                          For valid indices, refer to \ref Tps6522x_vccaVmonRegDataIndex
 *  \param      vmonNum             [IN]    Indicates which VMON the API is working with
 * 
 *  \return     Success code if the data was written to the PMIC, error code otherwise.
 *              For valid success/error codes, refer to \ref Pmic_ErrorCodes
 */
static int32_t tps6522xWriteVccaVmonData(
    Pmic_CoreHandle_t *pPmicCoreHandle, const uint8_t *regData, const uint8_t vmonNum)
{
    int32_t status = PMIC_ST_SUCCESS;

    Pmic_criticalSectionStart(pPmicCoreHandle);

    // Write to VCCA_PG_WINDOW
    if (vmonNum == TPS6522X_VOLTAGE_MONITOR_VCCA_VMON)
    {
        status = Pmic_commIntf_sendByte(pPmicCoreHandle, 
                                        gTps6522xVccaVmonRegisters[vmonNum].vccaPgWindowRegAddr, 
                                        regData[TPS6522X_VCCA_PG_WINDOW_REGDATA_INDEX]);
    }

    // Write to VMON_PG_WINDOW
    if ((status == PMIC_ST_SUCCESS) && ((vmonNum == TPS6522X_VOLTAGE_MONITOR_VMON1) ||
                                        (vmonNum == TPS6522X_VOLTAGE_MONITOR_VMON2)))
    {
        status = Pmic_commIntf_sendByte(pPmicCoreHandle, 
                                        gTps6522xVccaVmonRegisters[vmonNum].vmonPgWindowRegAddr, 
                                        regData[TPS6522X_VMON_PG_WINDOW_REGDATA_INDEX]);
    }

    // Write to VMON_PG_LEVEL
    if ((status == PMIC_ST_SUCCESS) && ((vmonNum == TPS6522X_VOLTAGE_MONITOR_VMON1) ||
                                        (vmonNum == TPS6522X_VOLTAGE_MONITOR_VMON2)))
    {
        status = Pmic_commIntf_sendByte(pPmicCoreHandle, 
                                        gTps6522xVccaVmonRegisters[vmonNum].vmonPgLevelRegAddr, 
                                        regData[TPS6522X_VMON_PG_LEVEL_REGDATA_INDEX]);
    }

    // Write to RAIL_SEL_3
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_commIntf_sendByte(pPmicCoreHandle, 
                                        gTps6522xVccaVmonRegisters[vmonNum].vccaVmonRailSelRegAddr, 
                                        regData[TPS6522X_VCCA_VMON_RAIL_SEL_REGDATA_INDEX]);
    }

    // Write to VCCA_VMON_CTRL
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_commIntf_sendByte(pPmicCoreHandle, 
                                        gTps6522xVccaVmonRegisters[vmonNum].vccaVmonCtrlRegAddr, 
                                        regData[TPS6522X_VCCA_VMON_CTRL_REGDATA_INDEX]);
    }
    
    Pmic_criticalSectionStop(pPmicCoreHandle);

    return status;
}

int32_t tps6522xSetVccaVmonCfg(Pmic_CoreHandle_t *pPmicCoreHandle, 
                               const tps6522xVccaVmonCfg_t vccaVmonCfg,
                               const uint8_t vmonNum)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData[TPS6522X_VCCA_VMON_REGDATA_NUM_INDICES] = {0U};

    // Parameter check
    if (vccaVmonCfg.validParams == 0U)
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }
    if (status == PMIC_ST_SUCCESS)
    {
        status = tps6522xVmonPgSetWithinRangeCheck(vccaVmonCfg, vmonNum);
    }
    
    // Read all registers pertaining to the VCCA_VMON/VMONx resource
    if (status == PMIC_ST_SUCCESS)
    {
        status = tps6522xReadVccaVmonRegs(pPmicCoreHandle, regData, vmonNum);
    }

    // modify the desired bit fields of the registers that were 
    // read (desired bit fields are governed by validParams)
    if (status == PMIC_ST_SUCCESS)
    {
        tps6522xModifyVccaVmonData(vccaVmonCfg, regData, vmonNum);
    }

    // After modification of register bit fields, write values back to PMIC
    if (status == PMIC_ST_SUCCESS)
    {
        status = tps6522xWriteVccaVmonData(pPmicCoreHandle, regData, vmonNum);
    }

    return status;
}

/**
 *  \brief      This function is used to set the configuration of all valid Bucks.
 *              Valid Bucks are specified by the end-user via the \p pwrRsrcCfg
 *              parameter. 
 * 
 *  \param      pPmicCoreHandle     [IN]    PMIC interface handle
 *  \param      pwrRsrcCfg          [IN]    Power resource configuration struct
 *                                          holding all user-specified BUCK information
 * 
 *  \return     Success code if the configuration of all valid Bucks are set, error
 *              code otherwise. For valid success/error codes, refer to \ref Pmic_ErrorCodes
 */
static int32_t tps6522xSetAllValidBuckCfg(Pmic_CoreHandle_t *pPmicCoreHandle,
                                          const tps6522xPwrRsrcCfg_t pwrRsrcCfg)
{
    uint8_t iter = 0;
    uint8_t validParam = 0;
    int32_t status = PMIC_ST_SUCCESS;

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

    return status;
}

/**
 *  \brief      This function is used to set the configuration of all valid LDOs.
 *              Valid LDOs are specified by the end-user via the \p pwrRsrcCfg
 *              parameter. 
 * 
 *  \param      pPmicCoreHandle     [IN]    PMIC interface handle
 *  \param      pwrRsrcCfg          [IN]    Power resource configuration struct
 *                                          holding all user-specified LDO information
 * 
 *  \return     Success code if the configuration of all valid LDOs are set, error
 *              code otherwise. For valid success/error codes, refer to \ref Pmic_ErrorCodes
 */
static int32_t tps6522xSetAllValidLdoCfg(Pmic_CoreHandle_t *pPmicCoreHandle,
                                         const tps6522xPwrRsrcCfg_t pwrRsrcCfg)
{
    uint8_t iter = 0;
    uint8_t validParam = 0;
    int32_t status = PMIC_ST_SUCCESS;

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

    return status;
}

/**
 *  \brief      This function is used to set the configuration of all valid VMONs
 *              (VCCA_VMON/VMONx). Valid VMONs are specified by the end-user via the 
 *              \p pwrRsrcCfg parameter. 
 * 
 *  \param      pPmicCoreHandle     [IN]    PMIC interface handle
 *  \param      pwrRsrcCfg          [IN]    Power resource configuration struct
 *                                          holding all user-specified VMON information
 * 
 *  \return     Success code if the configuration of all valid VMONs are set, error
 *              code otherwise. For valid success/error codes, refer to \ref Pmic_ErrorCodes
 */
static int32_t tps6522xSetAllValidVccaVmonCfg(Pmic_CoreHandle_t *pPmicCoreHandle,
                                              const tps6522xPwrRsrcCfg_t pwrRsrcCfg)
{
    uint8_t iter = 0;
    uint8_t validParam = 0;
    int32_t status = PMIC_ST_SUCCESS;

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

    return status; 
}

int32_t tps6522xSetPwrRsrcCfg(Pmic_CoreHandle_t *pPmicCoreHandle,
                              const tps6522xPwrRsrcCfg_t pwrRsrcCfg)
{
    int32_t status = PMIC_ST_SUCCESS;

    status = tps6522xParamCheck_constPwrRsrcCfg(pPmicCoreHandle, pwrRsrcCfg);

    if (status == PMIC_ST_SUCCESS)
    {
        status = tps6522xSetAllValidBuckCfg(pPmicCoreHandle, pwrRsrcCfg);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        status = tps6522xSetAllValidLdoCfg(pPmicCoreHandle, pwrRsrcCfg);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        status = tps6522xSetAllValidVccaVmonCfg(pPmicCoreHandle, pwrRsrcCfg);
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

static int32_t tps6522xDecipherTwarnStat(Pmic_CoreHandle_t *pPmicCoreHandle, tps6522xThermalStat_t *pThermalStat)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0;

    // If TWARN_STAT validParam set, read STAT_MISC register and extract the TWARN_STAT bit
    if (pmic_validParamCheck(pThermalStat->validParams, PMIC_THERMAL_STAT_WARN_VALID))
    {
        status = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_STAT_MISC_REGADDR, &regData);

        if (status == PMIC_ST_SUCCESS)
        {
            pThermalStat->twarnStat = Pmic_getBitField_b(regData, TPS6522X_TWARN_STAT_SHIFT);
        }
    }

    return status;
}

static int32_t tps6522xDecipherTsdOrdStat(Pmic_CoreHandle_t *pPmicCoreHandle, tps6522xThermalStat_t *pThermalStat)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0;

    // If TWARN_ORD_STAT validParam set, read STAT_MODERATE_ERR register and extract the TSD_ORD_STAT bit
    if (pmic_validParamCheck(pThermalStat->validParams, PMIC_THERMAL_STAT_ORD_SHTDWN_VALID))
    {
        status = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_STAT_MODERATE_ERR_REGADDR, &regData);

        if (status == PMIC_ST_SUCCESS)
        {
            pThermalStat->tsdOrdStat = Pmic_getBitField_b(regData, TPS6522X_TSD_ORD_STAT_SHIFT);
        }
    }

    return status;
}

static int32_t tps6522xDecipherTsdImmStat(Pmic_CoreHandle_t *pPmicCoreHandle, tps6522xThermalStat_t *pThermalStat)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0;

    // If TSD_IMM_STAT validParam set, read STAT_SEVERE_ERR register and extract the TSD_IMM_STAT bit
    if (pmic_validParamCheck(pThermalStat->validParams, PMIC_THERMAL_STAT_IMM_SHTDWN_VALID))
    {
        status = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_STAT_SEVERE_ERR_REGADDR, &regData);

        if (status == PMIC_ST_SUCCESS)
        {
            pThermalStat->tsdImmStat = Pmic_getBitField_b(regData, TPS6522X_TSD_IMM_STAT_SHIFT);
        }
    }

    return status;
}

int32_t tps6522xGetThermalStat(Pmic_CoreHandle_t *pPmicCoreHandle, tps6522xThermalStat_t *pThermalStat)
{
    int32_t status = PMIC_ST_SUCCESS;

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

    if (status == PMIC_ST_SUCCESS)
    {
        status = tps6522xDecipherTwarnStat(pPmicCoreHandle, pThermalStat);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        status = tps6522xDecipherTsdOrdStat(pPmicCoreHandle, pThermalStat);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        status = tps6522xDecipherTsdImmStat(pPmicCoreHandle, pThermalStat);
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
