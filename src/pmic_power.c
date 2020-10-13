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
*   \file    pmic_power.c
*
*   \brief   This file contains the default API's for PMIC power
*            configuration
*
*/

#include <pmic_types.h>
#include <pmic_power.h>

#include <pmic_core_priv.h>

#include <pmic_power_tps6594x_priv.h>
#include <pmic_power_lp8764x_priv.h>

/*!
 * \brief   This function is used to validate BUCK power resource subtype for
 *          PwmMp mode.
 */
static int32_t Pmic_powerValidateBuckPwrRsrcPwmMpMode(uint8_t  pmicDeviceType,
                                                      uint16_t pwrRsrc)
{
    int32_t status = PMIC_ST_SUCCESS;

    switch (pmicDeviceType)
    {
        case PMIC_DEV_LEO_TPS6594X:
            if((pwrRsrc != PMIC_TPS6594X_REGULATOR_BUCK1) &&
                (pwrRsrc != PMIC_TPS6594X_REGULATOR_BUCK3))
            {
                status = PMIC_ST_ERR_INV_PARAM;
            }

            break;
        case PMIC_DEV_HERA_LP8764X:
            if((pwrRsrc != PMIC_LP8764X_REGULATOR_BUCK1) &&
                (pwrRsrc != PMIC_LP8764X_REGULATOR_BUCK3))
            {
                status = PMIC_ST_ERR_INV_PARAM;
            }

            break;
        default:
            status = PMIC_ST_ERR_INV_PARAM;
            break;
    }

    return status;
}

/*!
 * \brief   This function is used to validate LDO power resource subtype for
 *          bypass mode.
 */
static int32_t Pmic_powerValidateLdoPwrRsrcBypassMode(uint8_t  pmicDeviceType,
                                                      uint16_t pwrRsrc)
{
    int32_t status = PMIC_ST_SUCCESS;

    switch (pmicDeviceType)
    {
        case PMIC_DEV_LEO_TPS6594X:
            if((pwrRsrc != PMIC_TPS6594X_REGULATOR_LDO1) &&
               (pwrRsrc != PMIC_TPS6594X_REGULATOR_LDO2) &&
               (pwrRsrc != PMIC_TPS6594X_REGULATOR_LDO3))
            {
                status = PMIC_ST_ERR_INV_PARAM;
            }

            break;
        default:
            status = PMIC_ST_ERR_INV_PARAM;
            break;
    }

    return status;
}
/*!
 * \brief   This function is used to validate the VCCA power resorce type for
 *          the specific PMIC device.
 */
static int32_t Pmic_powerValidateVccaPwrRsrcType(uint8_t pmicDeviceType,
                                                 uint8_t pwrRsrcType)
{
    int32_t status = PMIC_ST_SUCCESS;

    switch (pmicDeviceType)
    {
        case PMIC_DEV_LEO_TPS6594X:
            if(PMIC_TPS6594X_POWER_RESOURCE_TYPE_VCCA != pwrRsrcType)
            {
                status = PMIC_ST_ERR_INV_PARAM;
            }

            break;
        case PMIC_DEV_HERA_LP8764X:
            if(PMIC_LP8764X_POWER_RESOURCE_TYPE_VCCA != pwrRsrcType)
            {
                status = PMIC_ST_ERR_INV_PARAM;
            }

            break;
        default:
            status = PMIC_ST_ERR_INV_PARAM;
            break;
    }

    return status;
}

/*!
 * \brief   This function is used to validate the VMON power resorce type for
 *          the specific PMIC device
 */
static int32_t Pmic_powerValidateVmonPwrRsrcType(uint8_t pmicDeviceType,
                                                 uint8_t pwrRsrcType)
{
    int32_t status = PMIC_ST_SUCCESS;

    switch (pmicDeviceType)
    {
        case PMIC_DEV_HERA_LP8764X:
            if(PMIC_LP8764X_POWER_RESOURCE_TYPE_VMON != pwrRsrcType)
            {
                status = PMIC_ST_ERR_INV_PARAM;
            }

            break;
        default:
            status = PMIC_ST_ERR_INV_PARAM;
            break;
    }

    return status;
}

/*!
 * \brief   This function is used to validate the LDO power resorce type for
 *          the specific PMIC device
 */
static int32_t Pmic_powerValidateLdoPwrRsrcType(uint8_t pmicDeviceType,
                                                uint8_t pwrRsrcType)
{
    int32_t status = PMIC_ST_SUCCESS;

    switch (pmicDeviceType)
    {
        case PMIC_DEV_LEO_TPS6594X:
            if(PMIC_TPS6594X_POWER_RESOURCE_TYPE_LDO != pwrRsrcType)
            {
                status = PMIC_ST_ERR_INV_PARAM;
            }
            break;
        default:
            status = PMIC_ST_ERR_INV_PARAM;
            break;
    }

    return status;
}

/*!
 * \brief   This function is used to validate the BUCK power resorce type for
 *          the specific PMIC device
 */
static int32_t Pmic_powerValidateBuckPwrRsrcType(uint8_t pmicDeviceType,
                                                 uint8_t pwrRsrcType)
{
    int32_t status = PMIC_ST_SUCCESS;

    switch (pmicDeviceType)
    {
        case PMIC_DEV_LEO_TPS6594X:
            if(PMIC_TPS6594X_POWER_RESOURCE_TYPE_BUCK != pwrRsrcType)
            {
                status = PMIC_ST_ERR_INV_PARAM;
            }

            break;
        case PMIC_DEV_HERA_LP8764X:
            if(PMIC_LP8764X_POWER_RESOURCE_TYPE_BUCK != pwrRsrcType)
            {
                status = PMIC_ST_ERR_INV_PARAM;
            }

            break;
        default:
            status = PMIC_ST_ERR_INV_PARAM;
            break;
    }

    return status;
}

/*!
 * \brief   This function is used to validate the BUCK/LDO/VMON  power resorce
 *          type for the specific PMIC device
 */
static int32_t Pmic_powerValidateBuckLdoVmonPwrRsrcType(uint8_t pmicDeviceType,
                                                        uint8_t pwrRsrcType)
{
    int32_t status = PMIC_ST_SUCCESS;

    if((PMIC_ST_SUCCESS !=
        (Pmic_powerValidateBuckPwrRsrcType(pmicDeviceType, pwrRsrcType))) &&
       (PMIC_ST_SUCCESS !=
        (Pmic_powerValidateLdoPwrRsrcType(pmicDeviceType, pwrRsrcType))) &&
       (PMIC_ST_SUCCESS !=
        (Pmic_powerValidateVmonPwrRsrcType(pmicDeviceType, pwrRsrcType))))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    return status;
}

/*!
 * \brief   This function is used to validate the BUCK/VMON  power resorce
 *          type for the specific PMIC device
 */
static int32_t Pmic_powerValidateBuckVmonPwrRsrcType(uint8_t pmicDeviceType,
                                                     uint8_t pwrRsrcType)
{
    int32_t status = PMIC_ST_SUCCESS;

    status = Pmic_powerValidateBuckPwrRsrcType(pmicDeviceType, pwrRsrcType);

    if(PMIC_ST_ERR_INV_PARAM == status)
    {
        status = Pmic_powerValidateVmonPwrRsrcType(pmicDeviceType, pwrRsrcType);
    }

    return status;
}

/*!
 * \brief   This function is used to validate the LDO/VMON  power resorce
 *          type for the specific PMIC device
 */
static int32_t Pmic_powerValidateLdoVmonPwrRsrcType(uint8_t pmicDeviceType,
                                                    uint8_t pwrRsrcType)
{
    int32_t status = PMIC_ST_SUCCESS;

    status = Pmic_powerValidateLdoPwrRsrcType(pmicDeviceType, pwrRsrcType);

    if(PMIC_ST_ERR_INV_PARAM == status)
    {
        status = Pmic_powerValidateVmonPwrRsrcType(pmicDeviceType, pwrRsrcType);
    }

    return status;
}

/*!
 * \brief   This function is used to validate the BUCK/LDO power resorce
 *          type for the specific PMIC device
 */
static int32_t Pmic_powerValidateBuckLdoPwrRsrcType(uint8_t pmicDeviceType,
                                                    uint8_t pwrRsrcType)
{
    int32_t status = PMIC_ST_SUCCESS;

    status = Pmic_powerValidateBuckPwrRsrcType(pmicDeviceType, pwrRsrcType);

    if(PMIC_ST_ERR_INV_PARAM == status)
    {
        status = Pmic_powerValidateLdoPwrRsrcType(pmicDeviceType, pwrRsrcType);
    }

    return status;
}

/*!
 * \brief   This function is used to get validate the power resources and vmon
 *           based on the valid params set.
 */
static int32_t Pmic_powerValidatecfgDataPwrRsrctype(uint32_t validPrm,
                                                    uint16_t pwrRsrc,
                                                    uint8_t  pmicDeviceType)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t pwrRsrcType;

    pwrRsrcType = Pmic_powerGetPwrRsrcType(pwrRsrc);

    switch(validPrm)
    {
        case PMIC_CFG_REGULATOR_VMON_RV_SEL_VALID:
        case PMIC_CFG_REGULATOR_VMON_VOLTAGE_SET_VALID:
            status = Pmic_powerValidateBuckLdoVmonPwrRsrcType(pmicDeviceType,
                                                              pwrRsrcType);
            break;
        case PMIC_CFG_REGULATOR_BUCK_PLDN_EN_VALID:
        case PMIC_CFG_REGULATOR_BUCK_VOUT_SEL_VALID:
        case PMIC_CFG_REGULATOR_BUCK_FPWM_VALID:
        case PMIC_CFG_REGULATOR_BUCK_ILIM_VALID:
        case PMIC_CFG_REGULATOR_BUCK_PWM_MP_VALID:
            status = Pmic_powerValidateBuckPwrRsrcType(pmicDeviceType,
                                                       pwrRsrcType);

            if((PMIC_ST_SUCCESS == status) &&
               (PMIC_CFG_REGULATOR_BUCK_PWM_MP_VALID == validPrm))
            {
                status = Pmic_powerValidateBuckPwrRsrcPwmMpMode(pmicDeviceType,
                                                                pwrRsrc);
            }
            break;
        case PMIC_CFG_REGULATOR_EN_VALID:
            status = Pmic_powerValidateBuckLdoPwrRsrcType(pmicDeviceType,
                                                          pwrRsrcType);
            break;
        case PMIC_CFG_REGULATOR_LDO_SLOW_RAMP_EN_VALID:
        case PMIC_CFG_REGULATOR_LDO_PLDN_SEL_VALID:
        case PMIC_CFG_REGULATOR_LDO_RV_TIMEOUT_SEL_VALID:
        case PMIC_CFG_REGULATOR_LDO_BYPASS_MODE_EN_VALID:
            status = Pmic_powerValidateLdoPwrRsrcType(pmicDeviceType,
                                                      pwrRsrcType);

            if((PMIC_ST_SUCCESS == status) &&
               (PMIC_CFG_REGULATOR_LDO_BYPASS_MODE_EN_VALID == validPrm))
            {
                status = Pmic_powerValidateLdoPwrRsrcBypassMode(pmicDeviceType,
                                                                pwrRsrc);
            }
            break;
        case PMIC_CFG_VCCA_PWR_GOOD_LVL_VALID:
            status = Pmic_powerValidateVccaPwrRsrcType(pmicDeviceType,
                                                       pwrRsrcType);
            break;
        case PMIC_CFG_VMON_RANGE_VALID:
            status = Pmic_powerValidateVmonPwrRsrcType(pmicDeviceType,
                                                       pwrRsrcType);
            break;
        case PMIC_CFG_REGULATOR_BUCK_VMON_SLEW_RATE_VALID:
            status = Pmic_powerValidateBuckVmonPwrRsrcType(pmicDeviceType,
                                                           pwrRsrcType);
            break;
        default :
            status = PMIC_ST_ERR_INV_PARAM;
            break;
    }

    return status;
}

/*!
 * \brief   This function is used to get the PMIC power resources
            register configuration for the specific PMIC device.
 */
static int32_t Pmic_powerGetRsrcRegCfg(uint8_t                  pmicDeviceType,
                                       Pmic_powerRsrcRegCfg_t **pPwrRsrcRegCfg)
{
    int32_t status = PMIC_ST_SUCCESS;

    if(NULL == pPwrRsrcRegCfg)
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if(PMIC_ST_SUCCESS == status)
    {
        switch(pmicDeviceType)
        {
            case PMIC_DEV_LEO_TPS6594X:
                pmic_get_tps6594x_pwrRsrceRegCfg(pPwrRsrcRegCfg);
                break;
            case PMIC_DEV_HERA_LP8764X:
                pmic_get_lp8764x_pwrRsrceRegCfg(pPwrRsrcRegCfg);
                break;
            default:
                status = PMIC_ST_ERR_INV_DEVICE;
                break;
        }
    }

    return status;
}

/*!
 * \brief   This function is used to get the required BitPos and BitMask
 *           of residual voltage checking for BUCK/LDO/VMON pin.
 */
static int32_t Pmic_powerGetRvCheckEnBitPos(uint8_t   pmicDeviceType,
                                            uint16_t  pwrRsrc,
                                            uint8_t  *pBitPos,
                                            uint8_t  *pBitMask)
{
    int32_t status = PMIC_ST_SUCCESS;

    switch (pmicDeviceType)
    {
        case PMIC_DEV_LEO_TPS6594X:
            *pBitPos = PMIC_POWER_RESOURCEX_CTRL_POWER_RESOURCEX_RV_SEL_SHIFT;
            *pBitMask = PMIC_POWER_RESOURCEX_CTRL_POWER_RESOURCEX_RV_SEL_MASK;
            break;
        case PMIC_DEV_HERA_LP8764X:
            if(PMIC_LP8764X_POWER_SOURCE_VMON1 == pwrRsrc)
            {
                 *pBitPos = PMIC_POWER_VCCA_VMON_CTRL_VMON1_RV_SEL_SHIFT;
                 *pBitMask = PMIC_POWER_VCCA_VMON_CTRL_VMON1_RV_SEL_MASK;

            }
            else if(PMIC_LP8764X_POWER_SOURCE_VMON2 == pwrRsrc)
            {
                *pBitPos = PMIC_POWER_VCCA_VMON_CTRL_VMON2_RV_SEL_SHIFT;
                *pBitMask = PMIC_POWER_VCCA_VMON_CTRL_VMON2_RV_SEL_MASK;
            }
            else
            {
                *pBitPos = \
                        PMIC_POWER_RESOURCEX_CTRL_POWER_RESOURCEX_RV_SEL_SHIFT;
                *pBitMask = \
                        PMIC_POWER_RESOURCEX_CTRL_POWER_RESOURCEX_RV_SEL_MASK;
            }
            break;
        default:
            status = PMIC_ST_ERR_INV_PARAM;
            break;
    }

    return status;
}

/*!
 * \brief   This function is used to Enable/Disable residual voltage checking
 *          for BUCK/LDO/VMON pin.
 */
static int32_t Pmic_powerSetRvCheckEn(Pmic_CoreHandle_t *pPmicCoreHandle,
                                      uint16_t           pwrRsrc,
                                      bool               residualVoltCheck)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    uint8_t bitPos  = 0U;
    uint8_t pwrRsrcIndex = 0U;
    Pmic_powerRsrcRegCfg_t *pPwrRsrcRegCfg = NULL;
    uint8_t bitMask = 0U;

    pwrRsrcIndex = Pmic_powerGetPwrRsrcId(pwrRsrc);

    /* Get PMIC power resources register configuration */
    pmicStatus = Pmic_powerGetRsrcRegCfg(pPmicCoreHandle->pmicDeviceType,
                                         &pPwrRsrcRegCfg);

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        Pmic_criticalSectionStart(pPmicCoreHandle);
        pmicStatus = Pmic_commIntf_recvByte(
                                      pPmicCoreHandle,
                                      pPwrRsrcRegCfg[pwrRsrcIndex].ctrlRegAddr,
                                      &regData);

        if(PMIC_ST_SUCCESS == pmicStatus)
        {
            pmicStatus = Pmic_powerGetRvCheckEnBitPos(
                                                pPmicCoreHandle->pmicDeviceType,
                                                pwrRsrc,
                                                &bitPos,
                                                &bitMask);

            Pmic_setBitField(&regData,
                             bitPos,
                             bitMask,
                             residualVoltCheck);
        }

        if(PMIC_ST_SUCCESS == pmicStatus)
        {
            pmicStatus = Pmic_commIntf_sendByte(
                                       pPmicCoreHandle,
                                       pPwrRsrcRegCfg[pwrRsrcIndex].ctrlRegAddr,
                                       regData);
        }

        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    return pmicStatus;
}

/*!
 * \brief   This function is used to get Enable/Disable status for residual
 *          voltage checking for BUCK/LDO/VMON pin.
 */
static int32_t Pmic_powerGetRvCheckEn(Pmic_CoreHandle_t *pPmicCoreHandle,
                                      uint16_t           pwrRsrc,
                                      bool              *pResidualVoltCheck)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    uint8_t bitPos  = 0U;
    uint8_t pwrRsrcIndex = 0U;
    Pmic_powerRsrcRegCfg_t *pPwrRsrcRegCfg = NULL;
    uint8_t bitMask = 0U;

    pwrRsrcIndex = Pmic_powerGetPwrRsrcId(pwrRsrc);

    /* Get PMIC power resources register configuration */
    pmicStatus = Pmic_powerGetRsrcRegCfg(pPmicCoreHandle->pmicDeviceType,
                                         &pPwrRsrcRegCfg);
    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        Pmic_criticalSectionStart(pPmicCoreHandle);
        pmicStatus = Pmic_commIntf_recvByte(
                                       pPmicCoreHandle,
                                       pPwrRsrcRegCfg[pwrRsrcIndex].ctrlRegAddr,
                                       &regData);
        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        pmicStatus = Pmic_powerGetRvCheckEnBitPos(
                                                pPmicCoreHandle->pmicDeviceType,
                                                pwrRsrc,
                                                &bitPos,
                                                &bitMask);

        *pResidualVoltCheck = Pmic_getBitField(regData, bitPos, bitMask);
    }

    return pmicStatus;
}

/*!
 * \brief   This function is used to convert the vset value to voltage in mv
            for BUCK/VMON
 */
int32_t Pmic_powerBuckVmonConvertVSetVal2Voltage(const uint8_t *pVSetVal,
                                                 uint16_t      *pBaseMillivolt,
                                                 uint8_t       *pMillivoltStep,
                                                 uint8_t       *pBaseVoutCode)
{
    int32_t status = PMIC_ST_SUCCESS;

    if(*pVSetVal <= PMIC_POWER_VSET_VAL_0xE)
    {
        *pBaseMillivolt = PMIC_POWER_VOLTAGE_300MV;
        *pMillivoltStep = PMIC_POWER_VOLTAGE_STEP_20V;
        *pBaseVoutCode  = PMIC_POWER_VSET_VAL_0x0;
    }
    else if((*pVSetVal >= PMIC_POWER_VSET_VAL_0xF) &&
            (*pVSetVal <= PMIC_POWER_VSET_VAL_0x72))
    {
        *pBaseMillivolt = PMIC_POWER_VOLTAGE_600MV;
        *pMillivoltStep = PMIC_POWER_VOLTAGE_STEP_5V;
        *pBaseVoutCode  = PMIC_POWER_VSET_VAL_0xF;
    }
    else if((*pVSetVal >= PMIC_POWER_VSET_VAL_0x73) &&
            (*pVSetVal <= PMIC_POWER_VSET_VAL_0xAA))
    {
        *pBaseMillivolt = PMIC_POWER_VOLTAGE_1100MV;
        *pMillivoltStep = PMIC_POWER_VOLTAGE_STEP_10V;
        *pBaseVoutCode  = PMIC_POWER_VSET_VAL_0x73;
    }
    else if((*pVSetVal >= PMIC_POWER_VSET_VAL_0xAB) &&
            (*pVSetVal <= PMIC_POWER_VSET_VAL_0xFF))
    {
        *pBaseMillivolt = PMIC_POWER_VOLTAGE_1660MV;
        *pMillivoltStep = PMIC_POWER_VOLTAGE_STEP_20V;
        *pBaseVoutCode  = PMIC_POWER_VSET_VAL_0xAB;
    }
    else
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    return status;
}

/*!
 * \brief   This function is used to convert the vset value to voltage in mv
 *          when the selected voltage monitoring range for VMON is
 *          PMIC_LP8764X_VMON_RANGE_3V35_5V
 */
int32_t Pmic_powerVmonRange1ConvertVSetVal2Voltage(uint16_t *pBaseMillivolt,
                                                   uint8_t  *pMillivoltStep,
                                                   uint8_t  *pBaseVoutCode)
{
    int32_t status = PMIC_ST_SUCCESS;

    *pBaseMillivolt = PMIC_POWER_VOLTAGE_3350MV;
    *pMillivoltStep = PMIC_POWER_VOLTAGE_STEP_25V;
    *pBaseVoutCode  = PMIC_POWER_VSET_VAL_0x1D;

    return status;
}

/*!
 * \brief   This function is used to convert the vset value to voltage in mv
 *          for LDO
 */
int32_t Pmic_powerLdoConvertVSetVal2Voltage(uint16_t  pwrRsrc,
                                            uint16_t *pBaseMillivolt,
                                            uint8_t  *pMillivoltStep,
                                            uint8_t  *pBaseVoutCode)
{
    int32_t status = PMIC_ST_SUCCESS;

    if(PMIC_TPS6594X_REGULATOR_LDO4 == pwrRsrc)
    {
        *pBaseMillivolt = PMIC_POWER_VOLTAGE_1200MV;
        *pMillivoltStep = PMIC_POWER_VOLTAGE_STEP_25V;
        *pBaseVoutCode  = PMIC_POWER_VSET_VAL_0x20;
    }
    else
    {
        /* PMIC_TPS6594X_REGULATOR_LDO1, PMIC_TPS6594X_REGULATOR_LDO2,
           PMIC_TPS6594X_REGULATOR_LDO3 */
        *pBaseMillivolt = PMIC_POWER_VOLTAGE_600MV;
        *pMillivoltStep = PMIC_POWER_VOLTAGE_STEP_50V;
        *pBaseVoutCode  = PMIC_POWER_VSET_VAL_0x4;
    }

    return status;
}

/*!
 * \brief   This function is used to convert the vset value to voltage in mv.
 */
static int32_t Pmic_powerConvertVolt(Pmic_CoreHandle_t *pPmicCoreHandle,
                                     uint8_t            vSetVal,
                                     uint16_t           pwrRsrc,
                                     uint16_t          *millivolt)
{
    int32_t  status        = PMIC_ST_SUCCESS;
    uint16_t baseMillivolt = 0U;
    uint8_t  millivoltStep = 0U;
    uint8_t  baseVoutCode  = 0U;

    switch(pPmicCoreHandle->pmicDeviceType)
    {
        case PMIC_DEV_LEO_TPS6594X:
            status = Pmic_powerTPS6594xConvertVSet2Voltage(&vSetVal,
                                                           pwrRsrc,
                                                           &baseMillivolt,
                                                           &millivoltStep,
                                                           &baseVoutCode);
        break;
        case PMIC_DEV_HERA_LP8764X:
            status = Pmic_powerLP8764xConvertVSetVal2Voltage(pPmicCoreHandle,
                                                             &vSetVal,
                                                             pwrRsrc,
                                                             &baseMillivolt,
                                                             &millivoltStep,
                                                             &baseVoutCode);
        break;
        default:
            status = PMIC_ST_ERR_INV_PARAM;
            break;
    }

    if(PMIC_ST_SUCCESS == status)
    {
        *millivolt = (baseMillivolt +
                     ((vSetVal - baseVoutCode) * millivoltStep));
    }

    return status;
}

/*!
 * \brief   This function is used to convert the millivolt value to vset value
 *          for BUCK/VMON (For VMON : When range = 0)
 */
int32_t Pmic_powerBuckVmonConvertVoltage2VSetVal(uint16_t millivolt,
                                                 uint16_t *pBaseMillivolt,
                                                 uint8_t  *pMillivoltStep,
                                                 uint8_t  *pBaseVoutCode)
{
    int32_t status = PMIC_ST_SUCCESS;

    if((millivolt >= PMIC_POWER_VOLTAGE_300MV) &&
       (millivolt <= PMIC_POWER_VOLTAGE_580MV))
    {
        *pBaseMillivolt = PMIC_POWER_VOLTAGE_300MV;
        *pMillivoltStep = PMIC_POWER_VOLTAGE_STEP_20V;
        *pBaseVoutCode  = PMIC_POWER_VSET_VAL_0x0;
    }
    else if((millivolt >= PMIC_POWER_VOLTAGE_600MV) &&
            (millivolt <= PMIC_POWER_VOLTAGE_1095MV))
    {
        *pBaseMillivolt = PMIC_POWER_VOLTAGE_600MV;
        *pMillivoltStep = PMIC_POWER_VOLTAGE_STEP_5V;
        *pBaseVoutCode  = PMIC_POWER_VSET_VAL_0xF;
    }
    else if((millivolt >= PMIC_POWER_VOLTAGE_1100MV) &&
            (millivolt <= PMIC_POWER_VOLTAGE_1650MV))
    {
        *pBaseMillivolt = PMIC_POWER_VOLTAGE_1100MV;
        *pMillivoltStep = PMIC_POWER_VOLTAGE_STEP_10V;
        *pBaseVoutCode  = PMIC_POWER_VSET_VAL_0x73;
    }
    else if((millivolt >= PMIC_POWER_VOLTAGE_1660MV) &&
            (millivolt <= PMIC_POWER_VOLTAGE_3340MV))
    {
        *pBaseMillivolt = PMIC_POWER_VOLTAGE_1660MV;
        *pMillivoltStep = PMIC_POWER_VOLTAGE_STEP_20V;
        *pBaseVoutCode  = PMIC_POWER_VSET_VAL_0xAB;
    }
    else
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    return status;
}

/*!
 * \brief   This function is used to convert the millivolt value to vset code
 *          when the selected voltage monitoring range for VMON is
 *          PMIC_LP8764X_VMON_RANGE_3V35_5V
 */
int32_t Pmic_powerVmonRange1ConvertVoltage2VSetVal(uint16_t *pBaseMillivolt,
                                                   uint8_t  *pMillivoltStep,
                                                   uint8_t  *pBaseVoutCode)
{
    int32_t  status = PMIC_ST_SUCCESS;

    *pBaseMillivolt = PMIC_POWER_VOLTAGE_3350MV;
    *pMillivoltStep = PMIC_POWER_VOLTAGE_STEP_25V;
    *pBaseVoutCode  = PMIC_POWER_VSET_VAL_0x1D;

    return status;
}

/*!
 * \brief   This function is used to convert the millivolt value to vset value
 *          for LDO Regulators
 */
int32_t Pmic_powerLdoConvertVoltage2VSetVal(uint16_t  pwrRsrc,
                                            uint16_t *pBaseMillivolt,
                                            uint8_t  *pMillivoltStep,
                                            uint8_t  *pBaseVoutCode)
{
    int32_t status = PMIC_ST_SUCCESS;

    if(PMIC_TPS6594X_REGULATOR_LDO4 == pwrRsrc)
    {

        *pBaseMillivolt = PMIC_POWER_VOLTAGE_1200MV;
        *pMillivoltStep = PMIC_POWER_VOLTAGE_STEP_25V;
        *pBaseVoutCode  = PMIC_POWER_VSET_VAL_0x20;
    }
    else
    {
        /* PMIC_TPS6594X_REGULATOR_LDO1, PMIC_TPS6594X_REGULATOR_LDO2,
           PMIC_TPS6594X_REGULATOR_LDO3 */
        *pBaseMillivolt = PMIC_POWER_VOLTAGE_600MV;
        *pMillivoltStep = PMIC_POWER_VOLTAGE_STEP_50V;
        *pBaseVoutCode  = PMIC_POWER_VSET_VAL_0x4;
    }

    return status;
}

/*!
 * \brief   This function is used to get OV/UV voltage monitoring range for
 *          VMON2 and VMON1
 */
int32_t Pmic_powerGetVmonRange(Pmic_CoreHandle_t *pPmicCoreHandle,
                               uint16_t           pwrRsrc,
                               bool              *pVmonRange)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    uint8_t pwrRsrcIndex = 0U;
    Pmic_powerRsrcRegCfg_t *pPwrRsrcRegCfg = NULL;

    pwrRsrcIndex = Pmic_powerGetPwrRsrcId(pwrRsrc);

    /* Get PMIC power resources register configuration */
    pmicStatus = Pmic_powerGetRsrcRegCfg(pPmicCoreHandle->pmicDeviceType,
                                         &pPwrRsrcRegCfg);
    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        Pmic_criticalSectionStart(pPmicCoreHandle);

        pmicStatus = Pmic_commIntf_recvByte(
                                   pPmicCoreHandle,
                                   pPwrRsrcRegCfg[pwrRsrcIndex].pgWindowRegAddr,
                                   &regData);
        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        *pVmonRange = Pmic_getBitField(regData,
                                       PMIC_VMON_PG_WINDOW_VMON_RANGE_SHIFT,
                                       PMIC_VMON_PG_WINDOW_VMON_RANGE_MASK);
    }

    return pmicStatus;
}

/*!
 * \brief   This function is used to convert the millivolt value to vset value.
 */
static int32_t Pmic_powerConvertVoltage2VSetVal(
                                             Pmic_CoreHandle_t *pPmicCoreHandle,
                                             uint16_t           millivolt,
                                             uint16_t           pwrRsrc,
                                             uint8_t           *pVSetVal)
{
    int32_t  status        = PMIC_ST_SUCCESS;
    uint16_t baseMillivolt = 0U;
    uint8_t  millivoltStep = 0U;
    uint8_t  baseVoutCode  = 0U;

    switch(pPmicCoreHandle->pmicDeviceType)
    {
        case PMIC_DEV_LEO_TPS6594X:
            status = Pmic_powerTPS6594xConvertVoltage2VSetVal(millivolt,
                                                              pwrRsrc,
                                                              &baseMillivolt,
                                                              &millivoltStep,
                                                              &baseVoutCode);
            break;
        case PMIC_DEV_HERA_LP8764X:
            status = Pmic_powerLP8764xConvertVoltage2VSetVal(pPmicCoreHandle,
                                                             millivolt,
                                                             pwrRsrc,
                                                             &baseMillivolt,
                                                             &millivoltStep,
                                                             &baseVoutCode);
        break;
        default:
            status = PMIC_ST_ERR_INV_PARAM;
            break;
    }

    if((PMIC_ST_SUCCESS == status) &&
       (true == (millivolt % millivoltStep)))
       {
           status = PMIC_ST_ERR_INV_PARAM;
       }

    if(PMIC_ST_SUCCESS == status)
    {
        *pVSetVal = (baseVoutCode +
                    ((millivolt - baseMillivolt) / millivoltStep));
    }

    return status;
}

/*!
 * \brief   This function is used to select output voltage register for buck
 */
static int32_t Pmic_powerSetBuckVoutSel(Pmic_CoreHandle_t *pPmicCoreHandle,
                                        uint16_t           pwrRsrc,
                                        bool               buckVoutSel)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    uint8_t pwrRsrcIndex = 0U;
    Pmic_powerRsrcRegCfg_t *pPwrRsrcRegCfg = NULL;

    pwrRsrcIndex = Pmic_powerGetPwrRsrcId(pwrRsrc);
    /* Get PMIC power resources register configuration */
    pmicStatus = Pmic_powerGetRsrcRegCfg(pPmicCoreHandle->pmicDeviceType,
                                         &pPwrRsrcRegCfg);

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        Pmic_criticalSectionStart(pPmicCoreHandle);
        pmicStatus = Pmic_commIntf_recvByte(
                                       pPmicCoreHandle,
                                       pPwrRsrcRegCfg[pwrRsrcIndex].ctrlRegAddr,
                                       &regData);

        if(PMIC_ST_SUCCESS == pmicStatus)
        {
            Pmic_setBitField(&regData,
                             PMIC_BUCKX_CTRL_BUCKX_VSEL_SHIFT,
                             PMIC_BUCKX_CTRL_BUCKX_VSEL_MASK,
                             buckVoutSel);

            pmicStatus = Pmic_commIntf_sendByte(
                                       pPmicCoreHandle,
                                       pPwrRsrcRegCfg[pwrRsrcIndex].ctrlRegAddr,
                                       regData);
        }

        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    return pmicStatus;
}

/*!
 * \brief   This function is used to get output voltage register for buck
 */
static int32_t Pmic_powerGetBuckVoutSel(Pmic_CoreHandle_t *pPmicCoreHandle,
                                        uint16_t           pwrRsrc,
                                        bool              *pBuckVoutSel)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    uint8_t pwrRsrcIndex = 0U;
    Pmic_powerRsrcRegCfg_t *pPwrRsrcRegCfg = NULL;

    pwrRsrcIndex = Pmic_powerGetPwrRsrcId(pwrRsrc);
    /* Get PMIC power resources register configuration */
    pmicStatus = Pmic_powerGetRsrcRegCfg(pPmicCoreHandle->pmicDeviceType,
                                         &pPwrRsrcRegCfg);

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        Pmic_criticalSectionStart(pPmicCoreHandle);

        pmicStatus = Pmic_commIntf_recvByte(
                                       pPmicCoreHandle,
                                       pPwrRsrcRegCfg[pwrRsrcIndex].ctrlRegAddr,
                                       &regData);
        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        *pBuckVoutSel = Pmic_getBitField(regData,
                                         PMIC_BUCKX_CTRL_BUCKX_VSEL_SHIFT,
                                         PMIC_BUCKX_CTRL_BUCKX_VSEL_MASK);
    }

    return pmicStatus;
}

/*!
 * \brief   This function is used to set PWM or Auto Mode for BUCK Regulator
 */
static int32_t Pmic_powerSetBuckFpwmMode(Pmic_CoreHandle_t *pPmicCoreHandle,
                                         uint16_t           pwrRsrc,
                                         bool               buckFpwmMode)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    uint8_t pwrRsrcIndex = 0U;
    Pmic_powerRsrcRegCfg_t *pPwrRsrcRegCfg = NULL;

    pwrRsrcIndex = Pmic_powerGetPwrRsrcId(pwrRsrc);
    /* Get PMIC power resources register configuration */
    pmicStatus = Pmic_powerGetRsrcRegCfg(pPmicCoreHandle->pmicDeviceType,
                                         &pPwrRsrcRegCfg);

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        Pmic_criticalSectionStart(pPmicCoreHandle);
        pmicStatus = Pmic_commIntf_recvByte(
                                       pPmicCoreHandle,
                                       pPwrRsrcRegCfg[pwrRsrcIndex].ctrlRegAddr,
                                       &regData);

        if(PMIC_ST_SUCCESS == pmicStatus)
        {
            Pmic_setBitField(&regData,
                             PMIC_BUCKX_CTRL_BUCKX_FPWM_SHIFT,
                             PMIC_BUCKX_CTRL_BUCKX_FPWM_MASK,
                             buckFpwmMode);

            pmicStatus = Pmic_commIntf_sendByte(
                                       pPmicCoreHandle,
                                       pPwrRsrcRegCfg[pwrRsrcIndex].ctrlRegAddr,
                                       regData);
        }

        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    return pmicStatus;
}

/*!
 * \brief   This function is used to get PWM or Auto Mode for BUCK Regulator
 */
static int32_t Pmic_powerGetBuckFpwmMode(Pmic_CoreHandle_t *pPmicCoreHandle,
                                         uint16_t           pwrRsrc,
                                         bool              *pBuckFpwmMode)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    uint8_t pwrRsrcIndex = 0U;
    Pmic_powerRsrcRegCfg_t *pPwrRsrcRegCfg = NULL;

    pwrRsrcIndex = Pmic_powerGetPwrRsrcId(pwrRsrc);

    /* Get PMIC power resources register configuration */
    pmicStatus = Pmic_powerGetRsrcRegCfg(pPmicCoreHandle->pmicDeviceType,
                                         &pPwrRsrcRegCfg);
    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        Pmic_criticalSectionStart(pPmicCoreHandle);

        pmicStatus = Pmic_commIntf_recvByte(
                                       pPmicCoreHandle,
                                       pPwrRsrcRegCfg[pwrRsrcIndex].ctrlRegAddr,
                                       &regData);
        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        *pBuckFpwmMode = Pmic_getBitField(regData,
                                          PMIC_BUCKX_CTRL_BUCKX_FPWM_SHIFT,
                                          PMIC_BUCKX_CTRL_BUCKX_FPWM_MASK);
    }

    return pmicStatus;
}

/*!
 * \brief   This function is used to select between Multi phase with PWM OR AUTO
 *          Mode with Automatic phase adding and shedding for BUCK.
 */
static int32_t Pmic_powerSetBuckFpwmMpMode(Pmic_CoreHandle_t *pPmicCoreHandle,
                                           uint16_t           pwrRsrc,
                                           bool               buckFpwmMpMode)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    uint8_t pwrRsrcIndex = 0U;
    Pmic_powerRsrcRegCfg_t *pPwrRsrcRegCfg = NULL;

    pwrRsrcIndex = Pmic_powerGetPwrRsrcId(pwrRsrc);
    /* Get PMIC power resources register configuration */
    pmicStatus = Pmic_powerGetRsrcRegCfg(pPmicCoreHandle->pmicDeviceType,
                                         &pPwrRsrcRegCfg);

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        Pmic_criticalSectionStart(pPmicCoreHandle);

        pmicStatus = Pmic_commIntf_recvByte(
                                       pPmicCoreHandle,
                                       pPwrRsrcRegCfg[pwrRsrcIndex].ctrlRegAddr,
                                       &regData);
        if(PMIC_ST_SUCCESS == pmicStatus)
        {
            Pmic_setBitField(&regData,
                             PMIC_BUCKX_CTRL_BUCKX_FPWM_MP_SHIFT,
                             PMIC_BUCKX_CTRL_BUCKX_FPWM_MP_MASK,
                             buckFpwmMpMode);

            pmicStatus = Pmic_commIntf_sendByte(
                                       pPmicCoreHandle,
                                       pPwrRsrcRegCfg[pwrRsrcIndex].ctrlRegAddr,
                                       regData);
        }

        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    return pmicStatus;
}

/*!
 * \brief   This function is used to get the Mode between Multi phase with PWM
 *          OR AUTO Mode with Automatic phase adding and shedding for BUCK
 */
static int32_t Pmic_powerGetBuckPwmMpMode(Pmic_CoreHandle_t *pPmicCoreHandle,
                                          uint16_t           pwrRsrc,
                                          bool              *pBuckFpwmMpMode)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    uint8_t pwrRsrcIndex = 0U;
    Pmic_powerRsrcRegCfg_t *pPwrRsrcRegCfg = NULL;

    pwrRsrcIndex = Pmic_powerGetPwrRsrcId(pwrRsrc);

    /* Get PMIC power resources register configuration */
    pmicStatus = Pmic_powerGetRsrcRegCfg(pPmicCoreHandle->pmicDeviceType,
                                         &pPwrRsrcRegCfg);

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        Pmic_criticalSectionStart(pPmicCoreHandle);

        pmicStatus = Pmic_commIntf_recvByte(
                                       pPmicCoreHandle,
                                       pPwrRsrcRegCfg[pwrRsrcIndex].ctrlRegAddr,
                                       &regData);
        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        *pBuckFpwmMpMode = Pmic_getBitField(regData,
                                            PMIC_BUCKX_CTRL_BUCKX_FPWM_MP_SHIFT,
                                            PMIC_BUCKX_CTRL_BUCKX_FPWM_MP_MASK);
    }

    return pmicStatus;
}

/*!
 * \brief   This function is used to Enable/Disable Slow Ramp for LDO
 */
static int32_t Pmic_powerSetLdoSlowRampEn(Pmic_CoreHandle_t *pPmicCoreHandle,
                                          uint16_t           pwrRsrc,
                                          bool               ldoSlowRampEn)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    uint8_t pwrRsrcIndex = 0U;
    Pmic_powerRsrcRegCfg_t *pPwrRsrcRegCfg = NULL;

    pwrRsrcIndex = Pmic_powerGetPwrRsrcId(pwrRsrc);

    /* Get PMIC power resources register configuration */
    pmicStatus = Pmic_powerGetRsrcRegCfg(pPmicCoreHandle->pmicDeviceType,
                                         &pPwrRsrcRegCfg);

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        Pmic_criticalSectionStart(pPmicCoreHandle);

        pmicStatus = Pmic_commIntf_recvByte(
                                       pPmicCoreHandle,
                                       pPwrRsrcRegCfg[pwrRsrcIndex].ctrlRegAddr,
                                       &regData);
        if(PMIC_ST_SUCCESS == pmicStatus)
        {
            Pmic_setBitField(&regData,
                             PMIC_LDOX_CTRL_LDOX_SLOW_RAMP_EN_SHIFT,
                             PMIC_LDOX_CTRL_LDOX_SLOW_RAMP_EN_MASK,
                             ldoSlowRampEn);

            pmicStatus = Pmic_commIntf_sendByte(
                                       pPmicCoreHandle,
                                       pPwrRsrcRegCfg[pwrRsrcIndex].ctrlRegAddr,
                                       regData);
        }

        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    return pmicStatus;
}

/*!
 * \brief   Get Enable /Disable status for Slow Ramp for LDO
 */
static int32_t Pmic_powerGetLdoSlowRampEn(Pmic_CoreHandle_t *pPmicCoreHandle,
                                          uint16_t           pwrRsrc,
                                          bool              *pLdoSlowRampEn)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    uint8_t pwrRsrcIndex = 0U;
    Pmic_powerRsrcRegCfg_t *pPwrRsrcRegCfg = NULL;

    pwrRsrcIndex = Pmic_powerGetPwrRsrcId(pwrRsrc);

    /* Get PMIC power resources register configuration */
    pmicStatus = Pmic_powerGetRsrcRegCfg(pPmicCoreHandle->pmicDeviceType,
                                         &pPwrRsrcRegCfg);

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        Pmic_criticalSectionStart(pPmicCoreHandle);

        pmicStatus = Pmic_commIntf_recvByte(
                                       pPmicCoreHandle,
                                       pPwrRsrcRegCfg[pwrRsrcIndex].ctrlRegAddr,
                                       &regData);
       Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        *pLdoSlowRampEn =
                       Pmic_getBitField(regData,
                                        PMIC_LDOX_CTRL_LDOX_SLOW_RAMP_EN_SHIFT,
                                        PMIC_LDOX_CTRL_LDOX_SLOW_RAMP_EN_MASK);
    }

    return pmicStatus;
}

/*!
 * \brief   This function is used to enable/disable regulator.
 */
static int32_t Pmic_powerSetRegulatorEn(Pmic_CoreHandle_t *pPmicCoreHandle,
                                        uint16_t           pwrRsrc,
                                        bool               regulatorEn)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    uint8_t pwrRsrcIndex = 0U;
    Pmic_powerRsrcRegCfg_t *pPwrRsrcRegCfg = NULL;

    pwrRsrcIndex = Pmic_powerGetPwrRsrcId(pwrRsrc);

    /* Get PMIC power resources register configuration */
    pmicStatus = Pmic_powerGetRsrcRegCfg(pPmicCoreHandle->pmicDeviceType,
                                         &pPwrRsrcRegCfg);

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        Pmic_criticalSectionStart(pPmicCoreHandle);

        pmicStatus = Pmic_commIntf_recvByte(
                                       pPmicCoreHandle,
                                       pPwrRsrcRegCfg[pwrRsrcIndex].ctrlRegAddr,
                                       &regData);

        if(PMIC_ST_SUCCESS == pmicStatus)
        {
            Pmic_setBitField(&regData,
                             PMIC_POWER_RESOURCEX_CTRL_POWER_RESOURCEX_EN_SHIFT,
                             PMIC_POWER_RESOURCEX_CTRL_POWER_RESOURCEX_EN_MASK,
                             regulatorEn);

            pmicStatus = Pmic_commIntf_sendByte(
                                       pPmicCoreHandle,
                                       pPwrRsrcRegCfg[pwrRsrcIndex].ctrlRegAddr,
                                       regData);
        }

        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    return pmicStatus;
}

/*!
 * \brief   This function is used to get the enable/disable status of regulator
 */
static int32_t Pmic_powerGetRegulatorEn(Pmic_CoreHandle_t *pPmicCoreHandle,
                                        uint16_t           pwrRsrc,
                                        bool              *pRegulatorEn)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    uint8_t pwrRsrcIndex = 0U;
    Pmic_powerRsrcRegCfg_t *pPwrRsrcRegCfg = NULL;

    pwrRsrcIndex = Pmic_powerGetPwrRsrcId(pwrRsrc);

    /* Get PMIC power resources register configuration */
    pmicStatus = Pmic_powerGetRsrcRegCfg(pPmicCoreHandle->pmicDeviceType,
                                         &pPwrRsrcRegCfg);
    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        Pmic_criticalSectionStart(pPmicCoreHandle);

        pmicStatus = Pmic_commIntf_recvByte(
                                       pPmicCoreHandle,
                                       pPwrRsrcRegCfg[pwrRsrcIndex].ctrlRegAddr,
                                       &regData);
        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        *pRegulatorEn = Pmic_getBitField(
                             regData,
                             PMIC_POWER_RESOURCEX_CTRL_POWER_RESOURCEX_EN_SHIFT,
                             PMIC_POWER_RESOURCEX_CTRL_POWER_RESOURCEX_EN_MASK);
    }

    return pmicStatus;
}

/**
 * \brief   This function is used to get the voltage register of the BUCK
 */
static int32_t Pmic_powerGetBuckVoutRegAddr(Pmic_CoreHandle_t *pPmicCoreHandle,
                                            uint16_t           pwrRsrc,
                                            uint8_t           *pRegAddr)
{

    int32_t status = PMIC_ST_SUCCESS;
    bool buckVoutSel;
    uint8_t pwrRsrcIndex;

    pwrRsrcIndex = Pmic_powerGetPwrRsrcId(pwrRsrc);

    Pmic_powerRsrcRegCfg_t *pPwrRsrcRegCfg = NULL;

    /* Get PMIC power resources register configuration */
    status = Pmic_powerGetRsrcRegCfg(pPmicCoreHandle->pmicDeviceType,
                                     &pPwrRsrcRegCfg);

    if(PMIC_ST_SUCCESS == status)
    {
        *pRegAddr = pPwrRsrcRegCfg[pwrRsrcIndex].vout1RegAddr;
        /* Get the voltage register adress in case of BUCK regulator */
        status = Pmic_powerGetBuckVoutSel(pPmicCoreHandle,
                                          pwrRsrc,
                                          &(buckVoutSel));
    }

    if(PMIC_ST_SUCCESS == status)
    {
        switch (pPmicCoreHandle->pmicDeviceType)
        {
            case PMIC_DEV_LEO_TPS6594X:
                if(PMIC_TPS6594X_REGULATOR_BUCK_VOUT_SEL_VOUT2 == buckVoutSel)
                {
                    *pRegAddr = pPwrRsrcRegCfg[pwrRsrcIndex].vout2RegAddr;
                }

                break;
            case PMIC_DEV_HERA_LP8764X:
                if(PMIC_LP8764X_REGULATOR_BUCK_VOUT_SEL_VOUT2 == buckVoutSel)
                {
                    *pRegAddr = pPwrRsrcRegCfg[pwrRsrcIndex].vout2RegAddr;
                }

                break;
             default:
                status = PMIC_ST_ERR_INV_PARAM;
                break;
        }
    }

    return status;
}

/**
 * \brief   This function is used to get the voltage register of the power
 *          resource.
 */
static int32_t Pmic_GetVoltageRegAddr(Pmic_CoreHandle_t *pPmicCoreHandle,
                                      uint16_t           pwrRsrc,
                                      uint8_t           *pRegAddr)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t pwrRsrcIndex;
    uint8_t pwrRsrcType;
    Pmic_powerRsrcRegCfg_t *pPwrRsrcRegCfg = NULL;

    pwrRsrcIndex = Pmic_powerGetPwrRsrcId(pwrRsrc);
    pwrRsrcType  = Pmic_powerGetPwrRsrcType(pwrRsrc);

    /* Get PMIC power resources register configuration */
    status = Pmic_powerGetRsrcRegCfg(pPmicCoreHandle->pmicDeviceType,
                                     &pPwrRsrcRegCfg);
    if(PMIC_ST_SUCCESS == status)
    {
        if(PMIC_ST_SUCCESS == Pmic_powerValidateLdoVmonPwrRsrcType(
                                                pPmicCoreHandle->pmicDeviceType,
                                                pwrRsrcType))
        {
            *pRegAddr = pPwrRsrcRegCfg[pwrRsrcIndex].vout1RegAddr;
        }
        else if(PMIC_ST_SUCCESS == Pmic_powerValidateBuckPwrRsrcType(
                                                pPmicCoreHandle->pmicDeviceType,
                                                pwrRsrcType))
        {
            status = Pmic_powerGetBuckVoutRegAddr(pPmicCoreHandle,
                                                  pwrRsrc,
                                                  pRegAddr);
        }
        else
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
    }

    return status;
}

/**
 * \brief   This function is used to validate the voltage levels for
 *          Regulators/VMON
 */
static int32_t Pmic_powerValidateVoltageLevel(
                                             Pmic_CoreHandle_t *pPmicCoreHandle,
                                             uint16_t           pwrRsrc,
                                             uint16_t           voltage_mV)
{
    int32_t  status = PMIC_ST_SUCCESS;
    uint8_t  pwrRsrcType = 0U;
    uint16_t ldoMinVoltageValue = 0U;
    bool     vmonRange = 0U;

    pwrRsrcType = Pmic_powerGetPwrRsrcType(pwrRsrc);

    switch(pPmicCoreHandle->pmicDeviceType)
    {
        case PMIC_DEV_LEO_TPS6594X:
            if(PMIC_TPS6594X_POWER_RESOURCE_TYPE_BUCK == pwrRsrcType)
            {
                if((voltage_mV < PMIC_TPS6594X_REGULATOR_BUCK_MIN_VOLTAGE) ||
                   (voltage_mV > PMIC_TPS6594X_REGULATOR_BUCK_MAX_VOLTAGE))
                {
                    status = PMIC_ST_ERR_INV_PARAM;
                }
            }
            else if(PMIC_TPS6594X_POWER_RESOURCE_TYPE_LDO == pwrRsrcType)
            {
                if(PMIC_TPS6594X_REGULATOR_LDO4 == pwrRsrc)
                {
                    ldoMinVoltageValue = PMIC_TPS6594X_POWER_LDO4_MIN_VOLTAGE;
                }
                else
                {
                    ldoMinVoltageValue = \
                                       PMIC_TPS6594X_POWER_LDO1_2_3_MIN_VOLTAGE;
                }

                if((voltage_mV < ldoMinVoltageValue) ||
                   (voltage_mV > PMIC_TPS6594X_POWER_LDO_MAX_VOLTAGE))
                {
                    status = PMIC_ST_ERR_INV_PARAM;
                }
            }
            else
            {
                status = PMIC_ST_ERR_INV_PARAM;
            }

            break;
        case PMIC_DEV_HERA_LP8764X:
            if(PMIC_LP8764X_POWER_RESOURCE_TYPE_VMON == pwrRsrcType)
            {
                status = Pmic_powerGetVmonRange(pPmicCoreHandle,
                                                pwrRsrc,
                                                &(vmonRange));

                if((PMIC_ST_SUCCESS == status) &&
                   (PMIC_LP8764X_VMON_RANGE_0V3_3V34 == vmonRange))
                {
                    if((voltage_mV < PMIC_LP8764X_RANGE0_VMON_MIN_VOLTAGE)  ||
                       (voltage_mV > PMIC_LP8764X_RANGE0_VMON_MAX_VOLTAGE))
                    {
                        status = PMIC_ST_ERR_INV_PARAM;
                    }
                }
                else if((PMIC_ST_SUCCESS == status) &&
                        (PMIC_LP8764X_VMON_RANGE_3V35_5V == vmonRange))
                {
                    if((voltage_mV < PMIC_LP8764X_RANGE1_VMON_MIN_VOLTAGE)   ||
                       (voltage_mV > PMIC_LP8764X_RANGE1_VMON_MAX_VOLTAGE))
                    {
                        status = PMIC_ST_ERR_INV_PARAM;
                    }
                }
                else
                {
                    status = PMIC_ST_ERR_INV_PARAM;
                }
            }
            else if(PMIC_LP8764X_POWER_RESOURCE_TYPE_BUCK == pwrRsrcType)
            {
                if((voltage_mV < PMIC_LP8764X_REGULATOR_BUCK_MIN_VOLTAGE) ||
                   (voltage_mV > PMIC_LP8764X_REGULATOR_BUCK_MAX_VOLTAGE))
                {
                    status = PMIC_ST_ERR_INV_PARAM;
                }
            }
            else
            {
                status = PMIC_ST_ERR_INV_PARAM;
            }

            break;
        default:
            status = PMIC_ST_ERR_INV_PARAM;
            break;
    }

    return status;
}

/**
 * \brief   This function is used to get the required bit position of the
 *          voltage levels for Regulators/VMON
 */
static int32_t Pmic_powerGetVoltageBitField(uint8_t   pmicDeviceType,
                                            uint16_t  pwrRsrc,
                                            uint8_t  *pBitPos,
                                            uint8_t  *pBitMask)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t pwrRsrcType;

    pwrRsrcType = Pmic_powerGetPwrRsrcType(pwrRsrc);
    switch(pmicDeviceType)
    {
        case PMIC_DEV_LEO_TPS6594X:
            if(PMIC_TPS6594X_POWER_RESOURCE_TYPE_BUCK == pwrRsrcType)
            {
                *pBitPos = PMIC_BUCKX_VOUT_X_BUCKX_VSETX_SHIFT;
                *pBitMask = PMIC_BUCKX_VOUT_X_BUCKX_VSETX_MASK;
            }
            else if(PMIC_TPS6594X_POWER_RESOURCE_TYPE_LDO == pwrRsrcType)
            {
                if(PMIC_TPS6594X_REGULATOR_LDO4 == pwrRsrc)
                {
                    *pBitPos = PMIC_LDO4_VOUT_LDO4_VSET_SHIFT;
                    *pBitMask = PMIC_LDO4_VOUT_LDO4_VSET_MASK;
                }
                else if((PMIC_TPS6594X_REGULATOR_LDO1 == pwrRsrc)    ||
                        (PMIC_TPS6594X_REGULATOR_LDO2 == pwrRsrc)    ||
                        (PMIC_TPS6594X_REGULATOR_LDO3 == pwrRsrc))
                {
                    *pBitPos = PMIC_LDO1_2_3_VOUT_LDO1_2_3_VSET_SHIFT;
                    *pBitMask = PMIC_LDO1_2_3_VOUT_LDO1_2_3_VSET_MASK;
                }
                else
                {
                    status = PMIC_ST_ERR_INV_PARAM;
                }
            }
            else
            {
                status = PMIC_ST_ERR_INV_PARAM;
            }

            break;
        case PMIC_DEV_HERA_LP8764X:
            if(PMIC_LP8764X_POWER_RESOURCE_TYPE_BUCK == pwrRsrcType)
            {
                *pBitPos = PMIC_BUCKX_VOUT_X_BUCKX_VSETX_SHIFT;
                *pBitMask = PMIC_BUCKX_VOUT_X_BUCKX_VSETX_MASK;
            }
            else if(PMIC_LP8764X_POWER_RESOURCE_TYPE_VMON == pwrRsrcType)
            {
                *pBitPos = PMIC_VMONX_PG_LEVEL_VMONX_PG_SET_SHIFT;
                *pBitMask = PMIC_VMONX_PG_LEVEL_VMONX_PG_SET_MASK;
            }
            else
            {
                status = PMIC_ST_ERR_INV_PARAM;
            }

            break;
        default :
            status = PMIC_ST_ERR_INV_PARAM;
            break;
    }

    return status;
}

/*!
 * \brief   This function is used to set the voltage level for the regulator
 *          in mv OR ppower good level for VMON in mv.
 */
static int32_t Pmic_powerSetVoltage(Pmic_CoreHandle_t *pPmicCoreHandle,
                                    uint16_t           pwrRsrc,
                                    uint16_t           voltage_mV)
{
    int32_t  status  = PMIC_ST_SUCCESS;
    uint8_t  vSetVal = 0U;
    uint8_t  regAddr = 0U;
    uint8_t  regData = 0U;
    uint8_t  bitPos  = 0U;
    uint8_t  bitMask = 0U;

    /* Get the regulator voltage register address */
    status = Pmic_GetVoltageRegAddr(pPmicCoreHandle, pwrRsrc, &regAddr);

    if(PMIC_ST_SUCCESS == status)
    {
        status = Pmic_powerValidateVoltageLevel(pPmicCoreHandle,
                                                pwrRsrc,
                                                voltage_mV);
    }

    if(PMIC_ST_SUCCESS == status)
    {
        status = Pmic_powerConvertVoltage2VSetVal(pPmicCoreHandle,
                                                  voltage_mV,
                                                  pwrRsrc,
                                                  &vSetVal);
    }

    if(PMIC_ST_SUCCESS == status)
    {
        Pmic_criticalSectionStart(pPmicCoreHandle);
        status = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        regAddr,
                                        &regData);

        if(PMIC_ST_SUCCESS == status)
        {
            status = Pmic_powerGetVoltageBitField(
                                                pPmicCoreHandle->pmicDeviceType,
                                                pwrRsrc,
                                                &bitPos,
                                                &bitMask);
        }

        if(PMIC_ST_SUCCESS == status)
        {
            Pmic_setBitField(&regData, bitPos, bitMask, vSetVal);
            status = Pmic_commIntf_sendByte(pPmicCoreHandle, regAddr, regData);
        }

        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    return status;
}

/**
 * \brief   This function is used to read the voltage of the power resource.
 */
static int32_t Pmic_powerGetVoltage(Pmic_CoreHandle_t *pPmicCoreHandle,
                                    uint16_t           pwrRsrc,
                                    uint16_t          *pMillivolt)
{
    int32_t  status  = PMIC_ST_SUCCESS;
    uint8_t  regData = 0U;
    uint8_t  regAddr = 0U;
    uint8_t  vSetVal = 0U;
    uint8_t  bitPos  = 0U;
    uint8_t  bitMask = 0U;

    if(NULL == pMillivolt)
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if(PMIC_ST_SUCCESS == status)
    {
        /* Get the regulator voltage register address */
        status = Pmic_GetVoltageRegAddr(pPmicCoreHandle, pwrRsrc, &regAddr);
    }

    if(PMIC_ST_SUCCESS == status)
    {
        Pmic_criticalSectionStart(pPmicCoreHandle);
        status = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        regAddr,
                                        &regData);
        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    if(PMIC_ST_SUCCESS == status)
    {
        status = Pmic_powerGetVoltageBitField(pPmicCoreHandle->pmicDeviceType,
                                              pwrRsrc,
                                              &bitPos,
                                              &bitMask);
        if(PMIC_ST_SUCCESS == status)
        {
            vSetVal = Pmic_getBitField(regData, bitPos, bitMask);
        }
    }

    if(PMIC_ST_SUCCESS == status)
    {
        status = Pmic_powerConvertVolt(pPmicCoreHandle,
                                       vSetVal,
                                       pwrRsrc,
                                       pMillivolt);
    }

    return status;
}

/*!
 * \brief   This function is used to validate the current limit of BUCK.
 */
static int32_t Pmic_powerValidateBuckCurrentLimit(uint8_t  pmicDeviceType,
                                                  uint8_t  currentLimit,
                                                  uint16_t pwrRsrc)
{
    int32_t status = PMIC_ST_SUCCESS;

    switch(pmicDeviceType)
    {
        case PMIC_DEV_LEO_TPS6594X:
            if((pwrRsrc >= PMIC_TPS6594X_REGULATOR_BUCK1) &&
               (pwrRsrc <= PMIC_TPS6594X_REGULATOR_BUCK4))
            {
                if((currentLimit < PMIC_TPS6594X_BUCK1_4_CURRENT_LIMIT_MIN) ||
                   (currentLimit > PMIC_TPS6594X_BUCK1_4_CURRENT_LIMIT_MAX))
                {
                    status = PMIC_ST_ERR_INV_PARAM;
                }
            }
            else if(pwrRsrc == PMIC_TPS6594X_REGULATOR_BUCK5)
            {
                if((currentLimit < PMIC_TPS6594X_BUCK5_CURRENT_LIMIT_MIN) ||
                   (currentLimit > PMIC_TPS6594X_BUCK5_CURRENT_LIMIT_MAX))
                {
                    status = PMIC_ST_ERR_INV_PARAM;
                }
            }
            break;
        case PMIC_DEV_HERA_LP8764X:
            if((currentLimit < PMIC_LP8764X_BUCK_CURRENT_LIMIT_MIN) ||
               (currentLimit > PMIC_LP8764X_BUCK_CURRENT_LIMIT_MAX))
            {
                status = PMIC_ST_ERR_INV_PARAM;
            }
            break;
        default:
            status = PMIC_ST_ERR_INV_PARAM;
            break;
    }

    return status;
}

/*!
 * \brief   This function is used to set the switch peak current limit for BUCK.
 */
static int32_t Pmic_powerSetBuckCurrentLimit(
                                             Pmic_CoreHandle_t *pPmicCoreHandle,
                                             uint16_t           pwrRsrc,
                                             uint8_t            currentLimit)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData    = 0U;
    uint8_t pwrRsrcIndex;
    Pmic_powerRsrcRegCfg_t *pPwrRsrcRegCfg = NULL;

    pwrRsrcIndex = Pmic_powerGetPwrRsrcId(pwrRsrc);

    /* Get PMIC power resources register configuration */
    pmicStatus = Pmic_powerGetRsrcRegCfg(pPmicCoreHandle->pmicDeviceType,
                                         &pPwrRsrcRegCfg);
    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        pmicStatus = Pmic_powerValidateBuckCurrentLimit(
                                                pPmicCoreHandle->pmicDeviceType,
                                                currentLimit,
                                                pwrRsrc);
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        Pmic_criticalSectionStart(pPmicCoreHandle);
        pmicStatus = Pmic_commIntf_recvByte(
                                     pPmicCoreHandle,
                                     pPwrRsrcRegCfg[pwrRsrcIndex].configRegAddr,
                                     &regData);

        if(PMIC_ST_SUCCESS == pmicStatus)
        {
            Pmic_setBitField(&regData,
                             PMIC_BUCKX_CONF_BUCKX_ILIM_SHIFT,
                             PMIC_BUCKX_CONF_BUCKX_ILIM_MASK,
                             currentLimit);

            pmicStatus = Pmic_commIntf_sendByte(
                                     pPmicCoreHandle,
                                     pPwrRsrcRegCfg[pwrRsrcIndex].configRegAddr,
                                     regData);
        }

        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    return pmicStatus;
}

/*!
 * \brief   This function is used to get the switch peak current limit of BUCK.
 */
static int32_t Pmic_powerGetBuckCurrentLimit(Pmic_CoreHandle_t *pPmicCoreHandle,
                                             uint16_t           pwrRsrc,
                                             uint8_t           *pCurrentLimit)
{
    int32_t  pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    uint8_t pwrRsrcIndex;
    Pmic_powerRsrcRegCfg_t *pPwrRsrcRegCfg = NULL;

    pwrRsrcIndex = Pmic_powerGetPwrRsrcId(pwrRsrc);

    /* Get PMIC power resources register configuration */
    pmicStatus = Pmic_powerGetRsrcRegCfg(pPmicCoreHandle->pmicDeviceType,
                                         &pPwrRsrcRegCfg);
    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        Pmic_criticalSectionStart(pPmicCoreHandle);
        pmicStatus = Pmic_commIntf_recvByte(
                                     pPmicCoreHandle,
                                     pPwrRsrcRegCfg[pwrRsrcIndex].configRegAddr,
                                     &regData);
        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        *pCurrentLimit = Pmic_getBitField(regData,
                                          PMIC_BUCKX_CONF_BUCKX_ILIM_SHIFT,
                                          PMIC_BUCKX_CONF_BUCKX_ILIM_MASK);
    }

    return pmicStatus;
}

/*!
 * \brief   This function is used to validate the slew rate for BUCK/VMON
 */
static int32_t Pmic_powerValidateSlewRateLimit(uint8_t pmicDeviceType,
                                               uint8_t slewRate)
{
    int32_t status = PMIC_ST_SUCCESS;

    switch(pmicDeviceType)
    {
        case PMIC_DEV_LEO_TPS6594X:
            if(slewRate > PMIC_TPS6594X_BUCK_SLEW_RATE_MAX)
            {
                status = PMIC_ST_ERR_INV_PARAM;
            }

            break;
        case PMIC_DEV_HERA_LP8764X:
            if(slewRate > PMIC_LP8764X_BUCK_SLEW_RATE_MAX)
            {
                status = PMIC_ST_ERR_INV_PARAM;
            }

            break;
        default:
            status = PMIC_ST_ERR_INV_PARAM;
            break;
    }
    return status;
}

/*!
 * \brief   This function is used to get the required bit position
 *          of BUCK/VMON Slew rate.
 */
static int32_t Pmic_powerGetBuckVmonSlewRateBitPos(uint8_t   pmicDeviceType,
                                                   uint16_t  pwrRsrc,
                                                   uint8_t  *pBitPos,
                                                   uint8_t  *pBitMask)
{
    int32_t status = PMIC_ST_SUCCESS;

    switch (pmicDeviceType)
    {
        case PMIC_DEV_LEO_TPS6594X:
            *pBitPos  = PMIC_BUCKX_CONF_BUCKX_SLEW_RATE_SHIFT;
            *pBitMask = PMIC_BUCKX_CONF_BUCKX_SLEW_RATE_MASK;
            break;
        case PMIC_DEV_HERA_LP8764X:
            if(PMIC_LP8764X_POWER_SOURCE_VMON1 == pwrRsrc)
            {
                *pBitPos  = PMIC_VMON_CONF_VMON1_SLEW_RATE_SHIFT;
                *pBitMask = PMIC_VMON_CONF_VMON1_SLEW_RATE_MASK;
            }
            else if(PMIC_LP8764X_POWER_SOURCE_VMON2 == pwrRsrc)
            {
                *pBitPos  = PMIC_VMON_CONF_VMON2_SLEW_RATE_SHIFT;
                *pBitMask = PMIC_VMON_CONF_VMON2_SLEW_RATE_MASK;
            }
            else
            {
                *pBitPos  = PMIC_BUCKX_CONF_BUCKX_SLEW_RATE_SHIFT;
                *pBitMask = PMIC_BUCKX_CONF_BUCKX_SLEW_RATE_MASK;
            }
            break;
        default:
            status = PMIC_ST_ERR_INV_PARAM;
            break;
    }

    return status;
}

/*!
 * \brief   This function is used to set the output voltage slew rate for
 *           BUCK/VMON
 */
static int32_t Pmic_powerSetBuckVmonSlewRate(
                                            Pmic_CoreHandle_t *pPmicCoreHandle,
                                            uint16_t           pwrRsrc,
                                            uint8_t            buckVmonSlewRate)
{
    int32_t  status = PMIC_ST_SUCCESS;
    uint8_t  regData    = 0U;
    uint8_t  bitPos     = 0U;
    uint8_t  bitMask    = 0U;
    uint8_t pwrRsrcIndex;
    Pmic_powerRsrcRegCfg_t *pPwrRsrcRegCfg = NULL;

    pwrRsrcIndex = Pmic_powerGetPwrRsrcId(pwrRsrc);
    /* Get PMIC power resources register configuration */
    status = Pmic_powerGetRsrcRegCfg(pPmicCoreHandle->pmicDeviceType,
                                     &pPwrRsrcRegCfg);

    if(PMIC_ST_SUCCESS == status)
    {
        status = Pmic_powerValidateSlewRateLimit(
                                                pPmicCoreHandle->pmicDeviceType,
                                                buckVmonSlewRate);
    }

    if(PMIC_ST_SUCCESS == status)
    {
        Pmic_criticalSectionStart(pPmicCoreHandle);
        status = Pmic_commIntf_recvByte(
                                     pPmicCoreHandle,
                                     pPwrRsrcRegCfg[pwrRsrcIndex].configRegAddr,
                                     &regData);

        if(PMIC_ST_SUCCESS == status)
        {
            status = Pmic_powerGetBuckVmonSlewRateBitPos(
                                                pPmicCoreHandle->pmicDeviceType,
                                                pwrRsrc,
                                                &bitPos,
                                                &bitMask);
        }

        if(PMIC_ST_SUCCESS == status)
        {
            Pmic_setBitField(&regData, bitPos, bitMask, buckVmonSlewRate);
            status = Pmic_commIntf_sendByte(
                                     pPmicCoreHandle,
                                     pPwrRsrcRegCfg[pwrRsrcIndex].configRegAddr,
                                     regData);
        }

        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    return status;
}

/*!
 * \brief   This function is used to get the Output voltage slew rate for
 *          BUCK/VMON.
 */
static int32_t Pmic_powerGetBuckVmonSlewRate(Pmic_CoreHandle_t *pPmicCoreHandle,
                                             uint16_t           pwrRsrc,
                                             uint8_t           *pSlewRate)
{
    int32_t status  = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    uint8_t bitPos  = 0U;
    uint8_t bitMask = 0U;
    uint8_t pwrRsrcIndex = 0U;
    Pmic_powerRsrcRegCfg_t *pPwrRsrcRegCfg = NULL;

    pwrRsrcIndex = Pmic_powerGetPwrRsrcId(pwrRsrc);

    /* Get PMIC power resources register configuration */
    status = Pmic_powerGetRsrcRegCfg(pPmicCoreHandle->pmicDeviceType,
                                         &pPwrRsrcRegCfg);
    if(PMIC_ST_SUCCESS == status)
    {
        Pmic_criticalSectionStart(pPmicCoreHandle);
        status = Pmic_commIntf_recvByte(
                                     pPmicCoreHandle,
                                     pPwrRsrcRegCfg[pwrRsrcIndex].configRegAddr,
                                     &regData);
        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    if(PMIC_ST_SUCCESS == status)
    {
        status = Pmic_powerGetBuckVmonSlewRateBitPos(
                                                pPmicCoreHandle->pmicDeviceType,
                                                pwrRsrc,
                                                &bitPos,
                                                &bitMask);
        if(PMIC_ST_SUCCESS == status)
        {
            *pSlewRate = Pmic_getBitField(regData, bitPos, bitMask);
        }
    }

    return status;
}

/*!
 * \brief   This function is used to enable/disable Buck Output Pull Down
 *           Resistor when BUCK is disabled.
 */
static int32_t Pmic_powerSetBuckPullDownEn(Pmic_CoreHandle_t *pPmicCoreHandle,
                                           uint16_t           pwrRsrc,
                                           bool               buckPullDownEn)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData    = 0U;
    uint8_t pwrRsrcIndex;
    Pmic_powerRsrcRegCfg_t *pPwrRsrcRegCfg = NULL;

    pwrRsrcIndex = Pmic_powerGetPwrRsrcId(pwrRsrc);

    /* Get PMIC power resources register configuration */
    pmicStatus = Pmic_powerGetRsrcRegCfg(pPmicCoreHandle->pmicDeviceType,
                                         &pPwrRsrcRegCfg);

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        Pmic_criticalSectionStart(pPmicCoreHandle);
        pmicStatus = Pmic_commIntf_recvByte(
                                      pPmicCoreHandle,
                                      pPwrRsrcRegCfg[pwrRsrcIndex].ctrlRegAddr,
                                      &regData);

        if(PMIC_ST_SUCCESS == pmicStatus)
        {
            /* Enable/Disable output pull-down resistor for BUCK regulator */
            Pmic_setBitField(&regData,
                             PMIC_BUCKX_CTRL_BUCKX_PLDN_SHIFT,
                             PMIC_BUCKX_CTRL_BUCKX_PLDN_MASK,
                             buckPullDownEn);
            pmicStatus = Pmic_commIntf_sendByte(
                                       pPmicCoreHandle,
                                       pPwrRsrcRegCfg[pwrRsrcIndex].ctrlRegAddr,
                                       regData);
        }

        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    return pmicStatus;
}

/*!
 * \brief   This function is used to get enable/disable status for  Buck
 *          Output Pull Down Resistor
 */
static int32_t Pmic_powerGetBuckPullDownEn(Pmic_CoreHandle_t *pPmicCoreHandle,
                                           uint16_t           pwrRsrc,
                                           bool              *pBuckPullDownEn)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData    = 0U;
    uint8_t pwrRsrcIndex;
    Pmic_powerRsrcRegCfg_t *pPwrRsrcRegCfg = NULL;

    pwrRsrcIndex = Pmic_powerGetPwrRsrcId(pwrRsrc);

    /* Get PMIC power resources register configuration */
    pmicStatus = Pmic_powerGetRsrcRegCfg(pPmicCoreHandle->pmicDeviceType,
                                         &pPwrRsrcRegCfg);
    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        Pmic_criticalSectionStart(pPmicCoreHandle);
        pmicStatus = Pmic_commIntf_recvByte(
                                       pPmicCoreHandle,
                                       pPwrRsrcRegCfg[pwrRsrcIndex].ctrlRegAddr,
                                       &regData);

        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        *pBuckPullDownEn = Pmic_getBitField(regData,
                                            PMIC_BUCKX_CTRL_BUCKX_PLDN_SHIFT,
                                            PMIC_BUCKX_CTRL_BUCKX_PLDN_MASK);
    }

    return pmicStatus;
}

/*!
 * \brief   This function is used to validate the LDO pull down resistor value.
 */
static int32_t Pmic_powerValidateLdoPullDownSel(uint8_t pmicDeviceType,
                                                uint8_t ldoPullDownSel)
{
    int32_t status = PMIC_ST_SUCCESS;

    switch(pmicDeviceType)
    {
        case PMIC_DEV_LEO_TPS6594X:
            if(ldoPullDownSel > PMIC_TPS6594X_REGULATOR_LDO_PLDN_VAL_500OHM)
            {
                status = PMIC_ST_ERR_INV_PARAM;
            }

            break;
        default:
            status = PMIC_ST_ERR_INV_PARAM;
            break;
    }

    return status;
}

/*!
 * \brief   This function is used to select the resistor value for output
 *          pull-down resistor for LDO
 */
static int32_t Pmic_powerSetLdoPullDownSel(Pmic_CoreHandle_t *pPmicCoreHandle,
                                           uint16_t           pwrRsrc,
                                           uint8_t            ldoPullDownSel)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData    = 0U;
    uint8_t pwrRsrcIndex;
    Pmic_powerRsrcRegCfg_t *pPwrRsrcRegCfg = NULL;

    pwrRsrcIndex = Pmic_powerGetPwrRsrcId(pwrRsrc);

    /* Get PMIC power resources register configuration */
    pmicStatus = Pmic_powerGetRsrcRegCfg(pPmicCoreHandle->pmicDeviceType,
                                         &pPwrRsrcRegCfg);
    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        pmicStatus = Pmic_powerValidateLdoPullDownSel(
                                                pPmicCoreHandle->pmicDeviceType,
                                                ldoPullDownSel);
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        Pmic_criticalSectionStart(pPmicCoreHandle);
        pmicStatus = Pmic_commIntf_recvByte(
                                       pPmicCoreHandle,
                                       pPwrRsrcRegCfg[pwrRsrcIndex].ctrlRegAddr,
                                       &regData);

        if(PMIC_ST_SUCCESS == pmicStatus)
        {
            /* Selects output pull-down resistor for LDO regulator */
            Pmic_setBitField(&regData,
                             PMIC_LDOX_CTRL_LDOX_PLDN_SHIFT,
                             PMIC_LDOX_CTRL_LDOX_PLDN_MASK,
                             ldoPullDownSel);

            pmicStatus = Pmic_commIntf_sendByte(
                                      pPmicCoreHandle,
                                      pPwrRsrcRegCfg[pwrRsrcIndex].ctrlRegAddr,
                                      regData);
        }

        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    return pmicStatus;
}

/*!
 * \brief   Get the resistor value for output pull-down
 *          resistor for LDO/VMON
 */
static int32_t Pmic_powerGetLdoPullDownSel(Pmic_CoreHandle_t *pPmicCoreHandle,
                                           uint16_t           pwrRsrc,
                                           uint8_t           *pLdoPullDownSel)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData    = 0U;
    uint8_t pwrRsrcIndex;
    Pmic_powerRsrcRegCfg_t *pPwrRsrcRegCfg = NULL;

    pwrRsrcIndex = Pmic_powerGetPwrRsrcId(pwrRsrc);

    /* Get PMIC power resources register configuration */
    pmicStatus = Pmic_powerGetRsrcRegCfg(pPmicCoreHandle->pmicDeviceType,
                                         &pPwrRsrcRegCfg);
    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        Pmic_criticalSectionStart(pPmicCoreHandle);
        pmicStatus = Pmic_commIntf_recvByte(
                                       pPmicCoreHandle,
                                       pPwrRsrcRegCfg[pwrRsrcIndex].ctrlRegAddr,
                                       &regData);
        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        /* Get output pull-down resistor for LDO/VMON */
        *pLdoPullDownSel = Pmic_getBitField(regData,
                                            PMIC_LDOX_CTRL_LDOX_PLDN_SHIFT,
                                            PMIC_LDOX_CTRL_LDOX_PLDN_MASK);
    }

    return pmicStatus;
}

/**
 * \brief   This function is used to set the powergood level for the
 *          VCCA pin.
 */
static int32_t Pmic_powerSetVccaPwrGudLvl(Pmic_CoreHandle_t *pPmicCoreHandle,
                                          bool               vccaPwrGudLvl)
{
    int32_t  pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData     = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);

    /* Reading the Powergood level value */
    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        PMIC_VCCA_PG_WINDOW_REGADDR,
                                        &regData);

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        Pmic_setBitField(&regData,
                         PMIC_VCCA_PG_WINDOW_VCCA_PG_SET_SHIFT,
                         PMIC_VCCA_PG_WINDOW_VCCA_PG_SET_MASK,
                         vccaPwrGudLvl);

        pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                            PMIC_VCCA_PG_WINDOW_REGADDR,
                                            regData);
    }

    Pmic_criticalSectionStop(pPmicCoreHandle);
    return pmicStatus;
}

/**
 * \brief   This function is used to validate Power Good threshold level
 */
static int32_t Pmic_validateThreshold(uint8_t pmicDeviceType,
                                      uint8_t pwrGudThresholdLvl)
{
    int32_t status = PMIC_ST_SUCCESS;

    switch(pmicDeviceType)
    {
        case PMIC_DEV_LEO_TPS6594X:
                if(pwrGudThresholdLvl >
                                 PMIC_TPS6594X_PG_OV_UV_THRESHOLD_LVL_100_OR_10)
                {
                    status = PMIC_ST_ERR_INV_PARAM;
                }
                break;
            case PMIC_DEV_HERA_LP8764X:
                if(pwrGudThresholdLvl >
                                 PMIC_LP8764X_PG_OV_UV_THRESHOLD_LVL_100_OR_10)
                {
                    status = PMIC_ST_ERR_INV_PARAM;
                }
                break;
            default:
                status = PMIC_ST_ERR_INV_PARAM;
                break;
    }

    return status;
}
/*!
 * \brief   Set Power Good low threshold level for power resources.
 */
static int32_t Pmic_powerSetLowThreshold(
                                       Pmic_CoreHandle_t *pPmicCoreHandle,
                                       uint8_t            pwrRsrc,
                                       uint8_t            pwrGudUvThresholdLvl)
{
    int32_t  pmicStatus = PMIC_ST_SUCCESS;
    uint8_t  regData    = 0U;
    Pmic_powerRsrcRegCfg_t *pPwrRsrcRegCfg = NULL;

    /* Get PMIC power resources register configuration */
    pmicStatus = Pmic_powerGetRsrcRegCfg(pPmicCoreHandle->pmicDeviceType,
                                         &pPwrRsrcRegCfg);

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        Pmic_validateThreshold(pPmicCoreHandle->pmicDeviceType,
                               pwrGudUvThresholdLvl);
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        Pmic_criticalSectionStart(pPmicCoreHandle);
        /* Setting the PGOOD LOW threshold value */
        pmicStatus = Pmic_commIntf_recvByte(
                                       pPmicCoreHandle,
                                       pPwrRsrcRegCfg[pwrRsrc].pgWindowRegAddr,
                                       &regData);

        if(PMIC_ST_SUCCESS == pmicStatus)
        {
            Pmic_setBitField(
                   &regData,
                   PMIC_POWER_RESOURCEX_PG_WINDOW_POWER_RESOURCEX_UV_THR_SHIFT,
                   PMIC_POWER_RESOURCEX_PG_WINDOW_POWER_RESOURCEX_UV_THR_MASK,
                   pwrGudUvThresholdLvl);
            pmicStatus = Pmic_commIntf_sendByte(
                                        pPmicCoreHandle,
                                        pPwrRsrcRegCfg[pwrRsrc].pgWindowRegAddr,
                                        regData);
        }

        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    return pmicStatus;
}

/*!
 * \brief   Get Power Good low threshold level for power resources.
 */
static int32_t Pmic_powerGetLowThreshold(
                                      Pmic_CoreHandle_t *pPmicCoreHandle,
                                      uint8_t            pwrRsrc,
                                      uint8_t           *pPwrGudUvThresholdLvl)
{
    int32_t  pmicStatus = PMIC_ST_SUCCESS;
    uint8_t  regData    = 0U;
    Pmic_powerRsrcRegCfg_t *pPwrRsrcRegCfg = NULL;

    /* Get PMIC power resources register configuration */
    pmicStatus = Pmic_powerGetRsrcRegCfg(pPmicCoreHandle->pmicDeviceType,
                                         &pPwrRsrcRegCfg);

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        Pmic_criticalSectionStart(pPmicCoreHandle);

        /* Getting the PGOOD LOW threshold value */
        pmicStatus = Pmic_commIntf_recvByte(
                                        pPmicCoreHandle,
                                        pPwrRsrcRegCfg[pwrRsrc].pgWindowRegAddr,
                                        &regData);
        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        *pPwrGudUvThresholdLvl =
          Pmic_getBitField(
                  regData,
                  PMIC_POWER_RESOURCEX_PG_WINDOW_POWER_RESOURCEX_UV_THR_SHIFT,
                  PMIC_POWER_RESOURCEX_PG_WINDOW_POWER_RESOURCEX_UV_THR_MASK);
    }

    return pmicStatus;
}

/*!
 * \brief   Set Power Good High threshold level for power resources.
 */
static int32_t Pmic_powerSetHighThreshold(Pmic_CoreHandle_t *pPmicCoreHandle,
                                          uint8_t            pwrRsrc,
                                          uint8_t            powerGoodOvThr)
{
    int32_t  pmicStatus = PMIC_ST_SUCCESS;
    uint8_t  regData    = 0U;
    Pmic_powerRsrcRegCfg_t *pPwrRsrcRegCfg = NULL;

    /* Get PMIC power resources register configuration */
    pmicStatus = Pmic_powerGetRsrcRegCfg(pPmicCoreHandle->pmicDeviceType,
                                         &pPwrRsrcRegCfg);

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        Pmic_validateThreshold(pPmicCoreHandle->pmicDeviceType,
                               powerGoodOvThr);
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        Pmic_criticalSectionStart(pPmicCoreHandle);

        /* Setting the PGOOD HIGH threshold value */
        pmicStatus = Pmic_commIntf_recvByte(
                                       pPmicCoreHandle,
                                       pPwrRsrcRegCfg[pwrRsrc].pgWindowRegAddr,
                                       &regData);

        if(PMIC_ST_SUCCESS == pmicStatus)
        {
            Pmic_setBitField(
                  &regData,
                  PMIC_POWER_RESOURCEX_PG_WINDOW_POWER_RESOURCEX_OV_THR_SHIFT,
                  PMIC_POWER_RESOURCEX_PG_WINDOW_POWER_RESOURCEX_OV_THR_MASK,
                  powerGoodOvThr);
            pmicStatus = Pmic_commIntf_sendByte(
                                       pPmicCoreHandle,
                                       pPwrRsrcRegCfg[pwrRsrc].pgWindowRegAddr,
                                       regData);
        }

        Pmic_criticalSectionStop(pPmicCoreHandle);

    }

    return pmicStatus;
}

/*!
 * \brief   Get Power Good High threshold level for power resources.
 */
static int32_t Pmic_powerGetHighThreshold(Pmic_CoreHandle_t *pPmicCoreHandle,
                                          uint8_t            pwrRsrc,
                                          uint8_t           *pPowerGoodOvThr)
{
    int32_t  pmicStatus = PMIC_ST_SUCCESS;
    uint8_t  regData    = 0U;
    Pmic_powerRsrcRegCfg_t *pPwrRsrcRegCfg = NULL;

    /* Get PMIC power resources register configuration */
    pmicStatus = Pmic_powerGetRsrcRegCfg(pPmicCoreHandle->pmicDeviceType,
                                         &pPwrRsrcRegCfg);

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        Pmic_criticalSectionStart(pPmicCoreHandle);
        /* Setting the PGOOD HIGH threshold value */
        pmicStatus = Pmic_commIntf_recvByte(
                                        pPmicCoreHandle,
                                        pPwrRsrcRegCfg[pwrRsrc].pgWindowRegAddr,
                                        &regData);
        Pmic_criticalSectionStop(pPmicCoreHandle);

    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        *pPowerGoodOvThr =
           Pmic_getBitField(
                  regData,
                  PMIC_POWER_RESOURCEX_PG_WINDOW_POWER_RESOURCEX_OV_THR_SHIFT,
                  PMIC_POWER_RESOURCEX_PG_WINDOW_POWER_RESOURCEX_OV_THR_MASK);
    }

    return pmicStatus;
}

/**
 * \brief   This function is used to get the powergood level for the
 *          VCCA pin.
 */
static int32_t Pmic_powerGetVccaPwrGudLvl(Pmic_CoreHandle_t *pPmicCoreHandle,
                                          bool              *pVccaPwrGudLvl)
{
    int32_t  pmicStatus = PMIC_ST_SUCCESS;
    uint8_t  regData    = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);

    /* Reading the Powergood level value */
    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        PMIC_VCCA_PG_WINDOW_REGADDR,
                                        &regData);
    Pmic_criticalSectionStop(pPmicCoreHandle);

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        *pVccaPwrGudLvl = Pmic_getBitField(
                                         regData,
                                         PMIC_VCCA_PG_WINDOW_VCCA_PG_SET_SHIFT,
                                         PMIC_VCCA_PG_WINDOW_VCCA_PG_SET_MASK);

    }

    return pmicStatus;
}

/*!
 * \brief   This function is used to get the required Bit Position and Bit Mask
 *           of VMON enable/disable status for all power resources
 */
static int32_t Pmic_powerGetVmonEnBitPos(uint8_t   pmicDeviceType,
                                         uint16_t  pwrRsrc,
                                         uint8_t   *pBitPos,
                                         uint8_t   *pBitMask)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t pwrRsrcType;

    pwrRsrcType = Pmic_powerGetPwrRsrcType(pwrRsrc);
    switch(pmicDeviceType)
    {
        case PMIC_DEV_LEO_TPS6594X:
            if((PMIC_TPS6594X_POWER_RESOURCE_TYPE_BUCK == pwrRsrcType) ||
               (PMIC_TPS6594X_POWER_RESOURCE_TYPE_LDO  == pwrRsrcType))
            {
               *pBitPos = PMIC_REGULATOR_CTRL_REGULATOR_VMON_EN_SHIFT;
               *pBitMask = PMIC_REGULATOR_CTRL_REGULATOR_VMON_EN_MASK;
            }
            else if(PMIC_TPS6594X_POWER_RESOURCE_TYPE_VCCA == pwrRsrcType)
            {
               *pBitPos = PMIC_VCCA_VMON_CTRL_VCCA_VMON_EN_SHIFT;
               *pBitMask = PMIC_VCCA_VMON_CTRL_VCCA_VMON_EN_MASK;
            }
            else
            {
                status = PMIC_ST_ERR_INV_PARAM;
            }

            break;
        case PMIC_DEV_HERA_LP8764X:
            if(PMIC_LP8764X_POWER_RESOURCE_TYPE_BUCK == pwrRsrcType)
            {
               *pBitPos = PMIC_REGULATOR_CTRL_REGULATOR_VMON_EN_SHIFT;
               *pBitMask = PMIC_REGULATOR_CTRL_REGULATOR_VMON_EN_MASK;
            }
            else if(PMIC_LP8764X_POWER_RESOURCE_TYPE_VMON == pwrRsrcType)
            {
                if(PMIC_LP8764X_POWER_SOURCE_VMON1 == pwrRsrc)
                {
                    *pBitPos = PMIC_VCCA_VMON_CTRL_VMON1_EN_SHIFT;
                    *pBitMask = PMIC_VCCA_VMON_CTRL_VMON1_EN_MASK;
                }
                else if(PMIC_LP8764X_POWER_SOURCE_VMON2 == pwrRsrc)
                {
                    *pBitPos = PMIC_VCCA_VMON_CTRL_VMON2_EN_SHIFT;
                    *pBitMask = PMIC_VCCA_VMON_CTRL_VMON2_EN_MASK;
                }
                else
                {
                    status = PMIC_ST_ERR_INV_PARAM;
                }
            }
            else if(PMIC_LP8764X_POWER_RESOURCE_TYPE_VCCA == pwrRsrcType)
            {
               *pBitPos = PMIC_VCCA_VMON_CTRL_VCCA_VMON_EN_SHIFT;
               *pBitMask = PMIC_VCCA_VMON_CTRL_VCCA_VMON_EN_MASK;
            }
            else
            {
                status = PMIC_ST_ERR_INV_PARAM;
            }

            break;
        default:
            status = PMIC_ST_ERR_INV_PARAM;
            break;
    }

    return status;
}

/*!
 * \brief   This function is used to enable/disable the VMON for
 *          all power resource
 */
static int32_t Pmic_powerSetVmonEn(Pmic_CoreHandle_t *pPmicCoreHandle,
                                   uint16_t           pwrRsrc,
                                   bool               vmonEn)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    uint8_t bitPos  = 0U;
    uint8_t pwrRsrcIndex = 0U;
    Pmic_powerRsrcRegCfg_t *pPwrRsrcRegCfg = NULL;
    uint8_t bitMask = 0U;

    pwrRsrcIndex = Pmic_powerGetPwrRsrcId(pwrRsrc);

    /* Get PMIC power resources register configuration */
    pmicStatus = Pmic_powerGetRsrcRegCfg(pPmicCoreHandle->pmicDeviceType,
                                         &pPwrRsrcRegCfg);

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        Pmic_criticalSectionStart(pPmicCoreHandle);
        pmicStatus = Pmic_commIntf_recvByte(
                                       pPmicCoreHandle,
                                       pPwrRsrcRegCfg[pwrRsrcIndex].ctrlRegAddr,
                                       &regData);

        if(PMIC_ST_SUCCESS == pmicStatus)
        {
            pmicStatus = Pmic_powerGetVmonEnBitPos(
                                                pPmicCoreHandle->pmicDeviceType,
                                                pwrRsrc,
                                                &bitPos,
                                                &bitMask);

            Pmic_setBitField(&regData, bitPos, bitMask, vmonEn);
       }

        if(PMIC_ST_SUCCESS == pmicStatus)
        {
            pmicStatus = Pmic_commIntf_sendByte(
                                       pPmicCoreHandle,
                                       pPwrRsrcRegCfg[pwrRsrcIndex].ctrlRegAddr,
                                       regData);
        }

        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    return pmicStatus;
}

/*!
 * \brief   This function is used to get enable/disable status of the VMON for
 *          all power resource
 */
static int32_t Pmic_powerGetvmonEn(Pmic_CoreHandle_t *pPmicCoreHandle,
                                   uint16_t           pwrRsrc,
                                   bool              *pVmonEn)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    uint8_t bitPos  = 0U;
    uint8_t pwrRsrcIndex = 0U;
    Pmic_powerRsrcRegCfg_t *pPwrRsrcRegCfg = NULL;
    uint8_t bitMask = 0U;

    pwrRsrcIndex = Pmic_powerGetPwrRsrcId(pwrRsrc);

    /* Get PMIC power resources register configuration */
    pmicStatus = Pmic_powerGetRsrcRegCfg(pPmicCoreHandle->pmicDeviceType,
                                         &pPwrRsrcRegCfg);

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        Pmic_criticalSectionStart(pPmicCoreHandle);
        pmicStatus = Pmic_commIntf_recvByte(
                                       pPmicCoreHandle,
                                       pPwrRsrcRegCfg[pwrRsrcIndex].ctrlRegAddr,
                                       &regData);

        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        pmicStatus = Pmic_powerGetVmonEnBitPos(pPmicCoreHandle->pmicDeviceType,
                                               pwrRsrc,
                                               &bitPos,
                                               &bitMask);

        *pVmonEn = Pmic_getBitField(regData, bitPos, bitMask);
    }

    return pmicStatus;
}

/*!
 * \brief   This function is used to validate the Rail group limit for all
 *          power resources
 */
static int32_t Pmic_powerValidateRailGrpLimit(uint8_t pmicDeviceType,
                                              uint8_t railGrpSel)
{
    int32_t status = PMIC_ST_SUCCESS;

    switch(pmicDeviceType)
    {
        case PMIC_DEV_LEO_TPS6594X:
            if(railGrpSel > PMIC_TPS6594X_POWER_RAIL_SEL_OTHER)
            {
                status = PMIC_ST_ERR_INV_PARAM;
            }
            break;
        case PMIC_DEV_HERA_LP8764X:
            if(railGrpSel > PMIC_LP8764X_POWER_RAIL_SEL_OTHER)
            {
                status = PMIC_ST_ERR_INV_PARAM;
            }
            break;
        default:
            status = PMIC_ST_ERR_INV_PARAM;
            break;
    }

    return status;
}

/*!
 * \brief   This function is used to select the Rail group of power
 *          resource of the PMIC
 */
static int32_t Pmic_powerSetRailGrpSel(Pmic_CoreHandle_t *pPmicCoreHandle,
                                       uint16_t           pwrRsrc,
                                       uint8_t            railGrpSel)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regAddr = 0U;
    uint8_t regData = 0U;
    uint8_t pwrRsrcIndex = 0U;
    uint8_t bitMask = 0U;
    Pmic_powerRsrcRegCfg_t *pPwrRsrcRegCfg = NULL;

    pwrRsrcIndex = Pmic_powerGetPwrRsrcId(pwrRsrc);

    /* Get PMIC power resources register configuration */
    pmicStatus = Pmic_powerGetRsrcRegCfg(pPmicCoreHandle->pmicDeviceType,
                                         &pPwrRsrcRegCfg);
    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        regAddr = pPwrRsrcRegCfg[pwrRsrcIndex].railGrpRegAddr;
        pmicStatus = Pmic_powerValidateRailGrpLimit(
                                                pPmicCoreHandle->pmicDeviceType,
                                                railGrpSel);
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        Pmic_criticalSectionStart(pPmicCoreHandle);
        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                            regAddr,
                                            &regData);
        if(PMIC_ST_SUCCESS == pmicStatus)
        {
            bitMask = (uint8_t)(PMIC_RAIL_SEL_X_PWR_RSRC_X_GRP_SEL_BITFIELD \
                      << pPwrRsrcRegCfg[pwrRsrcIndex].railGrpBitShiftVal);

            Pmic_setBitField(&regData,
                             pPwrRsrcRegCfg[pwrRsrcIndex].railGrpBitShiftVal,
                             bitMask,
                             railGrpSel);

            pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                                regAddr,
                                                regData);
        }

        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    return pmicStatus;
}

/*!
 * \brief   This function is used to get the Rail group of power
 *          regulator of the PMIC
 */
static int32_t Pmic_powerGetRailGrpSel(Pmic_CoreHandle_t *pPmicCoreHandle,
                                       uint16_t           pwrRsrc,
                                       uint8_t           *pRailGrpSel)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    uint8_t pwrRsrcIndex = 0U;
    uint8_t bitMask = 0U;
    Pmic_powerRsrcRegCfg_t *pPwrRsrcRegCfg = NULL;

    pwrRsrcIndex = Pmic_powerGetPwrRsrcId(pwrRsrc);

    /* Get PMIC power resources register configuration */
    pmicStatus = Pmic_powerGetRsrcRegCfg(pPmicCoreHandle->pmicDeviceType,
                                         &pPwrRsrcRegCfg);
    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        Pmic_criticalSectionStart(pPmicCoreHandle);
        pmicStatus = Pmic_commIntf_recvByte(
                                    pPmicCoreHandle,
                                    pPwrRsrcRegCfg[pwrRsrcIndex].railGrpRegAddr,
                                    &regData);
        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        bitMask = (uint8_t)(PMIC_RAIL_SEL_X_PWR_RSRC_X_GRP_SEL_BITFIELD \
                  << pPwrRsrcRegCfg[pwrRsrcIndex].railGrpBitShiftVal);

        *pRailGrpSel = Pmic_getBitField(
                                regData,
                                pPwrRsrcRegCfg[pwrRsrcIndex].railGrpBitShiftVal,
                                bitMask);
    }

    return pmicStatus;
}

/*!
 * \brief   This function is used to Select OV/UV voltage monitoring range for
 *          VMON2/VMON1
 */
static int32_t Pmic_powerSetVmonRange(Pmic_CoreHandle_t *pPmicCoreHandle,
                                      uint16_t           pwrRsrc,
                                      bool               vmonRange)

{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    uint8_t pwrRsrcIndex = 0U;
    Pmic_powerRsrcRegCfg_t *pPwrRsrcRegCfg = NULL;

    pwrRsrcIndex = Pmic_powerGetPwrRsrcId(pwrRsrc);

    /* Get PMIC power resources register configuration */
    pmicStatus = Pmic_powerGetRsrcRegCfg(pPmicCoreHandle->pmicDeviceType,
                                         &pPwrRsrcRegCfg);

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        Pmic_criticalSectionStart(pPmicCoreHandle);
        pmicStatus = Pmic_commIntf_recvByte(
                                   pPmicCoreHandle,
                                   pPwrRsrcRegCfg[pwrRsrcIndex].pgWindowRegAddr,
                                   &regData);

        if(PMIC_ST_SUCCESS == pmicStatus)
        {
            Pmic_setBitField(&regData,
                             PMIC_VMON_PG_WINDOW_VMON_RANGE_SHIFT,
                             PMIC_VMON_PG_WINDOW_VMON_RANGE_MASK,
                             vmonRange);

            pmicStatus = Pmic_commIntf_sendByte(
                                   pPmicCoreHandle,
                                   pPwrRsrcRegCfg[pwrRsrcIndex].pgWindowRegAddr,
                                   regData);
        }

        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    return pmicStatus;
}

/*!
 * \brief   This function is to configure the various control parameters for
 *          power resources.
 */
static int32_t Pmic_powerSetPwrResourceCtrlCfg(
                                       Pmic_CoreHandle_t       *pPmicCoreHandle,
                                       uint16_t                 pwrRsrc,
                                       Pmic_PowerResourceCfg_t  pwrRsrcCfg)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    if(true == pmic_validParamCheck(pwrRsrcCfg.validParams,
                                    PMIC_CFG_REGULATOR_VMON_RV_SEL_VALID))
    {
        pmicStatus = Pmic_powerValidatecfgDataPwrRsrctype(
                                           PMIC_CFG_REGULATOR_VMON_RV_SEL_VALID,
                                           pwrRsrc,
                                           pPmicCoreHandle->pmicDeviceType);

        if(PMIC_ST_SUCCESS == pmicStatus)
        {
            /* Enable/Disable residual voltage checking  */
            pmicStatus = Pmic_powerSetRvCheckEn(pPmicCoreHandle,
                                                pwrRsrc,
                                                pwrRsrcCfg.rvCheckEn);
        }
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (true == pmic_validParamCheck(pwrRsrcCfg.validParams,
                                     PMIC_CFG_REGULATOR_BUCK_PLDN_EN_VALID)))
    {
        pmicStatus = Pmic_powerValidatecfgDataPwrRsrctype(
                                          PMIC_CFG_REGULATOR_BUCK_PLDN_EN_VALID,
                                          pwrRsrc,
                                          pPmicCoreHandle->pmicDeviceType);

        if(PMIC_ST_SUCCESS == pmicStatus)
        {
            /* Enable/Disable pull down resistor for Buck */
            pmicStatus = Pmic_powerSetBuckPullDownEn(
                                              pPmicCoreHandle,
                                              pwrRsrc,
                                              pwrRsrcCfg.buckPullDownEn);
        }
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (true == pmic_validParamCheck(pwrRsrcCfg.validParams,
                                     PMIC_CFG_REGULATOR_BUCK_VOUT_SEL_VALID)))
    {
        pmicStatus = Pmic_powerValidatecfgDataPwrRsrctype(
                                         PMIC_CFG_REGULATOR_BUCK_VOUT_SEL_VALID,
                                         pwrRsrc,
                                         pPmicCoreHandle->pmicDeviceType);

        if(PMIC_ST_SUCCESS == pmicStatus)
        {
            /* Select output voltage register for buck */
            pmicStatus = Pmic_powerSetBuckVoutSel(pPmicCoreHandle,
                                                  pwrRsrc,
                                                  pwrRsrcCfg.buckVoutSel);
        }
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (true == pmic_validParamCheck(pwrRsrcCfg.validParams,
                                     PMIC_CFG_REGULATOR_BUCK_FPWM_VALID)))
    {
        pmicStatus = Pmic_powerValidatecfgDataPwrRsrctype(
                                             PMIC_CFG_REGULATOR_BUCK_FPWM_VALID,
                                             pwrRsrc,
                                             pPmicCoreHandle->pmicDeviceType);

        if(PMIC_ST_SUCCESS == pmicStatus)
        {
            /* Select PWM or Auto Mode */
            pmicStatus = Pmic_powerSetBuckFpwmMode(
                                                pPmicCoreHandle,
                                                pwrRsrc,
                                                pwrRsrcCfg.buckFpwmMode);
        }
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (true == pmic_validParamCheck(pwrRsrcCfg.validParams,
                                     PMIC_CFG_REGULATOR_BUCK_PWM_MP_VALID)))
    {
        pmicStatus = Pmic_powerValidatecfgDataPwrRsrctype(
                                          PMIC_CFG_REGULATOR_BUCK_PWM_MP_VALID,
                                          pwrRsrc,
                                          pPmicCoreHandle->pmicDeviceType);
        if(PMIC_ST_SUCCESS == pmicStatus)
        {
            /* Select Multi phase with PWM or Auto Mode */
            pmicStatus = Pmic_powerSetBuckFpwmMpMode(
                                               pPmicCoreHandle,
                                               pwrRsrc,
                                               pwrRsrcCfg.buckFpwmMpMode);
        }
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (true == pmic_validParamCheck(pwrRsrcCfg.validParams,
                                    PMIC_CFG_REGULATOR_LDO_SLOW_RAMP_EN_VALID)))
    {
        pmicStatus = Pmic_powerValidatecfgDataPwrRsrctype(
                                      PMIC_CFG_REGULATOR_LDO_SLOW_RAMP_EN_VALID,
                                      pwrRsrc,
                                      pPmicCoreHandle->pmicDeviceType);
        if(PMIC_ST_SUCCESS == pmicStatus)
        {
            /* Enable/Disable Slow Ramp for LDO */
            pmicStatus = Pmic_powerSetLdoSlowRampEn(
                                               pPmicCoreHandle,
                                               pwrRsrc,
                                               pwrRsrcCfg.ldoSlowRampEn);
        }
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (true == pmic_validParamCheck(pwrRsrcCfg.validParams,
                                     PMIC_CFG_REGULATOR_LDO_PLDN_SEL_VALID)))
    {
        pmicStatus = Pmic_powerValidatecfgDataPwrRsrctype(
                                          PMIC_CFG_REGULATOR_LDO_PLDN_SEL_VALID,
                                          pwrRsrc,
                                          pPmicCoreHandle->pmicDeviceType);
        if(PMIC_ST_SUCCESS == pmicStatus)
        {
            /* Set resistor value for output pull-down for LDO */
            pmicStatus = Pmic_powerSetLdoPullDownSel(
                                              pPmicCoreHandle,
                                              pwrRsrc,
                                              pwrRsrcCfg.ldoPullDownSel);
        }
    }

    return pmicStatus;
}

/*!
 * \brief   This function is to configure the various configuration parameters
 *          for BUCK/VMON
 */
static int32_t Pmic_powerSetBuckVmonConfig(
                                       Pmic_CoreHandle_t      *pPmicCoreHandle,
                                       uint16_t                pwrRsrc,
                                       Pmic_PowerResourceCfg_t pwrRsrcCfg)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    if(true == pmic_validParamCheck(pwrRsrcCfg.validParams,
                                    PMIC_CFG_REGULATOR_BUCK_ILIM_VALID))
    {
        pmicStatus = Pmic_powerValidatecfgDataPwrRsrctype(
                                             PMIC_CFG_REGULATOR_BUCK_ILIM_VALID,
                                             pwrRsrc,
                                             pPmicCoreHandle->pmicDeviceType);
        if(PMIC_ST_SUCCESS == pmicStatus)
        {
            /* Set Switch peak current limit for BUCK */
            pmicStatus = Pmic_powerSetBuckCurrentLimit(
                                            pPmicCoreHandle,
                                            pwrRsrc,
                                            pwrRsrcCfg.buckCurrentLimit);
        }
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (true == pmic_validParamCheck(pwrRsrcCfg.validParams,
                                 PMIC_CFG_REGULATOR_BUCK_VMON_SLEW_RATE_VALID)))
    {
        pmicStatus = Pmic_powerValidatecfgDataPwrRsrctype(
                                   PMIC_CFG_REGULATOR_BUCK_VMON_SLEW_RATE_VALID,
                                   pwrRsrc,
                                   pPmicCoreHandle->pmicDeviceType);

        if(PMIC_ST_SUCCESS == pmicStatus)
        {
            /* Set Output voltage slew rate for BUCK/VMON */
            pmicStatus = Pmic_powerSetBuckVmonSlewRate(
                                             pPmicCoreHandle,
                                             pwrRsrc,
                                             pwrRsrcCfg.buckVmonSlewRate);
        }
    }

    return pmicStatus;
}

/*!
 * \brief   This function is to configure the various power-good parameters for
 *          power resources and vmon
 */
static int32_t Pmic_powerSetPwrRsrcePgoodConfig(
                                      Pmic_CoreHandle_t       *pPmicCoreHandle,
                                      uint16_t                 pwrRsrc,
                                      Pmic_PowerResourceCfg_t  pwrRsrcCfg)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (true == pmic_validParamCheck(pwrRsrcCfg.validParams,
                                     PMIC_CFG_VCCA_PWR_GOOD_LVL_VALID)))
    {
        pmicStatus = Pmic_powerValidatecfgDataPwrRsrctype(
                                          PMIC_CFG_VCCA_PWR_GOOD_LVL_VALID,
                                          pwrRsrc,
                                          pPmicCoreHandle->pmicDeviceType);
        if(PMIC_ST_SUCCESS == pmicStatus)
        {
            /* Set Powergood level for VCCA pin */
            pmicStatus = Pmic_powerSetVccaPwrGudLvl(pPmicCoreHandle,
                                                    pwrRsrcCfg.vccaPwrGudLvl);
        }
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (true == pmic_validParamCheck(pwrRsrcCfg.validParams,
                                     PMIC_CFG_VMON_RANGE_VALID)))
    {
        pmicStatus = Pmic_powerValidatecfgDataPwrRsrctype(
                                               PMIC_CFG_VMON_RANGE_VALID,
                                               pwrRsrc,
                                               pPmicCoreHandle->pmicDeviceType);
        if(PMIC_ST_SUCCESS == pmicStatus)
        {
            /* OV/UV voltage monitoring range selection for VMON */
            pmicStatus = Pmic_powerSetVmonRange(pPmicCoreHandle,
                                                pwrRsrc,
                                                pwrRsrcCfg.vmonRange);
        }
    }

    if(pmic_validParamCheck(pwrRsrcCfg.validParams,
                            PMIC_CFG_PWR_RESOURCE_PG_UV_THRESHOLD_LVL_VALID))
    {
        /* Set Powergood low threshold level for power resources */
        pmicStatus = Pmic_powerSetLowThreshold(pPmicCoreHandle,
                                               pwrRsrc,
                                               pwrRsrcCfg.pgUvThresholdLvl);

   }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (pmic_validParamCheck(pwrRsrcCfg.validParams,
                             PMIC_CFG_PWR_RESOURCE_PG_OV_THRESHOLD_LVL_VALID)))
    {
        /* Set Powergood high threshold level for power resources */
        pmicStatus = Pmic_powerSetHighThreshold(pPmicCoreHandle,
                                                pwrRsrc,
                                                pwrRsrcCfg.pgOvThresholdLvl);

    }

    return pmicStatus;
}

/*!
 * \brief   This function is to get the various control parameters for
 *          regulators and VMON
 */
static int32_t Pmic_powerGetPwrResourceCtrlCfg(
                                       Pmic_CoreHandle_t       *pPmicCoreHandle,
                                       uint16_t                 pwrRsrc,
                                       Pmic_PowerResourceCfg_t *pPwrRsrcCfg)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    if(true == pmic_validParamCheck(pPwrRsrcCfg->validParams,
                                    PMIC_CFG_REGULATOR_VMON_RV_SEL_VALID))
    {
        pmicStatus = Pmic_powerValidatecfgDataPwrRsrctype(
                                          PMIC_CFG_REGULATOR_VMON_RV_SEL_VALID,
                                          pwrRsrc,
                                          pPmicCoreHandle->pmicDeviceType);
        if(PMIC_ST_SUCCESS == pmicStatus)
        {
            /* Get Enable/Disable status for residual voltage checking */
            pmicStatus = Pmic_powerGetRvCheckEn(pPmicCoreHandle,
                                                pwrRsrc,
                                                &(pPwrRsrcCfg->rvCheckEn));
        }
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (true == pmic_validParamCheck(pPwrRsrcCfg->validParams,
                                     PMIC_CFG_REGULATOR_BUCK_PLDN_EN_VALID)))
    {
        pmicStatus = Pmic_powerValidatecfgDataPwrRsrctype(
                                          PMIC_CFG_REGULATOR_BUCK_PLDN_EN_VALID,
                                          pwrRsrc,
                                          pPmicCoreHandle->pmicDeviceType);

        if(PMIC_ST_SUCCESS == pmicStatus)
        {
            /* Get Enable/Disable status of pull down resistor for Buck */
            pmicStatus = Pmic_powerGetBuckPullDownEn(
                                           pPmicCoreHandle,
                                           pwrRsrc,
                                           &(pPwrRsrcCfg->buckPullDownEn));
        }
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (true == pmic_validParamCheck(pPwrRsrcCfg->validParams,
                                     PMIC_CFG_VMON_EN_VALID)))
    {
        if(PMIC_ST_SUCCESS == pmicStatus)
        {
            /* Get Enable/Disable status for Voltage monitor */
            pmicStatus = Pmic_powerGetvmonEn(pPmicCoreHandle,
                                             pwrRsrc,
                                             &(pPwrRsrcCfg->vmonEn));
        }
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (true == pmic_validParamCheck(pPwrRsrcCfg->validParams,
                                     PMIC_CFG_REGULATOR_BUCK_VOUT_SEL_VALID)))
    {
        pmicStatus = Pmic_powerValidatecfgDataPwrRsrctype(
                                         PMIC_CFG_REGULATOR_BUCK_VOUT_SEL_VALID,
                                         pwrRsrc,
                                         pPmicCoreHandle->pmicDeviceType);

        if(PMIC_ST_SUCCESS == pmicStatus)
        {
             /* Get output voltage register for buck*/
            pmicStatus = Pmic_powerGetBuckVoutSel(pPmicCoreHandle,
                                                  pwrRsrc,
                                                  &(pPwrRsrcCfg->buckVoutSel));
        }
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (true == pmic_validParamCheck(pPwrRsrcCfg->validParams,
                                     PMIC_CFG_REGULATOR_BUCK_FPWM_VALID)))
    {
        pmicStatus = Pmic_powerValidatecfgDataPwrRsrctype(
                                             PMIC_CFG_REGULATOR_BUCK_FPWM_VALID,
                                             pwrRsrc,
                                             pPmicCoreHandle->pmicDeviceType);

        if(PMIC_ST_SUCCESS == pmicStatus)
        {
            /* Get PWM or Auto Mode */
            pmicStatus = Pmic_powerGetBuckFpwmMode(
                                                  pPmicCoreHandle,
                                                  pwrRsrc,
                                                  &(pPwrRsrcCfg->buckFpwmMode));
        }
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (true == pmic_validParamCheck(pPwrRsrcCfg->validParams,
                                     PMIC_CFG_REGULATOR_BUCK_PWM_MP_VALID)))
    {
        pmicStatus = Pmic_powerValidatecfgDataPwrRsrctype(
                                           PMIC_CFG_REGULATOR_BUCK_PWM_MP_VALID,
                                           pwrRsrc,
                                           pPmicCoreHandle->pmicDeviceType);

        if(PMIC_ST_SUCCESS == pmicStatus)
        {
            /* Get Multi phase with PWM or Auto Mode status */
            pmicStatus = Pmic_powerGetBuckPwmMpMode(
                                                pPmicCoreHandle,
                                                pwrRsrc,
                                                &(pPwrRsrcCfg->buckFpwmMpMode));
        }
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (true == pmic_validParamCheck(pPwrRsrcCfg->validParams,
                                     PMIC_CFG_REGULATOR_EN_VALID)))
    {
        pmicStatus = Pmic_powerValidatecfgDataPwrRsrctype(
                                               PMIC_CFG_REGULATOR_EN_VALID,
                                               pwrRsrc,
                                               pPmicCoreHandle->pmicDeviceType);
        if(PMIC_ST_SUCCESS == pmicStatus)
        {
            /* Get Enable/Disable status for Regulator */
            pmicStatus = Pmic_powerGetRegulatorEn(pPmicCoreHandle,
                                                  pwrRsrc,
                                                  &(pPwrRsrcCfg->regulatorEn));
        }
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (true == pmic_validParamCheck(pPwrRsrcCfg->validParams,
                                    PMIC_CFG_REGULATOR_LDO_SLOW_RAMP_EN_VALID)))
    {
        pmicStatus = Pmic_powerValidatecfgDataPwrRsrctype(
                                      PMIC_CFG_REGULATOR_LDO_SLOW_RAMP_EN_VALID,
                                      pwrRsrc,
                                      pPmicCoreHandle->pmicDeviceType);
        if(PMIC_ST_SUCCESS == pmicStatus)
        {
            /* Get Enable/Disable status of Slow Ramp for LDO */
            pmicStatus = Pmic_powerGetLdoSlowRampEn(
                                                 pPmicCoreHandle,
                                                 pwrRsrc,
                                                 &(pPwrRsrcCfg->ldoSlowRampEn));
        }
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (true == pmic_validParamCheck(pPwrRsrcCfg->validParams,
                                     PMIC_CFG_REGULATOR_LDO_PLDN_SEL_VALID)))
    {
        pmicStatus = Pmic_powerValidatecfgDataPwrRsrctype(
                                          PMIC_CFG_REGULATOR_LDO_PLDN_SEL_VALID,
                                          pwrRsrc,
                                          pPmicCoreHandle->pmicDeviceType);
        if(PMIC_ST_SUCCESS == pmicStatus)
        {
           /* Get resistor value for output pull-down for LDO and VMON */
            pmicStatus = Pmic_powerGetLdoPullDownSel(
                                                pPmicCoreHandle,
                                                pwrRsrc,
                                                &(pPwrRsrcCfg->ldoPullDownSel));
        }
    }

    return pmicStatus;
}

/*!
 * \brief   This function is to get the various configuration parameters for
 *          BUCk/VMON
 */
static int32_t Pmic_powerGetBuckVmonConfig(
                                      Pmic_CoreHandle_t        *pPmicCoreHandle,
                                      uint16_t                  pwrRsrc,
                                      Pmic_PowerResourceCfg_t  *pPwrRsrcCfg)
{
    int32_t  pmicStatus = PMIC_ST_SUCCESS;

    if(true == pmic_validParamCheck(pPwrRsrcCfg->validParams,
                                    PMIC_CFG_REGULATOR_BUCK_ILIM_VALID))
    {
        pmicStatus = Pmic_powerValidatecfgDataPwrRsrctype(
                                             PMIC_CFG_REGULATOR_BUCK_ILIM_VALID,
                                             pwrRsrc,
                                             pPmicCoreHandle->pmicDeviceType);
        if(PMIC_ST_SUCCESS == pmicStatus)
        {
             /* Get Switch peak current limit for BUCK */
            pmicStatus = Pmic_powerGetBuckCurrentLimit(
                                              pPmicCoreHandle,
                                              pwrRsrc,
                                              &(pPwrRsrcCfg->buckCurrentLimit));
        }
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (true == pmic_validParamCheck(pPwrRsrcCfg->validParams,
                                 PMIC_CFG_REGULATOR_BUCK_VMON_SLEW_RATE_VALID)))
    {
        pmicStatus = Pmic_powerValidatecfgDataPwrRsrctype(
                                   PMIC_CFG_REGULATOR_BUCK_VMON_SLEW_RATE_VALID,
                                   pwrRsrc,
                                   pPmicCoreHandle->pmicDeviceType);

        if(PMIC_ST_SUCCESS == pmicStatus)
        {
            /* Get Output voltage slew rate for BUCK, VMON1 and VMON2 */
            pmicStatus = Pmic_powerGetBuckVmonSlewRate(
                                              pPmicCoreHandle,
                                              pwrRsrc,
                                              &(pPwrRsrcCfg->buckVmonSlewRate));
        }
    }

    return pmicStatus;
}

/*!
 * \brief   This function is to get the various power-good parameters for
 *          power resources and vmon
 */
static int32_t Pmic_powerGetPwrRsrcePgoodConfig(
                                      Pmic_CoreHandle_t        *pPmicCoreHandle,
                                      uint16_t                  pwrRsrc,
                                      Pmic_PowerResourceCfg_t  *pPwrRsrcCfg)

{
    int32_t  pmicStatus = PMIC_ST_SUCCESS;

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (true == pmic_validParamCheck(pPwrRsrcCfg->validParams,
                                     PMIC_CFG_VCCA_PWR_GOOD_LVL_VALID)))
    {
        pmicStatus = Pmic_powerValidatecfgDataPwrRsrctype(
                                               PMIC_CFG_VCCA_PWR_GOOD_LVL_VALID,
                                               pwrRsrc,
                                               pPmicCoreHandle->pmicDeviceType);
        if(PMIC_ST_SUCCESS == pmicStatus)
        {
             /* get Powergood level for VCCA pin */
            pmicStatus = Pmic_powerGetVccaPwrGudLvl(
                                                 pPmicCoreHandle,
                                                 &(pPwrRsrcCfg->vccaPwrGudLvl));
        }
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (true == pmic_validParamCheck(pPwrRsrcCfg->validParams,
                                     PMIC_CFG_VMON_RANGE_VALID)))
    {
        pmicStatus = Pmic_powerValidatecfgDataPwrRsrctype(
                                               PMIC_CFG_VMON_RANGE_VALID,
                                               pwrRsrc,
                                               pPmicCoreHandle->pmicDeviceType);
        if(PMIC_ST_SUCCESS == pmicStatus)
        {
            /* Get OV/UV voltage monitoring range selection for VMON */
            pmicStatus = Pmic_powerGetVmonRange(pPmicCoreHandle,
                                                pwrRsrc,
                                                &(pPwrRsrcCfg->vmonRange));
        }
    }

    if(pmic_validParamCheck(pPwrRsrcCfg->validParams,
                             PMIC_CFG_PWR_RESOURCE_PG_UV_THRESHOLD_LVL_VALID))
    {
    /* get Powergood low threshold level for power resources */
        pmicStatus = Pmic_powerGetLowThreshold(
                                              pPmicCoreHandle,
                                              pwrRsrc,
                                              &(pPwrRsrcCfg->pgUvThresholdLvl));
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (pmic_validParamCheck(pPwrRsrcCfg->validParams,
                             PMIC_CFG_PWR_RESOURCE_PG_OV_THRESHOLD_LVL_VALID)))
    {
        /* Get Powergood high threshold level for power resources */
        pmicStatus = Pmic_powerGetHighThreshold(
                                              pPmicCoreHandle,
                                              pwrRsrc,
                                              &(pPwrRsrcCfg->pgOvThresholdLvl));
    }

    return pmicStatus;
}

/*!
 * \brief   This function is to validate the power resource limit for the
 *          specific PMIC device.
 */
static int32_t Pmic_powerValidatePwrRsrcLimit(
                                    const Pmic_CoreHandle_t *pPmicCoreHandle,
                                    uint8_t  pwrRsrcType,
                                    uint16_t pwrRsrc)
{
    int32_t status = PMIC_ST_SUCCESS;

    switch(pPmicCoreHandle->pmicDeviceType)
    {
        case PMIC_DEV_LEO_TPS6594X:
            if(PMIC_TPS6594X_POWER_RESOURCE_TYPE_VCCA == pwrRsrcType)
            {
                if(pwrRsrc != PMIC_TPS6594X_POWER_SOURCE_VCCA)
                {
                    status = PMIC_ST_ERR_INV_PARAM;
                }
            }
            else if(PMIC_TPS6594X_POWER_RESOURCE_TYPE_BUCK == pwrRsrcType)
            {
                if((pwrRsrc > PMIC_TPS6594X_BUCK_MAX) ||
                   (pwrRsrc < PMIC_TPS6594X_BUCK_MIN))
                {
                    status = PMIC_ST_ERR_INV_PARAM;
                }
            }
            else if(PMIC_TPS6594X_POWER_RESOURCE_TYPE_LDO == pwrRsrcType)
            {
                if((pwrRsrc > PMIC_TPS6594X_LDO_MAX) ||
                   (pwrRsrc < PMIC_TPS6594X_LDO_MIN))
                {
                    status = PMIC_ST_ERR_INV_PARAM;
                }
            }
            else
            {
                status = PMIC_ST_ERR_INV_PARAM;
            }

            break;
        case PMIC_DEV_HERA_LP8764X:
            if(PMIC_LP8764X_POWER_RESOURCE_TYPE_VCCA == pwrRsrcType)
           {
                if(pwrRsrc != PMIC_LP8764X_POWER_SOURCE_VCCA)
                {
                    status = PMIC_ST_ERR_INV_PARAM;
                }
           }
           else if(PMIC_LP8764X_POWER_RESOURCE_TYPE_BUCK == pwrRsrcType)
           {
                if((pwrRsrc > PMIC_LP8764X_BUCK_MAX) ||
                   (pwrRsrc < PMIC_LP8764X_BUCK_MIN))
                {
                    status = PMIC_ST_ERR_INV_PARAM;
                }
           }
           else if(PMIC_LP8764X_POWER_RESOURCE_TYPE_VMON == pwrRsrcType)
           {
                if((pwrRsrc > PMIC_LP8764X_VMON_MAX) ||
                   (pwrRsrc < PMIC_LP8764X_VMON_MIN))
                {
                    status = PMIC_ST_ERR_INV_PARAM;
                }
           }
           else if((PMIC_LP8764X_POWER_RESOURCE_TYPE_LDO == pwrRsrcType) &&
                   (false == pPmicCoreHandle->pPmic_SubSysInfo->ldoEnable))
           {
                status = PMIC_ST_ERR_INV_PARAM;
           }
           else
           {
                status = PMIC_ST_ERR_INV_PARAM;
           }

            break;
        default:
            status = PMIC_ST_ERR_INV_PARAM;
            break;
    }

    return status;
}

/*!
 * \brief   This function is to validate the parameters and power resource
 *           limit for the specific PMIC device.
 */
static int32_t Pmic_powerParamCheck(const Pmic_CoreHandle_t *pPmicCoreHandle,
                                    uint16_t                 pwrResource)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t pwrRsrcType;

    if(NULL == pPmicCoreHandle)
    {
        status = PMIC_ST_ERR_INV_HANDLE;
    }

    pwrRsrcType = Pmic_powerGetPwrRsrcType(pwrResource);

    if(PMIC_ST_SUCCESS == status)
    {
        status = Pmic_powerValidatePwrRsrcLimit(pPmicCoreHandle,
                                                pwrRsrcType,
                                                pwrResource);
    }

    return status;
}

/*!
 * \brief   This function is used to select the bypass or linear regulator
 *           mode for LDO.
 */
static int32_t Pmic_powerSetLdoBypassModeEn(Pmic_CoreHandle_t *pPmicCoreHandle,
                                            uint16_t           pwrRsrc,
                                            bool               ldoBypassModeEn)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    uint8_t pwrRsrcIndex = 0U;
    Pmic_powerRsrcRegCfg_t *pPwrRsrcRegCfg = NULL;

    pwrRsrcIndex = Pmic_powerGetPwrRsrcId(pwrRsrc);

    /* Get PMIC power resources register configuration */
    pmicStatus = Pmic_powerGetRsrcRegCfg(pPmicCoreHandle->pmicDeviceType,
                                         &pPwrRsrcRegCfg);

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        Pmic_criticalSectionStart(pPmicCoreHandle);
        pmicStatus = Pmic_commIntf_recvByte(
                                      pPmicCoreHandle,
                                      pPwrRsrcRegCfg[pwrRsrcIndex].vout1RegAddr,
                                      &regData);
        if(PMIC_ST_SUCCESS == pmicStatus)
        {
            Pmic_setBitField(&regData,
                             PMIC_LDO1_2_3_VOUT_LDO1_2_3_BYPASS_SHIFT,
                             PMIC_LDO1_2_3_VOUT_LDO1_2_3_BYPASS_MASK,
                             ldoBypassModeEn);

            pmicStatus = Pmic_commIntf_sendByte(
                                      pPmicCoreHandle,
                                      pPwrRsrcRegCfg[pwrRsrcIndex].vout1RegAddr,
                                      regData);
        }

        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    return pmicStatus;
}

/*!
 * \brief   Get Enable /Disable status for Slow Ramp for LDO
 */
static int32_t Pmic_powerGetLdoBypassModeEn(Pmic_CoreHandle_t *pPmicCoreHandle,
                                            uint16_t           pwrRsrc,
                                            bool              *pLdoBypassModeEn)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    uint8_t pwrRsrcIndex = 0U;
    Pmic_powerRsrcRegCfg_t *pPwrRsrcRegCfg = NULL;

    pwrRsrcIndex = Pmic_powerGetPwrRsrcId(pwrRsrc);

    /* Get PMIC power resources register configuration */
    pmicStatus = Pmic_powerGetRsrcRegCfg(pPmicCoreHandle->pmicDeviceType,
                                         &pPwrRsrcRegCfg);

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        Pmic_criticalSectionStart(pPmicCoreHandle);
        pmicStatus = Pmic_commIntf_recvByte(
                                      pPmicCoreHandle,
                                      pPwrRsrcRegCfg[pwrRsrcIndex].vout1RegAddr,
                                      &regData);
        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        *pLdoBypassModeEn = Pmic_getBitField(
                                      regData,
                                      PMIC_LDO1_2_3_VOUT_LDO1_2_3_BYPASS_SHIFT,
                                      PMIC_LDO1_2_3_VOUT_LDO1_2_3_BYPASS_MASK);
    }

    return pmicStatus;
}

 /*!
 * \brief   This function is used to validate the ldo residual voltage timeout
 *          limit.
 */
static int32_t Pmic_powerValidateLdoRvTimeoutLimit(uint8_t pmicDeviceType,
                                                   uint8_t ldoRvTimeoutSel)
{
    int32_t status = PMIC_ST_SUCCESS;

    switch (pmicDeviceType)
    {
        case PMIC_DEV_LEO_TPS6594X:
            if(ldoRvTimeoutSel > PMIC_TPS6594X_REGULATOR_LDO_RV_TIMEOUT_16MS)
            {
                status = PMIC_ST_ERR_INV_PARAM;
            }

            break;
        default:
            status = PMIC_ST_ERR_INV_PARAM;
            break;
    }

    return status;
}

/*!
 * \brief   This function is used to set LDO residual voltage check timeout
 *          for LDO
 */
static int32_t Pmic_powerSetLdoRvTimeoutSel(Pmic_CoreHandle_t *pPmicCoreHandle,
                                            uint16_t           pwrRsrc,
                                            uint8_t            ldoRvTimeoutSel)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    uint8_t regAddr = 0U;
    uint8_t pwrRsrcIndex = 0U;
    uint8_t bitMask = 0U;
    Pmic_powerRsrcRegCfg_t *pPwrRsrcRegCfg = NULL;

    pwrRsrcIndex = Pmic_powerGetPwrRsrcId(pwrRsrc);

    /* Get PMIC power resources register configuration */
    pmicStatus = Pmic_powerGetRsrcRegCfg(pPmicCoreHandle->pmicDeviceType,
                                         &pPwrRsrcRegCfg);
    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        regAddr = pPwrRsrcRegCfg[pwrRsrcIndex].ldoRvTimeOutRegAddr;

        pmicStatus = Pmic_powerValidateLdoRvTimeoutLimit(
                                                pPmicCoreHandle->pmicDeviceType,
                                                ldoRvTimeoutSel);
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        bitMask = (uint8_t)(PMIC_REGULATOR_LDO_RV_TIMEOUT_SEL_BITFIELD \
                  << pPwrRsrcRegCfg[pwrRsrcIndex].ldoRvTimeOutBitShiftVal);
        Pmic_criticalSectionStart(pPmicCoreHandle);
        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                            regAddr,
                                            &regData);
        if(PMIC_ST_SUCCESS == pmicStatus)
        {
            Pmic_setBitField(
                           &regData,
                           pPwrRsrcRegCfg[pwrRsrcIndex].ldoRvTimeOutBitShiftVal,
                           bitMask,
                           ldoRvTimeoutSel);
            pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                                regAddr,
                                                regData);
        }

        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    return pmicStatus;
}

/*!
 * \brief   This function is used to get LDO residual voltage check timeout
 *          for LDO
 */
static int32_t Pmic_powerGetLdoRvTimeoutSel(
                                          Pmic_CoreHandle_t *pPmicCoreHandle,
                                          uint16_t           pwrRsrc,
                                          uint8_t           *pLdoResVoltTimeout)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    uint8_t regAddr = 0U;
    uint8_t pwrRsrcIndex = 0U;
    uint8_t bitMask = 0U;
    Pmic_powerRsrcRegCfg_t *pPwrRsrcRegCfg = NULL;

    pwrRsrcIndex = Pmic_powerGetPwrRsrcId(pwrRsrc);

    /* Get PMIC power resources register configuration */
    pmicStatus = Pmic_powerGetRsrcRegCfg(pPmicCoreHandle->pmicDeviceType,
                                         &pPwrRsrcRegCfg);
    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        regAddr = pPwrRsrcRegCfg[pwrRsrcIndex].ldoRvTimeOutRegAddr;
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        Pmic_criticalSectionStart(pPmicCoreHandle);
        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                            regAddr,
                                            &regData);
        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        bitMask = (uint8_t)(PMIC_REGULATOR_LDO_RV_TIMEOUT_SEL_BITFIELD \
                  << pPwrRsrcRegCfg[pwrRsrcIndex].ldoRvTimeOutBitShiftVal);
        *pLdoResVoltTimeout =
                   Pmic_getBitField(
                           regData,
                           pPwrRsrcRegCfg[pwrRsrcIndex].ldoRvTimeOutBitShiftVal,
                           bitMask);
    }

    return pmicStatus;
}

/*!
 * \brief   This function is used to set the Type of voltage monitoring for
 *          PGOOD signal
 */
static int32_t Pmic_powerSetPgoodWindow(Pmic_CoreHandle_t *pPmicCoreHandle,
                                        bool               pgoodWindow)
{
    int32_t status  = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);
    status = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                    PMIC_PGOOD_SEL_4_REGADDR,
                                    &regData);
    if(PMIC_ST_SUCCESS == status)
    {
        Pmic_setBitField(&regData,
                         PMIC_PGOOD_SEL_4_PGOOD_WINDOW_SHIFT,
                         PMIC_PGOOD_SEL_4_PGOOD_WINDOW_MASK,
                         pgoodWindow);
        status = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                        PMIC_PGOOD_SEL_4_REGADDR,
                                        regData);
    }

    Pmic_criticalSectionStop(pPmicCoreHandle);

    return status;
}

/*!
 * \brief   This function is used to get the Type of voltage monitoring for
 *           PGOOD signal
 */
static int32_t Pmic_powerGetVoltageMonitoringPg(
                                             Pmic_CoreHandle_t *pPmicCoreHandle,
                                             bool              *pPgoodWindow)
{
    int32_t status  = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);

    status = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                    PMIC_PGOOD_SEL_4_REGADDR,
                                    &regData);
    Pmic_criticalSectionStop(pPmicCoreHandle);

    if(PMIC_ST_SUCCESS == status)
    {
        *pPgoodWindow = Pmic_getBitField(regData,
                                         PMIC_PGOOD_SEL_4_PGOOD_WINDOW_SHIFT,
                                         PMIC_PGOOD_SEL_4_PGOOD_WINDOW_MASK);
    }

    return status;
}

/*!
 * \brief   This function is used to set the PGOOD signal polarity
 */
static int32_t Pmic_powerSetPgoodPolarity(Pmic_CoreHandle_t *pPmicCoreHandle,
                                          bool               pgoodPolarity)
{
    int32_t status  = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);
    status = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                    PMIC_PGOOD_SEL_4_REGADDR,
                                    &regData);
    if(PMIC_ST_SUCCESS == status)
    {
        Pmic_setBitField(&regData,
                         PMIC_PGOOD_SEL_4_PGOOD_POL_SHIFT,
                         PMIC_PGOOD_SEL_4_PGOOD_POL_MASK,
                         pgoodPolarity);
        status = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                        PMIC_PGOOD_SEL_4_REGADDR,
                                        regData);
    }

    Pmic_criticalSectionStop(pPmicCoreHandle);

    return status;
}

/*!
 * \brief   This function is used to get the PGOOD signal polarity
 */
static int32_t Pmic_powerGetPgoodPolarity(Pmic_CoreHandle_t *pPmicCoreHandle,
                                          bool              *pPgoodPolarity)
{
    int32_t status  = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);
    status = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                    PMIC_PGOOD_SEL_4_REGADDR,
                                    &regData);
    Pmic_criticalSectionStop(pPmicCoreHandle);

    if(PMIC_ST_SUCCESS == status)
    {
        *pPgoodPolarity = Pmic_getBitField(regData,
                                           PMIC_PGOOD_SEL_4_PGOOD_POL_SHIFT,
                                           PMIC_PGOOD_SEL_4_PGOOD_POL_MASK);
    }

    return status;
}

/*!
 * \brief   This function is used to set Deglitch time for BUCKx_VMON/
 *          LDOx_VMON/VCCA_VMON
 */
static int32_t Pmic_powerSetDeglitchTimeSel(Pmic_CoreHandle_t *pPmicCoreHandle,
                                            bool               deglitchTimeSel)
{
    int32_t status  = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);
    status = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                    PMIC_VCCA_VMON_CTRL_REGADDR,
                                    &regData);
    if(PMIC_ST_SUCCESS == status)
    {
        /* Set Deglitch time */
        Pmic_setBitField(&regData,
                         PMIC_VCCA_VMON_CTRL_VMON_DEGLITCH_SEL_SHIFT,
                         PMIC_VCCA_VMON_CTRL_VMON_DEGLITCH_SEL_MASK,
                         deglitchTimeSel);
        status = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                        PMIC_VCCA_VMON_CTRL_REGADDR,
                                        regData);
    }

    Pmic_criticalSectionStop(pPmicCoreHandle);
    return status;
}

/*!
 * \brief   Get Deglitch time for BUCKx_VMON/LDOx_VMON/VCCA_VMON/VMONX
 */
static int32_t Pmic_powerGetDeglitchTimeSel(Pmic_CoreHandle_t *pPmicCoreHandle,
                                            bool              *pDeglitchTimeSel)
{
    int32_t status  = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);
    status = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                    PMIC_VCCA_VMON_CTRL_REGADDR,
                                    &regData);
    Pmic_criticalSectionStop(pPmicCoreHandle);

    if(PMIC_ST_SUCCESS == status)
    {
        /* Get Deglitch time */
        *pDeglitchTimeSel = Pmic_getBitField(
                                    regData,
                                    PMIC_VCCA_VMON_CTRL_VMON_DEGLITCH_SEL_SHIFT,
                                    PMIC_VCCA_VMON_CTRL_VMON_DEGLITCH_SEL_MASK);
    }

    return status;
}

/*!
 * \brief   Get Trigger selection for Severe Error
 */
static int32_t Pmic_powerGetSevereErrorTrig(Pmic_CoreHandle_t *pPmicCoreHandle,
                                            uint8_t           *pSevereErrorTrig)
{
    int32_t status  = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);
    status = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                    PMIC_FSM_TRIG_SEL_1_REGADDR,
                                    &regData);
    Pmic_criticalSectionStop(pPmicCoreHandle);

    if(PMIC_ST_SUCCESS == status)
    {
        *pSevereErrorTrig = Pmic_getBitField(
                                      regData,
                                      PMIC_FSM_TRIG_SEL_1_SEVERE_ERR_TRIG_SHIFT,
                                      PMIC_FSM_TRIG_SEL_1_SEVERE_ERR_TRIG_MASK);
    }

    return status;
}

/*!
 * \brief   Get Trigger selection for OTHER rail group
 */
static int32_t Pmic_powerGetOtherRailTrig(Pmic_CoreHandle_t *pPmicCoreHandle,
                                          uint8_t           *pOtherRailTrig)
{
    int32_t status  = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);
    status = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                    PMIC_FSM_TRIG_SEL_1_REGADDR,
                                    &regData);
    Pmic_criticalSectionStop(pPmicCoreHandle);

    if(PMIC_ST_SUCCESS == status)
    {
        *pOtherRailTrig =
                     Pmic_getBitField(regData,
                                      PMIC_FSM_TRIG_SEL_1_OTHER_RAIL_TRIG_SHIFT,
                                      PMIC_FSM_TRIG_SEL_1_OTHER_RAIL_TRIG_MASK);
    }

    return status;
}

/*!
 * \brief   Get Trigger selection for SOC rail group
 */
static int32_t Pmic_powerGetSocRailTrig(Pmic_CoreHandle_t *pPmicCoreHandle,
                                        uint8_t           *pSocRailTrig)
{
    int32_t status  = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);
    status = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                    PMIC_FSM_TRIG_SEL_1_REGADDR,
                                    &regData);
    Pmic_criticalSectionStop(pPmicCoreHandle);

    if(PMIC_ST_SUCCESS == status)
    {
        *pSocRailTrig =
                  Pmic_getBitField(regData,
                                   PMIC_FSM_TRIG_SEL_1_SOC_RAIL_TRIG_SHIFT,
                                   PMIC_FSM_TRIG_SEL_1_SOC_RAIL_TRIG_MASK);
    }

    return status;
}

/*!
 * \brief   Get Trigger selection for MCU rail group
 */
static int32_t Pmic_powerGetMcuRailTrig(Pmic_CoreHandle_t *pPmicCoreHandle,
                                        uint8_t           *pMcuRailTrig)
{
    int32_t status  = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);
    status = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                    PMIC_FSM_TRIG_SEL_1_REGADDR,
                                    &regData);
    Pmic_criticalSectionStop(pPmicCoreHandle);

    if(PMIC_ST_SUCCESS == status)
    {
        *pMcuRailTrig =
                      Pmic_getBitField(regData,
                                       PMIC_FSM_TRIG_SEL_1_MCU_RAIL_TRIG_SHIFT,
                                       PMIC_FSM_TRIG_SEL_1_MCU_RAIL_TRIG_MASK);
    }

    return status;
}

/*!
 * \brief   Get Trigger selection for Moderate error
 */
static int32_t Pmic_powerGetModerateRailTrig(
                                           Pmic_CoreHandle_t *pPmicCoreHandle,
                                           uint8_t           *pModerateRailTrig)
{
    int32_t status  = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);
    status = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                    PMIC_FSM_TRIG_SEL_2_REGADDR,
                                    &regData);
    Pmic_criticalSectionStop(pPmicCoreHandle);

    if(PMIC_ST_SUCCESS == status)
    {
        *pModerateRailTrig = Pmic_getBitField(
                                    regData,
                                    PMIC_FSM_TRIG_SEL_2_MODERATE_ERR_TRIG_SHIFT,
                                    PMIC_FSM_TRIG_SEL_2_MODERATE_ERR_TRIG_MASK);
    }

    return status;
}

/*!
 * \brief   This function is used to validate the FSM trigger
 */
static int32_t Pmic_validateFsmTrig(uint8_t socRailTrig)
{
    int32_t status = PMIC_ST_SUCCESS;

    if(socRailTrig > PMIC_POWER_TRIG_SOC_PWR_ERR)
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    return status;
}

/*!
 * \brief   This function is used to Set the trigger selection for severe Error
 */
static int32_t Pmic_powerSetSevereErrorTrig(Pmic_CoreHandle_t *pPmicCoreHandle,
                                            uint8_t            severeErrorTrig)
{
    int32_t status  = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    status = Pmic_validateFsmTrig(severeErrorTrig);

    if(PMIC_ST_SUCCESS == status)
    {
        Pmic_criticalSectionStart(pPmicCoreHandle);
        status = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        PMIC_FSM_TRIG_SEL_1_REGADDR,
                                        &regData);
        if(PMIC_ST_SUCCESS == status)
        {
            Pmic_setBitField(&regData,
                             PMIC_FSM_TRIG_SEL_1_SEVERE_ERR_TRIG_SHIFT,
                             PMIC_FSM_TRIG_SEL_1_SEVERE_ERR_TRIG_MASK,
                             severeErrorTrig);
            status = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                            PMIC_FSM_TRIG_SEL_1_REGADDR,
                                            regData);
        }

        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    return status;
}

/*!
 * \brief   This function is used to Set trigger selection fro other rail group
 */
static int32_t Pmic_powerSetOtherRailTrig(Pmic_CoreHandle_t *pPmicCoreHandle,
                                          uint8_t            otherRailTrig)
{
    int32_t status  = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    status = Pmic_validateFsmTrig(otherRailTrig);

    if(PMIC_ST_SUCCESS == status)
    {
        Pmic_criticalSectionStart(pPmicCoreHandle);
        status = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        PMIC_FSM_TRIG_SEL_1_REGADDR,
                                        &regData);
        if(PMIC_ST_SUCCESS == status)
        {
            Pmic_setBitField(&regData,
                             PMIC_FSM_TRIG_SEL_1_OTHER_RAIL_TRIG_SHIFT,
                             PMIC_FSM_TRIG_SEL_1_OTHER_RAIL_TRIG_MASK,
                             otherRailTrig);
            status = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                            PMIC_FSM_TRIG_SEL_1_REGADDR,
                                            regData);
        }

        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    return status;
}

/*!
 * \brief   This function is used to Set the trigger selection for soc rail
 *          group
 */
static int32_t Pmic_powerSetSocRailTrig(Pmic_CoreHandle_t *pPmicCoreHandle,
                                        uint8_t            socRailTrig)
{
    int32_t status  = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    status = Pmic_validateFsmTrig(socRailTrig);

    if(PMIC_ST_SUCCESS == status)
    {
        Pmic_criticalSectionStart(pPmicCoreHandle);
        status = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        PMIC_FSM_TRIG_SEL_1_REGADDR,
                                        &regData);
        if(PMIC_ST_SUCCESS == status)
        {
            Pmic_setBitField(&regData,
                             PMIC_FSM_TRIG_SEL_1_SOC_RAIL_TRIG_SHIFT,
                             PMIC_FSM_TRIG_SEL_1_SOC_RAIL_TRIG_MASK,
                             socRailTrig);
            status = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                            PMIC_FSM_TRIG_SEL_1_REGADDR,
                                            regData);
        }

        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    return status;
}

/*!
 * \brief   This function is used to Set the trigger selection for mcu rail
 *          group
 */
static int32_t Pmic_powerSetMcuRailTrig(Pmic_CoreHandle_t *pPmicCoreHandle,
                                        uint8_t            mcuRailTrig)
{
    int32_t status  = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    status = Pmic_validateFsmTrig(mcuRailTrig);

    if(PMIC_ST_SUCCESS == status)
    {
        Pmic_criticalSectionStart(pPmicCoreHandle);
        status = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        PMIC_FSM_TRIG_SEL_1_REGADDR,
                                        &regData);
        if(PMIC_ST_SUCCESS == status)
        {
            Pmic_setBitField(&regData,
                             PMIC_FSM_TRIG_SEL_1_MCU_RAIL_TRIG_SHIFT,
                             PMIC_FSM_TRIG_SEL_1_MCU_RAIL_TRIG_MASK,
                             mcuRailTrig);
            status = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                            PMIC_FSM_TRIG_SEL_1_REGADDR,
                                            regData);
        }

        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    return status;
}

/*!
 * \brief   This function is used to Set the trigger selection for Moderate
 *          Error
 */
static int32_t Pmic_powerSetModerateRailTrig(
                                            Pmic_CoreHandle_t *pPmicCoreHandle,
                                            uint8_t            moderateRailTrig)
{
    int32_t status  = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    status = Pmic_validateFsmTrig(moderateRailTrig);

    if(PMIC_ST_SUCCESS == status)
    {
        Pmic_criticalSectionStart(pPmicCoreHandle);
        status = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        PMIC_FSM_TRIG_SEL_2_REGADDR,
                                        &regData);
        if(PMIC_ST_SUCCESS == status)
        {
            Pmic_setBitField(&regData,
                             PMIC_FSM_TRIG_SEL_2_MODERATE_ERR_TRIG_SHIFT,
                             PMIC_FSM_TRIG_SEL_2_MODERATE_ERR_TRIG_MASK,
                             moderateRailTrig);
            status = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                            PMIC_FSM_TRIG_SEL_2_REGADDR,
                                            regData);
        }

        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    return status;
}

/*!
 * \brief   This function is to validate the power good source limit for the
 *          specific PMIC device.
 */
static int32_t Pmic_validatepGoodSrcType(uint8_t pmicDeviceType,
                                         uint16_t pgoodSrc)
{
    int32_t status = PMIC_ST_SUCCESS;

    switch(pmicDeviceType)
    {
        case PMIC_DEV_LEO_TPS6594X:
            status = Pmic_validate_tps6594x_pGoodSrcType(pgoodSrc);
            break;
        case PMIC_DEV_HERA_LP8764X:
            status = Pmic_validate_lp8764x_pGoodSrcType(pgoodSrc);
            break;
        default:
            status = PMIC_ST_ERR_INV_PARAM;
            break;
    }

    return status;
}

/*!
 * \brief   This function is to validate the power good source limit for the
 *          specific PMIC device and the core handle.
 */
static int32_t Pmic_validatePowerGoodParam(Pmic_CoreHandle_t   *pPmicCoreHandle,
                                           uint16_t             pgoodSrcSel)
{
    int32_t status = PMIC_ST_SUCCESS;

    if(NULL == pPmicCoreHandle)
    {
        status = PMIC_ST_ERR_INV_HANDLE;
    }

    if(status == PMIC_ST_SUCCESS)
    {
        status = Pmic_validatepGoodSrcType(pPmicCoreHandle->pmicDeviceType,
                                           pgoodSrcSel);
    }

    return status;
}
/*!
 * \brief   This function is to validate the power good signal source selection
 *          limit for the specific PMIC device.
 */
static int32_t Pmic_powerValidatePgoodSelType(uint8_t  pmicDeviceType,
                                              uint16_t pgoodSrcSel,
                                              uint8_t  pgoodSelType)
{
    int32_t status = PMIC_ST_SUCCESS;

    switch(pmicDeviceType)
    {
        case PMIC_DEV_LEO_TPS6594X:
            status = Pmic_validate_tps6594x_pGoodSelType(pgoodSrcSel,
                                                         pgoodSelType);
            break;
        case PMIC_DEV_HERA_LP8764X:
            status = Pmic_validate_lp8764x_pGoodSelType(pgoodSrcSel,
                                                        pgoodSelType);
            break;
        default:
            status = PMIC_ST_ERR_INV_PARAM;
            break;
    }

    return status;
}

/*!
 * \brief   This function is used to get the PMIC power-good sources
            register configuration for the specific PMIC device.
 */
static int32_t Pmic_powerGetPgoodSrcRegCfg(
                                 uint8_t                      pmicDeviceType,
                                 Pmic_powerPgoodSrcRegCfg_t **pPwrGoodSrcRegCfg)
{
    int32_t status = PMIC_ST_SUCCESS;

    if(NULL == pPwrGoodSrcRegCfg)
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if(PMIC_ST_SUCCESS == status)
    {
        switch(pmicDeviceType)
        {
            case PMIC_DEV_LEO_TPS6594X:
                pmic_get_tps6594x_pwrPgoodSrcRegCfg(pPwrGoodSrcRegCfg);
                break;
            case PMIC_DEV_HERA_LP8764X:
                pmic_get_lp8764x_pwrPgoodSrcRegCfg(pPwrGoodSrcRegCfg);
                break;
            default:
                status = PMIC_ST_ERR_INV_DEVICE;
                break;
        }
    }

    return status;
}

/*!
 * \brief   This function is used to get the Bitmask value for source signal
 */
static int32_t Pmic_getPgoodSelTypeBitmask(uint8_t   pmicDeviceType,
                                           uint16_t  pgoodSrcSel,
                                           uint8_t  *pBitMask)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t  pGoodSrcType;

    pGoodSrcType = Pmic_powerGetPwrRsrcType(pgoodSrcSel);
    switch(pmicDeviceType)
    {
        case PMIC_DEV_LEO_TPS6594X:
            if((PMIC_TPS6594X_PGOOD_SOURCE_TYPE_BUCK == pGoodSrcType) ||
               (PMIC_TPS6594X_PGOOD_SOURCE_TYPE_LDO  == pGoodSrcType))
            {
                *pBitMask = PMIC_PGOOD_SEL_PGOOD_SRC_REGULATOR_BITFIELD;
            }
            else if((PMIC_TPS6594X_PGOOD_SOURCE_TYPE_NRSTOUT == pGoodSrcType) ||
                    (PMIC_TPS6594X_PGOOD_SOURCE_TYPE_NRSTOUT_SOC ==
                                                                pGoodSrcType) ||
                    (PMIC_TPS6594X_PGOOD_SOURCE_TYPE_TDIE == pGoodSrcType)    ||
                    (PMIC_TPS6594X_PGOOD_SOURCE_TYPE_VCCA == pGoodSrcType))
            {
                *pBitMask = PMIC_PGOOD_SEL_PGOOD_SRC_BITFIELD;
            }
            else
            {
                status = PMIC_ST_ERR_INV_PARAM;
            }

            break;
        case PMIC_DEV_HERA_LP8764X:
            if(PMIC_LP8764X_PGOOD_SOURCE_TYPE_BUCK == pGoodSrcType)
            {
                *pBitMask = PMIC_PGOOD_SEL_PGOOD_SRC_REGULATOR_BITFIELD;
            }
            else if((PMIC_LP8764X_PGOOD_SOURCE_TYPE_NRSTOUT == pGoodSrcType)  ||
                    (PMIC_LP8764X_PGOOD_SOURCE_TYPE_NRSTOUT_SOC ==
                                                                pGoodSrcType) ||
                    (PMIC_LP8764X_PGOOD_SOURCE_TYPE_TDIE == pGoodSrcType)     ||
                    (PMIC_LP8764X_PGOOD_SOURCE_TYPE_VMON == pGoodSrcType)     ||
                    (PMIC_TPS6594X_PGOOD_SOURCE_TYPE_VCCA == pGoodSrcType))
            {
                *pBitMask = PMIC_PGOOD_SEL_PGOOD_SRC_BITFIELD;
            }
            else
            {
                status = PMIC_ST_ERR_INV_PARAM;
            }

            break;
        default:
            status= PMIC_ST_ERR_INV_PARAM;
            break;
    }

    return status;
}

/*!
 * \brief   This function is to set the source signal for power good signal
 */
static int32_t Pmic_setPgoodSelType(Pmic_CoreHandle_t *pPmicCoreHandle,
                                    uint16_t           pgoodSrcSel,
                                    uint8_t            pgoodSelType)
{
    int32_t status     = PMIC_ST_SUCCESS;
    uint8_t pgoodSrcId = 0U;
    uint8_t regAddr    = 0U;
    uint8_t bitMask    = 0U;
    uint8_t shiftVal   = 0U;
    uint8_t regData    = 0U;
    Pmic_powerPgoodSrcRegCfg_t *pPGoodSrcRegCfg = NULL;

    /* Get PMIC power-good sources register configuration */
    status = Pmic_powerGetPgoodSrcRegCfg(pPmicCoreHandle->pmicDeviceType,
                                         &pPGoodSrcRegCfg);

    if(PMIC_ST_SUCCESS == status)
    {
        pgoodSrcId = Pmic_powerGetPwrRsrcId(pgoodSrcSel);
    }

    if(PMIC_ST_SUCCESS == status)
    {
        status = Pmic_getPgoodSelTypeBitmask(pPmicCoreHandle->pmicDeviceType,
                                             pgoodSrcSel,
                                             &bitMask);
    }

    if(PMIC_ST_SUCCESS == status)
    {
        regAddr  = pPGoodSrcRegCfg[pgoodSrcId].regAddr;
        shiftVal = pPGoodSrcRegCfg[pgoodSrcId].shiftValue;
        bitMask  = bitMask << shiftVal;

        Pmic_criticalSectionStart(pPmicCoreHandle);
        status = Pmic_commIntf_recvByte(pPmicCoreHandle, regAddr, &regData);

        if(PMIC_ST_SUCCESS == status)
        {
            Pmic_setBitField(&regData, shiftVal, bitMask, pgoodSelType);
            status = Pmic_commIntf_sendByte(pPmicCoreHandle, regAddr, regData);
        }

        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    return status;
}

/*!
 * \brief   This function is to get the source signal for power good signal
 */
static int32_t Pmic_getPgoodSelType(Pmic_CoreHandle_t *pPmicCoreHandle,
                                    uint16_t           pgoodSrcSel,
                                    uint8_t           *pPgoodSelType)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t pgoodSrcId;
    uint8_t regAddr;
    uint8_t bitMask;
    uint8_t shiftVal;
    uint8_t regData = 0U;

    Pmic_powerPgoodSrcRegCfg_t *pPGoodSrcRegCfg = NULL;

    /* Get PMIC power-good sources register configuration */
    status = Pmic_powerGetPgoodSrcRegCfg(pPmicCoreHandle->pmicDeviceType,
                                         &pPGoodSrcRegCfg);

    if(PMIC_ST_SUCCESS == status)
    {
        pgoodSrcId = Pmic_powerGetPwrRsrcId(pgoodSrcSel);
    }

    if(PMIC_ST_SUCCESS == status)
    {
        status = Pmic_getPgoodSelTypeBitmask(pPmicCoreHandle->pmicDeviceType,
                                             pgoodSrcSel,
                                             &bitMask);
    }

    if(PMIC_ST_SUCCESS == status)
    {
        regAddr  = pPGoodSrcRegCfg[pgoodSrcId].regAddr;
        shiftVal = pPGoodSrcRegCfg[pgoodSrcId].shiftValue;
        bitMask  = bitMask << shiftVal;

        Pmic_criticalSectionStart(pPmicCoreHandle);
        status = Pmic_commIntf_recvByte(pPmicCoreHandle, regAddr, &regData);
        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    if(PMIC_ST_SUCCESS == status)
    {
        *pPgoodSelType = Pmic_getBitField(regData, shiftVal, bitMask);
    }

    return status;
}

/*!
 * \brief   This function is used to get the thermal warning status.
 */
static int32_t Pmic_getThermalWarnStat(Pmic_CoreHandle_t  *pPmicCoreHandle,
                                       bool               *pStateWarning)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData;

    /* Reading the Thermal Status */
    if(PMIC_ST_SUCCESS == status)
    {
        Pmic_criticalSectionStart(pPmicCoreHandle);
        status = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        PMIC_STAT_MISC_REGADDR,
                                        &regData);
        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    if(PMIC_ST_SUCCESS == status)
    {
        *pStateWarning = Pmic_getBitField(regData,
                                          PMIC_STAT_MISC_TWARN_STAT_SHIFT,
                                          PMIC_STAT_MISC_TWARN_STAT_MASK);
    }

    return status;
}

/*!
 * \brief   This function is used to get the oderly shutdown temperature status.
 */
static int32_t Pmic_getOderlyShutdownStat(Pmic_CoreHandle_t  *pPmicCoreHandle,
                                          bool               *pStatOderlyShtDwn)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData;

    /* Reading the Thermal Status */
    if(PMIC_ST_SUCCESS == status)
    {
        Pmic_criticalSectionStart(pPmicCoreHandle);
        status = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        PMIC_STAT_MODERATE_ERR_REGADDR,
                                        &regData);
        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    if(PMIC_ST_SUCCESS == status)
    {
        *pStatOderlyShtDwn = Pmic_getBitField(
                                      regData,
                                      PMIC_STAT_MODERATE_ERR_TSD_ORD_STAT_SHIFT,
                                      PMIC_STAT_MODERATE_ERR_TSD_ORD_STAT_MASK);
    }

    return status;
}

/*!
 * \brief   This function is used to get the immediate shutdown status.
 */
static int32_t Pmic_getImmediateShutdownStat(
                                            Pmic_CoreHandle_t  *pPmicCoreHandle,
                                            bool               *pStatImmShtDwn)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData;

    /* Reading the Thermal Status */
    if(PMIC_ST_SUCCESS == status)
    {
        Pmic_criticalSectionStart(pPmicCoreHandle);
        status = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        PMIC_STAT_SEVERE_ERR_REGADDR,
                                        &regData);
        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    if(PMIC_ST_SUCCESS == status)
    {
        *pStatImmShtDwn =
                      Pmic_getBitField(regData,
                                       PMIC_STAT_SEVERE_ERR_TSD_IMM_STAT_SHIFT,
                                       PMIC_STAT_SEVERE_ERR_TSD_IMM_STAT_MASK);
    }

    return status;
}

/*!
 * \brief   This function is used to get the status for current limit for
 *          BUCK/LDO
 */
static int32_t Pmic_powerGetCurrentLimitLvlStat(
                                       Pmic_CoreHandle_t  *pPmicCoreHandle,
                                       uint16_t            pwrRsrc,
                                       bool               *pCurrentLimitLvlStat)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    uint8_t pwrRsrcIndex = 0U;
    uint8_t bitMask = 0U;
    Pmic_powerRsrcRegCfg_t *pPwrRsrcRegCfg = NULL;

    pwrRsrcIndex = Pmic_powerGetPwrRsrcId(pwrRsrc);

    /* Get PMIC power resources register configuration */
    pmicStatus = Pmic_powerGetRsrcRegCfg(pPmicCoreHandle->pmicDeviceType,
                                         &pPwrRsrcRegCfg);
    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        Pmic_criticalSectionStart(pPmicCoreHandle);
        pmicStatus = Pmic_commIntf_recvByte(
                                     pPmicCoreHandle,
                                     pPwrRsrcRegCfg[pwrRsrcIndex].statusRegAddr,
                                     &regData);
        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        bitMask = (uint8_t)(PMIC_POWER_RESOURCE_STATUS_BITFIELD \
                  << pPwrRsrcRegCfg[pwrRsrcIndex].iLimStatBitShift);

        *pCurrentLimitLvlStat = Pmic_getBitField(
                                  regData,
                                  pPwrRsrcRegCfg[pwrRsrcIndex].iLimStatBitShift,
                                  bitMask);

    }

    return pmicStatus;
}

/*!
 * \brief   This function is used to get the status for under voltage threshold
 */
static int32_t Pmic_powerGetUnderVoltageTholdStat(
                                             Pmic_CoreHandle_t *pPmicCoreHandle,
                                             uint16_t           pwrRsrc,
                                             bool              *pUvTholdStat)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    uint8_t pwrRsrcIndex = 0U;
    uint8_t bitMask = 0U;
    Pmic_powerRsrcRegCfg_t *pPwrRsrcRegCfg = NULL;

    pwrRsrcIndex = Pmic_powerGetPwrRsrcId(pwrRsrc);

    /* Get PMIC power resources register configuration */
    pmicStatus = Pmic_powerGetRsrcRegCfg(pPmicCoreHandle->pmicDeviceType,
                                         &pPwrRsrcRegCfg);
    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        Pmic_criticalSectionStart(pPmicCoreHandle);
        pmicStatus = Pmic_commIntf_recvByte(
                                     pPmicCoreHandle,
                                     pPwrRsrcRegCfg[pwrRsrcIndex].statusRegAddr,
                                     &regData);
        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        bitMask = (uint8_t)(PMIC_POWER_RESOURCE_STATUS_BITFIELD \
                  << pPwrRsrcRegCfg[pwrRsrcIndex].uvStatBitShift);

        *pUvTholdStat = Pmic_getBitField(
                                    regData,
                                    pPwrRsrcRegCfg[pwrRsrcIndex].uvStatBitShift,
                                    bitMask);

    }

    return pmicStatus;
}

/*!
 * \brief   This function is used to get the status for Over voltage threshold
 */
static int32_t Pmic_powerGetOverVoltageTholdStat(
                                             Pmic_CoreHandle_t *pPmicCoreHandle,
                                             uint16_t           pwrRsrc,
                                             bool              *pOvTholdStat)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    uint8_t pwrRsrcIndex = 0U;
    uint8_t bitMask = 0U;
    Pmic_powerRsrcRegCfg_t *pPwrRsrcRegCfg = NULL;

    pwrRsrcIndex = Pmic_powerGetPwrRsrcId(pwrRsrc);

    /* Get PMIC power resources register configuration */
    pmicStatus = Pmic_powerGetRsrcRegCfg(pPmicCoreHandle->pmicDeviceType,
                                         &pPwrRsrcRegCfg);
    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        Pmic_criticalSectionStart(pPmicCoreHandle);
        pmicStatus = Pmic_commIntf_recvByte(
                                     pPmicCoreHandle,
                                     pPwrRsrcRegCfg[pwrRsrcIndex].statusRegAddr,
                                     &regData);
        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        bitMask = (uint8_t)(PMIC_POWER_RESOURCE_STATUS_BITFIELD \
                  << pPwrRsrcRegCfg[pwrRsrcIndex].ovStatBitShift);

        *pOvTholdStat = Pmic_getBitField(
                                    regData,
                                    pPwrRsrcRegCfg[pwrRsrcIndex].ovStatBitShift,
                                    bitMask);
    }

    return pmicStatus;
}

/*!
 * \brief   This function is used to get the status for Over voltage Level for
 *          VCCA
 */
static int32_t Pmic_powerGetOverVoltageProtectionLvlStat(
                                        Pmic_CoreHandle_t *pPmicCoreHandle,
                                        uint16_t           pwrRsrc,
                                        bool              *pOvProtectionLvlStat)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    uint8_t pwrRsrcIndex = 0U;
    uint8_t bitMask = 0U;
    Pmic_powerRsrcRegCfg_t *pPwrRsrcRegCfg = NULL;

    pwrRsrcIndex = Pmic_powerGetPwrRsrcId(pwrRsrc);

    /* Get PMIC power resources register configuration */
    pmicStatus = Pmic_powerGetRsrcRegCfg(pPmicCoreHandle->pmicDeviceType,
                                         &pPwrRsrcRegCfg);
    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        Pmic_criticalSectionStart(pPmicCoreHandle);
        pmicStatus = Pmic_commIntf_recvByte(
                                     pPmicCoreHandle,
                                     pPwrRsrcRegCfg[pwrRsrcIndex].statusRegAddr,
                                     &regData);
        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        bitMask = (uint8_t)(PMIC_POWER_RESOURCE_STATUS_BITFIELD \
                  << PMIC_STAT_SEVERE_ERR_VCCA_OVP_STAT_SHIFT);

        *pOvProtectionLvlStat = Pmic_getBitField(
                                       regData,
                                       PMIC_STAT_SEVERE_ERR_VCCA_OVP_STAT_SHIFT,
                                       bitMask);
    }

    return pmicStatus;
}

/*!
 * \brief   This function is used to write the Thermal Threshold value.
 */
static int32_t Pmic_writeThermalThreshold(Pmic_CoreHandle_t *pPmicCoreHandle,
                                          uint8_t            thermalTholdVal)
{
    int32_t status = PMIC_ST_SUCCESS;

    Pmic_criticalSectionStart(pPmicCoreHandle);
    status = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                    PMIC_CONFIG_1_REGADDR,
                                    thermalTholdVal);
    Pmic_criticalSectionStop(pPmicCoreHandle);

    return status;
}

/*!
 * \brief   This function is used to read the Thermal Threshold value.
 */
static int32_t Pmic_readThermalThreshold(Pmic_CoreHandle_t *pPmicCoreHandle,
                                         uint8_t           *pThermalTholdVal)
{
    int32_t status = PMIC_ST_SUCCESS;

    Pmic_criticalSectionStart(pPmicCoreHandle);

    /* Reading the thermal threshold register */
    status = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                    PMIC_CONFIG_1_REGADDR,
                                    pThermalTholdVal);

    Pmic_criticalSectionStop(pPmicCoreHandle);

    return status;
}

/*!
 * \brief   This function is used to set the thermal warning threshold
            temperature value
 */
static int32_t Pmic_setThermalWarnThold(Pmic_CoreHandle_t *pPmicCoreHandle,
                                        bool               thermalWarnThold)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t  thresholdRegvalue;

    status = Pmic_readThermalThreshold(pPmicCoreHandle, &thresholdRegvalue);
    if(PMIC_ST_SUCCESS == status)
    {
       Pmic_setBitField(&thresholdRegvalue,
                        PMIC_CONFIG_1_TWARN_LEVEL_SHIFT,
                        PMIC_CONFIG_1_TWARN_LEVEL_MASK,
                        thermalWarnThold);
    }

    if(PMIC_ST_SUCCESS == status)
    {
        status = Pmic_writeThermalThreshold(pPmicCoreHandle, thresholdRegvalue);
    }

    return status;
}

/*!
 * \brief   This function is used to set the thermal shutdown threshold
            temperature value
 */
static int32_t Pmic_setThermalShutdownThold(
                                        Pmic_CoreHandle_t *pPmicCoreHandle,
                                        bool               thermalShutdownThold)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t  thresholdRegvalue;

    switch(pPmicCoreHandle->pmicDeviceType)
    {
        case PMIC_DEV_LEO_TPS6594X:
            status = Pmic_readThermalThreshold(pPmicCoreHandle,
                                               &thresholdRegvalue);
            if(PMIC_ST_SUCCESS == status)
            {
                 Pmic_setBitField(&thresholdRegvalue,
                                  PMIC_CONFIG_1_TSD_ORD_LEVEL_SHIFT,
                                  PMIC_CONFIG_1_TSD_ORD_LEVEL_MASK,
                                  thermalShutdownThold);
            }

            if(PMIC_ST_SUCCESS == status)
            {
                status = Pmic_writeThermalThreshold(pPmicCoreHandle,
                                                    thresholdRegvalue);
            }

            break;
        case PMIC_DEV_HERA_LP8764X:
            status = PMIC_ST_ERR_INV_PARAM;
            break;
        default:
            status = PMIC_ST_ERR_INV_DEVICE;
            break;
    }

    return status;
}

/*!
 * \brief   This function is used to get the thermal warning threshold
            temperature value
 */
static int32_t Pmic_getThermalWarnThold(Pmic_CoreHandle_t *pPmicCoreHandle,
                                        bool              *pThermalWarnThold)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t  thresholdRegvalue;

    status = Pmic_readThermalThreshold(pPmicCoreHandle, &thresholdRegvalue);
    if(PMIC_ST_SUCCESS == status)
    {
         *pThermalWarnThold = Pmic_getBitField(thresholdRegvalue,
                                               PMIC_CONFIG_1_TWARN_LEVEL_SHIFT,
                                               PMIC_CONFIG_1_TWARN_LEVEL_MASK);
    }

    return status;
}

/*!
 * \brief   This function is used to get the thermal shutdown threshold
            temperature value
 */
static int32_t Pmic_getThermalShutdownThold(
                                       Pmic_CoreHandle_t *pPmicCoreHandle,
                                       bool              *pThermalShutdownThold)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t thresholdRegvalue;

    switch(pPmicCoreHandle->pmicDeviceType)
    {
        case PMIC_DEV_LEO_TPS6594X:
            status = Pmic_readThermalThreshold(pPmicCoreHandle,
                                               &thresholdRegvalue);
            if(PMIC_ST_SUCCESS == status)
            {
                *pThermalShutdownThold = Pmic_getBitField(
                                              thresholdRegvalue,
                                              PMIC_CONFIG_1_TSD_ORD_LEVEL_SHIFT,
                                              PMIC_CONFIG_1_TSD_ORD_LEVEL_MASK);
            }

            break;
        case PMIC_DEV_HERA_LP8764X:
            status = PMIC_ST_ERR_INV_PARAM;
            break;
        default:
            status = PMIC_ST_ERR_INV_DEVICE;
            break;
    }

    return status;
}

/*!
 * \brief   This function is to validate the power resource interrupt type
 *          for the specific PMIC device.
 */
static int32_t Pmic_powerValidateIntrType(uint8_t  pmicDeviceType,
                                          uint16_t pwrResource,
                                          uint8_t  intrType)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t  pwrResourceType;

    pwrResourceType = Pmic_powerGetPwrRsrcType(pwrResource);

    switch(pmicDeviceType)
    {
        case PMIC_DEV_LEO_TPS6594X:
            if((PMIC_TPS6594X_POWER_RESOURCE_TYPE_BUCK == pwrResourceType) ||
               (PMIC_TPS6594X_POWER_RESOURCE_TYPE_LDO == pwrResourceType))
            {
                if((intrType != PMIC_TPS6594X_POWER_OV_INT) &&
                   (intrType != PMIC_TPS6594X_POWER_UV_INT) &&
                   (intrType != PMIC_TPS6594X_POWER_ILIM_INT))
                {
                    status = PMIC_ST_ERR_INV_PARAM;
                }
            }
            else if(PMIC_TPS6594X_POWER_RESOURCE_TYPE_VCCA == pwrResourceType)
            {
                if((intrType != PMIC_TPS6594X_POWER_OV_INT) &&
                   (intrType != PMIC_TPS6594X_POWER_UV_INT))
                {
                    status = PMIC_ST_ERR_INV_PARAM;
                }
            }
            else
            {
                status = PMIC_ST_ERR_INV_PARAM;
            }

            break;
        case PMIC_DEV_HERA_LP8764X:
            if((PMIC_LP8764X_POWER_RESOURCE_TYPE_VCCA == pwrResourceType) ||
               (PMIC_LP8764X_POWER_RESOURCE_TYPE_VMON == pwrResourceType))
            {
                if((intrType != PMIC_LP8764X_POWER_OV_INT) &&
                   (intrType != PMIC_LP8764X_POWER_UV_INT))
                {
                    status = PMIC_ST_ERR_INV_PARAM;
                }
            }

            else if(PMIC_LP8764X_POWER_RESOURCE_TYPE_BUCK == pwrResourceType)
            {
                if((intrType != PMIC_LP8764X_POWER_OV_INT) &&
                   (intrType != PMIC_LP8764X_POWER_UV_INT) &&
                   (intrType != PMIC_LP8764X_POWER_ILIM_INT))
                {
                    status = PMIC_ST_ERR_INV_PARAM;
                }
            }
            else
            {
                status = PMIC_ST_ERR_INV_PARAM;
            }

            break;
        default:
            status = PMIC_ST_ERR_INV_PARAM;
            break;
    }

    return status;
}

/*!
 * \brief   This function is to validate the  Power interrupt
 *          range for the specific PMIC device.
 */
static int32_t Pmic_validateIntr(Pmic_CoreHandle_t *pPmicCoreHandle,
                                 const uint8_t      intrType)
{
    int32_t status = PMIC_ST_SUCCESS;

    if(NULL == pPmicCoreHandle)
    {
        status = PMIC_ST_ERR_INV_HANDLE;
    }

    if(PMIC_ST_SUCCESS == status)
    {
        switch(pPmicCoreHandle->pmicDeviceType)
        {
            case PMIC_DEV_HERA_LP8764X:
                if(intrType > PMIC_LP8764X_POWER_COMMON_INTERRUPT_MAX)
                {
                    status = PMIC_ST_ERR_INV_PARAM;
                }

                break;
            case PMIC_DEV_LEO_TPS6594X:
                if(intrType > PMIC_TPS6594X_POWER_COMMON_INTERRUPT_MAX)
                {
                    status = PMIC_ST_ERR_INV_PARAM;
                }

                break;
            default:
                status = PMIC_ST_ERR_INV_PARAM;
                break;
        }
    }

    return status;
}

/*!
 * \brief   This function is used to Enable/Disable Power resource Interrupt
 */
static int32_t Pmic_powerRsrcIntrEnable(Pmic_CoreHandle_t *pPmicCoreHandle,
                                        const uint16_t     pwrResource,
                                        bool               intrEnable,
                                        const uint8_t      intrType)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t irqNum;
    uint8_t pwrRsrcIndex;
    Pmic_powerRsrcRegCfg_t *pPwrRsrcRegCfg = NULL;

    pwrRsrcIndex = Pmic_powerGetPwrRsrcId(pwrResource);

    /* Get PMIC power resources register configuration */
    status = Pmic_powerGetRsrcRegCfg(pPmicCoreHandle->pmicDeviceType,
                                     &pPwrRsrcRegCfg);
    if(PMIC_ST_SUCCESS == status)
    {
        irqNum = pPwrRsrcRegCfg[pwrRsrcIndex].irqNumber;
        irqNum = irqNum - intrType;
        status = Pmic_irqMaskIntr(pPmicCoreHandle, irqNum, intrEnable);
    }

    return status;
}

/*!
 * \brief   This function is used to get the PMIC interrupt
            configuration for the specific PMIC device.
 */
static int32_t Pmic_powerIntCfg(uint8_t              pmicDeviceType,
                                Pmic_powerIntCfg_t **pPwrCommonIntCfg)
{
    int32_t status = PMIC_ST_SUCCESS;

    if(NULL == pPwrCommonIntCfg)
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if(PMIC_ST_SUCCESS == status)
    {
        switch(pmicDeviceType)
        {
            case PMIC_DEV_LEO_TPS6594X:
                pmic_get_tps6594x_pwrCommonIntCfg(pPwrCommonIntCfg);
                break;
            case PMIC_DEV_HERA_LP8764X:
                pmic_get_lp8764x_pwrCommonIntCfg(pPwrCommonIntCfg);
                break;
            default:
                status = PMIC_ST_ERR_INV_DEVICE;
                break;
        }
    }

    return status;
}

/*!
 * \brief   This function is used to Enable/Disable Power Interrupt
 */
static int32_t Pmic_powerIntrEnable(Pmic_CoreHandle_t *pPmicCoreHandle,
                                    bool               intrEnable,
                                    const uint8_t      intrType)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_powerIntCfg_t *pPwrIntCfg = NULL;

    /* Get PMIC power interrupt configuration */
    status = Pmic_powerIntCfg(pPmicCoreHandle->pmicDeviceType,
                              &pPwrIntCfg);

    if(PMIC_ST_SUCCESS == status)
    {
        status = Pmic_irqMaskIntr(pPmicCoreHandle,
                                  pPwrIntCfg[intrType].irqNum,
                                  intrEnable);
    }

    return status;
}

/**
 * \brief   API to set power resources configurations.
 *
 * Requirement: REQ_TAG(PDK-5829), REQ_TAG(PDK-5841), REQ_TAG(PDK-5848)
 * Design: did_pmic_power_cfg_readback
 *
 *          This function can be used to configure the various control and
 *          configuration parameters for BUCK/LDO/VCCA/VMON power resources  and
 *          also used to set the various control and configuration of
 *          voltage monitor parameters for BUCK/LDO/VCCA/VMON power resources
 *
 *          To set control and configuration params for BUCK, the application
 *          has to configure the below defined structure members of
 *          Pmic_PowerResourceCfg_t:
 *          rvCheckEn, buckPullDownEn, vmonEn, buckVoutRegSel, buckFpwmMode,
 *          buckFpwmMpMode, regulatorEn, buckCurrentLimit,
 *          buckVmonSlewRate, voltage_mV, pgUvThresholdLvl,
 *          pgOvThresholdLvl, railGrpSel
 *
 *          To set control and configuration params for LDO, the application
 *          has to configure the below defined structure members of
 *          Pmic_PowerResourceCfg_t:
 *          rvCheckEn, vmonEn, regulatorEn, ldoPullDownSel, ldoSlowRampEn,
 *          ldoBypassModeEn, ldoRvTimeoutSel, voltage_mV,
 *          pgUvThresholdLvl, pgOvThresholdLvl, railGrpSel
 *
 *          To set control and configuration params for VCCA, the application
 *          has to configure the below defined structure members of
 *          Pmic_PowerResourceCfg_t:
 *          vmonEn, vccaPwrGudLvl, pgUvThresholdLvl,
 *          pgOvThresholdLvl, railGrpSel
 *
 *          To set control and configuration params for VMON, the application
 *          has to configure the below defined structure members of
 *          Pmic_PowerResourceCfg_t:
 *          rvCheckEn, vmonEn, vccaPwrGudLvl, vmonRange,
 *          pgUvThresholdLvl, pgOvThresholdLvl, railGrpSel, voltage_mV
 *          Valid only for LP8764x HERA Device
 *
 * \param   pPmicCoreHandle    [IN]    PMIC Interface Handle.
 * \param   pwrResource        [IN]    PMIC Power resource
 *                                     Valid values for TPS6594x Leo Device
 *                                     \ref Pmic_Tps6594xLeo_Power_Resource.
 *                                     Valid values for LP8764x HERA Device
 *                                     \ref Pmic_Lp8764xHera_Power_Resource.
 * \param   pwrResourceCfg     [IN]    Power Resource configuration for
 *                                     BUCK/LDO/VMON/VCCA
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code.
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_powerSetPwrResourceCfg(
                             Pmic_CoreHandle_t                 *pPmicCoreHandle,
                             const uint16_t                     pwrResource,
                             const Pmic_PowerResourceCfg_t      pwrResourceCfg)
{
    int32_t  pmicStatus = PMIC_ST_SUCCESS;

    pmicStatus = Pmic_powerParamCheck(pPmicCoreHandle, pwrResource);

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        pmicStatus = Pmic_powerSetPwrResourceCtrlCfg(pPmicCoreHandle,
                                                     pwrResource,
                                                     pwrResourceCfg);
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        pmicStatus = Pmic_powerSetBuckVmonConfig(pPmicCoreHandle,
                                                 pwrResource,
                                                 pwrResourceCfg);
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        pmicStatus = Pmic_powerSetPwrRsrcePgoodConfig(pPmicCoreHandle,
                                                      pwrResource,
                                                      pwrResourceCfg);
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (true == pmic_validParamCheck(pwrResourceCfg.validParams,
                                  PMIC_CFG_REGULATOR_LDO_BYPASS_MODE_EN_VALID)))
    {
        pmicStatus = Pmic_powerValidatecfgDataPwrRsrctype(
                                    PMIC_CFG_REGULATOR_LDO_BYPASS_MODE_EN_VALID,
                                    pwrResource,
                                    pPmicCoreHandle->pmicDeviceType);
        if(PMIC_ST_SUCCESS == pmicStatus)
        {
            /* Set the mode for LDO regulator */
            pmicStatus = Pmic_powerSetLdoBypassModeEn(
                                                pPmicCoreHandle,
                                                pwrResource,
                                                pwrResourceCfg.ldoBypassModeEn);
        }
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (true == pmic_validParamCheck(pwrResourceCfg.validParams,
                                  PMIC_CFG_REGULATOR_LDO_RV_TIMEOUT_SEL_VALID)))
    {
        pmicStatus = Pmic_powerValidatecfgDataPwrRsrctype(
                                    PMIC_CFG_REGULATOR_LDO_RV_TIMEOUT_SEL_VALID,
                                    pwrResource,
                                    pPmicCoreHandle->pmicDeviceType);
        if(PMIC_ST_SUCCESS == pmicStatus)
        {
            /* Set the residual voltage check timeout for LDO regulator */
            pmicStatus = Pmic_powerSetLdoRvTimeoutSel(
                                                pPmicCoreHandle,
                                                pwrResource,
                                                pwrResourceCfg.ldoRvTimeoutSel);
        }
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (true == pmic_validParamCheck(pwrResourceCfg.validParams,
                                     PMIC_CFG_PWR_RESOURCE_RAIL_GRP_SEL_VALID)))
    {
        /* Rail selection for power resources */
        pmicStatus = Pmic_powerSetRailGrpSel(pPmicCoreHandle,
                                             pwrResource,
                                             pwrResourceCfg.railGrpSel);
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (true == pmic_validParamCheck(pwrResourceCfg.validParams,
                                    PMIC_CFG_REGULATOR_VMON_VOLTAGE_SET_VALID)))
    {
        pmicStatus = Pmic_powerValidatecfgDataPwrRsrctype(
                                     PMIC_CFG_REGULATOR_VMON_VOLTAGE_SET_VALID,
                                     pwrResource,
                                     pPmicCoreHandle->pmicDeviceType);
        if(PMIC_ST_SUCCESS == pmicStatus)
        {
            /* Set the voltage level for regulator/ powergood voltage level
               for VMON */
            pmicStatus = Pmic_powerSetVoltage(pPmicCoreHandle,
                                              pwrResource,
                                              pwrResourceCfg.voltage_mV);
        }
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (true == pmic_validParamCheck(pwrResourceCfg.validParams,
                                     PMIC_CFG_REGULATOR_EN_VALID)))
    {
        pmicStatus = Pmic_powerValidatecfgDataPwrRsrctype(
                                               PMIC_CFG_REGULATOR_EN_VALID,
                                               pwrResource,
                                               pPmicCoreHandle->pmicDeviceType);
        if(PMIC_ST_SUCCESS == pmicStatus)
        {
            /* Enable/Disable Regulator */
            pmicStatus = Pmic_powerSetRegulatorEn(pPmicCoreHandle,
                                                  pwrResource,
                                                  pwrResourceCfg.regulatorEn);
        }
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (true == pmic_validParamCheck(pwrResourceCfg.validParams,
                                     PMIC_CFG_VMON_EN_VALID)))
    {
        if(PMIC_ST_SUCCESS == pmicStatus)
        {
            /* Enable/Disable Voltage monitor for all power resources */
            pmicStatus = Pmic_powerSetVmonEn(pPmicCoreHandle,
                                             pwrResource,
                                             pwrResourceCfg.vmonEn);
        }
    }

    return pmicStatus;
}

/**
 * \brief   API to get power resources configurations.
 *
 * Requirement: REQ_TAG(PDK-5829), REQ_TAG(PDK-5848), REQ_TAG(PDK-5850)
 * Design: did_pmic_power_cfg_readback
 *
 *          This function can be used to get the various control and
 *          configuration parameters for BUCK/LDO/VCCA/VMON power resources and
 *          also used to get the various control and configuration of
 *          voltage monitor parameters for BUCK/LDO/VCCA/VMON power resources
 *
 *          Application can get these control and configuration params for BUCK,
 *          which is stored in the below defined structure members of
 *          Pmic_PowerResourceCfg_t:
 *          rvCheckEn, buckPullDownEn, vmonEn, buckVoutRegSel, buckFpwmMode,
 *          buckFpwmMpMode, regulatorEn, buckCurrentLimit,
 *          buckVmonSlewRate, voltage_mV, pgUvThresholdLvl,
 *          pgOvThresholdLvl, railGrpSel
 *
 *          Application can get these control and configuration params for LDO,
 *          which is stored in the below defined structure members of
 *          Pmic_PowerResourceCfg_t:
 *          rvCheckEn, vmonEn, regulatorEn, ldoPullDownSel, ldoSlowRampEn,
 *          ldoBypassModeEn, ldoRvTimeoutSel, voltage_mV,
 *          pgUvThresholdLvl, pgOvThresholdLvl, railGrpSel
 *
 *          Application can get these control and configuration params for VCCA,
 *          which is stored in the below defined structure members of
 *          Pmic_PowerResourceCfg_t:
 *          vmonEn, vccaPwrGudLvl, pgUvThresholdLvl,
 *          pgOvThresholdLvl, railGrpSel
 *
 *          Application can get these control and configuration params for VMON,
 *          which is stored in the below defined structure members of
 *          Pmic_PowerResourceCfg_t:
 *          rvCheckEn, vmonEn, vccaPwrGudLvl, vmonRange,
 *          pgUvThresholdLvl, pgOvThresholdLvl, railGrpSel, voltage_mV
 *          Valid only for LP8764x HERA Device
 *
 * \param   pPmicCoreHandle    [IN]    PMIC Interface Handle.
 * \param   pwrResource        [IN]    PMIC Power resource
 *                                     Valid values for TPS6594x Leo Device
 *                                     \ref Pmic_Tps6594xLeo_Power_Resource.
 *                                     Valid values for LP8764x HERA Device
 *                                     \ref Pmic_Lp8764xHera_Power_Resource.
 * \param   pPwrResourceCfg    [OUT]   Pointer to store Power Resource
 *                                     configuration for BUCK/LDO/VMON/VCCA
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code.
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_powerGetPwrResourceCfg(
                                   Pmic_CoreHandle_t           *pPmicCoreHandle,
                                   const uint16_t               pwrResource,
                                   Pmic_PowerResourceCfg_t     *pPwrResourceCfg)
{
    int32_t  pmicStatus = PMIC_ST_SUCCESS;

    pmicStatus = Pmic_powerParamCheck(pPmicCoreHandle, pwrResource);

    if(NULL == pPwrResourceCfg)
    {
        pmicStatus = PMIC_ST_ERR_NULL_PARAM;
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        pmicStatus = Pmic_powerGetPwrResourceCtrlCfg(pPmicCoreHandle,
                                                     pwrResource,
                                                     pPwrResourceCfg);
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        pmicStatus = Pmic_powerGetBuckVmonConfig(pPmicCoreHandle,
                                                 pwrResource,
                                                 pPwrResourceCfg);
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        Pmic_powerGetPwrRsrcePgoodConfig(pPmicCoreHandle,
                                         pwrResource,
                                         pPwrResourceCfg);
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (true == pmic_validParamCheck(pPwrResourceCfg->validParams,
                                  PMIC_CFG_REGULATOR_LDO_BYPASS_MODE_EN_VALID)))
    {
        pmicStatus = Pmic_powerValidatecfgDataPwrRsrctype(
                                    PMIC_CFG_REGULATOR_LDO_BYPASS_MODE_EN_VALID,
                                    pwrResource,
                                    pPmicCoreHandle->pmicDeviceType);
        if(PMIC_ST_SUCCESS == pmicStatus)
        {
             /* Get the mode for LDO regulator */
            pmicStatus = Pmic_powerGetLdoBypassModeEn(
                                           pPmicCoreHandle,
                                           pwrResource,
                                           &(pPwrResourceCfg->ldoBypassModeEn));
        }
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (true == pmic_validParamCheck(pPwrResourceCfg->validParams,
                                  PMIC_CFG_REGULATOR_LDO_RV_TIMEOUT_SEL_VALID)))
    {
        pmicStatus = Pmic_powerValidatecfgDataPwrRsrctype(
                                    PMIC_CFG_REGULATOR_LDO_RV_TIMEOUT_SEL_VALID,
                                    pwrResource,
                                    pPmicCoreHandle->pmicDeviceType);
        if(PMIC_ST_SUCCESS == pmicStatus)
        {
            /* Get the residual voltage check timeout for LDO regulator */
            pmicStatus = Pmic_powerGetLdoRvTimeoutSel(
                                           pPmicCoreHandle,
                                           pwrResource,
                                           &(pPwrResourceCfg->ldoRvTimeoutSel));
        }
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (true == pmic_validParamCheck(pPwrResourceCfg->validParams,
                                     PMIC_CFG_PWR_RESOURCE_RAIL_GRP_SEL_VALID)))
    {
        if(PMIC_ST_SUCCESS == pmicStatus)
        {
            /* Get Rail selection for power resources */
            pmicStatus = Pmic_powerGetRailGrpSel(
                                               pPmicCoreHandle,
                                               pwrResource,
                                               &(pPwrResourceCfg->railGrpSel));
        }
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (true == pmic_validParamCheck(pPwrResourceCfg->validParams,
                                    PMIC_CFG_REGULATOR_VMON_VOLTAGE_SET_VALID)))
    {
        pmicStatus = Pmic_powerValidatecfgDataPwrRsrctype(
                                      PMIC_CFG_REGULATOR_VMON_VOLTAGE_SET_VALID,
                                      pwrResource,
                                      pPmicCoreHandle->pmicDeviceType);
        if(PMIC_ST_SUCCESS == pmicStatus)
        {
            /* Get the voltage level of the regulator */
            pmicStatus = Pmic_powerGetVoltage(pPmicCoreHandle,
                                              pwrResource,
                                              &(pPwrResourceCfg->voltage_mV));
        }
    }

    return pmicStatus;
}

/**
 * \brief   API to Set Power configuration
 *
 * Requirement: REQ_TAG(PDK-5829), REQ_TAG(PDK-5848), REQ_TAG(PDK_5847)
 * Design: did_pmic_power_cfg_readback, did_pmic_power_pgood_cfg_readback
 *
 *          This function is used to set the power configuration
 *          parameters such as selection of type of voltage monitoring, and
 *          polarity of the power-good signal, deglitch time select for all
 *          power resources.
 *
 *          Application can set the voltage monitoring for PGOOD
 *          by configuring the following structure members of
 *          Pmic_PowerCommonCfg_t:
 *          pgoodWindow
 *
 *          Application can set the PGOOD signal polarity
 *          by configuring the following structure members of
 *          Pmic_PowerCommonCfg_t:
 *          pgoodPolarity
 *
 *          Application can set the Deglitch time select for all power resources
 *          by configuring the following structure members of
 *          Pmic_PowerCommonCfg_t:
 *          deglitchTimeSel
 *
 *          Application can set/select  trigger selection for :
 *          severe Error, other rail group, soc rail group, mcu rail group and
 *          Moderate Error
 *          by configuring the following structure members of
 *          Pmic_PowerCommonCfg_t:
 *          severeErrorTrig, otherRailTrig, socRailTrig, mcuRailTrig
 *          moderateRailTrig
 *
 *
 * \param   pPmicCoreHandle  [IN]    PMIC Interface Handle.
 * \param   powerCommonCfg   [IN]    Power configuration.
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code.
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_powerSetCommonConfig(Pmic_CoreHandle_t     *pPmicCoreHandle,
                                  Pmic_PowerCommonCfg_t  powerCommonCfg)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    if(NULL == pPmicCoreHandle)
    {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (true == pmic_validParamCheck(powerCommonCfg.validParams,
                                     PMIC_POWER_PGOOD_WINDOW_VALID)))
    {
        pmicStatus = Pmic_powerSetPgoodWindow(pPmicCoreHandle,
                                              powerCommonCfg.pgoodWindow);
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (true == pmic_validParamCheck(powerCommonCfg.validParams,
                                     PMIC_POWER_PGOOD_POL_VALID)))
    {
        /* Set PGOOD signal polarity */
        pmicStatus = Pmic_powerSetPgoodPolarity(pPmicCoreHandle,
                                                powerCommonCfg.pgoodPolarity);
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (true == pmic_validParamCheck(powerCommonCfg.validParams,
                                     PMIC_CFG_DEGLITCH_TIME_SEL_VALID)))
    {
        /* Set Deglitch time select for BUCKx_VMON/LDOx_VMON/VCCA_VMON/VMONX */
        pmicStatus = Pmic_powerSetDeglitchTimeSel(
                                                pPmicCoreHandle,
                                                powerCommonCfg.deglitchTimeSel);
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (true == pmic_validParamCheck(powerCommonCfg.validParams,
                                     PMIC_SEVERE_ERR_TRIG_VALID)))
    {
        /* Set the trigger selection for severe Error */
        pmicStatus = Pmic_powerSetSevereErrorTrig(
                                                pPmicCoreHandle,
                                                powerCommonCfg.severeErrorTrig);
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (true == pmic_validParamCheck(powerCommonCfg.validParams,
                                     PMIC_OTHER_RAIL_TRIG_VALID)))
    {
        /* Set trigger selection fro other rail group */
        pmicStatus = Pmic_powerSetOtherRailTrig(pPmicCoreHandle,
                                                powerCommonCfg.otherRailTrig);
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (true == pmic_validParamCheck(powerCommonCfg.validParams,
                                     PMIC_SOC_RAIL_TRIG_VALID)))
    {
        /* Set the trigger selection for soc rail group */
        pmicStatus = Pmic_powerSetSocRailTrig(pPmicCoreHandle,
                                              powerCommonCfg.socRailTrig);
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (true == pmic_validParamCheck(powerCommonCfg.validParams,
                                     PMIC_MCU_RAIL_TRIG_VALID)))
    {
        /* Set the trigger selection for mcu rail group */
        pmicStatus = Pmic_powerSetMcuRailTrig(pPmicCoreHandle,
                                              powerCommonCfg.mcuRailTrig);
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (true == pmic_validParamCheck(powerCommonCfg.validParams,
                                     PMIC_MODERATE_ERR_TRIG_VALID)))
    {
        /* Set the trigger selection for Moderate Error */
        pmicStatus = Pmic_powerSetModerateRailTrig(
                                               pPmicCoreHandle,
                                               powerCommonCfg.moderateRailTrig);
    }

    return pmicStatus;
}

/**
 * \brief   API to Get Power configuration
 *
 * Requirement: REQ_TAG(PDK-5829), REQ_TAG(PDK-5848), REQ_TAG(PDK_5847)
 * Design: did_pmic_power_cfg_readback, did_pmic_power_pgood_cfg_readback
 *
 *          This function is used to get the power configuration
 *          parameters such as selection of type of voltage monitoring, and
 *          polarity of the power-good signal, deglitch time select for all
 *          power resources.
 *
 *          Application can get the voltage monitoring for PGOOD
 *          by configuring the following structure members of
 *          Pmic_PowerCommonCfg_t:
 *          pgoodWindow
 *
 *          Application can get the PGOOD signal polarity
 *          by configuring the following structure members of
 *          Pmic_PowerCommonCfg_t:
 *          pgoodPolarity
 *
 *          Application can get the Deglitch time select for all power resources
 *          by configuring the following structure members of
 *          Pmic_PowerCommonCfg_t:
 *          deglitchTimeSel
 *
 *          Application can get trigger selection for :
 *          severe Error, other rail group, soc rail group, mcu rail group and
 *          Moderate Error
 *          by configuring the following structure members of
 *          Pmic_PowerCommonCfg_t:
 *          severeErrorTrig, otherRailTrig, socRailTrig, mcuRailTrig
 *          moderateRailTrig
 *
 * \param   pPmicCoreHandle  [IN]    PMIC Interface Handle.
 * \param   pPowerCommonCfg  [OUT]   Pointer to hold Power configuration.
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code.
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_powerGetCommonConfig(Pmic_CoreHandle_t     *pPmicCoreHandle,
                                  Pmic_PowerCommonCfg_t *pPowerCommonCfg)
{
    int32_t  pmicStatus = PMIC_ST_SUCCESS;

    if(NULL == pPmicCoreHandle)
    {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (NULL == pPowerCommonCfg))
    {
        pmicStatus = PMIC_ST_ERR_NULL_PARAM;
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (true == pmic_validParamCheck(pPowerCommonCfg->validParams,
                                     PMIC_POWER_PGOOD_WINDOW_VALID)))
    {
        /* Type of voltage monitoring for PGOOD signal */
        pmicStatus = Pmic_powerGetVoltageMonitoringPg(
                                               pPmicCoreHandle,
                                               &(pPowerCommonCfg->pgoodWindow));
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (true == pmic_validParamCheck(pPowerCommonCfg->validParams,
                                     PMIC_POWER_PGOOD_POL_VALID)))
    {
        /* Get PGOOD signal polarity */
        pmicStatus = Pmic_powerGetPgoodPolarity(
                                             pPmicCoreHandle,
                                             &(pPowerCommonCfg->pgoodPolarity));
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (true == pmic_validParamCheck(pPowerCommonCfg->validParams,
                                     PMIC_CFG_DEGLITCH_TIME_SEL_VALID)))
    {
        /* get Deglitch time select for BUCKx_VMON, LDOx_VMON and VCCA_VMON */
        pmicStatus = Pmic_powerGetDeglitchTimeSel(
                                           pPmicCoreHandle,
                                           &(pPowerCommonCfg->deglitchTimeSel));
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (true == pmic_validParamCheck(pPowerCommonCfg->validParams,
                                     PMIC_SEVERE_ERR_TRIG_VALID)))
    {
        /* get Trigger selection for Severe Error */
        pmicStatus = Pmic_powerGetSevereErrorTrig(
                                           pPmicCoreHandle,
                                           &(pPowerCommonCfg->severeErrorTrig));
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (true == pmic_validParamCheck(pPowerCommonCfg->validParams,
                                     PMIC_OTHER_RAIL_TRIG_VALID)))
    {
        /* get Trigger selection for OTHER rail group */
        pmicStatus = Pmic_powerGetOtherRailTrig(
                                             pPmicCoreHandle,
                                             &(pPowerCommonCfg->otherRailTrig));
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (true == pmic_validParamCheck(pPowerCommonCfg->validParams,
                                     PMIC_SOC_RAIL_TRIG_VALID)))
    {
        /* get Trigger selection for SOC rail group */
        pmicStatus = Pmic_powerGetSocRailTrig(pPmicCoreHandle,
                                              &(pPowerCommonCfg->socRailTrig));
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (true == pmic_validParamCheck(pPowerCommonCfg->validParams,
                                     PMIC_MCU_RAIL_TRIG_VALID)))
    {
        /* get Trigger selection for MCU rail group */
        pmicStatus = Pmic_powerGetMcuRailTrig(pPmicCoreHandle,
                                              &(pPowerCommonCfg->mcuRailTrig));
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (true == pmic_validParamCheck(pPowerCommonCfg->validParams,
                                     PMIC_MODERATE_ERR_TRIG_VALID)))
    {
        /* get Trigger selection for Moderate Error */
        pmicStatus = Pmic_powerGetModerateRailTrig(
                                          pPmicCoreHandle,
                                          &(pPowerCommonCfg->moderateRailTrig));
    }

    return pmicStatus;
}

/**
 * \brief   API to Set Power good configuration
 *
 * Requirement: REQ_TAG(PDK-5847)
 * Design: did_pmic_power_pgood_cfg_readback
 *
 *          This function is used to control and configure the power good
 *          source control. For the
 *          following, power good signal control can be selected:
 *          All supported Bucks and Ldo by the PMIC, VCCA , thermal warning,
 *          nRSTOUT pin and nRSTOUT_SOC pin.
 *
 * \param   pPmicCoreHandle  [IN]     PMIC Interface Handle.
 * \param   pgoodSrcSel      [IN]     Power Good Source.
 *                                    Valid values for TPS6594x Leo Device
 *                                    \ref Pmic_Tps6594xLeo_Pgood_SourceType.
 *                                    Valid values for LP8764x HERA Device
 *                                    \ref Pmic_Lp8764xHera_Pgood_SourceType.
 * \param   pgoodSelType     [IN]     Power Good configuration.
 *                                    Valid values for TPS6594x Leo Device
 *                                For  LDO/BUCK
 *                                \ref Pmic_TPS6594x_Power_Good_Regulator_Signal
 *                                For  VCCA
 *                                    \ref Pmic_TPS6594x_Power_Good_Vcca
 *                                For  Thermal Warning
 *                                    \ref Pmic_TPS6594x_Power_Good_Thermal_Warn
 *                                For  nRSTOUT
 *                                    \ref Pmic_TPS6594x_Power_Good_Nrstout
 *                                For  nRSTOUT_SOC
 *                                    \ref Pmic_TPS6594x_Power_Good_Nrstout_Soc
 *                                    Valid values for LP8764x HERA Device
 *                                For  BUCK
 *                                    \ref Pmic_LP8764x_Power_Good_Buck_Signal
 *                                For  VCCA/VMON
 *                                    \ref Pmic_LP8764x_Power_Good_Vcca_Vmon
 *                                For : Thermal Warning
 *                                    \ref Pmic_LP8764x_Power_Good_Thermal_Warn
 *                                For  nRSTOUT
 *                                    \ref Pmic_LP8764x_Power_Good_Nrstout
 *                                For  nRSTOUT_SOC
 *                                    \ref Pmic_LP8764x_Power_Good_Nrstout_Soc
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code.
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_powerSetConfigPowerGood(Pmic_CoreHandle_t   *pPmicCoreHandle,
                                     uint16_t             pgoodSrcSel,
                                     uint8_t              pgoodSelType)
{
    int32_t  pmicStatus = PMIC_ST_SUCCESS;

    pmicStatus = Pmic_validatePowerGoodParam(pPmicCoreHandle, pgoodSrcSel);

    if(pmicStatus == PMIC_ST_SUCCESS)
    {
        pmicStatus = Pmic_powerValidatePgoodSelType(
                                                pPmicCoreHandle->pmicDeviceType,
                                                pgoodSrcSel,
                                                pgoodSelType);
    }

    if(pmicStatus == PMIC_ST_SUCCESS)
    {
        pmicStatus = Pmic_setPgoodSelType(pPmicCoreHandle,
                                          pgoodSrcSel,
                                          pgoodSelType);
    }

    return pmicStatus;
}

/**
 * \brief   Get Power good configuration
 *
 * Requirement: REQ_TAG(PDK-5847)
 * Design: did_pmic_power_pgood_cfg_readback
 *
 *          This function is used to get various power good conifg.
 *          This funci=tion also provides the pgood source control for
 *          different power resources and pins.
 *
 * \param   pPmicCoreHandle  [IN]     PMIC Interface Handle.
 * \param   pgoodSrcSel      [IN]     Power Good Source.
 *                                    Valid values for TPS6594x Leo Device
 *                                    \ref Pmic_Tps6594xLeo_Pgood_SourceType.
 *                                    Valid values for LP8764x HERA Device
 *                                    \ref Pmic_Lp8764xHera_Pgood_SourceType.
 * \param   pPgoodSelType    [OUT]    Power Good configuration.
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code.
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_powerGetConfigPowerGood(Pmic_CoreHandle_t *pPmicCoreHandle,
                                     uint16_t           pgoodSrcSel,
                                     uint8_t           *pPgoodSelType)
{
    int32_t  pmicStatus = PMIC_ST_SUCCESS;

    pmicStatus = Pmic_validatePowerGoodParam(pPmicCoreHandle, pgoodSrcSel);

    if((pmicStatus == PMIC_ST_SUCCESS) &&
       (NULL == pPgoodSelType))
    {
        pmicStatus = PMIC_ST_ERR_NULL_PARAM;
    }

    if(pmicStatus == PMIC_ST_SUCCESS)
    {
        pmicStatus = Pmic_getPgoodSelType(pPmicCoreHandle,
                                          pgoodSrcSel,
                                          pPgoodSelType);
    }

    return pmicStatus;
}

/**
 * \brief   API to get power resources status.
 *
 * Requirement: REQ_TAG(PDK-5829), REQ_TAG(PDK-5848), REQ_TAG(PDK-5850)
 * Design: did_pmic_power_cfg_readback
 *
 *          This function can be used to get the status related to current limit
 *          , voltage over and under limit for BUCK/LDO/VCCA/VMON power
 *          resources
 *
 *          Application can get the current limit status that if the output
 *          current is above current limit level for BUCK by configuring the
 *          following structure members of
 *          Pmic_PowerResourceStat_t:
 *          currentLimitLvlStat
 *
 *          Application can get the output voltage status that if the
 *          output voltage is below undervoltage threshold for BUCK/LDO by
 *          configuring the following structure members of
 *          Pmic_PowerResourceStat_t :
 *          underVoltageTholdStat
 *          For VMON/VCCA the same member is used to get the input voltage
 *          status that if input voltage is below undervoltage level.
 *
 *          Application can get the output voltage status that if the
 *          output voltage is above overvoltage threshold for BUCK/LDO by
 *          configuring the following structure members of
 *          Pmic_PowerResourceStat_t :
 *          overVoltageTholdStat
 *          For VMON/VCCA the same member is used to get the input voltage
 *          status that if input voltage is above overvoltage level.
 *
 *          Application can get the VCCA voltage status that if the VCCA
 *          voltage is above overvoltage protection level  by configuring the
 *          following structure members of
 *          Pmic_PowerResourceStat_t :
 *          overVoltageProtectionLvlStat
 *
 *
 * \param   pPmicCoreHandle    [IN]    PMIC Interface Handle.
 * \param   pwrResource        [IN]    PMIC Power resource
 *                                     Valid values for TPS6594x Leo Device
 *                                     \ref Pmic_Tps6594xLeo_Power_Resource.
 *                                     Valid values for LP8764x HERA Device
 *                                     \ref Pmic_Lp8764xHera_Power_Resource.
 * \param   pPwrRsrcStatCfg    [OUT]   Pointer to store Power Resource
 *                                     configuration for BUCK/LDO/VMON/VCCA
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code.
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_powerGetPwrRsrcStat(Pmic_CoreHandle_t        *pPmicCoreHandle,
                                 const uint16_t            pwrResource,
                                 Pmic_PowerResourceStat_t *pPwrRsrcStatCfg)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t pwrRsrcType;

    pmicStatus = Pmic_powerParamCheck(pPmicCoreHandle, pwrResource);

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        pwrRsrcType = Pmic_powerGetPwrRsrcType(pwrResource);
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (NULL == pPwrRsrcStatCfg))
    {
        pmicStatus = PMIC_ST_ERR_NULL_PARAM;
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (true == pmic_validParamCheck(pPwrRsrcStatCfg->validParams,
                                     PMIC_POWER_REGULATOR_ILIM_STAT_VALID)))
    {
        pmicStatus = Pmic_powerValidateBuckLdoPwrRsrcType(
                                                pPmicCoreHandle->pmicDeviceType,
                                                pwrRsrcType);
        if(PMIC_ST_SUCCESS == pmicStatus)
        {
            /* Get the current limit level. */
            pmicStatus = Pmic_powerGetCurrentLimitLvlStat(
                                       pPmicCoreHandle,
                                       pwrResource,
                                       &(pPwrRsrcStatCfg->currentLimitLvlStat));
        }
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (true == pmic_validParamCheck(pPwrRsrcStatCfg->validParams,
                                  PMIC_POWER_RESOURCE_UV_STAT_VALID)))
    {
        if(PMIC_ST_SUCCESS == pmicStatus)
        {
             /* Get the output under voltage status */
            pmicStatus = Pmic_powerGetUnderVoltageTholdStat(
                                     pPmicCoreHandle,
                                     pwrResource,
                                     &(pPwrRsrcStatCfg->underVoltageTholdStat));
        }
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (true == pmic_validParamCheck(pPwrRsrcStatCfg->validParams,
                                  PMIC_POWER_RESOURCE_OV_STAT_VALID)))
    {
        if(PMIC_ST_SUCCESS == pmicStatus)
        {
             /* Get the output over voltage status */
            pmicStatus = Pmic_powerGetOverVoltageTholdStat(
                                      pPmicCoreHandle,
                                      pwrResource,
                                      &(pPwrRsrcStatCfg->overVoltageTholdStat));
        }
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (true == pmic_validParamCheck(pPwrRsrcStatCfg->validParams,
                                  PMIC_POWER_VCCA_OV_LVL_STAT_VALID)))
    {
        pmicStatus = Pmic_powerValidateVccaPwrRsrcType(
                                                pPmicCoreHandle->pmicDeviceType,
                                                pwrRsrcType);
        if(PMIC_ST_SUCCESS == pmicStatus)
        {
            /* Get the voltage level status for VCCA */
            pmicStatus = Pmic_powerGetOverVoltageProtectionLvlStat(
                              pPmicCoreHandle,
                              pwrResource,
                              &(pPwrRsrcStatCfg->overVoltageProtectionLvlStat));
        }
    }

    return pmicStatus;
}

/**
 * \brief   API to get PMIC die temperature thermal status.
 *
 * Requirement: REQ_TAG(PDK-5840)
 * Design: did_pmic_power_thermal_cfg_readback
 *
 *          This function is used to get the thermal status of the PMIC
 *          (die temperature)
 *
 *          Application can get the thermal status that if the  die junction
 *          above the thermal warning level  by
 *          configuring the following structure members of
 *          Pmic_PowerThermalStat_t:
 *          thermalStateWarning
 *
 *          Application can get the thermal status that if the  die junction
 *          above the thermal level causing a sequenced shutdown by
 *          configuring the following structure members of
 *          Pmic_PowerThermalStat_t:
 *          thermalStateOderlyShtDwn
 *
 *          Application can get the thermal status that if the  die junction
 *          above the thermal level causing an immediate shutdown by
 *          configuring the following structure members of
 *          Pmic_PowerThermalStat_t:
 *          thermalStateImmShtDwn
 *
 * \param   pPmicCoreHandle       [IN]    PMIC Interface Handle.
 * \param   pPwrThermalStatCfg    [OUT]   Pointer to store Thermal
 *                                        configuration for PMIC
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code.
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_powerGetPwrThermalStat(Pmic_CoreHandle_t       *pPmicCoreHandle,
                                    Pmic_PowerThermalStat_t *pPwrThermalStatCfg)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    if(NULL == pPmicCoreHandle)
    {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (NULL == pPwrThermalStatCfg))
    {
        pmicStatus = PMIC_ST_ERR_NULL_PARAM;
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (true == pmic_validParamCheck(pPwrThermalStatCfg->validParams,
                                     PMIC_THERMAL_STAT_WARN_VALID)))
    {
        /* Get the thermal warning status   */
        pmicStatus = Pmic_getThermalWarnStat(
                                    pPmicCoreHandle,
                                    &(pPwrThermalStatCfg->thermalStateWarning));
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (true == pmic_validParamCheck(pPwrThermalStatCfg->validParams,
                                     PMIC_THERMAL_STAT_ORD_SHTDWN_VALID)))
    {
        /* Get the thermal oderly shutdown status   */
        pmicStatus = Pmic_getOderlyShutdownStat(
                               pPmicCoreHandle,
                               &(pPwrThermalStatCfg->thermalStateOderlyShtDwn));
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (true == pmic_validParamCheck(pPwrThermalStatCfg->validParams,
                                     PMIC_THERMAL_STAT_IMM_SHTDWN_VALID)))
    {
        /* Get the thermal immediate shutdown status */
        pmicStatus = Pmic_getImmediateShutdownStat(
                                  pPmicCoreHandle,
                                  &(pPwrThermalStatCfg->thermalStateImmShtDwn));
    }

    return pmicStatus;
}

/**
 * \brief    API to configure the thermal temperature threshold level for PMIC.
 *
 * Requirement: REQ_TAG(PDK-5840)
 * Design: did_pmic_power_thermal_cfg_readback
 *
 *          To configure the the thermal wrarning threshold temperature level,
 *          the application has to configure the below defined structure
 *          member of the Pmic_PowerThermalCfg_t:
 *          thermalWarnThold
 *
 *          To configure the the thermal shutdown threshold temperature level,
 *          the application has to configure the below defined structure member
 *          of the Pmic_PowerThermalCfg_t:
 *          thermalShutdownThold
 *
 * \param   pPmicCoreHandle       [IN]    PMIC Interface Handle.
 * \param   thermalThreshold      [IN]    Thermal Configuration.
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code.
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_powerSetThermalConfig(Pmic_CoreHandle_t     *pPmicCoreHandle,
                                   Pmic_PowerThermalCfg_t thermalThreshold)
{
    int32_t  pmicStatus = PMIC_ST_SUCCESS;

    if(NULL == pPmicCoreHandle)
    {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (true == pmic_validParamCheck(thermalThreshold.validParams,
                                     PMIC_THERMAL_WARN_VALID)))
    {
        /* Set the thermal warning threshold temperature value */
        pmicStatus = Pmic_setThermalWarnThold(
                                             pPmicCoreHandle,
                                             thermalThreshold.thermalWarnThold);
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (true == pmic_validParamCheck(thermalThreshold.validParams,
                                     PMIC_THERMAL_SHTDWN_VALID)))
    {
        /* Set the thermal shutdown threshold temperature value */
        pmicStatus = Pmic_setThermalShutdownThold(
                                         pPmicCoreHandle,
                                         thermalThreshold.thermalShutdownThold);
    }

    return pmicStatus;
}

/**
 * \brief   Get the PMIC thermal threshold value function.
 *
 * Requirement: REQ_TAG(PDK-5840)
 * Design: did_pmic_power_thermal_cfg_readback
 *
 *          This function is used to get the thermal temperature threshold
 *          value for the PMIC.
 *
 *          To get the the thermal wrarning threshold temperature level,
 *          the application has to configure the below defined structure
 *          member of the Pmic_PowerThermalCfg_t:
 *          thermalWarnThold
 *
 *          To get the the thermal shutdown threshold temperature level,
 *          the application has to configure the below defined structure member
 *          of the Pmic_PowerThermalCfg_t:
 *          thermalShutdownThold
 *
 * \param   pPmicCoreHandle       [IN]     PMIC Interface Handle.
 * \param   pThermalThreshold     [OUT]    Pointer to hold Thermal Cfg
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code.
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_powerGetThermalConfig(Pmic_CoreHandle_t      *pPmicCoreHandle,
                                   Pmic_PowerThermalCfg_t *pThermalThreshold)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    if(NULL == pPmicCoreHandle)
    {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (NULL == pThermalThreshold))
    {
        pmicStatus = PMIC_ST_ERR_NULL_PARAM;
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (true == pmic_validParamCheck(pThermalThreshold->validParams,
                                     PMIC_THERMAL_WARN_VALID)))
    {
        /* Get the thermal warning threshold temperature value */
        pmicStatus = Pmic_getThermalWarnThold(
                                        pPmicCoreHandle,
                                        &(pThermalThreshold->thermalWarnThold));
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (true == pmic_validParamCheck(pThermalThreshold->validParams,
                                     PMIC_THERMAL_SHTDWN_VALID)))
    {
        /* Get the thermal shutdown threshold temperature value */
        pmicStatus = Pmic_getThermalShutdownThold(
                                    pPmicCoreHandle,
                                    &(pThermalThreshold->thermalShutdownThold));
    }

    return pmicStatus;
}

/*!
 * \brief   API to enable/disable Power interrupt.
 *
 * Requirement: REQ_TAG(PDK-5829), REQ_TAG(PDK-5848)
 * Design: did_pmic_power_cfg_readback
 *
 *          This function is used to enable/disable thermal Interrupts
 *
 *
 * \param   pPmicCoreHandle    [IN]    PMIC Interface Handle.
 * \param   pwrResource        [IN]    PMIC Power resource
 *                                     Valid values for TPS6594x Leo Device
 *                                     \ref Pmic_Tps6594xLeo_Power_Resource.
 *                                     Valid values for LP8764x HERA Device
 *                                     \ref Pmic_Lp8764xHera_Power_Resource
 * \param   intrType           [IN]    Interrupt type
 *                                     Valid values for TPS6594x Leo Device
 *                                     \ref Pmic_Tps6594x_PowerInterruptType
 *                                     Valid values for LP8764x HERA Device
 *                                     \ref Pmic_LP8764x_PowerInterruptType
 * \param   intrEnable         [IN]    Enable/Disable the interrupt.
 *                                     For Vaild values:
 *                                     \ref Pmic_PowerInterruptCfg
 *
 * \return  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_powerSetPwrRsrcIntr(Pmic_CoreHandle_t *pPmicCoreHandle,
                                 const uint16_t     pwrResource,
                                 const uint8_t      intrType,
                                 bool               intrEnable)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    pmicStatus = Pmic_powerParamCheck(pPmicCoreHandle, pwrResource);

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        pmicStatus = Pmic_powerValidateIntrType(pPmicCoreHandle->pmicDeviceType,
                                                pwrResource,
                                                intrType);
    }
    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        pmicStatus = Pmic_powerRsrcIntrEnable(pPmicCoreHandle,
                                              pwrResource,
                                              intrEnable,
                                              intrType);
    }

    return pmicStatus;
}

/*!
 * \brief   API to enable/disable Power interrupt.
 *
 * Requirement: REQ_TAG(PDK-5841), REQ_TAG(PDK-5840)
 * Design: did_pmic_power_cfg_readback, did_pmic_power_thermal_cfg_readback
 *
 *          This function is used to enable/disable power Interrupts
 *
 * \param   pPmicCoreHandle  [IN]    PMIC Interface Handle.
 * \param   intrType         [IN]    Interrupt type
 *                                   Valid values for TPS6594x Leo Device
 *                                   \ref Pmic_Tps6594x_PowerInterruptCommonType
 *                                   Valid values for LP8764x HERA Device
 *                                   \ref Pmic_LP8764x_PowerInterruptCommonType
 * \param   intrEnable       [IN]    Enable/Disable the interrupt.
 *                                   For Vaild values:
 *                                   \ref Pmic_PowerInterruptCfg
 *
 * \return  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_powerSetIntr(Pmic_CoreHandle_t *pPmicCoreHandle,
                          const uint8_t      intrType,
                          bool               intrEnable)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    pmicStatus = Pmic_validateIntr(pPmicCoreHandle, intrType);

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        pmicStatus = Pmic_powerIntrEnable(pPmicCoreHandle,
                                          intrEnable,
                                          intrType);
    }

    return pmicStatus;
}
