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
        case PMIC_CFG_REGULATOR_BUCK_FREQ_SEL_VALID:
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
 * \brief   This function is used to get the required BitPos of residual
 *          voltage checking for BUCK/LDO/VMON pin.
 */
static int32_t Pmic_powerGetRvCheckEnBitPos(uint8_t   pmicDeviceType,
                                            uint16_t  pwrRsrc,
                                            uint8_t  *pBitPos)
{
    int32_t status = PMIC_ST_SUCCESS;

    switch (pmicDeviceType)
    {
        case PMIC_DEV_LEO_TPS6594X:
            *pBitPos = PMIC_POWER_RESOURCEX_CTRL_POWER_RESOURCEX_RV_SEL_SHIFT;
            break;
        case PMIC_DEV_HERA_LP8764X:
            if(PMIC_LP8764X_POWER_SOURCE_VMON1 == pwrRsrc)
            {
                 *pBitPos = PMIC_POWER_VCCA_VMON_CTRL_VMON1_RV_SEL_SHIFT;

            }
            else if(PMIC_LP8764X_POWER_SOURCE_VMON2 == pwrRsrc)
            {
                *pBitPos = PMIC_POWER_VCCA_VMON_CTRL_VMON2_RV_SEL_SHIFT;
            }
            else
            {
                *pBitPos = \
                        PMIC_POWER_RESOURCEX_CTRL_POWER_RESOURCEX_RV_SEL_SHIFT;
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
                                                &bitPos);

            BIT_POS_SET_VAL(regData, bitPos, residualVoltCheck);
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
                                                &bitPos);

        *pResidualVoltCheck = BIT_POS_GET_VAL(regData, bitPos);
    }

    return pmicStatus;
}

/*!
 * \brief   This function is used to convert the vset value to voltage in mv
            for BUCK/VMON
 */
int32_t Pmic_powerBuckVmonConvertVSetVal2Voltage(uint8_t   *pVSetVal,
                                                 uint16_t  *pBaseMillivolt,
                                                 uint8_t   *pMillivoltStep,
                                                 uint8_t   *pBaseVoutCode)
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
int32_t Pmic_powerVmonRange1ConvertVSetVal2Voltage(uint8_t  *pVSetVal,
                                                   uint16_t *pBaseMillivolt,
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
int32_t Pmic_powerLdoConvertVSetVal2Voltage(uint8_t  *pVSetVal,
                                            uint16_t  pwrRsrc,
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
int32_t Pmic_powerVmonRange1ConvertVoltage2VSetVal(uint16_t millivolt,
                                                   uint16_t *pBaseMillivolt,
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
int32_t Pmic_powerLdoConvertVoltage2VSetVal(uint16_t  millivolt,
                                            uint16_t  pwrRsrc,
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
                                   pPwrRsrcRegCfg[pwrRsrcIndex].pgWindowRegAddr,
                                   &regData);
        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        *pVmonRange = HW_REG_GET_FIELD(regData,
                                       PMIC_VMON_PG_WINDOW_VMON_RANGE);
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
       (true  == millivolt % millivoltStep))
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
            HW_REG_SET_FIELD(regData, PMIC_BUCKX_CTRL_BUCKX_VSEL, buckVoutSel);

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
        *pBuckVoutSel = HW_REG_GET_FIELD(regData, PMIC_BUCKX_CTRL_BUCKX_VSEL);
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
            HW_REG_SET_FIELD(regData, PMIC_BUCKX_CTRL_BUCKX_FPWM, buckFpwmMode);

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
        *pBuckFpwmMode = HW_REG_GET_FIELD(regData,
                                          PMIC_BUCKX_CTRL_BUCKX_FPWM);
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
            HW_REG_SET_FIELD(regData,
                             PMIC_BUCKX_CTRL_BUCKX_FPWM_MP,
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
        *pBuckFpwmMpMode = HW_REG_GET_FIELD(regData,
                                            PMIC_BUCKX_CTRL_BUCKX_FPWM_MP);
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
            HW_REG_SET_FIELD(regData,
                             PMIC_LDOX_CTRL_LDOX_SLOW_RAMP_EN,
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
        *pLdoSlowRampEn = HW_REG_GET_FIELD(regData,
                                           PMIC_LDOX_CTRL_LDOX_SLOW_RAMP_EN);
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
            HW_REG_SET_FIELD(regData,
                             PMIC_POWER_RESOURCEX_CTRL_POWER_RESOURCEX_EN,
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
        *pRegulatorEn = HW_REG_GET_FIELD(
                                  regData,
                                  PMIC_POWER_RESOURCEX_CTRL_POWER_RESOURCEX_EN);
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
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t pwrRsrcType;
    uint16_t ldoMinVoltageValue;
    bool    vmonRange;

    pwrRsrcType  = Pmic_powerGetPwrRsrcType(pwrRsrc);

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

            Pmic_setBitField(&regData, bitPos, bitMask, vSetVal);

       }

        if(PMIC_ST_SUCCESS == status)
        {
            status = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                            regAddr,
                                            regData);
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
    uint8_t  bitPos = 0U;
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

        vSetVal = Pmic_getBitField(regData, bitPos, bitMask);
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
            HW_REG_SET_FIELD(regData, PMIC_BUCKX_CONF_BUCKX_ILIM, currentLimit);

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
static int32_t Pmic_powerGetBuckCurrentLimit(
                                            Pmic_CoreHandle_t *pPmicCoreHandle,
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
        *pCurrentLimit = HW_REG_GET_FIELD(regData, PMIC_BUCKX_CONF_BUCKX_ILIM);
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

            Pmic_setBitField(&regData, bitPos, bitMask, buckVmonSlewRate);
        }

        if(PMIC_ST_SUCCESS == status)
        {
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
static int32_t Pmic_powerGetBuckVmonSlewRate(
                                            Pmic_CoreHandle_t *pPmicCoreHandle,
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

    status = Pmic_powerGetBuckVmonSlewRateBitPos(
                                                pPmicCoreHandle->pmicDeviceType,
                                                pwrRsrc,
                                                &bitPos,
                                                &bitMask);
    *pSlewRate = Pmic_getBitField(regData, bitPos, bitMask);
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
            HW_REG_SET_FIELD(regData,
                             PMIC_BUCKX_CTRL_BUCKX_PLDN,
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
        *pBuckPullDownEn = HW_REG_GET_FIELD(regData,
                                            PMIC_BUCKX_CTRL_BUCKX_PLDN);
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
            HW_REG_SET_FIELD(regData,
                             PMIC_LDOX_CTRL_LDOX_PLDN,
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
        *pLdoPullDownSel = HW_REG_GET_FIELD(regData, PMIC_LDOX_CTRL_LDOX_PLDN);
    }

    return pmicStatus;
}

/*!
 * \brief   This function is used to set Deglitch time for BUCKx_VMON/
 *          LDOx_VMON/VCCA_VMON
 */
static int32_t Pmic_powerSetPwrRsrcVmonDeglitchTimeSel(
                                            Pmic_CoreHandle_t *pPmicCoreHandle,
                                            bool               deglitchTimeSel)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData    = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);

    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        PMIC_VCCA_VMON_CTRL_REGADDR,
                                        &regData);

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        /* Set Deglitch time */
        HW_REG_SET_FIELD(regData,
                         PMIC_VCCA_VMON_CTRL_VMON_DEGLITCH_SEL,
                         deglitchTimeSel);
        pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                            PMIC_VCCA_VMON_CTRL_REGADDR,
                                            regData);
    }

    Pmic_criticalSectionStop(pPmicCoreHandle);
    return pmicStatus;
}

/*!
 * \brief   Get Deglitch time for BUCKx_VMON/LDOx_VMON/VCCA_VMON/VMONX
 */
static int32_t Pmic_powerGetPwrRsrcVmonDeglitchTimeSel(
                                            Pmic_CoreHandle_t *pPmicCoreHandle,
                                            bool              *pDeglitchTimeSel)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData    = 0U;

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        Pmic_criticalSectionStart(pPmicCoreHandle);

        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                            PMIC_VCCA_VMON_CTRL_REGADDR,
                                            &regData);
        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        /* Get Deglitch time */
        *pDeglitchTimeSel = HW_REG_GET_FIELD(
                                    regData,
                                    PMIC_VCCA_VMON_CTRL_VMON_DEGLITCH_SEL);
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
        HW_REG_SET_FIELD(regData,
                         PMIC_VCCA_PG_WINDOW_VCCA_PG_SET,
                         vccaPwrGudLvl);

        pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                            PMIC_VCCA_PG_WINDOW_REGADDR,
                                            regData);
    }

    Pmic_criticalSectionStop(pPmicCoreHandle);
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
        *pVccaPwrGudLvl = HW_REG_GET_FIELD(regData,
                                           PMIC_VCCA_PG_WINDOW_VCCA_PG_SET);

    }

    return pmicStatus;
}

/*!
 * \brief   This function is used to get the required Bit Position of VMON
 *          enable/disable status for all power resources
 */
static int32_t Pmic_powerGetVmonEnBitPos(uint8_t   pmicDeviceType,
                                         uint16_t  pwrRsrc,
                                         uint8_t   *pBitPos)
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
            }
            else if(PMIC_TPS6594X_POWER_RESOURCE_TYPE_VCCA == pwrRsrcType)
            {
               *pBitPos = PMIC_VCCA_VMON_CTRL_VMON1_EN_SHIFT;
            }
            else
            {
                status = PMIC_ST_ERR_INV_PARAM;
            }

            break;
        case PMIC_DEV_HERA_LP8764X:
            if(PMIC_LP8764X_POWER_RESOURCE_TYPE_BUCK == pwrRsrcType)
            {
               *pBitPos = PMIC_VCCA_CTRL_VCCA_VMON_EN_SHIFT;
            }
            else if(PMIC_LP8764X_POWER_RESOURCE_TYPE_VMON == pwrRsrcType)
            {
                if(PMIC_LP8764X_POWER_SOURCE_VMON1 == pwrRsrc)
                {
                    *pBitPos = PMIC_VCCA_VMON_CTRL_VMON1_EN_SHIFT;
                }
                else if(PMIC_LP8764X_POWER_SOURCE_VMON2 == pwrRsrc)
                {
                    *pBitPos = PMIC_VCCA_VMON_CTRL_VMON2_EN_SHIFT;
                }
                else
                {
                    status = PMIC_ST_ERR_INV_PARAM;
                }
            }
            else if(PMIC_LP8764X_POWER_RESOURCE_TYPE_VCCA == pwrRsrcType)
            {
               *pBitPos = PMIC_VCCA_VMON_CTRL_VMON1_EN_SHIFT;
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
                                                &bitPos);

            BIT_POS_SET_VAL(regData, bitPos, vmonEn);
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
                                               &bitPos);

        *pVmonEn = BIT_POS_GET_VAL(regData, bitPos);
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
    uint8_t regAddr;
    uint8_t regData    = 0U;
    uint8_t pwrRsrcIndex;
    uint8_t bitMask;
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
    uint8_t regData    = 0U;
    uint8_t pwrRsrcIndex;
    uint8_t bitMask;
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

        *pRailGrpSel =  Pmic_getBitField(
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
                                   pPwrRsrcRegCfg[pwrRsrcIndex].pgWindowRegAddr,
                                   &regData);

        if(PMIC_ST_SUCCESS == pmicStatus)
        {
            HW_REG_SET_FIELD(regData,
                             PMIC_VMON_PG_WINDOW_VMON_RANGE,
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

    if(pmic_validParamCheck(pwrRsrcCfg.validParams,
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
       (pmic_validParamCheck(pwrRsrcCfg.validParams,
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
       (pmic_validParamCheck(pwrRsrcCfg.validParams,
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
       (pmic_validParamCheck(pwrRsrcCfg.validParams,
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
       (pmic_validParamCheck(pwrRsrcCfg.validParams,
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
       (pmic_validParamCheck(pwrRsrcCfg.validParams,
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
       (pmic_validParamCheck(pwrRsrcCfg.validParams,
                             PMIC_CFG_DEGLITCH_TIME_SEL_VALID)))
    {
        if(PMIC_ST_SUCCESS == pmicStatus)
        {
            /* Set Deglitch time select for BUCKx_VMON/LDOx_VMON/VCCA_VMON
               /VMONX */
            pmicStatus = Pmic_powerSetPwrRsrcVmonDeglitchTimeSel(
                                             pPmicCoreHandle,
                                             pwrRsrcCfg.deglitchTimeSel);
        }
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (pmic_validParamCheck(pwrRsrcCfg.validParams,
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

    if(pmic_validParamCheck(pwrRsrcCfg.validParams,
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
       (pmic_validParamCheck(pwrRsrcCfg.validParams,
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
       (pmic_validParamCheck(pwrRsrcCfg.validParams,
                             PMIC_CFG_VCCA_PWR_GOOD_LVL_VALID)))
    {
        pmicStatus = Pmic_powerValidatecfgDataPwrRsrctype(
                                          PMIC_CFG_VCCA_PWR_GOOD_LVL_VALID,
                                          pwrRsrc,
                                          pPmicCoreHandle->pmicDeviceType);
        if(PMIC_ST_SUCCESS == pmicStatus)
        {
            /* Set Powergood level for VCCA pin */
            pmicStatus = Pmic_powerSetVccaPwrGudLvl(
                                                pPmicCoreHandle,
                                                pwrRsrcCfg.vccaPwrGudLvl);
        }
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (pmic_validParamCheck(pwrRsrcCfg.validParams,
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

    if(pmic_validParamCheck(pPwrRsrcCfg->validParams,
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
       (pmic_validParamCheck(pPwrRsrcCfg->validParams,
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
       (pmic_validParamCheck(pPwrRsrcCfg->validParams,
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
       (pmic_validParamCheck(pPwrRsrcCfg->validParams,
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
       (pmic_validParamCheck(pPwrRsrcCfg->validParams,
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
       (pmic_validParamCheck(pPwrRsrcCfg->validParams,
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
       (pmic_validParamCheck(pPwrRsrcCfg->validParams,
                             PMIC_CFG_REGULATOR_EN_VALID)))
    {
        pmicStatus = Pmic_powerValidatecfgDataPwrRsrctype(
                                               PMIC_CFG_REGULATOR_EN_VALID,
                                               pwrRsrc,
                                               pPmicCoreHandle->pmicDeviceType);
        if(PMIC_ST_SUCCESS == pmicStatus)
        {
            /* Get Enable/Disable status for Regulator */
            pmicStatus = Pmic_powerGetRegulatorEn(
                                               pPmicCoreHandle,
                                               pwrRsrc,
                                               &(pPwrRsrcCfg->regulatorEn));
        }
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (pmic_validParamCheck(pPwrRsrcCfg->validParams,
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
       (pmic_validParamCheck(pPwrRsrcCfg->validParams,
                             PMIC_CFG_DEGLITCH_TIME_SEL_VALID)))
    {
        if(PMIC_ST_SUCCESS == pmicStatus)
        {
             /* get Deglitch time select for BUCKx_VMON, LDOx_VMON and
               VCCA_VMON */
            pmicStatus = Pmic_powerGetPwrRsrcVmonDeglitchTimeSel(
                                           pPmicCoreHandle,
                                           &(pPwrRsrcCfg->deglitchTimeSel));
        }
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (pmic_validParamCheck(pPwrRsrcCfg->validParams,
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

    if(pmic_validParamCheck(pPwrRsrcCfg->validParams,
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
       (pmic_validParamCheck(pPwrRsrcCfg->validParams,
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
                                  Pmic_CoreHandle_t           *pPmicCoreHandle,
                                  uint16_t                     pwrRsrc,
                                  Pmic_PowerResourceCfg_t *pPwrRsrcCfg)

{
    int32_t  pmicStatus = PMIC_ST_SUCCESS;

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (pmic_validParamCheck(pPwrRsrcCfg->validParams,
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
       (pmic_validParamCheck(pPwrRsrcCfg->validParams,
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

    return pmicStatus;
}

/*!
 * \brief   This function is to validate the power resource limit for the
 *          specific PMIC device.
 */
static int32_t Pmic_powerValidatePwrRsrcLimit(
                                             Pmic_CoreHandle_t *pPmicCoreHandle,
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
static int32_t Pmic_powerParamCheck(Pmic_CoreHandle_t *pPmicCoreHandle,
                                    uint16_t           pwrResource)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t pwrRsrcType;

    if(NULL == pPmicCoreHandle)
    {
        status = PMIC_ST_ERR_INV_HANDLE;
    }

    pwrRsrcType  = Pmic_powerGetPwrRsrcType(pwrResource);

    if(PMIC_ST_SUCCESS == status)
    {
        status = Pmic_powerValidatePwrRsrcLimit(pPmicCoreHandle,
                                                pwrRsrcType,
                                                pwrResource);
    }

    return status;
}


/*!
 * \brief   Thsi function is used to get the switching frequency for BUCK
 */
static int32_t Pmic_powerGetBuckFreqSel(Pmic_CoreHandle_t *pPmicCoreHandle,
                                        uint16_t           pwrRsrc,
                                        uint8_t           *pBuckFreqSel)
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

        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                            PMIC_FREQ_SEL_REG_REGADDR,
                                            &regData);
        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        *pBuckFreqSel = BIT_POS_GET_VAL(
                              regData,
                              pPwrRsrcRegCfg[pwrRsrcIndex].buckFreqBitShiftVal);
    }

    return pmicStatus;
}


 /*!
 * \brief   This function is used to validate the buck frequency limit.
 */
static int32_t Pmic_powerValidateBuckFrequencyLimit(uint8_t  pmicDeviceType,
                                                    uint16_t pwrRsrc,
                                                    uint8_t  buckFreqSel)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    switch(pmicDeviceType)
    {
        case PMIC_DEV_LEO_TPS6594X:
            if(buckFreqSel > PMIC_TPS6594X_BUCK_FREQ_SEL_8M8)
            {
                pmicStatus = PMIC_ST_ERR_INV_PARAM;
            }
            break;
        case PMIC_DEV_HERA_LP8764X:
            if(buckFreqSel > PMIC_LP8764X_BUCK_FREQ_SEL_4M4)
            {
                pmicStatus = PMIC_ST_ERR_INV_PARAM;
            }
            break;
        default:
            pmicStatus = PMIC_ST_ERR_INV_PARAM;
    }

    return pmicStatus;
}

/*!
 * \brief   Thsi function is used to set the switching frequency for BUCK
 */
static int32_t Pmic_powerSetBuckFreqSel(Pmic_CoreHandle_t *pPmicCoreHandle,
                                        uint16_t           pwrRsrc,
                                        uint8_t            buckFreqSel)
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
        pmicStatus = Pmic_powerValidateBuckFrequencyLimit(
                                                pPmicCoreHandle->pmicDeviceType,
                                                pwrRsrc,
                                                buckFreqSel);
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        Pmic_criticalSectionStart(pPmicCoreHandle);

        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                            PMIC_FREQ_SEL_REG_REGADDR,
                                            &regData);

        if(PMIC_ST_SUCCESS == pmicStatus)
        {
            BIT_POS_SET_VAL(regData,
                            pPwrRsrcRegCfg[pwrRsrcIndex].buckFreqBitShiftVal,
                            buckFreqSel);
        }

        if(PMIC_ST_SUCCESS == pmicStatus)
        {
            pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                                PMIC_FREQ_SEL_REG_REGADDR,
                                                regData);
        }

        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    return pmicStatus;
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
                                      pPwrRsrcRegCfg[pwrRsrcIndex].vout1RegAddr,
                                     &regData);
        if(PMIC_ST_SUCCESS == pmicStatus)
        {
            HW_REG_SET_FIELD(regData,
                             PMIC_LDO1_2_3_VOUT_LDO1_2_3_BYPASS,
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
                                      pPwrRsrcRegCfg[pwrRsrcIndex].vout1RegAddr,
                                      &regData);
        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        *pLdoBypassModeEn = HW_REG_GET_FIELD(
                                            regData,
                                            PMIC_LDO1_2_3_VOUT_LDO1_2_3_BYPASS);
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
    uint8_t regData    = 0U;
    uint8_t regAddr;
    uint8_t pwrRsrcIndex;
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
        Pmic_criticalSectionStart(pPmicCoreHandle);
        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                            regAddr,
                                            &regData);
        BIT_POS_SET_VAL(regData,
                        pPwrRsrcRegCfg[pwrRsrcIndex].ldoRvTimeOutBitShiftVal,
                        ldoRvTimeoutSel);

        pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                            regAddr,
                                            regData);
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
    uint8_t regData    = 0U;
    uint8_t regAddr;
    uint8_t pwrRsrcIndex;
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
        *pLdoResVoltTimeout = BIT_POS_GET_VAL(
                          regData,
                          pPwrRsrcRegCfg[pwrRsrcIndex].ldoRvTimeOutBitShiftVal);
    }

    return pmicStatus;
}

/**
 * \brief   API to set power resources configurations.
 *          This function can be used to configure the various control and
 *          configuration parameters for BUCK/LDO/VCCA/VMON power resources  and
 *          also used to set the various control and configuration of
 *          voltage monitor parameters for BUCK/LDO/VCCA/VMON power resources
 *
 *          To set control and configuration params for BUCK, the application
 *          has to configure the below defined structure members of
 *          Pmic_PowerResourceCfg_t:
 *          rvCheckEn, buckPullDownEn, vmonEn, buckVoutRegSel, buckFpwmMode,
 *          buckFpwmMpMode, buckFreqSel, regulatorEn, buckCurrentLimit,
 *          buckVmonSlewRate, voltage_mV, deglitchTimeSel, pgUvThresholdLvl,
 *          pgOvThresholdLvl, railGrpSel
 *
 *          To set control and configuration params for LDO, the application
 *          has to configure the below defined structure members of
 *          Pmic_PowerResourceCfg_t:
 *          rvCheckEn, vmonEn, regulatorEn, ldoPullDownSel, ldoSlowRampEn,
 *          ldoBypassModeEn, ldoRvTimeoutSel, voltage_mV, deglitchTimeSel,
 *          pgUvThresholdLvl, pgOvThresholdLvl, railGrpSel
 *
 *          To set control and configuration params for VCCA, the application
 *          has to configure the below defined structure members of
 *          Pmic_PowerResourceCfg_t:
 *          vmonEn, deglitchTimeSel, vccaPwrGudLvl, pgUvThresholdLvl,
 *          pgOvThresholdLvl, railGrpSel
 *
 *          To set control and configuration params for VMON, the application
 *          has to configure the below defined structure members of
 *          Pmic_PowerResourceCfg_t:
 *          rvCheckEn, vmonEn, deglitchTimeSel, vccaPwrGudLvl, vmonRange,
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
        pmicStatus =  Pmic_powerSetBuckVmonConfig(pPmicCoreHandle,
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
       (pmic_validParamCheck(pwrResourceCfg.validParams,
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
       (pmic_validParamCheck(pwrResourceCfg.validParams,
                             PMIC_CFG_REGULATOR_BUCK_FREQ_SEL_VALID)))
    {
        pmicStatus = Pmic_powerValidatecfgDataPwrRsrctype(
                                         PMIC_CFG_REGULATOR_BUCK_FREQ_SEL_VALID,
                                         pwrResource,
                                         pPmicCoreHandle->pmicDeviceType);
        if(PMIC_ST_SUCCESS == pmicStatus)
        {
            /* Set Switching frequency for buck regulator */
            pmicStatus = Pmic_powerSetBuckFreqSel(pPmicCoreHandle,
                                                  pwrResource,
                                                  pwrResourceCfg.buckFreqSel);
        }
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (pmic_validParamCheck(pwrResourceCfg.validParams,
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
       (pmic_validParamCheck(pwrResourceCfg.validParams,
                             PMIC_CFG_PWR_RESOURCE_RAIL_GRP_SEL_VALID)))
    {
        /* Rail selection for power resources */
        pmicStatus = Pmic_powerSetRailGrpSel(pPmicCoreHandle,
                                             pwrResource,
                                             pwrResourceCfg.railGrpSel);
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (pmic_validParamCheck(pwrResourceCfg.validParams,
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
       (pmic_validParamCheck(pwrResourceCfg.validParams,
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
       (pmic_validParamCheck(pwrResourceCfg.validParams,
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
 *          This function can be used to get the various control and
 *          configuration parameters for BUCK/LDO/VCCA/VMON power resources and
 *          also used to get the various control and configuration of
 *          voltage monitor parameters for BUCK/LDO/VCCA/VMON power resources
 *
 *          Application can get these control and configuration params for BUCK,
 *          which is stored in the below defined structure members of
 *          Pmic_PowerResourceCfg_t:
 *          rvCheckEn, buckPullDownEn, vmonEn, buckVoutRegSel, buckFpwmMode,
 *          buckFpwmMpMode, buckFreqSel, regulatorEn, buckCurrentLimit,
 *          buckVmonSlewRate, voltage_mV, deglitchTimeSel, pgUvThresholdLvl,
 *          pgOvThresholdLvl, railGrpSel
 *
 *          Application can get these control and configuration params for LDO,
 *          which is stored in the below defined structure members of
 *          Pmic_PowerResourceCfg_t:
 *          rvCheckEn, vmonEn, regulatorEn, ldoPullDownSel, ldoSlowRampEn,
 *          ldoBypassModeEn, ldoRvTimeoutSel, voltage_mV, deglitchTimeSel,
 *          pgUvThresholdLvl, pgOvThresholdLvl, railGrpSel
 *
 *          Application can get these control and configuration params for VCCA,
 *          which is stored in the below defined structure members of
 *          Pmic_PowerResourceCfg_t:
 *          vmonEn, deglitchTimeSel, vccaPwrGudLvl, pgUvThresholdLvl,
 *          pgOvThresholdLvl, railGrpSel
 *
 *          Application can get these control and configuration params for VMON,
 *          which is stored in the below defined structure members of
 *          Pmic_PowerResourceCfg_t:
 *          rvCheckEn, vmonEn, deglitchTimeSel, vccaPwrGudLvl, vmonRange,
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
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        pmicStatus =  Pmic_powerGetPwrResourceCtrlCfg(pPmicCoreHandle,
                                                      pwrResource,
                                                      pPwrResourceCfg);
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        pmicStatus =  Pmic_powerGetBuckVmonConfig(pPmicCoreHandle,
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
       (pmic_validParamCheck(pPwrResourceCfg->validParams,
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
       (pmic_validParamCheck(pPwrResourceCfg->validParams,
                             PMIC_CFG_REGULATOR_BUCK_FREQ_SEL_VALID)))
    {
        pmicStatus = Pmic_powerValidatecfgDataPwrRsrctype(
                                          PMIC_CFG_REGULATOR_BUCK_FREQ_SEL_VALID,
                                          pwrResource,
                                          pPmicCoreHandle->pmicDeviceType);
        if(PMIC_ST_SUCCESS == pmicStatus)
        {
            /* Get Switching frequency for buck regulator */
            pmicStatus = Pmic_powerGetBuckFreqSel(
                                              pPmicCoreHandle,
                                              pwrResource,
                                              &(pPwrResourceCfg->buckFreqSel));
        }
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (pmic_validParamCheck(pPwrResourceCfg->validParams,
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
       (pmic_validParamCheck(pPwrResourceCfg->validParams,
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
       (pmic_validParamCheck(pPwrResourceCfg->validParams,
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
