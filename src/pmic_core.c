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
 *  \file  pmic_core.c
 *
 *  \brief This file contains PMIC generic driver APIs
 */

#include "pmic_core_priv.h"
#include "pmic_core_tps6594x.h"
#include "pmic_core_lp8764x.h"

#include "pmic_rtc_tps6594x_priv.h"
#include "pmic_fsm_priv.h"
#include "pmic_irq_tps6594x_priv.h"

#include "pmic_power_priv.h"

// clang-format off
static const Pmic_DevSubSysInfo_t pmicSubSysInfo[] = {
  /* PMIC_DEV_LEO_TPS6594x */
    {.gpioEnable = (bool)true,
     .rtcEnable = (bool)true,
     .wdgEnable = (bool)true,
     .buckEnable = (bool)true,
     .ldoEnable = (bool)true,
     .esmEnable = (bool)true},
 /* PMIC_DEV_HERA_LP8764x */
    {.gpioEnable = (bool)true,
     .wdgEnable = (bool)true,
     .buckEnable = (bool)true,
     .esmEnable = (bool)true},
 /* PMIC_DEV_BURTON_TPS6522x */
    {.gpioEnable = (bool)true,
     .wdgEnable = (bool)true,
     .buckEnable = (bool)true,
     .ldoEnable = (bool)true,
     .esmEnable = (bool)true,
     .adcEnable = (bool)true }
};
// clang-format on

bool pmic_validParamCheck(uint32_t validParamVal, uint8_t bitPos)
{
    bool retVal = (bool)false;

    if (((validParamVal >> bitPos) & 0x01U) != 0U)
    {
        retVal = (bool)true;
    }

    return retVal;
}

void Pmic_criticalSectionStart(const Pmic_CoreHandle_t *pPmicCoreHandle)
{
    if (NULL != pPmicCoreHandle->pFnPmicCritSecStart)
    {
        pPmicCoreHandle->pFnPmicCritSecStart();
    }
}

void Pmic_criticalSectionStop(const Pmic_CoreHandle_t *pPmicCoreHandle)
{
    if (NULL != pPmicCoreHandle->pFnPmicCritSecStop)
    {
        pPmicCoreHandle->pFnPmicCritSecStop();
    }
}

int32_t Pmic_setRecoveryCntCfg(Pmic_CoreHandle_t *pPmicCoreHandle, const Pmic_RecovCntCfg_t recovCntCfg)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regVal;
    uint8_t clrCntVal = 1U;

    if (NULL == pPmicCoreHandle)
    {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) &&
        (((bool)true) == pmic_validParamCheck(recovCntCfg.validParams, PMIC_CFG_RECOV_CNT_THR_VAL_VALID)) &&
        ((recovCntCfg.thrVal > PMIC_RECOV_CNT_THR_MAX)))
    {
        pmicStatus = PMIC_ST_ERR_INV_PARAM;
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) &&
        (((bool)true) == pmic_validParamCheck(recovCntCfg.validParams, PMIC_CFG_RECOV_CNT_CLR_CNT_VALID)) &&
        (recovCntCfg.clrCnt != ((bool)true)))
    {
        pmicStatus = PMIC_ST_ERR_INV_PARAM;
    }

    if (PMIC_ST_SUCCESS == pmicStatus)
    {
        /* Start Critical Section */
        Pmic_criticalSectionStart(pPmicCoreHandle);

        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_RECOV_CNT_REG_2_REGADDR, &regVal);

        if (PMIC_ST_SUCCESS == pmicStatus)
        {
            if (((bool)true) == pmic_validParamCheck(recovCntCfg.validParams, PMIC_CFG_RECOV_CNT_CLR_CNT_VALID))
            {
                Pmic_setBitField(&regVal,
                                 PMIC_RECOV_CNT_REG_2_RECOV_CNT_CLR_SHIFT,
                                 PMIC_RECOV_CNT_REG_2_RECOV_CNT_CLR_MASK,
                                 clrCntVal);
            }

            if (((bool)true) == pmic_validParamCheck(recovCntCfg.validParams, PMIC_CFG_RECOV_CNT_THR_VAL_VALID))
            {
                Pmic_setBitField(&regVal,
                                 PMIC_RECOV_CNT_REG_2_RECOV_CNT_CLR_SHIFT,
                                 PMIC_RECOV_CNT_REG_2_RECOV_CNT_CLR_MASK,
                                 recovCntCfg.thrVal);
            }

            pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle, PMIC_RECOV_CNT_REG_2_REGADDR, regVal);
        }

        /* Stop Critical Section */
        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    return pmicStatus;
}

int32_t Pmic_getRecoveryCntCfg(Pmic_CoreHandle_t *pPmicCoreHandle, Pmic_RecovCntCfg_t *pRecovCntCfg)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regVal = 0U;

    if (NULL == pPmicCoreHandle)
    {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) && (NULL == pRecovCntCfg))
    {
        pmicStatus = PMIC_ST_ERR_NULL_PARAM;
    }

    if (PMIC_ST_SUCCESS == pmicStatus)
    {

        /* Start Critical Section */
        Pmic_criticalSectionStart(pPmicCoreHandle);

        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_RECOV_CNT_REG_2_REGADDR, &regVal);
        /* Stop Critical Section */
        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    if (PMIC_ST_SUCCESS == pmicStatus)
    {
        if (((bool)true) == pmic_validParamCheck(pRecovCntCfg->validParams, PMIC_CFG_RECOV_CNT_THR_VAL_VALID))
        {
            pRecovCntCfg->thrVal = Pmic_getBitField(
                regVal, PMIC_RECOV_CNT_REG_2_RECOV_CNT_THR_SHIFT, PMIC_RECOV_CNT_REG_2_RECOV_CNT_THR_MASK);
        }
        if (((bool)true) == pmic_validParamCheck(pRecovCntCfg->validParams, PMIC_CFG_RECOV_CNT_CLR_CNT_VALID))
        {
            pRecovCntCfg->clrCnt = ((bool)true);

            if ((Pmic_getBitField(
                    regVal, PMIC_RECOV_CNT_REG_2_RECOV_CNT_CLR_SHIFT, PMIC_RECOV_CNT_REG_2_RECOV_CNT_CLR_MASK)) == 0U)
            {
                pRecovCntCfg->clrCnt = ((bool)false);
            }
        }
    }

    return pmicStatus;
}

int32_t Pmic_getRecoveryCnt(Pmic_CoreHandle_t *pPmicCoreHandle, uint8_t *pRecovCntVal)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regVal = 0U;

    if (NULL == pPmicCoreHandle)
    {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) && (NULL == pRecovCntVal))
    {
        pmicStatus = PMIC_ST_ERR_NULL_PARAM;
    }

    if (PMIC_ST_SUCCESS == pmicStatus)
    {
        /* Start Critical Section */
        Pmic_criticalSectionStart(pPmicCoreHandle);

        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_RECOV_CNT_REG_1_REGADDR, &regVal);

        /* Stop Critical Section */
        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    if (PMIC_ST_SUCCESS == pmicStatus)
    {
        *pRecovCntVal =
            Pmic_getBitField(regVal, PMIC_RECOV_CNT_REG_1_RECOV_CNT_SHIFT, PMIC_RECOV_CNT_REG_1_RECOV_CNT_MASK);
    }

    return pmicStatus;
}

/*!
 * \brief  API to Initialize pPmicCoreHandle for pmicDeviceType, Comm Mode,
 *         Main Slave Address, and NVM Slave Address
 *
 *         This function gets device configuration from pPmicConfigData and
 *         initializes pPmicCoreHandle after validation of given params depends
 *         on validParams bit fields
 */
static int32_t Pmic_initCoreHandleBasicDevCfgParams(const Pmic_CoreCfg_t *pPmicConfigData,
                                                    Pmic_CoreHandle_t    *pPmicCoreHandle)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    /* Check and update PMIC Handle device type */
    if (((bool)true) == pmic_validParamCheck(pPmicConfigData->validParams, PMIC_CFG_DEVICE_TYPE_VALID))
    {
        if ((PMIC_DEV_HERA_LP8764X != pPmicConfigData->pmicDeviceType) &&
            (PMIC_DEV_LEO_TPS6594X != pPmicConfigData->pmicDeviceType) &&
            (PMIC_DEV_BURTON_TPS6522X != pPmicConfigData->pmicDeviceType))
        {
            pmicStatus = PMIC_ST_ERR_INV_PARAM;
        }

        if (PMIC_ST_SUCCESS == pmicStatus)
        {
            pPmicCoreHandle->pmicDeviceType = pPmicConfigData->pmicDeviceType;
        }
    }

    /* Check and update PMIC Handle Comm Mode */
    if ((PMIC_ST_SUCCESS == pmicStatus) &&
        (((bool)true) == pmic_validParamCheck(pPmicConfigData->validParams, PMIC_CFG_COMM_MODE_VALID)))
    {
        if ((PMIC_INTF_SINGLE_I2C != pPmicConfigData->commMode) && (PMIC_INTF_DUAL_I2C != pPmicConfigData->commMode) &&
            (PMIC_INTF_SPI != pPmicConfigData->commMode))
        {
            pmicStatus = PMIC_ST_ERR_INV_PARAM;
        }
        else
        {
            pPmicCoreHandle->commMode = pPmicConfigData->commMode;
        }
    }

    /* Check and update PMIC Handle Main Slave Address */
    if ((PMIC_ST_SUCCESS == pmicStatus) &&
        (((bool)true) == pmic_validParamCheck(pPmicConfigData->validParams, PMIC_CFG_SLAVEADDR_VALID)))
    {
        if ((PMIC_INTF_SINGLE_I2C == pPmicCoreHandle->commMode) || (PMIC_INTF_DUAL_I2C == pPmicCoreHandle->commMode))
        {
            pPmicCoreHandle->slaveAddr = pPmicConfigData->slaveAddr;
        }
    }

    /* Check and update PMIC Handle NVM Slave Address */
    if ((PMIC_ST_SUCCESS == pmicStatus) &&
        (((bool)true) == pmic_validParamCheck(pPmicConfigData->validParams, PMIC_CFG_NVMSLAVEADDR_VALID)))
    {
        if ((PMIC_INTF_SINGLE_I2C == pPmicCoreHandle->commMode) || (PMIC_INTF_DUAL_I2C == pPmicCoreHandle->commMode))
        {
            pPmicCoreHandle->nvmSlaveAddr = pPmicConfigData->nvmSlaveAddr;
        }
    }

    return pmicStatus;
}

/*!
 * \brief  API to Initialize pPmicCoreHandle for I2C1 Speed, I2C2 Speed and
 *         Main Comm Handle
 *
 *         This function gets device configuration from pPmicConfigData and
 *         initializes pPmicCoreHandle after validation of given params depends
 *         on validParams bit fields
 */
static int32_t Pmic_initCoreHandleI2CSpeedCommHandle(const Pmic_CoreCfg_t *pPmicConfigData,
                                                     Pmic_CoreHandle_t    *pPmicCoreHandle)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    /* Check and update PMIC Handle I2C1 Speed */
    if (((bool)true) == pmic_validParamCheck(pPmicConfigData->validParams, PMIC_CFG_I2C1_SPEED_VALID))
    {

        if (pPmicConfigData->i2c1Speed > PMIC_I2C_FORCED_HS_MODE)
        {
            pmicStatus = PMIC_ST_ERR_INV_PARAM;
        }
        else
        {
            if ((PMIC_INTF_SINGLE_I2C == pPmicCoreHandle->commMode) ||
                (PMIC_INTF_DUAL_I2C == pPmicCoreHandle->commMode))
            {
                pPmicCoreHandle->i2c1Speed = pPmicConfigData->i2c1Speed;
            }
        }
    }

    /* Check and update PMIC Handle I2C2 Speed */
    if ((PMIC_ST_SUCCESS == pmicStatus) &&
        (((bool)true) == pmic_validParamCheck(pPmicConfigData->validParams, PMIC_CFG_I2C2_SPEED_VALID)))
    {
        if (pPmicConfigData->i2c2Speed > PMIC_I2C_FORCED_HS_MODE)
        {
            pmicStatus = PMIC_ST_ERR_INV_PARAM;
        }
        else
        {
            if (PMIC_INTF_DUAL_I2C == pPmicCoreHandle->commMode)
            {
                pPmicCoreHandle->i2c2Speed = pPmicConfigData->i2c2Speed;
            }
        }
    }

    /* Check and update PMIC Handle Main Comm Handle */
    if ((PMIC_ST_SUCCESS == pmicStatus) &&
        (((bool)true) == pmic_validParamCheck(pPmicConfigData->validParams, PMIC_CFG_COMM_HANDLE_VALID)))
    {
        if (NULL == pPmicConfigData->pCommHandle)
        {
            pmicStatus = PMIC_ST_ERR_NULL_FPTR;
        }
        else
        {
            pPmicCoreHandle->pCommHandle = pPmicConfigData->pCommHandle;
        }
    }

    return pmicStatus;
}

/*!
 * \brief  API to Initialize pPmicCoreHandle for QA Slave Address and
 *         QA Comm Handle
 *
 *         This function gets device configuration from pPmicConfigData and
 *         initializes pPmicCoreHandle after validation of given params depends
 *         on validParams bit fields
 */
static int32_t Pmic_initCoreHandleQADevCfgParams(const Pmic_CoreCfg_t *pPmicConfigData,
                                                 Pmic_CoreHandle_t    *pPmicCoreHandle)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    /* Check and update PMIC Handle QA Slave Address */
    if (((bool)true) == pmic_validParamCheck(pPmicConfigData->validParams, PMIC_CFG_QASLAVEADDR_VALID))
    {
        if ((PMIC_INTF_SINGLE_I2C == pPmicCoreHandle->commMode) || (PMIC_INTF_DUAL_I2C == pPmicCoreHandle->commMode))
        {
            pPmicCoreHandle->qaSlaveAddr = pPmicConfigData->qaSlaveAddr;
        }
    }

    /* Check and update PMIC Handle QA Comm Handle */
    if (((bool)true) == pmic_validParamCheck(pPmicConfigData->validParams, PMIC_CFG_QACOMM_HANDLE_VALID))
    {
        if (NULL == pPmicConfigData->pQACommHandle)
        {
            pmicStatus = PMIC_ST_ERR_NULL_FPTR;
        }
        else
        {
            if (PMIC_INTF_DUAL_I2C == pPmicCoreHandle->commMode)
            {
                pPmicCoreHandle->pQACommHandle = pPmicConfigData->pQACommHandle;
            }
        }
    }

    return pmicStatus;
}

/*!
 * \brief  API to Initialize pPmicCoreHandle for Comm IO RD Fn, Comm IO Wr Fn,
 *         Critical Section Start Fn and Critical Section Stop Fn
 *
 *         This function gets device configuration from pPmicConfigData and
 *         initializes pPmicCoreHandle after validation of given params depends
 *         on validParams bit fields
 */
static int32_t Pmic_initCoreHandleCommIOCriticalSectionFns(const Pmic_CoreCfg_t *pPmicConfigData,
                                                           Pmic_CoreHandle_t    *pPmicCoreHandle)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    /* Check and update PMIC Handle Comm IO RD Fn */
    if (((bool)true) == pmic_validParamCheck(pPmicConfigData->validParams, PMIC_CFG_COMM_IO_RD_VALID))
    {
        if (NULL == pPmicConfigData->pFnPmicCommIoRead)
        {
            pmicStatus = PMIC_ST_ERR_NULL_FPTR;
        }
        else
        {
            pPmicCoreHandle->pFnPmicCommIoRead = pPmicConfigData->pFnPmicCommIoRead;
        }
    }

    /* Check and update PMIC Handle Comm IO WR Fn */
    if ((PMIC_ST_SUCCESS == pmicStatus) &&
        (((bool)true) == pmic_validParamCheck(pPmicConfigData->validParams, PMIC_CFG_COMM_IO_WR_VALID)))
    {
        if (NULL == pPmicConfigData->pFnPmicCommIoWrite)
        {
            pmicStatus = PMIC_ST_ERR_NULL_FPTR;
        }
        else
        {
            pPmicCoreHandle->pFnPmicCommIoWrite = pPmicConfigData->pFnPmicCommIoWrite;
        }
    }

    /* Check and update PMIC Handle Critical Section Start Fn */
    if ((PMIC_ST_SUCCESS == pmicStatus) &&
        (((bool)true) == pmic_validParamCheck(pPmicConfigData->validParams, PMIC_CFG_CRITSEC_START_VALID)))
    {
        if (NULL == pPmicConfigData->pFnPmicCritSecStart)
        {
            pmicStatus = PMIC_ST_ERR_NULL_FPTR;
        }
        else
        {
            pPmicCoreHandle->pFnPmicCritSecStart = pPmicConfigData->pFnPmicCritSecStart;
        }
    }

    /* Check and update PMIC Handle Critical Section Stop Fn */
    if ((PMIC_ST_SUCCESS == pmicStatus) &&
        (((bool)true) == pmic_validParamCheck(pPmicConfigData->validParams, PMIC_CFG_CRITSEC_STOP_VALID)))
    {
        if (NULL == pPmicConfigData->pFnPmicCritSecStop)
        {
            pmicStatus = PMIC_ST_ERR_NULL_FPTR;
        }
        else
        {
            pPmicCoreHandle->pFnPmicCritSecStop = pPmicConfigData->pFnPmicCritSecStop;
        }
    }

    return pmicStatus;
}

/*!
 * \brief  API to check if the device requested is the one on the bus
 *
 *         Note: In this API, the default PMIC device is assumed as TPS6594x
 *               LEO PMIC. While adding support for New PMIC device, developer
 *               need to update the API functionality for New PMIC device
 *               accordingly.
 *
 */
static int32_t Pmic_validateDevOnBus(Pmic_CoreHandle_t *pPmicCoreHandle, int32_t *pStatus)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regVal = 0U;

    /* Start Critical Section */
    Pmic_criticalSectionStart(pPmicCoreHandle);

    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_DEV_REV_REGADDR, &regVal);

    Pmic_criticalSectionStop(pPmicCoreHandle);

    if (PMIC_ST_SUCCESS == pmicStatus)
    {
        pPmicCoreHandle->pmicDevRev = Pmic_getBitField(regVal,
                                                       PMIC_DEV_REV_TI_DEVICE_ID_PG_2_0_SILICON_REV_SHIFT,
                                                       PMIC_DEV_REV_TI_DEVICE_ID_PG_2_0_SILICON_REV_MASK);

        /* Validate if the device requested is the one on the bus */
        switch (pPmicCoreHandle->pmicDeviceType)
        {
            case PMIC_DEV_HERA_LP8764X:
                if (PMIC_LP8764X_DEV_REV_ID_PG_2_0 != pPmicCoreHandle->pmicDevRev)
                {
                    *pStatus = PMIC_ST_WARN_INV_DEVICE_ID;
                }
                break;
            case PMIC_DEV_BURTON_TPS6522X:
                break;
            default:
                /* Default case is valid only for TPS6594x LEO PMIC */
                if (PMIC_TPS6594X_DEV_REV_ID_PG_2_0 != pPmicCoreHandle->pmicDevRev)
                {
                    *pStatus = PMIC_ST_WARN_INV_DEVICE_ID;
                }
                break;
        }
    }

    return pmicStatus;
}

/*!
 * \brief  API to update CRC Enable status info to PMIC handle and Check if the
 *        device requested is the one on the bus
 *
 */
static int32_t Pmic_updateCrcEnableStatValidateDevOnBus(Pmic_CoreHandle_t *pPmicCoreHandle, int32_t *pStatus)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t i2c1SpiCrcStat = 0xFF, i2c2CrcStat = 0xFF;

    pPmicCoreHandle->crcEnable = (bool)false;

    pmicStatus = Pmic_getCrcStatus(pPmicCoreHandle, &i2c1SpiCrcStat, &i2c2CrcStat);

    if (PMIC_ST_SUCCESS == pmicStatus)
    {
        if ((PMIC_INTF_SINGLE_I2C == pPmicCoreHandle->commMode) || (PMIC_INTF_SPI == pPmicCoreHandle->commMode))
        {
            if (i2c1SpiCrcStat == 1U)
            {
                pPmicCoreHandle->crcEnable = (bool)true;
            }
        }
        else
        {
            if (i2c2CrcStat == 1U)
            {
                pPmicCoreHandle->crcEnable = (bool)true;
            }

            if (i2c1SpiCrcStat != i2c2CrcStat)
            {
                pmicStatus = PMIC_ST_ERR_CRC_STATUS_FAIL;
            }
        }
    }

    if (PMIC_ST_SUCCESS == pmicStatus)
    {
        pmicStatus = Pmic_validateDevOnBus(pPmicCoreHandle, pStatus);
    }

    return pmicStatus;
}

/*!
 * \brief  API to update PMIC subsystem info to PMIC handle and Check the Main
 *         and QA communication interface if PMIC handle is ready for rw
 */
static int32_t Pmic_updateSubSysInfoValidateMainQaCommIFRdWr(const Pmic_CoreCfg_t *pPmicConfigData,
                                                             Pmic_CoreHandle_t    *pPmicCoreHandle)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regVal = 0U;

    /* Update PMIC subsystem info to PMIC handle */
    pPmicCoreHandle->pPmic_SubSysInfo = (&pmicSubSysInfo[pPmicCoreHandle->pmicDeviceType]);

    /* Check the Main communication interface if PMIC handle is ready for rw */
    if (0x0U == pPmicCoreHandle->drvInitStatus)
    {
        /* Update CRC Enable status info to PMIC handle and Check if the
         * device requested is the one on the bus */
        pmicStatus = Pmic_updateCrcEnableStatValidateDevOnBus(pPmicCoreHandle, &status);

        if (PMIC_ST_SUCCESS == pmicStatus)
        {
            pPmicCoreHandle->drvInitStatus = DRV_INIT_SUCCESS | pPmicConfigData->instType;

            /* Start Critical Section */
            Pmic_criticalSectionStart(pPmicCoreHandle);

            pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_MANUFACTURING_VER_REGADDR, &regVal);
            Pmic_criticalSectionStop(pPmicCoreHandle);

            if (PMIC_ST_SUCCESS == pmicStatus)
            {
                pPmicCoreHandle->pmicDevSiliconRev = Pmic_getBitField(
                    regVal, PMIC_MANUFACTURING_VER_SILICON_REV_SHIFT, PMIC_MANUFACTURING_VER_SILICON_REV_MASK);

                if ((PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType) &&
                    (PMIC_SILICON_REV_ID_PG_1_0 == pPmicCoreHandle->pmicDevSiliconRev))
                {
                    Pmic_tps6594x_reInitInterruptConfig();
                }

                pmicStatus = status;
            }
        }
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) && (NULL != pPmicCoreHandle->pQACommHandle) &&
        (0x0U != pPmicCoreHandle->drvInitStatus))
    {
        /* Start Critical Section */
        Pmic_criticalSectionStart(pPmicCoreHandle);

        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_WDG_LONGWIN_CFG_REGADDR, &regVal);

        Pmic_criticalSectionStop(pPmicCoreHandle);

        if (PMIC_ST_SUCCESS == pmicStatus)
        {
            pPmicCoreHandle->drvInitStatus |= pPmicConfigData->instType;
        }
    }

    return pmicStatus;
}

int32_t Pmic_init(const Pmic_CoreCfg_t *pPmicConfigData, Pmic_CoreHandle_t *pPmicCoreHandle)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    if ((NULL == pPmicCoreHandle) || (NULL == pPmicConfigData))
    {
        pmicStatus = PMIC_ST_ERR_NULL_PARAM;
    }

    /* Check and update PMIC Handle for device type, Comm Mode,
     * Main Slave Address and NVM Slave Address */
    if (PMIC_ST_SUCCESS == pmicStatus)
    {
        pmicStatus = Pmic_initCoreHandleBasicDevCfgParams(pPmicConfigData, pPmicCoreHandle);
    }

    /* Check and update PMIC Handle for I2C1 Speed, I2C2 Speed and
     * Main Comm Handle */
    if (PMIC_ST_SUCCESS == pmicStatus)
    {
        pmicStatus = Pmic_initCoreHandleI2CSpeedCommHandle(pPmicConfigData, pPmicCoreHandle);
    }

    /* Check and update PMIC Handle for QA Slave Address and QA Comm Handle */
    if (PMIC_ST_SUCCESS == pmicStatus)
    {
        pmicStatus = Pmic_initCoreHandleQADevCfgParams(pPmicConfigData, pPmicCoreHandle);
    }

    /* Check and update PMIC Handle for Comm IO RD Fn, Comm IO Wr Fn,
     * Critical Section Start Fn and Critical Section Stop Fn */
    if (PMIC_ST_SUCCESS == pmicStatus)
    {
        pmicStatus = Pmic_initCoreHandleCommIOCriticalSectionFns(pPmicConfigData, pPmicCoreHandle);
    }

    /* Check for required members for I2C/SPI Main handle comm */
    if ((PMIC_ST_SUCCESS == pmicStatus) &&
        ((NULL == pPmicCoreHandle->pCommHandle) || (NULL == pPmicCoreHandle->pFnPmicCritSecStart) ||
         (NULL == pPmicCoreHandle->pFnPmicCritSecStop) || (NULL == pPmicCoreHandle->pFnPmicCommIoRead) ||
         (NULL == pPmicCoreHandle->pFnPmicCommIoWrite)))
    {
        pmicStatus = PMIC_ST_ERR_INSUFFICIENT_CFG;
    }

    /* Update PMIC subsystem info to PMIC handle and Check the Main and QA
     * communication interface if PMIC handle is ready for rw  */
    if (PMIC_ST_SUCCESS == pmicStatus)
    {
        pmicStatus = Pmic_updateSubSysInfoValidateMainQaCommIFRdWr(pPmicConfigData, pPmicCoreHandle);
    }

    return pmicStatus;
}

int32_t Pmic_deinit(Pmic_CoreHandle_t *pPmicCoreHandle)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    if (NULL == pPmicCoreHandle)
    {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    }

    if (PMIC_ST_SUCCESS == pmicStatus)
    {
        pPmicCoreHandle->pCommHandle = NULL;
        pPmicCoreHandle->pQACommHandle = NULL;
        pPmicCoreHandle->pFnPmicCritSecStart = NULL;
        pPmicCoreHandle->pFnPmicCritSecStop = NULL;
        pPmicCoreHandle->pFnPmicCommIoRead = NULL;
        pPmicCoreHandle->pFnPmicCommIoWrite = NULL;
        pPmicCoreHandle->pPmic_SubSysInfo = NULL;
        pPmicCoreHandle->drvInitStatus = 0x00U;
    }

    return pmicStatus;
}

/*!
 * \brief   API to get Scratch pad Register Address
 *          Note: In this API, the default scratchPadRegId is assumed as
 *                PMIC_SCRATCH_PAD_REG_1. While adding support for New PMIC
 *                device, developer need to update the API functionality for
 *                New PMIC device accordingly.
 */
static void Pmic_getScratchPadRegAddr(uint8_t scratchPadRegId, uint8_t *pRegAddr)
{
    switch (scratchPadRegId)
    {
        case PMIC_SCRATCH_PAD_REG_2:
            *pRegAddr = PMIC_SCRATCH_PAD_REG_2_REGADDR;
            break;
        case PMIC_SCRATCH_PAD_REG_3:
            *pRegAddr = PMIC_SCRATCH_PAD_REG_3_REGADDR;
            break;
        case PMIC_SCRATCH_PAD_REG_4:
            *pRegAddr = PMIC_SCRATCH_PAD_REG_4_REGADDR;
            break;
        default:
            *pRegAddr = PMIC_SCRATCH_PAD_REG_1_REGADDR;
            break;
    }
}

int32_t Pmic_setScratchPadValue(Pmic_CoreHandle_t *pPmicCoreHandle, const uint8_t scratchPadRegId, const uint8_t data)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regAddr;

    if (NULL == pPmicCoreHandle)
    {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) && (scratchPadRegId > PMIC_SCRATCH_PAD_REG_4))
    {
        pmicStatus = PMIC_ST_ERR_INV_PARAM;
    }

    if (PMIC_ST_SUCCESS == pmicStatus)
    {
        Pmic_getScratchPadRegAddr(scratchPadRegId, &regAddr);

        Pmic_criticalSectionStart(pPmicCoreHandle);
        pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle, regAddr, data);
        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    return pmicStatus;
}

int32_t Pmic_getScratchPadValue(Pmic_CoreHandle_t *pPmicCoreHandle, const uint8_t scratchPadRegId, uint8_t *pData)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regAddr;

    if (NULL == pPmicCoreHandle)
    {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) && (scratchPadRegId > PMIC_SCRATCH_PAD_REG_4))
    {
        pmicStatus = PMIC_ST_ERR_INV_PARAM;
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) && (NULL == pData))
    {
        pmicStatus = PMIC_ST_ERR_NULL_PARAM;
    }

    if (PMIC_ST_SUCCESS == pmicStatus)
    {
        Pmic_getScratchPadRegAddr(scratchPadRegId, &regAddr);

        Pmic_criticalSectionStart(pPmicCoreHandle);
        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, regAddr, pData);
        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    return pmicStatus;
}

/*!
 * \brief   API to get User Spare Register Bit fields
 *          Note: In this API, the default userSpareRegNum is assumed as
 *                PMIC_USER_SPARE_REG_1. While adding support for New PMIC
 *                device, developer need to update the API functionality for
 *                New PMIC device accordingly.
 */
static void Pmic_getUserSpareRegBitFields(uint8_t userSpareRegNum, uint8_t *pBitShift, uint8_t *pBitMask)
{
    switch (userSpareRegNum)
    {
        case PMIC_USER_SPARE_REG_2:
            *pBitShift = PMIC_USER_SPARE_REGS_USER_SPARE_2_SHIFT;
            *pBitMask = PMIC_USER_SPARE_REGS_USER_SPARE_2_MASK;
            break;
        case PMIC_USER_SPARE_REG_3:
            *pBitShift = PMIC_USER_SPARE_REGS_USER_SPARE_3_SHIFT;
            *pBitMask = PMIC_USER_SPARE_REGS_USER_SPARE_3_MASK;
            break;
        case PMIC_USER_SPARE_REG_4:
            *pBitShift = PMIC_USER_SPARE_REGS_USER_SPARE_4_SHIFT;
            *pBitMask = PMIC_USER_SPARE_REGS_USER_SPARE_4_MASK;
            break;
        default:
            *pBitShift = PMIC_USER_SPARE_REGS_USER_SPARE_1_SHIFT;
            *pBitMask = PMIC_USER_SPARE_REGS_USER_SPARE_1_MASK;
            break;
    }
}

int32_t Pmic_setUserSpareValue(Pmic_CoreHandle_t *pPmicCoreHandle, const uint8_t userSpareRegNum, const uint8_t data)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t bitShift, bitMask, regVal;

    if (NULL == pPmicCoreHandle)
    {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) && (userSpareRegNum > PMIC_USER_SPARE_REG_4))
    {
        pmicStatus = PMIC_ST_ERR_INV_PARAM;
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) && (data > PMIC_USER_SPARE_REG_VAL_1))
    {
        pmicStatus = PMIC_ST_ERR_INV_PARAM;
    }

    if (PMIC_ST_SUCCESS == pmicStatus)
    {
        Pmic_criticalSectionStart(pPmicCoreHandle);
        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_USER_SPARE_REGS_REGADDR, &regVal);
        if (PMIC_ST_SUCCESS == pmicStatus)
        {
            Pmic_getUserSpareRegBitFields(userSpareRegNum, &bitShift, &bitMask);

            Pmic_setBitField(&regVal, bitShift, bitMask, data);

            pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle, PMIC_USER_SPARE_REGS_REGADDR, regVal);
        }

        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    return pmicStatus;
}

int32_t Pmic_getUserSpareValue(Pmic_CoreHandle_t *pPmicCoreHandle, const uint8_t userSpareRegNum, uint8_t *pData)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regVal, bitShift, bitMask;

    if (NULL == pPmicCoreHandle)
    {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) && (userSpareRegNum > PMIC_USER_SPARE_REG_4))
    {
        pmicStatus = PMIC_ST_ERR_INV_PARAM;
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) && (NULL == pData))
    {
        pmicStatus = PMIC_ST_ERR_NULL_PARAM;
    }

    if (PMIC_ST_SUCCESS == pmicStatus)
    {
        Pmic_criticalSectionStart(pPmicCoreHandle);
        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_USER_SPARE_REGS_REGADDR, &regVal);
        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    if (PMIC_ST_SUCCESS == pmicStatus)
    {
        Pmic_getUserSpareRegBitFields(userSpareRegNum, &bitShift, &bitMask);

        *pData = Pmic_getBitField(regVal, bitShift, bitMask);
    }

    return pmicStatus;
}

/*!
 * \brief  API to Enable/Disable Spread Spectrum
 */
static int32_t Pmic_spreadSpectrumEnable(Pmic_CoreHandle_t *pPmicCoreHandle, const Pmic_CommonCtrlCfg_t commonCtrlCfg)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);

    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_SPREAD_SPECTRUM_1_REGADDR, &regData);

    if (PMIC_ST_SUCCESS == pmicStatus)
    {
        if (((bool)PMIC_SPREAD_SPECTRUM_CFG_ENABLE) == commonCtrlCfg.sreadSpectrumEn)
        {
            Pmic_setBitField(&regData,
                             PMIC_SPREAD_SPECTRUM_1_SS_EN_SHIFT,
                             PMIC_SPREAD_SPECTRUM_1_SS_EN_MASK,
                             PMIC_SPREAD_SPECTRUM_CFG_ENABLE);
        }
        else
        {
            Pmic_setBitField(&regData,
                             PMIC_SPREAD_SPECTRUM_1_SS_EN_SHIFT,
                             PMIC_SPREAD_SPECTRUM_1_SS_EN_MASK,
                             PMIC_SPREAD_SPECTRUM_CFG_DISABLE);
        }

        pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle, PMIC_SPREAD_SPECTRUM_1_REGADDR, regData);
    }

    Pmic_criticalSectionStop(pPmicCoreHandle);

    return pmicStatus;
}

/*!
 * \brief  API to get the status of  Spread Spectrum is Enabled/Disabled
 */
static int32_t Pmic_getSpreadSpectrumEnable(Pmic_CoreHandle_t *pPmicCoreHandle, Pmic_CommonCtrlCfg_t *pCommonCtrlCfg)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);
    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_SPREAD_SPECTRUM_1_REGADDR, &regData);
    Pmic_criticalSectionStop(pPmicCoreHandle);

    if (PMIC_ST_SUCCESS == pmicStatus)
    {
        pCommonCtrlCfg->sreadSpectrumEn = (bool)false;

        if (Pmic_getBitField(regData, PMIC_SPREAD_SPECTRUM_1_SS_EN_SHIFT, PMIC_SPREAD_SPECTRUM_1_SS_EN_MASK) == 1U)
        {
            pCommonCtrlCfg->sreadSpectrumEn = (bool)true;
        }
    }

    return pmicStatus;
}

/*!
 * \brief  API to Enable/Disable Skip EEPROM Default Load to CONF and Other
 *         registers
 */
static int32_t Pmic_skipEepromDefaultLoadEnable(Pmic_CoreHandle_t         *pPmicCoreHandle,
                                                const Pmic_CommonCtrlCfg_t commonCtrlCfg)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    if (PMIC_DEV_HERA_LP8764X != pPmicCoreHandle->pmicDeviceType)
    {
        pmicStatus = PMIC_ST_ERR_INV_PARAM;
    }

    if (PMIC_ST_SUCCESS == pmicStatus)
    {
        Pmic_criticalSectionStart(pPmicCoreHandle);

        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_STARTUP_CTRL_REGADDR, &regData);

        if (PMIC_ST_SUCCESS == pmicStatus)
        {
            if (((bool)PMIC_LP8764X_SKIP_EEPROM_DEF_LD_TO_CONF_OTHER_REGS_ENABLED) ==
                commonCtrlCfg.skipEepromDefaultLoadEn)
            {
                Pmic_setBitField(&regData,
                                 PMIC_STARTUP_CTRL_SKIP_LP_STANDBY_EE_READ_SHIFT,
                                 PMIC_STARTUP_CTRL_SKIP_LP_STANDBY_EE_READ_MASK,
                                 PMIC_LP8764X_SKIP_EEPROM_DEF_LD_TO_CONF_OTHER_REGS_ENABLED);
            }
            else
            {
                Pmic_setBitField(&regData,
                                 PMIC_STARTUP_CTRL_SKIP_LP_STANDBY_EE_READ_SHIFT,
                                 PMIC_STARTUP_CTRL_SKIP_LP_STANDBY_EE_READ_MASK,
                                 PMIC_LP8764X_SKIP_EEPROM_DEF_LD_TO_CONF_OTHER_REGS_DISABLED);
            }

            pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle, PMIC_STARTUP_CTRL_REGADDR, regData);
        }

        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    return pmicStatus;
}

/*!
 * \brief  API to get the status of Skip EEPROM Default Load to CONF and Other
 *         registers is Enabled/Disabled
 */
static int32_t Pmic_getSkipEepromDefaultLoadEnable(Pmic_CoreHandle_t    *pPmicCoreHandle,
                                                   Pmic_CommonCtrlCfg_t *pCommonCtrlCfg)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);
    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_STARTUP_CTRL_REGADDR, &regData);
    Pmic_criticalSectionStop(pPmicCoreHandle);

    if (PMIC_ST_SUCCESS == pmicStatus)
    {
        pCommonCtrlCfg->skipEepromDefaultLoadEn = (bool)false;

        if (Pmic_getBitField(regData,
                             PMIC_STARTUP_CTRL_SKIP_LP_STANDBY_EE_READ_SHIFT,
                             PMIC_STARTUP_CTRL_SKIP_LP_STANDBY_EE_READ_MASK) == 1U)
        {
            pCommonCtrlCfg->skipEepromDefaultLoadEn = (bool)true;
        }
    }

    return pmicStatus;
}

/*!
 * \brief   API to get Register Address and Register Bit fields of EEPROM
 *          Default Load to CONF registers
 *          Note: In this API, the default pmicDeviceType is assumed as
 *                LEO PMIC TPS6594x. While adding support for New PMIC
 *                device, developer need to update the API functionality for
 *                New PMIC device accordingly.
 */
static void Pmic_getEepromDefaultLoadRegAddrBitFields(const Pmic_CoreHandle_t *pPmicCoreHandle,
                                                      uint8_t                 *pRegAddr,
                                                      uint8_t                 *pBitShift,
                                                      uint8_t                 *pBitMask)
{
    switch (pPmicCoreHandle->pmicDeviceType)
    {
        case PMIC_DEV_HERA_LP8764X:
            *pRegAddr = PMIC_RTC_CTRL_2_REGADDR;
            *pBitShift = PMIC_RTC_CTRL_2_FIRST_STARTUP_DONE_SHIFT;
            *pBitMask = PMIC_RTC_CTRL_2_FIRST_STARTUP_DONE_MASK;
            break;
        default:
            *pRegAddr = PMIC_STARTUP_CTRL_REGADDR;
            *pBitShift = PMIC_STARTUP_CTRL_FIRST_STARTUP_DONE_SHIFT;
            *pBitMask = PMIC_STARTUP_CTRL_FIRST_STARTUP_DONE_MASK;
            break;
    }
}

/*!
 * \brief  API to Enable/Disable EEPROM Default Load to CONF registers
 */
static int32_t Pmic_eepromDefaultLoadEnable(Pmic_CoreHandle_t         *pPmicCoreHandle,
                                            const Pmic_CommonCtrlCfg_t commonCtrlCfg)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    uint8_t regAddr = 0U, bitShift = 0U, bitMask = 0U;
    uint8_t maxVal;

    switch (pPmicCoreHandle->pmicDeviceType)
    {
        case PMIC_DEV_HERA_LP8764X:
            maxVal = PMIC_LP8764X_EEPROM_DEFAULTS_NOT_LOADED_TO_CONF_OTHER_REGS;
            break;
        default:
            maxVal = PMIC_TPS6594X_EEPROM_DEFAULTS_NOT_LOADED_TO_RTC_DOMAIN_BITS;
            break;
    }

    if (commonCtrlCfg.eepromDefaultLoad > maxVal)
    {
        pmicStatus = PMIC_ST_ERR_INV_PARAM;
    }

    if (PMIC_ST_SUCCESS == pmicStatus)
    {
        Pmic_getEepromDefaultLoadRegAddrBitFields(pPmicCoreHandle, &regAddr, &bitShift, &bitMask);

        Pmic_criticalSectionStart(pPmicCoreHandle);

        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, regAddr, &regData);

        if (PMIC_ST_SUCCESS == pmicStatus)
        {
            Pmic_setBitField(&regData, bitShift, bitMask, commonCtrlCfg.eepromDefaultLoad);

            pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle, regAddr, regData);
        }

        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    return pmicStatus;
}

/*!
 * \brief  API to get the status of EEPROM Default Load to CONF registers is
 *         Enabled/Disabled
 */
static int32_t Pmic_getEepromDefaultLoadEnable(Pmic_CoreHandle_t *pPmicCoreHandle, Pmic_CommonCtrlCfg_t *pCommonCtrlCfg)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    uint8_t regAddr = 0U, bitShift = 0U, bitMask = 0U;

    Pmic_getEepromDefaultLoadRegAddrBitFields(pPmicCoreHandle, &regAddr, &bitShift, &bitMask);

    Pmic_criticalSectionStart(pPmicCoreHandle);
    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, regAddr, &regData);
    Pmic_criticalSectionStop(pPmicCoreHandle);

    if (PMIC_ST_SUCCESS == pmicStatus)
    {
        pCommonCtrlCfg->eepromDefaultLoad = Pmic_getBitField(regData, bitShift, bitMask);
    }

    return pmicStatus;
}

/*!
 * \brief  API to set ENABLE_DRV Pin Configuration
 */
static int32_t Pmic_setEnableDrvPinCfg(Pmic_CoreHandle_t *pPmicCoreHandle, const Pmic_CommonCtrlCfg_t commonCtrlCfg)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t forceEnDrvLowVal, regData = 0U;

    if (commonCtrlCfg.enDrv > PMIC_PIN_SIGNAL_LEVEL_HIGH)
    {
        pmicStatus = PMIC_ST_ERR_INV_PARAM;
    }

    if (PMIC_ST_SUCCESS == pmicStatus)
    {
        Pmic_criticalSectionStart(pPmicCoreHandle);
        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_ENABLE_DRV_STAT_REGADDR, &regData);

        if (PMIC_ST_SUCCESS == pmicStatus)
        {
            forceEnDrvLowVal = Pmic_getBitField(
                regData, PMIC_ENABLE_DRV_STAT_FORCE_EN_DRV_LOW_SHIFT, PMIC_ENABLE_DRV_STAT_FORCE_EN_DRV_LOW_MASK);

            if (PMIC_ENABLE_DRV_I2C_SPI_CONFIG_DISABLE == forceEnDrvLowVal)
            {
                pmicStatus = PMIC_ST_ERR_INV_EN_DRV_PIN_CFG;
            }
        }

        if (PMIC_ST_SUCCESS == pmicStatus)
        {
            pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_ENABLE_DRV_REG_REGADDR, &regData);
        }

        if (PMIC_ST_SUCCESS == pmicStatus)
        {
            Pmic_setBitField(&regData,
                             PMIC_ENABLE_DRV_REG_ENABLE_DRV_SHIFT,
                             PMIC_ENABLE_DRV_REG_ENABLE_DRV_MASK,
                             commonCtrlCfg.enDrv);

            pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle, PMIC_ENABLE_DRV_REG_REGADDR, regData);
        }

        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    return pmicStatus;
}

/*!
 * \brief  API to get ENABLE_DRV Pin Configuration
 */
static int32_t Pmic_getEnableDrvPinCfg(Pmic_CoreHandle_t *pPmicCoreHandle, Pmic_CommonCtrlCfg_t *pCommonCtrlCfg)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);
    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_ENABLE_DRV_REG_REGADDR, &regData);
    Pmic_criticalSectionStop(pPmicCoreHandle);

    if (PMIC_ST_SUCCESS == pmicStatus)
    {
        pCommonCtrlCfg->enDrv =
            Pmic_getBitField(regData, PMIC_ENABLE_DRV_REG_ENABLE_DRV_SHIFT, PMIC_ENABLE_DRV_REG_ENABLE_DRV_MASK);
    }

    return pmicStatus;
}

/*!
 * \brief  API to set Register Lock/UnLock Configuration
 */
static int32_t Pmic_setRegisterLockUnLockCfg(Pmic_CoreHandle_t         *pPmicCoreHandle,
                                             const Pmic_CommonCtrlCfg_t commonCtrlCfg)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    if ((PMIC_REGISTER_UNLOCK != commonCtrlCfg.regLock) && (PMIC_REGISTER_LOCK != commonCtrlCfg.regLock))
    {
        pmicStatus = PMIC_ST_ERR_INV_PARAM;
    }

    if (PMIC_ST_SUCCESS == pmicStatus)
    {
        Pmic_criticalSectionStart(pPmicCoreHandle);

        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_REGISTER_LOCK_REGADDR, &regData);

        if (PMIC_ST_SUCCESS == pmicStatus)
        {
            Pmic_setBitField(&regData,
                             PMIC_REGISTER_LOCK_REGISTER_LOCK_STATUS_SHIFT,
                             PMIC_REGISTER_LOCK_REGISTER_LOCK_STATUS_WRITE_MASK,
                             commonCtrlCfg.regLock);

            pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle, PMIC_REGISTER_LOCK_REGADDR, regData);
        }

        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    return pmicStatus;
}

/*!
 * \brief  API to set Spread Spectrum modulation Depth Value Configuration
 */
static int32_t Pmic_setSpreadSpectrumModDepthCfg(Pmic_CoreHandle_t         *pPmicCoreHandle,
                                                 const Pmic_CommonCtrlCfg_t commonCtrlCfg)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    if (commonCtrlCfg.spreadSpectrumDepth > PMIC_SPREAD_SPECTRUM_MODULATION_DEPTH_8_4_PERCENT)
    {
        pmicStatus = PMIC_ST_ERR_INV_PARAM;
    }

    if (PMIC_ST_SUCCESS == pmicStatus)
    {
        Pmic_criticalSectionStart(pPmicCoreHandle);

        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_SPREAD_SPECTRUM_1_REGADDR, &regData);

        if (PMIC_ST_SUCCESS == pmicStatus)
        {
            Pmic_setBitField(&regData,
                             PMIC_SPREAD_SPECTRUM_1_SS_DEPTH_SHIFT,
                             PMIC_SPREAD_SPECTRUM_1_SS_DEPTH_MASK,
                             commonCtrlCfg.spreadSpectrumDepth);

            pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle, PMIC_SPREAD_SPECTRUM_1_REGADDR, regData);
        }

        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    return pmicStatus;
}

/*!
 * \brief  API to get Spread Spectrum modulation Depth Value Configuration
 */
static int32_t Pmic_getSpreadSpectrumModDepthCfg(Pmic_CoreHandle_t    *pPmicCoreHandle,
                                                 Pmic_CommonCtrlCfg_t *pCommonCtrlCfg)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);
    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_SPREAD_SPECTRUM_1_REGADDR, &regData);
    Pmic_criticalSectionStop(pPmicCoreHandle);

    if (PMIC_ST_SUCCESS == pmicStatus)
    {
        pCommonCtrlCfg->spreadSpectrumDepth =
            Pmic_getBitField(regData, PMIC_SPREAD_SPECTRUM_1_SS_DEPTH_SHIFT, PMIC_SPREAD_SPECTRUM_1_SS_DEPTH_MASK);
    }

    return pmicStatus;
}

int32_t Pmic_setCommonCtrlConfig(Pmic_CoreHandle_t *pPmicCoreHandle, const Pmic_CommonCtrlCfg_t commonCtrlCfg)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    if (NULL == pPmicCoreHandle)
    {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) &&
        ((bool)true == pmic_validParamCheck(commonCtrlCfg.validParams, PMIC_CFG_SPREAD_SPECTRUM_EN_VALID)))
    {
        /* Enable/Disable Spread Spectrum */
        pmicStatus = Pmic_spreadSpectrumEnable(pPmicCoreHandle, commonCtrlCfg);
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) &&
        ((bool)true == pmic_validParamCheck(commonCtrlCfg.validParams, PMIC_CFG_SKIP_EEPROM_LOAD_VALID)))
    {
        /* Enable/Disable Skip EEPROM Default Load to CONF and Other registers*/
        pmicStatus = Pmic_skipEepromDefaultLoadEnable(pPmicCoreHandle, commonCtrlCfg);
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) &&
        ((bool)true == pmic_validParamCheck(commonCtrlCfg.validParams, PMIC_CFG_EEPROM_DEFAULT_VALID)))
    {
        /* Enable/Disable EEPROM Default Load to CONF registers*/
        pmicStatus = Pmic_eepromDefaultLoadEnable(pPmicCoreHandle, commonCtrlCfg);
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) &&
        ((bool)true == pmic_validParamCheck(commonCtrlCfg.validParams, PMIC_CFG_ENABLE_DRV_VALID)))
    {
        /* Set ENABLE_DRV Pin Configuration */
        pmicStatus = Pmic_setEnableDrvPinCfg(pPmicCoreHandle, commonCtrlCfg);
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) &&
        ((bool)true == pmic_validParamCheck(commonCtrlCfg.validParams, PMIC_CFG_REG_LOCK_VALID)))
    {
        /* Set Register Lock/UnLock Configuration */
        pmicStatus = Pmic_setRegisterLockUnLockCfg(pPmicCoreHandle, commonCtrlCfg);
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) &&
        ((bool)true == pmic_validParamCheck(commonCtrlCfg.validParams, PMIC_CFG_SPREAD_SPECTRUM_DEPTH_VALID)))
    {
        /* Spread Spectrum modulation Depth Value Configuration */
        pmicStatus = Pmic_setSpreadSpectrumModDepthCfg(pPmicCoreHandle, commonCtrlCfg);
    }

    return pmicStatus;
}

int32_t Pmic_getCommonCtrlConfig(Pmic_CoreHandle_t *pPmicCoreHandle, Pmic_CommonCtrlCfg_t *pCommonCtrlCfg)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    if (NULL == pPmicCoreHandle)
    {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) && (NULL == pCommonCtrlCfg))
    {
        pmicStatus = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) &&
        ((bool)true == pmic_validParamCheck(pCommonCtrlCfg->validParams, PMIC_CFG_SPREAD_SPECTRUM_EN_VALID)))
    {
        /* Get the status of Spread Spectrum is Enabled/Disabled  */
        pmicStatus = Pmic_getSpreadSpectrumEnable(pPmicCoreHandle, pCommonCtrlCfg);
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) &&
        ((bool)true == pmic_validParamCheck(pCommonCtrlCfg->validParams, PMIC_CFG_SKIP_EEPROM_LOAD_VALID)))
    {
        if (PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
        {
            /* Get the status of Skip EEPROM Default Load to CONF and Other
             * registers is Enabled/Disabled */
            pmicStatus = Pmic_getSkipEepromDefaultLoadEnable(pPmicCoreHandle, pCommonCtrlCfg);
        }
        else
        {
            pmicStatus = PMIC_ST_ERR_INV_PARAM;
        }
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) &&
        ((bool)true == pmic_validParamCheck(pCommonCtrlCfg->validParams, PMIC_CFG_EEPROM_DEFAULT_VALID)))
    {
        /* Get the status of EEPROM Default Load to CONF registers is
         * Enabled/Disabled */
        pmicStatus = Pmic_getEepromDefaultLoadEnable(pPmicCoreHandle, pCommonCtrlCfg);
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) &&
        ((bool)true == pmic_validParamCheck(pCommonCtrlCfg->validParams, PMIC_CFG_ENABLE_DRV_VALID)))
    {
        /* Get ENABLE_DRV Pin Configuration */
        pmicStatus = Pmic_getEnableDrvPinCfg(pPmicCoreHandle, pCommonCtrlCfg);
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) &&
        ((bool)true == pmic_validParamCheck(pCommonCtrlCfg->validParams, PMIC_CFG_SPREAD_SPECTRUM_DEPTH_VALID)))
    {
        /* Get Spread Spectrum modulation Depth Value Configuration */
        pmicStatus = Pmic_getSpreadSpectrumModDepthCfg(pPmicCoreHandle, pCommonCtrlCfg);
    }

    return pmicStatus;
}

/*!
 * \brief  API to set AMUX_OUT/REF_OUT Pin Control Configuration
 */
static int32_t Pmic_setAmuxOutRefOutPinCfg(Pmic_CoreHandle_t *pPmicCoreHandle, const Pmic_MiscCtrlCfg_t miscCtrlCfg)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    bool    amuxRefEn = miscCtrlCfg.amuxOutRefOutEn;

    Pmic_criticalSectionStart(pPmicCoreHandle);

    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_MISC_CTRL_REGADDR, &regData);

    if (PMIC_ST_SUCCESS == pmicStatus)
    {
        switch (pPmicCoreHandle->pmicDeviceType)
        {
            case PMIC_DEV_HERA_LP8764X:
                if (((bool)PMIC_LP8764X_REF_OUT_PIN_CFG_ENABLE) == amuxRefEn)
                {
                    Pmic_setBitField(&regData,
                                     PMIC_MISC_CTRL_AMUXOUT_REFOUT_EN_SHIFT,
                                     PMIC_MISC_CTRL_AMUXOUT_REFOUT_EN_MASK,
                                     PMIC_LP8764X_REF_OUT_PIN_CFG_ENABLE);
                }
                else
                {
                    Pmic_setBitField(&regData,
                                     PMIC_MISC_CTRL_AMUXOUT_REFOUT_EN_SHIFT,
                                     PMIC_MISC_CTRL_AMUXOUT_REFOUT_EN_MASK,
                                     PMIC_LP8764X_REF_OUT_PIN_CFG_DISABLE);
                }
                break;
            default:
                /* Default case is valid only for TPS6594x LEO PMIC */
                if (((bool)PMIC_TPS6594X_AMUX_OUT_PIN_CFG_ENABLE) == amuxRefEn)
                {
                    Pmic_setBitField(&regData,
                                     PMIC_MISC_CTRL_AMUXOUT_REFOUT_EN_SHIFT,
                                     PMIC_MISC_CTRL_AMUXOUT_REFOUT_EN_MASK,
                                     PMIC_TPS6594X_AMUX_OUT_PIN_CFG_ENABLE);
                }
                else
                {
                    Pmic_setBitField(&regData,
                                     PMIC_MISC_CTRL_AMUXOUT_REFOUT_EN_SHIFT,
                                     PMIC_MISC_CTRL_AMUXOUT_REFOUT_EN_MASK,
                                     PMIC_TPS6594X_AMUX_OUT_PIN_CFG_DISABLE);
                }
                break;
        }

        pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle, PMIC_MISC_CTRL_REGADDR, regData);
    }

    Pmic_criticalSectionStop(pPmicCoreHandle);

    return pmicStatus;
}

/*!
 * \brief  API to get AMUX_OUT/REF_OUT Pin Control Configuration
 */
static int32_t Pmic_getAmuxOutRefOutPinCfg(Pmic_CoreHandle_t *pPmicCoreHandle, Pmic_MiscCtrlCfg_t *pMiscCtrlCfg)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);
    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_MISC_CTRL_REGADDR, &regData);
    Pmic_criticalSectionStop(pPmicCoreHandle);

    if (PMIC_ST_SUCCESS == pmicStatus)
    {
        pMiscCtrlCfg->amuxOutRefOutEn = (bool)false;

        if (Pmic_getBitField(regData, PMIC_MISC_CTRL_AMUXOUT_REFOUT_EN_SHIFT, PMIC_MISC_CTRL_AMUXOUT_REFOUT_EN_MASK) ==
            1U)
        {
            pMiscCtrlCfg->amuxOutRefOutEn = (bool)true;
        }
    }

    return pmicStatus;
}

/*!
 * \brief  API to set Internal Clock Monitoring Configuration
 */
static int32_t Pmic_setInternalClkMonitorCfg(Pmic_CoreHandle_t *pPmicCoreHandle, const Pmic_MiscCtrlCfg_t miscCtrlCfg)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);

    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_MISC_CTRL_REGADDR, &regData);

    if (PMIC_ST_SUCCESS == pmicStatus)
    {
        switch (pPmicCoreHandle->pmicDeviceType)
        {
            case PMIC_DEV_HERA_LP8764X:
                if (((bool)PMIC_LP8764X_REF_OUT_PIN_CFG_ENABLE) == miscCtrlCfg.clkMonEn)
                {
                    Pmic_setBitField(&regData,
                                     PMIC_MISC_CTRL_CLKMON_EN_SHIFT,
                                     PMIC_MISC_CTRL_CLKMON_EN_MASK,
                                     PMIC_LP8764X_REF_OUT_PIN_CFG_ENABLE);
                }
                else
                {
                    Pmic_setBitField(&regData,
                                     PMIC_MISC_CTRL_CLKMON_EN_SHIFT,
                                     PMIC_MISC_CTRL_CLKMON_EN_MASK,
                                     PMIC_LP8764X_REF_OUT_PIN_CFG_DISABLE);
                }
                break;
            default:
                /* Default case is valid only for TPS6594x LEO PMIC */
                if (((bool)PMIC_TPS6594X_AMUX_OUT_PIN_CFG_ENABLE) == miscCtrlCfg.clkMonEn)
                {
                    Pmic_setBitField(&regData,
                                     PMIC_MISC_CTRL_CLKMON_EN_SHIFT,
                                     PMIC_MISC_CTRL_CLKMON_EN_MASK,
                                     PMIC_TPS6594X_AMUX_OUT_PIN_CFG_ENABLE);
                }
                else
                {
                    Pmic_setBitField(&regData,
                                     PMIC_MISC_CTRL_CLKMON_EN_SHIFT,
                                     PMIC_MISC_CTRL_CLKMON_EN_MASK,
                                     PMIC_TPS6594X_AMUX_OUT_PIN_CFG_DISABLE);
                }
                break;
        }

        pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle, PMIC_MISC_CTRL_REGADDR, regData);
    }

    Pmic_criticalSectionStop(pPmicCoreHandle);

    return pmicStatus;
}

/*!
 * \brief  API to get Internal Clock Monitoring Configuration
 */
static int32_t Pmic_getInternalClkMonitorCfg(Pmic_CoreHandle_t *pPmicCoreHandle, Pmic_MiscCtrlCfg_t *pMiscCtrlCfg)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);
    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_MISC_CTRL_REGADDR, &regData);
    Pmic_criticalSectionStop(pPmicCoreHandle);

    if (PMIC_ST_SUCCESS == pmicStatus)
    {
        pMiscCtrlCfg->clkMonEn = (bool)false;

        if (Pmic_getBitField(regData, PMIC_MISC_CTRL_CLKMON_EN_SHIFT, PMIC_MISC_CTRL_CLKMON_EN_MASK) == 1U)
        {
            pMiscCtrlCfg->clkMonEn = (bool)true;
        }
    }

    return pmicStatus;
}

/*!
 * \brief  API to select SYNCCLKOUT Frequency
 */
static int32_t Pmic_selectSyncClkOutFreq(Pmic_CoreHandle_t *pPmicCoreHandle, const Pmic_MiscCtrlCfg_t miscCtrlCfg)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    if (miscCtrlCfg.syncClkOutFreqSel > PMIC_SYNCCLKOUT_4_4_MHZ)
    {
        pmicStatus = PMIC_ST_ERR_INV_PARAM;
    }

    if (PMIC_ST_SUCCESS == pmicStatus)
    {
        Pmic_criticalSectionStart(pPmicCoreHandle);

        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_MISC_CTRL_REGADDR, &regData);

        if (PMIC_ST_SUCCESS == pmicStatus)
        {
            Pmic_setBitField(&regData,
                             PMIC_MISC_CTRL_SYNCCLKOUT_FREQ_SEL_SHIFT,
                             PMIC_MISC_CTRL_SYNCCLKOUT_FREQ_SEL_MASK,
                             miscCtrlCfg.syncClkOutFreqSel);

            pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle, PMIC_MISC_CTRL_REGADDR, regData);
        }

        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    return pmicStatus;
}

/*!
 * \brief  API to get Get SYNCCLKOUT Frequency Selection Configuration
 */
static int32_t Pmic_getSyncClkOutFreqSelectCfg(Pmic_CoreHandle_t *pPmicCoreHandle, Pmic_MiscCtrlCfg_t *pMiscCtrlCfg)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);
    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_MISC_CTRL_REGADDR, &regData);
    Pmic_criticalSectionStop(pPmicCoreHandle);

    if (PMIC_ST_SUCCESS == pmicStatus)
    {
        pMiscCtrlCfg->syncClkOutFreqSel = Pmic_getBitField(
            regData, PMIC_MISC_CTRL_SYNCCLKOUT_FREQ_SEL_SHIFT, PMIC_MISC_CTRL_SYNCCLKOUT_FREQ_SEL_MASK);
    }

    return pmicStatus;
}

/*!
 * \brief  API to Select External Clock
 */
static int32_t Pmic_selectExternalClk(Pmic_CoreHandle_t *pPmicCoreHandle, const Pmic_MiscCtrlCfg_t miscCtrlCfg)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    if (miscCtrlCfg.extClkSel > PMIC_AUTOMATIC_EXT_CLK)
    {
        pmicStatus = PMIC_ST_ERR_INV_PARAM;
    }

    if (PMIC_ST_SUCCESS == pmicStatus)
    {
        Pmic_criticalSectionStart(pPmicCoreHandle);

        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_MISC_CTRL_REGADDR, &regData);

        if (PMIC_ST_SUCCESS == pmicStatus)
        {
            Pmic_setBitField(
                &regData, PMIC_MISC_CTRL_SEL_EXT_CLK_SHIFT, PMIC_MISC_CTRL_SEL_EXT_CLK_MASK, miscCtrlCfg.extClkSel);

            pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle, PMIC_MISC_CTRL_REGADDR, regData);
        }

        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    return pmicStatus;
}

/*!
 * \brief  API to get External Clock Selection Configuration
 */
static int32_t Pmic_getExternalClkSelectCfg(Pmic_CoreHandle_t *pPmicCoreHandle, Pmic_MiscCtrlCfg_t *pMiscCtrlCfg)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);
    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_MISC_CTRL_REGADDR, &regData);
    Pmic_criticalSectionStop(pPmicCoreHandle);

    if (PMIC_ST_SUCCESS == pmicStatus)
    {
        pMiscCtrlCfg->extClkSel =
            Pmic_getBitField(regData, PMIC_MISC_CTRL_SEL_EXT_CLK_SHIFT, PMIC_MISC_CTRL_SEL_EXT_CLK_MASK);
    }

    return pmicStatus;
}

/*!
 * \brief  API to select External Clock Frequency
 */
static int32_t Pmic_selectExternalClkFreq(Pmic_CoreHandle_t *pPmicCoreHandle, const Pmic_MiscCtrlCfg_t miscCtrlCfg)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    uint8_t maxVal;

    switch (pPmicCoreHandle->pmicDeviceType)
    {
        case PMIC_DEV_HERA_LP8764X:
            maxVal = PMIC_LP8764X_SYNCCLKIN_8_8_MHZ;
            break;
        default:
            maxVal = PMIC_TPS6594X_SYNCCLKIN_4_4_MHZ;
            break;
    }

    if (miscCtrlCfg.syncClkInFreq > maxVal)
    {
        pmicStatus = PMIC_ST_ERR_INV_PARAM;
    }

    if (PMIC_ST_SUCCESS == pmicStatus)
    {
        Pmic_criticalSectionStart(pPmicCoreHandle);

        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_PLL_CTRL_REGADDR, &regData);

        if (PMIC_ST_SUCCESS == pmicStatus)
        {
            Pmic_setBitField(
                &regData, PMIC_PLL_CTRL_EXT_CLK_FREQ_SHIFT, PMIC_PLL_CTRL_EXT_CLK_FREQ_MASK, miscCtrlCfg.syncClkInFreq);

            pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle, PMIC_PLL_CTRL_REGADDR, regData);
        }

        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    return pmicStatus;
}

/*!
 * \brief  API to configure NRSTOUT_SOC Signal
 */
static int32_t Pmic_setNRstOutSocSignalCfg(Pmic_CoreHandle_t *pPmicCoreHandle, const Pmic_MiscCtrlCfg_t miscCtrlCfg)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    if (miscCtrlCfg.nRstOutSocSignal > PMIC_PIN_SIGNAL_LEVEL_HIGH)
    {
        pmicStatus = PMIC_ST_ERR_INV_PARAM;
    }

    if (PMIC_ST_SUCCESS == pmicStatus)
    {
        Pmic_criticalSectionStart(pPmicCoreHandle);

        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_MISC_CTRL_REGADDR, &regData);

        if (PMIC_ST_SUCCESS == pmicStatus)
        {
            Pmic_setBitField(&regData,
                             PMIC_MISC_CTRL_NRSTOUT_SOC_SHIFT,
                             PMIC_MISC_CTRL_NRSTOUT_SOC_MASK,
                             miscCtrlCfg.nRstOutSocSignal);

            pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle, PMIC_MISC_CTRL_REGADDR, regData);
        }

        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    return pmicStatus;
}

/*!
 * \brief  API to configure NRSTOUT Signal
 */
static int32_t Pmic_setNRstOutSignalCfg(Pmic_CoreHandle_t *pPmicCoreHandle, const Pmic_MiscCtrlCfg_t miscCtrlCfg)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    if (miscCtrlCfg.nRstOutSignal > PMIC_PIN_SIGNAL_LEVEL_HIGH)
    {
        pmicStatus = PMIC_ST_ERR_INV_PARAM;
    }

    if (PMIC_ST_SUCCESS == pmicStatus)
    {
        Pmic_criticalSectionStart(pPmicCoreHandle);

        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_MISC_CTRL_REGADDR, &regData);

        if (PMIC_ST_SUCCESS == pmicStatus)
        {
            Pmic_setBitField(
                &regData, PMIC_MISC_CTRL_NRSTOUT_SHIFT, PMIC_MISC_CTRL_NRSTOUT_MASK, miscCtrlCfg.nRstOutSignal);

            pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle, PMIC_MISC_CTRL_REGADDR, regData);
        }

        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    return pmicStatus;
}

/*!
 * \brief  API to get ExternalClk Frequency Selection Configuration
 */
static int32_t Pmic_getExternalClkFreqSelectCfg(Pmic_CoreHandle_t *pPmicCoreHandle, Pmic_MiscCtrlCfg_t *pMiscCtrlCfg)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);
    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_PLL_CTRL_REGADDR, &regData);
    Pmic_criticalSectionStop(pPmicCoreHandle);

    if (PMIC_ST_SUCCESS == pmicStatus)
    {
        pMiscCtrlCfg->syncClkInFreq =
            Pmic_getBitField(regData, PMIC_PLL_CTRL_EXT_CLK_FREQ_SHIFT, PMIC_PLL_CTRL_EXT_CLK_FREQ_MASK);
    }

    return pmicStatus;
}

/*!
 * \brief  API to get NRSTOUT_SOC Signal Configuration
 */
static int32_t Pmic_getNRstOutSocSignalCfg(Pmic_CoreHandle_t *pPmicCoreHandle, Pmic_MiscCtrlCfg_t *pMiscCtrlCfg)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);
    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_MISC_CTRL_REGADDR, &regData);
    Pmic_criticalSectionStop(pPmicCoreHandle);

    if (PMIC_ST_SUCCESS == pmicStatus)
    {
        pMiscCtrlCfg->nRstOutSocSignal =
            Pmic_getBitField(regData, PMIC_MISC_CTRL_NRSTOUT_SOC_SHIFT, PMIC_MISC_CTRL_NRSTOUT_SOC_MASK);
    }

    return pmicStatus;
}

/*!
 * \brief  API to get NRSTOUT Signal Configuration
 */
static int32_t Pmic_getNRstOutSignalCfg(Pmic_CoreHandle_t *pPmicCoreHandle, Pmic_MiscCtrlCfg_t *pMiscCtrlCfg)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);
    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_MISC_CTRL_REGADDR, &regData);
    Pmic_criticalSectionStop(pPmicCoreHandle);

    if (PMIC_ST_SUCCESS == pmicStatus)
    {
        pMiscCtrlCfg->nRstOutSignal =
            Pmic_getBitField(regData, PMIC_MISC_CTRL_NRSTOUT_SHIFT, PMIC_MISC_CTRL_NRSTOUT_MASK);
    }

    return pmicStatus;
}

/*!
 * \brief   API to set ExternalClk Frequency Selection, NRSTOUT_SOC and
 *          NRSTOUT Signal Configuration
 */
static int32_t Pmic_setExtclkfreqSelNRstOutSocNRstOutCfg(Pmic_CoreHandle_t       *pPmicCoreHandle,
                                                         const Pmic_MiscCtrlCfg_t miscCtrlCfg)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    if ((bool)true == pmic_validParamCheck(miscCtrlCfg.validParams, PMIC_CFG_SYNC_CLK_IN_FREQ_VALID))
    {
        /* Selects External Clock Frequency */
        pmicStatus = Pmic_selectExternalClkFreq(pPmicCoreHandle, miscCtrlCfg);
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) &&
        ((bool)true == pmic_validParamCheck(miscCtrlCfg.validParams, PMIC_CFG_NRSTOUT_SOC_VALID)))
    {
        /* Configure NRSTOUT_SOC Signal */
        pmicStatus = Pmic_setNRstOutSocSignalCfg(pPmicCoreHandle, miscCtrlCfg);
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) &&
        ((bool)true == pmic_validParamCheck(miscCtrlCfg.validParams, PMIC_CFG_NRSTOUT_VALID)))
    {
        /* Configure NRSTOUT Signal */
        pmicStatus = Pmic_setNRstOutSignalCfg(pPmicCoreHandle, miscCtrlCfg);
    }

    return pmicStatus;
}

int32_t Pmic_setMiscCtrlConfig(Pmic_CoreHandle_t *pPmicCoreHandle, const Pmic_MiscCtrlCfg_t miscCtrlCfg)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    if (NULL == pPmicCoreHandle)
    {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) &&
        ((bool)true == pmic_validParamCheck(miscCtrlCfg.validParams, PMIC_CFG_AMUX_OUT_REF_OUT_EN_VALID)))
    {
        /* Set AMUX_OUT/REF_OUT Pin Control Configuration */
        pmicStatus = Pmic_setAmuxOutRefOutPinCfg(pPmicCoreHandle, miscCtrlCfg);
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) &&
        ((bool)true == pmic_validParamCheck(miscCtrlCfg.validParams, PMIC_CFG_CLK_MON_EN_VALID)))
    {
        /* Set Internal Clock Monitoring Configuration*/
        pmicStatus = Pmic_setInternalClkMonitorCfg(pPmicCoreHandle, miscCtrlCfg);
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) &&
        ((bool)true == pmic_validParamCheck(miscCtrlCfg.validParams, PMIC_CFG_SYNC_CLK_OUT_FREQ_SEL_VALID)))
    {
        /* Selects SYNCCLKOUT Frequency*/
        pmicStatus = Pmic_selectSyncClkOutFreq(pPmicCoreHandle, miscCtrlCfg);
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) &&
        ((bool)true == pmic_validParamCheck(miscCtrlCfg.validParams, PMIC_CFG_EXT_CLK_SEL_VALID)))
    {
        /* Select External Clock */
        pmicStatus = Pmic_selectExternalClk(pPmicCoreHandle, miscCtrlCfg);
    }

    if (PMIC_ST_SUCCESS == pmicStatus)
    {
        /* Selects External Clock Frequency, Configure NRSTOUT_SOC and NRSTOUT
         * Signal */
        pmicStatus = Pmic_setExtclkfreqSelNRstOutSocNRstOutCfg(pPmicCoreHandle, miscCtrlCfg);
    }

    return pmicStatus;
}

/*!
 * \brief   API to get PMIC ExternalClk Frequency Selection, NRSTOUT_SOC and
 *          NRSTOUT Signal Configuration
 */
static int32_t Pmic_getExtclkfreqSelNRstOutSocNRstOutCfg(Pmic_CoreHandle_t  *pPmicCoreHandle,
                                                         Pmic_MiscCtrlCfg_t *pMiscCtrlCfg)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    if ((bool)true == pmic_validParamCheck(pMiscCtrlCfg->validParams, PMIC_CFG_SYNC_CLK_IN_FREQ_VALID))
    {
        /* Get ExternalClk Frequency Selection Configuration */
        pmicStatus = Pmic_getExternalClkFreqSelectCfg(pPmicCoreHandle, pMiscCtrlCfg);
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) &&
        ((bool)true == pmic_validParamCheck(pMiscCtrlCfg->validParams, PMIC_CFG_NRSTOUT_SOC_VALID)))
    {
        /* Get NRSTOUT_SOC Signal Configuration */
        pmicStatus = Pmic_getNRstOutSocSignalCfg(pPmicCoreHandle, pMiscCtrlCfg);
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) &&
        ((bool)true == pmic_validParamCheck(pMiscCtrlCfg->validParams, PMIC_CFG_NRSTOUT_VALID)))
    {
        /* Get NRSTOUT Signal Configuration */
        pmicStatus = Pmic_getNRstOutSignalCfg(pPmicCoreHandle, pMiscCtrlCfg);
    }

    return pmicStatus;
}

int32_t Pmic_getMiscCtrlConfig(Pmic_CoreHandle_t *pPmicCoreHandle, Pmic_MiscCtrlCfg_t *pMiscCtrlCfg)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    if (NULL == pPmicCoreHandle)
    {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) && (NULL == pMiscCtrlCfg))
    {
        pmicStatus = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) &&
        ((bool)true == pmic_validParamCheck(pMiscCtrlCfg->validParams, PMIC_CFG_AMUX_OUT_REF_OUT_EN_VALID)))
    {
        /* Get AMUX_OUT/REF_OUT Pin Control Configuration */
        pmicStatus = Pmic_getAmuxOutRefOutPinCfg(pPmicCoreHandle, pMiscCtrlCfg);
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) &&
        ((bool)true == pmic_validParamCheck(pMiscCtrlCfg->validParams, PMIC_CFG_CLK_MON_EN_VALID)))
    {
        /* Get Internal Clock Monitoring Configuration*/
        pmicStatus = Pmic_getInternalClkMonitorCfg(pPmicCoreHandle, pMiscCtrlCfg);
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) &&
        ((bool)true == pmic_validParamCheck(pMiscCtrlCfg->validParams, PMIC_CFG_SYNC_CLK_OUT_FREQ_SEL_VALID)))
    {
        /* Get SYNCCLKOUT Frequency Selection Configuration */
        pmicStatus = Pmic_getSyncClkOutFreqSelectCfg(pPmicCoreHandle, pMiscCtrlCfg);
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) &&
        ((bool)true == pmic_validParamCheck(pMiscCtrlCfg->validParams, PMIC_CFG_EXT_CLK_SEL_VALID)))
    {
        /* Get External Clock Selection Configuration */
        pmicStatus = Pmic_getExternalClkSelectCfg(pPmicCoreHandle, pMiscCtrlCfg);
    }

    if (PMIC_ST_SUCCESS == pmicStatus)
    {
        /* Get ExternalClk Frequency Selection, NRSTOUT_SOC and NRSTOUT Signal
         * Configuration */
        pmicStatus = Pmic_getExtclkfreqSelNRstOutSocNRstOutCfg(pPmicCoreHandle, pMiscCtrlCfg);
    }

    return pmicStatus;
}

/*!
 * \brief  API to Enable/Disable Backup Battery Charging
 */
static int32_t Pmic_backupBatteryChargingEnable(Pmic_CoreHandle_t          *pPmicCoreHandle,
                                                const Pmic_BatteryCtrlCfg_t batteryCtrlCfg)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);

    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_CONFIG_2_REGADDR, &regData);

    if (PMIC_ST_SUCCESS == pmicStatus)
    {
        if (((bool)PMIC_TPS6594X_BB_CHARGINGING_CFG_ENABLE) == batteryCtrlCfg.chargingEn)
        {
            Pmic_setBitField(&regData,
                             PMIC_CONFIG_2_BB_CHARGER_EN_SHIFT,
                             PMIC_CONFIG_2_BB_CHARGER_EN_MASK,
                             PMIC_TPS6594X_BB_CHARGINGING_CFG_ENABLE);
        }
        else
        {
            Pmic_setBitField(&regData,
                             PMIC_CONFIG_2_BB_CHARGER_EN_SHIFT,
                             PMIC_CONFIG_2_BB_CHARGER_EN_MASK,
                             PMIC_TPS6594X_BB_CHARGINGING_CFG_DISABLE);
        }

        pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle, PMIC_CONFIG_2_REGADDR, regData);
    }

    Pmic_criticalSectionStop(pPmicCoreHandle);

    return pmicStatus;
}

/*!
 * \brief  API to get the status of Backup Battery Charging is Enabled/Disabled
 */
static int32_t Pmic_getBackupBatteryChargingCfg(Pmic_CoreHandle_t     *pPmicCoreHandle,
                                                Pmic_BatteryCtrlCfg_t *pBatteryCtrlCfg)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);
    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_CONFIG_2_REGADDR, &regData);
    Pmic_criticalSectionStop(pPmicCoreHandle);

    if (PMIC_ST_SUCCESS == pmicStatus)
    {
        pBatteryCtrlCfg->chargingEn = (bool)false;

        if (Pmic_getBitField(regData, PMIC_CONFIG_2_BB_CHARGER_EN_SHIFT, PMIC_CONFIG_2_BB_CHARGER_EN_MASK) == 1U)
        {
            pBatteryCtrlCfg->chargingEn = (bool)true;
        }
    }

    return pmicStatus;
}

/*!
 * \brief  API to Set Backup Battery configuration for End of charge Voltage
 */
static int32_t Pmic_setBackupBatteryEndOfChargeVoltage(Pmic_CoreHandle_t          *pPmicCoreHandle,
                                                       const Pmic_BatteryCtrlCfg_t batteryCtrlCfg)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);

    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_CONFIG_2_REGADDR, &regData);

    if (PMIC_ST_SUCCESS == pmicStatus)
    {
        Pmic_setBitField(
            &regData, PMIC_CONFIG_2_BB_VEOC_SHIFT, PMIC_CONFIG_2_BB_VEOC_MASK, batteryCtrlCfg.endOfChargeVoltage);

        pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle, PMIC_CONFIG_2_REGADDR, regData);
    }

    Pmic_criticalSectionStop(pPmicCoreHandle);

    return pmicStatus;
}

/*!
 * \brief  API to get Backup Battery configuration for End of charge Voltage
 */
static int32_t Pmic_getBackupBatteryEndOfChargeVoltage(Pmic_CoreHandle_t     *pPmicCoreHandle,
                                                       Pmic_BatteryCtrlCfg_t *pBatteryCtrlCfg)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);
    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_CONFIG_2_REGADDR, &regData);
    Pmic_criticalSectionStop(pPmicCoreHandle);

    if (PMIC_ST_SUCCESS == pmicStatus)
    {
        pBatteryCtrlCfg->endOfChargeVoltage =
            Pmic_getBitField(regData, PMIC_CONFIG_2_BB_VEOC_SHIFT, PMIC_CONFIG_2_BB_VEOC_MASK);
    }

    return pmicStatus;
}

/*!
 * \brief  API to get Backup Battery charging current value
 */
static int32_t Pmic_setBackupBatteryChargingCurrentVal(Pmic_CoreHandle_t          *pPmicCoreHandle,
                                                       const Pmic_BatteryCtrlCfg_t batteryCtrlCfg)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);

    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_CONFIG_2_REGADDR, &regData);

    if (PMIC_ST_SUCCESS == pmicStatus)
    {
        Pmic_setBitField(
            &regData, PMIC_CONFIG_2_BB_ICHR_SHIFT, PMIC_CONFIG_2_BB_ICHR_MASK, batteryCtrlCfg.chargeCurrent);

        pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle, PMIC_CONFIG_2_REGADDR, regData);
    }

    Pmic_criticalSectionStop(pPmicCoreHandle);

    return pmicStatus;
}

/*!
 * \brief  API to get Backup Battery charging current value
 */
static int32_t Pmic_getBackupBatteryChargingCurrentVal(Pmic_CoreHandle_t     *pPmicCoreHandle,
                                                       Pmic_BatteryCtrlCfg_t *pBatteryCtrlCfg)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);
    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_CONFIG_2_REGADDR, &regData);
    Pmic_criticalSectionStop(pPmicCoreHandle);

    if (PMIC_ST_SUCCESS == pmicStatus)
    {
        pBatteryCtrlCfg->chargeCurrent =
            Pmic_getBitField(regData, PMIC_CONFIG_2_BB_ICHR_SHIFT, PMIC_CONFIG_2_BB_ICHR_MASK);
    }

    return pmicStatus;
}

int32_t Pmic_setBatteryCtrlConfig(Pmic_CoreHandle_t *pPmicCoreHandle, const Pmic_BatteryCtrlCfg_t batteryCtrlCfg)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    if (NULL == pPmicCoreHandle)
    {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) && (PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType))
    {
        pmicStatus = PMIC_ST_ERR_NOT_SUPPORTED;
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) &&
        ((bool)true == pmic_validParamCheck(batteryCtrlCfg.validParams, PMIC_CFG_CHARGING_EN_VALID)))
    {
        /* Enable/Disable Backup Battery Charging */
        pmicStatus = Pmic_backupBatteryChargingEnable(pPmicCoreHandle, batteryCtrlCfg);
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) &&
        ((bool)true == pmic_validParamCheck(batteryCtrlCfg.validParams, PMIC_CFG_END_OF_CHARGE_VOLTAGE_VALID)))
    {
        if (batteryCtrlCfg.endOfChargeVoltage > PMIC_TPS6594X_BB_ENDOF_CHARGE_VOLATGE_3_3_V)
        {
            pmicStatus = PMIC_ST_ERR_INV_PARAM;
        }

        if (PMIC_ST_SUCCESS == pmicStatus)
        {
            /* Set Backup Battery configuration for End of charge Voltage*/
            pmicStatus = Pmic_setBackupBatteryEndOfChargeVoltage(pPmicCoreHandle, batteryCtrlCfg);
        }
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) &&
        ((bool)true == pmic_validParamCheck(batteryCtrlCfg.validParams, PMIC_CFG_CHARGE_CURRENT_VALID)))
    {
        if (batteryCtrlCfg.chargeCurrent > PMIC_TPS6594X_BB_CHARGING_CURRENT_500)
        {
            pmicStatus = PMIC_ST_ERR_INV_PARAM;
        }

        if (PMIC_ST_SUCCESS == pmicStatus)
        {
            /* Set Backup Battery charging current value*/
            pmicStatus = Pmic_setBackupBatteryChargingCurrentVal(pPmicCoreHandle, batteryCtrlCfg);
        }
    }

    return pmicStatus;
}

int32_t Pmic_getBatteryCtrlConfig(Pmic_CoreHandle_t *pPmicCoreHandle, Pmic_BatteryCtrlCfg_t *pBatteryCtrlCfg)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    if (NULL == pPmicCoreHandle)
    {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) && (NULL == pBatteryCtrlCfg))
    {
        pmicStatus = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) && (PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType))
    {
        pmicStatus = PMIC_ST_ERR_NOT_SUPPORTED;
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) &&
        ((bool)true == pmic_validParamCheck(pBatteryCtrlCfg->validParams, PMIC_CFG_CHARGING_EN_VALID)))
    {
        /* Get the status of Backup Battery Charging is Enabled/Disabled  */
        pmicStatus = Pmic_getBackupBatteryChargingCfg(pPmicCoreHandle, pBatteryCtrlCfg);
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) &&
        ((bool)true == pmic_validParamCheck(pBatteryCtrlCfg->validParams, PMIC_CFG_END_OF_CHARGE_VOLTAGE_VALID)))
    {
        /* Get Backup Battery configuration for End of charge Voltage*/
        pmicStatus = Pmic_getBackupBatteryEndOfChargeVoltage(pPmicCoreHandle, pBatteryCtrlCfg);
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) &&
        ((bool)true == pmic_validParamCheck(pBatteryCtrlCfg->validParams, PMIC_CFG_CHARGE_CURRENT_VALID)))
    {
        /* Get Backup Battery charging current value*/
        pmicStatus = Pmic_getBackupBatteryChargingCurrentVal(pPmicCoreHandle, pBatteryCtrlCfg);
    }

    return pmicStatus;
}

/*!
 * \brief   API to get NRSTOUT_SOC/ NRSTOUT/ EN_DRV Register Bit fields
 *          Note: In this API, the default pinType is assumed as EN_DRV. While
 *          adding support for New PMIC device, developer need to update the API
 *          functionality for New PMIC device accordingly.
 */
static void Pmic_getPinTypeRegBitFields(const uint8_t pinType, uint8_t *pBitShift, uint8_t *pBitMask)
{
    switch (pinType)
    {
        case PMIC_PIN_TYPE_NRSTOUT_SOC:
            *pBitShift = PMIC_ENABLE_DRV_STAT_NRSTOUT_SOC_IN_SHIFT;
            *pBitMask = PMIC_ENABLE_DRV_STAT_NRSTOUT_SOC_IN_MASK;
            break;
        case PMIC_PIN_TYPE_NRSTOUT:
            *pBitShift = PMIC_ENABLE_DRV_STAT_NRSTOUT_IN_SHIFT;
            *pBitMask = PMIC_ENABLE_DRV_STAT_NRSTOUT_IN_MASK;
            break;
        default:
            *pBitShift = PMIC_ENABLE_DRV_STAT_EN_DRV_IN_SHIFT;
            *pBitMask = PMIC_ENABLE_DRV_STAT_EN_DRV_IN_MASK;
            break;
    }
}

int32_t Pmic_getPinValue(Pmic_CoreHandle_t *pPmicCoreHandle, const uint8_t pinType, uint8_t *pPinValue)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regVal;
    uint8_t bitShift, bitMask;

    if (NULL == pPmicCoreHandle)
    {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) && (pinType > PMIC_PIN_TYPE_NRSTOUT))
    {
        pmicStatus = PMIC_ST_ERR_INV_PARAM;
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) && (NULL == pPinValue))
    {
        pmicStatus = PMIC_ST_ERR_NULL_PARAM;
    }

    if (PMIC_ST_SUCCESS == pmicStatus)
    {
        Pmic_criticalSectionStart(pPmicCoreHandle);
        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_ENABLE_DRV_STAT_REGADDR, &regVal);
        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    if (PMIC_ST_SUCCESS == pmicStatus)
    {
        Pmic_getPinTypeRegBitFields(pinType, &bitShift, &bitMask);

        *pPinValue = Pmic_getBitField(regVal, bitShift, bitMask);
    }

    return pmicStatus;
}

/*!
 * \brief  API to get the status of SPMI LPM Control is Enabled/Disabled
 */
static int32_t Pmic_getSpmiLpmCtrlCfg(Pmic_CoreHandle_t *pPmicCoreHandle, Pmic_CommonCtrlStat_t *pCommonCtrlStat)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);
    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_ENABLE_DRV_STAT_REGADDR, &regData);
    Pmic_criticalSectionStop(pPmicCoreHandle);

    if (PMIC_ST_SUCCESS == pmicStatus)
    {
        pCommonCtrlStat->spmiLpmStat = (bool)false;

        if (Pmic_getBitField(regData, PMIC_ENABLE_DRV_STAT_SPMI_LPM_EN_SHIFT, PMIC_ENABLE_DRV_STAT_SPMI_LPM_EN_MASK) ==
            1U)
        {
            pCommonCtrlStat->spmiLpmStat = (bool)true;
        }
    }

    return pmicStatus;
}

/*!
 * \brief  API to get the status of ENABLE_DRV Configuration by I2C/SPI is
 *         Enabled or not
 */
static int32_t Pmic_getEnableDrvI2CSPICfg(Pmic_CoreHandle_t *pPmicCoreHandle, Pmic_CommonCtrlStat_t *pCommonCtrlStat)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);
    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_ENABLE_DRV_STAT_REGADDR, &regData);
    Pmic_criticalSectionStop(pPmicCoreHandle);

    if (PMIC_ST_SUCCESS == pmicStatus)
    {
        pCommonCtrlStat->forceEnDrvLowStat = Pmic_getBitField(
            regData, PMIC_ENABLE_DRV_STAT_FORCE_EN_DRV_LOW_SHIFT, PMIC_ENABLE_DRV_STAT_FORCE_EN_DRV_LOW_MASK);
    }

    return pmicStatus;
}

/*!
 * \brief  API to get Backup Battery charging End of charge Indication Status
 */
static int32_t Pmic_getBackupBatteryEocIndicationStat(Pmic_CoreHandle_t     *pPmicCoreHandle,
                                                      Pmic_CommonCtrlStat_t *pCommonCtrlStat)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);
    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_CONFIG_2_REGADDR, &regData);
    Pmic_criticalSectionStop(pPmicCoreHandle);

    if (PMIC_ST_SUCCESS == pmicStatus)
    {
        pCommonCtrlStat->bbEndOfChargeIndication =
            Pmic_getBitField(regData, PMIC_CONFIG_2_BB_EOC_RDY_SHIFT, PMIC_CONFIG_2_BB_EOC_RDY_MASK);
    }

    return pmicStatus;
}

/*!
 * \brief  API to get Register Lock Status
 */
static int32_t Pmic_getRegLockStat(Pmic_CoreHandle_t *pPmicCoreHandle, Pmic_CommonCtrlStat_t *pCommonCtrlStat)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);
    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_REGISTER_LOCK_REGADDR, &regData);
    Pmic_criticalSectionStop(pPmicCoreHandle);

    if (PMIC_ST_SUCCESS == pmicStatus)
    {
        pCommonCtrlStat->regLockStat = Pmic_getBitField(
            regData, PMIC_REGISTER_LOCK_REGISTER_LOCK_STATUS_SHIFT, PMIC_REGISTER_LOCK_REGISTER_LOCK_STATUS_READ_MASK);
    }

    return pmicStatus;
}

/*!
 * \brief  API to get External Clock Validity Status
 */
static int32_t Pmic_getExtClkValidityStat(Pmic_CoreHandle_t *pPmicCoreHandle, Pmic_CommonCtrlStat_t *pCommonCtrlStat)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);
    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_STAT_MISC_REGADDR, &regData);
    Pmic_criticalSectionStop(pPmicCoreHandle);

    if (PMIC_ST_SUCCESS == pmicStatus)
    {
        pCommonCtrlStat->extClkValidity =
            Pmic_getBitField(regData, PMIC_STAT_MISC_EXT_CLK_STAT_SHIFT, PMIC_STAT_MISC_EXT_CLK_STAT_MASK);
    }

    return pmicStatus;
}

/*!
 * \brief  API to get Startup Pin Status
 */
static int32_t Pmic_getStartupPinStat(Pmic_CoreHandle_t *pPmicCoreHandle, Pmic_CommonCtrlStat_t *pCommonCtrlStat)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);
    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_STAT_STARTUP_REGADDR, &regData);
    Pmic_criticalSectionStop(pPmicCoreHandle);

    if (PMIC_ST_SUCCESS == pmicStatus)
    {
        pCommonCtrlStat->startupPin =
            Pmic_getBitField(regData, PMIC_STAT_STARTUP_ENABLE_STAT_SHIFT, PMIC_STAT_STARTUP_ENABLE_STAT_MASK);
    }

    return pmicStatus;
}

/*!
 * \brief  API to get EN_DRV Pin status
 */
static int32_t Pmic_getEnDrvPinStat(Pmic_CoreHandle_t *pPmicCoreHandle, Pmic_CommonCtrlStat_t *pCommonCtrlStat)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);
    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_STAT_READBACK_ERR_REGADDR, &regData);
    Pmic_criticalSectionStop(pPmicCoreHandle);

    if (PMIC_ST_SUCCESS == pmicStatus)
    {
        pCommonCtrlStat->enDrvPin = Pmic_getBitField(regData,
                                                     PMIC_STAT_READBACK_ERR_EN_DRV_READBACK_STAT_SHIFT,
                                                     PMIC_STAT_READBACK_ERR_EN_DRV_READBACK_STAT_MASK);
    }

    return pmicStatus;
}

/*!
 * \brief  API to get NRSTOUT_SOC Pin Status
 */
static int32_t Pmic_getNRstOutSocPinStat(Pmic_CoreHandle_t *pPmicCoreHandle, Pmic_CommonCtrlStat_t *pCommonCtrlStat)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);
    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_STAT_READBACK_ERR_REGADDR, &regData);
    Pmic_criticalSectionStop(pPmicCoreHandle);

    if (PMIC_ST_SUCCESS == pmicStatus)
    {
        pCommonCtrlStat->nRstOutSocPin = Pmic_getBitField(regData,
                                                          PMIC_STAT_READBACK_ERR_NRSTOUT_SOC_READBACK_STAT_SHIFT,
                                                          PMIC_STAT_READBACK_ERR_NRSTOUT_SOC_READBACK_STAT_MASK);
    }

    return pmicStatus;
}

/*!
 * \brief  API to get NRSTOUT Pin Status
 */
static int32_t Pmic_getNRstOutPinStat(Pmic_CoreHandle_t *pPmicCoreHandle, Pmic_CommonCtrlStat_t *pCommonCtrlStat)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);
    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_STAT_READBACK_ERR_REGADDR, &regData);
    Pmic_criticalSectionStop(pPmicCoreHandle);

    if (PMIC_ST_SUCCESS == pmicStatus)
    {
        pCommonCtrlStat->nRstOutPin = Pmic_getBitField(regData,
                                                       PMIC_STAT_READBACK_ERR_NRSTOUT_READBACK_STAT_SHIFT,
                                                       PMIC_STAT_READBACK_ERR_NRSTOUT_READBACK_STAT_MASK);
    }

    return pmicStatus;
}

/*!
 * \brief  API to get NINT Pin Status
 */
static int32_t Pmic_getNIntPinStat(Pmic_CoreHandle_t *pPmicCoreHandle, Pmic_CommonCtrlStat_t *pCommonCtrlStat)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);
    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_STAT_READBACK_ERR_REGADDR, &regData);
    Pmic_criticalSectionStop(pPmicCoreHandle);

    if (PMIC_ST_SUCCESS == pmicStatus)
    {
        pCommonCtrlStat->nIntPin = Pmic_getBitField(
            regData, PMIC_STAT_READBACK_ERR_NINT_READBACK_STAT_SHIFT, PMIC_STAT_READBACK_ERR_NINT_READBACK_STAT_MASK);
    }

    return pmicStatus;
}

/*!
 * \brief   API to get Startup Pin, EN_DRV Pin, NRSTOUT_SOC Pin, NRSTOUT Pin,
 *          and NINT Pin Status
 */
static int32_t Pmic_getStartupEndrvNrstoutsocNrstoutNintPinStat(Pmic_CoreHandle_t     *pPmicCoreHandle,
                                                                Pmic_CommonCtrlStat_t *pCommonCtrlStat)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    if ((bool)true == pmic_validParamCheck(pCommonCtrlStat->validParams, PMIC_CFG_STARTUP_PIN_STAT_VALID))
    {
        /* Get Startup Pin Status*/
        pmicStatus = Pmic_getStartupPinStat(pPmicCoreHandle, pCommonCtrlStat);
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) &&
        ((bool)true == pmic_validParamCheck(pCommonCtrlStat->validParams, PMIC_CFG_EN_DRV_PIN_STAT_VALID)))
    {
        /* Get EN_DRV Pin Status*/
        pmicStatus = Pmic_getEnDrvPinStat(pPmicCoreHandle, pCommonCtrlStat);
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) &&
        ((bool)true == pmic_validParamCheck(pCommonCtrlStat->validParams, PMIC_CFG_NRSTOUTSOC_PIN_STAT_VALID)))
    {
        /* Get NRSTOUT_SOC Pin Status*/
        pmicStatus = Pmic_getNRstOutSocPinStat(pPmicCoreHandle, pCommonCtrlStat);
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) &&
        ((bool)true == pmic_validParamCheck(pCommonCtrlStat->validParams, PMIC_CFG_NRSTOUT_PIN_STAT_VALID)))
    {
        /* Get NRSTOUT Pin Status*/
        pmicStatus = Pmic_getNRstOutPinStat(pPmicCoreHandle, pCommonCtrlStat);
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) &&
        ((bool)true == pmic_validParamCheck(pCommonCtrlStat->validParams, PMIC_CFG_NINT_PIN_STAT_VALID)))
    {
        /* Get NINT Pin Status*/
        pmicStatus = Pmic_getNIntPinStat(pPmicCoreHandle, pCommonCtrlStat);
    }

    return pmicStatus;
}

int32_t Pmic_getCommonCtrlStat(Pmic_CoreHandle_t *pPmicCoreHandle, Pmic_CommonCtrlStat_t *pCommonCtrlStat)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    if (NULL == pPmicCoreHandle)
    {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) && (NULL == pCommonCtrlStat))
    {
        pmicStatus = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) &&
        ((bool)true == pmic_validParamCheck(pCommonCtrlStat->validParams, PMIC_CFG_SPMI_LPM_STAT_VALID)))
    {
        /* Get the status of SPMI LPM Control is Enabled/Disabled  */
        pmicStatus = Pmic_getSpmiLpmCtrlCfg(pPmicCoreHandle, pCommonCtrlStat);
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) &&
        ((bool)true == pmic_validParamCheck(pCommonCtrlStat->validParams, PMIC_CFG_FORCE_ENABLE_DRV_LOW_STAT_VALID)))
    {
        /* Get the status of ENABLE_DRV Configuration by I2C/SPI is Enabled or not*/
        pmicStatus = Pmic_getEnableDrvI2CSPICfg(pPmicCoreHandle, pCommonCtrlStat);
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) &&
        ((bool)true == pmic_validParamCheck(pCommonCtrlStat->validParams, PMIC_CFG_BB_EOC_INDICATION_STAT_VALID)))
    {
        /* Get Backup Battery charging End of charge Indication Status*/
        pmicStatus = Pmic_getBackupBatteryEocIndicationStat(pPmicCoreHandle, pCommonCtrlStat);
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) &&
        ((bool)true == pmic_validParamCheck(pCommonCtrlStat->validParams, PMIC_CFG_REGISTER_LOCK_STAT_VALID)))
    {
        /* Get Register Lock Status*/
        pmicStatus = Pmic_getRegLockStat(pPmicCoreHandle, pCommonCtrlStat);
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) &&
        ((bool)true == pmic_validParamCheck(pCommonCtrlStat->validParams, PMIC_CFG_EXT_CLK_VALIDITY_STAT_VALID)))
    {
        /* Get External Clock Validity Status*/
        pmicStatus = Pmic_getExtClkValidityStat(pPmicCoreHandle, pCommonCtrlStat);
    }

    if (PMIC_ST_SUCCESS == pmicStatus)
    {
        /* Get Startup, EN_DRV, NRSTOUT_SOC, NRSTOUT and NINT Pin Status */
        pmicStatus = Pmic_getStartupEndrvNrstoutsocNrstoutNintPinStat(pPmicCoreHandle, pCommonCtrlStat);
    }

    return pmicStatus;
}

int32_t Pmic_getI2CSpeed(Pmic_CoreHandle_t *pPmicCoreHandle, uint8_t *pI2C1Speed, uint8_t *pI2C2Speed)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regVal;

    if (NULL == pPmicCoreHandle)
    {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) && ((NULL == pI2C1Speed) || (NULL == pI2C2Speed)))
    {
        pmicStatus = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) && (pPmicCoreHandle->commMode > PMIC_INTF_DUAL_I2C))
    {
        pmicStatus = PMIC_ST_ERR_INV_COMM_MODE;
    }

    if (PMIC_ST_SUCCESS == pmicStatus)
    {
        Pmic_criticalSectionStart(pPmicCoreHandle);
        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_CONFIG_1_REGADDR, &regVal);
        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    if (PMIC_ST_SUCCESS == pmicStatus)
    {
        *pI2C1Speed = Pmic_getBitField(regVal, PMIC_CONFIG_1_I2C1_HS_SHIFT, PMIC_CONFIG_1_I2C1_HS_MASK);

        if (PMIC_INTF_DUAL_I2C == pPmicCoreHandle->commMode)
        {
            *pI2C2Speed = Pmic_getBitField(regVal, PMIC_CONFIG_1_I2C2_HS_SHIFT, PMIC_CONFIG_1_I2C2_HS_MASK);
        }
    }

    return pmicStatus;
}

int32_t Pmic_enableCRC(Pmic_CoreHandle_t *pPmicCoreHandle)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    if (NULL == pPmicCoreHandle)
    {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    }

    if (PMIC_ST_SUCCESS == pmicStatus)
    {
        pmicStatus = Pmic_fsmEnableI2cTrigger(pPmicCoreHandle, PMIC_FSM_I2C_TRIGGER2, PMIC_FSM_I2C_TRIGGER_VAL_1);
    }

    if (PMIC_ST_SUCCESS == pmicStatus)
    {
        pPmicCoreHandle->crcEnable = (bool)true;
    }

    return pmicStatus;
}

int32_t Pmic_getCrcStatus(Pmic_CoreHandle_t *pPmicCoreHandle, uint8_t *pI2c1SpiCrcStatus, uint8_t *pI2c2CrcStatus)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regVal = 0U;

    if (NULL == pPmicCoreHandle)
    {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) && ((NULL == pI2c1SpiCrcStatus) || (NULL == pI2c2CrcStatus)))
    {
        pmicStatus = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) && (pPmicCoreHandle->commMode > PMIC_INTF_SPI))
    {
        pmicStatus = PMIC_ST_ERR_INV_COMM_MODE;
    }

    if (PMIC_ST_SUCCESS == pmicStatus)
    {
        Pmic_criticalSectionStart(pPmicCoreHandle);
        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_SERIAL_IF_CONFIG_REGADDR, &regVal);
        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    if (PMIC_ST_SUCCESS == pmicStatus)
    {
        *pI2c1SpiCrcStatus = Pmic_getBitField(
            regVal, PMIC_SERIAL_IF_CONFIG_I2C1_SPI_CRC_EN_SHIFT, PMIC_SERIAL_IF_CONFIG_I2C1_SPI_CRC_EN_MASK);

        if (PMIC_INTF_DUAL_I2C == pPmicCoreHandle->commMode)
        {
            *pI2c2CrcStatus = Pmic_getBitField(
                regVal, PMIC_SERIAL_IF_CONFIG_I2C2_CRC_EN_SHIFT, PMIC_SERIAL_IF_CONFIG_I2C2_CRC_EN_MASK);
        }
    }

    return pmicStatus;
}

int32_t Pmic_getDeviceInfo(Pmic_CoreHandle_t *pPmicCoreHandle, Pmic_DeviceInfo_t *pDeviceInfo)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regVal;

    if (NULL == pPmicCoreHandle)
    {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) && (NULL == pDeviceInfo))
    {
        pmicStatus = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) && ((PMIC_SILICON_REV_ID_PG_2_0 != pPmicCoreHandle->pmicDevSiliconRev) &&
                                            (PMIC_SILICON_REV_ID_PG_1_0 != pPmicCoreHandle->pmicDevSiliconRev)))
    {
        pmicStatus = PMIC_ST_ERR_INV_SILICON_REVISION;
    }

    if (PMIC_ST_SUCCESS == pmicStatus)
    {
        Pmic_criticalSectionStart(pPmicCoreHandle);

        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_DEV_REV_REGADDR, &regVal);
        if (PMIC_ST_SUCCESS == pmicStatus)
        {
            pDeviceInfo->deviceID = Pmic_getBitField(
                regVal, PMIC_DEV_REV_TI_DEVICE_ID_SILICON_REV_SHIFT, PMIC_DEV_REV_TI_DEVICE_ID_SILICON_REV_MASK);

            if (PMIC_SILICON_REV_ID_PG_2_0 == pPmicCoreHandle->pmicDevSiliconRev)
            {
                pDeviceInfo->deviceID = Pmic_getBitField(regVal,
                                                         PMIC_DEV_REV_TI_DEVICE_ID_PG_2_0_SILICON_REV_SHIFT,
                                                         PMIC_DEV_REV_TI_DEVICE_ID_PG_2_0_SILICON_REV_MASK);
            }

            pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_NVM_CODE_1_REGADDR, &(pDeviceInfo->nvmID));
        }

        if (PMIC_ST_SUCCESS == pmicStatus)
        {
            pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_NVM_CODE_2_REGADDR, &(pDeviceInfo->nvmRev));
        }

        if (PMIC_ST_SUCCESS == pmicStatus)
        {
            pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_MANUFACTURING_VER_REGADDR, &regVal);

            pDeviceInfo->siliconRev = Pmic_getBitField(
                regVal, PMIC_MANUFACTURING_VER_SILICON_REV_SHIFT, PMIC_MANUFACTURING_VER_SILICON_REV_MASK);
        }

        if ((PMIC_ST_SUCCESS == pmicStatus) && (PMIC_SILICON_REV_ID_PG_2_0 == pPmicCoreHandle->pmicDevSiliconRev))
        {
            pmicStatus =
                Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_CUSTOMER_NVM_ID_REG_REGADDR, &(pDeviceInfo->customNvmID));
        }

        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    return pmicStatus;
}

int32_t Pmic_setI2CSpeedCfg(Pmic_CoreHandle_t *pPmicCoreHandle)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regVal;

    if (NULL == pPmicCoreHandle)
    {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    }

    if (PMIC_ST_SUCCESS == pmicStatus)
    {
        Pmic_criticalSectionStart(pPmicCoreHandle);

        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_CONFIG_1_REGADDR, &regVal);
        if (PMIC_ST_SUCCESS == pmicStatus)
        {
            if ((PMIC_INTF_SINGLE_I2C == pPmicCoreHandle->commMode) ||
                (PMIC_INTF_DUAL_I2C == pPmicCoreHandle->commMode))
            {
                Pmic_setBitField(
                    &regVal, PMIC_CONFIG_1_I2C1_HS_SHIFT, PMIC_CONFIG_1_I2C1_HS_MASK, pPmicCoreHandle->i2c1Speed);
            }

            if (PMIC_INTF_DUAL_I2C == pPmicCoreHandle->commMode)
            {
                Pmic_setBitField(
                    &regVal, PMIC_CONFIG_1_I2C2_HS_SHIFT, PMIC_CONFIG_1_I2C2_HS_MASK, pPmicCoreHandle->i2c2Speed);
            }

            pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle, PMIC_CONFIG_1_REGADDR, regVal);
        }

        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    return pmicStatus;
}
