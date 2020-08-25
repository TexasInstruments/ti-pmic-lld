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
 *  \file  pmic_core.c
 *
 *  \brief This file contains PMIC generic driver APIs
 */

#include <pmic_core_priv.h>
#include <pmic_rtc_tps6594x_priv.h>
#include <pmic_fsm_priv.h>

static const Pmic_DevSubSysInfo_t pmicSubSysInfo[] =
{
     /* PMIC_DEV_LEO_TPS6594x */
    {
        .gpioEnable = (bool)true,
        .rtcEnable  = (bool)true,
        .wdgEnable  = (bool)true,
        .buckEnable = (bool)true,
        .ldoEnable  = (bool)true
    },
     /* PMIC_DEV_HERA_LP8764x */
    {
        .gpioEnable = (bool)true,
        .rtcEnable  = (bool)false,
        .wdgEnable  = (bool)true,
        .buckEnable = (bool)true,
        .ldoEnable  = (bool)false
    }
};

/*!
 * \brief  API to setup nSLEEP signals.
 *         This function setup nSLEEP signal bits with STARTUP_DEST
 * Which is common for all supported PMICs. This API needs to be called
 * at PMIC init before clearing Enable and Start-Up interrupts.
 *
 *  \param   pPmicCoreHandle  [IN]  PMIC Interface Handle
 */
int32_t Pmic_nSleepSignalsSetup(Pmic_CoreHandle_t *pPmicCoreHandle)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t srcRegAddr = 0U;
    uint8_t srcBitMask = 0U;
    uint8_t srcBitShift = 0U;
    uint8_t dstRegAddr = 0U;
    uint8_t regData = 0U;
    uint8_t regVal = 0U;

    if(NULL == pPmicCoreHandle)
    {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        switch(pPmicCoreHandle->pmicDeviceType)
        {
            case PMIC_DEV_LEO_TPS6594X:
                srcRegAddr = PMIC_RTC_CTRL_2_REGADDR;
                dstRegAddr = PMIC_FSM_NSLEEP_TRIGGERS_REGADDR;
                srcBitMask = PMIC_RTC_CTRL_2_STARTUP_DEST_MASK;
                srcBitShift = PMIC_RTC_CTRL_2_STARTUP_DEST_SHIFT;
                break;
             case PMIC_DEV_HERA_LP8764X:
                srcRegAddr = PMIC_STARTUP_CTRL_REGADDR;
                dstRegAddr = PMIC_FSM_NSLEEP_TRIGGERS_REGADDR;
                srcBitMask = PMIC_STARTUP_CTRL_STARTUP_DEST_MASK;
                srcBitShift = PMIC_STARTUP_CTRL_STARTUP_DEST_SHIFT;
                break;
             default:
                pmicStatus = PMIC_ST_ERR_INV_DEVICE;
                break;
        }
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        Pmic_criticalSectionStart(pPmicCoreHandle);

        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                            srcRegAddr,
                                            &regData);

        if(PMIC_ST_SUCCESS == pmicStatus)
        {
            regVal = (regData & srcBitMask) >> srcBitShift;

            pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                                dstRegAddr,
                                                &regData);
        }

        if(PMIC_ST_SUCCESS == pmicStatus)
        {
            HW_REG_SET_FIELD(regData,
                             PMIC_FSM_NSLEEP_TRIGGERS_NSLEEP2B,
                             (BIT_POS_GET_VAL(regVal, 1U)));
            HW_REG_SET_FIELD(regData,
                             PMIC_FSM_NSLEEP_TRIGGERS_NSLEEP1B,
                             (BIT_POS_GET_VAL(regVal, 0U)));
            pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                                dstRegAddr,
                                                regData);
        }

        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    return pmicStatus;
}

/*!
 * \brief  Checks the validParam bit position is set or not in validParamVal
 *         This function checks the given bit position is being set or not in
 *         the validParamVal argument
 *
 *  \param   validParamVal [IN]   Valid param value
 *  \param   bitPos        [IN]   bit position value
 *
 *  \retval  Return true if the given bit pos is set, else return false
 */
bool pmic_validParamCheck(uint32_t validParamVal, uint8_t bitPos)
{
    bool retVal = (bool)false;

    retVal = ((validParamVal  >> bitPos) & 0x01U);

    return retVal;
}

/*!
 * \brief  Function call wrapper to lock PMIC LLD critical section
 *         This function locks to critical area by calling registred locking
 *         mechanism using pmic core handle.
 *
 *  \param   pPmicCoreHandle  [IN]  PMIC Interface Handle
 */
void Pmic_criticalSectionStart(const Pmic_CoreHandle_t *pPmicCoreHandle)
{
    if(NULL != pPmicCoreHandle->pFnPmicCritSecStart)
    {
        pPmicCoreHandle->pFnPmicCritSecStart();
    }
}

/*!
 * \brief  Function call wrapper to unlock PMIC LLD critical section
 *         This function unlocks to critical area by calling registred locking
 *         mechanism using pmic core handle.
 *
 *  \param   pPmicCoreHandle  [IN]  PMIC Interface Handle
 */
void Pmic_criticalSectionStop(const Pmic_CoreHandle_t *pPmicCoreHandle)
{
    if(NULL != pPmicCoreHandle->pFnPmicCritSecStart)
    {
        pPmicCoreHandle->pFnPmicCritSecStop();
    }
}

/*!
 * \brief  API to Set Recovery Counter Configuration.
 *         This function configures PMIC Recovery Counter register, controlling
 *         recovery count Threshold and Clear, when corresponding validParam
 *         bit field is set in the Pmic_RecovCntCfg_t structure.
 *         For more information \ref Pmic_RecovCntCfg_t
 *
 * \param   pPmicCoreHandle   [IN]    PMIC Interface Handle.
 * \param   recovCntCfg       [IN]    Set configuration value for
 *                                    Recovery counter
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_SetRecoveryCntCfg(Pmic_CoreHandle_t  *pPmicCoreHandle,
                               Pmic_RecovCntCfg_t  recovCntCfg)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t  regVal;

    if(NULL == pPmicCoreHandle)
    {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (((bool)true) == pmic_validParamCheck(recovCntCfg.validParams,
                                     PMIC_CFG_RECOV_CNT_THR_VAL_VALID)) &&
       ((recovCntCfg.thrVal > PMIC_RECOV_CNT_THR_MAX)))
    {
        pmicStatus = PMIC_ST_ERR_INV_PARAM;
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (((bool)true) == pmic_validParamCheck(recovCntCfg.validParams,
                                     PMIC_CFG_RECOV_CNT_CLR_CNT_VALID)) &&
       (recovCntCfg.clrCnt != 1U))
    {
        pmicStatus = PMIC_ST_ERR_INV_PARAM;
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        /* Start Critical Section */
        Pmic_criticalSectionStart(pPmicCoreHandle);

        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                            PMIC_RECOV_CNT_REG_2_REGADDR,
                                            &regVal);

        if(PMIC_ST_SUCCESS == pmicStatus)
        {
            if(((bool)true) == pmic_validParamCheck(recovCntCfg.validParams,
                                            PMIC_CFG_RECOV_CNT_CLR_CNT_VALID))
            {
                HW_REG_SET_FIELD(regVal,
                                 PMIC_RECOV_CNT_REG_2_RECOV_CNT_CLR,
                                 recovCntCfg.clrCnt);
            }

            if(((bool)true) == pmic_validParamCheck(recovCntCfg.validParams,
                                            PMIC_CFG_RECOV_CNT_THR_VAL_VALID))
            {
                HW_REG_SET_FIELD(regVal,
                                 PMIC_RECOV_CNT_REG_2_RECOV_CNT_CLR,
                                 recovCntCfg.thrVal);
            }

            pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                                PMIC_RECOV_CNT_REG_2_REGADDR,
                                                regVal);
        }

        /* Stop Critical Section */
        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    return pmicStatus;
}

/*!
 * \brief  API to Get Recovery Counter Configuration.
 *         This function gets PMIC Recovery Counter configuration values. Like,
 *         recovery count Threshold and Clear, when corresponding validParam
 *         bit field is set in the Pmic_RecovCntCfg_t structure.
 *         For more information \ref Pmic_RecovCntCfg_t
 *
 * \param   pPmicCoreHandle       [IN]    PMIC Interface Handle.
 * \param   pRecovCntCfg          [OUT]   Pointer to store recovery counter
 *                                        configuration value
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_getRecoveryCntCfg(Pmic_CoreHandle_t *pPmicCoreHandle,
                               Pmic_RecovCntCfg_t *pRecovCntCfg)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t  regVal = 0U;

    if(NULL == pPmicCoreHandle)
    {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    }

    if((PMIC_ST_SUCCESS == pmicStatus) && (NULL == pRecovCntCfg))
    {
        pmicStatus = PMIC_ST_ERR_NULL_PARAM;
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {

        /* Start Critical Section */
        Pmic_criticalSectionStart(pPmicCoreHandle);

        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                            PMIC_RECOV_CNT_REG_2_REGADDR,
                                            &regVal);
        /* Stop Critical Section */
        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
            if(((bool)true) == pmic_validParamCheck(pRecovCntCfg->validParams,
                                            PMIC_CFG_RECOV_CNT_THR_VAL_VALID))
            {
                pRecovCntCfg->thrVal = HW_REG_GET_FIELD(regVal,
                                          PMIC_RECOV_CNT_REG_2_RECOV_CNT_THR);
            }
            if(((bool)true) == pmic_validParamCheck(pRecovCntCfg->validParams,
                                            PMIC_CFG_RECOV_CNT_CLR_CNT_VALID))
            {
                pRecovCntCfg->clrCnt = HW_REG_GET_FIELD(regVal,
                                             PMIC_RECOV_CNT_REG_2_RECOV_CNT_CLR);
            }
    }

    return pmicStatus;
}

/*!
 * \brief  API to Read Recovery Count Value.
 *         This function reads out the recovery count value.
 *
 * \param   pPmicCoreHandle       [IN]    PMIC Interface Handle.
 * \param   pRecovCntVal          [OUT]   Pointer to store recovery count
 *                                        value
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_getRecoveryCnt(Pmic_CoreHandle_t *pPmicCoreHandle,
                            uint8_t           *pRecovCntVal)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regVal = 0U;

    if(NULL == pPmicCoreHandle)
    {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    }

    if((PMIC_ST_SUCCESS == pmicStatus) && (NULL == pRecovCntVal))
    {
        pmicStatus = PMIC_ST_ERR_NULL_PARAM;
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        /* Start Critical Section */
        Pmic_criticalSectionStart(pPmicCoreHandle);

        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                            PMIC_RECOV_CNT_REG_1_REGADDR,
                                            &regVal);

        /* Stop Critical Section */
        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        *pRecovCntVal = HW_REG_GET_FIELD(regVal,
                                         PMIC_RECOV_CNT_REG_1_RECOV_CNT);
    }

    return pmicStatus;
}

/*!
 * \brief  API to Initialize pmic core handle for PMIC LLD.
 *         This function gets device configuration from pCoreCfgHandle and
 *         initializes device specific information in pPmicCoreHandle after
 *         validation of given params depends on validParams bitfileds
 *         and does some basic validation on PMIC interface I2C/SPI,
 *         confirming that PMIC is accessible for PMIC configuration and
 *         monitor features.
 *
 *  \param   pPmicConfigData [IN]   PMIC Configuration data
 *  \param   pPmicCoreHandle [OUT]  PMIC Interface Handle.
 *
 *  \retval  PMIC_ST_SUCCESS in case of success or appropriate error code
 *           For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_init(const Pmic_CoreCfg_t *pPmicConfigData,
                  Pmic_CoreHandle_t    *pPmicCoreHandle)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t  regVal = 0U;

    if((NULL == pPmicCoreHandle) || (NULL == pPmicConfigData))
    {
        pmicStatus = PMIC_ST_ERR_NULL_PARAM;
    }

    /* Check and update PMIC Handle device type */
    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (((bool)true) == pmic_validParamCheck(pPmicConfigData->validParams,
                                           PMIC_CFG_DEVICE_TYPE_VALID)))
    {
       if((PMIC_DEV_HERA_LP8764X != pPmicConfigData->pmicDeviceType) &&
          (PMIC_DEV_LEO_TPS6594X != pPmicConfigData->pmicDeviceType))
        {
            pmicStatus = PMIC_ST_ERR_INV_PARAM;
        }

        if(PMIC_ST_SUCCESS == pmicStatus)
        {
            pPmicCoreHandle->pmicDeviceType = pPmicConfigData->pmicDeviceType;
        }
    }

    /* Check and update PMIC Handle Comm Mode */
    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (((bool)true) == pmic_validParamCheck(pPmicConfigData->validParams,
                                           PMIC_CFG_COMM_MODE_VALID)))
    {
        if((PMIC_INTF_SINGLE_I2C != pPmicConfigData->commMode) &&
           (PMIC_INTF_DUAL_I2C != pPmicConfigData->commMode)   &&
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
    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (((bool)true) == pmic_validParamCheck(pPmicConfigData->validParams,
                                     PMIC_CFG_SLAVEADDR_VALID)))
    {
        if((PMIC_INTF_SINGLE_I2C == pPmicCoreHandle->commMode) ||
           (PMIC_INTF_DUAL_I2C == pPmicCoreHandle->commMode))
        {
            pPmicCoreHandle->slaveAddr = pPmicConfigData->slaveAddr;
        }
    }

    /* Check and update PMIC Handle QA Slave Address */
    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (((bool)true) == pmic_validParamCheck(pPmicConfigData->validParams,
                                     PMIC_CFG_QASLAVEADDR_VALID)))
    {
        if((PMIC_INTF_SINGLE_I2C == pPmicCoreHandle->commMode) ||
           (PMIC_INTF_DUAL_I2C == pPmicCoreHandle->commMode))
        {
            pPmicCoreHandle->qaSlaveAddr = pPmicConfigData->qaSlaveAddr;
        }
    }

    /* Check and update PMIC Handle CRC Enable */
    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (((bool)true) == pmic_validParamCheck(pPmicConfigData->validParams,
                                     PMIC_CFG_CRC_ENABLE_VALID)))
    {
        pPmicCoreHandle->crcEnable = pPmicConfigData->crcEnable;
    }

    /* Check and update PMIC Handle Main Comm Handle */
    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (((bool)true) == pmic_validParamCheck(pPmicConfigData->validParams,
                                     PMIC_CFG_COMM_HANDLE_VALID)))
    {
        if(NULL == pPmicConfigData->pCommHandle)
        {
            pmicStatus = PMIC_ST_ERR_NULL_FPTR;
        }
        else
        {
            pPmicCoreHandle->pCommHandle = pPmicConfigData->pCommHandle;
        }
    }

    /* Check and update PMIC Handle QA Comm Handle */
    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (((bool)true) == pmic_validParamCheck(pPmicConfigData->validParams,
                                     PMIC_CFG_QACOMM_HANDLE_VALID)))
    {
        if(NULL == pPmicConfigData->pQACommHandle)
        {
            pmicStatus = PMIC_ST_ERR_NULL_FPTR;
        }
        else
        {
            if(PMIC_INTF_DUAL_I2C == pPmicCoreHandle->commMode)
            {
                pPmicCoreHandle->pQACommHandle = pPmicConfigData->pQACommHandle;
            }
        }
    }

    /* Check and update PMIC Handle Comm IO RD Fn */
    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (((bool)true) == pmic_validParamCheck(pPmicConfigData->validParams,
                                     PMIC_CFG_COMM_IO_RD_VALID)))
    {
        if(NULL == pPmicConfigData->pFnPmicCommIoRead)
        {
            pmicStatus = PMIC_ST_ERR_NULL_FPTR;
        }
        else
        {
            pPmicCoreHandle->pFnPmicCommIoRead =
                             pPmicConfigData->pFnPmicCommIoRead;
        }
    }

    /* Check and update PMIC Handle Comm IO WR Fn */
    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (((bool)true) == pmic_validParamCheck(pPmicConfigData->validParams,
                                     PMIC_CFG_COMM_IO_WR_VALID)))
    {
        if(NULL == pPmicConfigData->pFnPmicCommIoWrite)
        {
            pmicStatus = PMIC_ST_ERR_NULL_FPTR;
        }
        else
        {
            pPmicCoreHandle->pFnPmicCommIoWrite =
                             pPmicConfigData->pFnPmicCommIoWrite;
        }
    }

    /* Check and update PMIC Handle Critical Section Start Fn */
    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (((bool)true) == pmic_validParamCheck(pPmicConfigData->validParams,
                                     PMIC_CFG_CRITSEC_START_VALID)))
    {
        if(NULL == pPmicConfigData->pFnPmicCritSecStart)
        {
            pmicStatus = PMIC_ST_ERR_NULL_FPTR;
        }
        else
        {
            pPmicCoreHandle->pFnPmicCritSecStart =
                             pPmicConfigData->pFnPmicCritSecStart;
        }
    }

    /* Check and update PMIC Handle Critical Section Stop Fn */
    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (((bool)true) == pmic_validParamCheck(pPmicConfigData->validParams,
                                     PMIC_CFG_CRITSEC_STOP_VALID)))
    {
        if(NULL == pPmicConfigData->pFnPmicCritSecStart)
        {
            pmicStatus = PMIC_ST_ERR_NULL_FPTR;
        }
        else
        {
            pPmicCoreHandle->pFnPmicCritSecStop =
                             pPmicConfigData->pFnPmicCritSecStop;
        }
    }

    /* Check for required members for I2C/SPI Main handle comm */
    if((PMIC_ST_SUCCESS == pmicStatus) &&
       ((NULL == pPmicCoreHandle->pCommHandle)          ||
        (NULL == pPmicCoreHandle->pFnPmicCritSecStart)  ||
        (NULL == pPmicCoreHandle->pFnPmicCritSecStop)   ||
        (NULL == pPmicCoreHandle->pFnPmicCommIoRead)    ||
        (NULL == pPmicCoreHandle->pFnPmicCommIoWrite)))
    {
        pmicStatus = PMIC_ST_ERR_INSUFFICIENT_CFG;
    }

    /* Update PMIC subsystem info to PMIC handle */
    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        pPmicCoreHandle->pPmic_SubSysInfo =
                         (Pmic_DevSubSysInfo_t *)
                         (&pmicSubSysInfo[pPmicCoreHandle->pmicDeviceType]);
    }

    /* Check the Main communication interface if PMIC handle is ready for rw */
    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (0x0U == pPmicCoreHandle->drvInitStatus))
    {
        /* Start Critical Section */
        Pmic_criticalSectionStart(pPmicCoreHandle);

        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                            PMIC_DEV_REV_REGADDR,
                                            &regVal);

        Pmic_criticalSectionStop(pPmicCoreHandle);

        if(PMIC_ST_SUCCESS == pmicStatus)
        {
            /* Validate if the device requested is the one on the bus */
            switch (pPmicCoreHandle->pmicDeviceType)
            {
                case PMIC_DEV_HERA_LP8764X:
                    if(PMIC_HERA_DEV_REV_ID != regVal)
                    {
                        pmicStatus = PMIC_ST_ERR_INV_DEVICE;
                    }
                    break;
                case PMIC_DEV_LEO_TPS6594X:
                    if(PMIC_LEO_DEV_REV_ID != regVal)
                    {
                        pmicStatus = PMIC_ST_ERR_INV_DEVICE;
                    }
                    break;
                default:
                    pmicStatus = PMIC_ST_ERR_INV_DEVICE;
                    break;
            }
        }

        if(PMIC_ST_SUCCESS == pmicStatus)
        {
            pPmicCoreHandle->drvInitStatus = DRV_INIT_SUCCESS |
                                             pPmicConfigData->instType;
        }
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (NULL != pPmicCoreHandle->pQACommHandle) &&
       (0x0U != pPmicCoreHandle->drvInitStatus))
    {
        /* Start Critical Section */
        Pmic_criticalSectionStart(pPmicCoreHandle);

        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                            PMIC_WDG_LONGWIN_CFG_REGADDR,
                                            &regVal);

        Pmic_criticalSectionStop(pPmicCoreHandle);

        if(PMIC_ST_SUCCESS == pmicStatus)
        {
            pPmicCoreHandle->drvInitStatus |= pPmicConfigData->instType;
        }
    }

    return pmicStatus;
}

/*!
 * \brief  API to DeInitilizes an existing PMIC Instance.
 *         This function takes an existing Instance pPmicCoreHandle and
 *         closes the LLD being used for this Instance. It should be called
 *         only once per valid pPmicCoreHandle. Should not be called
 *         if Pmic_init() is not called.
 *
 *  \param   pPmicCoreHandle  [IN] PMIC Interface Handle.
 *
 *  \retval  PMIC_ST_SUCCESS in case of success or appropriate error code
 *           For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_deinit(Pmic_CoreHandle_t  *pPmicCoreHandle)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    if(NULL == pPmicCoreHandle)
    {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
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

static int32_t Pmic_getScratchPadRegAddr(uint8_t  scratchPadRegId,
                                         uint8_t *pRegAddr)
{
    int32_t status = PMIC_ST_SUCCESS;

    switch(scratchPadRegId)
    {
        case PMIC_SCRATCH_PAD_REG_1 :
            *pRegAddr = PMIC_SCRATCH_PAD_REG_1_REGADDR;
            break;
        case PMIC_SCRATCH_PAD_REG_2 :
            *pRegAddr = PMIC_SCRATCH_PAD_REG_2_REGADDR;
            break;
        case PMIC_SCRATCH_PAD_REG_3 :
            *pRegAddr = PMIC_SCRATCH_PAD_REG_3_REGADDR;
            break;
        case PMIC_SCRATCH_PAD_REG_4 :
            *pRegAddr = PMIC_SCRATCH_PAD_REG_4_REGADDR;
            break;
        default:
            status = PMIC_ST_ERR_INV_PARAM;
            break;
    }

    return status;
}

/*!
 * \brief   API to set/write value in/to scratchpad register.
 *          This function is used write data to scratchpad register of PMIC
 *
 * \param   pPmicCoreHandle    [IN]    PMIC Interface Handle.
 * \param   scratchPadRegNum   [IN]    ScratchPad register number
 *                                     \ref Pmic_ScratchPad_Sel
 * \param   data               [IN]    Data/Value to be written to scratchpad.
 *
 * \return  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_setScratchPadValue(Pmic_CoreHandle_t *pPmicCoreHandle,
                                uint8_t            scratchPadRegId,
                                uint8_t            data)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regAddr;

    if(NULL == pPmicCoreHandle)
    {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (scratchPadRegId > PMIC_SCRATCH_PAD_REG_4))
    {
        pmicStatus = PMIC_ST_ERR_INV_PARAM;
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        pmicStatus = Pmic_getScratchPadRegAddr(scratchPadRegId, &regAddr);
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        Pmic_criticalSectionStart(pPmicCoreHandle);
        pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle, regAddr, data);
        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    return pmicStatus;
}

/*!
 * \brief   API to get/read data from scratchpad register.
 *          This function is used read data from scratchpad register of PMIC
 *
 * \param   pPmicCoreHandle    [IN]    PMIC Interface Handle.
 * \param   scratchPadRegNum   [IN]    ScratchPad register number
 *                                     \ref Pmic_ScratchPad_Sel
 * \param   data               [OUT]   Parameter to hold the Data/Value read
 *                                     from scratchpad.
 *
 * \return  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_getScratchPadValue(Pmic_CoreHandle_t *pPmicCoreHandle,
                                uint8_t            scratchPadRegId,
                                uint8_t            *pData)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regAddr;

    if(NULL == pPmicCoreHandle)
    {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (scratchPadRegId > PMIC_SCRATCH_PAD_REG_4))
    {
        pmicStatus = PMIC_ST_ERR_INV_PARAM;
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (NULL == pData))
    {
        pmicStatus = PMIC_ST_ERR_NULL_PARAM;
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        pmicStatus = Pmic_getScratchPadRegAddr(scratchPadRegId, &regAddr);
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        Pmic_criticalSectionStart(pPmicCoreHandle);
        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, regAddr, pData);
        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    return pmicStatus;
}
