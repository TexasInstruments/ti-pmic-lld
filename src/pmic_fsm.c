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
*   \file    pmic_fsm.c
*
*   \brief   This file contains the default API's for PMIC FSM state
*            configuration
*
*/

#include <pmic_types.h>
#include <pmic_fsm.h>

#include <pmic_core_priv.h>
#include <pmic_io_priv.h>
#include <pmic_fsm_priv.h>

#include <pmic_rtc_tps6594x_priv.h>

/*!
 * \brief   This function is used to get the regAddr, mask and shift values to
 *          configure standBy/LPStandby State
 *
 *          Note: In this API, the default PMIC device is assumed as TPS6594x
 *                LEO PMIC. While adding support for New PMIC device, developer
 *                need to update the API functionality for New PMIC device
 *                accordingly.
 */
static void Pmic_fsmGetstandByCfgRegFields(uint8_t  pmicDeviceType,
                                           uint8_t  *pRegAddr,
                                           uint8_t  *pBitPos,
                                           uint8_t  *pBitMask)
{
    switch(pmicDeviceType)
    {
        case PMIC_DEV_HERA_LP8764X:
            *pRegAddr = PMIC_STARTUP_CTRL_REGADDR;
            *pBitPos = PMIC_STARTUP_CTRL_LP_STANDBY_SEL_SHIFT;
            *pBitMask = PMIC_STARTUP_CTRL_LP_STANDBY_SEL_MASK;
            break;
        default:
            /* Default case is valid only for TPS6594x LEO PMIC */
            *pRegAddr = PMIC_RTC_CTRL_2_REGADDR;
            *pBitPos = PMIC_RTC_CTRL_2_LP_STANDBY_SEL_SHIFT;
            *pBitMask = PMIC_RTC_CTRL_2_LP_STANDBY_SEL_MASK;
            break;
    }

}

/*!
 * \brief   This function is used to get the required BitPos and Mask of
 *          NSLEEP Mask Signals.
 */
static void Pmic_fsmGetNsleepMaskBitField(bool     nsleepType,
                                          uint8_t *pBitPos,
                                          uint8_t *pBitMask)
{
    if(nsleepType == PMIC_NSLEEP1_SIGNAL)
    {
        *pBitPos  = PMIC_CONFIG_1_NSLEEP1_MASK_SHIFT;
        *pBitMask = PMIC_CONFIG_1_NSLEEP1_MASK_MASK;
    }

    if(nsleepType == PMIC_NSLEEP2_SIGNAL)
    {
        *pBitPos  = PMIC_CONFIG_1_NSLEEP2_MASK_SHIFT;
        *pBitMask = PMIC_CONFIG_1_NSLEEP2_MASK_MASK;
    }
}

/*!
 * \brief   This function is used to get the required BitPos and Mask of
 *          FSM I2C Trigger Type .
 */
static void Pmic_fsmGetI2cTriggerMaskBitField(uint8_t  i2cTriggerType,
                                              uint8_t *pBitPos,
                                              uint8_t *pBitMask)
{
    switch(i2cTriggerType)
    {
        case PMIC_FSM_I2C_TRIGGER1:
            *pBitPos = PMIC_FSM_I2C_TRIGGERS_TRIGGER_I2C_1_SHIFT;
            *pBitMask  = PMIC_FSM_I2C_TRIGGERS_TRIGGER_I2C_1_MASK;
            break;
        case PMIC_FSM_I2C_TRIGGER2:
            *pBitPos = PMIC_FSM_I2C_TRIGGERS_TRIGGER_I2C_2_SHIFT;
            *pBitMask  = PMIC_FSM_I2C_TRIGGERS_TRIGGER_I2C_2_MASK;
            break;
        case PMIC_FSM_I2C_TRIGGER3:
            *pBitPos = PMIC_FSM_I2C_TRIGGERS_TRIGGER_I2C_3_SHIFT;
            *pBitMask  = PMIC_FSM_I2C_TRIGGERS_TRIGGER_I2C_3_MASK;
            break;
        case PMIC_FSM_I2C_TRIGGER4:
            *pBitPos = PMIC_FSM_I2C_TRIGGERS_TRIGGER_I2C_4_SHIFT;
            *pBitMask  = PMIC_FSM_I2C_TRIGGERS_TRIGGER_I2C_4_MASK;
            break;
        case PMIC_FSM_I2C_TRIGGER5:
            *pBitPos = PMIC_FSM_I2C_TRIGGERS_TRIGGER_I2C_5_SHIFT;
            *pBitMask  = PMIC_FSM_I2C_TRIGGERS_TRIGGER_I2C_5_MASK;
            break;
        case PMIC_FSM_I2C_TRIGGER6:
            *pBitPos = PMIC_FSM_I2C_TRIGGERS_TRIGGER_I2C_6_SHIFT;
            *pBitMask  = PMIC_FSM_I2C_TRIGGERS_TRIGGER_I2C_6_MASK;
            break;
        case PMIC_FSM_I2C_TRIGGER7:
            *pBitPos = PMIC_FSM_I2C_TRIGGERS_TRIGGER_I2C_7_SHIFT;
            *pBitMask  = PMIC_FSM_I2C_TRIGGERS_TRIGGER_I2C_7_MASK;
            break;
        default:
            *pBitPos = PMIC_FSM_I2C_TRIGGERS_TRIGGER_I2C_0_SHIFT;
            *pBitMask  = PMIC_FSM_I2C_TRIGGERS_TRIGGER_I2C_0_MASK;
            break;
    }
}

/*!
 * \brief   This function is used to set S2R/Deep sleep FSM Misson state.
 */
static int32_t Pmic_setS2RState(Pmic_CoreHandle_t *pPmicCoreHandle)
{
    int32_t status  = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;


    Pmic_criticalSectionStart(pPmicCoreHandle);
    status = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                    PMIC_FSM_NSLEEP_TRIGGERS_REGADDR,
                                    &regData);

    if(PMIC_ST_SUCCESS == status)
    {
        Pmic_setBitField(&regData,
                         PMIC_FSM_NSLEEP_TRIGGERS_NSLEEP1B_SHIFT,
                         (PMIC_FSM_NSLEEP_TRIGGERS_NSLEEP2B_MASK |
                         PMIC_FSM_NSLEEP_TRIGGERS_NSLEEP1B_MASK),
                         PMIC_FSM_NSLEEPX_RESET);
        status = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                        PMIC_FSM_NSLEEP_TRIGGERS_REGADDR,
                                        regData);
    }

    Pmic_criticalSectionStop(pPmicCoreHandle);

    return status;
}

/*!
 * \brief   API to set PMIC Nsleep1B/2B Signal value.
 *
 * Requirement: REQ_TAG(PDK-9146)
 * Design: did_pmic_fsm_cfg_readback
 * Architecture: aid_pmic_fsm_cfg
 *
 *          This function is used to configure the Nsleep1B/2B signal level
 *
 * \param   pPmicCoreHandle  [IN]   PMIC Interface Handle.
 * \param   nsleepType       [IN]   NSLEEP signal
 *                                  Valid values: \ref Pmic_Nsleep_Signals
 * \param   nsleepVal        [IN]   PMIC Nsleep signal level High/Low to be
 *                                  configured.
 *                                  Valid values \ref Pmic_Nsleep_SignalLvl.
 *
 * \return  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_fsmSetNsleepSignalVal(Pmic_CoreHandle_t *pPmicCoreHandle,
                                   const bool         nsleepType,
                                   const uint8_t      nsleepVal)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    uint8_t bitPos  = 0U;
    uint8_t bitMask = 0U;

    if(NULL == pPmicCoreHandle)
    {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (nsleepVal > PMIC_NSLEEP_HIGH))
    {
        pmicStatus = PMIC_ST_ERR_INV_PARAM;
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        if(PMIC_NSLEEP1_SIGNAL == nsleepType)
        {
            bitPos  = PMIC_FSM_NSLEEP_TRIGGERS_NSLEEP1B_SHIFT;
            bitMask = PMIC_FSM_NSLEEP_TRIGGERS_NSLEEP1B_MASK;
        }
        if(PMIC_NSLEEP2_SIGNAL == nsleepType)
        {
            bitPos  = PMIC_FSM_NSLEEP_TRIGGERS_NSLEEP2B_SHIFT;
            bitMask = PMIC_FSM_NSLEEP_TRIGGERS_NSLEEP2B_MASK;
        }

        Pmic_criticalSectionStart(pPmicCoreHandle);
        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                            PMIC_FSM_NSLEEP_TRIGGERS_REGADDR,
                                            &regData);

        if(PMIC_ST_SUCCESS == pmicStatus)
        {
            Pmic_setBitField(&regData, bitPos, bitMask, nsleepVal);
            pmicStatus = Pmic_commIntf_sendByte(
                                            pPmicCoreHandle,
                                            PMIC_FSM_NSLEEP_TRIGGERS_REGADDR,
                                            regData);
        }

        Pmic_criticalSectionStop(pPmicCoreHandle);

    }

    return pmicStatus;
}

/*!
 * \brief   API to get PMIC Nsleep1B/2B Signal value.
 *
 * Requirement: REQ_TAG(PDK-9146)
 * Design: did_pmic_fsm_cfg_readback
 * Architecture: aid_pmic_fsm_cfg
 *
 *          This function is used to read the signal level of the Nsleep1B/2B
 *          signal
 *
 * \param   pPmicCoreHandle [IN]    PMIC Interface Handle
 * \param   nsleepType      [IN]    NSLEEP signal
 *                                  Valid values: \ref Pmic_Nsleep_Signals
 * \param   pNsleepVal      [OUT]   Pointer to store PMIC Nsleep signal level
 *                                  High/Low.
 *                                  Valid values \ref Pmic_Nsleep_SignalLvl
 *
 * \return  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_fsmGetNsleepSignalVal(Pmic_CoreHandle_t *pPmicCoreHandle,
                                   const bool         nsleepType,
                                   uint8_t           *pNsleepVal)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    uint8_t bitPos  = 0U;
    uint8_t bitMask = 0U;

    if(NULL == pPmicCoreHandle)
    {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    }

    if((PMIC_ST_SUCCESS == pmicStatus) && (NULL == pNsleepVal))
    {
        pmicStatus = PMIC_ST_ERR_NULL_PARAM;
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        if(PMIC_NSLEEP1_SIGNAL == nsleepType)
        {
            bitPos  = PMIC_FSM_NSLEEP_TRIGGERS_NSLEEP1B_SHIFT;
            bitMask = PMIC_FSM_NSLEEP_TRIGGERS_NSLEEP1B_MASK;
        }
        if(PMIC_NSLEEP2_SIGNAL == nsleepType)
        {
            bitPos  = PMIC_FSM_NSLEEP_TRIGGERS_NSLEEP2B_SHIFT;
            bitMask = PMIC_FSM_NSLEEP_TRIGGERS_NSLEEP2B_MASK;
        }

        Pmic_criticalSectionStart(pPmicCoreHandle);
        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                            PMIC_FSM_NSLEEP_TRIGGERS_REGADDR,
                                            &regData);
        Pmic_criticalSectionStop(pPmicCoreHandle);

        if(PMIC_ST_SUCCESS == pmicStatus)
        {
            *pNsleepVal = Pmic_getBitField(regData, bitPos, bitMask);

        }

    }

    return pmicStatus;

}

/*!
 * \brief   API to initiate FSM I2C trigger for given FSM I2C trigger type
 *
 * Requirement: REQ_TAG(PDK-9330)
 * Design: did_pmic_fsm_i2c_trigger
 * Architecture: aid_pmic_fsm_cfg
 *
 *          This function is used to to initiate FSM I2C trigger for given FSM
 *          I2C trigger type
 *          Note: In this API, the default i2cTriggerType is assumed as
 *                PMIC_FSM_I2C_TRIGGER0. While adding support for New PMIC
 *                device, developer need to update the API functionality for
 *                New PMIC device accordingly.
 *
 * \param   pPmicCoreHandle [IN]    PMIC Interface Handle
 * \param   i2cTriggerType  [IN]    FSM I2C Trigger Type
 *                                  Valid values: \ref Pmic_Fsm_I2c_Trigger_Type
 * \param   i2cTriggerVal   [IN]    FSM I2C Trigger Value
 *                                  Valid values: \ref Pmic_Fsm_I2c_Trigger_Val
 *
 * \return  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_fsmEnableI2cTrigger(Pmic_CoreHandle_t  *pPmicCoreHandle,
                                 const uint8_t       i2cTriggerType,
                                 const uint8_t       i2cTriggerVal)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData  = 0U;
    uint8_t bitPos = 0;
    uint8_t bitMask  = 0;

    if(NULL == pPmicCoreHandle)
    {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (PMIC_FSM_I2C_TRIGGER3 == i2cTriggerType))
    {
        pmicStatus = PMIC_ST_ERR_NOT_SUPPORTED;
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       ((PMIC_SILICON_REV_ID_PG_2_0 != pPmicCoreHandle->pmicDevSiliconRev) &&
        ((i2cTriggerType > PMIC_FSM_I2C_TRIGGER0) &&
         (i2cTriggerType < PMIC_FSM_I2C_TRIGGER4))))
    {
        pmicStatus = PMIC_ST_ERR_NOT_SUPPORTED;
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       ((i2cTriggerType > PMIC_FSM_I2C_TRIGGER7) ||
        (i2cTriggerVal > PMIC_FSM_I2C_TRIGGER_VAL_1)))
    {
        pmicStatus = PMIC_ST_ERR_INV_PARAM;
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       ((i2cTriggerType < PMIC_FSM_I2C_TRIGGER4) &&
        (i2cTriggerVal != PMIC_FSM_I2C_TRIGGER_VAL_1)))
    {
        pmicStatus = PMIC_ST_ERR_INV_PARAM;
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        Pmic_criticalSectionStart(pPmicCoreHandle);

        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                            PMIC_FSM_I2C_TRIGGERS_REGADDR,
                                            &regData);

        if(PMIC_ST_SUCCESS == pmicStatus)
        {
            Pmic_fsmGetI2cTriggerMaskBitField(i2cTriggerType,
                                              &bitPos,
                                              &bitMask);
            Pmic_setBitField(&regData,
                             bitPos,
                             bitMask,
                             i2cTriggerVal);

            pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                                PMIC_FSM_I2C_TRIGGERS_REGADDR,
                                                regData);
        }

        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    return pmicStatus;
}

/*!
 * \brief   API to get FSM I2C trigger Value for given FSM I2C trigger type
 *
 * Requirement: REQ_TAG(PDK-9330)
 * Design: did_pmic_fsm_i2c_trigger
 * Architecture: aid_pmic_fsm_cfg
 *
 *          This function is used to read the FSM I2C trigger Value of the
 *          FSM Trigger Type
 *
 * \param   pPmicCoreHandle [IN]    PMIC Interface Handle
 * \param   i2cTriggerType  [IN]    FSM I2C Trigger Type
 *                                  Valid values: \ref Pmic_Fsm_I2c_Trigger_Type
 * \param   pI2cTriggerVal  [OUT]   Pointer to store FSM I2C Trigger Value
 *                                  Valid values \ref Pmic_Fsm_I2c_Trigger_Val
 *
 * \return  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_fsmGetI2cTriggerVal(Pmic_CoreHandle_t *pPmicCoreHandle,
                                 const uint8_t      i2cTriggerType,
                                 uint8_t           *pI2cTriggerVal)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    uint8_t bitPos  = 0U;
    uint8_t bitMask = 0U;

    if(NULL == pPmicCoreHandle)
    {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    }

    if((PMIC_ST_SUCCESS == pmicStatus) && (NULL == pI2cTriggerVal))
    {
        pmicStatus = PMIC_ST_ERR_NULL_PARAM;
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (i2cTriggerType > PMIC_FSM_I2C_TRIGGER7))
    {
        pmicStatus = PMIC_ST_ERR_INV_PARAM;
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        Pmic_criticalSectionStart(pPmicCoreHandle);
        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                            PMIC_FSM_I2C_TRIGGERS_REGADDR,
                                            &regData);
        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        Pmic_fsmGetI2cTriggerMaskBitField(i2cTriggerType,
                                          &bitPos,
                                          &bitMask);
        *pI2cTriggerVal = Pmic_getBitField(regData, bitPos, bitMask);
    }

    return pmicStatus;
}

/*!
 * \brief   This function is used to set the defined FSM mission state.
 *
 *          Note: In this API, the default PMIC FSM State is assumed as
 *                LP_STANBY_STATE. While adding support for New PMIC FSM State,
 *                developer need to update the API functionality for New PMIC
 *                FSM State State accordingly.
 */
static int32_t Pmic_setState(Pmic_CoreHandle_t  *pPmicCoreHandle,
                             uint8_t             pmicNextState)
{
    int32_t status = PMIC_ST_SUCCESS;

    switch(pmicNextState)
    {
        case PMIC_FSM_STANBY_STATE:
           status = Pmic_fsmDeviceOffRequestCfg(pPmicCoreHandle,
                                                PMIC_FSM_I2C_TRIGGER0_TYPE,
                                                PMIC_FSM_STANBY_STATE);
           break;
        case PMIC_FSM_ACTIVE_STATE:
           status = Pmic_fsmSetNsleepSignalVal(pPmicCoreHandle,
                                               PMIC_NSLEEP2_SIGNAL,
                                               PMIC_NSLEEP_HIGH);

           if(PMIC_ST_SUCCESS == status)
           {
               status = Pmic_fsmSetNsleepSignalVal(pPmicCoreHandle,
                                                   PMIC_NSLEEP1_SIGNAL,
                                                   PMIC_NSLEEP_HIGH);
           }

           break;
        case PMIC_FSM_MCU_ONLY_STATE:
           status = Pmic_fsmSetNsleepSignalVal(pPmicCoreHandle,
                                               PMIC_NSLEEP1_SIGNAL,
                                               PMIC_NSLEEP_LOW);

           if(PMIC_ST_SUCCESS == status)
           {
               status = Pmic_fsmSetNsleepSignalVal(pPmicCoreHandle,
                                                   PMIC_NSLEEP2_SIGNAL,
                                                   PMIC_NSLEEP_HIGH);
           }

           break;
        case PMIC_FSM_S2R_STATE:
           status = Pmic_setS2RState(pPmicCoreHandle);

           break;
       default:
           /* Default case is valid only for LP_STANBY_STATE */
           status = Pmic_fsmDeviceOffRequestCfg(pPmicCoreHandle,
                                                PMIC_FSM_I2C_TRIGGER0_TYPE,
                                                PMIC_FSM_LP_STANBY_STATE);
           break;
        }

    return status;
}

/*!
 * \brief  API to initiate OFF Request FSM transition.
 *
 * Requirement: REQ_TAG(PDK-5851), REQ_TAG(PDK-9159), REQ_TAG(PDK-9329)
 * Design: did_pmic_lpstandby_cfg
 * Architecture: aid_pmic_fsm_cfg
 *
 *         This function initiate OFF Request FSM transition from any other
 *         mission state to the STANDBY state or the LP_STANDBY state
 *
 * \param   pPmicCoreHandle   [IN]    PMIC Interface Handle.
 * \param   eventType         [IN]    Event Type used to initiate OFF Request
 *                                    Valid values:
 *                                    \ref Pmic_Fsm_Off_Request_Type
 * \param   fsmState          [IN]    FSM state.
 *                                    Only Valid for:
 *                                                  PMIC_FSM_STANBY_STATE and
 *                                                  PMIC_FSM_LP_STANBY_STATE
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_fsmDeviceOffRequestCfg(Pmic_CoreHandle_t  *pPmicCoreHandle,
                                    uint8_t             eventType,
                                    uint8_t             fsmState)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    uint8_t regAddr = 0U;
    uint8_t bitPos  = 0U;
    uint8_t bitMask = 0U;

    if(NULL == pPmicCoreHandle)
    {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (eventType != PMIC_FSM_I2C_TRIGGER0_TYPE) &&
       (eventType != PMIC_FSM_ENABLE_PIN_TYPE) &&
       (eventType != PMIC_FSM_NPWRON_PIN_TYPE))
    {
        pmicStatus = PMIC_ST_ERR_INV_PARAM;
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (fsmState != PMIC_FSM_STANBY_STATE) &&
       (fsmState != PMIC_FSM_LP_STANBY_STATE))
    {
        pmicStatus = PMIC_ST_ERR_INV_PARAM;
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        Pmic_fsmGetstandByCfgRegFields(pPmicCoreHandle->pmicDeviceType,
                                       &regAddr,
                                       &bitPos,
                                       &bitMask);

        Pmic_criticalSectionStart(pPmicCoreHandle);

        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                            regAddr,
                                            &regData);

        if(PMIC_ST_SUCCESS == pmicStatus)
        {
            Pmic_setBitField(&regData, bitPos, bitMask, fsmState);

            pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                                regAddr,
                                                regData);
        }

        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (PMIC_FSM_I2C_TRIGGER0_TYPE == eventType))
    {
        pmicStatus = Pmic_fsmEnableI2cTrigger(pPmicCoreHandle,
                                              PMIC_FSM_I2C_TRIGGER0,
                                              PMIC_FSM_I2C_TRIGGER_VAL_1);
    }

    return pmicStatus;
}

/*!
 * \brief  API to initiate Runtime BIST.
 *
 * Requirement: REQ_TAG(PDK-5849)
 * Design: did_pmic_runtime_bist_cfg
 * Architecture: aid_pmic_fsm_cfg
 *
 *         This function initiates a request to exercise runtime BIST on the
 *         device
 *         Valid only for TPS6594x Leo PMIC PG2.0 and LP8764x Hera PMIC PG2.0
 *
 * \param   pPmicCoreHandle   [IN]    PMIC Interface Handle.
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_fsmRequestRuntimeBist(Pmic_CoreHandle_t  *pPmicCoreHandle)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    if(NULL == pPmicCoreHandle)
    {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (PMIC_SILICON_REV_ID_PG_2_0 != pPmicCoreHandle->pmicDevSiliconRev))
    {
        pmicStatus = PMIC_ST_ERR_NOT_SUPPORTED;
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        pmicStatus = Pmic_fsmEnableI2cTrigger(pPmicCoreHandle,
                                              PMIC_FSM_I2C_TRIGGER1,
                                              PMIC_FSM_I2C_TRIGGER_VAL_1);
    }

    return pmicStatus;
}

/*!
 * \brief  API to initiate ON Request FSM transition.
 *
 * Requirement: REQ_TAG(PDK-5837)
 * Design: did_pmic_fsm_cfg
 * Architecture: aid_pmic_fsm_cfg
 *
 *         This function setup nSLEEP signal bits with STARTUP_DEST
 *         Which is common for all supported PMICs. This API needs to be called
 *         at PMIC init before clearing Enable and Start-Up interrupts.
 *
 *  \param   pPmicCoreHandle  [IN]  PMIC Interface Handle
 *
 *          Note: In this API, the default PMIC device is assumed as TPS6594x
 *                LEO PMIC. While adding support for New PMIC device, developer
 *                need to update the API functionality for New PMIC device
 *                accordingly.
 */
int32_t Pmic_fsmDeviceOnRequest(Pmic_CoreHandle_t *pPmicCoreHandle)
{
    int32_t pmicStatus  = PMIC_ST_SUCCESS;
    uint8_t srcRegAddr  = 0U;
    uint8_t srcBitMask  = 0U;
    uint8_t srcBitShift = 0U;
    uint8_t dstRegAddr  = 0U;
    uint8_t regData     = 0U;
    uint8_t regVal      = 0U;
    uint8_t bitShift    = 0U;
    uint8_t bitMask     = 0U;

    if(NULL == pPmicCoreHandle)
    {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        switch(pPmicCoreHandle->pmicDeviceType)
        {
             case PMIC_DEV_HERA_LP8764X:
                srcRegAddr = PMIC_STARTUP_CTRL_REGADDR;
                dstRegAddr = PMIC_FSM_NSLEEP_TRIGGERS_REGADDR;
                srcBitMask = PMIC_STARTUP_CTRL_STARTUP_DEST_MASK;
                srcBitShift = PMIC_STARTUP_CTRL_STARTUP_DEST_SHIFT;
                break;
             default:
                /* Default case is valid only for TPS6594x LEO PMIC */
                srcRegAddr = PMIC_RTC_CTRL_2_REGADDR;
                dstRegAddr = PMIC_FSM_NSLEEP_TRIGGERS_REGADDR;
                srcBitMask = PMIC_RTC_CTRL_2_STARTUP_DEST_MASK;
                srcBitShift = PMIC_RTC_CTRL_2_STARTUP_DEST_SHIFT;
                break;
        }

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
            bitShift = 0x1U;
            bitMask  = (0x1U << bitShift);
            Pmic_setBitField(&regData,
                             PMIC_FSM_NSLEEP_TRIGGERS_NSLEEP2B_SHIFT,
                             PMIC_FSM_NSLEEP_TRIGGERS_NSLEEP2B_MASK,
                             (Pmic_getBitField(regVal, bitShift, bitMask)));

            bitShift = 0U;
            bitMask  = (0x1U << bitShift);
            Pmic_setBitField(&regData,
                             PMIC_FSM_NSLEEP_TRIGGERS_NSLEEP1B_SHIFT,
                             PMIC_FSM_NSLEEP_TRIGGERS_NSLEEP1B_MASK,
                             (Pmic_getBitField(regVal, bitShift, bitMask)));
            pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                                dstRegAddr,
                                                regData);
        }

        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    return pmicStatus;
}

/*!
 * \brief  API to Set FSM mission States.
 *
 * Requirement: REQ_TAG(PDK-5837), REQ_TAG(PDK-5851)
 * Design: did_pmic_fsm_cfg, did_pmic_lpstandby_cfg
 * Architecture: aid_pmic_fsm_cfg
 *
 *         This function is used for set/change the FSM mission states for PMIC
 *         using Nsleep1B and Nsleep2B signals in absence of GPIO pins
 *         Note: Application need to unmask Nsleep1B and Nsleep2B signals for
 *               FSM state transitions except for Standby/LpStandby State
 *               Application need to mask Nsleep1B and Nsleep2B signals for
 *               Standby/LpStandby State transition
 *               Application has to ensure the wakeup pins or RTC Timer/Alarm
 *               Interrupts are configured properly before triggering the PMIC
 *               device to Standby/LP Standby state.If not configured properly
 *               then PMIC device can't resume from sleep state
 *               Application has to ensure to connect/access the peripherals
 *               connected to only MCU Power lines when PMIC switch from Active
 *               to MCU state. If Application connects/access the peripherals
 *               connected to SOC Power lines when PMIC switch from Active to
 *               MCU state, then Application behaviour is unexpected.
 *
 * \param   pPmicCoreHandle  [IN]  PMIC Interface Handle
 * \param   pmicState        [IN]  PMIC FSM MISSION STATE
 *                                 Valid values: \ref Pmic_Fsm_Mission_State
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_fsmSetMissionState(Pmic_CoreHandle_t  *pPmicCoreHandle,
                                const uint8_t       pmicState)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    if(NULL == pPmicCoreHandle)
    {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    }

    if((PMIC_ST_SUCCESS == pmicStatus) && (pmicState > PMIC_FSM_STATE_MAX))
    {
        pmicStatus = PMIC_ST_ERR_INV_PARAM;
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        pmicStatus = Pmic_setState(pPmicCoreHandle, pmicState);
    }

    return pmicStatus;
}

/*!
 * \brief  API to MASK/UNMASK NSLEEP1B or 2B Signal.
 *
 * Requirement: REQ_TAG(PDK-5837)
 * Design: did_pmic_fsm_cfg_readback
 * Architecture: aid_pmic_fsm_cfg
 *
 *         This function is used for Masking/Unmasking for NSLEEP2B or NSLEEP1B
 *         signal.
 *
 * \param   pPmicCoreHandle  [IN]  PMIC Interface Handle
 * \param   nsleepType       [IN]  NSLEEP signal
 *                                 Valid values: \ref Pmic_Nsleep_Signals
 * \param   maskEnable       [IN]  Parameter to select masking/unmasking
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_fsmSetNsleepSignalMask(Pmic_CoreHandle_t  *pPmicCoreHandle,
                                    const bool          nsleepType,
                                    const bool          maskEnable)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t  regData = 0U;
    uint8_t  bitPos  = 0U;
    uint8_t  bitMask = 0U;
    uint8_t  maskEnableVal = 0U;

    if(NULL == pPmicCoreHandle)
    {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        Pmic_criticalSectionStart(pPmicCoreHandle);
        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                            PMIC_CONFIG_1_REGADDR,
                                            &regData);

        if(PMIC_ST_SUCCESS == pmicStatus)
        {
            Pmic_fsmGetNsleepMaskBitField(nsleepType,
                                          &bitPos,
                                          &bitMask);

            if(((bool)true) == maskEnable)
            {
                maskEnableVal = 1U;
            }

            Pmic_setBitField(&regData, bitPos, bitMask, maskEnableVal);
            pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                                PMIC_CONFIG_1_REGADDR,
                                                regData);
        }

        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    return pmicStatus;
}

/*!
 * \brief  API to read the status of the NSLEEP1B/2B Signal is masked or not
 *
 * Requirement: REQ_TAG(PDK-9151)
 * Design: did_pmic_fsm_cfg_readback
 * Architecture: aid_pmic_fsm_cfg
 *
 *         This function is used to read the status of the NSLEEP1B/2B Signal is
 *         masked or not
 *
 * \param   pPmicCoreHandle  [IN]  PMIC Interface Handle
 * \param   nsleepType       [IN]  NSLEEP signal
 *                                 Valid values: \ref Pmic_Nsleep_Signals
 * \param   pNsleepStat      [OUT] Pointer to store Nsleep Signal is masked or
 *                                 not
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_fsmGetNsleepSignalMaskStat(Pmic_CoreHandle_t  *pPmicCoreHandle,
                                        const bool          nsleepType,
                                        bool               *pNsleepStat)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t  regData = 0U;
    uint8_t  bitPos  = 0U;
    uint8_t  bitMask = 0U;
    uint8_t  maskEnableStat = 0U;

    if(NULL == pPmicCoreHandle)
    {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    }

    if((PMIC_ST_SUCCESS == pmicStatus) && (NULL == pNsleepStat))
    {
         pmicStatus = PMIC_ST_ERR_NULL_PARAM;
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        Pmic_criticalSectionStart(pPmicCoreHandle);
        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                            PMIC_CONFIG_1_REGADDR,
                                            &regData);
        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        *pNsleepStat = (bool)false;

        Pmic_fsmGetNsleepMaskBitField(nsleepType,
                                      &bitPos,
                                      &bitMask);
        maskEnableStat = Pmic_getBitField(regData, bitPos, bitMask);

        if(maskEnableStat == 1U)
        {
            *pNsleepStat = (bool)true;
        }
    }

    return pmicStatus;
}

/*!
 * \brief  API to Enable/Disable Fast BIST
 *          Note: In this API, the default pmicDeviceType is assumed as
 *                TPS6594X Leo Device. While adding support for New PMIC
 *                device, developer need to update the API functionality for
 *                New PMIC device accordingly.
 */
static int32_t Pmic_fsmEnableFastBIST(Pmic_CoreHandle_t   *pPmicCoreHandle,
                                      const Pmic_FsmCfg_t  fsmCfg)
{
    int32_t pmicStatus  = PMIC_ST_SUCCESS;
    uint8_t regAddr     = 0U;
    uint8_t regData     = 0U;
    uint8_t bitShift    = 0U;
    uint8_t bitMask     = 0U;

    switch(pPmicCoreHandle->pmicDeviceType)
    {
         case PMIC_DEV_HERA_LP8764X:
            regAddr = PMIC_STARTUP_CTRL_REGADDR;
            bitMask = PMIC_STARTUP_CTRL_FAST_BIST_MASK;
            bitShift = PMIC_STARTUP_CTRL_FAST_BIST_SHIFT;
            break;
         default:
            regAddr = PMIC_RTC_CTRL_2_REGADDR;
            bitMask = PMIC_RTC_CTRL_2_FAST_BIST_MASK;
            bitShift = PMIC_RTC_CTRL_2_FAST_BIST_SHIFT;
            break;
    }

    Pmic_criticalSectionStart(pPmicCoreHandle);

    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        regAddr,
                                        &regData);

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        if(((bool)PMIC_FSM_FAST_BIST_ENABLE) == fsmCfg.fastBistEn)
        {
            Pmic_setBitField(&regData,
                             bitShift,
                             bitMask,
                             PMIC_FSM_FAST_BIST_ENABLE);
        }
        else
        {
            Pmic_setBitField(&regData,
                             bitShift,
                             bitMask,
                             PMIC_FSM_FAST_BIST_DISABLE);
        }

        pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                            regAddr,
                                            regData);
    }

    Pmic_criticalSectionStop(pPmicCoreHandle);

    return pmicStatus;
}

/*!
 * \brief  API to Get Fast BIST is enabled or not
 *          Note: In this API, the default pmicDeviceType is assumed as
 *                TPS6594X Leo Device. While adding support for New PMIC
 *                device, developer need to update the API functionality for
 *                New PMIC device accordingly.
 */
static int32_t Pmic_fsmGetFastBISTCfg(Pmic_CoreHandle_t   *pPmicCoreHandle,
                                      Pmic_FsmCfg_t       *pFsmCfg)
{
    int32_t pmicStatus  = PMIC_ST_SUCCESS;
    uint8_t regAddr     = 0U;
    uint8_t regData     = 0U;
    uint8_t bitShift    = 0U;
    uint8_t bitMask     = 0U;

    switch(pPmicCoreHandle->pmicDeviceType)
    {
         case PMIC_DEV_HERA_LP8764X:
            regAddr = PMIC_STARTUP_CTRL_REGADDR;
            bitMask = PMIC_STARTUP_CTRL_FAST_BIST_MASK;
            bitShift = PMIC_STARTUP_CTRL_FAST_BIST_SHIFT;
            break;
         default:
            regAddr = PMIC_RTC_CTRL_2_REGADDR;
            bitMask = PMIC_RTC_CTRL_2_FAST_BIST_MASK;
            bitShift = PMIC_RTC_CTRL_2_FAST_BIST_SHIFT;
            break;
    }

    Pmic_criticalSectionStart(pPmicCoreHandle);

    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        regAddr,
                                        &regData);

    Pmic_criticalSectionStop(pPmicCoreHandle);

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        pFsmCfg->fastBistEn = (bool)false;

        if(Pmic_getBitField(regData, bitShift, bitMask) == 1U)
        {
            pFsmCfg->fastBistEn = (bool)true;
        }
    }

    return pmicStatus;
}

/*!
 * \brief  API to Set LPStandby/Standby State
 *          Note: In this API, the default pmicDeviceType is assumed as
 *                TPS6594X Leo Device. While adding support for New PMIC
 *                device, developer need to update the API functionality for
 *                New PMIC device accordingly.
 */
static int32_t Pmic_fsmSetLpStandbyState(Pmic_CoreHandle_t   *pPmicCoreHandle,
                                         const Pmic_FsmCfg_t  fsmCfg)
{
    int32_t pmicStatus  = PMIC_ST_SUCCESS;
    uint8_t regAddr     = 0U;
    uint8_t regData     = 0U;
    uint8_t bitShift    = 0U;
    uint8_t bitMask     = 0U;

    switch(pPmicCoreHandle->pmicDeviceType)
    {
         case PMIC_DEV_HERA_LP8764X:
            regAddr = PMIC_STARTUP_CTRL_REGADDR;
            bitMask = PMIC_STARTUP_CTRL_LP_STANDBY_SEL_MASK;
            bitShift = PMIC_STARTUP_CTRL_LP_STANDBY_SEL_SHIFT;
            break;
         default:
            regAddr = PMIC_RTC_CTRL_2_REGADDR;
            bitMask = PMIC_RTC_CTRL_2_LP_STANDBY_SEL_MASK;
            bitShift = PMIC_RTC_CTRL_2_LP_STANDBY_SEL_SHIFT;
            break;
    }

    Pmic_criticalSectionStart(pPmicCoreHandle);

    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        regAddr,
                                        &regData);

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        if(((bool)PMIC_FSM_SELECT_LPSTANDBY_STATE) == fsmCfg.lpStandbySel)
        {
            Pmic_setBitField(&regData,
                             bitShift,
                             bitMask,
                             PMIC_FSM_SELECT_LPSTANDBY_STATE);
        }
        else
        {
            Pmic_setBitField(&regData,
                             bitShift,
                             bitMask,
                             PMIC_FSM_SELECT_STANDBY_STATE);
        }

        pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                            regAddr,
                                            regData);
    }

    Pmic_criticalSectionStop(pPmicCoreHandle);

    return pmicStatus;
}

/*!
 * \brief  API to Get LPStandby/Standby State
 *          Note: In this API, the default pmicDeviceType is assumed as
 *                TPS6594X Leo Device. While adding support for New PMIC
 *                device, developer need to update the API functionality for
 *                New PMIC device accordingly.
 */
static int32_t Pmic_fsmGetLpStandbyStateCfg(
                                           Pmic_CoreHandle_t   *pPmicCoreHandle,
                                           Pmic_FsmCfg_t       *pFsmCfg)
{
    int32_t pmicStatus  = PMIC_ST_SUCCESS;
    uint8_t regAddr     = 0U;
    uint8_t regData     = 0U;
    uint8_t bitShift    = 0U;
    uint8_t bitMask     = 0U;

    switch(pPmicCoreHandle->pmicDeviceType)
    {
         case PMIC_DEV_HERA_LP8764X:
            regAddr = PMIC_STARTUP_CTRL_REGADDR;
            bitMask = PMIC_STARTUP_CTRL_LP_STANDBY_SEL_MASK;
            bitShift = PMIC_STARTUP_CTRL_LP_STANDBY_SEL_SHIFT;
            break;
         default:
            regAddr = PMIC_RTC_CTRL_2_REGADDR;
            bitMask = PMIC_RTC_CTRL_2_LP_STANDBY_SEL_MASK;
            bitShift = PMIC_RTC_CTRL_2_LP_STANDBY_SEL_SHIFT;
            break;
    }

    Pmic_criticalSectionStart(pPmicCoreHandle);

    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        regAddr,
                                        &regData);

    Pmic_criticalSectionStop(pPmicCoreHandle);

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        pFsmCfg->lpStandbySel = (bool)false;

        if(Pmic_getBitField(regData, bitShift, bitMask) == 1U)
        {
            pFsmCfg->lpStandbySel = (bool)true;
        }
    }

    return pmicStatus;
}

/*!
 * \brief  API to Enable/Disable Buck/LDO regulators ILIM interrupts affect FSM
           triggers */
static int32_t Pmic_fsmEnableBuckLdoIlimIntAffectFsm(
                                          Pmic_CoreHandle_t   *pPmicCoreHandle,
                                          const Pmic_FsmCfg_t  fsmCfg)
{
    int32_t pmicStatus  = PMIC_ST_SUCCESS;
    uint8_t regData     = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);

    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        PMIC_CONFIG_1_REGADDR,
                                        &regData);

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        if(((bool)PMIC_FSM_ILIM_INT_FSMCTRL_ENABLE) == fsmCfg.ilimIntfsmCtrlEn)
        {
            Pmic_setBitField(&regData,
                             PMIC_CONFIG_1_EN_ILIM_FSM_CTRL_SHIFT,
                             PMIC_CONFIG_1_EN_ILIM_FSM_CTRL_MASK,
                             PMIC_FSM_ILIM_INT_FSMCTRL_ENABLE);
        }
        else
        {
            Pmic_setBitField(&regData,
                             PMIC_CONFIG_1_EN_ILIM_FSM_CTRL_SHIFT,
                             PMIC_CONFIG_1_EN_ILIM_FSM_CTRL_MASK,
                             PMIC_FSM_ILIM_INT_FSMCTRL_DISABLE);
        }

        pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                            PMIC_CONFIG_1_REGADDR,
                                            regData);
    }

    Pmic_criticalSectionStop(pPmicCoreHandle);

    return pmicStatus;
}

/*!
 * \brief  API to Get Buck/LDO regulators ILIM interrupts affect FSM triggers is
 *         enabled or not
 */
static int32_t Pmic_fsmGetBuckLdoIlimIntAffectFsmCfg(
                                         Pmic_CoreHandle_t   *pPmicCoreHandle,
                                         Pmic_FsmCfg_t       *pFsmCfg)
{
    int32_t pmicStatus  = PMIC_ST_SUCCESS;
    uint8_t regData     = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);

    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        PMIC_CONFIG_1_REGADDR,
                                        &regData);

    Pmic_criticalSectionStop(pPmicCoreHandle);

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        pFsmCfg->ilimIntfsmCtrlEn = (bool)false;

        if(Pmic_getBitField(regData,
                            PMIC_CONFIG_1_EN_ILIM_FSM_CTRL_SHIFT,
                            PMIC_CONFIG_1_EN_ILIM_FSM_CTRL_MASK) == 1U)
        {
            pFsmCfg->ilimIntfsmCtrlEn = (bool)true;
        }
    }

    return pmicStatus;
}

/*!
 * \brief  API to Set FSM Startup Destination State
 *          Note: In this API, the default pmicDeviceType is assumed as
 *                TPS6594X Leo Device. While adding support for New PMIC
 *                device, developer need to update the API functionality for
 *                New PMIC device accordingly.
 */
static int32_t Pmic_fsmSetStartupDestState(Pmic_CoreHandle_t   *pPmicCoreHandle,
                                           const Pmic_FsmCfg_t  fsmCfg)
{
    int32_t pmicStatus  = PMIC_ST_SUCCESS;
    uint8_t regAddr     = 0U;
    uint8_t regData     = 0U;
    uint8_t bitShift    = 0U;
    uint8_t bitMask     = 0U;

    switch(pPmicCoreHandle->pmicDeviceType)
    {
         case PMIC_DEV_HERA_LP8764X:
            regAddr = PMIC_STARTUP_CTRL_REGADDR;
            bitMask = PMIC_STARTUP_CTRL_STARTUP_DEST_MASK;
            bitShift = PMIC_STARTUP_CTRL_STARTUP_DEST_SHIFT;
            break;
         default:
            regAddr = PMIC_RTC_CTRL_2_REGADDR;
            bitMask = PMIC_RTC_CTRL_2_STARTUP_DEST_MASK;
            bitShift = PMIC_RTC_CTRL_2_STARTUP_DEST_SHIFT;
            break;
    }

    Pmic_criticalSectionStart(pPmicCoreHandle);

    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        regAddr,
                                        &regData);

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        Pmic_setBitField(&regData,
                         bitShift,
                         bitMask,
                         fsmCfg.fsmStarupDestSel);

        pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                            regAddr,
                                            regData);
    }

    Pmic_criticalSectionStop(pPmicCoreHandle);

    return pmicStatus;
}

/*!
 * \brief  API to Get Get FSM Startup Destination State
 *          Note: In this API, the default pmicDeviceType is assumed as
 *                TPS6594X Leo Device. While adding support for New PMIC
 *                device, developer need to update the API functionality for
 *                New PMIC device accordingly.
 */
static int32_t Pmic_fsmGetStartupDestStateCfg(
                                          Pmic_CoreHandle_t   *pPmicCoreHandle,
                                          Pmic_FsmCfg_t       *pFsmCfg)
{
    int32_t pmicStatus  = PMIC_ST_SUCCESS;
    uint8_t regAddr     = 0U;
    uint8_t regData     = 0U;
    uint8_t bitShift    = 0U;
    uint8_t bitMask     = 0U;

    switch(pPmicCoreHandle->pmicDeviceType)
    {
         case PMIC_DEV_HERA_LP8764X:
            regAddr = PMIC_STARTUP_CTRL_REGADDR;
            bitMask = PMIC_STARTUP_CTRL_STARTUP_DEST_MASK;
            bitShift = PMIC_STARTUP_CTRL_STARTUP_DEST_SHIFT;
            break;
         default:
            regAddr = PMIC_RTC_CTRL_2_REGADDR;
            bitMask = PMIC_RTC_CTRL_2_STARTUP_DEST_MASK;
            bitShift = PMIC_RTC_CTRL_2_STARTUP_DEST_SHIFT;
            break;
    }

    Pmic_criticalSectionStart(pPmicCoreHandle);

    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        regAddr,
                                        &regData);

    Pmic_criticalSectionStop(pPmicCoreHandle);

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        pFsmCfg->fsmStarupDestSel = Pmic_getBitField(regData,
                                                     bitShift,
                                                     bitMask);
    }

    return pmicStatus;
}

/*!
 * \brief   API to set PMIC FSM configuration.
 *
 * Requirement: REQ_TAG(PDK-9144), REQ_TAG(PDK-9134), REQ_TAG(PDK-9128)
 * Design: did_pmic_fsm_cfg_readback
 * Architecture: aid_pmic_fsm_cfg
 *
 *          This function is used to set the required FSM configuration when
 *          corresponding bit field is set.
 *
 * \param   pPmicCoreHandle [IN]    PMIC Interface Handle.
 * \param   fsmCfg          [IN]    Set required FSM configuration
 *
 * \return  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_fsmSetConfiguration(Pmic_CoreHandle_t   *pPmicCoreHandle,
                                 const Pmic_FsmCfg_t  fsmCfg)
{
    int32_t pmicStatus  = PMIC_ST_SUCCESS;

    if(NULL == pPmicCoreHandle)
    {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       ((bool)true == pmic_validParamCheck(fsmCfg.validParams,
                                           PMIC_FSM_CFG_FAST_BIST_EN_VALID)))
    {
        /* Enable/Disable Fast BIST */
        pmicStatus = Pmic_fsmEnableFastBIST(pPmicCoreHandle, fsmCfg);
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       ((bool)true == pmic_validParamCheck(fsmCfg.validParams,
                                           PMIC_FSM_CFG_LP_STANDBYSEL_VALID)))
    {
        /* Set LPStandby/Standby State */
        pmicStatus = Pmic_fsmSetLpStandbyState(pPmicCoreHandle, fsmCfg);
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       ((bool)true == pmic_validParamCheck(
                                       fsmCfg.validParams,
                                       PMIC_FSM_CFG_ILIM_INT_FSMCTRL_EN_VALID)))
    {
        /* Enable/Disable Buck/LDO regulators ILIM interrupts affect FSM
         * triggers */
        pmicStatus = Pmic_fsmEnableBuckLdoIlimIntAffectFsm(pPmicCoreHandle,
                                                           fsmCfg);
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       ((bool)true == pmic_validParamCheck(
                                      fsmCfg.validParams,
                                      PMIC_FSM_CFG_FSM_STARTUP_DEST_SEL_VALID)))
    {
        if(fsmCfg.fsmStarupDestSel > PMIC_FSM_STARTUPDEST_ACTIVE)
        {
            pmicStatus = PMIC_ST_ERR_INV_PARAM;
        }

        if(PMIC_ST_SUCCESS == pmicStatus)
        {
            /* Set FSM Startup Destination State */
            pmicStatus = Pmic_fsmSetStartupDestState(pPmicCoreHandle, fsmCfg);
        }
    }

    return pmicStatus;
}

/*!
 * \brief   API to get PMIC FSM configuration.
 *
 * Requirement: REQ_TAG(PDK-9144), REQ_TAG(PDK-9134), REQ_TAG(PDK-9128)
 * Design: did_pmic_fsm_cfg_readback
 * Architecture: aid_pmic_fsm_cfg
 *
 *          This function is used to get the FSM configuration when
 *          corresponding validParam bit fields are set in Pmic_FsmCfg_t
 *          structure
 *
 * \param   pPmicCoreHandle [IN]       PMIC Interface Handle
 * \param   pFsmCfg         [IN/OUT]   Pointer to store FSM configuration
 *
 * \return  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_fsmGetConfiguration(Pmic_CoreHandle_t *pPmicCoreHandle,
                                 Pmic_FsmCfg_t     *pFsmCfg)
{
    int32_t pmicStatus  = PMIC_ST_SUCCESS;

    if(NULL == pPmicCoreHandle)
    {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    }

    if((PMIC_ST_SUCCESS == pmicStatus) && (NULL == pFsmCfg))
    {
        pmicStatus = PMIC_ST_ERR_NULL_PARAM;
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       ((bool)true == pmic_validParamCheck(pFsmCfg->validParams,
                                           PMIC_FSM_CFG_FAST_BIST_EN_VALID)))
    {
        /* Get Fast BIST is enabled or not*/
        pmicStatus = Pmic_fsmGetFastBISTCfg(pPmicCoreHandle, pFsmCfg);
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       ((bool)true == pmic_validParamCheck(pFsmCfg->validParams,
                                           PMIC_FSM_CFG_LP_STANDBYSEL_VALID)))
    {
        /* Get LPStandby/Standby State */
        pmicStatus = Pmic_fsmGetLpStandbyStateCfg(pPmicCoreHandle, pFsmCfg);
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       ((bool)true == pmic_validParamCheck(
                                       pFsmCfg->validParams,
                                       PMIC_FSM_CFG_ILIM_INT_FSMCTRL_EN_VALID)))
    {
        /* Get Buck/LDO regulators ILIM interrupts affect FSM
         * triggers is enabled or not */
        pmicStatus = Pmic_fsmGetBuckLdoIlimIntAffectFsmCfg(pPmicCoreHandle,
                                                           pFsmCfg);
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       ((bool)true == pmic_validParamCheck(
                                      pFsmCfg->validParams,
                                      PMIC_FSM_CFG_FSM_STARTUP_DEST_SEL_VALID)))
    {
        /* Get FSM Startup Destination State */
        pmicStatus = Pmic_fsmGetStartupDestStateCfg(pPmicCoreHandle, pFsmCfg);
    }

    return pmicStatus;
}

/*!
 * \brief  API to configure PFSM Delay
 *
 * Requirement: REQ_TAG(PDK-9136)
 * Design: did_pmic_pfsm_cfg_readback
 * Architecture: aid_pmic_fsm_cfg
 *
 *         This function is used to configure PFSM Delay. PFSM Delay will affect
 *         the total power up sequence time before the system is released from
 *         reset
 *         Consider If the PFSM_Delay value is 'x' then Delay will calculated as
 *          Delay = x *(50ns * 2^PFSM_DELAY_STEP)
 *         Note: In this API, the default delay Type is assumed as
 *         PMIC_PFSM_DELAY1
 *         While adding support for New PMIC, developer need to update the API
 *         functionality for New PMIC device accordingly.
 *
 * \param   pPmicCoreHandle  [IN]  PMIC Interface Handle
 * \param   pFsmDelayType    [IN]  PFSM Delay Type
 *                                 Valid values: \ref Pmic_Pfsm_Delay_Type
 * \param   pfsmDelay        [IN]  Delay for PFSM
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_fsmSetPfsmDelay(Pmic_CoreHandle_t  *pPmicCoreHandle,
                             const uint8_t       pFsmDelayType,
                             const uint8_t       pfsmDelay)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regAddr = 0U;

    if(NULL == pPmicCoreHandle)
    {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (pFsmDelayType > PMIC_PFSM_DELAY4))
    {
        pmicStatus = PMIC_ST_ERR_INV_PARAM;
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        switch(pFsmDelayType)
        {
            case PMIC_PFSM_DELAY2:
                regAddr = PMIC_FSM_PFSM_DELAY_REG_2_REGADDR;
                break;
            case PMIC_PFSM_DELAY3:
                regAddr = PMIC_FSM_PFSM_DELAY_REG_3_REGADDR;
                break;
            case PMIC_PFSM_DELAY4:
                regAddr = PMIC_FSM_PFSM_DELAY_REG_4_REGADDR;
                break;
            default:
                regAddr = PMIC_FSM_PFSM_DELAY_REG_1_REGADDR;
                break;
        }

        Pmic_criticalSectionStart(pPmicCoreHandle);
        pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                            regAddr,
                                            pfsmDelay);
        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    return pmicStatus;
}

/*!
 * \brief  API to read PFSM Delay
 *
 * Requirement: REQ_TAG(PDK-9136)
 * Design: did_pmic_pfsm_cfg_readback
 * Architecture: aid_pmic_fsm_cfg
 *
 *         This function is used to read PFSM Delay
 *         Note: In this API, the default delay Type is assumed as
 *         PMIC_PFSM_DELAY1
 *         While adding support for New PMIC, developer need to update the API
 *         functionality for New PMIC device accordingly.
 *
 * \param   pPmicCoreHandle  [IN]   PMIC Interface Handle
 * \param   pFsmDelayType    [IN]   PFSM Delay Type
 *                                  Valid values: \ref Pmic_Pfsm_Delay_Type
 * \param   pPfsmDelay       [OUT]  Pointer to store the Delay for PFSM
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_fsmGetPfsmDelay(Pmic_CoreHandle_t  *pPmicCoreHandle,
                             uint8_t             pFsmDelayType,
                             uint8_t            *pPfsmDelay)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regAddr = 0U;

    if(NULL == pPmicCoreHandle)
    {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    }

    if((PMIC_ST_SUCCESS == pmicStatus) && (NULL == pPfsmDelay))
    {
        pmicStatus = PMIC_ST_ERR_NULL_PARAM;
    }

    if((PMIC_ST_SUCCESS == pmicStatus) && (pFsmDelayType > PMIC_PFSM_DELAY4))
    {
        pmicStatus = PMIC_ST_ERR_INV_PARAM;
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        switch(pFsmDelayType)
        {
            case PMIC_PFSM_DELAY2:
                regAddr = PMIC_FSM_PFSM_DELAY_REG_2_REGADDR;
                break;
            case PMIC_PFSM_DELAY3:
                regAddr = PMIC_FSM_PFSM_DELAY_REG_3_REGADDR;
                break;
            case PMIC_PFSM_DELAY4:
                regAddr = PMIC_FSM_PFSM_DELAY_REG_4_REGADDR;
                break;
            default:
                regAddr = PMIC_FSM_PFSM_DELAY_REG_1_REGADDR;
                break;
        }

        Pmic_criticalSectionStart(pPmicCoreHandle);
        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                            regAddr,
                                            pPfsmDelay);

        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    return pmicStatus;
}

/*!
 * \brief   API to recover from SOC Power Error using Nsleep1B and Nsleep2B
 *          signal
 *
 * Requirement: REQ_TAG(PDK-9123)
 * Design: did_pmic_fsm_recover_soc_pwr_err
 * Architecture: aid_pmic_fsm_cfg
 *
 *          This function is used to recover from SOC Power Error without
 *          rebooting the system
 *          Note: Application need to call this API from MCU domain when SOC
 *          Power Error on Primary PMIC
 *          Step-1 - PMIC LLD has to configure NSLEEP2 & NSLEEP1 signals to ‘10’
 *          Step-2 - Application has to wait for 9us
 *          Step-3 - PMIC LLD has to configure NSLEEP2 & NSLEEP1 signals to ‘11’
 *
 *          Note: Valid only for TPS6594x Leo PMIC PG2.0 and LP8764x Hera PMIC
 *          PG2.0
 *
 * \param   pPmicCoreHandle [IN]    PMIC Interface Handle
 * \param   nsleepVal       [IN]    PMIC Nsleep signal level High/Low to be
 *                                  configured.
 *                                  Valid values \ref Pmic_Nsleep_SignalLvl
 *
 * \return  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_fsmRecoverSocPwrErr(Pmic_CoreHandle_t *pPmicCoreHandle,
                                 const uint8_t      nsleepVal)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    if(NULL == pPmicCoreHandle)
    {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (PMIC_SILICON_REV_ID_PG_2_0 != pPmicCoreHandle->pmicDevSiliconRev))
    {
        pmicStatus = PMIC_ST_ERR_NOT_SUPPORTED;
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        pmicStatus = Pmic_fsmSetNsleepSignalMask(pPmicCoreHandle,
                                                 PMIC_NSLEEP2_SIGNAL,
                                                 ((bool)PMIC_NSLEEPX_UNMASK));
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        pmicStatus = Pmic_fsmSetNsleepSignalMask(pPmicCoreHandle,
                                                 PMIC_NSLEEP1_SIGNAL,
                                                 ((bool)PMIC_NSLEEPX_UNMASK));
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        pmicStatus = Pmic_fsmSetNsleepSignalVal(pPmicCoreHandle,
                                                PMIC_NSLEEP2_SIGNAL,
                                                PMIC_NSLEEP_HIGH);
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        pmicStatus = Pmic_fsmSetNsleepSignalVal(pPmicCoreHandle,
                                                PMIC_NSLEEP1_SIGNAL,
                                                nsleepVal);
    }

    return pmicStatus;
}

/*!
 * \brief  API to initiate DDR/GPIO Retention Mode
 *
 * Requirement: REQ_TAG(PDK-9563), REQ_TAG(PDK-9564)
 * Design: did_pmic_ddr_gpio_retention_cfg
 * Architecture: aid_pmic_fsm_cfg
 *
 *         This function initiates a request to exercise DDR/GPIO Retention Mode
 *         on the device based on the Retention Mode
 *         Note: PMIC_FSM_GPIO_RETENTION_MODE is valid only for J7200 SOC
 *               Application has to ensure to connect/access the peripherals
 *               connected to only MCU Power lines except EN_GPIORET_LDSW,
 *               VDD_WK_0V8 Power lines when PMIC switch from Active to MCU
 *               state in GPIO Rentention mode with FSM i2c6 trigger value as
 *               '0'. If Application connects/access the peripherals connected
 *               to EN_GPIORET_LDSW, VDD_WK_0V8 Power lines when PMIC switch
 *               from Active to MCU state in GPIO Rentention mode with FSM i2c6
 *               trigger value as '0', then Application behaviour is unexpected.
 *               Application has to ensure to connect/access the peripherals
 *               connected to only MCU Power lines except VDD1_LPDDR4_1V8,
 *               VDD_DDR_1V1, GPIO_EN_VDDR_IO Power lines when PMIC switch from
 *               Active to MCU state in DDR Rentention mode with FSM i2c7
 *               trigger value as '0'. If Application connects/access the
 *               peripherals connected to VDD1_LPDDR4_1V8, VDD_DDR_1V1,
 *               GPIO_EN_VDDR_IO Power lines when PMIC switch from Active to MCU
 *               state in DDR Rentention mode with FSM i2c7 trigger value as
 *               '0', then Application behaviour is unexpected.
 *
 * \param   pPmicCoreHandle  [IN]  PMIC Interface Handle
 * \param   retentionMode    [IN]  Retention Mode
 *                                   Valid values: \ref Pmic_Fsm_Retention_Mode
 *                                     PMIC_FSM_GPIO_RETENTION_MODE is valid
 *                                     only for J7200 SOC
 * \param   i2cTriggerVal    [IN]   FSM I2C Trigger Value
 *                                   Valid values: \ref Pmic_Fsm_I2c_Trigger_Val
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_fsmRequestDdrGpioRetentionMode(
                                        Pmic_CoreHandle_t  *pPmicCoreHandle,
                                        const uint8_t       retentionMode,
                                        const uint8_t       i2cTriggerVal)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    if(NULL == pPmicCoreHandle)
    {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (retentionMode > PMIC_FSM_GPIO_RETENTION_MODE))
    {
        pmicStatus = PMIC_ST_ERR_INV_PARAM;
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        if(PMIC_FSM_DDR_RETENTION_MODE == retentionMode)
        {
            pmicStatus = Pmic_fsmEnableI2cTrigger(pPmicCoreHandle,
                                                  PMIC_FSM_I2C_TRIGGER7,
                                                  i2cTriggerVal);
        }
        else
        {
            pmicStatus = Pmic_fsmEnableI2cTrigger(pPmicCoreHandle,
                                                  PMIC_FSM_I2C_TRIGGER6,
                                                  i2cTriggerVal);
        }
    }

    return pmicStatus;
}
