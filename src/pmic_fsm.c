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
 */
static int32_t Pmic_fsmGetstandByCfgRegFields(uint8_t  pmicDeviceType,
                                              uint8_t  *pRegAddr,
                                              uint8_t  *pBitPos,
                                              uint8_t  *pBitMask)
{
    int32_t status = PMIC_ST_SUCCESS;

    switch(pmicDeviceType)
    {
        case PMIC_DEV_LEO_TPS6594X:
            *pRegAddr = PMIC_RTC_CTRL_2_REGADDR;
            *pBitPos = PMIC_RTC_CTRL_2_LP_STANDBY_SEL_SHIFT;
            *pBitMask = PMIC_RTC_CTRL_2_LP_STANDBY_SEL_MASK;
            break;
        case PMIC_DEV_HERA_LP8764X:
            *pRegAddr = PMIC_STARTUP_CTRL_REGADDR;
            *pBitPos = PMIC_STARTUP_CTRL_LP_STANDBY_SEL_SHIFT;
            *pBitMask = PMIC_STARTUP_CTRL_LP_STANDBY_SEL_MASK;
            break;
        default:
            status = PMIC_ST_ERR_INV_DEVICE;
            break;
    }

    return status;
}

/*!
 * \brief   This function is used to get the required BitPos and Mask of
 *          NSLEEP Signals.
 */
static int32_t Pmic_powerGetNsleepMaskBitField(bool     nsleepType,
                                               uint8_t *pBitPos,
                                               uint8_t *pBitMask)
{
    int32_t status = PMIC_ST_SUCCESS;

    if(nsleepType == PMIC_NSLEEP1_SIGNAL)
    {
        *pBitPos  = PMIC_CONFIG_1_NSLEEP1_MASK_SHIFT;
        *pBitMask = PMIC_CONFIG_1_NSLEEP1_MASK_MASK;
    }
    else if(nsleepType == PMIC_NSLEEP2_SIGNAL)
    {
        *pBitPos  = PMIC_CONFIG_1_NSLEEP2_MASK_SHIFT;
        *pBitMask = PMIC_CONFIG_1_NSLEEP2_MASK_MASK;
    }

    return status;
}

/*!
 * \brief   This function is used to set S2R/Deep sleep FSM Misson state.
 */
static int32_t Pmic_setS2RState(Pmic_CoreHandle_t *pPmicCoreHandle)
{
    int32_t status  = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    if(PMIC_ST_SUCCESS == status)
    {
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
    }

    return status;
}

/*!
 * \brief   This function is used to set the defined NSLEEP Signal.
 */
static int32_t Pmic_setNsleepSignal(Pmic_CoreHandle_t *pPmicCoreHandle,
                                    bool               value,
                                    bool               nsleepSignal)
{
    int32_t status  = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    uint8_t bitPos  = 0U;
    uint8_t bitMask = 0U;

    if(PMIC_NSLEEP1_SIGNAL == nsleepSignal)
    {
        bitPos  = PMIC_FSM_NSLEEP_TRIGGERS_NSLEEP1B_SHIFT;
        bitMask = PMIC_FSM_NSLEEP_TRIGGERS_NSLEEP1B_MASK;
    }
    else if(PMIC_NSLEEP2_SIGNAL == nsleepSignal)
    {
        bitPos  = PMIC_FSM_NSLEEP_TRIGGERS_NSLEEP2B_SHIFT;
        bitMask = PMIC_FSM_NSLEEP_TRIGGERS_NSLEEP2B_MASK;
    }

    if(PMIC_ST_SUCCESS == status)
    {
        Pmic_criticalSectionStart(pPmicCoreHandle);
        status = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        PMIC_FSM_NSLEEP_TRIGGERS_REGADDR,
                                        &regData);

        if(PMIC_ST_SUCCESS == status)
        {
            Pmic_setBitField(&regData, bitPos, bitMask, value);
            status = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                            PMIC_FSM_NSLEEP_TRIGGERS_REGADDR,
                                            regData);
        }

        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    return status;
}

/*!
 *  \brief This function is used to initiate I2C trigger for given I2C trigger
 *         type.
 */
static int32_t Pmic_fsmEnableI2cTrigger(Pmic_CoreHandle_t  *pPmicCoreHandle,
                                        uint8_t             i2cTriggerType)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData  = 0U;
    uint8_t bitShift = 0;
    uint8_t bitMask  = 0;;

    Pmic_criticalSectionStart(pPmicCoreHandle);

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                            PMIC_FSM_I2C_TRIGGERS_REGADDR,
                                            &regData);
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        if(i2cTriggerType == PMIC_FSM_I2C_TRIGGER0)
        {
            bitShift = PMIC_FSM_I2C_TRIGGERS_TRIGGER_I2C_0_SHIFT;
            bitMask  = PMIC_FSM_I2C_TRIGGERS_TRIGGER_I2C_0_MASK;
        }
        else if(i2cTriggerType == PMIC_FSM_I2C_TRIGGER1)
        {
            bitShift = PMIC_FSM_I2C_TRIGGERS_TRIGGER_I2C_1_SHIFT;
            bitMask  = PMIC_FSM_I2C_TRIGGERS_TRIGGER_I2C_1_MASK;
        }

        Pmic_setBitField(&regData,
                         bitShift,
                         bitMask,
                         PMIC_FSM_I2C_TRIGGER_VAL);

        pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                            PMIC_FSM_I2C_TRIGGERS_REGADDR,
                                            regData);
    }

    Pmic_criticalSectionStop(pPmicCoreHandle);

    return pmicStatus;
}

/*!
 * \brief   This function is used to set the defined FSM mission state.
 */
static int32_t Pmic_setState(Pmic_CoreHandle_t  *pPmicCoreHandle,
                             uint8_t             pmicNextState)
{
    int32_t status = PMIC_ST_SUCCESS;

    if(PMIC_ST_SUCCESS == status)
    {
         switch(pmicNextState)
         {
             case PMIC_FSM_LP_STANBY_STATE:
                status = Pmic_fsmDeviceOffRequestCfg(pPmicCoreHandle,
                                                     PMIC_FSM_I2C_TRIGGER0_TYPE,
                                                     PMIC_FSM_LP_STANBY_STATE);
                break;
             case PMIC_FSM_STANBY_STATE:
                status = Pmic_fsmDeviceOffRequestCfg(pPmicCoreHandle,
                                                     PMIC_FSM_I2C_TRIGGER0_TYPE,
                                                     PMIC_FSM_STANBY_STATE);
                break;
             case PMIC_FSM_ACTIVE_STATE:
                status = Pmic_setNsleepSignal(pPmicCoreHandle,
                                              PMIC_FSM_NSLEEPX_SET,
                                              PMIC_NSLEEP2_SIGNAL);

                if(PMIC_ST_SUCCESS == status)
                {
                    status = Pmic_setNsleepSignal(pPmicCoreHandle,
                                                  PMIC_FSM_NSLEEPX_SET,
                                                  PMIC_NSLEEP1_SIGNAL);
                }

                break;
             case PMIC_FSM_MCU_ONLY_STATE:
                status = Pmic_setNsleepSignal(pPmicCoreHandle,
                                              PMIC_FSM_NSLEEPX_RESET,
                                              PMIC_NSLEEP1_SIGNAL);

                if(PMIC_ST_SUCCESS == status)
                {
                    status = Pmic_setNsleepSignal(pPmicCoreHandle,
                                                  PMIC_FSM_NSLEEPX_SET,
                                                  PMIC_NSLEEP2_SIGNAL);
                }

                break;
             case PMIC_FSM_S2R_STATE:
                status = Pmic_setS2RState(pPmicCoreHandle);

                break;
            default:
                status = PMIC_ST_ERR_INV_PARAM;
                break;
        }
    }

    return status;
}

/*!
 * \brief  API to initiate OFF Request FSM transition.
 *
 * Requirement: REQ_TAG(PDK-5851)
 * Design: did_pmic_lpstandby_cfg
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
        pmicStatus = Pmic_fsmGetstandByCfgRegFields(
                                            pPmicCoreHandle->pmicDeviceType,
                                            &regAddr,
                                            &bitPos,
                                            &bitMask);

        Pmic_criticalSectionStart(pPmicCoreHandle);

        if(PMIC_ST_SUCCESS == pmicStatus)
        {
            pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                                regAddr,
                                                &regData);
        }
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
                                              PMIC_FSM_I2C_TRIGGER0);
    }

    return pmicStatus;
}

/*!
 * \brief  API to initiate Runtime BIST.
 *
 * Requirement: REQ_TAG(PDK-5849)
 * Design: did_pmic_runtime_bist_cfg
 *
 *         This function initiates a request to exercise runtime BIST on the
 *         device
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

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        pmicStatus = Pmic_fsmEnableI2cTrigger(pPmicCoreHandle,
                                              PMIC_FSM_I2C_TRIGGER1);
    }

    return pmicStatus;
}

/*!
 * \brief  API to initiate ON Request FSM transition.
 *
 * Requirement: REQ_TAG(PDK-5837)
 * Design: did_pmic_fsm_cfg
 *
 *         This function setup nSLEEP signal bits with STARTUP_DEST
 *         Which is common for all supported PMICs. This API needs to be called
 *         at PMIC init before clearing Enable and Start-Up interrupts.
 *
 *  \param   pPmicCoreHandle  [IN]  PMIC Interface Handle
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
 * Requirement: REQ_TAG(PDK-5837), REQ_TAG(PDK-5851), REQ_TAG(PDK-5844),
 *              REQ_TAG(PDK-5831)
 * Design: did_pmic_fsm_cfg, did_pmic_lpstandby_cfg,
 *         did_pmic_lpstandby_wkup_cfg
 *
 *         This function is used for set/change the FSM mission states for PMIC
 *
 * \param   pPmicCoreHandle  [IN]  PMIC Interface Handle
 * \param   pmicState        [IN]  PMIC FSM MISSION STATE
 *                                 Valid values: \ref Pmic_Fsm_Mission_State
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_fsmSetMissionState(Pmic_CoreHandle_t  *pPmicCoreHandle,
                                uint8_t             pmicState)
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
 * \brief  API to MASK/UNMASK NSLEEP Signal.
 *
 * Requirement: REQ_TAG(PDK-5837), REQ_TAG(PDK-5844), REQ_TAG(PDK-5831)
 * Design: did_pmic_fsm_cfg, did_pmic_lpstandby_wkup_cfg
 *
 *         This function is used for Masking/Unmasking for NSLEEP2 or NSLEEP1
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
                                    bool                nsleepType,
                                    bool                maskEnable)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t  regData = 0U;
    uint8_t  bitPos  = 0U;
    uint8_t  bitMask = 0U;

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
            pmicStatus = Pmic_powerGetNsleepMaskBitField(nsleepType,
                                                         &bitPos,
                                                         &bitMask);
        }

        if(PMIC_ST_SUCCESS == pmicStatus)
        {
            Pmic_setBitField(&regData, bitPos, bitMask, maskEnable);
            pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                                PMIC_CONFIG_1_REGADDR,
                                                regData);
        }

        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    return pmicStatus;
}
