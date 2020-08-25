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
 * \brief  API to initiate OFF Request FSM transition
 *         This function initiate OFF Request FSM transition from any other
 *         mission state to the STANDBY state or the LP_STANDBY state
 *
 * \param   pPmicCoreHandle   [IN]    PMIC Interface Handle.
 * \param   eventType         [IN]    Event Type used to initiate OFF Request
 * \param   fsmState          [IN]    FSM state
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_fsmDeviceOffRequestCfg(Pmic_CoreHandle_t  *pPmicCoreHandle,
                                    uint8_t             eventType,
                                    bool                fsmState)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    uint8_t regAddr = 0U;
    uint8_t bitPos = 0U;
    uint8_t bitMask = 0U;

    if(NULL == pPmicCoreHandle)
    {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (eventType != PMIC_FSM_I2C_TRIGGER0_EVENT_TYPE))
    {
        pmicStatus = PMIC_ST_ERR_INV_PARAM;
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        Pmic_criticalSectionStart(pPmicCoreHandle);

        if(eventType == PMIC_FSM_I2C_TRIGGER0_EVENT_TYPE)
        {

            pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                                PMIC_CONFIG_1_REGADDR,
                                                &regData);
            if(PMIC_ST_SUCCESS == pmicStatus)
            {
                Pmic_setBitField(&regData,
                                 PMIC_CONFIG_1_NSLEEP1_MASK_SHIFT,
                                 PMIC_CONFIG_1_NSLEEP1_MASK_MASK,
                                 PMIC_NSLEEP1B_FSM_MASK);

                Pmic_setBitField(&regData,
                                 PMIC_CONFIG_1_NSLEEP2_MASK_SHIFT,
                                 PMIC_CONFIG_1_NSLEEP2_MASK_MASK,
                                 PMIC_NSLEEP2B_FSM_MASK);

                pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                                    PMIC_CONFIG_1_REGADDR,
                                                    regData);
            }
        }

        if(PMIC_ST_SUCCESS == pmicStatus)
        {
            pmicStatus = Pmic_fsmGetstandByCfgRegFields(
                                                pPmicCoreHandle->pmicDeviceType,
                                                &regAddr,
                                                &bitPos,
                                                &bitMask);
        }

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
    return pmicStatus;
}

/*!
 * \brief  API to initiate OFF Request FSM transition
 *         This function initiate OFF Request FSM transition from any other
 *         mission state to the STANDBY state or the LP_STANDBY state
 *
 * \param   pPmicCoreHandle   [IN]    PMIC Interface Handle.
 * \param   i2cTriggerType    [IN]    FSM I2C Trigger type use to initiate OFF
 *                                    Request
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_fsmEnableI2cTrigger(Pmic_CoreHandle_t  *pPmicCoreHandle,
                                 bool                i2cTriggerType)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    if(NULL == pPmicCoreHandle)
    {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (PMIC_FSM_I2C_TRIGGER0_TYPE != i2cTriggerType))
    {
        pmicStatus = PMIC_ST_ERR_INV_PARAM;
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        Pmic_criticalSectionStart(pPmicCoreHandle);

        if(PMIC_ST_SUCCESS == pmicStatus)
        {
            pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                                PMIC_FSM_I2C_TRIGGERS_REGADDR,
                                                &regData);
        }

        if(PMIC_ST_SUCCESS == pmicStatus)
        {
            Pmic_setBitField(&regData,
                             PMIC_FSM_I2C_TRIGGERS_TRIGGER_I2C_0_SHIFT,
                             PMIC_FSM_I2C_TRIGGERS_TRIGGER_I2C_0_MASK,
                             PMIC_FSM_I2C_TRIGGER_VAL);

            pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                                PMIC_FSM_I2C_TRIGGERS_REGADDR,
                                                regData);
        }

        Pmic_criticalSectionStop(pPmicCoreHandle);

    }
    return pmicStatus;
}

