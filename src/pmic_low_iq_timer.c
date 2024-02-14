/******************************************************************************
 * Copyright (c) 2024 Texas Instruments Incorporated - http://www.ti.com
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
*   \file    pmic_low_iq_timer.c
*
*   @brief   This file contains the default API's for PMIC FSM state
*            configuration
*/

#include "pmic_low_iq_timer.h"
#include "pmic_low_iq_timer_priv.h"

/*!
 * \brief API to set Timer Configuration
 */
int32_t Pmic_SetTimerConfig(Pmic_CoreHandle_t *pPmicCoreHandle, uint8_t  tmrData)
{
    int32_t pmicStatus  = PMIC_ST_SUCCESS;
    uint8_t regData     = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);

    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        PMIC_TMR_CFG_REG_REGADDR,
                                        &regData);

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        switch(tmrData)
        {
            case 0 :
                Pmic_setBitField(&regData,
                                 PMIC_TMR_CFG_REG_TMR_CFG_SHIFT,
                                 PMIC_TMR_CFG_REG_TMR_CFG_MASK,
                                 PMIC_TMR_CFG_DATA0);
                break;

            case 1 :
                Pmic_setBitField(&regData,
                                 PMIC_TMR_CFG_REG_TMR_CFG_SHIFT,
                                 PMIC_TMR_CFG_REG_TMR_CFG_MASK,
                                 PMIC_TMR_CFG_DATA1);
                break;

            case 2 :
                Pmic_setBitField(&regData,
                                 PMIC_TMR_CFG_REG_TMR_CFG_SHIFT,
                                 PMIC_TMR_CFG_REG_TMR_CFG_MASK,
                                 PMIC_TMR_CFG_DATA2);
                break;

            case 3 :
                Pmic_setBitField(&regData,
                                 PMIC_TMR_CFG_REG_TMR_CFG_SHIFT,
                                 PMIC_TMR_CFG_REG_TMR_CFG_MASK,
                                 PMIC_TMR_CFG_DATA3);
                break;

            case 4 :
                Pmic_setBitField(&regData,
                                 PMIC_TMR_CFG_REG_TMR_CFG_SHIFT,
                                 PMIC_TMR_CFG_REG_TMR_CFG_MASK,
                                 PMIC_TMR_CFG_DATA4);
                break;

            default :
                Pmic_setBitField(&regData,
                                 PMIC_TMR_CFG_REG_TMR_CFG_SHIFT,
                                 PMIC_TMR_CFG_REG_TMR_CFG_MASK,
                                 PMIC_TMR_CFG_DATA5);
                break;

        }

        pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                            PMIC_TMR_CFG_REG_REGADDR,
                                            regData);
    }

    Pmic_criticalSectionStop(pPmicCoreHandle);

    return pmicStatus;
}

/*!
 * \brief API to get Timer Configuration
 */
int32_t Pmic_GetTimerConfig(Pmic_CoreHandle_t *pPmicCoreHandle, uint8_t  *tmrConfigData)
{
    int32_t pmicStatus  = PMIC_ST_SUCCESS;
    uint8_t regData     = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);

    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        PMIC_TMR_CFG_REG_REGADDR,
                                        &regData);

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        *tmrConfigData = Pmic_getBitField(regData,
                                          PMIC_TMR_CFG_REG_TMR_CFG_SHIFT,
                                          PMIC_TMR_CFG_REG_TMR_CFG_MASK);
    }

    Pmic_criticalSectionStop(pPmicCoreHandle);

    return pmicStatus;
}

/*!
 * \brief API to set Timer Prescale
 */
int32_t Pmic_SetTimerPrescale(Pmic_CoreHandle_t *pPmicCoreHandle, uint8_t  tmrData)
{
    int32_t pmicStatus  = PMIC_ST_SUCCESS;
    uint8_t regData     = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);

    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        PMIC_TMR_CFG_REG_REGADDR,
                                        &regData);

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        switch(tmrData)
        {
            case 0 :
                Pmic_setBitField(&regData,
                                 PMIC_TMR_CFG_REG_TMR_PS_SHIFT,
                                 PMIC_TMR_CFG_REG_TMR_PS_MASK,
                                 PMIC_TMR_PS_DATA0);
                break;

            case 1 :
                Pmic_setBitField(&regData,
                                 PMIC_TMR_CFG_REG_TMR_PS_SHIFT,
                                 PMIC_TMR_CFG_REG_TMR_PS_MASK,
                                 PMIC_TMR_PS_DATA1);
                break;

            case 2 :
                Pmic_setBitField(&regData,
                                 PMIC_TMR_CFG_REG_TMR_PS_SHIFT,
                                 PMIC_TMR_CFG_REG_TMR_PS_MASK,
                                 PMIC_TMR_PS_DATA2);
                break;

            default :
                Pmic_setBitField(&regData,
                                 PMIC_TMR_CFG_REG_TMR_PS_SHIFT,
                                 PMIC_TMR_CFG_REG_TMR_PS_MASK,
                                 PMIC_TMR_CFG_DATA3);
                break;
        }

        pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                            PMIC_TMR_CFG_REG_REGADDR,
                                            regData);
    }

    Pmic_criticalSectionStop(pPmicCoreHandle);

    return pmicStatus;
}

/*!
 * \brief API to get Timer Prescale
 */
int32_t Pmic_GetTimerPrescale(Pmic_CoreHandle_t *pPmicCoreHandle, uint8_t  *tmrConfigData)
{
    int32_t pmicStatus  = PMIC_ST_SUCCESS;
    uint8_t regData     = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);

    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        PMIC_TMR_CFG_REG_REGADDR,
                                        &regData);

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        *tmrConfigData = Pmic_getBitField(regData,
                                          PMIC_TMR_CFG_REG_TMR_PS_SHIFT,
                                          PMIC_TMR_CFG_REG_TMR_PS_MASK);
    }

    Pmic_criticalSectionStop(pPmicCoreHandle);

    return pmicStatus;
}

/*!
 * \brief API to Clears (resets) low power timer to 0
 */
int32_t Pmic_TimerClear(Pmic_CoreHandle_t *pPmicCoreHandle)
{
    int32_t pmicStatus  = PMIC_ST_SUCCESS;
    uint8_t regData     = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);

    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        PMIC_TMR_CFG_REG_REGADDR,
                                        &regData);

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        Pmic_setBitField(&regData,
                         PMIC_TMR_CFG_REG_TMR_CLR_SHIFT,
                         PMIC_TMR_CFG_REG_TMR_CLR_MASK,
                         PMIC_TMR_CLR_DATA);
    }

    pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                        PMIC_TMR_CFG_REG_REGADDR,
                                        regData);

    Pmic_criticalSectionStop(pPmicCoreHandle);

    return pmicStatus;
}

int32_t Pmic_GetLPWake0(Pmic_CoreHandle_t *pPmicCoreHandle, uint8_t  *tmrlpwakeData)
{
    int32_t pmicStatus  = PMIC_ST_SUCCESS;
    uint8_t regData     = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);

    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        PMIC_TMR_LP_WAKE0_REGADDR,
                                        &regData);

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        *tmrlpwakeData = Pmic_getBitField(regData,
                                       PMIC_TMR_LP_WAKE0_TMR_LP_WAKE_B0_SHIFT,
                                       PMIC_TMR_LP_WAKE0_TMR_LP_WAKE_B0_MASK);
    }

    Pmic_criticalSectionStop(pPmicCoreHandle);

    return pmicStatus;
}

int32_t Pmic_SetLPWake0(Pmic_CoreHandle_t *pPmicCoreHandle, uint8_t  tmrlpwakeData)
{
    int32_t pmicStatus  = PMIC_ST_SUCCESS;
    uint8_t regData     = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);

    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        PMIC_TMR_LP_WAKE0_REGADDR,
                                        &regData);

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        Pmic_setBitField(&regData,
                         PMIC_TMR_LP_WAKE0_TMR_LP_WAKE_B0_SHIFT,
                         PMIC_TMR_LP_WAKE0_TMR_LP_WAKE_B0_MASK,
                         tmrlpwakeData);
    }

    Pmic_criticalSectionStop(pPmicCoreHandle);

    return pmicStatus;
}

int32_t Pmic_GetLPWake1(Pmic_CoreHandle_t *pPmicCoreHandle, uint8_t  *tmrlpwakeData)
{
    int32_t pmicStatus  = PMIC_ST_SUCCESS;
    uint8_t regData     = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);

    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        PMIC_TMR_LP_WAKE1_REGADDR,
                                        &regData);

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        *tmrlpwakeData = Pmic_getBitField(regData,
                                       PMIC_TMR_LP_WAKE1_TMR_LP_WAKE_B1_SHIFT,
                                       PMIC_TMR_LP_WAKE1_TMR_LP_WAKE_B1_MASK);
    }

    Pmic_criticalSectionStop(pPmicCoreHandle);

    return pmicStatus;
}

int32_t Pmic_SetLPWake1(Pmic_CoreHandle_t *pPmicCoreHandle, uint8_t  tmrlpwakeData)
{
    int32_t pmicStatus  = PMIC_ST_SUCCESS;
    uint8_t regData     = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);

    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        PMIC_TMR_LP_WAKE1_REGADDR,
                                        &regData);

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        Pmic_setBitField(&regData,
                         PMIC_TMR_LP_WAKE1_TMR_LP_WAKE_B1_SHIFT,
                         PMIC_TMR_LP_WAKE1_TMR_LP_WAKE_B1_MASK,
                         tmrlpwakeData);
    }

    Pmic_criticalSectionStop(pPmicCoreHandle);

    return pmicStatus;
}

int32_t Pmic_GetLPWake2(Pmic_CoreHandle_t *pPmicCoreHandle, uint8_t  *tmrlpwakeData)
{
    int32_t pmicStatus  = PMIC_ST_SUCCESS;
    uint8_t regData     = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);

    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        PMIC_TMR_LP_WAKE2_REGADDR,
                                        &regData);

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        *tmrlpwakeData = Pmic_getBitField(regData,
                                       PMIC_TMR_LP_WAKE2_TMR_LP_WAKE_B2_SHIFT,
                                       PMIC_TMR_LP_WAKE2_TMR_LP_WAKE_B2_MASK);
    }

    Pmic_criticalSectionStop(pPmicCoreHandle);

    return pmicStatus;
}

int32_t Pmic_SetLPWake2(Pmic_CoreHandle_t *pPmicCoreHandle, uint8_t  tmrlpwakeData)
{
    int32_t pmicStatus  = PMIC_ST_SUCCESS;
    uint8_t regData     = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);

    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        PMIC_TMR_LP_WAKE2_REGADDR,
                                        &regData);

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        Pmic_setBitField(&regData,
                         PMIC_TMR_LP_WAKE2_TMR_LP_WAKE_B2_SHIFT,
                         PMIC_TMR_LP_WAKE2_TMR_LP_WAKE_B2_MASK,
                         tmrlpwakeData);
    }

    Pmic_criticalSectionStop(pPmicCoreHandle);

    return pmicStatus;
}

int32_t Pmic_GetTimerCounter0(Pmic_CoreHandle_t *pPmicCoreHandle, uint8_t  *tmrcntData)
{
    int32_t pmicStatus  = PMIC_ST_SUCCESS;
    uint8_t regData     = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);

    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        PMIC_TMR_CNT0_REGADDR,
                                        &regData);

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        *tmrcntData = Pmic_getBitField(regData,
                                       PMIC_TMR_CNT0_TMR_CNT_B0_SHIFT,
                                       PMIC_TMR_CNT0_TMR_CNT_B0_MASK);
    }

    Pmic_criticalSectionStop(pPmicCoreHandle);

    return pmicStatus;
}

int32_t Pmic_SetTimerCounter0(Pmic_CoreHandle_t *pPmicCoreHandle, uint8_t  tmrcntData)
{
    int32_t pmicStatus  = PMIC_ST_SUCCESS;
    uint8_t regData     = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);

    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        PMIC_TMR_CNT0_REGADDR,
                                        &regData);

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        Pmic_setBitField(&regData,
                         PMIC_TMR_CNT0_TMR_CNT_B0_SHIFT,
                         PMIC_TMR_CNT0_TMR_CNT_B0_MASK,
                         tmrcntData);
    }

    Pmic_criticalSectionStop(pPmicCoreHandle);

    return pmicStatus;
}

int32_t Pmic_GetTimerCounter1(Pmic_CoreHandle_t *pPmicCoreHandle, uint8_t  *tmrcntData)
{
    int32_t pmicStatus  = PMIC_ST_SUCCESS;
    uint8_t regData     = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);

    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        PMIC_TMR_CNT1_REGADDR,
                                        &regData);

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        *tmrcntData = Pmic_getBitField(regData,
                                       PMIC_TMR_CNT1_TMR_CNT_B1_SHIFT,
                                       PMIC_TMR_CNT1_TMR_CNT_B1_MASK);
    }

    Pmic_criticalSectionStop(pPmicCoreHandle);

    return pmicStatus;
}

int32_t Pmic_SetTimerCounter1(Pmic_CoreHandle_t *pPmicCoreHandle, uint8_t  tmrcntData)
{
    int32_t pmicStatus  = PMIC_ST_SUCCESS;
    uint8_t regData     = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);

    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        PMIC_TMR_CNT1_REGADDR,
                                        &regData);

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        Pmic_setBitField(&regData,
                         PMIC_TMR_CNT1_TMR_CNT_B1_SHIFT,
                         PMIC_TMR_CNT1_TMR_CNT_B1_MASK,
                         tmrcntData);
    }

    Pmic_criticalSectionStop(pPmicCoreHandle);

    return pmicStatus;
}
int32_t Pmic_GetTimerCounter2(Pmic_CoreHandle_t *pPmicCoreHandle, uint8_t  *tmrcntData)
{
    int32_t pmicStatus  = PMIC_ST_SUCCESS;
    uint8_t regData     = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);

    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        PMIC_TMR_CNT2_REGADDR,
                                        &regData);

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        *tmrcntData = Pmic_getBitField(regData,
                                       PMIC_TMR_CNT2_TMR_CNT_B2_SHIFT,
                                       PMIC_TMR_CNT2_TMR_CNT_B2_MASK);
    }

    Pmic_criticalSectionStop(pPmicCoreHandle);

    return pmicStatus;
}

int32_t Pmic_SetTimerCounter2(Pmic_CoreHandle_t *pPmicCoreHandle, uint8_t  tmrcntData)
{
    int32_t pmicStatus  = PMIC_ST_SUCCESS;
    uint8_t regData     = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);

    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        PMIC_TMR_CNT2_REGADDR,
                                        &regData);

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        Pmic_setBitField(&regData,
                         PMIC_TMR_CNT2_TMR_CNT_B2_SHIFT,
                         PMIC_TMR_CNT2_TMR_CNT_B2_MASK,
                         tmrcntData);
    }

    Pmic_criticalSectionStop(pPmicCoreHandle);

    return pmicStatus;
}