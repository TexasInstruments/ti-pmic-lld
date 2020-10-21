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
 *   @file    pmic_wdg.c
 *
 *   @brief   This file contains the API definitions for PMIC Watchdog
 *            configuration
 *
 */

#include <pmic_wdg_priv.h>
#include <pmic_core_priv.h>

/*
 * \brief  Function to Check pmic core handle
 */
static int32_t Pmic_checkPmicCoreHandle(
                             const Pmic_CoreHandle_t *pPmicCoreHandle)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint32_t drvInitStatus = 0U;

    /* Validate pPmicCoreHandle */
    if(NULL == pPmicCoreHandle)
    {
        status = PMIC_ST_ERR_INV_HANDLE;
    }

    if(PMIC_ST_SUCCESS == status)
    {
        if(PMIC_INTF_SINGLE_I2C == pPmicCoreHandle->commMode)
        {
            drvInitStatus = DRV_INIT_SUCCESS | PMIC_MAIN_INST;
        }
        else if(PMIC_INTF_DUAL_I2C == pPmicCoreHandle->commMode)
        {
            drvInitStatus = DRV_INIT_SUCCESS | PMIC_MAIN_INST | PMIC_QA_INST;
        }
        else if(PMIC_INTF_SPI == pPmicCoreHandle->commMode)
        {
            drvInitStatus = DRV_INIT_SUCCESS | PMIC_MAIN_INST;
        }
        else
        {
            drvInitStatus = 0x00U;
        }

        if(drvInitStatus != pPmicCoreHandle->drvInitStatus)
        {
            status = PMIC_ST_ERR_INV_HANDLE;
        }
    }

    return status;
}

/*
 * \brief  Function to Check watchdog subsystem and pmic core handle
 */
static int32_t Pmic_WdgValidatePmicCoreHandle(
                                   const Pmic_CoreHandle_t *pPmicCoreHandle)
{
    int32_t status = PMIC_ST_SUCCESS;

    status = Pmic_checkPmicCoreHandle(pPmicCoreHandle);

    /* Check the watch dog sub-system supported by pmic device */
    if(PMIC_ST_SUCCESS == status)
    {
        if(((bool)true) != pPmicCoreHandle->pPmic_SubSysInfo->wdgEnable)
        {
            status = PMIC_ST_ERR_INV_DEVICE;
        }
    }

    return status;
}

/*
 * \brief  Function to set watchdog windows time intervals
 */
static int32_t Pmic_WdgSetWindowsTimeIntervals(
                    Pmic_CoreHandle_t   *pPmicCoreHandle,
                    const Pmic_WdgCfg_t  wdgCfg)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regVal = 0U;

    /* Set wdg long window time interval */
    if(((bool)true) == pmic_validParamCheck(wdgCfg.validParams,
                              PMIC_CFG_WDG_LONGWINDURATION_VALID))
    {
        if((PMIG_WD_LONGWIN_100_MILLISEC != wdgCfg.longWinDuration_ms) &&
           ((PMIG_WD_LONGWIN_MILLISEC_MIN > wdgCfg.longWinDuration_ms) ||
            (PMIG_WD_LONGWIN_MILLISEC_MAX < wdgCfg.longWinDuration_ms)))
        {
            status = PMIC_ST_ERR_INV_WDG_WINDOW;
        }
        if(PMIC_ST_SUCCESS == status)
        {
            if(PMIG_WD_LONGWIN_100_MILLISEC == wdgCfg.longWinDuration_ms)
            {
                regVal = 0x00U;
            }
            else
            {
                regVal = (uint8_t)(wdgCfg.longWinDuration_ms /
                                PMIG_WD_LONGWIN_MILLISEC_DIV);
            }

            Pmic_criticalSectionStart(pPmicCoreHandle);

            status = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                            PMIC_WD_LONGWIN_CFG_REGADDR,
                                            regVal);

            Pmic_criticalSectionStop(pPmicCoreHandle);
        }
    }

    /* Set wdg window1 time interval */
    if((PMIC_ST_SUCCESS == status) &&
       (((bool)true) == pmic_validParamCheck(wdgCfg.validParams,
                                PMIC_CFG_WDG_WIN1DURATION_VALID)))
    {
        if((PMIG_WD_WIN1_2_MICROSEC_MIN > wdgCfg.win1Duration_us) ||
           (PMIG_WD_WIN1_2_MICROSEC_MAX < wdgCfg.win1Duration_us))
        {
            status = PMIC_ST_ERR_INV_WDG_WINDOW;
        }
        if(PMIC_ST_SUCCESS == status)
        {
            regVal = ((uint8_t)(wdgCfg.win1Duration_us /
                             PMIG_WD_WIN1_2_MICROSEC_DIV) - 1U);
            regVal &= PMIC_WD_WIN1_CFG_WD_WIN1_MASK;

            Pmic_criticalSectionStart(pPmicCoreHandle);

            status = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                            PMIC_WD_WIN1_CFG_REGADDR,
                                            regVal);

            Pmic_criticalSectionStop(pPmicCoreHandle);
        }
    }

    /* Set wdg window2 time interval */
    if((PMIC_ST_SUCCESS == status) &&
       (((bool)true) == pmic_validParamCheck(wdgCfg.validParams,
                                PMIC_CFG_WDG_WIN2DURATION_VALID)))
    {
        if((PMIG_WD_WIN1_2_MICROSEC_MIN > wdgCfg.win2Duration_us) ||
           (PMIG_WD_WIN1_2_MICROSEC_MAX < wdgCfg.win2Duration_us))
        {
            status = PMIC_ST_ERR_INV_WDG_WINDOW;
        }
        if(PMIC_ST_SUCCESS == status)
        {
            regVal = ((uint8_t)(wdgCfg.win1Duration_us /
                             PMIG_WD_WIN1_2_MICROSEC_DIV) - 1U);
            regVal &= PMIC_WD_WIN2_CFG_WD_WIN2_MASK;

            Pmic_criticalSectionStart(pPmicCoreHandle);

            status = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                            PMIC_WD_WIN2_CFG_REGADDR,
                                            regVal);

            Pmic_criticalSectionStop(pPmicCoreHandle);
        }
    }

    return status;
}

/*
 * \brief  Function to get watchdog windows time intervals
 */
static int32_t Pmic_WdgGetWindowsTimeIntervals(
                    Pmic_CoreHandle_t   *pPmicCoreHandle,
                    Pmic_WdgCfg_t       *pWdgCfg)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regVal = 0U;

    /* Set wdg long window time interval */
    if(((bool)true) == pmic_validParamCheck(pWdgCfg->validParams,
                              PMIC_CFG_WDG_LONGWINDURATION_VALID))
    {
        Pmic_criticalSectionStart(pPmicCoreHandle);

        status = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        PMIC_WD_LONGWIN_CFG_REGADDR,
                                        &regVal);

        Pmic_criticalSectionStop(pPmicCoreHandle);

        if(PMIC_ST_SUCCESS == status)
        {
            if(0x0U == regVal)
            {
                pWdgCfg->longWinDuration_ms = PMIG_WD_LONGWIN_100_MILLISEC;
            }
            else
            {
                pWdgCfg->longWinDuration_ms = ((uint32_t)regVal *
                                            PMIG_WD_LONGWIN_MILLISEC_DIV);
            }
        }
    }

    /* Get wdg window1 time interval */
    if((PMIC_ST_SUCCESS == status) &&
       (((bool)true) == pmic_validParamCheck(pWdgCfg->validParams,
                                PMIC_CFG_WDG_WIN1DURATION_VALID)))
    {
        Pmic_criticalSectionStart(pPmicCoreHandle);

        status = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        PMIC_WD_WIN1_CFG_REGADDR,
                                        &regVal);

        Pmic_criticalSectionStop(pPmicCoreHandle);

        if(PMIC_ST_SUCCESS == status)
        {
            regVal &= PMIC_WD_WIN1_CFG_WD_WIN1_MASK;

            pWdgCfg->win1Duration_us = (((uint32_t)regVal) + 1U) *
                                       ((uint32_t)PMIG_WD_WIN1_2_MICROSEC_DIV);
        }
    }

    /* Get wdg window2 time interval */
    if((PMIC_ST_SUCCESS == status) &&
       (((bool)true) == pmic_validParamCheck(pWdgCfg->validParams,
                                PMIC_CFG_WDG_WIN2DURATION_VALID)))
    {
        Pmic_criticalSectionStart(pPmicCoreHandle);

        status = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        PMIC_WD_WIN2_CFG_REGADDR,
                                        &regVal);

        Pmic_criticalSectionStop(pPmicCoreHandle);

        if(PMIC_ST_SUCCESS == status)
        {
            regVal &= PMIC_WD_WIN2_CFG_WD_WIN2_MASK;

            pWdgCfg->win2Duration_us = (((uint32_t)regVal) + 1U) *
                                       ((uint32_t)PMIG_WD_WIN1_2_MICROSEC_DIV);
        }
    }

    return status;
}

/*
 * \brief  Function to set watchdog Threshold values
 */
static int32_t Pmic_WdgSetThresholdValues(Pmic_CoreHandle_t  *pPmicCoreHandle,
                                          const Pmic_WdgCfg_t wdgCfg)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regVal = 0U;

    /* Set wdg fail threshold value */
    if(((bool)true) == pmic_validParamCheck(wdgCfg.validParams,
                              PMIC_CFG_WDG_FAILTHRESHOLD_VALID))
    {
        if(wdgCfg.failThreshold > PMIC_WDG_FAIL_THRESHOLD_COUNT_7)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }

        Pmic_criticalSectionStart(pPmicCoreHandle);

        if(PMIC_ST_SUCCESS == status)
        {
            status = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                            PMIC_WD_THR_CFG_REGADDR,
                                            &regVal);
        }
        if(PMIC_ST_SUCCESS == status)
        {
            Pmic_setBitField(&regVal,
                             PMIC_WD_THR_CFG_WD_FAIL_TH_SHIFT,
                             PMIC_WD_THR_CFG_WD_FAIL_TH_MASK,
                             wdgCfg.failThreshold);

            status = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                            PMIC_WD_THR_CFG_REGADDR,
                                            regVal);
        }

        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    /* Set wdg reset threshold value */
    if((PMIC_ST_SUCCESS == status) &&
       (((bool)true) == pmic_validParamCheck(wdgCfg.validParams,
                              PMIC_CFG_WDG_RSTTHRESHOLD_VALID)))
    {
        if(wdgCfg.rstThreshold > PMIC_WDG_RESET_THRESHOLD_COUNT_7)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }

        Pmic_criticalSectionStart(pPmicCoreHandle);

        if(PMIC_ST_SUCCESS == status)
        {
            status = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                            PMIC_WD_THR_CFG_REGADDR,
                                            &regVal);
        }

        if(PMIC_ST_SUCCESS == status)
        {
            Pmic_setBitField(&regVal,
                             PMIC_WD_THR_CFG_WD_RST_TH_SHIFT,
                             PMIC_WD_THR_CFG_WD_RST_TH_MASK,
                             wdgCfg.rstThreshold);

            status = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                            PMIC_WD_THR_CFG_REGADDR,
                                            regVal);
        }

        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    return status;
}

/*
 * \brief  Function to get watchdog Threshold values
 */
static int32_t Pmic_WdgGetThresholdValues(Pmic_CoreHandle_t  *pPmicCoreHandle,
                                          Pmic_WdgCfg_t      *pWdgCfg)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regVal = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);

    status = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                    PMIC_WD_THR_CFG_REGADDR,
                                    &regVal);

    Pmic_criticalSectionStop(pPmicCoreHandle);

    /* Get wdg fail threshold value */
    if((PMIC_ST_SUCCESS == status) &&
       (((bool)true) == pmic_validParamCheck(pWdgCfg->validParams,
                              PMIC_CFG_WDG_FAILTHRESHOLD_VALID)))
    {
        pWdgCfg->failThreshold = Pmic_getBitField(
                                               regVal,
                                               PMIC_WD_THR_CFG_WD_FAIL_TH_SHIFT,
                                               PMIC_WD_THR_CFG_WD_FAIL_TH_MASK);

    }

    /* Get wdg reset threshold value */
    if((PMIC_ST_SUCCESS == status) &&
       (((bool)true) == pmic_validParamCheck(pWdgCfg->validParams,
                              PMIC_CFG_WDG_RSTTHRESHOLD_VALID)))
    {
        pWdgCfg->rstThreshold = Pmic_getBitField(
                                               regVal,
                                               PMIC_WD_THR_CFG_WD_RST_TH_SHIFT,
                                               PMIC_WD_THR_CFG_WD_RST_TH_MASK);
    }

    /* Get wdg warm reset value */
    if((PMIC_ST_SUCCESS == status) &&
       (((bool)true) == pmic_validParamCheck(pWdgCfg->validParams,
                                     PMIC_CFG_WDG_RSTENABLE_VALID)))
    {
        if(Pmic_getBitField(
                            regVal,
                            PMIC_WD_THR_CFG_WD_RST_EN_SHIFT,
                            PMIC_WD_THR_CFG_WD_RST_EN_MASK) != 0U)
        {
            pWdgCfg->rstEnable = (bool)true;
        }
        else
        {
            pWdgCfg->rstEnable = (bool)false;
        }

    }

    return status;
}

/*
 * \brief  Function to set watchdog return long window control
 */
static int32_t Pmic_WdgSetRetToLongWindow(Pmic_CoreHandle_t  *pPmicCoreHandle,
                                          bool                enable)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regVal = 0U;
    uint8_t enableVal = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);

    status = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                    PMIC_WD_MODE_REG_REGADDR,
                                    &regVal);

    if(PMIC_ST_SUCCESS == status)
    {
        if(((bool)true) == enable)
        {
            enableVal = 1U;
        }

        Pmic_setBitField(&regVal,
                         PMIC_WD_MODE_REG_WD_RETURN_LONGWIN_SHIFT,
                         PMIC_WD_MODE_REG_WD_RETURN_LONGWIN_MASK,
                         enableVal);

        status = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                        PMIC_WD_MODE_REG_REGADDR,
                                        regVal);
    }

    Pmic_criticalSectionStop(pPmicCoreHandle);
    return status;
}

/*
 * \brief  Function to set watchdog control parameters
 */
static int32_t Pmic_WdgSetCtrlParams(Pmic_CoreHandle_t  *pPmicCoreHandle,
                                     const Pmic_WdgCfg_t wdgCfg)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regVal = 0U;
    uint8_t pwrHoldVal = 0U;
    uint8_t wdgModeVal = 0U;
    uint8_t rstEnableVal = 0U;

    /* Set wdg mode */
    if(((bool)true) == pmic_validParamCheck(wdgCfg.validParams,
                              PMIC_CFG_WDG_WDGMODE_VALID))
    {

        Pmic_criticalSectionStart(pPmicCoreHandle);

        status = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        PMIC_WD_MODE_REG_REGADDR,
                                        &regVal);
        if(PMIC_ST_SUCCESS == status)
        {
            if(((bool)true) == wdgCfg.wdgMode)
            {
                wdgModeVal = 1U;
            }

            Pmic_setBitField(&regVal,
                             PMIC_WD_MODE_REG_WD_MODE_SELECT_SHIFT,
                             PMIC_WD_MODE_REG_WD_MODE_SELECT_MASK,
                             wdgModeVal);

            status = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                            PMIC_WD_MODE_REG_REGADDR,
                                            regVal);
        }

        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    /* Set wdg power hold vale */
    if((PMIC_ST_SUCCESS == status) &&
       (((bool)true) == pmic_validParamCheck(wdgCfg.validParams,
                                     PMIC_CFG_WDG_PWRHOLD_VALID)))
    {

        Pmic_criticalSectionStart(pPmicCoreHandle);

        status = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        PMIC_WD_MODE_REG_REGADDR,
                                        &regVal);

        if(PMIC_ST_SUCCESS == status)
        {
            if(((bool)true) == wdgCfg.pwrHold)
            {
                pwrHoldVal = 1U;
            }

            Pmic_setBitField(&regVal,
                             PMIC_WD_MODE_REG_WD_PWRHOLD_SHIFT,
                             PMIC_WD_MODE_REG_WD_PWRHOLD_MASK,
                             pwrHoldVal);

            status = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                            PMIC_WD_MODE_REG_REGADDR,
                                            regVal);
        }

        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    /* Set wdg warm reset enable value */
    if((PMIC_ST_SUCCESS == status) &&
       (((bool)true) == pmic_validParamCheck(wdgCfg.validParams,
                                     PMIC_CFG_WDG_RSTENABLE_VALID)))
    {

        Pmic_criticalSectionStart(pPmicCoreHandle);

        status = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        PMIC_WD_THR_CFG_REGADDR,
                                        &regVal);

        if(PMIC_ST_SUCCESS == status)
        {
            if(((bool)true) == wdgCfg.rstEnable)
            {
                rstEnableVal = 1U;
            }

            Pmic_setBitField(&regVal,
                             PMIC_WD_THR_CFG_WD_RST_EN_SHIFT,
                             PMIC_WD_THR_CFG_WD_RST_EN_MASK,
                             rstEnableVal);

            status = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                            PMIC_WD_THR_CFG_REGADDR,
                                            regVal);
        }

        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    /* Set wdg return to long window bit */
    if((PMIC_ST_SUCCESS == status) &&
       (((bool)true) == pmic_validParamCheck(wdgCfg.validParams,
                                     PMIC_CFG_WDG_RETLONGWIN_VALID)))
    {

        status = Pmic_WdgSetRetToLongWindow(pPmicCoreHandle,
                                            wdgCfg.retLongWin);
    }

    return status;
}

/*
 * \brief  Function to get watchdog control parameters
 */
static int32_t Pmic_WdgGetCtrlParams(Pmic_CoreHandle_t  *pPmicCoreHandle,
                                     Pmic_WdgCfg_t      *pWdgCfg)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regVal = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);

    status = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                    PMIC_WD_MODE_REG_REGADDR,
                                    &regVal);

    Pmic_criticalSectionStop(pPmicCoreHandle);

    /* Get wdg mode */
    if((PMIC_ST_SUCCESS == status) &&
       (((bool)true) == pmic_validParamCheck(pWdgCfg->validParams,
                                     PMIC_CFG_WDG_WDGMODE_VALID)))
    {
         if(Pmic_getBitField(regVal,
                             PMIC_WD_MODE_REG_WD_MODE_SELECT_SHIFT,
                             PMIC_WD_MODE_REG_WD_MODE_SELECT_MASK) != 0U)
        {
            pWdgCfg->wdgMode = (bool)true;
        }
        else
        {
            pWdgCfg->wdgMode = (bool)false;
        }

    }

    /* Get wdg power hold vale */
    if((PMIC_ST_SUCCESS == status) &&
       (((bool)true) == pmic_validParamCheck(pWdgCfg->validParams,
                                     PMIC_CFG_WDG_PWRHOLD_VALID)))
    {
        if(Pmic_getBitField(regVal,
                            PMIC_WD_MODE_REG_WD_PWRHOLD_SHIFT,
                            PMIC_WD_MODE_REG_WD_PWRHOLD_MASK) != 0U)
        {
            pWdgCfg->pwrHold = (bool)true;
        }
        else
        {
            pWdgCfg->pwrHold = (bool)false;
        }

    }

    /* Get wdg return to long window bit */
    if((PMIC_ST_SUCCESS == status) &&
       (((bool)true) == pmic_validParamCheck(pWdgCfg->validParams,
                                     PMIC_CFG_WDG_RETLONGWIN_VALID)))
    {
        if(Pmic_getBitField(regVal,
                            PMIC_WD_MODE_REG_WD_RETURN_LONGWIN_SHIFT,
                            PMIC_WD_MODE_REG_WD_RETURN_LONGWIN_MASK) != 0U)
        {
            pWdgCfg->retLongWin = (bool)true;
        }
        else
        {
            pWdgCfg->retLongWin = (bool)false;
        }

    }

    return status;
}

/*
 * \brief  Function to set watchdog QA configurations
 */
static int32_t Pmic_WdgSetQaConfigurations(
                            Pmic_CoreHandle_t  *pPmicCoreHandle,
                            const Pmic_WdgCfg_t wdgCfg)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regVal = 0U;

    /* Set wdg QA Feedback value */
    if(((bool)true) == pmic_validParamCheck(wdgCfg.validParams,
                              PMIC_CFG_WDG_QA_FDBK_VALID))
    {
        if(wdgCfg.qaFdbk > PMIC_WDG_QA_FEEDBACK_VALUE_3)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }

        Pmic_criticalSectionStart(pPmicCoreHandle);

        if(PMIC_ST_SUCCESS == status)
        {
            status = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                            PMIC_WD_QA_CFG_REGADDR,
                                            &regVal);
        }
        if(PMIC_ST_SUCCESS == status)
        {
            Pmic_setBitField(&regVal,
                             PMIC_WD_QA_CFG_WD_QA_FDBK_SHIFT,
                             PMIC_WD_QA_CFG_WD_QA_FDBK_MASK,
                             wdgCfg.qaFdbk);

            status = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                            PMIC_WD_QA_CFG_REGADDR,
                                            regVal);
        }

        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    /* Set wdg QA LFSR value */
    if((PMIC_ST_SUCCESS == status) &&
       (((bool)true) == pmic_validParamCheck(wdgCfg.validParams,
                               PMIC_CFG_WDG_QA_LFSR_VALID)))
    {
        if(wdgCfg.qaLfsr > PMIC_WDG_QA_LFSR_VALUE_3)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }

        Pmic_criticalSectionStart(pPmicCoreHandle);

        if(PMIC_ST_SUCCESS == status)
        {
            status = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                            PMIC_WD_QA_CFG_REGADDR,
                                            &regVal);
        }

        if(PMIC_ST_SUCCESS == status)
        {
            Pmic_setBitField(&regVal,
                             PMIC_WD_QA_CFG_WD_QA_LFSR_SHIFT,
                             PMIC_WD_QA_CFG_WD_QA_LFSR_MASK,
                             wdgCfg.qaLfsr);

            status = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                            PMIC_WD_QA_CFG_REGADDR,
                                            regVal);
        }

        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    /* Set wdg QA Question Seed value */
    if((PMIC_ST_SUCCESS == status) &&
       (((bool)true) == pmic_validParamCheck(wdgCfg.validParams,
                               PMIC_CFG_WDG_QA_QUES_SEED_VALID)))
    {
        if(wdgCfg.qaQuesSeed > PMIC_WDG_QA_QUES_SEED_VALUE_15)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }

        Pmic_criticalSectionStart(pPmicCoreHandle);

        if(PMIC_ST_SUCCESS == status)
        {
            status = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                            PMIC_WD_QA_CFG_REGADDR,
                                            &regVal);
        }

        if(PMIC_ST_SUCCESS == status)
        {
            Pmic_setBitField(&regVal,
                             PMIC_WD_QA_CFG_WD_QUESTION_SEED_SHIFT,
                             PMIC_WD_QA_CFG_WD_QUESTION_SEED_MASK,
                             wdgCfg.qaQuesSeed);

            status = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                            PMIC_WD_QA_CFG_REGADDR,
                                            regVal);
        }

        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    return status;
}

/*
 * \brief  Function to get watchdog QA configurations
 */
static int32_t Pmic_WdgGetQaConfigurations(
                            Pmic_CoreHandle_t  *pPmicCoreHandle,
                            Pmic_WdgCfg_t      *pWdgCfg)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regVal = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);

    status = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                    PMIC_WD_QA_CFG_REGADDR,
                                    &regVal);

    Pmic_criticalSectionStop(pPmicCoreHandle);

    /* Get wdg QA Feedback value */
    if((PMIC_ST_SUCCESS == status) &&
       (((bool)true) == pmic_validParamCheck(pWdgCfg->validParams,
                              PMIC_CFG_WDG_QA_FDBK_VALID)))
    {
        pWdgCfg->qaFdbk =
                        Pmic_getBitField(regVal,
                                         PMIC_WD_QA_CFG_WD_QA_FDBK_SHIFT,
                                         PMIC_WD_QA_CFG_WD_QA_FDBK_MASK);
    }

    /* Get wdg QA LFSR value */
    if((PMIC_ST_SUCCESS == status) &&
       (((bool)true) == pmic_validParamCheck(pWdgCfg->validParams,
                               PMIC_CFG_WDG_QA_LFSR_VALID)))
    {
        pWdgCfg->qaLfsr =
                        Pmic_getBitField(regVal,
                                         PMIC_WD_QA_CFG_WD_QA_LFSR_SHIFT,
                                         PMIC_WD_QA_CFG_WD_QA_LFSR_MASK);
    }

    /* Get wdg QA Question Seed value */
    if((PMIC_ST_SUCCESS == status) &&
       (((bool)true) == pmic_validParamCheck(pWdgCfg->validParams,
                               PMIC_CFG_WDG_QA_QUES_SEED_VALID)))
    {
        pWdgCfg->qaQuesSeed =
                        Pmic_getBitField(regVal,
                                         PMIC_WD_QA_CFG_WD_QUESTION_SEED_SHIFT,
                                         PMIC_WD_QA_CFG_WD_QUESTION_SEED_MASK);
    }

    return status;
}

/*
 * \brief  Function to Enable/Disable Watchdog Timer
 */
static int32_t Pmic_wdgEnDisState(Pmic_CoreHandle_t *pPmicCoreHandle,
                                  bool               enable)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regVal = 0U;
    uint8_t enableVal = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);

    status = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                    PMIC_WD_THR_CFG_REGADDR,
                                    &regVal);

    if(PMIC_ST_SUCCESS == status)
    {
        if(((bool)true) == enable)
        {
            enableVal = 1U;
        }

        Pmic_setBitField(&regVal,
                         PMIC_WD_THR_CFG_WD_EN_SHIFT,
                         PMIC_WD_THR_CFG_WD_EN_MASK,
                         enableVal);

        status = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                        PMIC_WD_THR_CFG_REGADDR,
                                        regVal);
    }

    Pmic_criticalSectionStop(pPmicCoreHandle);

    return status;
}


/*
 * \brief  Function to get watchdog QA answer count and question value
 */
static int32_t Pmic_wdgReadQuesandAnswerCount(
                                  Pmic_CoreHandle_t *pPmicCoreHandle,
                                  uint8_t           *pQaAnsCnt,
                                  uint8_t           *pQaQuesCnt)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regVal = 0U;

    /*! Start Critical Section */
    Pmic_criticalSectionStart(pPmicCoreHandle);

    /*! Reading answer count and question value */
    status = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                    PMIC_WD_QUESTION_ANSW_CNT_REGADDR,
                                    &regVal);

    /*! Stop Critical Section */
    Pmic_criticalSectionStop(pPmicCoreHandle);

    if(PMIC_ST_SUCCESS == status)
    {
        *pQaAnsCnt = Pmic_getBitField(
                                  regVal,
                                  PMIC_WD_QUESTION_ANSW_CNT_WD_ANSW_CNT_SHIFT,
                                  PMIC_WD_QUESTION_ANSW_CNT_WD_ANSW_CNT_MASK);
        *pQaQuesCnt = Pmic_getBitField(
                                  regVal,
                                  PMIC_WD_QUESTION_ANSW_CNT_WD_QUESTION_SHIFT,
                                  PMIC_WD_QUESTION_ANSW_CNT_WD_QUESTION_MASK);
    }

    return status;
}

/*
 * \brief  Function to get watchdog bad event
 */
static bool is_wdgBadEventDetected(Pmic_CoreHandle_t *pPmicCoreHandle)
{
    uint8_t regVal = 0U;
    bool bitFieldVal = (bool)false;
    int32_t status = PMIC_ST_SUCCESS;

    status = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                    PMIC_WD_FAIL_CNT_REG_REGADDR,
                                    &regVal);

    if((PMIC_ST_SUCCESS == status) &&
       (Pmic_getBitField(regVal,
                        PMIC_WD_FAIL_CNT_REG_WD_BAD_EVENT_SHIFT,
                        PMIC_WD_FAIL_CNT_REG_WD_BAD_EVENT_MASK) != 0U))
    {
        bitFieldVal = (bool)true;
    }

    return bitFieldVal;
}

/*
 * \brief  Function to get 4X1 mux output
 */
static uint8_t mux_4x1(uint8_t x0,
                       uint8_t x1,
                       uint8_t x2,
                       uint8_t x3,
                       uint8_t qaFdbk)
{
    uint8_t y = 0U;

    switch(qaFdbk)
    {
        case 0U:
            y = x0;
            break;
        case 1U:
            y = x1;
            break;
        case 2U:
            y = x2;
            break;
        case 3U:
            y = x3;
            break;
        default:
            break;
    }

    return y;
}

/*
 * \brief  Function to Evaluate Watchdog Answers
 */
static uint8_t Pmic_getAnswerByte(uint8_t qaQuesCnt,
                                  uint8_t qaAnsCnt,
                                  uint8_t qaFdbk)
{
    uint8_t q0 = 0U, q1 = 0U, q2 = 0U, q3 = 0U;
    uint8_t a0 = 0U, a1 = 0U;
    uint8_t qaAns = 0U;

    q0 = ((qaQuesCnt >> 0U) & 1U);
    q1 = ((qaQuesCnt >> 1U) & 1U);
    q2 = ((qaQuesCnt >> 2U) & 1U);
    q3 = ((qaQuesCnt >> 3U) & 1U);

    a0 = ((qaAnsCnt >> 0U) & 1U);
    a1 = ((qaAnsCnt >> 1U) & 1U);

    /* Reference-Answer-X[0] */
    qaAns = (mux_4x1(q0, q1, q2, q3, qaFdbk)
                      ^ mux_4x1(q3, q0, q1, q2, qaFdbk)
                      ^ a1);
    /* Reference-Answer-X[1] */
    qaAns |= (((mux_4x1(q0, q1, q2, q3, qaFdbk)
                         ^ mux_4x1(q2, q0, q1, q3, qaFdbk)
                         ^ a1 ^ q1)) << 1U);
    /* Reference-Answer-X[2] */
    qaAns |= (((mux_4x1(q0, q1, q2, q3, qaFdbk)
                         ^ mux_4x1(q3, q0, q1, q2, qaFdbk)
                         ^ a1 ^ q1)) << 2U);
    /* Reference-Answer-X[3] */
    qaAns |= (((mux_4x1(q0, q1, q2, q3, qaFdbk)
                         ^ mux_4x1(q2, q0, q1, q3, qaFdbk)
                         ^ a1 ^ q3)) << 3U);
    /* Reference-Answer-X[4] */
    qaAns |= (((mux_4x1(q1, q0, q2, q3, qaFdbk)
                         ^ a0)) << 4U);
    /* Reference-Answer-X[5] */
    qaAns |= (((mux_4x1(q3, q0, q1, q2, qaFdbk)
                         ^ a0)) << 5U);
    /* Reference-Answer-X[6] */
    qaAns |= (((mux_4x1(q0, q1, q2, q3, qaFdbk)
                         ^ a0)) << 6U);
    /* Reference-Answer-X[7] */
    qaAns |= (((mux_4x1(q2, q0, q1, q2, qaFdbk)
                         ^ a0)) << 7U);

    return qaAns;
}

/*
 * \brief  Function to Evaluate and write Watchdog Answers
 */
static int32_t Pmic_wdgQaWriteAnswers(Pmic_CoreHandle_t *pPmicCoreHandle)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t qaAnsCnt   = 0U;
    uint8_t qaQuesCnt  = 0U;
    uint8_t qaFdbk     = 0U;
    int8_t  ansIndex   = 0;
    uint8_t answer     = 0U;

    /* Start Critical Section */
    Pmic_criticalSectionStart(pPmicCoreHandle);

    status = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                    PMIC_WD_QA_CFG_REGADDR,
                                    &qaFdbk);

    /* Stop Critical Section */
    Pmic_criticalSectionStop(pPmicCoreHandle);

    /* Get wdg QA Feedback value */
    if(PMIC_ST_SUCCESS == status)
    {
        qaFdbk = Pmic_getBitField(qaFdbk,
                                  PMIC_WD_QA_CFG_WD_QA_FDBK_SHIFT,
                                  PMIC_WD_QA_CFG_WD_QA_FDBK_MASK);
    }

    for(ansIndex = 3; ansIndex >= 0; ansIndex--)
    {
        if(PMIC_ST_SUCCESS == status)
        {
            status = Pmic_wdgReadQuesandAnswerCount(pPmicCoreHandle,
                                                    &qaAnsCnt,
                                                    &qaQuesCnt);
        }

        if((PMIC_ST_SUCCESS == status) && (qaAnsCnt == ((uint8_t)ansIndex)))
        {
            answer = Pmic_getAnswerByte(qaQuesCnt, qaAnsCnt, qaFdbk);
            /*! Writing watch dog four Answers */
            status = Pmic_commIntf_sendByte(
                                   pPmicCoreHandle,
                                   PMIC_WD_ANSWER_REG_REGADDR,
                                   answer);

            if((PMIC_ST_SUCCESS == status) &&
               (((bool)true) == is_wdgBadEventDetected(pPmicCoreHandle)))
            {
                status = PMIC_ST_ERR_INV_WDG_ANSWER;
                break;
            }
        }
    }

    return status;
}

/*!
 * \brief   API to Enable Watchdog timer.
 *
 * Requirement: REQ_TAG(PDK-5839), REQ_TAG(PDK-5854)
 * Design: did_pmic_wdg_cfg_readback
 *
 *          This function is used to Enable the PMIC watchdog. User ensure
 *          that, this function needs to be called to enable watchdog timer
 *          before configuring or starting watchdog trigger or QA mode.
 *
 * \param   pPmicCoreHandle [IN]    PMIC Interface Handle
 *
 * \return  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_wdgEnable(Pmic_CoreHandle_t *pPmicCoreHandle)
{
    int32_t status = PMIC_ST_SUCCESS;

    /* Validate pPmicCoreHandle and WDG subsystem */
    status = Pmic_WdgValidatePmicCoreHandle(pPmicCoreHandle);

    if(PMIC_ST_SUCCESS == status)
    {
        status = Pmic_wdgEnDisState(pPmicCoreHandle, PMIC_WDG_ENABLE);
    }

    return status;
}

/*!
 * \brief   API to Disable Watchdog timer.
 *
 * Requirement: REQ_TAG(PDK-5839), REQ_TAG(PDK-5854)
 * Design: did_pmic_wdg_cfg_readback
 *
 *          This function is used to Disable the PMIC watchdog. User ensure
 *          that, after using this function, complete watchdog functionality
 *          and configuration will be deactivated.
 *
 * \param   pPmicCoreHandle [IN]    PMIC Interface Handle
 *
 * \return  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_wdgDisable(Pmic_CoreHandle_t *pPmicCoreHandle)
{
    int32_t status = PMIC_ST_SUCCESS;

    /* Validate pPmicCoreHandle and WDG subsystem */
    status = Pmic_WdgValidatePmicCoreHandle(pPmicCoreHandle);

    if(PMIC_ST_SUCCESS == status)
    {
        status = Pmic_wdgEnDisState(pPmicCoreHandle, PMIC_WDG_DISABLE);
    }

    return status;
}

/*!
 * \brief   API to set PMIC watchdog configurations.
 *
 * Requirement: REQ_TAG(PDK-5839), REQ_TAG(PDK-5854)
 * Design: did_pmic_wdg_cfg_readback
 *
 *          This function is used to configure the watchdog parameters
 *          in the PMIC for trigger mode or Q&A(question and answer) mode,
 *          when corresponding validParam bit fields are set in
 *          Pmic_WdgCfg_t structure.
 *          User has to call Pmic_wdgEnable() before set the configuration.
 *
 * \param   pPmicCoreHandle [IN]    PMIC Interface Handle
 * \param   wdgCfg          [IN]    Watchdog configuration
 *
 * \return  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_wdgSetCfg(Pmic_CoreHandle_t   *pPmicCoreHandle,
                       const Pmic_WdgCfg_t  wdgCfg)
{
    int32_t status = PMIC_ST_SUCCESS;

    /* Validate pPmicCoreHandle and WDG subsystem */
    status = Pmic_WdgValidatePmicCoreHandle(pPmicCoreHandle);

    if(PMIC_ST_SUCCESS == status)
    {
        status = Pmic_WdgSetWindowsTimeIntervals(pPmicCoreHandle, wdgCfg);
    }

    if(PMIC_ST_SUCCESS == status)
    {
        status = Pmic_WdgSetThresholdValues(pPmicCoreHandle, wdgCfg);
    }

    if(PMIC_ST_SUCCESS == status)
    {
        status = Pmic_WdgSetCtrlParams(pPmicCoreHandle, wdgCfg);
    }

    if(PMIC_ST_SUCCESS == status)
    {
        status = Pmic_WdgSetQaConfigurations(pPmicCoreHandle, wdgCfg);
    }

    return status;
}

/*!
 * \brief   API to get PMIC watchdog configurations.
 *
 * Requirement: REQ_TAG(PDK-5839), REQ_TAG(PDK-5854)
 * Design: did_pmic_wdg_cfg_readback
 *
 *          This function is used to get configuration of the watchdog
 *          from the PMIC for trigger mode or Q&A(question and answer) mode,
 *          when corresponding validParam bit fields are set in
 *          Pmic_WdgCfg_t structure.
 *          User has to call Pmic_wdgEnable() before get the configuration.
 *
 * \param   pPmicCoreHandle [IN]    PMIC Interface Handle
 * \param   pWdgCf          [OUT]   Watchdog configuration pointer
 *
 * \return  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_wdgGetCfg(Pmic_CoreHandle_t *pPmicCoreHandle,
                       Pmic_WdgCfg_t     *pWdgCfg)
{
    int32_t status = PMIC_ST_SUCCESS;

    /* Validate pPmicCoreHandle and WDG subsystem */
    status = Pmic_WdgValidatePmicCoreHandle(pPmicCoreHandle);

    if((PMIC_ST_SUCCESS == status) && (NULL == pWdgCfg))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if(PMIC_ST_SUCCESS == status)
    {
        status = Pmic_WdgGetWindowsTimeIntervals(pPmicCoreHandle, pWdgCfg);
    }

    if(PMIC_ST_SUCCESS == status)
    {
        status = Pmic_WdgGetThresholdValues(pPmicCoreHandle, pWdgCfg);
    }

    if(PMIC_ST_SUCCESS == status)
    {
        status = Pmic_WdgGetCtrlParams(pPmicCoreHandle, pWdgCfg);
    }

    if(PMIC_ST_SUCCESS == status)
    {
        status = Pmic_WdgGetQaConfigurations(pPmicCoreHandle, pWdgCfg);
    }

    return status;
}

/*!
 * \brief   API to Start watchdog QA mode.
 *
 * Requirement: REQ_TAG(PDK-5839)
 * Design: did_pmic_wdg_cfg_readback
 *
 *          This function is used to start watchdog sequence and continues
 *          till the given num_of_sequences. User has to ensure, configure
 *          all Watchdog QA parameters properly using Pmic_wdgSetCfg() API,
 *          before starting QA sequence using this API. User can use
 *          Pmic_wdgSetCfg() API to stop watchdog trigger mode.
 *
 *          Note: To perform QA sequences, user has to adjust Long window
 *                time interval, Window1 time interval and Window2 time
 *                inervals depends on errors given by API. If user gets
 *                PMIC_ST_ERR_INV_WDG_WINDOW, then user has to increase the
 *                Long window or window1 time interval. If user gets
 *                PMIC_ST_ERR_WDG_EARLY_ANSWER, then user has to reduce
 *                the Window1 time inerval.
 *
 * \param   pPmicCoreHandle  [IN]    PMIC Interface Handle
 * \param   num_of_sequences [IN]    number of QA sequences.
 *                                   If PMIC_WD_QA_INFINITE_SEQ is used,
 *                                   then API runs for infinite sequence.
 * \param   maxCnt           [IN]    Number of iterations to wait for an
 *                                   Good/Bad event. The value should be greater
 *                                   than or equal to PMIC_WDG_WAIT_CNT_MIN_VAL.
 *
 * \return  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_wdgStartQaSequence(Pmic_CoreHandle_t *pPmicCoreHandle,
                                uint32_t           num_of_sequences,
                                uint32_t           maxCnt)
{
    int32_t status     = PMIC_ST_SUCCESS;
    uint8_t regVal     = 0x0U;
    uint8_t failCnt    = 0U;
    uint32_t loopCount = 0U;
    uint32_t sequences = num_of_sequences;
    int8_t flag        = 0;

    /* Validate pPmicCoreHandle and WDG subsystem */
    status = Pmic_WdgValidatePmicCoreHandle(pPmicCoreHandle);

    if((PMIC_ST_SUCCESS == status) &&
       (maxCnt < PMIC_WDG_WAIT_CNT_MIN_VAL))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    if(PMIC_ST_SUCCESS == status)
    {
        /*! Start Critical Section */
        Pmic_criticalSectionStart(pPmicCoreHandle);

        /*! Reading watchdog mode value */
        status = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        PMIC_WD_MODE_REG_REGADDR,
                                        &regVal);

        if(PMIC_ST_SUCCESS == status)
        {
            Pmic_setBitField(&regVal,
                             PMIC_WD_MODE_REG_WD_MODE_SELECT_SHIFT,
                             PMIC_WD_MODE_REG_WD_MODE_SELECT_MASK,
                             PMIC_WDG_QA_MODE);
            /*! Set watchdog mode to QA mode */
            status = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                            PMIC_WD_MODE_REG_REGADDR,
                                            regVal);
        }

        /*! Stop Critical Section */
        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    /* Disable ret to Long Window */
    if(PMIC_ST_SUCCESS == status)
    {
        status = Pmic_WdgSetRetToLongWindow(pPmicCoreHandle,
                                            PMIC_WDG_RETLONGWIN_DISABLE);
    }

    /* Clear WDG Error bits */
    if(PMIC_ST_SUCCESS == status)
    {
        /* Start Critical Section */
        Pmic_criticalSectionStart(pPmicCoreHandle);

        status = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        PMIC_WD_ERR_STATUS_REGADDR,
                                        &regVal);
        if(PMIC_ST_SUCCESS == status)
        {
            status = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                            PMIC_WD_ERR_STATUS_REGADDR,
                                            regVal);
        }

        /* Stop Critical Section */
        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    /* Write Answers for Long Window */
    if(PMIC_ST_SUCCESS == status)
    {
        status = Pmic_wdgQaWriteAnswers(pPmicCoreHandle);
        if((PMIC_ST_SUCCESS != status) &&
           (PMIC_ST_ERR_INV_WDG_ANSWER == status))
        {
            /* Start Critical Section */
            Pmic_criticalSectionStart(pPmicCoreHandle);

            status = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                            PMIC_WD_ERR_STATUS_REGADDR,
                                            &regVal);

            /* Stop Critical Section */
            Pmic_criticalSectionStop(pPmicCoreHandle);

            if(PMIC_ST_SUCCESS == status)
            {
                if(0U != (regVal & 0x01U))
                {
                    status = PMIC_ST_ERR_INV_WDG_WINDOW;
                }
            }
        }
    }

    /* Dummy Read operations to sync the WatchDog */
    if(PMIC_ST_SUCCESS == status)
    {
        status = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        PMIC_WD_ERR_STATUS_REGADDR,
                                        &regVal);
    }

    if(PMIC_ST_SUCCESS == status)
    {
        status = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        PMIC_WD_FAIL_CNT_REG_REGADDR,
                                        &regVal);
    }

    if(PMIC_ST_SUCCESS == status)
    {
        status = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        PMIC_WD_FAIL_CNT_REG_REGADDR,
                                        &regVal);
    }

    /* Write QA Answers for given numbers of sequences */
    while((PMIC_ST_SUCCESS == status) &&
          ((PMIC_WD_QA_INFINITE_SEQ == sequences) ||
           (sequences > 0U)))
    {
        /*! Write Answer to WDOG for the sequence */
        status = Pmic_wdgQaWriteAnswers(pPmicCoreHandle);

        if(PMIC_ST_ERR_INV_WDG_ANSWER == status)
        {
            status = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                            PMIC_WD_ERR_STATUS_REGADDR,
                                            &regVal);
            if(PMIC_ST_SUCCESS == status)
            {
                if(0U != (regVal & 0x10U))
                {
                    status = PMIC_ST_ERR_INV_WDG_WINDOW;
                    break;
                }
            }
        }

        /* Update loopCount value for while loop */
        loopCount = maxCnt;
        while((PMIC_ST_SUCCESS == status) && (loopCount > 0U))
        {
            status = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                            PMIC_WD_FAIL_CNT_REG_REGADDR,
                                            &failCnt);

            if((PMIC_ST_SUCCESS == status) && (0U != (failCnt & 0x40U)))
            {
                status = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                                PMIC_WD_ERR_STATUS_REGADDR,
                                                &regVal);
                if((PMIC_ST_SUCCESS == status) && (0U != (regVal & 0x08U)))
                {
                    status = PMIC_ST_ERR_WDG_EARLY_ANSWER;
                }

                sequences = 0U;
                flag = 1;
            }
            else
            {
                if((PMIC_ST_SUCCESS == status) && (0U != (failCnt & 0x20U)))
                {
                    sequences--;
                    flag = 1;
                }
            }

            if(flag == 1)
            {
                break;
            }

            loopCount--;
        }
    }

    if(PMIC_ST_SUCCESS == status)
    {
        /* Enable Return long window Enable */
        status = Pmic_WdgSetRetToLongWindow(pPmicCoreHandle,
                                   PMIC_WDG_RETLONGWIN_ENABLE);
    }

    return status;
}

/*!
 * \brief   API to get PMIC watchdog error status.
 *
 * Requirement: REQ_TAG(PDK-5839), REQ_TAG(PDK-5854)
 * Design: did_pmic_wdg_cfg_readback
 *
 *          This function is used to get the watchdog error status from the
 *          PMIC for trigger mode or Q&A(question and answer) mode,
 *          when corresponding validParam bit fields are set in
 *          Pmic_WdgErrStatus_t structure.
 *          User has to call Pmic_wdgEnable() before getting the error status.
 *
 * \param   pPmicCoreHandle [IN]    PMIC Interface Handle
 * \param   pErrStatus      [OUT]   Watchdog error status pointer
 *
 * \return  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_wdgGetErrorStatus(Pmic_CoreHandle_t   *pPmicCoreHandle,
                               Pmic_WdgErrStatus_t *pErrStatus)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regVal = 0x0U;

    /* Validate pPmicCoreHandle and WDG subsystem */
    status = Pmic_WdgValidatePmicCoreHandle(pPmicCoreHandle);

    if((PMIC_ST_SUCCESS == status) && (NULL == pErrStatus))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if(PMIC_ST_SUCCESS == status)
    {
        /*! Start Critical Section */
        Pmic_criticalSectionStart(pPmicCoreHandle);

        /*! Reading error status register */
        status = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        PMIC_WD_ERR_STATUS_REGADDR,
                                        &regVal);

        /*! Stop Critical Section */
        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    /* Get watchdog error status */
    if((PMIC_ST_SUCCESS == status) &&
       (((bool)true) == pmic_validParamCheck(pErrStatus->validParams,
                               PMIC_CFG_WD_LONGWIN_TIMEOUT_ERRSTAT_VALID)))
    {
        if(Pmic_getBitField(regVal,
                            PMIC_WD_ERR_STATUS_WD_LONGWIN_TIMEOUT_INT_SHIFT,
                            PMIC_WD_ERR_STATUS_WD_LONGWIN_TIMEOUT_INT_MASK) != 0U)
        {
            pErrStatus->wdLongWinTimeout = (bool)true;
        }
        else
        {
            pErrStatus->wdLongWinTimeout = (bool)false;
        }

    }

    if((PMIC_ST_SUCCESS == status) &&
       (((bool)true) == pmic_validParamCheck(pErrStatus->validParams,
                               PMIC_CFG_WD_TIMEOUT_ERRSTAT_VALID)))
    {
        if(Pmic_getBitField(regVal,
                            PMIC_WD_ERR_STATUS_WD_TIMEOUT_SHIFT,
                            PMIC_WD_ERR_STATUS_WD_TIMEOUT_MASK) != 0U)
        {
            pErrStatus->wdTimeout = (bool)true;
        }
        else
        {
            pErrStatus->wdTimeout = (bool)false;
        }

    }

    if((PMIC_ST_SUCCESS == status) &&
       (((bool)true) == pmic_validParamCheck(pErrStatus->validParams,
                               PMIC_CFG_WD_TRIG_EARLY_ERRSTAT_VALID)))
    {
        if(Pmic_getBitField(regVal,
                            PMIC_WD_ERR_STATUS_WD_TRIG_EARLY_SHIFT,
                            PMIC_WD_ERR_STATUS_WD_TRIG_EARLY_MASK) != 0U)
        {
            pErrStatus->wdTrigEarly = (bool)true;
        }
        else
        {
            pErrStatus->wdTrigEarly = (bool)false;
        }

    }

    if((PMIC_ST_SUCCESS == status) &&
       (((bool)true) == pmic_validParamCheck(pErrStatus->validParams,
                               PMIC_CFG_WD_ANSW_EARLY_ERRSTAT_VALID)))
    {
        if(Pmic_getBitField(regVal,
                            PMIC_WD_ERR_STATUS_WD_ANSW_EARLY_SHIFT,
                            PMIC_WD_ERR_STATUS_WD_ANSW_EARLY_MASK) != 0U)
        {
            pErrStatus->wdAnswearly = (bool)true;
        }
        else
        {
            pErrStatus->wdAnswearly = (bool)false;
        }

    }

    if((PMIC_ST_SUCCESS == status) &&
       (((bool)true) == pmic_validParamCheck(pErrStatus->validParams,
                               PMIC_CFG_WD_SEQ_ERR_ERRSTAT_VALID)))
    {
        if(Pmic_getBitField(regVal,
                            PMIC_WD_ERR_STATUS_WD_SEQ_ERR_SHIFT,
                            PMIC_WD_ERR_STATUS_WD_SEQ_ERR_MASK) != 0U)
        {
            pErrStatus->wdSeqErr = (bool)true;
        }
        else
        {
            pErrStatus->wdSeqErr = (bool)false;
        }

    }

    if((PMIC_ST_SUCCESS == status) &&
       (((bool)true) == pmic_validParamCheck(pErrStatus->validParams,
                               PMIC_CFG_WD_ANSW_ERR_ERRSTAT_VALID)))
    {
        if(Pmic_getBitField(regVal,
                            PMIC_WD_ERR_STATUS_WD_ANSW_ERR_SHIFT,
                            PMIC_WD_ERR_STATUS_WD_ANSW_ERR_MASK) != 0U)
        {
            pErrStatus->wdAnswErr = (bool)true;
        }
        else
        {
            pErrStatus->wdAnswErr = (bool)false;
        }

    }

    if((PMIC_ST_SUCCESS == status) &&
       (((bool)true) == pmic_validParamCheck(pErrStatus->validParams,
                               PMIC_CFG_WD_FAIL_INT_ERRSTAT_VALID)))
    {
        if(Pmic_getBitField(regVal,
                            PMIC_WD_ERR_STATUS_WD_FAIL_INT_SHIFT,
                            PMIC_WD_ERR_STATUS_WD_FAIL_INT_MASK) != 0U)
        {
            pErrStatus->wdFailInt = (bool)true;
        }
        else
        {
            pErrStatus->wdFailInt = (bool)false;
        }

    }

    if((PMIC_ST_SUCCESS == status) &&
       (((bool)true) == pmic_validParamCheck(pErrStatus->validParams,
                               PMIC_CFG_WD_RST_INT_ERRSTAT_VALID)))
    {
        if(Pmic_getBitField(regVal,
                            PMIC_WD_ERR_STATUS_WD_RST_INT_SHIFT,
                            PMIC_WD_ERR_STATUS_WD_RST_INT_MASK) != 0U)
        {
            pErrStatus->wdRstInt = (bool)true;
        }
        else
        {
            pErrStatus->wdRstInt = (bool)false;
        }

    }

    return status;
}

/*!
 * \brief   API to get PMIC watchdog fail count.
 *
 * Requirement: REQ_TAG(PDK-5839), REQ_TAG(PDK-5854)
 * Design: did_pmic_wdg_cfg_readback
 *
 *          This function is used to get the watchdog fail count from the PMIC
 *          for trigger mode or Q&A(question and answer) mode.
 *          User has to call Pmic_wdgEnable() before getting the fail count.
 *
 * \param   pPmicCoreHandle [IN]    PMIC Interface Handle
 * \param   pFailCount      [OUT]   Watchdog fail count pointer
 *
 * \return  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_wdgGetFailCount(Pmic_CoreHandle_t *pPmicCoreHandle,
                                    uint8_t    *pFailCount)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regVal = 0x00U;

    /* Validate pPmicCoreHandle and WDG subsystem */
    status = Pmic_WdgValidatePmicCoreHandle(pPmicCoreHandle);

    if((PMIC_ST_SUCCESS == status) && (NULL == pFailCount))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if(PMIC_ST_SUCCESS == status)
    {
        /*! Start Critical Section */
        Pmic_criticalSectionStart(pPmicCoreHandle);

        /*! Reading error status register */
        status = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        PMIC_WD_FAIL_CNT_REG_REGADDR,
                                        &regVal);

        /*! Stop Critical Section */
        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    if(PMIC_ST_SUCCESS == status)
    {
        *pFailCount = Pmic_getBitField(regVal,
                                       PMIC_WD_FAIL_CNT_REG_WD_FAIL_CNT_SHIFT,
                                       PMIC_WD_FAIL_CNT_REG_WD_FAIL_CNT_MASK);
    }

    return status;
}

/*!
 * \brief   API to Start watchdog Trigger mode.
 *
 * Requirement: REQ_TAG(PDK-5854)
 * Design: did_pmic_wdg_cfg_readback
 *
 *          This function is used to start watchdog trigger mode.
 *          User has to ensure, configure all Watchdog trigger parameters
 *          properly using Pmic_wdgSetCfg() API, before starting watchdog
 *          trigger mode using this API. User can use Pmic_wdgSetCfg() API
 *          to stop watchdog trigger mode.
 *
 *          Note: To perform watchdog trigger mode, user has to
 *                adjust Long window time interval, Window1 time interval
 *                and Window2 time inervals as below, depends on the
 *                time-period of the trigger pulse provided by other
 *                device.
 *                1. Longwindow time interval must be greater than Trigger
 *                   pulse time period.
 *                2. Window1 time interval must be less than T-off time of
 *                   the Trigger pulse time period.
 *                3. Window2 time interval must be greater than T-on time
 *                   of the Trigger pulse time period.
 *                4. (Window1 time interval + Window2 time interval)
 *                   approximately equal to the Trigger pulse time period.
 *
 * \param   pPmicCoreHandle  [IN]    PMIC Interface Handle
 *
 * \return  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_wdgStartTriggerSequence(Pmic_CoreHandle_t *pPmicCoreHandle)
{
    int32_t status     = PMIC_ST_SUCCESS;
    uint8_t regVal     = 0x0U;

    /* Validate pPmicCoreHandle and WDG subsystem */
    status = Pmic_WdgValidatePmicCoreHandle(pPmicCoreHandle);

    if(PMIC_ST_SUCCESS == status)
    {
        /*! Start Critical Section */
        Pmic_criticalSectionStart(pPmicCoreHandle);

        /*! Reading watchdog mode value */
        status = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        PMIC_WD_MODE_REG_REGADDR,
                                        &regVal);

        if(PMIC_ST_SUCCESS == status)
        {
            Pmic_setBitField(&regVal,
                             PMIC_WD_MODE_REG_WD_MODE_SELECT_SHIFT,
                             PMIC_WD_MODE_REG_WD_MODE_SELECT_MASK,
                             PMIC_WDG_TRIGGER_MODE);
            /*! Set watchdog mode to trigger mode */
            status = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                            PMIC_WD_MODE_REG_REGADDR,
                                            regVal);
        }

        /*! Stop Critical Section */
        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    return status;
}
