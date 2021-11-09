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
*   \file    pmic_esm.c
*
*   \brief   This file contains the default APIs for PMIC ESM
*            configuration.
*
*/

#include <pmic_esm.h>
#include <pmic_esm_priv.h>
#include <pmic_core_priv.h>
#include <pmic_io_priv.h>

#include <pmic_irq.h>
#include <pmic_irq_tps6594x.h>
#include <pmic_irq_lp8764x.h>

/*!
 * \brief   This function returns the corresponding uint8 value
 */
static uint8_t Pmic_esmGetU8Val(bool esmVal)
{
    uint8_t esmU8Val = 0U;

    if(((bool)true) == esmVal)
    {
        esmU8Val = 1U;
    }

    return esmU8Val;
}

/*!
 * \brief    This function is used to validate whether ESM subsystem is valid
 *           for the programmed pmic device and does the NULL param check for
 *           pPmicCoreHandle.
 */
static int32_t Pmic_esmValidateParams(const Pmic_CoreHandle_t *pPmicCoreHandle)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    if(NULL == pPmicCoreHandle)
    {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (((bool)false) == pPmicCoreHandle->pPmic_SubSysInfo->esmEnable))
    {
        pmicStatus = PMIC_ST_ERR_INV_DEVICE;
    }

    return pmicStatus;
}

/*!
 * \brief   This function is used to get the ESM_MCU/ESM_SOC Base Register
 *          address
 */
static void Pmic_esmGetBaseRegAddr(const bool  esmType,
                                   uint8_t    *pEsmBaseAddr)
{
    if(0U != Pmic_esmGetU8Val(esmType))
    {
        (*pEsmBaseAddr) = PMIC_ESM_SOC_BASE_REGADDR;
    }
    else
    {
        (*pEsmBaseAddr) = PMIC_ESM_MCU_BASE_REGADDR;
    }
}

/*!
 * \brief   This function is used to check the Device used.
 *
 *          Note: In this API, the default PMIC device is assumed as TPS6594x
 *                LEO PMIC. While adding support for New PMIC device, developer
 *                need to update the API functionality for New PMIC device
 *                accordingly.
 */
static int32_t Pmic_esmDeviceCheck(const Pmic_CoreHandle_t  *pPmicCoreHandle,
                                   const bool                esmType)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    switch(pPmicCoreHandle->pmicDeviceType)
    {
        case PMIC_DEV_HERA_LP8764X:
            if(((bool)true) == esmType)
            {
                pmicStatus = PMIC_ST_ERR_INV_DEVICE;
            }
            else
            {
                pmicStatus = PMIC_ST_SUCCESS;
            }

            break;
        default:
            /* Default case is valid only for TPS6594x LEO PMIC */
            pmicStatus = PMIC_ST_SUCCESS;
            break;
    }

    return pmicStatus;
}

/*!
 * \brief   This function is used to check the current state of ESM
 *          (Start/Stop State)
 */
static int32_t Pmic_esmCheckState(Pmic_CoreHandle_t   *pPmicCoreHandle,
                                  const uint8_t        esmBaseRegAddr)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regAddr    = 0U;
    uint8_t regData    = 0U;

    regAddr = (esmBaseRegAddr + PMIC_ESM_START_REG_OFFSET);

    /* Start Critical Section */
    Pmic_criticalSectionStart(pPmicCoreHandle);

    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        regAddr,
                                        &regData);

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (Pmic_getBitField(regData,
                         PMIC_ESM_X_START_REG_ESM_X_START_SHIFT,
                         PMIC_ESM_X_START_REG_ESM_X_START_MASK)
        == PMIC_ESM_VAL_1))
    {
        pmicStatus = PMIC_ST_ERR_ESM_STARTED;
    }

    /* Stop Critical Section */
    Pmic_criticalSectionStop(pPmicCoreHandle);

    return pmicStatus;
}

/*!
 * \brief   This function is used to Start ESM_MCU/ESM_SOC
 */
static int32_t Pmic_esmXStart(Pmic_CoreHandle_t   *pPmicCoreHandle,
                              const uint8_t        esmBaseRegAddr,
                              const bool           esmState)
{
    int32_t pmicStatus  = PMIC_ST_SUCCESS;
    uint8_t regAddr     = 0U;
    uint8_t regData     = 0U;
    uint8_t esmU8Val = 0U;

    regAddr = (esmBaseRegAddr + PMIC_ESM_START_REG_OFFSET);
    esmU8Val = Pmic_esmGetU8Val(esmState);

    /* Start Critical Section */
    Pmic_criticalSectionStart(pPmicCoreHandle);

    /* Read the PMIC ESM_MCU/ESM_SOC START Register */
    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        regAddr,
                                        &regData);

    /* Write 0 or 1 to ESM_MCU/ESM_SOC START BIT to Start/Stop ESM */
    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        Pmic_setBitField(&regData,
                         PMIC_ESM_X_START_REG_ESM_X_START_SHIFT,
                         PMIC_ESM_X_START_REG_ESM_X_START_MASK,
                         esmU8Val);

        pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                            regAddr,
                                            regData);
    }

    /* Stop Critical Section */
    Pmic_criticalSectionStop(pPmicCoreHandle);

    return pmicStatus;
}

/*!
 * \brief   This function is used to Enable ESM_MCU/ESM_SOC
 */
static int32_t Pmic_esmXEnable(Pmic_CoreHandle_t   *pPmicCoreHandle,
                               const uint8_t        esmBaseRegAddr,
                               const bool           esmToggle)
{
    int32_t pmicStatus   = PMIC_ST_SUCCESS;
    uint8_t regAddr      = 0U;
    uint8_t regData      = 0U;
    uint8_t esmU8Val = 0U;

    /* Check if ESM is started */
    pmicStatus = Pmic_esmCheckState(pPmicCoreHandle, esmBaseRegAddr);

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        regAddr = (esmBaseRegAddr + PMIC_ESM_MODE_CFG_REG_OFFSET);
        esmU8Val = Pmic_esmGetU8Val(esmToggle);

        /* Start Critical Section */
        Pmic_criticalSectionStart(pPmicCoreHandle);

        /* Read the PMIC ESM MODE CFG Register */
        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                            regAddr,
                                            &regData);

        /* Configure ESM_X_EN(where X is ESM_MCU or ESM_SOC) Bit to enable
         * or Disable.
         */
        if(PMIC_ST_SUCCESS == pmicStatus)
        {
            Pmic_setBitField(&regData,
                             PMIC_ESM_X_MODE_CFG_ESM_X_EN_SHIFT,
                             PMIC_ESM_X_MODE_CFG_ESM_X_EN_MASK,
                             esmU8Val);

            pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                                regAddr,
                                                regData);
        }

        /* Stop Critical Section */
        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    return pmicStatus;
}

/*!
 * \brief   This function is used to Enable/Disable PMIC ESM SOC Interrupts
 */
static int32_t Pmic_esmSocIntrEnable(Pmic_CoreHandle_t        *pPmicCoreHandle,
                                     const Pmic_EsmIntrCfg_t   esmIntrCfg)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    /* Mask/Un-mask ESM SOC Interrupts */

    pmicStatus = Pmic_irqMaskIntr(pPmicCoreHandle,
                                  PMIC_TPS6594X_ESM_SOC_PIN_INT,
                                  !esmIntrCfg.esmPinIntr);

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        pmicStatus = Pmic_irqMaskIntr(pPmicCoreHandle,
                                      PMIC_TPS6594X_ESM_SOC_FAIL_INT,
                                      !esmIntrCfg.esmFailIntr);
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {

        pmicStatus = Pmic_irqMaskIntr(pPmicCoreHandle,
                                      PMIC_TPS6594X_ESM_SOC_RST_INT,
                                      !esmIntrCfg.esmRstIntr);
    }

    return pmicStatus;
}

/*!
 * \brief   This function is used to Enable/Disable PMIC ESM Interrupts.
 *
 *          Note: In this API, the default PMIC device is assumed as TPS6594x
 *                LEO PMIC. While adding support for New PMIC device, developer
 *                need to update the API functionality for New PMIC device
 *                accordingly.
 */
static int32_t Pmic_esmIntrEnable(Pmic_CoreHandle_t        *pPmicCoreHandle,
                                  const bool                esmType,
                                  const Pmic_EsmIntrCfg_t   esmIntrCfg)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    switch(pPmicCoreHandle->pmicDeviceType)
    {
        case PMIC_DEV_HERA_LP8764X:
            pmicStatus = Pmic_irqMaskIntr(pPmicCoreHandle,
                                      PMIC_LP8764X_ESM_MCU_PIN_INT,
                                      !esmIntrCfg.esmPinIntr);

            if(PMIC_ST_SUCCESS == pmicStatus)
            {
                pmicStatus = Pmic_irqMaskIntr(pPmicCoreHandle,
                                              PMIC_LP8764X_ESM_MCU_FAIL_INT,
                                              !esmIntrCfg.esmFailIntr);
            }

            if(PMIC_ST_SUCCESS == pmicStatus)
            {
                pmicStatus = Pmic_irqMaskIntr(pPmicCoreHandle,
                                              PMIC_LP8764X_ESM_MCU_RST_INT,
                                              !esmIntrCfg.esmRstIntr);
            }

            break;
        default:
            /* Default case is valid only for TPS6594x LEO PMIC */
            if(PMIC_ESM_MODE_MCU == esmType)
            {
                pmicStatus = Pmic_irqMaskIntr(pPmicCoreHandle,
                                          PMIC_TPS6594X_ESM_MCU_PIN_INT,
                                          !esmIntrCfg.esmPinIntr);
                if(PMIC_ST_SUCCESS == pmicStatus)
                {
                    pmicStatus = Pmic_irqMaskIntr(pPmicCoreHandle,
                                                PMIC_TPS6594X_ESM_MCU_FAIL_INT,
                                                !esmIntrCfg.esmFailIntr);
                }

                if(PMIC_ST_SUCCESS == pmicStatus)
                {
                    pmicStatus = Pmic_irqMaskIntr(pPmicCoreHandle,
                                                 PMIC_TPS6594X_ESM_MCU_RST_INT,
                                                 !esmIntrCfg.esmRstIntr);
                }
            }
            else
            {
                 /* Mask/Un-mask ESM SOC Interrupts */
                pmicStatus = Pmic_esmSocIntrEnable(pPmicCoreHandle,
                                                   esmIntrCfg);
            }

            break;
    }

    return pmicStatus;
}

/*!
 * \brief   This function is used to Set the ESM DELAY1 value
 */
static int32_t Pmic_esmSetDelay1Value(Pmic_CoreHandle_t   *pPmicCoreHandle,
                                      const Pmic_EsmCfg_t  esmCfg,
                                      const uint8_t        esmBaseRegAddr)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData    = 0U;
    uint8_t regAddr    = 0U;

    regAddr = (esmBaseRegAddr + PMIC_ESM_DELAY1_REG_OFFSET);

    if(PMIC_ESM_DELAY_MICROSEC_MAX < esmCfg.esmDelay1_us)
    {
       pmicStatus = PMIC_ST_ERR_INV_ESM_VAL;
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        regData = (uint8_t)(esmCfg.esmDelay1_us / PMIC_ESM_DELAY_MICROSEC_DIV);

        /* Start Critical Section */
        Pmic_criticalSectionStart(pPmicCoreHandle);

        pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                            regAddr,
                                            regData);

        /* Stop Critical Section */
        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    return pmicStatus;
}

/*!
 * \brief   This function is used to Set the ESM DELAY2 value
 */
static int32_t Pmic_esmSetDelay2Value(Pmic_CoreHandle_t   *pPmicCoreHandle,
                                      const Pmic_EsmCfg_t  esmCfg,
                                      const uint8_t        esmBaseRegAddr)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData    = 0U;
    uint8_t regAddr    = 0U;

    regAddr = (esmBaseRegAddr + PMIC_ESM_DELAY2_REG_OFFSET);

    if(PMIC_ESM_DELAY_MICROSEC_MAX < esmCfg.esmDelay2_us)
    {
       pmicStatus = PMIC_ST_ERR_INV_ESM_VAL;
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        regData = (uint8_t)(esmCfg.esmDelay2_us / PMIC_ESM_DELAY_MICROSEC_DIV);

        /* Start Critical Section */
        Pmic_criticalSectionStart(pPmicCoreHandle);

        pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                            regAddr,
                                            regData);

        /* Stop Critical Section */
        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    return pmicStatus;
}

/*!
 * \brief   This function is used to Set the ESM Error Count Threshold value
 */
static int32_t Pmic_esmSetErrCntThrValue(Pmic_CoreHandle_t   *pPmicCoreHandle,
                                         const Pmic_EsmCfg_t  esmCfg,
                                         const uint8_t        esmBaseRegAddr)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData    = 0U;
    uint8_t regAddr    = 0U;

    regAddr = (esmBaseRegAddr + PMIC_ESM_MODE_CFG_REG_OFFSET);

    /* Start Critical Section */
    Pmic_criticalSectionStart(pPmicCoreHandle);

    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        regAddr,
                                        &regData);

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        Pmic_setBitField(&regData,
                         PMIC_ESM_X_MODE_CFG_ESM_X_ERR_CNT_TH_SHIFT,
                         PMIC_ESM_X_MODE_CFG_ESM_X_ERR_CNT_TH_MASK,
                         esmCfg.esmErrCntThr);

        pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                            regAddr,
                                            regData);
    }

    /* Stop Critical Section */
    Pmic_criticalSectionStop(pPmicCoreHandle);

    return pmicStatus;
}

/*!
 * \brief   This function is used to Set the ESM HMAX value
 */
static int32_t Pmic_esmSetHmaxValue(Pmic_CoreHandle_t   *pPmicCoreHandle,
                                    const Pmic_EsmCfg_t  esmCfg,
                                    const uint8_t        esmBaseRegAddr)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData    = 0U;
    uint8_t regAddr    = 0U;

    regAddr = (esmBaseRegAddr + PMIC_ESM_HMAX_REG_OFFSET);

    if((PMIC_ESM_PWM_PULSE_MICROSEC_MAX < esmCfg.esmHmax_us) ||
       (PMIC_ESM_PWM_PULSE_MICROSEC_MIN > esmCfg.esmHmax_us))
    {
        pmicStatus = PMIC_ST_ERR_INV_ESM_VAL;
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        regData = (uint8_t)((esmCfg.esmHmax_us - PMIC_ESM_PWM_PULSE_MICROSEC_MIN) /
                    PMIC_ESM_PWM_PULSE_MICROSEC_DIV);

        /* Start Critical Section */
        Pmic_criticalSectionStart(pPmicCoreHandle);

        pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                            regAddr,
                                            regData);

        /* Stop Critical Section */
        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    return pmicStatus;
}

/*!
 * \brief   This function is used to Set the ESM HMIN value
 */
static int32_t Pmic_esmSetHminValue(Pmic_CoreHandle_t   *pPmicCoreHandle,
                                    const Pmic_EsmCfg_t  esmCfg,
                                    const uint8_t        esmBaseRegAddr)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData    = 0U;
    uint8_t regAddr    = 0U;

    regAddr = (esmBaseRegAddr + PMIC_ESM_HMIN_REG_OFFSET);

    if((PMIC_ESM_PWM_PULSE_MICROSEC_MAX < esmCfg.esmHmin_us) ||
       (PMIC_ESM_PWM_PULSE_MICROSEC_MIN > esmCfg.esmHmin_us))
    {
        pmicStatus = PMIC_ST_ERR_INV_ESM_VAL;
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        regData = (uint8_t)((esmCfg.esmHmin_us - PMIC_ESM_PWM_PULSE_MICROSEC_MIN) /
                    PMIC_ESM_PWM_PULSE_MICROSEC_DIV);

        /* Start Critical Section */
        Pmic_criticalSectionStart(pPmicCoreHandle);

        pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                            regAddr,
                                            regData);

        /* Stop Critical Section */
        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    return pmicStatus;
}

/*!
 * \brief   This function is used to Set the ESM LMAX value
 */
static int32_t Pmic_esmSetLmaxValue(Pmic_CoreHandle_t   *pPmicCoreHandle,
                                    const Pmic_EsmCfg_t  esmCfg,
                                    const uint8_t        esmBaseRegAddr)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData    = 0U;
    uint8_t regAddr    = 0U;

    regAddr = (esmBaseRegAddr + PMIC_ESM_LMAX_REG_OFFSET);

    if((PMIC_ESM_PWM_PULSE_MICROSEC_MAX < esmCfg.esmLmax_us) ||
       (PMIC_ESM_PWM_PULSE_MICROSEC_MIN > esmCfg.esmLmax_us))
    {
        pmicStatus = PMIC_ST_ERR_INV_ESM_VAL;
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        regData = (uint8_t)((esmCfg.esmLmax_us - PMIC_ESM_PWM_PULSE_MICROSEC_MIN) /
                    PMIC_ESM_PWM_PULSE_MICROSEC_DIV);

        /* Start Critical Section */
        Pmic_criticalSectionStart(pPmicCoreHandle);

        pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                            regAddr,
                                            regData);

        /* Stop Critical Section */
        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    return pmicStatus;
}

/*!
 * \brief   This function is used to Set the ESM LMIN value
 */
static int32_t Pmic_esmSetLminValue(Pmic_CoreHandle_t   *pPmicCoreHandle,
                                    const Pmic_EsmCfg_t  esmCfg,
                                    const uint8_t        esmBaseRegAddr)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData    = 0U;
    uint8_t regAddr    = 0U;

    regAddr = (esmBaseRegAddr + PMIC_ESM_LMIN_REG_OFFSET);

    if((PMIC_ESM_PWM_PULSE_MICROSEC_MAX < esmCfg.esmLmin_us) ||
       (PMIC_ESM_PWM_PULSE_MICROSEC_MIN > esmCfg.esmLmin_us))
    {
        pmicStatus = PMIC_ST_ERR_INV_ESM_VAL;
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        regData = (uint8_t)((esmCfg.esmLmin_us - PMIC_ESM_PWM_PULSE_MICROSEC_MIN) /
                    PMIC_ESM_PWM_PULSE_MICROSEC_DIV);

        /* Start Critical Section */
        Pmic_criticalSectionStart(pPmicCoreHandle);

        pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                            regAddr,
                                            regData);

        /* Stop Critical Section */
        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    return pmicStatus;
}

/*!
 * \brief   This function is used to configure the ESM EN DRV clear
 */
static int32_t Pmic_esmSetEnDrvValue(Pmic_CoreHandle_t   *pPmicCoreHandle,
                                     const Pmic_EsmCfg_t  esmCfg,
                                     const uint8_t        esmBaseRegAddr)
{
    int32_t pmicStatus  = PMIC_ST_SUCCESS;
    uint8_t regData     = 0U;
    uint8_t regAddr     = 0U;
    uint8_t esmU8Val = 0U;

    regAddr = (esmBaseRegAddr + PMIC_ESM_MODE_CFG_REG_OFFSET);
    esmU8Val = Pmic_esmGetU8Val(esmCfg.esmEnDrv);

    /* Start Critical Section */
    Pmic_criticalSectionStart(pPmicCoreHandle);

    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        regAddr,
                                        &regData);

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        Pmic_setBitField(&regData,
                         PMIC_ESM_X_MODE_CFG_ESM_X_ENDRV_SHIFT,
                         PMIC_ESM_X_MODE_CFG_ESM_X_ENDRV_MASK,
                         esmU8Val);

        pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                            regAddr,
                                            regData);
    }

    /* Stop Critical Section */
    Pmic_criticalSectionStop(pPmicCoreHandle);

    return pmicStatus;
}

/*!
 * \brief   This function is used to Set the ESM Mode
 */
static int32_t Pmic_esmSetMode(Pmic_CoreHandle_t   *pPmicCoreHandle,
                               const Pmic_EsmCfg_t  esmCfg,
                               const uint8_t        esmBaseRegAddr)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData    = 0U;
    uint8_t regAddr    = 0U;
    uint8_t esmU8Val = 0U;

    regAddr = (esmBaseRegAddr + PMIC_ESM_MODE_CFG_REG_OFFSET);
    esmU8Val = Pmic_esmGetU8Val(esmCfg.esmMode);

    /* Start Critical Section */
    Pmic_criticalSectionStart(pPmicCoreHandle);

    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        regAddr,
                                        &regData);

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        Pmic_setBitField(&regData,
                         PMIC_ESM_X_MODE_CFG_ESM_X_MODE_SHIFT,
                         PMIC_ESM_X_MODE_CFG_ESM_X_MODE_MASK,
                         esmU8Val);

        pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                            regAddr,
                                            regData);
    }

    /* Stop Critical Section */
    Pmic_criticalSectionStop(pPmicCoreHandle);

    return pmicStatus;
}

/*!
 * \brief   This function is used to Get the configured ESM DELAY1 value
 */
static int32_t Pmic_esmGetDelay1Value(Pmic_CoreHandle_t *pPmicCoreHandle,
                                      Pmic_EsmCfg_t     *pEsmCfg,
                                      const uint8_t      esmBaseRegAddr)
{
    int32_t pmicStatus  = PMIC_ST_SUCCESS;
    uint8_t regData     = 0U;
    uint8_t regAddr     = 0U;

    regAddr = (esmBaseRegAddr + PMIC_ESM_DELAY1_REG_OFFSET);

    /* Start Critical Section */
    Pmic_criticalSectionStart(pPmicCoreHandle);

    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        regAddr,
                                        &regData);

    /* Stop Critical Section */
    Pmic_criticalSectionStop(pPmicCoreHandle);

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        pEsmCfg->esmDelay1_us = ((uint32_t)regData * PMIC_ESM_DELAY_MICROSEC_DIV);
    }

    return pmicStatus;
}

/*!
 * \brief   This function is used to Get the configured ESM DELAY2 value
 */
static int32_t Pmic_esmGetDelay2Value(Pmic_CoreHandle_t *pPmicCoreHandle,
                                      Pmic_EsmCfg_t     *pEsmCfg,
                                      const uint8_t      esmBaseRegAddr)
{
    int32_t pmicStatus  = PMIC_ST_SUCCESS;
    uint8_t regData     = 0U;
    uint8_t regAddr     = 0U;

    regAddr = (esmBaseRegAddr + PMIC_ESM_DELAY2_REG_OFFSET);

    /* Start Critical Section */
    Pmic_criticalSectionStart(pPmicCoreHandle);

    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        regAddr,
                                        &regData);

    /* Stop Critical Section */
    Pmic_criticalSectionStop(pPmicCoreHandle);

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        pEsmCfg->esmDelay2_us = ((uint32_t)regData *
                                 PMIC_ESM_DELAY_MICROSEC_DIV);
    }

    return pmicStatus;
}

/*!
 * \brief   This function is used to Get the configured ESM Error count
 *          threshold value
 */
static int32_t Pmic_esmGetErrCntThrValue(Pmic_CoreHandle_t *pPmicCoreHandle,
                                         Pmic_EsmCfg_t     *pEsmCfg,
                                         const uint8_t      esmBaseRegAddr)
{
    int32_t pmicStatus  = PMIC_ST_SUCCESS;
    uint8_t regData     = 0U;
    uint8_t regAddr     = 0U;

    regAddr = (esmBaseRegAddr + PMIC_ESM_MODE_CFG_REG_OFFSET);

    /* Start Critical Section */
    Pmic_criticalSectionStart(pPmicCoreHandle);

    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        regAddr,
                                        &regData);

    /* Stop Critical Section */
    Pmic_criticalSectionStop(pPmicCoreHandle);

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        pEsmCfg->esmErrCntThr = Pmic_getBitField(regData,
                                  PMIC_ESM_X_MODE_CFG_ESM_X_ERR_CNT_TH_SHIFT,
                                  PMIC_ESM_X_MODE_CFG_ESM_X_ERR_CNT_TH_MASK);
    }

    return pmicStatus;
}

/*!
 * \brief   This function is used to Get the configured ESM HMAX value
 */
static int32_t Pmic_esmGetHmaxValue(Pmic_CoreHandle_t *pPmicCoreHandle,
                                    Pmic_EsmCfg_t     *pEsmCfg,
                                    const uint8_t      esmBaseRegAddr)
{
    int32_t pmicStatus  = PMIC_ST_SUCCESS;
    uint8_t regData     = 0U;
    uint8_t regAddr     = 0U;

    regAddr = (esmBaseRegAddr + PMIC_ESM_HMAX_REG_OFFSET);

    /* Start Critical Section */
    Pmic_criticalSectionStart(pPmicCoreHandle);

    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        regAddr,
                                        &regData);

    /* Stop Critical Section */
    Pmic_criticalSectionStop(pPmicCoreHandle);

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        pEsmCfg->esmHmax_us = ((((uint16_t)regData) *
                                  PMIC_ESM_PWM_PULSE_MICROSEC_DIV) +
                                ((uint16_t)PMIC_ESM_PWM_PULSE_MICROSEC_MIN));
    }

    return pmicStatus;
}

/*!
 * \brief   This function is used to Get the configured ESM HMIN value
 */
static int32_t Pmic_esmGetHminValue(Pmic_CoreHandle_t *pPmicCoreHandle,
                                    Pmic_EsmCfg_t     *pEsmCfg,
                                    const uint8_t      esmBaseRegAddr)
{
    int32_t pmicStatus  = PMIC_ST_SUCCESS;
    uint8_t regData     = 0U;
    uint8_t regAddr     = 0U;

    regAddr = (esmBaseRegAddr + PMIC_ESM_HMIN_REG_OFFSET);

    /* Start Critical Section */
    Pmic_criticalSectionStart(pPmicCoreHandle);

    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        regAddr,
                                        &regData);

    /* Stop Critical Section */
    Pmic_criticalSectionStop(pPmicCoreHandle);

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        pEsmCfg->esmHmin_us = ((((uint16_t)regData) *
                                  PMIC_ESM_PWM_PULSE_MICROSEC_DIV) +
                                ((uint16_t)PMIC_ESM_PWM_PULSE_MICROSEC_MIN));
    }

    return pmicStatus;
}

/*!
 * \brief   This function is used to Get the configured ESM LMAX value
 */
static int32_t Pmic_esmGetLmaxValue(Pmic_CoreHandle_t *pPmicCoreHandle,
                                    Pmic_EsmCfg_t     *pEsmCfg,
                                    const uint8_t      esmBaseRegAddr)
{
    int32_t pmicStatus  = PMIC_ST_SUCCESS;
    uint8_t regData     = 0U;
    uint8_t regAddr     = 0U;

    regAddr = (esmBaseRegAddr + PMIC_ESM_LMAX_REG_OFFSET);

    /* Start Critical Section */
    Pmic_criticalSectionStart(pPmicCoreHandle);

    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        regAddr,
                                        &regData);

    /* Stop Critical Section */
    Pmic_criticalSectionStop(pPmicCoreHandle);

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        pEsmCfg->esmLmax_us = ((((uint16_t)regData) *
                                  PMIC_ESM_PWM_PULSE_MICROSEC_DIV) +
                                ((uint16_t)PMIC_ESM_PWM_PULSE_MICROSEC_MIN));
    }

    return pmicStatus;
}

/*!
 * \brief   This function is used to Get the configured ESM LMIN value
 */
static int32_t Pmic_esmGetLminValue(Pmic_CoreHandle_t *pPmicCoreHandle,
                                    Pmic_EsmCfg_t     *pEsmCfg,
                                    const uint8_t      esmBaseRegAddr)
{
    int32_t pmicStatus  = PMIC_ST_SUCCESS;
    uint8_t regData     = 0U;
    uint8_t regAddr     = 0U;

    regAddr = (esmBaseRegAddr + PMIC_ESM_LMIN_REG_OFFSET);

    /* Start Critical Section */
    Pmic_criticalSectionStart(pPmicCoreHandle);

    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        regAddr,
                                        &regData);

    /* Stop Critical Section */
    Pmic_criticalSectionStop(pPmicCoreHandle);

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        pEsmCfg->esmLmin_us = ((((uint16_t)regData) *
                                  PMIC_ESM_PWM_PULSE_MICROSEC_DIV) +
                                ((uint16_t)PMIC_ESM_PWM_PULSE_MICROSEC_MIN));
    }

    return pmicStatus;
}

/*!
 * \brief   This function is used to Get the configured ESM EN DRV bit
 */
static int32_t Pmic_esmGetEnDrvValue(Pmic_CoreHandle_t *pPmicCoreHandle,
                                     Pmic_EsmCfg_t     *pEsmCfg,
                                     const uint8_t      esmBaseRegAddr)
{
    int32_t pmicStatus  = PMIC_ST_SUCCESS;
    uint8_t regData     = 0U;
    uint8_t regAddr     = 0U;
    uint8_t bitFieldVal = 0U;

    regAddr = (esmBaseRegAddr + PMIC_ESM_MODE_CFG_REG_OFFSET);

    /* Start Critical Section */
    Pmic_criticalSectionStart(pPmicCoreHandle);

    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        regAddr,
                                        &regData);

    /* Stop Critical Section */
    Pmic_criticalSectionStop(pPmicCoreHandle);

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        bitFieldVal = Pmic_getBitField(regData,
                                         PMIC_ESM_X_MODE_CFG_ESM_X_ENDRV_SHIFT,
                                         PMIC_ESM_X_MODE_CFG_ESM_X_ENDRV_MASK);
        if(bitFieldVal != 0U)
        {
            pEsmCfg->esmEnDrv = (bool)true;
        }
        else
        {
            pEsmCfg->esmEnDrv = (bool)false;
        }

    }

    return pmicStatus;
}

/*!
 * \brief   This function is used to Get the configured ESM Mode
 */
static int32_t Pmic_esmGetModeValue(Pmic_CoreHandle_t *pPmicCoreHandle,
                                    Pmic_EsmCfg_t     *pEsmCfg,
                                    const uint8_t      esmBaseRegAddr)
{
    int32_t pmicStatus  = PMIC_ST_SUCCESS;
    uint8_t regData     = 0U;
    uint8_t regAddr     = 0U;
    uint8_t bitFieldVal = 0U;

    regAddr = (esmBaseRegAddr + PMIC_ESM_MODE_CFG_REG_OFFSET);

    /* Start Critical Section */
    Pmic_criticalSectionStart(pPmicCoreHandle);

    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        regAddr,
                                        &regData);

    /* Stop Critical Section */
    Pmic_criticalSectionStop(pPmicCoreHandle);

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        bitFieldVal = Pmic_getBitField(regData,
                                       PMIC_ESM_X_MODE_CFG_ESM_X_MODE_SHIFT,
                                       PMIC_ESM_X_MODE_CFG_ESM_X_MODE_MASK);
        if(bitFieldVal != 0U)
        {
            pEsmCfg->esmMode = (bool)true;
        }
        else
        {
            pEsmCfg->esmMode = (bool)false;
        }

    }

    return pmicStatus;
}

/*!
 * \brief   API to start PMIC ESM.
 *
 * Requirement: REQ_TAG(PDK-5833)
 * Design: did_pmic_esm_cfg_readback
 * Architecture: aid_pmic_esm_cfg
 *
 *          This function is used to Start/Stop the PMIC ESM_MCU/ESM_SOC
 *          Note: Application has to ensure to do proper configuration of ESM
 *                time intervals of Level or PWM  mode.If not configured
 *                properly then ESM will trigger the warm reset to the PMIC
 *                device. This may cause system reset if PMIC is connected to
 *                SOC/MCU
 *
 * \param   pPmicCoreHandle [IN]    PMIC Interface Handle.
 * \param   esmType         [IN]    PMIC ESM Type
 *                                  For valid values:
 *                                  \ref Pmic_EsmTypes
 * \param   esmState        [IN]    To start or stop PMIC ESM
 *                                  For valid values:
 *                                  \ref Pmic_EsmStates
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_esmStart(Pmic_CoreHandle_t   *pPmicCoreHandle,
                      const bool           esmType,
                      const bool           esmState)
{
    int32_t pmicStatus      = PMIC_ST_SUCCESS;
    uint8_t esmBaseRegAddr  = 0U;

    pmicStatus = Pmic_esmValidateParams(pPmicCoreHandle);

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        pmicStatus = Pmic_esmDeviceCheck(pPmicCoreHandle, esmType);
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        Pmic_esmGetBaseRegAddr(esmType, &esmBaseRegAddr);
        pmicStatus = Pmic_esmXStart(pPmicCoreHandle,
                                    esmBaseRegAddr,
                                    esmState);
    }

    return pmicStatus;
}

/*!
 * \brief   API to Enable/Disable PMIC ESM.
 *
 * Requirement: REQ_TAG(PDK-5833)
 * Design: did_pmic_esm_cfg_readback
 * Architecture: aid_pmic_esm_cfg
 *
 *          This function is used to Enable/Disable the PMIC ESM_MCU/ESM_SOC
 *          This API must be called only when ESM is in STOP state.
 *
 * \param   pPmicCoreHandle [IN]    PMIC Interface Handle.
 * \param   esmType         [IN]    PMIC ESM Type
 *                                  For valid values:
 *                                  \ref Pmic_EsmTypes
 * \param   esmToggle       [IN]    To Enable/Disable PMIC ESM_MCU/ESM_SOC
 *                                  For valid values:
 *                                  \ref Pmic_EsmToggle
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_esmEnable(Pmic_CoreHandle_t   *pPmicCoreHandle,
                       const bool           esmType,
                       const bool           esmToggle)
{
    int32_t pmicStatus     = PMIC_ST_SUCCESS;
    uint8_t esmBaseRegAddr = 0U;

    pmicStatus = Pmic_esmValidateParams(pPmicCoreHandle);

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        pmicStatus = Pmic_esmDeviceCheck(pPmicCoreHandle, esmType);
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        Pmic_esmGetBaseRegAddr(esmType, &esmBaseRegAddr);
        pmicStatus = Pmic_esmXEnable(pPmicCoreHandle,
                                     esmBaseRegAddr,
                                     esmToggle);
    }

    return pmicStatus;
}

/*!
 * \brief   API to Read PMIC ESM Enable/Disable state.
 *
 * Requirement: REQ_TAG(PDK-5833)
 * Design: did_pmic_esm_cfg_readback
 * Architecture: aid_pmic_esm_cfg
 *
 *          This function is used to read the Enable/Disable state of
 *          PMIC ESM_MCU/ESM_SOC.
 *
 * \param   pPmicCoreHandle [IN]    PMIC Interface Handle.
 * \param   esmType         [IN]    PMIC ESM Type
 *                                  For valid values:
 *                                  \ref Pmic_EsmTypes
 * \param   pEsmState       [OUT]   Pointer to store ESM Enable State.
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_esmGetEnableState(Pmic_CoreHandle_t   *pPmicCoreHandle,
                               const bool           esmType,
                               bool                *pEsmState)
{
    int32_t pmicStatus     = PMIC_ST_SUCCESS;
    uint8_t esmBaseRegAddr = 0U;
    uint8_t regAddr        = 0U;
    uint8_t regData        = 0U;

    pmicStatus = Pmic_esmValidateParams(pPmicCoreHandle);

    if((PMIC_ST_SUCCESS == pmicStatus) && (NULL == pEsmState))
    {
        pmicStatus = PMIC_ST_ERR_NULL_PARAM;
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        pmicStatus = Pmic_esmDeviceCheck(pPmicCoreHandle, esmType);
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        Pmic_esmGetBaseRegAddr(esmType, &esmBaseRegAddr);
        regAddr = (esmBaseRegAddr + PMIC_ESM_MODE_CFG_REG_OFFSET);
        *pEsmState = (bool)false;

        /* Start Critical Section */
        Pmic_criticalSectionStart(pPmicCoreHandle);

        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                            regAddr,
                                            &regData);

        /* Stop Critical Section */
        Pmic_criticalSectionStop(pPmicCoreHandle);

        if((PMIC_ST_SUCCESS == pmicStatus) &&
           (Pmic_getBitField(regData,
                             PMIC_ESM_X_MODE_CFG_ESM_X_EN_SHIFT,
                             PMIC_ESM_X_MODE_CFG_ESM_X_EN_MASK)
            == PMIC_ESM_VAL_1))
        {
            *pEsmState = (bool)true;
        }
    }

    return pmicStatus;
}

/*!
 * \brief   This function is used to Set the ESM Error Count Threshold value,
 *          ESM EN DRV clear and ESM Mode configuration
 */
static int32_t Pmic_esmSetErrcntthresholdEndrvClrModeCfg(
                                          Pmic_CoreHandle_t   *pPmicCoreHandle,
                                          const Pmic_EsmCfg_t  esmCfg,
                                          const uint8_t        esmBaseRegAddr)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    if(((bool)true) == pmic_validParamCheck(esmCfg.validParams,
                                            PMIC_ESM_CFG_ERR_CNT_THR_VALID))
    {
        if(PMIC_ESM_ERR_CNT_THR_MAX < esmCfg.esmErrCntThr)
        {
            pmicStatus = PMIC_ST_ERR_INV_PARAM;
        }

        if(PMIC_ST_SUCCESS == pmicStatus)
        {
            pmicStatus = Pmic_esmSetErrCntThrValue(pPmicCoreHandle,
                                                   esmCfg,
                                                   esmBaseRegAddr);
        }
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (((bool)true) == pmic_validParamCheck(esmCfg.validParams,
                                           PMIC_ESM_CFG_EN_DRV_VALID)))
    {
        /* Set ESM EN DRV */
        pmicStatus = Pmic_esmSetEnDrvValue(pPmicCoreHandle,
                                           esmCfg,
                                           esmBaseRegAddr);
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (((bool)true) == pmic_validParamCheck(esmCfg.validParams,
                                           PMIC_ESM_CFG_MODE_VALID)))
    {
        /* Set ESM Mode */
        pmicStatus = Pmic_esmSetMode(pPmicCoreHandle,
                                     esmCfg,
                                     esmBaseRegAddr);
    }

    return pmicStatus;
}

/*!
 * \brief   This function is used to Set the ESM HMAX, HMIN, LMAX, & LMIN value
 *          configuration
 */
static int32_t Pmic_esmSetHmaxHminLmaxLminCfg(
                                           Pmic_CoreHandle_t  *pPmicCoreHandle,
                                           const Pmic_EsmCfg_t esmCfg,
                                           const uint8_t       esmBaseRegAddr)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    if(((bool)true) == pmic_validParamCheck(esmCfg.validParams,
                                           PMIC_ESM_CFG_HMAX_VALID))
    {
        pmicStatus = Pmic_esmSetHmaxValue(pPmicCoreHandle,
                                          esmCfg,
                                          esmBaseRegAddr);
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (((bool)true) == pmic_validParamCheck(esmCfg.validParams,
                                           PMIC_ESM_CFG_HMIN_VALID)))
    {
        pmicStatus = Pmic_esmSetHminValue(pPmicCoreHandle,
                                          esmCfg,
                                          esmBaseRegAddr);
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (((bool)true) == pmic_validParamCheck(esmCfg.validParams,
                                           PMIC_ESM_CFG_LMAX_VALID)))
    {
        pmicStatus = Pmic_esmSetLmaxValue(pPmicCoreHandle,
                                          esmCfg,
                                          esmBaseRegAddr);
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (((bool)true) == pmic_validParamCheck(esmCfg.validParams,
                                           PMIC_ESM_CFG_LMIN_VALID)))
    {
        pmicStatus = Pmic_esmSetLminValue(pPmicCoreHandle,
                                          esmCfg,
                                          esmBaseRegAddr);
    }

    return pmicStatus;
}

/*!
 * \brief   This function is used to set the ESM mode, delay-1 and delay-2 time
 *          time intervals, Error Count Threshold value, HMAX, HMIN, LMAX,
 *          LMIN and select EN DRV clear for ESM_MCU and ESM_SOC.
 */
static int32_t Pmic_esmSetConfig(Pmic_CoreHandle_t  *pPmicCoreHandle,
                                 const Pmic_EsmCfg_t esmCfg,
                                 const uint8_t       esmBaseRegAddr)
{
    int32_t pmicStatus     = PMIC_ST_SUCCESS;

    if(((bool)true) == pmic_validParamCheck(esmCfg.validParams,
                                           PMIC_ESM_CFG_DELAY1_VALID))
    {
        pmicStatus = Pmic_esmSetDelay1Value(pPmicCoreHandle,
                                            esmCfg,
                                            esmBaseRegAddr);
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (((bool)true) == pmic_validParamCheck(esmCfg.validParams,
                                           PMIC_ESM_CFG_DELAY2_VALID)))
    {
        pmicStatus = Pmic_esmSetDelay2Value(pPmicCoreHandle,
                                            esmCfg,
                                            esmBaseRegAddr);
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        pmicStatus = Pmic_esmSetErrcntthresholdEndrvClrModeCfg(pPmicCoreHandle,
                                                               esmCfg,
                                                               esmBaseRegAddr);
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        pmicStatus = Pmic_esmSetHmaxHminLmaxLminCfg(pPmicCoreHandle,
                                                    esmCfg,
                                                    esmBaseRegAddr);
    }

    return pmicStatus;
}

/*!
 * \brief   API to Set PMIC ESM Configuration.
 *
 * Requirement: REQ_TAG(PDK-5833)
 * Design: did_pmic_esm_cfg_readback
 * Architecture: aid_pmic_esm_cfg
 *
 *          This function is used to set the ESM mode, delay-1 and delay-2 time
 *          time intervals, Error Count Threshold value, HMAX, HMIN, LMAX,
 *          LMIN and select EN DRV clear for ESM_MCU and ESM_SOC.
 *          This API must be called only when ESM is in STOP State.
 *
 * \param   pPmicCoreHandle [IN]    PMIC Interface Handle.
 * \param   esmType         [IN]    PMIC ESM Type
 *                                  For valid values:
 *                                  \ref Pmic_EsmTypes
 * \param   esmCfg          [IN]    PMIC ESM Configuration
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_esmSetConfiguration(Pmic_CoreHandle_t   *pPmicCoreHandle,
                                 const bool           esmType,
                                 const Pmic_EsmCfg_t  esmCfg)
{
    int32_t pmicStatus     = PMIC_ST_SUCCESS;
    uint8_t esmBaseRegAddr = 0U;

    pmicStatus = Pmic_esmValidateParams(pPmicCoreHandle);

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        pmicStatus = Pmic_esmDeviceCheck(pPmicCoreHandle, esmType);
    }

    if((PMIC_ST_SUCCESS == pmicStatus) && (0U == esmCfg.validParams))
    {
        pmicStatus = PMIC_ST_ERR_INSUFFICIENT_CFG;
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        Pmic_esmGetBaseRegAddr(esmType, &esmBaseRegAddr);
        pmicStatus = Pmic_esmCheckState(pPmicCoreHandle, esmBaseRegAddr);
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        pmicStatus = Pmic_esmSetConfig(pPmicCoreHandle, esmCfg, esmBaseRegAddr);
    }

    return pmicStatus;
}

/*!
 * \brief   This function is used to Get the ESM HMAX, HMIN, LMAX, & LMIN value
 */
static int32_t Pmic_esmGetHmaxHminLmaxLminCfg(
                                            Pmic_CoreHandle_t  *pPmicCoreHandle,
                                            Pmic_EsmCfg_t      *pEsmCfg,
                                            const uint8_t       esmBaseRegAddr)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    if(((bool)true) == pmic_validParamCheck(pEsmCfg->validParams,
                                           PMIC_ESM_CFG_HMAX_VALID))
    {
        pmicStatus = Pmic_esmGetHmaxValue(pPmicCoreHandle,
                                          pEsmCfg,
                                          esmBaseRegAddr);
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (((bool)true) == pmic_validParamCheck(pEsmCfg->validParams,
                                           PMIC_ESM_CFG_HMIN_VALID)))
    {
        pmicStatus = Pmic_esmGetHminValue(pPmicCoreHandle,
                                          pEsmCfg,
                                          esmBaseRegAddr);
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (((bool)true) == pmic_validParamCheck(pEsmCfg->validParams,
                                           PMIC_ESM_CFG_LMAX_VALID)))
    {
        pmicStatus = Pmic_esmGetLmaxValue(pPmicCoreHandle,
                                          pEsmCfg,
                                          esmBaseRegAddr);
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (((bool)true) == pmic_validParamCheck(pEsmCfg->validParams,
                                           PMIC_ESM_CFG_LMIN_VALID)))
    {
        pmicStatus = Pmic_esmGetLminValue(pPmicCoreHandle,
                                          pEsmCfg,
                                          esmBaseRegAddr);
    }

    return pmicStatus;
}

/*!
 * \brief   This function is used to to get the configured ESM mode, delay-1 and
 *          delay-2 time time intervals, Error Count Threshold value, HMAX,
 *          HMIN, LMAX, LMIN and select EN DRV clear for ESM_MCU and ESM_SOC
 */
static int32_t Pmic_esmGetConfig(Pmic_CoreHandle_t   *pPmicCoreHandle,
                                 Pmic_EsmCfg_t       *pEsmCfg,
                                 const uint8_t        esmBaseRegAddr)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    if(((bool)true) == pmic_validParamCheck(pEsmCfg->validParams,
                                           PMIC_ESM_CFG_DELAY1_VALID))
    {
        pmicStatus = Pmic_esmGetDelay1Value(pPmicCoreHandle,
                                            pEsmCfg,
                                            esmBaseRegAddr);
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (((bool)true) == pmic_validParamCheck(pEsmCfg->validParams,
                                           PMIC_ESM_CFG_DELAY2_VALID)))
    {
        pmicStatus = Pmic_esmGetDelay2Value(pPmicCoreHandle,
                                            pEsmCfg,
                                            esmBaseRegAddr);
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (((bool)true) == pmic_validParamCheck(pEsmCfg->validParams,
                                           PMIC_ESM_CFG_ERR_CNT_THR_VALID)))
    {
        pmicStatus = Pmic_esmGetErrCntThrValue(pPmicCoreHandle,
                                               pEsmCfg,
                                               esmBaseRegAddr);
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (((bool)true) == pmic_validParamCheck(pEsmCfg->validParams,
                                           PMIC_ESM_CFG_EN_DRV_VALID)))
    {
        /* Get ESM EN DRV */
        pmicStatus = Pmic_esmGetEnDrvValue(pPmicCoreHandle,
                                           pEsmCfg,
                                           esmBaseRegAddr);
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (((bool)true) == pmic_validParamCheck(pEsmCfg->validParams,
                                           PMIC_ESM_CFG_MODE_VALID)))
    {
        /* Get ESM Mode */
        pmicStatus = Pmic_esmGetModeValue(pPmicCoreHandle,
                                          pEsmCfg,
                                          esmBaseRegAddr);
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        pmicStatus = Pmic_esmGetHmaxHminLmaxLminCfg(pPmicCoreHandle,
                                                    pEsmCfg,
                                                    esmBaseRegAddr);
    }

    return pmicStatus;
}

/*!
 * \brief   API to Get the PMIC ESM Configuration.
 *
 * Requirement: REQ_TAG(PDK-5833)
 * Design: did_pmic_esm_cfg_readback
 * Architecture: aid_pmic_esm_cfg
 *
 *          This function is used to get the configured ESM mode, delay-1 and
 *          delay-2 time time intervals, Error Count Threshold value, HMAX,
 *          HMIN, LMAX, LMIN and select EN DRV clear for ESM_MCU and ESM_SOC.
 *
 * \param   pPmicCoreHandle [IN]       PMIC Interface Handle.
 * \param   esmType         [IN]       PMIC ESM Type.
 *                                     For valid values:
 *                                     \ref Pmic_EsmTypes.
 * \param   pEsmCfg         [IN/OUT]   Pointer to store the specified ESM
 *                                     configuration.
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values: \ref Pmic_ErrorCodes
 */
int32_t Pmic_esmGetConfiguration(Pmic_CoreHandle_t   *pPmicCoreHandle,
                                 const bool           esmType,
                                 Pmic_EsmCfg_t       *pEsmCfg)
{
    int32_t pmicStatus      = PMIC_ST_SUCCESS;
    uint8_t esmBaseRegAddr  = 0U;

    pmicStatus = Pmic_esmValidateParams(pPmicCoreHandle);

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        pmicStatus = Pmic_esmDeviceCheck(pPmicCoreHandle, esmType);
    }

    if((PMIC_ST_SUCCESS == pmicStatus) && (NULL == pEsmCfg))
    {
        pmicStatus = PMIC_ST_ERR_NULL_PARAM;
    }

    if((PMIC_ST_SUCCESS == pmicStatus) && (0U == pEsmCfg->validParams))
    {
        pmicStatus = PMIC_ST_ERR_INSUFFICIENT_CFG;
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        Pmic_esmGetBaseRegAddr(esmType, &esmBaseRegAddr);

        pmicStatus = Pmic_esmGetConfig(pPmicCoreHandle,
                                       pEsmCfg,
                                       esmBaseRegAddr);
    }

    return pmicStatus;
}

/*!
 * \brief   API to Set PMIC ESM Interrupts.
 *
 * Requirement: REQ_TAG(PDK-5833)
 * Design: did_pmic_esm_cfg_readback
 * Architecture: aid_pmic_esm_cfg
 *
 *          This function is used to mask/unmask the ESM RST, FAIL and
 *          PIN Interrupts for both ESM MCU and ESM SOC.
 *
 * \param   pPmicCoreHandle [IN]    PMIC Interface Handle.
 * \param   esmType         [IN]    PMIC ESM Type
 *                                  For valid values:
 *                                  \ref Pmic_EsmTypes
 * \param   esmIntrCfg      [IN]    PMIC ESM interrupts mask/un-mask.
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_esmSetInterrupt(Pmic_CoreHandle_t        *pPmicCoreHandle,
                             const bool                esmType,
                             const Pmic_EsmIntrCfg_t   esmIntrCfg)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    pmicStatus = Pmic_esmValidateParams(pPmicCoreHandle);

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        pmicStatus = Pmic_esmDeviceCheck(pPmicCoreHandle, esmType);
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        pmicStatus = Pmic_esmIntrEnable(pPmicCoreHandle,
                                        esmType,
                                        esmIntrCfg);
    }

    return pmicStatus;
}

/*!
 * \brief   API to Get the current ESM Error Count.
 *
 * Requirement: REQ_TAG(PDK-5833)
 * Design: did_pmic_esm_cfg_readback
 * Architecture: aid_pmic_esm_cfg
 *
 *          This function is used to get the current Error count for ESM MCU
 *          ESM SOC.
 *
 * \param   pPmicCoreHandle [IN]    PMIC Interface Handle.
 * \param   esmType         [IN]    PMIC ESM Type.
 *                                  For valid values:
 *                                  \ref Pmic_EsmTypes.
 * \param   pEsmErrCnt      [OUT]   Pointer to store the Error Count.
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code.
 *          For valid values: \ref Pmic_ErrorCodes
 */
int32_t Pmic_esmGetErrCnt(Pmic_CoreHandle_t   *pPmicCoreHandle,
                          const bool           esmType,
                          uint8_t             *pEsmErrCnt)
{
    int32_t pmicStatus     = PMIC_ST_SUCCESS;
    uint8_t esmBaseRegAddr = 0U;
    uint8_t regAddr        = 0U;
    uint8_t regData        = 0U;

    pmicStatus = Pmic_esmValidateParams(pPmicCoreHandle);

    if((PMIC_ST_SUCCESS == pmicStatus) && (NULL == pEsmErrCnt))
    {
        pmicStatus = PMIC_ST_ERR_NULL_PARAM;
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        pmicStatus = Pmic_esmDeviceCheck(pPmicCoreHandle, esmType);
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        Pmic_esmGetBaseRegAddr(esmType, &esmBaseRegAddr);
        regAddr = (esmBaseRegAddr + PMIC_ESM_ERR_CNT_REG_OFFSET);

        /* Start Critical Section */
        Pmic_criticalSectionStart(pPmicCoreHandle);

        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                            regAddr,
                                            &regData);

        /* Stop Critical Section */
        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        (*pEsmErrCnt) = Pmic_getBitField(regData,
                                  PMIC_ESM_X_ERR_CNT_REG_ESM_X_ERR_CNT_SHIFT,
                                  PMIC_ESM_X_ERR_CNT_REG_ESM_X_ERR_CNT_MASK);
    }

    return pmicStatus;
}

/*!
 * \brief   API to read status of PMIC ESM is started or not.
 *
 * Requirement: REQ_TAG(PDK-9150)
 * Design: did_pmic_esm_cfg_readback
 * Architecture: aid_pmic_esm_cfg
 *
 *          This function is used to read status of PMIC ESM_MCU/ESM_SOC is
 *          started or not
 *
 * \param   pPmicCoreHandle [IN]    PMIC Interface Handle.
 * \param   esmType         [IN]    PMIC ESM Type
 *                                  For valid values:
 *                                  \ref Pmic_EsmTypes
 * \param   pEsmState        [IN]   Pointer to store the status of PMIC ESM is
 *                                  started or not
 *                                  For valid values:
 *                                  \ref Pmic_EsmStates
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_esmGetStatus(Pmic_CoreHandle_t   *pPmicCoreHandle,
                          const bool           esmType,
                          bool                *pEsmState)
{

    int32_t pmicStatus      = PMIC_ST_SUCCESS;
    uint8_t esmBaseRegAddr  = 0U;
    uint8_t regAddr        = 0U;
    uint8_t regData        = 0U;

    pmicStatus = Pmic_esmValidateParams(pPmicCoreHandle);

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        pmicStatus = Pmic_esmDeviceCheck(pPmicCoreHandle, esmType);
    }

    if((PMIC_ST_SUCCESS == pmicStatus) && (NULL == pEsmState))
    {
        pmicStatus = PMIC_ST_ERR_NULL_PARAM;
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        Pmic_esmGetBaseRegAddr(esmType, &esmBaseRegAddr);

        regAddr = (esmBaseRegAddr + PMIC_ESM_START_REG_OFFSET);
        *pEsmState = PMIC_ESM_STOP;

        /* Start Critical Section */
        Pmic_criticalSectionStart(pPmicCoreHandle);

        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                            regAddr,
                                            &regData);

        /* Stop Critical Section */
        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (Pmic_getBitField(regData,
                         PMIC_ESM_X_START_REG_ESM_X_START_SHIFT,
                         PMIC_ESM_X_START_REG_ESM_X_START_MASK)
        == PMIC_ESM_VAL_1))
    {
        *pEsmState = PMIC_ESM_START;
    }

    return pmicStatus;
}
