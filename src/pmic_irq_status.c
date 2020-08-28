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
 * \file   pmic_irq_status.c
 *
 * \brief  This file contains APIs definitions for PMIC Interrupt IRQ Status
 *         Handler.
 *
 */

#include <pmic_irq.h>
#include <pmic_core_priv.h>
#include <pmic_irq_tps6594x_priv.h>
#include <pmic_irq_lp8764x_priv.h>
#include <cfg/tps6594x/pmic_irq_tps6594x.h>
#include <cfg/lp8764x/pmic_irq_lp8764x.h>
#include <pmic_irq_priv.h>

/*!
 * \brief  Function to Set the intStatus bit position.
 */
void Pmic_intrBitSet(Pmic_IrqStatus_t  *pErrStat, uint32_t pos)
{
    uint32_t intStatSize = 0U;

    /* Size of intStatus in bits */
    intStatSize = sizeof(pErrStat->intStatus[0U]) << 3U;

    pErrStat->intStatus[pos / intStatSize] |=
                            (((uint32_t)1U) << (pos % intStatSize));
}

/*!
 * \brief  Function to Clear the intStatus bit position.
 */
static void Pmic_intrBitClear(Pmic_IrqStatus_t *pErrStat,
                              const uint8_t    *pIrqNum)
{
    uint32_t intStatSize = 0U;

    /* Size of intStatus in bits */
    intStatSize = sizeof(pErrStat->intStatus[0U]) << 3U;

    pErrStat->intStatus[(*pIrqNum) / intStatSize] &=
                                         ~(1U << ((*pIrqNum) % intStatSize));
}

/*!
 * \brief  Function to Extract the intStatus bit position.
 */
static uint8_t Pmic_intrBitExtract(const Pmic_IrqStatus_t *pErrStat,
                                   uint8_t                 maxVal)
{
    uint8_t irqNum       = 0U;
    uint32_t intStatSize = 0U;

    /* Size of intStatus in bits */
    intStatSize = sizeof(pErrStat->intStatus[0U]) << 3U;
    for(irqNum = 0U; irqNum < maxVal; irqNum++)
    {
        if(((pErrStat->intStatus[irqNum / intStatSize]) & 
            (((uint32_t)1U) << (irqNum % intStatSize))) != 0U)
        {
            break;
        }
    }

    return irqNum;
}

/*! 
 * \brief  Function to Check the device specific Max IrqNum.
 */
static int32_t Pmic_irqValidateIrqNum(const Pmic_CoreHandle_t  *pPmicCoreHandle,
                                      const uint8_t             irqNum)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    switch(pPmicCoreHandle->pmicDeviceType)
    {
        case PMIC_DEV_LEO_TPS6594X:
            if((irqNum > PMIC_TPS6594X_IRQ_MAX_NUM) &&
               (irqNum != PMIC_IRQ_ALL))
            {
                pmicStatus = PMIC_ST_ERR_INV_PARAM;
            }
            break;
        case PMIC_DEV_HERA_LP8764X:
            if((irqNum > PMIC_LP8764X_IRQ_MAX_NUM) && (irqNum != PMIC_IRQ_ALL))
            {
                pmicStatus = PMIC_ST_ERR_INV_PARAM;
            }
            break;
        default:
            pmicStatus = PMIC_ST_ERR_INV_DEVICE;
            break;
    }

    return pmicStatus;
}


/*! 
 * \brief  Function to get the device specific Max IrqNum.
 */
static int32_t Pmic_getMaxVal(const Pmic_CoreHandle_t  *pPmicCoreHandle,
                              uint8_t                  *maxVal)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    switch(pPmicCoreHandle->pmicDeviceType)
    {
        case PMIC_DEV_LEO_TPS6594X:
            (*maxVal) = PMIC_TPS6594X_IRQ_MAX_NUM;
            break;

        case PMIC_DEV_HERA_LP8764X:
            (*maxVal) = PMIC_LP8764X_IRQ_MAX_NUM;
            break;

        default:
            pmicStatus = PMIC_ST_ERR_INV_DEVICE;
            break;
    }

    return pmicStatus;
}

/*! 
 * \brief  Function to get the Device specific Interrupt Configuration
 *         registers.
 */
static int32_t Pmic_get_intrCfg(const Pmic_CoreHandle_t  *pPmicCoreHandle,
                                Pmic_IntrCfg_t          **pIntrCfg)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    if(NULL == pIntrCfg)
    {
        pmicStatus = PMIC_ST_ERR_NULL_PARAM;
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        switch(pPmicCoreHandle->pmicDeviceType)
        {
        case PMIC_DEV_LEO_TPS6594X:
            pmic_get_tps6594x_intrCfg(pIntrCfg);
            break;

        case PMIC_DEV_HERA_LP8764X:
            pmic_get_lp8764x_intrCfg(pIntrCfg);
            break;

        default:
            pmicStatus = PMIC_ST_ERR_INV_DEVICE;
            break;
        }
    }

    return pmicStatus;
}

/*!
 * \brief  Function to get the Device specific GPIO Interrupt configuration 
 *         registers.
 */
static int32_t Pmic_get_gpioIntrCfg(const Pmic_CoreHandle_t *pPmicCoreHandle,
                                    Pmic_GpioIntrTypeCfg_t **pGpioIntrCfg)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    if(NULL == pGpioIntrCfg)
    {
        pmicStatus = PMIC_ST_ERR_NULL_PARAM;
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        switch(pPmicCoreHandle->pmicDeviceType)
        {
        case PMIC_DEV_LEO_TPS6594X:
            pmic_get_tps6594x_intrGpioCfg(pGpioIntrCfg);
            break;

        case PMIC_DEV_HERA_LP8764X:
            pmic_get_lp8764x_intrGpioCfg(pGpioIntrCfg);
            break;

        default:
            pmicStatus = PMIC_ST_ERR_INV_DEVICE;
            break;
        }
    }

    return pmicStatus;
}

/*!
 * \brief  Function to Mask/Unmask GPIO Interrupts.
 */
static int32_t Pmic_irqGpioMask(Pmic_CoreHandle_t *pPmicCoreHandle,
                                const uint8_t      irqGpioNum,
                                const bool         mask,
                                const uint8_t      gpioIntrType)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData    = 0U;
    Pmic_GpioIntrTypeCfg_t *pGpioIntrCfg = NULL;
    uint8_t bitMask    = 0U;
    uint8_t maskVal    = 0U;

    pmicStatus = Pmic_get_gpioIntrCfg(pPmicCoreHandle, &pGpioIntrCfg);

    /* Start Critical Section */
    Pmic_criticalSectionStart(pPmicCoreHandle);

    if((PMIC_IRQ_GPIO_RISE_INT_TYPE == gpioIntrType) ||
       (PMIC_IRQ_GPIO_RISE_FALL_INT_TYPE == gpioIntrType))
    {
        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                          pGpioIntrCfg[irqGpioNum].gpioRiseIntrMaskRegAddr,
                          &regData);

        if(PMIC_ST_SUCCESS == pmicStatus)
        {
            if(((bool)true) == mask)
            {
                maskVal = 1U;
            }
            bitMask = (PMIC_IRQ_MASK_CLR_BITFIELD <<
                       pGpioIntrCfg[irqGpioNum].gpioRiseMaskBitPos);
            Pmic_setBitField(&regData,
                             pGpioIntrCfg[irqGpioNum].gpioRiseMaskBitPos,
                             bitMask,
                             maskVal);
            pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle,
                          pGpioIntrCfg[irqGpioNum].gpioRiseIntrMaskRegAddr,
                          regData);
        }
    }

    if((PMIC_IRQ_GPIO_FALL_INT_TYPE == gpioIntrType) || 
       (PMIC_IRQ_GPIO_RISE_FALL_INT_TYPE == gpioIntrType))
    {
        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                          pGpioIntrCfg[irqGpioNum].gpioFallIntrMaskRegAddr,
                          &regData);

        if(PMIC_ST_SUCCESS == pmicStatus)
        {
            if(((bool)true) == mask)
            {
                maskVal = 1U;
            }
            bitMask = (PMIC_IRQ_MASK_CLR_BITFIELD <<
                       pGpioIntrCfg[irqGpioNum].gpioFallMaskBitPos);
            Pmic_setBitField(&regData,
                             pGpioIntrCfg[irqGpioNum].gpioFallMaskBitPos,
                             bitMask,
                             maskVal);
            pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle,
                          pGpioIntrCfg[irqGpioNum].gpioFallIntrMaskRegAddr,
                          regData);
        }
    }

    /* Stop Critical Section */
    Pmic_criticalSectionStop(pPmicCoreHandle);

    return pmicStatus;
}

/*!
 * \brief  Function to Mask/Unmask GPIO Interrupts.
 */
static int32_t Pmic_maskGpioIntr(Pmic_CoreHandle_t *pPmicCoreHandle,
                                 const uint8_t      irqGpioNum,
                                 const bool         mask,
                                 const uint8_t      gpioIntrType)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t irqGpioId  = 0U;

    if(PMIC_IRQ_GPIO_ALL_INT_MASK_NUM != irqGpioNum)
    {
        pmicStatus = Pmic_irqGpioMask(pPmicCoreHandle,
                                      irqGpioNum,
                                      mask,
                                      gpioIntrType);
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (PMIC_IRQ_GPIO_ALL_INT_MASK_NUM == irqGpioNum))
    {
        for(irqGpioId = 0U; irqGpioId < (PMIC_IRQ_GPIO_ALL_INT_MASK_NUM - 1U); irqGpioId++)
        {
            pmicStatus = Pmic_irqGpioMask(pPmicCoreHandle,
                                          irqGpioId,
                                          mask,
                                          gpioIntrType);
        }
    }

    return pmicStatus;
}

/*! 
 * \brief  Function to clear IRQ status.
 */
static int32_t Pmic_irqClear(Pmic_CoreHandle_t *pPmicCoreHandle,
                             const uint8_t      irqNum)
{
    int32_t pmicStatus       = PMIC_ST_SUCCESS;
    uint8_t regData          = 0U;
    Pmic_IntrCfg_t *pIntrCfg = NULL;
    uint8_t bitMask          = 0U;

    pmicStatus = Pmic_get_intrCfg(pPmicCoreHandle, &pIntrCfg);

    /* Start Critical Section */
    Pmic_criticalSectionStart(pPmicCoreHandle);

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                            pIntrCfg[irqNum].intrClrRegAddr,
                                            &regData);
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        bitMask = (PMIC_IRQ_MASK_CLR_BITFIELD <<
                   pIntrCfg[irqNum].intrClrBitPos);
        Pmic_setBitField(&regData,
                         pIntrCfg[irqNum].intrClrBitPos,
                         bitMask,
                         PMIC_IRQ_CLEAR);
        pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                           pIntrCfg[irqNum].intrClrRegAddr,
                                           regData);
    }

    /* Stop Critical Section */
    Pmic_criticalSectionStop(pPmicCoreHandle);

    return pmicStatus;
}

/*! 
 * \brief  Function to Clear Interrupt Status register.
 */
static int32_t Pmic_irqClearStatus(Pmic_CoreHandle_t *pPmicCoreHandle,
                                   const uint8_t      irqNum)
{
    int32_t pmicStatus       = PMIC_ST_SUCCESS;
    uint8_t irqId            = 0U;
    uint8_t maxVal           = 0U;

    if(PMIC_IRQ_ALL != irqNum)
    {
        pmicStatus = Pmic_irqClear(pPmicCoreHandle, irqNum);
    }

    if((PMIC_ST_SUCCESS == pmicStatus) && (PMIC_IRQ_ALL == irqNum))
    {
        pmicStatus = Pmic_getMaxVal(pPmicCoreHandle, &maxVal);
        if(PMIC_ST_SUCCESS == pmicStatus)
        {
            for(irqId = 0U; irqId < maxVal; irqId++)
            {
                pmicStatus = Pmic_irqClear(pPmicCoreHandle, irqId);
            }
        }
    }

    return pmicStatus;
}

/*!
 * \brief  Function to Mask/Unmask Interrupts.
 */
static int32_t Pmic_irqMask(Pmic_CoreHandle_t    *pPmicCoreHandle,
                            const uint8_t         irqNum,
                            const bool            mask,
                            const Pmic_IntrCfg_t *pIntrCfg)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData    = 0U;
    uint8_t bitMask    = 0U;
    uint8_t maskVal    = 0U;

    /* Start Critical Section */
    Pmic_criticalSectionStart(pPmicCoreHandle);

    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        pIntrCfg[irqNum].intrMaskRegAddr,
                                        &regData);

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        if(((bool)true) == mask)
        {
            maskVal = 1U;
        }
        bitMask = (PMIC_IRQ_MASK_CLR_BITFIELD <<
                   pIntrCfg[irqNum].intrMaskBitPos);
        Pmic_setBitField(&regData,
                         pIntrCfg[irqNum].intrMaskBitPos,
                         bitMask,
                         maskVal);
        pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                      pIntrCfg[irqNum].intrMaskRegAddr,
                                      regData);
    }

    /* Stop Critical Section */
    Pmic_criticalSectionStop(pPmicCoreHandle);

    return pmicStatus;
}

/*!
 * \brief  Function to Mask/Unmask PMIC Interrupts except GPIO.
 */
static int32_t Pmic_maskIntr(Pmic_CoreHandle_t *pPmicCoreHandle,
                             const uint8_t      irqNum,
                             const bool         mask)
{
    int32_t pmicStatus       = PMIC_ST_SUCCESS;
    uint8_t irqId            = 0U;
    uint8_t maxVal           = 0U;
    Pmic_IntrCfg_t *pIntrCfg = NULL;

    pmicStatus = Pmic_get_intrCfg(pPmicCoreHandle, &pIntrCfg);

    if((PMIC_ST_SUCCESS == pmicStatus) && (PMIC_IRQ_ALL != irqNum))
    {
        if(PMIC_IRQ_INVALID_REGADDR == pIntrCfg[irqNum].intrMaskRegAddr)
        {
            pmicStatus = PMIC_ST_ERR_FAIL;
        }

        if(PMIC_ST_SUCCESS == pmicStatus)
        {
            pmicStatus = Pmic_irqMask(pPmicCoreHandle, irqNum, mask, pIntrCfg);
        }
    }

    if((PMIC_ST_SUCCESS == pmicStatus) && (PMIC_IRQ_ALL == irqNum))
    {
        pmicStatus = Pmic_getMaxVal(pPmicCoreHandle, &maxVal);
        if(PMIC_ST_SUCCESS == pmicStatus)
        {
            for(irqId = 0U; irqId < maxVal; irqId++)
            {
                if(PMIC_IRQ_INVALID_REGADDR == pIntrCfg[irqId].intrMaskRegAddr)
                {
                    continue;
                }

                pmicStatus = Pmic_irqMask(pPmicCoreHandle, irqId, mask, pIntrCfg);
            }
        }
    }

    return pmicStatus;
}

/*!
 * \brief  Function to get the PMIC_INT_TOP Register value.
 */
static int32_t Pmic_getIntrTopRegVal(Pmic_CoreHandle_t *pPmicCoreHandle,
                                     uint8_t           *regValue)
{
    int32_t  pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData     = 0U;

    /* Start Critical Section */
    Pmic_criticalSectionStart(pPmicCoreHandle);

    /* Read Top level Interrupt TOP register in the Hierarchy */
    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        PMIC_INT_TOP_REGADDR,
                                        &regData);

    /* Stop Critical Section */
    Pmic_criticalSectionStop (pPmicCoreHandle);
    (*regValue) = regData;

    return pmicStatus;
}

/*!
 * \brief  Function to get the L1 error registers.
 */
static void Pmic_irqGetL1Reg(const Pmic_CoreHandle_t *pPmicCoreHandle,
                             uint8_t                  regValue,
                             uint16_t                *l1RegAddr,
                             uint8_t                  count)
{
    (*l1RegAddr) = PMIC_INT_UNUSED_REGADDR;

    switch(regValue & (1U << count))
    {
        case PMIC_INT_TOP_BUCK_INT_MASK:
             (*l1RegAddr) = PMIC_INT_BUCK_REGADDR;
             break;

        case PMIC_INT_TOP_LDO_VMON_INT_MASK:
             if(PMIC_DEV_HERA_LP8764X  == pPmicCoreHandle->pmicDeviceType)
             {
                 (*l1RegAddr) = PMIC_INT_VMON_REGADDR;
             }

             if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
             {
                 (*l1RegAddr) = PMIC_INT_LDO_VMON_REGADDR;
             }

             break;

        case PMIC_INT_TOP_GPIO_INT_MASK:
             (*l1RegAddr) = PMIC_INT_GPIO_REGADDR;
             break;

        case PMIC_INT_TOP_STARTUP_INT_MASK:
             (*l1RegAddr) = PMIC_INT_STARTUP_REGADDR;
             break;

        case PMIC_INT_TOP_MISC_INT_MASK:
             (*l1RegAddr) = PMIC_INT_MISC_REGADDR;
             break;

        case PMIC_INT_TOP_MODERATE_ERR_INT_MASK:
             (*l1RegAddr) = PMIC_INT_MODERATE_ERR_REGADDR;
             break;

        case PMIC_INT_TOP_SEVERE_ERR_INT_MASK:
             (*l1RegAddr) = PMIC_INT_SEVERE_ERR_REGADDR;
             break;

        case PMIC_INT_TOP_FSM_ERR_INT_MASK:
             (*l1RegAddr) = PMIC_INT_FSM_ERR_REGADDR;
             break;

        default:
             break;
    }

    if(PMIC_INT_UNUSED_REGADDR == (*l1RegAddr))
    {
        (*l1RegAddr) = 0U;
    }
}

/*!
 * \brief  Function to decipher L2 Error.
 */
static int32_t Pmic_irqGetL2Error(Pmic_CoreHandle_t *pPmicCoreHandle,
                                  uint16_t           l1RegAddr,
                                  Pmic_IrqStatus_t  *pErrStat)
{
    int32_t pmicStatus  = PMIC_ST_SUCCESS;

    switch(pPmicCoreHandle->pmicDeviceType)
    {
        case PMIC_DEV_LEO_TPS6594X:
            pmicStatus = Pmic_tps6594x_irqGetL2Error(pPmicCoreHandle,
                                                     l1RegAddr,
                                                     pErrStat);
            break;

        case PMIC_DEV_HERA_LP8764X:
            pmicStatus = Pmic_lp8764x_irqGetL2Error(pPmicCoreHandle,
                                                    l1RegAddr,
                                                    pErrStat);
            break;

        default:
            pmicStatus = PMIC_ST_ERR_INV_DEVICE;
            break;
    }

    return pmicStatus;
}

/*!
 * \brief   Function to Extract Interrupts as per Hierarchy given in TRM and
 *          clear the bit in pErrStat.
 */
static int32_t Pmic_extractErrStatus(const Pmic_CoreHandle_t *pPmicCoreHandle,
                                     Pmic_IrqStatus_t        *pErrStat,
                                     uint8_t                 *pIrqNum)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t maxVal     = 0U;

    pmicStatus = Pmic_getMaxVal(pPmicCoreHandle, &maxVal);
    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        *pIrqNum = Pmic_intrBitExtract(pErrStat, maxVal);
        /* To clear the Error Bit position after extracting */
        Pmic_intrBitClear(pErrStat, pIrqNum);
    }

    else
    {
        pmicStatus = PMIC_ST_ERR_INV_DEVICE;
    }

    return pmicStatus;
}

/*
 * \brief   API to read Error status.
 *          This function does the following:
 *             1. This function gets the interrupt status by reading pmic
 *                IRQ register as per IRQ hierarchy defined in device TRM.
 *             2. Decipher error from top register to actual error code.
 *             3. Store the status of all Interrupts.
 *             4. Support clearing interrupts depends on clearIRQ flag.
 *
 * \param   pPmicCoreHandle   [IN]    PMIC Interface Handle.
 * \param   pErrStat          [OUT]   Pointer to store Error status.
 * \param   clearIRQ          [IN]    Flag to clear Interrupt status after
 *                                    deciphering the interrupt status.
 *                                    For valid values: \ref Pmic_IrqClearFlag.
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code.
 *          For valid values: \ref Pmic_ErrorCodes.
 */
int32_t  Pmic_irqGetErrStatus(Pmic_CoreHandle_t *pPmicCoreHandle,
                              Pmic_IrqStatus_t  *pErrStat,
                              const bool         clearIRQ)
{
    int32_t pmicStatus   = PMIC_ST_SUCCESS;
    uint8_t regValue     = 0U;
    uint16_t l1RegAddr   = 0U;
    uint8_t count        = 0U;
    uint8_t clearIRQStat = 0U;

    if(NULL == pPmicCoreHandle)
    {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    }

    if((PMIC_ST_SUCCESS == pmicStatus) && (NULL == pErrStat))
    {
        pmicStatus = PMIC_ST_ERR_NULL_PARAM;
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        /* Clearing all Error Status structure members */
        pErrStat->intStatus[0U] = 0U;
        pErrStat->intStatus[1U] = 0U;
        pErrStat->intStatus[2U] = 0U;
        pErrStat->intStatus[3U] = 0U;

        pmicStatus = Pmic_getIntrTopRegVal(pPmicCoreHandle, &regValue);

        if(PMIC_ST_SUCCESS == pmicStatus)
        {
            for(count = 7U;; count--)
            {
                l1RegAddr = 0U;
                Pmic_irqGetL1Reg(pPmicCoreHandle,
                                 regValue,
                                 &l1RegAddr,
                                 count);
                if(0U != l1RegAddr)
                {
                    pmicStatus = Pmic_irqGetL2Error(pPmicCoreHandle,
                                                    l1RegAddr,
                                                    pErrStat);
                }
                if((PMIC_ST_SUCCESS != pmicStatus) || (count == 0U))
                {
                    break;
                }
            }
            if(((bool)true) == clearIRQ)
            {
                clearIRQStat =1U;
            }

            if(PMIC_IRQ_CLEAR == clearIRQStat)
            {
                pmicStatus = Pmic_irqClrErrStatus(pPmicCoreHandle,
                                                  PMIC_IRQ_ALL);
            }
        }
    }

    return pmicStatus;
}

/*
 * \brief   API to clear Error status.
 *          This function does the following:
 *          1. This function clears the IRQ status in PMIC register for a given
 *             IRQ Number.
 *          2. Validates given IRQ Number and find the IRQ register that is 
 *             to be cleared.
 *          3. Expected to be called after an IRQ status is deciphered by
 *             Pmic_irqGetErrStatus().
 *
 * \param   pPmicCoreHandle   [IN]    PMIC Interface Handle.
 * \param   irqNum            [IN]    Interrupt number to clear the status.
 *                                    Valid values are:
 *                                    For TPS6594x LEO:
 *                                    \ref Pmic_tps6594x_IrqNum.
 *                                    For LP8764x HERA:
 *                                    \ref Pmic_lp8764x_IrqNum.
 *                                    For all: \ref Pmic_IrqNum.
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code.
 *          For valid values: \ref Pmic_ErrorCodes.
 */
int32_t Pmic_irqClrErrStatus(Pmic_CoreHandle_t *pPmicCoreHandle,
                             const uint8_t            irqNum)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    if(NULL == pPmicCoreHandle)
    {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        pmicStatus = Pmic_irqValidateIrqNum(pPmicCoreHandle, irqNum);
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        pmicStatus = Pmic_irqClearStatus(pPmicCoreHandle, irqNum);
    }

    return pmicStatus;
}

/*
 * \brief   API to mask/unmask interrupts.
 *          This function does the following:
 *          1. This function mask/unmask the given IRQ Number.
 *          2. Validates given IRQ Number and find the IRQ register that
 *             is to be masked/unmasked.
 *
 * \param   pPmicCoreHandle   [IN]    PMIC Interface Handle.
 * \param   irqNum            [IN]    Interrupt status to be cleared.
 *                                    Valid values are:
 *                                    For TPS6594x LEO:
 *                                    \ref Pmic_tps6594x_IrqNum.
 *                                    For LP8764x HERA:
 *                                    \ref Pmic_lp8764x_IrqNum.
 *                                    For all: \ref Pmic_IrqNum.
 * \param   mask              [IN]    Parameter to mask/unmask INTR.
 *                                    For valid values: \ref Pmic_IrqMaskFlag.
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code.
 *          For valid values: \ref Pmic_ErrorCodes.
 */
int32_t Pmic_irqMaskIntr(Pmic_CoreHandle_t *pPmicCoreHandle,
                         const uint8_t      irqNum,
                         const bool         mask)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    if(NULL == pPmicCoreHandle)
    {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        pmicStatus = Pmic_irqValidateIrqNum(pPmicCoreHandle, irqNum);
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        pmicStatus = Pmic_maskIntr(pPmicCoreHandle, irqNum, mask);
    }

    return pmicStatus;
}

/*
 * \brief   API to extract each Error status.
 *          This function is used to extract each Error status from pErrStat 
 *          as per the hierarchy given in the TRM. This function clears the 
 *          Error status after the status is extracted. This API is expected to
 *          be called after Pmic_irqGetErrStatus.
 *
 * \param   pPmicCoreHandle   [IN]    PMIC Interface Handle.
 * \param   pErrStat          [IN]    Pointer containing Error Status.
 * \param   pIrqNum           [OUT]   Pointer to store the IRQ Number extracted
 *                                    For TPS6594x LEO:
 *                                    \ref Pmic_tps6594x_IrqNum.
 *                                    For LP8764x HERA:
 *                                    \ref Pmic_lp8764x_IrqNum.
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code.
 *          For valid values \ref: Pmic_ErrorCodes.
 */
int32_t Pmic_getNextErrorStatus(const Pmic_CoreHandle_t *pPmicCoreHandle,
                                Pmic_IrqStatus_t        *pErrStat,
                                uint8_t                 *pIrqNum)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    if(NULL == pPmicCoreHandle)
    {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    }

    if((PMIC_ST_SUCCESS == pmicStatus) && (NULL == pErrStat))
    {
        pmicStatus = PMIC_ST_ERR_NULL_PARAM;
    }

    if((PMIC_ST_SUCCESS == pmicStatus) && (NULL == pIrqNum))
    {
        pmicStatus = PMIC_ST_ERR_NULL_PARAM;
    }

    if((PMIC_ST_SUCCESS == pmicStatus) && 
       ((pErrStat->intStatus[0U] == 0U) && (pErrStat->intStatus[1U] == 0U) &&
        (pErrStat->intStatus[2U] == 0U) && (pErrStat->intStatus[3U] == 0U)))
    {
        pmicStatus = PMIC_ST_ERR_INV_INT;
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        pmicStatus = Pmic_extractErrStatus(pPmicCoreHandle, pErrStat, pIrqNum);
    }

    return pmicStatus;
}

/*
 * \brief   API to mask/unmask GPIO interrupts.
 *          This function is used to Mask or Unmask GPIO Rise and Fall
 *          Interrupts based on the GPIO IRQ Number.
 *
 * \param   pPmicCoreHandle   [IN]    PMIC Interface Handle.
 * \param   irqGpioNum        [IN]    GPIO Interrupt to be masked/unmasked.
 *                                    Valid values:
 *                                    For TPS6594x LEO PMIC:
 *                                    \ref Pmic_tps6594x_IrqGpioNum.
 *                                    For LP8764x HERA PMIC:
 *                                    \ref Pmic_lp8764x_IrqGpioNum.
 *                                    For all: \ref Pmic_IrqGpioNum.
 * \param   mask              [IN]    Parameter to mask/unmask INTR.
 *                                    Valid values: \ref Pmic_IrqMaskFlag.
 * \param   gpioIntrType      [IN]    Parameter to mask GPIO RISE and FALL
 *                                    Interrupt.
 *                                    Valid values: \ref Pmic_IrqGpioIntrType.
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code.
 *          For valid values \ref Pmic_ErrorCodes.
 */
int32_t Pmic_irqGpioMaskIntr(Pmic_CoreHandle_t *pPmicCoreHandle,
                             const uint8_t      irqGpioNum,
                             const bool         mask,
                             const uint8_t      gpioIntrType)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    if(NULL == pPmicCoreHandle)
    {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (gpioIntrType > PMIC_IRQ_GPIO_RISE_FALL_INT_TYPE))
    {
        pmicStatus = PMIC_ST_ERR_INV_PARAM;
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (irqGpioNum > PMIC_IRQ_GPIO_ALL_INT_MASK_NUM))
    {
        pmicStatus = PMIC_ST_ERR_INV_PARAM;
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       ((irqGpioNum == PMIC_TPS6594X_IRQ_GPIO_11_INT_MASK_NUM) &&
        (PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)))
    {
        pmicStatus = PMIC_ST_ERR_INV_PARAM;
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        pmicStatus = Pmic_maskGpioIntr(pPmicCoreHandle,
                                       irqGpioNum,
                                       mask,
                                       gpioIntrType);
    }

    return pmicStatus;
}
