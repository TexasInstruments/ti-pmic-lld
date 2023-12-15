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
 * \file   pmic_irq_status.c
 *
 * \brief  This file contains APIs definitions for PMIC Interrupt IRQ Status
 *         Handler.
 *
 */

#include "pmic_irq.h"
#include "pmic_core_priv.h"
#include "pmic_irq_tps6594x_priv.h"
#include "pmic_irq_tps6522x_priv.h"
#include "pmic_irq_lp8764x_priv.h"
#include "pmic_irq_tps6594x.h"
#include "pmic_irq_tps6522x.h"
#include "pmic_irq_lp8764x.h"
#include "pmic_irq_priv.h"

void Pmic_intrBitSet(Pmic_IrqStatus_t *pErrStat, uint32_t pos)
{
    uint32_t intStatSize = 0U;

    // 4 x 8 = 32 -> size of intStatus[0] in bits
    intStatSize = sizeof(pErrStat->intStatus[0U]) << 3U;

    // interrupts 0 to 31 go to index 0, interrupts 32 to 63 go to index 1,
    // interrupts 64 to 95 go to index 2, interrupts 96 to 127 go to index 3
    // At an index, the interrupt is stored at its corresponding bit
    // (e.g., interrupt 49 will be stored at bit 17 at index 1)
    pErrStat->intStatus[pos / intStatSize] |= (((uint32_t)1U) << (pos % intStatSize));
}

/*!
 * \brief  Function to Clear the intStatus bit position.
 */
static void Pmic_intrBitClear(Pmic_IrqStatus_t *pErrStat, const uint8_t *pIrqNum)
{
    uint32_t intStatSize = 0U;

    // 4 x 8 = 32 -> size of intStatus[0] in bits
    intStatSize = sizeof(pErrStat->intStatus[0U]) << 3U;

    // interrupts 0 to 31 go to index 0, interrupts 32 to 63 go to index 1,
    // interrupts 64 to 95 go to index 2, interrupts 96 to 127 go to index 3
    // At an index, an interrupt's corresponding bit is cleared
    // (e.g., interrupt 49 will be stored at bit 17 at index 1)
    pErrStat->intStatus[(*pIrqNum) / intStatSize] &= ~(1U << ((*pIrqNum) % intStatSize));
}

/*!
 * \brief  Function to Extract the intStatus bit position.
 */
static uint8_t Pmic_intrBitExtract(const Pmic_IrqStatus_t *pErrStat, uint8_t maxVal)
{
    uint8_t  irqNum = 0U;
    uint32_t intStatSize = 0U;

    /* Size of intStatus in bits */
    intStatSize = sizeof(pErrStat->intStatus[0U]) << 3U;
    for (irqNum = 0U; irqNum < maxVal; irqNum++)
    {
        if (((pErrStat->intStatus[irqNum / intStatSize]) & (((uint32_t)1U) << (irqNum % intStatSize))) != 0U)
        {
            break;
        }
    }

    return irqNum;
}

/*!
 * \brief  Function to Check the device specific Max IrqNum.
 *
 *          Note: In this API, the default PMIC device is assumed as TPS6594x
 *                LEO PMIC. While adding support for New PMIC device, developer
 *                need to update the API functionality for New PMIC device
 *                accordingly.
 */
static int32_t Pmic_irqValidateIrqNum(const Pmic_CoreHandle_t *pPmicCoreHandle, const uint8_t irqNum)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t maxVal;

    switch (pPmicCoreHandle->pmicDeviceType)
    {
        case PMIC_DEV_HERA_LP8764X:
            if (PMIC_SILICON_REV_ID_PG_1_0 == pPmicCoreHandle->pmicDevSiliconRev)
            {
                /* SOFT REBOOT is not valid for PG 1.0*/
                maxVal = PMIC_LP8764X_IRQ_MAX_NUM_PG_1_0;
            }
            else
            {
                maxVal = PMIC_LP8764X_IRQ_MAX_NUM_PG_2_0;
            }

            break;
        case PMIC_DEV_BURTON_TPS6522X:
            maxVal = PMIC_TPS6522X_IRQ_MAX_NUM;

            break;
        default:
            /* Default case is valid only for TPS6594x LEO PMIC */
            if (PMIC_SILICON_REV_ID_PG_1_0 == pPmicCoreHandle->pmicDevSiliconRev)
            {
                /* SOFT REBOOT is not valid for PG 1.0*/
                maxVal = PMIC_TPS6594X_IRQ_MAX_NUM_PG_1_0;
            }
            else
            {
                maxVal = PMIC_TPS6594X_IRQ_MAX_NUM_PG_2_0;
            }
    }

    if ((irqNum > maxVal) && (irqNum != PMIC_IRQ_ALL))
    {
        pmicStatus = PMIC_ST_ERR_INV_PARAM;
    }

    return pmicStatus;
}

/*!
 * \brief  Function to Check the device specific Max IrqNum
 *
 *          Note: In this API, the default PMIC device is assumed as TPS6594x
 *                LEO PMIC. While adding support for New PMIC device, developer
 *                need to update the API functionality for New PMIC device
 *                accordingly.
 */
static int32_t Pmic_irqValidateIrqNumGetMaskIntrStatus(const Pmic_CoreHandle_t *pPmicCoreHandle, const uint8_t irqNum)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t maxVal;

    switch (pPmicCoreHandle->pmicDeviceType)
    {
        case PMIC_DEV_HERA_LP8764X:
            if (PMIC_SILICON_REV_ID_PG_1_0 == pPmicCoreHandle->pmicDevSiliconRev)
            {
                /* SOFT REBOOT is not valid for PG 1.0*/
                maxVal = PMIC_LP8764X_IRQ_MAX_NUM_PG_1_0;
            }
            else
            {
                maxVal = PMIC_LP8764X_IRQ_MAX_NUM_PG_2_0;
            }

            break;
        case PMIC_DEV_BURTON_TPS6522X:
            maxVal = PMIC_TPS6522X_IRQ_MAX_NUM;

            break;
        default:
            /* Default case is valid only for TPS6594x LEO PMIC */
            if (PMIC_SILICON_REV_ID_PG_1_0 == pPmicCoreHandle->pmicDevSiliconRev)
            {
                /* SOFT REBOOT is not valid for PG 1.0*/
                maxVal = PMIC_TPS6594X_IRQ_MAX_NUM_PG_1_0;
            }
            else
            {
                maxVal = PMIC_TPS6594X_IRQ_MAX_NUM_PG_2_0;
            }

            break;
    }

    if (irqNum > maxVal)
    {
        pmicStatus = PMIC_ST_ERR_INV_PARAM;
    }

    return pmicStatus;
}

/*!
 * \brief  Function to get the device specific Max IrqNum.
 *
 *          Note: In this API, the default PMIC device is assumed as TPS6594x
 *                LEO PMIC. While adding support for New PMIC device, developer
 *                need to update the API functionality for New PMIC device
 *                accordingly.
 */
static void Pmic_getMaxVal(const Pmic_CoreHandle_t *pPmicCoreHandle, uint8_t *maxVal)
{
    switch (pPmicCoreHandle->pmicDeviceType)
    {
        case PMIC_DEV_HERA_LP8764X:
            if (PMIC_SILICON_REV_ID_PG_1_0 == pPmicCoreHandle->pmicDevSiliconRev)
            {
                /* SOFT REBOOT is not valid for PG 1.0*/
                (*maxVal) = PMIC_LP8764X_IRQ_MAX_NUM_PG_1_0;
            }
            else
            {
                (*maxVal) = PMIC_LP8764X_IRQ_MAX_NUM_PG_2_0;
            }

            break;
        case PMIC_DEV_BURTON_TPS6522X:
            (*maxVal) = PMIC_TPS6522X_IRQ_MAX_NUM;

            break;
        default:
            /* Default case is valid only for TPS6594x LEO PMIC */
            if (PMIC_SILICON_REV_ID_PG_1_0 == pPmicCoreHandle->pmicDevSiliconRev)
            {
                /* SOFT REBOOT is not valid for PG 1.0*/
                (*maxVal) = PMIC_TPS6594X_IRQ_MAX_NUM_PG_1_0;
            }
            else
            {
                (*maxVal) = PMIC_TPS6594X_IRQ_MAX_NUM_PG_2_0;
            }

            break;
    }
}

/*!
 * \brief  Function to get the Device specific Interrupt Configuration
 *         registers.
 *
 *          Note: In this API, the default PMIC device is assumed as TPS6594x
 *                LEO PMIC. While adding support for New PMIC device, developer
 *                need to update the API functionality for New PMIC device
 *                accordingly.
 */
static void Pmic_get_intrCfg(const Pmic_CoreHandle_t *pPmicCoreHandle, Pmic_IntrCfg_t **pIntrCfg)
{
    switch (pPmicCoreHandle->pmicDeviceType)
    {
        case PMIC_DEV_HERA_LP8764X:
            pmic_get_lp8764x_intrCfg(pIntrCfg);
            break;
        case PMIC_DEV_BURTON_TPS6522X:
            pmic_get_tps6522x_intrCfg(pIntrCfg);
            break;
        default:
            /* Default case is valid only for TPS6594x LEO PMIC */
            pmic_get_tps6594x_intrCfg(pIntrCfg);
            break;
    }
}

/*!
 * \brief  Function to get the Device specific GPIO Interrupt configuration
 *         registers.
 *
 *          Note: In this API, the default PMIC device is assumed as TPS6594x
 *                LEO PMIC. While adding support for New PMIC device, developer
 *                need to update the API functionality for New PMIC device
 *                accordingly.
 */
static void Pmic_get_gpioIntrCfg(const Pmic_CoreHandle_t *pPmicCoreHandle, Pmic_GpioIntrTypeCfg_t **pGpioIntrCfg)
{
    switch (pPmicCoreHandle->pmicDeviceType)
    {
        case PMIC_DEV_HERA_LP8764X:
            pmic_get_lp8764x_intrGpioCfg(pGpioIntrCfg);
            break;
        case PMIC_DEV_BURTON_TPS6522X:
            pmic_get_tps6522x_intrGpioCfg(pGpioIntrCfg);
            break;
        default:
            pmic_get_tps6594x_intrGpioCfg(pGpioIntrCfg);
            break;
    }
}

/*!
 * \brief  Function to Mask/Unmask GPIO Interrupts.
 */
static int32_t Pmic_irqGpioMask(Pmic_CoreHandle_t *pPmicCoreHandle,
                                const uint8_t      irqGpioNum,
                                const bool         mask,
                                const uint8_t      gpioIntrType)
{
    int32_t                 pmicStatus = PMIC_ST_SUCCESS;
    uint8_t                 regData = 0U;
    Pmic_GpioIntrTypeCfg_t *pGpioIntrCfg = NULL;
    uint8_t                 bitMask = 0U;
    uint8_t                 maskVal = 0U;

    Pmic_get_gpioIntrCfg(pPmicCoreHandle, &pGpioIntrCfg);

    /* Start Critical Section */
    Pmic_criticalSectionStart(pPmicCoreHandle);

    if ((PMIC_IRQ_GPIO_RISE_INT_TYPE == gpioIntrType) || (PMIC_IRQ_GPIO_RISE_FALL_INT_TYPE == gpioIntrType))
    {
        pmicStatus =
            Pmic_commIntf_recvByte(pPmicCoreHandle, pGpioIntrCfg[irqGpioNum].gpioRiseIntrMaskRegAddr, &regData);

        if (PMIC_ST_SUCCESS == pmicStatus)
        {
            if (((bool)true) == mask)
            {
                maskVal = 1U;
            }
            bitMask = (PMIC_IRQ_MASK_CLR_BITFIELD << pGpioIntrCfg[irqGpioNum].gpioRiseMaskBitPos);
            Pmic_setBitField(&regData, pGpioIntrCfg[irqGpioNum].gpioRiseMaskBitPos, bitMask, maskVal);
            pmicStatus =
                Pmic_commIntf_sendByte(pPmicCoreHandle, pGpioIntrCfg[irqGpioNum].gpioRiseIntrMaskRegAddr, regData);
        }
    }

    if ((PMIC_IRQ_GPIO_FALL_INT_TYPE == gpioIntrType) || (PMIC_IRQ_GPIO_RISE_FALL_INT_TYPE == gpioIntrType))
    {
        pmicStatus =
            Pmic_commIntf_recvByte(pPmicCoreHandle, pGpioIntrCfg[irqGpioNum].gpioFallIntrMaskRegAddr, &regData);

        if (PMIC_ST_SUCCESS == pmicStatus)
        {
            if (((bool)true) == mask)
            {
                maskVal = 1U;
            }
            bitMask = (PMIC_IRQ_MASK_CLR_BITFIELD << pGpioIntrCfg[irqGpioNum].gpioFallMaskBitPos);
            Pmic_setBitField(&regData, pGpioIntrCfg[irqGpioNum].gpioFallMaskBitPos, bitMask, maskVal);
            pmicStatus =
                Pmic_commIntf_sendByte(pPmicCoreHandle, pGpioIntrCfg[irqGpioNum].gpioFallIntrMaskRegAddr, regData);
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
    uint8_t irqGpioId = 0U;

    if (PMIC_IRQ_GPIO_ALL_INT_MASK_NUM != irqGpioNum)
    {
        pmicStatus = Pmic_irqGpioMask(pPmicCoreHandle, irqGpioNum, mask, gpioIntrType);
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) && (PMIC_IRQ_GPIO_ALL_INT_MASK_NUM == irqGpioNum))
    {
        for (irqGpioId = 0U; irqGpioId < (PMIC_IRQ_GPIO_ALL_INT_MASK_NUM - 1U); irqGpioId++)
        {
            pmicStatus = Pmic_irqGpioMask(pPmicCoreHandle, irqGpioId, mask, gpioIntrType);
        }
    }

    return pmicStatus;
}

/*!
 * \brief  Function to clear IRQ status.
 */
static int32_t Pmic_irqClear(Pmic_CoreHandle_t *pPmicCoreHandle, const uint8_t irqNum)
{
    int32_t         pmicStatus = PMIC_ST_SUCCESS;
    uint8_t         regData = 0U;
    Pmic_IntrCfg_t *pIntrCfg = NULL;
    uint8_t         bitMask = 0U;

    Pmic_get_intrCfg(pPmicCoreHandle, &pIntrCfg);

    /* Start Critical Section */
    Pmic_criticalSectionStart(pPmicCoreHandle);

    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, pIntrCfg[irqNum].intrClrRegAddr, &regData);

    if (PMIC_ST_SUCCESS == pmicStatus)
    {
        bitMask = (PMIC_IRQ_MASK_CLR_BITFIELD << pIntrCfg[irqNum].intrClrBitPos);
        Pmic_setBitField(&regData, pIntrCfg[irqNum].intrClrBitPos, bitMask, PMIC_IRQ_CLEAR);
        pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle, pIntrCfg[irqNum].intrClrRegAddr, regData);
    }

    /* Stop Critical Section */
    Pmic_criticalSectionStop(pPmicCoreHandle);

    return pmicStatus;
}

/*!
 * \brief  Function to Clear Interrupt Status register.
 */
static int32_t Pmic_irqClearStatus(Pmic_CoreHandle_t *pPmicCoreHandle, const uint8_t irqNum)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t irqId = 0U;
    uint8_t maxVal = 0U;

    if (PMIC_IRQ_ALL != irqNum)
    {
        pmicStatus = Pmic_irqClear(pPmicCoreHandle, irqNum);
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) && (PMIC_IRQ_ALL == irqNum))
    {
        Pmic_getMaxVal(pPmicCoreHandle, &maxVal);

        for (irqId = 0U; irqId < maxVal; irqId++)
        {
            pmicStatus = Pmic_irqClear(pPmicCoreHandle, irqId);
        }
    }

    return pmicStatus;
}

/*!
 * \brief  Function to Mask/Unmask Interrupts.
 */
static int32_t
Pmic_irqMask(Pmic_CoreHandle_t *pPmicCoreHandle, const uint8_t irqNum, const bool mask, const Pmic_IntrCfg_t *pIntrCfg)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    uint8_t bitMask = 0U;
    uint8_t maskVal = 0U;

    /* Start Critical Section */
    Pmic_criticalSectionStart(pPmicCoreHandle);

    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, pIntrCfg[irqNum].intrMaskRegAddr, &regData);

    if (PMIC_ST_SUCCESS == pmicStatus)
    {
        if (((bool)true) == mask)
        {
            maskVal = 1U;
        }
        bitMask = (PMIC_IRQ_MASK_CLR_BITFIELD << pIntrCfg[irqNum].intrMaskBitPos);
        Pmic_setBitField(&regData, pIntrCfg[irqNum].intrMaskBitPos, bitMask, maskVal);
        pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle, pIntrCfg[irqNum].intrMaskRegAddr, regData);
    }

    /* Stop Critical Section */
    Pmic_criticalSectionStop(pPmicCoreHandle);

    return pmicStatus;
}

/*!
 * \brief  Function to Mask/Unmask PMIC Interrupts except GPIO.
 */
static int32_t Pmic_maskIntr(Pmic_CoreHandle_t *pPmicCoreHandle, const uint8_t irqNum, const bool mask)
{
    int32_t         pmicStatus = PMIC_ST_SUCCESS;
    uint8_t         irqId = 0U;
    uint8_t         maxVal = 0U;
    Pmic_IntrCfg_t *pIntrCfg = NULL;

    Pmic_get_intrCfg(pPmicCoreHandle, &pIntrCfg);

    if (PMIC_IRQ_ALL != irqNum)
    {
        if (PMIC_IRQ_INVALID_REGADDR == pIntrCfg[irqNum].intrMaskRegAddr)
        {
            pmicStatus = PMIC_ST_ERR_FAIL;
        }

        if (PMIC_ST_SUCCESS == pmicStatus)
        {
            pmicStatus = Pmic_irqMask(pPmicCoreHandle, irqNum, mask, pIntrCfg);
        }
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) && (PMIC_IRQ_ALL == irqNum))
    {
        Pmic_getMaxVal(pPmicCoreHandle, &maxVal);

        for (irqId = 0U; irqId < maxVal; irqId++)
        {
            if (PMIC_IRQ_INVALID_REGADDR == pIntrCfg[irqId].intrMaskRegAddr)
            {
                continue;
            }

            pmicStatus = Pmic_irqMask(pPmicCoreHandle, irqId, mask, pIntrCfg);
        }
    }

    return pmicStatus;
}

/*!
 * \brief  Function to get the PMIC_INT_TOP Register value.
 */
static int32_t Pmic_getIntrTopRegVal(Pmic_CoreHandle_t *pPmicCoreHandle, uint8_t *regValue)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    /* Start Critical Section */
    Pmic_criticalSectionStart(pPmicCoreHandle);

    /* Read Top level Interrupt TOP register in the Hierarchy */
    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_INT_TOP_REGADDR, &regData);

    /* Stop Critical Section */
    Pmic_criticalSectionStop(pPmicCoreHandle);
    (*regValue) = regData;

    return pmicStatus;
}

/*!
 * \brief  Function to get the L1 error registers for INT_MISC,
 *         INT_MODERATE_ERR, INT_SEVERE_ERR, INT_FSM_ERR
 *
 */
static void Pmic_irqGetMiscModerateSevereFsmErr(uint8_t regValue, uint16_t *l1RegAddr, uint8_t count)
{
    switch (regValue & (1U << count))
    {
        case PMIC_INT_TOP_MISC_INT_MASK:
            (*l1RegAddr) = PMIC_INT_MISC_REGADDR;
            break;

        case PMIC_INT_TOP_MODERATE_ERR_INT_MASK:
            (*l1RegAddr) = PMIC_INT_MODERATE_ERR_REGADDR;
            break;

        case PMIC_INT_TOP_SEVERE_ERR_INT_MASK:
            (*l1RegAddr) = PMIC_INT_SEVERE_ERR_REGADDR;
            break;

        default:
            (*l1RegAddr) = PMIC_INT_FSM_ERR_REGADDR;
            break;
    }
}

/*!
 * \brief  Function to get the L1 error registers.
 */
static void
Pmic_irqGetL1Reg(const Pmic_CoreHandle_t *pPmicCoreHandle, uint8_t regValue, uint16_t *l1RegAddr, uint8_t count)
{
    (*l1RegAddr) = PMIC_INT_UNUSED_REGADDR;

    switch (regValue & (1U << count))
    {
        case PMIC_INT_TOP_BUCK_INT_MASK:
            (*l1RegAddr) = PMIC_INT_BUCK_REGADDR;
            break;

        case PMIC_INT_TOP_LDO_VMON_INT_MASK:
            if (PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
            {
                (*l1RegAddr) = PMIC_INT_VMON_REGADDR;
            }
            else if ((PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType) ||
                     (PMIC_DEV_BURTON_TPS6522X == pPmicCoreHandle->pmicDeviceType))
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
        case PMIC_INT_TOP_MODERATE_ERR_INT_MASK:
        case PMIC_INT_TOP_SEVERE_ERR_INT_MASK:
        case PMIC_INT_TOP_FSM_ERR_INT_MASK:
            Pmic_irqGetMiscModerateSevereFsmErr(regValue, l1RegAddr, count);
            break;

        default:
            break;
    }

    if (PMIC_INT_UNUSED_REGADDR == (*l1RegAddr))
    {
        (*l1RegAddr) = 0U;
    }
}

/*!
 * \brief  Function to decipher L2 Error.
 *
 *          Note: In this API, the default PMIC device is assumed as TPS6594x
 *                LEO PMIC. While adding support for New PMIC device, developer
 *                need to update the API functionality for New PMIC device
 *                accordingly.
 */
static int32_t Pmic_irqGetL2Error(Pmic_CoreHandle_t *pPmicCoreHandle, uint16_t l1RegAddr, Pmic_IrqStatus_t *pErrStat)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    switch (pPmicCoreHandle->pmicDeviceType)
    {
        case PMIC_DEV_HERA_LP8764X:
            pmicStatus = Pmic_lp8764x_irqGetL2Error(pPmicCoreHandle, l1RegAddr, pErrStat);
            break;
        case PMIC_DEV_BURTON_TPS6522X:
            pmicStatus = Pmic_tps6522x_irqGetL2Error(pPmicCoreHandle, l1RegAddr, pErrStat);
            break;
        default:
            /* Default case is valid only for TPS6594x LEO PMIC */
            pmicStatus = Pmic_tps6594x_irqGetL2Error(pPmicCoreHandle, l1RegAddr, pErrStat);
            break;
    }

    return pmicStatus;
}

/*!
 * \brief   Function to Extract Interrupts as per Hierarchy given in TRM and
 *          clear the bit in pErrStat.
 */
static void
Pmic_extractErrStatus(const Pmic_CoreHandle_t *pPmicCoreHandle, Pmic_IrqStatus_t *pErrStat, uint8_t *pIrqNum)
{
    uint8_t maxVal = 0U;

    Pmic_getMaxVal(pPmicCoreHandle, &maxVal);

    *pIrqNum = Pmic_intrBitExtract(pErrStat, maxVal);
    /* To clear the Error Bit position after extracting */
    Pmic_intrBitClear(pErrStat, pIrqNum);
}

int32_t Pmic_irqGetErrStatus(Pmic_CoreHandle_t *pPmicCoreHandle, Pmic_IrqStatus_t *pErrStat, const bool clearIRQ)
{
    int32_t  pmicStatus = PMIC_ST_SUCCESS;
    uint8_t  regValue = 0U;
    uint16_t l1RegAddr = 0U;
    uint8_t  count = 0U;
    uint8_t  clearIRQStat = 0U;

    if (NULL == pPmicCoreHandle)
    {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) && (NULL == pErrStat))
    {
        pmicStatus = PMIC_ST_ERR_NULL_PARAM;
    }

    if (PMIC_ST_SUCCESS == pmicStatus)
    {
        // Clearing all Error Status structure members
        pErrStat->intStatus[0U] = 0U;
        pErrStat->intStatus[1U] = 0U;
        pErrStat->intStatus[2U] = 0U;
        pErrStat->intStatus[3U] = 0U;

        // Read value of INT_TOP
        pmicStatus = Pmic_getIntrTopRegVal(pPmicCoreHandle, &regValue);

        if (PMIC_ST_SUCCESS == pmicStatus)
        {
            for (count = 7U;; count--)
            {
                l1RegAddr = 0U;

                // If a bit field within INT_TOP is non-zero, get the l1RegAddr of the bit field
                Pmic_irqGetL1Reg(pPmicCoreHandle, regValue, &l1RegAddr, count);

                if (0U != l1RegAddr)
                {
                    // Find and record all errors pertaining to the non-zero bitfield found in INT_TOP
                    pmicStatus = Pmic_irqGetL2Error(pPmicCoreHandle, l1RegAddr, pErrStat);
                }
                if ((PMIC_ST_SUCCESS != pmicStatus) || (count == 0U))
                {
                    break;
                }
            }
            if (((bool)true) == clearIRQ)
            {
                clearIRQStat = 1U;
            }

            if (PMIC_IRQ_CLEAR == clearIRQStat)
            {
                pmicStatus = Pmic_irqClrErrStatus(pPmicCoreHandle, PMIC_IRQ_ALL);
            }
        }
    }

    return pmicStatus;
}

int32_t Pmic_irqClrErrStatus(Pmic_CoreHandle_t *pPmicCoreHandle, const uint8_t irqNum)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    if (NULL == pPmicCoreHandle)
    {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    }

    if (PMIC_ST_SUCCESS == pmicStatus)
    {
        pmicStatus = Pmic_irqValidateIrqNum(pPmicCoreHandle, irqNum);
    }

    if (PMIC_ST_SUCCESS == pmicStatus)
    {
        pmicStatus = Pmic_irqClearStatus(pPmicCoreHandle, irqNum);
    }

    return pmicStatus;
}

int32_t Pmic_irqMaskIntr(Pmic_CoreHandle_t *pPmicCoreHandle, const uint8_t irqNum, const bool mask)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    if (NULL == pPmicCoreHandle)
    {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    }

    if (PMIC_ST_SUCCESS == pmicStatus)
    {
        pmicStatus = Pmic_irqValidateIrqNum(pPmicCoreHandle, irqNum);
    }

    if (PMIC_ST_SUCCESS == pmicStatus)
    {
        pmicStatus = Pmic_maskIntr(pPmicCoreHandle, irqNum, mask);
    }

    return pmicStatus;
}

int32_t Pmic_getNextErrorStatus(const Pmic_CoreHandle_t *pPmicCoreHandle, Pmic_IrqStatus_t *pErrStat, uint8_t *pIrqNum)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    if (NULL == pPmicCoreHandle)
    {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) && (NULL == pErrStat))
    {
        pmicStatus = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) && (NULL == pIrqNum))
    {
        pmicStatus = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) && ((pErrStat->intStatus[0U] == 0U) && (pErrStat->intStatus[1U] == 0U) &&
                                            (pErrStat->intStatus[2U] == 0U) && (pErrStat->intStatus[3U] == 0U)))
    {
        pmicStatus = PMIC_ST_ERR_INV_INT;
    }

    if (PMIC_ST_SUCCESS == pmicStatus)
    {
        Pmic_extractErrStatus(pPmicCoreHandle, pErrStat, pIrqNum);
    }

    return pmicStatus;
}

int32_t Pmic_irqGpioMaskIntr(Pmic_CoreHandle_t *pPmicCoreHandle,
                             const uint8_t      irqGpioNum,
                             const bool         mask,
                             const uint8_t      gpioIntrType)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    if (NULL == pPmicCoreHandle)
    {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) && (gpioIntrType > PMIC_IRQ_GPIO_RISE_FALL_INT_TYPE))
    {
        pmicStatus = PMIC_ST_ERR_INV_PARAM;
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) && (irqGpioNum > PMIC_IRQ_GPIO_ALL_INT_MASK_NUM))
    {
        pmicStatus = PMIC_ST_ERR_INV_PARAM;
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) && ((irqGpioNum == PMIC_TPS6594X_IRQ_GPIO_11_INT_MASK_NUM) &&
                                            (PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)))
    {
        pmicStatus = PMIC_ST_ERR_INV_PARAM;
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) && ((irqGpioNum > PMIC_TPS6522X_IRQ_GPIO_6_INT_MASK_NUM) &&
                                            (PMIC_DEV_BURTON_TPS6522X == pPmicCoreHandle->pmicDeviceType)))
    {
        pmicStatus = PMIC_ST_ERR_INV_PARAM;
    }

    if (PMIC_ST_SUCCESS == pmicStatus)
    {
        pmicStatus = Pmic_maskGpioIntr(pPmicCoreHandle, irqGpioNum, mask, gpioIntrType);
    }

    return pmicStatus;
}

/*!
 * \brief  Function to get status of the Interrupts is masked or not.
 */
static int32_t Pmic_getIrqMaskStatus(Pmic_CoreHandle_t    *pPmicCoreHandle,
                                     const uint8_t         irqNum,
                                     bool                 *pMaskStatus,
                                     const Pmic_IntrCfg_t *pIntrCfg)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    uint8_t bitMask = 0U;

    /* Start Critical Section */
    Pmic_criticalSectionStart(pPmicCoreHandle);

    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, pIntrCfg[irqNum].intrMaskRegAddr, &regData);

    /* Stop Critical Section */
    Pmic_criticalSectionStop(pPmicCoreHandle);

    if (PMIC_ST_SUCCESS == pmicStatus)
    {
        bitMask = (PMIC_IRQ_MASK_CLR_BITFIELD << pIntrCfg[irqNum].intrMaskBitPos);
        *pMaskStatus = PMIC_IRQ_UNMASK;

        if ((Pmic_getBitField(regData, pIntrCfg[irqNum].intrMaskBitPos, bitMask)) == PMIC_IRQ_MASK_VAL_1)
        {
            *pMaskStatus = PMIC_IRQ_MASK;
        }
    }

    return pmicStatus;
}

/*!
 * \brief  Function to get status of the Interrupts is masked or not except
 *         GPIO Interrupts.
 */
static int32_t Pmic_getMaskIntrStatus(Pmic_CoreHandle_t *pPmicCoreHandle, const uint8_t irqNum, bool *pMaskStatus)
{
    int32_t         pmicStatus = PMIC_ST_SUCCESS;
    Pmic_IntrCfg_t *pIntrCfg = NULL;

    Pmic_get_intrCfg(pPmicCoreHandle, &pIntrCfg);

    if (PMIC_IRQ_INVALID_REGADDR == pIntrCfg[irqNum].intrMaskRegAddr)
    {
        pmicStatus = PMIC_ST_ERR_FAIL;
    }

    if (PMIC_ST_SUCCESS == pmicStatus)
    {
        pmicStatus = Pmic_getIrqMaskStatus(pPmicCoreHandle, irqNum, pMaskStatus, pIntrCfg);
    }

    return pmicStatus;
}

int32_t Pmic_irqGetMaskIntrStatus(Pmic_CoreHandle_t *pPmicCoreHandle, const uint8_t irqNum, bool *pMaskStatus)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    if (NULL == pPmicCoreHandle)
    {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) && (NULL == pMaskStatus))
    {
        pmicStatus = PMIC_ST_ERR_NULL_PARAM;
    }

    if (PMIC_ST_SUCCESS == pmicStatus)
    {
        pmicStatus = Pmic_irqValidateIrqNumGetMaskIntrStatus(pPmicCoreHandle, irqNum);
    }

    if (PMIC_ST_SUCCESS == pmicStatus)
    {
        pmicStatus = Pmic_getMaskIntrStatus(pPmicCoreHandle, irqNum, pMaskStatus);
    }

    return pmicStatus;
}

/*!
 * \brief  Function to read the status of GPIO Interrupts is masked or not
 */
static int32_t Pmic_getIrqGpioMaskStatus(Pmic_CoreHandle_t *pPmicCoreHandle,
                                         const uint8_t      irqGpioNum,
                                         const uint8_t      gpioIntrType,
                                         bool              *pRiseIntrMaskStat,
                                         bool              *pFallIntrMaskStat)
{
    int32_t                 pmicStatus = PMIC_ST_SUCCESS;
    uint8_t                 regData = 0U;
    Pmic_GpioIntrTypeCfg_t *pGpioIntrCfg = NULL;
    uint8_t                 bitMask = 0U;

    Pmic_get_gpioIntrCfg(pPmicCoreHandle, &pGpioIntrCfg);

    /* Start Critical Section */
    Pmic_criticalSectionStart(pPmicCoreHandle);

    if ((PMIC_IRQ_GPIO_RISE_INT_TYPE == gpioIntrType) || (PMIC_IRQ_GPIO_RISE_FALL_INT_TYPE == gpioIntrType))
    {
        pmicStatus =
            Pmic_commIntf_recvByte(pPmicCoreHandle, pGpioIntrCfg[irqGpioNum].gpioRiseIntrMaskRegAddr, &regData);

        if (PMIC_ST_SUCCESS == pmicStatus)
        {
            bitMask = (PMIC_IRQ_MASK_CLR_BITFIELD << pGpioIntrCfg[irqGpioNum].gpioRiseMaskBitPos);
            *pRiseIntrMaskStat = PMIC_IRQ_UNMASK;

            if ((Pmic_getBitField(regData, pGpioIntrCfg[irqGpioNum].gpioRiseMaskBitPos, bitMask)) ==
                PMIC_IRQ_MASK_VAL_1)
            {
                *pRiseIntrMaskStat = PMIC_IRQ_MASK;
            }
        }
    }

    if ((PMIC_IRQ_GPIO_FALL_INT_TYPE == gpioIntrType) || (PMIC_IRQ_GPIO_RISE_FALL_INT_TYPE == gpioIntrType))
    {
        pmicStatus =
            Pmic_commIntf_recvByte(pPmicCoreHandle, pGpioIntrCfg[irqGpioNum].gpioFallIntrMaskRegAddr, &regData);

        if (PMIC_ST_SUCCESS == pmicStatus)
        {
            bitMask = (PMIC_IRQ_MASK_CLR_BITFIELD << pGpioIntrCfg[irqGpioNum].gpioFallMaskBitPos);
            *pFallIntrMaskStat = PMIC_IRQ_UNMASK;

            if ((Pmic_getBitField(regData, pGpioIntrCfg[irqGpioNum].gpioFallMaskBitPos, bitMask)) ==
                PMIC_IRQ_MASK_VAL_1)
            {
                *pFallIntrMaskStat = PMIC_IRQ_MASK;
            }
        }
    }

    /* Stop Critical Section */
    Pmic_criticalSectionStop(pPmicCoreHandle);

    return pmicStatus;
}

/*!
 * \brief  Function to read the status of GPIO Interrupts is masked or not
 */
static int32_t Pmic_getMaskGpioIntrStatus(Pmic_CoreHandle_t *pPmicCoreHandle,
                                          const uint8_t      irqGpioNum,
                                          const uint8_t      gpioIntrType,
                                          bool              *pRiseIntrMaskStat,
                                          bool              *pFallIntrMaskStat)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    pmicStatus =
        Pmic_getIrqGpioMaskStatus(pPmicCoreHandle, irqGpioNum, gpioIntrType, pRiseIntrMaskStat, pFallIntrMaskStat);

    return pmicStatus;
}

int32_t Pmic_irqGetGpioMaskIntr(Pmic_CoreHandle_t *pPmicCoreHandle,
                                const uint8_t      irqGpioNum,
                                const uint8_t      gpioIntrType,
                                bool              *pRiseIntrMaskStat,
                                bool              *pFallIntrMaskStat)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    if (NULL == pPmicCoreHandle)
    {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) && ((NULL == pRiseIntrMaskStat) || (NULL == pFallIntrMaskStat)))
    {
        pmicStatus = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) && (gpioIntrType > PMIC_IRQ_GPIO_RISE_FALL_INT_TYPE))
    {
        pmicStatus = PMIC_ST_ERR_INV_PARAM;
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) && (irqGpioNum >= PMIC_IRQ_GPIO_ALL_INT_MASK_NUM))
    {
        pmicStatus = PMIC_ST_ERR_INV_PARAM;
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) && ((irqGpioNum == PMIC_TPS6594X_IRQ_GPIO_11_INT_MASK_NUM) &&
                                            (PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)))
    {
        pmicStatus = PMIC_ST_ERR_INV_PARAM;
    }

    if (PMIC_ST_SUCCESS == pmicStatus)
    {
        pmicStatus =
            Pmic_getMaskGpioIntrStatus(pPmicCoreHandle, irqGpioNum, gpioIntrType, pRiseIntrMaskStat, pFallIntrMaskStat);
    }

    return pmicStatus;
}
