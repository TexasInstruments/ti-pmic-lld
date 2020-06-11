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
 * /file   pmic_irq_status.c
 *
 * /brief  This file contains APIs definitions for PMIC Interrupt IRQ Status
 *         Handler.
 *
 */

#include <pmic_irq.h>
#include <pmic_irq_priv.h>

/*!
 * \brief   Error Bit Mask Status Validation
 * \param   errBitStatus - Error Mask value extracted from
 *          errStat from the application
 * \param   mask  - Expected Error mask value
 */
static int32_t Pmic_errorBitMaskCheck(uint32_t errBitStatus,
                                      uint8_t  mask)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    if((errBitStatus & mask) != errBitStatus)
    {
        pmicStatus = PMIC_ST_ERR_INV_INT;
    }

    return pmicStatus;
}

/*!
 * \brief   Function to get Error bit status
 *
 * \param   regValue   [IN]    Register Value
 * \param   offset     [IN]    Register offset
 * \param   maxBits    [IN]    Maximum bits
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code.
 *          Where retVal is of enum type #Pmic_Status_t
 */
static int32_t  Pmic_getErrBitStatus(uint8_t  regValue,
                                     uint8_t  offset,
                                     uint32_t maxBits)
{
    uint32_t bitIdx = PMIC_INT_INVALID_REGADDDR;

    for(bitIdx = offset; bitIdx < maxBits; bitIdx++)
    {
        if((regValue & (1U << bitIdx)))
        {
            bitIdx = (regValue & (1U << bitIdx));
            break;
        }
    }

    return bitIdx;
}

/*!
 * \brief   This function is used for clearing the interrupt.
 */
static int32_t Pmic_irqClear(Pmic_CoreHandle_t *pPmicCoreHandle,
                             const uint32_t     errStat)
{
    int32_t  pmicStatus      = PMIC_ST_SUCCESS;
    uint32_t irqL1RegAddr    = 0U;
    uint32_t irqL2RegAddr    = 0U;
    uint32_t errBitStatus    = 0U;
    uint32_t regAddr         = 0U;
    uint8_t  regValue        = 0U;

    if(0x00U == errStat)
    {
        pmicStatus = PMIC_ST_ERR_INV_INT;
    }

    /* Extract Level 1, Level 2 register offsets and error code */
    irqL1RegAddr   = PMIC_IRQID_L1REG(errStat);
    irqL2RegAddr   = PMIC_IRQID_L2REG(errStat);
    errBitStatus   = PMIC_IRQID_BITMASK(errStat);

    /*
     * Find the register to be updated to clear IRQ
     * If the IRQ status is directly from L1 register, use L1;
     * else, L2 register contains the IRQ status bits, use L2;
     */
    if(0U != irqL2RegAddr)
    {
        regAddr = irqL2RegAddr;
    }
    else
    {
        regAddr = irqL1RegAddr;
    }

    if(0U != regAddr)
    {
        /* Start Critical Section */
        Pmic_criticalSectionStart(pPmicCoreHandle);

        /* Read, update and write back register */
        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                            regAddr,
                                            &regValue);
        if(PMIC_ST_SUCCESS == pmicStatus)
        {
            regValue  |= errBitStatus;
            pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                                regAddr,
                                                regValue);
        }

        /* Stop Critical Section*/
        Pmic_criticalSectionStop(pPmicCoreHandle);
    }

    return pmicStatus;
}

/*!
 * \brief   This function is validate the interrupt to be cleared.
 */
static int32_t Pmic_irqValidate(Pmic_CoreHandle_t *pPmicCoreHandle,
                                const uint32_t     errStat)
{
    int32_t  pmicStatus   = PMIC_ST_SUCCESS;
    uint32_t irqL1RegAddr = 0U;
    uint32_t irqL2RegAddr = 0U;
    uint32_t errBitStatus = 0U;

    /* Extract Level 1, Level 2 register offsets and error code */
    irqL1RegAddr   = PMIC_IRQID_L1REG  (errStat);
    irqL2RegAddr   = PMIC_IRQID_L2REG  (errStat);
    errBitStatus   = PMIC_IRQID_BITMASK(errStat);

    /* Validate Level 1 and 2 register offset values and the error code */
    switch(irqL1RegAddr)
    {
        case PMIC_INT_BUCK_REGADDDR:
            switch(irqL2RegAddr)
            {
                case PMIC_INT_BUCK1_2_REGADDDR:
                case PMIC_INT_BUCK3_4_REGADDDR:
                    /* Any of the bits 0-7 are valid, so no error */
                    break;
                case PMIC_INT_BUCK5_REGADDDR:
                    if(PMIC_DEV_HERA_LP8764X  ==
                       pPmicCoreHandle->pmicDeviceType)
                    {
                        /* BUCK5 not supported by HERA */
                        pmicStatus = PMIC_ST_ERR_INV_INT;
                    }
                    break;
                default:
                    pmicStatus = PMIC_ST_ERR_INV_INT;
                    break;
            }
            break;
        case PMIC_INT_LDO_VMON_REGADDDR:
            if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
            {
                /* LDO-VMON not supported by HERA */
                pmicStatus = PMIC_ST_ERR_INV_INT;
            }

            switch(irqL2RegAddr)
            {
                case PMIC_INT_LDO1_2_REGADDDR:
                    pmicStatus = Pmic_errorBitMaskCheck(errBitStatus,
                                                        PMIC_INT_LDO_1_3_MASK);
                    break;
                case PMIC_INT_LDO3_4_REGADDDR:
                    pmicStatus = Pmic_errorBitMaskCheck(errBitStatus,
                                                        PMIC_INT_LDO_2_4_MASK);
                    break;
                case PMIC_INT_VMON_REGADDDR:
                    pmicStatus = Pmic_errorBitMaskCheck(errBitStatus,
                                                       PMIC_INT_LDO_VMON_MASK);
                    break;
                default:
                    pmicStatus = PMIC_ST_ERR_INV_INT;
                    break;
            }
            break;
        case PMIC_INT_GPIO_REGADDDR:
            switch(irqL2RegAddr)
            {
                case PMIC_INT_GPIO1_8_REGADDDR:
                    /* Any of the bits 0-7 are valid, so no error */
                    break;
                case PMIC_INT_UNUSED_REGADDDR:
                    if(PMIC_DEV_LEO_TPS6594X      ==
                       pPmicCoreHandle->pmicDeviceType)
                    {
                        pmicStatus = Pmic_errorBitMaskCheck(errBitStatus,
                                                       PMIC_INT_GPIO9_11_MASK);
                    }
                    else
                    {
                        pmicStatus = Pmic_errorBitMaskCheck(errBitStatus,
                                                          PMIC_INT_GPIO9_MASK);
                    }
                    break;
                default:
                    pmicStatus = PMIC_ST_ERR_INV_INT;
                    break;
            }
            break;
        case PMIC_INT_STARTUP_REGADDDR:
            switch(irqL2RegAddr)
            {
                case PMIC_INT_RTC_STATUS_REGADDDR:
                    if(PMIC_DEV_LEO_TPS6594X      ==
                       pPmicCoreHandle->pmicDeviceType)
                    {
                        pmicStatus = Pmic_errorBitMaskCheck(errBitStatus,
                                                            PMIC_INT_RTC_MASK);
                    }
                    else
                    {
                        pmicStatus = PMIC_ST_ERR_INV_INT;
                    }

                    break;
                case PMIC_INT_UNUSED_REGADDDR:
                    pmicStatus = Pmic_errorBitMaskCheck(errBitStatus,
                                                        PMIC_INT_STARTUP_MASK);
                    break;
                default:
                    pmicStatus = PMIC_ST_ERR_INV_INT;
                    break;
            }
            break;
        case PMIC_INT_MISC_WARN_REGADDDR:
            switch(irqL2RegAddr)
            {
                case PMIC_INT_UNUSED_REGADDDR:
                    pmicStatus = Pmic_errorBitMaskCheck(errBitStatus,
                                                      PMIC_INT_MISC_WARN_MASK);
                    break;
                default:
                    pmicStatus = PMIC_ST_ERR_INV_INT;
                    break;
            }
            break;
        case PMIC_INT_MODERATE_REGADDDR:
            switch(irqL2RegAddr)
            {
                case PMIC_INT_UNUSED_REGADDDR:
                    /* Any of the bits 0-7 are valid, so no error */
                    break;
                default:
                    pmicStatus = PMIC_ST_ERR_INV_INT;
                    break;
            }
            break;
        case PMIC_INT_SEVERE_REGADDDR:
            switch(irqL2RegAddr)
            {
                case PMIC_INT_UNUSED_REGADDDR:
                    pmicStatus = Pmic_errorBitMaskCheck(errBitStatus,
                                                        PMIC_INT_SEVERE_MASK);
                    break;
                default:
                    pmicStatus = PMIC_ST_ERR_INV_INT;
                    break;
            }
            break;
        case PMIC_INT_FSM_REGADDDR:
            switch(irqL2RegAddr)
            {
                case PMIC_INT_UNUSED_REGADDDR:
                    pmicStatus = Pmic_errorBitMaskCheck(errBitStatus,
                                                        PMIC_INT_FSM_ERR_MASK);
                    break;
                case PMIC_INT_COMM_ERR_REGADDDR:
                    pmicStatus = Pmic_errorBitMaskCheck(errBitStatus,
                                                       PMIC_INT_COMM_ERR_MASK);
                    break;
                case PMIC_INT_RDBACK_ERR_REGADDDR:
                    pmicStatus = Pmic_errorBitMaskCheck(errBitStatus,
                                                       PMIC_INT_RDBK_ERR_MASK);
                    break;
                case PMIC_INT_ESM_REGADDDR:
                    if(PMIC_DEV_HERA_LP8764X ==
                       pPmicCoreHandle->pmicDeviceType)
                    {
                        pmicStatus = Pmic_errorBitMaskCheck(errBitStatus,
                                                        PMIC_INT_ESM_MCU_MASK);
                    }
                    else
                    {
                        pmicStatus = Pmic_errorBitMaskCheck(errBitStatus,
                                                            PMIC_INT_ESM_MASK);
                    }

                    break;
                case PMIC_INT_WD_ERR_STATUS_REGADDDR:
                    pmicStatus = Pmic_errorBitMaskCheck(errBitStatus,
                                                        PMIC_INT_WD_ERR_MASK);
                    break;
                default:
                    pmicStatus = PMIC_ST_ERR_INV_INT;
                    break;
            }
            break;
        default:
            /* Not a valid Level 1 Register! */
            pmicStatus = PMIC_ST_ERR_INV_INT;
            break;
    }

    return pmicStatus;
}

/*!
 * \brief   This function is get the level-1 error register address.
 */
static int32_t Pmic_irqGetL1Error(Pmic_CoreHandle_t *pPmicCoreHandle,
                                  uint8_t            regValue,
                                  uint32_t          *regAddr)
{
    int32_t  pmicStatus = PMIC_ST_SUCCESS;
    int32_t  tmp        = 0U;

    /*
     * Start from highest priority error to get
     * Level 1 Register offset to get more error info
     */
    (*regAddr) = PMIC_INT_UNUSED_REGADDDR;

    for(tmp = 7U;
       (tmp >= 0) && (PMIC_INT_UNUSED_REGADDDR == (*regAddr));
        tmp--)
    {
        switch(regValue & (1U << tmp))
        {
            case PMIC_INT_TOP_BUCK_MASK:
                (*regAddr) = PMIC_INT_BUCK_REGADDDR;
                break;

            case PMIC_INT_TOP_LDO_VMON_MASK:
                if(PMIC_DEV_HERA_LP8764X  ==
                   pPmicCoreHandle->pmicDeviceType)
                {
                    /*LDO-VMON not supported by HERA*/
                    break;
                }

                (*regAddr) = PMIC_INT_LDO_VMON_REGADDDR;
                break;

            case PMIC_INT_TOP_GPIO_MASK:
                (*regAddr) = PMIC_INT_GPIO_REGADDDR;
                break;

            case PMIC_INT_TOP_STARTUP_MASK:
                (*regAddr) = PMIC_INT_STARTUP_REGADDDR;
                break;

            case PMIC_INT_TOP_MISC_WARN_MASK:
                (*regAddr) = PMIC_INT_MISC_WARN_REGADDDR;
                break;

            case PMIC_INT_TOP_MODERATE_MASK:
                (*regAddr) = PMIC_INT_MODERATE_REGADDDR;
                break;

            case PMIC_INT_TOP_SEVERE_MASK:
                (*regAddr) = PMIC_INT_SEVERE_REGADDDR;
                break;

            case PMIC_INT_TOP_FSM_MASK:
                (*regAddr) = PMIC_INT_FSM_REGADDDR;
                break;

            default:
                break;
        }
    }

    /*
     * If some invalid value is found for register offset,
     * return error
     */
    if(PMIC_INT_UNUSED_REGADDDR == (*regAddr))
    {
        (*regAddr) = 0U;
    }

    return pmicStatus;
}

/*!
 * \brief   This function is get the level-2 error register address.
 */
static int32_t Pmic_irqGetL2Error(Pmic_CoreHandle_t *pPmicCoreHandle,
                                  uint8_t            regValue,
                                  uint32_t          *regAddr,
                                  uint32_t          *irqL1RegAddr,
                                  uint32_t          *irqL2RegAddr,
                                  uint32_t          *errBitStatus)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    /*
     * Now that we have the Level 1 Register value,
     * save Level 1 register address to generate final error code
     */
    (*irqL1RegAddr) = (*regAddr);
    (*regAddr)      = PMIC_INT_INVALID_REGADDDR;

    /* Get the relevant Level 2 register, if applicable */
    switch(*irqL1RegAddr)
    {
        case PMIC_INT_BUCK_REGADDDR:
            if(regValue & PMIC_INT_BUCK_BUCK1_2_INT_MASK)
            {
                (*regAddr) = PMIC_INT_BUCK1_2_REGADDDR;
            }
            else if(regValue & PMIC_INT_BUCK_BUCK3_4_INT_MASK)
            {
                (*regAddr) = PMIC_INT_BUCK3_4_REGADDDR;
            }
            else if((PMIC_DEV_LEO_TPS6594X ==
                     pPmicCoreHandle->pmicDeviceType) &&
                    (regValue & PMIC_INT_BUCK_BUCK5_INT_MASK))
            {
                /* BUCK5 not supported by HERA, only LEO supports this */
                (*regAddr) = PMIC_INT_BUCK5_REGADDDR;
            }

            break;
        case PMIC_INT_LDO_VMON_REGADDDR:
            if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
            {
                /* LDO-VMON not supported by HERA */
                break;
            }

            if(regValue & PMIC_INT_LDO_VMON_LDO1_2_MASK)
            {
                (*regAddr) = PMIC_INT_LDO1_2_REGADDDR;
            }
            else if(regValue & PMIC_INT_LDO_VMON_LDO3_4_MASK)
            {
                (*regAddr) = PMIC_INT_LDO3_4_REGADDDR;
            }
            else if(regValue & PMIC_INT_LDO_VMON_VCCA_INT_MASK)
            {
                (*regAddr) = PMIC_INT_VMON_REGADDDR;
            }

            break;
        case PMIC_INT_GPIO_REGADDDR:
            if(regValue & PMIC_INT_GPIO_GPIO1_8_INT_MASK)
            {
                (*regAddr) = PMIC_INT_GPIO1_8_REGADDDR;
            }
            else if(regValue & PMIC_INT_GPIO_GPIO9_INT_MASK)
            {
                (*regAddr)      = (*irqL1RegAddr);
                (*irqL2RegAddr) = PMIC_INT_UNUSED_REGADDDR;
                (*errBitStatus) = PMIC_INT_GPIO_GPIO9_INT_MASK;
            }
            else if(regValue & PMIC_INT_GPIO_GPIO10_INT_MASK)
            {
                (*regAddr)      = (*irqL1RegAddr);
                (*irqL2RegAddr) = PMIC_INT_UNUSED_REGADDDR;
                (*errBitStatus) = PMIC_INT_GPIO_GPIO10_INT_MASK;
            }
            else if(regValue & PMIC_INT_GPIO_GPIO11_INT_MASK)
            {
                (*regAddr)      = (*irqL1RegAddr);
                (*irqL2RegAddr) = PMIC_INT_UNUSED_REGADDDR;
                (*errBitStatus) = PMIC_INT_GPIO_GPIO11_INT_MASK;
            }

            break;
        case PMIC_INT_STARTUP_REGADDDR:
            if(regValue & PMIC_INT_STARTUP_NPWRON_START_INT_MASK)
            {
                (*regAddr)      = (*irqL1RegAddr);
                (*irqL2RegAddr) = PMIC_INT_UNUSED_REGADDDR;
                (*errBitStatus) = PMIC_INT_STARTUP_NPWRON_START_INT_MASK;
            }
            else if(regValue & PMIC_INT_STARTUP_ENABLE_INT_MASK)
            {
                (*regAddr)      = (*irqL1RegAddr);
                (*irqL2RegAddr) = PMIC_INT_UNUSED_REGADDDR;
                (*errBitStatus) = PMIC_INT_STARTUP_ENABLE_INT_MASK;
            }
            else if(regValue & PMIC_INT_STARTUP_FSD_INT_MASK)
            {
                (*regAddr)      = (*irqL1RegAddr);
                (*irqL2RegAddr) = PMIC_INT_UNUSED_REGADDDR;
                (*errBitStatus) = PMIC_INT_STARTUP_FSD_INT_MASK;
            }
            else if(regValue & PMIC_INT_STARTUP_RTC_INT_MASK)
            {
                (*regAddr) = PMIC_INT_RTC_STATUS_REGADDDR;
            }

            break;
        case PMIC_INT_MISC_WARN_REGADDDR:
            if(regValue & PMIC_INT_MISC_BIST_PASS_INT_MASK)
            {
                (*regAddr)      = (*irqL1RegAddr);
                (*irqL2RegAddr) = PMIC_INT_UNUSED_REGADDDR;
                (*errBitStatus) = PMIC_INT_MISC_BIST_PASS_INT_MASK;
            }
            else if(regValue & PMIC_INT_MISC_EXT_CLK_INT_MASK)
            {
                (*regAddr)      = (*irqL1RegAddr);
                (*irqL2RegAddr) = PMIC_INT_UNUSED_REGADDDR;
                (*errBitStatus) = PMIC_INT_MISC_EXT_CLK_INT_MASK;
            }
            else if(regValue & PMIC_INT_MISC_TWARN_INT_MASK)
            {
                (*regAddr)      = (*irqL1RegAddr);
                (*irqL2RegAddr) = PMIC_INT_UNUSED_REGADDDR;
                (*errBitStatus) = PMIC_INT_MISC_TWARN_INT_MASK;
            }

            break;
        case PMIC_INT_MODERATE_REGADDDR:
            /* Bits 0-7 are valid in this register */
            /* Bits 0-7 are final in Hierarchy for this error */
            (*regAddr)      = (*irqL1RegAddr);
            (*irqL2RegAddr) = PMIC_INT_UNUSED_REGADDDR;
            (*errBitStatus) = Pmic_getErrBitStatus(regValue, 0U, 8U);
            break;
        case PMIC_INT_SEVERE_REGADDDR:
            if(regValue & (PMIC_INT_SEVERE_TSD_IMM_INT_MASK))
            {
                (*regAddr)      = (*irqL1RegAddr);
                (*irqL2RegAddr) = PMIC_INT_UNUSED_REGADDDR;
                (*errBitStatus) = PMIC_INT_SEVERE_TSD_IMM_INT_MASK;
            }
            else if(regValue & PMIC_INT_SEVERE_VCCA_OVP_INT_MASK)
            {
                (*regAddr)      = (*irqL1RegAddr);
                (*irqL2RegAddr) = PMIC_INT_UNUSED_REGADDDR;
                (*errBitStatus) = PMIC_INT_SEVERE_VCCA_OVP_INT_MASK;
            }
            else if(regValue & PMIC_INT_SEVERE_PFSM_ERR_INT_MASK)
            {
                (*regAddr)      = (*irqL1RegAddr);
                (*irqL2RegAddr) = PMIC_INT_UNUSED_REGADDDR;
                (*errBitStatus) = PMIC_INT_SEVERE_PFSM_ERR_INT_MASK;
            }

            break;
        case PMIC_INT_FSM_REGADDDR:
            if(regValue & (PMIC_INT_FSM_IMM_SHUTDOWN_INT_MASK))
            {
                (*regAddr)      = (*irqL1RegAddr);
                (*irqL2RegAddr) = PMIC_INT_UNUSED_REGADDDR;
                (*errBitStatus) = PMIC_INT_FSM_IMM_SHUTDOWN_INT_MASK;
            }
            else if(regValue & (PMIC_INT_FSM_ORD_SHUTDOWN_INT_MASK))
            {
                (*regAddr)      = (*irqL1RegAddr);
                (*irqL2RegAddr) = PMIC_INT_UNUSED_REGADDDR;
                (*errBitStatus) = PMIC_INT_FSM_ORD_SHUTDOWN_INT_MASK;
            }
            else if(regValue & (PMIC_INT_FSM_MCU_PWR_ERR_INT_MASK))
            {
                (*regAddr)      = (*irqL1RegAddr);
                (*irqL2RegAddr) = PMIC_INT_UNUSED_REGADDDR;
                (*errBitStatus) = PMIC_INT_FSM_MCU_PWR_ERR_INT_MASK;
            }
            else if(regValue & (PMIC_INT_FSM_SOC_PWR_ERR_INT_MASK))
            {
                (*regAddr)      = (*irqL1RegAddr);
                (*irqL2RegAddr) = PMIC_INT_UNUSED_REGADDDR;
                (*errBitStatus) = PMIC_INT_FSM_SOC_PWR_ERR_INT_MASK;
            }
            else if(regValue & PMIC_INT_FSM_COMM_ERR_INT_MASK)
            {
                (*regAddr) = PMIC_INT_COMM_ERR_REGADDDR;
            }
            else if(regValue & PMIC_INT_FSM_READBACK_ERR_INT_MASK)
            {
                (*regAddr) = PMIC_INT_RDBACK_ERR_REGADDDR;
            }
            else if(regValue & PMIC_INT_FSM_ESM_INT_MASK)
            {
                (*regAddr) = PMIC_INT_ESM_REGADDDR;
            }
            else if(regValue & PMIC_INT_FSM_WD_INT_MASK)
            {
                (*regAddr) = PMIC_INT_WD_ERR_STATUS_REGADDDR;
            }

            break;
        default:
            break;
    }

    /*
     * If some invalid value is found for register
     * offset, return error
     */
    if(PMIC_INT_INVALID_REGADDDR == (*regAddr))
    {
        pmicStatus = PMIC_ST_ERR_INV_INT;
    }

    return pmicStatus;
}

/*!
 * \brief   This function is used to generate actual error bitmask.
 */
static int32_t Pmic_irqGetError(Pmic_CoreHandle_t *pPmicCoreHandle,
                                uint32_t          *errBitStatus,
                                uint32_t          *irqL2RegAddr,
                                uint8_t           *regValue)
{
    int32_t  pmicStatus  = PMIC_ST_SUCCESS;

    (*errBitStatus) = PMIC_INT_INVALID_REGADDDR;
    /* Form the actual error bitmask from Level 2 register value */
    switch((*irqL2RegAddr))
    {
        case PMIC_INT_BUCK1_2_REGADDDR:
        case PMIC_INT_BUCK3_4_REGADDDR:
            if((*regValue) & PMIC_INT_BUCK_1_3_MASK)
            {
                (*errBitStatus) = Pmic_getErrBitStatus((*regValue), 0U, 4U);
            }
            else if(((*regValue) & PMIC_INT_BUCK_2_4_MASK))
            {
                (*errBitStatus) = Pmic_getErrBitStatus((*regValue), 4U, 8U);
            }

            break;
        case PMIC_INT_BUCK5_REGADDDR:
            if(PMIC_DEV_HERA_LP8764X  ==
               pPmicCoreHandle->pmicDeviceType)
            {
                /* BUCK5 not supported by HERA */
                break;
            }

            if((*regValue) & PMIC_INT_BUCK_5_MASK)
            {
                (*errBitStatus) = Pmic_getErrBitStatus((*regValue), 0U, 4U);
            }

            break;
        case PMIC_INT_LDO1_2_REGADDDR:
        case PMIC_INT_LDO3_4_REGADDDR:
            if(PMIC_DEV_HERA_LP8764X  ==
               pPmicCoreHandle->pmicDeviceType)
            {
                /* LDO-VMON not supported by HERA */
                break;
            }

            if((*regValue) & PMIC_INT_LDO_1_3_MASK)
            {
                (*errBitStatus) = Pmic_getErrBitStatus((*regValue), 0U, 4U);
            }
            else if((*regValue) & PMIC_INT_LDO_2_4_MASK)
            {
                (*errBitStatus) = Pmic_getErrBitStatus((*regValue), 4U, 8U);
            }

            break;
        case PMIC_INT_VMON_REGADDDR:
            if(PMIC_DEV_HERA_LP8764X  ==
               pPmicCoreHandle->pmicDeviceType)
            {
                /* LDO-VMON not supported by HERA */
                break;
            }

            if(((*regValue) & PMIC_INT_LDO_VMON_MASK) ==
                (*regValue))
            {
                (*errBitStatus) = Pmic_getErrBitStatus((*regValue), 0U, 2U);
            }

            break;
        case PMIC_INT_GPIO1_8_REGADDDR:
            if((*regValue) & PMIC_INT_GPIO1_8_MASK)
            {
                (*errBitStatus) = Pmic_getErrBitStatus((*regValue), 0U, 8U);
            }

            break;
        case PMIC_INT_RTC_STATUS_REGADDDR:
            if((*regValue) &
                PMIC_INT_RTC_STATUS_TIMER_MASK)
            {
                (*errBitStatus) = PMIC_INT_RTC_STATUS_TIMER_MASK;
            }
            else if((*regValue) &
                     PMIC_INT_RTC_STATUS_ALARM_MASK)
            {
                (*errBitStatus) = PMIC_INT_RTC_STATUS_ALARM_MASK;
            }
            else if((*regValue) &
                     PMIC_INT_RTC_STATUS_POWER_UP_MASK)
            {
                (*errBitStatus) =
                             PMIC_INT_RTC_STATUS_POWER_UP_MASK;
            }

            break;
        case PMIC_INT_COMM_ERR_REGADDDR:
            if((*regValue) & PMIC_INT_COMM_ERR_MASK)
            {
                (*errBitStatus) = Pmic_getErrBitStatus((*regValue), 0U, 8U);
            }

            break;
        case PMIC_INT_RDBACK_ERR_REGADDDR:
            if((*regValue) & PMIC_INT_RDBACK_ERR_EN_DRV_MASK)
            {
                (*errBitStatus) = PMIC_INT_RDBACK_ERR_EN_DRV_MASK;
            }
            else if((*regValue) &
                             PMIC_INT_RDBACK_ERR_NRST_SOC_MASK)
            {
                (*errBitStatus) =
                             PMIC_INT_RDBACK_ERR_NRST_SOC_MASK;
            }

            break;
        case PMIC_INT_ESM_REGADDDR:
            if(((*regValue) & PMIC_INT_ESM_MCU_MASK))
            {
                (*errBitStatus) = Pmic_getErrBitStatus((*regValue), 3U, 3U);
            }
            else if((pPmicCoreHandle->pmicDeviceType ==
                     PMIC_DEV_LEO_TPS6594X) &&
                    ((*regValue) & PMIC_INT_ESM_SOC_MASK))
            {
                /* ESM_SOC not supported by HERA */
                (*errBitStatus) = Pmic_getErrBitStatus((*regValue), 0U, 3U);
            }

            break;
        case PMIC_INT_WD_ERR_STATUS_REGADDDR:
            if(((*regValue) & PMIC_INT_WD_ERR_MASK))
            {
                (*errBitStatus) = Pmic_getErrBitStatus((*regValue), 0U, 8U);
            }
            break;
        default:
            break;
    }

   if(PMIC_INT_INVALID_REGADDDR == (*errBitStatus))
   {
       pmicStatus = PMIC_ST_ERR_INV_INT;
   }

    return pmicStatus;
}

/*!
 * \brief   This function is used to mask Interrupt.
 */
static int32_t Pmic_maskIntr(Pmic_CoreHandle_t *pPmicCoreHandle,
                             uint16_t           interruptMask)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData    = 0U;
    uint8_t regAddr    = 0U;
    uint8_t regMask    = 0U;

    regMask = interruptMask >> 8U;
    regAddr = (regAddr | interruptMask);
    Pmic_criticalSectionStart(pPmicCoreHandle);

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        /* Masking Interrupt */
        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                            regAddr,
                                            &regData);
        if(PMIC_ST_SUCCESS == pmicStatus)
        {
            regData = regData | regMask;

            pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                                regAddr,
                                                regData);
        }
    }

    Pmic_criticalSectionStop(pPmicCoreHandle);

    return pmicStatus;
}

/*!
 * \brief   This function is used to un-mask Interrupt.
 */
static int32_t Pmic_unMaskIntr(Pmic_CoreHandle_t *pPmicCoreHandle,
                               uint16_t           interruptMask)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData    = 0U;
    uint8_t regAddr    = 0U;
    uint8_t regMask    = 0U;

    regMask = interruptMask >> 8U;
    regAddr = regAddr | interruptMask;

    Pmic_criticalSectionStart(pPmicCoreHandle);

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        /* Un-Masking Interrupt */
        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                            regAddr,
                                            &regData);
        if(PMIC_ST_SUCCESS == pmicStatus)
        {
            regData = regData & (~regMask);

            pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle,
                                                regAddr,
                                                regData);
        }
    }

    Pmic_criticalSectionStop(pPmicCoreHandle);

    return pmicStatus;
}

/*!
 * \brief   PMIC function to read Error status
 *          This function does the following:
 *             1. This function gets the interrupt status by reading pmic
 *                IRQ register as per IRQ hierarchy defined in device TRM.
 *             2. Decipher error from top register to actual error code.
 *             3. Support clearing interrupt using clearIRQ flag as required.
 *             4. Works with the valid PMIC instance else does not do any
 *                operation.
 *
 * \param   pPmicCoreHandle   [IN]    PMIC Interface Handle.
 * \param   pErrStat          [OUT]   Variable to hold error interrupt ID
 * \param   clearIRQ          [IN]    Variable to control whether to clear the
 *                                    IRQ or not.
 *                                    Valid values: \ref Pmic_IrqClearFlag
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code.
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t  Pmic_irqGetErrStatus(Pmic_CoreHandle_t *pPmicCoreHandle,
                              uint32_t          *pErrStat,
                              bool               clearIRQ)
{
    int32_t   pmicStatus    = PMIC_ST_SUCCESS;
    uint8_t   regValue      = 0U;
    uint32_t  regAddr       = 0U;
    uint32_t  irqL1RegAddr  = 0U;
    uint32_t  irqL2RegAddr  = 0U;
    uint32_t  errBitStatus  = 0U;

    if(NULL == pPmicCoreHandle)
    {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (NULL == pErrStat))
    {
        pmicStatus = PMIC_ST_ERR_NULL_PARAM;
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        /* Start Critical Section */
        Pmic_criticalSectionStart(pPmicCoreHandle);
        /* Read Top level Interrupt TOP register in the Hierarchy */
        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                            PMIC_INT_TOP_REGADDDR,
                                            &regValue);
        /* Stop Critical Section */
        Pmic_criticalSectionStop (pPmicCoreHandle);
        if(PMIC_ST_SUCCESS == pmicStatus)
        {
            pmicStatus = Pmic_irqGetL1Error(pPmicCoreHandle,
                                            regValue,
                                            &regAddr);
        }

        if(PMIC_ST_SUCCESS == pmicStatus)
        {
            /* Start Critical Section */
            Pmic_criticalSectionStart(pPmicCoreHandle);
            /* Read from Level 1 Register to know more error info */
            pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                                regAddr,
                                                &regValue);
            /* Stop Critical Section */
            Pmic_criticalSectionStop(pPmicCoreHandle);
        }

        if(PMIC_ST_SUCCESS == pmicStatus)
        {
            pmicStatus = Pmic_irqGetL2Error(pPmicCoreHandle, regValue,
                                            &regAddr,        &irqL1RegAddr,
                                            &irqL2RegAddr,   &errBitStatus);
        }

        if((PMIC_ST_SUCCESS == pmicStatus) &&
           (0U != irqL1RegAddr) &&
           (irqL1RegAddr != regAddr))
        {
            /* Save the L2 register value now */
            irqL2RegAddr = regAddr;

            if(PMIC_INT_UNUSED_REGADDDR != irqL2RegAddr)
            {
                /* Start Critical Section */
                Pmic_criticalSectionStart(pPmicCoreHandle);
                /* Read from Level 2 Register to know more error info */
                pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                                    regAddr,
                                                    &regValue);
                /* Stop Critical Section */
                Pmic_criticalSectionStop(pPmicCoreHandle);
                if(PMIC_ST_SUCCESS == pmicStatus)
                {
                    pmicStatus = Pmic_irqGetError(pPmicCoreHandle,
                                                  &errBitStatus,
                                                  &irqL2RegAddr,
                                                  &regValue);
                }
            }
            else
            {
                /*
                 * L2 register is invalid, so previous
                 * regValue is the Error bitmask
                 */
                errBitStatus = regValue;
            }
        }

        if(PMIC_INT_UNUSED_REGADDDR == irqL2RegAddr)
        {
            irqL2RegAddr = 0U;
        }

        if(PMIC_ST_SUCCESS == pmicStatus)
        {
            /*
             * We have proper values for Level 1, Level 2
             *  and actual error bitmask,
             * so we can form the error interrupt ID
             */
            (*pErrStat) = PMIC_IRQID(irqL1RegAddr,
                                     irqL2RegAddr,
                                     errBitStatus);
            /* If application wants to clear IRQ immediately, do it now */
            if((1U == clearIRQ) && (0U != (*pErrStat)))
            {
                pmicStatus  = Pmic_irqClear(pPmicCoreHandle, *pErrStat);
                if(PMIC_ST_SUCCESS != pmicStatus)
                {
                    pmicStatus = PMIC_ST_ERR_CLEAR_INT_FAILED;
                }
            }
        }
    }

    return pmicStatus;
}

 /*!
 * \brief: PMIC function to clear Error status
 *         This function does the following:
 *          1. This function clears the IRQ bits in PMIC register for a given
 *             error code.
 *          2. Validates error code given by application and find the IRQ
 *             register that is to be updated.
 *          3. Expected to be called after an error code is generated by
 *             Pmic_irqGetErrStatus().
 *          4. Works with the valid PMIC instance else does not do any
 *             operation
 *
 * \param   pPmicCoreHandle   [IN]    PMIC Interface Handle.
 * \param   errStat           [IN]    Error status
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code.
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t  Pmic_irqClrErrStatus(Pmic_CoreHandle_t      *pPmicCoreHandle,
                              const uint32_t          errStat)
{
    int32_t   pmicStatus = PMIC_ST_SUCCESS;

   /*
    * errBitStatus Validation
    * appm - mask extracted from errStat given by application
    * expm - expected mask value
    */

    if(NULL == pPmicCoreHandle)
    {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        pmicStatus = Pmic_irqValidate(pPmicCoreHandle, errStat);
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        pmicStatus = Pmic_irqClear(pPmicCoreHandle, errStat);
    }

    return pmicStatus;
}

 /*!
 * \brief: PMIC function mask/unmask interrupts
 *
 * \param   pPmicCoreHandle   [IN]    PMIC Interface Handle.
 * \param   interrupt         [IN]    Interrupt details
 *                                    Valid values: \ref Pmic_IrqInterrupt
 * \param   mask              [IN]    Parameter to mask/unmask INTR
 *                                    Valid values: \ref Pmic_IrqMaskFlag
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code.
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_irqMaskIntr(Pmic_CoreHandle_t *pPmicCoreHandle,
                         uint16_t           interruptMask,
                         bool               mask)
{
    int32_t  pmicStatus  = PMIC_ST_SUCCESS;
    /* Flag to define Critical section started or not */

    if(NULL == pPmicCoreHandle)
    {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (false == pPmicCoreHandle->pPmic_SubSysInfo->rtcEnable))
    {
        pmicStatus = PMIC_ST_ERR_INV_DEVICE;
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
        {
            if(PMIC_IRQ_MASK == mask)
            {
                pmicStatus = Pmic_maskIntr(pPmicCoreHandle, interruptMask);
            }

            if(PMIC_IRQ_UNMASK == mask)
            {
                pmicStatus = Pmic_unMaskIntr(pPmicCoreHandle, interruptMask);

            }
        }

    return pmicStatus;
}
