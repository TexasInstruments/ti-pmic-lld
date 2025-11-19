/******************************************************************************
 * Copyright (c) 2025 Texas Instruments Incorporated - http://www.ti.com
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
 *    distribution and/or other materials provided with the
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
 * @file pmic_irq.c
 *
 * @brief PMIC LLD IRQ module source file for TPS6522x-Q1.
 */
#include <stdint.h>

#include "pmic.h"
#include "pmic_irq.h"
#include "pmic_io.h"
#include "regmap/irq.h"

#define PMIC_INVALID_REGADDR ((uint8_t)0xFFU)

typedef struct Pmic_IrqInfo_s
{
    uint8_t statRegAddr;
    uint8_t maskRegAddr;
    uint8_t bitShift;
} Pmic_IrqInfo_t;

/**
 * @brief IRQ information table for TPS6522x-Q1 Burton
 * Maps IRQ numbers (0-48) to their status/mask register addresses and bit positions
 */
static const Pmic_IrqInfo_t pmicIRQs[PMIC_IRQ_INT_MAX + 1U] =
{
    // WD_ERR_STATUS register (non-standard location at 0x408)
    [PMIC_IRQ_WD_RST_NMI]             = {WD_ERR_STATUS_REGADDR, PMIC_INVALID_REGADDR, WD_RST_INT_SHIFT},
    [PMIC_IRQ_WD_FAIL_NMI]            = {WD_ERR_STATUS_REGADDR, PMIC_INVALID_REGADDR, WD_FAIL_INT_SHIFT},
    [PMIC_IRQ_WD_LONGWIN_TIMEOUT_NMI] = {WD_ERR_STATUS_REGADDR, PMIC_INVALID_REGADDR, WD_LONGWIN_TIMEOUT_INT_SHIFT},

    // INT_ESM register
    [PMIC_IRQ_ESM_MCU_RST_INT]  = {INT_ESM_REGADDR, MASK_ESM_REGADDR, ESM_MCU_RST_INT_SHIFT},
    [PMIC_IRQ_ESM_MCU_FAIL_INT] = {INT_ESM_REGADDR, MASK_ESM_REGADDR, ESM_MCU_FAIL_INT_SHIFT},
    [PMIC_IRQ_ESM_MCU_PIN_INT]  = {INT_ESM_REGADDR, MASK_ESM_REGADDR, ESM_MCU_PIN_INT_SHIFT},

    // INT_FSM_ERR register
    [PMIC_IRQ_I2C2_ERR_INT]      = {INT_FSM_ERR_REGADDR, MASK_FSM_ERR_REGADDR, I2C2_ERR_INT_SHIFT},
    [PMIC_IRQ_COMM_ERR_INT]      = {INT_FSM_ERR_REGADDR, MASK_FSM_ERR_REGADDR, COMM_ERR_INT_SHIFT},
    [PMIC_IRQ_SOC_PWR_ERR_INT]   = {INT_FSM_ERR_REGADDR, MASK_FSM_ERR_REGADDR, SOC_PWR_ERR_INT_SHIFT},
    [PMIC_IRQ_MCU_PWR_ERR_INT]   = {INT_FSM_ERR_REGADDR, MASK_FSM_ERR_REGADDR, MCU_PWR_ERR_INT_SHIFT},
    [PMIC_IRQ_ORD_SHUTDOWN_INT]  = {INT_FSM_ERR_REGADDR, MASK_FSM_ERR_REGADDR, ORD_SHUTDOWN_INT_SHIFT},
    [PMIC_IRQ_IMM_SHUTOWN_INT]   = {INT_FSM_ERR_REGADDR, MASK_FSM_ERR_REGADDR, IMM_SHUTDOWN_INT_SHIFT},

    // INT_SEVERE_ERR register
    [PMIC_IRQ_BG_XMON_INT]  = {INT_SEVERE_ERR_REGADDR, PMIC_INVALID_REGADDR, BG_XMON_INT_SHIFT},
    [PMIC_IRQ_PFSM_ERR_INT] = {INT_SEVERE_ERR_REGADDR, PMIC_INVALID_REGADDR, PFSM_ERR_INT_SHIFT},
    [PMIC_IRQ_VCCA_OVP_INT] = {INT_SEVERE_ERR_REGADDR, PMIC_INVALID_REGADDR, VCCA_OVP_INT_SHIFT},
    [PMIC_IRQ_TSD_IMM_INT]  = {INT_SEVERE_ERR_REGADDR, PMIC_INVALID_REGADDR, TSD_IMM_INT_SHIFT},

    // INT_MODERATE_ERR register
    [PMIC_IRQ_RECOV_CNT_INT]    = {INT_MODERATE_ERR_REGADDR, PMIC_INVALID_REGADDR, RECOV_CNT_INT_SHIFT},
    [PMIC_IRQ_REG_CRC_ERR_INT]  = {INT_MODERATE_ERR_REGADDR, MASK_MODERATE_ERR_REGADDR, REG_CRC_ERR_INT_SHIFT},
    [PMIC_IRQ_BIST_FAIL_INT]    = {INT_MODERATE_ERR_REGADDR, MASK_MODERATE_ERR_REGADDR, BIST_FAIL_INT_SHIFT},
    [PMIC_IRQ_TSD_ORD_INT]      = {INT_MODERATE_ERR_REGADDR, PMIC_INVALID_REGADDR, TSD_ORD_INT_SHIFT},

    // INT_MISC register
    [PMIC_IRQ_ADC_CONV_READY_INT] = {INT_MISC_REGADDR, MASK_MISC_REGADDR, ADC_CONV_READY_INT_SHIFT},
    [PMIC_IRQ_PB_RISE_INT]        = {INT_MISC_REGADDR, MASK_MISC_REGADDR, PB_RISE_INT_SHIFT},
    [PMIC_IRQ_PB_FALL_INT]        = {INT_MISC_REGADDR, MASK_MISC_REGADDR, PB_FALL_INT_SHIFT},
    [PMIC_IRQ_PB_LONG_INT]        = {INT_MISC_REGADDR, MASK_MISC_REGADDR, PB_LONG_INT_SHIFT},
    [PMIC_IRQ_TWARN_INT]          = {INT_MISC_REGADDR, MASK_MISC_REGADDR, TWARN_INT_SHIFT},
    [PMIC_IRQ_REG_UNLOCK_INT]     = {INT_MISC_REGADDR, MASK_MISC_REGADDR, REG_UNLOCK_INT_SHIFT},
    [PMIC_IRQ_EXT_CLK_INT]        = {INT_MISC_REGADDR, MASK_MISC_REGADDR, EXT_CLK_INT_SHIFT},
    [PMIC_IRQ_BIST_PASS_INT]      = {INT_MISC_REGADDR, MASK_MISC_REGADDR, BIST_PASS_INT_SHIFT},

    // INT_STARTUP register
    [PMIC_IRQ_SOFT_REBOOT_INT] = {INT_STARTUP_REGADDR, MASK_STARTUP_REGADDR, SOFT_REBOOT_INT_SHIFT},
    [PMIC_IRQ_FSD_INT]         = {INT_STARTUP_REGADDR, MASK_STARTUP_REGADDR, FSD_INT_SHIFT},
    [PMIC_IRQ_PB_SHORT_INT]    = {INT_STARTUP_REGADDR, MASK_STARTUP_REGADDR, PB_SHORT_INT_SHIFT},
    [PMIC_IRQ_ENABLE_INT]      = {INT_STARTUP_REGADDR, MASK_STARTUP_REGADDR, ENABLE_INT_SHIFT},
    [PMIC_IRQ_VSENSE_INT]      = {INT_STARTUP_REGADDR, MASK_STARTUP_REGADDR, VSENSE_INT_SHIFT},

    // INT_GPIO register
    [PMIC_IRQ_GPIO6_INT] = {INT_GPIO_REGADDR, PMIC_INVALID_REGADDR, GPIO6_INT_SHIFT},
    [PMIC_IRQ_GPIO5_INT] = {INT_GPIO_REGADDR, PMIC_INVALID_REGADDR, GPIO5_INT_SHIFT},
    [PMIC_IRQ_GPIO4_INT] = {INT_GPIO_REGADDR, PMIC_INVALID_REGADDR, GPIO4_INT_SHIFT},
    [PMIC_IRQ_GPIO3_INT] = {INT_GPIO_REGADDR, PMIC_INVALID_REGADDR, GPIO3_INT_SHIFT},
    [PMIC_IRQ_GPIO2_INT] = {INT_GPIO_REGADDR, PMIC_INVALID_REGADDR, GPIO2_INT_SHIFT},
    [PMIC_IRQ_GPIO1_INT] = {INT_GPIO_REGADDR, PMIC_INVALID_REGADDR, GPIO1_INT_SHIFT},

    // INT_LDO_VMON register
    [PMIC_IRQ_VMON2_UVOV_INT] = {INT_LDO_VMON_REGADDR, MASK_LDO_VMON_REGADDR, VMON2_UVOV_INT_SHIFT},
    [PMIC_IRQ_VMON1_UVOV_INT] = {INT_LDO_VMON_REGADDR, MASK_LDO_VMON_REGADDR, VMON1_UVOV_INT_SHIFT},
    [PMIC_IRQ_VCCA_UVOV_INT]  = {INT_LDO_VMON_REGADDR, MASK_LDO_VMON_REGADDR, VCCA_UVOV_INT_SHIFT},
    [PMIC_IRQ_LDO3_UVOV_INT]  = {INT_LDO_VMON_REGADDR, MASK_LDO_VMON_REGADDR, LDO3_UVOV_INT_SHIFT},
    [PMIC_IRQ_LDO2_UVOV_INT]  = {INT_LDO_VMON_REGADDR, MASK_LDO_VMON_REGADDR, LDO2_UVOV_INT_SHIFT},
    [PMIC_IRQ_LDO1_UVOV_INT]  = {INT_LDO_VMON_REGADDR, MASK_LDO_VMON_REGADDR, LDO1_UVOV_INT_SHIFT},

    // INT_BUCK register
    [PMIC_IRQ_BUCK4_UVOV_INT] = {INT_BUCK_REGADDR, MASK_BUCK_REGADDR, BUCK4_UVOV_INT_SHIFT},
    [PMIC_IRQ_BUCK3_UVOV_INT] = {INT_BUCK_REGADDR, MASK_BUCK_REGADDR, BUCK3_UVOV_INT_SHIFT},
    [PMIC_IRQ_BUCK2_UVOV_INT] = {INT_BUCK_REGADDR, MASK_BUCK_REGADDR, BUCK2_UVOV_INT_SHIFT},
    [PMIC_IRQ_BUCK1_UVOV_INT] = {INT_BUCK_REGADDR, MASK_BUCK_REGADDR, BUCK1_UVOV_INT_SHIFT},
};

int32_t Pmic_irqSetMask(Pmic_Handle_t *handle, uint8_t irqNum, bool shouldMask)
{
    int32_t status = Pmic_checkHandle(handle);

    if ((status == PMIC_ST_SUCCESS) && (irqNum > PMIC_IRQ_INT_MAX))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) && (pmicIRQs[irqNum].maskRegAddr == PMIC_INVALID_REGADDR))
    {
        // This IRQ is not maskable
        status = PMIC_ST_ERR_INV_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioUpdateByte_bCS(handle,
                                       pmicIRQs[irqNum].maskRegAddr,
                                       pmicIRQs[irqNum].bitShift,
                                       shouldMask);
    }

    return status;
}

int32_t Pmic_irqSetMasks(Pmic_Handle_t *handle, uint8_t numIrqMasks, const Pmic_IrqMask_t *irqMasks)
{
    int32_t status = Pmic_checkHandle(handle);

    if ((status == PMIC_ST_SUCCESS) && (irqMasks == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) && (numIrqMasks == 0U))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    for (uint8_t i = 0U; (i < numIrqMasks) && (status == PMIC_ST_SUCCESS); i++)
    {
        status = Pmic_irqSetMask(handle, irqMasks[i].irqNum, irqMasks[i].mask);
    }

    return status;
}

int32_t Pmic_irqGetMask(Pmic_Handle_t *handle, uint8_t numIrqMasks, Pmic_IrqMask_t *irqMasks)
{
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    if ((status == PMIC_ST_SUCCESS) && (irqMasks == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) && (numIrqMasks == 0U))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    for (uint8_t i = 0U; (i < numIrqMasks) && (status == PMIC_ST_SUCCESS); i++)
    {
        uint8_t irqNum = irqMasks[i].irqNum;

        if (irqNum > PMIC_IRQ_INT_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
            break;
        }

        if (pmicIRQs[irqNum].maskRegAddr == PMIC_INVALID_REGADDR)
        {
            // This IRQ is not maskable
            irqMasks[i].mask = false;
        }
        else
        {
            status = Pmic_ioRxByte_CS(handle, pmicIRQs[irqNum].maskRegAddr, &regData);
            if (status == PMIC_ST_SUCCESS)
            {
                irqMasks[i].mask = Pmic_getBitField_b(regData, pmicIRQs[irqNum].bitShift);
            }
        }
    }

    return status;
}

int32_t Pmic_irqGetStatus(Pmic_Handle_t *handle, Pmic_IrqStat_t *irqStat)
{
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    if ((status == PMIC_ST_SUCCESS) && (irqStat == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    // Clear the status structure
    if (status == PMIC_ST_SUCCESS)
    {
        irqStat->intrStat[0] = 0U;
        irqStat->intrStat[1] = 0U;
    }

    // Read all interrupt status registers and build the status bitmap
    for (uint8_t irqNum = 0U; (irqNum <= PMIC_IRQ_INT_MAX) && (status == PMIC_ST_SUCCESS); irqNum++)
    {
        status = Pmic_ioRxByte_CS(handle, pmicIRQs[irqNum].statRegAddr, &regData);

        if (status == PMIC_ST_SUCCESS)
        {
            bool flagSet = Pmic_getBitField_b(regData, pmicIRQs[irqNum].bitShift);

            if (flagSet)
            {
                // Set the corresponding bit in the status array
                uint8_t arrayIndex = irqNum / 32U;
                uint8_t bitIndex = irqNum % 32U;
                irqStat->intrStat[arrayIndex] |= (1U << bitIndex);
            }
        }
    }

    return status;
}

int32_t Pmic_irqGetNextFlag(Pmic_IrqStat_t *irqStat, uint8_t *irqNum)
{
    int32_t status = PMIC_ST_SUCCESS;

    if ((irqStat == NULL) || (irqNum == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        bool foundFlag = false;

        // Search through the status array for the next set bit
        for (uint8_t i = 0U; i <= PMIC_IRQ_INT_MAX; i++)
        {
            uint8_t arrayIndex = i / 32U;
            uint8_t bitIndex = i % 32U;

            if ((irqStat->intrStat[arrayIndex] & (1U << bitIndex)) != 0U)
            {
                *irqNum = i;
                // Clear this bit so next call returns the next flag
                irqStat->intrStat[arrayIndex] &= ~(1U << bitIndex);
                foundFlag = true;
                break;
            }
        }

        if (!foundFlag)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
    }

    return status;
}

int32_t Pmic_irqGetFlag(Pmic_Handle_t *handle, uint8_t irqNum, bool *flag)
{
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    if ((status == PMIC_ST_SUCCESS) && (flag == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) && (irqNum > PMIC_IRQ_INT_MAX))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioRxByte_CS(handle, pmicIRQs[irqNum].statRegAddr, &regData);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        *flag = Pmic_getBitField_b(regData, pmicIRQs[irqNum].bitShift);
    }

    return status;
}

int32_t Pmic_irqClrFlag(Pmic_Handle_t *handle, uint8_t irqNum)
{
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    if ((status == PMIC_ST_SUCCESS) && (irqNum > PMIC_IRQ_INT_MAX))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Set the bit to 1 to clear (write-1-to-clear)
        Pmic_setBitField_b(&regData, pmicIRQs[irqNum].bitShift, true);
        status = Pmic_ioTxByte_CS(handle, pmicIRQs[irqNum].statRegAddr, regData);
    }

    return status;
}

int32_t Pmic_irqClrAllFlags(Pmic_Handle_t *handle)
{
    int32_t status = Pmic_checkHandle(handle);

    // Clear all interrupt status registers by writing 0xFF (write-1-to-clear)
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioTxByte_CS(handle, INT_BUCK_REGADDR, 0xFFU);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioTxByte_CS(handle, INT_LDO_VMON_REGADDR, 0xFFU);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioTxByte_CS(handle, INT_GPIO_REGADDR, 0xFFU);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioTxByte_CS(handle, INT_STARTUP_REGADDR, 0xFFU);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioTxByte_CS(handle, INT_MISC_REGADDR, 0xFFU);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioTxByte_CS(handle, INT_MODERATE_ERR_REGADDR, 0xFFU);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioTxByte_CS(handle, INT_SEVERE_ERR_REGADDR, 0xFFU);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioTxByte_CS(handle, INT_FSM_ERR_REGADDR, 0xFFU);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioTxByte_CS(handle, INT_ESM_REGADDR, 0xFFU);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioTxByte_CS(handle, WD_ERR_STATUS_REGADDR, 0xFFU);
    }

    return status;
}
