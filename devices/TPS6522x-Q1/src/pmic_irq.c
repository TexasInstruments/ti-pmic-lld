/******************************************************************************
 * Copyright (c) 2026 Texas Instruments Incorporated - http://www.ti.com
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
#include <stdbool.h>

#include "pmic.h"
#include "pmic_irq.h"
#include "pmic_io.h"
#include "regmap/irq.h"

#include <string.h>

#define CLEAR_ALL_STAT_BITS   (0xFFU)
#define PMIC_IRQ_MASKABLE     ((bool)true)
#define PMIC_IRQ_NON_MASKABLE ((bool)false)

/**
 * @brief Copy Pmic_IrqMask_t structure member-wise
 */
static inline void IRQ_copyIrqMask(const Pmic_IrqMask_t *src, Pmic_IrqMask_t *dst)
{
    memmove((void *)dst, (const void *)src, sizeof(Pmic_IrqMask_t));
}

/**
 * @brief Copy Pmic_IrqStatus_t structure member-wise
 */
static inline void IRQ_copyIrqStat(const Pmic_IrqStatus_t *src, Pmic_IrqStatus_t *dst)
{
    memmove((void *)dst, (const void *)src, sizeof(Pmic_IrqStatus_t));
}

typedef struct Pmic_IrqInfo_s
{
    uint16_t statRegAddr;
    uint16_t maskRegAddr;
    uint16_t maskRegAddr2;  /* Secondary mask register (0 if not used) */
    uint8_t bitShift;
    bool isMaskable;
} Pmic_IrqInfo_t;

/**
 * @brief IRQ information table for TPS6522x-Q1 Burton
 * Maps IRQ numbers (0-48) to their status/mask register addresses and bit positions
 */
static const Pmic_IrqInfo_t pmicIRQs[PMIC_IRQ_INT_MAX + 1U] =
{
    // WD_ERR_STATUS register (non-standard location at 0x408) - Non-Maskable Interrupts
    [PMIC_IRQ_WD_RST_NMI]             = {WD_ERR_STATUS_REG, 0, 0, WD_RST_INT_SHIFT, PMIC_IRQ_NON_MASKABLE},
    [PMIC_IRQ_WD_FAIL_NMI]            = {WD_ERR_STATUS_REG, 0, 0, WD_FAIL_INT_SHIFT, PMIC_IRQ_NON_MASKABLE},
    [PMIC_IRQ_WD_LONGWIN_TIMEOUT_NMI] = {WD_ERR_STATUS_REG, 0, 0, WD_LONGWIN_TIMEOUT_INT_SHIFT, PMIC_IRQ_NON_MASKABLE},

    // INT_ESM register - Maskable
    [PMIC_IRQ_ESM_MCU_RST_INT]  = {INT_ESM_REG, MASK_ESM_REG, 0, ESM_MCU_RST_INT_SHIFT, PMIC_IRQ_MASKABLE},
    [PMIC_IRQ_ESM_MCU_FAIL_INT] = {INT_ESM_REG, MASK_ESM_REG, 0, ESM_MCU_FAIL_INT_SHIFT, PMIC_IRQ_MASKABLE},
    [PMIC_IRQ_ESM_MCU_PIN_INT]  = {INT_ESM_REG, MASK_ESM_REG, 0, ESM_MCU_PIN_INT_SHIFT, PMIC_IRQ_MASKABLE},

    // INT_FSM_ERR register - Maskable
    [PMIC_IRQ_I2C2_ERR_INT]      = {INT_FSM_ERR_REG, MASK_FSM_ERR_REG, 0, I2C2_ERR_INT_SHIFT, PMIC_IRQ_MASKABLE},
    [PMIC_IRQ_COMM_ERR_INT]      = {INT_FSM_ERR_REG, MASK_FSM_ERR_REG, 0, COMM_ERR_INT_SHIFT, PMIC_IRQ_MASKABLE},
    [PMIC_IRQ_SOC_PWR_ERR_INT]   = {INT_FSM_ERR_REG, MASK_FSM_ERR_REG, 0, SOC_PWR_ERR_INT_SHIFT, PMIC_IRQ_MASKABLE},
    [PMIC_IRQ_MCU_PWR_ERR_INT]   = {INT_FSM_ERR_REG, MASK_FSM_ERR_REG, 0, MCU_PWR_ERR_INT_SHIFT, PMIC_IRQ_MASKABLE},
    [PMIC_IRQ_ORD_SHUTDOWN_INT]  = {INT_FSM_ERR_REG, MASK_FSM_ERR_REG, 0, ORD_SHUTDOWN_INT_SHIFT, PMIC_IRQ_MASKABLE},
    [PMIC_IRQ_IMM_SHUTOWN_INT]   = {INT_FSM_ERR_REG, MASK_FSM_ERR_REG, 0, IMM_SHUTDOWN_INT_SHIFT, PMIC_IRQ_MASKABLE},

    // INT_SEVERE_ERR register - Non-Maskable Interrupts
    [PMIC_IRQ_BG_XMON_INT]  = {INT_SEVERE_ERR_REG, 0, 0, BG_XMON_INT_SHIFT, PMIC_IRQ_NON_MASKABLE},
    [PMIC_IRQ_PFSM_ERR_INT] = {INT_SEVERE_ERR_REG, 0, 0, PFSM_ERR_INT_SHIFT, PMIC_IRQ_NON_MASKABLE},
    [PMIC_IRQ_VCCA_OVP_INT] = {INT_SEVERE_ERR_REG, 0, 0, VCCA_OVP_INT_SHIFT, PMIC_IRQ_NON_MASKABLE},
    [PMIC_IRQ_TSD_IMM_INT]  = {INT_SEVERE_ERR_REG, 0, 0, TSD_IMM_INT_SHIFT, PMIC_IRQ_NON_MASKABLE},

    // INT_MODERATE_ERR register - Mixed
    [PMIC_IRQ_RECOV_CNT_INT]    = {INT_MODERATE_ERR_REG, 0, 0, RECOV_CNT_INT_SHIFT, PMIC_IRQ_NON_MASKABLE},
    [PMIC_IRQ_REG_CRC_ERR_INT]  = {INT_MODERATE_ERR_REG, MASK_MODERATE_ERR_REG, 0, REG_CRC_ERR_INT_SHIFT, PMIC_IRQ_MASKABLE},
    [PMIC_IRQ_BIST_FAIL_INT]    = {INT_MODERATE_ERR_REG, MASK_MODERATE_ERR_REG, 0, BIST_FAIL_INT_SHIFT, PMIC_IRQ_MASKABLE},
    [PMIC_IRQ_TSD_ORD_INT]      = {INT_MODERATE_ERR_REG, 0, 0, TSD_ORD_INT_SHIFT, PMIC_IRQ_NON_MASKABLE},

    // INT_MISC register - Maskable
    [PMIC_IRQ_ADC_CONV_READY_INT] = {INT_MISC_REG, MASK_MISC_REG, 0, ADC_CONV_READY_INT_SHIFT, PMIC_IRQ_MASKABLE},
    [PMIC_IRQ_PB_RISE_INT]        = {INT_MISC_REG, MASK_MISC_REG, 0, PB_RISE_INT_SHIFT, PMIC_IRQ_MASKABLE},
    [PMIC_IRQ_PB_FALL_INT]        = {INT_MISC_REG, MASK_MISC_REG, 0, PB_FALL_INT_SHIFT, PMIC_IRQ_MASKABLE},
    [PMIC_IRQ_PB_LONG_INT]        = {INT_MISC_REG, MASK_MISC_REG, 0, PB_LONG_INT_SHIFT, PMIC_IRQ_MASKABLE},
    [PMIC_IRQ_TWARN_INT]          = {INT_MISC_REG, MASK_MISC_REG, 0, TWARN_INT_SHIFT, PMIC_IRQ_MASKABLE},
    [PMIC_IRQ_REG_UNLOCK_INT]     = {INT_MISC_REG, MASK_MISC_REG, 0, REG_UNLOCK_INT_SHIFT, PMIC_IRQ_MASKABLE},
    [PMIC_IRQ_EXT_CLK_INT]        = {INT_MISC_REG, MASK_MISC_REG, 0, EXT_CLK_INT_SHIFT, PMIC_IRQ_MASKABLE},
    [PMIC_IRQ_BIST_PASS_INT]      = {INT_MISC_REG, MASK_MISC_REG, 0, BIST_PASS_INT_SHIFT, PMIC_IRQ_MASKABLE},

    // INT_STARTUP register - Maskable
    [PMIC_IRQ_SOFT_REBOOT_INT] = {INT_STARTUP_REG, MASK_STARTUP_REG, 0, SOFT_REBOOT_INT_SHIFT, PMIC_IRQ_MASKABLE},
    [PMIC_IRQ_FSD_INT]         = {INT_STARTUP_REG, MASK_STARTUP_REG, 0, FSD_INT_SHIFT, PMIC_IRQ_MASKABLE},
    [PMIC_IRQ_PB_SHORT_INT]    = {INT_STARTUP_REG, MASK_STARTUP_REG, 0, PB_SHORT_INT_SHIFT, PMIC_IRQ_MASKABLE},
    [PMIC_IRQ_ENABLE_INT]      = {INT_STARTUP_REG, MASK_STARTUP_REG, 0, ENABLE_INT_SHIFT, PMIC_IRQ_MASKABLE},
    [PMIC_IRQ_VSENSE_INT]      = {INT_STARTUP_REG, MASK_STARTUP_REG, 0, VSENSE_INT_SHIFT, PMIC_IRQ_MASKABLE},

    // INT_GPIO register - Maskable Interrupts (dual mask: FALL and RISE registers)
    [PMIC_IRQ_GPIO6_INT] = {INT_GPIO_REG, MASK_GPIO_FALL_REG, MASK_GPIO_RISE_REG, GPIO6_INT_SHIFT, PMIC_IRQ_MASKABLE},
    [PMIC_IRQ_GPIO5_INT] = {INT_GPIO_REG, MASK_GPIO_FALL_REG, MASK_GPIO_RISE_REG, GPIO5_INT_SHIFT, PMIC_IRQ_MASKABLE},
    [PMIC_IRQ_GPIO4_INT] = {INT_GPIO_REG, MASK_GPIO_FALL_REG, MASK_GPIO_RISE_REG, GPIO4_INT_SHIFT, PMIC_IRQ_MASKABLE},
    [PMIC_IRQ_GPIO3_INT] = {INT_GPIO_REG, MASK_GPIO_FALL_REG, MASK_GPIO_RISE_REG, GPIO3_INT_SHIFT, PMIC_IRQ_MASKABLE},
    [PMIC_IRQ_GPIO2_INT] = {INT_GPIO_REG, MASK_GPIO_FALL_REG, MASK_GPIO_RISE_REG, GPIO2_INT_SHIFT, PMIC_IRQ_MASKABLE},
    [PMIC_IRQ_GPIO1_INT] = {INT_GPIO_REG, MASK_GPIO_FALL_REG, MASK_GPIO_RISE_REG, GPIO1_INT_SHIFT, PMIC_IRQ_MASKABLE},

    // INT_LDO_VMON register - Maskable
    [PMIC_IRQ_VMON2_UVOV_INT] = {INT_LDO_VMON_REG, MASK_LDO_VMON_REG, 0, VMON2_UVOV_INT_SHIFT, PMIC_IRQ_MASKABLE},
    [PMIC_IRQ_VMON1_UVOV_INT] = {INT_LDO_VMON_REG, MASK_LDO_VMON_REG, 0, VMON1_UVOV_INT_SHIFT, PMIC_IRQ_MASKABLE},
    [PMIC_IRQ_VCCA_UVOV_INT]  = {INT_LDO_VMON_REG, MASK_LDO_VMON_REG, 0, VCCA_UVOV_INT_SHIFT, PMIC_IRQ_MASKABLE},
    [PMIC_IRQ_LDO3_UVOV_INT]  = {INT_LDO_VMON_REG, MASK_LDO_VMON_REG, 0, LDO3_UVOV_INT_SHIFT, PMIC_IRQ_MASKABLE},
    [PMIC_IRQ_LDO2_UVOV_INT]  = {INT_LDO_VMON_REG, MASK_LDO_VMON_REG, 0, LDO2_UVOV_INT_SHIFT, PMIC_IRQ_MASKABLE},
    [PMIC_IRQ_LDO1_UVOV_INT]  = {INT_LDO_VMON_REG, MASK_LDO_VMON_REG, 0, LDO1_UVOV_INT_SHIFT, PMIC_IRQ_MASKABLE},

    // INT_BUCK register - Maskable
    [PMIC_IRQ_BUCK4_UVOV_INT] = {INT_BUCK_REG, MASK_BUCK_REG, 0, BUCK4_UVOV_INT_SHIFT, PMIC_IRQ_MASKABLE},
    [PMIC_IRQ_BUCK3_UVOV_INT] = {INT_BUCK_REG, MASK_BUCK_REG, 0, BUCK3_UVOV_INT_SHIFT, PMIC_IRQ_MASKABLE},
    [PMIC_IRQ_BUCK2_UVOV_INT] = {INT_BUCK_REG, MASK_BUCK_REG, 0, BUCK2_UVOV_INT_SHIFT, PMIC_IRQ_MASKABLE},
    [PMIC_IRQ_BUCK1_UVOV_INT] = {INT_BUCK_REG, MASK_BUCK_REG, 0, BUCK1_UVOV_INT_SHIFT, PMIC_IRQ_MASKABLE},
};

int32_t Pmic_irqSetMask(const Pmic_Handle_t *handle, uint8_t irqNum, bool shouldMask)
{
    int32_t status = Pmic_checkHandle(handle);

    if ((status == PMIC_ST_SUCCESS) && (irqNum > PMIC_IRQ_INT_MAX))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) && (pmicIRQs[irqNum].isMaskable == PMIC_IRQ_NON_MASKABLE))
    {
        // This IRQ is not maskable
        status = PMIC_ST_ERR_INV_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        /* Write to primary mask register */
        status = Pmic_ioUpdateByte_bCS(handle,
                                       pmicIRQs[irqNum].maskRegAddr,
                                       pmicIRQs[irqNum].bitShift,
                                       shouldMask);

        /* If secondary mask register exists, write to it as well */
        if ((status == PMIC_ST_SUCCESS) && (pmicIRQs[irqNum].maskRegAddr2 != 0U))
        {
            status = Pmic_ioUpdateByte_bCS(handle,
                                           pmicIRQs[irqNum].maskRegAddr2,
                                           pmicIRQs[irqNum].bitShift,
                                           shouldMask);
        }
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_irqSetMasks(const Pmic_Handle_t *handle, uint8_t numIrqMasks, const Pmic_IrqMask_t irqMasks[])
{
    Pmic_IrqMask_t irqMasksLocal[PMIC_IRQ_INT_MAX + 1U];
    int32_t status = Pmic_checkHandle(handle);

    if ((status == PMIC_ST_SUCCESS) && (irqMasks == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) && (numIrqMasks == 0U))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        for (uint8_t i = 0U; i < numIrqMasks; i++)
        {
            IRQ_copyIrqMask(&irqMasks[i], &irqMasksLocal[i]);
        }
    }

    for (uint8_t i = 0U; (i < numIrqMasks) && (status == PMIC_ST_SUCCESS); i++)
    {
        status = Pmic_irqSetMask(handle, irqMasksLocal[i].irqNum, irqMasksLocal[i].mask);
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_irqGetMask(const Pmic_Handle_t *handle, uint8_t numIrqMasks, Pmic_IrqMask_t *irqMasks)
{
    Pmic_IrqMask_t irqMasksLocal[PMIC_IRQ_INT_MAX + 1U];
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;
    uint8_t regData2 = 0U;

    if ((status == PMIC_ST_SUCCESS) && (irqMasks == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) && (numIrqMasks == 0U))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        for (uint8_t i = 0U; i < numIrqMasks; i++)
        {
            IRQ_copyIrqMask(&irqMasks[i], &irqMasksLocal[i]);
        }
    }

    for (uint8_t i = 0U; (i < numIrqMasks) && (status == PMIC_ST_SUCCESS); i++)
    {
        uint8_t irqNum = irqMasksLocal[i].irqNum;

        if (irqNum > PMIC_IRQ_INT_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
            break;
        }

        if (pmicIRQs[irqNum].isMaskable == PMIC_IRQ_NON_MASKABLE)
        {
            // This IRQ is not maskable
            irqMasksLocal[i].mask = false;
        }
        else
        {
            /* Read primary mask register */
            status = Pmic_ioRxByte_CS(handle, pmicIRQs[irqNum].maskRegAddr, &regData);
            if (status == PMIC_ST_SUCCESS)
            {
                bool mask1 = Pmic_getBitField_b(regData, pmicIRQs[irqNum].bitShift);

                /* If secondary mask register exists, read it too */
                if (pmicIRQs[irqNum].maskRegAddr2 != 0U)
                {
                    status = Pmic_ioRxByte_CS(handle, pmicIRQs[irqNum].maskRegAddr2, &regData2);
                    if (status == PMIC_ST_SUCCESS)
                    {
                        bool mask2 = Pmic_getBitField_b(regData2, pmicIRQs[irqNum].bitShift);
                        /* Interrupt is masked only if BOTH registers have mask set */
                        irqMasksLocal[i].mask = mask1 && mask2;
                    }
                }
                else
                {
                    /* Single mask register - use its value directly */
                    irqMasksLocal[i].mask = mask1;
                }
            }
        }
    }

    if (status == PMIC_ST_SUCCESS)
    {
        for (uint8_t i = 0U; i < numIrqMasks; i++)
        {
            IRQ_copyIrqMask(&irqMasksLocal[i], &irqMasks[i]);
        }
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_irqGetStatus(const Pmic_Handle_t *handle, Pmic_IrqStatus_t *irqStat)
{
    Pmic_IrqStatus_t irqStatLocal;
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    if ((status == PMIC_ST_SUCCESS) && (irqStat == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    // Clear the status structure
    if (status == PMIC_ST_SUCCESS)
    {
        irqStatLocal.intrStat[0] = 0U;
        irqStatLocal.intrStat[1] = 0U;
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
                irqStatLocal.intrStat[arrayIndex] |= (1U << bitIndex);
            }
        }
    }

    if (status == PMIC_ST_SUCCESS)
    {
        IRQ_copyIrqStat(&irqStatLocal, irqStat);
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_irqGetNextFlag(const Pmic_Handle_t *handle, Pmic_IrqStatus_t *irqStat, uint8_t *irqNum)
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
            status = PMIC_ST_WARN_NO_IRQ_REMAINING;
        }
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_irqGetFlag(const Pmic_Handle_t *handle, uint8_t irqNum, bool *flag)
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

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_irqClrFlag(const Pmic_Handle_t *handle, uint8_t irqNum)
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
        Pmic_setBitField_b(&regData, pmicIRQs[irqNum].bitShift, (uint8_t)(1U << pmicIRQs[irqNum].bitShift), true);
        status = Pmic_ioTxByte_CS(handle, pmicIRQs[irqNum].statRegAddr, regData);
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_irqClrAllFlags(const Pmic_Handle_t *handle)
{
    int32_t status = Pmic_checkHandle(handle);

    // Clear all interrupt status registers by writing 0xFF (write-1-to-clear)
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioTxByte_CS(handle, INT_BUCK_REG, CLEAR_ALL_STAT_BITS);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioTxByte_CS(handle, INT_LDO_VMON_REG, CLEAR_ALL_STAT_BITS);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioTxByte_CS(handle, INT_GPIO_REG, CLEAR_ALL_STAT_BITS);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioTxByte_CS(handle, INT_STARTUP_REG, CLEAR_ALL_STAT_BITS);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioTxByte_CS(handle, INT_MISC_REG, CLEAR_ALL_STAT_BITS);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioTxByte_CS(handle, INT_MODERATE_ERR_REG, CLEAR_ALL_STAT_BITS);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioTxByte_CS(handle, INT_SEVERE_ERR_REG, CLEAR_ALL_STAT_BITS);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioTxByte_CS(handle, INT_FSM_ERR_REG, CLEAR_ALL_STAT_BITS);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioTxByte_CS(handle, INT_ESM_REG, CLEAR_ALL_STAT_BITS);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioTxByte_CS(handle, WD_ERR_STATUS_REG, CLEAR_ALL_STAT_BITS);
    }

    return Pmic_logStatus(handle, status);
}
