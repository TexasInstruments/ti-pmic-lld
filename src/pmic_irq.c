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
 * @file pmic_irq.c
 *
 * @brief PMIC LLD IRQ module source file containing definitions to APIs that
 * interact with PMIC IRQs.
 */
#include <stdint.h>

#include "pmic.h"
#include "pmic_irq.h"
#include "pmic_io.h"

#include "regmap/esm.h"
#include "regmap/irq.h"
#include "regmap/wdg.h"
#include <stdint.h>

#define PMIC_NUM_MASK_REGS ((uint8_t)6U)

#define PMIC_NUM_MASK_REGS ((uint8_t)6U)

/**
 * @anchor Pmic_IrqInfo
 * @name PMIC IRQ Information Struct
 *
 * @brief This struct is used to hold information regarding an IRQ.
 *
 * @param statRegAddr Address of the register holding the IRQ status bit.
 * @param maskRegAddr Address of the register holding the bit that masks the IRQ.
 * @param bitShift Position of the IRQ status/mask bit.
 */
typedef struct Pmic_IrqInfo_s
{
    uint8_t statRegAddr;
    uint8_t maskRegAddr;
    uint8_t bitShift;
} Pmic_IrqInfo_t;

/**
 * @brief All IRQs of TPS65036x PMIC that are directly clearable
 */
static const Pmic_IrqInfo_t pmicIRQs[PMIC_IRQ_MAX + 1U] =
{
    // 0
    {
        .statRegAddr = PMIC_INT_BUCK_LDO_REGADDR,
        .maskRegAddr = PMIC_INVALID_REGADDR,
        .bitShift = PMIC_LDO_SC_INT_SHIFT
    },
    // 1
    {
        .statRegAddr = PMIC_INT_BUCK_LDO_REGADDR,
        .maskRegAddr = PMIC_INVALID_REGADDR,
        .bitShift = PMIC_BUCK3_SC_INT_SHIFT
    },
    // 2
    {
        .statRegAddr = PMIC_INT_BUCK_LDO_REGADDR,
        .maskRegAddr = PMIC_INVALID_REGADDR,
        .bitShift = PMIC_BUCK2_SC_INT_SHIFT
    },
    // 3
    {
        .statRegAddr = PMIC_INT_BUCK_LDO_REGADDR,
        .maskRegAddr = PMIC_INVALID_REGADDR,
        .bitShift = PMIC_BUCK1_SC_INT_SHIFT
    },
    // 4
    {
        .statRegAddr = PMIC_INT_BUCK1_2_REGADDR,
        .maskRegAddr = PMIC_MASK_BUCK1_2_REGADDR,
        .bitShift = PMIC_BUCK2_OVP_INT_SHIFT
    },
    // 5
    {
        .statRegAddr = PMIC_INT_BUCK1_2_REGADDR,
        .maskRegAddr = PMIC_MASK_BUCK1_2_REGADDR,
        .bitShift = PMIC_BUCK2_UV_INT_SHIFT
    },
    // 6
    {
        .statRegAddr = PMIC_INT_BUCK1_2_REGADDR,
        .maskRegAddr = PMIC_MASK_BUCK1_2_REGADDR,
        .bitShift = PMIC_BUCK2_OV_INT_SHIFT
    },
    // 7
    {
        .statRegAddr = PMIC_INT_BUCK1_2_REGADDR,
        .maskRegAddr = PMIC_MASK_BUCK1_2_REGADDR,
        .bitShift = PMIC_BUCK1_OVP_INT_SHIFT
    },
    // 8
    {
        .statRegAddr = PMIC_INT_BUCK1_2_REGADDR,
        .maskRegAddr = PMIC_MASK_BUCK1_2_REGADDR,
        .bitShift = PMIC_BUCK1_UV_INT_SHIFT
    },
    // 9
    {
        .statRegAddr = PMIC_INT_BUCK1_2_REGADDR,
        .maskRegAddr = PMIC_MASK_BUCK1_2_REGADDR,
        .bitShift = PMIC_BUCK1_OV_INT_SHIFT
    },
    // 10
    {
        .statRegAddr = PMIC_INT_BUCK3_LDO_REGADDR,
        .maskRegAddr = PMIC_MASK_BUCK3_LDO_REGADDR,
        .bitShift = PMIC_LDO_OVP_INT_SHIFT
    },
    // 11
    {
        .statRegAddr = PMIC_INT_BUCK3_LDO_REGADDR,
        .maskRegAddr = PMIC_MASK_BUCK3_LDO_REGADDR,
        .bitShift = PMIC_LDO_UV_INT_SHIFT
    },
    // 12
    {
        .statRegAddr = PMIC_INT_BUCK3_LDO_REGADDR,
        .maskRegAddr = PMIC_MASK_BUCK3_LDO_REGADDR,
        .bitShift = PMIC_LDO_OV_INT_SHIFT
    },
    // 13
    {
        .statRegAddr = PMIC_INT_BUCK3_LDO_REGADDR,
        .maskRegAddr = PMIC_MASK_BUCK3_LDO_REGADDR,
        .bitShift = PMIC_BUCK3_OVP_INT_SHIFT
    },
    // 14
    {
        .statRegAddr = PMIC_INT_BUCK3_LDO_REGADDR,
        .maskRegAddr = PMIC_MASK_BUCK3_LDO_REGADDR,
        .bitShift = PMIC_BUCK3_UV_INT_SHIFT
    },
    // 15
    {
        .statRegAddr = PMIC_INT_BUCK3_LDO_REGADDR,
        .maskRegAddr = PMIC_MASK_BUCK3_LDO_REGADDR,
        .bitShift = PMIC_BUCK3_OV_INT_SHIFT
    },
    // 16
    {
        .statRegAddr = PMIC_INT_MISC_REGADDR,
        .maskRegAddr = PMIC_MASK_MISC_REGADDR,
        .bitShift = PMIC_TWARN_INT_SHIFT
    },
    // 17
    {
        .statRegAddr = PMIC_INT_MISC_REGADDR,
        .maskRegAddr = PMIC_MASK_MISC_REGADDR,
        .bitShift = PMIC_B1_PVIN_UVLO_INT_SHIFT
    },
    // 18
    {
        .statRegAddr = PMIC_INT_MISC_REGADDR,
        .maskRegAddr = PMIC_MASK_MISC_REGADDR,
        .bitShift = PMIC_BUCKS_VSET_ERR_INT_SHIFT
    },
    // 19
    {
        .statRegAddr = PMIC_INT_MISC_REGADDR,
        .maskRegAddr = PMIC_INVALID_REGADDR,
        .bitShift = PMIC_CFG_NVM_VERIFY_ERR_SHIFT
    },
    // 20
    {
        .statRegAddr = PMIC_INT_MISC_REGADDR,
        .maskRegAddr = PMIC_INVALID_REGADDR,
        .bitShift = PMIC_CFG_NVM_VERIFY_DONE_SHIFT
    },
    // 21
    {
        .statRegAddr = PMIC_INT_MISC_REGADDR,
        .maskRegAddr = PMIC_INVALID_REGADDR,
        .bitShift = PMIC_CFG_NVM_PRG_DONE_SHIFT
    },
    // 22
    {
        .statRegAddr = PMIC_INT_MISC_REGADDR,
        .maskRegAddr = PMIC_MASK_MISC_REGADDR,
        .bitShift = PMIC_ABIST_FAIL_INT_SHIFT
    },
    // 23
    {
        .statRegAddr = PMIC_INT_MISC_REGADDR,
        .maskRegAddr = PMIC_MASK_MISC_REGADDR,
        .bitShift = PMIC_ABIST_DONE_INT_SHIFT
    },
    // 24
    {
        .statRegAddr = PMIC_INT_MODERATE_ERR_REGADDR,
        .maskRegAddr = PMIC_MASK_MODERATE_ERR_REGADDR,
        .bitShift = PMIC_GPO_READBACK_INT_SHIFT
    },
    // 25
    {
        .statRegAddr = PMIC_INT_MODERATE_ERR_REGADDR,
        .maskRegAddr = PMIC_MASK_MODERATE_ERR_REGADDR,
        .bitShift = PMIC_NINT_READBACK_INT_SHIFT
    },
    // 26
    {
        .statRegAddr = PMIC_INT_MODERATE_ERR_REGADDR,
        .maskRegAddr = PMIC_MASK_MODERATE_ERR_REGADDR,
        .bitShift = PMIC_CONFIG_CRC_INT_SHIFT
    },
    // 27
    {
        .statRegAddr = PMIC_INT_MODERATE_ERR_REGADDR,
        .maskRegAddr = PMIC_MASK_MODERATE_ERR_REGADDR,
        .bitShift = PMIC_TRIM_TEST_CRC_INT_SHIFT
    },
    // 28
    {
        .statRegAddr = PMIC_INT_MODERATE_ERR_REGADDR,
        .maskRegAddr = PMIC_INVALID_REGADDR,
        .bitShift = PMIC_RECOV_CNT_INT_SHIFT
    },
    // 29
    {
        .statRegAddr = PMIC_INT_SEVERE_ERR_REGADDR,
        .maskRegAddr = PMIC_INVALID_REGADDR,
        .bitShift = PMIC_TSD_IMM_INT_SHIFT
    },
    // 30
    {
        .statRegAddr = PMIC_INT_FSM_ERR_REGADDR,
        .maskRegAddr = PMIC_INVALID_REGADDR,
        .bitShift = PMIC_WD_FIRST_NOK_INT_SHIFT
    },
    // 31
    {
        .statRegAddr = PMIC_INT_FSM_ERR_REGADDR,
        .maskRegAddr = PMIC_INVALID_REGADDR,
        .bitShift = PMIC_WAIT_FOR_PWRCYCLE_INT_SHIFT
    },
    // 32
    {
        .statRegAddr = PMIC_INT_FSM_ERR_REGADDR,
        .maskRegAddr = PMIC_INVALID_REGADDR,
        .bitShift = PMIC_WARM_RESET_INT_SHIFT
    },
    // 33
    {
        .statRegAddr = PMIC_INT_FSM_ERR_REGADDR,
        .maskRegAddr = PMIC_INVALID_REGADDR,
        .bitShift = PMIC_ORD_SHUTDOWN_INT_SHIFT
    },
    // 34
    {
        .statRegAddr = PMIC_INT_FSM_ERR_REGADDR,
        .maskRegAddr = PMIC_INVALID_REGADDR,
        .bitShift = PMIC_IMM_SHUTDOWN_INT_SHIFT
    },
    // 35
    {
        .statRegAddr = PMIC_INT_COMM_ERR_REGADDR,
        .maskRegAddr = PMIC_MASK_COMM_ERR_REGADDR,
        .bitShift = PMIC_MCU_COMM_ERR_INT_SHIFT
    },
    // 36
    {
        .statRegAddr = PMIC_INT_COMM_ERR_REGADDR,
        .maskRegAddr = PMIC_MASK_COMM_ERR_REGADDR,
        .bitShift = PMIC_COMM_ADR_ERR_INT_SHIFT
    },
    // 37
    {
        .statRegAddr = PMIC_INT_COMM_ERR_REGADDR,
        .maskRegAddr = PMIC_MASK_COMM_ERR_REGADDR,
        .bitShift = PMIC_COMM_CRC_ERR_INT_SHIFT
    },
    // 38
    {
        .statRegAddr = PMIC_INT_ESM_REGADDR,
        .maskRegAddr = PMIC_MASK_ESM_REGADDR,
        .bitShift = PMIC_ESM_MCU_RST_INT_SHIFT
    },
    // 39
    {
        .statRegAddr = PMIC_INT_ESM_REGADDR,
        .maskRegAddr = PMIC_MASK_ESM_REGADDR,
        .bitShift = PMIC_ESM_MCU_FAIL_INT_SHIFT
    },
    // 40
    {
        .statRegAddr = PMIC_INT_ESM_REGADDR,
        .maskRegAddr = PMIC_MASK_ESM_REGADDR,
        .bitShift = PMIC_ESM_MCU_PIN_INT_SHIFT
    },
    // 41
    {
        .statRegAddr = PMIC_WD_ERR_STATUS_REGADDR,
        .maskRegAddr = PMIC_INVALID_REGADDR,
        .bitShift = PMIC_WD_RST_INT_SHIFT
    },
    // 42
    {
        .statRegAddr = PMIC_WD_ERR_STATUS_REGADDR,
        .maskRegAddr = PMIC_INVALID_REGADDR,
        .bitShift = PMIC_WD_FAIL_INT_SHIFT
    },
    // 43
    {
        .statRegAddr = PMIC_WD_ERR_STATUS_REGADDR,
        .maskRegAddr = PMIC_INVALID_REGADDR,
        .bitShift = PMIC_WD_LONGWIN_TIMEOUT_INT_SHIFT
    }
};

/*!
 * @brief Function to set the intrStat bit position.
 */
static inline void IRQ_setIntrStat(Pmic_IrqStat_t *irqStat, uint32_t irqNum)
{
    if (irqNum <= PMIC_IRQ_MAX)
    {
        // IRQs 0 to 31 go to index 0, IRQs 32 to 63 go to index 1.
        // At an index, the IRQ is stored at its corresponding bit
        // (e.g., IRQ 49's status will be stored at bit 17 at index 1)
        irqStat->intrStat[irqNum / PMIC_NUM_BITS_IN_INTR_STAT] |= ((uint32_t)1U << (irqNum % PMIC_NUM_BITS_IN_INTR_STAT));
    }
}

static void IRQ_markMaskReg(uint8_t irqNum, uint8_t *markedMaskRegs)
{
    // Iterate over markedMaskRegs array to see if the IRQ's mask
    // register has already been marked for read/write. Mark the
    // register if it hasn't been marked yet.
    for (uint8_t i = 0U; i < PMIC_NUM_MASK_REGS; i++)
    {
        // Element is non-empty if its value is non-zero
        if (markedMaskRegs[i] != 0U)
        {
            // Mask register has already been marked; exit loop
            if (markedMaskRegs[i] == pmicIRQs[irqNum].maskRegAddr)
            {
                break;
            }
            // Element is occupied by another mask register; move onto next element
            else
            {
                continue;
            }
        }
        // Element is empty. Store/mark IRQ's mask register and exit loop
        else
        {
            markedMaskRegs[i] = pmicIRQs[irqNum].maskRegAddr;
            break;
        }
    }
}

static inline void IRQ_markMaskRegForConfig(uint8_t irqNum, uint8_t *markedMaskRegs)
{
    IRQ_markMaskReg(irqNum, markedMaskRegs);
}

static inline void IRQ_markMaskRegForRead(uint8_t irqNum, uint8_t *markedMaskRegs)
{
    IRQ_markMaskReg(irqNum, markedMaskRegs);
}

static int32_t IRQ_configMarkedMaskedRegs(
    const Pmic_CoreHandle_t *pmicHandle, uint8_t numIrqMasks, const Pmic_IrqMask_t *irqMasks, const uint8_t *markedMaskRegs)
{
    uint8_t regData = 0U, i = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    // While there are marked mask registers...
    while (markedMaskRegs[i] != 0U)
    {
        // Read marked mask register
        Pmic_criticalSectionStart(pmicHandle);
        status = Pmic_ioRx(pmicHandle, markedMaskRegs[i], &regData);
        Pmic_criticalSectionStop(pmicHandle);

        if (status == PMIC_ST_SUCCESS)
        {
            // For each mask configuration in irqMasks array, if the configuration's
            // mask register equals the marked mask register, set the mask configuration
            for (uint8_t j = 0U; j < numIrqMasks; j++)
            {
                const uint8_t irqNum = irqMasks[j].irqNum;

                if (pmicIRQs[irqNum].maskRegAddr == markedMaskRegs[i])
                {
                    Pmic_setBitField_b(&regData,
                                       pmicIRQs[irqNum].bitShift,
                                       (uint8_t)(1U << pmicIRQs[irqNum].bitShift),
                                       irqMasks[j].mask);
                }
            }

            // Write new register value back to PMIC
            if (status == PMIC_ST_SUCCESS)
            {
                Pmic_criticalSectionStart(pmicHandle);
                status = Pmic_ioTx(pmicHandle, markedMaskRegs[i], regData);
                Pmic_criticalSectionStop(pmicHandle);
            }
        }

        i++;
        if ((i == PMIC_NUM_MASK_REGS) || (status != PMIC_ST_SUCCESS))
        {
            break;
        }
    }

    return status;
}

int32_t Pmic_irqSetMasks(const Pmic_CoreHandle_t *pmicHandle, uint8_t numIrqMasks, const Pmic_IrqMask_t *irqMasks)
{
    uint8_t markedMaskRegs[PMIC_NUM_MASK_REGS] = {0U};
    int32_t status = Pmic_checkPmicCoreHandle(pmicHandle);

    if ((status == PMIC_ST_SUCCESS) && (irqMasks == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) && (numIrqMasks == 0U))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    // Mark all the IRQ mask registers that require configuration
    if (status == PMIC_ST_SUCCESS)
    {
        // For each mask configuration...
        for (uint8_t i = 0U; i < numIrqMasks; i++)
        {
            const uint8_t irqNum = irqMasks[i].irqNum;

            // Check for invalid IRQ number
            if (irqNum > PMIC_IRQ_MAX)
            {
                status = PMIC_ST_ERR_INV_PARAM;
            }

            // Check whether IRQ is a NMI
            if ((status == PMIC_ST_SUCCESS) && (pmicIRQs[irqNum].maskRegAddr == PMIC_INVALID_REGADDR))
            {
                status = PMIC_ST_IRQ_NON_MASKABLE;
            }

            // Mark mask register that corresponds to the IRQ for configuration
            if (status == PMIC_ST_SUCCESS)
            {
                IRQ_markMaskRegForConfig(irqNum, markedMaskRegs);
            }
        }
    }

    // Configure all IRQ mask registers that are marked for configuration
    if (status == PMIC_ST_SUCCESS)
    {
        status = IRQ_configMarkedMaskedRegs(pmicHandle, numIrqMasks, irqMasks, markedMaskRegs);
    }

    return status;
}

static int32_t IRQ_readMarkedMaskedRegs(
    const Pmic_CoreHandle_t *pmicHandle, uint8_t numIrqMasks, Pmic_IrqMask_t *irqMasks, const uint8_t *markedMaskRegs)
{
    uint8_t regData = 0U, i = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    // While there are marked mask registers...
    while (markedMaskRegs[i] != 0U)
    {
        // Read marked mask register
        Pmic_criticalSectionStart(pmicHandle);
        status = Pmic_ioRx(pmicHandle, markedMaskRegs[i], &regData);
        Pmic_criticalSectionStop(pmicHandle);

        if (status == PMIC_ST_SUCCESS)
        {
            // For each mask configuration in irqMasks array, if the configuration's
            // mask register equals the marked mask register, extract the mask configuration
            for (uint8_t j = 0U; j < numIrqMasks; j++)
            {
                const uint8_t irqNum = irqMasks[j].irqNum;

                if (pmicIRQs[irqNum].maskRegAddr == markedMaskRegs[i])
                {
                    irqMasks[j].mask = Pmic_getBitField_b(regData, pmicIRQs[irqNum].bitShift);
                }
            }
        }

        i++;
        if ((i == PMIC_NUM_MASK_REGS) || (status != PMIC_ST_SUCCESS))
        {
            break;
        }
    }

    return status;
}

int32_t Pmic_irqGetMasks(const Pmic_CoreHandle_t *pmicHandle, uint8_t numIrqMasks, Pmic_IrqMask_t *irqMasks)
{
    uint8_t markedMaskRegs[PMIC_NUM_MASK_REGS] = {0U};
    int32_t status = Pmic_checkPmicCoreHandle(pmicHandle);

    if ((status == PMIC_ST_SUCCESS) && (irqMasks == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) && (numIrqMasks == 0U))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    // Mark all the IRQ mask registers that require a read
    if (status == PMIC_ST_SUCCESS)
    {
        // For each mask configuration...
        for (uint8_t i = 0U; i < numIrqMasks; i++)
        {
            const uint8_t irqNum = irqMasks[i].irqNum;

            // Check for invalid IRQ number
            if (irqNum > PMIC_IRQ_MAX)
            {
                status = PMIC_ST_ERR_INV_PARAM;
            }

            // Check whether IRQ is a NMI
            if ((status == PMIC_ST_SUCCESS) && (pmicIRQs[irqNum].maskRegAddr == PMIC_INVALID_REGADDR))
            {
                status = PMIC_ST_IRQ_NON_MASKABLE;
            }

            // Mark mask register that corresponds to the IRQ for read
            if (status == PMIC_ST_SUCCESS)
            {
                IRQ_markMaskRegForRead(irqNum, markedMaskRegs);
            }
        }
    }

    // Read all IRQ mask registers that are marked
    if (status == PMIC_ST_SUCCESS)
    {
        status = IRQ_readMarkedMaskedRegs(pmicHandle, numIrqMasks, irqMasks, markedMaskRegs);
    }

    return status;
}

static inline void IRQ_extractBits(Pmic_IrqStat_t *irqStat, uint8_t regData, const uint8_t *irqs, uint8_t numIrqs)
{
    for (uint8_t i = 0; i < numIrqs; i++)
    {
        if (Pmic_getBitField_b(regData, pmicIRQs[irqs[i]].bitShift))
        {
            IRQ_setIntrStat(irqStat, irqs[i]);
        }
    }
}

static int32_t IRQ_readL2IntCommErr(const Pmic_CoreHandle_t *pmicHandle, Pmic_IrqStat_t *irqStat)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    // Read INT_COMM_ERR register
    Pmic_criticalSectionStart(pmicHandle);
    status = Pmic_ioRx(pmicHandle, PMIC_INT_COMM_ERR_REGADDR, &regData);
    Pmic_criticalSectionStop(pmicHandle);

    if (status == PMIC_ST_SUCCESS)
    {
        const uint8_t irqs[] = {
            PMIC_MCU_COMM_ERR_INT,
            PMIC_COMM_ADR_ERR_INT,
            PMIC_COMM_CRC_ERR_INT
        };

        IRQ_extractBits(irqStat, regData, irqs, COUNT(irqs));
    }

    return status;
}

static int32_t IRQ_readL2IntEsm(const Pmic_CoreHandle_t *pmicHandle, Pmic_IrqStat_t *irqStat)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    Pmic_criticalSectionStart(pmicHandle);
    status = Pmic_ioRx(pmicHandle, PMIC_INT_ESM_REGADDR, &regData);
    Pmic_criticalSectionStop(pmicHandle);

    if (status == PMIC_ST_SUCCESS)
    {
        const uint8_t irqs[] = {
            PMIC_ESM_MCU_RST_INT,
            PMIC_ESM_MCU_FAIL_INT,
            PMIC_ESM_MCU_PIN_INT
        };

        IRQ_extractBits(irqStat, regData, irqs, COUNT(irqs));
    }

    return status;
}

static int32_t IRQ_readL2WdErrStatus(const Pmic_CoreHandle_t *pmicHandle, Pmic_IrqStat_t *irqStat)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    // Read WD_ERR_STATUS register
    Pmic_criticalSectionStart(pmicHandle);
    status = Pmic_ioRx(pmicHandle, PMIC_WD_ERR_STATUS_REGADDR, &regData);
    Pmic_criticalSectionStop(pmicHandle);

    if (status == PMIC_ST_SUCCESS)
    {
        const uint8_t irqs[] = {
            PMIC_WD_RST_INT,
            PMIC_WD_FAIL_INT,
            PMIC_WD_LONGWIN_TIMEOUT_INT
        };

        IRQ_extractBits(irqStat, regData, irqs, COUNT(irqs));
    }

    return status;
}

static int32_t IRQ_readL1IntFsmErr(const Pmic_CoreHandle_t *pmicHandle, Pmic_IrqStat_t *irqStat)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    // Read INT_FSM_ERR register
    Pmic_criticalSectionStart(pmicHandle);
    status = Pmic_ioRx(pmicHandle, PMIC_INT_FSM_ERR_REGADDR, &regData);
    Pmic_criticalSectionStop(pmicHandle);

    // If WD_INT bit is set, read WD_ERR_STATUS register
    if ((status == PMIC_ST_SUCCESS) && Pmic_getBitField_b(regData, PMIC_WD_INT_SHIFT))
    {
        status = IRQ_readL2WdErrStatus(pmicHandle, irqStat);
    }

    // If COMM_ERR_INT bit is set, read INT_COMM_ERR register
    if ((status == PMIC_ST_SUCCESS) && Pmic_getBitField_b(regData, PMIC_COMM_ERR_INT_SHIFT))
    {
        status = IRQ_readL2IntCommErr(pmicHandle, irqStat);
    }

    // If ESM_MCU_INT bit is set, read INT_ESM register
    if ((status == PMIC_ST_SUCCESS) && Pmic_getBitField_b(regData, PMIC_ESM_MCU_INT_SHIFT))
    {
        status = IRQ_readL2IntEsm(pmicHandle, irqStat);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        const uint8_t irqs[] = {
            PMIC_WD_FIRST_NOK_INT,
            PMIC_WAIT_FOR_PWRCYCLE_INT,
            PMIC_WARM_RESET_INT,
            PMIC_ORD_SHUTDOWN_INT,
            PMIC_IMM_SHUTDOWN_INT
        };

        IRQ_extractBits(irqStat, regData, irqs, COUNT(irqs));
    }

    return status;
}

static int32_t IRQ_readL1IntSevereErr(const Pmic_CoreHandle_t *pmicHandle, Pmic_IrqStat_t *irqStat)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    // Read INT_SEVERE_ERR register
    Pmic_criticalSectionStart(pmicHandle);
    status = Pmic_ioRx(pmicHandle, PMIC_INT_SEVERE_ERR_REGADDR, &regData);
    Pmic_criticalSectionStop(pmicHandle);

    if (status == PMIC_ST_SUCCESS)
    {
        const uint8_t irqs[] = {
            PMIC_TSD_IMM_INT
        };

        IRQ_extractBits(irqStat, regData, irqs, COUNT(irqs));
    }

    return status;
}

static int32_t IRQ_readL1IntModerateErr(const Pmic_CoreHandle_t *pmicHandle, Pmic_IrqStat_t *irqStat)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    // Read INT_MODERATE_ERR register
    Pmic_criticalSectionStart(pmicHandle);
    status = Pmic_ioRx(pmicHandle, PMIC_INT_MODERATE_ERR_REGADDR, &regData);
    Pmic_criticalSectionStop(pmicHandle);

    if (status == PMIC_ST_SUCCESS)
    {
        const uint8_t irqs[] = {
            PMIC_GPO_READBACK_INT,
            PMIC_NINT_READBACK_INT,
            PMIC_CONFIG_CRC_INT,
            PMIC_TRIM_TEST_CRC_INT,
            PMIC_RECOV_CNT_INT
        };

        IRQ_extractBits(irqStat, regData, irqs, COUNT(irqs));
    }

    return status;
}

static int32_t IRQ_readL1IntMisc(const Pmic_CoreHandle_t *pmicHandle, Pmic_IrqStat_t *irqStat)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    // Read INT_MISC register
    Pmic_criticalSectionStart(pmicHandle);
    status = Pmic_ioRx(pmicHandle, PMIC_INT_MISC_REGADDR, &regData);
    Pmic_criticalSectionStop(pmicHandle);

    if (status == PMIC_ST_SUCCESS)
    {
        const uint8_t irqs[] = {
            PMIC_TWARN_INT,
            PMIC_B1_PVIN_UVLO_INT,
            PMIC_BUCKS_VSET_ERR_INT,
            PMIC_CFG_NVM_VERIFY_ERR,
            PMIC_CFG_NVM_VERIFY_DONE,
            PMIC_CFG_NVM_PRG_DONE,
            PMIC_ABIST_FAIL_INT,
            PMIC_ABIST_DONE_INT
        };

        IRQ_extractBits(irqStat, regData, irqs, COUNT(irqs));
    }

    return status;
}

static int32_t IRQ_readL2IntBuck3Ldo(const Pmic_CoreHandle_t *pmicHandle, Pmic_IrqStat_t *irqStat)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    // Read INT_BUCK3_LDO register
    Pmic_criticalSectionStart(pmicHandle);
    status = Pmic_ioRx(pmicHandle, PMIC_INT_BUCK3_LDO_REGADDR, &regData);
    Pmic_criticalSectionStop(pmicHandle);

    if (status == PMIC_ST_SUCCESS)
    {
        const uint8_t irqs[] = {
            PMIC_LDO_OVP_INT,
            PMIC_LDO_UV_INT,
            PMIC_LDO_OV_INT,
            PMIC_BUCK3_OVP_INT,
            PMIC_BUCK3_UV_INT,
            PMIC_BUCK3_OV_INT
        };

        IRQ_extractBits(irqStat, regData, irqs, COUNT(irqs));
    }

    return status;
}

static int32_t IRQ_readL2IntBuck1_2(const Pmic_CoreHandle_t *pmicHandle, Pmic_IrqStat_t *irqStat)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    // Read INT_BUCK1_2 register
    Pmic_criticalSectionStart(pmicHandle);
    status = Pmic_ioRx(pmicHandle, PMIC_INT_BUCK1_2_REGADDR, &regData);
    Pmic_criticalSectionStop(pmicHandle);

    if (status == PMIC_ST_SUCCESS)
    {
        const uint8_t irqs[] = {
            PMIC_BUCK2_OVP_INT,
            PMIC_BUCK2_UV_INT,
            PMIC_BUCK2_OV_INT,
            PMIC_BUCK1_OVP_INT,
            PMIC_BUCK1_UV_INT,
            PMIC_BUCK1_OV_INT,
        };

        IRQ_extractBits(irqStat, regData, irqs, COUNT(irqs));
    }

    return status;
}

static int32_t IRQ_readL1IntBuckLdo(const Pmic_CoreHandle_t *pmicHandle, Pmic_IrqStat_t *irqStat)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    // Read INT_BUCK_LDO register
    Pmic_criticalSectionStart(pmicHandle);
    status = Pmic_ioRx(pmicHandle, PMIC_INT_BUCK_LDO_REGADDR, &regData);
    Pmic_criticalSectionStop(pmicHandle);

    if (status == PMIC_ST_SUCCESS)
    {
        const uint8_t irqs[] = {
            PMIC_LDO_SC_INT,
            PMIC_BUCK3_SC_INT,
            PMIC_BUCK2_SC_INT,
            PMIC_BUCK1_SC_INT,
        };

        IRQ_extractBits(irqStat, regData, irqs, COUNT(irqs));

        // If LDO_INT bit or BUCK3_INT bit is set, read INT_BUCK3_LDO register
        if (Pmic_getBitField_b(regData, PMIC_LDO_INT_SHIFT) || Pmic_getBitField_b(regData, PMIC_BUCK3_INT_SHIFT))
        {
            status = IRQ_readL2IntBuck3Ldo(pmicHandle, irqStat);
        }

        // If BUCK2_INT bit or BUCK1_INT bit is set is set, read INT_BUCK1_2 register
        if ((status == PMIC_ST_SUCCESS) &&
            (Pmic_getBitField_b(regData, PMIC_BUCK2_INT_SHIFT) || Pmic_getBitField_b(regData, PMIC_BUCK1_INT_SHIFT)))
        {
            status = IRQ_readL2IntBuck1_2(pmicHandle, irqStat);
        }
    }

    return status;
}

static int32_t IRQ_readL0(const Pmic_CoreHandle_t *pmicHandle, Pmic_IrqStat_t *irqStat)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    // Read INT_TOP register
    Pmic_criticalSectionStart(pmicHandle);
    status = Pmic_ioRx(pmicHandle, PMIC_INT_TOP_REGADDR, &regData);
    Pmic_criticalSectionStop(pmicHandle);

    // If FSM_ERR_INT bit is set, read INT_FSM_ERR register
    if ((status == PMIC_ST_SUCCESS) && Pmic_getBitField_b(regData, PMIC_FSM_ERR_INT_SHIFT))
    {
        status = IRQ_readL1IntFsmErr(pmicHandle, irqStat);
    }

    // If SEVERE_ERR_INT bit is set, read INT_SEVERE_ERR register
    if ((status == PMIC_ST_SUCCESS) && Pmic_getBitField_b(regData, PMIC_SEVERE_ERR_INT_SHIFT))
    {
        status = IRQ_readL1IntSevereErr(pmicHandle, irqStat);
    }

    // If MODERATE_ERR_INT bit is set, read INT_MODERATE_ERR register
    if ((status == PMIC_ST_SUCCESS) && Pmic_getBitField_b(regData, PMIC_MODERATE_ERR_INT_SHIFT))
    {
        status = IRQ_readL1IntModerateErr(pmicHandle, irqStat);
    }

    // If MISC_INT bit is set, read INT_MISC register
    if ((status == PMIC_ST_SUCCESS) && Pmic_getBitField_b(regData, PMIC_MISC_INT_SHIFT))
    {
        status = IRQ_readL1IntMisc(pmicHandle, irqStat);
    }

    // If BUCK_LDO_INT bit is set, read INT_BUCK_LDO register
    if ((status == PMIC_ST_SUCCESS) && Pmic_getBitField_b(regData, PMIC_BUCK_LDO_INT_SHIFT))
    {
        status = IRQ_readL1IntBuckLdo(pmicHandle, irqStat);
    }

    return status;
}

int32_t Pmic_irqGetStat(const Pmic_CoreHandle_t *pmicHandle, Pmic_IrqStat_t *irqStat)
{
    int32_t status = Pmic_checkPmicCoreHandle(pmicHandle);

    if ((status == PMIC_ST_SUCCESS) && (irqStat == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        memset(irqStat->intrStat, 0U, PMIC_NUM_ELEM_IN_INTR_STAT);

        status = IRQ_readL0(pmicHandle, irqStat);
    }

    return status;
}

static uint8_t IRQ_getNextFlag(Pmic_IrqStat_t *irqStat)
{
    uint8_t index = 0U, bitPos = 0U;
    bool foundFlag = false;

    // For each element in struct member intrStat of irqStat...
    for (index = 0U; index < PMIC_NUM_ELEM_IN_INTR_STAT; index++)
    {
        // If current element has no IRQ statuses set, move onto next element
        if (irqStat->intrStat[index] == 0U)
        {
            continue;
        }

        // For each bit in the element...
        for (bitPos = 0U; bitPos < PMIC_NUM_BITS_IN_INTR_STAT; bitPos++)
        {
            // If the bit is set...
            if ((irqStat->intrStat[index] & (1U << bitPos)) != 0U)
            {
                // Clear bit in intrStat element and exit loop
                irqStat->intrStat[index] &= ~(1U << bitPos);
                foundFlag = true;
                break;
            }
        }

        if (foundFlag)
        {
            break;
        }
    }

    // Return the corresponding IRQ number
    return (bitPos + (PMIC_NUM_BITS_IN_INTR_STAT * index));
}

int32_t Pmic_irqGetNextFlag(Pmic_IrqStat_t *irqStat, uint8_t *irqNum)
{
    int32_t status = PMIC_ST_SUCCESS;

    if (((irqStat == NULL) || (irqNum == NULL)))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) && (irqStat->intrStat[0U] == 0U) && (irqStat->intrStat[1U] == 0U))
    {
        status = PMIC_ST_WARN_NO_IRQ_REMAINING;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        *irqNum = IRQ_getNextFlag(irqStat);
    }

    return status;
}

int32_t Pmic_irqGetFlag(const Pmic_CoreHandle_t *pmicHandle, uint8_t irqNum, bool *flag)
{
    uint8_t regData = 0U;
    int32_t status = Pmic_checkPmicCoreHandle(pmicHandle);

    if ((status == PMIC_ST_SUCCESS) && (irqNum > PMIC_IRQ_MAX))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) && (flag == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    // Read IRQ status register
    if (status == PMIC_ST_SUCCESS)
    {
        Pmic_criticalSectionStart(pmicHandle);
        status = Pmic_ioRx(pmicHandle, pmicIRQs[irqNum].statRegAddr, &regData);
        Pmic_criticalSectionStop(pmicHandle);
    }

    // Extract IRQ status
    if (status == PMIC_ST_SUCCESS)
    {
        *flag = Pmic_getBitField_b(regData, pmicIRQs[irqNum].bitShift);
    }

    return status;
}

int32_t Pmic_irqClrFlag(const Pmic_CoreHandle_t *pmicHandle, uint8_t irqNum)
{
    uint8_t regData = 0U;
    int32_t status = Pmic_checkPmicCoreHandle(pmicHandle);

    if ((status == PMIC_ST_SUCCESS) && (irqNum > PMIC_IRQ_MAX))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // IRQ statuses are W1C - write 1 to clear
        Pmic_setBitField(&regData, pmicIRQs[irqNum].bitShift, (uint8_t)(1U << pmicIRQs[irqNum].bitShift), 1U);

        // Write data to PMIC
        Pmic_criticalSectionStart(pmicHandle);
        status = Pmic_ioTx(pmicHandle, pmicIRQs[irqNum].statRegAddr, regData);
        Pmic_criticalSectionStop(pmicHandle);
    }

    return status;
}

int32_t Pmic_irqClrAllFlags(const Pmic_CoreHandle_t *pmicHandle)
{
    int32_t status = Pmic_checkPmicCoreHandle(pmicHandle);

    /* All IRQ statuses are W1C - write 1 to clear */

    // Clear INT_BUCK_LDO
    if (status == PMIC_ST_SUCCESS)
    {
        Pmic_criticalSectionStart(pmicHandle);
        status = Pmic_ioTx(pmicHandle, PMIC_INT_BUCK_LDO_REGADDR, 0xFFU);
        Pmic_criticalSectionStop(pmicHandle);
    }

    // Clear INT_BUCK1_2
    if (status == PMIC_ST_SUCCESS)
    {
        Pmic_criticalSectionStart(pmicHandle);
        status = Pmic_ioTx(pmicHandle, PMIC_INT_BUCK1_2_REGADDR, 0xFFU);
        Pmic_criticalSectionStop(pmicHandle);
    }

    // Clear INT_BUCK3_LDO
    if (status == PMIC_ST_SUCCESS)
    {
        Pmic_criticalSectionStart(pmicHandle);
        status = Pmic_ioTx(pmicHandle, PMIC_INT_BUCK3_LDO_REGADDR, 0xFFU);
        Pmic_criticalSectionStop(pmicHandle);
    }

    // Clear INT_MISC
    if (status == PMIC_ST_SUCCESS)
    {
        Pmic_criticalSectionStart(pmicHandle);
        status = Pmic_ioTx(pmicHandle, PMIC_INT_MISC_REGADDR, 0xFFU);
        Pmic_criticalSectionStop(pmicHandle);
    }

    // Clear INT_MODERATE_ERR
    if (status == PMIC_ST_SUCCESS)
    {
        Pmic_criticalSectionStart(pmicHandle);
        status = Pmic_ioTx(pmicHandle, PMIC_INT_MODERATE_ERR_REGADDR, 0xFFU);
        Pmic_criticalSectionStop(pmicHandle);
    }

    // Clear INT_SEVERE_ERR
    if (status == PMIC_ST_SUCCESS)
    {
        Pmic_criticalSectionStart(pmicHandle);
        status = Pmic_ioTx(pmicHandle, PMIC_INT_SEVERE_ERR_REGADDR, 0xFFU);
        Pmic_criticalSectionStop(pmicHandle);
    }

    // Clear INT_FSM_ERR
    if (status == PMIC_ST_SUCCESS)
    {
        Pmic_criticalSectionStart(pmicHandle);
        status = Pmic_ioTx(pmicHandle, PMIC_INT_FSM_ERR_REGADDR, 0xFFU);
        Pmic_criticalSectionStop(pmicHandle);
    }

    // Clear INT_COMM_ERR
    if (status == PMIC_ST_SUCCESS)
    {
        Pmic_criticalSectionStart(pmicHandle);
        status = Pmic_ioTx(pmicHandle, PMIC_INT_COMM_ERR_REGADDR, 0xFFU);
        Pmic_criticalSectionStop(pmicHandle);
    }

    // Clear INT_ESM
    if (status == PMIC_ST_SUCCESS)
    {
        Pmic_criticalSectionStart(pmicHandle);
        status = Pmic_ioTx(pmicHandle, PMIC_INT_ESM_REGADDR, 0xFFU);
        Pmic_criticalSectionStop(pmicHandle);
    }

    // Clear WD_ERR_STATUS
    if (status == PMIC_ST_SUCCESS)
    {
        Pmic_criticalSectionStart(pmicHandle);
        status = Pmic_ioTx(pmicHandle, PMIC_WD_ERR_STATUS_REGADDR, 0xFFU);
        Pmic_criticalSectionStop(pmicHandle);
    }

    return status;
}
