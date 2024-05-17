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
#include "pmic.h"
#include "pmic_irq.h"

#include "pmic_io.h"

#include "regmap/irq.h"
#include "regmap/wdg.h"

/**
 * @anchor Pmic_IrqInfo
 * @name PMIC IRQ Information Struct
 *
 * @brief This struct is used to hold information regarding an IRQ.
 *
 * @param irqStatRegAddr Address of the register holding the IRQ status bit.
 * @param irqStatBitPos Position of the IRQ status bit.
 * @param irqMaskRegAddr Address of the register holding the bit that masks the IRQ.
 * @param irqMaskBitPos Position of the IRQ mask bit.
 */
typedef struct Pmic_IrqInfo_s
{
    uint8_t irqStatRegAddr;
    uint8_t irqStatBitPos;
    uint8_t irqMaskRegAddr;
    uint8_t irqMaskBitPos;
} Pmic_IrqInfo_t;

/**
 * @brief All IRQs of TPS65036x PMIC that are directly clearable
 */
static const Pmic_IrqInfo_t pmicIRQs[PMIC_IRQ_MAX + 1U] =
{
    // 0
    {
        .irqStatRegAddr = PMIC_INT_BUCK_LDO_REGADDR,
        .irqStatBitPos = PMIC_LDO_SC_INT_SHIFT,
        .irqMaskRegAddr = PMIC_INVALID_REGADDR,
        .irqMaskBitPos = PMIC_INVALID_VALUE
    },
    // 1
    {
        .irqStatRegAddr = PMIC_INT_BUCK_LDO_REGADDR,
        .irqStatBitPos = PMIC_BUCK3_SC_INT_SHIFT,
        .irqMaskRegAddr = PMIC_INVALID_REGADDR,
        .irqMaskBitPos = PMIC_INVALID_VALUE
    },
    // 2
    {
        .irqStatRegAddr = PMIC_INT_BUCK_LDO_REGADDR,
        .irqStatBitPos = PMIC_BUCK2_SC_INT_SHIFT,
        .irqMaskRegAddr = PMIC_INVALID_REGADDR,
        .irqMaskBitPos = PMIC_INVALID_VALUE
    },
    // 3
    {
        .irqStatRegAddr = PMIC_INT_BUCK_LDO_REGADDR,
        .irqStatBitPos = PMIC_BUCK1_SC_INT_SHIFT,
        .irqMaskRegAddr = PMIC_INVALID_REGADDR,
        .irqMaskBitPos = PMIC_INVALID_VALUE
    },
    // 4
    {
        .irqStatRegAddr = PMIC_INT_BUCK1_2_REGADDR,
        .irqStatBitPos = PMIC_BUCK2_OVP_INT_SHIFT,
        .irqMaskRegAddr = PMIC_MASK_BUCK1_2_REGADDR,
        .irqMaskBitPos = PMIC_BUCK2_OVP_MASK_SHIFT
    },
    // 5
    {
        .irqStatRegAddr = PMIC_INT_BUCK1_2_REGADDR,
        .irqStatBitPos = PMIC_BUCK2_UV_INT_SHIFT,
        .irqMaskRegAddr = PMIC_MASK_BUCK1_2_REGADDR,
        .irqMaskBitPos = PMIC_BUCK2_UV_MASK_SHIFT
    },
    // 6
    {
        .irqStatRegAddr = PMIC_INT_BUCK1_2_REGADDR,
        .irqStatBitPos = PMIC_BUCK2_OV_INT_SHIFT,
        .irqMaskRegAddr = PMIC_MASK_BUCK1_2_REGADDR,
        .irqMaskBitPos = PMIC_BUCK2_OV_MASK_SHIFT
    },
    // 7
    {
        .irqStatRegAddr = PMIC_INT_BUCK1_2_REGADDR,
        .irqStatBitPos = PMIC_BUCK1_OVP_INT_SHIFT,
        .irqMaskRegAddr = PMIC_MASK_BUCK1_2_REGADDR,
        .irqMaskBitPos = PMIC_BUCK1_OVP_MASK_SHIFT
    },
    // 8
    {
        .irqStatRegAddr = PMIC_INT_BUCK1_2_REGADDR,
        .irqStatBitPos = PMIC_BUCK1_UV_INT_SHIFT,
        .irqMaskRegAddr = PMIC_MASK_BUCK1_2_REGADDR,
        .irqMaskBitPos = PMIC_BUCK1_UV_MASK_SHIFT
    },
    // 9
    {
        .irqStatRegAddr = PMIC_INT_BUCK1_2_REGADDR,
        .irqStatBitPos = PMIC_BUCK1_OV_INT_SHIFT,
        .irqMaskRegAddr = PMIC_MASK_BUCK1_2_REGADDR,
        .irqMaskBitPos = PMIC_BUCK1_OV_MASK_SHIFT
    },
    // 10
    {
        .irqStatRegAddr = PMIC_INT_BUCK3_LDO_REGADDR,
        .irqStatBitPos = PMIC_LDO_OVP_INT_SHIFT,
        .irqMaskRegAddr = PMIC_MASK_BUCK3_LDO_REGADDR,
        .irqMaskBitPos = PMIC_LDO_OVP_MASK_SHIFT
    },
    // 11
    {
        .irqStatRegAddr = PMIC_INT_BUCK3_LDO_REGADDR,
        .irqStatBitPos = PMIC_LDO_UV_INT_SHIFT,
        .irqMaskRegAddr = PMIC_MASK_BUCK3_LDO_REGADDR,
        .irqMaskBitPos = PMIC_LDO_UV_MASK_SHIFT
    },
    // 12
    {
        .irqStatRegAddr = PMIC_INT_BUCK3_LDO_REGADDR,
        .irqStatBitPos = PMIC_LDO_OV_INT_SHIFT,
        .irqMaskRegAddr = PMIC_MASK_BUCK3_LDO_REGADDR,
        .irqMaskBitPos = PMIC_LDO_OV_MASK_SHIFT
    },
    // 13
    {
        .irqStatRegAddr = PMIC_INT_BUCK3_LDO_REGADDR,
        .irqStatBitPos = PMIC_BUCK3_OVP_INT_SHIFT,
        .irqMaskRegAddr = PMIC_MASK_BUCK3_LDO_REGADDR,
        .irqMaskBitPos = PMIC_BUCK3_OVP_MASK_SHIFT
    },
    // 14
    {
        .irqStatRegAddr = PMIC_INT_BUCK3_LDO_REGADDR,
        .irqStatBitPos = PMIC_BUCK3_UV_INT_SHIFT,
        .irqMaskRegAddr = PMIC_MASK_BUCK3_LDO_REGADDR,
        .irqMaskBitPos = PMIC_BUCK3_UV_MASK_SHIFT
    },
    // 15
    {
        .irqStatRegAddr = PMIC_INT_BUCK3_LDO_REGADDR,
        .irqStatBitPos = PMIC_BUCK3_OV_INT_SHIFT,
        .irqMaskRegAddr = PMIC_MASK_BUCK3_LDO_REGADDR,
        .irqMaskBitPos = PMIC_BUCK3_OV_MASK_SHIFT
    },
    // 16
    {
        .irqStatRegAddr = PMIC_INT_MISC_REGADDR,
        .irqStatBitPos = PMIC_TWARN_INT_SHIFT,
        .irqMaskRegAddr = PMIC_MASK_MISC_REGADDR,
        .irqMaskBitPos = PMIC_TWARN_MASK_SHIFT
    },
    // 17
    {
        .irqStatRegAddr = PMIC_INT_MISC_REGADDR,
        .irqStatBitPos = PMIC_B1_PVIN_UVLO_INT_SHIFT,
        .irqMaskRegAddr = PMIC_MASK_MISC_REGADDR,
        .irqMaskBitPos = PMIC_B1_PVIN_UVLO_MASK_SHIFT
    },
    // 18
    {
        .irqStatRegAddr = PMIC_INT_MISC_REGADDR,
        .irqStatBitPos = PMIC_BUCKS_VSET_ERR_INT_SHIFT,
        .irqMaskRegAddr = PMIC_MASK_MISC_REGADDR,
        .irqMaskBitPos = PMIC_BUCKS_VSET_ERR_MASK_SHIFT
    },
    // 19
    {
        .irqStatRegAddr = PMIC_INT_MISC_REGADDR,
        .irqStatBitPos = PMIC_CFG_NVM_VERIFY_ERR_SHIFT,
        .irqMaskRegAddr = PMIC_INVALID_REGADDR,
        .irqMaskBitPos = PMIC_INVALID_VALUE
    },
    // 20
    {
        .irqStatRegAddr = PMIC_INT_MISC_REGADDR,
        .irqStatBitPos = PMIC_CFG_NVM_VERIFY_DONE_SHIFT,
        .irqMaskRegAddr = PMIC_INVALID_REGADDR,
        .irqMaskBitPos = PMIC_INVALID_VALUE
    },
    // 21
    {
        .irqStatRegAddr = PMIC_INT_MISC_REGADDR,
        .irqStatBitPos = PMIC_CFG_NVM_PRG_DONE_SHIFT,
        .irqMaskRegAddr = PMIC_INVALID_REGADDR,
        .irqMaskBitPos = PMIC_INVALID_VALUE
    },
    // 22
    {
        .irqStatRegAddr = PMIC_INT_MISC_REGADDR,
        .irqStatBitPos = PMIC_ABIST_FAIL_INT_SHIFT,
        .irqMaskRegAddr = PMIC_MASK_MISC_REGADDR,
        .irqMaskBitPos = PMIC_ABIST_FAIL_MASK_SHIFT
    },
    // 23
    {
        .irqStatRegAddr = PMIC_INT_MISC_REGADDR,
        .irqStatBitPos = PMIC_ABIST_DONE_INT_SHIFT,
        .irqMaskRegAddr = PMIC_MASK_MISC_REGADDR,
        .irqMaskBitPos = PMIC_ABIST_DONE_MASK_SHIFT
    },
    // 24
    {
        .irqStatRegAddr = PMIC_INT_MODERATE_ERR_REGADDR,
        .irqStatBitPos = PMIC_GPO_READBACK_INT_SHIFT,
        .irqMaskRegAddr = PMIC_MASK_MODERATE_ERR_REGADDR,
        .irqMaskBitPos = PMIC_GPO_READBACK_MASK_SHIFT
    },
    // 25
    {
        .irqStatRegAddr = PMIC_INT_MODERATE_ERR_REGADDR,
        .irqStatBitPos = PMIC_NINT_READBACK_INT_SHIFT,
        .irqMaskRegAddr = PMIC_MASK_MODERATE_ERR_REGADDR,
        .irqMaskBitPos = PMIC_NINT_READBACK_MASK_SHIFT
    },
    // 26
    {
        .irqStatRegAddr = PMIC_INT_MODERATE_ERR_REGADDR,
        .irqStatBitPos = PMIC_CONFIG_CRC_INT_SHIFT,
        .irqMaskRegAddr = PMIC_MASK_MODERATE_ERR_REGADDR,
        .irqMaskBitPos = PMIC_CONFIG_CRC_MASK_SHIFT
    },
    // 27
    {
        .irqStatRegAddr = PMIC_INT_MODERATE_ERR_REGADDR,
        .irqStatBitPos = PMIC_TRIM_TEST_CRC_INT_SHIFT,
        .irqMaskRegAddr = PMIC_MASK_MODERATE_ERR_REGADDR,
        .irqMaskBitPos = PMIC_TRIM_TEST_CRC_MASK_SHIFT
    },
    // 28
    {
        .irqStatRegAddr = PMIC_INT_MODERATE_ERR_REGADDR,
        .irqStatBitPos = PMIC_RECOV_CNT_INT_SHIFT,
        .irqMaskRegAddr = PMIC_INVALID_REGADDR,
        .irqMaskBitPos = PMIC_INVALID_VALUE
    },
    // 29
    {
        .irqStatRegAddr = PMIC_INT_SEVERE_ERR_REGADDR,
        .irqStatBitPos = PMIC_TSD_IMM_INT_SHIFT,
        .irqMaskRegAddr = PMIC_INVALID_REGADDR,
        .irqMaskBitPos = PMIC_INVALID_VALUE
    },
    // 30
    {
        .irqStatRegAddr = PMIC_INT_FSM_ERR_REGADDR,
        .irqStatBitPos = PMIC_WD_FIRST_NOK_INT_SHIFT,
        .irqMaskRegAddr = PMIC_INVALID_REGADDR,
        .irqMaskBitPos = PMIC_INVALID_VALUE
    },
    // 31
    {
        .irqStatRegAddr = PMIC_INT_FSM_ERR_REGADDR,
        .irqStatBitPos = PMIC_WAIT_FOR_PWRCYCLE_INT_SHIFT,
        .irqMaskRegAddr = PMIC_INVALID_REGADDR,
        .irqMaskBitPos = PMIC_INVALID_VALUE
    },
    // 32
    {
        .irqStatRegAddr = PMIC_INT_FSM_ERR_REGADDR,
        .irqStatBitPos = PMIC_WARM_RESET_INT_SHIFT,
        .irqMaskRegAddr = PMIC_INVALID_REGADDR,
        .irqMaskBitPos = PMIC_INVALID_VALUE
    },
    // 33
    {
        .irqStatRegAddr = PMIC_INT_FSM_ERR_REGADDR,
        .irqStatBitPos = PMIC_ORD_SHUTDOWN_INT_SHIFT,
        .irqMaskRegAddr = PMIC_INVALID_REGADDR,
        .irqMaskBitPos = PMIC_INVALID_VALUE
    },
    // 34
    {
        .irqStatRegAddr = PMIC_INT_FSM_ERR_REGADDR,
        .irqStatBitPos = PMIC_IMM_SHUTDOWN_INT_SHIFT,
        .irqMaskRegAddr = PMIC_INVALID_REGADDR,
        .irqMaskBitPos = PMIC_INVALID_VALUE
    },
    // 35
    {
        .irqStatRegAddr = PMIC_INT_COMM_ERR_REGADDR,
        .irqStatBitPos = PMIC_MCU_COMM_ERR_INT_SHIFT,
        .irqMaskRegAddr = PMIC_MASK_COMM_ERR_REGADDR,
        .irqMaskBitPos = PMIC_MCU_COMM_ERR_MASK_SHIFT
    },
    // 36
    {
        .irqStatRegAddr = PMIC_INT_COMM_ERR_REGADDR,
        .irqStatBitPos = PMIC_COMM_ADR_ERR_INT_SHIFT,
        .irqMaskRegAddr = PMIC_MASK_COMM_ERR_REGADDR,
        .irqMaskBitPos = PMIC_COMM_ADR_ERR_MASK_SHIFT
    },
    // 37
    {
        .irqStatRegAddr = PMIC_INT_COMM_ERR_REGADDR,
        .irqStatBitPos = PMIC_COMM_CRC_ERR_INT_SHIFT,
        .irqMaskRegAddr = PMIC_MASK_COMM_ERR_REGADDR,
        .irqMaskBitPos = PMIC_COMM_CRC_ERR_MASK_SHIFT
    },
    // 38
    {
        .irqStatRegAddr = PMIC_INT_ESM_REGADDR,
        .irqStatBitPos = PMIC_ESM_MCU_RST_INT_SHIFT,
        .irqMaskRegAddr = PMIC_MASK_ESM_REGADDR,
        .irqMaskBitPos = PMIC_ESM_MCU_RST_MASK_SHIFT
    },
    // 39
    {
        .irqStatRegAddr = PMIC_INT_ESM_REGADDR,
        .irqStatBitPos = PMIC_ESM_MCU_FAIL_INT_SHIFT,
        .irqMaskRegAddr = PMIC_MASK_ESM_REGADDR,
        .irqMaskBitPos = PMIC_ESM_MCU_FAIL_MASK_SHIFT
    },
    // 40
    {
        .irqStatRegAddr = PMIC_INT_ESM_REGADDR,
        .irqStatBitPos = PMIC_ESM_MCU_PIN_INT_SHIFT,
        .irqMaskRegAddr = PMIC_MASK_ESM_REGADDR,
        .irqMaskBitPos = PMIC_ESM_MCU_PIN_MASK_SHIFT
    },
    // 41
    {
        .irqStatRegAddr = PMIC_WD_ERR_STATUS_REGADDR,
        .irqStatBitPos = PMIC_WD_RST_INT_SHIFT,
        .irqMaskRegAddr = PMIC_INVALID_REGADDR,
        .irqMaskBitPos = PMIC_INVALID_VALUE
    },
    // 42
    {
        .irqStatRegAddr = PMIC_WD_ERR_STATUS_REGADDR,
        .irqStatBitPos = PMIC_WD_FAIL_INT_SHIFT,
        .irqMaskRegAddr = PMIC_INVALID_REGADDR,
        .irqMaskBitPos = PMIC_INVALID_VALUE
    },
    // 43
    {
        .irqStatRegAddr = PMIC_WD_ERR_STATUS_REGADDR,
        .irqStatBitPos = PMIC_WD_LONGWIN_TIMEOUT_INT_SHIFT,
        .irqMaskRegAddr = PMIC_INVALID_REGADDR,
        .irqMaskBitPos = PMIC_INVALID_VALUE
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

/*!
 * @brief Function to clear the intrStat bit position.
 */
static inline void IRQ_clearIntrStat(Pmic_IrqStat_t *irqStat, uint32_t irqNum)
{
    if (irqNum <= PMIC_IRQ_MAX)
    {
        // IRQs 0 to 31 go to index 0, IRQs 32 to 63 go to index 1.
        // At an index, an IRQ's corresponding bit is cleared
        // (e.g., IRQ 49's status at bit 17 at index 1 will be cleared)
        irqStat->intrStat[irqNum / PMIC_NUM_BITS_IN_INTR_STAT] &= ~((uint32_t)1U << (irqNum % PMIC_NUM_BITS_IN_INTR_STAT));
    }
}

int32_t Pmic_irqSetMask(const Pmic_CoreHandle_t *pmicHandle, uint8_t numIrqMasks, const Pmic_IrqMask_t *irqMasks)
{
    uint8_t regData = 0U;
    int32_t status = Pmic_checkPmicCoreHandle(pmicHandle);

    if ((status == PMIC_ST_SUCCESS) && (irqMasks == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) && (numIrqMasks > (PMIC_IRQ_MAX + 1U)))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        for (uint8_t i = 0U; i < numIrqMasks; i++)
        {
            const uint8_t irqNum = irqMasks[i].irqNum;
            uint8_t irqMaskRegAddr = 0U, irqMaskBitShift = 0U, irqMaskBitMask = 0U;

            // Check for invalid IRQ number
            if (irqNum > PMIC_IRQ_MAX)
            {
                status = PMIC_ST_ERR_INV_PARAM;
            }
            else
            {
                irqMaskRegAddr = pmicIRQs[irqNum].irqMaskRegAddr;
                irqMaskBitShift = pmicIRQs[irqNum].irqMaskBitPos;
                irqMaskBitMask = 1U << pmicIRQs[irqNum].irqMaskBitPos;
            }

            // Check whether IRQ is maskable
            if ((status == PMIC_ST_SUCCESS) && (irqMaskRegAddr == PMIC_INVALID_REGADDR))
            {
                status = PMIC_ST_ERR_NOT_SUPPORTED;
            }

            // Read IRQ mask register
            if (status == PMIC_ST_SUCCESS)
            {
                Pmic_criticalSectionStart(pmicHandle);
                status = Pmic_ioRx(pmicHandle, irqMaskRegAddr, &regData);
                Pmic_criticalSectionStop(pmicHandle);
            }

            if (status == PMIC_ST_SUCCESS)
            {
                // Modify IRQ mask bit field
                Pmic_setBitField_b(&regData, irqMaskBitShift, irqMaskBitMask, irqMasks[i].mask);

                // Write new register value back to PMIC
                Pmic_criticalSectionStart(pmicHandle);
                status = Pmic_ioTx(pmicHandle, irqMaskRegAddr, regData);
                Pmic_criticalSectionStop(pmicHandle);
            }

            if (status != PMIC_ST_SUCCESS)
            {
                break;
            }
        }
    }

    return status;
}

int32_t Pmic_irqGetMask(const Pmic_CoreHandle_t *pmicHandle, uint8_t numIrqMasks, Pmic_IrqMask_t *irqMasks)
{
    int32_t status = Pmic_checkPmicCoreHandle(pmicHandle);
    uint8_t regData = 0U;

    if ((status == PMIC_ST_SUCCESS) && (irqMasks == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) && (numIrqMasks > (PMIC_IRQ_MAX + 1U)))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        for (uint8_t i = 0U; i < numIrqMasks; i++)
        {
            const uint8_t irqNum = irqMasks[i].irqNum;
            uint8_t irqMaskRegAddr = 0U, irqMaskBitShift = 0U;

            // Check for invalid IRQ number
            if (irqNum > PMIC_IRQ_MAX)
            {
                status = PMIC_ST_ERR_INV_PARAM;
            }
            else
            {
                irqMaskRegAddr = pmicIRQs[irqNum].irqMaskRegAddr;
                irqMaskBitShift = pmicIRQs[irqNum].irqMaskBitPos;
            }

            // Check whether IRQ is maskable
            if ((status == PMIC_ST_SUCCESS) && (irqMaskRegAddr == PMIC_INVALID_REGADDR))
            {
                status = PMIC_ST_ERR_NOT_SUPPORTED;
            }

            // Read IRQ mask register
            if (status == PMIC_ST_SUCCESS)
            {
                Pmic_criticalSectionStart(pmicHandle);
                status = Pmic_ioRx(pmicHandle, irqMaskRegAddr, &regData);
                Pmic_criticalSectionStop(pmicHandle);
            }

            if (status == PMIC_ST_SUCCESS)
            {
                // Extract IRQ mask bit field
                irqMasks[i].mask = Pmic_getBitField_b(regData, irqMaskBitShift);
            }
            else
            {
                break;
            }
        }
    }

    return status;
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
        // Store MCU_COMM_ERR_INT status
        if (Pmic_getBitField_b(regData, PMIC_MCU_COMM_ERR_INT_SHIFT))
        {
            IRQ_setIntrStat(irqStat, PMIC_MCU_COMM_ERR_INT);
        }

        // Store COMM_ADR_ERR_INT status
        if (Pmic_getBitField_b(regData, PMIC_COMM_ADR_ERR_INT_SHIFT))
        {
            IRQ_setIntrStat(irqStat, PMIC_COMM_ADR_ERR_INT);
        }

        // Store COMM_CRC_ERR_INT status
        if (Pmic_getBitField_b(regData, PMIC_COMM_CRC_ERR_INT_SHIFT))
        {
            IRQ_setIntrStat(irqStat, PMIC_COMM_CRC_ERR_INT);
        }
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
        // Store ESM_MCU_RST_INT status
        if (Pmic_getBitField_b(regData, PMIC_ESM_MCU_RST_INT_SHIFT))
        {
            IRQ_setIntrStat(irqStat, PMIC_ESM_MCU_RST_INT);
        }

        // Store ESM_MCU_FAIL_INT status
        if (Pmic_getBitField_b(regData, PMIC_ESM_MCU_FAIL_INT_SHIFT))
        {
            IRQ_setIntrStat(irqStat, PMIC_ESM_MCU_FAIL_INT);
        }

        // Store ESM_MCU_PIN_INT status
        if (Pmic_getBitField_b(regData, PMIC_ESM_MCU_PIN_INT_SHIFT))
        {
            IRQ_setIntrStat(irqStat, PMIC_ESM_MCU_PIN_INT);
        }
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
        // Store WD_RST_INT status
        if (Pmic_getBitField_b(regData, PMIC_WD_RST_INT_SHIFT))
        {
            IRQ_setIntrStat(irqStat, PMIC_WD_RST_INT);
        }

        // Store WD_FAIL_INT status
        if (Pmic_getBitField_b(regData, PMIC_WD_FAIL_INT_SHIFT))
        {
            IRQ_setIntrStat(irqStat, PMIC_WD_FAIL_INT);
        }

        // Store WD_LONGWIN_TIMEOUT_INT status
        if (Pmic_getBitField_b(regData, PMIC_WD_LONGWIN_TIMEOUT_INT_SHIFT))
        {
            IRQ_setIntrStat(irqStat, PMIC_WD_LONGWIN_TIMEOUT_INT);
        }
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
        // Store WD_FIRST_NOK_INT status
        if (Pmic_getBitField_b(regData, PMIC_WD_FIRST_NOK_INT_SHIFT))
        {
            IRQ_setIntrStat(irqStat, PMIC_WD_FIRST_NOK_INT);
        }

        // Store WAIT_FOR_PWRCYCLE_INT status
        if (Pmic_getBitField_b(regData, PMIC_WAIT_FOR_PWRCYCLE_INT_SHIFT))
        {
            IRQ_setIntrStat(irqStat, PMIC_WAIT_FOR_PWRCYCLE_INT);
        }

        // Store WARM_RESET_INT status
        if (Pmic_getBitField_b(regData, PMIC_WARM_RESET_INT_SHIFT))
        {
            IRQ_setIntrStat(irqStat, PMIC_WARM_RESET_INT);
        }

        // Store ORD_SHUTDOWN_INT status
        if (Pmic_getBitField_b(regData, PMIC_ORD_SHUTDOWN_INT_SHIFT))
        {
            IRQ_setIntrStat(irqStat, PMIC_ORD_SHUTDOWN_INT);
        }

        // Store IMM_SHUTDOWN_INT status
        if (Pmic_getBitField_b(regData, PMIC_IMM_SHUTDOWN_INT_SHIFT))
        {
            IRQ_setIntrStat(irqStat, PMIC_IMM_SHUTDOWN_INT);
        }
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
        // Store TSD_IMM_INT status
        if (Pmic_getBitField_b(regData, PMIC_TSD_IMM_INT_SHIFT))
        {
            IRQ_setIntrStat(irqStat, PMIC_TSD_IMM_INT);
        }
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
        // Store GPO_READBACK_INT status
        if (Pmic_getBitField_b(regData, PMIC_GPO_READBACK_INT_SHIFT))
        {
            IRQ_setIntrStat(irqStat, PMIC_GPO_READBACK_INT);
        }

        // Store NINT_READBACK_INT status
        if (Pmic_getBitField_b(regData, PMIC_NINT_READBACK_INT_SHIFT))
        {
            IRQ_setIntrStat(irqStat, PMIC_NINT_READBACK_INT);
        }

        // Store CONFIG_CRC_INT status
        if (Pmic_getBitField_b(regData, PMIC_CONFIG_CRC_INT_SHIFT))
        {
            IRQ_setIntrStat(irqStat, PMIC_CONFIG_CRC_INT);
        }

        // Store TRIM_TEST_CRC_INT status
        if (Pmic_getBitField_b(regData, PMIC_TRIM_TEST_CRC_INT_SHIFT))
        {
            IRQ_setIntrStat(irqStat, PMIC_TRIM_TEST_CRC_INT);
        }

        // Store RECOV_CNT_INT status
        if (Pmic_getBitField_b(regData, PMIC_RECOV_CNT_INT_SHIFT))
        {
            IRQ_setIntrStat(irqStat, PMIC_RECOV_CNT_INT);
        }
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
        // Store TWARN_INT status
        if (Pmic_getBitField_b(regData, PMIC_TWARN_INT_SHIFT))
        {
            IRQ_setIntrStat(irqStat, PMIC_TWARN_INT);
        }

        // Store B1_PVIN_UVLO_INT status
        if (Pmic_getBitField_b(regData, PMIC_B1_PVIN_UVLO_INT_SHIFT))
        {
            IRQ_setIntrStat(irqStat, PMIC_B1_PVIN_UVLO_INT);
        }

        // Store BUCKS_VSET_ERR_INT status
        if (Pmic_getBitField_b(regData, PMIC_BUCKS_VSET_ERR_INT_SHIFT))
        {
            IRQ_setIntrStat(irqStat, PMIC_BUCKS_VSET_ERR_INT);
        }

        // Store CFG_NVM_VERIFY_ERR status
        if (Pmic_getBitField_b(regData, PMIC_CFG_NVM_VERIFY_ERR_SHIFT))
        {
            IRQ_setIntrStat(irqStat, PMIC_CFG_NVM_VERIFY_ERR);
        }

        // Store CFG_NVM_VERIFY_DONE status
        if (Pmic_getBitField_b(regData, PMIC_CFG_NVM_VERIFY_DONE_SHIFT))
        {
            IRQ_setIntrStat(irqStat, PMIC_CFG_NVM_VERIFY_DONE);
        }

        // Store CFG_NVM_PRG_DONE status
        if (Pmic_getBitField_b(regData, PMIC_CFG_NVM_PRG_DONE_SHIFT))
        {
            IRQ_setIntrStat(irqStat, PMIC_CFG_NVM_PRG_DONE);
        }

        // Store ABIST_FAIL_INT status
        if (Pmic_getBitField_b(regData, PMIC_ABIST_FAIL_INT_SHIFT))
        {
            IRQ_setIntrStat(irqStat, PMIC_ABIST_FAIL_INT);
        }

        // Store ABIST_DONE_INT status
        if (Pmic_getBitField_b(regData, PMIC_ABIST_DONE_INT_SHIFT))
        {
            IRQ_setIntrStat(irqStat, PMIC_ABIST_DONE_INT);
        }
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
        // Store LDO_OVP_INT status
        if (Pmic_getBitField_b(regData, PMIC_LDO_OVP_INT_SHIFT))
        {
            IRQ_setIntrStat(irqStat, PMIC_LDO_OVP_INT);
        }

        // Store LDO_UV_INT status
        if (Pmic_getBitField_b(regData, PMIC_LDO_UV_INT_SHIFT))
        {
            IRQ_setIntrStat(irqStat, PMIC_LDO_UV_INT);
        }

        // Store LDO_OV_INT status
        if (Pmic_getBitField_b(regData, PMIC_LDO_OV_INT_SHIFT))
        {
            IRQ_setIntrStat(irqStat, PMIC_LDO_OV_INT);
        }

        // Store BUCK3_OVP_INT status
        if (Pmic_getBitField_b(regData, PMIC_BUCK3_OVP_INT_SHIFT))
        {
            IRQ_setIntrStat(irqStat, PMIC_BUCK3_OVP_INT);
        }

        // Store BUCK3_UV_INT status
        if (Pmic_getBitField_b(regData, PMIC_BUCK3_UV_INT_SHIFT))
        {
            IRQ_setIntrStat(irqStat, PMIC_BUCK3_UV_INT);
        }

        // Store BUCK3_OV_INT status
        if (Pmic_getBitField_b(regData, PMIC_BUCK3_OV_INT_SHIFT))
        {
            IRQ_setIntrStat(irqStat, PMIC_BUCK3_OV_INT);
        }
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
        // Store BUCK2_OVP_INT status
        if (Pmic_getBitField_b(regData, PMIC_BUCK2_OVP_INT_SHIFT))
        {
            IRQ_setIntrStat(irqStat, PMIC_BUCK2_OVP_INT);
        }

        // Store BUCK2_UV_INT status
        if (Pmic_getBitField_b(regData, PMIC_BUCK2_UV_INT_SHIFT))
        {
            IRQ_setIntrStat(irqStat, PMIC_BUCK2_UV_INT);
        }

        // Store BUCK2_OV_INT status
        if (Pmic_getBitField_b(regData, PMIC_BUCK2_OV_INT_SHIFT))
        {
            IRQ_setIntrStat(irqStat, PMIC_BUCK2_OV_INT);
        }

        // Store BUCK1_OVP_INT status
        if (Pmic_getBitField_b(regData, PMIC_BUCK1_OVP_INT_SHIFT))
        {
            IRQ_setIntrStat(irqStat, PMIC_BUCK1_OVP_INT);
        }

        // Store BUCK1_UV_INT status
        if (Pmic_getBitField_b(regData, PMIC_BUCK1_UV_INT_SHIFT))
        {
            IRQ_setIntrStat(irqStat, PMIC_BUCK1_UV_INT);
        }

        // Store BUCK1_OV_INT status
        if (Pmic_getBitField_b(regData, PMIC_BUCK1_OV_INT_SHIFT))
        {
            IRQ_setIntrStat(irqStat, PMIC_BUCK1_OV_INT);
        }
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
        // Store LDO_SC_INT status
        if (Pmic_getBitField_b(regData, PMIC_LDO_SC_INT_SHIFT))
        {
            IRQ_setIntrStat(irqStat, PMIC_LDO_SC_INT);
        }

        // Store BUCK3_SC_INT status
        if (Pmic_getBitField_b(regData, PMIC_BUCK3_SC_INT_SHIFT))
        {
            IRQ_setIntrStat(irqStat, PMIC_BUCK3_SC_INT);
        }

        // Store BUCK2_SC_INT status
        if (Pmic_getBitField_b(regData, PMIC_BUCK2_SC_INT_SHIFT))
        {
            IRQ_setIntrStat(irqStat, PMIC_BUCK2_SC_INT);
        }

        // Store BUCK1_SC_INT status
        if (Pmic_getBitField_b(regData, PMIC_BUCK1_SC_INT_SHIFT))
        {
            IRQ_setIntrStat(irqStat, PMIC_BUCK1_SC_INT);
        }

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
                break;
            }
        }
    }

    // Return the corresponding IRQ number
    return (bitPos + (PMIC_NUM_BITS_IN_INTR_STAT * index));
}

int32_t Pmic_irqGetNextFlag(const Pmic_CoreHandle_t *pmicHandle, Pmic_IrqStat_t *irqStat, uint8_t *irqNum)
{
    int32_t status = Pmic_checkPmicCoreHandle(pmicHandle);

    if ((status == PMIC_ST_SUCCESS) && ((irqStat == NULL) || (irqNum == NULL)))
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
        status = Pmic_ioRx(pmicHandle, pmicIRQs[irqNum].irqStatRegAddr, &regData);
        Pmic_criticalSectionStop(pmicHandle);
    }

    // Extract IRQ status
    if (status == PMIC_ST_SUCCESS)
    {
        *flag = Pmic_getBitField_b(regData, pmicIRQs[irqNum].irqStatBitPos);
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
        Pmic_setBitField(&regData, pmicIRQs[irqNum].irqStatBitPos, (1U << pmicIRQs[irqNum].irqStatBitPos), 1U);

        // Write data to PMIC
        Pmic_criticalSectionStart(pmicHandle);
        status = Pmic_ioTx(pmicHandle, pmicIRQs[irqNum].irqStatRegAddr, regData);
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