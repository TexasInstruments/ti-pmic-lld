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
#include "pmic_io.h"
#include "pmic_irq.h"

#include "regmap/irq.h"

// Valid values of the SPI_ERR[1:0] bit field
#define SPI_ERR_CMD                 ((uint8_t)1U)
#define SPI_ERR_FORMAT              ((uint8_t)2U)
#define SPI_ERR_OUTPUT_MISMATCH     ((uint8_t)3U)

/**
 * @anchor Pmic_IrqInfo
 * @name PMIC IRQ Information Struct
 *
 * @brief This struct is used to hold information regarding an IRQ.
 *
 * @param regCmd Read command to access the register holding the IRQ status bit.
 * @param bitShift Position of the IRQ status/mask bit.
 */
typedef struct Pmic_IrqInfo_s
{
    uint8_t regCmd;
    uint8_t bitShift;
} Pmic_IrqInfo_t;

/**
 * All IRQ statuses that affect the status byte of the PMIC response during an
 * I/O transfer.
 */
static const Pmic_IrqInfo_t pmicIRQs[PMIC_IRQ_NUM] = {
    {PMIC_CMD_RD_VMON_STAT_1, VBATL_OV_SHIFT},              // 0
    {PMIC_CMD_RD_VMON_STAT_1, VBATL_UV_SHIFT},              // 1
    {PMIC_CMD_RD_VMON_STAT_1, VCP_UV_SHIFT},                // 2
    {PMIC_CMD_RD_VMON_STAT_2, VDD6_OV_SHIFT},               // 3
    {PMIC_CMD_RD_VMON_STAT_2, VDD6_UV_SHIFT},               // 4
    {PMIC_CMD_RD_VMON_STAT_2, VDD5_OV_SHIFT},               // 5
    {PMIC_CMD_RD_VMON_STAT_2, VDD5_UV_SHIFT},               // 6
    {PMIC_CMD_RD_VMON_STAT_2, VDD_3P5_OV_SHIFT},            // 7
    {PMIC_CMD_RD_VMON_STAT_2, VDD_3P5_UV_SHIFT},            // 8
    {PMIC_CMD_RD_VMON_STAT_3, VREG_UV_SHIFT},               // 9
    {PMIC_CMD_RD_VMON_STAT_3, VDD6_LP_UV_SHIFT},            // 10
    {PMIC_CMD_RD_VMON_STAT_3, VSOUT2_OV_SHIFT},             // 11
    {PMIC_CMD_RD_VMON_STAT_3, VSOUT2_UV_SHIFT},             // 12
    {PMIC_CMD_RD_VMON_STAT_3, VSOUT1_OV_SHIFT},             // 13
    {PMIC_CMD_RD_VMON_STAT_3, VSOUT1_UV_SHIFT},             // 14
    {PMIC_CMD_RD_SAFETY_STAT_1, VDD6_OT_FLAG_SHIFT},        // 15
    {PMIC_CMD_RD_SAFETY_STAT_1, VDD6_TP_SHIFT},             // 16
    {PMIC_CMD_RD_SAFETY_STAT_1, VDD5_ILIM_SHIFT},           // 17
    {PMIC_CMD_RD_SAFETY_STAT_1, VDD5_OT_FLAG_SHIFT},        // 18
    {PMIC_CMD_RD_SAFETY_STAT_1, VDD5_TP_SHIFT},             // 19
    {PMIC_CMD_RD_SAFETY_STAT_1, VDD_3P5_ILIM_SHIFT},        // 20
    {PMIC_CMD_RD_SAFETY_STAT_1, VDD_3P5_OT_FLAG_SHIFT},     // 21
    {PMIC_CMD_RD_SAFETY_STAT_1, VDD_3P5_TP_SHIFT},          // 22
    {PMIC_CMD_RD_SAFETY_STAT_2, VDD6_ILIM_SHIFT},           // 23
    {PMIC_CMD_RD_SAFETY_STAT_2, VSOUT2_ILIM_SHIFT},         // 24
    {PMIC_CMD_RD_SAFETY_STAT_2, VSOUT2_OT_FLAG_SHIFT},      // 25
    {PMIC_CMD_RD_SAFETY_STAT_2, VSOUT2_TP_SHIFT},           // 26
    {PMIC_CMD_RD_SAFETY_STAT_2, VSOUT1_ILIM_SHIFT},         // 27
    {PMIC_CMD_RD_SAFETY_STAT_2, VSOUT1_OT_FLAG_SHIFT},      // 28
    {PMIC_CMD_RD_SAFETY_STAT_2, VSOUT1_TP_SHIFT},           // 29
    {PMIC_CMD_RD_SAFETY_STAT_3, CFG_CRC_ERR_SHIFT},         // 30
    {PMIC_CMD_RD_SAFETY_STAT_3, EE_CRC_ERR_SHIFT},          // 31
    {PMIC_CMD_RD_SAFETY_STAT_3, NRES_ERR_SHIFT},            // 32
    {PMIC_CMD_RD_SAFETY_STAT_3, LBIST_ERR_SHIFT},           // 33
    {PMIC_CMD_RD_SAFETY_STAT_3, ABIST_ERR_SHIFT},           // 34
    {PMIC_CMD_RD_SAFETY_STAT_4, SPI_ERR_SHIFT},             // 35
    {PMIC_CMD_RD_SAFETY_STAT_4, SPI_ERR_SHIFT},             // 36
    {PMIC_CMD_RD_SAFETY_STAT_4, SPI_ERR_SHIFT},             // 37
    {PMIC_CMD_RD_SAFETY_STAT_4, LO_LPMCLK_SHIFT},           // 38
    {PMIC_CMD_RD_SAFETY_STAT_4, ENDRV_ERR_SHIFT},           // 39
    {PMIC_CMD_RD_SAFETY_STAT_4, TRIM_ERR_VMON_SHIFT},       // 40
    {PMIC_CMD_RD_SPI_INV_TRAN_STAT, INVALID_CMD_SHIFT},     // 41
    {PMIC_CMD_RD_SPI_INV_TRAN_STAT, UNDEF_CMD_SHIFT},       // 42
    {PMIC_CMD_RD_SPI_INV_TRAN_STAT, CRC_ERR_SHIFT},         // 43
    {PMIC_CMD_RD_SPI_INV_TRAN_STAT, LONG_FRM_SHIFT},        // 44
    {PMIC_CMD_RD_SPI_INV_TRAN_STAT, SHORT_FRM_SHIFT},       // 45
    {PMIC_CMD_RD_SAFETY_ERR_STAT_1, ERROR_PIN_FAIL_SHIFT},  // 46
    {PMIC_CMD_RD_SAFETY_ERR_STAT_1, WD_FAIL_SHIFT},         // 47
    {PMIC_CMD_RD_SAFETY_ERR_STAT_2, DIAG_STATE_TO_SHIFT},   // 48
    {PMIC_CMD_RD_SAM_STAT, SAM_BIST_FAIL_SHIFT},            // 49
    {PMIC_CMD_RD_SAM_STAT, POWER_ON_RESET_SHIFT},           // 50
    {PMIC_CMD_RD_SAM_STAT, SAM_CNT_ERR_SHIFT},              // 51
    {PMIC_CMD_RD_SAM_SIG_STAT, COS_P_OV_SHIFT},             // 52
    {PMIC_CMD_RD_SAM_SIG_STAT, COS_P_UV_SHIFT},             // 53
    {PMIC_CMD_RD_SAM_SIG_STAT, COS_N_OV_SHIFT},             // 54
    {PMIC_CMD_RD_SAM_SIG_STAT, COS_N_UV_SHIFT},             // 55
    {PMIC_CMD_RD_SAM_SIG_STAT, SIN_P_OV_SHIFT},             // 56
    {PMIC_CMD_RD_SAM_SIG_STAT, SIN_P_UV_SHIFT},             // 57
    {PMIC_CMD_RD_SAM_SIG_STAT, SIN_N_OV_SHIFT},             // 58
    {PMIC_CMD_RD_SAM_SIG_STAT, SIN_N_UV_SHIFT}              // 59
};

// Function to set the intrStat bit position.
static inline void IRQ_setIntrStat(Pmic_IrqStat_t *irqStat, uint32_t irqNum)
{
    if (irqNum <= PMIC_IRQ_MAX)
    {
        // IRQs 0-31 are assigned to index 0, IRQs 32-63 are assigned to index 1.
        // At an index, the IRQ is stored at its corresponding bit position
        // (e.g., IRQ 49's status will be stored at bit 17 at index 1)
        irqStat->intrStat[irqNum / PMIC_NUM_BITS_IN_INTR_STAT] |= ((uint32_t)1U << (irqNum % PMIC_NUM_BITS_IN_INTR_STAT));
    }
}

static inline void IRQ_setIntrBits(Pmic_IrqStat_t *irqStat, uint8_t regData, const uint8_t irqs[], uint8_t numIrqs)
{
    for (uint8_t i = 0U; i < numIrqs; i++)
    {
        if (Pmic_getBitField_b(regData, pmicIRQs[irqs[i]].bitShift))
        {
            IRQ_setIntrStat(irqStat, irqs[i]);
        }
    }
}

static int32_t IRQ_readVmonStat(const Pmic_CoreHandle_t *pmicHandle, Pmic_IrqStat_t *irqStat)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;
    const uint8_t vmonStat1Irq[] = {
        PMIC_IRQ_VBATL_OV,
        PMIC_IRQ_VBATL_UV,
        PMIC_IRQ_VCP_UV
    };
    const uint8_t vmonStat2Irq[] = {
        PMIC_IRQ_VDD6_OV,
        PMIC_IRQ_VDD6_UV,
        PMIC_IRQ_VDD5_OV,
        PMIC_IRQ_VDD5_UV,
        PMIC_IRQ_VDD_3P5_OV,
        PMIC_IRQ_VDD_3P5_UV
    };
    const uint8_t vmonStat3Irq[] = {
        PMIC_IRQ_VREG_UV,
        PMIC_IRQ_VDD6_LP_UV,
        PMIC_IRQ_VSOUT2_OV,
        PMIC_IRQ_VSOUT2_UV,
        PMIC_IRQ_VSOUT1_OV,
        PMIC_IRQ_VSOUT1_UV
    };

    // Read VMON_STAT_1 register
    status = Pmic_ioRxByte_CS(pmicHandle, PMIC_CMD_RD_VMON_STAT_1, &regData);

    if (status == PMIC_ST_SUCCESS)
    {
        // Set the corresponding bits in irqStat
        IRQ_setIntrBits(irqStat, regData, vmonStat1Irq, COUNT(vmonStat1Irq));

        // Read VMON_STAT_2 register
        status = Pmic_ioRxByte_CS(pmicHandle, PMIC_CMD_RD_VMON_STAT_2, &regData);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Set the corresponding bits in irqStat
        IRQ_setIntrBits(irqStat, regData, vmonStat2Irq, COUNT(vmonStat2Irq));

        // Read VMON_STAT_3 register
        status = Pmic_ioRxByte_CS(pmicHandle, PMIC_CMD_RD_VMON_STAT_3, &regData);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Set the corresponding bits in irqStat
        IRQ_setIntrBits(irqStat, regData, vmonStat3Irq, COUNT(vmonStat3Irq));
    }

    return status;
}

static int32_t IRQ_readSafetyStat1_2(const Pmic_CoreHandle_t *pmicHandle, Pmic_IrqStat_t *irqStat)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;
    const uint8_t safetyStat1Irq[] = {
        PMIC_IRQ_VDD6_OT_FLAG,
        PMIC_IRQ_VDD6_TP,
        PMIC_IRQ_VDD5_ILIM,
        PMIC_IRQ_VDD5_OT_FLAG,
        PMIC_IRQ_VDD5_TP,
        PMIC_IRQ_VDD_3P5_ILIM,
        PMIC_IRQ_VDD_3P5_OT_FLAG,
        PMIC_IRQ_VDD_3P5_TP
    };
    const uint8_t safetyStat2Irq[] = {
        PMIC_IRQ_VDD6_ILIM,
        PMIC_IRQ_VSOUT2_ILIM,
        PMIC_IRQ_VSOUT2_OT_FLAG,
        PMIC_IRQ_VSOUT2_TP,
        PMIC_IRQ_VSOUT1_ILIM,
        PMIC_IRQ_VSOUT1_OT_FLAG,
        PMIC_IRQ_VSOUT1_TP
    };

    // Read SAFETY_STAT_1 register
    status = Pmic_ioRxByte_CS(pmicHandle, PMIC_CMD_RD_SAFETY_STAT_1, &regData);

    if (status == PMIC_ST_SUCCESS)
    {
        // Set the corresponding bits in irqStat
        IRQ_setIntrBits(irqStat, regData, safetyStat1Irq, COUNT(safetyStat1Irq));

        // Read SAFETY_STAT_2 register
        status = Pmic_ioRxByte_CS(pmicHandle, PMIC_CMD_RD_SAFETY_STAT_2, &regData);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Set the corresponding bits in irqStat
        IRQ_setIntrBits(irqStat, regData, safetyStat2Irq, COUNT(safetyStat2Irq));
    }

    return status;
}

static int32_t IRQ_readSafetyStat3_4(const Pmic_CoreHandle_t *pmicHandle, Pmic_IrqStat_t *irqStat)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;
    const uint8_t safetyStat3Irq[] = {
        PMIC_IRQ_CFG_CRC_ERR,
        PMIC_IRQ_EE_CRC_ERR,
        PMIC_IRQ_NRES_ERR,
        PMIC_IRQ_LBIST_ERR,
        PMIC_IRQ_ABIST_ERR
    };
    const uint8_t safetyStat4Irq[] = {
        PMIC_IRQ_LO_LPMCLK,
        PMIC_IRQ_ENDRV_ERR,
        PMIC_IRQ_TRIM_ERR_VMON
    };

    // Read SAFETY_STAT_3 register
    status = Pmic_ioRxByte_CS(pmicHandle, PMIC_CMD_RD_SAFETY_STAT_3, &regData);

    if (status == PMIC_ST_SUCCESS)
    {
        // Set the corresponding bits in irqStat
        IRQ_setIntrBits(irqStat, regData, safetyStat3Irq, COUNT(safetyStat3Irq));

        // Read SAFETY_STAT_4 register
        status = Pmic_ioRxByte_CS(pmicHandle, PMIC_CMD_RD_SAFETY_STAT_4, &regData);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Set the corresponding bits in irqStat
        IRQ_setIntrBits(irqStat, regData, safetyStat4Irq, COUNT(safetyStat4Irq));

        // Special case; certain SPI status errors share the same bit field
        switch (Pmic_getBitField(regData, SPI_ERR_SHIFT, SPI_ERR_MASK))
        {
            case SPI_ERR_CMD:
            {
                IRQ_setIntrStat(irqStat, PMIC_IRQ_SPI_CMD_ERR);
                break;
            }
            case SPI_ERR_FORMAT:
            {
                IRQ_setIntrStat(irqStat, PMIC_IRQ_SPI_FORMAT_ERR);
                break;
            }
            case SPI_ERR_OUTPUT_MISMATCH:
            {
                IRQ_setIntrStat(irqStat, PMIC_IRQ_SPI_DATA_OUTPUT_MISMATCH);
                break;
            }
            default:
            {
                /* No error */
                break;
            }
        }
    }

    return status;
}

static int32_t IRQ_readSafetyStat(const Pmic_CoreHandle_t *pmicHandle, Pmic_IrqStat_t *irqStat)
{
    int32_t status = IRQ_readSafetyStat1_2(pmicHandle, irqStat);

    if (status == PMIC_ST_SUCCESS)
    {
        status = IRQ_readSafetyStat3_4(pmicHandle, irqStat);
    }

    return status;
}

static int32_t IRQ_readSafetyErrStat(const Pmic_CoreHandle_t *pmicHandle, Pmic_IrqStat_t *irqStat)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;
    const uint8_t safetyErrStat1Irq[] = {
        PMIC_IRQ_ERROR_PIN_FAIL,
        PMIC_IRQ_WD_FAIL
    };

    // Read SAFETY_ERR_STAT_1 register
    status = Pmic_ioRxByte_CS(pmicHandle, PMIC_CMD_RD_SAFETY_ERR_STAT_1, &regData);

    if (status == PMIC_ST_SUCCESS)
    {
        // Set the corresponding bits in irqStat
        IRQ_setIntrBits(irqStat, regData, safetyErrStat1Irq, COUNT(safetyErrStat1Irq));

        // Read SAFETY_ERR_STAT_2 register
        status = Pmic_ioRxByte_CS(pmicHandle, PMIC_CMD_RD_SAFETY_ERR_STAT_2, &regData);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Set the corresponding bit in irqStat
        IRQ_setIntrStat(irqStat, PMIC_IRQ_DIAG_STATE_TO);
    }

    return status;
}

static int32_t IRQ_readSamStatus(const Pmic_CoreHandle_t *pmicHandle, Pmic_IrqStat_t *irqStat)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;
    const uint8_t samStatIrq[] = {
        PMIC_IRQ_SAM_BIST_FAIL,
        PMIC_IRQ_POWER_ON_RESET,
        PMIC_IRQ_SAM_CNT_ERR
    };
    const uint8_t samSigStatIrq[] = {
        PMIC_IRQ_COS_P_OV,
        PMIC_IRQ_COS_P_UV,
        PMIC_IRQ_COS_N_OV,
        PMIC_IRQ_COS_N_UV,
        PMIC_IRQ_SIN_P_OV,
        PMIC_IRQ_SIN_P_UV,
        PMIC_IRQ_SIN_N_OV,
        PMIC_IRQ_SIN_N_UV
    };

    // Read SAM_STAT register
    status = Pmic_ioRxByte_CS(pmicHandle, PMIC_CMD_RD_SAM_STAT, &regData);

    if (status == PMIC_ST_SUCCESS)
    {
        // Set the corresponding bit in irqStat
        IRQ_setIntrBits(irqStat, regData, samStatIrq, COUNT(samStatIrq));

        // Read SAM_SIG_STAT register
        status = Pmic_ioRxByte_CS(pmicHandle, PMIC_CMD_RD_SAM_SIG_STAT, &regData);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Set the corresponding bit in irqStat
        IRQ_setIntrBits(irqStat, regData, samSigStatIrq, COUNT(samSigStatIrq));
    }

    return status;
}

static int32_t IRQ_readSpiInvTranStat(const Pmic_CoreHandle_t *pmicHandle, Pmic_IrqStat_t *irqStat)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;
    const uint8_t spiInvTranStatIrq[] = {
        PMIC_IRQ_INVALID_CMD,
        PMIC_IRQ_UNDEF_CMD,
        PMIC_IRQ_CRC_ERR,
        PMIC_IRQ_LONG_FRM,
        PMIC_IRQ_SHORT_FRM
    };

    // Read SPI_INV_TRAN_STAT register
    status = Pmic_ioRxByte_CS(pmicHandle, PMIC_CMD_RD_SPI_INV_TRAN_STAT, &regData);

    if (status == PMIC_ST_SUCCESS)
    {
        // Set the corresponding bit in irqStat
        IRQ_setIntrBits(irqStat, regData, spiInvTranStatIrq, COUNT(spiInvTranStatIrq));
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
        memset(irqStat->intrStat, 0U, PMIC_NUM_ELEM_IN_INTR_STAT * sizeof(irqStat->intrStat[0U]));

        status = IRQ_readVmonStat(pmicHandle, irqStat);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        status = IRQ_readSafetyStat(pmicHandle, irqStat);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        status = IRQ_readSafetyErrStat(pmicHandle, irqStat);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        status = IRQ_readSamStatus(pmicHandle, irqStat);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        status = IRQ_readSpiInvTranStat(pmicHandle, irqStat);
    }

    // Clear irqStat if there was an error at any point
    if ((status != PMIC_ST_SUCCESS) && (irqStat != NULL))
    {
        memset(irqStat->intrStat, 0U, PMIC_NUM_ELEM_IN_INTR_STAT * sizeof(irqStat->intrStat[0U]));
    }

    return status;
}

static uint8_t IRQ_getNextFlag(Pmic_IrqStat_t *irqStat)
{
    uint8_t index = 0U, bitPos = 0U;
    bool foundFlag = (bool)false;

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
                foundFlag = (bool)true;
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
        status = Pmic_ioRxByte_CS(pmicHandle, pmicIRQs[irqNum].regCmd, &regData);
    }

    // Extract IRQ status
    if (status == PMIC_ST_SUCCESS)
    {
        // Special case; certain SPI status errors share the same bit field
        if ((irqNum >= PMIC_IRQ_SPI_CMD_ERR) && (irqNum <= PMIC_IRQ_SPI_DATA_OUTPUT_MISMATCH))
        {
            uint8_t spiErr = Pmic_getBitField(regData, SPI_ERR_SHIFT, SPI_ERR_MASK);

            if (((irqNum == PMIC_IRQ_SPI_CMD_ERR) && (spiErr == SPI_ERR_CMD)) ||
                ((irqNum == PMIC_IRQ_SPI_FORMAT_ERR) && (spiErr == SPI_ERR_FORMAT)) ||
                ((irqNum == PMIC_IRQ_SPI_DATA_OUTPUT_MISMATCH) && (spiErr == SPI_ERR_OUTPUT_MISMATCH)))
            {
                *flag = (bool)true;
            }
        }
        else
        {
            *flag = Pmic_getBitField_b(regData, pmicIRQs[irqNum].bitShift);
        }
    }

    return status;
}
