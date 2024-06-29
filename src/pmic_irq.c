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

/* ========================================================================== */
/*                                Include Files                               */
/* ========================================================================== */
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "pmic.h"
#include "pmic_common.h"
#include "pmic_io.h"
#include "pmic_irq.h"

#include "regmap/irq.h"

/* ========================================================================== */
/*                               Macros & Typedefs                            */
/* ========================================================================== */
#define PMIC_INVALID_REG        ((uint8_t)0xFFU)
#define NUM_MASKABLE_REGISTERS  (9U)
#define NUM_CLEARABLE_REGISTERS (13U)

// Used to hold information about IRQs, which register contains their status
// bit, and which register contains their mask bit, along with the relevant
// bitshift to use.
typedef struct Pmic_IrqInfo_s {
    uint8_t statReg;
    uint8_t maskReg;
    uint8_t shift;
} Pmic_IrqInfo_t;

// Tie Register addresses, bitshifts, and IRQ numbers together
static const Pmic_IrqInfo_t IRQ[PMIC_IRQ_NUM] = {
    { INT_BUCK_LDO_LS1_VMON1_REG, PMIC_INVALID_REG, BUCK1_SC_INT_SHIFT }, // 0
    { INT_BUCK_12_REG, MASK_BUCK_12_REG, BUCK1_OV_INT_SHIFT }, // 1
    { INT_BUCK_12_REG, MASK_BUCK_12_REG, BUCK1_UV_INT_SHIFT }, // 2
    { INT_BUCK_12_REG, MASK_BUCK_12_REG, BUCK1_RV_INT_SHIFT }, // 3
    { INT_BUCK_12_REG, MASK_BUCK_12_REG, BUCK1_ILIM_INT_SHIFT }, // 4
    { INT_BUCK_LDO_LS1_VMON1_REG, PMIC_INVALID_REG, BUCK2_SC_INT_SHIFT }, // 5
    { INT_BUCK_12_REG, MASK_BUCK_12_REG, BUCK2_OV_INT_SHIFT }, // 6
    { INT_BUCK_12_REG, MASK_BUCK_12_REG, BUCK2_UV_INT_SHIFT }, // 7
    { INT_BUCK_12_REG, MASK_BUCK_12_REG, BUCK2_RV_INT_SHIFT }, // 8
    { INT_BUCK_12_REG, MASK_BUCK_12_REG, BUCK2_ILIM_INT_SHIFT }, // 9
    { INT_BUCK_LDO_LS1_VMON1_REG, PMIC_INVALID_REG, BUCK3_SC_INT_SHIFT }, // 10
    { INT_BUCK3_LDO_LS1_VMON1_REG, MASK_BUCK3_LDO_LS1_VMON1_REG, BUCK3_OV_INT_SHIFT }, // 11
    { INT_BUCK3_LDO_LS1_VMON1_REG, MASK_BUCK3_LDO_LS1_VMON1_REG, BUCK3_UV_INT_SHIFT }, // 12
    { INT_BUCK3_LDO_LS1_VMON1_REG, MASK_BUCK3_LDO_LS1_VMON1_REG, BUCK3_RV_INT_SHIFT }, // 13
    { INT_BUCK3_LDO_LS1_VMON1_REG, MASK_BUCK3_LDO_LS1_VMON1_REG, BUCK3_ILIM_INT_SHIFT }, // 14
    { INT_BUCK_LDO_LS1_VMON1_REG, PMIC_INVALID_REG, LDO_LS1_VMON1_SC_INT_SHIFT }, // 15
    { INT_BUCK3_LDO_LS1_VMON1_REG, MASK_BUCK3_LDO_LS1_VMON1_REG, LDO_LS1_VMON1_OV_INT_SHIFT }, // 16
    { INT_BUCK3_LDO_LS1_VMON1_REG, MASK_BUCK3_LDO_LS1_VMON1_REG, LDO_LS1_VMON1_UV_INT_SHIFT }, // 17
    { INT_BUCK3_LDO_LS1_VMON1_REG, MASK_BUCK3_LDO_LS1_VMON1_REG, LDO_LS1_VMON1_RV_INT_SHIFT }, // 18
    { INT_BUCK3_LDO_LS1_VMON1_REG, MASK_BUCK3_LDO_LS1_VMON1_REG, LDO_LS1_VMON1_ILIM_INT_SHIFT }, // 19
    { INT_LS2_VMON2_REG, MASK_LS2_VMON2_REG, LS2_VMON2_SC_NMI_SHIFT }, // 20
    { INT_LS2_VMON2_REG, MASK_LS2_VMON2_REG, LS2_VMON2_OV_INT_SHIFT }, // 21
    { INT_LS2_VMON2_REG, MASK_LS2_VMON2_REG, LS2_VMON2_UV_INT_SHIFT }, // 22
    { INT_LS2_VMON2_REG, MASK_LS2_VMON2_REG, LS2_VMON2_RV_INT_SHIFT }, // 23
    { INT_LS2_VMON2_REG, MASK_LS2_VMON2_REG, LS2_VMON2_ILIM_INT_SHIFT }, // 24
    { INT_VCCA_REG, MASK_VCCA_REG, VCCA_OV_INT_SHIFT }, // 25
    { INT_VCCA_REG, MASK_VCCA_REG, VCCA_UV_INT_SHIFT }, // 26
    { INT_STARTUP_REG, MASK_STARTUP_REG, ENABLE_INT_SHIFT }, // 27
    { INT_MISC_REG, MASK_MISC_REG, ABIST_FAIL_INT_SHIFT }, // 28
    { INT_MISC_REG, MASK_MISC_REG, BUCKS_VSET_ERR_INT_SHIFT }, // 29
    { INT_MISC_REG, MASK_MISC_REG, EXT_CLK_INT_SHIFT }, // 30
    { INT_MISC_REG, MASK_MISC_REG, TWARN_INT_SHIFT }, // 31
    { INT_MODERATE_ERR_REG, MASK_MODERATE_ERR_REG, TSD_ORD_NMI_SHIFT }, // 32
    { INT_MODERATE_ERR_REG, MASK_MODERATE_ERR_REG, RECOV_CNT_NMI_SHIFT }, // 33
    { INT_MODERATE_ERR_REG, MASK_MODERATE_ERR_REG, TRIM_TEST_CRC_INT_SHIFT }, // 34
    { INT_MODERATE_ERR_REG, MASK_MODERATE_ERR_REG, CONFIG_CRC_INT_SHIFT }, // 35
    { INT_MODERATE_ERR_REG, MASK_MODERATE_ERR_REG, NINT_GPO_RDBK_INT_SHIFT }, // 36
    { INT_MODERATE_ERR_REG, MASK_MODERATE_ERR_REG, NRSTOUT_RDBK_INT_SHIFT }, // 37
    { INT_SEVERE_ERR_REG, PMIC_INVALID_REG, TSD_IMM_NMI_SHIFT }, // 38
    { INT_SEVERE_ERR_REG, PMIC_INVALID_REG, VCCA_OVP_NMI_SHIFT }, // 39
    { INT_COMM_ERR_REG, MASK_COMM_ERR_REG, COMM_FRM_ERR_INT_SHIFT }, // 40
    { INT_COMM_ERR_REG, MASK_COMM_ERR_REG, COMM_CRC_ERR_INT_SHIFT }, // 41
    { INT_COMM_ERR_REG, MASK_COMM_ERR_REG, COMM_ADR_ERR_INT_SHIFT }, // 42
    { INT_COMM_ERR_REG, MASK_COMM_ERR_REG, COMM_MCU_ERR_INT_SHIFT }, // 43
    { INT_ESM_REG, MASK_ESM_REG, ESM_MCU_PIN_INT_SHIFT }, // 44
    { INT_ESM_REG, MASK_ESM_REG, ESM_MCU_FAIL_INT_SHIFT }, // 45
    { INT_ESM_REG, MASK_ESM_REG, ESM_MCU_RST_INT_SHIFT }, // 46
    { INT_FSM_ERR_REG, PMIC_INVALID_REG, WD_FIRST_NOK_INT_SHIFT }, // 47
    { WD_ERR_STAT_REG, PMIC_INVALID_REG, WD_LONGWIN_TIMEOUT_INT_SHIFT }, // 48
    { WD_ERR_STAT_REG, PMIC_INVALID_REG, WD_TIMEOUT_INT_SHIFT }, // 49
    { WD_ERR_STAT_REG, PMIC_INVALID_REG, WD_ANSWER_EARLY_INT_SHIFT }, // 50
    { WD_ERR_STAT_REG, PMIC_INVALID_REG, WD_SEQ_ERR_INT_SHIFT }, // 51
    { WD_ERR_STAT_REG, PMIC_INVALID_REG, WD_ANSWER_ERR_INT_SHIFT }, // 52
    { WD_ERR_STAT_REG, PMIC_INVALID_REG, WD_FAIL_INT_SHIFT }, // 53
    { WD_ERR_STAT_REG, PMIC_INVALID_REG, WD_RST_INT_SHIFT }, // 54
    { INT_FSM_ERR_REG, PMIC_INVALID_REG, REGULATOR_ERR_INT_SHIFT }, // 55
    { INT_FSM_ERR_REG, PMIC_INVALID_REG, IMM_SHUTDOWN_INT_SHIFT }, // 56
    { INT_FSM_ERR_REG, PMIC_INVALID_REG, ORD_SHUTDOWN_INT_SHIFT }, // 57
    { INT_FSM_ERR_REG, PMIC_INVALID_REG, WARM_RESET_INT_SHIFT }, // 58
};

static const uint8_t MaskableRegisters[NUM_MASKABLE_REGISTERS] = {
    MASK_BUCK_12_REG,
    MASK_BUCK3_LDO_LS1_VMON1_REG,
    MASK_LS2_VMON2_REG,
    MASK_VCCA_REG,
    MASK_STARTUP_REG,
    MASK_MISC_REG,
    MASK_MODERATE_ERR_REG,
    MASK_COMM_ERR_REG,
    MASK_ESM_REG,
};

static const uint8_t ClearableRegisters[NUM_CLEARABLE_REGISTERS] = {
    INT_BUCK_LDO_LS1_VMON1_REG,
    INT_BUCK_12_REG,
    INT_BUCK3_LDO_LS1_VMON1_REG,
    INT_LS2_VMON2_REG,
    INT_VCCA_REG,
    INT_STARTUP_REG,
    INT_MISC_REG,
    INT_MODERATE_ERR_REG,
    INT_SEVERE_ERR_REG,
    INT_FSM_ERR_REG,
    INT_COMM_ERR_REG,
    INT_ESM_REG,
    WD_ERR_STAT_REG,
};

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

static int32_t IRQ_setMask(Pmic_CoreHandle_t *handle, uint8_t irqNum, bool shouldMask) {
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    uint8_t irqMaskRegAddr = 0U, irqMaskBitShift = 0U;

    // Check for invalid IRQ number
    if (irqNum > PMIC_IRQ_MAX) {
        status = PMIC_ST_ERR_INV_PARAM;
    } else {
        irqMaskRegAddr = IRQ[irqNum].maskReg;
        irqMaskBitShift = IRQ[irqNum].shift;
    }

    // Check whether IRQ is maskable
    if ((status == PMIC_ST_SUCCESS) &&
        (irqMaskRegAddr == PMIC_INVALID_REG)) {
        status = PMIC_ST_ERR_NOT_SUPPORTED;
    }

    // Read IRQ mask register
    if (status == PMIC_ST_SUCCESS) {
        Pmic_criticalSectionStart(handle);
        status = Pmic_ioRxByte(handle, irqMaskRegAddr, &regData);
        Pmic_criticalSectionStop(handle);
    }

    if (status == PMIC_ST_SUCCESS) {
        // Modify IRQ mask bit field
        Pmic_setBitField_b(&regData, irqMaskBitShift, shouldMask);

        // Write new register value back to PMIC
        Pmic_criticalSectionStart(handle);
        status = Pmic_ioTxByte(handle, irqMaskRegAddr, regData);
        Pmic_criticalSectionStop(handle);
    }

    return status;
}

static inline bool IRQ_anyMasksForReg(uint8_t numMasks, const Pmic_IrqMask_t *masks, uint8_t regAddr) {
    bool anyRegs = false;

    for (uint8_t i = 0U; i < numMasks || anyRegs; i++) {
        const uint8_t irqNum = masks[i].irqNum;
        anyRegs = (IRQ[irqNum].maskReg == regAddr);
    }

    return anyRegs;
}

static int32_t IRQ_handleRecordsForReg(Pmic_CoreHandle_t *handle,
                                       uint8_t numMasks,
                                       const Pmic_IrqMask_t *masks,
                                       uint8_t regAddr,
                                       uint8_t *processedMasks)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    const bool anyMasks = IRQ_anyMasksForReg(numMasks, masks, regAddr);

    if (anyMasks) {
        // Get the current value of this IRQ mask register
        Pmic_criticalSectionStart(handle);
        status = Pmic_ioRxByte(handle, regAddr, &regData);
        Pmic_criticalSectionStop(handle);
    }

    if ((status == PMIC_ST_SUCCESS) && anyMasks) {
        for (uint8_t i = 0U; i < numMasks; i++) {
            const uint8_t irqNum = masks[i].irqNum;
            const uint8_t userMask = (uint8_t)(1U << IRQ[irqNum].shift);

            // If the current mask setting isn't targeted at the register we are
            // currently building, skip it
            if (regAddr != IRQ[irqNum].maskReg) {
                continue;
            }

            if (masks[i].mask) {
                regData |= userMask;
            } else {
                regData &= ~userMask;
            }

            // Increment the counter indicating we processed this record
            *processedMasks += 1;
        }
    }

    // If status is still good and we did find records that apply to this
    // register, write the new value of this register back to the device
    if ((status == PMIC_ST_SUCCESS) && *processedMasks > 0) {
        Pmic_criticalSectionStart(handle);
        status = Pmic_ioTxByte(handle, regAddr, regData);
        Pmic_criticalSectionStop(handle);
    }

    return status;
}

int32_t Pmic_irqSetMask(Pmic_CoreHandle_t *handle, uint8_t irqNum, bool shouldMask) {
    int32_t status = Pmic_checkPmicCoreHandle(handle);

    if (status == PMIC_ST_SUCCESS) {
        status = IRQ_setMask(handle, irqNum, shouldMask);
    }

    return status;
}

int32_t Pmic_irqSetMasks(Pmic_CoreHandle_t *handle, uint8_t numMasks, const Pmic_IrqMask_t *masks) {
    int32_t status = Pmic_checkPmicCoreHandle(handle);
    uint8_t lastProcessed = 0U, totalProcessed = 0U;

    if ((status == PMIC_ST_SUCCESS) && (masks == NULL)) {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) && (numMasks > PMIC_IRQ_NUM)) {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    for (uint8_t regIndex = 0U; regIndex < NUM_MASKABLE_REGISTERS; regIndex++) {
        // If status is no longer good or all user requested masks have been
        // processed, we can stop iterating
        if ((status != PMIC_ST_SUCCESS) || (totalProcessed >= numMasks)) {
            break;
        }

        if (status == PMIC_ST_SUCCESS) {
            status = IRQ_handleRecordsForReg(handle, numMasks, masks, MaskableRegisters[regIndex], &lastProcessed);
            totalProcessed += lastProcessed;
        }
    }

    return status;
}

int32_t Pmic_irqGetMask(Pmic_CoreHandle_t *handle, uint8_t numIrqMasks, Pmic_IrqMask_t *irqMasks) {
    int32_t status = Pmic_checkPmicCoreHandle(handle);
    uint8_t regData = 0U;

    if ((status == PMIC_ST_SUCCESS) && (irqMasks == NULL)) {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) && (numIrqMasks > (PMIC_IRQ_MAX + 1U))) {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    if (status == PMIC_ST_SUCCESS) {
        for (uint8_t i = 0U; i < numIrqMasks; i++) {
            const uint8_t irqNum = irqMasks[i].irqNum;
            uint8_t irqMaskRegAddr = 0U, irqMaskBitShift = 0U;

            // Check for invalid IRQ number
            if (irqNum > PMIC_IRQ_MAX) {
                status = PMIC_ST_ERR_INV_PARAM;
            } else {
                irqMaskRegAddr = IRQ[irqNum].maskReg;
                irqMaskBitShift = IRQ[irqNum].shift;
            }

            // Check whether IRQ is maskable
            if ((status == PMIC_ST_SUCCESS) &&
                (irqMaskRegAddr == PMIC_INVALID_REG)) {
                status = PMIC_ST_ERR_NOT_SUPPORTED;
            }

            // Read IRQ mask register
            if (status == PMIC_ST_SUCCESS) {
                Pmic_criticalSectionStart(handle);
                status = Pmic_ioRxByte(handle, irqMaskRegAddr, &regData);
                Pmic_criticalSectionStop(handle);
            }

            if (status == PMIC_ST_SUCCESS) {
                // Extract IRQ mask bit field
                irqMasks[i].mask = Pmic_getBitField_b(regData, irqMaskBitShift);
            } else {
                break;
            }
        }
    }

    return status;
}

static inline void IRQ_extractBits(Pmic_IrqStat_t *irqStat, uint8_t regData, const uint8_t irqs[], uint8_t numIrqs) {
    for (uint8_t i = 0; i < numIrqs; i++) {
        if (Pmic_getBitField_b(regData, IRQ[irqs[i]].shift)) {
            IRQ_setIntrStat(irqStat, irqs[i]);
        }
    }
}

// NOTE: This function should only be called from within a critical section
static int32_t IRQ_getStatFSM(Pmic_CoreHandle_t *handle, Pmic_IrqStat_t *irqStat) {
    uint8_t regData = 0U;
    int32_t status = Pmic_ioRxByte(handle, INT_FSM_ERR_REG, &regData);
    bool esmMcuInt, commErrInt, wdInt;

    // Top level IRQ numbers for INT_FSM_ERR
    const uint8_t fsmIrqs[] = {
        PMIC_FSM_IMM_SHUTDOWN_INT,
        PMIC_FSM_ORD_SHUTDOWN_INT,
        PMIC_FSM_WARM_RESET_INT,
        PMIC_REGULATOR_ERR_INT,
        PMIC_WDG_FIRST_NOK_INT,
    };

    // Top level IRQ numbers for INT_COMM_ERR
    const uint8_t commIrqs[] = {
        PMIC_COMM_FRM_ERR_INT,
        PMIC_COMM_CRC_ERR_INT,
        PMIC_COMM_ADR_ERR_INT,
        PMIC_COMM_MCU_ERR_INT,
    };

    // Top level IRQ numbers for INT_ESM
    const uint8_t esmIrqs[] = {
        PMIC_ESM_MCU_PIN_INT,
        PMIC_ESM_MCU_FAIL_INT,
        PMIC_ESM_MCU_RST_INT,
    };

    // Top level IRQ numbers for WD_ERR_STAT
    const uint8_t wdIrq[] = {
        PMIC_WDG_LONGWIN_TIMEOUT_INT,
        PMIC_WDG_TIMEOUT_INT,
        PMIC_WDG_ANSWER_EARLY_INT,
        PMIC_WDG_SEQ_ERR_INT,
        PMIC_WDG_ANSWER_ERR_INT,
        PMIC_WDG_FAIL_INT,
        PMIC_WDG_RST_INT,
    };

    // Parse L0 bits from INT_FSM_ERR
    if (status == PMIC_ST_SUCCESS) {
        IRQ_extractBits(irqStat, regData, fsmIrqs, COUNT(fsmIrqs));
    }

    // Extract L1 indicator bits from regData so it can be repurposed
    esmMcuInt = Pmic_getBitField_b(regData, ESM_MCU_INT_SHIFT);
    commErrInt = Pmic_getBitField_b(regData, COMM_ERR_INT_SHIFT);
    wdInt = Pmic_getBitField_b(regData, WD_INT_SHIFT);

    // If ESM_MCU_INT is set, read and extract bits from L1 register INT_ESM
    if ((status == PMIC_ST_SUCCESS) && esmMcuInt) {
        status = Pmic_ioRxByte(handle, INT_ESM_REG, &regData);

        if (status == PMIC_ST_SUCCESS) {
            IRQ_extractBits(irqStat, regData, esmIrqs, COUNT(esmIrqs));
        }
    }

    // If COMM_ERR_INT is set, read and extract bits from L1 register INT_COMM_ERR
    if ((status == PMIC_ST_SUCCESS) && commErrInt) {
        status = Pmic_ioRxByte(handle, INT_COMM_ERR_REG, &regData);

        if (status == PMIC_ST_SUCCESS) {
            IRQ_extractBits(irqStat, regData, commIrqs, COUNT(commIrqs));
        }
    }

    // If WD_INT is set, read and extract bits from L1 register WD_ERR_STAT
    if ((status == PMIC_ST_SUCCESS) && wdInt) {
        status = Pmic_ioRxByte(handle, WD_ERR_STAT_REG, &regData);

        if (status == PMIC_ST_SUCCESS) {
            IRQ_extractBits(irqStat, regData, wdIrq, COUNT(wdIrq));
        }
    }

    return status;
}

// NOTE: This function should only be called from within a critical section
static int32_t IRQ_getStatSevere(Pmic_CoreHandle_t *handle, Pmic_IrqStat_t *irqStat) {
    uint8_t regData = 0U;
    int32_t status = Pmic_ioRxByte(handle, INT_SEVERE_ERR_REG, &regData);

    const uint8_t irqs[] = {PMIC_SE_TSD_IMM_INT, PMIC_SE_VCCA_OVP_INT};

    if (status == PMIC_ST_SUCCESS) {
        IRQ_extractBits(irqStat, regData, irqs, COUNT(irqs));
    }

    return status;
}

// NOTE: This function should only be called from within a critical section
static int32_t IRQ_getStatModerate(Pmic_CoreHandle_t *handle, Pmic_IrqStat_t *irqStat) {
    uint8_t regData = 0U;
    int32_t status = Pmic_ioRxByte(handle, INT_MODERATE_ERR_REG, &regData);

    const uint8_t irqs[] = {
        PMIC_ME_TSD_ORD_INT,
        PMIC_ME_RECOV_CNT_INT,
        PMIC_ME_TRIM_TEST_CRC_INT,
        PMIC_ME_CONFIG_CRC_INT,
        PMIC_ME_NINT_READBACK_INT,
        PMIC_ME_NRSTOUT_READBACK_INT,
    };

    if (status == PMIC_ST_SUCCESS) {
        IRQ_extractBits(irqStat, regData, irqs, COUNT(irqs));
    }

    return status;
}

// NOTE: This function should only be called from within a critical section
static int32_t IRQ_getStatMisc(Pmic_CoreHandle_t *handle, Pmic_IrqStat_t *irqStat) {
    uint8_t regData = 0U;
    int32_t status = Pmic_ioRxByte(handle, INT_MISC_REG, &regData);

    const uint8_t irqs[] = {
        PMIC_MISC_ABIST_FAIL_INT,
        PMIC_MISC_BUCKS_VSET_ERR_INT,
        PMIC_MISC_EXT_CLK_INT,
        PMIC_MISC_TWARN_INT,
    };

    if (status == PMIC_ST_SUCCESS) {
        IRQ_extractBits(irqStat, regData, irqs, COUNT(irqs));
    }

    return status;
}

// NOTE: This function should only be called from within a critical section
static int32_t IRQ_getStatStartup(Pmic_CoreHandle_t *handle, Pmic_IrqStat_t *irqStat) {
    uint8_t regData = 0U;
    int32_t status = Pmic_ioRxByte(handle, INT_STARTUP_REG, &regData);

    if (Pmic_getBitField_b(regData, IRQ[PMIC_STARTUP_ENABLE_INT].shift)) {
        IRQ_setIntrStat(irqStat, PMIC_STARTUP_ENABLE_INT);
    }

    return status;
}

// NOTE: This function should only be called from within a critical section
static int32_t IRQ_getStatVccaVmon1(Pmic_CoreHandle_t *handle, Pmic_IrqStat_t *irqStat) {
    uint8_t regData = 0U;
    int32_t status = Pmic_ioRxByte(handle, INT_VCCA_REG, &regData);

    const uint8_t irqs[] = {PMIC_VCCA_OV_INT, PMIC_VCCA_UV_INT};

    if (status == PMIC_ST_SUCCESS) {
        IRQ_extractBits(irqStat, regData, irqs, COUNT(irqs));
    }

    return status;
}

// NOTE: This function should only be called from within a critical section
static int32_t IRQ_getStatLs2Vmon2(Pmic_CoreHandle_t *handle, Pmic_IrqStat_t *irqStat) {
    uint8_t regData = 0U;
    int32_t status = Pmic_ioRxByte(handle, INT_LS2_VMON2_REG, &regData);

    const uint8_t irqs[] = {
        PMIC_LS2_VMON2_SC_INT,
        PMIC_LS2_VMON2_OV_INT,
        PMIC_LS2_VMON2_UV_INT,
        PMIC_LS2_VMON2_RV_INT,
    };

    if (status == PMIC_ST_SUCCESS) {
        IRQ_extractBits(irqStat, regData, irqs, COUNT(irqs));
    }

    return status;
}

// NOTE: This function should only be called from within a critical section
static int32_t IRQ_getStatBucks(Pmic_CoreHandle_t *handle, Pmic_IrqStat_t *irqStat) {
    uint8_t regData = 0U;
    int32_t status = Pmic_ioRxByte(handle, INT_BUCK_LDO_LS1_VMON1_REG, &regData);
    bool buck12, buck3Ldo;

    const uint8_t topIrqs[] = {
        PMIC_BUCK1_SC_NMI,
        PMIC_BUCK2_SC_NMI,
        PMIC_BUCK3_SC_NMI,
        PMIC_LDO_LS1_VMON1_SC_INT,
    };

    const uint8_t buck12Irqs[] = {
        PMIC_BUCK1_OV_INT,
        PMIC_BUCK1_UV_INT,
        PMIC_BUCK1_RV_INT,
        PMIC_BUCK1_ILIM_INT,
        PMIC_BUCK2_OV_INT,
        PMIC_BUCK2_UV_INT,
        PMIC_BUCK2_RV_INT,
        PMIC_BUCK2_ILIM_INT,
    };

    const uint8_t buck3LdoIrqs[] = {
        PMIC_BUCK3_OV_INT,
        PMIC_BUCK3_UV_INT,
        PMIC_BUCK3_RV_INT,
        PMIC_BUCK3_ILIM_INT,
        PMIC_LDO_LS1_VMON1_OV_INT,
        PMIC_LDO_LS1_VMON1_UV_INT,
        PMIC_LDO_LS1_VMON1_RV_INT,
        PMIC_LDO_LS1_VMON1_ILIM_INT,
    };

    if (status == PMIC_ST_SUCCESS) {
        IRQ_extractBits(irqStat, regData, topIrqs, COUNT(topIrqs));
    }

    // Extract L1 indicator bits from regData so it can be repurposed
    buck12 = (Pmic_getBitField_b(regData, BUCK1_INT_SHIFT) ||
              Pmic_getBitField_b(regData, BUCK2_INT_SHIFT));
    buck3Ldo = (Pmic_getBitField_b(regData, BUCK3_INT_SHIFT) ||
                Pmic_getBitField_b(regData, LDO_LS1_VMON1_INT_SHIFT));

    // If BUCK1_INT or BUCK2_INT are set, read and extract bits from L1 register
    // INT_BUCK_12
    if ((status == PMIC_ST_SUCCESS) && buck12) {
        status = Pmic_ioRxByte(handle, INT_BUCK_12_REG, &regData);

        if (status == PMIC_ST_SUCCESS) {
            IRQ_extractBits(irqStat, regData, buck12Irqs, COUNT(buck12Irqs));
        }
    }

    // If BUCK3_INT or LDO_LS2_VMON1_INT are set, read and extract bits from L1
    // register INT_BUCK3_LDO_LS1_VMON1
    if ((status == PMIC_ST_SUCCESS) && buck3Ldo) {
        status = Pmic_ioRxByte(handle, INT_COMM_ERR_REG, &regData);

        if (status == PMIC_ST_SUCCESS) {
            IRQ_extractBits(irqStat, regData, buck3LdoIrqs, COUNT(buck3LdoIrqs));
        }
    }

    return status;
}

static int32_t IRQ_getStat(Pmic_CoreHandle_t *handle, Pmic_IrqStat_t *irqStat) {
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    // Clear register backing to ensure all data is fresh
    memset(irqStat, 0U, sizeof(Pmic_IrqStat_t));

    // Obtain critical section around all of these reads
    Pmic_criticalSectionStart(handle);

    status = Pmic_ioRxByte(handle, INT_TOP_REG, &regData);

    if ((status == PMIC_ST_SUCCESS) && Pmic_getBitField_b(regData, FSM_ERR_INT_SHIFT)) {
        status = IRQ_getStatFSM(handle, irqStat);
    }

    if ((status == PMIC_ST_SUCCESS) && Pmic_getBitField_b(regData, SEVERE_ERR_INT_SHIFT)) {
        status = IRQ_getStatSevere(handle, irqStat);
    }

    if ((status == PMIC_ST_SUCCESS) && Pmic_getBitField_b(regData, MODERATE_ERR_INT_SHIFT)) {
        status = IRQ_getStatModerate(handle, irqStat);
    }

    if ((status == PMIC_ST_SUCCESS) && Pmic_getBitField_b(regData, MISC_INT_SHIFT)) {
        status = IRQ_getStatMisc(handle, irqStat);
    }

    if ((status == PMIC_ST_SUCCESS) && Pmic_getBitField_b(regData, STARTUP_INT_SHIFT)) {
        status = IRQ_getStatStartup(handle, irqStat);
    }

    if ((status == PMIC_ST_SUCCESS) && Pmic_getBitField_b(regData, VCCA_INT_SHIFT)) {
        status = IRQ_getStatVccaVmon1(handle, irqStat);
    }

    if ((status == PMIC_ST_SUCCESS) && Pmic_getBitField_b(regData, LS2_VMON2_INT_SHIFT)) {
        status = IRQ_getStatLs2Vmon2(handle, irqStat);
    }

    if ((status == PMIC_ST_SUCCESS) && Pmic_getBitField_b(regData, BUCK_LDO_LS1_VMON1_INT_SHIFT)) {
        status = IRQ_getStatBucks(handle, irqStat);
    }

    // Release critical section
    Pmic_criticalSectionStop(handle);

    return status;
}

int32_t Pmic_irqGetStat(Pmic_CoreHandle_t *handle, Pmic_IrqStat_t *irqStat) {
    int32_t status = Pmic_checkPmicCoreHandle(handle);

    if ((status == PMIC_ST_SUCCESS) && (irqStat == NULL)) {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if (status == PMIC_ST_SUCCESS) {
        status = IRQ_getStat(handle, irqStat);
    }

    return status;
}

static uint8_t IRQ_getNextFlag(Pmic_IrqStat_t *irqStat) {
    uint8_t index = 0U, bitPos = 0U;
    bool foundFlag = false;

    // For each element in struct member intrStat of irqStat...
    for (index = 0U; index < PMIC_NUM_ELEM_IN_INTR_STAT; index++) {
        // If current element has no IRQ statuses set, move onto next element
        if (irqStat->intrStat[index] == 0U) {
            continue;
        }

        // For each bit in the element...
        for (bitPos = 0U; bitPos < PMIC_NUM_BITS_IN_INTR_STAT; bitPos++) {
            // If the bit is set...
            if ((irqStat->intrStat[index] & (1U << bitPos)) != 0U) {
                // Clear bit in intrStat element and exit loop
                irqStat->intrStat[index] &= ~(1U << bitPos);

                foundFlag = true;
                break;
            }
        }

        if (foundFlag) {
            break;
        }
    }

    // Return the corresponding IRQ number
    return (bitPos + (PMIC_NUM_BITS_IN_INTR_STAT * index));
}

int32_t Pmic_irqGetNextFlag(Pmic_IrqStat_t *irqStat, uint8_t *irqNum) {
    int32_t status = PMIC_ST_SUCCESS;

    const bool irqStatEmpty = ((irqStat->intrStat[0] == 0) && (irqStat->intrStat[1] == 0));

    if ((status == PMIC_ST_SUCCESS) &&
        ((irqStat == NULL) || (irqNum == NULL))) {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) && irqStatEmpty) {
        status = PMIC_ST_WARN_NO_IRQ_REMAINING;
    }

    if (status == PMIC_ST_SUCCESS) {
        *irqNum = IRQ_getNextFlag(irqStat);
    }

    return status;
}

int32_t Pmic_irqGetFlag(Pmic_CoreHandle_t *handle, uint8_t irqNum, bool *flag) {
    int32_t status = Pmic_checkPmicCoreHandle(handle);
    uint8_t regData = 0U;

    if ((status == PMIC_ST_SUCCESS) && (irqNum > PMIC_IRQ_MAX)) {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) && (flag == NULL)) {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    // Read IRQ status register
    if (status == PMIC_ST_SUCCESS) {
        Pmic_criticalSectionStart(handle);
        status = Pmic_ioRxByte(handle, IRQ[irqNum].statReg, &regData);
        Pmic_criticalSectionStop(handle);
    }

    // Extract IRQ status
    if (status == PMIC_ST_SUCCESS) {
        *flag = Pmic_getBitField_b(regData, IRQ[irqNum].shift);
    }

    return status;
}

int32_t Pmic_irqClrFlag(Pmic_CoreHandle_t *handle, uint8_t irqNum) {
    int32_t status = Pmic_checkPmicCoreHandle(handle);
    uint8_t regData = 0U;

    if ((status == PMIC_ST_SUCCESS) && (irqNum > PMIC_IRQ_MAX)) {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    if (status == PMIC_ST_SUCCESS) {
        // IRQ statuses are W1C - write 1 to clear
        Pmic_setBitField_b(&regData, IRQ[irqNum].shift, PMIC_ENABLE);

        // Write data to PMIC
        Pmic_criticalSectionStart(handle);
        status = Pmic_ioTxByte(handle, IRQ[irqNum].statReg, regData);
        Pmic_criticalSectionStop(handle);
    }

    return status;
}

int32_t Pmic_irqClrAllFlags(Pmic_CoreHandle_t *handle) {
    int32_t status = Pmic_checkPmicCoreHandle(handle);

    // All IRQ statuses are W1C, writing to reserved bits has no effect, so just
    // write every bit to 1
    Pmic_criticalSectionStart(handle);
    for (uint8_t i = 0; i < NUM_CLEARABLE_REGISTERS; i++) {
        if (status == PMIC_ST_SUCCESS) {
            status = Pmic_ioTxByte(handle, ClearableRegisters[i], 0xFFU);
        } else {
            break;
        }
    }
    Pmic_criticalSectionStop(handle);

    return status;
}
