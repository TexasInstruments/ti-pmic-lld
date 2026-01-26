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
#include <stdbool.h>
#include <string.h>

#include "pmic.h"
#include "pmic_common.h"
#include "pmic_io.h"
#include "pmic_irq.h"

#include "regmap/irq.h"

/* ========================================================================== */
/*                               Macros & Typedefs                            */
/* ========================================================================== */
#define NUM_MASKABLE_REGISTERS  (9U)
#define NUM_CLEARABLE_REGISTERS (13U)
#define PMIC_IRQ_LOOP_MAX       (59U)  // PMIC_IRQ_NUM = 59

#define CLEAR_ALL_STAT_BITS     (0xFFU)

#define PMIC_IRQ_MASKABLE     ((bool)true)
#define PMIC_IRQ_NON_MASKABLE ((bool)false)

// Used to hold information about IRQs, which register contains their status
// bit, and which register contains their mask bit, along with the relevant
// bitshift to use.
typedef struct Pmic_IrqInfo_s {
    uint8_t statReg;
    uint8_t maskReg;
    uint8_t shift;
    bool isMaskable;
} Pmic_IrqInfo_t;

// Tie Register addresses, bitshifts, and IRQ numbers together
static const Pmic_IrqInfo_t pmicIRQs[PMIC_IRQ_NUM] = {
    // Short-circuit NMIs
    { INT_BUCK_LDO_LS1_VMON1_REG, 0, BUCK1_SC_INT_SHIFT, PMIC_IRQ_NON_MASKABLE }, // 0
    { INT_BUCK_12_REG, MASK_BUCK_12_REG, BUCK1_OV_INT_SHIFT, PMIC_IRQ_MASKABLE }, // 1
    { INT_BUCK_12_REG, MASK_BUCK_12_REG, BUCK1_UV_INT_SHIFT, PMIC_IRQ_MASKABLE }, // 2
    { INT_BUCK_12_REG, MASK_BUCK_12_REG, BUCK1_RV_INT_SHIFT, PMIC_IRQ_MASKABLE }, // 3
    { INT_BUCK_12_REG, MASK_BUCK_12_REG, BUCK1_ILIM_INT_SHIFT, PMIC_IRQ_MASKABLE }, // 4
    { INT_BUCK_LDO_LS1_VMON1_REG, 0, BUCK2_SC_INT_SHIFT, PMIC_IRQ_NON_MASKABLE }, // 5
    { INT_BUCK_12_REG, MASK_BUCK_12_REG, BUCK2_OV_INT_SHIFT, PMIC_IRQ_MASKABLE }, // 6
    { INT_BUCK_12_REG, MASK_BUCK_12_REG, BUCK2_UV_INT_SHIFT, PMIC_IRQ_MASKABLE }, // 7
    { INT_BUCK_12_REG, MASK_BUCK_12_REG, BUCK2_RV_INT_SHIFT, PMIC_IRQ_MASKABLE }, // 8
    { INT_BUCK_12_REG, MASK_BUCK_12_REG, BUCK2_ILIM_INT_SHIFT, PMIC_IRQ_MASKABLE }, // 9
    { INT_BUCK_LDO_LS1_VMON1_REG, 0, BUCK3_SC_INT_SHIFT, PMIC_IRQ_NON_MASKABLE }, // 10
    { INT_BUCK3_LDO_LS1_VMON1_REG, MASK_BUCK3_LDO_LS1_VMON1_REG, BUCK3_OV_INT_SHIFT, PMIC_IRQ_MASKABLE }, // 11
    { INT_BUCK3_LDO_LS1_VMON1_REG, MASK_BUCK3_LDO_LS1_VMON1_REG, BUCK3_UV_INT_SHIFT, PMIC_IRQ_MASKABLE }, // 12
    { INT_BUCK3_LDO_LS1_VMON1_REG, MASK_BUCK3_LDO_LS1_VMON1_REG, BUCK3_RV_INT_SHIFT, PMIC_IRQ_MASKABLE }, // 13
    { INT_BUCK3_LDO_LS1_VMON1_REG, MASK_BUCK3_LDO_LS1_VMON1_REG, BUCK3_ILIM_INT_SHIFT, PMIC_IRQ_MASKABLE }, // 14
    { INT_BUCK_LDO_LS1_VMON1_REG, 0, LDO_LS1_VMON1_SC_INT_SHIFT, PMIC_IRQ_NON_MASKABLE }, // 15
    { INT_BUCK3_LDO_LS1_VMON1_REG, MASK_BUCK3_LDO_LS1_VMON1_REG, LDO_LS1_VMON1_OV_INT_SHIFT, PMIC_IRQ_MASKABLE }, // 16
    { INT_BUCK3_LDO_LS1_VMON1_REG, MASK_BUCK3_LDO_LS1_VMON1_REG, LDO_LS1_VMON1_UV_INT_SHIFT, PMIC_IRQ_MASKABLE }, // 17
    { INT_BUCK3_LDO_LS1_VMON1_REG, MASK_BUCK3_LDO_LS1_VMON1_REG, LDO_LS1_VMON1_RV_INT_SHIFT, PMIC_IRQ_MASKABLE }, // 18
    { INT_BUCK3_LDO_LS1_VMON1_REG, MASK_BUCK3_LDO_LS1_VMON1_REG, LDO_LS1_VMON1_ILIM_INT_SHIFT, PMIC_IRQ_MASKABLE }, // 19
    { INT_LS2_VMON2_REG, 0, LS2_VMON2_SC_NMI_SHIFT, PMIC_IRQ_NON_MASKABLE }, // 20
    { INT_LS2_VMON2_REG, MASK_LS2_VMON2_REG, LS2_VMON2_OV_INT_SHIFT, PMIC_IRQ_MASKABLE }, // 21
    { INT_LS2_VMON2_REG, MASK_LS2_VMON2_REG, LS2_VMON2_UV_INT_SHIFT, PMIC_IRQ_MASKABLE }, // 22
    { INT_LS2_VMON2_REG, MASK_LS2_VMON2_REG, LS2_VMON2_RV_INT_SHIFT, PMIC_IRQ_MASKABLE }, // 23
    { INT_LS2_VMON2_REG, MASK_LS2_VMON2_REG, LS2_VMON2_ILIM_INT_SHIFT, PMIC_IRQ_MASKABLE }, // 24
    { INT_VCCA_REG, MASK_VCCA_REG, VCCA_OV_INT_SHIFT, PMIC_IRQ_MASKABLE }, // 25
    { INT_VCCA_REG, MASK_VCCA_REG, VCCA_UV_INT_SHIFT, PMIC_IRQ_MASKABLE }, // 26
    { INT_STARTUP_REG, MASK_STARTUP_REG, ENABLE_INT_SHIFT, PMIC_IRQ_MASKABLE }, // 27
    { INT_MISC_REG, MASK_MISC_REG, ABIST_FAIL_INT_SHIFT, PMIC_IRQ_MASKABLE }, // 28
    { INT_MISC_REG, MASK_MISC_REG, BUCKS_VSET_ERR_INT_SHIFT, PMIC_IRQ_MASKABLE }, // 29
    { INT_MISC_REG, MASK_MISC_REG, EXT_CLK_INT_SHIFT, PMIC_IRQ_MASKABLE }, // 30
    { INT_MISC_REG, MASK_MISC_REG, TWARN_INT_SHIFT, PMIC_IRQ_MASKABLE }, // 31
    // Thermal/Recovery NMIs
    { INT_MODERATE_ERR_REG, 0, TSD_ORD_NMI_SHIFT, PMIC_IRQ_NON_MASKABLE }, // 32
    { INT_MODERATE_ERR_REG, 0, RECOV_CNT_NMI_SHIFT, PMIC_IRQ_NON_MASKABLE }, // 33
    { INT_MODERATE_ERR_REG, MASK_MODERATE_ERR_REG, TRIM_TEST_CRC_INT_SHIFT, PMIC_IRQ_MASKABLE }, // 34
    { INT_MODERATE_ERR_REG, MASK_MODERATE_ERR_REG, CONFIG_CRC_INT_SHIFT, PMIC_IRQ_MASKABLE }, // 35
    { INT_MODERATE_ERR_REG, MASK_MODERATE_ERR_REG, NINT_GPO_RDBK_INT_SHIFT, PMIC_IRQ_MASKABLE }, // 36
    { INT_MODERATE_ERR_REG, MASK_MODERATE_ERR_REG, NRSTOUT_RDBK_INT_SHIFT, PMIC_IRQ_MASKABLE }, // 37
    { INT_SEVERE_ERR_REG, 0, TSD_IMM_NMI_SHIFT, PMIC_IRQ_NON_MASKABLE }, // 38
    { INT_SEVERE_ERR_REG, 0, VCCA_OVP_NMI_SHIFT, PMIC_IRQ_NON_MASKABLE }, // 39
    { INT_COMM_ERR_REG, MASK_COMM_ERR_REG, COMM_FRM_ERR_INT_SHIFT, PMIC_IRQ_MASKABLE }, // 40
    { INT_COMM_ERR_REG, MASK_COMM_ERR_REG, COMM_CRC_ERR_INT_SHIFT, PMIC_IRQ_MASKABLE }, // 41
    { INT_COMM_ERR_REG, MASK_COMM_ERR_REG, COMM_ADR_ERR_INT_SHIFT, PMIC_IRQ_MASKABLE }, // 42
    { INT_COMM_ERR_REG, MASK_COMM_ERR_REG, COMM_MCU_ERR_INT_SHIFT, PMIC_IRQ_MASKABLE }, // 43
    { INT_ESM_REG, MASK_ESM_REG, ESM_MCU_PIN_INT_SHIFT, PMIC_IRQ_MASKABLE }, // 44
    { INT_ESM_REG, MASK_ESM_REG, ESM_MCU_FAIL_INT_SHIFT, PMIC_IRQ_MASKABLE }, // 45
    { INT_ESM_REG, MASK_ESM_REG, ESM_MCU_RST_INT_SHIFT, PMIC_IRQ_MASKABLE }, // 46
    // Watchdog/FSM NMIs
    { INT_FSM_ERR_REG, 0, WD_FIRST_NOK_INT_SHIFT, PMIC_IRQ_NON_MASKABLE }, // 47
    { WD_ERR_STAT_REG, 0, WD_LONGWIN_TIMEOUT_INT_SHIFT, PMIC_IRQ_NON_MASKABLE }, // 48
    { WD_ERR_STAT_REG, 0, WD_TIMEOUT_INT_SHIFT, PMIC_IRQ_NON_MASKABLE }, // 49
    { WD_ERR_STAT_REG, 0, WD_ANSWER_EARLY_INT_SHIFT, PMIC_IRQ_NON_MASKABLE }, // 50
    { WD_ERR_STAT_REG, 0, WD_SEQ_ERR_INT_SHIFT, PMIC_IRQ_NON_MASKABLE }, // 51
    { WD_ERR_STAT_REG, 0, WD_ANSWER_ERR_INT_SHIFT, PMIC_IRQ_NON_MASKABLE }, // 52
    { WD_ERR_STAT_REG, 0, WD_FAIL_INT_SHIFT, PMIC_IRQ_NON_MASKABLE }, // 53
    { WD_ERR_STAT_REG, 0, WD_RST_INT_SHIFT, PMIC_IRQ_NON_MASKABLE }, // 54
    { INT_FSM_ERR_REG, 0, REGULATOR_ERR_INT_SHIFT, PMIC_IRQ_NON_MASKABLE }, // 55
    { INT_FSM_ERR_REG, 0, IMM_SHUTDOWN_INT_SHIFT, PMIC_IRQ_NON_MASKABLE }, // 56
    { INT_FSM_ERR_REG, 0, ORD_SHUTDOWN_INT_SHIFT, PMIC_IRQ_NON_MASKABLE }, // 57
    { INT_FSM_ERR_REG, 0, WARM_RESET_INT_SHIFT, PMIC_IRQ_NON_MASKABLE }, // 58
};

static inline void IRQ_copyIrqMask(const Pmic_IrqMask_t *src, Pmic_IrqMask_t *dst) {
    (void)memmove((void *)dst, (const void *)src, sizeof(Pmic_IrqMask_t));
}

static inline void IRQ_copyIrqStat(const Pmic_IrqStatus_t *src, Pmic_IrqStatus_t *dst) {
    (void)memmove((void *)dst, (const void *)src, sizeof(Pmic_IrqStatus_t));
}

static inline void IRQ_setIntrStat(Pmic_IrqStatus_t *irqStat, uint32_t irqNum)
{
    if (irqNum <= PMIC_IRQ_MAX)
    {
        // IRQs 0 to 31 go to index 0, IRQs 32 to 63 go to index 1.
        // At an index, the IRQ is stored at its corresponding bit
        // (e.g., IRQ 49's status will be stored at bit 17 at index 1)
        irqStat->intrStat[irqNum / PMIC_NUM_BITS_IN_INTR_STAT] |= ((uint32_t)1UL << (irqNum % PMIC_NUM_BITS_IN_INTR_STAT));
    }
}

static int32_t IRQ_setMask(const Pmic_Handle_t *handle, uint8_t irqNum, bool shouldMask) {
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    uint8_t irqMaskRegAddr = 0U, irqMaskBitShift = 0U;

    // Check for invalid IRQ number
    if (irqNum > PMIC_IRQ_MAX) {
        status = PMIC_ST_ERR_INV_PARAM;
    } else {
        irqMaskRegAddr = pmicIRQs[irqNum].maskReg;
        irqMaskBitShift = pmicIRQs[irqNum].shift;
    }

    // Check whether IRQ is maskable
    if ((status == PMIC_ST_SUCCESS) &&
        (pmicIRQs[irqNum].isMaskable == PMIC_IRQ_NON_MASKABLE)) {
        status = PMIC_ST_ERR_NOT_SUPPORTED;
    }

    // Read IRQ mask register
    Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);
    if (status == PMIC_ST_SUCCESS) {
        status = Pmic_ioRxByte(handle, irqMaskRegAddr, &regData);
    }

    if (status == PMIC_ST_SUCCESS) {
        // Modify IRQ mask bit field
        Pmic_setBitField_b(&regData, irqMaskBitShift, shouldMask);

        // Write new register value back to PMIC
        status = Pmic_ioTxByte(handle, irqMaskRegAddr, regData);
    }
    Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);

    return status;
}

/**
 * @brief Validates an array of IRQ masks
 *
 * Checks that all IRQ numbers are valid and maskable. This ensures
 * we don't process invalid or unsupported IRQs.
 *
 * @param numMasks Number of masks to validate
 * @param masks    Array of IRQ mask structures
 *
 * @return PMIC_ST_SUCCESS if all valid, ERR_INV_PARAM if invalid IRQ number,
 *         ERR_NOT_SUPPORTED if non-maskable IRQ found
 */
static int32_t IRQ_validateMasks(uint8_t numMasks, const Pmic_IrqMask_t masks[]) {
    int32_t status = PMIC_ST_SUCCESS;

    for (uint8_t i = 0U; i < numMasks; i++) {
        const uint8_t irqNum = masks[i].irqNum;

        if (irqNum > PMIC_IRQ_MAX) {
            status = PMIC_ST_ERR_INV_PARAM;
        } else if (pmicIRQs[irqNum].isMaskable == PMIC_IRQ_NON_MASKABLE) {
            status = PMIC_ST_ERR_NOT_SUPPORTED;
        }

        if (status != PMIC_ST_SUCCESS) {
            break;
        }
    }

    return status;
}

/**
 * @brief Checks if any masks apply to specified register
 *
 * First validates all IRQ masks, then checks if any apply to the
 * specified register address.
 */
static int32_t IRQ_anyMasksForReg(uint8_t numMasks, const Pmic_IrqMask_t masks[], uint8_t regAddr, bool *anyMasks) {
    int32_t status = IRQ_validateMasks(numMasks, masks);
    bool anyRegs = (bool)false;

    if (status == PMIC_ST_SUCCESS) {
        for (uint8_t i = 0U; (i < numMasks) && !anyRegs; i++) {
            anyRegs = (pmicIRQs[masks[i].irqNum].maskReg == regAddr);
        }
        *anyMasks = anyRegs;
    }

    return status;
}

static int32_t IRQ_handleRecordsForReg(const Pmic_Handle_t *handle,
                                       uint8_t numMasks,
                                       const Pmic_IrqMask_t masks[],
                                       uint8_t regAddr,
                                       uint8_t *processedMasks)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    bool anyMasks = (bool)false;

    status = IRQ_anyMasksForReg(numMasks, masks, regAddr, &anyMasks);

    Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);
    if ((status == PMIC_ST_SUCCESS) && anyMasks) {
        // Get the current value of this IRQ mask register
        status = Pmic_ioRxByte(handle, regAddr, &regData);
    }

    if ((status == PMIC_ST_SUCCESS) && anyMasks) {
        for (uint8_t i = 0U; (i < PMIC_IRQ_LOOP_MAX) && (i < numMasks); i++) {
            const uint8_t irqNum = masks[i].irqNum;
            const Pmic_IrqInfo_t *pIrq = &pmicIRQs[irqNum];
            const uint8_t userMask = (uint8_t)(1UL << pIrq->shift);

            // If the current mask setting isn't targeted at the register we are
            // currently building, skip it
            if (regAddr != pIrq->maskReg) {
                continue;
            }

            if (masks[i].mask != false) {
                regData |= userMask;
            } else {
                regData &= ~userMask;
            }

            *processedMasks += 1U;
        }
    }

    // If status is still good and we did find records that apply to this
    // register, write the new value of this register back to the device
    if ((status == PMIC_ST_SUCCESS) && (*processedMasks > 0U)) {
        status = Pmic_ioTxByte(handle, regAddr, regData);
    }
    Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);

    return status;
}

int32_t Pmic_irqSetMask(const Pmic_Handle_t *handle, uint8_t irqNum, bool shouldMask) {
    int32_t status = Pmic_checkHandle(handle);

    if (status == PMIC_ST_SUCCESS) {
        status = IRQ_setMask(handle, irqNum, shouldMask);
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_irqSetMasks(const Pmic_Handle_t *handle, uint8_t numMasks, const Pmic_IrqMask_t *masks) {
    Pmic_IrqMask_t localMasks[PMIC_IRQ_NUM];
    int32_t status = Pmic_checkHandle(handle);
    uint32_t totalProcessed = 0U;
    uint8_t lastProcessed = 0U;

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

    if ((status == PMIC_ST_SUCCESS) && (masks == NULL)) {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) && (numMasks > PMIC_IRQ_NUM)) {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    if (status == PMIC_ST_SUCCESS) {
        for (uint8_t i = 0U; (i < PMIC_IRQ_LOOP_MAX) && (i < numMasks); i++) {
            IRQ_copyIrqMask(&masks[i], &localMasks[i]);
        }
    }

    for (uint8_t regIndex = 0U; regIndex < NUM_MASKABLE_REGISTERS; regIndex++) {
        // If status is no longer good or all user requested masks have been
        // processed, we can stop iterating
        if ((status != PMIC_ST_SUCCESS) || (totalProcessed >= numMasks)) {
            break;
        }

        status = IRQ_handleRecordsForReg(handle, numMasks, localMasks, MaskableRegisters[regIndex], &lastProcessed);
        totalProcessed += lastProcessed;
        lastProcessed = 0U;
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_irqGetMask(const Pmic_Handle_t *handle, uint8_t numIrqMasks, Pmic_IrqMask_t *irqMasks) {
    Pmic_IrqMask_t localIrqMasks[PMIC_IRQ_NUM];
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    if ((status == PMIC_ST_SUCCESS) && (irqMasks == NULL)) {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) && (numIrqMasks > PMIC_IRQ_NUM)) {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    if (status == PMIC_ST_SUCCESS) {
        for (uint8_t i = 0U; (i < PMIC_IRQ_LOOP_MAX) && (i < numIrqMasks); i++) {
            IRQ_copyIrqMask(&irqMasks[i], &localIrqMasks[i]);
        }
    }

    if (status == PMIC_ST_SUCCESS) {
        for (uint8_t i = 0U; (i < PMIC_IRQ_LOOP_MAX) && (i < numIrqMasks); i++) {
            const uint8_t irqNum = localIrqMasks[i].irqNum;
            uint8_t irqMaskRegAddr = 0U, irqMaskBitShift = 0U;

            // Check for invalid IRQ number
            if (irqNum > PMIC_IRQ_MAX) {
                status = PMIC_ST_ERR_INV_PARAM;
            } else {
                irqMaskRegAddr = pmicIRQs[irqNum].maskReg;
                irqMaskBitShift = pmicIRQs[irqNum].shift;
            }

            // Check whether IRQ is maskable
            if ((status == PMIC_ST_SUCCESS) &&
                (pmicIRQs[irqNum].isMaskable == PMIC_IRQ_NON_MASKABLE)) {
                status = PMIC_ST_ERR_NOT_SUPPORTED;
            }

            // Read IRQ mask register
            if (status == PMIC_ST_SUCCESS) {
                status = Pmic_ioRxByte_CS(handle, irqMaskRegAddr, &regData);
            }

            if (status == PMIC_ST_SUCCESS) {
                // Extract IRQ mask bit field
                localIrqMasks[i].mask = Pmic_getBitField_b(regData, irqMaskBitShift);
            } else {
                break;
            }
        }
    }

    if (status == PMIC_ST_SUCCESS) {
        for (uint8_t i = 0U; (i < PMIC_IRQ_LOOP_MAX) && (i < numIrqMasks); i++) {
            IRQ_copyIrqMask(&localIrqMasks[i], &irqMasks[i]);
        }
    }

    return Pmic_logStatus(handle, status);
}

static inline void IRQ_extractBits(Pmic_IrqStatus_t *irqStat, uint8_t regData, const uint8_t irqs[], uint8_t numIrqs) {
    for (uint8_t i = 0U; (i < PMIC_IRQ_LOOP_MAX) && (i < numIrqs); i++) {
        const uint8_t irqNum = irqs[i];
        if (Pmic_getBitField_b(regData, pmicIRQs[irqNum].shift)) {
            IRQ_setIntrStat(irqStat, irqNum);
        }
    }
}

// NOTE: This function should only be called from within a critical section
static int32_t IRQ_getStatFSM(const Pmic_Handle_t *handle, Pmic_IrqStatus_t *irqStat) {
    uint8_t regData = 0U;
    int32_t status = Pmic_ioRxByte(handle, INT_FSM_ERR_REG, &regData);
    bool esmMcuInt, commErrInt, wdInt;

    // Top level IRQ numbers for INT_FSM_ERR
    const uint8_t fsmIrqs[] = {
        (uint8_t)PMIC_FSM_IMM_SHUTDOWN_NMI,
        (uint8_t)PMIC_FSM_ORD_SHUTDOWN_NMI,
        (uint8_t)PMIC_FSM_WARM_RESET_NMI,
        (uint8_t)PMIC_REGULATOR_ERR_NMI,
        (uint8_t)PMIC_WDG_FIRST_NOK_NMI
    };

    // Top level IRQ numbers for INT_COMM_ERR
    const uint8_t commIrqs[] = {
        (uint8_t)PMIC_COMM_FRM_ERR_INT,
        (uint8_t)PMIC_COMM_CRC_ERR_INT,
        (uint8_t)PMIC_COMM_ADR_ERR_INT,
        (uint8_t)PMIC_COMM_MCU_ERR_INT
    };

    // Top level IRQ numbers for INT_ESM
    const uint8_t esmIrqs[] = {
        (uint8_t)PMIC_ESM_MCU_PIN_INT,
        (uint8_t)PMIC_ESM_MCU_FAIL_INT,
        (uint8_t)PMIC_ESM_MCU_RST_INT
    };

    // Top level IRQ numbers for WD_ERR_STAT
    const uint8_t wdIrq[] = {
        (uint8_t)PMIC_WDG_LONGWIN_TIMEOUT_NMI,
        (uint8_t)PMIC_WDG_TIMEOUT_NMI,
        (uint8_t)PMIC_WDG_ANSWER_EARLY_NMI,
        (uint8_t)PMIC_WDG_SEQ_ERR_NMI,
        (uint8_t)PMIC_WDG_ANSWER_ERR_NMI,
        (uint8_t)PMIC_WDG_FAIL_NMI,
        (uint8_t)PMIC_WDG_RST_NMI
    };

    // Parse L0 bits from INT_FSM_ERR
    if (status == PMIC_ST_SUCCESS) {
        IRQ_extractBits(irqStat, regData, fsmIrqs, (uint8_t)COUNT(fsmIrqs));
    }

    // Extract L1 indicator bits from regData so it can be repurposed
    esmMcuInt = Pmic_getBitField_b(regData, ESM_MCU_INT_SHIFT);
    commErrInt = Pmic_getBitField_b(regData, COMM_ERR_INT_SHIFT);
    wdInt = Pmic_getBitField_b(regData, WD_INT_SHIFT);

    // If ESM_MCU_INT is set, read and extract bits from L1 register INT_ESM
    if ((status == PMIC_ST_SUCCESS) && (esmMcuInt == true)) {
        status = Pmic_ioRxByte(handle, INT_ESM_REG, &regData);

        if (status == PMIC_ST_SUCCESS) {
            IRQ_extractBits(irqStat, regData, esmIrqs, (uint8_t)COUNT(esmIrqs));
        }
    }

    // If COMM_ERR_INT is set, read and extract bits from L1 register INT_COMM_ERR
    if ((status == PMIC_ST_SUCCESS) && (commErrInt == true)) {
        status = Pmic_ioRxByte(handle, INT_COMM_ERR_REG, &regData);

        if (status == PMIC_ST_SUCCESS) {
            IRQ_extractBits(irqStat, regData, commIrqs, (uint8_t)COUNT(commIrqs));
        }
    }

    // If WD_INT is set, read and extract bits from L1 register WD_ERR_STAT
    if ((status == PMIC_ST_SUCCESS) && (wdInt == true)) {
        status = Pmic_ioRxByte(handle, WD_ERR_STAT_REG, &regData);

        if (status == PMIC_ST_SUCCESS) {
            IRQ_extractBits(irqStat, regData, wdIrq, (uint8_t)COUNT(wdIrq));
        }
    }

    return status;
}

// NOTE: This function should only be called from within a critical section
static int32_t IRQ_getStatSevere(const Pmic_Handle_t *handle, Pmic_IrqStatus_t *irqStat) {
    uint8_t regData = 0U;
    int32_t status = Pmic_ioRxByte(handle, INT_SEVERE_ERR_REG, &regData);

    const uint8_t irqs[] = {(uint8_t)PMIC_SE_TSD_IMM_NMI, (uint8_t)PMIC_SE_VCCA_OVP_NMI};

    if (status == PMIC_ST_SUCCESS) {
        IRQ_extractBits(irqStat, regData, irqs, (uint8_t)COUNT(irqs));
    }

    return status;
}

// NOTE: This function should only be called from within a critical section
static int32_t IRQ_getStatModerate(const Pmic_Handle_t *handle, Pmic_IrqStatus_t *irqStat) {
    uint8_t regData = 0U;
    int32_t status = Pmic_ioRxByte(handle, INT_MODERATE_ERR_REG, &regData);

    const uint8_t irqs[] = {
        (uint8_t)PMIC_ME_TSD_ORD_NMI,
        (uint8_t)PMIC_ME_RECOV_CNT_NMI,
        (uint8_t)PMIC_ME_TRIM_TEST_CRC_INT,
        (uint8_t)PMIC_ME_CONFIG_CRC_INT,
        (uint8_t)PMIC_ME_NINT_READBACK_INT,
        (uint8_t)PMIC_ME_NRSTOUT_READBACK_INT
    };

    if (status == PMIC_ST_SUCCESS) {
        IRQ_extractBits(irqStat, regData, irqs, (uint8_t)COUNT(irqs));
    }

    return status;
}

// NOTE: This function should only be called from within a critical section
static int32_t IRQ_getStatMisc(const Pmic_Handle_t *handle, Pmic_IrqStatus_t *irqStat) {
    uint8_t regData = 0U;
    int32_t status = Pmic_ioRxByte(handle, INT_MISC_REG, &regData);

    const uint8_t irqs[] = {
        (uint8_t)PMIC_MISC_ABIST_FAIL_INT,
        (uint8_t)PMIC_MISC_BUCKS_VSET_ERR_INT,
        (uint8_t)PMIC_MISC_EXT_CLK_INT,
        (uint8_t)PMIC_MISC_TWARN_INT
    };

    if (status == PMIC_ST_SUCCESS) {
        IRQ_extractBits(irqStat, regData, irqs, (uint8_t)COUNT(irqs));
    }

    return status;
}

// NOTE: This function should only be called from within a critical section
static int32_t IRQ_getStatStartup(const Pmic_Handle_t *handle, Pmic_IrqStatus_t *irqStat) {
    uint8_t regData = 0U;
    int32_t status = Pmic_ioRxByte(handle, INT_STARTUP_REG, &regData);

    if (Pmic_getBitField_b(regData, pmicIRQs[PMIC_STARTUP_ENABLE_INT].shift)) {
        IRQ_setIntrStat(irqStat, PMIC_STARTUP_ENABLE_INT);
    }

    return status;
}

// NOTE: This function should only be called from within a critical section
static int32_t IRQ_getStatVccaVmon1(const Pmic_Handle_t *handle, Pmic_IrqStatus_t *irqStat) {
    uint8_t regData = 0U;
    int32_t status = Pmic_ioRxByte(handle, INT_VCCA_REG, &regData);

    const uint8_t irqs[] = {(uint8_t)PMIC_VCCA_OV_INT, (uint8_t)PMIC_VCCA_UV_INT};

    if (status == PMIC_ST_SUCCESS) {
        IRQ_extractBits(irqStat, regData, irqs, (uint8_t)COUNT(irqs));
    }

    return status;
}

// NOTE: This function should only be called from within a critical section
static int32_t IRQ_getStatLs2Vmon2(const Pmic_Handle_t *handle, Pmic_IrqStatus_t *irqStat) {
    uint8_t regData = 0U;
    int32_t status = Pmic_ioRxByte(handle, INT_LS2_VMON2_REG, &regData);

    const uint8_t irqs[] = {
        (uint8_t)PMIC_LS2_VMON2_OV_INT,
        (uint8_t)PMIC_LS2_VMON2_UV_INT,
        (uint8_t)PMIC_LS2_VMON2_RV_INT,
        (uint8_t)PMIC_LS2_VMON2_ILIM_INT,
        (uint8_t)PMIC_LS2_VMON2_SC_NMI
    };

    if (status == PMIC_ST_SUCCESS) {
        IRQ_extractBits(irqStat, regData, irqs, (uint8_t)COUNT(irqs));
    }

    return status;
}

// NOTE: This function should only be called from within a critical section
static int32_t IRQ_getStatBucks(const Pmic_Handle_t *handle, Pmic_IrqStatus_t *irqStat) {
    uint8_t regData = 0U;
    int32_t status = Pmic_ioRxByte(handle, INT_BUCK_LDO_LS1_VMON1_REG, &regData);
    bool buck12, buck3Ldo;

    const uint8_t topIrqs[] = {
        (uint8_t)PMIC_BUCK1_SC_NMI,
        (uint8_t)PMIC_BUCK2_SC_NMI,
        (uint8_t)PMIC_BUCK3_SC_NMI,
        (uint8_t)PMIC_LDO_LS1_VMON1_SC_NMI
    };

    const uint8_t buck12Irqs[] = {
        (uint8_t)PMIC_BUCK1_OV_INT,
        (uint8_t)PMIC_BUCK1_UV_INT,
        (uint8_t)PMIC_BUCK1_RV_INT,
        (uint8_t)PMIC_BUCK1_ILIM_INT,
        (uint8_t)PMIC_BUCK2_OV_INT,
        (uint8_t)PMIC_BUCK2_UV_INT,
        (uint8_t)PMIC_BUCK2_RV_INT,
        (uint8_t)PMIC_BUCK2_ILIM_INT
    };

    const uint8_t buck3LdoIrqs[] = {
        (uint8_t)PMIC_BUCK3_OV_INT,
        (uint8_t)PMIC_BUCK3_UV_INT,
        (uint8_t)PMIC_BUCK3_RV_INT,
        (uint8_t)PMIC_BUCK3_ILIM_INT,
        (uint8_t)PMIC_LDO_LS1_VMON1_OV_INT,
        (uint8_t)PMIC_LDO_LS1_VMON1_UV_INT,
        (uint8_t)PMIC_LDO_LS1_VMON1_RV_INT,
        (uint8_t)PMIC_LDO_LS1_VMON1_ILIM_INT
    };

    if (status == PMIC_ST_SUCCESS) {
        IRQ_extractBits(irqStat, regData, topIrqs, (uint8_t)COUNT(topIrqs));
    }

    // Extract L1 indicator bits from regData so it can be repurposed
    buck12 = ((Pmic_getBitField_b(regData, BUCK1_INT_SHIFT) == true) ||
              (Pmic_getBitField_b(regData, BUCK2_INT_SHIFT) == true));
    buck3Ldo = ((Pmic_getBitField_b(regData, BUCK3_INT_SHIFT) == true) ||
                (Pmic_getBitField_b(regData, LDO_LS1_VMON1_INT_SHIFT) == true));

    // If BUCK1_INT or BUCK2_INT are set, read and extract bits from L1 register
    // INT_BUCK_12
    if ((status == PMIC_ST_SUCCESS) && (buck12 == true)) {
        status = Pmic_ioRxByte(handle, INT_BUCK_12_REG, &regData);

        if (status == PMIC_ST_SUCCESS) {
            IRQ_extractBits(irqStat, regData, buck12Irqs, (uint8_t)COUNT(buck12Irqs));
        }
    }

    // If BUCK3_INT or LDO_LS2_VMON1_INT are set, read and extract bits from L1
    // register INT_BUCK3_LDO_LS1_VMON1
    if ((status == PMIC_ST_SUCCESS) && (buck3Ldo == true)) {
        status = Pmic_ioRxByte(handle, INT_BUCK3_LDO_LS1_VMON1_REG, &regData);

        if (status == PMIC_ST_SUCCESS) {
            IRQ_extractBits(irqStat, regData, buck3LdoIrqs, (uint8_t)COUNT(buck3LdoIrqs));
        }
    }

    return status;
}

static int32_t IRQ_getStat(const Pmic_Handle_t *handle, Pmic_IrqStatus_t *irqStat) {
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    // Clear register backing to ensure all data is fresh
    for (uint8_t i = 0U; i < PMIC_NUM_ELEM_IN_INTR_STAT; i++) {
        irqStat->intrStat[i] = 0U;
    }

    // Obtain critical section around all of these reads
    Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);

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
    Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);

    return status;
}

int32_t Pmic_irqGetStatus(const Pmic_Handle_t *handle, Pmic_IrqStatus_t *irqStat) {
    Pmic_IrqStatus_t localIrqStat;
    int32_t status = Pmic_checkHandle(handle);

    if ((status == PMIC_ST_SUCCESS) && (irqStat == NULL)) {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if (status == PMIC_ST_SUCCESS) {
        status = IRQ_getStat(handle, &localIrqStat);
    }

    if (status == PMIC_ST_SUCCESS) {
        IRQ_copyIrqStat(&localIrqStat, irqStat);
    }

    return Pmic_logStatus(handle, status);
}

static uint8_t IRQ_getNextFlag(Pmic_IrqStatus_t *irqStat) {
    uint8_t statIndex = 0U, bitPos = 0U;
    bool foundFlag = false;

    // For each element in struct member intrStat of irqStat...
    for (statIndex = 0U; statIndex < PMIC_NUM_ELEM_IN_INTR_STAT; statIndex++) {
        // If current element has no IRQ statuses set, move onto next element
        if (irqStat->intrStat[statIndex] == 0U) {
            continue;
        }

        // For each bit in the element...
        for (bitPos = 0U; bitPos < PMIC_NUM_BITS_IN_INTR_STAT; bitPos++) {
            // If the bit is set...
            const uint32_t mask = ((uint32_t)1UL << bitPos);
            if ((irqStat->intrStat[statIndex] & mask) != 0U) {
                // Clear bit in intrStat element and exit loop
                irqStat->intrStat[statIndex] &= ~mask;

                foundFlag = true;
                break;
            }
        }

        if (foundFlag) {
            break;
        }
    } /* LCOV_EXCL_LINE */

    // Return the corresponding IRQ number
    return (bitPos + (PMIC_NUM_BITS_IN_INTR_STAT * statIndex));
}

int32_t Pmic_irqGetNextFlag(const Pmic_Handle_t *handle, Pmic_IrqStatus_t *irqStat, uint8_t *irqNum) {
    Pmic_IrqStatus_t localIrqStat;
    int32_t status = PMIC_ST_SUCCESS;
    bool irqStatEmpty = false;

    if (((irqStat == NULL) || (irqNum == NULL))) {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if (status == PMIC_ST_SUCCESS) {
        IRQ_copyIrqStat(irqStat, &localIrqStat);
        irqStatEmpty = ((localIrqStat.intrStat[0U] == 0U) && (localIrqStat.intrStat[1U] == 0U));
    }

    if ((status == PMIC_ST_SUCCESS) && irqStatEmpty) {
        status = PMIC_ST_WARN_NO_IRQ_REMAINING;
    }

    if (status == PMIC_ST_SUCCESS) {
        *irqNum = IRQ_getNextFlag(&localIrqStat);
        IRQ_copyIrqStat(&localIrqStat, irqStat);
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_irqGetFlag(const Pmic_Handle_t *handle, uint8_t irqNum, bool *flag) {
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    if ((status == PMIC_ST_SUCCESS) && (irqNum > PMIC_IRQ_MAX)) {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) && (flag == NULL)) {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    // Read IRQ status register
    if (status == PMIC_ST_SUCCESS) {
        status = Pmic_ioRxByte_CS(handle, pmicIRQs[irqNum].statReg, &regData);
    }

    // Extract IRQ status
    if (status == PMIC_ST_SUCCESS) {
        *flag = Pmic_getBitField_b(regData, pmicIRQs[irqNum].shift);
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_irqClrFlag(const Pmic_Handle_t *handle, uint8_t irqNum) {
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    if ((status == PMIC_ST_SUCCESS) && (irqNum > PMIC_IRQ_MAX)) {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    if (status == PMIC_ST_SUCCESS) {
        // IRQ statuses are W1C - write 1 to clear
        Pmic_setBitField_b(&regData, pmicIRQs[irqNum].shift, PMIC_ENABLE);

        // Write data to PMIC
        Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);
        status = Pmic_ioTxByte(handle, pmicIRQs[irqNum].statReg, regData);
        Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_irqClrAllFlags(const Pmic_Handle_t *handle) {
    int32_t status = Pmic_checkHandle(handle);

    static const uint8_t ClearableRegisters[NUM_CLEARABLE_REGISTERS] = {
        (uint8_t)INT_BUCK_LDO_LS1_VMON1_REG,
        (uint8_t)INT_BUCK_12_REG,
        (uint8_t)INT_BUCK3_LDO_LS1_VMON1_REG,
        (uint8_t)INT_LS2_VMON2_REG,
        (uint8_t)INT_VCCA_REG,
        (uint8_t)INT_STARTUP_REG,
        (uint8_t)INT_MISC_REG,
        (uint8_t)INT_MODERATE_ERR_REG,
        (uint8_t)INT_SEVERE_ERR_REG,
        (uint8_t)INT_FSM_ERR_REG,
        (uint8_t)INT_COMM_ERR_REG,
        (uint8_t)INT_ESM_REG,
        (uint8_t)WD_ERR_STAT_REG,
    };

    // All IRQ statuses are W1C, writing to reserved bits has no effect, so just
    // write every bit to 1
    Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);
    for (uint8_t i = 0; i < NUM_CLEARABLE_REGISTERS; i++) {
        if (status == PMIC_ST_SUCCESS) {
            status = Pmic_ioTxByte(handle, ClearableRegisters[i], CLEAR_ALL_STAT_BITS);
        } else {
            break;
        }
    }
    Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);

    return Pmic_logStatus(handle, status);
}
