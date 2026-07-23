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
/**
 * @file pmic_irq.c
 *
 * @brief PMIC LLD IRQ module source file containing definitions to APIs that
 * interact with PMIC IRQs.
 */
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "pmic.h"
#include "pmic_irq.h"
#include "pmic_io.h"

#include "regmap/esm.h"
#include "regmap/irq.h"
#include "regmap/wdg.h"

#define NUM_MASKABLE_REGISTERS  ((uint8_t)6U)
#define NUM_CLEARABLE_REGISTERS ((uint8_t)10U)
#define PMIC_IRQ_LOOP_MAX       (44U)  // PMIC_IRQ_MAX + 1 = 43 + 1

#define CLEAR_ALL_STAT_BITS (0xFFU)

#define PMIC_IRQ_MASKABLE     ((bool)true)
#define PMIC_IRQ_NON_MASKABLE ((bool)false)

/**
 * @anchor Pmic_IrqInfo
 * @name PMIC IRQ Information Struct
 *
 * @brief This struct is used to hold information regarding an IRQ.
 *
 * @param statRegAddr Address of the register holding the IRQ status bit.
 * @param maskRegAddr Address of the register holding the bit that masks the IRQ (only valid if isMaskable is true).
 * @param bitShift Position of the IRQ status/mask bit.
 * @param isMaskable Indicates whether this IRQ can be masked.
 */
typedef struct Pmic_IrqInfo_s
{
    uint8_t statRegAddr;
    uint8_t maskRegAddr;
    uint8_t bitShift;
    bool isMaskable;
} Pmic_IrqInfo_t;

/**
 * @brief All IRQs of TPS65036x PMIC that are directly clearable
 */
static const Pmic_IrqInfo_t pmicIRQs[PMIC_IRQ_MAX + 1U] =
{
    // 0 - Short-circuit NMIs
    {
        .statRegAddr = PMIC_INT_BUCK_LDO_REG,
        .maskRegAddr = 0,
        .bitShift = PMIC_LDO_SC_INT_SHIFT,
        .isMaskable = PMIC_IRQ_NON_MASKABLE
    },
    // 1
    {
        .statRegAddr = PMIC_INT_BUCK_LDO_REG,
        .maskRegAddr = 0,
        .bitShift = PMIC_BUCK3_SC_INT_SHIFT,
        .isMaskable = PMIC_IRQ_NON_MASKABLE
    },
    // 2
    {
        .statRegAddr = PMIC_INT_BUCK_LDO_REG,
        .maskRegAddr = 0,
        .bitShift = PMIC_BUCK2_SC_INT_SHIFT,
        .isMaskable = PMIC_IRQ_NON_MASKABLE
    },
    // 3
    {
        .statRegAddr = PMIC_INT_BUCK_LDO_REG,
        .maskRegAddr = 0,
        .bitShift = PMIC_BUCK1_SC_INT_SHIFT,
        .isMaskable = PMIC_IRQ_NON_MASKABLE
    },
    // 4-9 - BUCK1/2 Maskable interrupts
    {.statRegAddr = PMIC_INT_BUCK1_2_REG, .maskRegAddr = PMIC_MASK_BUCK1_2_REG, .bitShift = PMIC_BUCK2_OVP_INT_SHIFT, .isMaskable = PMIC_IRQ_MASKABLE},
    {.statRegAddr = PMIC_INT_BUCK1_2_REG, .maskRegAddr = PMIC_MASK_BUCK1_2_REG, .bitShift = PMIC_BUCK2_UV_INT_SHIFT, .isMaskable = PMIC_IRQ_MASKABLE},
    {.statRegAddr = PMIC_INT_BUCK1_2_REG, .maskRegAddr = PMIC_MASK_BUCK1_2_REG, .bitShift = PMIC_BUCK2_OV_INT_SHIFT, .isMaskable = PMIC_IRQ_MASKABLE},
    {.statRegAddr = PMIC_INT_BUCK1_2_REG, .maskRegAddr = PMIC_MASK_BUCK1_2_REG, .bitShift = PMIC_BUCK1_OVP_INT_SHIFT, .isMaskable = PMIC_IRQ_MASKABLE},
    {.statRegAddr = PMIC_INT_BUCK1_2_REG, .maskRegAddr = PMIC_MASK_BUCK1_2_REG, .bitShift = PMIC_BUCK1_UV_INT_SHIFT, .isMaskable = PMIC_IRQ_MASKABLE},
    {.statRegAddr = PMIC_INT_BUCK1_2_REG, .maskRegAddr = PMIC_MASK_BUCK1_2_REG, .bitShift = PMIC_BUCK1_OV_INT_SHIFT, .isMaskable = PMIC_IRQ_MASKABLE},
    // 10-15 - BUCK3/LDO Maskable interrupts
    {.statRegAddr = PMIC_INT_BUCK3_LDO_REG, .maskRegAddr = PMIC_MASK_BUCK3_LDO_REG, .bitShift = PMIC_LDO_OVP_INT_SHIFT, .isMaskable = PMIC_IRQ_MASKABLE},
    {.statRegAddr = PMIC_INT_BUCK3_LDO_REG, .maskRegAddr = PMIC_MASK_BUCK3_LDO_REG, .bitShift = PMIC_LDO_UV_INT_SHIFT, .isMaskable = PMIC_IRQ_MASKABLE},
    {.statRegAddr = PMIC_INT_BUCK3_LDO_REG, .maskRegAddr = PMIC_MASK_BUCK3_LDO_REG, .bitShift = PMIC_LDO_OV_INT_SHIFT, .isMaskable = PMIC_IRQ_MASKABLE},
    {.statRegAddr = PMIC_INT_BUCK3_LDO_REG, .maskRegAddr = PMIC_MASK_BUCK3_LDO_REG, .bitShift = PMIC_BUCK3_OVP_INT_SHIFT, .isMaskable = PMIC_IRQ_MASKABLE},
    {.statRegAddr = PMIC_INT_BUCK3_LDO_REG, .maskRegAddr = PMIC_MASK_BUCK3_LDO_REG, .bitShift = PMIC_BUCK3_UV_INT_SHIFT, .isMaskable = PMIC_IRQ_MASKABLE},
    {.statRegAddr = PMIC_INT_BUCK3_LDO_REG, .maskRegAddr = PMIC_MASK_BUCK3_LDO_REG, .bitShift = PMIC_BUCK3_OV_INT_SHIFT, .isMaskable = PMIC_IRQ_MASKABLE},
    // 16-18 - MISC Maskable interrupts
    {.statRegAddr = PMIC_INT_MISC_REG, .maskRegAddr = PMIC_MASK_MISC_REG, .bitShift = PMIC_TWARN_INT_SHIFT, .isMaskable = PMIC_IRQ_MASKABLE},
    {.statRegAddr = PMIC_INT_MISC_REG, .maskRegAddr = PMIC_MASK_MISC_REG, .bitShift = PMIC_B1_PVIN_UVLO_INT_SHIFT, .isMaskable = PMIC_IRQ_MASKABLE},
    {.statRegAddr = PMIC_INT_MISC_REG, .maskRegAddr = PMIC_MASK_MISC_REG, .bitShift = PMIC_BUCKS_VSET_ERR_INT_SHIFT, .isMaskable = PMIC_IRQ_MASKABLE},
    // 19-21 - NVM NMIs
    {.statRegAddr = PMIC_INT_MISC_REG, .maskRegAddr = 0, .bitShift = PMIC_CFG_NVM_VERIFY_ERR_SHIFT, .isMaskable = PMIC_IRQ_NON_MASKABLE},
    {.statRegAddr = PMIC_INT_MISC_REG, .maskRegAddr = 0, .bitShift = PMIC_CFG_NVM_VERIFY_DONE_SHIFT, .isMaskable = PMIC_IRQ_NON_MASKABLE},
    {.statRegAddr = PMIC_INT_MISC_REG, .maskRegAddr = 0, .bitShift = PMIC_CFG_NVM_PRG_DONE_SHIFT, .isMaskable = PMIC_IRQ_NON_MASKABLE},
    // 22-23 - ABIST Maskable
    {.statRegAddr = PMIC_INT_MISC_REG, .maskRegAddr = PMIC_MASK_MISC_REG, .bitShift = PMIC_ABIST_FAIL_INT_SHIFT, .isMaskable = PMIC_IRQ_MASKABLE},
    {.statRegAddr = PMIC_INT_MISC_REG, .maskRegAddr = PMIC_MASK_MISC_REG, .bitShift = PMIC_ABIST_DONE_INT_SHIFT, .isMaskable = PMIC_IRQ_MASKABLE},
    // 24-27 - MODERATE_ERR Maskable
    {.statRegAddr = PMIC_INT_MODERATE_ERR_REG, .maskRegAddr = PMIC_MASK_MODERATE_ERR_REG, .bitShift = PMIC_GPO_READBACK_INT_SHIFT, .isMaskable = PMIC_IRQ_MASKABLE},
    {.statRegAddr = PMIC_INT_MODERATE_ERR_REG, .maskRegAddr = PMIC_MASK_MODERATE_ERR_REG, .bitShift = PMIC_NINT_READBACK_INT_SHIFT, .isMaskable = PMIC_IRQ_MASKABLE},
    {.statRegAddr = PMIC_INT_MODERATE_ERR_REG, .maskRegAddr = PMIC_MASK_MODERATE_ERR_REG, .bitShift = PMIC_CONFIG_CRC_INT_SHIFT, .isMaskable = PMIC_IRQ_MASKABLE},
    {.statRegAddr = PMIC_INT_MODERATE_ERR_REG, .maskRegAddr = PMIC_MASK_MODERATE_ERR_REG, .bitShift = PMIC_TRIM_TEST_CRC_INT_SHIFT, .isMaskable = PMIC_IRQ_MASKABLE},
    // 28 - Recovery counter NMI
    {.statRegAddr = PMIC_INT_MODERATE_ERR_REG, .maskRegAddr = 0, .bitShift = PMIC_RECOV_CNT_INT_SHIFT, .isMaskable = PMIC_IRQ_NON_MASKABLE},
    // 29 - Thermal shutdown NMI
    {.statRegAddr = PMIC_INT_SEVERE_ERR_REG, .maskRegAddr = 0, .bitShift = PMIC_TSD_IMM_INT_SHIFT, .isMaskable = PMIC_IRQ_NON_MASKABLE},
    // 30-34 - FSM Error NMIs
    {.statRegAddr = PMIC_INT_FSM_ERR_REG, .maskRegAddr = 0, .bitShift = PMIC_WD_FIRST_NOK_INT_SHIFT, .isMaskable = PMIC_IRQ_NON_MASKABLE},
    {.statRegAddr = PMIC_INT_FSM_ERR_REG, .maskRegAddr = 0, .bitShift = PMIC_WAIT_FOR_PWRCYCLE_INT_SHIFT, .isMaskable = PMIC_IRQ_NON_MASKABLE},
    {.statRegAddr = PMIC_INT_FSM_ERR_REG, .maskRegAddr = 0, .bitShift = PMIC_WARM_RESET_INT_SHIFT, .isMaskable = PMIC_IRQ_NON_MASKABLE},
    {.statRegAddr = PMIC_INT_FSM_ERR_REG, .maskRegAddr = 0, .bitShift = PMIC_ORD_SHUTDOWN_INT_SHIFT, .isMaskable = PMIC_IRQ_NON_MASKABLE},
    {.statRegAddr = PMIC_INT_FSM_ERR_REG, .maskRegAddr = 0, .bitShift = PMIC_IMM_SHUTDOWN_INT_SHIFT, .isMaskable = PMIC_IRQ_NON_MASKABLE},
    // 35-37 - COMM_ERR Maskable
    {.statRegAddr = PMIC_INT_COMM_ERR_REG, .maskRegAddr = PMIC_MASK_COMM_ERR_REG, .bitShift = PMIC_MCU_COMM_ERR_INT_SHIFT, .isMaskable = PMIC_IRQ_MASKABLE},
    {.statRegAddr = PMIC_INT_COMM_ERR_REG, .maskRegAddr = PMIC_MASK_COMM_ERR_REG, .bitShift = PMIC_COMM_ADR_ERR_INT_SHIFT, .isMaskable = PMIC_IRQ_MASKABLE},
    {.statRegAddr = PMIC_INT_COMM_ERR_REG, .maskRegAddr = PMIC_MASK_COMM_ERR_REG, .bitShift = PMIC_COMM_CRC_ERR_INT_SHIFT, .isMaskable = PMIC_IRQ_MASKABLE},
    // 38-40 - ESM Maskable
    {.statRegAddr = PMIC_INT_ESM_REG, .maskRegAddr = PMIC_MASK_ESM_REG, .bitShift = PMIC_ESM_MCU_RST_INT_SHIFT, .isMaskable = PMIC_IRQ_MASKABLE},
    {.statRegAddr = PMIC_INT_ESM_REG, .maskRegAddr = PMIC_MASK_ESM_REG, .bitShift = PMIC_ESM_MCU_FAIL_INT_SHIFT, .isMaskable = PMIC_IRQ_MASKABLE},
    {.statRegAddr = PMIC_INT_ESM_REG, .maskRegAddr = PMIC_MASK_ESM_REG, .bitShift = PMIC_ESM_MCU_PIN_INT_SHIFT, .isMaskable = PMIC_IRQ_MASKABLE},
    // 41-43 - Watchdog NMIs
    {.statRegAddr = PMIC_WD_ERR_STATUS_REG, .maskRegAddr = 0, .bitShift = PMIC_WD_RST_INT_SHIFT, .isMaskable = PMIC_IRQ_NON_MASKABLE},
    {.statRegAddr = PMIC_WD_ERR_STATUS_REG, .maskRegAddr = 0, .bitShift = PMIC_WD_FAIL_INT_SHIFT, .isMaskable = PMIC_IRQ_NON_MASKABLE},
    {.statRegAddr = PMIC_WD_ERR_STATUS_REG, .maskRegAddr = 0, .bitShift = PMIC_WD_LONGWIN_TIMEOUT_INT_SHIFT, .isMaskable = PMIC_IRQ_NON_MASKABLE}
};

static const uint8_t MaskableRegisters[NUM_MASKABLE_REGISTERS] = {
    PMIC_MASK_BUCK1_2_REG,
    PMIC_MASK_BUCK3_LDO_REG,
    PMIC_MASK_MISC_REG,
    PMIC_MASK_MODERATE_ERR_REG,
    PMIC_MASK_COMM_ERR_REG,
    PMIC_MASK_ESM_REG
};

static const uint8_t ClearableRegisters[NUM_CLEARABLE_REGISTERS] = {
    PMIC_INT_BUCK_LDO_REG,
    PMIC_INT_BUCK1_2_REG,
    PMIC_INT_BUCK3_LDO_REG,
    PMIC_INT_MISC_REG,
    PMIC_INT_MODERATE_ERR_REG,
    PMIC_INT_SEVERE_ERR_REG,
    PMIC_INT_FSM_ERR_REG,
    PMIC_INT_COMM_ERR_REG,
    PMIC_INT_ESM_REG,
    PMIC_WD_ERR_STATUS_REG
};

static inline void IRQ_copyIrqStat(const Pmic_IrqStatus_t *src, Pmic_IrqStatus_t *dst)
{
    (void)memmove((void *)dst, (const void *)src, sizeof(Pmic_IrqStatus_t));
}

static inline void IRQ_copyIrqMask(const Pmic_IrqMask_t *src, Pmic_IrqMask_t *dst)
{
    (void)memmove((void *)dst, (const void *)src, sizeof(Pmic_IrqMask_t));
}

/*!
 * @brief Function to set the intrStat bit position.
 */
static inline void IRQ_setIntrStat(Pmic_IrqStatus_t *irqStat, uint32_t irqNum)
{
    if (irqNum <= PMIC_IRQ_MAX)
    {
        // IRQs 0 to 31 go to index 0, IRQs 32 to 63 go to index 1.
        // At an index, the IRQ is stored at its corresponding bit
        // (e.g., IRQ 49's status will be stored at bit 17 at index 1)
        irqStat->intrStat[irqNum / PMIC_NUM_BITS_IN_INTR_STAT_ELEM] |= ((uint32_t)1UL << (irqNum % PMIC_NUM_BITS_IN_INTR_STAT_ELEM));
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
        irqMaskRegAddr = pmicIRQs[irqNum].maskRegAddr;
        irqMaskBitShift = pmicIRQs[irqNum].bitShift;
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
        // Modify IRQ mask bit field and write new register value back to PMIC
        Pmic_setBitField_b(&regData, irqMaskBitShift, shouldMask);
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
        } else {
            /* IRQ number is valid and maskable - status remains SUCCESS */
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
            anyRegs = (pmicIRQs[masks[i].irqNum].maskRegAddr == regAddr);
        }
        *anyMasks = anyRegs;
    }

    return status;
}

static int32_t IRQ_handleRecordsForReg(const Pmic_Handle_t *handle,
                                       uint8_t numMasks,
                                       const Pmic_IrqMask_t *masks,
                                       uint8_t regAddr,
                                       uint8_t *processedMasks)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    bool anyMasks = (bool)false;

    status = IRQ_anyMasksForReg(numMasks, masks, regAddr, &anyMasks);

    // Get the current value of this IRQ mask register
    Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);
    if ((status == PMIC_ST_SUCCESS) && anyMasks) {
        status = Pmic_ioRxByte(handle, regAddr, &regData);
    }

    if ((status == PMIC_ST_SUCCESS) && anyMasks) {
        for (uint8_t i = 0U; (i < PMIC_IRQ_LOOP_MAX) && (i < numMasks); i++) {
            const uint8_t irqNum = masks[i].irqNum;
            const Pmic_IrqInfo_t *pIrq = &pmicIRQs[irqNum];
            const uint8_t userMask = (uint8_t)(1UL << pIrq->bitShift);

            // If the current mask setting isn't targeted at the register we are
            // currently building, skip it
            if (regAddr != pIrq->maskRegAddr) {
                continue;
            }

            if (masks[i].mask != (bool)false) {
                regData |= userMask;
            } else {
                regData &= ~userMask;
            }

            if (*processedMasks < numMasks) {
                *processedMasks += 1U;
            }
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

    if ((status == PMIC_ST_SUCCESS) && (masks == NULL)) {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) && (numMasks > PMIC_IRQ_MAX)) {
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

static void IRQ_readOneMask(const Pmic_Handle_t *handle, Pmic_IrqMask_t *localMask, int32_t *status)
{
    uint8_t regData = 0U;
    const uint8_t irqNum = localMask->irqNum;
    uint8_t irqMaskRegAddr = 0U;
    uint8_t irqMaskBitShift = 0U;

    if (irqNum > PMIC_IRQ_MAX) {
        *status = PMIC_ST_ERR_INV_PARAM;
    } else {
        irqMaskRegAddr = pmicIRQs[irqNum].maskRegAddr;
        irqMaskBitShift = pmicIRQs[irqNum].bitShift;
    }

    if ((*status == PMIC_ST_SUCCESS) && (pmicIRQs[irqNum].isMaskable == PMIC_IRQ_NON_MASKABLE)) {
        *status = PMIC_ST_ERR_NOT_SUPPORTED;
    }

    if (*status == PMIC_ST_SUCCESS) {
        *status = Pmic_ioRxByte_CS(handle, irqMaskRegAddr, &regData);
    }

    if (*status == PMIC_ST_SUCCESS) {
        localMask->mask = Pmic_getBitField_b(regData, irqMaskBitShift);
    }
}

int32_t Pmic_irqGetMask(const Pmic_Handle_t *handle, uint8_t numIrqMasks, Pmic_IrqMask_t *irqMasks) {
    Pmic_IrqMask_t localMasks[PMIC_IRQ_NUM];
    int32_t status = Pmic_checkHandle(handle);

    if ((status == PMIC_ST_SUCCESS) && (irqMasks == NULL)) {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) && (numIrqMasks > PMIC_IRQ_MAX)) {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    if (status == PMIC_ST_SUCCESS) {
        for (uint8_t i = 0U; (i < PMIC_IRQ_LOOP_MAX) && (i < numIrqMasks); i++) {
            localMasks[i].irqNum = irqMasks[i].irqNum;
            IRQ_readOneMask(handle, &localMasks[i], &status);
            if (status != PMIC_ST_SUCCESS) {
                break;
            }
        }
    }

    // Only write to output array if all reads succeeded
    if (status == PMIC_ST_SUCCESS) {
        for (uint8_t i = 0U; (i < PMIC_IRQ_LOOP_MAX) && (i < numIrqMasks); i++) {
            IRQ_copyIrqMask(&localMasks[i], &irqMasks[i]);
        }
    }

    return Pmic_logStatus(handle, status);
}

static inline void IRQ_extractBits(Pmic_IrqStatus_t *irqStat, uint8_t regData, const uint8_t irqs[], uint8_t numIrqs) {
    for (uint8_t i = 0U; (i < PMIC_IRQ_LOOP_MAX) && (i < numIrqs); i++) {
        const uint8_t irqNum = irqs[i];
        if (Pmic_getBitField_b(regData, pmicIRQs[irqNum].bitShift)) {
            IRQ_setIntrStat(irqStat, irqNum);
        }
    }
}

static int32_t IRQ_readL2IntCommErr(const Pmic_Handle_t *handle, Pmic_IrqStatus_t *irqStat)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    // Read INT_COMM_ERR register
    status = Pmic_ioRxByte_CS(handle, PMIC_INT_COMM_ERR_REG, &regData);

    if (status == PMIC_ST_SUCCESS)
    {
        const uint8_t irqs[] = {
            PMIC_MCU_COMM_ERR_INT,
            PMIC_COMM_ADR_ERR_INT,
            PMIC_COMM_CRC_ERR_INT
        };

        IRQ_extractBits(irqStat, regData, irqs, (uint8_t)COUNT(irqs));
    }

    return status;
}

static int32_t IRQ_readL2IntEsm(const Pmic_Handle_t *handle, Pmic_IrqStatus_t *irqStat)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    // Read INT_ESM register
    status = Pmic_ioRxByte_CS(handle, PMIC_INT_ESM_REG, &regData);

    if (status == PMIC_ST_SUCCESS)
    {
        const uint8_t irqs[] = {
            PMIC_ESM_MCU_RST_INT,
            PMIC_ESM_MCU_FAIL_INT,
            PMIC_ESM_MCU_PIN_INT
        };

        IRQ_extractBits(irqStat, regData, irqs, (uint8_t)COUNT(irqs));
    }

    return status;
}

static int32_t IRQ_readL2WdErrStatus(const Pmic_Handle_t *handle, Pmic_IrqStatus_t *irqStat)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    // Read WD_ERR_STATUS register
    status = Pmic_ioRxByte_CS(handle, PMIC_WD_ERR_STATUS_REG, &regData);

    if (status == PMIC_ST_SUCCESS)
    {
        const uint8_t irqs[] = {
            PMIC_WD_RST_NMI,
            PMIC_WD_FAIL_NMI,
            PMIC_WD_LONGWIN_TIMEOUT_NMI
        };

        IRQ_extractBits(irqStat, regData, irqs, (uint8_t)COUNT(irqs));
    }

    return status;
}

static int32_t IRQ_readL1IntFsmErr(const Pmic_Handle_t *handle, Pmic_IrqStatus_t *irqStat)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    // Read INT_FSM_ERR register
    status = Pmic_ioRxByte_CS(handle, PMIC_INT_FSM_ERR_REG, &regData);

    // If WD_INT bit is set, read WD_ERR_STATUS register
    if ((status == PMIC_ST_SUCCESS) && Pmic_getBitField_b(regData, PMIC_WD_INT_SHIFT))
    {
        status = IRQ_readL2WdErrStatus(handle, irqStat);
    }

    // If COMM_ERR_INT bit is set, read INT_COMM_ERR register
    if ((status == PMIC_ST_SUCCESS) && Pmic_getBitField_b(regData, PMIC_COMM_ERR_INT_SHIFT))
    {
        status = IRQ_readL2IntCommErr(handle, irqStat);
    }

    // If ESM_MCU_INT bit is set, read INT_ESM register
    if ((status == PMIC_ST_SUCCESS) && Pmic_getBitField_b(regData, PMIC_ESM_MCU_INT_SHIFT))
    {
        status = IRQ_readL2IntEsm(handle, irqStat);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        const uint8_t irqs[] = {
            PMIC_WD_FIRST_NOK_NMI,
            PMIC_WAIT_FOR_PWRCYCLE_NMI,
            PMIC_WARM_RESET_NMI,
            PMIC_ORD_SHUTDOWN_NMI,
            PMIC_IMM_SHUTDOWN_NMI
        };

        IRQ_extractBits(irqStat, regData, irqs, (uint8_t)COUNT(irqs));
    }

    return status;
}

static int32_t IRQ_readL1IntSevereErr(const Pmic_Handle_t *handle, Pmic_IrqStatus_t *irqStat)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    // Read INT_SEVERE_ERR register
    status = Pmic_ioRxByte_CS(handle, PMIC_INT_SEVERE_ERR_REG, &regData);

    if (status == PMIC_ST_SUCCESS)
    {
        const uint8_t irqs[] = {
            PMIC_TSD_IMM_NMI
        };

        IRQ_extractBits(irqStat, regData, irqs, (uint8_t)COUNT(irqs));
    }

    return status;
}

static int32_t IRQ_readL1IntModerateErr(const Pmic_Handle_t *handle, Pmic_IrqStatus_t *irqStat)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    // Read INT_MODERATE_ERR register
    status = Pmic_ioRxByte_CS(handle, PMIC_INT_MODERATE_ERR_REG, &regData);

    if (status == PMIC_ST_SUCCESS)
    {
        const uint8_t irqs[] = {
            PMIC_GPO_READBACK_INT,
            PMIC_NINT_READBACK_INT,
            PMIC_CONFIG_CRC_INT,
            PMIC_TRIM_TEST_CRC_INT,
            PMIC_RECOV_CNT_NMI
        };

        IRQ_extractBits(irqStat, regData, irqs, (uint8_t)COUNT(irqs));
    }

    return status;
}

static int32_t IRQ_readL1IntMisc(const Pmic_Handle_t *handle, Pmic_IrqStatus_t *irqStat)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    // Read INT_MISC register
    status = Pmic_ioRxByte_CS(handle, PMIC_INT_MISC_REG, &regData);

    if (status == PMIC_ST_SUCCESS)
    {
        const uint8_t irqs[] = {
            PMIC_TWARN_INT,
            PMIC_B1_PVIN_UVLO_INT,
            PMIC_BUCKS_VSET_ERR_INT,
            PMIC_CFG_NVM_VERIFY_ERR_NMI,
            PMIC_CFG_NVM_VERIFY_DONE_NMI,
            PMIC_CFG_NVM_PRG_DONE_NMI,
            PMIC_ABIST_FAIL_INT,
            PMIC_ABIST_DONE_INT
        };

        IRQ_extractBits(irqStat, regData, irqs, (uint8_t)COUNT(irqs));
    }

    return status;
}

static int32_t IRQ_readL2IntBuck3Ldo(const Pmic_Handle_t *handle, Pmic_IrqStatus_t *irqStat)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    // Read INT_BUCK3_LDO register
    status = Pmic_ioRxByte_CS(handle, PMIC_INT_BUCK3_LDO_REG, &regData);

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

        IRQ_extractBits(irqStat, regData, irqs, (uint8_t)COUNT(irqs));
    }

    return status;
}

static int32_t IRQ_readL2IntBuck1_2(const Pmic_Handle_t *handle, Pmic_IrqStatus_t *irqStat)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    // Read INT_BUCK1_2 register
    status = Pmic_ioRxByte_CS(handle, PMIC_INT_BUCK1_2_REG, &regData);

    if (status == PMIC_ST_SUCCESS)
    {
        const uint8_t irqs[6] = {
            PMIC_BUCK2_OVP_INT,
            PMIC_BUCK2_UV_INT,
            PMIC_BUCK2_OV_INT,
            PMIC_BUCK1_OVP_INT,
            PMIC_BUCK1_UV_INT,
            PMIC_BUCK1_OV_INT
        };

        IRQ_extractBits(irqStat, regData, irqs, (uint8_t)COUNT(irqs));
    }

    return status;
}

static int32_t IRQ_readL1IntBuckLdo(const Pmic_Handle_t *handle, Pmic_IrqStatus_t *irqStat)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    // Read INT_BUCK_LDO register
    status = Pmic_ioRxByte_CS(handle, PMIC_INT_BUCK_LDO_REG, &regData);

    if (status == PMIC_ST_SUCCESS)
    {
        const uint8_t irqs[4] = {
            PMIC_LDO_SC_NMI,
            PMIC_BUCK3_SC_NMI,
            PMIC_BUCK2_SC_NMI,
            PMIC_BUCK1_SC_NMI
        };

        IRQ_extractBits(irqStat, regData, irqs, (uint8_t)COUNT(irqs));

        // If LDO_INT bit or BUCK3_INT bit is set, read INT_BUCK3_LDO register
        if (Pmic_getBitField_b(regData, PMIC_LDO_INT_SHIFT) || Pmic_getBitField_b(regData, PMIC_BUCK3_INT_SHIFT))
        {
            status = IRQ_readL2IntBuck3Ldo(handle, irqStat);
        }

        // If BUCK2_INT bit or BUCK1_INT bit is set is set, read INT_BUCK1_2 register
        if ((status == PMIC_ST_SUCCESS) &&
            (Pmic_getBitField_b(regData, PMIC_BUCK2_INT_SHIFT) || Pmic_getBitField_b(regData, PMIC_BUCK1_INT_SHIFT)))
        {
            status = IRQ_readL2IntBuck1_2(handle, irqStat);
        }
    }

    return status;
}

static int32_t IRQ_readL0(const Pmic_Handle_t *handle, Pmic_IrqStatus_t *irqStat)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    // Read INT_TOP register
    status = Pmic_ioRxByte_CS(handle, PMIC_INT_TOP_REG, &regData);

    // If FSM_ERR_INT bit is set, read INT_FSM_ERR register
    if ((status == PMIC_ST_SUCCESS) && Pmic_getBitField_b(regData, PMIC_FSM_ERR_INT_SHIFT))
    {
        status = IRQ_readL1IntFsmErr(handle, irqStat);
    }

    // If SEVERE_ERR_INT bit is set, read INT_SEVERE_ERR register
    if ((status == PMIC_ST_SUCCESS) && Pmic_getBitField_b(regData, PMIC_SEVERE_ERR_INT_SHIFT))
    {
        status = IRQ_readL1IntSevereErr(handle, irqStat);
    }

    // If MODERATE_ERR_INT bit is set, read INT_MODERATE_ERR register
    if ((status == PMIC_ST_SUCCESS) && Pmic_getBitField_b(regData, PMIC_MODERATE_ERR_INT_SHIFT))
    {
        status = IRQ_readL1IntModerateErr(handle, irqStat);
    }

    // If MISC_INT bit is set, read INT_MISC register
    if ((status == PMIC_ST_SUCCESS) && Pmic_getBitField_b(regData, PMIC_MISC_INT_SHIFT))
    {
        status = IRQ_readL1IntMisc(handle, irqStat);
    }

    // If BUCK_LDO_INT bit is set, read INT_BUCK_LDO register
    if ((status == PMIC_ST_SUCCESS) && Pmic_getBitField_b(regData, PMIC_BUCK_LDO_INT_SHIFT))
    {
        status = IRQ_readL1IntBuckLdo(handle, irqStat);
    }

    return status;
}

int32_t Pmic_irqGetStatus(const Pmic_Handle_t *handle, Pmic_IrqStatus_t *irqStat)
{
    Pmic_IrqStatus_t localStat = (Pmic_IrqStatus_t){0};
    int32_t status = Pmic_checkHandle(handle);

    if ((status == PMIC_ST_SUCCESS) && (irqStat == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        (void)memset(localStat.intrStat, 0, sizeof(localStat.intrStat));

        status = IRQ_readL0(handle, &localStat);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        IRQ_copyIrqStat(&localStat, irqStat);
    }

    return Pmic_logStatus(handle, status);
}

static uint8_t IRQ_getNextFlag(Pmic_IrqStatus_t *irqStat)
{
    uint8_t idx = 0U, bitPos = 0U;
    bool foundFlag = (bool)false;

    // For each element in struct member intrStat of irqStat...
    for (idx = 0U; idx < PMIC_NUM_ELEM_IN_INTR_STAT; idx++)
    {
        // If current element has no IRQ statuses set, move onto next element
        if (irqStat->intrStat[idx] == 0U)
        {
            continue;
        }

        // For each bit in the element...
        for (bitPos = 0U; bitPos < PMIC_NUM_BITS_IN_INTR_STAT_ELEM; bitPos++)
        {
            const uint32_t mask = (uint32_t)(1UL << bitPos);
            // If the bit is set...
            if ((irqStat->intrStat[idx] & mask) != 0U)
            {
                // Clear bit in intrStat element and exit loop
                irqStat->intrStat[idx] &= ~mask;
                foundFlag = (bool)true;
                break;
            }
        }

        if (foundFlag)
        {
            break;
        }
    } /* LCOV_EXCL_LINE */

    // Return the corresponding IRQ number
    return (bitPos + (PMIC_NUM_BITS_IN_INTR_STAT_ELEM * idx));
}

int32_t Pmic_irqGetNextFlag(const Pmic_Handle_t *handle, Pmic_IrqStatus_t *irqStat, uint8_t *irqNum)
{
    int32_t status = PMIC_ST_SUCCESS;
    bool irqStatEmpty = (bool)false;

    if ((irqStat == NULL) || (irqNum == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        irqStatEmpty = (irqStat->intrStat[0U] == 0U) && (irqStat->intrStat[1U] == 0U);
    }

    if ((status == PMIC_ST_SUCCESS) && irqStatEmpty)
    {
        status = PMIC_ST_WARN_NO_IRQ_REMAINING;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        *irqNum = IRQ_getNextFlag(irqStat);
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_irqGetFlag(const Pmic_Handle_t *handle, uint8_t irqNum, bool *flag)
{
    uint8_t regData = 0U;
    int32_t status = Pmic_checkHandle(handle);

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
        status = Pmic_ioRxByte_CS(handle, pmicIRQs[irqNum].statRegAddr, &regData);
    }

    // Extract IRQ status
    if (status == PMIC_ST_SUCCESS)
    {
        *flag = Pmic_getBitField_b(regData, pmicIRQs[irqNum].bitShift);
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_irqClrFlag(const Pmic_Handle_t *handle, uint8_t irqNum)
{
    uint8_t regData = 0U;
    int32_t status = Pmic_checkHandle(handle);

    if ((status == PMIC_ST_SUCCESS) && (irqNum > PMIC_IRQ_MAX))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // IRQ statuses are W1C - write 1 to clear
        Pmic_setBitField(&regData, pmicIRQs[irqNum].bitShift, (uint8_t)(1UL << pmicIRQs[irqNum].bitShift), 1U);

        // Write data to PMIC
        status = Pmic_ioTxByte_CS(handle, pmicIRQs[irqNum].statRegAddr, regData);
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_irqClrAllFlags(const Pmic_Handle_t *handle) {
    int32_t status = Pmic_checkHandle(handle);

    // All IRQ statuses are W1C, writing to reserved bits has no effect, so just
    // write every bit to 1
    Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);
    for (uint8_t i = 0U; i < NUM_CLEARABLE_REGISTERS; i++) {
        if (status == PMIC_ST_SUCCESS) {
            status = Pmic_ioTxByte(handle, ClearableRegisters[i], CLEAR_ALL_STAT_BITS);
        } else {
            break;
        }
    }
    Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);

    return Pmic_logStatus(handle, status);
}
