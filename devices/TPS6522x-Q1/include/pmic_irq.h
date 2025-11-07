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
#ifndef PMIC_IRQ_H
#define PMIC_IRQ_H

/**
 * @file pmic_irq.h
 *
 * @brief PMIC IRQ interface. Contains APIs, macros/defines, and data structures
 * used to configure, control, and interact with PMIC interrupt-related features.
 */

/* ========================================================================= */
/*                              Include Files                                */
/* ========================================================================= */

#include "pmic_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                            Macros & Typedefs                               */
/* ========================================================================== */

/**
 * @anchor Pmic_IrqNum
 * @name PMIC Interrupt Requests
 *
 * @brief Identifiers for PMIC interrupt requests.
 *
 * @{
 */
#define PMIC_IRQ_WD_RST_NMI             (0U)
#define PMIC_IRQ_WD_FAIL_NMI            (1U)
#define PMIC_IRQ_WD_LONGWIN_TIMEOUT_NMI (2U)
#define PMIC_IRQ_ESM_MCU_RST_INT        (3U)
#define PMIC_IRQ_ESM_MCU_FAIL_INT       (4U)
#define PMIC_IRQ_ESM_MCU_PIN_INT        (5U)
#define PMIC_IRQ_I2C2_ERR_INT           (6U)
#define PMIC_IRQ_COMM_ERR_INT           (7U)
#define PMIC_IRQ_SOC_PWR_ERR_INT        (8U)
#define PMIC_IRQ_MCU_PWR_ERR_INT        (9U)
#define PMIC_IRQ_ORD_SHUTDOWN_INT       (10U)
#define PMIC_IRQ_IMM_SHUTOWN_INT        (11U)
#define PMIC_IRQ_BG_XMON_INT            (12U)
#define PMIC_IRQ_PFSM_ERR_INT           (13U)
#define PMIC_IRQ_VCCA_OVP_INT           (14U)
#define PMIC_IRQ_TSD_IMM_INT            (15U)
#define PMIC_IRQ_RECOV_CNT_INT          (16U)
#define PMIC_IRQ_REG_CRC_ERR_INT        (17U)
#define PMIC_IRQ_BIST_FAIL_INT          (18U)
#define PMIC_IRQ_TSD_ORD_INT            (19U)
#define PMIC_IRQ_ADC_CONV_READY_INT     (20U)
#define PMIC_IRQ_PB_RISE_INT            (21U)
#define PMIC_IRQ_PB_FALL_INT            (22U)
#define PMIC_IRQ_PB_LONG_INT            (23U)
#define PMIC_IRQ_TWARN_INT              (24U)
#define PMIC_IRQ_REG_UNLOCK_INT         (25U)
#define PMIC_IRQ_EXT_CLK_INT            (26U)
#define PMIC_IRQ_BIST_PASS_INT          (27U)
#define PMIC_IRQ_SOFT_REBOOT_INT        (28U)
#define PMIC_IRQ_FSD_INT                (29U)
#define PMIC_IRQ_PB_SHORT_INT           (30U)
#define PMIC_IRQ_ENABLE_INT             (31U)
#define PMIC_IRQ_VSENSE_INT             (32U)
#define PMIC_IRQ_GPIO6_INT              (33U)
#define PMIC_IRQ_GPIO5_INT              (34U)
#define PMIC_IRQ_GPIO4_INT              (35U)
#define PMIC_IRQ_GPIO3_INT              (36U)
#define PMIC_IRQ_GPIO2_INT              (37U)
#define PMIC_IRQ_GPIO1_INT              (38U)
#define PMIC_IRQ_VMON2_UVOV_INT         (39U)
#define PMIC_IRQ_VMON1_UVOV_INT         (40U)
#define PMIC_IRQ_VCCA_UVOV_INT          (41U)
#define PMIC_IRQ_LDO3_UVOV_INT          (42U)
#define PMIC_IRQ_LDO2_UVOV_INT          (43U)
#define PMIC_IRQ_LDO1_UVOV_INT          (44U)
#define PMIC_IRQ_BUCK4_UVOV_INT         (45U)
#define PMIC_IRQ_BUCK3_UVOV_INT         (46U)
#define PMIC_IRQ_BUCK2_UVOV_INT         (47U)
#define PMIC_IRQ_BUCK1_UVOV_INT         (48U)
#define PMIC_IRQ_INT_MIN                (PMIC_IRQ_WD_RST_INT)
#define PMIC_IRQ_INT_MAX                (PMIC_IRQ_BUCK1_UVOV_INT)
/** @} */

/**
 * @anchor Pmic_IrqMaskControl
 * @name PMIC IRQ Mask Control
 *
 * @brief values used to mask/unmask PMIC interrupts.
 *
 * @{
 */
#define PMIC_IRQ_MASK   ((bool)true)
#define PMIC_IRQ_UNMASK ((bool)false)
/** @} */

// For internal use only
#define PMIC_NUM_ELEM_IN_INTR_STAT ((uint8_t)2U)
#define PMIC_NUM_BITS_IN_INTR_STAT ((uint8_t)32U)

/* ========================================================================== */
/*                           Structures and Enums                             */
/* ========================================================================== */

/**
 * @anchor Pmic_IrqMask
 * @name PMIC IRQ Mask Structure
 *
 * @brief Structure to represent the mask state of a PMIC interrupt.
 *
 * @param irqNum The interrupt number.
 *
 * @param mask The mask state. True - masked, false - unmasked.
 */
typedef struct Pmic_IrqMask_s {
    uint8_t irqNum;
    bool mask;
} Pmic_IrqMask_t;

/**
 * @anchor Pmic_IrqStat
 * @name PMIC IRQ Status Structure
 *
 * @brief Structure used to hold the status of all PMIC IRQs.
 *
 * @param intrStat For each element in this array, an individual bit represents
 * whether a PMIC IRQ is set. For instance, Bit 0 of element 0 is the 1st PMIC
 * IRQ status, bit 0 of element 1 is the 32nd PMIC IRQ status.
 */
typedef struct Pmic_IrqStat_s {
    uint32_t intrStat[PMIC_NUM_ELEM_IN_INTR_STAT];
} Pmic_IrqStat_t;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 * @brief Set the mask configuration for a single PMIC IRQ.
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @param irqNum [IN] Number of IRQ mask configurations to set.
 *
 * @param shouldMask [IN] Whether this IRQ should be masked or not. See @ref
 * Pmic_IrqMaskControl.
 *
 * @return Success code if IRQ mask configuration(s) have been set, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_irqSetMask(Pmic_Handle_t *handle, uint8_t irqNum, bool shouldMask);

/**
 * @brief Set the mask configuration for multiple PMIC IRQs.
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @param numIrqMasks [IN] Number of IRQ mask configurations to set.
 *
 * @param irqMasks [IN] Array of IRQ mask configurations.
 *
 * @return Success code if IRQ mask configuration(s) have been set, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_irqSetMasks(Pmic_Handle_t *handle, uint8_t numIrqMasks, const Pmic_IrqMask_t *irqMasks);

/**
 * @brief Get the mask configuration for PMIC IRQs.
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @param numIrqMasks [IN] Number of IRQ mask configurations to obtain.
 *
 * @param irqMasks [OUT] Array of IRQ mask configurations.
 *
 * @return Success code if IRQ mask configuration(s) have been obtained, error
 * code otherwise. For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_irqGetMask(Pmic_Handle_t *handle, uint8_t numIrqMasks, Pmic_IrqMask_t *irqMasks);

/**
 * @brief Get the status of all PMIC IRQs.
 *
 * @attention End-user must call this API first before calling `Pmic_irqGetNextFlag()`.
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @param irqStat [OUT] Status of all PMIC IRQs.
 *
 * @return Success code if all PMIC IRQ statuses have been obtained, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_irqGetStat(Pmic_Handle_t *handle, Pmic_IrqStat_t *irqStat);

/**
 * @brief Get the next PMIC IRQ that has its flag set (status bit set to 1).
 *
 * @attention End-user must call `Pmic_irqGetStat()` first to get all PMIC IRQ
 * statuses. Once the IRQ statuses have been obtained, it is passed as input to
 * this API so that the next IRQ flag can be discovered. Once the next flag is
 * found, end-user can call `Pmic_irqClrFlag()` to clear the flag.
 *
 * @param irqStat [IN/OUT] Status of all PMIC IRQs. Once the next IRQ flag has
 * been found, the corresponding status bit in struct member `intrStat` will be
 * cleared.
 *
 * @param irqNum [OUT] The next IRQ that has its flag set. For valid values,
 * refer to @ref Pmic_IRQs.
 *
 * @return Success code if the next IRQ that has its flag set has been obtained,
 * error code otherwise. For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_irqGetNextFlag(Pmic_IrqStat_t *irqStat, uint8_t *irqNum);

/**
 * @brief Get the flag status of a specific IRQ.
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param irqNum [IN] Target PMIC IRQ. For valid values, refer to @ref Pmic_IRQs.
 *
 * @param flag [OUT] Status flag of the target PMIC IRQ. This parameter returned
 * as true indicates that the target IRQ's status flag is set to 1. Otherwise,
 * the target IRQ's status flag is set to 0.
 *
 * @return Success code if the PMIC IRQ flag has been obtained, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_irqGetFlag(Pmic_Handle_t *handle, uint8_t irqNum, bool *flag);

/**
 * @brief Clear a specific PMIC IRQ flag.
 *
 * @attention This API is meant to be called after getting the next flag status from
 * `Pmic_irqGetNextFlag()` or getting a specific flag status from `Pmic_irqGetFlag()`
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param irqNum [IN] Target PMIC IRQ to clear. For valid values, refer to @ref
 * Pmic_IRQs.
 *
 * @return Success code if the PMIC IRQ flag has been cleared, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_irqClrFlag(Pmic_Handle_t *handle, uint8_t irqNum);

/**
 * @brief Clear all PMIC IRQ flags.
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @return Success code if all PMIC IRQ flags have been cleared, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_irqClrAllFlags(Pmic_Handle_t *handle);

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* PMIC_IRQ_H */
