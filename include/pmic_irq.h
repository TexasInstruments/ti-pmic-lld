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
 * @file pmic_irq.h
 *
 * @brief PMIC LLD IRQ module header file.
 *
 * @details This module contains declarations/definitions of macros, data
 * structures, and APIs used to interact with PMIC interrupt requests (IRQs).
 * Some components of the PMIC IRQ module are as follows: unmasking/masking
 * IRQs, getting the masking status of IRQs, getting IRQ statuses, and
 * clearing IRQ flags.
 */
#ifndef PMIC_IRQ_H
#define PMIC_IRQ_H

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                Include Files                               */
/* ========================================================================== */
#include <stdint.h>

#include "pmic_common.h"

/* ========================================================================== */
/*                               Macros & Typedefs                            */
/* ========================================================================== */

/**
 * @anchor Pmic_IRQs
 * @name Interrupt Requests
 *
 * @brief Defines to be passed into IRQ module APIs to unmask/mask, get IRQ
 * masking status, get IRQ flag status, and clear IRQ flags.
 *
 * @note The IRQs listed here are all clearable; Those ending with "NMI" are
 * non-maskable.
 *
 * @{
 */
#define PMIC_BUCK1_SC_NMI            ((uint8_t)0U)
#define PMIC_BUCK1_OV_INT            ((uint8_t)1U)
#define PMIC_BUCK1_UV_INT            ((uint8_t)2U)
#define PMIC_BUCK1_RV_INT            ((uint8_t)3U)
#define PMIC_BUCK1_ILIM_INT          ((uint8_t)4U)
#define PMIC_BUCK2_SC_NMI            ((uint8_t)5U)
#define PMIC_BUCK2_OV_INT            ((uint8_t)6U)
#define PMIC_BUCK2_UV_INT            ((uint8_t)7U)
#define PMIC_BUCK2_RV_INT            ((uint8_t)8U)
#define PMIC_BUCK2_ILIM_INT          ((uint8_t)9U)
#define PMIC_BUCK3_SC_NMI            ((uint8_t)10U)
#define PMIC_BUCK3_OV_INT            ((uint8_t)11U)
#define PMIC_BUCK3_UV_INT            ((uint8_t)12U)
#define PMIC_BUCK3_RV_INT            ((uint8_t)13U)
#define PMIC_BUCK3_ILIM_INT          ((uint8_t)14U)
#define PMIC_LDO_LS1_VMON1_SC_INT    ((uint8_t)15U)
#define PMIC_LDO_LS1_VMON1_OV_INT    ((uint8_t)16U)
#define PMIC_LDO_LS1_VMON1_UV_INT    ((uint8_t)17U)
#define PMIC_LDO_LS1_VMON1_RV_INT    ((uint8_t)18U)
#define PMIC_LDO_LS1_VMON1_ILIM_INT  ((uint8_t)19U)
#define PMIC_LS2_VMON2_SC_INT        ((uint8_t)20U)
#define PMIC_LS2_VMON2_OV_INT        ((uint8_t)21U)
#define PMIC_LS2_VMON2_UV_INT        ((uint8_t)22U)
#define PMIC_LS2_VMON2_RV_INT        ((uint8_t)23U)
#define PMIC_LS2_ILIM_INT            ((uint8_t)24U)
#define PMIC_VCCA_OV_INT             ((uint8_t)25U)
#define PMIC_VCCA_UV_INT             ((uint8_t)26U)
#define PMIC_STARTUP_ENABLE_INT      ((uint8_t)27U)
#define PMIC_MISC_ABIST_FAIL_INT     ((uint8_t)28U)
#define PMIC_MISC_BUCKS_VSET_ERR_INT ((uint8_t)29U)
#define PMIC_MISC_EXT_CLK_INT        ((uint8_t)30U)
#define PMIC_MISC_TWARN_INT          ((uint8_t)31U)
#define PMIC_ME_TSD_ORD_INT          ((uint8_t)32U)
#define PMIC_ME_RECOV_CNT_INT        ((uint8_t)33U)
#define PMIC_ME_TRIM_TEST_CRC_INT    ((uint8_t)34U)
#define PMIC_ME_CONFIG_CRC_INT       ((uint8_t)35U)
#define PMIC_ME_NINT_READBACK_INT    ((uint8_t)36U)
#define PMIC_ME_NRSTOUT_READBACK_INT ((uint8_t)37U)
#define PMIC_SE_TSD_IMM_INT          ((uint8_t)38U)
#define PMIC_SE_VCCA_OVP_INT         ((uint8_t)39U)
#define PMIC_COMM_FRM_ERR_INT        ((uint8_t)40U)
#define PMIC_COMM_CRC_ERR_INT        ((uint8_t)41U)
#define PMIC_COMM_ADR_ERR_INT        ((uint8_t)42U)
#define PMIC_COMM_MCU_ERR_INT        ((uint8_t)43U)
#define PMIC_ESM_MCU_PIN_INT         ((uint8_t)44U)
#define PMIC_ESM_MCU_FAIL_INT        ((uint8_t)45U)
#define PMIC_ESM_MCU_RST_INT         ((uint8_t)46U)
#define PMIC_WDG_FIRST_NOK_INT       ((uint8_t)47U)
#define PMIC_WDG_LONGWIN_TIMEOUT_INT ((uint8_t)48U)
#define PMIC_WDG_TIMEOUT_INT         ((uint8_t)49U)
#define PMIC_WDG_ANSWER_EARLY_INT    ((uint8_t)50U)
#define PMIC_WDG_SEQ_ERR_INT         ((uint8_t)51U)
#define PMIC_WDG_ANSWER_ERR_INT      ((uint8_t)52U)
#define PMIC_WDG_FAIL_INT            ((uint8_t)53U)
#define PMIC_WDG_RST_INT             ((uint8_t)54U)
#define PMIC_REGULATOR_ERR_INT       ((uint8_t)55U)
#define PMIC_FSM_IMM_SHUTDOWN_INT    ((uint8_t)56U)
#define PMIC_FSM_ORD_SHUTDOWN_INT    ((uint8_t)57U)
#define PMIC_FSM_WARM_RESET_INT      ((uint8_t)58U)
#define PMIC_IRQ_MAX                 (PMIC_FSM_WARM_RESET_INT)
#define PMIC_IRQ_NUM                 (PMIC_IRQ_MAX + 1U)
#define PMIC_IRQ_ALL                 ((uint8_t)0xFFU)
/** @} */

// Defines used internally only
#define PMIC_NUM_ELEM_IN_INTR_STAT   ((uint8_t)2U)
#define PMIC_NUM_BITS_IN_INTR_STAT   ((uint8_t)32U)

/* ========================================================================== */
/*                             Structures and Enums                           */
/* ========================================================================== */

/**
 * @anchor Pmic_IrqMask
 * @name PMIC IRQ Mask Struct
 *
 * @brief Struct used to describe the mask configuration of an IRQ.
 *
 * @param irqNum Target PMIC IRQ number. For valid values, refer to @ref Pmic_IRQs.
 * @param mask Mask configuration of the IRQ. When set to true, the IRQ is
 * masked (indicating that it will not trigger). Otherwise, the IRQ is unmasked.
 */
typedef struct Pmic_IrqMask_s {
    uint8_t irqNum;
    bool mask;
} Pmic_IrqMask_t;

/**
 * @anchor Pmic_IrqStat
 * @name PMIC IRQ Status Struct
 *
 * @brief Struct used to hold the status of all PMIC IRQs.
 *
 * @param intrStat For each element in this array, an individual bit represents
 * whether a PMIC IRQ is set. For instance, Bit 0 of element 0 is the 1st PMIC
 * IRQ status, bit 0 of element 1 is the 32nd PMIC IRQ status.
 */
typedef struct Pmic_IrqStat_s {
    uint32_t intrStat[PMIC_NUM_ELEM_IN_INTR_STAT];
} Pmic_IrqStat_t;

/* ========================================================================== */
/*                             Function Declarations                          */
/* ========================================================================== */

/**
 * @brief Set the mask configuration for a single PMIC IRQ.
 *
 * @param pmicHandle [IN] PMIC interface handle.
 * @param irqNum     [IN] Number of IRQ mask configurations to set.
 * @param shouldMask [IN] Whether this IRQ should be masked or not. See @ref
 * Pmic_IrqMaskControl.
 *
 * @return Success code if IRQ mask configuration(s) have been set, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_irqSetMask(Pmic_CoreHandle_t *handle, uint8_t irqNum, bool shouldMask);

/**
 * @brief Set the mask configuration for multiple PMIC IRQs.
 *
 * @param pmicHandle  [IN] PMIC interface handle.
 * @param numIrqMasks [IN] Number of IRQ mask configurations to set.
 * @param irqMasks    [IN] Array of IRQ mask configurations.
 *
 * @return Success code if IRQ mask configuration(s) have been set, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_irqSetMasks(Pmic_CoreHandle_t *handle, uint8_t numIrqMasks, const Pmic_IrqMask_t *irqMasks);

/**
 * @brief Get the mask configuration for PMIC IRQs.
 *
 * @param pmicHandle  [IN] PMIC interface handle.
 * @param numIrqMasks [IN] Number of IRQ mask configurations to obtain.
 * @param irqMasks    [OUT] Array of IRQ mask configurations.
 *
 * @return Success code if IRQ mask configuration(s) have been obtained, error
 * code otherwise. For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_irqGetMask(Pmic_CoreHandle_t *handle, uint8_t numIrqMasks, Pmic_IrqMask_t *irqMasks);

/**
 * @brief Get the status of all PMIC IRQs.
 *
 * @attention End-user must call this API first before calling `Pmic_irqGetNextFlag()`.
 *
 * @param pmicHandle [IN] PMIC interface handle.
 * @param irqStat    [OUT] Status of all PMIC IRQs.
 *
 * @return Success code if all PMIC IRQ statuses have been obtained, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_irqGetStat(Pmic_CoreHandle_t *handle, Pmic_IrqStat_t *irqStat);

/**
 * @brief Get the next PMIC IRQ that has its flag set (status bit set to 1).
 *
 * @attention End-user must call `Pmic_irqGetStat()` first to get all PMIC IRQ
 * statuses. Once the IRQ statuses have been obtained, it is passed as input to
 * this API so that the next IRQ flag can be discovered. Once the next flag is
 * found, end-user can call `Pmic_irqClrFlag()` to clear the flag.
 *
 * @param handle [IN] PMIC interface handle.
 * @param irqStat [IN/OUT] Status of all PMIC IRQs. Once the next IRQ flag has
 * been found, the corresponding status bit in struct member `intrStat` will be
 * cleared.
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
 * @param irqNum [IN] Target PMIC IRQ. For valid values, refer to @ref Pmic_IRQs.
 * @param flag [OUT] Status flag of the target PMIC IRQ. This parameter returned
 * as true indicates that the target IRQ's status flag is set to 1. Otherwise,
 * the target IRQ's status flag is set to 0.
 *
 * @return Success code if the PMIC IRQ flag has been obtained, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_irqGetFlag(Pmic_CoreHandle_t *handle, uint8_t irqNum, bool *flag);

/**
 * @brief Clear a specific PMIC IRQ flag.
 *
 * @attention This API is meant to be called after getting the next flag status from
 * `Pmic_irqGetNextFlag()` or getting a specific flag status from `Pmic_irqGetFlag()`
 *
 * @param handle [IN] PMIC interface handle.
 * @param irqNum [IN] Target PMIC IRQ to clear. For valid values, refer to @ref
 * Pmic_IRQs.
 *
 * @return Success code if the PMIC IRQ flag has been cleared, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_irqClrFlag(Pmic_CoreHandle_t *handle, uint8_t irqNum);

/**
 * @brief Clear all PMIC IRQ flags.
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @return Success code if all PMIC IRQ flags have been cleared, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_irqClrAllFlags(Pmic_CoreHandle_t *handle);

#ifdef __cplusplus
}

#endif /* __cplusplus */
#endif /* PMIC_IRQ_H */
