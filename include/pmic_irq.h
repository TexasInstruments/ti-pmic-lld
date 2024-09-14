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
 */
#ifndef __PMIC_IRQ_H__
#define __PMIC_IRQ_H__

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                Include Files                               */
/* ========================================================================== */
#include "pmic_common.h"

/* ========================================================================== */
/*                              Macros & Typedefs                             */
/* ========================================================================== */

/**
 * @anchor Pmic_IRQs
 * @name TPS65385x Interrupt Requests
 *
 * @brief Definitions of the PMIC statuses that can be indicated in each PMIC
 * response within the status byte during an I/O transfer. Passed as input into
 * IRQ APIs to accomplish various tasks such as getting IRQ statuses and
 * clearing IRQ flags.
 *
 * @attention All statuses listed are nonmaskable.
 *
 * @{
 */
#define PMIC_IRQ_VBATL_OV                   ((uint8_t)0U)
#define PMIC_IRQ_VBATL_UV                   ((uint8_t)1U)
#define PMIC_IRQ_VCP_UV                     ((uint8_t)2U)
#define PMIC_IRQ_VDD6_OV                    ((uint8_t)3U)
#define PMIC_IRQ_VDD6_UV                    ((uint8_t)4U)
#define PMIC_IRQ_VDD5_OV                    ((uint8_t)5U)
#define PMIC_IRQ_VDD5_UV                    ((uint8_t)6U)
#define PMIC_IRQ_VDD_3P5_OV                 ((uint8_t)7U)
#define PMIC_IRQ_VDD_3P5_UV                 ((uint8_t)8U)
#define PMIC_IRQ_VREG_UV                    ((uint8_t)9U)
#define PMIC_IRQ_VDD6_LP_UV                 ((uint8_t)10U)
#define PMIC_IRQ_VSOUT2_OV                  ((uint8_t)11U)
#define PMIC_IRQ_VSOUT2_UV                  ((uint8_t)12U)
#define PMIC_IRQ_VSOUT1_OV                  ((uint8_t)13U)
#define PMIC_IRQ_VSOUT1_UV                  ((uint8_t)14U)
#define PMIC_IRQ_VDD6_OT_FLAG               ((uint8_t)15U)
#define PMIC_IRQ_VDD6_TP                    ((uint8_t)16U)
#define PMIC_IRQ_VDD5_ILIM                  ((uint8_t)17U)
#define PMIC_IRQ_VDD5_OT_FLAG               ((uint8_t)18U)
#define PMIC_IRQ_VDD5_TP                    ((uint8_t)19U)
#define PMIC_IRQ_VDD_3P5_ILIM               ((uint8_t)20U)
#define PMIC_IRQ_VDD_3P5_OT_FLAG            ((uint8_t)21U)
#define PMIC_IRQ_VDD_3P5_TP                 ((uint8_t)22U)
#define PMIC_IRQ_VDD6_ILIM                  ((uint8_t)23U)
#define PMIC_IRQ_VSOUT2_ILIM                ((uint8_t)24U)
#define PMIC_IRQ_VSOUT2_OT_FLAG             ((uint8_t)25U)
#define PMIC_IRQ_VSOUT2_TP                  ((uint8_t)26U)
#define PMIC_IRQ_VSOUT1_ILIM                ((uint8_t)27U)
#define PMIC_IRQ_VSOUT1_OT_FLAG             ((uint8_t)28U)
#define PMIC_IRQ_VSOUT1_TP                  ((uint8_t)29U)
#define PMIC_IRQ_CFG_CRC_ERR                ((uint8_t)30U)
#define PMIC_IRQ_EE_CRC_ERR                 ((uint8_t)31U)
#define PMIC_IRQ_NRES_ERR                   ((uint8_t)32U)
#define PMIC_IRQ_LBIST_ERR                  ((uint8_t)33U)
#define PMIC_IRQ_ABIST_ERR                  ((uint8_t)34U)
#define PMIC_IRQ_SPI_CMD_ERR                ((uint8_t)35U)
#define PMIC_IRQ_SPI_FORMAT_ERR             ((uint8_t)36U)
#define PMIC_IRQ_SPI_DATA_OUTPUT_MISMATCH   ((uint8_t)37U)
#define PMIC_IRQ_LO_LPMCLK                  ((uint8_t)38U)
#define PMIC_IRQ_ENDRV_ERR                  ((uint8_t)39U)
#define PMIC_IRQ_TRIM_ERR_VMON              ((uint8_t)40U)
#define PMIC_IRQ_INVALID_CMD                ((uint8_t)41U)
#define PMIC_IRQ_UNDEF_CMD                  ((uint8_t)42U)
#define PMIC_IRQ_CRC_ERR                    ((uint8_t)43U)
#define PMIC_IRQ_LONG_FRM                   ((uint8_t)44U)
#define PMIC_IRQ_SHORT_FRM                  ((uint8_t)45U)
#define PMIC_IRQ_ERROR_PIN_FAIL             ((uint8_t)46U)
#define PMIC_IRQ_WD_FAIL                    ((uint8_t)47U)
#define PMIC_IRQ_DIAG_STATE_TO              ((uint8_t)48U)
#define PMIC_IRQ_SAM_BIST_FAIL              ((uint8_t)49U)
#define PMIC_IRQ_POWER_ON_RESET             ((uint8_t)50U)
#define PMIC_IRQ_SAM_CNT_ERR                ((uint8_t)51U)
#define PMIC_IRQ_COS_P_OV                   ((uint8_t)52U)
#define PMIC_IRQ_COS_P_UV                   ((uint8_t)53U)
#define PMIC_IRQ_COS_N_OV                   ((uint8_t)54U)
#define PMIC_IRQ_COS_N_UV                   ((uint8_t)55U)
#define PMIC_IRQ_SIN_P_OV                   ((uint8_t)56U)
#define PMIC_IRQ_SIN_P_UV                   ((uint8_t)57U)
#define PMIC_IRQ_SIN_N_OV                   ((uint8_t)58U)
#define PMIC_IRQ_SIN_N_UV                   ((uint8_t)59U)
#define PMIC_IRQ_MAX                        (PMIC_IRQ_SIN_N_UV)
#define PMIC_IRQ_NUM                        (PMIC_IRQ_MAX + 1U)
/** @} */

/**
 * @brief Defines used internally by PMIC LLD for the IRQ module.
 *
 * @{
 */
#define PMIC_NUM_ELEM_IN_INTR_STAT          ((uint8_t)2U)
#define PMIC_NUM_BITS_IN_INTR_STAT          ((uint8_t)32U)
/** @} */

/* ========================================================================== */
/*                             Structures and Enums                           */
/* ========================================================================== */

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
typedef struct Pmic_IrqStat_s
{
    uint32_t intrStat[PMIC_NUM_ELEM_IN_INTR_STAT];
} Pmic_IrqStat_t;

/* ========================================================================== */
/*                             Function Declarations                          */
/* ========================================================================== */

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
int32_t Pmic_irqGetStat(const Pmic_CoreHandle_t *pmicHandle, Pmic_IrqStat_t *irqStat);

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
 * @param pmicHandle [IN] PMIC interface handle.
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
int32_t Pmic_irqGetFlag(const Pmic_CoreHandle_t *pmicHandle, uint8_t irqNum, bool *flag);

/**
 * @brief Clear a specific PMIC IRQ flag.
 *
 * @attention This API is meant to be called after getting the next flag status from
 * `Pmic_irqGetNextFlag()` or getting a specific flag status from `Pmic_irqGetFlag()`.
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @param irqNum [IN] Target PMIC IRQ to clear. For valid values, refer to
 * @ref Pmic_IRQs.
 *
 * @return Success code if the PMIC IRQ flag has been cleared, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_irqClrFlag(const Pmic_CoreHandle_t *pmicHandle, uint8_t irqNum);

/**
 * @brief Clear all PMIC IRQ flags.
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @return Success code if all PMIC IRQ flags have been cleared, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_irqClrAllFlags(const Pmic_CoreHandle_t *handle);

#ifdef __cplusplus
}

#endif /* __cplusplus */
#endif /* __PMIC_IRQ_H__ */
