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
 * @file   pmic_irq.h
 *
 * @brief  PMIC IRQ Driver API/interface file.
 */

#ifndef __PMIC_IRQ_H__
#define __PMIC_IRQ_H__


/* ==========================================================================*/
/*                             Include Files                                 */
/* ==========================================================================*/
#include <stdbool.h>
#include <stdint.h>

#include "pmic_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup Pmic_IRQ PMIC Interrupt Request
 * @{
 * @brief Contains definitions related to PMIC IRQ functionality.
 */

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/**
 * @anchor Pmic_IRQs
 * @name PMIC Interrupt Requests
 *
 * @brief Maskable and nonmaskable PMIC interrupt requests.
 *
 * @details IRQs below are labeled as STD_IRQ, CFG_NMI, or NCFG_NMI.
 * 1. STD_IRQ indicates the IRQ is standard (configurable, maskable).
 * 2. CFG_NMI indicates the IRQ is a configurable non-maskable interrupt.
 * 3. NCFG_NMI indicates the IRQ is a non-configurable non-maskable interrupt.
 *
 * @{
 */
#define PMIC_CFG_REG_CRC_ERR_INT                 (0U)  // NCFG_NMI
#define PMIC_NRST_RDBK_ERR_INT                   (1U)  // STD_IRQ
#define PMIC_SAFE_OUT1_RDBK_ERR_INT              (2U)  // STD_IRQ
#define PMIC_EN_OUT_RDBK_ERR_INT                 (3U)  // STD_IRQ
#define PMIC_GPO1_RDBK_ERR_INT                   (4U)  // STD_IRQ
#define PMIC_GPO2_RDBK_ERR_INT                   (5U)  // STD_IRQ
#define PMIC_GPO3_RDBK_ERR_INT                   (6U)  // STD_IRQ
#define PMIC_GPO4_RDBK_ERR_INT                   (7U)  // STD_IRQ
#define PMIC_NORMAL_OFF_INT                      (8U)  // NCFG_NMI
#define PMIC_OFF_INT_EVT_ERR_INT                 (9U)  // NCFG_NMI
#define PMIC_OFF_PROT_EVT_INT                    (10U) // NCFG_NMI
#define PMIC_FIRST_PWR_ON_INT                    (11U) // NCFG_NMI
#define PMIC_CLK_ERR_INT                         (12U) // NCFG_NMI
#define PMIC_INTERNAL_OV_INT                     (13U) // NCFG_NMI
#define PMIC_INIT_AN_TMO_INT                     (14U) // NCFG_NMI
#define PMIC_CRC_ERR_INT                         (15U) // NCFG_NMI
#define PMIC_SYS_CLK_ERR_PROT_INT                (16U) // NCFG_NMI
#define PMIC_RST_MCU_TMO_INT                     (17U) // NCFG_NMI
#define PMIC_BGXM_ERR_INT                        (18U) // NCFG_NMI
#define PMIC_VBAT_OVP_ERR_INT                    (19U) // NCFG_NMI
#define PMIC_BB_OVP_ERR_INT                      (20U) // NCFG_NMI
#define PMIC_BB_BST_TMO_INT                      (21U) // NCFG_NMI
#define PMIC_BB_PK_ILIM_ERR_INT                  (22U) // NCFG_NMI
#define PMIC_WD_TMO_INT                          (23U) // NCFG_NMI
#define PMIC_WD_TRIG_EARLY_INT                   (24U) // NCFG_NMI
#define PMIC_WD_ANSW_EARLY_INT                   (25U) // NCFG_NMI
#define PMIC_WD_SEQ_ERR_INT                      (26U) // NCFG_NMI
#define PMIC_WD_ANSW_ERR_INT                     (27U) // NCFG_NMI
#define PMIC_WD_LONGWIN_TMO_INT                  (28U) // NCFG_NMI
#define PMIC_WD_TH1_ERR_INT                      (29U) // STD_IRQ
#define PMIC_WD_TH2_ERR_INT                      (30U) // STD_IRQ
#define PMIC_ESM_ERR_INT                         (31U) // NCFG_NMI
#define PMIC_ESM_DLY1_ERR_INT                    (32U) // STD_IRQ
#define PMIC_ESM_DLY2_ERR_INT                    (33U) // STD_IRQ
#define PMIC_BB_UV_ERR_INT                       (34U) // STD_IRQ
#define PMIC_BB_OV_ERR_INT                       (35U) // STD_IRQ
#define PMIC_LDO1_UV_ERR_INT                     (36U) // STD_IRQ
#define PMIC_LDO1_OV_ERR_INT                     (37U) // STD_IRQ
#define PMIC_LDO2_UV_ERR_INT                     (38U) // STD_IRQ
#define PMIC_LDO2_OV_ERR_INT                     (39U) // STD_IRQ
#define PMIC_LDO3_UV_ERR_INT                     (40U) // STD_IRQ
#define PMIC_LDO3_OV_ERR_INT                     (41U) // STD_IRQ
#define PMIC_LDO4_UV_ERR_INT                     (42U) // STD_IRQ
#define PMIC_LDO4_OV_ERR_INT                     (43U) // STD_IRQ
#define PMIC_PLDO1_UV_ERR_INT                    (44U) // STD_IRQ
#define PMIC_PLDO1_OV_ERR_INT                    (45U) // STD_IRQ
#define PMIC_PLDO2_UV_ERR_INT                    (46U) // STD_IRQ
#define PMIC_PLDO2_OV_ERR_INT                    (47U) // STD_IRQ
#define PMIC_EXT_VMON1_UV_ERR_INT                (48U) // STD_IRQ
#define PMIC_EXT_VMON1_OV_ERR_INT                (49U) // STD_IRQ
#define PMIC_EXT_VMON2_UV_ERR_INT                (50U) // STD_IRQ
#define PMIC_EXT_VMON2_OV_ERR_INT                (51U) // STD_IRQ
#define PMIC_LDO1_ILIM_ERR_INT                   (52U) // CFG_NMI
#define PMIC_LDO2_ILIM_ERR_INT                   (53U) // CFG_NMI
#define PMIC_LDO3_ILIM_ERR_INT                   (54U) // CFG_NMI
#define PMIC_LDO4_ILIM_ERR_INT                   (55U) // CFG_NMI
#define PMIC_PLDO1_ILIM_ERR_INT                  (56U) // CFG_NMI
#define PMIC_PLDO2_ILIM_ERR_INT                  (57U) // CFG_NMI
#define PMIC_BB_AVG_ILIM_ERR_INT                 (58U) // NCFG_NMI
#define PMIC_ABIST_ERR_INT                       (59U) // NCFG_NMI
#define PMIC_LBIST_ERR_INT                       (60U) // NCFG_NMI
#define PMIC_TMR_REG_ERR_INT                     (61U) // NCFG_NMI
#define PMIC_M_PMIC_HB_ERR_INT                   (62U) // NCFG_NMI
#define PMIC_SAFE_ST_TMO_RST_ERR_INT             (63U) // NCFG_NMI
#define PMIC_SPI_CRC_ERR_INT                     (64U) // NCFG_NMI
#define PMIC_FRM_ERR_INT                         (65U) // NCFG_NMI
#define PMIC_ADDR_ERR_INT                        (66U) // NCFG_NMI
#define PMIC_SCLK_ERR_INT                        (67U) // NCFG_NMI
#define PMIC_COMP1_ERR_INT                       (68U) // NCFG_NMI
#define PMIC_COMP1P_UV_ERR_INT                   (69U) // STD_IRQ
#define PMIC_COMP1P_OV_ERR_INT                   (70U) // STD_IRQ
#define PMIC_COMP1N_UV_ERR_INT                   (71U) // STD_IRQ
#define PMIC_COMP1N_OV_ERR_INT                   (72U) // STD_IRQ
#define PMIC_COMP2_ERR_INT                       (73U) // NCFG_NMI
#define PMIC_COMP2P_UV_ERR_INT                   (74U) // STD_IRQ
#define PMIC_COMP2P_OV_ERR_INT                   (75U) // STD_IRQ
#define PMIC_COMP2N_UV_ERR_INT                   (76U) // STD_IRQ
#define PMIC_COMP2N_OV_ERR_INT                   (77U) // STD_IRQ
#define PMIC_IRQ_MAX                             (PMIC_COMP2N_OV_ERR_INT)
#define PMIC_IRQ_NUM                             (PMIC_IRQ_MAX + 1U)
#define PMIC_IRQ_ALL                             ((uint8_t)0xFFU)
/** @} */

// Defines used internally only
#define PMIC_NUM_ELEM_IN_INTR_STAT               (4U)
#define PMIC_NUM_BITS_IN_INTR_ELEM               ((uint8_t)(8U * sizeof(uint32_t)))

/**
 * @defgroup Pmic_IRQMacros PMIC Interrupt Request Macros
 * @{
 * @ingroup Pmic_IRQ
 * @brief Contains macros used in the IRQ module of PMIC driver.
 */

/**
 * @anchor Pmic_IrqCfgValidParams
 * @name PMIC Interrupt Request Configuration Valid Parameters
 *
 * @brief Valid parameters of the Pmic_IrqCfg_t struct.
 *
 * @{
 */
#define PMIC_IRQ_CFG_MASK_VALID         (0U)
#define PMIC_IRQ_CFG_CONFIG_VALID       (1U)
/** @} */

/**
 * @anchor Pmic_IrqCfgValidParamShifts
 * @name PMIC Interrupt Request Configuration Valid Parameter Shifts
 *
 * @brief Valid parameter shifts of the Pmic_IrqCfg_t struct.
 *
 * @{
 */
#define PMIC_IRQ_CFG_MASK_VALID_SHIFT   (1U << PMIC_IRQ_CFG_MASK_VALID)
#define PMIC_IRQ_CFG_CONFIG_VALID_SHIFT (1U << PMIC_IRQ_CFG_CONFIG_VALID)
#define PMIC_IRQ_CFG_ALL_VALID_SHIFT (\
    PMIC_IRQ_CFG_MASK_VALID_SHIFT |\
    PMIC_IRQ_CFG_CONFIG_VALID_SHIFT)
/** @} */

/** * @} */
/* End of Pmic_IRQMacros */

/*==========================================================================*/
/*                         Structures and Enums                             */
/*==========================================================================*/

/**
 * @defgroup Pmic_IRQStructures PMIC IRQ Structures
 * @{
 * @ingroup Pmic_IRQ
 * @brief Contains structures used in the IRQ module of PMIC driver.
 */

/**
 * @brief Structure used to set/get PMIC IRQ configurations.
 * @ingroup Pmic_IRQStructures
 *
 * @note The PMIC has three categories for interrupt requests: configurable
 * maskable interrupts, configurable non-maskable interrupts, and
 * non-configurable non-maskable interrupts. The `mask` parameter is invalid
 * for non-maskable interrupts. The `config` parameter is invalid for
 * non-configurable interrupts.
 *
 * @attention The `irqNum` field is always a required parameter to be specified.
 * Consequently, there is no corresponding validParam value for it.
 *
 * @param validParams For valid values, see @ref Pmic_IrqCfgValidParamShifts.
 *
 * @param irqNum Interrupt request number. For valid values, see
 * @ref Pmic_IRQs.
 *
 * @param mask When set to true, the IRQ is masked. When set to false, IRQ is
 * unmasked.
 *
 * @param config Interrupt behavior configuration.
 */
typedef struct Pmic_IrqCfg_s {
    uint8_t validParams;
    uint8_t irqNum;

    bool mask;
    uint8_t config;
} Pmic_IrqCfg_t;

/**
 * @brief Structure for storing PMIC interrupt status.
 * @ingroup Pmic_IRQStructures
 *
 * @param intStatus Array to store interrupt status.
 */
typedef struct Pmic_IrqStat_s {
    uint32_t intStat[PMIC_NUM_ELEM_IN_INTR_STAT];
} Pmic_IrqStat_t;

/** * @} */
/* End of Pmic_IRQStructures */

/*==========================================================================*/
/*                         Function Declarations                            */
/*==========================================================================*/

/**
 * @defgroup Pmic_IRQFunctions PMIC Interrupt Request Functions
 * @{
 * @ingroup Pmic_IRQ
 * @brief Contains functions used in the IRQ module of PMIC driver.
 */

/**
 * @brief Set the configuration for a single PMIC IRQ.
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @return Success code if IRQ mask configuration(s) have been set, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_irqSetCfg(Pmic_CoreHandle_t *handle, const Pmic_IrqCfg_t *irqCfg);

/**
 * @brief Set the configuration for multiple PMIC IRQs.
 *
 * @param handle  [IN] PMIC interface handle.
 * @param numIrqs [IN] Number of IRQ configurations to set.
 * @param irqCfg  [IN] Array of IRQ configurations.
 *
 * @return Success code if IRQ configuration(s) have been set, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_irqSetCfgs(Pmic_CoreHandle_t *handle, uint8_t numIrqs, const Pmic_IrqCfg_t *irqCfg);

/**
 * @brief Get the mask configuration for a PMIC IRQ.
 *
 * @param handle      [IN]     PMIC interface handle.
 * @param irqMask     [IN/OUT] Array of IRQ mask configurations.
 *
 * @return Success code if IRQ mask configuration(s) have been obtained, error
 * code otherwise. For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_irqGetCfg(Pmic_CoreHandle_t *handle, Pmic_IrqCfg_t *irqCfg);

/**
 * @brief Get the mask configuration for PMIC IRQs.
 *
 * @param handle      [IN]     PMIC interface handle.
 * @param numIrqMasks [IN]     Number of IRQ mask configurations to obtain.
 * @param irqMasks    [IN/OUT] Array of IRQ mask configurations.
 *
 * @return Success code if IRQ mask configuration(s) have been obtained, error
 * code otherwise. For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_irqGetCfgs(Pmic_CoreHandle_t *handle, uint8_t numIrqs, Pmic_IrqCfg_t *irqCfgs);

/**
 * @brief Get the status of all PMIC IRQs.
 *
 * @attention End-user must call this API first before calling `Pmic_irqGetNextFlag()`.
 *
 * @param handle  [IN] PMIC interface handle.
 * @param irqStat [OUT] Status of all PMIC IRQs.
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

/** * @} */
/* End of Pmic_IRQFunctions */
/** * @} */
/* End of Pmic_IRQ */

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* __PMIC_IRQ_H__ */
