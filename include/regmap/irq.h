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
 * @file irq.h
 *
 * @brief PMIC LLD register addresses and bit fields pertaining to the IRQ
 * module.
 */
#ifndef __IRQ_H__
#define __IRQ_H__

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                Include Files                               */
/* ========================================================================== */
#include "pmic_common.h"

/* ========================================================================== */
/*                      TPS65036x IRQ Module Register Map                     */
/* ========================================================================== */

#define PMIC_MASK_BUCK1_2_REGADDR           ((uint8_t)0x36U)
#define PMIC_MASK_BUCK3_LDO_REGADDR         ((uint8_t)0x37U)
#define PMIC_MASK_MISC_REGADDR              ((uint8_t)0x38U)
#define PMIC_MASK_MODERATE_ERR_REGADDR      ((uint8_t)0x39U)
#define PMIC_MASK_COMM_ERR_REGADDR          ((uint8_t)0x3AU)
#define PMIC_MASK_ESM_REGADDR               ((uint8_t)0x3BU)
#define PMIC_INT_TOP_REGADDR                ((uint8_t)0x4CU)
#define PMIC_INT_BUCK_LDO_REGADDR           ((uint8_t)0x4DU)
#define PMIC_INT_BUCK1_2_REGADDR            ((uint8_t)0x4EU)
#define PMIC_INT_BUCK3_LDO_REGADDR          ((uint8_t)0x4FU)
#define PMIC_INT_MISC_REGADDR               ((uint8_t)0x50U)
#define PMIC_INT_MODERATE_ERR_REGADDR       ((uint8_t)0x51U)
#define PMIC_INT_SEVERE_ERR_REGADDR         ((uint8_t)0x52U)
#define PMIC_INT_FSM_ERR_REGADDR            ((uint8_t)0x53U)
#define PMIC_INT_COMM_ERR_REGADDR           ((uint8_t)0x54U)
#define PMIC_INT_ESM_REGADDR                ((uint8_t)0x55U)

/* ========================================================================== */
/*                               Macros & Typedefs                            */
/* ========================================================================== */

/**
 * @anchor Pmic_intTopBitPos
 * @name TPS65036x INT_TOP Register Bit Fields
 *
 * @brief Mask and shift values for all bit fields of the INT_TOP register.
 *
 * @{
 */
#define PMIC_FSM_ERR_INT_SHIFT              ((uint8_t)7U)
#define PMIC_FSM_ERR_INT_MASK               ((uint8_t)1U << PMIC_FSM_ERR_INT_SHIFT)
#define PMIC_SEVERE_ERR_INT_SHIFT           ((uint8_t)6U)
#define PMIC_SEVERE_ERR_INT_MASK            ((uint8_t)1U << PMIC_SEVERE_ERR_INT_SHIFT)
#define PMIC_MODERATE_ERR_INT_SHIFT         ((uint8_t)5U)
#define PMIC_MODERATE_ERR_INT_MASK          ((uint8_t)1U << PMIC_MODERATE_ERR_INT_SHIFT)
#define PMIC_MISC_INT_SHIFT                 ((uint8_t)4U)
#define PMIC_MISC_INT_MASK                  ((uint8_t)1U << PMIC_MISC_INT_SHIFT)
#define PMIC_BUCK_LDO_INT_SHIFT             ((uint8_t)0U)
#define PMIC_BUCK_LDO_INT_MASK              ((uint8_t)1U << PMIC_BUCK_LDO_INT_SHIFT)
/** @} */

/**
 * @anchor Pmic_maskBuck1_2BitPos
 * @name TPS65036x MASK_BUCK1_2 Register Bit Positions
 *
 * @brief Register bit positions/shifts of the MASK_BUCK1_2 register.
 *
 * @{
 */
#define PMIC_BUCK2_OVP_MASK_SHIFT           ((uint8_t)6U)
#define PMIC_BUCK2_UV_MASK_SHIFT            ((uint8_t)5U)
#define PMIC_BUCK2_OV_MASK_SHIFT            ((uint8_t)4U)
#define PMIC_BUCK1_OVP_MASK_SHIFT           ((uint8_t)2U)
#define PMIC_BUCK1_UV_MASK_SHIFT            ((uint8_t)1U)
#define PMIC_BUCK1_OV_MASK_SHIFT            ((uint8_t)0U)
/** @} */

/**
 * @anchor Pmic_maskBuck3LdoBitPos
 * @name TPS65036x MASK_BUCK3_LDO Register Bit Positions
 *
 * @brief Register bit positions/shifts of the MASK_BUCK3_LDO register.
 *
 * @{
 */
#define PMIC_LDO_OVP_MASK_SHIFT             ((uint8_t)6U)
#define PMIC_LDO_UV_MASK_SHIFT              ((uint8_t)5U)
#define PMIC_LDO_OV_MASK_SHIFT              ((uint8_t)4U)
#define PMIC_BUCK3_OVP_MASK_SHIFT           ((uint8_t)2U)
#define PMIC_BUCK3_UV_MASK_SHIFT            ((uint8_t)1U)
#define PMIC_BUCK3_OV_MASK_SHIFT            ((uint8_t)0U)
/** @} */

/**
 * @anchor Pmic_maskMiscBitPos
 * @name TPS65036x MASK_MISC Register Bit Positions
 *
 * @brief Register bit positions/shifts of the MASK_MISC register.
 *
 * @{
 */
#define PMIC_TWARN_MASK_SHIFT               ((uint8_t)7U)
#define PMIC_B1_PVIN_UVLO_MASK_SHIFT        ((uint8_t)6U)
#define PMIC_BUCKS_VSET_ERR_MASK_SHIFT      ((uint8_t)5U)
#define PMIC_ABIST_FAIL_MASK_SHIFT          ((uint8_t)1U)
#define PMIC_ABIST_DONE_MASK_SHIFT          ((uint8_t)0U)
/** @} */

/**
 * @anchor Pmic_maskModerateErrBitPos
 * @name TPS65036x MASK_MODERATE_ERR Register Bit Positions
 *
 * @brief Register bit positions/shifts of the MASK_MODERATE_ERR register.
 *
 * @{
 */
#define PMIC_GPO_READBACK_MASK_SHIFT        ((uint8_t)6U)
#define PMIC_NINT_READBACK_MASK_SHIFT       ((uint8_t)4U)
#define PMIC_CONFIG_CRC_MASK_SHIFT          ((uint8_t)3U)
#define PMIC_TRIM_TEST_CRC_MASK_SHIFT       ((uint8_t)2U)
/** @} */

/**
 * @anchor Pmic_maskCommErrBitPos
 * @name TPS65036x MASK_COMM_ERR Register Bit Positions
 *
 * @brief Register bit positions/shifts of the MASK_COMM_ERR register.
 *
 * @{
 */
#define PMIC_MCU_COMM_ERR_MASK_SHIFT        ((uint8_t)4U)
#define PMIC_COMM_ADR_ERR_MASK_SHIFT        ((uint8_t)3U)
#define PMIC_COMM_CRC_ERR_MASK_SHIFT        ((uint8_t)1U)
/** @} */

/**
 * @anchor Pmic_maskEsmBitPos
 * @name TPS65036x MASK_ESM Register Bit Positions
 *
 * @brief Register bit positions/shifts of the MASK_ESM register.
 *
 * @{
 */
#define PMIC_ESM_MCU_RST_MASK_SHIFT         ((uint8_t)5U)
#define PMIC_ESM_MCU_FAIL_MASK_SHIFT        ((uint8_t)4U)
#define PMIC_ESM_MCU_PIN_MASK_SHIFT         ((uint8_t)3U)
/** @} */

/**
 * @anchor Pmic_intBuckLdoBitPos
 * @name TPS65036x INT_BUCK_LDO Register Bit Positions
 *
 * @brief Register bit positions/shifts of the INT_BUCK_LDO register.
 *
 * @{
 */
#define PMIC_LDO_SC_INT_SHIFT               ((uint8_t)7U)
#define PMIC_BUCK3_SC_INT_SHIFT             ((uint8_t)6U)
#define PMIC_BUCK2_SC_INT_SHIFT             ((uint8_t)5U)
#define PMIC_BUCK1_SC_INT_SHIFT             ((uint8_t)4U)
#define PMIC_LDO_INT_SHIFT                  ((uint8_t)3U)
#define PMIC_BUCK3_INT_SHIFT                ((uint8_t)2U)
#define PMIC_BUCK2_INT_SHIFT                ((uint8_t)1U)
#define PMIC_BUCK1_INT_SHIFT                ((uint8_t)0U)
/** @} */

/**
 * @anchor Pmic_intBuck1_2BitPos
 * @name TPS65036x INT_BUCK1_2 Register Bit Positions
 *
 * @brief Register bit positions/shifts of the INT_BUCK1_2 register.
 *
 * @{
 */
#define PMIC_BUCK2_OVP_INT_SHIFT            ((uint8_t)6U)
#define PMIC_BUCK2_UV_INT_SHIFT             ((uint8_t)5U)
#define PMIC_BUCK2_OV_INT_SHIFT             ((uint8_t)4U)
#define PMIC_BUCK1_OVP_INT_SHIFT            ((uint8_t)2U)
#define PMIC_BUCK1_UV_INT_SHIFT             ((uint8_t)1U)
#define PMIC_BUCK1_OV_INT_SHIFT             ((uint8_t)0U)
/** @} */

/**
 * @anchor Pmic_intBuck3LdoBitPos
 * @name TPS65036x INT_BUCK3_LDO Register Bit Positions
 *
 * @brief Register bit positions/shifts of the INT_BUCK3_LDO register.
 *
 * @{
 */
#define PMIC_LDO_OVP_INT_SHIFT              ((uint8_t)6U)
#define PMIC_LDO_UV_INT_SHIFT               ((uint8_t)5U)
#define PMIC_LDO_OV_INT_SHIFT               ((uint8_t)4U)
#define PMIC_BUCK3_OVP_INT_SHIFT            ((uint8_t)2U)
#define PMIC_BUCK3_UV_INT_SHIFT             ((uint8_t)1U)
#define PMIC_BUCK3_OV_INT_SHIFT             ((uint8_t)0U)
/** @} */

/**
 * @anchor Pmic_intMiscBitPos
 * @name TPS65036x INT_MISC Register Bit Positions
 *
 * @brief Register bit positions/shifts of the INT_MISC register.
 *
 * @{
 */
#define PMIC_TWARN_INT_SHIFT                ((uint8_t)7U)
#define PMIC_B1_PVIN_UVLO_INT_SHIFT         ((uint8_t)6U)
#define PMIC_BUCKS_VSET_ERR_INT_SHIFT       ((uint8_t)5U)
#define PMIC_CFG_NVM_VERIFY_ERR_SHIFT       ((uint8_t)4U)
#define PMIC_CFG_NVM_VERIFY_DONE_SHIFT      ((uint8_t)3U)
#define PMIC_CFG_NVM_PRG_DONE_SHIFT         ((uint8_t)2U)
#define PMIC_ABIST_FAIL_INT_SHIFT           ((uint8_t)1U)
#define PMIC_ABIST_DONE_INT_SHIFT           ((uint8_t)0U)
/** @} */

/**
 * @anchor Pmic_intModerateErrBitPos
 * @name TPS65036x INT_MODERATE_ERR Register Bit Positions
 *
 * @brief Register bit positions/shifts of the INT_MODERATE_ERR register.
 *
 * @{
 */
#define PMIC_GPO_READBACK_INT_SHIFT         ((uint8_t)6U)
#define PMIC_NINT_READBACK_INT_SHIFT        ((uint8_t)4U)
#define PMIC_CONFIG_CRC_INT_SHIFT           ((uint8_t)3U)
#define PMIC_TRIM_TEST_CRC_INT_SHIFT        ((uint8_t)2U)
#define PMIC_RECOV_CNT_INT_SHIFT            ((uint8_t)1U)
/** @} */

/**
 * @anchor Pmic_intSevereErrBitPos
 * @name TPS65036x INT_SEVERE_ERR Register Bit Positions
 *
 * @brief Register bit positions/shifts of the INT_SEVERE_ERR register.
 *
 * @{
 */
#define PMIC_TSD_IMM_INT_SHIFT              ((uint8_t)0U)
/** @} */

/**
 * @anchor Pmic_intFsmErrBitPos
 * @name TPS65036x INT_FSM_ERR Register Bit Positions
 *
 * @brief Register bit positions/shifts of the INT_FSM_ERR register.
 *
 * @{
 */
#define PMIC_WD_INT_SHIFT                   ((uint8_t)7U)
#define PMIC_COMM_ERR_INT_SHIFT             ((uint8_t)6U)
#define PMIC_ESM_MCU_INT_SHIFT              ((uint8_t)5U)
#define PMIC_WD_FIRST_NOK_INT_SHIFT         ((uint8_t)4U)
#define PMIC_WAIT_FOR_PWRCYCLE_INT_SHIFT    ((uint8_t)3U)
#define PMIC_WARM_RESET_INT_SHIFT           ((uint8_t)2U)
#define PMIC_ORD_SHUTDOWN_INT_SHIFT         ((uint8_t)1U)
#define PMIC_IMM_SHUTDOWN_INT_SHIFT         ((uint8_t)0U)
/** @} */

/**
 * @anchor Pmic_intCommErrBitPos
 * @name TPS65036x INT_COMM_ERR Register Bit Positions
 *
 * @brief Register bit positions/shifts of the INT_COMM_ERR register.
 *
 * @{
 */
#define PMIC_MCU_COMM_ERR_INT_SHIFT         ((uint8_t)4U)
#define PMIC_COMM_ADR_ERR_INT_SHIFT         ((uint8_t)3U)
#define PMIC_COMM_CRC_ERR_INT_SHIFT         ((uint8_t)1U)
/** @} */

/**
 * @anchor Pmic_intEsmBitPos
 * @name TPS65036x INT_ESM Register Bit Positions
 *
 * @brief Register bit positions/shifts of the INT_ESM register.
 *
 * @{
 */
#define PMIC_ESM_MCU_RST_INT_SHIFT          ((uint8_t)5U)
#define PMIC_ESM_MCU_FAIL_INT_SHIFT         ((uint8_t)4U)
#define PMIC_ESM_MCU_PIN_INT_SHIFT          ((uint8_t)3U)
/** @} */

#ifdef __cplusplus
}

#endif /* __cplusplus */
#endif /* __IRQ_H__ */
