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
 * @file esm.h
 *
 * @brief PMIC LLD register addresses and bit fields pertaining to the ESM
 * module.
 */
#ifndef __ESM_H__
#define __ESM_H__

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                Include Files                               */
/* ========================================================================== */
#include "pmic_common.h"

/* ========================================================================== */
/*                      TPS65036x ESM Module Register Map                     */
/* ========================================================================== */

#define PMIC_ESM_START_REG_REGADDR          ((uint8_t)0x08U)
#define PMIC_ESM_DELAY1_REG_REGADDR         ((uint8_t)0x3EU)
#define PMIC_ESM_DELAY2_REG_REGADDR         ((uint8_t)0x3FU)
#define PMIC_ESM_MODE_CFG_REGADDR           ((uint8_t)0x40U)
#define PMIC_ESM_HMAX_REG_REGADDR           ((uint8_t)0x41U)
#define PMIC_ESM_HMIN_REG_REGADDR           ((uint8_t)0x42U)
#define PMIC_ESM_LMAX_REG_REGADDR           ((uint8_t)0x43U)
#define PMIC_ESM_LMIN_REG_REGADDR           ((uint8_t)0x44U)
#define PMIC_ESM_ERR_CNT_REG_REGADDR        ((uint8_t)0x5DU)

/* ========================================================================== */
/*                               Macros & Typedefs                            */
/* ========================================================================== */

/**
 * @anchor Pmic_esmStartRegBitFields
 * @name TPS65036x ESM_START_REG Register Bit Fields
 *
 * @brief Mask and shift values for all bit fields of the ESM_START_REG register.
 *
 * @{
 */
#define PMIC_ESM_MCU_START_SHIFT            ((uint8_t)0U)
#define PMIC_ESM_MCU_START_MASK             ((uint8_t)1U << PMIC_ESM_MCU_START_SHIFT)
/** @} */

/**
 * @anchor Pmic_esmDelay1RegBitFields
 * @name TPS65036x ESM_DELAY1_REG Register Bit Fields
 *
 * @brief Mask and shift values for all bit fields of the ESM_DELAY1_REG register.
 *
 * @{
 */
#define PMIC_ESM_MCU_DELAY1_SHIFT            ((uint8_t)0U)
#define PMIC_ESM_MCU_DELAY1_MASK             ((uint8_t)0xFFU << PMIC_ESM_MCU_DELAY1_SHIFT)
/** @} */

/**
 * @anchor Pmic_esmDelay2RegBitFields
 * @name TPS65036x ESM_DELAY2_REG Register Bit Fields
 *
 * @brief Mask and shift values for all bit fields of the ESM_DELAY2_REG register.
 *
 * @{
 */
#define PMIC_ESM_MCU_DELAY2_SHIFT            ((uint8_t)0U)
#define PMIC_ESM_MCU_DELAY2_MASK             ((uint8_t)0xFFU << PMIC_ESM_MCU_DELAY2_SHIFT)
/** @} */

/**
 * @anchor Pmic_esmModeCfgBitFields
 * @name TPS65036x ESM_MODE_CFG Register Bit Fields
 *
 * @brief Mask and shift values for all bit fields of the ESM_MODE_CFG register.
 *
 * @{
 */
#define PMIC_ESM_MCU_MODE_SHIFT             ((uint8_t)7U)
#define PMIC_ESM_MCU_MODE_MASK              ((uint8_t)1U << PMIC_ESM_MCU_MODE_SHIFT)
#define PMIC_ESM_MCU_EN_SHIFT               ((uint8_t)6U)
#define PMIC_ESM_MCU_EN_MASK                ((uint8_t)1U << PMIC_ESM_MCU_EN_SHIFT)
#define PMIC_ESM_MCU_ERR_CNT_TH_SHIFT       ((uint8_t)0U)
#define PMIC_ESM_MCU_ERR_CNT_TH_MASK        ((uint8_t)0x0FU << PMIC_ESM_MCU_ERR_CNT_TH_SHIFT)
/** @} */

/**
 * @anchor Pmic_esmHmaxRegBitFields
 * @name TPS65036x ESM_HMAX_REG Register Bit Fields
 *
 * @brief Mask and shift values for all bit fields of the ESM_HMAX_REG register.
 *
 * @{
 */
#define PMIC_ESM_MCU_HMAX_SHIFT            ((uint8_t)0U)
#define PMIC_ESM_MCU_HMAX_MASK             ((uint8_t)0xFFU << PMIC_ESM_MCU_HMAX_SHIFT)
/** @} */

/**
 * @anchor Pmic_esmHminRegBitFields
 * @name TPS65036x ESM_HMIN_REG Register Bit Fields
 *
 * @brief Mask and shift values for all bit fields of the ESM_HMIN_REG register.
 *
 * @{
 */
#define PMIC_ESM_MCU_HMIN_SHIFT            ((uint8_t)0U)
#define PMIC_ESM_MCU_HMIN_MASK             ((uint8_t)0xFFU << PMIC_ESM_MCU_HMIN_SHIFT)
/** @} */

/**
 * @anchor Pmic_esmLmaxRegBitFields
 * @name TPS65036x ESM_LMAX_REG Register Bit Fields
 *
 * @brief Mask and shift values for all bit fields of the ESM_LMAX_REG register.
 *
 * @{
 */
#define PMIC_ESM_MCU_LMAX_SHIFT            ((uint8_t)0U)
#define PMIC_ESM_MCU_LMAX_MASK             ((uint8_t)0xFFU << PMIC_ESM_MCU_LMAX_SHIFT)
/** @} */

/**
 * @anchor Pmic_esmLminRegBitFields
 * @name TPS65036x ESM_LMIN_REG Register Bit Fields
 *
 * @brief Mask and shift values for all bit fields of the ESM_LMIN_REG register.
 *
 * @{
 */
#define PMIC_ESM_MCU_LMIN_SHIFT            ((uint8_t)0U)
#define PMIC_ESM_MCU_LMIN_MASK             ((uint8_t)0xFFU << PMIC_ESM_MCU_LMIN_SHIFT)
/** @} */

/**
 * @anchor Pmic_esmErrCntRegBitFields
 * @name TPS65036x ESM_ERR_CNT_REG Register Bit Fields
 *
 * @brief Mask and shift values for all bit fields of the ESM_ERR_CNT_REG register.
 *
 * @{
 */
#define PMIC_ESM_MCU_ERR_CNT_SHIFT          ((uint8_t)0U)
#define PMIC_ESM_MCU_ERR_CNT_MASK           ((uint8_t)0x1FU << PMIC_ESM_MCU_ERR_CNT_SHIFT)
/** @} */

/**
 * @anchor Pmic_intEsmBitFields
 * @name TPS65036x INT_ESM Register Bit Fields
 *
 * @brief Mask and shift values for all bit fields of the INT_ESM register.
 *
 * @{
 */
#define PMIC_ESM_MCU_RST_INT_SHIFT          ((uint8_t)5U)
#define PMIC_ESM_MCU_RST_INT_MASK           ((uint8_t)1U << PMIC_ESM_MCU_RST_INT_SHIFT)
#define PMIC_ESM_MCU_FAIL_INT_SHIFT         ((uint8_t)4U)
#define PMIC_ESM_MCU_FAIL_INT_MASK          ((uint8_t)1U << PMIC_ESM_MCU_FAIL_INT_SHIFT)
#define PMIC_ESM_MCU_PIN_INT_SHIFT          ((uint8_t)3U)
#define PMIC_ESM_MCU_PIN_INT_MASK           ((uint8_t)1U << PMIC_ESM_MCU_PIN_INT_SHIFT)
/** @} */

#ifdef __cplusplus
}

#endif /* __cplusplus */
#endif /* __ESM_H__ */
