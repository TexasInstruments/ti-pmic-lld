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
#ifndef __PMIC_REGMAP_ESM_H__
#define __PMIC_REGMAP_ESM_H__

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup Pmic_ESMRegDefs PMIC Error State Machine Register Address Definitions
 * @{
 * @brief Register address definitions related to ESM control and configuration.
 */
#define PMIC_ESM_CTRL_REG     (0x47U)
#define PMIC_ESM_CFG1_REG     (0x48U)
#define PMIC_ESM_CFG2_REG     (0x49U)
#define PMIC_ESM_INT_CFG_REG  (0x4AU)
#define PMIC_ESM_DELAY1_REG   (0x4BU)
#define PMIC_ESM_DELAY2_REG   (0x4CU)
#define PMIC_ESM_HMAX_CFG_REG (0x4DU)
#define PMIC_ESM_HMIN_CFG_REG (0x4EU)
#define PMIC_ESM_LMAX_CFG_REG (0x4FU)
#define PMIC_ESM_LMIN_CFG_REG (0x50U)
#define PMIC_ESM_ERR_STAT_REG (0x51U)
/**
 * @}
 */

/**
 * @defgroup Pmic_ESMFieldDefs PMIC Error State Machine Bitfield Position Definitions
 * @{
 * @brief Bitfield position definitions related to ESM control and configuration.
 */

/** @brief Bit shift position for enabling ESM in the control register. */
#define PMIC_ESM_CTRL_REG_SHIFT     (0x00U)

/** @brief Bit mask for enabling ESM in the control register. */
#define PMIC_ESM_CTRL_REG_MASK      (0x01U << PMIC_ESM_CTRL_REG_SHIFT)

/**
 * @brief Bit shift position and mask for error threshold configuration in ESM
 * configuration register 1.
 */
#define PMIC_ESM_CFG1_ERR_TH_SHIFT  (0x00U)
#define PMIC_ESM_CFG1_ERR_TH_MASK   (0x07U << PMIC_ESM_CFG1_ERR_TH_SHIFT)


/**
 * @brief Bit shift position and mask for enabling ESM in ESM configuration register 1.
 */
#define PMIC_ESM_CFG1_ESM_EN_SHIFT  (0x06U)
#define PMIC_ESM_CFG1_ESM_EN_MASK   (0x01U << PMIC_ESM_CFG1_ESM_EN_SHIFT)

/**
 * @brief Bit shift position and mask for configuring ESM mode in ESM configuration
 * register 1.
 */
#define PMIC_ESM_CFG1_ESM_CFG_SHIFT (0x07U)
#define PMIC_ESM_CFG1_ESM_CFG_MASK  (0x01U << PMIC_ESM_CFG1_ESM_CFG_SHIFT)

/**
 * @brief Bit shift position and mask for time configuration in ESM
 * configuration register 2.
 */
#define PMIC_ESM_CFG2_TIME_CFG_SHIFT (0x00U)
#define PMIC_ESM_CFG2_TIME_CFG_MASK  (0x03U << PMIC_ESM_CFG2_TIME_CFG_SHIFT)

/**
 * @brief Bit shift position and mask for configuring ESM deglitch in ESM
 * configuration register 2.
 */
#define PMIC_ESM_CFG2_ESM_DGL_SHIFT (0x03U)
#define PMIC_ESM_CFG2_ESM_DGL_MASK  (0x01U << PMIC_ESM_CFG2_ESM_DGL_SHIFT)

/**
 * @brief Bit shift position and mask for configuring ESM level polarity in ESM
 * configuration register 2.
 */
#define PMIC_ESM_CFG2_ESM_LVL_POL_SHIFT (0x04U)
#define PMIC_ESM_CFG2_ESM_LVL_POL_MASK  (0x01U << PMIC_ESM_CFG2_ESM_LVL_POL_SHIFT)

/**
 * @brief Bit shift position and mask for ESM interrupt mask in ESM interrupt
 * configuration register.
 */
#define PMIC_ESM_INT_MASK_SHIFT (0X01U)
#define PMIC_ESM_INT_MASK_MASK  (0X01U << PMIC_ESM_INT_MASK_SHIFT)

/**
 * @brief Bit shift position and mask for ESM delay 1 interrupt mask in ESM
 * interrupt configuration register.
 */
#define PMIC_ESM_DLY1_INT_MASK_SHIFT (0X02U)
#define PMIC_ESM_DLY1_INT_MASK_MASK  (0X01U << PMIC_ESM_DLY1_INT_MASK_SHIFT)

/**
 * @brief Bit shift position and mask for ESM delay 1 interrupt configuration
 * in ESM interrupt configuration register.
 */
#define PMIC_ESM_DLY1_INT_CFG_SHIFT (0X03U)
#define PMIC_ESM_DLY1_INT_CFG_MASK  (0X03U << PMIC_ESM_DLY1_INT_CFG_SHIFT)

/**
 * @brief Bit shift position and mask for ESM delay 2 interrupt mask in ESM interrupt
 * configuration register.
 */
#define PMIC_ESM_DLY2_INT_MASK_SHIFT (0X04U)
#define PMIC_ESM_DLY2_INT_MASK_MASK  (0X01U << PMIC_ESM_DLY2_INT_MASK_SHIFT)

/**
 * @brief Bit shift position and mask for ESM delay 2 interrupt configuration
 * in ESM interrupt configuration register.
 */
#define PMIC_ESM_DLY2_INT_CFG_SHIFT (0X05U)
#define PMIC_ESM_DLY2_INT_CFG_MASK  (0X03U << PMIC_ESM_DLY2_INT_CFG_SHIFT)

/**
 * @brief Bit shift position and mask for ESM error count in the error status
 * register.
 */
#define PMIC_ESM_ERR_STAT_ESM_ERR_CNT_SHIFT (0X00U)
#define PMIC_ESM_ERR_STAT_ESM_ERR_CNT_MASK  (0X07U << PMIC_ESM_ERR_STAT_ESM_ERR_CNT_SHIFT)

/**
 * @brief Bit shift position and mask for ESM error in the error status
 * register.
 */
#define PMIC_ESM_ERR_STAT_ESM_ERR_SHIFT (0X05U)
#define PMIC_ESM_ERR_STAT_ESM_ERR_MASK  (0X01U << PMIC_ESM_ERR_STAT_ESM_ERR_SHIFT)

/**
 * @brief Bit shift position and mask for ESM delay 1 error in the error status
 * register.
 */
#define PMIC_ESM_ERR_STAT_ESM_DLY1_ERR_SHIFT (0X06U)
#define PMIC_ESM_ERR_STAT_ESM_DLY1_ERR_MASK  (0X01U << PMIC_ESM_ERR_STAT_ESM_DLY1_ERR_SHIFT)

/**
 * @brief Bit shift position and mask for ESM delay 2 error in the error status
 * register.
 */
#define PMIC_ESM_ERR_STAT_ESM_DLY2_ERR_SHIFT (0X07U)
#define PMIC_ESM_ERR_STAT_ESM_DLY2_ERR_MASK  (0X01U << PMIC_ESM_ERR_STAT_ESM_DLY2_ERR_SHIFT)

/**
 * @}
 */
/* End of Pmic_ESMFieldDefs */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __PMIC_REGMAP_ESM_H__ */