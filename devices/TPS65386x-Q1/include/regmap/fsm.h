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
#ifndef __PMIC_REGMAP_FSM_H__
#define __PMIC_REGMAP_FSM_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief PMIC FSM module register addresses */
#ifndef SAFETY_CFG_REG
#define SAFETY_CFG_REG         (0x07U)
#endif
#define WAKE_CFG_REG           (0x12U)
#define WAKE_STAT_REG          (0x13U)
#define PWRL_CFG_REG           (0x14U)
#define PWRL_CTRL_REG          (0x15U)
#define STATE_CTRL_REG         (0x16U)
#define STATE_STAT_REG         (0x17U)
#define RST_MCU_TMR_REG_REG    (0x18U)
#define STBY_CFG_REG           (0x19U)
#define RST_MCU_CFG_REG        (0x1AU)
#define SAFE_TMO_CFG_REG       (0x52U)
#define DEV_ERR_STAT_REG       (0x66U)

/** @brief SAFETY_CFG - Safety Configuration */
#ifndef __PMIC_REGMAP_IRQ_H__
#define PWD_TH_SHIFT               (0U)
#define AUTO_BIST_EN_SHIFT         (5U)
#define CFG_REG_CRC_INT_CFG_SHIFT  (6U)
#define PWD_TH_MASK                (0x1FU << PWD_TH_SHIFT)
#define AUTO_BIST_EN_MASK          (1U << AUTO_BIST_EN_SHIFT)
#define CFG_REG_CRC_INT_CFG_MASK   (1U << CFG_REG_CRC_INT_CFG_SHIFT)
#endif

/** @brief WAKE_CFG - Wake Configuration */
#define WAKE1_DGL_CFG_SHIFT    (0U)
#define WAKE2_DGL_CFG_SHIFT    (1U)
#define WAKE1_STBY_CFG_SHIFT   (4U)
#define WAKE2_STBY_CFG_SHIFT   (5U)
#define WAKE1_DGL_CFG_MASK     (1U << WAKE1_DGL_CFG_SHIFT)
#define WAKE2_DGL_CFG_MASK     (1U << WAKE2_DGL_CFG_SHIFT)
#define WAKE1_STBY_CFG_MASK    (1U << WAKE1_STBY_CFG_SHIFT)
#define WAKE2_STBY_CFG_MASK    (1U << WAKE2_STBY_CFG_SHIFT)

/** @brief WAKE_STAT - Wake Status */
#define WAKE1_LVL_DGL_SHIFT    (0U)
#define WAKE2_LVL_DGL_SHIFT    (1U)
#define WAKE_SOURCE_SHIFT      (4U)
#define WAKE1_LVL_DGL_MASK     (1U << WAKE1_LVL_DGL_SHIFT)
#define WAKE2_LVL_DGL_MASK     (1U << WAKE2_LVL_DGL_SHIFT)
#define WAKE_SOURCE_MASK       (0xFU << WAKE_SOURCE_SHIFT)

/** @brief PWRL_CFG - Power Latch Configuration */
#define WAKE1_PWRL_EN_SHIFT        (0U)
#define WAKE2_PWRL_EN_SHIFT        (1U)
#define STBY_ERR_WK_PWRL_EN_SHIFT  (5U)
#define PWRD_DLY_CFG_SHIFT         (6U)
#define WAKE1_PWRL_EN_MASK         (1U << WAKE1_PWRL_EN_SHIFT)
#define WAKE2_PWRL_EN_MASK         (1U << WAKE2_PWRL_EN_SHIFT)
#define STBY_ERR_WK_PWRL_EN_MASK   (1U << STBY_ERR_WK_PWRL_EN_SHIFT)
#define PWRD_DLY_CFG_MASK          (3U << PWRD_DLY_CFG_SHIFT)

/** @brief PWRL_CTRL - Power Latch Control */
#define WAKE1_PWRL_SHIFT       (1U)
#define WAKE2_PWRL_SHIFT       (2U)
#define M_PMIC_WK_PWRL_SHIFT   (4U)
#define TMR_WK_PWRL_SHIFT      (5U)
#define STBY_ERR_WK_PWRL_SHIFT (7U)
#define WAKE1_PWRL_MASK        (1U << WAKE1_PWRL_SHIFT)
#define WAKE2_PWRL_MASK        (1U << WAKE2_PWRL_SHIFT)
#define M_PMIC_WK_PWRL_MASK    (1U << M_PMIC_WK_PWRL_SHIFT)
#define TMR_WK_PWRL_MASK       (1U << TMR_WK_PWRL_SHIFT)
#define STBY_ERR_WK_PWRL_MASK  (1U << STBY_ERR_WK_PWRL_SHIFT)

/** @brief STATE_CTRL - State Control */
#define STATE_REQ_SHIFT    (0U)
#define STATE_REQ_MASK     (7U << STATE_REQ_SHIFT)

/** @brief STATE_STAT - State Status */
#define STATE_SHIFT            (0U)
#define PWRD_DLY_ACTV_SHIFT    (4U)
#define RST_MCU_RQ_FLAG_SHIFT  (5U)
#define STATE_MASK             (0xFU << STATE_SHIFT)
#define PWRD_DLY_ACTV_MASK     (1U << PWRD_DLY_ACTV_SHIFT)
#define RST_MCU_RQ_FLAG_MASK   (1U << RST_MCU_RQ_FLAG_SHIFT)

/** @brief RST_MCU_TMR_REG - Reset MCU Timer Register */
#define RST_MCU_TMR_SHIFT  (0U)
#define RST_MCU_TMR_MASK   (0x7FU << RST_MCU_TMR_SHIFT)

/** @brief STBY_CFG - Standby Configuration */
#define VBAT_STBY_ENTRY_TH_SHIFT   (0U)
#define VBAT_STBY_EXIT_TH_SHIFT    (3U)
#define STBY_EN_SHIFT              (4U)
#define NRST_EN_STBY_SHIFT         (5U)
#define VBAT_STBY_ENTRY_TH_MASK    (7U << VBAT_STBY_ENTRY_TH_SHIFT)
#define VBAT_STBY_EXIT_TH_MASK     (1U << VBAT_STBY_EXIT_TH_SHIFT)
#define STBY_EN_MASK               (1U << STBY_EN_SHIFT)
#define NRST_EN_STBY_MASK          (1U << NRST_EN_STBY_SHIFT)

/** @brief RST_MCU_CFG - Reset MCU Configuration */
#define NRST_EXT_SHIFT         (0U)
#define RST_MCU_TMO_CFG_SHIFT  (6U)
#define NRST_EXT_MASK          (0xFU << NRST_EXT_SHIFT)
#define RST_MCU_TMO_CFG_MASK   (3U << RST_MCU_TMO_CFG_SHIFT)

/** @brief SAFE_TMO_CFG - Safe Timeout Configuration */
#define SAFE_LOCK_TH_SHIFT (0U)
#define SAFE_TMO_SHIFT     (5U)
#define SAFE_LOCK_TH_MASK  (0x1FU << SAFE_LOCK_TH_SHIFT)
#define SAFE_TMO_MASK      (7U << SAFE_TMO_SHIFT)

/** @brief DEV_ERR_STAT - Device Error Status */
#define DEV_ERR_CNT_SHIFT          (0U)
#define SAFE_ST_TMO_RST_ERR_SHIFT  (7U)
#define DEV_ERR_CNT_MASK           (0x1FU << DEV_ERR_CNT_SHIFT)
#define SAFE_ST_TMO_RST_ERR_MASK   (1U << SAFE_ST_TMO_RST_ERR_SHIFT)

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* __PMIC_REGMAP_FSM_H__ */
