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
 * @file irq.h
 *
 * @brief PMIC LLD IRQ module register addresses and bit fields.
 */
#ifndef REGMAP_IRQ_H
#define REGMAP_IRQ_H

/* ========================================================================== */
/*                                Include Files                               */
/* ========================================================================== */
#include "pmic_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                              Register Addresses                            */
/* ========================================================================== */

#define MASK_BUCK_REG         ((uint16_t)0x49U)
#define MASK_LDO_VMON_REG     ((uint16_t)0x4CU)
#define MASK_GPIO_FALL_REG    ((uint16_t)0x4FU)
#define MASK_GPIO_RISE_REG    ((uint16_t)0x50U)
#define MASK_STARTUP_REG      ((uint16_t)0x52U)
#define MASK_MISC_REG         ((uint16_t)0x53U)
#define MASK_MODERATE_ERR_REG ((uint16_t)0x54U)
#define MASK_FSM_ERR_REG      ((uint16_t)0x56U)
#define MASK_ESM_REG          ((uint16_t)0x59U)
#define INT_TOP_REG           ((uint16_t)0x5AU)
#define INT_BUCK_REG          ((uint16_t)0x5BU)
#define INT_LDO_VMON_REG      ((uint16_t)0x5FU)
#define INT_GPIO_REG          ((uint16_t)0x63U)
#define INT_STARTUP_REG       ((uint16_t)0x65U)
#define INT_MISC_REG          ((uint16_t)0x66U)
#define INT_MODERATE_ERR_REG  ((uint16_t)0x67U)
#define INT_SEVERE_ERR_REG    ((uint16_t)0x68U)
#define INT_FSM_ERR_REG       ((uint16_t)0x69U)
#define INT_ESM_REG           ((uint16_t)0x6CU)
#define STAT_BUCK_REG         ((uint16_t)0x6DU)
#define STAT_LDO_VMON_REG     ((uint16_t)0x70U)
#define STAT_MISC_REG         ((uint16_t)0x74U)
#define STAT_MODERATE_ERR_REG ((uint16_t)0x75U)
#define STAT_SEVERE_ERR_REG   ((uint16_t)0x76U)
#define WD_ERR_STATUS_REG     ((uint16_t)0x408U)

/* ========================================================================== */
/*                              Register Bit Fields                           */
/* ========================================================================== */

// MASK_BUCK
#define BUCK4_UVOV_MASK_SHIFT (3U)
#define BUCK3_UVOV_MASK_SHIFT (2U)
#define BUCK2_UVOV_MASK_SHIFT (1U)
#define BUCK1_UVOV_MASK_SHIFT (0U)
#define BUCK4_UVOV_MASK_MASK  (0x01UL << BUCK4_UVOV_MASK_SHIFT)
#define BUCK3_UVOV_MASK_MASK  (0x01UL << BUCK3_UVOV_MASK_SHIFT)
#define BUCK2_UVOV_MASK_MASK  (0x01UL << BUCK2_UVOV_MASK_SHIFT)
#define BUCK1_UVOV_MASK_MASK  (0x01UL << BUCK1_UVOV_MASK_SHIFT)

// MASK_LDO_VMON
#define VMON2_UVOV_MASK_SHIFT (6U)
#define VMON1_UVOV_MASK_SHIFT (5U)
#define VCCA_UVOV_MASK_SHIFT  (4U)
#define LDO3_UVOV_MASK_SHIFT  (2U)
#define LDO2_UVOV_MASK_SHIFT  (1U)
#define LDO1_UVOV_MASK_SHIFT  (0U)
#define VMON2_UVOV_MASK_MASK  (0x01UL << VMON2_UVOV_MASK_SHIFT)
#define VMON1_UVOV_MASK_MASK  (0x01UL << VMON1_UVOV_MASK_SHIFT)
#define VCCA_UVOV_MASK_MASK   (0x01UL << VCCA_UVOV_MASK_SHIFT)
#define LDO3_UVOV_MASK_MASK   (0x01UL << LDO3_UVOV_MASK_SHIFT)
#define LDO2_UVOV_MASK_MASK   (0x01UL << LDO2_UVOV_MASK_SHIFT)
#define LDO1_UVOV_MASK_MASK   (0x01UL << LDO1_UVOV_MASK_SHIFT)

// MASK_GPIO_FALL
#define GPIO6_FALL_MASK_SHIFT (5U)
#define GPIO5_FALL_MASK_SHIFT (4U)
#define GPIO4_FALL_MASK_SHIFT (3U)
#define GPIO3_FALL_MASK_SHIFT (2U)
#define GPIO2_FALL_MASK_SHIFT (1U)
#define GPIO1_FALL_MASK_SHIFT (0U)
#define GPIO6_FALL_MASK_MASK (0x01UL << GPIO6_FALL_MASK_SHIFT)
#define GPIO5_FALL_MASK_MASK (0x01UL << GPIO5_FALL_MASK_SHIFT)
#define GPIO4_FALL_MASK_MASK (0x01UL << GPIO4_FALL_MASK_SHIFT)
#define GPIO3_FALL_MASK_MASK (0x01UL << GPIO3_FALL_MASK_SHIFT)
#define GPIO2_FALL_MASK_MASK (0x01UL << GPIO2_FALL_MASK_SHIFT)
#define GPIO1_FALL_MASK_MASK (0x01UL << GPIO1_FALL_MASK_SHIFT)

// MASK_GPIO_RISE
#define GPIO6_RISE_MASK_SHIFT (5U)
#define GPIO5_RISE_MASK_SHIFT (4U)
#define GPIO4_RISE_MASK_SHIFT (3U)
#define GPIO3_RISE_MASK_SHIFT (2U)
#define GPIO2_RISE_MASK_SHIFT (1U)
#define GPIO1_RISE_MASK_SHIFT (0U)
#define GPIO6_RISE_MASK_MASK  (0x01UL << GPIO6_RISE_MASK_SHIFT)
#define GPIO5_RISE_MASK_MASK  (0x01UL << GPIO5_RISE_MASK_SHIFT)
#define GPIO4_RISE_MASK_MASK  (0x01UL << GPIO4_RISE_MASK_SHIFT)
#define GPIO3_RISE_MASK_MASK  (0x01UL << GPIO3_RISE_MASK_SHIFT)
#define GPIO2_RISE_MASK_MASK  (0x01UL << GPIO2_RISE_MASK_SHIFT)
#define GPIO1_RISE_MASK_MASK  (0x01UL << GPIO1_RISE_MASK_SHIFT)

// MASK_STARTUP
#define SOFT_REBOOT_MASK_SHIFT (5U)
#define FSD_MASK_SHIFT         (4U)
#define PB_SHORT_MASK_SHIFT    (2U)
#define ENABLE_MASK_SHIFT      (1U)
#define VSENSE_MASK_SHIFT      (0U)
#define SOFT_REBOOT_MASK_MASK  (0x01UL << SOFT_REBOOT_MASK_SHIFT)
#define FSD_MASK_MASK          (0x01UL << FSD_MASK_SHIFT)
#define PB_SHORT_MASK_MASK     (0x01UL << PB_SHORT_MASK_SHIFT)
#define ENABLE_MASK_MASK       (0x01UL << ENABLE_MASK_SHIFT)
#define VSENSE_MASK_MASK       (0x01UL << VSENSE_MASK_SHIFT)

// MASK_MISC
#define ADC_CONV_READY_MASK_SHIFT (7U)
#define PB_RISE_MASK_SHIFT        (6U)
#define PB_FALL_MASK_SHIFT        (5U)
#define PB_LONG_MASK_SHIFT        (4U)
#define TWARN_MASK_SHIFT          (3U)
#define REG_UNLOCK_MASK_SHIFT     (2U)
#define EXT_CLK_MASK_SHIFT        (1U)
#define BIST_PASS_MASK_SHIFT      (0U)
#define ADC_CONV_READY_MASK_MASK  (0x01UL << ADC_CONV_READY_MASK_SHIFT)
#define PB_RISE_MASK_MASK         (0x01UL << PB_RISE_MASK_SHIFT)
#define PB_FALL_MASK_MASK         (0x01UL << PB_FALL_MASK_SHIFT)
#define PB_LONG_MASK_MASK         (0x01UL << PB_LONG_MASK_SHIFT)
#define TWARN_MASK_MASK           (0x01UL << TWARN_MASK_SHIFT)
#define REG_UNLOCK_MASK_MASK      (0x01UL << REG_UNLOCK_MASK_SHIFT)
#define EXT_CLK_MASK_MASK         (0x01UL << EXT_CLK_MASK_SHIFT)
#define BIST_PASS_MASK_MASK       (0x01UL << BIST_PASS_MASK_SHIFT)

// MASK_MODERATE_ERR
#define REG_CRC_ERR_MASK_SHIFT (2U)
#define BIST_FAIL_MASK_SHIFT   (1U)
#define REG_CRC_ERR_MASK_MASK  (0x01UL << REG_CRC_ERR_MASK_SHIFT)
#define BIST_FAIL_MASK_MASK    (0x01UL << BIST_FAIL_MASK_SHIFT)

// MASK_FSM_ERR
#define I2C2_ERR_MASK_SHIFT     (5U)
#define COMM_ERR_MASK_SHIFT     (4U)
#define SOC_PWR_ERR_MASK_SHIFT  (3U)
#define MCU_PWR_ERR_MASK_SHIFT  (2U)
#define ORD_SHUTDOWN_MASK_SHIFT (1U)
#define IMM_SHUTDOWN_MASK_SHIFT (0U)
#define I2C2_ERR_MASK_MASK      (0x01UL << I2C2_ERR_MASK_SHIFT)
#define COMM_ERR_MASK_MASK      (0x01UL << COMM_ERR_MASK_SHIFT)
#define SOC_PWR_ERR_MASK_MASK   (0x01UL << SOC_PWR_ERR_MASK_SHIFT)
#define MCU_PWR_ERR_MASK_MASK   (0x01UL << MCU_PWR_ERR_MASK_SHIFT)
#define ORD_SHUTDOWN_MASK_MASK  (0x01UL << ORD_SHUTDOWN_MASK_SHIFT)
#define IMM_SHUTDOWN_MASK_MASK  (0x01UL << IMM_SHUTDOWN_MASK_SHIFT)

// MASK_ESM
#define ESM_MCU_RST_MASK_SHIFT  (5U)
#define ESM_MCU_FAIL_MASK_SHIFT (4U)
#define ESM_MCU_PIN_MASK_SHIFT  (3U)
#define ESM_MCU_RST_MASK_MASK   (0x01UL << ESM_MCU_RST_MASK_SHIFT)
#define ESM_MCU_FAIL_MASK_MASK  (0x01UL << ESM_MCU_FAIL_MASK_SHIFT)
#define ESM_MCU_PIN_MASK_MASK   (0x01UL << ESM_MCU_PIN_MASK_SHIFT)

// INT_TOP
#define FSM_ERR_INT_SHIFT      (7U)
#define SEVERE_ERR_INT_SHIFT   (6U)
#define MODERATE_ERR_INT_SHIFT (5U)
#define MISC_INT_SHIFT         (4U)
#define STARTUP_INT_SHIFT      (3U)
#define GPIO_INT_SHIFT         (2U)
#define LDO_VMON_INT_SHIFT     (1U)
#define BUCK_INT_SHIFT         (0U)
#define FSM_ERR_INT_MASK       (0x01UL << FSM_ERR_INT_SHIFT)
#define SEVERE_ERR_INT_MASK    (0x01UL << SEVERE_ERR_INT_SHIFT)
#define MODERATE_ERR_INT_MASK  (0x01UL << MODERATE_ERR_INT_SHIFT)
#define MISC_INT_MASK          (0x01UL << MISC_INT_SHIFT)
#define STARTUP_INT_MASK       (0x01UL << STARTUP_INT_SHIFT)
#define GPIO_INT_MASK          (0x01UL << GPIO_INT_SHIFT)
#define LDO_VMON_INT_MASK      (0x01UL << LDO_VMON_INT_SHIFT)
#define BUCK_INT_MASK          (0x01UL << BUCK_INT_SHIFT)

// INT_BUCK
#define BUCK4_UVOV_INT_SHIFT (3U)
#define BUCK3_UVOV_INT_SHIFT (2U)
#define BUCK2_UVOV_INT_SHIFT (1U)
#define BUCK1_UVOV_INT_SHIFT (0U)
#define BUCK4_UVOV_INT_MASK  (0x01UL << BUCK4_UVOV_INT_SHIFT)
#define BUCK3_UVOV_INT_MASK  (0x01UL << BUCK3_UVOV_INT_SHIFT)
#define BUCK2_UVOV_INT_MASK  (0x01UL << BUCK2_UVOV_INT_SHIFT)
#define BUCK1_UVOV_INT_MASK  (0x01UL << BUCK1_UVOV_INT_SHIFT)

// INT_LDO_VMON
#define VMON2_UVOV_INT_SHIFT (6U)
#define VMON1_UVOV_INT_SHIFT (5U)
#define VCCA_UVOV_INT_SHIFT  (4U)
#define LDO3_UVOV_INT_SHIFT  (2U)
#define LDO2_UVOV_INT_SHIFT  (1U)
#define LDO1_UVOV_INT_SHIFT  (0U)
#define VMON2_UVOV_INT_MASK  (0x01UL << VMON2_UVOV_INT_SHIFT)
#define VMON1_UVOV_INT_MASK  (0x01UL << VMON1_UVOV_INT_SHIFT)
#define VCCA_UVOV_INT_MASK   (0x01UL << VCCA_UVOV_INT_SHIFT)
#define LDO3_UVOV_INT_MASK   (0x01UL << LDO3_UVOV_INT_SHIFT)
#define LDO2_UVOV_INT_MASK   (0x01UL << LDO2_UVOV_INT_SHIFT)
#define LDO1_UVOV_INT_MASK   (0x01UL << LDO1_UVOV_INT_SHIFT)

// INT_GPIO
#define GPIO6_INT_SHIFT (5U)
#define GPIO5_INT_SHIFT (4U)
#define GPIO4_INT_SHIFT (3U)
#define GPIO3_INT_SHIFT (2U)
#define GPIO2_INT_SHIFT (1U)
#define GPIO1_INT_SHIFT (0U)
#define GPIO6_INT_MASK  (0x01UL << GPIO6_INT_SHIFT)
#define GPIO5_INT_MASK  (0x01UL << GPIO5_INT_SHIFT)
#define GPIO4_INT_MASK  (0x01UL << GPIO4_INT_SHIFT)
#define GPIO3_INT_MASK  (0x01UL << GPIO3_INT_SHIFT)
#define GPIO2_INT_MASK  (0x01UL << GPIO2_INT_SHIFT)
#define GPIO1_INT_MASK  (0x01UL << GPIO1_INT_SHIFT)

// INT_STARTUP
#define SOFT_REBOOT_INT_SHIFT (5U)
#define FSD_INT_SHIFT         (4U)
#define PB_SHORT_INT_SHIFT    (2U)
#define ENABLE_INT_SHIFT      (1U)
#define VSENSE_INT_SHIFT      (0U)
#define SOFT_REBOOT_INT_MASK  (0x01UL << SOFT_REBOOT_INT_SHIFT)
#define FSD_INT_MASK          (0x01UL << FSD_INT_SHIFT)
#define PB_SHORT_INT_MASK     (0x01UL << PB_SHORT_INT_SHIFT)
#define ENABLE_INT_MASK       (0x01UL << ENABLE_INT_SHIFT)
#define VSENSE_INT_MASK       (0x01UL << VSENSE_INT_SHIFT)

// INT_MISC
#define ADC_CONV_READY_INT_SHIFT (7U)
#define PB_RISE_INT_SHIFT        (6U)
#define PB_FALL_INT_SHIFT        (5U)
#define PB_LONG_INT_SHIFT        (4U)
#define TWARN_INT_SHIFT          (3U)
#define REG_UNLOCK_INT_SHIFT     (2U)
#define EXT_CLK_INT_SHIFT        (1U)
#define BIST_PASS_INT_SHIFT      (0U)
#define ADC_CONV_READY_INT_MASK  (0x01UL << ADC_CONV_READY_INT_SHIFT)
#define PB_RISE_INT_MASK         (0x01UL << PB_RISE_INT_SHIFT)
#define PB_FALL_INT_MASK         (0x01UL << PB_FALL_INT_SHIFT)
#define PB_LONG_INT_MASK         (0x01UL << PB_LONG_INT_SHIFT)
#define TWARN_INT_MASK           (0x01UL << TWARN_INT_SHIFT)
#define REG_UNLOCK_INT_MASK      (0x01UL << REG_UNLOCK_INT_SHIFT)
#define EXT_CLK_INT_MASK         (0x01UL << EXT_CLK_INT_SHIFT)
#define BIST_PASS_INT_MASK       (0x01UL << BIST_PASS_INT_SHIFT)

// INT_MODERATE_ERR
#define RECOV_CNT_INT_SHIFT   (3U)
#define REG_CRC_ERR_INT_SHIFT (2U)
#define BIST_FAIL_INT_SHIFT   (1U)
#define TSD_ORD_INT_SHIFT     (0U)
#define RECOV_CNT_INT_MASK    (0x01UL << RECOV_CNT_INT_SHIFT)
#define REG_CRC_ERR_INT_MASK  (0x01UL << REG_CRC_ERR_INT_SHIFT)
#define BIST_FAIL_INT_MASK    (0x01UL << BIST_FAIL_INT_SHIFT)
#define TSD_ORD_INT_MASK      (0x01UL << TSD_ORD_INT_SHIFT)

// INT_SEVERE_ERR
#define BG_XMON_INT_SHIFT   (3U)
#define PFSM_ERR_INT_SHIFT  (2U)
#define VCCA_OVP_INT_SHIFT  (1U)
#define TSD_IMM_INT_SHIFT   (0U)
#define BG_XMON_INT_MASK    (0x01UL << BG_XMON_INT_SHIFT)
#define PFSM_ERR_INT_MASK   (0x01UL << PFSM_ERR_INT_SHIFT)
#define VCCA_OVP_INT_MASK   (0x01UL << VCCA_OVP_INT_SHIFT)
#define TSD_IMM_INT_MASK    (0x01UL << TSD_IMM_INT_SHIFT)

// INT_FSM_ERR
#define WD_INT_SHIFT            (7U)
#define ESM_INT_SHIFT           (6U)
#define I2C2_ERR_INT_SHIFT      (5U)
#define COMM_ERR_INT_SHIFT      (4U)
#define SOC_PWR_ERR_INT_SHIFT   (3U)
#define MCU_PWR_ERR_INT_SHIFT   (2U)
#define ORD_SHUTDOWN_INT_SHIFT  (1U)
#define IMM_SHUTDOWN_INT_SHIFT  (0U)
#define WD_INT_MASK             (0x01UL << WD_INT_SHIFT)
#define ESM_INT_MASK            (0x01UL << ESM_INT_SHIFT)
#define I2C2_ERR_INT_MASK       (0x01UL << I2C2_ERR_INT_SHIFT)
#define COMM_ERR_INT_MASK       (0x01UL << COMM_ERR_INT_SHIFT)
#define SOC_PWR_ERR_INT_MASK    (0x01UL << SOC_PWR_ERR_INT_SHIFT)
#define MCU_PWR_ERR_INT_MASK    (0x01UL << MCU_PWR_ERR_INT_SHIFT)
#define ORD_SHUTDOWN_INT_MASK   (0x01UL << ORD_SHUTDOWN_INT_SHIFT)
#define IMM_SHUTDOWN_INT_MASK   (0x01UL << IMM_SHUTDOWN_INT_SHIFT)

// INT_ESM
#define ESM_MCU_RST_INT_SHIFT  (5U)
#define ESM_MCU_FAIL_INT_SHIFT (4U)
#define ESM_MCU_PIN_INT_SHIFT  (3U)
#define ESM_MCU_RST_INT_MASK   (0x01UL << ESM_MCU_RST_INT_SHIFT)
#define ESM_MCU_FAIL_INT_MASK  (0x01UL << ESM_MCU_FAIL_INT_SHIFT)
#define ESM_MCU_PIN_INT_MASK   (0x01UL << ESM_MCU_PIN_INT_SHIFT)

// WD_ERR_STATUS
#define WD_RST_INT_SHIFT             (7U)
#define WD_FAIL_INT_SHIFT            (6U)
#define WD_ANSW_ERR_SHIFT            (5U)
#define WD_SEQ_ERR_SHIFT             (4U)
#define WD_ANSW_EARLY_SHIFT          (3U)
#define WD_TRIG_EARLY_SHIFT          (2U)
#define WD_TIMEOUT_SHIFT             (1U)
#define WD_LONGWIN_TIMEOUT_INT_SHIFT (0U)
#define WD_RST_INT_MASK              (0x01UL << WD_RST_INT_SHIFT)
#define WD_FAIL_INT_MASK             (0x01UL << WD_FAIL_INT_SHIFT)
#define WD_ANSW_ERR_MASK             (0x01UL << WD_ANSW_ERR_SHIFT)
#define WD_SEQ_ERR_MASK              (0x01UL << WD_SEQ_ERR_SHIFT)
#define WD_ANSW_EARLY_MASK           (0x01UL << WD_ANSW_EARLY_SHIFT)
#define WD_TRIG_EARLY_MASK           (0x01UL << WD_TRIG_EARLY_SHIFT)
#define WD_TIMEOUT_MASK              (0x01UL << WD_TIMEOUT_SHIFT)
#define WD_LONGWIN_TIMEOUT_INT_MASK  (0x01UL << WD_LONGWIN_TIMEOUT_INT_SHIFT)

// STAT_BUCK
#define BUCK4_UVOV_STAT_SHIFT (3U)
#define BUCK3_UVOV_STAT_SHIFT (2U)
#define BUCK2_UVOV_STAT_SHIFT (1U)
#define BUCK1_UVOV_STAT_SHIFT (0U)
#define BUCK4_UVOV_STAT_MASK  (0x01UL <<  BUCK4_UVOV_STAT_SHIFT)
#define BUCK3_UVOV_STAT_MASK  (0x01UL <<  BUCK3_UVOV_STAT_SHIFT)
#define BUCK2_UVOV_STAT_MASK  (0x01UL <<  BUCK2_UVOV_STAT_SHIFT)
#define BUCK1_UVOV_STAT_MASK  (0x01UL <<  BUCK1_UVOV_STAT_SHIFT)

// STAT_LDO_VMON
#define VMON2_UVOV_STAT_SHIFT (6U)
#define VMON1_UVOV_STAT_SHIFT (5U)
#define VCCA_UVOV_STAT_SHIFT  (4U)
#define LDO3_UVOV_STAT_SHIFT  (2U)
#define LDO2_UVOV_STAT_SHIFT  (1U)
#define LDO1_UVOV_STAT_SHIFT  (0U)
#define VMON2_UVOV_STAT_MASK  (0x01UL <<  VMON2_UVOV_STAT_SHIFT)
#define VMON1_UVOV_STAT_MASK  (0x01UL <<  VMON1_UVOV_STAT_SHIFT)
#define VCCA_UVOV_STAT_MASK   (0x01UL <<  VCCA_UVOV_STAT_SHIFT)
#define LDO3_UVOV_STAT_MASK   (0x01UL <<  LDO3_UVOV_STAT_SHIFT)
#define LDO2_UVOV_STAT_MASK   (0x01UL <<  LDO2_UVOV_STAT_SHIFT)
#define LDO1_UVOV_STAT_MASK   (0x01UL <<  LDO1_UVOV_STAT_SHIFT)

// STAT_MISC
#define TWARN_STAT_SHIFT   (3U)
#define EXT_CLK_STAT_SHIFT (1U)
#define TWARN_STAT_MASK    (0x01UL << TWARN_STAT_SHIFT)
#define EXT_CLK_STAT_MASK  (0x01UL << EXT_CLK_STAT_SHIFT)

// STAT_MODERATE_ERR
#define TSD_ORD_STAT_SHIFT (0U)
#define TSD_ORD_STAT_MASK  (0x01UL << TSD_ORD_STAT_SHIFT)

// STAT_SEVERE_ERR
#define BG_XMON_STAT_SHIFT  (3U)
#define VCCA_OVP_STAT_SHIFT (1U)
#define TSD_IMM_STAT_SHIFT  (0U)
#define BG_XMON_STAT_MASK   (0x01UL << BG_XMON_STAT_SHIFT)
#define VCCA_OVP_STAT_MASK  (0x01UL << VCCA_OVP_STAT_SHIFT)
#define TSD_IMM_STAT_MASK   (0x01UL << TSD_IMM_STAT_SHIFT)

#ifdef __cplusplus
}

#endif /* __cplusplus */
#endif /* REGMAP_IRQ_H */
