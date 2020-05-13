/******************************************************************************
 * Copyright (c) 2020 Texas Instruments Incorporated - http://www.ti.com
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
 * \file   pmic_irq_priv.h
 *
 * \brief  The macro definitions, structures and function prototypes for
 *         configuring PMIC IRQ
 */

#ifndef PMIC_IRQ_PRIV_H_
#define PMIC_IRQ_PRIV_H_

/* ========================================================================= */
/*                             Include Files                                 */
/* ========================================================================= */

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================= */
/*                             Macros & Typedefs                             */
/* ========================================================================= */

 /**
 *  \anchor Pmic_InvalidRegAddr
 *  \name PMIC Invalid IRQ register address
 *
 *  @{
 */
#define PMIC_INT_INVALID_REGADDDR              (0xABCDU)
/* @} */

/*!
 * \brief: INT_TOP Register Bit Masks
 */
#define PMIC_INT_TOP_BUCK_MASK                 (0x01U)
#define PMIC_INT_TOP_LDO_VMON_MASK             (0x02U)
#define PMIC_INT_TOP_GPIO_MASK                 (0x04U)
#define PMIC_INT_TOP_STARTUP_MASK              (0x08U)
#define PMIC_INT_TOP_MISC_WARN_MASK            (0x10U)
#define PMIC_INT_TOP_MODERATE_MASK             (0x20U)
#define PMIC_INT_TOP_SEVERE_MASK               (0x40U)
#define PMIC_INT_TOP_FSM_MASK                  (0x80U)

/*!
 * \brief: Interrupt Hierarchy Level 1 Registers Bit Masks
 */
#define PMIC_INT_BUCK_BUCK1_2_INT_MASK         (0x01U)
#define PMIC_INT_BUCK_BUCK3_4_INT_MASK         (0x02U)
#define PMIC_INT_BUCK_BUCK5_INT_MASK           (0x04U)
#define PMIC_INT_LDO_VMON_LDO1_2_MASK          (0x01U)
#define PMIC_INT_LDO_VMON_LDO3_4_MASK          (0x02U)
#define PMIC_INT_LDO_VMON_VCCA_INT_MASK        (0x10U)
#define PMIC_INT_GPIO_GPIO9_INT_MASK           (0x01U)
#define PMIC_INT_GPIO_GPIO10_INT_MASK          (0x02U)
#define PMIC_INT_GPIO_GPIO11_INT_MASK          (0x04U)
#define PMIC_INT_GPIO_GPIO1_8_INT_MASK         (0x08U)
#define PMIC_INT_STARTUP_NPWRON_START_INT_MASK (0x01U)
#define PMIC_INT_STARTUP_ENABLE_INT_MASK       (0x02U)
#define PMIC_INT_STARTUP_RTC_INT_MASK          (0x04U)
#define PMIC_INT_STARTUP_FSD_INT_MASK          (0x10U)
#define PMIC_INT_MISC_BIST_PASS_INT_MASK       (0x01U)
#define PMIC_INT_MISC_EXT_CLK_INT_MASK         (0x02U)
#define PMIC_INT_MISC_TWARN_INT_MASK           (0x08U)
#define PMIC_INT_SEVERE_TSD_IMM_INT_MASK       (0x01U)
#define PMIC_INT_SEVERE_VCCA_OVP_INT_MASK      (0x02U)
#define PMIC_INT_SEVERE_PFSM_ERR_INT_MASK      (0x04U)
#define PMIC_INT_FSM_IMM_SHUTDOWN_INT_MASK     (0x01U)
#define PMIC_INT_FSM_ORD_SHUTDOWN_INT_MASK     (0x02U)
#define PMIC_INT_FSM_MCU_PWR_ERR_INT_MASK      (0x04U)
#define PMIC_INT_FSM_SOC_PWR_ERR_INT_MASK      (0x08U)
#define PMIC_INT_FSM_COMM_ERR_INT_MASK         (0x10U)
#define PMIC_INT_FSM_READBACK_ERR_INT_MASK     (0x20U)
#define PMIC_INT_FSM_ESM_INT_MASK              (0x40U)
#define PMIC_INT_FSM_WD_INT_MASK               (0x80U)

/*!
 * \brief: Interrupt Hierarchy Level 2 Registers Bit Masks
 */
#define PMIC_INT_BUCK1_OV_MASK                 (0x01U)
#define PMIC_INT_BUCK1_UV_MASK                 (0x02U)
#define PMIC_INT_BUCK1_SC_MASK                 (0x04U)
#define PMIC_INT_BUCK1_ILIM_MASK               (0x08U)
#define PMIC_INT_BUCK2_OV_MASK                 (0x10U)
#define PMIC_INT_BUCK2_UV_MASK                 (0x20U)
#define PMIC_INT_BUCK2_SC_MASK                 (0x40U)
#define PMIC_INT_BUCK2_ILIM_MASK               (0x80U)
#define PMIC_INT_BUCK3_OV_MASK                 (0x01U)
#define PMIC_INT_BUCK3_UV_MASK                 (0x02U)
#define PMIC_INT_BUCK3_SC_MASK                 (0x04U)
#define PMIC_INT_BUCK3_ILIM_MASK               (0x08U)
#define PMIC_INT_BUCK4_OV_MASK                 (0x10U)
#define PMIC_INT_BUCK4_UV_MASK                 (0x20U)
#define PMIC_INT_BUCK4_SC_MASK                 (0x40U)
#define PMIC_INT_BUCK4_ILIM_MASK               (0x80U)
#define PMIC_INT_BUCK5_OV_MASK                 (0x01U)
#define PMIC_INT_BUCK5_UV_MASK                 (0x02U)
#define PMIC_INT_BUCK5_SC_MASK                 (0x04U)
#define PMIC_INT_BUCK5_ILIM_MASK               (0x08U)
#define PMIC_INT_LDO1_OV_MASK                  (0x01U)
#define PMIC_INT_LDO1_UV_MASK                  (0x02U)
#define PMIC_INT_LDO1_SC_MASK                  (0x04U)
#define PMIC_INT_LDO1_ILIM_MASK                (0x08U)
#define PMIC_INT_LDO2_OV_MASK                  (0x10U)
#define PMIC_INT_LDO2_UV_MASK                  (0x20U)
#define PMIC_INT_LDO2_SC_MASK                  (0x40U)
#define PMIC_INT_LDO2_ILIM_MASK                (0x80U)
#define PMIC_INT_LDO3_OV_MASK                  (0x01U)
#define PMIC_INT_LDO3_UV_MASK                  (0x02U)
#define PMIC_INT_LDO3_SC_MASK                  (0x04U)
#define PMIC_INT_LDO3_ILIM_MASK                (0x08U)
#define PMIC_INT_LDO4_OV_MASK                  (0x10U)
#define PMIC_INT_LDO4_UV_MASK                  (0x20U)
#define PMIC_INT_LDO4_SC_MASK                  (0x40U)
#define PMIC_INT_LDO4_ILIM_MASK                (0x80U)
#define PMIC_INT_LDO_VCCA_OV_MASK              (0x01U)
#define PMIC_INT_LDO_VCCA_UV_MASK              (0x02U)
#define PMIC_INT_GPIO1_MASK                    (0x01U)
#define PMIC_INT_GPIO2_MASK                    (0x02U)
#define PMIC_INT_GPIO3_MASK                    (0x04U)
#define PMIC_INT_GPIO4_MASK                    (0x08U)
#define PMIC_INT_GPIO5_MASK                    (0x10U)
#define PMIC_INT_GPIO6_MASK                    (0x20U)
#define PMIC_INT_GPIO7_MASK                    (0x40U)
#define PMIC_INT_GPIO8_MASK                    (0x80U)
#define PMIC_INT_GPIO9_MASK                    (0x01U)
#define PMIC_INT_GPIO10_MASK                   (0x02U)
#define PMIC_INT_ENABLE_MASK                   (0x02U)
#define PMIC_INT_FSD_MASK                      (0x04U)
#define PMIC_INT_BIST_PASS_MASK                (0x01U)
#define PMIC_INT_EXT_CLK_MASK                  (0x02U)
#define PMIC_INT_TWARN_MASK                    (0x08U)
#define PMIC_INT_TSD_ORD_MASK                  (0x01U)
#define PMIC_INT_BIST_FAIL_MASK                (0x02U)
#define PMIC_INT_REG_CRC_ERR_MASK              (0x04U)
#define PMIC_INT_RECOV_CNT_MASK                (0x08U)
#define PMIC_INT_SPMI_ERR_MASK                 (0x10U)
#define PMIC_INT_NINT_RDBK_MASK                (0x40U)
#define PMIC_INT_NRSTOUT_RBDK_MASK             (0x80U)
#define PMIC_INT_TSD_IMM_MASK                  (0x01U)
#define PMIC_INT_VCCA_OVP_MASK                 (0x02U)
#define PMIC_INT_PFSM_ERR_MASK                 (0x04U)
#define PMIC_INT_IMM_SHUTDOWN_MASK             (0x01U)
#define PMIC_INT_ORD_SHUTDOWN_MASK             (0x02U)
#define PMIC_INT_MCU_PWR_ERR_MASK              (0x04U)
#define PMIC_INT_SOC_PWR_ERR_MASK              (0x08U)
#define PMIC_INT_COMM_FRM_ERR_MASK             (0x01U)
#define PMIC_INT_COMM_CRC_ERR_MASK             (0x02U)
#define PMIC_INT_COMM_ADR_ERR_MASK             (0x08U)
#define PMIC_INT_COMM_I2C2_CRC_ERR_MASK        (0x20U)
#define PMIC_INT_COMM_I2C2_ADR_ERR_MASK        (0x80U)
#define PMIC_INT_RDBACK_ERR_EN_DRV_MASK        (0x01U)
#define PMIC_INT_RDBACK_ERR_NRST_SOC_MASK      (0x08U)
#define PMIC_INT_ESM_SOC_PIN_MASK              (0x01U)
#define PMIC_INT_ESM_SOC_FAIL_MASK             (0x02U)
#define PMIC_INT_ESM_SOC_RST_MASK              (0x04U)
#define PMIC_INT_ESM_MCU_PIN_MASK              (0x08U)
#define PMIC_INT_ESM_MCU_FAIL_MASK             (0x10U)
#define PMIC_INT_ESM_MCU_RST_MASK              (0x20U)
#define PMIC_INT_WD_ERR_ST_LONGWIN_TOUT_MASK   (0x01U)
#define PMIC_INT_WD_ERR_ST_TOUT_MASK           (0x02U)
#define PMIC_INT_WD_ERR_ST_TRIG_EARLY_MASK     (0x04U)
#define PMIC_INT_WD_ERR_ST_ANSW_EARLY_MASK     (0x08U)
#define PMIC_INT_WD_ERR_ST_SEQ_ERR_MASK        (0x10U)
#define PMIC_INT_WD_ERR_ST_ANSW_ERR_MASK       (0x20U)
#define PMIC_INT_WD_ERR_ST_FAIL_MASK           (0x40U)
#define PMIC_INT_WD_ERR_ST_RST_MASK            (0x80U)

/*!
 * \brief: IRQ Mask Bits to validate error bits
 */
#define PMIC_INT_BUCK_1_3_MASK                 (0x0FU)
#define PMIC_INT_BUCK_2_4_MASK                 (0xF0U)
#define PMIC_INT_BUCK_5_MASK                   (0x0FU)
#define PMIC_INT_LDO_1_3_MASK                  (0x0FU)
#define PMIC_INT_LDO_2_4_MASK                  (0xF0U)
#define PMIC_INT_LDO_VMON_MASK                 (0x13U)
#define PMIC_INT_GPIO1_8_MASK                  (0xFFU)
#define PMIC_INT_GPIO9_MASK                    (0x01U)
#define PMIC_INT_GPIO10_MASK                   (0x02U)
#define PMIC_INT_STARTUP_MASK                  (0x13U)
#define PMIC_INT_MISC_WARN_MASK                (0x0BU)
#define PMIC_INT_SEVERE_MASK                   (0x07U)
#define PMIC_INT_FSM_ERR_MASK                  (0x0FU)
#define PMIC_INT_COMM_ERR_MASK                 (0xABU)
#define PMIC_INT_RDBK_ERR_MASK                 (0x09U)
#define PMIC_INT_ESM_SOC_MASK                  (0x07U)
#define PMIC_INT_ESM_MCU_MASK                  (0x38U)
#define PMIC_INT_ESM_MASK                      (0x3FU)
#define PMIC_INT_WD_ERR_MASK                   (0xFFU)

/*!
 * \brief: Interrupt Hierarchy Level 0 Register offsets
 */
#define PMIC_INT_TOP_REGADDDR                  (0x5AU)

/*!
 * \brief: Interrupt Hierarchy Level 1 Register offsets
 */
#define PMIC_INT_BUCK_REGADDDR                 (0x5BU)
#define PMIC_INT_LDO_VMON_REGADDDR             (0x5FU)
#define PMIC_INT_GPIO_REGADDDR                 (0x63U)
#define PMIC_INT_STARTUP_REGADDDR              (0x65U)
#define PMIC_INT_MISC_WARN_REGADDDR            (0x66U)
#define PMIC_INT_MODERATE_REGADDDR             (0x67U)
#define PMIC_INT_SEVERE_REGADDDR               (0x68U)
#define PMIC_INT_FSM_REGADDDR                  (0x69U)

/** Interrupt Hierarchy Level 2 Register offsets */

/*!
 * \brief: INT_BUCK Sources
 */
#define PMIC_INT_BUCK1_2_REGADDDR              (0x5CU)
#define PMIC_INT_BUCK3_4_REGADDDR              (0x5DU)
#define PMIC_INT_BUCK5_REGADDDR                (0x5EU)

/*!
 * \brief: INT_LDO_VMON Sources
 */
#define PMIC_INT_LDO1_2_REGADDDR               (0x60U)
#define PMIC_INT_LDO3_4_REGADDDR               (0x61U)
#define PMIC_INT_VMON_REGADDDR                 (0x62U)

/*!
 * \brief: INT_GPIO Sources
 */
#define PMIC_INT_GPIO1_8_REGADDDR              (0x64U)

/*!
 * \brief: INT_FSM sources
 */
#define PMIC_INT_COMM_ERR_REGADDDR             (0x6AU)
#define PMIC_INT_RDBACK_ERR_REGADDDR           (0x6BU)
#define PMIC_INT_ESM_REGADDDR                  (0x6CU)
#define PMIC_INT_WD_ERR_STATUS_REGADDDR        (0x408U)

#define PMIC_INT_UNUSED_REGADDDR               (0xFFU)


/*!
 * \brief: Interrupt MASK registers address
 */
#define PMIC_IRQ_MASK_BUCK1_2_REGADDR         (0x49U)
#define PMIC_IRQ_MASK_BUCK3_4_REGADDR         (0x4AU)
#define PMIC_IRQ_MASK_BUCK5_REGADDR           (0x4BU)
#define PMIC_IRQ_MASK_LDO1_2_REGADDR          (0x4CU)
#define PMIC_IRQ_MASK_LDO3_4_REGADDR          (0x4DU)
#define PMIC_IRQ_MASK_VMON_REGADDR            (0x4EU)
#define PMIC_IRQ_MASK_GPIO1_8_FALL_REGADDR    (0x4FU)
#define PMIC_IRQ_MASK_GPIO1_8_RISE_REGADDR    (0x50U)
#define PMIC_IRQ_MASK_STARTUP_REGADDR         (0x52U)
#define PMIC_IRQ_MASK_MISC_REGADDR            (0x53U)
#define PMIC_IRQ_MASK_MODERATE_ERR_REGADDR    (0x54U)
#define PMIC_IRQ_MASK_FSM_ERR_REGADDR         (0x56U)
#define PMIC_IRQ_MASK_COMM_ERR_REGADDR        (0x57U)
#define PMIC_IRQ_MASK_READBACK_ERR_REGADDR    (0x58U)
#define PMIC_IRQ_MASK_ESM_REGADDR             (0x59U)

/*!
 * \brief: MASK value for Interrupts
 */
#define PMIC_IRQ_MASK_BUCK1_2                 (0xBFU)
#define PMIC_IRQ_MASK_BUCK3_4                 (0xBBU)
#define PMIC_IRQ_MASK_BUCK5                   (0x0BU)
#define PMIC_IRQ_MASK_LDO1_2                  (0xBBU)
#define PMIC_IRQ_MASK_LDO3_4                  (0xBBU)
#define PMIC_IRQ_MASK_VMON                    (0x03U)
#define PMIC_IRQ_MASK_GPIO1_8_FALL            (0xFFU)
#define PMIC_IRQ_MASK_GPIO1_8_RISE            (0xFFU)
#define PMIC_IRQ_MASK_STARTUP                 (0x13U)
#define PMIC_IRQ_MASK_MISC                    (0x0BU)
#define PMIC_IRQ_MASK_MODERATE_ERR            (0xFAU)
#define PMIC_IRQ_MASK_FSM_ERR                 (0xFFU)
#define PMIC_IRQ_MASK_COMM_ERR                (0xBBU)
#define PMIC_IRQ_MASK_READBACK_ERR            (0x09U)
#define PMIC_IRQ_MASK_ESM                     (0x3FU)

/*!
 * \brief: Individual interrupt bitmasks for BUCK1_2
 */
#define PMIC_IRQ_BUCK2_ILIM                   (0x8U)
#define PMIC_IRQ_BUCK2_UV                     (0x6U)
#define PMIC_IRQ_BUCK2_OV                     (0x5U)
#define PMIC_IRQ_BUCK1_ILIM                   (0x4U)
#define PMIC_IRQ_BUCK1_UV                     (0x2U)
#define PMIC_IRQ_BUCK1_OV                     (0x1U)

/*!
 * \brief: Individual interrupt bitmasks for BUCK3_4
 */
#define PMIC_IRQ_BUCK4_ILIM                   (0x8U)
#define PMIC_IRQ_BUCK4_UV                     (0x6U)
#define PMIC_IRQ_BUCK4_OV                     (0x5U)
#define PMIC_IRQ_BUCK3_ILIM                   (0x4U)
#define PMIC_IRQ_BUCK3_UV                     (0x2U)
#define PMIC_IRQ_BUCK3_OV                     (0x1U)

/*!
 * \brief: Individual interrupt bitmasks for BUCK5
 */
#define PMIC_IRQ_BUCK5_ILIM                   (0x4U)
#define PMIC_IRQ_BUCK5_UV                     (0x2U)
#define PMIC_IRQ_BUCK5_OV                     (0x1U)

/*!
 * \brief: Individual interrupt bitmasks for LDO1_2
 */
#define PMIC_IRQ_LDO2_ILIM                    (0x8U)
#define PMIC_IRQ_LDO2_UV                      (0x6U)
#define PMIC_IRQ_LDO2_OV                      (0x5U)
#define PMIC_IRQ_LDO1_ILIM                    (0x4U)
#define PMIC_IRQ_LDO1_UV                      (0x2U)
#define PMIC_IRQ_LDO1_OV                      (0x1U)

/*!
 * \brief: Individual interrupt bitmasks for LDO3_4
 */
#define PMIC_IRQ_LDO4_ILIM                    (0x8U)
#define PMIC_IRQ_LDO4_UV                      (0x6U)
#define PMIC_IRQ_LDO4_OV                      (0x5U)
#define PMIC_IRQ_LDO3_ILIM                    (0x4U)
#define PMIC_IRQ_LDO3_UV                      (0x2U)
#define PMIC_IRQ_LDO3_OV                      (0x1U)

/*!
 * \brief: Individual interrupt bitmasks for VMON
 */
#define PMIC_IRQ_VCCA_UV                      (0x2U)
#define PMIC_IRQ_VCCA_OV                      (0x1U)

/*!
 * \brief: Individual interrupt bitmasks for GPIO1_8 FALL
 */
#define PMIC_IRQ_GPIO8_FALL                   (0x8U)
#define PMIC_IRQ_GPIO7_FALL                   (0x7U)
#define PMIC_IRQ_GPIO6_FALL                   (0x6U)
#define PMIC_IRQ_GPIO5_FALL                   (0x5U)
#define PMIC_IRQ_GPIO4_FALL                   (0x4U)
#define PMIC_IRQ_GPIO3_FALL                   (0x3U)
#define PMIC_IRQ_GPIO2_FALL                   (0x2U)
#define PMIC_IRQ_GPIO1_FALL                   (0x1U)

/*!
 * \brief: Individual interrupt bitmasks for GPIO1_8 RISE
 */
#define PMIC_IRQ_GPIO8_RISE                   (0x8U)
#define PMIC_IRQ_GPIO7_RISE                   (0x7U)
#define PMIC_IRQ_GPIO6_RISE                   (0x6U)
#define PMIC_IRQ_GPIO5_RISE                   (0x5U)
#define PMIC_IRQ_GPIO4_RISE                   (0x4U)
#define PMIC_IRQ_GPIO3_RISE                   (0x3U)
#define PMIC_IRQ_GPIO2_RISE                   (0x2U)
#define PMIC_IRQ_GPIO1_RISE                   (0x1U)

/*!
 * \brief: Individual interrupt bitmasks for GPIO9_10 RISE/FALL
 */
#define PMIC_IRQ_GPIO10_RISE                  (0x5U)
#define PMIC_IRQ_GPIO9_RISE                   (0x4U)
#define PMIC_IRQ_GPIO10_FALL                  (0x2U)
#define PMIC_IRQ_GPIO9_FALL                   (0x1U)

/*!
 * \brief: Individual interrupt bitmasks for GPIO9_10 FALL
 */
#define PMIC_IRQ_FSD                          (0x5U)
#define PMIC_IRQ_ENABLE                       (0x2U)

/*!
 * \brief: Individual interrupt bitmasks for MISC Error
 */
#define PMIC_IRQ_TWARN                        (0x4U)
#define PMIC_IRQ_EXT_CLK                      (0x2U)
#define PMIC_IRQ_BIST_PASS                    (0x1U)

/*!
 * \brief: Individual interrupt bitmasks for STARTUP Error
 */
#define PMIC_IRQ_NRSTOUT_READBACK             (0x8U)
#define PMIC_IRQ_NINT_READBACK                (0x7U)
#define PMIC_IRQ_SPMI_ERR                     (0x5U)
#define PMIC_IRQ_REG_CRC_ERR                  (0x3U)
#define PMIC_IRQ_BIST_FAIL                    (0x2U)

/*!
 * \brief: Individual interrupt bitmasks for MODERATE Error
 */
#define PMIC_IRQ_SOC_PWR_ERR                  (0x4U)
#define PMIC_IRQ_MCU_PWR_ERR                  (0x3U)
#define PMIC_IRQ_ORD_SHUTDOWN                 (0x2U)
#define PMIC_IRQ_IMM_SHUTDOWN                 (0x1U)

/*!
 * \brief: Individual interrupt bitmasks for COMM Error
 */
#define PMIC_IRQ_I2C2_ADR_ERR                 (0x8U)
#define PMIC_IRQ_I2C2_CRC_ERR                 (0x6U)
#define PMIC_IRQ_COMM_ADR_ERR                 (0x4U)
#define PMIC_IRQ_COMM_CRC_ERR                 (0x2U)
#define PMIC_IRQ_COMM_FRM_ERR                 (0x1U)

#define PMIC_IRQ_NRSTOUT_SOC_READBACK         (0x4U)
#define PMIC_IRQ_EN_DRV_READBACK              (0x1U)

#define PMIC_IRQ_ESM_MCU_RST                  (0x6U)
#define PMIC_IRQ_ESM_MCU_FAIL                 (0x5U)
#define PMIC_IRQ_ESM_MCU_PIN                  (0x4U)
#define PMIC_IRQ_ESM_SOC_RST                  (0x3U)
#define PMIC_IRQ_ESM_SOC_FAIL                 (0x2U)
#define PMIC_IRQ_ESM_SOC_PIN                  (0x1U)



#ifdef __cplusplus
}

#endif /* __cplusplus */

#endif /* PMIC_IRQ_PRIV_H_ */
