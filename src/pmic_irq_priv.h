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
 *         configuring PMIC IRQ.
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
/*!
 * \brief  Interrupt Hierarchy Level 0 Register offsets
 */
#define PMIC_INT_TOP_REGADDR                  (0x5AU)

/*!
 *  \brief  PMIC Interrupt Hierarchy Level 1 Register offsets
 */
#define PMIC_INT_BUCK_REGADDR                 (0x5BU)
#define PMIC_INT_GPIO_REGADDR                 (0x63U)
#define PMIC_INT_STARTUP_REGADDR              (0x65U)
#define PMIC_INT_MISC_REGADDR                 (0x66U)
#define PMIC_INT_MODERATE_ERR_REGADDR         (0x67U)
#define PMIC_INT_SEVERE_ERR_REGADDR           (0x68U)
#define PMIC_INT_FSM_ERR_REGADDR              (0x69U)

/*! Interrupt Hierarchy Level 2 Register offsets */
/*!
 * \brief  INT_BUCK Sources
 */
#define PMIC_INT_BUCK1_2_REGADDR              (0x5CU)
#define PMIC_INT_BUCK3_4_REGADDR              (0x5DU)

/*!
 * \brief  INT_LDO_VMON Sources
 */
#define PMIC_INT_VMON_REGADDR                 (0x62U)

/*!
 * \brief  INT_GPIO Sources
 */
#define PMIC_INT_GPIO1_8_REGADDR              (0x64U)

/*!
 * \brief  INT_FSM sources
 */
#define PMIC_INT_COMM_ERR_REGADDR             (0x6AU)
#define PMIC_INT_READBACK_ERR_REGADDR         (0x6BU)
#define PMIC_INT_ESM_REGADDR                  (0x6CU)
#define PMIC_WD_ERR_STATUS_REGADDR            (0x408U)

#define PMIC_INT_UNUSED_REGADDR               (0xFFU)

/*!
 *  \brief  PMIC Interrupt MASK register offsets
 */
#define PMIC_MASK_BUCK1_2_REGADDR         (0x49U)
#define PMIC_MASK_BUCK3_4_REGADDR         (0x4AU)
#define PMIC_MASK_VMON_REGADDR            (0x4EU)
#define PMIC_MASK_GPIO1_8_FALL_REGADDR    (0x4FU)
#define PMIC_MASK_GPIO1_8_RISE_REGADDR    (0x50U)
#define PMIC_MASK_STARTUP_REGADDR         (0x52U)
#define PMIC_MASK_MISC_REGADDR            (0x53U)
#define PMIC_MASK_MODERATE_ERR_REGADDR    (0x54U)
#define PMIC_MASK_FSM_ERR_REGADDR         (0x56U)
#define PMIC_MASK_COMM_ERR_REGADDR        (0x57U)
#define PMIC_MASK_READBACK_ERR_REGADDR    (0x58U)
#define PMIC_MASK_ESM_REGADDR             (0x59U)

/*!
 * \brief  INT_TOP Register Bit Masks
 */
#define PMIC_INT_TOP_BUCK_INT_MASK                 (0x01U)
#define PMIC_INT_TOP_GPIO_INT_MASK                 (0x04U)
#define PMIC_INT_TOP_STARTUP_INT_MASK              (0x08U)
#define PMIC_INT_TOP_MISC_INT_MASK                 (0x10U)
#define PMIC_INT_TOP_MODERATE_ERR_INT_MASK         (0x20U)
#define PMIC_INT_TOP_SEVERE_ERR_INT_MASK           (0x40U)
#define PMIC_INT_TOP_FSM_ERR_INT_MASK              (0x80U)

/*! Interrupt Hierarchy Level 1 Registers Bit Masks */
/*!
 * \brief  Buck Interrupt Mask
 */
#define PMIC_INT_BUCK_BUCK1_2_INT_MASK         (0x01U)
#define PMIC_INT_BUCK_BUCK3_4_INT_MASK         (0x02U)

/*!
 * \brief  GPIO Interrupt Mask
 */
#define PMIC_INT_GPIO_GPIO9_INT_MASK           (0x01U)
#define PMIC_INT_GPIO_GPIO10_INT_MASK          (0x02U)
#define PMIC_INT_GPIO_GPIO1_8_INT_MASK         (0x08U)

/*!
 * \brief  FSM Error Interrupt Mask
 */
#define PMIC_INT_FSM_ERR_IMM_SHUTDOWN_INT_MASK     (0x01U)
#define PMIC_INT_FSM_ERR_ORD_SHUTDOWN_INT_MASK     (0x02U)
#define PMIC_INT_FSM_ERR_MCU_PWR_ERR_INT_MASK      (0x04U)
#define PMIC_INT_FSM_ERR_SOC_PWR_ERR_INT_MASK      (0x08U)
#define PMIC_INT_FSM_ERR_COMM_ERR_INT_MASK         (0x10U)
#define PMIC_INT_FSM_ERR_READBACK_ERR_INT_MASK     (0x20U)
#define PMIC_INT_FSM_ERR_ESM_INT_MASK              (0x40U)
#define PMIC_INT_FSM_ERR_WD_INT_MASK               (0x80U)

/*! Interrupt Hierarchy Level 2 Registers Bit Masks */
/*!
 * \brief  PMIC_INT_BUCK1_2 Register Bit Masks
 */
#define PMIC_INT_BUCK1_2_BUCK1_OV_INT_MASK                       (0x01U)
#define PMIC_INT_BUCK1_2_BUCK1_UV_INT_MASK                       (0x02U)
#define PMIC_INT_BUCK1_2_BUCK1_SC_INT_MASK                       (0x04U)
#define PMIC_INT_BUCK1_2_BUCK1_ILIM_INT_MASK                     (0x08U)
#define PMIC_INT_BUCK1_2_BUCK2_OV_INT_MASK                       (0x10U)
#define PMIC_INT_BUCK1_2_BUCK2_UV_INT_MASK                       (0x20U)
#define PMIC_INT_BUCK1_2_BUCK2_SC_INT_MASK                       (0x40U)
#define PMIC_INT_BUCK1_2_BUCK2_ILIM_INT_MASK                     (0x80U)

/*!
 * \brief  PMIC_INT_BUCK3_4 Register Bit Masks
 */
#define PMIC_INT_BUCK3_4_BUCK3_OV_INT_MASK                       (0x01U)
#define PMIC_INT_BUCK3_4_BUCK3_UV_INT_MASK                       (0x02U)
#define PMIC_INT_BUCK3_4_BUCK3_SC_INT_MASK                       (0x04U)
#define PMIC_INT_BUCK3_4_BUCK3_ILIM_INT_MASK                     (0x08U)
#define PMIC_INT_BUCK3_4_BUCK4_OV_INT_MASK                       (0x10U)
#define PMIC_INT_BUCK3_4_BUCK4_UV_INT_MASK                       (0x20U)
#define PMIC_INT_BUCK3_4_BUCK4_SC_INT_MASK                       (0x40U)
#define PMIC_INT_BUCK3_4_BUCK4_ILIM_INT_MASK                     (0x80U)

/*!
 * \brief  PMIC_INT_VMON Register Bit Masks
 */
#define PMIC_INT_VMON_VCCA_OV_INT_MASK                           (0x01U)
#define PMIC_INT_VMON_VCCA_UV_INT_MASK                           (0x02U)

/*!
 * \brief  PMIC_INT_GPIO1_8 Register Bit Masks
 */
#define PMIC_INT_GPIO1_8_GPIO1_INT_MASK                          (0x01U)
#define PMIC_INT_GPIO1_8_GPIO2_INT_MASK                          (0x02U)
#define PMIC_INT_GPIO1_8_GPIO3_INT_MASK                          (0x04U)
#define PMIC_INT_GPIO1_8_GPIO4_INT_MASK                          (0x08U)
#define PMIC_INT_GPIO1_8_GPIO5_INT_MASK                          (0x10U)
#define PMIC_INT_GPIO1_8_GPIO6_INT_MASK                          (0x20U)
#define PMIC_INT_GPIO1_8_GPIO7_INT_MASK                          (0x40U)
#define PMIC_INT_GPIO1_8_GPIO8_INT_MASK                          (0x80U)

/*!
 * \brief  PMIC_INT_STARTUP Register Bit Masks
 */
#define PMIC_INT_STARTUP_ENABLE_INT_MASK                         (0x02U)
#define PMIC_INT_STARTUP_FSD_INT_MASK                            (0x10U)
/*! Valid only PG 2.0 */
#define PMIC_INT_STARTUP_SOFT_REBOOT_INT_MASK                    (0x20U)

/*!
 * \brief  PMIC_INT_MISC Register Bit Masks
 */
#define PMIC_INT_MISC_BIST_PASS_INT_MASK                         (0x01U)
#define PMIC_INT_MISC_EXT_CLK_INT_MASK                           (0x02U)
#define PMIC_INT_MISC_TWARN_INT_MASK                             (0x08U)

/*!
 * \brief  PMIC_INT_MODERATE_ERR Register Bit Masks
 */
#define PMIC_INT_MODERATE_ERR_TSD_ORD_INT_MASK                   (0x01U)
#define PMIC_INT_MODERATE_ERR_BIST_FAIL_INT_MASK                 (0x02U)
#define PMIC_INT_MODERATE_ERR_REG_CRC_ERR_INT_MASK               (0x04U)

/** \brief PMIC_INT_MODERATE_ERR_NRSTOUT_READBACK_INT_MASK Bit Positions
 *         valid only for PG2.0 */
#define PMIC_INT_MODERATE_ERR_RECOV_CNT_INT_MASK                 (0x08U)

/** \brief PMIC_INT_MODERATE_ERR_PFSM_ERR_INT_MASK_PG_1_0 Bit Positions
 *         valid only for PG1.0 */
#define PMIC_INT_MODERATE_ERR_PFSM_ERR_INT_MASK_PG_1_0           (0x08U)

#define PMIC_INT_MODERATE_ERR_SPMI_ERR_INT_MASK                  (0x10U)

/** \brief PMIC_INT_MODERATE_ERR_NINT_READBACK_INT_MASK Bit Positions
 *         valid only for PG2.0 */
#define PMIC_INT_MODERATE_ERR_NINT_READBACK_INT_MASK             (0x40U)

/** \brief PMIC_INT_MODERATE_ERR_RECOV_CNT_INT_MASK_PG_1_0 Bit Positions
 *         valid only for PG1.0 */
#define PMIC_INT_MODERATE_ERR_RECOV_CNT_INT_MASK_PG_1_0          (0x40U)

/** \brief PMIC_INT_MODERATE_ERR_NRSTOUT_READBACK_INT_MASK Bit Positions
 *         valid only for PG2.0 */
#define PMIC_INT_MODERATE_ERR_NRSTOUT_READBACK_INT_MASK          (0x80U)

/*!
 * \brief  PMIC_INT_SEVERE_ERR Register Bit Masks
 */
#define PMIC_INT_SEVERE_ERR_TSD_IMM_INT_MASK                     (0x01U)
#define PMIC_INT_SEVERE_ERR_VCCA_OVP_INT_MASK                    (0x02U)
#define PMIC_INT_SEVERE_ERR_PFSM_ERR_INT_MASK                    (0x04U)

/*!
 * \brief  PMIC_INT_COMM_ERR Register Bit Masks
 */
#define PMIC_INT_COMM_ERR_COMM_FRM_ERR_INT_MASK                  (0x01U)
#define PMIC_INT_COMM_ERR_COMM_CRC_ERR_INT_MASK                  (0x02U)
#define PMIC_INT_COMM_ERR_COMM_ADR_ERR_INT_MASK                  (0x08U)
#define PMIC_INT_COMM_ERR_I2C2_CRC_ERR_INT_MASK                  (0x20U)
#define PMIC_INT_COMM_ERR_I2C2_ADR_ERR_INT_MASK                  (0x80U)

/*!
 * \brief  PMIC_INT_READBACK_ERR Register Bit Masks
 */
#define PMIC_INT_READBACK_ERR_EN_DRV_READBACK_INT_MASK           (0x01U)

/** \brief PMIC_INT_READBACK_ERR_NINT_READBACK_INT_MASK Bit Positions
 *         valid only for PG1.0 */
#define PMIC_INT_READBACK_ERR_NINT_READBACK_INT_MASK             (0x02U)

/** \brief PMIC_INT_READBACK_ERR_NRSTOUT_READBACK_INT_MASK Bit Positions
 *         valid only for PG1.0 */
#define PMIC_INT_READBACK_ERR_NRSTOUT_READBACK_INT_MASK          (0x04U)

#define PMIC_INT_READBACK_ERR_NRSTOUT_SOC_READBACK_INT_MASK      (0x08U)

/*!
 * \brief  PMIC_INT_ESM Register Bit Masks
 */
#define PMIC_INT_ESM_ESM_SOC_PIN_INT_MASK                        (0x01U)
#define PMIC_INT_ESM_ESM_SOC_FAIL_INT_MASK                       (0x02U)
#define PMIC_INT_ESM_ESM_SOC_RST_INT_MASK                        (0x04U)
#define PMIC_INT_ESM_ESM_MCU_PIN_INT_MASK                        (0x08U)
#define PMIC_INT_ESM_ESM_MCU_FAIL_INT_MASK                       (0x10U)
#define PMIC_INT_ESM_ESM_MCU_RST_INT_MASK                        (0x20U)

/*!
 * \brief  IRQ Mask Bits to validate error bits
 */
#define PMIC_INT_WD_ERR_MASK                   (0xC1U)

/*!
 * \brief  PMIC Interrupt register Bit Positions
 */
/*! PMIC_INT_ESM Register Bit Positions */
#define PMIC_INT_ESM_ESM_MCU_RST_INT_SHIFT                      (0x5U)
#define PMIC_INT_ESM_ESM_MCU_FAIL_INT_SHIFT                     (0x4U)
#define PMIC_INT_ESM_ESM_MCU_PIN_INT_SHIFT                      (0x3U)
#define PMIC_INT_ESM_ESM_SOC_RST_INT_SHIFT                      (0x2U)
#define PMIC_INT_ESM_ESM_SOC_FAIL_INT_SHIFT                     (0x1U)
#define PMIC_INT_ESM_ESM_SOC_PIN_INT_SHIFT                      (0x0U)

/*! PMIC_INT_READBACK_ERR Register Bit Positions */
#define PMIC_INT_READBACK_ERR_NRSTOUT_SOC_READBACK_INT_SHIFT    (0x3U)

/** \brief PMIC_INT_READBACK_ERR_NRSTOUT_READBACK_INT_SHIFT Bit Positions
 *         valid only for PG1.0 */
#define PMIC_INT_READBACK_ERR_NRSTOUT_READBACK_INT_SHIFT        (0x2U)

/** \brief PMIC_INT_READBACK_ERR_NINT_READBACK_INT_SHIFT Bit Positions
 *         valid only for PG1.0 */
#define PMIC_INT_READBACK_ERR_NINT_READBACK_INT_SHIFT           (0x1U)

#define PMIC_INT_READBACK_ERR_EN_DRV_READBACK_INT_SHIFT         (0x0U)

/*! PMIC_INT_COMM_ERR Register Bit Positions */
#define PMIC_INT_COMM_ERR_I2C2_ADR_ERR_INT_SHIFT                (0x7U)
#define PMIC_INT_COMM_ERR_I2C2_CRC_ERR_INT_SHIFT                (0x5U)
#define PMIC_INT_COMM_ERR_COMM_ADR_ERR_INT_SHIFT                (0x3U)
#define PMIC_INT_COMM_ERR_COMM_CRC_ERR_INT_SHIFT                (0x1U)
#define PMIC_INT_COMM_ERR_COMM_FRM_ERR_INT_SHIFT                (0x0U)

/*! PMIC_INT_FSM_ERR Register Bit Positions */
#define PMIC_INT_FSM_ERR_SOC_PWR_ERR_INT_SHIFT                  (0x3U)
#define PMIC_INT_FSM_ERR_MCU_PWR_ERR_INT_SHIFT                  (0x2U)
#define PMIC_INT_FSM_ERR_ORD_SHUTDOWN_INT_SHIFT                 (0x1U)
#define PMIC_INT_FSM_ERR_IMM_SHUTDOWN_INT_SHIFT                 (0x0U)

/*! PMIC_INT_SEVERE_ERR Register Bit Positions */
#define PMIC_INT_SEVERE_ERR_PFSM_ERR_INT_SHIFT                  (0x2U)
#define PMIC_INT_SEVERE_ERR_VCCA_OVP_INT_SHIFT                  (0x1U)
#define PMIC_INT_SEVERE_ERR_TSD_IMM_INT_SHIFT                   (0x0U)

/*! PMIC_INT_MODERATE_ERR Register Bit Positions */

/** \brief PMIC_INT_MODERATE_ERR_NRSTOUT_READBACK_INT_SHIFT Bit Positions
 *         valid only for PG2.0 */
#define PMIC_INT_MODERATE_ERR_NRSTOUT_READBACK_INT_SHIFT        (0x7U)
/** \brief PMIC_INT_MODERATE_ERR_NINT_READBACK_INT_SHIFT Bit Positions
 *         valid only for PG2.0 */
#define PMIC_INT_MODERATE_ERR_NINT_READBACK_INT_SHIFT           (0x6U)

/** \brief PMIC_INT_MODERATE_ERR_RECOV_CNT_INT_SHIFT Bit Positions
 *         valid only for PG1.0 */
#define PMIC_INT_MODERATE_ERR_RECOV_CNT_INT_SHIFT_PG_1_0        (0x6U)

#define PMIC_INT_MODERATE_ERR_SPMI_ERR_INT_SHIFT                (0x4U)

/** \brief PMIC_INT_MODERATE_ERR_PFSM_ERR_INT_SHIFT Bit Positions
 *         valid only for PG1.0 */
#define PMIC_INT_MODERATE_ERR_PFSM_ERR_INT_SHIFT_PG_1_0         (0x3U)

/** \brief PMIC_INT_MODERATE_ERR_RECOV_CNT_INT_SHIFT Bit Positions
 *         valid only for PG2.0 */
#define PMIC_INT_MODERATE_ERR_RECOV_CNT_INT_SHIFT               (0x3U)

#define PMIC_INT_MODERATE_ERR_REG_CRC_ERR_INT_SHIFT             (0x2U)
#define PMIC_INT_MODERATE_ERR_BIST_FAIL_INT_SHIFT               (0x1U)
#define PMIC_INT_MODERATE_ERR_TSD_ORD_INT_SHIFT                 (0x0U)

/*! PMIC_INT_MISC Register Bit Positions */
#define PMIC_INT_MISC_TWARN_INT_SHIFT                           (0x3U)
#define PMIC_INT_MISC_EXT_CLK_INT_SHIFT                         (0x1U)
#define PMIC_INT_MISC_BIST_PASS_INT_SHIFT                       (0x0U)

/*! PMIC_INT_STARTUP Register Bit Positions */
/*! Valid only for PG2.0 */
#define PMIC_INT_STARTUP_SOFT_REBOOT_INT_SHIFT                  (0x5U)
#define PMIC_INT_STARTUP_FSD_INT_SHIFT                          (0x4U)
#define PMIC_INT_STARTUP_ENABLE_INT_SHIFT                       (0x1U)

/*! PMIC_INT_GPIO1_8 Register Bit Positions */
#define PMIC_INT_GPIO1_8_GPIO8_INT_SHIFT                        (0x7U)
#define PMIC_INT_GPIO1_8_GPIO7_INT_SHIFT                        (0x6U)
#define PMIC_INT_GPIO1_8_GPIO6_INT_SHIFT                        (0x5U)
#define PMIC_INT_GPIO1_8_GPIO5_INT_SHIFT                        (0x4U)
#define PMIC_INT_GPIO1_8_GPIO4_INT_SHIFT                        (0x3U)
#define PMIC_INT_GPIO1_8_GPIO3_INT_SHIFT                        (0x2U)
#define PMIC_INT_GPIO1_8_GPIO2_INT_SHIFT                        (0x1U)
#define PMIC_INT_GPIO1_8_GPIO1_INT_SHIFT                        (0x0U)

/*! PMIC_INT_GPIO Register Bit Positions */
#define PMIC_INT_GPIO_GPIO10_INT_SHIFT                          (0x1U)
#define PMIC_INT_GPIO_GPIO9_INT_SHIFT                           (0x0U)

/*! PMIC_INT_VMON Register Bit Positions */
#define PMIC_INT_VMON_VCCA_UV_INT_SHIFT                         (0x1U)
#define PMIC_INT_VMON_VCCA_OV_INT_SHIFT                         (0x0U)

/*! PMIC_INT_BUCK3_4 Register Bit Positions */
#define PMIC_INT_BUCK3_4_BUCK4_ILIM_INT_SHIFT                   (0x7U)
#define PMIC_INT_BUCK3_4_BUCK4_SC_INT_SHIFT                     (0x6U)
#define PMIC_INT_BUCK3_4_BUCK4_UV_INT_SHIFT                     (0x5U)
#define PMIC_INT_BUCK3_4_BUCK4_OV_INT_SHIFT                     (0x4U)
#define PMIC_INT_BUCK3_4_BUCK3_ILIM_INT_SHIFT                   (0x3U)
#define PMIC_INT_BUCK3_4_BUCK3_SC_INT_SHIFT                     (0x2U)
#define PMIC_INT_BUCK3_4_BUCK3_UV_INT_SHIFT                     (0x1U)
#define PMIC_INT_BUCK3_4_BUCK3_OV_INT_SHIFT                     (0x0U)

/*! PMIC_INT_BUCK1_2 Register Bit Positions */
#define PMIC_INT_BUCK1_2_BUCK2_ILIM_INT_SHIFT                   (0x7U)
#define PMIC_INT_BUCK1_2_BUCK2_SC_INT_SHIFT                     (0x6U)
#define PMIC_INT_BUCK1_2_BUCK2_UV_INT_SHIFT                     (0x5U)
#define PMIC_INT_BUCK1_2_BUCK2_OV_INT_SHIFT                     (0x4U)
#define PMIC_INT_BUCK1_2_BUCK1_ILIM_INT_SHIFT                   (0x3U)
#define PMIC_INT_BUCK1_2_BUCK1_SC_INT_SHIFT                     (0x2U)
#define PMIC_INT_BUCK1_2_BUCK1_UV_INT_SHIFT                     (0x1U)
#define PMIC_INT_BUCK1_2_BUCK1_OV_INT_SHIFT                     (0x0U)

/*!
 * \brief  PMIC Mask register Bit Positions
 */

/*! PMIC_MASK_ESM Register Bit Positions */
#define PMIC_MASK_ESM_ESM_MCU_RST_MASK_SHIFT                        (0x5U)
#define PMIC_MASK_ESM_ESM_MCU_FAIL_MASK_SHIFT                       (0x4U)
#define PMIC_MASK_ESM_ESM_MCU_PIN_MASK_SHIFT                        (0x3U)
#define PMIC_MASK_ESM_ESM_SOC_RST_MASK_SHIFT                        (0x2U)
#define PMIC_MASK_ESM_ESM_SOC_FAIL_MASK_SHIFT                       (0x1U)
#define PMIC_MASK_ESM_ESM_SOC_PIN_MASK_SHIFT                        (0x0U)

/*! PMIC_MASK_READBACK_ERR Register Bit Positions */
#define PMIC_MASK_READBACK_ERR_NRSTOUT_SOC_READBACK_MASK_SHIFT      (0x3U)

/** \brief PMIC_MASK_READBACK_ERR_NRSTOUT_READBACK_MASK_SHIFT Bit Positions
 *         valid only for PG1.0 */
#define PMIC_MASK_READBACK_ERR_NRSTOUT_READBACK_MASK_SHIFT          (0x2U)

/** \brief PMIC_MASK_READBACK_ERR_NINT_READBACK_MASK_SHIFT Bit Positions
 *         valid only for PG1.0 */
#define PMIC_MASK_READBACK_ERR_NINT_READBACK_MASK_SHIFT             (0x1U)

#define PMIC_MASK_READBACK_ERR_EN_DRV_READBACK_MASK_SHIFT           (0x0U)

/*! PMIC_MASK_COMM_ERR Register Bit Positions */
#define PMIC_MASK_COMM_ERR_I2C2_ADR_ERR_MASK_SHIFT                  (0x7U)
#define PMIC_MASK_COMM_ERR_I2C2_CRC_ERR_MASK_SHIFT                  (0x5U)
#define PMIC_MASK_COMM_ERR_COMM_ADR_ERR_MASK_SHIFT                  (0x3U)
#define PMIC_MASK_COMM_ERR_COMM_CRC_ERR_MASK_SHIFT                  (0x1U)
#define PMIC_MASK_COMM_ERR_COMM_FRM_ERR_MASK_SHIFT                  (0x0U)

/*! PMIC_MASK_FSM_ERR Register Bit Positions */
#define PMIC_MASK_FSM_ERR_SOC_PWR_ERR_MASK_SHIFT                    (0x3U)
#define PMIC_MASK_FSM_ERR_MCU_PWR_ERR_MASK_SHIFT                    (0x2U)
#define PMIC_MASK_FSM_ERR_ORD_SHUTDOWN_MASK_SHIFT                   (0x1U)
#define PMIC_MASK_FSM_ERR_IMM_SHUTDOWN_MASK_SHIFT                   (0x0U)

/*! PMIC_MASK_MODERATE_ERR Register Bit Positions */

/** \brief PMIC_MASK_MODERATE_ERR_NRSTOUT_READBACK_MASK_SHIFT Bit Positions
 *         valid only for PG2.0 */
#define PMIC_MASK_MODERATE_ERR_NRSTOUT_READBACK_MASK_SHIFT          (0x7U)
/** \brief PMIC_MASK_MODERATE_ERR_NINT_READBACK_MASK_SHIFT Bit Positions
 *         valid only for PG2.0 */
#define PMIC_MASK_MODERATE_ERR_NINT_READBACK_MASK_SHIFT             (0x6U)

#define PMIC_MASK_MODERATE_ERR_SPMI_ERR_MASK_SHIFT                  (0x4U)
#define PMIC_MASK_MODERATE_ERR_REG_CRC_ERR_MASK_SHIFT               (0x2U)
#define PMIC_MASK_MODERATE_ERR_BIST_FAIL_MASK_SHIFT                 (0x1U)

/*! PMIC_MASK_MISC Register Bit Positions */
#define PMIC_MASK_MISC_EXT_CLK_MASK_SHIFT                           (0x1U)
#define PMIC_MASK_MISC_BIST_PASS_MASK_SHIFT                         (0x0U)

/*! PMIC_MASK_STARTUP Register Bit Positions */
/*! Valid only for PG 2.0 */
#define PMIC_MASK_STARTUP_SOFT_REBOOT_MASK_SHIFT                    (0x5U)
#define PMIC_MASK_STARTUP_FSD_MASK_SHIFT                            (0x4U)
#define PMIC_MASK_STARTUP_ENABLE_MASK_SHIFT                         (0x1U)

/*! PMIC_MASK_GPIO1_8_RISE Register Bit Positions */
#define PMIC_MASK_GPIO1_8_RISE_GPIO8_RISE_MASK_SHIFT                (0x7U)
#define PMIC_MASK_GPIO1_8_RISE_GPIO7_RISE_MASK_SHIFT                (0x6U)
#define PMIC_MASK_GPIO1_8_RISE_GPIO6_RISE_MASK_SHIFT                (0x5U)
#define PMIC_MASK_GPIO1_8_RISE_GPIO5_RISE_MASK_SHIFT                (0x4U)
#define PMIC_MASK_GPIO1_8_RISE_GPIO4_RISE_MASK_SHIFT                (0x3U)
#define PMIC_MASK_GPIO1_8_RISE_GPIO3_RISE_MASK_SHIFT                (0x2U)
#define PMIC_MASK_GPIO1_8_RISE_GPIO2_RISE_MASK_SHIFT                (0x1U)
#define PMIC_MASK_GPIO1_8_RISE_GPIO1_RISE_MASK_SHIFT                (0x0U)

/*! PMIC_MASK_GPIO1_8_FALL Register Bit Positions */
#define PMIC_MASK_GPIO1_8_FALL_GPIO8_FALL_MASK_SHIFT                (0x7U)
#define PMIC_MASK_GPIO1_8_FALL_GPIO7_FALL_MASK_SHIFT                (0x6U)
#define PMIC_MASK_GPIO1_8_FALL_GPIO6_FALL_MASK_SHIFT                (0x5U)
#define PMIC_MASK_GPIO1_8_FALL_GPIO5_FALL_MASK_SHIFT                (0x4U)
#define PMIC_MASK_GPIO1_8_FALL_GPIO4_FALL_MASK_SHIFT                (0x3U)
#define PMIC_MASK_GPIO1_8_FALL_GPIO3_FALL_MASK_SHIFT                (0x2U)
#define PMIC_MASK_GPIO1_8_FALL_GPIO2_FALL_MASK_SHIFT                (0x1U)
#define PMIC_MASK_GPIO1_8_FALL_GPIO1_FALL_MASK_SHIFT                (0x0U)

/*! PMIC_MASK_BUCK3_4 Register Bit Positions */
#define PMIC_MASK_BUCK3_4_BUCK4_ILIM_MASK_SHIFT                     (0x7U)
#define PMIC_MASK_BUCK3_4_BUCK4_UV_MASK_SHIFT                       (0x5U)
#define PMIC_MASK_BUCK3_4_BUCK4_OV_MASK_SHIFT                       (0x4U)
#define PMIC_MASK_BUCK3_4_BUCK3_ILIM_MASK_SHIFT                     (0x3U)
#define PMIC_MASK_BUCK3_4_BUCK3_UV_MASK_SHIFT                       (0x1U)
#define PMIC_MASK_BUCK3_4_BUCK3_OV_MASK_SHIFT                       (0x0U)

/*! PMIC_MASK_BUCK1_2 Register Bit Positions */
#define PMIC_MASK_BUCK1_2_BUCK2_ILIM_MASK_SHIFT                     (0x7U)
#define PMIC_MASK_BUCK1_2_BUCK2_UV_MASK_SHIFT                       (0x5U)
#define PMIC_MASK_BUCK1_2_BUCK2_OV_MASK_SHIFT                       (0x4U)
#define PMIC_MASK_BUCK1_2_BUCK1_ILIM_MASK_SHIFT                     (0x3U)
#define PMIC_MASK_BUCK1_2_BUCK1_UV_MASK_SHIFT                       (0x1U)
#define PMIC_MASK_BUCK1_2_BUCK1_OV_MASK_SHIFT                       (0x0U)

/*! PMIC INVALID MACROS */
#define PMIC_IRQ_INVALID_REGADDR                                    (0x0U)
#define PMIC_IRQ_INVALID_BIT_SHIFT                                  (0x0U)
#define PMIC_INVALID_DEVICE                                         (0xFFU)

/*!
 * \brief Bit field Value for intrMaskBitPos/intrClrBitPos/
 *        gpioRiseMaskBitPos/gpioFallMaskBitPos
 */
#define PMIC_IRQ_MASK_CLR_BITFIELD                                  (1U)

/*!
 * \brief Mask Value of the PMIC IRQ
 */
#define PMIC_IRQ_MASK_VAL_1                                         (1U)

/*==========================================================================*/
/*                         Structures and Enums                             */
/*==========================================================================*/
/*!
* \brief   PMIC Interrupt details object structure.
*
* \param   intrClrRegAddr    Interrupt Clear Register Address.
* \param   intrClrBitPos     Interrupt Clear Register Bit Position.
* \param   intrMaskRegAddr   Interrupt Mask Register Address.
* \param   intrMaskBitPos    Interrupt Mask Register bit position.
*/
typedef struct Pmic_IntrCfg_s
{
    uint16_t intrClrRegAddr;
    uint8_t  intrClrBitPos;
    uint8_t  intrMaskRegAddr;
    uint8_t  intrMaskBitPos;
} Pmic_IntrCfg_t;

/*!
* \brief   PMIC GPIO Interrupt Mask details object structure.
*
* \param   gpioRiseIntrMaskRegAddr    GPIO RISE Interrupt Mask Register Address
* \param   gpioRiseMaskBitPos         GPIO RISE Interrupt Mask Register Bit
*                                     position.
* \param   gpioFallIntrMaskRegAddr    GPIO FALL Interrupt Mask Register Address
* \param   gpioFallMaskBitPos         GPIO FALL Interrupt Mask Register Bit
*                                     position.
*/
typedef struct Pmic_GpioIntrTypeCfg_s
{
    uint8_t gpioRiseIntrMaskRegAddr;
    uint8_t gpioRiseMaskBitPos;
    uint8_t gpioFallIntrMaskRegAddr;
    uint8_t gpioFallMaskBitPos;
} Pmic_GpioIntrTypeCfg_t;

/*==========================================================================*/
/*                         Function Declarations                            */
/*==========================================================================*/
/*!
 * \brief  Function to Set the intStatus bit position.
 */
void Pmic_intrBitSet(Pmic_IrqStatus_t  *pErrStat, uint32_t pos);

#ifdef __cplusplus
}

#endif /* __cplusplus */

#endif /* PMIC_IRQ_PRIV_H_ */
