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
 *  \ingroup  DRV_PMIC_MODULE
 *  \defgroup DRV_PMIC_IRQ_MODULE PMIC Interrupt Driver API
 *            These are PMIC Interrupt driver parameters and API
 *
 *  @{
 */

/**
 * \file   pmic_irq.h
 *
 * \brief  PMIC IRQ Driver API/interface file.
 */

#ifndef PMIC_IRQ_H_
#define PMIC_IRQ_H_

/* ==========================================================================*/
/*                             Include Files                                 */
/* ==========================================================================*/
#include <pmic_core.h>
#include <pmic_core_priv.h>
#include <pmic_irq_tps6594x_priv.h>
#include <pmic_irq_priv.h>
#include <cfg/tps6594x/pmic_irq_tps6594x.h>
#include <cfg/lp8764x/pmic_irq_lp8764x.h>

#ifdef __cplusplus
extern "C" {
#endif
/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/**
 *  \anchor Pmic_IrqClearFlag
 *  \name PMIC IRQ Clear flag
 *
 *  @{
 */
#define PMIC_IRQ_CLEAR_NONE             (0x0U)
#define PMIC_IRQ_CLEAR                  (0x1U)
/* @} */

/**
 *  \anchor Pmic_IrqMaskFlag
 *  \name PMIC IRQ MAsk flag
 *
 *  @{
 */
#define PMIC_IRQ_MASK             (0x0U)
#define PMIC_IRQ_UNMASK           (0x1U)
/* @} */

/**
 *  \anchor Pmic_IrqID
 *  \name PMIC IRQ ID definesUnique IRQ Code Identifier Parameters
 *
 *         Parameters:
 *         L1 - Level 1 Code in Interrupt Hierarchy
 *         L2 - Level 2 Code in Interrupt Hierarchy
 *         bit - actual error bit mask
 *         Bitfield breakup: Individual bit breakup is for Driver usage only
 *         Bits 0-7   : Interrupt Bit Mask from actual Interrupt register for
 *         the Event/Error
 *         Bits 8-23  : Interrupt Hierarchy Level 2 Identifier,
 *                      Use 0x0000U if not applicable
 *         Bits 24-31 : Interrupt Hierarchy Level 1 Identifier
 *
 *  @{
 */
#define PMIC_IRQID(L1, L2, bit)            ((uint32_t)((L1 << 24U) | \
                                            (L2 << 8U) | bit))
#define PMIC_IRQID_L1REG(errcode)          ((errcode >> 24U) & 0xFFU)
#define PMIC_IRQID_L2REG(errcode)          ((errcode >> 8U) & 0xFFFFU)
#define PMIC_IRQID_BITMASK(errcode)        ((errcode >> 0U) & 0xFFU)
/* @} */

  /**
 *  \anchor Pmic_InterruptId
 *  \name PMIC Interrupt ID
 *
 *  @{
 */
#define PMIC_INT_ID_BUCK1_OV           PMIC_IRQID(                            \
                                            PMIC_INT_BUCK_REGADDR,    \
                                            PMIC_INT_BUCK1_2_REGADDR, \
                                            PMIC_INT_BUCK1_2_BUCK1_OV_INT_MASK)
#define PMIC_INT_ID_BUCK1_UV           PMIC_IRQID(                            \
                                            PMIC_INT_BUCK_REGADDR,    \
                                            PMIC_INT_BUCK1_2_REGADDR, \
                                            PMIC_INT_BUCK1_2_BUCK1_UV_INT_MASK)
#define PMIC_INT_ID_BUCK1_SC           PMIC_IRQID(                            \
                                            PMIC_INT_BUCK_REGADDR,    \
                                            PMIC_INT_BUCK1_2_REGADDR, \
                                            PMIC_INT_BUCK1_2_BUCK1_SC_INT_MASK)
#define PMIC_INT_ID_BUCK1_ILIM         PMIC_IRQID(                            \
                                          PMIC_INT_BUCK_REGADDR,    \
                                          PMIC_INT_BUCK1_2_REGADDR, \
                                          PMIC_INT_BUCK1_2_BUCK1_ILIM_INT_MASK)
#define PMIC_INT_ID_BUCK2_OV           PMIC_IRQID(                            \
                                            PMIC_INT_BUCK_REGADDR,    \
                                            PMIC_INT_BUCK1_2_REGADDR, \
                                            PMIC_INT_BUCK1_2_BUCK2_OV_INT_MASK)
#define PMIC_INT_ID_BUCK2_UV           PMIC_IRQID(                            \
                                            PMIC_INT_BUCK_REGADDR,    \
                                            PMIC_INT_BUCK1_2_REGADDR, \
                                            PMIC_INT_BUCK1_2_BUCK2_UV_INT_MASK)
#define PMIC_INT_ID_BUCK2_SC           PMIC_IRQID(                            \
                                            PMIC_INT_BUCK_REGADDR,    \
                                            PMIC_INT_BUCK1_2_REGADDR, \
                                            PMIC_INT_BUCK1_2_BUCK2_SC_INT_MASK)
#define PMIC_INT_ID_BUCK2_ILIM         PMIC_IRQID(                            \
                                         PMIC_INT_BUCK_REGADDR,    \
                                         PMIC_INT_BUCK1_2_REGADDR, \
                                         PMIC_INT_BUCK1_2_BUCK2_ILIM_INT_MASK)
#define PMIC_INT_ID_BUCK3_OV           PMIC_IRQID(                            \
                                            PMIC_INT_BUCK_REGADDR,    \
                                            PMIC_INT_BUCK3_4_REGADDR, \
                                            PMIC_INT_BUCK3_4_BUCK3_OV_INT_MASK)
#define PMIC_INT_ID_BUCK3_UV           PMIC_IRQID(                            \
                                            PMIC_INT_BUCK_REGADDR,    \
                                            PMIC_INT_BUCK3_4_REGADDR, \
                                            PMIC_INT_BUCK3_4_BUCK3_UV_INT_MASK)
#define PMIC_INT_ID_BUCK3_SC           PMIC_IRQID(                            \
                                            PMIC_INT_BUCK_REGADDR,    \
                                            PMIC_INT_BUCK3_4_REGADDR, \
                                            PMIC_INT_BUCK3_4_BUCK3_SC_INT_MASK)
#define PMIC_INT_ID_BUCK3_ILIM         PMIC_IRQID(                            \
                                          PMIC_INT_BUCK_REGADDR,    \
                                          PMIC_INT_BUCK3_4_REGADDR, \
                                          PMIC_INT_BUCK3_4_BUCK3_ILIM_INT_MASK)
#define PMIC_INT_ID_BUCK4_OV           PMIC_IRQID(                            \
                                            PMIC_INT_BUCK_REGADDR,    \
                                            PMIC_INT_BUCK3_4_REGADDR, \
                                            PMIC_INT_BUCK3_4_BUCK4_OV_INT_MASK)
#define PMIC_INT_ID_BUCK4_UV           PMIC_IRQID(                            \
                                            PMIC_INT_BUCK_REGADDR,    \
                                            PMIC_INT_BUCK3_4_REGADDR, \
                                            PMIC_INT_BUCK3_4_BUCK4_UV_INT_MASK)
#define PMIC_INT_ID_BUCK4_SC           PMIC_IRQID(                            \
                                            PMIC_INT_BUCK_REGADDR,    \
                                            PMIC_INT_BUCK3_4_REGADDR, \
                                            PMIC_INT_BUCK3_4_BUCK4_SC_INT_MASK)
#define PMIC_INT_ID_BUCK4_ILIM         PMIC_IRQID(                            \
                                          PMIC_INT_BUCK_REGADDR,    \
                                          PMIC_INT_BUCK3_4_REGADDR, \
                                          PMIC_INT_BUCK3_4_BUCK4_ILIM_INT_MASK)
#define PMIC_INT_ID_BUCK5_OV           PMIC_IRQID(                            \
                                              PMIC_INT_BUCK_REGADDR,    \
                                              PMIC_INT_BUCK5_REGADDR,   \
                                              PMIC_INT_BUCK5_BUCK5_OV_INT_MASK)
#define PMIC_INT_ID_BUCK5_UV           PMIC_IRQID(                            \
                                              PMIC_INT_BUCK_REGADDR,    \
                                              PMIC_INT_BUCK5_REGADDR,   \
                                              PMIC_INT_BUCK5_BUCK5_UV_INT_MASK)
#define PMIC_INT_ID_BUCK5_SC           PMIC_IRQID(                            \
                                              PMIC_INT_BUCK_REGADDR,    \
                                              PMIC_INT_BUCK5_REGADDR,   \
                                              PMIC_INT_BUCK5_BUCK5_SC_INT_MASK)
#define PMIC_INT_ID_BUCK5_ILIM         PMIC_IRQID(                            \
                                            PMIC_INT_BUCK_REGADDR,    \
                                            PMIC_INT_BUCK5_REGADDR,   \
                                            PMIC_INT_BUCK5_BUCK5_ILIM_INT_MASK)

#define PMIC_INT_ID_LDO1_OV            PMIC_IRQID(                            \
                                             PMIC_INT_LDO_VMON_REGADDR,\
                                             PMIC_INT_LDO1_2_REGADDR,  \
                                             PMIC_INT_LDO1_2_LDO1_OV_INT_MASK)
#define PMIC_INT_ID_LDO1_UV            PMIC_IRQID(                            \
                                             PMIC_INT_LDO_VMON_REGADDR,\
                                             PMIC_INT_LDO1_2_REGADDR,  \
                                             PMIC_INT_LDO1_2_LDO1_UV_INT_MASK)
#define PMIC_INT_ID_LDO1_SC            PMIC_IRQID(                            \
                                             PMIC_INT_LDO_VMON_REGADDR,\
                                             PMIC_INT_LDO1_2_REGADDR,  \
                                             PMIC_INT_LDO1_2_LDO1_SC_INT_MASK)
#define PMIC_INT_ID_LDO1_ILIM          PMIC_IRQID(                            \
                                            PMIC_INT_LDO_VMON_REGADDR,\
                                            PMIC_INT_LDO1_2_REGADDR,  \
                                            PMIC_INT_LDO1_2_LDO1_ILIM_INT_MASK)
#define PMIC_INT_ID_LDO2_OV            PMIC_IRQID(                            \
                                             PMIC_INT_LDO_VMON_REGADDR,\
                                             PMIC_INT_LDO1_2_REGADDR,  \
                                             PMIC_INT_LDO1_2_LDO2_OV_INT_MASK)
#define PMIC_INT_ID_LDO2_UV            PMIC_IRQID(                            \
                                             PMIC_INT_LDO_VMON_REGADDR,\
                                             PMIC_INT_LDO1_2_REGADDR,  \
                                             PMIC_INT_LDO1_2_LDO2_UV_INT_MASK)
#define PMIC_INT_ID_LDO2_SC            PMIC_IRQID(                            \
                                             PMIC_INT_LDO_VMON_REGADDR,\
                                             PMIC_INT_LDO1_2_REGADDR,  \
                                             PMIC_INT_LDO1_2_LDO2_SC_INT_MASK)
#define PMIC_INT_ID_LDO2_ILIM          PMIC_IRQID(                            \
                                            PMIC_INT_LDO_VMON_REGADDR,\
                                            PMIC_INT_LDO1_2_REGADDR,  \
                                            PMIC_INT_LDO1_2_LDO2_ILIM_INT_MASK)
#define PMIC_INT_ID_LDO3_OV            PMIC_IRQID(                            \
                                             PMIC_INT_LDO_VMON_REGADDR,\
                                             PMIC_INT_LDO3_4_REGADDR,  \
                                             PMIC_INT_LDO3_4_LDO3_OV_INT_MASK)
#define PMIC_INT_ID_LDO3_UV            PMIC_IRQID(                            \
                                             PMIC_INT_LDO_VMON_REGADDR,\
                                             PMIC_INT_LDO3_4_REGADDR,  \
                                             PMIC_INT_LDO3_4_LDO3_UV_INT_MASK)
#define PMIC_INT_ID_LDO3_SC            PMIC_IRQID(                            \
                                             PMIC_INT_LDO_VMON_REGADDR,\
                                             PMIC_INT_LDO3_4_REGADDR,  \
                                             PMIC_INT_LDO3_4_LDO3_SC_INT_MASK)
#define PMIC_INT_ID_LDO3_ILIM          PMIC_IRQID(                            \
                                            PMIC_INT_LDO_VMON_REGADDR,\
                                            PMIC_INT_LDO3_4_REGADDR,  \
                                            PMIC_INT_LDO3_4_LDO3_ILIM_INT_MASK)
#define PMIC_INT_ID_LDO4_OV            PMIC_IRQID(                            \
                                             PMIC_INT_LDO_VMON_REGADDR,\
                                             PMIC_INT_LDO3_4_REGADDR,  \
                                             PMIC_INT_LDO3_4_LDO4_OV_INT_MASK)
#define PMIC_INT_ID_LDO4_UV            PMIC_IRQID(                            \
                                             PMIC_INT_LDO_VMON_REGADDR,\
                                             PMIC_INT_LDO3_4_REGADDR,  \
                                             PMIC_INT_LDO3_4_LDO4_UV_INT_MASK)
#define PMIC_INT_ID_LDO4_SC            PMIC_IRQID(                            \
                                             PMIC_INT_LDO_VMON_REGADDR,\
                                             PMIC_INT_LDO3_4_REGADDR,  \
                                             PMIC_INT_LDO3_4_LDO4_SC_INT_MASK)
#define PMIC_INT_ID_LDO4_ILIM          PMIC_IRQID(                            \
                                            PMIC_INT_LDO_VMON_REGADDR,\
                                            PMIC_INT_LDO3_4_REGADDR,  \
                                            PMIC_INT_LDO3_4_LDO4_ILIM_INT_MASK)
#define PMIC_INT_ID_GPIO1              PMIC_IRQID(                            \
                                             PMIC_INT_GPIO_REGADDR,    \
                                             PMIC_INT_GPIO1_8_REGADDR, \
                                             PMIC_INT_GPIO1_8_GPIO1_INT_MASK)
#define PMIC_INT_ID_GPIO2              PMIC_IRQID(                            \
                                             PMIC_INT_GPIO_REGADDR,    \
                                             PMIC_INT_GPIO1_8_REGADDR, \
                                             PMIC_INT_GPIO1_8_GPIO2_INT_MASK)
#define PMIC_INT_ID_GPIO3              PMIC_IRQID(                            \
                                             PMIC_INT_GPIO_REGADDR,    \
                                             PMIC_INT_GPIO1_8_REGADDR, \
                                             PMIC_INT_GPIO1_8_GPIO3_INT_MASK)
#define PMIC_INT_ID_GPIO4              PMIC_IRQID(                            \
                                             PMIC_INT_GPIO_REGADDR,    \
                                             PMIC_INT_GPIO1_8_REGADDR, \
                                             PMIC_INT_GPIO1_8_GPIO4_INT_MASK)
#define PMIC_INT_ID_GPIO5              PMIC_IRQID(                            \
                                             PMIC_INT_GPIO_REGADDR,    \
                                             PMIC_INT_GPIO1_8_REGADDR, \
                                             PMIC_INT_GPIO1_8_GPIO5_INT_MASK)
#define PMIC_INT_ID_GPIO6              PMIC_IRQID(                            \
                                             PMIC_INT_GPIO_REGADDR,    \
                                             PMIC_INT_GPIO1_8_REGADDR, \
                                             PMIC_INT_GPIO1_8_GPIO6_INT_MASK)
#define PMIC_INT_ID_GPIO7              PMIC_IRQID(                            \
                                             PMIC_INT_GPIO_REGADDR,    \
                                             PMIC_INT_GPIO1_8_REGADDR, \
                                             PMIC_INT_GPIO1_8_GPIO7_INT_MASK)
#define PMIC_INT_ID_GPIO8              PMIC_IRQID(                            \
                                             PMIC_INT_GPIO_REGADDR,    \
                                             PMIC_INT_GPIO1_8_REGADDR, \
                                             PMIC_INT_GPIO1_8_GPIO8_INT_MASK)
#define PMIC_INT_ID_GPIO9              PMIC_IRQID(                            \
                                                 PMIC_INT_GPIO_REGADDR,    \
                                                 0x0000U,  \
                                                 PMIC_INT_GPIO_GPIO9_INT_MASK)
#define PMIC_INT_ID_GPIO10             PMIC_IRQID(                            \
                                                 PMIC_INT_GPIO_REGADDR,    \
                                                 0x0000U,  \
                                                 PMIC_INT_GPIO_GPIO10_INT_MASK)
#define PMIC_INT_ID_GPIO11             PMIC_IRQID(                            \
                                                 PMIC_INT_GPIO_REGADDR,    \
                                                 0x0000U,  \
                                                 PMIC_INT_GPIO_GPIO11_INT_MASK)

#define PMIC_INT_ID_NPWRON_START       PMIC_IRQID(                            \
                                                 PMIC_INT_STARTUP_REGADDR, \
                                                 0x0000U,  \
                                                 PMIC_INT_STARTUP_NPWRON_START_INT_MASK)
#define PMIC_INT_ID_ENABLE             PMIC_IRQID(                            \
                                             PMIC_INT_STARTUP_REGADDR, \
                                             0x0000U,  \
                                             PMIC_INT_STARTUP_ENABLE_INT_MASK)
#define PMIC_INT_ID_RTC_TIMER          PMIC_IRQID(                            \
                                              PMIC_INT_STARTUP_REGADDR,    \
                                              PMIC_INT_RTC_STATUS_REGADDR, \
                                              PMIC_RTC_STATUS_TIMER_MASK)
#define PMIC_INT_ID_RTC_ALARM          PMIC_IRQID(                            \
                                              PMIC_INT_STARTUP_REGADDR,    \
                                              PMIC_INT_RTC_STATUS_REGADDR, \
                                              PMIC_RTC_STATUS_ALARM_MASK)
#define PMIC_INT_ID_FSD                PMIC_IRQID(                            \
                                              PMIC_INT_STARTUP_REGADDR,    \
                                              0x0000U,     \
                                              PMIC_INT_STARTUP_FSD_INT_MASK)

#define PMIC_INT_ID_BIST_PASS          PMIC_IRQID(                            \
                                              PMIC_INT_MISC_REGADDR,  \
                                              0x0000U,     \
                                              PMIC_INT_MISC_BIST_PASS_INT_MASK)
#define PMIC_INT_ID_EXT_CLK            PMIC_IRQID(                            \
                                              PMIC_INT_MISC_REGADDR,  \
                                              0x0000U,     \
                                              PMIC_INT_MISC_EXT_CLK_INT_MASK)
#define PMIC_INT_ID_TWARN              PMIC_IRQID(                            \
                                              PMIC_INT_MISC_REGADDR,  \
                                              0x0000U,     \
                                              PMIC_INT_MISC_TWARN_INT_MASK)

#define PMIC_INT_ID_TSD_ORD            PMIC_IRQID(                            \
                                        PMIC_INT_MODERATE_ERR_REGADDR, \
                                        0x0000U,     \
                                        PMIC_INT_MODERATE_ERR_TSD_ORD_INT_MASK)
#define PMIC_INT_ID_BIST_FAIL          PMIC_IRQID(                            \
                                      PMIC_INT_MODERATE_ERR_REGADDR, \
                                      0x0000U,     \
                                      PMIC_INT_MODERATE_ERR_BIST_FAIL_INT_MASK)
#define PMIC_INT_ID_REG_CRC_ERR        PMIC_IRQID(                            \
                                    PMIC_INT_MODERATE_ERR_REGADDR, \
                                    0x0000U,     \
                                    PMIC_INT_MODERATE_ERR_REG_CRC_ERR_INT_MASK)
#define PMIC_INT_ID_RECOV_CNT          PMIC_IRQID(                            \
                                      PMIC_INT_MODERATE_ERR_REGADDR, \
                                      0x0000U,     \
                                      PMIC_INT_MODERATE_ERR_RECOV_CNT_INT_MASK)
#define PMIC_INT_ID_SPMI_ERR           PMIC_IRQID(                            \
                                      PMIC_INT_MODERATE_ERR_REGADDR, \
                                      0x0000U,     \
                                      PMIC_INT_MODERATE_ERR_SPMI_ERR_INT_MASK)
#define PMIC_INT_ID_NPWRON_LONG        PMIC_IRQID(                            \
                                    PMIC_INT_MODERATE_ERR_REGADDR, \
                                    0x0000U,     \
                                    PMIC_INT_MODERATE_ERR_NPWRON_LONG_INT_MASK)
#define PMIC_INT_ID_NINT_RDBK          PMIC_IRQID(                            \
                                  PMIC_INT_MODERATE_ERR_REGADDR, \
                                  0x0000U,     \
                                  PMIC_INT_MODERATE_ERR_NINT_READBACK_INT_MASK)
#define PMIC_INT_ID_NRSTOUT_RDBK       PMIC_IRQID(                            \
                              PMIC_INT_MODERATE_ERR_REGADDR, \
                              0x0000U,     \
                              PMIC_INT_MODERATE_ERR_NRSTOUT_READBACK_INT_MASK)

#define PMIC_INT_ID_TSD_IMM            PMIC_IRQID(                            \
                                          PMIC_INT_SEVERE_ERR_REGADDR,   \
                                          0x0000U,     \
                                          PMIC_INT_SEVERE_ERR_TSD_IMM_INT_MASK)
#define PMIC_INT_ID_VCCA_OVP           PMIC_IRQID(                            \
                                          PMIC_INT_SEVERE_ERR_REGADDR,   \
                                          0x0000U,     \
                                          PMIC_INT_SEVERE_ERR_VCCA_OVP_INT_MASK)
#define PMIC_INT_ID_PFSM_ERR           PMIC_IRQID(                            \
                                          PMIC_INT_SEVERE_ERR_REGADDR,   \
                                          0x0000U,     \
                                          PMIC_INT_SEVERE_ERR_PFSM_ERR_INT_MASK)

#define PMIC_INT_ID_IMM_SHUTDOWN       PMIC_IRQID(                            \
                                      PMIC_INT_FSM_ERR_REGADDR,       \
                                      0x0000U,     \
                                      PMIC_INT_FSM_ERR_IMM_SHUTDOWN_INT_MASK)
#define PMIC_INT_ID_ORD_SHUTDOWN       PMIC_IRQID(                            \
                                      PMIC_INT_FSM_ERR_REGADDR,       \
                                      0x0000U,     \
                                      PMIC_INT_FSM_ERR_ORD_SHUTDOWN_INT_MASK)
#define PMIC_INT_ID_MCU_PWR_ERR        PMIC_IRQID(                            \
                                         PMIC_INT_FSM_ERR_REGADDR,       \
                                         0x0000U,     \
                                         PMIC_INT_FSM_ERR_MCU_PWR_ERR_INT_MASK)
#define PMIC_INT_ID_SOC_PWR_ERR        PMIC_IRQID(                            \
                                         PMIC_INT_FSM_ERR_REGADDR,       \
                                         0x0000U,     \
                                         PMIC_INT_FSM_ERR_SOC_PWR_ERR_INT_MASK)
#define PMIC_INT_ID_COMM_FRM_ERR       PMIC_IRQID(                            \
                                      PMIC_INT_FSM_ERR_REGADDR,       \
                                      PMIC_INT_COMM_ERR_REGADDR,     \
                                      PMIC_INT_COMM_ERR_COMM_FRM_ERR_INT_MASK)
#define PMIC_INT_ID_COMM_CRC_ERR       PMIC_IRQID(                            \
                                      PMIC_INT_FSM_ERR_REGADDR,       \
                                      PMIC_INT_COMM_ERR_REGADDR,     \
                                      PMIC_INT_COMM_ERR_COMM_CRC_ERR_INT_MASK)
#define PMIC_INT_ID_COMM_ADR_ERR       PMIC_IRQID(                            \
                                      PMIC_INT_FSM_ERR_REGADDR,       \
                                      PMIC_INT_COMM_ERR_REGADDR,     \
                                      PMIC_INT_COMM_ERR_COMM_ADR_ERR_INT_MASK)
#define PMIC_INT_ID_I2C2_CRC_ERR       PMIC_IRQID(                            \
                                      PMIC_INT_FSM_ERR_REGADDR,       \
                                      PMIC_INT_COMM_ERR_REGADDR,     \
                                      PMIC_INT_COMM_ERR_I2C2_CRC_ERR_INT_MASK)
#define PMIC_INT_ID_I2C2_ADR_ERR       PMIC_IRQID(                            \
                                      PMIC_INT_FSM_ERR_REGADDR,       \
                                      PMIC_INT_COMM_ERR_REGADDR,     \
                                      PMIC_INT_COMM_ERR_I2C2_ADR_ERR_INT_MASK)
#define PMIC_INT_ID_EN_DRV_RDBACK      PMIC_IRQID(                            \
                                PMIC_INT_FSM_ERR_REGADDR,         \
                                PMIC_INT_RDBACK_ERR_REGADDR,     \
                                PMIC_INT_READBACK_ERR_EN_DRV_READBACK_INT_MASK)
#define PMIC_INT_ID_NRSTOUT_SOC_RDBACK   PMIC_IRQID(                          \
                          PMIC_INT_FSM_ERR_REGADDR,         \
                          PMIC_INT_RDBACK_ERR_REGADDR,     \
                          PMIC_INT_READBACK_ERR_NRSTOUT_SOC_READBACK_INT_MASK)

#define PMIC_INT_ID_ESM_SOC_PIN        PMIC_IRQID(                            \
                                             PMIC_INT_FSM_ERR_REGADDR,    \
                                             PMIC_INT_ESM_REGADDR,       \
                                             PMIC_INT_ESM_ESM_SOC_PIN_INT_MASK)
#define PMIC_INT_ID_ESM_SOC_FAIL       PMIC_IRQID(                            \
                                            PMIC_INT_FSM_ERR_REGADDR,    \
                                            PMIC_INT_ESM_REGADDR,       \
                                            PMIC_INT_ESM_ESM_SOC_FAIL_INT_MASK)
#define PMIC_INT_ID_ESM_SOC_RST        PMIC_IRQID(                            \
                                             PMIC_INT_FSM_ERR_REGADDR,    \
                                             PMIC_INT_ESM_REGADDR,       \
                                             PMIC_INT_ESM_ESM_SOC_RST_INT_MASK)
#define PMIC_INT_ID_ESM_MCU_PIN        PMIC_IRQID(                            \
                                             PMIC_INT_FSM_ERR_REGADDR,    \
                                             PMIC_INT_ESM_REGADDR,       \
                                             PMIC_INT_ESM_ESM_MCU_PIN_INT_MASK)
#define PMIC_INT_ID_ESM_MCU_FAIL       PMIC_IRQID(                            \
                                            PMIC_INT_FSM_ERR_REGADDR,    \
                                            PMIC_INT_ESM_REGADDR,       \
                                            PMIC_INT_ESM_ESM_MCU_FAIL_INT_MASK)
#define PMIC_INT_ID_ESM_MCU_RST        PMIC_IRQID(                            \
                                             PMIC_INT_FSM_ERR_REGADDR,    \
                                             PMIC_INT_ESM_REGADDR,       \
                                             PMIC_INT_ESM_ESM_MCU_RST_INT_MASK)

#define PMIC_INT_ID_WDG_LONGWIN_TIMEOUT  PMIC_IRQID(                          \
                                PMIC_INT_FSM_ERR_REGADDR,           \
                                PMIC_WD_ERR_STATUS_REGADDR,    \
                                PMIC_WD_ERR_STATUS_WD_LONGWIN_TIMEOUT_INT_MASK)
#define PMIC_INT_ID_WDG_TIMEOUT        PMIC_IRQID(                            \
                                          PMIC_INT_FSM_ERR_REGADDR,           \
                                          PMIC_WD_ERR_STATUS_REGADDR,    \
                                          PMIC_WD_ERR_STATUS_WD_TIMEOUT_MASK)
#define PMIC_INT_ID_WDG_TRIG_EARLY     PMIC_IRQID(                            \
                                         PMIC_INT_FSM_ERR_REGADDR,           \
                                         PMIC_WD_ERR_STATUS_REGADDR,    \
                                         PMIC_WD_ERR_STATUS_WD_TRIG_EARLY_MASK)
#define PMIC_INT_ID_WDG_ANSW_EARLY     PMIC_IRQID(                            \
                                         PMIC_INT_FSM_ERR_REGADDR,           \
                                         PMIC_WD_ERR_STATUS_REGADDR,    \
                                         PMIC_WD_ERR_STATUS_WD_ANSW_EARLY_MASK)
#define PMIC_INT_ID_WDG_SEQ_ERR        PMIC_IRQID(                            \
                                          PMIC_INT_FSM_ERR_REGADDR,           \
                                          PMIC_WD_ERR_STATUS_REGADDR,    \
                                          PMIC_WD_ERR_STATUS_WD_SEQ_ERR_MASK)
#define PMIC_INT_ID_WDG_ANSW_ERR       PMIC_IRQID(                            \
                                          PMIC_INT_FSM_ERR_REGADDR,           \
                                          PMIC_WD_ERR_STATUS_REGADDR,    \
                                          PMIC_WD_ERR_STATUS_WD_ANSW_ERR_MASK)
#define PMIC_INT_ID_WDG_FAIL           PMIC_IRQID(                            \
                                          PMIC_INT_FSM_ERR_REGADDR,           \
                                          PMIC_WD_ERR_STATUS_REGADDR,    \
                                          PMIC_WD_ERR_STATUS_WD_FAIL_INT_MASK)
#define PMIC_INT_ID_WDG_RST            PMIC_IRQID(                            \
                                          PMIC_INT_FSM_ERR_REGADDR,           \
                                          PMIC_WD_ERR_STATUS_REGADDR,    \
                                          PMIC_WD_ERR_STATUS_WD_RST_INT_MASK)
/* @} */


/**
 *  \anchor Pmic_IrqInterruptMask
 *  \name   PMIC Interrupts Masks
 *
 *  Application can use below shifted values to mask/unmask an interrupt
 *  Bitfield breakup:
 *         Bits 0-7   : Interrupt Mask register address.
 *         Bits 8-15  : Actual interrupt Mask.
 *
 *  @{
 */
#define PMIC_IRQ_MASK_BUCK1_2_MASK   ((uint16_t)PMIC_MASK_BUCK1_2_MASK  << 8U) \
                                              | PMIC_MASK_BUCK1_2_REGADDR
#define PMIC_IRQ_BUCK2_ILIM_MASK     ((uint16_t)PMIC_MASK_BUCK1_2_BUCK2_ILIM_MASK_MASK    << 8U) \
                                              | PMIC_MASK_BUCK1_2_REGADDR
#define PMIC_IRQ_BUCK2_UV_MASK       ((uint16_t)PMIC_MASK_BUCK1_2_BUCK2_UV_MASK_MASK      << 8U) \
                                              | PMIC_MASK_BUCK1_2_REGADDR
#define PMIC_IRQ_BUCK2_OV_MASK       ((uint16_t)PMIC_MASK_BUCK1_2_BUCK2_OV_MASK_MASK      << 8U) \
                                              | PMIC_MASK_BUCK1_2_REGADDR
#define PMIC_IRQ_BUCK1_ILIM_MASK     ((uint16_t)PMIC_MASK_BUCK1_2_BUCK1_ILIM_MASK_MASK    << 8U) \
                                              | PMIC_MASK_BUCK1_2_REGADDR
#define PMIC_IRQ_BUCK1_UV_MASK       ((uint16_t)PMIC_MASK_BUCK1_2_BUCK1_UV_MASK_MASK      << 8U) \
                                              | PMIC_MASK_BUCK1_2_REGADDR
#define PMIC_IRQ_BUCK1_OV_MASK       ((uint16_t)PMIC_MASK_BUCK1_2_BUCK1_OV_MASK_MASK      << 8U) \
                                              | PMIC_MASK_BUCK1_2_REGADDR


#define PMIC_IRQ_MASK_BUCK3_4_MASK  ((uint16_t)PMIC_MASK_BUCK3_4_MASK   << 8U) \
                                             | PMIC_MASK_BUCK3_4_REGADDR
#define PMIC_IRQ_BUCK4_ILIM_MASK    ((uint16_t)PMIC_MASK_BUCK3_4_BUCK4_ILIM_MASK_MASK     << 8U) \
                                             | PMIC_MASK_BUCK3_4_REGADDR
#define PMIC_IRQ_BUCK4_UV_MASK      ((uint16_t)PMIC_MASK_BUCK3_4_BUCK4_UV_MASK_MASK       << 8U) \
                                             | PMIC_MASK_BUCK3_4_REGADDR
#define PMIC_IRQ_BUCK4_OV_MASK      ((uint16_t)PMIC_MASK_BUCK3_4_BUCK4_OV_MASK_MASK       << 8U) \
                                             | PMIC_MASK_BUCK3_4_REGADDR
#define PMIC_IRQ_BUCK3_ILIM_MASK    ((uint16_t)PMIC_MASK_BUCK3_4_BUCK3_ILIM_MASK_MASK     << 8U) \
                                             | PMIC_MASK_BUCK3_4_REGADDR
#define PMIC_IRQ_BUCK3_UV_MASK      ((uint16_t)PMIC_MASK_BUCK3_4_BUCK3_UV_MASK_MASK       << 8U) \
                                             | PMIC_MASK_BUCK3_4_REGADDR
#define PMIC_IRQ_BUCK3_OV_MASK      ((uint16_t)PMIC_MASK_BUCK3_4_BUCK3_OV_MASK_MASK       << 8U) \
                                             | PMIC_MASK_BUCK3_4_REGADDR


#define PMIC_IRQ_MASK_BUCK5_MASK      ((uint16_t)PMIC_MASK_BUCK5_MASK   << 8U) \
                                             | PMIC_MASK_BUCK5_REGADDR
#define PMIC_IRQ_BUCK5_ILIM_MASK      ((uint16_t)PMIC_MASK_BUCK5_BUCK5_ILIM_MASK_MASK   << 8U) \
                                             | PMIC_MASK_BUCK5_REGADDR
#define PMIC_IRQ_BUCK5_UV_MASK        ((uint16_t)PMIC_MASK_BUCK5_BUCK5_UV_MASK_MASK     << 8U) \
                                             | PMIC_MASK_BUCK5_REGADDR
#define PMIC_IRQ_BUCK5_OV_MASK        ((uint16_t)PMIC_MASK_BUCK5_BUCK5_OV_MASK_MASK     << 8U) \
                                             | PMIC_MASK_BUCK5_REGADDR


#define PMIC_IRQ_MASK_LDO1_2_MASK    ((uint16_t)PMIC_MASK_LDO1_2_MASK   << 8U) \
                                              | PMIC_MASK_LDO1_2_REGADDR
#define PMIC_IRQ_LDO2_ILIM_MASK      ((uint16_t)PMIC_MASK_LDO1_2_LDO2_ILIM_MASK_MASK     << 8U) \
                                              | PMIC_MASK_LDO1_2_REGADDR
#define PMIC_IRQ_LDO2_UV_MASK        ((uint16_t)PMIC_MASK_LDO1_2_LDO2_UV_MASK_MASK       << 8U) \
                                              | PMIC_MASK_LDO1_2_REGADDR
#define PMIC_IRQ_LDO2_OV_MASK        ((uint16_t)PMIC_MASK_LDO1_2_LDO2_OV_MASK_MASK       << 8U) \
                                              | PMIC_MASK_LDO1_2_REGADDR
#define PMIC_IRQ_LDO1_ILIM_MASK      ((uint16_t)PMIC_MASK_LDO1_2_LDO1_ILIM_MASK_MASK     << 8U) \
                                              | PMIC_MASK_LDO1_2_REGADDR
#define PMIC_IRQ_LDO1_UV_MASK        ((uint16_t)PMIC_MASK_LDO1_2_LDO1_UV_MASK_MASK       << 8U) \
                                              | PMIC_MASK_LDO1_2_REGADDR
#define PMIC_IRQ_LDO1_OV_MASK        ((uint16_t)PMIC_MASK_LDO1_2_LDO1_OV_MASK_MASK       << 8U) \
                                              | PMIC_MASK_LDO1_2_REGADDR


#define PMIC_IRQ_MASK_LDO3_4_MASK   ((uint16_t)PMIC_MASK_LDO3_4_MASK    << 8U) \
                                             | PMIC_MASK_LDO3_4_REGADDR
#define PMIC_IRQ_LDO4_ILIM_MASK     ((uint16_t)PMIC_MASK_LDO3_4_LDO4_ILIM_MASK_MASK      << 8U) \
                                             | PMIC_MASK_LDO3_4_REGADDR
#define PMIC_IRQ_LDO4_UV_MASK       ((uint16_t)PMIC_MASK_LDO3_4_LDO4_UV_MASK_MASK        << 8U) \
                                             | PMIC_MASK_LDO3_4_REGADDR
#define PMIC_IRQ_LDO4_OV_MASK       ((uint16_t)PMIC_MASK_LDO3_4_LDO4_OV_MASK_MASK        << 8U) \
                                             | PMIC_MASK_LDO3_4_REGADDR
#define PMIC_IRQ_LDO3_ILIM_MASK     ((uint16_t)PMIC_MASK_LDO3_4_LDO3_ILIM_MASK_MASK      << 8U) \
                                             | PMIC_MASK_LDO3_4_REGADDR
#define PMIC_IRQ_LDO3_UV_MASK       ((uint16_t)PMIC_MASK_LDO3_4_LDO3_UV_MASK_MASK        << 8U) \
                                             | PMIC_MASK_LDO3_4_REGADDR
#define PMIC_IRQ_LDO3_OV_MASK       ((uint16_t)PMIC_MASK_LDO3_4_LDO3_OV_MASK_MASK        << 8U) \
                                             | PMIC_MASK_LDO3_4_REGADDR


#define PMIC_IRQ_MASK_VMON_MASK       ((uint16_t)PMIC_MASK_VMON_MASK    << 8U) \
                                             | PMIC_MASK_VMON_REGADDR
#define PMIC_IRQ_VCCA_UV_MASK         ((uint16_t)PMIC_MASK_VMON_VCCA_UV_MASK_MASK      << 8U) \
                                             | PMIC_MASK_VMON_REGADDR
#define PMIC_IRQ_VCCA_OV_MASK         ((uint16_t)PMIC_MASK_VMON_VCCA_OV_MASK_MASK      << 8U) \
                                             | PMIC_MASK_VMON_REGADDR


#define PMIC_IRQ_MASK_GPIO1_8_FALL_MASK ((uint16_t)PMIC_MASK_GPIO1_8_FALL_MASK \
                                          << 8U)  |                           \
                                           PMIC_MASK_GPIO1_8_FALL_REGADDR
#define PMIC_IRQ_GPIO8_FALL_MASK   ((uint16_t)PMIC_IRQ_MASK_GPIO1_8_FALL_GPIO8_FALL_MASK_MASK      << 8U) \
                                          | PMIC_MASK_GPIO1_8_FALL_REGADDR
#define PMIC_IRQ_GPIO7_FALL_MASK   ((uint16_t)PMIC_IRQ_MASK_GPIO1_8_FALL_GPIO7_FALL_MASK_MASK      << 8U) \
                                          | PMIC_MASK_GPIO1_8_FALL_REGADDR
#define PMIC_IRQ_GPIO6_FALL_MASK   ((uint16_t)PMIC_IRQ_MASK_GPIO1_8_FALL_GPIO6_FALL_MASK_MASK      << 8U) \
                                          | PMIC_MASK_GPIO1_8_FALL_REGADDR
#define PMIC_IRQ_GPIO5_FALL_MASK   ((uint16_t)PMIC_IRQ_MASK_GPIO1_8_FALL_GPIO5_FALL_MASK_MASK      << 8U) \
                                          | PMIC_MASK_GPIO1_8_FALL_REGADDR
#define PMIC_IRQ_GPIO4_FALL_MASK   ((uint16_t)PMIC_IRQ_MASK_GPIO1_8_FALL_GPIO4_FALL_MASK_MASK      << 8U) \
                                          | PMIC_MASK_GPIO1_8_FALL_REGADDR
#define PMIC_IRQ_GPIO3_FALL_MASK   ((uint16_t)PMIC_IRQ_MASK_GPIO1_8_FALL_GPIO3_FALL_MASK_MASK      << 8U) \
                                          | PMIC_MASK_GPIO1_8_FALL_REGADDR
#define PMIC_IRQ_GPIO2_FALL_MASK   ((uint16_t)PMIC_IRQ_MASK_GPIO1_8_FALL_GPIO2_FALL_MASK_MASK      << 8U) \
                                          | PMIC_MASK_GPIO1_8_FALL_REGADDR
#define PMIC_IRQ_GPIO1_FALL_MASK   ((uint16_t)PMIC_IRQ_MASK_GPIO1_8_FALL_GPIO1_FALL_MASK_MASK      << 8U) \
                                          | PMIC_MASK_GPIO1_8_FALL_REGADDR


#define PMIC_IRQ_MASK_GPIO1_8_RISE_MASK  \
                                   ((uint16_t)PMIC_MASK_GPIO1_8_RISE_MASK      \
                                   << 8U) | PMIC_MASK_GPIO1_8_RISE_REGADDR

#define PMIC_IRQ_GPIO8_RISE_MASK  ((uint16_t)PMIC_IRQ_MASK_GPIO1_8_RISE_GPIO8_RISE_MASK_MASK       << 8U) \
                                          | PMIC_MASK_GPIO1_8_RISE_REGADDR
#define PMIC_IRQ_GPIO7_RISE_MASK  ((uint16_t)PMIC_IRQ_MASK_GPIO1_8_RISE_GPIO7_RISE_MASK_MASK       << 8U) \
                                          | PMIC_MASK_GPIO1_8_RISE_REGADDR
#define PMIC_IRQ_GPIO6_RISE_MASK  ((uint16_t)PMIC_IRQ_MASK_GPIO1_8_RISE_GPIO6_RISE_MASK_MASK       << 8U) \
                                          | PMIC_MASK_GPIO1_8_RISE_REGADDR
#define PMIC_IRQ_GPIO5_RISE_MASK  ((uint16_t)PMIC_IRQ_MASK_GPIO1_8_RISE_GPIO5_RISE_MASK_MASK       << 8U) \
                                          | PMIC_MASK_GPIO1_8_RISE_REGADDR
#define PMIC_IRQ_GPIO4_RISE_MASK  ((uint16_t)PMIC_IRQ_MASK_GPIO1_8_RISE_GPIO4_RISE_MASK_MASK       << 8U) \
                                          | PMIC_MASK_GPIO1_8_RISE_REGADDR
#define PMIC_IRQ_GPIO3_RISE_MASK  ((uint16_t)PMIC_IRQ_MASK_GPIO1_8_RISE_GPIO3_RISE_MASK_MASK       << 8U) \
                                          | PMIC_MASK_GPIO1_8_RISE_REGADDR
#define PMIC_IRQ_GPIO2_RISE_MASK  ((uint16_t)PMIC_IRQ_MASK_GPIO1_8_RISE_GPIO2_RISE_MASK_MASK       << 8U) \
                                          | PMIC_MASK_GPIO1_8_RISE_REGADDR
#define PMIC_IRQ_GPIO1_RISE_MASK  ((uint16_t)PMIC_IRQ_MASK_GPIO1_8_RISE_GPIO1_RISE_MASK_MASK       << 8U) \
                                          | PMIC_MASK_GPIO1_8_RISE_REGADDR


#define PMIC_IRQ_MASK_STARTUP_MASK  ((uint16_t)PMIC_MASK_STARTUP_MASK   << 8U) \
                                               | PMIC_MASK_STARTUP_REGADDR
#define PMIC_IRQ_FSD_MASK           ((uint16_t)PMIC_MASK_STARTUP_FSD_MASK_MASK            << 8U) \
                                               | PMIC_MASK_STARTUP_REGADDR
#define PMIC_IRQ_ENABLE_MASK        ((uint16_t)PMIC_MASK_STARTUP_ENABLE_MASK_MASK         << 8U) \
                                               | PMIC_MASK_STARTUP_REGADDR
#define PMIC_IRQ_NPWRON_START_MASK  ((uint16_t)PMIC_MASK_STARTUP_NPWRON_START_MASK_MASK   << 8U) \
                                               | PMIC_MASK_STARTUP_REGADDR


#define PMIC_IRQ_MASK_MISC_MASK       ((uint16_t)PMIC_MASK_MISC_MASK    << 8U) \
                                               | PMIC_MASK_MISC_REGADDR
#define PMIC_IRQ_TWARN_MASK           ((uint16_t)PMIC_MASK_MISC_TWARN_MASK_MASK        << 8U) \
                                               | PMIC_MASK_MISC_REGADDR
#define PMIC_IRQ_EXT_CLK_MASK         ((uint16_t)PMIC_MASK_MISC_EXT_CLK_MASK_MASK      << 8U) \
                                               | PMIC_MASK_MISC_REGADDR
#define PMIC_IRQ_BIST_PASS_MASK       ((uint16_t)PMIC_MASK_MISC_BIST_PASS_MASK_MASK    << 8U) \
                                               | PMIC_MASK_MISC_REGADDR


#define PMIC_IRQ_MASK_MODERATE_ERR_MASK ((uint16_t)PMIC_MASK_MODERATE_ERR_MASK \
                                   << 8U) | PMIC_MASK_MODERATE_ERR_REGADDR
#define PMIC_IRQ_NRSTOUT_READBACK_ MASK ((uint16_t)PMIC_MASK_MODERATE_ERR_NRSTOUT_READBACK_MASK_MASK  \
                                   << 8U) | PMIC_MASK_MODERATE_ERR_REGADDR
#define PMIC_IRQ_NINT_READBACK_MASK      ((uint16_t)PMIC_MASK_MODERATE_ERR_NINT_READBACK_MASK_MASK    \
                                   << 8U) | PMIC_MASK_MODERATE_ERR_REGADDR
#define PMIC_IRQ_NPWRON_LONG_MASK        ((uint16_t)PMIC_MASK_MODERATE_ERR_NPWRON_LONG_MASK_MASK      \
                                   << 8U) | PMIC_MASK_MODERATE_ERR_REGADDR
#define PMIC_IRQ_SPMI_ERR_MASK           ((uint16_t)PMIC_MASK_MODERATE_ERR_SPMI_ERR_MASK_MASK         \
                                   << 8U) | PMIC_MASK_MODERATE_ERR_REGADDR
#define PMIC_IRQ_REG_CRC_ERR_MASK        ((uint16_t)PMIC_MASK_MODERATE_ERR_REG_CRC_ERR_MASK_MASK      \
                                   << 8U) | PMIC_MASK_MODERATE_ERR_REGADDR
#define PMIC_IRQ_BIST_FAIL_MASK          ((uint16_t)PMIC_MASK_MODERATE_ERR_BIST_FAIL_MASK_MASK        \
                                   << 8U) | PMIC_MASK_MODERATE_ERR_REGADDR


#define PMIC_IRQ_MASK_FSM_ERR_MASK   ((uint16_t)PMIC_MASK_FSM_ERR_MASK  << 8U) \
                                               | PMIC_MASK_FSM_ERR_REGADDR
#define PMIC_IRQ_SOC_PWR_ERR_MASK    ((uint16_t)PMIC_MASK_FSM_ERR_SOC_PWR_ERR_MASK_MASK   << 8U) \
                                               | PMIC_MASK_FSM_ERR_REGADDR
#define PMIC_IRQ_MCU_PWR_ERR_MASK    ((uint16_t)PMIC_MASK_FSM_ERR_MCU_PWR_ERR_MASK_MASK   << 8U) \
                                               | PMIC_MASK_FSM_ERR_REGADDR
#define PMIC_IRQ_ORD_SHUTDOWN_MASK   ((uint16_t)PMIC_MASK_FSM_ERR_ORD_SHUTDOWN_MASK_MASK  << 8U) \
                                               | PMIC_MASK_FSM_ERR_REGADDR
#define PMIC_IRQ_IMM_SHUTDOWN_MASK   ((uint16_t)PMIC_MASK_FSM_ERR_IMM_SHUTDOWN_MASK_MASK  << 8U) \
                                               | PMIC_MASK_FSM_ERR_REGADDR


#define PMIC_IRQ_MASK_COMM_ERR_MASK ((uint16_t)PMIC_MASK_COMM_ERR_MASK  << 8U) \
                                              | PMIC_MASK_COMM_ERR_REGADDR
#define PMIC_IRQ_I2C2_ADR_ERR_MASK  ((uint16_t)PMIC_MASK_COMM_ERR_I2C2_ADR_ERR_MASK_MASK   << 8U) \
                                              | PMIC_MASK_COMM_ERR_REGADDR
#define PMIC_IRQ_I2C2_CRC_ERR_MASK  ((uint16_t)PMIC_MASK_COMM_ERR_I2C2_CRC_ERR_MASK_MASK   << 8U) \
                                              | PMIC_MASK_COMM_ERR_REGADDR
#define PMIC_IRQ_COMM_ADR_ERR_MASK  ((uint16_t)PMIC_MASK_COMM_ERR_COMM_ADR_ERR_MASK_MASK   << 8U) \
                                              | PMIC_MASK_COMM_ERR_REGADDR
#define PMIC_IRQ_COMM_CRC_ERR_MASK  ((uint16_t)PMIC_MASK_COMM_ERR_COMM_CRC_ERR_MASK_MASK   << 8U) \
                                              | PMIC_MASK_COMM_ERR_REGADDR
#define PMIC_IRQ_COMM_FRM_ERR_MASK  ((uint16_t)PMIC_MASK_COMM_ERR_COMM_FRM_ERR_MASK_MASK   << 8U) \
                                              | PMIC_MASK_COMM_ERR_REGADDR
#define PMIC_IRQ_MASK_READBACK_ERR_MASK                                       \
                                ((uint16_t)PMIC_MASK_READBACK_ERR_MASK  << 8U) \
                                | PMIC_MASK_READBACK_ERR_REGADDR
#define PMIC_IRQ_NRSTOUT_SOC_READBACK_MASK                                    \
                             ((uint16_t)PMIC_MASK_READBACK_ERR_NRSTOUT_SOC_READBACK_MASK_MASK  << 8U) \
                             | PMIC_MASK_READBACK_ERR_REGADDR
#define PMIC_IRQ_EN_DRV_READBACK_MASK                                         \
                               ((uint16_t)PMIC_MASK_READBACK_ERR_EN_DRV_READBACK_MASK_MASK     << 8U) \
                               | PMIC_MASK_READBACK_ERR_REGADDR


#define PMIC_IRQ_MASK_ESM_MASK        ((uint16_t)PMIC_MASK_ESM_MASK     << 8U) \
                                               | PMIC_MASK_ESM_REGADDR
#define PMIC_IRQ_ESM_MCU_RST_MASK     ((uint16_t)PMIC_MASK_ESM_ESM_MCU_RST_MASK_MASK  << 8U) \
                                               | PMIC_MASK_ESM_REGADDR
#define PMIC_IRQ_ESM_MCU_FAIL_MASK    ((uint16_t)PMIC_MASK_ESM_ESM_MCU_FAIL_MASK_MASK << 8U) \
                                               | PMIC_MASK_ESM_REGADDR
#define PMIC_IRQ_ESM_MCU_PIN_MASK     ((uint16_t)PMIC_MASK_ESM_ESM_MCU_PIN_MASK_MASK  << 8U) \
                                               | PMIC_MASK_ESM_REGADDR
#define PMIC_IRQ_ESM_SOC_RST_MASK     ((uint16_t)PMIC_MASK_ESM_ESM_SOC_RST_MASK_MASK  << 8U) \
                                               | PMIC_MASK_ESM_REGADDR
#define PMIC_IRQ_ESM_SOC_FAIL_MASK    ((uint16_t)PMIC_MASK_ESM_ESM_SOC_FAIL_MASK_MASK << 8U) \
                                               | PMIC_MASK_ESM_REGADDR
#define PMIC_IRQ_ESM_SOC_PIN_MASK     ((uint16_t)PMIC_MASK_ESM_ESM_SOC_PIN_MASK_MASK  << 8U) \
                                               | PMIC_MASK_ESM_REGADDR
/* @} */

/*==========================================================================*/
/*                         Structures and Enums                             */
/*==========================================================================*/

/*==========================================================================*/
/*                         Function Declarations                            */
/*==========================================================================*/
/*!
 * \brief: PMIC function to read Error status
 *         This function does the following:
 *             1. This function gets the interrupt status by reading pmic
 *                IRQ register as per IRQ hierarchy defined in device TRM.
 *             2. Decipher error from top register to actual error code.
 *             3. Support clearing interrupt using clearIRQ flag as required.
 *             4. Works with the valid PMIC instance else does not do any
 *                operation.
 *
 * \param   pPmicCoreHandle   [IN]    PMIC Interface Handle.
 * \param   pErrStat          [OUT]   Variable to hold error interrupt ID
 * \param   clearIRQ          [IN]    Variable to control whether to clear the
 *                                    IRQ or not.
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code.
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t  Pmic_irqGetErrStatus(Pmic_CoreHandle_t *pPmicCoreHandle,
                              uint32_t          *pErrStat,
                              bool               clearIRQ);

/*!
 * \brief: PMIC function to clear Error status
 *         This function does the following:
 *          1. This function clears the IRQ bits in PMIC register for a given
 *             error code.
 *          2. Validates error code given by application and find the IRQ
 *             register that is to be updated.
 *          3. Expected to be called after an error code is generated by
 *             Pmic_irqGetErrStatus().
 *          4. Works with the valid PMIC instance else does not do any
 *             operation
 *
 * \param   pPmicCoreHandle   [IN]    PMIC Interface Handle.
 * \param   errStat           [IN]    Error status
 *
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code.
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t  Pmic_irqClrErrStatus(Pmic_CoreHandle_t *pPmicCoreHandle,
                              const uint32_t     errStat);

 /*!
 * \brief: PMIC function to mask/unmask interrupts
 *
 * \param   pPmicCoreHandle   [IN]    PMIC Interface Handle.
 * \param   interruptMask     [IN]    Interrupt Mask value
 * \param   mask              [IN]    Parameter to mask/unmask INTR
 *                                    Valid values: \ref Pmic_IrqMaskFlag
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code.
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_irqMaskIntr(Pmic_CoreHandle_t *pPmicCoreHandle,
                         uint16_t           interruptMask,
                         bool               mask);

#ifdef __cplusplus
}

#endif /* __cplusplus */

#endif /* PMIC_IRQ_H_ */

/* @} */
