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
 * \file   pmic_irq_tps6594x.c
 *
 * \brief  This file contains the TPS6594x Leo PMIC Interrupt APIs definitions 
 *         and structures.
 *
 */

#include <pmic_irq.h>
#include <pmic_core_priv.h>
#include <pmic_irq_priv.h>
#include <pmic_irq_tps6594x.h>
#include <pmic_irq_tps6594x_priv.h>

/* PMIC TPS6594x Interrupt Configuration as per Pmic_tps6594x_IrqNum. */
static Pmic_IntrCfg_t tps6594x_intCfg[] =
{
    {
        PMIC_WD_ERR_STATUS_REGADDR,
        PMIC_WD_ERR_STATUS_WD_RST_INT_SHIFT,
        PMIC_IRQ_INVALID_REGADDR,
        PMIC_IRQ_INVALID_BIT_SHIFT
    },
    {
        PMIC_WD_ERR_STATUS_REGADDR,
        PMIC_WD_ERR_STATUS_WD_FAIL_INT_SHIFT,
        PMIC_IRQ_INVALID_REGADDR,
        PMIC_IRQ_INVALID_BIT_SHIFT
    },
    {
        PMIC_WD_ERR_STATUS_REGADDR,
        PMIC_WD_ERR_STATUS_WD_LONGWIN_TIMEOUT_INT_SHIFT,
        PMIC_IRQ_INVALID_REGADDR,
        PMIC_IRQ_INVALID_BIT_SHIFT
    },
    {
        PMIC_INT_ESM_REGADDR,
        PMIC_INT_ESM_ESM_MCU_RST_INT_SHIFT,
        PMIC_MASK_ESM_REGADDR,
        PMIC_MASK_ESM_ESM_MCU_RST_MASK_SHIFT
    },
    {
        PMIC_INT_ESM_REGADDR,
        PMIC_INT_ESM_ESM_MCU_FAIL_INT_SHIFT,
        PMIC_MASK_ESM_REGADDR,
        PMIC_MASK_ESM_ESM_MCU_FAIL_MASK_SHIFT
    },
    {
        PMIC_INT_ESM_REGADDR,
        PMIC_INT_ESM_ESM_MCU_PIN_INT_SHIFT,
        PMIC_MASK_ESM_REGADDR,
        PMIC_MASK_ESM_ESM_MCU_PIN_MASK_SHIFT
    },
    {
        PMIC_INT_ESM_REGADDR,
        PMIC_INT_ESM_ESM_SOC_RST_INT_SHIFT,
        PMIC_MASK_ESM_REGADDR,
        PMIC_MASK_ESM_ESM_SOC_RST_MASK_SHIFT
    },
    {
        PMIC_INT_ESM_REGADDR,
        PMIC_INT_ESM_ESM_SOC_FAIL_INT_SHIFT,
        PMIC_MASK_ESM_REGADDR,
        PMIC_MASK_ESM_ESM_SOC_FAIL_MASK_SHIFT
    },
    {
        PMIC_INT_ESM_REGADDR,
        PMIC_INT_ESM_ESM_SOC_PIN_INT_SHIFT,
        PMIC_MASK_ESM_REGADDR,
        PMIC_MASK_ESM_ESM_SOC_PIN_MASK_SHIFT
    },
    {
        PMIC_INT_READBACK_ERR_REGADDR,
        PMIC_INT_READBACK_ERR_NRSTOUT_SOC_READBACK_INT_SHIFT,
        PMIC_MASK_READBACK_ERR_REGADDR,
        PMIC_MASK_READBACK_ERR_NRSTOUT_SOC_READBACK_MASK_SHIFT
    },
    {
        PMIC_INT_READBACK_ERR_REGADDR,
        PMIC_INT_READBACK_ERR_EN_DRV_READBACK_INT_SHIFT,
        PMIC_MASK_READBACK_ERR_REGADDR,
        PMIC_MASK_READBACK_ERR_EN_DRV_READBACK_MASK_SHIFT
    },
    {
        PMIC_INT_COMM_ERR_REGADDR,
        PMIC_INT_COMM_ERR_I2C2_ADR_ERR_INT_SHIFT,
        PMIC_MASK_COMM_ERR_REGADDR,
        PMIC_MASK_COMM_ERR_I2C2_ADR_ERR_MASK_SHIFT
    },
    {
        PMIC_INT_COMM_ERR_REGADDR,
        PMIC_INT_COMM_ERR_I2C2_CRC_ERR_INT_SHIFT,
        PMIC_MASK_COMM_ERR_REGADDR,
        PMIC_MASK_COMM_ERR_I2C2_CRC_ERR_MASK_SHIFT
    },
    {
        PMIC_INT_COMM_ERR_REGADDR,
        PMIC_INT_COMM_ERR_COMM_ADR_ERR_INT_SHIFT,
        PMIC_MASK_COMM_ERR_REGADDR,
        PMIC_MASK_COMM_ERR_COMM_ADR_ERR_MASK_SHIFT
    },
    {
        PMIC_INT_COMM_ERR_REGADDR,
        PMIC_INT_COMM_ERR_COMM_CRC_ERR_INT_SHIFT,
        PMIC_MASK_COMM_ERR_REGADDR,
        PMIC_MASK_COMM_ERR_COMM_CRC_ERR_MASK_SHIFT
    },
    {
        PMIC_INT_COMM_ERR_REGADDR,
        PMIC_INT_COMM_ERR_COMM_FRM_ERR_INT_SHIFT,
        PMIC_MASK_COMM_ERR_REGADDR,
        PMIC_MASK_COMM_ERR_COMM_FRM_ERR_MASK_SHIFT
    },
    {
        PMIC_INT_FSM_ERR_REGADDR,
        PMIC_INT_FSM_ERR_SOC_PWR_ERR_INT_SHIFT,
        PMIC_MASK_FSM_ERR_REGADDR,
        PMIC_MASK_FSM_ERR_SOC_PWR_ERR_MASK_SHIFT
    },
    {
        PMIC_INT_FSM_ERR_REGADDR,
        PMIC_INT_FSM_ERR_MCU_PWR_ERR_INT_SHIFT,
        PMIC_MASK_FSM_ERR_REGADDR,
        PMIC_MASK_FSM_ERR_MCU_PWR_ERR_MASK_SHIFT
    },
    {
        PMIC_INT_FSM_ERR_REGADDR,
        PMIC_INT_FSM_ERR_ORD_SHUTDOWN_INT_SHIFT,
        PMIC_MASK_FSM_ERR_REGADDR,
        PMIC_MASK_FSM_ERR_ORD_SHUTDOWN_MASK_SHIFT
    },
    {
        PMIC_INT_FSM_ERR_REGADDR,
        PMIC_INT_FSM_ERR_IMM_SHUTDOWN_INT_SHIFT,
        PMIC_MASK_FSM_ERR_REGADDR,
        PMIC_MASK_FSM_ERR_IMM_SHUTDOWN_MASK_SHIFT
    },
    {
        PMIC_INT_SEVERE_ERR_REGADDR,
        PMIC_INT_SEVERE_ERR_PFSM_ERR_INT_SHIFT,
        PMIC_IRQ_INVALID_REGADDR,
        PMIC_IRQ_INVALID_BIT_SHIFT
    },
    {
        PMIC_INT_SEVERE_ERR_REGADDR,
        PMIC_INT_SEVERE_ERR_VCCA_OVP_INT_SHIFT,
        PMIC_IRQ_INVALID_REGADDR,
        PMIC_IRQ_INVALID_BIT_SHIFT
    },
    {
        PMIC_INT_SEVERE_ERR_REGADDR,
        PMIC_INT_SEVERE_ERR_TSD_IMM_INT_SHIFT,
        PMIC_IRQ_INVALID_REGADDR,
        PMIC_IRQ_INVALID_BIT_SHIFT
    },
    {
        PMIC_INT_MODERATE_ERR_REGADDR,
        PMIC_INT_MODERATE_ERR_NRSTOUT_READBACK_INT_SHIFT,
        PMIC_MASK_MODERATE_ERR_REGADDR,
        PMIC_MASK_MODERATE_ERR_NRSTOUT_READBACK_MASK_SHIFT
    },
    {
        PMIC_INT_MODERATE_ERR_REGADDR,
        PMIC_INT_MODERATE_ERR_NINT_READBACK_INT_SHIFT,
        PMIC_MASK_MODERATE_ERR_REGADDR,
        PMIC_MASK_MODERATE_ERR_NINT_READBACK_MASK_SHIFT
    },
    {
        PMIC_INT_MODERATE_ERR_REGADDR,
        PMIC_INT_MODERATE_ERR_NPWRON_LONG_INT_SHIFT,
        PMIC_MASK_MODERATE_ERR_REGADDR,
        PMIC_MASK_MODERATE_ERR_NPWRON_LONG_MASK_SHIFT
    },
    {
        PMIC_INT_MODERATE_ERR_REGADDR,
        PMIC_INT_MODERATE_ERR_SPMI_ERR_INT_SHIFT,
        PMIC_MASK_MODERATE_ERR_REGADDR,
        PMIC_MASK_MODERATE_ERR_SPMI_ERR_MASK_SHIFT
    },
    {
        PMIC_INT_MODERATE_ERR_REGADDR,
        PMIC_INT_MODERATE_ERR_RECOV_CNT_INT_SHIFT,
        PMIC_IRQ_INVALID_REGADDR,
        PMIC_IRQ_INVALID_BIT_SHIFT
    },
    {
        PMIC_INT_MODERATE_ERR_REGADDR,
        PMIC_INT_MODERATE_ERR_REG_CRC_ERR_INT_SHIFT,
        PMIC_MASK_MODERATE_ERR_REGADDR,
        PMIC_MASK_MODERATE_ERR_REG_CRC_ERR_MASK_SHIFT
    },
    {
        PMIC_INT_MODERATE_ERR_REGADDR,
        PMIC_INT_MODERATE_ERR_BIST_FAIL_INT_SHIFT,
        PMIC_MASK_MODERATE_ERR_REGADDR,
        PMIC_MASK_MODERATE_ERR_BIST_FAIL_MASK_SHIFT
    },
    {
        PMIC_INT_MODERATE_ERR_REGADDR,
        PMIC_INT_MODERATE_ERR_TSD_ORD_INT_SHIFT,
        PMIC_IRQ_INVALID_REGADDR,
        PMIC_IRQ_INVALID_BIT_SHIFT
    },
    {
        PMIC_INT_MISC_REGADDR,
        PMIC_INT_MISC_TWARN_INT_SHIFT,
        PMIC_MASK_MISC_REGADDR,
        PMIC_MASK_MISC_TWARN_MASK_SHIFT
    },
    {
        PMIC_INT_MISC_REGADDR,
        PMIC_INT_MISC_EXT_CLK_INT_SHIFT,
        PMIC_MASK_MISC_REGADDR,
        PMIC_MASK_MISC_EXT_CLK_MASK_SHIFT
    },
    {
        PMIC_INT_MISC_REGADDR,
        PMIC_INT_MISC_BIST_PASS_INT_SHIFT,
        PMIC_MASK_MISC_REGADDR,
        PMIC_MASK_MISC_BIST_PASS_MASK_SHIFT
    },
    {
        PMIC_INT_STARTUP_REGADDR,
        PMIC_INT_STARTUP_FSD_INT_SHIFT,
        PMIC_MASK_STARTUP_REGADDR,
        PMIC_MASK_STARTUP_FSD_MASK_SHIFT
    },
    {
        PMIC_RTC_STATUS_REGADDR,
        PMIC_RTC_STATUS_ALARM_SHIFT,
        PMIC_RTC_INTERRUPTS_REGADDR,
        PMIC_RTC_INTERRUPTS_IT_ALARM_SHIFT
    },
    {
        PMIC_RTC_STATUS_REGADDR,
        PMIC_RTC_STATUS_TIMER_SHIFT,
        PMIC_RTC_INTERRUPTS_REGADDR,
        PMIC_RTC_INTERRUPTS_IT_TIMER_SHIFT
    },
    {
        PMIC_INT_STARTUP_REGADDR,
        PMIC_INT_STARTUP_ENABLE_INT_SHIFT,
        PMIC_MASK_STARTUP_REGADDR,
        PMIC_MASK_STARTUP_ENABLE_MASK_SHIFT,
    },
    {
        PMIC_INT_STARTUP_REGADDR,
        PMIC_INT_STARTUP_NPWRON_START_INT_SHIFT,
        PMIC_MASK_STARTUP_REGADDR,
        PMIC_MASK_STARTUP_NPWRON_START_MASK_SHIFT
    },
    {
        PMIC_INT_GPIO1_8_REGADDR,
        PMIC_INT_GPIO1_8_GPIO8_INT_SHIFT,
        PMIC_IRQ_INVALID_REGADDR,
        PMIC_IRQ_INVALID_BIT_SHIFT
    },
    {
        PMIC_INT_GPIO1_8_REGADDR,
        PMIC_INT_GPIO1_8_GPIO7_INT_SHIFT,
        PMIC_IRQ_INVALID_REGADDR,
        PMIC_IRQ_INVALID_BIT_SHIFT
    },
    {
        PMIC_INT_GPIO1_8_REGADDR,
        PMIC_INT_GPIO1_8_GPIO6_INT_SHIFT,
        PMIC_IRQ_INVALID_REGADDR,
        PMIC_IRQ_INVALID_BIT_SHIFT
    },
    {
        PMIC_INT_GPIO1_8_REGADDR,
        PMIC_INT_GPIO1_8_GPIO5_INT_SHIFT,
        PMIC_IRQ_INVALID_REGADDR,
        PMIC_IRQ_INVALID_BIT_SHIFT
    },
    {
        PMIC_INT_GPIO1_8_REGADDR,
        PMIC_INT_GPIO1_8_GPIO4_INT_SHIFT,
        PMIC_IRQ_INVALID_REGADDR,
        PMIC_IRQ_INVALID_BIT_SHIFT
    },
    {
        PMIC_INT_GPIO1_8_REGADDR,
        PMIC_INT_GPIO1_8_GPIO3_INT_SHIFT,
        PMIC_IRQ_INVALID_REGADDR,
        PMIC_IRQ_INVALID_BIT_SHIFT
    },
    {
        PMIC_INT_GPIO1_8_REGADDR,
        PMIC_INT_GPIO1_8_GPIO2_INT_SHIFT,
        PMIC_IRQ_INVALID_REGADDR,
        PMIC_IRQ_INVALID_BIT_SHIFT
    },
    {
        PMIC_INT_GPIO1_8_REGADDR,
        PMIC_INT_GPIO1_8_GPIO1_INT_SHIFT,
        PMIC_IRQ_INVALID_REGADDR,
        PMIC_IRQ_INVALID_BIT_SHIFT
    },
    {
        PMIC_INT_GPIO_REGADDR,
        PMIC_INT_GPIO_GPIO11_INT_SHIFT,
        PMIC_IRQ_INVALID_REGADDR,
        PMIC_IRQ_INVALID_BIT_SHIFT
    },
    {
        PMIC_INT_GPIO_REGADDR,
        PMIC_INT_GPIO_GPIO10_INT_SHIFT,
        PMIC_IRQ_INVALID_REGADDR,
        PMIC_IRQ_INVALID_BIT_SHIFT
    },
    {
        PMIC_INT_GPIO_REGADDR,
        PMIC_INT_GPIO_GPIO9_INT_SHIFT,
        PMIC_IRQ_INVALID_REGADDR,
        PMIC_IRQ_INVALID_BIT_SHIFT
    },
    {
        PMIC_INT_VMON_REGADDR,
        PMIC_INT_VMON_VCCA_UV_INT_SHIFT,
        PMIC_MASK_VMON_REGADDR,
        PMIC_MASK_VMON_VCCA_UV_MASK_SHIFT
    },
    {
        PMIC_INT_VMON_REGADDR,
        PMIC_INT_VMON_VCCA_OV_INT_SHIFT,
        PMIC_MASK_VMON_REGADDR,
        PMIC_MASK_VMON_VCCA_OV_MASK_SHIFT
    },
    {
        PMIC_INT_LDO3_4_REGADDR,
        PMIC_INT_LDO3_4_LDO4_ILIM_INT_SHIFT,
        PMIC_MASK_LDO3_4_REGADDR,
        PMIC_MASK_LDO3_4_LDO4_ILIM_MASK_SHIFT
    },
    {
        PMIC_INT_LDO3_4_REGADDR,
        PMIC_INT_LDO3_4_LDO4_SC_INT_SHIFT,
        PMIC_IRQ_INVALID_REGADDR,
        PMIC_IRQ_INVALID_BIT_SHIFT
    },
    {
        PMIC_INT_LDO3_4_REGADDR,
        PMIC_INT_LDO3_4_LDO4_UV_INT_SHIFT,
        PMIC_MASK_LDO3_4_REGADDR,
        PMIC_MASK_LDO3_4_LDO4_UV_MASK_SHIFT
    },
    {
        PMIC_INT_LDO3_4_REGADDR,
        PMIC_INT_LDO3_4_LDO4_OV_INT_SHIFT,
        PMIC_MASK_LDO3_4_REGADDR,
        PMIC_MASK_LDO3_4_LDO4_OV_MASK_SHIFT
    },
    {
        PMIC_INT_LDO3_4_REGADDR,
        PMIC_INT_LDO3_4_LDO3_ILIM_INT_SHIFT,
        PMIC_MASK_LDO3_4_REGADDR,
        PMIC_MASK_LDO3_4_LDO3_ILIM_MASK_SHIFT
    },
    {
        PMIC_INT_LDO3_4_REGADDR,
        PMIC_INT_LDO3_4_LDO3_SC_INT_SHIFT,
        PMIC_IRQ_INVALID_REGADDR,
        PMIC_IRQ_INVALID_BIT_SHIFT
    },
    {
        PMIC_INT_LDO3_4_REGADDR,
        PMIC_INT_LDO3_4_LDO3_UV_INT_SHIFT,
        PMIC_MASK_LDO3_4_REGADDR,
        PMIC_MASK_LDO3_4_LDO3_UV_MASK_SHIFT
    },
    {
        PMIC_INT_LDO3_4_REGADDR,
        PMIC_INT_LDO3_4_LDO3_OV_INT_SHIFT,
        PMIC_MASK_LDO3_4_REGADDR,
        PMIC_MASK_LDO3_4_LDO3_OV_MASK_SHIFT
    },
    {
        PMIC_INT_LDO1_2_REGADDR,
        PMIC_INT_LDO1_2_LDO2_ILIM_INT_SHIFT,
        PMIC_MASK_LDO1_2_REGADDR,
        PMIC_MASK_LDO1_2_LDO2_ILIM_MASK_SHIFT
    },
    {
        PMIC_INT_LDO1_2_REGADDR,
        PMIC_INT_LDO1_2_LDO2_SC_INT_SHIFT,
        PMIC_IRQ_INVALID_REGADDR,
        PMIC_IRQ_INVALID_BIT_SHIFT
    },
    {
        PMIC_INT_LDO1_2_REGADDR,
        PMIC_INT_LDO1_2_LDO2_UV_INT_SHIFT,
        PMIC_MASK_LDO1_2_REGADDR,
        PMIC_MASK_LDO1_2_LDO2_UV_MASK_SHIFT
    },
    {
        PMIC_INT_LDO1_2_REGADDR,
        PMIC_INT_LDO1_2_LDO2_OV_INT_SHIFT,
        PMIC_MASK_LDO1_2_REGADDR,
        PMIC_MASK_LDO1_2_LDO2_OV_MASK_SHIFT
    },
    {
        PMIC_INT_LDO1_2_REGADDR,
        PMIC_INT_LDO1_2_LDO1_ILIM_INT_SHIFT,
        PMIC_MASK_LDO1_2_REGADDR,
        PMIC_MASK_LDO1_2_LDO1_ILIM_MASK_SHIFT
    },
    {
        PMIC_INT_LDO1_2_REGADDR,
        PMIC_INT_LDO1_2_LDO1_SC_INT_SHIFT,
        PMIC_IRQ_INVALID_REGADDR,
        PMIC_IRQ_INVALID_BIT_SHIFT
    },
    {
        PMIC_INT_LDO1_2_REGADDR,
        PMIC_INT_LDO1_2_LDO1_UV_INT_SHIFT,
        PMIC_MASK_LDO1_2_REGADDR,
        PMIC_MASK_LDO1_2_LDO1_UV_MASK_SHIFT
    },
    {
        PMIC_INT_LDO1_2_REGADDR,
        PMIC_INT_LDO1_2_LDO1_OV_INT_SHIFT,
        PMIC_MASK_LDO1_2_REGADDR,
        PMIC_MASK_LDO1_2_LDO1_OV_MASK_SHIFT
    },
    {
        PMIC_INT_BUCK5_REGADDR,
        PMIC_INT_BUCK5_BUCK5_ILIM_INT_SHIFT,
        PMIC_MASK_BUCK5_REGADDR,
        PMIC_MASK_BUCK5_BUCK5_ILIM_MASK_SHIFT
    },
    {
        PMIC_INT_BUCK5_REGADDR,
        PMIC_INT_BUCK5_BUCK5_SC_INT_SHIFT,
        PMIC_IRQ_INVALID_REGADDR,
        PMIC_IRQ_INVALID_BIT_SHIFT
    },
    {
        PMIC_INT_BUCK5_REGADDR,
        PMIC_INT_BUCK5_BUCK5_UV_INT_SHIFT,
        PMIC_MASK_BUCK5_REGADDR,
        PMIC_MASK_BUCK5_BUCK5_UV_MASK_SHIFT
    },
    {
        PMIC_INT_BUCK5_REGADDR,
        PMIC_INT_BUCK5_BUCK5_OV_INT_SHIFT,
        PMIC_MASK_BUCK5_REGADDR,
        PMIC_MASK_BUCK5_BUCK5_OV_MASK_SHIFT
    },
    {
        PMIC_INT_BUCK3_4_REGADDR,
        PMIC_INT_BUCK3_4_BUCK4_ILIM_INT_SHIFT,
        PMIC_MASK_BUCK3_4_REGADDR,
        PMIC_MASK_BUCK3_4_BUCK4_ILIM_MASK_SHIFT
    },
    {
        PMIC_INT_BUCK3_4_REGADDR,
        PMIC_INT_BUCK3_4_BUCK4_SC_INT_SHIFT,
        PMIC_IRQ_INVALID_REGADDR,
        PMIC_IRQ_INVALID_BIT_SHIFT
    },
    {
        PMIC_INT_BUCK3_4_REGADDR,
        PMIC_INT_BUCK3_4_BUCK4_UV_INT_SHIFT,
        PMIC_MASK_BUCK3_4_REGADDR,
        PMIC_MASK_BUCK3_4_BUCK4_UV_MASK_SHIFT
    },
    {
        PMIC_INT_BUCK3_4_REGADDR,
        PMIC_INT_BUCK3_4_BUCK4_OV_INT_SHIFT,
        PMIC_MASK_BUCK3_4_REGADDR,
        PMIC_MASK_BUCK3_4_BUCK4_OV_MASK_SHIFT
    },
    {
        PMIC_INT_BUCK3_4_REGADDR,
        PMIC_INT_BUCK3_4_BUCK3_ILIM_INT_SHIFT,
        PMIC_MASK_BUCK3_4_REGADDR,
        PMIC_MASK_BUCK3_4_BUCK3_ILIM_MASK_SHIFT
    },
    {
        PMIC_INT_BUCK3_4_REGADDR,
        PMIC_INT_BUCK3_4_BUCK3_SC_INT_SHIFT,
        PMIC_IRQ_INVALID_REGADDR,
        PMIC_IRQ_INVALID_BIT_SHIFT
    },
    {
        PMIC_INT_BUCK3_4_REGADDR,
        PMIC_INT_BUCK3_4_BUCK3_UV_INT_SHIFT,
        PMIC_MASK_BUCK3_4_REGADDR,
        PMIC_MASK_BUCK3_4_BUCK3_UV_MASK_SHIFT
    },
    {
        PMIC_INT_BUCK3_4_REGADDR,
        PMIC_INT_BUCK3_4_BUCK3_OV_INT_SHIFT,
        PMIC_MASK_BUCK3_4_REGADDR,
        PMIC_MASK_BUCK3_4_BUCK3_OV_MASK_SHIFT
    },
    {
        PMIC_INT_BUCK1_2_REGADDR,
        PMIC_INT_BUCK1_2_BUCK2_ILIM_INT_SHIFT,
        PMIC_MASK_BUCK1_2_REGADDR,
        PMIC_MASK_BUCK1_2_BUCK2_ILIM_MASK_SHIFT
    },
    {
        PMIC_INT_BUCK1_2_REGADDR,
        PMIC_INT_BUCK1_2_BUCK2_SC_INT_SHIFT,
        PMIC_IRQ_INVALID_REGADDR,
        PMIC_IRQ_INVALID_BIT_SHIFT
    },
    {
        PMIC_INT_BUCK1_2_REGADDR,
        PMIC_INT_BUCK1_2_BUCK2_UV_INT_SHIFT,
        PMIC_MASK_BUCK1_2_REGADDR,
        PMIC_MASK_BUCK1_2_BUCK2_UV_MASK_SHIFT
    },
    {
        PMIC_INT_BUCK1_2_REGADDR,
        PMIC_INT_BUCK1_2_BUCK2_OV_INT_SHIFT,
        PMIC_MASK_BUCK1_2_REGADDR,
        PMIC_MASK_BUCK1_2_BUCK2_OV_MASK_SHIFT
    },
    {
        PMIC_INT_BUCK1_2_REGADDR,
        PMIC_INT_BUCK1_2_BUCK1_ILIM_INT_SHIFT,
        PMIC_MASK_BUCK1_2_REGADDR,
        PMIC_MASK_BUCK1_2_BUCK1_ILIM_MASK_SHIFT
    },
    {
        PMIC_INT_BUCK1_2_REGADDR,
        PMIC_INT_BUCK1_2_BUCK1_SC_INT_SHIFT,
        PMIC_IRQ_INVALID_REGADDR,
        PMIC_IRQ_INVALID_BIT_SHIFT
    },
    {
        PMIC_INT_BUCK1_2_REGADDR,
        PMIC_INT_BUCK1_2_BUCK1_UV_INT_SHIFT,
        PMIC_MASK_BUCK1_2_REGADDR,
        PMIC_MASK_BUCK1_2_BUCK1_UV_MASK_SHIFT
    },
    {
        PMIC_INT_BUCK1_2_REGADDR,
        PMIC_INT_BUCK1_2_BUCK1_OV_INT_SHIFT,
        PMIC_MASK_BUCK1_2_REGADDR,
        PMIC_MASK_BUCK1_2_BUCK1_OV_MASK_SHIFT
    }
};

/*  PMIC TPS6594x GPIO Interrupt Mask Configuration as per 
 *  Pmic_tps6594x_IrqGpioNum.
 */
static Pmic_GpioIntrTypeCfg_t tps6594x_gpioIntrCfg[] =
{
    {
        PMIC_MASK_GPIO1_8_RISE_REGADDR,
        PMIC_MASK_GPIO1_8_RISE_GPIO1_RISE_MASK_SHIFT,
        PMIC_MASK_GPIO1_8_FALL_REGADDR,
        PMIC_MASK_GPIO1_8_FALL_GPIO1_FALL_MASK_SHIFT
    },
    {
        PMIC_MASK_GPIO1_8_RISE_REGADDR,
        PMIC_MASK_GPIO1_8_RISE_GPIO2_RISE_MASK_SHIFT,
        PMIC_MASK_GPIO1_8_FALL_REGADDR,
        PMIC_MASK_GPIO1_8_FALL_GPIO2_FALL_MASK_SHIFT
    },
    {
        PMIC_MASK_GPIO1_8_RISE_REGADDR,
        PMIC_MASK_GPIO1_8_RISE_GPIO3_RISE_MASK_SHIFT,
        PMIC_MASK_GPIO1_8_FALL_REGADDR,
        PMIC_MASK_GPIO1_8_FALL_GPIO3_FALL_MASK_SHIFT
    },
    {
        PMIC_MASK_GPIO1_8_RISE_REGADDR,
        PMIC_MASK_GPIO1_8_RISE_GPIO4_RISE_MASK_SHIFT,
        PMIC_MASK_GPIO1_8_FALL_REGADDR,
        PMIC_MASK_GPIO1_8_FALL_GPIO4_FALL_MASK_SHIFT
    },
    {
        PMIC_MASK_GPIO1_8_RISE_REGADDR,
        PMIC_MASK_GPIO1_8_RISE_GPIO5_RISE_MASK_SHIFT,
        PMIC_MASK_GPIO1_8_FALL_REGADDR,
        PMIC_MASK_GPIO1_8_FALL_GPIO5_FALL_MASK_SHIFT
    },
    {
        PMIC_MASK_GPIO1_8_RISE_REGADDR,
        PMIC_MASK_GPIO1_8_RISE_GPIO6_RISE_MASK_SHIFT,
        PMIC_MASK_GPIO1_8_FALL_REGADDR,
        PMIC_MASK_GPIO1_8_FALL_GPIO6_FALL_MASK_SHIFT
    },
    {
        PMIC_MASK_GPIO1_8_RISE_REGADDR,
        PMIC_MASK_GPIO1_8_RISE_GPIO7_RISE_MASK_SHIFT,
        PMIC_MASK_GPIO1_8_FALL_REGADDR,
        PMIC_MASK_GPIO1_8_FALL_GPIO7_FALL_MASK_SHIFT
    },
    {
        PMIC_MASK_GPIO1_8_RISE_REGADDR,
        PMIC_MASK_GPIO1_8_RISE_GPIO8_RISE_MASK_SHIFT,
        PMIC_MASK_GPIO1_8_FALL_REGADDR,
        PMIC_MASK_GPIO1_8_FALL_GPIO8_FALL_MASK_SHIFT
    },
    {
        PMIC_MASK_GPIO9_11_REGADDR,
        PMIC_MASK_GPIO9_11_GPIO9_RISE_MASK_SHIFT,
        PMIC_MASK_GPIO9_11_REGADDR,
        PMIC_MASK_GPIO9_11_GPIO9_FALL_MASK_SHIFT
    },
    {
        PMIC_MASK_GPIO9_11_REGADDR,
        PMIC_MASK_GPIO9_11_GPIO10_RISE_MASK_SHIFT,
        PMIC_MASK_GPIO9_11_REGADDR,
        PMIC_MASK_GPIO9_11_GPIO10_FALL_MASK_SHIFT
    },
    {
        PMIC_MASK_GPIO9_11_REGADDR,
        PMIC_MASK_GPIO9_11_GPIO11_RISE_MASK_SHIFT,
        PMIC_MASK_GPIO9_11_REGADDR,
        PMIC_MASK_GPIO9_11_GPIO11_FALL_MASK_SHIFT
    }
};

/*
 * \brief   Get TPS6594x Interrupt config.
 *          This function is used to get TPS6594x Interrupt configuration.
 *
 * \param   pIntrCfg   [OUT]  to store tps6594x Interrupt configuration.
 */
void pmic_get_tps6594x_intrCfg(Pmic_IntrCfg_t **pIntrCfg)
{
    *pIntrCfg = tps6594x_intCfg;
}

/*
 * \brief   Get TPS6594x Interrupt config.
 *          This function is used to get TPS6594x Interrupt configuration.
 *
 * \param   pGpioIntrCfg   [OUT]  to store tps6594x Interrupt configuration.
 */
void pmic_get_tps6594x_intrGpioCfg(Pmic_GpioIntrTypeCfg_t **pGpioIntrCfg)
{
    *pGpioIntrCfg = tps6594x_gpioIntrCfg;
}

/*!
 * \brief  Function to decipher BUCK Error.
 */
static int32_t Pmic_tps6594x_getBuckErr(Pmic_CoreHandle_t *pPmicCoreHandle,
                                        uint8_t            regValue,
                                        Pmic_IrqStatus_t  *pErrStat)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData    = 0U;

    /* Start Critical Section */
    Pmic_criticalSectionStart(pPmicCoreHandle);

    /* PMIC BUCK 5 Interrupt Status Check */
    if(((regValue & PMIC_INT_BUCK_BUCK5_INT_MASK) != 0U) && 
       (PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType))
    {
        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                            PMIC_INT_BUCK5_REGADDR,
                                            &regData);
        if((PMIC_ST_SUCCESS == pmicStatus) && (0U != regData))
        {
            if((regData & PMIC_INT_BUCK5_BUCK5_OV_INT_MASK) != 0U)
            {
                Pmic_intrBitSet(pErrStat, PMIC_TPS6594X_BUCK5_OV_INT);
            }

            if((regData & PMIC_INT_BUCK5_BUCK5_UV_INT_MASK) != 0U)
            {
                Pmic_intrBitSet(pErrStat, PMIC_TPS6594X_BUCK5_UV_INT);
            }

            if((regData & PMIC_INT_BUCK5_BUCK5_SC_INT_MASK) != 0U)
            {
                Pmic_intrBitSet(pErrStat, PMIC_TPS6594X_BUCK5_SC_INT);
            }

            if((regData & PMIC_INT_BUCK5_BUCK5_ILIM_INT_MASK) != 0U)
            {
                Pmic_intrBitSet(pErrStat, PMIC_TPS6594X_BUCK5_ILIM_INT);
            }
        }
    }

    /* PMIC BUCK3_4 Interrupt Status Check */
    if((regValue & PMIC_INT_BUCK_BUCK3_4_INT_MASK) != 0U)
    {
        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                            PMIC_INT_BUCK3_4_REGADDR,
                                            &regData);
        if((PMIC_ST_SUCCESS == pmicStatus) && (0U != regData))
        {
            if((regData & PMIC_INT_BUCK3_4_BUCK4_OV_INT_MASK) != 0U)
            {
                Pmic_intrBitSet(pErrStat, PMIC_TPS6594X_BUCK4_OV_INT);
            }

            if((regData & PMIC_INT_BUCK3_4_BUCK4_UV_INT_MASK) != 0U)
            {
                Pmic_intrBitSet(pErrStat, PMIC_TPS6594X_BUCK4_UV_INT);
            }

            if((regData & PMIC_INT_BUCK3_4_BUCK4_SC_INT_MASK) != 0U)
            {
                Pmic_intrBitSet(pErrStat, PMIC_TPS6594X_BUCK4_SC_INT);
            }

            if((regData & PMIC_INT_BUCK3_4_BUCK4_ILIM_INT_MASK) != 0U)
            {
                Pmic_intrBitSet(pErrStat, PMIC_TPS6594X_BUCK4_ILIM_INT);
            }

            if((regData & PMIC_INT_BUCK3_4_BUCK3_OV_INT_MASK) != 0U)
            {
                Pmic_intrBitSet(pErrStat, PMIC_TPS6594X_BUCK3_OV_INT);
            }

            if((regData & PMIC_INT_BUCK3_4_BUCK3_UV_INT_MASK) != 0U)
            {
                Pmic_intrBitSet(pErrStat, PMIC_TPS6594X_BUCK3_UV_INT);
            }

            if((regData & PMIC_INT_BUCK3_4_BUCK3_SC_INT_MASK) != 0U)
            {
                Pmic_intrBitSet(pErrStat, PMIC_TPS6594X_BUCK3_SC_INT);
            }

            if((regData & PMIC_INT_BUCK3_4_BUCK3_ILIM_INT_MASK) != 0U)
            {
                Pmic_intrBitSet(pErrStat, PMIC_TPS6594X_BUCK3_ILIM_INT);
            }
        }
    }

    /* PMIC BUCK1_2 Interrupt Status Check */
    if((regValue & PMIC_INT_BUCK_BUCK1_2_INT_MASK) != 0U)
    {
        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                            PMIC_INT_BUCK1_2_REGADDR,
                                            &regData);
        if((PMIC_ST_SUCCESS == pmicStatus) && (0U != regData))
        {
            if((regData & PMIC_INT_BUCK1_2_BUCK2_OV_INT_MASK) != 0U)
            {
                Pmic_intrBitSet(pErrStat, PMIC_TPS6594X_BUCK2_OV_INT);
            }

            if((regData & PMIC_INT_BUCK1_2_BUCK2_UV_INT_MASK) != 0U)
            {
                Pmic_intrBitSet(pErrStat, PMIC_TPS6594X_BUCK2_UV_INT);
            }

            if((regData & PMIC_INT_BUCK1_2_BUCK2_SC_INT_MASK) != 0U)
            {
                Pmic_intrBitSet(pErrStat, PMIC_TPS6594X_BUCK2_SC_INT);
            }

            if((regData & PMIC_INT_BUCK1_2_BUCK2_ILIM_INT_MASK) != 0U)
            {
                Pmic_intrBitSet(pErrStat, PMIC_TPS6594X_BUCK2_ILIM_INT);
            }

            if((regData & PMIC_INT_BUCK1_2_BUCK1_OV_INT_MASK) != 0U)
            {
                Pmic_intrBitSet(pErrStat, PMIC_TPS6594X_BUCK1_OV_INT);
            }

            if((regData & PMIC_INT_BUCK1_2_BUCK1_UV_INT_MASK) != 0U)
            {
                Pmic_intrBitSet(pErrStat, PMIC_TPS6594X_BUCK1_UV_INT);
            }

            if((regData & PMIC_INT_BUCK1_2_BUCK1_SC_INT_MASK) != 0U)
            {
                Pmic_intrBitSet(pErrStat, PMIC_TPS6594X_BUCK1_SC_INT);
            }

            if((regData & PMIC_INT_BUCK1_2_BUCK1_ILIM_INT_MASK) != 0U)
            {
                Pmic_intrBitSet(pErrStat, PMIC_TPS6594X_BUCK1_ILIM_INT);
            }
        }
    }

    /* Stop Critical Section */
    Pmic_criticalSectionStop(pPmicCoreHandle);

    return pmicStatus;
}

/*!
 * \brief  Function to decipher LDO_VMON Error.
 */
static int32_t Pmic_tps6594x_getLdoVmonErr(Pmic_CoreHandle_t *pPmicCoreHandle,
                                           uint8_t            regValue,
                                           Pmic_IrqStatus_t  *pErrStat)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData    = 0U;

    /* Start Critical Section */
    Pmic_criticalSectionStart(pPmicCoreHandle);

    if((regValue & PMIC_INT_LDO_VMON_LDO1_2_INT_MASK) != 0U)
    {
        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                            PMIC_MASK_LDO1_2_REGADDR,
                                            &regData);

        if((PMIC_ST_SUCCESS == pmicStatus) && (0U != regData))
        {
            if((regData & PMIC_INT_LDO1_2_LDO1_OV_INT_MASK) != 0U)
            {
                Pmic_intrBitSet(pErrStat, PMIC_TPS6594X_LDO1_OV_INT);
            }

            if((regData & PMIC_INT_LDO1_2_LDO1_UV_INT_MASK) != 0U)
            {
                Pmic_intrBitSet(pErrStat, PMIC_TPS6594X_LDO1_UV_INT);
            }

            if((regData & PMIC_INT_LDO1_2_LDO1_SC_INT_MASK) != 0U)
            {
                Pmic_intrBitSet(pErrStat, PMIC_TPS6594X_LDO1_SC_INT);
            }

            if((regData & PMIC_INT_LDO1_2_LDO1_ILIM_INT_MASK) != 0U)
            {
                Pmic_intrBitSet(pErrStat, PMIC_TPS6594X_LDO1_ILIM_INT);
            }

            if((regData & PMIC_INT_LDO1_2_LDO2_OV_INT_MASK) != 0U)
            {
                Pmic_intrBitSet(pErrStat, PMIC_TPS6594X_LDO2_OV_INT);
            }

            if((regData & PMIC_INT_LDO1_2_LDO2_UV_INT_MASK) != 0U)
            {
                Pmic_intrBitSet(pErrStat, PMIC_TPS6594X_LDO2_UV_INT);
            }

            if((regData & PMIC_INT_LDO1_2_LDO2_SC_INT_MASK) != 0U)
            {
                Pmic_intrBitSet(pErrStat, PMIC_TPS6594X_LDO2_SC_INT);
            }

            if((regData & PMIC_INT_LDO1_2_LDO2_ILIM_INT_MASK) != 0U)
            {
                Pmic_intrBitSet(pErrStat, PMIC_TPS6594X_LDO2_ILIM_INT);
            }
        }
    }

    if((regValue & PMIC_INT_LDO_VMON_LDO3_4_INT_MASK) != 0U)
    {
        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                            PMIC_MASK_LDO3_4_REGADDR,
                                            &regData);

        if((PMIC_ST_SUCCESS == pmicStatus) && (0U != regData))
        {
            if((regData & PMIC_INT_LDO3_4_LDO3_OV_INT_MASK) != 0U)
            {
                Pmic_intrBitSet(pErrStat, PMIC_TPS6594X_LDO3_OV_INT);
            }

            if((regData & PMIC_INT_LDO3_4_LDO3_UV_INT_MASK) != 0U)
            {
                Pmic_intrBitSet(pErrStat, PMIC_TPS6594X_LDO3_UV_INT);
            }

            if((regData & PMIC_INT_LDO3_4_LDO3_SC_INT_MASK) != 0U)
            {
                Pmic_intrBitSet(pErrStat, PMIC_TPS6594X_LDO3_SC_INT);
            }

            if((regData & PMIC_INT_LDO3_4_LDO3_ILIM_INT_MASK) != 0U)
            {
                Pmic_intrBitSet(pErrStat, PMIC_TPS6594X_LDO3_ILIM_INT);
            }

            if((regData & PMIC_INT_LDO3_4_LDO4_OV_INT_MASK) != 0U)
            {
                Pmic_intrBitSet(pErrStat, PMIC_TPS6594X_LDO4_OV_INT);
            }

            if((regData & PMIC_INT_LDO3_4_LDO4_UV_INT_MASK) != 0U)
            {
                Pmic_intrBitSet(pErrStat, PMIC_TPS6594X_LDO4_UV_INT);
            }

            if((regData & PMIC_INT_LDO3_4_LDO4_SC_INT_MASK) != 0U)
            {
                Pmic_intrBitSet(pErrStat, PMIC_TPS6594X_LDO4_SC_INT);
            }

            if((regData & PMIC_INT_LDO3_4_LDO4_ILIM_INT_MASK) != 0U)
            {
                Pmic_intrBitSet(pErrStat, PMIC_TPS6594X_LDO4_ILIM_INT);
            }
        }
    }

    if((regValue & PMIC_INT_LDO_VMON_VCCA_INT_MASK) != 0U)
    {
        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                            PMIC_INT_VMON_REGADDR,
                                            &regData);

        if((PMIC_ST_SUCCESS == pmicStatus) && (0U != regData))
        {
            if((regData & PMIC_INT_VMON_VCCA_OV_INT_MASK) != 0U)
            {
                Pmic_intrBitSet(pErrStat, PMIC_TPS6594X_VCCA_OV_INT);
            }

            if((regData & PMIC_INT_VMON_VCCA_UV_INT_MASK) != 0U)
            {
                Pmic_intrBitSet(pErrStat, PMIC_TPS6594X_VCCA_UV_INT);
            }
        }
    }

    /* Stop Critical Section */
    Pmic_criticalSectionStop(pPmicCoreHandle);

    return pmicStatus;
}

/*!
 * \brief  Function to decipher GPIO Error.
 */
static int32_t Pmic_tps6594x_getGpioErr(Pmic_CoreHandle_t *pPmicCoreHandle,
                                        uint8_t            regValue,
                                        Pmic_IrqStatus_t  *pErrStat)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData    = 0U;

    /* Start Critical Section */
    Pmic_criticalSectionStart(pPmicCoreHandle);

    /* Checking GPIO1_8 Bit field for INT_GPIO Register */
    if((regValue & PMIC_INT_GPIO_GPIO1_8_INT_MASK) != 0U)
    {
        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                            PMIC_INT_GPIO1_8_REGADDR,
                                            &regData);

       if((PMIC_ST_SUCCESS == pmicStatus) && 
          (0U != regData))
       {
           if((regData & PMIC_INT_GPIO1_8_GPIO1_INT_MASK) != 0U)
           {
               Pmic_intrBitSet(pErrStat, PMIC_TPS6594X_GPIO1_INT);
           }

           if((regData & PMIC_INT_GPIO1_8_GPIO2_INT_MASK) != 0U)
           {
               Pmic_intrBitSet(pErrStat, PMIC_TPS6594X_GPIO2_INT);
           }

           if((regData & PMIC_INT_GPIO1_8_GPIO3_INT_MASK) != 0U)
           {
               Pmic_intrBitSet(pErrStat, PMIC_TPS6594X_GPIO3_INT);
           }

           if((regData & PMIC_INT_GPIO1_8_GPIO4_INT_MASK) != 0U)
           {
               Pmic_intrBitSet(pErrStat, PMIC_TPS6594X_GPIO4_INT);
           }

           if((regData & PMIC_INT_GPIO1_8_GPIO5_INT_MASK) != 0U)
           {
               Pmic_intrBitSet(pErrStat, PMIC_TPS6594X_GPIO5_INT);
           }

           if((regData & PMIC_INT_GPIO1_8_GPIO6_INT_MASK) != 0U)
           {
               Pmic_intrBitSet(pErrStat, PMIC_TPS6594X_GPIO6_INT);
           }

           if((regData & PMIC_INT_GPIO1_8_GPIO7_INT_MASK) != 0U)
           {
               Pmic_intrBitSet(pErrStat, PMIC_TPS6594X_GPIO7_INT);
           }

           if((regData & PMIC_INT_GPIO1_8_GPIO8_INT_MASK) != 0U)
           {
                Pmic_intrBitSet(pErrStat, PMIC_TPS6594X_GPIO8_INT);
           }
        }
    }

    if((regValue & PMIC_INT_GPIO_GPIO9_INT_MASK) != 0U)
    {
        Pmic_intrBitSet(pErrStat, PMIC_TPS6594X_GPIO9_INT);
    }

    if((regValue & PMIC_INT_GPIO_GPIO10_INT_MASK) != 0U)
    {
        Pmic_intrBitSet(pErrStat, PMIC_TPS6594X_GPIO10_INT);
    }

    if((regValue & PMIC_INT_GPIO_GPIO11_INT_MASK) != 0U)
    {
        Pmic_intrBitSet(pErrStat, PMIC_TPS6594X_GPIO11_INT);
    }

    /* Stop Critical Section */
    Pmic_criticalSectionStop(pPmicCoreHandle);

    return pmicStatus;
}

/*!
 * \brief  Function to decipher STARTUP Error.
 */
static int32_t Pmic_tps6594x_getStartupErr(Pmic_CoreHandle_t *pPmicCoreHandle,
                                           uint8_t            regValue,
                                           Pmic_IrqStatus_t  *pErrStat)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData    = 0U;

    if((regValue & PMIC_INT_STARTUP_NPWRON_START_INT_MASK) != 0U)
    {
        Pmic_intrBitSet(pErrStat, PMIC_TPS6594X_NPWRON_START_INT);
    }

    if((regValue & PMIC_INT_STARTUP_ENABLE_INT_MASK) != 0U)
    {
        Pmic_intrBitSet(pErrStat, PMIC_TPS6594X_ENABLE_INT);
    }

    if((regValue & PMIC_INT_STARTUP_FSD_INT_MASK) != 0U)
    {
        Pmic_intrBitSet(pErrStat, PMIC_TPS6594X_FSD_INT);
    }

    if((regValue & PMIC_INT_STARTUP_RTC_INT_MASK) != 0U)
    {
        /* Start Critical Section */
        Pmic_criticalSectionStart(pPmicCoreHandle);

        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                            PMIC_RTC_STATUS_REGADDR,
                                            &regData);

        if(PMIC_ST_SUCCESS == pmicStatus)
        {
            if((regData & PMIC_RTC_STATUS_TIMER_MASK) != 0U)
            {
                Pmic_intrBitSet(pErrStat, PMIC_TPS6594X_RTC_TIMER_INT);
            }

            if((regData & PMIC_RTC_STATUS_ALARM_MASK) != 0U)
            {
                Pmic_intrBitSet(pErrStat, PMIC_TPS6594X_RTC_ALARM_INT);
            }
        }

        /* Stop Critical Section */
        Pmic_criticalSectionStop(pPmicCoreHandle);

    }

    return pmicStatus;
}

/*!
 * \brief  Function to decipher MISC Error.
 */
static void Pmic_tps6594x_getMiscErr(uint8_t            regValue,
                                     Pmic_IrqStatus_t  *pErrStat)
{
    if((regValue & PMIC_INT_MISC_BIST_PASS_INT_MASK) != 0U)
    {
        Pmic_intrBitSet(pErrStat, PMIC_TPS6594X_BIST_PASS_INT);
    }

    if((regValue & PMIC_INT_MISC_EXT_CLK_INT_MASK) != 0U)
    {
        Pmic_intrBitSet(pErrStat, PMIC_TPS6594X_EXT_CLK_INT );
    }

    if((regValue & PMIC_INT_MISC_TWARN_INT_MASK) != 0U)
    {
        Pmic_intrBitSet(pErrStat, PMIC_TPS6594X_TWARN_INT);
    }
}

/*!
 * \brief  Function to decipher MODERATE Error.
 */
static void Pmic_tps6594x_getModerateErr(uint8_t            regValue,
                                         Pmic_IrqStatus_t  *pErrStat)
{
    if((regValue & PMIC_INT_MODERATE_ERR_TSD_ORD_INT_MASK) != 0U)
    {
        Pmic_intrBitSet(pErrStat, PMIC_TPS6594X_TSD_ORD_INT);
    }

    if((regValue & PMIC_INT_MODERATE_ERR_BIST_FAIL_INT_MASK) != 0U)
    {
        Pmic_intrBitSet(pErrStat, PMIC_TPS6594X_BIST_FAIL_INT);
    }

    if((regValue & PMIC_INT_MODERATE_ERR_REG_CRC_ERR_INT_MASK) != 0U)
    {
        Pmic_intrBitSet(pErrStat, PMIC_TPS6594X_REG_CRC_ERR_INT);
    }

    if((regValue & PMIC_INT_MODERATE_ERR_RECOV_CNT_INT_MASK) != 0U)
    {
        Pmic_intrBitSet(pErrStat, PMIC_TPS6594X_RECOV_CNT_INT);
    }

    if((regValue & PMIC_INT_MODERATE_ERR_SPMI_ERR_INT_MASK) != 0U)
    {
        Pmic_intrBitSet(pErrStat, PMIC_TPS6594X_SPMI_ERR_INT);
    }

    if((regValue & PMIC_INT_MODERATE_ERR_NPWRON_LONG_INT_MASK) != 0U)
    {
        Pmic_intrBitSet(pErrStat, PMIC_TPS6594X_NPWRON_LONG_INT);
    }

    if((regValue & PMIC_INT_MODERATE_ERR_NINT_READBACK_INT_MASK) != 0U)
    {
        Pmic_intrBitSet(pErrStat, PMIC_TPS6594X_NINT_READBACK_INT);
    }

    if((regValue & PMIC_INT_MODERATE_ERR_NRSTOUT_READBACK_INT_MASK) != 0U)
    {
        Pmic_intrBitSet(pErrStat, PMIC_TPS6594X_NRSTOUT_READBACK_INT);
    }
}

/*!
 * \brief  Function to decipher SEVERE Error.
 */
static void Pmic_tps6594x_getSevereErr(uint8_t            regValue,
                                       Pmic_IrqStatus_t  *pErrStat)
{
    if((regValue & PMIC_INT_SEVERE_ERR_TSD_IMM_INT_MASK) != 0U)
    {
        Pmic_intrBitSet(pErrStat, PMIC_TPS6594X_TSD_IMM_INT);
    }

    if((regValue & PMIC_INT_SEVERE_ERR_VCCA_OVP_INT_MASK) != 0U)
    {
        Pmic_intrBitSet(pErrStat, PMIC_TPS6594X_VCCA_OVP_INT);
    }

    if((regValue & PMIC_INT_SEVERE_ERR_PFSM_ERR_INT_MASK) != 0U)
    {
        Pmic_intrBitSet(pErrStat, PMIC_TPS6594X_PFSM_ERR_INT);
    }
}

/*!
 * \brief  Function to decipher FSM Error.
 */
static int32_t Pmic_tps6594x_getFSMErr(Pmic_CoreHandle_t *pPmicCoreHandle,
                                       uint8_t            regValue,
                                       Pmic_IrqStatus_t  *pErrStat)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData    = 0U;

    /* Start Critical Section */
    Pmic_criticalSectionStart(pPmicCoreHandle);

    if((regValue & PMIC_INT_FSM_ERR_IMM_SHUTDOWN_INT_MASK) != 0U)
    {
        Pmic_intrBitSet(pErrStat, PMIC_TPS6594X_IMM_SHUTOWN_INT);
    }

    if((regValue & PMIC_INT_FSM_ERR_ORD_SHUTDOWN_INT_MASK) != 0U)
    {
        Pmic_intrBitSet(pErrStat, PMIC_TPS6594X_ORD_SHUTDOWN_INT);
    }

    if((regValue & PMIC_INT_FSM_ERR_MCU_PWR_ERR_INT_MASK) != 0U)
    {
        Pmic_intrBitSet(pErrStat, PMIC_TPS6594X_MCU_PWR_ERR_INT);
    }

    if((regValue & PMIC_INT_FSM_ERR_SOC_PWR_ERR_INT_MASK) != 0U)
    {
        Pmic_intrBitSet(pErrStat, PMIC_TPS6594X_SOC_PWR_ERR_INT);
    }

    if((regValue & PMIC_INT_FSM_ERR_COMM_ERR_INT_MASK) != 0U)
    {
        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                            PMIC_INT_COMM_ERR_REGADDR,
                                            &regData);

        if((PMIC_ST_SUCCESS == pmicStatus) && (0U != regData))
        {
            if((regData & PMIC_INT_COMM_ERR_COMM_FRM_ERR_INT_MASK) != 0U)
            {
                Pmic_intrBitSet(pErrStat, PMIC_TPS6594X_COMM_FRM_ERR_INT);
            }

            if((regData & PMIC_INT_COMM_ERR_COMM_CRC_ERR_INT_MASK) != 0U)
            {
                Pmic_intrBitSet(pErrStat, PMIC_TPS6594X_COMM_CRC_ERR_INT);
            }

            if((regData & PMIC_INT_COMM_ERR_COMM_ADR_ERR_INT_MASK) != 0U)
            {
                Pmic_intrBitSet(pErrStat, PMIC_TPS6594X_COMM_ADR_ERR_INT);
            }

            if((regData & PMIC_INT_COMM_ERR_I2C2_CRC_ERR_INT_MASK) != 0U)
            {
                Pmic_intrBitSet(pErrStat, PMIC_TPS6594X_I2C2_CRC_ERR_INT);
            }

            if((regData & PMIC_INT_COMM_ERR_I2C2_ADR_ERR_INT_MASK) != 0U)
            {
                Pmic_intrBitSet(pErrStat, PMIC_TPS6594X_I2C2_ADR_ERR_INT);
            }
        }
    }

    if((regValue & PMIC_INT_FSM_ERR_READBACK_ERR_INT_MASK) != 0U)
    {
        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                            PMIC_INT_READBACK_ERR_REGADDR,
                                            &regData);

        if((PMIC_ST_SUCCESS == pmicStatus) && (0U != regData))
        {
            if((regData & PMIC_INT_READBACK_ERR_EN_DRV_READBACK_INT_MASK)
                != 0U)
            {
                Pmic_intrBitSet(pErrStat, 
                                PMIC_TPS6594X_EN_DRV_READBACK_INT);
            }

            if((regData & PMIC_INT_READBACK_ERR_NRSTOUT_SOC_READBACK_INT_MASK)
                != 0U)
            {
                    Pmic_intrBitSet(pErrStat,
                                    PMIC_TPS6594X_NRSTOUT_SOC_READBACK_INT);
            }
        }
    }

    if((regValue & PMIC_INT_FSM_ERR_ESM_INT_MASK) != 0U)
    {
        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                            PMIC_INT_ESM_REGADDR,
                                            &regData);

        if((PMIC_ST_SUCCESS == pmicStatus) && (0U != regData))
        {
            if((regData & PMIC_INT_ESM_ESM_SOC_PIN_INT_MASK) != 0U)
            {
                Pmic_intrBitSet(pErrStat, PMIC_TPS6594X_ESM_SOC_PIN_INT);
            }

            if((regData & PMIC_INT_ESM_ESM_SOC_FAIL_INT_MASK) != 0U)
            {
                Pmic_intrBitSet(pErrStat, PMIC_TPS6594X_ESM_SOC_FAIL_INT);
            }

            if((regData & PMIC_INT_ESM_ESM_SOC_RST_INT_MASK) != 0U)
            {
                Pmic_intrBitSet(pErrStat, PMIC_TPS6594X_ESM_SOC_RST_INT);
            }

            if((regData & PMIC_INT_ESM_ESM_MCU_PIN_INT_MASK) != 0U)
            {
                Pmic_intrBitSet(pErrStat, PMIC_TPS6594X_ESM_MCU_PIN_INT);
            }

            if((regData & PMIC_INT_ESM_ESM_MCU_FAIL_INT_MASK) != 0U)
            {
                Pmic_intrBitSet(pErrStat, PMIC_TPS6594X_ESM_MCU_FAIL_INT);
            }

            if((regData & PMIC_INT_ESM_ESM_MCU_RST_INT_MASK) != 0U)
            {
                Pmic_intrBitSet(pErrStat, PMIC_TPS6594X_ESM_MCU_RST_INT);
            }
        }
    }

    if((regValue & PMIC_INT_FSM_ERR_WD_INT_MASK) != 0U)
    {
        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                            PMIC_WD_ERR_STATUS_REGADDR,
                                            &regData);

        if((PMIC_ST_SUCCESS == pmicStatus) &&
           ((regData & PMIC_INT_WD_ERR_MASK) != 0U))
        {
            if((regData & PMIC_WD_ERR_STATUS_WD_RST_INT_MASK) != 0U)
            {
                Pmic_intrBitSet(pErrStat, PMIC_TPS6594X_WD_RST_INT);
            }

            if((regData & PMIC_WD_ERR_STATUS_WD_FAIL_INT_MASK) != 0U)
            {
                Pmic_intrBitSet(pErrStat, PMIC_TPS6594X_WD_FAIL_INT);
            }

            if((regData & PMIC_WD_ERR_STATUS_WD_LONGWIN_TIMEOUT_INT_MASK)
                != 0U)
            {
                Pmic_intrBitSet(pErrStat,
                                PMIC_TPS6594X_WD_LONGWIN_TIMEOUT_INT);
            }
        }
    }

    /* Stop Critical Section */
    Pmic_criticalSectionStop(pPmicCoreHandle);

    return pmicStatus;
}

/*!
 * \brief  Function to decipher the L2 Error for TPS6594x Leo PMIC.
 */
int32_t Pmic_tps6594x_irqGetL2Error(Pmic_CoreHandle_t *pPmicCoreHandle,
                                    uint32_t           l1RegAddr,
                                    Pmic_IrqStatus_t  *pErrStat)
{
    int32_t pmicStatus  = PMIC_ST_SUCCESS;
    uint8_t regValue    = 0U;

    /* Start Critical Section */
    Pmic_criticalSectionStart(pPmicCoreHandle);

    /* Read the L1 register value */
    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        l1RegAddr,
                                        &regValue);

    /* Stop Critical Section */
    Pmic_criticalSectionStop(pPmicCoreHandle);

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        switch(l1RegAddr)
        {
            case PMIC_INT_BUCK_REGADDR:
                pmicStatus = Pmic_tps6594x_getBuckErr(pPmicCoreHandle,
                                                      regValue,
                                                      pErrStat);
                break;

            case PMIC_INT_LDO_VMON_REGADDR:
                pmicStatus = Pmic_tps6594x_getLdoVmonErr(pPmicCoreHandle,
                                                         regValue,
                                                         pErrStat);
                break;

            case PMIC_INT_GPIO_REGADDR:
                pmicStatus = Pmic_tps6594x_getGpioErr(pPmicCoreHandle,
                                                      regValue,
                                                      pErrStat);
                break;

            case PMIC_INT_STARTUP_REGADDR:
                pmicStatus = Pmic_tps6594x_getStartupErr(pPmicCoreHandle,
                                                         regValue,
                                                         pErrStat);
                break;

            case PMIC_INT_MISC_REGADDR:
                Pmic_tps6594x_getMiscErr(regValue, pErrStat);
                break;

            case PMIC_INT_MODERATE_ERR_REGADDR:
                Pmic_tps6594x_getModerateErr(regValue,
                                             pErrStat);
                break;

            case PMIC_INT_SEVERE_ERR_REGADDR:
                Pmic_tps6594x_getSevereErr(regValue,
                                           pErrStat);
                break;

            case PMIC_INT_FSM_ERR_REGADDR:
                pmicStatus = Pmic_tps6594x_getFSMErr(pPmicCoreHandle,
                                                     regValue,
                                                     pErrStat);
                break;

            default:
                pmicStatus = PMIC_ST_ERR_INV_INT;
                break;
        }
    }

    return pmicStatus;
}
