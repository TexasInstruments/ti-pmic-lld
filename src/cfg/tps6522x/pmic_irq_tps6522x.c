/******************************************************************************
 * Copyright (c) 2023 Texas Instruments Incorporated - http://www.ti.com
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
 * \file   pmic_irq_tps6522x.c
 *
 * \brief  This file contains the TPS6522x Burton PMIC Interrupt APIs definitions
 *         and structures.
 *
 */

#include "pmic_irq.h"
#include "pmic_core_priv.h"
#include "pmic_irq_priv.h"
#include "pmic_irq_tps6522x.h"
#include "pmic_irq_tps6522x_priv.h"
#include "pmic_power_priv.h"
#include "pmic_wdg_priv.h"

// clang-format off
/* PMIC TPS6522x Interrupt Configuration as per Pmic_tps6522x_IrqNum. */
static Pmic_IntrCfg_t gTps6522x_intCfg[] =
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
        PMIC_INT_FSM_ERR_REGADDR,
        PMIC_INT_FSM_ERR_I2C2_ERR_INT_SHIFT,
        PMIC_MASK_FSM_ERR_REGADDR,
        PMIC_MASK_FSM_ERR_I2C2_ERR_MASK_SHIFT
    },
    {
        PMIC_INT_FSM_ERR_REGADDR,
        PMIC_INT_FSM_ERR_COMM_ERR_INT_SHIFT,
        PMIC_MASK_FSM_ERR_REGADDR,
        PMIC_MASK_FSM_ERR_COMM_ERR_MASK_SHIFT
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
        PMIC_INT_SEVERE_ERR_BG_XMON_INT_SHIFT,
        PMIC_IRQ_INVALID_REGADDR,
        PMIC_IRQ_INVALID_BIT_SHIFT
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
        PMIC_INT_MISC_ADC_CONV_READY_INT_SHIFT,
        PMIC_MASK_MISC_REGADDR,
        PMIC_MASK_MISC_ADC_CONV_READY_MASK_SHIFT
    },
    {
        PMIC_INT_MISC_REGADDR,
        PMIC_INT_MISC_PB_RISE_INT_SHIFT,
        PMIC_MASK_MISC_REGADDR,
        PMIC_MASK_MISC_PB_RISE_MASK_SHIFT
    },
    {
        PMIC_INT_MISC_REGADDR,
        PMIC_INT_MISC_PB_FALL_INT_SHIFT,
        PMIC_MASK_MISC_REGADDR,
        PMIC_MASK_MISC_PB_FALL_MASK_SHIFT
    },
    {
        PMIC_INT_MISC_REGADDR,
        PMIC_INT_MISC_PB_LONG_INT_SHIFT,
        PMIC_MASK_MISC_REGADDR,
        PMIC_MASK_MISC_PB_LONG_MASK_SHIFT
    },
    {
        PMIC_INT_MISC_REGADDR,
        PMIC_INT_MISC_TWARN_INT_SHIFT,
        PMIC_MASK_MISC_REGADDR,
        PMIC_MASK_MISC_TWARN_MASK_SHIFT
    },
    {
        PMIC_INT_MISC_REGADDR,
        PMIC_INT_MISC_REG_UNLOCK_INT_SHIFT,
        PMIC_MASK_MISC_REGADDR,
        PMIC_MASK_MISC_REG_UNLOCK_MASK_SHIFT
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
        PMIC_INT_STARTUP_SOFT_REBOOT_INT_SHIFT,
        PMIC_MASK_STARTUP_REGADDR,
        PMIC_MASK_STARTUP_SOFT_REBOOT_MASK_SHIFT
    },
    {
        PMIC_INT_STARTUP_REGADDR,
        PMIC_INT_STARTUP_FSD_INT_SHIFT,
        PMIC_MASK_STARTUP_REGADDR,
        PMIC_MASK_STARTUP_FSD_MASK_SHIFT
    },
    {
        PMIC_INT_STARTUP_REGADDR,
        PMIC_INT_STARTUP_PB_SHORT_INT_SHIFT,
        PMIC_MASK_STARTUP_REGADDR,
        PMIC_MASK_STARTUP_PB_SHORT_MASK_SHIFT
    },
    {
        PMIC_INT_STARTUP_REGADDR,
        PMIC_INT_STARTUP_ENABLE_INT_SHIFT,
        PMIC_MASK_STARTUP_REGADDR,
        PMIC_MASK_STARTUP_ENABLE_MASK_SHIFT,
    },
    {
        PMIC_INT_STARTUP_REGADDR,
        PMIC_INT_STARTUP_VSENSE_INT_SHIFT,
        PMIC_MASK_STARTUP_REGADDR,
        PMIC_MASK_STARTUP_VSENSE_MASK_SHIFT,
    },
    {
        PMIC_INT_GPIO_REGADDR,
        PMIC_INT_GPIO_GPIO6_INT_SHIFT,
        PMIC_IRQ_INVALID_REGADDR,
        PMIC_IRQ_INVALID_BIT_SHIFT
    },
    {
        PMIC_INT_GPIO_REGADDR,
        PMIC_INT_GPIO_GPIO5_INT_SHIFT,
        PMIC_IRQ_INVALID_REGADDR,
        PMIC_IRQ_INVALID_BIT_SHIFT
    },
    {
        PMIC_INT_GPIO_REGADDR,
        PMIC_INT_GPIO_GPIO4_INT_SHIFT,
        PMIC_IRQ_INVALID_REGADDR,
        PMIC_IRQ_INVALID_BIT_SHIFT
    },
    {
        PMIC_INT_GPIO_REGADDR,
        PMIC_INT_GPIO_GPIO3_INT_SHIFT,
        PMIC_IRQ_INVALID_REGADDR,
        PMIC_IRQ_INVALID_BIT_SHIFT
    },
    {
        PMIC_INT_GPIO_REGADDR,
        PMIC_INT_GPIO_GPIO2_INT_SHIFT,
        PMIC_IRQ_INVALID_REGADDR,
        PMIC_IRQ_INVALID_BIT_SHIFT
    },
    {
        PMIC_INT_GPIO_REGADDR,
        PMIC_INT_GPIO_GPIO1_INT_SHIFT,
        PMIC_IRQ_INVALID_REGADDR,
        PMIC_IRQ_INVALID_BIT_SHIFT
    },
    {
        PMIC_INT_LDO_VMON_REGADDR,
        PMIC_INT_LDO_VMON_VMON2_UVOV_INT_SHIFT,
        PMIC_MASK_LDO_VMON_REGADDR,
        PMIC_MASK_LDO_VMON_VMON2_UVOV_MASK_SHIFT
    },
    {
        PMIC_INT_LDO_VMON_REGADDR,
        PMIC_INT_LDO_VMON_VMON1_UVOV_INT_SHIFT,
        PMIC_MASK_LDO_VMON_REGADDR,
        PMIC_MASK_LDO_VMON_VMON1_UVOV_MASK_SHIFT
    },
    {
        PMIC_INT_LDO_VMON_REGADDR,
        PMIC_INT_LDO_VMON_VCCA_UVOV_INT_SHIFT,
        PMIC_MASK_LDO_VMON_REGADDR,
        PMIC_MASK_LDO_VMON_VCCA_UVOV_MASK_SHIFT
    },
    {
        PMIC_INT_LDO_VMON_REGADDR,
        PMIC_INT_LDO_VMON_LDO3_UVOV_INT_SHIFT,
        PMIC_MASK_LDO_VMON_REGADDR,
        PMIC_MASK_LDO_VMON_LDO3_UVOV_MASK_SHIFT
    },
    {
        PMIC_INT_LDO_VMON_REGADDR,
        PMIC_INT_LDO_VMON_LDO2_UVOV_INT_SHIFT,
        PMIC_MASK_LDO_VMON_REGADDR,
        PMIC_MASK_LDO_VMON_LDO2_UVOV_MASK_SHIFT
    },
    {
        PMIC_INT_LDO_VMON_REGADDR,
        PMIC_INT_LDO_VMON_LDO1_UVOV_INT_SHIFT,
        PMIC_MASK_LDO_VMON_REGADDR,
        PMIC_MASK_LDO_VMON_LDO1_UVOV_MASK_SHIFT
    },
    {
        PMIC_INT_BUCK_REGADDR,
        PMIC_INT_BUCK4_UVOV_INT_SHIFT,
        PMIC_MASK_BUCK_REGADDR,
        PMIC_MASK_BUCK4_UVOV_MASK_SHIFT
    },
    {
        PMIC_INT_BUCK_REGADDR,
        PMIC_INT_BUCK3_UVOV_INT_SHIFT,
        PMIC_MASK_BUCK_REGADDR,
        PMIC_MASK_BUCK3_UVOV_MASK_SHIFT
    },
    {
        PMIC_INT_BUCK_REGADDR,
        PMIC_INT_BUCK2_UVOV_INT_SHIFT,
        PMIC_MASK_BUCK_REGADDR,
        PMIC_MASK_BUCK2_UVOV_MASK_SHIFT
    },
    {
        PMIC_INT_BUCK_REGADDR,
        PMIC_INT_BUCK1_UVOV_INT_SHIFT,
        PMIC_MASK_BUCK_REGADDR,
        PMIC_MASK_BUCK1_UVOV_MASK_SHIFT
    }
};

// clang-format off
/*  PMIC TPS6522x GPIO Interrupt Mask Configuration as per Pmic_IrqGpioNum. */
static Pmic_GpioIntrTypeCfg_t tps6522x_gpioIntrCfg[] =
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
    }
};
// clang-format on

void pmic_get_tps6522x_intrCfg(Pmic_IntrCfg_t **pIntrCfg)
{
    *pIntrCfg = gTps6522x_intCfg;
}

void pmic_get_tps6522x_intrGpioCfg(Pmic_GpioIntrTypeCfg_t **pGpioIntrCfg)
{
    *pGpioIntrCfg = tps6522x_gpioIntrCfg;
}

/*!
 * \brief  Function to decipher BUCK Error
 */
static void Pmic_tps6522x_getBuckErr(Pmic_CoreHandle_t *pPmicCoreHandle, uint8_t regValue, Pmic_IrqStatus_t *pErrStat)
{
    if ((regValue & PMIC_INT_BUCK_BUCK1_UVOV_INT_MASK) != 0U)
    {
        Pmic_intrBitSet(pErrStat, PMIC_TPS6522X_BUCK1_UVOV_INT);
    }

    if ((regValue & PMIC_INT_BUCK_BUCK2_UVOV_INT_MASK) != 0U)
    {
        Pmic_intrBitSet(pErrStat, PMIC_TPS6522X_BUCK2_UVOV_INT);
    }

    if ((regValue & PMIC_INT_BUCK_BUCK3_UVOV_INT_MASK) != 0U)
    {
        Pmic_intrBitSet(pErrStat, PMIC_TPS6522X_BUCK3_UVOV_INT);
    }

    if ((regValue & PMIC_INT_BUCK_BUCK4_UVOV_INT_MASK) != 0U)
    {
        Pmic_intrBitSet(pErrStat, PMIC_TPS6522X_BUCK4_UVOV_INT);
    }
}

/*!
 * \brief  Function to decipher LDO_VMON Error
 */
static void
Pmic_tps6522x_getLdoVmonErr(Pmic_CoreHandle_t *pPmicCoreHandle, uint8_t regValue, Pmic_IrqStatus_t *pErrStat)
{
    if ((regValue & PMIC_INT_LDO_VMON_LDO1_UVOV_INT_MASK) != 0U)
    {
        Pmic_intrBitSet(pErrStat, PMIC_TPS6522X_LDO1_UVOV_INT);
    }

    if ((regValue & PMIC_INT_LDO_VMON_LDO2_UVOV_INT_MASK) != 0U)
    {
        Pmic_intrBitSet(pErrStat, PMIC_TPS6522X_LDO2_UVOV_INT);
    }

    if ((regValue & PMIC_INT_LDO_VMON_LDO3_UVOV_INT_MASK) != 0U)
    {
        Pmic_intrBitSet(pErrStat, PMIC_TPS6522X_LDO3_UVOV_INT);
    }

    if ((regValue & PMIC_INT_LDO_VMON_VCCA_UVOV_INT_MASK) != 0U)
    {
        Pmic_intrBitSet(pErrStat, PMIC_TPS6522X_VCCA_UVOV_INT);
    }

    if ((regValue & PMIC_INT_LDO_VMON_VMON1_UVOV_INT_MASK) != 0U)
    {
        Pmic_intrBitSet(pErrStat, PMIC_TPS6522X_VMON1_UVOV_INT);
    }

    if ((regValue & PMIC_INT_LDO_VMON_VMON2_UVOV_INT_MASK) != 0U)
    {
        Pmic_intrBitSet(pErrStat, PMIC_TPS6522X_VMON2_UVOV_INT);
    }
}

/*!
 * \brief  Function to decipher GPIO Error
 */
static void Pmic_tps6522x_getGpioErr(Pmic_CoreHandle_t *pPmicCoreHandle, uint8_t regValue, Pmic_IrqStatus_t *pErrStat)
{
    if ((regValue & PMIC_INT_GPIO_GPIO1_INT_MASK) != 0U)
    {
        Pmic_intrBitSet(pErrStat, PMIC_TPS6522X_GPIO1_INT);
    }

    if ((regValue & PMIC_INT_GPIO_GPIO2_INT_MASK) != 0U)
    {
        Pmic_intrBitSet(pErrStat, PMIC_TPS6522X_GPIO2_INT);
    }

    if ((regValue & PMIC_INT_GPIO_GPIO3_INT_MASK) != 0U)
    {
        Pmic_intrBitSet(pErrStat, PMIC_TPS6522X_GPIO3_INT);
    }

    if ((regValue & PMIC_INT_GPIO_GPIO4_INT_MASK) != 0U)
    {
        Pmic_intrBitSet(pErrStat, PMIC_TPS6522X_GPIO4_INT);
    }

    if ((regValue & PMIC_INT_GPIO_GPIO5_INT_MASK) != 0U)
    {
        Pmic_intrBitSet(pErrStat, PMIC_TPS6522X_GPIO5_INT);
    }

    if ((regValue & PMIC_INT_GPIO_GPIO6_INT_MASK) != 0U)
    {
        Pmic_intrBitSet(pErrStat, PMIC_TPS6522X_GPIO6_INT);
    }
}

/*!
 * \brief  Function to decipher STARTUP Error.
 */
static void
Pmic_tps6522x_getStartupErr(Pmic_CoreHandle_t *pPmicCoreHandle, uint8_t regValue, Pmic_IrqStatus_t *pErrStat)
{
    if ((regValue & PMIC_INT_STARTUP_VSENSE_INT_MASK) != 0U)
    {
        Pmic_intrBitSet(pErrStat, PMIC_TPS6522X_VSENSE_INT);
    }

    if ((regValue & PMIC_INT_STARTUP_ENABLE_INT_MASK) != 0U)
    {
        Pmic_intrBitSet(pErrStat, PMIC_TPS6522X_ENABLE_INT);
    }

    if ((regValue & PMIC_INT_STARTUP_PB_SHORT_INT_MASK) != 0U)
    {
        Pmic_intrBitSet(pErrStat, PMIC_TPS6522X_PB_SHORT_INT);
    }

    if ((regValue & PMIC_INT_STARTUP_FSD_INT_MASK) != 0U)
    {
        Pmic_intrBitSet(pErrStat, PMIC_TPS6522X_FSD_INT);
    }

    if ((regValue & PMIC_INT_STARTUP_SOFT_REBOOT_INT_MASK) != 0U)
    {
        Pmic_intrBitSet(pErrStat, PMIC_TPS6522X_SOFT_REBOOT_INT);
    }
}

/*!
 * \brief  Function to decipher MISC Error.
 */
static void Pmic_tps6522x_getMiscErr(uint8_t regValue, Pmic_IrqStatus_t *pErrStat)
{
    if ((regValue & PMIC_INT_MISC_BIST_PASS_INT_MASK) != 0U)
    {
        Pmic_intrBitSet(pErrStat, PMIC_TPS6522X_BIST_PASS_INT);
    }

    if ((regValue & PMIC_INT_MISC_EXT_CLK_INT_MASK) != 0U)
    {
        Pmic_intrBitSet(pErrStat, PMIC_TPS6522X_EXT_CLK_INT);
    }

    if ((regValue & PMIC_INT_MISC_REG_UNLOCK_INT_MASK) != 0U)
    {
        Pmic_intrBitSet(pErrStat, PMIC_TPS6522X_REG_UNLOCK_INT);
    }

    if ((regValue & PMIC_INT_MISC_TWARN_INT_MASK) != 0U)
    {
        Pmic_intrBitSet(pErrStat, PMIC_TPS6522X_TWARN_INT);
    }

    if ((regValue & PMIC_INT_MISC_PB_LONG_INT_MASK) != 0U)
    {
        Pmic_intrBitSet(pErrStat, PMIC_TPS6522X_PB_LONG_INT);
    }

    if ((regValue & PMIC_INT_MISC_PB_FALL_INT_MASK) != 0U)
    {
        Pmic_intrBitSet(pErrStat, PMIC_TPS6522X_PB_FALL_INT);
    }

    if ((regValue & PMIC_INT_MISC_PB_RISE_INT_MASK) != 0U)
    {
        Pmic_intrBitSet(pErrStat, PMIC_TPS6522X_PB_RISE_INT);
    }

    if ((regValue & PMIC_INT_MISC_ADC_CONV_READY_INT_MASK) != 0U)
    {
        Pmic_intrBitSet(pErrStat, PMIC_TPS6522X_ADC_CONV_READY_INT);
    }
}

/*!
 * \brief  Function to decipher MODERATE Error
 *
 */
static void
Pmic_tps6522x_getModerateErr(const Pmic_CoreHandle_t *pPmicCoreHandle, uint8_t regValue, Pmic_IrqStatus_t *pErrStat)
{
    if ((regValue & PMIC_INT_MODERATE_ERR_TSD_ORD_INT_MASK) != 0U)
    {
        Pmic_intrBitSet(pErrStat, PMIC_TPS6522X_TSD_ORD_INT);
    }

    if ((regValue & PMIC_INT_MODERATE_ERR_BIST_FAIL_INT_MASK) != 0U)
    {
        Pmic_intrBitSet(pErrStat, PMIC_TPS6522X_BIST_FAIL_INT);
    }

    if ((regValue & PMIC_INT_MODERATE_ERR_REG_CRC_ERR_INT_MASK) != 0U)
    {
        Pmic_intrBitSet(pErrStat, PMIC_TPS6522X_REG_CRC_ERR_INT);
    }

    if ((regValue & PMIC_INT_MODERATE_ERR_RECOV_CNT_INT_MASK) != 0U)
    {
        Pmic_intrBitSet(pErrStat, PMIC_TPS6522X_RECOV_CNT_INT);
    }
}

/*!
 * \brief  Function to decipher SEVERE Error
 */
static void
Pmic_tps6522x_getSevereErr(const Pmic_CoreHandle_t *pPmicCoreHandle, uint8_t regValue, Pmic_IrqStatus_t *pErrStat)
{
    if ((regValue & PMIC_INT_SEVERE_ERR_TSD_IMM_INT_MASK) != 0U)
    {
        Pmic_intrBitSet(pErrStat, PMIC_TPS6522X_TSD_IMM_INT);
    }

    if ((regValue & PMIC_INT_SEVERE_ERR_VCCA_OVP_INT_MASK) != 0U)
    {
        Pmic_intrBitSet(pErrStat, PMIC_TPS6522X_VCCA_OVP_INT);
    }

    if ((regValue & PMIC_INT_SEVERE_ERR_PFSM_ERR_INT_MASK) != 0U)
    {
        Pmic_intrBitSet(pErrStat, PMIC_TPS6522X_PFSM_ERR_INT);
    }

    if ((regValue & PMIC_INT_SEVERE_ERR_BG_XMON_INT_MASK) != 0U)
    {
        Pmic_intrBitSet(pErrStat, PMIC_TPS6522X_BG_XMON_INT);
    }
}

/*!
 * \brief  Function to check FSM - ESM Error
 */
static void Pmic_tps6522x_getFsmEsmErr(Pmic_CoreHandle_t *pPmicCoreHandle, uint8_t regValue, Pmic_IrqStatus_t *pErrStat)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    if ((regValue & PMIC_INT_FSM_ERR_ESM_INT_MASK) != 0U)
    {
        /* Start Critical Section */
        Pmic_criticalSectionStart(pPmicCoreHandle);

        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_INT_ESM_REGADDR, &regData);

        /* Stop Critical Section */
        Pmic_criticalSectionStop(pPmicCoreHandle);

        if ((PMIC_ST_SUCCESS == pmicStatus) && (0U != regData))
        {
            if ((regData & PMIC_INT_ESM_ESM_MCU_PIN_INT_MASK) != 0U)
            {
                Pmic_intrBitSet(pErrStat, PMIC_TPS6522X_ESM_MCU_PIN_INT);
            }

            if ((regData & PMIC_INT_ESM_ESM_MCU_FAIL_INT_MASK) != 0U)
            {
                Pmic_intrBitSet(pErrStat, PMIC_TPS6522X_ESM_MCU_FAIL_INT);
            }

            if ((regData & PMIC_INT_ESM_ESM_MCU_RST_INT_MASK) != 0U)
            {
                Pmic_intrBitSet(pErrStat, PMIC_TPS6522X_ESM_MCU_RST_INT);
            }
        }
    }
}

/*!
 * \brief  Function to decipher FSM Error.
 */
static int32_t Pmic_tps6522x_getFSMErr(Pmic_CoreHandle_t *pPmicCoreHandle, uint8_t regValue, Pmic_IrqStatus_t *pErrStat)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    if ((regValue & PMIC_INT_FSM_ERR_IMM_SHUTDOWN_INT_MASK) != 0U)
    {
        Pmic_intrBitSet(pErrStat, PMIC_TPS6522X_IMM_SHUTOWN_INT);
    }

    if ((regValue & PMIC_INT_FSM_ERR_ORD_SHUTDOWN_INT_MASK) != 0U)
    {
        Pmic_intrBitSet(pErrStat, PMIC_TPS6522X_ORD_SHUTDOWN_INT);
    }

    if ((regValue & PMIC_INT_FSM_ERR_MCU_PWR_ERR_INT_MASK) != 0U)
    {
        Pmic_intrBitSet(pErrStat, PMIC_TPS6522X_MCU_PWR_ERR_INT);
    }

    if ((regValue & PMIC_INT_FSM_ERR_SOC_PWR_ERR_INT_MASK) != 0U)
    {
        Pmic_intrBitSet(pErrStat, PMIC_TPS6522X_SOC_PWR_ERR_INT);
    }

    if ((regValue & PMIC_INT_FSM_ERR_COMM_ERR_INT_MASK) != 0U)
    {
        Pmic_intrBitSet(pErrStat, PMIC_TPS6522X_COMM_ERR_INT);
    }

    if ((regValue & PMIC_INT_FSM_ERR_I2C2_ERR_INT_MASK) != 0U)
    {
        Pmic_intrBitSet(pErrStat, PMIC_TPS6522X_I2C2_ERR_INT);
    }

    /* Check/decipher FSM Error for PMIC_INT_ESM Register Bit */
    Pmic_tps6522x_getFsmEsmErr(pPmicCoreHandle, regValue, pErrStat);

    /* Check/decipher FSM Error for IRQ Mask Bit */
    if ((regValue & PMIC_INT_FSM_ERR_WD_INT_MASK) != 0U)
    {
        /* Start Critical Section */
        Pmic_criticalSectionStart(pPmicCoreHandle);

        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_WD_ERR_STATUS_REGADDR, &regData);

        /* Stop Critical Section */
        Pmic_criticalSectionStop(pPmicCoreHandle);

        if ((PMIC_ST_SUCCESS == pmicStatus) && ((regData & PMIC_INT_WD_ERR_MASK) != 0U))
        {
            if ((regData & PMIC_WD_ERR_STATUS_WD_RST_INT_MASK) != 0U)
            {
                Pmic_intrBitSet(pErrStat, PMIC_TPS6522X_WD_RST_INT);
            }

            if ((regData & PMIC_WD_ERR_STATUS_WD_FAIL_INT_MASK) != 0U)
            {
                Pmic_intrBitSet(pErrStat, PMIC_TPS6522X_WD_FAIL_INT);
            }

            if ((regData & PMIC_WD_ERR_STATUS_WD_LONGWIN_TIMEOUT_INT_MASK) != 0U)
            {
                Pmic_intrBitSet(pErrStat, PMIC_TPS6522X_WD_LONGWIN_TIMEOUT_INT);
            }
        }
    }

    return pmicStatus;
}

/*!
 * \brief  Call Function to decipher the Startup, Miscellaneous, Moderate,
 *         Severe, FSM Error
 */
static int32_t Pmic_tps6522x_getStartupMiscModerateSevereFsmErr(Pmic_CoreHandle_t *pPmicCoreHandle,
                                                                Pmic_IrqStatus_t  *pErrStat,
                                                                uint16_t           l1RegAddr,
                                                                uint8_t            regValue)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    switch (l1RegAddr)
    {
        case PMIC_INT_STARTUP_REGADDR:
            Pmic_tps6522x_getStartupErr(pPmicCoreHandle, regValue, pErrStat);
            break;

        case PMIC_INT_MISC_REGADDR:
            Pmic_tps6522x_getMiscErr(regValue, pErrStat);
            break;

        case PMIC_INT_MODERATE_ERR_REGADDR:
            Pmic_tps6522x_getModerateErr(pPmicCoreHandle, regValue, pErrStat);
            break;

        case PMIC_INT_SEVERE_ERR_REGADDR:
            Pmic_tps6522x_getSevereErr(pPmicCoreHandle, regValue, pErrStat);
            break;

        default:
            pmicStatus = Pmic_tps6522x_getFSMErr(pPmicCoreHandle, regValue, pErrStat);
            break;
    }

    return pmicStatus;
}

int32_t Pmic_tps6522x_irqGetL2Error(Pmic_CoreHandle_t *pPmicCoreHandle, uint16_t l1RegAddr, Pmic_IrqStatus_t *pErrStat)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regValue = 0U;

    /* Start Critical Section */
    Pmic_criticalSectionStart(pPmicCoreHandle);

    /* Read the L1 register value */
    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, l1RegAddr, &regValue);

    /* Stop Critical Section */
    Pmic_criticalSectionStop(pPmicCoreHandle);

    if (PMIC_ST_SUCCESS == pmicStatus)
    {
        switch (l1RegAddr)
        {
            case PMIC_INT_BUCK_REGADDR:
                Pmic_tps6522x_getBuckErr(pPmicCoreHandle, regValue, pErrStat);
                break;

            case PMIC_INT_LDO_VMON_REGADDR:
                Pmic_tps6522x_getLdoVmonErr(pPmicCoreHandle, regValue, pErrStat);
                break;

            case PMIC_INT_GPIO_REGADDR:
                Pmic_tps6522x_getGpioErr(pPmicCoreHandle, regValue, pErrStat);
                break;

            case PMIC_INT_STARTUP_REGADDR:
            case PMIC_INT_MISC_REGADDR:
            case PMIC_INT_MODERATE_ERR_REGADDR:
            case PMIC_INT_SEVERE_ERR_REGADDR:
            case PMIC_INT_FSM_ERR_REGADDR:
                pmicStatus =
                    Pmic_tps6522x_getStartupMiscModerateSevereFsmErr(pPmicCoreHandle, pErrStat, l1RegAddr, regValue);
                break;

            default:
                pmicStatus = PMIC_ST_ERR_INV_INT;
                break;
        }
    }

    return pmicStatus;
}
