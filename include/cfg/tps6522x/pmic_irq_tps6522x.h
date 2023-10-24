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
 *  \addtogroup DRV_PMIC_IRQ_MODULE
 *
 *  @{
 */

/**
 * \file   pmic_irq_tps6522x.h
 *
 * \brief  TPS6522x Burton PMIC IRQ Driver API/interface file.
 *
 */

#ifndef PMIC_IRQ_TPS6522X_H_
#define PMIC_IRQ_TPS6522X_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#ifdef __cplusplus
extern "C"
{
#endif

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/**
 *  \anchor Pmic_tps6522x_IrqNum
 *  \name PMIC IRQ Interrupt values for TPS6522x BURTON PMIC Device.
 *
 *  @{
 */
/*! PMIC WDG RESET Interrupt */
#define PMIC_TPS6522X_WD_RST_INT              (0U)
/*! PMIC WDG FAIL Interrupt */
#define PMIC_TPS6522X_WD_FAIL_INT             (1U)
/*! PMIC WDG LONG WINDOW TIMEOUT Interrupt */
#define PMIC_TPS6522X_WD_LONGWIN_TIMEOUT_INT  (2U)

/*! PMIC ESM MCU RESET Interrupt */
#define PMIC_TPS6522X_ESM_MCU_RST_INT         (3U)
/*! PMIC ESM MCU FAIL Interrupt */
#define PMIC_TPS6522X_ESM_MCU_FAIL_INT        (4U)
/*! PMIC ESM MCU PIN Interrupt */
#define PMIC_TPS6522X_ESM_MCU_PIN_INT         (5U)

/*! PMIC I2C2 ERROR Interrupt */
#define PMIC_TPS6522X_I2C2_ERR_INT            (6U)
/*! PMIC COMMUNICATION ERROR Interrupt */
#define PMIC_TPS6522X_COMM_ERR_INT            (7U)
/*! PMIC SOC POWER ERR Interrupt */
#define PMIC_TPS6522X_SOC_PWR_ERR_INT         (8U)
/*! PMIC MCU POWER ERR Interrupt */
#define PMIC_TPS6522X_MCU_PWR_ERR_INT         (9U)
/*! PMIC ORDERLY SHUTDOWN Interrupt */
#define PMIC_TPS6522X_ORD_SHUTDOWN_INT        (10U)
/*! PMIC IMMEDIATE SHUTDOWN Interrupt */
#define PMIC_TPS6522X_IMM_SHUTOWN_INT         (11U)

/*! PMIC Bandgap Cross Monitor Interrupt */
#define PMIC_TPS6522X_BG_XMON_INT             (12U)
/*! PMIC PFSM ERROR Interrupt */
#define PMIC_TPS6522X_PFSM_ERR_INT            (13U)
/*! PMIC VCCA Over-Voltage Interrupt */
#define PMIC_TPS6522X_VCCA_OVP_INT            (14U)
/*! PMIC Thermal Threshold Immediate Shutdown Interrupt */
#define PMIC_TPS6522X_TSD_IMM_INT             (15U)

/*! PMIC RECOV_CNT Threshold Interrupt */
#define PMIC_TPS6522X_RECOV_CNT_INT           (16U)
/*! PMIC Register CRC Error Interrupt */
#define PMIC_TPS6522X_REG_CRC_ERR_INT         (17U)
/*! PMIC LBIST/ABIST Error Interrupt */
#define PMIC_TPS6522X_BIST_FAIL_INT           (18U)
/*! PMIC Thermal Shutdown Orderly Interrupt */
#define PMIC_TPS6522X_TSD_ORD_INT             (19U)

/*! PMIC ADC Conversion Ready Interrupt */
#define PMIC_TPS6522X_ADC_CONV_READY_INT      (20U)
/*! PMIC Push Button Rise Interrupt */
#define PMIC_TPS6522X_PB_RISE_INT             (21U)
/*! PMIC Push Button Fall Interrupt */
#define PMIC_TPS6522X_PB_FALL_INT             (22U)
/*! PMIC Push Button Long Press Interrupt */
#define PMIC_TPS6522X_PB_LONG_INT             (23U)
/*! PMIC Thermal Warning Interrupt */
#define PMIC_TPS6522X_TWARN_INT               (24U)
/*! PMIC Register Unlock Interrupt */
#define PMIC_TPS6522X_REG_UNLOCK_INT          (25U)
/*! PMIC External Clock Interrupt */
#define PMIC_TPS6522X_EXT_CLK_INT             (26U)
/*! PMIC BIST PASS Interrupt */
#define PMIC_TPS6522X_BIST_PASS_INT           (27U)

/*! PMIC Soft Reboot Interrupt */
#define PMIC_TPS6522X_SOFT_REBOOT_INT         (28U)
/*! PMIC First Supply Detection Interrupt */
#define PMIC_TPS6522X_FSD_INT                 (29U)
/*! PMIC Push Button Short Press Interrupt */
#define PMIC_TPS6522X_PB_SHORT_INT            (30U)
/*! PMIC ENABLE Interrupt */
#define PMIC_TPS6522X_ENABLE_INT              (31U)
/*! PMIC VSENSE interrupt */
#define PMIC_TPS6522X_VSENSE_INT              (32U)

/*! PMIC GPIO PIN 6 Interrupt */
#define PMIC_TPS6522X_GPIO6_INT               (33U)
/*! PMIC GPIO PIN 5 Interrupt */
#define PMIC_TPS6522X_GPIO5_INT               (34U)
/*! PMIC GPIO PIN 4 Interrupt */
#define PMIC_TPS6522X_GPIO4_INT               (35U)
/*! PMIC GPIO PIN 3 Interrupt */
#define PMIC_TPS6522X_GPIO3_INT               (36U)
/*! PMIC GPIO PIN 2 Interrupt */
#define PMIC_TPS6522X_GPIO2_INT               (37U)
/*! PMIC GPIO PIN 1 Interrupt */
#define PMIC_TPS6522X_GPIO1_INT               (38U)

/*! PMIC VMON2 Under/Over-voltage detection Interrupt */
#define PMIC_TPS6522X_VMON2_UVOV_INT          (39U)
/*! PMIC VMON1 Under/Over-voltage detection Interrupt */
#define PMIC_TPS6522X_VMON1_UVOV_INT          (40U)
/*! PMIC VCCA Under/Over-voltage detection Interrupt */
#define PMIC_TPS6522X_VCCA_UVOV_INT           (41U)
/*! PMIC LDO3 Under/Over-voltage detection Interrupt */
#define PMIC_TPS6522X_LDO3_UVOV_INT           (42U)
/*! PMIC LDO2 Under/Over-voltage detection Interrupt */
#define PMIC_TPS6522X_LDO2_UVOV_INT           (43U)
/*! PMIC LDO1 Under/Over-voltage detection Interrupt */
#define PMIC_TPS6522X_LDO1_UVOV_INT           (44U)

/*! PMIC BUCK4 Under/Over-voltage detection Interrupt */
#define PMIC_TPS6522X_BUCK4_UVOV_INT          (45U)
/*! PMIC BUCK3 Under/Over-voltage detection Interrupt */
#define PMIC_TPS6522X_BUCK3_UVOV_INT          (46U)
/*! PMIC BUCK2 Under/Over-voltage detection Interrupt */
#define PMIC_TPS6522X_BUCK2_UVOV_INT          (47U)
/*! PMIC BUCK1 Under/Over-voltage detection Interrupt */
#define PMIC_TPS6522X_BUCK1_UVOV_INT          (48U)

/*! PMIC Max Interrupt Number */
#define PMIC_TPS6522X_IRQ_MAX_NUM             (49U)

/* @} */

/**
 *  \anchor Pmic_tps6522x_IrqGpioNum
 *  \name PMIC GPIO Interrupt Mask values for tps6522x.
 *
 *  @{
 */
#define PMIC_TPS6522X_IRQ_GPIO_1_INT_MASK_NUM (0U)
#define PMIC_TPS6522X_IRQ_GPIO_2_INT_MASK_NUM (1U)
#define PMIC_TPS6522X_IRQ_GPIO_3_INT_MASK_NUM (2U)
#define PMIC_TPS6522X_IRQ_GPIO_4_INT_MASK_NUM (3U)
#define PMIC_TPS6522X_IRQ_GPIO_5_INT_MASK_NUM (4U)
#define PMIC_TPS6522X_IRQ_GPIO_6_INT_MASK_NUM (5U)

/* @} */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* PMIC_IRQ_TPS6522X_H_ */

/* @} */
