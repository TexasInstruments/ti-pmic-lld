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
 * \file   pmic_irq_tps6522x_priv.h
 *
 * \brief  The macro definitions, structures and function prototypes for
 *         configuring PMIC IRQ.
 */

#ifndef PMIC_IRQ_TPS6522X_PRIV_H_
#define PMIC_IRQ_TPS6522X_PRIV_H_

/* ========================================================================= */
/*                              Include Files                                */
/* ========================================================================= */

#ifdef __cplusplus
extern "C"
{
#endif

/* ========================================================================= */
/*                           Macros & Typedefs                               */
/* ========================================================================= */

/*!
 * \brief LDO_VMON interrupt mask register address
 */
#define PMIC_MASK_LDO_VMON_REGADDR               (0x4CU)

/*!
 * \brief BUCK interrupt mask register address
 */
#define PMIC_MASK_BUCK_REGADDR                   (0x49U)

/*! PMIC INT_BUCK Register Bit Masks */
#define PMIC_INT_BUCK_BUCK1_UVOV_INT_MASK        (0x01U)
#define PMIC_INT_BUCK_BUCK2_UVOV_INT_MASK        (0x02U)
#define PMIC_INT_BUCK_BUCK3_UVOV_INT_MASK        (0x04U)
#define PMIC_INT_BUCK_BUCK4_UVOV_INT_MASK        (0x08U)

/*! PMIC INT_LDO_VMON Register Bit Masks */
#define PMIC_INT_LDO_VMON_LDO1_UVOV_INT_MASK     (0x01U)
#define PMIC_INT_LDO_VMON_LDO2_UVOV_INT_MASK     (0x02U)
#define PMIC_INT_LDO_VMON_LDO3_UVOV_INT_MASK     (0x04U)
#define PMIC_INT_LDO_VMON_VCCA_UVOV_INT_MASK     (0x10U)
#define PMIC_INT_LDO_VMON_VMON1_UVOV_INT_MASK    (0x20U)
#define PMIC_INT_LDO_VMON_VMON2_UVOV_INT_MASK    (0x40U)

/*! PMIC INT_GPIO Register Bit Masks */
#define PMIC_INT_GPIO_GPIO1_INT_MASK             (0x01U)
#define PMIC_INT_GPIO_GPIO2_INT_MASK             (0x02U)
#define PMIC_INT_GPIO_GPIO3_INT_MASK             (0x04U)
#define PMIC_INT_GPIO_GPIO4_INT_MASK             (0x08U)
#define PMIC_INT_GPIO_GPIO5_INT_MASK             (0x10U)
#define PMIC_INT_GPIO_GPIO6_INT_MASK             (0x20U)

/*! PMIC INT_STARTUP Register Bit Masks */
#define PMIC_INT_STARTUP_VSENSE_INT_MASK         (0x01U)
#define PMIC_INT_STARTUP_PB_SHORT_INT_MASK       (0x04U)

/*! PMIC INT_MISC Register Bit Masks */
#define PMIC_INT_MISC_REG_UNLOCK_INT_MASK        (0x04U)
#define PMIC_INT_MISC_PB_LONG_INT_MASK           (0x10U)
#define PMIC_INT_MISC_PB_FALL_INT_MASK           (0x20U)
#define PMIC_INT_MISC_PB_RISE_INT_MASK           (0x40U)
#define PMIC_INT_MISC_ADC_CONV_READY_INT_MASK    (0x80U)

/*! PMIC INT_FSM_ERR Register Bit Masks */
#define PMIC_INT_FSM_ERR_I2C2_ERR_INT_MASK       (0x20U)

/*! PMIC_INT_MISC Register Bit Positions */
#define PMIC_INT_MISC_ADC_CONV_READY_INT_SHIFT   (0x7U)
#define PMIC_INT_MISC_PB_RISE_INT_SHIFT          (0x6U)
#define PMIC_INT_MISC_PB_FALL_INT_SHIFT          (0x5U)
#define PMIC_INT_MISC_PB_LONG_INT_SHIFT          (0x4U)
#define PMIC_INT_MISC_REG_UNLOCK_INT_SHIFT       (0x2U)

/*! PMIC_INT_STARTUP Register Bit Positions */
#define PMIC_INT_STARTUP_PB_SHORT_INT_SHIFT      (0x2U)
#define PMIC_INT_STARTUP_VSENSE_INT_SHIFT        (0x0U)

/*! PMIC_MASK_MISC Register Bit Positions */
#define PMIC_MASK_MISC_ADC_CONV_READY_MASK_SHIFT (0x7U)
#define PMIC_MASK_MISC_PB_RISE_MASK_SHIFT        (0x6U)
#define PMIC_MASK_MISC_PB_FALL_MASK_SHIFT        (0x5U)
#define PMIC_MASK_MISC_PB_LONG_MASK_SHIFT        (0x4U)
#define PMIC_MASK_MISC_REG_UNLOCK_MASK_SHIFT     (0x2U)

/*! PMIC_MASK_STARTUP Register Bit Positions */
#define PMIC_MASK_STARTUP_PB_SHORT_MASK_SHIFT    (0x2U)
#define PMIC_MASK_STARTUP_VSENSE_MASK_SHIFT      (0x0U)

/*! PMIC_INT_GPIO Register Bit Positions */
#define PMIC_INT_GPIO_GPIO6_INT_SHIFT            (0x5U)
#define PMIC_INT_GPIO_GPIO5_INT_SHIFT            (0x4U)
#define PMIC_INT_GPIO_GPIO4_INT_SHIFT            (0x3U)
#define PMIC_INT_GPIO_GPIO3_INT_SHIFT            (0x2U)
#define PMIC_INT_GPIO_GPIO2_INT_SHIFT            (0x1U)
#define PMIC_INT_GPIO_GPIO1_INT_SHIFT            (0x0U)

/*! PMIC_INT_LDO_VMON Register Bit Positions */
#define PMIC_INT_LDO_VMON_VMON2_UVOV_INT_SHIFT   (0x6U)
#define PMIC_INT_LDO_VMON_VMON1_UVOV_INT_SHIFT   (0x5U)
#define PMIC_INT_LDO_VMON_VCCA_UVOV_INT_SHIFT    (0x4U)
#define PMIC_INT_LDO_VMON_LDO3_UVOV_INT_SHIFT    (0x2U)
#define PMIC_INT_LDO_VMON_LDO2_UVOV_INT_SHIFT    (0x1U)
#define PMIC_INT_LDO_VMON_LDO1_UVOV_INT_SHIFT    (0x0U)

/*! PMIC_MASK_LDO_VMON Register Bit Positions */
#define PMIC_MASK_LDO_VMON_VMON2_UVOV_MASK_SHIFT (0x6U)
#define PMIC_MASK_LDO_VMON_VMON1_UVOV_MASK_SHIFT (0x5U)
#define PMIC_MASK_LDO_VMON_VCCA_UVOV_MASK_SHIFT  (0x4U)
#define PMIC_MASK_LDO_VMON_LDO3_UVOV_MASK_SHIFT  (0x2U)
#define PMIC_MASK_LDO_VMON_LDO2_UVOV_MASK_SHIFT  (0x1U)
#define PMIC_MASK_LDO_VMON_LDO1_UVOV_MASK_SHIFT  (0x0U)

/*! PMIC_INT_BUCK Register Bit Positions */
#define PMIC_INT_BUCK4_UVOV_INT_SHIFT            (0x3U)
#define PMIC_INT_BUCK3_UVOV_INT_SHIFT            (0x2U)
#define PMIC_INT_BUCK2_UVOV_INT_SHIFT            (0x1U)
#define PMIC_INT_BUCK1_UVOV_INT_SHIFT            (0x0U)

/*! PMIC_MASK_BUCK Register Bit Positions */
#define PMIC_MASK_BUCK4_UVOV_MASK_SHIFT          (0x3U)
#define PMIC_MASK_BUCK3_UVOV_MASK_SHIFT          (0x2U)
#define PMIC_MASK_BUCK2_UVOV_MASK_SHIFT          (0x1U)
#define PMIC_MASK_BUCK1_UVOV_MASK_SHIFT          (0x0U)

/* ========================================================================= */
/*                          Structures and Enums                             */
/* ========================================================================= */

/* ========================================================================= */
/*                          Function Declarations                            */
/* ========================================================================= */
/*!
 * \brief  Function to get the PMIC Interrupt Registers for TPS6522x Burton PMIC.
 */
void pmic_get_tps6522x_intrCfg(Pmic_IntrCfg_t **pIntrCfg);

/*!
 * \brief  Function to get the PMIC GPIO Interrupt Mask Registers for
 *         TPS6522x Burton PMIC.
 */
void pmic_get_tps6522x_intrGpioCfg(Pmic_GpioIntrTypeCfg_t **pIntGpioCfg);

/*!
 * \brief  Function to decipher the L2 Error for TPS6522x Burton PMIC.
 */
int32_t Pmic_tps6522x_irqGetL2Error(Pmic_CoreHandle_t *pPmicCoreHandle, uint16_t l1RegAddr, Pmic_IrqStatus_t *pErrStat);

#ifdef __cplusplus
}

#endif /* __cplusplus */

#endif /* PMIC_IRQ_TPS6522X_PRIV_H_ */
