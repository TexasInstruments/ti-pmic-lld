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
 * \file   pmic_gpio_priv.h
 *
 * \brief: This file contains macro definitions, structures and function
 *         prototypes for driver specific PMIC gpio configuration
 */

#ifndef PMIC_GPIO_PRIV_H_
#define PMIC_GPIO_PRIV_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <pmic_gpio.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */
/*!
 * \brief  GPIO Register Address
 */
#define PMIC_GPIO1_CONF_REGADDR                 (0x31U)
#define PMIC_GPIO2_CONF_REGADDR                 (0x32U)
#define PMIC_GPIO3_CONF_REGADDR                 (0x33U)
#define PMIC_GPIO4_CONF_REGADDR                 (0x34U)
#define PMIC_GPIO5_CONF_REGADDR                 (0x35U)
#define PMIC_GPIO6_CONF_REGADDR                 (0x36U)
#define PMIC_GPIO7_CONF_REGADDR                 (0x37U)
#define PMIC_GPIO8_CONF_REGADDR                 (0x38U)
#define PMIC_GPIO9_CONF_REGADDR                 (0x39U)
#define PMIC_GPIO10_CONF_REGADDR                (0x3AU)
#define PMIC_GPIO_OUT_1_REGADDR                 (0x3DU)
#define PMIC_GPIO_OUT_2_REGADDR                 (0x3EU)
#define PMIC_GPIO_IN_1_REGADDR                  (0x3FU)
#define PMIC_GPIO_IN_2_REGADDR                  (0x40U)

/*!
 * \brief  GPIO Interrupt Mask Register Address
 */
#define PMIC_FSM_TRIG_MASK_1_REGADDR            (0x46U)
#define PMIC_FSM_TRIG_MASK_2_REGADDR            (0x47U)
#define PMIC_FSM_TRIG_MASK_3_REGADDR            (0x48U)

/*!
 * \brief  GPIO Register bit fields for all GPIO pins
 */
#define PMIC_GPIOX_CONF_GPIO_SEL_SHIFT             (0x05U)
#define PMIC_GPIOX_CONF_GPIO_DEGLITCH_EN_SHIFT     (0x04U)
#define PMIC_GPIOX_CONF_GPIO_PU_PD_EN_SHIFT        (0x03U)
#define PMIC_GPIOX_CONF_GPIO_PU_SEL_SHIFT          (0x02U)
#define PMIC_GPIOX_CONF_GPIO_OD_SHIFT              (0x01U)
#define PMIC_GPIOX_CONF_GPIO_DIR_SHIFT             (0x00U)

/*!
 * \brief  GPIO IN Register bit fields
 */
#define PMIC_GPIO_IN_1_GPIO1_IN_SHIFT         (0x00U)
#define PMIC_GPIO_IN_1_GPIO2_IN_SHIFT         (0x01U)
#define PMIC_GPIO_IN_1_GPIO3_IN_SHIFT         (0x02U)
#define PMIC_GPIO_IN_1_GPIO4_IN_SHIFT         (0x03U)
#define PMIC_GPIO_IN_1_GPIO5_IN_SHIFT         (0x04U)
#define PMIC_GPIO_IN_1_GPIO6_IN_SHIFT         (0x05U)
#define PMIC_GPIO_IN_1_GPIO7_IN_SHIFT         (0x06U)
#define PMIC_GPIO_IN_1_GPIO8_IN_SHIFT         (0x07U)
#define PMIC_GPIO_IN_2_GPIO9_IN_SHIFT         (0x00U)
#define PMIC_GPIO_IN_2_GPIO10_IN_SHIFT        (0x01U)

/*!
 * \brief  GPIO OUT Register bit fields
 */
#define PMIC_GPIO_OUT_1_GPIO1_OUT_SHIFT         (0x00U)
#define PMIC_GPIO_OUT_1_GPIO2_OUT_SHIFT         (0x01U)
#define PMIC_GPIO_OUT_1_GPIO3_OUT_SHIFT         (0x02U)
#define PMIC_GPIO_OUT_1_GPIO4_OUT_SHIFT         (0x03U)
#define PMIC_GPIO_OUT_1_GPIO5_OUT_SHIFT         (0x04U)
#define PMIC_GPIO_OUT_1_GPIO6_OUT_SHIFT         (0x05U)
#define PMIC_GPIO_OUT_1_GPIO7_OUT_SHIFT         (0x06U)
#define PMIC_GPIO_OUT_1_GPIO8_OUT_SHIFT         (0x07U)
#define PMIC_GPIO_OUT_2_GPIO9_OUT_SHIFT         (0x00U)
#define PMIC_GPIO_OUT_2_GPIO10_OUT_SHIFT        (0x01U)

/*!
 * \brief  GPIO FSM TRIG Mask Register bit fields
 */
/*! Bit fields for PMIC_FSM_TRIG_MASK_1_REG */
#define PMIC_FSM_TRIG_MASK_1_GPIO1_FSM_MASK_SHIFT          (0x0U)
#define PMIC_FSM_TRIG_MASK_1_GPIO1_FSM_MASK_POL_SHIFT      (0x1U)
#define PMIC_FSM_TRIG_MASK_1_GPIO2_FSM_MASK_SHIFT          (0x2U)
#define PMIC_FSM_TRIG_MASK_1_GPIO2_FSM_MASK_POL_SHIFT      (0x3U)
#define PMIC_FSM_TRIG_MASK_1_GPIO3_FSM_MASK_SHIFT          (0x4U)
#define PMIC_FSM_TRIG_MASK_1_GPIO3_FSM_MASK_POL_SHIFT      (0x5U)
#define PMIC_FSM_TRIG_MASK_1_GPIO4_FSM_MASK_SHIFT          (0x6U)
#define PMIC_FSM_TRIG_MASK_1_GPIO4_FSM_MASK_POL_SHIFT      (0x7U)
/*! Bit fields for PMIC_FSM_TRIG_MASK_2_REG */
#define PMIC_FSM_TRIG_MASK_2_GPIO5_FSM_MASK_SHIFT          (0x0U)
#define PMIC_FSM_TRIG_MASK_2_GPIO5_FSM_MASK_POL_SHIFT      (0x1U)
#define PMIC_FSM_TRIG_MASK_2_GPIO6_FSM_MASK_SHIFT          (0x2U)
#define PMIC_FSM_TRIG_MASK_2_GPIO6_FSM_MASK_POL_SHIFT      (0x3U)
#define PMIC_FSM_TRIG_MASK_2_GPIO7_FSM_MASK_SHIFT          (0x4U)
#define PMIC_FSM_TRIG_MASK_2_GPIO7_FSM_MASK_POL_SHIFT      (0x5U)
#define PMIC_FSM_TRIG_MASK_2_GPIO8_FSM_MASK_SHIFT          (0x6U)
#define PMIC_FSM_TRIG_MASK_2_GPIO8_FSM_MASK_POL_SHIFT      (0x7U)
/*! Bit fields for PMIC_FSM_TRIG_MASK_3_REG */
#define PMIC_FSM_TRIG_MASK_3_GPIO9_FSM_MASK_SHIFT          (0x0U)
#define PMIC_FSM_TRIG_MASK_3_GPIO9_FSM_MASK_POL_SHIFT      (0x1U)
#define PMIC_FSM_TRIG_MASK_3_GPIO10_FSM_MASK_SHIFT         (0x2U)
#define PMIC_FSM_TRIG_MASK_3_GPIO10_FSM_MASK_POL_SHIFT     (0x3U)
#define PMIC_FSM_TRIG_MASK_3_GPIO11_FSM_MASK_SHIFT         (0x4U)
#define PMIC_FSM_TRIG_MASK_3_GPIO11_FSM_MASK_POL_SHIFT     (0x5U)

/*!
 * \brief  GPIO Register bit mask values for all GPIO pins
 */
#define PMIC_GPIOX_CONF_GPIO_SEL_MASK   (uint8_t) \
                                        (0x07U << \
                                        PMIC_GPIOX_CONF_GPIO_SEL_SHIFT)
#define PMIC_GPIOX_CONF_GPIO_DEGLITCH_EN_MASK     \
                                        (uint8_t) (0x01U <<  \
                                        PMIC_GPIOX_CONF_GPIO_DEGLITCH_EN_SHIFT)
#define PMIC_GPIOX_CONF_GPIO_PU_PD_EN_MASK        \
                                        (uint8_t) (0x01U <<  \
                                        PMIC_GPIOX_CONF_GPIO_PU_PD_EN_SHIFT)
#define PMIC_GPIOX_CONF_GPIO_PU_SEL_MASK          \
                                        (uint8_t) (0x01U <<  \
                                        PMIC_GPIOX_CONF_GPIO_PU_SEL_SHIFT)
#define PMIC_GPIOX_CONF_GPIO_OD_MASK    (uint8_t)  \
                                        (0x01U <<  \
                                        PMIC_GPIOX_CONF_GPIO_OD_SHIFT)
#define PMIC_GPIOX_CONF_GPIO_DIR_MASK   (uint8_t)  \
                                        (0x01U <<  \
                                        PMIC_GPIOX_CONF_GPIO_DIR_SHIFT)


/*!
 * \brief  GPIO IN Register bit mask values
 */
#define PMIC_GPIO_IN_1_GPIO1_IN_MASK     (uint8_t)  \
                                         (0x01U <<  \
                                          PMIC_GPIO_IN_1_GPIO1_IN_SHIFT)
#define PMIC_GPIO_IN_1_GPIO2_IN_MASK     (uint8_t)  \
                                         (0x01U <<  \
                                          PMIC_GPIO_IN_1_GPIO2_IN_SHIFT)
#define PMIC_GPIO_IN_1_GPIO3_IN_MASK     (uint8_t)  \
                                         (0x01U <<  \
                                          PMIC_GPIO_IN_1_GPIO3_IN_SHIFT)
#define PMIC_GPIO_IN_1_GPIO4_IN_MASK     (uint8_t)  \
                                         (0x01U <<  \
                                          PMIC_GPIO_IN_1_GPIO4_IN_SHIFT)
#define PMIC_GPIO_IN_1_GPIO5_IN_MASK     (uint8_t)  \
                                         (0x01U <<  \
                                          PMIC_GPIO_IN_1_GPIO5_IN_SHIFT)
#define PMIC_GPIO_IN_1_GPIO6_IN_MASK     (uint8_t)  \
                                         (0x01U <<  \
                                          PMIC_GPIO_IN_1_GPIO6_IN_SHIFT)
#define PMIC_GPIO_IN_1_GPIO7_IN_MASK     (uint8_t)  \
                                         (0x01U <<  \
                                          PMIC_GPIO_IN_1_GPIO7_IN_SHIFT)
#define PMIC_GPIO_IN_1_GPIO8_IN_MASK     (uint8_t)  \
                                         (0x01U <<  \
                                          PMIC_GPIO_IN_1_GPIO8_IN_SHIFT)
#define PMIC_GPIO_IN_2_GPIO9_IN_MASK     (uint8_t)  \
                                         (0x01U <<  \
                                          PMIC_GPIO_IN_2_GPIO9_IN_SHIFT)
#define PMIC_GPIO_IN_2_GPIO10_IN_MASK    (uint8_t)  \
                                         (0x01U <<  \
                                          PMIC_GPIO_IN_2_GPIO10_IN_SHIFT)

/*!
 * \brief  GPIO OUT Register bit mask values
 */
#define PMIC_GPIO_OUT_1_GPIO1_OUT_MASK     (uint8_t)  \
                                           (0x01U <<  \
                                            PMIC_GPIO_OUT_1_GPIO1_OUT_SHIFT)
#define PMIC_GPIO_OUT_1_GPIO2_OUT_MASK     (uint8_t)  \
                                           (0x01U <<  \
                                            PMIC_GPIO_OUT_1_GPIO2_OUT_SHIFT)
#define PMIC_GPIO_OUT_1_GPIO3_OUT_MASK     (uint8_t)  \
                                           (0x01U <<  \
                                            PMIC_GPIO_OUT_1_GPIO3_OUT_SHIFT)
#define PMIC_GPIO_OUT_1_GPIO4_OUT_MASK     (uint8_t)  \
                                           (0x01U <<  \
                                            PMIC_GPIO_OUT_1_GPIO4_OUT_SHIFT)
#define PMIC_GPIO_OUT_1_GPIO5_OUT_MASK     (uint8_t)  \
                                           (0x01U <<  \
                                            PMIC_GPIO_OUT_1_GPIO5_OUT_SHIFT)
#define PMIC_GPIO_OUT_1_GPIO6_OUT_MASK     (uint8_t)  \
                                           (0x01U <<  \
                                            PMIC_GPIO_OUT_1_GPIO6_OUT_SHIFT)
#define PMIC_GPIO_OUT_1_GPIO7_OUT_MASK     (uint8_t)  \
                                           (0x01U <<  \
                                            PMIC_GPIO_OUT_1_GPIO7_OUT_SHIFT)
#define PMIC_GPIO_OUT_1_GPIO8_OUT_MASK     (uint8_t)  \
                                           (0x01U <<  \
                                            PMIC_GPIO_OUT_1_GPIO8_OUT_SHIFT)
#define PMIC_GPIO_OUT_2_GPIO9_OUT_MASK     (uint8_t)  \
                                           (0x01U <<  \
                                            PMIC_GPIO_OUT_2_GPIO9_OUT_SHIFT)
#define PMIC_GPIO_OUT_2_GPIO10_OUT_MASK    (uint8_t)  \
                                           (0x01U <<  \
                                            PMIC_GPIO_OUT_2_GPIO10_OUT_SHIFT)

/*!
 * \brief  GPIO FSM TRIG Mask Register bit mask
 */
/*! Bit Mask for PMIC_FSM_TRIG_MASK_1_REG */
#define PMIC_FSM_TRIG_MASK_1_GPIO1_FSM_MASK_MASK          \
                                (uint8_t)(0x01 <<  \
                                PMIC_FSM_TRIG_MASK_1_GPIO1_FSM_MASK_SHIFT)
#define PMIC_FSM_TRIG_MASK_1_GPIO1_FSM_MASK_POL_MASK      \
                                (uint8_t)(0x01 <<  \
                                PMIC_FSM_TRIG_MASK_1_GPIO1_FSM_MASK_POL_SHIFT)
#define PMIC_FSM_TRIG_MASK_1_GPIO2_FSM_MASK_MASK         \
                                (uint8_t)(0x01 <<  \
                                PMIC_FSM_TRIG_MASK_1_GPIO2_FSM_MASK_SHIFT)
#define PMIC_FSM_TRIG_MASK_1_GPIO2_FSM_MASK_POL_MASK     \
                                (uint8_t)(0x01 <<  \
                                PMIC_FSM_TRIG_MASK_1_GPIO2_FSM_MASK_POL_SHIFT)
#define PMIC_FSM_TRIG_MASK_1_GPIO3_FSM_MASK_MASK         \
                                (uint8_t)(0x01 <<  \
                                PMIC_FSM_TRIG_MASK_1_GPIO3_FSM_MASK_SHIFT)
#define PMIC_FSM_TRIG_MASK_1_GPIO3_FSM_MASK_POL_MASK     \
                                (uint8_t)(0x01 <<  \
                                PMIC_FSM_TRIG_MASK_1_GPIO3_FSM_MASK_POL_SHIFT)
#define PMIC_FSM_TRIG_MASK_1_GPIO4_FSM_MASK_MASK         \
                                (uint8_t)(0x01 <<  \
                                PMIC_FSM_TRIG_MASK_1_GPIO4_FSM_MASK_SHIFT)
#define PMIC_FSM_TRIG_MASK_1_GPIO4_FSM_MASK_POL_MASK     \
                                (uint8_t)(0x01 <<  \
                                PMIC_FSM_TRIG_MASK_1_GPIO4_FSM_MASK_POL_SHIFT)
/*! Bit Mask for PMIC_FSM_TRIG_MASK_2_REG */
#define PMIC_FSM_TRIG_MASK_2_GPIO5_FSM_MASK_MASK         \
                                (uint8_t)(0x01 <<  \
                                PMIC_FSM_TRIG_MASK_2_GPIO5_FSM_MASK_SHIFT)
#define PMIC_FSM_TRIG_MASK_2_GPIO5_FSM_MASK_POL_MASK     \
                                (uint8_t)(0x01 <<  \
                                PMIC_FSM_TRIG_MASK_2_GPIO5_FSM_MASK_POL_SHIFT)
#define PMIC_FSM_TRIG_MASK_2_GPIO6_FSM_MASK_MASK         \
                                (uint8_t)(0x01 <<  \
                                PMIC_FSM_TRIG_MASK_2_GPIO6_FSM_MASK_SHIFT)
#define PMIC_FSM_TRIG_MASK_2_GPIO6_FSM_MASK_POL_MASK     \
                                (uint8_t)(0x01 <<  \
                                PMIC_FSM_TRIG_MASK_2_GPIO6_FSM_MASK_POL_SHIFT)
#define PMIC_FSM_TRIG_MASK_2_GPIO7_FSM_MASK_MASK         \
                                (uint8_t)(0x01 <<  \
                                PMIC_FSM_TRIG_MASK_2_GPIO7_FSM_MASK_SHIFT)
#define PMIC_FSM_TRIG_MASK_2_GPIO7_FSM_MASK_POL_MASK     \
                                (uint8_t)(0x01 <<  \
                                PMIC_FSM_TRIG_MASK_2_GPIO7_FSM_MASK_POL_SHIFT)
#define PMIC_FSM_TRIG_MASK_2_GPIO8_FSM_MASK_MASK         \
                                (uint8_t)(0x01 <<  \
                                PMIC_FSM_TRIG_MASK_2_GPIO8_FSM_MASK_SHIFT)
#define PMIC_FSM_TRIG_MASK_2_GPIO8_FSM_MASK_POL_MASK     \
                                (uint8_t)(0x01 <<  \
                                PMIC_FSM_TRIG_MASK_2_GPIO8_FSM_MASK_POL_SHIFT)
/*! Bit Mask for PMIC_FSM_TRIG_MASK_3_REG */
#define PMIC_FSM_TRIG_MASK_3_GPIO9_FSM_MASK_MASK         \
                                (uint8_t)(0x01 <<  \
                                PMIC_FSM_TRIG_MASK_3_GPIO9_FSM_MASK_SHIFT)
#define PMIC_FSM_TRIG_MASK_2_GPIO9_FSM_MASK_POL_MASK     \
                                (uint8_t)(0x01 <<  \
                                PMIC_FSM_TRIG_MASK_3_GPIO9_FSM_MASK_POL_SHIFT)
#define PMIC_FSM_TRIG_MASK_3_GPIO10_FSM_MASK_MASK        \
                                (uint8_t)(0x01 <<  \
                                PMIC_FSM_TRIG_MASK_3_GPIO10_FSM_MASK_SHIFT)
#define PMIC_FSM_TRIG_MASK_2_GPIO10_FSM_MASK_POL_MASK    \
                                (uint8_t)(0x01 <<  \
                                PMIC_FSM_TRIG_MASK_3_GPIO10_FSM_MASK_POL_SHIFT)
#define PMIC_FSM_TRIG_MASK_3_GPIO11_FSM_MASK_MASK        \
                                (uint8_t)(0x01 <<  \
                                PMIC_FSM_TRIG_MASK_3_GPIO11_FSM_MASK_SHIFT)
#define PMIC_FSM_TRIG_MASK_2_GPIO11_FSM_MASK_POL_MASK    \
                                (uint8_t)(0x01 <<  \
                                PMIC_FSM_TRIG_MASK_3_GPIO11_FSM_MASK_POL_SHIFT)

/*!
 * \brief  GPIO NPWRON/Enable PIN
 *          NPWRON is valid only for TPS6594x Leo Device
 */
#define PMIC_NPWRON_ENABLE_PIN                            (0U)

/*!
 * \brief  GPIO IN/OUT bit field
 */
#define PMIC_GPIO_IN_OUT_X_GPIOX_IN_OUT_BITFIELD          (1U)

/** \brief Max value for GPIO Pin Function */
#define PMIC_GPIO_PINFUNC_MAX              (7U)

/**
 *  \anchor Pmic_GpioInterruptMask
 *  \name   PMIC GPIO Interrupt Mask selection
 *
 *  @{
 */
#define PMIC_GPIO_INT_ENABLE               (0U)
#define PMIC_GPIO_INT_MASK                 (1U)
/*  @} */

/*==========================================================================*/
/*                         Structures and Enums                             */
/*==========================================================================*/
/*!
 * \brief   PMIC GPIO Pins with Input Ouput Configuration
 *
 * \param   regAddr          GPIO Pin Register Address
 * \param   outRegAddr       GPIO OUT Register Address
 * \param   inRegAddr        GPIO IN Register Address
 * \param   inRegBitPos      Bit position of GPIO IN Register bit position
 * \param   outRegBitPos     Bit position of GPIO OUT Register bit position
 */
typedef struct Pmic_GpioInOutCfg_s
{
    uint8_t regAddr;
    uint8_t outRegAddr;
    uint8_t inRegAddr;
    uint8_t inRegBitPos;
    uint8_t outRegBitPos;
} Pmic_GpioInOutCfg_t;

/*!
* \brief   PMIC gpio details object structure
*
* \param   intRegAddr         Register Address of the gpio interrupt
* \param   intRegBitPos       Register bit position of gpio interrupt
* \param   intRegPolBitPos    Register bit position of gpio polarity
*/
typedef struct Pmic_GpioIntRegCfg_s
{
    uint8_t  intRegAddr;
    uint8_t  intRegBitPos;
    uint8_t  intRegPolBitPos;
} Pmic_GpioIntRegCfg_t;

/*==========================================================================*/
/*                         Function Declarations                            */
/*==========================================================================*/
/*!
 * \brief   This function is used to set the GPIO and NPWRON Pin Functionality
 */
int32_t Pmic_gpioSetPinFunc(Pmic_CoreHandle_t   *pPmicCoreHandle,
                            const uint8_t        pin,
                            const Pmic_GpioCfg_t gpioCfg);

/*!
 * \brief   This function is used to set the NPWRON/Enable deglitch time and
 *          NPWRON/Enable Pull UP/Down configuration
 *          Valid only for TPS6594x PMIC
 */
int32_t Pmic_gpioSetNPwronEnableDeglitchPullCtrlCfg(
                                          Pmic_CoreHandle_t   *pPmicCoreHandle,
                                          const Pmic_GpioCfg_t gpioCfg);

/*!
 * \brief   This function is used to set the NPWRON Pin Polarity
 */
int32_t Pmic_gpioSetPinPolarity(Pmic_CoreHandle_t    *pPmicCoreHandle,
                                const Pmic_GpioCfg_t  gpioCfg);

/*!
 * \brief   This function is used to get the GPIO and NPWRON Deglitch time
 */
int32_t Pmic_gpioGetDeglitchTime(Pmic_CoreHandle_t *pPmicCoreHandle,
                                 const uint8_t      pin,
                                 Pmic_GpioCfg_t    *pGpioCfg);

/*!
 * \brief   This function is used to get the GPIO and NPWRON Pin Functionality
 */
int32_t Pmic_gpioGetPinFunc(Pmic_CoreHandle_t *pPmicCoreHandle,
                            const uint8_t      pin,
                            Pmic_GpioCfg_t    *pGpioCfg);

/*!
 * \brief   This function is used to get the GPIO and NPWRON Pin Pull Control
 */
int32_t Pmic_gpioGetPullCtrl(Pmic_CoreHandle_t *pPmicCoreHandle,
                             const uint8_t      pin,
                             Pmic_GpioCfg_t    *pGpioCfg);

/*!
 * \brief   This function is used to get the NPWRON Pin Polarity
 */
int32_t Pmic_gpioGetPinPolarity(Pmic_CoreHandle_t *pPmicCoreHandle,
                                Pmic_GpioCfg_t    *pGpioCfg);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif/* PMIC_GPIO_PRIV_H_ */
