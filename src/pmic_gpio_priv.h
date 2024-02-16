/******************************************************************************
 * Copyright (c) 2024 Texas Instruments Incorporated - http://www.ti.com
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

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/**
 * \brief  GPIO CONF Register bit fields
 */
#define PMIC_GPIOX_CONF_GPIO_SEL_SHIFT (0x05U)
#define PMIC_GPIOX_CONF_GPIO_DEGLITCH_EN_SHIFT (0x04U)
#define PMIC_GPIOX_CONF_GPIO_PU_PD_EN_SHIFT (0x03U)
#define PMIC_GPIOX_CONF_GPIO_PU_SEL_SHIFT (0x02U)
#define PMIC_GPIOX_CONF_GPIO_OD_SHIFT (0x01U)
#define PMIC_GPIOX_CONF_GPIO_DIR_SHIFT (0x00U)

/**
 * \brief  GPIO IN Register bit fields
 */
#define PMIC_GPIO_IN_1_GPIO1_IN_SHIFT (0x00U)
#define PMIC_GPIO_IN_1_GPIO2_IN_SHIFT (0x01U)
#define PMIC_GPIO_IN_1_GPIO3_IN_SHIFT (0x02U)
#define PMIC_GPIO_IN_1_GPIO4_IN_SHIFT (0x03U)
#define PMIC_GPIO_IN_1_GPIO5_IN_SHIFT (0x04U)
#define PMIC_GPIO_IN_1_GPIO6_IN_SHIFT (0x05U)
#define PMIC_GPIO_IN_1_GPIO7_IN_SHIFT (0x06U)
#define PMIC_GPIO_IN_1_GPIO8_IN_SHIFT (0x07U)
#define PMIC_GPIO_IN_2_GPIO9_IN_SHIFT (0x00U)
#define PMIC_GPIO_IN_2_GPIO10_IN_SHIFT (0x01U)

/**
 * \brief  GPIO OUT Register bit fields
 */
#define PMIC_GPIO_OUT_1_GPIO1_OUT_SHIFT (0x00U)
#define PMIC_GPIO_OUT_1_GPIO2_OUT_SHIFT (0x01U)
#define PMIC_GPIO_OUT_1_GPIO3_OUT_SHIFT (0x02U)
#define PMIC_GPIO_OUT_1_GPIO4_OUT_SHIFT (0x03U)
#define PMIC_GPIO_OUT_1_GPIO5_OUT_SHIFT (0x04U)
#define PMIC_GPIO_OUT_1_GPIO6_OUT_SHIFT (0x05U)
#define PMIC_GPIO_OUT_1_GPIO7_OUT_SHIFT (0x06U)
#define PMIC_GPIO_OUT_1_GPIO8_OUT_SHIFT (0x07U)
#define PMIC_GPIO_OUT_2_GPIO9_OUT_SHIFT (0x00U)
#define PMIC_GPIO_OUT_2_GPIO10_OUT_SHIFT (0x01U)

/**
 * \brief  GPIO Register bit mask values for all GPIO pins
 */
#define PMIC_GPIOX_CONF_GPIO_SEL_MASK                                          \
  (uint8_t)(0x07U << PMIC_GPIOX_CONF_GPIO_SEL_SHIFT)
#define PMIC_GPIOX_CONF_GPIO_DEGLITCH_EN_MASK                                  \
  (uint8_t)(0x01U << PMIC_GPIOX_CONF_GPIO_DEGLITCH_EN_SHIFT)
#define PMIC_GPIOX_CONF_GPIO_PU_PD_EN_MASK                                     \
  (uint8_t)(0x01U << PMIC_GPIOX_CONF_GPIO_PU_PD_EN_SHIFT)
#define PMIC_GPIOX_CONF_GPIO_PU_SEL_MASK                                       \
  (uint8_t)(0x01U << PMIC_GPIOX_CONF_GPIO_PU_SEL_SHIFT)
#define PMIC_GPIOX_CONF_GPIO_OD_MASK                                           \
  (uint8_t)(0x01U << PMIC_GPIOX_CONF_GPIO_OD_SHIFT)
#define PMIC_GPIOX_CONF_GPIO_DIR_MASK                                          \
  (uint8_t)(0x01U << PMIC_GPIOX_CONF_GPIO_DIR_SHIFT)

/**
 * \brief  GPIO IN/OUT bit field
 */
#define PMIC_GPIO_IN_OUT_X_GPIOX_IN_OUT_BITFIELD (1U)

/** \brief Max value for GPIO Pin Function */
#define PMIC_GPIO_PINFUNC_MAX (7U)

#define PMIC_GPIO_INT_ENABLE (0U)
#define PMIC_GPIO_INT_MASK (1U)
#define PMIC_DEV_BB_TPS653860XX (3U)

#define PMIC_BB_GPO1 (1)
#define PMIC_BB_GPO2 (2)
#define PMIC_BB_GPO3 (3)
#define PMIC_BB_GPO4 (4)

#define PMIC_TPS653860XX_GPIO_PIN_MIN (1U)
#define PMIC_TPS653860XX_GPIO_PIN_MAX (6U)

#define PMIC_GPI_1_CONF_REGADDR (0x7EU)
#define PMIC_GPO_1_CONF_REGADDR (0x7CU)
#define PMIC_GPO_2_CONF_REGADDR (0x7DU)

#define PMIC_GPI_1_GPI_1_SHIFT (0x00)
#define PMIC_GPI_4_GPI_2_SHIFT (0x01)

#define PMIC_GPO_1_GPO_1_SHIFT (0x00)
#define PMIC_GPO_2_GPI_3_SHIFT (0x02)
#define PMIC_GPO_3_GPO_1_SHIFT (0x00)
#define PMIC_GPO_4_GPO_3_SHIFT (0x03)

#define PMIC_GPI_1_GPI_1_MASK (uint8_t)(0x01U << PMIC_GPI_1_GPI_1_SHIFT)
#define PMIC_GPI_4_GPI_2_MASK (uint8_t)(0x03U << PMIC_GPI_4_GPI_2_SHIFT)

#define PMIC_GPO_1_GPO_2_MASK (uint8_t)(0x07U << PMIC_GPO_1_GPO_1_SHIFT)
#define PMIC_GPO_2_GPO_3_MASK (uint8_t)(0x07U << PMIC_GPO_2_GPI_3_SHIFT)
#define PMIC_GPO_3_GPO_2_MASK (uint8_t)(0x07U << PMIC_GPO_3_GPO_1_SHIFT)
#define PMIC_GPO_4_GPO_3_MASK (uint8_t)(0x07U << PMIC_GPO_4_GPO_3_SHIFT)

#define PMIC_RDBK_DGL_CFG2_REGADDR (0x80U)
#define PMIC_RDBK_DGL_CFG3_REGADDR (0x81U)

#define PMIC_RDBK_GPO1_F_SHIFT (0X00)
#define PMIC_RDBK_GPO1_R_SHIFT (0X02)
#define PMIC_RDBK_GPO2_F_SHIFT (0X04)
#define PMIC_RDBK_GPO2_R_SHIFT (0X06)

#define PMIC_RDBK_GPO1_F_MASK (uint8_t)(0x03U << PMIC_RDBK_GPO1_F_SHIFT)
#define PMIC_RDBK_GPO1_R_MASK (uint8_t)(0x03U << PMIC_RDBK_GPO1_R_SHIFT)
#define PMIC_RDBK_GPO2_F_MASK (uint8_t)(0x03U << PMIC_RDBK_GPO2_F_SHIFT)
#define PMIC_RDBK_GPO2_R_MASK (uint8_t)(0x03U << PMIC_RDBK_GPO2_R_SHIFT)

#define PMIC_RDBK_GPO3_F_SHIFT (0X00)
#define PMIC_RDBK_GPO3_R_SHIFT (0X02)
#define PMIC_RDBK_GPO4_F_SHIFT (0X04)
#define PMIC_RDBK_GPO4_R_SHIFT (0X06)

#define PMIC_RDBK_GPO3_F_MASK (uint8_t)(0x03U << PMIC_RDBK_GPO3_F_SHIFT)
#define PMIC_RDBK_GPO3_R_MASK (uint8_t)(0x03U << PMIC_RDBK_GPO3_R_SHIFT)
#define PMIC_RDBK_GPO4_F_MASK (uint8_t)(0x03U << PMIC_RDBK_GPO4_F_SHIFT)
#define PMIC_RDBK_GPO4_R_MASK (uint8_t)(0x03U << PMIC_RDBK_GPO4_R_SHIFT)

#define PMIC_GPO_CFG2_EN_OUT_PU_CFG_SHIFT (6)
#define PMIC_GPO_CFG2_EN_OUT_PU_CFG_MASK                                       \
  (0x03U << PMIC_GPO_CFG2_EN_OUT_PU_CFG_SHIFT)

#define PMIC_GPO_HIGH_IMPEDANCE (0x02U)
#define PMIC_GPO_PULL_UP_LDO (0x01U)
#define PMIC_GPO_PULL_UP_VDDIO (0x00U)

#define PMIC_GPIO_PULL_DISABLED (0x00U)
#define PMIC_GPIO_PULL_UP (0x01U)
#define PMIC_GPIO_PULL_UP_TO_LDO (0x02U)

/* Macros for GPO_CFG1 register (7Ch) */
#define PMIC_GPO_CFG1_REG_ADDR (0x7CU)
#define PMIC_GPO_CFG1_M_PMIC_CFG_SHIFT (6)
#define PMIC_GPO_CFG1_M_PMIC_CFG_MASK (0x01U << PMIC_GPO_CFG1_M_PMIC_CFG_SHIFT)
#define PMIC_GPO_CFG1_GPO2_CFG_SHIFT (3)
#define PMIC_GPO_CFG1_GPO2_CFG_MASK (0x07U << PMIC_GPO_CFG1_GPO2_CFG_SHIFT)
#define PMIC_GPO_CFG1_GPO1_CFG_SHIFT (0)
#define PMIC_GPO_CFG1_GPO1_CFG_MASK (0x07U << PMIC_GPO_CFG1_GPO1_CFG_SHIFT)

/* Macros for GPO_CFG2 register (7Dh) */
#define PMIC_GPO_CFG2_REG_ADDR (0x7DU)
#define PMIC_GPO_CFG2_GPO_EN_SHIFT (5)
#define PMIC_GPO_CFG2_GPO_EN_MASK (0x06U << PMIC_GPO_CFG2_GPO_EN_SHIFT)
#define PMIC_GPO_CFG2_GPO4_CFG_SHIFT (3)
#define PMIC_GPO_CFG2_GPO4_CFG_MASK (0x07U << PMIC_GPO_CFG2_GPO4_CFG_SHIFT)
#define PMIC_GPO_CFG2_GPO3_CFG_SHIFT (0)
#define PMIC_GPO_CFG2_GPO3_CFG_MASK (0x07U << PMIC_GPO_CFG2_GPO3_CFG_SHIFT)

/* Macros for GPI_CFG register (7Eh)*/
#define PMIC_GPI_CFG_REG_ADDR (0x7EU)
#define PMIC_GPI_CFG_GPI1_SHIFT (0U)
#define PMIC_GPI_CFG_GPI1_MASK (0x1U << PMIC_GPI_CFG_GPI1_SHIFT)
#define PMIC_GPI_CFG_GPI4_SHIFT (1U)
#define PMIC_GPI_CFG_GPI4_MASK (0x3U << PMIC_GPI_CFG_GPI4_SHIFT)

/*==========================================================================*/
/*                         Structures and Enums                             */
/*==========================================================================*/

/**
 * @brief   PMIC gpio details object structure
 *
 * @param   intRegAddr         Register Address of the gpio interrupt
 * @param   intRegBitPos       Register bit position of gpio interrupt
 * @param   intRegPolBitPos    Register bit position of gpio polarity
 */
typedef struct Pmic_GpioIntRegCfg_s {
  uint8_t intRegAddr;
  uint8_t intRegBitPos;
  uint8_t intRegPolBitPos;
} Pmic_GpioIntRegCfg_t;

static Pmic_GpioInOutCfg_t gTps65386_gpioInOutCfg[] = {
    {
        PMIC_GPI_1_CONF_REGADDR,
        PMIC_GPI_1_GPI_1_SHIFT,
        PMIC_GPI_1_GPI_1_MASK,
    },
    {
        PMIC_GPI_1_CONF_REGADDR,
        PMIC_GPI_4_GPI_2_SHIFT,
        PMIC_GPI_4_GPI_2_MASK,
    },
    {
        PMIC_GPO_1_CONF_REGADDR,
        PMIC_GPO_1_GPO_1_SHIFT,
        PMIC_GPO_1_GPO_2_MASK,

    },
    {
        PMIC_GPO_1_CONF_REGADDR,
        PMIC_GPO_2_GPI_3_SHIFT,
        PMIC_GPO_2_GPO_3_MASK,
    },
    {
        PMIC_GPO_2_CONF_REGADDR,
        PMIC_GPO_3_GPO_1_SHIFT,
        PMIC_GPO_3_GPO_2_MASK,
    },
    {
        PMIC_GPO_2_CONF_REGADDR,
        PMIC_GPO_4_GPO_3_SHIFT,
        PMIC_GPO_4_GPO_3_MASK,
    },
};

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* PMIC_GPIO_PRIV_H_ */
