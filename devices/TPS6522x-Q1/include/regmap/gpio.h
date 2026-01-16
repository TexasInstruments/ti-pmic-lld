/******************************************************************************
 * Copyright (c) 2025 Texas Instruments Incorporated - http://www.ti.com
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
 * @file gpio.h
 *
 * @brief PMIC LLD GPIO module register addresses and bit fields.
 */
#ifndef REGMAP_GPIO_H
#define REGMAP_GPIO_H

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

#define GPIO1_CONF_REG      ((uint16_t)0x31U)
#define GPIO2_CONF_REG      ((uint16_t)0x32U)
#define GPIO3_CONF_REG      ((uint16_t)0x33U)
#define GPIO4_CONF_REG      ((uint16_t)0x34U)
#define GPIO5_CONF_REG      ((uint16_t)0x35U)
#define GPIO6_CONF_REG      ((uint16_t)0x36U)
#define POWER_ON_CONFIG_REG ((uint16_t)0x3CU)
#define GPIO_OUT_1_REG      ((uint16_t)0x3DU)
#define GPIO_IN_1_REG       ((uint16_t)0x3FU)
#define STAT_STARTUP_REG    ((uint16_t)0x73U)
#define ENABLE_DRV_REG_REG  ((uint16_t)0x80U)
#define ENABLE_DRV_STAT_REG ((uint16_t)0x82U)

/* ========================================================================== */
/*                              Register Bit Fields                           */
/* ========================================================================== */

// GPIOx_CONF
#define GPIO_SEL_SHIFT         (5U)
#define GPIO_DEGLITCH_EN_SHIFT (4U)
#define GPIO_PU_PD_EN_SHIFT    (3U)
#define GPIO_PU_SEL_SHIFT      (2U)
#define GPIO_OD_SHIFT          (1U)
#define GPIO_DIR_SHIFT         (0U)
#define GPIO_SEL_MASK          (0x03U << GPIO_SEL_SHIFT)
#define GPIO_DEGLITCH_EN_MASK  (0x01U << GPIO_DEGLITCH_EN_SHIFT)
#define GPIO_PU_PD_EN_MASK     (0x01U << GPIO_PU_PD_EN_SHIFT)
#define GPIO_PU_SEL_MASK       (0x01U << GPIO_PU_SEL_SHIFT)
#define GPIO_OD_MASK           (0x01U << GPIO_OD_SHIFT)
#define GPIO_DIR_MASK          (0x01U << GPIO_DIR_SHIFT)

// POWER_ON_CONFIG
#define EN_PB_VSENSE_CONFIG_SHIFT (6U)
#define EN_PB_DEGL_SHIFT          (5U)
#define NINT_ENDRV_SEL_SHIFT      (1U)
#define NINT_ENDRV_PU_SEL_SHIFT   (0U)
#define EN_PB_VSENSE_CONFIG_MASK  (0x03U << EN_PB_VSENSE_CONFIG_SHIFT)
#define EN_PB_DEGL_MASK           (0x01U << EN_PB_DEGL_SHIFT)
#define NINT_ENDRV_SEL_MASK       (0x01U << NINT_ENDRV_SEL_SHIFT)
#define NINT_ENDRV_PU_SEL_MASK    (0x01U << NINT_ENDRV_PU_SEL_SHIFT)

// GPIO_OUT_1
#define GPIO6_OUT_SHIFT (5U)
#define GPIO5_OUT_SHIFT (4U)
#define GPIO4_OUT_SHIFT (3U)
#define GPIO3_OUT_SHIFT (2U)
#define GPIO2_OUT_SHIFT (1U)
#define GPIO1_OUT_SHIFT (0U)
#define GPIO6_OUT_MASK  (0x01U << GPIO6_OUT_SHIFT)
#define GPIO5_OUT_MASK  (0x01U << GPIO5_OUT_SHIFT)
#define GPIO4_OUT_MASK  (0x01U << GPIO4_OUT_SHIFT)
#define GPIO3_OUT_MASK  (0x01U << GPIO3_OUT_SHIFT)
#define GPIO2_OUT_MASK  (0x01U << GPIO2_OUT_SHIFT)
#define GPIO1_OUT_MASK  (0x01U << GPIO1_OUT_SHIFT)

// GPIO_IN_1
#define GPIO6_IN_SHIFT (5U)
#define GPIO5_IN_SHIFT (4U)
#define GPIO4_IN_SHIFT (3U)
#define GPIO3_IN_SHIFT (2U)
#define GPIO2_IN_SHIFT (1U)
#define GPIO1_IN_SHIFT (0U)
#define GPIO6_IN_MASK  (0x01U << GPIO6_IN_SHIFT)
#define GPIO5_IN_MASK  (0x01U << GPIO5_IN_SHIFT)
#define GPIO4_IN_MASK  (0x01U << GPIO4_IN_SHIFT)
#define GPIO3_IN_MASK  (0x01U << GPIO3_IN_SHIFT)
#define GPIO2_IN_MASK  (0x01U << GPIO2_IN_SHIFT)
#define GPIO1_IN_MASK  (0x01U << GPIO1_IN_SHIFT)

// ENABLE_DRV_REG
#define ENABLE_DRV_SHIFT (0U)
#define ENABLE_DRV_MASK  (0x01U << ENABLE_DRV_SHIFT)

// ENABLE_DRV_STAT
#define TSD_DISABLE_SHIFT      (5U)
#define FORCE_EN_DRV_LOW_SHIFT (3U)
#define NRSTOUT_IN_SHIFT       (1U)
#define NINT_EN_DRV_IN_SHIFT   (0U)
#define TSD_DISABLE_MASK       (0x01U << TSD_DISABLE_SHIFT)
#define FORCE_EN_DRV_LOW_MASK  (0x01U << FORCE_EN_DRV_LOW_SHIFT)
#define NRSTOUT_IN_MASK        (0x01U << NRSTOUT_IN_SHIFT)
#define NINT_EN_DRV_IN_MASK    (0x01U << NINT_EN_DRV_IN_SHIFT)

// STAT_STARTUP
#define PB_LEVEL_STAT_SHIFT (2U)
#define ENABLE_STAT_SHIFT   (1U)
#define VSENSE_STAT_SHIFT   (0U)
#define PB_LEVEL_STAT_MASK  (0x01U << PB_LEVEL_STAT_SHIFT)
#define ENABLE_STAT_MASK    (0x01U << ENABLE_STAT_SHIFT)
#define VSENSE_STAT_MASK    (0x01U << VSENSE_STAT_SHIFT)

#ifdef __cplusplus
}

#endif /* __cplusplus */
#endif /* REGMAP_GPIO_H */
