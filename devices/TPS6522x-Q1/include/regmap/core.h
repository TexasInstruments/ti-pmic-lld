/******************************************************************************
 * Copyright (c) 2026 Texas Instruments Incorporated - http://www.ti.com
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
 * @file core.h
 *
 * @brief PMIC LLD Core module register addresses and bit fields.
 */
#ifndef REGMAP_CORE_H
#define REGMAP_CORE_H

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

#define DEV_REV_REG              ((uint16_t)0x01U)
#define NVM_CODE_1_REG           ((uint16_t)0x02U)
#define NVM_CODE_2_REG           ((uint16_t)0x03U)
#define CONFIG_1_REG             ((uint16_t)0x7DU)
#define CONFIG_2_REG             ((uint16_t)0x7EU)
#define MISC_CTRL_REG            ((uint16_t)0x81U)
#define REGISTER_LOCK_REG        ((uint16_t)0xA1U)
#define MANUFACTURING_VER_REG    ((uint16_t)0xA6U)
#define CUSTOMER_NVM_ID_REG_REG  ((uint16_t)0xA7U)
#define SCRATCH_PAD_REG_1_REG    ((uint16_t)0xC9U)
#define SCRATCH_PAD_REG_2_REG    ((uint16_t)0xCAU)
#define SCRATCH_PAD_REG_3_REG    ((uint16_t)0xCBU)
#define SCRATCH_PAD_REG_4_REG    ((uint16_t)0xCCU)
#define CRC_CALC_CONTROL_REG     ((uint16_t)0xEFU)
#define REGMAP_USER_CRC_LOW_REG  ((uint16_t)0xF0U)
#define REGMAP_USER_CRC_HIGH_REG ((uint16_t)0xF1U)

/* ========================================================================== */
/*                              Register Bit Fields                           */
/* ========================================================================== */

// DEV_REV
#define TI_DEVICE_ID_SHIFT (0U)
#define TI_DEVICE_ID_MASK  (0xFFU << TI_DEVICE_ID_SHIFT)

// NVM_CODE_1
#define TI_NVM_ID_SHIFT (0U)
#define TI_NVM_ID_MASK  (0xFFU << TI_NVM_ID_SHIFT)

// NVM_CODE_2
#define TI_NVM_REV_SHIFT (0U)
#define TI_NVM_REV_MASK  (0xFFU << TI_NVM_REV_SHIFT)

// CONFIG_1
#define NSLEEP2_MASK_SHIFT  (7U)
#define NSLEEP1_MASK_SHIFT  (6U)
#define I2C2_HS_SHIFT       (4U)
#define I2C1_HS_SHIFT       (3U)
#define TSD_ORD_LEVEL_SHIFT (1U)
#define TWARN_LEVEL_SHIFT   (0U)
#define NSLEEP2_MASK_MASK   (0x01UL << NSLEEP2_MASK_SHIFT)
#define NSLEEP1_MASK_MASK   (0x01UL << NSLEEP1_MASK_SHIFT)
#define I2C2_HS_MASK        (0x01UL << I2C2_HS_SHIFT)
#define I2C1_HS_MASK        (0x01UL << I2C1_HS_SHIFT)
#define TSD_ORD_LEVEL_MASK  (0x01UL << TSD_ORD_LEVEL_SHIFT)
#define TWARN_LEVEL_MASK    (0x01UL << TWARN_LEVEL_SHIFT)

// CONFIG_2
#define I2C2_CRC_EN_SHIFT     (5U)
#define I2C1_SPI_CRC_EN_SHIFT (4U)
#define I2C2_CRC_EN_MASK      (0x01UL << I2C2_CRC_EN_SHIFT)
#define I2C1_SPI_CRC_EN_MASK  (0x01UL << I2C1_SPI_CRC_EN_SHIFT)

// MISC_CTRL
#define SEL_EXT_CLK_SHIFT (5U)
#define LPM_EN_SHIFT      (2U)
#define NRSTOUT_SHIFT     (0U)
#define SEL_EXT_CLK_MASK  (0x01UL << SEL_EXT_CLK_SHIFT)
#define LPM_EN_MASK       (0x01UL << LPM_EN_SHIFT)
#define NRSTOUT_MASK      (0x01UL << NRSTOUT_SHIFT)

// REGISTER_LOCK
#define REGISTER_LOCK_STATUS_SHIFT (0U)
#define REGISTER_LOCK_STATUS_MASK  (0x01UL << REGISTER_LOCK_STATUS_SHIFT)

// MANUFACTURING_VER
#define SILICON_REV_SHIFT (0U)
#define SILICON_REV_MASK  (0xFFU << SILICON_REV_SHIFT)

// SCRATCH_PAD_REG_1
#define SCRATCH_PAD_1_SHIFT (0U)
#define SCRATCH_PAD_1_MASK  (0xFFU << SCRATCH_PAD_1_SHIFT)

// SCRATCH_PAD_REG_2
#define SCRATCH_PAD_2_SHIFT (0U)
#define SCRATCH_PAD_2_MASK  (0xFFU << SCRATCH_PAD_2_SHIFT)

// SCRATCH_PAD_REG_3
#define SCRATCH_PAD_3_SHIFT (0U)
#define SCRATCH_PAD_3_MASK  (0xFFU << SCRATCH_PAD_3_SHIFT)

// SCRATCH_PAD_REG_4
#define SCRATCH_PAD_4_SHIFT (0U)
#define SCRATCH_PAD_4_MASK  (0xFFU << SCRATCH_PAD_4_SHIFT)

/* CRC_CALC_CONTROL */
#define RUN_CRC_UPDATE_SHIFT (1U)
#define RUN_CRC_BIST_SHIFT   (0U)
#define RUN_CRC_UPDATE_MASK  (1UL << RUN_CRC_UPDATE_SHIFT)
#define RUN_CRC_BIST_MASK    (1UL << RUN_CRC_BIST_SHIFT)

/* REGMAP_USER_CRC_LOW */
#define REGMAP_USER_CRC16_LOW_SHIFT (0U)
#define REGMAP_USER_CRC16_LOW_MASK  (0xFFUL << REGMAP_USER_CRC16_LOW_SHIFT)

/* REGMAP_USER_CRC_HIGH */
#define REGMAP_USER_CRC16_HIGH_SHIFT (0U)
#define REGMAP_USER_CRC16_HIGH_MASK  (0xFFUL << REGMAP_USER_CRC16_HIGH_SHIFT)

#ifdef __cplusplus
}

#endif /* __cplusplus */
#endif /* REGMAP_CORE_H */
