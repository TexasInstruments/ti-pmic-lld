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
#ifndef PMIC_REGMAP_CORE_H
#define PMIC_REGMAP_CORE_H

/**
 * @file core.h
 *
 * @brief PMIC LLD register addresses and bit fields pertaining to the Core
 * module.
 */

/* ========================================================================== */
/*                                Include Files                               */
/* ========================================================================== */

#include "pmic_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                       LP8774x Core Module Register Map                     */
/* ========================================================================== */

// Core module register addresses
#define DEV_REV_REGADDR             (0x01U)
#define NVM_CODE_1_REGADDR          (0x02U)
#define NVM_CODE_2_REGADDR          (0x03U)
#define MANUFACTURING_VER_REGADDR   (0x04U)
#define REGISTER_LOCK_REGADDR       (0x0AU)
#define SCRATCH_PAD_REG_1_REGADDR   (0x0BU)
#define SCRATCH_PAD_REG_2_REGADDR   (0x0CU)
#define SCRATCH_PAD_REG_3_REGADDR   (0x0DU)
#define SCRATCH_PAD_REG_4_REGADDR   (0x0EU)
#define CLK_CONF_REGADDR            (0x1AU)
#define INTERFACE_CONF_REGADDR      (0x1BU)
#define CONFIG_1_REGADDR            (0x37U)
#define CONFIG_CRC_REG_1_REGADDR    (0x45U)
#define CONFIG_CRC_REG_2_REGADDR    (0x46U)
#define CONFIG_CRC_CONFIG_REGADDR   (0x62U)
#define CALCUL_CONFIG_CRC_1_REGADDR (0x63U)
#define CALCUL_CONFIG_CRC_2_REGADDR (0x64U)

// REGISTER_LOCK register bit field shifts and masks
#define REGISTER_LOCK_STATUS_SHIFT (0U)
#define REGISTER_LOCK_STATUS_MASK  (0x1U << REGISTER_LOCK_STATUS_SHIFT)

// CLK_CONF register bit field shifts and masks
#define EXT_CLK_FREQ_SHIFT    (0U)
#define EXT_CLK_FREQ_MASK     (0x3U << EXT_CLK_FREQ_SHIFT)
#define SEL_EXT_CLK_SHIFT     (2U)
#define SEL_EXT_CLK_MASK      (0x1U << SEL_EXT_CLK_SHIFT)
#define EXT_CLK_DET_DIS_SHIFT (3U)
#define EXT_CLK_DET_DIS_MASK  (0x1U << EXT_CLK_DET_DIS_SHIFT)
#define SS_EN_SHIFT           (7U)
#define SS_EN_MASK            (0x1U << SS_EN_SHIFT)

// INTERFACE_CONF register bit field shifts and masks
#define NINT_PU_EN_SHIFT    (0U)
#define NINT_PU_EN_MASK     (0x1U << NINT_PU_EN_SHIFT)
#define NINT_POL_SHIFT      (1U)
#define NINT_POL_MASK       (0x1U << NINT_POL_SHIFT)
#define NINT_OD_SHIFT       (2U)
#define NINT_OD_MASK        (0x1U << NINT_OD_SHIFT)
#define NRSTOUT_PU_EN_SHIFT (3U)
#define NRSTOUT_PU_EN_MASK  (0x1U << NRSTOUT_PU_EN_SHIFT)
#define NRSTOUT_POL_SHIFT   (4U)
#define NRSTOUT_POL_MASK    (0x1U << NRSTOUT_POL_SHIFT)
#define NRSTOUT_OD_SHIFT    (5U)
#define NRSTOUT_OD_MASK     (0x1U << NRSTOUT_OD_SHIFT)
#define NERR_PU_DIS_SHIFT   (6U)
#define NERR_PU_DIS_MASK    (0x1U << NERR_PU_DIS_SHIFT)
#define SPI_CRC_EN_SHIFT    (7U)
#define SPI_CRC_EN_MASK     (0x1U << SPI_CRC_EN_SHIFT)

// CONFIG_1 register bit field shifts and masks
#define TWARN_LEVEL_SHIFT           (0U)
#define TWARN_LEVEL_MASK            (0x1U << TWARN_LEVEL_SHIFT)
#define TSD_ORD_LEVEL_SHIFT         (1U)
#define TSD_ORD_LEVEL_MASK          (0x1U << TSD_ORD_LEVEL_SHIFT)
#define TWARN_CONFIG_SHIFT          (2U)
#define TWARN_CONFIG_MASK           (0x1U << TWARN_CONFIG_SHIFT)
#define BLOCK_ENB_CTRL_CONFIG_SHIFT (3U)
#define BLOCK_ENB_CTRL_CONFIG_MASK  (0x1U << BLOCK_ENB_CTRL_CONFIG_SHIFT)
#define NRSTOUT_EXT_SHIFT           (4U)
#define NRSTOUT_EXT_MASK            (0xFU << NRSTOUT_EXT_SHIFT)

// CONFIG_CRC_CONFIG register bit field shifts and masks
#define CONFIG_CRC_EN_SHIFT     (0U)
#define CONFIG_CRC_EN_MASK      (1U << CONFIG_CRC_EN_SHIFT)
#define CONFIG_CRC_STATUS_SHIFT (2U)
#define CONFIG_CRC_STATUS_MASK  (1U << CONFIG_CRC_STATUS_SHIFT)

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* PMIC_REGMAP_CORE_H */
