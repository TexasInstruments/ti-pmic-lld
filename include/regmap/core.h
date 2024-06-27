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
#ifndef __PMIC_REGMAP_CORE_H__
#define __PMIC_REGMAP_CORE_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* PMIC Module Device Revision Infos */
#define PMIC_DEV_ID_REG   (0x01U)
#define PMIC_DEV_ID_SHIFT (0U)
#define PMIC_DEV_ID_MASK  ((uint8_t)(0xFFU << PMIC_DEV_ID_SHIFT))

/* PMIC Scratchpad register Addresses */
#define PMIC_SCRATCH_PAD_REG_1_REG ((uint8_t)0x0AU)
#define PMIC_SCRATCH_PAD_REG_2_REG ((uint8_t)0x0BU)
#define PMIC_SCRATCH_PAD_REG_3_REG ((uint8_t)0x0CU)
#define PMIC_SCRATCH_PAD_REG_4_REG ((uint8_t)0x0DU)

/* Register lock definitions */
#define PMIC_REGISTER_LOCK_REG          ((uint8_t)0x09U)
#define PMIC_REGISTER_LOCK_STATUS_SHIFT (0U)
#define PMIC_REGISTER_LOCK_STATUS_MASK  (1U << PMIC_REGISTER_LOCK_STATUS_SHIFT)

/* CONFIG_CRC_* register definitions */
#define PMIC_CONFIG_CRC_CONFIG_REG ((uint8_t)0x60U)
#define PMIC_CONFIG_CRC_REG_1_REG  ((uint8_t)0x44U)
#define PMIC_CONFIG_CRC_REG_2_REG  ((uint8_t)0x45U)

#define PMIC_CONFIG_CRC_EN_SHIFT     (0U)
#define PMIC_CONFIG_CRC_CALC_SHIFT   (1U)
#define PMIC_CONFIG_CRC_STATUS_SHIFT (2U)

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* __PMIC_REGMAP_CORE_H__ */
