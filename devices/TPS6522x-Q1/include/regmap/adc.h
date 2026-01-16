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
 * @file adc.h
 *
 * @brief PMIC LLD ADC module register addresses and bit fields.
 */
#ifndef REGMAP_ADC_H
#define REGMAP_ADC_H

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

#define ADC_CTRL_REG         ((uint16_t)0xACU)
#define ADC_RESULT_REG_1_REG ((uint16_t)0xADU)
#define ADC_RESULT_REG_2_REG ((uint16_t)0xAEU)

/* ========================================================================== */
/*                              Register Bit Fields                           */
/* ========================================================================== */

// ADC_CTRL
#define ADC_STATUS_SHIFT      (7U)
#define ADC_RDIV_EN_SHIFT     (3U)
#define ADC_THERMAL_SEL_SHIFT (2U)
#define ADC_CONT_CONV_SHIFT   (1U)
#define ADC_START_SHIFT       (0U)
#define ADC_STATUS_MASK       (1U << ADC_STATUS_SHIFT)
#define ADC_RDIV_EN_MASK      (1U << ADC_RDIV_EN_SHIFT)
#define ADC_THERMAL_SEL_MASK  (1U << ADC_THERMAL_SEL_SHIFT)
#define ADC_CONT_CONV_MASK    (1U << ADC_CONT_CONV_SHIFT)
#define ADC_START_MASK        (1U << ADC_START_SHIFT)

// ADC_RESULT_REG_1
#define ADC_RESULT_11_4_SHIFT (0U)
#define ADC_RESULT_11_4_MASK  (0xFFU << 0U)

// ADC_RESULT_REG_2
#define ADC_RESULT_3_0_SHIFT (4U)
#define ADC_RESULT_3_0_MASK  (0x0FU << ADC_RESULT_3_0_SHIFT)

#ifdef __cplusplus
}

#endif /* __cplusplus */
#endif /* REGMAP_ADC_H */
