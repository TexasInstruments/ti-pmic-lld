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
 *  \file   pmic_adc_priv.h
 *
 *  \brief  This file contains macro definitions, structures and function
 *          prototypes for driver specific PMIC ADC configuration
 */

#ifndef PMIC_ADC_PRIV_H_
#define PMIC_ADC_PRIV_H_

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

/*!
 *  \anchor Pmic_ADC_regAddr
 *  \name PMIC ADC Register Addresses
 *
 * @{
 */
#define PMIC_ADC_CTRL_REGADDR                      (0xACU)
#define PMIC_ADC_RESULT_REG_1_REGADDR              (0xADU)
#define PMIC_ADC_RESULT_REG_2_REGADDR              (0xAEU)
/* @} */

/*!
 *  \anchor Pmic_ADC_CTRL_shiftVal
    \name PMIC ADC_CTRL Register Shift Values
 */
#define PMIC_ADC_STATUS_SHIFT                 (4U)
#define PMIC_ADC_RDIV_EN_SHIFT                (3U)
#define PMIC_ADC_THERMAL_SEL_SHIFT            (2U)
#define PMIC_ADC_CONT_CONV_SHIFT              (1U)
#define PMIC_ADC_START_SHIFT                  (0U)
/* @} */

/*!
 *  \anchor Pmic_ADC_CTRL_maskVal
    \name PMIC ADC_CTRL Register Mask Values
 */
#define PMIC_ADC_STATUS_MASK                  (1U << PMIC_ADC_STATUS_SHIFT)
#define PMIC_ADC_RDIV_EN_MASK                 (1U << PMIC_ADC_RDIV_EN_SHIFT)
#define PMIC_ADC_THERMAL_SEL_MASK             (1U << PMIC_ADC_THERMAL_SEL_SHIFT)
#define PMIC_ADC_CONT_CONV_MASK               (1U << PMIC_ADC_CONT_CONV_SHIFT)
#define PMIC_ADC_START_MASK                   (1U << PMIC_ADC_START_SHIFT)
/* @} */

/*!
 *  \anchor Pmic_ADC_RESULT_REG_2_shiftVal
    \name PMIC ADC_RESULT_REG_2 Register Shift Value
 */
#define PMIC_ADC_RESULT_3_0_SHIFT (4U)

/*!
 *  \anchor Pmic_ADC_RESULT_REG_2_maskVal
    \name PMIC ADC_RESULT_REG_2 Register Mask Value
 */
#define PMIC_ADC_RESULT_3_0_MASK  (0xFU << PMIC_ADC_RESULT_3_0_SHIFT)

/*!
 *  \anchor Pmic_ADC_miscValues
    \name PMIC ADC Miscellaneous values/constants
 */
#define PMIC_ADC_START                             (1U)
#define PMIC_ADC_RESISTOR_DIVIDER_VAL              (6U)

/* ========================================================================== */
/*                            Structures and Enums                            */
/*==========================================================================  */

/* ========================================================================== */
/*                           Function Declarations                            */
/* ========================================================================== */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* PMIC_ADC_PRIV_H_ */
