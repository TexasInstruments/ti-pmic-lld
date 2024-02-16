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
 * @file   pmic_esm_priv.h
 *
 * @brief: This file contains macro definitions, structures and function
 *         prototypes for driver specific PMIC ESM configuration
 */

#ifndef PMIC_ESM_PRIV_H_
#define PMIC_ESM_PRIV_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/** ESM Register Offsets */
#define PMIC_ESM_CFG1_REG_OFFSET (0x01U)
#define PMIC_ESM_CFG2_REG_OFFSET (0x02U)
#define PMIC_ESM_INT_CFG_REG_OFFSET (0x03U)
#define PMIC_ESM_DELAY1_REG_OFFSET (0x04U)
#define PMIC_ESM_DELAY2_REG_OFFSET (0x05U)
#define PMIC_ESM_HMAX_REG_OFFSET (0x06U)
#define PMIC_ESM_HMIN_REG_OFFSET (0x07U)
#define PMIC_ESM_LMAX_REG_OFFSET (0x08U)
#define PMIC_ESM_LMIN_REG_OFFSET (0x09U)
#define PMIC_ESM_ERR_STAT_REG_OFFSET (0x10U)

/*Macros for ESM_CTRL*/
#define ESM_CTRL_REG (0x47U) // Enable ESM
#define ESM_CTRL_REG_SHIFT (0x00U)
#define ESM_CTRL_REG_MASK (0x01U)

/*Macros for ESM_CFG_1*/
#define ESM_CFG1_REG (0x48U)

#define ESM_CFG1_ERR_TH_SHIFT (0x00U)
#define ESM_CFG1_ESM_EN_SHIFT (0x06U)
#define ESM_CFG1_ESM_CFG_SHIFT (0x07U)

#define ESM_CFG1_ERR_TH_MASK (0x07U << ESM_CFG1_ERR_TH_SHIFT)
#define ESM_CFG1_ESM_EN_MASK (0x01U << ESM_CFG1_ESM_EN_SHIFT)
#define ESM_CFG1_ESM_CFG_MASK (0x01U << ESM_CFG1_ESM_CFG_SHIFT)

/*Macros for ESM_CFG_2*/
#define ESM_CFG2_REG (0x49U)

#define ESM_CFG2_TIME_CFG_SHIFT (0x00U)
#define ESM_CFG2_ESM_DGL_SHIFT (0x03U)
#define ESM_CFG2_ESM_LVL_POL_SHIFT (0x04U)

#define ESM_CFG2_TIME_CFG_MASK (0x03U << ESM_CFG2_TIME_CFG_SHIFT)
#define ESM_CFG2_ESM_DGL_MASK (0x01U << ESM_CFG2_ESM_DGL_SHIFT)
#define ESM_CFG2_ESM_LVL_POL_MASK (0x01U << ESM_CFG2_ESM_LVL_POL_SHIFT)

/*Macros for Interrupt*/
#define ESM_INT_CFG_REG (0x4AU)

#define ESM_INT_MASK_SHIFT (0X01U)
#define ESM_DLY1_INT_MASK_SHIFT (0X02U)
#define ESM_DLY1_INT_CFG_SHIFT (0X03U)
#define ESM_DLY2_INT_MASK_SHIFT (0X04U)
#define ESM_DLY2_INT_CFG_SHIFT (0X05U)

#define ESM_INT_MASK_MASK (0X01U << ESM_INT_MASK_SHIFT)
#define ESM_DLY1_INT_MASK_MASK (0X01U << ESM_DLY1_INT_MASK_SHIFT)
#define ESM_DLY1_INT_CFG_MASK (0X03U << ESM_DLY1_INT_CFG_SHIFT)
#define ESM_DLY2_INT_MASK_MASK (0X01U << ESM_DLY2_INT_MASK_SHIFT)
#define ESM_DLY2_INT_CFG_MASK (0X03U << ESM_DLY2_INT_CFG_SHIFT)

// Macro Delay Time
#define ESM_DELAY1_REG (0x4BU)
#define ESM_DELAY2_REG (0x4CU)

#define ESM_HMAX_CFG_REG (0x4DU)
#define ESM_HMIN_CFG_REG (0x4EU)
#define ESM_LMAX_CFG_REG (0x4FU)
#define ESM_LMIN_CFG_REG (0x50U)

/*Macros for Error Stat*/
#define ESM_ERR_STAT_REG (0x51U)

#define ESM_ERR_STAT_ESM_ERR_CNT_SHIFT (0X00U)
#define ESM_ERR_STAT_ESM_ERR_SHIFT (0X05U)
#define ESM_ERR_STAT_ESM_DLY1_ERR_SHIFT (0X06U)
#define ESM_ERR_STAT_ESM_DLY2_ERR_SHIFT (0X07U)

#define ESM_ERR_STAT_ESM_ERR_CNT_MASK (0X07U << ESM_ERR_STAT_ESM_ERR_CNT_SHIFT)
#define ESM_ERR_STAT_ESM_ERR_MASK (0X01U << ESM_ERR_STAT_ESM_ERR_SHIFT)
#define ESM_ERR_STAT_ESM_DLY1_ERR_MASK                                         \
  (0X01U << ESM_ERR_STAT_ESM_DLY1_ERR_SHIFT)
#define ESM_ERR_STAT_ESM_DLY2_ERR_MASK                                         \
  (0X01U << ESM_ERR_STAT_ESM_DLY2_ERR_SHIFT)

/**
 * \brief  ESM Error Count Threshold Max Value
 */
#define PMIC_ESM_ERR_CNT_THR_MAX (15U)

/**
 * \brief  ESM Delay1 and Delay2 Time interval Max and Divisor macros
 */
#define PMIC_ESM_DELAY_MICROSEC_MAX (522240U)
#define PMIC_ESM_DELAY_MICROSEC_DIV (2048U)

#define PMIC_ESM_PWM_PULSE_MICROSEC_MIN (15U)
#define PMIC_ESM_PWM_PULSE_MICROSEC_MAX (3840U)
#define PMIC_ESM_PWM_PULSE_MICROSEC_DIV (15U)

/**
 *  \anchor Pmic_EsmCflag
 *  \name PMIC Pmic_EsmCfg_s member configuration type
 *
 *  @{
 */
/** \brief validParams value used to set/get ESM delay-1 time interval */
#define PMIC_ESM_CFG_DELAY1_VALID (0x00U)
/** \brief validParams value used to set/get ESM delay-2 time interval  */
#define PMIC_ESM_CFG_DELAY2_VALID (0x01U)
/** \brief validParams value used to set/get ESM Error count Threshold value */
#define PMIC_ESM_CFG_ERR_CNT_THR_VALID (0x02U)
/** \brief validParams value used to set/get ESM Maximum high-pulse
 *         time-threshold value  */
#define PMIC_ESM_CFG_HMAX_VALID (0x03U)
/** \brief validParams value used to set/get ESM Minimum high-pulse
 *         time-threshold value  */
#define PMIC_ESM_CFG_HMIN_VALID (0x04U)
/** \brief validParams value used to set/get ESM Maximum low-pulse
 *         time-threshold value */
#define PMIC_ESM_CFG_LMAX_VALID (0x05U)
/** \brief validParams value used to set/get  ESM Minimum low-pulse
 *         time-threshold value */
#define PMIC_ESM_CFG_LMIN_VALID (0x06U)
/** \brief validParams value used to set/get ESM ENABLE_DRV clear configuration
 */
#define PMIC_ESM_CFG_EN_DRV_VALID (0x07U)
/** \brief validParams value used to set/get ESM mode */
#define PMIC_ESM_CFG_MODE_VALID (0x08U)
/*  @} */

/*==========================================================================*/
/*                         Structures and Enums                             */
/*==========================================================================*/

/*==========================================================================*/
/*                         Function Declarations                            */
/*==========================================================================*/

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* PMIC_ESM_PRIV_H_ */