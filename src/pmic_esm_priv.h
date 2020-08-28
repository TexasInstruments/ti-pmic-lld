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
 * \file   pmic_esm_priv.h
 *
 * \brief: This file contains macro definitions, structures and function
 *         prototypes for driver specific PMIC esm configuration
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
/*!
 * \brief  ESM Register Address
 */
/*! ESM MCU Register Base Address */
#define PMIC_ESM_MCU_BASE_REGADDR                   (0x8FU)

/*! ESM SOC Register Base Address */
#define PMIC_ESM_SOC_BASE_REGADDR                   (0x98U)

/*! ESM Register Offsets */
#define PMIC_ESM_START_REG_OFFSET                   (0x0U)
#define PMIC_ESM_DELAY1_REG_OFFSET                  (0x1U)
#define PMIC_ESM_DELAY2_REG_OFFSET                  (0x2U)
#define PMIC_ESM_MODE_CFG_REG_OFFSET                (0x3U)
#define PMIC_ESM_HMAX_REG_OFFSET                    (0x4U)
#define PMIC_ESM_HMIN_REG_OFFSET                    (0x5U)
#define PMIC_ESM_LMAX_REG_OFFSET                    (0x6U)
#define PMIC_ESM_LMIN_REG_OFFSET                    (0x7U)
#define PMIC_ESM_ERR_CNT_REG_OFFSET                 (0x8U)

/*!
 * \brief  ESM MCU and ESM SOC Register Bit fields
 */
#define PMIC_ESM_X_START_REG_ESM_X_START_SHIFT          (0x0U)
#define PMIC_ESM_X_DELAY1_REG_ESM_X_DELAY1_SHIFT        (0x0U)
#define PMIC_ESM_X_DELAY2_REG_ESM_X_DELAY2_SHIFT        (0x0U)
#define PMIC_ESM_X_MODE_CFG_ESM_X_MODE_SHIFT            (0x7U)
#define PMIC_ESM_X_MODE_CFG_ESM_X_EN_SHIFT              (0x6U)
#define PMIC_ESM_X_MODE_CFG_ESM_X_ENDRV_SHIFT           (0x5U)
#define PMIC_ESM_X_MODE_CFG_ESM_X_ERR_CNT_TH_SHIFT      (0x0U)
#define PMIC_ESM_X_HMAX_REG_ESM_X_HMAX_SHIFT            (0x0U)
#define PMIC_ESM_X_HMIN_REG_ESM_X_HMIN_SHIFT            (0x0U)
#define PMIC_ESM_X_LMAX_REG_ESM_X_LMAX_SHIFT            (0x0U)
#define PMIC_ESM_X_LMIN_REG_ESM_X_LMIN_SHIFT            (0x0U)
#define PMIC_ESM_X_ERR_CNT_REG_ESM_X_ERR_CNT_SHIFT      (0x0U)

/*!
 * \brief  ESM MCU and ESM SOC Register Bit masks
 */
#define PMIC_ESM_X_START_REG_ESM_X_START_MASK           ((uint8_t)   \
                       (0x1U << PMIC_ESM_X_START_REG_ESM_X_START_SHIFT))
#define PMIC_ESM_X_DELAY1_REG_ESM_X_DELAY1_MASK         ((uint8_t)   \
                       (0x7U << PMIC_ESM_X_DELAY1_REG_ESM_X_DELAY1_SHIFT))
#define PMIC_ESM_X_DELAY2_REG_ESM_X_DELAY2_MASK         ((uint8_t)   \
                       (0x7U << PMIC_ESM_X_DELAY2_REG_ESM_X_DELAY2_SHIFT))
#define PMIC_ESM_X_MODE_CFG_ESM_X_MODE_MASK             ((uint8_t)   \
                       (0x1U << PMIC_ESM_X_MODE_CFG_ESM_X_MODE_SHIFT))
#define PMIC_ESM_X_MODE_CFG_ESM_X_EN_MASK               ((uint8_t)   \
                       (0x1U << PMIC_ESM_X_MODE_CFG_ESM_X_EN_SHIFT))
#define PMIC_ESM_X_MODE_CFG_ESM_X_ENDRV_MASK            ((uint8_t)   \
                       (0x1U << PMIC_ESM_X_MODE_CFG_ESM_X_ENDRV_SHIFT))
#define PMIC_ESM_X_MODE_CFG_ESM_X_ERR_CNT_TH_MASK       ((uint8_t)   \
                       (0x0FU << PMIC_ESM_X_MODE_CFG_ESM_X_ERR_CNT_TH_SHIFT))
#define PMIC_ESM_X_HMAX_REG_ESM_X_HMAX_MASK             ((uint8_t)   \
                       (0x7U << PMIC_ESM_X_HMAX_REG_ESM_X_HMAX_SHIFT))
#define PMIC_ESM_X_HMIN_REG_ESM_X_HMIN_MASK             ((uint8_t)   \
                       (0x7U << PMIC_ESM_X_HMIN_REG_ESM_X_HMIN_SHIFT))
#define PMIC_ESM_X_LMAX_REG_ESM_X_LMAX_MASK             ((uint8_t)   \
                       (0x7U << PMIC_ESM_X_LMAX_REG_ESM_X_LMAX_SHIFT))
#define PMIC_ESM_X_LMIN_REG_ESM_X_LMIN_MASK             ((uint8_t)   \
                       (0x7U << PMIC_ESM_X_LMIN_REG_ESM_X_LMIN_SHIFT))
#define PMIC_ESM_X_ERR_CNT_REG_ESM_X_ERR_CNT_MASK       ((uint8_t)   \
                       (0x4U << PMIC_ESM_X_ERR_CNT_REG_ESM_X_ERR_CNT_SHIFT))

/*!
 * \brief  ESM Delay1 and Delay2 Time interval Max and Divisor macros
 */
#define PMIC_ESM_DELAY_MICROSEC_MAX             (522240U)
#define PMIC_ESM_DELAY_MICROSEC_DIV             (2048U)

/*!
 * \brief  ESM HMAX, HMIN, LMAX, LMIN Time interval Min, Max and Divisor macros
 */
#define PMIC_ESM_PWM_PULSE_MICROSEC_MIN                   (15U)
#define PMIC_ESM_PWM_PULSE_MICROSEC_MAX                   (3840U)
#define PMIC_ESM_PWM_PULSE_MICROSEC_DIV                   (15U)

/*!
 * \brief  ESM Error Count Threshold Max Value
 */
#define PMIC_ESM_ERR_CNT_THR_MAX                (15U)

#define PMIC_ESM_VAL_1                    (1U)

/*==========================================================================*/
/*                         Structures and Enums                             */
/*==========================================================================*/

/*==========================================================================*/
/*                         Function Declarations                            */
/*==========================================================================*/


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif/* PMIC_ESM_PRIV_H_ */
