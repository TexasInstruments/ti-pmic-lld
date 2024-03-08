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
*   @file    pmic_ilim_priv.h
*
*   @brief   This file contains the private MACRO's and function definitions for
*            PMIC ILIM configuration
*
*/

#ifndef PMIC_INC_ILIM_PRIV_H_
#define PMIC_INC_ILIM_PRIV_H_

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/*!
 * \brief  PMIC ILIM register Addresses
 */
#define PMIC_ILIM_CFG_REGADDR                       (0x60U)
#define PMIC_ILIM_DGL_CFG_REGADDR                   (0x61U)
#define PMIC_ILIM_STAT_REGADDR                      (0x62U)

/*!
 * \brief  PMIC ILIM Config register Shift Values
 */
#define PMIC_LDO1_ILIM_CFG_SHIFT           (0x00U)
#define PMIC_LDO2_ILIM_CFG_SHIFT           (0x01U)
#define PMIC_LDO3_ILIM_CFG_SHIFT           (0x02U)
#define PMIC_LDO4_ILIM_CFG_SHIFT           (0x03U)
#define PMIC_PLDO1_ILIM_CFG_SHIFT          (0x04U)
#define PMIC_PLDO2_ILIM_CFG_SHIFT          (0x05U)

/*!
 * \brief  PMIC ILIM_DGL_CFG register Shift Values
 */
#define PMIC_LDO1_ILIM_DGL_CFG_SHIFT   (0x00U)
#define PMIC_LDO2_ILIM_DGL_CFG_SHIFT   (0x01U)
#define PMIC_LDO3_ILIM_DGL_CFG_SHIFT   (0x02U)
#define PMIC_LDO4_ILIM_DGL_CFG_SHIFT   (0x03U)
#define PMIC_PLDO1_ILIM_DGL_CFG_SHIFT  (0x04U)
#define PMIC_PLDO2_ILIM_DGL_CFG_SHIFT  (0x05U)

/*!
 * \brief  PMIC ILIM_STAT register Shift Values
 */
#define PMIC_LDO1_ILIM_ERR_SHIFT          (0x00U)
#define PMIC_LDO2_ILIM_ERR_SHIFT          (0x01U)
#define PMIC_LDO3_ILIM_ERR_SHIFT          (0x02U)
#define PMIC_LDO4_ILIM_ERR_SHIFT          (0x03U)
#define PMIC_PLDO1_ILIM_ERR_SHIFT         (0x04U)
#define PMIC_PLDO2_ILIM_ERR_SHIFT         (0x05U)
#define PMIC_BB_AVG_ILIM_ERR_SHIFT        (0x06U)

/*!
 * \brief  PMIC ILIM CFG register Mask Values
 */
#define PMIC_LDO1_ILIM_CFG_MASK                        \
                  ((uint8_t)(0x01U << PMIC_LDO1_ILIM_CFG_SHIFT))
#define PMIC_LDO2_ILIM_CFG_MASK                        \
                  ((uint8_t)(0x01U << PMIC_LDO2_ILIM_CFG_SHIFT))
#define PMIC_LDO3_ILIM_CFG_MASK                        \
                  ((uint8_t)(0x01U << PMIC_LDO3_ILIM_CFG_SHIFT))
#define PMIC_LDO4_ILIM_CFG_MASK                        \
                  ((uint8_t)(0x01U << PMIC_LDO4_ILIM_CFG_SHIFT))
#define PMIC_PLDO1_ILIM_CFG_MASK                        \
                  ((uint8_t)(0x01U << PMIC_PLDO1_ILIM_CFG_SHIFT))
#define PMIC_PLDO2_ILIM_CFG_MASK                        \
                  ((uint8_t)(0x01U << PMIC_PLDO2_ILIM_CFG_SHIFT))

/*!
 * \brief  PMIC ILIM DGL CFG register Mask Values
 */
#define PMIC_LDO1_ILIM_DGL_CFG_MASK                        \
                  ((uint8_t)(0x01U << PMIC_LDO1_ILIM_DGL_CFG_SHIFT))
#define PMIC_LDO2_ILIM_DGL_CFG_MASK                        \
                  ((uint8_t)(0x01U << PMIC_LDO2_ILIM_DGL_CFG_SHIFT))
#define PMIC_LDO3_ILIM_DGL_CFG_MASK                        \
                  ((uint8_t)(0x01U << PMIC_LDO3_ILIM_DGL_CFG_SHIFT))
#define PMIC_LDO4_ILIM_DGL_CFG_MASK                        \
                  ((uint8_t)(0x01U << PMIC_LDO4_ILIM_DGL_CFG_SHIFT))
#define PMIC_PLDO1_ILIM_DGL_CFG_MASK                        \
                  ((uint8_t)(0x01U << PMIC_PLDO1_ILIM_DGL_CFG_SHIFT))
#define PMIC_PLDO2_ILIM_DGL_CFG_MASK                        \
                  ((uint8_t)(0x01U << PMIC_PLDO2_ILIM_DGL_CFG_SHIFT))

/*!
 * \brief  PMIC ILIM_STAT register Mask Values
 */
#define PMIC_LDO1_ILIM_ERR_MASK                        \
                  ((uint8_t)(0x01U << PMIC_LDO1_ILIM_ERR_SHIFT))
#define PMIC_LDO2_ILIM_ERR_MASK                        \
                  ((uint8_t)(0x01U << PMIC_LDO2_ILIM_ERR_SHIFT))
#define PMIC_LDO3_ILIM_ERR_MASK                        \
                  ((uint8_t)(0x01U << PMIC_LDO3_ILIM_ERR_SHIFT))
#define PMIC_LDO4_ILIM_ERR_MASK                        \
                  ((uint8_t)(0x01U << PMIC_LDO4_ILIM_ERR_SHIFT))
#define PMIC_PLDO1_ILIM_ERR_MASK                        \
                  ((uint8_t)(0x01U << PMIC_PLDO1_ILIM_ERR_SHIFT))
#define PMIC_PLDO2_ILIM_ERR_MASK                        \
                  ((uint8_t)(0x01U << PMIC_PLDO2_ILIM_ERR_SHIFT))
#define PMIC_BB_AVG_ILIM_ERR_MASK                        \
                  ((uint8_t)(0x01U << PMIC_BB_AVG_ILIM_ERR_SHIFT))

/* ILIM_CFG Data*/
#define PMIC_ILIM_CFG_DATA1         (0x00U) /* interrupt signal set (NINT low and STAT[0] bit set) upon error condition          */
#define PMIC_ILIM_CFG_DATA2         (0x01U) /* interrupt signal set (NINT low and STAT[0] bit set) and device in-activates the
                                               PLDO2 until the error condition is removed and the LDO1_ILIM_ERR flag is cleared  */
/* ILIM_DGL_CFG Data*/
#define PMIC_ILIM_DGL_CFG_DATA1     (0x00U) /* 10 µs (typical)     */
#define PMIC_ILIM_DGL_CFG_DATA2     (0x01U) /* 1  ms (typical)     */

/* ILIM_STAT Data*/
#define PMIC_ILIM_AFTER_CLEAR_DATA  (0x00U) /* Data after bit is cleared */
#define PMIC_ILIM_ERR_CLEAR_DATA    (0x01U) /* Write 1 to clear    */

#endif /* PMIC_INC_ILIM_PRIV_H_ */
