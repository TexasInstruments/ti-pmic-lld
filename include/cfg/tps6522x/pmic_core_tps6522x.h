/******************************************************************************
 * Copyright (c) 2023-2024 Texas Instruments Incorporated - http://www.ti.com
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
 *  \file pmic_core_tps6522x.h
 *
 *  \brief  The macro definitions for TPS6522x Burton PMIC driver specific
 *          PMIC common configuration
 */

#ifndef PMIC_CORE_TPS6522X_H_
#define PMIC_CORE_TPS6522X_H_

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

/**
 *  \anchor Pmic_Tps6522xBurton_ExtClk_Freq_Sel
 *  \name   PMIC External Clock (SYNCCLKIN) Frequency selection
 *
 *  @{
 */
/** \brief  SYNCCLKIN Frequency as 1.1 MHz */
#define PMIC_TPS6522X_SYNCCLKIN_1_1_MHZ (0U)
/** \brief  SYNCCLKIN Frequency as 2.2 MHz */
#define PMIC_TPS6522X_SYNCCLKIN_2_2_MHZ (1U)
/** \brief  SYNCCLKIN Frequency as 4.4 MHz */
#define PMIC_TPS6522X_SYNCCLKIN_4_4_MHZ (2U)
/** \brief  SYNCCLKIN Frequency as 8.8 MHz */
#define PMIC_TPS6522X_SYNCCLKIN_8_8_MHZ (3U)
/** @} */

/**
 *  \anchor Pmic_Tps6522xBurton_config2_regBitFields 
 *  \name PMIC CONFIG_2 Register Bit Field Masks and Shifts
 * 
 *  @{
 */
#define PMIC_TPS6522X_I2C2_CRC_EN_SHIFT         (5U)
#define PMIC_TPS6522X_I2C2_CRC_EN_MASK          (1U << PMIC_TPS6522X_I2C2_CRC_EN_SHIFT)
#define PMIC_TPS6522X_I2C1_SPI_CRC_EN_SHIFT     (4U)
#define PMIC_TPS6522X_I2C1_SPI_CRC_EN_MASK      (1U << PMIC_TPS6522X_I2C1_SPI_CRC_EN_SHIFT)
/** @} */
/*==========================================================================*/
/*                         Structures and Enums                             */
/*==========================================================================*/

/*==========================================================================*/
/*                         Function Declarations                            */
/*==========================================================================*/

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* PMIC_CORE_TPS6522X_H_ */
