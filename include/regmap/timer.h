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
#ifndef __PMIC_REGMAP_TIMER_H__
#define __PMIC_REGMAP_TIMER_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief PMIC Timer module register addresses */
#define TMR_CFG_REG        (0x6AU)
#define TMR_LP_WAKE0_REG   (0x6BU)
#define TMR_LP_WAKE1_REG   (0x6CU)
#define TMR_LP_WAKE2_REG   (0x6DU)
#define TMR_CNT0_REG       (0x6EU)
#define TMR_CNT1_REG       (0x6FU)
#define TMR_CNT2_REG       (0x70U)

/** @brief Macros for sequential register lengths */
#define TMR_LP_WAKE_REG_CNT (3U)
#define TMR_CNT_REG_CNT     (3U)

/** @brief TMR_CFG_REG - Timer Configuration Register */
#define TMR_CFG_SHIFT  (0U)
#define TMR_PS_SHIFT   (3U)
#define TMR_CLR_SHIFT  (5U)
#define TMR_CFG_MASK   (7U << TMR_CFG_SHIFT)
#define TMR_PS_MASK    (3U << TMR_PS_SHIFT)
#define TMR_CLR_MASK   (1U << TMR_CLR_SHIFT)

/** @brief TMR_LP_WAKE0 - Timer Low Power Wake 0 */
#define TMR_LP_WAKE_B0_SHIFT  (0U)
#define TMR_LP_WAKE_B0_MASK   (0xFFU << TMR_LP_WAKE_B0_SHIFT)

/** @brief TMR_LP_WAKE1 - Timer Low Power Wake 1 */
#define TMR_LP_WAKE_B1_SHIFT  (0U)
#define TMR_LP_WAKE_B1_MASK   (0xFFU << TMR_LP_WAKE_B1_SHIFT)

/** @brief TMR_LP_WAKE2 - Timer Low Power Wake 2 */
#define TMR_LP_WAKE_B2_SHIFT  (0U)
#define TMR_LP_WAKE_B2_MASK   (0xFFU << TMR_LP_WAKE_B2_SHIFT)

/** @brief TMR_CNT0 - Timer Count 0 */
#define TMR_CNT_B0_SHIFT      (0U)
#define TMR_CNT_B0_MASK       (0xFFU << TMR_CNT_B0_SHIFT)

/** @brief TMR_CNT1 - Timer Count 1 */
#define TMR_CNT_B1_SHIFT      (0U)
#define TMR_CNT_B1_MASK       (0xFFU << TMR_CNT_B1_SHIFT)

/** @brief TMR_CNT2 - Timer Count 2 */
#define TMR_CNT_B2_SHIFT      (0U)
#define TMR_CNT_B2_MASK       (0xFFU << TMR_CNT_B2_SHIFT)

#ifdef __cplusplus
}
#endif
#endif /* __PMIC_REGMAP_TIMER_H__ */
