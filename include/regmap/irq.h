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
#ifndef __IRQ_H__
#define __IRQ_H__

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                Include Files                               */
/* ========================================================================== */
#include <stdint.h>

/* ========================================================================== */
/*                      LP8772x IRQ Module Register Map                       */
/* ========================================================================== */
#define MASK_BUCK_12_REG             ((uint8_t)0x2DU)
#define MASK_BUCK3_LDO_LS1_VMON1_REG ((uint8_t)0x2EU)
#define MASK_LS2_VMON2_REG           ((uint8_t)0x2FU)
#define MASK_VCCA_REG                ((uint8_t)0x30U)
#define MASK_STARTUP_REG             ((uint8_t)0x31U)
#define MASK_MISC_REG                ((uint8_t)0x32U)
#define MASK_MODERATE_ERR_REG        ((uint8_t)0x33U)
#define MASK_COMM_ERR_REG            ((uint8_t)0x34U)
#define MASK_ESM_REG                 ((uint8_t)0x35U)
#define INT_TOP_REG                  ((uint8_t)0x46U)
#define INT_BUCK_LDO_LS1_VMON1_REG   ((uint8_t)0x47U)
#define INT_BUCK_12_REG              ((uint8_t)0x48U)
#define INT_BUCK3_LDO_LS1_VMON1_REG  ((uint8_t)0x49U)
#define INT_LS2_VMON2_REG            ((uint8_t)0x4AU)
#define INT_VCCA_REG                 ((uint8_t)0x4BU)
#define INT_STARTUP_REG              ((uint8_t)0x4CU)
#define INT_MISC_REG                 ((uint8_t)0x4DU)
#define INT_MODERATE_ERR_REG         ((uint8_t)0x4EU)
#define INT_SEVERE_ERR_REG           ((uint8_t)0x4FU)
#define INT_FSM_ERR_REG              ((uint8_t)0x50U)
#define INT_COMM_ERR_REG             ((uint8_t)0x51U)
#define INT_ESM_REG                  ((uint8_t)0x52U)
#define WD_ERR_STAT_REG              ((uint8_t)0x5EU)

/* ========================================================================== */
/*                               Macros & Typedefs                            */
/* ========================================================================== */

// INT_TOP_REG Register Bitfields
#define BUCK_LDO_LS1_VMON1_INT_SHIFT ((uint8_t)0U)
#define LS2_VMON2_INT_SHIFT          ((uint8_t)1U)
#define VCCA_INT_SHIFT               ((uint8_t)2U)
#define STARTUP_INT_SHIFT            ((uint8_t)3U)
#define MISC_INT_SHIFT               ((uint8_t)4U)
#define MODERATE_ERR_INT_SHIFT       ((uint8_t)5U)
#define SEVERE_ERR_INT_SHIFT         ((uint8_t)6U)
#define FSM_ERR_INT_SHIFT            ((uint8_t)7U)

// INT_BUCK_LDO_LS1_VMON1_REG Bitfields
#define BUCK1_INT_SHIFT              ((uint8_t)0U)
#define BUCK2_INT_SHIFT              ((uint8_t)1U)
#define BUCK3_INT_SHIFT              ((uint8_t)2U)
#define LDO_LS1_VMON1_INT_SHIFT      ((uint8_t)3U)
#define BUCK1_SC_INT_SHIFT           ((uint8_t)4U)
#define BUCK2_SC_INT_SHIFT           ((uint8_t)5U)
#define BUCK3_SC_INT_SHIFT           ((uint8_t)6U)
#define LDO_LS1_VMON1_SC_INT_SHIFT   ((uint8_t)7U)

// INT_BUCK12_REG and MASK_BUCK12_REG Bitfields
#define BUCK1_OV_INT_SHIFT           ((uint8_t)0U)
#define BUCK1_UV_INT_SHIFT           ((uint8_t)1U)
#define BUCK1_RV_INT_SHIFT           ((uint8_t)2U)
#define BUCK1_ILIM_INT_SHIFT         ((uint8_t)3U)
#define BUCK2_OV_INT_SHIFT           ((uint8_t)4U)
#define BUCK2_UV_INT_SHIFT           ((uint8_t)5U)
#define BUCK2_RV_INT_SHIFT           ((uint8_t)6U)
#define BUCK2_ILIM_INT_SHIFT         ((uint8_t)7U)

// INT_BUCK3_LDO_LS1_VMON1_REG and MASK_BUCK3_LDO_LS1_VMON1_REG Bitfields
#define BUCK3_OV_INT_SHIFT           ((uint8_t)0U)
#define BUCK3_UV_INT_SHIFT           ((uint8_t)1U)
#define BUCK3_RV_INT_SHIFT           ((uint8_t)2U)
#define BUCK3_ILIM_INT_SHIFT         ((uint8_t)3U)
#define LDO_LS1_VMON1_OV_INT_SHIFT   ((uint8_t)4U)
#define LDO_LS1_VMON1_UV_INT_SHIFT   ((uint8_t)5U)
#define LDO_LS1_VMON1_RV_INT_SHIFT   ((uint8_t)6U)
#define LDO_LS1_VMON1_ILIM_INT_SHIFT ((uint8_t)7U)

// INT_LS2_VMON2_REG and MASK_LS2_VMON2 Bitfields
#define LS2_VMON2_OV_INT_SHIFT       ((uint8_t)0U)
#define LS2_VMON2_UV_INT_SHIFT       ((uint8_t)1U)
#define LS2_VMON2_RV_INT_SHIFT       ((uint8_t)2U)
#define LS2_VMON2_ILIM_INT_SHIFT     ((uint8_t)3U)
#define LS2_VMON2_SC_NMI_SHIFT       ((uint8_t)4U)

// INT_VCCA_REG and MASK_VCCA_REG Bitfields
#define VCCA_OV_INT_SHIFT            ((uint8_t)0U)
#define VCCA_UV_INT_SHIFT            ((uint8_t)1U)

// INT_STARTUP_REG and MASK_STARTUP_REG Bitfields
#define ENABLE_INT_SHIFT             ((uint8_t)1U)

// INT_MISC_REG and MASK_MISC_REG Bitfields
#define ABIST_FAIL_INT_SHIFT         ((uint8_t)2U)
#define BUCKS_VSET_ERR_INT_SHIFT     ((uint8_t)5U)
#define EXT_CLK_INT_SHIFT            ((uint8_t)6U)
#define TWARN_INT_SHIFT              ((uint8_t)7U)

// INT_MODERATE_ERR_REG and MASK_MODERATE_ERR_REG Bitfields
#define TSD_ORD_NMI_SHIFT            ((uint8_t)0U)
#define RECOV_CNT_NMI_SHIFT          ((uint8_t)1U)
#define TRIM_TEST_CRC_INT_SHIFT      ((uint8_t)2U)
#define CONFIG_CRC_INT_SHIFT         ((uint8_t)3U)
#define NINT_GPO_RDBK_INT_SHIFT      ((uint8_t)4U)
#define NRSTOUT_RDBK_INT_SHIFT       ((uint8_t)5U)

// INT_COMM_ERR_REG and MASK_COMM_ERR_REG Bitfields
#define COMM_FRM_ERR_INT_SHIFT       ((uint8_t)0U)
#define COMM_CRC_ERR_INT_SHIFT       ((uint8_t)1U)
#define COMM_ADR_ERR_INT_SHIFT       ((uint8_t)3U)
#define COMM_MCU_ERR_INT_SHIFT       ((uint8_t)4U)

// INT_SEVERE_ERR_REG Bitfields
#define TSD_IMM_NMI_SHIFT            ((uint8_t)0U)
#define VCCA_OVP_NMI_SHIFT           ((uint8_t)1U)

// INT_ESM_REG and MASK_ESM_REG Bitfields
#define ESM_MCU_PIN_INT_SHIFT        ((uint8_t)3U)
#define ESM_MCU_FAIL_INT_SHIFT       ((uint8_t)4U)
#define ESM_MCU_RST_INT_SHIFT        ((uint8_t)5U)

// INT_FSM_ERR_REG Bitfields
#define IMM_SHUTDOWN_INT_SHIFT       ((uint8_t)0U)
#define ORD_SHUTDOWN_INT_SHIFT       ((uint8_t)1U)
#define WARM_RESET_INT_SHIFT         ((uint8_t)2U)
#define REGULATOR_ERR_INT_SHIFT      ((uint8_t)3U)
#define WD_FIRST_NOK_INT_SHIFT       ((uint8_t)4U)
#define ESM_MCU_INT_SHIFT            ((uint8_t)5U)
#define COMM_ERR_INT_SHIFT           ((uint8_t)6U)
#define WD_INT_SHIFT                 ((uint8_t)7U)

// WD_ERR_STAT_REG Bitfields
#define WD_LONGWIN_TIMEOUT_INT_SHIFT ((uint8_t)0U)
#define WD_TIMEOUT_INT_SHIFT         ((uint8_t)1U)
#define WD_ANSWER_EARLY_INT_SHIFT    ((uint8_t)3U)
#define WD_SEQ_ERR_INT_SHIFT         ((uint8_t)4U)
#define WD_ANSWER_ERR_INT_SHIFT      ((uint8_t)5U)
#define WD_FAIL_INT_SHIFT            ((uint8_t)6U)
#define WD_RST_INT_SHIFT             ((uint8_t)7U)

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* __IRQ_H__ */
