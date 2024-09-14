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
 * @file irq.h
 *
 * @brief PMIC LLD register bit fields pertaining to the IRQ module.
 */
#ifndef __IRQ_H__
#define __IRQ_H__

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                Include Files                               */
/* ========================================================================== */
#include "pmic_common.h"

/* ========================================================================== */
/*                               Macros & Typedefs                            */
/* ========================================================================== */

// SAFETY_STAT_1 register bit field shifts and masks.
#define VDD6_OT_FLAG_SHIFT      ((uint8_t)7U)
#define VDD6_OT_FLAG_MASK       ((uint8_t)(1U << VDD6_OT_FLAG_SHIFT))
#define VDD6_TP_SHIFT           ((uint8_t)6U)
#define VDD6_TP_MASK            ((uint8_t)(1U << VDD6_TP_SHIFT))
#define VDD5_ILIM_SHIFT         ((uint8_t)5U)
#define VDD5_ILIM_MASK          ((uint8_t)(1U << VDD5_ILIM_SHIFT))
#define VDD5_OT_FLAG_SHIFT      ((uint8_t)4U)
#define VDD5_OT_FLAG_MASK       ((uint8_t)(1U << VDD5_OT_FLAG_SHIFT))
#define VDD5_TP_SHIFT           ((uint8_t)3U)
#define VDD5_TP_MASK            ((uint8_t)(1U << VDD5_TP_SHIFT))
#define VDD_3P5_ILIM_SHIFT      ((uint8_t)2U)
#define VDD_3P5_ILIM_MASK       ((uint8_t)(1U << VDD_3P5_ILIM_SHIFT))
#define VDD_3P5_OT_FLAG_SHIFT   ((uint8_t)1U)
#define VDD_3P5_OT_FLAG_MASK    ((uint8_t)(1U << VDD_3P5_OT_FLAG_SHIFT))
#define VDD_3P5_TP_SHIFT        ((uint8_t)0U)
#define VDD_3P5_TP_MASK         ((uint8_t)(1U << VDD_3P5_TP_SHIFT))

// SAFETY_STAT_2 register bit field shifts and masks.
#define VDD6_ILIM_SHIFT         ((uint8_t)6U)
#define VDD6_ILIM_MASK          ((uint8_t)(1U << VDD6_ILIM_SHIFT))
#define VSOUT2_ILIM_SHIFT       ((uint8_t)5U)
#define VSOUT2_ILIM_MASK        ((uint8_t)(1U << VSOUT2_ILIM_SHIFT))
#define VSOUT2_OT_FLAG_SHIFT    ((uint8_t)4U)
#define VSOUT2_OT_FLAG_MASK     ((uint8_t)(1U << VSOUT2_OT_FLAG_SHIFT))
#define VSOUT2_TP_SHIFT         ((uint8_t)3U)
#define VSOUT2_TP_MASK          ((uint8_t)(1U << VSOUT2_TP_SHIFT))
#define VSOUT1_ILIM_SHIFT       ((uint8_t)2U)
#define VSOUT1_ILIM_MASK        ((uint8_t)(1U << VSOUT1_ILIM_SHIFT))
#define VSOUT1_OT_FLAG_SHIFT    ((uint8_t)1U)
#define VSOUT1_OT_FLAG_MASK     ((uint8_t)(1U << VSOUT1_OT_FLAG_SHIFT))
#define VSOUT1_TP_SHIFT         ((uint8_t)0U)
#define VSOUT1_TP_MASK          ((uint8_t)(1U << VSOUT1_TP_SHIFT))

// SAFETY_STAT_3 register bit field shifts and masks.
#define CFG_CRC_ERR_SHIFT       ((uint8_t)6U)
#define CFG_CRC_ERR_MASK        ((uint8_t)(1U << CFG_CRC_ERR_SHIFT))
#define EE_CRC_ERR_SHIFT        ((uint8_t)5U)
#define EE_CRC_ERR_MASK         ((uint8_t)(1U << EE_CRC_ERR_SHIFT))
#define NRES_ERR_SHIFT          ((uint8_t)4U)
#define NRES_ERR_MASK           ((uint8_t)(1U << NRES_ERR_SHIFT))
#define LBIST_ERR_SHIFT         ((uint8_t)3U)
#define LBIST_ERR_MASK          ((uint8_t)(1U << LBIST_ERR_SHIFT))
#define ABIST_ERR_SHIFT         ((uint8_t)2U)
#define ABIST_ERR_MASK          ((uint8_t)(1U << ABIST_ERR_SHIFT))
#define LBIST_DONE_SHIFT        ((uint8_t)1U)
#define LBIST_DONE_MASK         ((uint8_t)(1U << LBIST_DONE_SHIFT))
#define ABIST_DONE_SHIFT        ((uint8_t)0U)
#define ABIST_DONE_MASK         ((uint8_t)(1U << ABIST_DONE_SHIFT))

// SAFETY_STAT_4 register bit field shifts and masks.
#define SPI_ERR_SHIFT           ((uint8_t)6U)
#define SPI_ERR_MASK            ((uint8_t)(3U << SPI_ERR_SHIFT))
#define LO_LPMCLK_SHIFT         ((uint8_t)5U)
#define LO_LPMCLK_MASK          ((uint8_t)(1U << LO_LPMCLK_SHIFT))
#define ENDRV_ERR_SHIFT         ((uint8_t)1U)
#define ENDRV_ERR_MASK          ((uint8_t)(1U <<ENDRV_ERR_SHIFT))
#define TRIM_ERR_VMON_SHIFT     ((uint8_t)0U)
#define TRIM_ERR_VMON_MASK      ((uint8_t)(1U << TRIM_ERR_VMON_SHIFT))

// SAFETY_STAT_5 register bit field shifts and masks.
#define STATE_SHIFT             ((uint8_t)5U)
#define STATE_MASK              ((uint8_t)(3U << STATE_SHIFT))
#define ENDRV_RDBCK_SHIFT       ((uint8_t)4U)
#define ENDRV_RDBCK_MASK        ((uint8_t)(1U << ENDRV_RDBCK_SHIFT))
#define WD_FAIL_CNT_SHIFT       ((uint8_t)0U)
#define WD_FAIL_CNT_MASK        ((uint8_t)(7U << WD_FAIL_CNT_SHIFT))

// SAFETY_ERR_STAT_1 register bit field shifts and masks.
#define ERROR_PIN_FAIL_SHIFT    ((uint8_t)5U)
#define ERROR_PIN_FAIL_MASK     ((uint8_t)1U << ERROR_PIN_FAIL_SHIFT)
#define WD_FAIL_SHIFT           ((uint8_t)4U)
#define WD_FAIL_MASK            ((uint8_t)1U << WD_FAIL_SHIFT)
#define DEV_ERR_CNT_SHIFT       ((uint8_t)0U)
#define DEV_ERR_CNT_MASK        ((uint8_t)0xFU << DEV_ERR_CNT_SHIFT)

// SAFETY_ERR_STAT_2 register bit field shifts and masks.
#define DIAG_STATE_TO_SHIFT     ((uint8_t)4U)
#define DIAG_STATE_TO_MASK      ((uint8_t)1U << DIAG_STATE_TO_SHIFT)
#define MCU_ERR_CNT_SHIFT       ((uint8_t)0U)
#define MCU_ERR_CNT_MASK        ((uint8_t)0xFU << MCU_ERR_CNT_SHIFT)

// VMON_STAT_1 register bit field shifts and masks.
#define VBATL_OV_SHIFT         ((uint8_t)7U)
#define VBATL_OV_MASK          ((uint8_t)(1U << VBATL_OV_SHIFT))
#define VBATL_UV_SHIFT         ((uint8_t)6U)
#define VBATL_UV_MASK          ((uint8_t)(1U << VBATL_UV_SHIFT))
#define VCP_UV_SHIFT           ((uint8_t)5U)
#define VCP_UV_MASK            ((uint8_t)(1U << VCP_UV_SHIFT))

// VMON_STAT_2 register bit field shifts and masks.
#define VDD6_OV_SHIFT          ((uint8_t)7U)
#define VDD6_OV_MASK           ((uint8_t)(1U << VDD6_OV_SHIFT))
#define VDD6_UV_SHIFT          ((uint8_t)6U)
#define VDD6_UV_MASK           ((uint8_t)(1U << VDD6_UV_SHIFT))
#define VDD5_OV_SHIFT          ((uint8_t)5U)
#define VDD5_OV_MASK           ((uint8_t)(1U << VDD5_OV_SHIFT))
#define VDD5_UV_SHIFT          ((uint8_t)4U)
#define VDD5_UV_MASK           ((uint8_t)(1U << VDD5_UV_SHIFT))
#define VDD_3P5_OV_SHIFT       ((uint8_t)3U)
#define VDD_3P5_OV_MASK        ((uint8_t)(1U << VDD_3P5_OV_SHIFT))
#define VDD_3P5_UV_SHIFT       ((uint8_t)2U)
#define VDD_3P5_UV_MASK        ((uint8_t)(1U << VDD_3P5_UV_SHIFT))

// VMON_STAT_3 register bit field shifts and masks.
#define VREG_UV_SHIFT          ((uint8_t)5U)
#define VREG_UV_MASK           ((uint8_t)(1U << VREG_UV_SHIFT))
#define VDD6_LP_UV_SHIFT       ((uint8_t)4U)
#define VDD6_LP_UV_MASK        ((uint8_t)(1U << VDD6_LP_UV_SHIFT))
#define VSOUT2_OV_SHIFT        ((uint8_t)3U)
#define VSOUT2_OV_MASK         ((uint8_t)(1U << VSOUT2_OV_SHIFT))
#define VSOUT2_UV_SHIFT        ((uint8_t)2U)
#define VSOUT2_UV_MASK         ((uint8_t)(1U << VSOUT2_UV_SHIFT))
#define VSOUT1_OV_SHIFT        ((uint8_t)1U)
#define VSOUT1_OV_MASK         ((uint8_t)(1U << VSOUT1_OV_SHIFT))
#define VSOUT1_UV_SHIFT        ((uint8_t)0U)
#define VSOUT1_UV_MASK         ((uint8_t)(1U << VSOUT1_UV_SHIFT))

// SAM_STAT register bit field shifts and masks.
#define COS_COMP_SHIFT         ((uint8_t)7U)
#define COS_COMP_MASK          ((uint8_t)(1U << COS_COMP_SHIFT))
#define SIN_COMP_SHIFT         ((uint8_t)6U)
#define SIN_COMP_MASK          ((uint8_t)(1U << SIN_COMP_SHIFT))
#define SAM_ROT_DIR_SHIFT      ((uint8_t)5U)
#define SAM_ROT_DIR_MASK       ((uint8_t)(1U << SAM_ROT_DIR_SHIFT))
#define SAM_ROT_CHG_SHIFT      ((uint8_t)4U)
#define SAM_ROT_CHG_MASK       ((uint8_t)(1U << SAM_ROT_CHG_SHIFT))
#define COUNT_RD_ERR_SHIFT     ((uint8_t)3U)
#define COUNT_RD_ERR_MASK      ((uint8_t)(1U << COUNT_RD_ERR_SHIFT))
#define SAM_BIST_FAIL_SHIFT    ((uint8_t)2U)
#define SAM_BIST_FAIL_MASK     ((uint8_t)(1U << SAM_BIST_FAIL_SHIFT))
#define POWER_ON_RESET_SHIFT   ((uint8_t)1U)
#define POWER_ON_RESET_MASK    ((uint8_t)(1U << POWER_ON_RESET_SHIFT))
#define SAM_CNT_ERR_SHIFT      ((uint8_t)0U)
#define SAM_CNT_ERR_MASK       ((uint8_t)(1U << SAM_CNT_ERR_SHIFT))

// SAM_SIG_STAT register bit field shifts and masks.
#define COS_P_OV_SHIFT     ((uint8_t)7U)
#define COS_P_OV_MASK      ((uint8_t)(1U << COS_P_OV_SHIFT))
#define COS_P_UV_SHIFT     ((uint8_t)6U)
#define COS_P_UV_MASK      ((uint8_t)(1U << COS_P_UV_SHIFT))
#define COS_N_OV_SHIFT     ((uint8_t)5U)
#define COS_N_OV_MASK      ((uint8_t)(1U << COS_N_OV_SHIFT))
#define COS_N_UV_SHIFT     ((uint8_t)4U)
#define COS_N_UV_MASK      ((uint8_t)(1U << COS_N_UV_SHIFT))
#define SIN_P_OV_SHIFT     ((uint8_t)3U)
#define SIN_P_OV_MASK      ((uint8_t)(1U << SIN_P_OV_SHIFT))
#define SIN_P_UV_SHIFT     ((uint8_t)2U)
#define SIN_P_UV_MASK      ((uint8_t)(1U << SIN_P_UV_SHIFT))
#define SIN_N_OV_SHIFT     ((uint8_t)1U)
#define SIN_N_OV_MASK      ((uint8_t)(1U << SIN_N_OV_SHIFT))
#define SIN_N_UV_SHIFT     ((uint8_t)0U)
#define SIN_N_UV_MASK      ((uint8_t)(1U << SIN_N_UV_SHIFT))

// SPI_INV_TRAN_STAT register bit field shifts and masks.
#define INVALID_CMD_SHIFT  ((uint8_t)4U)
#define INVALID_CMD_MASK   ((uint8_t)(1U << INVALID_CMD_SHIFT))
#define UNDEF_CMD_SHIFT    ((uint8_t)3U)
#define UNDEF_CMD_MASK     ((uint8_t)(1U << UNDEF_CMD_SHIFT))
#define CRC_ERR_SHIFT      ((uint8_t)2U)
#define CRC_ERR_MASK       ((uint8_t)(1U << CRC_ERR_SHIFT))
#define LONG_FRM_SHIFT     ((uint8_t)1U)
#define LONG_FRM_MASK      ((uint8_t)(1U << LONG_FRM_SHIFT))
#define SHORT_FRM_SHIFT    ((uint8_t)0U)
#define SHORT_FRM_MASK     ((uint8_t)(1U << SHORT_FRM_SHIFT))

#ifdef __cplusplus
}

#endif /* __cplusplus */
#endif /* __IRQ_H__ */
