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
#ifndef __PMIC_REGMAP_IRQ_H__
#define __PMIC_REGMAP_IRQ_H__

#ifdef __cplusplus
extern "C" {
#endif

// IRQ Status Registers
#define REG_STAT_REG                    ((uint16_t)0x09U)
#define RDBK_ERR_STAT_REG               ((uint16_t)0x0BU)
#define OFF_STATE_STAT1_REG             ((uint16_t)0x0FU)
#define OFF_STATE_STAT2_REG             ((uint16_t)0x10U)
#define WD_ERR_STAT_REG                 ((uint16_t)0x46U)
#define ESM_ERR_STAT_REG                ((uint16_t)0x51U)
#define DCDC_STAT_REG                   ((uint16_t)0x5AU)
#define VMON_LDO_STAT_REG               ((uint16_t)0x5CU)
#define VMON_PLDO_STAT_REG              ((uint16_t)0x5EU)
#define EXT_VMON_STAT_REG               ((uint16_t)0x5FU)
#define ILIM_STAT_REG                   ((uint16_t)0x62U)
#define BIST_STAT_REG                   ((uint16_t)0x65U)
#define DEV_ERR_STAT_REG                ((uint16_t)0x66U)
#define SPI_ERR_STAT_REG                ((uint16_t)0x67U)
#define CM_STAT1_REG                    ((uint16_t)0x77U)
#define CM_STAT2_REG                    ((uint16_t)0x78U)

// IRQ Clear Registers
#define OFF_STATE_CLR_REG               ((uint16_t)0x11U)

// IRQ Configuration and Mask Registers
#define SAFETY_CFG_REG                  ((uint16_t)0x07U)
#define RDBK_INT_MASK_CFG_REG           ((uint16_t)0x0CU)
#define RDBK_INT_CFG1_REG               ((uint16_t)0x0DU)
#define RDBK_INT_CFG2_REG               ((uint16_t)0x0EU)
#define OV_INT_MASK_REG                 ((uint16_t)0x34U)
#define OV_INT_CFG1_REG                 ((uint16_t)0x35U)
#define OV_INT_CFG2_REG                 ((uint16_t)0x36U)
#define OV_DCDC_CFG_REG                 ((uint16_t)0x37U)
#define UV_INT_MASK_REG                 ((uint16_t)0x38U)
#define UV_INT_CFG1_REG                 ((uint16_t)0x39U)
#define UV_INT_CFG2_REG                 ((uint16_t)0x3AU)
#define UV_DCDC_CFG_REG                 ((uint16_t)0x3BU)
#define WD_INT_CFG_REG                  ((uint16_t)0x42U)
#define ESM_INT_CFG_REG                 ((uint16_t)0x4AU)
#define ILIM_CFG_REG                    ((uint16_t)0x60U)
#define CM_COMP_INT_MASK_CFG_REG        ((uint16_t)0x79U)
#define CM_VMON_INT_MASK_CFG_REG        ((uint16_t)0x7AU)
#define CM_VMON_INT_CFG_REG             ((uint16_t)0x7BU)

// REG_STAT_REG Bit Shifts
#define CFG_REG_CRC_ERR_SHIFT           (0U)

// RDBK_ERR_STAT_REG Bit Shifts
#define NRST_RDBK_ERR_SHIFT             (0U)
#define SAFE_OUT1_RDBK_ERR_SHIFT        (1U)
#define EN_OUT_RDBK_ERR_SHIFT           (2U)
#define GPO1_RDBK_ERR_SHIFT             (3U)
#define GPO2_RDBK_ERR_SHIFT             (4U)
#define GPO3_RDBK_ERR_SHIFT             (5U)
#define GPO4_RDBK_ERR_SHIFT             (6U)

// OFF_STATE_STAT1_REG Bit Shifts
#define NORMAL_OFF_SHIFT                (0U)
#define OFF_INT_EVT_ERR_SHIFT           (1U)
#define OFF_PROT_EVT_SHIFT              (2U)
#define FIRST_PWR_ON_SHIFT              (4U)
#define CLK_ERR_SHIFT                   (5U)
#define INTERNAL_OV_SHIFT               (6U)
#define INIT_AN_TMO_SHIFT               (7U)

// OFF_STATE_STAT2_REG Bit Shifts
#define CRC_ERR_SHIFT                   (0U)
#define SYS_CLK_ERR_PROT_SHIFT          (1U)
#define RST_MCU_TMO_SHIFT               (2U)
#define BGXM_ERR_SHIFT                  (3U)
#define VBAT_OVP_ERR_SHIFT              (4U)
#define BB_OVP_ERR_SHIFT                (5U)
#define BB_BST_TMO_SHIFT                (6U)
#define BB_PK_ILIM_ERR_SHIFT            (7U)

// WD_ERR_STAT_REG Bit Shifts
#define WD_TMO_SHIFT                    (0U)
#define WD_TRIG_EARLY_SHIFT             (1U)
#define WD_ANSW_EARLY_SHIFT             (2U)
#define WD_SEQ_ERR_SHIFT                (3U)
#define WD_ANSW_ERR_SHIFT               (4U)
#define WD_LONGWIN_TMO_SHIFT            (5U)
#define WD_TH1_ERR_SHIFT                (6U)
#define WD_TH2_ERR_SHIFT                (7U)

// ESM_ERR_STAT_REG Bit Shifts
#define ESM_ERR_SHIFT                   (5U)
#define ESM_DLY1_ERR_SHIFT              (6U)
#define ESM_DLY2_ERR_SHIFT              (7U)

// DCDC_STAT_REG Bit Shifts
#define BB_UV_ERR_SHIFT                 (0U)
#define BB_OV_ERR_SHIFT                 (1U)

// VMON_LDO_STAT_REG Bit Shifts
#define LDO1_UV_ERR_SHIFT               (0U)
#define LDO1_OV_ERR_SHIFT               (1U)
#define LDO2_UV_ERR_SHIFT               (2U)
#define LDO2_OV_ERR_SHIFT               (3U)
#define LDO3_UV_ERR_SHIFT               (4U)
#define LDO3_OV_ERR_SHIFT               (5U)
#define LDO4_UV_ERR_SHIFT               (6U)
#define LDO4_OV_ERR_SHIFT               (7U)

// VMON_PLDO_STAT_REG Bit Shifts
#define PLDO1_UV_ERR_SHIFT              (0U)
#define PLDO1_OV_ERR_SHIFT              (1U)
#define PLDO2_UV_ERR_SHIFT              (2U)
#define PLDO2_OV_ERR_SHIFT              (3U)

// EXT_VMON_STAT_REG Bit Shifts
#define EXT_VMON1_UV_ERR_SHIFT          (0U)
#define EXT_VMON1_OV_ERR_SHIFT          (1U)
#define EXT_VMON2_UV_ERR_SHIFT          (2U)
#define EXT_VMON2_OV_ERR_SHIFT          (3U)

// ILIM_STAT_REG Bit Shifts
#define LDO1_ILIM_ERR_SHIFT             (0U)
#define LDO2_ILIM_ERR_SHIFT             (1U)
#define LDO3_ILIM_ERR_SHIFT             (2U)
#define LDO4_ILIM_ERR_SHIFT             (3U)
#define PLDO1_ILIM_ERR_SHIFT            (4U)
#define PLDO2_ILIM_ERR_SHIFT            (5U)
#define BB_AVG_ILIM_ERR_SHIFT           (6U)

// BIST_STAT_REG Bit Shifts
#define ABIST_ERR_SHIFT                 (2U)
#define LBIST_ERR_SHIFT                 (3U)
#define TMR_REG_ERR_SHIFT               (4U)
#define M_PMIC_HB_ERR_SHIFT             (5U)

// DEV_ERR_STAT_REG Bit Shifts
#define SAFE_ST_TMO_RST_ERR_SHIFT       (7U)

// SPI_ERR_STAT_REG Bit Shifts
#define SPI_CRC_ERR_SHIFT               (0U)
#define FRM_ERR_SHIFT                   (1U)
#define ADDR_ERR_SHIFT                  (2U)
#define SCLK_ERR_SHIFT                  (3U)

// CM_STAT1_REG Bit Shifts
#define COMP1_ERR_SHIFT                 (1U)
#define COMP1P_UV_ERR_SHIFT             (2U)
#define COMP1P_OV_ERR_SHIFT             (3U)
#define COMP1N_UV_ERR_SHIFT             (4U)
#define COMP1N_OV_ERR_SHIFT             (5U)

// CM_STAT2_REG Bit Shifts
#define COMP2_ERR_SHIFT                 (1U)
#define COMP2P_UV_ERR_SHIFT             (2U)
#define COMP2P_OV_ERR_SHIFT             (3U)
#define COMP2N_UV_ERR_SHIFT             (4U)
#define COMP2N_OV_ERR_SHIFT             (5U)

// OFF_STATE_CLR_REG Bit Shifts
#define OFF_STATE_STAT_CLR_SHIFT        (0U)

// SAFETY_CFG_REG Bit Shifts
#define CFG_REG_CRC_INT_CFG_SHIFT       (6U)
#define CFG_REG_CRC_INT_CFG_MASK        (1U << CFG_REG_CRC_INT_CFG_SHIFT)

// RDBK_INT_MASK_CFG_REG Bit Shifts
#define NRST_RDBK_INT_MASK_SHIFT        (0U)
#define SAFE_OUT1_RDBK_INT_MASK_SHIFT   (1U)
#define EN_OUT_RDBK_INT_MASK_SHIFT      (2U)
#define GPO1_RDBK_INT_MASK_SHIFT        (3U)
#define GPO2_RDBK_INT_MASK_SHIFT        (4U)
#define GPO3_RDBK_INT_MASK_SHIFT        (5U)
#define GPO4_RDBK_INT_MASK_SHIFT        (6U)

// RDBK_INT_CFG1_REG Bit Masks and Shifts
#define NRST_RDBK_INT_CFG_SHIFT         (0U)
#define SAFE_OUT1_RDBK_INT_CFG_SHIFT    (2U)
#define EN_OUT_RDBK_INT_CFG_SHIFT       (4U)
#define NRST_RDBK_INT_CFG_MASK          (0x3U << NRST_RDBK_INT_CFG_SHIFT)
#define SAFE_OUT1_RDBK_INT_CFG_MASK     (0x3U << SAFE_OUT1_RDBK_INT_CFG_SHIFT)
#define EN_OUT_RDBK_INT_CFG_MASK        (0x3U << EN_OUT_RDBK_INT_CFG_SHIFT)

// RDBK_INT_CFG2_REG Bit Masks and Shifts
#define GPO1_RDBK_INT_CFG_SHIFT         (0U)
#define GPO2_RDBK_INT_CFG_SHIFT         (2U)
#define GPO3_RDBK_INT_CFG_SHIFT         (4U)
#define GPO4_RDBK_INT_CFG_SHIFT         (6U)
#define GPO1_RDBK_INT_CFG_MASK          (0x3U << GPO1_RDBK_INT_CFG_SHIFT)
#define GPO2_RDBK_INT_CFG_MASK          (0x3U << GPO2_RDBK_INT_CFG_SHIFT)
#define GPO3_RDBK_INT_CFG_MASK          (0x3U << GPO3_RDBK_INT_CFG_SHIFT)
#define GPO4_RDBK_INT_CFG_MASK          (0x3U << GPO4_RDBK_INT_CFG_SHIFT)

// OV_INT_MASK_REG Bit Shifts
#define LDO1_OV_INT_MASK_SHIFT          (0U)
#define LDO2_OV_INT_MASK_SHIFT          (1U)
#define LDO3_OV_INT_MASK_SHIFT          (2U)
#define LDO4_OV_INT_MASK_SHIFT          (3U)
#define PLDO1_OV_INT_MASK_SHIFT         (4U)
#define PLDO2_OV_INT_MASK_SHIFT         (5U)
#define EXT_VMON1_OV_INT_MASK_SHIFT     (6U)
#define EXT_VMON2_OV_INT_MASK_SHIFT     (7U)

// OV_INT_CFG1_REG Bit Masks and Shifts
#define LDO1_OV_INT_CFG_SHIFT           (0U)
#define LDO2_OV_INT_CFG_SHIFT           (2U)
#define LDO3_OV_INT_CFG_SHIFT           (4U)
#define LDO4_OV_INT_CFG_SHIFT           (6U)
#define LDO1_OV_INT_CFG_MASK            (0x3U << LDO1_OV_INT_CFG_SHIFT)
#define LDO2_OV_INT_CFG_MASK            (0x3U << LDO2_OV_INT_CFG_SHIFT)
#define LDO3_OV_INT_CFG_MASK            (0x3U << LDO3_OV_INT_CFG_SHIFT)
#define LDO4_OV_INT_CFG_MASK            (0x3U << LDO4_OV_INT_CFG_SHIFT)

// OV_INT_CFG2_REG Bit Masks and Shifts
#define PLDO1_OV_INT_CFG_SHIFT          (0U)
#define PLDO2_OV_INT_CFG_SHIFT          (2U)
#define EXT_VMON1_OV_INT_CFG_SHIFT      (4U)
#define EXT_VMON2_OV_INT_CFG_SHIFT      (6U)
#define PLDO1_OV_INT_CFG_MASK           (0x3U << PLDO1_OV_INT_CFG_SHIFT)
#define PLDO2_OV_INT_CFG_MASK           (0x3U << PLDO2_OV_INT_CFG_SHIFT)
#define EXT_VMON1_OV_INT_CFG_MASK       (0x3U << EXT_VMON1_OV_INT_CFG_SHIFT)
#define EXT_VMON2_OV_INT_CFG_MASK       (0x3U << EXT_VMON2_OV_INT_CFG_SHIFT)

// OV_DCDC_CFG_REG Bit Masks and Shifts
#define BB_OV_INT_MASK_SHIFT            (0U)
#define BB_OV_INT_CFG_SHIFT             (1U)
#define BB_OV_INT_CFG_MASK              (0x3U << BB_OV_INT_CFG_SHIFT)

// UV_INT_MASK_REG Bit Masks and Shifts
#define LDO1_UV_INT_MASK_SHIFT          (0U)
#define LDO2_UV_INT_MASK_SHIFT          (1U)
#define LDO3_UV_INT_MASK_SHIFT          (2U)
#define LDO4_UV_INT_MASK_SHIFT          (3U)
#define PLDO1_UV_INT_MASK_SHIFT         (4U)
#define PLDO2_UV_INT_MASK_SHIFT         (5U)
#define EXT_VMON1_UV_INT_MASK_SHIFT     (6U)
#define EXT_VMON2_UV_INT_MASK_SHIFT     (7U)

// UV_INT_CFG1_REG Bit Masks and Shifts
#define LDO1_UV_INT_CFG_SHIFT           (0U)
#define LDO2_UV_INT_CFG_SHIFT           (2U)
#define LDO3_UV_INT_CFG_SHIFT           (4U)
#define LDO4_UV_INT_CFG_SHIFT           (6U)
#define LDO1_UV_INT_CFG_MASK            (0x3U << LDO1_UV_INT_CFG_SHIFT)
#define LDO2_UV_INT_CFG_MASK            (0x3U << LDO2_UV_INT_CFG_SHIFT)
#define LDO3_UV_INT_CFG_MASK            (0x3U << LDO3_UV_INT_CFG_SHIFT)
#define LDO4_UV_INT_CFG_MASK            (0x3U << LDO4_UV_INT_CFG_SHIFT)

// UV_INT_CFG2_REG Bit Masks and Shifts
#define PLDO1_UV_INT_CFG_SHIFT          (0U)
#define PLDO2_UV_INT_CFG_SHIFT          (2U)
#define EXT_VMON1_UV_INT_CFG_SHIFT      (4U)
#define EXT_VMON2_UV_INT_CFG_SHIFT      (6U)
#define PLDO1_UV_INT_CFG_MASK           (0x3U << PLDO1_UV_INT_CFG_SHIFT)
#define PLDO2_UV_INT_CFG_MASK           (0x3U << PLDO2_UV_INT_CFG_SHIFT)
#define EXT_VMON1_UV_INT_CFG_MASK       (0x3U << EXT_VMON1_UV_INT_CFG_SHIFT)
#define EXT_VMON2_UV_INT_CFG_MASK       (0x3U << EXT_VMON2_UV_INT_CFG_SHIFT)

// UV_DCDC_CFG_REG Bit Masks and Shifts
#define BB_UV_INT_MASK_SHIFT            (0U)
#define BB_UV_INT_CFG_SHIFT             (1U)
#define BB_UV_INT_CFG_MASK              (0x3U << BB_UV_INT_CFG_SHIFT)

// WD_INT_CFG_REG Bit Masks and Shifts
#define WD_TH1_INT_MASK_SHIFT           (0U)
#define WD_TH1_INT_CFG_SHIFT            (1U)
#define WD_TH2_INT_MASK_SHIFT           (4U)
#define WD_TH2_INT_CFG_SHIFT            (5U)
#define WD_TH1_INT_CFG_MASK             (0x3U << WD_TH1_INT_CFG_SHIFT)
#define WD_TH2_INT_CFG_MASK             (0x3U << WD_TH2_INT_CFG_SHIFT)

// ESM_INT_CFG_REG Bit Masks and Shifts
#define ESM_INT_MASK_SHIFT              (0U)
#define ESM_DLY1_INT_MASK_SHIFT         (1U)
#define ESM_DLY1_INT_CFG_SHIFT          (2U)
#define ESM_DLY2_INT_MASK_SHIFT         (4U)
#define ESM_DLY2_INT_CFG_SHIFT          (5U)
#define ESM_DLY1_INT_CFG_MASK           (0x3U << ESM_DLY1_INT_CFG_SHIFT)
#define ESM_DLY2_INT_CFG_MASK           (0x3U << ESM_DLY2_INT_CFG_SHIFT)

// ILIM_CFG_REG Bit Masks and Shifts
#define LDO1_ILIM_CFG_SHIFT             (0U)
#define LDO2_ILIM_CFG_SHIFT             (1U)
#define LDO3_ILIM_CFG_SHIFT             (2U)
#define LDO4_ILIM_CFG_SHIFT             (3U)
#define PLDO1_ILIM_CFG_SHIFT            (4U)
#define PLDO2_ILIM_CFG_SHIFT            (5U)
#define LDO1_ILIM_CFG_MASK              (1U << LDO1_ILIM_CFG_SHIFT)
#define LDO2_ILIM_CFG_MASK              (1U << LDO2_ILIM_CFG_SHIFT)
#define LDO3_ILIM_CFG_MASK              (1U << LDO3_ILIM_CFG_SHIFT)
#define LDO4_ILIM_CFG_MASK              (1U << LDO4_ILIM_CFG_SHIFT)
#define PLDO1_ILIM_CFG_MASK             (1U << PLDO1_ILIM_CFG_SHIFT)
#define PLDO2_ILIM_CFG_MASK             (1U << PLDO2_ILIM_CFG_SHIFT)

// CM_COMP_INT_MASK_CFG_REG Bit Masks and Shifts
#define COMP1_INT_MASK_SHIFT            (0U)
#define COMP2_INT_MASK_SHIFT            (1U)
#define COMP1_INT_CFG_SHIFT             (4U)
#define COMP2_INT_CFG_SHIFT             (6U)
#define COMP1_INT_CFG_MASK              (0x3U << COMP1_INT_CFG_SHIFT)
#define COMP2_INT_CFG_MASK              (0x3U << COMP1_INT_CFG_SHIFT)

// CM_VMON_INT_MASK_CFG_REG Bit Masks and Shifts
#define COMP1P_UV_INT_MASK_SHIFT        (0U)
#define COMP1P_OV_INT_MASK_SHIFT        (1U)
#define COMP1N_UV_INT_MASK_SHIFT        (2U)
#define COMP1N_OV_INT_MASK_SHIFT        (3U)
#define COMP2P_UV_INT_MASK_SHIFT        (4U)
#define COMP2P_OV_INT_MASK_SHIFT        (5U)
#define COMP2N_UV_INT_MASK_SHIFT        (6U)
#define COMP2N_OV_INT_MASK_SHIFT        (7U)

// CM_VMON_INT_CFG_REG Bit Masks and Shifts
#define COMP1P_UV_INT_CFG_SHIFT         (0U)
#define COMP1P_OV_INT_CFG_SHIFT         (1U)
#define COMP1N_UV_INT_CFG_SHIFT         (2U)
#define COMP1N_OV_INT_CFG_SHIFT         (3U)
#define COMP2P_UV_INT_CFG_SHIFT         (4U)
#define COMP2P_OV_INT_CFG_SHIFT         (5U)
#define COMP2N_UV_INT_CFG_SHIFT         (6U)
#define COMP2N_OV_INT_CFG_SHIFT         (7U)
#define COMP1P_UV_INT_CFG_MASK          (1U << COMP1P_UV_INT_CFG_SHIFT)
#define COMP1P_OV_INT_CFG_MASK          (1U << COMP1P_OV_INT_CFG_SHIFT)
#define COMP1N_UV_INT_CFG_MASK          (1U << COMP1N_UV_INT_CFG_SHIFT)
#define COMP1N_OV_INT_CFG_MASK          (1U << COMP1N_OV_INT_CFG_SHIFT)
#define COMP2P_UV_INT_CFG_MASK          (1U << COMP2P_UV_INT_CFG_SHIFT)
#define COMP2P_OV_INT_CFG_MASK          (1U << COMP2P_OV_INT_CFG_SHIFT)
#define COMP2N_UV_INT_CFG_MASK          (1U << COMP2N_UV_INT_CFG_SHIFT)
#define COMP2N_OV_INT_CFG_MASK          (1U << COMP2N_OV_INT_CFG_SHIFT)

#ifdef __cplusplus
}
#endif
#endif /* __PMIC_REGMAP_IRQ_H__ */
