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
 * @file   pmic_irq.c
 *
 * @brief  This file contains the TPS65386 BB(Black-Bird) PMIC Interrupt APIs
 * definitions and structures.
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stddef.h>
#include <stdint.h>

#include "pmic.h"
#include "pmic_common.h"
#include "pmic_io.h"

#include "pmic_irq.h"
#include "regmap/irq.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
// PMIC IRQ invalid macros
#define PMIC_IRQ_INVALID_REG              ((uint16_t)0xFFFFU)
#define PMIC_IRQ_INVALID_MASK_SHIFT       ((uint8_t)0xFFU)

// Number of R/W1C status registers supported on this device
#define NUM_STAT_REGISTERS (16U)
#define NUM_MASK_REGISTERS (7U)
#define NUM_CONF_REGISTERS (13U)

// Helper macros for defining IRQ register/shift/mask linkages
#define IRQ_DEF_STD(sr, mr, ss, ms, cr, cs, cm) { sr, mr, ss, ms, cr, cs, cm }
#define IRQ_DEF_NMI_CFG(sr, ss, cr, cs, cm) {\
    sr, PMIC_IRQ_INVALID_REG, ss, PMIC_IRQ_INVALID_MASK_SHIFT,\
    cr, cs, cm }
#define IRQ_DEF_NMI_NCFG(sr, ss) {\
    sr, PMIC_IRQ_INVALID_REG, ss, PMIC_IRQ_INVALID_MASK_SHIFT,\
    PMIC_IRQ_INVALID_REG, PMIC_IRQ_INVALID_MASK_SHIFT, PMIC_IRQ_INVALID_MASK_SHIFT }

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */
typedef struct Pmic_IrqInfo_s {
    uint16_t statReg;
    uint16_t maskReg;
    uint8_t statShift;
    uint8_t maskShift;

    uint16_t confReg;
    uint8_t confShift;
    uint8_t confMask;
} Pmic_IrqInfo_t;

/* ========================================================================== */
/*                           Variables and Data                               */
/* ========================================================================== */
/* PMIC TPS65386x Interrupt Configuration as per Pmic_tps65386x_IrqNum. */
static const Pmic_IrqInfo_t IRQ[PMIC_IRQ_NUM] = {
    // PMIC_CFG_REG_CRC_ERR_INT
    IRQ_DEF_NMI_CFG(REG_STAT_REG,
                    CFG_REG_CRC_ERR_SHIFT,
                    SAFETY_CFG_REG,
                    CFG_REG_CRC_INT_CFG_SHIFT,
                    CFG_REG_CRC_INT_CFG_MASK),
    // PMIC_NRST_RDBK_ERR_INT
    IRQ_DEF_STD(RDBK_ERR_STAT_REG,
                RDBK_INT_MASK_CFG_REG,
                NRST_RDBK_ERR_SHIFT,
                NRST_RDBK_INT_MASK_SHIFT,
                RDBK_INT_CFG1_REG,
                NRST_RDBK_INT_CFG_SHIFT,
                NRST_RDBK_INT_CFG_MASK),
    // PMIC_SAFE_OUT1_RDBK_ERR_INT
    IRQ_DEF_STD(RDBK_ERR_STAT_REG,
                RDBK_INT_MASK_CFG_REG,
                SAFE_OUT1_RDBK_ERR_SHIFT,
                SAFE_OUT1_RDBK_INT_MASK_SHIFT,
                RDBK_INT_CFG1_REG,
                SAFE_OUT1_RDBK_INT_CFG_SHIFT,
                SAFE_OUT1_RDBK_INT_CFG_MASK),
    // PMIC_EN_OUT_RDBK_ERR_INT
    IRQ_DEF_STD(RDBK_ERR_STAT_REG,
                RDBK_INT_MASK_CFG_REG,
                EN_OUT_RDBK_ERR_SHIFT,
                EN_OUT_RDBK_INT_MASK_SHIFT,
                RDBK_INT_CFG1_REG,
                EN_OUT_RDBK_INT_CFG_SHIFT,
                EN_OUT_RDBK_INT_CFG_MASK),
    // PMIC_GPO1_RDBK_ERR_INT
    IRQ_DEF_STD(RDBK_ERR_STAT_REG,
                RDBK_INT_MASK_CFG_REG,
                GPO1_RDBK_ERR_SHIFT,
                GPO1_RDBK_INT_MASK_SHIFT,
                RDBK_INT_CFG2_REG,
                GPO1_RDBK_INT_CFG_SHIFT,
                GPO1_RDBK_INT_CFG_MASK),
    // PMIC_GPO2_RDBK_ERR_INT
    IRQ_DEF_STD(RDBK_ERR_STAT_REG,
                RDBK_INT_MASK_CFG_REG,
                GPO2_RDBK_ERR_SHIFT,
                GPO2_RDBK_INT_MASK_SHIFT,
                RDBK_INT_CFG2_REG,
                GPO2_RDBK_INT_CFG_SHIFT,
                GPO2_RDBK_INT_CFG_MASK),
    // PMIC_GPO3_RDBK_ERR_INT
    IRQ_DEF_STD(RDBK_ERR_STAT_REG,
                RDBK_INT_MASK_CFG_REG,
                GPO3_RDBK_ERR_SHIFT,
                GPO3_RDBK_INT_MASK_SHIFT,
                RDBK_INT_CFG2_REG,
                GPO3_RDBK_INT_CFG_SHIFT,
                GPO3_RDBK_INT_CFG_MASK),
    // PMIC_GPO4_RDBK_ERR_INT
    IRQ_DEF_STD(RDBK_ERR_STAT_REG,
                RDBK_INT_MASK_CFG_REG,
                GPO4_RDBK_ERR_SHIFT,
                GPO4_RDBK_INT_MASK_SHIFT,
                RDBK_INT_CFG2_REG,
                GPO4_RDBK_INT_CFG_SHIFT,
                GPO4_RDBK_INT_CFG_MASK),
    // PMIC_NORMAL_OFF_INT
    IRQ_DEF_NMI_NCFG(OFF_STATE_STAT1_REG, NORMAL_OFF_SHIFT),
    // PMIC_OFF_INT_EVT_ERR_INT
    IRQ_DEF_NMI_NCFG(OFF_STATE_STAT1_REG, OFF_INT_EVT_ERR_SHIFT),
    // PMIC_OFF_PROT_EVT_INT
    IRQ_DEF_NMI_NCFG(OFF_STATE_STAT1_REG, OFF_PROT_EVT_SHIFT),
    // PMIC_FIRST_PWR_ON_INT
    IRQ_DEF_NMI_NCFG(OFF_STATE_STAT1_REG, FIRST_PWR_ON_SHIFT),
    // PMIC_CLK_ERR_INT
    IRQ_DEF_NMI_NCFG(OFF_STATE_STAT1_REG, CLK_ERR_SHIFT),
    // PMIC_INTERNAL_OV_INT
    IRQ_DEF_NMI_NCFG(OFF_STATE_STAT1_REG, INTERNAL_OV_SHIFT),
    // PMIC_INIT_AN_TMO_INT
    IRQ_DEF_NMI_NCFG(OFF_STATE_STAT1_REG, INIT_AN_TMO_SHIFT),
    // PMIC_CRC_ERR_INT
    IRQ_DEF_NMI_NCFG(OFF_STATE_STAT2_REG, CRC_ERR_SHIFT),
    // PMIC_SYS_CLK_ERR_PROT_INT
    IRQ_DEF_NMI_NCFG(OFF_STATE_STAT2_REG, SYS_CLK_ERR_PROT_SHIFT),
    // PMIC_RST_MCU_TMO_INT
    IRQ_DEF_NMI_NCFG(OFF_STATE_STAT2_REG, RST_MCU_TMO_SHIFT),
    // PMIC_BGXM_ERR_INT
    IRQ_DEF_NMI_NCFG(OFF_STATE_STAT2_REG, BGXM_ERR_SHIFT),
    // PMIC_VBAT_OVP_ERR_INT
    IRQ_DEF_NMI_NCFG(OFF_STATE_STAT2_REG, VBAT_OVP_ERR_SHIFT),
    // PMIC_BB_OVP_ERR_INT
    IRQ_DEF_NMI_NCFG(OFF_STATE_STAT2_REG, BB_OVP_ERR_SHIFT),
    // PMIC_BB_BST_TMO_INT
    IRQ_DEF_NMI_NCFG(OFF_STATE_STAT2_REG, BB_BST_TMO_SHIFT),
    // PMIC_BB_PK_ILIM_ERR_INT
    IRQ_DEF_NMI_NCFG(OFF_STATE_STAT2_REG, BB_PK_ILIM_ERR_SHIFT),
    // PMIC_WD_TMO_INT
    IRQ_DEF_NMI_NCFG(WD_ERR_STAT_REG, WD_TMO_SHIFT),
    // PMIC_WD_TRIG_EARLY_INT
    IRQ_DEF_NMI_NCFG(WD_ERR_STAT_REG, WD_TRIG_EARLY_SHIFT),
    // PMIC_WD_ANSW_EARLY_INT
    IRQ_DEF_NMI_NCFG(WD_ERR_STAT_REG, WD_ANSW_EARLY_SHIFT),
    // PMIC_WD_SEQ_ERR_INT
    IRQ_DEF_NMI_NCFG(WD_ERR_STAT_REG, WD_SEQ_ERR_SHIFT),
    // PMIC_WD_ANSW_ERR_INT
    IRQ_DEF_NMI_NCFG(WD_ERR_STAT_REG, WD_ANSW_ERR_SHIFT),
    // PMIC_WD_LONGWIN_TMO_INT
    IRQ_DEF_NMI_NCFG(WD_ERR_STAT_REG, WD_LONGWIN_TMO_SHIFT),
    // PMIC_WD_TH1_ERR_INT
    IRQ_DEF_STD(WD_ERR_STAT_REG,
                WD_INT_CFG_REG,
                WD_TH1_ERR_SHIFT,
                WD_TH1_INT_MASK_SHIFT,
                WD_INT_CFG_REG,
                WD_TH1_INT_CFG_SHIFT,
                WD_TH1_INT_CFG_MASK),
    // PMIC_WD_TH2_ERR_INT
    IRQ_DEF_STD(WD_ERR_STAT_REG,
                WD_INT_CFG_REG,
                WD_TH2_ERR_SHIFT,
                WD_TH2_INT_MASK_SHIFT,
                WD_INT_CFG_REG,
                WD_TH2_INT_CFG_SHIFT,
                WD_TH2_INT_CFG_MASK),
    // PMIC_ESM_ERR_INT
    IRQ_DEF_NMI_NCFG(ESM_ERR_STAT_REG, ESM_ERR_SHIFT),
    // PMIC_ESM_DLY1_ERR_INT
    IRQ_DEF_STD(ESM_ERR_STAT_REG,
                ESM_INT_CFG_REG,
                ESM_DLY1_ERR_SHIFT,
                ESM_DLY1_INT_MASK_SHIFT,
                ESM_INT_CFG_REG,
                ESM_DLY1_INT_CFG_SHIFT,
                ESM_DLY1_INT_CFG_MASK),
    // PMIC_ESM_DLY2_ERR_INT
    IRQ_DEF_STD(ESM_ERR_STAT_REG,
                ESM_INT_CFG_REG,
                ESM_DLY2_ERR_SHIFT,
                ESM_DLY2_INT_MASK_SHIFT,
                ESM_INT_CFG_REG,
                ESM_DLY2_INT_CFG_SHIFT,
                ESM_DLY2_INT_CFG_MASK),
    // PMIC_BB_UV_ERR_INT
    IRQ_DEF_STD(DCDC_STAT_REG,
                UV_DCDC_CFG_REG,
                BB_UV_ERR_SHIFT,
                BB_UV_INT_MASK_SHIFT,
                UV_DCDC_CFG_REG,
                BB_UV_INT_CFG_SHIFT,
                BB_UV_INT_CFG_MASK),
    // PMIC_BB_OV_ERR_INT
    IRQ_DEF_STD(DCDC_STAT_REG,
                OV_DCDC_CFG_REG,
                BB_OV_ERR_SHIFT,
                BB_OV_INT_MASK_SHIFT,
                OV_DCDC_CFG_REG,
                BB_OV_INT_CFG_SHIFT,
                BB_OV_INT_CFG_MASK),
    // PMIC_LDO1_UV_ERR_INT
    IRQ_DEF_STD(VMON_LDO_STAT_REG,
                UV_INT_MASK_REG,
                LDO1_UV_ERR_SHIFT,
                LDO1_UV_INT_MASK_SHIFT,
                UV_INT_CFG1_REG,
                LDO1_UV_INT_CFG_SHIFT,
                LDO1_UV_INT_CFG_MASK),
    // PMIC_LDO1_OV_ERR_INT
    IRQ_DEF_STD(VMON_LDO_STAT_REG,
                OV_INT_MASK_REG,
                LDO1_OV_ERR_SHIFT,
                LDO1_OV_INT_MASK_SHIFT,
                OV_INT_CFG1_REG,
                LDO1_OV_INT_CFG_SHIFT,
                LDO1_OV_INT_CFG_MASK),
    // PMIC_LDO2_UV_ERR_INT
    IRQ_DEF_STD(VMON_LDO_STAT_REG,
                UV_INT_MASK_REG,
                LDO2_UV_ERR_SHIFT,
                LDO2_UV_INT_MASK_SHIFT,
                UV_INT_CFG1_REG,
                LDO2_UV_INT_CFG_SHIFT,
                LDO2_UV_INT_CFG_MASK),
    // PMIC_LDO2_OV_ERR_INT
    IRQ_DEF_STD(VMON_LDO_STAT_REG,
                OV_INT_MASK_REG,
                LDO2_OV_ERR_SHIFT,
                LDO2_OV_INT_MASK_SHIFT,
                OV_INT_CFG1_REG,
                LDO2_OV_INT_CFG_SHIFT,
                LDO2_OV_INT_CFG_MASK),
    // PMIC_LDO3_UV_ERR_INT
    IRQ_DEF_STD(VMON_LDO_STAT_REG,
                UV_INT_MASK_REG,
                LDO3_UV_ERR_SHIFT,
                LDO3_UV_INT_MASK_SHIFT,
                UV_INT_CFG1_REG,
                LDO3_UV_INT_CFG_SHIFT,
                LDO3_UV_INT_CFG_MASK),
    // PMIC_LDO3_OV_ERR_INT
    IRQ_DEF_STD(VMON_LDO_STAT_REG,
                OV_INT_MASK_REG,
                LDO3_OV_ERR_SHIFT,
                LDO3_OV_INT_MASK_SHIFT,
                OV_INT_CFG1_REG,
                LDO3_OV_INT_CFG_SHIFT,
                LDO3_OV_INT_CFG_MASK),
    // PMIC_LDO4_UV_ERR_INT
    IRQ_DEF_STD(VMON_LDO_STAT_REG,
                UV_INT_MASK_REG,
                LDO4_UV_ERR_SHIFT,
                LDO4_UV_INT_MASK_SHIFT,
                UV_INT_CFG1_REG,
                LDO4_UV_INT_CFG_SHIFT,
                LDO4_UV_INT_CFG_MASK),
    // PMIC_LDO4_OV_ERR_INT
    IRQ_DEF_STD(VMON_LDO_STAT_REG,
                OV_INT_MASK_REG,
                LDO4_OV_ERR_SHIFT,
                LDO4_OV_INT_MASK_SHIFT,
                OV_INT_CFG1_REG,
                LDO4_OV_INT_CFG_SHIFT,
                LDO4_OV_INT_CFG_MASK),
    // PMIC_PLDO1_UV_ERR_INT
    IRQ_DEF_STD(VMON_PLDO_STAT_REG,
                UV_INT_MASK_REG,
                PLDO1_UV_ERR_SHIFT,
                PLDO1_UV_INT_MASK_SHIFT,
                UV_INT_CFG2_REG,
                PLDO1_UV_INT_CFG_SHIFT,
                PLDO1_UV_INT_CFG_MASK),
    // PMIC_PLDO1_OV_ERR_INT
    IRQ_DEF_STD(VMON_PLDO_STAT_REG,
                OV_INT_MASK_REG,
                PLDO1_OV_ERR_SHIFT,
                PLDO1_OV_INT_MASK_SHIFT,
                OV_INT_CFG2_REG,
                PLDO1_OV_INT_CFG_SHIFT,
                PLDO1_OV_INT_CFG_MASK),
    // PMIC_PLDO2_UV_ERR_INT
    IRQ_DEF_STD(VMON_PLDO_STAT_REG,
                UV_INT_MASK_REG,
                PLDO2_UV_ERR_SHIFT,
                PLDO2_UV_INT_MASK_SHIFT,
                UV_INT_CFG2_REG,
                PLDO2_UV_INT_CFG_SHIFT,
                PLDO2_UV_INT_CFG_MASK),
    // PMIC_PLDO2_OV_ERR_INT
    IRQ_DEF_STD(VMON_PLDO_STAT_REG,
                OV_INT_MASK_REG,
                PLDO2_OV_ERR_SHIFT,
                PLDO2_OV_INT_MASK_SHIFT,
                OV_INT_CFG2_REG,
                PLDO2_OV_INT_CFG_SHIFT,
                PLDO2_OV_INT_CFG_MASK),
    // PMIC_EXT_VMON1_UV_ERR_INT
    IRQ_DEF_STD(EXT_VMON_STAT_REG,
                UV_INT_MASK_REG,
                EXT_VMON1_UV_ERR_SHIFT,
                EXT_VMON1_UV_INT_MASK_SHIFT,
                UV_INT_CFG2_REG,
                EXT_VMON1_UV_INT_CFG_SHIFT,
                EXT_VMON1_UV_INT_CFG_MASK),
    // PMIC_EXT_VMON1_OV_ERR_INT
    IRQ_DEF_STD(EXT_VMON_STAT_REG,
                OV_INT_MASK_REG,
                EXT_VMON1_OV_ERR_SHIFT,
                EXT_VMON1_OV_INT_MASK_SHIFT,
                OV_INT_CFG2_REG,
                EXT_VMON1_OV_INT_CFG_SHIFT,
                EXT_VMON1_OV_INT_CFG_MASK),
    // PMIC_EXT_VMON2_UV_ERR_INT
    IRQ_DEF_STD(EXT_VMON_STAT_REG,
                UV_INT_MASK_REG,
                EXT_VMON2_UV_ERR_SHIFT,
                EXT_VMON2_UV_INT_MASK_SHIFT,
                UV_INT_CFG2_REG,
                EXT_VMON2_UV_INT_CFG_SHIFT,
                EXT_VMON2_UV_INT_CFG_MASK),
    // PMIC_EXT_VMON2_OV_ERR_INT
    IRQ_DEF_STD(EXT_VMON_STAT_REG,
                OV_INT_MASK_REG,
                EXT_VMON2_OV_ERR_SHIFT,
                EXT_VMON2_OV_INT_MASK_SHIFT,
                OV_INT_CFG2_REG,
                EXT_VMON2_OV_INT_CFG_SHIFT,
                EXT_VMON2_OV_INT_CFG_MASK),
    // PMIC_LDO1_ILIM_ERR_INT
    IRQ_DEF_NMI_CFG(ILIM_STAT_REG,
                    LDO1_ILIM_ERR_SHIFT,
                    ILIM_CFG_REG,
                    LDO1_ILIM_CFG_SHIFT,
                    LDO1_ILIM_CFG_MASK),
    // PMIC_LDO2_ILIM_ERR_INT
    IRQ_DEF_NMI_CFG(ILIM_STAT_REG,
                    LDO2_ILIM_ERR_SHIFT,
                    ILIM_CFG_REG,
                    LDO2_ILIM_CFG_SHIFT,
                    LDO2_ILIM_CFG_MASK),
    // PMIC_LDO3_ILIM_ERR_INT
    IRQ_DEF_NMI_CFG(ILIM_STAT_REG,
                    LDO3_ILIM_ERR_SHIFT,
                    ILIM_CFG_REG,
                    LDO3_ILIM_CFG_SHIFT,
                    LDO3_ILIM_CFG_MASK),
    // PMIC_LDO4_ILIM_ERR_INT
    IRQ_DEF_NMI_CFG(ILIM_STAT_REG,
                    LDO4_ILIM_ERR_SHIFT,
                    ILIM_CFG_REG,
                    LDO4_ILIM_CFG_SHIFT,
                    LDO4_ILIM_CFG_MASK),
    // PMIC_PLDO1_ILIM_ERR_INT
    IRQ_DEF_NMI_CFG(ILIM_STAT_REG,
                    PLDO1_ILIM_ERR_SHIFT,
                    ILIM_CFG_REG,
                    PLDO1_ILIM_CFG_SHIFT,
                    PLDO1_ILIM_CFG_MASK),
    // PMIC_PLDO2_ILIM_ERR_INT
    IRQ_DEF_NMI_CFG(ILIM_STAT_REG,
                    PLDO2_ILIM_ERR_SHIFT,
                    ILIM_CFG_REG,
                    PLDO2_ILIM_CFG_SHIFT,
                    PLDO2_ILIM_CFG_MASK),
    // PMIC_BB_AVG_ILIM_ERR_INT
    IRQ_DEF_NMI_NCFG(ILIM_STAT_REG, BB_AVG_ILIM_ERR_SHIFT),
    // PMIC_ABIST_ERR_INT
    IRQ_DEF_NMI_NCFG(BIST_STAT_REG, ABIST_ERR_SHIFT),
    // PMIC_LBIST_ERR_INT
    IRQ_DEF_NMI_NCFG(BIST_STAT_REG, LBIST_ERR_SHIFT),
    // PMIC_TMR_REG_ERR_INT
    IRQ_DEF_NMI_NCFG(BIST_STAT_REG, TMR_REG_ERR_SHIFT),
    // PMIC_M_PMIC_HB_ERR_INT
    IRQ_DEF_NMI_NCFG(BIST_STAT_REG, M_PMIC_HB_ERR_SHIFT),
    // PMIC_SAFE_ST_TMO_RST_ERR_INT
    IRQ_DEF_NMI_NCFG(DEV_ERR_STAT_REG, SAFE_ST_TMO_RST_ERR_SHIFT),
    // PMIC_SPI_CRC_ERR_INT
    IRQ_DEF_NMI_NCFG(SPI_ERR_STAT_REG, SPI_CRC_ERR_SHIFT),
    // PMIC_FRM_ERR_INT
    IRQ_DEF_NMI_NCFG(SPI_ERR_STAT_REG, FRM_ERR_SHIFT),
    // PMIC_ADDR_ERR_INT
    IRQ_DEF_NMI_NCFG(SPI_ERR_STAT_REG, ADDR_ERR_SHIFT),
    // PMIC_SCLK_ERR_INT
    IRQ_DEF_NMI_NCFG(SPI_ERR_STAT_REG, SCLK_ERR_SHIFT),
    // PMIC_COMP1_ERR_INT
    IRQ_DEF_NMI_NCFG(CM_STAT1_REG, COMP1_ERR_SHIFT),
    // PMIC_COMP1P_UV_ERR_INT
    IRQ_DEF_STD(CM_STAT1_REG,
                CM_VMON_INT_MASK_CFG_REG,
                COMP1P_UV_ERR_SHIFT,
                COMP1P_UV_INT_MASK_SHIFT,
                CM_VMON_INT_CFG_REG,
                COMP1P_UV_INT_CFG_SHIFT,
                COMP1P_UV_INT_CFG_MASK),
    // PMIC_COMP1P_OV_ERR_INT
    IRQ_DEF_STD(CM_STAT1_REG,
                CM_VMON_INT_MASK_CFG_REG,
                COMP1P_OV_ERR_SHIFT,
                COMP1P_OV_INT_MASK_SHIFT,
                CM_VMON_INT_CFG_REG,
                COMP1P_OV_INT_CFG_SHIFT,
                COMP1P_OV_INT_CFG_MASK),
    // PMIC_COMP1N_UV_ERR_INT
    IRQ_DEF_STD(CM_STAT1_REG,
                CM_VMON_INT_MASK_CFG_REG,
                COMP1N_UV_ERR_SHIFT,
                COMP1N_UV_INT_MASK_SHIFT,
                CM_VMON_INT_CFG_REG,
                COMP1N_UV_INT_CFG_SHIFT,
                COMP1N_UV_INT_CFG_MASK),
    // PMIC_COMP1N_OV_ERR_INT
    IRQ_DEF_STD(CM_STAT1_REG,
                CM_VMON_INT_MASK_CFG_REG,
                COMP1N_OV_ERR_SHIFT,
                COMP1N_OV_INT_MASK_SHIFT,
                CM_VMON_INT_CFG_REG,
                COMP1N_OV_INT_CFG_SHIFT,
                COMP1N_OV_INT_CFG_MASK),
    // PMIC_COMP2_ERR_INT
    IRQ_DEF_NMI_NCFG(CM_STAT2_REG, COMP2_ERR_SHIFT),
    // PMIC_COMP2P_UV_ERR_INT
    IRQ_DEF_STD(CM_STAT2_REG,
                CM_VMON_INT_MASK_CFG_REG,
                COMP2P_UV_ERR_SHIFT,
                COMP2P_UV_INT_MASK_SHIFT,
                CM_VMON_INT_CFG_REG,
                COMP2P_UV_INT_CFG_SHIFT,
                COMP2P_UV_INT_CFG_MASK),
    // PMIC_COMP2P_OV_ERR_INT
    IRQ_DEF_STD(CM_STAT2_REG,
                CM_VMON_INT_MASK_CFG_REG,
                COMP2P_OV_ERR_SHIFT,
                COMP2P_OV_INT_MASK_SHIFT,
                CM_VMON_INT_CFG_REG,
                COMP2P_OV_INT_CFG_SHIFT,
                COMP2P_OV_INT_CFG_MASK),
    // PMIC_COMP2N_UV_ERR_INT
    IRQ_DEF_STD(CM_STAT2_REG,
                CM_VMON_INT_MASK_CFG_REG,
                COMP2N_UV_ERR_SHIFT,
                COMP2N_UV_INT_MASK_SHIFT,
                CM_VMON_INT_CFG_REG,
                COMP2N_UV_INT_CFG_SHIFT,
                COMP2N_UV_INT_CFG_MASK),
    // PMIC_COMP2N_OV_ERR_INT
    IRQ_DEF_STD(CM_STAT2_REG,
                CM_VMON_INT_MASK_CFG_REG,
                COMP2N_OV_ERR_SHIFT,
                COMP2N_OV_INT_MASK_SHIFT,
                CM_VMON_INT_CFG_REG,
                COMP2N_OV_INT_CFG_SHIFT,
                COMP2N_OV_INT_CFG_MASK),
};

static const uint16_t IrqStatusRegisters[NUM_STAT_REGISTERS] = {
    REG_STAT_REG,
    RDBK_ERR_STAT_REG,
    OFF_STATE_STAT1_REG,
    OFF_STATE_STAT2_REG,
    WD_ERR_STAT_REG,
    ESM_ERR_STAT_REG,
    DCDC_STAT_REG,
    VMON_LDO_STAT_REG,
    VMON_PLDO_STAT_REG,
    EXT_VMON_STAT_REG,
    ILIM_STAT_REG,
    BIST_STAT_REG,
    DEV_ERR_STAT_REG,
    SPI_ERR_STAT_REG,
    CM_STAT1_REG,
    CM_STAT2_REG,
};

static const uint16_t IrqMaskRegisters[] = {
    RDBK_INT_MASK_CFG_REG,
    WD_INT_CFG_REG,
    UV_DCDC_CFG_REG,
    OV_DCDC_CFG_REG,
    UV_INT_MASK_REG,
    OV_INT_MASK_REG,
    CM_VMON_INT_MASK_CFG_REG,
};

static const uint16_t IrqConfRegisters[] = {
    RDBK_INT_CFG1_REG,
    RDBK_INT_CFG2_REG,
    SAFETY_CFG_REG,
    WD_INT_CFG_REG,
    ESM_INT_CFG_REG,
    UV_DCDC_CFG_REG,
    OV_DCDC_CFG_REG,
    UV_INT_CFG1_REG,
    OV_INT_CFG1_REG,
    UV_INT_CFG2_REG,
    OV_INT_CFG2_REG,
    ILIM_CFG_REG,
    CM_VMON_INT_CFG_REG,
};

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */
static inline void IRQ_setIntrStat(Pmic_IrqStat_t *irqStat, uint32_t irqNum)
{
    const uint32_t mask = ((uint32_t)1U << (irqNum % PMIC_NUM_BITS_IN_INTR_ELEM));

    if (irqNum < PMIC_IRQ_NUM)
    {
        // IRQs 0 to 31 go to index 0, IRQs 32 to 63 go to index 1.
        // At an index, the IRQ is stored at its corresponding bit
        // (e.g., IRQ 49's status will be stored at bit 17 at index 1)
        irqStat->intStat[irqNum / PMIC_NUM_BITS_IN_INTR_ELEM] |= mask;
    }
}

static inline void IRQ_clrIntrStat(Pmic_IrqStat_t *irqStat, uint32_t irqNum)
{
    const uint32_t mask = ((uint32_t)1U << (irqNum % PMIC_NUM_BITS_IN_INTR_ELEM));

    if (irqNum < PMIC_IRQ_NUM)
    {
        irqStat->intStat[irqNum / PMIC_NUM_BITS_IN_INTR_ELEM] &= ~mask;
    }
}

static uint8_t IRQ_getNextFlag(Pmic_IrqStat_t *irqStat) {
    uint8_t index = 0U, bitPos = 0U;
    bool foundFlag = false;

    // For each element in struct member intStat of irqStat...
    for (index = 0U; index < PMIC_NUM_ELEM_IN_INTR_STAT; index++) {
        // If current element has no IRQ statuses set, move onto next element
        if (irqStat->intStat[index] == 0U) {
            continue;
        }

        // For each bit in the element...
        for (bitPos = 0U; bitPos < PMIC_NUM_BITS_IN_INTR_ELEM; bitPos++) {
            // If the bit is set...
            if ((irqStat->intStat[index] & (1U << bitPos)) != 0U) {
                // Clear bit in intrStat element and exit loop
                irqStat->intStat[index] &= ~(1U << bitPos);
                foundFlag = true;
                break;
            }
        }

        if (foundFlag) {
            break;
        }
    }

    // Return the corresponding IRQ number
    return (bitPos + (PMIC_NUM_BITS_IN_INTR_ELEM * index));
}

static int32_t IRQ_setMask(Pmic_CoreHandle_t *handle, uint8_t irqNum, bool shouldMask) {
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    uint16_t maskReg = 0U;
    uint8_t maskShift = 0U;

    // Check for invalid IRQ number
    if (irqNum > PMIC_IRQ_MAX) {
        status = PMIC_ST_ERR_INV_PARAM;
    } else {
        maskReg = IRQ[irqNum].maskReg;
        maskShift = IRQ[irqNum].maskShift;
    }

    // Check whether IRQ is maskable
    if ((status == PMIC_ST_SUCCESS) && (maskReg == PMIC_IRQ_INVALID_REG)) {
        status = PMIC_ST_ERR_NOT_SUPPORTED;
    }

    // Read IRQ mask register
    Pmic_criticalSectionStart(handle);
    if (status == PMIC_ST_SUCCESS) {
        status = Pmic_ioRxByte(handle, maskReg, &regData);
    }

    if (status == PMIC_ST_SUCCESS) {
        // Modify IRQ mask bit field and write new register value
        Pmic_setBitField_b(&regData, maskShift, shouldMask);
        status = Pmic_ioTxByte(handle, maskReg, regData);
    }
    Pmic_criticalSectionStop(handle);

    return status;
}

static int32_t IRQ_setConfig(Pmic_CoreHandle_t *handle, uint8_t irqNum, uint8_t config) {
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    uint16_t configReg = 0U;
    uint8_t configShift = 0U;
    uint8_t configMask = 0U;

    // Check for invalid IRQ number
    if (irqNum > PMIC_IRQ_MAX) {
        status = PMIC_ST_ERR_INV_PARAM;
    } else {
        configReg = IRQ[irqNum].confReg;
        configShift = IRQ[irqNum].confShift;
        configMask = IRQ[irqNum].confMask;
    }

    // Check whether IRQ is configurable
    if ((status == PMIC_ST_SUCCESS) && (configReg == PMIC_IRQ_INVALID_REG)) {
        status = PMIC_ST_ERR_NOT_SUPPORTED;
    }

    // Read IRQ configuration register
    Pmic_criticalSectionStart(handle);
    if (status == PMIC_ST_SUCCESS) {
        status = Pmic_ioRxByte(handle, configReg, &regData);
    }

    if (status == PMIC_ST_SUCCESS) {
        // Modify IRQ config bit field and write new value
        Pmic_setBitField(&regData, configShift, configMask, config);
        status = Pmic_ioTxByte(handle, configReg, regData);
    }
    Pmic_criticalSectionStop(handle);

    return status;
}

// Check irqCfg->config param
static inline int32_t IRQ_checkConfigParam(const Pmic_IrqCfg_t *irqCfg)
{
    int32_t status = PMIC_ST_SUCCESS;

    // Check whether irqCfg->config is valid for irqNum=PMIC_CFG_REG_CRC_ERR_INT
    if ((irqCfg->irqNum == PMIC_CFG_REG_CRC_ERR_INT) && (irqCfg->config > PMIC_IRQ_CONFIG1_MAX)) {
        status = PMIC_ST_ERR_INV_PARAM;
    }
    // Check whether irqCfg->config is valid for irqNum>=PMIC_COMP1P_UV_ERR_INT && irqNum<=PMIC_COMP2N_OV_ERR_INT
    else if ((irqCfg->irqNum >= PMIC_COMP1P_UV_ERR_INT) && (irqCfg->irqNum <= PMIC_COMP2N_OV_ERR_INT) &&
            (irqCfg->config > PMIC_IRQ_CONFIG2_MAX)) {
        status = PMIC_ST_ERR_INV_PARAM;
    }
    // Check whether irqCfg->config is valid for all other IRQs
    else if (irqCfg->config > PMIC_IRQ_CONFIG0_MAX) {
        status = PMIC_ST_ERR_INV_PARAM;
    }
    else {
    }

    return status;
}

int32_t Pmic_irqSetCfg(Pmic_CoreHandle_t *handle, const Pmic_IrqCfg_t *irqCfg) {
    int32_t status = Pmic_checkPmicCoreHandle(handle);

    // Parameter validation
    if ((status == PMIC_ST_SUCCESS) && (irqCfg == NULL)) {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) && (irqCfg->irqNum > PMIC_IRQ_MAX)) {
       status = PMIC_ST_ERR_INV_PARAM;
    }

    if (status == PMIC_ST_SUCCESS) {
        status = IRQ_checkConfigParam(irqCfg);
    }

    // Set IRQ mask if requested
    if (Pmic_validParamStatusCheck(irqCfg->validParams, PMIC_IRQ_CFG_MASK_VALID, status)) {
        status = IRQ_setMask(handle, irqCfg->irqNum, irqCfg->mask);
    }

    // Set IRQ config if requested
    if (Pmic_validParamStatusCheck(irqCfg->validParams, PMIC_IRQ_CFG_CONFIG_VALID_SHIFT, status)) {
        status = IRQ_setConfig(handle, irqCfg->irqNum, irqCfg->config);
    }

    return status;
}

static inline bool IRQ_anyMasksForReg(uint8_t numCfgs, const Pmic_IrqCfg_t *cfgs, uint16_t regAddr) {
    bool anyRegs = false;

    for (uint8_t i = 0U; (i < numCfgs) && !anyRegs; i++) {
        const uint8_t irqNum = cfgs[i].irqNum;
        anyRegs = (IRQ[irqNum].maskReg == regAddr);
    }

    return anyRegs;
}

static inline int32_t IRQ_anyConfigsForReg(uint8_t numCfgs, const Pmic_IrqCfg_t *cfgs, uint16_t regAddr, bool *anyConfig) {
    int32_t status = PMIC_ST_SUCCESS;

    *anyConfig = (bool)false;
    for (uint8_t i = 0U; (i < numCfgs) && !(*anyConfig); i++) {
        status = IRQ_checkConfigParam(&cfgs[i]);

        if (status == PMIC_ST_SUCCESS) {
            const uint8_t irqNum = cfgs[i].irqNum;
            *anyConfig = (IRQ[irqNum].confReg == regAddr);
        } else {
            break;
        }
    }

    return status;
}

static int32_t IRQ_handleRecordsForRegMask(Pmic_CoreHandle_t *handle,
                                           uint8_t numCfgs,
                                           const Pmic_IrqCfg_t *cfgs,
                                           uint16_t regAddr,
                                           uint8_t *processedCfgs) {
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    const bool anyMasks = IRQ_anyMasksForReg(numCfgs, cfgs, regAddr);

    if (anyMasks) {
        // Get the current value of this IRQ mask register
        Pmic_criticalSectionStart(handle);
        status = Pmic_ioRxByte(handle, regAddr, &regData);
    }

    if ((status == PMIC_ST_SUCCESS) && anyMasks) {
        for (uint8_t i = 0U; i < numCfgs; i++) {
            const uint8_t irqNum = cfgs[i].irqNum;
            const uint8_t userMask = (uint8_t)(1U << IRQ[irqNum].maskShift);

            // If the current mask setting isn't targeted at the register we are
            // currently building, skip it
            if (regAddr != IRQ[irqNum].maskReg) {
                continue;
            }

            // If the user didn't set the mask validParam for this config, skip it
            if (Pmic_validParamCheck(cfgs->validParams, PMIC_IRQ_CFG_MASK_VALID) == (bool)false) {
                continue;
            }

            if (cfgs[i].mask) {
                regData |= userMask;
            } else {
                regData &= ~userMask;
            }

            // Increment the counter indicating we processed this record
            *processedCfgs += 1U;
        }
    }

    // If status is still good and we did find records that apply to this
    // register, write the new value of this register back to the device
    if ((status == PMIC_ST_SUCCESS) && *processedCfgs > 0) {
        status = Pmic_ioTxByte(handle, regAddr, regData);
    }

    // If a critical section was obtained earlier, it can now be released
    if (anyMasks) {
        Pmic_criticalSectionStop(handle);
    }

    return status;
}

static int32_t IRQ_handleRecordsForRegConfig(Pmic_CoreHandle_t *handle,
                                             uint8_t numCfgs,
                                             const Pmic_IrqCfg_t *cfgs,
                                             uint16_t regAddr,
                                             uint8_t *processedCfgs) {
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    bool anyConfigs = (bool)false;

    status = IRQ_anyConfigsForReg(numCfgs, cfgs, regAddr, &anyConfigs);

    // Get the current value of this IRQ mask register
    Pmic_criticalSectionStart(handle);
    if ((status == PMIC_ST_SUCCESS) && anyConfigs) {
        status = Pmic_ioRxByte(handle, regAddr, &regData);
    }

    if ((status == PMIC_ST_SUCCESS) && anyConfigs) {
        for (uint8_t i = 0U; i < numCfgs; i++) {
            const uint8_t irqNum = cfgs[i].irqNum;
            const uint8_t userMask = (uint8_t)((cfgs[i].config << IRQ[irqNum].confShift) & IRQ[irqNum].confMask);

            // If the current mask setting isn't targeted at the register we are
            // currently building, skip it
            if (regAddr != IRQ[irqNum].confReg) {
                continue;
            }

            // If the user didn't set the config validParam for this config, skip it
            if (Pmic_validParamCheck(cfgs->validParams, PMIC_IRQ_CFG_CONFIG_VALID) == (bool)false) {
                continue;
            }

            regData &= ~IRQ[irqNum].confMask;
            regData |= userMask;

            // Increment the counter indicating we processed this record
            *processedCfgs += 1U;
        }
    }

    // If status is still good and we did find records that apply to this
    // register, write the new value of this register back to the device
    if ((status == PMIC_ST_SUCCESS) && (*processedCfgs > 0U)) {
        status = Pmic_ioTxByte(handle, regAddr, regData);
    }
    Pmic_criticalSectionStop(handle);

    return status;
}

int32_t Pmic_irqSetCfgs(Pmic_CoreHandle_t *handle, uint8_t numIrqs, const Pmic_IrqCfg_t *irqCfgs) {
    int32_t status = Pmic_checkPmicCoreHandle(handle);
    uint8_t lastProcessed = 0U, totalProcessed = 0U;

    // Parameter validation
    if ((status == PMIC_ST_SUCCESS) && (irqCfgs == NULL)) {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) && (numIrqs > PMIC_IRQ_NUM)) {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    // Handle mask controls
    for (uint8_t regIndex = 0U; regIndex < NUM_MASK_REGISTERS; regIndex++) {
        // If status is no longer good or all user requested masks have been
        // processed, we can stop iterating
        if ((status != PMIC_ST_SUCCESS) || (totalProcessed >= numIrqs)) {
            break;
        }

        if (status == PMIC_ST_SUCCESS) {
            status = IRQ_handleRecordsForRegMask(handle, numIrqs, irqCfgs, IrqMaskRegisters[regIndex], &lastProcessed);
            totalProcessed += lastProcessed;
        }
    }

    // Reset tracking variables
    lastProcessed = 0U;
    totalProcessed = 0U;

    // Handle config controls
    for (uint8_t regIndex = 0U; regIndex < NUM_CONF_REGISTERS; regIndex++) {
        // If status is no longer good or all user requested masks have been
        // processed, we can stop iterating
        if ((status != PMIC_ST_SUCCESS) || (totalProcessed >= numIrqs)) {
            break;
        }

        if (status == PMIC_ST_SUCCESS) {
            status = IRQ_handleRecordsForRegConfig(handle, numIrqs, irqCfgs, IrqConfRegisters[regIndex], &lastProcessed);
            totalProcessed += lastProcessed;
        }
    }

    return status;
}

static int32_t IRQ_getMaskOrConfig(Pmic_CoreHandle_t *handle, Pmic_IrqCfg_t *irqCfg, bool getMask)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    uint16_t reg = 0U;
    uint8_t shift = 0U;
    uint8_t mask = 0U;

    // NOTE: This function does not validate param; that is handled in the
    // higher level function that calls this one.

    // Check for invalid IRQ number
    if (irqCfg->irqNum > PMIC_IRQ_MAX) {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    if (status == PMIC_ST_SUCCESS) {
        if (getMask) {
            reg = IRQ[irqCfg->irqNum].maskReg;
            shift = IRQ[irqCfg->irqNum].maskShift;
        } else {
            reg = IRQ[irqCfg->irqNum].confReg;
            shift = IRQ[irqCfg->irqNum].confShift;
            mask = IRQ[irqCfg->irqNum].confMask;
        }
    }

    // Check whether IRQ is configurable
    if ((status == PMIC_ST_SUCCESS) && (reg == PMIC_IRQ_INVALID_REG)) {
        status = PMIC_ST_ERR_NOT_SUPPORTED;
    }

    // Read mask/config register
    if (status == PMIC_ST_SUCCESS) {
        Pmic_criticalSectionStart(handle);
        status = Pmic_ioRxByte(handle, reg, &regData);
        Pmic_criticalSectionStop(handle);
    }

    // Extract IRQ mask/config bit field
    if (status == PMIC_ST_SUCCESS) {
        if (getMask) {
            irqCfg->mask = Pmic_getBitField_b(regData, shift);
        } else {
            irqCfg->config = Pmic_getBitField(regData, shift, mask);
        }
    }

    return status;
}

static inline int32_t IRQ_getCfg(Pmic_CoreHandle_t *handle, Pmic_IrqCfg_t *irqCfg) {
    int32_t status = PMIC_ST_SUCCESS;
    const bool getMask = (bool)true;
    const bool getConfig = (bool)false;

    if (Pmic_validParamCheck(irqCfg->validParams, PMIC_IRQ_CFG_MASK_VALID)) {
        status = IRQ_getMaskOrConfig(handle, irqCfg, getMask);
    }

    if (Pmic_validParamStatusCheck(irqCfg->validParams, PMIC_IRQ_CFG_CONFIG_VALID, status)) {
        status = IRQ_getMaskOrConfig(handle, irqCfg, getConfig);
    }

    return status;
}

int32_t Pmic_irqGetCfg(Pmic_CoreHandle_t *handle, Pmic_IrqCfg_t *irqCfg) {
    int32_t status = Pmic_checkPmicCoreHandle(handle);

    if ((status == PMIC_ST_SUCCESS) && (irqCfg == NULL)) {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if (status == PMIC_ST_SUCCESS) {
        status = IRQ_getCfg(handle, irqCfg);
    }

    return status;
}

int32_t Pmic_irqGetCfgs(Pmic_CoreHandle_t *handle, uint8_t numIrqs, Pmic_IrqCfg_t *irqCfgs) {
    int32_t status = Pmic_checkPmicCoreHandle(handle);

    if ((status == PMIC_ST_SUCCESS) && (irqCfgs == NULL)) {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) && (numIrqs > PMIC_IRQ_NUM)) {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    for (uint8_t i = 0U; ((status == PMIC_ST_SUCCESS) && (i < numIrqs)); i++) {
        status = IRQ_getCfg(handle, &irqCfgs[i]);
    }

    return status;
}

static int32_t IRQ_getIrqStatForReg(Pmic_CoreHandle_t *handle, Pmic_IrqStat_t *irqStat, uint16_t regAddr) {
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    // Read from the status register
    Pmic_criticalSectionStart(handle);
    status = Pmic_ioRxByte(handle, regAddr, &regData);
    Pmic_criticalSectionStop(handle);

    if (status == PMIC_ST_SUCCESS) {
        for (uint8_t i = 0U; i < PMIC_IRQ_NUM; i++) {
            // If the register for this IRQ isn't the one we just read, skip it
            if (IRQ[i].statReg != regAddr) {
                continue;
            }

            // If the status bit is set for this IRQ, indicate that in irqStat,
            // otherwise clear the bit in irqStat
            if (Pmic_getBitField_b(regData, IRQ[i].statShift)) {
                IRQ_setIntrStat(irqStat, i);
            } else {
                IRQ_clrIntrStat(irqStat, i);
            }
        }
    }

    return status;
}

int32_t Pmic_irqGetStat(Pmic_CoreHandle_t *handle, Pmic_IrqStat_t *irqStat) {
    int32_t status = Pmic_checkPmicCoreHandle(handle);

    if ((status == PMIC_ST_SUCCESS) && (irqStat == NULL)) {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    for (uint8_t i = 0U; i < NUM_STAT_REGISTERS; i++) {
        if (status != PMIC_ST_SUCCESS) {
            break;
        }

        status = IRQ_getIrqStatForReg(handle, irqStat, IrqStatusRegisters[i]);
    }

    return status;
}

int32_t Pmic_irqGetNextFlag(Pmic_IrqStat_t *irqStat, uint8_t *irqNum) {
    int32_t status = PMIC_ST_SUCCESS;
    bool irqStatEmpty = false;

    if ((irqStat == NULL) || (irqNum == NULL)) {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if (status == PMIC_ST_SUCCESS) {
        irqStatEmpty = (
            (irqStat->intStat[0U] != 0U) ||
            (irqStat->intStat[1U] != 0U) ||
            (irqStat->intStat[2U] != 0U) ||
            (irqStat->intStat[3U] != 0U));
    }

    if ((status == PMIC_ST_SUCCESS) && irqStatEmpty) {
        status = PMIC_ST_WARN_NO_IRQ_REMAINING;
    }

    if (status == PMIC_ST_SUCCESS) {
        *irqNum = IRQ_getNextFlag(irqStat);
    }

    return status;
}

int32_t Pmic_irqGetFlag(Pmic_CoreHandle_t *handle, uint8_t irqNum, bool *flag) {
    int32_t status = Pmic_checkPmicCoreHandle(handle);
    uint8_t regData = 0U;

    if ((status == PMIC_ST_SUCCESS) && (irqNum > PMIC_IRQ_MAX)) {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) && (flag == NULL)) {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    // Read IRQ status register
    if (status == PMIC_ST_SUCCESS) {
        Pmic_criticalSectionStart(handle);
        status = Pmic_ioRxByte(handle, IRQ[irqNum].statReg, &regData);
        Pmic_criticalSectionStop(handle);
    }

    // Extract IRQ status
    if (status == PMIC_ST_SUCCESS) {
        *flag = Pmic_getBitField_b(regData, IRQ[irqNum].statShift);
    }

    return status;
}

int32_t Pmic_irqClrFlag(Pmic_CoreHandle_t *handle, uint8_t irqNum) {
    int32_t status = Pmic_checkPmicCoreHandle(handle);

    uint8_t regData = 0U;
    uint8_t shift = 0U;
    uint16_t reg = 0U;

    if ((status == PMIC_ST_SUCCESS) && (irqNum > PMIC_IRQ_MAX)) {
        status = PMIC_ST_ERR_INV_PARAM;
    } else {
        shift = IRQ[irqNum].statShift;
        reg = IRQ[irqNum].statReg;
    }

    // handle special case of OFF_STATE_STAT{1,2}_REG which cannot be cleared
    // with W1/C, but instead has a dedicated register with a single bit which
    // clears all bits in these two registers
    if ((status == PMIC_ST_SUCCESS) &&
        ((reg == OFF_STATE_STAT1_REG) || (reg == OFF_STATE_STAT2_REG))) {
        shift = OFF_STATE_STAT_CLR_SHIFT;
        reg = OFF_STATE_CLR_REG;
    }

    if (status == PMIC_ST_SUCCESS) {
        // IRQ statuses are W1C - write 1 to clear
        Pmic_setBitField_b(&regData, shift, PMIC_ENABLE);

        // Write data to PMIC
        Pmic_criticalSectionStart(handle);
        status = Pmic_ioTxByte(handle, reg, regData);
        Pmic_criticalSectionStop(handle);
    }

    return status;
}

int32_t Pmic_irqClrAllFlags(Pmic_CoreHandle_t *handle) {
    int32_t status = Pmic_checkPmicCoreHandle(handle);
    uint8_t regData = 0U;
    uint16_t reg = 0U;

    // All IRQ statuses are W1C, writing to reserved bits has no effect, so just
    // write every bit to 1
    Pmic_criticalSectionStart(handle);
    for (uint8_t i = 0U; i < NUM_STAT_REGISTERS; i++) {
        if (status != PMIC_ST_SUCCESS) {
            break;
        }

        reg = IrqStatusRegisters[i];

        // OFF_STATE_STAT{1,2}_REG are special case, and will be handled
        // separately
        if ((reg == OFF_STATE_STAT1_REG) || (reg == OFF_STATE_STAT2_REG)) {
            continue;
        }

        if (reg != DEV_ERR_STAT_REG)
        {
            status = Pmic_ioTxByte(handle, reg, 0xFFU);
        }
        // Special case: DEV_ERR_STAT has DEV_ERR_CNT bit field in it. Setting it can
        // incur unexpected or undesired behavior, such as causing PMIC to turn off
        else
        {
            status = Pmic_ioRxByte(handle, reg, &regData);
            if (status == PMIC_ST_SUCCESS)
            {
                Pmic_setBitField_b(&regData, SAFE_ST_TMO_RST_ERR_SHIFT, PMIC_ENABLE);
                status = Pmic_ioTxByte(handle, reg, regData);
            }
        }
    }

    // Clear OFF_STATE_STAT{1,2}_REG
    if (status == PMIC_ST_SUCCESS) {
        status = Pmic_ioTxByte(handle, OFF_STATE_CLR_REG, 1U << OFF_STATE_STAT_CLR_SHIFT);
    }

    Pmic_criticalSectionStop(handle);

    return status;
}
