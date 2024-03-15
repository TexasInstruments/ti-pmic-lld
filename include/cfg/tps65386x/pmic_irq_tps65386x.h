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
 * @file   pmic_irq_tps6594x.h
 *
 * @brief  TPS65386x BlackBird PMIC IRQ Driver API/interface file.
 *
 */

#ifndef PMIC_IRQ_TPS65386X_H_
#define PMIC_IRQ_TPS65386X_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

#define PMIC_BB_CFG_REG_CRC_INT_CFG (0U)

#define PMIC_BB_NRST_RDBK_INT_CFG (1U)
#define PMIC_BB_SAFE_OUT1_RDBK_INT_CFG (2U)
#define PMIC_BB_EN_OUT_RDBK_INT_CFG (3U)

#define PMIC_BB_GPO1_RDBK_INT_CFG (4U)
#define PMIC_BB_GPO2_RDBK_INT_CFG (5U)
#define PMIC_BB_GPO3_RDBK_INT_CFG (6U)
#define PMIC_BB_GPO4_RDBK_INT_CFG (7U)

#define PMIC_BB_OFF_INT_EVT_ERR (8U)

#define PMIC_BB_LDO1_OV_INT_CFG (9U)
#define PMIC_BB_LDO2_OV_INT_CFG (10U)
#define PMIC_BB_LDO3_OV_INT_CFG (11U)
#define PMIC_BB_LDO4_OV_INT_CFG (12U)

#define PMIC_BB_PLDO1_OV_INT_CFG (13U)
#define PMIC_BB_PLDO2_OV_INT_CFG (14U)
#define PMIC_BB_EXT_VMON1_OV_INT_CFG (15U)
#define PMIC_BB_EXT_VMON2_OV_INT_CFG (16U)

#define PMIC_BB_BB_OV_INT_CFG (17U)

#define PMIC_BB_LDO1_UV_INT_CFG (18U)
#define PMIC_BB_LDO2_UV_INT_CFG (19U)
#define PMIC_BB_LDO3_UV_INT_CFG (20U)
#define PMIC_BB_LDO4_UV_INT_CFG (21U)

#define PMIC_BB_PLDO1_UV_INT_CFG (22U)
#define PMIC_BB_PLDO2_UV_INT_CFG (23U)
#define PMIC_BB_EXT_VMON1_UV_INT_CFG (24U)
#define PMIC_BB_EXT_VMON2_UV_INT_CFG (25U)

#define PMIC_BB_BB_UV_INT_CFG (26U)

#define PMIC_BB_WD_TH1_INT_CFG (27U)
#define PMIC_BB_WD_TH2_INT_CFG (28U)

#define PMIC_BB_ESM_DLY1_INT_CFG (29U)
#define PMIC_BB_ESM_DLY2_INT_CFG (30U)

#define PMIC_BB_COMP1_INT_CFG (31U)
#define PMIC_BB_COMP2_INT_CFG (32U)

#define PMIC_BB_COMP1P_UV_INT_CFG (33U)
#define PMIC_BB_COMP1P_OV_INT_CFG (34U)
#define PMIC_BB_COMP1N_UV_INT_CFG (35U)
#define PMIC_BB_COMP1N_OV_INT_CFG (36U)
#define PMIC_BB_COMP2P_UV_INT_CFG (37U)
#define PMIC_BB_COMP2P_OV_INT_CFG (38U)
#define PMIC_BB_COMP2N_UV_INT_CFG (39U)
#define PMIC_BB_COMP2N_OV_INT_CFG (40U)

#define PMIC_BB_NRST_RDBK_INT_MASK (41U)
#define PMIC_BB_SAFE_OUT1_RDBK_INT_MASK (42U)
#define PMIC_BB_EN_OUT_RDBK_INT_MASK (43U)
#define PMIC_BB_GPO1_RDBK_INT_MASK (44U)
#define PMIC_BB_GPO2_RDBK_INT_MASK (45U)
#define PMIC_BB_GPO3_RDBK_INT_MASK (46U)
#define PMIC_BB_GPO4_RDBK_INT_MASK (47U)

#define PMIC_BB_LDO1_OV_INT_MASK (48U)
#define PMIC_BB_LDO2_OV_INT_MASK (49U)
#define PMIC_BB_LDO3_OV_INT_MASK (50U)
#define PMIC_BB_LDO4_OV_INT_MASK (51U)
#define PMIC_BB_PLDO1_OV_INT_MASK (52U)
#define PMIC_BB_PLDO2_OV_INT_MASK (54U)
#define PMIC_BB_EXT_VMON1_OV_INT_MASK (54U)
#define PMIC_BB_EXT_VMON2_OV_INT_MASK (55U)

#define PMIC_BB_BB_OV_INT_MASK (56U)

#define PMIC_BB_LDO1_UV_INT_MASK (57U)
#define PMIC_BB_LDO2_UV_INT_MASK (58U)
#define PMIC_BB_LDO3_UV_INT_MASK (59U)
#define PMIC_BB_LDO4_UV_INT_MASK (60U)
#define PMIC_BB_PLDO1_UV_INT_MASK (61U)
#define PMIC_BB_PLDO2_UV_INT_MASK (62U)
#define PMIC_BB_EXT_VMON1_UV_INT_MASK (63U)
#define PMIC_BB_EXT_VMON2_UV_INT_MASK (64U)

#define PMIC_BB_BB_UV_INT_MASK (65U)

#define PMIC_BB_WD_TH1_INT_MASK (66U)
#define PMIC_BB_WD_TH2_INT_MASK (67U)

#define PMIC_BB_ESM_INT_MASK (68U)
#define PMIC_BB_ESM_DLY1_INT_MASK (69U)
#define PMIC_BB_ESM_DLY2_INT_MASK (70U)

#define PMIC_BB_COMP1_INT_MASK (71U)
#define PMIC_BB_COMP2_INT_MASK (72U)

#define PMIC_BB_COMP1P_UV_INT_MASK (73U)
#define PMIC_BB_COMP1P_OV_INT_MASK (74U)
#define PMIC_BB_COMP1N_UV_INT_MASK (75U)
#define PMIC_BB_COMP1N_OV_INT_MASK (76U)
#define PMIC_BB_COMP2P_UV_INT_MASK (77U)
#define PMIC_BB_COMP2P_OV_INT_MASK (78U)
#define PMIC_BB_COMP2N_UV_INT_MASK (79U)
#define PMIC_BB_COMP2N_OV_INT_MASK (80U)

#define PMIC_BB_IRQ_MAX_NUM (81U)

#define PMIC_BB_IRQ_GPO_0_INT_MASK_NUM (0U)
#define PMIC_BB_IRQ_GPO_1_INT_MASK_NUM (1U)
#define PMIC_BB_IRQ_GPO_2_INT_MASK_NUM (2U)
#define PMIC_BB_IRQ_GPO_3_INT_MASK_NUM (3U)

/*==========================================================================*/
/*                         Function Declarations                            */
/*==========================================================================*/

void pmic_get_BB_intrCfg(Pmic_IntrCfg_t **pIntrCfg);

void pmic_get_BB_intrGpioCfg(Pmic_GpioIntrTypeCfg_t **pIntGpioCfg);

int32_t Pmic_BB_irqGetL2Error(Pmic_CoreHandle_t *pPmicCoreHandle,
                              uint16_t l1RegAddr, Pmic_IrqStatus_t *pErrStat);

static int32_t Pmic_bb_getCMVMONErr(Pmic_CoreHandle_t *pPmicCoreHandle,
                                    Pmic_IrqStatus_t *pErrStat);

static int32_t Pmic_bb_getCOMPErr(Pmic_CoreHandle_t *pPmicCoreHandle,
                                  Pmic_IrqStatus_t *pErrStat);

static int32_t Pmic_bb_getESMErr(Pmic_CoreHandle_t *pPmicCoreHandle,
                                 Pmic_IrqStatus_t *pErrStat);

static int32_t Pmic_bb_getOV_VMONPLDOErr(Pmic_CoreHandle_t *pPmicCoreHandle,
                                         Pmic_IrqStatus_t *pErrStat);

static int32_t Pmic_bb_getRDBKErr(Pmic_CoreHandle_t *pPmicCoreHandle,
                                  uint8_t regValue, Pmic_IrqStatus_t *pErrStat);

static int32_t Pmic_bb_getUV_LDO_Err(Pmic_CoreHandle_t *pPmicCoreHandle,
                                     Pmic_IrqStatus_t *pErrStat);

static int32_t Pmic_bb_getUV_VMONPLDOErr(Pmic_CoreHandle_t *pPmicCoreHandle,
                                         Pmic_IrqStatus_t *pErrStat);

static int32_t Pmic_bb_getOV_LDO_Err(Pmic_CoreHandle_t *pPmicCoreHandle,
                                     Pmic_IrqStatus_t *pErrStat);

static int32_t
Pmic_bb_getOV_LDO_VMON_PLDOErr(Pmic_CoreHandle_t *pPmicCoreHandle,
                               uint8_t regValue, Pmic_IrqStatus_t *pErrStat);

static int32_t
Pmic_bb_getUV_LDO_VMON_PLDOErr(Pmic_CoreHandle_t *pPmicCoreHandle,
                               uint8_t regValue, Pmic_IrqStatus_t *pErrStat);

static int32_t Pmic_bb_getWDGErr(Pmic_CoreHandle_t *pPmicCoreHandle,
                                 Pmic_IrqStatus_t *pErrStat);

static int32_t Pmic_bb_getSafetyOffStateErr(uint8_t regValue,
                                            Pmic_IrqStatus_t *pErrStat);

int32_t Pmic_bb_irqGetL2Error(Pmic_CoreHandle_t *pPmicCoreHandle,
                              uint16_t l1RegAddr, Pmic_IrqStatus_t *pErrStat);

void pmic_get_bb_intrCfg(Pmic_IntrCfg_t **pIntrCfg);

void pmic_get_bb_intrGpioCfg(Pmic_GpioIntrTypeCfg_t **pGpioIntrCfg);

void Pmic_BB_reInitInterruptConfig(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* PMIC_IRQ_TPS65386X_H_ */
