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
 *  \addtogroup DRV_PMIC_POWER_MODULE
 *
 *  @{
 */

/**
 * \file   pmic_power_tps65386x.h
 *
 * \brief  PMIC TPS65386x BB PMIC Power Resources Driver API/interface file.
 *
 */

#ifndef PMIC_POWER_TPS65386X_H_
#define PMIC_POWER_TPS65386X_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <pmic_core_priv.h>
#include <pmic_io_priv.h>
#include <pmic_power_priv.h>
#include <pmic_types.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/**
 *  \anchor Pmic_TPS65386x BB_Power_ResourceType
 *  \name   PMIC Power Resource Type for BB TPS65386x
 *
 *  @{
 */
#define PMIC_TPS65386X_POWER_RESOURCE_TYPE_VCCA (0U)
#define PMIC_TPS65386X_POWER_RESOURCE_TYPE_BUCK (1U)
#define PMIC_TPS65386X_POWER_RESOURCE_TYPE_LDO (2U)
#define PMIC_TPS65386X_POWER_RESOURCE_TYPE_PLDO (3U)

/*  @} */

/**
 *  \anchor Pmic_TPS65386x BB_Power_Resource
 *  \name   PMIC Power Resources for BB TPS65386x
 *
 *  @{
 */
#define PMIC_TPS65386X_POWER_SOURCE_VCCA                                       \
  ((((uint16_t)PMIC_TPS65386X_POWER_RESOURCE_TYPE_VCCA << 8U) | 0x0U))

#define PMIC_TPS65386X_REGULATOR_BUCK                                          \
  ((((uint16_t)PMIC_TPS65386X_POWER_RESOURCE_TYPE_BUCK << 8U) | 0x1U))

#define PMIC_TPS65386X_REGULATOR_LDO1                                          \
  ((((uint16_t)PMIC_TPS65386X_POWER_RESOURCE_TYPE_LDO << 8U) | 0x2U))
#define PMIC_TPS65386X_REGULATOR_LDO2                                          \
  ((((uint16_t)PMIC_TPS65386X_POWER_RESOURCE_TYPE_LDO << 8U) | 0x3U))
#define PMIC_TPS65386X_REGULATOR_LDO3                                          \
  ((((uint16_t)PMIC_TPS65386X_POWER_RESOURCE_TYPE_LDO << 8U) | 0x4U))
#define PMIC_TPS65386X_REGULATOR_LDO4                                          \
  ((((uint16_t)PMIC_TPS65386X_POWER_RESOURCE_TYPE_LDO << 8U) | 0x5U))

#define PMIC_TPS65386X_REGULATOR_PLDO1                                         \
  ((((uint16_t)PMIC_TPS65386X_POWER_RESOURCE_TYPE_PLDO << 8U) | 0x6U))
#define PMIC_TPS65386X_REGULATOR_PLDO2                                         \
  ((((uint16_t)PMIC_TPS65386X_POWER_RESOURCE_TYPE_PLDO << 8U) | 0x7U))

/* PMIC LDO Ramp Time for Soft-Start Configuration Bit */
#define PMIC_LDO_PLDO_SHORT_RAMP_TIME (0U)
#define PMIC_LDO_PLDO_LONG_RAMP_TIME (1U)

/* PMIC PLDO Mode Selection Configuration Bit */
#define PMIC_PLDO_NON_TRACKING_MODE (0U)
#define PMIC_PLDO_TRACKING_MODE (1U)

/* PMIC LDO Current Limit Level configuration Bit */
#define PMIC_LDO_PLDO_ILIM_LVL_CFG_OPT0 (0U)
#define PMIC_LDO_PLDO_ILIM_LVL_CFG_OPT1 (1U)
#define PMIC_LDO_PLDO_ILIM_LVL_CFG_OPT2 (2U)
#define PMIC_LDO_PLDO_ILIM_LVL_CFG_OPT3 (3U)

/* LDO and PLDO voltage level configuration macros */
#define PMIC_LDO_PLDO_LVL_CFG_VOLT_1_0V (0U)
#define PMIC_LDO_PLDO_LVL_CFG_VOLT_1_05V (1U)
#define PMIC_LDO_PLDO_LVL_CFG_VOLT_1_1V (2U)
#define PMIC_LDO_PLDO_LVL_CFG_VOLT_1_15V (3U)
#define PMIC_LDO_PLDO_LVL_CFG_VOLT_1_2V (4U)
#define PMIC_LDO_PLDO_LVL_CFG_VOLT_1_25V (5U)
#define PMIC_LDO_PLDO_LVL_CFG_VOLT_1_3V (6U)
#define PMIC_LDO_PLDO_LVL_CFG_VOLT_1_35V (7U)
#define PMIC_LDO_PLDO_LVL_CFG_VOLT_1_4V (8U)
#define PMIC_LDO_PLDO_LVL_CFG_VOLT_1_45V (9U)
#define PMIC_LDO_PLDO_LVL_CFG_VOLT_1_5V (AU)
#define PMIC_LDO_PLDO_LVL_CFG_VOLT_1_55V (BU)
#define PMIC_LDO_PLDO_LVL_CFG_VOLT_1_6V (CU)
#define PMIC_LDO_PLDO_LVL_CFG_VOLT_1_65V (DU)
#define PMIC_LDO_PLDO_LVL_CFG_VOLT_1_7V (EU)
#define PMIC_LDO_PLDO_LVL_CFG_VOLT_1_75V (FU)
#define PMIC_LDO_PLDO_LVL_CFG_VOLT_1_8V (10U)
#define PMIC_LDO_PLDO_LVL_CFG_VOLT_2_5V (11U)
#define PMIC_LDO_PLDO_LVL_CFG_VOLT_3V (12U)
#define PMIC_LDO_PLDO_LVL_CFG_VOLT_3_3V (13U)
#define PMIC_LDO_PLDO_LVL_CFG_VOLT_5V (14U)
#define PMIC_LDO_LVL_CFG_VOLT_BYPASS_MODE (15U)

/* PLDO Voltage monitoring range macros */
#define PMIC_PLDO_VTRACK_RNG_LESS_THAN_2V (0U)
#define PMIC_PLDO_VTRACK_RNG_GRT_THAN_2V (1U)

/*==========================================================================*/
/*                         Structures and Enums                             */
/*==========================================================================*/

/**
 * @struct Pmic_powerBuckBoostCfgReg_s
 * @brief Structure representing the configuration register for Buck-Boost
 * operation.
 *
 * This structure provides a way to organize and access the individual bit
 * fields of the Buck-Boost Configuration Register.
 */
typedef struct Pmic_powerBuckBoostCfgReg_s {
  uint8_t bbPgoodCfg; /**< Bit field for Buck-Boost PGOOD configuration. */
  uint8_t bbSsEn;     /**< Bit field for Buck-Boost Dual Random Spread Spectrum
                         (DRSS) modulation. */
  uint8_t bbStbyLvlCfg;  /**< Bit field for Buck-Boost voltage selection during
                            low-power STANDBY state operation. */
  uint8_t bbLvlCfg;      /**< Bit field for Buck-Boost voltage selection for
                            operating and sequencing states. */
  uint8_t bbCfgRegAddr;  /**< Bit field for Buck-Boost Register Address. */
  uint8_t bbCfgRegShift; /**< Bit field for Buck-Boost Register Shift Value. */
  uint8_t bbCfgRegMask;  /**< Bit field for Buck-Boost Register Mask Value. */
} Pmic_powerBuckBoostCfgReg_t;

typedef struct Pmic_ldoCfgReg_s {
  uint8_t ldoRtCfg;
  uint8_t ldoIlimLvlCfg;
  uint8_t ldoLvlCfg;
  uint8_t ldoRegAddr;
  uint8_t ldoRegShift;
  uint8_t ldoRegMask;
} Pmic_ldoCfgReg_t;

typedef struct Pmic_pldoCfgReg_s {
  uint8_t pldoModeSel;
  uint8_t pldoIlimLvlCfg;
  uint8_t pldoLvlCfg;
  uint8_t pldoRegAddr;
  uint8_t pldoRegShift;
  uint8_t pldoRegMask;
} Pmic_pldoCfgReg_t;

typedef struct Pmic_pldoVTrackRtReg_s {
  uint8_t pldoVTrackRng;
  uint8_t pldo1RTCfgVal;
  uint8_t pldo2RTCfgVal;
  uint8_t pldoVTrackRTRegAddr;
  uint8_t pldoVTrackRTRegShift;
  uint8_t pldoVTrackRTRegMask;
} Pmic_pldoVTrackRtReg_t;

typedef struct Pmic_pgoodCfgReg_s {
  uint8_t pldo2PgoodCfg; /**< Bit field for PLDO2 PGOOD configuration. */
  uint8_t pldo1PgoodCfg; /**< Bit field for PLDO1 PGOOD configuration. */
  uint8_t ldo4PgoodCfg;  /**< Bit field for LDO4 PGOOD configuration. */
  uint8_t ldo3PgoodCfg;  /**< Bit field for LDO3 PGOOD configuration. */
  uint8_t ldo2PgoodCfg;  /**< Bit field for LDO2 PGOOD configuration. */
  uint8_t ldo1PgoodCfg;  /**< Bit field for LDO1 PGOOD configuration. */
  uint8_t pgoodRegAddr;
  uint8_t pgoodRegShift;
  uint8_t pgoodRegMask;
} Pmic_pgoodCfgReg_t;

typedef struct Pmic_pldoEnOutCtrlReg_s {
  uint8_t enOut2Enable;
  uint8_t enOut1Enable;
  uint8_t pldo2Ctrl;
  uint8_t pldo1Ctrl;
  uint8_t pldoEnOutRegAddr;
} Pmic_pldoEnOutCtrlReg_t;

typedef struct Pmic_DscgDisCtrlReg_s {
  uint8_t pldo2DscgDis; /**< Bit 5 (PLDO2_DSCG_DIS). */
  uint8_t pldo1DscgDis; /**< Bit 4 (PLDO1_DSCG_DIS). */
  uint8_t ldo4DscgDis;  /**< Bit 3 (LDO4_DSCG_DIS). */
  uint8_t ldo3DscgDis;  /**< Bit 2 (LDO3_DSCG_DIS). */
  uint8_t ldo2DscgDis;  /**< Bit 1 (LDO2_DSCG_DIS). */
  uint8_t ldo1DscgDis;  /**< Bit 0 (LDO1_DSCG_DIS). */
  uint8_t dscgDisCtrlRegAddr;
} Pmic_DscgDisCtrlReg_t;

typedef struct Pmic_powerResourceConfig_s {
  uint8_t pmicConfigShiftVal;
  uint8_t pmicConfigMaskVal;
} Pmic_powerRsrcCfg_t;

typedef struct Pmic_powerResourceRegCfg_s {
  uint8_t buckConfigRegAddr;
  uint8_t ldo1ConfigRegAddr;
  uint8_t ldo2ConfigRegAddr;
  uint8_t ldo3ConfigRegAddr;
  uint8_t ldo4ConfigRegAddr;
  uint8_t pldo1ConfigRegAddr;
  uint8_t pldo2ConfigRegAddr;
  uint8_t pldoConfigRegAddr;
  uint8_t dscgConfigRegAddr;
  uint8_t pgoodConfigRegAddr;
  uint8_t ldoCtrlRegAddr;
  uint8_t enoutCtrlRegAddr;
} Pmic_powerRsrcRegCfg_t;

/*==========================================================================*/
/*                         Function Declarations                            */
/*==========================================================================*/

static void initializeBuckBoostCfgReg(Pmic_powerBuckBoostCfgReg_t *config) {
  /* Set default initialization values */
  config->bbPgoodCfg = PMIC_ST_DEFAULT_DATA;
  config->bbSsEn = PMIC_ST_DEFAULT_DATA;
  config->bbStbyLvlCfg = PMIC_ST_DEFAULT_DATA;
  config->bbLvlCfg = PMIC_ST_DEFAULT_DATA;
}

static void initializeLDOCfgReg(Pmic_ldoCfgReg_t *config) {
  /* Set default initialization values */
  config->ldoRtCfg = PMIC_ST_DEFAULT_DATA;
  config->ldoIlimLvlCfg = PMIC_ST_DEFAULT_DATA;
  config->ldoLvlCfg = PMIC_ST_DEFAULT_DATA,
  config->ldoRegAddr = PMIC_ST_DEFAULT_DATA;
}

static void initializePLDOCfgReg(Pmic_pldoCfgReg_t *config) {
  /* Set default initialization values */
  config->pldoModeSel = PMIC_ST_DEFAULT_DATA;
  config->pldoIlimLvlCfg = PMIC_ST_DEFAULT_DATA;
  config->pldoLvlCfg = PMIC_ST_DEFAULT_DATA,
  config->pldoRegAddr = PMIC_ST_DEFAULT_DATA;
}

static void initializePLDOVTrackRTReg(Pmic_pldoVTrackRtReg_t *config) {
  /* Set default initialization values */
  config->pldoVTrackRng = PMIC_ST_DEFAULT_DATA;
  config->pldo1RTCfgVal = PMIC_ST_DEFAULT_DATA;
  config->pldo2RTCfgVal = PMIC_ST_DEFAULT_DATA;
}

static void initializePLDOEnOutReg(Pmic_pldoEnOutCtrlReg_t *config) {
  /* Set default initialization values */
  config->enOut1Enable = PMIC_ST_DEFAULT_DATA;
  config->enOut2Enable = PMIC_ST_DEFAULT_DATA;
  config->pldo1Ctrl = PMIC_ST_DEFAULT_DATA;
  config->pldo2Ctrl = PMIC_ST_DEFAULT_DATA;
  config->pldoEnOutRegAddr = PMIC_ST_DEFAULT_DATA;
}

void pmic_get_tps65386x_pwrRsrceRegCfg(Pmic_powerRsrcRegCfg_t **pPwrRsrcRegCfg);

int32_t Pmic_powerSetBuckBstCfg(Pmic_CoreHandle_t *pPmicCoreHandle,
                                Pmic_powerBuckBoostCfgReg_t *buckBstCfg,
                                Pmic_powerRsrcRegCfg_t *pwrRsrcRegCfg,
                                Pmic_powerRsrcCfg_t *pwrRsrcCfg);

int32_t Pmic_powerSetLdoConfigRegister(void *pPmicCoreHandle, uint8_t ldoNumber,
                                       Pmic_ldoCfgReg_t *ldoConfig,
                                       Pmic_powerRsrcCfg_t *pwrRsrcCfg,
                                       Pmic_powerRsrcRegCfg_t *pwrRsrcRegCfg);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* PMIC_POWER_TPS65386X_H_ */

/* @} */
