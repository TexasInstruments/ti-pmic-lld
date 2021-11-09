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
 *  \ingroup DRV_PMIC_MODULE
 *  \defgroup DRV_PMIC_COMMON_MODULE PMIC Common Driver API
 *
 *  PMIC common Driver contains Miscellaneous APIs supported by all supported
 *  PMIC Devices.
 *  Like, PMIC recovery count APIs, Read/Write Scratchpad registers APIs
 *  and PMIC nSLEEP Setup APIs.
 *
 *  @{
 */

/**
 *  \file pmic_core.h
 *
 *  \brief PMIC Driver Common API/interface file.
 */

#ifndef PMIC_CORE_H_
#define PMIC_CORE_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <pmic_types.h>
#include <cfg/tps6594x/pmic_core_tps6594x.h>
#include <cfg/lp8764x/pmic_core_lp8764x.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */
/*!
 * \brief  PMIC driver Core Handle INIT status Magic Number.
 *         Used to validate Handle to avoid corrupted PmicHandle usage.
 *         on Success: (DRV_INIT_SUCCESS | Pmic_InstType_t)
 */
#define DRV_INIT_SUCCESS                      (0xABCD0000U)

/** \brief Silicon Revision Id - PG 2.0 for TPS6594x Leo and LP8764x Hera */
#define PMIC_SILICON_REV_ID_PG_2_0           (0x08U)

/** \brief Silicon Revision Id - PG 1.0 for TPS6594x Leo and LP8764x Hera */
#define PMIC_SILICON_REV_ID_PG_1_0           (0x0U)

/**
 *  \anchor Pmic_RecoveryCntCfgType
 *  \name PMIC Recovery Counter Configuration Type
 *
 *  @{
 */
 /** \brief validParams value used to set/get Recovery Counter Threshold Value
  */
#define PMIC_CFG_RECOV_CNT_THR_VAL_VALID      (0U)
 /** \brief validParams value used to Clear/get Recovery Counter Value*/
#define PMIC_CFG_RECOV_CNT_CLR_CNT_VALID      (1U)
/* @} */

/**
 *  \anchor Pmic_RecoveryCntCfgTypeStructPrmBitShiftVal
 *  \name   PMIC Recovery Count Configuration Structure Param Bit shift values
 *
 *  Application can use below shifted values to set the validParams
 *  struct members defined in Pmic_RecovCntCfg_t structure
 *
 *  @{
 */
#define PMIC_CFG_RECOV_CNT_THR_VAL_VALID_SHIFT  \
                       (1U << PMIC_CFG_RECOV_CNT_THR_VAL_VALID)
#define PMIC_CFG_RECOV_CNT_CLR_CNT_VALID_SHIFT  \
                       (1U << PMIC_CFG_RECOV_CNT_CLR_CNT_VALID)
/* @} */

/**
 *  \anchor Pmic_ScratchPad_Sel
 *  \name   PMIC Scratchpad register selection
 *
 *  @{
 */
#define PMIC_SCRATCH_PAD_REG_1             (0x0U)
#define PMIC_SCRATCH_PAD_REG_2             (0x1U)
#define PMIC_SCRATCH_PAD_REG_3             (0x2U)
#define PMIC_SCRATCH_PAD_REG_4             (0x3U)
/*  @} */

/**
 *  \anchor Pmic_UserSpareReg_Sel
 *  \name   PMIC User Spare register selection
 *
 *  @{
 */
#define PMIC_USER_SPARE_REG_1             (0x0U)
#define PMIC_USER_SPARE_REG_2             (0x1U)
#define PMIC_USER_SPARE_REG_3             (0x2U)
#define PMIC_USER_SPARE_REG_4             (0x3U)
/*  @} */

/**
 *  \anchor Pmic_UserSpareReg_Val
 *  \name   PMIC User Spare register Value
 *
 *  @{
 */
#define PMIC_USER_SPARE_REG_VAL_0         (0x0U)
#define PMIC_USER_SPARE_REG_VAL_1         (0x1U)
/*  @} */

/**
 *  \anchor Pmic_CommonCtrlStatValidParamCfg
 *  \name   PMIC Common control param status Structure Param Bits
 *
 *  PMIC valid params configuration type for
 *  the structure member validParams of Pmic_CommonCtrlStat_t
 *  structure
 *
 *  @{
 */
 /** \brief validParams value used to get Backup Battery End of charge Indication
  *         Status */
#define PMIC_CFG_BB_EOC_INDICATION_STAT_VALID               (0U)
/** \brief validParams value used to get Register lock status */
#define PMIC_CFG_REGISTER_LOCK_STAT_VALID                   (1U)
/** \brief validParams value used to get External clock validity status */
#define PMIC_CFG_EXT_CLK_VALIDITY_STAT_VALID                (2U)
/** \brief validParams value used to get Startup(nPWRON/Enable) pin status */
#define PMIC_CFG_STARTUP_PIN_STAT_VALID                     (3U)
/** \brief validParams value used to get EN_DRV Pin status */
#define PMIC_CFG_EN_DRV_PIN_STAT_VALID                      (4U)
/** \brief validParams value used to get nRSTOUT_SOC Pin status */
#define PMIC_CFG_NRSTOUTSOC_PIN_STAT_VALID                  (5U)
/** \brief validParams value used to get nRSTOUT Pin status */
#define PMIC_CFG_NRSTOUT_PIN_STAT_VALID                     (6U)
/** \brief validParams value used to get nINT Pin status */
#define PMIC_CFG_NINT_PIN_STAT_VALID                        (7U)
 /** \brief validParams value used to get SPMI Low Power Mode status */
#define PMIC_CFG_SPMI_LPM_STAT_VALID                        (8U)
 /** \brief validParams value used to get status of ENABLE_DRV Configuration by
  *         I2C/SPI */
#define PMIC_CFG_FORCE_ENABLE_DRV_LOW_STAT_VALID            (9U)
/*  @} */

/**
 *  \anchor Pmic_CommonCtrlStatValidParamBitShiftValues
 *  \name   PMIC Common Control param status valid param bit shift values
 *
 *  Application can use below shifted values to set the validParam
 *  member defined in Pmic_CommonCtrlStat_t structure
 *
 *  @{
 */
#define PMIC_CFG_BB_EOC_INDICATION_STAT_VALID_SHIFT       \
                         (1U << PMIC_CFG_BB_EOC_INDICATION_STAT_VALID)
#define PMIC_CFG_REGISTER_LOCK_STAT_VALID_SHIFT       \
                         (1U << PMIC_CFG_REGISTER_LOCK_STAT_VALID)
#define PMIC_CFG_EXT_CLK_VALIDITY_STAT_VALID_SHIFT       \
                         (1U << PMIC_CFG_EXT_CLK_VALIDITY_STAT_VALID)
#define PMIC_CFG_STARTUP_PIN_STAT_VALID_SHIFT       \
                         (1U << PMIC_CFG_STARTUP_PIN_STAT_VALID)
#define PMIC_CFG_EN_DRV_PIN_STAT_VALID_SHIFT       \
                         (1U << PMIC_CFG_EN_DRV_PIN_STAT_VALID)
#define PMIC_CFG_NRSTOUTSOC_PIN_STAT_VALID_SHIFT       \
                         (1U << PMIC_CFG_NRSTOUTSOC_PIN_STAT_VALID)
#define PMIC_CFG_NRSTOUT_PIN_STAT_VALID_SHIFT       \
                         (1U << PMIC_CFG_NRSTOUT_PIN_STAT_VALID)
#define PMIC_CFG_NINT_PIN_STAT_VALID_SHIFT       \
                         (1U << PMIC_CFG_NINT_PIN_STAT_VALID)
#define PMIC_CFG_SPMI_LPM_STAT_VALID_SHIFT       \
                         (1U << PMIC_CFG_SPMI_LPM_STAT_VALID)
#define PMIC_CFG_FORCE_ENABLE_DRV_LOW_STAT_VALID_SHIFT       \
                         (1U << PMIC_CFG_FORCE_ENABLE_DRV_LOW_STAT_VALID)
/*  @} */

/**
 *  \anchor Pmic_SignalLvl
 *  \name   PMIC signal level of the STARTUP(NPWRON/ENABLE)/ NINT / NRSTOUT_SOC/
 *          NRSTOUT/ EN_DRV Pin
 *
 *  @{
 */
#define PMIC_PIN_SIGNAL_LEVEL_LOW     (0U)
#define PMIC_PIN_SIGNAL_LEVEL_HIGH    (1U)
/*  @} */

/**
 *  \anchor Pmic_PinType_Sel
 *  \name   PMIC Pin Type Selection as NRSTOUT_SOC/ NRSTOUT/ EN_DRV
 *
 *  @{
 */
#define PMIC_PIN_TYPE_EN_DRV                (0U)
#define PMIC_PIN_TYPE_NRSTOUT_SOC           (1U)
#define PMIC_PIN_TYPE_NRSTOUT               (2U)
/*  @} */

/**
 *  \anchor Pmic_EnableDrvI2CSPICfg_Stat
 *  \name   PMIC ENABLE_DRV I2C/SPI configuration status
 *
 *  @{
 */
#define PMIC_ENABLE_DRV_I2C_SPI_CONFIG_ENABLE            (0U)
#define PMIC_ENABLE_DRV_I2C_SPI_CONFIG_DISABLE           (1U)
/*  @} */

/**
 *  \anchor Pmic_RegisterLock_Config
 *  \name   PMIC Register Lock Configuration
 *
 *  @{
 */
 /** \brief Unlock PMIC Registers */
#define PMIC_REGISTER_UNLOCK             (0x9BU)
/** \brief  Lock PMIC Registers - Write any value other than 0x9B.
 *          Here 0x10 is used  */
#define PMIC_REGISTER_LOCK               (0x10U)
/*  @} */

/**
 *  \anchor Pmic_RegisterLock_Stat
 *  \name   PMIC Register Lock Status
 *
 *  @{
 */
#define PMIC_REGISTER_STATUS_UNLOCK             (0x0U)
#define PMIC_REGISTER_STATUS_LOCK               (0x1U)
/*  @} */

/**
 *  \anchor Pmic_SpreadSpectrum_Cfg
 *  \name   PMIC Spread Spectrum Configuration
 *
 *  @{
 */
 /** \brief Disable Spread Spectrum Configuration */
#define PMIC_SPREAD_SPECTRUM_CFG_DISABLE              0U
/** \brief Enable Spread Spectrum Configuration */
#define PMIC_SPREAD_SPECTRUM_CFG_ENABLE               1U

/*  @} */

/**
 *  \anchor Pmic_SpreadSpectrum_Mod_Depth_Sel
 *  \name   PMIC Spread Spectrum Modulation Depth Sel
 *
 *  @{
 */
 /** \brief No Modulation */
#define PMIC_SPREAD_SPECTRUM_MODULATION_DEPTH_NONE             (0U)
/** \brief  Modulation Depth as +/- 6.3%  */
#define PMIC_SPREAD_SPECTRUM_MODULATION_DEPTH_6_3_PERCENT      (1U)
/** \brief  Modulation Depth as +/- 8.4% */
#define PMIC_SPREAD_SPECTRUM_MODULATION_DEPTH_8_4_PERCENT      (2U)
/*  @} */

/**
 *  \anchor Pmic_SpmiLpmModeCtrl_Stat
 *  \name   PMIC SPMI LPM Mode Control Status
 *
 *  @{
 */
 /** \brief SPMI LPM Mode Control Configuration is Disabled */
#define PMIC_SPMI_LPM_MODE_CTRL_CFG_DISABLED     (0U)
/** \brief SPMI LPM Mode Control Configuration is Enabled */
#define PMIC_SPMI_LPM_MODE_CTRL_CFG_ENABLED      (1U)
/*  @} */

/**
 *  \anchor Pmic_InternalClkMonitor_Cfg
 *  \name   PMIC Internal Clock Monitoring Configuration
 *
 *  @{
 */
 /** \brief Disable Internal Clock Monitoring */
#define PMIC_INTERNAL_CLK_MONITORING_CFG_DISABLE     (0U)
/** \brief Enable Internal Clock Monitoring */
#define PMIC_INTERNAL_CLK_MONITORING_CFG_ENABLE      (1U)
/*  @} */

/**
 *  \anchor Pmic_SyncClkOut_Freq_Sel
 *  \name   PMIC SYNCCLKOUT Frequency selection
 *
 *  @{
 */
 /** \brief DISABLE SYNCCLKOUT */
#define PMIC_SYNCCLKOUT_DISABLE      (0U)
/** \brief  SYNCCLKOUT Frequency as 1.1 MHz */
#define PMIC_SYNCCLKOUT_1_1_MHZ      (1U)
/** \brief  SYNCCLKOUT Frequency as 2.2 MHz */
#define PMIC_SYNCCLKOUT_2_2_MHZ      (2U)
/** \brief  SYNCCLKOUT Frequency as 4.4 MHz */
#define PMIC_SYNCCLKOUT_4_4_MHZ      (3U)
/*  @} */

/**
 *  \anchor Pmic_ExtClk_Sel
 *  \name   PMIC External Clock selection
 *
 *  @{
 */
 /** \brief Forced to Internal RC Oscillator */
#define PMIC_INTERNAL_RC_OSC             (0U)
/** \brief  Selects External clock when Available  */
#define PMIC_AUTOMATIC_EXT_CLK           (1U)
/*  @} */

/**
 *  \anchor Pmic_ExtClkValidStat
 *  \name   PMIC External Clock Validity Status
 *
 *  @{
 */
#define PMIC_EXT_CLK_STATUS_VALID       (0U)
#define PMIC_EXT_CLK_STATUS_INVALID     (1U)
/*  @} */

/**
 *  \anchor Pmic_CrcStatus
 *  \name   PMIC CRC Status for I2C1/I2C2/SPI
 *
 *  @{
 */
 /** \brief  CRC is disabled  */
#define PMIC_CRC_STATUS_DISABLED    (0U)
 /** \brief  CRC is enabled  */
#define PMIC_CRC_STATUS_ENABLED     (1U)
/*  @} */

/**
 *  \anchor Pmic_CommonCtrlValidParamCfg
 *  \name   PMIC Common Control Configuration Structure Param Bits
 *
 *  PMIC valid params configuration type for
 *  the structure member validParams of Pmic_CommonCtrlCfg_t
 *  structure
 *
 *  @{
 */
 /** \brief validParams value used to set/get Spread Spectrum
  *         Enable/Disable */
#define PMIC_CFG_SPREAD_SPECTRUM_EN_VALID         (0U)
/** \brief validParams value used to set/get ENABLE_DRV pin
  *         configuration */
#define PMIC_CFG_ENABLE_DRV_VALID                 (1U)
 /** \brief validParams value used to set/get Register Lock
  *         configuration */
#define PMIC_CFG_REG_LOCK_VALID                   (2U)
/** \brief validParams value used to set/get Spread Spectrum
  *         modulation Depth Value */
#define PMIC_CFG_SPREAD_SPECTRUM_DEPTH_VALID      (3U)
 /** \brief validParams value used to set/get to Enable/Disable load
  *         from EEPROM defaults on RTC Domain(TPS6594X Leo)/ conf(LP8764x Hera)
  *         registers */
#define PMIC_CFG_EEPROM_DEFAULT_VALID             (4U)
 /** \brief validParams value used to set/get to Enable/Disable to
  *         skip EEPROM defaults load on conf and Other registers
  *         Valid only for LP8764x Hera*/
#define PMIC_CFG_SKIP_EEPROM_LOAD_VALID           (5U)
/*  @} */

/**
 *  \anchor Pmic_CommonCtrlValidParamBitShiftValues
 *  \name   PMIC Common Control Configuration valid param bit shift values
 *
 *  Application can use below shifted values to set the validParam
 *  member defined in Pmic_CommonCtrlCfg_t structure
 *
 *  @{
 */
#define PMIC_CFG_SPREAD_SPECTRUM_EN_VALID_SHIFT       \
                         (1U << PMIC_CFG_SPREAD_SPECTRUM_EN_VALID)
#define PMIC_CFG_ENABLE_DRV_VALID_SHIFT       \
                         (1U << PMIC_CFG_ENABLE_DRV_VALID)
#define PMIC_CFG_REG_LOCK_VALID_SHIFT       \
                         (1U << PMIC_CFG_REG_LOCK_VALID)
#define PMIC_CFG_SPREAD_SPECTRUM_DEPTH_VALID_SHIFT       \
                         (1U << PMIC_CFG_SPREAD_SPECTRUM_DEPTH_VALID)
#define PMIC_CFG_EEPROM_DEFAULT_VALID_SHIFT       \
                         (1U << PMIC_CFG_EEPROM_DEFAULT_VALID)
#define PMIC_CFG_SKIP_EEPROM_LOAD_VALID_SHIFT       \
                         (1U << PMIC_CFG_SKIP_EEPROM_LOAD_VALID)
/*  @} */

/**
 *  \anchor Pmic_MiscCtrlValidParamCfg
 *  \name   PMIC Miscellaneous Control Configuration Structure Param Bits
 *
 *  PMIC valid params configuration type for
 *  the structure member validParams of Pmic_MiscCtrlCfg_t
 *  structure
 *
 *  @{
 */
  /** \brief validParams value used to set/get to Enable/Disable Band gap
   *          Voltage to AMUX OUT/REF OUT Pin */
#define PMIC_CFG_AMUX_OUT_REF_OUT_EN_VALID        (0U)
 /** \brief validParams value used to set/get to Enable or Disable internal
  *         Clock Monitoring */
#define PMIC_CFG_CLK_MON_EN_VALID                 (1U)
 /** \brief validParams value used to set/get to Select SYNCCLKOUT Frequency */
#define PMIC_CFG_SYNC_CLK_OUT_FREQ_SEL_VALID      (2U)
 /** \brief validParams value used to set/get External clock Selection */
#define PMIC_CFG_EXT_CLK_SEL_VALID                (3U)
 /** \brief validParams value used to set/get to Select External clock Frequency
  */
#define PMIC_CFG_SYNC_CLK_IN_FREQ_VALID           (4U)
 /** \brief validParams value used to set/get to NRSTOUT_SOC Signal
  */
#define PMIC_CFG_NRSTOUT_SOC_VALID                (5U)
 /** \brief validParams value used to set/get NRSTOUT Signal
  */
#define PMIC_CFG_NRSTOUT_VALID                    (6U)
/*  @} */

/**
 *  \anchor Pmic_MiscCtrlValidParamBitShiftValues
 *  \name   PMIC Miscellaneous Control Configuration valid param bit shift
 *          values
 *
 *  Application can use below shifted values to set the validParam
 *  member defined in Pmic_MiscCtrlCfg_t structure
 *
 *  @{
 */
#define PMIC_CFG_AMUX_OUT_REF_OUT_EN_VALID_SHIFT       \
                         (1U << PMIC_CFG_AMUX_OUT_REF_OUT_EN_VALID)
#define PMIC_CFG_CLK_MON_EN_VALID_SHIFT       \
                         (1U << PMIC_CFG_CLK_MON_EN_VALID)
#define PMIC_CFG_SYNC_CLK_OUT_FREQ_SEL_VALID_SHIFT       \
                         (1U << PMIC_CFG_SYNC_CLK_OUT_FREQ_SEL_VALID)
#define PMIC_CFG_EXT_CLK_SEL_VALID_SHIFT       \
                         (1U << PMIC_CFG_EXT_CLK_SEL_VALID)
#define PMIC_CFG_SYNC_CLK_IN_FREQ_VALID_SHIFT       \
                         (1U << PMIC_CFG_SYNC_CLK_IN_FREQ_VALID)
#define PMIC_CFG_NRSTOUT_SOC_VALID_SHIFT       \
                         (1U << PMIC_CFG_NRSTOUT_SOC_VALID)
#define PMIC_CFG_NRSTOUT_VALID_SHIFT       \
                         (1U << PMIC_CFG_NRSTOUT_VALID)
/*  @} */

/**
 *  \anchor Pmic_BatteryCtrlValidParamCfg
 *  \name   PMIC Backup Battery Control Configuration Structure Param Bits
 *
 *  PMIC valid params configuration type for
 *  the structure member validParams of Pmic_BatteryCtrlCfg_t
 *  structure
 *
 *  @{
 */
  /** \brief validParams value used to set/get to Enable/Disable Backup Battery
   *         Charging */
#define PMIC_CFG_CHARGING_EN_VALID                 (0U)
  /** \brief validParams value used to set/get to Backup Battery configuration for
   *         End of charge Voltage */
#define PMIC_CFG_END_OF_CHARGE_VOLTAGE_VALID       (1U)
  /** \brief validParams value used to set/get to Backup Battery charging current
   *         value */
#define PMIC_CFG_CHARGE_CURRENT_VALID              (2U)
/*  @} */

/**
 *  \anchor Pmic_BatteryCtrlValidParamBitShiftValues
 *  \name   PMIC Backup Battery Control Configuration valid param bit shift
 *          values
 *
 *  Application can use below shifted values to set the validParam
 *  member defined in Pmic_BatteryCtrlCfg_t structure
 *
 *  @{
 */
#define PMIC_CFG_CHARGING_EN_VALID_SHIFT       \
                         (1U << PMIC_CFG_CHARGING_EN_VALID)
#define PMIC_CFG_END_OF_CHARGE_VOLTAGE_VALID_SHIFT       \
                         (1U << PMIC_CFG_END_OF_CHARGE_VOLTAGE_VALID)
#define PMIC_CFG_CHARGE_CURRENT_VALID_SHIFT       \
                         (1U << PMIC_CFG_CHARGE_CURRENT_VALID)
/*  @} */

/*==========================================================================*/
/*                         Structures and Enums                             */
/*==========================================================================*/
/*!
 * \brief  PMIC Recovery Counter Configuration
 *         Note: validParams is input param for all Set and Get APIs. other
 *         params except validParams is input param for Set APIS and output
 *         param for Get APIs
 *
 * \param   validParams   Selection of structure parameters to be set,
 *                        from the combination of \ref Pmic_RecoveryCntCfgType
 *                        and the corresponding member value must be updated.
 *                        Valid values \ref Pmic_RecoveryCntCfgType
 *  \param  thrVal        Recovery Counter Threshold Value.
 *                         Valid only when PMIC_CFG_RECOV_CNT_THR_VAL_VALID
 *                         bit is set.
 *  \param  clrCnt        Clear Recovery Counter Value and value should be 1U.
 *                         Valid only when PMIC_CFG_RECOV_CNT_CLR_VAL_VALID
 *                         bit is set.
 */
typedef struct Pmic_RecovCntCfg_s
{
    uint8_t    validParams;
    uint8_t    thrVal;
    bool       clrCnt;
} Pmic_RecovCntCfg_t;

/*!
 * \brief  PMIC common control param configuration
 *         Note: validParams is input param for all Set and Get APIs. other
 *         params except validParams is input param for Set APIs and output
 *         param for Get APIs
 *
 * \param   validParams             Selection of structure parameters to be set,
 *                                  from the combination of
 *                                  \ref Pmic_CommonCtrlValidParamCfg
 *                                  and the corresponding member value must be
 *                                  updated
 *                                    Valid values
 *                                        \ref Pmic_CommonCtrlValidParamCfg
 *  \param  sreadSpectrumEn         Spread Spectrum Enable Value
 *                                  Valid only when
 *                                  PMIC_CFG_SPREAD_SPECTRUM_EN_VALID bit is set
 *                                    Valid values \ref Pmic_SpreadSpectrum_Cfg
 *  \param  skipEepromDefaultLoadEn Enable/Disable to skip EEPROM defaults load
 *                                  on conf registers when device transition
 *                                  from  Lpstandby to INIT state
 *                                  Valid only for LP8764x Hera Device
 *                                  Valid only when
 *                                  PMIC_CFG_SKIP_EEPROM_LOAD_VALID
 *                                  bit is set.
 *                                    Valid values
 *                                    \ref Pmic_Lp8764xHera_Skip_EepromDef_LdCfg
 *  \param  eepromDefaultLoad       Load/Not Loaded from EEPROM defaults on
 *                                  RTC domain Registers (for TPS6594x Leo
 *                                  Device) when device transition from
 *                                  Lpstandby/SafeRecovery to INIT state
 *                                  Enable/Disable load from EEPROM defaults on
 *                                  conf registers when
 *                                  skipEepromDefaultLoadEn = 0
 *                                  when device transition from Lpstandby to
 *                                  INIT state
 *                                  Load/Not Loaded load from EEPROM defaults on
 *                                  conf registers when device transition from
 *                                  SafeRecovery to INIT state.Doesn't depends
 *                                  on  skipEepromDefaultLoadEn Value
 *                                  (for LP8764x Hera Device)
 *                                  Valid only when
 *                                  PMIC_CFG_EEPROM_DEFAULT_VALID bit is set.
 *                                      Valid values
 *                                         \ref Pmic_Tps6594xLeo_EepromDef_LdCfg
 *                                           (for TPS6594x Leo Device)
 *                                         \ref Pmic_Lp8764xHera_EepromDef_LdCfg
 *                                           (for LP8764x Hera Device)
 *  \param  enDrv                   Control of ENABLE_DRV pin. Can be configured
 *                                  only When forceEnDrvLow set to 0 else
 *                                  ENABLE_DRV pin is set to
 *                                   PMIC_PIN_SIGNAL_LEVEL_LOW.
 *                                     Valid values \ref Pmic_SignalLvl
 *                                  Valid only when PMIC_CFG_ENABLE_DRV_VALID
 *                                  bit is set.
 *  \param  regLock                 Register Lock configuration
 *                                  Valid values \ref Pmic_RegisterLock_Config
 *                                  Valid only when PMIC_CFG_REG_LOCK_VALID
 *                                  bit is set
 *                                  Valid only for Pmic_setCommonCtrlConfig API
 *  \param  spreadSpectrumDepth     Spread Spectrum modulation Depth Value
 *                                     Valid values
 *                                        \ref Pmic_SpreadSpectrum_Mod_Depth_Sel
 *                                  Valid only when
 *                                  PMIC_CFG_SPREAD_SPECTRUM_DEPTH_VALID
 *                                  bit is set.
 */
typedef struct Pmic_CommonCtrlCfg_s
{
    uint8_t    validParams;
    bool       sreadSpectrumEn;
    bool       skipEepromDefaultLoadEn;
    uint8_t    eepromDefaultLoad;
    uint8_t    enDrv;
    uint8_t    regLock;
    uint8_t    spreadSpectrumDepth;
} Pmic_CommonCtrlCfg_t;

/*!
 * \brief  PMIC Miscellaneous control param Configuration
 *         Note: validParams is input param for all Set and Get APIs. other
 *         params except validParams is input param for Set APIs and output
 *         param for Get APIs
 *
 * \param   validParams         Selection of structure parameters to be set,
 *                              from the combination of
 *                              \ref Pmic_MiscCtrlValidParamCfg and the
 *                              corresponding member value must be updated.
 *                                 Valid values \ref Pmic_MiscCtrlValidParamCfg
 *  \param  amuxOutRefOutEn     Enable/Disable Band gap Voltage to AMUX OUT Pin
 *                              (for TPS6594x Leo Device) or RFF OUT Pin
 *                              (for LP8764x Leo Device)
 *                              Valid only when
 *                              PMIC_CFG_AMUX_OUT_REF_OUT_EN_VALID bit is set
 *                                 Valid values
 *                                     \ref Pmic_Tps6594xLeo_AMuxOutPinCtrl_Cfg
 *                                      (for TPS6594x Leo Device)
 *                                     \ref Pmic_Lp8764xHera_RefOutPinCtrl_Cfg
 *                                      (for LP8764x Hera Device)
 *  \param  clkMonEn            Enable or Disable internal Clock Monitoring
 *                              Valid only when PMIC_CFG_CLK_MON_EN_VALID
 *                              bit is set.
 *                                 Valid values \ref Pmic_InternalClkMonitor_Cfg
 *  \param  syncClkOutFreqSel   Selects SYNCCLKOUT Frequency
 *                                 Valid values \ref Pmic_SyncClkOut_Freq_Sel
 *                              Valid only when
 *                              PMIC_CFG_SYNC_CLK_OUT_FREQ_SEL_VALID bit is set
 *  \param  extClkSel           External clock Selection
 *                                 Valid values \ref Pmic_ExtClk_Sel
 *                              Valid only when PMIC_CFG_EXT_CLK_SEL_VALID
 *                              bit is set.
 *  \param  syncClkInFreq       Selects External clock Frequency
 *                              Valid only when PMIC_CFG_SYNC_CLK_IN_FREQ_VALID
 *                              bit is set.
 *                                 Valid values
 *                                     \ref Pmic_Tps6594xLeo_ExtClk_Freq_Sel
 *                                      (for TPS6594x Leo Device)
 *                                     \ref Pmic_Lp8764xHera_ExtClk_Freq_Sel
 *                                      (for LP8764x Hera Device)
 *  \param  nRstOutSocSignal    Configure NRSTOUT_SOC Signal
 *                              Note: When Application configures
 *                              nRstOutSocSignal as PMIC_PIN_SIGNAL_LEVEL_LOW
 *                              then SOC will be in reset
 *                              Valid only when PMIC_CFG_NRSTOUT_SOC_VALID
 *                              bit is set.
 *                                 Valid values \ref Pmic_SignalLvl
 *  \param  nRstOutSignal       Configure NRSTOUT Signal
 *                              Note: When Application configures
 *                              nRstOutSignal as PMIC_PIN_SIGNAL_LEVEL_LOW
 *                              then MCU will be in reset
 *                              Valid only when PMIC_CFG_NRSTOUT_VALID
 *                              bit is set.
 *                                 Valid values \ref Pmic_SignalLvl
 */
typedef struct Pmic_MiscCtrlCfg_s
{
    uint8_t    validParams;
    bool       amuxOutRefOutEn;
    bool       clkMonEn;
    uint8_t    syncClkOutFreqSel;
    uint8_t    extClkSel;
    uint8_t    syncClkInFreq;
    uint8_t    nRstOutSocSignal;
    uint8_t    nRstOutSignal;
} Pmic_MiscCtrlCfg_t;

/*!
 * \brief  PMIC Backup Battery control param Configuration
 *         Note: validParams is input param for all Set and Get APIs. other
 *         params except validParams is input param for Set APIs and output
 *         param for Get APIs
 *
 * \param   validParams         Selection of structure parameters to be set,
 *                              from the combination of
 *                              \ref Pmic_BatteryCtrlValidParamCfg and the
 *                              corresponding member value must be updated.
 *  \param  chargingEn          Enable/Disable Backup Battery Charging
 *                              Valid only when PMIC_CFG_CHARGING_EN_VALID
 *                              bit is set.
 *                              Valid only for TPS6594x Leo Device
 *                                 Valid values
 *                                    \ref Pmic_Tps6594xLeo_BatteryCharging_Cfg
 *  \param  endOfChargeVoltage  Backup Battery configuration for End of charge
 *                              Voltage
 *                              Valid only when
 *                              PMIC_CFG_END_OF_CHARGE_VOLTAGE_VALID bit is set.
 *                              Valid only for TPS6594x Leo Device
 *                                 Valid values
 *                                  \ref Pmic_Tps6594xLeo_EndOfChargeVoltage_Sel
 *  \param  chargeCurrent       Backup Battery charging current value
 *                              Valid only when PMIC_CFG_CHARGE_CURRENT_VALID
 *                              bit is set.
 *                              Valid only for TPS6594x Leo Device
 *                                 Valid values
 *                                    \ref Pmic_Tps6594xLeo_Charging_Current_Sel
 */
typedef struct Pmic_BatteryCtrlCfg_s
{
    uint8_t    validParams;
    bool       chargingEn;
    uint8_t    endOfChargeVoltage;
    uint8_t    chargeCurrent;
} Pmic_BatteryCtrlCfg_t;

/*!
 * \brief  PMIC common control param status
 *         Note: validParams is input param for all Get APIs. other params
 *         except validParams is output param for Get APIs
 *
 * \param   validParams        Selection of structure parameters to be set, from
 *                             the combination of
 *                             \ref Pmic_CommonCtrlStatValidParamCfg
 *                             and the corresponding member value must be
 *                             updated
 *                                Valid values
 *                                         \ref Pmic_CommonCtrlStatValidParamCfg
 *  \param  spmiLpmStat        SPMI Low Power Mode Control Status.
 *                             Valid only when PMIC_CFG_SPMI_LPM_STAT_VALID
 *                             bit is set.
 *                                Valid values \ref Pmic_SpmiLpmModeCtrl_Stat
 *  \param  forceEnDrvLowStat  Status of ENABLE_DRV Configuration by I2C/SPI
 *                             Valid only when
 *                             PMIC_CFG_FORCE_ENABLE_DRV_LOW_STAT_VALID
 *                             bit is set.
 *                                Valid values \ref Pmic_EnableDrvI2CSPICfg_Stat
 *  \param  bbEocIndication    Backup Battery End of charge Indication Status
 *                             Valid only when
 *                             PMIC_CFG_BB_EOC_INDICATION_STAT_VALID
 *                             bit is set
 *                              Valid only for TPS6594x Leo Device
 *                                 Valid values
 *                                     \ref Pmic_Tps6594xLeo_BBEoCIndicationStat
 *  \param  regLockStat        Register lock status
 *                             Valid only when PMIC_CFG_REGISTER_LOCK_STAT_VALID
 *                             bit is set
 *                                 Valid values \ref Pmic_RegisterLock_Stat
 *  \param  extClkValidity     External clock validity status. The status value
 *                             is valid only when External clock is connected
 *                             Ignore the status value when External clock is not
 *                             connected
 *                             Valid only when
 *                             PMIC_CFG_EXT_CLK_VALIDITY_STAT_VALID bit is set
 *                                Valid values \ref Pmic_ExtClkValidStat
 *  \param  startupPin         Startup(nPWRON/Enable) pin status
 *                             Valid only when PMIC_CFG_STARTUP_PIN_STAT_VALID
 *                             bit is set
 *                                Valid values \ref Pmic_SignalLvl
 *  \param  enDrvPin           EN_DRV Pin status
 *                             Valid only when PMIC_CFG_EN_DRV_PIN_STAT_VALID
 *                             bit is set.
 *                                Valid values \ref Pmic_SignalLvl
 *  \param  nRstOutSocPin      nRSTOUT_SOC Pin status
 *                             Valid only when
 *                             PMIC_CFG_NRSTOUTSOC_PIN_STAT_VALID bit is set.
 *                                Valid values \ref Pmic_SignalLvl
 *  \param  nRstOutPin        nRSTOUT Pin status
 *                             Valid only when PMIC_CFG_NRSTOUT_PIN_STAT_VALID
 *                             bit is set.
 *                                Valid values \ref Pmic_SignalLvl
 *  \param  nIntPin           nINT Pin status
 *                             Valid only when PMIC_CFG_NINT_PIN_STAT_VALID
 *                             bit is set.
 *                                Valid values \ref Pmic_SignalLvl
 */
typedef struct Pmic_CommonCtrlStat_s
{
    uint32_t   validParams;
    bool       spmiLpmStat;
    uint8_t    forceEnDrvLowStat;
    uint8_t    bbEndOfChargeIndication;
    uint8_t    regLockStat;
    uint8_t    extClkValidity;
    uint8_t    startupPin;
    uint8_t    enDrvPin;
    uint8_t    nRstOutSocPin;
    uint8_t    nRstOutPin;
    uint8_t    nIntPin;
} Pmic_CommonCtrlStat_t;

/*!
 * \brief  PMIC Device Information
 *
 *  \param  deviceID        TI Device ID Value
 *  \param  nvmID           TI NVM ID Value
 *  \param  nvmRev          TI NVM Revision
 *  \param  siliconRev      TI Silicon Revision
 *  \param  customNvmID     Customer configured NVM ID Value
 *                          customNvmID value is valid only for TPS6594x Leo
 *                          PMIC PG2.0 and LP8764x Hera PMIC PG2.0
 */
typedef struct Pmic_DeviceInfo_s
{
    uint8_t    deviceID;
    uint8_t    nvmID;
    uint8_t    nvmRev;
    uint8_t    siliconRev;
    uint8_t    customNvmID;
} Pmic_DeviceInfo_t;

/*==========================================================================*/
/*                         Function Declarations                            */
/*==========================================================================*/
/*!
 * \brief  API to Set Recovery Counter Configuration.
 *
 * Requirement: REQ_TAG(PDK-5809)
 * Design: did_pmic_err_recov_cnt_cfg_readback
 * Architecture: aid_pmic_core_misc_cfg
 *
 *         This function configures PMIC Recovery Counter register, controlling
 *         recovery count Threshold and Clear, when corresponding validParam
 *         bit field is set in the Pmic_RecovCntCfg_t structure.
 *         For more information \ref Pmic_RecovCntCfg_t
 *
 * \param   pPmicCoreHandle   [IN]    PMIC Interface Handle.
 * \param   recovCntCfg       [IN]    Set configuration value for
 *                                    Recovery counter
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_setRecoveryCntCfg(Pmic_CoreHandle_t        *pPmicCoreHandle,
                               const Pmic_RecovCntCfg_t  recovCntCfg);

/*!
 * \brief  API to Get Recovery Counter Configuration.
 *
 * Requirement: REQ_TAG(PDK-5809)
 * Design: did_pmic_err_recov_cnt_cfg_readback
 * Architecture: aid_pmic_core_misc_cfg
 *
 *         This function gets PMIC Recovery Counter configuration values. Like,
 *         recovery count Threshold and Clear, when corresponding validParam
 *         bit field is set in the Pmic_RecovCntCfg_t structure.
 *         For more information \ref Pmic_RecovCntCfg_t
 *
 * \param   pPmicCoreHandle       [IN]       PMIC Interface Handle.
 * \param   pRecovCntCfg          [IN/OUT]   Pointer to store recovery counter
 *                                           configuration value
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_getRecoveryCntCfg(Pmic_CoreHandle_t  *pPmicCoreHandle,
                               Pmic_RecovCntCfg_t *pRecovCntCfg);

/*!
 * \brief  API to Read Recovery Count Value.
 *
 * Requirement: REQ_TAG(PDK-5809)
 * Design: did_pmic_err_recov_cnt_cfg_readback
 * Architecture: aid_pmic_core_misc_cfg
 *
 *         This function reads out the recovery count value.
 *
 * \param   pPmicCoreHandle       [IN]    PMIC Interface Handle.
 * \param   pRecovCntVal          [OUT]   Pointer to store recovery count
 *                                        value
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_getRecoveryCnt(Pmic_CoreHandle_t *pPmicCoreHandle,
                            uint8_t           *pRecovCntVal);

/*!
 * \brief   API to set/write value in/to scratchpad register.
 *
 * Requirement: REQ_TAG(PDK-5810), REQ_TAG(PDK-5843)
 * Design: did_pmic_comm_single_i2c_cfg, did_pmic_comm_spi_cfg
 * Architecture: aid_pmic_core_misc_cfg
 *
 *          This function is used write data to scratchpad register of PMIC
 *
 * \param   pPmicCoreHandle    [IN]    PMIC Interface Handle.
 * \param   scratchPadRegNum   [IN]    ScratchPad register number
 *                                         Valid values \ref Pmic_ScratchPad_Sel
 * \param   data               [IN]    Data/Value to be written to scratchpad.
 *
 * \return  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_setScratchPadValue(Pmic_CoreHandle_t *pPmicCoreHandle,
                                const uint8_t      scratchPadRegNum,
                                const uint8_t      data);

/*!
 * \brief   API to get/read data from scratchpad register.
 *
 * Requirement: REQ_TAG(PDK-5810), REQ_TAG(PDK-5843)
 * Design: did_pmic_comm_single_i2c_cfg, did_pmic_comm_spi_cfg
 * Architecture: aid_pmic_core_misc_cfg
 *
 *          This function is used read data from scratchpad register of PMIC
 *
 * \param   pPmicCoreHandle    [IN]    PMIC Interface Handle.
 * \param   scratchPadRegNum   [IN]    ScratchPad register number
 *                                          Valid values\ref Pmic_ScratchPad_Sel
 * \param   pData               [OUT]  Parameter to hold the Data/Value read
 *                                     from scratchpad.
 *
 * \return  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_getScratchPadValue(Pmic_CoreHandle_t *pPmicCoreHandle,
                                const uint8_t      scratchPadRegNum,
                                uint8_t            *pData);

/*!
 * \brief   API to set/write value in/to User Spare register.
 *
 * Requirement: REQ_TAG(PDK-9133)
 * Design: did_pmic_user_spare_cfg_readback
 * Architecture: aid_pmic_core_misc_cfg
 *
 *          This function is used write data to User Spare register of PMIC
 *
 * \param   pPmicCoreHandle    [IN]    PMIC Interface Handle.
 * \param   userSpareRegNum    [IN]    UserSpare register number
 *                                       Valid values \ref Pmic_UserSpareReg_Sel
 * \param   data               [IN]    Data/Value to be written to UserSpare
 *                                     register
 *                                       Valid values \ref Pmic_UserSpareReg_Val
 *
 * \return  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_setUserSpareValue(Pmic_CoreHandle_t *pPmicCoreHandle,
                               const uint8_t      userSpareRegNum,
                               const uint8_t      data);

/*!
 * \brief   API to get/read data from User Spare register.
 *
 * Requirement: REQ_TAG(PDK-9133)
 * Design: did_pmic_user_spare_cfg_readback
 * Architecture: aid_pmic_core_misc_cfg
 *
 *          This function is used read data from User Spare register of PMIC
 *
 * \param   pPmicCoreHandle    [IN]    PMIC Interface Handle.
 * \param   userSpareRegNum    [IN]    User Spare register number
 *                                       Valid values \ref Pmic_UserSpareReg_Sel
 * \param   pData              [OUT]   Parameter to hold the Data/Value read
 *                                     from User Spare register.
 *
 * \return  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_getUserSpareValue(Pmic_CoreHandle_t *pPmicCoreHandle,
                               const uint8_t      userSpareRegNum,
                               uint8_t           *pData);

/*!
 * \brief   API to set PMIC common control parameter configuration.
 *
 * Requirement: REQ_TAG(PDK-9112), REQ_TAG(PDK-9114), REQ_TAG(PDK-9131),
 *              REQ_TAG(PDK-9143), REQ_TAG(PDK-9111)
 * Design: did_pmic_common_ctrl_cfg_readback
 * Architecture: aid_pmic_core_misc_cfg
 *
 *          This function is used to set the required common control parameter
 *          configuration when corresponding validParam bit field is set in
 *          the Pmic_CommonCtrlCfg_t
 *          For more information \ref Pmic_CommonCtrlCfg_t
 *
 * \param   pPmicCoreHandle [IN]    PMIC Interface Handle.
 * \param   commonCtrlCfg   [IN]    Set PMIC required common control parameter
 *                                  configuration.
 *
 * \return  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_setCommonCtrlConfig(Pmic_CoreHandle_t          *pPmicCoreHandle,
                                 const Pmic_CommonCtrlCfg_t  commonCtrlCfg);

/*!
 * \brief   API to get PMIC common control parameter configuration.
 *
 * Requirement: REQ_TAG(PDK-9112), REQ_TAG(PDK-9114), REQ_TAG(PDK-9131),
 *              REQ_TAG(PDK-9143)
 * Design: did_pmic_common_ctrl_cfg_readback
 * Architecture: aid_pmic_core_misc_cfg
 *
 *          This function is used to get the required common control parameter
 *          configuration when corresponding validParam bit field is set in
 *          the Pmic_CommonCtrlCfg_t
 *          For more information \ref Pmic_CommonCtrlCfg_t
 *
 * \param   pPmicCoreHandle [IN]        PMIC Interface Handle.
 * \param   pCommonCtrlCfg  [IN/OUT]    Pointer to store PMIC required common
 *                                      control parameter configuration.
 *
 * \return  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_getCommonCtrlConfig(Pmic_CoreHandle_t          *pPmicCoreHandle,
                                 Pmic_CommonCtrlCfg_t       *pCommonCtrlCfg);

/*!
 * \brief   API to set PMIC Miscellaneous control parameter configuration.
 *
 * Requirement: REQ_TAG(PDK-9132), REQ_TAG(PDK-9127), REQ_TAG(PDK-9111)
 * Design: did_pmic_misc_ctrl_cfg_readback
 * Architecture: aid_pmic_core_misc_cfg
 *
 *          This function is used to set the required miscellaneous control
 *          parameter configuration when corresponding validParam bit field is
 *          set in the Pmic_MiscCtrlCfg_t
 *          For more information \ref Pmic_MiscCtrlCfg_t
 *
 * \param   pPmicCoreHandle [IN]    PMIC Interface Handle.
 * \param   miscCtrlCfg     [IN]    Set PMIC required miscellaneous control
 *                                  parameter configuration.
 *
 * \return  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_setMiscCtrlConfig(Pmic_CoreHandle_t          *pPmicCoreHandle,
                               const Pmic_MiscCtrlCfg_t    miscCtrlCfg);

/*!
 * \brief   API to get PMIC Miscellaneous control parameter configuration.
 *
 * Requirement: REQ_TAG(PDK-9132), REQ_TAG(PDK-9127)
 * Design: did_pmic_misc_ctrl_cfg_readback
 * Architecture: aid_pmic_core_misc_cfg
 *
 *          This function is used to get the required miscellaneous control
 *          parameter configuration when corresponding validParam bit field is
 *          set in the Pmic_MiscCtrlCfg_t
 *          For more information \ref Pmic_MiscCtrlCfg_t
 *
 * \param   pPmicCoreHandle [IN]       PMIC Interface Handle.
 * \param   pMiscCtrlCfg    [IN/OUT]   Pointer to store PMIC required
 *                                     miscellaneous control parameter
 *                                     configuration.
 *
 * \return  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_getMiscCtrlConfig(Pmic_CoreHandle_t          *pPmicCoreHandle,
                               Pmic_MiscCtrlCfg_t         *pMiscCtrlCfg);

/*!
 * \brief   API to set PMIC Battery Backup control parameter configuration.
 *
 * Requirement: REQ_TAG(PDK-9130), REQ_TAG(PDK-9111)
 * Design: did_pmic_battery_ctrl_cfg_readback
 * Architecture: aid_pmic_core_misc_cfg
 *
 *          This function is used to set the required Battery Backup control
 *          parameter configuration when corresponding validParam bit field is
 *          set in the Pmic_BatteryCtrlCfg_t
 *          For more information \ref Pmic_BatteryCtrlCfg_t
 *
 * \param   pPmicCoreHandle [IN]    PMIC Interface Handle.
 * \param   batteryCtrlCfg  [IN]    Set PMIC required Battery Backup control
 *                                  parameter configuration.
 *
 * \return  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_setBatteryCtrlConfig(Pmic_CoreHandle_t            *pPmicCoreHandle,
                                  const Pmic_BatteryCtrlCfg_t   batteryCtrlCfg);

/*!
 * \brief   API to get PMIC Battery Backup control parameter configuration.
 *         Note: validParams is input param for all Set and Get APIs. other
 *         params except validParams is input param for Set APIs and output
 *         param for Get APIs
 *
 * Requirement: REQ_TAG(PDK-9130)
 * Design: did_pmic_battery_ctrl_cfg_readback
 * Architecture: aid_pmic_core_misc_cfg
 *
 *          This function is used to get the required Battery Backup control
 *          parameter configuration when corresponding validParam bit field is
 *          set in the Pmic_BatteryCtrlCfg_t
 *          For more information \ref Pmic_BatteryCtrlCfg_t
 *
 * \param   pPmicCoreHandle    [IN]       PMIC Interface Handle.
 * \param   pBatteryCtrlCfg    [IN/OUT]   Pointer to store PMIC required Battery
 *                                        Backup control parameter configuration
 *
 * \return  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_getBatteryCtrlConfig(Pmic_CoreHandle_t          *pPmicCoreHandle,
                                  Pmic_BatteryCtrlCfg_t      *pBatteryCtrlCfg);

/*!
 * \brief   API to get PMIC GPIO NRSTOUT_SOC/ NRSTOUT/ EN_DRV Pin
 *
 * Requirement: REQ_TAG(PDK-9137), REQ_TAG(PDK-9131)
 * Design: did_pmic_pin_readback
 * Architecture: aid_pmic_core_misc_cfg
 *
 *          This function is used to read the signal level of the NRSTOUT_SOC/
 *          NRSTOUT/ EN_DRV Pin
 *
 * \param   pPmicCoreHandle [IN]    PMIC Interface Handle
 * \param   pinType         [IN]    PMIC pin type.
 *                                   Valid values of pin type
 *                                   \ref Pmic_PinType_Sel
 * \param   pPinValue       [OUT]   Pointer to store the status of pin type
  *                                    Valid values \ref Pmic_SignalLvl
 *
 * \return  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_getPinValue(Pmic_CoreHandle_t *pPmicCoreHandle,
                         const uint8_t      pinType,
                         uint8_t           *pPinValue);

/*!
 * \brief   API to get PMIC common control parameter status.
 *
 * Requirement: REQ_TAG(PDK-9126), REQ_TAG(PDK-9124), REQ_TAG(PDK-9130),
 *              REQ_TAG(PDK-9125), REQ_TAG(PDK-9139), REQ_TAG(PDK-9138),
 *              REQ_TAG(PDK-9112)
 * Design: did_pmic_common_ctrl_status_readback
 * Architecture: aid_pmic_core_misc_cfg
 *
 *          This function is used to get the required common control parameter
 *          status when corresponding validParam bit field is set in
 *          the Pmic_CommonCtrlStat_t
 *          For more information \ref Pmic_CommonCtrlStat_t
 *
 * \param   pPmicCoreHandle  [IN]       PMIC Interface Handle.
 * \param   pCommonCtrlStat  [IN/OUT]   Pointer to store PMIC required common
 *                                      control parameter status.
 *
 * \return  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_getCommonCtrlStat(Pmic_CoreHandle_t           *pPmicCoreHandle,
                               Pmic_CommonCtrlStat_t       *pCommonCtrlStat);

/*!
 * \brief   API to get configured value for I2C1 or I2C2 Speed based on commMode
 *
 * Requirement: REQ_TAG(PDK-9129)
 * Design: did_pmic_i2c_speed_readback
 * Architecture: aid_pmic_core_misc_cfg
 *
 *          This function is used to get the configured value for I2C1 or
 *          I2C2 Speed based on commMode
 *
 * \param   pPmicCoreHandle [IN]    PMIC Interface Handle.
 * \param   pI2C1Speed      [OUT]   Pointer to store I2C1 Speed for both
 *                                  PMIC_INTF_SINGLE_I2C and PMIC_INTF_DUAL_I2C
 *                                  interface
 *                                     Valid Vaues \ref Pmic_I2CSpeedSel
 * \param   pI2C2Speed      [OUT]   Pointer to store I2C2 Speed for both
 *                                  PMIC_INTF_DUAL_I2C interface only
 *                                     Valid Vaues \ref Pmic_I2CSpeedSel
 *
 * \return  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_getI2CSpeed(Pmic_CoreHandle_t     *pPmicCoreHandle,
                         uint8_t               *pI2C1Speed,
                         uint8_t               *pI2C2Speed);

/*!
 * \brief   API to Enable CRC
 *
 * Requirement: REQ_TAG(PDK-9119)
 * Design: did_pmic_crc_enable
 * Architecture: aid_pmic_core_misc_cfg
 *
 *          This function is used to enable CRC on Primary PMIC which enables
 *          CRC for I2C1, I2C2, SPI interface of both Primary and Secondary PMIC
 *          Note: Application shall not do reads and writes of the any
 *          PMIC registers for at least 2ms to allow the recalculation of the
 *          register CRC value due to the change
 *          Valid only for TPS6594x Leo PMIC PG2.0 and LP8764x Hera PMIC PG2.0
 *
 * \param   pPmicCoreHandle [IN]    PMIC Interface Handle.
 *
 * \return  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_enableCRC(Pmic_CoreHandle_t     *pPmicCoreHandle);

/*!
 * \brief   API to get CRC Status
 *
 * Requirement: REQ_TAG(PDK-9329)
 * Design: did_pmic_crc_status
 * Architecture: aid_pmic_core_misc_cfg
 *
 *          This function is used to get the CRC Status based on commMode
 *
 * \param   pPmicCoreHandle      [IN]    PMIC Interface Handle.
 * \param   pI2c1SpiCrcStatus    [OUT]   Pointer to store CRC Status for I2C1/
 *                                       SPI interface
 *                                       Valid Vaues \ref Pmic_CrcStatus
 * \param   pI2c2CrcStatus       [OUT]   Pointer to store CRC Status for I2C2
 *                                       interface
 *                                       Valid Vaues \ref Pmic_CrcStatus
 *
 * \return  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_getCrcStatus(Pmic_CoreHandle_t     *pPmicCoreHandle,
                          uint8_t               *pI2c1SpiCrcStatus,
                          uint8_t               *pI2c2CrcStatus);

/*!
 * \brief   API to get PMIC Device Information
 *
 * Requirement: REQ_TAG(PDK-9109), REQ_TAG(PDK-9110), REQ_TAG(PDK-9149),
 *              REQ_TAG(PDK-9159), REQ_TAG(PDK-9329)
 * Design: did_pmic_dev_info_readback
 * Architecture: aid_pmic_core_misc_cfg
 *
 *          This function is used to get PMIC Device Information such as
 *          TI DeviceID, TI NVM ID, TI NVM Revision, TI Silicon Revision,
 *          Custom NVM ID
 *          Note: customNvmID is valid only for TPS6594x Leo PMIC PG2.0 and
 *          LP8764x Hera PMIC PG2.0
 *
 * \param   pPmicCoreHandle      [IN]    PMIC Interface Handle.
 * \param   pDeviceInfo          [OUT]   PMIC Device Information value
 *
 * \return  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_getDeviceInfo(Pmic_CoreHandle_t     *pPmicCoreHandle,
                           Pmic_DeviceInfo_t     *pDeviceInfo);

/*!
 * \brief   API to set I2C1 or I2C2 Speed based on commMode
 *
 * Requirement: REQ_TAG(PDK-9129)
 * Design: did_pmic_i2c_speed_readback
 * Architecture: aid_pmic_core_misc_cfg
 *
 *          This function is used to configure I2C1 speed for Single or Dual I2C
 *          Interface and I2C2 Speed for Dual I2C Interface based on commMode.
 *
 *          Note: I2C Master before switching the I2C speed to HS/Standard Mode,
 *          I2C Master has to configure I2C1/I2C2 speed accordingly then only
 *          I2C Master can communicate with PMIC in HS/Standard Mode
 *
 * \param   pPmicCoreHandle [IN]    PMIC Interface Handle.
 *
 * \return  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_setI2CSpeedCfg(Pmic_CoreHandle_t     *pPmicCoreHandle);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* PMIC_CORE_H_ */

/* @} */
