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
 *  @file pmic_core.h
 *
 *  @brief PMIC Driver Common API/interface file.
 */

#ifndef PMIC_CORE_H_
#define PMIC_CORE_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "pmic_core_tps65386x.h"
#include "pmic_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */
/**
 * @brief  PMIC driver Core Handle INIT status Magic Number.
 *         Used to validate Handle to avoid corrupted PmicHandle usage.
 *         on Success: (DRV_INIT_SUCCESS | Pmic_InstType_t)
 */
#define DRV_INIT_SUCCESS (0xABCD0000U)

#define PMIC_SILICON_REV_ID_PG_2_0 (0x08U)

#define PMIC_SILICON_REV_ID_PG_1_0 (0x0U)

/**
 *  @anchor Pmic_ScratchPad_Sel
 *  @name   PMIC Scratchpad register selection
 *
 *  @{
 */
#define PMIC_SCRATCH_PAD_REG_1 (0x0U)
#define PMIC_SCRATCH_PAD_REG_2 (0x1U)
/*  @} */

/**
 *  @anchor Pmic_CommonCtrlStatValidParamCfg
 *  @name   PMIC Common control param status Structure Param Bits
 *
 *  PMIC valid params configuration type for
 *  the structure member validParams of Pmic_CommonCtrlStat_t
 *  structure
 *
 *  @{
 */
/** @brief validParams value used to get Register lock status */
#define PMIC_CFG_REGISTER_LOCK_STAT_VALID (1U)
/** @brief validParams value used to get External clock validity status */
#define PMIC_CFG_EXT_CLK_VALIDITY_STAT_VALID (2U)
/** @brief validParams value used to get Startup(nPWRON/Enable) pin status */
#define PMIC_CFG_STARTUP_PIN_STAT_VALID (3U)
/** @brief validParams value used to get nRSTOUT_SOC Pin status */
#define PMIC_CFG_NRSTOUTSOC_PIN_STAT_VALID (5U)
/** @brief validParams value used to get nRSTOUT Pin status */
#define PMIC_CFG_NRSTOUT_PIN_STAT_VALID (6U)
/** @brief validParams value used to get nINT Pin status */
#define PMIC_CFG_NINT_PIN_STAT_VALID (7U)
/** @brief validParams value used to get SPMI Low Power Mode status */
#define PMIC_CFG_SPMI_LPM_STAT_VALID (8U)
/** @brief validParams value used to get status of ENABLE_DRV Configuration by
 *         I2C/SPI */
#define PMIC_CFG_FORCE_ENABLE_DRV_LOW_STAT_VALID (9U)
/*  @} */

#define PMIC_CFG_EN_OUT_PIN_STAT_VALID (0U)
/** @brief validParams value used to get SAFE_OUT1 Pin status */
#define PMIC_CFG_SAFE_OUT1_PIN_STAT_VALID (1U)
/** @brief validParams value used to get NRST Pin status */
#define PMIC_CFG_NRST_PIN_STAT_VALID (2U)

/**
 *  @anchor Pmic_CommonCtrlStatValidParamBitShiftValues
 *  @name   PMIC Common Control param status valid param bit shift values
 *
 *  Application can use below shifted values to set the validParam
 *  member defined in Pmic_CommonCtrlStat_t structure
 *
 *  @{
 */
#define PMIC_CFG_BB_EOC_INDICATION_STAT_VALID_SHIFT                            \
    (1U << PMIC_CFG_BB_EOC_INDICATION_STAT_VALID)
#define PMIC_CFG_REGISTER_LOCK_STAT_VALID_SHIFT                                \
    (1U << PMIC_CFG_REGISTER_LOCK_STAT_VALID)
#define PMIC_CFG_EXT_CLK_VALIDITY_STAT_VALID_SHIFT                             \
    (1U << PMIC_CFG_EXT_CLK_VALIDITY_STAT_VALID)
#define PMIC_CFG_STARTUP_PIN_STAT_VALID_SHIFT                                  \
    (1U << PMIC_CFG_STARTUP_PIN_STAT_VALID)
#define PMIC_CFG_EN_DRV_PIN_STAT_VALID_SHIFT                                   \
    (1U << PMIC_CFG_EN_DRV_PIN_STAT_VALID)
#define PMIC_CFG_NRSTOUTSOC_PIN_STAT_VALID_SHIFT                               \
    (1U << PMIC_CFG_NRSTOUTSOC_PIN_STAT_VALID)
#define PMIC_CFG_NRSTOUT_PIN_STAT_VALID_SHIFT                                  \
    (1U << PMIC_CFG_NRSTOUT_PIN_STAT_VALID)
#define PMIC_CFG_NINT_PIN_STAT_VALID_SHIFT (1U << PMIC_CFG_NINT_PIN_STAT_VALID)
#define PMIC_CFG_SPMI_LPM_STAT_VALID_SHIFT (1U << PMIC_CFG_SPMI_LPM_STAT_VALID)
#define PMIC_CFG_FORCE_ENABLE_DRV_LOW_STAT_VALID_SHIFT                         \
    (1U << PMIC_CFG_FORCE_ENABLE_DRV_LOW_STAT_VALID)
/*  @} */

/**
 *  @anchor Pmic_SignalLvl
 *  @name   PMIC signal level of the STARTUP(NPWRON/ENABLE)/ NINT / NRSTOUT_SOC/
 *          NRSTOUT/ EN_DRV Pin
 *
 *  @{
 */
#define PMIC_PIN_SIGNAL_LEVEL_LOW (0U)
#define PMIC_PIN_SIGNAL_LEVEL_HIGH (1U)
/*  @} */

/**
 *  @anchor Pmic_PinType_Sel
 *  @name   PMIC Pin Type Selection as NRSTOUT_SOC/ NRSTOUT/ EN_DRV
 *
 *  @{
 */
#define PMIC_PIN_TYPE_EN_DRV (0U)
#define PMIC_PIN_TYPE_NRSTOUT_SOC (1U)
#define PMIC_PIN_TYPE_NRSTOUT (2U)
/*  @} */

#define PMIC_PIN_TYPE_NRST_RDBK_LVL (0U)
#define PMIC_PIN_TYPE_SAFE_OUT1_RDBK_LVL (1U)
#define PMIC_PIN_TYPE_EN_OUT_RDBK_LVL (2U)
#define PMIC_PIN_TYPE_GPO1_RDBK_LVL (3U)
#define PMIC_PIN_TYPE_GPO2_RDBK_LVL (4U)
#define PMIC_PIN_TYPE_GPO3_RDBK_LVL (5U)
#define PMIC_PIN_TYPE_GPO4_RDBK_LVL (6U)

/**
 *  @anchor Pmic_EnableDrvI2CSPICfg_Stat
 *  @name   PMIC ENABLE_DRV I2C/SPI configuration status
 *
 *  @{
 */
#define PMIC_ENABLE_DRV_I2C_SPI_CONFIG_ENABLE (0U)
#define PMIC_ENABLE_DRV_I2C_SPI_CONFIG_DISABLE (1U)
/*  @} */

/**
 *  @anchor Pmic_RegisterLock_Config
 *  @name   PMIC Register Lock Configuration
 *
 *  @{
 */

/** @brief Unlock PMIC Registers */
#define PMIC_REGISTER_UNLOCK_DATA1 (0x98U)
#define PMIC_REGISTER_UNLOCK_DATA2 (0xB8U)
#define PMIC_TMR_COUNTER_UNLOCK_DATA1 (0x13U)
#define PMIC_TMR_COUNTER_UNLOCK_DATA2 (0x7DU)

/** @brief  Lock PMIC Registers - Write any value other than.
 *          Here 0x10 is used  */
#define PMIC_REGISTER_LOCK_1 (0x10U)
#define PMIC_REGISTER_LOCK_2 (0x10U)
/*  @} */

/**
 *  @anchor Pmic_RegisterLock_Stat
 *  @name   PMIC Register Lock Status
 *
 *  @{
 */
#define PMIC_REGISTER_STATUS_UNLOCK (0x0U)
#define PMIC_REGISTER_STATUS_LOCK (0x1U)
/*  @} */

/**
 *  @anchor Pmic_SpreadSpectrum_Cfg
 *  @name   PMIC Spread Spectrum Configuration
 *
 *  @{
 */
/** @brief Disable Spread Spectrum Configuration */
#define PMIC_SPREAD_SPECTRUM_CFG_DISABLE 0U
/** @brief Enable Spread Spectrum Configuration */
#define PMIC_SPREAD_SPECTRUM_CFG_ENABLE 1U

/*  @} */

/**
 *  @anchor Pmic_SpreadSpectrum_Mod_Depth_Sel
 *  @name   PMIC Spread Spectrum Modulation Depth Sel
 *
 *  @{
 */
/** @brief No Modulation */
#define PMIC_SPREAD_SPECTRUM_MODULATION_DEPTH_NONE (0U)
/** @brief  Modulation Depth as +/- 6.3%  */
#define PMIC_SPREAD_SPECTRUM_MODULATION_DEPTH_6_3_PERCENT (1U)
/** @brief  Modulation Depth as +/- 8.4% */
#define PMIC_SPREAD_SPECTRUM_MODULATION_DEPTH_8_4_PERCENT (2U)
/*  @} */

/**
 *  @anchor Pmic_SpmiLpmModeCtrl_Stat
 *  @name   PMIC SPMI LPM Mode Control Status
 *
 *  @{
 */
/** @brief SPMI LPM Mode Control Configuration is Disabled */
#define PMIC_SPMI_LPM_MODE_CTRL_CFG_DISABLED (0U)
/** @brief SPMI LPM Mode Control Configuration is Enabled */
#define PMIC_SPMI_LPM_MODE_CTRL_CFG_ENABLED (1U)
/*  @} */

/**
 *  @anchor Pmic_InternalClkMonitor_Cfg
 *  @name   PMIC Internal Clock Monitoring Configuration
 *
 *  @{
 */
/** @brief Disable Internal Clock Monitoring */
#define PMIC_INTERNAL_CLK_MONITORING_CFG_DISABLE (0U)
/** @brief Enable Internal Clock Monitoring */
#define PMIC_INTERNAL_CLK_MONITORING_CFG_ENABLE (1U)
/*  @} */

/**
 *  @anchor Pmic_SyncClkOut_Freq_Sel
 *  @name   PMIC SYNCCLKOUT Frequency selection
 *
 *  @{
 */
/** @brief DISABLE SYNCCLKOUT */
#define PMIC_SYNCCLKOUT_DISABLE (0U)
/** @brief  SYNCCLKOUT Frequency as 1.1 MHz */
#define PMIC_SYNCCLKOUT_1_1_MHZ (1U)
/** @brief  SYNCCLKOUT Frequency as 2.2 MHz */
#define PMIC_SYNCCLKOUT_2_2_MHZ (2U)
/** @brief  SYNCCLKOUT Frequency as 4.4 MHz */
#define PMIC_SYNCCLKOUT_4_4_MHZ (3U)
/*  @} */

/**
 *  @anchor Pmic_ExtClk_Sel
 *  @name   PMIC External Clock selection
 *
 *  @{
 */
/** @brief Forced to Internal RC Oscillator */
#define PMIC_INTERNAL_RC_OSC (0U)
/** @brief  Selects External clock when Available  */
#define PMIC_AUTOMATIC_EXT_CLK (1U)
/*  @} */

/**
 *  @anchor Pmic_ExtClkValidStat
 *  @name   PMIC External Clock Validity Status
 *
 *  @{
 */
#define PMIC_EXT_CLK_STATUS_VALID (0U)
#define PMIC_EXT_CLK_STATUS_INVALID (1U)
/*  @} */

/**
 *  @anchor Pmic_CrcStatus
 *  @name   PMIC CRC Status for I2C1/I2C2/SPI
 *
 *  @{
 */
/** @brief  CRC is disabled  */
#define PMIC_CRC_STATUS_DISABLED (0U)
/** @brief  CRC is enabled  */
#define PMIC_CRC_STATUS_ENABLED (1U)
/*  @} */

/**
 *  @anchor Pmic_CommonCtrlValidParamCfg
 *  @name   PMIC Common Control Configuration Structure Param Bits
 *
 *  PMIC valid params configuration type for
 *  the structure member validParams of Pmic_CommonCtrlCfg_t
 *  structure
 *
 *  @{
 */
/** @brief validParams value used to set/get Spread Spectrum
 *         Enable/Disable */
#define PMIC_CFG_SPREAD_SPECTRUM_EN_VALID (1U)
/** \brief validParams value used to set/get ENABLE_DRV pin
 *         configuration */
#define PMIC_CFG_ENABLE_SAFEOUT_VALID (1U)
/** \brief validParams value used to set/get Register Lock
 *         configuration */
#define PMIC_CFG_REG_LOCK_VALID (2U)
/** \brief validParams value used to set/get Spread Spectrum
 *         modulation Depth Value */
#define PMIC_CFG_SPREAD_SPECTRUM_DEPTH_VALID (3U)
/** \brief validParams value used to set/get to Enable/Disable load
 *         from EEPROM defaults on RTC Domain(TPS6594X Leo)/ conf(LP8764x Hera)
 *         registers */
#define PMIC_CFG_EEPROM_DEFAULT_VALID (4U)
/** \brief validParams value used to set/get to Enable/Disable to
 *         skip EEPROM defaults load on conf and Other registers
 *         Valid only for LP8764x Hera*/
#define PMIC_CFG_SKIP_EEPROM_LOAD_VALID (5U)
/*  @} */

/**
 *  @anchor Pmic_CommonCtrlValidParamBitShiftValues
 *  @name   PMIC Common Control Configuration valid param bit shift values
 *
 *  Application can use below shifted values to set the validParam
 *  member defined in Pmic_CommonCtrlCfg_t structure
 *
 *  @{
 */
#define PMIC_CFG_SPREAD_SPECTRUM_EN_VALID_SHIFT                                \
    (1U << PMIC_CFG_SPREAD_SPECTRUM_EN_VALID)
#define PMIC_CFG_ENABLE_DRV_VALID_SHIFT (1U << PMIC_CFG_ENABLE_DRV_VALID)
#define PMIC_CFG_REG_LOCK_VALID_SHIFT (1U << PMIC_CFG_REG_LOCK_VALID)
#define PMIC_CFG_SPREAD_SPECTRUM_DEPTH_VALID_SHIFT                             \
    (1U << PMIC_CFG_SPREAD_SPECTRUM_DEPTH_VALID)
#define PMIC_CFG_EEPROM_DEFAULT_VALID_SHIFT                                    \
    (1U << PMIC_CFG_EEPROM_DEFAULT_VALID)
#define PMIC_CFG_SKIP_EEPROM_LOAD_VALID_SHIFT                                  \
    (1U << PMIC_CFG_SKIP_EEPROM_LOAD_VALID)
/*  @} */

/**
 *  @anchor Pmic_MiscCtrlValidParamCfg
 *  @name   PMIC Miscellaneous Control Configuration Structure Param Bits
 *
 *  PMIC valid params configuration type for
 *  the structure member validParams of Pmic_MiscCtrlCfg_t
 *  structure
 *
 *  @{
 */
/** @brief validParams value used to set/get to Enable/Disable Band gap
 *          Voltage to AMUX OUT/REF OUT Pin */
#define PMIC_CFG_AMUX_OUT_REF_OUT_EN_VALID (0U)

#define PMIC_CFG_AMUX_DMUX_EN_VALID (6U)
/** @brief validParams value used to set/get to Enable or Disable internal
 *         Clock Monitoring */
#define PMIC_CFG_CLK_MON_EN_VALID (1U)
/** @brief validParams value used to set/get to Select SYNCCLKOUT Frequency */
#define PMIC_CFG_SYNC_CLK_OUT_FREQ_SEL_VALID (2U)
/** @brief validParams value used to set/get External clock Selection */
#define PMIC_CFG_EXT_CLK_SEL_VALID (3U)
/** @brief validParams value used to set/get to Select External clock Frequency
 */
#define PMIC_CFG_SYNC_CLK_IN_FREQ_VALID (4U)
/** @brief validParams value used to set/get to NRSTOUT_SOC Signal
 */
#define PMIC_CFG_NRSTOUT_SOC_VALID (5U)
/** @brief validParams value used to set/get NRSTOUT Signal
 */
#define PMIC_CFG_NRSTOUT_VALID (6U)
/*  @} */

/**
 *  @anchor Pmic_MiscCtrlValidParamBitShiftValues
 *  @name   PMIC Miscellaneous Control Configuration valid param bit shift
 *          values
 *
 *  Application can use below shifted values to set the validParam
 *  member defined in Pmic_MiscCtrlCfg_t structure
 *
 *  @{
 */
#define PMIC_CFG_AMUX_OUT_REF_OUT_EN_VALID_SHIFT                               \
    (1U << PMIC_CFG_AMUX_OUT_REF_OUT_EN_VALID)
#define PMIC_CFG_CLK_MON_EN_VALID_SHIFT (1U << PMIC_CFG_CLK_MON_EN_VALID)
#define PMIC_CFG_SYNC_CLK_OUT_FREQ_SEL_VALID_SHIFT                             \
    (1U << PMIC_CFG_SYNC_CLK_OUT_FREQ_SEL_VALID)
#define PMIC_CFG_EXT_CLK_SEL_VALID_SHIFT (1U << PMIC_CFG_EXT_CLK_SEL_VALID)
#define PMIC_CFG_SYNC_CLK_IN_FREQ_VALID_SHIFT                                  \
    (1U << PMIC_CFG_SYNC_CLK_IN_FREQ_VALID)
#define PMIC_CFG_NRSTOUT_SOC_VALID_SHIFT (1U << PMIC_CFG_NRSTOUT_SOC_VALID)
#define PMIC_CFG_NRSTOUT_VALID_SHIFT (1U << PMIC_CFG_NRSTOUT_VALID)
/*  @} */

/**
 *  @anchor Pmic_BatteryCtrlValidParamCfg
 *  @name   PMIC Backup Battery Control Configuration Structure Param Bits
 *
 *  PMIC valid params configuration type for
 *  the structure member validParams of Pmic_BatteryCtrlCfg_t
 *  structure
 *
 *  @{
 */
/** @brief validParams value used to set/get to Enable/Disable Backup Battery
 *         Charging */
#define PMIC_CFG_CHARGING_EN_VALID (0U)
/** @brief validParams value used to set/get to Backup Battery configuration for
 *         End of charge Voltage */
#define PMIC_CFG_END_OF_CHARGE_VOLTAGE_VALID (1U)
/** @brief validParams value used to set/get to Backup Battery charging current
 *         value */
#define PMIC_CFG_CHARGE_CURRENT_VALID (2U)
/*  @} */

/**
 *  @anchor Pmic_BatteryCtrlValidParamBitShiftValues
 *  @name   PMIC Backup Battery Control Configuration valid param bit shift
 *          values
 *
 *  Application can use below shifted values to set the validParam
 *  member defined in Pmic_BatteryCtrlCfg_t structure
 *
 *  @{
 */
#define PMIC_CFG_CHARGING_EN_VALID_SHIFT (1U << PMIC_CFG_CHARGING_EN_VALID)
#define PMIC_CFG_END_OF_CHARGE_VOLTAGE_VALID_SHIFT                             \
    (1U << PMIC_CFG_END_OF_CHARGE_VOLTAGE_VALID)
#define PMIC_CFG_CHARGE_CURRENT_VALID_SHIFT                                    \
    (1U << PMIC_CFG_CHARGE_CURRENT_VALID)
/*  @} */

/**
 * @brief   PMIC SAFE STATE OUT CONFIG CONTROL MACROS
 */
#define PMIC_SAFE_OUT1_ENABLE_LOW (0x00U)
#define PMIC_SAFE_OUT1_ENABLE_HIGH (0x01U)

#define PMIC_SAFE_OUT2_ENABLE_LOW (0x00U)
#define PMIC_SAFE_OUT2_ENABLE_FOLLOW_CFG (0x01U)

#define PMIC_SAFE_OUT2_CFG_INV_LEVEL (0x00U)
#define PMIC_SAFE_OUT2_CFG_MATCH_LEVEL (0x01U)
#define PMIC_SAFE_OUT2_CFG_PWM_HEARTBEAT_LOW (0x02U)
#define PMIC_SAFE_OUT2_CFG_PWM_HEARTBEAT_SECOND (0x03U)
#define PMIC_SAFE_OUT2_CFG_DELAYED_CONTINUOUS (0x04U)
#define PMIC_SAFE_OUT2_CFG_DELAYED_PULSE (0x05U)

/**
 * @brief   PMIC SAFE STATE OUT2 CONFIG-1 MACROS
 */
#define PMIC_SAFE_OUT2_PRESCALE_4US (0x00U)
#define PMIC_SAFE_OUT2_PRESCALE_16US (0x01U)
#define PMIC_SAFE_OUT2_PRESCALE_64US (0x02U)
#define PMIC_SAFE_OUT2_PRESCALE_256US (0x03U)
#define PMIC_SAFE_OUT2_PRESCALE_1MS (0x04U)
#define PMIC_SAFE_OUT2_PRESCALE_4MS (0x05U)
#define PMIC_SAFE_OUT2_PRESCALE_16MS (0x06U)
#define PMIC_SAFE_OUT2_PRESCALE_65MS (0x07U)

/*==========================================================================*/
/*                         Structures and Enums                             */
/*==========================================================================*/
/**
 * @brief  PMIC Recovery Counter Configuration
 *         Note: validParams is input param for all Set and Get APIs. other
 *         params except validParams is input param for Set APIS and output
 *         param for Get APIs
 *
 * @param   validParams   Selection of structure parameters to be set,
 *                        from the combination of @ref Pmic_RecoveryCntCfgType
 *                        and the corresponding member value must be updated.
 *                        Valid values @ref Pmic_RecoveryCntCfgType
 *  @param  thrVal        Recovery Counter Threshold Value.
 *                         Valid only when PMIC_CFG_RECOV_CNT_THR_VAL_VALID
 *                         bit is set.
 *  @param  clrCnt        Clear Recovery Counter Value and value should be 1U.
 *                         Valid only when PMIC_CFG_RECOV_CNT_CLR_VAL_VALID
 *                         bit is set.
 */
typedef struct Pmic_RecovCntCfg_s {
    uint8_t validParams;
    uint8_t thrVal;
    bool clrCnt;
} Pmic_RecovCntCfg_t;

/**
 * @brief  PMIC common control param configuration
 *         Note: validParams is input param for all Set and Get APIs. other
 *         params except validParams is input param for Set APIs and output
 *         param for Get APIs
 *
 * @param   validParams             Selection of structure parameters to be set,
 *                                  from the combination of
 *                                  @ref Pmic_CommonCtrlValidParamCfg
 *                                  and the corresponding member value must be
 *                                  updated
 *                                    Valid values
 *                                        @ref Pmic_CommonCtrlValidParamCfg
 *  @param  sreadSpectrumEn         Spread Spectrum Enable Value
 *                                  Valid only when
 *                                  PMIC_CFG_SPREAD_SPECTRUM_EN_VALID bit is set
 *                                    Valid values @ref Pmic_SpreadSpectrum_Cfg
 *  @param  skipEepromDefaultLoadEn Enable/Disable to skip EEPROM defaults load
 *                                  on conf registers when device transition
 *                                  from  Lpstandby to INIT state
 *                                  Valid only for LP8764x Hera Device
 *                                  Valid only when
 *                                  PMIC_CFG_SKIP_EEPROM_LOAD_VALID
 *                                  bit is set.
 *                                    Valid values
 *                                    @ref Pmic_Lp8764xHera_Skip_EepromDef_LdCfg
 *  @param  eepromDefaultLoad       Load/Not Loaded from EEPROM defaults on
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
 *                                         @ref Pmic_Tps6594xLeo_EepromDef_LdCfg
 *                                           (for TPS6594x Leo Device)
 *                                         @ref Pmic_Lp8764xHera_EepromDef_LdCfg
 *                                           (for LP8764x Hera Device)
 *  @param  enDrv                   Control of ENABLE_DRV pin. Can be configured
 *                                  only When forceEnDrvLow set to 0 else
 *                                  ENABLE_DRV pin is set to
 *                                   PMIC_PIN_SIGNAL_LEVEL_LOW.
 *                                     Valid values @ref Pmic_SignalLvl
 *                                  Valid only when PMIC_CFG_ENABLE_DRV_VALID
 *                                  bit is set.
 *  @param  regLock                 Register Lock configuration
 *                                  Valid values @ref Pmic_RegisterLock_Config
 *                                  Valid only when PMIC_CFG_REG_LOCK_VALID
 *                                  bit is set
 *                                  Valid only for Pmic_setCommonCtrlConfig API
 *  @param  spreadSpectrumDepth     Spread Spectrum modulation Depth Value
 *                                     Valid values
 *                                        @ref Pmic_SpreadSpectrum_Mod_Depth_Sel
 *                                  Valid only when
 *                                  PMIC_CFG_SPREAD_SPECTRUM_DEPTH_VALID
 *                                  bit is set.
 */
typedef struct Pmic_CommonCtrlCfg_s {
    uint8_t validParams;
    bool sreadSpectrumEn;
    bool skipEepromDefaultLoadEn;
    uint8_t eepromDefaultLoad;
    uint8_t enDrv;
    uint8_t regLock;
    uint8_t spreadSpectrumDepth;
    uint8_t eNsafeOut1;
    uint8_t eNsafeOut2;
    uint8_t regLock_1;
    uint8_t regLock_2;
    uint8_t cntLock_1;
    uint8_t cntLock_2;
} Pmic_CommonCtrlCfg_t;

/**
 * @brief Structure to hold PMIC safe state configuration.
 *
 * This structure contains parameters related to PMIC safe state configuration.
 */
typedef struct Pmic_SafeStateCfg_s {
    uint8_t validParams;       /**< Indicates which parameters are valid */
    uint8_t safeStateTMO;      /**< Safe state timeout */
    uint8_t safeLockThreshold; /**< Safe state lock threshold */
} Pmic_SafeStateCfg_t;

/**
 * @brief  PMIC Miscellaneous control param Configuration
 *         Note: validParams is input param for all Set and Get APIs. other
 *         params except validParams is input param for Set APIs and output
 *         param for Get APIs
 *
 * @param   validParams         Selection of structure parameters to be set,
 *                              from the combination of
 *                              @ref Pmic_MiscCtrlValidParamCfg and the
 *                              corresponding member value must be updated.
 *                                 Valid values @ref Pmic_MiscCtrlValidParamCfg
 *  @param  amuxOutRefOutEn     Enable/Disable Band gap Voltage to AMUX OUT Pin
 *                              (for TPS6594x Leo Device) or RFF OUT Pin
 *                              (for LP8764x Leo Device)
 *                              Valid only when
 *                              PMIC_CFG_AMUX_OUT_REF_OUT_EN_VALID bit is set
 *                                 Valid values
 *                                     @ref Pmic_Tps6594xLeo_AMuxOutPinCtrl_Cfg
 *                                      (for TPS6594x Leo Device)
 *                                     @ref Pmic_Lp8764xHera_RefOutPinCtrl_Cfg
 *                                      (for LP8764x Hera Device)
 *  @param  clkMonEn            Enable or Disable internal Clock Monitoring
 *                              Valid only when PMIC_CFG_CLK_MON_EN_VALID
 *                              bit is set.
 *                                 Valid values @ref Pmic_InternalClkMonitor_Cfg
 *  @param  syncClkOutFreqSel   Selects SYNCCLKOUT Frequency
 *                                 Valid values @ref Pmic_SyncClkOut_Freq_Sel
 *                              Valid only when
 *                              PMIC_CFG_SYNC_CLK_OUT_FREQ_SEL_VALID bit is set
 *  @param  extClkSel           External clock Selection
 *                                 Valid values @ref Pmic_ExtClk_Sel
 *                              Valid only when PMIC_CFG_EXT_CLK_SEL_VALID
 *                              bit is set.
 *  @param  syncClkInFreq       Selects External clock Frequency
 *                              Valid only when PMIC_CFG_SYNC_CLK_IN_FREQ_VALID
 *                              bit is set.
 *                                 Valid values
 *                                     @ref Pmic_Tps6594xLeo_ExtClk_Freq_Sel
 *                                      (for TPS6594x Leo Device)
 *                                     @ref Pmic_Lp8764xHera_ExtClk_Freq_Sel
 *                                      (for LP8764x Hera Device)
 *  @param  nRstOutSocSignal    Configure NRSTOUT_SOC Signal
 *                              Note: When Application configures
 *                              nRstOutSocSignal as PMIC_PIN_SIGNAL_LEVEL_LOW
 *                              then SOC will be in reset
 *                              Valid only when PMIC_CFG_NRSTOUT_SOC_VALID
 *                              bit is set.
 *                                 Valid values @ref Pmic_SignalLvl
 *  @param  nRstOutSignal       Configure NRSTOUT Signal
 *                              Note: When Application configures
 *                              nRstOutSignal as PMIC_PIN_SIGNAL_LEVEL_LOW
 *                              then MCU will be in reset
 *                              Valid only when PMIC_CFG_NRSTOUT_VALID
 *                              bit is set.
 *                                 Valid values @ref Pmic_SignalLvl
 */
typedef struct Pmic_MiscCtrlCfg_s {
    uint8_t validParams;
    bool amuxOutRefOutEn;
    bool clkMonEn;
    uint8_t syncClkOutFreqSel;
    uint8_t extClkSel;
    uint8_t syncClkInFreq;
    uint8_t nRstOutSocSignal;
    uint8_t nRstOutSignal;
} Pmic_MiscCtrlCfg_t;

/**
 * @brief Structure to hold PMIC diagnostic output configuration control
 * information.
 *
 * This structure contains various control parameters related to PMIC diagnostic
 * output configuration.
 */
typedef struct Pmic_DiagOutCfgCtrl_s {
    uint8_t validParams;        /**< Indicates which parameters are valid */
    uint8_t DiagOutCtrl;        /**< Diagnostic output control */
    uint8_t DiagOutCtrl_AMUXEn; /**< Diagnostic output AMUX enable */
    uint8_t DiagOutCtrl_DMUXEn; /**< Diagnostic output DMUX enable */
    uint8_t DiagGrpSel;         /**< Diagnostic group selection */
} Pmic_DiagOutCfgCtrl_t;

/**
 * @brief  PMIC Backup Battery control param Configuration
 *         Note: validParams is input param for all Set and Get APIs. other
 *         params except validParams is input param for Set APIs and output
 *         param for Get APIs
 *
 * @param   validParams         Selection of structure parameters to be set,
 *                              from the combination of
 *                              @ref Pmic_BatteryCtrlValidParamCfg and the
 *                              corresponding member value must be updated.
 *  @param  chargingEn          Enable/Disable Backup Battery Charging
 *                              Valid only when PMIC_CFG_CHARGING_EN_VALID
 *                              bit is set.
 *                              Valid only for TPS6594x Leo Device
 *                                 Valid values
 *                                    @ref Pmic_Tps6594xLeo_BatteryCharging_Cfg
 *  @param  endOfChargeVoltage  Backup Battery configuration for End of charge
 *                              Voltage
 *                              Valid only when
 *                              PMIC_CFG_END_OF_CHARGE_VOLTAGE_VALID bit is set.
 *                              Valid only for TPS6594x Leo Device
 *                                 Valid values
 *                                  @ref Pmic_Tps6594xLeo_EndOfChargeVoltage_Sel
 *  @param  chargeCurrent       Backup Battery charging current value
 *                              Valid only when PMIC_CFG_CHARGE_CURRENT_VALID
 *                              bit is set.
 *                              Valid only for TPS6594x Leo Device
 *                                 Valid values
 *                                    @ref Pmic_Tps6594xLeo_Charging_Current_Sel
 */
typedef struct Pmic_BatteryCtrlCfg_s {
    uint8_t validParams;
    bool chargingEn;
    uint8_t endOfChargeVoltage;
    uint8_t chargeCurrent;
} Pmic_BatteryCtrlCfg_t;

/**
 * @brief  PMIC common control param status
 *         Note: validParams is input param for all Get APIs. other params
 *         except validParams is output param for Get APIs
 *
 * @param   validParams        Selection of structure parameters to be set, from
 *                             the combination of
 *                             @ref Pmic_CommonCtrlStatValidParamCfg
 *                             and the corresponding member value must be
 *                             updated
 *                                Valid values
 *                                         @ref Pmic_CommonCtrlStatValidParamCfg
 *  @param  spmiLpmStat        SPMI Low Power Mode Control Status.
 *                             Valid only when PMIC_CFG_SPMI_LPM_STAT_VALID
 *                             bit is set.
 *                                Valid values @ref Pmic_SpmiLpmModeCtrl_Stat
 *  @param  forceEnDrvLowStat  Status of ENABLE_DRV Configuration by I2C/SPI
 *                             Valid only when
 *                             PMIC_CFG_FORCE_ENABLE_DRV_LOW_STAT_VALID
 *                             bit is set.
 *                                Valid values @ref Pmic_EnableDrvI2CSPICfg_Stat
 *  @param  bbEocIndication    Backup Battery End of charge Indication Status
 *                             Valid only when
 *                             PMIC_CFG_BB_EOC_INDICATION_STAT_VALID
 *                             bit is set
 *                              Valid only for TPS6594x Leo Device
 *                                 Valid values
 *                                     @ref Pmic_Tps6594xLeo_BBEoCIndicationStat
 *  @param  regLockStat        Register lock status
 *                             Valid only when PMIC_CFG_REGISTER_LOCK_STAT_VALID
 *                             bit is set
 *                                 Valid values @ref Pmic_RegisterLock_Stat
 *  @param  extClkValidity     External clock validity status. The status value
 *                             is valid only when External clock is connected
 *                             Ignore the status value when External clock is
 * not connected Valid only when PMIC_CFG_EXT_CLK_VALIDITY_STAT_VALID bit is set
 *                                Valid values @ref Pmic_ExtClkValidStat
 *  @param  startupPin         Startup(nPWRON/Enable) pin status
 *                             Valid only when PMIC_CFG_STARTUP_PIN_STAT_VALID
 *                             bit is set
 *                                Valid values @ref Pmic_SignalLvl
 *  @param  enDrvPin           EN_DRV Pin status
 *                             Valid only when PMIC_CFG_EN_DRV_PIN_STAT_VALID
 *                             bit is set.
 *                                Valid values @ref Pmic_SignalLvl
 *  @param  nRstOutSocPin      nRSTOUT_SOC Pin status
 *                             Valid only when
 *                             PMIC_CFG_NRSTOUTSOC_PIN_STAT_VALID bit is set.
 *                                Valid values @ref Pmic_SignalLvl
 *  @param  nRstOutPin        nRSTOUT Pin status
 *                             Valid only when PMIC_CFG_NRSTOUT_PIN_STAT_VALID
 *                             bit is set.
 *                                Valid values @ref Pmic_SignalLvl
 *  @param  nIntPin           nINT Pin status
 *                             Valid only when PMIC_CFG_NINT_PIN_STAT_VALID
 *                             bit is set.
 *                                Valid values @ref Pmic_SignalLvl
 */
typedef struct Pmic_CommonCtrlStat_s {
    uint32_t validParams;
    bool spmiLpmStat;
    uint8_t forceEnDrvLowStat;
    uint8_t bbEndOfChargeIndication;
    uint8_t regLockStat;
    uint8_t extClkValidity;
    uint8_t startupPin;
    uint8_t enDrvPin;
    uint8_t nRstOutSocPin;
    uint8_t nRstOutPin;
    uint8_t nIntPin;
    uint8_t enOutPin;
    uint8_t safeOut1Pin;
    uint8_t nRstPin;
    uint8_t cfgregLockStat;
    uint8_t cntregLockStat;
} Pmic_CommonCtrlStat_t;

/**
 * @brief  PMIC Device Information
 *
 *  @param  deviceID        TI Device ID Value
 *  @param  nvmID           TI NVM ID Value
 *  @param  nvmRev          TI NVM Revision
 *  @param  siliconRev      TI Silicon Revision
 *  @param  customNvmID     Customer configured NVM ID Value
 *                          customNvmID value is valid only for TPS6594x Leo
 *                          PMIC PG2.0 and LP8764x Hera PMIC PG2.0
 */
typedef struct Pmic_DeviceInfo_s {
    uint8_t deviceID;
    uint8_t nvmID;
    uint8_t nvmRev;
    uint8_t siliconRev;
    uint8_t customNvmID;
} Pmic_DeviceInfo_t;

/**
 * @brief Structure to hold common PMIC state status information.
 *
 * This structure contains pointers to various status bits related to PMIC
 * state.
 */
typedef struct Pmic_CommonStateStat_s {
    uint8_t *rstMcuCnt;    /**< Pointer to RST_MCU_CNT bits (7-6) */
    uint8_t *rstMcuRqFlag; /**< Pointer to RST_MCU_RQ_FLAG bit (5) */
    uint8_t *pwrdDlyActv;  /**< Pointer to PWRD_DLY_ACTV bit (4) */
    uint8_t *state;        /**< Pointer to STATE bits (3-0) */
} Pmic_CommonStateStat_t;

/**
 * @brief Structure to hold common PMIC state control information.
 *
 * This structure contains the state request information for PMIC.
 */
typedef struct Pmic_CommonStateCtrl_s {
    uint8_t state_req; /**< State request value */
} Pmic_CommonStateCtrl_t;

/*==========================================================================*/
/*                         Function Declarations                            */
/*==========================================================================*/

int32_t Pmic_setRecoveryCntCfg(Pmic_CoreHandle_t *pPmicCoreHandle,
                               const Pmic_RecovCntCfg_t recovCntCfg);

int32_t Pmic_getRecoveryCntCfg(Pmic_CoreHandle_t *pPmicCoreHandle,
                               Pmic_RecovCntCfg_t *pRecovCntCfg);

int32_t Pmic_getRecoveryCnt(Pmic_CoreHandle_t *pPmicCoreHandle,
                            uint8_t *pRecovCntVal);

int32_t Pmic_setScratchPadValue(Pmic_CoreHandle_t *pPmicCoreHandle,
                                const uint8_t scratchPadRegNum,
                                const uint8_t data);

int32_t Pmic_getScratchPadValue(Pmic_CoreHandle_t *pPmicCoreHandle,
                                const uint8_t scratchPadRegNum, uint8_t *pData);

int32_t Pmic_setUserSpareValue(Pmic_CoreHandle_t *pPmicCoreHandle,
                               const uint8_t userSpareRegNum,
                               const uint8_t data);

int32_t Pmic_getUserSpareValue(Pmic_CoreHandle_t *pPmicCoreHandle,
                               const uint8_t userSpareRegNum, uint8_t *pData);

int32_t Pmic_setCommonCtrlConfig(Pmic_CoreHandle_t *pPmicCoreHandle,
                                 const Pmic_CommonCtrlCfg_t commonCtrlCfg);

int32_t Pmic_getCommonCtrlConfig(Pmic_CoreHandle_t *pPmicCoreHandle,
                                 Pmic_CommonCtrlCfg_t *pCommonCtrlCfg);

int32_t Pmic_setMiscCtrlConfig(Pmic_CoreHandle_t *pPmicCoreHandle,
                               const Pmic_MiscCtrlCfg_t miscCtrlCfg);

int32_t Pmic_getMiscCtrlConfig(Pmic_CoreHandle_t *pPmicCoreHandle,
                               Pmic_MiscCtrlCfg_t *pMiscCtrlCfg);

int32_t Pmic_setBatteryCtrlConfig(Pmic_CoreHandle_t *pPmicCoreHandle,
                                  const Pmic_BatteryCtrlCfg_t batteryCtrlCfg);

int32_t Pmic_getBatteryCtrlConfig(Pmic_CoreHandle_t *pPmicCoreHandle,
                                  Pmic_BatteryCtrlCfg_t *pBatteryCtrlCfg);

int32_t Pmic_getPinValue(Pmic_CoreHandle_t *pPmicCoreHandle,
                         const uint8_t pinType, uint8_t *pPinValue);

int32_t Pmic_getCommonCtrlStat(Pmic_CoreHandle_t *pPmicCoreHandle,
                               Pmic_CommonCtrlStat_t *pCommonCtrlStat);

int32_t Pmic_getI2CSpeed(Pmic_CoreHandle_t *pPmicCoreHandle,
                         uint8_t *pI2C1Speed, uint8_t *pI2C2Speed);

int32_t Pmic_enableCRC(Pmic_CoreHandle_t *pPmicCoreHandle);

int32_t Pmic_getCrcStatus(Pmic_CoreHandle_t *pPmicCoreHandle,
                          uint8_t *pI2c1SpiCrcStatus, uint8_t *pI2c2CrcStatus);

int32_t Pmic_getDeviceInfo(Pmic_CoreHandle_t *pPmicCoreHandle,
                           Pmic_DeviceInfo_t *pDeviceInfo);

int32_t Pmic_setDiagOutCtrlConfig(Pmic_CoreHandle_t *pPmicCoreHandle,
                                  const Pmic_DiagOutCfgCtrl_t DiagOutCfgCtrl);

int32_t Pmic_getDiagOutCtrlConfig(Pmic_CoreHandle_t *pPmicCoreHandle,
                                  Pmic_DiagOutCfgCtrl_t *pDiagOutCfgCtrl);

int32_t Pmic_spreadSpectrumEnable(Pmic_CoreHandle_t *pPmicCoreHandle,
                                  const Pmic_CommonCtrlCfg_t commonCtrlCfg);

int32_t Pmic_getSpreadSpectrumEnable(Pmic_CoreHandle_t *pPmicCoreHandle,
                                     Pmic_CommonCtrlCfg_t *pCommonCtrlCfg);

int32_t Pmic_getCommonStat(Pmic_CoreHandle_t *pPmicCoreHandle,
                           Pmic_CommonCtrlStat_t *pCommonCtrlStat);

int32_t Pmic_validateDevOnBus(Pmic_CoreHandle_t *pPmicCoreHandle);

int32_t Pmic_setEnableSafeOutCfg(Pmic_CoreHandle_t *pPmicCoreHandle,
                                 const Pmic_CommonCtrlCfg_t commonCtrlCfg);

int32_t Pmic_getSafeOutPinCfg(Pmic_CoreHandle_t *pPmicCoreHandle,
                              Pmic_CommonCtrlCfg_t *pCommonCtrlCfg);

int32_t Pmic_setI2CSpeedCfg(Pmic_CoreHandle_t *pPmicCoreHandle);

int32_t Pmic_setRegisterLockUnlock(Pmic_CoreHandle_t *pPmicCoreHandle,
                                   const Pmic_CommonCtrlCfg_t commonCtrlCfg);

int32_t Pmic_setCounterLockUnlock(Pmic_CoreHandle_t *pPmicCoreHandle,
                                  const Pmic_CommonCtrlCfg_t commonCtrlCfg);

int32_t Pmic_getRegLockStat(Pmic_CoreHandle_t *pPmicCoreHandle,
                            Pmic_CommonCtrlStat_t *pCommonCtrlStat);

int32_t Pmic_getTmrCntLockStat(Pmic_CoreHandle_t *pPmicCoreHandle,
                               Pmic_CommonCtrlStat_t *pCommonCtrlStat);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* PMIC_CORE_H_ */
