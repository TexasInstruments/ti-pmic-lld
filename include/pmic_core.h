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
 * @file pmic_core.h
 *
 * @brief PMIC LLD Core module header file.
 *
 * @details This module contains declarations/definitions of macros, data
 * structures, and APIs used to interact with the PMIC core. Some components of
 * the PMIC core module are as follows: device state transitions, low power mode
 * configuration, getting device recovery counter status, and lock/unlock PMIC
 * registers.
 */
#ifndef __PMIC_CORE_H__
#define __PMIC_CORE_H__

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                Include Files                               */
/* ========================================================================== */
#include "pmic_common.h"

/* ========================================================================== */
/*                              Macros & Typedefs                             */
/* ========================================================================== */

/**
 * @anchor Pmic_CoreLpmCfgValidParams
 * @name TPS65036x PMIC Low Power Mode Configuration Valid Parameters
 *
 * @brief Indication of which parameters are valid within the Pmic_CoreLpmCfg_t
 * struct.
 *
 * @details For more information on the parameters, refer to @ref Pmic_CoreLpmCfg.
 *
 * @note End-user could combine multiple valid parameters using the OR operator.
 *
 * @{
 */
#define PMIC_LPM_PIN_DETECTION_VALID        ((uint32_t)(1U << 0U))
#define PMIC_LPM_DETECTION_DELAY_VALID      ((uint32_t)(1U << 1U))
#define PMIC_LPM_VMON_EN_VALID              ((uint32_t)(1U << 2U))
#define PMIC_LPM_ESM_EN_VALID               ((uint32_t)(1U << 3U))
#define PMIC_LPM_WDG_EN_VALID               ((uint32_t)(1U << 4U))
/** @} */

/**
 * @anchor Pmic_CoreCrc16CfgValidParams
 * @name TPS65036x PMIC CRC16 Configuration Valid Parameters
 *
 * @brief Indication of which parameters are valid within the Pmic_CoreCrc16Cfg_t
 * struct.
 *
 * @details For more information on the parameters, refer to @ref Pmic_CoreCrc16Cfg.
 *
 * @note End-user could combine multiple valid parameters using the OR operator.
 *
 * @{
 */
#define PMIC_CRC16_ENABLE_VALID             ((uint32_t)(1U << 0U))
#define PMIC_CRC16_ACTIVATE_CALC_VALID      ((uint32_t)(1U << 1U))
/** @} */

/**
 * @anchor Pmic_drvInitVal
 * @name PMIC LLD Initialized Value
 *
 * @brief When the drvInitStat struct member of Pmic_CoreHandle_t struct type is
 * equal to this value, the driver has been successfully initialized.
 *
 * @details The value is used to help prevent corrupt PMIC handle usage. When
 * converting to ASCII, the value reads out to be "PMIC".
 *
 * @{
 */
#define PMIC_DRV_INIT_SUCCESS               ((uint32_t)0x504D4943U)
/** @} */

/**
 * @anchor Pmic_scratchPadRegSel
 * @name TPS65036x PMIC Scratch Pad Register Selection
 *
 * @brief Scratch pad register numbers used by the Pmic_setScratchPadVal() PMIC
 * Core API.
 *
 * @{
 */
#define PMIC_SCRATCH_PAD_REG_1              ((uint8_t)0U)
#define PMIC_SCRATCH_PAD_REG_2              ((uint8_t)1U)
#define PMIC_SCRATCH_PAD_REG_3              ((uint8_t)2U)
#define PMIC_SCRATCH_PAD_REG_4              ((uint8_t)3U)
#define PMIC_SCRATCH_PAD_REG_MAX            (PMIC_SCRATCH_PAD_REG_4)
/** @} */

/**
 * @anchor Pmic_fsmCommands
 * @name TPS65036x PMIC FSM Commands
 *
 * @brief Valid FSM commands to be passed into FSM_COMMAND_REG register.
 *
 * @{
 */
#define PMIC_SAFE_RECOVERY_REQUEST          ((uint8_t)0x4BU)
#define PMIC_COLD_BOOT_REQUEST              ((uint8_t)0x55U)
#define PMIC_LOW_POWER_ENTRY_REQUEST        ((uint8_t)0x87U)
#define PMIC_OFF_REQUEST                    ((uint8_t)0x99U)
#define PMIC_LOW_POWER_EXIT_REQUEST         ((uint8_t)0xC5U)
#define PMIC_WARM_RESET_REQUEST             ((uint8_t)0xCCU)
/** @} */

/**
 * @anchor Pmic_LpmPinDetection
 * @name TPS65036x Low Power Mode Pin Detection Values
 *
 * @brief Valid values of the LOWPWR_SEL bit field.
 *
 * @{
 */
/** @brief All interrupts must be cleared for the low power pin to be detected. */
#define PMIC_ALL_IRQ_CLEARED_CONDITION      ((uint8_t)0U)
/** @brief Delay value defined by LOWPWR_DELAY bit must be met. */
#define PMIC_DELAY_VALUE_MET_CONDITION      ((uint8_t)1U)
#define PMIC_PIN_DETECTION_CONDITION_MAX    (PMIC_DELAY_VALUE_MET_CONDITION)
/** @} */

/**
 * @anchor Pmic_detectionDelay
 * @name TPS65036x Low Power Mode Pin Detection Delay Values
 *
 * @brief Valid values of the LOWPWR_DELAY bit field.
 *
 * @{
 */
#define PMIC_DETECTION_DELAY_50_MS          ((uint8_t)0U)
#define PMIC_DETECTION_DELAY_100_MS         ((uint8_t)1U)
#define PMIC_DETECTION_DELAY_250_MS         ((uint8_t)2U)
#define PMIC_DETECTION_DELAY_500_MS         ((uint8_t)3U)
#define PMIC_DETECTION_DELAY_MAX            (PMIC_DETECTION_DELAY_500_MS)
/** @} */

/**
 * @anchor Pmic_regLockUnlockValues
 * @name TPS65036x Register Lock/Unlock Values
 *
 * @brief Values to be passed into the `lock` parameter of the Pmic_setRegLock()
 * API.
 *
 * @{
 */
#define PMIC_LOCK                           ((bool)true)
#define PMIC_UNLOCK                         ((bool)false)
/** @} */

/* ========================================================================== */
/*                             Structures and Enums                           */
/* ========================================================================== */

/**
 * @anchor Pmic_CoreLpmCfg
 * @name TPS65036x Low Power Mode Configuration
 *
 * @brief Struct used to set/get PMIC low power mode (LPM) configurations.
 *
 * @param validParams Each bit in this variable represents whether a struct member
 * is valid. For valid values, refer to @ref Pmic_CoreLpmCfgValidParams.
 *
 * @param pinDetection Low power pin detection configuration. For valid values,
 * @ref Pmic_LpmPinDetection.
 *
 * @param detectionDelay Delay time after nRSTOUT has been activated before low
 * power pin can be recognized. For valid values, refer to @ref Pmic_detectionDelay.
 *
 * @param vmonEn Activation/deactivation of VMONs in LPM state. When set to true
 * (`PMIC_ENABLE`), VMONs are activated in LPM, else VMONs are deactivated in LPM.
 *
 * @param esmEn Activation/deactivation of the ESM in LPM state. When set to true
 * (`PMIC_ENABLE`), the ESM is activated in LPM, else the ESM is deactivated in LPM.
 *
 * @param wdgEn Activation/deactivation of the WDG in LPM state. When set to true
 * (`PMIC_ENABLE`), the WDG is activated in LPM, else the WDG is deactivated in LPM.
 *
 * @{
 */
typedef struct Pmic_CoreLpmCfg_s
{
    uint32_t validParams;

    uint8_t pinDetection;
    uint8_t detectionDelay;

    bool vmonEn;
    bool esmEn;
    bool wdgEn;
} Pmic_CoreLpmCfg_t;
/** @} */

/**
 * @anchor Pmic_CoreCrc16Cfg
 * @name TPS65036x CRC16 Configuration
 *
 * @brief Struct used to set/get PMIC low CRC16 configurations.
 *
 * @param validParams Each bit in this variable represents whether a struct member
 * is valid. For valid values, refer to @ref Pmic_CoreCrc16CfgValidParams.
 *
 * @param enable CRC16 enable/disable configuration. Valid values are `PMIC_ENABLE`
 * and `PMIC_DISABLE`.
 *
 * @param activateCalc CRC16 calculation activation/deactivation configuration.
 * Valid values are `PMIC_ENABLE` and `PMIC_DISABLE`.
 *
 * @{
 */
typedef struct Pmic_CoreCrc16Cfg_s
{
    uint32_t validParams;

    bool enable;
    bool activateCalc;
}Pmic_CoreCrc16Cfg_t;

/* ========================================================================== */
/*                             Function Declarations                          */
/* ========================================================================== */

/**
 * @brief Get PMIC device ID.
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @param devId [OUT] PMIC Device ID.
 *
 * @return Success code if the PMIC device ID has been obtained, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_getDevId(const Pmic_CoreHandle_t *pmicHandle, uint8_t *devId);

/**
 * @brief Get PMIC NVM ID.
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @param nvmId [IN] PMIC NVM ID.
 *
 * @return Success code if the PMIC NVM ID has been obtained, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_getNvmId(const Pmic_CoreHandle_t *pmicHandle, uint8_t *nvmId);

/**
 * @brief Get PMIC NVM revision.
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @param nvmRev [OUT] PMIC NVM revision.
 *
 * @return Success code if the PMIC NVM revision has been obtained, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_getNvmRev(const Pmic_CoreHandle_t *pmicHandle, uint8_t *nvmRev);

/**
 * @brief Get PMIC silicon revision.
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @param siliconRev [OUT] PMIC silicon revision.
 *
 * @return Success code if the PMIC silicon revision has been obtained, error
 * code otherwise. For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_getSiliconRev(const Pmic_CoreHandle_t *pmicHandle, uint8_t *siliconRev);

/**
 * @brief Unlock/lock PMIC registers.
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @param lock [IN] This parameter decides whether the API locks or unlocks PMIC
 * registers. For valid values, refer to @ref Pmic_regLockUnlockValues
 *
 * @return Success code if the register unlock/lock key has been sent to the
 * PMIC, error code otherwise. For valid success/error codes, refer to
 * @ref Pmic_errorCodes.
 */
int32_t Pmic_setRegLock(const Pmic_CoreHandle_t *pmicHandle, bool lock);

/**
 * @brief Unlock PMIC registers.
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @return Success code if the register unlock key has been sent to the PMIC, error
 * code otherwise. For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_unlockRegs(const Pmic_CoreHandle_t *pmicHandle);

/**
 * @brief Lock PMIC registers.
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @return Success code if the register lock key has been sent to the PMIC, error
 * code otherwise. for valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_lockRegs(const Pmic_CoreHandle_t *pmicHandle);

/**
 * @brief Get the PMIC register lock status.
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @param regLockStat [OUT] PMIC register lock status. Value is set to true if PMIC
 * configuration registers are locked, else value is set to false.
 *
 * @return Success code if the PMIC register lock status has been read, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_getRegLock(const Pmic_CoreHandle_t *pmicHandle, bool *regLockStat);

/**
 * @brief Enable or disable PMIC serial communication CRC8.
 *
 * @param pmicHandle [IN/OUT] PMIC interface handle. The crcEnable struct member
 * will be set equal to parameter `crcEnable` upon API call success.
 *
 * @param crc8Enable [IN] CRC8 enable/disable. When set to equal to PMIC_ENABLE,
 * CRC8 will be enabled. Else, CRC8 will be disabled.
 *
 * @return Success code if CRC8 has been enabled or disabled, error code otherwise.
 * For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_enableDisableCRC8(Pmic_CoreHandle_t *pmicHandle, bool crc8Enable);

/**
 * @brief Enable PMIC serial communication CRC8.
 *
 * @param pmicHandle [IN/OUT] PMIC interface handle. The crcEnable struct member
 * will be set to true upon API call success.
 *
 * @return Success code if CRC8 has been enabled, error code otherwise. For
 * valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_enableCRC8(Pmic_CoreHandle_t *pmicHandle);

/**
 * @brief Disable PMIC serial communication CRC8.
 *
 * @param pmicHandle [IN/OUT] PMIC interface handle. The crcEnable struct member
 * will be set to false upon API call success.
 *
 * @return Success code if CRC8 has been disabled, error code otherwise. For
 * valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_disableCRC8(Pmic_CoreHandle_t *pmicHandle);

/**
 * @brief Get the PMIC serial communication CRC8 enable status.
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @param crcEnabled [OUT] CRC8 enable status. True if CRC8 is enabled,
 * otherwise false.
 *
 * @return Success code if CRC8 enable status has been obtained, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_getCRC8Enable(Pmic_CoreHandle_t *pmicHandle, bool *crcEnabled);

/**
 * @brief Send a FSM command to the PMIC.
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @param fsmCmd [IN] MCU command for FSM state transition. For valid values,
 * refer to @ref Pmic_fsmCommands.
 *
 * @return Success code if the FSM command has been sent to the PMIC, error
 * code otherwise. For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_sendFsmCmd(const Pmic_CoreHandle_t *pmicHandle, uint8_t fsmCmd);

/**
 * @brief Turn on/off the power sequence logic for regulators and other
 * components on the PMIC.
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @param pwrOn [IN] Device PWR_ON configuration. When set to PMIC_ENABLE, the
 * device sequence logic is turned on. Else the sequence logic is turned off.
 *
 * @return Success code if the PMIC PWR_ON bit is set, error code otherwise. For
 * valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_setPwrOn(const Pmic_CoreHandle_t *pmicHandle, bool pwrOn);

/**
 * @brief Get the status of the PMIC PWR_ON bit.
 *
 * @details PMIC sequence triggers for components (like regulators) can be
 * associated with the PWR_ON bit via the SEQ_TRIG_X registers, where X is the
 * name of the component.
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @param pwrOnStat [OUT] Device power on status. When set to true, the device
 * sequence logic is turned on. Else the sequence logic is turned off.
 *
 * @return Success code if the device power on status has been obtained, error
 * code otherwise. For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_getPwrOn(const Pmic_CoreHandle_t *pmicHandle, bool *pwrOnStat);

/**
 * @brief Set PMIC low power mode configurations.
 *
 * @details The following options are configurable via this API
 * 1. LPM pin detection option (validParam: PMIC_LPM_PIN_DETECTION_VALID)
 * 2. Delay time after nRSTOUT has been activated before LPM pin is recognized
 * (validParam: PMIC_LPM_DETECTION_DELAY_VALID)
 * 3. VMON activation in LPM mode (validParam: PMIC_LPM_VMON_EN_VALID)
 * 4. ESM activation in LPM mode (validParam: PMIC_LPM_ESM_EN_VALID)
 * 5. WDG activation in LPM mode (validParam: PMIC_LPM_WDG_EN_VALID)
 * For more information on LPM configurations, refer to @ref Pmic_CoreLpmCfg.
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @param lpmCfg [IN] Low power mode configuration to write to PMIC.
 *
 * @return Success code if LPM configurations have been set, error code otherwise.
 * For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_setLpmCfg(const Pmic_CoreHandle_t *pmicHandle, const Pmic_CoreLpmCfg_t *lpmCfg);

/**
 * @brief Get PMIC low power mode configurations. This "get" API supports
 * obtaining the same parameters that are settable through the "Set" API
 * (`Pmic_setLpmCfg`).
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @param lpmCfg [OUT] Low power mode configurations obtained from PMIC.
 *
 * @return Success code if LPM configurations have been obtained, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_getLpmCfg(const Pmic_CoreHandle_t *pmicHandle, Pmic_CoreLpmCfg_t *lpmCfg);

/**
 * @brief Set PMIC CRC16 configurations.
 *
 * @details The following options are configurable via this API
 * 1. enable (validParam: PMIC_CRC16_ENABLE_VALID)
 * 2. activateCalc (validParam: PMIC_CRC16_ACTIVATE_CALC_VALID)
 * For more information on CRC configurations, refer to @ref Pmic_CoreCrc16Cfg.
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @param crc16Cfg [IN] CRC16 configurations to write to PMIC.
 *
 * @attention CRC16 is not yet supported by PMIC LLD. It is recommended to
 * keep CRC16 disabled.
 *
 * @return Success code if CRC16 configurations have been set, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_setCRC16Cfg(const Pmic_CoreHandle_t *pmicHandle, const Pmic_CoreCrc16Cfg_t *crc16Cfg);

/**
 * @brief Get PMIC CRC16 configurations. This "get" API supports obtaining the
 * same parameters that are settable through the "Set" API (`Pmic_setCRC16Cfg`).
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @param crc16Cfg [OUT] CRC16 configurations obtained from PMIC.
 *
 * @return Success code if CRC16 configurations have been obtained, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_getCRC16Cfg(const Pmic_CoreHandle_t *pmicHandle, Pmic_CoreCrc16Cfg_t *crc16Cfg);

/**
 * @brief Trigger run-time ABIST (analog built-in self test) on the PMIC.
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @return Success code if run-time ABIST command has been sent to PMIC, error
 * code otherwise. For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_runABIST(const Pmic_CoreHandle_t *pmicHandle);

/**
 * @brief Get the active status of PMIC ABIST.
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @param isActive [OUT] ABIST status. When the value is true, ABIST is active.
 * Otherwise, ABIST is inactive.
 *
 * @return Success code if the ABIST status has been obtained, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_getABISTStat(const Pmic_CoreHandle_t *pmicHandle, bool *isActive);

/**
 * @brief Write a value to a target scratch pad register on the PMIC.
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @param scratchPadRegNum [IN] Target scratch pad register number.
 *
 * @param value [IN] Desired value to be written to scratch pad register.
 *
 * @return Success code if value has been written to PMIC scratch pad register,
 * error code otherwise. For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_setScratchPadVal(const Pmic_CoreHandle_t *pmicHandle, uint8_t scratchPadRegNum, uint8_t value);

/**
 * @brief Obtain the value of a scratch pad register on the PMIC.
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @param scratchPadRegNum [IN] Target scratch pad register number.
 *
 * @param value [OUT] Scratch pad value obtained from the PMIC.
 *
 * @return Success code if target scratch pad register value has been obtained
 * from the PMIC, error code otherwise. For valid success/error codes, refer to
 * @ref Pmic_errorCodes.
 */
int32_t Pmic_getScratchPadVal(const Pmic_CoreHandle_t *pmicHandle, uint8_t scratchPadRegNum, uint8_t *value);

#ifdef __cplusplus
}

#endif /* __cplusplus */
#endif /* __PMIC_CORE_H__ */
