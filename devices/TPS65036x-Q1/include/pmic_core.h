/******************************************************************************
 * Copyright (c) 2026 Texas Instruments Incorporated - http://www.ti.com
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
#ifndef PMIC_CORE_H
#define PMIC_CORE_H

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
#define PMIC_LPM_PIN_DETECTION_VALID        (1UL << 0U)
#define PMIC_LPM_DETECTION_DELAY_VALID      (1UL << 1U)
#define PMIC_LPM_VMON_EN_VALID              (1UL << 2U)
#define PMIC_LPM_ESM_EN_VALID               (1UL << 3U)
#define PMIC_LPM_WDG_EN_VALID               (1UL << 4U)
#define PMIC_LPM_ENABLE_ALL_VALID           (PMIC_LPM_VMON_EN_VALID | PMIC_LPM_ESM_EN_VALID | PMIC_LPM_WDG_EN_VALID)
/** @} */

/**
 * @anchor Pmic_cfgCrcCalcMode
 * @name TPS65036x PMIC Configuration CRC Calculation Mode
 *
 * @brief Values to be passed into the `calculate` parameter of `Pmic_configCrcEnable()`.
 *
 * @{
 */
#define PMIC_CFG_CRC_RECALCULATE    ((bool)true)
#define PMIC_CFG_CRC_ENABLE_ONLY    ((bool)false)
/** @} */

/**
 * @anchor Pmic_scratchPadRegSel
 * @name TPS65036x PMIC Scratch Pad Register Selection
 *
 * @brief Scratch pad register numbers used by the Pmic_setScratchPadValue() PMIC
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
 * @brief Values to be passed into the `lock` parameter of the Pmic_setRegLockState()
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
 * @anchor Pmic_ConfigCrcStat
 * @name TPS65036x Configuration CRC Status
 *
 * @brief Struct used to obtain PMIC configuration register CRC status.
 *
 * @param crcEn Reflects the CONFIG_CRC_EN bit. True if configuration CRC is enabled.
 *
 * @param crcCalc Reflects the CONFIG_CRC_CALC bit. True if a one-shot CRC calculation
 * has been triggered and not yet cleared.
 *
 * @param errorDetected Reflects CONFIG_CRC_STATUS (bit 2 of CONFIG_CRC_CONFIG) and
 * CONFIG_CRC_STAT (bit 3 of STAT_MODERATE_ERR). True indicates a CRC mismatch was
 * detected. Auto-cleared by hardware upon reading.
 *
 * @{
 */
typedef struct Pmic_ConfigCrcStat_s
{
    bool crcEn;
    bool crcCalc;
    bool errorDetected;
} Pmic_ConfigCrcStat_t;
/** @} */

/* ========================================================================== */
/*                             Function Declarations                          */
/* ========================================================================== */

/**
 * @brief Get PMIC NVM revision.
 *
 * Design: PMICDRV-584
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-521, PMICDRV-522, PMICDRV-524,
 *               PMICDRV-528, PMICDRV-547
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param nvmRev [OUT] PMIC NVM revision.
 *
 * @return Success code if the PMIC NVM revision has been obtained, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_getNvmRev(const Pmic_Handle_t *handle, uint8_t *nvmRev);

/**
 * @brief Get PMIC silicon revision.
 *
 * Design: PMICDRV-759
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-521, PMICDRV-522, PMICDRV-524,
 *               PMICDRV-528, PMICDRV-547
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param siliconRev [OUT] PMIC silicon revision.
 *
 * @return Success code if the PMIC silicon revision has been obtained, error
 * code otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_getSiliconRev(const Pmic_Handle_t *handle, uint8_t *siliconRev);

/**
 * @brief Unlock/lock PMIC registers.
 *
 * Design: PMICDRV-587
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-508, PMICDRV-521, PMICDRV-522,
 *               PMICDRV-523, PMICDRV-545, PMICDRV-546
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param lock [IN] This parameter decides whether the API locks or unlocks PMIC
 * registers. For valid values, refer to @ref Pmic_regLockUnlockValues
 *
 * @return Success code if the register unlock/lock key has been sent to the
 * PMIC, error code otherwise. For valid success/error codes, refer to
 * @ref Pmic_ErrorCodes.
 */
int32_t Pmic_setRegLockState(const Pmic_Handle_t *handle, bool lock);

/**
 * @brief Lock PMIC registers.
 *
 * Design: PMICDRV-760
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-521, PMICDRV-522, PMICDRV-523, PMICDRV-545
 *               PMICDRV-546
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @return Success code if the register unlock key has been sent to the PMIC, error
 * code otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_enableRegLock(const Pmic_Handle_t *handle);

/**
 * @brief Unlock PMIC registers.
 *
 * Design: PMICDRV-761
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-521, PMICDRV-522, PMICDRV-523, PMICDRV-545
 *               PMICDRV-546
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @return Success code if the register lock key has been sent to the PMIC, error
 * code otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_disableRegLock(const Pmic_Handle_t *handle);

/**
 * @brief Get the PMIC register lock status.
 *
 * Design: PMICDRV-588
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-508, PMICDRV-521, PMICDRV-522,
 *               PMICDRV-528, PMICDRV-545, PMICDRV-546
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param regLockStat [OUT] PMIC register lock status. Value is set to true if PMIC
 * configuration registers are locked, else value is set to false.
 *
 * @return Success code if the PMIC register lock status has been read, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_getRegLockState(const Pmic_Handle_t *handle, bool *regLockStat);

/**
 * @brief Turn on/off the power sequence logic for regulators and other
 * components on the PMIC.
 *
 * Design: PMICDRV-762
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-521, PMICDRV-522, PMICDRV-523
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param pwrOn [IN] Device PWR_ON configuration. When set to PMIC_ENABLE, the
 * device sequence logic is turned on. Else the sequence logic is turned off.
 *
 * @return Success code if the PMIC PWR_ON bit is set, error code otherwise. For
 * valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_setPwrOn(const Pmic_Handle_t *handle, bool pwrOn);

/**
 * @brief Get the status of the PMIC PWR_ON bit.
 *
 * Design: PMICDRV-763
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-521, PMICDRV-522, PMICDRV-528
 *
 * @details PMIC sequence triggers for components (like regulators) can be
 * associated with the PWR_ON bit via the SEQ_TRIG_X registers, where X is the
 * name of the component.
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param pwrOnStat [OUT] Device power on status. When set to true, the device
 * sequence logic is turned on. Else the sequence logic is turned off.
 *
 * @return Success code if the device power on status has been obtained, error
 * code otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_getPwrOn(const Pmic_Handle_t *handle, bool *pwrOnStat);

/**
 * @brief Set PMIC low power mode configurations.
 *
 * Design: PMICDRV-764
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-521, PMICDRV-522, PMICDRV-523
 *
 * @details The following options are configurable via this API
 * 1. LPM pin detection option (validParams: PMIC_LPM_PIN_DETECTION_VALID)
 * 2. Delay time after nRSTOUT has been activated before LPM pin is recognized
 * (validParams: PMIC_LPM_DETECTION_DELAY_VALID)
 * 3. VMON activation in LPM mode (validParams: PMIC_LPM_VMON_EN_VALID)
 * 4. ESM activation in LPM mode (validParams: PMIC_LPM_ESM_EN_VALID)
 * 5. WDG activation in LPM mode (validParams: PMIC_LPM_WDG_EN_VALID)
 * For more information on LPM configurations, refer to @ref Pmic_CoreLpmCfg.
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param lpmCfg [IN] Low power mode configuration to write to PMIC.
 *
 * @return Success code if LPM configurations have been set, error code otherwise.
 * For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_setLpmCfg(const Pmic_Handle_t *handle, const Pmic_CoreLpmCfg_t *lpmCfg);

/**
 * @brief Get PMIC low power mode configurations. This API supports getting the same
 * configurations that are settable by `Pmic_setLpmCfg()`.
 *
 * Design: PMICDRV-765
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-521, PMICDRV-522, PMICDRV-528
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param lpmCfg [OUT] Low power mode configurations obtained from PMIC.
 *
 * @return Success code if LPM configurations have been obtained, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_getLpmCfg(const Pmic_Handle_t *handle, Pmic_CoreLpmCfg_t *lpmCfg);

/**
 * @brief Enable PMIC configuration register CRC checking.
 *
 * @details Optionally recalculates and writes the SW-computed CRC to the PMIC before
 * enabling continuous CRC checking. Pass `PMIC_CFG_CRC_RECALCULATE` to trigger a
 * fresh CRC computation and hardware validation before enabling. Pass
 * `PMIC_CFG_CRC_ENABLE_ONLY` to enable CRC checking using the existing value in
 * CONFIG_CRC_REG_1/2.
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param calculate [IN] Whether to recalculate the CRC before enabling.
 * For valid values, refer to @ref Pmic_cfgCrcCalcMode.
 *
 * @return Success code if configuration CRC has been enabled, error code otherwise.
 * For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_configCrcEnable(const Pmic_Handle_t *handle, bool calculate);

/**
 * @brief Disable PMIC configuration register CRC checking.
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @return Success code if configuration CRC has been disabled, error code otherwise.
 * For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_configCrcDisable(const Pmic_Handle_t *handle);

/**
 * @brief Get PMIC configuration register CRC status.
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param configCrcStat [OUT] Configuration CRC status obtained from PMIC.
 * For more information, refer to @ref Pmic_ConfigCrcStat.
 *
 * @return Success code if configuration CRC status has been obtained, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_getConfigCrcStatus(const Pmic_Handle_t *handle, Pmic_ConfigCrcStat_t *configCrcStat);

/**
 * @brief Compute the SW configuration register CRC, write it to the PMIC, and
 * trigger a hardware validation.
 *
 * @details Reads registers 0x14 through 0x4C, computes a CRC16 (polynomial 0x755B,
 * init 0xFFFF), writes the result to CONFIG_CRC_REG_1/2, then triggers a one-shot
 * hardware comparison via CONFIG_CRC_CALC. Returns `PMIC_ST_ERR_CONFIG_REG_CRC`
 * if the hardware reports a mismatch.
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @return Success code if the CRC calculation and validation succeeded, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_configCrcCalculate(const Pmic_Handle_t *handle);

/**
 * @brief Read the device-computed configuration CRC from the PMIC.
 *
 * @details Reads the CRC result stored in CALCUL_CONFIG_CRC_1/2 (0x65/0x66) by
 * the PMIC hardware after a CONFIG_CRC_CALC trigger.
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param value [OUT] Device-computed CRC value (LSB from 0x65, MSB from 0x66).
 *
 * @return Success code if the device CRC has been read, error code otherwise.
 * For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_getConfigCrc(const Pmic_Handle_t *handle, uint16_t *value);

/**
 * @brief Write the SW-computed configuration CRC to the PMIC.
 *
 * @details Writes the expected CRC to CONFIG_CRC_REG_1 (0x4D, LSB) and
 * CONFIG_CRC_REG_2 (0x4E, MSB). The PMIC hardware compares this value against
 * its own computed CRC when CONFIG_CRC_CALC is asserted or CONFIG_CRC_EN is active.
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param value [IN] CRC value to write.
 *
 * @return Success code if the CRC value has been written, error code otherwise.
 * For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_setConfigCrc(const Pmic_Handle_t *handle, uint16_t value);

/**
 * @brief Trigger run-time ABIST (analog built-in self test) on the PMIC.
 *
 * Design: PMICDRV-768
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-521, PMICDRV-522, PMICDRV-523, PMICDRV-548
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @return Success code if run-time ABIST command has been sent to PMIC, error
 * code otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_runABIST(const Pmic_Handle_t *handle);

/**
 * @brief Get the active status of PMIC ABIST.
 *
 * Design: PMICDRV-769
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-521, PMICDRV-522, PMICDRV-528, PMICDRV-548
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param isActive [OUT] ABIST status. When the value is true, ABIST is active.
 * Otherwise, ABIST is inactive.
 *
 * @return Success code if the ABIST status has been obtained, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_getABISTStat(const Pmic_Handle_t *handle, bool *isActive);

/**
 * @brief Write a value to a target scratch pad register on the PMIC.
 *
 * Design: PMICDRV-591
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-508, PMICDRV-511, PMICDRV-512,
 *               PMICDRV-515, PMICDRV-516, PMICDRV-521, PMICDRV-522, PMICDRV-523,
 *               PMICDRV-527, PMICDRV-545, PMICDRV-551
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param scratchPadRegNum [IN] Target scratch pad register number.
 *
 * @param value [IN] Desired value to be written to scratch pad register.
 *
 * @return Success code if value has been written to PMIC scratch pad register,
 * error code otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_setScratchPadValue(const Pmic_Handle_t *handle, uint8_t scratchPadRegNum, uint8_t value);

/**
 * @brief Obtain the value of a scratch pad register on the PMIC.
 *
 * Design: PMICDRV-592
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-508, PMICDRV-511, PMICDRV-512,
 *               PMICDRV-515, PMICDRV-516, PMICDRV-521, PMICDRV-522, PMICDRV-527,
 *               PMICDRV-528, PMICDRV-545, PMICDRV-551
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param scratchPadRegNum [IN] Target scratch pad register number.
 *
 * @param value [OUT] Scratch pad value obtained from the PMIC.
 *
 * @return Success code if target scratch pad register value has been obtained
 * from the PMIC, error code otherwise. For valid success/error codes, refer to
 * @ref Pmic_ErrorCodes.
 */
int32_t Pmic_getScratchPadValue(const Pmic_Handle_t *handle, uint8_t scratchPadRegNum, uint8_t *value);

#ifdef __cplusplus
}

#endif /* __cplusplus */
#endif /* PMIC_CORE_H */
