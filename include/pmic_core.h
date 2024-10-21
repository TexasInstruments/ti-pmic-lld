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
 * structures, and APIs used to interact with PMIC core functionalities.
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
 * @anchor Pmic_DeviceCfgValidParams
 * @name PMIC Device Configuration Valid Parameters
 *
 * @brief Indication of which parameters are valid within the Pmic_DeviceCfg_t
 * struct. For more information on the parameters, refer to @ref Pmic_DeviceCfg.
 *
 * @note End-user could combine multiple valid parameters using the OR operator.
 *
 * @{
 */
#define PMIC_DIAG_EXIT_VALID                    ((uint32_t)(1U << 0U))
#define PMIC_DIAG_EXIT_MASK_VALID               ((uint32_t)(1U << 1U))
#define PMIC_ENABLE_DRV_VALID                   ((uint32_t)(1U << 2U))
#define PMIC_DISABLE_AUTO_BIST_VALID            ((uint32_t)(1U << 3U))
#define PMIC_DISABLE_SAFE_LOCK_TIMEOUT_VALID    ((uint32_t)(1U << 4U))
#define PMIC_SAFE_TIMEOUT_DURATION_VALID        ((uint32_t)(1U << 5U))
#define PMIC_SAFE_LOCK_THR_VALID                ((uint32_t)(1U << 6U))
#define PMIC_DEV_ERR_CNT_VALID                  ((uint32_t)(1U << 7U))
#define PMIC_PWR_DWN_THR_VALID                  ((uint32_t)(1U << 8U))
/** @} */

/**
 * @anchor Pmic_drvInitValues
 * @name PMIC LLD Initialization Values
 *
 * @brief Indication of whether the driver is initialized or not.
 *
 * @details The value is used to help prevent corrupt PMIC handle usage. When
 * converting @p PMIC_DRV_INIT to ASCII, the value reads out to be "PMIC".
 *
 * @{
 */
#define PMIC_DRV_INIT                           ((uint32_t)0x504D4943U)
#define PMIC_DRV_UNINIT                         ((uint32_t)0U)

/**
 * @anchor Pmic_regLockUnlockValues
 * @name TPS65385x Register Lock/Unlock Values
 *
 * @brief Values to be passed into the `lock` parameter of the Pmic_setRegLock()
 * API.
 *
 * @{
 */
#define PMIC_LOCK                               ((bool)true)
#define PMIC_UNLOCK                             ((bool)false)
/** @} */

/**
 * @anchor Pmic_DevState
 * @name PMIC Device States
 *
 * @brief Valid values returned from the Pmic_getDevState() API.
 *
 * @{
 */
#define PMIC_DIAGNOSTIC_STATE                   ((uint8_t)7U)
#define PMIC_ACTIVE_STATE                       ((uint8_t)5U)
#define PMIC_SAFE_STATE                         ((uint8_t)4U)
/** @} */

/**
 * @anchor Pmic_DeviceCfgMaxValues
 * @name PMIC Device Configuration Maximum Values
 *
 * @brief Maximum values of the Pmic_DeviceCfg_t struct parameters.
 *
 * @{
 */
#define PMIC_SAFE_TO_DURATION_MAX               ((uint8_t)7U)
#define PMIC_SAFE_LOCK_THR_MAX                  ((uint8_t)0xFU)
#define PMIC_DEV_ERR_CNT_MAX                    ((uint8_t)0xFU)
#define PMIC_PWR_DWN_THR_MAX                    ((uint8_t)0xFU)
/** @} */

/* ========================================================================== */
/*                             Structures and Enums                           */
/* ========================================================================== */

/**
 * @anchor Pmic_DeviceCfg
 * @name PMIC Device Configuration Struct
 *
 * @brief Used to set and get PMIC device configurations.
 *
 * @param validParams Each bit in this variable represents whether a struct
 * member is valid. for valid values, refer to @ref Pmic_deviceCfgValidParams.
 *
 * @param diagExit Control bit for enabling/disabling PMIC state transitions
 * from DIAGNOSTIC state to ACTIVE state.
 *
 * @param diagExitMask Control bit for keeping the device in DIAGNOSTIC state.
 * Masks/unmasks DIAG_EXIT bit.
 *
 * @param enableDrv Control bit for the ENDRV output. When set to true, the pin
 * is high only when WD_FAIL_CNT[2:0] < 5, MCU_ERR_CNT[3:0] <= MCU_ERR_CNT_TH[3:0],
 * and the device is in DIAGNOSTIC or ACTIVE state. Otherwise the pin is set low.
 *
 * @param disableAutoBIST Enable/disable the ABIST and LBIST when the PMIC is in
 * the RESET state. When set to true, ABIST and LBIST are disabled from
 * automatically initiating when the device enters RESET state.
 *
 * @param disableSafeLockTimeout Controls the timeout function of the SAFE-state
 * lock, which protects against potential MCU locked state. When set to true, the
 * SAFE state lock timeout is disabled and the device remains locked in the SAFE
 * state when the DEV_ERR_CNT[3:0] counter value reaches the SAFE_LOCK_THR[3:0]
 * threshold value. Otherwise, the SAFE state lock timeout is enabled and the
 * device transitions to the RESET state after a programmed delay regardless of
 * the DEV_ERR_CNT[3:0] and SAFE_LOCK_THR[3:0] bit settings. This configuration
 * is automatically cleared after the MCU sends a SAFE_EXIT command in the SAFE
 * state.
 *
 * @param safeTimeoutDuration The time limit / duration of the SAFE state. If
 * the duration elapses and Safe Lock Timeout is enabled, a transition to RESET
 * state occurs. For the maximum valid value, see @ref Pmic_DeviceCfgMaxValues.
 *
 * @param safeLockThr Safe lock threshold (SAFE_LOCK_THR[3:0]). If
 * DEV_ERR_CNT[3:0] is equal to this threshold value and the Safe Lock Timeout
 * is enabled, a transition to RESET state occurs. For the maximum valid value,
 * see @ref Pmic_DeviceCfgMaxValues.
 *
 * @param devErrCnt Device error count (DEV_ERR_CNT[3:0]). The counter keeps
 * track of transitions to the SAFE state and global device error conditions.
 * When the counter is greater than or equal to the power down threshold, the
 * PMIC goes to the OFF state. For the maximum valid value, see
 * @ref Pmic_DeviceCfgMaxValues.
 *
 * @param PwrDwnThr Power down threshold. when DEV_ERR_CNT[3:0] is equal to this
 * threshold value, the device goes to the OFF state, and wakes up on a new
 * ignition or CAN-wakeup event. For the maximum valid value, see
 * @ref Pmic_DeviceCfgMaxValues.
 */
typedef struct Pmic_DeviceCfg_s
{
    uint32_t validParams;

    bool diagExit;
    bool diagExitMask;

    bool enableDrv;

    bool disableAutoBIST;
    bool disableSafeLockTimeout;

    uint8_t safeTimeoutDuration;
    uint8_t safeLockThr;

    uint8_t devErrCnt;
    uint8_t pwrDwnThr;
} Pmic_DeviceCfg_t;

/**
 * @anchor Pmic_BistStat
 * @name PMIC BIST Statuses
 *
 * @brief PMIC built-in self test statuses. Consists of BIST statuses for logic,
 * analog, and steering angle monitor (SAM).
 *
 * @param lbistDone Logic BIST completion status flag.
 *
 * @param abistDone Analog BIST completion status flag.
 *
 * @param lbistErr Error flag indicating failure of LBIST or LBIST did not pass.
 *
 * @param abistErr Error flag indicating failure of ABIST or ABIST did not pass.
 *
 * @param samBistErr Error flag indicating failure of SAM BIST or SAM BIST did
 * not pass.
 */
typedef struct Pmic_BistStat_s
{
    bool lbistDone;
    bool abistDone;

    bool lbistErr;
    bool abistErr;
    bool samBistErr;
} Pmic_BistStat_t;

/* ========================================================================== */
/*                             Function Declarations                          */
/* ========================================================================== */

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
 * @brief Get the PMIC register lock status.
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @param locked [OUT] PMIC register lock status. Value is set to true if PMIC
 * configuration registers are locked, else value is set to false.
 *
 * @return Success code if the PMIC register lock status has been read, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_getRegLock(const Pmic_CoreHandle_t *pmicHandle, bool *locked);

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
 * @brief Set PMIC core device configurations.
 *
 * @details The following options are configurable via this API
 * 1. Diagnostic Exit (validParam: PMIC_DIAG_EXIT_VALID)
 * 2. Diagnostic Exit Mask (validParam: PMIC_DIAG_EXIT_MASK_VALID)
 * 3. ENABLE_DRV (validParam: PMIC_ENABLE_DRV_VALID)
 * 4. Automatic BIST Enable/Disable (validParam: PMIC_DISABLE_AUTO_BIST_VALID)
 * 5. Safe Lock Timeout Enable/Disable (validParam: PMIC_DISABLE_SAFE_LOCK_TIMEOUT_VALID)
 * 6. Safe Timeout Duration (validParam: PMIC_SAFE_TIMEOUT_DURATION_VALID)
 * 7. Safe Lock Threshold (validParam: PMIC_SAFE_LOCK_THR_VALID)
 * 8. Device Error Count (validParam: PMIC_DEV_ERR_CNT_VALID)
 * 9. Power Down Threshold (validParam: PMIC_PWR_DWN_THR_VALID)
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @param deviceCfg [IN] PMIC core device configurations to write to PMIC.
 *
 * @return Success code if PMIC device configurations have been set, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_setDeviceCfg(const Pmic_CoreHandle_t *pmicHandle, const Pmic_DeviceCfg_t *deviceCfg);

/**
 * @brief Get PMIC core device configurations. This API supports getting the
 * same configurations that are settable through Pmic_setDeviceCfg().
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @param deviceCfg [OUT] PMIC core device configurations obtained from the PMIC.
 *
 * @return success code if the PMIC device configurations have been obtained,
 * error code otherwise. For valid success/error codes, refer to
 * @ref Pmic_errorCodes.
 */
int32_t Pmic_getDeviceCfg(const Pmic_CoreHandle_t *pmicHandle, Pmic_DeviceCfg_t *deviceCfg);

/**
 * @brief Calculates the CRC of the PMIC configuration registers and sends the
 * result to the PMIC.
 *
 * @details The PMIC has a CRC controller that performs a CRC to verify the
 * integrity of the device configuration registers. To perform this CRC check,
 * the PMIC must have the known-good checksum value from the MCU, which is
 * achieved by calling this API.
 * Once the checksum value has been written and configuration register CRC is
 * enabled, the PMIC's CRC controller calculates an internal checksum-value of
 * the configuration registers. Afterwards, this value is compared against the
 * known-good checksum value sent by the MCU via this API.
 *
 * @note This API should be called before Pmic_setCfgCrc() or
 * Pmic_enableCfgCrc().
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @return Success code if the configuration register CRC has been written to
 * the PMIC, error code otherwise. for valid success/error codes, refer to
 * @ref Pmic_errorCodes.
 */
int32_t Pmic_writeCfgCrc(const Pmic_CoreHandle_t *pmicHandle);

/**
 * @brief Enable or disable PMIC configuration register CRC.
 *
 * @attention The desired device register configuration should be set and
 * configuration register CRC should be written via Pmic_writeCfgCrc() before
 * calling this API.
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @param CfgCrcEnable [IN] Enables configuration register CRC if set to
 * true, Otherwise, disables configuration register CRC.
 *
 * @return Success code if the PMIC configuration register CRC has been enabled
 * or disabled, error code otherwise. For valid success/error codes, refer to
 * @ref Pmic_errorCodes.
 */
int32_t Pmic_setCfgCrc(const Pmic_CoreHandle_t *pmicHandle, bool CfgCrcEnable);

/**
 * @brief Enable PMIC configuration register CRC. This API is a subset of
 * Pmic_setCfgCrc().
 *
 * @attention The desired device register configuration should be set and
 * configuration register CRC should be written via Pmic_writeCfgCrc() before
 * calling this API.
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @return Success code if the PMIC configuration register CRC has been enabled,
 * error code otherwise. For valid success/error codes, refer to
 * @ref Pmic_errorCodes.
 */
int32_t Pmic_enableCfgCrc(const Pmic_CoreHandle_t *pmicHandle);

/**
 * @brief Disable PMIC configuration register CRC. This API is a subset of
 * Pmic_setCfgCrc().
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @return Success code if the PMIC configuration register CRC has been
 * disabled, error code otherwise. For valid success/error codes, refer to
 * @ref Pmic_errorCodes.
 */
int32_t Pmic_disableCfgCrc(const Pmic_CoreHandle_t *pmicHandle);

/**
 * @brief Get the PMIC configuration register CRC error status.
 *
 * @note In the SAFE State, the CRC error status can be cleared via this API
 * after the MCU has disabled configuration register CRC and sends the SAFE_EXIT
 * command.
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @param CfgCrcErr [OUT] CRC error status obtained from the PMIC. This
 * parameter returned as true indicates that the calculated CRC value for the
 * configuration registers does not match expected CRC value stored in the
 * SAFETY_CFG_CRC register.
 *
 * @return Success code if the PMIC configuration register CRC error status has
 * been obtained, error code otherwise. For valid success/error codes, refer to
 * @ref Pmic_errorCodes.
 */
int32_t Pmic_getCfgCrcErrStat(const Pmic_CoreHandle_t *pmicHandle, bool *CfgCrcErr);

/**
 * @brief Send PMIC RESET request.
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @return Success code if RESET request has been sent to the PMIC, error code
 * otherwise. for valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_sendRstReq(const Pmic_CoreHandle_t *pmicHandle);

/**
 * @brief Send PMIC Safe Exit request.
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @return Success code if Safe Exit request has been sent to the PMIC, error
 * code otherwise. For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_sendSafeExitReq(const Pmic_CoreHandle_t *pmicHandle);

/**
 * @brief Get the PMIC device state.
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @param devState [OUT] Device state obtained from the PMIC. See
 * @ref Pmic_DevState for the possible valid device states.
 *
 * @return Success code if the device state has been obtained from the PMIC,
 * error code otherwise. For valid success/error codes, refer to
 * @ref Pmic_errorCodes.
 */
int32_t Pmic_getDevState(const Pmic_CoreHandle_t *pmicHandle, uint8_t *devState);

/**
 * @brief Get the PMIC built-in self test statuses.
 *
 * @note This API also clears the BIST status errors (if the underlying cause of
 * the statuses have been resolved).
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @param bistStat [OUT] BIST statuses obtained from the PMIC.
 *
 * @return Success code if the BIST statuses have been obtained from the PMIC,
 * error code otherwise. For valid success/error codes, refer to
 * @ref Pmic_errorCodes.
 */
int32_t Pmic_getBistStat(const Pmic_CoreHandle_t *pmicHandle, Pmic_BistStat_t *bistStat);

/**
 * @brief Initiate a PMIC LBIST run.
 *
 * @note Upon successful call of this API, the DIAG_EXIT_MASK bit will be set
 * to zero. If end-user wishes to remain in Diagnostic state, the bit should be
 * set to 1 after LBIST completion.
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @return Success code if LBIST has been initiated, error code otherwise. For
 * valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_startLBIST(const Pmic_CoreHandle_t *pmicHandle);

/**
 * @brief Initiate a PMIC ABIST run.
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @return Success code if ABIST has been initiated, error code otherwise. For
 * valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_startABIST(const Pmic_CoreHandle_t *pmicHandle);

/**
 * @brief Used internally by the driver to get the CRC8 lookup table.
 *
 * @param crc8Table [OUT] CRC8 lookup table.
 */
void Pmic_getCrc8Table(const uint8_t **crc8Table);

/**
 * @brief Used internally by the driver to get CRC8 data for I/O transfers or
 * configuration register CRC.
 *
 * @param data [IN] Array of bytes.
 *
 * @param length [IN] Number of bytes in `data` parameter.
 *
 * @return CRC8 value calculated based on the input data and length.
 */
uint8_t Pmic_getCRC8Val(const uint8_t *data, uint8_t length);

#ifdef __cplusplus
}

#endif /* __cplusplus */
#endif /* __PMIC_CORE_H__ */
