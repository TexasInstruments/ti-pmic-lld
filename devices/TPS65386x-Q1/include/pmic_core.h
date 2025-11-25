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
#ifndef PMIC_CORE_H
#define PMIC_CORE_H

/**
 * @file pmic_core.h
 * @brief PMIC Driver Core API/Interface
 */

/**
 * @defgroup DRV_PMIC_CORE_MODULE PMIC Core Feature Control
 * @brief Control, configuration, and status of fundamental PMIC features.
 */

/**
 * @defgroup DRV_PMIC_CORE_LOCK_GROUP PMIC Register Lock Control
 * @ingroup DRV_PMIC_CORE_MODULE
 * @brief Control register locks on the PMIC.
 */

/**
 * @defgroup DRV_PMIC_CORE_ID_GROUP PMIC Device Identification
 * @ingroup DRV_PMIC_CORE_MODULE
 * @brief Get (and set) identifying information on the PMIC.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdbool.h>
#include <stdint.h>

#include "pmic_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/**
 * @anchor Pmic_CoreLockCfgValidParam
 * @name PMIC Lock Control Valid Params
 *
 * @{
 */
#define PMIC_CFG_REG_LOCK_VALID (1U << 0U)
#define PMIC_CFG_CNT_LOCK_VALID (1U << 1U)
/** @} */

/**
 * @anchor Pmic_CoreLockCfgValidParamShift
 * @name PMIC Lock Control Valid Params Bit Shift Positions
 *
 * @{
 */
#define PMIC_CFG_LOCK_ALL_VALID_SHIFT (\
    PMIC_CFG_REG_LOCK_VALID |\
    PMIC_CFG_CNT_LOCK_VALID)
/** @} */

/**
 * @anchor Pmic_CoreLockControl
 * @name PMIC Lock Control Enumeration for use with `Pmic_{get,set}{Reg,Cnt}LockState()`
 *
 * @{
 */
#define PMIC_LOCK_DISABLE (0U)
#define PMIC_LOCK_ENABLE  (1U)
/** @} */

/**
 * @anchor Pmic_ScratchPadRegNum
 * @name PMIC Scratch Pad Register Numbers
 *
 * @brief Scratch pad register numbers used by the `Pmic_setScratchPadValue()`
 * and `Pmic_getScratchPadValue()` APIs.
 *
 * @{
 */
#define PMIC_SCRATCH_PAD_REG_1   ((uint8_t)0U)
#define PMIC_SCRATCH_PAD_REG_2   ((uint8_t)1U)
#define PMIC_SCRATCH_PAD_REG_MAX (PMIC_SCRATCH_PAD_REG_2)
/** @} */

/**
 * @anchor Pmic_CommonCtrlCfgValidParam
 * @name PMIC Common Control Configuration Valid Params
 *
 * @{
 */
#define PMIC_COMMON_CTRL_SPREAD_SPECTRUM_EN_VALID (1U << 0U)
#define PMIC_COMMON_CTRL_ENSAFEOUT1_VALID         (1U << 1U)
#define PMIC_COMMON_CTRL_ENSAFEOUT2_VALID         (1U << 2U)
/** @} */

/**
 * @anchor Pmic_DiagOutCfgCtrlValidParam
 * @name PMIC Diagnostic Output Control Configuration Valid Params
 *
 * @{
 */
#define PMIC_DIAG_OUT_CTRL_AMUX_EN_VALID (1U << 0U)
#define PMIC_DIAG_OUT_CTRL_DMUX_EN_VALID (1U << 1U)
#define PMIC_DIAG_OUT_CTRL_VALID         (1U << 2U)
/** @} */

/*==========================================================================*/
/*                         Structures and Enums                             */
/*==========================================================================*/

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
 * @brief Controls PMIC register lock and timer/rotation counter register lock,
 * or reports lock status.
 *
 * @note `validParams` is an input parameter for all Set and Get APIs. Other
 * struct members are input params for Set APIs and output params for Get APIs.
 *
 * @param validParams Selection of structure parameters to be set, from the
 * combination of @ref Pmic_CoreLockCfgValidParamShift and the corresponding
 * member value will be updated.
 *
 * @param cfgLock Configuration Register Lock. True if registers are l Valid
 * only when PMIC_CFG_REG_LOCK_VALID bit is set
 *
 * @param cntLock Timer/Rotation Counter Register Lock configuration. Valid
 * only when PMIC_CFG_REG_LOCK_VALID bit is set
 */
typedef struct Pmic_Lock_s {
    uint16_t validParams;

    bool cfgLock;
    bool cntLock;
} Pmic_Lock_t;

/**
 * @brief PMIC common control configuration structure
 *
 * Used for spread spectrum and SAFEOUT pin configuration
 *
 * @param validParams Selection of structure parameters to be set, from
 * @ref Pmic_CommonCtrlCfgValidParam
 * @param spreadSpectrumEn Spread spectrum enable (0=disable, 1=enable).
 * Valid when PMIC_COMMON_CTRL_SPREAD_SPECTRUM_EN_VALID is set
 * @param enSafeOut1 SAFEOUT1 enable (0=disable, 1=enable).
 * Valid when PMIC_COMMON_CTRL_ENSAFEOUT1_VALID is set
 * @param enSafeOut2 SAFEOUT2 enable (0=disable, 1=enable).
 * Valid when PMIC_COMMON_CTRL_ENSAFEOUT2_VALID is set
 */
typedef struct Pmic_CommonCtrlCfg_s {
    uint32_t validParams;
    uint8_t spreadSpectrumEn;
    uint8_t enSafeOut1;
    uint8_t enSafeOut2;
} Pmic_CommonCtrlCfg_t;

/**
 * @brief PMIC common control status structure
 *
 * Used to read back pin states and register lock status
 */
typedef struct Pmic_CommonCtrlStat_s {
    uint8_t nRstPin;          /**< NRST pin status (readback from STAT_READBACK_ERR reg bit 0) */
    uint8_t safeOut1Pin;      /**< SAFE_OUT1 pin status (readback from STAT_READBACK_ERR reg bit 1) */
    uint8_t enOutPin;         /**< EN_OUT pin status (readback from STAT_READBACK_ERR reg bit 2) */
    uint8_t cfgregLockStat;   /**< Configuration register lock status (from REG_STAT_REG bit 0) */
} Pmic_CommonCtrlStat_t;

/**
 * @brief PMIC diagnostic output control configuration structure
 *
 * @param validParams Selection of structure parameters to be set, from
 * @ref Pmic_DiagOutCfgCtrlValidParam
 * @param diagOutCtrl_AMUXEn AMUX enable (0=disable, 1=enable).
 * Valid when PMIC_DIAG_OUT_CTRL_AMUX_EN_VALID is set
 * @param diagOutCtrl_DMUXEn DMUX enable (0=disable, 1=enable).
 * Valid when PMIC_DIAG_OUT_CTRL_DMUX_EN_VALID is set
 * @param diagOutCtrl Diagnostic output control state (0=disabled, 1=AMUX, 2=DMUX).
 * Valid when PMIC_DIAG_OUT_CTRL_VALID is set
 */
typedef struct Pmic_DiagOutCfgCtrl_s {
    uint32_t validParams;
    uint8_t diagOutCtrl_AMUXEn;
    uint8_t diagOutCtrl_DMUXEn;
    uint8_t diagOutCtrl;
} Pmic_DiagOutCfgCtrl_t;

/*==========================================================================*/
/*                         Function Declarations                            */
/*==========================================================================*/

/**
 * @brief Get PMIC NVM revision.
 *
 * Design: PMICDRV-584
 * Architecture: PMICDRV-507, PMICDRV-516, PMICDRV-508, PMICDRV-547, PMICDRV-549, PMICDRV-550
 *               PMICDRV-551, PMICDRV-506, PMICDRV-524, PMICDRV-504, PMICDRV-522, PMICDRV-528
 *               PMICDRV-521, PMICDRV-512
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param nvmRev [OUT] PMIC NVM revision.
 *
 * @return PMIC_ST_SUCCESS if PMIC NVM revision has been obtained, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_getNvmRev(Pmic_Handle_t *handle, uint8_t *nvmRev);

/**
 * @brief Get PMIC silicon revision.
 *
 * Design: PMICDRV-583
 * Architecture: PMICDRV-507, PMICDRV-516, PMICDRV-508, PMICDRV-547, PMICDRV-549, PMICDRV-550
 *               PMICDRV-551, PMICDRV-506, PMICDRV-524, PMICDRV-504, PMICDRV-522, PMICDRV-528
 *               PMICDRV-521, PMICDRV-512
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param siliconRev [OUT] PMIC silicon revision.
 *
 * @return PMIC_ST_SUCCESS if PMIC silicon revision has been obtained, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_getSiliconRev(Pmic_Handle_t *handle, uint8_t *siliconRev);

/**
 * @ingroup DRV_PMIC_CORE_LOCK_GROUP
 * @brief Set register lock and counter lock configurations. This API is a superset
 * of `Pmic_setRegLockState()` and `Pmic_setCntLockState()`.
 *
 * Design: PMICDRV-585
 * Architecture: PMICDRV-507, PMICDRV-516, PMICDRV-508, PMICDRV-523, PMICDRV-547, PMICDRV-549
 *               PMICDRV-550, PMICDRV-551, PMICDRV-545, PMICDRV-546, PMICDRV-506, PMICDRV-504
 *               PMICDRV-522, PMICDRV-521, PMICDRV-512
 *
 * @param handle [IN] Pointer to the PMIC core handle structure.
 * @param config [IN] Pointer to the lock configuration structure containing
 * the register lock/unlock parameters.
 *
 * @return Returns PMIC_ST_SUCCESS if the operation is successful; otherwise,
 * it returns an appropriate error code. For possible values, see @ref
 * Pmic_ErrorCodes.
 */
int32_t Pmic_setLockCfg(Pmic_Handle_t *handle, const Pmic_Lock_t *config);

/**
 * @ingroup DRV_PMIC_CORE_LOCK_GROUP
 * @brief Get register lock and counter lock configurations. This API is a superset
 * of `Pmic_getRegLockState()` and `Pmic_getCntLockState()`.
 *
 * Design: PMICDRV-586
 * Architecture: PMICDRV-507, PMICDRV-516, PMICDRV-508, PMICDRV-547, PMICDRV-549, PMICDRV-550
 *               PMICDRV-551, PMICDRV-545, PMICDRV-546, PMICDRV-506, PMICDRV-504, PMICDRV-522
 *               PMICDRV-528, PMICDRV-521, PMICDRV-512
 *
 * @param handle Pointer to the PMIC core handle structure.
 * @param config [IN/OUT] Pointer to the lock configuration structure to store
 * the retrieved register lock status.
 *
 * @return Returns PMIC_ST_SUCCESS if the operation is successful; otherwise,
 * it returns an appropriate error code. For possible values, see @ref
 * Pmic_ErrorCodes.
 */
int32_t Pmic_getLockCfg(Pmic_Handle_t *handle, Pmic_Lock_t *config);

/**
 * @ingroup DRV_PMIC_CORE_LOCK_GROUP
 * @brief Lock/unlock registers that are locked by CFG_REG_LOCK. This API is a
 * subset of `Pmic_setLockCfg()`.
 *
 * Design: PMICDRV-587
 * Architecture: PMICDRV-507, PMICDRV-516, PMICDRV-508, PMICDRV-523, PMICDRV-547, PMICDRV-549
 *               PMICDRV-550, PMICDRV-551, PMICDRV-545, PMICDRV-546, PMICDRV-506, PMICDRV-504
 *               PMICDRV-522, PMICDRV-521, PMICDRV-512
 *
 * @param handle    [IN] Pointer to the PMIC core handle structure.
 * @param lockState [IN] Lock registers with PMIC_LOCK_ENABLE, unlock with
 * PMIC_LOCK_DISABLE. See @ref Pmic_CoreLockControl.
 *
 * @return Returns PMIC_ST_SUCCESS if the operation is successful; otherwise,
 * it returns an appropriate error code. For possible values, see @ref
 * Pmic_ErrorCodes.
 */
int32_t Pmic_setRegLockState(Pmic_Handle_t *handle, uint8_t lockState);

/**
 * @ingroup DRV_PMIC_CORE_LOCK_GROUP
 * @brief Get lock state for registers locked by CFG_REG_LOCK. This API is a subset
 * of `Pmic_getLockCfg()`.
 *
 * Design: PMICDRV-588
 * Architecture: PMICDRV-507, PMICDRV-516, PMICDRV-508, PMICDRV-547, PMICDRV-549, PMICDRV-550
 *               PMICDRV-551, PMICDRV-545, PMICDRV-546, PMICDRV-506, PMICDRV-504, PMICDRV-522
 *               PMICDRV-528, PMICDRV-521, PMICDRV-512
 *
 * @param handle    [IN]  Pointer to the PMIC core handle structure.
 * @param lockState [OUT] If PMIC_LOCK_ENABLE, registers are locked. If
 * PMIC_LOCK_DISABLE, registers are unlocked. See @ref Pmic_CoreLockControl.
 *
 * @return Returns PMIC_ST_SUCCESS if the operation is successful; otherwise,
 * it returns an appropriate error code. For possible values, see @ref
 * Pmic_ErrorCodes.
 */
int32_t Pmic_getRegLockState(Pmic_Handle_t *handle, uint8_t *lockState);

/**
 * @ingroup DRV_PMIC_CORE_LOCK_GROUP
 * @brief Lock/unlock registers that are locked by CNT_REG_LOCK. This API is a
 * subset of `Pmic_setLockCfg()`.
 *
 * Design: PMICDRV-589
 * Architecture: PMICDRV-507, PMICDRV-516, PMICDRV-508, PMICDRV-523, PMICDRV-547, PMICDRV-549
 *               PMICDRV-550, PMICDRV-551, PMICDRV-545, PMICDRV-546, PMICDRV-506, PMICDRV-504
 *               PMICDRV-522, PMICDRV-521, PMICDRV-512
 *
 * @param handle    [IN] Pointer to the PMIC core handle structure.
 * @param lockState [IN] Lock registers with PMIC_LOCK_ENABLE, unlock with
 * PMIC_LOCK_DISABLE. See @ref Pmic_CoreLockControl.
 *
 * @return Returns PMIC_ST_SUCCESS if the operation is successful; otherwise,
 * it returns an appropriate error code. For possible values, see @ref
 * Pmic_ErrorCodes.
 */
int32_t Pmic_setCntLockState(Pmic_Handle_t *handle, uint8_t lockState);

/**
 * @ingroup DRV_PMIC_CORE_LOCK_GROUP
 * @brief Get lock state for registers locked by CNT_REG_LOCK. This API is a subset
 * of `Pmic_getLockCfg()`.
 *
 * Design: PMICDRV-590
 * Architecture: PMICDRV-507, PMICDRV-516, PMICDRV-508, PMICDRV-547, PMICDRV-549, PMICDRV-550
 *               PMICDRV-551, PMICDRV-545, PMICDRV-546, PMICDRV-506, PMICDRV-504, PMICDRV-522
 *               PMICDRV-528, PMICDRV-521, PMICDRV-512
 *
 * @param handle    [IN]  Pointer to the PMIC core handle structure.
 * @param lockState [OUT] If PMIC_LOCK_ENABLE, registers are locked. If
 * PMIC_LOCK_DISABLE, registers are unlocked. See @ref Pmic_CoreLockControl.
 *
 * @return Returns PMIC_ST_SUCCESS if the operation is successful; otherwise,
 * it returns an appropriate error code. For possible values, see @ref
 * Pmic_ErrorCodes.
 */
int32_t Pmic_getCntLockState(Pmic_Handle_t *handle, uint8_t *lockState);

/**
 * @brief Write a value to a target scratch pad register on the PMIC.
 *
 * Design: PMICDRV-684
 * Architecture: PMICDRV-507, PMICDRV-516, PMICDRV-508, PMICDRV-523, PMICDRV-547, PMICDRV-549
 *               PMICDRV-550, PMICDRV-551, PMICDRV-545, PMICDRV-546, PMICDRV-506, PMICDRV-504
 *               PMICDRV-522, PMICDRV-521, PMICDRV-512
 *
 * @param handle           [IN] PMIC interface handle.
 * @param scratchPadRegNum [IN] Target scratch pad register number.
 * @param value            [IN] Value to be written to scratch pad register.
 *
 * @return Success code if value has been written to PMIC scratch pad register,
 * error code otherwise. For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_setScratchPadValue(Pmic_Handle_t *handle, uint8_t scratchPadRegNum, uint8_t value);

/**
 * @brief Obtain the value of a scratch pad register on the PMIC.
 *
 * Design: PMICDRV-685
 * Architecture: PMICDRV-507, PMICDRV-516, PMICDRV-508, PMICDRV-547, PMICDRV-549, PMICDRV-550
 *               PMICDRV-551, PMICDRV-545, PMICDRV-546, PMICDRV-506, PMICDRV-504, PMICDRV-522
 *               PMICDRV-528, PMICDRV-521, PMICDRV-512
 *
 * @param handle           [IN] PMIC interface handle.
 * @param scratchPadRegNum [IN] Target scratch pad register number.
 * @param value            [OUT] Scratch pad value obtained from the PMIC.
 *
 * @return Success code if target scratch pad register value has been obtained
 * from the PMIC, error code otherwise. For valid success/error codes, refer to
 * @ref Pmic_errorCodes.
 */
int32_t Pmic_getScratchPadValue(Pmic_Handle_t *handle, uint8_t scratchPadRegNum, uint8_t *value);

/**
 * @brief Enable or disable spread spectrum modulation.
 *
 * @param handle [IN] PMIC interface handle.
 * @param config [IN] Common control configuration with spreadSpectrumEn set.
 *
 * @return PMIC_ST_SUCCESS if successful, error code otherwise.
 */
int32_t Pmic_spreadSpectrumEnable(Pmic_Handle_t *handle, Pmic_CommonCtrlCfg_t config);

/**
 * @brief Get spread spectrum enable status.
 *
 * @param handle [IN] PMIC interface handle.
 * @param config [OUT] Common control configuration to store spread spectrum status.
 *
 * @return PMIC_ST_SUCCESS if successful, error code otherwise.
 */
int32_t Pmic_getSpreadSpectrumEnable(Pmic_Handle_t *handle, Pmic_CommonCtrlCfg_t *config);

/**
 * @brief Enable or disable SAFEOUT pins.
 *
 * @param handle [IN] PMIC interface handle.
 * @param config [IN] Common control configuration with enSafeOut1/enSafeOut2 set.
 *
 * @return PMIC_ST_SUCCESS if successful, error code otherwise.
 */
int32_t Pmic_setEnableSafeOutCfg(Pmic_Handle_t *handle, Pmic_CommonCtrlCfg_t config);

/**
 * @brief Get SAFEOUT pin configuration status.
 *
 * @param handle [IN] PMIC interface handle.
 * @param config [OUT] Common control configuration to store SAFEOUT status.
 *
 * @return PMIC_ST_SUCCESS if successful, error code otherwise.
 */
int32_t Pmic_getSafeOutPinCfg(Pmic_Handle_t *handle, Pmic_CommonCtrlCfg_t *config);

/**
 * @brief Get common control status (pin states and lock status).
 *
 * @param handle [IN] PMIC interface handle.
 * @param stat [OUT] Common control status structure.
 *
 * @return PMIC_ST_SUCCESS if successful, error code otherwise.
 */
int32_t Pmic_getCommonStat(Pmic_Handle_t *handle, Pmic_CommonCtrlStat_t *stat);

/**
 * @brief Set diagnostic output control configuration (AMUX/DMUX enable).
 *
 * @param handle [IN] PMIC interface handle.
 * @param config [IN] Diagnostic output control configuration.
 *
 * @return PMIC_ST_SUCCESS if successful, error code otherwise.
 */
int32_t Pmic_setDiagOutCtrlConfig(Pmic_Handle_t *handle, Pmic_DiagOutCfgCtrl_t config);

/**
 * @brief Get diagnostic output control configuration.
 *
 * @param handle [IN] PMIC interface handle.
 * @param config [OUT] Diagnostic output control configuration.
 *
 * @return PMIC_ST_SUCCESS if successful, error code otherwise.
 */
int32_t Pmic_getDiagOutCtrlConfig(Pmic_Handle_t *handle, Pmic_DiagOutCfgCtrl_t *config);

/**
 * @brief Set AMUX channel configuration.
 *
 * @param handle [IN] PMIC interface handle.
 * @param channel [IN] AMUX channel to select (0-31).
 *
 * @return PMIC_ST_SUCCESS if successful, error code otherwise.
 */
int32_t Pmic_setDiagAMUXFeatureCfg(Pmic_Handle_t *handle, uint8_t channel);

/**
 * @brief Get AMUX channel configuration.
 *
 * @param handle [IN] PMIC interface handle.
 * @param channel [OUT] Current AMUX channel (0-31).
 *
 * @return PMIC_ST_SUCCESS if successful, error code otherwise.
 */
int32_t Pmic_getDiagAMUXFeatureCfg(Pmic_Handle_t *handle, uint8_t *channel);

/**
 * @brief Set DMUX group configuration.
 *
 * @param handle [IN] PMIC interface handle.
 * @param group [IN] DMUX group to select (0-31).
 *
 * @return PMIC_ST_SUCCESS if successful, error code otherwise.
 */
int32_t Pmic_setDiagDMUXFeatureCfg(Pmic_Handle_t *handle, uint8_t group);

/**
 * @brief Get DMUX group configuration.
 *
 * @param handle [IN] PMIC interface handle.
 * @param group [OUT] Current DMUX group (0-31).
 *
 * @return PMIC_ST_SUCCESS if successful, error code otherwise.
 */
int32_t Pmic_getDiagDMUXFeatureCfg(Pmic_Handle_t *handle, uint8_t *group);

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* PMIC_CORE_H */
