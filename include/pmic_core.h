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
#ifndef PMIC_CORE_H
#define PMIC_CORE_H

/**
 * @file pmic_core.h
 *
 * @brief PMIC Core interface. Contains APIs, macros/defines, and data structures
 * used to configure, control, and interact with PMIC core and miscellaneous
 * features.
 */

/* ========================================================================= */
/*                              Include Files                                */
/* ========================================================================= */

#include "pmic_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================= */
/*                            Macros & Typedefs                              */
/* ========================================================================= */

/**
 * @anchor Pmic_scratchPadRegSel
 * @name PMIC Scratch Pad Register Selection
 *
 * @brief Scratch pad register numbers used by `Pmic_{set,get}ScratchPadVal()`.
 *
 * @{
 */
#define PMIC_SCRATCH_PAD_REG_1   ((uint8_t)0U)
#define PMIC_SCRATCH_PAD_REG_2   ((uint8_t)1U)
#define PMIC_SCRATCH_PAD_REG_3   ((uint8_t)2U)
#define PMIC_SCRATCH_PAD_REG_4   ((uint8_t)3U)
#define PMIC_SCRATCH_PAD_REG_MIN (PMIC_SCRATCH_PAD_REG_1)
#define PMIC_SCRATCH_PAD_REG_MAX (PMIC_SCRATCH_PAD_REG_4)
/** @} */

/**
 * @anchor Pmic_CoreLockControl
 * @name PMIC Lock Enable/Disable Values
 *
 * @brief PMIC lock control enumeration for use with `Pmic_{get,set}RegLockState()`.
 *
 * @{
 */
#define PMIC_LOCK_DISABLE ((bool)false)
#define PMIC_LOCK_ENABLE  ((bool)true)
/** @} */

/* ========================================================================= */
/*                           Structures and Enums                            */
/* ========================================================================= */

/* ========================================================================= */
/*                           Function Declarations                           */
/* ========================================================================= */

/**
 * @brief Get PMIC device ID.
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param devId [OUT] PMIC device ID.
 *
 * @return Success code if PMIC device ID has been obtained, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_getDeviceId(const Pmic_Handle_t *handle, uint8_t *devId);

/**
 * @brief Get PMIC device silicon revision.
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param siRev [OUT] PMIC device silicon revision.
 *
 * @return Success code if PMIC device silicon revision has been obtained, error
 * code otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_getDeviceSiRev(const Pmic_Handle_t *handle, uint8_t *siRev);

/**
 * @brief Get PMIC device NVM ID.
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param nvmId [OUT] PMIC device NVM ID.
 *
 * @return Success code if PMIC device NVM ID has been obtained, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_getDeviceNvmId(const Pmic_Handle_t *handle, uint8_t *nvmId);

/**
 * @brief Get PMIC device NVM revision.
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param nvmRev [OUT] PMIC device NVM revision.
 *
 * @return Success code if PMIC device NVM revision has been obtained, error
 * code otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_getDeviceNvmRev(const Pmic_Handle_t *handle, uint8_t *nvmRev);

/**
 * @brief Enable/disable PMIC user-space register lock.
 *
 * @details User registers except the ESM and WDG configuration registers are
 * write protected by a register lock. End-user can utilize this API to enable
 * or disable this register lock.
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param lock [IN] Enable/disable register lock. For valid values, refer to
 * @ref Pmic_CoreLockControl.
 *
 * @return Success code if PMIC user-space registers have been locked/unlocked,
 * error code otherwise. For valid success error codes, refer to
 * @ref Pmic_ErrorCodes.
 */
int32_t Pmic_setRegLockState(const Pmic_Handle_t *handle, bool lock);

/**
 * @brief Get PMIC user-space register lock status.
 *
 * @details User registers except the ESM and WDG configuration registers are
 * write protected by a register lock. End-user can utilize this API to get the
 * enable/disable status of the register lock.
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param isLocked [OUT] PMIC user-space register lock status. For valid
 * returned values, refer to @ref Pmic_CoreLockControl.
 *
 * @return Success code if the PMIC user-space register lock status has been
 * obtained, error code otherwise. For valid success/error codes, refer to
 * @ref Pmic_ErrorCodes.
 */
int32_t Pmic_getRegLockState(const Pmic_Handle_t *handle, bool *isLocked);

/**
 * @brief Set the value of a target scratch pad register on the PMIC.
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param scratchPadRegNum [IN] Target scratch pad register number. For valid
 * values, refer to @ref Pmic_scratchPadRegSel.
 *
 * @param value [IN] Value to be written to scratch pad register.
 *
 * @return Success code if value has been written to PMIC scratch pad register,
 * error code otherwise. For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_setScratchPadValue(const Pmic_Handle_t *handle, uint8_t scratchPadRegNum, uint8_t value);

/**
 * @brief Obtain the value of a scratch pad register on the PMIC.
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param scratchPadRegNum [IN] Target scratch pad register number. For valid
 * values, refer to @ref Pmic_scratchPadRegSel.
 *
 * @param value [OUT] Scratch pad value obtained from the PMIC.
 *
 * @return Success code if target scratch pad register value has been obtained
 * from the PMIC, error code otherwise. For valid success/error codes, refer to
 * @ref Pmic_errorCodes.
 */
int32_t Pmic_getScratchPadValue(const Pmic_Handle_t *handle, uint8_t scratchPadRegNum, uint8_t *value);

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* PMIC_CORE_H */
