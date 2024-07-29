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
 * structures, and APIs used to interact with the PMIC core functionality.
 */
#ifndef __PMIC_CORE_H__
#define __PMIC_CORE_H__

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                Include Files                               */
/* ========================================================================== */
#include <stdint.h>
#include <stdbool.h>

#include "pmic_common.h"

/**
 * @anchor Pmic_scratchPadRegSel
 * @name PMIC Scratch Pad Register Selection
 *
 * @brief Scratch pad register numbers used by the `Pmic_setScratchPadVal()`
 * PMIC Core API.
 *
 * @{
 */
#define PMIC_SCRATCH_PAD_REG_1   ((uint8_t)0U)
#define PMIC_SCRATCH_PAD_REG_2   ((uint8_t)1U)
#define PMIC_SCRATCH_PAD_REG_3   ((uint8_t)2U)
#define PMIC_SCRATCH_PAD_REG_4   ((uint8_t)3U)
#define PMIC_SCRATCH_PAD_REG_MAX (PMIC_SCRATCH_PAD_REG_4)
/** @} */

/**
 * @anchor Pmic_CoreLockControl
 * @name PMIC Lock Control Enumeration for use with `Pmic_{get,set}RegLockState()`
 *
 * @{
 */
#define PMIC_LOCK_DISABLE ((bool)false)
#define PMIC_LOCK_ENABLE  ((bool)true)
/** @} */

/**
 * @anchor Pmic_ConfigCrcRecalculate
 * @name PMIC Config CRC recalculate value
 *
 * @{
 */
#define PMIC_CFG_CRC_RECALCULATE ((bool)true)
#define PMIC_CFG_CRC_ENABLE_ONLY ((bool)false)
/** @} */

/**
 * @brief Write a value to a target scratch pad register on the PMIC.
 *
 * @param handle           [IN] PMIC interface handle.
 * @param scratchPadRegNum [IN] Target scratch pad register number.
 * @param value            [IN] Value to be written to scratch pad register.
 *
 * @return Success code if value has been written to PMIC scratch pad register,
 * error code otherwise. For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_setScratchPadVal(Pmic_CoreHandle_t *handle, uint8_t scratchPadRegNum, uint8_t value);

/**
 * @brief Obtain the value of a scratch pad register on the PMIC.
 *
 * @param handle           [IN] PMIC interface handle.
 * @param scratchPadRegNum [IN] Target scratch pad register number.
 * @param value            [OUT] Scratch pad value obtained from the PMIC.
 *
 * @return Success code if target scratch pad register value has been obtained
 * from the PMIC, error code otherwise. For valid success/error codes, refer to
 * @ref Pmic_errorCodes.
 */
int32_t Pmic_getScratchPadVal(Pmic_CoreHandle_t *handle, uint8_t scratchPadRegNum, uint8_t *value);

/**
 * @brief Set lock state for registers locked by REGISTER_LOCK.
 *
 * @param handle    [IN] Pointer to the PMIC core handle structure.
 * @param lockState [IN] Lock registers with PMIC_LOCK_ENABLE, unlock with
 * PMIC_LOCK_DISABLE. See @ref Pmic_CoreLockControl.
 *
 * @return Returns PMIC_ST_SUCCESS if the operation is successful; otherwise,
 * it returns an appropriate error code. For possible values, see @ref
 * Pmic_ErrorCodes.
 */
int32_t Pmic_setRegLockState(Pmic_CoreHandle_t *handle, bool lockState);

/**
 * @brief Get lock state for registers locked by REGISTER_LOCK.
 *
 * @param handle    [IN]  Pointer to the PMIC core handle structure.
 * @param lockState [OUT] If PMIC_LOCK_ENABLE, registers are locked. If
 * PMIC_LOCK_DISABLE, registers are unlocked. See @ref Pmic_CoreLockControl.
 *
 * @return Returns PMIC_ST_SUCCESS if the operation is successful; otherwise,
 * it returns an appropriate error code. For possible values, see @ref
 * Pmic_ErrorCodes.
 */
int32_t Pmic_getRegLockState(Pmic_CoreHandle_t *handle, bool *lockState);

/**
 * @brief Enables config register CRC.
 *
 * @param handle    [IN] Pointer to the PMIC core handle structure.
 * @param calculate [IN] If true (PMIC_CFG_CRC_RECALCULATE), this API will
 * calculate the new CONFIG_CRC value and write it to CONFIG_CRC_REG_{1,2}
 * before enabling CONFIG_CRC. If false (PMIC_CFG_CRC_ENABLE_ONLY), this API
 * will only enable the CONFIG_CRC feature. See @ref Pmic_ConfigCrcRecalculate.
 *
 * @return Returns PMIC_ST_SUCCESS if the operation is successful; otherwise,
 * it returns an appropriate error code. For possible values, see @ref
 * Pmic_ErrorCodes.
 */
int32_t Pmic_configCrcEnable(Pmic_CoreHandle_t *handle, bool calculate);

/**
 * @brief Disables config register CRC.
 *
 * @param handle    [IN] Pointer to the PMIC core handle structure.
 *
 * @return Returns PMIC_ST_SUCCESS if the operation is successful; otherwise,
 * it returns an appropriate error code. For possible values, see @ref
 * Pmic_ErrorCodes.
 */
int32_t Pmic_configCrcDisable(Pmic_CoreHandle_t *handle);

/**
 * @brief Calculate config register CRC, writes value to PMIC, and verifies
 * correctness.
 *
 * @param handle    [IN] Pointer to the PMIC core handle structure.
 *
 * @return Returns PMIC_ST_SUCCESS if the operation is successful; otherwise,
 * it returns an appropriate error code. For possible values, see @ref
 * Pmic_ErrorCodes.
 */
int32_t Pmic_configCrcCalculate(Pmic_CoreHandle_t *handle);

/**
 * @brief Retrieve the config register CRC value currently stored on the PMIC as
 * a result of continuous CRC calculation. This API does not start a new CRC
 * calculation or affect the CRC value provided by the MCU.
 *
 * @param handle    [IN]  Pointer to the PMIC core handle structure.
 * @param crc       [OUT] The current CRC value on device.
 *
 * @return Returns PMIC_ST_SUCCESS if the operation is successful; otherwise,
 * it returns an appropriate error code. For possible values, see @ref
 * Pmic_ErrorCodes.
 */
int32_t Pmic_configCrcGetFromDevice(Pmic_CoreHandle_t *handle, uint16_t *crc);

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* __PMIC_CORE_H__ */
