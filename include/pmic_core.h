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
 * @anchor Pmic_drvInitValues
 * @name PMIC LLD Initialization Values
 *
 * @brief Indication of whether the driver is initialized or not.
 *
 * @details The value is used to help prevent corrupt PMIC handle usage. When
 * converting \p PMIC_DRV_INIT to ASCII, the value reads out to be "PMIC".
 *
 * @{
 */
#define PMIC_DRV_INIT       ((uint32_t)0x504D4943U)
#define PMIC_DRV_UNINIT     ((uint32_t)0U)

/**
 * @anchor Pmic_regLockUnlockValues
 * @name TPS65385x Register Lock/Unlock Values
 *
 * @brief Values to be passed into the `lock` parameter of the Pmic_setRegLock()
 * API.
 *
 * @{
 */
#define PMIC_LOCK           ((bool)true)
#define PMIC_UNLOCK         ((bool)false)
/** @} */

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

#ifdef __cplusplus
}

#endif /* __cplusplus */
#endif /* __PMIC_CORE_H__ */
