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
#ifndef __PMIC_CORE_H__
#define __PMIC_CORE_H__

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

#include "pmic_types.h"

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
#define PMIC_CFG_REG_LOCK_VALID (0U)
#define PMIC_CFG_CNT_LOCK_VALID (1U)
/** @} */

/**
 * @anchor Pmic_CoreLockCfgValidParamShift
 * @name PMIC Lock Control Valid Params Bit Shift Positions
 *
 * @{
 */
#define PMIC_CFG_REG_LOCK_VALID_SHIFT (1U << PMIC_CFG_REG_LOCK_VALID)
#define PMIC_CFG_CNT_LOCK_VALID_SHIFT (1U << PMIC_CFG_CNT_LOCK_VALID)
#define PMIC_CFG_LOCK_ALL_VALID_SHIFT (\
    PMIC_CFG_REG_LOCK_VALID_SHIFT |\
    PMIC_CFG_CNT_LOCK_VALID_SHIFT)
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

/*==========================================================================*/
/*                         Function Declarations                            */
/*==========================================================================*/
/**
 * @ingroup DRV_PMIC_CORE_LOCK_GROUP
 * @brief Set register lock/unlock configuration.
 * This function is responsible for configuring the register lock/unlock
 * settings of the PMIC based on the provided parameters in the commonCtrlCfg
 * structure. The function initiates a critical section to ensure atomicity of
 * the operation. It then sends the register unlock configuration data to the
 * PMIC via the communication interface. Upon successful transmission of the
 * unlock data, it sends the register lock configuration data. Finally, it
 * terminates the critical section.
 *
 * @param handle [IN] Pointer to the PMIC core handle structure.
 * @param config [IN] Pointer to the lock configuration structure containing
 * the register lock/unlock parameters.
 *
 * @return Returns PMIC_ST_SUCCESS if the operation is successful; otherwise,
 * it returns an appropriate error code. For possible values, see @ref
 * Pmic_ErrorCodes.
 */
int32_t Pmic_setLockCfg(Pmic_CoreHandle_t *handle, const Pmic_Lock_t *config);

/**
 * @ingroup DRV_PMIC_CORE_LOCK_GROUP
 * @brief Get register lock status for all lockable registers on PMIC.
 *
 * @param handle Pointer to the PMIC core handle structure.
 * @param config [IN/OUT] Pointer to the lock configuration structure to store
 * the retrieved register lock status.
 *
 * @return Returns PMIC_ST_SUCCESS if the operation is successful; otherwise,
 * it returns an appropriate error code. For possible values, see @ref
 * Pmic_ErrorCodes.
 */
int32_t Pmic_getLockCfg(Pmic_CoreHandle_t *handle, Pmic_Lock_t *config);

/**
 * @ingroup DRV_PMIC_CORE_LOCK_GROUP
 * @brief Set lock state for registers locked by CFG_REG_LOCK.
 *
 * @param handle    [IN] Pointer to the PMIC core handle structure.
 * @param lockState [IN] Lock registers with PMIC_LOCK_ENABLE, unlock with
 * PMIC_LOCK_DISABLE. See @ref Pmic_CoreLockControl.
 *
 * @return Returns PMIC_ST_SUCCESS if the operation is successful; otherwise,
 * it returns an appropriate error code. For possible values, see @ref
 * Pmic_ErrorCodes.
 */
int32_t Pmic_setRegLockState(Pmic_CoreHandle_t *handle, uint8_t lockState);

/**
 * @ingroup DRV_PMIC_CORE_LOCK_GROUP
 * @brief Get lock state for registers locked by CFG_REG_LOCK.
 *
 * @param handle    [IN]  Pointer to the PMIC core handle structure.
 * @param lockState [OUT] If PMIC_LOCK_ENABLE, registers are locked. If
 * PMIC_LOCK_DISABLE, registers are unlocked. See @ref Pmic_CoreLockControl.
 *
 * @return Returns PMIC_ST_SUCCESS if the operation is successful; otherwise,
 * it returns an appropriate error code. For possible values, see @ref
 * Pmic_ErrorCodes.
 */
int32_t Pmic_getRegLockState(Pmic_CoreHandle_t *handle, uint8_t *lockState);

/**
 * @ingroup DRV_PMIC_CORE_LOCK_GROUP
 * @brief Set lock state for registers locked by CNT_REG_LOCK.
 *
 * @param handle    [IN] Pointer to the PMIC core handle structure.
 * @param lockState [IN] Lock registers with PMIC_LOCK_ENABLE, unlock with
 * PMIC_LOCK_DISABLE. See @ref Pmic_CoreLockControl.
 *
 * @return Returns PMIC_ST_SUCCESS if the operation is successful; otherwise,
 * it returns an appropriate error code. For possible values, see @ref
 * Pmic_ErrorCodes.
 */
int32_t Pmic_setCntLockState(Pmic_CoreHandle_t *handle, uint8_t lockState);

/**
 * @ingroup DRV_PMIC_CORE_LOCK_GROUP
 * @brief Get lock state for registers locked by CNT_REG_LOCK.
 *
 * @param handle    [IN]  Pointer to the PMIC core handle structure.
 * @param lockState [OUT] If PMIC_LOCK_ENABLE, registers are locked. If
 * PMIC_LOCK_DISABLE, registers are unlocked. See @ref Pmic_CoreLockControl.
 *
 * @return Returns PMIC_ST_SUCCESS if the operation is successful; otherwise,
 * it returns an appropriate error code. For possible values, see @ref
 * Pmic_ErrorCodes.
 */
int32_t Pmic_getCntLockState(Pmic_CoreHandle_t *handle, uint8_t *lockState);

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* __PMIC_CORE_H__ */
