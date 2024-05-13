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

#ifndef __PMIC_CORE_H__
#define __PMIC_CORE_H__

#include <stdbool.h>
#include <stdint.h>

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include "pmic_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup Pmic_Core PMIC Core
 * @{
 * @brief Contains definitions related to PMIC core functionality.
 */

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/**
 * @brief Silicon Revision ID for Power Group 2.0.
 */
#define PMIC_SILICON_REV_ID_PG_2_0 (0x08U)

/**
 * @brief Silicon Revision ID for Power Group 1.0.
 */
#define PMIC_SILICON_REV_ID_PG_1_0 (0x0U)

/**
 * @brief validParams value used to get Register lock status
 */
#define PMIC_CFG_REG_LOCK_STAT_VALID (1U)

/**
 * @brief Unlock data 1 for register and timer counter.
 */
#define PMIC_REGISTER_UNLOCK_DATA1 (0x98U)

/**
 * @brief Unlock data 2 for register and timer counter.
 */
#define PMIC_REGISTER_UNLOCK_DATA2 (0xB8U)

/**
 * @brief Unlock data 1 for timer and rotational counter.
 */
#define PMIC_TMR_COUNTER_UNLOCK_DATA1 (0x13U)

/**
 * @brief Unlock data 2 for timer and rotational counter.
 */
#define PMIC_TMR_COUNTER_UNLOCK_DATA2 (0x7DU)

/**
 * @brief Lock data for register.
 */
#define PMIC_REGISTER_LOCK_1 (0x10U)

/**
 * @brief Lock data for timer counter.
 */
#define PMIC_REGISTER_LOCK_2 (0x10U)

/**
 * @brief Register status indicating unlocked state.
 */
#define PMIC_REGISTER_STATUS_UNLOCK (0x0U)

/**
 * @brief Register status indicating locked state.
 */
#define PMIC_REGISTER_STATUS_LOCK (0x1U)

/**
 * @brief Configuration for CRC (Cyclic Redundancy Check) status when disabled.
 */
#define PMIC_CRC_STATUS_DISABLED (0U)

/**
 * @brief Configuration for CRC status when enabled.
 */
#define PMIC_CRC_STATUS_ENABLED (1U)

/**
 * @brief Validity of Register Lock configuration.
 */
#define PMIC_CFG_REG_LOCK_VALID (2U)

/**
 * @brief Shift value for the validity of Register Lock configuration.
 */
#define PMIC_CFG_REG_LOCK_VALID_SHIFT (1U << PMIC_CFG_REG_LOCK_VALID)

/*==========================================================================*/
/*                         Structures and Enums                             */
/*==========================================================================*/

/**
 * @defgroup Pmic_CoreStructures PMIC Core Structures
 * @{
 * @ingroup Pmic_Core
 * @brief Contains structures used in the core module of PMIC driver.
 */

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
 *  @param  regLock                 Register Lock configuration
 *                                  Valid values @ref Pmic_RegisterLock_Config
 *                                  Valid only when PMIC_CFG_REG_LOCK_VALID
 *                                  bit is set
 *                                  Valid only for Pmic_setCommonCtrlConfig API
 *  @param  regLock_1               Lock/Unlock sequence-1 for configuration
 * register lock/unlock.
 *  @param  regLock_2               Lock/Unlock sequence-2 for configuration
 * register lock/unlock.
 *  @param  cntLock_1               Lock/Unlock sequence-1 for timer and
 * rotation counter register lock/unlock.
 *  @param  cntLock_2               Lock/Unlock sequence-2 for timer and
 * rotation counter register lock/unlock.
 *
 * @ingroup Pmic_CoreStructures
 */
typedef struct Pmic_CommonCtrlCfg_s {
    uint32_t validParams;
    uint8_t regLock;
    uint8_t regLock_1;
    uint8_t regLock_2;
    uint8_t cntLock_1;
    uint8_t cntLock_2;
} Pmic_CommonCtrlCfg_t;

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
 *  @param  regLockStat        Register lock status
 *                             Valid only when PMIC_CFG_REG_LOCK_STAT_VALID
 *                             bit is set
 *                                 Valid values @ref Pmic_RegisterLock_Stat
 *  @param  cfgregLockStat    Lock status of configuration registers
 *  @param  cntregLockStat
 *
 * @ingroup Pmic_CoreStructures
 */
typedef struct Pmic_CommonCtrlStat_s {
    uint32_t validParams;
    uint8_t regLockStat;
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
 *
 * @ingroup Pmic_CoreStructures
 */
typedef struct Pmic_DeviceInfo_s {
    uint8_t deviceID;
    uint8_t nvmID;
    uint8_t nvmRev;
    uint8_t siliconRev;
    uint8_t customNvmID;
} Pmic_DeviceInfo_t;

/**
 * @}
 */ /* End of Pmic_CoreStructures */

/*==========================================================================*/
/*                         Function Declarations                            */
/*==========================================================================*/

/**
 * @defgroup Pmic_CoreFunctions PMIC Core Functions
 * @{
 * @ingroup Pmic_Core
 * @brief Contains functions used in the core module of PMIC driver.
 */

/**
 * @brief Set register lock/unlock configuration.
 * This function is responsible for configuring the register lock/unlock
 * settings of the PMIC based on the provided parameters in the commonCtrlCfg
 * structure. The function initiates a critical section to ensure atomicity of
 * the operation. It then sends the register unlock configuration data to the
 * PMIC via the communication interface. Upon successful transmission of the
 * unlock data, it sends the register lock configuration data. Finally, it
 * terminates the critical section.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle structure.
 * @param commonCtrlCfg Common control configuration structure containing the
 * register lock/unlock parameters.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 *                      otherwise, it returns an appropriate error code.
 *
 * @ingroup Pmic_CoreFunctions
 */
int32_t Pmic_setRegisterLockUnlock(Pmic_CoreHandle_t *pPmicCoreHandle,
                                   const Pmic_CommonCtrlCfg_t commonCtrlCfg);

/**
 * @brief Get register lock status.
 * This function retrieves the current register lock status from the PMIC.
 * It reads the register lock status byte from the PMIC, extracts the lock
 * status bit, and stores it in the provided common control status structure.
 * The function starts and ends a critical section for PMIC operations.
 * If the operation is successful, PMIC_ST_SUCCESS is returned; otherwise, an
 * error code indicating the failure is returned.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle structure.
 *                        It holds necessary information required to operate on
 * the PMIC.
 * @param pCommonCtrlStat Pointer to the common control status structure to
 * store the retrieved register lock status.
 * @return int32_t Returns PMIC_ST_SUCCESS if the operation is successful;
 *                 otherwise, returns an error code.
 *
 * @ingroup Pmic_CoreFunctions
 */
int32_t Pmic_getRegLockStat(Pmic_CoreHandle_t *pPmicCoreHandle,
                            Pmic_CommonCtrlStat_t *pCommonCtrlStat);

/**
 * @}
 */ /* End of Pmic_CoreFunctions */

/**
 * @}
 */ /* End of Pmic_Core */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __PMIC_CORE_H__ */
