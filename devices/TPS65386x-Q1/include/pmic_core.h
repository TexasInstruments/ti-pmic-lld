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

/**
 * @defgroup DRV_PMIC_CORE_MUX_GROUP PMIC Multiplexer Control
 * @ingroup DRV_PMIC_CORE_MODULE
 * @brief Control AMUX (Analog Multiplexer) and DMUX (Digital Multiplexer) for diagnostic signal observation.
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
#define PMIC_CFG_REG_LOCK_VALID (1UL << 0U)
#define PMIC_CFG_CNT_LOCK_VALID (1UL << 1U)
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
 * @anchor Pmic_MuxCfgValidParam
 * @name PMIC Multiplexer Configuration Valid Params
 *
 * @brief Valid parameters of the Pmic_MuxCfg_t structure. Set the
 * validParams member of Pmic_MuxCfg_t to indicate which members are
 * to be set/retrieved.
 *
 * @{
 */
#define PMIC_CFG_MUX_MODE_VALID          (1UL << 0U)
#define PMIC_CFG_MUX_AMUX_EN_VALID       (1UL << 1U)
#define PMIC_CFG_MUX_DMUX_EN_VALID       (1UL << 2U)
#define PMIC_CFG_MUX_AMUX_CHANNEL_VALID  (1UL << 3U)
#define PMIC_CFG_MUX_DMUX_GROUP_VALID    (1UL << 4U)
/** @} */

/**
 * @anchor Pmic_MuxCfgValidParamHelpers
 * @name PMIC Multiplexer Configuration Valid Params Helpers
 *
 * @brief Helper macros for common combinations of valid parameters.
 *
 * @{
 */
#define PMIC_CFG_MUX_AMUX_ALL_VALID (PMIC_CFG_MUX_AMUX_EN_VALID | \
                                     PMIC_CFG_MUX_AMUX_CHANNEL_VALID)
#define PMIC_CFG_MUX_DMUX_ALL_VALID (PMIC_CFG_MUX_DMUX_EN_VALID | \
                                     PMIC_CFG_MUX_DMUX_GROUP_VALID)
#define PMIC_CFG_MUX_ALL_VALID      (PMIC_CFG_MUX_MODE_VALID | \
                                     PMIC_CFG_MUX_AMUX_EN_VALID | \
                                     PMIC_CFG_MUX_DMUX_EN_VALID | \
                                     PMIC_CFG_MUX_AMUX_CHANNEL_VALID | \
                                     PMIC_CFG_MUX_DMUX_GROUP_VALID)
/** @} */

/**
 * @anchor Pmic_MuxMode
 * @name PMIC Multiplexer Mode Values
 *
 * @brief Possible values for the muxMode member of Pmic_MuxCfg_t.
 *
 * @{
 */
#define PMIC_MUX_MODE_DISABLED  (0U)
#define PMIC_MUX_MODE_AMUX      (1U)
#define PMIC_MUX_MODE_DMUX      (2U)
/** @} */

/**
 * @anchor Pmic_MuxLimits
 * @name PMIC Multiplexer Channel and Group Limits
 *
 * @brief Maximum values for AMUX channel and DMUX group selection.
 *
 * @{
 */
#define PMIC_MUX_AMUX_CHANNEL_MAX  ((uint8_t)0x1FU)
#define PMIC_MUX_DMUX_GROUP_MAX    ((uint8_t)0x1FU)
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
 * @param cfgLock Configuration Register Lock. Valid only when PMIC_CFG_REG_LOCK_VALID bit is set
 *
 * @param cntLock Timer/Rotation Counter Register Lock configuration. Valid
 * only when PMIC_CFG_CNT_LOCK_VALID bit is set
 */
typedef struct Pmic_Lock_s {
    uint16_t validParams;

    bool cfgLock;
    bool cntLock;
} Pmic_Lock_t;

/**
 * @brief PMIC multiplexer configuration structure.
 *
 * Used to configure and read back the AMUX (Analog Multiplexer) and
 * DMUX (Digital Multiplexer) settings for diagnostic signal observation.
 *
 * @note validParams is an input parameter for all Set and Get APIs. Other
 * struct members are input params for Set APIs and output params for Get APIs.
 *
 * @param validParams Selection of structure parameters to be set, from
 * @ref Pmic_MuxCfgValidParam. OR together multiple bits to set/get
 * multiple parameters in a single call.
 *
 * @param muxMode Multiplexer mode selection. Valid when PMIC_CFG_MUX_MODE_VALID
 * is set. For valid values, see @ref Pmic_MuxMode.
 *
 * @param amuxEnable AMUX (Analog Multiplexer) enable. Valid when
 * PMIC_CFG_MUX_AMUX_EN_VALID is set. Set to true to enable, false to disable.
 *
 * @param dmuxEnable DMUX (Digital Multiplexer) enable. Valid when
 * PMIC_CFG_MUX_DMUX_EN_VALID is set. Set to true to enable, false to disable.
 *
 * @param amuxChannel AMUX channel selection (0-31). Valid when
 * PMIC_CFG_MUX_AMUX_CHANNEL_VALID is set. Selects which analog signal
 * is routed to the AMUX output pin.
 *
 * @param dmuxGroup DMUX group selection (0-31). Valid when
 * PMIC_CFG_MUX_DMUX_GROUP_VALID is set. Selects which digital signal
 * group is routed to the DMUX output pins.
 */
typedef struct Pmic_MuxCfg_s {
    uint32_t validParams;

    uint8_t muxMode;
    bool amuxEnable;
    bool dmuxEnable;
    uint8_t amuxChannel;
    uint8_t dmuxGroup;
} Pmic_MuxCfg_t;

/*==========================================================================*/
/*                         Function Declarations                            */
/*==========================================================================*/

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
 * @return PMIC_ST_SUCCESS if PMIC NVM revision has been obtained, error code
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
 * @return PMIC_ST_SUCCESS if PMIC silicon revision has been obtained, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_getSiliconRev(const Pmic_Handle_t *handle, uint8_t *siliconRev);

/**
 * @ingroup DRV_PMIC_CORE_LOCK_GROUP
 * @brief Set register lock and counter lock configurations. This API is a superset
 * of `Pmic_setRegLockState()` and `Pmic_setCntLockState()`.
 *
 * Design: PMICDRV-585
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-521, PMICDRV-522, PMICDRV-523, PMICDRV-545
 *               PMICDRV-546
 *
 * @param handle [IN] Pointer to the PMIC core handle structure.
 * @param config [IN] Pointer to the lock configuration structure containing
 * the register lock/unlock parameters.
 *
 * @return Returns PMIC_ST_SUCCESS if the operation is successful; otherwise,
 * it returns an appropriate error code. For possible values, see @ref
 * Pmic_ErrorCodes.
 */
int32_t Pmic_setLockCfg(const Pmic_Handle_t *handle, const Pmic_Lock_t *config);

/**
 * @ingroup DRV_PMIC_CORE_LOCK_GROUP
 * @brief Get register lock and counter lock configurations. This API is a superset
 * of `Pmic_getRegLockState()` and `Pmic_getCntLockState()`.
 *
 * Design: PMICDRV-586
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-521, PMICDRV-522, PMICDRV-528, PMICDRV-545
 *               PMICDRV-546
 *
 * @param handle Pointer to the PMIC core handle structure.
 * @param config [IN/OUT] Pointer to the lock configuration structure to store
 * the retrieved register lock status.
 *
 * @return Returns PMIC_ST_SUCCESS if the operation is successful; otherwise,
 * it returns an appropriate error code. For possible values, see @ref
 * Pmic_ErrorCodes.
 */
int32_t Pmic_getLockCfg(const Pmic_Handle_t *handle, Pmic_Lock_t *config);

/**
 * @ingroup DRV_PMIC_CORE_LOCK_GROUP
 * @brief Lock/unlock registers that are locked by CFG_REG_LOCK. This API is a
 * subset of `Pmic_setLockCfg()`.
 *
 * Design: PMICDRV-587
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-508, PMICDRV-521, PMICDRV-522,
 *               PMICDRV-523, PMICDRV-545, PMICDRV-546
 *
 * @param handle    [IN] Pointer to the PMIC core handle structure.
 * @param lockState [IN] Lock registers with PMIC_LOCK_ENABLE, unlock with
 * PMIC_LOCK_DISABLE. See @ref Pmic_CoreLockControl.
 *
 * @return Returns PMIC_ST_SUCCESS if the operation is successful; otherwise,
 * it returns an appropriate error code. For possible values, see @ref
 * Pmic_ErrorCodes.
 */
int32_t Pmic_setRegLockState(const Pmic_Handle_t *handle, bool lockState);

/**
 * @ingroup DRV_PMIC_CORE_LOCK_GROUP
 * @brief Get lock state for registers locked by CFG_REG_LOCK. This API is a subset
 * of `Pmic_getLockCfg()`.
 *
 * Design: PMICDRV-588
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-508, PMICDRV-521, PMICDRV-522,
 *               PMICDRV-528, PMICDRV-545, PMICDRV-546
 *
 * @param handle    [IN]  Pointer to the PMIC core handle structure.
 * @param lockState [OUT] If PMIC_LOCK_ENABLE, registers are locked. If
 * PMIC_LOCK_DISABLE, registers are unlocked. See @ref Pmic_CoreLockControl.
 *
 * @return Returns PMIC_ST_SUCCESS if the operation is successful; otherwise,
 * it returns an appropriate error code. For possible values, see @ref
 * Pmic_ErrorCodes.
 */
int32_t Pmic_getRegLockState(const Pmic_Handle_t *handle, uint8_t *lockState);

/**
 * @ingroup DRV_PMIC_CORE_LOCK_GROUP
 * @brief Lock/unlock registers that are locked by CNT_REG_LOCK. This API is a
 * subset of `Pmic_setLockCfg()`.
 *
 * Design: PMICDRV-589
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-508, PMICDRV-521, PMICDRV-522,
 *               PMICDRV-523, PMICDRV-545, PMICDRV-546
 *
 * @param handle    [IN] Pointer to the PMIC core handle structure.
 * @param lockState [IN] Lock registers with PMIC_LOCK_ENABLE, unlock with
 * PMIC_LOCK_DISABLE. See @ref Pmic_CoreLockControl.
 *
 * @return Returns PMIC_ST_SUCCESS if the operation is successful; otherwise,
 * it returns an appropriate error code. For possible values, see @ref
 * Pmic_ErrorCodes.
 */
int32_t Pmic_setCntLockState(const Pmic_Handle_t *handle, uint8_t lockState);

/**
 * @ingroup DRV_PMIC_CORE_LOCK_GROUP
 * @brief Get lock state for registers locked by CNT_REG_LOCK. This API is a subset
 * of `Pmic_getLockCfg()`.
 *
 * Design: PMICDRV-590
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-508, PMICDRV-521, PMICDRV-522,
 *               PMICDRV-528, PMICDRV-545, PMICDRV-546
 *
 * @param handle    [IN]  Pointer to the PMIC core handle structure.
 * @param lockState [OUT] If PMIC_LOCK_ENABLE, registers are locked. If
 * PMIC_LOCK_DISABLE, registers are unlocked. See @ref Pmic_CoreLockControl.
 *
 * @return Returns PMIC_ST_SUCCESS if the operation is successful; otherwise,
 * it returns an appropriate error code. For possible values, see @ref
 * Pmic_ErrorCodes.
 */
int32_t Pmic_getCntLockState(const Pmic_Handle_t *handle, uint8_t *lockState);

/**
 * @brief Write a value to a target scratch pad register on the PMIC.
 *
 * Design: PMICDRV-591
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-508, PMICDRV-511, PMICDRV-512,
 *               PMICDRV-515, PMICDRV-516, PMICDRV-521, PMICDRV-522, PMICDRV-523,
 *               PMICDRV-527, PMICDRV-545, PMICDRV-551
 *
 * @param handle           [IN] PMIC interface handle.
 * @param scratchPadRegNum [IN] Target scratch pad register number.
 * @param value            [IN] Value to be written to scratch pad register.
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
 * @param handle           [IN] PMIC interface handle.
 * @param scratchPadRegNum [IN] Target scratch pad register number.
 * @param value            [OUT] Scratch pad value obtained from the PMIC.
 *
 * @return Success code if target scratch pad register value has been obtained
 * from the PMIC, error code otherwise. For valid success/error codes, refer to
 * @ref Pmic_ErrorCodes.
 */
int32_t Pmic_getScratchPadValue(const Pmic_Handle_t *handle, uint8_t scratchPadRegNum, uint8_t *value);

/**
 * @ingroup DRV_PMIC_CORE_MUX_GROUP
 * @brief Set PMIC multiplexer configuration.
 *
 * Design: PMICDRV-789
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-521, PMICDRV-522,
 *               PMICDRV-523, PMICDRV-547
 *
 * @details This API configures the AMUX (Analog Multiplexer) and DMUX
 * (Digital Multiplexer) for diagnostic signal observation. The validParams
 * member of the configuration structure determines which settings are updated.
 * You can update a single parameter, multiple parameters, or all parameters
 * in one call using the helper macros (PMIC_CFG_MUX_AMUX_ALL_VALID, etc.).
 *
 * @param handle [IN] PMIC interface handle.
 * @param config [IN] Multiplexer configuration. Use validParams to select
 * which members to set.
 *
 * @return PMIC_ST_SUCCESS if successful, error code otherwise.
 * For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_setMuxCfg(const Pmic_Handle_t *handle, const Pmic_MuxCfg_t *config);

/**
 * @ingroup DRV_PMIC_CORE_MUX_GROUP
 * @brief Get PMIC multiplexer configuration.
 *
 * Design: PMICDRV-790
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-521, PMICDRV-522,
 *               PMICDRV-528, PMICDRV-547
 *
 * @details This API retrieves the current AMUX and DMUX configuration from
 * the PMIC. The validParams member of the configuration structure determines
 * which settings are read. Use PMIC_CFG_MUX_ALL_VALID to read all settings.
 *
 * @param handle [IN] PMIC interface handle.
 * @param config [IN/OUT] Multiplexer configuration. Set validParams to
 * indicate which members to retrieve. The selected members will be populated
 * with current PMIC values.
 *
 * @return PMIC_ST_SUCCESS if successful, error code otherwise.
 * For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_getMuxCfg(const Pmic_Handle_t *handle, Pmic_MuxCfg_t *config);

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* PMIC_CORE_H */
