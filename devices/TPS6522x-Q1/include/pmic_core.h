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

/* ========================================================================== */
/*                            Macros & Typedefs                               */
/* ========================================================================== */

/**
 * @anchor Pmic_ScratchpadRegs
 * @name PMIC Scratchpad Registers
 *
 * @brief Enumeration of PMIC scratchpad registers.
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

/* ========================================================================== */
/*                           Structures and Enums                             */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 * @brief Enable or disable PMIC register lock.
 *
 * Design: PMICDRV-587
 * Architecture: PMICDRV-507, PMICDRV-516, PMICDRV-508, PMICDRV-523, PMICDRV-547, PMICDRV-549
 *               PMICDRV-550, PMICDRV-551, PMICDRV-545, PMICDRV-546, PMICDRV-506, PMICDRV-504
 *               PMICDRV-522, PMICDRV-521, PMICDRV-512
 *
 * @param handle Pointer to the PMIC handle.
 *
 * @param lock True to lock the registers, false to unlock.
 *
 * @return PMIC_ST_SUCCESS if PMIC registers have been locked/unlocked, error
 * code otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_setRegLockState(Pmic_Handle_t *handle, bool lock);

/**
 * @brief Get PMIC register enable state.
 *
 * Design: PMICDRV-588
 * Architecture: PMICDRV-507, PMICDRV-516, PMICDRV-508, PMICDRV-547, PMICDRV-549, PMICDRV-550
 *               PMICDRV-551, PMICDRV-545, PMICDRV-546, PMICDRV-506, PMICDRV-504, PMICDRV-522
 *               PMICDRV-528, PMICDRV-521, PMICDRV-512
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param isLocked [OUT] Register lock state obtained from the PMIC.
 *
 * @return PMIC_ST_SUCCESS if PMIC register lock state has been obtained,
 * error code otherwise. For valid success/error codes, refer to
 * @ref Pmic_ErrorCodes.
 */
int32_t Pmic_getRegLockState(Pmic_Handle_t *handle, bool *isLocked);

/**
 * @brief Set the value of a PMIC scratchpad register.
 *
 * Design: PMICDRV-684
 * Architecture: PMICDRV-507, PMICDRV-516, PMICDRV-508, PMICDRV-523, PMICDRV-547, PMICDRV-549
 *               PMICDRV-550, PMICDRV-551, PMICDRV-545, PMICDRV-546, PMICDRV-506, PMICDRV-504
 *               PMICDRV-522, PMICDRV-521, PMICDRV-512
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @param scratchpadRegNum [IN] Scratchpad register number. For valid
 * scratchpad register numbers, refer to @ref Pmic_ScratchpadRegs.
 *
 * @param value [IN] Value to be written to the scratchpad register.
 *
 * @return PMIC_ST_SUCCESS if the value has been written successfully,
 * error code otherwise. For valid success/error codes, refer to
 * @ref Pmic_ErrorCodes.
 */
int32_t Pmic_setScratchPadValue(const Pmic_Handle_t *pmicHandle, uint8_t scratchpadRegNum, uint8_t value);

/**
 * @brief Get the value of a PMIC scratchpad register.
 *
 * Design: PMICDRV-685
 * Architecture: PMICDRV-507, PMICDRV-516, PMICDRV-508, PMICDRV-547, PMICDRV-549, PMICDRV-550
 *               PMICDRV-551, PMICDRV-545, PMICDRV-546, PMICDRV-506, PMICDRV-504, PMICDRV-522
 *               PMICDRV-528, PMICDRV-521, PMICDRV-512
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @param scratchpadRegNum [IN] Scratchpad register number. For valid
 * scratchpad register numbers, refer to @ref Pmic_ScratchpadRegs.
 *
 * @param value [OUT] Scratchpad register value obtained from the PMIC.
 *
 * @return PMIC_ST_SUCCESS if the value has been read successfully,
 * error code otherwise. For valid success/error codes, refer to
 * @ref Pmic_ErrorCodes.
 */
int32_t Pmic_getScratchPadValue(const Pmic_Handle_t *pmicHandle, uint8_t scratchpadRegNum, uint8_t *value);

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* PMIC_CORE_H */
